/**
 * @file main.c
 * @brief Control modular de tostadora de café sobre ESP-IDF y FreeRTOS.
 *
 * @details El sistema separa botones, MAX6675, RTC, FSM, actuadores, LCD y
 * registro SD en tareas dedicadas. Las tareas se comunican mediante colas.
 * El driver SPI serializa el acceso al bus compartido por MAX6675 y SDSPI;
 * LCD, RTC y FatFS tienen un único propietario, por lo que no requieren
 * mutexes adicionales.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/sdspi_host.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_private/esp_clk.h"
#include "soc/rtc.h"

/** @brief Etiqueta de registro general del sistema. */
static const char *TAG = "COFFEE_ROASTER";

/** @name Pines y parámetros de hardware
 *  @{
 */
/** @brief Pin SDA del DS3231. */
#define I2C_SDA_PIN GPIO_NUM_21
/** @brief Pin SCL del DS3231. */
#define I2C_SCL_PIN GPIO_NUM_22
/** @brief Dirección I2C del DS3231. */
#define DS3231_I2C_ADDR 0x68
/** @brief Timeout de transacción I2C en milisegundos. */
#define I2C_TIMEOUT_MS 50

/** @brief Pines del bus SPI2 compartido por MAX6675 y MicroSD. */
#define SPI_MISO_PIN GPIO_NUM_19
#define SPI_CLK_PIN GPIO_NUM_18
#define SPI_MOSI_PIN GPIO_NUM_23
/** @brief Chip-select del MAX6675. */
#define MAX6675_CS_PIN GPIO_NUM_4
/** @brief Chip-select de la tarjeta MicroSD, administrado por SDSPI. */
#define SD_CS_PIN GPIO_NUM_5
/** @brief Frecuencia máxima SDSPI para cableado de protoboard. */
#define SD_SPI_MAX_FREQ_KHZ 5000
/** @brief Punto de montaje FatFS. */
#define SD_MOUNT_POINT "/sdcard"
/** @brief Ruta del archivo CSV de telemetría. */
#define SD_LOG_PATH "/sdcard/tueste.csv"

/** @brief Pines del LCD paralelo 16x2 en modo de 4 bits. */
#define LCD_RS_PIN GPIO_NUM_25
#define LCD_E_PIN GPIO_NUM_26
#define LCD_D4_PIN GPIO_NUM_27
#define LCD_D5_PIN GPIO_NUM_14
#define LCD_D6_PIN GPIO_NUM_16
#define LCD_D7_PIN GPIO_NUM_13

/** @brief Pines de selección de perfil (34/35 requieren pull-up externo). */
#define BUTTON_PROFILE_1_PIN GPIO_NUM_34
#define BUTTON_PROFILE_2_PIN GPIO_NUM_35
#define BUTTON_PROFILE_3_PIN GPIO_NUM_32

/** @brief Salidas de calefactor, agitador, ventilador y buzzer activo. */
#define HEATER_RELAY_PIN GPIO_NUM_33
#define AGITATOR_MOTOR_PIN GPIO_NUM_2
#define COOLING_FAN_PIN GPIO_NUM_15
#define BUZZER_PIN GPIO_NUM_17

/** @brief Nivel activo del SSR Active LOW. */
#define RELAY_ON 0
/** @brief Nivel inactivo del SSR Active LOW. */
#define RELAY_OFF 1
/** @brief Histéresis de control en grados Celsius. */
#define TEMP_HYSTERESIS_C 2.0f
/** @brief Objetivos actuales de los perfiles 1, 2 y 3 en grados Celsius. */
static const float PROFILE_TARGETS_C[3] = {100.0f, 120.0f, 140.0f};
/** @brief Duración de un tueste en segundos. */
#define ROAST_DURATION_SEC 120

/** @} */

/** @name Temporización y tamaños de cola
 *  @{
 */
/** @brief Período de lectura de botones, en milisegundos. */
#define BUTTON_PERIOD_MS 50
/** @brief Período de lectura MAX6675, en milisegundos. */
#define TEMP_SENSOR_PERIOD_MS 500
/** @brief Período de lectura DS3231, en milisegundos. */
#define RTC_PERIOD_MS 1000
/** @brief Período de ejecución de la FSM, en milisegundos. */
#define CONTROL_FSM_PERIOD_MS 100
/** @brief Período máximo de refresco del LCD, en milisegundos. */
#define DISPLAY_REFRESH_PERIOD_MS 250
/** @brief Período de registro de telemetría en SD, en milisegundos. */
#define SD_LOG_PERIOD_MS 1000
/** @brief Pulsación mínima para solicitar una parada de emergencia. */
#define EMERGENCY_HOLD_MS 1000
/** @brief Tiempo máximo sin muestra térmica válida durante tueste. */
#define TEMP_STALE_TIMEOUT_MS 1500
/** @brief Profundidad de la cola de eventos de botones. */
#define BUTTON_QUEUE_LENGTH 8
/** @brief Profundidad de colas latest-value de sensores. */
#define SENSOR_QUEUE_LENGTH 1
/** @brief Profundidad de la cola de comandos de actuadores. */
#define ACTUATOR_QUEUE_LENGTH 8
/** @brief Profundidad de la cola latest-value del LCD. */
#define DISPLAY_QUEUE_LENGTH 1
/** @brief Profundidad de la cola de registro SD. */
#define SD_QUEUE_LENGTH 16
/** @} */

/** @name Tipos de datos e IPC
 *  @{
 */

/** @brief Estados de la máquina de estados de control. */
typedef enum {
    FSM_STATE_IDLE = 0,       /**< Menú principal, sin tueste activo. */
    FSM_STATE_ROAST_ACTIVE,   /**< Perfil de tueste en ejecución. */
    FSM_STATE_EMERGENCY       /**< Parada de emergencia y enfriamiento. */
} ControlState_t;

/** @brief Clases de evento producidas por la tarea de botones. */
typedef enum {
    BUTTON_EVENT_SHORT_PRESS = 0, /**< Toque corto de selección/confirmación. */
    BUTTON_EVENT_EMERGENCY_STOP  /**< Pulsación sostenida al menos un segundo. */
} ButtonEventType_t;

/** @brief Acciones solicitadas al controlador de buzzer de actuadores. */
typedef enum {
    BUZZER_REQUEST_KEEP = 0, /**< Mantener el patrón actual. */
    BUZZER_REQUEST_STOP,     /**< Apagar y cancelar cualquier patrón. */
    BUZZER_REQUEST_PATTERN,  /**< Reproducir un número finito de pitidos. */
    BUZZER_REQUEST_EMERGENCY /**< Repetir 80 ms encendido y 920 ms apagado. */
} BuzzerRequest_t;

/**
 * @brief Telemetría unificada creada por la FSM.
 */
typedef struct {
    char time_str[16];             /**< Hora HH:MM:SS. */
    char date_str[16];             /**< Fecha DD/MM/YYYY. */
    float temperature_c;           /**< Temperatura de termocupla. */
    esp_err_t temp_status;         /**< Resultado de lectura MAX6675. */
    esp_err_t rtc_status;          /**< Resultado de lectura DS3231. */
    bool heater_on;                /**< Estado lógico del SSR. */
    bool fan_on;                   /**< Estado del ventilador. */
    bool emergency_active;         /**< Bandera de alerta visual/audible. */
    ControlState_t state;           /**< Estado actual de la FSM. */
    int32_t selected_profile;      /**< Perfil activo (0 si no hay tueste). */
    float target_temperature_c;    /**< Temperatura objetivo. */
    int32_t remaining_seconds;     /**< Tiempo restante del perfil. */
} Telemetry_t;

/**
 * @brief Mensaje de selección de perfil o parada de emergencia.
 */
typedef struct {
    ButtonEventType_t type; /**< Tipo de evento. */
    uint8_t profile;        /**< Perfil 1-3 asociado al toque corto. */
} ButtonEvent_t;

/**
 * @brief Lectura del MAX6675 y estado de la sonda.
 */
typedef struct {
    esp_err_t status;      /**< ESP_OK si la lectura fue válida. */
    float temperature_c;   /**< Temperatura medida en grados Celsius. */
} TemperatureMessage_t;

/**
 * @brief Mensaje de reloj procedente del DS3231.
 */
typedef struct {
    esp_err_t status; /**< Estado de la lectura del RTC. */
    char time_str[16]; /**< Hora HH:MM:SS. */
    char date_str[16]; /**< Fecha DD/MM/YYYY. */
} TimeMessage_t;

/**
 * @brief Comando completo de salidas para la tarea de actuadores.
 */
typedef struct {
    bool heater_on;                 /**< Estado solicitado del SSR. */
    bool motor_on;                  /**< Estado solicitado del agitador. */
    bool fan_on;                    /**< Estado solicitado del ventilador. */
    BuzzerRequest_t buzzer_request; /**< Acción solicitada para el buzzer. */
    uint32_t beep_duration_ms;      /**< Duración de cada pitido finito. */
    uint8_t beep_count;             /**< Número de pitidos finitos. */
} ActuatorCommand_t;

/** @brief Cola de eventos de pulsación y parada de emergencia. */
static QueueHandle_t xButtonQueue;
/** @brief Cola de una posición con la lectura más reciente del MAX6675. */
static QueueHandle_t xTempQueue;
/** @brief Cola de una posición con la hora más reciente del RTC. */
static QueueHandle_t xTimeQueue;
/** @brief Cola de comandos de los actuadores. */
static QueueHandle_t xActuatorQueue;
/** @brief Cola de una posición con la telemetría más reciente para LCD. */
static QueueHandle_t xDisplayQueue;
/** @brief Cola de muestras pendientes de registrar en SD. */
static QueueHandle_t xSDQueue;

/** @brief Manejador del bus I2C usado exclusivamente por la tarea RTC. */
static i2c_master_bus_handle_t rtc_bus_handle;
/** @brief Manejador del DS3231. */
static i2c_master_dev_handle_t rtc_device_handle;
/** @brief Manejador SPI del MAX6675. */
static spi_device_handle_t max6675_device_handle;
/** @brief Estado de montaje SD; accedido únicamente por la tarea logger. */
static bool sd_card_mounted;

/** @} */

/** @name Prototipos de tareas
 *  @{
 */
static void vTaskButtons(void *pvParameters);
static void vTaskTempSensor(void *pvParameters);
static void vTaskRTCClock(void *pvParameters);
static void vTaskControlFSM(void *pvParameters);
static void vTaskActuators(void *pvParameters);
static void vTaskLCDDisplay(void *pvParameters);
static void vTaskSDLogger(void *pvParameters);
/** @} */

/** @name Conversión y acceso RTC
 *  @{
 */

/**
 * @brief Convierte un valor BCD a decimal.
 * @param value Byte codificado en BCD.
 * @return Valor decimal de ocho bits.
 */
static uint8_t bcd_to_decimal(uint8_t value)
{
    return (uint8_t)(((value >> 4) * 10U) + (value & 0x0FU));
}

/**
 * @brief Convierte un valor decimal a BCD.
 * @param value Valor decimal de ocho bits.
 * @return Byte codificado en BCD.
 */
static uint8_t decimal_to_bcd(uint8_t value)
{
    return (uint8_t)(((value / 10U) << 4) | (value % 10U));
}

/**
 * @brief Convierte la abreviatura inglesa de mes a número de mes.
 * @param month_text Cadena de tres letras de la macro __DATE__.
 * @return Mes entre 1 y 12.
 */
static uint8_t parse_month(const char *month_text)
{
    static const char *const months[] = {
        "Jan", "Feb", "Mar", "Apr", "May", "Jun",
        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    };

    for (uint8_t index = 0; index < 12; index++) {
        if (strncmp(month_text, months[index], 3) == 0) {
            return (uint8_t)(index + 1U);
        }
    }
    return 1;
}

/**
 * @brief Sincroniza el DS3231 con la fecha de compilación si el OSF está activo.
 * @param device_handle Manejador del DS3231.
 * @param force_reset Fuerza la escritura de la hora de compilación.
 * @return ESP_OK en éxito o código de error I2C.
 */
static esp_err_t ds3231_init_or_update(i2c_master_dev_handle_t device_handle,
                                       bool force_reset)
{
    if (device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t status_register = 0x0F;
    uint8_t status_value = 0;
    esp_err_t result = i2c_master_transmit_receive(
        device_handle, &status_register, 1, &status_value, 1, I2C_TIMEOUT_MS);
    if (result != ESP_OK) {
        return result;
    }

    if (force_reset || ((status_value & 0x80U) != 0U)) {
        char month_text[4] = {0};
        int day = 1;
        int year = 2000;
        int hour = 0;
        int minute = 0;
        int second = 0;
        (void)sscanf(__DATE__, "%3s %d %d", month_text, &day, &year);
        (void)sscanf(__TIME__, "%d:%d:%d", &hour, &minute, &second);

        uint8_t write_buffer[8] = {
            0x00,
            decimal_to_bcd((uint8_t)second),
            decimal_to_bcd((uint8_t)minute),
            decimal_to_bcd((uint8_t)hour),
            1,
            decimal_to_bcd((uint8_t)day),
            decimal_to_bcd(parse_month(month_text)),
            decimal_to_bcd((uint8_t)(year % 100))
        };

        result = i2c_master_transmit(device_handle, write_buffer,
                                     sizeof(write_buffer), I2C_TIMEOUT_MS);
        if (result != ESP_OK) {
            return result;
        }

        const uint8_t clear_osf[] = {0x0F, 0x00};
        return i2c_master_transmit(device_handle, clear_osf,
                                   sizeof(clear_osf), I2C_TIMEOUT_MS);
    }
    return ESP_OK;
}

/**
 * @brief Lee la fecha y hora del DS3231 y las convierte a struct tm.
 * @param device_handle Manejador del DS3231.
 * @param time_info Puntero donde se almacena la fecha/hora.
 * @return ESP_OK en éxito o código de error I2C.
 */
static esp_err_t ds3231_get_time(i2c_master_dev_handle_t device_handle,
                                 struct tm *time_info)
{
    if (device_handle == NULL || time_info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t register_address = 0;
    uint8_t data[7] = {0};
    esp_err_t result = i2c_master_transmit_receive(
        device_handle, &register_address, 1, data, sizeof(data), I2C_TIMEOUT_MS);
    if (result != ESP_OK) {
        return result;
    }

    time_info->tm_sec = bcd_to_decimal(data[0] & 0x7F);
    time_info->tm_min = bcd_to_decimal(data[1] & 0x7F);
    time_info->tm_hour = bcd_to_decimal(data[2] & 0x3F);
    time_info->tm_mday = bcd_to_decimal(data[4] & 0x3F);
    time_info->tm_mon = (int)bcd_to_decimal(data[5] & 0x1F) - 1;
    time_info->tm_year = (int)bcd_to_decimal(data[6]) + 100;
    return ESP_OK;
}

/** @} */

/** @name LCD paralelo de 16x2
 *  @{
 */

/**
 * @brief Pulsa la entrada Enable del LCD.
 * @return No devuelve un valor.
 */
static void lcd_pulse_enable(void)
{
    (void)gpio_set_level(LCD_E_PIN, 1);
    esp_rom_delay_us(50);
    (void)gpio_set_level(LCD_E_PIN, 0);
    esp_rom_delay_us(100);
}

/**
 * @brief Escribe un nibble en las líneas D4-D7 y pulsa Enable.
 * @param nibble Nibble bajo que se transmitirá.
 * @return No devuelve un valor.
 */
static void lcd_write_nibble(uint8_t nibble)
{
    (void)gpio_set_level(LCD_D4_PIN, (nibble >> 0) & 1U);
    (void)gpio_set_level(LCD_D5_PIN, (nibble >> 1) & 1U);
    (void)gpio_set_level(LCD_D6_PIN, (nibble >> 2) & 1U);
    (void)gpio_set_level(LCD_D7_PIN, (nibble >> 3) & 1U);
    esp_rom_delay_us(10);
    lcd_pulse_enable();
}

/**
 * @brief Envía un byte al LCD como instrucción o dato.
 * @param value Byte que se transmite.
 * @param is_data true para carácter, false para instrucción.
 * @return No devuelve un valor.
 */
static void lcd_send_byte(uint8_t value, bool is_data)
{
    (void)gpio_set_level(LCD_RS_PIN, is_data ? 1 : 0);
    esp_rom_delay_us(10);
    lcd_write_nibble(value >> 4);
    lcd_write_nibble(value & 0x0F);
}

/**
 * @brief Envía una instrucción al controlador LCD.
 * @param command Código de instrucción.
 * @return No devuelve un valor.
 */
static void lcd_send_command(uint8_t command)
{
    lcd_send_byte(command, false);
    if (command == 0x01 || command == 0x02) {
        vTaskDelay(pdMS_TO_TICKS(10));
    } else {
        esp_rom_delay_us(200);
    }
}

/**
 * @brief Escribe un carácter en el LCD.
 * @param character Carácter que se mostrará.
 * @return No devuelve un valor.
 */
static void lcd_send_character(char character)
{
    lcd_send_byte((uint8_t)character, true);
    esp_rom_delay_us(100);
}

/**
 * @brief Configura los GPIO y la secuencia de inicialización del LCD.
 * @return ESP_OK en éxito o código de error GPIO.
 */
static esp_err_t lcd_init_parallel(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = (1ULL << LCD_RS_PIN) | (1ULL << LCD_E_PIN) |
                        (1ULL << LCD_D4_PIN) | (1ULL << LCD_D5_PIN) |
                        (1ULL << LCD_D6_PIN) | (1ULL << LCD_D7_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t result = gpio_config(&config);
    if (result != ESP_OK) {
        return result;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    (void)gpio_set_level(LCD_RS_PIN, 0);
    (void)gpio_set_level(LCD_E_PIN, 0);
    lcd_write_nibble(0x03);
    vTaskDelay(pdMS_TO_TICKS(10));
    lcd_write_nibble(0x03);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x03);
    esp_rom_delay_us(500);
    lcd_write_nibble(0x02);
    vTaskDelay(pdMS_TO_TICKS(5));

    lcd_send_command(0x28);
    lcd_send_command(0x08);
    lcd_send_command(0x01);
    lcd_send_command(0x06);
    lcd_send_command(0x0C);
    return ESP_OK;
}

/**
 * @brief Posiciona el cursor del LCD.
 * @param column Columna destino (0-15).
 * @param row Fila destino (0-1).
 * @return No devuelve un valor.
 */
static void lcd_set_cursor(uint8_t column, uint8_t row)
{
    const uint8_t row_offsets[] = {0x00, 0x40};
    if (row > 1) {
        row = 1;
    }
    lcd_send_command((uint8_t)(0x80 | (column + row_offsets[row])));
}

/**
 * @brief Escribe exactamente 16 caracteres en una fila, rellenando con espacios.
 * @param row Fila de destino.
 * @param text Texto a mostrar.
 * @return No devuelve un valor.
 */
static void lcd_write_line(uint8_t row, const char *text)
{
    char line[17];
    memset(line, ' ', 16);
    line[16] = '\0';
    if (text != NULL) {
        size_t length = strlen(text);
        if (length > 16) {
            length = 16;
        }
        memcpy(line, text, length);
    }

    lcd_set_cursor(0, row);
    for (size_t index = 0; index < 16; index++) {
        lcd_send_character(line[index]);
    }
}

/** @} */

/** @name Inicialización de periféricos
 *  @{
 */

/**
 * @brief Configura botones y salidas de actuadores en estado seguro.
 * @return ESP_OK en éxito o código de error GPIO.
 */
static esp_err_t init_gpio(void)
{
    const gpio_config_t buttons_with_pullup = {
        .pin_bit_mask = 1ULL << BUTTON_PROFILE_3_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    const gpio_config_t buttons_external_pullup = {
        .pin_bit_mask = (1ULL << BUTTON_PROFILE_1_PIN) |
                        (1ULL << BUTTON_PROFILE_2_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    const gpio_config_t relay_output = {
        .pin_bit_mask = 1ULL << HEATER_RELAY_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t result = gpio_set_level(HEATER_RELAY_PIN, RELAY_OFF);
    if (result != ESP_OK) {
        return result;
    }
    result = gpio_config(&relay_output);
    if (result != ESP_OK) {
        return result;
    }
    result = gpio_set_level(HEATER_RELAY_PIN, RELAY_OFF);
    if (result != ESP_OK) {
        return result;
    }

    const gpio_config_t other_outputs = {
        .pin_bit_mask = (1ULL << AGITATOR_MOTOR_PIN) |
                        (1ULL << COOLING_FAN_PIN) |
                        (1ULL << BUZZER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    result = gpio_config(&other_outputs);
    if (result != ESP_OK) {
        return result;
    }
    result = gpio_set_level(AGITATOR_MOTOR_PIN, 0);
    if (result != ESP_OK) {
        return result;
    }
    result = gpio_set_level(COOLING_FAN_PIN, 0);
    if (result != ESP_OK) {
        return result;
    }
    result = gpio_set_level(BUZZER_PIN, 0);
    if (result != ESP_OK) {
        return result;
    }

    result = gpio_config(&buttons_with_pullup);
    if (result != ESP_OK) {
        return result;
    }
    return gpio_config(&buttons_external_pullup);
}

/**
 * @brief Inicializa el bus I2C y adjunta el DS3231.
 * @return ESP_OK en éxito o código de error del driver I2C.
 */
static esp_err_t init_i2c_bus(void)
{
    (void)gpio_reset_pin(I2C_SDA_PIN);
    (void)gpio_reset_pin(I2C_SCL_PIN);

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true
    };
    esp_err_t result = i2c_new_master_bus(&bus_config, &rtc_bus_handle);
    if (result != ESP_OK) {
        return result;
    }

    const i2c_device_config_t rtc_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DS3231_I2C_ADDR,
        .scl_speed_hz = 400000
    };
    return i2c_master_bus_add_device(rtc_bus_handle, &rtc_config,
                                    &rtc_device_handle);
}

/**
 * @brief Inicializa SPI2 y adjunta el MAX6675 sin reservar el bus para SD.
 * @return ESP_OK en éxito o código de error del driver SPI.
 */
static esp_err_t init_spi_bus(void)
{
    const spi_bus_config_t bus_config = {
        .mosi_io_num = SPI_MOSI_PIN,
        .miso_io_num = SPI_MISO_PIN,
        .sclk_io_num = SPI_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000
    };
    esp_err_t result = spi_bus_initialize(SPI2_HOST, &bus_config,
                                          SPI_DMA_CH_AUTO);
    if (result != ESP_OK) {
        return result;
    }

    const spi_device_interface_config_t sensor_config = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = MAX6675_CS_PIN,
        .queue_size = 1
    };
    return spi_bus_add_device(SPI2_HOST, &sensor_config,
                              &max6675_device_handle);
}

/**
 * @brief Crea una cola y registra un error explícito si falla.
 * @param length Número máximo de elementos de la cola.
 * @param item_size Tamaño de cada elemento.
 * @param queue_name Nombre usado en el registro de error.
 * @return Manejador de cola o NULL si no hay memoria disponible.
 */
static QueueHandle_t create_queue(UBaseType_t length,
                                  UBaseType_t item_size,
                                  const char *queue_name)
{
    QueueHandle_t queue = xQueueCreate(length, item_size);
    if (queue == NULL) {
        ESP_LOGE(TAG, "No se pudo crear la cola %s.", queue_name);
    }
    return queue;
}

/**
 * @brief Crea una tarea e informa si FreeRTOS no puede iniciarla.
 * @param task_entry Función de entrada de la tarea.
 * @param name Nombre de tarea.
 * @param stack_size Tamaño solicitado de stack en bytes en ESP-IDF.
 * @param priority Prioridad FreeRTOS.
 * @return ESP_OK si la tarea inició; ESP_ERR_NO_MEM si falló la creación.
 */
static esp_err_t create_task(TaskFunction_t task_entry,
                             const char *name,
                             uint32_t stack_size,
                             UBaseType_t priority)
{
    if (xTaskCreate(task_entry, name, stack_size, NULL, priority, NULL) != pdPASS) {
        ESP_LOGE(TAG, "No se pudo crear la tarea %s.", name);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/**
 * @brief Monta la tarjeta SD en SPI2 y crea la cabecera CSV si es necesario.
 * @return ESP_OK si FATFS quedó montado; código de error en caso contrario.
 */
static esp_err_t init_sd_card(void)
{
    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card = NULL;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = SD_SPI_MAX_FREQ_KHZ;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS_PIN;
    slot_config.host_id = host.slot;

    esp_err_t result = esp_vfs_fat_sdspi_mount(
        SD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (result != ESP_OK) {
        sd_card_mounted = false;
        ESP_LOGE("SD", "No se pudo montar la MicroSD: %s",
                 esp_err_to_name(result));
        return result;
    }

    FILE *file = fopen(SD_LOG_PATH, "a");
    if (file == NULL) {
        sd_card_mounted = false;
        ESP_LOGE("SD", "No se pudo abrir %s para inicializar el CSV.",
                 SD_LOG_PATH);
        return ESP_FAIL;
    }

    if (fseek(file, 0, SEEK_END) != 0) {
        (void)fclose(file);
        sd_card_mounted = false;
        ESP_LOGE("SD", "No se pudo posicionar el archivo de telemetría.");
        return ESP_FAIL;
    }
    if (ftell(file) == 0 &&
        fprintf(file, "Hora,Fecha,Temperatura_C,Objetivo_C,Rele,Estado\n") < 0) {
        (void)fclose(file);
        sd_card_mounted = false;
        ESP_LOGE("SD", "No se pudo escribir la cabecera CSV.");
        return ESP_FAIL;
    }
    if (fclose(file) != 0) {
        sd_card_mounted = false;
        ESP_LOGE("SD", "No se pudo cerrar el archivo de telemetría.");
        return ESP_FAIL;
    }

    sd_card_mounted = true;
    ESP_LOGI("SD", "MicroSD montada en %s a %d kHz.",
             SD_MOUNT_POINT, SD_SPI_MAX_FREQ_KHZ);
    return ESP_OK;
}

/** @} */

/** @name Drivers de sensor y publicación de eventos
 *  @{
 */

/**
 * @brief Lee una conversión del MAX6675 y valida el bit de sonda abierta.
 * @param temperature_c Puntero que recibe la temperatura en grados Celsius.
 * @return ESP_OK en lectura válida o código de error SPI/sonda.
 */
static esp_err_t max6675_read_temperature(float *temperature_c)
{
    if (temperature_c == NULL || max6675_device_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t raw_data = 0;
    spi_transaction_t transaction = {
        .length = 16,
        .rx_buffer = &raw_data,
        .tx_buffer = NULL
    };
    esp_err_t result = spi_device_transmit(max6675_device_handle, &transaction);
    if (result != ESP_OK) {
        return result;
    }

    raw_data = __builtin_bswap16(raw_data);
    ESP_LOGD("MAX6675", "Raw SPI Data: 0x%04X", raw_data);
    if ((raw_data & 0x04U) != 0U) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *temperature_c = (float)(raw_data >> 3) * 0.25f;
    return ESP_OK;
}

/**
 * @brief Devuelve el perfil del botón actualmente pulsado, con prioridad P1-P3.
 * @return Número de perfil 1-3 o cero si no hay pulsación.
 */
static uint8_t read_pressed_profile(void)
{
    if (gpio_get_level(BUTTON_PROFILE_1_PIN) == 0) {
        return 1;
    }
    if (gpio_get_level(BUTTON_PROFILE_2_PIN) == 0) {
        return 2;
    }
    if (gpio_get_level(BUTTON_PROFILE_3_PIN) == 0) {
        return 3;
    }
    return 0;
}

/**
 * @brief Indica si el botón de un perfil específico continúa pulsado.
 * @param profile Número de perfil (1-3).
 * @return true si la entrada correspondiente está en nivel bajo.
 */
static bool is_profile_button_pressed(uint8_t profile)
{
    switch (profile) {
        case 1: return gpio_get_level(BUTTON_PROFILE_1_PIN) == 0;
        case 2: return gpio_get_level(BUTTON_PROFILE_2_PIN) == 0;
        case 3: return gpio_get_level(BUTTON_PROFILE_3_PIN) == 0;
        default: return false;
    }
}

/**
 * @brief Espera cooperativamente hasta que los tres botones estén liberados.
 * @return No devuelve un valor.
 */
static void wait_for_button_release(void)
{
    while (gpio_get_level(BUTTON_PROFILE_1_PIN) == 0 ||
           gpio_get_level(BUTTON_PROFILE_2_PIN) == 0 ||
           gpio_get_level(BUTTON_PROFILE_3_PIN) == 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * @brief Publica un evento de botones sin bloquear la tarea de muestreo.
 * @param event Evento que se publicará.
 * @return No devuelve un valor.
 */
static void publish_button_event(const ButtonEvent_t *event)
{
    if (xQueueSend(xButtonQueue, event, 0) != pdPASS) {
        ESP_LOGW("BUTTON", "Cola de botones llena; evento descartado.");
    }
}

/** @} */

/** @name Tareas de adquisición de datos
 *  @{
 */

/**
 * @brief Muestrea botones, realiza debounce y publica toque o parada sostenida.
 * @param pvParameters Parámetros FreeRTOS no utilizados.
 * @return No devuelve un valor; la tarea se ejecuta indefinidamente.
 */
static void vTaskButtons(void *pvParameters)
{
    (void)pvParameters;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t debounce_ticks = pdMS_TO_TICKS(BUTTON_PERIOD_MS);
    const TickType_t hold_ticks = pdMS_TO_TICKS(EMERGENCY_HOLD_MS);
    uint8_t tracked_profile = 0;
    TickType_t press_started = 0;
    bool press_debounced = false;
    bool emergency_sent = false;

    for (;;) {
        TickType_t now_tick = xTaskGetTickCount();
        uint8_t pressed_profile = read_pressed_profile();

        if (tracked_profile == 0 && pressed_profile != 0) {
            tracked_profile = pressed_profile;
            press_started = now_tick;
            press_debounced = false;
            emergency_sent = false;
        } else if (tracked_profile != 0) {
            if (!is_profile_button_pressed(tracked_profile)) {
                if (press_debounced && !emergency_sent) {
                    wait_for_button_release();
                    const ButtonEvent_t event = {
                        .type = BUTTON_EVENT_SHORT_PRESS,
                        .profile = tracked_profile
                    };
                    publish_button_event(&event);
                }
                tracked_profile = 0;
                press_debounced = false;
                emergency_sent = false;
            } else {
                TickType_t held_ticks = now_tick - press_started;
                if (!press_debounced && held_ticks >= debounce_ticks) {
                    press_debounced = true;
                }
                if (press_debounced && !emergency_sent &&
                    held_ticks >= hold_ticks) {
                    const ButtonEvent_t event = {
                        .type = BUTTON_EVENT_EMERGENCY_STOP,
                        .profile = tracked_profile
                    };
                    publish_button_event(&event);
                    emergency_sent = true;
                    wait_for_button_release();
                    tracked_profile = 0;
                    press_debounced = false;
                    emergency_sent = false;
                    last_wake = xTaskGetTickCount();
                }
            }
        }

        vTaskDelayUntil(&last_wake, debounce_ticks);
    }
}

/**
 * @brief Muestrea MAX6675 cada 500 ms y publica lectura válida o error.
 * @param pvParameters Parámetros FreeRTOS no utilizados.
 * @return No devuelve un valor; la tarea se ejecuta indefinidamente.
 */
static void vTaskTempSensor(void *pvParameters)
{
    (void)pvParameters;
    TickType_t last_wake = xTaskGetTickCount();
    esp_err_t previous_status = ESP_OK;

    for (;;) {
        TemperatureMessage_t message = {
            .status = ESP_FAIL,
            .temperature_c = 0.0f
        };
        message.status = max6675_read_temperature(&message.temperature_c);
        if (message.status != previous_status) {
            if (message.status == ESP_OK) {
                ESP_LOGI("MAX6675", "Lectura de termocupla recuperada.");
            } else {
                ESP_LOGW("MAX6675", "Lectura inválida: %s",
                         esp_err_to_name(message.status));
            }
            previous_status = message.status;
        }
        if (xQueueOverwrite(xTempQueue, &message) != pdPASS) {
            ESP_LOGE("MAX6675", "No se pudo publicar la muestra.");
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TEMP_SENSOR_PERIOD_MS));
    }
}

/**
 * @brief Lee el DS3231 cada segundo y publica hora y fecha formateadas.
 * @param pvParameters Parámetros FreeRTOS no utilizados.
 * @return No devuelve un valor; la tarea se ejecuta indefinidamente.
 */
static void vTaskRTCClock(void *pvParameters)
{
    (void)pvParameters;
    TickType_t last_wake = xTaskGetTickCount();
    esp_err_t init_status = ds3231_init_or_update(rtc_device_handle, false);
    if (init_status != ESP_OK) {
        ESP_LOGE("RTC", "No se pudo inicializar DS3231: %s",
                 esp_err_to_name(init_status));
    }

    esp_err_t previous_status = ESP_OK;
    for (;;) {
        TimeMessage_t message = {
            .status = ESP_FAIL,
            .time_str = "RTC ERR",
            .date_str = "--/--/----"
        };
        struct tm time_info = {0};
        message.status = ds3231_get_time(rtc_device_handle, &time_info);
        if (message.status == ESP_OK) {
            (void)strftime(message.time_str, sizeof(message.time_str),
                           "%H:%M:%S", &time_info);
            (void)strftime(message.date_str, sizeof(message.date_str),
                           "%d/%m/%Y", &time_info);
        }

        if (message.status != previous_status) {
            if (message.status == ESP_OK) {
                ESP_LOGI("RTC", "Lectura DS3231 recuperada.");
            } else {
                ESP_LOGW("RTC", "No se pudo leer DS3231: %s",
                         esp_err_to_name(message.status));
            }
            previous_status = message.status;
        }
        if (xQueueOverwrite(xTimeQueue, &message) != pdPASS) {
            ESP_LOGE("RTC", "No se pudo publicar la hora.");
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(RTC_PERIOD_MS));
    }
}

/** @} */

/** @name FSM y comandos de actuadores
 *  @{
 */

/**
 * @brief Envía el estado completo de salidas a la tarea de actuadores.
 * @param state Estados lógicos que se aplicarán.
 * @param buzzer_request Acción solicitada para el buzzer.
 * @param beep_duration_ms Duración de cada pitido finito.
 * @param beep_count Número de pitidos finitos.
 * @return true si el comando se publicó correctamente.
 */
static bool send_actuator_command(const ActuatorCommand_t *state,
                                 BuzzerRequest_t buzzer_request,
                                 uint32_t beep_duration_ms,
                                 uint8_t beep_count)
{
    ActuatorCommand_t command = *state;
    command.buzzer_request = buzzer_request;
    command.beep_duration_ms = beep_duration_ms;
    command.beep_count = beep_count;
    if (xQueueSend(xActuatorQueue, &command, pdMS_TO_TICKS(20)) != pdPASS) {
        ESP_LOGE("ACTUATOR", "No se pudo publicar el comando de salidas.");
        return false;
    }
    return true;
}

/**
 * @brief Aplica la parada segura y activa enfriamiento audible/de aire.
 * @param telemetry Telemetría FSM que se actualiza.
 * @param actuators Estado de salida que se publicará.
 * @return No devuelve un valor.
 */
static void enter_emergency(Telemetry_t *telemetry,
                            ActuatorCommand_t *actuators)
{
    telemetry->state = FSM_STATE_EMERGENCY;
    telemetry->emergency_active = true;
    telemetry->heater_on = false;
    telemetry->fan_on = true;
    telemetry->selected_profile = 0;
    telemetry->remaining_seconds = 0;

    actuators->heater_on = false;
    actuators->motor_on = false;
    actuators->fan_on = true;
    (void)send_actuator_command(actuators, BUZZER_REQUEST_EMERGENCY, 0, 0);
}

/**
 * @brief Inicia el perfil seleccionado con su target y señal acústica.
 * @param profile Perfil de tueste entre 1 y 3.
 * @param now_tick Tick actual de FreeRTOS.
 * @param roast_started_tick Salida donde se guarda el inicio del tueste.
 * @param telemetry Telemetría que recibirá el nuevo estado.
 * @param actuators Estado de salida que se publicará.
 * @return true si el perfil se inició.
 */
static bool start_roast(uint8_t profile,
                        TickType_t now_tick,
                        TickType_t *roast_started_tick,
                        Telemetry_t *telemetry,
                        ActuatorCommand_t *actuators)
{
    if (profile < 1 || profile > 3 ||
        telemetry->temp_status != ESP_OK) {
        ESP_LOGW("FSM", "Tueste no iniciado: perfil o lectura de temperatura inválidos.");
        return false;
    }

    telemetry->state = FSM_STATE_ROAST_ACTIVE;
    telemetry->emergency_active = false;
    telemetry->selected_profile = profile;
    telemetry->target_temperature_c = PROFILE_TARGETS_C[profile - 1U];
    telemetry->remaining_seconds = ROAST_DURATION_SEC;
    telemetry->heater_on = false;
    telemetry->fan_on = false;
    *roast_started_tick = now_tick;

    actuators->heater_on = false;
    actuators->motor_on = true;
    actuators->fan_on = false;
    (void)send_actuator_command(actuators, BUZZER_REQUEST_PATTERN, 100, 2);
    ESP_LOGI("FSM", "Perfil %u iniciado: objetivo %.1f C.",
             (unsigned)profile, (double)telemetry->target_temperature_c);
    return true;
}

/**
 * @brief Controla el proceso, consume colas y publica telemetría.
 * @param pvParameters Parámetros FreeRTOS no utilizados.
 * @return No devuelve un valor; la tarea se ejecuta indefinidamente.
 */
static void vTaskControlFSM(void *pvParameters)
{
    (void)pvParameters;
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t roast_started_tick = 0;
    TickType_t last_valid_temperature_tick = 0;
    TickType_t last_display_tick = 0;
    TickType_t last_sd_tick = xTaskGetTickCount() -
                              pdMS_TO_TICKS(SD_LOG_PERIOD_MS);
    TickType_t last_sd_drop_log_tick = 0;
    Telemetry_t telemetry = {
        .time_str = "RTC ERR",
        .date_str = "--/--/----",
        .temperature_c = 0.0f,
        .temp_status = ESP_ERR_INVALID_STATE,
        .rtc_status = ESP_ERR_INVALID_STATE,
        .state = FSM_STATE_IDLE
    };
    ActuatorCommand_t actuators = {
        .heater_on = false,
        .motor_on = false,
        .fan_on = false,
        .buzzer_request = BUZZER_REQUEST_KEEP,
        .beep_duration_ms = 0,
        .beep_count = 0
    };
    bool display_dirty = true;

    for (;;) {
        TemperatureMessage_t temperature_message;
        while (xQueueReceive(xTempQueue, &temperature_message, 0) == pdPASS) {
            telemetry.temp_status = temperature_message.status;
            if (temperature_message.status == ESP_OK) {
                telemetry.temperature_c = temperature_message.temperature_c;
                last_valid_temperature_tick = xTaskGetTickCount();
            }
            display_dirty = true;
        }

        TimeMessage_t time_message;
        while (xQueueReceive(xTimeQueue, &time_message, 0) == pdPASS) {
            telemetry.rtc_status = time_message.status;
            (void)snprintf(telemetry.time_str, sizeof(telemetry.time_str),
                           "%s", time_message.time_str);
            (void)snprintf(telemetry.date_str, sizeof(telemetry.date_str),
                           "%s", time_message.date_str);
            display_dirty = true;
        }

        ButtonEvent_t button_event;
        while (xQueueReceive(xButtonQueue, &button_event, 0) == pdPASS) {
            if (telemetry.state == FSM_STATE_ROAST_ACTIVE &&
                button_event.type == BUTTON_EVENT_EMERGENCY_STOP) {
                ESP_LOGW("SAFETY", "Parada de emergencia sostenida por el usuario.");
                enter_emergency(&telemetry, &actuators);
                display_dirty = true;
            } else if (telemetry.state == FSM_STATE_EMERGENCY &&
                       button_event.type == BUTTON_EVENT_SHORT_PRESS) {
                telemetry.state = FSM_STATE_IDLE;
                telemetry.emergency_active = false;
                telemetry.fan_on = false;
                telemetry.selected_profile = 0;
                actuators.heater_on = false;
                actuators.motor_on = false;
                actuators.fan_on = false;
                (void)send_actuator_command(&actuators, BUZZER_REQUEST_STOP, 0, 0);
                ESP_LOGI("FSM", "Alerta de emergencia limpiada por el usuario.");
                display_dirty = true;
            } else if (telemetry.state == FSM_STATE_IDLE &&
                       !telemetry.emergency_active &&
                       button_event.type == BUTTON_EVENT_SHORT_PRESS) {
                display_dirty |= start_roast(button_event.profile,
                                             xTaskGetTickCount(),
                                             &roast_started_tick,
                                             &telemetry,
                                             &actuators);
            }
        }

        if (telemetry.state == FSM_STATE_ROAST_ACTIVE) {
            if (telemetry.temp_status != ESP_OK) {
                ESP_LOGE("SAFETY", "Falla de termocupla durante el tueste; parada segura.");
                enter_emergency(&telemetry, &actuators);
                display_dirty = true;
            } else if ((xTaskGetTickCount() - last_valid_temperature_tick) >=
                       pdMS_TO_TICKS(TEMP_STALE_TIMEOUT_MS)) {
                ESP_LOGE("SAFETY",
                         "Muestra de temperatura obsoleta por %u ms; parada de emergencia.",
                         (unsigned)TEMP_STALE_TIMEOUT_MS);
                enter_emergency(&telemetry, &actuators);
                display_dirty = true;
            } else {
                TickType_t elapsed_ticks = xTaskGetTickCount() - roast_started_tick;
                uint32_t elapsed_seconds =
                    (uint32_t)(elapsed_ticks / configTICK_RATE_HZ);

                if (elapsed_seconds >= ROAST_DURATION_SEC) {
                    telemetry.state = FSM_STATE_IDLE;
                    telemetry.emergency_active = false;
                    telemetry.heater_on = false;
                    telemetry.fan_on = true;
                    telemetry.selected_profile = 0;
                    telemetry.remaining_seconds = 0;
                    actuators.heater_on = false;
                    actuators.motor_on = false;
                    actuators.fan_on = true;
                    (void)send_actuator_command(&actuators,
                                                BUZZER_REQUEST_PATTERN,
                                                400, 3);
                    ESP_LOGI("FSM", "Tueste finalizado normalmente.");
                    display_dirty = true;
                } else {
                    telemetry.remaining_seconds =
                        ROAST_DURATION_SEC - (int32_t)elapsed_seconds;
                    bool desired_heater_on = telemetry.heater_on;
                    if (telemetry.temperature_c <
                        (telemetry.target_temperature_c - TEMP_HYSTERESIS_C)) {
                        desired_heater_on = true;
                    } else if (telemetry.temperature_c >
                               (telemetry.target_temperature_c + TEMP_HYSTERESIS_C)) {
                        desired_heater_on = false;
                    }

                    if (desired_heater_on != telemetry.heater_on) {
                        telemetry.heater_on = desired_heater_on;
                        actuators.heater_on = desired_heater_on;
                        (void)send_actuator_command(&actuators,
                                                    BUZZER_REQUEST_KEEP, 0, 0);
                    }
                }
            }
        }

        TickType_t now_tick = xTaskGetTickCount();
        if (display_dirty ||
            (now_tick - last_display_tick) >=
                pdMS_TO_TICKS(DISPLAY_REFRESH_PERIOD_MS)) {
            if (xQueueOverwrite(xDisplayQueue, &telemetry) != pdPASS) {
                ESP_LOGE("FSM", "No se pudo publicar telemetría al LCD.");
            }
            last_display_tick = now_tick;
            display_dirty = false;
        }

        if ((now_tick - last_sd_tick) >= pdMS_TO_TICKS(SD_LOG_PERIOD_MS)) {
            if (xQueueSend(xSDQueue, &telemetry, 0) != pdPASS &&
                (now_tick - last_sd_drop_log_tick) >= pdMS_TO_TICKS(10000)) {
                ESP_LOGW("FSM", "Cola SD llena; muestra de telemetría descartada.");
                last_sd_drop_log_tick = now_tick;
            }
            last_sd_tick = now_tick;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_FSM_PERIOD_MS));
    }
}

/** @} */

/** @name Tareas de salida y registro
 *  @{
 */

/**
 * @brief Configura la línea del buzzer para el inicio de un patrón finito.
 * @param request Acción nueva solicitada.
 * @param command Comando que contiene duración y cuenta.
 * @param mode Salida donde se almacena el modo vigente.
 * @param buzzer_on Salida donde se almacena el nivel actual del buzzer.
 * @param remaining Salida donde se almacena la cuenta de pitidos.
 * @param gap_pending Salida que indica una pausa entre pitidos.
 * @param duration_ticks Salida con la duración de encendido en ticks.
 * @param deadline Salida con el próximo instante de transición.
 * @return No devuelve un valor.
 */
static void apply_buzzer_request(BuzzerRequest_t request,
                                 const ActuatorCommand_t *command,
                                 BuzzerRequest_t *mode,
                                 bool *buzzer_on,
                                 uint8_t *remaining,
                                 bool *gap_pending,
                                 TickType_t *duration_ticks,
                                 TickType_t *deadline)
{
    if (request == BUZZER_REQUEST_KEEP) {
        return;
    }
    if (request == BUZZER_REQUEST_STOP ||
        (request == BUZZER_REQUEST_PATTERN && command->beep_count == 0)) {
        *mode = BUZZER_REQUEST_STOP;
        *buzzer_on = false;
        *remaining = 0;
        *gap_pending = false;
        (void)gpio_set_level(BUZZER_PIN, 0);
        return;
    }

    *mode = request;
    *gap_pending = false;
    if (request == BUZZER_REQUEST_EMERGENCY) {
        *buzzer_on = true;
        *remaining = 0;
        *deadline = xTaskGetTickCount() + pdMS_TO_TICKS(80);
    } else {
        *remaining = command->beep_count;
        *duration_ticks = pdMS_TO_TICKS(command->beep_duration_ms);
        if (*duration_ticks == 0) {
            *duration_ticks = 1;
        }
        *buzzer_on = true;
        *deadline = xTaskGetTickCount() + *duration_ticks;
    }
    (void)gpio_set_level(BUZZER_PIN, 1);
}

/**
 * @brief Atiende comandos de relé, motor, ventilador y patrones de buzzer.
 * @param pvParameters Parámetros FreeRTOS no utilizados.
 * @return No devuelve un valor; la tarea espera comandos o vencimientos.
 */
static void vTaskActuators(void *pvParameters)
{
    (void)pvParameters;
    BuzzerRequest_t buzzer_mode = BUZZER_REQUEST_STOP;
    bool buzzer_on = false;
    bool gap_pending = false;
    uint8_t beeps_remaining = 0;
    TickType_t beep_duration_ticks = 1;
    TickType_t deadline = 0;

    for (;;) {
        TickType_t wait_ticks = portMAX_DELAY;
        if (buzzer_mode == BUZZER_REQUEST_EMERGENCY ||
            buzzer_mode == BUZZER_REQUEST_PATTERN) {
            TickType_t now_tick = xTaskGetTickCount();
            wait_ticks = ((int32_t)(deadline - now_tick) > 0)
                             ? deadline - now_tick
                             : 0;
        }

        ActuatorCommand_t command;
        if (xQueueReceive(xActuatorQueue, &command, wait_ticks) == pdPASS) {
            (void)gpio_set_level(HEATER_RELAY_PIN,
                                 command.heater_on ? RELAY_ON : RELAY_OFF);
            (void)gpio_set_level(AGITATOR_MOTOR_PIN,
                                 command.motor_on ? 1 : 0);
            (void)gpio_set_level(COOLING_FAN_PIN,
                                 command.fan_on ? 1 : 0);
            apply_buzzer_request(command.buzzer_request, &command,
                                 &buzzer_mode, &buzzer_on, &beeps_remaining,
                                 &gap_pending, &beep_duration_ticks, &deadline);
            continue;
        }

        TickType_t now_tick = xTaskGetTickCount();
        if (buzzer_mode == BUZZER_REQUEST_EMERGENCY) {
            buzzer_on = !buzzer_on;
            (void)gpio_set_level(BUZZER_PIN, buzzer_on ? 1 : 0);
            deadline = now_tick + pdMS_TO_TICKS(buzzer_on ? 80 : 920);
        } else if (buzzer_mode == BUZZER_REQUEST_PATTERN) {
            if (buzzer_on) {
                buzzer_on = false;
                (void)gpio_set_level(BUZZER_PIN, 0);
                if (beeps_remaining > 0) {
                    beeps_remaining--;
                }
                if (beeps_remaining == 0) {
                    buzzer_mode = BUZZER_REQUEST_STOP;
                } else {
                    gap_pending = true;
                    deadline = now_tick + pdMS_TO_TICKS(100);
                }
            } else if (gap_pending) {
                gap_pending = false;
                buzzer_on = true;
                (void)gpio_set_level(BUZZER_PIN, 1);
                deadline = now_tick + beep_duration_ticks;
            }
        }
    }
}

/**
 * @brief Espera telemetría y actualiza las dos líneas del LCD.
 * @param pvParameters Parámetros FreeRTOS no utilizados.
 * @return No devuelve un valor; la tarea duerme en xDisplayQueue.
 */
static void vTaskLCDDisplay(void *pvParameters)
{
    (void)pvParameters;
    esp_err_t result = lcd_init_parallel();
    if (result != ESP_OK) {
        ESP_LOGE("LCD", "No se pudo inicializar el LCD: %s",
                 esp_err_to_name(result));
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        Telemetry_t telemetry;
        if (xQueueReceive(xDisplayQueue, &telemetry, portMAX_DELAY) != pdPASS) {
            continue;
        }

        char line[32];
        if (telemetry.state == FSM_STATE_EMERGENCY) {
            (void)snprintf(line, sizeof(line), "! EMERGENCIA !");
            lcd_write_line(0, line);
            (void)snprintf(line, sizeof(line), "%3.0fC Enfriando",
                           (double)telemetry.temperature_c);
            lcd_write_line(1, line);
        } else if (telemetry.state == FSM_STATE_IDLE) {
            (void)snprintf(line, sizeof(line), "Hora: %s",
                           telemetry.time_str);
            lcd_write_line(0, line);
            lcd_write_line(1, "Elija P1, P2 o P3");
        } else {
            int32_t minutes = telemetry.remaining_seconds / 60;
            int32_t seconds = telemetry.remaining_seconds % 60;
            if (telemetry.temp_status == ESP_OK) {
                (void)snprintf(line, sizeof(line), "%3.0fC %02ld:%02ldm %s",
                               (double)telemetry.temperature_c,
                               (long)minutes, (long)seconds,
                               telemetry.heater_on ? "*" : " ");
            } else {
                (void)snprintf(line, sizeof(line), "Err Termocupla");
            }
            lcd_write_line(0, line);
            (void)snprintf(line, sizeof(line), "P%ld:%3.0fC %s",
                           (long)telemetry.selected_profile,
                           (double)telemetry.target_temperature_c,
                           telemetry.time_str);
            lcd_write_line(1, line);
        }
    }
}

/**
 * @brief Monta la tarjeta SD y bloquea esperando muestras de telemetría.
 * @param pvParameters Parámetros FreeRTOS no utilizados.
 * @return No devuelve un valor; la tarea duerme en xSDQueue.
 */
static void vTaskSDLogger(void *pvParameters)
{
    (void)pvParameters;
    esp_err_t mount_result = init_sd_card();
    if (mount_result != ESP_OK) {
        ESP_LOGW("SD", "Registro deshabilitado hasta reiniciar el sistema.");
    }

    for (;;) {
        Telemetry_t telemetry;
        if (xQueueReceive(xSDQueue, &telemetry, portMAX_DELAY) != pdPASS) {
            continue;
        }
        if (!sd_card_mounted) {
            continue;
        }

        FILE *file = fopen(SD_LOG_PATH, "a");
        if (file == NULL) {
            sd_card_mounted = false;
            ESP_LOGE("SD", "Se perdió acceso al archivo; logger deshabilitado.");
            continue;
        }
        int written = fprintf(file, "%s,%s,%.2f,%.2f,%d,%d\n",
                              telemetry.time_str, telemetry.date_str,
                              (double)telemetry.temperature_c,
                              (double)telemetry.target_temperature_c,
                              telemetry.heater_on ? 1 : 0,
                              (int)telemetry.state);
        int close_result = fclose(file);
        if (written < 0 || close_result != 0) {
            sd_card_mounted = false;
            ESP_LOGE("SD", "Error de escritura; logger deshabilitado.");
        }
    }
}

/** @} */

/**
 * @brief Fuerza las salidas a estado seguro durante una falla de inicialización.
 * @return No devuelve un valor; los errores de GPIO quedan registrados.
 */
static void force_safe_outputs(void)
{
    esp_err_t result = gpio_set_level(HEATER_RELAY_PIN, RELAY_OFF);
    if (result != ESP_OK) {
        ESP_LOGE("SAFETY", "No se pudo forzar SSR apagado: %s",
                 esp_err_to_name(result));
    }
    result = gpio_set_level(AGITATOR_MOTOR_PIN, 0);
    if (result != ESP_OK) {
        ESP_LOGE("SAFETY", "No se pudo apagar el agitador: %s",
                 esp_err_to_name(result));
    }
    result = gpio_set_level(COOLING_FAN_PIN, 0);
    if (result != ESP_OK) {
        ESP_LOGE("SAFETY", "No se pudo apagar el ventilador: %s",
                 esp_err_to_name(result));
    }
    result = gpio_set_level(BUZZER_PIN, 0);
    if (result != ESP_OK) {
        ESP_LOGE("SAFETY", "No se pudo apagar el buzzer: %s",
                 esp_err_to_name(result));
    }
}

/**
 * @brief Inicializa hardware y colas y crea las siete tareas desacopladas.
 * @return No devuelve un valor; ante falla registra el error y conserva salidas seguras.
 */
void app_main(void)
{
    rtc_cpu_freq_config_t cpu_config;
    if (rtc_clk_cpu_freq_mhz_to_config(80, &cpu_config)) {
        rtc_clk_cpu_freq_set_config_fast(&cpu_config);
    } else {
        ESP_LOGW(TAG, "Se conserva la frecuencia de CPU por defecto.");
    }

    ESP_LOGI(TAG, "Inicializando tostadora modular.");
    esp_err_t result = init_gpio();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Fallo al configurar GPIO: %s", esp_err_to_name(result));
        force_safe_outputs();
        return;
    }

    result = init_i2c_bus();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Fallo al inicializar I2C: %s", esp_err_to_name(result));
        force_safe_outputs();
        return;
    }
    result = init_spi_bus();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Fallo al inicializar SPI2/MAX6675: %s",
                 esp_err_to_name(result));
        force_safe_outputs();
        return;
    }

    xButtonQueue = create_queue(BUTTON_QUEUE_LENGTH,
                                sizeof(ButtonEvent_t), "button");
    xTempQueue = create_queue(SENSOR_QUEUE_LENGTH,
                              sizeof(TemperatureMessage_t), "temperature");
    xTimeQueue = create_queue(SENSOR_QUEUE_LENGTH,
                              sizeof(TimeMessage_t), "time");
    xActuatorQueue = create_queue(ACTUATOR_QUEUE_LENGTH,
                                  sizeof(ActuatorCommand_t), "actuator");
    xDisplayQueue = create_queue(DISPLAY_QUEUE_LENGTH,
                                 sizeof(Telemetry_t), "display");
    xSDQueue = create_queue(SD_QUEUE_LENGTH,
                            sizeof(Telemetry_t), "sd_logger");
    if (xButtonQueue == NULL || xTempQueue == NULL || xTimeQueue == NULL ||
        xActuatorQueue == NULL || xDisplayQueue == NULL || xSDQueue == NULL) {
        ESP_LOGE(TAG, "Falla al crear colas IPC; tareas de control no iniciadas.");
        force_safe_outputs();
        return;
    }

    /* Crear consumidores y adquisición antes de habilitar la FSM de control. */
    if ((result = create_task(vTaskTempSensor, "temp_sensor", 2048, 4)) != ESP_OK ||
        (result = create_task(vTaskRTCClock, "rtc_clock", 2048, 2)) != ESP_OK ||
        (result = create_task(vTaskSDLogger, "sd_logger", 4096, 1)) != ESP_OK ||
        (result = create_task(vTaskLCDDisplay, "lcd_display", 3072, 3)) != ESP_OK ||
        (result = create_task(vTaskActuators, "actuators", 2048, 5)) != ESP_OK ||
        (result = create_task(vTaskButtons, "buttons", 2048, 6)) != ESP_OK ||
        (result = create_task(vTaskControlFSM, "control_fsm", 3072, 5)) != ESP_OK) {
        ESP_LOGE(TAG,
                 "Inicio incompleto de tareas (%s); se mantienen salidas seguras.",
                 esp_err_to_name(result));
        force_safe_outputs();
        return;
    }
    ESP_LOGI(TAG, "Las siete tareas de control y servicios están activas.");
}
