/**
 * @file main.c
 * @brief Sistema Autónomo de Control y Telemetría para Tostadora de Café.
 * @details Este programa ejecuta el control de temperatura por histéresis,
 *          gestiona perfiles de tueste, registra la telemetría en un archivo CSV
 *          dentro de una tarjeta MicroSD, actualiza la pantalla LCD Paralela
 *          y gestiona el reloj RTC DS3231 utilizando FreeRTOS en una ESP32.
 * 
 * @date 2026-10-05
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#include "soc/rtc.h"

/** @brief Etiqueta común para los mensajes de registro del sistema. */
static const char *TAG = "MAIN_SYSTEM";

/** @name Definiciones de hardware y constantes
 *  @{
 */

/** @brief GPIO de datos SDA del bus I2C del RTC. */
#define I2C_SDA_PIN      21
/** @brief GPIO de reloj SCL del bus I2C del RTC. */
#define I2C_SCL_PIN      22
/** @brief Dirección I2C de 7 bits del DS3231. */
#define DS3231_I2C_ADDR  0x68
/** @brief Tiempo máximo de espera para transacciones I2C, en milisegundos. */
#define I2C_TIMEOUT_MS   50

/** @brief GPIO MISO compartido por el bus SPI. */
#define PIN_NUM_MISO     GPIO_NUM_19
/** @brief GPIO SCLK compartido por el bus SPI. */
#define PIN_NUM_CLK      GPIO_NUM_18
/** @brief GPIO chip-select del MAX6675. */
#define PIN_NUM_MAX_CS   GPIO_NUM_4
/** @brief GPIO MOSI compartido por el bus SPI. */
#define PIN_NUM_MOSI     GPIO_NUM_23
/** @brief GPIO chip-select de la tarjeta MicroSD. */
#define PIN_NUM_SD_CS    GPIO_NUM_5
/** @brief Punto de montaje VFS de la tarjeta MicroSD. */
#define MOUNT_POINT      "/sdcard"

/** @brief GPIO de selección de registro del LCD. */
#define LCD_RS           GPIO_NUM_25
/** @brief GPIO de habilitación del LCD. */
#define LCD_E            GPIO_NUM_26
/** @brief GPIO del bit de datos D4 del LCD. */
#define LCD_D4           GPIO_NUM_27
/** @brief GPIO del bit de datos D5 del LCD. */
#define LCD_D5           GPIO_NUM_14
/** @brief GPIO del bit de datos D6 del LCD. */
#define LCD_D6           GPIO_NUM_16
/** @brief GPIO del bit de datos D7 del LCD. */
#define LCD_D7           GPIO_NUM_13

/** @brief Botón para seleccionar el perfil 1. */
#define PIN_BTN_1        GPIO_NUM_34
/** @brief Botón para seleccionar el perfil 2. */
#define PIN_BTN_2        GPIO_NUM_35
/** @brief Botón para seleccionar el perfil 3. */
#define PIN_BTN_3        GPIO_NUM_32

/** @brief GPIO de control del relé calefactor. */
#define PIN_HEATER_RELAY GPIO_NUM_33
/** @brief GPIO de control del motor agitador. */
#define PIN_MOTOR_AGITADOR GPIO_NUM_2
/** @brief GPIO de control del ventilador. */
#define PIN_VENTILADOR   GPIO_NUM_15
/** @brief GPIO de salida del buzzer activo. */
#define PIN_BUZZER       GPIO_NUM_17
/** @brief Nivel lógico que activa el relé (ajustar para relés Active LOW). */
#define RELAY_ON         0
/** @brief Nivel lógico que desactiva el relé (ajustar para relés Active LOW). */
#define RELAY_OFF        1
/** @brief Banda de histéresis de control de temperatura, en grados Celsius. */
#define HYSTERESIS       2.0f

/** @} */

/** @name Estructuras y variables globales
 *  @{
 */

/**
 * @brief Datos de sensor y estado del proceso enviados a la tarea LCD.
 */
typedef struct {
    char time_str[16];             /**< Hora actual formateada. */
    char date_str[16];             /**< Fecha actual formateada. */
    float temperatura;             /**< Temperatura actual en grados Celsius. */
    bool tc_error;                 /**< Indica una lectura inválida de termocupla. */
    bool heater_on;                /**< Estado lógico del calentador. */
    bool parada_emergencia;        /**< Indica que está activa una alerta de emergencia. */
    bool en_proceso;               /**< Indica si hay un tueste activo. */
    int perfil_seleccionado;       /**< Número del perfil de tueste activo. */
    float temp_target;             /**< Temperatura objetivo del perfil, en grados Celsius. */
    int tiempo_restante_sec;       /**< Tiempo restante del tueste en segundos. */
} SensorData_t;

/** @brief Cola FreeRTOS que transfiere telemetría a la tarea LCD. */
QueueHandle_t sensorQueue;
/** @brief Manejador del dispositivo I2C DS3231. */
i2c_master_dev_handle_t rtc_dev_handle;
/** @brief Manejador SPI del sensor de termocupla MAX6675. */
spi_device_handle_t max6675_spi_handle;
/** @brief Indica si la tarjeta SD está montada y habilitada para registro. */
static bool g_sd_mounted = false;

/** @} */

/** @name Conversión y comunicación con el RTC
 *  @{
 */

/**
 * @brief Convierte un valor codificado en BCD a entero decimal.
 * @param val Byte en formato BCD.
 * @return Valor decimal convertido.
 */
static inline uint8_t bcd2dec(uint8_t val) { return ((val >> 4) * 10) + (val & 0x0F); }
/**
 * @brief Convierte un entero decimal a formato BCD.
 * @param val Valor decimal de entrada.
 * @return Byte codificado en BCD.
 */
static inline uint8_t dec2bcd(uint8_t val) { return ((val / 10) << 4) | (val % 10); }

/**
 * @brief Obtiene el número de mes a partir de su abreviatura en inglés.
 * @param month_str Cadena con la abreviatura de tres letras.
 * @return Mes entre 1 y 12; devuelve enero si no coincide.
 */
static int parse_month(const char *month_str) {
    const char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", 
                            "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    for (int i = 0; i < 12; i++) {
        if (strncmp(month_str, months[i], 3) == 0) return i + 1;
    }
    return 1;
}

/**
 * @brief Inicializa o actualiza la fecha y hora del RTC cuando sea necesario.
 * @param dev_handle Manejador del dispositivo DS3231.
 * @param force_reset Fuerza la escritura de fecha/hora de compilación.
 * @return ESP_OK si la operación tuvo éxito; de lo contrario, el error I2C.
 */
esp_err_t ds3231_init_or_update(i2c_master_dev_handle_t dev_handle, bool force_reset) {
    if (dev_handle == NULL) return ESP_ERR_INVALID_STATE;
    uint8_t reg_status = 0x0F;
    uint8_t status_val = 0;

    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_status, 1, &status_val, 1, I2C_TIMEOUT_MS);
    if (ret != ESP_OK) return ret;

    bool osf_flag = (status_val & 0x80) != 0;

    if (force_reset || osf_flag) {
        char month_buf[4];
        int day, year, hour, min, sec;
        
        sscanf(__DATE__, "%s %d %d", month_buf, &day, &year);
        sscanf(__TIME__, "%d:%d:%d", &hour, &min, &sec);

        uint8_t month = parse_month(month_buf);
        uint8_t write_buf[8];
        
        write_buf[0] = 0x00;
        write_buf[1] = dec2bcd(sec);
        write_buf[2] = dec2bcd(min);
        write_buf[3] = dec2bcd(hour);
        write_buf[4] = 1;
        write_buf[5] = dec2bcd(day);
        write_buf[6] = dec2bcd(month);
        write_buf[7] = dec2bcd(year % 100);

        ret = i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_TIMEOUT_MS);
        if (ret != ESP_OK) return ret;

        uint8_t clear_osf[2] = {0x0F, 0x00};
        return i2c_master_transmit(dev_handle, clear_osf, sizeof(clear_osf), I2C_TIMEOUT_MS);
    }

    return ESP_OK;
}

/**
 * @brief Lee fecha y hora del DS3231 y las convierte a struct tm.
 * @param dev_handle Manejador del dispositivo DS3231.
 * @param timeinfo Puntero donde se almacenará la fecha y hora leída.
 * @return ESP_OK si la lectura tuvo éxito; de lo contrario, el error I2C.
 */
esp_err_t ds3231_get_time(i2c_master_dev_handle_t dev_handle, struct tm *timeinfo) {
    if (dev_handle == NULL) return ESP_ERR_INVALID_STATE;
    uint8_t reg_addr = 0x00;
    uint8_t data[7];

    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, sizeof(data), I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGW("RTC", "Timeout/Error I2C en DS3231: %s", esp_err_to_name(ret));
        return ret;
    }

    timeinfo->tm_sec  = bcd2dec(data[0] & 0x7F);
    timeinfo->tm_min  = bcd2dec(data[1] & 0x7F);
    timeinfo->tm_hour = bcd2dec(data[2] & 0x3F);
    timeinfo->tm_mday = bcd2dec(data[4] & 0x3F);
    timeinfo->tm_mon  = bcd2dec(data[5] & 0x1F) - 1;
    timeinfo->tm_year = bcd2dec(data[6]) + 100;

    return ESP_OK;
}

/** @} */

/** @name Control del LCD paralelo de 16x2 caracteres
 *  @{
 */

/**
 * @brief Genera un pulso de habilitación para el LCD.
 * @return No devuelve un valor.
 */
void lcd_pulse_enable(void) {
    gpio_set_level(LCD_E, 1);
    esp_rom_delay_us(50);
    gpio_set_level(LCD_E, 0);
    esp_rom_delay_us(100);
}

/**
 * @brief Escribe cuatro bits en el bus de datos del LCD.
 * @param nibble Nibble bajo que se transmitirá.
 * @return No devuelve un valor.
 */
void lcd_write_nibble(uint8_t nibble) {
    gpio_set_level(LCD_D4, (nibble >> 0) & 0x01);
    gpio_set_level(LCD_D5, (nibble >> 1) & 0x01);
    gpio_set_level(LCD_D6, (nibble >> 2) & 0x01);
    gpio_set_level(LCD_D7, (nibble >> 3) & 0x01);
    esp_rom_delay_us(10);
    lcd_pulse_enable();
}

/**
 * @brief Envía un byte al LCD como comando o como dato.
 * @param val Byte que se transmitirá.
 * @param is_data Cero para comando y distinto de cero para carácter.
 * @return No devuelve un valor.
 */
void lcd_send_byte(uint8_t val, uint8_t is_data) {
    gpio_set_level(LCD_RS, is_data);
    esp_rom_delay_us(10);
    lcd_write_nibble(val >> 4);
    lcd_write_nibble(val & 0x0F);
}

/**
 * @brief Envía un comando de control al LCD.
 * @param cmd Código de comando del controlador LCD.
 * @return No devuelve un valor.
 */
void lcd_send_cmd(uint8_t cmd) {
    lcd_send_byte(cmd, 0);
    if (cmd == 0x01 || cmd == 0x02) {
        vTaskDelay(pdMS_TO_TICKS(10));
    } else {
        esp_rom_delay_us(200);
    }
}

/**
 * @brief Envía un carácter al LCD.
 * @param c Carácter que se mostrará.
 * @return No devuelve un valor.
 */
void lcd_send_char(char c) {
    lcd_send_byte(c, 1);
    esp_rom_delay_us(100);
}

/**
 * @brief Configura los GPIO y ejecuta la secuencia de inicialización del LCD.
 * @return No devuelve un valor.
 */
void lcd_init_parallel(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL<<LCD_RS) | (1ULL<<LCD_E) |
                        (1ULL<<LCD_D4) | (1ULL<<LCD_D5) |
                        (1ULL<<LCD_D6) | (1ULL<<LCD_D7),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LCD_RS, 0);
    gpio_set_level(LCD_E, 0);

    lcd_write_nibble(0x03);
    vTaskDelay(pdMS_TO_TICKS(10));
    lcd_write_nibble(0x03);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x03);
    esp_rom_delay_us(500);

    lcd_write_nibble(0x02);
    vTaskDelay(pdMS_TO_TICKS(5));

    lcd_send_cmd(0x28);
    lcd_send_cmd(0x08);
    lcd_send_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(10));
    lcd_send_cmd(0x06);
    lcd_send_cmd(0x0C);
}

/**
 * @brief Posiciona el cursor del LCD en una columna y fila.
 * @param col Columna de destino.
 * @param row Fila de destino (0 o 1).
 * @return No devuelve un valor.
 */
void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t offsets[] = {0x00, 0x40};
    if (row > 1) row = 1;
    lcd_send_cmd(0x80 | (col + offsets[row]));
}

/**
 * @brief Escribe una cadena terminada en nulo en la posición actual del LCD.
 * @param str Cadena que se mostrará.
 * @return No devuelve un valor.
 */
void lcd_print_string(const char *str) {
    while (*str) {
        lcd_send_char(*str++);
    }
}

/** @} */

/** @name Almacenamiento MicroSD, buzzer y termocupla
 *  @{
 */

/**
 * @brief Inicializa SDSPI, monta la tarjeta y crea la cabecera del CSV si falta.
 * @param spi_host Host SPI previamente inicializado y compartido.
 * @return No devuelve un valor; el resultado se refleja en g_sd_mounted.
 */
void init_sd_card(spi_host_device_t spi_host) {
    esp_err_t ret;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t *card;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = spi_host;
    host.max_freq_khz = 5000;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_SD_CS;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        g_sd_mounted = false;
        if (ret == ESP_FAIL) {
            ESP_LOGE("SD", "Error al montar el sistema de archivos FAT.");
        } else {
            ESP_LOGE("SD", "Error al inicializar la tarjeta SD (%s).", esp_err_to_name(ret));
        }
        return;
    }

    g_sd_mounted = true;
    ESP_LOGI("SD", "Tarjeta MicroSD montada con éxito en %s", MOUNT_POINT);

    FILE *f = fopen("/sdcard/tueste.csv", "a");
    if (f != NULL) {
        fseek(f, 0, SEEK_END);
        if (ftell(f) == 0) {
            fprintf(f, "Tiempo,Temperatura_Real,Temperatura_Target,Relay_Status\n");
        }
        fclose(f);
    } else {
        g_sd_mounted = false;
        ESP_LOGW("SD", "No se pudo abrir tueste.csv; se deshabilita el registro en SD.");
    }
}

/**
 * @brief Anexa una muestra de telemetría al archivo CSV de la tarjeta SD.
 * @param data Muestra de telemetría que se registrará.
 * @return No devuelve un valor.
 */
static void log_telemetry_to_sd(const SensorData_t *data) {
    if (!g_sd_mounted || data == NULL) return;

    FILE *f = fopen("/sdcard/tueste.csv", "a");
    if (f != NULL) {
        fprintf(f, "%s,%.2f,%.2f,%d\n", 
                data->time_str,
                data->temperatura,
                data->temp_target,
                data->heater_on ? 1 : 0);
        fclose(f);
    } else {
        g_sd_mounted = false;
        ESP_LOGW("SD", "No se pudo abrir tueste.csv; se deshabilita el registro en SD.");
    }
}

/**
 * @brief Emite una secuencia de pitidos por el buzzer activo.
 * @param duration_ms Duración de cada pitido en milisegundos.
 * @param count Número de pitidos que se emitirán.
 * @return No devuelve un valor.
 */
void buzzer_beep(uint32_t duration_ms, uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        gpio_set_level(PIN_BUZZER, 1);
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        gpio_set_level(PIN_BUZZER, 0);
        if (i + 1 < count) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

/**
 * @brief Lee la temperatura del MAX6675 mediante SPI.
 * @param temp_out Puntero donde se almacena la temperatura en grados Celsius.
 * @return true si la lectura es válida; false si hay error o termocupla abierta.
 */
bool max6675_read_temp(float *temp_out) {
    if (max6675_spi_handle == NULL) return false;

    uint16_t raw_data = 0;
    spi_transaction_t t = {
        .length = 16,
        .rx_buffer = &raw_data,
        .tx_buffer = NULL
    };

    esp_err_t ret = spi_device_transmit(max6675_spi_handle, &t);
    ESP_LOGI("MAX6675_DEBUG", "Raw SPI Data: 0x%04X", raw_data);

    if (ret == ESP_OK) {
        raw_data = __builtin_bswap16(raw_data);

        if (raw_data & 0x04) {
            return false;
        }

        *temp_out = (raw_data >> 3) * 0.25f;
        return true;
    }

    ESP_LOGW("MAX6675", "Error de comunicación SPI (%s)", esp_err_to_name(ret));
    return false;
}

/**
 * @brief Apaga los actuadores del tueste y actualiza el estado seguro.
 * @param sample Estado del proceso que se actualizará; puede ser NULL.
 * @param emergency_cooling Activa el ventilador y marca alerta si es true.
 * @return No devuelve un valor.
 */
static void set_safe_state(SensorData_t *sample, bool emergency_cooling) {
    gpio_set_level(PIN_HEATER_RELAY, RELAY_OFF);
    gpio_set_level(PIN_MOTOR_AGITADOR, 0);
    gpio_set_level(PIN_VENTILADOR, emergency_cooling ? 1 : 0);

    if (sample != NULL) {
        sample->parada_emergencia = emergency_cooling;
        sample->heater_on = false;
        sample->en_proceso = false;
        sample->perfil_seleccionado = 0;
        sample->tiempo_restante_sec = 0;
    }
}

/**
 * @brief Espera hasta que todos los botones estén liberados.
 * @return No devuelve un valor.
 */
static void wait_for_buttons_release(void) {
    while (gpio_get_level(PIN_BTN_1) == 0 ||
           gpio_get_level(PIN_BTN_2) == 0 ||
           gpio_get_level(PIN_BTN_3) == 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/** @} */

/** @name Tareas de FreeRTOS
 *  @{
 */

/**
 * @brief Muestrea sensores, procesa botones, controla el tueste y registra datos.
 * @param pvParameters Parámetros de tarea FreeRTOS (no utilizados).
 * @return No devuelve un valor; la tarea se ejecuta indefinidamente.
 */
void vTaskSampling(void *pvParameters) {
    SensorData_t sample = {0};
    struct tm timeinfo;
    
    int tiempo_total_sec = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        /** @brief Lee y formatea la hora actual del RTC. */
        if (ds3231_get_time(rtc_dev_handle, &timeinfo) == ESP_OK) {
            strftime(sample.time_str, sizeof(sample.time_str), "%H:%M:%S", &timeinfo);
        } else {
            snprintf(sample.time_str, sizeof(sample.time_str), "RTC ERR");
        }

        /** @brief Actualiza la temperatura y activa el estado seguro ante error. */
        if (max6675_read_temp(&sample.temperatura)) {
            sample.tc_error = false;
        } else {
            sample.tc_error = true;
            sample.temperatura = 0.0f;

            if (sample.en_proceso) {
                ESP_LOGE("SAFETY", "Falla de termocupla detectada durante tueste. Abortando proceso...");
                set_safe_state(&sample, true);
            }
        }

        /** @brief Procesa el estado de emergencia, reposo o tueste activo. */
        if (sample.en_proceso) {
            bool btn1_pressed = gpio_get_level(PIN_BTN_1) == 0;
            bool btn2_pressed = gpio_get_level(PIN_BTN_2) == 0;
            bool btn3_pressed = gpio_get_level(PIN_BTN_3) == 0;

            if (btn1_pressed || btn2_pressed || btn3_pressed) {
                vTaskDelay(pdMS_TO_TICKS(1000));

                if ((btn1_pressed && gpio_get_level(PIN_BTN_1) == 0) ||
                    (btn2_pressed && gpio_get_level(PIN_BTN_2) == 0) ||
                    (btn3_pressed && gpio_get_level(PIN_BTN_3) == 0)) {
                    set_safe_state(&sample, true);
                    ESP_LOGW("SAFETY", "Parada de emergencia activada.");
                    wait_for_buttons_release();
                }
            }
        } else if (sample.parada_emergencia) {
            gpio_set_level(PIN_HEATER_RELAY, RELAY_OFF);
            gpio_set_level(PIN_MOTOR_AGITADOR, 0);
            sample.heater_on = false;
            buzzer_beep(80, 1);

            if (gpio_get_level(PIN_BTN_1) == 0 ||
                gpio_get_level(PIN_BTN_2) == 0 ||
                gpio_get_level(PIN_BTN_3) == 0) {
                vTaskDelay(pdMS_TO_TICKS(20));

                if (gpio_get_level(PIN_BTN_1) == 0 ||
                    gpio_get_level(PIN_BTN_2) == 0 ||
                    gpio_get_level(PIN_BTN_3) == 0) {
                    wait_for_buttons_release();
                    sample.parada_emergencia = false;
                    gpio_set_level(PIN_VENTILADOR, 0);
                    ESP_LOGI("SYSTEM", "Alerta de emergencia limpiada por el usuario.");
                }
            }
        } else {
            gpio_set_level(PIN_HEATER_RELAY, RELAY_OFF);
            gpio_set_level(PIN_MOTOR_AGITADOR, 0); 
            sample.heater_on = false;

            if (gpio_get_level(PIN_BTN_1) == 0 || gpio_get_level(PIN_BTN_2) == 0 || gpio_get_level(PIN_BTN_3) == 0) {
                vTaskDelay(pdMS_TO_TICKS(50));

                if (gpio_get_level(PIN_BTN_1) == 0) {
                    ESP_LOGI("BTN_DEBUG", "¡Boton 1 Presionado! Activando Perfil 1 (196°C)");
                    sample.en_proceso = true;
                    sample.perfil_seleccionado = 1;
                    sample.temp_target = 100.0f;
                    tiempo_total_sec = 2 * 60;
                } else if (gpio_get_level(PIN_BTN_2) == 0) {
                    ESP_LOGI("BTN_DEBUG", "¡Boton 2 Presionado! Activando Perfil 2 (210°C)");
                    sample.en_proceso = true;
                    sample.perfil_seleccionado = 2;
                    sample.temp_target = 120.0f;
                    tiempo_total_sec = 2 * 60;
                } else if (gpio_get_level(PIN_BTN_3) == 0) {
                    ESP_LOGI("BTN_DEBUG", "¡Boton 3 Presionado! Activando Perfil 3 (231°C)");
                    sample.en_proceso = true;
                    sample.perfil_seleccionado = 3;
                    sample.temp_target = 140.0f;
                    tiempo_total_sec = 2 * 60;
                }
                sample.tiempo_restante_sec = tiempo_total_sec;
                if (sample.en_proceso) {
                    wait_for_buttons_release();
                    gpio_set_level(PIN_VENTILADOR, 0);
                    buzzer_beep(100, 2);
                }
            }
        }
        
        /** @brief Controla agitador y calefactor mediante histéresis. */
        if (sample.en_proceso && !sample.tc_error) {
            gpio_set_level(PIN_MOTOR_AGITADOR, 1);
            gpio_set_level(PIN_VENTILADOR, 0);

            if (sample.temperatura < (sample.temp_target - HYSTERESIS)) {
                gpio_set_level(PIN_HEATER_RELAY, RELAY_ON);
                sample.heater_on = true;
            } else if (sample.temperatura > (sample.temp_target + HYSTERESIS)) {
                gpio_set_level(PIN_HEATER_RELAY, RELAY_OFF);
                sample.heater_on = false;
            }

            if (tiempo_total_sec > 0) {
                tiempo_total_sec--;
                sample.tiempo_restante_sec = tiempo_total_sec;
            } else {
                set_safe_state(&sample, true);
                sample.parada_emergencia = false;
                buzzer_beep(400, 3);
            }
        }

        /** @brief Publica la telemetría por consola y para la tarea LCD. */
        printf("%s,%.2f,%.2f,%d,%d,%d\n", 
               sample.time_str,
               sample.temperatura,
               sample.temp_target,
               sample.heater_on ? 1 : 0,
               sample.en_proceso ? 1 : 0,
               gpio_get_level(PIN_VENTILADOR)
        );

        /** @brief Registra la muestra en SD si el montaje está disponible. */
        log_telemetry_to_sd(&sample);

        xQueueSend(sensorQueue, &sample, pdMS_TO_TICKS(50));
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Recibe muestras de telemetría y actualiza la pantalla LCD.
 * @param pvParameters Parámetros de tarea FreeRTOS (no utilizados).
 * @return No devuelve un valor; la tarea se ejecuta indefinidamente.
 */
void vTaskLCDDisplay(void *pvParameters) {
    SensorData_t data;
    char buffer[24];

    lcd_init_parallel();
    bool proceso_anterior = false;

    while (1) {
        if (xQueueReceive(sensorQueue, &data, portMAX_DELAY) == pdTRUE) {

            if (proceso_anterior && !data.en_proceso) {
                lcd_send_cmd(0x01);
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            proceso_anterior = data.en_proceso;

            if (!data.en_proceso) {
                if (data.parada_emergencia) {
                    lcd_set_cursor(0, 0);
                    lcd_print_string("! CANCELADO !   ");

                    lcd_set_cursor(0, 1);
                    snprintf(buffer, sizeof(buffer), "%3.0fC Enfriando..",
                             data.temperatura);
                    lcd_print_string(buffer);
                } else {
                    lcd_set_cursor(0, 0);
                    snprintf(buffer, sizeof(buffer), "Hora: %s  ", data.time_str);
                    lcd_print_string(buffer);

                    lcd_set_cursor(0, 1);
                    lcd_print_string("Elija P1, P2 o P3");
                }
            } else {
                int min_rest = data.tiempo_restante_sec / 60;
                int sec_rest = data.tiempo_restante_sec % 60;

                lcd_set_cursor(0, 0);
                if (data.tc_error) {
                    lcd_print_string("Err Termocupla  ");
                } else {
                    snprintf(buffer, sizeof(buffer), "%3.0fC %02d:%02dm %s", 
                             data.temperatura, min_rest, sec_rest, data.heater_on ? "*" : " ");
                    lcd_print_string(buffer);
                }

                lcd_set_cursor(0, 1);
                snprintf(buffer, sizeof(buffer), "P%d:%3.0fC %s", 
                         data.perfil_seleccionado, data.temp_target, data.time_str);
                lcd_print_string(buffer);
            }
        }
    }
}

/** @} */

/**
 * @brief Punto de entrada de la aplicación: configura buses, GPIO y tareas.
 * @return No devuelve un valor.
 */
void app_main(void) {
    rtc_cpu_freq_config_t max_freq;
    rtc_clk_cpu_freq_mhz_to_config(80, &max_freq);
    rtc_clk_cpu_freq_set_config_fast(&max_freq);

    ESP_LOGI(TAG, "Iniciando sistema...");

    sensorQueue = xQueueCreate(5, sizeof(SensorData_t));

    /** @brief Inicializa el chip-select del MAX6675 en estado inactivo. */
    gpio_set_direction(PIN_NUM_MAX_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_MAX_CS, 1);

    /** @brief Restablece los pines e inicializa el bus I2C del RTC. */
    gpio_reset_pin(I2C_SDA_PIN);
    gpio_reset_pin(I2C_SCL_PIN);

    i2c_master_bus_config_t i2c_bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus));

    i2c_device_config_t rtc_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DS3231_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &rtc_dev_cfg, &rtc_dev_handle));

    /** @brief Inicializa SPI2 compartido por MAX6675 y MicroSD. */
    spi_bus_config_t spi_bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));

    /** @brief Registra el sensor MAX6675 en el bus SPI2. */
    spi_device_interface_config_t max6675_cfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_NUM_MAX_CS,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &max6675_cfg, &max6675_spi_handle));

    /** @brief Monta la MicroSD usando el bus SPI existente. */
    init_sd_card(SPI2_HOST);

    /** @brief Valida o inicializa la fecha y hora del RTC. */
    ESP_ERROR_CHECK(ds3231_init_or_update(rtc_dev_handle, false));

    /** @brief Configura el botón con pull-up interno y los botones externos. */
    gpio_config_t btn3_conf = {
        .pin_bit_mask = (1ULL << PIN_BTN_3),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&btn3_conf));

    gpio_config_t btn_external_conf = {
        .pin_bit_mask = (1ULL << PIN_BTN_1) | (1ULL << PIN_BTN_2),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&btn_external_conf));

    /** @brief Configura relé, motor, ventilador y buzzer como salidas. */
    gpio_config_t relay_conf = {
        .pin_bit_mask = (1ULL << PIN_HEATER_RELAY) | (1ULL << PIN_MOTOR_AGITADOR) |
                        (1ULL << PIN_VENTILADOR) | (1ULL << PIN_BUZZER),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&relay_conf);
    
    gpio_set_level(PIN_HEATER_RELAY, RELAY_OFF);
    gpio_set_level(PIN_MOTOR_AGITADOR, 0);
    gpio_set_level(PIN_VENTILADOR, 0);
    gpio_set_level(PIN_BUZZER, 0);

    /** @brief Crea las tareas de muestreo y actualización del LCD. */
    xTaskCreate(vTaskSampling, "sampling_task", 4096, NULL, 5, NULL);
    xTaskCreate(vTaskLCDDisplay, "lcd_task", 3072, NULL, 4, NULL);
}