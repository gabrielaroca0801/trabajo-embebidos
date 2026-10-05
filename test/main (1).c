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
static const char *TAG = "MAIN_SYSTEM";

/* =========================================================================
 * DEFINICIONES DE HARDWARE Y CONSTANTES
 * ========================================================================= */

// Bus I2C (DS3231)

#define I2C_SDA_PIN      21     /**< GPIO para la línea de datos SDA */
#define I2C_SCL_PIN      22     /**< GPIO para la línea de reloj SCL */
#define DS3231_I2C_ADDR  0x68   /**< Dirección I2C del RTC DS3231 */

// Bus SPI (MAX6675)

#define PIN_NUM_MISO     GPIO_NUM_19  /**< GPIO para la línea MISO / SO */
#define PIN_NUM_CLK      GPIO_NUM_18  /**< GPIO para el Reloj SCK / SCLK */
#define PIN_NUM_MAX_CS   GPIO_NUM_4   /**< Chip Select exclusivo para el sensor MAX6675 */

// Pines LCD 16x2 Paralela (Modo 4 bits)

#define LCD_RS           GPIO_NUM_25
#define LCD_E            GPIO_NUM_26
#define LCD_D4           GPIO_NUM_27
#define LCD_D5           GPIO_NUM_14
#define LCD_D6           GPIO_NUM_16
#define LCD_D7           GPIO_NUM_13

// Botones de selección de perfil
#define PIN_BTN_1          GPIO_NUM_34  /**< GPIO para el botón de selección del Perfil 1 :196°C */
#define PIN_BTN_2          GPIO_NUM_35  /**< GPIO para el botón de selección del Perfil 2 :210°C*/
#define PIN_BTN_3          GPIO_NUM_32  /**< GPIO para el botón de selección del Perfil 3 :231°C*/

// Pin del Relé
#define PIN_HEATER_RELAY   GPIO_NUM_33  /**< Relé de control de la resistencia (110V) */
#define HYSTERESIS         2.0f

// motor y ventilador:
#define PIN_MOTOR_AGITADOR   GPIO_NUM_2   /**< GPIO para el control del motor de espátulas */
#define PIN_VENTILADOR       GPIO_NUM_15  /**< GPIO para el control del ventilador */

// Punto de montaje para la tarjeta SD
#define PIN_NUM_MOSI    23         /**< GPIO para la línea MOSI / DI (Requerido por la SD) */
#define PIN_NUM_SD_CS   5          /**< Chip Select exclusivo para la tarjeta MicroSD */
#define MOUNT_POINT "/sdcard"

/* =========================================================================
 * ESTRUCTURAS DE DATOS Y VARIABLES GLOBALES
 * ========================================================================= */

/**
 * @brief Estructura de telemetría para compartir datos entre tareas de FreeRTOS.
 */

typedef struct {

    char time_str[16];  /**< Cadena con la hora formateada HH:MM:SS */
    char date_str[16];   /**< Cadena con la fecha formateada DD/MM/YYYY */
    float temperatura;  /**< Lectura actual del sensor de temperatura (°C) */   
    bool tc_error;      /**< Bandera de falla de lectura en la termocupla */
    bool heater_on;     /**< Bandera de estado del calentador */
    
    // Estado del proceso
    bool en_proceso;             /**< Estado del tueste (true: activo, false: reposo) */
    int perfil_seleccionado;    /**< Perfil de tueste seleccionado (1, 2 o 3) */
    float temp_target;          /**< Temperatura objetivo del perfil seleccionado (°C) */
    int tiempo_restante_sec;    /**< Segundos restantes del proceso */

} SensorData_t;

/**
 * @brief Cola de FreeRTOS para transferir telemetría a la tarea de la pantalla LCD */
QueueHandle_t sensorQueue;

/** @brief Manejador del dispositivo I2C para el RTC DS3231 */
i2c_master_dev_handle_t rtc_dev_handle;

/**
 * @brief Manejador del dispositivo SPI para la termocupla MAX6675 */
spi_device_handle_t max6675_spi_handle;


/**
 * @brief Convierte un número en formato BCD (Binary Coded Decimal) a Decimal.
 * @param[in] val Valor entero en formato BCD.
 * @return Valor convertido a base decimal de 8 bits (uint8_t).
 */
static inline uint8_t bcd2dec(uint8_t val) { return ((val >> 4) * 10) + (val & 0x0F); }

/**
 * @brief Convierte un número en formato Decimal a BCD (Binary Coded Decimal).
 * @param[in] val Valor entero en base decimal.
 * @return Valor convertido a formato BCD de 8 bits (uint8_t).
 */
static inline uint8_t dec2bcd(uint8_t val) { return ((val / 10) << 4) | (val % 10); }

/**
 * @brief Convierte el nombre abreviado de un mes en texto a su número de mes correspondiente (1-12).
 * @param[in] month_str Cadena de texto con los primeros 3 caracteres del mes en inglés (ej. "Jan", "Feb").
 * @return Número de mes (1 para Enero hasta 12 para Diciembre). Si no coincide, retorna 1 por defecto.
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
 * @brief Inicializa o actualiza la fecha y hora del RTC DS3231 según la hora de compilación.
 * @details Lee el registro de estado (0x0F) para verificar si el bit OSF (Oscillator Stop Flag) está activo.
 *          Si el reloj perdió energía o si \p force_reset es true, parsea las macros de C `__DATE__` y `__TIME__`
 *          para reconfigurar el DS3231 vía I2C y limpia la bandera OSF.
 * 
 * @param[in] dev_handle Manejador I2C del dispositivo DS3231.
 * @param[in] force_reset `true` para forzar la reescritura de la hora de compilación; `false` para reescribir únicamente si la batería de respaldo falló.
 * @return `ESP_OK` si la transacción I2C fue exitosa, o un código de error de ESP-IDF en caso contrario.
 */
esp_err_t ds3231_init_or_update(i2c_master_dev_handle_t dev_handle, bool force_reset) {
    uint8_t reg_status = 0x0F;
    uint8_t status_val = 0;

    // Leer el registro de estado (0x0F)
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_status, 1, &status_val, 1, -1);
    if (ret != ESP_OK) return ret;

    // El bit 7 (OSF) indica si el oscilador se detuvo por falta de energía
    bool osf_flag = (status_val & 0x80) != 0;

    // Solo escribe la hora de compilación si forzamos el reset O si el RTC perdió energía
    if (force_reset || osf_flag) {
        char month_buf[4];
        int day, year, hour, min, sec;
        
        sscanf(__DATE__, "%s %d %d", month_buf, &day, &year);
        sscanf(__TIME__, "%d:%d:%d", &hour, &min, &sec);

        uint8_t month = parse_month(month_buf);
        uint8_t write_buf[8];
        
        write_buf[0] = 0x00; // Registro inicial
        write_buf[1] = dec2bcd(sec);
        write_buf[2] = dec2bcd(min);
        write_buf[3] = dec2bcd(hour);
        write_buf[4] = 1;
        write_buf[5] = dec2bcd(day);
        write_buf[6] = dec2bcd(month);
        write_buf[7] = dec2bcd(year % 100);

        ret = i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), -1);
        if (ret != ESP_OK) return ret;

        // Limpiar el bit OSF (Escribir 0 en el registro 0x0F)
        uint8_t clear_osf[2] = {0x0F, 0x00};
        return i2c_master_transmit(dev_handle, clear_osf, sizeof(clear_osf), -1);
    }

    return ESP_OK;
}

/**
 * @brief Obtiene la fecha y hora actual almacenada en el RTC DS3231.
 * @details Lee 7 bytes de datos por I2C desde el registro `0x00`, los convierte de BCD a decimal 
 *          y llena una estructura estándar `struct tm` de la librería `time.h`.
 * 
 * @param[in]  dev_handle Manejador I2C del dispositivo DS3231.
 * @param[out] timeinfo Puntero a la estructura `struct tm` donde se cargarán los datos de fecha/hora.
 * @return `ESP_OK` si la lectura I2C fue exitosa, o un código de error de ESP-IDF si falla.
 */
esp_err_t ds3231_get_time(i2c_master_dev_handle_t dev_handle, struct tm *timeinfo) {
    uint8_t reg_addr = 0x00;
    uint8_t data[7];

    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, sizeof(data), -1);
    if (ret != ESP_OK) return ret;


    timeinfo->tm_sec  = bcd2dec(data[0] & 0x7F);
    timeinfo->tm_min  = bcd2dec(data[1] & 0x7F);
    timeinfo->tm_hour = bcd2dec(data[2] & 0x3F);
    timeinfo->tm_mday = bcd2dec(data[4] & 0x3F);
    timeinfo->tm_mon  = bcd2dec(data[5] & 0x1F) - 1;
    timeinfo->tm_year = bcd2dec(data[6]) + 100;

    return ESP_OK;

}


/**
 * @brief Genera un pulso de habilitación (Enable) para el controlador LCD HD44780.
 * @details Conmuta la línea LCD_E de ALTO a BAJO para sincronizar el envío de datos 
 *          o comandos hacia la pantalla LCD Paralela. Incluye retardos en microsegundos 
 *          para garantizar los tiempos de establecimiento (setup/hold) del bus.
 */
void lcd_pulse_enable(void) {

    gpio_set_level(LCD_E, 1);
    esp_rom_delay_us(50);
    gpio_set_level(LCD_E, 0);
    esp_rom_delay_us(100);

}

void lcd_write_nibble(uint8_t nibble) {

    gpio_set_level(LCD_D4, (nibble >> 0) & 0x01);
    gpio_set_level(LCD_D5, (nibble >> 1) & 0x01);
    gpio_set_level(LCD_D6, (nibble >> 2) & 0x01);
    gpio_set_level(LCD_D7, (nibble >> 3) & 0x01);
    esp_rom_delay_us(10);
    lcd_pulse_enable();

}

void lcd_send_byte(uint8_t val, uint8_t is_data) {

    gpio_set_level(LCD_RS, is_data);
    esp_rom_delay_us(10);
    
    lcd_write_nibble(val >> 4);   // Nibble alto
    lcd_write_nibble(val & 0x0F); // Nibble bajo
}

void lcd_send_cmd(uint8_t cmd) {

    lcd_send_byte(cmd, 0);

    if (cmd == 0x01 || cmd == 0x02) {
        vTaskDelay(pdMS_TO_TICKS(10)); // Tiempo extra para Clear Display / Return Home
    }
    else {
        esp_rom_delay_us(200);
    }

}

void lcd_send_char(char c) {

    lcd_send_byte(c, 1);
    esp_rom_delay_us(100);

}

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

    // Secuencia de inicialización modo 4 bits

    lcd_write_nibble(0x03);
    vTaskDelay(pdMS_TO_TICKS(10));

    lcd_write_nibble(0x03);
    vTaskDelay(pdMS_TO_TICKS(5));

    lcd_write_nibble(0x03);
    esp_rom_delay_us(500);

    lcd_write_nibble(0x02); // Establecer oficialmente el modo de 4 bits
    vTaskDelay(pdMS_TO_TICKS(5));

    // Configuración de pantalla

    lcd_send_cmd(0x28); // 4 bits, 2 líneas, fuente 5x8
    lcd_send_cmd(0x08); // Apagar pantalla
    lcd_send_cmd(0x01); // Limpiar pantalla
    vTaskDelay(pdMS_TO_TICKS(10));
    lcd_send_cmd(0x06); // Incremento de cursor a la derecha
    lcd_send_cmd(0x0C); // Encender pantalla sin cursor

}

void lcd_set_cursor(uint8_t col, uint8_t row) {

    uint8_t offsets[] = {0x00, 0x40};
    if (row > 1) row = 1;
    lcd_send_cmd(0x80 | (col + offsets[row]));

}

void lcd_print_string(const char *str) {

    while (*str) {
        lcd_send_char(*str++);
    }

}



/**
 * @brief Inicializa y monta el sistema de archivos VFS FAT en la tarjeta MicroSD.
 * @details Configura el driver SDSPI y crea la cabecera en el archivo tueste.csv si es nuevo.
 */
void init_sd_card(void) {
    esp_err_t ret;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false, // No formatear si hay error para cuidar datos
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t *card;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    // Configuración del puerto de la Tarjeta SD
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_SD_CS; // Pin GPIO 13 para seleccionar la SD
    slot_config.host_id = host.slot;

    // Montar el sistema de archivos VFS FAT en la MicroSD
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE("SD", "Error al montar el sistema de archivos FAT.");
        } else {
            ESP_LOGE("SD", "Error al inicializar la tarjeta SD (%s).", esp_err_to_name(ret));
        }
        return;
    }

    ESP_LOGI("SD", "Tarjeta MicroSD montada con éxito en %s", MOUNT_POINT);

    // Crear o escribir el encabezado del archivo CSV si es nuevo
    FILE *f = fopen("/sdcard/tueste.csv", "a");
    if (f != NULL) {
        // Escribe la cabecera del archivo si está vacío
        fseek(f, 0, SEEK_END);
        if (ftell(f) == 0) {
            fprintf(f, "Tiempo,Temperatura_Real,Temperatura_Target,Relay_Status\n");
        }
        fclose(f);
    }
}

/**
 * @brief Lee la temperatura actual desde el módulo MAX6675 vía SPI.
 * @param[out] temp_out Puntero donde se almacenará el valor de temperatura leído (°C).
 * @return true si la lectura fue exitosa, false si hay error o la termocupla está desconectada.
 */
bool max6675_read_temp(float *temp_out) {
    uint16_t raw_data = 0;
    spi_transaction_t t = {
        .length = 16,
        .rx_buffer = &raw_data,
        .tx_buffer = NULL
    };

    esp_err_t ret = spi_device_transmit(max6675_spi_handle, &t);

    if (ret == ESP_OK) {
        raw_data = __builtin_bswap16(raw_data);

        // Bit 2: Termocupla desconectada o en circuito abierto
        if (raw_data & 0x04) {
            return false;
        }

        *temp_out = (raw_data >> 3) * 0.25f;
        return true;
    }

    return false;
}

/* =========================================================================
 * TAREAS DE FREERTOS
 * ========================================================================= */

/**
 * @brief Tarea principal de muestreo, control térmico, telemetría y logging en SD.
 * @details Se ejecuta periódicamente cada 1000 ms. Realiza la lectura de sensores,
 *          gestiona el algoritmo de control por histéresis, escribe en la tarjeta SD
 *          y envía la información a la cola de la pantalla LCD.
 * 
 * @param[in] pvParameters Parámetros recibidos al crear la tarea (no utilizado).
 */

void vTaskSampling(void *pvParameters) {
    SensorData_t sample = {0};
    struct tm timeinfo;
    
    int tiempo_total_sec = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // 1. Obtener Hora del RTC
        if (ds3231_get_time(rtc_dev_handle, &timeinfo) == ESP_OK) {
            strftime(sample.time_str, sizeof(sample.time_str), "%H:%M:%S", &timeinfo);
        } else {
            snprintf(sample.time_str, sizeof(sample.time_str), "RTC ERR");
        }

        // 2. Leer Termocupla
        if (max6675_read_temp(&sample.temperatura)) {
            sample.tc_error = false;
        } else {
            sample.tc_error = true;
            sample.temperatura = 0.0f;
        }

    // 3. Lógica de Selección por Botones (si no hay proceso activo)
        // Lógica de Selección por Botones (si no hay proceso activo)
        if (!sample.en_proceso) {
            gpio_set_level(PIN_HEATER_RELAY, 0); 
            gpio_set_level(PIN_MOTOR_AGITADOR, 0); 
            gpio_set_level(PIN_VENTILADOR, 0);     
            sample.heater_on = false;

            // Verificar si algún botón fue presionado (estado bajo = 0V)
            if (gpio_get_level(PIN_BTN_1) == 0 || gpio_get_level(PIN_BTN_2) == 0 || gpio_get_level(PIN_BTN_3) == 0) {
                vTaskDelay(pdMS_TO_TICKS(50)); // Antirrebote por software (50 ms)

                if (gpio_get_level(PIN_BTN_1) == 0) {
                    sample.en_proceso = true;
                    sample.perfil_seleccionado = 1;
                    sample.temp_target = 196.0f; // Ajuste a temperatura real de tueste
                    tiempo_total_sec = 2 * 60;
                } else if (gpio_get_level(PIN_BTN_2) == 0) {
                    sample.en_proceso = true;
                    sample.perfil_seleccionado = 2;
                    sample.temp_target = 210.0f; // Ajuste a temperatura real de tueste
                    tiempo_total_sec = 2 * 60;
                } else if (gpio_get_level(PIN_BTN_3) == 0) {
                    sample.en_proceso = true;
                    sample.perfil_seleccionado = 3;
                    sample.temp_target = 231.0f; // Ajuste a temperatura real de tueste
                    tiempo_total_sec = 2 * 60;
                }
                sample.tiempo_restante_sec = tiempo_total_sec;
            }
        }
        
        // 4. Lógica durante el proceso activo
        if (sample.en_proceso) {
            // Activar agitador continuamente durante el tueste
            gpio_set_level(PIN_MOTOR_AGITADOR, 1);
            gpio_set_level(PIN_VENTILADOR, 0);

            // Control ON/OFF con Histéresis para la resistencia
            if (!sample.tc_error) {
                if (sample.temperatura < (sample.temp_target - HYSTERESIS)) {
                    gpio_set_level(PIN_HEATER_RELAY, 1);
                    sample.heater_on = true;
                } else if (sample.temperatura > (sample.temp_target + HYSTERESIS)) {
                    gpio_set_level(PIN_HEATER_RELAY, 0);
                    sample.heater_on = false;
                }
            } else {
                gpio_set_level(PIN_HEATER_RELAY, 0);
                sample.heater_on = false;
            }

            // Descontar 1 segundo
            if (tiempo_total_sec > 0) {
                tiempo_total_sec--;
                sample.tiempo_restante_sec = tiempo_total_sec;
            } else {
                // Proceso Finalizado -> Entrar a estado seguro / Apagado
                sample.en_proceso = false;
                gpio_set_level(PIN_HEATER_RELAY, 0);
                gpio_set_level(PIN_MOTOR_AGITADOR, 0);
                gpio_set_level(PIN_VENTILADOR, 1); // Puedes encender el ventilador por unos segundos para enfriar
                sample.heater_on = false;
                sample.perfil_seleccionado = 0;
            }
        }

                          
          // 5. Envío de telemetría por Puerto Serial (Formato CSV con Timestamp) si esta conectado a un PC
        // Imprime: HH:MM:SS, Temperatura, Temp_Target, Heater_ON, Motor_ON, Ventilador_ON
        printf("%s,%.2f,%.2f,%d,%d,%d\n", 
               sample.time_str,
               sample.temperatura,
               sample.temp_target,
               sample.heater_on ? 1 : 0,
               (sample.en_proceso && sample.tiempo_restante_sec > 0) ? 1 : 0, // Agitador
               (!sample.en_proceso && sample.perfil_seleccionado == 0) ? 1 : 0  // Ventilador
        );


        // --- GUARDADO EN TARJETA SD ---
        FILE *f = fopen("/sdcard/tueste.csv", "a"); // "a" para append (agregar al final)
        if (f != NULL) {
            fprintf(f, "%s,%.2f,%.2f,%d\n", 
                    sample.time_str,
                    sample.temperatura,
                    sample.temp_target,
                    sample.heater_on ? 1 : 0);
            fclose(f); // Cerrar inmediatamente para asegurar que los datos se escriban
        } else {
            ESP_LOGE("SD", "Error al abrir el archivo tueste.csv");
        }

        xQueueSend(sensorQueue, &sample, pdMS_TO_TICKS(50));
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000)); // Periodo exacto de 1 segundo
    }
}

/**
 * @brief Tarea encarga de la actualización de la pantalla LCD Paralela.
 * @details Recibe los datos de la cola `sensorQueue` y formatea la interfaz gráfica
 *          según el estado del sistema (Menú de Selección o Proceso Activo).
 * 
 * @param[in] pvParameters Parámetros recibidos al crear la tarea (no utilizado).
 */
void vTaskLCDDisplay(void *pvParameters) {
    SensorData_t data;
    char buffer[24];

    lcd_init_parallel();

    bool proceso_anterior = false; // Variable local para detectar transición

    while (1) {
        if (xQueueReceive(sensorQueue, &data, portMAX_DELAY) == pdTRUE) {

            // Si el proceso acaba de terminar, limpiar pantalla antes de mostrar el menú
            if (proceso_anterior && !data.en_proceso) {
                lcd_send_cmd(0x01); // Comando Clear Display
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            proceso_anterior = data.en_proceso;


            if (!data.en_proceso) {
                // Modo Selección: Muestra mensaje y la Hora actual
                lcd_set_cursor(0, 0);
                snprintf(buffer, sizeof(buffer), "Hora: %s  ", data.time_str);
                lcd_print_string(buffer);

                lcd_set_cursor(0, 1);
                lcd_print_string("Elija P1, P2 o P3");
            } else {
                // Modo Proceso Activo: Muestra Temperatura, Tiempo restante y Hora
                int min_rest = data.tiempo_restante_sec / 60;
                int sec_rest = data.tiempo_restante_sec % 60;

                // Línea 1: Temp actual y Tiempo Restante (ej: "195C 10:45m  *")
                lcd_set_cursor(0, 0);
                if (data.tc_error) {
                    lcd_print_string("Err Termocupla  ");
                } else {
                    snprintf(buffer, sizeof(buffer), "%3.0fC %02d:%02dm %s", 
                             data.temperatura, min_rest, sec_rest, data.heater_on ? "*" : " ");
                    lcd_print_string(buffer);
                }

                // Línea 2: Perfil y Hora (ej: "P1:196C  14:30:15")
                lcd_set_cursor(0, 1);
                snprintf(buffer, sizeof(buffer), "P%d:%3.0fC %s", 
                         data.perfil_seleccionado, data.temp_target, data.time_str);
                lcd_print_string(buffer);
            }
        }
    }
}

/* =========================================================================
 * FUNCIÓN PRINCIPAL (ENTRY POINT)
 * ========================================================================= */

/**
 * @brief Punto de entrada principal de la aplicación en ESP-IDF.
 * @details Configura la frecuencia del reloj, inicializa la cola de FreeRTOS, los buses I2C y SPI,
 *          configura los GPIOs de actuadores y botones, e inicia las tareas del planificador.
 */


void app_main(void) {
    rtc_cpu_freq_config_t max_freq;
    rtc_clk_cpu_freq_mhz_to_config(80, &max_freq);
    rtc_clk_cpu_freq_set_config_fast(&max_freq);

    ESP_LOGI(TAG, "Iniciando sistema...");

    sensorQueue = xQueueCreate(5, sizeof(SensorData_t));

    // 1. Configurar Pines CS de SPI como salidas en ALTO para evitar colisiones
    gpio_set_direction(PIN_NUM_MAX_CS, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_SD_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_MAX_CS, 1);
    gpio_set_level(PIN_NUM_SD_CS, 1);

    // 2. Inicializar I2C para RTC DS3231
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

    // 3. Inicializar Bus SPI HABILITANDO DMA CANAL AUTO
    spi_bus_config_t spi_bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000
    };
    // Habilitar SPI_DMA_CH_AUTO para permitir convivencia entre SD y MAX6675
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_bus_cfg, SPI_DMA_CH_AUTO));

    // 4. Agregar Dispositivo MAX6675 al SPI
    spi_device_interface_config_t max6675_cfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz
        .mode = 0,
        .spics_io_num = PIN_NUM_MAX_CS,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &max6675_cfg, &max6675_spi_handle));

    // 5. Inicializar SD
    init_sd_card();

    // 6. Configurar RTC
    ESP_ERROR_CHECK(ds3231_init_or_update(rtc_dev_handle, false));

    // 7. Configuración de Botones
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << PIN_BTN_1) | (1ULL << PIN_BTN_2) | (1ULL << PIN_BTN_3),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // Funciona en GPIO32; en 34 y 35 se REQUIERE resistencia física externa a 3.3V
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_conf);

    // 8. Actuadores
    gpio_config_t relay_conf = {
        .pin_bit_mask = (1ULL << PIN_HEATER_RELAY) | (1ULL << PIN_MOTOR_AGITADOR) | (1ULL << PIN_VENTILADOR),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&relay_conf);
    
    gpio_set_level(PIN_HEATER_RELAY, 0);
    gpio_set_level(PIN_MOTOR_AGITADOR, 0);
    gpio_set_level(PIN_VENTILADOR, 0);

    // 9. Crear Tareas
    xTaskCreate(vTaskSampling, "sampling_task", 4096, NULL, 5, NULL);
    xTaskCreate(vTaskLCDDisplay, "lcd_task", 3072, NULL, 4, NULL);
}
