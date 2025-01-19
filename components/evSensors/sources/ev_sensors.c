
#include <stdint.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_sleep.h>
#include "dht.h"
#include "mhz19b.h"
#include "s12sd.h"
#include "max4466.h"
#include "ev_lora.h"

#define ENV_SENSORS_LOG_TAG "envnode⇝sensors"

// Configuraciones para el sensor DHT22.
#define DHT_SENSOR_TYPE DHT_TYPE_AM2301
#define DHT22_GPIO_NUM GPIO_NUM_5

// Configuraciones para el sensor MHZ19B.
#define MHZ_UART_TX_PIN GPIO_NUM_4
#define MHZ_UART_RX_PIN GPIO_NUM_3
mhz19b_dev_t dev;
char version[6];
uint16_t range;
bool autocal;

// Variables de contención para los datos del entorno.
float env_temperature = 0.;
float env_humidity = 0.;
int8_t cut_env_temperature = 0;
uint8_t cut_env_humidity = 0;
uint8_t env_uv_index = 0;
uint8_t env_noise = 0;
int16_t env_co2 = 0;

uint8_t macframe_payload[] = {};

#define LORAWAN_UPLINK_US 7.8e8               // 13 minutos en us.
#define LORAWAN_UPLINK_MS_RETRY 60000         // 1 minuto en ms.
#define LORAWAN_JOIN_MS_RETRY 60000           // 1 minutos en ms.
#define LORAWAN_JOIN_US_DEEP_RETRY 1200000000 // 20 minutos en us.
#define LORAWAN_MAX_JOIN_RETRIES 5            // 5 veces.
#define LORAWAN_MAX_UPLINK_RETRIES 5          // 5 veces.

extern bool is_lora_join;
int lorawan_rejoin_attemps = 0;
int lorawan_uplink_attemps = 0;

/**
 * @section Configuración inicial del sensor DHT22.
 */
esp_err_t init_dht22_sensor(void)
{
    ESP_LOGW(ENV_SENSORS_LOG_TAG, "configurando sensor dht22…");
    if (gpio_set_pull_mode(DHT22_GPIO_NUM, GPIO_PULLDOWN_ONLY) != ESP_OK)
    {
        ESP_LOGE(ENV_SENSORS_LOG_TAG, "error: no inició el sensor dht22.");
        return ESP_FAIL;
    }

    ESP_LOGI(ENV_SENSORS_LOG_TAG, "éxito: sensor dht22 listo para leer.");
    return ESP_OK;
}

/**
 * @section Configuración inicial del sensor GUVAS12SD.
 */
esp_err_t init_guvas12sd_sensor(void)
{
    ESP_LOGW(ENV_SENSORS_LOG_TAG, "configurando sensor guvas12sd…");
    if (adc_s12sd_init() != ESP_OK)
    {
        ESP_LOGE(ENV_SENSORS_LOG_TAG, "error: no inició el sensor guvas12sd.");
        return ESP_FAIL;
    }

    ESP_LOGI(ENV_SENSORS_LOG_TAG, "éxito: sensor guvas12sd listo para leer.");
    return ESP_OK;
}

/**
 * @section Configuración inicial del sensor MAX4466.
 */
esp_err_t init_max4466_sensor(void)
{
    ESP_LOGW(ENV_SENSORS_LOG_TAG, "configurando sensor max4466…");
    if (max4466_init() != ESP_OK)
    {
        ESP_LOGE(ENV_SENSORS_LOG_TAG, "error: no inició el sensor max4466.");
        return ESP_FAIL;
    }

    ESP_LOGI(ENV_SENSORS_LOG_TAG, "éxito: sensor max4466 listo para leer.");
    return ESP_OK;
}

/**
 * @section Configuración inicial del sensor MHZ19B.
 */
esp_err_t init_mhz19b_sensor(void)
{
    ESP_LOGW(ENV_SENSORS_LOG_TAG, "configurando sensor mhz19b…");
    ESP_ERROR_CHECK(mhz19b_init(&dev, UART_NUM_2, MHZ_UART_TX_PIN, MHZ_UART_RX_PIN));

    if (!mhz19b_detect(&dev))
    {
        while (!mhz19b_detect(&dev))
        {
            vTaskDelay(pdMS_TO_TICKS(500));
            ESP_LOGW(ENV_SENSORS_LOG_TAG, "…");
        };
    }
    ESP_LOGI(ENV_SENSORS_LOG_TAG, "éxito: sensor mhz19b conectado.");

    mhz19b_get_version(&dev, version);
    ESP_LOGI(ENV_SENSORS_LOG_TAG, "versión del sensor (tx) >> mhz19b responde (rx) >> v%s.", version);

    mhz19b_set_range(&dev, MHZ19B_RANGE_2000);
    mhz19b_set_auto_calibration(&dev, true);

    mhz19b_get_range(&dev, &range);
    ESP_LOGI(ENV_SENSORS_LOG_TAG, "rango de operación (tx) >> mhz19b responde (rx) >> %dppm.", range);
    mhz19b_get_auto_calibration(&dev, &autocal);
    ESP_LOGI(ENV_SENSORS_LOG_TAG, "autocalibración (tx) >> mhz19b responde (rx) >> %s.", autocal ? "sí" : "no");

    ESP_LOGW(ENV_SENSORS_LOG_TAG, "el sensor mhz19b debe precalentarse previo a lecturar datos…");
    while (mhz19b_is_warming_up(&dev, false))
    {
        ESP_LOGW(ENV_SENSORS_LOG_TAG, "…");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    ESP_LOGI(ENV_SENSORS_LOG_TAG, "éxito: sensor mhz19b listo para leer.");
    return ESP_OK;
}

/**
 * @section Calibración zero-point del sensor MHZ19B.
 */
void calibrate_mhz19b_sensor(void)
{
    ESP_LOGW(ENV_SENSORS_LOG_TAG, "PROCESO DE CALIBRACIÓN MHZ19B.");
    ESP_LOGW(ENV_SENSORS_LOG_TAG, "leyendo: recopilando datos en mhz19b por 20 minutos…");
    vTaskDelay(pdMS_TO_TICKS(1200000));
    ESP_LOGW(ENV_SENSORS_LOG_TAG, "leyendo: enviando comando de calibración…");
    mhz19b_start_calibration(&dev);
    ESP_LOGI(ENV_SENSORS_LOG_TAG, "éxito: calibración zeropoint completada.");
}

/**
 * @section Normalizar muestras de sensores.
 */
void normalize_sensors_lectures(void)
{
    ESP_LOGW(ENV_SENSORS_LOG_TAG, "Normalizando valores de sensores…");

    uint32_t normalize_samples = 5; // número de muestras de normalización.
    uint32_t current_sample = 0;

    while (current_sample < normalize_samples)
    {
        current_sample++;

        /**
         * 1) DHT22 - TEMPERATURA Y HUMEDAD.
         */
        ESP_LOGW(ENV_SENSORS_LOG_TAG, "normalizando: sensor dht22…");
        while (dht_read_float_data(DHT_SENSOR_TYPE, DHT22_GPIO_NUM, &env_humidity, &env_temperature) != ESP_OK)
        {
            ESP_LOGE(ENV_SENSORS_LOG_TAG, "error: reintentando lectura en dht22…");
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        ESP_LOGI(ENV_SENSORS_LOG_TAG, "éxito: humedad relativa: %.2f %%.", env_humidity);
        ESP_LOGI(ENV_SENSORS_LOG_TAG, "éxito: temperatura: %.2f °C.", env_temperature);

        /**
         * 2) GUVA-S12SD - RADIACIÓN UV.
         */
        ESP_LOGW(ENV_SENSORS_LOG_TAG, "normalizando sensor guvas12sd…");
        while (adc_s12sd_measure(&env_uv_index) != ESP_OK)
        {
            ESP_LOGE(ENV_SENSORS_LOG_TAG, "error: reintentando lectura en guvas12sd…");
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        ESP_LOGI(ENV_SENSORS_LOG_TAG, "éxito: índice uv: %u índ.", env_uv_index);

        /**
         * 3) MAX4466 - RUIDO AMBIENTAL.
         */
        ESP_LOGW(ENV_SENSORS_LOG_TAG, "normalizando: sensor max4466…");
        while (max4466_average_read(&env_noise) != ESP_OK)
        {
            ESP_LOGE(ENV_SENSORS_LOG_TAG, "error: reintentando lectura en max4466…");
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        ESP_LOGI(ENV_SENSORS_LOG_TAG, "éxito: ruido ambiente: %u dB SPL.", env_noise);

        /**
         * 4) MHZ19B - CO2.
         */
        ESP_LOGW(ENV_SENSORS_LOG_TAG, "normalizando: sensor mhz19b…");
        while (mhz19b_read_co2(&dev, &env_co2) != ESP_OK)
        {
            ESP_LOGE(ENV_SENSORS_LOG_TAG, "error: reintentando lectura en mhz19b…");
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        ESP_LOGI(ENV_SENSORS_LOG_TAG, "éxito: co2: %i ppm.", env_co2);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
 * @section Recuperar datos iniciales.
 */

/*********************************************************
 * @section Tarea freertos de sensado de las variables            *
 * ambientales. No es necesario depender varias tareas           *
 * simultáneas, el sensado no está limitado por tiempo real.  *
 *********************************************************/
void env_sensing_routine(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    while (true)
    {
        /**
         *  @section 0. Revisar el estado de la conexión lorawan.
         */
        if (is_lora_join)
        {
            /**
             *  @section 1. Sensado de las variables temperatura y humedad con el módulo DHT22.
             */
            ESP_LOGW(ENV_SENSORS_LOG_TAG, "leyendo: recuperando valores del sensor dht22…");
            while (dht_read_float_data(DHT_SENSOR_TYPE, DHT22_GPIO_NUM, &env_humidity, &env_temperature) != ESP_OK)
            {
                ESP_LOGE(ENV_SENSORS_LOG_TAG, "error: reintentando lectura en dht22…");
                vTaskDelay(pdMS_TO_TICKS(500));
            }

            ESP_LOGI(ENV_SENSORS_LOG_TAG, "éxito: humedad relativa: %.2f %%.", env_humidity);
            ESP_LOGI(ENV_SENSORS_LOG_TAG, "éxito: temperatura: %.2f °C.", env_temperature);

            // conversión a valores enteros para envío lora.
            cut_env_humidity = (uint8_t)env_humidity;
            cut_env_temperature = (int8_t)env_temperature;

            /**
             *  @section 2. Sensado de la variable de índice uv del sensor GUVAS12SD.
             */
            ESP_LOGW(ENV_SENSORS_LOG_TAG, "leyendo: recuperando valores del sensor guvas12sd…");
            while (adc_s12sd_measure(&env_uv_index) != ESP_OK)
            {
                ESP_LOGE(ENV_SENSORS_LOG_TAG, "error: reintentando lectura en guvas12sd…");
                vTaskDelay(pdMS_TO_TICKS(500));
            }

            ESP_LOGI(ENV_SENSORS_LOG_TAG, "éxito: índice uv: %u índ.", env_uv_index);

            /**
             *  @section 3. Sensado de la variable de rudio ambiente MAX4466.
             */
            ESP_LOGW(ENV_SENSORS_LOG_TAG, "leyendo: recuperando valores del sensor max4466…");
            while (max4466_average_read(&env_noise) != ESP_OK)
            {
                ESP_LOGE(ENV_SENSORS_LOG_TAG, "error: reintentando lectura en max4466…");
                vTaskDelay(pdMS_TO_TICKS(500));
            }

            ESP_LOGI(ENV_SENSORS_LOG_TAG, "éxito: ruido ambiente: %u dB-SPL.", env_noise);

            /**
             *  @section 4. Sensado de las variable de co2 del sensor MHZ19B.
             */
            ESP_LOGW(ENV_SENSORS_LOG_TAG, "leyendo: recuperando valores del sensor mhz19b…");
            while (mhz19b_read_co2(&dev, &env_co2) != ESP_OK)
            {
                ESP_LOGE(ENV_SENSORS_LOG_TAG, "error: reintentando lectura en mhz19b…");
                vTaskDelay(pdMS_TO_TICKS(500));
            }

            ESP_LOGI(ENV_SENSORS_LOG_TAG, "éxito: co2: %i ppm.", env_co2);
            vTaskDelay(pdMS_TO_TICKS(1000));

            /**
             * @section 5. Convertir los datos recopilados a bytes crudos.
             */
            ESP_LOGW(ENV_SENSORS_LOG_TAG, "convirtiendo datos a bytes crudos…");
            sensors_data_bytes_conversion(cut_env_temperature, cut_env_humidity, env_uv_index,
                                          env_noise, env_co2, macframe_payload);
            ESP_LOGI(ENV_SENSORS_LOG_TAG, "conversión exitosa.");

            /**
             * @section 6.transmisión lora y enviarlos al GW TTN.
             */
            ESP_LOGW(ENV_SENSORS_LOG_TAG, "enviando datos al servidor lorawan…");
            if (send_lora_uplink(macframe_payload) == ESP_OK)
            {
                lorawan_uplink_attemps = 0;
                // deep sleep
                ESP_LOGI(ENV_SENSORS_LOG_TAG,
                         "datos enviados, durmiendo módulo por 15 minutos…");
                ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(LORAWAN_UPLINK_US));
                esp_deep_sleep_start();
            }
            else
            {
                lorawan_uplink_attemps++;
                if (lorawan_uplink_attemps >= LORAWAN_MAX_UPLINK_RETRIES)
                {
                    ESP_LOGE(ENV_SENSORS_LOG_TAG,
                             "límite de fallos de envío, reiniciando módulo…");
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    esp_restart();
                }
                else
                    ESP_LOGW(ENV_SENSORS_LOG_TAG,
                             "fallo de envío, reintentando en un minuto…");
                vTaskDelay(pdMS_TO_TICKS(LORAWAN_UPLINK_MS_RETRY));
            }
        }
        else
        {
            lorawan_rejoin_attemps++;

            if (lorawan_rejoin_attemps > LORAWAN_MAX_JOIN_RETRIES)
            {
                // deep sleep
                lorawan_rejoin_attemps = 0;
                ESP_LOGE(ENV_SENSORS_LOG_TAG,
                         "sin conexión con el servidor lora, durmiendo módulo por 20 minutos…");
                ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(LORAWAN_JOIN_US_DEEP_RETRY));
                esp_deep_sleep_start();
            }
            else
            {
                ESP_LOGW(ENV_SENSORS_LOG_TAG,
                         "sin conexión con el servidor lora, reintentando en 1 minuto…");
                vTaskDelay(pdMS_TO_TICKS(LORAWAN_JOIN_MS_RETRY));
                is_lora_join = (join_lorawan_net() == ESP_OK) ? true : false;
            }
        }
    }
}
