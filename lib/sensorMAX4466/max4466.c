
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <esp_system.h>
#include <esp_types.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_err.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Arduino.h"
#include "max4466.h"

#define ADC_CHANNEL 6
#define ADC_RESOLUTION 12 // bits
#define SAMPLE_WINDOW 50
#define SOUND_SAMPLING_AVG 100

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define ADC_MIC_UNIT ADC_UNIT_1
#define ADC_MIC_CHANNEL ADC_CHANNEL_5
#define ADC_MIC_ATTEN ADC_ATTEN_DB_12
#define ADC_MIC_BIT_WIDTH ADC_BITWIDTH_12
#define SAMPLE_ROUNDS 10

static const char *TAG = "envnode⇝max4466";

extern adc_oneshot_unit_handle_t adc_oneshot_handle;
adc_cali_handle_t adc_calibrate_mic_handle;

static long map_values(long x, long in_min, long in_max, long out_min, long out_max)
{
    const long run = in_max - in_min;
    if (run == 0)
    {
        log_e("map(): Invalid input range, min == max");
        return -1; // AVR returns -1, SAM returns 0
    }
    const long rise = out_max - out_min;
    const long delta = x - in_min;
    return (delta * rise) / run + out_min;
}

esp_err_t max4466_init(void)
{
    ESP_LOGW(TAG, "configurando convertidor a/d[1], canal[5]…");

    adc_oneshot_chan_cfg_t os_conf = {
        .bitwidth = ADC_MIC_BIT_WIDTH,
        .atten = ADC_MIC_ATTEN,
    };

    adc_oneshot_config_channel(adc_oneshot_handle, ADC_CHANNEL_5, &os_conf);

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_5,
        .atten = ADC_MIC_ATTEN,
        .bitwidth = ADC_MIC_BIT_WIDTH,
    };

    ESP_LOGI(TAG, "calibrando a/d[1], canal[5]… esquema de calibración: curve fitting.");
    adc_cali_create_scheme_curve_fitting(&cali_config, &adc_calibrate_mic_handle);

    return ESP_OK;
}

void map_max4466_samples(void)
{
    uint32_t voltage_amp = 0;
    uint32_t max_voltage = 0;
    uint32_t min_voltage = 3300;
    uint32_t amp_db_noise = 0;
    uint32_t rectified_amp_db_noise = 0;
    uint8_t spl_ref = 20;
    uint8_t max4466_gain = 200;
    float electret_sensitivity = 6.31;
    float mv_ref = (electret_sensitivity * max4466_gain);

    while (true)
    {
        uint32_t start_time = millis();

        while (millis() - start_time < SAMPLE_WINDOW)
        {
            int adc_raw = 0;
            int adc_volt = 0;

            adc_oneshot_read(adc_oneshot_handle, ADC_CHANNEL_5, &adc_raw);
            adc_cali_raw_to_voltage(adc_calibrate_mic_handle, adc_raw, &adc_volt);

            if (adc_volt > max_voltage)
                max_voltage = adc_volt;
            if (adc_volt < min_voltage)
                min_voltage = adc_volt;
        }

        voltage_amp = max_voltage - min_voltage;
        amp_db_noise = 20 * log10((voltage_amp / mv_ref) / 20e-6) - spl_ref;
        rectified_amp_db_noise = map_values(amp_db_noise, 60, 120, 40, 100);

        ESP_LOGI(TAG, "Amplitud del sonido: %lu mV.", voltage_amp);
        ESP_LOGI(TAG, "Ruido ambiente: %lu dB SPL.", amp_db_noise);
        // ESP_LOGI(TAG, "Ruido ambiente rectificado: %lu dB SPL.", rectified_amp_db_noise);

        voltage_amp = 0;
        amp_db_noise = 0;
        rectified_amp_db_noise = 0;
        max_voltage = 0;
        min_voltage = 3300;

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t max4466_samples_read(float *noise_sample)
{
    uint32_t start_time = millis(); // inicio de la ventana de muestreo.
    uint32_t voltage_amp = 0;
    uint32_t max_voltage = 0;
    uint32_t min_voltage = 3300;

    while (millis() - start_time < SAMPLE_WINDOW)
    {
        int adc_raw = 0;
        int adc_volt = 0;

        adc_oneshot_read(adc_oneshot_handle, ADC_CHANNEL_5, &adc_raw);
        adc_cali_raw_to_voltage(adc_calibrate_mic_handle, adc_raw, &adc_volt);

        if (adc_volt > max_voltage)
            max_voltage = adc_volt;
        if (adc_volt < min_voltage)
            min_voltage = adc_volt;
    }

    voltage_amp = max_voltage - min_voltage;
    *noise_sample = voltage_amp;
    vTaskDelay(pdMS_TO_TICKS(1));

    return ESP_OK;
}

esp_err_t max4466_average_read(uint8_t *db_noise)
{
    uint8_t total_samples[SOUND_SAMPLING_AVG];
    uint8_t avg_noise = 0;
    esp_err_t ret;

    for (int i = 0; i < SOUND_SAMPLING_AVG; i++)
    {
        float sample_noise = 0;
        ret = max4466_samples_read(&sample_noise);
        total_samples[i] = sample_noise;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    max4466_process_samples(total_samples, SOUND_SAMPLING_AVG, &avg_noise);
    vTaskDelay(pdMS_TO_TICKS(1000));

    *db_noise = avg_noise;
    return ESP_OK;
}

void max4466_process_samples(uint8_t *arr, int size, uint8_t *avg_noise)
{
    float avg_sum = 0;
    float square_sum = 0;
    float bias_shift = 1650;
    uint8_t spl_ref = 12;
    uint8_t max4466_gain = 125;
    float electret_sensitivity = 6.31;
    float mv_ref = (electret_sensitivity * max4466_gain);

    /* Suma de cuadrados */
    for (int i = 0; i < size; i++)
    {
        avg_sum += arr[i];
        square_sum += arr[i] * arr[i];
    }

    float samples_avg = avg_sum / size;
    float stand_dev = sqrt((square_sum / size) - (samples_avg * samples_avg));

    ESP_LOGE(TAG, "Media: %.2f mV.", samples_avg);
    ESP_LOGE(TAG, "Desviación estándar: %.2f.", stand_dev);

    /* Eliminar valores atípicos */
    int new_samples_num = 0;
    int new_samples_arr[size];

    for (int i = 0; i < size; i++)
    {
        if (abs(arr[i] - samples_avg) <= 2 * stand_dev)
        {
            new_samples_arr[new_samples_num++] = arr[i];
        }
    }

    uint32_t new_avg_sum = 0;
    for (int i = 0; i < new_samples_num; i++)
    {
        new_avg_sum += new_samples_arr[i];
    }

    float new_avg_voltage = new_avg_sum / new_samples_num;
    ESP_LOGE(TAG, "Media filtrada: %.2f mV.", new_avg_voltage);

    // Calibración con nuevos valores de referencia y obtención de dB SPL.
    // amp_db_noise = spl_ref + (20 * log10(voltage_amp / mv_ref));
    uint8_t avg_db_noise = 20 * log10((new_avg_voltage / mv_ref) / 20e-6) - spl_ref;
    uint8_t rectified_avg_db_noise = map_values(avg_db_noise, 60, 120, 40, 100);

    *avg_noise = avg_db_noise;
}
