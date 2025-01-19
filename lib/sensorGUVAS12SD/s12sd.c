/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file s12sd.c
 *
 * ESP-IDF driver for GUVA-S12SD UV sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include <string.h>
#include <stdio.h>
#include <sdkconfig.h>
#include <esp_system.h>
#include <esp_types.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include "s12sd.h"

/*
 * static constant declerations
 */
static const char *TAG = "envnode⇝s12sd";

/*
 * manejadores globles del adc.
 */
adc_oneshot_unit_handle_t adc_oneshot_handle;
adc_cali_handle_t adc_calibrate_handle;

/*
 * functions and subrountines
 */

/**
 * @brief converts millivolt (0 to 1500mV) to uv index. see GUVA-S12SD datasheet for details.
 *
 * @param[in] milli_volt voltage, in millivolts, to convert
 *
 * @return uv index (0 to 11), an out-of-range value returns 255
 */
static inline uint8_t adc_s12sd_convert_uv_index(const float milli_volt)
{
    if (milli_volt > ADC_UV_MV_TO_INDEX_0_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_0_MAX)
    {
        return 0;
    }
    else if (milli_volt > ADC_UV_MV_TO_INDEX_1_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_1_MAX)
    {
        return 1;
    }
    else if (milli_volt > ADC_UV_MV_TO_INDEX_2_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_2_MAX)
    {
        return 2;
    }
    else if (milli_volt > ADC_UV_MV_TO_INDEX_3_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_3_MAX)
    {
        return 3;
    }
    else if (milli_volt > ADC_UV_MV_TO_INDEX_4_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_4_MAX)
    {
        return 4;
    }
    else if (milli_volt > ADC_UV_MV_TO_INDEX_5_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_5_MAX)
    {
        return 5;
    }
    else if (milli_volt > ADC_UV_MV_TO_INDEX_6_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_6_MAX)
    {
        return 6;
    }
    else if (milli_volt > ADC_UV_MV_TO_INDEX_7_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_7_MAX)
    {
        return 7;
    }
    else if (milli_volt > ADC_UV_MV_TO_INDEX_8_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_8_MAX)
    {
        return 8;
    }
    else if (milli_volt > ADC_UV_MV_TO_INDEX_9_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_9_MAX)
    {
        return 9;
    }
    else if (milli_volt > ADC_UV_MV_TO_INDEX_10_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_10_MAX)
    {
        return 10;
    }
    else if (milli_volt > ADC_UV_MV_TO_INDEX_11_MIN && milli_volt <= ADC_UV_MV_TO_INDEX_11_MAX)
    {
        return 11;
    }
    else
    {
        return 255;
    }
}

esp_err_t adc_s12sd_init(void)
{
    esp_err_t ret = ESP_OK;

    adc_oneshot_unit_init_cfg_t init_conf = {
        .unit_id = ADC_UNIT_1,
    };

    adc_oneshot_chan_cfg_t os_conf = {
        .bitwidth = ADC_UV_DIGI_BIT_WIDTH,
        .atten = ADC_UV_ATTEN,
    };

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_6,
        .atten = ADC_UV_ATTEN,
        .bitwidth = ADC_UV_DIGI_BIT_WIDTH,
    };

    ESP_LOGW(TAG, "configurando convertidor a/d[1], canal[6]…");
    ESP_GOTO_ON_ERROR(adc_oneshot_new_unit(&init_conf, &adc_oneshot_handle), err,
                      TAG, "error: falló al configurar a/d para el sensor s12sd.");

    ESP_GOTO_ON_ERROR(adc_oneshot_config_channel(adc_oneshot_handle, ADC_CHANNEL_6, &os_conf),
                      err, TAG, "error: falló al configurar el canal a/d s12sd.");

    ESP_LOGI(TAG, "calibrando convertidor a/d[1], canal[6]… esquema de calibración: curve fitting.");
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &adc_calibrate_handle);

    return ESP_OK;

err:
    adc_oneshot_del_unit(adc_oneshot_handle);
    adc_cali_delete_scheme_curve_fitting(adc_calibrate_handle);
    return ret;
}

esp_err_t adc_s12sd_measure(uint8_t *uv_index)
{
    esp_err_t ret = ESP_OK;
    int avg_sum = 0;
    float avg_volt = 0;

    for (int i = 0; i < ADC_UV_SAMPLE_SIZE; i++)
    {
        int adc_raw;
        int adc_volt;

        ret = adc_oneshot_read(adc_oneshot_handle, ADC_CHANNEL_6, &adc_raw);

        if (ret == ESP_OK)
        {
            ret = adc_cali_raw_to_voltage(adc_calibrate_handle, adc_raw, &adc_volt);

            if (ret != ESP_OK)
                return ret;

            avg_sum += adc_volt;
        }
        else
            return ESP_ERR_INVALID_STATE;
    }

    // average voltage (mV)
    avg_volt = avg_sum / ADC_UV_SAMPLE_SIZE;

    // convert voltage to uv index
    *uv_index = adc_s12sd_convert_uv_index(avg_volt);

    ESP_LOGI(TAG, "a/d[1]-ch[6]. Muestras=%d, Voltaje=%.2f mV.", ADC_UV_SAMPLE_SIZE, avg_volt);
    return ESP_OK;
}

esp_err_t adc_s12sd_deinit(void)
{
    esp_err_t ret = ESP_OK;

    ret = adc_oneshot_del_unit(adc_oneshot_handle);
    ret = adc_cali_delete_scheme_curve_fitting(adc_calibrate_handle);

    return ret;
}