#ifndef EV_LORA_H
#define EV_LORA_H

#include <stdint.h>
#include <esp_err.h>

#ifdef __cplusplus__
extern "C"
{
#endif

    esp_err_t init_lora_phy(void);

    esp_err_t set_lora_parameters(void);

    esp_err_t setup_lorawan_node(void);

    esp_err_t join_lorawan_net(void);

    void sensors_data_bytes_conversion(int8_t env_temp, uint8_t env_hum, uint8_t env_uv,
                                       uint8_t env_dB, int16_t env_co2, uint8_t *out_array);

    esp_err_t send_lora_uplink(uint8_t *bytes_data);

#ifdef __cplusplus__
}
#endif

#endif // EV_LORA_H
