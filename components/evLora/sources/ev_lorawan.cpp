
#include <stdbool.h>
#include <esp_log.h>
#include <esp_err.h>
#include <RadioLib.h>
#include <Arduino.h>

#define TAG "envnode⇝lorawan_1.1"
#define LORA_NODE_ID 1

bool is_lora_join = false;

/**
 * @section Configuraciones definidas de la capa mac LoRa.
 * Los ajustes se basan en la definición del estándar loraWan
 * y sus configuraciones de estructura de tramas, políticas de
 * uso, etc.
 */

/** Aprovisionamiento y unión a la red con el servidor LNS. */
uint64_t join_EUI = 0x0000000000000000;
uint64_t dev_EUI = 0x70B3D57ED006C5BA;
uint8_t app_KEY[] = {0x36, 0x18, 0x60, 0xA5, 0xAF, 0xAC,
                     0xE9, 0x86, 0x2E, 0x49, 0x59, 0xB1,
                     0x13, 0x60, 0x15, 0xDB};
uint8_t nwk_KEY[] = {0x38, 0xE8, 0x88, 0x07, 0xD4, 0xB0,
                     0x2C, 0x10, 0x0E, 0x27, 0xE8, 0x85,
                     0xCF, 0xD8, 0x40, 0x57};

/** política de uso justo de The Things Industries. */
#define LORAWAN_UPLINK_DELAY 600000 // 10 minutos en ms.
/** plan de frecuencias regional. */
#define LORAWAN_FREQ_PLAN US915
/** Subbanda MHz compatible con TTN. */
const uint8_t lorawan_sub_band = 1;
/** Configuraciones de la trama mac. */
const uint8_t LORA_FPORT = 7;
bool LORA_CONFIRMED_FRAMES = true;

extern SX1262 radio;
const LoRaWANBand_t region_plan = LORAWAN_FREQ_PLAN;
LoRaWANNode node(&radio, &region_plan, lorawan_sub_band);

extern "C"
{
    // RadioLib has many more states - see https://jgromes.github.io/RadioLib/group__status__codes.html
    static char *state_decode(int16_t result)
    {
        switch (result)
        {
        case RADIOLIB_ERR_NONE:
            return "ERR_NONE";
        case RADIOLIB_ERR_CHIP_NOT_FOUND:
            return "ERR_CHIP_NOT_FOUND";
        case RADIOLIB_ERR_PACKET_TOO_LONG:
            return "ERR_PACKET_TOO_LONG";
        case RADIOLIB_ERR_RX_TIMEOUT:
            return "ERR_RX_TIMEOUT";
        case RADIOLIB_ERR_CRC_MISMATCH:
            return "ERR_CRC_MISMATCH";
        case RADIOLIB_ERR_INVALID_BANDWIDTH:
            return "ERR_INVALID_BANDWIDTH";
        case RADIOLIB_ERR_INVALID_SPREADING_FACTOR:
            return "ERR_INVALID_SPREADING_FACTOR";
        case RADIOLIB_ERR_INVALID_CODING_RATE:
            return "ERR_INVALID_CODING_RATE";
        case RADIOLIB_ERR_INVALID_FREQUENCY:
            return "ERR_INVALID_FREQUENCY";
        case RADIOLIB_ERR_INVALID_OUTPUT_POWER:
            return "ERR_INVALID_OUTPUT_POWER";
        case RADIOLIB_ERR_NETWORK_NOT_JOINED:
            return "RADIOLIB_ERR_NETWORK_NOT_JOINED";
        case RADIOLIB_ERR_DOWNLINK_MALFORMED:
            return "RADIOLIB_ERR_DOWNLINK_MALFORMED";
        case RADIOLIB_ERR_INVALID_REVISION:
            return "RADIOLIB_ERR_INVALID_REVISION";
        case RADIOLIB_ERR_INVALID_PORT:
            return "RADIOLIB_ERR_INVALID_PORT";
        case RADIOLIB_ERR_NO_RX_WINDOW:
            return "RADIOLIB_ERR_NO_RX_WINDOW";
        case RADIOLIB_ERR_INVALID_CID:
            return "RADIOLIB_ERR_INVALID_CID";
        case RADIOLIB_ERR_UPLINK_UNAVAILABLE:
            return "RADIOLIB_ERR_UPLINK_UNAVAILABLE";
        case RADIOLIB_ERR_COMMAND_QUEUE_FULL:
            return "RADIOLIB_ERR_COMMAND_QUEUE_FULL";
        case RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND:
            return "RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND";
        case RADIOLIB_ERR_JOIN_NONCE_INVALID:
            return "RADIOLIB_ERR_JOIN_NONCE_INVALID";
        case RADIOLIB_ERR_N_FCNT_DOWN_INVALID:
            return "RADIOLIB_ERR_N_FCNT_DOWN_INVALID";
        case RADIOLIB_ERR_A_FCNT_DOWN_INVALID:
            return "RADIOLIB_ERR_A_FCNT_DOWN_INVALID";
        case RADIOLIB_ERR_DWELL_TIME_EXCEEDED:
            return "RADIOLIB_ERR_DWELL_TIME_EXCEEDED";
        case RADIOLIB_ERR_CHECKSUM_MISMATCH:
            return "RADIOLIB_ERR_CHECKSUM_MISMATCH";
        case RADIOLIB_ERR_NO_JOIN_ACCEPT:
            return "RADIOLIB_ERR_NO_JOIN_ACCEPT";
        case RADIOLIB_LORAWAN_SESSION_RESTORED:
            return "RADIOLIB_LORAWAN_SESSION_RESTORED";
        case RADIOLIB_LORAWAN_NEW_SESSION:
            return "RADIOLIB_LORAWAN_NEW_SESSION";
        case RADIOLIB_ERR_NONCES_DISCARDED:
            return "RADIOLIB_ERR_NONCES_DISCARDED";
        case RADIOLIB_ERR_SESSION_DISCARDED:
            return "RADIOLIB_ERR_SESSION_DISCARDED";
        }
        return "NO_CODE_DOCUMENTED";
    }

    static void debug(bool failed, char *message, int state)
    {
        if (failed)
        {
            ESP_LOGW(TAG, "%s", message);
            ESP_LOGE(TAG, "%s", state_decode(state));
        }
    }

    esp_err_t setup_lorawan_node(void)
    {
        ESP_LOGW(TAG, "estableciendo la configuración para sesión OTAA…");

        int16_t lora_state = node.beginOTAA(join_EUI, dev_EUI, nwk_KEY, app_KEY);
        debug(lora_state != RADIOLIB_ERR_NONE,
              "configuración inicial de sesión OTAA fallida.", lora_state);

        if (lora_state != RADIOLIB_ERR_NONE)
            return ESP_FAIL;

        ESP_LOGI(TAG, "configuración de sesión OTAA establecida.");
        return ESP_OK;
    }

    esp_err_t join_lorawan_net(void)
    {
        ESP_LOGW(TAG, "uniéndose (join) al servidor de red lorawan…");
        int16_t lora_state = node.activateOTAA();

        debug(lora_state != RADIOLIB_LORAWAN_NEW_SESSION,
              "sesión (join) con servidor lorawan fallida.", lora_state);

        if (lora_state != RADIOLIB_LORAWAN_NEW_SESSION)
            return ESP_FAIL;

        ESP_LOGI(TAG, "nodo lorawan unido a la red con éxito.");
        ESP_LOGW(TAG, "el devAddr del nodo lorawan es: %lu.", (unsigned long)node.getDevAddr());

        node.setADR(false);
        ESP_LOGW(TAG, "datarate adapatativo no configurado.");

        node.setDatarate(1); // SF9 + DR1 configurado.
        ESP_LOGW(TAG, "Esquema DR[1] SF[9] establecido.");

        node.setDutyCycle(true, 0);
        ESP_LOGW(TAG, "ciclo de trabajo establecido según US915.");

        node.setDwellTime(true, 400);
        ESP_LOGW(TAG, "tiempo en el aire máximo establecido por banda.");

        node.setTxPower(21);
        ESP_LOGW(TAG, "potencia de transmisión máxima permisible.");

        return ESP_OK;
    }

    /**
     *Primer dato.env_co2->int16_t->2 bytes.
     *Segundo dato.env_temp->int8_t->1 byte.
     *Tercer dato.env_hum->uint8_t->1 byte.
     *Cuarto dato.env_uv->uint8_t->1 byte.
     *Quinto dato.env_dB->uint8_t->1 byte.
     *Sexto dato.lora_id->uint8_t->1 byte.
     *Total de bytes a transmitir en payload : 7 bytes.
     */
    void sensors_data_bytes_conversion(int8_t env_temp, uint8_t env_hum, uint8_t env_uv,
                                       uint8_t env_dB, int16_t env_co2, uint8_t *out_array)
    {
        // array de bytes de carga útil.
        uint8_t lora_macframe_payload[7] = {0};

        lora_macframe_payload[0] = highByte(env_co2);
        lora_macframe_payload[1] = lowByte(env_co2);
        lora_macframe_payload[2] = env_temp;
        lora_macframe_payload[3] = env_hum;
        lora_macframe_payload[4] = env_uv;
        lora_macframe_payload[5] = env_dB;
        lora_macframe_payload[6] = LORA_NODE_ID;

        for (int i = 0; i < 7; i++)
        {
            out_array[i] = lora_macframe_payload[i];
            ESP_LOGW(TAG, "byte [%i] del macframe payload: %u.", i, out_array[i]);
        }

        ESP_LOGW(TAG, "análisis de recuperación de los valores originales.");
        // Combinar los dos bytes para formar el valor original
        int16_t og_env_co2 = ((int16_t)out_array[0] << 8) | out_array[1];
        ESP_LOGI(TAG, "co2 original: %i ppm.", og_env_co2);
        ESP_LOGI(TAG, "temperatura original: %i °C.", out_array[2]);
        ESP_LOGI(TAG, "humedad original: %i %%.", out_array[3]);
        ESP_LOGI(TAG, "índice uv original: %i índ.", out_array[4]);
        ESP_LOGI(TAG, "ruido ambiental original: %i dB.", out_array[5]);
        ESP_LOGI(TAG, "id lora original: %i.", out_array[6]);
    }

    esp_err_t send_lora_uplink(uint8_t *bytes_data)
    {
        int lora_state = node.sendReceive(bytes_data, 7,
                                          LORA_FPORT, LORA_CONFIRMED_FRAMES);

        if (lora_state < RADIOLIB_ERR_NONE)
        {
            ESP_LOGE(TAG, "error de envío del mensaje uplink.");
            return ESP_FAIL;
        }
        else
            ESP_LOGI(TAG, "mensaje uplink enviado.");

        // Revisar por posibles downlinks.
        // (state 0 = no downlink, state 1/2 = downlink en la ventana Rx1/Rx2)
        if (lora_state > 0)
            ESP_LOGW(TAG, "se recibió un downlink.");
        else
            ESP_LOGW(TAG, "no se recibieron downlinks.");

        return ESP_OK;
    }
}