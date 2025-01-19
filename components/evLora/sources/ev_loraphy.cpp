
/**
 * @file ev_loraphy.cpp
 * Archivo fuente que hace uso de la librería radioLib para:
 * 1. Crear un canal SPI para comunicación con el chip LoRa SX1262.
 * 2. Crear un driver radio para el chip LoRa SX1262.
 * 3. Establecer configuraciones LoRa para la transmisión en la capa física.
 * 4. Enlazar el driver LoRa Phy y sus primitivas de comunicación a la capa LoRaWan.
 */

#define TAG "envnode⇝loraphy"

#include <esp_err.h>
#include <esp_log.h>
#include <RadioLib.h>
#include <Arduino.h>
#include <SPI.h>

/**
 * @section Configuraciones definidas de la capa física LoRa.
 * Los ajustes se basan en la definición de la capa física de
 * los parámetros regionales lora y el plan de frecuencias US915.
 */

/** pinout heltec lora 32, datasheet V3.1. */
#define RADIO_NSS 8
#define RADIO_IRQ 14
#define RADIO_RST 12
#define RADIO_BUSY 13
#define RADIO_MOSI 10
#define RADIO_MISO 11
#define RADIO_SCK 9

#define LORAPHY_FREQ 922.3
#define LORAPHY_BW 125.0
#define LORAPHY_SF 7
#define LORAPHY_DBM 21
#define LORAPHY_SYNCWORD 0x34
#define LORAPHY_PREAMBLE 8

SPIClass *hspi = new SPIClass(HSPI);
SX1262 radio = new Module(RADIO_NSS, RADIO_IRQ, RADIO_RST, RADIO_BUSY, *hspi);

extern "C"
{
    esp_err_t init_lora_phy(void)
    {
        ESP_LOGW(TAG, "preparando chip lora sx1262…");

        hspi->begin(RADIO_SCK, RADIO_MISO, RADIO_MOSI, RADIO_NSS);
        int state = radio.begin();

        if (state != RADIOLIB_ERR_NONE)
        {
            ESP_LOGE(TAG, "error al iniciar el chip, código: %d\n", state);
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "chip lora sx1262 inicializado.");

        return ESP_OK;
    }

    /**
     * @brief Establece parámetros de la capa física LoRa, regulados
     * siguiendo los parámetros regionales adoptados nacionalmente, AU915.
     */
    esp_err_t set_lora_parameters(void)
    {
        ESP_LOGW(TAG, "estableciendo frecuencia sub-banda [2], 902.3 Mhz + 200KHz.");
        // radio.setFrequency(LORAPHY_FREQ);

        ESP_LOGW(TAG, "estableciendo ancho de banda de 125 KHz.");
        // radio.setBandwidth(LORAPHY_BW);

        ESP_LOGW(TAG, "estableciendo SF7 [DR3].");
        // radio.setSpreadingFactor(LORAPHY_SF);

        ESP_LOGW(TAG, "estableciendo potencia de emisión 21 dBm.");
        radio.setOutputPower(LORAPHY_DBM);

        ESP_LOGW(TAG, "estableciendo  palabra de sincronísmo 0x34.");
        radio.setSyncWord(LORAPHY_SYNCWORD);

        ESP_LOGW(TAG, "estableciendo preámbulo de 8 símbolos.");
        radio.setPreambleLength(LORAPHY_PREAMBLE);

        ESP_LOGW(TAG, "estableciendo tcxo para compensar frecuencia.");
        radio.setTCXO(2.4);

        return ESP_OK;
    }
}
