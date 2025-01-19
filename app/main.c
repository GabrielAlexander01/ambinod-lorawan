/**
 * @file main.c
 * Archivo fuente donde se inicaliza el programa, juntos a sus funciones
 * de lectura de sensores y envío de datos.
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ev_sensors.h"
#include "ev_lora.h"
// #include "max4466.h" // descomentar para test sensor ruido.

extern bool is_lora_join;

/**
 * @brief Punto de entrada a la aplicación esp32 env-sensor-firmware.
 * @return Estado de la ejecución del firmware. Si retorna, el programa se
 * detuvo o se interrumpió.
 */
void app_main(void)
{
    /* 1. Inicializar los sensores (aprox. 3mm) */
    ESP_ERROR_CHECK(init_dht22_sensor());
    ESP_ERROR_CHECK(init_guvas12sd_sensor());
    ESP_ERROR_CHECK(init_max4466_sensor());
    ESP_ERROR_CHECK(init_mhz19b_sensor());

    /* 2. Normalizar las muestras de sensores (aprox. 1mm) */
    normalize_sensors_lectures();

    // auxiliares de calibración
    // calibrate_mhz19b_sensor(); // descomentar para calibrar sensor co2.
    map_max4466_samples(); // descomentar para test sensor ruido.

    /* 3. Iniciar el radio LoRa y establecer conexión con el servidor TTN LoRaWAN (aprox. 1mm) */
    ESP_ERROR_CHECK(init_lora_phy());
    ESP_ERROR_CHECK(setup_lorawan_node());
    is_lora_join = (join_lorawan_net() == ESP_OK) ? true : false;

    /* 4. Rutina de medición y envío de datos (aprox. 1mm)*/
    xTaskCreate(env_sensing_routine, "medición de variables", 3000 * 11, NULL, 5, NULL);
}