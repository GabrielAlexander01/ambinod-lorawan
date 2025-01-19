#ifndef EV_SENSORS_H
#define EV_SENSORS_H

#include <esp_err.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Configurar las opciones gpio del sensor dht22.
     */
    esp_err_t init_dht22_sensor(void);

    /**
     * @brief Configurar las opciones adc del sensor guvas12sd.
     */
    esp_err_t init_guvas12sd_sensor(void);

    /**
     * @brief Configurar las opciones adc del sensor guvas12sd.
     */
    esp_err_t init_max4466_sensor(void);

    /**
     * @brief Configurar las opciones uart del sensor mhz19b.
     */
    esp_err_t init_mhz19b_sensor(void);

    /**
     * @brief Configurar la calibración del sensor mhz19b.
     */
    void calibrate_mhz19b_sensor(void);

    /**
     * @brief Normalizar las lecturas de los sensores.
     */
    void normalize_sensors_lectures(void);

    /**
     * @brief Tarea freertos para sensar las variables ambientales.
     */
    void env_sensing_routine(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // EV_SENSORS_H
