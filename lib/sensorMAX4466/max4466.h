
#ifndef MAX9814_H
#define MAX9814_H

#include <esp_err.h>

#ifdef __cplusplus__
extern "C"
{
#endif

    esp_err_t max4466_init(void);

    void map_max4466_samples(void);

    esp_err_t max4466_samples_read(float *noise);

    esp_err_t max4466_average_read(uint8_t *db_noise);

    void max4466_process_samples(uint8_t *arr, int size, uint8_t *avg_noise);

#ifdef __cplusplus__
}
#endif

#endif // MAX9814_H