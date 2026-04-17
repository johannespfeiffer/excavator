#include "bmi160.h"

bmi160_status_t bmi160_init(uint8_t bus_index)
{
    (void)bus_index;
    return BMI160_STATUS_OK;
}

bmi160_status_t bmi160_read_sample(uint8_t bus_index, bmi160_sample_t *sample)
{
    (void)bus_index;

    if (sample == 0) {
        return BMI160_STATUS_ERROR;
    }

    *sample = (bmi160_sample_t){0};
    return BMI160_STATUS_OK;
}
