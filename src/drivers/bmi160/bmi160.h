#ifndef EXCAVATOR_BMI160_H
#define EXCAVATOR_BMI160_H

#include <stdint.h>

typedef enum {
    BMI160_STATUS_OK = 0,
    BMI160_STATUS_ERROR = 1,
} bmi160_status_t;

typedef struct {
    float accel_x_mps2;
    float accel_y_mps2;
    float accel_z_mps2;
    float gyro_x_radps;
    float gyro_y_radps;
    float gyro_z_radps;
} bmi160_sample_t;

bmi160_status_t bmi160_init(uint8_t bus_index);
bmi160_status_t bmi160_read_sample(uint8_t bus_index, bmi160_sample_t *sample);

#endif
