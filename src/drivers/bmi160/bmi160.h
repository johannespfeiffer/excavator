#ifndef EXCAVATOR_BMI160_H
#define EXCAVATOR_BMI160_H

#include "platform.h"

#include <stdint.h>

typedef enum {
    BMI160_STATUS_OK = 0,
    BMI160_STATUS_INVALID_ARGUMENT = 1,
    BMI160_STATUS_COMMUNICATION_ERROR = 2,
    BMI160_STATUS_CHIP_ID_MISMATCH = 3,
} bmi160_status_t;

typedef struct {
    float accel_x_mps2;
    float accel_y_mps2;
    float accel_z_mps2;
    float gyro_x_radps;
    float gyro_y_radps;
    float gyro_z_radps;
} bmi160_sample_t;

enum {
    BMI160_I2C_ADDRESS = 0x69u,
    BMI160_CHIP_ID_VALUE = 0xD1u,
    BMI160_REG_CHIP_ID = 0x00u,
    BMI160_REG_GYRO_DATA = 0x0Cu,
    BMI160_REG_ACCEL_DATA = 0x12u,
    BMI160_REG_CMD = 0x7Eu,
};

bmi160_status_t bmi160_init(platform_i2c_bus_t bus);
bmi160_status_t bmi160_read_sample(platform_i2c_bus_t bus, bmi160_sample_t *sample);

#endif
