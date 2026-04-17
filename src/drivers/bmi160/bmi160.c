#include "bmi160.h"

#include <stddef.h>

#define BMI160_REG_ACC_CONF 0x40u
#define BMI160_REG_ACC_RANGE 0x41u
#define BMI160_REG_GYR_CONF 0x42u
#define BMI160_REG_GYR_RANGE 0x43u

#define BMI160_CMD_ACC_NORMAL_MODE 0x11u
#define BMI160_CMD_GYR_NORMAL_MODE 0x15u

#define BMI160_ACC_CONF_VALUE 0x28u
#define BMI160_ACC_RANGE_2G 0x03u
#define BMI160_GYR_CONF_VALUE 0x28u
#define BMI160_GYR_RANGE_2000_DPS 0x00u

#define BMI160_ACC_SCALE_MPS2 (2.0f * 9.80665f / 32768.0f)
#define BMI160_GYR_SCALE_RADPS (2000.0f * 0.01745329251994329577f / 32768.0f)

static bmi160_status_t bmi160_read_register(platform_i2c_bus_t bus, uint8_t reg, uint8_t *value)
{
    if (value == NULL) {
        return BMI160_STATUS_INVALID_ARGUMENT;
    }

    if (platform_i2c_read_registers(bus, BMI160_I2C_ADDRESS, reg, value, 1u) != PLATFORM_IO_STATUS_OK) {
        return BMI160_STATUS_COMMUNICATION_ERROR;
    }

    return BMI160_STATUS_OK;
}

static bmi160_status_t bmi160_write_register(platform_i2c_bus_t bus, uint8_t reg, uint8_t value)
{
    if (platform_i2c_write_registers(bus, BMI160_I2C_ADDRESS, reg, &value, 1u) != PLATFORM_IO_STATUS_OK) {
        return BMI160_STATUS_COMMUNICATION_ERROR;
    }

    return BMI160_STATUS_OK;
}

static int16_t bmi160_read_le16(const uint8_t *buffer)
{
    return (int16_t)((uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8));
}

bmi160_status_t bmi160_init(platform_i2c_bus_t bus)
{
    uint8_t chip_id = 0u;

    if (bus >= PLATFORM_I2C_COUNT) {
        return BMI160_STATUS_INVALID_ARGUMENT;
    }

    if (bmi160_read_register(bus, BMI160_REG_CHIP_ID, &chip_id) != BMI160_STATUS_OK) {
        return BMI160_STATUS_COMMUNICATION_ERROR;
    }

    if (chip_id != BMI160_CHIP_ID_VALUE) {
        return BMI160_STATUS_CHIP_ID_MISMATCH;
    }

    if (bmi160_write_register(bus, BMI160_REG_CMD, BMI160_CMD_ACC_NORMAL_MODE) != BMI160_STATUS_OK) {
        return BMI160_STATUS_COMMUNICATION_ERROR;
    }

    if (bmi160_write_register(bus, BMI160_REG_CMD, BMI160_CMD_GYR_NORMAL_MODE) != BMI160_STATUS_OK) {
        return BMI160_STATUS_COMMUNICATION_ERROR;
    }

    if (bmi160_write_register(bus, BMI160_REG_ACC_CONF, BMI160_ACC_CONF_VALUE) != BMI160_STATUS_OK) {
        return BMI160_STATUS_COMMUNICATION_ERROR;
    }

    if (bmi160_write_register(bus, BMI160_REG_ACC_RANGE, BMI160_ACC_RANGE_2G) != BMI160_STATUS_OK) {
        return BMI160_STATUS_COMMUNICATION_ERROR;
    }

    if (bmi160_write_register(bus, BMI160_REG_GYR_CONF, BMI160_GYR_CONF_VALUE) != BMI160_STATUS_OK) {
        return BMI160_STATUS_COMMUNICATION_ERROR;
    }

    if (bmi160_write_register(bus, BMI160_REG_GYR_RANGE, BMI160_GYR_RANGE_2000_DPS) != BMI160_STATUS_OK) {
        return BMI160_STATUS_COMMUNICATION_ERROR;
    }

    return BMI160_STATUS_OK;
}

bmi160_status_t bmi160_read_sample(platform_i2c_bus_t bus, bmi160_sample_t *sample)
{
    uint8_t data[12];
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;

    if ((bus >= PLATFORM_I2C_COUNT) || (sample == NULL)) {
        return BMI160_STATUS_INVALID_ARGUMENT;
    }

    /*
     * BMI160 exposes gyro and accel samples in one contiguous register block,
     * so a single burst read keeps the sample coherent.
     */
    if (platform_i2c_read_registers(bus, BMI160_I2C_ADDRESS, BMI160_REG_GYRO_DATA, data, sizeof(data)) !=
        PLATFORM_IO_STATUS_OK) {
        return BMI160_STATUS_COMMUNICATION_ERROR;
    }

    gyro_x_raw = bmi160_read_le16(&data[0]);
    gyro_y_raw = bmi160_read_le16(&data[2]);
    gyro_z_raw = bmi160_read_le16(&data[4]);
    accel_x_raw = bmi160_read_le16(&data[6]);
    accel_y_raw = bmi160_read_le16(&data[8]);
    accel_z_raw = bmi160_read_le16(&data[10]);

    *sample = (bmi160_sample_t){
        .accel_x_mps2 = (float)accel_x_raw * BMI160_ACC_SCALE_MPS2,
        .accel_y_mps2 = (float)accel_y_raw * BMI160_ACC_SCALE_MPS2,
        .accel_z_mps2 = (float)accel_z_raw * BMI160_ACC_SCALE_MPS2,
        .gyro_x_radps = (float)gyro_x_raw * BMI160_GYR_SCALE_RADPS,
        .gyro_y_radps = (float)gyro_y_raw * BMI160_GYR_SCALE_RADPS,
        .gyro_z_radps = (float)gyro_z_raw * BMI160_GYR_SCALE_RADPS,
    };

    return BMI160_STATUS_OK;
}
