#include "bmi160.h"
#include "platform.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

static void write_le16(uint8_t *buffer, int16_t value)
{
    buffer[0] = (uint8_t)(value & 0xFF);
    buffer[1] = (uint8_t)((uint16_t)value >> 8);
}

static int float_close(float a, float b, float tolerance)
{
    return fabsf(a - b) <= tolerance;
}

int main(void)
{
    uint8_t raw_data[12];
    uint8_t config_value = 0u;
    bmi160_sample_t sample = {0};
    const platform_status_t status = platform_init();

    assert(platform_status_ok(status));
    assert(bmi160_init(PLATFORM_I2C1) == BMI160_STATUS_OK);

    assert(platform_simulated_i2c_get_registers(PLATFORM_I2C1,
                                                BMI160_I2C_ADDRESS,
                                                0x40u,
                                                &config_value,
                                                1u));
    assert(config_value == 0x28u);

    write_le16(&raw_data[0], 1000);
    write_le16(&raw_data[2], -1000);
    write_le16(&raw_data[4], 500);
    write_le16(&raw_data[6], 16384);
    write_le16(&raw_data[8], -16384);
    write_le16(&raw_data[10], 8192);

    assert(platform_simulated_i2c_set_registers(PLATFORM_I2C1,
                                                BMI160_I2C_ADDRESS,
                                                BMI160_REG_GYRO_DATA,
                                                raw_data,
                                                sizeof(raw_data)));
    assert(bmi160_read_sample(PLATFORM_I2C1, &sample) == BMI160_STATUS_OK);
    assert(float_close(sample.accel_x_mps2, 9.80665f, 0.001f));
    assert(float_close(sample.accel_y_mps2, -9.80665f, 0.001f));
    assert(float_close(sample.accel_z_mps2, 4.903325f, 0.001f));
    assert(float_close(sample.gyro_x_radps, 1.0652645f, 0.001f));
    assert(float_close(sample.gyro_y_radps, -1.0652645f, 0.001f));
    assert(float_close(sample.gyro_z_radps, 0.53263223f, 0.001f));

    {
        const uint8_t wrong_chip_id = 0x00u;
        (void)platform_init();
        assert(platform_simulated_i2c_set_registers(PLATFORM_I2C2,
                                                    BMI160_I2C_ADDRESS,
                                                    BMI160_REG_CHIP_ID,
                                                    &wrong_chip_id,
                                                    1u));
        assert(bmi160_init(PLATFORM_I2C2) == BMI160_STATUS_CHIP_ID_MISMATCH);
    }

    assert(bmi160_init(PLATFORM_I2C_COUNT) == BMI160_STATUS_INVALID_ARGUMENT);
    assert(bmi160_read_sample(PLATFORM_I2C1, 0) == BMI160_STATUS_INVALID_ARGUMENT);

    return 0;
}
