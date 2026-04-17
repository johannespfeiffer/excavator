#include "app.h"

#include "config.h"
#include "excavator_state.h"
#include "bmi160.h"
#include "gps_parser.h"
#include "platform.h"

int app_run(void)
{
    static const platform_i2c_bus_t k_sensor_buses[EXCAVATOR_SENSOR_COUNT] = {
        PLATFORM_I2C1,
        PLATFORM_I2C2,
        PLATFORM_I2C3,
        PLATFORM_FMPI2C1,
    };

    uint8_t index;
    excavator_config_t config = excavator_config_default();
    platform_status_t platform_status = platform_init();
    excavator_state_t state;
    gps_fix_t gps_fix = {0};
    bmi160_sample_t sample = {0};

    excavator_state_init(&state, &config);
    excavator_state_set_platform_status(&state, platform_status);

    if (!platform_status_ok(platform_status)) {
        return 1;
    }

    for (index = 0u; index < EXCAVATOR_SENSOR_COUNT; ++index) {
        const platform_i2c_bus_t bus = k_sensor_buses[index];

        if (bmi160_init(bus) != BMI160_STATUS_OK) {
            return 2;
        }

        if (bmi160_read_sample(bus, &sample) != BMI160_STATUS_OK) {
            return 3;
        }

        if (!excavator_state_set_imu_sample(&state, (excavator_sensor_id_t)index, &sample)) {
            return 4;
        }
    }

    if (!excavator_state_estimate_orientation_quasistatic(&state)) {
        return 5;
    }

    gps_parser_reset();
    if (gps_parser_poll_uart(&gps_fix)) {
        excavator_state_set_gps_fix(&state, &gps_fix);
    }

    (void)excavator_state_update_result(&state);

    return 0;
}
