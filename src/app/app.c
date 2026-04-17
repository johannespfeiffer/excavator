#include "app.h"

#include "config.h"
#include "excavator_state.h"
#include "bmi160.h"
#include "gps_parser.h"
#include "platform.h"
#include "telemetry.h"

int app_run(void)
{
#if !defined(EXCAVATOR_TARGET_STM32F446RE)
    /* Sensor wiring on STM32F446RE: S1->I2C1 (PB8/PB9). */
    static const platform_i2c_bus_t k_sensor_buses[EXCAVATOR_SENSOR_COUNT] = {
        PLATFORM_I2C1,
        PLATFORM_I2C2,
        PLATFORM_I2C3,
        PLATFORM_FMPI2C1,
    };

    uint8_t index;
#endif
    excavator_config_t config = excavator_config_default();
    platform_status_t platform_status = platform_init();
    excavator_state_t state;
    bmi160_sample_t sample = {0};
    char output_line[256];
    size_t output_length = 0u;

    excavator_state_init(&state, &config);
    excavator_state_set_platform_status(&state, platform_status);

    if (!platform_status_ok(platform_status)) {
        return 1;
    }

#if defined(EXCAVATOR_TARGET_STM32F446RE)
    /*
     * Current hardware bench target: bring up the real S1 BMI160 on I2C1 and
     * verify one coherent sample can be read. The remaining sensors and GPS
     * path are still host-side/simulated work.
     */
    if (bmi160_init(PLATFORM_I2C1) != BMI160_STATUS_OK) {
        return 2;
    }

    if (bmi160_read_sample(PLATFORM_I2C1, &sample) != BMI160_STATUS_OK) {
        return 3;
    }

    return excavator_state_set_imu_sample(&state, EXCAVATOR_SENSOR_S1, &sample) ? 0 : 4;
#else
    gps_fix_t gps_fix = {0};

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
    output_length = telemetry_format_status_line(&state, output_line, sizeof(output_line));
    if (output_length > 0u) {
        (void)platform_uart_write(PLATFORM_UART_OUTPUT, (const uint8_t *)output_line, (uint16_t)output_length);
    }

    return 0;
#endif
}
