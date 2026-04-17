#include "app.h"

#include "config.h"
#include "excavator_state.h"
#include "bmi160.h"
#include "gps_parser.h"
#include "platform.h"
#include "telemetry.h"

#include <stdio.h>

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

#if defined(EXCAVATOR_TARGET_STM32F446RE)
    int status_code = 0;
    uint32_t loop_delay;

    /*
     * Bench target for the NUCLEO-F446RE: read S1 on I2C1 and emit repeated
     * plain-ASCII telemetry over USART2/ST-LINK VCP.
     */
    for (;;) {
        bmi160_status_t read_status = BMI160_STATUS_OK;

        status_code = 0;

        if (!platform_status_ok(state.platform_status)) {
            status_code = 1;
        } else if (bmi160_init(PLATFORM_I2C1) != BMI160_STATUS_OK) {
            status_code = 2;
        } else if ((read_status = bmi160_read_sample(PLATFORM_I2C1, &sample)) != BMI160_STATUS_OK) {
            status_code = 3;
        } else if (!excavator_state_set_imu_sample(&state, EXCAVATOR_SENSOR_S1, &sample)) {
            status_code = 4;
        }

        if (status_code == 0) {
            state.estimation.valid = true;
            state.estimation.orientation.s1_angle_rad =
                orientation_quasistatic_angle_from_accel(
                    &sample, config.sensor_mounts[EXCAVATOR_SENSOR_S1].angle_offset_rad);
        }

        output_length = (size_t)snprintf(output_line,
                                         sizeof(output_line),
                                         "bench=s1 status=%d platform_ok=%u i2c1=%u read=%u s1_mdeg=%ld ax=%ld ay=%ld az=%ld\r\n",
                                         status_code,
                                         platform_status_ok(state.platform_status) ? 1u : 0u,
                                         platform_i2c_ready(state.platform_status, PLATFORM_I2C1) ? 1u : 0u,
                                         (status_code == 0) ? 1u : 0u,
                                         (long)(state.estimation.orientation.s1_angle_rad * 1000.0f),
                                         (long)(sample.accel_x_mps2 * 1000.0f),
                                         (long)(sample.accel_y_mps2 * 1000.0f),
                                         (long)(sample.accel_z_mps2 * 1000.0f));
        if ((output_length > 0u) && (output_length < sizeof(output_line))) {
            (void)platform_uart_write(PLATFORM_UART_OUTPUT,
                                      (const uint8_t *)output_line,
                                      (uint16_t)output_length);
        }

        for (loop_delay = 0u; loop_delay < 1600000u; ++loop_delay) {
            __asm__ volatile ("nop");
        }
    }
#else
    gps_fix_t gps_fix = {0};

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

    (void)excavator_state_calibrate_zero_pose(&state);

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
