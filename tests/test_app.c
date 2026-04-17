#include "bmi160.h"
#include "config.h"
#include "excavator_state.h"
#include "gps_parser.h"
#include "platform.h"
#include "telemetry.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

/*
 * app_run() now loops forever, so the test exercises the pipeline directly:
 * platform init → sensor init + calibration → one measurement tick → telemetry output.
 */
int main(void)
{
    static const platform_i2c_bus_t k_buses[EXCAVATOR_SENSOR_COUNT] = {
        PLATFORM_I2C1,
        PLATFORM_I2C2,
        PLATFORM_I2C3,
        PLATFORM_FMPI2C1,
    };

    excavator_config_t config = excavator_config_default();
    platform_status_t platform_status = platform_init();
    excavator_state_t state;
    bmi160_sample_t sample = {0};
    char output_line[256];
    size_t output_length;
    uint8_t tx_buffer[256];
    uint16_t tx_length = 0u;
    uint8_t index;

    excavator_state_init(&state, &config);
    excavator_state_set_platform_status(&state, platform_status);
    assert(platform_status_ok(platform_status));

    /* Init phase: bring up all sensors and capture initial samples for calibration. */
    for (index = 0u; index < EXCAVATOR_SENSOR_COUNT; ++index) {
        assert(bmi160_init(k_buses[index]) == BMI160_STATUS_OK);
        assert(bmi160_read_sample(k_buses[index], &sample) == BMI160_STATUS_OK);
        assert(excavator_state_set_imu_sample(&state, (excavator_sensor_id_t)index, &sample));
    }

    assert(excavator_state_calibrate_zero_pose(&state));
    gps_parser_reset();

    /* One measurement tick (mirrors the body of the main loop in app_run). */
    for (index = 0u; index < EXCAVATOR_SENSOR_COUNT; ++index) {
        assert(bmi160_read_sample(k_buses[index], &sample) == BMI160_STATUS_OK);
        assert(excavator_state_set_imu_sample(&state, (excavator_sensor_id_t)index, &sample));
    }

    assert(excavator_state_estimate_orientation_quasistatic(&state));
    (void)excavator_state_update_result(&state);

    output_length = telemetry_format_status_line(&state, output_line, sizeof(output_line));
    assert(output_length > 0u);
    assert(platform_uart_write(PLATFORM_UART_OUTPUT, (const uint8_t *)output_line, (uint16_t)output_length) ==
           PLATFORM_IO_STATUS_OK);

    assert(platform_simulated_uart_copy_tx(PLATFORM_UART_OUTPUT, tx_buffer, sizeof(tx_buffer), &tx_length) ==
           PLATFORM_IO_STATUS_OK);
    assert(tx_length > 0u);
    tx_buffer[tx_length] = '\0';
    assert(strstr((const char *)tx_buffer, "gps_valid=0") != NULL);
    assert(strstr((const char *)tx_buffer, "delta_m=0.000") != NULL);
    assert(strstr((const char *)tx_buffer, "platform_ok=1") != NULL);

    return 0;
}
