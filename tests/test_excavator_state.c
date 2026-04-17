#include "excavator_state.h"

#include <assert.h>

static excavator_config_t configured_config(void)
{
    return (excavator_config_t){
        .geometry = {
            .l1_m = 1.0f,
            .l2_m = 2.0f,
            .l3_m = 3.0f,
            .l4_m = 4.0f,
            .o1_m = 0.5f,
        },
    };
}

int main(void)
{
    uint8_t index;
    excavator_state_t state;
    const excavator_config_t config = configured_config();
    const bmi160_sample_t sample = {
        .accel_x_mps2 = 1.0f,
        .accel_y_mps2 = 2.0f,
        .accel_z_mps2 = 3.0f,
        .gyro_x_radps = 0.1f,
        .gyro_y_radps = 0.2f,
        .gyro_z_radps = 0.3f,
    };
    const gps_fix_t gps_fix = {
        .valid = true,
        .altitude_m = 42.0f,
    };
    const orientation_estimate_t orientation = {
        .s1_angle_rad = 0.1f,
        .s2_angle_rad = 0.2f,
        .s3_angle_rad = 0.3f,
        .s4_angle_rad = 0.4f,
    };

    excavator_state_init(&state, &config);
    assert(!excavator_state_inputs_ready(&state));

    excavator_state_set_platform_status(&state, platform_init());
    excavator_state_set_gps_fix(&state, &gps_fix);
    excavator_state_set_orientation(&state, &orientation);

    for (index = 0u; index < EXCAVATOR_SENSOR_COUNT; ++index) {
        assert(excavator_state_set_imu_sample(&state, (excavator_sensor_id_t)index, &sample));
    }

    assert(excavator_state_inputs_ready(&state));
    assert(excavator_state_update_result(&state));
    assert(state.result.valid);
    assert(state.result.bucket_height.inputs_valid);
    assert(state.result.bucket_height.absolute_height_m == 42.0f);

    excavator_state_set_gps_fix(&state, &(gps_fix_t){ .valid = false, .altitude_m = 0.0f });
    assert(!excavator_state_update_result(&state));
    assert(!state.result.valid);

    excavator_state_set_gps_fix(&state, &gps_fix);
    excavator_state_set_orientation(&state, 0);
    assert(!excavator_state_update_result(&state));
    assert(!state.result.valid);

    assert(!excavator_state_set_imu_sample(&state, EXCAVATOR_SENSOR_COUNT, &sample));
    assert(!excavator_state_set_imu_sample(&state, EXCAVATOR_SENSOR_S1, 0));

    return 0;
}
