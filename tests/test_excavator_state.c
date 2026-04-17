#include "excavator_state.h"

#include <assert.h>
#include <math.h>

static excavator_config_t configured_config(void)
{
    excavator_config_t config = excavator_config_default();
    config.geometry = (excavator_geometry_t){
        .l1_m = 1.0f,
        .l2_m = 2.0f,
        .l3_m = 3.0f,
        .l4_m = 4.0f,
        .o1_m = 0.5f,
    };
    config.sensor_mounts[EXCAVATOR_SENSOR_S1].angle_offset_rad = 0.0f;
    config.sensor_mounts[EXCAVATOR_SENSOR_S2].angle_offset_rad = 0.1f;
    config.sensor_mounts[EXCAVATOR_SENSOR_S3].angle_offset_rad = -0.1f;
    config.sensor_mounts[EXCAVATOR_SENSOR_S4].angle_offset_rad = 0.0f;
    return config;
}

static int float_close(float a, float b, float tolerance)
{
    const float diff = (a > b) ? (a - b) : (b - a);
    return diff <= tolerance;
}

int main(void)
{
    uint8_t index;
    excavator_state_t state;
    const excavator_config_t config = configured_config();
    const bmi160_sample_t sample = {
        .accel_x_mps2 = 0.0f,
        .accel_y_mps2 = 0.0f,
        .accel_z_mps2 = 9.80665f,
        .gyro_x_radps = 0.1f,
        .gyro_y_radps = 0.2f,
        .gyro_z_radps = 0.3f,
    };
    const gps_fix_t gps_fix = {
        .valid = true,
        .altitude_m = 42.0f,
    };

    excavator_state_init(&state, &config);
    assert(!excavator_state_inputs_ready(&state));

    excavator_state_set_platform_status(&state, platform_init());
    excavator_state_set_gps_fix(&state, &gps_fix);

    for (index = 0u; index < EXCAVATOR_SENSOR_COUNT; ++index) {
        assert(excavator_state_set_imu_sample(&state, (excavator_sensor_id_t)index, &sample));
    }

    assert(excavator_state_estimate_orientation_quasistatic(&state));
    assert(excavator_state_inputs_ready(&state));
    assert(excavator_state_update_result(&state));
    assert(state.result.valid);
    assert(state.result.bucket_height.inputs_valid);
    assert(float_close(state.result.bucket_height.delta_height_m, -0.09983342f, 0.0001f));
    assert(float_close(state.result.bucket_height.absolute_height_m, 41.90016658f, 0.0001f));
    assert(float_close(state.estimation.orientation.s1_angle_rad, 0.0f, 0.0001f));
    assert(float_close(state.estimation.orientation.s2_angle_rad, 0.1f, 0.0001f));
    assert(float_close(state.estimation.orientation.s3_angle_rad, -0.1f, 0.0001f));
    assert(float_close(state.estimation.orientation.s4_angle_rad, 0.0f, 0.0001f));

    excavator_state_set_gps_fix(&state, &(gps_fix_t){ .valid = false, .altitude_m = 0.0f });
    assert(!excavator_state_update_result(&state));
    assert(!state.result.valid);

    excavator_state_set_gps_fix(&state, &gps_fix);
    excavator_state_set_orientation(&state, 0);
    assert(!excavator_state_update_result(&state));
    assert(!state.result.valid);

    assert(excavator_state_set_imu_sample(&state, EXCAVATOR_SENSOR_S1, &(bmi160_sample_t){
                                                                     .accel_x_mps2 = 0.0f,
                                                                     .accel_y_mps2 = 0.0f,
                                                                     .accel_z_mps2 = 0.0f,
                                                                 }));
    assert(!excavator_state_estimate_orientation_quasistatic(&state));

    assert(!excavator_state_set_imu_sample(&state, EXCAVATOR_SENSOR_COUNT, &sample));
    assert(!excavator_state_set_imu_sample(&state, EXCAVATOR_SENSOR_S1, 0));

    return 0;
}
