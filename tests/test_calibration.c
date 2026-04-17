#include "calibration.h"

#include <assert.h>
#include <math.h>

static bmi160_sample_t sample_from_angle(float angle_rad)
{
    return (bmi160_sample_t){
        .accel_x_mps2 = -9.80665f * sinf(angle_rad),
        .accel_y_mps2 = 0.0f,
        .accel_z_mps2 = 9.80665f * cosf(angle_rad),
    };
}

static int float_close(float a, float b, float tolerance)
{
    const float diff = (a > b) ? (a - b) : (b - a);
    return diff <= tolerance;
}

int main(void)
{
    uint8_t index;
    excavator_calibration_t calibration;
    excavator_config_t config = excavator_config_default();
    bmi160_sample_t zero_pose_samples[EXCAVATOR_IMU_COUNT];

    config.sensor_mounts[0].angle_offset_rad = 0.0f;
    config.sensor_mounts[1].angle_offset_rad = 0.1f;
    config.sensor_mounts[2].angle_offset_rad = -0.1f;
    config.sensor_mounts[3].angle_offset_rad = 0.2f;

    zero_pose_samples[0] = sample_from_angle(0.0f);
    zero_pose_samples[1] = sample_from_angle(0.2f);
    zero_pose_samples[2] = sample_from_angle(-0.3f);
    zero_pose_samples[3] = sample_from_angle(0.4f);

    excavator_calibration_reset(&calibration);
    assert(!calibration.valid);
    for (index = 0u; index < EXCAVATOR_IMU_COUNT; ++index) {
        assert(float_close(calibration.angle_trim_rad[index], 0.0f, 0.0001f));
    }

    assert(excavator_calibration_compute_zero_pose(&calibration, zero_pose_samples, &config));
    assert(calibration.valid);
    assert(float_close(excavator_calibration_total_offset_rad(&calibration, &config, 0u), 0.0f, 0.0001f));
    assert(float_close(excavator_calibration_total_offset_rad(&calibration, &config, 1u), -0.2f, 0.0001f));
    assert(float_close(excavator_calibration_total_offset_rad(&calibration, &config, 2u), 0.3f, 0.0001f));
    assert(float_close(excavator_calibration_total_offset_rad(&calibration, &config, 3u), -0.4f, 0.0001f));

    zero_pose_samples[0].accel_z_mps2 = 0.0f;
    zero_pose_samples[0].accel_x_mps2 = 0.0f;
    assert(!excavator_calibration_compute_zero_pose(&calibration, zero_pose_samples, &config));
    assert(!calibration.valid);

    return 0;
}
