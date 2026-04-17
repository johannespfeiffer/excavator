#include "calibration.h"

#include "orientation.h"

void excavator_calibration_reset(excavator_calibration_t *calibration)
{
    uint8_t index;

    if (calibration == 0) {
        return;
    }

    calibration->valid = false;
    for (index = 0u; index < EXCAVATOR_IMU_COUNT; ++index) {
        calibration->angle_trim_rad[index] = 0.0f;
    }
}

bool excavator_calibration_compute_zero_pose(excavator_calibration_t *calibration,
                                             const bmi160_sample_t *samples,
                                             const excavator_config_t *config)
{
    uint8_t index;

    if ((calibration == 0) || (samples == 0) || (config == 0)) {
        return false;
    }

    for (index = 0u; index < EXCAVATOR_IMU_COUNT; ++index) {
        const bmi160_sample_t *sample = &samples[index];

        if (!orientation_quasistatic_sample_valid(sample, &config->quasistatic_filter)) {
            excavator_calibration_reset(calibration);
            return false;
        }

        /*
         * In the zero pose every segment angle should evaluate to 0 rad.
         * The trim therefore cancels the current measured angle together with
         * the static mount offset from the configuration.
         */
        calibration->angle_trim_rad[index] =
            -orientation_quasistatic_angle_from_accel(sample, config->sensor_mounts[index].angle_offset_rad);
    }

    calibration->valid = true;
    return true;
}

float excavator_calibration_total_offset_rad(const excavator_calibration_t *calibration,
                                             const excavator_config_t *config,
                                             uint8_t sensor_index)
{
    const float mount_offset_rad =
        (config != 0 && sensor_index < EXCAVATOR_IMU_COUNT) ? config->sensor_mounts[sensor_index].angle_offset_rad : 0.0f;
    const float trim_offset_rad = (calibration != 0 && calibration->valid && sensor_index < EXCAVATOR_IMU_COUNT)
                                      ? calibration->angle_trim_rad[sensor_index]
                                      : 0.0f;

    return mount_offset_rad + trim_offset_rad;
}
