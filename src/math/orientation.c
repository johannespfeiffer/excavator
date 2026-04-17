#include "orientation.h"

#include <math.h>

orientation_estimate_t orientation_estimate_level(void)
{
    return (orientation_estimate_t){
        .s1_angle_rad = 0.0f,
        .s2_angle_rad = 0.0f,
        .s3_angle_rad = 0.0f,
        .s4_angle_rad = 0.0f,
    };
}

bool orientation_quasistatic_sample_valid(const bmi160_sample_t *sample,
                                          const excavator_quasistatic_filter_t *filter)
{
    const float magnitude_mps2 =
        (sample != 0) ? sqrtf((sample->accel_x_mps2 * sample->accel_x_mps2) +
                              (sample->accel_y_mps2 * sample->accel_y_mps2) +
                              (sample->accel_z_mps2 * sample->accel_z_mps2))
                      : 0.0f;

    if ((sample == 0) || (filter == 0)) {
        return false;
    }

    return (magnitude_mps2 >= filter->min_magnitude_mps2) &&
           (magnitude_mps2 <= filter->max_magnitude_mps2);
}

float orientation_quasistatic_angle_from_accel(const bmi160_sample_t *sample, float angle_offset_rad)
{
    if (sample == 0) {
        return angle_offset_rad;
    }

    /*
     * Current mounting assumption for the quasistatic fallback:
     * X axis runs along the lever, Z axis is normal to the lever plane.
     * At the zero pose the lever is horizontal and gravity projects to +Z.
     */
    return atan2f(-sample->accel_x_mps2, sample->accel_z_mps2) + angle_offset_rad;
}
