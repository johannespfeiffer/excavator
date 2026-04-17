#include "orientation.h"

#include <assert.h>
#include <math.h>

static int float_close(float a, float b, float tolerance)
{
    const float diff = (a > b) ? (a - b) : (b - a);
    return diff <= tolerance;
}

int main(void)
{
    const excavator_quasistatic_filter_t filter = {
        .min_magnitude_mps2 = 7.5f,
        .max_magnitude_mps2 = 12.0f,
    };
    const bmi160_sample_t level = {
        .accel_x_mps2 = 0.0f,
        .accel_y_mps2 = 0.0f,
        .accel_z_mps2 = 9.80665f,
    };
    const bmi160_sample_t plus_ninety = {
        .accel_x_mps2 = -9.80665f,
        .accel_y_mps2 = 0.0f,
        .accel_z_mps2 = 0.0f,
    };
    const bmi160_sample_t minus_forty_five = {
        .accel_x_mps2 = 6.934348f,
        .accel_y_mps2 = 0.0f,
        .accel_z_mps2 = 6.934348f,
    };
    const bmi160_sample_t invalid = {
        .accel_x_mps2 = 0.0f,
        .accel_y_mps2 = 0.0f,
        .accel_z_mps2 = 0.0f,
    };

    assert(orientation_quasistatic_sample_valid(&level, &filter));
    assert(float_close(orientation_quasistatic_angle_from_accel(&level, 0.0f), 0.0f, 0.0001f));
    assert(float_close(orientation_quasistatic_angle_from_accel(&plus_ninety, 0.0f), (float)M_PI_2, 0.0001f));
    assert(float_close(orientation_quasistatic_angle_from_accel(&minus_forty_five, 0.0f), -(float)M_PI / 4.0f, 0.0001f));
    assert(float_close(orientation_quasistatic_angle_from_accel(&level, 0.25f), 0.25f, 0.0001f));
    assert(!orientation_quasistatic_sample_valid(&invalid, &filter));
    assert(!orientation_quasistatic_sample_valid(0, &filter));
    assert(!orientation_quasistatic_sample_valid(&level, 0));

    return 0;
}
