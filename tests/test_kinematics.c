#include "kinematics.h"

#include <assert.h>
#include <math.h>

static int float_close(float a, float b, float tolerance)
{
    const float diff = (a > b) ? (a - b) : (b - a);
    return diff <= tolerance;
}

int main(void)
{
    const excavator_geometry_t geometry = {
        .l1_m = 1.0f,
        .l2_m = 2.0f,
        .l3_m = 3.0f,
        .l4_m = 4.0f,
        .o1_m = 0.5f,
    };
    const orientation_estimate_t orientation = orientation_estimate_level();
    const gps_fix_t gps_fix = {
        .valid = true,
        .altitude_m = 123.45f,
    };

    const bucket_height_result_t result =
        kinematics_calculate_bucket_height(&geometry, &orientation, &gps_fix);

    assert(result.inputs_valid);
    assert(float_close(result.absolute_height_m, 123.45f, 0.0001f));
    assert(float_close(result.delta_height_m, 0.0f, 0.0001f));

    {
        const orientation_estimate_t raised = {
            .s1_angle_rad = (float)M_PI / 2.0f,
            .s2_angle_rad = 0.0f,
            .s3_angle_rad = 0.0f,
            .s4_angle_rad = 0.0f,
        };
        const bucket_height_result_t raised_result =
            kinematics_calculate_bucket_height(&geometry, &raised, &gps_fix);
        assert(raised_result.inputs_valid);
        assert(float_close(raised_result.delta_height_m, 1.0f, 0.0001f));
        assert(float_close(raised_result.absolute_height_m, 124.45f, 0.0001f));
    }

    {
        const orientation_estimate_t mixed = {
            .s1_angle_rad = 0.0f,
            .s2_angle_rad = (float)M_PI / 6.0f,
            .s3_angle_rad = -(float)M_PI / 2.0f,
            .s4_angle_rad = (float)M_PI / 2.0f,
        };
        const bucket_height_result_t mixed_result =
            kinematics_calculate_bucket_height(&geometry, &mixed, &gps_fix);
        const float expected_delta = (2.0f * 0.5f) + (-3.0f) + (4.5f);
        assert(mixed_result.inputs_valid);
        assert(float_close(mixed_result.delta_height_m, expected_delta, 0.0001f));
        assert(float_close(mixed_result.absolute_height_m, 123.45f + expected_delta, 0.0001f));
    }

    {
        const bucket_height_result_t missing_gps =
            kinematics_calculate_bucket_height(&geometry, &orientation, 0);
        assert(!missing_gps.inputs_valid);
        assert(float_close(missing_gps.absolute_height_m, 0.0f, 0.0001f));
        assert(float_close(missing_gps.delta_height_m, 0.0f, 0.0001f));
    }

    {
        const gps_fix_t invalid_gps = {
            .valid = false,
            .altitude_m = 999.0f,
        };
        const bucket_height_result_t result_invalid_gps =
            kinematics_calculate_bucket_height(&geometry, &orientation, &invalid_gps);
        assert(!result_invalid_gps.inputs_valid);
        assert(float_close(result_invalid_gps.absolute_height_m, 0.0f, 0.0001f));
    }

    {
        const excavator_geometry_t zero_geometry = {0};
        const bucket_height_result_t result_zero_geometry =
            kinematics_calculate_bucket_height(&zero_geometry, &orientation, &gps_fix);
        assert(!result_zero_geometry.inputs_valid);
        assert(float_close(result_zero_geometry.absolute_height_m, 123.45f, 0.0001f));
        assert(float_close(result_zero_geometry.delta_height_m, 0.0f, 0.0001f));
    }

    {
        const bucket_height_result_t missing_orientation =
            kinematics_calculate_bucket_height(&geometry, 0, &gps_fix);
        assert(!missing_orientation.inputs_valid);
        assert(float_close(missing_orientation.delta_height_m, 0.0f, 0.0001f));
    }

    return 0;
}
