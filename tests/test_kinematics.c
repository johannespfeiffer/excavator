#include "kinematics.h"

#include <assert.h>

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
    assert(result.absolute_height_m == 123.45f);
    assert(result.delta_height_m == 0.0f);

    {
        const bucket_height_result_t missing_gps =
            kinematics_calculate_bucket_height(&geometry, &orientation, 0);
        assert(!missing_gps.inputs_valid);
        assert(missing_gps.absolute_height_m == 0.0f);
    }

    {
        const gps_fix_t invalid_gps = {
            .valid = false,
            .altitude_m = 999.0f,
        };
        const bucket_height_result_t result_invalid_gps =
            kinematics_calculate_bucket_height(&geometry, &orientation, &invalid_gps);
        assert(!result_invalid_gps.inputs_valid);
        assert(result_invalid_gps.absolute_height_m == 0.0f);
    }

    {
        const excavator_geometry_t zero_geometry = {0};
        const bucket_height_result_t result_zero_geometry =
            kinematics_calculate_bucket_height(&zero_geometry, &orientation, &gps_fix);
        assert(!result_zero_geometry.inputs_valid);
        assert(result_zero_geometry.absolute_height_m == 123.45f);
    }

    {
        const bucket_height_result_t missing_orientation =
            kinematics_calculate_bucket_height(&geometry, 0, &gps_fix);
        assert(!missing_orientation.inputs_valid);
    }

    return 0;
}
