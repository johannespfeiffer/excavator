#include "kinematics.h"

#include <assert.h>

int main(void)
{
    const excavator_geometry_t geometry = {
        .l1_m = 0.0f,
        .l2_m = 0.0f,
        .l3_m = 0.0f,
        .l4_m = 0.0f,
        .o1_m = 0.0f,
    };
    const orientation_estimate_t orientation = orientation_estimate_level();
    const gps_fix_t gps_fix = {
        .valid = true,
        .altitude_m = 123.45f,
    };

    const bucket_height_result_t result =
        kinematics_calculate_bucket_height(&geometry, &orientation, &gps_fix);

    assert(result.absolute_height_m == 123.45f);
    assert(result.delta_height_m == 0.0f);
    return 0;
}
