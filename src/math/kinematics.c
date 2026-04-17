#include "kinematics.h"

bucket_height_result_t kinematics_calculate_bucket_height(const excavator_geometry_t *geometry,
                                                          const orientation_estimate_t *orientation,
                                                          const gps_fix_t *gps_fix)
{
    const bool geometry_valid = excavator_geometry_is_configured(geometry);
    const bool orientation_valid = (orientation != 0);
    const bool gps_valid = (gps_fix != 0) && gps_fix->valid;
    const bool inputs_valid = geometry_valid && orientation_valid && gps_valid;
    const float gps_altitude_m = gps_valid ? gps_fix->altitude_m : 0.0f;

    return (bucket_height_result_t){
        .inputs_valid = inputs_valid,
        .delta_height_m = 0.0f,
        .absolute_height_m = gps_altitude_m,
    };
}
