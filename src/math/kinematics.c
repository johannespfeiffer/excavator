#include "kinematics.h"

#include <math.h>

bucket_height_result_t kinematics_calculate_bucket_height(const excavator_geometry_t *geometry,
                                                          const orientation_estimate_t *orientation,
                                                          const gps_fix_t *gps_fix)
{
    const bool geometry_valid = excavator_geometry_is_configured(geometry);
    const bool orientation_valid = (orientation != 0);
    const bool gps_valid = (gps_fix != 0) && gps_fix->valid;
    const bool inputs_valid = geometry_valid && orientation_valid && gps_valid;
    const float gps_altitude_m = gps_valid ? gps_fix->altitude_m : 0.0f;
    const float delta_height_m =
        (geometry_valid && orientation_valid)
            ? (geometry->l1_m * sinf(orientation->s1_angle_rad)) +
                  (geometry->l2_m * sinf(orientation->s2_angle_rad)) +
                  (geometry->l3_m * sinf(orientation->s3_angle_rad)) +
                  ((geometry->l4_m + geometry->o1_m) * sinf(orientation->s4_angle_rad))
            : 0.0f;

    return (bucket_height_result_t){
        .inputs_valid = inputs_valid,
        /*
         * Current kinematic model assumes each absolute segment angle is measured
         * against the horizontal zero pose described in the README. The tip
         * offset O1 is treated as collinear with the bucket segment.
         */
        .delta_height_m = delta_height_m,
        .absolute_height_m = gps_altitude_m + delta_height_m,
    };
}
