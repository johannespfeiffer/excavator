#ifndef EXCAVATOR_KINEMATICS_H
#define EXCAVATOR_KINEMATICS_H

#include "config.h"
#include "gps_parser.h"
#include "orientation.h"

typedef struct {
    bool inputs_valid;
    float delta_height_m;
    float absolute_height_m;
} bucket_height_result_t;

bucket_height_result_t kinematics_calculate_bucket_height(const excavator_geometry_t *geometry,
                                                          const orientation_estimate_t *orientation,
                                                          const gps_fix_t *gps_fix);

#endif
