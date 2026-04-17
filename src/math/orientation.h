#ifndef EXCAVATOR_ORIENTATION_H
#define EXCAVATOR_ORIENTATION_H

#include "bmi160.h"
#include "config.h"

#include <stdbool.h>

typedef struct {
    float s1_angle_rad;
    float s2_angle_rad;
    float s3_angle_rad;
    float s4_angle_rad;
} orientation_estimate_t;

orientation_estimate_t orientation_estimate_level(void);
bool orientation_quasistatic_sample_valid(const bmi160_sample_t *sample,
                                          const excavator_quasistatic_filter_t *filter);
float orientation_quasistatic_angle_from_accel(const bmi160_sample_t *sample, float angle_offset_rad);

#endif
