#ifndef EXCAVATOR_ORIENTATION_H
#define EXCAVATOR_ORIENTATION_H

typedef struct {
    float s1_angle_rad;
    float s2_angle_rad;
    float s3_angle_rad;
    float s4_angle_rad;
} orientation_estimate_t;

orientation_estimate_t orientation_estimate_level(void);

#endif
