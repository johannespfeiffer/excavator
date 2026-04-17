#include "orientation.h"

orientation_estimate_t orientation_estimate_level(void)
{
    return (orientation_estimate_t){
        .s1_angle_rad = 0.0f,
        .s2_angle_rad = 0.0f,
        .s3_angle_rad = 0.0f,
        .s4_angle_rad = 0.0f,
    };
}
