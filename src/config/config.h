#ifndef EXCAVATOR_CONFIG_H
#define EXCAVATOR_CONFIG_H

#include <stdbool.h>

enum {
    EXCAVATOR_IMU_COUNT = 4,
};

typedef struct {
    float l1_m;
    float l2_m;
    float l3_m;
    float l4_m;
    float o1_m;
} excavator_geometry_t;

typedef struct {
    float angle_offset_rad;
} excavator_sensor_mount_t;

typedef struct {
    float min_magnitude_mps2;
    float max_magnitude_mps2;
} excavator_quasistatic_filter_t;

typedef struct {
    excavator_geometry_t geometry;
    excavator_sensor_mount_t sensor_mounts[EXCAVATOR_IMU_COUNT];
    excavator_quasistatic_filter_t quasistatic_filter;
} excavator_config_t;

excavator_config_t excavator_config_default(void);
bool excavator_geometry_is_configured(const excavator_geometry_t *geometry);

#endif
