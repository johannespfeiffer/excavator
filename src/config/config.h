#ifndef EXCAVATOR_CONFIG_H
#define EXCAVATOR_CONFIG_H

#include <stdbool.h>

typedef struct {
    float l1_m;
    float l2_m;
    float l3_m;
    float l4_m;
    float o1_m;
} excavator_geometry_t;

typedef struct {
    excavator_geometry_t geometry;
} excavator_config_t;

excavator_config_t excavator_config_default(void);
bool excavator_geometry_is_configured(const excavator_geometry_t *geometry);

#endif
