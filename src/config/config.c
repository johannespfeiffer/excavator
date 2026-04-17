#include "config.h"

excavator_config_t excavator_config_default(void)
{
    const excavator_config_t config = {
        .geometry = {
            .l1_m = 0.0f,
            .l2_m = 0.0f,
            .l3_m = 0.0f,
            .l4_m = 0.0f,
            .o1_m = 0.0f,
        },
    };

    return config;
}

bool excavator_geometry_is_configured(const excavator_geometry_t *geometry)
{
    if (geometry == 0) {
        return false;
    }

    return (geometry->l1_m > 0.0f) &&
           (geometry->l2_m > 0.0f) &&
           (geometry->l3_m > 0.0f) &&
           (geometry->l4_m > 0.0f) &&
           (geometry->o1_m >= 0.0f);
}
