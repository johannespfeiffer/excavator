#include "config.h"

#include <assert.h>

int main(void)
{
    const excavator_config_t default_config = excavator_config_default();
    const excavator_geometry_t configured_geometry = {
        .l1_m = 1.0f,
        .l2_m = 2.0f,
        .l3_m = 3.0f,
        .l4_m = 4.0f,
        .o1_m = 0.5f,
    };
    const excavator_geometry_t invalid_offset = {
        .l1_m = 1.0f,
        .l2_m = 2.0f,
        .l3_m = 3.0f,
        .l4_m = 4.0f,
        .o1_m = -0.1f,
    };

    assert(!excavator_geometry_is_configured(&default_config.geometry));
    assert(excavator_geometry_is_configured(&configured_geometry));
    assert(!excavator_geometry_is_configured(&invalid_offset));
    assert(!excavator_geometry_is_configured(0));

    return 0;
}
