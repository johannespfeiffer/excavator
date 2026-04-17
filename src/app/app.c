#include "app.h"

#include "config.h"
#include "excavator_state.h"
#include "bmi160.h"
#include "gps_parser.h"
#include "orientation.h"
#include "platform.h"

int app_run(void)
{
    excavator_config_t config = excavator_config_default();
    platform_status_t platform_status = platform_init();
    excavator_state_t state;
    gps_fix_t gps_fix = {0};
    orientation_estimate_t orientation = orientation_estimate_level();

    excavator_state_init(&state, &config);
    excavator_state_set_platform_status(&state, platform_status);
    excavator_state_set_orientation(&state, &orientation);

    if (!platform_status_ok(platform_status)) {
        return 1;
    }

    gps_parser_reset();
    (void)gps_parser_poll_uart(&gps_fix);
    excavator_state_set_gps_fix(&state, &gps_fix);
    (void)excavator_state_update_result(&state);

    return 0;
}
