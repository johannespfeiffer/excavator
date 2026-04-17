#include "app.h"

#include "config.h"
#include "bmi160.h"
#include "gps_parser.h"
#include "kinematics.h"
#include "orientation.h"
#include "platform.h"

int app_run(void)
{
    excavator_config_t config = excavator_config_default();
    platform_status_t platform_status = platform_init();
    bmi160_sample_t imu_sample = {0};
    gps_fix_t gps_fix = {0};
    orientation_estimate_t orientation = orientation_estimate_level();

    (void)imu_sample;

    if (!platform_status_ok(platform_status)) {
        return 1;
    }

    gps_parser_reset();
    (void)gps_parser_poll_uart(&gps_fix);

    {
        bucket_height_result_t height =
            kinematics_calculate_bucket_height(&config.geometry, &orientation, &gps_fix);
        (void)height;
    }

    return 0;
}
