#include "telemetry.h"

#include <assert.h>
#include <string.h>

static excavator_state_t sample_state(void)
{
    excavator_state_t state;
    excavator_state_init(&state, &(excavator_config_t){
                                     .geometry = {
                                         .l1_m = 1.0f,
                                         .l2_m = 2.0f,
                                         .l3_m = 3.0f,
                                         .l4_m = 4.0f,
                                         .o1_m = 0.5f,
                                     },
                                 });
    state.platform_status = platform_init();
    state.gps_reference.valid = true;
    state.gps_reference.altitude_m = 101.25f;
    state.estimation.valid = true;
    state.estimation.orientation = (orientation_estimate_t){
        .s1_angle_rad = 0.1f,
        .s2_angle_rad = 0.2f,
        .s3_angle_rad = 0.3f,
        .s4_angle_rad = 0.4f,
    };
    state.result.valid = true;
    state.result.bucket_height = (bucket_height_result_t){
        .inputs_valid = true,
        .delta_height_m = 1.5f,
        .absolute_height_m = 102.75f,
    };
    return state;
}

int main(void)
{
    char buffer[256];
    size_t length;
    const excavator_state_t state = sample_state();

    length = telemetry_format_status_line(&state, buffer, sizeof(buffer));
    assert(length > 0u);
    assert(strstr(buffer, "gps_valid=1") != 0);
    assert(strstr(buffer, "gps_alt_m=101.250") != 0);
    assert(strstr(buffer, "delta_m=1.500") != 0);
    assert(strstr(buffer, "abs_m=102.750") != 0);
    assert(strstr(buffer, "result_valid=1") != 0);
    assert(strstr(buffer, "platform_ok=1") != 0);
    assert(strstr(buffer, "\r\n") != 0);

    assert(telemetry_format_status_line(0, buffer, sizeof(buffer)) == 0u);
    assert(telemetry_format_status_line(&state, 0, sizeof(buffer)) == 0u);
    assert(telemetry_format_status_line(&state, buffer, 0u) == 0u);

    return 0;
}
