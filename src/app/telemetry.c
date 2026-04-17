#include "telemetry.h"

#include <stdio.h>

size_t telemetry_format_status_line(const excavator_state_t *state, char *buffer, size_t buffer_size)
{
    int written = 0;

    if ((state == 0) || (buffer == 0) || (buffer_size == 0u)) {
        return 0u;
    }

    written = snprintf(buffer,
                       buffer_size,
                       "gps_valid=%u gps_alt_m=%.3f delta_m=%.3f abs_m=%.3f result_valid=%u inputs_ready=%u "
                       "platform_ok=%u s1=%.3f s2=%.3f s3=%.3f s4=%.3f\r\n",
                       state->gps_reference.valid ? 1u : 0u,
                       state->gps_reference.altitude_m,
                       state->result.bucket_height.delta_height_m,
                       state->result.bucket_height.absolute_height_m,
                       state->result.valid ? 1u : 0u,
                       excavator_state_inputs_ready(state) ? 1u : 0u,
                       platform_status_ok(state->platform_status) ? 1u : 0u,
                       state->estimation.orientation.s1_angle_rad,
                       state->estimation.orientation.s2_angle_rad,
                       state->estimation.orientation.s3_angle_rad,
                       state->estimation.orientation.s4_angle_rad);

    if (written < 0) {
        buffer[0] = '\0';
        return 0u;
    }

    if ((size_t)written >= buffer_size) {
        return buffer_size - 1u;
    }

    return (size_t)written;
}
