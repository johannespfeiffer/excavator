#ifndef EXCAVATOR_TELEMETRY_H
#define EXCAVATOR_TELEMETRY_H

#include "excavator_state.h"

#include <stddef.h>

size_t telemetry_format_status_line(const excavator_state_t *state, char *buffer, size_t buffer_size);

#endif
