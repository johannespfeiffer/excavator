#ifndef EXCAVATOR_GPS_PARSER_H
#define EXCAVATOR_GPS_PARSER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool valid;
    float altitude_m;
} gps_fix_t;

void gps_parser_reset(void);
bool gps_parser_push_bytes(const uint8_t *data, uint16_t length);
bool gps_parser_next_fix(gps_fix_t *fix);
bool gps_parser_poll_uart(gps_fix_t *fix);
gps_fix_t gps_parser_consume_line(const char *line);

#endif
