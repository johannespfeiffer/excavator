#ifndef EXCAVATOR_GPS_PARSER_H
#define EXCAVATOR_GPS_PARSER_H

#include <stdbool.h>

typedef struct {
    bool valid;
    float altitude_m;
} gps_fix_t;

void gps_parser_reset(void);
gps_fix_t gps_parser_consume_line(const char *line);

#endif
