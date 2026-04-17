#include "gps_parser.h"

void gps_parser_reset(void)
{
}

gps_fix_t gps_parser_consume_line(const char *line)
{
    (void)line;

    return (gps_fix_t){
        .valid = false,
        .altitude_m = 0.0f,
    };
}
