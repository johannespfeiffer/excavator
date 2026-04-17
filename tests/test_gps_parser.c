#include "gps_parser.h"
#include "platform.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

static int float_close(float a, float b, float tolerance)
{
    const float diff = (a > b) ? (a - b) : (b - a);
    return diff <= tolerance;
}

int main(void)
{
    gps_fix_t fix = {0};
    const gps_fix_t direct_fix =
        gps_parser_consume_line("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47");

    assert(platform_status_ok(platform_init()));
    assert(direct_fix.valid);
    assert(float_close(direct_fix.altitude_m, 545.4f, 0.001f));

    gps_parser_reset();
    {
        const uint8_t chunk1[] = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,";
        const uint8_t chunk2[] = "545.4,M,46.9,M,,*47\r\n";

        assert(gps_parser_push_bytes(chunk1, (uint16_t)(sizeof(chunk1) - 1u)));
        assert(!gps_parser_next_fix(&fix));
        assert(gps_parser_push_bytes(chunk2, (uint16_t)(sizeof(chunk2) - 1u)));
        assert(gps_parser_next_fix(&fix));
        assert(fix.valid);
        assert(float_close(fix.altitude_m, 545.4f, 0.001f));
    }

    gps_parser_reset();
    {
        const uint8_t invalid_sentence[] = "$GPRMC,092204.999,A,4250.5589,S,14718.5084,E,0.02,31.66,200804,,,A*43\r\n";

        assert(gps_parser_push_bytes(invalid_sentence, (uint16_t)(sizeof(invalid_sentence) - 1u)));
        assert(!gps_parser_next_fix(&fix));
    }

    gps_parser_reset();
    {
        const char *uart_sentence = "$GNGGA,101010,4807.038,N,01131.000,E,1,10,0.8,123.7,M,46.9,M,,*52\r\n";
        assert(platform_simulated_uart_feed(PLATFORM_UART_GPS,
                                            (const uint8_t *)uart_sentence,
                                            (uint16_t)strlen(uart_sentence)) == PLATFORM_IO_STATUS_OK);
        assert(gps_parser_poll_uart(&fix));
        assert(fix.valid);
        assert(float_close(fix.altitude_m, 123.7f, 0.001f));
    }

    gps_parser_reset();
    {
        const uint8_t two_sentences[] =
            "$GPGGA,111111,4807.038,N,01131.000,E,1,08,0.9,10.1,M,46.9,M,,*00\r\n"
            "$GPGGA,222222,4807.038,N,01131.000,E,1,08,0.9,20.2,M,46.9,M,,*00\r\n";

        assert(gps_parser_push_bytes(two_sentences, (uint16_t)(sizeof(two_sentences) - 1u)));
        assert(gps_parser_next_fix(&fix));
        assert(float_close(fix.altitude_m, 10.1f, 0.001f));
        assert(gps_parser_next_fix(&fix));
        assert(float_close(fix.altitude_m, 20.2f, 0.001f));
    }

    gps_parser_reset();
    {
        const gps_fix_t invalid_altitude =
            gps_parser_consume_line("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,ABC,M,46.9,M,,*47");
        assert(!invalid_altitude.valid);
        assert(float_close(invalid_altitude.altitude_m, 0.0f, 0.001f));
    }

    gps_parser_reset();
    assert(!gps_parser_push_bytes(NULL, 1u));
    assert(!gps_parser_consume_line(NULL).valid);
    assert(platform_uart_read_byte(PLATFORM_UART_GPS, &((uint8_t){0})) == PLATFORM_IO_STATUS_EMPTY);

    return 0;
}
