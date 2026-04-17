#include "gps_parser.h"

#include "platform.h"

#include <stddef.h>

enum {
    GPS_RING_BUFFER_SIZE = 256,
    GPS_LINE_BUFFER_SIZE = 128,
    GPS_GGA_ALTITUDE_FIELD_INDEX = 9,
};

typedef struct {
    uint8_t buffer[GPS_RING_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} gps_ring_buffer_t;

static gps_ring_buffer_t g_rx_ring = {{0}, 0u, 0u, 0u};
static char g_line_buffer[GPS_LINE_BUFFER_SIZE];
static size_t g_line_length = 0u;

static bool gps_ring_push_byte(uint8_t byte)
{
    if (g_rx_ring.count >= GPS_RING_BUFFER_SIZE) {
        return false;
    }

    g_rx_ring.buffer[g_rx_ring.head] = byte;
    g_rx_ring.head = (uint16_t)((g_rx_ring.head + 1u) % GPS_RING_BUFFER_SIZE);
    g_rx_ring.count++;
    return true;
}

static bool gps_ring_pop_byte(uint8_t *byte_out)
{
    if ((byte_out == NULL) || (g_rx_ring.count == 0u)) {
        return false;
    }

    *byte_out = g_rx_ring.buffer[g_rx_ring.tail];
    g_rx_ring.tail = (uint16_t)((g_rx_ring.tail + 1u) % GPS_RING_BUFFER_SIZE);
    g_rx_ring.count--;
    return true;
}

static bool gps_string_contains(const char *haystack, const char *needle)
{
    size_t start = 0u;
    size_t offset;

    if ((haystack == NULL) || (needle == NULL) || (needle[0] == '\0')) {
        return false;
    }

    while (haystack[start] != '\0') {
        offset = 0u;
        while ((needle[offset] != '\0') && (haystack[start + offset] == needle[offset])) {
            offset++;
        }

        if (needle[offset] == '\0') {
            return true;
        }

        start++;
    }

    return false;
}

static float gps_parse_positive_float(const char *text, bool *ok)
{
    float value = 0.0f;
    float divisor = 1.0f;
    bool seen_digit = false;
    bool fractional = false;
    size_t index = 0u;

    if (ok != NULL) {
        *ok = false;
    }

    if (text == NULL) {
        return 0.0f;
    }

    while (text[index] != '\0') {
        const char c = text[index];

        if (c == '.') {
            if (fractional) {
                return 0.0f;
            }
            fractional = true;
            index++;
            continue;
        }

        if ((c < '0') || (c > '9')) {
            return 0.0f;
        }

        seen_digit = true;
        if (!fractional) {
            value = (value * 10.0f) + (float)(c - '0');
        } else {
            divisor *= 10.0f;
            value += (float)(c - '0') / divisor;
        }

        index++;
    }

    if (ok != NULL) {
        *ok = seen_digit;
    }

    return value;
}

static gps_fix_t gps_parse_gga_sentence(const char *line)
{
    char talker_and_type[8];
    char altitude_field[16];
    size_t line_index = 0u;
    size_t field_index = 0u;
    size_t field_char_index = 0u;
    bool altitude_ok = false;

    if (line == NULL) {
        return (gps_fix_t){ false, 0.0f };
    }

    while ((line[line_index] != '\0') &&
           (line[line_index] != ',') &&
           (line[line_index] != '*') &&
           (field_char_index + 1u < sizeof(talker_and_type))) {
        talker_and_type[field_char_index++] = line[line_index++];
    }
    talker_and_type[field_char_index] = '\0';

    if (!gps_string_contains(talker_and_type, "GGA")) {
        return (gps_fix_t){ false, 0.0f };
    }

    field_index = 0u;
    field_char_index = 0u;

    /* Only the GGA altitude field is needed for the current height pipeline. */
    while (line[line_index] != '\0') {
        const char c = line[line_index++];

        if ((c == ',') || (c == '*')) {
            if (field_index == GPS_GGA_ALTITUDE_FIELD_INDEX) {
                altitude_field[field_char_index] = '\0';
                const float altitude_m = gps_parse_positive_float(altitude_field, &altitude_ok);
                return (gps_fix_t){
                    .valid = altitude_field[0] != '\0' && altitude_ok,
                    .altitude_m = altitude_ok ? altitude_m : 0.0f,
                };
            }

            field_index++;
            field_char_index = 0u;
            if (c == '*') {
                break;
            }
            continue;
        }

        if ((field_index == GPS_GGA_ALTITUDE_FIELD_INDEX) && (field_char_index + 1u < sizeof(altitude_field))) {
            altitude_field[field_char_index++] = c;
        }
    }

    return (gps_fix_t){ false, 0.0f };
}

void gps_parser_reset(void)
{
    g_rx_ring.head = 0u;
    g_rx_ring.tail = 0u;
    g_rx_ring.count = 0u;
    g_line_length = 0u;
}

bool gps_parser_push_bytes(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    if ((length > 0u) && (data == NULL)) {
        return false;
    }

    for (index = 0u; index < length; ++index) {
        if (!gps_ring_push_byte(data[index])) {
            return false;
        }
    }

    return true;
}

bool gps_parser_next_fix(gps_fix_t *fix)
{
    uint8_t byte = 0u;

    if (fix == NULL) {
        return false;
    }

    /* Preserve partial lines across calls so chunked UART input is safe. */
    while (gps_ring_pop_byte(&byte)) {
        if (byte == '\r') {
            continue;
        }

        if (byte == '\n') {
            g_line_buffer[g_line_length] = '\0';
            *fix = gps_parser_consume_line(g_line_buffer);
            g_line_length = 0u;
            return fix->valid;
        }

        if (g_line_length + 1u >= sizeof(g_line_buffer)) {
            g_line_length = 0u;
            continue;
        }

        g_line_buffer[g_line_length++] = (char)byte;
    }

    return false;
}

bool gps_parser_poll_uart(gps_fix_t *fix)
{
    uint8_t byte = 0u;
    platform_io_status_t io_status;

    if (fix == NULL) {
        return false;
    }

    /* Drain the simulated UART first, then let the line assembler emit at most one fix. */
    do {
        io_status = platform_uart_read_byte(PLATFORM_UART_GPS, &byte);
        if (io_status == PLATFORM_IO_STATUS_OK) {
            if (!gps_parser_push_bytes(&byte, 1u)) {
                return false;
            }
        }
    } while (io_status == PLATFORM_IO_STATUS_OK);

    return gps_parser_next_fix(fix);
}

gps_fix_t gps_parser_consume_line(const char *line)
{
    if ((line == NULL) || (line[0] != '$')) {
        return (gps_fix_t){ false, 0.0f };
    }

    return gps_parse_gga_sentence(line + 1);
}
