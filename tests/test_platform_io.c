#include "platform.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

int main(void)
{
    uint8_t value = 0u;
    uint8_t readback[2] = {0u, 0u};
    uint8_t byte = 0u;
    uint8_t big_buffer[257];
    const uint8_t write_values[2] = {0x12u, 0x34u};
    const uint8_t uart_payload[3] = {'A', 'B', 'C'};
    unsigned int index;

    assert(platform_status_ok(platform_init()));

    assert(platform_i2c_probe_address(PLATFORM_I2C_COUNT, 0x68u) == PLATFORM_IO_STATUS_INVALID_ARGUMENT);
    assert(platform_i2c_probe_address(PLATFORM_I2C1, 0x68u) == PLATFORM_IO_STATUS_OK);
    assert(platform_i2c_probe_address(PLATFORM_I2C1, 0x69u) == PLATFORM_IO_STATUS_NOT_FOUND);
    assert(platform_i2c_read_registers(PLATFORM_I2C_COUNT, 0x68u, 0x00u, &value, 1u) ==
           PLATFORM_IO_STATUS_INVALID_ARGUMENT);
    assert(platform_i2c_read_registers(PLATFORM_I2C1, 0x42u, 0x00u, &value, 1u) == PLATFORM_IO_STATUS_NOT_FOUND);
    assert(platform_i2c_write_registers(PLATFORM_I2C1, 0x68u, 0x20u, write_values, 2u) == PLATFORM_IO_STATUS_OK);
    assert(platform_i2c_read_registers(PLATFORM_I2C1, 0x68u, 0x20u, readback, 2u) == PLATFORM_IO_STATUS_OK);
    assert(readback[0] == 0x12u);
    assert(readback[1] == 0x34u);

    assert(platform_uart_read_byte(PLATFORM_UART_COUNT, &byte) == PLATFORM_IO_STATUS_INVALID_ARGUMENT);
    assert(platform_uart_read_byte(PLATFORM_UART_GPS, NULL) == PLATFORM_IO_STATUS_INVALID_ARGUMENT);
    assert(platform_simulated_uart_feed(PLATFORM_UART_GPS, uart_payload, 3u) == PLATFORM_IO_STATUS_OK);
    assert(platform_uart_read_byte(PLATFORM_UART_GPS, &byte) == PLATFORM_IO_STATUS_OK);
    assert(byte == 'A');
    assert(platform_uart_read_byte(PLATFORM_UART_GPS, &byte) == PLATFORM_IO_STATUS_OK);
    assert(byte == 'B');
    assert(platform_uart_read_byte(PLATFORM_UART_GPS, &byte) == PLATFORM_IO_STATUS_OK);
    assert(byte == 'C');
    assert(platform_uart_read_byte(PLATFORM_UART_GPS, &byte) == PLATFORM_IO_STATUS_EMPTY);

    for (index = 0u; index < sizeof(big_buffer); ++index) {
        big_buffer[index] = (uint8_t)index;
    }
    assert(platform_init().backend == PLATFORM_BACKEND_SIMULATED);
    assert(platform_simulated_uart_feed(PLATFORM_UART_GPS, big_buffer, (uint16_t)sizeof(big_buffer)) ==
           PLATFORM_IO_STATUS_OVERFLOW);

    return 0;
}
