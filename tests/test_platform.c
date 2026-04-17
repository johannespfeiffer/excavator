#include "platform.h"

#include <assert.h>

int main(void)
{
    const platform_status_t status = platform_init();
    const platform_status_t empty_status = {
        .backend = PLATFORM_BACKEND_STM32F446RE,
        .ready_flags = 0u,
        .error_flags = PLATFORM_ERROR_NOT_IMPLEMENTED,
    };

    assert(platform_backend(status) == PLATFORM_BACKEND_SIMULATED);
    assert(platform_status_ok(status));
    assert(platform_uart_ready(status, PLATFORM_UART_GPS));
    assert(platform_uart_ready(status, PLATFORM_UART_OUTPUT));
    assert(platform_i2c_ready(status, PLATFORM_I2C1));
    assert(platform_i2c_ready(status, PLATFORM_I2C2));
    assert(platform_i2c_ready(status, PLATFORM_I2C3));
    assert(platform_i2c_ready(status, PLATFORM_FMPI2C1));
    assert(!platform_uart_ready(status, PLATFORM_UART_COUNT));
    assert(!platform_i2c_ready(status, PLATFORM_I2C_COUNT));
    assert(platform_backend(empty_status) == PLATFORM_BACKEND_STM32F446RE);
    assert(!platform_status_ok(empty_status));

    return 0;
}
