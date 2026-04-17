#include "app.h"
#include "platform.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

int main(void)
{
    uint8_t tx_buffer[256];
    uint16_t tx_length = 0u;
    const int result = app_run();

    assert(result == 0);
    assert(platform_simulated_uart_copy_tx(PLATFORM_UART_OUTPUT, tx_buffer, sizeof(tx_buffer), &tx_length) ==
           PLATFORM_IO_STATUS_OK);
    assert(tx_length > 0u);
    tx_buffer[tx_length] = '\0';
    assert(strstr((const char *)tx_buffer, "gps_valid=0") != 0);
    assert(strstr((const char *)tx_buffer, "delta_m=0.000") != 0);
    assert(strstr((const char *)tx_buffer, "platform_ok=1") != 0);
    return 0;
}
