#include "platform.h"

#include <stddef.h>
#include <stdint.h>

typedef struct {
    platform_i2c_bus_t bus;
    uint32_t bitrate_hz;
} platform_i2c_config_t;

typedef struct {
    platform_uart_t uart;
    uint32_t baud_rate;
    bool rx_enabled;
    bool tx_enabled;
} platform_uart_config_t;

static const platform_i2c_config_t k_i2c_configs[PLATFORM_I2C_COUNT] = {
    { PLATFORM_I2C1, 400000u },
    { PLATFORM_I2C2, 400000u },
    { PLATFORM_I2C3, 400000u },
    { PLATFORM_FMPI2C1, 400000u },
};

/*
 * GPS and output UART defaults are placeholders until the concrete serial
 * protocols are fixed in the hardware integration step.
 */
static const platform_uart_config_t k_uart_configs[PLATFORM_UART_COUNT] = {
    { PLATFORM_UART_GPS, 115200u, true, false },
    { PLATFORM_UART_OUTPUT, 115200u, false, true },
};

static platform_backend_t platform_detect_backend(void)
{
#if defined(EXCAVATOR_TARGET_STM32F446RE)
    return PLATFORM_BACKEND_STM32F446RE;
#else
    return PLATFORM_BACKEND_SIMULATED;
#endif
}

static uint32_t platform_required_ready_mask(void)
{
    return PLATFORM_READY_CLOCK |
           PLATFORM_READY_GPIO |
           PLATFORM_READY_UART_GPS |
           PLATFORM_READY_UART_OUTPUT |
           PLATFORM_READY_I2C1 |
           PLATFORM_READY_I2C2 |
           PLATFORM_READY_I2C3 |
           PLATFORM_READY_FMPI2C1 |
           PLATFORM_READY_SELF_TEST;
}

static uint32_t platform_i2c_ready_flag(platform_i2c_bus_t bus)
{
    switch (bus) {
    case PLATFORM_I2C1:
        return PLATFORM_READY_I2C1;
    case PLATFORM_I2C2:
        return PLATFORM_READY_I2C2;
    case PLATFORM_I2C3:
        return PLATFORM_READY_I2C3;
    case PLATFORM_FMPI2C1:
        return PLATFORM_READY_FMPI2C1;
    case PLATFORM_I2C_COUNT:
    default:
        return 0u;
    }
}

static uint32_t platform_uart_ready_flag(platform_uart_t uart)
{
    switch (uart) {
    case PLATFORM_UART_GPS:
        return PLATFORM_READY_UART_GPS;
    case PLATFORM_UART_OUTPUT:
        return PLATFORM_READY_UART_OUTPUT;
    case PLATFORM_UART_COUNT:
    default:
        return 0u;
    }
}

static void platform_init_clock(platform_status_t *status)
{
    if (status == NULL) {
        return;
    }

    status->ready_flags |= PLATFORM_READY_CLOCK;
}

static void platform_init_gpio(platform_status_t *status)
{
    if (status == NULL) {
        return;
    }

    status->ready_flags |= PLATFORM_READY_GPIO;
}

static void platform_init_uarts(platform_status_t *status)
{
    size_t index;

    if (status == NULL) {
        return;
    }

    for (index = 0; index < PLATFORM_UART_COUNT; ++index) {
        const platform_uart_config_t *config = &k_uart_configs[index];

        if ((config->rx_enabled == config->tx_enabled) || (config->baud_rate == 0u)) {
            status->error_flags |= PLATFORM_ERROR_UART_CONFIG;
            continue;
        }

        status->ready_flags |= platform_uart_ready_flag(config->uart);
    }
}

static void platform_init_i2c_buses(platform_status_t *status)
{
    size_t index;

    if (status == NULL) {
        return;
    }

    for (index = 0; index < PLATFORM_I2C_COUNT; ++index) {
        const platform_i2c_config_t *config = &k_i2c_configs[index];

        if ((config->bus >= PLATFORM_I2C_COUNT) || (config->bitrate_hz == 0u)) {
            status->error_flags |= PLATFORM_ERROR_I2C_CONFIG;
            continue;
        }

        status->ready_flags |= platform_i2c_ready_flag(config->bus);
    }
}

static void platform_run_self_test(platform_status_t *status)
{
    const uint32_t required_without_self_test = platform_required_ready_mask() & ~PLATFORM_READY_SELF_TEST;

    if (status == NULL) {
        return;
    }

    if ((status->error_flags != PLATFORM_ERROR_NONE) ||
        ((status->ready_flags & required_without_self_test) != required_without_self_test)) {
        status->error_flags |= PLATFORM_ERROR_SELF_TEST;
        return;
    }

    status->ready_flags |= PLATFORM_READY_SELF_TEST;
}

platform_status_t platform_init(void)
{
    platform_status_t status = {
        .backend = platform_detect_backend(),
        .ready_flags = 0u,
        .error_flags = 0u,
    };

#if defined(EXCAVATOR_TARGET_STM32F446RE)
    status.error_flags |= PLATFORM_ERROR_NOT_IMPLEMENTED;
    return status;
#endif

    platform_init_clock(&status);
    platform_init_gpio(&status);
    platform_init_uarts(&status);
    platform_init_i2c_buses(&status);
    platform_run_self_test(&status);

    return status;
}

platform_backend_t platform_backend(platform_status_t status)
{
    return status.backend;
}

bool platform_status_ok(platform_status_t status)
{
    return (status.error_flags == PLATFORM_ERROR_NONE) &&
           ((status.ready_flags & platform_required_ready_mask()) == platform_required_ready_mask());
}

bool platform_uart_ready(platform_status_t status, platform_uart_t uart)
{
    const uint32_t ready_flag = platform_uart_ready_flag(uart);

    return (ready_flag != 0u) && ((status.ready_flags & ready_flag) == ready_flag);
}

bool platform_i2c_ready(platform_status_t status, platform_i2c_bus_t bus)
{
    const uint32_t ready_flag = platform_i2c_ready_flag(bus);

    return (ready_flag != 0u) && ((status.ready_flags & ready_flag) == ready_flag);
}
