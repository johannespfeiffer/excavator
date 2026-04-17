#include "platform.h"
#include "platform_simulated_io.h"

#include <stddef.h>
#include <stdint.h>

typedef struct {
    platform_i2c_bus_t bus;
    platform_gpio_pin_t scl;
    platform_gpio_pin_t sda;
    uint32_t bitrate_hz;
} platform_i2c_config_t;

typedef struct {
    platform_uart_t uart;
    uint32_t baud_rate;
    bool rx_enabled;
    bool tx_enabled;
} platform_uart_config_t;

static const platform_i2c_config_t k_i2c_configs[PLATFORM_I2C_COUNT] = {
    { PLATFORM_I2C1, { PLATFORM_GPIO_PORT_B, 8u }, { PLATFORM_GPIO_PORT_B, 9u }, 400000u },
    { PLATFORM_I2C2, { PLATFORM_GPIO_PORT_B, 10u }, { PLATFORM_GPIO_PORT_B, 3u }, 400000u },
    { PLATFORM_I2C3, { PLATFORM_GPIO_PORT_A, 8u }, { PLATFORM_GPIO_PORT_C, 9u }, 400000u },
    { PLATFORM_FMPI2C1, { PLATFORM_GPIO_PORT_C, 6u }, { PLATFORM_GPIO_PORT_C, 7u }, 400000u },
};

#if !defined(EXCAVATOR_TARGET_STM32F446RE)
/*
 * GPS and output UART defaults are placeholders until the concrete serial
 * protocols are fixed in the hardware integration step.
 */
static const platform_uart_config_t k_uart_configs[PLATFORM_UART_COUNT] = {
    { PLATFORM_UART_GPS, 115200u, true, false },
    { PLATFORM_UART_OUTPUT, 115200u, false, true },
};
#endif

static platform_status_t g_platform_status = {
    .backend = PLATFORM_BACKEND_SIMULATED,
    .ready_flags = 0u,
    .error_flags = PLATFORM_ERROR_NOT_IMPLEMENTED,
};

#if defined(EXCAVATOR_TARGET_STM32F446RE)
enum {
    RCC_BASE = 0x40023800u,
    RCC_AHB1ENR_OFFSET = 0x30u,
    RCC_APB1RSTR_OFFSET = 0x20u,
    RCC_APB1ENR_OFFSET = 0x40u,
    GPIOB_BASE = 0x40020400u,
    GPIO_MODER_OFFSET = 0x00u,
    GPIO_OTYPER_OFFSET = 0x04u,
    GPIO_OSPEEDR_OFFSET = 0x08u,
    GPIO_PUPDR_OFFSET = 0x0Cu,
    GPIO_AFRH_OFFSET = 0x24u,
    I2C1_BASE = 0x40005400u,
    I2C_CR1_OFFSET = 0x00u,
    I2C_CR2_OFFSET = 0x04u,
    I2C_DR_OFFSET = 0x10u,
    I2C_SR1_OFFSET = 0x14u,
    I2C_SR2_OFFSET = 0x18u,
    I2C_CCR_OFFSET = 0x1Cu,
    I2C_TRISE_OFFSET = 0x20u,
};

enum {
    RCC_AHB1ENR_GPIOBEN = 1u << 1,
    RCC_APB1RSTR_I2C1RST = 1u << 21,
    RCC_APB1ENR_I2C1EN = 1u << 21,
    I2C_CR1_PE = 1u << 0,
    I2C_CR1_START = 1u << 8,
    I2C_CR1_STOP = 1u << 9,
    I2C_CR1_ACK = 1u << 10,
    I2C_CR1_SWRST = 1u << 15,
    I2C_SR1_SB = 1u << 0,
    I2C_SR1_ADDR = 1u << 1,
    I2C_SR1_BTF = 1u << 2,
    I2C_SR1_RXNE = 1u << 6,
    I2C_SR1_TXE = 1u << 7,
    I2C_SR1_AF = 1u << 10,
    I2C_SR2_BUSY = 1u << 1,
    I2C_CCR_FS = 1u << 15,
    PLATFORM_I2C_TIMEOUT = 1000000u,
};

static volatile uint32_t *platform_reg32(uint32_t address)
{
    return (volatile uint32_t *)address;
}

static bool platform_wait_for_reg_bits(uint32_t address, uint32_t mask, bool set)
{
    uint32_t attempt;

    for (attempt = 0u; attempt < PLATFORM_I2C_TIMEOUT; ++attempt) {
        const uint32_t value = *platform_reg32(address);

        if (set ? ((value & mask) == mask) : ((value & mask) == 0u)) {
            return true;
        }
    }

    return false;
}

static void platform_i2c1_clear_addr(void)
{
    volatile uint32_t discard;

    discard = *platform_reg32(I2C1_BASE + I2C_SR1_OFFSET);
    discard = *platform_reg32(I2C1_BASE + I2C_SR2_OFFSET);
    (void)discard;
}

static void platform_i2c1_clear_ack_failure(void)
{
    *platform_reg32(I2C1_BASE + I2C_SR1_OFFSET) &= ~I2C_SR1_AF;
}

static bool platform_i2c1_start(void)
{
    volatile uint32_t *const cr1 = platform_reg32(I2C1_BASE + I2C_CR1_OFFSET);

    if (!platform_wait_for_reg_bits(I2C1_BASE + I2C_SR2_OFFSET, I2C_SR2_BUSY, false)) {
        return false;
    }

    *cr1 |= I2C_CR1_START;
    return platform_wait_for_reg_bits(I2C1_BASE + I2C_SR1_OFFSET, I2C_SR1_SB, true);
}

static bool platform_i2c1_send_address(uint8_t address_byte)
{
    uint32_t attempt;

    *platform_reg32(I2C1_BASE + I2C_DR_OFFSET) = address_byte;

    for (attempt = 0u; attempt < PLATFORM_I2C_TIMEOUT; ++attempt) {
        const uint32_t sr1 = *platform_reg32(I2C1_BASE + I2C_SR1_OFFSET);

        if ((sr1 & I2C_SR1_ADDR) != 0u) {
            return true;
        }

        if ((sr1 & I2C_SR1_AF) != 0u) {
            platform_i2c1_clear_ack_failure();
            return false;
        }
    }

    return false;
}

static bool platform_i2c1_write_byte(uint8_t value)
{
    if (!platform_wait_for_reg_bits(I2C1_BASE + I2C_SR1_OFFSET, I2C_SR1_TXE, true)) {
        return false;
    }

    *platform_reg32(I2C1_BASE + I2C_DR_OFFSET) = value;
    return platform_wait_for_reg_bits(I2C1_BASE + I2C_SR1_OFFSET, I2C_SR1_BTF, true);
}

static void platform_i2c1_stop(void)
{
    *platform_reg32(I2C1_BASE + I2C_CR1_OFFSET) |= I2C_CR1_STOP;
}

static bool platform_i2c1_write_registers_impl(uint8_t device_address,
                                               uint8_t register_address,
                                               const uint8_t *data,
                                               uint16_t length)
{
    uint16_t index;

    if (!platform_i2c1_start()) {
        return false;
    }

    if (!platform_i2c1_send_address((uint8_t)(device_address << 1))) {
        platform_i2c1_stop();
        return false;
    }

    platform_i2c1_clear_addr();

    if (!platform_i2c1_write_byte(register_address)) {
        platform_i2c1_stop();
        return false;
    }

    for (index = 0u; index < length; ++index) {
        if (!platform_i2c1_write_byte(data[index])) {
            platform_i2c1_stop();
            return false;
        }
    }

    platform_i2c1_stop();
    return true;
}

static bool platform_i2c1_read_one(uint8_t *data)
{
    volatile uint32_t *const cr1 = platform_reg32(I2C1_BASE + I2C_CR1_OFFSET);

    *cr1 &= ~I2C_CR1_ACK;
    platform_i2c1_clear_addr();
    platform_i2c1_stop();

    if (!platform_wait_for_reg_bits(I2C1_BASE + I2C_SR1_OFFSET, I2C_SR1_RXNE, true)) {
        *cr1 |= I2C_CR1_ACK;
        return false;
    }

    *data = (uint8_t)(*platform_reg32(I2C1_BASE + I2C_DR_OFFSET) & 0xFFu);
    *cr1 |= I2C_CR1_ACK;
    return true;
}

static bool platform_i2c1_read_many(uint8_t *data, uint16_t length)
{
    volatile uint32_t *const cr1 = platform_reg32(I2C1_BASE + I2C_CR1_OFFSET);
    uint16_t remaining = length;

    *cr1 |= I2C_CR1_ACK;
    platform_i2c1_clear_addr();

    while (remaining > 3u) {
        if (!platform_wait_for_reg_bits(I2C1_BASE + I2C_SR1_OFFSET, I2C_SR1_RXNE, true)) {
            platform_i2c1_stop();
            return false;
        }

        *data++ = (uint8_t)(*platform_reg32(I2C1_BASE + I2C_DR_OFFSET) & 0xFFu);
        remaining--;
    }

    if (!platform_wait_for_reg_bits(I2C1_BASE + I2C_SR1_OFFSET, I2C_SR1_BTF, true)) {
        platform_i2c1_stop();
        return false;
    }

    *cr1 &= ~I2C_CR1_ACK;
    *data++ = (uint8_t)(*platform_reg32(I2C1_BASE + I2C_DR_OFFSET) & 0xFFu);
    remaining--;

    if (!platform_wait_for_reg_bits(I2C1_BASE + I2C_SR1_OFFSET, I2C_SR1_BTF, true)) {
        platform_i2c1_stop();
        *cr1 |= I2C_CR1_ACK;
        return false;
    }

    platform_i2c1_stop();
    *data++ = (uint8_t)(*platform_reg32(I2C1_BASE + I2C_DR_OFFSET) & 0xFFu);
    remaining--;

    if (!platform_wait_for_reg_bits(I2C1_BASE + I2C_SR1_OFFSET, I2C_SR1_RXNE, true)) {
        *cr1 |= I2C_CR1_ACK;
        return false;
    }

    *data = (uint8_t)(*platform_reg32(I2C1_BASE + I2C_DR_OFFSET) & 0xFFu);
    *cr1 |= I2C_CR1_ACK;
    return remaining == 1u;
}

static bool platform_i2c1_read_registers_impl(uint8_t device_address,
                                              uint8_t register_address,
                                              uint8_t *data,
                                              uint16_t length)
{
    if (!platform_i2c1_start()) {
        return false;
    }

    if (!platform_i2c1_send_address((uint8_t)(device_address << 1))) {
        platform_i2c1_stop();
        return false;
    }

    platform_i2c1_clear_addr();

    if (!platform_i2c1_write_byte(register_address)) {
        platform_i2c1_stop();
        return false;
    }

    if (!platform_i2c1_start()) {
        platform_i2c1_stop();
        return false;
    }

    if (!platform_i2c1_send_address((uint8_t)((device_address << 1) | 0x01u))) {
        platform_i2c1_stop();
        return false;
    }

    if (length == 1u) {
        return platform_i2c1_read_one(data);
    }

    return platform_i2c1_read_many(data, length);
}

static bool platform_target_i2c1_init(const platform_i2c_config_t *config)
{
    volatile uint32_t *const rcc_ahb1enr = platform_reg32(RCC_BASE + RCC_AHB1ENR_OFFSET);
    volatile uint32_t *const rcc_apb1rstr = platform_reg32(RCC_BASE + RCC_APB1RSTR_OFFSET);
    volatile uint32_t *const rcc_apb1enr = platform_reg32(RCC_BASE + RCC_APB1ENR_OFFSET);
    volatile uint32_t *const gpiob_moder = platform_reg32(GPIOB_BASE + GPIO_MODER_OFFSET);
    volatile uint32_t *const gpiob_otyper = platform_reg32(GPIOB_BASE + GPIO_OTYPER_OFFSET);
    volatile uint32_t *const gpiob_ospeedr = platform_reg32(GPIOB_BASE + GPIO_OSPEEDR_OFFSET);
    volatile uint32_t *const gpiob_pupdr = platform_reg32(GPIOB_BASE + GPIO_PUPDR_OFFSET);
    volatile uint32_t *const gpiob_afrh = platform_reg32(GPIOB_BASE + GPIO_AFRH_OFFSET);
    volatile uint32_t *const i2c1_cr1 = platform_reg32(I2C1_BASE + I2C_CR1_OFFSET);
    volatile uint32_t *const i2c1_cr2 = platform_reg32(I2C1_BASE + I2C_CR2_OFFSET);
    volatile uint32_t *const i2c1_ccr = platform_reg32(I2C1_BASE + I2C_CCR_OFFSET);
    volatile uint32_t *const i2c1_trise = platform_reg32(I2C1_BASE + I2C_TRISE_OFFSET);

    if ((config == NULL) || (config->bus != PLATFORM_I2C1)) {
        return false;
    }

    *rcc_ahb1enr |= RCC_AHB1ENR_GPIOBEN;
    (void)*rcc_ahb1enr;

    *gpiob_moder &= ~((3u << (config->scl.pin * 2u)) | (3u << (config->sda.pin * 2u)));
    *gpiob_moder |= (2u << (config->scl.pin * 2u)) | (2u << (config->sda.pin * 2u));
    *gpiob_otyper |= (1u << config->scl.pin) | (1u << config->sda.pin);
    *gpiob_ospeedr &= ~((3u << (config->scl.pin * 2u)) | (3u << (config->sda.pin * 2u)));
    *gpiob_ospeedr |= (3u << (config->scl.pin * 2u)) | (3u << (config->sda.pin * 2u));
    *gpiob_pupdr &= ~((3u << (config->scl.pin * 2u)) | (3u << (config->sda.pin * 2u)));
    *gpiob_pupdr |= (1u << (config->scl.pin * 2u)) | (1u << (config->sda.pin * 2u));
    *gpiob_afrh &= ~((0xFu << ((config->scl.pin - 8u) * 4u)) | (0xFu << ((config->sda.pin - 8u) * 4u)));
    *gpiob_afrh |= (4u << ((config->scl.pin - 8u) * 4u)) | (4u << ((config->sda.pin - 8u) * 4u));

    *rcc_apb1enr |= RCC_APB1ENR_I2C1EN;
    *rcc_apb1rstr |= RCC_APB1RSTR_I2C1RST;
    *rcc_apb1rstr &= ~RCC_APB1RSTR_I2C1RST;

    *i2c1_cr1 = I2C_CR1_SWRST;
    *i2c1_cr1 = 0u;
    *i2c1_cr2 = 16u;
    *i2c1_ccr = I2C_CCR_FS | 13u;
    *i2c1_trise = 6u;
    *i2c1_cr1 = I2C_CR1_PE | I2C_CR1_ACK;

    return true;
}
#endif

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
#if defined(EXCAVATOR_TARGET_STM32F446RE)
    return PLATFORM_READY_CLOCK |
           PLATFORM_READY_GPIO |
           PLATFORM_READY_I2C1 |
           PLATFORM_READY_SELF_TEST;
#else
    return PLATFORM_READY_CLOCK |
           PLATFORM_READY_GPIO |
           PLATFORM_READY_UART_GPS |
           PLATFORM_READY_UART_OUTPUT |
           PLATFORM_READY_I2C1 |
           PLATFORM_READY_I2C2 |
           PLATFORM_READY_I2C3 |
           PLATFORM_READY_FMPI2C1 |
           PLATFORM_READY_SELF_TEST;
#endif
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
#if defined(EXCAVATOR_TARGET_STM32F446RE)
    (void)status;
#else
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
#endif
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

#if defined(EXCAVATOR_TARGET_STM32F446RE)
        if (config->bus != PLATFORM_I2C1) {
            continue;
        }

        if (!platform_target_i2c1_init(config)) {
            status->error_flags |= PLATFORM_ERROR_I2C_CONFIG;
            continue;
        }
#endif

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
    platform_init_clock(&status);
    platform_init_gpio(&status);
    platform_init_uarts(&status);
    platform_init_i2c_buses(&status);
    platform_run_self_test(&status);
    g_platform_status = status;

    return status;
#else
    platform_init_clock(&status);
    platform_init_gpio(&status);
    platform_init_uarts(&status);
    platform_init_i2c_buses(&status);
    platform_simulated_reset();
    platform_run_self_test(&status);
    g_platform_status = status;

    return status;
#endif
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

bool platform_i2c_pinout(platform_i2c_bus_t bus, platform_i2c_pinout_t *pinout)
{
    if ((bus >= PLATFORM_I2C_COUNT) || (pinout == NULL)) {
        return false;
    }

    pinout->bus = k_i2c_configs[bus].bus;
    pinout->scl = k_i2c_configs[bus].scl;
    pinout->sda = k_i2c_configs[bus].sda;
    pinout->bitrate_hz = k_i2c_configs[bus].bitrate_hz;

    return true;
}

platform_io_status_t platform_i2c_write_registers(platform_i2c_bus_t bus,
                                                  uint8_t device_address,
                                                  uint8_t register_address,
                                                  const uint8_t *data,
                                                  uint16_t length)
{
#if defined(EXCAVATOR_TARGET_STM32F446RE)
    if ((bus >= PLATFORM_I2C_COUNT) || ((length > 0u) && (data == NULL))) {
        return PLATFORM_IO_STATUS_INVALID_ARGUMENT;
    }

    if ((bus != PLATFORM_I2C1) || !platform_status_ok(g_platform_status) || !platform_i2c_ready(g_platform_status, bus)) {
        return PLATFORM_IO_STATUS_NOT_READY;
    }

    return platform_i2c1_write_registers_impl(device_address, register_address, data, length) ?
               PLATFORM_IO_STATUS_OK :
               PLATFORM_IO_STATUS_NOT_FOUND;
#else
    if (bus >= PLATFORM_I2C_COUNT) {
        return PLATFORM_IO_STATUS_INVALID_ARGUMENT;
    }

    if (!platform_status_ok(g_platform_status) || !platform_i2c_ready(g_platform_status, bus)) {
        return PLATFORM_IO_STATUS_NOT_READY;
    }
    return platform_simulated_i2c_write_registers(bus, device_address, register_address, data, length);
#endif
}

platform_io_status_t platform_i2c_read_registers(platform_i2c_bus_t bus,
                                                 uint8_t device_address,
                                                 uint8_t register_address,
                                                 uint8_t *data,
                                                 uint16_t length)
{
#if defined(EXCAVATOR_TARGET_STM32F446RE)
    if ((bus >= PLATFORM_I2C_COUNT) || ((length > 0u) && (data == NULL))) {
        return PLATFORM_IO_STATUS_INVALID_ARGUMENT;
    }

    if ((bus != PLATFORM_I2C1) || !platform_status_ok(g_platform_status) || !platform_i2c_ready(g_platform_status, bus)) {
        return PLATFORM_IO_STATUS_NOT_READY;
    }

    return platform_i2c1_read_registers_impl(device_address, register_address, data, length) ?
               PLATFORM_IO_STATUS_OK :
               PLATFORM_IO_STATUS_NOT_FOUND;
#else
    if (bus >= PLATFORM_I2C_COUNT) {
        return PLATFORM_IO_STATUS_INVALID_ARGUMENT;
    }

    if (!platform_status_ok(g_platform_status) || !platform_i2c_ready(g_platform_status, bus)) {
        return PLATFORM_IO_STATUS_NOT_READY;
    }
    return platform_simulated_i2c_read_registers(bus, device_address, register_address, data, length);
#endif
}

platform_io_status_t platform_uart_read_byte(platform_uart_t uart, uint8_t *byte_out)
{
#if defined(EXCAVATOR_TARGET_STM32F446RE)
    (void)uart;
    (void)byte_out;
    return PLATFORM_IO_STATUS_NOT_IMPLEMENTED;
#else
    if ((uart >= PLATFORM_UART_COUNT) || (byte_out == NULL)) {
        return PLATFORM_IO_STATUS_INVALID_ARGUMENT;
    }

    if (!platform_status_ok(g_platform_status) || !platform_uart_ready(g_platform_status, uart)) {
        return PLATFORM_IO_STATUS_NOT_READY;
    }
    return platform_simulated_uart_read_byte(uart, byte_out);
#endif
}

bool platform_simulated_i2c_set_registers(platform_i2c_bus_t bus,
                                          uint8_t device_address,
                                          uint8_t register_address,
                                          const uint8_t *data,
                                          uint16_t length)
{
#if defined(EXCAVATOR_TARGET_STM32F446RE)
    (void)bus;
    (void)device_address;
    (void)register_address;
    (void)data;
    (void)length;
    return false;
#else
    return platform_simulated_i2c_set_registers_impl(bus, device_address, register_address, data, length);
#endif
}

bool platform_simulated_i2c_get_registers(platform_i2c_bus_t bus,
                                          uint8_t device_address,
                                          uint8_t register_address,
                                          uint8_t *data,
                                          uint16_t length)
{
#if defined(EXCAVATOR_TARGET_STM32F446RE)
    (void)bus;
    (void)device_address;
    (void)register_address;
    (void)data;
    (void)length;
    return false;
#else
    return platform_simulated_i2c_get_registers_impl(bus, device_address, register_address, data, length);
#endif
}

platform_io_status_t platform_simulated_uart_feed(platform_uart_t uart,
                                                  const uint8_t *data,
                                                  uint16_t length)
{
#if defined(EXCAVATOR_TARGET_STM32F446RE)
    (void)uart;
    (void)data;
    (void)length;
    return PLATFORM_IO_STATUS_NOT_IMPLEMENTED;
#else
    return platform_simulated_uart_feed_impl(uart, data, length);
#endif
}
