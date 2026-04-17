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
    { PLATFORM_I2C1, { PLATFORM_GPIO_PORT_B, 8u }, { PLATFORM_GPIO_PORT_B, 9u }, 100000u },
    { PLATFORM_I2C2, { PLATFORM_GPIO_PORT_B, 10u }, { PLATFORM_GPIO_PORT_C, 12u }, 100000u },
    { PLATFORM_I2C3, { PLATFORM_GPIO_PORT_A, 8u }, { PLATFORM_GPIO_PORT_C, 9u }, 100000u },
    { PLATFORM_FMPI2C1, { PLATFORM_GPIO_PORT_C, 6u }, { PLATFORM_GPIO_PORT_C, 7u }, 100000u },
};

/*
 * Target assumption for bench logging:
 * PLATFORM_UART_OUTPUT uses USART2 TX on PA2 at 115200 baud, which matches
 * the common Nucleo/ST-LINK VCP wiring.
 */
static const platform_uart_config_t k_uart_configs[PLATFORM_UART_COUNT] = {
    { PLATFORM_UART_GPS, 115200u, true, false },
    { PLATFORM_UART_OUTPUT, 115200u, false, true },
};

static platform_status_t g_platform_status = {
    .backend = PLATFORM_BACKEND_SIMULATED,
    .ready_flags = 0u,
    .error_flags = PLATFORM_ERROR_NOT_IMPLEMENTED,
};

#if defined(EXCAVATOR_TARGET_STM32F446RE)
#include <stm32f446xx.h>

enum {
    PLATFORM_I2C_TIMEOUT = 10000u,
};

static platform_i2c_debug_t g_i2c1_debug;

static void platform_i2c1_debug_record(uint32_t step)
{
    g_i2c1_debug.step = step;
    g_i2c1_debug.sr1  = I2C1->SR1;
    g_i2c1_debug.sr2  = I2C1->SR2;
}

static bool platform_wait_for_reg_bits(volatile uint32_t *reg, uint32_t mask, bool set)
{
    uint32_t attempt;

    for (attempt = 0u; attempt < PLATFORM_I2C_TIMEOUT; ++attempt) {
        const uint32_t value = *reg;

        if (set ? ((value & mask) == mask) : ((value & mask) == 0u)) {
            return true;
        }
    }

    return false;
}

static void platform_i2c1_clear_addr(void)
{
    volatile uint32_t discard;

    discard = I2C1->SR1;
    discard = I2C1->SR2;
    (void)discard;
}

static void platform_i2c1_clear_ack_failure(void)
{
    I2C1->SR1 &= ~I2C_SR1_AF_Msk;
}

static bool platform_i2c1_start(void)
{
    if (!platform_wait_for_reg_bits(&I2C1->SR2, I2C_SR2_BUSY_Msk, false)) {
        platform_i2c1_debug_record(1u); /* step 1: bus busy, never released */
        return false;
    }

    I2C1->CR1 |= I2C_CR1_START_Msk;

    if (!platform_wait_for_reg_bits(&I2C1->SR1, I2C_SR1_SB_Msk, true)) {
        platform_i2c1_debug_record(2u); /* step 2: START sent but SB never set */
        return false;
    }

    return true;
}

static bool platform_i2c1_send_address(uint8_t address_byte)
{
    uint32_t attempt;

    I2C1->DR = address_byte;

    for (attempt = 0u; attempt < PLATFORM_I2C_TIMEOUT; ++attempt) {
        const uint32_t sr1 = I2C1->SR1;

        if ((sr1 & I2C_SR1_ADDR_Msk) != 0u) {
            g_i2c1_debug.step = 0u; /* success */
            return true;
        }

        if ((sr1 & I2C_SR1_AF_Msk) != 0u) {
            platform_i2c1_debug_record(3u); /* step 3: NACK received (AF flag) */
            platform_i2c1_clear_ack_failure();
            return false;
        }
    }

    platform_i2c1_debug_record(4u); /* step 4: timeout, no ADDR and no AF */
    return false;
}

static bool platform_i2c1_write_byte(uint8_t value)
{
    if (!platform_wait_for_reg_bits(&I2C1->SR1, I2C_SR1_TXE_Msk, true)) {
        platform_i2c1_debug_record(5u); /* step 5: TXE timeout */
        return false;
    }

    I2C1->DR = value;

    if (!platform_wait_for_reg_bits(&I2C1->SR1, I2C_SR1_BTF_Msk, true)) {
        platform_i2c1_debug_record(6u); /* step 6: BTF timeout after write */
        return false;
    }

    return true;
}

static void platform_i2c1_stop(void)
{
    I2C1->CR1 |= I2C_CR1_STOP_Msk;
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
    I2C1->CR1 &= ~I2C_CR1_ACK_Msk;
    platform_i2c1_clear_addr();
    platform_i2c1_stop();

    if (!platform_wait_for_reg_bits(&I2C1->SR1, I2C_SR1_RXNE_Msk, true)) {
        I2C1->CR1 |= I2C_CR1_ACK_Msk;
        return false;
    }

    *data = (uint8_t)(I2C1->DR & 0xFFu);
    I2C1->CR1 |= I2C_CR1_ACK_Msk;
    return true;
}

static bool platform_i2c1_read_many(uint8_t *data, uint16_t length)
{
    uint16_t remaining = length;

    I2C1->CR1 |= I2C_CR1_ACK_Msk;
    platform_i2c1_clear_addr();

    while (remaining > 3u) {
        if (!platform_wait_for_reg_bits(&I2C1->SR1, I2C_SR1_RXNE_Msk, true)) {
            platform_i2c1_stop();
            return false;
        }

        *data++ = (uint8_t)(I2C1->DR & 0xFFu);
        remaining--;
    }

    if (!platform_wait_for_reg_bits(&I2C1->SR1, I2C_SR1_BTF_Msk, true)) {
        platform_i2c1_stop();
        return false;
    }

    I2C1->CR1 &= ~I2C_CR1_ACK_Msk;
    *data++ = (uint8_t)(I2C1->DR & 0xFFu);
    remaining--;

    if (!platform_wait_for_reg_bits(&I2C1->SR1, I2C_SR1_BTF_Msk, true)) {
        platform_i2c1_stop();
        I2C1->CR1 |= I2C_CR1_ACK_Msk;
        return false;
    }

    platform_i2c1_stop();
    *data++ = (uint8_t)(I2C1->DR & 0xFFu);
    remaining--;

    if (!platform_wait_for_reg_bits(&I2C1->SR1, I2C_SR1_RXNE_Msk, true)) {
        I2C1->CR1 |= I2C_CR1_ACK_Msk;
        return false;
    }

    *data = (uint8_t)(I2C1->DR & 0xFFu);
    I2C1->CR1 |= I2C_CR1_ACK_Msk;
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
    if ((config == NULL) || (config->bus != PLATFORM_I2C1)) {
        return false;
    }

    /* Enable GPIOB clock and flush before touching GPIO registers. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN_Msk;
    (void)RCC->AHB1ENR;

    /* PB8 (SCL) and PB9 (SDA): AF mode (10b), open-drain, high speed, pull-up, AF4. */
    GPIOB->MODER   &= ~((3u << (config->scl.pin * 2u)) | (3u << (config->sda.pin * 2u)));
    GPIOB->MODER   |=  (2u << (config->scl.pin * 2u))  | (2u << (config->sda.pin * 2u));
    GPIOB->OTYPER  |=  (1u << config->scl.pin) | (1u << config->sda.pin);
    GPIOB->OSPEEDR &= ~((3u << (config->scl.pin * 2u)) | (3u << (config->sda.pin * 2u)));
    GPIOB->OSPEEDR |=  (3u << (config->scl.pin * 2u))  | (3u << (config->sda.pin * 2u));
    GPIOB->PUPDR   &= ~((3u << (config->scl.pin * 2u)) | (3u << (config->sda.pin * 2u)));
    GPIOB->PUPDR   |=  (1u << (config->scl.pin * 2u))  | (1u << (config->sda.pin * 2u));
    GPIOB->AFR[1]  &= ~((0xFu << ((config->scl.pin - 8u) * 4u)) | (0xFu << ((config->sda.pin - 8u) * 4u)));
    GPIOB->AFR[1]  |=  (4u << ((config->scl.pin - 8u) * 4u))    | (4u << ((config->sda.pin - 8u) * 4u));

    /* Enable I2C1 clock, pulse reset, then configure for 400 kHz (APB1 = 16 MHz HSI). */
    RCC->APB1ENR  |=  RCC_APB1ENR_I2C1EN_Msk;
    RCC->APB1RSTR |=  RCC_APB1RSTR_I2C1RST_Msk;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST_Msk;

    I2C1->CR1   =  I2C_CR1_SWRST_Msk;
    I2C1->CR1   =  0u;
    I2C1->CR2   =  16u;                        /* FREQ = 16 MHz */
    I2C1->CCR   =  80u;                        /* standard mode, 100 kHz: 16 MHz / (2 * 100 kHz) */
    I2C1->TRISE =  17u;                        /* (16 MHz * 1000 ns) + 1 */
    I2C1->CR1   =  I2C_CR1_PE_Msk | I2C_CR1_ACK_Msk;

    return true;
}

static bool platform_target_uart_output_init(const platform_uart_config_t *config)
{
    const uint32_t tx_pin = 2u; /* PA2 = USART2_TX, AF7 */

    if ((config == NULL) || (config->uart != PLATFORM_UART_OUTPUT) || !config->tx_enabled ||
        (config->baud_rate != 115200u)) {
        return false;
    }

    /* Enable GPIOA clock and flush. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN_Msk;
    (void)RCC->AHB1ENR;

    /* PA2: AF mode (10b), push-pull, medium speed, no pull, AF7 (USART2_TX). */
    GPIOA->MODER   &= ~(3u << (tx_pin * 2u));
    GPIOA->MODER   |=  (2u << (tx_pin * 2u));
    GPIOA->OTYPER  &= ~(1u << tx_pin);
    GPIOA->OSPEEDR &= ~(3u << (tx_pin * 2u));
    GPIOA->OSPEEDR |=  (2u << (tx_pin * 2u));
    GPIOA->PUPDR   &= ~(3u << (tx_pin * 2u));
    GPIOA->AFR[0]  &= ~(0xFu << (tx_pin * 4u));
    GPIOA->AFR[0]  |=  (7u   << (tx_pin * 4u));

    /* Enable USART2 clock, pulse reset, then configure 115200 baud, TX only. */
    RCC->APB1ENR  |=  RCC_APB1ENR_USART2EN_Msk;
    RCC->APB1RSTR |=  RCC_APB1RSTR_USART2RST_Msk;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST_Msk;

    USART2->CR1 = 0u;
    USART2->CR2 = 0u;
    USART2->CR3 = 0u;
    USART2->BRR = 0x008Bu;                     /* 16 MHz / 115200 ≈ 139 = 0x8B */
    USART2->CR1 = USART_CR1_UE_Msk | USART_CR1_TE_Msk;

    return true;
}

platform_i2c_debug_t platform_i2c1_get_debug(void)
{
    return g_i2c1_debug;
}

static platform_io_status_t platform_target_uart_output_write(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    if ((length > 0u) && (data == NULL)) {
        return PLATFORM_IO_STATUS_INVALID_ARGUMENT;
    }

    for (index = 0u; index < length; ++index) {
        if (!platform_wait_for_reg_bits(&USART2->SR, USART_SR_TXE_Msk, true)) {
            return PLATFORM_IO_STATUS_NOT_READY;
        }

        USART2->DR = data[index];
    }

    return PLATFORM_IO_STATUS_OK;
}
#endif

#if !defined(EXCAVATOR_TARGET_STM32F446RE)
platform_i2c_debug_t platform_i2c1_get_debug(void)
{
    platform_i2c_debug_t debug = {0u, 0u, 0u};
    return debug;
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
           PLATFORM_READY_UART_OUTPUT |
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

        if (config->uart != PLATFORM_UART_OUTPUT) {
            continue;
        }

        if (!platform_target_uart_output_init(config)) {
            status->error_flags |= PLATFORM_ERROR_UART_CONFIG;
            continue;
        }

        status->ready_flags |= platform_uart_ready_flag(config->uart);
    }
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

platform_io_status_t platform_uart_write(platform_uart_t uart, const uint8_t *data, uint16_t length)
{
#if defined(EXCAVATOR_TARGET_STM32F446RE)
    if ((uart >= PLATFORM_UART_COUNT) || ((length > 0u) && (data == NULL))) {
        return PLATFORM_IO_STATUS_INVALID_ARGUMENT;
    }

    if ((uart != PLATFORM_UART_OUTPUT) || !platform_status_ok(g_platform_status) || !platform_uart_ready(g_platform_status, uart)) {
        return PLATFORM_IO_STATUS_NOT_READY;
    }

    return platform_target_uart_output_write(data, length);
#else
    if ((uart >= PLATFORM_UART_COUNT) || ((length > 0u) && (data == NULL))) {
        return PLATFORM_IO_STATUS_INVALID_ARGUMENT;
    }

    if (!platform_status_ok(g_platform_status) || !platform_uart_ready(g_platform_status, uart)) {
        return PLATFORM_IO_STATUS_NOT_READY;
    }

    return platform_simulated_uart_write(uart, data, length);
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

platform_io_status_t platform_simulated_uart_copy_tx(platform_uart_t uart,
                                                     uint8_t *data,
                                                     uint16_t max_length,
                                                     uint16_t *copied_length)
{
#if defined(EXCAVATOR_TARGET_STM32F446RE)
    (void)uart;
    (void)data;
    (void)max_length;
    (void)copied_length;
    return PLATFORM_IO_STATUS_NOT_IMPLEMENTED;
#else
    return platform_simulated_uart_copy_tx_impl(uart, data, max_length, copied_length);
#endif
}
