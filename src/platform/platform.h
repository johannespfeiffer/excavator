#ifndef EXCAVATOR_PLATFORM_H
#define EXCAVATOR_PLATFORM_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    PLATFORM_BACKEND_SIMULATED = 0,
    PLATFORM_BACKEND_STM32F446RE = 1,
} platform_backend_t;

typedef enum {
    PLATFORM_I2C1 = 0,
    PLATFORM_I2C2 = 1,
    PLATFORM_I2C3 = 2,
    PLATFORM_FMPI2C1 = 3,
    PLATFORM_I2C_COUNT = 4,
} platform_i2c_bus_t;

typedef enum {
    PLATFORM_GPIO_PORT_INVALID = 0,
    PLATFORM_GPIO_PORT_A,
    PLATFORM_GPIO_PORT_B,
    PLATFORM_GPIO_PORT_C,
    PLATFORM_GPIO_PORT_D,
    PLATFORM_GPIO_PORT_E,
    PLATFORM_GPIO_PORT_F,
    PLATFORM_GPIO_PORT_G,
    PLATFORM_GPIO_PORT_H,
} platform_gpio_port_t;

typedef enum {
    PLATFORM_UART_GPS = 0,
    PLATFORM_UART_OUTPUT = 1,
    PLATFORM_UART_COUNT = 2,
} platform_uart_t;

typedef enum {
    PLATFORM_READY_CLOCK = 1u << 0,
    PLATFORM_READY_GPIO = 1u << 1,
    PLATFORM_READY_UART_GPS = 1u << 2,
    PLATFORM_READY_UART_OUTPUT = 1u << 3,
    PLATFORM_READY_I2C1 = 1u << 4,
    PLATFORM_READY_I2C2 = 1u << 5,
    PLATFORM_READY_I2C3 = 1u << 6,
    PLATFORM_READY_FMPI2C1 = 1u << 7,
    PLATFORM_READY_SELF_TEST = 1u << 8,
} platform_ready_flag_t;

typedef enum {
    PLATFORM_ERROR_NONE = 0,
    PLATFORM_ERROR_CLOCK = 1u << 0,
    PLATFORM_ERROR_GPIO = 1u << 1,
    PLATFORM_ERROR_UART_CONFIG = 1u << 2,
    PLATFORM_ERROR_I2C_CONFIG = 1u << 3,
    PLATFORM_ERROR_SELF_TEST = 1u << 4,
    PLATFORM_ERROR_NOT_IMPLEMENTED = 1u << 5,
} platform_error_flag_t;

typedef enum {
    PLATFORM_IO_STATUS_OK = 0,
    PLATFORM_IO_STATUS_INVALID_ARGUMENT = 1,
    PLATFORM_IO_STATUS_NOT_READY = 2,
    PLATFORM_IO_STATUS_NOT_FOUND = 3,
    PLATFORM_IO_STATUS_NOT_IMPLEMENTED = 4,
    PLATFORM_IO_STATUS_EMPTY = 5,
    PLATFORM_IO_STATUS_OVERFLOW = 6,
} platform_io_status_t;

typedef struct {
    platform_backend_t backend;
    uint32_t ready_flags;
    uint32_t error_flags;
} platform_status_t;

typedef struct {
    platform_gpio_port_t port;
    uint8_t pin;
} platform_gpio_pin_t;

typedef struct {
    platform_i2c_bus_t bus;
    platform_gpio_pin_t scl;
    platform_gpio_pin_t sda;
    uint32_t bitrate_hz;
} platform_i2c_pinout_t;

/*
 * I2C debug snapshot populated on the target after every failed operation.
 * step: 0=ok 1=busy 2=sb 3=nack(AF) 4=addr_timeout 5=txe 6=btf 7=rxne 8=btf_read
 * sr1:  raw I2C1->SR1 at failure; 0x0400=AF(NACK), 0x0200=BERR, 0x0000=silent
 */
typedef struct {
    uint32_t step;
    uint32_t sr1;
    uint32_t sr2;
} platform_i2c_debug_t;

typedef struct {
    uint32_t scl_level;
    uint32_t sda_level;
    uint32_t moder;
    uint32_t otyper;
    uint32_t pupdr;
    uint32_t afr1;
} platform_i2c1_gpio_debug_t;

platform_i2c_debug_t platform_i2c1_get_debug(void);
platform_i2c1_gpio_debug_t platform_i2c1_get_gpio_debug(void);

platform_status_t platform_init(void);
platform_backend_t platform_backend(platform_status_t status);
bool platform_status_ok(platform_status_t status);
bool platform_uart_ready(platform_status_t status, platform_uart_t uart);
bool platform_i2c_ready(platform_status_t status, platform_i2c_bus_t bus);
bool platform_i2c_pinout(platform_i2c_bus_t bus, platform_i2c_pinout_t *pinout);
platform_io_status_t platform_i2c_probe_address(platform_i2c_bus_t bus, uint8_t device_address);
platform_io_status_t platform_i2c_write_registers(platform_i2c_bus_t bus,
                                                  uint8_t device_address,
                                                  uint8_t register_address,
                                                  const uint8_t *data,
                                                  uint16_t length);
platform_io_status_t platform_i2c_read_registers(platform_i2c_bus_t bus,
                                                 uint8_t device_address,
                                                 uint8_t register_address,
                                                 uint8_t *data,
                                                 uint16_t length);
platform_io_status_t platform_uart_read_byte(platform_uart_t uart, uint8_t *byte_out);
platform_io_status_t platform_uart_write(platform_uart_t uart, const uint8_t *data, uint16_t length);
bool platform_simulated_i2c_set_registers(platform_i2c_bus_t bus,
                                          uint8_t device_address,
                                          uint8_t register_address,
                                          const uint8_t *data,
                                          uint16_t length);
bool platform_simulated_i2c_get_registers(platform_i2c_bus_t bus,
                                          uint8_t device_address,
                                          uint8_t register_address,
                                          uint8_t *data,
                                          uint16_t length);
platform_io_status_t platform_simulated_uart_feed(platform_uart_t uart,
                                                  const uint8_t *data,
                                                  uint16_t length);
platform_io_status_t platform_simulated_uart_copy_tx(platform_uart_t uart,
                                                     uint8_t *data,
                                                     uint16_t max_length,
                                                     uint16_t *copied_length);

#endif
