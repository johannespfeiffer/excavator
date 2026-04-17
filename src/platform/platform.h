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
} platform_io_status_t;

typedef struct {
    platform_backend_t backend;
    uint32_t ready_flags;
    uint32_t error_flags;
} platform_status_t;

platform_status_t platform_init(void);
platform_backend_t platform_backend(platform_status_t status);
bool platform_status_ok(platform_status_t status);
bool platform_uart_ready(platform_status_t status, platform_uart_t uart);
bool platform_i2c_ready(platform_status_t status, platform_i2c_bus_t bus);
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

#endif
