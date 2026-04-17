#ifndef EXCAVATOR_PLATFORM_SIMULATED_IO_H
#define EXCAVATOR_PLATFORM_SIMULATED_IO_H

#include "platform.h"

#include <stdbool.h>
#include <stdint.h>

void platform_simulated_reset(void);
platform_io_status_t platform_simulated_i2c_write_registers(platform_i2c_bus_t bus,
                                                            uint8_t device_address,
                                                            uint8_t register_address,
                                                            const uint8_t *data,
                                                            uint16_t length);
platform_io_status_t platform_simulated_i2c_read_registers(platform_i2c_bus_t bus,
                                                           uint8_t device_address,
                                                           uint8_t register_address,
                                                           uint8_t *data,
                                                           uint16_t length);
platform_io_status_t platform_simulated_uart_read_byte(platform_uart_t uart, uint8_t *byte_out);
platform_io_status_t platform_simulated_uart_write(platform_uart_t uart, const uint8_t *data, uint16_t length);
bool platform_simulated_i2c_set_registers_impl(platform_i2c_bus_t bus,
                                               uint8_t device_address,
                                               uint8_t register_address,
                                               const uint8_t *data,
                                               uint16_t length);
bool platform_simulated_i2c_get_registers_impl(platform_i2c_bus_t bus,
                                               uint8_t device_address,
                                               uint8_t register_address,
                                               uint8_t *data,
                                               uint16_t length);
platform_io_status_t platform_simulated_uart_feed_impl(platform_uart_t uart,
                                                       const uint8_t *data,
                                                       uint16_t length);
platform_io_status_t platform_simulated_uart_copy_tx_impl(platform_uart_t uart,
                                                          uint8_t *data,
                                                          uint16_t max_length,
                                                          uint16_t *copied_length);

#endif
