#include "platform_simulated_io.h"

#include <stddef.h>

typedef struct {
    bool present;
    uint8_t address;
    uint8_t registers[256];
} platform_simulated_i2c_device_t;

typedef struct {
    uint8_t buffer[256];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} platform_simulated_uart_fifo_t;

enum {
    PLATFORM_SIM_BMI160_ADDRESS = 0x68u,
    PLATFORM_SIM_BMI160_REG_CHIP_ID = 0x00u,
    PLATFORM_SIM_BMI160_CHIP_ID = 0xD1u,
    PLATFORM_SIM_BMI160_REG_GYRO_DATA = 0x0Cu,
    PLATFORM_SIM_BMI160_REG_ACCEL_DATA = 0x12u,
    PLATFORM_SIM_BMI160_RAW_1G = 16384u,
};

static platform_simulated_i2c_device_t g_simulated_i2c_devices[PLATFORM_I2C_COUNT];
static platform_simulated_uart_fifo_t g_simulated_uart_fifos[PLATFORM_UART_COUNT];

static bool platform_simulated_valid_i2c_transaction(platform_i2c_bus_t bus,
                                                     const uint8_t *write_data,
                                                     uint8_t *read_data,
                                                     uint16_t length)
{
    if (bus >= PLATFORM_I2C_COUNT) {
        return false;
    }

    if ((length > 0u) && (write_data == NULL) && (read_data == NULL)) {
        return false;
    }

    return true;
}

void platform_simulated_reset(void)
{
    size_t index;

    for (index = 0; index < PLATFORM_I2C_COUNT; ++index) {
        platform_simulated_i2c_device_t *device = &g_simulated_i2c_devices[index];
        size_t reg_index;

        device->present = true;
        device->address = PLATFORM_SIM_BMI160_ADDRESS;

        for (reg_index = 0; reg_index < sizeof(device->registers); ++reg_index) {
            device->registers[reg_index] = 0u;
        }

        /* Each simulated bus starts with one BMI160-like device for host tests. */
        device->registers[PLATFORM_SIM_BMI160_REG_CHIP_ID] = PLATFORM_SIM_BMI160_CHIP_ID;
        /*
         * Default the simulated IMU to a level, quasistatic pose:
         * zero gyro and +1 g on the Z accelerometer axis.
         */
        device->registers[PLATFORM_SIM_BMI160_REG_GYRO_DATA + 0u] = 0u;
        device->registers[PLATFORM_SIM_BMI160_REG_GYRO_DATA + 1u] = 0u;
        device->registers[PLATFORM_SIM_BMI160_REG_GYRO_DATA + 2u] = 0u;
        device->registers[PLATFORM_SIM_BMI160_REG_GYRO_DATA + 3u] = 0u;
        device->registers[PLATFORM_SIM_BMI160_REG_GYRO_DATA + 4u] = 0u;
        device->registers[PLATFORM_SIM_BMI160_REG_GYRO_DATA + 5u] = 0u;
        device->registers[PLATFORM_SIM_BMI160_REG_ACCEL_DATA + 0u] = 0u;
        device->registers[PLATFORM_SIM_BMI160_REG_ACCEL_DATA + 1u] = 0u;
        device->registers[PLATFORM_SIM_BMI160_REG_ACCEL_DATA + 2u] = 0u;
        device->registers[PLATFORM_SIM_BMI160_REG_ACCEL_DATA + 3u] = 0u;
        device->registers[PLATFORM_SIM_BMI160_REG_ACCEL_DATA + 4u] = (uint8_t)(PLATFORM_SIM_BMI160_RAW_1G & 0xFFu);
        device->registers[PLATFORM_SIM_BMI160_REG_ACCEL_DATA + 5u] =
            (uint8_t)(PLATFORM_SIM_BMI160_RAW_1G >> 8);
    }

    for (index = 0; index < PLATFORM_UART_COUNT; ++index) {
        g_simulated_uart_fifos[index].head = 0u;
        g_simulated_uart_fifos[index].tail = 0u;
        g_simulated_uart_fifos[index].count = 0u;
    }
}

platform_io_status_t platform_simulated_i2c_write_registers(platform_i2c_bus_t bus,
                                                            uint8_t device_address,
                                                            uint8_t register_address,
                                                            const uint8_t *data,
                                                            uint16_t length)
{
    platform_simulated_i2c_device_t *device;
    uint16_t index;

    if (!platform_simulated_valid_i2c_transaction(bus, data, NULL, length)) {
        return PLATFORM_IO_STATUS_INVALID_ARGUMENT;
    }

    device = &g_simulated_i2c_devices[bus];
    if (!device->present || (device->address != device_address)) {
        return PLATFORM_IO_STATUS_NOT_FOUND;
    }

    for (index = 0; index < length; ++index) {
        device->registers[(uint8_t)(register_address + index)] = data[index];
    }

    return PLATFORM_IO_STATUS_OK;
}

platform_io_status_t platform_simulated_i2c_read_registers(platform_i2c_bus_t bus,
                                                           uint8_t device_address,
                                                           uint8_t register_address,
                                                           uint8_t *data,
                                                           uint16_t length)
{
    const platform_simulated_i2c_device_t *device;
    uint16_t index;

    if (!platform_simulated_valid_i2c_transaction(bus, NULL, data, length)) {
        return PLATFORM_IO_STATUS_INVALID_ARGUMENT;
    }

    device = &g_simulated_i2c_devices[bus];
    if (!device->present || (device->address != device_address)) {
        return PLATFORM_IO_STATUS_NOT_FOUND;
    }

    for (index = 0; index < length; ++index) {
        data[index] = device->registers[(uint8_t)(register_address + index)];
    }

    return PLATFORM_IO_STATUS_OK;
}

platform_io_status_t platform_simulated_uart_read_byte(platform_uart_t uart, uint8_t *byte_out)
{
    platform_simulated_uart_fifo_t *fifo;

    if ((uart >= PLATFORM_UART_COUNT) || (byte_out == NULL)) {
        return PLATFORM_IO_STATUS_INVALID_ARGUMENT;
    }

    fifo = &g_simulated_uart_fifos[uart];
    if (fifo->count == 0u) {
        return PLATFORM_IO_STATUS_EMPTY;
    }

    *byte_out = fifo->buffer[fifo->tail];
    fifo->tail = (uint16_t)((fifo->tail + 1u) % (uint16_t)sizeof(fifo->buffer));
    fifo->count--;
    return PLATFORM_IO_STATUS_OK;
}

bool platform_simulated_i2c_set_registers_impl(platform_i2c_bus_t bus,
                                               uint8_t device_address,
                                               uint8_t register_address,
                                               const uint8_t *data,
                                               uint16_t length)
{
    return platform_simulated_i2c_write_registers(bus, device_address, register_address, data, length) ==
           PLATFORM_IO_STATUS_OK;
}

bool platform_simulated_i2c_get_registers_impl(platform_i2c_bus_t bus,
                                               uint8_t device_address,
                                               uint8_t register_address,
                                               uint8_t *data,
                                               uint16_t length)
{
    return platform_simulated_i2c_read_registers(bus, device_address, register_address, data, length) ==
           PLATFORM_IO_STATUS_OK;
}

platform_io_status_t platform_simulated_uart_feed_impl(platform_uart_t uart,
                                                       const uint8_t *data,
                                                       uint16_t length)
{
    platform_simulated_uart_fifo_t *fifo;
    uint16_t index;

    if ((uart >= PLATFORM_UART_COUNT) || ((length > 0u) && (data == NULL))) {
        return PLATFORM_IO_STATUS_INVALID_ARGUMENT;
    }

    fifo = &g_simulated_uart_fifos[uart];
    for (index = 0; index < length; ++index) {
        if (fifo->count >= sizeof(fifo->buffer)) {
            return PLATFORM_IO_STATUS_OVERFLOW;
        }

        fifo->buffer[fifo->head] = data[index];
        fifo->head = (uint16_t)((fifo->head + 1u) % (uint16_t)sizeof(fifo->buffer));
        fifo->count++;
    }

    return PLATFORM_IO_STATUS_OK;
}
