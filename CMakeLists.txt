cmake_minimum_required(VERSION 3.21)

project(excavator_firmware C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(EXCAVATOR_ENABLE_TARGET "Build the STM32 target firmware" ON)
option(EXCAVATOR_ENABLE_TESTS "Build host-side unit tests" ON)

set(EXCAVATOR_COMMON_SOURCES
    src/app/main.c
    src/app/app.c
    src/app/telemetry.c
    src/config/config.c
    src/drivers/bmi160/bmi160.c
    src/drivers/gps/gps_parser.c
    src/math/kinematics.c
    src/math/orientation.c
    src/model/calibration.c
    src/model/excavator_state.c
    src/platform/platform.c
    src/platform/platform_simulated_io.c
)

set(EXCAVATOR_COMMON_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/app
    ${CMAKE_CURRENT_SOURCE_DIR}/src/config
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/bmi160
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/gps
    ${CMAKE_CURRENT_SOURCE_DIR}/src/math
    ${CMAKE_CURRENT_SOURCE_DIR}/src/model
    ${CMAKE_CURRENT_SOURCE_DIR}/src/platform
)

add_library(excavator_core STATIC ${EXCAVATOR_COMMON_SOURCES})
target_include_directories(excavator_core PUBLIC ${EXCAVATOR_COMMON_INCLUDES})
target_compile_options(excavator_core PRIVATE -Wall -Wextra -Wpedantic)
if(NOT MSVC)
    target_link_libraries(excavator_core PUBLIC m)
endif()

if(EXCAVATOR_ENABLE_TARGET)
    if(CMAKE_C_COMPILER_ID MATCHES "GNU")
        target_compile_options(excavator_core PRIVATE
            -mcpu=cortex-m4
            -mthumb
            -ffunction-sections
            -fdata-sections
            -ffreestanding
        )
    endif()
    target_compile_definitions(excavator_core PRIVATE EXCAVATOR_TARGET_STM32F446RE=1)

    add_executable(excavator_firmware.elf
        src/platform/startup_placeholder.c
    )
    target_link_libraries(excavator_firmware.elf PRIVATE excavator_core)
    target_compile_definitions(excavator_firmware.elf PRIVATE EXCAVATOR_TARGET_STM32F446RE=1)

    if(CMAKE_C_COMPILER_ID MATCHES "GNU")
        target_compile_options(excavator_firmware.elf PRIVATE
            -mcpu=cortex-m4
            -mthumb
            -ffunction-sections
            -fdata-sections
            -ffreestanding
        )
        target_link_options(excavator_firmware.elf PRIVATE
            -T${CMAKE_CURRENT_SOURCE_DIR}/linker/stm32f446re.ld
            -Wl,--gc-sections
            -Wl,-Map,${CMAKE_CURRENT_BINARY_DIR}/excavator_firmware.map
            -mcpu=cortex-m4
            -mthumb
        )
    endif()

    add_custom_command(
        TARGET excavator_firmware.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary
                $<TARGET_FILE:excavator_firmware.elf>
                ${CMAKE_CURRENT_BINARY_DIR}/excavator_firmware.bin
        COMMAND ${CMAKE_OBJCOPY} -O ihex
                $<TARGET_FILE:excavator_firmware.elf>
                ${CMAKE_CURRENT_BINARY_DIR}/excavator_firmware.hex
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:excavator_firmware.elf>
        VERBATIM
        COMMENT "Generating firmware artifacts for excavator_firmware.elf"
    )
endif()

if(EXCAVATOR_ENABLE_TESTS)
    include(CTest)
    enable_testing()
    add_subdirectory(tests)
endif()
