BUILD_DIR ?= build
BUILD_TYPE ?= Debug
GENERATOR ?= Unix Makefiles
ARM_GCC ?= arm-none-eabi-gcc

.PHONY: configure build clean flash debug size test check-toolchain check-openocd

all: build

configure: check-toolchain
	cmake -S . -B $(BUILD_DIR) -G "$(GENERATOR)" -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) -DCMAKE_TOOLCHAIN_FILE=cmake/arm-gcc-toolchain.cmake -DEXCAVATOR_ENABLE_TESTS=OFF

build: configure
	cmake --build $(BUILD_DIR)

clean:
	cmake -E rm -rf $(BUILD_DIR)

flash: build check-openocd
	openocd -f openocd/stm32f446re.cfg -c "program $(BUILD_DIR)/excavator_firmware.elf verify reset exit"

debug: build check-openocd
	openocd -f openocd/stm32f446re.cfg

size: build
	arm-none-eabi-size $(BUILD_DIR)/excavator_firmware.elf

test:
	cmake -S . -B $(BUILD_DIR)-host -G "$(GENERATOR)" -DEXCAVATOR_ENABLE_TARGET=OFF -DEXCAVATOR_ENABLE_TESTS=ON
	cmake --build $(BUILD_DIR)-host
	ctest --test-dir $(BUILD_DIR)-host --output-on-failure

check-toolchain:
	@command -v cmake >/dev/null 2>&1 || { \
		echo "cmake is not installed or not in PATH."; \
		echo "Install CMake, then retry \`make\`."; \
		exit 127; \
	}
	@command -v $(ARM_GCC) >/dev/null 2>&1 || { \
		echo "$(ARM_GCC) is not installed or not in PATH."; \
		echo "Install the ARM GCC toolchain to build firmware, or run \`make test\` for the host-side test suite."; \
		exit 127; \
	}

check-openocd:
	@command -v openocd >/dev/null 2>&1 || { \
		echo "openocd is not installed or not in PATH."; \
		echo "Install OpenOCD to use \`make flash\` or \`make debug\`."; \
		exit 127; \
	}
