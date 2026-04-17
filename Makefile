BUILD_DIR ?= build
BUILD_TYPE ?= Debug
GENERATOR ?= Unix Makefiles

.PHONY: configure build clean flash debug size test

configure:
	cmake -S . -B $(BUILD_DIR) -G "$(GENERATOR)" -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) -DCMAKE_TOOLCHAIN_FILE=cmake/arm-gcc-toolchain.cmake

build: configure
	cmake --build $(BUILD_DIR)

clean:
	cmake -E rm -rf $(BUILD_DIR)

flash: build
	openocd -f openocd/stm32f446re.cfg -c "program $(BUILD_DIR)/excavator_firmware.elf verify reset exit"

debug: build
	openocd -f openocd/stm32f446re.cfg

size: build
	arm-none-eabi-size $(BUILD_DIR)/excavator_firmware.elf

test:
	cmake -S . -B $(BUILD_DIR)-host -G "$(GENERATOR)" -DEXCAVATOR_ENABLE_TARGET=OFF -DEXCAVATOR_ENABLE_TESTS=ON
	cmake --build $(BUILD_DIR)-host
	ctest --test-dir $(BUILD_DIR)-host --output-on-failure
