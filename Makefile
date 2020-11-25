BOARD_PORT?=/dev/cu.usbmodemC1
ARDUINO_HARDWARE_DIR?=~/Library/Arduino15/packages/arduino/hardware/avr/1.8.3
FQBN=pje:avr:micro_pbmw

BUILD_PROPERTIES=$(shell arduino-cli compile --fqbn arduino:avr:micro --show-properties | grep 'compiler.cpp.flags=' | sed 's/fpermissive/fno-permissive/; s/{compiler.warning_flags}/-Wall -Wextra -Wno-missing-field-initializers/; s/std=gnu++11/std=gnu++17/')

custom_hardware_definition:
	mkdir -p ~/Documents/Arduino/hardware/pje/avr/
	cp config/*.txt ~/Documents/Arduino/hardware/pje/avr/

deps: custom_hardware_definition
	which arduino-cli || brew install arduino-cli
	arduino-cli core update-index
	arduino-cli core install arduino:avr
	arduino-cli lib install USBMIDI
	arduino-cli lib install "Adafruit ADS1X15"

build:
	arduino-cli compile --fqbn $(FQBN) --verbose --build-properties "compiler.warning_flags=-Wpedantic,$(BUILD_PROPERTIES)" pbmw.ino

upload: build
	arduino-cli upload --port "$(BOARD_PORT)" --fqbn $(FQBN) --verbose pbmw.ino

clean:
	arduino-cli cache clean
	rm -rf build
	rm -rf .clangd

.PHONY: build clean deps upload custom_hardware_definition
