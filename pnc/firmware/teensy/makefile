# ZPNet Teensy 4.1 -- incremental builds, explicit 600 MHz default.
#
#   make build                      Reuse compiler intermediates.
#   make build CLEAN=1              Rebuild the selected configuration.
#   make build MODE=diagnostic      Verbose compiler transcript; same flags.
#   make build JOBS=4               Override Arduino CLI compile parallelism.
#   make clean                      Remove local build outputs and caches.
#
# Use flash-teensy for the complete build/archive/bootloader/service workflow.
# Direct make flash assumes HalfKay already owns the device and the cluster
# has been stopped. It does not manage services or archive the image.

.DEFAULT_GOAL := build

NAME          := teensy
ARDUINO_CLI   ?= ../../bin/arduino-cli
TEENSY_LOADER ?= teensy_loader_cli
CPU           ?= 600
USB_MODE      := serial
BOARD         := teensy:avr:teensy41:usb=$(USB_MODE),speed=$(CPU)

MODE          ?= fast
CLEAN         ?= 0
# Empty leaves parallelism to the installed CLI; set only if it supports --jobs.
JOBS          ?=

# Separate intermediates from exported images. flash-teensy removes only the
# exports, allowing the CLI to reuse compiled objects on subsequent builds.
OUTPUT_DIR    := $(CURDIR)/build
CACHE_DIR     := $(OUTPUT_DIR)/cache/$(CPU)-$(USB_MODE)

# Preserve the supplied makefile's C++ flags and platform optimization defaults.
# Debug symbols remain available in the archived ELF for fault-address lookup.
CXX_FLAGS = -std=gnu++17 -fno-exceptions -fpermissive -fno-rtti \
            -fno-threadsafe-statics -felide-constructors -Wno-error=narrowing \
            -Wno-psabi

ifeq ($(filter $(MODE),fast diagnostic),)
$(error MODE must be fast or diagnostic)
endif
ifeq ($(filter $(CLEAN),0 1),)
$(error CLEAN must be 0 or 1)
endif
ifneq ($(strip $(PROFILE)),)
$(error PROFILE is retired; this makefile preserves the baseline compiler flags)
endif

COMPILE_OPTIONS = $(if $(filter 1,$(CLEAN)),--clean) \
                  $(if $(filter diagnostic,$(MODE)),--verbose) \
                  $(if $(strip $(JOBS)),--jobs $(JOBS))

.PHONY: all build flash flash-only clean clobber
all: build

build:
	@echo "Building Teensy 4.1: CPU=$(CPU) MHz MODE=$(MODE) CLEAN=$(CLEAN)"
	"$(ARDUINO_CLI)" compile $(COMPILE_OPTIONS) \
	  --fqbn "$(BOARD)" \
	  --build-path "$(CACHE_DIR)" \
	  --build-property "build.flags.cpp=$(CXX_FLAGS)" \
	  --output-dir "$(OUTPUT_DIR)" \
	  .

flash: build
	$(MAKE) flash-only

flash-only:
	sudo "$(TEENSY_LOADER)" -mmcu=TEENSY41 -w "$(OUTPUT_DIR)/$(NAME).ino.hex"

clean:
	rm -rf "$(OUTPUT_DIR)"

# Deliberately scoped to this firmware; do not purge other Arduino projects.
clobber: clean
