# ================================================================
# Teensy 4.1 build + flash helper
# ================================================================
# Assumes arduino-cli lives in ../../bin/arduino-cli
# and teensy_loader_cli is installed in $PATH
#
# Warnings control:
#   WARNINGS=off   → suppress all compiler warnings (default)
#   WARNINGS=on    → show full compiler warnings
#
# USB configuration:
#   Dual USB Serial (ZPNet + Debug)
#
# Example:
#   make flash
#   make flash WARNINGS=on
# ================================================================

# ------------------------------------------------
# ccache (speed)
# ------------------------------------------------
export CCACHE_CPP2=yes
export PATH := /usr/lib/ccache:$(PATH)
# ------------------------------------------------

NAME=teensy

# Base board
BOARD_BASE=teensy:avr:teensy41

# USB type: Dual Serial
USB_TYPE=usb=serial2

# Full board spec (authoritative)
BOARD=$(BOARD_BASE):$(USB_TYPE)

PORT=/dev/zpnet-teensy
BAUD=115200
ARDUINO_CLI=../../bin/arduino-cli

# ------------------------------------------------
# Warning control
# ------------------------------------------------

WARNINGS ?= off

ifeq ($(WARNINGS),on)
  WARN_FLAGS =
else
  WARN_FLAGS = -w
endif

BUILD_FLAGS = \
  --build-property compiler.cpp.extra_flags="$(WARN_FLAGS)" \
  --build-property compiler.c.extra_flags="$(WARN_FLAGS)"

# ------------------------------------------------
# Targets
# ------------------------------------------------

all: build

# Compile sketch into ./build directory
build:
	$(ARDUINO_CLI) compile \
	  --fqbn $(BOARD) \
	  $(BUILD_FLAGS) \
	  . \
	  --output-dir ./build

# Upload using arduino-cli (requires Teensy Loader GUI running)
upload: build
	$(ARDUINO_CLI) upload -p $(PORT) --fqbn $(BOARD) .
	@echo "✅ Upload complete (arduino-cli)."

# Flash using teensy_loader_cli (headless, no button required)
flash: build
	@echo "🚀 Flashing Teensy 4.1 via HID reboot (no button)..."
	sudo teensy_loader_cli --mcu=TEENSY41 -w -s build/$(NAME).ino.hex
	@echo "✅ Teensy flashed successfully."

# Serial monitor (ZPNet channel)
monitor:
	@echo "🔍 Opening serial monitor on $(PORT) at $(BAUD). Press Ctrl-A then K to quit (screen)."
	screen $(PORT) $(BAUD)

# List connected boards
info:
	@echo "ℹ️  Listing connected boards..."
	$(ARDUINO_CLI) board list

# Clean local build artifacts
clean:
	rm -rf build

# Clean local + Arduino CLI global build cache
clobber: clean
	@echo "🧹 Removing Arduino CLI global build cache..."
	rm -rf ~/.arduino15/packages/*/hardware/*/*/build

.PHONY: all build upload flash monitor info clean clobber
