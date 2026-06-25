# =============================================================
# ZPNet Teensy 4.1 — Build + Flash (Cold Ownership)
#
# PRINCIPLE:
#   • Bootloader (HalfKay) must have cold ownership
#   • Makefile does NOT attempt to enter bootloader
#   • Flashing assumes HalfKay is already active
#
# Transport doctrine:
#   • Teensy firmware is USB CDC Serial only
#   • RawHID/HID transport selection has been retired
#
# =============================================================

# -------------------------------------------------------------
# Configuration
# -------------------------------------------------------------

NAME          = teensy
ARDUINO_CLI   = ../../bin/arduino-cli
TEENSY_LOADER = teensy_loader_cli

BOARD = teensy:avr:teensy41:usb=serial

# -------------------------------------------------------------
# Targets
# -------------------------------------------------------------

all: build

# -------------------------------------------------------------
# Build
# -------------------------------------------------------------

build:
	@echo "🔧 Building Teensy 4.1 serial-only firmware..."
	$(ARDUINO_CLI) compile \
	  --fqbn $(BOARD) \
	  . \
	  --output-dir ./build

# -------------------------------------------------------------
# Flash (HalfKay must already own the machine)
# -------------------------------------------------------------

flash: build
	@echo "🚀 Flashing Teensy 4.1 (HalfKay must already be active)..."
	sudo $(TEENSY_LOADER) \
	  -mmcu=TEENSY41 \
	  -w \
	  build/$(NAME).ino.hex
	@echo "✅ Flash complete."

# -------------------------------------------------------------
# Housekeeping
# -------------------------------------------------------------

clean:
	rm -rf build

clobber: clean
	@echo "🧹 Removing Arduino CLI build cache..."
	rm -rf ~/.arduino15/packages/*/hardware/*/*/build

.PHONY: all build flash clean clobber