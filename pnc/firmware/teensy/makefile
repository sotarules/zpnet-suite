# =============================================================
# ZPNet Teensy 4.1 — Build + Flash (Cold Ownership)
#
# PRINCIPLE:
#   • Bootloader (HalfKay) must have cold ownership
#   • Makefile does NOT attempt to enter bootloader
#   • Flashing assumes HalfKay is already active
#
# Flashing is correct ONLY after:
#   • physical reset
#   • external RESET_B control
#   • button press
#
# =============================================================

# -------------------------------------------------------------
# Configuration
# -------------------------------------------------------------

NAME          = teensy
ARDUINO_CLI   = ../../bin/arduino-cli
TEENSY_LOADER = teensy_loader_cli

# USB CDC serial is the only supported runtime transport.
USB_MODE = serial
BOARD    = teensy:avr:teensy41:usb=$(USB_MODE)

# -------------------------------------------------------------
# Targets
# -------------------------------------------------------------

all: build

# -------------------------------------------------------------
# Build
# -------------------------------------------------------------

build:
	@echo "🔧 Building Teensy 4.1 (cold-boot compatible)..."
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