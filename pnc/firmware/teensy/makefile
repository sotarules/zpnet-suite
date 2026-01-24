# =============================================================
# ZPNet Teensy 4.1 build + flash helper
#
# ARCHITECTURAL NOTE:
#   This firmware uses a SINGLE RawHID interface.
#
#   • Application traffic and debug output are SOFTWARE-MULTIPLEXED
#     over one RawHID endpoint.
#   • No CDC / Serial interface is present.
#   • USB Type MUST be RawHID.
#
#   Debug framing:
#     pkt[0] = channel selector (e.g. 0xD0 = debug, 0x01 = app)
#
# =============================================================

NAME        = teensy
BOARD       = teensy:avr:teensy41:usb=rawhid
ARDUINO_CLI = ../../bin/arduino-cli

# -------------------------------------------------------------
# USB configuration (FIXED)
# -------------------------------------------------------------

# RawHID only. No CDC, no composite.
USB_MODE = hid

# No serial port exists in RawHID-only mode
PORT =
BAUD = 115200

# -------------------------------------------------------------
# Targets
# -------------------------------------------------------------

all: build

# -------------------------------------------------------------
# Build
# -------------------------------------------------------------

build:
	@echo "🔧 Building Teensy (USB_RAWHID, single interface, multiplexed)..."
	$(ARDUINO_CLI) compile \
	  --fqbn $(BOARD) \
	  . \
	  --output-dir ./build

# -------------------------------------------------------------
# Flash (HID-based, no button required)
# -------------------------------------------------------------

flash: build
	@echo "🚀 Flashing Teensy 4.1 via HID reboot (no button)..."
	sudo teensy_loader_cli \
	  --mcu=TEENSY41 \
	  -w -s \
	  build/$(NAME).ino.hex
	@echo "✅ Teensy flashed successfully."

# -------------------------------------------------------------
# Upload (CDC) — intentionally unavailable
# -------------------------------------------------------------

upload:
	@echo "❌ Upload via CDC is unavailable."
	@echo "   This firmware uses RawHID only."
	@echo "   Use: make flash"

# -------------------------------------------------------------
# Serial monitor — intentionally unavailable
# -------------------------------------------------------------

monitor:
	@echo "❌ Serial monitor unavailable (RawHID-only firmware)."

# -------------------------------------------------------------
# Info / utilities
# -------------------------------------------------------------

info:
	@echo "ℹ️  Listing connected boards..."
	$(ARDUINO_CLI) board list

clean:
	rm -rf build

clobber: clean
	@echo "🧹 Removing Arduino CLI global build cache..."
	rm -rf ~/.arduino15/packages/*/hardware/*/*/build

.PHONY: all build flash upload monitor info clean clobber
