# =============================================================
# ZPNet Teensy 4.1 build + flash helper
#
# TRANSPORT BIFURCATION:
#   Build and runtime behavior are controlled exclusively by:
#
#     ZPNET_TRANSPORT = HID | SERIAL
#
#   This variable is REQUIRED and permanent.
#
# FLASHING RULE:
#   Teensy is ALWAYS flashed via teensy_loader_cli.
#   USB mode affects runtime behavior only.
# =============================================================

# -------------------------------------------------------------
# Required environment variable
# -------------------------------------------------------------

ifndef ZPNET_TRANSPORT
$(error ❌ ZPNET_TRANSPORT is not set. Use: export ZPNET_TRANSPORT=HID or SERIAL)
endif

ifeq ($(ZPNET_TRANSPORT),HID)
USB_MODE   = rawhid
USB_LABEL  = USB_RAWHID
HAS_SERIAL = no
else ifeq ($(ZPNET_TRANSPORT),SERIAL)
USB_MODE   = serial
USB_LABEL  = USB_CDC_SERIAL
HAS_SERIAL = yes
else
$(error ❌ Invalid ZPNET_TRANSPORT='$(ZPNET_TRANSPORT)'. Must be HID or SERIAL)
endif

# -------------------------------------------------------------
# Compile-time transport defines
# -------------------------------------------------------------

CPP_DEFS =
ifeq ($(ZPNET_TRANSPORT),HID)
CPP_DEFS += -DZPNET_TRANSPORT_HID
else ifeq ($(ZPNET_TRANSPORT),SERIAL)
CPP_DEFS += -DZPNET_TRANSPORT_SERIAL
endif

# -------------------------------------------------------------
# Toolchain
# -------------------------------------------------------------

NAME        = teensy
ARDUINO_CLI = ../../bin/arduino-cli
TEENSY_LOADER = teensy_loader_cli

BOARD = teensy:avr:teensy41:usb=$(USB_MODE)

# Serial parameters (used only if SERIAL)
PORT ?= /dev/zpnet-teensy-serial
BAUD ?= 115200

# -------------------------------------------------------------
# Targets
# -------------------------------------------------------------

all: build

# -------------------------------------------------------------
# Build
# -------------------------------------------------------------

build:
	@echo "🔧 Building Teensy 4.1 ($(USB_LABEL))..."
	@echo "    Transport mode: $(ZPNET_TRANSPORT)"
	$(ARDUINO_CLI) compile \
	  --fqbn $(BOARD) \
	  --build-property build.extra_flags="$(CPP_DEFS)" \
	  . \
	  --output-dir ./build

# -------------------------------------------------------------
# Flash (authoritative for Teensy)
# -------------------------------------------------------------

flash: build
	@echo "🚀 Flashing Teensy 4.1 via teensy_loader_cli..."
	sudo $(TEENSY_LOADER) \
	  --mcu=TEENSY41 \
	  -w -s \
	  build/$(NAME).ino.hex
	@echo "✅ Teensy flashed successfully."

# -------------------------------------------------------------
# Upload (explicitly unsupported)
# -------------------------------------------------------------

upload:
	@echo "❌ Upload via arduino-cli is unsupported on Teensy."
	@echo "   Use: make flash"

# -------------------------------------------------------------
# Serial monitor (SERIAL only)
# -------------------------------------------------------------

monitor:
ifeq ($(HAS_SERIAL),yes)
	@echo "📡 Opening serial monitor on $(PORT)..."
	$(ARDUINO_CLI) monitor \
	  --port $(PORT) \
	  --baud $(BAUD)
else
	@echo "❌ Serial monitor unavailable (HID transport)."
endif

# -------------------------------------------------------------
# Info / utilities
# -------------------------------------------------------------

info:
	@echo "ℹ️  Transport mode: $(ZPNET_TRANSPORT)"
	@echo "ℹ️  USB mode:       $(USB_MODE)"
	@echo "ℹ️  Board FQBN:     $(BOARD)"
	@echo
	$(ARDUINO_CLI) board list

clean:
	rm -rf build

clobber: clean
	@echo "🧹 Removing Arduino CLI global build cache..."
	rm -rf ~/.arduino15/packages/*/hardware/*/*/build

.PHONY: all build flash upload monitor info clean clobber