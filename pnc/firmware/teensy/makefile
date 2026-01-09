# Teensy 4.1 build + flash helper
# Assumes arduino-cli lives in ../../bin/arduino-cli
# and teensy_loader_cli is installed in $PATH

NAME=teensy
BOARD=teensy:avr:teensy41
PORT=/dev/zpnet-teensy
BAUD=115200
ARDUINO_CLI=../../bin/arduino-cli

all: build

# Compile sketch into ./build directory
build:
	$(ARDUINO_CLI) compile --fqbn $(BOARD) . --output-dir ./build

# Upload using arduino-cli (requires Teensy Loader GUI running)
upload: build
	$(ARDUINO_CLI) upload -p $(PORT) --fqbn $(BOARD) .
	@echo "✅ Upload complete (arduino-cli)."

# Flash using teensy_loader_cli (headless, no button required)
flash: build
	@echo "🚀 Flashing Teensy 4.1 via HID reboot (no button)..."
	sudo teensy_loader_cli --mcu=TEENSY41 -w -s build/$(NAME).ino.hex
	@echo "✅ Teensy flashed successfully."

# Serial monitor
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
