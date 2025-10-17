"""
ZPNet Teensy Listener  —  Stellar-Compliant Revision

Reads newline-delimited JSON telemetry from the Teensy (USB serial).
Each message is expected to be a valid JSON object containing an
event_type (e.g., TEENSY_STATUS) and payload fields.

All higher-level interpretation (e.g., health inference, aggregation)
is handled by the Aggregator.

Author: The Mule
"""

import os
import serial
import json
import logging
import time
from zpnet.shared.logger import setup_logging
from zpnet.shared.events import create_event

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------
SERIAL_PORT = os.environ.get("ZPNET_TEENSY_PORT", "/dev/ttyACM0")
BAUDRATE = 115200
RECONNECT_DELAY_S = 5     # seconds between reconnection attempts
READ_TIMEOUT_S = 1        # serial read timeout


# ---------------------------------------------------------------------
# Core Routine
# ---------------------------------------------------------------------
def run():
    """
    Continuously read serial lines, parse as JSON, and insert into
    the zpnet_events table via create_event().

    Emits:
        TEENSY_STATUS, ZPNET_BOOT, or any valid event emitted by the Teensy.
    """
    logging.info(f"🔌 ZPNet Teensy Listener started on {SERIAL_PORT} @ {BAUDRATE} baud")

    ser = None

    while True:
        # Ensure an open serial connection
        if ser is None or not ser.is_open:
            try:
                ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=READ_TIMEOUT_S)
                logging.info("✅ Serial connection to Teensy established")
            except serial.SerialException as e:
                logging.warning(f"🛑 Could not open serial port: {e}")
                time.sleep(RECONNECT_DELAY_S)
                continue

        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            try:
                evt = json.loads(line)
            except json.JSONDecodeError:
                logging.warning(f"⚠️ Invalid JSON from Teensy: {line}")
                continue

            event_type = evt.pop("event_type", "UNKNOWN")
            create_event(event_type, evt)
            logging.debug(f"📡 Event received: {event_type}")

        except serial.SerialException as e:
            logging.warning(f"💥 Serial error: {e} — reconnecting")
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            time.sleep(RECONNECT_DELAY_S)

        except Exception as e:
            logging.exception(f"🔥 Unexpected error in listener loop: {e}")
            time.sleep(2)


# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------
def bootstrap():
    """Initialize logging and start the listener loop."""
    setup_logging()
    run()
