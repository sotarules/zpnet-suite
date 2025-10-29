"""
ZPNet Teensy Listener — Stellar-Compliant + Constants-Integrated Revision (v2025-10-28c)

Reads newline-delimited JSON telemetry from the Teensy (USB serial).
Each message is expected to be a valid JSON object containing an
event_type (e.g., TEENSY_STATUS) and payload fields.

Now imports serial configuration and timing constants from
zpnet.shared.constants for unified parameter control across systems.

Author: The Mule
"""

import serial
import json
import logging
import time

from zpnet.shared.logger import setup_logging
from zpnet.shared.events import create_event
from zpnet.shared.constants import (
    DB_PATH,
    TEENSY_SERIAL_PORT,
    TEENSY_BAUDRATE,
    TEENSY_RECONNECT_DELAY_S,
    TEENSY_READ_TIMEOUT_S,
)

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
    logging.info(f"🔌 [teensy_listener] started on {TEENSY_SERIAL_PORT} @ {TEENSY_BAUDRATE} baud")

    ser = None

    while True:
        # Ensure an open serial connection
        if ser is None or not ser.is_open:
            try:
                ser = serial.Serial(TEENSY_SERIAL_PORT, TEENSY_BAUDRATE, timeout=TEENSY_READ_TIMEOUT_S)
                logging.info("✅ [teensy_listener] serial connection to Teensy established")
            except serial.SerialException as e:
                logging.warning(f"🛑 [teensy_listener] could not open serial port: {e}")
                time.sleep(TEENSY_RECONNECT_DELAY_S)
                continue

        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            try:
                evt = json.loads(line)
            except json.JSONDecodeError:
                logging.warning(f"⚠️ [teensy_listener] invalid JSON from Teensy: {line}")
                continue

            event_type = evt.pop("event_type", "UNKNOWN")
            create_event(event_type, evt)
            logging.debug(f"📡 [teensy_listener] event received: {event_type}")

        except serial.SerialException as e:
            logging.warning(f"💥 [teensy_listener] serial error: {e} — reconnecting")
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            time.sleep(TEENSY_RECONNECT_DELAY_S)

        except Exception as e:
            logging.exception(f"🔥 [teensy_listener] unexpected error in listener loop: {e}")
            time.sleep(2)


# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------
def bootstrap():
    """Initialize logging and start the listener loop."""
    setup_logging()
    run()
