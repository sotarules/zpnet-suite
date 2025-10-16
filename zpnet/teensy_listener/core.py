"""
ZPNet Teensy Listener

Reads newline-delimited JSON events from the Teensy via USB serial.
Emits structured events into the telemetry stream using create_event().
"""

import os
import serial
import json
import time
import logging
from zpnet.shared.events import create_event
from zpnet.shared.logger import setup_logging

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------
SERIAL_PORT = os.environ.get("ZPNET_TEENSY_PORT", "/dev/ttyACM0")
BAUDRATE = 115200
HEARTBEAT_TIMEOUT = 120  # seconds without heartbeat before warning

# ---------------------------------------------------------------------
# Main listener
# ---------------------------------------------------------------------
def run():
    logging.info(f"🔌 ZPNet Teensy Listener started on {SERIAL_PORT} @ {BAUDRATE} baud")

    ser = None
    last_heartbeat = time.time()
    teensy_lost = False  # flag to prevent repeated SYSTEM_ERROR spam

    while True:
        # -------------------------------------------------------------
        # Establish / re-establish serial connection
        # -------------------------------------------------------------
        if ser is None or not ser.is_open:
            try:
                ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
                logging.info("✅ Serial connection to Teensy established")
            except serial.SerialException as e:
                logging.warning(f"🛑 Could not open serial port: {e}")
                time.sleep(5)
                continue

        # -------------------------------------------------------------
        # Read and process lines from Teensy
        # -------------------------------------------------------------
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()

            if not line:
                # check for heartbeat timeout
                if time.time() - last_heartbeat > HEARTBEAT_TIMEOUT:
                    if not teensy_lost:
                        create_event("SYSTEM_ERROR", {
                            "component": "teensy",
                            "message": "Heartbeat timeout"
                        })
                        logging.warning("⏱️ No heartbeat received — SYSTEM_ERROR emitted")
                        teensy_lost = True
                continue

            # ---------------------------------------------------------
            # Attempt to parse incoming JSON
            # ---------------------------------------------------------
            try:
                evt = json.loads(line)
            except json.JSONDecodeError:
                logging.warning(f"⚠️ Bad JSON from Teensy: {line}")
                continue

            event_type = evt.pop("event_type", "UNKNOWN")
            create_event(event_type, evt)

            # ---------------------------------------------------------
            # Handle heartbeat and reconnection logic
            # ---------------------------------------------------------
            if event_type == "HEARTBEAT":
                last_heartbeat = time.time()
                if teensy_lost:
                    create_event("TEENSY_RECONNECTED", {"millis": evt.get("millis", 0)})
                    logging.info("💚 Teensy heartbeat restored — TEENSY_RECONNECTED emitted")
                    teensy_lost = False

        # -------------------------------------------------------------
        # Error handling and recovery
        # -------------------------------------------------------------
        except serial.SerialException as e:
            logging.error(f"💥 Serial error: {e}")
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            time.sleep(3)

        except Exception as e:
            logging.exception(f"🔥 Unexpected error in listener loop: {e}")
            time.sleep(2)

# ---------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------
def bootstrap():
    setup_logging()
    run()
