"""
ZPNet Shared Serial Utilities

Provides minimal, explicit helpers for sending commands to the Teensy
over USB CDC serial.  This module owns *transport only* — not semantics.

Author: The Mule + GPT
"""

import json
import logging
import serial
import time

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

TEENSY_SERIAL_DEV = "/dev/ttyACM0"
TEENSY_BAUD = 115200
SERIAL_TIMEOUT_S = 1.0


# ---------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------

def send_teensy_command(command: dict) -> None:
    """
    Send a single JSON command to the Teensy.

    Args:
        command (dict): JSON-serializable command object.

    Raises:
        Exception: on serial open or write failure.
    """
    payload = json.dumps(command, separators=(",", ":")) + "\n"

    logging.debug(
        "[serial] sending to Teensy: %s",
        payload.strip()
    )

    with serial.Serial(
        TEENSY_SERIAL_DEV,
        TEENSY_BAUD,
        timeout=SERIAL_TIMEOUT_S,
        write_timeout=SERIAL_TIMEOUT_S,
    ) as ser:
        # Small delay ensures CDC is ready after open
        time.sleep(0.05)
        ser.write(payload.encode("utf-8"))
        ser.flush()
