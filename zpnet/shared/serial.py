"""
ZPNet Shared Serial Utilities

Provides minimal, explicit helpers for sending commands to the Teensy
over USB CDC serial.

This module owns:
  • Transport (opening / writing serial)
  • Canonical command construction
  • NO semantics, NO scheduling, NO persistence

Author: The Mule + GPT
"""

import json
import logging
import serial
import time

from zpnet.shared.constants import (
    TEENSY_SERIAL_PORT,
    TEENSY_BAUDRATE,
    TEENSY_READ_TIMEOUT_S,
)

# ---------------------------------------------------------------------
# Transport
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

    logging.debug("[serial] → Teensy: %s", payload.strip())

    with serial.Serial(
        TEENSY_SERIAL_PORT,
        TEENSY_BAUDRATE,
        timeout=TEENSY_READ_TIMEOUT_S,
        write_timeout=TEENSY_READ_TIMEOUT_S,
    ) as ser:
        # Small delay ensures CDC is ready after open
        time.sleep(0.05)
        ser.write(payload.encode("utf-8"))
        ser.flush()


# ---------------------------------------------------------------------
# Canonical command builders
# ---------------------------------------------------------------------

# ===== Event-generating commands (routine monitors) =====

def cmd_teensy_status() -> dict:
    """Request Teensy to enqueue TEENSY_STATUS event."""
    return {"cmd": "TEENSY.STATUS"}

def cmd_gnss_status() -> dict:
    """Request Teensy to enqueue GNSS_STATUS event."""
    return {"cmd": "GNSS.STATUS"}

def cmd_gnss_data() -> dict:
    """Request Teensy to enqueue GNSS_DATA event."""
    return {"cmd": "GNSS.DATA"}


# ===== Immediate query commands (interactive / future use) =====

def cmd_teensy_status_query() -> dict:
    """Immediate query for TEENSY_STATUS (no event)."""
    return {"cmd": "TEENSY.STATUS?"}

def cmd_gnss_status_query() -> dict:
    """Immediate query for GNSS_STATUS (no event)."""
    return {"cmd": "GNSS.STATUS?"}

def cmd_gnss_data_query() -> dict:
    """Immediate query for GNSS_DATA (no event)."""
    return {"cmd": "GNSS.DATA?"}
