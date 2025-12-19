"""
ZPNet Teensy Event Despooler — Self-Timed Daemon

This daemon owns the Teensy USB CDC serial interface and periodically
drains the Teensy-side EVENT QUEUE using the explicit EVENTS.GET command.

Design contract:
  • The Teensy emits NOTHING unless explicitly commanded.
  • All durable truth flows through the EVENT queue.
  • EVENT queue output is framed (EVENTS_BEGIN / EVENTS_END).
  • This daemon owns the serial port continuously.
  • Time-based pacing is internal and explicit.
  • QUERY-style immediate responses are NOT handled here.

Author: The Mule + GPT
"""

import json
import logging
import time
import serial

from zpnet.shared.logger import setup_logging
from zpnet.shared.events import create_event
from zpnet.shared.constants import (
    TEENSY_SERIAL_PORT,
    TEENSY_BAUDRATE,
    TEENSY_READ_TIMEOUT_S,
)

# ---------------------------------------------------------------------
# Despool timing (transport policy, not scheduler policy)
# ---------------------------------------------------------------------
EVENT_DESPOOL_INTERVAL_S = 2.0   # how often we ask the Teensy if anything happened

# ---------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------
CMD_EVENTS_GET = {"cmd": "EVENTS.GET"}
EVENTS_BEGIN = "EVENTS_BEGIN"
EVENTS_END = "EVENTS_END"


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------
def open_teensy_serial() -> serial.Serial:
    """
    Open the Teensy serial port with canonical settings.
    Called exactly once at daemon startup.
    """
    ser = serial.Serial(
        TEENSY_SERIAL_PORT,
        TEENSY_BAUDRATE,
        timeout=TEENSY_READ_TIMEOUT_S,
        write_timeout=TEENSY_READ_TIMEOUT_S,
    )
    time.sleep(0.05)  # CDC settle
    return ser


def send_events_get(ser: serial.Serial) -> None:
    """
    Issue EVENTS.GET command to the Teensy.
    """
    payload = json.dumps(CMD_EVENTS_GET, separators=(",", ":")) + "\n"
    logging.debug("[teensy_listener] → EVENTS.GET")
    ser.write(payload.encode("utf-8"))
    ser.flush()


def drain_events_once(ser: serial.Serial) -> tuple[int, float]:
    """
    Perform exactly one framed EVENT despool transaction.

    Returns:
        (events_received, elapsed_seconds)
    """
    start_time = time.time()
    events_received = 0
    in_drain = False

    send_events_get(ser)

    while True:
        line = ser.readline()
        if not line:
            if in_drain:
                logging.warning("[teensy_listener] timeout during event drain")
            break

        try:
            msg = json.loads(line.decode("utf-8", errors="ignore").strip())
        except json.JSONDecodeError:
            logging.warning(f"[teensy_listener] invalid JSON from Teensy: {line!r}")
            continue

        event_type = msg.get("event_type")
        if not event_type:
            logging.warning(f"[teensy_listener] missing event_type: {msg}")
            continue

        # --------------------------------------------------------------
        # EVENT framing
        # --------------------------------------------------------------
        if event_type == EVENTS_BEGIN:
            in_drain = True
            logging.debug(
                "[teensy_listener] EVENTS_BEGIN "
                f"(count={msg.get('count')}, dropped={msg.get('dropped')})"
            )
            continue

        if event_type == EVENTS_END:
            break

        if not in_drain:
            # Protocol violation: unframed message
            logging.warning(
                f"[teensy_listener] unframed message ignored: {event_type}"
            )
            continue

        # --------------------------------------------------------------
        # EVENT payload
        # --------------------------------------------------------------
        payload = msg.copy()
        payload.pop("event_type", None)

        create_event(event_type, payload)
        events_received += 1

    elapsed = time.time() - start_time
    return events_received, elapsed


# ---------------------------------------------------------------------
# Main Daemon Loop
# ---------------------------------------------------------------------
def run() -> None:
    """
    Main daemon loop.

    Owns the serial port continuously and periodically
    drains the Teensy EVENT queue.
    """
    logging.info("🚀 [teensy_listener] Teensy event despooler daemon starting")

    ser = None
    try:
        ser = open_teensy_serial()
        logging.info("🔌 [teensy_listener] serial connection established")

        while True:
            events_received, elapsed = drain_events_once(ser)

            logging.info(
                f"✅ [teensy_listener] despool complete "
                f"({events_received} events, {elapsed:.2f}s)"
            )

            time.sleep(EVENT_DESPOOL_INTERVAL_S)

    except Exception as e:
        logging.exception(f"💥 [teensy_listener] daemon failure: {e}")
        raise

    finally:
        try:
            if ser:
                ser.close()
        except Exception:
            pass


# ---------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------
def bootstrap() -> None:
    """Setup logging and enter daemon loop."""
    setup_logging()
    run()
