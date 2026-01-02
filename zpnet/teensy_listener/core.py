"""
ZPNet Teensy Event Despooler — Transport Authority + Real-Time Query Plane

This daemon owns the Teensy USB CDC serial interface and performs two
orthogonal functions:

  1) Durable Event Despooling
     - Periodically drains the Teensy-side EVENT QUEUE via EVENTS.GET
     - Persists all events to PostgreSQL
     - This is the canonical memory path

  2) Ephemeral Real-Time Queries (NEW)
     - Synchronously issues Teensy commands
     - Immediately drains resulting events
     - Returns matching events in-memory only
     - No persistence, no aggregation, no side effects

Design invariants:
  • Exactly one owner of the Teensy serial port
  • Durable truth flows through Postgres
  • Ephemeral truth is observable but not remembered
  • Dashboard never touches serial directly

Author: The Mule + GPT
"""

import json
import logging
import time
import threading
import socket
import os
import serial
from typing import Any, Dict, List

from zpnet.shared.logger import setup_logging
from zpnet.shared.events import create_event
from zpnet.shared.constants import (
    TEENSY_SERIAL_PORT,
    TEENSY_BAUDRATE,
    TEENSY_READ_TIMEOUT_S,
)

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------
EVENT_DESPOOL_INTERVAL_S = 2.0
REALTIME_SOCKET_PATH = "/tmp/zpnet_teensy_rt.sock"
REALTIME_MAX_EVENTS = 16

# ---------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------
CMD_EVENTS_GET = {"cmd": "EVENTS.GET"}
EVENTS_BEGIN = "EVENTS_BEGIN"
EVENTS_END = "EVENTS_END"

# ---------------------------------------------------------------------
# Serial ownership primitives
# ---------------------------------------------------------------------
_serial_lock = threading.RLock()
_serial: serial.Serial | None = None

# ---------------------------------------------------------------------
# Serial helpers
# ---------------------------------------------------------------------
def open_teensy_serial() -> serial.Serial:
    ser = serial.Serial(
        TEENSY_SERIAL_PORT,
        TEENSY_BAUDRATE,
        timeout=TEENSY_READ_TIMEOUT_S,
        write_timeout=TEENSY_READ_TIMEOUT_S,
    )
    time.sleep(0.05)  # CDC settle
    return ser


def send_cmd(ser: serial.Serial, cmd: dict) -> None:
    payload = json.dumps(cmd, separators=(",", ":")) + "\n"
    logging.debug("[teensy_listener] → %s", payload.strip())
    ser.write(payload.encode("utf-8"))
    ser.flush()


# ---------------------------------------------------------------------
# Low-level framed drain (shared)
# ---------------------------------------------------------------------
def drain_events(
    ser: serial.Serial,
    *,
    persist: bool,
    collect: bool,
    stop_after: int | None = None,
) -> List[Dict[str, Any]]:
    """
    Drain one framed EVENTS.GET transaction.

    Args:
        persist: whether to write events to Postgres
        collect: whether to return events in-memory
        stop_after: optional max events to collect

    Returns:
        list of collected event dicts (if collect=True)
    """
    collected: List[Dict[str, Any]] = []
    in_drain = False

    send_cmd(ser, CMD_EVENTS_GET)

    while True:
        line = ser.read_until(b'\n', size=1024)
        if not line:
            break

        try:
            msg = json.loads(line.decode("utf-8", errors="ignore").strip())
        except json.JSONDecodeError:
            continue

        event_type = msg.get("event_type")
        if not event_type:
            continue

        if event_type == EVENTS_BEGIN:
            in_drain = True
            continue

        if event_type == EVENTS_END:
            break

        if not in_drain:
            continue

        payload = msg.copy()
        payload.pop("event_type", None)

        if persist:
            create_event(event_type, payload)

        if collect:
            collected.append(
                {
                    "event_type": event_type,
                    "payload": payload,
                }
            )
            if stop_after and len(collected) >= stop_after:
                break

    return collected


# ---------------------------------------------------------------------
# Durable despooling path (unchanged semantics)
# ---------------------------------------------------------------------
def drain_events_once(ser: serial.Serial) -> tuple[int, float]:
    """
    Perform one durable despool transaction.

    IMPORTANT:
        This path must yield to real-time queries.
        If the serial lock is held (likely by a real-time request),
        we skip this cycle rather than blocking.

    Returns:
        (events_received, elapsed_seconds)
    """
    start = time.time()

    acquired = _serial_lock.acquire(timeout=0.05)
    if not acquired:
        # Real-time query has priority; we do not block.
        return 0, 0.0

    try:
        events = drain_events(
            ser,
            persist=True,
            collect=False,
        )
        return len(events), time.time() - start
    finally:
        _serial_lock.release()


def perform_realtime_query(
    cmd: dict,
    *,
    timeout_s: float = 1.0,
) -> List[Dict[str, Any]]:
    """
    Issue a Teensy query command (CMD?) and synchronously return
    the immediate JSON reply.

    Semantics:
      • Reads exactly one JSON line
      • Does NOT drain the event queue
      • Does NOT persist anything
      • Returns a list for API symmetry

    Returns:
        list containing one dict with keys:
            { "type": ..., "payload": {...} }
        or empty list on timeout / failure
    """
    if _serial is None:
        return []

    deadline = time.time() + timeout_s

    with _serial_lock:
        send_cmd(_serial, cmd)

        # Read exactly one immediate reply line
        while time.time() < deadline:
            line = _serial.read_until(b"\n", size=1024)
            if not line:
                continue

            try:
                msg = json.loads(line.decode("utf-8", errors="ignore").strip())
            except json.JSONDecodeError:
                continue

            # Immediate replies use "type", not "event_type"
            msg_type = msg.get("type")
            if not msg_type:
                continue

            payload = msg.copy()
            payload.pop("type", None)

            return [
                {
                    "event_type": msg_type,
                    "payload": payload,
                }
            ]

    return []


# ---------------------------------------------------------------------
# Real-time IPC server (Unix socket)
# ---------------------------------------------------------------------
def realtime_socket_server():
    if os.path.exists(REALTIME_SOCKET_PATH):
        os.unlink(REALTIME_SOCKET_PATH)

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.bind(REALTIME_SOCKET_PATH)
    sock.listen(1)

    logging.info("🧭 [teensy_listener] real-time socket ready")

    while True:
        conn, _ = sock.accept()
        try:
            raw = conn.recv(4096)
            if not raw:
                continue

            req = json.loads(raw.decode("utf-8"))
            cmd = req.get("cmd")
            timeout = float(req.get("timeout_s", 1.0))

            if not isinstance(cmd, dict):
                raise ValueError("invalid cmd")

            events = perform_realtime_query(cmd, timeout_s=timeout)

            logging.info(
                "[rt] PHOTODIODE query returned %d events",
                len(events)
            )

            resp = json.dumps(
                {
                    "ok": True,
                    "events": events,
                },
                separators=(",", ":"),
            )

            # ----------------------------------------------------------
            # IMPORTANT:
            # Client may disconnect early (timeout, UI navigation, etc.).
            # BrokenPipeError must NOT kill this server thread.
            # ----------------------------------------------------------
            try:
                conn.sendall(resp.encode("utf-8"))
            except BrokenPipeError:
                logging.debug(
                    "[teensy_listener][rt] client disconnected before response could be delivered"
                )

        except BrokenPipeError:
            # Treat as benign: client left early. Do not attempt further writes.
            logging.debug(
                "[teensy_listener][rt] broken pipe during request handling (client disconnected)"
            )

        except Exception as e:
            # If the client is still connected, attempt to send an error response.
            # If the client disconnected, swallow BrokenPipe cleanly.
            err = json.dumps(
                {
                    "ok": False,
                    "error": str(e),
                },
                separators=(",", ":"),
            )
            try:
                conn.sendall(err.encode("utf-8"))
            except BrokenPipeError:
                logging.debug(
                    "[teensy_listener][rt] client disconnected before error could be delivered"
                )

        finally:
            try:
                conn.close()
            except Exception:
                pass


# ---------------------------------------------------------------------
# Main daemon loop
# ---------------------------------------------------------------------
def run() -> None:
    logging.info("🚀 [teensy_listener] starting Teensy transport authority")

    global _serial
    try:
        _serial = open_teensy_serial()
        logging.info("🔌 [teensy_listener] serial connection established")

        # Start real-time IPC server
        threading.Thread(
            target=realtime_socket_server,
            daemon=True,
        ).start()

        while True:
            events_received, elapsed = drain_events_once(_serial)
            logging.info(
                "✅ [teensy_listener] despool complete (%d events, %.2fs)",
                events_received,
                elapsed,
            )
            time.sleep(EVENT_DESPOOL_INTERVAL_S)

    except Exception as e:
        logging.exception("💥 [teensy_listener] daemon failure: %s", e)
        raise

    finally:
        try:
            if _serial:
                _serial.close()
        except Exception:
            pass


# ---------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------
def bootstrap() -> None:
    setup_logging()
    run()
