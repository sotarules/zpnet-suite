"""
transport.py — ZPNet Host-Side Transport Layer (Pi)

This module is the single authoritative byte ↔ meaning boundary on the host
side. It is intentionally symmetric with the Teensy serial-only transport.

Architecture:
  • USB CDC Serial is the only physical path.
  • Public API remains invariant:
      - transport_send(...)
      - transport_register_receive_callback(...)
      - transport_init()
      - transport_close()

Responsibilities:
  • Own physical serial I/O
  • Delimit / undelimit semantic messages
  • Demultiplex by traffic byte
  • Deliver decoded messages via registered callbacks

Design philosophy:
  • Transport is boring and correct
  • Meaning crosses this boundary exactly once
  • Receive is adversarial and callback-driven
  • Transport never dies because of data
"""

from __future__ import annotations

import json
import logging
import signal
import threading
import time
from typing import Callable, Dict, Optional

import serial

from zpnet.shared import constants
from zpnet.shared.constants import (
    Payload,
    TRAFFIC_DEBUG,
    TRAFFIC_PUBLISH_SUBSCRIBE,
    TRAFFIC_REQUEST_RESPONSE,
)
from zpnet.shared.util import payload_to_json_bytes

import faulthandler
faulthandler.register(signal.SIGUSR1)

# ---------------------------------------------------------------------
# Physical device path
# ---------------------------------------------------------------------

TEENSY_SERIAL_PATH = getattr(constants, "TEENSY_SERIAL_PATH", "/dev/zpnet-teensy-serial")

# ---------------------------------------------------------------------
# Constants (must match Teensy)
# ---------------------------------------------------------------------

TRANSPORT_MAX_MESSAGE = 16 * 1024
RX_BUF_MAX = TRANSPORT_MAX_MESSAGE + 64

STX_PREFIX = b"<STX="
ETX_SEQ = b"<ETX>"
ETX_LEN = len(ETX_SEQ)

_VALID_TRAFFIC = {
    TRAFFIC_DEBUG,
    TRAFFIC_REQUEST_RESPONSE,
    TRAFFIC_PUBLISH_SUBSCRIBE,
}

RAW_TRANSPORT_LOG_PATH = "/home/mule/zpnet/logs/zpnet-transport.log"

# ---------------------------------------------------------------------
# Retry policy (aggressive, silent)
# ---------------------------------------------------------------------

_RETRY_MIN_SLEEP_S = 0.02
_RETRY_MAX_SLEEP_S = 0.50

# ---------------------------------------------------------------------
# Physical transport state (authoritative)
# ---------------------------------------------------------------------

# _ser is the single source of truth for SERIAL physical state.
# None => device not open / unavailable
# not None => open serial.Serial instance
_ser: Optional[serial.Serial] = None

# TX serialization. Serial write is physically single-path, but keeping the
# lock preserves the old public concurrency contract.
_tx_lock = threading.Lock()

# Supervisor control
_transport_stop = threading.Event()
_transport_supervisor_started = False

# ---------------------------------------------------------------------
# RX assembly state (observable, module-owned)
# ---------------------------------------------------------------------

rx_buf = bytearray()
rx_have_traffic = False
rx_traffic: Optional[int] = None

# ---------------------------------------------------------------------
# Open / close primitives (IMMORTAL)
# ---------------------------------------------------------------------

def _open_serial(path: str) -> None:
    global _ser

    if _ser is not None:
        return  # idempotent

    _ser = serial.Serial(port=path, baudrate=115200, timeout=None)
    logging.info("[transport] SERIAL device opened")


def _close_serial() -> None:
    global _ser

    ser = _ser
    if ser is not None:
        try:
            ser.close()
        except Exception:
            pass

    _ser = None
    logging.info("[transport] SERIAL device closed")

# ---------------------------------------------------------------------
# Forensic logging
# ---------------------------------------------------------------------

def render_bytes_lossless(data: bytes) -> str:
    out = []
    for b in data:
        if 32 <= b <= 126:   # printable ASCII
            out.append(chr(b))
        else:
            out.append(f"\\x{b:02X}")
    return ''.join(out)


def _log_raw_transport(traffic: int, message: bytes, direction: str) -> None:
    with open(RAW_TRANSPORT_LOG_PATH, "a") as f:
        f.write(
            f"{direction} 0x{traffic:02X} len={len(message)} "
            f"{render_bytes_lossless(message)}\n"
        )

# ---------------------------------------------------------------------
# Receive callback registry
# ---------------------------------------------------------------------

_transport_recv_cb: Dict[int, Callable[[Payload], None]] = {}


def transport_register_receive_callback(traffic: int, cb: Callable[[Payload], None]) -> None:
    _transport_recv_cb[traffic] = cb

# ---------------------------------------------------------------------
# Framing helpers
# ---------------------------------------------------------------------

def _delimit_for_send(payload: bytes) -> bytes:
    header = b"<STX=" + str(len(payload)).encode("ascii") + b">"
    return header + payload + ETX_SEQ


def _undelimit_on_receive(msg: bytes) -> bytes:
    header_end = msg.index(b">")
    declared_len = int(msg[len(STX_PREFIX):header_end])
    json_start = header_end + 1
    etx_pos = json_start + declared_len
    return msg[json_start:etx_pos]


def _validate_frame(framed: bytes) -> bool:
    header_end = framed.find(b">")
    ok = (
        framed.startswith(STX_PREFIX)
        and framed.endswith(ETX_SEQ)
        and header_end != -1
        and framed[len(STX_PREFIX):header_end].isdigit()
        and (
            len(framed)
            - (header_end + 1)
            - ETX_LEN
            == int(framed[len(STX_PREFIX):header_end])
        )
    )

    if not ok:
        logging.info("🧩 transport detected incorrectly formatted frame: %r", framed)

    return ok

# ---------------------------------------------------------------------
# Dispatch (IMMORTAL)
# ---------------------------------------------------------------------

def _dispatch_complete_message(traffic: int, framed: bytes) -> None:
    if not _validate_frame(framed):
        return

    raw = _undelimit_on_receive(framed)
    payload = json.loads(raw.decode("utf-8"))
    cb = _transport_recv_cb.get(traffic)
    if cb is None:
        logging.info("🧩 transport received unregistered traffic byte: 0x%02X", traffic)
        return
    cb(payload)

# ---------------------------------------------------------------------
# Physical primitives
# ---------------------------------------------------------------------

def _serial_write(data: bytes) -> None:
    ser = _ser
    if ser is None:
        return
    ser.write(data)
    ser.flush()


def _serial_read_some() -> bytes:
    ser = _ser
    if ser is None:
        return b""

    waiting = getattr(ser, "in_waiting", 0)
    return ser.read(waiting if waiting else 1)

# ---------------------------------------------------------------------
# TX path
# ---------------------------------------------------------------------

def _serial_tx_send(traffic: int, framed: bytes) -> None:
    msg = bytes([traffic]) + framed
    _log_raw_transport(traffic, msg, "Pi -> Teensy")
    _serial_write(msg)

# ---------------------------------------------------------------------
# RX loop (RETURN on physical failure)
# ---------------------------------------------------------------------

def _rx_reset() -> None:
    global rx_have_traffic, rx_traffic

    rx_buf.clear()
    rx_have_traffic = False
    rx_traffic = None


def _dispatch_if_complete() -> None:
    global rx_have_traffic, rx_traffic

    if not rx_have_traffic or rx_traffic is None:
        rx_buf.clear()
        return

    if len(rx_buf) < len(STX_PREFIX) + 2:
        return

    if not rx_buf.startswith(STX_PREFIX):
        logging.info("🧩 transport RX reset: missing STX prefix")
        _rx_reset()
        return

    header_end = rx_buf.find(b">")
    if header_end < 0:
        return

    declared = rx_buf[len(STX_PREFIX):header_end]
    if not declared.isdigit():
        logging.info("🧩 transport RX reset: invalid length header")
        _rx_reset()
        return

    declared_len = int(declared)
    required_total = header_end + 1 + declared_len + ETX_LEN
    if required_total > RX_BUF_MAX:
        logging.info("🧩 transport RX reset: declared length overflow")
        _rx_reset()
        return

    if len(rx_buf) < required_total:
        return

    etx_pos = header_end + 1 + declared_len
    if bytes(rx_buf[etx_pos:etx_pos + ETX_LEN]) != ETX_SEQ:
        logging.info("🧩 transport RX reset: missing ETX")
        _rx_reset()
        return

    framed = bytes(rx_buf[:required_total])
    traffic = rx_traffic
    _log_raw_transport(traffic, framed, "Teensy -> Pi")
    _dispatch_complete_message(traffic, framed)
    _rx_reset()


def _serial_rx_loop() -> None:
    global rx_have_traffic, rx_traffic

    _rx_reset()

    while True:
        data = _serial_read_some()
        for b in data:
            if not rx_have_traffic:
                if b in _VALID_TRAFFIC:
                    rx_traffic = b
                    rx_have_traffic = True
                    rx_buf.clear()
                continue

            rx_buf.append(b)
            if len(rx_buf) > RX_BUF_MAX:
                logging.info("🧩 transport RX reset: buffer overflow")
                _rx_reset()
                continue

            _dispatch_if_complete()

# ---------------------------------------------------------------------
# Supervisor (IMMORTAL CAMPER)
# ---------------------------------------------------------------------

def _supervisor_loop() -> None:
    """
    Immortal supervisor — keeps the serial RX path alive across device
    unavailability, flash cycles, and transient I/O errors.

    Behaviour:
      • First failure logs a single INFO line
      • All subsequent failures are swallowed silently
      • Closes stale handles before each retry
      • Polls with capped exponential backoff (20 ms → 500 ms)
      • Logs once on successful (re)connect
    """
    announced = False
    backoff = _RETRY_MIN_SLEEP_S

    while not _transport_stop.is_set():
        try:
            _open_serial(TEENSY_SERIAL_PATH)
            if announced:
                logging.info("✅ [transport] SERIAL device available — RX loop starting")
                announced = False
            backoff = _RETRY_MIN_SLEEP_S
            _serial_rx_loop()

        except Exception:
            # Device not ready, I/O error, stale handle, etc.
            # Close whatever is open so the next iteration re-opens cleanly.
            _close_serial()

            if not announced:
                logging.info("⏳ [transport] device not ready — supervisor retrying silently")
                announced = True

            time.sleep(backoff)
            backoff = min(backoff * 2.0, _RETRY_MAX_SLEEP_S)

# ---------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------

def transport_send(traffic: int, payload: Payload) -> None:
    raw = payload_to_json_bytes(payload)
    if len(raw) > TRANSPORT_MAX_MESSAGE:
        logging.info(
            "[transport] refusing oversized outbound payload traffic=0x%02X len=%d max=%d",
            traffic,
            len(raw),
            TRANSPORT_MAX_MESSAGE,
        )
        return

    framed = _delimit_for_send(raw)

    with _tx_lock:
        _serial_tx_send(traffic, framed)


def transport_init() -> None:
    global _transport_supervisor_started

    if not _transport_supervisor_started:
        threading.Thread(
            target=_supervisor_loop,
            daemon=True,
            name="transport-supervisor",
        ).start()
        _transport_supervisor_started = True


def transport_close() -> None:
    _transport_stop.set()
    _close_serial()


def transport_rx_snapshot() -> dict:
    """
    Return a read-only snapshot of the current RX assembly state.

    Semantics:
      • Observational only
      • No locking
      • No mutation
      • No interpretation
      • Best-effort physical truth
    """

    return {
        "rx_buf_len": len(rx_buf),
        "rx_buf_bytes": bytes(rx_buf),
        "rx_traffic": rx_traffic if rx_have_traffic else None,
        "rx_have_traffic": rx_have_traffic,
    }
