"""
transport.py — ZPNet Host-Side Transport Layer (Pi)

This module is the single authoritative byte ↔ meaning boundary
on the host side. It is intentionally symmetric with
`transport.cpp` on the Teensy.

NEW ARCHITECTURE (TRUTHFUL SPLIT):
  • Four explicit physical paths:
      - HID   TX
      - HID   RX
      - SERIAL TX
      - SERIAL RX
  • Public API remains invariant:
      - transport_send(...)
      - transport_register_receive_callback(...)
      - transport_init()
      - transport_close()

Responsibilities:
  • Own physical I/O (hidraw OR USB CDC serial)
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
import os
import signal
import threading
import time
import hid
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
# Environment-selected transport (authoritative)
# ---------------------------------------------------------------------

ENV_TRANSPORT = "ZPNET_TRANSPORT"
TRANSPORT_HID = "HID"
TRANSPORT_SERIAL = "SERIAL"

TEENSY_HIDRAW_PATH = getattr(constants, "TEENSY_HIDRAW_PATH", "/dev/zpnet-teensy-hid")
TEENSY_SERIAL_PATH = getattr(constants, "TEENSY_SERIAL_PATH", "/dev/zpnet-teensy-serial")

# ---------------------------------------------------------------------
# Constants (must match Teensy)
# ---------------------------------------------------------------------

TRANSPORT_BLOCK_SIZE = 64
TRANSPORT_MAX_MESSAGE = 10 * 1024

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

# _hid_dev is the single source of truth for HID physical state.
# None => device not open / unavailable
# not None => valid, open hid.device()
_hid_dev: Optional[hid.device] = None

# _ser is the single source of truth for SERIAL physical state.
# None => device not open / unavailable
# not None => open serial.Serial instance
_ser: Optional["serial.Serial"] = None

_transport_mode: Optional[str] = None

# TX serialization (RawHID exclusivity)
_tx_lock = threading.Lock()

# Supervisor control
_transport_stop = threading.Event()
_transport_supervisor_started = False

# ---------------------------------------------------------------------
# RX assembly state (observable, module-owned)
# ---------------------------------------------------------------------

rx_buf = bytearray()
rx_have_traffic = False
rx_traffic = None

# ---------------------------------------------------------------------
# Open / close primitives (IMMORTAL)
# ---------------------------------------------------------------------

def _open_hid(_: str = None) -> None:
    global _hid_dev

    if _hid_dev is not None:
        return  # already open

    dev = hid.device()
    dev.open_path(b'1-1.1:1.0')
    dev.set_nonblocking(False)

    _hid_dev = dev
    logging.info("[transport] HID device opened")


def _close_hid() -> None:
    global _hid_dev

    dev = _hid_dev
    if dev is not None:
        try:
            dev.close()
        except Exception:
            pass

    _hid_dev = None
    logging.info("[transport] HID device closed")


def _open_serial(path: str) -> None:
    global _ser

    if _ser is not None:
        return  # idempotent

    import serial
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
# Helpers
# ---------------------------------------------------------------------

def _sleep_backoff(s: float) -> float:
    time.sleep(s)
    return min(s * 2.0, _RETRY_MAX_SLEEP_S)

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
    ok = (
        framed.startswith(STX_PREFIX)
        and framed.endswith(ETX_SEQ)
        and framed.find(b">") != -1
        and framed[len(STX_PREFIX):framed.find(b">")].isdigit()
        and (
            len(framed)
            - (framed.find(b">") + 1)
            - ETX_LEN
            == int(framed[len(STX_PREFIX):framed.find(b">")])
        )
    )

    if not ok:
        logging.info("🧩 transport detected incorrectly formatted frame: %r",framed)

    return ok

# ---------------------------------------------------------------------
# Dispatch (IMMORTAL)
# ---------------------------------------------------------------------

def _dispatch_complete_message(traffic: int, framed: bytes) -> None:
    _validate_frame(framed)
    raw = _undelimit_on_receive(framed)
    payload = json.loads(raw.decode("utf-8"))
    cb = _transport_recv_cb[traffic]
    cb(payload)

# ---------------------------------------------------------------------
# Physical primitives
# ---------------------------------------------------------------------

def _hid_send_block(block: bytes) -> None:
    _hid_dev.write(block)


def _hid_recv_block(timeout_ms: int = 1000) -> bytes:
    data = _hid_dev.read(TRANSPORT_BLOCK_SIZE, timeout_ms)
    return bytes(data)


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
# TX paths
# ---------------------------------------------------------------------

def _hid_tx_send(traffic: int, framed: bytes) -> None:
    msg = bytes([traffic]) + framed
    _log_raw_transport(traffic, msg, "Pi -> Teensy")

    offset = 0
    while offset < len(msg):
        chunk = msg[offset:offset + TRANSPORT_BLOCK_SIZE]
        chunk += b"\x00" * (TRANSPORT_BLOCK_SIZE - len(chunk))
        _hid_send_block(chunk)
        offset += TRANSPORT_BLOCK_SIZE


def _serial_tx_send(traffic: int, framed: bytes) -> None:
    msg = bytes([traffic]) + framed
    _log_raw_transport(traffic, msg, "Pi -> Teensy")
    _serial_write(msg)

# ---------------------------------------------------------------------
# RX loops (RETURN on physical failure)
# ---------------------------------------------------------------------

def _hid_rx_loop() -> None:

    global rx_buf, rx_have_traffic, rx_traffic

    rx_buf = bytearray()
    traffic: Optional[int] = None

    while True:
        block = _hid_recv_block()
        if block is None:
            continue  # idle wait

        # Mule defense 1
        if len(block)  == 0:
            continue  # idle wait

        if traffic is None:
            traffic = block[0]
            rx_buf.clear()
            rx_buf.extend(block[1:])
        else:
            rx_buf.extend(block)

        etx_pos = rx_buf.find(ETX_SEQ)
        if etx_pos != -1 and rx_buf.startswith(STX_PREFIX):
            framed = bytes(rx_buf[:etx_pos + ETX_LEN])
            _log_raw_transport(traffic, framed, "Teensy -> Pi")
            _dispatch_complete_message(traffic, framed)
            traffic = None
            rx_buf.clear()


def _serial_rx_loop() -> None:

    global rx_buf, rx_have_traffic, rx_traffic

    rx_buf = bytearray()
    traffic: Optional[int] = None

    while True:
        data = _serial_read_some()
        for b in data:
            if traffic is None:
                if b in _VALID_TRAFFIC:
                    traffic = b
                    rx_buf.clear()
                continue

            rx_buf.append(b)
            if len(rx_buf) > TRANSPORT_MAX_MESSAGE:
                traffic = None
                rx_buf.clear()
                continue

            etx_pos = rx_buf.find(ETX_SEQ)
            if etx_pos != -1 and rx_buf.startswith(STX_PREFIX):
                framed = bytes(rx_buf[:etx_pos + ETX_LEN])
                _log_raw_transport(traffic, framed, "Teensy -> Pi")
                _dispatch_complete_message(traffic, framed)
                traffic = None
                rx_buf.clear()

# ---------------------------------------------------------------------
# Supervisor (IMMORTAL CAMPER)
# ---------------------------------------------------------------------

def _supervisor_loop() -> None:
    while not _transport_stop.is_set():
        mode = _transport_mode

        if mode == TRANSPORT_SERIAL:
            _open_serial(TEENSY_SERIAL_PATH)
            _serial_rx_loop()

        elif mode == TRANSPORT_HID:
            _open_hid()
            _hid_rx_loop()

        else:
            raise RuntimeError("transport mode not set")

# ---------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------

def transport_send(traffic: int, payload: Payload) -> None:
    raw = payload_to_json_bytes(payload)
    framed = _delimit_for_send(raw)

    with _tx_lock:
        if _transport_mode == TRANSPORT_HID:
            _hid_tx_send(traffic, framed)
        elif _transport_mode == TRANSPORT_SERIAL:
            _serial_tx_send(traffic, framed)
        else:
            raise RuntimeError("transport not initialized")


def transport_init() -> None:
    global _transport_mode, _transport_supervisor_started

    mode = os.environ.get(ENV_TRANSPORT, "").strip().upper()
    if mode not in (TRANSPORT_HID, TRANSPORT_SERIAL):
        raise RuntimeError(f"{ENV_TRANSPORT} must be HID or SERIAL")

    _transport_mode = mode

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
    _close_hid()

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
