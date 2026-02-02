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
  • Preserve snoop and forensic logging

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
import threading
import time
from typing import Callable, Dict, Optional

from zpnet.shared import constants
from zpnet.shared.constants import (
    Payload,
    TRAFFIC_DEBUG,
    TRAFFIC_PUBLISH_SUBSCRIBE,
    TRAFFIC_REQUEST_RESPONSE,
)
from zpnet.shared.util import payload_to_json_bytes

logger = logging.getLogger(__name__)

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
SERIAL_SNOOP_PATH = "/home/mule/zpnet/logs/zpnet-teensy-listener-snoop.log"

# ---------------------------------------------------------------------
# Retry policy (aggressive, silent)
# ---------------------------------------------------------------------

_RETRY_MIN_SLEEP_S = 0.02
_RETRY_MAX_SLEEP_S = 0.50

def _sleep_backoff(s: float) -> float:
    time.sleep(s)
    return min(s * 2.0, _RETRY_MAX_SLEEP_S)

# ---------------------------------------------------------------------
# Forensic logging
# ---------------------------------------------------------------------

def _log_raw_transport(traffic: int, message: bytes, direction: str) -> None:
    try:
        ts = time.time()
        with open(RAW_TRANSPORT_LOG_PATH, "a") as f:
            f.write(
                f"{direction} 0x{traffic:02X} len={len(message)} {message.decode('utf-8', errors='replace')}\n"
            )
    except Exception:
        pass


def _snoop(direction: str, payload: bytes) -> None:
    try:
        with open(SERIAL_SNOOP_PATH, "a") as f:
            f.write(
                f"{time.time():.6f} {direction} "
                f"{payload.decode('utf-8', errors='replace')}\n"
            )
    except Exception:
        pass

# ---------------------------------------------------------------------
# Receive callback registry
# ---------------------------------------------------------------------

_transport_recv_cb: Dict[int, Callable[[Payload], None]] = {}

def transport_register_receive_callback(traffic: int, cb: Callable[[Payload], None]) -> None:
    _transport_recv_cb[traffic] = cb

# ---------------------------------------------------------------------
# Physical transport state
# ---------------------------------------------------------------------

_transport_mode: Optional[str] = None

_hid_fd: Optional[int] = None
_hid_present = threading.Event()

_ser = None  # pyserial.Serial

# Supervisor control
_transport_stop = threading.Event()
_transport_supervisor_started = False

# ---------------------------------------------------------------------
# Framing helpers
# ---------------------------------------------------------------------

def _delimit_for_send(payload: bytes) -> bytes:
    header = b"<STX=" + str(len(payload)).encode("ascii") + b">"
    return header + payload + ETX_SEQ


def _undelimit_on_receive(msg: bytes) -> bytes:
    if not msg.startswith(STX_PREFIX):
        raise ValueError("missing <STX=>")

    header_end = msg.find(b">")
    if header_end == -1:
        raise ValueError("unterminated STX header")

    declared_len = int(msg[len(STX_PREFIX):header_end])
    json_start = header_end + 1
    etx_pos = json_start + declared_len

    if msg[etx_pos:etx_pos + ETX_LEN] != ETX_SEQ:
        raise ValueError("ETX not at declared boundary")

    return msg[json_start:json_start + declared_len]

# ---------------------------------------------------------------------
# Dispatch (IMMORTAL)
# ---------------------------------------------------------------------

def _dispatch_complete_message(traffic: int, framed: bytes) -> None:
    try:
        raw = _undelimit_on_receive(framed)
        _snoop("→", raw)
        payload = json.loads(raw.decode("utf-8"))

        cb = _transport_recv_cb.get(traffic)
        if cb:
            cb(payload)

    except Exception:
        logger.exception(
            "⚠️ [transport] dropped malformed frame traffic=0x%02X len=%d",
            traffic,
            len(framed),
        )

# ---------------------------------------------------------------------
# Physical primitives
# ---------------------------------------------------------------------

def _hid_send_block(block: bytes) -> None:
    if len(block) != TRANSPORT_BLOCK_SIZE:
        raise ValueError("bad HID block size")
    os.write(_hid_fd, block)


def _hid_recv_block() -> bytes:
    buf = bytearray()
    while len(buf) < TRANSPORT_BLOCK_SIZE:
        chunk = os.read(_hid_fd, TRANSPORT_BLOCK_SIZE - len(buf))
        if not chunk:
            continue
        buf.extend(chunk)
    return bytes(buf)


def _serial_write(data: bytes) -> None:
    _ser.write(data)
    _ser.flush()


def _serial_read_some() -> bytes:
    waiting = getattr(_ser, "in_waiting", 0)
    return _ser.read(waiting if waiting else 1)

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
    rx_buf = bytearray()
    traffic: Optional[int] = None

    while True:
        block = _hid_recv_block()

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
# Physical open / close
# ---------------------------------------------------------------------

def _open_hid(path: str) -> None:
    global _hid_fd
    _hid_fd = os.open(path, os.O_RDWR)
    _hid_present.set()


def _close_hid() -> None:
    global _hid_fd
    if _hid_fd is not None:
        os.close(_hid_fd)
        _hid_fd = None
    _hid_present.clear()


def _open_serial(path: str) -> None:
    global _ser
    import serial
    _ser = serial.Serial(port=path, baudrate=115200, timeout=None)


def _close_serial() -> None:
    global _ser
    if _ser:
        _ser.close()
        _ser = None

# ---------------------------------------------------------------------
# Supervisor (IMMORTAL CAMPER)
# ---------------------------------------------------------------------

def _supervisor_loop() -> None:
    sleep_s = _RETRY_MIN_SLEEP_S

    while not _transport_stop.is_set():
        try:
            mode = _transport_mode
            if mode is None:
                sleep_s = _sleep_backoff(sleep_s)
                continue

            if mode == TRANSPORT_SERIAL:
                if not os.path.exists(TEENSY_SERIAL_PATH):
                    sleep_s = _sleep_backoff(sleep_s)
                    continue
                if _ser is None:
                    _open_serial(TEENSY_SERIAL_PATH)
                sleep_s = _RETRY_MIN_SLEEP_S
                _serial_rx_loop()

            elif mode == TRANSPORT_HID:
                if not os.path.exists(TEENSY_HIDRAW_PATH):
                    sleep_s = _sleep_backoff(sleep_s)
                    continue
                if _hid_fd is None:
                    _open_hid(TEENSY_HIDRAW_PATH)
                sleep_s = _RETRY_MIN_SLEEP_S
                _hid_rx_loop()

        except Exception:
            try:
                _close_serial()
                _close_hid()
            except Exception:
                pass
            sleep_s = _sleep_backoff(sleep_s)

# ---------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------

def transport_send(traffic: int, payload: Payload) -> None:
    raw = payload_to_json_bytes(payload)
    _snoop("←", raw)
    framed = _delimit_for_send(raw)

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
    try:
        _close_serial()
        _close_hid()
    finally:
        pass
