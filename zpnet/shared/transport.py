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

Non-responsibilities:
  • No RPC correlation
  • No retries or policy
  • No semantic inference
  • No application logic

Design philosophy:
  • Transport is boring and correct
  • Meaning crosses this boundary exactly once
  • Receive is adversarial and callback-driven
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from typing import Callable, Dict, Optional

from zpnet.shared import constants
from zpnet.shared.constants import Payload, TRAFFIC_DEBUG, TRAFFIC_PUBLISH_SUBSCRIBE, TRAFFIC_REQUEST_RESPONSE
from zpnet.shared.util import payload_to_json_bytes

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------
# Environment-selected transport (authoritative)
# ---------------------------------------------------------------------

ENV_TRANSPORT = "ZPNET_TRANSPORT"
TRANSPORT_HID = "HID"
TRANSPORT_SERIAL = "SERIAL"

# Canonical device paths (prefer shared constants if present)
TEENSY_HIDRAW_PATH = getattr(constants, "TEENSY_HIDRAW_PATH", "/dev/zpnet-teensy-hid")
TEENSY_SERIAL_PATH = getattr(constants, "TEENSY_SERIAL_PATH", "/dev/zpnet-teensy-serial")

# ---------------------------------------------------------------------
# Constants (authoritative, must match Teensy)
# ---------------------------------------------------------------------

TRANSPORT_BLOCK_SIZE = 64
TRANSPORT_MAX_MESSAGE = 10 * 1024

STX_PREFIX = b"<STX="
ETX_SEQ = b"<ETX>"
ETX_LEN = len(ETX_SEQ)

SERIAL_SNOOP_PATH = "/home/mule/zpnet/logs/zpnet-teensy-listener-snoop.log"

# Traffic bytes (shared vocabulary)
_VALID_TRAFFIC = {TRAFFIC_DEBUG, TRAFFIC_REQUEST_RESPONSE, TRAFFIC_PUBLISH_SUBSCRIBE}

# ---------------------------------------------------------------------
# Forensic logging (authoritative black box)
# ---------------------------------------------------------------------

RAW_TRANSPORT_LOG_PATH = "/home/mule/zpnet/logs/zpnet-transport.log"


# ---------------------------------------------------------------------
# Forensic snooping (semantic)
# --------------------------------------------------------------------

def _log_raw_transport(traffic: int, message: bytes, direction: str) -> None:
    try:
        ts = time.time()
        traffic_hex = f"0x{traffic:02X}"
        length = len(message)

        ascii_preview = message.decode("utf-8", errors="replace")
        hex_preview = message.hex()

        with open(RAW_TRANSPORT_LOG_PATH, "a") as f:
            f.write(
                f"{ts:.6f} {direction} traffic={traffic_hex} len={length}\n"
                f"ASCII: {ascii_preview}\n"
                f"HEX:   {hex_preview}\n"
                f"---\n"
            )
    except Exception:
        # Logging must never interfere with transport
        pass


def _snoop(direction: str, payload: bytes) -> None:
    try:
        with open(SERIAL_SNOOP_PATH, "a") as f:
            f.write(f"{time.time():.6f} {direction} {payload.decode('utf-8', errors='replace')}\n")
    except Exception:
        pass

# ---------------------------------------------------------------------
# Receive callback registry (semantic)
# ---------------------------------------------------------------------

_transport_recv_cb: Dict[int, Callable[[Payload], None]] = {}


def transport_register_receive_callback(traffic: int, cb: Callable[[Payload], None]) -> None:
    _transport_recv_cb[traffic] = cb


# ---------------------------------------------------------------------
# Physical transport state
# ---------------------------------------------------------------------

_transport_mode: Optional[str] = None

# HID backend
_hid_fd: Optional[int] = None
_hid_present = threading.Event()

# SERIAL backend
_ser = None  # pyserial.Serial instance when in SERIAL mode


# ---------------------------------------------------------------------
# Delimiting / Undelimiting (shared semantic helpers)
# ---------------------------------------------------------------------

def _delimit_for_send(payload: bytes) -> bytes:
    """
    Apply framing to semantic bytes.

    <STX=n> JSON <ETX>
    """
    header = b"<STX=" + str(len(payload)).encode("ascii") + b">"
    return header + payload + ETX_SEQ


def _undelimit_on_receive(msg: bytes) -> bytes:
    """
    Remove <STX=n> ... <ETX> framing and return raw JSON bytes.
    """
    header, _, rest = msg.partition(b">")
    declared_len = int(header[len(STX_PREFIX):])
    body, _, _ = rest.partition(ETX_SEQ)
    return body[:declared_len]

# ---------------------------------------------------------------------
# Dispatch (shared)
# ---------------------------------------------------------------------

def _dispatch_complete_message(traffic: int, framed_message: bytes) -> None:
    """
    framed_message is expected to be exactly:
      <STX=n> JSON <ETX>
    """
    undelimited = _undelimit_on_receive(framed_message)

    # Semantic snoop (post-undelimiting)
    _snoop("→", undelimited)

    payload = json.loads(undelimited.decode("utf-8"))

    cb = _transport_recv_cb.get(traffic)
    if cb is None:
        return
    cb(payload)


# ---------------------------------------------------------------------
# HID physical primitives (private)
# ---------------------------------------------------------------------

def _hid_send_block(block: bytes) -> None:
    if len(block) != TRANSPORT_BLOCK_SIZE:
        raise ValueError(f"HID block must be {TRANSPORT_BLOCK_SIZE} bytes")
    if not _hid_present.is_set() or _hid_fd is None:
        raise RuntimeError("Teensy HID not present")
    os.write(_hid_fd, block)


def _hid_recv_block() -> bytes:
    if not _hid_present.is_set() or _hid_fd is None:
        raise RuntimeError("Teensy HID not present")

    buf = bytearray()
    while len(buf) < TRANSPORT_BLOCK_SIZE:
        chunk = os.read(_hid_fd, TRANSPORT_BLOCK_SIZE - len(buf))
        if not chunk:
            continue
        buf.extend(chunk)
    return bytes(buf)


# ---------------------------------------------------------------------
# SERIAL physical primitives (private)
# ---------------------------------------------------------------------

def _serial_write(data: bytes) -> None:
    if _ser is None:
        raise RuntimeError("Teensy serial not open")
    _ser.write(data)
    _ser.flush()


def _serial_read_some() -> bytes:
    """
    Best-effort stream read:
      • If bytes are waiting, drain them.
      • Otherwise block for at least 1 byte.
    """
    if _ser is None:
        raise RuntimeError("Teensy serial not open")

    try:
        waiting = getattr(_ser, "in_waiting", 0)
    except Exception:
        waiting = 0

    if waiting and waiting > 0:
        return _ser.read(waiting)

    # Block until at least one byte arrives
    return _ser.read(1)


# ---------------------------------------------------------------------
# TX path split
# ---------------------------------------------------------------------

def _hid_tx_send(traffic: int, framed: bytes) -> None:
    """
    HID TX: fixed 64-byte reports. Mandatory padding.
    Physical message = [traffic] + framed
    """
    msg = bytes([traffic]) + framed
    _log_raw_transport(traffic, msg, "Pi -> Teensy")

    offset = 0
    while offset < len(msg):
        chunk = msg[offset:offset + TRANSPORT_BLOCK_SIZE]
        if len(chunk) < TRANSPORT_BLOCK_SIZE:
            chunk += b"\x00" * (TRANSPORT_BLOCK_SIZE - len(chunk))
        _hid_send_block(chunk)
        offset += TRANSPORT_BLOCK_SIZE


def _serial_tx_send(traffic: int, framed: bytes) -> None:
    """
    SERIAL TX: stream write. No block padding.
    Physical message = [traffic] + framed
    """
    msg = bytes([traffic]) + framed
    _log_raw_transport(traffic, msg, "Pi -> Teensy")
    _serial_write(msg)


# ---------------------------------------------------------------------
# RX path split
# ---------------------------------------------------------------------

def _hid_rx_loop() -> None:
    """
    HID RX: record device. Read fixed 64-byte blocks.
    First block: [traffic][payload...]
    Continuations: [payload...]
    Completion is length-authoritative via <STX=n>.
    """
    try:
        rx_buf = bytearray()
        traffic: Optional[int] = None
        expected_total: Optional[int] = None

        while True:
            block = _hid_recv_block()

            if traffic is None:
                traffic = block[0]
                payload_part = block[1:]
            else:
                payload_part = block

            if payload_part:
                rx_buf.extend(payload_part)

                # Simple completion test
                etx_pos = rx_buf.find(ETX_SEQ)
                if etx_pos != -1 and rx_buf.startswith(STX_PREFIX):
                    framed = bytes(rx_buf[:etx_pos + ETX_LEN])
                    traffic_done = traffic

                    traffic = None
                    rx_buf.clear()

                    _log_raw_transport(traffic_done, framed, "Teensy -> Pi")
                    _dispatch_complete_message(traffic_done, framed)

    except Exception:
        logger.exception("💥 [transport] HID RX loop fatal exception")


def _serial_rx_loop() -> None:
    """
    SERIAL RX: byte stream.
    Completion rule:
      framed message starts with <STX and ends with <ETX>
    """
    try:
        rx_buf = bytearray()
        traffic: Optional[int] = None

        while True:
            data = _serial_read_some()
            if not data:
                continue

            for b in data:
                # Hunt for traffic byte
                if traffic is None:
                    if b not in _VALID_TRAFFIC:
                        continue
                    traffic = b
                    rx_buf.clear()
                    continue

                # Accumulate framed bytes
                rx_buf.append(b)

                # Hard cap to prevent runaway
                if len(rx_buf) > TRANSPORT_MAX_MESSAGE:
                    traffic = None
                    rx_buf.clear()
                    continue

                # Simple completion test
                etx_pos = rx_buf.find(ETX_SEQ)
                if etx_pos != -1 and rx_buf.startswith(STX_PREFIX):
                    framed = bytes(rx_buf[:etx_pos + ETX_LEN])
                    traffic_done = traffic

                    traffic = None
                    rx_buf.clear()

                    _log_raw_transport(traffic_done, framed, "Teensy -> Pi")
                    _dispatch_complete_message(traffic_done, framed)

    except Exception:
        logger.exception("💥 [transport] SERIAL RX loop fatal exception")



# ---------------------------------------------------------------------
# Public semantic send API (facade)
# ---------------------------------------------------------------------

def transport_send(traffic: int, payload: Payload) -> None:
    """
    Public facade: send ONE complete semantic message.

    Callers do not know or care whether transport is HID or SERIAL.
    """
    raw = payload_to_json_bytes(payload)

    # Semantic snoop (pre-framing)
    _snoop("←", raw)

    framed = _delimit_for_send(raw)

    if _transport_mode is None:
        raise RuntimeError("transport not initialized")

    # Mode-specific TX (explicit split)
    if _transport_mode == TRANSPORT_HID:
        _hid_tx_send(traffic, framed)
        return
    if _transport_mode == TRANSPORT_SERIAL:
        _serial_tx_send(traffic, framed)
        return

    raise RuntimeError(f"unknown transport mode: {_transport_mode}")


# ---------------------------------------------------------------------
# Physical open / close
# ---------------------------------------------------------------------

def _open_hid(path: str) -> None:
    global _hid_fd
    if _hid_fd is not None:
        raise RuntimeError("HID already open")

    _hid_fd = os.open(path, os.O_RDWR)
    _hid_present.set()
    logger.info("🚀 [transport] HID opened: %s", path)


def _close_hid() -> None:
    global _hid_fd
    if _hid_fd is not None:
        try:
            os.close(_hid_fd)
        finally:
            _hid_fd = None
            _hid_present.clear()
            logger.info("🛑 [transport] HID closed")


def _open_serial(path: str) -> None:
    global _ser
    if _ser is not None:
        raise RuntimeError("serial already open")

    try:
        import serial  # type: ignore
    except Exception as e:
        raise RuntimeError("pyserial is required for ZPNET_TRANSPORT=SERIAL") from e

    _ser = serial.Serial(
        port=path,
        baudrate=115200,
        timeout=None,       # blocking
        write_timeout=None
    )
    logger.info("🚀 [transport] SERIAL opened: %s", path)


def _close_serial() -> None:
    global _ser
    if _ser is not None:
        try:
            _ser.close()
        finally:
            _ser = None
            logger.info("🛑 [transport] SERIAL closed")


def transport_close() -> None:
    """
    Close the underlying transport. (Best-effort.)
    """
    global _transport_mode

    try:
        if _transport_mode == TRANSPORT_HID:
            _close_hid()
        elif _transport_mode == TRANSPORT_SERIAL:
            _close_serial()
    finally:
        _transport_mode = None


# ---------------------------------------------------------------------
# Init (facade)
# ---------------------------------------------------------------------

def transport_init() -> None:
    """
    Initialize host-side transport based on ZPNET_TRANSPORT and start RX thread.

    Public behavior is invariant.
    Internal RX/TX are explicitly mode-split.
    """
    global _transport_mode

    mode = os.environ.get(ENV_TRANSPORT, "").strip().upper()
    if mode not in (TRANSPORT_HID, TRANSPORT_SERIAL):
        raise RuntimeError(f"{ENV_TRANSPORT} must be set to HID or SERIAL (got: {mode!r})")

    _transport_mode = mode

    if _transport_mode == TRANSPORT_HID:
        if not os.path.exists(TEENSY_HIDRAW_PATH):
            raise RuntimeError(f"Teensy HID device not found: {TEENSY_HIDRAW_PATH}")
        _open_hid(TEENSY_HIDRAW_PATH)

        threading.Thread(
            target=_hid_rx_loop,
            daemon=True,
            name="transport-hid-rx",
        ).start()

    elif _transport_mode == TRANSPORT_SERIAL:
        if not os.path.exists(TEENSY_SERIAL_PATH):
            raise RuntimeError(f"Teensy serial device not found: {TEENSY_SERIAL_PATH}")
        _open_serial(TEENSY_SERIAL_PATH)

        threading.Thread(
            target=_serial_rx_loop,
            daemon=True,
            name="transport-serial-rx",
        ).start()

    logger.info("✅ [transport] initialized mode=%s", _transport_mode)