"""
transport.py — ZPNet Host-Side Transport Layer (Pi)

This module is the single authoritative byte ↔ meaning boundary
on the host side. It is intentionally symmetric with
`transport.cpp` on the Teensy.

Responsibilities:
  • Own physical I/O (hidraw OR USB CDC serial)
  • Fragment and reassemble fixed 64-byte blocks
  • Delimit / undelimit messages by traffic class
  • Demultiplex by traffic byte
  • Deliver decoded messages via registered callbacks
  • Preserve forensic snooping

Non-responsibilities:
  • No RPC correlation
  • No retries or policy
  • No semantic inference
  • No application logic

Design philosophy:
  • Transport is boring and correct
  • Meaning crosses this boundary exactly once
  • Receive is asynchronous and callback-driven
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from typing import Callable, Dict, Optional, Any

from zpnet.shared import constants
from zpnet.shared.constants import Payload
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
ETX_SEQ    = b"<ETX>"
ETX_LEN    = len(ETX_SEQ)

SERIAL_SNOOP_PATH = "/home/mule/zpnet/logs/zpnet-teensy-listener-snoop.log"

# ---------------------------------------------------------------------
# Forensic logging (authoritative black box)
# ---------------------------------------------------------------------

RAW_TRANSPORT_LOG_PATH = "/home/mule/zpnet/logs/zpnet-transport.log"

def _log_raw_transport(
    traffic: int,
    message: bytes,
    direction: str
) -> None:
    """
    Log a fully reassembled transport message exactly once,
    before any semantic processing.

    This is a forensic truth surface.
    """
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

# ---------------------------------------------------------------------
# Receive callback registry (semantic)
# ---------------------------------------------------------------------

_transport_recv_cb: Dict[int, Callable[[Payload], None]] = {}

def transport_register_receive_callback(
    traffic: int,
    cb: Callable[[Payload], None]
) -> None:
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

# Shared write lock
_write_lock = threading.Lock()

# ---------------------------------------------------------------------
# Public physical boundary (64-byte blocks)
# ---------------------------------------------------------------------

def transport_send_block(block: bytes) -> None:
    """
    Physical boundary: send exactly one 64-byte block.
    Padding is physical only (zeros), never semantic.
    """
    if not isinstance(block, (bytes, bytearray)):
        raise TypeError("transport_send_block requires bytes/bytearray")

    if len(block) != TRANSPORT_BLOCK_SIZE:
        raise ValueError(f"transport_send_block requires {TRANSPORT_BLOCK_SIZE} bytes, got {len(block)}")

    if _transport_mode is None:
        raise RuntimeError("transport not initialized")

    with _write_lock:
        if _transport_mode == TRANSPORT_HID:
            if not _hid_present.is_set() or _hid_fd is None:
                raise RuntimeError("Teensy HID not present")
            os.write(_hid_fd, block)
            return

        if _transport_mode == TRANSPORT_SERIAL:
            if _ser is None:
                raise RuntimeError("Teensy serial not open")
            _ser.write(block)
            _ser.flush()
            return

        raise RuntimeError(f"unknown transport mode: {_transport_mode}")

def transport_recv_block() -> bytes:
    """
    Physical boundary: receive exactly one 64-byte block.
    Blocks until full block is available.
    """
    if _transport_mode is None:
        raise RuntimeError("transport not initialized")

    if _transport_mode == TRANSPORT_HID:
        if not _hid_present.is_set() or _hid_fd is None:
            raise RuntimeError("Teensy HID not present")

        # hidraw read should return exactly 64 bytes, but loop defensively
        buf = bytearray()
        while len(buf) < TRANSPORT_BLOCK_SIZE:
            chunk = os.read(_hid_fd, TRANSPORT_BLOCK_SIZE - len(buf))
            if not chunk:
                continue
            buf.extend(chunk)
        return bytes(buf)

    if _transport_mode == TRANSPORT_SERIAL:
        if _ser is None:
            raise RuntimeError("Teensy serial not open")

        # pyserial read() blocks until timeout; we run with timeout=None (blocking)
        data = _ser.read(TRANSPORT_BLOCK_SIZE)
        while len(data) < TRANSPORT_BLOCK_SIZE:
            more = _ser.read(TRANSPORT_BLOCK_SIZE - len(data))
            if not more:
                continue
            data += more
        return data

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

    # Blocking read model: timeout=None means block until requested bytes arrive.
    _ser = serial.Serial(
        port=path,
        baudrate=115200,
        timeout=None,
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
    if _transport_mode == TRANSPORT_HID:
        _close_hid()
    elif _transport_mode == TRANSPORT_SERIAL:
        _close_serial()
    _transport_mode = None

# ---------------------------------------------------------------------
# Delimiting (send-side)
# ---------------------------------------------------------------------

def _delimit_for_send(traffic: int, payload: bytes) -> bytes:
    """
    Apply framing to all semantic traffic.

    <STX=n> JSON <ETX>
    """
    header = b"<STX=" + str(len(payload)).encode("ascii") + b">"
    return header + payload + ETX_SEQ


# ---------------------------------------------------------------------
# Undelimiting (receive-side)
# ---------------------------------------------------------------------

def _undelimit_on_receive(traffic: int, msg: bytes) -> bytes:
    """
    Remove <STX=n> ... <ETX> framing and return raw JSON bytes.
    All semantic traffic is now framed.
    """
    header, _, rest = msg.partition(b">")
    declared_len = int(header[len(STX_PREFIX):])
    body, _, _ = rest.partition(ETX_SEQ)
    return body[:declared_len]


# ---------------------------------------------------------------------
# Public semantic send API
# ---------------------------------------------------------------------

def transport_send(traffic: int, payload: Payload) -> None:
    """
    Send ONE complete semantic message.

    The object is JSON-encoded here, then passed through
    transport-level delimiting and fragmentation.
    """
    raw = payload_to_json_bytes(payload)

    _snoop("←", raw)

    framed = _delimit_for_send(traffic, raw)

    # transport.cpp semantics: first block carries traffic byte
    _fragment_and_send(framed, traffic)

# ---------------------------------------------------------------------
# Fragmentation (send-side)
# ---------------------------------------------------------------------

def _fragment_and_send(data: bytes, traffic: int) -> None:
    """
    Fragment a fully delimited message into 64-byte blocks.

    First block:
      [traffic][payload...]

    Subsequent blocks:
      [payload...]

    Padding is physical only (zeros), never semantic.
    """
    if _transport_mode is None:
        raise RuntimeError("transport not initialized")

    msg = bytes([traffic]) + data

    # 🔴 FORENSIC TAP — log before any processing
    _log_raw_transport(traffic, msg, "Pi -> Teensy")

    offset = 0
    while offset < len(msg):
        chunk = msg[offset:offset + TRANSPORT_BLOCK_SIZE]
        if len(chunk) < TRANSPORT_BLOCK_SIZE:
            chunk += b"\x00" * (TRANSPORT_BLOCK_SIZE - len(chunk))
        transport_send_block(chunk)
        offset += TRANSPORT_BLOCK_SIZE

# ---------------------------------------------------------------------
# Reader loop (single RX authority)
# ---------------------------------------------------------------------

def _try_parse_expected_len(buf: bytearray) -> Optional[int]:
    """
    If buf begins with <STX=n>, return the total framed message length:
      header_len + n + ETX_LEN
    Otherwise return None.

    Minimal contract:
      • require STX at offset 0
      • parse decimal digits until '>'
      • do NOT rely on padding
    """
    if len(buf) < len(STX_PREFIX) + 2:   # need at least "<STX=0>"
        return None

    if not buf.startswith(STX_PREFIX):
        return None

    i = len(STX_PREFIX)
    n = 0
    saw_digit = False

    while i < len(buf) and 48 <= buf[i] <= 57:  # '0'..'9'
        saw_digit = True
        n = (n * 10) + (buf[i] - 48)
        i += 1

    if not saw_digit:
        return None

    if i >= len(buf) or buf[i] != ord('>'):
        return None

    header_len = i + 1  # include '>'
    return header_len + n + ETX_LEN

def reader_loop() -> None:
    """
    Blocking reader loop.

    Owns:
      • block reassembly
      • traffic extraction
      • delimiter-based message termination (length-authoritative for REQUEST_RESPONSE)
      • dispatch
    """
    try:
        rx_buf = bytearray()
        traffic: Optional[int] = None
        expected_total: Optional[int] = None

        while True:
            try:
                block = transport_recv_block()
            except Exception:
                logger.exception("💥 [transport] transport_recv_block fatal")
                return

            # First block: traffic byte is leading
            if traffic is None:
                traffic = block[0]
                payload_part = block[1:]
            else:
                payload_part = block

            if payload_part:
                rx_buf.extend(payload_part)

                if expected_total is None:
                    expected_total = _try_parse_expected_len(rx_buf)

                if expected_total is not None and len(rx_buf) >= expected_total:
                    message = bytes(rx_buf[:expected_total])

                    # Message-atomic receive model
                    rx_buf.clear()
                    expected_total = None
                    traffic_done = traffic
                    traffic = None

                    # 🔴 FORENSIC TAP — log before any processing
                    _log_raw_transport(traffic_done, message, "Teensy -> Pi")

                    _dispatch_complete_message(traffic_done, message)

            # else: wait for an explicit invariant (do nothing)

    except Exception:
        logger.exception("💥 [transport] reader_loop fatal exception")

# ---------------------------------------------------------------------
# Dispatch
# ---------------------------------------------------------------------

def _dispatch_complete_message(traffic: int, message: bytes) -> None:

    undelimited = _undelimit_on_receive(traffic, message)

    _snoop("→", undelimited)

    payload = json.loads(undelimited.decode("utf-8"))

    cb = _transport_recv_cb.get(traffic)
    if cb is None:
        return

    cb(payload)

# ---------------------------------------------------------------------
# Forensic snooping
# ---------------------------------------------------------------------

def _snoop(direction: str, payload: bytes) -> None:
    try:
        with open(SERIAL_SNOOP_PATH, "a") as f:
            f.write(f"{time.time():.6f} {direction} {payload.decode('utf-8', errors='replace')}\n")
    except Exception:
        pass

# ---------------------------------------------------------------------
# Init
# ---------------------------------------------------------------------

def transport_init() -> None:
    """
    Initialize host-side transport based on ZPNET_TRANSPORT and start RX thread.
    """
    global _transport_mode

    mode = os.environ.get(ENV_TRANSPORT, "").strip().upper()
    if mode not in (TRANSPORT_HID, TRANSPORT_SERIAL):
        raise RuntimeError(
            f"{ENV_TRANSPORT} must be set to HID or SERIAL (got: {mode!r})"
        )

    _transport_mode = mode

    if _transport_mode == TRANSPORT_HID:
        if not os.path.exists(TEENSY_HIDRAW_PATH):
            raise RuntimeError(f"Teensy HID device not found: {TEENSY_HIDRAW_PATH}")
        _open_hid(TEENSY_HIDRAW_PATH)

    elif _transport_mode == TRANSPORT_SERIAL:
        if not os.path.exists(TEENSY_SERIAL_PATH):
            raise RuntimeError(f"Teensy serial device not found: {TEENSY_SERIAL_PATH}")
        _open_serial(TEENSY_SERIAL_PATH)

    logger.info("✅ [transport] initialized mode=%s", _transport_mode)

    threading.Thread(
        target=reader_loop,
        daemon=True,
        name="transport-reader"
    ).start()
