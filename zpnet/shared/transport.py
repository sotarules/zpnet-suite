"""
transport.py — ZPNet Host-Side Transport Layer (Pi)

This module is the single authoritative byte ↔ meaning boundary
on the host side. It is intentionally symmetric with
`transport.cpp` on the Teensy.

Responsibilities:
  • Own hidraw I/O
  • Fragment and reassemble 64-byte HID packets
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

# ---------------------------------------------------------------------
# Constants (authoritative, must match Teensy)
# ---------------------------------------------------------------------

HID_PACKET_SIZE = 64
TRANSPORT_MAX_MESSAGE = 10 * 1024

STX_PREFIX = b"<STX="
ETX_SEQ    = b"<ETX>"

SERIAL_SNOOP_PATH = "/home/mule/zpnet/logs/zpnet-teensy-listener-snoop.log"

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------
# Raw HID forensic logging (authoritative black box)
# ---------------------------------------------------------------------

RAW_HID_LOG_PATH = "/home/mule/zpnet/logs/zpnet-hid.log"

def _log_raw_hid(
    traffic: int,
    message: bytes
) -> None:
    """
    Log a fully reassembled HID message exactly once,
    before any semantic processing.

    This is a forensic truth surface.
    """
    try:
        ts = time.time()
        traffic_hex = f"0x{traffic:02X}"
        length = len(message)

        # Safe dual representation
        ascii_preview = message.decode("utf-8", errors="replace")
        hex_preview = message.hex()

        with open(RAW_HID_LOG_PATH, "a") as f:
            f.write(
                f"{ts:.6f} traffic={traffic_hex} len={length}\n"
                f"ASCII: {ascii_preview}\n"
                f"HEX:   {hex_preview}\n"
                f"---\n"
            )
    except Exception:
        # Logging must never interfere with transport
        pass

# ---------------------------------------------------------------------
# Receive callback registry
# ---------------------------------------------------------------------

_transport_recv_cb: Dict[int, Callable[[Any], None]] = {}

def transport_register_receive_callback(
    traffic: int,
    cb: Callable[[Any], None]
) -> None:
    _transport_recv_cb[traffic] = cb


# ---------------------------------------------------------------------
# HID device state
# ---------------------------------------------------------------------

hid_fd: Optional[int] = None
hid_present = threading.Event()
hid_write_lock = threading.Lock()

# ---------------------------------------------------------------------
# HID open / close
# ---------------------------------------------------------------------

def open_hid(path: str) -> None:
    global hid_fd
    if hid_fd is not None:
        raise RuntimeError("HID already open")

    hid_fd = os.open(path, os.O_RDWR)
    hid_present.set()
    logger.info("🚀 [transport] HID opened: %s", path)

def close_hid() -> None:
    global hid_fd
    if hid_fd is not None:
        os.close(hid_fd)
        hid_fd = None
        hid_present.clear()
        logger.info("🛑 [transport] HID closed")

# ---------------------------------------------------------------------
# Delimiting (send-side)
# ---------------------------------------------------------------------

def _delimit_for_send(traffic: int, payload: bytes) -> bytes:
    """
    Apply traffic-specific delimiting.

    REQUEST_RESPONSE:
      <STX=n> JSON <ETX>

    All other traffic:
      raw JSON bytes
    """
    if traffic == constants.TRAFFIC_REQUEST_RESPONSE:
        header = b"<STX=" + str(len(payload)).encode("ascii") + b">"
        return header + payload + ETX_SEQ

    return payload

# ---------------------------------------------------------------------
# Undelimiting (receive-side)
# ---------------------------------------------------------------------

def _undelimit_on_receive(traffic: int, msg: bytes) -> bytes:
    """
    Remove traffic-specific delimiters and return raw JSON bytes.

    Any malformed message is a fatal error.
    """
    if traffic == constants.TRAFFIC_REQUEST_RESPONSE:
        header, _, rest = msg.partition(b">")
        declared_len = int(header[len(STX_PREFIX):])
        body, _, _ = rest.partition(ETX_SEQ)
        return body[:declared_len]

    return msg

# ---------------------------------------------------------------------
# Public send API
# ---------------------------------------------------------------------

def transport_send(traffic: int, payload: Payload) -> None:
    """
    Send ONE complete semantic message.

    The object is JSON-encoded here, then passed through
    transport-level delimiting and fragmentation.
    """

    # Semantic → JSON → bytes
    raw = payload_to_json_bytes(payload)

    _snoop("←", raw)

    framed = _delimit_for_send(traffic, raw)
    _fragment_and_send(framed, traffic)

# ---------------------------------------------------------------------
# Fragmentation (send-side)
# ---------------------------------------------------------------------

def _fragment_and_send(data: bytes, traffic: int) -> None:
    """
    Fragment a fully delimited message into 64-byte HID packets.

    First packet:
      [traffic][payload...]

    Subsequent packets:
      [payload...]

    Padding with zero bytes defines message termination.
    """
    global hid_fd

    if not hid_present.is_set():
        raise RuntimeError("Teensy HID not present")

    data = bytes([traffic]) + data
    offset = 0

    with hid_write_lock:
        while offset < len(data):
            chunk = data[offset:offset + HID_PACKET_SIZE]
            if len(chunk) < HID_PACKET_SIZE:
                chunk += b"\x00" * (HID_PACKET_SIZE - len(chunk))

            try:
                os.write(hid_fd, chunk)
            except OSError as e:
                if e.errno in (5, 19):  # EIO / ENODEV
                    hid_present.clear()
                    try:
                        os.close(hid_fd)
                    except Exception:
                        pass
                    hid_fd = None
                    raise RuntimeError("⚠️ [transport] Teensy disconnected") from e
                raise

            offset += HID_PACKET_SIZE

# ---------------------------------------------------------------------
# Reader loop (single RX authority)
# ---------------------------------------------------------------------

def reader_loop() -> None:
    """
    Blocking reader loop.

    Owns:
      • packet reassembly
      • traffic extraction
      • padding-based message termination
      • undelimiting
      • dispatch
    """
    try:
        global hid_fd

        if hid_fd is None:
            raise RuntimeError("⚠️ [transport] reader_loop started with HID not open")

        rx_buf = bytearray()
        traffic: Optional[int] = None

        while True:
            try:
                pkt = os.read(hid_fd, HID_PACKET_SIZE)
            except OSError as e:
                if e.errno in (5, 19):
                    logger.warning("⚠️ [transport] hidraw vanished during read")
                    hid_present.clear()
                    try:
                        os.close(hid_fd)
                    except Exception:
                        pass
                    hid_fd = None
                    return
                raise

            if not pkt:
                continue

            # First packet: extract traffic byte
            if traffic is None:
                traffic = pkt[0]
                pkt = pkt[1:]

            stripped = pkt.rstrip(b"\x00")
            has_padding = len(stripped) < len(pkt)

            if stripped:
                rx_buf.extend(stripped)

            if has_padding:
                message = bytes(rx_buf)
                rx_buf.clear()

                # 🔴 FORENSIC TAP — log before any processing
                _log_raw_hid(traffic, message)

                _dispatch_complete_message(traffic, message)
                traffic = None
    except Exception:
        logging.exception("💥 [transport] reader_loop fatal exception")

# ---------------------------------------------------------------------
# Dispatch
# ---------------------------------------------------------------------

def _dispatch_complete_message(traffic: int, message: bytes) -> None:

    undelimited = _undelimit_on_receive(traffic, message)
    _snoop("→", undelimited)

    if traffic == constants.TRAFFIC_REQUEST_RESPONSE:
        payload = json.loads(undelimited.decode("utf-8"))
    else:
        payload = undelimited

    cb = _transport_recv_cb[traffic]

    cb(payload)

# ---------------------------------------------------------------------
# Forensic snooping
# ---------------------------------------------------------------------

def _snoop(direction: str, payload: bytes) -> None:
    with open(SERIAL_SNOOP_PATH, "a") as f:
        f.write(f"{time.time():.6f} {direction} {payload.decode('utf-8', errors='replace')}\n")

def transport_init() -> None:
    open_hid(constants.TEENSY_HIDRAW_PATH)
    try:

        threading.Thread(target=reader_loop, daemon=True, name="transport-reader").start()

    except Exception:
        logging.exception("💥 [transport] teensy_listener fatal crash")