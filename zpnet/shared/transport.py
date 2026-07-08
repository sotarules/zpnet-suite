"""
transport.py — ZPNet Host-Side Transport Layer (Pi)

This module is the host-side byte ↔ meaning boundary. It is intentionally
symmetric with transport.cpp on the Teensy.

Wire protocol:

    [traffic byte] <STX=json_len> JSON <ETX>

Serial-only doctrine:
  • USB CDC serial is the only physical path.
  • HID support is retired.
  • The length is authoritative; ETX is retained as a redundant safety check.

Responsibilities:
  • Own physical serial I/O.
  • Delimit / undelimit semantic messages.
  • Demultiplex by traffic byte.
  • Deliver decoded messages via registered callbacks.

Design philosophy:
  • Transport is boring and correct.
  • Meaning crosses this boundary exactly once.
  • Receive is adversarial and callback-driven.
  • Transport never dies because of data.
"""

from __future__ import annotations

import json
import logging
import os
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
# Serial device path
# ---------------------------------------------------------------------

TEENSY_SERIAL_PATH = getattr(constants, "TEENSY_SERIAL_PORT", "/dev/zpnet-teensy-serial")

# ---------------------------------------------------------------------
# Constants (must match Teensy)
# ---------------------------------------------------------------------

TRANSPORT_MAX_MESSAGE = 32 * 1024
FRAME_SLACK = 64
RX_BUF_MAX = TRANSPORT_MAX_MESSAGE + FRAME_SLACK

STX_PREFIX = b"<STX="
ETX_SEQ = b"<ETX>"
ETX_LEN = len(ETX_SEQ)

_VALID_TRAFFIC = {
    TRAFFIC_DEBUG,
    TRAFFIC_REQUEST_RESPONSE,
    TRAFFIC_PUBLISH_SUBSCRIBE,
}

RAW_TRANSPORT_LOG_PATH = "/home/mule/zpnet/logs/zpnet-transport.log"
RAW_TRANSPORT_LOG_ENABLED = True

# ---------------------------------------------------------------------
# Retry / attach policy
# ---------------------------------------------------------------------

_RETRY_MIN_SLEEP_S = 0.02
_RETRY_MAX_SLEEP_S = 0.50

# USB CDC attach is not atomic from the application protocol's point of view:
# opening the serial device can coincide with Teensy reboot/setup traffic and
# with stale bytes buffered by the host driver.  Do not allow the first
# application command to race that window.  Drain and settle before declaring
# the transport ready for Pi->Teensy writes.
_ATTACH_DRAIN_S = 1.00
_READY_SEND_TIMEOUT_S = 10.0

# ---------------------------------------------------------------------
# Physical transport state
# ---------------------------------------------------------------------

_ser: Optional["serial.Serial"] = None
_tx_lock = threading.Lock()

_transport_stop = threading.Event()
_transport_supervisor_started = False
_transport_ready = threading.Event()
_transport_ready_lock = threading.Lock()
_transport_ready_generation = 0
_transport_attach_count = 0
_transport_attach_drain_bytes = 0
_transport_last_ready_ts: Optional[float] = None

# ---------------------------------------------------------------------
# RX assembly state
# ---------------------------------------------------------------------

rx_buf = bytearray()
rx_have_traffic = False
rx_traffic: Optional[int] = None

# ---------------------------------------------------------------------
# Open / close primitives
# ---------------------------------------------------------------------

def _open_serial(path: str) -> None:
    global _ser

    if _ser is not None:
        return

    kwargs = {
        "port": path,
        "baudrate": getattr(constants, "TEENSY_BAUDRATE", 115200),
        "timeout": None,
    }

    # POSIX pyserial supports exclusive=True.  It gives us a cheap proof that
    # zpnet-pubsub is the only process physically touching the serial device.
    try:
        _ser = serial.Serial(**kwargs, exclusive=True)
    except TypeError:
        _ser = serial.Serial(**kwargs)

    logging.info("[transport] SERIAL device opened: %s", path)


def _transport_mark_not_ready() -> None:
    _transport_ready.clear()


def _transport_mark_ready() -> None:
    global _transport_ready_generation, _transport_last_ready_ts

    with _transport_ready_lock:
        _transport_ready_generation += 1
        _transport_last_ready_ts = time.time()
        _transport_ready.set()


def _close_serial() -> None:
    global _ser

    _transport_mark_not_ready()

    ser = _ser
    if ser is None:
        return

    try:
        ser.close()
    except Exception:
        pass

    _ser = None
    logging.info("[transport] SERIAL device closed")


def _reset_host_rx_state() -> None:
    _rx_reset()


def _serial_reset_buffers() -> None:
    ser = _ser
    if ser is None:
        return

    for name in ("reset_input_buffer", "reset_output_buffer"):
        fn = getattr(ser, name, None)
        if fn is None:
            continue
        try:
            fn()
        except Exception:
            logging.debug("[transport] %s failed during attach", name, exc_info=True)


def _drain_attach_noise() -> int:
    global _transport_attach_count, _transport_attach_drain_bytes

    ser = _ser
    if ser is None:
        return 0

    deadline = time.monotonic() + _ATTACH_DRAIN_S
    drained = 0

    while time.monotonic() < deadline:
        try:
            waiting = getattr(ser, "in_waiting", 0)
        except Exception:
            waiting = 0

        if waiting:
            chunk = ser.read(waiting)
            drained += len(chunk)
            continue

        time.sleep(0.02)

    try:
        waiting = getattr(ser, "in_waiting", 0)
        if waiting:
            drained += len(ser.read(waiting))
        reset_input = getattr(ser, "reset_input_buffer", None)
        if reset_input is not None:
            reset_input()
    except Exception:
        logging.debug("[transport] final attach drain failed", exc_info=True)

    _transport_attach_count += 1
    _transport_attach_drain_bytes += drained
    if drained:
        logging.info("[transport] drained %d startup attach bytes before ready", drained)
    return drained

# ---------------------------------------------------------------------
# Raw wire logging
# ---------------------------------------------------------------------

def render_bytes_lossless(data: bytes) -> str:
    out = []
    for b in data:
        if 32 <= b <= 126:
            out.append(chr(b))
        else:
            out.append(f"\\x{b:02X}")
    return "".join(out)


def _log_wire(direction: str, wire: bytes) -> None:
    if not RAW_TRANSPORT_LOG_ENABLED:
        return

    traffic = wire[0] if wire else 0

    try:
        os.makedirs(os.path.dirname(RAW_TRANSPORT_LOG_PATH), exist_ok=True)
        with open(RAW_TRANSPORT_LOG_PATH, "a") as f:
            f.write(
                f"{direction} 0x{traffic:02X} len={len(wire)} "
                f"{render_bytes_lossless(wire)}\n"
            )
    except Exception as e:
        logging.warning("⚠️ [transport] raw transport log failed: %r", e)

# ---------------------------------------------------------------------
# Receive callback registry
# ---------------------------------------------------------------------

_transport_recv_cb: Dict[int, Callable[[Payload], None]] = {}


def transport_register_receive_callback(traffic: int, cb: Callable[[Payload], None]) -> None:
    _transport_recv_cb[traffic] = cb

# ---------------------------------------------------------------------
# Framing helpers
# ---------------------------------------------------------------------

def _frame_payload(payload: bytes) -> bytes:
    return b"<STX=" + str(len(payload)).encode("ascii") + b">" + payload + ETX_SEQ


def _make_wire_message(traffic: int, payload: bytes) -> bytes:
    return bytes([traffic]) + _frame_payload(payload)


def _payload_from_frame(frame: bytes) -> bytes:
    header_end = frame.index(b">")
    declared_len = int(frame[len(STX_PREFIX):header_end])
    json_start = header_end + 1
    return frame[json_start:json_start + declared_len]


def _validate_frame(frame: bytes) -> bool:
    header_end = frame.find(b">")
    if (
        not frame.startswith(STX_PREFIX)
        or not frame.endswith(ETX_SEQ)
        or header_end == -1
    ):
        logging.info("🧩 transport detected incorrectly formatted frame: %r", frame)
        return False

    declared_raw = frame[len(STX_PREFIX):header_end]
    if not declared_raw.isdigit():
        logging.info("🧩 transport detected incorrectly formatted frame: %r", frame)
        return False

    declared_len = int(declared_raw)
    actual_len = len(frame) - (header_end + 1) - ETX_LEN
    if actual_len != declared_len:
        logging.info("🧩 transport detected incorrectly formatted frame: %r", frame)
        return False

    return True

# ---------------------------------------------------------------------
# Dispatch
# ---------------------------------------------------------------------

def _dispatch_frame(traffic: int, frame: bytes) -> None:
    if not _validate_frame(frame):
        return

    raw = _payload_from_frame(frame)
    payload = json.loads(raw.decode("utf-8"))

    cb = _transport_recv_cb.get(traffic)
    if cb is None:
        logging.debug("[transport] no receive callback for traffic=0x%02X — frame discarded", traffic)
        return

    try:
        cb(payload)
    except Exception:
        logging.exception("[transport] receive callback failed traffic=0x%02X — frame discarded", traffic)

# ---------------------------------------------------------------------
# Physical primitives
# ---------------------------------------------------------------------

def _serial_write_all(data: bytes) -> None:
    offset = 0

    while offset < len(data):
        ser = _ser
        if ser is None:
            raise RuntimeError("SERIAL transport is not open")

        n = ser.write(data[offset:])
        ser.flush()

        if n is None:
            n = len(data) - offset

        if n <= 0:
            raise RuntimeError("SERIAL write made no progress")

        offset += n


def _serial_read_some() -> bytes:
    ser = _ser
    if ser is None:
        return b""

    waiting = getattr(ser, "in_waiting", 0)
    return ser.read(waiting if waiting else 1)

# ---------------------------------------------------------------------
# TX path
# ---------------------------------------------------------------------

def _serial_tx_send(wire: bytes) -> None:
    _serial_write_all(wire)
    _log_wire("Pi -> Teensy", wire)

# ---------------------------------------------------------------------
# RX path
# ---------------------------------------------------------------------

def _rx_begin(traffic: int) -> None:
    global rx_have_traffic, rx_traffic

    rx_traffic = traffic
    rx_have_traffic = True
    rx_buf.clear()


def _rx_reset() -> None:
    global rx_have_traffic, rx_traffic

    rx_buf.clear()
    rx_have_traffic = False
    rx_traffic = None


def _rx_stx_accepts(b: int) -> bool:
    if len(rx_buf) >= len(STX_PREFIX):
        return True
    return STX_PREFIX.startswith(bytes(rx_buf) + bytes([b]))


def _rx_header_complete() -> bool:
    return len(rx_buf) > len(STX_PREFIX) and b">" in rx_buf[len(STX_PREFIX):]


def _rx_should_resync(b: int) -> bool:
    if b not in _VALID_TRAFFIC:
        return False

    if len(rx_buf) == 0:
        return True

    if _rx_header_complete():
        return False

    if len(rx_buf) < len(STX_PREFIX):
        return not _rx_stx_accepts(b)

    # We have <STX= but not the closing '>' yet. A traffic byte cannot be a
    # legal decimal length byte, so it is a fresh frame boundary.
    return True


def _dispatch_if_complete() -> None:
    if not rx_have_traffic or rx_traffic is None:
        rx_buf.clear()
        return

    if len(rx_buf) < len(STX_PREFIX):
        return

    if not rx_buf.startswith(STX_PREFIX):
        logging.info("🧩 transport detected incorrectly formatted frame: %r", bytes(rx_buf))
        _rx_reset()
        return

    header_end = rx_buf.find(b">")
    if header_end == -1:
        return

    declared_raw = bytes(rx_buf[len(STX_PREFIX):header_end])
    if not declared_raw.isdigit():
        logging.info("🧩 transport detected incorrectly formatted frame: %r", bytes(rx_buf))
        _rx_reset()
        return

    declared_len = int(declared_raw)
    required_total = header_end + 1 + declared_len + ETX_LEN

    if required_total > RX_BUF_MAX:
        logging.info("🧩 transport declared oversized frame: %r", bytes(rx_buf))
        _rx_reset()
        return

    if len(rx_buf) < required_total:
        return

    frame = bytes(rx_buf[:required_total])
    traffic = rx_traffic

    if not _validate_frame(frame):
        _rx_reset()
        return

    _log_wire("Teensy -> Pi", bytes([traffic]) + frame)
    _dispatch_frame(traffic, frame)
    _rx_reset()


def _serial_rx_loop() -> None:
    _rx_reset()

    while True:
        data = _serial_read_some()
        for b in data:
            if not rx_have_traffic:
                if b in _VALID_TRAFFIC:
                    _rx_begin(b)
                continue

            # Recover from orphan traffic bytes / partial prior frames.  Once
            # STX is established, valid traffic-looking bytes are payload bytes;
            # before STX is established, they are fresh frame boundaries.
            if _rx_should_resync(b):
                logging.debug(
                    "[transport] RX resync on traffic=0x%02X discarded_prefix=%r",
                    b,
                    bytes(rx_buf),
                )
                _rx_begin(b)
                continue

            rx_buf.append(b)

            if len(rx_buf) > RX_BUF_MAX:
                logging.info("🧩 transport receive buffer overflow: len=%d", len(rx_buf))
                _rx_reset()
                continue

            if len(rx_buf) <= len(STX_PREFIX) and not STX_PREFIX.startswith(bytes(rx_buf)):
                logging.info("🧩 transport detected incorrectly formatted frame: %r", bytes(rx_buf))
                _rx_reset()
                continue

            _dispatch_if_complete()

# ---------------------------------------------------------------------
# Supervisor
# ---------------------------------------------------------------------

def _supervisor_loop() -> None:
    announced = False
    backoff = _RETRY_MIN_SLEEP_S

    while not _transport_stop.is_set():
        try:
            _transport_mark_not_ready()
            _open_serial(TEENSY_SERIAL_PATH)
            _reset_host_rx_state()
            _serial_reset_buffers()
            _drain_attach_noise()
            if announced:
                logging.info("✅ [transport] SERIAL device available — RX loop starting")
                announced = False
            logging.info("✅ [transport] SERIAL ready — RX loop starting")
            _transport_mark_ready()
            backoff = _RETRY_MIN_SLEEP_S
            _serial_rx_loop()

        except Exception as e:
            _transport_mark_not_ready()
            _close_serial()

            if not announced:
                logging.info(
                    "⏳ [transport] SERIAL device not ready — supervisor retrying silently: %r",
                    e,
                )
                announced = True
            else:
                logging.debug("[transport] SERIAL supervisor retry after exception: %r", e)

            time.sleep(backoff)
            backoff = min(backoff * 2.0, _RETRY_MAX_SLEEP_S)

# ---------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------

def transport_wait_ready(timeout_s: Optional[float] = _READY_SEND_TIMEOUT_S) -> bool:
    if not _transport_supervisor_started:
        return False
    return _transport_ready.wait(timeout=timeout_s)


def transport_diagnostics() -> dict:
    with _transport_ready_lock:
        return {
            "ready": _transport_ready.is_set(),
            "ready_generation": _transport_ready_generation,
            "attach_count": _transport_attach_count,
            "attach_drain_bytes": _transport_attach_drain_bytes,
            "last_ready_ts": _transport_last_ready_ts,
        }


def transport_send(traffic: int, payload: Payload) -> None:
    if not _transport_supervisor_started:
        raise RuntimeError("transport not initialized")

    if not transport_wait_ready(_READY_SEND_TIMEOUT_S):
        raise RuntimeError("SERIAL transport is not ready")

    raw = payload_to_json_bytes(payload)
    wire = _make_wire_message(traffic, raw)

    with _tx_lock:
        _serial_tx_send(wire)


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
    _transport_mark_not_ready()
    _close_serial()


def transport_rx_snapshot() -> dict:
    snap = transport_diagnostics()
    snap.update({
        "rx_buf_len": len(rx_buf),
        "rx_buf_bytes": bytes(rx_buf),
        "rx_traffic": rx_traffic if rx_have_traffic else None,
        "rx_have_traffic": rx_have_traffic,
    })
    return snap
