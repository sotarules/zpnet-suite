"""
ZPNet Teensy Listener — HIDRAW Transport Authority + Deterministic RPC

HID-only implementation.
Exactly one hidraw reader.
Explicit channel separation:
  • 0xD0 → debug (written to log file)
  • 0x00 → framed transport (<STX=N> ... <ETX>)

Preserves all semantics of the serial-based listener.

Author: The Mule + GPT
"""

import json
import logging
import os
import socket
import threading
import time
import itertools
import queue
from queue import Queue

from typing import Dict, Optional

from zpnet.shared.logger import setup_logging
from zpnet.shared.events import create_event
from zpnet.shared.constants import TEENSY_HIDRAW_PATH

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

HID_PACKET_SIZE = 64
DEBUG_MARKER = 0xD0

RPC_SOCKET_PATH = "/tmp/zpnet_teensy_rt.sock"
RPC_BACKLOG = 8

EVENT_DESPOOL_INTERVAL_S = 10.0

STX_PREFIX = b"<STX="
ETX_SEQ = b"<ETX>"

DEBUG_LOG_PATH =    "/home/mule/zpnet/logs/zpnet-debug.log"
SERIAL_SNOOP_PATH = "/home/mule/zpnet/logs/zpnet-teensy-listener-snoop.log"

# ---------------------------------------------------------------------
# Global state
# ---------------------------------------------------------------------

# req_id → Queue[dict]
pending_replies: Dict[int, Queue] = {}
req_id_counter = itertools.count(1)

state_lock = threading.Lock()

hid_fd: Optional[int] = None
hid_write_lock = threading.Lock()

# Debug log handle
debug_log_fh = None

# ---------------------------------------------------------------------
# Debug sink
# ---------------------------------------------------------------------

def open_debug_log() -> None:
    global debug_log_fh
    os.makedirs(os.path.dirname(DEBUG_LOG_PATH), exist_ok=True)
    debug_log_fh = open(DEBUG_LOG_PATH, "w", buffering=1)
    logging.info("[teensy_listener] debug log opened at %s", DEBUG_LOG_PATH)

def write_debug_line(text: str) -> None:
    if debug_log_fh:
        debug_log_fh.write(text + "\n")

# ---------------------------------------------------------------------
# HID helpers
# ---------------------------------------------------------------------

def open_hid_blocking() -> int:
    """
    Open hidraw device, blocking until available.
    """
    while True:
        try:
            fd = os.open(TEENSY_HIDRAW_PATH, os.O_RDWR)
            logging.info("[teensy_listener] hidraw connected")
            return fd
        except Exception as e:
            logging.warning(
                "[teensy_listener] waiting for hidraw (%s)",
                e,
            )
            time.sleep(1.0)

def hid_write(data: bytes) -> None:
    """
    Write framed bytes to hidraw.

    Each write is exactly:
      [1-byte report ID][64-byte payload]

    Payload is zero-padded as required by HID.
    """
    if not data:
        return

    offset = 0
    with hid_write_lock:
        while offset < len(data):
            # Take up to 64 bytes of REAL data
            chunk = data[offset: offset + HID_PACKET_SIZE]

            # Pad payload to exactly 64 bytes
            if len(chunk) < HID_PACKET_SIZE:
                chunk = chunk + b"\x00" * (HID_PACKET_SIZE - len(chunk))

            # Prepend HID report ID (transport = 0x00)
            pkt = b"\x00" + chunk

            # Write exactly 65 bytes
            os.write(hid_fd, pkt)

            # Advance by PAYLOAD width, not padded length
            offset += HID_PACKET_SIZE

# ---------------------------------------------------------------------
# Framed transport helpers
# ---------------------------------------------------------------------

def send_frame(payload: dict) -> None:
    """
    Frame and send a JSON payload to the Teensy.
    This is the ONLY outbound path.
    """
    raw = json.dumps(
        payload,
        separators=(",", ":"),
        ensure_ascii=False
    ).encode("utf-8")

    frame = (
        b"<STX=" +
        str(len(raw)).encode("ascii") +
        b">" +
        raw +
        ETX_SEQ
    )

    _snoop("←", raw)

    hid_write(frame)

# ---------------------------------------------------------------------
# Transport ingestion (bytewise)
# ---------------------------------------------------------------------

_transport_buf = bytearray()

def transport_ingest(data: bytes) -> None:
    """
    Bytewise framed transport ingestion.
    """
    _transport_buf.extend(data)

    while True:
        stx = _transport_buf.find(b"<")
        if stx == -1:
            _transport_buf.clear()
            return

        if not _transport_buf.startswith(STX_PREFIX, stx):
            del _transport_buf[:stx + 1]
            continue

        gt = _transport_buf.find(b">", stx)
        if gt == -1:
            return

        try:
            length = int(_transport_buf[stx + len(STX_PREFIX):gt])
        except ValueError:
            del _transport_buf[:gt + 1]
            continue

        payload_start = gt + 1
        payload_end = payload_start + length
        etx_end = payload_end + len(ETX_SEQ)

        if len(_transport_buf) < etx_end:
            return

        if _transport_buf[payload_end:etx_end] != ETX_SEQ:
            del _transport_buf[:payload_end]
            continue

        payload = bytes(_transport_buf[payload_start:payload_end])
        del _transport_buf[:etx_end]

        _snoop("→", payload)

        try:
            msg = json.loads(payload.decode("utf-8"))
        except json.JSONDecodeError:
            continue

        route_message(msg)

# ---------------------------------------------------------------------
# Message routing
# ---------------------------------------------------------------------

def route_message(msg: dict) -> None:
    # ---------------- RPC replies ----------------
    if "req_id" in msg:
        req_id = msg["req_id"]
        with state_lock:
            q = pending_replies.pop(req_id, None)
        if q:
            q.put(msg)
        return

    # ---------------- Event drain ----------------
    if msg.get("control") == "EVENTS_BEGIN":
        return

    if "event_type" in msg:
        payload = msg.copy()
        event_type = payload.pop("event_type")
        create_event(event_type, payload)

# ---------------------------------------------------------------------
# HID reader (sole owner)
# ---------------------------------------------------------------------

def hid_reader() -> None:
    global hid_fd

    last_drain = 0.0

    # Persistent debug accumulator
    debug_buf = bytearray()

    while True:
        try:
            pkt = os.read(hid_fd, HID_PACKET_SIZE + 1)
            if not pkt:
                continue

            report_id = pkt[0]
            data = pkt[1:]

            # ---------------- DEBUG CHANNEL ----------------
            if report_id == DEBUG_MARKER:

                # Strip only right-side zero padding
                payload = data.rstrip(b"\0")

                # Accumulate meaningful bytes
                debug_buf.extend(payload)

                # If this packet had padding, it ends the message
                if len(payload) < (HID_PACKET_SIZE - 1):
                    if debug_buf:
                        text = debug_buf.decode("utf-8", errors="replace")
                        write_debug_line(text)
                    debug_buf.clear()

                continue

            # ---------------- TRANSPORT CHANNEL ----------------
            transport_ingest(data)

            now = time.time()
            if now - last_drain >= EVENT_DESPOOL_INTERVAL_S:
                send_frame({"cmd": "EVENTS.GET"})
                last_drain = now

        except Exception as e:
            logging.error(
                "[teensy_listener] hidraw error (%s) — reopening",
                e,
            )
            try:
                os.close(hid_fd)
            except Exception:
                pass
            hid_fd = open_hid_blocking()
            last_drain = 0.0


# ---------------------------------------------------------------------
# RPC handler
# ---------------------------------------------------------------------

def handle_client(conn: socket.socket) -> None:
    req_id = 0
    q: Queue[dict] | None = None

    try:
        # Buffered read-until-JSON (stream-safe)
        buf = bytearray()
        while True:
            chunk = conn.recv(4096)
            if not chunk:
                break
            buf.extend(chunk)
            try:
                req = json.loads(buf.decode("utf-8"))
                break
            except json.JSONDecodeError:
                continue

        req_id = next(req_id_counter)
        req["req_id"] = req_id

        q = Queue(maxsize=1)

        with state_lock:
            pending_replies[req_id] = q

        send_frame(req)

        reply = q.get(timeout=10.0)

        try:
            conn.sendall(
                json.dumps(reply, separators=(",", ":")).encode("utf-8")
            )
        except BrokenPipeError:
            logging.warning(
                "[teensy_listener] client disconnected before reply (req_id=%s)",
                req_id,
            )

    except queue.Empty:
        logging.warning(
            "[teensy_listener] RPC TIMEOUT req_id=%s",
            req_id,
        )
        with state_lock:
            pending_replies.pop(req_id, None)

        try:
            conn.sendall(
                json.dumps(
                    {"success": False, "message": "RPC timeout"},
                    separators=(",", ":")
                ).encode("utf-8")
            )
        except BrokenPipeError:
            pass

    except Exception as e:
        logging.exception(
            "[teensy_listener] RPC ERROR req_id=%s: %s",
            req_id,
            e,
        )
        with state_lock:
            if req_id:
                pending_replies.pop(req_id, None)

        try:
            conn.sendall(
                json.dumps(
                    {"success": False, "message": str(e)},
                    separators=(",", ":")
                ).encode("utf-8")
            )
        except BrokenPipeError:
            pass

    finally:
        try:
            conn.close()
        except Exception:
            pass

# ---------------------------------------------------------------------
# RPC server
# ---------------------------------------------------------------------

def rpc_server() -> None:
    if os.path.exists(RPC_SOCKET_PATH):
        os.unlink(RPC_SOCKET_PATH)

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.bind(RPC_SOCKET_PATH)
    os.chmod(RPC_SOCKET_PATH, 0o660)
    sock.listen(RPC_BACKLOG)

    logging.info(
        "🧭 [teensy_listener] RPC socket ready at %s",
        RPC_SOCKET_PATH,
    )

    while True:
        conn, _ = sock.accept()
        threading.Thread(
            target=handle_client,
            args=(conn,),
            daemon=True
        ).start()

# ---------------------------------------------------------------------
# Serial snoop
# ---------------------------------------------------------------------

def _snoop(direction: str, payload: bytes) -> None:
    if not SERIAL_SNOOP_PATH:
        return
    try:
        with open(SERIAL_SNOOP_PATH, "a") as f:
            f.write(
                f"{time.time():.6f} {direction} "
                f"{payload.decode('utf-8', errors='replace')}\n"
            )
    except Exception:
        pass

# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------

def run() -> None:
    global hid_fd

    setup_logging()
    open_debug_log()

    logging.info("🚀 [teensy_listener] starting HID transport authority")

    hid_fd = open_hid_blocking()

    threading.Thread(target=hid_reader, daemon=True).start()
    threading.Thread(target=rpc_server, daemon=True).start()

    while True:
        time.sleep(1)

# ---------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------

def bootstrap() -> None:
    run()
