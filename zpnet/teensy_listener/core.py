"""
ZPNet Teensy Listener — RawHID Transport Authority + Deterministic RPC

RawHID message semantics:
  • A message consists of one or more 64-byte frames
  • The final frame contains at least one \0 padding byte
  • Message bytes are rstripped(payload) concatenated across frames

This layer is transport-only and semantics-free.

Defensive stance:
  • USB transport may vanish (EIO/ENODEV) during flash/re-enumeration
  • RPC clients may disconnect at any time (BrokenPipe/ECONNRESET)
  • The listener must never die because a client disappeared
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
ASCII_LT     = ord("<")

RPC_SOCKET_PATH = "/tmp/zpnet_teensy_rt.sock"
RPC_BACKLOG = 8

EVENT_DESPOOL_INTERVAL_S = 10.0

STX_PREFIX = b"<STX="
ETX_SEQ    = b"<ETX>"

DEBUG_LOG_PATH = "/home/mule/zpnet/logs/zpnet-debug.log"
SERIAL_SNOOP_PATH = "/home/mule/zpnet/logs/zpnet-teensy-listener-snoop.log"


# ---------------------------------------------------------------------
# Global state
# ---------------------------------------------------------------------

pending_replies: Dict[int, Queue] = {}
req_id_counter = itertools.count(1)

state_lock = threading.Lock()

hid_fd: Optional[int] = None
hid_present = threading.Event()
hid_write_lock = threading.Lock()

debug_log_fh = None


# ---------------------------------------------------------------------
# Debug sink
# ---------------------------------------------------------------------

def open_debug_log() -> None:
    global debug_log_fh
    os.makedirs(os.path.dirname(DEBUG_LOG_PATH), exist_ok=True)
    debug_log_fh = open(DEBUG_LOG_PATH, "w", buffering=1)

def write_debug_line(text: str) -> None:
    if debug_log_fh:
        debug_log_fh.write(text + "\n")


# ---------------------------------------------------------------------
# RPC socket send helper (client may disappear)
# ---------------------------------------------------------------------

def _safe_sendall(conn: socket.socket, payload: bytes) -> None:
    """
    Best-effort send. Clients can disconnect anytime; that is not an error
    worthy of killing the service or spamming tracebacks.
    """
    try:
        conn.sendall(payload)
    except (BrokenPipeError, ConnectionResetError):
        # client closed early; ignore
        return
    except OSError as e:
        # Treat common "client went away" cases as ignorable.
        # EPIPE=32, ECONNRESET=104, ENOTCONN=107 (varies by platform)
        if e.errno in (32, 104, 107):
            return
        raise


# ---------------------------------------------------------------------
# HID helpers (AUTHORITATIVE TRANSPORT BOUNDARY)
# ---------------------------------------------------------------------

def open_hid_blocking() -> int:
    global hid_present

    while True:
        try:
            fd = os.open(TEENSY_HIDRAW_PATH, os.O_RDWR)
            logging.info("[teensy_listener] hidraw connected")
            hid_present.set()
            return fd
        except Exception:
            hid_present.clear()
            time.sleep(1.0)

def hid_write(data: bytes) -> None:
    global hid_fd

    if not data:
        return

    if not hid_present.is_set():
        raise RuntimeError("Teensy HID not present")

    offset = 0
    with hid_write_lock:
        while offset < len(data):
            chunk = data[offset: offset + HID_PACKET_SIZE]
            if len(chunk) < HID_PACKET_SIZE:
                chunk += b"\x00" * (HID_PACKET_SIZE - len(chunk))
            try:
                os.write(hid_fd, chunk)
            except OSError as e:
                if e.errno in (5, 19):  # EIO or ENODEV
                    logging.warning(
                        "[teensy_listener] hidraw vanished during write (errno=%d)",
                        e.errno
                    )
                    hid_present.clear()
                    try:
                        os.close(hid_fd)
                    except Exception:
                        pass
                    hid_fd = None
                    raise RuntimeError("Teensy disconnected")
                raise

            offset += HID_PACKET_SIZE


# ---------------------------------------------------------------------
# RawHID message reassembly
# ---------------------------------------------------------------------

def read_rawhid_message() -> Optional[bytes]:
    global hid_fd

    buf = bytearray()

    while True:
        try:
            pkt = os.read(hid_fd, HID_PACKET_SIZE)
        except OSError as e:
            if e.errno in (5, 19):  # EIO or ENODEV
                logging.warning(
                    "[teensy_listener] hidraw vanished during read (errno=%d)",
                    e.errno
                )
                hid_present.clear()
                try:
                    os.close(hid_fd)
                except Exception:
                    pass
                hid_fd = None
                return None
            raise

        if not pkt:
            continue

        stripped = pkt.rstrip(b"\x00")
        has_padding = len(stripped) < len(pkt)
        buf.extend(stripped)

        if has_padding:
            return bytes(buf)


# ---------------------------------------------------------------------
# Transport helpers
# ---------------------------------------------------------------------

def send_frame(payload: dict) -> None:
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
# Process a response message from Teensy
# ---------------------------------------------------------------------

def process_response(msg: bytes) -> None:
    if not msg.startswith(STX_PREFIX):
        return

    gt = msg.find(b">")
    if gt == -1:
        return

    length = int(msg[len(STX_PREFIX):gt])

    payload_start = gt + 1
    payload_end   = payload_start + length
    etx_end       = payload_end + len(ETX_SEQ)

    if msg[payload_end:etx_end] != ETX_SEQ:
        return

    payload = msg[payload_start:payload_end]
    _snoop("→", payload)

    parsed = json.loads(payload.decode("utf-8"))

    route_message(parsed)


# ---------------------------------------------------------------------
# Message routing
# ---------------------------------------------------------------------

def route_message(msg: dict) -> None:
    if "req_id" in msg:
        with state_lock:
            q = pending_replies.pop(msg["req_id"], None)
        if q:
            q.put(msg)
        return

    if msg.get("control") == "EVENTS_BEGIN":
        return

    if "event_type" in msg:
        payload = msg.copy()
        create_event(payload.pop("event_type"), payload)


# ---------------------------------------------------------------------
# HID reader (reconnect-aware, single authority)
# ---------------------------------------------------------------------

def hid_reader() -> None:

    try:
        global hid_fd
        last_drain = 0.0

        while True:
            if not hid_present.is_set():
                logging.info("[teensy_listener] waiting for Teensy HID...")
                hid_fd = open_hid_blocking()
                continue

            msg = read_rawhid_message()
            if not msg:
                continue

            first = msg[0]

            if first == DEBUG_MARKER:
                write_debug_line(
                    msg[1:].decode("utf-8", errors="replace")
                )
                continue

            if first == ASCII_LT:
                process_response(msg)

            now = time.time()
            if now - last_drain >= EVENT_DESPOOL_INTERVAL_S:
                try:
                    send_frame({"cmd": "EVENTS.GET"})
                except RuntimeError:
                    pass
                last_drain = now

    except Exception:
        logging.exception("[hid_reader] teensy_listener unexpected exception")

# ---------------------------------------------------------------------
# RPC handler
# ---------------------------------------------------------------------
def handle_client(conn: socket.socket) -> None:
    req_id = 0
    q: Optional[Queue] = None
    buf = bytearray()
    while True:
        chunk = conn.recv(4096)
        if not chunk:
            return
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
    _safe_sendall(conn, json.dumps(reply, separators=(",", ":")).encode("utf-8"))
    conn.close()

# ---------------------------------------------------------------------
# RPC server
# ---------------------------------------------------------------------

def rpc_server() -> None:

    try:
        if os.path.exists(RPC_SOCKET_PATH):
            os.unlink(RPC_SOCKET_PATH)

        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.bind(RPC_SOCKET_PATH)
        os.chmod(RPC_SOCKET_PATH, 0o660)
        sock.listen(RPC_BACKLOG)

        while True:
            conn, _ = sock.accept()
            threading.Thread(
                target=handle_client,
                args=(conn,),
                daemon=True
            ).start()

    except Exception:
        logging.exception("[rpc_server] teensy_listener unexpected exception")

# ---------------------------------------------------------------------
# Serial snoop
# ---------------------------------------------------------------------

def _snoop(direction: str, payload: bytes) -> None:
    with open(SERIAL_SNOOP_PATH, "a") as f:
        f.write(f"{time.time():.6f} {direction} {payload.decode('utf-8', errors='replace')}\n")

# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------

def run() -> None:
    global hid_fd
    setup_logging()
    open_debug_log()
    try:
        hid_fd = open_hid_blocking()
        threading.Thread(target=hid_reader, daemon=True).start()
        threading.Thread(target=rpc_server, daemon=True).start()
        while True:
            time.sleep(1)
    except Exception:
        logging.exception("[main] teensy_listener fatal crash")

def bootstrap() -> None:
    run()
