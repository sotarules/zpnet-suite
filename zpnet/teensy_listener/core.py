"""
ZPNet Teensy Listener — RawHID Transport Authority + Deterministic RPC

RawHID message semantics:
  • A message consists of one or more 64-byte frames
  • The final frame contains at least one \\0 padding byte
  • Message bytes are rstripped(payload) concatenated across frames

This layer is transport-only and semantics-free.

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
# HID helpers
# ---------------------------------------------------------------------

def open_hid_blocking() -> int:
    while True:
        try:
            fd = os.open(TEENSY_HIDRAW_PATH, os.O_RDWR)
            logging.info("[teensy_listener] hidraw connected")
            return fd
        except Exception:
            time.sleep(1.0)

def hid_write(data: bytes) -> None:
    if not data:
        return

    offset = 0
    with hid_write_lock:
        while offset < len(data):
            chunk = data[offset: offset + HID_PACKET_SIZE]
            if len(chunk) < HID_PACKET_SIZE:
                chunk += b"\x00" * (HID_PACKET_SIZE - len(chunk))
            os.write(hid_fd, chunk)
            offset += HID_PACKET_SIZE

# ---------------------------------------------------------------------
# RawHID message reassembly (NEW CORE)
# ---------------------------------------------------------------------

def read_rawhid_message() -> bytes:
    """
    Read exactly one RawHID message.

    A message ends when a frame contains any \\0 padding.
    """
    buf = bytearray()

    while True:
        pkt = os.read(hid_fd, HID_PACKET_SIZE)
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
    """
    Ingest exactly one framed transport message.
    The message is guaranteed complete by the RawHID layer.
    """

    # Expect leading <STX=...>
    if not msg.startswith(STX_PREFIX):
        return

    gt = msg.find(b">")
    if gt == -1:
        return

    try:
        length = int(msg[len(STX_PREFIX):gt])
    except ValueError:
        return

    payload_start = gt + 1
    payload_end   = payload_start + length
    etx_end       = payload_end + len(ETX_SEQ)

    if msg[payload_end:etx_end] != ETX_SEQ:
        return

    payload = msg[payload_start:payload_end]

    _snoop("→", payload)

    try:
        parsed = json.loads(payload.decode("utf-8"))
    except json.JSONDecodeError:
        return

    route_message(parsed)


# ---------------------------------------------------------------------
# Message routing (unchanged)
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
# HID reader (NOW CLEAN)
# ---------------------------------------------------------------------

def hid_reader() -> None:
    last_drain = 0.0

    while True:
        msg = read_rawhid_message()
        if not msg:
            continue

        logging.info("[teensy_listener] received rawhid message: %s",
                     msg.decode("utf-8", errors="replace"))

        first = msg[0]

        # DEBUG
        if first == DEBUG_MARKER:
            write_debug_line(
                msg[1:].decode("utf-8", errors="replace")
            )
            continue

        # TRANSPORT
        if first == ASCII_LT:
            logging.info("[teensy_listener] received transport message: %s",
                         msg.decode("utf-8", errors="replace"))
            process_response(msg)

        now = time.time()
        if now - last_drain >= EVENT_DESPOOL_INTERVAL_S:
            send_frame({"cmd": "EVENTS.GET"})
            last_drain = now

# ---------------------------------------------------------------------
# RPC handler (unchanged)
# ---------------------------------------------------------------------

def handle_client(conn: socket.socket) -> None:
    req_id = 0
    q: Optional[Queue] = None

    try:
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

        conn.sendall(
            json.dumps(reply, separators=(",", ":")).encode("utf-8")
        )

    except queue.Empty:
        with state_lock:
            pending_replies.pop(req_id, None)
        conn.sendall(
            json.dumps(
                {"success": False, "message": "RPC timeout"},
                separators=(",", ":")
            ).encode("utf-8")
        )

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

    hid_fd = open_hid_blocking()

    threading.Thread(target=hid_reader, daemon=True).start()
    threading.Thread(target=rpc_server, daemon=True).start()

    while True:
        time.sleep(1)

def bootstrap() -> None:
    run()
