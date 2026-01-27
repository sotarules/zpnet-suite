"""
ZPNet Teensy Listener — RawHID Transport Authority + Deterministic RPC

Role:
  • Own the RawHID transport
  • Frame and send commands to Teensy
  • Reassemble framed responses
  • Perform strict req_id-based RPC correlation

Non-responsibilities:
  • No event handling
  • No semantic interpretation
  • No aggregation
  • No fallback routing
"""

import json
import logging
import os
import socket
import threading
import time
import itertools
from queue import Queue
from typing import Dict, Optional, Tuple

from zpnet.shared.logger import setup_logging
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
# Debug sink (transport-level only)
# ---------------------------------------------------------------------

def open_debug_log() -> None:
    global debug_log_fh
    os.makedirs(os.path.dirname(DEBUG_LOG_PATH), exist_ok=True)
    debug_log_fh = open(DEBUG_LOG_PATH, "w", buffering=1)

def write_debug_line(text: str) -> None:
    if debug_log_fh:
        debug_log_fh.write(text + "\n")

# ---------------------------------------------------------------------
# HID helpers (unowned boundary)
# ---------------------------------------------------------------------

def open_hid_blocking() -> int:
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

    if not hid_present.is_set():
        raise RuntimeError("Teensy HID not present")

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
                    logging.warning("[teensy_listener] hidraw vanished during write")
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

def read_rawhid_message() -> Optional[Tuple[bool, bytes]]:
    global hid_fd

    buf = bytearray()
    is_debug = False

    while True:
        try:
            pkt = os.read(hid_fd, HID_PACKET_SIZE)
        except OSError as e:
            if e.errno in (5, 19):
                logging.warning("[teensy_listener] hidraw vanished during read")
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

        # Detect debug on ANY packet
        if pkt[0] == DEBUG_MARKER:
            is_debug = True
            pkt = pkt[1:]  # strip marker unconditionally

        # Strip right-side zero padding
        stripped = pkt.rstrip(b"\x00")
        has_padding = len(stripped) < len(pkt)

        if stripped:
            buf.extend(stripped)

        if has_padding:
            return is_debug, bytes(buf)

# ---------------------------------------------------------------------
# Transport helpers
# ---------------------------------------------------------------------

def send_frame(payload: dict) -> None:
    raw = json.dumps(payload, separators=(",", ":")).encode("utf-8")

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
# Response parsing + routing
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

def route_message(msg: dict) -> None:
    req_id = msg.get("req_id")
    if req_id is None:
        # Protocol violation — ignore silently
        return

    with state_lock:
        q = pending_replies.pop(req_id, None)

    if q:
        q.put(msg)

# ---------------------------------------------------------------------
# HID reader (single authority)
# ---------------------------------------------------------------------

def hid_reader() -> None:
    global hid_fd

    try:
        while True:
            if not hid_present.is_set():
                logging.info("[teensy_listener] waiting for Teensy HID...")
                hid_fd = open_hid_blocking()
                continue

            result = read_rawhid_message()
            if not result:
                continue

            is_debug, msg = result

            if is_debug:
                write_debug_line(
                    msg.decode("utf-8", errors="replace")
                )
                continue

            if msg and msg[0] == ASCII_LT:
                process_response(msg)

    except Exception:
        logging.exception("[hid_reader] fatal exception")


# ---------------------------------------------------------------------
# RPC handler
# ---------------------------------------------------------------------

def handle_client(conn: socket.socket) -> None:
    try:
        buf = conn.recv(65536)
        if not buf:
            return

        req = json.loads(buf.decode("utf-8"))

        req_id = next(req_id_counter)
        req["req_id"] = req_id

        q = Queue(maxsize=1)
        with state_lock:
            pending_replies[req_id] = q

        send_frame(req)

        reply = q.get(timeout=10.0)
        conn.sendall(json.dumps(reply, separators=(",", ":")).encode("utf-8"))

    finally:
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
        logging.exception("[rpc_server] fatal exception")

# ---------------------------------------------------------------------
# Serial snoop (forensics only)
# ---------------------------------------------------------------------

def _snoop(direction: str, payload: bytes) -> None:
    with open(SERIAL_SNOOP_PATH, "a") as f:
        f.write(
            f"{time.time():.6f} {direction} "
            f"{payload.decode('utf-8', errors='replace')}\n"
        )

# ---------------------------------------------------------------------
# Entrypoint
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
