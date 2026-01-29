"""
ZPNet Teensy Listener — Deterministic RPC Broker (Transport-Aligned)

Role:
  • Own RPC socket
  • Perform strict req_id-based RPC correlation
  • Apply retry / timeout policy
  • Register and dispatch semantic receive callbacks

Transport:
  • RawHID I/O, chunking, reassembly, demux, JSON decode handled by transport.py

Non-responsibilities:
  • No framing
  • No packetization
  • No semantic interpretation
"""

from __future__ import annotations

import itertools
import json
import logging
import os
import socket
import threading
import time
from queue import Queue, Empty
from typing import Dict, Any
from typing import Optional, TextIO

from zpnet.shared.constants import TRAFFIC_REQUEST_RESPONSE, TRAFFIC_DEBUG
from zpnet.shared.logger import setup_logging
from zpnet.shared.transport import transport_send, transport_register_receive_callback, transport_init

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

RPC_SOCKET_PATH = "/tmp/zpnet_teensy_rt.sock"
RPC_BACKLOG = 8

MAX_TEENSY_RETRIES = 3
REPLY_TIMEOUT_S = 3.0

DEBUG_LOG_PATH = "/home/mule/zpnet/logs/zpnet-debug.log"

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------
# Global state
# ---------------------------------------------------------------------

pending_replies: Dict[int, Queue] = {}
req_id_counter = itertools.count(1)

state_lock = threading.Lock()

debug_log_fh: Optional[TextIO] = None

# ---------------------------------------------------------------------
# Debug sink
# ---------------------------------------------------------------------

def open_debug_log() -> None:
    global debug_log_fh
    os.makedirs(os.path.dirname(DEBUG_LOG_PATH), exist_ok=True)
    debug_log_fh = open(DEBUG_LOG_PATH, "w", buffering=1)

# ---------------------------------------------------------------------
# Debug receive callback
# ---------------------------------------------------------------------

def on_receive_debug(message: bytes) -> None:
    """
    Receive DEBUG traffic from Teensy.
    """
    text = message.decode("utf-8", errors="replace")
    debug_log_fh.write(repr(message) + "\n")

# ---------------------------------------------------------------------
# REQUEST / RESPONSE receive callback
# ---------------------------------------------------------------------

def on_receive_request_response(payload: Dict[str, Any]) -> None:
    """
    Receive a REQUEST_RESPONSE message from Teensy.
    """
    req_id = payload.get("req_id")
    if req_id is None:
        raise RuntimeError("Response missing req_id")

    with state_lock:
        q = pending_replies.pop(req_id, None)

    if q is None:
        raise RuntimeError(f"Unexpected req_id {req_id}")

    q.put(payload)


# ---------------------------------------------------------------------
# RPC handler
# ---------------------------------------------------------------------

def handle_client(conn: socket.socket) -> None:
    req_id = None
    q = None

    try:
        buf = conn.recv(65536)
        if not buf:
            return

        req = json.loads(buf.decode("utf-8"))

        # Assign req_id
        req_id = next(req_id_counter)
        req["req_id"] = req_id

        q = Queue(maxsize=1)
        with state_lock:
            pending_replies[req_id] = q

        for _ in range(MAX_TEENSY_RETRIES):
            transport_send(TRAFFIC_REQUEST_RESPONSE, req)

            try:
                reply = q.get(timeout=REPLY_TIMEOUT_S)
                conn.sendall(
                    json.dumps(reply, separators=(",", ":")).encode("utf-8")
                )
                return

            except Empty:
                continue

        # ---------------------------------------------------------
        # All retries exhausted
        # ---------------------------------------------------------
        failure = {
            "req_id": req_id,
            "success": False,
            "message": (
                f"TEENSY did not respond after "
                f"{MAX_TEENSY_RETRIES} attempts"
            ),
        }

        conn.sendall(
            json.dumps(failure, separators=(",", ":")).encode("utf-8")
        )

    finally:
        if req_id is not None:
            with state_lock:
                pending_replies.pop(req_id, None)
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
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:

    setup_logging()
    open_debug_log()
    transport_init()

    # -------------------------------------------------------------
    # Register receive callbacks (symmetric with Teensy)
    # -------------------------------------------------------------

    transport_register_receive_callback(TRAFFIC_DEBUG, on_receive_debug)
    transport_register_receive_callback(TRAFFIC_REQUEST_RESPONSE, on_receive_request_response)

    try:
        threading.Thread(target=rpc_server, daemon=True, name="rpc-server").start()

        while True:
            time.sleep(1)

    except Exception:
        logging.exception("[main] teensy_listener fatal crash")

def bootstrap() -> None:
    run()
