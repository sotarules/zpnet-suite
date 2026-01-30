"""
ZPNet Teensy Listener — Deterministic RPC Broker + Pub/Sub Router

Role:
  • Own Teensy RPC socket
  • Perform strict req_id-based RPC correlation
  • Apply retry / timeout policy at transport boundary
  • Own Pi-side pub/sub ingress
  • Maintain authoritative subscription table
  • Route published traffic to Pi subsystems

Semantics:
  • Pub/Sub is fire-and-forget
  • No retries
  • No delivery guarantees
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
from typing import Dict, Any, Optional, TextIO, Set

from zpnet.shared.constants import TRAFFIC_REQUEST_RESPONSE, TRAFFIC_DEBUG
from zpnet.shared.logger import setup_logging
from zpnet.shared.transport import (
    transport_send,
    transport_register_receive_callback,
    transport_init,
)

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

RPC_SOCKET_PATH = "/tmp/zpnet_teensy_rt.sock"
PS_SOCKET_PATH  = "/tmp/zpnet_teensy_ps.sock"

RPC_BACKLOG = 8
PS_BACKLOG  = 16

MAX_TEENSY_RETRIES = 3
REPLY_TIMEOUT_S = 3.0

DEBUG_LOG_PATH = "/home/mule/zpnet/logs/zpnet-debug.log"

SOCKET_DIR = "/tmp"

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------
# Global state
# ---------------------------------------------------------------------

pending_replies: Dict[int, Queue] = {}
req_id_counter = itertools.count(1)

# Pub/Sub subscription table
subscriptions: Dict[str, Set[str]] = {}

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
# Teensy receive callbacks
# ---------------------------------------------------------------------

def on_receive_debug(message: bytes) -> None:
    debug_log_fh.write(repr(message) + "\n")

def on_receive_request_response(payload: Dict[str, Any]) -> None:
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
        raw = conn.recv(65536)
        if not raw:
            return

        req = json.loads(raw.decode("utf-8"))

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
# Pub/Sub ingress and routing
# ---------------------------------------------------------------------

def route_publish(msg: Dict[str, Any]) -> None:
    topic = msg.get("topic")
    if not topic:
        raise RuntimeError("Published message missing topic")

    # -------------------------------------------------------------
    # SUBSCRIPTION control plane
    # -------------------------------------------------------------
    if topic == "SUBSCRIPTION":
        payload = msg.get("payload", {})
        subsystem = payload.get("subsystem")
        topics = payload.get("topics")

        if not subsystem or not isinstance(topics, list):
            raise RuntimeError("Malformed SUBSCRIPTION message")

        with state_lock:
            subscriptions[subsystem] = set(topics)

        logger.info(
            "🚀 [pubsub] subscription update %s -> %s",
            subsystem,
            topics,
        )
        return

    # -------------------------------------------------------------
    # Fan-out routing
    # -------------------------------------------------------------
    with state_lock:
        targets = [
            subsystem
            for subsystem, topic_set in subscriptions.items()
            if topic in topic_set
        ]

    logger.info("🚀 [pubsub] fan out routing for %s",targets)

    raw = json.dumps(msg, separators=(",", ":")).encode("utf-8")

    for subsystem in targets:
        sock_path = f"{SOCKET_DIR}/zpnet-{subsystem.lower()}-pubsub.sock"
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                sock.connect(sock_path)
                sock.sendall(raw)
                sock.shutdown(socket.SHUT_WR)
        except Exception:
            # Best-effort by design
            continue

# ---------------------------------------------------------------------
# Pub/Sub server
# ---------------------------------------------------------------------

def pubsub_server() -> None:
    try:
        if os.path.exists(PS_SOCKET_PATH):
            os.unlink(PS_SOCKET_PATH)

        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(PS_SOCKET_PATH)
        os.chmod(PS_SOCKET_PATH, 0o660)
        srv.listen(PS_BACKLOG)

        while True:
            conn, _ = srv.accept()
            with conn:
                raw = conn.recv(65536)
                if not raw:
                    continue
                msg = json.loads(raw.decode("utf-8"))
                route_publish(msg)

    except Exception:
        logging.exception("💥 [pubsub_server] fatal exception")

# ---------------------------------------------------------------------
# RPC server
# ---------------------------------------------------------------------

def rpc_server() -> None:
    try:
        if os.path.exists(RPC_SOCKET_PATH):
            os.unlink(RPC_SOCKET_PATH)

        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(RPC_SOCKET_PATH)
        os.chmod(RPC_SOCKET_PATH, 0o660)
        srv.listen(RPC_BACKLOG)

        while True:
            conn, _ = srv.accept()
            threading.Thread(
                target=handle_client,
                args=(conn,),
                daemon=True,
            ).start()

    except Exception:
        logging.exception("💥 [rpc_server] fatal exception")

# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    setup_logging()
    open_debug_log()
    transport_init()

    transport_register_receive_callback(TRAFFIC_DEBUG, on_receive_debug)
    transport_register_receive_callback(
        TRAFFIC_REQUEST_RESPONSE,
        on_receive_request_response,
    )

    try:
        threading.Thread(
            target=rpc_server,
            daemon=True,
            name="rpc-server",
        ).start()

        threading.Thread(
            target=pubsub_server,
            daemon=True,
            name="pubsub-server",
        ).start()

        while True:
            time.sleep(1)

    except Exception:
        logging.exception("💥 [teensy_listener] fatal crash")

def bootstrap() -> None:
    run()
