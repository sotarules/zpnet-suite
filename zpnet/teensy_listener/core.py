"""
ZPNet Teensy Listener — Deterministic RPC Broker + Pub/Sub Router (VERBOSE)

This version is intentionally noisy.
Its purpose is to make the entire control-plane and data-plane observable
while the pub/sub architecture settles.

REMOVE OR DOWNGRADE LOGGING ONCE STABLE.
"""

from __future__ import annotations

import itertools
import json
import logging
import os
import socket
import threading
import time
import json
from queue import Queue, Empty
from typing import Dict, Any, Optional, TextIO, Set

from zpnet.shared.constants import (
    TRAFFIC_REQUEST_RESPONSE,
    TRAFFIC_DEBUG,
    TRAFFIC_PUBLISH_SUBSCRIBE,
)
from zpnet.shared.logger import setup_logging
from zpnet.shared.util import payload_to_json_str, payload_to_json_bytes
from zpnet.shared.transport import (
    transport_send,
    transport_register_receive_callback,
    transport_init,
)
from zpnet.shared.db import open_db

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

# Pub/Sub subscription table: subsystem -> set(topics)
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
    logger.info("📝 [debug] debug log opened at %s", DEBUG_LOG_PATH)

# ---------------------------------------------------------------------
# Teensy receive callbacks (semantic only)
# ---------------------------------------------------------------------

def on_receive_debug(payload: Dict[str, Any]) -> None:
    logger.info("🐞 [rx-debug] %s", payload)
    if debug_log_fh:
        debug_log_fh.write(payload_to_json_str(payload) + "\n")


def on_receive_request_response(payload: Dict[str, Any]) -> None:
    logger.info("📥 [rx-rpc] response payload=%s", payload)

    req_id = payload.get("req_id")
    if req_id is None:
        raise RuntimeError("Response missing req_id")

    with state_lock:
        q = pending_replies.pop(req_id, None)

    if q is None:
        raise RuntimeError(f"Unexpected req_id {req_id}")

    q.put(payload)


def on_receive_publish_subscribe(payload: Dict[str, Any]) -> None:
    """
    Pub/Sub ingress from Teensy (0xD2).
    """
    logger.info(
        "📥 [rx-0xD2] publish from TEENSY topic=%s payload=%s",
        payload.get("topic"),
        payload.get("payload"),
    )
    route_publish(payload, forward_to_teensy=False)

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
        logger.info("📤 [rpc] incoming request %s", req)

        req_id = next(req_id_counter)
        req["req_id"] = req_id

        q = Queue(maxsize=1)
        with state_lock:
            pending_replies[req_id] = q

        for attempt in range(1, MAX_TEENSY_RETRIES + 1):
            logger.info(
                "📡 [rpc] sending to TEENSY req_id=%s attempt=%d",
                req_id,
                attempt,
            )
            transport_send(TRAFFIC_REQUEST_RESPONSE, req)

            try:
                reply = q.get(timeout=REPLY_TIMEOUT_S)
                logger.info(
                    "✅ [rpc] reply received req_id=%s payload=%s",
                    req_id,
                    reply,
                )
                conn.sendall(
                    json.dumps(reply, separators=(",", ":")).encode("utf-8")
                )
                return

            except Empty:
                logger.warning(
                    "⏳ [rpc] timeout waiting for reply req_id=%s",
                    req_id,
                )

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
# Persisten subscription loader
# ---------------------------------------------------------------------

def load_persisted_subscriptions() -> Dict[str, Set[str]]:
    """
    Load subscription state from system.subscriptions.

    Returns:
        dict: subsystem -> set(topics)
    """
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            "SELECT subscriptions FROM system WHERE id = 1"
        )
        row = cur.fetchone()

    if not row or not row["subscriptions"]:
        return {}

    raw = row["subscriptions"]  # psycopg decodes JSONB -> Python dict
    return {
        subsystem: set(topics)
        for subsystem, topics in raw.items()
    }

# ---------------------------------------------------------------------
# Persisten subscription saver
# ---------------------------------------------------------------------

def persist_subscriptions(subscriptions: Dict[str, Set[str]]) -> None:
    """
    Persist current subscription table into system.subscriptions.

    Semantics:
      • clobber-and-go
      • authoritative
    """
    payload = {
        subsystem: sorted(topics)
        for subsystem, topics in subscriptions.items()
    }

    with open_db() as conn:
        cur = conn.cursor()
        cur.execute(
            """
            UPDATE system
            SET subscriptions = %s::jsonb,
                updated_at = now()
            WHERE id = 1
            """,
            (json.dumps(payload),),
        )

# ---------------------------------------------------------------------
# Pub/Sub routing core (HEAVILY INSTRUMENTED)
# ---------------------------------------------------------------------

def route_publish(msg: Dict[str, Any], *, forward_to_teensy: bool) -> None:
    topic = msg.get("topic")
    payload = msg.get("payload")

    logger.info(
        "📨 [pubsub] incoming publish topic=%s forward_to_teensy=%s payload=%s",
        topic,
        forward_to_teensy,
        payload,
    )

    if not topic:
        raise RuntimeError("Published message missing topic")

    # -------------------------------------------------------------
    # SUBSCRIBE control plane (announcement)
    # -------------------------------------------------------------
    if topic == "SUBSCRIBE":
        subsystem = payload.get("subsystem")
        topics = payload.get("topics")

        if not subsystem or not isinstance(topics, list):
            raise RuntimeError("Malformed SUBSCRIBE message")

        with state_lock:
            subscriptions[subsystem] = set(topics)

        persist_subscriptions(subscriptions)

        logger.info(
            "🚀 [pubsub] subscription announcement ACCEPTED subsystem=%s topics=%s",
            subsystem,
            topics,
        )

        logger.info(
            "🧭 [pubsub] current subscription table=%s",
            {k: sorted(v) for k, v in subscriptions.items()},
        )

        if forward_to_teensy:
            logger.info(
                "📡 [pubsub] REPEATING subscription announcement to TEENSY %s -> %s",
                subsystem,
                topics,
            )
            transport_send(TRAFFIC_PUBLISH_SUBSCRIBE, msg)

        return

    # -------------------------------------------------------------
    # Fan-out to Pi subscribers (data plane)
    # -------------------------------------------------------------
    with state_lock:
        logger.info(
            "🧭 [pubsub] routing snapshot subscriptions=%s",
            {k: sorted(v) for k, v in subscriptions.items()},
        )

        targets = [
            subsystem
            for subsystem, topic_set in subscriptions.items()
            if topic in topic_set
        ]

    logger.info(
        "🎯 [pubsub] routing decision topic=%s targets=%s",
        topic,
        targets,
    )

    raw = payload_to_json_bytes(msg)

    for subsystem in targets:
        sock_path = f"{SOCKET_DIR}/zpnet-{subsystem.lower()}-pubsub.sock"
        logger.info(
            "📤 [pubsub] fan-out ATTEMPT topic=%s -> %s",
            topic,
            sock_path,
        )
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                sock.connect(sock_path)
                sock.sendall(raw)
                sock.shutdown(socket.SHUT_WR)
            logger.info(
                "✅ [pubsub] fan-out SUCCESS topic=%s -> %s",
                topic,
                subsystem,
            )
        except Exception as e:
            logger.warning(
                "⚠️ [pubsub] fan-out FAILED topic=%s -> %s err=%r",
                topic,
                subsystem,
                e,
            )

    # -------------------------------------------------------------
    # Forward to Teensy (Pi-originated only)
    # -------------------------------------------------------------
    if forward_to_teensy:
        logger.info(
            "📡 [pubsub] forwarding publish to TEENSY topic=%s payload=%s",
            topic,
            payload,
        )
        transport_send(TRAFFIC_PUBLISH_SUBSCRIBE, msg)

# ---------------------------------------------------------------------
# Pub/Sub server (Pi-side ingress)
# ---------------------------------------------------------------------

def pubsub_server() -> None:
    try:
        if os.path.exists(PS_SOCKET_PATH):
            os.unlink(PS_SOCKET_PATH)

        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(PS_SOCKET_PATH)
        os.chmod(PS_SOCKET_PATH, 0o660)
        srv.listen(PS_BACKLOG)

        logger.info(
            "🟢 [pubsub-server] listening on %s backlog=%d",
            PS_SOCKET_PATH,
            PS_BACKLOG,
        )

        while True:
            conn, _ = srv.accept()
            with conn:
                raw = conn.recv(65536)
                if not raw:
                    continue
                msg = json.loads(raw.decode("utf-8"))
                logger.info(
                    "📥 [pubsub-server] received from PI socket msg=%s",
                    msg,
                )
                route_publish(msg, forward_to_teensy=True)

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

        logger.info(
            "🟢 [rpc-server] listening on %s backlog=%d",
            RPC_SOCKET_PATH,
            RPC_BACKLOG,
        )

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

    logger.info("🚀 [startup] initializing transport")
    transport_init()

    # Restore persisted subscriptions
    restored = load_persisted_subscriptions()

    with state_lock:
        subscriptions.update(restored)

    logger.info(
        "♻️ [pubsub] restored subscriptions from system table: %s",
        {k: sorted(v) for k, v in subscriptions.items()},
    )

    # Replay to Teensy
    for subsystem, topics in restored.items():
        msg = {
            "topic": "SUBSCRIBE",
            "payload": {
                "subsystem": subsystem,
                "topics": sorted(topics),
            },
        }

        logger.info(
            "📡 [pubsub] replaying persisted subscription to TEENSY %s -> %s",
            subsystem,
            sorted(topics),
        )

        transport_send(TRAFFIC_PUBLISH_SUBSCRIBE, msg)

    transport_register_receive_callback(
        TRAFFIC_DEBUG,
        on_receive_debug,
    )
    transport_register_receive_callback(
        TRAFFIC_REQUEST_RESPONSE,
        on_receive_request_response,
    )
    transport_register_receive_callback(
        TRAFFIC_PUBLISH_SUBSCRIBE,
        on_receive_publish_subscribe,
    )

    logger.info("🚀 [startup] transport callbacks registered")

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

        logger.info("🚀 [startup] teensy_listener fully online")

        while True:
            time.sleep(1)

    except Exception:
        logging.exception("💥 [teensy_listener] fatal crash")


def bootstrap() -> None:
    run()
