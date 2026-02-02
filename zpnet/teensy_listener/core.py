"""
ZPNet Teensy Listener — Deterministic RPC Broker + Pub/Sub Router

Roles:
  • Pi-side device driver for Teensy (RPC + pub/sub transport)
  • Authoritative pub/sub router
  • First-class Pi process (PUBSUB) for orchestration & introspection

IMPORTANT:
  • SUBSCRIBE / UNSUBSCRIBE are COMMANDS ONLY
  • Pub/Sub channel is DATA PLANE ONLY
"""

from __future__ import annotations

import itertools
import json
import logging
import os
import socket
import threading
from queue import Queue, Empty
from typing import Dict, Any, Optional, TextIO, Set, List

from zpnet.processes.processes import server_setup, send_command, list_subsystems
from zpnet.shared.constants import (
    TRAFFIC_REQUEST_RESPONSE,
    TRAFFIC_DEBUG,
    TRAFFIC_PUBLISH_SUBSCRIBE,
)
from zpnet.shared.logger import setup_logging
from zpnet.shared.transport import (
    transport_send,
    transport_register_receive_callback,
    transport_init,
)
from zpnet.shared.util import payload_to_json_str, payload_to_json_bytes

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

# ---------------------------------------------------------------------
# Global state
# ---------------------------------------------------------------------

pending_replies: Dict[int, Queue] = {}
req_id_counter = itertools.count(1)

# Router state (empty until applied later)
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
    logging.info("📝 [debug] debug log opened at %s", DEBUG_LOG_PATH)

# ---------------------------------------------------------------------
# Teensy receive callbacks
# ---------------------------------------------------------------------

def on_receive_debug(payload: Dict[str, Any]) -> None:
    logging.info("🐞 [rx-debug] %s", payload)
    if debug_log_fh:
        debug_log_fh.write(payload_to_json_str(payload) + "\n")

def on_receive_request_response(payload: Dict[str, Any]) -> None:
    req_id = payload.get("req_id")
    if req_id is None:
        raise RuntimeError("Response missing req_id")

    with state_lock:
        q = pending_replies.pop(req_id, None)

    if q is None:
        raise RuntimeError(f"Unexpected req_id {req_id}")

    q.put(payload)

def on_receive_publish_subscribe(payload: Dict[str, Any]) -> None:
    route_publish(payload, forward_to_teensy=False)

# ---------------------------------------------------------------------
# RPC handler (Pi → Teensy)
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
                conn.sendall(json.dumps(reply, separators=(",", ":")).encode())
                return
            except Empty:
                continue

        conn.sendall(json.dumps({
            "req_id": req_id,
            "success": False,
            "message": "TEENSY did not respond",
        }).encode())

    finally:
        if req_id is not None:
            with state_lock:
                pending_replies.pop(req_id, None)
        conn.close()

# ---------------------------------------------------------------------
# Pub/Sub routing core (DATA PLANE ONLY)
# ---------------------------------------------------------------------

def route_publish(msg: Dict[str, Any], *, forward_to_teensy: bool) -> None:
    topic = msg.get("topic")
    if not topic:
        return

    with state_lock:
        targets = [
            subsystem
            for subsystem, topic_set in subscriptions.items()
            if topic in topic_set
        ]

    raw = payload_to_json_bytes(msg)

    for subsystem in targets:
        sock_path = f"{SOCKET_DIR}/zpnet-{subsystem.lower()}-pubsub.sock"
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                sock.connect(sock_path)
                sock.sendall(raw)
                sock.shutdown(socket.SHUT_WR)
        except Exception:
            logging.warning("⚠️ [pubsub] fan-out failed %s -> %s", topic, subsystem)

    if forward_to_teensy:
        transport_send(TRAFFIC_PUBLISH_SUBSCRIBE, msg)

# ---------------------------------------------------------------------
# Servers
# ---------------------------------------------------------------------

def pubsub_server() -> None:
    if os.path.exists(PS_SOCKET_PATH):
        os.unlink(PS_SOCKET_PATH)

    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(PS_SOCKET_PATH)
    srv.listen(PS_BACKLOG)

    while True:
        conn, _ = srv.accept()
        with conn:
            raw = conn.recv(65536)
            if not raw:
                continue
            msg = json.loads(raw.decode())
            route_publish(msg, forward_to_teensy=True)

def rpc_server() -> None:
    if os.path.exists(RPC_SOCKET_PATH):
        os.unlink(RPC_SOCKET_PATH)

    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(RPC_SOCKET_PATH)
    srv.listen(RPC_BACKLOG)

    while True:
        conn, _ = srv.accept()
        threading.Thread(target=handle_client, args=(conn,), daemon=True).start()

def build_teensy_setsubscriptions_payload(union_subs: list[dict]) -> dict:
    """
    Convert union subscription truth into Teensy SETSUBSCRIPTIONS payload.
    """

    entries = []

    for row in union_subs:
        if row.get("machine") != "TEENSY":
            continue

        process = row.get("subsystem")
        subs = row.get("subscriptions", [])

        topics = [s["name"] for s in subs if "name" in s]

        if not process or not topics:
            continue

        entries.append({
            "process": process,
            "topics": topics,
        })

    return {
        "subscriptions": entries
    }

# ---------------------------------------------------------------------
# PUBSUB command surface
# ---------------------------------------------------------------------

def cmd_report(_: Optional[dict]) -> Dict[str, Any]:
    with state_lock:
        return {
            "success": True,
            "message": "OK",
            "payload": {
                "subscriptions": {
                    k: sorted(v) for k, v in subscriptions.items()
                }
            },
        }

def cmd_allsubscriptions(_: Optional[dict]) -> Dict[str, Any]:
    results: List[Dict[str, Any]] = []

    for subsystem in list_subsystems():

        try:
            # Self-query: answer directly
            if subsystem == "PUBSUB":
                results.append({
                    "machine": "PI",
                    "subsystem": "PUBSUB",
                    "subscriptions": [],
                })
                continue

            # IPC for real subsystems
            resp = send_command(
                machine="PI",
                subsystem=subsystem,  # protocol identity
                command="SUBSCRIPTIONS",
            )

            payload = resp.get("payload")
            if payload:
                results.append(payload)

        except Exception:
            logging.warning(
                "⚠️ [pubsub] failed to query SUBSCRIPTIONS from %s",
                subsystem,
            )

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "subscriptions": results
        },
    }

def cmd_unionsubscriptions(_: Optional[dict]) -> Dict[str, Any]:
    """
    Return the union of PI-side and TEENSY-side declared subscriptions.

    Semantics:
      • Observational only
      • No routing mutation
      • Best-effort
      • Canonical payload shape
    """

    results: List[Dict[str, Any]] = []

    # ------------------------------------------------------------
    # 1. PI-side subscriptions (local)
    # ------------------------------------------------------------
    try:
        pi_resp = cmd_allsubscriptions(None)
        pi_payload = pi_resp.get("payload", {}).get("subscriptions", [])
        results.extend(pi_payload)
    except Exception:
        logging.warning("⚠️ [pubsub] failed to collect PI subscriptions")

    # ------------------------------------------------------------
    # 2. TEENSY-side subscriptions (remote)
    # ------------------------------------------------------------
    try:
        teensy_payload = send_command(
            machine="TEENSY",
            subsystem="PUBSUB",
            command="ALLSUBSCRIPTIONS",
        ).get("payload", {}).get("subscriptions", [])

        results.extend(teensy_payload)

    except Exception:
        logging.warning("⚠️ [pubsub] failed to collect TEENSY subscriptions")

    # ------------------------------------------------------------
    # 3. Canonical ordering (nice but optional)
    # ------------------------------------------------------------
    results.sort(key=lambda x: (x.get("machine"), x.get("subsystem")))

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "subscriptions": results
        },
    }

def cmd_refresh(_: Optional[dict]) -> Dict[str, Any]:
    """
    REFRESH — commit canonical subscription truth.

    Semantics:
      • Collect PI + TEENSY declared subscriptions
      • Union them into a single canonical payload
      • Commit that payload verbatim to:
          1) TEENSY (SETSUBSCRIPTIONS)
          2) PI (local router state)
    """

    # ------------------------------------------------------------
    # 1. Observe + union (single source of truth)
    # ------------------------------------------------------------
    union_resp = cmd_unionsubscriptions(None)
    union_payload = union_resp.get("payload", {})

    logging.info("🚀 ️ [pubsub] REFRESH union payload: %s", union_payload)

    # ------------------------------------------------------------
    # 2. Commit union verbatim to TEENSY
    # ------------------------------------------------------------
    logging.info("🚀 ️ [pubsub] REFRESH setting subscriptions in Teensy payload: %s", union_payload)
    try:
        teensy_resp = send_command(
            machine="TEENSY",
            subsystem="PUBSUB",
            command="SETSUBSCRIPTIONS",
            args=union_payload,
        )
    except Exception as e:
        logging.warning("⚠️ [pubsub] REFRESH failed applying TEENSY state: %s", e)
        return {
            "success": False,
            "message": "REFRESH failed (TEENSY SETSUBSCRIPTIONS error)",
            "payload": {
                "union": union_payload,
            },
        }

    if not teensy_resp.get("success", False):
        return {
            "success": False,
            "message": "REFRESH failed (TEENSY rejected SETSUBSCRIPTIONS)",
            "payload": {
                "union": union_payload,
                "teensy_response": teensy_resp,
            },
        }



    # ------------------------------------------------------------
    # 3. Commit same union locally on PI
    # ------------------------------------------------------------
    new_table: Dict[str, Set[str]] = {}

    for row in union_payload.get("subscriptions", []):
        subsystem = row.get("subsystem")
        subs = row.get("subscriptions", [])

        if not subsystem:
            continue

        for s in subs:
            name = s.get("name")
            if not name:
                continue
            new_table.setdefault(subsystem, set()).add(name)



    with state_lock:
        subscriptions.clear()
        subscriptions.update(new_table)
        logging.info("🚀 ️ [pubsub] subscriptions updated: %s", subscriptions)

    # ------------------------------------------------------------
    # 4. Return committed truth
    # ------------------------------------------------------------
    return {
        "success": True,
        "message": "OK",
        "payload": union_payload,
    }


# ---------------------------------------------------------------------
# Declarations
# ---------------------------------------------------------------------

COMMANDS = {
    "REPORT": cmd_report,
    "ALLSUBSCRIPTIONS": cmd_allsubscriptions,
    "UNIONSUBSCRIPTIONS": cmd_unionsubscriptions,
    "REFRESH": cmd_refresh,
}

# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    setup_logging()
    open_debug_log()
    transport_init()

    transport_register_receive_callback(TRAFFIC_DEBUG, on_receive_debug)
    transport_register_receive_callback(TRAFFIC_REQUEST_RESPONSE, on_receive_request_response)
    transport_register_receive_callback(TRAFFIC_PUBLISH_SUBSCRIBE, on_receive_publish_subscribe)

    threading.Thread(target=rpc_server, daemon=True).start()
    threading.Thread(target=pubsub_server, daemon=True).start()

    server_setup(
        subsystem="PUBSUB",
        commands=COMMANDS,
        subscriptions={},
    )

def bootstrap() -> None:
    run()