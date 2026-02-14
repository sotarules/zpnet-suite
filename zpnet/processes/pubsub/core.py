"""
ZPNet Publish/Subscribe — Deterministic RPC Broker + Pub/Sub Router

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
import time
from queue import Queue, Empty
from typing import Dict, Any, Optional, TextIO, Set, List, Tuple

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
    transport_init, )
from zpnet.shared.util import payload_to_json_str, payload_to_json_bytes

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

RPC_SOCKET_PATH = "/tmp/zpnet_teensy_rt.sock"
PS_SOCKET_PATH  = "/tmp/zpnet_teensy_ps.sock"

RPC_BACKLOG = 8
PS_BACKLOG  = 16

MAX_TEENSY_RETRIES = 3
REPLY_TIMEOUT_S = 30.0

DEBUG_LOG_PATH = "/home/mule/zpnet/logs/zpnet-debug.log"
SOCKET_DIR = "/tmp"

# ---------------------------------------------------------------------
# Global state
# ---------------------------------------------------------------------

pending_replies: Dict[int, Queue] = {}
req_id_counter = itertools.count(1)

# ---------------------------------------------------------------------
# Routing table (cartesian subscription edges)
# ---------------------------------------------------------------------
#
# Logical model:
#   Each subscription declaration expands into atomic edges of the form:
#
#       (machine, subsystem, topic)
#
#   This is the Cartesian product of:
#       • declaring entities (machine × subsystem)
#       • declared topics
#
# Physical storage:
#   routes_by_topic groups those edges by topic, dropping the topic
#   dimension into the dictionary key:
#
#       routes_by_topic[topic] = {
#           (machine, subsystem),
#           (machine, subsystem),
#           ...
#       }
#
# Interpretation:
#   • Each tuple represents ONE independent delivery target
#   • The set enforces idempotence (no duplicate delivery edges)
#   • No tuple implies ordering, grouping, or batching
#   • Fan-out is performed once per tuple
#
# This structure is intentionally the inverse of declaration form:
#   declaration:  subsystem -> [topics]
#   routing:      topic -> {(machine, subsystem)}
#
# ---------------------------------------------------------------------

routes_by_topic: Dict[str, Set[Tuple[str, str]]] = {}
applied_union: Dict[str, Any] = {}

state_lock = threading.Lock()
debug_log_fh: Optional[TextIO] = None

# ------------------------------------------------------------------
# Logging
# ------------------------------------------------------------------

PUBSUB_LOG_PATH = "/home/mule/zpnet/logs/zpnet-pubsub.log"

pubsub_log_fh: Optional[TextIO] = None

def open_pubsub_log() -> None:
    global pubsub_log_fh
    if pubsub_log_fh:
        pubsub_log_fh.close()
    os.makedirs(os.path.dirname(PUBSUB_LOG_PATH), exist_ok=True)
    pubsub_log_fh = open(PUBSUB_LOG_PATH, "w", buffering=1)
    logging.info("📝 [open_pubsub_log] pubsub log opened at %s", PUBSUB_LOG_PATH)

def log_pubsub(payload: Dict[str, Any]) -> None:
    if pubsub_log_fh:
        topic = payload["topic"]
        payload_string = payload_to_json_str(payload)
        pubsub_log_fh.write(f"{topic} {payload_string}\n" )


# ---------------------------------------------------------------------
# Debug sink
# ---------------------------------------------------------------------

def open_debug_log() -> None:
    global debug_log_fh
    os.makedirs(os.path.dirname(DEBUG_LOG_PATH), exist_ok=True)
    debug_log_fh = open(DEBUG_LOG_PATH, "w", buffering=1)
    logging.info("📝 [debug] debug log opened at %s", DEBUG_LOG_PATH)

# ---------------------------------------------------------------------
# Teensy receive callbacks (GATED)
# ---------------------------------------------------------------------

def on_receive_debug(payload: Dict[str, Any]) -> None:
    if debug_log_fh:
        debug_log_fh.write(payload_to_json_str(payload) + "\n")

def on_receive_request_response(payload: Dict[str, Any]) -> None:
    req_id = payload.get("req_id")
    if req_id is None:
        logging.warning("⚠️ response missing req_id — discarded")
        return

    with state_lock:
        q = pending_replies.pop(req_id, None)

    if q is None:
        logging.warning("⚠️ [on_receive_request_response] late or duplicate response req_id=%d (discarded)", req_id)
        return

    sent_ms = payload.get("req_ts_ms")
    if sent_ms is None:
        logging.warning("⚠️ response missing req_ts_ms — discarded")
        return

    now_ms = int(time.monotonic() * 1000)
    latency = now_ms - sent_ms

    payload["latency"] = latency

    q.put(payload)

def on_receive_publish_subscribe(payload: Dict[str, Any]) -> None:
    # Message originated from Teensy. Never forward back to Teensy.
    route_publish(payload, forward_to_teensy=False)

# ---------------------------------------------------------------------
# RPC handler (Pi → Teensy)
# ---------------------------------------------------------------------

def handle_client(conn: socket.socket) -> None:
    try:
        raw = conn.recv(65536)
        if not raw:
            return

        req = json.loads(raw.decode("utf-8"))

        req_id = None

        for attempt in range(MAX_TEENSY_RETRIES):
            # NEW: fresh req_id per attempt
            req_id = next(req_id_counter)
            req["req_id"] = req_id
            req["req_ts_ms"] = int(time.monotonic() * 1000)

            q = Queue(maxsize=1)
            with state_lock:
                pending_replies[req_id] = q

            try:
                transport_send(TRAFFIC_REQUEST_RESPONSE, req)

                reply = q.get(timeout=REPLY_TIMEOUT_S)
                conn.sendall(
                    json.dumps(reply, separators=(",", ":")).encode()
                )
                return

            except Empty:
                # Timeout on this attempt — clean up and retry
                with state_lock:
                    pending_replies.pop(req_id, None)
                continue

        # All retries exhausted
        conn.sendall(
            json.dumps({
                "req_id": req_id,
                "success": False,
                "message": "TEENSY did not respond",
            }).encode()
        )

    finally:
        conn.close()


# ---------------------------------------------------------------------
# Pub/Sub routing core
# ---------------------------------------------------------------------

def route_publish(msg: Dict[str, Any], *, forward_to_teensy: bool) -> None:
    """
    Route a single published message to all interested targets.

    Semantics:
      • Fan-out to PI targets via per-subsystem pubsub sockets
      • Forward to TEENSY at most once (transport layer) if:
          - forward_to_teensy is True, and
          - at least one TEENSY target exists for the topic
      • Best-effort, silence on per-target failure
    """
    topic = msg.get("topic")
    if not topic:
        return

    log_pubsub(msg)

    # Snapshot the Cartesian product slice for this topic:
    #   all (machine, subsystem) pairs that subscribed to it.
    with state_lock:
        topic_routes = routes_by_topic.get(topic, set())

    # Copy the set so fan-out iterates over a stable view
    # of the subscription edge set.
    targets = set(topic_routes)

    raw = payload_to_json_bytes(msg)

    teensy_needed = False

    for machine, subsystem in targets:
        if machine == "TEENSY":
            teensy_needed = True
            continue

        # PI target: deliver to its pubsub socket
        sock_path = f"{SOCKET_DIR}/zpnet-{subsystem.lower()}-pubsub.sock"
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                sock.connect(sock_path)
                sock.sendall(raw)
                sock.shutdown(socket.SHUT_WR)
        except Exception:
            logging.warning("⚠️ [pubsub] fan-out failed %s -> %s:%s", topic, machine, subsystem)

    if forward_to_teensy and teensy_needed:
        transport_send(TRAFFIC_PUBLISH_SUBSCRIBE, msg)

# ---------------------------------------------------------------------
# Servers
# ---------------------------------------------------------------------

def pubsub_server() -> None:
    """
    Accepts PI-originated publishes (local publishers).
    These may need to be forwarded to Teensy if Teensy targets exist.
    """
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

# ---------------------------------------------------------------------
# PUBSUB command surface
# ---------------------------------------------------------------------

def cmd_diagnostics(_: Optional[dict]) -> Dict[str, Any]:
    with state_lock:
        pending_count = len(pending_replies)
        pending_ids = list(pending_replies.keys())

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "pending_reply_count": pending_count,
            "pending_req_ids": pending_ids[:50],  # cap for sanity
            "req_id_current": next(req_id_counter),
            "routes_topic_count": len(routes_by_topic),
            "active_threads": threading.active_count(),
        },
    }

def cmd_report(_: Optional[dict]) -> Dict[str, Any]:
    """
    Report the applied routing truth in its raw execution form.

    The payload is the Cartesian product of:
        (machine, subsystem, topic)

    This is the exact structure used internally by the router,
    with no projection, grouping, or semantic reshaping.
    """

    rows: List[Dict[str, str]] = []

    with state_lock:
        for topic, targets in routes_by_topic.items():
            for machine, subsystem in targets:
                rows.append({
                    "machine": machine,
                    "subsystem": subsystem,
                    "topic": topic,
                })

    # Deterministic ordering for diffing and sanity
    rows.sort(key=lambda r: (r["machine"], r["subsystem"], r["topic"]))

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "routes": rows
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
                subsystem=subsystem,
                command="SUBSCRIPTIONS",
            )

            payload = resp.get("payload")
            if payload:
                results.append(payload)

        except Exception:
            logging.exception(
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

    # 1) PI-side
    try:
        pi_resp = cmd_allsubscriptions(None)
        pi_payload = pi_resp.get("payload", {}).get("subscriptions", [])
        results.extend(pi_payload)
    except Exception:
        logging.warning("⚠️ [pubsub] failed to collect PI subscriptions")

    # 2) TEENSY-side
    try:
        teensy_payload = send_command(
            machine="TEENSY",
            subsystem="PUBSUB",
            command="ALLSUBSCRIPTIONS",
        ).get("payload", {}).get("subscriptions", [])

        results.extend(teensy_payload)

    except Exception:
        logging.warning("⚠️ [pubsub] failed to collect TEENSY subscriptions")

    # 3) Canonical ordering
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
          2) PI (local routing state)
    """

    # 1) Observe + union (single source of truth)
    union_resp = cmd_unionsubscriptions(None)
    union_payload = union_resp.get("payload", {})

    # 2) Commit union verbatim to TEENSY
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

    # 3) Build machine-qualified routes_by_topic from union
    new_routes: Dict[str, Set[Tuple[str, str]]] = {}

    for row in union_payload.get("subscriptions", []):
        machine = row.get("machine")
        subsystem = row.get("subsystem")
        subs = row.get("subscriptions", [])

        if not machine or not subsystem:
            continue

        for s in subs:
            name = s.get("name") if isinstance(s, dict) else None
            if not name:
                continue
            new_routes.setdefault(name, set()).add((machine, subsystem))

    # 4) Commit locally
    with state_lock:
        routes_by_topic.clear()
        routes_by_topic.update(new_routes)

        applied_union.clear()
        applied_union.update(union_payload)

        logging.info("🚀 [pubsub] routes updated (%d topics)", len(routes_by_topic))

    # 5) Return committed truth
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
    "DIAGNOSTICS": cmd_diagnostics,
}

def _delayed_refresh(delay_s: float = 10.0) -> None:
    """
    Invoke REFRESH after a fixed delay to allow full system bring-up.
    """
    import time

    time.sleep(delay_s)
    try:
        logging.info("🔄 [pubsub] auto REFRESH starting")
        cmd_refresh(None)
        logging.info("✅ [pubsub] auto REFRESH complete")
    except Exception:
        logging.exception("❌ [pubsub] auto REFRESH failed")

# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    setup_logging()
    open_debug_log()
    open_pubsub_log()
    transport_init()

    transport_register_receive_callback(TRAFFIC_DEBUG, on_receive_debug)
    transport_register_receive_callback(TRAFFIC_REQUEST_RESPONSE, on_receive_request_response)
    transport_register_receive_callback(TRAFFIC_PUBLISH_SUBSCRIBE, on_receive_publish_subscribe)

    threading.Thread(target=rpc_server, daemon=True).start()
    threading.Thread(target=pubsub_server, daemon=True).start()

    # Automatic control-plane convergence
    threading.Thread(
        target=_delayed_refresh,
        daemon=True,
        name="pubsub-auto-refresh",
    ).start()

    # Process lifetime (never returns)
    server_setup(
        subsystem="PUBSUB",
        commands=COMMANDS,
        subscriptions={},
    )

def bootstrap() -> None:
    run()