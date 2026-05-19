"""
ZPNet Publish/Subscribe — Deterministic RPC Broker + Pub/Sub Router

Roles:
  • Pi-side device driver for Teensy (RPC + pub/sub transport)
  • Authoritative pub/sub router
  • First-class Pi process (PUBSUB) for orchestration & introspection
  • TCP bridge for SERVER machine (Meteor/Node.js via reverse tunnel)

IMPORTANT:
  • SUBSCRIBE / UNSUBSCRIBE are COMMANDS ONLY
  • Pub/Sub channel is DATA PLANE ONLY

SERVER integration:
  • Pubsub listens on a TCP port (localhost only)
  • SERVER connects through the SSH reverse tunnel
  • Wire protocol: newline-delimited JSON, bidirectional
  • SERVER is a first-class machine with subscriptions and commands
  • One persistent connection at a time; graceful reconnect

SERVER command relay:
  • Pi-side processes send commands to SERVER via send_command(machine="SERVER")
  • These arrive at the SERVER_CMD_SOCKET Unix socket owned by pubsub
  • Pubsub relays them over the TCP bridge and returns the response
  • Symmetric with the Teensy RPC relay (RPC_SOCKET_PATH)
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

# Transport retry configuration
TRANSPORT_RETRY_INTERVAL_S = 0.25   # poll every 250 ms — aggressive but not a spinlock

# Subsystems that PUBSUB should never query during ALLSUBSCRIPTIONS.
#
# Exact skips are long-lived utility/test surfaces. Prefix skips are especially
# important for transient test programs whose socket names include historical
# PIDs. Those sockets can remain in /tmp after the owning process exits and can
# poison route convergence if PUBSUB treats them as normal services.
SUBSYSTEM_SKIP = {"TIMEBASE_WATCH"}
SUBSYSTEM_SKIP_PREFIXES = (
    "CLOCK_WITNESS_CYCLES_",
)

# PI-side subscription discovery is an external boundary. SUBSCRIPTIONS should
# be answered immediately by the injected process runtime command, so use a
# short, explicit timeout here. This prevents a stale but connectable Unix
# socket from holding REFRESH hostage.
PI_SUBSCRIPTIONS_RETRIES = 1
PI_SUBSCRIPTIONS_RETRY_DELAY_S = 0.05
PI_SUBSCRIPTIONS_TIMEOUT_S = 1.0

# SERVER TCP configuration
SERVER_TCP_HOST = "127.0.0.1"     # localhost only — SERVER arrives via reverse tunnel
SERVER_TCP_PORT = 9800            # configurable; must match SERVER-side client
SERVER_TCP_BACKLOG = 1            # one connection at a time

# SERVER command relay socket (Pi → SERVER via pubsub TCP bridge)
# Symmetric with RPC_SOCKET_PATH (Pi → TEENSY via pubsub HID/serial)
SERVER_CMD_SOCKET_PATH = "/tmp/zpnet_server_cmd.sock"
SERVER_CMD_BACKLOG = 4
SERVER_CMD_TIMEOUT_S = 30.0       # max wait for SERVER to respond

# REFRESH instrumentation
#
# These are observational only. They do not change routing semantics.
# A stuck REFRESH is usually blocked inside a synchronous boundary, so a
# separate watchdog thread periodically reports the last declared stage.
REFRESH_STUCK_LOG_INTERVAL_S = 5.0
REFRESH_HISTORY_MAX = 25
ROUTE_NO_TARGET_LOG_INTERVAL_S = 10.0
ROUTE_TRACE_TOPICS = {"TIMEBASE_FRAGMENT", "WATCHDOG_ANOMALY", "CLOCK_WITNESS"}

# Single-flight REFRESH behavior. REFRESH is a convergence transaction, not a
# concurrent workload. If one REFRESH is already in flight, automatic triggers
# retry rather than overlapping another transaction. Manual REFRESH receives a
# truthful busy response.
REFRESH_BUSY_RETRY_DELAY_S = 1.0
AUTO_REFRESH_ATTEMPTS = 8
SERVER_SUBSCRIBE_REFRESH_ATTEMPTS = 8

# ---------------------------------------------------------------------
# Global state
# ---------------------------------------------------------------------

pending_replies: Dict[int, Queue] = {}
req_id_counter = itertools.count(1)

# ---------------------------------------------------------------------
# REFRESH instrumentation state
# ---------------------------------------------------------------------

refresh_id_counter = itertools.count(1)
refresh_state_lock = threading.Lock()
refresh_single_flight_lock = threading.Lock()
active_refreshes: Dict[int, Dict[str, Any]] = {}
refresh_history: List[Dict[str, Any]] = []

# Topic-level no-route warnings are rate-limited so a missing TIMEBASE route
# can be seen clearly without flooding syslog every second.
route_no_target_last_log: Dict[str, float] = {}

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

# ---------------------------------------------------------------------
# SERVER connection state
# ---------------------------------------------------------------------
#
# Persistent bidirectional TCP connection to the SERVER machine.
#
# server_conn is the live socket (or None if SERVER is not connected).
# server_conn_lock serializes writes and connection lifecycle.
# server_subscriptions holds the most recent subscription declaration
# from SERVER, in the same canonical shape as PI/TEENSY declarations.
#
# server_pending_commands holds pending command relay requests:
#   req_id → Queue(maxsize=1)
# These are commands sent FROM Pi-side processes TO SERVER.
# The reader thread deposits SERVER's response into the queue.
#
# Semantics:
#   • One connection at a time
#   • Reader thread owns recv; writer path is shared via lock
#   • Disconnect clears server_conn and server_subscriptions
#   • Reconnect replaces both atomically
#
# ---------------------------------------------------------------------

server_conn: Optional[socket.socket] = None
server_conn_lock = threading.Lock()
server_subscriptions: List[Dict[str, Any]] = []
server_pending_commands: Dict[int, Queue] = {}

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
# Instrumentation helpers
# ---------------------------------------------------------------------

def _ms_since(start: float) -> int:
    return int((time.monotonic() - start) * 1000)


def _fmt_kv(fields: Dict[str, Any]) -> str:
    parts = []
    for key in sorted(fields.keys()):
        value = fields[key]
        if isinstance(value, float):
            parts.append(f"{key}={value:.3f}")
        else:
            parts.append(f"{key}={value!r}")
    return " ".join(parts)


def _subscription_summary(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    machine_counts: Dict[str, int] = {}
    topic_count = 0

    for row in rows:
        machine = str(row.get("machine") or "?")
        machine_counts[machine] = machine_counts.get(machine, 0) + 1
        subs = row.get("subscriptions") or []
        if isinstance(subs, list):
            topic_count += len(subs)

    return {
        "rows": len(rows),
        "topic_declarations": topic_count,
        "machines": machine_counts,
    }


def _route_summary(routes: Dict[str, Set[Tuple[str, str]]]) -> Dict[str, int]:
    edge_count = 0
    for targets in routes.values():
        edge_count += len(targets)
    return {
        "topics": len(routes),
        "edges": edge_count,
    }


def _log_route_watch_topics(refresh_id: Optional[int], routes: Dict[str, Set[Tuple[str, str]]]) -> None:
    for topic in sorted(ROUTE_TRACE_TOPICS):
        targets = sorted(routes.get(topic, set()))
        if targets:
            logging.info(
                "🧭 [pubsub.routes] watched topic=%s targets=%d %s",
                topic,
                len(targets),
                targets,
            )
            _refresh_stage(
                refresh_id,
                "refresh.routes.watch_topic.present",
                topic=topic,
                targets=len(targets),
                target_list=targets,
            )
        else:
            logging.warning(
                "🕳️ [pubsub.routes] watched topic has no targets topic=%s route_topics=%d",
                topic,
                len(routes),
            )
            _refresh_stage(
                refresh_id,
                "refresh.routes.watch_topic.missing",
                topic=topic,
                route_topics=len(routes),
            )


def _payload_size_bytes(payload: Any) -> int:
    try:
        return len(payload_to_json_bytes(payload))
    except Exception:
        try:
            return len(json.dumps(payload, separators=(",", ":")).encode("utf-8"))
        except Exception:
            return -1


def _args_refresh_id(args: Optional[dict]) -> Optional[int]:
    if not isinstance(args, dict):
        return None

    raw = args.get("_refresh_id", args.get("refresh_id"))
    if raw is None:
        return None

    try:
        return int(raw)
    except (TypeError, ValueError):
        return None


def _args_origin(args: Optional[dict], default: str) -> str:
    if not isinstance(args, dict):
        return default

    origin = args.get("origin") or args.get("_origin")
    if origin is None:
        return default

    return str(origin)


def _subsystem_skip_reason(subsystem: str) -> Optional[str]:
    if subsystem in SUBSYSTEM_SKIP:
        return "SUBSYSTEM_SKIP"

    for prefix in SUBSYSTEM_SKIP_PREFIXES:
        if subsystem.startswith(prefix):
            return f"SUBSYSTEM_SKIP_PREFIX:{prefix}"

    return None


def _refresh_busy_payload(origin: str) -> Dict[str, Any]:
    with refresh_state_lock:
        active = [dict(v) for v in active_refreshes.values()]

    now = time.monotonic()
    for item in active:
        started = float(item.get("started_monotonic", now))
        stage_since = float(item.get("stage_since_monotonic", now))
        item["elapsed_s"] = round(now - started, 3)
        item["stage_elapsed_s"] = round(now - stage_since, 3)
        item.pop("started_monotonic", None)
        item.pop("stage_since_monotonic", None)

    logging.warning(
        "🔁 [pubsub.refresh] busy origin=%s active_refreshes=%d active=%s",
        origin,
        len(active),
        active,
    )

    return {
        "success": False,
        "message": "REFRESH already active",
        "payload": {
            "origin": origin,
            "active": active,
        },
    }


def _is_refresh_busy_response(resp: Dict[str, Any]) -> bool:
    return resp.get("message") == "REFRESH already active"


def _refresh_with_retry(
    *,
    origin: str,
    label: str,
    attempts: int,
    delay_s: float,
) -> Dict[str, Any]:
    last_resp: Dict[str, Any] = {
        "success": False,
        "message": "REFRESH not attempted",
        "payload": {},
    }

    for attempt in range(1, attempts + 1):
        try:
            logging.info(
                "🔄 [pubsub] %s REFRESH starting attempt=%d/%d origin=%s",
                label,
                attempt,
                attempts,
                origin,
            )
            resp = cmd_refresh({"origin": origin})
            last_resp = resp

            if resp.get("success"):
                logging.info(
                    "✅ [pubsub] %s REFRESH complete attempt=%d/%d message=%s",
                    label,
                    attempt,
                    attempts,
                    resp.get("message"),
                )
                return resp

            if _is_refresh_busy_response(resp) and attempt < attempts:
                logging.warning(
                    "🔁 [pubsub] %s REFRESH busy attempt=%d/%d; retrying in %.3fs",
                    label,
                    attempt,
                    attempts,
                    delay_s,
                )
                time.sleep(delay_s)
                continue

            logging.warning(
                "❌ [pubsub] %s REFRESH returned failure attempt=%d/%d message=%s payload_keys=%s",
                label,
                attempt,
                attempts,
                resp.get("message"),
                sorted((resp.get("payload") or {}).keys()),
            )
            return resp

        except Exception as e:
            logging.exception(
                "❌ [pubsub] %s REFRESH raised attempt=%d/%d error=%r",
                label,
                attempt,
                attempts,
                e,
            )
            last_resp = {
                "success": False,
                "message": "REFRESH raised exception",
                "payload": {"error": repr(e)},
            }
            if attempt < attempts:
                time.sleep(delay_s)

    return last_resp


def _refresh_begin(origin: str) -> int:
    refresh_id = next(refresh_id_counter)
    now = time.monotonic()
    with refresh_state_lock:
        active_refreshes[refresh_id] = {
            "id": refresh_id,
            "origin": origin,
            "thread": threading.current_thread().name,
            "started_monotonic": now,
            "stage": "begin",
            "stage_since_monotonic": now,
            "last_fields": {},
        }
        active_count = len(active_refreshes)

    if active_count > 1:
        logging.warning(
            "🔁 [pubsub.refresh:%d] begin origin=%s thread=%s concurrent_refreshes=%d",
            refresh_id, origin, threading.current_thread().name, active_count,
        )
    else:
        logging.info(
            "🔄 [pubsub.refresh:%d] begin origin=%s thread=%s",
            refresh_id, origin, threading.current_thread().name,
        )

    return refresh_id


def _refresh_stage(refresh_id: Optional[int], stage: str, **fields: Any) -> None:
    if refresh_id is None:
        return

    now = time.monotonic()

    with refresh_state_lock:
        state = active_refreshes.get(refresh_id)
        if state is not None:
            started = float(state.get("started_monotonic", now))
            prior_stage_since = float(state.get("stage_since_monotonic", now))
            state["stage"] = stage
            state["stage_since_monotonic"] = now
            state["last_fields"] = dict(fields)
        else:
            started = now
            prior_stage_since = now

    logging.info(
        "🔎 [pubsub.refresh:%d] stage=%s elapsed_ms=%d prior_stage_ms=%d %s",
        refresh_id,
        stage,
        int((now - started) * 1000),
        int((now - prior_stage_since) * 1000),
        _fmt_kv(fields),
    )


def _refresh_finish(refresh_id: int, success: bool, message: str, **fields: Any) -> None:
    now = time.monotonic()

    with refresh_state_lock:
        state = active_refreshes.pop(refresh_id, None)
        active_count = len(active_refreshes)

    if state is None:
        elapsed_ms = 0
        origin = "?"
        thread = "?"
        stage = "?"
    else:
        elapsed_ms = int((now - float(state.get("started_monotonic", now))) * 1000)
        origin = str(state.get("origin", "?"))
        thread = str(state.get("thread", "?"))
        stage = str(state.get("stage", "?"))

    history_entry = {
        "id": refresh_id,
        "origin": origin,
        "thread": thread,
        "success": success,
        "message": message,
        "elapsed_ms": elapsed_ms,
        "last_stage": stage,
        "finished_monotonic": now,
        "fields": dict(fields),
    }

    with refresh_state_lock:
        refresh_history.append(history_entry)
        del refresh_history[:-REFRESH_HISTORY_MAX]

    log_fn = logging.info if success else logging.warning
    log_fn(
        "%s [pubsub.refresh:%d] finish success=%s elapsed_ms=%d origin=%s "
        "thread=%s last_stage=%s active_refreshes=%d message=%s %s",
        "✅" if success else "❌",
        refresh_id,
        success,
        elapsed_ms,
        origin,
        thread,
        stage,
        active_count,
        message,
        _fmt_kv(fields),
    )


def _refresh_snapshot() -> Dict[str, Any]:
    with refresh_state_lock:
        active = [dict(v) for v in active_refreshes.values()]
        history = list(refresh_history)

    now = time.monotonic()
    for item in active:
        started = float(item.get("started_monotonic", now))
        stage_since = float(item.get("stage_since_monotonic", now))
        item["elapsed_s"] = round(now - started, 3)
        item["stage_elapsed_s"] = round(now - stage_since, 3)
        item.pop("started_monotonic", None)
        item.pop("stage_since_monotonic", None)

    for item in history:
        item.pop("finished_monotonic", None)

    return {
        "active": active,
        "history": history[-10:],
    }


def refresh_watchdog_loop() -> None:
    while True:
        time.sleep(REFRESH_STUCK_LOG_INTERVAL_S)

        now = time.monotonic()
        with refresh_state_lock:
            snapshot = [dict(v) for v in active_refreshes.values()]

        if not snapshot:
            continue

        with state_lock:
            pending_count = len(pending_replies)
            pending_ids = list(pending_replies.keys())[:20]
            route_topics = len(routes_by_topic)
            server_sub_count = len(server_subscriptions)

        for item in snapshot:
            refresh_id = int(item.get("id", -1))
            started = float(item.get("started_monotonic", now))
            stage_since = float(item.get("stage_since_monotonic", now))
            logging.warning(
                "⏱️ [pubsub.refresh:%d] still active elapsed_s=%.3f "
                "stage=%s stage_elapsed_s=%.3f origin=%s thread=%s "
                "pending_replies=%d pending_ids=%s route_topics=%d "
                "server_subscriptions=%d last_fields=%s",
                refresh_id,
                now - started,
                item.get("stage"),
                now - stage_since,
                item.get("origin"),
                item.get("thread"),
                pending_count,
                pending_ids,
                route_topics,
                server_sub_count,
                item.get("last_fields"),
            )


def _log_no_targets(topic: str, forward_to_teensy: bool) -> None:
    if topic not in ROUTE_TRACE_TOPICS:
        return

    now = time.monotonic()
    last = route_no_target_last_log.get(topic, 0.0)
    if now - last < ROUTE_NO_TARGET_LOG_INTERVAL_S:
        return

    route_no_target_last_log[topic] = now
    logging.warning(
        "🕳️ [pubsub.route] no targets for topic=%s forward_to_teensy=%s "
        "routes_have_topics=%d",
        topic,
        forward_to_teensy,
        len(routes_by_topic),
    )



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

    with state_lock:
        pending_count = len(pending_replies)

    logging.info(
        "📩 [pubsub.rpc] TEENSY response req_id=%s latency_ms=%s success=%s "
        "message=%r pending_after=%d",
        req_id,
        latency,
        payload.get("success"),
        payload.get("message"),
        pending_count,
    )

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
        subsystem = req.get("subsystem")
        command = req.get("command")

        logging.info(
            "➡️ [pubsub.rpc] PI->TEENSY request subsystem=%s command=%s bytes=%d",
            subsystem,
            command,
            len(raw),
        )

        req_id = None

        for attempt in range(MAX_TEENSY_RETRIES):
            # NEW: fresh req_id per attempt
            req_id = next(req_id_counter)
            req["req_id"] = req_id
            req["req_ts_ms"] = int(time.monotonic() * 1000)

            q = Queue(maxsize=1)
            with state_lock:
                pending_replies[req_id] = q
                pending_count = len(pending_replies)

            attempt_start = time.monotonic()

            try:
                logging.info(
                    "➡️ [pubsub.rpc] send TEENSY attempt=%d/%d req_id=%s "
                    "subsystem=%s command=%s pending=%d payload_bytes=%d",
                    attempt + 1,
                    MAX_TEENSY_RETRIES,
                    req_id,
                    subsystem,
                    command,
                    pending_count,
                    _payload_size_bytes(req),
                )

                transport_send(TRAFFIC_REQUEST_RESPONSE, req)

                reply = q.get(timeout=REPLY_TIMEOUT_S)
                elapsed_ms = _ms_since(attempt_start)
                logging.info(
                    "⬅️ [pubsub.rpc] reply TEENSY req_id=%s subsystem=%s "
                    "command=%s elapsed_ms=%d success=%s message=%r",
                    req_id,
                    subsystem,
                    command,
                    elapsed_ms,
                    reply.get("success"),
                    reply.get("message"),
                )
                conn.sendall(
                    json.dumps(reply, separators=(",", ":")).encode()
                )
                return

            except Empty:
                elapsed_ms = _ms_since(attempt_start)
                logging.warning(
                    "⌛ [pubsub.rpc] TEENSY timeout req_id=%s subsystem=%s "
                    "command=%s attempt=%d/%d elapsed_ms=%d",
                    req_id,
                    subsystem,
                    command,
                    attempt + 1,
                    MAX_TEENSY_RETRIES,
                    elapsed_ms,
                )
                # Timeout on this attempt — clean up and retry
                with state_lock:
                    pending_replies.pop(req_id, None)
                continue

        logging.warning(
            "❌ [pubsub.rpc] TEENSY exhausted retries subsystem=%s command=%s "
            "last_req_id=%s",
            subsystem,
            command,
            req_id,
        )

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
      • Forward to SERVER via persistent TCP connection if:
          - at least one SERVER target exists for the topic
          - SERVER is connected
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

    if not targets:
        _log_no_targets(topic, forward_to_teensy)

    raw = payload_to_json_bytes(msg)

    teensy_needed = False
    server_needed = False

    for machine, subsystem in targets:
        if machine == "TEENSY":
            teensy_needed = True
            continue

        if machine == "SERVER":
            server_needed = True
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

    if server_needed:
        _server_send_publication(msg)


def _server_send_publication(msg: Dict[str, Any]) -> None:
    """
    Forward a publication to SERVER over the persistent TCP connection.

    Wire format: newline-delimited JSON with type=publish.
    Best-effort: if SERVER is disconnected, silently skip.
    """
    wire_msg = {
        "type": "publish",
        "topic": msg.get("topic"),
        "payload": msg.get("payload"),
    }

    _server_send(wire_msg)


def _server_send(wire_msg: Dict[str, Any]) -> None:
    """
    Send a single wire message to SERVER.

    Serializes as compact JSON + newline.
    Thread-safe via server_conn_lock.
    Best-effort: disconnects are logged, not fatal.
    """
    with server_conn_lock:
        conn = server_conn
        if conn is None:
            return

        try:
            line = json.dumps(wire_msg, separators=(",", ":")) + "\n"
            conn.sendall(line.encode("utf-8"))
        except Exception:
            logging.warning("⚠️ [pubsub] SERVER send failed (disconnected?)")


# ---------------------------------------------------------------------
# SERVER TCP listener and connection handler
# ---------------------------------------------------------------------

def server_tcp_listener() -> None:
    """
    Accept TCP connections from SERVER.

    Binds to localhost on SERVER_TCP_PORT.
    Accepts one connection at a time.
    Each connection spawns a reader thread and replaces any prior connection.
    """
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((SERVER_TCP_HOST, SERVER_TCP_PORT))
    srv.listen(SERVER_TCP_BACKLOG)

    logging.info("🌐 [pubsub] SERVER TCP listener on %s:%d", SERVER_TCP_HOST, SERVER_TCP_PORT)

    while True:
        conn, addr = srv.accept()
        logging.info("🌐 [pubsub] SERVER connected from %s:%d", addr[0], addr[1])

        # Replace any prior connection
        _server_disconnect("new connection replacing old")

        with server_conn_lock:
            global server_conn
            server_conn = conn

        threading.Thread(
            target=_server_reader,
            args=(conn,),
            daemon=True,
            name="server-tcp-reader",
        ).start()


def _server_disconnect(reason: str) -> None:
    """
    Cleanly tear down the SERVER connection.

    Clears the socket, subscription state, and any pending command relays.
    Pending command relays receive an error response so callers don't hang.
    """
    global server_conn

    with server_conn_lock:
        old = server_conn
        server_conn = None

    if old is not None:
        try:
            old.close()
        except Exception:
            pass
        logging.info("🌐 [pubsub] SERVER disconnected (%s)", reason)

    with state_lock:
        server_subscriptions.clear()

        # Unblock any Pi-side callers waiting for SERVER command responses
        for req_id, q in server_pending_commands.items():
            q.put({
                "success": False,
                "message": "SERVER disconnected",
                "payload": {},
            })
        server_pending_commands.clear()


def _server_reader(conn: socket.socket) -> None:
    """
    Read newline-delimited JSON messages from SERVER.

    Each message has a "type" field:
      • "subscribe"  — SERVER declares its subscriptions
      • "command"    — SERVER sends a command to PI or TEENSY
      • "publish"    — SERVER publishes to the bus
      • "response"   — SERVER returns a response to a relayed command

    On disconnect (EOF or error), cleans up and exits.
    """
    buf = b""

    try:
        while True:
            chunk = conn.recv(65536)
            if not chunk:
                break

            buf += chunk

            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue

                msg = json.loads(line.decode("utf-8"))
                msg_type = msg.get("type")

                if msg_type == "subscribe":
                    _server_handle_subscribe(msg)
                elif msg_type == "command":
                    _server_handle_command(msg, conn)
                elif msg_type == "publish":
                    _server_handle_publish(msg)
                elif msg_type == "response":
                    _server_handle_response(msg)
                else:
                    logging.warning("⚠️ [pubsub] SERVER sent unknown message type: %r", msg_type)

    except Exception:
        logging.exception("💥 [pubsub] SERVER reader exception")
    finally:
        _server_disconnect("reader exited")


def _server_handle_subscribe(msg: Dict[str, Any]) -> None:
    """
    Process a subscription declaration from SERVER.

    Expected shape:
        {
            "type": "subscribe",
            "subscriptions": [
                {
                    "machine": "SERVER",
                    "subsystem": "SYSTEM",
                    "subscriptions": [
                        {"name": "EVENTS"},
                        {"name": "TIMEBASE"}
                    ]
                }
            ]
        }

    This replaces the entire SERVER subscription set.
    A REFRESH is triggered automatically to rebuild routes.
    """
    with state_lock:
        server_subscriptions.clear()
        server_subscriptions.extend(msg.get("subscriptions", []))

    logging.info(
        "🌐 [pubsub] SERVER subscriptions updated (%d entries)",
        len(server_subscriptions),
    )

    # Auto-refresh routes to include SERVER subscriptions. Run in its own
    # worker so the SERVER reader is never held hostage by route convergence.
    threading.Thread(
        target=_refresh_with_retry,
        kwargs={
            "origin": "server-subscribe",
            "label": "SERVER subscribe-triggered",
            "attempts": SERVER_SUBSCRIBE_REFRESH_ATTEMPTS,
            "delay_s": REFRESH_BUSY_RETRY_DELAY_S,
        },
        daemon=True,
        name="pubsub-refresh-server-subscribe",
    ).start()


def _server_handle_command(msg: Dict[str, Any], conn: socket.socket) -> None:
    """
    Route a command from SERVER to the target machine.

    Expected shape:
        {
            "type": "command",
            "req_id": <int>,       (SERVER-assigned, returned in response)
            "machine": "PI"|"TEENSY",
            "subsystem": "CLOCKS",
            "command": "REPORT",
            "args": {...}          (optional)
        }

    The response is sent back to SERVER on the same connection as:
        {
            "type": "response",
            "req_id": <int>,
            "success": true|false,
            "message": "...",
            "payload": {...}
        }
    """
    server_req_id = msg.get("req_id")
    machine = msg.get("machine")
    subsystem = msg.get("subsystem")
    command = msg.get("command")
    args = msg.get("args")

    try:
        if machine == "TEENSY":
            resp = _server_command_to_teensy(subsystem, command, args)
        elif machine == "PI":
            resp = send_command(
                machine="PI",
                subsystem=subsystem,
                command=command,
                args=args,
            )
        else:
            resp = {
                "success": False,
                "message": f"unknown target machine: {machine}",
                "payload": {},
            }
    except Exception as e:
        resp = {
            "success": False,
            "message": str(e),
            "payload": {},
        }

    wire_resp = {
        "type": "response",
        "req_id": server_req_id,
        "success": resp.get("success", False),
        "message": resp.get("message", ""),
        "payload": resp.get("payload", {}),
    }

    _server_send(wire_resp)


def _server_command_to_teensy(
    subsystem: str,
    command: str,
    args: Optional[Dict[str, Any]],
) -> Dict[str, Any]:
    """
    Route a SERVER-originated command to TEENSY via the RPC path.

    Uses the same req_id / pending_replies / transport_send mechanism
    as handle_client, but without the Unix socket wrapper.
    """
    req: Dict[str, Any] = {
        "machine": "TEENSY",
        "subsystem": subsystem,
        "command": command,
    }
    if args is not None:
        req["args"] = args

    for attempt in range(MAX_TEENSY_RETRIES):
        req_id = next(req_id_counter)
        req["req_id"] = req_id
        req["req_ts_ms"] = int(time.monotonic() * 1000)

        q = Queue(maxsize=1)
        with state_lock:
            pending_replies[req_id] = q
            pending_count = len(pending_replies)

        attempt_start = time.monotonic()

        try:
            logging.info(
                "➡️ [pubsub.rpc] SERVER->TEENSY send attempt=%d/%d req_id=%s "
                "subsystem=%s command=%s pending=%d payload_bytes=%d",
                attempt + 1,
                MAX_TEENSY_RETRIES,
                req_id,
                subsystem,
                command,
                pending_count,
                _payload_size_bytes(req),
            )
            transport_send(TRAFFIC_REQUEST_RESPONSE, req)
            reply = q.get(timeout=REPLY_TIMEOUT_S)
            logging.info(
                "⬅️ [pubsub.rpc] SERVER->TEENSY reply req_id=%s subsystem=%s "
                "command=%s elapsed_ms=%d success=%s message=%r",
                req_id,
                subsystem,
                command,
                _ms_since(attempt_start),
                reply.get("success"),
                reply.get("message"),
            )
            return reply

        except Empty:
            logging.warning(
                "⌛ [pubsub.rpc] SERVER->TEENSY timeout req_id=%s subsystem=%s "
                "command=%s attempt=%d/%d elapsed_ms=%d",
                req_id,
                subsystem,
                command,
                attempt + 1,
                MAX_TEENSY_RETRIES,
                _ms_since(attempt_start),
            )
            with state_lock:
                pending_replies.pop(req_id, None)
            continue

    return {
        "success": False,
        "message": "TEENSY did not respond",
        "payload": {},
    }


def _server_handle_publish(msg: Dict[str, Any]) -> None:
    """
    Handle a publication from SERVER.

    SERVER publishes are routed to all subscribers (PI + TEENSY)
    but NOT echoed back to SERVER.
    """
    pub_msg = {
        "topic": msg.get("topic"),
        "payload": msg.get("payload"),
    }

    # Route to PI and TEENSY targets, but do not fan-out back to SERVER.
    # We achieve this by routing normally and relying on the fact that
    # SERVER targets in route_publish will send back to the same connection.
    # However, to avoid echo, we route without SERVER targets.

    topic = pub_msg.get("topic")
    if not topic:
        return

    log_pubsub(pub_msg)

    with state_lock:
        topic_routes = routes_by_topic.get(topic, set())

    targets = set(topic_routes)
    raw = payload_to_json_bytes(pub_msg)

    teensy_needed = False

    for machine, subsystem in targets:
        if machine == "TEENSY":
            teensy_needed = True
            continue

        if machine == "SERVER":
            # Do NOT echo back to the originator
            continue

        # PI target
        sock_path = f"{SOCKET_DIR}/zpnet-{subsystem.lower()}-pubsub.sock"
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                sock.connect(sock_path)
                sock.sendall(raw)
                sock.shutdown(socket.SHUT_WR)
        except Exception:
            logging.warning("⚠️ [pubsub] fan-out failed %s -> %s:%s", topic, machine, subsystem)

    if teensy_needed:
        transport_send(TRAFFIC_PUBLISH_SUBSCRIBE, pub_msg)


def _server_handle_response(msg: Dict[str, Any]) -> None:
    """
    Handle a command response from SERVER.

    This is the return path for commands relayed from Pi-side
    processes to SERVER via the SERVER_CMD_SOCKET.

    Expected shape:
        {
            "type": "response",
            "req_id": <int>,
            "success": true|false,
            "message": "...",
            "payload": {...}
        }

    The req_id is used to find the waiting Queue and unblock
    the caller in server_cmd_relay_server.
    """
    req_id = msg.get("req_id")
    if req_id is None:
        logging.warning("⚠️ [pubsub] SERVER response missing req_id — discarded")
        return

    with state_lock:
        q = server_pending_commands.pop(req_id, None)

    if q is None:
        logging.warning(
            "⚠️ [pubsub] SERVER response for unknown req_id=%s — discarded",
            req_id,
        )
        return

    q.put(msg)


# ---------------------------------------------------------------------
# SERVER command relay (Pi → SERVER)
# ---------------------------------------------------------------------
#
# This is the reverse direction: a Pi-side process calls
# send_command(machine="SERVER", ...) which connects to
# SERVER_CMD_SOCKET_PATH.  Pubsub accepts the connection,
# forwards the command over the TCP bridge, and blocks until
# SERVER responds (or timeout).
#
# Structurally symmetric with handle_client (Pi → TEENSY relay).
#
# ---------------------------------------------------------------------

def server_cmd_relay_server() -> None:
    """
    Accept Pi-side command requests destined for SERVER.

    Binds SERVER_CMD_SOCKET_PATH.
    Each connection is one synchronous command relay.
    """
    if os.path.exists(SERVER_CMD_SOCKET_PATH):
        os.unlink(SERVER_CMD_SOCKET_PATH)

    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SERVER_CMD_SOCKET_PATH)
    srv.listen(SERVER_CMD_BACKLOG)

    logging.info("🌐 [pubsub] SERVER command relay on %s", SERVER_CMD_SOCKET_PATH)

    while True:
        conn, _ = srv.accept()
        threading.Thread(
            target=_handle_server_cmd_client,
            args=(conn,),
            daemon=True,
            name="server-cmd-relay",
        ).start()


def _handle_server_cmd_client(conn: socket.socket) -> None:
    """
    Relay a single command from a Pi-side caller to SERVER.

    Protocol (identical to Teensy RPC from the caller's perspective):
      1. Read JSON request from Unix socket
      2. Assign a req_id and forward to SERVER over TCP
      3. Block on Queue until SERVER responds or timeout
      4. Return response to caller on Unix socket

    If SERVER is not connected, returns an immediate error.
    """
    try:
        raw = conn.recv(65536)
        if not raw:
            return

        req = json.loads(raw.decode("utf-8"))

        # Check SERVER connectivity before attempting relay
        with server_conn_lock:
            if server_conn is None:
                conn.sendall(json.dumps({
                    "success": False,
                    "message": "SERVER not connected",
                    "payload": {},
                }, separators=(",", ":")).encode("utf-8"))
                return

        # Assign a relay req_id (internal to pubsub, not the caller's)
        req_id = next(req_id_counter)

        q = Queue(maxsize=1)
        with state_lock:
            server_pending_commands[req_id] = q

        # Forward to SERVER over TCP
        wire_msg = {
            "type": "command",
            "req_id": req_id,
            "machine": "SERVER",
            "subsystem": req.get("subsystem"),
            "command": req.get("command"),
        }
        if req.get("args") is not None:
            wire_msg["args"] = req["args"]

        _server_send(wire_msg)

        # Block until SERVER responds or timeout
        try:
            resp = q.get(timeout=SERVER_CMD_TIMEOUT_S)

            # Return the semantic response, stripping wire envelope
            relay_resp = {
                "success": resp.get("success", False),
                "message": resp.get("message", ""),
                "payload": resp.get("payload", {}),
            }

        except Empty:
            with state_lock:
                server_pending_commands.pop(req_id, None)

            relay_resp = {
                "success": False,
                "message": "SERVER did not respond",
                "payload": {},
            }

        conn.sendall(
            json.dumps(relay_resp, separators=(",", ":")).encode("utf-8")
        )

    finally:
        conn.close()


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
        server_cmd_pending = len(server_pending_commands)
        route_summary = _route_summary(routes_by_topic)

    with server_conn_lock:
        server_connected = server_conn is not None

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "pending_reply_count": pending_count,
            "pending_req_ids": pending_ids[:50],  # cap for sanity
            "req_id_current": next(req_id_counter),
            "routes_topic_count": route_summary["topics"],
            "routes_edge_count": route_summary["edges"],
            "active_threads": threading.active_count(),
            "server_connected": server_connected,
            "server_subscription_count": len(server_subscriptions),
            "server_cmd_pending": server_cmd_pending,
            "refresh_single_flight_locked": refresh_single_flight_lock.locked(),
            "subsystem_skip": sorted(SUBSYSTEM_SKIP),
            "subsystem_skip_prefixes": list(SUBSYSTEM_SKIP_PREFIXES),
            "pi_subscriptions_retries": PI_SUBSCRIPTIONS_RETRIES,
            "pi_subscriptions_timeout_s": PI_SUBSCRIPTIONS_TIMEOUT_S,
            "refresh": _refresh_snapshot(),
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

def cmd_allsubscriptions(args: Optional[dict]) -> Dict[str, Any]:
    refresh_id = _args_refresh_id(args)
    results: List[Dict[str, Any]] = []

    subsystems = list_subsystems()
    logging.info(
        "🧭 [pubsub.allsubscriptions] begin refresh_id=%s subsystem_count=%d subsystems=%s",
        refresh_id,
        len(subsystems),
        subsystems,
    )
    _refresh_stage(
        refresh_id,
        "pi.allsubscriptions.begin",
        subsystem_count=len(subsystems),
        subsystems=subsystems,
    )

    started = time.monotonic()

    for subsystem in subsystems:
        subsystem_started = time.monotonic()

        # Skip known non-service subsystems and transient test utilities.
        # This protects REFRESH from stale /tmp sockets left by short-lived
        # diagnostic programs such as CLOCK_WITNESS_CYCLES_<pid>.
        skip_reason = _subsystem_skip_reason(subsystem)
        if skip_reason:
            logging.info(
                "⏭️ [pubsub.allsubscriptions] skip subsystem=%s reason=%s",
                subsystem,
                skip_reason,
            )
            _refresh_stage(
                refresh_id,
                "pi.subscriptions.query.skipped",
                subsystem=subsystem,
                reason=skip_reason,
            )
            continue

        try:
            # Self-query: answer directly
            if subsystem == "PUBSUB":
                logging.info(
                    "🧭 [pubsub.allsubscriptions] self subsystem=PUBSUB subscriptions=0"
                )
                results.append({
                    "machine": "PI",
                    "subsystem": "PUBSUB",
                    "subscriptions": [],
                })
                continue

            # Skip subsystems whose command socket doesn't exist
            sock_path = f"{SOCKET_DIR}/zpnet-{subsystem.lower()}-command.sock"
            if not os.path.exists(sock_path):
                logging.info(
                    "⏭️ [pubsub.allsubscriptions] skip subsystem=%s reason=no_socket path=%s",
                    subsystem,
                    sock_path,
                )
                continue

            _refresh_stage(
                refresh_id,
                "pi.subscriptions.query.begin",
                subsystem=subsystem,
                sock_path=sock_path,
            )
            logging.info(
                "📨 [pubsub.allsubscriptions] query begin subsystem=%s sock=%s",
                subsystem,
                sock_path,
            )

            # IPC for real subsystems. This is a synchronous boundary and one
            # of the most important places to see the last log line if REFRESH
            # stalls.
            resp = send_command(
                machine="PI",
                subsystem=subsystem,
                command="SUBSCRIPTIONS",
                retries=PI_SUBSCRIPTIONS_RETRIES,
                retry_delay_s=PI_SUBSCRIPTIONS_RETRY_DELAY_S,
                timeout_s=PI_SUBSCRIPTIONS_TIMEOUT_S,
            )

            payload = resp.get("payload")
            sub_count = 0
            if isinstance(payload, dict):
                sub_count = len(payload.get("subscriptions") or [])

            logging.info(
                "📬 [pubsub.allsubscriptions] query complete subsystem=%s "
                "elapsed_ms=%d success=%s message=%r subscriptions=%d",
                subsystem,
                _ms_since(subsystem_started),
                resp.get("success"),
                resp.get("message"),
                sub_count,
            )
            _refresh_stage(
                refresh_id,
                "pi.subscriptions.query.complete",
                subsystem=subsystem,
                elapsed_ms=_ms_since(subsystem_started),
                success=resp.get("success"),
                subscriptions=sub_count,
            )

            if payload:
                results.append(payload)

        except Exception as e:
            # Stale socket or subsystem not yet ready — skip quietly in terms of
            # behavior, but log visibly while diagnosing REFRESH brittleness.
            logging.warning(
                "⚠️ [pubsub.allsubscriptions] query failed subsystem=%s "
                "elapsed_ms=%d error=%r",
                subsystem,
                _ms_since(subsystem_started),
                e,
            )
            _refresh_stage(
                refresh_id,
                "pi.subscriptions.query.failed",
                subsystem=subsystem,
                elapsed_ms=_ms_since(subsystem_started),
                error=repr(e),
            )

    summary = _subscription_summary(results)
    logging.info(
        "🧭 [pubsub.allsubscriptions] complete refresh_id=%s elapsed_ms=%d "
        "rows=%d topic_declarations=%d machines=%s",
        refresh_id,
        _ms_since(started),
        summary["rows"],
        summary["topic_declarations"],
        summary["machines"],
    )
    _refresh_stage(
        refresh_id,
        "pi.allsubscriptions.complete",
        elapsed_ms=_ms_since(started),
        rows=summary["rows"],
        topic_declarations=summary["topic_declarations"],
        machines=summary["machines"],
    )

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "subscriptions": results
        },
    }

def cmd_unionsubscriptions(args: Optional[dict]) -> Dict[str, Any]:
    """
    Return the union of PI-side, TEENSY-side, and SERVER-side
    declared subscriptions.

    Semantics:
      • Observational only
      • No routing mutation
      • Best-effort
      • Canonical payload shape
    """

    refresh_id = _args_refresh_id(args)
    results: List[Dict[str, Any]] = []

    logging.info("🧬 [pubsub.union] begin refresh_id=%s", refresh_id)
    _refresh_stage(refresh_id, "union.begin")

    # 1) PI-side
    try:
        _refresh_stage(refresh_id, "union.pi.begin")
        pi_started = time.monotonic()
        pi_resp = cmd_allsubscriptions({"_refresh_id": refresh_id})
        pi_payload = pi_resp.get("payload", {}).get("subscriptions", [])
        results.extend(pi_payload)
        pi_summary = _subscription_summary(pi_payload)
        logging.info(
            "🧬 [pubsub.union] PI complete refresh_id=%s elapsed_ms=%d "
            "rows=%d topic_declarations=%d machines=%s",
            refresh_id,
            _ms_since(pi_started),
            pi_summary["rows"],
            pi_summary["topic_declarations"],
            pi_summary["machines"],
        )
        _refresh_stage(
            refresh_id,
            "union.pi.complete",
            elapsed_ms=_ms_since(pi_started),
            rows=pi_summary["rows"],
            topic_declarations=pi_summary["topic_declarations"],
            machines=pi_summary["machines"],
        )
    except Exception:
        logging.exception("⚠️ [pubsub] failed to collect PI subscriptions")
        _refresh_stage(refresh_id, "union.pi.failed")

    # 2) TEENSY-side
    try:
        _refresh_stage(refresh_id, "union.teensy.begin")
        teensy_started = time.monotonic()
        logging.info(
            "🧬 [pubsub.union] TEENSY ALLSUBSCRIPTIONS begin refresh_id=%s",
            refresh_id,
        )

        teensy_resp = send_command(
            machine="TEENSY",
            subsystem="PUBSUB",
            command="ALLSUBSCRIPTIONS",
        )
        teensy_payload = teensy_resp.get("payload", {}).get("subscriptions", [])

        results.extend(teensy_payload)
        teensy_summary = _subscription_summary(teensy_payload)
        logging.info(
            "🧬 [pubsub.union] TEENSY ALLSUBSCRIPTIONS complete refresh_id=%s "
            "elapsed_ms=%d success=%s message=%r rows=%d topic_declarations=%d",
            refresh_id,
            _ms_since(teensy_started),
            teensy_resp.get("success"),
            teensy_resp.get("message"),
            teensy_summary["rows"],
            teensy_summary["topic_declarations"],
        )
        _refresh_stage(
            refresh_id,
            "union.teensy.complete",
            elapsed_ms=_ms_since(teensy_started),
            success=teensy_resp.get("success"),
            rows=teensy_summary["rows"],
            topic_declarations=teensy_summary["topic_declarations"],
        )

    except Exception:
        logging.exception("⚠️ [pubsub] failed to collect TEENSY subscriptions")
        _refresh_stage(refresh_id, "union.teensy.failed")

    # 3) SERVER-side (from cached subscription declaration)
    with state_lock:
        server_rows = list(server_subscriptions)

    if server_rows:
        results.extend(server_rows)

    server_summary = _subscription_summary(server_rows)
    logging.info(
        "🧬 [pubsub.union] SERVER cached refresh_id=%s rows=%d topic_declarations=%d",
        refresh_id,
        server_summary["rows"],
        server_summary["topic_declarations"],
    )
    _refresh_stage(
        refresh_id,
        "union.server.cached",
        rows=server_summary["rows"],
        topic_declarations=server_summary["topic_declarations"],
    )

    # 4) Canonical ordering
    results.sort(key=lambda x: (x.get("machine"), x.get("subsystem")))

    summary = _subscription_summary(results)
    logging.info(
        "🧬 [pubsub.union] complete refresh_id=%s rows=%d "
        "topic_declarations=%d payload_bytes=%d machines=%s",
        refresh_id,
        summary["rows"],
        summary["topic_declarations"],
        _payload_size_bytes({"subscriptions": results}),
        summary["machines"],
    )
    _refresh_stage(
        refresh_id,
        "union.complete",
        rows=summary["rows"],
        topic_declarations=summary["topic_declarations"],
        payload_bytes=_payload_size_bytes({"subscriptions": results}),
        machines=summary["machines"],
    )

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "subscriptions": results
        },
    }

def cmd_refresh(args: Optional[dict]) -> Dict[str, Any]:
    """
    REFRESH — commit canonical subscription truth.

    Semantics:
      • Collect PI + TEENSY + SERVER declared subscriptions
      • Union them into a single canonical payload
      • Commit that payload verbatim to:
          1) TEENSY (SETSUBSCRIPTIONS)
          2) PI (local routing state)
      • SERVER subscriptions are included in the routing table
        but are NOT forwarded to TEENSY (TEENSY does not need to
        know about SERVER; pubsub handles SERVER fan-out)

    Concurrency doctrine:
      • REFRESH is single-flight. It is a convergence transaction, so
        overlapping refreshes are not allowed to race each other.
      • Automatic triggers retry when busy. Manual callers receive an
        explicit "REFRESH already active" response.
    """

    origin = _args_origin(args, "command")

    if not refresh_single_flight_lock.acquire(blocking=False):
        return _refresh_busy_payload(origin)

    refresh_id = _refresh_begin(origin)
    refresh_started = time.monotonic()

    try:
        try:
            # 1) Observe + union (single source of truth)
            _refresh_stage(refresh_id, "refresh.union.begin")
            union_resp = cmd_unionsubscriptions({"_refresh_id": refresh_id})
            union_payload = union_resp.get("payload", {})
            union_rows = union_payload.get("subscriptions", [])
            union_summary = _subscription_summary(union_rows)
            _refresh_stage(
                refresh_id,
                "refresh.union.complete",
                rows=union_summary["rows"],
                topic_declarations=union_summary["topic_declarations"],
                payload_bytes=_payload_size_bytes(union_payload),
                machines=union_summary["machines"],
            )

            # 2) Commit union verbatim to TEENSY
            #    NOTE: TEENSY receives the full union including SERVER entries.
            #    This is harmless — TEENSY ignores subscriptions it doesn't
            #    originate, and the union is observational truth.
            try:
                _refresh_stage(
                    refresh_id,
                    "refresh.teensy.setsubscriptions.begin",
                    rows=union_summary["rows"],
                    topic_declarations=union_summary["topic_declarations"],
                    payload_bytes=_payload_size_bytes(union_payload),
                )
                teensy_started = time.monotonic()
                logging.info(
                    "📤 [pubsub.refresh:%d] TEENSY SETSUBSCRIPTIONS begin "
                    "rows=%d topic_declarations=%d payload_bytes=%d",
                    refresh_id,
                    union_summary["rows"],
                    union_summary["topic_declarations"],
                    _payload_size_bytes(union_payload),
                )

                teensy_resp = send_command(
                    machine="TEENSY",
                    subsystem="PUBSUB",
                    command="SETSUBSCRIPTIONS",
                    args=union_payload,
                )
                logging.info(
                    "📥 [pubsub.refresh:%d] TEENSY SETSUBSCRIPTIONS complete "
                    "elapsed_ms=%d success=%s message=%r",
                    refresh_id,
                    _ms_since(teensy_started),
                    teensy_resp.get("success"),
                    teensy_resp.get("message"),
                )
                _refresh_stage(
                    refresh_id,
                    "refresh.teensy.setsubscriptions.complete",
                    elapsed_ms=_ms_since(teensy_started),
                    success=teensy_resp.get("success"),
                    message=teensy_resp.get("message"),
                )
            except Exception as e:
                logging.warning(
                    "⚠️ [pubsub] REFRESH failed applying TEENSY state: %s",
                    e,
                    exc_info=True,
                )
                _refresh_finish(
                    refresh_id,
                    False,
                    "REFRESH failed (TEENSY SETSUBSCRIPTIONS error)",
                    elapsed_ms=_ms_since(refresh_started),
                    error=repr(e),
                )
                return {
                    "success": False,
                    "message": "REFRESH failed (TEENSY SETSUBSCRIPTIONS error)",
                    "payload": {
                        "union": union_payload,
                    },
                }

            if not teensy_resp.get("success", False):
                _refresh_finish(
                    refresh_id,
                    False,
                    "REFRESH failed (TEENSY rejected SETSUBSCRIPTIONS)",
                    elapsed_ms=_ms_since(refresh_started),
                    teensy_message=teensy_resp.get("message"),
                )
                return {
                    "success": False,
                    "message": "REFRESH failed (TEENSY rejected SETSUBSCRIPTIONS)",
                    "payload": {
                        "union": union_payload,
                        "teensy_response": teensy_resp,
                    },
                }

            # 3) Build machine-qualified routes_by_topic from union
            _refresh_stage(refresh_id, "refresh.routes.build.begin")
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

            route_summary = _route_summary(new_routes)
            _refresh_stage(
                refresh_id,
                "refresh.routes.build.complete",
                topics=route_summary["topics"],
                edges=route_summary["edges"],
            )
            _log_route_watch_topics(refresh_id, new_routes)

            # 4) Commit locally
            _refresh_stage(refresh_id, "refresh.routes.commit.begin")
            with state_lock:
                routes_by_topic.clear()
                routes_by_topic.update(new_routes)

                applied_union.clear()
                applied_union.update(union_payload)

            logging.info(
                "🚀 [pubsub] routes updated topics=%d edges=%d",
                route_summary["topics"],
                route_summary["edges"],
            )
            _refresh_stage(
                refresh_id,
                "refresh.routes.commit.complete",
                topics=route_summary["topics"],
                edges=route_summary["edges"],
            )

            _refresh_finish(
                refresh_id,
                True,
                "OK",
                topics=route_summary["topics"],
                edges=route_summary["edges"],
                rows=union_summary["rows"],
                topic_declarations=union_summary["topic_declarations"],
            )

            # 5) Return committed truth
            return {
                "success": True,
                "message": "OK",
                "payload": union_payload,
            }

        except Exception as e:
            logging.exception("❌ [pubsub.refresh:%d] unhandled exception", refresh_id)
            _refresh_finish(
                refresh_id,
                False,
                "REFRESH failed (unhandled exception)",
                elapsed_ms=_ms_since(refresh_started),
                error=repr(e),
            )
            raise

    finally:
        refresh_single_flight_lock.release()


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
    _refresh_with_retry(
        origin="startup-auto",
        label="auto",
        attempts=AUTO_REFRESH_ATTEMPTS,
        delay_s=REFRESH_BUSY_RETRY_DELAY_S,
    )

# ---------------------------------------------------------------------
# Transport init with aggressive retry
# ---------------------------------------------------------------------

def _transport_init_with_retry() -> None:
    """
    Attempt transport_init() in a tight silent loop until the HID
    device is available.  This handles the race condition where PUBSUB
    launches before the Teensy USB HID device has been enumerated by
    the kernel (e.g. during flash/reboot).

    Catches every transient failure mode observed during bring-up:
      • OSError / IOError   — HID device present but not responding
                               (errno 5: Input/output error)
      • FileNotFoundError   — /dev/zpnet-teensy-serial symlink not yet
                               created by udev
      • serial.SerialException — pyserial wrapper around the above
                               (e.g. "could not open port")
      • Exception           — any other transient init failure

    Behaviour:
      • First failure logs a single INFO line so we know we're waiting
      • All subsequent failures are swallowed silently
      • Polls every TRANSPORT_RETRY_INTERVAL_S (250 ms)
      • Runs indefinitely until success — the device WILL appear
      • On success logs once and returns
    """
    announced = False

    while True:
        try:
            transport_init()
            if announced:
                logging.info("✅ [transport] device available — transport initialized")
            return
        except Exception:
            if not announced:
                logging.info("⏳ [transport] device not ready — polling every %.0f ms",
                             TRANSPORT_RETRY_INTERVAL_S * 1000)
                announced = True
            time.sleep(TRANSPORT_RETRY_INTERVAL_S)

# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    setup_logging()
    open_debug_log()
    open_pubsub_log()

    # Block here until the Teensy HID device is enumerable.
    # Silent indefinite retry — survives flash, reboot, late enumeration.
    _transport_init_with_retry()

    transport_register_receive_callback(TRAFFIC_DEBUG, on_receive_debug)
    transport_register_receive_callback(TRAFFIC_REQUEST_RESPONSE, on_receive_request_response)
    transport_register_receive_callback(TRAFFIC_PUBLISH_SUBSCRIBE, on_receive_publish_subscribe)

    threading.Thread(target=rpc_server, daemon=True).start()
    threading.Thread(target=pubsub_server, daemon=True).start()

    # SERVER TCP listener — accepts connections from the Meteor server
    threading.Thread(
        target=server_tcp_listener,
        daemon=True,
        name="server-tcp-listener",
    ).start()

    # SERVER command relay — accepts Pi→SERVER command requests
    threading.Thread(
        target=server_cmd_relay_server,
        daemon=True,
        name="server-cmd-relay",
    ).start()

    # REFRESH watchdog — observational only. It reports any REFRESH that
    # remains active across REFRESH_STUCK_LOG_INTERVAL_S windows.
    threading.Thread(
        target=refresh_watchdog_loop,
        daemon=True,
        name="pubsub-refresh-watchdog",
    ).start()

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