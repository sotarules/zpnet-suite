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
from queue import Queue, Empty, Full
from typing import Dict, Any, Optional, TextIO, Set, List, Tuple

from zpnet.processes.processes import server_setup, send_command, list_subsystems
from zpnet.shared.constants import (
    TRAFFIC_REQUEST_RESPONSE,
    TRAFFIC_DEBUG,
    TRAFFIC_PUBLISH_SUBSCRIBE,
)
from zpnet.shared.logger import setup_logging
from zpnet.shared.socket_io import (
    recv_json_until_eof,
    recv_until_eof,
    send_bytes_and_shutdown,
)
import zpnet.shared.transport as transport_module
from zpnet.shared.transport import (
    transport_send,
    transport_register_receive_callback,
    transport_init,
    transport_wait_ready,
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
REPLY_TIMEOUT_S = 30.0

DEBUG_LOG_PATH = "/home/mule/zpnet/logs/zpnet-debug.log"
SOCKET_DIR = "/tmp"

# Transport retry configuration
TRANSPORT_RETRY_INTERVAL_S = 0.25   # poll every 250 ms — aggressive but not a spinlock

# Raw byte-for-byte transport logging is normally disabled.  Set the
# ZPNET_TRANSPORT_RAW_LOG environment variable to 1 to enable it temporarily.
TRANSPORT_RAW_LOG_ENABLED = os.environ.get("ZPNET_TRANSPORT_RAW_LOG") == "1"

# Formal Pi delivery is custody queued per target.  One unavailable subscriber
# must neither drop its publication nor block unrelated targets.  Each target's
# worker retries the FIFO head until Unix-socket delivery succeeds; the router
# itself only performs an unbounded in-memory enqueue.
PI_FANOUT_SOCKET_TIMEOUT_S = 2.0
PI_FANOUT_SLOW_WARN_S = 0.100
PI_DELIVERY_RETRY_INTERVAL_S = 0.25
PI_DELIVERY_RETRY_LOG_INTERVAL_S = 30.0

# Subsystems that PUBSUB should never query during ALLSUBSCRIPTIONS.
# These are test/utility programs that may leave stale sockets behind.
SUBSYSTEM_SKIP = {"TIMEBASE_WATCH"}

# SERVER TCP configuration
SERVER_TCP_HOST = "127.0.0.1"     # localhost only — SERVER arrives via reverse tunnel
SERVER_TCP_PORT = 9800            # configurable; must match SERVER-side client
SERVER_TCP_BACKLOG = 1            # one connection at a time

# SERVER command relay socket (Pi → SERVER via pubsub TCP bridge)
# Symmetric with RPC_SOCKET_PATH (Pi → TEENSY via pubsub HID/serial)
SERVER_CMD_SOCKET_PATH = "/tmp/zpnet_server_cmd.sock"
SERVER_CMD_BACKLOG = 4
SERVER_CMD_TIMEOUT_S = 30.0       # max wait for SERVER to respond

# Ad-hoc diagnostic tap.
#
# This is an observer-only local Unix stream socket for temporary tools such
# as metrics panels, terminal listeners, and debugging probes.  Clients declare
# ephemeral topic interest over a persistent newline-delimited JSON connection.
# They are never written into the formal subscription union and are never
# forwarded to TEENSY.  Delivery is best-effort with a tiny bounded queue so a
# slow ad-hoc client cannot apply backpressure to the core bus.
ADHOC_TAP_SOCKET_PATH = "/tmp/zpnet_pubsub_tap.sock"
ADHOC_TAP_BACKLOG = 8
ADHOC_QUEUE_SIZE = 2
ADHOC_SEND_TIMEOUT_S = 2.0
ADHOC_RECV_TIMEOUT_S = 1.0

# Teensy route-table custody monitor.
#
# If Teensy reboots, its in-firmware PUBSUB route table is empty even though
# the Pi-side union remains valid.  Reapply the cached union when that happens.
TEENSY_ROUTE_MONITOR_INTERVAL_S = 30.0

# Teensy can publish valid data immediately when the serial transport attaches,
# before PUBSUB has committed its first canonical route table.  Retain that
# complete startup publication prefix until the first successful REFRESH, then
# replay it through the ordinary router in original arrival order.  This is a
# bus custody boundary, not a CLOCKS-specific exception.

# ---------------------------------------------------------------------
# Global state
# ---------------------------------------------------------------------

# Request IDs are a wire-level uint32 contract with Teensy.  Keep allocation
# explicit and observable: diagnostics must never advance the sequence, and an
# exhausted uint32 space is a hard protocol failure rather than silent wrap.
pending_replies: Dict[int, Dict[str, Any]] = {}
recent_replies: Dict[int, Dict[str, Any]] = {}
recent_reply_order = Queue()
RECENT_REPLY_HISTORY_MAX = 64

req_id_counter = itertools.count(1)
req_id_last_issued = 0
req_id_lock = threading.Lock()


def _next_req_id() -> int:
    global req_id_last_issued

    with req_id_lock:
        req_id = next(req_id_counter)
        if req_id <= 0 or req_id > 0xFFFFFFFF:
            raise RuntimeError(
                f"PUBSUB request-id space exhausted/corrupt: req_id={req_id}"
            )
        req_id_last_issued = req_id
        return req_id


def _remember_reply_state(req_id: int, entry: Dict[str, Any], outcome: str) -> None:
    snapshot = dict(entry)
    snapshot.pop("queue", None)
    snapshot["outcome"] = outcome
    snapshot["retired_monotonic"] = time.monotonic()

    with state_lock:
        recent_replies[req_id] = snapshot
        recent_reply_order.put(req_id)

        while recent_reply_order.qsize() > RECENT_REPLY_HISTORY_MAX:
            old_req_id = recent_reply_order.get_nowait()
            if old_req_id != req_id:
                recent_replies.pop(old_req_id, None)


def _register_pending_reply(
    *,
    req_id: int,
    queue: Queue,
    subsystem: str,
    command: str,
    attempt: int,
    source: str,
    req_ts_ms: int,
) -> Dict[str, Any]:
    entry = {
        "queue": queue,
        "subsystem": subsystem,
        "command": command,
        "attempt": attempt,
        "source": source,
        "req_ts_ms": req_ts_ms,
        "sent_monotonic": time.monotonic(),
    }
    with state_lock:
        pending_replies[req_id] = entry
    return entry


def _retire_pending_reply(req_id: int, outcome: str) -> Optional[Dict[str, Any]]:
    """Retire one request that will never receive another lawful response."""
    with state_lock:
        entry = pending_replies.pop(req_id, None)
    if entry is not None:
        _remember_reply_state(req_id, entry, outcome)
    return entry

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

# Startup route-convergence custody for every Teensy-origin publication.  The
# lock serializes final backlog release against the transport RX callback, so a
# live frame cannot overtake any retained predecessor.  The backlog is
# intentionally unbounded: evidence is never evicted merely because control
# plane convergence is delayed.
startup_custody_lock = threading.Lock()
startup_custody_active = True
startup_custody_backlog: List[Dict[str, Any]] = []
startup_custody_retained = 0
startup_custody_released = 0
startup_custody_release_failures = 0
startup_custody_unrouted_released = 0
startup_custody_first_topic: Optional[str] = None
startup_custody_first_sequence: Optional[int] = None
startup_custody_last_topic: Optional[str] = None
startup_custody_last_sequence: Optional[int] = None


class _PiDeliveryTarget:
    """One formal Pi subscriber's ordered, lossless in-memory custody queue."""

    def __init__(self, subsystem: str) -> None:
        self.subsystem = subsystem
        self.queue: Queue = Queue()  # intentionally unbounded; formal evidence is never evicted
        self.worker_started = False
        self.enqueued = 0
        self.delivered = 0
        self.retry_count = 0
        self.max_backlog = 0
        self.current_topic: Optional[str] = None
        self.current_sequence: Optional[int] = None
        self.current_enqueued_monotonic: Optional[float] = None
        self.blocked_since_monotonic: Optional[float] = None
        self.last_enqueued_topic: Optional[str] = None
        self.last_enqueued_sequence: Optional[int] = None
        self.last_delivered_topic: Optional[str] = None
        self.last_delivered_sequence: Optional[int] = None
        self.last_delivery_monotonic: Optional[float] = None


# Per-target isolation is essential: PI:PHOTONS being down must never stall
# PI:CLOCKS, PI:GNSS, or another formal subscriber.  The lock covers target
# creation, enqueue order, and diagnostics counters only; socket I/O occurs in
# the independent worker for each target.
pi_delivery_lock = threading.Lock()
pi_delivery_targets: Dict[str, _PiDeliveryTarget] = {}

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

# ---------------------------------------------------------------------
# Teensy route-table custody monitor state
# ---------------------------------------------------------------------

teensy_route_monitor_probe_count = 0
teensy_route_monitor_probe_fail_count = 0
teensy_route_monitor_report_ok_count = 0
teensy_route_monitor_empty_count = 0
teensy_route_monitor_reapply_count = 0
teensy_route_monitor_reapply_fail_count = 0
teensy_route_monitor_last_route_count: Optional[int] = None
teensy_route_monitor_last_probe_ts: Optional[float] = None
teensy_route_monitor_last_empty_ts: Optional[float] = None
teensy_route_monitor_last_reapply_ts: Optional[float] = None
teensy_route_monitor_last_reapply_topic_count = 0
teensy_route_monitor_last_reapply_subscription_count = 0
teensy_route_monitor_last_status = "INIT"

# ---------------------------------------------------------------------
# Ad-hoc diagnostic tap state
# ---------------------------------------------------------------------

class _AdhocClient:
    def __init__(self, client_id: int, conn: socket.socket) -> None:
        self.client_id = client_id
        self.conn = conn
        self.topics: Set[str] = set()
        self.queue = Queue(maxsize=ADHOC_QUEUE_SIZE)
        self.connected_at = time.monotonic()
        self.sent_count = 0
        self.drop_count = 0
        self.closed = False


adhoc_client_id_counter = itertools.count(1)
adhoc_clients: Dict[int, _AdhocClient] = {}
adhoc_by_topic: Dict[str, Set[int]] = {}
adhoc_connect_count = 0
adhoc_disconnect_count = 0
adhoc_publish_enqueue_count = 0
adhoc_publish_drop_count = 0

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
        logging.error(
            "❌ [rpc] Teensy response missing req_id — protocol identity lost; "
            "req_ts_ms=%r success=%r message=%r",
            payload.get("req_ts_ms"),
            payload.get("success"),
            payload.get("message"),
        )
        return

    with state_lock:
        entry = pending_replies.pop(req_id, None)
        retired = recent_replies.get(req_id)

    if entry is None:
        if retired is None:
            logging.error(
                "❌ [rpc] response for UNKNOWN req_id=%r — discarded; "
                "req_ts_ms=%r success=%r message=%r",
                req_id,
                payload.get("req_ts_ms"),
                payload.get("success"),
                payload.get("message"),
            )
        else:
            age_ms = int(
                (time.monotonic() - retired["sent_monotonic"]) * 1000
            )
            logging.warning(
                "⚠️ [rpc] response for RETIRED req_id=%r classification=%s "
                "source=%s target=%s.%s attempt=%d age_ms=%d "
                "req_ts_ms=%r success=%r message=%r — discarded",
                req_id,
                retired.get("outcome"),
                retired.get("source"),
                retired.get("subsystem"),
                retired.get("command"),
                retired.get("attempt"),
                age_ms,
                payload.get("req_ts_ms"),
                payload.get("success"),
                payload.get("message"),
            )
        return

    sent_ms = payload.get("req_ts_ms")
    if sent_ms is None:
        _remember_reply_state(req_id, entry, "RESPONSE_MISSING_REQ_TS")
        logging.error(
            "❌ [rpc] response missing req_ts_ms req_id=%r source=%s "
            "target=%s.%s attempt=%d — discarded",
            req_id,
            entry["source"],
            entry["subsystem"],
            entry["command"],
            entry["attempt"],
        )
        return

    latency = int(time.monotonic() * 1000) - sent_ms
    payload["latency"] = latency
    _remember_reply_state(req_id, entry, "DELIVERED")
    entry["queue"].put(payload)

def _publication_sequence(msg: Dict[str, Any]) -> Optional[int]:
    payload = msg.get("payload")
    if not isinstance(payload, dict):
        return None
    value = payload.get("sequence")
    try:
        return int(value) if value is not None else None
    except (TypeError, ValueError):
        return None


def _retain_startup_publication(msg: Dict[str, Any]) -> bool:
    """Retain every Teensy publication until PUBSUB commits its first routes."""
    global startup_custody_retained
    global startup_custody_first_topic, startup_custody_first_sequence
    global startup_custody_last_topic, startup_custody_last_sequence

    with startup_custody_lock:
        if not startup_custody_active:
            return False

        topic = str(msg.get("topic") or "")
        sequence = _publication_sequence(msg)
        startup_custody_backlog.append(msg)
        startup_custody_retained += 1
        if startup_custody_first_topic is None:
            startup_custody_first_topic = topic
            startup_custody_first_sequence = sequence
        startup_custody_last_topic = topic
        startup_custody_last_sequence = sequence
        return True


def on_receive_publish_subscribe(payload: Dict[str, Any]) -> None:
    # Message originated from Teensy. Never forward back to Teensy.
    if _retain_startup_publication(payload):
        return
    route_publish(payload, forward_to_teensy=False)

# ---------------------------------------------------------------------
# RPC handler (Pi → Teensy)
# ---------------------------------------------------------------------

def handle_client(conn: socket.socket) -> None:
    try:
        try:
            req = recv_json_until_eof(conn)
        except RuntimeError:
            return

        req_id = None

        for attempt in range(MAX_TEENSY_RETRIES):
            req_id = _next_req_id()
            req_ts_ms = int(time.monotonic() * 1000)
            req["req_id"] = req_id
            req["req_ts_ms"] = req_ts_ms

            q = Queue(maxsize=1)
            _register_pending_reply(
                req_id=req_id,
                queue=q,
                subsystem=str(req.get("subsystem") or ""),
                command=str(req.get("command") or ""),
                attempt=attempt + 1,
                source="PI_RPC",
                req_ts_ms=req_ts_ms,
            )

            try:
                transport_send(TRAFFIC_REQUEST_RESPONSE, req)

                reply = q.get(timeout=REPLY_TIMEOUT_S)
                conn.sendall(
                    json.dumps(reply, separators=(",", ":")).encode()
                )
                return

            except RuntimeError as exc:
                if str(exc) != "SERIAL transport is not ready":
                    raise
                _retire_pending_reply(req_id, "SEND_TRANSPORT_NOT_READY")
                if transport_wait_ready(timeout_s=REPLY_TIMEOUT_S):
                    continue
                conn.sendall(
                    json.dumps({
                        "req_id": req_id,
                        "success": False,
                        "message": "TEENSY transport did not become ready",
                    }, separators=(",", ":")).encode()
                )
                return

            except Empty:
                timed_out = _retire_pending_reply(req_id, "TIMEOUT")
                if timed_out is not None:
                    logging.warning(
                        "⏱️ [rpc] Teensy timeout req_id=%d source=%s "
                        "target=%s.%s attempt=%d timeout_s=%.1f",
                        req_id,
                        timed_out["source"],
                        timed_out["subsystem"],
                        timed_out["command"],
                        timed_out["attempt"],
                        REPLY_TIMEOUT_S,
                    )
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
# Ad-hoc diagnostic tap
# ---------------------------------------------------------------------

def _adhoc_parse_topics(msg: Dict[str, Any]) -> List[str]:
    topics = msg.get("topics")
    if topics is None and msg.get("topic") is not None:
        topics = [msg.get("topic")]
    if isinstance(topics, str):
        topics = [topics]
    if not isinstance(topics, list):
        return []

    out: List[str] = []
    for topic in topics:
        name = str(topic).strip()
        if name:
            out.append(name)
    return out


def _adhoc_queue_wire(client: _AdhocClient, wire: Optional[bytes]) -> None:
    if client.closed:
        return

    try:
        client.queue.put_nowait(wire)
        return
    except Full:
        pass

    # Keep newest telemetry, never build an unbounded queue.  This applies to
    # ad-hoc observers only; formal route delivery is unchanged.
    try:
        client.queue.get_nowait()
    except Empty:
        pass

    client.drop_count += 1
    try:
        client.queue.put_nowait(wire)
    except Full:
        client.drop_count += 1


def _adhoc_queue_json(client: _AdhocClient, msg: Dict[str, Any]) -> None:
    wire = (json.dumps(msg, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")
    _adhoc_queue_wire(client, wire)


def _adhoc_register(conn: socket.socket) -> _AdhocClient:
    global adhoc_connect_count

    client = _AdhocClient(next(adhoc_client_id_counter), conn)
    with state_lock:
        adhoc_clients[client.client_id] = client
        adhoc_connect_count += 1

    logging.info("🔎 [pubsub tap] client connected id=%d", client.client_id)
    return client


def _adhoc_wake_writer(client: _AdhocClient) -> None:
    try:
        client.queue.put_nowait(None)
        return
    except Full:
        pass

    try:
        client.queue.get_nowait()
    except Empty:
        pass

    try:
        client.queue.put_nowait(None)
    except Full:
        pass


def _adhoc_unregister(client_id: int, reason: str) -> None:
    global adhoc_disconnect_count

    with state_lock:
        client = adhoc_clients.pop(client_id, None)
        if client is None:
            return

        client.closed = True
        for topic in list(client.topics):
            ids = adhoc_by_topic.get(topic)
            if ids is not None:
                ids.discard(client_id)
                if not ids:
                    adhoc_by_topic.pop(topic, None)
        client.topics.clear()
        adhoc_disconnect_count += 1

    try:
        client.conn.shutdown(socket.SHUT_RDWR)
    except Exception:
        pass
    try:
        client.conn.close()
    except Exception:
        pass

    _adhoc_wake_writer(client)
    logging.info("🔎 [pubsub tap] client disconnected id=%d (%s)", client_id, reason)


def _adhoc_set_topics(client: _AdhocClient, topics: List[str], *, replace: bool) -> List[str]:
    clean = set(topics)

    with state_lock:
        if replace:
            new_topics = clean
        else:
            new_topics = set(client.topics) | clean

        for topic in list(client.topics):
            if topic not in new_topics:
                ids = adhoc_by_topic.get(topic)
                if ids is not None:
                    ids.discard(client.client_id)
                    if not ids:
                        adhoc_by_topic.pop(topic, None)

        for topic in new_topics:
            adhoc_by_topic.setdefault(topic, set()).add(client.client_id)

        client.topics = new_topics
        return sorted(client.topics)


def _adhoc_remove_topics(client: _AdhocClient, topics: List[str]) -> List[str]:
    remove = set(topics)

    with state_lock:
        for topic in remove:
            ids = adhoc_by_topic.get(topic)
            if ids is not None:
                ids.discard(client.client_id)
                if not ids:
                    adhoc_by_topic.pop(topic, None)
            client.topics.discard(topic)
        return sorted(client.topics)


def _adhoc_handle_client_msg(client: _AdhocClient, msg: Dict[str, Any]) -> None:
    msg_type = msg.get("type")

    if msg_type == "set_topics":
        topics = _adhoc_set_topics(client, _adhoc_parse_topics(msg), replace=True)
        _adhoc_queue_json(client, {
            "type": "subscribed",
            "client_id": client.client_id,
            "topics": topics,
        })
        return

    if msg_type == "subscribe":
        topics = _adhoc_set_topics(client, _adhoc_parse_topics(msg), replace=False)
        _adhoc_queue_json(client, {
            "type": "subscribed",
            "client_id": client.client_id,
            "topics": topics,
        })
        return

    if msg_type == "unsubscribe":
        topics = _adhoc_remove_topics(client, _adhoc_parse_topics(msg))
        _adhoc_queue_json(client, {
            "type": "subscribed",
            "client_id": client.client_id,
            "topics": topics,
        })
        return

    if msg_type == "ping":
        _adhoc_queue_json(client, {
            "type": "pong",
            "client_id": client.client_id,
            "topics": sorted(client.topics),
            "sent_count": client.sent_count,
            "drop_count": client.drop_count,
        })
        return

    _adhoc_queue_json(client, {
        "type": "error",
        "client_id": client.client_id,
        "message": f"unknown tap message type: {msg_type!r}",
    })


def _adhoc_writer(client: _AdhocClient) -> None:
    try:
        client.conn.settimeout(ADHOC_SEND_TIMEOUT_S)
        while True:
            wire = client.queue.get()
            if wire is None:
                return
            client.conn.sendall(wire)
            client.sent_count += 1
    except Exception:
        logging.info("🔎 [pubsub tap] writer exiting id=%d", client.client_id)
    finally:
        _adhoc_unregister(client.client_id, "writer exited")


def _adhoc_client_session(conn: socket.socket) -> None:
    client = _adhoc_register(conn)

    threading.Thread(
        target=_adhoc_writer,
        args=(client,),
        daemon=True,
        name=f"pubsub-tap-writer-{client.client_id}",
    ).start()

    _adhoc_queue_json(client, {
        "type": "hello",
        "client_id": client.client_id,
        "protocol": "ZPNET_PUBSUB_TAP_V1",
        "commands": ["set_topics", "subscribe", "unsubscribe", "ping"],
    })

    buf = b""
    try:
        conn.settimeout(ADHOC_RECV_TIMEOUT_S)
        while True:
            try:
                chunk = conn.recv(65536)
            except socket.timeout:
                continue

            if not chunk:
                return

            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue
                try:
                    msg = json.loads(line.decode("utf-8"))
                except Exception:
                    _adhoc_queue_json(client, {
                        "type": "error",
                        "client_id": client.client_id,
                        "message": "bad JSON",
                    })
                    continue
                _adhoc_handle_client_msg(client, msg)
    except Exception:
        logging.info("🔎 [pubsub tap] reader exiting id=%d", client.client_id)
    finally:
        _adhoc_unregister(client.client_id, "reader exited")


def _route_publish_to_adhoc(topic: str, msg: Dict[str, Any]) -> None:
    global adhoc_publish_enqueue_count, adhoc_publish_drop_count

    with state_lock:
        ids = set(adhoc_by_topic.get(topic, set()))
        # A wildcard tap is useful for bench debugging, but remains explicitly
        # opt-in.  Slow wildcard clients still cannot backpressure the bus.
        ids.update(adhoc_by_topic.get("*", set()))
        clients = [adhoc_clients.get(client_id) for client_id in ids]

    if not clients:
        return

    wire_msg = {
        "type": "publish",
        "topic": topic,
        "payload": msg.get("payload"),
    }
    wire = (json.dumps(wire_msg, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")

    for client in clients:
        if client is None:
            continue
        before = client.drop_count
        _adhoc_queue_wire(client, wire)
        adhoc_publish_enqueue_count += 1
        if client.drop_count != before:
            adhoc_publish_drop_count += (client.drop_count - before)


def adhoc_tap_server() -> None:
    if os.path.exists(ADHOC_TAP_SOCKET_PATH):
        os.unlink(ADHOC_TAP_SOCKET_PATH)

    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(ADHOC_TAP_SOCKET_PATH)
    srv.listen(ADHOC_TAP_BACKLOG)

    logging.info("🔎 [pubsub tap] listener on %s", ADHOC_TAP_SOCKET_PATH)

    while True:
        conn, _ = srv.accept()
        threading.Thread(
            target=_adhoc_client_session,
            args=(conn,),
            daemon=True,
            name="pubsub-tap-client",
        ).start()


# ---------------------------------------------------------------------
# Formal Pi fan-out instrumentation
# ---------------------------------------------------------------------

def _fanout_to_pi(
    topic: str,
    subsystem: str,
    raw: bytes,
    *,
    log_failure: bool = True,
) -> bool:
    """Perform one bounded Unix-socket delivery attempt for a formal Pi target."""
    sock_path = f"{SOCKET_DIR}/zpnet-{subsystem.lower()}-pubsub.sock"
    started = time.monotonic()
    connected: Optional[float] = None

    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.settimeout(PI_FANOUT_SOCKET_TIMEOUT_S)
            sock.connect(sock_path)
            connected = time.monotonic()
            send_bytes_and_shutdown(sock, raw)

        completed = time.monotonic()
        connect_s = connected - started
        send_s = completed - connected
        total_s = completed - started

        if total_s >= PI_FANOUT_SLOW_WARN_S:
            logging.warning(
                "🐌 [pubsub] slow Pi fan-out topic=%s target=PI:%s bytes=%d "
                "connect_ms=%.3f send_ms=%.3f total_ms=%.3f timeout_s=%.3f",
                topic,
                subsystem,
                len(raw),
                connect_s * 1000.0,
                send_s * 1000.0,
                total_s * 1000.0,
                PI_FANOUT_SOCKET_TIMEOUT_S,
            )
        return True

    except Exception as exc:
        failed = time.monotonic()
        connect_ms = (
            None
            if connected is None
            else (connected - started) * 1000.0
        )
        send_ms = (
            None
            if connected is None
            else (failed - connected) * 1000.0
        )

        if log_failure:
            logging.exception(
                "💥 [pubsub] Pi fan-out failed topic=%s target=PI:%s "
                "socket=%s bytes=%d phase=%s connect_ms=%s send_ms=%s "
                "total_ms=%.3f timeout_s=%.3f error=%r",
                topic,
                subsystem,
                sock_path,
                len(raw),
                "CONNECT" if connected is None else "SEND",
                "unavailable" if connect_ms is None else f"{connect_ms:.3f}",
                "unavailable" if send_ms is None else f"{send_ms:.3f}",
                (failed - started) * 1000.0,
                PI_FANOUT_SOCKET_TIMEOUT_S,
                exc,
            )
        return False


def _pi_delivery_worker(target: _PiDeliveryTarget) -> None:
    """Deliver one target's FIFO head until success before advancing custody."""
    while True:
        item = target.queue.get()
        topic = str(item["topic"])
        raw = item["raw"]
        sequence = item.get("sequence")
        enqueued_monotonic = float(item["enqueued_monotonic"])

        with pi_delivery_lock:
            target.current_topic = topic
            target.current_sequence = sequence
            target.current_enqueued_monotonic = enqueued_monotonic

        attempts = 0
        last_failure_log = 0.0
        while True:
            now = time.monotonic()
            log_failure = attempts == 0 or (
                now - last_failure_log >= PI_DELIVERY_RETRY_LOG_INTERVAL_S
            )
            if _fanout_to_pi(
                topic, target.subsystem, raw, log_failure=log_failure
            ):
                delivered_at = time.monotonic()
                with pi_delivery_lock:
                    was_blocked = target.blocked_since_monotonic is not None
                    blocked_since = target.blocked_since_monotonic
                    target.delivered += 1
                    target.last_delivered_topic = topic
                    target.last_delivered_sequence = sequence
                    target.last_delivery_monotonic = delivered_at
                    target.current_topic = None
                    target.current_sequence = None
                    target.current_enqueued_monotonic = None
                    target.blocked_since_monotonic = None
                target.queue.task_done()
                if was_blocked:
                    blocked_ms = (
                        (delivered_at - blocked_since) * 1000.0
                        if blocked_since is not None
                        else 0.0
                    )
                    logging.info(
                        "✅ [pubsub] Pi delivery custody resumed target=PI:%s "
                        "topic=%s sequence=%s retries=%d blocked_ms=%.1f backlog=%d",
                        target.subsystem,
                        topic,
                        sequence,
                        attempts,
                        blocked_ms,
                        target.queue.qsize(),
                    )
                break

            attempts += 1
            with pi_delivery_lock:
                target.retry_count += 1
                if target.blocked_since_monotonic is None:
                    target.blocked_since_monotonic = time.monotonic()
            if log_failure:
                last_failure_log = now
                logging.warning(
                    "📥 [pubsub] retaining undelivered formal Pi publication "
                    "target=PI:%s topic=%s sequence=%s retries=%d backlog=%d",
                    target.subsystem,
                    topic,
                    sequence,
                    attempts,
                    target.queue.qsize() + 1,
                )
            time.sleep(PI_DELIVERY_RETRY_INTERVAL_S)


def _enqueue_pi_delivery(
    msg: Dict[str, Any], topic: str, subsystem: str, raw: bytes
) -> None:
    """Transfer one routed publication into a formal Pi target's FIFO custody."""
    start_worker = False
    sequence = _publication_sequence(msg)
    item = {
        "topic": topic,
        "raw": raw,
        "sequence": sequence,
        "enqueued_monotonic": time.monotonic(),
    }

    with pi_delivery_lock:
        target = pi_delivery_targets.get(subsystem)
        if target is None:
            target = _PiDeliveryTarget(subsystem)
            pi_delivery_targets[subsystem] = target
        target.queue.put_nowait(item)
        target.enqueued += 1
        target.last_enqueued_topic = topic
        target.last_enqueued_sequence = sequence
        backlog = target.queue.qsize()
        if backlog > target.max_backlog:
            target.max_backlog = backlog
        if not target.worker_started:
            target.worker_started = True
            start_worker = True

    if start_worker:
        threading.Thread(
            target=_pi_delivery_worker,
            args=(target,),
            daemon=True,
            name=f"pubsub-pi-delivery-{subsystem.lower()}",
        ).start()


def _release_startup_custody_after_route_commit() -> bool:
    """Replay the retained Teensy startup prefix through committed routes."""
    global startup_custody_active, startup_custody_released
    global startup_custody_release_failures, startup_custody_unrouted_released

    # Hold the custody lock through the complete transfer.  The transport RX
    # callback blocks here, so newly arriving live publications cannot overtake
    # the retained prefix.  route_publish() hands formal Pi deliveries to their
    # per-target FIFO queues; a dead subscriber therefore cannot stall this
    # global release or any unrelated target.
    with startup_custody_lock:
        if not startup_custody_active:
            return True

        released_now = 0
        unrouted_now = 0
        while startup_custody_backlog:
            msg = startup_custody_backlog[0]
            topic = str(msg.get("topic") or "")
            sequence = _publication_sequence(msg)
            with state_lock:
                targets = set(routes_by_topic.get(topic, set()))
            deliverable_targets = {
                (machine, subsystem)
                for machine, subsystem in targets
                if machine != "TEENSY"
            }
            if not deliverable_targets:
                unrouted_now += 1

            try:
                # Message originated from Teensy, so it must never be reflected
                # back to Teensy during replay.
                route_publish(msg, forward_to_teensy=False)
            except Exception:
                startup_custody_release_failures += 1
                logging.exception(
                    "💥 [pubsub] startup publication custody release blocked: "
                    "retained=%d released_now=%d next_topic=%s next_sequence=%s",
                    len(startup_custody_backlog),
                    released_now,
                    topic,
                    sequence,
                )
                return False

            startup_custody_backlog.pop(0)
            startup_custody_released += 1
            startup_custody_unrouted_released += 1 if not deliverable_targets else 0
            released_now += 1

        startup_custody_active = False
        logging.info(
            "✅ [pubsub] startup publication custody released in order: "
            "count=%d unrouted=%d first=%s/%s last=%s/%s",
            released_now,
            unrouted_now,
            startup_custody_first_topic,
            startup_custody_first_sequence,
            startup_custody_last_topic,
            startup_custody_last_sequence,
        )
        return True


# ---------------------------------------------------------------------
# Pub/Sub routing core
# ---------------------------------------------------------------------

def route_publish(msg: Dict[str, Any], *, forward_to_teensy: bool) -> None:
    """
    Route a single published message to all interested targets.

    Semantics:
      • Transfer PI-target publications into per-subsystem FIFO custody queues
      • Forward to TEENSY at most once (transport layer) if:
          - forward_to_teensy is True, and
          - at least one TEENSY target exists for the topic
      • Forward to SERVER via persistent TCP connection if:
          - at least one SERVER target exists for the topic
          - SERVER is connected
      • A PI subscriber outage retains evidence for ordered retry without blocking peers
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

    _route_publish_to_adhoc(topic, msg)

    teensy_needed = False
    server_needed = False

    for machine, subsystem in targets:
        if machine == "TEENSY":
            teensy_needed = True
            continue

        if machine == "SERVER":
            server_needed = True
            continue

        # PI target: transfer custody immediately; an independent target worker
        # retries the FIFO head until the subscriber accepts it.
        _enqueue_pi_delivery(msg, topic, subsystem, raw)

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

    # Auto-refresh routes to include SERVER subscriptions
    try:
        cmd_refresh(None)
    except Exception:
        logging.exception("⚠️ [pubsub] auto REFRESH after SERVER subscribe failed")


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
    *,
    source: str = "SERVER_RPC",
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
        req_id = _next_req_id()
        req_ts_ms = int(time.monotonic() * 1000)
        req["req_id"] = req_id
        req["req_ts_ms"] = req_ts_ms

        q = Queue(maxsize=1)
        _register_pending_reply(
            req_id=req_id,
            queue=q,
            subsystem=subsystem,
            command=command,
            attempt=attempt + 1,
            source=source,
            req_ts_ms=req_ts_ms,
        )

        try:
            transport_send(TRAFFIC_REQUEST_RESPONSE, req)
            reply = q.get(timeout=REPLY_TIMEOUT_S)
            return reply

        except RuntimeError as exc:
            if str(exc) != "SERIAL transport is not ready":
                raise
            _retire_pending_reply(req_id, "SEND_TRANSPORT_NOT_READY")
            if transport_wait_ready(timeout_s=REPLY_TIMEOUT_S):
                continue
            return {
                "success": False,
                "message": "TEENSY transport did not become ready",
                "payload": {},
            }

        except Empty:
            timed_out = _retire_pending_reply(req_id, "TIMEOUT")
            if timed_out is not None:
                logging.warning(
                    "⏱️ [rpc] Teensy timeout req_id=%d source=%s "
                    "target=%s.%s attempt=%d timeout_s=%.1f",
                    req_id,
                    timed_out["source"],
                    timed_out["subsystem"],
                    timed_out["command"],
                    timed_out["attempt"],
                    REPLY_TIMEOUT_S,
                )
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
    _route_publish_to_adhoc(topic, pub_msg)

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

        # PI target: transfer custody immediately; an independent target worker
        # retries the FIFO head until the subscriber accepts it.
        _enqueue_pi_delivery(pub_msg, topic, subsystem, raw)

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
        try:
            req = recv_json_until_eof(conn)
        except RuntimeError:
            return

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
            raw = recv_until_eof(conn)
            if not raw:
                continue

            try:
                msg = json.loads(raw.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                # One malformed or abandoned publisher must never terminate
                # the long-lived authoritative pub/sub routing thread.
                logging.exception(
                    "⚠️ [pubsub] malformed PI publication ignored "
                    "(bytes=%d preview=%r)",
                    len(raw),
                    raw[:256],
                )
                continue

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
        pending_detail = [
            {
                "req_id": req_id,
                "source": entry["source"],
                "subsystem": entry["subsystem"],
                "command": entry["command"],
                "attempt": entry["attempt"],
                "age_ms": int(
                    (time.monotonic() - entry["sent_monotonic"]) * 1000
                ),
            }
            for req_id, entry in pending_replies.items()
        ]
        recent_detail = [
            {
                "req_id": req_id,
                "outcome": entry.get("outcome"),
                "source": entry.get("source"),
                "subsystem": entry.get("subsystem"),
                "command": entry.get("command"),
                "attempt": entry.get("attempt"),
            }
            for req_id, entry in list(recent_replies.items())[-16:]
        ]
        server_cmd_pending = len(server_pending_commands)
        adhoc_client_count = len(adhoc_clients)
        adhoc_topic_count = len(adhoc_by_topic)
        applied_subscription_count = len(applied_union.get("subscriptions", []))
        applied_topic_count = len(routes_by_topic)
        adhoc_routes = [
            {
                "client_id": client.client_id,
                "topics": sorted(client.topics),
                "sent_count": client.sent_count,
                "drop_count": client.drop_count,
            }
            for client in adhoc_clients.values()
        ]
        startup_custody = {
            "scope": "ALL_TEENSY_PUBLICATIONS_UNTIL_FIRST_ROUTE_COMMIT",
            "active": startup_custody_active,
            "backlog_depth": len(startup_custody_backlog),
            "retained": startup_custody_retained,
            "released": startup_custody_released,
            "release_failures": startup_custody_release_failures,
            "unrouted_released": startup_custody_unrouted_released,
            "first_topic": startup_custody_first_topic,
            "first_sequence": startup_custody_first_sequence,
            "last_topic": startup_custody_last_topic,
            "last_sequence": startup_custody_last_sequence,
        }
        route_monitor = {
            "interval_s": TEENSY_ROUTE_MONITOR_INTERVAL_S,
            "probe_count": teensy_route_monitor_probe_count,
            "probe_fail_count": teensy_route_monitor_probe_fail_count,
            "report_ok_count": teensy_route_monitor_report_ok_count,
            "empty_count": teensy_route_monitor_empty_count,
            "reapply_count": teensy_route_monitor_reapply_count,
            "reapply_fail_count": teensy_route_monitor_reapply_fail_count,
            "last_route_count": teensy_route_monitor_last_route_count,
            "last_probe_ts": teensy_route_monitor_last_probe_ts,
            "last_empty_ts": teensy_route_monitor_last_empty_ts,
            "last_reapply_ts": teensy_route_monitor_last_reapply_ts,
            "last_reapply_topic_count": teensy_route_monitor_last_reapply_topic_count,
            "last_reapply_subscription_count": teensy_route_monitor_last_reapply_subscription_count,
            "last_status": teensy_route_monitor_last_status,
        }

    with pi_delivery_lock:
        now = time.monotonic()
        pi_delivery_custody = [
            {
                "target": f"PI:{target.subsystem}",
                "worker_started": target.worker_started,
                "backlog_depth": target.queue.qsize(),
                "in_flight": target.current_topic is not None,
                "enqueued": target.enqueued,
                "delivered": target.delivered,
                "retry_count": target.retry_count,
                "max_backlog": target.max_backlog,
                "current_topic": target.current_topic,
                "current_sequence": target.current_sequence,
                "current_age_ms": (
                    int((now - target.current_enqueued_monotonic) * 1000)
                    if target.current_enqueued_monotonic is not None
                    else None
                ),
                "blocked_ms": (
                    int((now - target.blocked_since_monotonic) * 1000)
                    if target.blocked_since_monotonic is not None
                    else 0
                ),
                "last_enqueued_topic": target.last_enqueued_topic,
                "last_enqueued_sequence": target.last_enqueued_sequence,
                "last_delivered_topic": target.last_delivered_topic,
                "last_delivered_sequence": target.last_delivered_sequence,
            }
            for target in sorted(
                pi_delivery_targets.values(), key=lambda value: value.subsystem
            )
        ]

    with server_conn_lock:
        server_connected = server_conn is not None

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "pending_reply_count": pending_count,
            "pending_req_ids": pending_ids[:50],  # cap for sanity
            "pending_reply_detail": pending_detail[:50],
            "recent_reply_detail": recent_detail,
            "req_id_last_issued": req_id_last_issued,
            "routes_topic_count": len(routes_by_topic),
            "applied_topic_count": applied_topic_count,
            "applied_subscription_count": applied_subscription_count,
            "active_threads": threading.active_count(),
            "server_connected": server_connected,
            "server_subscription_count": len(server_subscriptions),
            "server_cmd_pending": server_cmd_pending,
            "adhoc_socket_path": ADHOC_TAP_SOCKET_PATH,
            "adhoc_client_count": adhoc_client_count,
            "adhoc_topic_count": adhoc_topic_count,
            "adhoc_connect_count": adhoc_connect_count,
            "adhoc_disconnect_count": adhoc_disconnect_count,
            "adhoc_publish_enqueue_count": adhoc_publish_enqueue_count,
            "adhoc_publish_drop_count": adhoc_publish_drop_count,
            "adhoc_routes": adhoc_routes,
            "teensy_route_monitor": route_monitor,
            "startup_publication_custody": startup_custody,
            "pi_delivery_custody": pi_delivery_custody,
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
        # Skip known non-service subsystems (test/utility programs)
        if subsystem in SUBSYSTEM_SKIP:
            logging.debug(
                "ℹ️ [pubsub] skipping %s (in SUBSYSTEM_SKIP)", subsystem,
            )
            continue

        try:
            # Self-query: answer directly
            if subsystem == "PUBSUB":
                results.append({
                    "machine": "PI",
                    "subsystem": "PUBSUB",
                    "subscriptions": [],
                })
                continue

            # Skip subsystems whose command socket doesn't exist
            sock_path = f"{SOCKET_DIR}/zpnet-{subsystem.lower()}-command.sock"
            if not os.path.exists(sock_path):
                logging.debug(
                    "ℹ️ [pubsub] skipping %s (no socket at %s)",
                    subsystem, sock_path,
                )
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
            # Stale socket or subsystem not yet ready — skip quietly
            logging.debug(
                "ℹ️ [pubsub] skipping %s (SUBSCRIPTIONS query failed)",
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
    Return the union of PI-side, TEENSY-side, and SERVER-side
    declared subscriptions.

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

    # 3) SERVER-side (from cached subscription declaration)
    with state_lock:
        if server_subscriptions:
            results.extend(list(server_subscriptions))

    # 4) Canonical ordering
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
      • Collect PI + TEENSY + SERVER declared subscriptions
      • Union them into a single canonical payload
      • Commit that payload verbatim to:
          1) TEENSY (SETSUBSCRIPTIONS)
          2) PI (local routing state)
      • SERVER subscriptions are included in the routing table
        but are NOT forwarded to TEENSY (TEENSY does not need to
        know about SERVER; pubsub handles SERVER fan-out)
    """

    # 1) Observe + union (single source of truth)
    union_resp = cmd_unionsubscriptions(None)
    union_payload = union_resp.get("payload", {})

    # 2) Commit union verbatim to TEENSY
    #    NOTE: TEENSY receives the full union including SERVER entries.
    #    This is harmless — TEENSY ignores subscriptions it doesn't
    #    originate, and the union is observational truth.
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

    _release_startup_custody_after_route_commit()

    # 5) Return committed truth
    return {
        "success": True,
        "message": "OK",
        "payload": union_payload,
    }


# ---------------------------------------------------------------------
# Teensy route-table custody monitor
# ---------------------------------------------------------------------

def _copy_applied_union_for_teensy_route_reapply() -> Dict[str, Any]:
    """
    Return a stable copy of the last locally committed subscription union.

    Do not rediscover subscriptions here.  If Teensy rebooted, the Pi-side
    process sockets and the last committed union remain the authority we want
    to foist back onto the fresh Teensy runtime.
    """
    with state_lock:
        return json.loads(json.dumps(applied_union))


def _teensy_route_report() -> Optional[Dict[str, Any]]:
    """
    Ask Teensy PUBSUB for its current route table.

    Returns the response payload on success, or None when Teensy is not
    reachable / not ready.  Failures are intentionally quiet because this
    thread is a background custody probe.
    """
    resp = _server_command_to_teensy(
        "PUBSUB", "REPORT", None, source="ROUTE_MONITOR"
    )
    if not resp.get("success"):
        return None
    payload = resp.get("payload")
    return payload if isinstance(payload, dict) else {}


def _teensy_route_count(payload: Dict[str, Any]) -> int:
    routes = payload.get("routes")
    return len(routes) if isinstance(routes, list) else 0


def _teensy_route_reapply_cached_union(union_payload: Dict[str, Any]) -> bool:
    """
    Reapply the cached Pi-side union to Teensy with SETSUBSCRIPTIONS.
    """
    resp = _server_command_to_teensy(
        "PUBSUB", "SETSUBSCRIPTIONS", union_payload, source="ROUTE_MONITOR_REAPPLY"
    )
    return bool(resp.get("success"))


def teensy_route_monitor_loop() -> None:
    """
    Periodically verify that Teensy's PUBSUB route table still exists.

    The monitor is specifically for Teensy reboot recovery.  A reboot clears
    process_pubsub's in-firmware routing state, while the Pi-side PUBSUB process
    still has the authoritative last committed union in applied_union.  When
    Teensy reports an empty route table and the Pi has a non-empty cached union,
    foist that cached union back onto Teensy without repolling every service.
    """
    global teensy_route_monitor_probe_count
    global teensy_route_monitor_probe_fail_count
    global teensy_route_monitor_report_ok_count
    global teensy_route_monitor_empty_count
    global teensy_route_monitor_reapply_count
    global teensy_route_monitor_reapply_fail_count
    global teensy_route_monitor_last_route_count
    global teensy_route_monitor_last_probe_ts
    global teensy_route_monitor_last_empty_ts
    global teensy_route_monitor_last_reapply_ts
    global teensy_route_monitor_last_reapply_topic_count
    global teensy_route_monitor_last_reapply_subscription_count
    global teensy_route_monitor_last_status

    logging.info(
        "🧭 [pubsub] Teensy route monitor armed (interval=%.1fs)",
        TEENSY_ROUTE_MONITOR_INTERVAL_S,
    )

    while True:
        time.sleep(TEENSY_ROUTE_MONITOR_INTERVAL_S)

        teensy_route_monitor_probe_count += 1
        teensy_route_monitor_last_probe_ts = time.time()

        payload = _teensy_route_report()
        if payload is None:
            teensy_route_monitor_probe_fail_count += 1
            teensy_route_monitor_last_status = "TEENSY_UNREACHABLE"
            continue

        teensy_route_monitor_report_ok_count += 1
        route_count = _teensy_route_count(payload)
        teensy_route_monitor_last_route_count = route_count

        if route_count != 0:
            teensy_route_monitor_last_status = "NOMINAL"
            continue

        union_payload = _copy_applied_union_for_teensy_route_reapply()
        subscriptions = union_payload.get("subscriptions")
        if not isinstance(subscriptions, list) or len(subscriptions) == 0:
            teensy_route_monitor_last_status = "EMPTY_NO_CACHED_UNION"
            continue

        topic_names = set()
        for row in subscriptions:
            if not isinstance(row, dict):
                continue
            for sub in row.get("subscriptions", []):
                if isinstance(sub, dict) and sub.get("name"):
                    topic_names.add(str(sub.get("name")))

        teensy_route_monitor_empty_count += 1
        teensy_route_monitor_last_empty_ts = time.time()

        logging.info(
            "🔄 [pubsub] Teensy route table empty; reapplying cached union "
            "(%d subscriptions, %d topics)",
            len(subscriptions),
            len(topic_names),
        )

        if _teensy_route_reapply_cached_union(union_payload):
            teensy_route_monitor_reapply_count += 1
            teensy_route_monitor_last_reapply_ts = time.time()
            teensy_route_monitor_last_reapply_topic_count = len(topic_names)
            teensy_route_monitor_last_reapply_subscription_count = len(subscriptions)
            teensy_route_monitor_last_status = "REAPPLIED_CACHED_UNION"
            logging.info("✅ [pubsub] Teensy route table restored from cached union")
        else:
            teensy_route_monitor_reapply_fail_count += 1
            teensy_route_monitor_last_status = "REAPPLY_FAILED"


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
        if not transport_wait_ready(timeout_s=REPLY_TIMEOUT_S):
            logging.warning("⚠️ [pubsub] auto REFRESH skipped — Teensy transport not ready")
            return
        logging.info("🔄 [pubsub] auto REFRESH starting")
        cmd_refresh(None)
        logging.info("✅ [pubsub] auto REFRESH complete")
    except Exception:
        logging.exception("❌ [pubsub] auto REFRESH failed")

# ---------------------------------------------------------------------
# Transport init with aggressive retry
# ---------------------------------------------------------------------

def _transport_init_with_retry() -> None:
    """
    Start the serial transport supervisor, retrying if the transport
    subsystem itself is not ready yet.  Device availability after startup is
    owned by the serial transport supervisor, which survives flash/reboot and
    late udev enumeration.

    Behaviour:
      • First failure logs a single INFO line so we know we're waiting
      • All subsequent failures are swallowed silently
      • Polls every TRANSPORT_RETRY_INTERVAL_S (250 ms)
      • Runs indefinitely until transport_init succeeds
      • On success logs once and returns
    """
    announced = False

    transport_init()

    while True:
        if transport_wait_ready(timeout_s=5.0):
            if announced:
                logging.info("✅ [transport] device available — transport initialized")
            else:
                logging.info("✅ [transport] device available — transport initialized")
            return

        if not announced:
            logging.info("⏳ [transport] waiting for serial readiness every %.0f ms",
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

    transport_module.RAW_TRANSPORT_LOG_ENABLED = TRANSPORT_RAW_LOG_ENABLED
    logging.info(
        "📝 [transport] raw wire logging %s",
        "ENABLED" if TRANSPORT_RAW_LOG_ENABLED else "disabled",
    )

    # Register receive handlers before starting the serial RX supervisor.
    # The Teensy emits DEBUG/PUBSUB frames immediately during boot; the
    # transport must not observe a live serial stream before PUBSUB has
    # installed its traffic callbacks.
    transport_register_receive_callback(TRAFFIC_DEBUG, on_receive_debug)
    transport_register_receive_callback(TRAFFIC_REQUEST_RESPONSE, on_receive_request_response)
    transport_register_receive_callback(TRAFFIC_PUBLISH_SUBSCRIBE, on_receive_publish_subscribe)

    # Start the serial transport supervisor.  The supervisor handles flash,
    # reboot, late enumeration, and transient device disappearance after this.
    _transport_init_with_retry()

    threading.Thread(target=rpc_server, daemon=True).start()
    threading.Thread(target=pubsub_server, daemon=True).start()

    # Ad-hoc observer tap for metrics panels and debugging tools.
    # This is intentionally outside the formal subscription union.
    threading.Thread(
        target=adhoc_tap_server,
        daemon=True,
        name="pubsub-adhoc-tap",
    ).start()

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

    # Automatic control-plane convergence
    threading.Thread(
        target=_delayed_refresh,
        daemon=True,
        name="pubsub-auto-refresh",
    ).start()

    # Teensy route-table custody monitor.  If Teensy reboots and loses its
    # in-firmware PUBSUB routes, reapply the cached Pi-side union without
    # repolling every service.
    threading.Thread(
        target=teensy_route_monitor_loop,
        daemon=True,
        name="teensy-route-monitor",
    ).start()

    # Process lifetime (never returns)
    server_setup(
        subsystem="PUBSUB",
        commands=COMMANDS,
        subscriptions={},
    )

def bootstrap() -> None:
    run()