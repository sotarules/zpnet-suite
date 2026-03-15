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

        try:
            transport_send(TRAFFIC_REQUEST_RESPONSE, req)
            reply = q.get(timeout=REPLY_TIMEOUT_S)
            return reply

        except Empty:
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

    with server_conn_lock:
        server_connected = server_conn is not None

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "pending_reply_count": pending_count,
            "pending_req_ids": pending_ids[:50],  # cap for sanity
            "req_id_current": next(req_id_counter),
            "routes_topic_count": len(routes_by_topic),
            "active_threads": threading.active_count(),
            "server_connected": server_connected,
            "server_subscription_count": len(server_subscriptions),
            "server_cmd_pending": server_cmd_pending,
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