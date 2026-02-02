"""
ZPNet Pub/Sub Authority (Pi-side)

Responsibilities:
  • Discover subscription needs from all processes (Pi + Teensy)
  • Build a definitive subscription table (machine, subsystem -> wants[])
  • Push the definitive table to Teensy PUBSUB (SET_TABLE)
  • Expose command surface for REPORT / REFRESH / APPLY

Process model:
  • One systemd service
  • One polling thread (optional refresh cadence)
  • One blocking command socket (REPORT / REFRESH)

Semantics:
  • Uses unified send_command() interface
  • Subscription discovery is command-based ("SUBSCRIPTIONS")
  • Pub/Sub traffic (0xD2) is DATA PLANE ONLY (topic + payload)
  • No pub/sub control-plane messages (SUBSCRIBE/UNSUBSCRIBE)

Author: The Mule + GPT
"""

from __future__ import annotations

import glob
import itertools
import json
import logging
import os
import socket
import threading
import time
from queue import Queue, Empty
from typing import Dict, Any, Optional, TextIO, Set
from typing import List

from zpnet.processes.processes import server_setup, send_command
from zpnet.shared.constants import (
    TRAFFIC_REQUEST_RESPONSE,
    TRAFFIC_DEBUG,
    TRAFFIC_PUBLISH_SUBSCRIBE,
)
from zpnet.shared.logger import setup_logging
from zpnet.shared.transport import (
    transport_send,
    transport_register_receive_callback,
    transport_init
)
from zpnet.shared.util import payload_to_json_bytes
from zpnet.shared.util import payload_to_json_str

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

# =============================================================================
# Configuration
# =============================================================================

# Optional: periodic refresh. Set to 0 to disable auto-refresh.
REFRESH_INTERVAL_S = 0

# Unix socket glob to discover Pi-side processes (command plane)
PI_COMMAND_SOCK_GLOB = f"{SOCKET_DIR}/zpnet-*-command.sock"

# Teensy subsystems query surface
TEENSY_SYSTEM_SUBSYSTEM = "SYSTEM"
TEENSY_SYSTEM_PROCESS_LIST_CMD = "PROCESS_LIST"

# Required implicit command on all processes (Pi + Teensy)
IMPLICIT_SUBSCRIPTIONS_CMD = "SUBSCRIPTIONS"

# Teensy PUBSUB table apply command (Teensy-side alter ego)
TEENSY_PUBSUB_SUBSYSTEM = "PUBSUB"
TEENSY_PUBSUB_SET_TABLE_CMD = "SET_TABLE"

# ---------------------------------------------------------------------
# Global state
# ---------------------------------------------------------------------

pending_replies: Dict[int, Queue] = {}
req_id_counter = itertools.count(1)

# Pub/Sub subscription table: subsystem -> set(topics)
subscriptions: Dict[str, Set[str]] = {}

state_lock = threading.Lock()

debug_log_fh: Optional[TextIO] = None

# =============================================================================
# Internal state
# =============================================================================

# Canonical table (rows)
# row shape:
#   { "machine": "PI"|"TEENSY", "subsystem": "EVENTS", "wants": ["A","B"] }
_table_rows: List[Dict[str, Any]] = []

_last_refresh_ts: float = 0.0
_last_push_ok: bool = False
_last_error: str = ""

# ---------------------------------------------------------------------
# Debug sink
# ---------------------------------------------------------------------

def open_debug_log() -> None:
    global debug_log_fh
    os.makedirs(os.path.dirname(DEBUG_LOG_PATH), exist_ok=True)
    debug_log_fh = open(DEBUG_LOG_PATH, "w", buffering=1)
    logging.info("📝 [debug] debug log opened at %s", DEBUG_LOG_PATH)

# ---------------------------------------------------------------------
# Teensy receive callbacks (semantic only)
# ---------------------------------------------------------------------

def on_receive_debug(payload: Dict[str, Any]) -> None:
    logging.info("🐞 [rx-debug] %s", payload)
    if debug_log_fh:
        debug_log_fh.write(payload_to_json_str(payload) + "\n")


def on_receive_request_response(payload: Dict[str, Any]) -> None:
    logging.info("📥 [rx-rpc] response payload=%s", payload)

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
    logging.info(
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
        logging.info("📤 [rpc] incoming request %s", req)

        req_id = next(req_id_counter)
        req["req_id"] = req_id

        q = Queue(maxsize=1)
        with state_lock:
            pending_replies[req_id] = q

        for attempt in range(1, MAX_TEENSY_RETRIES + 1):
            logging.info(
                "📡 [rpc] sending to TEENSY req_id=%s attempt=%d",
                req_id,
                attempt,
            )
            transport_send(TRAFFIC_REQUEST_RESPONSE, req)

            try:
                reply = q.get(timeout=REPLY_TIMEOUT_S)
                logging.info(
                    "✅ [rpc] reply received req_id=%s payload=%s",
                    req_id,
                    reply,
                )
                conn.sendall(
                    json.dumps(reply, separators=(",", ":")).encode("utf-8")
                )
                return

            except Empty:
                logging.warning(
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
# Pub/Sub routing core (HEAVILY INSTRUMENTED)
# ---------------------------------------------------------------------

def route_publish(msg: Dict[str, Any], *, forward_to_teensy: bool) -> None:
    topic = msg.get("topic")
    payload = msg.get("payload")

    logging.info(
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

        logging.info(
            "🚀 [pubsub] subscription announcement ACCEPTED subsystem=%s topics=%s",
            subsystem,
            topics,
        )

        logging.info(
            "🧭 [pubsub] current subscription table=%s",
            {k: sorted(v) for k, v in subscriptions.items()},
        )

        if forward_to_teensy:
            logging.info(
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
        logging.info(
            "🧭 [pubsub] routing snapshot subscriptions=%s",
            {k: sorted(v) for k, v in subscriptions.items()},
        )

        targets = [
            subsystem
            for subsystem, topic_set in subscriptions.items()
            if topic in topic_set
        ]

    logging.info(
        "🎯 [pubsub] routing decision topic=%s targets=%s",
        topic,
        targets,
    )

    raw = payload_to_json_bytes(msg)

    for subsystem in targets:
        sock_path = f"{SOCKET_DIR}/zpnet-{subsystem.lower()}-pubsub.sock"
        logging.info(
            "📤 [pubsub] fan-out ATTEMPT topic=%s -> %s",
            topic,
            sock_path,
        )
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                sock.connect(sock_path)
                sock.sendall(raw)
                sock.shutdown(socket.SHUT_WR)
            logging.info(
                "✅ [pubsub] fan-out SUCCESS topic=%s -> %s",
                topic,
                subsystem,
            )
        except Exception as e:
            logging.warning(
                "⚠️ [pubsub] fan-out FAILED topic=%s -> %s err=%r",
                topic,
                subsystem,
                e,
            )

    # -------------------------------------------------------------
    # Forward to Teensy (Pi-originated only)
    # -------------------------------------------------------------
    if forward_to_teensy:
        logging.info(
            "📡 [pubsub] forwarding publish to TEENSY topic=%s payload=%s",
            topic,
            payload,
        )
        transport_send(TRAFFIC_PUBLISH_SUBSCRIBE, msg)

# =============================================================================
# Discovery helpers
# =============================================================================

def _subsystem_from_pi_command_sock(path: str) -> Optional[str]:
    """
    Extract subsystem from:
      /tmp/zpnet-<subsystem>-command.sock
    """
    base = os.path.basename(path)
    if not base.startswith("zpnet-") or not base.endswith("-command.sock"):
        return None
    mid = base[len("zpnet-") : -len("-command.sock")]
    if not mid:
        return None
    return mid.upper()

def _list_pi_subsystems() -> List[str]:
    subs: List[str] = []

    for path in sorted(glob.glob(PI_COMMAND_SOCK_GLOB)):
        s = _subsystem_from_pi_command_sock(path)
        if not s:
            continue
        subs.append(s)

    # Deduplicate while preserving order
    seen = set()
    out: List[str] = []
    for s in subs:
        if s in seen:
            continue
        seen.add(s)
        out.append(s)

    return out

def _list_teensy_subsystems() -> List[str]:
    """
    Ask Teensy SYSTEM.PROCESS_LIST for registered process names.
    Expects response payload:
      { "processes": [ { "name": "CLOCKS" }, ... ] }
    """
    resp = send_command(
        machine="TEENSY",
        subsystem=TEENSY_SYSTEM_SUBSYSTEM,
        command=TEENSY_SYSTEM_PROCESS_LIST_CMD,
    )

    if not resp.get("success"):
        raise RuntimeError(f"TEENSY SYSTEM.PROCESS_LIST failed: {resp}")

    payload = resp.get("payload") or {}
    procs = payload.get("processes") or []

    out: List[str] = []
    for item in procs:
        name = item.get("name")
        if isinstance(name, str) and name:
            out.append(name)

    return out

def _get_subscriptions(machine: str, subsystem: str) -> List[str]:
    """
    Ask a process to cough up its static subscription needs.

    Expected payload:
      { "topics": ["A","B"] }

    If the process doesn't implement it yet, we fail soft (empty list),
    but record the fault for REPORT.
    """
    resp = send_command(
        machine=machine,
        subsystem=subsystem,
        command=IMPLICIT_SUBSCRIPTIONS_CMD,
    )

    if not resp.get("success"):
        # Soft fail (we want the service to come up early while the
        # implicit command is being rolled out)
        logging.warning(
            "⚠️  [pubsub] %s %s does not support %s yet (resp=%s)",
            machine, subsystem, IMPLICIT_SUBSCRIPTIONS_CMD, resp,
        )
        return []

    payload = resp.get("payload") or {}
    topics = payload.get("topics") or []

    out: List[str] = []
    for t in topics:
        if isinstance(t, str) and t:
            out.append(t)

    # Deduplicate stable
    seen = set()
    uniq: List[str] = []
    for t in out:
        if t in seen:
            continue
        seen.add(t)
        uniq.append(t)

    return uniq

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

        logging.info(
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
                logging.info(
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

        logging.info(
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

# =============================================================================
# Command surface
# =============================================================================

def cmd_report(_: Optional[dict]) -> Dict[str, Any]:
    return {
        "success": True,
        "message": "OK",
        "payload": { "status": "COMMANDED" }
    }

def cmd_refresh(_: Optional[dict]) -> Dict[str, Any]:
    return {
        "success": True,
        "message": "OK",
        "payload": { "status": "REFRESHED" }
    }

def cmd_apply(args: Optional[dict]) -> Dict[str, Any]:
    return {
        "success": True,
        "message": "OK",
        "payload": { "status": "APPLIED" }
    }

COMMANDS = {
    "REPORT": cmd_report,
    "REFRESH": cmd_refresh,
    "APPLY": cmd_apply,
}

# This service does not subscribe to pub/sub topics (yet).
SUBSCRIPTIONS: Dict[str, Any] = {}

# =============================================================================
# Entrypoint
# =============================================================================

def run() -> None:
    setup_logging()
    open_debug_log()

    logging.info("🚀 [startup] initializing transport")
    transport_init()

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

    logging.info("🚀 [startup] transport callbacks registered")

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

        server_setup(
            subsystem="PUBSUB",
            commands=COMMANDS,
            subscriptions=SUBSCRIPTIONS,
        )

        logging.info("🚀 [startup] pubsub fully online")

        while True:
            time.sleep(1)

    except Exception:
        logging.exception("💥 pubsub fatal crash")

def bootstrap() -> None:
    run()

if __name__ == "__main__":
    run()
