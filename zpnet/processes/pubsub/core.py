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
  • SERVER is a first-class machine with static formal routes and commands
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
TRANSPORT_RETRY_INTERVAL_S = 0.25   # retry transport subsystem initialization indefinitely
TRANSPORT_STATE_POLL_S = 0.10       # observe ready/lost generations without busy-spinning
RPC_REPLY_POLL_S = 0.10             # wake promptly when a transport generation disappears

# A serial generation is necessary but not sufficient for command readiness.
# Prove the Pi->Teensy->Pi request/response plane with a short non-mutating
# PUBSUB.REPORT round trip and refresh that generation-bound fact into SYSTEM.
TEENSY_RPC_PROBE_INTERVAL_S = 3.0
TEENSY_RPC_PROBE_TIMEOUT_S = 2.0
TEENSY_RPC_FEATURE_TTL_S = 10.0
TEENSY_RPC_STATUS_LOG_INTERVAL_S = 30.0

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
PI_DELIVERY_INITIAL_WARN_GRACE_S = 2.0  # ordinary process-start races stay silent

# Canonical formal routing graph.  Process lifetime never authors these edges.
# Keep this list byte-for-byte equivalent to the Teensy STATIC_ROUTES table.
#
# The current formal union contains no SERVER recipient.  SERVER remains a
# command/publish peer, but any future SERVER subscription is added here as code
# truth rather than declared over the live TCP connection.
STATIC_ROUTE_EDGES: Tuple[Tuple[str, str, str], ...] = (
    ("PI", "CLOCKS", "CLOCKS_FRAGMENT"),
    ("PI", "CLOCKS", "CLOCKS_RECOVERY_STALLED"),
    ("PI", "CLOCKS", "WATCHDOG_ANOMALY"),
    ("PI", "EVENTS", "EVENTS"),
    ("PI", "PHOTONS", "PHOTONS_FRAGMENT"),
    ("PI", "SYSTEM", "GNSS_ANNOUNCEMENT"),
)

# Retained only so retired private discovery helpers below remain harmless if
# called during a mixed-version diagnostic session.  They are not command API.
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

# Teensy can publish valid data immediately when the serial transport attaches.
# Retain that complete startup publication prefix until the static route table
# has been installed, then replay through the ordinary router in original order.
# In normal startup the static table is installed before transport opens, so the
# backlog is empty and per-target delivery custody begins immediately.

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

# PUBSUB process availability is independent of the serial device lifetime.
# Local sockets come up immediately; this state machine tracks successive
# Teensy transport generations underneath them.  A disappearing generation
# wakes every RPC bound to that generation immediately so a known USB outage
# never burns a 30-second semantic reply timeout.
transport_state_lock = threading.Lock()
transport_ready_event = threading.Event()
transport_state_ready = False
transport_generation = 0
transport_ready_transition_count = 0
transport_loss_transition_count = 0
transport_pending_retired_on_loss = 0
transport_last_transition_monotonic: Optional[float] = None
transport_last_transition = "PROCESS_START"

rpc_peer_disconnect_count = 0
rpc_unknown_response_count = 0
rpc_retired_response_count = 0
rpc_req_ts_mismatch_count = 0

# Application-level command-plane readiness.  This is intentionally separate
# from transport_state_ready: publications can flow while semantic RPC is not yet
# answering.  The proof is bound to one transport generation.
teensy_rpc_readiness_lock = threading.Lock()
teensy_rpc_readiness_wakeup = threading.Event()
teensy_rpc_readiness_status = "INITIALIZING"
teensy_rpc_readiness_generation = 0
teensy_rpc_readiness_reason = "PROCESS_START"
teensy_rpc_readiness_last_probe_monotonic: Optional[float] = None
teensy_rpc_readiness_last_success_monotonic: Optional[float] = None
teensy_rpc_readiness_probe_count = 0
teensy_rpc_readiness_success_count = 0
teensy_rpc_readiness_failure_count = 0
teensy_rpc_readiness_ever_ready = False
# The readiness worker is infrastructure and must survive ordinary USB/flash churn.
# Count escaped exceptions as forensic testimony rather than letting the daemon retain
# a permanently stale INITIALIZING/HOLD lease after the worker thread dies.
teensy_rpc_readiness_worker_exception_count = 0
teensy_rpc_readiness_last_worker_exception: Optional[str] = None
teensy_rpc_feature_publish_count = 0
teensy_rpc_feature_publish_fail_count = 0


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
    transport_generation_id: int,
) -> Dict[str, Any]:
    entry = {
        "queue": queue,
        "subsystem": subsystem,
        "command": command,
        "attempt": attempt,
        "source": source,
        "req_ts_ms": req_ts_ms,
        "transport_generation": int(transport_generation_id),
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


def _teensy_rpc_readiness_snapshot() -> Dict[str, Any]:
    with teensy_rpc_readiness_lock:
        last_probe = teensy_rpc_readiness_last_probe_monotonic
        last_success = teensy_rpc_readiness_last_success_monotonic
        return {
            "status": teensy_rpc_readiness_status,
            "generation": int(teensy_rpc_readiness_generation),
            "reason": teensy_rpc_readiness_reason,
            "probe_count": int(teensy_rpc_readiness_probe_count),
            "success_count": int(teensy_rpc_readiness_success_count),
            "failure_count": int(teensy_rpc_readiness_failure_count),
            "ever_ready": bool(teensy_rpc_readiness_ever_ready),
            "last_probe_age_s": (
                None if last_probe is None
                else round(max(0.0, time.monotonic() - last_probe), 3)
            ),
            "last_success_age_s": (
                None if last_success is None
                else round(max(0.0, time.monotonic() - last_success), 3)
            ),
            "feature_publish_count": int(teensy_rpc_feature_publish_count),
            "feature_publish_fail_count": int(teensy_rpc_feature_publish_fail_count),
            "worker_exception_count": int(teensy_rpc_readiness_worker_exception_count),
            "last_worker_exception": teensy_rpc_readiness_last_worker_exception,
            "probe_interval_s": float(TEENSY_RPC_PROBE_INTERVAL_S),
            "probe_timeout_s": float(TEENSY_RPC_PROBE_TIMEOUT_S),
            "feature_ttl_s": float(TEENSY_RPC_FEATURE_TTL_S),
        }


def _set_teensy_rpc_readiness(status: str, generation: int, reason: str) -> None:
    """Update local generation-bound readiness testimony."""
    global teensy_rpc_readiness_status, teensy_rpc_readiness_generation
    global teensy_rpc_readiness_reason
    normalized = str(status or "").strip().upper()
    if normalized not in {"INITIALIZING", "NOMINAL", "HOLD"}:
        raise ValueError(f"unsupported TEENSY_RPC readiness status {status!r}")
    with teensy_rpc_readiness_lock:
        teensy_rpc_readiness_status = normalized
        teensy_rpc_readiness_generation = int(generation)
        teensy_rpc_readiness_reason = str(reason)


def _transport_state_snapshot() -> Tuple[bool, int]:
    with transport_state_lock:
        return bool(transport_state_ready), int(transport_generation)


def _wait_for_transport_generation() -> int:
    """Wait indefinitely for one live serial generation; startup absence is normal."""
    while True:
        transport_ready_event.wait()
        ready, generation = _transport_state_snapshot()
        if ready and generation > 0:
            return generation


def _retire_transport_generation_pending(generation: int) -> int:
    """Wake every RPC whose serial generation is known to have disappeared."""
    retired_items: List[Tuple[int, Dict[str, Any]]] = []
    with state_lock:
        for req_id, entry in list(pending_replies.items()):
            if int(entry.get("transport_generation") or 0) != int(generation):
                continue
            retired_items.append((req_id, pending_replies.pop(req_id)))

    for req_id, entry in retired_items:
        _remember_reply_state(req_id, entry, "TRANSPORT_GENERATION_LOST")
        try:
            entry["queue"].put_nowait({
                "_pubsub_internal": "TRANSPORT_GENERATION_LOST",
                "transport_generation": int(generation),
            })
        except Full:
            # A real response won the race and already filled the private queue.
            pass
    return len(retired_items)


def _transport_state_monitor_loop() -> None:
    """Convert serial ready/lost transitions into explicit PUBSUB generations."""
    global transport_state_ready, transport_generation
    global transport_ready_transition_count, transport_loss_transition_count
    global transport_pending_retired_on_loss
    global transport_last_transition_monotonic, transport_last_transition

    while True:
        try:
            ready_now = bool(transport_wait_ready(timeout_s=TRANSPORT_STATE_POLL_S))
        except Exception:
            # The transport supervisor owns device errors.  PUBSUB treats this as
            # temporarily unavailable and keeps waiting.
            ready_now = False

        lost_generation: Optional[int] = None
        became_ready_generation: Optional[int] = None
        with transport_state_lock:
            if ready_now and not transport_state_ready:
                transport_generation += 1
                transport_state_ready = True
                transport_ready_transition_count += 1
                transport_last_transition_monotonic = time.monotonic()
                transport_last_transition = "READY"
                became_ready_generation = int(transport_generation)
                transport_ready_event.set()
            elif not ready_now and transport_state_ready:
                lost_generation = int(transport_generation)
                transport_state_ready = False
                transport_loss_transition_count += 1
                transport_last_transition_monotonic = time.monotonic()
                transport_last_transition = "LOST"
                transport_ready_event.clear()

        if lost_generation is not None:
            retired = _retire_transport_generation_pending(lost_generation)
            transport_pending_retired_on_loss += retired
            with teensy_rpc_readiness_lock:
                prior_rpc_ready = bool(teensy_rpc_readiness_ever_ready)
            _set_teensy_rpc_readiness(
                "HOLD" if prior_rpc_ready else "INITIALIZING",
                lost_generation,
                "TRANSPORT_GENERATION_LOST",
            )
            teensy_rpc_readiness_wakeup.set()
            logging.info(
                "⏳ [transport] serial generation=%d unavailable; "
                "retired_inflight_rpc=%d; waiting indefinitely for reattach",
                lost_generation, retired,
            )

        if became_ready_generation is not None:
            with teensy_rpc_readiness_lock:
                prior_rpc_ready = bool(teensy_rpc_readiness_ever_ready)
            _set_teensy_rpc_readiness(
                "HOLD" if prior_rpc_ready else "INITIALIZING",
                became_ready_generation,
                "TRANSPORT_READY_AWAITING_RPC_PROOF",
            )
            teensy_rpc_readiness_wakeup.set()
            if became_ready_generation > 1:
                logging.info(
                    "✅ [transport] serial reattached as generation=%d",
                    became_ready_generation,
                )
            else:
                logging.debug(
                    "[transport] initial serial generation=%d ready",
                    became_ready_generation,
                )

        if ready_now:
            time.sleep(TRANSPORT_STATE_POLL_S)


def _transport_supervisor_loop() -> None:
    """Start transport independently of every PUBSUB-facing socket and never give up."""
    announced = False
    while True:
        try:
            transport_init()
            break
        except Exception as exc:
            if not announced:
                logging.info(
                    "⏳ [transport] supervisor initialization unavailable; retrying silently: %r",
                    exc,
                )
                announced = True
            time.sleep(TRANSPORT_RETRY_INTERVAL_S)

    _transport_state_monitor_loop()


def _teensy_rpc_exchange(
    req: Dict[str, Any],
    *,
    subsystem: str,
    command: str,
    source: str,
    reply_timeout_s: float = REPLY_TIMEOUT_S,
    max_stable_timeouts: int = MAX_TEENSY_RETRIES,
    log_timeouts: bool = True,
) -> Dict[str, Any]:
    """Perform one semantic RPC within exactly one transport generation.

    Before the first successful send, startup transport churn is ordinary and
    the caller may wait for a sendable generation.  Once a command is transmitted,
    that generation owns the RPC.  Same-generation no-response timeouts may retry;
    loss of the bound generation terminates the RPC immediately and the command is
    never replayed into a successor generation.
    """
    timeout_s = float(reply_timeout_s)
    timeout_limit = int(max_stable_timeouts)
    if timeout_s <= 0.0 or timeout_limit <= 0:
        raise ValueError("RPC timeout and stable-timeout limit must be positive")
    stable_timeouts = 0
    bound_generation: Optional[int] = None

    while stable_timeouts < timeout_limit:
        if bound_generation is None:
            generation = _wait_for_transport_generation()
            ready, current_generation = _transport_state_snapshot()
            if not ready or current_generation != generation:
                continue
        else:
            ready, current_generation = _transport_state_snapshot()
            if not ready or current_generation != bound_generation:
                return {
                    "success": False,
                    "message": "TEENSY transport generation changed during RPC",
                    "payload": {},
                }
            generation = bound_generation

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
            attempt=stable_timeouts + 1,
            source=source,
            req_ts_ms=req_ts_ms,
            transport_generation_id=generation,
        )

        try:
            transport_send(TRAFFIC_REQUEST_RESPONSE, req)
            if bound_generation is None:
                bound_generation = generation
        except RuntimeError as exc:
            if str(exc) != "SERIAL transport is not ready":
                _retire_pending_reply(req_id, "SEND_FAILURE")
                raise
            _retire_pending_reply(req_id, "SEND_TRANSPORT_NOT_READY")
            continue
        except Exception:
            _retire_pending_reply(req_id, "SEND_FAILURE")
            raise

        deadline = time.monotonic() + timeout_s
        transport_lost = False
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            try:
                reply = q.get(timeout=min(RPC_REPLY_POLL_S, remaining))
            except Empty:
                ready, observed_generation = _transport_state_snapshot()
                if not ready or observed_generation != generation:
                    _retire_pending_reply(req_id, "TRANSPORT_GENERATION_LOST")
                    transport_lost = True
                    break
                continue

            if reply.get("_pubsub_internal") == "TRANSPORT_GENERATION_LOST":
                transport_lost = True
                break
            return reply

        if transport_lost:
            return {
                "success": False,
                "message": "TEENSY transport generation changed during RPC",
                "payload": {},
            }

        timed_out = _retire_pending_reply(req_id, "TIMEOUT")
        stable_timeouts += 1
        if timed_out is not None and log_timeouts:
            logging.warning(
                "⏱️ [rpc] Teensy timeout req_id=%d source=%s target=%s.%s "
                "attempt=%d/%d generation=%d timeout_s=%.1f",
                req_id, source, subsystem, command, stable_timeouts,
                timeout_limit, generation, timeout_s,
            )

    return {
        "success": False,
        "message": "TEENSY did not respond on a stable transport",
        "payload": {},
    }


def _publish_teensy_rpc_feature() -> None:
    """Refresh PI.PUBSUB.TEENSY_RPC into SYSTEM with an expiring lease."""
    global teensy_rpc_feature_publish_count, teensy_rpc_feature_publish_fail_count
    snapshot = _teensy_rpc_readiness_snapshot()
    try:
        response = send_command(
            machine="PI",
            subsystem="SYSTEM",
            command="SET_FEATURE",
            args={
                "machine": "PI",
                "subsystem": "PUBSUB",
                "feature": "TEENSY_RPC",
                "status": snapshot["status"],
                "ttl_s": TEENSY_RPC_FEATURE_TTL_S,
                "generation": snapshot["generation"],
            },
        )
    except Exception:
        teensy_rpc_feature_publish_fail_count += 1
        return
    if not isinstance(response, dict) or not response.get("success"):
        teensy_rpc_feature_publish_fail_count += 1
        return
    teensy_rpc_feature_publish_count += 1


def _teensy_rpc_readiness_loop() -> None:
    """Continuously prove semantic RPC and route custody on each serial generation."""
    global teensy_rpc_readiness_last_probe_monotonic
    global teensy_rpc_readiness_last_success_monotonic
    global teensy_rpc_readiness_probe_count, teensy_rpc_readiness_success_count
    global teensy_rpc_readiness_failure_count, teensy_rpc_readiness_ever_ready
    global teensy_rpc_readiness_worker_exception_count
    global teensy_rpc_readiness_last_worker_exception

    last_failure_log = 0.0
    while True:
        try:
            ready, generation = _transport_state_snapshot()
            if not ready or generation <= 0:
                with teensy_rpc_readiness_lock:
                    prior_ready = bool(teensy_rpc_readiness_ever_ready)
                _set_teensy_rpc_readiness(
                    "HOLD" if prior_ready else "INITIALIZING",
                    generation,
                    "TRANSPORT_UNAVAILABLE",
                )
                _publish_teensy_rpc_feature()
                teensy_rpc_readiness_wakeup.wait(timeout=TEENSY_RPC_PROBE_INTERVAL_S)
                teensy_rpc_readiness_wakeup.clear()
                continue

            before_probe = _teensy_rpc_readiness_snapshot()
            prior_ready = bool(before_probe.get("ever_ready"))
            already_nominal = bool(
                before_probe.get("status") == "NOMINAL"
                and int(before_probe.get("generation") or 0) == int(generation)
            )
            if not already_nominal:
                _set_teensy_rpc_readiness(
                    "HOLD" if prior_ready else "INITIALIZING",
                    generation,
                    "AWAITING_RPC_ROUND_TRIP",
                )
                _publish_teensy_rpc_feature()

            request = {
                "machine": "TEENSY",
                "subsystem": "PUBSUB",
                "command": "REPORT",
            }
            with teensy_rpc_readiness_lock:
                teensy_rpc_readiness_probe_count += 1
                teensy_rpc_readiness_last_probe_monotonic = time.monotonic()

            response = _teensy_rpc_exchange(
                request,
                subsystem="PUBSUB",
                command="REPORT",
                source="READINESS_PROBE",
                reply_timeout_s=TEENSY_RPC_PROBE_TIMEOUT_S,
                max_stable_timeouts=1,
                log_timeouts=False,
            )

            still_ready, observed_generation = _transport_state_snapshot()
            if not still_ready or observed_generation != generation:
                # The transport monitor owns the generation transition; never promote
                # testimony obtained across a different physical transport lifetime.
                continue

            if isinstance(response, dict) and response.get("success"):
                with teensy_rpc_readiness_lock:
                    teensy_rpc_readiness_success_count += 1
                    teensy_rpc_readiness_ever_ready = True
                    teensy_rpc_readiness_last_success_monotonic = time.monotonic()

                # A fresh Teensy can answer RPC while its volatile PUBSUB route table
                # is still empty.  Application readiness therefore requires both the
                # generation-bound round trip and convergence of the cached route union.
                # Use the REPORT payload we just proved so the common non-empty case
                # costs no second RPC.
                if already_nominal:
                    route_ok, route_reason = True, "ALREADY_ADMITTED"
                else:
                    payload = response.get("payload")
                    route_ok, route_reason = _ensure_teensy_route_custody(
                        generation,
                        payload if isinstance(payload, dict) else None,
                        source="READINESS",
                    )
                if route_ok:
                    _set_teensy_rpc_readiness(
                        "NOMINAL", generation, "PUBSUB_REPORT_AND_ROUTES_PROVED"
                    )
                    if not already_nominal:
                        logging.info(
                            "✅ [pubsub/readiness] Teensy RPC and route custody proved "
                            "on transport generation=%d",
                            generation,
                        )
                else:
                    _set_teensy_rpc_readiness(
                        "HOLD",
                        generation,
                        f"ROUTE_CUSTODY_NOT_PROVED:{route_reason}",
                    )
                    now = time.monotonic()
                    if now - last_failure_log >= TEENSY_RPC_STATUS_LOG_INTERVAL_S:
                        logging.info(
                            "⏳ [pubsub/readiness] transport generation=%d has RPC "
                            "but route custody is not yet proved: %s",
                            generation, route_reason,
                        )
                        last_failure_log = now
            else:
                with teensy_rpc_readiness_lock:
                    teensy_rpc_readiness_failure_count += 1
                    prior_ready = bool(teensy_rpc_readiness_ever_ready)
                reason = str(
                    response.get("message") if isinstance(response, dict) else "malformed response"
                )
                _set_teensy_rpc_readiness(
                    "HOLD" if prior_ready else "INITIALIZING",
                    generation,
                    f"RPC_PROBE_FAILED:{reason}",
                )
                now = time.monotonic()
                if now - last_failure_log >= TEENSY_RPC_STATUS_LOG_INTERVAL_S:
                    logging.info(
                        "⏳ [pubsub/readiness] transport generation=%d is attached but "
                        "Teensy RPC is not yet proved: %s",
                        generation, reason,
                    )
                    last_failure_log = now

            _publish_teensy_rpc_feature()
            teensy_rpc_readiness_wakeup.wait(timeout=TEENSY_RPC_PROBE_INTERVAL_S)
            teensy_rpc_readiness_wakeup.clear()

        except Exception as exc:
            # USB disappearance may race a readiness probe after the transport monitor
            # observed the generation as ready.  transport_send() can then raise from
            # the serial flush path (for example termios EIO).  That is transport churn,
            # not permission for this daemon-lifetime worker to die.  Preserve the
            # traceback as testimony, publish a non-NOMINAL lease, and retry forever.
            teensy_rpc_readiness_worker_exception_count += 1
            teensy_rpc_readiness_last_worker_exception = (
                f"{type(exc).__module__}.{type(exc).__name__}: {exc}"
            )
            ready_now, generation_now = _transport_state_snapshot()
            with teensy_rpc_readiness_lock:
                prior_ready = bool(teensy_rpc_readiness_ever_ready)
            _set_teensy_rpc_readiness(
                "HOLD" if prior_ready else "INITIALIZING",
                generation_now,
                f"READINESS_WORKER_EXCEPTION:{type(exc).__name__}",
            )
            logging.exception(
                "⚠️ [pubsub/readiness] readiness iteration failed but worker remains "
                "alive: transport_ready=%s generation=%d exception_count=%d",
                ready_now, generation_now, teensy_rpc_readiness_worker_exception_count,
            )
            _publish_teensy_rpc_feature()
            teensy_rpc_readiness_wakeup.wait(timeout=TRANSPORT_RETRY_INTERVAL_S)
            teensy_rpc_readiness_wakeup.clear()

def _send_client_bytes_best_effort(conn: socket.socket, raw: bytes, *, context: str) -> bool:
    """A caller disappearing before its reply arrives is ordinary IPC churn."""
    global rpc_peer_disconnect_count
    try:
        conn.sendall(raw)
        return True
    except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError) as exc:
        rpc_peer_disconnect_count += 1
        logging.debug("[pubsub] %s peer disappeared before reply: %r", context, exc)
        return False

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
        # One delivery outage episode spans queue heads until the target catches
        # fully back up.  This prevents a temporarily saturated/unavailable
        # subscriber from generating a fresh traceback for every retained row.
        self.blocked_since_monotonic: Optional[float] = None
        self.blocked_last_log_monotonic: Optional[float] = None
        self.blocked_retry_count = 0
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
# Formal SERVER routing, like PI and TEENSY routing, is code-owned by the static
# route graph above.  Legacy SERVER subscribe frames are ignored and counted.
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
# Legacy compatibility container for retired discovery helpers below.  It is
# intentionally never populated and is not topology authority.
server_subscriptions: List[Dict[str, Any]] = []
server_pending_commands: Dict[int, Queue] = {}
server_subscription_declarations_ignored = 0

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
teensy_route_monitor_last_status = "NOT_STARTED"
# One owner serializes route inspection/reapply/verification across immediate
# generation admission and the periodic custody monitor.
teensy_route_custody_lock = threading.Lock()
teensy_route_custody_worker_exception_count = 0
teensy_route_custody_last_worker_exception: Optional[str] = None

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
    global rpc_unknown_response_count, rpc_retired_response_count
    global rpc_req_ts_mismatch_count

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
        entry = pending_replies.get(req_id)
        retired = recent_replies.get(req_id)

    if entry is None:
        if retired is None:
            rpc_unknown_response_count += 1
            logging.debug(
                "[rpc] late/unknown response req_id=%r req_ts_ms=%r discarded",
                req_id, payload.get("req_ts_ms"),
            )
        else:
            rpc_retired_response_count += 1
            logging.debug(
                "[rpc] late response for retired req_id=%r classification=%s discarded",
                req_id, retired.get("outcome"),
            )
        return

    sent_ms = payload.get("req_ts_ms")
    if sent_ms is None:
        logging.error(
            "❌ [rpc] response missing req_ts_ms req_id=%r source=%s "
            "target=%s.%s attempt=%d — discarded without stealing pending custody",
            req_id, entry["source"], entry["subsystem"],
            entry["command"], entry["attempt"],
        )
        return

    try:
        response_ts_ms = int(sent_ms)
    except (TypeError, ValueError):
        logging.error(
            "❌ [rpc] response has invalid req_ts_ms req_id=%r value=%r — discarded",
            req_id, sent_ms,
        )
        return

    if response_ts_ms != int(entry["req_ts_ms"]):
        # PUBSUB request IDs restart with the process.  A delayed response from a
        # previous PUBSUB lifetime can therefore reuse the same numeric req_id.
        # req_ts_ms is the second half of the wire identity: never let stale
        # testimony steal the current pending request.
        rpc_req_ts_mismatch_count += 1
        logging.debug(
            "[rpc] stale response identity req_id=%r response_ts=%r current_ts=%r discarded",
            req_id, response_ts_ms, entry["req_ts_ms"],
        )
        return

    with state_lock:
        current = pending_replies.get(req_id)
        if current is not entry:
            return
        pending_replies.pop(req_id, None)

    latency = int(time.monotonic() * 1000) - response_ts_ms
    payload["latency"] = latency
    _remember_reply_state(req_id, entry, "DELIVERED")
    try:
        entry["queue"].put_nowait(payload)
    except Full:
        logging.error(
            "❌ [rpc] private reply queue unexpectedly full req_id=%r target=%s.%s",
            req_id, entry["subsystem"], entry["command"],
        )

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

        subsystem = str(req.get("subsystem") or "")
        command = str(req.get("command") or "")
        reply = _teensy_rpc_exchange(
            req,
            subsystem=subsystem,
            command=command,
            source="PI_RPC",
        )
        _send_client_bytes_best_effort(
            conn,
            json.dumps(reply, separators=(",", ":")).encode(),
            context=f"PI_RPC {subsystem}.{command}",
        )
    except Exception:
        # Programming/protocol defects remain loud.  Peer disappearance and
        # transport churn are handled below this boundary and never traceback.
        logging.exception("💥 [rpc] unexpected local RPC broker failure")
    finally:
        try:
            conn.close()
        except Exception:
            pass


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
) -> Tuple[bool, Optional[Dict[str, Any]]]:
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
        return True, None

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
        return False, {
            "socket": sock_path,
            "phase": "CONNECT" if connected is None else "SEND",
            "connect_ms": connect_ms,
            "send_ms": send_ms,
            "total_ms": (failed - started) * 1000.0,
            "timeout_s": PI_FANOUT_SOCKET_TIMEOUT_S,
            "error_type": type(exc).__name__,
            "error": repr(exc),
        }


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

        while True:
            delivered, failure = _fanout_to_pi(topic, target.subsystem, raw)
            if delivered:
                delivered_at = time.monotonic()
                backlog = target.queue.qsize()
                with pi_delivery_lock:
                    was_blocked = target.blocked_since_monotonic is not None
                    blocked_since = target.blocked_since_monotonic
                    episode_retries = target.blocked_retry_count
                    episode_was_logged = target.blocked_last_log_monotonic is not None
                    target.delivered += 1
                    target.last_delivered_topic = topic
                    target.last_delivered_sequence = sequence
                    target.last_delivery_monotonic = delivered_at
                    target.current_topic = None
                    target.current_sequence = None
                    target.current_enqueued_monotonic = None

                    caught_up = was_blocked and backlog == 0
                    if caught_up:
                        target.blocked_since_monotonic = None
                        target.blocked_last_log_monotonic = None
                        target.blocked_retry_count = 0

                target.queue.task_done()
                # A sub-second process-start race is invisible.  Only close an
                # outage episode in INFO if we previously had reason to announce it.
                if caught_up and episode_was_logged:
                    blocked_ms = (
                        (delivered_at - blocked_since) * 1000.0
                        if blocked_since is not None else 0.0
                    )
                    logging.info(
                        "✅ [pubsub] Pi delivery custody caught up target=PI:%s "
                        "topic=%s sequence=%s outage_ms=%.1f retries=%d backlog=0",
                        target.subsystem, topic, sequence, blocked_ms, episode_retries,
                    )
                break

            failure = failure or {}
            now = time.monotonic()
            with pi_delivery_lock:
                first_block = target.blocked_since_monotonic is None
                if first_block:
                    target.blocked_since_monotonic = now
                    target.blocked_retry_count = 0
                target.retry_count += 1
                target.blocked_retry_count += 1
                blocked_since = target.blocked_since_monotonic or now
                episode_retries = target.blocked_retry_count
                total_retries = target.retry_count
                last_log = target.blocked_last_log_monotonic
                blocked_s = max(0.0, now - blocked_since)
                log_failure = (
                    blocked_s >= PI_DELIVERY_INITIAL_WARN_GRACE_S
                    and (
                        last_log is None
                        or now - last_log >= PI_DELIVERY_RETRY_LOG_INTERVAL_S
                    )
                )
                if log_failure:
                    target.blocked_last_log_monotonic = now

            if log_failure:
                logging.warning(
                    "⏳ [pubsub] Pi subscriber unavailable; retaining ordered custody "
                    "target=PI:%s topic=%s sequence=%s backlog=%d blocked_s=%.1f "
                    "episode_retries=%d total_retries=%d phase=%s error=%s",
                    target.subsystem, topic, sequence, target.queue.qsize() + 1,
                    blocked_s, episode_retries, total_retries,
                    failure.get("phase"), failure.get("error"),
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


def _install_static_routes() -> None:
    """Install the compile-time route graph before transport can deliver data."""
    new_routes: Dict[str, Set[Tuple[str, str]]] = {}
    for machine, subsystem, topic in STATIC_ROUTE_EDGES:
        new_routes.setdefault(topic, set()).add((machine, subsystem))

    route_count = sum(len(targets) for targets in new_routes.values())
    if route_count != len(STATIC_ROUTE_EDGES):
        raise RuntimeError("PUBSUB static route graph contains duplicate edges")

    with state_lock:
        routes_by_topic.clear()
        routes_by_topic.update(new_routes)

        # Keep the legacy diagnostic cache truthful, but it is no longer an
        # authority and is never reconstructed from process declarations.
        applied_union.clear()
        grouped: Dict[Tuple[str, str], List[str]] = {}
        for machine, subsystem, topic in STATIC_ROUTE_EDGES:
            grouped.setdefault((machine, subsystem), []).append(topic)
        applied_union["subscriptions"] = [
            {
                "machine": machine,
                "subsystem": subsystem,
                "subscriptions": [
                    {"name": topic} for topic in sorted(topics)
                ],
            }
            for (machine, subsystem), topics in sorted(grouped.items())
        ]

    logging.info(
        "🚀 [pubsub] static formal routing installed (%d routes, %d topics)",
        route_count,
        len(new_routes),
    )
    if not _release_startup_custody_after_route_commit():
        raise RuntimeError("PUBSUB static route install could not release startup custody")


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
        except Exception as exc:
            logging.debug("[pubsub] SERVER send skipped after disconnect: %r", exc)


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

    Clears the socket and any pending command relays.  Formal routing is static
    code truth and therefore does not change when SERVER disconnects.
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
      • "subscribe"  — legacy declaration frame; ignored (topology is static)
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

    except (ConnectionResetError, BrokenPipeError, ConnectionAbortedError, OSError) as exc:
        logging.debug("[pubsub] SERVER reader connection ended: %r", exc)
    except Exception:
        logging.exception("💥 [pubsub] SERVER reader protocol failure")
    finally:
        _server_disconnect("reader exited")


def _server_handle_subscribe(msg: Dict[str, Any]) -> None:
    """Reject SERVER topology authorship while preserving wire compatibility."""
    global server_subscription_declarations_ignored

    declared = msg.get("subscriptions")
    declared_count = len(declared) if isinstance(declared, list) else 0
    with state_lock:
        server_subscription_declarations_ignored += 1

    logging.debug(
        "[pubsub] ignored legacy SERVER subscription declaration (%d entries); "
        "formal topology is static code truth",
        declared_count,
    )


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
    """Route a SERVER-originated command through the same resilient Teensy RPC court."""
    req: Dict[str, Any] = {
        "machine": "TEENSY",
        "subsystem": subsystem,
        "command": command,
    }
    if args is not None:
        req["args"] = args
    return _teensy_rpc_exchange(
        req, subsystem=subsystem, command=command, source=source
    )


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
        req_id = _next_req_id()

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

        _send_client_bytes_best_effort(
            conn,
            json.dumps(relay_resp, separators=(",", ":")).encode("utf-8"),
            context="SERVER_CMD_RELAY",
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
                logging.warning(
                    "⚠️ [pubsub] malformed PI publication ignored "
                    "(bytes=%d preview=%r)",
                    len(raw), raw[:256],
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
                "transport_generation": entry.get("transport_generation"),
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
                "transport_generation": entry.get("transport_generation"),
            }
            for req_id, entry in list(recent_replies.items())[-16:]
        ]
        server_cmd_pending = len(server_pending_commands)
        adhoc_client_count = len(adhoc_clients)
        adhoc_topic_count = len(adhoc_by_topic)
        applied_subscription_count = len(applied_union.get("subscriptions", []))
        applied_topic_count = len(routes_by_topic)
        static_route_count = sum(len(targets) for targets in routes_by_topic.values())
        static_server_route_count = sum(
            1 for machine, _, _ in STATIC_ROUTE_EDGES if machine == "SERVER"
        )
        ignored_server_subscriptions = server_subscription_declarations_ignored
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
            "worker_exception_count": teensy_route_custody_worker_exception_count,
            "last_worker_exception": teensy_route_custody_last_worker_exception,
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
                "blocked_episode_retry_count": target.blocked_retry_count,
                "last_failure_log_age_ms": (
                    int((now - target.blocked_last_log_monotonic) * 1000)
                    if target.blocked_last_log_monotonic is not None
                    else None
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

    transport_ready, transport_generation_now = _transport_state_snapshot()
    teensy_rpc_readiness = _teensy_rpc_readiness_snapshot()

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "pending_reply_count": pending_count,
            "transport": {
                "ready": transport_ready,
                "generation": transport_generation_now,
                "ready_transition_count": transport_ready_transition_count,
                "loss_transition_count": transport_loss_transition_count,
                "pending_retired_on_loss": transport_pending_retired_on_loss,
                "last_transition": transport_last_transition,
                "last_transition_age_ms": (
                    int((time.monotonic() - transport_last_transition_monotonic) * 1000)
                    if transport_last_transition_monotonic is not None else None
                ),
            },
            "rpc_peer_disconnect_count": rpc_peer_disconnect_count,
            "rpc_unknown_response_count": rpc_unknown_response_count,
            "rpc_retired_response_count": rpc_retired_response_count,
            "rpc_req_ts_mismatch_count": rpc_req_ts_mismatch_count,
            "teensy_rpc_readiness": teensy_rpc_readiness,
            "pending_req_ids": pending_ids[:50],  # cap for sanity
            "pending_reply_detail": pending_detail[:50],
            "recent_reply_detail": recent_detail,
            "req_id_last_issued": req_id_last_issued,
            "routes_topic_count": len(routes_by_topic),
            "static_route_count": static_route_count,
            "static_server_route_count": static_server_route_count,
            "applied_topic_count": applied_topic_count,
            "applied_subscription_count": applied_subscription_count,
            "active_threads": threading.active_count(),
            "server_connected": server_connected,
            "server_subscription_declarations_ignored": ignored_server_subscriptions,
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


def _teensy_route_report(*, source: str = "ROUTE_MONITOR") -> Optional[Dict[str, Any]]:
    """Ask Teensy PUBSUB for its current route table."""
    resp = _server_command_to_teensy(
        "PUBSUB", "REPORT", None, source=source
    )
    if not resp.get("success"):
        return None
    payload = resp.get("payload")
    return payload if isinstance(payload, dict) else None


def _teensy_route_count(payload: Dict[str, Any]) -> Optional[int]:
    routes = payload.get("routes")
    return len(routes) if isinstance(routes, list) else None


def _teensy_route_reapply_cached_union(
    union_payload: Dict[str, Any], *, source: str = "ROUTE_MONITOR_REAPPLY"
) -> bool:
    """Reapply the cached Pi-side union to Teensy with SETSUBSCRIPTIONS."""
    resp = _server_command_to_teensy(
        "PUBSUB", "SETSUBSCRIPTIONS", union_payload, source=source
    )
    return bool(resp.get("success"))


def _ensure_teensy_route_custody(
    generation: int,
    initial_report: Optional[Dict[str, Any]] = None,
    *,
    source: str,
) -> Tuple[bool, str]:
    """Prove or restore the volatile Teensy route table for one transport generation."""
    global teensy_route_monitor_report_ok_count
    global teensy_route_monitor_empty_count
    global teensy_route_monitor_reapply_count
    global teensy_route_monitor_reapply_fail_count
    global teensy_route_monitor_last_route_count
    global teensy_route_monitor_last_empty_ts
    global teensy_route_monitor_last_reapply_ts
    global teensy_route_monitor_last_reapply_topic_count
    global teensy_route_monitor_last_reapply_subscription_count
    global teensy_route_monitor_last_status

    with teensy_route_custody_lock:
        ready, current_generation = _transport_state_snapshot()
        if not ready or current_generation != int(generation):
            teensy_route_monitor_last_status = "GENERATION_CHANGED"
            return False, "TRANSPORT_GENERATION_CHANGED"

        payload = initial_report
        if payload is None:
            payload = _teensy_route_report(source=f"{source}_REPORT")
        if payload is None:
            teensy_route_monitor_last_status = "TEENSY_UNREACHABLE"
            return False, "ROUTE_REPORT_UNAVAILABLE"

        route_count = _teensy_route_count(payload)
        if route_count is None:
            teensy_route_monitor_last_status = "ROUTE_REPORT_INVALID"
            return False, "ROUTE_REPORT_INVALID"

        teensy_route_monitor_report_ok_count += 1
        teensy_route_monitor_last_route_count = route_count
        if route_count != 0:
            teensy_route_monitor_last_status = "NOMINAL"
            return True, "ROUTES_PRESENT"

        union_payload = _copy_applied_union_for_teensy_route_reapply()
        subscriptions = union_payload.get("subscriptions")
        if not isinstance(subscriptions, list) or len(subscriptions) == 0:
            teensy_route_monitor_last_status = "EMPTY_NO_CACHED_UNION"
            return False, "EMPTY_NO_CACHED_UNION"

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
            "🔄 [pubsub] Teensy route table empty on generation=%d; "
            "reapplying cached union (%d subscriptions, %d topics) source=%s",
            generation, len(subscriptions), len(topic_names), source,
        )

        if not _teensy_route_reapply_cached_union(
            union_payload, source=f"{source}_REAPPLY"
        ):
            teensy_route_monitor_reapply_fail_count += 1
            teensy_route_monitor_last_status = "REAPPLY_FAILED"
            return False, "REAPPLY_FAILED"

        # SETSUBSCRIPTIONS success is not the proof.  Read back the same producer
        # generation and require a non-empty route table before application admission.
        ready, current_generation = _transport_state_snapshot()
        if not ready or current_generation != int(generation):
            teensy_route_monitor_last_status = "GENERATION_CHANGED_AFTER_REAPPLY"
            return False, "TRANSPORT_GENERATION_CHANGED_AFTER_REAPPLY"

        verify_payload = _teensy_route_report(source=f"{source}_VERIFY")
        if verify_payload is None:
            teensy_route_monitor_reapply_fail_count += 1
            teensy_route_monitor_last_status = "REAPPLY_VERIFY_UNAVAILABLE"
            return False, "REAPPLY_VERIFY_UNAVAILABLE"

        verify_count = _teensy_route_count(verify_payload)
        if verify_count is None or verify_count == 0:
            teensy_route_monitor_reapply_fail_count += 1
            teensy_route_monitor_last_route_count = verify_count
            teensy_route_monitor_last_status = "REAPPLY_VERIFY_FAILED"
            return False, "REAPPLY_VERIFY_FAILED"

        teensy_route_monitor_reapply_count += 1
        teensy_route_monitor_last_reapply_ts = time.time()
        teensy_route_monitor_last_reapply_topic_count = len(topic_names)
        teensy_route_monitor_last_reapply_subscription_count = len(subscriptions)
        teensy_route_monitor_last_route_count = verify_count
        teensy_route_monitor_last_status = "REAPPLIED_CACHED_UNION"
        logging.info(
            "✅ [pubsub] Teensy route table restored and read-back proved "
            "on generation=%d routes=%d",
            generation, verify_count,
        )
        return True, "REAPPLIED_AND_VERIFIED"


def teensy_route_monitor_loop() -> None:
    """Periodically prove route custody even after generation admission."""
    global teensy_route_monitor_probe_count
    global teensy_route_monitor_probe_fail_count
    global teensy_route_monitor_last_probe_ts
    global teensy_route_monitor_last_status
    global teensy_route_custody_worker_exception_count
    global teensy_route_custody_last_worker_exception

    logging.info(
        "🧭 [pubsub] Teensy route monitor armed (interval=%.1fs)",
        TEENSY_ROUTE_MONITOR_INTERVAL_S,
    )

    while True:
        time.sleep(TEENSY_ROUTE_MONITOR_INTERVAL_S)
        try:
            ready, generation = _transport_state_snapshot()
            if not ready or generation <= 0:
                teensy_route_monitor_last_status = "TRANSPORT_UNAVAILABLE"
                continue

            teensy_route_monitor_probe_count += 1
            teensy_route_monitor_last_probe_ts = time.time()
            ok, reason = _ensure_teensy_route_custody(
                generation, source="ROUTE_MONITOR"
            )
            if not ok:
                teensy_route_monitor_probe_fail_count += 1
                ready_now, observed_generation = _transport_state_snapshot()
                if ready_now and observed_generation == generation:
                    with teensy_rpc_readiness_lock:
                        prior_ready = bool(teensy_rpc_readiness_ever_ready)
                    _set_teensy_rpc_readiness(
                        "HOLD" if prior_ready else "INITIALIZING",
                        generation,
                        f"ROUTE_MONITOR_NOT_PROVED:{reason}",
                    )
                    _publish_teensy_rpc_feature()
                    teensy_rpc_readiness_wakeup.set()
                    logging.info(
                        "⏳ [pubsub] periodic Teensy route custody not proved "
                        "generation=%d reason=%s; readiness held for immediate retry",
                        generation, reason,
                    )
        except Exception as exc:
            teensy_route_custody_worker_exception_count += 1
            teensy_route_custody_last_worker_exception = (
                f"{type(exc).__module__}.{type(exc).__name__}: {exc}"
            )
            teensy_route_monitor_last_status = (
                f"WORKER_EXCEPTION:{type(exc).__name__}"
            )
            ready_now, generation_now = _transport_state_snapshot()
            with teensy_rpc_readiness_lock:
                prior_ready = bool(teensy_rpc_readiness_ever_ready)
            _set_teensy_rpc_readiness(
                "HOLD" if prior_ready else "INITIALIZING",
                generation_now,
                f"ROUTE_MONITOR_EXCEPTION:{type(exc).__name__}",
            )
            _publish_teensy_rpc_feature()
            teensy_rpc_readiness_wakeup.set()
            logging.exception(
                "⚠️ [pubsub] route custody monitor iteration failed but worker "
                "remains alive: exception_count=%d",
                teensy_route_custody_worker_exception_count,
            )

# ---------------------------------------------------------------------
# Declarations
# ---------------------------------------------------------------------

COMMANDS = {
    "REPORT": cmd_report,
    "DIAGNOSTICS": cmd_diagnostics,
}

# ---------------------------------------------------------------------
# Transport init with aggressive retry
# ---------------------------------------------------------------------

# Transport startup is intentionally asynchronous.  _transport_supervisor_loop()
# above owns indefinite serial initialization and generation tracking; PUBSUB's
# local sockets never wait for a device to exist.

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

    # Routing and receive callbacks are code truth and exist before either side
    # of the transport.  This closes the only ordering requirement PUBSUB owns.
    _install_static_routes()
    transport_register_receive_callback(TRAFFIC_DEBUG, on_receive_debug)
    transport_register_receive_callback(TRAFFIC_REQUEST_RESPONSE, on_receive_request_response)
    transport_register_receive_callback(TRAFFIC_PUBLISH_SUBSCRIBE, on_receive_publish_subscribe)

    # Expose every Pi-facing endpoint immediately.  A missing Teensy is now a
    # transport state underneath PUBSUB, not a reason for PUBSUB itself to be
    # unavailable to independently-starting services.
    threading.Thread(target=rpc_server, daemon=True, name="pubsub-teensy-rpc").start()
    threading.Thread(target=pubsub_server, daemon=True, name="pubsub-local-publish").start()
    threading.Thread(
        target=adhoc_tap_server, daemon=True, name="pubsub-adhoc-tap"
    ).start()
    threading.Thread(
        target=server_tcp_listener, daemon=True, name="server-tcp-listener"
    ).start()
    threading.Thread(
        target=server_cmd_relay_server, daemon=True, name="server-cmd-relay"
    ).start()

    # Serial availability is fully asynchronous and self-healing.  This thread
    # retries initialization forever, tracks attach generations, and wakes any
    # in-flight RPC immediately when its generation disappears.
    threading.Thread(
        target=_transport_supervisor_loop,
        daemon=True,
        name="pubsub-transport-supervisor",
    ).start()
    threading.Thread(
        target=_teensy_rpc_readiness_loop,
        daemon=True,
        name="pubsub-teensy-rpc-readiness",
    ).start()
    threading.Thread(
        target=teensy_route_monitor_loop,
        daemon=True,
        name="pubsub-teensy-route-custody",
    ).start()

    # PUBSUB's own command surface is also independent of serial readiness.
    # server_setup owns process lifetime and never returns in normal operation.
    server_setup(
        subsystem="PUBSUB",
        commands=COMMANDS,
    )


def bootstrap() -> None:
    run()