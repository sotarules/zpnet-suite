"""
ZPNet Process Runtime (Pi-Side)

Provides a unified, declarative runtime for Pi-side processes.

A process declares:
  • its subsystem name
  • the commands it serves
  • optionally, one publication-ingress callback

Formal pub/sub topology is not declared by processes.  PUBSUB owns that graph
as static code truth; this module only exposes the target socket and dispatches
an already-routed publication to the owning process.

All transport, sockets, threads, and routing are owned here.

Semantics:
  • Commands are synchronous (request → response)
  • Pub/Sub is asynchronous, best-effort
  • No polling
  • No retries
  • No lifecycle obsession
  • Subscription state is clobber-and-go

Machine routing:
  • PI      — direct Unix domain socket to subsystem
  • TEENSY  — via pubsub RPC broker (Unix socket → HID/serial)
  • SERVER  — via pubsub TCP bridge (Unix socket → TCP → Meteor)

Author: The Mule + GPT
"""

from __future__ import annotations

import json
import logging
import os
import socket
import sys
import threading
import time
from typing import Any, Dict, Callable, Optional

from zpnet.shared.constants import Payload

# =============================================================================
# Socket naming
# =============================================================================

SOCKET_DIR = "/tmp"

def command_socket_path(subsystem: str) -> str:
    return f"{SOCKET_DIR}/zpnet-{subsystem.lower()}-command.sock"

def pubsub_socket_path(subsystem: str) -> str:
    return f"{SOCKET_DIR}/zpnet-{subsystem.lower()}-pubsub.sock"


TEENSY_REQUEST_RESPONSE_SOCKET = "/tmp/zpnet_teensy_rt.sock"
TEENSY_PUBLISH_SUBSCRIBE_SOCKET = "/tmp/zpnet_teensy_ps.sock"


# Read-only local tap exposed by the Pi PUBSUB broker. Dashboard-style clients
# use this surface to keep last-known-good topic snapshots without issuing
# command/response requests during repaint.
PUBSUB_TAP_SOCKET = "/tmp/zpnet_pubsub_tap.sock"

# Pi PUBSUB fan-out uses one Unix-stream connection per publication and closes
# that connection after sendall().  EOF, not an arbitrary recv() boundary, is
# therefore the message delimiter.  Keep the bound generous beside current
# ~40 KiB MONITOR traffic while still rejecting a runaway peer.
PUBSUB_RECV_CHUNK_BYTES = 65536
PUBSUB_MAX_MESSAGE_BYTES = 1024 * 1024


class PubSubTapCache:
    """Maintain latest payloads for a fixed set of local PUBSUB topics."""

    def __init__(self, *topics: str, reconnect_s: float = 1.0):
        normalized = tuple(sorted({str(topic) for topic in topics if str(topic)}))
        if not normalized:
            raise ValueError("at least one topic is required")
        self._topics = normalized
        self._reconnect_s = float(reconnect_s)
        self._lock = threading.Lock()
        self._payloads: Dict[str, Dict[str, Any]] = {}
        self._updated_monotonic: Dict[str, float] = {}
        self._error: Optional[str] = None
        self._thread: Optional[threading.Thread] = None

    def start(self) -> "PubSubTapCache":
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return self
            self._thread = threading.Thread(
                target=self._listen_loop,
                name="zpnet-pubsub-tap-" + "-".join(t.lower() for t in self._topics),
                daemon=True,
            )
            self._thread.start()
        return self

    def get(self, topic: str) -> Optional[Dict[str, Any]]:
        self.start()
        with self._lock:
            payload = self._payloads.get(str(topic))
            return dict(payload) if isinstance(payload, dict) else None

    def age_s(self, topic: str) -> Optional[float]:
        self.start()
        with self._lock:
            updated = self._updated_monotonic.get(str(topic))
        return None if updated is None else max(0.0, time.monotonic() - updated)

    def error(self) -> Optional[str]:
        self.start()
        with self._lock:
            return self._error

    def _listen_loop(self) -> None:
        subscribe = (
            json.dumps(
                {"type": "set_topics", "topics": list(self._topics)},
                separators=(",", ":"),
            ).encode("utf-8")
            + b"\n"
        )

        while True:
            try:
                with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                    sock.settimeout(2.0)
                    sock.connect(PUBSUB_TAP_SOCKET)
                    sock.settimeout(None)

                    with sock.makefile("rwb") as stream:
                        stream.write(subscribe)
                        stream.flush()
                        with self._lock:
                            self._error = None

                        for raw in stream:
                            try:
                                msg = json.loads(raw.decode("utf-8"))
                            except Exception:
                                continue
                            if not isinstance(msg, dict) or msg.get("type") != "publish":
                                continue
                            topic = str(msg.get("topic") or "")
                            payload = msg.get("payload")
                            if topic not in self._topics or not isinstance(payload, dict):
                                continue
                            with self._lock:
                                self._payloads[topic] = dict(payload)
                                self._updated_monotonic[topic] = time.monotonic()
                                self._error = None
            except Exception as exc:
                with self._lock:
                    self._error = str(exc)
                time.sleep(self._reconnect_s)


def create_pubsub_cache(*topics: str, reconnect_s: float = 1.0) -> PubSubTapCache:
    """Create and start a last-known-good cache for local PUBSUB topics."""
    return PubSubTapCache(*topics, reconnect_s=reconnect_s).start()

# =============================================================================
# SERVER command relay socket
# =============================================================================
#
# Commands destined for SERVER are relayed through pubsub's TCP bridge.
# This Unix domain socket is owned by pubsub and speaks the same
# request/response JSON protocol as the Teensy RPC socket.
#
# Wire format (identical to TEENSY RPC):
#   Request:  {"machine": "SERVER", "subsystem": "...", "command": "...", "args": {...}}
#   Response: {"success": true|false, "message": "...", "payload": {...}}
#
# This socket exists only when pubsub is running.
# Failures propagate the same way as TEENSY failures.
#
# =============================================================================

SERVER_COMMAND_SOCKET = "/tmp/zpnet_server_cmd.sock"

# =============================================================================
# Helpers
# =============================================================================

def list_subsystems() -> list[str]:
    """
    Return a list of active Pi-side subsystems.

    Semantics:
      • Discovered by presence of command sockets
      • Filesystem is the source of truth
      • Returned names are UPPERCASE (protocol identity)
      • Order is sorted for determinism
    """
    subsystems: set[str] = set()

    for fname in os.listdir(SOCKET_DIR):
        if not fname.startswith("zpnet-"):
            continue
        if not fname.endswith("-command.sock"):
            continue

        subsystem_fs = fname[len("zpnet-"):-len("-command.sock")]
        subsystems.add(subsystem_fs.upper())

    return sorted(subsystems)

# =============================================================================
# Canonical RPC client
# =============================================================================

def send_command(
    *,
    machine: str,
    subsystem: str,
    command: str,
    args: Optional[Dict[str, Any]] = None,
    retries: int = 5,
    retry_delay_s: float = 0.25,
) -> Dict[str, Any]:
    """
    Send a single command and return a single response.

    Semantics:
      • JSON-in / JSON-out
      • Bounded retries at unowned transport boundary
      • Any persistent failure propagates

    Machine routing:
      • PI      — direct to subsystem's Unix command socket
      • TEENSY  — via pubsub RPC broker (TEENSY_REQUEST_RESPONSE_SOCKET)
      • SERVER  — via pubsub TCP bridge (SERVER_COMMAND_SOCKET)
    """

    if machine == "PI":
        sock_path = command_socket_path(subsystem)

        # ------------------------------------------------------------
        # Minimal defensive check: missing subsystem socket
        # ------------------------------------------------------------
        if not os.path.exists(sock_path):
            return {
                "success": False,
                "message": "BAD",
                "payload": {
                    "error": "unknown subsystem"
                }
            }

    elif machine == "SERVER":
        sock_path = SERVER_COMMAND_SOCKET

    else:
        # TEENSY (and any future machine routed through pubsub RPC)
        sock_path = TEENSY_REQUEST_RESPONSE_SOCKET

    req: Dict[str, Any] = {
        "machine": machine,
        "subsystem": subsystem,
        "command": command,
    }

    # Diagnostic provenance for direct Pi command traffic.  Unknown/custom
    # clients omit this block, which is itself useful evidence when a caller
    # disconnects before consuming a response.
    if machine == "PI":
        req["_client"] = {
            "schema": "ZPNET_COMMAND_CLIENT_V1",
            "process": os.path.basename(sys.argv[0]) or "<unknown>",
            "pid": os.getpid(),
            "thread": threading.current_thread().name,
            "request_id": (
                f"{os.getpid()}:{threading.current_thread().name}:"
                f"{time.monotonic_ns()}"
            ),
            "sent_time_ns": time.time_ns(),
        }

    if args is not None:
        req["args"] = args

    raw = json.dumps(
        req,
        separators=(",", ":"),
        ensure_ascii=False,
    ).encode("utf-8")

    last_exc: Exception | None = None

    for attempt in range(1, retries + 1):
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                sock.connect(sock_path)
                sock.sendall(raw)
                sock.shutdown(socket.SHUT_WR)

                # The server closes this one-command connection after sending
                # exactly one JSON response. A Unix stream socket may split a
                # lawful response across multiple recv() calls, so EOF—not the
                # first recv() boundary—is the response frame delimiter.
                response_chunks: list[bytes] = []
                while True:
                    chunk = sock.recv(65536)
                    if not chunk:
                        break
                    response_chunks.append(chunk)

                if not response_chunks:
                    raise RuntimeError("empty response")

                resp_raw = b"".join(response_chunks)
                return json.loads(resp_raw.decode("utf-8"))

        except (
            FileNotFoundError,
            ConnectionRefusedError,
            ConnectionResetError,
            RuntimeError,
        ) as e:
            last_exc = e
            if attempt >= retries:
                break
            time.sleep(retry_delay_s)

    raise RuntimeError(
        f"[send_command] failed after {retries} attempts "
        f"({machine} {subsystem} {command})"
    ) from last_exc



# =============================================================================
# Public Pub/Sub API — publishing (UNCHANGED)
# =============================================================================

def publish(
    topic: str,
    payload: Payload,
) -> None:
    """
    Publish a message under a topic.

    Semantics:
      • Fire-and-forget
      • Best-effort
      • No retries
      • No return value
      • Silence on failure
    """

    msg = {
        "topic": topic,
        "payload": payload,
    }

    raw = json.dumps(
        msg,
        separators=(",", ":"),
        ensure_ascii=False,
    ).encode("utf-8")

    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.connect(TEENSY_PUBLISH_SUBSCRIBE_SOCKET)
            sock.sendall(raw)
            sock.shutdown(socket.SHUT_WR)
    except Exception:
        return


# =============================================================================
# Internal helpers
# =============================================================================

def _bind_unix_socket(path: str) -> socket.socket:
    if os.path.exists(path):
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as probe:
                probe.connect(path)
            raise RuntimeError(f"socket already active: {path}")
        except ConnectionRefusedError:
            os.unlink(path)

    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(path)
    srv.listen()
    return srv


# =============================================================================
# Command server (internal)
# =============================================================================

def _send_command_response(
    conn: socket.socket,
    *,
    subsystem: str,
    command: str,
    response: dict,
    client_meta: Optional[Dict[str, Any]] = None,
    handler_ms: Optional[float] = None,
    request_age_ms: Optional[float] = None,
) -> bool:
    raw = json.dumps(response, separators=(",", ":")).encode("utf-8")

    try:
        conn.sendall(raw)
        return True
    except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError) as exc:
        # A command client may time out, close its websocket, or otherwise
        # disappear while a slow handler is still producing its response.
        # That abandons this one response; it must never terminate the
        # subsystem's long-lived command-serving thread.
        meta = client_meta if isinstance(client_meta, dict) else {}
        canonical_client = meta.get("schema") == "ZPNET_COMMAND_CLIENT_V1"
        logging.warning(
            "[commands] %s client disconnected before response "
            "command=%s bytes=%d handler_ms=%s request_age_ms=%s "
            "canonical_client=%s client_process=%r client_pid=%r "
            "client_thread=%r request_id=%r error=%r",
            subsystem,
            command,
            len(raw),
            f"{handler_ms:.3f}" if handler_ms is not None else "unknown",
            f"{request_age_ms:.3f}" if request_age_ms is not None else "unknown",
            canonical_client,
            meta.get("process"),
            meta.get("pid"),
            meta.get("thread"),
            meta.get("request_id"),
            exc,
        )
        return False


def _serve_commands(
    *,
    subsystem: str,
    commands: Dict[str, Callable[[Optional[dict]], dict]],
) -> None:
    sock_path = command_socket_path(subsystem)

    logging.info("🚀 [commands] %s → %s", subsystem, sock_path)
    srv = _bind_unix_socket(sock_path)

    while True:
        conn, _ = srv.accept()
        request_started_ns = time.monotonic_ns()
        with conn:
            raw = conn.recv(65536)
            if not raw:
                logging.warning(
                    "[commands] %s empty request ignored",
                    subsystem,
                )
                continue

            try:
                req = json.loads(raw.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                logging.exception(
                    "[commands] %s malformed request ignored "
                    "(bytes=%d preview=%r)",
                    subsystem,
                    len(raw),
                    raw[:256],
                )
                continue

            cmd = req["command"]
            args = req.get("args")
            client_meta = req.get("_client")

            # ------------------------------------------------------------
            # Minimal defensive check for invalid command
            # ------------------------------------------------------------
            if cmd not in commands:
                resp = {
                    "success": False,
                    "message": "BAD",
                    "payload": {
                        "error": "unknown command"
                    }
                }
                request_age_ms = (time.monotonic_ns() - request_started_ns) / 1_000_000.0
                _send_command_response(
                    conn,
                    subsystem=subsystem,
                    command=cmd,
                    response=resp,
                    client_meta=client_meta,
                    handler_ms=0.0,
                    request_age_ms=request_age_ms,
                )
                continue

            handler = commands[cmd]
            handler_started_ns = time.monotonic_ns()
            resp = handler(args)
            handler_ms = (time.monotonic_ns() - handler_started_ns) / 1_000_000.0
            request_age_ms = (time.monotonic_ns() - request_started_ns) / 1_000_000.0

            _send_command_response(
                conn,
                subsystem=subsystem,
                command=cmd,
                response=resp,
                client_meta=client_meta,
                handler_ms=handler_ms,
                request_age_ms=request_age_ms,
            )



# =============================================================================
# Pub/Sub server (internal)
# =============================================================================

def _serve_pubsub(
    *,
    subsystem: str,
    publication_handler: Callable[[str, Any], None],
) -> None:
    sock_path = pubsub_socket_path(subsystem)
    srv = _bind_unix_socket(sock_path)

    while True:
        conn, _ = srv.accept()
        with conn:
            chunks: list[bytes] = []
            total_bytes = 0
            try:
                while True:
                    chunk = conn.recv(PUBSUB_RECV_CHUNK_BYTES)
                    if not chunk:
                        break
                    total_bytes += len(chunk)
                    if total_bytes > PUBSUB_MAX_MESSAGE_BYTES:
                        raise ValueError(
                            "pubsub publication exceeds "
                            f"{PUBSUB_MAX_MESSAGE_BYTES} bytes"
                        )
                    chunks.append(chunk)

                if not chunks:
                    continue

                raw = b"".join(chunks)
                msg = json.loads(raw.decode("utf-8"))
            except Exception:
                tail = b"" if not chunks else b"".join(chunks)[-80:]
                logging.exception(
                    "[pubsub] bad message ignored subsystem=%s bytes=%d "
                    "ends_with_brace=%s tail=%r",
                    subsystem,
                    total_bytes,
                    tail.rstrip().endswith(b"}"),
                    tail,
                )
                continue

            topic = msg.get("topic")
            payload = msg.get("payload")
            logging.debug(
                "[pubsub] recv subsystem=%s topic=%r bytes=%d chunks=%d",
                subsystem,
                topic,
                total_bytes,
                len(chunks),
            )

            if not isinstance(topic, str) or not topic:
                logging.warning("[pubsub] %s ignoring malformed topic=%r", subsystem, topic)
                continue

            try:
                publication_handler(topic, payload)
            except Exception:
                logging.exception("[pubsub] publication handler failed (%s:%s)", subsystem, topic)

# =============================================================================
# Public declarative API
# =============================================================================

def server_setup(
    *,
    subsystem: str,
    commands: Dict[str, Callable[[Optional[dict]], dict]] | None = None,
    publication_handler: Callable[[str, Any], None] | None = None,
    blocking: bool = True,
) -> None:
    """
    Declaratively start a ZPNet Pi-side process.

    The caller declares:
      • subsystem name
      • command handlers
      • optionally, one publication-ingress callback

    PUBSUB owns formal topic routing.  This runtime owns only the target socket
    and invokes the callback for publications already routed to this subsystem.

    If blocking=True (default), this function never returns.
    If blocking=False, this function returns after launching server
    threads, allowing the caller to do further initialization (e.g.
    recovery) before entering its own main loop.
    """

    logging.info("🚀 [process] starting subsystem: %s", subsystem)

    # Do not mutate the caller's command table.  There is deliberately no
    # implicit SUBSCRIPTIONS command: process lifetime does not author topology.
    commands = dict(commands or {})

    # -----------------------------------------------------------------
    # Command plane
    # -----------------------------------------------------------------

    threading.Thread(
        target=_serve_commands,
        kwargs={
            "subsystem": subsystem,
            "commands": commands,
        },
        daemon=True,
        name=f"{subsystem}-commands",
    ).start()

    # -----------------------------------------------------------------
    # Pub/Sub plane
    # -----------------------------------------------------------------

    if publication_handler is not None:
        threading.Thread(
            target=_serve_pubsub,
            kwargs={
                "subsystem": subsystem,
                "publication_handler": publication_handler,
            },
            daemon=True,
            name=f"{subsystem}-pubsub",
        ).start()

    # -----------------------------------------------------------------
    # Process lifetime
    # -----------------------------------------------------------------

    if blocking:
        while True:
            time.sleep(3600)