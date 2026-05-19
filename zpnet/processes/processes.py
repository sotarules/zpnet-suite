"""
ZPNet Process Runtime (Pi-Side)

Provides a unified, declarative runtime for Pi-side processes.

A process declares:
  • its subsystem name
  • the commands it serves
  • the topics it subscribes to (topic → handler)
  • no routing logic whatsoever

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
import stat
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
# External-boundary timeouts / sizing
# =============================================================================
#
# A Unix socket file is not proof of a healthy peer. Test utilities and stale
# processes can leave sockets behind; a peer may also accept and never answer.
# Commands remain synchronous, but every external socket operation is bounded.
# PUBSUB may pass a much shorter timeout for cheap SUBSCRIPTIONS discovery.

COMMAND_SOCKET_TIMEOUT_S = 30.0
COMMAND_SERVER_CLIENT_TIMEOUT_S = 30.0
PUBLISH_SOCKET_TIMEOUT_S = 1.0
BIND_PROBE_TIMEOUT_S = 0.25
COMMAND_RECV_CHUNK = 65536
SLOW_COMMAND_LOG_MS = 500

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

    Socket files are filtered to real Unix sockets. This does not prove the
    owning process is healthy — send_command() owns that boundary — but it does
    keep unrelated files from entering process discovery.
    """
    subsystems: set[str] = set()

    try:
        filenames = os.listdir(SOCKET_DIR)
    except FileNotFoundError:
        return []

    for fname in filenames:
        if not fname.startswith("zpnet-"):
            continue
        if not fname.endswith("-command.sock"):
            continue

        path = os.path.join(SOCKET_DIR, fname)
        try:
            if not stat.S_ISSOCK(os.stat(path).st_mode):
                continue
        except FileNotFoundError:
            continue

        subsystem_fs = fname[len("zpnet-"):-len("-command.sock")]
        subsystems.add(subsystem_fs.upper())

    return sorted(subsystems)


def _recv_response_bytes(sock: socket.socket) -> bytes:
    """
    Read a command response from a synchronous command socket.

    The protocol sends one compact JSON response. Most replies fit in a single
    read; the timeout on the socket bounds any stale peer that accepts but does
    not close or complete the response.
    """
    chunks: list[bytes] = []

    while True:
        chunk = sock.recv(COMMAND_RECV_CHUNK)
        if not chunk:
            break
        chunks.append(chunk)
        if len(chunk) < COMMAND_RECV_CHUNK:
            break

    return b"".join(chunks)


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
    timeout_s: float = COMMAND_SOCKET_TIMEOUT_S,
) -> Dict[str, Any]:
    """
    Send a single command and return a single response.

    Semantics:
      • JSON-in / JSON-out
      • Bounded retries at unowned transport boundary
      • Bounded connect/send/receive wait via timeout_s
      • Any persistent failure propagates

    Machine routing:
      • PI      — direct to subsystem's Unix command socket
      • TEENSY  — via pubsub RPC broker (TEENSY_REQUEST_RESPONSE_SOCKET)
      • SERVER  — via pubsub TCP bridge (SERVER_COMMAND_SOCKET)
    """

    if machine == "PI":
        sock_path = command_socket_path(subsystem)

        # ------------------------------------------------------------
        # Minimal boundary check: missing subsystem socket
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

    if args is not None:
        req["args"] = args

    raw = json.dumps(
        req,
        separators=(",", ":"),
        ensure_ascii=False,
    ).encode("utf-8")

    last_exc: Exception | None = None

    for attempt in range(1, retries + 1):
        attempt_started = time.monotonic()

        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                sock.settimeout(timeout_s)
                sock.connect(sock_path)
                sock.sendall(raw)
                sock.shutdown(socket.SHUT_WR)

                resp_raw = _recv_response_bytes(sock)
                if not resp_raw:
                    raise RuntimeError("empty response")

                elapsed_ms = int((time.monotonic() - attempt_started) * 1000)
                if elapsed_ms >= SLOW_COMMAND_LOG_MS:
                    logging.info(
                        "🐢 [send_command] slow response machine=%s subsystem=%s "
                        "command=%s attempt=%d/%d elapsed_ms=%d timeout_s=%.3f bytes=%d",
                        machine,
                        subsystem,
                        command,
                        attempt,
                        retries,
                        elapsed_ms,
                        timeout_s,
                        len(resp_raw),
                    )

                return json.loads(resp_raw.decode("utf-8"))

        except (
            FileNotFoundError,
            ConnectionRefusedError,
            ConnectionResetError,
            BrokenPipeError,
            RuntimeError,
            socket.timeout,
            TimeoutError,
            json.JSONDecodeError,
        ) as e:
            last_exc = e
            elapsed_ms = int((time.monotonic() - attempt_started) * 1000)
            logging.warning(
                "⚠️ [send_command] failed attempt=%d/%d machine=%s subsystem=%s "
                "command=%s elapsed_ms=%d timeout_s=%.3f error=%r",
                attempt,
                retries,
                machine,
                subsystem,
                command,
                elapsed_ms,
                timeout_s,
                e,
            )

            if attempt >= retries:
                break
            time.sleep(retry_delay_s)

    raise RuntimeError(
        f"[send_command] failed after {retries} attempts "
        f"({machine} {subsystem} {command}) "
        f"timeout_s={timeout_s} last_error={last_exc!r}"
    ) from last_exc


# =============================================================================
# Public Pub/Sub API — publishing
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
            sock.settimeout(PUBLISH_SOCKET_TIMEOUT_S)
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
                probe.settimeout(BIND_PROBE_TIMEOUT_S)
                probe.connect(path)
            raise RuntimeError(f"socket already active: {path}")
        except ConnectionRefusedError:
            os.unlink(path)
        except FileNotFoundError:
            # A racing cleanup removed it between exists() and connect().
            pass

    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(path)
    srv.listen()
    return srv


def _send_json(conn: socket.socket, payload: Dict[str, Any]) -> None:
    conn.sendall(json.dumps(payload, separators=(",", ":")).encode("utf-8"))


# =============================================================================
# Command server (internal)
# =============================================================================

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
        with conn:
            try:
                conn.settimeout(COMMAND_SERVER_CLIENT_TIMEOUT_S)
                raw = conn.recv(COMMAND_RECV_CHUNK)
                if not raw:
                    continue

                req = json.loads(raw.decode("utf-8"))
                cmd = req.get("command")
                args = req.get("args")

                # ------------------------------------------------------------
                # Minimal boundary check for invalid command
                # ------------------------------------------------------------
                if cmd not in commands:
                    _send_json(conn, {
                        "success": False,
                        "message": "BAD",
                        "payload": {
                            "error": "unknown command"
                        }
                    })
                    continue

                handler = commands[cmd]
                handler_started = time.monotonic()

                try:
                    resp = handler(args)
                except Exception as e:
                    logging.exception(
                        "💥 [commands] handler failed subsystem=%s command=%s",
                        subsystem,
                        cmd,
                    )
                    resp = {
                        "success": False,
                        "message": "BAD",
                        "payload": {
                            "error": "handler exception",
                            "detail": str(e),
                        },
                    }

                elapsed_ms = int((time.monotonic() - handler_started) * 1000)
                if elapsed_ms >= SLOW_COMMAND_LOG_MS:
                    logging.info(
                        "🐢 [commands] slow handler subsystem=%s command=%s elapsed_ms=%d",
                        subsystem,
                        cmd,
                        elapsed_ms,
                    )

                _send_json(conn, resp)

            except (socket.timeout, TimeoutError):
                logging.warning(
                    "⏱️ [commands] request timeout subsystem=%s socket=%s",
                    subsystem,
                    sock_path,
                )
                continue

            except Exception:
                # A malformed request or broken client must not kill the
                # command server thread. This is an external boundary.
                logging.exception(
                    "💥 [commands] bad request ignored subsystem=%s socket=%s",
                    subsystem,
                    sock_path,
                )
                continue


# =============================================================================
# Pub/Sub server (internal)
# =============================================================================

def _serve_pubsub(*, subsystem: str, subscriptions: Dict[str, Callable[[dict], None]]) -> None:
    sock_path = pubsub_socket_path(subsystem)
    srv = _bind_unix_socket(sock_path)

    while True:
        conn, _ = srv.accept()
        with conn:
            try:
                conn.settimeout(COMMAND_SERVER_CLIENT_TIMEOUT_S)
                raw = conn.recv(COMMAND_RECV_CHUNK)
                if not raw:
                    continue
                msg = json.loads(raw.decode("utf-8"))
            except (socket.timeout, TimeoutError):
                logging.warning("[pubsub] recv timeout (ignored) %s", subsystem)
                continue
            except Exception:
                logging.exception("[pubsub] bad message (ignored) %s", subsystem)
                continue

            topic = msg.get("topic")
            payload = msg.get("payload")

            handler = subscriptions.get(topic)
            if handler is None:
                # IMPORTANT: do NOT kill the server thread
                logging.warning("[pubsub] %s ignoring unknown topic=%r", subsystem, topic)
                continue

            try:
                handler(payload)
            except Exception:
                logging.exception("[pubsub] handler failed (%s:%s)", subsystem, topic)

# =============================================================================
# Public declarative API
# =============================================================================

def server_setup(
    *,
    subsystem: str,
    commands: Dict[str, Callable[[Optional[dict]], dict]] | None = None,
    subscriptions: Dict[str, Callable[[dict], None]] | None = None,
    blocking: bool = True,
) -> None:
    """
    Declaratively start a ZPNet Pi-side process.

    The caller declares:
      • subsystem name
      • command handlers
      • pub/sub topic → handler mapping

    All sockets, threads, and routing are handled internally.

    If blocking=True (default), this function never returns.
    If blocking=False, this function returns after launching server
    threads, allowing the caller to do further initialization (e.g.
    recovery) before entering its own main loop.
    """

    logging.info("🚀 [process] starting subsystem: %s", subsystem)

    commands = commands or {}
    subscriptions = subscriptions or {}

    # -----------------------------------------------------------------
    # Inject implicit SUBSCRIPTIONS command
    # -----------------------------------------------------------------

    def _cmd_subscriptions(_: Optional[dict]) -> dict:
        return {
            "success": True,
            "message": "OK",
            "payload": {
                "machine": "PI",
                "subsystem": subsystem,
                "subscriptions": [
                    { "name": topic }
                    for topic in sorted(subscriptions.keys())
                ],
            },
        }

    # Do not mutate caller's command table
    effective_commands = dict(commands)
    effective_commands["SUBSCRIPTIONS"] = _cmd_subscriptions

    # -----------------------------------------------------------------
    # Command plane
    # -----------------------------------------------------------------

    threading.Thread(
        target=_serve_commands,
        kwargs={
            "subsystem": subsystem,
            "commands": effective_commands,
        },
        daemon=True,
        name=f"{subsystem}-commands",
    ).start()

    # -----------------------------------------------------------------
    # Pub/Sub plane
    # -----------------------------------------------------------------

    threading.Thread(
        target=_serve_pubsub,
        kwargs={
            "subsystem": subsystem,
            "subscriptions": subscriptions,
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
