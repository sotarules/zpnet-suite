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

Author: The Mule + GPT
"""

from __future__ import annotations

import json
import logging
import os
import socket
import threading
import time
from typing import Any, Dict, Callable, Optional, Iterable

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
# Canonical RPC client (UNCHANGED)
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
    """

    if machine == "PI":
        sock_path = command_socket_path(subsystem)
    else:
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
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                sock.connect(sock_path)
                sock.sendall(raw)
                sock.shutdown(socket.SHUT_WR)

                resp_raw = sock.recv(65536)
                if not resp_raw:
                    raise RuntimeError("empty response")

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
        # Publishing failures are intentionally silent
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
            raw = conn.recv(65536)
            req = json.loads(raw.decode("utf-8"))

            cmd = req["command"]
            args = req.get("args")

            handler = commands[cmd]
            resp = handler(args)

            conn.sendall(
                json.dumps(resp, separators=(",", ":")).encode("utf-8")
            )


# =============================================================================
# Pub/Sub server (internal)
# =============================================================================

def _serve_pubsub(
    *,
    subsystem: str,
    subscriptions: Dict[str, Callable[[dict], None]],
) -> None:
    """
    Receive published messages and dispatch to per-topic handlers.

    Semantics:
      • One socket
      • One handler per topic
      • Exact topic match
      • Best-effort
      • No retries
      • No fan-in callback
    """

    sock_path = pubsub_socket_path(subsystem)

    srv = _bind_unix_socket(sock_path)

    while True:
        conn, _ = srv.accept()
        with conn:
            raw = conn.recv(65536)
            msg = json.loads(raw.decode("utf-8"))

            topic = msg.get("topic")
            payload = msg.get("payload")

            handler = subscriptions.get(topic)
            if handler is None:
                return

            try:
                handler(payload)
            except Exception:
                logging.exception(
                    "[pubsub] handler failed (%s:%s)",
                    subsystem,
                    topic,
                )

# =============================================================================
# Announce subscriptions (internal)
# =============================================================================

def _announce_subscriptions(
    *,
    subsystem: str,
    subscriptions: Dict[str, Callable[[dict], None]],
) -> None:
    topics = sorted(subscriptions.keys())

    if not topics:
        logging.info(
            "🚀 [pubsub] %s has no subscriptions to announce",
            subsystem,
        )
        return

    payload: Payload = {
        "subsystem": subsystem,
        "topics": topics,
    }

    logging.info(
        "🚀 [pubsub] announcing SUBSCRIBE %s → %s",
        subsystem,
        topics,
    )

    publish("SUBSCRIBE", payload)

# =============================================================================
# Public declarative API
# =============================================================================

def server_setup(
    *,
    subsystem: str,
    commands: Dict[str, Callable[[Optional[dict]], dict]],
    subscriptions: Dict[str, Callable[[dict], None]],
) -> None:
    """
    Declaratively start a ZPNet Pi-side process.

    This function never returns.

    The caller declares:
      • subsystem name
      • command handlers
      • pub/sub topic → handler mapping

    All sockets, threads, and routing are handled internally.
    """

    logging.info("🚀 [process] starting subsystem: %s", subsystem)

    # Command plane
    threading.Thread(
        target=_serve_commands,
        kwargs={
            "subsystem": subsystem,
            "commands": commands,
        },
        daemon=True,
        name=f"{subsystem}-commands",
    ).start()

    # Pub/Sub plane
    threading.Thread(
        target=_serve_pubsub,
        kwargs={
            "subsystem": subsystem,
            "subscriptions": subscriptions,
        },
        daemon=True,
        name=f"{subsystem}-pubsub",
    ).start()

    _announce_subscriptions(
        subsystem=subsystem,
        subscriptions=subscriptions,
    )

    # Block forever (process lifetime)
    while True:
        time.sleep(3600)
