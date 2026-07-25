"""
Shared socket I/O helpers for one-message stream protocols.

Protocol contract:
  • One logical message per connection
  • Sender writes the complete message, then shuts down its write side
  • Receiver reads until EOF
  • recv() boundaries are never treated as message boundaries
"""

from __future__ import annotations

import json
import socket
from typing import Any, Mapping


DEFAULT_CHUNK_SIZE = 65536
DEFAULT_MAX_MESSAGE_BYTES = 1024 * 1024


class SocketMessageTooLargeError(ValueError):
    """Raised when a received stream message exceeds its configured limit."""


def recv_until_eof(
    sock: socket.socket,
    *,
    chunk_size: int = DEFAULT_CHUNK_SIZE,
    max_bytes: int | None = DEFAULT_MAX_MESSAGE_BYTES,
) -> bytes:
    """
    Read one complete stream message, using EOF as the frame delimiter.

    This helper is for one-message-per-connection protocols whose sender calls
    shutdown(socket.SHUT_WR) or closes the connection after sending.

    Args:
        sock:
            Connected stream socket.
        chunk_size:
            Maximum bytes requested from each recv() call.
        max_bytes:
            Maximum accepted total message size. Use None for no limit.

    Returns:
        Complete message bytes. An immediate EOF returns b"".

    Raises:
        ValueError:
            If chunk_size or max_bytes is invalid.
        SocketMessageTooLargeError:
            If the accumulated message exceeds max_bytes.
        OSError:
            Propagated from socket.recv().
    """
    if chunk_size <= 0:
        raise ValueError("chunk_size must be greater than zero")
    if max_bytes is not None and max_bytes < 0:
        raise ValueError("max_bytes must be non-negative or None")

    chunks: list[bytes] = []
    total_bytes = 0

    while True:
        chunk = sock.recv(chunk_size)
        if not chunk:
            break

        total_bytes += len(chunk)
        if max_bytes is not None and total_bytes > max_bytes:
            raise SocketMessageTooLargeError(
                f"socket message exceeds maximum size: "
                f"{total_bytes} > {max_bytes} bytes"
            )

        chunks.append(chunk)

    return b"".join(chunks)


def recv_json_until_eof(
    sock: socket.socket,
    *,
    chunk_size: int = DEFAULT_CHUNK_SIZE,
    max_bytes: int | None = DEFAULT_MAX_MESSAGE_BYTES,
    require_object: bool = True,
) -> Any:
    """
    Receive one complete UTF-8 JSON document, framed by stream EOF.

    Args:
        sock:
            Connected stream socket.
        chunk_size:
            Maximum bytes requested from each recv() call.
        max_bytes:
            Maximum accepted total encoded message size.
        require_object:
            When True, require the decoded JSON value to be a dictionary.

    Returns:
        Decoded JSON value.

    Raises:
        RuntimeError:
            If the peer sends no message bytes.
        UnicodeDecodeError:
            If the message is not valid UTF-8.
        json.JSONDecodeError:
            If the message is not valid JSON.
        TypeError:
            If require_object is True and the JSON value is not an object.
        SocketMessageTooLargeError:
            If the encoded message exceeds max_bytes.
        OSError:
            Propagated from socket.recv().
    """
    raw = recv_until_eof(
        sock,
        chunk_size=chunk_size,
        max_bytes=max_bytes,
    )

    if not raw:
        raise RuntimeError("empty socket message")

    value = json.loads(raw.decode("utf-8"))

    if require_object and not isinstance(value, dict):
        raise TypeError(
            f"expected JSON object, got {type(value).__name__}"
        )

    return value


def send_bytes_and_shutdown(
    sock: socket.socket,
    data: bytes,
) -> None:
    """
    Send one complete message and close only the socket's write direction.

    The peer may continue sending its response on the same connection.
    """
    sock.sendall(data)
    sock.shutdown(socket.SHUT_WR)


def send_json_and_shutdown(
    sock: socket.socket,
    payload: Mapping[str, Any],
    *,
    ensure_ascii: bool = False,
) -> None:
    """
    Serialize one compact JSON object, send it, and shut down the write side.
    """
    raw = json.dumps(
        payload,
        separators=(",", ":"),
        ensure_ascii=ensure_ascii,
    ).encode("utf-8")

    send_bytes_and_shutdown(sock, raw)
