"""
ZPNet Shared HTTP Utilities — Gzip Transport Helpers

Centralizes HTTP request-body preparation for ZPNet.

Design principles:
  • Explicit compression (no implicit library behavior)
  • Deterministic headers
  • JSON-first semantics
  • No retry or transport policy here (belongs to callers)

This module does NOT:
  • Perform HTTP requests
  • Implement retries or backoff
  • Handle responses

Author: The Mule + GPT
"""

import json
import gzip
from typing import Tuple, Dict, Any


# ---------------------------------------------------------------------
# Gzip helpers
# ---------------------------------------------------------------------

def gzip_bytes(data: bytes) -> bytes:
    """
    Gzip-compress raw bytes.

    Args:
        data (bytes): Raw payload

    Returns:
        bytes: Gzipped payload
    """
    if not isinstance(data, (bytes, bytearray)):
        raise TypeError("gzip_bytes expects bytes")

    return gzip.compress(data)


def gzip_json(
    payload: Any,
    *,
    content_type: str = "application/json"
) -> Tuple[bytes, Dict[str, str]]:
    """
    Serialize a payload to JSON, gzip it, and return payload + headers.

    Args:
        payload (Any): JSON-serializable object
        content_type (str): Content-Type header value

    Returns:
        (gzipped_bytes, headers)

    Raises:
        TypeError: if payload is not JSON-serializable
    """
    try:
        json_bytes = json.dumps(
            payload,
            separators=(",", ":"),
            ensure_ascii=False
        ).encode("utf-8")
    except Exception as e:
        raise TypeError(f"gzip_json: JSON serialization failed: {e}")

    gzipped = gzip.compress(json_bytes)

    headers = {
        "Content-Type": content_type,
        "Content-Encoding": "gzip",
        "Content-Length": str(len(gzipped)),
        "Connection": "close",
    }

    return gzipped, headers


def gzip_text(
    text: str,
    *,
    content_type: str = "text/plain"
) -> Tuple[bytes, Dict[str, str]]:
    """
    Gzip-compress a UTF-8 text payload.

    Args:
        text (str): Text payload
        content_type (str): Content-Type header value

    Returns:
        (gzipped_bytes, headers)
    """
    if not isinstance(text, str):
        raise TypeError("gzip_text expects str")

    raw = text.encode("utf-8")
    gzipped = gzip.compress(raw)

    headers = {
        "Content-Type": content_type,
        "Content-Encoding": "gzip",
        "Content-Length": str(len(gzipped)),
        "Connection": "close",
    }

    return gzipped, headers
