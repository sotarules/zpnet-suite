import json
from datetime import datetime
from typing import Any

from zpnet.shared.constants import Payload


def normalize_payload(payload: Any) -> dict:
    """
    Ensure payload is always a dict.

    Accepts:
      • dict  → returned as-is
      • str   → json.loads
      • None  → {}

    Raises:
      • ValueError on malformed JSON
    """
    if payload is None:
        return {}
    if isinstance(payload, dict):
        return payload
    if isinstance(payload, (str, bytes, bytearray)):
        return json.loads(payload)
    raise TypeError(f"Unsupported payload type: {type(payload)}")

def normalize_ts(ts: Any) -> datetime:

    """
    Ensure timestamp is a timezone-aware datetime.
    """
    if isinstance(ts, datetime):
        return ts
    if isinstance(ts, str):
        return datetime.fromisoformat(ts.replace("Z", "+00:00"))
    raise TypeError(f"Unsupported ts type: {type(ts)}")

def payload_to_json_bytes(payload: Payload) -> bytes:
    return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")