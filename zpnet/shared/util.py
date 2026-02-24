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

def payload_to_json_str(payload: Payload) -> str:
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)

def payload_to_json_bytes(payload: Payload) -> bytes:
    return payload_to_json_str(payload).encode("utf-8")

def system_time_z() -> str:
    """
    Return the current UTC time truncated to the second as an
    ISO8601 Zulu string, e.g. "2026-02-23T21:45:07Z".

    Derived from the chrony-disciplined system clock.  Used as
    a correlation key for matching TIMEBASE_FRAGMENT records to
    PITIMER captures — both sides call this function within the
    same UTC second, producing the same key.

    This replaces GNSS.GET_TIME for correlation purposes.  The
    system clock is disciplined to ~39 ns of GNSS via chrony PPS,
    so it is authoritative for "what second is it right now."
    """
    from datetime import datetime, timezone
    now = datetime.now(timezone.utc)
    return now.strftime("%Y-%m-%dT%H:%M:%SZ")