import json
from datetime import datetime
from typing import Any, Iterable, Mapping

from zpnet.shared.constants import Payload


FEATURE_STATUS_INITIALIZING = "INITIALIZING"
FEATURE_STATUS_NOMINAL = "NOMINAL"
FEATURE_STATUS_HOLD = "HOLD"
FEATURE_STATUS_ANOMALY = "ANOMALY"

FEATURE_STATUSES = {
    FEATURE_STATUS_INITIALIZING,
    FEATURE_STATUS_NOMINAL,
    FEATURE_STATUS_HOLD,
    FEATURE_STATUS_ANOMALY,
}


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


# -----------------------------------------------------------------------------
# Feature-state helpers
# -----------------------------------------------------------------------------

def normalize_feature_status(status: Any, *, default: str = FEATURE_STATUS_INITIALIZING) -> str:
    """Normalize a feature status into the shared closed vocabulary."""
    s = str(status or default).strip().upper()
    if s == "DOWN":
        return FEATURE_STATUS_ANOMALY
    return s if s in FEATURE_STATUSES else default


def feature_path_parts(name: str) -> tuple[str, str, str]:
    """Split MACHINE.SUBSYSTEM.FEATURE into normalized path parts."""
    parts = [p.strip().upper() for p in str(name or "").split(".") if p.strip()]
    if len(parts) != 3:
        raise ValueError(f"feature name must be MACHINE.SUBSYSTEM.FEATURE, got {name!r}")
    return parts[0], parts[1], parts[2]


def feature_tree(payload_or_tree: Any) -> dict:
    """
    Return the hierarchical feature tree from either SYSTEM.REPORT or a raw
    features payload.
    """
    payload = normalize_payload(payload_or_tree)
    features = payload.get("features")
    return features if isinstance(features, dict) else payload


def feature_get(payload_or_tree: Any, name: str, default: Any = None) -> Any:
    """Return a feature leaf by MACHINE.SUBSYSTEM.FEATURE path.

    Feature state is intentionally scalar-first:

      {"TEENSY": {"CLOCKS": {"SMARTZERO": "NOMINAL"}}}

    Older/richer payloads may still use a mapping leaf such as
    {"status": "NOMINAL", "detail": "..."}.  Return either shape
    unchanged so callers can normalize the status while preserving backward
    compatibility.
    """
    machine, subsystem, feature = feature_path_parts(name)
    tree = feature_tree(payload_or_tree)

    node = tree.get(machine)
    if not isinstance(node, Mapping):
        return default

    node = node.get(subsystem)
    if not isinstance(node, Mapping):
        return default

    return node.get(feature, default)


def feature_status(
    payload_or_tree: Any,
    name: str,
    default: str = FEATURE_STATUS_HOLD,
) -> str:
    """Return a feature's status; missing features are HOLD by default.

    Accept both the current scalar leaf form:

      "FLOORLINE": "NOMINAL"

    and the legacy/detail-capable mapping form:

      "FLOORLINE": {"status": "NOMINAL", "detail": "..."}
    """
    entry = feature_get(payload_or_tree, name)
    if isinstance(entry, Mapping):
        return normalize_feature_status(entry.get("status"), default=default)
    if entry is None:
        return normalize_feature_status(default, default=FEATURE_STATUS_HOLD)
    return normalize_feature_status(entry, default=default)


def feature_detail(payload_or_tree: Any, name: str, default: str = "") -> str:
    """Return a feature's detail string, if present.

    Scalar feature leaves intentionally carry no detail; focused subsystem
    reports remain the place for diagnostics.
    """
    entry = feature_get(payload_or_tree, name)
    if not isinstance(entry, Mapping):
        return default

    detail = entry.get("detail")
    return str(detail) if detail is not None else default


def feature_is_nominal(payload_or_tree: Any, name: str) -> bool:
    """Return True when the named feature is present and NOMINAL."""
    return feature_status(payload_or_tree, name) == FEATURE_STATUS_NOMINAL


def blocking_features(
    payload_or_tree: Any,
    names: Iterable[str],
) -> list[dict[str, str]]:
    """Return the subset of required features that are not NOMINAL."""
    blocked: list[dict[str, str]] = []

    for name in names:
        status = feature_status(payload_or_tree, name)
        if status == FEATURE_STATUS_NOMINAL:
            continue

        blocked.append({
            "name": name,
            "status": status,
            "detail": feature_detail(payload_or_tree, name),
        })

    return blocked


def features_all_nominal(
    payload_or_tree: Any,
    names: Iterable[str],
) -> bool:
    """Return True when every named feature is NOMINAL."""
    return not blocking_features(payload_or_tree, names)


def feature_summary(payload_or_tree: Any) -> dict:
    """Return a compact rollup over a MACHINE.SUBSYSTEM.FEATURE tree."""
    tree = feature_tree(payload_or_tree)

    counts = {
        FEATURE_STATUS_NOMINAL: 0,
        FEATURE_STATUS_INITIALIZING: 0,
        FEATURE_STATUS_HOLD: 0,
        FEATURE_STATUS_ANOMALY: 0,
    }
    blockers: list[str] = []

    for machine, subsystems in tree.items():
        if not isinstance(subsystems, Mapping):
            continue

        for subsystem, features in subsystems.items():
            if not isinstance(features, Mapping):
                continue

            for feature, entry in features.items():
                if isinstance(entry, Mapping):
                    status = normalize_feature_status(entry.get("status"))
                else:
                    status = normalize_feature_status(entry)
                counts[status] += 1

                if status != FEATURE_STATUS_NOMINAL:
                    blockers.append(f"{machine}.{subsystem}.{feature}")

    if counts[FEATURE_STATUS_ANOMALY]:
        rollup = FEATURE_STATUS_ANOMALY
    elif counts[FEATURE_STATUS_HOLD]:
        rollup = FEATURE_STATUS_HOLD
    elif counts[FEATURE_STATUS_INITIALIZING]:
        rollup = FEATURE_STATUS_INITIALIZING
    else:
        rollup = FEATURE_STATUS_NOMINAL

    return {
        "status": rollup,
        "counts": counts,
        "blocking_features": blockers,
    }
