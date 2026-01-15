"""
ZPNet Aggregator — Unified Truth Assembly Layer (PostgreSQL Edition)

Pure reducer: immutable event history → current system truth.

Invariants:
  • Events are append-only
  • Aggregates are last-write-wins snapshots
  • Payloads are normalized to dict at ingress
  • Timestamps are normalized to datetime

Author: The Mule + GPT
"""

import json
from datetime import datetime, timedelta, timezone
from typing import Any

from zpnet.shared.db import open_db
from zpnet.shared.logger import setup_logging
from zpnet.shared.teensy import photodiode_status

# ---------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------
BATTERY_LABEL = "battery"
BATTERY_CAPACITY_WH = 650

POWER_SAMPLE_STEP = 50
MAX_ERRORS_AGGREGATED = 5
ERROR_WINDOW_SEC = 3600


# ---------------------------------------------------------------------
# Normalization helpers (THIS IS THE KEY)
# ---------------------------------------------------------------------
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


# ---------------------------------------------------------------------
# Health classification
# ---------------------------------------------------------------------
def classify_health(states: list[str]) -> str:
    if not states:
        return "DOWN"
    if all(s == "NOMINAL" for s in states):
        return "NOMINAL"
    if any(s == "DOWN" for s in states):
        return "DOWN"
    return "HOLD"


# ---------------------------------------------------------------------
# Aggregate upsert
# ---------------------------------------------------------------------
def create_or_update_aggregate(aggregate_type: str, payload: dict) -> None:
    ts = datetime.now(timezone.utc)
    payload_json = json.dumps(payload, separators=(",", ":"))

    with open_db() as conn:
        cur = conn.cursor()
        cur.execute(
            """
            INSERT INTO aggregates (aggregate_type, timestamp, payload)
            VALUES (%s, %s, %s)
            ON CONFLICT (aggregate_type)
            DO UPDATE SET
                timestamp = EXCLUDED.timestamp,
                payload   = EXCLUDED.payload
            """,
            (aggregate_type, ts, payload_json),
        )


# ---------------------------------------------------------------------
# Event access helpers
# ---------------------------------------------------------------------
def latest_event(event_type: str) -> dict | None:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT ts, payload
            FROM zpnet_events
            WHERE event_type = %s
            ORDER BY ts DESC
            LIMIT 1
            """,
            (event_type,),
        )
        row = cur.fetchone()

    if not row:
        return None

    return {
        "ts": normalize_ts(row["ts"]),
        "payload": normalize_payload(row["payload"]),
    }


# ---------------------------------------------------------------------
# Battery State of Charge (PostgreSQL)
# ---------------------------------------------------------------------
def aggregate_battery_state_of_charge():
    # --------------------------------------------------------------
    # Find most recent battery swap
    # --------------------------------------------------------------
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT ts
            FROM zpnet_events
            WHERE event_type = 'SWAP_BATTERY'
            ORDER BY ts DESC
            LIMIT 1
            """
        )
        row = cur.fetchone()

    if not row:
        # No swap yet — cannot estimate SoC
        return

    swap_point = normalize_ts(row["ts"])

    # --------------------------------------------------------------
    # Fetch POWER_STATUS samples since swap
    # --------------------------------------------------------------
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT ts, payload
            FROM zpnet_events
            WHERE event_type = 'POWER_STATUS'
              AND ts >= %s
            ORDER BY ts ASC
            """,
            (swap_point,),
        )
        rows = cur.fetchall()

    if len(rows) < 2:
        # Not enough data yet
        return

    total_wh = 0.0
    last_ts = None
    last_power = None
    samples_used = 0

    # --------------------------------------------------------------
    # Integrate battery power over time
    # --------------------------------------------------------------
    for i in range(0, len(rows), POWER_SAMPLE_STEP):
        row = rows[i]

        ts = normalize_ts(row["ts"])
        payload = normalize_payload(row["payload"])

        sensors = payload.get("sensors", [])
        battery = next(
            (
                s for s in sensors
                if s.get("label", "").lower() == BATTERY_LABEL
            ),
            None,
        )

        if not battery:
            continue

        power_w = battery.get("power_w")
        if power_w is None:
            continue

        if last_ts is not None and last_power is not None:
            dt = (ts - last_ts).total_seconds()
            avg_power = (power_w + last_power) / 2.0
            total_wh += abs(avg_power * dt / 3600.0)
            samples_used += 1

        last_ts = ts
        last_power = power_w

    # --------------------------------------------------------------
    # Compute remaining capacity
    # --------------------------------------------------------------
    remaining_wh = max(0.0, BATTERY_CAPACITY_WH - total_wh)
    remaining_pct = round(
        100.0 * remaining_wh / BATTERY_CAPACITY_WH, 1
    )

    if last_power and last_power > 1.0:
        tte_minutes = round(remaining_wh / last_power * 60.0, 1)
    else:
        tte_minutes = float("inf")

    # --------------------------------------------------------------
    # Health classification
    # --------------------------------------------------------------
    if remaining_pct <= 3:
        health_state = "DOWN"
    elif remaining_pct <= 10:
        health_state = "HOLD"
    else:
        health_state = "NOMINAL"

    # --------------------------------------------------------------
    # Emit aggregate
    # --------------------------------------------------------------
    create_or_update_aggregate(
        "BATTERY_STATE_OF_CHARGE",
        {
            "remaining_pct": remaining_pct,
            "tte_minutes": tte_minutes,
            "wh_used_since_recharge": round(total_wh, 2),
            "wh_remaining_estimate": round(remaining_wh, 2),
            "samples_used": samples_used,
            "sample_step": POWER_SAMPLE_STEP,
            "battery_capacity_wh": BATTERY_CAPACITY_WH,
            "health_state": health_state,
        },
    )


# ---------------------------------------------------------------------
# Raspberry Pi Status (PostgreSQL)
# ---------------------------------------------------------------------
def aggregate_raspberry_pi_status():
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT payload
            FROM zpnet_events
            WHERE event_type = 'RASPBERRY_PI_STATUS'
            ORDER BY ts DESC
            LIMIT 1
            """
        )
        row = cur.fetchone()

    if not row:
        return

    payload = normalize_payload(row["payload"])
    create_or_update_aggregate("RASPBERRY_PI_STATUS", payload)


# ---------------------------------------------------------------------
# Powe status
# ---------------------------------------------------------------------
def aggregate_power_status():
    ev = latest_event("POWER_STATUS")
    if not ev:
        return

    payload = ev["payload"]
    states = []

    for s in payload.get("sensors", []):
        v = s.get("voltage_v", 0.0)
        states.append(
            "DOWN" if v <= 0 else
            "HOLD" if v < 1.0 else
            "NOMINAL"
        )

    payload["health_state"] = classify_health(states)
    create_or_update_aggregate("POWER_STATUS", payload)


def aggregate_network_status():
    ev = latest_event("NETWORK_STATUS")
    if not ev:
        return

    payload = ev["payload"]
    net_state = payload.get("network_status")

    payload["health_state"] = (
        "NOMINAL" if net_state == "NOMINAL"
        else "DOWN" if net_state == "DOWN"
        else "HOLD"
    )

    create_or_update_aggregate("NETWORK_STATUS", payload)


def aggregate_sensor_scan():
    ev = latest_event("SENSOR_SCAN")
    if not ev:
        return

    payload = ev["payload"]
    states = [
        "DOWN" if v == "OFFLINE" else v
        for v in payload.values()
        if isinstance(v, str)
    ]

    payload["health_state"] = classify_health(states)
    create_or_update_aggregate("SENSOR_SCAN", payload)


def aggregate_teensy_status():
    ev = latest_event("TEENSY_STATUS")
    if not ev:
        return

    payload = ev["payload"]
    payload.setdefault("health_state", "NOMINAL")
    create_or_update_aggregate("TEENSY_STATUS", payload)


def aggregate_environment_status():
    ev = latest_event("ENVIRONMENT_STATUS")
    if not ev:
        create_or_update_aggregate("ENVIRONMENT_STATUS", {"health_state": "DOWN"})
        return

    payload = ev["payload"]
    payload.setdefault("health_state", "NOMINAL")
    create_or_update_aggregate("ENVIRONMENT_STATUS", payload)


def aggregate_laser_status():
    """
    Derive LASER_STATUS from direct physical observation.

    Semantics:
      • Driver presence comes from LASER_STATUS (I²C observation)
      • Emission state comes from imperative PHOTODIODE.STATUS? query
      • No dependence on LASER.ON / LASER.OFF event history
      • Ground truth is photonic, not intentional
    """
    status_ev = latest_event("LASER_STATUS")
    payload = status_ev["payload"] if status_ev else {}

    # Default: assume laser is not emitting
    laser_on = False

    # ------------------------------------------------------------------
    # Imperative photodiode query (safe IPC path)
    # ------------------------------------------------------------------
    try:
        from zpnet.shared.constants import LASER_PHOTODIODE_ON_THRESHOLD_V

        events = photodiode_status()
        if events:
            pd = events[0].get("payload", {})
            analog_v = pd.get("analog_v")

            if isinstance(analog_v, (int, float)):
                laser_on = analog_v >= LASER_PHOTODIODE_ON_THRESHOLD_V

    except Exception:
        # Imperative query failure must NEVER kill aggregation
        laser_on = False

    payload["laser_enabled"] = laser_on

    # ------------------------------------------------------------------
    # Health semantics
    # ------------------------------------------------------------------
    if payload.get("device_present") is True:
        payload["health_state"] = "NOMINAL"
    else:
        payload["health_state"] = "HOLD"

    create_or_update_aggregate("LASER_STATUS", payload)



def aggregate_gnss_data():
    ev = latest_event("GNSS_DATA")
    if not ev:
        create_or_update_aggregate("GNSS_DATA", {"health_state": "DOWN"})
        return

    payload = ev["payload"]
    payload.setdefault("health_state", "NOMINAL")
    create_or_update_aggregate("GNSS_DATA", payload)


def aggregate_system_errors():
    cutoff = datetime.now(timezone.utc) - timedelta(seconds=ERROR_WINDOW_SEC)

    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT ts, payload
            FROM zpnet_events
            WHERE event_type = 'SYSTEM_ERROR'
              AND ts >= %s
            ORDER BY ts DESC
            LIMIT %s
            """,
            (cutoff, MAX_ERRORS_AGGREGATED),
        )
        rows = cur.fetchall()

    events = []
    for row in rows:
        payload = normalize_payload(row["payload"])
        events.append(
            {
                "timestamp": normalize_ts(row["ts"]).isoformat().replace("+00:00", "Z"),
                "message": payload.get("exception")
                           or payload.get("message")
                           or "SYSTEM ERROR",
                "context": payload,
            }
        )

    create_or_update_aggregate("SYSTEM_ERROR", {"events": events})


# ---------------------------------------------------------------------
# Dispatcher
# ---------------------------------------------------------------------
def run():
    aggregate_raspberry_pi_status()
    aggregate_teensy_status()
    aggregate_environment_status()
    aggregate_gnss_data()
    aggregate_battery_state_of_charge()
    aggregate_power_status()
    aggregate_network_status()
    aggregate_sensor_scan()
    aggregate_laser_status()
    aggregate_system_errors()


def bootstrap():
    setup_logging()
    run()
