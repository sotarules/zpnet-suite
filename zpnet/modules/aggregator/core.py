"""
ZPNet Aggregator — Unified Truth Assembly Layer

This module assembles authoritative aggregates from persisted ZPNet events.
It performs NO hardware access, NO inference about causality, and NO cadence
assumptions.  It is a pure reducer from event history → current system truth.

Key principles:
  • Events are intentional and sparse
  • Absence of events carries no meaning
  • Aggregates mirror the most recent known truth
  • Health is classified explicitly and conservatively

Author: The Mule
"""

import json
import logging
import sqlite3
from datetime import datetime, timedelta, timezone

from zpnet.shared.logger import setup_logging
from zpnet.shared.constants import DB_PATH

# ---------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------
BATTERY_ADDR = "0x40"
BATTERY_CAPACITY_WH = 110.0

POWER_SAMPLE_STEP = 50
MAX_ERRORS_AGGREGATED = 5
ERROR_WINDOW_SEC = 3600  # 1 hour


# ---------------------------------------------------------------------
# Helper: Health classification
# ---------------------------------------------------------------------
def classify_health(states: list[str]) -> str:
    """
    Determine aggregate health from constituent subsystem states.

    Rules:
      • all NOMINAL → NOMINAL
      • any DOWN → DOWN
      • otherwise → HOLD
    """
    if not states:
        return "DOWN"
    if all(s == "NOMINAL" for s in states):
        return "NOMINAL"
    if any(s == "DOWN" for s in states):
        return "DOWN"
    return "HOLD"


# ---------------------------------------------------------------------
# Helper: Aggregate upsert
# ---------------------------------------------------------------------
def create_or_update_aggregate(aggregate_type: str, payload: dict) -> None:
    ts = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    payload_json = json.dumps(payload or {}, separators=(",", ":"))

    try:
        with sqlite3.connect(DB_PATH) as conn:
            cur = conn.cursor()
            cur.execute(
                """
                INSERT INTO aggregates (aggregate_type, timestamp, payload)
                VALUES (?, ?, ?)
                ON CONFLICT(aggregate_type)
                DO UPDATE SET timestamp=excluded.timestamp,
                              payload=excluded.payload
                """,
                (aggregate_type, ts, payload_json),
            )
            conn.commit()
        logging.debug(f"[aggregator] updated {aggregate_type}")
    except Exception as e:
        logging.exception(
            f"[aggregator] failed to update aggregate {aggregate_type}: {e}"
        )
        raise


# ---------------------------------------------------------------------
# Battery State of Charge
# ---------------------------------------------------------------------
def aggregate_battery_state_of_charge() -> None:
    """
    Estimate battery state of charge based on integrated POWER_STATUS
    events since the most recent SWAP_BATTERY marker.
    """
    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()

        cur.execute(
            "SELECT timestamp FROM zpnet_events WHERE event_type='SWAP_BATTERY' "
            "ORDER BY timestamp DESC LIMIT 1"
        )
        swap_row = cur.fetchone()
        if not swap_row:
            return

        swap_point = swap_row["timestamp"]

        cur.execute(
            "SELECT timestamp, payload FROM zpnet_events "
            "WHERE event_type='POWER_STATUS' AND timestamp>=? "
            "ORDER BY timestamp ASC",
            (swap_point,),
        )
        rows = cur.fetchall()

    if len(rows) < 2:
        return

    total_wh = 0.0
    last_ts = None
    last_power = None

    for i in range(0, len(rows), POWER_SAMPLE_STEP):
        row = rows[i]
        payload = json.loads(row["payload"])
        sensors = payload.get("sensors", [])
        battery = next((s for s in sensors if s["address"] == BATTERY_ADDR), None)
        if not battery:
            continue

        ts = datetime.fromisoformat(row["timestamp"])
        power_w = battery.get("power_w")
        if power_w is None:
            continue

        if last_ts and last_power is not None:
            dt = (ts - last_ts).total_seconds()
            avg_power = (power_w + last_power) / 2.0
            total_wh += abs(avg_power * dt / 3600.0)

        last_ts = ts
        last_power = power_w

    remaining_wh = max(0.0, BATTERY_CAPACITY_WH - total_wh)
    remaining_pct = round(100.0 * remaining_wh / BATTERY_CAPACITY_WH, 1)
    tte_minutes = (
        round(remaining_wh / last_power * 60.0, 1)
        if last_power and last_power > 0
        else float("inf")
    )

    if remaining_pct <= 3:
        health = "DOWN"
    elif remaining_pct <= 10:
        health = "HOLD"
    else:
        health = "NOMINAL"

    create_or_update_aggregate(
        "BATTERY_STATE_OF_CHARGE",
        {
            "remaining_pct": remaining_pct,
            "tte_minutes": tte_minutes,
            "wh_used_since_recharge": round(total_wh, 2),
            "wh_remaining_estimate": round(remaining_wh, 2),
            "battery_capacity_wh": BATTERY_CAPACITY_WH,
            "health_state": health,
        },
    )


# ---------------------------------------------------------------------
# Power Status
# ---------------------------------------------------------------------
def aggregate_power_status() -> None:
    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute(
            "SELECT payload FROM zpnet_events "
            "WHERE event_type='POWER_STATUS' "
            "ORDER BY timestamp DESC LIMIT 1"
        )
        row = cur.fetchone()
        if not row:
            return

    payload = json.loads(row["payload"])
    sensors = payload.get("sensors", [])

    states = []
    for s in sensors:
        v = s.get("voltage_v", 0.0)
        if v <= 0:
            states.append("DOWN")
        elif v < 1.0:
            states.append("HOLD")
        else:
            states.append("NOMINAL")

    payload["health_state"] = classify_health(states)
    create_or_update_aggregate("POWER_STATUS", payload)


# ---------------------------------------------------------------------
# Network Status
# ---------------------------------------------------------------------
def aggregate_network_status() -> None:
    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute(
            "SELECT payload FROM zpnet_events "
            "WHERE event_type='NETWORK_STATUS' "
            "ORDER BY timestamp DESC LIMIT 1"
        )
        row = cur.fetchone()
        if not row:
            return

    payload = json.loads(row["payload"])
    net_state = payload.get("network_status")

    if net_state == "NOMINAL":
        payload["health_state"] = "NOMINAL"
    elif net_state == "DOWN":
        payload["health_state"] = "DOWN"
    else:
        payload["health_state"] = "HOLD"

    create_or_update_aggregate("NETWORK_STATUS", payload)


# ---------------------------------------------------------------------
# Sensor Scan (OFFLINE → DOWN normalization)
# ---------------------------------------------------------------------
def aggregate_sensor_scan() -> None:
    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute(
            "SELECT payload FROM zpnet_events "
            "WHERE event_type='SENSOR_SCAN' "
            "ORDER BY timestamp DESC LIMIT 1"
        )
        row = cur.fetchone()
        if not row:
            return

    scan = json.loads(row["payload"])
    states = []

    for v in scan.values():
        if not isinstance(v, str):
            continue
        states.append("DOWN" if v == "OFFLINE" else v)

    scan["health_state"] = classify_health(states)
    create_or_update_aggregate("SENSOR_SCAN", scan)


# ---------------------------------------------------------------------
# Teensy Status
# ---------------------------------------------------------------------
def aggregate_teensy_status() -> None:
    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute(
            "SELECT payload FROM zpnet_events "
            "WHERE event_type='TEENSY_STATUS' "
            "ORDER BY timestamp DESC LIMIT 1"
        )
        row = cur.fetchone()
        if not row:
            return

    payload = json.loads(row["payload"])
    payload.setdefault("health_state", "NOMINAL")
    create_or_update_aggregate("TEENSY_STATUS", payload)


# ---------------------------------------------------------------------
# Environment Status
# ---------------------------------------------------------------------
def aggregate_environment_status() -> None:
    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute(
            "SELECT payload FROM zpnet_events "
            "WHERE event_type='ENVIRONMENT_STATUS' "
            "ORDER BY timestamp DESC LIMIT 1"
        )
        row = cur.fetchone()
        if not row:
            return

    payload = json.loads(row["payload"])
    payload.setdefault("health_state", "DOWN")
    create_or_update_aggregate("ENVIRONMENT_STATUS", payload)


# ---------------------------------------------------------------------
# Laser Status (Merged Truth)
# ---------------------------------------------------------------------
def aggregate_laser_status() -> None:
    """
    Merge observed laser driver truth (LASER_STATUS)
    with commanded truth (LASER_STATE.enabled).
    """
    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()

        cur.execute(
            "SELECT payload FROM zpnet_events "
            "WHERE event_type='LASER_STATUS' "
            "ORDER BY timestamp DESC LIMIT 1"
        )
        status_row = cur.fetchone()

        cur.execute(
            "SELECT payload FROM zpnet_events "
            "WHERE event_type='LASER_STATE' "
            "ORDER BY timestamp DESC LIMIT 1"
        )
        state_row = cur.fetchone()

    payload = json.loads(status_row["payload"]) if status_row else {}

    if state_row:
        state = json.loads(state_row["payload"])
        if isinstance(state.get("enabled"), bool):
            payload["laser_enabled"] = state["enabled"]

    payload.setdefault("health_state", "HOLD")
    create_or_update_aggregate("LASER_STATUS", payload)


# ---------------------------------------------------------------------
# GNSS Data (Authoritative, Query-Driven)
# ---------------------------------------------------------------------
def aggregate_gnss_data() -> None:
    """
    Aggregate GNSS_DATA as a singleton authoritative snapshot.

    Behavior:
      • Mirrors most recent GNSS_DATA event verbatim
      • Performs NO inference
      • Absence → explicit DOWN
    """
    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute(
            "SELECT payload FROM zpnet_events "
            "WHERE event_type='GNSS_DATA' "
            "ORDER BY timestamp DESC LIMIT 1"
        )
        row = cur.fetchone()

    if not row:
        create_or_update_aggregate(
            "GNSS_DATA",
            {"health_state": "DOWN"},
        )
        return

    payload = json.loads(row["payload"])
    payload.setdefault("health_state", "NOMINAL")
    create_or_update_aggregate("GNSS_DATA", payload)


# ---------------------------------------------------------------------
# System Errors
# ---------------------------------------------------------------------
def aggregate_system_errors() -> None:
    cutoff = datetime.now(timezone.utc) - timedelta(seconds=ERROR_WINDOW_SEC)

    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute(
            """
            SELECT timestamp, payload FROM zpnet_events
            WHERE event_type='SYSTEM_ERROR' AND timestamp >= ?
            ORDER BY timestamp DESC LIMIT ?
            """,
            (cutoff.isoformat(), MAX_ERRORS_AGGREGATED),
        )
        rows = cur.fetchall()

    events = []
    for row in rows:
        payload = json.loads(row["payload"])
        events.append(
            {
                "timestamp": row["timestamp"],
                "message": payload.get("exception")
                or payload.get("message")
                or "SYSTEM ERROR",
                "context": payload,
            }
        )

    create_or_update_aggregate("SYSTEM_ERROR", {"events": events})


# ---------------------------------------------------------------------
# Main Dispatcher
# ---------------------------------------------------------------------
def run() -> None:
    """
    Execute all aggregation pipelines sequentially.
    """
    aggregate_environment_status()
    aggregate_gnss_data()
    aggregate_battery_state_of_charge()
    aggregate_power_status()
    aggregate_network_status()
    aggregate_sensor_scan()
    aggregate_teensy_status()
    aggregate_laser_status()
    aggregate_system_errors()


def bootstrap() -> None:
    setup_logging()
    run()
