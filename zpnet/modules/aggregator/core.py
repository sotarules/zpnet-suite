"""
ZPNet Aggregator (Unified Health + Power + Environment + Laser Revision)

Computes summary aggregates from zpnet_events and stores them in the
aggregates table. Each subsystem publishes its health_state using the
NASA-style vocabulary: NOMINAL, HOLD, or DOWN.

This revision adds LASER_STATUS as a first-class aggregate.

Author: The Mule
"""

import json
import logging
import sqlite3
from datetime import datetime, timedelta, timezone

from zpnet.shared.logger import setup_logging
from zpnet.shared.constants import DB_PATH

BATTERY_ADDR = "0x40"
BATTERY_CAPACITY_WH = 110.0
V_BATTERY_FULL = 13.384
V_BATTERY_EMPTY = 11.00

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
      - all NOMINAL → NOMINAL
      - any DOWN → DOWN
      - otherwise HOLD
    """
    if not states:
        return "DOWN"
    if all(s == "NOMINAL" for s in states):
        return "NOMINAL"
    if any(s == "DOWN" for s in states):
        return "DOWN"
    return "HOLD"


# ---------------------------------------------------------------------
# Helper: Upsert aggregate
# ---------------------------------------------------------------------
def create_or_update_aggregate(aggregate_type: str, payload: dict):
    ts = datetime.now(timezone.utc).isoformat()
    payload_json = json.dumps(payload or {})

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
        logging.debug(f"✅ [aggregator] Aggregate updated: {aggregate_type}")
    except Exception as e:
        logging.exception(
            f"⚠️ [aggregator] failed to update aggregate {aggregate_type}: {e}"
        )
        raise


# ---------------------------------------------------------------------
# Battery State of Charge
# ---------------------------------------------------------------------
def aggregate_battery_state_of_charge():
    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()

        cur.execute(
            "SELECT timestamp FROM zpnet_events WHERE event_type='SWAP_BATTERY' "
            "ORDER BY timestamp DESC LIMIT 1"
        )
        swap_row = cur.fetchone()
        if not swap_row:
            raise RuntimeError("[aggregator] no SWAP_BATTERY event found")

        swap_point = swap_row["timestamp"]

        cur.execute(
            "SELECT timestamp, payload FROM zpnet_events "
            "WHERE event_type='POWER_STATUS' AND timestamp>=? ORDER BY timestamp ASC",
            (swap_point,),
        )
        power_rows = cur.fetchall()
        if len(power_rows) < 2:
            logging.info("[aggregator] waiting for more POWER_STATUS samples")
            return

    total_wh = 0.0
    last_ts = None
    last_power = None
    samples_used = 0

    for i in range(0, len(power_rows), POWER_SAMPLE_STEP):
        row = power_rows[i]
        payload = json.loads(row["payload"])
        sensors = payload.get("sensors", [])
        battery = next(
            (s for s in sensors if s["address"] == BATTERY_ADDR), None
        )
        if not battery or "power_w" not in battery:
            continue

        ts = datetime.fromisoformat(row["timestamp"])
        power_w = battery["power_w"]

        if last_ts and last_power:
            dt = (ts - last_ts).total_seconds()
            avg_power = (power_w + last_power) / 2.0
            total_wh += abs(avg_power * dt / 3600.0)
            samples_used += 1

        last_ts, last_power = ts, power_w

    remaining_wh = max(0.0, BATTERY_CAPACITY_WH - total_wh)
    remaining_pct = round(100.0 * remaining_wh / BATTERY_CAPACITY_WH, 1)
    tte_minutes = (
        round(remaining_wh / last_power * 60.0, 1)
        if last_power and last_power > 1.0
        else float("inf")
    )

    health_state = "NOMINAL" if remaining_pct > 10 else "HOLD"
    if remaining_pct <= 3:
        health_state = "DOWN"

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
# Power Status
# ---------------------------------------------------------------------
def aggregate_power_status():
    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute(
            "SELECT payload FROM zpnet_events WHERE event_type='POWER_STATUS' "
            "ORDER BY timestamp DESC LIMIT 1"
        )
        row = cur.fetchone()
        if not row:
            raise RuntimeError("[aggregator] no POWER_STATUS events found")

    payload = json.loads(row["payload"])
    sensors = payload.get("sensors", [])

    sensor_states = []
    for s in sensors:
        v = s.get("voltage_v", 0.0)
        if v <= 0:
            sensor_states.append("DOWN")
        elif v < 1.0:
            sensor_states.append("HOLD")
        else:
            sensor_states.append("NOMINAL")

    payload["health_state"] = classify_health(sensor_states)
    create_or_update_aggregate("POWER_STATUS", payload)


# ---------------------------------------------------------------------
# Network Status
# ---------------------------------------------------------------------
def aggregate_network_status():
    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute(
            "SELECT payload FROM zpnet_events WHERE event_type='NETWORK_STATUS' "
            "ORDER BY timestamp DESC LIMIT 1"
        )
        row = cur.fetchone()
        if not row:
            raise RuntimeError("[aggregator] no NETWORK_STATUS events found")

    payload = json.loads(row["payload"])
    net_state = payload.get("network_status", "UNKNOWN")

    if net_state == "NOMINAL":
        payload["health_state"] = "NOMINAL"
    elif net_state == "DOWN":
        payload["health_state"] = "DOWN"
    else:
        payload["health_state"] = "HOLD"

    create_or_update_aggregate("NETWORK_STATUS", payload)


# ---------------------------------------------------------------------
# Sensor Scan
# ---------------------------------------------------------------------
def aggregate_sensor_scan():
    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute(
            "SELECT payload FROM zpnet_events WHERE event_type='SENSOR_SCAN' "
            "ORDER BY timestamp DESC LIMIT 1"
        )
        row = cur.fetchone()
        if not row:
            raise RuntimeError("[aggregator] missing SENSOR_SCAN event")

    scan = json.loads(row["payload"])

    normalized_states = []
    for v in scan.values():
        if not isinstance(v, str):
            continue
        if v == "OFFLINE":
            normalized_states.append("DOWN")
        else:
            normalized_states.append(v)

    scan["health_state"] = classify_health(normalized_states)
    create_or_update_aggregate("SENSOR_SCAN", scan)


# ---------------------------------------------------------------------
# Teensy Status
# ---------------------------------------------------------------------
def aggregate_teensy_status():
    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute(
            "SELECT payload FROM zpnet_events WHERE event_type='TEENSY_STATUS' "
            "ORDER BY timestamp DESC LIMIT 1"
        )
        row = cur.fetchone()
        if not row:
            raise RuntimeError("[aggregator] no TEENSY_STATUS events found")

    payload = json.loads(row["payload"])
    status = payload.get("status", "UNKNOWN")

    if status == "NOMINAL":
        payload["health_state"] = "NOMINAL"
    elif status == "DOWN":
        payload["health_state"] = "DOWN"
    else:
        payload["health_state"] = "HOLD"

    create_or_update_aggregate("TEENSY_STATUS", payload)


# ---------------------------------------------------------------------
# Raspberry Pi Status
# ---------------------------------------------------------------------
def aggregate_raspberry_pi_status():
    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute(
            "SELECT payload FROM zpnet_events WHERE event_type='RASPBERRY_PI_STATUS' "
            "ORDER BY timestamp DESC LIMIT 1"
        )
        row = cur.fetchone()
        if not row:
            raise RuntimeError("[aggregator] no RASPBERRY_PI_STATUS events found")

    payload = json.loads(row["payload"])
    create_or_update_aggregate("RASPBERRY_PI_STATUS", payload)


# ---------------------------------------------------------------------
# Environment Status
# ---------------------------------------------------------------------
def aggregate_environment_status():
    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute(
            "SELECT payload FROM zpnet_events WHERE event_type='ENVIRONMENT_STATUS' "
            "ORDER BY timestamp DESC LIMIT 1"
        )
        row = cur.fetchone()
        if not row:
            raise RuntimeError("[aggregator] no ENVIRONMENT_STATUS events found")

    payload = json.loads(row["payload"])
    payload.setdefault("health_state", "DOWN")
    create_or_update_aggregate("ENVIRONMENT_STATUS", payload)


# ---------------------------------------------------------------------
# Laser Status (NEW)
# ---------------------------------------------------------------------
def aggregate_laser_status():
    """
    Aggregate LASER_STATUS as a singleton.
    No reinterpretation beyond health_state preservation.
    """
    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute(
            "SELECT payload FROM zpnet_events WHERE event_type='LASER_STATUS' "
            "ORDER BY timestamp DESC LIMIT 1"
        )
        row = cur.fetchone()
        if not row:
            raise RuntimeError("[aggregator] no LASER_STATUS events found")

    payload = json.loads(row["payload"])
    payload.setdefault("health_state", "DOWN")
    create_or_update_aggregate("LASER_STATUS", payload)


# ---------------------------------------------------------------------
# System Errors
# ---------------------------------------------------------------------
def aggregate_system_errors():
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
def run():
    """
    Execute all aggregation pipelines sequentially.
    """
    try:
        aggregate_raspberry_pi_status()
        aggregate_teensy_status()
        aggregate_environment_status()
        aggregate_laser_status()        # ← NEW
        aggregate_battery_state_of_charge()
        aggregate_power_status()
        aggregate_network_status()
        aggregate_sensor_scan()
        aggregate_system_errors()
    except Exception as e:
        logging.exception(f"🔥 [aggregator] aggregator loop failed: {e}")
        raise


def bootstrap():
    setup_logging()
    run()
