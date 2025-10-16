"""
ZPNet Aggregator

Computes summary aggregates from zpnet_events (event history)
and stores results in the aggregates table.
"""

import json
import logging
import sqlite3
from datetime import datetime, timedelta, timezone

from zpnet.shared.logger import setup_logging

DB_PATH = "/home/mule/zpnet/zpnet.db"

BATTERY_ADDR = "0x40"
BATTERY_CAPACITY_WH = 128.0
V_BATTERY_FULL = 13.384
V_BATTERY_EMPTY = 9.706

POWER_SAMPLE_STEP = 50
MAX_ERRORS_AGGREGATED = 5
ERROR_WINDOW_SEC = 3600  # 1 hour


# --------------------------
# Helper: upsert aggregate
# --------------------------
def create_or_update_aggregate(aggregate_type: str, payload: dict):
    ts = datetime.utcnow().isoformat()
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
        logging.debug(f"✅ Aggregate updated: {aggregate_type}")
    except Exception as e:
        logging.exception(f"⚠️ Failed to update aggregate {aggregate_type}: {e}")


# --------------------------
# Battery SoC Estimator
# --------------------------
def aggregate_battery_state_of_charge():
    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()

        cur.execute(
            "SELECT timestamp FROM zpnet_events WHERE event_type = 'SWAP_BATTERY' ORDER BY timestamp DESC LIMIT 1"
        )
        swap_row = cur.fetchone()
        if not swap_row:
            logging.warning("⚠️ No SWAP_BATTERY event found; skipping SoC")
            return

        swap_point = swap_row["timestamp"]
        swap_voltage = None

        cur.execute(
            """
            SELECT payload FROM zpnet_events
            WHERE event_type = 'POWER_STATUS' AND timestamp >= ?
            ORDER BY timestamp ASC LIMIT 1
            """,
            (swap_point,),
        )
        first_power_row = cur.fetchone()
        if first_power_row:
            try:
                sensors = json.loads(first_power_row["payload"]).get("sensors", [])
                battery = next((s for s in sensors if s["address"] == BATTERY_ADDR), None)
                if battery:
                    swap_voltage = battery.get("voltage")
            except Exception:
                pass

        cur.execute(
            """
            SELECT timestamp, payload FROM zpnet_events
            WHERE event_type = 'POWER_STATUS' AND timestamp >= ?
            ORDER BY timestamp ASC
            """,
            (swap_point,),
        )
        forward_rows = cur.fetchall()
        if len(forward_rows) < 2:
            logging.warning("⚠️ Not enough POWER_STATUS events after swap")
            return

    total_wh = 0.0
    last_ts = None
    last_power = None
    samples_used = 0

    for i in range(0, len(forward_rows), POWER_SAMPLE_STEP):
        row = forward_rows[i]
        try:
            payload = json.loads(row["payload"])
            sensors = payload.get("sensors", [])
            battery = next((s for s in sensors if s["address"] == BATTERY_ADDR), None)
            if not battery or "power" not in battery:
                continue

            ts = datetime.fromisoformat(row["timestamp"])
            power_W = battery["power"]

            if last_ts is not None and last_power is not None:
                dt_sec = (ts - last_ts).total_seconds()
                avg_power = (power_W + last_power) / 2.0
                wh = avg_power * dt_sec / 3600.0
                total_wh += abs(wh)
                samples_used += 1

            last_ts = ts
            last_power = power_W
        except Exception:
            continue

    remaining_wh = max(0.0, BATTERY_CAPACITY_WH - total_wh)
    remaining_pct = round(100.0 * remaining_wh / BATTERY_CAPACITY_WH, 1)
    tte_minutes = round(remaining_wh / last_power * 60.0, 1) if last_power and last_power > 1.0 else float("inf")

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
            "swap_time": swap_point,
            "swap_voltage": swap_voltage,
        },
    )


# --------------------------
# Network Status
# --------------------------
def aggregate_network_status():
    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute(
            """
            SELECT payload FROM zpnet_events
            WHERE event_type = 'NETWORK_STATUS'
            ORDER BY timestamp DESC LIMIT 1
            """
        )
        row = cur.fetchone()
        if not row:
            logging.warning("⚠️ No NETWORK_STATUS events found")
            return

        payload = json.loads(row["payload"])
        create_or_update_aggregate("NETWORK_STATUS", payload)


# --------------------------
# Sensor Scan
# --------------------------
def aggregate_sensor_scan():
    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()

        cur.execute(
            "SELECT payload FROM zpnet_events WHERE event_type = 'SENSOR_SCAN' ORDER BY timestamp DESC LIMIT 1"
        )
        row_scan = cur.fetchone()

        cur.execute(
            "SELECT payload FROM zpnet_events WHERE event_type = 'HEARTBEAT' ORDER BY timestamp DESC LIMIT 1"
        )
        row_hb = cur.fetchone()

    if not (row_scan and row_hb):
        logging.warning("⚠️ No SENSOR_SCAN or HEARTBEAT found")
        return

    payload = json.loads(row_scan["payload"]) if row_scan else {}
    hb = json.loads(row_hb["payload"]) if row_hb else {}
    payload["teensy_status"] = hb.get("status", "UNKNOWN")

    create_or_update_aggregate("SENSOR_SCAN", payload)


# --------------------------
# System Error Consolidation
# --------------------------
def aggregate_system_errors():
    cutoff = datetime.now(timezone.utc) - timedelta(seconds=ERROR_WINDOW_SEC)

    with sqlite3.connect(DB_PATH) as conn:
        conn.row_factory = sqlite3.Row
        cur = conn.cursor()
        cur.execute(
            """
            SELECT timestamp, payload FROM zpnet_events
            WHERE event_type = 'SYSTEM_ERROR' AND timestamp >= ?
            ORDER BY timestamp DESC LIMIT ?
            """,
            (cutoff.isoformat(), MAX_ERRORS_AGGREGATED),
        )
        rows = cur.fetchall()

    events = []
    for row in rows:
        try:
            payload = json.loads(row["payload"])
        except Exception:
            payload = {"raw": row["payload"]}
        events.append({
            "timestamp": row["timestamp"],
            "message": payload.get("exception") or payload.get("message") or "SYSTEM ERROR",
            "context": payload,
        })

    create_or_update_aggregate("SYSTEM_ERROR", {"events": events})


# --------------------------
# Main dispatcher
# --------------------------
def run():
    try:
        aggregate_battery_state_of_charge()
        aggregate_network_status()
        aggregate_sensor_scan()
        aggregate_system_errors()
    except Exception as e:
        logging.exception(f"🔥 Aggregator loop failed: {e}")


def bootstrap():
    setup_logging()
    run()
