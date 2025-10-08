"""
ZPNet Aggregator

Computes summary aggregates from zpnet_events (event history)
and stores results in the aggregates table.
"""

import sqlite3
import json
import logging
from datetime import datetime, timedelta, timezone

from zpnet.shared.logger import setup_logging
from zpnet.shared.events import create_event  # <-- NEW

DB_PATH = "/home/mule/zpnet/zpnet.db"

BATTERY_ADDR = "0x40"
BATTERY_CAPACITY_MAH = 10000
V_BATTERY_FULL = 13.384
V_BATTERY_EMPTY = 9.706

# Battery energy capacity (can tune this based on empirical data)
BATTERY_CAPACITY_WH = 128.0  # ~10Ah × 12.8V nominal

# Downsampling step for power aggregation
POWER_SAMPLE_STEP = 50

# System error aggregation params
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
        logging.debug(f"Aggregate updated: {aggregate_type}")
    except Exception as e:
        logging.exception(f"Failed to update aggregate {aggregate_type}: {e}")
        create_event(
            "SYSTEM_ERROR",
            {"component": "aggregator", "aggregate_type": aggregate_type, "exception": str(e)},
        )


# --------------------------
# Aggregate builders
# --------------------------
def aggregate_battery_state_of_charge():
    """
    Compute battery state of charge (SoC) based on POWER_STATUS events
    since the most recent SWAP_BATTERY event.
    """
    try:
        # --- Step 1: Find latest SWAP_BATTERY event ---
        with sqlite3.connect(DB_PATH) as conn:
            conn.row_factory = sqlite3.Row
            cur = conn.cursor()
            cur.execute(
                """
                SELECT timestamp
                FROM zpnet_events
                WHERE event_type = 'SWAP_BATTERY'
                ORDER BY timestamp DESC
                LIMIT 1
                """
            )
            swap_row = cur.fetchone()

        if not swap_row:
            logging.warning("No SWAP_BATTERY event found; skipping SoC aggregation")
            return

        swap_point = swap_row["timestamp"]
        swap_voltage = None

        logging.debug(f"Using SWAP_BATTERY event at {swap_point}")

        # --- Step 2: Find first POWER_STATUS event *after* swap for initial voltage ---
        with sqlite3.connect(DB_PATH) as conn:
            conn.row_factory = sqlite3.Row
            cur = conn.cursor()
            cur.execute(
                """
                SELECT payload
                FROM zpnet_events
                WHERE event_type = 'POWER_STATUS'
                  AND timestamp >= ?
                ORDER BY timestamp ASC
                LIMIT 1
                """,
                (swap_point,),
            )
            first_power_row = cur.fetchone()

        if first_power_row:
            try:
                payload = json.loads(first_power_row["payload"])
                sensors = payload.get("sensors", [])
                battery = next((s for s in sensors if s["address"] == BATTERY_ADDR), None)
                if battery and "voltage" in battery:
                    swap_voltage = battery["voltage"]
            except Exception:
                pass

        # --- Step 3: Query POWER_STATUS events after that timestamp ---
        with sqlite3.connect(DB_PATH) as conn:
            conn.row_factory = sqlite3.Row
            cur = conn.cursor()
            cur.execute(
                """
                SELECT timestamp, payload
                FROM zpnet_events
                WHERE event_type = 'POWER_STATUS'
                  AND timestamp >= ?
                ORDER BY timestamp ASC
                """,
                (swap_point,),
            )
            forward_rows = cur.fetchall()

        if len(forward_rows) < 2:
            logging.warning("Not enough POWER_STATUS events after swap to compute SoC")
            return

        # --- Step 4: Integrate power over time to estimate Wh consumed ---
        total_wh = 0.0
        last_ts = None
        last_power = None
        samples_used = 0

        for i in range(0, len(forward_rows), POWER_SAMPLE_STEP):
            row = forward_rows[i]
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

        # --- Step 5: Compute remaining capacity and time to empty ---
        remaining_wh = max(0.0, BATTERY_CAPACITY_WH - total_wh)
        remaining_pct = round(100.0 * remaining_wh / BATTERY_CAPACITY_WH, 1)

        if last_power and last_power > 1.0:
            tte_minutes = round(remaining_wh / last_power * 60.0, 1)
        else:
            tte_minutes = float("inf")

        # --- Step 6: Store aggregate ---
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

    except Exception as e:
        logging.exception(f"Battery SoC aggregation failed: {e}")
        create_event(
            "SYSTEM_ERROR",
            {"component": "aggregate_battery_state_of_charge", "exception": str(e)},
        )


def aggregate_network_status():
    try:
        with sqlite3.connect(DB_PATH) as conn:
            conn.row_factory = sqlite3.Row
            cur = conn.cursor()
            cur.execute(
                """
                SELECT timestamp, payload
                FROM zpnet_events
                WHERE event_type = 'NETWORK_STATUS'
                ORDER BY timestamp DESC
                LIMIT 1
                """
            )
            row = cur.fetchone()

        if not row:
            logging.warning("No NETWORK_STATUS events found")
            return

        payload = json.loads(row["payload"])
        create_or_update_aggregate("NETWORK_STATUS", payload)
    except Exception as e:
        logging.exception(f"Network status aggregation failed: {e}")
        create_event(
            "SYSTEM_ERROR",
            {"component": "aggregate_network_status", "exception": str(e)},
        )


def aggregate_sensor_scan():
    try:
        with sqlite3.connect(DB_PATH) as conn:
            conn.row_factory = sqlite3.Row
            cur = conn.cursor()

            # last SENSOR_SCAN event (Pi local sensors)
            cur.execute(
                """
                SELECT payload FROM zpnet_events
                WHERE event_type = 'SENSOR_SCAN'
                ORDER BY timestamp DESC LIMIT 1
                """
            )
            row_scan = cur.fetchone()

            # last HEARTBEAT from Teensy
            cur.execute(
                """
                SELECT payload FROM zpnet_events
                WHERE event_type = 'HEARTBEAT'
                ORDER BY timestamp DESC LIMIT 1
                """
            )
            row_hb = cur.fetchone()

        if not (row_scan and row_hb):
            logging.warning("No SENSOR_SCAN/HEARTBEAT events found")
            return

        # start with Pi’s SENSOR_SCAN payload
        payload = json.loads(row_scan["payload"]) if row_scan else {}

        # add teensy status from heartbeat
        if row_hb:
            hb = json.loads(row_hb["payload"])
            payload["teensy_status"] = hb.get("status", "UNKNOWN")

        create_or_update_aggregate("SENSOR_SCAN", payload)

    except Exception as e:
        logging.exception(f"Sensor scan aggregation failed: {e}")
        create_event(
            "SYSTEM_ERROR",
            {"component": "aggregate_sensor_scan", "exception": str(e)},
        )


def aggregate_system_errors():
    """
    Collect recent SYSTEM_ERROR events into a SYSTEM_ERROR aggregate payload.
    Payload format: { "events": [ {timestamp, message, ...}, ... ] }
    """
    try:
        cutoff = datetime.now(timezone.utc) - timedelta(seconds=ERROR_WINDOW_SEC)

        with sqlite3.connect(DB_PATH) as conn:
            conn.row_factory = sqlite3.Row
            cur = conn.cursor()
            cur.execute(
                """
                SELECT timestamp, payload
                FROM zpnet_events
                WHERE event_type = 'SYSTEM_ERROR'
                  AND timestamp >= ?
                ORDER BY timestamp DESC
                LIMIT ?
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
    except Exception as e:
        logging.exception(f"System error aggregation failed: {e}")
        create_event(
            "SYSTEM_ERROR",
            {"component": "aggregate_system_errors", "exception": str(e)},
        )


# --------------------------
# Main dispatcher
# --------------------------
def run():
    """Run all aggregator functions once."""
    aggregate_battery_state_of_charge()
    aggregate_network_status()
    aggregate_sensor_scan()
    aggregate_system_errors()


def bootstrap():
    setup_logging()
    run()
