"""
ZPNet CLOCKS Process — TIMEBASE Authority (Pi-side)

Responsibilities:
  • Capture PPS pulse on Raspberry Pi GPIO
  • Capture Pi cycle count and Pi nanoseconds at PPS
  • Receive TIMEBASE_FRAGMENT from Teensy
  • Augment fragment into full TIMEBASE record
  • Publish TIMEBASE
  • Persist TIMEBASE to PostgreSQL (append-only, best-effort)
  • Recover clocks after restart if campaign is active

Semantics:
  • No derived values
  • No smoothing, inference, or filtering
  • TIMEBASE records are sacred and immutable
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Dict, Any, Optional

import gpiod
from gpiod.line import Edge

from zpnet.processes.processes import (
    server_setup,
    publish,
    send_command,
)
from zpnet.shared.constants import Payload
from zpnet.shared.db import open_db
from zpnet.shared.logger import setup_logging

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

GPIO_CHIP = "/dev/gpiochip0"
PPS_GPIO_LINE = 18

# ---------------------------------------------------------------------
# Local state (process lifetime)
# ---------------------------------------------------------------------

_pi_pps_count: int = 0

# Latest Pi-side PPS capture
_last_pi_pps: Optional[Dict[str, Any]] = None

# Latest Teensy fragment awaiting augmentation
_last_teensy_fragment: Optional[Dict[str, Any]] = None

_state_lock = threading.Lock()

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

def _read_pi_cycle_count() -> int:
    """
    Best-effort Pi cycle count approximation.
    Uses time.monotonic_ns() as the authoritative Pi clock substrate.
    """
    return time.monotonic_ns()


def _read_pi_ns_from_cycles(cycles_ns: int) -> int:
    """
    Canonical Pi nanosecond clock.
    Currently identical to cycle count (monotonic_ns).
    """
    return cycles_ns


def _get_gnss_time() -> str:
    """
    Fetch last-known GNSS UTC time from GNSS process.
    Returns ISO8601 Z string.
    """
    payload = send_command(
        machine="PI",
        subsystem="GNSS",
        command="REPORT",
    )["payload"]

    date = payload["date"]
    time_s = payload["time"]
    return f"{date}T{time_s}Z"


def _get_active_campaign_name() -> Optional[str]:
    """
    Return active campaign name or None.
    """
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT campaign
            FROM campaigns
            WHERE active = true
            ORDER BY started_at DESC
            LIMIT 1
            """
        )
        row = cur.fetchone()

    return row["campaign"] if row else None


def _persist_timebase(tb: Dict[str, Any]) -> None:
    """
    Best-effort TIMEBASE insert.
    Failure is logged and ignored.
    """
    try:
        with open_db() as conn:
            cur = conn.cursor()
            cur.execute(
                """
                INSERT INTO timebase (
                    campaign,
                    gnss_time_utc,
                    teensy_dwt_cycles,
                    teensy_dwt_ns,
                    teensy_ocxo_ns,
                    teensy_rtc1_ns,
                    teensy_rtc2_ns,
                    pi_cycles,
                    pi_ns,
                    teensy_pps_count,
                    pi_pps_count                    
                )
                VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)
                """,
                (
                    tb["campaign"],
                    tb["gnss_time_utc"],
                    tb["teensy_dwt_cycles"],
                    tb["teensy_dwt_ns"],
                    tb.get("teensy_ocxo_ns"),
                    tb.get("teensy_rtc1_ns"),
                    tb.get("teensy_rtc2_ns"),
                    tb["pi_cycles"],
                    tb["pi_ns"],
                    tb.get("teensy_pps_count"),
                    tb.get("pi_pps_count"),
                ),
            )
    except Exception:
        logging.exception("⚠️ [clocks] failed to persist TIMEBASE (ignored)")


def _pps_listener() -> None:
    """
    Listen for PPS rising edge and capture Pi clocks immediately.
    """
    global _pi_pps_count
    global _last_pi_pps

    chip = gpiod.Chip(GPIO_CHIP)

    settings = gpiod.LineSettings(
        edge_detection=Edge.RISING
    )

    request = chip.request_lines(
        consumer="zpnet-clocks",
        config={PPS_GPIO_LINE: settings},
    )

    logging.info("⏱️ [clocks] PPS listener started on GPIO18")

    while True:
        request.wait_edge_events()

        for event in request.read_edge_events():

            # Kernel timestamp of the PPS edge (best possible truth)
            pps_kernel_ns = event.timestamp_ns

            # Capture Pi cycle counter ASAP
            pi_cycles = _read_pi_cycle_count()

            # Convert cycles → ns via your tau math
            pi_ns = _read_pi_ns_from_cycles(pi_cycles)

            with _state_lock:
                _last_pi_pps = {
                    "pps_kernel_ns": pps_kernel_ns,
                    "pi_cycles": pi_cycles,
                    "pi_ns": pi_ns,
                    "pi_pps_count": _pi_pps_count,
                }

            _pi_pps_count += 1

# ---------------------------------------------------------------------
# Teensy fragment handler
# ---------------------------------------------------------------------

def on_timebase_fragment(payload: Payload) -> None:
    """
    Receive TIMEBASE_FRAGMENT from Teensy.
    Attempt to complete TIMEBASE if Pi PPS is available.
    """
    global _last_teensy_fragment

    with _state_lock:
        _last_teensy_fragment = payload.copy()

    _try_complete_timebase()


def _try_complete_timebase() -> None:
    """
    Combine Pi PPS + Teensy fragment + GNSS time into TIMEBASE.
    """
    global _last_pi_pps, _last_teensy_fragment

    with _state_lock:
        if _last_pi_pps is None or _last_teensy_fragment is None:
            return

        pi_pps = _last_pi_pps
        frag = _last_teensy_fragment

        # Clear state immediately (one-shot)
        _last_pi_pps = None
        _last_teensy_fragment = None

    campaign = _get_active_campaign_name()
    if campaign is None:
        logging.warning("⚠️ [clocks] PPS received with no active campaign")
        return

    gnss_time = _get_gnss_time()

    timebase = {
        "campaign": campaign,
        "gnss_time_utc": gnss_time,

        # Teensy clocks
        "teensy_dwt_cycles": frag["dwt_cycles"],
        "teensy_dwt_ns": frag["dwt_ns"],
        "teensy_ocxo_ns": frag.get("ocxo_ns"),
        "teensy_rtc1_ns": frag.get("rtc1_ns"),
        "teensy_rtc2_ns": frag.get("rtc2_ns"),

        # Pi clocks
        "pi_cycles": pi_pps["pi_cycles"],
        "pi_ns": pi_pps["pi_ns"],

        # Counters (for sanity checking)
        "teensy_pps_count": frag.get("teensy_pps_count"),
        "pi_pps_count": pi_pps["pi_pps_count"],
    }

    # Publish sacred tuple
    publish("TIMEBASE", timebase)

    # Persist best-effort
    _persist_timebase(timebase)


# ---------------------------------------------------------------------
# Command handlers (control plane)
# ---------------------------------------------------------------------

def cmd_start(args: Optional[dict]) -> dict:
    """
    Start a new campaign.
    """
    global _pi_pps_count
    _pi_pps_count = 0

    campaign = args.get("campaign")

    with open_db() as conn:
        cur = conn.cursor()

        # Stop any existing campaign
        cur.execute(
            "UPDATE campaigns SET active=false, stopped_at=now() WHERE active=true"
        )

        cur.execute(
            """
            INSERT INTO campaigns (campaign, started_at, active)
            VALUES (%s, now(), true)
            """,
            (campaign,),
        )

    payload = send_command(
        machine="TEENSY",
        subsystem="CLOCKS",
        command="START",
        args={"campaign": campaign},
    )["payload"]

    return {"success": True, "message": "OK"}


def cmd_stop(_: Optional[dict]) -> dict:
    """
    Stop active campaign.
    """
    with open_db() as conn:
        cur = conn.cursor()
        cur.execute(
            "UPDATE campaigns SET active=false, stopped_at=now() WHERE active=true"
        )

    payload = send_command(
        machine="TEENSY",
        subsystem="CLOCKS",
        command="STOP"
    )["payload"]

    return {"success": True, "message": "OK"}


def cmd_report(_: Optional[dict]) -> Dict[str, Any]:
    """
    Report current CLOCKS state.

    Semantics:
      • Observational only
      • No inference
      • No derived values
    """

    payload: Dict[str, Any] = {
        "campaign": {
            "active": False
        }
    }

    with open_db(row_dict=True) as conn:
        cur = conn.cursor()

        # ------------------------------------------------------------
        # Fetch active campaign (authoritative)
        # ------------------------------------------------------------
        cur.execute(
            """
            SELECT id, campaign, started_at
            FROM campaigns
            WHERE active = true
            ORDER BY started_at DESC
            LIMIT 1
            """
        )
        campaign = cur.fetchone()

        if not campaign:
            return {
                "success": True,
                "message": "OK",
                "payload": payload,
            }

        campaign_id = campaign["id"]

        payload["campaign"] = {
            "active": True,
            "campaign_id": campaign_id,
            "campaign": campaign["campaign"],
            "started_at": campaign["started_at"].isoformat(),
            "last_timebase_id": None,
        }

        # ------------------------------------------------------------
        # Fetch most recent TIMEBASE for this campaign
        # ------------------------------------------------------------
        cur.execute(
            """
            SELECT *
            FROM timebase
            WHERE campaign = %s
            ORDER BY created_at DESC
            LIMIT 1
            """,
            (campaign["campaign"],)
        )
        tb = cur.fetchone()

        if tb:
            payload["campaign"]["last_timebase_id"] = tb["id"]

            payload["timebase"] = {
                "id": tb["id"],
                "created_at": tb["created_at"].isoformat(),
                "gnss_time_utc": tb["gnss_time_utc"].isoformat(),
                "teensy_dwt_cycles": tb["teensy_dwt_cycles"],
                "teensy_dwt_ns": tb["teensy_dwt_ns"],
                "teensy_ocxo_ns": tb["teensy_ocxo_ns"],
                "teensy_rtc1_ns": tb["teensy_rtc1_ns"],
                "teensy_rtc2_ns": tb["teensy_rtc2_ns"],
                "pi_cycles": tb["pi_cycles"],
                "pi_ns": tb["pi_ns"],
            }
        else:
            payload["timebase"] = None

    return {
        "success": True,
        "message": "OK",
        "payload": payload,
    }


COMMANDS = {
    "START": cmd_start,
    "STOP": cmd_stop,
    "REPORT": cmd_report,
}

# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    setup_logging()

    # Start PPS ISR thread
    threading.Thread(
        target=_pps_listener,
        daemon=True,
        name="pps-listener",
    ).start()

    # Start server
    server_setup(
        subsystem="CLOCKS",
        commands=COMMANDS,
        subscriptions={
            "TIMEBASE_FRAGMENT": on_timebase_fragment,
        },
    )


if __name__ == "__main__":
    run()
