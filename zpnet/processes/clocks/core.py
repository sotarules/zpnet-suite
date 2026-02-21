"""
ZPNet CLOCKS Process — TIMEBASE Authority (Pi-side)

Responsibilities:
  * Receive TIMEBASE_FRAGMENT from Teensy
  * Query PITIMER for Pi arch timer capture (CNTVCT_EL0)
  * Augment fragment with Pi capture, GNSS time, and system time
  * Publish TIMEBASE
  * Persist TIMEBASE to PostgreSQL (append-only, best-effort)
  * Compute per-clock residual statistics (Welford's algorithm)
  * Denormalize augmented report into active campaign payload
  * Recover clocks after restart if campaign is active
  * Command GNSS into Time Only mode when campaign has a location

Three tracked clocks:
  * GNSS: 10 MHz VCLK counted in Teensy ISR (~100 ns quantization)
  * DWT: 600 MHz Cortex-M7 cycle counter captured in Teensy ISR
  * Pi: 54 MHz CNTVCT_EL0 (ARM Generic Timer Virtual Count Register)
    captured by pps_poll on an isolated core via chrony-disciplined
    second-boundary polling (~400 ns stddev, ~15 ns detection latency)

Pi clock architecture:
  * chrony disciplines the system clock to ~39 ns of GNSS via PPS
  * A dedicated C program (pps_poll) runs on isolated core 3,
    busy-polling clock_gettime(CLOCK_REALTIME)
  * The instant a new second is detected, pps_poll captures
    CNTVCT_EL0 via a single MRS instruction
  * The PITIMER process wraps pps_poll and exposes captures via
    the standard ZPNet command socket interface
  * CLOCKS queries PITIMER.REPORT when each TIMEBASE_FRAGMENT
    arrives, obtaining the latest counter value
  * Pi residual = delta between consecutive captures minus the
    expected 54,000,000 ticks, converted to nanoseconds

Residual policy (all clocks):
  * Count actual ticks/nanoseconds between consecutive PPS edges
  * Subtract expected count for that clock's frequency
  * Express residual in nanoseconds
  * GNSS: Teensy ISR, residual vs 1e9 ns
  * DWT: Teensy ISR, residual vs 1e9 ns
  * Pi: pps_poll on isolated core, residual vs 54e6 ticks,
    scaled to nanoseconds by display_scale (ns_per_tick)
  * Running statistics via Welford's online algorithm

Campaign start:
  * cmd_start arms the Teensy (CLOCKS.START)
  * The Teensy zeros all clocks at the next PPS boundary
  * The Pi sets _request_start, consumed on next fragment arrival
  * Pi tick epoch is captured from the first PITIMER report

Recovery after restart:
  * On startup, if an active campaign exists with TIMEBASE rows
    in Postgres, the Pi orchestrates clock recovery
  * Patiently wait for GNSS to acquire satellites
  * Compute elapsed seconds since last TIMEBASE
  * Project all clock values forward using tau
  * Send RECOVER command to Teensy with projected values
  * Compute synthetic Pi tick epoch for continuous pi_ns
  * Resume normal TIMEBASE augmentation

Campaign report denormalization:
  * After each TIMEBASE, the active campaign row's JSONB payload
    is updated with a "report" key containing:
      - Raw timebase fields
      - Campaign metadata (name, seconds, HH:MM:SS)
      - Per-clock statistics: ns_now, tau, ppb, residual stats
  * cmd_report simply reads and returns the campaign payload
  * No active campaign -> empty OK response

Semantics:
  * No smoothing, inference, or filtering
  * TIMEBASE records are sacred and immutable
  * Derivative statistics (tau, ppb, residuals) live only in
    the campaign report -- never in the TIMEBASE row itself
"""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from datetime import datetime, timezone
from typing import Dict, Any, Optional

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

# Expected nanoseconds per PPS for all clock domains.
NS_PER_SECOND = 1_000_000_000

# ARM Generic Timer frequency (54 MHz on Pi 4/5).
# Used for Pi residual computation: expected ticks per second.
PI_TIMER_FREQ = 54_000_000
PI_NS_PER_TICK = 1e9 / PI_TIMER_FREQ  # ~18.519 ns

# Teensy DWT runs at 600 MHz.
# dwt_cycles = dwt_ns * 3 / 5   (inverse of ns = cycles * 5 / 3)
DWT_CYCLES_PER_NS_NUM = 3
DWT_CYCLES_PER_NS_DEN = 5

# How often to poll GNSS while waiting for a valid fix (seconds)
GNSS_POLL_INTERVAL = 5

# How often to log a "still waiting" message (seconds)
GNSS_WAIT_LOG_INTERVAL = 60

# ---------------------------------------------------------------------
# PPS residual tracking -- Welford's online algorithm
#
# Used for all three clock domains: GNSS, DWT, Pi.
#
# GNSS and DWT: accumulate in nanoseconds with expected = 1e9.
# Pi: accumulates in raw ticks with expected = PI_TIMER_FREQ.
#     display_scale converts ticks -> nanoseconds for reporting.
# ---------------------------------------------------------------------

class _PpsResidual:
    """Per-clock PPS residual state with Welford's online statistics."""

    __slots__ = (
        "last_value",
        "delta",
        "residual",
        "valid",
        "n",
        "mean",
        "m2",
        "display_scale",
    )

    def __init__(self, display_scale: float = 1.0) -> None:
        self.display_scale = display_scale
        self.reset()

    def reset(self) -> None:
        self.last_value: int = 0
        self.delta: int = 0
        self.residual: int = 0
        self.valid: bool = False
        self.n: int = 0
        self.mean: float = 0.0
        self.m2: float = 0.0

    def update(self, value_now: int, expected: int) -> None:
        """
        Update with a new clock reading.

        value_now and expected must be in the same units:
          - nanoseconds for GNSS and DWT (expected = 1e9)
          - raw ticks for Pi (expected = PI_TIMER_FREQ = 54e6)

        Conversion to nanoseconds happens in to_dict() via display_scale.
        """
        if self.last_value > 0:
            self.delta = value_now - self.last_value
            self.residual = self.delta - expected
            self.valid = True

            # Welford's online algorithm
            self.n += 1
            x = float(self.residual)
            d1 = x - self.mean
            self.mean += d1 / self.n
            d2 = x - self.mean
            self.m2 += d1 * d2
        else:
            self.delta = 0
            self.residual = 0
            self.valid = False

        self.last_value = value_now

    @property
    def variance(self) -> float:
        return (self.m2 / (self.n - 1)) if self.n >= 2 else 0.0

    @property
    def stddev(self) -> float:
        return math.sqrt(self.variance) if self.n >= 2 else 0.0

    @property
    def stderr(self) -> float:
        return (self.stddev / math.sqrt(self.n)) if self.n >= 2 else 0.0

    def to_dict(self) -> Dict[str, Any]:
        """
        Serialize residual state for the campaign report.

        All values are scaled to nanoseconds for consistent display.
        For GNSS and DWT (display_scale=1.0) this is a no-op.
        For Pi (display_scale=PI_NS_PER_TICK) this converts ticks -> ns.
        """
        s = self.display_scale
        return {
            "pps_valid": self.valid,
            "pps_delta": round(self.delta * s, 3),
            "pps_residual": round(self.residual * s, 3),
            "pps_n": self.n,
            "pps_mean": round(self.mean * s, 3),
            "pps_stddev": round(self.stddev * s, 3),
            "pps_stderr": round(self.stderr * s, 3),
        }

# ---------------------------------------------------------------------
# Local state (process lifetime)
# ---------------------------------------------------------------------

# Request flags -- set by cmd_start/cmd_stop, consumed by
# the fragment handler.
_request_start: bool = False
_request_stop: bool = False

# True while a campaign is actively acquiring.
_campaign_active: bool = False

# Pi tick epoch -- the CNTVCT_EL0 value at campaign start.
# Subtracted from raw counter to produce campaign-scoped ticks,
# which are then converted to nanoseconds for the TIMEBASE record.
_pi_tick_epoch: int = 0
_pi_tick_epoch_set: bool = False

# Per-clock PPS residual trackers.
# GNSS and DWT: nanoseconds, display_scale=1.0
# Pi: raw ticks, display_scale=PI_NS_PER_TICK
_residual_gnss = _PpsResidual()
_residual_dwt = _PpsResidual()
_residual_pi = _PpsResidual(display_scale=PI_NS_PER_TICK)

_state_lock = threading.Lock()

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

def _get_pitimer_capture() -> Optional[Dict[str, Any]]:
    """
    Fetch the latest Pi arch timer capture from the PITIMER process.

    Returns the PITIMER REPORT payload, or None if PITIMER is
    not available or has no capture yet.

    The payload contains:
      - counter: raw CNTVCT_EL0 value at last second boundary
      - delta: ticks since previous capture
      - residual: delta minus expected (PI_TIMER_FREQ)
      - residual_ns: residual in nanoseconds
      - detect_ns: detection latency (ns after second boundary)
      - seq: capture sequence number
      - freq: timer frequency (54 MHz)
      - ns_per_tick: nanoseconds per tick
    """
    try:
        resp = send_command(
            machine="PI",
            subsystem="PITIMER",
            command="REPORT",
        )
        payload = resp.get("payload", {})
        if payload.get("state") == "WAITING":
            return None
        return payload
    except Exception:
        logging.warning("⚠️ [clocks] PITIMER.REPORT failed -- Pi clock unavailable")
        return None


def _ticks_to_ns(ticks: int) -> int:
    """
    Convert a Pi tick count to nanoseconds.

    Uses integer arithmetic: ns = ticks * 1,000,000,000 // frequency
    At 54 MHz: 1 tick = 18.519 ns (truncated to integer).
    """
    return (ticks * NS_PER_SECOND) // PI_TIMER_FREQ


def _get_gnss_time() -> str:
    """
    Fetch last-known GNSS UTC time from GNSS process.
    Returns ISO8601 Z string.

    Raises KeyError if the GNSS process has no valid fix yet
    (date/time fields not present in REPORT payload).
    """
    payload = send_command(
        machine="PI",
        subsystem="GNSS",
        command="REPORT",
    )["payload"]

    date = payload["date"]
    time_s = payload["time"]
    return f"{date}T{time_s}Z"


def _wait_for_gnss_time() -> str:
    """
    Patiently wait for the GNSS receiver to produce a valid timestamp.

    After a cold start, the receiver may need minutes to acquire
    satellites and compute a fix.  This function polls the GNSS
    REPORT every GNSS_POLL_INTERVAL seconds and logs a "still
    waiting" message every GNSS_WAIT_LOG_INTERVAL seconds.

    Returns ISO8601 Z string once available.
    Never times out -- the math works regardless of how long
    the wait is.
    """
    wait_start = time.monotonic()
    last_log = wait_start

    while True:
        try:
            payload = send_command(
                machine="PI",
                subsystem="GNSS",
                command="REPORT",
            )["payload"]

            date = payload.get("date")
            time_s = payload.get("time")

            if date and time_s:
                return f"{date}T{time_s}Z"

        except Exception:
            pass

        now = time.monotonic()
        elapsed = now - wait_start

        if now - last_log >= GNSS_WAIT_LOG_INTERVAL:
            logging.info(
                "⏳ [recovery] waiting for GNSS time... (%.0fs elapsed)",
                elapsed,
            )
            last_log = now

        time.sleep(GNSS_POLL_INTERVAL)


def _get_active_campaign() -> Optional[Dict[str, Any]]:
    """
    Return active campaign row or None.

    Returns dict with keys: campaign, payload (JSONB).
    """
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT campaign, payload
            FROM campaigns
            WHERE active = true
            ORDER BY ts DESC
            LIMIT 1
            """
        )
        row = cur.fetchone()

    if row is None:
        return None

    payload = row["payload"]
    if isinstance(payload, str):
        payload = json.loads(payload)

    return {
        "campaign": row["campaign"],
        "payload": payload,
    }


def _set_gnss_mode_to(location: str) -> Dict[str, Any]:
    """Command GNSS process to switch to Time Only mode at a location."""
    return send_command(
        machine="PI",
        subsystem="GNSS",
        command="MODE",
        args={"mode": "TO", "location": location},
    )


def _set_gnss_mode_normal() -> Dict[str, Any]:
    """Command GNSS process to return to normal (CSS) mode."""
    return send_command(
        machine="PI",
        subsystem="GNSS",
        command="MODE",
        args={"mode": "NORMAL"},
    )


# ---------------------------------------------------------------------
# Clock statistics helpers
# ---------------------------------------------------------------------

def _seconds_to_hms(seconds: int) -> str:
    """Format integer seconds as HH:MM:SS."""
    h = seconds // 3600
    m = (seconds % 3600) // 60
    s = seconds % 60
    return f"{h:02d}:{m:02d}:{s:02d}"


def _compute_tau(clock_ns: int, gnss_ns: int) -> float:
    """Dimensionless ratio: clock_ns / gnss_ns."""
    if gnss_ns == 0:
        return 0.0
    return clock_ns / gnss_ns


def _compute_ppb(clock_ns: int, gnss_ns: int) -> float:
    """Deviation from GNSS in parts-per-billion."""
    if gnss_ns == 0:
        return 0.0
    return ((clock_ns - gnss_ns) / gnss_ns) * 1e9


def _build_clock_block(
    name: str,
    ns_now: int,
    gnss_ns: int,
    residual: _PpsResidual,
) -> Dict[str, Any]:
    """
    Build a per-clock statistics block for the campaign report.

    Contains: ns_now, tau, ppb, and all residual statistics.
    All residual stats are in nanoseconds.
    """
    block: Dict[str, Any] = {
        "ns_now": ns_now,
        "tau": round(_compute_tau(ns_now, gnss_ns), 12),
        "ppb": round(_compute_ppb(ns_now, gnss_ns), 3),
    }
    block.update(residual.to_dict())
    return block


# ---------------------------------------------------------------------
# Campaign report builder
# ---------------------------------------------------------------------

def _build_report(
    campaign_name: str,
    campaign_payload: Dict[str, Any],
    timebase: Dict[str, Any],
) -> Dict[str, Any]:
    """
    Build the augmented campaign report from a TIMEBASE record
    and current residual statistics.

    This is denormalized into the campaign's JSONB payload as
    the "report" key, and is what cmd_report returns verbatim.
    """
    gnss_ns = int(timebase.get("teensy_gnss_ns") or 0)
    dwt_ns = int(timebase.get("teensy_dwt_ns") or 0)
    pi_ns = int(timebase.get("pi_ns") or 0)

    # Campaign elapsed time derived from GNSS nanoseconds
    # (GNSS is truth -- use it as the elapsed-time reference)
    campaign_seconds = gnss_ns // NS_PER_SECOND if gnss_ns > 0 else 0

    report: Dict[str, Any] = {
        # Campaign metadata
        "campaign": campaign_name,
        "campaign_state": "STARTED",
        "campaign_seconds": campaign_seconds,
        "campaign_elapsed": _seconds_to_hms(campaign_seconds),
        "location": campaign_payload.get("location"),

        # Timestamps
        "gnss_time_utc": timebase.get("gnss_time_utc"),
        "system_time_utc": timebase.get("system_time_utc"),

        # PPS count
        "teensy_pps_count": timebase.get("teensy_pps_count"),

        # Raw clock values (for forensics / recovery)
        "teensy_dwt_cycles": timebase.get("teensy_dwt_cycles"),
        "teensy_ocxo_ns": timebase.get("teensy_ocxo_ns"),
        "teensy_rtc1_ns": timebase.get("teensy_rtc1_ns"),
        "teensy_rtc2_ns": timebase.get("teensy_rtc2_ns"),
        "pi_counter": timebase.get("pi_counter"),

        # Per-clock statistics (all residual stats in nanoseconds)
        "gnss": _build_clock_block("GNSS", gnss_ns, gnss_ns, _residual_gnss),
        "dwt": _build_clock_block("DWT", dwt_ns, gnss_ns, _residual_dwt),
        "pi": _build_clock_block("PI", pi_ns, gnss_ns, _residual_pi),
    }

    return report


# ---------------------------------------------------------------------
# Persistence
# ---------------------------------------------------------------------

def _persist_timebase(tb: Dict[str, Any], report: Dict[str, Any]) -> None:
    """
    Best-effort TIMEBASE insert + report denormalization.

    Failure is logged and ignored.
    """
    try:
        with open_db() as conn:
            cur = conn.cursor()

            cur.execute(
                """
                INSERT INTO timebase (campaign, payload)
                VALUES (%s, %s)
                """,
                (tb["campaign"], json.dumps(tb)),
            )

            cur.execute(
                """
                UPDATE campaigns
                SET payload = payload || jsonb_build_object('report', %s::jsonb)
                WHERE campaign = %s
                  AND active = true
                """,
                (json.dumps(report), tb["campaign"]),
            )

    except Exception:
        logging.exception("⚠️ [clocks] failed to persist TIMEBASE (ignored)")


# ---------------------------------------------------------------------
# Recovery -- restore clocks after Pi restart mid-campaign
# ---------------------------------------------------------------------

def _recover_campaign() -> None:
    """
    Detect and recover an in-progress campaign after Pi restart.

    Called once at startup, before normal operation begins.

    Algorithm:
      1) Check for active campaign in Postgres
      2) Fetch the most recent TIMEBASE row for that campaign
      3) Wait patiently for GNSS to produce a valid timestamp
      4) If campaign has a location, restore GNSS -> TO mode
      5) Capture GNSS time and PITIMER counter
      6) Compute elapsed seconds since last TIMEBASE
      7) Compute tau for each clock domain
      8) Project all clock values forward to the NEXT PPS edge
      9) Send RECOVER to Teensy with projected values
     10) Compute synthetic Pi tick epoch for continuous pi_ns
     11) Activate campaign
    """
    global _campaign_active, _pi_tick_epoch, _pi_tick_epoch_set

    # ----------------------------------------------------------
    # 1) Check for active campaign
    # ----------------------------------------------------------
    row = _get_active_campaign()
    if row is None:
        logging.info("🔍 [recovery] no active campaign -- nothing to recover")
        return

    campaign_name = row["campaign"]
    campaign_payload = row["payload"]
    campaign_location = campaign_payload.get("location")
    logging.info(
        "🔍 [recovery] active campaign found: '%s' (location: %s)",
        campaign_name,
        campaign_location or "none",
    )

    # ----------------------------------------------------------
    # 2) Fetch most recent TIMEBASE for this campaign
    # ----------------------------------------------------------
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY ts DESC
            LIMIT 1
            """,
            (campaign_name,),
        )
        tb_row = cur.fetchone()

    if tb_row is None:
        logging.warning(
            "⚠️ [recovery] campaign '%s' has no TIMEBASE rows -- cannot recover",
            campaign_name,
        )
        return

    last_tb = tb_row["payload"]
    if isinstance(last_tb, str):
        last_tb = json.loads(last_tb)

    last_gnss_utc_str = last_tb["gnss_time_utc"]
    last_gnss_utc = datetime.fromisoformat(
        last_gnss_utc_str.replace("Z", "+00:00")
    )
    last_dwt_ns = int(last_tb["teensy_dwt_ns"])
    last_gnss_ns = int(last_tb.get("teensy_gnss_ns") or 0)
    last_ocxo_ns = int(last_tb.get("teensy_ocxo_ns") or 0)
    last_pi_ns = int(last_tb.get("pi_ns") or 0)

    logging.info(
        "🔍 [recovery] last TIMEBASE: gnss_utc=%s  gnss_ns=%d  dwt_ns=%d  ocxo_ns=%d  pi_ns=%d",
        last_gnss_utc.isoformat(),
        last_gnss_ns,
        last_dwt_ns,
        last_ocxo_ns,
        last_pi_ns,
    )

    # If gnss_ns is not available, fall back to dwt_ns
    if last_gnss_ns == 0:
        logging.warning(
            "⚠️ [recovery] teensy_gnss_ns not available in TIMEBASE -- "
            "using dwt_ns as GNSS proxy (tau_dwt assumed ~ 1.0)"
        )
        last_gnss_ns = last_dwt_ns

    # ----------------------------------------------------------
    # 3) Wait for GNSS to produce a valid timestamp
    # ----------------------------------------------------------
    logging.info("⏳ [recovery] waiting for GNSS time to become available...")

    gnss_time_preflight = _wait_for_gnss_time()

    logging.info(
        "✅ [recovery] GNSS time available: %s",
        gnss_time_preflight,
    )

    # ----------------------------------------------------------
    # 4) If campaign has a location, restore GNSS -> TO mode
    # ----------------------------------------------------------
    if campaign_location:
        logging.info(
            "📡 [recovery] restoring GNSS -> TO mode for location '%s'",
            campaign_location,
        )
        try:
            gnss_resp = _set_gnss_mode_to(campaign_location)
            if gnss_resp.get("success"):
                logging.info("✅ [recovery] GNSS confirmed TO mode")
            else:
                logging.warning(
                    "⚠️ [recovery] GNSS MODE=TO failed: %s",
                    gnss_resp.get("message", "?"),
                )
        except Exception:
            logging.exception("⚠️ [recovery] GNSS MODE=TO IPC failed (continuing)")

    # ----------------------------------------------------------
    # 5) Capture GNSS time and PITIMER counter
    # ----------------------------------------------------------
    gnss_time_now_str = _get_gnss_time()
    gnss_time_now = datetime.fromisoformat(
        gnss_time_now_str.replace("Z", "+00:00")
    )

    pi_capture = _get_pitimer_capture()
    pi_counter_now = pi_capture["counter"] if pi_capture else 0

    logging.info(
        "⏱️ [recovery] GNSS time now: %s  Pi counter: %d",
        gnss_time_now_str,
        pi_counter_now,
    )

    # ----------------------------------------------------------
    # 6) Compute elapsed seconds since last TIMEBASE
    # ----------------------------------------------------------
    elapsed_td = gnss_time_now - last_gnss_utc
    elapsed_seconds = int(elapsed_td.total_seconds())

    if elapsed_seconds <= 0:
        logging.error(
            "❌ [recovery] elapsed seconds is %d -- time went backwards?",
            elapsed_seconds,
        )
        return

    logging.info(
        "📐 [recovery] elapsed since last TIMEBASE: %d seconds",
        elapsed_seconds,
    )

    # ----------------------------------------------------------
    # 7) Compute tau for each clock domain
    # ----------------------------------------------------------
    tau_dwt = float(last_dwt_ns) / float(last_gnss_ns) if last_gnss_ns > 0 else 1.0
    tau_ocxo = float(last_ocxo_ns) / float(last_gnss_ns) if (last_gnss_ns > 0 and last_ocxo_ns > 0) else 1.0
    tau_pi = float(last_pi_ns) / float(last_gnss_ns) if (last_gnss_ns > 0 and last_pi_ns > 0) else 1.0

    logging.info(
        "📐 [recovery] tau -- dwt=%.12f  ocxo=%.12f  pi=%.12f",
        tau_dwt,
        tau_ocxo,
        tau_pi,
    )

    # ----------------------------------------------------------
    # 8) Project clock values to the NEXT PPS edge
    # ----------------------------------------------------------
    project_seconds = elapsed_seconds + 1

    projected_gnss_ns = last_gnss_ns + (project_seconds * NS_PER_SECOND)
    projected_dwt_ns = last_dwt_ns + int(project_seconds * NS_PER_SECOND * tau_dwt)
    projected_ocxo_ns = last_ocxo_ns + int(project_seconds * NS_PER_SECOND * tau_ocxo) if last_ocxo_ns > 0 else 0
    projected_pi_ns = last_pi_ns + int(project_seconds * NS_PER_SECOND * tau_pi) if last_pi_ns > 0 else 0

    projected_dwt_cycles = (projected_dwt_ns * DWT_CYCLES_PER_NS_NUM) // DWT_CYCLES_PER_NS_DEN

    logging.info(
        "📐 [recovery] projected to PPS+1 -- gnss_ns=%d  dwt_cycles=%d  dwt_ns=%d  ocxo_ns=%d  pi_ns=%d",
        projected_gnss_ns,
        projected_dwt_cycles,
        projected_dwt_ns,
        projected_ocxo_ns,
        projected_pi_ns,
    )

    # ----------------------------------------------------------
    # 9) Send RECOVER to Teensy
    # ----------------------------------------------------------
    logging.info("🚀 [recovery] sending RECOVER to Teensy...")

    try:
        recover_resp = send_command(
            machine="TEENSY",
            subsystem="CLOCKS",
            command="RECOVER",
            args={
                "dwt_cycles": str(projected_dwt_cycles),
                "gnss_ns": str(projected_gnss_ns),
                "ocxo_ns": str(projected_ocxo_ns),
            },
        )

        if recover_resp.get("success", True):
            logging.info("✅ [recovery] Teensy RECOVER accepted")
        else:
            logging.warning(
                "⚠️ [recovery] Teensy RECOVER response: %s",
                recover_resp.get("message", "?"),
            )
    except Exception:
        logging.exception("❌ [recovery] Teensy RECOVER IPC failed")
        return

    # ----------------------------------------------------------
    # 10) Compute synthetic Pi tick epoch for continuous pi_ns
    # ----------------------------------------------------------
    if pi_counter_now > 0 and projected_pi_ns > 0:
        projected_pi_ticks = (projected_pi_ns * PI_TIMER_FREQ) // NS_PER_SECOND
        _pi_tick_epoch = pi_counter_now - projected_pi_ticks
        _pi_tick_epoch_set = True
        logging.info(
            "📐 [recovery] Pi tick epoch set: %d (projected pi_ticks=%d)",
            _pi_tick_epoch,
            projected_pi_ticks,
        )
    else:
        logging.warning(
            "⚠️ [recovery] PITIMER not available -- Pi tick epoch will be set on first capture"
        )

    # ----------------------------------------------------------
    # 11) Activate campaign
    # ----------------------------------------------------------
    _residual_gnss.reset()
    _residual_dwt.reset()
    _residual_pi.reset()
    _campaign_active = True

    logging.info(
        "✅ [recovery] campaign '%s' recovered -- "
        "Teensy RECOVER sent, awaiting first TIMEBASE_FRAGMENT",
        campaign_name,
    )


# ---------------------------------------------------------------------
# Teensy fragment handler
# ---------------------------------------------------------------------

def on_timebase_fragment(payload: Payload) -> None:
    """
    Receive TIMEBASE_FRAGMENT from Teensy and build TIMEBASE.

    This is the main data path.  Each fragment triggers:
      1) Pi-side observations (system time, PITIMER capture)
      2) TIMEBASE augmentation (combine Teensy + Pi + GNSS)
      3) Residual statistics update (GNSS, DWT, Pi)
      4) Report denormalization into campaign payload
      5) Publish + persist
    """
    global _request_start, _request_stop, _campaign_active
    global _pi_tick_epoch, _pi_tick_epoch_set

    # ----------------------------------------------------------
    # Capture Pi-side observations immediately
    # ----------------------------------------------------------
    system_time_utc = datetime.now(timezone.utc)
    pi_capture = _get_pitimer_capture()

    # ----------------------------------------------------------
    # Process request flags
    # ----------------------------------------------------------
    if _request_stop:
        _campaign_active = False
        _request_stop = False
        logging.info("⏹️ [clocks] campaign stopped")

    if _request_start:
        _residual_gnss.reset()
        _residual_dwt.reset()
        _residual_pi.reset()
        _pi_tick_epoch_set = False
        _campaign_active = True
        _request_start = False
        logging.info("▶️ [clocks] campaign started")

    if not _campaign_active:
        return

    system_time_str = system_time_utc.isoformat(timespec='microseconds')

    # ----------------------------------------------------------
    # Extract Pi counter and compute campaign-scoped pi_ns
    # ----------------------------------------------------------
    pi_counter = pi_capture["counter"] if pi_capture else None
    pi_ns = 0

    if pi_counter is not None:
        if not _pi_tick_epoch_set:
            _pi_tick_epoch = pi_counter
            _pi_tick_epoch_set = True
            logging.info("⏱️ [clocks] Pi tick epoch set: %d", _pi_tick_epoch)

        campaign_ticks = pi_counter - _pi_tick_epoch
        pi_ns = _ticks_to_ns(campaign_ticks)

        # Update Pi residual (raw ticks, expected = PI_TIMER_FREQ)
        _residual_pi.update(pi_counter, PI_TIMER_FREQ)

    # ----------------------------------------------------------
    # Build TIMEBASE record
    # ----------------------------------------------------------
    frag = payload

    row = _get_active_campaign()
    if row is None:
        logging.warning("⚠️ [clocks] fragment received with no active campaign")
        return

    campaign = row["campaign"]
    campaign_payload = row["payload"]
    gnss_time = _get_gnss_time()

    timebase = {
        "campaign": campaign,

        # System time when fragment arrived (chrony-disciplined, sanity check)
        "system_time_utc": system_time_str,

        # GNSS time from receiver (integer-second, via NMEA)
        "gnss_time_utc": gnss_time,

        # Teensy clocks (authoritative)
        "teensy_dwt_cycles": frag["dwt_cycles"],
        "teensy_dwt_ns": frag["dwt_ns"],
        "teensy_gnss_ns": frag["gnss_ns"],
        "teensy_ocxo_ns": frag.get("ocxo_ns"),
        "teensy_rtc1_ns": frag.get("rtc1_ns"),
        "teensy_rtc2_ns": frag.get("rtc2_ns"),

        # Pi clock (CNTVCT_EL0 via PITIMER)
        "pi_counter": pi_counter,
        "pi_ns": pi_ns,

        # Per-clock PPS residuals (nanoseconds)
        "isr_residual_gnss": frag.get("isr_residual_gnss"),
        "isr_residual_dwt": frag.get("isr_residual_dwt"),
        "isr_residual_ocxo": frag.get("isr_residual_ocxo"),
        "isr_residual_pi": round(_residual_pi.residual * PI_NS_PER_TICK, 3) if _residual_pi.valid else None,

        # Teensy PPS count
        "teensy_pps_count": frag.get("teensy_pps_count"),
    }

    # ----------------------------------------------------------
    # Update GNSS and DWT residual statistics (nanoseconds)
    # (Pi residual was already updated above from PITIMER capture)
    # ----------------------------------------------------------
    gnss_ns = int(frag.get("gnss_ns") or 0)
    dwt_ns = int(frag.get("dwt_ns") or 0)

    if gnss_ns > 0:
        _residual_gnss.update(gnss_ns, NS_PER_SECOND)
    if dwt_ns > 0:
        _residual_dwt.update(dwt_ns, NS_PER_SECOND)

    # ----------------------------------------------------------
    # Build augmented report
    # ----------------------------------------------------------
    report = _build_report(campaign, campaign_payload, timebase)

    # Publish sacred tuple
    publish("TIMEBASE", timebase)

    # Persist best-effort (also denormalizes report into campaign)
    _persist_timebase(timebase, report)


# ---------------------------------------------------------------------
# Command handlers (control plane)
# ---------------------------------------------------------------------

def cmd_start(args: Optional[dict]) -> dict:
    """
    Start a new campaign.

    Args:
        campaign (str):   Campaign name (required)
        location (str):   Profiled location name (optional)

    Sequence:
      1) Deactivate any existing campaign
      2) Insert new campaign row
      3) If location specified: command GNSS -> TO mode (blocking)
      4) Set _request_start flag (consumed on next fragment)
      5) Start Teensy CLOCKS acquisition (Teensy zeros at PPS)
    """
    global _request_start, _request_stop

    campaign = args.get("campaign")
    if not campaign:
        return {"success": False, "message": "START requires 'campaign' argument"}

    location = (args.get("location") or "").strip() or None

    # ----------------------------------------------------------
    # 1) Deactivate existing + insert new campaign
    # ----------------------------------------------------------
    campaign_payload = {
        "location": location,
        "started_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
    }

    with open_db() as conn:
        cur = conn.cursor()

        cur.execute(
            """
            UPDATE campaigns
            SET active = false,
                payload = payload || jsonb_build_object(
                    'stopped_at', to_jsonb(%s::text)
                )
            WHERE active = true
            """,
            (datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),),
        )

        cur.execute(
            """
            INSERT INTO campaigns (campaign, active, payload)
            VALUES (%s, true, %s)
            """,
            (campaign, json.dumps(campaign_payload)),
        )

    # ----------------------------------------------------------
    # 2) If location specified, switch GNSS to Time Only mode
    # ----------------------------------------------------------
    if location:
        logging.info(
            "📡 [clocks] commanding GNSS -> TO mode for location '%s'",
            location,
        )

        try:
            gnss_resp = _set_gnss_mode_to(location)
        except Exception:
            logging.exception("💥 [clocks] GNSS MODE=TO IPC failed")
            return {
                "success": False,
                "message": f"failed to command GNSS to TO mode for '{location}'",
            }

        if not gnss_resp.get("success"):
            msg = gnss_resp.get("message", "unknown error")
            logging.warning("⚠️ [clocks] GNSS MODE=TO failed: %s", msg)
            return {
                "success": False,
                "message": f"GNSS MODE=TO failed: {msg}",
                "payload": {"gnss_response": gnss_resp},
            }

        logging.info("✅ [clocks] GNSS confirmed TO mode")

    # ----------------------------------------------------------
    # 3) Arm start -- consumed on next TIMEBASE_FRAGMENT
    # ----------------------------------------------------------
    _request_stop = False
    _request_start = True

    # ----------------------------------------------------------
    # 4) Start Teensy CLOCKS acquisition
    # ----------------------------------------------------------
    send_command(
        machine="TEENSY",
        subsystem="CLOCKS",
        command="START",
        args={"campaign": campaign},
    )

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "campaign": campaign,
            "location": location,
        },
    }


def cmd_stop(_: Optional[dict]) -> dict:
    """
    Stop active campaign.

    Sequence:
      1) Read active campaign (to check if it had a location)
      2) Deactivate campaign in DB
      3) Set _request_stop flag (applied on next fragment)
      4) Stop Teensy CLOCKS acquisition
      5) If campaign had a location: restore GNSS -> NORMAL (CSS)
    """
    global _request_start, _request_stop

    row = _get_active_campaign()
    had_location = row["payload"].get("location") if row else None

    stopped_at = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")

    with open_db() as conn:
        cur = conn.cursor()
        cur.execute(
            """
            UPDATE campaigns
            SET active = false,
                payload = payload
                    || jsonb_build_object('stopped_at', to_jsonb(%s::text))
                    || CASE
                        WHEN payload ? 'report'
                        THEN jsonb_build_object(
                            'report',
                            (payload->'report') || '{"campaign_state":"STOPPED"}'::jsonb
                        )
                        ELSE '{}'::jsonb
                       END
            WHERE active = true
            """,
            (stopped_at,),
        )

    _request_start = False
    _request_stop = True

    send_command(
        machine="TEENSY",
        subsystem="CLOCKS",
        command="STOP",
    )

    if had_location:
        logging.info(
            "📡 [clocks] restoring GNSS -> NORMAL (campaign had location '%s')",
            had_location,
        )

        try:
            gnss_resp = _set_gnss_mode_normal()

            if not gnss_resp.get("success"):
                logging.warning(
                    "⚠️ [clocks] GNSS MODE=NORMAL failed: %s",
                    gnss_resp.get("message", "?"),
                )
        except Exception:
            logging.exception("⚠️ [clocks] GNSS MODE=NORMAL IPC failed (ignored)")

    return {"success": True, "message": "OK"}


def cmd_report(_: Optional[dict]) -> Dict[str, Any]:
    """
    Report current CLOCKS state.

    Simply reads and returns the active campaign's payload,
    which contains the denormalized report built each second
    by on_timebase_fragment.

    If no campaign is active, returns an empty OK response
    indicating clocks are idle.
    """

    row = _get_active_campaign()

    if not row:
        return {
            "success": True,
            "message": "OK",
            "payload": {
                "campaign_state": "IDLE",
            },
        }

    return {
        "success": True,
        "message": "OK",
        "payload": row["payload"],
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

    logging.info(
        "🕐 [clocks] Three tracked clocks: GNSS (Teensy 10MHz), "
        "DWT (Teensy 600MHz), Pi (CNTVCT_EL0 54MHz via PITIMER). "
        "chrony-disciplined system time for sanity checking."
    )

    # ----------------------------------------------------------
    # Recovery check -- before anything else, see if we need to
    # resume a campaign that was running when we last restarted.
    # ----------------------------------------------------------
    try:
        _recover_campaign()
    except Exception:
        logging.exception("❌ [recovery] unhandled exception during recovery")

    # ----------------------------------------------------------
    # Start server -- TIMEBASE_FRAGMENT subscription is the
    # primary data path.
    # ----------------------------------------------------------
    server_setup(
        subsystem="CLOCKS",
        commands=COMMANDS,
        subscriptions={
            "TIMEBASE_FRAGMENT": on_timebase_fragment,
        },
    )


if __name__ == "__main__":
    run()