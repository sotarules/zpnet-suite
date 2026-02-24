"""
ZPNet CLOCKS Process — TIMEBASE Authority (Pi-side)

Responsibilities:
  * Receive TIMEBASE_FRAGMENT from Teensy
  * Fetch correlated Pi capture from PITIMER by system clock key
  * Augment fragment with Pi capture, GNSS time, and system time
  * Publish TIMEBASE
  * Persist TIMEBASE to PostgreSQL (append-only, best-effort)
  * Compute per-clock residual statistics (Welford's algorithm)
  * Denormalize augmented report into active campaign payload
  * Recover clocks after restart if campaign is active
  * Command GNSS into Time Only mode when campaign has a location

Four tracked clocks:
  * GNSS: 10 MHz VCLK counted in Teensy ISR (~100 ns quantization)
  * DWT: 600 MHz Cortex-M7 cycle counter captured in Teensy ISR
  * Pi: 54 MHz CNTVCT_EL0 (ARM Generic Timer Virtual Count Register)
    captured by pps_poll on an isolated core via chrony-disciplined
    second-boundary polling (~400 ns stddev, ~15 ns detection latency)
  * OCXO: 10 MHz AOCJY1-A oven-controlled crystal oscillator counted
    via Teensy GPT1 external clock input (~100 ns quantization)

Pi clock correlation model:
  * Both PITIMER and CLOCKS derive their correlation key from
    the chrony-disciplined system clock via system_time_z()
  * pps_poll detects the second boundary from clock_gettime and
    stores the capture keyed by system_time_z() — within
    milliseconds of PPS
  * When a TIMEBASE_FRAGMENT arrives from Teensy (tens of ms
    after PPS, due to USB serial latency), CLOCKS calls
    system_time_z() to get the same key, then fetches the
    matching capture from PITIMER via REPORT(gnss_time=<key>)
  * Both sides read the same local clock within the same UTC
    second — no IPC to GNSS, no NMEA cache race, no latency
    window where the key could change
  * A match failure is a hard fault — CLOCKS logs a warning
    with full PITIMER diagnostics and skips that TIMEBASE

  NOTE: Previous versions called GNSS.GET_TIME for the correlation
  key.  The NMEA cache would be overwritten with the next second's
  forecast before CLOCKS could read it, causing systematic one-
  second mismatches.  Using the system clock eliminates this
  entirely.

Pi TDC diagnostics in TIMEBASE:
  * diag_pi_raw_counter     — raw CNTVCT_EL0 at detection moment
  * diag_pi_corrected       — counter minus detection latency
  * diag_pi_detect_ns       — polling detection latency (ns)
  * diag_pi_correction_ticks — ticks subtracted from raw counter
  * diag_pi_seq             — pps_poll capture sequence number
  * diag_pi_delta           — ticks between consecutive corrected captures
  * diag_pi_residual        — delta minus expected frequency (ticks)
  * diag_pi_residual_ns     — residual in nanoseconds (from pps_poll)

Residual policy (all clocks):
  * Count actual ticks/nanoseconds between consecutive PPS edges
  * Subtract expected count for that clock's frequency
  * Express residual in nanoseconds
  * GNSS: Teensy ISR, residual vs 1e9 ns
  * DWT: Teensy ISR, residual vs 1e9 ns
  * OCXO: Teensy ISR, residual vs 1e9 ns (100 ns quantization)
  * Pi: correlated capture from PITIMER, residual vs 54e6 ticks
    (using CORRECTED counter for best accuracy),
    scaled to nanoseconds by display_scale (ns_per_tick)
  * Running statistics via Welford's online algorithm
  * Outlier rejection on all clock domains: any delta that
    deviates from expected by more than 50% is rejected and
    logged, preventing a single glitch from permanently
    corrupting the running statistics

Campaign start:
  * cmd_start arms the Teensy (CLOCKS.START)
  * The Teensy zeros all clocks at the next PPS boundary
  * The Pi sets _request_start, consumed on next fragment arrival
  * Pi tick epoch is captured from the first correlated PITIMER capture

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
from zpnet.shared.util import system_time_z

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
# Used for all four clock domains: GNSS, DWT, OCXO, Pi.
#
# GNSS, DWT, OCXO: accumulate in nanoseconds with expected = 1e9.
# Pi: accumulates in raw ticks with expected = PI_TIMER_FREQ.
#     display_scale converts ticks -> nanoseconds for reporting.
#
# All clock domains use outlier rejection: any delta that deviates
# from expected by more than 50% is rejected and logged, preventing
# a single glitch (recovery discontinuity, skipped PPS, Teensy
# hiccup) from permanently corrupting the running statistics.
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
          - nanoseconds for GNSS, DWT, and OCXO (expected = 1e9)
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
        For GNSS, DWT, and OCXO (display_scale=1.0) this is a no-op.
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
# Outlier-safe residual update
# ---------------------------------------------------------------------

def _safe_residual_update(
    tracker: _PpsResidual,
    value_now: int,
    expected: int,
    label: str,
) -> None:
    """
    Update a residual tracker with outlier rejection.

    If the tracker already has a previous value, peek at the
    prospective delta before committing to Welford's.  If the
    delta deviates from expected by more than 50%, reject the
    sample, log a warning, and advance the anchor so the next
    delta starts clean.

    If this is the first sample (last_value == 0), just pass
    through to update() which will set the anchor without
    computing a delta.
    """
    if tracker.last_value > 0:
        prospective_delta = value_now - tracker.last_value
        deviation = abs(prospective_delta - expected)
        if deviation > expected // 2:
            logging.warning(
                "⚠️ [clocks] %s residual outlier rejected: "
                "delta=%d (expected %d, deviation %d)",
                label, prospective_delta, expected, deviation,
            )
            tracker.last_value = value_now
            return
    tracker.update(value_now, expected)


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
# NOTE: We use the CORRECTED counter for epoch and all downstream
# computations, so the Pi-side TDC correction is baked in.
_pi_tick_epoch: int = 0
_pi_tick_epoch_set: bool = False

# Per-clock PPS residual trackers.
# GNSS and DWT: nanoseconds, display_scale=1.0
# OCXO: nanoseconds (100 ns quantization from 10 MHz), display_scale=1.0
# Pi: raw ticks (CORRECTED), display_scale=PI_NS_PER_TICK
_residual_gnss = _PpsResidual()
_residual_dwt = _PpsResidual()
_residual_pi = _PpsResidual(display_scale=PI_NS_PER_TICK)
_residual_ocxo = _PpsResidual()

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

def _get_pitimer_capture_by_time(time_key: str) -> Optional[Dict[str, Any]]:
    """
    Fetch the Pi arch timer capture for a specific time key from PITIMER.

    This is the primary correlation path.  PITIMER maintains a ring
    buffer of captures indexed by system_time_z().  We request the
    capture matching the same UTC second as this fragment.

    Both PITIMER and CLOCKS derive the key from the same chrony-
    disciplined system clock, so the keys are guaranteed to match
    as long as both sides are within the same UTC second — which
    they always are (pps_poll stores within ms of PPS, fragment
    arrives tens of ms later via USB).

    Returns the PITIMER REPORT payload on match, or None on mismatch.
    On mismatch, logs a warning with full PITIMER diagnostics.
    """
    try:

        # Give time for zpnet-pitimer to recognize the PPS capture and store it in the ring buffer.
        time.sleep(0.100)

        resp = send_command(
            machine="PI",
            subsystem="PITIMER",
            command="REPORT",
            args={"gnss_time": time_key},
        )

        if resp.get("success"):
            return resp.get("payload", {})

        # Correlation failure — hard fault
        pitimer_payload = resp.get("payload", {})
        logging.warning(
            "🚨 [clocks] PITIMER correlation FAILED for time_key=%s — "
            "PITIMER response: %s",
            time_key,
            json.dumps(pitimer_payload, indent=2),
        )
        return None

    except Exception:
        logging.exception(
            "🚨 [clocks] PITIMER.REPORT IPC failed for time_key=%s",
            time_key,
        )
        return None


def _get_pitimer_capture_via_command() -> Optional[Dict[str, Any]]:
    """
    Fetch the latest Pi arch timer capture from the PITIMER process
    via the command channel (unkeyed).

    Used ONLY during recovery, before correlated captures are needed.

    Returns the PITIMER REPORT payload, or None if PITIMER is
    not available or has no capture yet.
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


def _wait_for_gnss_time() -> str:
    """
    Patiently wait for the GNSS receiver to produce a valid timestamp.

    After a cold start, the receiver may need minutes to acquire
    satellites and compute a fix.  This function polls GNSS.GET_TIME
    every GNSS_POLL_INTERVAL seconds and logs a "still waiting"
    message every GNSS_WAIT_LOG_INTERVAL seconds.

    Used only during recovery.

    Returns ISO8601 Z string once available.
    Never times out -- the math works regardless of how long
    the wait is.
    """
    wait_start = time.monotonic()
    last_log = wait_start

    while True:
        try:
            resp = send_command(
                machine="PI",
                subsystem="GNSS",
                command="GET_TIME",
            )
            if resp.get("success"):
                return resp["payload"]["gnss_time"]
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
    ocxo_ns = int(timebase.get("teensy_ocxo_ns") or 0)

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
        "pi_corrected": timebase.get("pi_corrected"),

        # Per-clock statistics (all residual stats in nanoseconds)
        "gnss": _build_clock_block("GNSS", gnss_ns, gnss_ns, _residual_gnss),
        "dwt": _build_clock_block("DWT", dwt_ns, gnss_ns, _residual_dwt),
        "pi": _build_clock_block("PI", pi_ns, gnss_ns, _residual_pi),
        "ocxo": _build_clock_block("OCXO", ocxo_ns, gnss_ns, _residual_ocxo),
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


# -----------------------------------------------------------------
# Recovery
# -----------------------------------------------------------------

def _recover_campaign() -> None:
    """
    Detect and recover an in-progress campaign after Pi restart.

    Called once at startup, before normal operation begins.

    Algorithm:
      1) Check for active campaign in Postgres
      2) Fetch the most recent TIMEBASE row for that campaign
      3) Wait patiently for GNSS to acquire satellites
      4) If campaign has a location, restore GNSS -> TO mode
      5) Capture GNSS time and PITIMER counter
      6) Compute elapsed seconds since last TIMEBASE
      7) Compute tau for each clock domain
      8) Project all clock values forward to the NEXT PPS edge
      9) Send RECOVER command to Teensy with projected values
         AND the persisted OCXO DAC value + calibration state
     10) Compute synthetic Pi tick epoch for continuous pi_ns
     11) Activate campaign

    NOTE: Recovery uses _get_pitimer_capture_via_command() (unkeyed)
    because correlated captures are not needed during recovery —
    we just need the latest counter value to compute the Pi epoch.

    NOTE: Recovery still uses GNSS.GET_TIME for elapsed time
    computation (needs actual GNSS time for datetime arithmetic).
    Normal operation uses system_time_z() for correlation keys.
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
    # Read persisted OCXO control state from campaign payload
    # ----------------------------------------------------------
    recover_ocxo_dac = campaign_payload.get("ocxo_dac")
    recover_calibrate = campaign_payload.get("calibrate_ocxo", False)
    recover_converged = campaign_payload.get("servo_converged", False)

    if recover_ocxo_dac is not None:
        logging.info(
            "🔧 [recovery] OCXO DAC: %d (calibrate=%s, converged=%s)",
            recover_ocxo_dac,
            recover_calibrate,
            recover_converged,
        )

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
    #
    # Use system_time_z() for the current time — it's the same
    # disciplined clock, and we only need it to compute elapsed
    # seconds (not for correlation).
    # ----------------------------------------------------------
    gnss_time_now_str = system_time_z()
    gnss_time_now = datetime.fromisoformat(
        gnss_time_now_str.replace("Z", "+00:00")
    )

    pi_capture = _get_pitimer_capture_via_command()
    pi_counter_now = 0
    if pi_capture:
        pi_counter_now = pi_capture.get("corrected") or pi_capture.get("counter", 0)

    logging.info(
        "⏱️ [recovery] system time now: %s  Pi counter (corrected): %d",
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
    # 9) Send RECOVER to Teensy (with OCXO DAC state)
    # ----------------------------------------------------------
    logging.info("🚀 [recovery] sending RECOVER to Teensy...")

    recover_args = {
        "dwt_cycles": str(projected_dwt_cycles),
        "gnss_ns": str(projected_gnss_ns),
        "ocxo_ns": str(projected_ocxo_ns),
    }

    # Restore OCXO DAC to last-known value so the crystal
    # doesn't reset to the boot default (2048) mid-campaign.
    if recover_ocxo_dac is not None:
        recover_args["set_dac"] = str(int(recover_ocxo_dac))

    # If calibration was active and hadn't converged,
    # re-enable the servo so it can continue hunting.
    # If it had already converged, just set the DAC and
    # don't re-enable — the value is already good.
    if recover_calibrate and not recover_converged:
        recover_args["calibrate_ocxo"] = "true"
        logging.info(
            "🔧 [recovery] re-enabling OCXO calibration servo (not yet converged)"
        )
    elif recover_calibrate and recover_converged:
        logging.info(
            "🔧 [recovery] OCXO servo was converged — restoring DAC=%d without re-calibrating",
            recover_ocxo_dac,
        )

    try:
        recover_resp = send_command(
            machine="TEENSY",
            subsystem="CLOCKS",
            command="RECOVER",
            args=recover_args,
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
    _residual_ocxo.reset()
    _campaign_active = True

    logging.info(
        "✅ [recovery] campaign '%s' recovered -- "
        "Teensy RECOVER sent, awaiting first TIMEBASE_FRAGMENT",
        campaign_name,
    )


# ---------------------------------------------------------------------
# TIMEBASE_FRAGMENT handler
# ---------------------------------------------------------------------

def on_timebase_fragment(payload: Payload) -> None:
    """
    Receive TIMEBASE_FRAGMENT from Teensy and build TIMEBASE.

    This is the main data path.  Each fragment triggers:
      1) Pi-side observations (system time)
      2) System clock correlation key (via system_time_z)
      3) Correlated Pi capture (via PITIMER.REPORT with time key)
      4) TIMEBASE augmentation (combine Teensy + Pi + GNSS)
      5) Residual statistics update (GNSS, DWT, OCXO, Pi)
      6) Report denormalization into campaign payload
      7) Publish + persist
      8) Persist latest OCXO DAC value into campaign payload

    Pi residual statistics are computed here in the fragment
    handler, driven by the correlated capture — not in a separate
    subscription handler.  Since the capture is guaranteed to match
    the fragment's PPS edge (by system clock key), there is no race
    condition and no need for separate timing.
    """
    global _request_start, _request_stop, _campaign_active
    global _pi_tick_epoch, _pi_tick_epoch_set

    # ----------------------------------------------------------
    # Capture Pi-side observations immediately
    # ----------------------------------------------------------
    system_time_utc = datetime.now(timezone.utc)

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
        _residual_ocxo.reset()
        _pi_tick_epoch_set = False
        _campaign_active = True
        _request_start = False
        logging.info("▶️ [clocks] campaign started")

    if not _campaign_active:
        return

    system_time_str = system_time_utc.isoformat(timespec='microseconds')

    # ----------------------------------------------------------
    # Get correlation key from the system clock.
    #
    # Both PITIMER and CLOCKS derive this from the same chrony-
    # disciplined clock.  pps_poll stored the capture with this
    # same key within milliseconds of PPS.  The fragment arrives
    # tens of ms later via USB, so the key is always available.
    # ----------------------------------------------------------
    time_key = system_time_z()

    # ----------------------------------------------------------
    # Fetch correlated Pi capture from PITIMER
    # ----------------------------------------------------------
    pi_capture = _get_pitimer_capture_by_time(time_key)

    # ----------------------------------------------------------
    # Extract Pi capture diagnostics and compute campaign-scoped pi_ns
    # ----------------------------------------------------------
    pi_counter_raw = None
    pi_counter_corrected = None
    pi_detect_ns = None
    pi_correction_ticks = None
    pi_seq = None
    pi_delta = None
    pi_residual = None
    pi_residual_ns = None
    pi_ns = 0

    if pi_capture is not None:
        pi_counter_raw = pi_capture.get("counter")
        pi_counter_corrected = pi_capture.get("corrected") or pi_counter_raw
        pi_detect_ns = pi_capture.get("detect_ns")
        pi_correction_ticks = pi_capture.get("correction_ticks", 0)
        pi_seq = pi_capture.get("seq")
        pi_delta = pi_capture.get("delta")
        pi_residual = pi_capture.get("residual")
        pi_residual_ns = pi_capture.get("residual_ns")

    if pi_counter_corrected is not None and not _pi_tick_epoch_set:
        _pi_tick_epoch = pi_counter_corrected
        _pi_tick_epoch_set = True
        logging.info(
            "⏱️ [clocks] Pi tick epoch set: %d (corrected counter, time_key=%s)",
            _pi_tick_epoch,
            time_key,
        )

    if pi_counter_corrected is not None and _pi_tick_epoch_set:
        campaign_ticks = pi_counter_corrected - _pi_tick_epoch
        pi_ns = _ticks_to_ns(campaign_ticks)

    # ----------------------------------------------------------
    # Update residual statistics (all four clocks)
    #
    # Pi residual is computed here, driven by the correlated
    # capture.  Since the capture is guaranteed to match the
    # fragment's PPS edge, every update is a clean 1-second delta.
    # ----------------------------------------------------------
    frag = payload
    gnss_ns = int(frag.get("gnss_ns") or 0)
    dwt_ns = int(frag.get("dwt_ns") or 0)
    ocxo_ns = int(frag.get("ocxo_ns") or 0)

    if gnss_ns > 0:
        _safe_residual_update(_residual_gnss, gnss_ns, NS_PER_SECOND, "GNSS")
    if dwt_ns > 0:
        _safe_residual_update(_residual_dwt, dwt_ns, NS_PER_SECOND, "DWT")
    if ocxo_ns > 0:
        _safe_residual_update(_residual_ocxo, ocxo_ns, NS_PER_SECOND, "OCXO")
    if pi_counter_corrected is not None:
        _safe_residual_update(_residual_pi, pi_counter_corrected, PI_TIMER_FREQ, "Pi")

    # ----------------------------------------------------------
    # Build TIMEBASE record
    # ----------------------------------------------------------
    row = _get_active_campaign()
    if row is None:
        logging.warning("⚠️ [clocks] fragment received with no active campaign")
        return

    campaign = row["campaign"]
    campaign_payload = row["payload"]

    timebase = {
        "campaign": campaign,

        # System time when fragment arrived
        "system_time_utc": system_time_str,

        # GNSS time correlation key (from system clock)
        "gnss_time_utc": time_key,

        # Teensy clocks (authoritative)
        "teensy_dwt_cycles": frag["dwt_cycles"],
        "teensy_dwt_ns": frag["dwt_ns"],
        "teensy_gnss_ns": frag["gnss_ns"],
        "teensy_ocxo_ns": frag.get("ocxo_ns"),
        "teensy_rtc1_ns": frag.get("rtc1_ns"),
        "teensy_rtc2_ns": frag.get("rtc2_ns"),

        # Pi clock
        "pi_counter": pi_counter_raw,
        "pi_corrected": pi_counter_corrected,
        "pi_ns": pi_ns,

        # Pi TDC diagnostics
        "diag_pi_raw_counter": pi_counter_raw,
        "diag_pi_corrected": pi_counter_corrected,
        "diag_pi_detect_ns": pi_detect_ns,
        "diag_pi_correction_ticks": pi_correction_ticks,
        "diag_pi_correction_ns": round(pi_correction_ticks * PI_NS_PER_TICK, 3) if pi_correction_ticks is not None else None,
        "diag_pi_seq": pi_seq,
        "diag_pi_delta": pi_delta,
        "diag_pi_residual": pi_residual,
        "diag_pi_residual_ns": pi_residual_ns,

        # Per-clock PPS residuals (nanoseconds)
        "isr_residual_gnss": frag.get("isr_residual_gnss"),
        "isr_residual_dwt": frag.get("isr_residual_dwt"),
        "isr_residual_ocxo": frag.get("isr_residual_ocxo"),
        "isr_residual_pi": round(_residual_pi.residual * PI_NS_PER_TICK, 3) if _residual_pi.valid else None,

        # Teensy PPS count
        "teensy_pps_count": frag.get("teensy_pps_count"),

        # Teensy TDC diagnostics
        "diag_teensy_dispatch_delta_ns": frag.get("dispatch_delta_ns"),
        "diag_teensy_pps_edge_valid": frag.get("pps_edge_valid"),
        "diag_teensy_pps_edge_correction_ns": frag.get("pps_edge_correction_ns"),
        "diag_teensy_detect_ns": frag.get("dispatch_delta_ns"),

        # OCXO DAC control state (from Teensy telemetry)
        "ocxo_dac": frag.get("ocxo_dac"),
        "calibrate_ocxo": frag.get("calibrate_ocxo"),
        "servo_converged": frag.get("servo_converged"),
    }

    # ----------------------------------------------------------
    # Build augmented report
    # ----------------------------------------------------------
    report = _build_report(campaign, campaign_payload, timebase)

    # Publish sacred tuple
    publish("TIMEBASE", timebase)

    # Persist best-effort (also denormalizes report into campaign)
    _persist_timebase(timebase, report)

    # ----------------------------------------------------------
    # Persist latest OCXO DAC value into campaign payload.
    #
    # This captures the servo's progress as it converges.
    # On recovery, the Pi reads this value and sends it back
    # to the Teensy so the OCXO resumes at the last-known
    # good control voltage.
    #
    # Best-effort — failure is logged and ignored.
    # ----------------------------------------------------------
    teensy_ocxo_dac = frag.get("ocxo_dac")
    teensy_calibrate = frag.get("calibrate_ocxo")
    teensy_converged = frag.get("servo_converged")

    if teensy_ocxo_dac is not None:
        try:
            with open_db() as conn:
                cur = conn.cursor()
                cur.execute(
                    """
                    UPDATE campaigns
                    SET payload = payload || %s::jsonb
                    WHERE campaign = %s
                      AND active = true
                    """,
                    (
                        json.dumps({
                            "ocxo_dac": int(teensy_ocxo_dac),
                            "calibrate_ocxo": bool(teensy_calibrate),
                            "servo_converged": bool(teensy_converged),
                        }),
                        campaign,
                    ),
                )
        except Exception:
            logging.exception("⚠️ [clocks] failed to persist OCXO DAC (ignored)")


# -----------------------------------------------------------------
# Command handlers
# -----------------------------------------------------------------

def cmd_start(args: Optional[dict]) -> dict:
    """
    Start a new campaign.

    Args:
        campaign (str):          Campaign name (required)
        location (str):          Profiled location name (optional)
        set_dac (int):           OCXO DAC value 0–4095 (optional)
        calibrate_ocxo (bool):   Enable OCXO calibration servo (optional)

    Sequence:
      1) Deactivate any existing campaign
      2) Insert new campaign row (with OCXO control state persisted)
      3) If location specified: command GNSS -> TO mode (blocking)
      4) Set _request_start flag (consumed on next fragment)
      5) Start Teensy CLOCKS acquisition (Teensy zeros at PPS)
    """
    global _request_start, _request_stop

    campaign = args.get("campaign")
    if not campaign:
        return {"success": False, "message": "START requires 'campaign' argument"}

    location = (args.get("location") or "").strip() or None

    # Parse OCXO control parameters
    set_dac = args.get("set_dac")

    # If caller didn't specify set_dac, read from config
    if set_dac is None:
        try:
            with open_db(row_dict=True) as conn:
                cur = conn.cursor()
                cur.execute(
                    "SELECT payload FROM config WHERE config_key = 'SYSTEM'"
                )
                row = cur.fetchone()
                if row and row["payload"].get("ocxo_dac") is not None:
                    set_dac = int(row["payload"]["ocxo_dac"])
                    logging.info("🔧 [clocks] using ocxo_dac=%d from SYSTEM config", set_dac)
        except Exception:
            logging.exception("⚠️ [clocks] failed to read SYSTEM config (ignored)")

    calibrate_ocxo = bool(args.get("calibrate_ocxo"))

    # ----------------------------------------------------------
    # 1) Deactivate existing + insert new campaign
    #    Persist OCXO control state so recovery can restore it.
    # ----------------------------------------------------------
    campaign_payload = {
        "location": location,
        "started_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
    }

    # Persist OCXO control state into campaign payload.
    # ocxo_dac stores the INITIAL requested value; it will be
    # overwritten every second by the latest Teensy telemetry
    # as the servo converges.  Recovery reads whichever value
    # is current at crash time.
    if set_dac is not None:
        campaign_payload["ocxo_dac"] = int(set_dac)
    if calibrate_ocxo:
        campaign_payload["calibrate_ocxo"] = True

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
    #    Pass through OCXO control parameters to Teensy.
    # ----------------------------------------------------------
    teensy_args = {"campaign": campaign}

    if set_dac is not None:
        teensy_args["set_dac"] = str(set_dac)

    if calibrate_ocxo:
        teensy_args["calibrate_ocxo"] = "true"

    send_command(
        machine="TEENSY",
        subsystem="CLOCKS",
        command="START",
        args=teensy_args,
    )

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "campaign": campaign,
            "location": location,
            "set_dac": set_dac,
            "calibrate_ocxo": calibrate_ocxo,
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


def cmd_clear(_: Optional[dict]) -> dict:
    """
    Delete all timebase records and all campaign records from Postgres.

    This is a development convenience for rapid iteration while
    shaking out clock behavior.  It also stops any active campaign
    so the process state is consistent with the empty database.
    """
    global _request_stop, _request_start, _campaign_active

    # Stop any active campaign so process state matches empty DB
    if _campaign_active:
        _campaign_active = False
        _request_start = False
        _request_stop = False
        logging.info("⏹️ [clocks] CLEAR stopped active campaign")

    try:
        with open_db() as conn:
            cur = conn.cursor()
            cur.execute("DELETE FROM timebase")
            tb_count = cur.rowcount
            cur.execute("DELETE FROM campaigns")
            camp_count = cur.rowcount

        logging.info(
            "🗑️ [clocks] CLEAR: deleted %d timebase rows, %d campaigns",
            tb_count, camp_count,
        )

        return {
            "success": True,
            "message": "OK",
            "payload": {
                "timebase_deleted": tb_count,
                "campaigns_deleted": camp_count,
            },
        }

    except Exception as e:
        logging.exception("❌ [clocks] CLEAR failed")
        return {
            "success": False,
            "message": str(e),
        }


COMMANDS = {
    "START": cmd_start,
    "STOP": cmd_stop,
    "REPORT": cmd_report,
    "CLEAR": cmd_clear,
}

# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    setup_logging()

    logging.info(
        "🕐 [clocks] Four tracked clocks: GNSS (Teensy 10MHz), "
        "DWT (Teensy 600MHz), OCXO (Teensy GPT1 10MHz), "
        "Pi (CNTVCT_EL0 54MHz via PITIMER correlated by system clock). "
        "chrony-disciplined system time for correlation keys. "
        "Pi TDC correction enabled (detect_ns → correction_ticks). "
        "Outlier rejection on all clock domains (>50%% deviation → reject). "
        "Pi capture correlated by system_time_z (request/response, no pub/sub)."
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
    # Start server — single subscription:
    #   TIMEBASE_FRAGMENT: primary data path (Teensy -> Pi)
    #
    # Pi captures are fetched via PITIMER.REPORT(gnss_time=<key>)
    # request/response in the fragment handler — no PPS_CAPTURE
    # subscription needed.  Correlation is guaranteed by shared
    # system clock (system_time_z).
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