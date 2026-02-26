"""
ZPNet CLOCKS Process — TIMEBASE Authority (Pi-side)

Responsibilities:
  * Receive TIMEBASE_FRAGMENT from Teensy
  * Fetch correlated Pi capture from PITIMER
  * Augment fragment with Pi capture, GNSS time, and system time
  * Publish TIMEBASE
  * Persist TIMEBASE to PostgreSQL (append-only, best-effort)
  * Compute per-clock residual statistics (Welford's algorithm)
  * Denormalize augmented report into active campaign payload
  * Recover clocks after restart if campaign is active
  * Command GNSS into Time Only mode when campaign has a location

Semantics:
  * No smoothing, inference, or filtering
  * TIMEBASE records are sacred and immutable
  * Derivative statistics (tau, ppb, residuals) live only in
    the campaign report -- never in the TIMEBASE row itself

Correlation (pps_count campaign fact — v2026-02-25+):
  * pps_count is a campaign-level first-class identity.
  * Intended meaning: pps_count == integer seconds elapsed since campaign inception (GNSS time).
  * PITIMER maintains a ring indexed by pps_count after SET_NEXT_PPS_COUNT is armed.
  * Teensy publishes teensy_pps_count in every TIMEBASE_FRAGMENT.

START:
  * CLOCKS arms PITIMER.SET_NEXT_PPS_COUNT(0)
  * Teensy starts and publishes teensy_pps_count (expected to begin at 0)

RECOVER:
  * CLOCKS reads last TIMEBASE row and computes next_pps_count based on elapsed wall-clock seconds.
  * CLOCKS arms PITIMER.SET_NEXT_PPS_COUNT(next_pps_count) so PITIMER resumes identity continuity.
  * CLOCKS sends Teensy RECOVER with projected clocks AND the same pps_count target.
  * Teensy is expected to resume teensy_pps_count at that value (campaign fact continuity).

NOTE:
  * This file assumes PITIMER no longer supports RESET_PPS_COUNT and instead supports SET_NEXT_PPS_COUNT.
  * For full continuity, Teensy CLOCKS.RECOVER must accept a "pps_count" argument and publish it as teensy_pps_count.

Separation of Concerns (v2026-02-25+):
  * on_timebase_fragment() is a pure adapter:
        TIMEBASE_FRAGMENT -> (correlate + augment) -> TIMEBASE
    It contains no recovery logic.
  * Recovery logic must fully parameterize local state BEFORE we resume normal streaming.
    In particular, Pi epoch anchoring is established inside _recover_campaign().

Hard Requirement:
  * There is no reduced-capacity mode. Pi epoch must be available.
  * If Pi epoch cannot be anchored, CLOCKS must refuse to publish TIMEBASE and fail fast.
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

NS_PER_SECOND = 1_000_000_000

PI_TIMER_FREQ = 54_000_000
PI_NS_PER_TICK = 1e9 / PI_TIMER_FREQ  # ~18.519 ns

DWT_CYCLES_PER_NS_NUM = 3
DWT_CYCLES_PER_NS_DEN = 5

GNSS_POLL_INTERVAL = 5
GNSS_WAIT_LOG_INTERVAL = 60

# PITIMER correlation retry (bounded, instrumented)
PITIMER_LOOKUP_RETRIES = 5
PITIMER_LOOKUP_SLEEP_S = 0.05  # 50 ms between attempts

# Recovery: allow a small window of "missed pps_count" and retry briefly.
RECOVERY_PI_EPOCH_MAX_ADVANCE_S = 5      # accept pps_count in [pending, pending+N]
RECOVERY_PI_EPOCH_RETRIES = 10          # retry the window a few times
RECOVERY_PI_EPOCH_SLEEP_S = 0.05        # 50ms between retries

# ---------------------------------------------------------------------
# Diagnostics (monotonic counters + last anomaly snapshots)
# ---------------------------------------------------------------------

_diag: Dict[str, Any] = {
    # Fragment ingress
    "fragments_received": 0,
    "fragments_ignored_no_campaign": 0,
    "fragments_missing_teensy_pps_count": 0,

    # Hard-fail gates (counted before raising)
    "hardfail_no_active_campaign": 0,
    "hardfail_missing_pitimer_capture": 0,
    "hardfail_missing_pi_counter": 0,
    "hardfail_pi_epoch_not_set": 0,
    "last_hardfail": {},

    # PITIMER control (pps_count campaign fact)
    "pitimer_set_requests": 0,
    "pitimer_set_ipc_failures": 0,
    "pitimer_set_failures": 0,
    "last_pitimer_set": {},

    # PITIMER correlation (pps_count-based)
    "pitimer_lookup_calls": 0,
    "pitimer_lookup_retries": 0,
    "pitimer_lookup_failures": 0,
    "pitimer_lookup_ipc_failures": 0,

    # pps_count continuity (as a campaign fact)
    "pps_count_seen": 0,
    "pps_count_repeat": 0,
    "pps_count_jump": 0,
    "pps_count_regress": 0,
    "last_pps_count": None,
    "last_pps_count_anomaly": {},

    # Recovery accounting (pps_count continuity)
    "recovery_checks": 0,
    "recovery_no_active_campaign": 0,
    "recovery_missing_timebase": 0,
    "recovery_missing_last_pps_count": 0,
    "recovery_elapsed_seconds_nonpositive": 0,
    "recovery_set_next_pps_failed": 0,

    # Recovery: GNSS wait
    "recovery_gnss_waits": 0,
    "recovery_gnss_wait_success": 0,
    "recovery_gnss_wait_fail": 0,
    "recovery_gnss_wait_seconds_total": 0.0,
    "recovery_gnss_wait_seconds_last": 0.0,
    "last_recovery_gnss_wait": {},

    # Recovery: Pi epoch anchoring
    "recovery_pi_epoch_attempts": 0,
    "recovery_pi_epoch_retry_loops": 0,
    "recovery_pi_epoch_search_iters": 0,
    "recovery_pi_epoch_success": 0,
    "recovery_pi_epoch_fail": 0,
    "recovery_pi_epoch_delta_s_last": None,
    "last_recovery_pi_epoch": {},

    "last_recovery": {},

    # Pi capture integrity
    "pi_capture_missing": 0,
    "pi_capture_missing_counter": 0,
    "pi_capture_seq_missing": 0,
    "pi_capture_seq_repeat": 0,
    "pi_capture_seq_jump": 0,
    "pi_capture_corrected_repeat": 0,

    # Residual outliers (all clocks)
    "outliers_rejected_total": 0,
    "outliers_rejected_gnss": 0,
    "outliers_rejected_dwt": 0,
    "outliers_rejected_ocxo": 0,
    "outliers_rejected_pi": 0,

    # Pi outlier classification
    "pi_delta_zero": 0,
    "pi_delta_double": 0,
    "pi_delta_other": 0,

    # Last anomaly snapshots (small, overwritten)
    "last_pitimer_failure": {},
    "last_pi_capture_anomaly": {},
    "last_outlier": {},
}

# Last-seen Pi capture identity (for detecting duplicates / gaps)
_last_pi_seq: Optional[int] = None
_last_pi_corrected: Optional[int] = None

# Last-seen pps_count from Teensy fragments (campaign fact continuity)
_last_pps_count_seen: Optional[int] = None

# ---------------------------------------------------------------------
# PPS residual tracking -- Welford's online algorithm
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
# Outlier-safe residual update (instrumented)
# ---------------------------------------------------------------------


def _classify_pi_delta(delta: int) -> str:
    if delta == 0:
        return "zero"
    # "about 2 seconds" in ticks
    if abs(delta - (2 * PI_TIMER_FREQ)) < 10_000:
        return "double"
    return "other"


def _note_outlier(
    label: str,
    value_now: int,
    last_value: int,
    expected: int,
    delta: int,
    deviation: int,
) -> None:
    _diag["outliers_rejected_total"] += 1
    if label == "GNSS":
        _diag["outliers_rejected_gnss"] += 1
    elif label == "DWT":
        _diag["outliers_rejected_dwt"] += 1
    elif label == "OCXO":
        _diag["outliers_rejected_ocxo"] += 1
    elif label == "Pi":
        _diag["outliers_rejected_pi"] += 1

        cls = _classify_pi_delta(delta)
        if cls == "zero":
            _diag["pi_delta_zero"] += 1
        elif cls == "double":
            _diag["pi_delta_double"] += 1
        else:
            _diag["pi_delta_other"] += 1

    _diag["last_outlier"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "label": label,
        "value_now": int(value_now),
        "last_value": int(last_value),
        "delta": int(delta),
        "expected": int(expected),
        "deviation": int(deviation),
    }


def _safe_residual_update(
    tracker: _PpsResidual,
    value_now: int,
    expected: int,
    label: str,
) -> None:
    """
    Update a residual tracker with outlier rejection + instrumentation.

    Policy:
      * Reject any delta with |delta-expected| > expected/2
      * On reject:
          - increment counters
          - capture last anomaly snapshot
          - advance anchor (last_value=value_now) so the next delta starts clean
      * No warnings required for correctness; the counters are the signal.
    """
    if tracker.last_value > 0:
        prospective_delta = value_now - tracker.last_value
        deviation = abs(prospective_delta - expected)
        if deviation > expected // 2:
            _note_outlier(
                label=label,
                value_now=value_now,
                last_value=tracker.last_value,
                expected=expected,
                delta=prospective_delta,
                deviation=deviation,
            )
            tracker.last_value = value_now
            return

    tracker.update(value_now, expected)


# ---------------------------------------------------------------------
# Local state (process lifetime)
# ---------------------------------------------------------------------

_request_start: bool = False
_request_stop: bool = False

_campaign_active: bool = False

_pi_tick_epoch: int = 0
_pi_tick_epoch_set: bool = False

_residual_gnss = _PpsResidual()
_residual_dwt = _PpsResidual()
_residual_pi = _PpsResidual(display_scale=PI_NS_PER_TICK)
_residual_ocxo = _PpsResidual()

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------


def _note_hardfail(reason: str, details: Optional[Dict[str, Any]] = None) -> None:
    _diag["last_hardfail"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "reason": reason,
        "details": details or {},
    }


def _set_pitimer_next_pps_count(next_pps_count: int) -> bool:
    """
    Best-effort: command PITIMER to arm pps_count assignment.

    Semantics:
      * PITIMER assigns pps_count=<next_pps_count> to the NEXT captured PPS after set.
      * Subsequent PPS captures increment monotonically.
    """
    _diag["pitimer_set_requests"] += 1
    try:
        resp = send_command(
            machine="PI",
            subsystem="PITIMER",
            command="SET_NEXT_PPS_COUNT",
            args={"pps_count": int(next_pps_count)},
        )
        ok = bool(resp.get("success", False))
        _diag["last_pitimer_set"] = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "success": ok,
            "next_pps_count": int(next_pps_count),
            "response": resp.get("payload", {}) if isinstance(resp, dict) else {},
        }
        if not ok:
            _diag["pitimer_set_failures"] += 1
        return ok
    except Exception:
        _diag["pitimer_set_ipc_failures"] += 1
        _diag["last_pitimer_set"] = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "success": False,
            "next_pps_count": int(next_pps_count),
            "exception": "PITIMER.SET_NEXT_PPS_COUNT IPC failed",
        }
        return False


def _get_pitimer_capture_by_pps_count(pps_count: int) -> Optional[Dict[str, Any]]:
    """
    Fetch the Pi arch timer capture for a specific pps_count from PITIMER.

    Instrumented + bounded retry:
      * PITIMER capture insertion and CLOCKS lookup are in separate threads/processes.
      * During startup / recovery / transient load, a keyed capture can be absent briefly.
      * We retry a few times to reduce false "missing capture" conditions.
      * Any failure is counted and surfaced via CLOCKS_INFO.
    """
    _diag["pitimer_lookup_calls"] += 1

    for attempt in range(1, PITIMER_LOOKUP_RETRIES + 1):
        try:
            if attempt > 1:
                _diag["pitimer_lookup_retries"] += 1
                time.sleep(PITIMER_LOOKUP_SLEEP_S)

            resp = send_command(
                machine="PI",
                subsystem="PITIMER",
                command="REPORT_PPS_COUNT",
                args={"pps_count": int(pps_count)},
            )

            if resp.get("success"):
                return resp.get("payload", {})

            pitimer_payload = resp.get("payload", {})
            if attempt == PITIMER_LOOKUP_RETRIES:
                _diag["pitimer_lookup_failures"] += 1
                _diag["last_pitimer_failure"] = {
                    "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                    "pps_count": int(pps_count),
                    "attempts": PITIMER_LOOKUP_RETRIES,
                    "pitimer_payload": pitimer_payload,
                }
            continue

        except Exception:
            if attempt == PITIMER_LOOKUP_RETRIES:
                _diag["pitimer_lookup_ipc_failures"] += 1
                _diag["last_pitimer_failure"] = {
                    "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                    "pps_count": int(pps_count),
                    "attempts": PITIMER_LOOKUP_RETRIES,
                    "exception": "PITIMER.REPORT_PPS_COUNT IPC failed",
                }
            continue

    return None


def _get_pitimer_capture_via_command() -> Optional[Dict[str, Any]]:
    """
    Fetch the latest Pi arch timer capture from PITIMER (unkeyed).
    Used ONLY during recovery for pi_counter_now (observational/logging only).
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
    Patiently wait for GNSS time to be available (ISO8601 Z).
    Used only during recovery. Instrumented.
    """
    _diag["recovery_gnss_waits"] += 1

    wait_start_mon = time.monotonic()
    wait_start_utc = datetime.now(timezone.utc)

    last_log = wait_start_mon

    while True:
        try:
            resp = send_command(
                machine="PI",
                subsystem="GNSS",
                command="GET_TIME",
            )
            if resp.get("success"):
                gnss_time = resp["payload"]["gnss_time"]

                waited_s = time.monotonic() - wait_start_mon
                _diag["recovery_gnss_wait_success"] += 1
                _diag["recovery_gnss_wait_seconds_total"] += float(waited_s)
                _diag["recovery_gnss_wait_seconds_last"] = float(waited_s)
                _diag["last_recovery_gnss_wait"] = {
                    "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                    "start_utc": wait_start_utc.isoformat().replace("+00:00", "Z"),
                    "waited_s": round(float(waited_s), 3),
                    "poll_interval_s": int(GNSS_POLL_INTERVAL),
                }

                return gnss_time
        except Exception:
            pass

        now = time.monotonic()
        elapsed = now - wait_start_mon

        if now - last_log >= GNSS_WAIT_LOG_INTERVAL:
            logging.info("⏳ [recovery] waiting for GNSS time... (%.0fs elapsed)", elapsed)
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

    return {"campaign": row["campaign"], "payload": payload}


def _set_gnss_mode_to(location: str) -> Dict[str, Any]:
    return send_command(
        machine="PI",
        subsystem="GNSS",
        command="MODE",
        args={"mode": "TO", "location": location},
    )


def _set_gnss_mode_normal() -> Dict[str, Any]:
    return send_command(
        machine="PI",
        subsystem="GNSS",
        command="MODE",
        args={"mode": "NORMAL"},
    )


def _note_pps_count(teensy_pps_count: int) -> bool:
    """
    Continuity checks for the campaign-fact pps_count stream.
    Observational only. Returns True if usable, False if a hard anomaly
    should abort processing (we choose to continue unless negative).
    """
    global _last_pps_count_seen

    k = int(teensy_pps_count)

    _diag["pps_count_seen"] += 1
    _diag["last_pps_count"] = k

    if _last_pps_count_seen is not None:
        if k == _last_pps_count_seen:
            _diag["pps_count_repeat"] += 1
            _diag["last_pps_count_anomaly"] = {
                "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "reason": "pps_count_repeat",
                "pps_count": k,
                "prev_pps_count": int(_last_pps_count_seen),
            }
        elif k < _last_pps_count_seen:
            _diag["pps_count_regress"] += 1
            _diag["last_pps_count_anomaly"] = {
                "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "reason": "pps_count_regress",
                "pps_count": k,
                "prev_pps_count": int(_last_pps_count_seen),
            }
        elif k > (_last_pps_count_seen + 1):
            _diag["pps_count_jump"] += 1
            _diag["last_pps_count_anomaly"] = {
                "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "reason": "pps_count_jump",
                "pps_count": k,
                "prev_pps_count": int(_last_pps_count_seen),
                "jump": int(k - _last_pps_count_seen),
            }

    _last_pps_count_seen = k
    return True


def _anchor_pi_epoch_after_recovery(
    pending_pps_count: int,
    projected_ticks_at_pending: int,
) -> None:
    """
    Establish _pi_tick_epoch deterministically during recovery.

    HARD REQUIREMENT:
      * If we cannot anchor epoch, recovery fails.

    Algorithm:
      * Try to obtain PITIMER capture for pps_count in [pending, pending+N]
      * Retry the entire window briefly
      * Once a capture is found:
          epoch = corrected_counter - (projected_ticks_at_pending + delta_s * PI_TIMER_FREQ)
    """
    global _pi_tick_epoch, _pi_tick_epoch_set

    _diag["recovery_pi_epoch_attempts"] += 1

    for attempt in range(1, RECOVERY_PI_EPOCH_RETRIES + 1):
        _diag["recovery_pi_epoch_retry_loops"] += 1
        for k in range(0, RECOVERY_PI_EPOCH_MAX_ADVANCE_S + 1):
            _diag["recovery_pi_epoch_search_iters"] += 1
            pps = int(pending_pps_count) + int(k)
            cap = _get_pitimer_capture_by_pps_count(pps)
            if cap is None:
                continue

            corrected = cap.get("corrected") or cap.get("counter")
            if corrected is None:
                continue

            corrected = int(corrected)
            projected_ticks_now = int(projected_ticks_at_pending) + (int(k) * PI_TIMER_FREQ)

            _pi_tick_epoch = corrected - projected_ticks_now
            _pi_tick_epoch_set = True

            _diag["recovery_pi_epoch_success"] += 1
            _diag["recovery_pi_epoch_delta_s_last"] = int(k)
            _diag["last_recovery_pi_epoch"] = {
                "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "attempt": attempt,
                "pending_pps_count": int(pending_pps_count),
                "used_pps_count": int(pps),
                "delta_s": int(k),
                "corrected_counter": int(corrected),
                "projected_ticks_at_pending": int(projected_ticks_at_pending),
                "projected_ticks_now": int(projected_ticks_now),
                "pi_tick_epoch": int(_pi_tick_epoch),
            }

            if k == 0:
                logging.info(
                    "⏱️ [recovery] Pi epoch anchored at target boundary: epoch=%d (pps_count=%d, corrected=%d, projected_ticks=%d)",
                    _pi_tick_epoch,
                    int(pps),
                    int(corrected),
                    int(projected_ticks_now),
                )
            else:
                logging.warning(
                    "⚠️ [recovery] Pi epoch anchored after missed boundary: epoch=%d (pending=%d, used=%d, delta_s=%d, corrected=%d, projected_ticks_now=%d)",
                    _pi_tick_epoch,
                    int(pending_pps_count),
                    int(pps),
                    int(k),
                    int(corrected),
                    int(projected_ticks_now),
                )

            return

        time.sleep(RECOVERY_PI_EPOCH_SLEEP_S)

    _diag["recovery_pi_epoch_fail"] += 1
    _diag["last_recovery_pi_epoch"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "pending_pps_count": int(pending_pps_count),
        "projected_ticks_at_pending": int(projected_ticks_at_pending),
        "attempts": int(RECOVERY_PI_EPOCH_RETRIES),
        "window_s": int(RECOVERY_PI_EPOCH_MAX_ADVANCE_S),
    }

    raise RuntimeError(
        f"recovery failed: unable to anchor pi epoch (pending_pps_count={int(pending_pps_count)})"
    )


# ---------------------------------------------------------------------
# Clock statistics helpers
# ---------------------------------------------------------------------


def _seconds_to_hms(seconds: int) -> str:
    h = seconds // 3600
    m = (seconds % 3600) // 60
    s = seconds % 60
    return f"{h:02d}:{m:02d}:{s:02d}"


def _compute_tau(clock_ns: int, gnss_ns: int) -> float:
    if gnss_ns == 0:
        return 0.0
    return clock_ns / gnss_ns


def _compute_ppb(clock_ns: int, gnss_ns: int) -> float:
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
    gnss_ns = int(timebase.get("teensy_gnss_ns") or 0)
    dwt_ns = int(timebase.get("teensy_dwt_ns") or 0)
    pi_ns = int(timebase.get("pi_ns") or 0)
    ocxo_ns = int(timebase.get("teensy_ocxo_ns") or 0)

    pps_count = timebase.get("pps_count")
    if isinstance(pps_count, int):
        campaign_seconds = pps_count
    else:
        campaign_seconds = gnss_ns // NS_PER_SECOND if gnss_ns > 0 else 0

    report: Dict[str, Any] = {
        "campaign": campaign_name,
        "campaign_state": "STARTED",
        "campaign_seconds": int(campaign_seconds),
        "campaign_elapsed": _seconds_to_hms(int(campaign_seconds)),
        "location": campaign_payload.get("location"),

        "gnss_time_utc": timebase.get("gnss_time_utc"),
        "system_time_utc": timebase.get("system_time_utc"),

        "pps_count": timebase.get("pps_count"),

        "teensy_dwt_cycles": timebase.get("teensy_dwt_cycles"),
        "teensy_ocxo_ns": timebase.get("teensy_ocxo_ns"),
        "teensy_rtc1_ns": timebase.get("teensy_rtc1_ns"),
        "teensy_rtc2_ns": timebase.get("teensy_rtc2_ns"),
        "pi_counter": timebase.get("pi_counter"),
        "pi_corrected": timebase.get("pi_corrected"),

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
    global _campaign_active, _pi_tick_epoch_set
    global _last_pi_seq, _last_pi_corrected
    global _last_pps_count_seen

    _diag["recovery_checks"] += 1

    row = _get_active_campaign()
    if row is None:
        _diag["recovery_no_active_campaign"] += 1
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
        _diag["recovery_missing_timebase"] += 1
        raise RuntimeError(f"recovery failed: campaign '{campaign_name}' has no TIMEBASE rows")

    last_tb = tb_row["payload"]
    if isinstance(last_tb, str):
        last_tb = json.loads(last_tb)

    last_system_utc_str = last_tb.get("system_time_utc") or last_tb.get("gnss_time_utc")
    if not last_system_utc_str:
        last_system_utc_str = system_time_z()

    last_system_utc = datetime.fromisoformat(last_system_utc_str.replace("Z", "+00:00"))

    last_gnss_utc_str = last_tb.get("gnss_time_utc") or last_system_utc_str
    last_gnss_utc = datetime.fromisoformat(last_gnss_utc_str.replace("Z", "+00:00"))

    last_pps_count = last_tb.get("pps_count")
    if last_pps_count is None:
        last_pps_count = last_tb.get("teensy_pps_count")
    if last_pps_count is None:
        _diag["recovery_missing_last_pps_count"] += 1
        raise RuntimeError("recovery failed: last TIMEBASE missing pps_count")

    last_pps_count = int(last_pps_count)

    last_dwt_ns = int(last_tb["teensy_dwt_ns"])
    last_gnss_ns = int(last_tb.get("teensy_gnss_ns") or 0)
    last_ocxo_ns = int(last_tb.get("teensy_ocxo_ns") or 0)
    last_pi_ns = int(last_tb.get("pi_ns") or 0)

    logging.info(
        "🔍 [recovery] last TIMEBASE: system_utc=%s  gnss_utc=%s  pps_count=%d  gnss_ns=%d  dwt_ns=%d  ocxo_ns=%d  pi_ns=%d",
        last_system_utc.isoformat(),
        last_gnss_utc.isoformat(),
        last_pps_count,
        last_gnss_ns,
        last_dwt_ns,
        last_ocxo_ns,
        last_pi_ns,
    )

    if last_gnss_ns == 0:
        logging.warning(
            "⚠️ [recovery] teensy_gnss_ns not available in TIMEBASE -- "
            "using dwt_ns as GNSS proxy (tau_dwt assumed ~ 1.0)"
        )
        last_gnss_ns = last_dwt_ns

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

    logging.info("⏳ [recovery] waiting for GNSS time to become available...")
    gnss_time_preflight = _wait_for_gnss_time()
    logging.info("✅ [recovery] GNSS time available: %s", gnss_time_preflight)

    if campaign_location:
        logging.info("📡 [recovery] restoring GNSS -> TO mode for location '%s'", campaign_location)
        gnss_resp = _set_gnss_mode_to(campaign_location)
        if not gnss_resp.get("success"):
            raise RuntimeError(f"recovery failed: GNSS MODE=TO failed: {gnss_resp.get('message','?')}")
        logging.info("✅ [recovery] GNSS confirmed TO mode")

    now_str = system_time_z()
    now_utc = datetime.fromisoformat(now_str.replace("Z", "+00:00"))

    elapsed_td = now_utc - last_system_utc
    elapsed_seconds = int(round(elapsed_td.total_seconds()))
    if elapsed_seconds <= 0:
        _diag["recovery_elapsed_seconds_nonpositive"] += 1
        raise RuntimeError(f"recovery failed: elapsed_seconds={elapsed_seconds}")

    project_seconds = elapsed_seconds + 1
    next_pps_count = last_pps_count + project_seconds

    _diag["last_recovery"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "campaign": campaign_name,
        "last_system_utc": last_system_utc_str,
        "last_gnss_utc": last_gnss_utc_str,
        "now_utc": now_str,
        "elapsed_seconds": int(elapsed_seconds),
        "project_seconds": int(project_seconds),
        "last_pps_count": int(last_pps_count),
        "next_pps_count": int(next_pps_count),
    }

    logging.info("📐 [recovery] pps_count continuity: last=%d  elapsed=%d  next=%d", last_pps_count, elapsed_seconds, next_pps_count)

    if not _set_pitimer_next_pps_count(next_pps_count):
        _diag["recovery_set_next_pps_failed"] += 1
        raise RuntimeError("recovery failed: PITIMER.SET_NEXT_PPS_COUNT failed")

    _last_pps_count_seen = None
    _last_pi_seq = None
    _last_pi_corrected = None

    tau_dwt = float(last_dwt_ns) / float(last_gnss_ns) if last_gnss_ns > 0 else 1.0
    tau_ocxo = float(last_ocxo_ns) / float(last_gnss_ns) if (last_gnss_ns > 0 and last_ocxo_ns > 0) else 1.0
    tau_pi = float(last_pi_ns) / float(last_gnss_ns) if (last_gnss_ns > 0 and last_pi_ns > 0) else 1.0

    logging.info("📐 [recovery] tau -- dwt=%.12f  ocxo=%.12f  pi=%.12f", tau_dwt, tau_ocxo, tau_pi)

    projected_gnss_ns = last_gnss_ns + (project_seconds * NS_PER_SECOND)
    projected_dwt_ns = last_dwt_ns + int(project_seconds * NS_PER_SECOND * tau_dwt)
    projected_ocxo_ns = last_ocxo_ns + int(project_seconds * NS_PER_SECOND * tau_ocxo) if last_ocxo_ns > 0 else 0
    projected_pi_ns = last_pi_ns + int(project_seconds * NS_PER_SECOND * tau_pi) if last_pi_ns > 0 else 0

    projected_dwt_cycles = (projected_dwt_ns * DWT_CYCLES_PER_NS_NUM) // DWT_CYCLES_PER_NS_DEN

    logging.info(
        "📐 [recovery] projected to PPS+1 -- gnss_ns=%d  dwt_cycles=%d  dwt_ns=%d  ocxo_ns=%d  pi_ns=%d  pps_count=%d",
        projected_gnss_ns,
        projected_dwt_cycles,
        projected_dwt_ns,
        projected_ocxo_ns,
        projected_pi_ns,
        next_pps_count,
    )

    recover_args: Dict[str, Any] = {
        "dwt_cycles": str(projected_dwt_cycles),
        "gnss_ns": str(projected_gnss_ns),
        "ocxo_ns": str(projected_ocxo_ns),
        "pps_count": str(int(next_pps_count)),
    }

    if recover_ocxo_dac is not None:
        recover_args["set_dac"] = str(int(recover_ocxo_dac))
    if recover_calibrate and not recover_converged:
        recover_args["calibrate_ocxo"] = "true"

    recover_resp = send_command(machine="TEENSY", subsystem="CLOCKS", command="RECOVER", args=recover_args)
    if not recover_resp.get("success", True):
        raise RuntimeError(f"recovery failed: Teensy RECOVER rejected: {recover_resp.get('message','?')}")

    _pi_tick_epoch_set = False
    if projected_pi_ns <= 0:
        raise RuntimeError("recovery failed: projected_pi_ns <= 0 (cannot anchor epoch)")

    projected_pi_ticks = (projected_pi_ns * PI_TIMER_FREQ) // NS_PER_SECOND
    _anchor_pi_epoch_after_recovery(
        pending_pps_count=int(next_pps_count),
        projected_ticks_at_pending=int(projected_pi_ticks),
    )

    _residual_gnss.reset()
    _residual_dwt.reset()
    _residual_pi.reset()
    _residual_ocxo.reset()

    _campaign_active = True

    logging.info(
        "✅ [recovery] campaign '%s' recovered -- epoch anchored (pi_tick_epoch_set=%s)",
        campaign_name,
        str(_pi_tick_epoch_set),
    )


# ---------------------------------------------------------------------
# TIMEBASE_FRAGMENT handler (PURE ADAPTER + HARD GATE)
# ---------------------------------------------------------------------


def on_timebase_fragment(payload: Payload) -> None:
    global _request_start, _request_stop, _campaign_active
    global _pi_tick_epoch, _pi_tick_epoch_set
    global _last_pi_seq, _last_pi_corrected
    global _last_pps_count_seen

    _diag["fragments_received"] += 1

    system_time_utc = datetime.now(timezone.utc)

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
        _pi_tick_epoch = 0

        _last_pi_seq = None
        _last_pi_corrected = None
        _last_pps_count_seen = None

        _campaign_active = True
        _request_start = False
        logging.info("▶️ [clocks] campaign started")

    if not _campaign_active:
        _diag["fragments_ignored_no_campaign"] += 1
        return

    system_time_str = system_time_utc.isoformat(timespec="microseconds")

    frag = payload

    teensy_pps_count_raw = frag.get("teensy_pps_count")
    if teensy_pps_count_raw is None:
        _diag["fragments_missing_teensy_pps_count"] += 1
        return

    try:
        pps_count = int(teensy_pps_count_raw)
    except Exception:
        _diag["fragments_missing_teensy_pps_count"] += 1
        return

    _note_pps_count(pps_count)

    # Correlate (hard requirement)
    pi_capture = _get_pitimer_capture_by_pps_count(pps_count)
    if pi_capture is None:
        _diag["hardfail_missing_pitimer_capture"] += 1
        _note_hardfail("missing_pitimer_capture", {"pps_count": int(pps_count)})
        raise RuntimeError(f"hard failure: missing PITIMER capture for pps_count={int(pps_count)}")

    gnss_time_utc_label = pi_capture.get("gnss_time") or system_time_z()

    pi_counter_raw = pi_capture.get("counter")
    pi_counter_corrected = pi_capture.get("corrected") or pi_counter_raw
    if pi_counter_corrected is None:
        _diag["hardfail_missing_pi_counter"] += 1
        _note_hardfail("missing_pi_counter", {"pps_count": int(pps_count)})
        raise RuntimeError(f"hard failure: PITIMER capture missing corrected/counter for pps_count={int(pps_count)}")

    pi_counter_corrected = int(pi_counter_corrected)

    pi_detect_ns = pi_capture.get("detect_ns")
    pi_correction_ticks = pi_capture.get("correction_ticks", 0)
    pi_seq = pi_capture.get("seq")
    pi_delta = pi_capture.get("delta")
    pi_residual = pi_capture.get("residual")
    pi_residual_ns = pi_capture.get("residual_ns")

    # Maintain capture integrity diagnostics (observational)
    if pi_seq is None:
        _diag["pi_capture_seq_missing"] += 1
    if isinstance(pi_seq, int):
        if _last_pi_seq is not None:
            if pi_seq == _last_pi_seq:
                _diag["pi_capture_seq_repeat"] += 1
            elif pi_seq > (_last_pi_seq + 1):
                _diag["pi_capture_seq_jump"] += 1
        _last_pi_seq = pi_seq

    if _last_pi_corrected is not None and pi_counter_corrected == _last_pi_corrected:
        _diag["pi_capture_corrected_repeat"] += 1
    _last_pi_corrected = pi_counter_corrected

    # HARD GATE: epoch must exist. No reduced-capacity mode.
    if not _pi_tick_epoch_set:
        _diag["hardfail_pi_epoch_not_set"] += 1
        _note_hardfail("pi_epoch_not_set", {"pps_count": int(pps_count)})
        raise RuntimeError("hard failure: pi_tick_epoch not set (epoch anchoring is required)")

    campaign_ticks = pi_counter_corrected - _pi_tick_epoch
    pi_ns = _ticks_to_ns(campaign_ticks)

    # Residual updates
    gnss_ns = int(frag.get("gnss_ns") or 0)
    dwt_ns = int(frag.get("dwt_ns") or 0)
    ocxo_ns = int(frag.get("ocxo_ns") or 0)

    if gnss_ns > 0:
        _safe_residual_update(_residual_gnss, gnss_ns, NS_PER_SECOND, "GNSS")
    if dwt_ns > 0:
        _safe_residual_update(_residual_dwt, dwt_ns, NS_PER_SECOND, "DWT")
    if ocxo_ns > 0:
        _safe_residual_update(_residual_ocxo, ocxo_ns, NS_PER_SECOND, "OCXO")
    _safe_residual_update(_residual_pi, pi_counter_corrected, PI_TIMER_FREQ, "Pi")

    row = _get_active_campaign()
    if row is None:
        _diag["hardfail_no_active_campaign"] += 1
        _note_hardfail("no_active_campaign", {"pps_count": int(pps_count)})
        raise RuntimeError("hard failure: fragment received but no active campaign exists")

    campaign = row["campaign"]
    campaign_payload = row["payload"]

    timebase = {
        "campaign": campaign,

        # Metadata timestamps (NOT identity)
        "system_time_utc": system_time_str,
        "gnss_time_utc": gnss_time_utc_label,

        # Campaign-fact identity
        "pps_count": int(pps_count),

        # Teensy authoritative clock state
        "teensy_dwt_cycles": frag["dwt_cycles"],
        "teensy_dwt_ns": frag["dwt_ns"],
        "teensy_gnss_ns": frag["gnss_ns"],
        "teensy_ocxo_ns": frag.get("ocxo_ns"),
        "teensy_rtc1_ns": frag.get("rtc1_ns"),
        "teensy_rtc2_ns": frag.get("rtc2_ns"),

        # Preserve raw Teensy value for backward visibility (should match pps_count)
        "teensy_pps_count": int(pps_count),

        # Pi capture
        "pi_counter": pi_counter_raw,
        "pi_corrected": pi_counter_corrected,
        "pi_ns": pi_ns,

        # Pi capture diagnostics
        "diag_pi_raw_counter": pi_counter_raw,
        "diag_pi_corrected": pi_counter_corrected,
        "diag_pi_detect_ns": pi_detect_ns,
        "diag_pi_correction_ticks": pi_correction_ticks,
        "diag_pi_correction_ns": round(pi_correction_ticks * PI_NS_PER_TICK, 3) if pi_correction_ticks is not None else None,
        "diag_pi_seq": pi_seq,
        "diag_pi_delta": pi_delta,
        "diag_pi_residual": pi_residual,
        "diag_pi_residual_ns": pi_residual_ns,

        # Teensy ISR residuals + derived Pi residual (in ns)
        "isr_residual_gnss": frag.get("isr_residual_gnss"),
        "isr_residual_dwt": frag.get("isr_residual_dwt"),
        "isr_residual_ocxo": frag.get("isr_residual_ocxo"),
        "isr_residual_pi": round(_residual_pi.residual * PI_NS_PER_TICK, 3) if _residual_pi.valid else None,

        # Teensy dispatch profiling / TDC diagnostics
        "diag_teensy_dispatch_delta_ns": frag.get("dispatch_delta_ns"),
        "diag_teensy_pps_edge_valid": frag.get("pps_edge_valid"),
        "diag_teensy_pps_edge_correction_ns": frag.get("pps_edge_correction_ns"),
        "diag_teensy_detect_ns": frag.get("dispatch_delta_ns"),

        # OCXO control state
        "ocxo_dac": frag.get("ocxo_dac"),
        "calibrate_ocxo": frag.get("calibrate_ocxo"),
        "servo_converged": frag.get("servo_converged"),
    }

    report = _build_report(campaign, campaign_payload, timebase)

    publish("TIMEBASE", timebase)
    _persist_timebase(timebase, report)

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
                        json.dumps(
                            {
                                "ocxo_dac": int(teensy_ocxo_dac),
                                "calibrate_ocxo": bool(teensy_calibrate),
                                "servo_converged": bool(teensy_converged),
                            }
                        ),
                        campaign,
                    ),
                )
        except Exception:
            logging.exception("⚠️ [clocks] failed to persist OCXO DAC (ignored)")


# -----------------------------------------------------------------
# Command handlers
# -----------------------------------------------------------------


def cmd_start(args: Optional[dict]) -> dict:
    global _request_start, _request_stop
    global _last_pps_count_seen

    campaign = args.get("campaign") if args else None
    if not campaign:
        return {"success": False, "message": "START requires 'campaign' argument"}

    location = (args.get("location") or "").strip() or None
    set_dac = args.get("set_dac")

    if set_dac is None:
        try:
            with open_db(row_dict=True) as conn:
                cur = conn.cursor()
                cur.execute("SELECT payload FROM config WHERE config_key = 'SYSTEM'")
                row = cur.fetchone()
                if row and row["payload"].get("ocxo_dac") is not None:
                    set_dac = int(row["payload"]["ocxo_dac"])
                    logging.info("🔧 [clocks] using ocxo_dac=%d from SYSTEM config", set_dac)
        except Exception:
            logging.exception("⚠️ [clocks] failed to read SYSTEM config (ignored)")

    calibrate_ocxo = bool(args.get("calibrate_ocxo"))

    campaign_payload = {
        "location": location,
        "started_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
    }

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

    if location:
        logging.info("📡 [clocks] commanding GNSS -> TO mode for location '%s'", location)
        gnss_resp = _set_gnss_mode_to(location)
        if not gnss_resp.get("success"):
            msg = gnss_resp.get("message", "unknown error")
            logging.warning("⚠️ [clocks] GNSS MODE=TO failed: %s", msg)
            return {"success": False, "message": f"GNSS MODE=TO failed: {msg}", "payload": {"gnss_response": gnss_resp}}

        logging.info("✅ [clocks] GNSS confirmed TO mode")

    _set_pitimer_next_pps_count(0)
    _last_pps_count_seen = None

    _request_stop = False
    _request_start = True

    teensy_args: Dict[str, Any] = {"campaign": campaign, "pps_count": "0"}
    if set_dac is not None:
        teensy_args["set_dac"] = str(set_dac)
    if calibrate_ocxo:
        teensy_args["calibrate_ocxo"] = "true"

    send_command(machine="TEENSY", subsystem="CLOCKS", command="START", args=teensy_args)

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "campaign": campaign,
            "location": location,
            "set_dac": set_dac,
            "calibrate_ocxo": calibrate_ocxo,
            "correlation": "pps_count_campaign_fact",
            "note": "PITIMER set next pps_count=0; Teensy expected to start pps_count at 0",
        },
    }


def cmd_stop(_: Optional[dict]) -> dict:
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

    send_command(machine="TEENSY", subsystem="CLOCKS", command="STOP")

    if had_location:
        logging.info("📡 [clocks] restoring GNSS -> NORMAL (campaign had location '%s')", had_location)
        try:
            gnss_resp = _set_gnss_mode_normal()
            if not gnss_resp.get("success"):
                logging.warning("⚠️ [clocks] GNSS MODE=NORMAL failed: %s", gnss_resp.get("message", "?"))
        except Exception:
            logging.exception("⚠️ [clocks] GNSS MODE=NORMAL IPC failed (ignored)")

    return {"success": True, "message": "OK"}


def cmd_report(_: Optional[dict]) -> Dict[str, Any]:
    row = _get_active_campaign()

    if not row:
        return {
            "success": True,
            "message": "OK",
            "payload": {"campaign_state": "IDLE"},
        }

    return {"success": True, "message": "OK", "payload": row["payload"]}


def cmd_clear(_: Optional[dict]) -> dict:
    global _request_stop, _request_start, _campaign_active
    global _last_pi_seq, _last_pi_corrected
    global _last_pps_count_seen
    global _pi_tick_epoch, _pi_tick_epoch_set

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

        _last_pi_seq = None
        _last_pi_corrected = None
        _last_pps_count_seen = None

        _pi_tick_epoch_set = False
        _pi_tick_epoch = 0

        logging.info("🗑️ [clocks] CLEAR: deleted %d timebase rows, %d campaigns", tb_count, camp_count)

        return {
            "success": True,
            "message": "OK",
            "payload": {"timebase_deleted": tb_count, "campaigns_deleted": camp_count},
        }

    except Exception as e:
        logging.exception("❌ [clocks] CLEAR failed")
        return {"success": False, "message": str(e)}


def cmd_clocks_info(_: Optional[dict]) -> Dict[str, Any]:
    payload = {
        "campaign_active": _campaign_active,
        "pi_tick_epoch_set": _pi_tick_epoch_set,
        "pi_tick_epoch": _pi_tick_epoch if _pi_tick_epoch_set else None,
        "last_pi_seq": _last_pi_seq,
        "last_pi_corrected": _last_pi_corrected,
        "last_pps_count_seen": _last_pps_count_seen,
        "diag": _diag,
    }
    return {"success": True, "message": "OK", "payload": payload}


COMMANDS = {
    "START": cmd_start,
    "STOP": cmd_stop,
    "REPORT": cmd_report,
    "CLEAR": cmd_clear,
    "CLOCKS_INFO": cmd_clocks_info,
}

# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------


def run() -> None:
    setup_logging()

    logging.info(
        "🕐 [clocks] Four tracked clocks: GNSS (Teensy 10MHz), "
        "DWT (Teensy 600MHz), OCXO (Teensy GPT1 10MHz), "
        "Pi (CNTVCT_EL0 54MHz via PITIMER). "
        "Correlation is campaign-fact pps_count identity (PITIMER.SET_NEXT_PPS_COUNT + REPORT_PPS_COUNT). "
        "pps_count is intended to equal GNSS seconds since campaign inception; recovery projects it forward across downtime. "
        "Time labels (system_time_z / gnss_time_utc strings) are metadata only. "
        "Outlier rejection on all clock domains (>50%% deviation -> reject). "
        "Instrumentation enabled (CLOCKS_INFO)."
    )

    _recover_campaign()

    server_setup(
        subsystem="CLOCKS",
        commands=COMMANDS,
        subscriptions={
            "TIMEBASE_FRAGMENT": on_timebase_fragment,
        },
    )


if __name__ == "__main__":
    run()