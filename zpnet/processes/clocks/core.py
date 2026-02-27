"""
ZPNet CLOCKS Process — TIMEBASE Authority (Pi-side) — DRACONIAN pps_count

Core contract (v2026-02-25+):

  • CLOCKS chooses a target pps_count = k (START => 0, RECOVER => computed).
  • CLOCKS issues a *request-to-set* on BOTH PITIMER and TEENSY:
        1) PITIMER.SET_PPS_COUNT(k)
        2) TEENSY CLOCKS.START/RECOVER (pps_count=k)
     These requests take effect on the *next* PPS edge.

  • Within one second, TEENSY will publish TIMEBASE_FRAGMENT with teensy_pps_count == k.
    If not: HARD FAIL.

  • On receipt of a TIMEBASE_FRAGMENT, CLOCKS interrogates PITIMER.REPORT immediately and demands:
        PITIMER.pps_count == TEENSY.teensy_pps_count
    If not: HARD FAULT → automatic recovery (STOP Teensy, re-arm, re-sync).

  • PITIMER is single-capture only. No keyed lookup. No buffering semantics. No "historical" correlation.

Separation of concerns:
  • START / RECOVER own all waiting and synchronization.
  • on_timebase_fragment() is strict: correlate-or-die, then persist.

Responsibilities:
  * Receive TIMEBASE_FRAGMENT from Teensy
  * Fetch correlated Pi capture from PITIMER (single latest capture)
  * Demand pps_count identity match (hard fault on mismatch)
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
"""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from datetime import datetime, timezone
from typing import Dict, Any, Optional, Tuple

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
PI_NS_PER_TICK = 1e9 / PI_TIMER_FREQ  # ~18.519 ns/tick

DWT_CYCLES_PER_NS_NUM = 3
DWT_CYCLES_PER_NS_DEN = 5

GNSS_POLL_INTERVAL = 5
GNSS_WAIT_LOG_INTERVAL = 60

# Draconian sync waits
SYNC_FRAGMENT_TIMEOUT_S = 2.0    # generous: fragment should arrive within ~10ms
SYNC_RECOVER_TIMEOUT_S = 5.0    # recovery needs more time: Teensy reinit + PPS edge
SYNC_PITIMER_POLL_S = 0.005      # 5ms polling while awaiting sync edge
SYNC_LOG_INTERVAL_S = 0.25       # keep logs sparse but visible during sync waits

# ---------------------------------------------------------------------
# Diagnostics (monotonic counters + last anomaly snapshots)
# ---------------------------------------------------------------------

_diag: Dict[str, Any] = {
    # Fragment ingress
    "fragments_received": 0,
    "fragments_ignored_no_campaign": 0,
    "fragments_missing_teensy_pps_count": 0,

    # Draconian correlation failures
    "hard_fault_pitimer_report_failed": 0,
    "hard_fault_pitimer_pps_missing": 0,
    "hard_fault_pps_mismatch": 0,
    "hard_fault_pitimer_counter_missing": 0,
    "hard_fault_epoch_not_set": 0,
    "hard_fault_no_active_campaign": 0,
    "hard_fault_sync_timeout": 0,
    "last_hard_fault": {},
    "hard_faults_total": 0,
    "auto_recovery_failures": 0,

    # PITIMER control plane
    "pitimer_set_requests": 0,
    "pitimer_set_failures": 0,
    "pitimer_set_ipc_failures": 0,
    "last_pitimer_set": {},

    # Sync waits
    "sync_waits": 0,
    "sync_wait_success": 0,
    "sync_wait_seconds_total": 0.0,
    "sync_wait_seconds_last": 0.0,
    "last_sync_wait": {},

    # pps_count continuity (campaign fact, as observed from Teensy)
    "pps_count_seen": 0,
    "pps_count_repeat": 0,
    "pps_count_jump": 0,
    "pps_count_regress": 0,
    "last_pps_count": None,
    "armed_pps_count": None,
    "last_pps_count_anomaly": {},

    # Epoch anchoring
    "epoch_sets": 0,
    "last_epoch_set": {},

    # Recovery accounting
    "recovery_checks": 0,
    "recovery_no_active_campaign": 0,
    "recovery_missing_timebase": 0,
    "recovery_missing_last_pps_count": 0,
    "recovery_elapsed_seconds_nonpositive": 0,
    "last_recovery": {},

    # GNSS wait
    "gnss_waits": 0,
    "gnss_wait_success": 0,
    "gnss_wait_seconds_total": 0.0,
    "gnss_wait_seconds_last": 0.0,
    "last_gnss_wait": {},

    # Residual outliers (all clocks)
    "outliers_rejected_total": 0,
    "outliers_rejected_gnss": 0,
    "outliers_rejected_dwt": 0,
    "outliers_rejected_ocxo": 0,
    "outliers_rejected_pi": 0,
    "last_outlier": {},

    # Pi outlier classification
    "pi_delta_zero": 0,
    "pi_delta_double": 0,
    "pi_delta_other": 0,

    # Pi capture integrity (observational)
    "pi_capture_seq_missing": 0,
    "pi_capture_seq_repeat": 0,
    "pi_capture_seq_jump": 0,
    "pi_capture_corrected_repeat": 0,
}

# ---------------------------------------------------------------------
# PPS residual tracking -- Welford's online algorithm
# ---------------------------------------------------------------------


class _PpsResidual:
    """Per-clock PPS residual state with Welford's online statistics."""

    __slots__ = (
        "last_value", "delta", "residual", "valid",
        "n", "mean", "m2", "display_scale",
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
      * On reject: increment counters, advance anchor, skip Welford update.
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

_campaign_active: bool = False

_pi_tick_epoch: int = 0
_pi_tick_epoch_set: bool = False

_last_pps_count_seen: Optional[int] = None

# What pps_count did we most recently tell PITIMER/Teensy to use?
# Used for verbose diagnostics on mismatch.
_armed_pps_count: Optional[int] = None

# Pi capture integrity tracking (observational)
_last_pi_seq: Optional[int] = None
_last_pi_corrected: Optional[int] = None

_residual_gnss = _PpsResidual()
_residual_dwt = _PpsResidual()
_residual_pi = _PpsResidual(display_scale=PI_NS_PER_TICK)
_residual_ocxo = _PpsResidual()

# Draconian sync: control-plane waits for a specific fragment pps_count
_sync_lock = threading.Lock()
_sync_expected_pps: Optional[int] = None
_sync_event = threading.Event()
_sync_fragment: Optional[Dict[str, Any]] = None

# Epoch anchoring mode for next sync edge
#  • START:   epoch = pi_corrected_at_edge  (ticks at pps_count=0)
#  • RECOVER: epoch = pi_corrected_at_edge - projected_pi_ticks_at_target
_pending_epoch_mode: Optional[str] = None               # "START" | "RECOVER" | None
_pending_epoch_target_pps: Optional[int] = None
_pending_projected_pi_ticks_at_target: Optional[int] = None

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------


_auto_recovery_in_progress: bool = False


def _hard_fault(reason: str, details: Dict[str, Any]) -> None:
    """Log a hard fault and trigger automatic recovery.

    Instead of crashing, we deactivate the campaign (so subsequent fragments
    are ignored) and spawn a background thread to run _recover_campaign().
    This must NOT block the PUBSUB handler thread.
    """
    global _campaign_active, _auto_recovery_in_progress

    _diag["hard_faults_total"] = _diag.get("hard_faults_total", 0) + 1
    _diag["last_hard_fault"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "reason": reason,
        "details": details,
    }

    # Deactivate campaign so the handler ignores fragments while we recover
    _campaign_active = False

    if _auto_recovery_in_progress:
        logging.error(
            "💥 [clocks] HARD FAULT: %s — auto-recovery already in progress, skipping",
            reason,
        )
        raise RuntimeError(f"HARD FAULT: {reason} (recovery already in progress)")

    logging.error(
        "💥 [clocks] HARD FAULT: %s  details=%s — initiating auto-recovery",
        reason, details,
    )

    _auto_recovery_in_progress = True

    # Spawn recovery on a background thread so we don't block PUBSUB
    def _auto_recover():
        global _auto_recovery_in_progress
        try:
            logging.info("🔄 [clocks] @%s auto-recovery starting...", system_time_z())
            _recover_campaign()
            logging.info("✅ [clocks] @%s auto-recovery complete", system_time_z())
        except Exception:
            logging.exception("💥 [clocks] auto-recovery FAILED — campaign deactivated")
            _diag["auto_recovery_failures"] = _diag.get("auto_recovery_failures", 0) + 1
        finally:
            _auto_recovery_in_progress = False

    t = threading.Thread(target=_auto_recover, name="clocks-auto-recover", daemon=True)
    t.start()

    # Raise to abort the current fragment handler invocation
    raise RuntimeError(f"HARD FAULT: {reason} (auto-recovery initiated)")


def _ticks_to_ns(ticks: int) -> int:
    """Convert Pi tick count to nanoseconds via integer arithmetic."""
    return (ticks * NS_PER_SECOND) // PI_TIMER_FREQ


def _note_pps_count(teensy_pps_count: int) -> None:
    """Observational continuity check on the pps_count stream from Teensy."""
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


def _get_active_campaign() -> Optional[Dict[str, Any]]:
    """Return active campaign row or None."""
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


def _persist_timebase(tb: Dict[str, Any], report: Dict[str, Any]) -> None:
    """Append TIMEBASE row and denormalize report into campaign payload."""
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


def _wait_for_gnss_time() -> str:
    """Patiently wait for GNSS time to be available (ISO8601 Z). Instrumented."""
    _diag["gnss_waits"] += 1
    t0 = time.monotonic()
    last_log = t0
    start_utc = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")

    while True:
        try:
            resp = send_command(machine="PI", subsystem="GNSS", command="GET_TIME")
            if resp.get("success"):
                gnss_time = resp["payload"]["gnss_time"]
                waited = time.monotonic() - t0

                _diag["gnss_wait_success"] += 1
                _diag["gnss_wait_seconds_total"] += float(waited)
                _diag["gnss_wait_seconds_last"] = float(waited)
                _diag["last_gnss_wait"] = {
                    "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                    "start_utc": start_utc,
                    "waited_s": round(float(waited), 3),
                    "poll_interval_s": int(GNSS_POLL_INTERVAL),
                }
                return gnss_time
        except Exception:
            pass

        now = time.monotonic()
        if now - last_log >= GNSS_WAIT_LOG_INTERVAL:
            logging.info("⏳ [clocks] waiting for GNSS time... (%.0fs elapsed)", now - t0)
            last_log = now

        time.sleep(GNSS_POLL_INTERVAL)


def _set_gnss_mode_to(location: str) -> Dict[str, Any]:
    return send_command(
        machine="PI", subsystem="GNSS", command="MODE",
        args={"mode": "TO", "location": location},
    )


def _set_gnss_mode_normal() -> Dict[str, Any]:
    return send_command(
        machine="PI", subsystem="GNSS", command="MODE",
        args={"mode": "NORMAL"},
    )


# ---------------------------------------------------------------------
# PITIMER control plane (DRACONIAN — minimal surface)
# ---------------------------------------------------------------------


def _set_pitimer_pps_count(k: int) -> None:
    """
    Command PITIMER to arm pps_count = k.

    Semantics: the next PPS capture will be labeled pps_count==k,
    then PITIMER increments by 1 each subsequent PPS.

    Hard fault on failure — we cannot proceed without this.
    """
    _diag["pitimer_set_requests"] += 1
    try:
        resp = send_command(
            machine="PI",
            subsystem="PITIMER",
            command="SET_PPS_COUNT",
            args={"pps_count": int(k)},
        )
        ok = bool(resp.get("success", False))
        _diag["last_pitimer_set"] = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "pps_count": int(k),
            "success": ok,
            "response": resp.get("payload", {}) if isinstance(resp, dict) else {},
        }
        if not ok:
            _diag["pitimer_set_failures"] += 1
            _hard_fault("pitimer_set_failed", {"pps_count": int(k), "resp": resp})
    except RuntimeError:
        raise  # re-raise hard faults
    except Exception:
        _diag["pitimer_set_ipc_failures"] += 1
        _hard_fault("pitimer_set_ipc_failed", {"pps_count": int(k)})


def _pitimer_report() -> Dict[str, Any]:
    """
    Fetch the single latest capture from PITIMER.

    Returns the payload dict on success. Hard fault on failure.
    This is the ONLY way CLOCKS obtains Pi timer data.
    """
    try:
        resp = send_command(machine="PI", subsystem="PITIMER", command="REPORT")
        if not resp.get("success", False):
            _diag["hard_fault_pitimer_report_failed"] += 1
            _hard_fault("pitimer_report_failed", {"resp": resp})
        return resp.get("payload", {}) if isinstance(resp, dict) else {}
    except RuntimeError:
        raise  # re-raise hard faults
    except Exception:
        _diag["hard_fault_pitimer_report_failed"] += 1
        _hard_fault("pitimer_report_ipc_failed", {})


# ---------------------------------------------------------------------
# Draconian sync primitives
# ---------------------------------------------------------------------


def _begin_sync_wait(
    expected_pps: int,
    epoch_mode: str,
    projected_pi_ticks_at_target: Optional[int],
) -> None:
    """
    Prepare to wait for a TIMEBASE_FRAGMENT with a specific pps_count.

    Called BEFORE issuing SET_PPS_COUNT and START/RECOVER so the
    fragment handler can latch the correct fragment when it arrives.
    """
    global _sync_expected_pps, _sync_fragment
    global _pending_epoch_mode, _pending_epoch_target_pps, _pending_projected_pi_ticks_at_target

    with _sync_lock:
        _sync_expected_pps = int(expected_pps)
        _sync_fragment = None
        _pending_epoch_mode = epoch_mode
        _pending_epoch_target_pps = int(expected_pps)
        _pending_projected_pi_ticks_at_target = projected_pi_ticks_at_target
        _sync_event.clear()


def _end_sync_wait(timeout_s: float = SYNC_FRAGMENT_TIMEOUT_S) -> Tuple[Dict[str, Any], float]:
    """
    Block until the sync fragment arrives or timeout. Hard fault on timeout.

    Returns (fragment_dict, waited_seconds).
    """
    global _sync_expected_pps

    t0 = time.monotonic()
    _diag["sync_waits"] += 1

    last_log = t0
    while True:
        remaining = timeout_s - (time.monotonic() - t0)
        if remaining <= 0:
            _diag["hard_fault_sync_timeout"] += 1
            _hard_fault("sync_timeout_waiting_for_fragment", {
                "expected_pps_count": _sync_expected_pps,
                "timeout_s": timeout_s,
            })
        if _sync_event.wait(timeout=min(remaining, SYNC_PITIMER_POLL_S)):
            break
        now = time.monotonic()
        if now - last_log >= SYNC_LOG_INTERVAL_S:
            logging.info("⏳ [clocks] waiting for TIMEBASE_FRAGMENT pps_count=%s...", str(_sync_expected_pps))
            last_log = now

    waited = time.monotonic() - t0
    _diag["sync_wait_success"] += 1
    _diag["sync_wait_seconds_total"] += float(waited)
    _diag["sync_wait_seconds_last"] = float(waited)
    _diag["last_sync_wait"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "expected_pps_count": int(_sync_expected_pps or -1),
        "waited_s": round(float(waited), 3),
        "timeout_s": float(timeout_s),
    }

    with _sync_lock:
        frag = dict(_sync_fragment or {})
        _sync_expected_pps = None
        return frag, float(waited)


# ---------------------------------------------------------------------
# Epoch anchoring (applied inside fragment handler on sync edge)
# ---------------------------------------------------------------------


def _apply_epoch_if_pending(teensy_pps_count: int, pi_corrected: int) -> None:
    """
    Set the Pi tick epoch on the first edge after START or RECOVER.

    START:   epoch = pi_corrected  (at pps_count=0, campaign ticks are zero)
    RECOVER: epoch = pi_corrected - projected_pi_ticks_at_target
    """
    global _pi_tick_epoch, _pi_tick_epoch_set
    global _pending_epoch_mode, _pending_epoch_target_pps, _pending_projected_pi_ticks_at_target

    if _pi_tick_epoch_set:
        return

    if _pending_epoch_mode is None or _pending_epoch_target_pps is None:
        _hard_fault("epoch_not_pending_but_unset", {"teensy_pps_count": int(teensy_pps_count)})

    if int(teensy_pps_count) != int(_pending_epoch_target_pps):
        _hard_fault("epoch_target_mismatch", {
            "want": int(_pending_epoch_target_pps),
            "got": int(teensy_pps_count),
        })

    if _pending_epoch_mode == "START":
        _pi_tick_epoch = int(pi_corrected)
        _pi_tick_epoch_set = True

    elif _pending_epoch_mode == "RECOVER":
        if _pending_projected_pi_ticks_at_target is None:
            _hard_fault("recover_epoch_missing_projection", {"pps_count": int(teensy_pps_count)})
        _pi_tick_epoch = int(pi_corrected) - int(_pending_projected_pi_ticks_at_target)
        _pi_tick_epoch_set = True

    else:
        _hard_fault("unknown_epoch_mode", {"mode": str(_pending_epoch_mode)})

    _diag["epoch_sets"] += 1
    _diag["last_epoch_set"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "mode": str(_pending_epoch_mode),
        "pps_count": int(teensy_pps_count),
        "pi_corrected": int(pi_corrected),
        "pi_tick_epoch": int(_pi_tick_epoch),
        "projected_pi_ticks_at_target": _pending_projected_pi_ticks_at_target,
    }

    logging.info(
        "✅ [clocks] epoch set mode=%s pps_count=%d epoch=%d",
        _diag["last_epoch_set"]["mode"],
        int(teensy_pps_count),
        int(_pi_tick_epoch),
    )

    # Clear pending epoch request
    _pending_epoch_mode = None
    _pending_epoch_target_pps = None
    _pending_projected_pi_ticks_at_target = None


# ---------------------------------------------------------------------
# Clock statistics helpers
# ---------------------------------------------------------------------


def _seconds_to_hms(seconds: int) -> str:
    h = seconds // 3600
    m = (seconds % 3600) // 60
    s = seconds % 60
    return f"{h:02d}:{m:02d}:{s:02d}"


def _compute_tau(clock_ns: int, gnss_ns: int) -> float:
    return (clock_ns / gnss_ns) if gnss_ns else 0.0


def _compute_ppb(clock_ns: int, gnss_ns: int) -> float:
    return (((clock_ns - gnss_ns) / gnss_ns) * 1e9) if gnss_ns else 0.0


def _build_clock_block(
    ns_now: int,
    gnss_ns: int,
    residual: _PpsResidual,
) -> Dict[str, Any]:
    block: Dict[str, Any] = {
        "ns_now": int(ns_now),
        "tau": round(_compute_tau(int(ns_now), int(gnss_ns)), 12),
        "ppb": round(_compute_ppb(int(ns_now), int(gnss_ns)), 3),
    }
    block.update(residual.to_dict())
    return block


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
    campaign_seconds = int(pps_count) if isinstance(pps_count, int) else (gnss_ns // NS_PER_SECOND if gnss_ns else 0)

    return {
        "campaign": campaign_name,
        "campaign_state": "STARTED",
        "campaign_seconds": int(campaign_seconds),
        "campaign_elapsed": _seconds_to_hms(int(campaign_seconds)),
        "location": campaign_payload.get("location"),
        "gnss_time_utc": timebase.get("gnss_time_utc"),
        "system_time_utc": timebase.get("system_time_utc"),
        "pps_count": int(timebase.get("pps_count") or 0),

        "teensy_dwt_cycles": timebase.get("teensy_dwt_cycles"),
        "teensy_ocxo_ns": timebase.get("teensy_ocxo_ns"),
        "teensy_rtc1_ns": timebase.get("teensy_rtc1_ns"),
        "teensy_rtc2_ns": timebase.get("teensy_rtc2_ns"),
        "pi_counter": timebase.get("pi_counter"),
        "pi_corrected": timebase.get("pi_corrected"),

        "gnss": _build_clock_block(gnss_ns, gnss_ns, _residual_gnss),
        "dwt": _build_clock_block(dwt_ns, gnss_ns, _residual_dwt),
        "pi": _build_clock_block(pi_ns, gnss_ns, _residual_pi),
        "ocxo": _build_clock_block(ocxo_ns, gnss_ns, _residual_ocxo),
    }


# ---------------------------------------------------------------------
# Draconian fragment handler
# ---------------------------------------------------------------------


def on_timebase_fragment(payload: Payload) -> None:
    """
    TIMEBASE_FRAGMENT handler — strict correlate-or-die.

    Flow:
      1. Extract teensy_pps_count from fragment.
      2. If sync-wait is active and pps_count matches, latch fragment and signal.
      3. If no active campaign, ignore.
      4. Call PITIMER.REPORT — get the single latest capture.
      5. DEMAND: pitimer.pps_count == teensy_pps_count. Hard fault on mismatch.
      6. If epoch not yet set, apply it (first edge after START/RECOVER).
      7. Compute campaign-relative Pi ns, update residuals, build TIMEBASE, persist.
    """
    global _campaign_active, _sync_fragment
    global _last_pi_seq, _last_pi_corrected
    global _armed_pps_count

    _diag["fragments_received"] += 1

    frag = payload

    # --- Extract pps_count from Teensy fragment ---
    teensy_pps_count_raw = frag.get("teensy_pps_count")
    if teensy_pps_count_raw is None:
        _diag["fragments_missing_teensy_pps_count"] += 1
        logging.warning("⚠️ [clocks] fragment missing teensy_pps_count (keys=%s)", list(frag.keys())[:10])
        return

    try:
        pps_count = int(teensy_pps_count_raw)
    except Exception:
        _diag["fragments_missing_teensy_pps_count"] += 1
        return

    # DEBUG: log every fragment during sync wait so we can see what's arriving
    with _sync_lock:
        if _sync_expected_pps is not None:
            logging.info(
                "🔎 [clocks] @%s fragment arrived: teensy_pps_count=%d (waiting for %d) match=%s",
                system_time_z(), pps_count, _sync_expected_pps, str(pps_count == _sync_expected_pps),
            )

    _note_pps_count(pps_count)

    # --- Sync latch: if we're waiting for this specific pps_count, grab it ---
    with _sync_lock:
        if _sync_expected_pps is not None and int(pps_count) == int(_sync_expected_pps):
            _sync_fragment = dict(frag)
            _sync_event.set()

    # --- No campaign => ignore ---
    if not _campaign_active:
        _diag["fragments_ignored_no_campaign"] += 1
        return

    # --- DRACONIAN CORRELATION ---
    # Check Teensy pps_count against what we armed (if tracked)
    armed = _armed_pps_count
    sysclk = system_time_z()
    if armed is not None and int(pps_count) != armed:
        _diag["hard_fault_pps_mismatch"] += 1
        delta = int(pps_count) - armed
        logging.error(
            "💥 [clocks] @%s TEENSY PPS COUNT MISMATCH! "
            "I told Teensy the next second is %d but Teensy sent me pps_count=%d (off by %+d).",
            sysclk, armed, int(pps_count), delta,
        )
        _hard_fault("pps_mismatch_teensy_vs_armed", {
            "system_time": sysclk,
            "armed_pps_count": armed,
            "teensy_pps_count": int(pps_count),
            "delta": delta,
        })

    # Give PITIMER's ppspoll thread time to detect and parse the PPS edge.
    # The Teensy fragment arrives ~1.5ms after PPS, but PITIMER's subprocess
    # may still be reading/parsing. 100ms is generous and deterministic.
    time.sleep(0.100)

    # Ask PITIMER for its latest capture. It should be for THIS PPS edge.
    pit = _pitimer_report()

    pit_pps = pit.get("pps_count")
    if pit_pps is None:
        _diag["hard_fault_pitimer_pps_missing"] += 1
        logging.error(
            "💥 [clocks] @%s PITIMER returned no pps_count! "
            "I told PITIMER the next second is %s but it has no count assigned. "
            "Teensy sent pps_count=%d. PITIMER state: %s",
            sysclk, str(armed), int(pps_count), pit.get("state", "?"),
        )
        _hard_fault("pitimer_pps_missing", {
            "system_time": sysclk,
            "armed_pps_count": armed,
            "teensy_pps_count": int(pps_count),
            "pitimer": pit,
        })

    if int(pit_pps) != int(pps_count):
        _diag["hard_fault_pps_mismatch"] += 1
        delta = int(pit_pps) - int(pps_count)
        logging.error(
            "💥 [clocks] @%s PITIMER PPS COUNT MISMATCH! "
            "I told PITIMER the next second is %s. "
            "I told Teensy the next second is %s. "
            "Teensy sent me pps_count=%d (good). "
            "But PITIMER reports pps_count=%d (off by %+d).",
            sysclk, str(armed), str(armed),
            int(pps_count), int(pit_pps), delta,
        )
        _hard_fault("pps_mismatch_teensy_vs_pitimer", {
            "system_time": sysclk,
            "armed_pps_count": armed,
            "teensy_pps_count": int(pps_count),
            "pitimer_pps_count": int(pit_pps),
            "delta": delta,
            "pitimer": pit,
        })

    logging.debug(
        "✅ [clocks] @%s draconian correlation OK: armed=%s teensy=%d pitimer=%d",
        sysclk, str(armed), int(pps_count), int(pit_pps),
    )

    # Advance armed expectation to next second
    _armed_pps_count = int(pps_count) + 1
    _diag["armed_pps_count"] = _armed_pps_count

    # --- Extract Pi counter from PITIMER capture ---
    pi_counter_raw = pit.get("counter")
    pi_corrected = pit.get("corrected") or pi_counter_raw
    if pi_corrected is None:
        _diag["hard_fault_pitimer_counter_missing"] += 1
        _hard_fault("pitimer_missing_counter", {
            "pps_count": int(pps_count),
            "pitimer": pit,
        })

    pi_corrected = int(pi_corrected)

    # Pi capture diagnostics (observational)
    pi_detect_ns = pit.get("detect_ns")
    pi_correction_ticks = pit.get("correction_ticks", 0)
    pi_seq = pit.get("seq")
    pi_delta = pit.get("delta")
    pi_residual = pit.get("residual")
    pi_residual_ns = pit.get("residual_ns")

    if pi_seq is None:
        _diag["pi_capture_seq_missing"] += 1
    if isinstance(pi_seq, int):
        if _last_pi_seq is not None:
            if pi_seq == _last_pi_seq:
                _diag["pi_capture_seq_repeat"] += 1
            elif pi_seq > (_last_pi_seq + 1):
                _diag["pi_capture_seq_jump"] += 1
        _last_pi_seq = pi_seq

    if _last_pi_corrected is not None and pi_corrected == _last_pi_corrected:
        _diag["pi_capture_corrected_repeat"] += 1
    _last_pi_corrected = pi_corrected

    # --- Epoch anchoring (first edge after START/RECOVER) ---
    if not _pi_tick_epoch_set:
        _apply_epoch_if_pending(teensy_pps_count=int(pps_count), pi_corrected=int(pi_corrected))

    # HARD GATE: epoch must exist
    if not _pi_tick_epoch_set:
        _diag["hard_fault_epoch_not_set"] += 1
        _hard_fault("pi_epoch_not_set_after_apply", {"pps_count": int(pps_count)})

    # --- Compute campaign-relative Pi nanoseconds ---
    system_time_utc = datetime.now(timezone.utc)
    system_time_str = system_time_utc.isoformat(timespec="microseconds")

    campaign_ticks = int(pi_corrected) - int(_pi_tick_epoch)
    pi_ns = _ticks_to_ns(int(campaign_ticks))

    gnss_time_utc_label = pit.get("gnss_time") or system_time_z()

    # --- Residual updates ---
    gnss_ns = int(frag.get("gnss_ns") or 0)
    dwt_ns = int(frag.get("dwt_ns") or 0)
    ocxo_ns = int(frag.get("ocxo_ns") or 0)

    if gnss_ns > 0:
        _safe_residual_update(_residual_gnss, gnss_ns, NS_PER_SECOND, "GNSS")
    if dwt_ns > 0:
        _safe_residual_update(_residual_dwt, dwt_ns, NS_PER_SECOND, "DWT")
    if ocxo_ns > 0:
        _safe_residual_update(_residual_ocxo, ocxo_ns, NS_PER_SECOND, "OCXO")
    _safe_residual_update(_residual_pi, pi_corrected, PI_TIMER_FREQ, "Pi")

    # --- Campaign lookup ---
    row = _get_active_campaign()
    if row is None:
        _diag["hard_fault_no_active_campaign"] += 1
        _hard_fault("no_active_campaign", {"pps_count": int(pps_count)})

    campaign = row["campaign"]
    campaign_payload = row["payload"]

    # --- Build TIMEBASE record ---
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

        # Preserve raw Teensy field (should match pps_count)
        "teensy_pps_count": int(pps_count),

        # Pi capture (single-edge, from PITIMER.REPORT)
        "pi_counter": pi_counter_raw,
        "pi_corrected": pi_corrected,
        "pi_ns": int(pi_ns),

        # Pi capture diagnostics
        "diag_pi_raw_counter": pi_counter_raw,
        "diag_pi_corrected": pi_corrected,
        "diag_pi_detect_ns": pi_detect_ns,
        "diag_pi_correction_ticks": pi_correction_ticks,
        "diag_pi_correction_ns": round(pi_correction_ticks * PI_NS_PER_TICK, 3) if pi_correction_ticks is not None else None,
        "diag_pi_seq": pi_seq,
        "diag_pi_delta": pi_delta,
        "diag_pi_residual": pi_residual,
        "diag_pi_residual_ns": pi_residual_ns,
        "diag_pitimer_edge_source": pit.get("edge_source"),

        # Teensy ISR residuals + derived Pi residual (in ns)
        "isr_residual_gnss": frag.get("isr_residual_gnss"),
        "isr_residual_dwt": frag.get("isr_residual_dwt"),
        "isr_residual_ocxo": frag.get("isr_residual_ocxo"),
        "isr_residual_pi": round(_residual_pi.residual * PI_NS_PER_TICK, 3) if _residual_pi.valid else None,

        # Teensy dispatch profiling / PPS diagnostics
        "diag_teensy_dispatch_delta_ns": frag.get("dispatch_delta_ns"),
        "diag_teensy_pps_edge_valid": frag.get("pps_edge_valid"),
        "diag_teensy_pps_edge_correction_ns": frag.get("pps_edge_correction_ns"),

        # OCXO control state
        "ocxo_dac": frag.get("ocxo_dac"),
        "calibrate_ocxo": frag.get("calibrate_ocxo"),
        "servo_converged": frag.get("servo_converged"),
    }

    report = _build_report(campaign, campaign_payload, timebase)

    publish("TIMEBASE", timebase)
    _persist_timebase(timebase, report)

    # Persist OCXO DAC fields into campaign payload (best-effort)
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


# ---------------------------------------------------------------------
# Control-plane: START / STOP / CLEAR / RECOVER
# ---------------------------------------------------------------------


def _reset_trackers() -> None:
    global _last_pps_count_seen, _last_pi_seq, _last_pi_corrected
    _last_pps_count_seen = None
    _last_pi_seq = None
    _last_pi_corrected = None
    _residual_gnss.reset()
    _residual_dwt.reset()
    _residual_pi.reset()
    _residual_ocxo.reset()


def _request_teensy_stop_best_effort() -> None:
    try:
        send_command(machine="TEENSY", subsystem="CLOCKS", command="STOP")
    except Exception:
        pass


def _request_teensy_start(campaign: str, pps_count: int, args: Dict[str, Any]) -> None:
    teensy_args = dict(args)
    teensy_args.update({"campaign": campaign, "pps_count": str(int(pps_count))})
    send_command(machine="TEENSY", subsystem="CLOCKS", command="START", args=teensy_args)


def _request_teensy_recover(pps_count: int, args: Dict[str, Any]) -> None:
    teensy_args = dict(args)
    teensy_args["pps_count"] = str(int(pps_count))
    send_command(machine="TEENSY", subsystem="CLOCKS", command="RECOVER", args=teensy_args)


def cmd_start(args: Optional[dict]) -> dict:
    """
    START (draconian):
      1. Create campaign in DB (deactivate any prior).
      2. Stop Teensy (best-effort).
      3. Reset epoch + trackers.
      4. Begin sync wait for fragment pps_count=0.
      5. PITIMER.SET_PPS_COUNT(0).
      6. TEENSY START(pps_count=0).
      7. Wait <= timeout for fragment pps_count=0 (or hard fail).
      8. On that fragment, handler sets epoch (START mode) and begins producing TIMEBASE.
    """
    global _campaign_active, _pi_tick_epoch_set, _pi_tick_epoch, _armed_pps_count

    campaign = args.get("campaign") if args else None
    if not campaign:
        return {"success": False, "message": "START requires 'campaign' argument"}

    location = (args.get("location") or "").strip() or None
    set_dac = args.get("set_dac")
    calibrate_ocxo = bool(args.get("calibrate_ocxo"))

    # Read default DAC from config if not specified
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

    campaign_payload = {
        "location": location,
        "started_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
    }
    if set_dac is not None:
        campaign_payload["ocxo_dac"] = int(set_dac)
    if calibrate_ocxo:
        campaign_payload["calibrate_ocxo"] = True

    # Deactivate prior campaigns + create new one
    with open_db() as conn:
        cur = conn.cursor()
        cur.execute(
            """
            UPDATE campaigns
            SET active = false,
                payload = payload || jsonb_build_object('stopped_at', to_jsonb(%s::text))
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

    # GNSS Time Only mode if location specified
    if location:
        logging.info("📡 [clocks] commanding GNSS -> TO mode for location '%s'", location)
        gnss_resp = _set_gnss_mode_to(location)
        if not gnss_resp.get("success"):
            msg = gnss_resp.get("message", "unknown error")
            logging.warning("⚠️ [clocks] GNSS MODE=TO failed: %s", msg)
            return {"success": False, "message": f"GNSS MODE=TO failed: {msg}", "payload": {"gnss_response": gnss_resp}}
        logging.info("✅ [clocks] GNSS confirmed TO mode")

    _request_teensy_stop_best_effort()

    # Reset epoch + trackers
    _pi_tick_epoch = 0
    _pi_tick_epoch_set = False
    _reset_trackers()

    # Prepare sync wait FIRST, then issue both requests-to-set
    _begin_sync_wait(expected_pps=0, epoch_mode="START", projected_pi_ticks_at_target=None)

    # Arm TEENSY first — slow path (serial round-trip)
    logging.info("📡 [start] @%s arming TEENSY START (slow path): next PPS edge will be pps_count=0", system_time_z())
    teensy_args: Dict[str, Any] = {"campaign": campaign}
    if set_dac is not None:
        teensy_args["set_dac"] = str(int(set_dac))
    if calibrate_ocxo:
        teensy_args["calibrate_ocxo"] = "true"

    _request_teensy_start(campaign=campaign, pps_count=0, args=teensy_args)
    logging.info("📡 [start] @%s TEENSY START returned — now arming PITIMER", system_time_z())

    # Arm PITIMER last — fast path (local IPC, microseconds)
    logging.info("📡 [start] @%s arming PITIMER: next PPS edge will be pps_count=0", system_time_z())
    _set_pitimer_pps_count(0)

    _armed_pps_count = 0
    _diag["armed_pps_count"] = 0
    logging.info("📡 [start] @%s both PITIMER and TEENSY armed for pps_count=0 — waiting for sync fragment...", system_time_z())

    # Activate campaign BEFORE waiting so the sync fragment gets fully
    # processed (including epoch application) when it arrives.
    _campaign_active = True

    # Wait for the sync fragment
    try:
        frag0, waited_s = _end_sync_wait()
    except Exception:
        _campaign_active = False
        return {"success": False, "message": "HARD FAULT during START sync (see logs/diag)"}

    logging.info("▶️ [clocks] @%s START sync achieved pps_count=0 (waited=%.3fs)", system_time_z(), waited_s)

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "campaign": campaign,
            "location": location,
            "set_dac": set_dac,
            "calibrate_ocxo": calibrate_ocxo,
            "pps_count": 0,
            "waited_s": round(float(waited_s), 3),
            "note": "epoch set by handler on sync fragment; TIMEBASE begins immediately",
        },
    }


def cmd_stop(_: Optional[dict]) -> dict:
    global _campaign_active, _pi_tick_epoch_set, _pi_tick_epoch

    row = _get_active_campaign()
    had_location = row["payload"].get("location") if row else None

    _request_teensy_stop_best_effort()

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

    _campaign_active = False
    _pi_tick_epoch_set = False
    _pi_tick_epoch = 0
    _reset_trackers()

    if had_location:
        logging.info("📡 [clocks] restoring GNSS -> NORMAL (campaign had location '%s')", had_location)
        try:
            gnss_resp = _set_gnss_mode_normal()
            if not gnss_resp.get("success"):
                logging.warning("⚠️ [clocks] GNSS MODE=NORMAL failed: %s", gnss_resp.get("message", "?"))
        except Exception:
            logging.exception("⚠️ [clocks] GNSS MODE=NORMAL IPC failed (ignored)")

    logging.info("⏹️ [clocks] campaign stopped")
    return {"success": True, "message": "OK"}


def cmd_report(_: Optional[dict]) -> Dict[str, Any]:
    row = _get_active_campaign()
    if not row:
        return {"success": True, "message": "OK", "payload": {"campaign_state": "IDLE"}}
    return {"success": True, "message": "OK", "payload": row["payload"]}


def cmd_clear(_: Optional[dict]) -> dict:
    global _campaign_active, _pi_tick_epoch_set, _pi_tick_epoch

    _request_teensy_stop_best_effort()

    try:
        with open_db() as conn:
            cur = conn.cursor()
            cur.execute("DELETE FROM timebase")
            tb_count = cur.rowcount
            cur.execute("DELETE FROM campaigns")
            camp_count = cur.rowcount

        _campaign_active = False
        _pi_tick_epoch_set = False
        _pi_tick_epoch = 0
        _reset_trackers()

        logging.info("🗑️ [clocks] CLEAR: deleted %d timebase rows, %d campaigns", tb_count, camp_count)
        return {"success": True, "message": "OK", "payload": {"timebase_deleted": tb_count, "campaigns_deleted": camp_count}}

    except Exception as e:
        logging.exception("❌ [clocks] CLEAR failed")
        return {"success": False, "message": str(e)}


# ---------------------------------------------------------------------
# Recovery (draconian)
# ---------------------------------------------------------------------


def _recover_campaign() -> None:
    """
    RECOVER (draconian):
      1. Read last TIMEBASE row, compute next_pps_count.
      2. Wait for GNSS time availability.
      3. Restore GNSS TO mode if campaign had a location.
      4. Begin sync wait for fragment pps_count=next_pps_count.
      5. PITIMER.SET_PPS_COUNT(next_pps_count).
      6. TEENSY RECOVER(pps_count=next_pps_count, projected clocks).
      7. Wait <= timeout for fragment pps_count=next_pps_count (or hard fail).
      8. On that fragment, handler:
           - calls PITIMER.REPORT
           - demands PITIMER.pps_count == fragment.teensy_pps_count
           - sets epoch (RECOVER mode) using projected_pi_ticks_at_target
           - begins TIMEBASE immediately
    """
    global _campaign_active, _pi_tick_epoch_set, _pi_tick_epoch, _armed_pps_count

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

    # Read last TIMEBASE row
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

    last_system_utc_str = last_tb.get("system_time_utc") or last_tb.get("gnss_time_utc") or system_time_z()
    last_system_utc = datetime.fromisoformat(last_system_utc_str.replace("Z", "+00:00"))

    last_pps_count = last_tb.get("pps_count") or last_tb.get("teensy_pps_count")
    if last_pps_count is None:
        _diag["recovery_missing_last_pps_count"] += 1
        raise RuntimeError("recovery failed: last TIMEBASE missing pps_count")
    last_pps_count = int(last_pps_count)

    last_dwt_ns = int(last_tb.get("teensy_dwt_ns") or 0)
    last_gnss_ns = int(last_tb.get("teensy_gnss_ns") or 0)
    last_ocxo_ns = int(last_tb.get("teensy_ocxo_ns") or 0)
    last_pi_ns = int(last_tb.get("pi_ns") or 0)

    if last_gnss_ns == 0:
        logging.warning(
            "⚠️ [recovery] teensy_gnss_ns not available in TIMEBASE -- "
            "using dwt_ns as GNSS proxy (tau_dwt assumed ~ 1.0)"
        )
        last_gnss_ns = last_dwt_ns

    logging.info(
        "🔍 [recovery] last TIMEBASE: system_utc=%s pps_count=%d gnss_ns=%d dwt_ns=%d ocxo_ns=%d pi_ns=%d",
        last_system_utc_str, last_pps_count, last_gnss_ns, last_dwt_ns, last_ocxo_ns, last_pi_ns,
    )

    recover_ocxo_dac = campaign_payload.get("ocxo_dac")
    recover_calibrate = campaign_payload.get("calibrate_ocxo", False)
    recover_converged = campaign_payload.get("servo_converged", False)

    if recover_ocxo_dac is not None:
        logging.info(
            "🔧 [recovery] OCXO DAC: %d (calibrate=%s, converged=%s)",
            recover_ocxo_dac, recover_calibrate, recover_converged,
        )

    # Wait for GNSS
    logging.info("⏳ [recovery] waiting for GNSS time to become available...")
    gnss_time_preflight = _wait_for_gnss_time()
    logging.info("✅ [recovery] GNSS time available: %s", gnss_time_preflight)

    # Restore GNSS TO mode if campaign had location
    if campaign_location:
        logging.info("📡 [recovery] restoring GNSS -> TO mode for location '%s'", campaign_location)
        gnss_resp = _set_gnss_mode_to(campaign_location)
        if not gnss_resp.get("success"):
            raise RuntimeError(f"recovery failed: GNSS MODE=TO failed: {gnss_resp.get('message', '?')}")
        logging.info("✅ [recovery] GNSS confirmed TO mode")

    # ---- Wait for routing to be live ----
    logging.info("⏳ [recovery] @%s waiting for first TIMEBASE_FRAGMENT to confirm routing is live...", system_time_z())
    _diag["fragments_received"]  # just reference; we'll poll this
    frag_wait_t0 = time.monotonic()
    frag_count_before = _diag["fragments_received"]
    ROUTING_WAIT_TIMEOUT_S = 30.0
    ROUTING_WAIT_POLL_S = 0.25
    while True:
        elapsed_wait = time.monotonic() - frag_wait_t0
        if _diag["fragments_received"] > frag_count_before:
            logging.info(
                "✅ [recovery] @%s routing confirmed — fragment received after %.1fs",
                system_time_z(), elapsed_wait,
            )
            break
        if elapsed_wait >= ROUTING_WAIT_TIMEOUT_S:
            raise RuntimeError(
                f"recovery failed: no TIMEBASE_FRAGMENT received in {ROUTING_WAIT_TIMEOUT_S}s "
                "(PUBSUB routing may not be established)"
            )
        time.sleep(ROUTING_WAIT_POLL_S)

    # ---- Stop Teensy ----
    logging.info("📡 [recovery] @%s sending Teensy STOP — routing is confirmed, now quiescing", system_time_z())
    _request_teensy_stop_best_effort()
    logging.info("⏳ [recovery] @%s waiting 2s for Teensy to quiesce...", system_time_z())
    time.sleep(2.0)
    logging.info("✅ [recovery] @%s Teensy quiesce complete", system_time_z())

    # ---- Snap to second boundary ----
    now_frac = time.time() % 1.0
    sleep_to_boundary = (1.0 - now_frac) + 0.050  # 50ms past the rollover
    logging.info(
        "⏳ [recovery] @%s snapping to second boundary (sleeping %.3fs)...",
        system_time_z(), sleep_to_boundary,
    )
    time.sleep(sleep_to_boundary)

    # Compute pps_count projection
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
        "now_utc": now_str,
        "elapsed_seconds": int(elapsed_seconds),
        "project_seconds": int(project_seconds),
        "last_pps_count": int(last_pps_count),
        "next_pps_count": int(next_pps_count),
    }

    logging.info(
        "📐 [recovery] pps_count continuity: last=%d elapsed=%d next=%d",
        last_pps_count, elapsed_seconds, next_pps_count,
    )

    # Project clocks to target edge
    tau_dwt = float(last_dwt_ns) / float(last_gnss_ns) if last_gnss_ns > 0 else 1.0
    tau_ocxo = float(last_ocxo_ns) / float(last_gnss_ns) if (last_gnss_ns > 0 and last_ocxo_ns > 0) else 1.0
    tau_pi = float(last_pi_ns) / float(last_gnss_ns) if (last_gnss_ns > 0 and last_pi_ns > 0) else 1.0

    logging.info("📐 [recovery] tau -- dwt=%.12f  ocxo=%.12f  pi=%.12f", tau_dwt, tau_ocxo, tau_pi)

    projected_gnss_ns = last_gnss_ns + (project_seconds * NS_PER_SECOND)
    projected_dwt_ns = last_dwt_ns + int(project_seconds * NS_PER_SECOND * tau_dwt)
    projected_ocxo_ns = last_ocxo_ns + int(project_seconds * NS_PER_SECOND * tau_ocxo) if last_ocxo_ns > 0 else 0
    projected_pi_ns = last_pi_ns + int(project_seconds * NS_PER_SECOND * tau_pi) if last_pi_ns > 0 else 0

    projected_dwt_cycles = (projected_dwt_ns * DWT_CYCLES_PER_NS_NUM) // DWT_CYCLES_PER_NS_DEN
    projected_pi_ticks_at_target = (int(projected_pi_ns) * PI_TIMER_FREQ) // NS_PER_SECOND

    if projected_pi_ns <= 0:
        raise RuntimeError("recovery failed: projected_pi_ns <= 0 (cannot anchor epoch)")

    logging.info(
        "📐 [recovery] projected@target -- gnss_ns=%d dwt_cycles=%d dwt_ns=%d ocxo_ns=%d pi_ns=%d pi_ticks=%d pps_count=%d",
        projected_gnss_ns, projected_dwt_cycles, projected_dwt_ns,
        projected_ocxo_ns, projected_pi_ns, projected_pi_ticks_at_target, next_pps_count,
    )

    # Reset epoch + trackers
    _pi_tick_epoch = 0
    _pi_tick_epoch_set = False
    _reset_trackers()

    # Begin sync wait FIRST
    _begin_sync_wait(
        expected_pps=int(next_pps_count),
        epoch_mode="RECOVER",
        projected_pi_ticks_at_target=int(projected_pi_ticks_at_target),
    )

    # Arm TEENSY first (slow path)
    logging.info(
        "📡 [recovery] @%s arming TEENSY RECOVER (slow path): next PPS edge will be pps_count=%d",
        system_time_z(), int(next_pps_count),
    )
    recover_args: Dict[str, Any] = {
        "dwt_cycles": str(int(projected_dwt_cycles)),
        "gnss_ns": str(int(projected_gnss_ns)),
        "ocxo_ns": str(int(projected_ocxo_ns)),
    }

    if recover_ocxo_dac is not None:
        recover_args["set_dac"] = str(int(recover_ocxo_dac))
    if recover_calibrate and not recover_converged:
        recover_args["calibrate_ocxo"] = "true"

    _request_teensy_recover(int(next_pps_count), recover_args)
    logging.info(
        "📡 [recovery] @%s TEENSY RECOVER returned — now arming PITIMER",
        system_time_z(),
    )

    # Arm PITIMER last (fast path)
    logging.info(
        "📡 [recovery] @%s arming PITIMER: next PPS edge will be pps_count=%d",
        system_time_z(), int(next_pps_count),
    )
    _set_pitimer_pps_count(int(next_pps_count))

    _armed_pps_count = int(next_pps_count)
    _diag["armed_pps_count"] = int(next_pps_count)
    logging.info(
        "📡 [recovery] @%s both PITIMER and TEENSY armed for pps_count=%d — waiting for sync fragment...",
        system_time_z(), int(next_pps_count),
    )

    # Activate campaign BEFORE waiting
    _campaign_active = True

    # Wait for sync fragment
    try:
        frag, waited_s = _end_sync_wait(timeout_s=SYNC_RECOVER_TIMEOUT_S)
    except Exception:
        _campaign_active = False
        raise
    logging.info(
        "✅ [recovery] @%s sync achieved pps_count=%d (waited=%.3fs) -- TIMEBASE resumes",
        system_time_z(), int(next_pps_count), float(waited_s),
    )


# ---------------------------------------------------------------------
# BASELINE — persist and retrieve baseline campaign for comparison
# ---------------------------------------------------------------------


def _get_baseline_from_config() -> Optional[Dict[str, Any]]:
    """Read baseline_id and baseline_ppb from SYSTEM config. Returns None if unset."""
    try:
        with open_db(row_dict=True) as conn:
            cur = conn.cursor()
            cur.execute("SELECT payload FROM config WHERE config_key = 'SYSTEM'")
            row = cur.fetchone()
            if row and row["payload"].get("baseline_id") is not None:
                return {
                    "baseline_id": row["payload"]["baseline_id"],
                    "baseline_ppb": row["payload"].get("baseline_ppb", {}),
                    "baseline_campaign": row["payload"].get("baseline_campaign"),
                    "baseline_pps_n": row["payload"].get("baseline_pps_n"),
                }
    except Exception:
        logging.exception("⚠️ [clocks] failed to read baseline from config")
    return None


def cmd_set_baseline(args: Optional[dict]) -> Dict[str, Any]:
    """
    SET_BASELINE id=<campaign_row_id>

    Reads the campaign row by its database primary key, extracts per-clock
    PPB from its denormalized report, and persists baseline_id + baseline_ppb
    into the SYSTEM config row.  The dashboard comparison readout reads
    this on every refresh — no restart required.
    """
    if not args or "id" not in args:
        return {"success": False, "message": "SET_BASELINE requires 'id' argument"}

    try:
        baseline_id = int(args["id"])
    except (ValueError, TypeError):
        return {"success": False, "message": f"Invalid baseline id: {args['id']}"}

    # Fetch the campaign row by its database ID
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            "SELECT id, campaign, payload FROM campaigns WHERE id = %s",
            (baseline_id,),
        )
        row = cur.fetchone()

    if row is None:
        return {"success": False, "message": f"No campaign with id={baseline_id}"}

    payload = row["payload"]
    if isinstance(payload, str):
        payload = json.loads(payload)

    report = payload.get("report")
    if not report:
        return {"success": False, "message": f"Campaign {baseline_id} has no report"}

    # Extract per-clock PPB from the report
    baseline_ppb = {}
    for key in ("gnss", "dwt", "pi", "ocxo"):
        blk = report.get(key, {})
        ppb = blk.get("ppb")
        if ppb is not None:
            baseline_ppb[key] = round(float(ppb), 3)

    if not baseline_ppb:
        return {"success": False, "message": f"Campaign {baseline_id} report has no PPB data"}

    # Persist into SYSTEM config (merge into existing JSON blob)
    baseline_blob = {
        "baseline_id": baseline_id,
        "baseline_ppb": baseline_ppb,
        "baseline_campaign": row["campaign"],
        "baseline_pps_n": report.get("pps_count"),
    }

    try:
        with open_db() as conn:
            cur = conn.cursor()
            cur.execute(
                """
                UPDATE config
                SET payload = payload || %s::jsonb
                WHERE config_key = 'SYSTEM'
                """,
                (json.dumps(baseline_blob),),
            )
    except Exception:
        logging.exception("❌ [clocks] failed to persist baseline to config")
        return {"success": False, "message": "Failed to persist baseline to config"}

    logging.info(
        "✅ [clocks] baseline set: id=%d campaign='%s' ppb=%s",
        baseline_id, row["campaign"], baseline_ppb,
    )

    return {
        "success": True,
        "message": "OK",
        "payload": baseline_blob,
    }


def cmd_baseline_info(_: Optional[dict]) -> Dict[str, Any]:
    """
    BASELINE_INFO — return the currently configured baseline.

    Returns baseline_id, baseline_ppb, campaign name, pps_n, location,
    and started_at from the referenced campaign row.
    """
    info = _get_baseline_from_config()
    if info is None:
        return {
            "success": True,
            "message": "OK",
            "payload": {"baseline_set": False},
        }

    baseline_id = info["baseline_id"]

    result: Dict[str, Any] = {
        "baseline_set": True,
        "baseline_id": baseline_id,
        "baseline_ppb": info.get("baseline_ppb", {}),
        "baseline_campaign": info.get("baseline_campaign"),
        "baseline_pps_n": info.get("baseline_pps_n"),
    }

    # Enrich with live campaign row data (location, started_at)
    try:
        with open_db(row_dict=True) as conn:
            cur = conn.cursor()
            cur.execute(
                "SELECT id, campaign, payload FROM campaigns WHERE id = %s",
                (baseline_id,),
            )
            row = cur.fetchone()
        if row:
            cpayload = row["payload"]
            if isinstance(cpayload, str):
                cpayload = json.loads(cpayload)
            result["baseline_location"] = cpayload.get("location")
            result["baseline_started_at"] = cpayload.get("started_at")
    except Exception:
        pass  # best-effort enrichment

    return {"success": True, "message": "OK", "payload": result}


# ---------------------------------------------------------------------
# CLOCKS_INFO
# ---------------------------------------------------------------------


def cmd_clocks_info(_: Optional[dict]) -> Dict[str, Any]:
    payload = {
        "campaign_active": _campaign_active,
        "pi_tick_epoch_set": _pi_tick_epoch_set,
        "pi_tick_epoch": _pi_tick_epoch if _pi_tick_epoch_set else None,
        "last_pps_count_seen": _last_pps_count_seen,
        "last_pi_seq": _last_pi_seq,
        "last_pi_corrected": _last_pi_corrected,
        "sync_expected_pps": _sync_expected_pps,
        "pending_epoch_mode": _pending_epoch_mode,
        "pending_epoch_target_pps": _pending_epoch_target_pps,
        "pending_projected_pi_ticks_at_target": _pending_projected_pi_ticks_at_target,
        "diag": _diag,
    }
    return {"success": True, "message": "OK", "payload": payload}


COMMANDS = {
    "START": cmd_start,
    "STOP": cmd_stop,
    "REPORT": cmd_report,
    "CLEAR": cmd_clear,
    "SET_BASELINE": cmd_set_baseline,
    "BASELINE_INFO": cmd_baseline_info,
    "CLOCKS_INFO": cmd_clocks_info,
}

# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------


def run() -> None:
    setup_logging()

    logging.info(
        "🕐 [clocks] DRACONIAN pps_count mode. "
        "Four tracked clocks: GNSS (Teensy 10MHz), DWT (Teensy 600MHz), "
        "OCXO (Teensy GPT1 10MHz), Pi (CNTVCT_EL0 54MHz via PITIMER). "
        "Correlation: TIMEBASE_FRAGMENT.teensy_pps_count MUST equal PITIMER.REPORT.pps_count. "
        "PITIMER is single-capture, no rings, no keyed lookup. "
        "START/RECOVER issue SET_PPS_COUNT then START/RECOVER on Teensy, "
        "then block waiting for the sync fragment. "
        "Outlier rejection on all clock domains (>50%% deviation -> reject). "
        "Instrumentation via CLOCKS_INFO. "
        "Baseline comparison via SET_BASELINE/BASELINE_INFO."
    )

    # Start command + pubsub servers FIRST (non-blocking) so that
    # TIMEBASE_FRAGMENT messages can arrive during recovery.
    # The sync latch in on_timebase_fragment() fires before the
    # campaign-active gate, so recovery's _end_sync_wait() will
    # receive the fragment it needs.
    server_setup(
        subsystem="CLOCKS",
        commands=COMMANDS,
        subscriptions={"TIMEBASE_FRAGMENT": on_timebase_fragment},
        blocking=False,
    )

    # Now recover (or stop stray Teensy streaming)
    row = _get_active_campaign()
    if row is None:
        _request_teensy_stop_best_effort()
    else:
        _recover_campaign()

    # Block forever (process lifetime)
    logging.info("🏁 [clocks] entering main loop")
    while True:
        time.sleep(3600)


if __name__ == "__main__":
    run()