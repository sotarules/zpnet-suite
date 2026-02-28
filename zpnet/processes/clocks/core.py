"""
ZPNet CLOCKS Process — TIMEBASE Authority (Pi-side) — v3 Queue Architecture

Core contract (v2026-02-28+):

  CLOCKS is a pure traffic cop.  It owns NO clock state.  It correlates
  data from two authorities by pps_count identity and produces TIMEBASE.

  Architecture:
    • Teensy owns: GNSS ns, DWT ns/cycles, OCXO ns — in TIMEBASE_FRAGMENT
    • PITIMER owns: Pi ticks, Pi ns (epoch-relative) — in GET_CAPTURE
    • CLOCKS owns: correlation, Welford statistics, campaign lifecycle

  v3: CLOCKS now manages PITIMER lifecycle via START/STOP commands,
  symmetric with how it manages the Teensy.  PITIMER starts in STOPPED
  state (capture loop runs, buffering disabled) and transitions to
  RUNNING when CLOCKS sends START.  This eliminates spurious eviction
  warnings when no campaign is active.

  Fragment reception is decoupled from processing:
    • on_timebase_fragment() — PUBSUB handler.  Extracts pps_count, drops
      fragment into a thread-safe queue, handles sync latch.  Returns
      immediately.  NEVER blocks, NEVER does IPC or I/O.
    • _process_loop() — dedicated thread.  Pulls from queue, calls
      PITIMER.GET_CAPTURE(pps_count=N), joins Teensy + Pi, builds
      TIMEBASE, persists.  Can take as long as needed — queue buffers.

  This eliminates the race condition where processing latency causes
  the PUBSUB handler to miss the next fragment delivery.

  PITIMER correlation is by identity (pps_count), not by timing:
    • GET_CAPTURE(pps_count=N) fetches and consumes from PITIMER's buffer
    • No sleep, no polling, no "latest capture" ambiguity
    • If pps_count not in buffer: skip (gap), not hard fault

  Epoch management is delegated to PITIMER:
    • CLOCKS calls PITIMER.SET_EPOCH(pi_tick_epoch) at START/RECOVER
    • PITIMER computes pi_ns for every capture autonomously
    • CLOCKS reads pi_ns from GET_CAPTURE — never computes it

Responsibilities:
  * Receive TIMEBASE_FRAGMENT from Teensy (queue-buffered)
  * Fetch correlated Pi capture from PITIMER by pps_count
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
    the campaign report — never in the TIMEBASE row itself
  * Gaps in pps_count are canonical (recovery) and non-fatal
"""

from __future__ import annotations

import json
import logging
import math
import queue
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
SYNC_FRAGMENT_TIMEOUT_S = 2.0
SYNC_RECOVER_TIMEOUT_S = 5.0
SYNC_PITIMER_POLL_S = 0.005
SYNC_LOG_INTERVAL_S = 0.25

# Fragment queue: maxsize=0 means unbounded
FRAGMENT_QUEUE_MAXSIZE = 0

# PITIMER GET_CAPTURE retry: after the fragment arrives from Teensy,
# PITIMER may not have captured the PPS edge yet.  We retry briefly.
PITIMER_CAPTURE_RETRIES = 20
PITIMER_CAPTURE_RETRY_INTERVAL_S = 0.010  # 10ms per retry = 200ms max

# ---------------------------------------------------------------------
# Fragment queue (decouples reception from processing)
# ---------------------------------------------------------------------

_fragment_queue: queue.Queue[Dict[str, Any]] = queue.Queue(maxsize=FRAGMENT_QUEUE_MAXSIZE)

# ---------------------------------------------------------------------
# Diagnostics (monotonic counters + last anomaly snapshots)
# ---------------------------------------------------------------------

_diag: Dict[str, Any] = {
    # Fragment ingress (PUBSUB handler — fast path)
    "fragments_received": 0,
    "fragments_queued": 0,
    "fragments_missing_teensy_pps_count": 0,

    # Fragment processing (processor thread — slow path)
    "fragments_processed": 0,
    "fragments_ignored_no_campaign": 0,
    "queue_depth_max_seen": 0,
    "queue_depth_current": 0,

    # Draconian correlation
    "hard_fault_pps_mismatch": 0,
    "hard_fault_pitimer_counter_missing": 0,
    "hard_fault_no_active_campaign": 0,
    "hard_fault_sync_timeout": 0,
    "last_hard_fault": {},
    "hard_faults_total": 0,
    "auto_recovery_failures": 0,

    # PITIMER capture fetch
    "pitimer_get_capture_requests": 0,
    "pitimer_get_capture_hits": 0,
    "pitimer_get_capture_misses": 0,
    "pitimer_get_capture_retries_total": 0,
    "last_pitimer_get_capture": {},

    # PITIMER skips (graceful — TIMEBASE not issued)
    "pitimer_skips_total": 0,
    "last_pitimer_skip": {},

    # PITIMER lifecycle control plane
    "pitimer_start_requests": 0,
    "pitimer_start_failures": 0,
    "pitimer_start_ipc_failures": 0,
    "last_pitimer_start": {},
    "pitimer_stop_requests": 0,
    "pitimer_stop_failures": 0,
    "last_pitimer_stop": {},
    "pitimer_epoch_requests": 0,
    "pitimer_epoch_failures": 0,
    "last_pitimer_epoch": {},

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

_last_pps_count_seen: Optional[int] = None

# What pps_count did we most recently tell PITIMER/Teensy to use?
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

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------


_auto_recovery_in_progress: bool = False


def _hard_fault(reason: str, details: Dict[str, Any]) -> None:
    """Log a hard fault and trigger automatic recovery."""
    global _campaign_active, _auto_recovery_in_progress

    _diag["hard_faults_total"] = _diag.get("hard_faults_total", 0) + 1
    _diag["last_hard_fault"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "reason": reason,
        "details": details,
    }

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

    raise RuntimeError(f"HARD FAULT: {reason} (auto-recovery initiated)")


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
    """Patiently wait for GNSS time to be available. Instrumented."""
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
# PITIMER control plane
# ---------------------------------------------------------------------


def _start_pitimer(pps_count: int) -> None:
    """
    Command PITIMER to START (transition to RUNNING, arm pps_count,
    clear buffer).  Hard fault on failure.
    """
    _diag["pitimer_start_requests"] += 1
    try:
        resp = send_command(
            machine="PI",
            subsystem="PITIMER",
            command="START",
            args={"pps_count": int(pps_count)},
        )
        ok = bool(resp.get("success", False))
        _diag["last_pitimer_start"] = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "pps_count": int(pps_count),
            "success": ok,
            "response": resp.get("payload", {}) if isinstance(resp, dict) else {},
        }
        if not ok:
            _diag["pitimer_start_failures"] += 1
            _hard_fault("pitimer_start_failed", {"pps_count": int(pps_count), "resp": resp})
    except RuntimeError:
        raise
    except Exception:
        _diag["pitimer_start_ipc_failures"] += 1
        _hard_fault("pitimer_start_ipc_failed", {"pps_count": int(pps_count)})


def _stop_pitimer_best_effort() -> None:
    """Command PITIMER to STOP (transition to STOPPED).  Best-effort."""
    _diag["pitimer_stop_requests"] += 1
    try:
        resp = send_command(
            machine="PI",
            subsystem="PITIMER",
            command="STOP",
        )
        ok = bool(resp.get("success", False))
        _diag["last_pitimer_stop"] = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "success": ok,
        }
        if not ok:
            _diag["pitimer_stop_failures"] += 1
            logging.warning("⚠️ [clocks] PITIMER STOP failed: %s", resp.get("message", "?"))
    except Exception:
        logging.warning("⚠️ [clocks] PITIMER STOP IPC failed (ignored)")


def _set_pitimer_epoch(pi_tick_epoch: int) -> None:
    """Command PITIMER to set the Pi tick epoch.  Hard fault on failure."""
    _diag["pitimer_epoch_requests"] += 1
    try:
        resp = send_command(
            machine="PI",
            subsystem="PITIMER",
            command="SET_EPOCH",
            args={"pi_tick_epoch": int(pi_tick_epoch)},
        )
        ok = bool(resp.get("success", False))
        _diag["last_pitimer_epoch"] = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "pi_tick_epoch": int(pi_tick_epoch),
            "success": ok,
        }
        if not ok:
            _diag["pitimer_epoch_failures"] += 1
            _hard_fault("pitimer_epoch_failed", {"pi_tick_epoch": int(pi_tick_epoch), "resp": resp})
    except RuntimeError:
        raise
    except Exception:
        _diag["pitimer_epoch_failures"] += 1
        _hard_fault("pitimer_epoch_ipc_failed", {"pi_tick_epoch": int(pi_tick_epoch)})


def _pitimer_get_capture(pps_count: int) -> Optional[Dict[str, Any]]:
    """
    Fetch a specific capture from PITIMER by pps_count.

    Retries briefly in case PITIMER hasn't captured this edge yet
    (the Teensy fragment can arrive before PITIMER's capture loop
    completes).  Returns None if not found after retries.
    """
    _diag["pitimer_get_capture_requests"] += 1
    retries = 0

    for attempt in range(PITIMER_CAPTURE_RETRIES + 1):
        try:
            resp = send_command(
                machine="PI",
                subsystem="PITIMER",
                command="GET_CAPTURE",
                args={"pps_count": int(pps_count)},
            )
            if resp.get("success"):
                _diag["pitimer_get_capture_hits"] += 1
                _diag["pitimer_get_capture_retries_total"] += retries
                _diag["last_pitimer_get_capture"] = {
                    "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                    "pps_count": int(pps_count),
                    "hit": True,
                    "retries": retries,
                }
                return resp.get("payload", {}) if isinstance(resp, dict) else {}

            # NOT_FOUND — PITIMER hasn't captured this edge yet
            if resp.get("message") == "NOT_FOUND" and attempt < PITIMER_CAPTURE_RETRIES:
                retries += 1
                time.sleep(PITIMER_CAPTURE_RETRY_INTERVAL_S)
                continue

            # Definitive miss after retries
            break

        except Exception:
            # IPC failure — don't retry, just miss
            break

    _diag["pitimer_get_capture_misses"] += 1
    _diag["pitimer_get_capture_retries_total"] += retries
    _diag["last_pitimer_get_capture"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "pps_count": int(pps_count),
        "hit": False,
        "retries": retries,
    }
    return None


def _fetch_environment() -> Optional[Dict[str, Any]]:
    """
    Fetch environment snapshot from SYSTEM REPORT for TIMEBASE correlation.

    Returns a flat dict with temperature, altitude, pressure, and power
    readings, or None if the report is unavailable.  Best-effort — a miss
    here should never block TIMEBASE production.
    """
    try:
        resp = send_command(machine="PI", subsystem="SYSTEM", command="REPORT")
        if not resp.get("success"):
            return None
        p = resp.get("payload", {})
        if not isinstance(p, dict):
            return None

        env = p.get("environment", {})
        gnss = p.get("gnss", {})
        gnss_clock = gnss.get("clock", {})
        teensy = p.get("teensy", {})
        pi = p.get("pi", {})
        power = p.get("power", {})
        battery = p.get("battery", {})

        # Power state: extract Pi, Teensy, OCXO domains from i2c-2
        i2c2 = power.get("i2c-2", {})
        pi_power = i2c2.get("0x40", {})
        teensy_power = i2c2.get("0x45", {})
        ocxo_power = i2c2.get("0x44", {})

        return {
            # Temperature
            "ambient_temp_c": env.get("temperature_c"),
            "teensy_temp_c": teensy.get("cpu_temp_c"),
            "pi_temp_c": pi.get("cpu_temp_c"),
            "gnss_temp_c": gnss_clock.get("temperature_c"),

            # Altitude (all forms)
            "barometric_altitude_m": env.get("altitude_m"),
            "gnss_altitude_m": gnss.get("altitude_m"),
            "ellipsoid_height_m": gnss.get("ellipsoid_height_m"),
            "geoid_sep_m": gnss.get("geoid_sep_m"),

            # Barometric pressure
            "pressure_hpa": env.get("pressure_hpa"),
            "humidity_pct": env.get("humidity_pct"),

            # Power state
            "pi_power": {
                "volts": pi_power.get("volts"),
                "amps": pi_power.get("amps"),
                "watts": pi_power.get("watts"),
            },
            "teensy_power": {
                "volts": teensy_power.get("volts"),
                "amps": teensy_power.get("amps"),
                "watts": teensy_power.get("watts"),
            },
            "ocxo_power": {
                "volts": ocxo_power.get("volts"),
                "amps": ocxo_power.get("amps"),
                "watts": ocxo_power.get("watts"),
            },

            # Battery state
            "battery": {
                "remaining_pct": battery.get("remaining_pct"),
                "tte_minutes": battery.get("tte_minutes"),
                "wh_remaining": battery.get("wh_remaining_estimate"),
                "health_state": battery.get("health_state"),
            },
        }
    except Exception:
        logging.debug("⚠️ [clocks] _fetch_environment failed (ignored)")
        return None


def _pitimer_report() -> Dict[str, Any]:
    """Fetch latest PITIMER state (non-destructive, for sync/diagnostics)."""
    try:
        resp = send_command(machine="PI", subsystem="PITIMER", command="REPORT")
        if resp.get("success"):
            return resp.get("payload", {}) if isinstance(resp, dict) else {}
    except Exception:
        pass
    return {}


# ---------------------------------------------------------------------
# Draconian sync primitives
# ---------------------------------------------------------------------


def _begin_sync_wait(expected_pps: int) -> None:
    """Prepare to wait for a TIMEBASE_FRAGMENT with a specific pps_count."""
    global _sync_expected_pps, _sync_fragment

    with _sync_lock:
        _sync_expected_pps = int(expected_pps)
        _sync_fragment = None
        _sync_event.clear()


def _end_sync_wait(timeout_s: float = SYNC_FRAGMENT_TIMEOUT_S) -> Tuple[Dict[str, Any], float]:
    """Block until the sync fragment arrives or timeout.  Hard fault on timeout."""
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
# Fragment handler (PUBSUB — fast path, NEVER blocks)
# ---------------------------------------------------------------------


def on_timebase_fragment(payload: Payload) -> None:
    """
    PUBSUB handler for TIMEBASE_FRAGMENT.

    This runs on the PUBSUB delivery thread.  It MUST return immediately.
    All it does:
      1. Extract teensy_pps_count.
      2. Handle sync latch (for START/RECOVER waits).
      3. Drop fragment into the processing queue.
    No IPC, no I/O, no sleep, no database.
    """
    global _sync_fragment

    _diag["fragments_received"] += 1

    frag = payload

    # --- Extract pps_count ---
    teensy_pps_count_raw = frag.get("teensy_pps_count")
    if teensy_pps_count_raw is None:
        _diag["fragments_missing_teensy_pps_count"] += 1
        return

    try:
        pps_count = int(teensy_pps_count_raw)
    except Exception:
        _diag["fragments_missing_teensy_pps_count"] += 1
        return

    # --- Sync latch (for START/RECOVER waits) ---
    with _sync_lock:
        if _sync_expected_pps is not None:
            logging.info(
                "🔎 [clocks] @%s fragment arrived: teensy_pps_count=%d (waiting for %d) match=%s",
                system_time_z(), pps_count, _sync_expected_pps, str(pps_count == _sync_expected_pps),
            )
            if int(pps_count) == int(_sync_expected_pps):
                _sync_fragment = dict(frag)
                _sync_event.set()

    # --- Enqueue for processing ---
    _fragment_queue.put(dict(frag))
    _diag["fragments_queued"] += 1

    depth = _fragment_queue.qsize()
    _diag["queue_depth_current"] = depth
    if depth > _diag["queue_depth_max_seen"]:
        _diag["queue_depth_max_seen"] = depth


# ---------------------------------------------------------------------
# Processor thread (slow path — all the heavy lifting)
# ---------------------------------------------------------------------


def _process_loop() -> None:
    """
    Dedicated thread: pulls fragments from queue, correlates with PITIMER,
    builds TIMEBASE, persists.  Runs forever.
    """
    global _campaign_active, _armed_pps_count
    global _last_pi_seq, _last_pi_corrected

    logging.info("🚀 [clocks] processor thread started")

    while True:
        try:
            frag = _fragment_queue.get(timeout=5.0)
        except queue.Empty:
            continue

        _diag["fragments_processed"] += 1
        _diag["queue_depth_current"] = _fragment_queue.qsize()

        # --- Extract pps_count ---
        teensy_pps_count_raw = frag.get("teensy_pps_count")
        if teensy_pps_count_raw is None:
            continue

        try:
            pps_count = int(teensy_pps_count_raw)
        except Exception:
            continue

        _note_pps_count(pps_count)

        # --- No campaign => ignore ---
        if not _campaign_active:
            _diag["fragments_ignored_no_campaign"] += 1
            continue

        # --- Check Teensy pps_count against what we armed ---
        armed = _armed_pps_count
        sysclk = system_time_z()
        if armed is not None and int(pps_count) != armed:
            # If a sync wait is active, this is likely a transitional
            # fragment from before the RECOVER took effect.  Skip it
            # rather than hard-faulting — the sync wait will handle
            # alignment.
            with _sync_lock:
                sync_active = _sync_expected_pps is not None
            if sync_active:
                logging.info(
                    "ℹ️ [clocks] @%s pps_count mismatch during sync wait: "
                    "armed=%d teensy=%d — skipping (transitional)",
                    sysclk, armed, int(pps_count),
                )
                _armed_pps_count = int(pps_count) + 1
                _diag["armed_pps_count"] = _armed_pps_count
                continue

            _diag["hard_fault_pps_mismatch"] += 1
            delta = int(pps_count) - armed
            logging.error(
                "💥 [clocks] @%s TEENSY PPS COUNT MISMATCH! "
                "armed=%d teensy=%d (off by %+d).",
                sysclk, armed, int(pps_count), delta,
            )
            try:
                _hard_fault("pps_mismatch_teensy_vs_armed", {
                    "system_time": sysclk,
                    "armed_pps_count": armed,
                    "teensy_pps_count": int(pps_count),
                    "delta": delta,
                })
            except RuntimeError:
                continue

        # --- Fetch Pi capture by pps_count identity ---
        pit = _pitimer_get_capture(int(pps_count))

        if pit is None:
            _diag["pitimer_skips_total"] += 1
            _diag["last_pitimer_skip"] = {
                "ts_utc": sysclk,
                "reason": "capture_not_found",
                "teensy_pps_count": int(pps_count),
                "armed_pps_count": armed,
            }
            logging.warning(
                "⚠️ [clocks] @%s PITIMER capture not found for pps_count=%d — "
                "skipping TIMEBASE (gap)",
                sysclk, int(pps_count),
            )
            _armed_pps_count = int(pps_count) + 1
            _diag["armed_pps_count"] = _armed_pps_count
            continue

        # --- Extract Pi data from capture ---
        pi_counter_raw = pit.get("counter")
        pi_corrected = pit.get("corrected") or pi_counter_raw
        pi_ns = pit.get("pi_ns")

        if pi_corrected is None:
            _diag["hard_fault_pitimer_counter_missing"] += 1
            logging.error(
                "💥 [clocks] @%s PITIMER capture missing counter for pps_count=%d",
                sysclk, int(pps_count),
            )
            try:
                _hard_fault("pitimer_missing_counter", {
                    "pps_count": int(pps_count),
                    "pitimer": pit,
                })
            except RuntimeError:
                continue

        pi_corrected = int(pi_corrected)

        if pi_ns is None:
            # The capture was stored before SET_EPOCH fired.
            # This is expected for the first record after START (pps_count=0,
            # where pi_ns is 0 by definition) or the first record after
            # RECOVER (processor won the race against SET_EPOCH).
            # Compute it locally as a fallback.
            pit_epoch = pit.get("pi_tick_epoch")  # not present — capture was pre-epoch
            logging.info(
                "ℹ️ [clocks] @%s pi_ns=None for pps_count=%d — "
                "pre-epoch capture, will be None in TIMEBASE",
                sysclk, int(pps_count),
            )

        logging.debug(
            "✅ [clocks] @%s correlation OK: teensy=%d pitimer=%d pi_ns=%s",
            sysclk, int(pps_count), int(pit.get("pps_count", -1)),
            str(pi_ns),
        )

        # Advance armed expectation to next second
        _armed_pps_count = int(pps_count) + 1
        _diag["armed_pps_count"] = _armed_pps_count

        # --- Pi capture diagnostics (observational) ---
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

        # --- Residual updates ---
        system_time_utc = datetime.now(timezone.utc)
        system_time_str = system_time_utc.isoformat(timespec="microseconds")

        gnss_time_utc_label = pit.get("gnss_time") or system_time_z()

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

        # --- Environment snapshot (best-effort, never blocks TIMEBASE) ---
        env_snapshot = _fetch_environment()

        # --- Campaign lookup ---
        row = _get_active_campaign()
        if row is None:
            _diag["hard_fault_no_active_campaign"] += 1
            try:
                _hard_fault("no_active_campaign", {"pps_count": int(pps_count)})
            except RuntimeError:
                continue

        campaign = row["campaign"]
        campaign_payload = row["payload"]

        # --- Build TIMEBASE record ---
        timebase = {
            "campaign": campaign,

            "system_time_utc": system_time_str,
            "gnss_time_utc": gnss_time_utc_label,

            "pps_count": int(pps_count),

            # Teensy authoritative clock state
            "teensy_dwt_cycles": frag["dwt_cycles"],
            "teensy_dwt_ns": frag["dwt_ns"],
            "teensy_gnss_ns": frag["gnss_ns"],
            "teensy_ocxo_ns": frag.get("ocxo_ns"),
            "teensy_rtc1_ns": frag.get("rtc1_ns"),
            "teensy_rtc2_ns": frag.get("rtc2_ns"),

            "teensy_pps_count": int(pps_count),

            # Pi authoritative clock state (from PITIMER)
            "pi_counter": pi_counter_raw,
            "pi_corrected": pi_corrected,
            "pi_ns": int(pi_ns) if pi_ns is not None else None,

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

            # Environment snapshot (correlated with this PPS edge)
            "environment": env_snapshot,
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
    START:
      1. Create campaign in DB (deactivate any prior).
      2. Stop Teensy + PITIMER (best-effort).
      3. Reset trackers.
      4. Begin sync wait for fragment pps_count=0.
      5. Arm TEENSY START(pps_count=0) — slow path.
      6. Start PITIMER(pps_count=0) — fast path.
      7. Wait for sync fragment pps_count=0.
      8. On sync fragment: fetch Pi capture, compute epoch, call SET_EPOCH.
      9. Activate campaign — processor thread begins TIMEBASE production.
    """
    global _campaign_active, _armed_pps_count

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
    _stop_pitimer_best_effort()
    _reset_trackers()

    # Prepare sync wait FIRST
    _begin_sync_wait(expected_pps=0)

    # Arm TEENSY first — slow path (serial round-trip)
    logging.info("📡 [start] @%s arming TEENSY START: pps_count=0", system_time_z())
    teensy_args: Dict[str, Any] = {"campaign": campaign}
    if set_dac is not None:
        teensy_args["set_dac"] = str(int(set_dac))
    if calibrate_ocxo:
        teensy_args["calibrate_ocxo"] = "true"

    _request_teensy_start(campaign=campaign, pps_count=0, args=teensy_args)
    logging.info("📡 [start] @%s TEENSY START returned — starting PITIMER", system_time_z())

    # Start PITIMER — fast path (transitions to RUNNING, arms pps_count=0)
    _start_pitimer(0)

    _armed_pps_count = 0
    _diag["armed_pps_count"] = 0
    logging.info("📡 [start] @%s both armed for pps_count=0 — waiting for sync fragment...", system_time_z())

    # Activate campaign BEFORE waiting so the processor thread will
    # accept pps_count=0 when it pulls it from the queue.  Without this,
    # the processor may drain and discard pps_count=0 as "no campaign"
    # before cmd_start returns and sets _campaign_active.
    _campaign_active = True

    # Wait for the sync fragment
    try:
        frag0, waited_s = _end_sync_wait()
    except Exception:
        _campaign_active = False
        return {"success": False, "message": "HARD FAULT during START sync (see logs/diag)"}

    logging.info("✅ [start] @%s sync fragment received (waited=%.3fs)", system_time_z(), waited_s)

    # --- Compute epoch from PITIMER REPORT (non-destructive peek) ---
    # DO NOT use GET_CAPTURE here — that would consume the capture,
    # leaving the processor thread unable to build TIMEBASE for pps_count=0.
    pit_report = _pitimer_report()
    pi_corrected_0 = pit_report.get("corrected")
    if pi_corrected_0 is None:
        return {"success": False, "message": "START failed: PITIMER REPORT has no corrected value"}

    pi_tick_epoch = int(pi_corrected_0)
    logging.info("📐 [start] @%s epoch: pi_tick_epoch=%d (corrected at pps_count=0)", system_time_z(), pi_tick_epoch)

    # Tell PITIMER the epoch so it can compute pi_ns from here on
    _set_pitimer_epoch(pi_tick_epoch)

    logging.info("▶️ [clocks] @%s START complete — campaign '%s' active", system_time_z(), campaign)

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "campaign": campaign,
            "location": location,
            "set_dac": set_dac,
            "calibrate_ocxo": calibrate_ocxo,
            "pps_count": 0,
            "pi_tick_epoch": pi_tick_epoch,
            "waited_s": round(float(waited_s), 3),
        },
    }


def cmd_stop(_: Optional[dict]) -> dict:
    global _campaign_active

    row = _get_active_campaign()
    had_location = row["payload"].get("location") if row else None

    _request_teensy_stop_best_effort()
    _stop_pitimer_best_effort()

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
    global _campaign_active

    _request_teensy_stop_best_effort()
    _stop_pitimer_best_effort()

    try:
        with open_db() as conn:
            cur = conn.cursor()
            cur.execute("DELETE FROM timebase")
            tb_count = cur.rowcount
            cur.execute("DELETE FROM campaigns")
            camp_count = cur.rowcount

        _campaign_active = False
        _reset_trackers()

        logging.info("🗑️ [clocks] CLEAR: deleted %d timebase rows, %d campaigns", tb_count, camp_count)
        return {"success": True, "message": "OK", "payload": {"timebase_deleted": tb_count, "campaigns_deleted": camp_count}}

    except Exception as e:
        logging.exception("❌ [clocks] CLEAR failed")
        return {"success": False, "message": str(e)}


# ---------------------------------------------------------------------
# Recovery
# ---------------------------------------------------------------------


def _recover_campaign() -> None:
    """
    RECOVER:
      1. Read last TIMEBASE row, compute next_pps_count.
      2. Wait for GNSS time availability.
      3. Restore GNSS TO mode if campaign had a location.
      4. Wait for routing (PUBSUB route check).
      5. Stop Teensy + PITIMER, quiesce, snap to second boundary.
      6. Arm TEENSY RECOVER + start PITIMER(next_pps_count).
      7. Wait for sync fragment.
      8. Compute epoch from sync edge, call PITIMER SET_EPOCH.
      9. Activate campaign.
    """
    global _campaign_active, _armed_pps_count

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
        logging.warning("⚠️ [recovery] teensy_gnss_ns=0 — using dwt_ns as proxy")
        last_gnss_ns = last_dwt_ns

    logging.info(
        "🔍 [recovery] last TIMEBASE: pps_count=%d gnss_ns=%d dwt_ns=%d ocxo_ns=%d pi_ns=%d",
        last_pps_count, last_gnss_ns, last_dwt_ns, last_ocxo_ns, last_pi_ns,
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
    logging.info("⏳ [recovery] waiting for GNSS time...")
    gnss_time_preflight = _wait_for_gnss_time()
    logging.info("✅ [recovery] GNSS time available: %s", gnss_time_preflight)

    # Restore GNSS TO mode if campaign had location
    if campaign_location:
        logging.info("📡 [recovery] restoring GNSS -> TO mode for '%s'", campaign_location)
        gnss_resp = _set_gnss_mode_to(campaign_location)
        if not gnss_resp.get("success"):
            raise RuntimeError(f"recovery failed: GNSS MODE=TO failed: {gnss_resp.get('message', '?')}")

    # Wait for routing to be live (PUBSUB route check, not fragment arrival).
    # After hard power-off, the Teensy boots into STOPPED and won't produce
    # TIMEBASE_FRAGMENTs until we send RECOVER.  So we can't wait for a
    # fragment — we verify that the PUBSUB route exists instead.
    logging.info("⏳ [recovery] @%s waiting for PUBSUB routing...", system_time_z())
    route_wait_t0 = time.monotonic()
    ROUTING_WAIT_TIMEOUT_S = 30.0
    ROUTING_WAIT_POLL_S = 0.5
    while True:
        elapsed_wait = time.monotonic() - route_wait_t0
        try:
            resp = send_command(machine="TEENSY", subsystem="PUBSUB", command="REPORT")
            if resp.get("success"):
                routes = resp.get("payload", {}).get("routes", [])
                found = any(
                    r.get("topic") == "TIMEBASE_FRAGMENT"
                    and r.get("subsystem") == "CLOCKS"
                    and r.get("machine") == "PI"
                    for r in routes
                )
                if found:
                    logging.info(
                        "✅ [recovery] @%s PUBSUB route confirmed (%.1fs)",
                        system_time_z(), elapsed_wait,
                    )
                    break
        except RuntimeError:
            raise
        except Exception:
            pass  # IPC not yet available — keep polling

        if elapsed_wait >= ROUTING_WAIT_TIMEOUT_S:
            raise RuntimeError(
                f"recovery failed: PUBSUB route not found in {ROUTING_WAIT_TIMEOUT_S}s"
            )
        time.sleep(ROUTING_WAIT_POLL_S)

    # Stop Teensy + PITIMER + quiesce
    logging.info("📡 [recovery] @%s stopping Teensy + PITIMER...", system_time_z())
    _request_teensy_stop_best_effort()
    _stop_pitimer_best_effort()
    time.sleep(2.0)

    # Snap to second boundary
    now_frac = time.time() % 1.0
    sleep_to_boundary = (1.0 - now_frac) + 0.050
    logging.info("⏳ [recovery] snapping to second boundary (sleeping %.3fs)", sleep_to_boundary)
    time.sleep(sleep_to_boundary)

    # Compute pps_count projection
    now_str = system_time_z()
    now_utc = datetime.fromisoformat(now_str.replace("Z", "+00:00"))
    elapsed_td = now_utc - last_system_utc
    elapsed_seconds = int(round(elapsed_td.total_seconds()))
    if elapsed_seconds <= 0:
        _diag["recovery_elapsed_seconds_nonpositive"] += 1
        raise RuntimeError(f"recovery failed: elapsed_seconds={elapsed_seconds}")

    project_seconds = elapsed_seconds + 2
    next_pps_count = last_pps_count + project_seconds

    _diag["last_recovery"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "campaign": campaign_name,
        "last_pps_count": int(last_pps_count),
        "elapsed_seconds": int(elapsed_seconds),
        "next_pps_count": int(next_pps_count),
    }

    logging.info(
        "📐 [recovery] pps_count: last=%d elapsed=%d next=%d",
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
        raise RuntimeError("recovery failed: projected_pi_ns <= 0")

    logging.info(
        "📐 [recovery] projected@target -- gnss_ns=%d dwt_ns=%d ocxo_ns=%d pi_ns=%d pi_ticks=%d",
        projected_gnss_ns, projected_dwt_ns, projected_ocxo_ns, projected_pi_ns, projected_pi_ticks_at_target,
    )

    _reset_trackers()

    # Begin sync wait FIRST
    _begin_sync_wait(expected_pps=int(next_pps_count))

    # Arm TEENSY first (slow path)
    logging.info("📡 [recovery] @%s arming TEENSY RECOVER: pps_count=%d", system_time_z(), next_pps_count)
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

    # Start PITIMER (transitions to RUNNING, arms pps_count)
    logging.info("📡 [recovery] @%s starting PITIMER: pps_count=%d", system_time_z(), next_pps_count)
    _start_pitimer(int(next_pps_count))

    _armed_pps_count = int(next_pps_count)
    _diag["armed_pps_count"] = int(next_pps_count)

    # Activate campaign BEFORE waiting so the processor thread will
    # accept the sync fragment when it pulls it from the queue.
    _campaign_active = True

    # Wait for sync fragment
    try:
        frag, waited_s = _end_sync_wait(timeout_s=SYNC_RECOVER_TIMEOUT_S)
    except Exception:
        _campaign_active = False
        raise

    logging.info("✅ [recovery] @%s sync fragment received (waited=%.3fs)", system_time_z(), waited_s)

    # --- Compute epoch from PITIMER REPORT (non-destructive peek) ---
    # DO NOT use GET_CAPTURE here — that would consume the capture,
    # leaving the processor thread unable to build TIMEBASE for this pps_count.
    pit_report = _pitimer_report()
    pi_corrected_at_sync = pit_report.get("corrected")
    if pi_corrected_at_sync is None:
        raise RuntimeError(f"recovery failed: PITIMER REPORT has no corrected value at pps_count={next_pps_count}")

    pi_corrected_at_sync = int(pi_corrected_at_sync)

    # RECOVER epoch: corrected_at_sync - projected_pi_ticks_at_target
    pi_tick_epoch = pi_corrected_at_sync - projected_pi_ticks_at_target

    logging.info(
        "📐 [recovery] epoch: corrected=%d - projected_ticks=%d = epoch=%d",
        pi_corrected_at_sync, projected_pi_ticks_at_target, pi_tick_epoch,
    )

    _set_pitimer_epoch(pi_tick_epoch)

    logging.info(
        "✅ [recovery] @%s campaign '%s' recovered — pps_count=%d, TIMEBASE resumes",
        system_time_z(), campaign_name, int(next_pps_count),
    )


# ---------------------------------------------------------------------
# BASELINE — persist and retrieve baseline campaign for comparison
# ---------------------------------------------------------------------


def _get_baseline_from_config() -> Optional[Dict[str, Any]]:
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
    if not args:
        return {"success": False, "message": "SET_BASELINE requires 'id' or 'campaign' argument"}

    baseline_id = args.get("id")
    campaign_name = args.get("campaign")

    if baseline_id is None and campaign_name is None:
        return {"success": False, "message": "SET_BASELINE requires 'id' or 'campaign' argument"}

    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        if baseline_id is not None:
            try:
                baseline_id = int(baseline_id)
            except (ValueError, TypeError):
                return {"success": False, "message": f"Invalid baseline id: {args['id']}"}
            cur.execute(
                "SELECT id, campaign, payload FROM campaigns WHERE id = %s",
                (baseline_id,),
            )
        else:
            cur.execute(
                "SELECT id, campaign, payload FROM campaigns WHERE campaign = %s ORDER BY ts DESC LIMIT 1",
                (campaign_name,),
            )
        row = cur.fetchone()

    if row is None:
        lookup = f"id={baseline_id}" if baseline_id is not None else f"campaign='{campaign_name}'"
        return {"success": False, "message": f"No campaign with {lookup}"}

    baseline_id = row["id"]

    payload = row["payload"]
    if isinstance(payload, str):
        payload = json.loads(payload)

    report = payload.get("report")
    if not report:
        return {"success": False, "message": f"Campaign {baseline_id} ('{row['campaign']}') has no report"}

    baseline_ppb = {}
    for key in ("gnss", "dwt", "pi", "ocxo"):
        blk = report.get(key, {})
        ppb = blk.get("ppb")
        if ppb is not None:
            baseline_ppb[key] = round(float(ppb), 3)

    if not baseline_ppb:
        return {"success": False, "message": f"Campaign {baseline_id} ('{row['campaign']}') report has no PPB data"}

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

    logging.info("✅ [clocks] baseline set: id=%d campaign='%s' ppb=%s", baseline_id, row["campaign"], baseline_ppb)
    return {"success": True, "message": "OK", "payload": baseline_blob}


def cmd_baseline_info(_: Optional[dict]) -> Dict[str, Any]:
    info = _get_baseline_from_config()
    if info is None:
        return {"success": True, "message": "OK", "payload": {"baseline_set": False}}

    baseline_id = info["baseline_id"]
    result: Dict[str, Any] = {
        "baseline_set": True,
        "baseline_id": baseline_id,
        "baseline_ppb": info.get("baseline_ppb", {}),
        "baseline_campaign": info.get("baseline_campaign"),
        "baseline_pps_n": info.get("baseline_pps_n"),
    }

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
        pass

    return {"success": True, "message": "OK", "payload": result}


def cmd_delete(args: Optional[dict]) -> Dict[str, Any]:
    """
    DELETE(campaign)

    Delete a campaign and all its TIMEBASE records by name.
    Refuses to delete the currently active campaign — stop it first.
    """
    if not args or "campaign" not in args:
        return {"success": False, "message": "DELETE requires 'campaign' argument"}

    campaign_name = args["campaign"]

    # Refuse to delete the active campaign
    row = _get_active_campaign()
    if row is not None and row["campaign"] == campaign_name:
        return {"success": False, "message": f"Campaign '{campaign_name}' is active — STOP it first"}

    try:
        with open_db() as conn:
            cur = conn.cursor()
            cur.execute(
                "DELETE FROM timebase WHERE campaign = %s",
                (campaign_name,),
            )
            tb_count = cur.rowcount
            cur.execute(
                "DELETE FROM campaigns WHERE campaign = %s",
                (campaign_name,),
            )
            camp_count = cur.rowcount
    except Exception as e:
        logging.exception("❌ [clocks] DELETE failed for campaign '%s'", campaign_name)
        return {"success": False, "message": str(e)}

    if camp_count == 0:
        return {"success": False, "message": f"No campaign named '{campaign_name}'"}

    logging.info(
        "🗑️ [clocks] DELETE: campaign='%s' — %d campaign row(s), %d timebase row(s) deleted",
        campaign_name, camp_count, tb_count,
    )
    return {
        "success": True, "message": "OK",
        "payload": {
            "campaign": campaign_name,
            "campaigns_deleted": camp_count,
            "timebase_deleted": tb_count,
        },
    }

# ---------------------------------------------------------------------
# CLOCKS_INFO
# ---------------------------------------------------------------------


def cmd_clocks_info(_: Optional[dict]) -> Dict[str, Any]:
    payload = {
        "campaign_active": _campaign_active,
        "last_pps_count_seen": _last_pps_count_seen,
        "last_pi_seq": _last_pi_seq,
        "last_pi_corrected": _last_pi_corrected,
        "sync_expected_pps": _sync_expected_pps,
        "diag": _diag,
    }
    return {"success": True, "message": "OK", "payload": payload}

def cmd_set_dac(args: Optional[dict]) -> Dict[str, Any]:
    """
    SET_DAC(dac)

    Update the OCXO DAC value in the SYSTEM config record.
    This becomes the default DAC for subsequent campaign starts.
    """
    if not args or "dac" not in args:
        return {"success": False, "message": "SET_DAC requires 'dac' argument"}

    try:
        dac = int(args["dac"])
    except (ValueError, TypeError):
        return {"success": False, "message": f"Invalid dac value: {args['dac']}"}

    try:
        with open_db() as conn:
            cur = conn.cursor()
            cur.execute(
                """
                UPDATE config
                SET payload = payload || %s::jsonb
                WHERE config_key = 'SYSTEM'
                """,
                (json.dumps({"ocxo_dac": dac}),),
            )
            if cur.rowcount == 0:
                return {"success": False, "message": "No SYSTEM config record found"}
    except Exception as e:
        logging.exception("❌ [clocks] SET_DAC failed")
        return {"success": False, "message": str(e)}

    logging.info("🔧 [clocks] SET_DAC: ocxo_dac=%d", dac)
    return {"success": True, "message": "OK", "payload": {"ocxo_dac": dac}}

COMMANDS = {
    "START": cmd_start,
    "STOP": cmd_stop,
    "REPORT": cmd_report,
    "CLEAR": cmd_clear,
    "DELETE": cmd_delete,
    "SET_DAC": cmd_set_dac,
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
        "🕐 [clocks] v3 Queue Architecture. "
        "PUBSUB handler enqueues fragments — processor thread correlates. "
        "PITIMER is Pi clock authority (owns epoch + pi_ns). "
        "CLOCKS manages PITIMER lifecycle via START/STOP (symmetric with Teensy). "
        "CLOCKS is pure traffic cop — no clock state. "
        "GET_CAPTURE by pps_count identity — no timing race. "
        "Commands: START, STOP, REPORT, CLEAR, SET_BASELINE, BASELINE_INFO, CLOCKS_INFO."
    )

    # Start processor thread
    threading.Thread(
        target=_process_loop,
        daemon=True,
        name="clocks-processor",
    ).start()

    # Start command + pubsub servers (non-blocking)
    server_setup(
        subsystem="CLOCKS",
        commands=COMMANDS,
        subscriptions={"TIMEBASE_FRAGMENT": on_timebase_fragment},
        blocking=False,
    )

    # Recover or stop stray Teensy + PITIMER
    row = _get_active_campaign()
    if row is None:
        _request_teensy_stop_best_effort()
        _stop_pitimer_best_effort()
    else:
        _recover_campaign()

    # Block forever
    logging.info("🏁 [clocks] entering main loop")
    while True:
        time.sleep(3600)


if __name__ == "__main__":
    run()