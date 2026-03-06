"""
ZPNet CLOCKS Process — TIMEBASE Authority (Pi-side) — v4 Nanosecond Recovery

Core contract (v2026-03-02+):

  CLOCKS is a pure traffic cop.  It owns NO clock state.  It correlates
  data from two authorities by pps_count identity and produces TIMEBASE.

  Architecture:
    • Teensy owns: GNSS ns, DWT ns/cycles, OCXO ns — in TIMEBASE_FRAGMENT
    • PITIMER owns: Pi ticks, Pi ns (epoch-relative) — in GET_CAPTURE
    • GNSS owns: GF-8802 discipline state — in GET_GNSS_INFO
    • CLOCKS owns: correlation, Welford statistics, campaign lifecycle

  v4 Nanosecond Recovery:

    CLOCKS speaks ONLY nanoseconds to both the Teensy and PITIMER.
    Each instrument privately derives its own internal representation
    (DWT cycle counts, Pi tick epoch) from the nanosecond value it
    receives.  CLOCKS never computes, sends, or persists cycle counts
    or epochs.

    Recovery is symmetric across all clock domains.  The same formula
    projects every domain forward:

      projected_gnss_ns = next_pps_count * NS_PER_SECOND
      projected_ns = projected_gnss_ns * last_clock_ns // last_gnss_ns

    The projected values are ABSOLUTE from campaign start.  The Teensy
    derives campaign_seconds (= pps_count) from gnss_ns / 1e9.

    This eliminates:
      • The +2 padding hack (GNSS time gives exact second)
      • DWT cycle count computation in CLOCKS
      • Pi epoch recovery from TIMEBASE history
      • Asymmetry between Teensy and Pi recovery paths
      • The SET_EPOCH race condition

    START is also simplified: PITIMER derives its own epoch from the
    first capture when given pi_ns=0 at campaign start.  No PEEK,
    no SET_EPOCH, no epoch computation in CLOCKS.

  Flash-cut campaign switching:

    START while a campaign is active performs a seamless "flash cut"
    to the new campaign.  The PPS stream continues uninterrupted.
    OCXOs are never disturbed.  Nanosecond counters and pps_count
    reset to zero under the new campaign name.  The previous campaign
    is closed in the DB with a stopped_at timestamp.

    This supports continuous operation across the Lantern experiment:
    overnight baseline → en route → Badwater → en route → Little Lakes
    without ever shutting down the system.

  v3 queue architecture (retained):

    Fragment reception is decoupled from processing:
      • on_timebase_fragment() — PUBSUB handler.  Fast path.
      • _process_loop() — dedicated thread.  Slow path.

    PITIMER correlation is by identity (pps_count), not by timing.

Responsibilities:
  * Receive TIMEBASE_FRAGMENT from Teensy (queue-buffered)
  * Fetch correlated Pi capture from PITIMER by pps_count
  * Fetch GF-8802 discipline snapshot from GNSS (GET_GNSS_INFO)
  * Augment fragment with Pi capture, GNSS time, GNSS discipline,
    environment, and system time
  * Publish TIMEBASE
  * Persist TIMEBASE to PostgreSQL (append-only, best-effort)
  * Compute per-clock residual statistics (Welford's algorithm)
  * Denormalize augmented report into active campaign payload
  * Recover clocks after restart if campaign is active
  * Flash-cut to new campaign while running (seamless switch)
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
import subprocess
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

# v4: CLOCKS no longer computes DWT cycle counts.
# DWT_CYCLES_PER_NS_NUM/DEN removed — the Teensy derives its own
# cycle counts from nanoseconds using dwt_ns_to_cycles().

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

# PITIMER stale detection: if the gap between the requested pps_count
# and the last successful hit exceeds this threshold, the capture has
# been evicted from PITIMER's buffer (max 120 entries).  Skip
# immediately instead of burning 200ms of retries on a hopeless case.
# This prevents the death spiral where CLOCKS falls further behind
# with every retry cycle.
PITIMER_STALE_THRESHOLD = 5

# PITIMER consecutive miss auto-recovery: if PITIMER misses this many
# consecutive captures, CLOCKS triggers a hard fault and auto-recovery.
# Cooldown prevents recovery loops.
PITIMER_MISS_RECOVERY_THRESHOLD = 3
PITIMER_MISS_RECOVERY_COOLDOWN_S = 60.0

PREFLIGHT_POLL_INTERVAL_S = 30
PREFLIGHT_LOG_PREFIX = "🛡️ [preflight]"

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

    # PITIMER peek (diagnostic only in v4 — not used in normal flow)
    "pitimer_peek_requests": 0,
    "pitimer_peek_hits": 0,
    "pitimer_peek_misses": 0,
    "last_pitimer_peek": {},

    # PITIMER skips (graceful — TIMEBASE not issued)
    "pitimer_skips_total": 0,
    "pitimer_stale_skips": 0,
    "pitimer_last_hit_pps_count": None,
    "last_pitimer_skip": {},

    # PITIMER consecutive miss auto-recovery
    "pitimer_consecutive_misses": 0,
    "pitimer_miss_recoveries": 0,
    "last_pitimer_miss_recovery": {},

    # PITIMER lifecycle control plane
    "pitimer_start_requests": 0,
    "pitimer_start_failures": 0,
    "pitimer_start_ipc_failures": 0,
    "last_pitimer_start": {},
    "pitimer_stop_requests": 0,
    "pitimer_stop_failures": 0,
    "last_pitimer_stop": {},
    "pitimer_recover_requests": 0,
    "pitimer_recover_failures": 0,
    "pitimer_recover_ipc_failures": 0,
    "last_pitimer_recover": {},

    # v4: epoch helpers retained for diagnostic use only
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

    # GNSS discipline info fetch
    "gnss_info_requests": 0,
    "gnss_info_hits": 0,
    "gnss_info_misses": 0,

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
_last_pitimer_miss_recovery_ts: float = 0.0

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


def _get_system_config() -> Dict[str, Any]:
    """Return SYSTEM config payload, or {} if unavailable."""
    try:
        with open_db(row_dict=True) as conn:
            cur = conn.cursor()
            cur.execute("SELECT payload FROM config WHERE config_key = 'SYSTEM'")
            row = cur.fetchone()
    except Exception:
        logging.exception("⚠️ [clocks] failed to read SYSTEM config")
        return {}

    if row is None:
        return {}

    payload = row["payload"]
    if isinstance(payload, str):
        payload = json.loads(payload)

    return payload if isinstance(payload, dict) else {}


def _get_current_location() -> Optional[str]:
    """Return the authoritative system-level current location, if set."""
    cfg = _get_system_config()
    location = cfg.get("current_location")
    if isinstance(location, str):
        location = location.strip()
        return location or None
    return None


def _get_location_record(location: Optional[str]) -> Optional[Dict[str, Any]]:
    """Return a location row and decoded payload, or None if not found."""
    if not location:
        return None

    try:
        with open_db(row_dict=True) as conn:
            cur = conn.cursor()
            cur.execute(
                """
                SELECT location, payload
                FROM locations
                WHERE location = %s
                LIMIT 1
                """,
                (location,),
            )
            row = cur.fetchone()
    except Exception:
        logging.exception("⚠️ [clocks] failed to read location record for '%s'", location)
        return None

    if row is None:
        return None

    payload = row["payload"]
    if isinstance(payload, str):
        payload = json.loads(payload)

    return {
        "location": row["location"],
        "payload": payload if isinstance(payload, dict) else {},
    }


def _get_machine_clock_data(location: Optional[str]) -> Dict[str, Any]:
    """Return machine_clock_data block for the given location, or {}."""
    row = _get_location_record(location)
    if row is None:
        return {}

    payload = row["payload"]
    mcd = payload.get("machine_clock_data", {})
    return mcd if isinstance(mcd, dict) else {}


MACHINE_CLOCK_DEFAULTS: Dict[str, Dict[str, Any]] = {
    "PI": {
        "machine_id": "zpnet-pi-1",
        "clock_domain": "PI",
    },
    "TEENSY": {
        "machine_id": "zpnet-teensy-1",
        "clock_domain": "DWT",
    },
}


def _get_machine_clock_entry(location: Optional[str], machine: str) -> Dict[str, Any]:
    """Return one machine entry from a location's machine_clock_data block, or {}."""
    mcd = _get_machine_clock_data(location)
    machines = mcd.get("machines", {})
    if not isinstance(machines, dict):
        return {}
    entry = machines.get(machine, {})
    return entry if isinstance(entry, dict) else {}


def _classify_machine_clock_quality(
    *,
    stddev_ns: Optional[float],
    stderr_ns: Optional[float],
    pps_samples: Optional[int],
) -> str:
    """
    Classify machine clock data quality from cumulative PPS statistics.

    This is intentionally simple and conservative.  The value is a
    convenience label for humans, not a scientific truth claim.
    """
    n = int(pps_samples or 0)
    if n <= 0 or stddev_ns is None or stderr_ns is None:
        return "WEAK"

    sdev = float(stddev_ns)
    serr = float(stderr_ns)

    if n >= 10_000 and sdev <= 75.0 and serr <= 2.0:
        return "STRONG"
    if n >= 1_000 and sdev <= 150.0 and serr <= 5.0:
        return "GOOD"
    if n >= 100 and sdev <= 500.0 and serr <= 25.0:
        return "FAIR"
    return "WEAK"


def _build_machine_clock_entry(
    *,
    machine: str,
    report_block: Dict[str, Any],
    existing_entry: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """
    Build one machine_clock_data.machines entry from the cumulative
    report block for a machine.
    """
    existing = existing_entry if isinstance(existing_entry, dict) else {}
    defaults = MACHINE_CLOCK_DEFAULTS.get(machine, {})

    stddev_ns = report_block.get("pps_stddev")
    stderr_ns = report_block.get("pps_stderr")
    pps_samples = report_block.get("pps_n")

    entry: Dict[str, Any] = {
        "machine_id": existing.get("machine_id", defaults.get("machine_id", machine.lower())),
        "clock_domain": existing.get("clock_domain", defaults.get("clock_domain", machine)),
        "mean_ppb_vs_gnss": report_block.get("ppb"),
        "tau_mean": report_block.get("tau"),
        "stddev_ns": stddev_ns,
        "stderr_ns": stderr_ns,
        "pps_samples": pps_samples,
        "quality": _classify_machine_clock_quality(
            stddev_ns=stddev_ns,
            stderr_ns=stderr_ns,
            pps_samples=pps_samples,
        ),
    }

    return entry


def _persist_machine_clock_data(
    *,
    location: Optional[str],
    source_campaign: str,
    report: Dict[str, Any],
) -> None:
    """
    Persist location-specific machine clock data.

    This is intentionally best-effort.  Machine clock data is the best
    currently known persisted estimate for each service-hosting machine
    at a given location.  It is updated opportunistically from the live
    cumulative report as TIMEBASE records arrive.
    """
    if not location:
        return

    row = _get_location_record(location)
    if row is None:
        logging.warning(
            "⚠️ [clocks] machine_clock_data update skipped — unknown location '%s'",
            location,
        )
        return

    payload = row["payload"]
    existing_mcd = payload.get("machine_clock_data", {})
    if not isinstance(existing_mcd, dict):
        existing_mcd = {}

    existing_machines = existing_mcd.get("machines", {})
    if not isinstance(existing_machines, dict):
        existing_machines = {}

    pi_report = report.get("pi", {})
    dwt_report = report.get("dwt", {})

    block = {
        "updated_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "source_campaign": source_campaign,
        "machines": {
            "PI": _build_machine_clock_entry(
                machine="PI",
                report_block=pi_report if isinstance(pi_report, dict) else {},
                existing_entry=existing_machines.get("PI", {}),
            ),
            "TEENSY": _build_machine_clock_entry(
                machine="TEENSY",
                report_block=dwt_report if isinstance(dwt_report, dict) else {},
                existing_entry=existing_machines.get("TEENSY", {}),
            ),
        },
    }

    try:
        with open_db() as conn:
            cur = conn.cursor()
            cur.execute(
                """
                UPDATE locations
                SET payload = payload || jsonb_build_object('machine_clock_data', %s::jsonb)
                WHERE location = %s
                """,
                (json.dumps(block), location),
            )
    except Exception:
        logging.exception(
            "⚠️ [clocks] failed to persist machine_clock_data for location '%s' (ignored)",
            location,
        )


def _location_has_time_only_profile(location: Optional[str]) -> bool:
    """
    True if the location record contains the geodetic facts needed
    to command GNSS into Time Only mode.
    """
    row = _get_location_record(location)
    if row is None:
        return False

    payload = row["payload"]
    return (
        payload.get("latitude") is not None
        and payload.get("longitude") is not None
        and payload.get("altitude") is not None
    )


def _ensure_gnss_mode_for_current_location() -> Optional[str]:
    """
    Keep GNSS in TO mode whenever the authoritative current location
    has a usable profile; otherwise place GNSS in NORMAL mode.

    Returns the current system location (which may be None).
    Raises RuntimeError only if a GNSS MODE command is explicitly rejected.
    """
    location = _get_current_location()

    if location and _location_has_time_only_profile(location):
        logging.info("📡 [clocks] ensuring GNSS -> TO mode for current location '%s'", location)
        gnss_resp = _set_gnss_mode_to(location)
        if not gnss_resp.get("success"):
            raise RuntimeError(f"GNSS MODE=TO failed for '{location}': {gnss_resp.get('message', '?')}")
        return location

    logging.info("📡 [clocks] ensuring GNSS -> NORMAL mode (no TO-capable current location)")
    gnss_resp = _set_gnss_mode_normal()
    if not gnss_resp.get("success"):
        raise RuntimeError(f"GNSS MODE=NORMAL failed: {gnss_resp.get('message', '?')}")
    return location


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


def _get_gnss_time() -> str:
    """Get GNSS time (non-blocking, raises on failure)."""
    resp = send_command(machine="PI", subsystem="GNSS", command="GET_TIME")
    if not resp.get("success"):
        raise RuntimeError(f"GNSS GET_TIME failed: {resp.get('message', '?')}")
    return resp["payload"]["gnss_time"]


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


def _start_pitimer(pps_count: int, pi_ns: Optional[int] = None) -> None:
    """
    Command PITIMER to START (transition to RUNNING, arm pps_count,
    clear buffer).  Optionally pass pi_ns for epoch derivation.

    v4: When pi_ns is provided, PITIMER derives its own epoch from the
    first capture.  No separate SET_EPOCH call needed.

    Hard fault on failure.
    """
    _diag["pitimer_start_requests"] += 1

    start_args: Dict[str, Any] = {"pps_count": int(pps_count)}
    if pi_ns is not None:
        start_args["pi_ns"] = int(pi_ns)

    try:
        resp = send_command(
            machine="PI",
            subsystem="PITIMER",
            command="START",
            args=start_args,
        )
        ok = bool(resp.get("success", False))
        _diag["last_pitimer_start"] = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "pps_count": int(pps_count),
            "pi_ns": int(pi_ns) if pi_ns is not None else None,
            "success": ok,
            "response": resp.get("payload", {}) if isinstance(resp, dict) else {},
        }
        if not ok:
            _diag["pitimer_start_failures"] += 1
            _hard_fault("pitimer_start_failed", {
                "pps_count": int(pps_count),
                "pi_ns": int(pi_ns) if pi_ns is not None else None,
                "resp": resp,
            })
    except RuntimeError:
        raise
    except Exception:
        _diag["pitimer_start_ipc_failures"] += 1
        _hard_fault("pitimer_start_ipc_failed", {
            "pps_count": int(pps_count),
            "pi_ns": int(pi_ns) if pi_ns is not None else None,
        })


def _recover_pitimer(pps_count: int, pi_ns: int) -> None:
    """
    Command PITIMER to RECOVER (transition to RUNNING, arm pps_count,
    set pending pi_ns for epoch derivation from first capture).

    v4: Symmetric with the Teensy's RECOVER command.  CLOCKS sends
    the projected pi_ns; PITIMER derives its own epoch internally.

    Hard fault on failure.
    """
    _diag["pitimer_recover_requests"] += 1
    try:
        resp = send_command(
            machine="PI",
            subsystem="PITIMER",
            command="RECOVER",
            args={"pps_count": int(pps_count), "pi_ns": int(pi_ns)},
        )
        ok = bool(resp.get("success", False))
        _diag["last_pitimer_recover"] = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "pps_count": int(pps_count),
            "pi_ns": int(pi_ns),
            "success": ok,
            "response": resp.get("payload", {}) if isinstance(resp, dict) else {},
        }
        if not ok:
            _diag["pitimer_recover_failures"] += 1
            _hard_fault("pitimer_recover_failed", {
                "pps_count": int(pps_count),
                "pi_ns": int(pi_ns),
                "resp": resp,
            })
    except RuntimeError:
        raise
    except Exception:
        _diag["pitimer_recover_ipc_failures"] += 1
        _hard_fault("pitimer_recover_ipc_failed", {
            "pps_count": int(pps_count),
            "pi_ns": int(pi_ns),
        })


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
    """
    Command PITIMER to set the Pi tick epoch.  Hard fault on failure.

    v4: This function is RETAINED as a diagnostic/override primitive
    but is NO LONGER CALLED during normal START or RECOVER flows.
    PITIMER derives its own epoch from pi_ns via START(pi_ns=X)
    or RECOVER(pps_count, pi_ns).
    """
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
    Fetch a specific capture from PITIMER by pps_count (CONSUMING it).

    Retries briefly in case PITIMER hasn't captured this edge yet
    (the Teensy fragment can arrive before PITIMER's capture loop
    completes).  Returns None if not found after retries.

    Stale detection: if we've been missing captures consecutively,
    the requested pps_count has likely been evicted from PITIMER's
    buffer (max 120 entries).  In that case, retrying is hopeless
    and just deepens the death spiral.  We skip immediately.
    """
    _diag["pitimer_get_capture_requests"] += 1

    # Stale detection: if we've missed more than PITIMER_STALE_THRESHOLD
    # consecutive captures, the buffer has moved past us.  Skip
    # immediately to let CLOCKS catch up to the present.
    consecutive_misses = (
        _diag["pitimer_get_capture_requests"]
        - _diag["pitimer_get_capture_hits"]
        - 1  # exclude the current request
    ) - _diag.get("pitimer_stale_skips", 0)

    # More precise: check gap between requested pps_count and last hit
    last_hit_pps = _diag.get("pitimer_last_hit_pps_count")
    if last_hit_pps is not None and (int(pps_count) - int(last_hit_pps)) > PITIMER_STALE_THRESHOLD:
        _diag["pitimer_stale_skips"] = _diag.get("pitimer_stale_skips", 0) + 1
        _diag["pitimer_get_capture_misses"] += 1
        _diag["last_pitimer_get_capture"] = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "pps_count": int(pps_count),
            "hit": False,
            "retries": 0,
            "stale_skip": True,
            "gap": int(pps_count) - int(last_hit_pps),
        }
        return None

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
                _diag["pitimer_last_hit_pps_count"] = int(pps_count)
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


def _pitimer_get_capture_peek(pps_count: int) -> Optional[Dict[str, Any]]:
    """
    Non-destructive peek at a specific capture.

    v4: This function is RETAINED as a diagnostic utility but is
    NO LONGER CALLED during normal START or RECOVER flows.  PITIMER
    derives its own epoch from pi_ns — no PEEK needed for epoch
    computation.
    """
    _diag["pitimer_peek_requests"] += 1
    retries = 0

    for attempt in range(PITIMER_CAPTURE_RETRIES + 1):
        try:
            resp = send_command(
                machine="PI",
                subsystem="PITIMER",
                command="GET_CAPTURE",
                args={"pps_count": int(pps_count), "peek": True},
            )
            if resp.get("success"):
                _diag["pitimer_peek_hits"] += 1
                _diag["last_pitimer_peek"] = {
                    "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                    "pps_count": int(pps_count),
                    "hit": True,
                    "retries": retries,
                    "corrected": resp.get("payload", {}).get("corrected"),
                }
                return resp.get("payload", {}) if isinstance(resp, dict) else {}

            if resp.get("message") == "NOT_FOUND" and attempt < PITIMER_CAPTURE_RETRIES:
                retries += 1
                time.sleep(PITIMER_CAPTURE_RETRY_INTERVAL_S)
                continue

            break

        except Exception:
            break

    _diag["pitimer_peek_misses"] += 1
    _diag["last_pitimer_peek"] = {
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


def _fetch_gnss_info() -> Optional[Dict[str, Any]]:
    """
    Fetch GF-8802 discipline snapshot from GNSS GET_GNSS_INFO.

    Returns the flat payload dict (freq_mode, pps_timing_error_ns,
    freq_error_ppb, clock_drift_ppb, temperature_c, traim, etc.)
    or None if unavailable.

    Best-effort — a miss here should never block TIMEBASE production.
    The returned dict is embedded verbatim as the "gnss" block in
    each TIMEBASE record, providing per-second visibility into the
    GF-8802's discipline loop alongside the Teensy and Pi clock data.
    """
    _diag["gnss_info_requests"] += 1
    try:
        resp = send_command(
            machine="PI",
            subsystem="GNSS",
            command="GET_GNSS_INFO",
        )
        if resp.get("success"):
            _diag["gnss_info_hits"] += 1
            return resp.get("payload", {}) if isinstance(resp, dict) else {}
        _diag["gnss_info_misses"] += 1
        return None
    except Exception:
        _diag["gnss_info_misses"] += 1
        logging.debug("⚠️ [clocks] _fetch_gnss_info failed (ignored)")
        return None


def _pitimer_report() -> Dict[str, Any]:
    """Fetch latest PITIMER state (non-destructive, for diagnostics only)."""
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

    logging.info("⏳ [_end_sync_wait] waiting for TIMEBASE_FRAGMENT pps_count=%s...", str(_sync_expected_pps))

    t0 = time.monotonic()
    _diag["sync_waits"] += 1

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
        "location": campaign_payload.get("location") or _get_current_location(),
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
    # Accept pps_count >= expected, not exact match.  The projected
    # next_pps_count is approximate, so the Teensy may land one or
    # two counts ahead.  We latch on the FIRST fragment that meets
    # or exceeds the target.
    with _sync_lock:
        if _sync_expected_pps is not None:
            match = int(pps_count) >= int(_sync_expected_pps)
            logging.info(
                "🔎 [on_timebase_fragment] @%s fragment arrived: teensy_pps_count=%d (waiting for >=%d) match=%s",
                system_time_z(), pps_count, _sync_expected_pps, str(match),
            )
            if match:
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
    global _last_pi_seq, _last_pi_corrected, _last_pitimer_miss_recovery_ts

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
            _diag["pitimer_consecutive_misses"] = _diag.get("pitimer_consecutive_misses", 0) + 1
            consecutive = _diag["pitimer_consecutive_misses"]
            _diag["last_pitimer_skip"] = {
                "ts_utc": sysclk,
                "reason": "capture_not_found",
                "teensy_pps_count": int(pps_count),
                "armed_pps_count": armed,
                "consecutive": consecutive,
            }
            logging.warning(
                "⚠️ [clocks] @%s PITIMER capture not found for pps_count=%d — "
                "skipping TIMEBASE (gap, consecutive=%d)",
                sysclk, int(pps_count), consecutive,
            )
            _armed_pps_count = int(pps_count) + 1
            _diag["armed_pps_count"] = _armed_pps_count

            # Auto-recovery on persistent PITIMER desync
            if consecutive >= PITIMER_MISS_RECOVERY_THRESHOLD:
                now_mono = time.monotonic()
                elapsed = now_mono - _last_pitimer_miss_recovery_ts
                if elapsed >= PITIMER_MISS_RECOVERY_COOLDOWN_S:
                    _last_pitimer_miss_recovery_ts = now_mono
                    _diag["pitimer_miss_recoveries"] = _diag.get("pitimer_miss_recoveries", 0) + 1
                    _diag["last_pitimer_miss_recovery"] = {
                        "ts_utc": sysclk,
                        "consecutive_misses": consecutive,
                        "pps_count": int(pps_count),
                    }
                    try:
                        _hard_fault("pitimer_consecutive_miss", {
                            "system_time": sysclk,
                            "consecutive_misses": consecutive,
                            "pps_count": int(pps_count),
                            "threshold": PITIMER_MISS_RECOVERY_THRESHOLD,
                        })
                    except RuntimeError:
                        pass  # _hard_fault raises to break out of processing
                else:
                    logging.warning(
                        "⚠️ [clocks] @%s PITIMER miss recovery cooldown: %.0fs remaining",
                        sysclk, PITIMER_MISS_RECOVERY_COOLDOWN_S - elapsed,
                    )

            continue

        # Successful capture — reset consecutive miss counter
        _diag["pitimer_consecutive_misses"] = 0

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

        # --- GNSS discipline snapshot (best-effort, never blocks TIMEBASE) ---
        gnss_info = _fetch_gnss_info()

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
            #
            # The pre-PPS spin loop captures the DWT shadow register
            # continuously.  The PPS ISR snapshots the shadow, and
            # the ASAP callback computes dispatch latency and TDC
            # correction.  All fields below are from the Teensy's
            # TIMEBASE_FRAGMENT.
            #
            # Latency and TDC correction (per-edge):
            "diag_teensy_dispatch_delta_ns": frag.get("dispatch_delta_ns"),
            "diag_teensy_dispatch_shadow_ns": frag.get("dispatch_shadow_ns"),
            "diag_teensy_dispatch_isr_ns": frag.get("dispatch_isr_ns"),
            "diag_teensy_pps_edge_valid": frag.get("pps_edge_valid"),
            "diag_teensy_pps_edge_correction_ns": frag.get("pps_edge_correction_ns"),
            "diag_teensy_tdc_needs_recal": frag.get("tdc_needs_recal"),

            # Approach time: GNSS ns from spin loop entry to PPS edge.
            # Tells you how long the spin loop actually ran.
            "diag_teensy_approach_ns": frag.get("pre_pps_approach_ns"),

            # Spin loop status for THIS edge:
            "diag_teensy_dispatch_timeout": frag.get("dispatch_timeout"),
            "diag_teensy_fine_was_late": frag.get("diag_fine_was_late"),
            "diag_teensy_spin_iters": frag.get("diag_spin_iters"),

            # Monotonic counters (diff consecutive records for per-second rates):
            "diag_teensy_coarse_fires": frag.get("diag_coarse_fires"),
            "diag_teensy_fine_fires": frag.get("diag_fine_fires"),
            "diag_teensy_late_count": frag.get("diag_late"),
            "diag_teensy_spin_count": frag.get("diag_spins"),
            "diag_teensy_timeout_count": frag.get("diag_timeouts"),

            # Raw DWT cycle counts for TDC derivation
            "diag_raw_isr_cyc": frag.get("diag_raw_isr_cyc"),
            "diag_raw_shadow_cyc": frag.get("diag_raw_shadow_cyc"),

            # OCXO control state
            "ocxo_dac": frag.get("ocxo_dac"),
            "calibrate_ocxo": frag.get("calibrate_ocxo"),
            "servo_converged": frag.get("servo_converged"),

            # Environment snapshot (correlated with this PPS edge)
            "environment": env_snapshot,

            # GF-8802 discipline snapshot (correlated with this PPS edge)
            "gnss": gnss_info,

            # Running Welford statistics — per-clock-domain residual
            # accumulators snapshotted at this PPS edge.  These are
            # cumulative from campaign start (or last reset).
            #
            # Each domain carries:
            #   n        — sample count
            #   mean_ns  — mean residual (ns)
            #   stddev_ns — population stddev of residual (ns)
            #   stderr_ns — standard error of the mean (ns)
            #   tau      — cumulative clock_ns / gnss_ns ratio
            #   ppb      — cumulative parts-per-billion offset vs GNSS
            #
            # The Pi residual tracker uses display_scale=PI_NS_PER_TICK,
            # so its mean/stddev/stderr are already in nanoseconds.
            "stats": {
                "gnss": {
                    "n": _residual_gnss.n,
                    "mean_ns": round(_residual_gnss.mean * _residual_gnss.display_scale, 3),
                    "stddev_ns": round(_residual_gnss.stddev * _residual_gnss.display_scale, 3),
                    "stderr_ns": round(_residual_gnss.stderr * _residual_gnss.display_scale, 3),
                    "tau": round(_compute_tau(gnss_ns, gnss_ns), 12) if gnss_ns else None,
                    "ppb": round(_compute_ppb(gnss_ns, gnss_ns), 3) if gnss_ns else None,
                },
                "dwt": {
                    "n": _residual_dwt.n,
                    "mean_ns": round(_residual_dwt.mean * _residual_dwt.display_scale, 3),
                    "stddev_ns": round(_residual_dwt.stddev * _residual_dwt.display_scale, 3),
                    "stderr_ns": round(_residual_dwt.stderr * _residual_dwt.display_scale, 3),
                    "tau": round(_compute_tau(dwt_ns, gnss_ns), 12) if gnss_ns else None,
                    "ppb": round(_compute_ppb(dwt_ns, gnss_ns), 3) if gnss_ns else None,
                },
                "ocxo": {
                    "n": _residual_ocxo.n,
                    "mean_ns": round(_residual_ocxo.mean * _residual_ocxo.display_scale, 3),
                    "stddev_ns": round(_residual_ocxo.stddev * _residual_ocxo.display_scale, 3),
                    "stderr_ns": round(_residual_ocxo.stderr * _residual_ocxo.display_scale, 3),
                    "tau": round(_compute_tau(ocxo_ns, gnss_ns), 12) if gnss_ns else None,
                    "ppb": round(_compute_ppb(ocxo_ns, gnss_ns), 3) if gnss_ns else None,
                },
                "pi": {
                    "n": _residual_pi.n,
                    "mean_ns": round(_residual_pi.mean * _residual_pi.display_scale, 3),
                    "stddev_ns": round(_residual_pi.stddev * _residual_pi.display_scale, 3),
                    "stderr_ns": round(_residual_pi.stderr * _residual_pi.display_scale, 3),
                    "tau": round(_compute_tau(int(pi_ns), gnss_ns), 12) if (gnss_ns and pi_ns is not None) else None,
                    "ppb": round(_compute_ppb(int(pi_ns), gnss_ns), 3) if (gnss_ns and pi_ns is not None) else None,
                },
            },
        }

        report = _build_report(campaign, campaign_payload, timebase)

        publish("TIMEBASE", timebase)
        _persist_timebase(timebase, report)

        # Opportunistically refresh location-specific machine clock data
        # from the cumulative live report.  This is campaign-scoped:
        # only campaigns that are explicitly associated with a location
        # may update that location's machine clock data.  We do NOT
        # fall back to the current system location here, because the
        # persisted association must come from the campaign record
        # itself, not from ambient system state.
        campaign_location_for_machine_clock_data = campaign_payload.get("location")
        if isinstance(campaign_location_for_machine_clock_data, str):
            campaign_location_for_machine_clock_data = (
                campaign_location_for_machine_clock_data.strip() or None
            )
        else:
            campaign_location_for_machine_clock_data = None

        if campaign_location_for_machine_clock_data:
            _persist_machine_clock_data(
                location=campaign_location_for_machine_clock_data,
                source_campaign=campaign,
                report=report,
            )

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
                                "ocxo_dac": float(teensy_ocxo_dac),
                                "calibrate_ocxo": bool(teensy_calibrate),
                                "servo_converged": bool(teensy_converged),
                            }),
                            campaign,
                        ),
                    )
            except Exception:
                logging.exception("⚠️ [clocks] failed to persist OCXO DAC (ignored)")

        # Persist calibrated DAC to SYSTEM config so it survives
        # across campaigns.  Only during active calibration — we
        # don't overwrite a manually-set value during normal runs.
        if teensy_calibrate and teensy_ocxo_dac is not None:
            try:
                with open_db() as conn:
                    cur = conn.cursor()
                    cur.execute(
                        """
                        UPDATE config
                        SET payload = payload || %s::jsonb
                        WHERE config_key = 'SYSTEM'
                        """,
                        (json.dumps({"ocxo_dac": float(teensy_ocxo_dac)}),),
                    )
            except Exception:
                logging.exception("⚠️ [clocks] failed to persist OCXO DAC to config (ignored)")


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
    START — v4 nanosecond architecture with flash-cut support:

    If NO campaign is active (cold start):
      0. Wait for preflight prerequisites (GNSS lock, chrony PPS
         selected, PITIMER warm).  This ensures the system clock is
         aligned with true PPS edges before arming.
      1. Reject if campaign name already exists in DB.
      2. Create campaign in DB (deactivate any prior).
      3. Stop Teensy + PITIMER (best-effort).
      4. Reset trackers.
      5. Begin sync wait for fragment pps_count=0.
      6. Arm TEENSY START(pps_count=0) — slow path.
      7. Start PITIMER(pps_count=0, pi_ns=0) — fast path.
      8. Wait for sync fragment pps_count=0.
      9. Activate campaign.

    If a campaign IS already active (flash-cut):
      1. Reject if campaign name already exists in DB.
      2. Close the active campaign in the DB (set stopped_at, deactivate).
      3. Create the new campaign in the DB (active).
      4. Reset trackers (Welford's, pi_seq, etc.).
      5. Arm Teensy START (zeroes all accumulators, pps_count=0).
         The Teensy resets its counters at the next PPS edge — no stop needed.
         The PPS stream continues uninterrupted.  OCXOs undisturbed.
      6. Arm PITIMER START(pi_ns=0) (new epoch from next capture).
      7. Sync on pps_count=0, resume processing under the new campaign name.

    The flash-cut is seamless: no STOP, no power cycle, no GNSS re-acquisition.
    The only observable effect is that nanosecond counters and pps_count reset
    to zero.  The OCXOs, GNSS receiver, and PPS stream are never interrupted.

    The preflight gate is skipped on flash-cut because the system is already
    running — chrony is locked, PITIMER is warm, GNSS is disciplined.
    """
    global _campaign_active, _armed_pps_count

    campaign = args.get("campaign") if args else None
    if not campaign:
        return {"success": False, "message": "START requires 'campaign' argument"}

    # --- Check for active campaign (determines cold start vs flash-cut) ---
    active_row = _get_active_campaign()
    flash_cut = active_row is not None

    # ------------------------------------------------------------------
    # Cold start only: wait for preflight prerequisites.
    #
    # On flash-cut the system is already running — chrony is locked,
    # PITIMER is warm, GNSS is disciplined.  No gate needed.
    #
    # On cold start the system may have just booted.  The preflight
    # gate ensures GNSS lock, chrony PPS selection, and PITIMER warmth
    # before we proceed.  This prevents the pps_count desynchronization
    # that occurs when PITIMER's ppscapture_next() detects phantom
    # second boundaries from an undisciplined system clock.
    # ------------------------------------------------------------------
    current_location = _get_current_location()

    # Keep GNSS in the correct steady-state mode for the current site
    # before attempting preflight or campaign start.
    try:
        _ensure_gnss_mode_for_current_location()
    except Exception as e:
        return {"success": False, "message": str(e)}

    if not flash_cut:
        _wait_for_preflight("start")

    # --- Reject duplicate campaign name ---
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            "SELECT id, active FROM campaigns WHERE campaign = %s",
            (campaign,),
        )
        existing = cur.fetchone()

    if existing is not None:
        if existing["active"]:
            return {
                "success": False,
                "message": f"Campaign '{campaign}' is already active (id={existing['id']}) — STOP it or choose a new name",
            }
        return {
            "success": False,
            "message": f"Campaign '{campaign}' already exists (id={existing['id']}) — DELETE it first or choose a new name",
        }

    location = current_location
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
                    set_dac = float(row["payload"]["ocxo_dac"])
                    logging.info("🔧 [clocks] using ocxo_dac=%s from SYSTEM config", set_dac)
        except Exception:
            logging.exception("⚠️ [clocks] failed to read SYSTEM config (ignored)")

    campaign_payload = {
        "location": location,
        "started_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
    }
    if set_dac is not None:
        campaign_payload["ocxo_dac"] = float(set_dac)
    if calibrate_ocxo:
        campaign_payload["calibrate_ocxo"] = True

    # --- Flash-cut preamble ---
    if flash_cut:
        prev_campaign = active_row["campaign"]
        prev_location = active_row["payload"].get("location")

        logging.info(
            "⚡ [start] FLASH CUT: '%s' → '%s' (seamless campaign switch, no stop)",
            prev_campaign, campaign,
        )

        campaign_payload["flash_cut_from"] = prev_campaign

        # Carry forward the previous location if the new campaign doesn't
        # specify one.  This preserves GNSS TO mode across the cut.
        if location is None and prev_location:
            location = prev_location
            campaign_payload["location"] = location
            logging.info(
                "📡 [start] inheriting location '%s' from previous campaign",
                location,
            )

    # Deactivate prior campaigns + create new one (atomic in single transaction)
    cutover_ts = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    with open_db() as conn:
        cur = conn.cursor()
        cur.execute(
            """
            UPDATE campaigns
            SET active = false,
                payload = payload || jsonb_build_object('stopped_at', to_jsonb(%s::text))
            WHERE active = true
            """,
            (cutover_ts,),
        )
        cur.execute(
            """
            INSERT INTO campaigns (campaign, active, payload)
            VALUES (%s, true, %s)
            """,
            (campaign, json.dumps(campaign_payload)),
        )

    # GNSS mode is now owned by the authoritative system-level current
    # location, not by campaign arguments.  It was already enforced above.

    # --- Stop Teensy + PITIMER before arming ---
    # On flash-cut this is still needed: the Teensy STOP clears its ISR
    # state so the subsequent START can arm cleanly at pps_count=0.
    # The STOP is instantaneous — the PPS stream from the GNSS receiver
    # continues, and the OCXOs keep oscillating.  We're just resetting
    # the Teensy's software counters and PITIMER's epoch.
    _request_teensy_stop_best_effort()
    _stop_pitimer_best_effort()

    # Drain stale fragments from the previous campaign.  Without this,
    # the processor thread may dequeue fragments that belonged to the
    # old campaign, call _get_active_campaign() (which now returns the
    # NEW campaign), and persist them under the wrong campaign name.
    # Those stale records create duplicate pps_count values when the
    # new campaign's counters eventually reach the same pps_count.
    while not _fragment_queue.empty():
        try:
            _fragment_queue.get_nowait()
        except queue.Empty:
            break

    _reset_trackers()

    # Prepare sync wait FIRST
    _begin_sync_wait(expected_pps=0)

    # Arm TEENSY first — slow path (serial round-trip)
    logging.info("📡 [start] @%s arming TEENSY START: pps_count=0", system_time_z())
    teensy_args: Dict[str, Any] = {"campaign": campaign}
    if set_dac is not None:
        teensy_args["set_dac"] = str(set_dac)
    if calibrate_ocxo:
        teensy_args["calibrate_ocxo"] = "true"

    _request_teensy_start(campaign=campaign, pps_count=0, args=teensy_args)
    logging.info("📡 [start] @%s TEENSY START returned — starting PITIMER", system_time_z())

    # v4: Start PITIMER with pi_ns=0.  At pps_count=0, pi_ns is 0 by
    # definition.  PITIMER derives epoch = corrected - 0 = corrected
    # from the first capture.  No PEEK, no SET_EPOCH needed.
    _start_pitimer(pps_count=0, pi_ns=0)

    # The sync fragment (pps_count=0) will also be enqueued for the
    # processor thread.  Arm at 0 so the processor accepts it and
    # advances to 1 itself.
    _armed_pps_count = 0
    _diag["armed_pps_count"] = 0
    logging.info("📡 [start] @%s both armed for pps_count=0 — waiting for sync fragment...", system_time_z())

    # Activate campaign BEFORE waiting for sync so the processor thread
    # will process the sync fragment when it arrives (rather than
    # discarding it as "no campaign").
    _campaign_active = True

    # Wait for the sync fragment
    try:
        frag0, waited_s = _end_sync_wait()
    except Exception:
        return {"success": False, "message": "HARD FAULT during START sync (see logs/diag)"}

    logging.info("✅ [start] @%s sync fragment received (waited=%.3fs)", system_time_z(), waited_s)

    if flash_cut:
        logging.info(
            "⚡ [start] @%s FLASH CUT complete — '%s' → '%s' (no interruption)",
            system_time_z(), prev_campaign, campaign,
        )
    else:
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
            "waited_s": round(float(waited_s), 3),
            "flash_cut": flash_cut,
            "previous_campaign": active_row["campaign"] if flash_cut else None,
        },
    }

def cmd_stop(_: Optional[dict]) -> dict:
    global _campaign_active

    row = _get_active_campaign()
    stop_location = (
        row["payload"].get("location")
        if row and isinstance(row.get("payload"), dict)
        else _get_current_location()
    )

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

    try:
        effective_location = _ensure_gnss_mode_for_current_location()
        if effective_location:
            logging.info(
                "📡 [clocks] stop complete — GNSS kept in TO mode for current location '%s' (campaign snapshot was '%s')",
                effective_location,
                stop_location,
            )
        else:
            logging.info("📡 [clocks] stop complete — GNSS returned to NORMAL mode")
    except Exception:
        logging.exception("⚠️ [clocks] failed to reconcile GNSS mode at stop (ignored)")

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
# RESUME — re-activate an explicitly stopped campaign
# ---------------------------------------------------------------------


def cmd_resume(args: Optional[dict]) -> dict:
    """
    RESUME(campaign)

    Re-activate a previously stopped campaign and recover clocks.

      1. Look up campaign by name (must exist, must NOT be active).
      2. Verify it has TIMEBASE rows (otherwise nothing to recover from).
      3. Re-activate in DB: active=true, clear stopped_at, set STARTED.
      4. Deactivate any OTHER active campaign first.
      5. Delegate to _recover_campaign() for the full clock recovery.

    This is the explicit-command equivalent of boot-time recovery.
    """
    global _campaign_active

    if not args or "campaign" not in args:
        return {"success": False, "message": "RESUME requires 'campaign' argument"}

    campaign_name = args["campaign"]

    # Refuse if there's already an active campaign running
    active_row = _get_active_campaign()
    if active_row is not None:
        if active_row["campaign"] == campaign_name:
            return {
                "success": False,
                "message": f"Campaign '{campaign_name}' is already active — nothing to resume",
            }
        return {
            "success": False,
            "message": (
                f"Campaign '{active_row['campaign']}' is currently active — "
                f"STOP it before resuming '{campaign_name}'"
            ),
        }

    # Look up the campaign
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, campaign, active, payload
            FROM campaigns
            WHERE campaign = %s
            ORDER BY ts DESC
            LIMIT 1
            """,
            (campaign_name,),
        )
        row = cur.fetchone()

    if row is None:
        return {"success": False, "message": f"No campaign named '{campaign_name}'"}

    payload = row["payload"]
    if isinstance(payload, str):
        payload = json.loads(payload)

    if row["active"]:
        return {"success": False, "message": f"Campaign '{campaign_name}' is already active"}

    # Verify it has TIMEBASE rows
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            "SELECT COUNT(*) AS cnt FROM timebase WHERE campaign = %s",
            (campaign_name,),
        )
        cnt_row = cur.fetchone()
    tb_count = cnt_row["cnt"] if cnt_row else 0

    if tb_count == 0:
        return {
            "success": False,
            "message": f"Campaign '{campaign_name}' has no TIMEBASE rows — use START instead",
        }

    # Re-activate
    resumed_at = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    with open_db() as conn:
        cur = conn.cursor()
        cur.execute(
            """
            UPDATE campaigns
            SET active = false
            WHERE active = true AND campaign != %s
            """,
            (campaign_name,),
        )
        cur.execute(
            """
            UPDATE campaigns
            SET active = true,
                payload = payload
                    - 'stopped_at'
                    || jsonb_build_object('resumed_at', to_jsonb(%s::text))
                    || CASE
                        WHEN payload ? 'report'
                        THEN jsonb_build_object(
                            'report',
                            (payload->'report') || '{"campaign_state":"STARTED"}'::jsonb
                        )
                        ELSE '{}'::jsonb
                       END
            WHERE campaign = %s
            """,
            (resumed_at, campaign_name),
        )
        if cur.rowcount == 0:
            return {"success": False, "message": f"Failed to re-activate campaign '{campaign_name}'"}

    logging.info(
        "▶️ [clocks] RESUME: campaign '%s' re-activated (%d TIMEBASE rows) — starting recovery...",
        campaign_name, tb_count,
    )

    try:
        _recover_campaign()
    except Exception as e:
        logging.exception("💥 [clocks] RESUME recovery failed for '%s'", campaign_name)
        _campaign_active = False
        try:
            with open_db() as conn:
                cur = conn.cursor()
                cur.execute(
                    """
                    UPDATE campaigns
                    SET active = false,
                        payload = payload || jsonb_build_object(
                            'stopped_at', to_jsonb(%s::text),
                            'resume_failed', to_jsonb(%s::text)
                        )
                    WHERE campaign = %s AND active = true
                    """,
                    (
                        datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                        str(e),
                        campaign_name,
                    ),
                )
        except Exception:
            pass
        return {"success": False, "message": f"RESUME recovery failed: {e}"}

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "campaign": campaign_name,
            "timebase_rows": tb_count,
            "resumed_at": resumed_at,
        },
    }


# ---------------------------------------------------------------------
# Recovery — v4 Nanosecond Architecture
# ---------------------------------------------------------------------

def _recover_campaign() -> None:
    """
    RECOVER — v4 nanosecond architecture:

    The recovery algorithm is symmetric across all four clock domains.
    CLOCKS speaks only nanoseconds to both the Teensy and PITIMER.
    Each instrument derives its own internal representation.

    Algorithm:
      0. Wait for preflight prerequisites (GNSS lock, chrony PPS
         selected, PITIMER warm).  This replaces the old
         _wait_for_gnss_time() call and ensures the system clock
         is aligned with true PPS edges before arming either side.
      1. Read last TIMEBASE row for clock nanosecond values and
         GNSS wall-clock timestamp.
      2. Get current GNSS wall-clock time.
      3. Compute elapsed whole seconds between the two GNSS times.
      4. next_pps_second = elapsed + 1 (the next PPS edge).
      5. next_pps_count = last_pps_count + next_pps_second.
      6. projected_gnss_ns = next_pps_count * NS_PER_SECOND.
      7. For each clock domain, project forward using the ratio
         from the last TIMEBASE — pure integer arithmetic:
           projected_ns = projected_gnss_ns * last_clock_ns // last_gnss_ns
      8. Send RECOVER to PITIMER with projected pi_ns.
      9. Send RECOVER to Teensy with projected dwt_ns, gnss_ns, ocxo_ns.
     10. Wait for sync fragment, activate campaign.

    No DWT cycle counts.  No Pi epochs.  No +2 padding.  No special cases.
    """
    global _campaign_active, _armed_pps_count

    _diag["recovery_checks"] += 1

    row = _get_active_campaign()
    if row is None:
        _diag["recovery_no_active_campaign"] += 1
        logging.info("🔍 [recovery] no active campaign — nothing to recover")
        return

    campaign_name = row["campaign"]
    campaign_payload = row["payload"]
    campaign_location = campaign_payload.get("location")
    system_location = _get_current_location()
    effective_location = campaign_location or system_location

    logging.info(
        "🔍 [recovery] active campaign found: '%s' (campaign location: %s, system location: %s)",
        campaign_name,
        campaign_location or "none",
        system_location or "none",
    )

    # ------------------------------------------------------------------
    # Step 1: Read last TIMEBASE row
    # ------------------------------------------------------------------
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
        logging.info(
            "ℹ️ [recovery] campaign '%s' has no TIMEBASE rows — "
            "treating as fresh start (cold restart)",
            campaign_name,
        )

        # ---------------------------------------------------------------
        # Cold restart: the campaign was created but no TIMEBASE was ever
        # produced (e.g. the system crashed during the initial START
        # before the first PPS edge arrived).  We re-arm everything at
        # pps_count=0 exactly as cmd_start would, reusing the existing
        # campaign record.
        # ---------------------------------------------------------------

        # Wait for all preflight prerequisites
        _wait_for_preflight("recovery/cold")

        # Restore GNSS TO mode if campaign had location
        try:
            effective_location = _ensure_gnss_mode_for_current_location()
            if effective_location:
                logging.info("📡 [recovery/cold] GNSS ensured in TO mode for '%s'", effective_location)
            else:
                logging.info("📡 [recovery/cold] GNSS ensured in NORMAL mode")
        except Exception as e:
            raise RuntimeError(f"recovery/cold failed: {e}")

        # Wait for PUBSUB routing
        logging.info("⏳ [recovery/cold] @%s waiting for PUBSUB routing...", system_time_z())
        route_wait_t0 = time.monotonic()
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
                            "✅ [recovery/cold] @%s PUBSUB route confirmed (%.1fs)",
                            system_time_z(), elapsed_wait,
                        )
                        break
            except RuntimeError:
                raise
            except Exception:
                pass
            if elapsed_wait >= 30.0:
                raise RuntimeError("recovery/cold failed: PUBSUB route not found in 30s")
            time.sleep(0.5)

        # Stop + drain + reset
        _request_teensy_stop_best_effort()
        _stop_pitimer_best_effort()

        while not _fragment_queue.empty():
            try:
                _fragment_queue.get_nowait()
            except queue.Empty:
                break

        _reset_trackers()

        # Arm at pps_count=0
        _begin_sync_wait(expected_pps=0)

        # Build Teensy args from campaign payload
        teensy_args: Dict[str, Any] = {"campaign": campaign_name}
        recover_ocxo_dac = campaign_payload.get("ocxo_dac")
        recover_calibrate = campaign_payload.get("calibrate_ocxo", False)
        if recover_ocxo_dac is not None:
            teensy_args["set_dac"] = str(float(recover_ocxo_dac))
        if recover_calibrate:
            teensy_args["calibrate_ocxo"] = "true"

        logging.info("📡 [recovery/cold] @%s arming TEENSY START: pps_count=0", system_time_z())
        _request_teensy_start(campaign=campaign_name, pps_count=0, args=teensy_args)

        _start_pitimer(pps_count=0, pi_ns=0)

        _armed_pps_count = 0
        _diag["armed_pps_count"] = 0
        _campaign_active = True

        try:
            frag, waited_s = _end_sync_wait()
        except Exception:
            _campaign_active = False
            raise

        logging.info(
            "✅ [recovery/cold] @%s campaign '%s' cold-restarted — TIMEBASE begins\n"
            "    waited = %.3fs",
            system_time_z(), campaign_name, waited_s,
        )

        _diag["last_recovery"] = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "campaign": campaign_name,
            "mode": "cold_restart",
            "pps_count": 0,
        }
        return

    last_tb = tb_row["payload"]
    if isinstance(last_tb, str):
        last_tb = json.loads(last_tb)

    last_pps_count = last_tb.get("pps_count") or last_tb.get("teensy_pps_count")
    if last_pps_count is None:
        _diag["recovery_missing_last_pps_count"] += 1
        raise RuntimeError("recovery failed: last TIMEBASE missing pps_count")
    last_pps_count = int(last_pps_count)

    last_gnss_ns = int(last_tb.get("teensy_gnss_ns") or 0)
    last_dwt_ns = int(last_tb.get("teensy_dwt_ns") or 0)
    last_ocxo_ns = int(last_tb.get("teensy_ocxo_ns") or 0)
    last_pi_ns = int(last_tb.get("pi_ns") or 0)

    # GNSS wall-clock timestamp from the last TIMEBASE record
    last_gnss_time_str = last_tb.get("gnss_time_utc") or last_tb.get("system_time_utc") or system_time_z()

    if last_gnss_ns == 0:
        logging.warning("⚠️ [recovery] teensy_gnss_ns=0 — using dwt_ns as proxy")
        last_gnss_ns = last_dwt_ns

    logging.info(
        "📐 [recovery] LAST TIMEBASE:\n"
        "    pps_count  = %d\n"
        "    gnss_ns    = %d\n"
        "    dwt_ns     = %d\n"
        "    ocxo_ns    = %d\n"
        "    pi_ns      = %d\n"
        "    gnss_time  = %s",
        last_pps_count, last_gnss_ns, last_dwt_ns, last_ocxo_ns,
        last_pi_ns, last_gnss_time_str,
    )

    # OCXO DAC state from campaign payload
    recover_ocxo_dac = campaign_payload.get("ocxo_dac")
    recover_calibrate = campaign_payload.get("calibrate_ocxo", False)
    recover_converged = campaign_payload.get("servo_converged", False)

    if recover_ocxo_dac is not None:
        logging.info(
            "🔧 [recovery] OCXO DAC: %s (calibrate=%s, converged=%s)",
            recover_ocxo_dac, recover_calibrate, recover_converged,
        )

    # ------------------------------------------------------------------
    # Step 2: Wait for all preflight prerequisites
    #
    # This replaces the old _wait_for_gnss_time() call.  The preflight
    # gate ensures GNSS lock, chrony PPS selection, and PITIMER warmth
    # before we proceed.  Once it passes, the system clock is aligned
    # with true PPS edges, eliminating the pps_count desynchronization
    # that occurred when recovery was attempted before chrony locked.
    # ------------------------------------------------------------------
    try:
        effective_location = _ensure_gnss_mode_for_current_location()
        if effective_location:
            logging.info("📡 [recovery] GNSS ensured in TO mode for '%s'", effective_location)
        else:
            logging.info("📡 [recovery] GNSS ensured in NORMAL mode")
    except Exception as e:
        raise RuntimeError(f"recovery failed: {e}")

    _wait_for_preflight("recovery")

    # ------------------------------------------------------------------
    # Wait for PUBSUB routing to be live
    # ------------------------------------------------------------------
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
            pass

        if elapsed_wait >= ROUTING_WAIT_TIMEOUT_S:
            raise RuntimeError(
                f"recovery failed: PUBSUB route not found in {ROUTING_WAIT_TIMEOUT_S}s"
            )
        time.sleep(ROUTING_WAIT_POLL_S)

    # ------------------------------------------------------------------
    # Stop Teensy + PITIMER, quiesce, snap to second boundary
    # ------------------------------------------------------------------
    logging.info("📡 [recovery] @%s stopping Teensy + PITIMER...", system_time_z())
    _request_teensy_stop_best_effort()
    _stop_pitimer_best_effort()
    time.sleep(2.0)

    now_frac = time.time() % 1.0
    sleep_to_boundary = (1.0 - now_frac) + 0.050
    logging.info("⏳ [recovery] snapping to second boundary (sleeping %.3fs)", sleep_to_boundary)
    time.sleep(sleep_to_boundary)

    # ------------------------------------------------------------------
    # Step 3: Compute elapsed GNSS seconds
    # ------------------------------------------------------------------
    current_gnss_time_str = _get_gnss_time()
    logging.info("📐 [recovery] current GNSS time: %s", current_gnss_time_str)

    last_gnss_utc = datetime.fromisoformat(last_gnss_time_str.replace("Z", "+00:00"))
    current_gnss_utc = datetime.fromisoformat(current_gnss_time_str.replace("Z", "+00:00"))

    elapsed_td = current_gnss_utc - last_gnss_utc
    elapsed_seconds = int(round(elapsed_td.total_seconds()))

    if elapsed_seconds <= 0:
        _diag["recovery_elapsed_seconds_nonpositive"] += 1
        raise RuntimeError(
            f"recovery failed: elapsed_seconds={elapsed_seconds} "
            f"(last={last_gnss_time_str} current={current_gnss_time_str})"
        )

    logging.info(
        "📐 [recovery] GNSS ELAPSED:\n"
        "    last_gnss_time    = %s\n"
        "    current_gnss_time = %s\n"
        "    elapsed_seconds   = %d",
        last_gnss_time_str, current_gnss_time_str, elapsed_seconds,
    )

    # ------------------------------------------------------------------
    # Step 4: next_pps_second = elapsed + 1
    # ------------------------------------------------------------------
    next_pps_second = elapsed_seconds + 1
    next_pps_count = last_pps_count + next_pps_second

    logging.info(
        "📐 [recovery] PPS PROJECTION:\n"
        "    next_pps_second = elapsed(%d) + 1 = %d\n"
        "    next_pps_count  = last(%d) + next_pps_second(%d) = %d",
        elapsed_seconds, next_pps_second,
        last_pps_count, next_pps_second, next_pps_count,
    )

    # ------------------------------------------------------------------
    # Step 5: Symmetric projection — all domains use the same formula
    #
    #   projected_ns = projected_gnss_ns * last_clock_ns // last_gnss_ns
    #
    # The projected values are ABSOLUTE (from campaign start), not
    # relative to the outage.  The Teensy derives campaign_seconds
    # (= teensy_pps_count) from gnss_ns / 1e9, so the nanosecond
    # values must represent the full campaign elapsed time.
    #
    # GNSS projects to next_pps_count seconds by definition (tau = 1.0).
    # Other domains scale by their tau ratio (last_clock_ns / last_gnss_ns).
    # ------------------------------------------------------------------
    projected_gnss_ns = next_pps_count * NS_PER_SECOND  # tau = 1.0 by definition
    projected_dwt_ns = projected_gnss_ns * last_dwt_ns // last_gnss_ns if last_gnss_ns > 0 else projected_gnss_ns
    projected_ocxo_ns = projected_gnss_ns * last_ocxo_ns // last_gnss_ns if (last_gnss_ns > 0 and last_ocxo_ns > 0) else 0
    projected_pi_ns = projected_gnss_ns * last_pi_ns // last_gnss_ns if (last_gnss_ns > 0 and last_pi_ns > 0) else 0

    # Log the implicit tau values for verification
    tau_dwt = last_dwt_ns / last_gnss_ns if last_gnss_ns > 0 else 1.0
    tau_ocxo = last_ocxo_ns / last_gnss_ns if (last_gnss_ns > 0 and last_ocxo_ns > 0) else 1.0
    tau_pi = last_pi_ns / last_gnss_ns if (last_gnss_ns > 0 and last_pi_ns > 0) else 1.0

    logging.info(
        "📐 [recovery] SYMMETRIC PROJECTION (all nanoseconds):\n"
        "    formula: projected_ns = projected_gnss_ns × last_clock_ns ÷ last_gnss_ns\n"
        "    projected_gnss_ns = next_pps_count(%d) × NS_PER_SECOND = %d\n"
        "    last_gnss_ns      = %d\n"
        "    ---\n"
        "    GNSS:  projected = %d  (tau = 1.000000000000)\n"
        "    DWT:   projected = %d  (tau = %.12f, last_dwt_ns = %d)\n"
        "    OCXO:  projected = %d  (tau = %.12f, last_ocxo_ns = %d)\n"
        "    Pi:    projected = %d  (tau = %.12f, last_pi_ns = %d)",
        next_pps_count, projected_gnss_ns, last_gnss_ns,
        projected_gnss_ns,
        projected_dwt_ns, tau_dwt, last_dwt_ns,
        projected_ocxo_ns, tau_ocxo, last_ocxo_ns,
        projected_pi_ns, tau_pi, last_pi_ns,
    )

    _diag["last_recovery"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "campaign": campaign_name,
        "last_pps_count": int(last_pps_count),
        "last_gnss_time": last_gnss_time_str,
        "current_gnss_time": current_gnss_time_str,
        "elapsed_seconds": int(elapsed_seconds),
        "next_pps_second": int(next_pps_second),
        "next_pps_count": int(next_pps_count),
        "projected_gnss_ns": int(projected_gnss_ns),
        "projected_dwt_ns": int(projected_dwt_ns),
        "projected_ocxo_ns": int(projected_ocxo_ns),
        "projected_pi_ns": int(projected_pi_ns),
        "tau_dwt": round(tau_dwt, 12),
        "tau_ocxo": round(tau_ocxo, 12),
        "tau_pi": round(tau_pi, 12),
    }

    _reset_trackers()

    # ------------------------------------------------------------------
    # Step 6: Begin sync wait, arm PITIMER RECOVER then Teensy RECOVER
    #
    # PITIMER is armed FIRST.  The Teensy's two-phase ISR + ASAP
    # architecture allows it to apply RECOVER retroactively to an edge
    # whose ISR already fired.  PITIMER cannot — once ppscapture_next()
    # returns and the capture is discarded as idle, that edge is gone.
    # By arming PITIMER first, we maximize the window for PITIMER to be
    # in RUNNING state when the next PPS edge arrives.
    # ------------------------------------------------------------------
    _begin_sync_wait(expected_pps=int(next_pps_count))

    # PITIMER RECOVER — symmetric, nanoseconds only (FIRST)
    logging.info(
        "📡 [recovery] @%s arming PITIMER RECOVER:\n"
        "    pps_count = %d\n"
        "    pi_ns     = %d",
        system_time_z(), next_pps_count, projected_pi_ns,
    )

    _recover_pitimer(pps_count=int(next_pps_count), pi_ns=int(projected_pi_ns))

    # Teensy RECOVER — all nanoseconds, no cycle counts (SECOND)
    logging.info(
        "📡 [recovery] @%s arming TEENSY RECOVER:\n"
        "    pps_count = %d\n"
        "    dwt_ns    = %d\n"
        "    gnss_ns   = %d\n"
        "    ocxo_ns   = %d",
        system_time_z(), next_pps_count,
        projected_dwt_ns, projected_gnss_ns, projected_ocxo_ns,
    )

    teensy_recover_args: Dict[str, Any] = {
        "campaign": campaign_name,  # <-- add this
        "dwt_ns": str(int(projected_dwt_ns)),
        "gnss_ns": str(int(projected_gnss_ns)),
        "ocxo_ns": str(int(projected_ocxo_ns)),
    }
    if recover_ocxo_dac is not None:
        teensy_recover_args["set_dac"] = str(float(recover_ocxo_dac))
    if recover_calibrate and not recover_converged:
        teensy_recover_args["calibrate_ocxo"] = "true"

    _request_teensy_recover(int(next_pps_count), teensy_recover_args)

    # The sync fragment will also be enqueued for the processor thread.
    # Arm at next_pps_count so the processor accepts it and advances itself.
    _armed_pps_count = int(next_pps_count)
    _diag["armed_pps_count"] = int(next_pps_count)

    # Activate campaign BEFORE waiting for sync so the processor thread
    # will process the sync fragment when it arrives (rather than
    # discarding it as "no campaign").
    _campaign_active = True
    try:
        frag, waited_s = _end_sync_wait(timeout_s=SYNC_RECOVER_TIMEOUT_S)
    except Exception:
        _campaign_active = False
        raise

    logging.info("✅ [recovery] @%s sync fragment received (waited=%.3fs)", system_time_z(), waited_s)

    teensy_pps_count = int(frag.get("teensy_pps_count", next_pps_count))
    overshoot = teensy_pps_count - next_pps_count

    if overshoot != 0:
        logging.info(
            "📐 [recovery] pps_count overshoot: teensy=%d projected=%d (overshoot=%+d)",
            teensy_pps_count, next_pps_count, overshoot,
        )

    # The sync fragment is also in the processor queue.  Arm at
    # teensy_pps_count so the processor accepts it and advances to +1.
    _armed_pps_count = teensy_pps_count
    _diag["armed_pps_count"] = _armed_pps_count

    # ------------------------------------------------------------------
    # Handle overshoot: re-arm PITIMER pps_count to align with Teensy
    #
    # PITIMER RECOVER armed it at next_pps_count, but if the Teensy
    # landed at next_pps_count + overshoot, PITIMER's numbering is off.
    # Re-arm so subsequent captures align.
    # ------------------------------------------------------------------
    if overshoot != 0:
        new_pitimer_next = teensy_pps_count + 1
        logging.info(
            "📐 [recovery] re-arming PITIMER: SET_PPS_COUNT=%d (align with Teensy after overshoot)",
            new_pitimer_next,
        )
        try:
            resp = send_command(
                machine="PI",
                subsystem="PITIMER",
                command="SET_PPS_COUNT",
                args={"pps_count": int(new_pitimer_next)},
            )
            if not resp.get("success"):
                logging.warning(
                    "⚠️ [recovery] SET_PPS_COUNT failed: %s (continuing anyway)",
                    resp.get("message", "?"),
                )
        except Exception:
            logging.warning("⚠️ [recovery] SET_PPS_COUNT IPC failed (continuing anyway)")

    # ------------------------------------------------------------------
    # Step 8: Campaign already active (activated before sync wait).
    # Log final state.
    # ------------------------------------------------------------------

    logging.info(
        "✅ [recovery] @%s campaign '%s' recovered — TIMEBASE resumes\n"
        "    teensy_pps_count = %d\n"
        "    overshoot        = %+d\n"
        "    armed_pps_count  = %d",
        system_time_z(), campaign_name,
        teensy_pps_count, overshoot, _armed_pps_count,
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

# ---------------------------------------------------------------------
# Preflight gate — prerequisites for campaign start/recovery
# ---------------------------------------------------------------------
#
# The system must not attempt to start or recover a campaign until the
# physical measurement chain is trustworthy.  This function checks a
# strict set of prerequisites and returns a structured verdict.
#
# Design:
#   • Every constraint is checked independently and reported by name.
#   • The function returns (ready: bool, reasons: List[str]).
#   • If ready is False, reasons contains a human-readable line for
#     each unmet constraint — suitable for direct logging.
#   • If ready is True, reasons is empty.
#   • Callers should retry at calm intervals (e.g. 30s) until ready.
#
# Constraints:
#   1. GNSS time must be valid (GF-8802 has time and date from satellites)
#   2. GNSS lock quality must be MEDIUM or STRONG (not WEAK)
#   3. GNSS PPS must be valid (discipline loop is active — TPS4 reporting)
#   4. Chrony must have selected the PPS refclock as its time source
#   5. PITIMER capture loop must be alive (producing captures)
#
# Rationale:
#   Without GNSS lock, the PPS signal is unanchored — measurements are
#   meaningless.  Without chrony PPS selection, the Pi system clock
#   second boundaries do not align with true PPS edges, which causes
#   PITIMER's ppscapture_next() to detect phantom boundaries.  This
#   desynchronizes the pps_count assignment between PITIMER and the
#   Teensy, producing the off-by-one failure observed in cold restart.
#
# This function must be added to clocks/core.py.  It requires
# `import subprocess` to be added to the imports.
# ---------------------------------------------------------------------


def _check_preflight() -> tuple[bool, list[str]]:
    """
    Check whether the system is ready to start or recover a campaign.

    Returns:
        (ready, reasons) where ready is True if all prerequisites are
        met, and reasons is a list of human-readable strings describing
        each unmet constraint.
    """
    reasons: list[str] = []

    # -----------------------------------------------------------------
    # 1. GNSS time valid
    # -----------------------------------------------------------------
    try:
        gnss_resp = send_command(machine="PI", subsystem="GNSS", command="REPORT")
        if not gnss_resp.get("success"):
            reasons.append("GNSS REPORT unavailable")
        else:
            gnss = gnss_resp.get("payload", {})

            # 1a. Time valid
            if not gnss.get("time_valid"):
                reasons.append("GNSS time not valid (no satellite time/date)")

            # 2. Lock quality
            lock_quality = gnss.get("lock_quality", "WEAK")
            if lock_quality == "WEAK":
                reasons.append(
                    f"GNSS lock quality is WEAK "
                    f"(satellites={gnss.get('satellites', '?')}, "
                    f"hdop={gnss.get('hdop', '?')})"
                )

            # 3. PPS valid (discipline loop active)
            if not gnss.get("pps_valid"):
                reasons.append("GNSS PPS not valid (discipline loop not active)")
            else:
                # Extra: check that discipline is in a good state
                discipline = gnss.get("discipline", {})
                freq_mode = discipline.get("freq_mode", -1)
                freq_mode_name = discipline.get("freq_mode_name", "UNKNOWN")
                # freq_mode: 0=IDLE, 1=WARM_UP, 2=COARSE_LOCK, 3=FINE_LOCK, 4=HOLDOVER, 5=RECOVERY
                # We require at least COARSE_LOCK (2) for recovery, FINE_LOCK (3) is ideal
                if freq_mode < 2:
                    reasons.append(
                        f"GNSS discipline not locked "
                        f"(freq_mode={freq_mode} '{freq_mode_name}', "
                        f"need at least COARSE_LOCK)"
                    )

    except Exception as e:
        reasons.append(f"GNSS REPORT failed: {e}")

    # -----------------------------------------------------------------
    # 4. Chrony PPS selected
    # -----------------------------------------------------------------
    try:
        result = subprocess.run(
            ["chronyc", "-c", "sources"],
            capture_output=True, text=True, timeout=5,
        )
        if result.returncode != 0:
            reasons.append(f"chronyc sources failed (rc={result.returncode})")
        else:
            # chronyc -c sources outputs CSV lines:
            # mode,state,name,stratum,poll,reach,last_rx,last_sample_offset,last_sample_err
            # We need a line where name contains "PPS" and state == "*" (selected)
            # In -c mode: field[0] = mode char (^=server, #=refclock), field[1] = state char
            pps_selected = False
            for line in result.stdout.strip().splitlines():
                fields = line.split(",")
                if len(fields) >= 3:
                    state = fields[1]
                    name = fields[2]
                    if "PPS" in name.upper() and state == "*":
                        pps_selected = True
                        break

            if not pps_selected:
                reasons.append(
                    "chrony has not selected PPS as time source "
                    "(system clock may not align with PPS edges)"
                )

    except subprocess.TimeoutExpired:
        reasons.append("chronyc sources timed out")
    except FileNotFoundError:
        reasons.append("chronyc not found")
    except Exception as e:
        reasons.append(f"chrony check failed: {e}")

    # -----------------------------------------------------------------
    # 5. PITIMER capture loop alive
    # -----------------------------------------------------------------
    try:
        pit_resp = send_command(machine="PI", subsystem="PITIMER", command="REPORT")
        if not pit_resp.get("success"):
            reasons.append("PITIMER REPORT unavailable")
        else:
            pit = pit_resp.get("payload", {})
            diag = pit.get("diag", {})
            captures_total = diag.get("captures_total", 0)
            if captures_total < 3:
                reasons.append(
                    f"PITIMER capture loop not warm "
                    f"(captures_total={captures_total}, need >= 3)"
                )

    except Exception as e:
        reasons.append(f"PITIMER REPORT failed: {e}")

    ready = len(reasons) == 0
    return ready, reasons


def _wait_for_preflight(context: str = "recovery") -> None:
    """
    Block until all preflight prerequisites are met.

    Logs the unmet constraints every PREFLIGHT_POLL_INTERVAL_S seconds
    so the operator can see exactly what the system is waiting for.

    Args:
        context: "recovery" or "start" — used only in log messages.
    """
    attempt = 0
    t0 = time.monotonic()

    while True:
        ready, reasons = _check_preflight()

        if ready:
            elapsed = time.monotonic() - t0
            if attempt > 0:
                logging.info(
                    "%s @%s all prerequisites met after %d checks (%.0fs) — "
                    "proceeding with %s",
                    PREFLIGHT_LOG_PREFIX, system_time_z(),
                    attempt, elapsed, context,
                )
            else:
                logging.info(
                    "%s @%s all prerequisites met — proceeding with %s",
                    PREFLIGHT_LOG_PREFIX, system_time_z(), context,
                )
            return

        attempt += 1
        elapsed = time.monotonic() - t0

        # Log each unmet constraint on its own line for clarity
        reason_block = "\n".join(f"    • {r}" for r in reasons)
        logging.info(
            "%s @%s not ready for %s (check #%d, %.0fs elapsed):\n%s",
            PREFLIGHT_LOG_PREFIX, system_time_z(),
            context, attempt, elapsed, reason_block,
        )

        time.sleep(PREFLIGHT_POLL_INTERVAL_S)


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
# LIST_CAMPAIGNS
# ---------------------------------------------------------------------


def cmd_list_campaigns(_: Optional[dict]) -> Dict[str, Any]:
    """
    LIST_CAMPAIGNS

    Return a compact list of all campaigns with name, start/stop times,
    active status, pps_count, and whether it's the current baseline.
    """
    baseline_info = _get_baseline_from_config()
    baseline_campaign = baseline_info.get("baseline_campaign") if baseline_info else None

    try:
        with open_db(row_dict=True) as conn:
            cur = conn.cursor()
            cur.execute(
                """
                SELECT id, campaign, active, ts, payload
                FROM campaigns
                ORDER BY ts ASC
                """
            )
            rows = cur.fetchall()
    except Exception as e:
        logging.exception("❌ [clocks] LIST_CAMPAIGNS query failed")
        return {"success": False, "message": str(e)}

    campaigns = []
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)

        report = payload.get("report", {})
        is_baseline = (row["campaign"] == baseline_campaign)

        entry: Dict[str, Any] = {
            "campaign": row["campaign"],
            "active": bool(row["active"]),
            "baseline": is_baseline,
            "started_at": payload.get("started_at"),
            "stopped_at": payload.get("stopped_at"),
            "resumed_at": payload.get("resumed_at"),
            "location": payload.get("location"),
            "pps_count": report.get("pps_count"),
        }
        campaigns.append(entry)

    lines = []
    for c in campaigns:
        star = " ★" if c["baseline"] else ""
        state = "ACTIVE" if c["active"] else "stopped"
        started = (c["started_at"] or "?")[:19]
        stopped = (c["stopped_at"] or "—")[:19] if not c["active"] else "—"
        pps = c["pps_count"] if c["pps_count"] is not None else "—"
        loc = c["location"] or "—"
        lines.append(
            f"  {c['campaign']}{star}  [{state}]  "
            f"start={started}  stop={stopped}  "
            f"pps={pps}  loc={loc}"
        )

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "count": len(campaigns),
            "campaigns": campaigns,
            "text": "\n".join(lines) if lines else "(no campaigns)",
        },
    }


# ---------------------------------------------------------------------
# CLOCKS_INFO
# ---------------------------------------------------------------------


def cmd_clocks_info(_: Optional[dict]) -> Dict[str, Any]:
    current_location = _get_current_location()
    payload = {
        "campaign_active": _campaign_active,
        "current_location": current_location,
        "time_only_capable": _location_has_time_only_profile(current_location),
        "machine_clock_data": _get_machine_clock_data(current_location),
        "last_pps_count_seen": _last_pps_count_seen,
        "last_pi_seq": _last_pi_seq,
        "last_pi_corrected": _last_pi_corrected,
        "sync_expected_pps": _sync_expected_pps,
        "diag": _diag,
    }
    return {"success": True, "message": "OK", "payload": payload}


# ============================================================================
# 4. cmd_set_dac — accept and persist float
# ============================================================================
#
# Full replacement:
#

def cmd_set_dac(args: Optional[dict]) -> Dict[str, Any]:
    """
    SET_DAC(dac)

    Update the OCXO DAC value in the SYSTEM config record.
    This becomes the default DAC for subsequent campaign starts.
    Accepts integer or decimal values (e.g. 3054 or 3054.238).
    """
    if not args or "dac" not in args:
        return {"success": False, "message": "SET_DAC requires 'dac' argument"}

    try:
        dac = float(args["dac"])
    except (ValueError, TypeError):
        return {"success": False, "message": f"Invalid dac value: {args['dac']}"}

    if dac < 0 or dac > 4095:
        return {"success": False, "message": f"DAC value {dac} out of range (0–4095)"}

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

    logging.info("🔧 [clocks] SET_DAC: ocxo_dac=%s", dac)
    return {"success": True, "message": "OK", "payload": {"ocxo_dac": dac}}

def cmd_set_location(args: Optional[dict]) -> Dict[str, Any]:
    """
    SET_LOCATION(location)

    Update the authoritative system-level current location in the
    SYSTEM config record.

    This location is used by CLOCKS to determine GNSS steady-state
    mode (TO vs NORMAL) and to resolve location-specific machine
    clock data.

    If the named location does not exist in the locations table,
    the command is rejected.
    """
    if not args or "location" not in args:
        return {"success": False, "message": "SET_LOCATION requires 'location' argument"}

    raw_location = args["location"]
    if not isinstance(raw_location, str):
        return {"success": False, "message": "SET_LOCATION location must be a string"}

    location = raw_location.strip()
    if not location:
        return {"success": False, "message": "SET_LOCATION location may not be empty"}

    row = _get_location_record(location)
    if row is None:
        return {"success": False, "message": f"No location named '{location}'"}

    try:
        with open_db() as conn:
            cur = conn.cursor()
            cur.execute(
                """
                UPDATE config
                SET payload = payload || %s::jsonb
                WHERE config_key = 'SYSTEM'
                """,
                (json.dumps({"current_location": location}),),
            )
            if cur.rowcount == 0:
                return {"success": False, "message": "No SYSTEM config record found"}
    except Exception as e:
        logging.exception("❌ [clocks] SET_LOCATION failed")
        return {"success": False, "message": str(e)}

    try:
        _ensure_gnss_mode_for_current_location()
    except Exception as e:
        return {
            "success": False,
            "message": f"Location set but GNSS mode reconciliation failed: {e}",
            "payload": {"current_location": location},
        }

    logging.info("📍 [clocks] SET_LOCATION: current_location='%s'", location)
    return {
        "success": True,
        "message": "OK",
        "payload": {
            "current_location": location,
            "time_only_capable": _location_has_time_only_profile(location),
            "machine_clock_data": _get_machine_clock_data(location),
        },
    }


def cmd_clear_location(_: Optional[dict]) -> Dict[str, Any]:
    """
    CLEAR_LOCATION()

    Remove the authoritative system-level current location from the
    SYSTEM config record.

    This is used for free-roaming operation or while profiling a new
    location that has not yet been established as the current site.
    """
    try:
        with open_db() as conn:
            cur = conn.cursor()
            cur.execute(
                """
                UPDATE config
                SET payload = payload - 'current_location'
                WHERE config_key = 'SYSTEM'
                """
            )
            if cur.rowcount == 0:
                return {"success": False, "message": "No SYSTEM config record found"}
    except Exception as e:
        logging.exception("❌ [clocks] CLEAR_LOCATION failed")
        return {"success": False, "message": str(e)}

    try:
        _ensure_gnss_mode_for_current_location()
    except Exception as e:
        return {
            "success": False,
            "message": f"Location cleared but GNSS mode reconciliation failed: {e}",
        }

    logging.info("📍 [clocks] CLEAR_LOCATION: current_location cleared")
    return {
        "success": True,
        "message": "OK",
        "payload": {
            "current_location": None,
        },
    }

COMMANDS = {
    "START": cmd_start,
    "STOP": cmd_stop,
    "RESUME": cmd_resume,
    "REPORT": cmd_report,
    "CLEAR": cmd_clear,
    "DELETE": cmd_delete,
    "SET_DAC": cmd_set_dac,
    "SET_LOCATION": cmd_set_location,
    "CLEAR_LOCATION": cmd_clear_location,
    "SET_BASELINE": cmd_set_baseline,
    "BASELINE_INFO": cmd_baseline_info,
    "LIST_CAMPAIGNS": cmd_list_campaigns,
    "CLOCKS_INFO": cmd_clocks_info,
}

# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------


def run() -> None:
    setup_logging()

    logging.info(
        "🕐 [clocks] v4 Nanosecond Recovery Architecture. "
        "CLOCKS speaks ONLY nanoseconds to Teensy and PITIMER. "
        "Each instrument derives its own internal representation. "
        "Recovery is symmetric: projected_ns = projected_gnss_ns × last_clock_ns ÷ last_gnss_ns. "
        "No DWT cycle counts. No Pi epochs. No SET_EPOCH in normal flow. "
        "START uses PITIMER START(pi_ns=0). "
        "START while active performs seamless flash-cut to new campaign. "
        "RECOVER uses PITIMER RECOVER(pps_count, pi_ns). "
        "Commands: START, STOP, RESUME, REPORT, CLEAR, DELETE, SET_DAC, "
        "SET_BASELINE, BASELINE_INFO, LIST_CAMPAIGNS, CLOCKS_INFO."
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
        try:
            effective_location = _ensure_gnss_mode_for_current_location()
            if effective_location:
                logging.info("📡 [clocks] boot idle state — GNSS kept in TO mode for '%s'", effective_location)
            else:
                logging.info("📡 [clocks] boot idle state — GNSS in NORMAL mode")
        except Exception:
            logging.exception("⚠️ [clocks] failed to reconcile GNSS mode at boot idle")
    else:
        _recover_campaign()

    # Block forever
    logging.info("🏁 [clocks] entering main loop")
    while True:
        time.sleep(3600)


if __name__ == "__main__":
    run()