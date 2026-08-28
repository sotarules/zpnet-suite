"""
ZPNet PHOTONS Process — canonical optical instrument + LANTERN campaign lifecycle.

Campaign symmetry contract:

    Pi LANTERN lifecycle authority
        -> Teensy PHOTONS.START / PHOTONS.STOP
        -> firmware-authored LANTERN_FRAGMENT_V1 boundary + CAMP statistics
        -> PHOTONS_FRAGMENT_V1
        -> Pi structural/accounting court + SYSTEM context
        -> Pi adds durable campaign ID/baseline provenance only
        -> canonical PHOTONS_V1
        -> ordered persistence + campaign_master read model

The Teensy remains the optical-science authority and runs continuously.
START/STOP never start or stop physical PHOTONS measurement and never reset the
always-on Welford/Better-Buckets population.  As in CLOCKS, START only tells the
firmware which recording lifecycle is beginning; firmware snapshots its own
cumulative accepted-lap N/T at a published boundary and authors CAMP PPB from
that origin against STANDARD_LAP_NS.

The Pi does not recompute, smooth, repair, or re-adjudicate lap or CAMP science.
It owns campaign_master identity, baseline relationships, restart policy, and
persistence.  Firmware owns the exact START/STOP measurement boundary.

Baselines remain campaign-to-campaign relationships stored by campaign_master ID.
No baseline statistics are copied into firmware and ``photons.baseline`` remains
untouched.  Durable recovery restores aggregate sufficient state plus only the
bounded PPB endpoint history the Pi literally possesses.  A surviving producer is
never repaired or replayed: Pi proves monotonic descent from the durable snapshot,
reacquires the producer's exact current bounded rings through the read-only export
court, restores Pi-owned custody, and adopts the producer's current LANTERN state.
If the producer was lost, held restore may deliberately install only the literal
suffix Pi already possesses and surrender unseen rolling ancestry instead of
inventing it.  Physical edge ancestry is always reacquired after restart.
"""

from __future__ import annotations

import copy
import hashlib
import json
from collections import deque
from dataclasses import dataclass
import logging
import math
import queue
import threading
import time
from datetime import datetime, timezone
from decimal import Decimal, InvalidOperation
from typing import Any, Dict, List, Optional, Tuple

from zpnet.processes.processes import (
    publish,
    send_command,
    server_setup,
)
from zpnet.shared.constants import Payload
from zpnet.shared.db import open_db
from zpnet.shared.logger import setup_logging


# ---------------------------------------------------------------------
# Contract
# ---------------------------------------------------------------------

SUBSYSTEM = "PHOTONS"
PHOTONS_FRAGMENT_TOPIC = "PHOTONS_FRAGMENT"
PHOTONS_TOPIC = "PHOTONS"

# PHOTONS is the always-on optical instrument.  LANTERN is the optional
# campaign interpretation layered over that stream.  Pre-Phase-4 rows used the
# provisional PHOTONS campaign_type and remain valid historical bring-up data;
# new generalized campaign_detail rows use LANTERN, matching CLOCKS/TEMPEST.
CAMPAIGN_TYPE_PHOTONS = "PHOTONS"  # legacy pre-Phase-4 detail type
CAMPAIGN_TYPE_LANTERN = "LANTERN"
LANTERN_CAMPAIGN_SCHEMA = "LANTERN_CAMPAIGN_V1"
LANTERN_FRAGMENT_SCHEMA = "LANTERN_FRAGMENT_V1"
LANTERN_MASTER_SCHEMA = "LANTERN_CAMPAIGN_MASTER_V1"
LANTERN_REPORT_SCHEMA = "LANTERN_REPORT_V1"

# Canonical Pi-side instrument schema and accepted firmware source schemas.
PHOTONS_SCHEMA = "PHOTONS_V1"
PHOTONS_FRAGMENT_SCHEMA = "PHOTONS_FRAGMENT_V1"
PHOTONS_INSTRUMENT_SCHEMA = "PHOTONS_INSTRUMENT_V1"
PHOTONS_SCIENCE_SCHEMA = "PHOTONS_SCIENCE_V2"
PHOTONS_STATS_SCHEMA = "PHOTONS_INSTRUMENT_STATS_V1"
PHOTONS_RACE_SCHEMA = "PHOTONS_RACE_V1"
PHOTONS_RACE_CADENCE_HZ = 1000
PHOTONS_RACE_CADENCE_NS = 1_000_000
PHOTONS_RACE_PULSE_NS = 91_000
PHOTONS_PPB_SEMANTICS = "LAP_BASELINE_NS_OFFSET_V1"

PHOTONS_RECOVERY_SCHEMA_VERSION = 1
PHOTONS_RECOVERY_CHUNK_MAX_ENDPOINTS = 4
PHOTONS_RECOVERY_10_MIN_SECONDS = 10 * 60
PHOTONS_RECOVERY_60_MIN_SECONDS = 60 * 60
PHOTONS_RECOVERY_8_HOUR_SECONDS = 8 * 60 * 60
PHOTONS_RECOVERY_24_HOUR_SECONDS = 24 * 60 * 60
PHOTONS_RECOVERY_SECOND_CAPACITY = PHOTONS_RECOVERY_10_MIN_SECONDS + 1
PHOTONS_RECOVERY_MINUTE_CAPACITY = 24 * 60 + 2
PHOTONS_RECOVERY_PROOF_TIMEOUT_S = 180.0
PHOTONS_RECOVERY_VERIFY_TOLERANCE = 1.0e-6
# Cross-representation corroboration only.  Integer N/T and an incrementally
# accumulated Welford mean are mathematically equivalent but do not share the
# same floating-point path.  A large divergence is worth surfacing to the
# operator, but it must never silently choose an older durable ancestor.
PHOTONS_WELFORD_GRAND_RATIO_DIAGNOSTIC_PPB = 0.1

# CLOCKS-parity checkpoint testimony. Firmware publishes one compact delta/proof
# per second; Pi PHOTONS persists the literal bounded recovery custody it actually
# observed.  A gap may leave that image temporarily incomplete.  Held restore may
# stage the exact retained suffix (plus exact epoch-origin testimony when useful),
# but PostgreSQL history is never replayed and unseen endpoints are never invented.
PHOTONS_PPB_FIRMWARE_DELTA_SCHEMA = "PHOTONS_PPB_CHECKPOINT_DELTA_V1"
PHOTONS_PPB_PI_CHECKPOINT_SCHEMA = "PI_PHOTONS_PPB_RESTORE_CHECKPOINT_V1"
PHOTONS_RECOVERY_CONFIG_KEY = "PHOTONS_RECOVERY"
PHOTONS_RECOVERY_SNAPSHOT_SCHEMA = "PI_PHOTONS_RECOVERY_SNAPSHOT_V1"
PHOTONS_RECOVERY_DESIRED_STATE_SCHEMA = "PI_PHOTONS_RECOVERY_DESIRED_STATE_V1"
PHOTONS_RECOVERY_RECEIPT_SCHEMA = "PI_PHOTONS_RECOVERY_RECEIPT_V1"

TEENSY_CAMPAIGN_START_ACCEPTED_STATUSES = {"start_requested", "flash_cut_requested"}
TEENSY_CAMPAIGN_STOP_ACCEPTED_STATUSES = {"stop_requested"}

# Phase-2 live data-plane queues.  maxsize=0 is intentionally unbounded: at the
# current 1 Hz instrument rate, temporary downstream stalls should create visible
# backlog rather than block PUBSUB ingress or silently discard valid testimony.
PHOTONS_INGRESS_QUEUE_MAXSIZE = 0
PHOTONS_PERSIST_QUEUE_MAXSIZE = 0
PHOTONS_STATE_RETRY_S = 0.25
PHOTONS_PERSIST_RETRY_S = 0.25

# Application-level startup infrastructure admission.  Keep PHOTONS_FRAGMENT
# ingress open while these facts mature, but do not perform durable recovery or
# issue Teensy commands until SYSTEM has proved both planes usable.
STARTUP_INFRASTRUCTURE_REQUIRED = (
    "PI.SYSTEM.POSTGRES",
    "PI.PUBSUB.TEENSY_RPC",
)
STARTUP_INFRASTRUCTURE_POLL_S = 0.5
STARTUP_INFRASTRUCTURE_QUIET_GRACE_S = 5.0
STARTUP_INFRASTRUCTURE_STATUS_LOG_INTERVAL_S = 30.0

# Runtime producer-lifetime reconciliation.  PUBSUB transport generation is an
# infrastructure boundary, not by itself proof that the Teensy rebooted.  PHOTONS
# pauses canonical authorship on a proved generation change, then asks the
# firmware recovery surface whether publication survived.  A surviving producer
# is released untouched; a newborn held producer is resurrected from the newest
# durable PHOTONS boundary through the same exact-N+1 court used at startup.
PHOTONS_RUNTIME_RECOVERY_POLL_S = 0.25
PHOTONS_RUNTIME_RECOVERY_BARRIER_LOG_S = 30.0

# SYSTEM owns these platform/context objects.  PHOTONS copies them transitively
# and does not derive replacement values or reinterpret their scientific meaning.
SYSTEM_CONTEXT_FIELDS = (
    "pi",
    "network",
    "sensors",
    "environment",
    "location",
    "gnss",
    "power",
    "battery",
)


# ---------------------------------------------------------------------
# Live state
# ---------------------------------------------------------------------

_state_lock = threading.Lock()
_campaign_lock = threading.Lock()
_recovery_lock = threading.RLock()
# Serialize worker publication/persistence with destructive maintenance commands.
# PHOTONS is only 1 Hz, so this intentionally favors an exact maintenance boundary
# over parallelism that could let a pre-CLEAR campaign row reappear afterward.
_maintenance_lock = threading.RLock()

_fragment_queue: queue.Queue[Payload] = queue.Queue(maxsize=PHOTONS_INGRESS_QUEUE_MAXSIZE)
_persist_queue: queue.Queue[Dict[str, Any]] = queue.Queue(maxsize=PHOTONS_PERSIST_QUEUE_MAXSIZE)
_state_worker_started = threading.Event()
_persistence_worker_started = threading.Event()
_campaign_control_ready = threading.Event()
_recovery_proof_durable = threading.Event()

_latest_fragment: Optional[Payload] = None
_latest_photons: Optional[Payload] = None

# One active Pi lifecycle plus any STOP-requested lifecycle whose firmware-authored
# final fragment has not yet crossed the state worker.  Teensy owns the actual
# recording boundaries; these structures contribute only durable Pi provenance.
_active_campaign: Optional[Dict[str, Any]] = None
_closing_campaigns: List[Dict[str, Any]] = []

_fragments_received = 0
_fragments_queued = 0
_fragments_processed = 0
_photons_published = 0
_rows_persisted = 0
_fragments_rejected = 0
_ingress_queue_depth_max = 0
_persist_queue_depth_max = 0
_system_report_retry_count = 0
_persistence_retry_count = 0
_startup_infrastructure_wait_count = 0
_startup_infrastructure_wait_seconds_last = 0.0
_last_startup_infrastructure_wait: Optional[Dict[str, Any]] = None
_campaign_start_count = 0
_campaign_stop_count = 0
_baseline_set_count = 0
_stale_campaign_retire_count = 0
_stats_reset_requests = 0
_stats_reset_success = 0
_stats_reset_failures = 0
# Operator STATS_RESET is control-plane asynchronous.  A short worker owns only
# producer-side arming so the command RPC can return immediately.  Completion is
# witnessed by the normal ordered persistence path when it commits canonical row 1
# of the requested firmware statistics epoch.
_stats_reset_control_lock = threading.Lock()
_stats_reset_in_progress = threading.Event()
_stats_reset_pending: Optional[Dict[str, Any]] = None
_report_photons_requests = 0
_report_stats_requests = 0
_clear_count = 0
_delete_count = 0
_truncate_count = 0
_flash_cut_count = 0
_inject_problem_requests = 0
_inject_problem_failures = 0
_last_maintenance: Optional[Dict[str, Any]] = None
_last_campaign_transition: Optional[Dict[str, Any]] = None
_last_structural_rejection: Optional[Dict[str, Any]] = None
_last_system_report_failure: Optional[Dict[str, Any]] = None
_last_persistence_failure: Optional[Dict[str, Any]] = None
_recovery_proof_expected: Optional[Dict[str, Any]] = None
_recovery_proof_persisted: Optional[Dict[str, Any]] = None
_recovery_status: Dict[str, Any] = {
    "schema": "PHOTONS_PI_RECOVERY_V1",
    "state": "NOT_STARTED",
    "enabled": True,
}
_recovery_attempt_count = 0
_recovery_restore_count = 0
_recovery_partial_history_restore_count = 0
_recovery_live_adopt_count = 0
_recovery_cold_start_count = 0
_recovery_failure_count = 0

_runtime_recovery_hold = threading.Event()
_runtime_recovery_monitor_started = threading.Event()
_runtime_recovery_generation = 0
_runtime_recovery_count = 0
_runtime_recovery_surviving_reattach_count = 0
_runtime_recovery_failure_count = 0
_runtime_recovery_rows_retired = 0
_runtime_recovery_last: Optional[Dict[str, Any]] = None

_ppb_checkpoint_lock = threading.RLock()
_ppb_checkpoint_runtime: Optional[Dict[str, Any]] = None
_ppb_checkpoint_rows_verified = 0
_ppb_checkpoint_recoverable_rows = 0
_ppb_checkpoint_warming_rows = 0
_ppb_checkpoint_gap_count = 0
_ppb_checkpoint_ingest_failure_count = 0
_ppb_checkpoint_last_ingest_failure: Optional[Dict[str, Any]] = None
_ppb_checkpoint_reacquire_requested = threading.Event()
_ppb_checkpoint_reacquire_worker_started = threading.Event()
_ppb_checkpoint_reacquire_request_count = 0
_ppb_checkpoint_reacquire_success_count = 0
_ppb_checkpoint_reacquire_failure_count = 0
_ppb_checkpoint_reacquire_last_result: Optional[Dict[str, Any]] = None
_ppb_checkpoint_reacquire_last_failure: Optional[Dict[str, Any]] = None

OPERATIONAL_STATE_SCHEMA = "PI_SUBSYSTEM_OPERATIONAL_STATE_V1"
OPERATIONAL_STATE_STARTING = "STARTING"
OPERATIONAL_STATE_RECOVERING = "RECOVERING"
OPERATIONAL_STATE_RUNNING = "RUNNING"
OPERATIONAL_STATE_HARD_FAILURE = "HARD_FAILURE"

_operational_state_lock = threading.Lock()
_operational_state: Dict[str, Any] = {
    "schema": OPERATIONAL_STATE_SCHEMA,
    "subsystem": "PHOTONS",
    "state": OPERATIONAL_STATE_STARTING,
    "entered_at_utc": None,
    "reason": "process_initialization",
    "source": "RUN",
    "details": {},
}
_hard_failure_event = threading.Event()
_hard_failure_lock = threading.Lock()
_hard_failure_entries = 0
_hard_failure_ingress_dropped = 0
_hard_failure_state_dropped = 0
_hard_failure_persistence_dropped = 0
_last_hard_failure: Optional[Dict[str, Any]] = None

# One general operator repair lane.  It may use read-only live-producer
# reacquisition, an implicit STATS_RESET, or full Phase-5 recovery as needed.
_repair_lock = threading.Lock()
_repair_event = threading.Event()
_repair_requests = 0
_repair_success = 0
_repair_failures = 0
_last_repair: Optional[Dict[str, Any]] = None

_lap_baseline_ns: Optional[str] = None
_lap_baseline_fs: Optional[int] = None
# Deprecated whole-picosecond compatibility mirror retained for recovery/report
# interoperability with already-durable PHOTONS rows.
_standard_lap_ns: Optional[str] = None
_standard_lap_ps: Optional[int] = None
_teensy_standard_configured = False
_lap_baseline_set_count = 0


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

def _utc_now_z() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _operational_state_snapshot() -> Dict[str, Any]:
    with _operational_state_lock:
        return copy.deepcopy(_operational_state)


def _hard_failure_active() -> bool:
    return _hard_failure_event.is_set()


def _repair_active() -> bool:
    return _repair_event.is_set()


def _set_operational_state(
    state: str,
    *,
    reason: Optional[str] = None,
    source: Optional[str] = None,
    details: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Publish one process-local PHOTONS lifecycle state; HARD_FAILURE is latched."""
    global _operational_state

    normalized = str(state or "").strip().upper()
    if normalized not in {
        OPERATIONAL_STATE_STARTING,
        OPERATIONAL_STATE_RECOVERING,
        OPERATIONAL_STATE_RUNNING,
        OPERATIONAL_STATE_HARD_FAILURE,
    }:
        raise ValueError(f"unsupported PHOTONS operational state {state!r}")

    with _operational_state_lock:
        if (
            str(_operational_state.get("state") or "") == OPERATIONAL_STATE_HARD_FAILURE
            and normalized != OPERATIONAL_STATE_HARD_FAILURE
        ):
            return copy.deepcopy(_operational_state)
        _operational_state = {
            "schema": OPERATIONAL_STATE_SCHEMA,
            "subsystem": "PHOTONS",
            "state": normalized,
            "entered_at_utc": _utc_now_z(),
            "reason": str(reason or ""),
            "source": str(source or ""),
            "details": copy.deepcopy(details or {}),
        }
        return copy.deepcopy(_operational_state)


def _enter_hard_failure(
    reason: str,
    details: Dict[str, Any],
    *,
    source: str,
) -> Dict[str, Any]:
    """Latch PHOTONS inert without exiting so systemd cannot erase the failure."""
    global _hard_failure_entries
    global _last_hard_failure
    global _recovery_status

    with _hard_failure_lock:
        if _hard_failure_active():
            return _operational_state_snapshot()

        context = {
            "fragment_queue_depth": _fragment_queue.qsize(),
            "persist_queue_depth": _persist_queue.qsize(),
            "state_worker_started": _state_worker_started.is_set(),
            "persistence_worker_started": _persistence_worker_started.is_set(),
            "campaign_control_ready": _campaign_control_ready.is_set(),
            "latest_fragment_sequence": (
                _latest_fragment.get("sequence")
                if isinstance(_latest_fragment, dict)
                else None
            ),
            "latest_photons_sequence": (
                _latest_photons.get("sequence")
                if isinstance(_latest_photons, dict)
                else None
            ),
        }
        failure_details = {
            "failure": copy.deepcopy(details or {}),
            "context": context,
            "action": (
                "PHOTONS is latched in HARD_FAILURE. No new canonical/persistent testimony "
                "or campaign mutation will occur. Read-only reports remain available."
            ),
        }

        _hard_failure_event.set()
        _campaign_control_ready.clear()
        _hard_failure_entries += 1
        snapshot = _set_operational_state(
            OPERATIONAL_STATE_HARD_FAILURE,
            reason=reason,
            source=source,
            details=failure_details,
        )
        _last_hard_failure = copy.deepcopy(snapshot)
        with _recovery_lock:
            _recovery_status = {
                "schema": "PHOTONS_PI_RECOVERY_V1",
                "enabled": True,
                "state": OPERATIONAL_STATE_HARD_FAILURE,
                "updated_at_utc": _utc_now_z(),
                "error": str(reason),
                "hard_failure": copy.deepcopy(snapshot),
            }
        logging.critical(
            "🛑 [photons] HARD_FAILURE LATCHED: reason=%s source=%s; "
            "queues fragment=%d persistence=%d, latest fragment=%s latest canonical=%s. "
            "Optical persistence and campaign mutation are stopped; read-only reports remain available.",
            reason,
            source,
            int(context["fragment_queue_depth"]),
            int(context["persist_queue_depth"]),
            context.get("latest_fragment_sequence"),
            context.get("latest_photons_sequence"),
        )
        return snapshot


def _require_dict(value: Any, path: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be an object")
    return value


def _require_int(value: Any, path: str, *, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"{path} must be an integer >= {minimum}; got {value!r}")
    return int(value)


def _require_u32(value: Any, path: str) -> int:
    out = _require_int(value, path)
    if out > 0xFFFFFFFF:
        raise ValueError(f"{path} must fit uint32; got {value!r}")
    return out


def _require_bool(value: Any, path: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{path} must be boolean; got {value!r}")
    return bool(value)


def _require_float(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{path} must be numeric; got {value!r}")
    out = float(value)
    if not math.isfinite(out):
        raise ValueError(f"{path} must be finite; got {value!r}")
    return out


def _validate_recovery_welford(value: Any, path: str) -> Dict[str, Any]:
    obj = _require_dict(value, path)
    n = _require_int(obj.get("n"), f"{path}.n")
    mean = _require_float(obj.get("mean"), f"{path}.mean")
    m2 = _require_float(obj.get("m2"), f"{path}.m2")
    min_value = _require_float(obj.get("min"), f"{path}.min")
    max_value = _require_float(obj.get("max"), f"{path}.max")
    if n == 0:
        if any(value != 0.0 for value in (mean, m2, min_value, max_value)):
            raise ValueError(f"{path} empty Welford must publish zero values")
    elif m2 < 0.0 or min_value > max_value or not (min_value <= mean <= max_value):
        raise ValueError(f"{path} Welford geometry is invalid")
    return {
        "n": n,
        "mean": mean,
        "m2": m2,
        "min": min_value,
        "max": max_value,
    }


def _welford_equivalent(a: Dict[str, Any], b: Dict[str, Any]) -> bool:
    if int(a["n"]) != int(b["n"]):
        return False
    return all(
        abs(float(a[key]) - float(b[key])) <= PHOTONS_RECOVERY_VERIFY_TOLERANCE
        for key in ("mean", "m2", "min", "max")
    )


def _welford_grand_ratio_diagnostic(
    *,
    welford_mean_ns: float,
    ratio_mean_ns: float,
    lap_baseline_fs: int,
) -> Dict[str, Any]:
    """Quantify Welford-vs-N/T drift without granting it custody authority."""
    lap_baseline_ns = float(lap_baseline_fs) / 1_000_000.0
    if lap_baseline_ns <= 0.0:
        raise ValueError("PHOTONS Welford/grand-ratio diagnostic has nonpositive lap baseline")
    delta_ns = float(welford_mean_ns) - float(ratio_mean_ns)
    delta_ppb = delta_ns
    return {
        "schema": "PHOTONS_WELFORD_GRAND_RATIO_DIAGNOSTIC_V1",
        "welford_mean_ns": float(welford_mean_ns),
        "grand_ratio_mean_ns": float(ratio_mean_ns),
        "delta_ns": float(delta_ns),
        "abs_delta_ns": abs(float(delta_ns)),
        "delta_ppb": float(delta_ppb),
        "abs_delta_ppb": abs(float(delta_ppb)),
        "diagnostic_limit_ppb": float(PHOTONS_WELFORD_GRAND_RATIO_DIAGNOSTIC_PPB),
        "notable": abs(float(delta_ppb)) > PHOTONS_WELFORD_GRAND_RATIO_DIAGNOSTIC_PPB,
        "recovery_authority_effect": "NONE_DIAGNOSTIC_ONLY",
    }


def _normalize_lap_baseline_ns(raw: Any, *, path: str) -> Tuple[str, int]:
    try:
        value = Decimal(str(raw))
    except (InvalidOperation, ValueError) as exc:
        raise ValueError(f"{path} is not decimal: {raw!r}") from exc
    if not value.is_finite() or value <= 0:
        raise ValueError(f"{path} must be finite and > 0; got {raw!r}")
    quantum = Decimal("0.000001")
    normalized = value.quantize(quantum)
    if normalized != value:
        raise ValueError(f"{path} may have at most six decimal places; got {raw!r}")
    text = format(normalized, ".6f")
    fs = int(normalized * 1_000_000)
    if fs <= 0:
        raise ValueError(f"{path} resolved to nonpositive femtoseconds")
    return text, fs


def _compat_standard_from_baseline_fs(lap_baseline_fs: int) -> Tuple[str, int]:
    ps = int(lap_baseline_fs) // 1000 + (1 if int(lap_baseline_fs) % 1000 >= 500 else 0)
    if ps <= 0:
        raise ValueError("PHOTONS LAP_BASELINE_NS cannot be represented by compatibility ps mirror")
    return format(Decimal(ps) / Decimal(1000), ".3f"), ps


def _load_lap_baseline_ns() -> Tuple[str, int, str, int]:
    """Load config.PHOTONS LAP_BASELINE_NS, explicitly migrating the legacy standard once."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute("SELECT payload FROM config WHERE config_key = 'PHOTONS'")
        row = cur.fetchone()
        if row is None:
            raise RuntimeError("config.PHOTONS row is required")
        payload = row.get("payload")
        if isinstance(payload, str):
            payload = json.loads(payload)
        payload = _require_dict(payload, "config.PHOTONS.payload")
        raw = payload.get("lap_baseline_ns")
        migrated = False
        if raw is None:
            raw = payload.get("standard_lap_ns")
            if raw is None:
                raise RuntimeError("config.PHOTONS.lap_baseline_ns is required")
            migrated = True
        baseline_text, baseline_fs = _normalize_lap_baseline_ns(
            raw, path=("config.PHOTONS.standard_lap_ns" if migrated else "config.PHOTONS.lap_baseline_ns")
        )
        if migrated:
            cur.execute(
                """
                UPDATE config
                SET payload = payload || jsonb_build_object(
                    'lap_baseline_ns', %s::text,
                    'lap_baseline_fs', %s::bigint,
                    'lap_baseline_source', 'LEGACY_STANDARD_LAP_NS_MIGRATION',
                    'lap_baseline_updated_at_utc', %s::text
                )
                WHERE config_key = 'PHOTONS'
                """,
                (baseline_text, baseline_fs, _utc_now_z()),
            )
            if cur.rowcount != 1:
                raise RuntimeError("config.PHOTONS legacy baseline migration did not update exactly one row")
    if migrated:
        logging.warning(
            "📐 [photons] migrated legacy config.PHOTONS.standard_lap_ns=%s to LAP_BASELINE_NS=%s",
            raw, baseline_text,
        )
    standard_text, standard_ps = _compat_standard_from_baseline_fs(baseline_fs)
    return baseline_text, baseline_fs, standard_text, standard_ps


def _configure_teensy_lap_baseline() -> None:
    """Install the exact config.PHOTONS operator reference before recovery/publication."""
    global _lap_baseline_ns
    global _lap_baseline_fs
    global _standard_lap_ns
    global _standard_lap_ps
    global _teensy_standard_configured

    baseline_text, baseline_fs, standard_text, standard_ps = _load_lap_baseline_ns()
    response = send_command(
        machine="TEENSY",
        subsystem=SUBSYSTEM,
        command="SET_LAP_BASELINE_NS",
        args={"lap_baseline_ns": baseline_text},
    )
    payload = response.get("payload") if isinstance(response, dict) else None
    if not isinstance(response, dict) or not response.get("success") or not isinstance(payload, dict):
        raise RuntimeError(f"Teensy PHOTONS.SET_LAP_BASELINE_NS failed: {response!r}")
    if payload.get("lap_baseline_configured") is not True:
        raise RuntimeError(f"Teensy did not confirm LAP_BASELINE_NS installation: {payload!r}")
    echoed_fs = _require_int(
        payload.get("lap_baseline_fs"), "PHOTONS.SET_LAP_BASELINE_NS.payload.lap_baseline_fs", minimum=1
    )
    if echoed_fs != baseline_fs:
        raise RuntimeError(
            f"Teensy LAP_BASELINE_NS echo mismatch: configured_fs={baseline_fs} echoed_fs={echoed_fs}"
        )
    _lap_baseline_ns = baseline_text
    _lap_baseline_fs = baseline_fs
    _standard_lap_ns = standard_text
    _standard_lap_ps = standard_ps
    _teensy_standard_configured = True
    logging.info(
        "✅ [photons] installed LAP_BASELINE_NS=%s ns (%d fs) on Teensy",
        baseline_text, baseline_fs,
    )


def _request_teensy_campaign_command(
    command: str,
    *,
    campaign: Optional[str] = None,
    accepted_statuses: set[str],
) -> Dict[str, Any]:
    """Send one PHOTONS lifecycle command and require an explicit firmware verdict."""
    args = {"campaign": campaign} if campaign is not None else None
    if args is None:
        response = send_command(
            machine="TEENSY",
            subsystem=SUBSYSTEM,
            command=command,
        )
    else:
        response = send_command(
            machine="TEENSY",
            subsystem=SUBSYSTEM,
            command=command,
            args=args,
        )
    payload = response.get("payload") if isinstance(response, dict) else None
    status = str(payload.get("status") or "") if isinstance(payload, dict) else ""
    if (
        not isinstance(response, dict)
        or not response.get("success")
        or not isinstance(payload, dict)
        or status not in accepted_statuses
    ):
        raise RuntimeError(
            f"Teensy PHOTONS.{command} rejected: status={status!r} response={response!r}"
        )
    return response


def _validate_firmware_campaign(
    fragment: Payload,
    sequence: int,
) -> Optional[Dict[str, Any]]:
    """Validate optional Teensy-authored LANTERN_FRAGMENT_V1 testimony."""
    raw = fragment.get("campaign")
    if raw is None:
        return None
    campaign = _require_dict(raw, "PHOTONS_FRAGMENT.campaign")
    if campaign.get("schema") != LANTERN_FRAGMENT_SCHEMA:
        raise ValueError(
            f"unsupported PHOTONS campaign schema {campaign.get('schema')!r}"
        )

    name = str(campaign.get("campaign") or "").strip()
    if not name:
        raise ValueError("PHOTONS_FRAGMENT.campaign.campaign is required")

    start_after = _require_int(
        campaign.get("start_after_sequence"),
        "PHOTONS_FRAGMENT.campaign.start_after_sequence",
        minimum=1,
    )
    if start_after >= sequence:
        raise ValueError(
            "PHOTONS campaign boundary is not strictly before its public row: "
            f"start_after_sequence={start_after} sequence={sequence}"
        )

    public_count = _require_int(
        campaign.get("public_count"),
        "PHOTONS_FRAGMENT.campaign.public_count",
        minimum=1,
    )
    expected_public_count = sequence - start_after
    if public_count != expected_public_count:
        raise ValueError(
            "PHOTONS campaign public-count/boundary arithmetic mismatch: "
            f"public_count={public_count} expected={expected_public_count} "
            f"sequence={sequence} start_after_sequence={start_after}"
        )
    final = campaign.get("final")
    if not isinstance(final, bool):
        raise ValueError("PHOTONS_FRAGMENT.campaign.final must be boolean")

    stop_after = campaign.get("stop_after_sequence")
    if final:
        stop_after = _require_int(
            stop_after,
            "PHOTONS_FRAGMENT.campaign.stop_after_sequence",
            minimum=1,
        )
        if stop_after != sequence:
            raise ValueError(
                "final PHOTONS campaign row must own its STOP boundary: "
                f"stop_after_sequence={stop_after} sequence={sequence}"
            )
    elif stop_after is not None:
        raise ValueError(
            "non-final PHOTONS campaign row must not publish stop_after_sequence"
        )

    stats = _require_dict(campaign.get("stats"), "PHOTONS_FRAGMENT.campaign.stats")
    lap_count = _require_int(
        stats.get("lap_count"),
        "PHOTONS_FRAGMENT.campaign.stats.lap_count",
    )
    total_ns = _require_int(
        stats.get("total_lap_gnss_ns"),
        "PHOTONS_FRAGMENT.campaign.stats.total_lap_gnss_ns",
    )
    instrument = _require_dict(fragment.get("photons"), "PHOTONS_FRAGMENT.photons")
    instrument_stats = _require_dict(
        instrument.get("stats"), "PHOTONS_FRAGMENT.photons.stats"
    )
    custody_lap_count = _require_int(
        instrument_stats.get("custody_lap_count"),
        "PHOTONS_FRAGMENT.photons.stats.custody_lap_count",
    )
    custody_total_ns = _require_int(
        instrument_stats.get("custody_total_lap_gnss_ns"),
        "PHOTONS_FRAGMENT.photons.stats.custody_total_lap_gnss_ns",
    )
    if lap_count > custody_lap_count or total_ns > custody_total_ns:
        raise ValueError(
            "PHOTONS campaign population exceeds monotonic instrument custody: "
            f"campaign_n={lap_count} custody_n={custody_lap_count} "
            f"campaign_total={total_ns} custody_total={custody_total_ns}"
        )

    if lap_count == 0:
        if total_ns != 0:
            raise ValueError("zero-lap PHOTONS campaign has nonzero total_lap_gnss_ns")
        if stats.get("sample_count") is not None or stats.get("ppb") is not None:
            raise ValueError("zero-lap PHOTONS campaign must not publish PPB testimony")
    else:
        if total_ns == 0:
            raise ValueError("nonempty PHOTONS campaign has zero total_lap_gnss_ns")
        sample_count = _require_int(
            stats.get("sample_count"),
            "PHOTONS_FRAGMENT.campaign.stats.sample_count",
            minimum=1,
        )
        if sample_count != lap_count:
            raise ValueError(
                "PHOTONS campaign sample-count mismatch: "
                f"lap_count={lap_count} sample_count={sample_count}"
            )
        if stats.get("mean_lap_ns") is None or stats.get("ppb") is None:
            raise ValueError("nonempty PHOTONS campaign is missing mean_lap_ns/ppb")

    return copy.deepcopy(campaign)


def _fetch_system_report() -> Dict[str, Any]:
    """Pull the authoritative current platform context from Pi SYSTEM."""
    response = send_command(
        machine="PI",
        subsystem="SYSTEM",
        command="REPORT",
        retries=1,
        retry_delay_s=0.0,
    )
    payload = response.get("payload") if isinstance(response, dict) else None
    if not isinstance(response, dict) or not response.get("success") or not isinstance(payload, dict):
        raise RuntimeError(f"SYSTEM.REPORT unavailable: {response!r}")
    return copy.deepcopy(payload)


def _read_startup_system_feature(name: str) -> Dict[str, Any]:
    """Read one SYSTEM readiness leaf without hiding malformed successful testimony."""
    try:
        response = send_command(
            machine="PI",
            subsystem="SYSTEM",
            command="GET_FEATURE",
            args={"name": name},
            retries=1,
            retry_delay_s=0.0,
        )
    except Exception as exc:
        return {
            "name": name,
            "known": False,
            "status": "UNAVAILABLE",
            "detail": f"SYSTEM.GET_FEATURE unavailable: {exc}",
        }

    if not isinstance(response, dict):
        raise RuntimeError(
            f"SYSTEM.GET_FEATURE {name} returned malformed response: {response!r}"
        )
    if not response.get("success"):
        return {
            "name": name,
            "known": False,
            "status": "UNAVAILABLE",
            "detail": str(response.get("message") or "SYSTEM.GET_FEATURE unavailable"),
        }

    payload = response.get("payload")
    if not isinstance(payload, dict):
        raise RuntimeError(
            f"SYSTEM.GET_FEATURE {name} returned no payload: {response!r}"
        )
    observed_name = str(payload.get("name") or "").strip().upper()
    if observed_name != name:
        raise RuntimeError(
            f"SYSTEM.GET_FEATURE identity mismatch: requested={name} observed={observed_name!r}"
        )
    known = payload.get("known")
    if not isinstance(known, bool):
        raise RuntimeError(
            f"SYSTEM.GET_FEATURE {name} payload.known must be boolean: {payload!r}"
        )
    status = str(payload.get("status") or "").strip().upper()
    if not status:
        raise RuntimeError(
            f"SYSTEM.GET_FEATURE {name} omitted status: {payload!r}"
        )

    result: Dict[str, Any] = {
        "name": name,
        "known": known,
        "status": status,
    }
    for key in ("age_s", "ttl_s", "expired", "generation"):
        if key in payload:
            result[key] = copy.deepcopy(payload.get(key))

    if name == "PI.PUBSUB.TEENSY_RPC" and known and status == "NOMINAL":
        if payload.get("expired") is not False:
            raise RuntimeError(
                "PI.PUBSUB.TEENSY_RPC is NOMINAL without an unexpired lease: "
                f"{payload!r}"
            )
        generation = payload.get("generation")
        if isinstance(generation, bool) or not isinstance(generation, int) or generation <= 0:
            raise RuntimeError(
                "PI.PUBSUB.TEENSY_RPC is NOMINAL without a positive transport generation: "
                f"{payload!r}"
            )

    return result


def _wait_for_startup_infrastructure() -> Dict[str, Any]:
    """Wait for PostgreSQL and semantic Teensy RPC before Phase-5 recovery."""
    global _startup_infrastructure_wait_count
    global _startup_infrastructure_wait_seconds_last
    global _last_startup_infrastructure_wait

    started = time.monotonic()
    last_log_at = started
    last_signature: Optional[Tuple[Tuple[str, str, bool], ...]] = None
    logged_wait = False
    checks = 0
    with _state_lock:
        _startup_infrastructure_wait_count += 1

    while True:
        checks += 1
        observed = [
            _read_startup_system_feature(name)
            for name in STARTUP_INFRASTRUCTURE_REQUIRED
        ]
        blockers = [
            item for item in observed
            if not item.get("known") or item.get("status") != "NOMINAL"
        ]
        now = time.monotonic()
        waited = now - started
        snapshot = {
            "ts_utc": _utc_now_z(),
            "status": "NOMINAL" if not blockers else "WAITING",
            "checks": int(checks),
            "waited_s": round(float(waited), 3),
            "required": copy.deepcopy(observed),
            "blockers": copy.deepcopy(blockers),
        }
        with _state_lock:
            _last_startup_infrastructure_wait = copy.deepcopy(snapshot)

        if not blockers:
            with _state_lock:
                _startup_infrastructure_wait_seconds_last = float(waited)
            if logged_wait:
                logging.info(
                    "✅ [photons/startup] infrastructure admitted after %.1fs: "
                    "PI.SYSTEM.POSTGRES=NOMINAL PI.PUBSUB.TEENSY_RPC=NOMINAL",
                    waited,
                )
            return copy.deepcopy(snapshot)

        signature = tuple(
            (
                str(item.get("name") or "?"),
                str(item.get("status") or "UNAVAILABLE"),
                bool(item.get("known")),
            )
            for item in blockers
        )
        should_log = bool(
            waited >= STARTUP_INFRASTRUCTURE_QUIET_GRACE_S
            and (
                not logged_wait
                or signature != last_signature
                or now - last_log_at >= STARTUP_INFRASTRUCTURE_STATUS_LOG_INTERVAL_S
            )
        )
        if should_log:
            detail = ", ".join(
                f"{item.get('name')}={item.get('status')}"
                + ("" if item.get("known") else "(unknown)")
                for item in blockers
            )
            logging.info(
                "⏳ [photons/startup] waiting for application infrastructure "
                "(%.1fs): %s",
                waited,
                detail,
            )
            logged_wait = True
            last_log_at = now
            last_signature = signature

        time.sleep(STARTUP_INFRASTRUCTURE_POLL_S)


def _validate_photons_fragment(fragment: Payload) -> Tuple[int, int, Optional[int], Dict[str, Any]]:
    """Prove structural/accounting coherence without re-authoring optical science."""
    fragment = _require_dict(fragment, "PHOTONS_FRAGMENT")
    if fragment.get("schema") != PHOTONS_FRAGMENT_SCHEMA:
        raise ValueError(
            f"unsupported PHOTONS_FRAGMENT schema {fragment.get('schema')!r}"
        )

    sequence = _require_int(fragment.get("sequence"), "PHOTONS_FRAGMENT.sequence", minimum=1)
    publish_count = _require_int(
        fragment.get("publish_count"), "PHOTONS_FRAGMENT.publish_count", minimum=0
    )

    instrument = _require_dict(fragment.get("photons"), "PHOTONS_FRAGMENT.photons")
    if instrument.get("schema") != PHOTONS_INSTRUMENT_SCHEMA:
        raise ValueError(
            f"unsupported PHOTONS instrument schema {instrument.get('schema')!r}"
        )

    science = _require_dict(instrument.get("science"), "photons.science")
    stats = _require_dict(instrument.get("stats"), "photons.stats")
    projection = _require_dict(instrument.get("projection"), "photons.projection")
    raw_cycles = _require_dict(instrument.get("raw_cycles"), "photons.raw_cycles")
    race = _require_dict(instrument.get("race"), "photons.race")
    interrupt = _require_dict(instrument.get("interrupt"), "photons.interrupt")
    accepted = _require_dict(science.get("accepted"), "photons.science.accepted")
    excluded = _require_dict(science.get("excluded"), "photons.science.excluded")
    reasons = _require_dict(
        science.get("exclusion_reasons"), "photons.science.exclusion_reasons"
    )
    accepted_raw = _require_dict(
        accepted.get("raw_cycles"), "photons.science.accepted.raw_cycles"
    )
    accepted_projected = _require_dict(
        accepted.get("projected_lap_ns"),
        "photons.science.accepted.projected_lap_ns",
    )
    excluded_raw = _require_dict(
        excluded.get("raw_cycles"), "photons.science.excluded.raw_cycles"
    )
    excluded_projected = _require_dict(
        excluded.get("projected_lap_ns"),
        "photons.science.excluded.projected_lap_ns",
    )
    stats_lap_time = _require_dict(stats.get("lap_time"), "photons.stats.lap_time")
    recovery = _require_dict(instrument.get("recovery"), "photons.recovery")

    if science.get("schema") != PHOTONS_SCIENCE_SCHEMA:
        raise ValueError(f"unsupported PHOTONS science schema {science.get('schema')!r}")
    if stats.get("schema") != PHOTONS_STATS_SCHEMA:
        raise ValueError(f"unsupported PHOTONS stats schema {stats.get('schema')!r}")
    if stats.get("ppb_semantics") != PHOTONS_PPB_SEMANTICS:
        raise ValueError(
            f"unsupported PHOTONS PPB semantics {stats.get('ppb_semantics')!r}"
        )

    # Real-race transport court.  The recurring 1 kHz source is physical testimony,
    # not a second science authority: completed races feed the existing raw/projection/
    # Welford court below.  Prove only the producer-authored geometry/accounting here.
    if instrument.get("source") != "PD200T_REAL_RACE":
        raise ValueError(
            f"unsupported PHOTONS physical source {instrument.get('source')!r}"
        )
    if race.get("schema") != PHOTONS_RACE_SCHEMA:
        raise ValueError(f"unsupported PHOTONS race schema {race.get('schema')!r}")

    cadence_hz = _require_int(race.get("cadence_hz"), "photons.race.cadence_hz", minimum=1)
    cadence_ns = _require_int(race.get("cadence_ns"), "photons.race.cadence_ns", minimum=1)
    pulse_ns = _require_int(race.get("pulse_ns"), "photons.race.pulse_ns", minimum=1)
    if (
        cadence_hz != PHOTONS_RACE_CADENCE_HZ
        or cadence_ns != PHOTONS_RACE_CADENCE_NS
        or pulse_ns != PHOTONS_RACE_PULSE_NS
    ):
        raise ValueError(
            "PHOTONS race geometry changed unexpectedly: "
            f"cadence={cadence_hz}Hz/{cadence_ns}ns pulse={pulse_ns}ns"
        )
    if race.get("launch_surrogate") != "LD_ON_FALLING_EDGE":
        raise ValueError(
            f"unsupported PHOTONS launch surrogate {race.get('launch_surrogate')!r}"
        )
    if race.get("flight_interpretation") != "ESTIMATED":
        raise ValueError(
            f"unsupported PHOTONS flight interpretation {race.get('flight_interpretation')!r}"
        )

    # process_interrupt counters are lifetime forensic testimony. Firmware
    # snapshots injury-counter origins when fresh physical ancestry begins and
    # publishes the unsigned uint32 deltas used by the rolling-custody court.
    # Prove that arithmetic here rather than trusting a derived validity flag.
    if not _require_bool(
        interrupt.get("ancestry_baseline_valid"),
        "photons.interrupt.ancestry_baseline_valid",
    ):
        raise ValueError("PHOTONS interrupt ancestry baseline is not established")
    callback_missing_lifetime = _require_u32(
        interrupt.get("callback_missing_count"),
        "photons.interrupt.callback_missing_count",
    )
    callback_missing_origin = _require_u32(
        interrupt.get("callback_missing_count_origin"),
        "photons.interrupt.callback_missing_count_origin",
    )
    callback_missing_since = _require_u32(
        interrupt.get("callback_missing_count_since_ancestry"),
        "photons.interrupt.callback_missing_count_since_ancestry",
    )
    inactive_lifetime = _require_u32(
        interrupt.get("inactive_edge_count"),
        "photons.interrupt.inactive_edge_count",
    )
    inactive_origin = _require_u32(
        interrupt.get("inactive_edge_count_origin"),
        "photons.interrupt.inactive_edge_count_origin",
    )
    inactive_since = _require_u32(
        interrupt.get("inactive_edge_count_since_ancestry"),
        "photons.interrupt.inactive_edge_count_since_ancestry",
    )
    if ((callback_missing_lifetime - callback_missing_origin) & 0xFFFFFFFF) != callback_missing_since:
        raise ValueError("PHOTONS callback-missing ancestry delta does not close")
    if ((inactive_lifetime - inactive_origin) & 0xFFFFFFFF) != inactive_since:
        raise ValueError("PHOTONS inactive-edge ancestry delta does not close")
    endpoint_admitted = _require_bool(
        stats.get("rolling_ppb_endpoint_admitted"),
        "photons.stats.rolling_ppb_endpoint_admitted",
    )
    if endpoint_admitted and (callback_missing_since != 0 or inactive_since != 0):
        raise ValueError(
            "PHOTONS rolling endpoint admitted despite post-ancestry interrupt injury"
        )

    race_cadence_ticks_total = _require_int(
        race.get("cadence_tick_count_total"), "photons.race.cadence_tick_count_total"
    )
    race_attempts_total = _require_int(
        race.get("attempt_count_total"), "photons.race.attempt_count_total"
    )
    race_completed_total = _require_int(
        race.get("completed_count_total"), "photons.race.completed_count_total"
    )
    race_missed_total = _require_int(
        race.get("missed_count_total"), "photons.race.missed_count_total"
    )
    race_invalid_total = _require_int(
        race.get("invalid_endpoint_total"), "photons.race.invalid_endpoint_total"
    )
    race_enqueue_failure_total = _require_int(
        race.get("enqueue_failure_total"), "photons.race.enqueue_failure_total"
    )
    race_skipped_not_quiet_total = _require_int(
        race.get("skipped_not_quiet_total"), "photons.race.skipped_not_quiet_total"
    )
    race_skipped_projection_total = _require_int(
        race.get("skipped_projection_total"), "photons.race.skipped_projection_total"
    )
    if race_cadence_ticks_total != (
        race_attempts_total
        + race_skipped_not_quiet_total
        + race_skipped_projection_total
    ):
        raise ValueError(
            "PHOTONS race cadence accounting does not close: "
            f"ticks={race_cadence_ticks_total} attempts={race_attempts_total} "
            f"not_quiet={race_skipped_not_quiet_total} "
            f"projection={race_skipped_projection_total}"
        )

    finalized_total = (
        race_completed_total
        + race_missed_total
        + race_invalid_total
        + race_enqueue_failure_total
    )
    if finalized_total > race_attempts_total or race_attempts_total - finalized_total > 1:
        raise ValueError(
            "PHOTONS race lifetime accounting does not close: "
            f"attempts={race_attempts_total} completed={race_completed_total} "
            f"missed={race_missed_total} invalid={race_invalid_total} "
            f"enqueue_failure={race_enqueue_failure_total}"
        )

    race_cadence_ticks_fragment = _require_int(
        race.get("cadence_ticks_this_fragment"), "photons.race.cadence_ticks_this_fragment"
    )
    race_attempts_fragment = _require_int(
        race.get("attempts_this_fragment"), "photons.race.attempts_this_fragment"
    )
    race_completed_fragment = _require_int(
        race.get("completed_this_fragment"), "photons.race.completed_this_fragment"
    )
    _require_int(race.get("missed_this_fragment"), "photons.race.missed_this_fragment")
    race_skipped_not_quiet_fragment = _require_int(
        race.get("skipped_not_quiet_this_fragment"),
        "photons.race.skipped_not_quiet_this_fragment",
    )
    race_skipped_projection_fragment = _require_int(
        race.get("skipped_projection_this_fragment"),
        "photons.race.skipped_projection_this_fragment",
    )
    if race_cadence_ticks_fragment != (
        race_attempts_fragment
        + race_skipped_not_quiet_fragment
        + race_skipped_projection_fragment
    ):
        raise ValueError(
            "PHOTONS race fragment cadence accounting does not close: "
            f"ticks={race_cadence_ticks_fragment} attempts={race_attempts_fragment} "
            f"not_quiet={race_skipped_not_quiet_fragment} "
            f"projection={race_skipped_projection_fragment}"
        )
    _require_int(
        race.get("invalid_endpoint_this_fragment"),
        "photons.race.invalid_endpoint_this_fragment",
    )
    _require_int(
        race.get("enqueue_failure_this_fragment"),
        "photons.race.enqueue_failure_this_fragment",
    )
    if race_completed_fragment > race_attempts_fragment + 1:
        # A race launched in the prior one-second fragment may lawfully complete
        # in this one, so at most one completion can lead this fragment's attempts.
        raise ValueError(
            "PHOTONS race fragment completion/attempt geometry is impossible: "
            f"attempts={race_attempts_fragment} completed={race_completed_fragment}"
        )

    race_flight = _validate_recovery_welford(
        race.get("flight_ns"), "photons.race.flight_ns"
    )
    raw_laps_fragment = _require_int(
        raw_cycles.get("laps_this_fragment"), "photons.raw_cycles.laps_this_fragment"
    )
    projected_laps_fragment = _require_int(
        instrument.get("projected_laps_this_fragment"),
        "photons.projected_laps_this_fragment",
    )
    if race_completed_fragment != raw_laps_fragment:
        raise ValueError(
            "PHOTONS completed-race/raw-ring mismatch: "
            f"completed={race_completed_fragment} raw_laps={raw_laps_fragment}"
        )
    if int(race_flight["n"]) != projected_laps_fragment:
        raise ValueError(
            "PHOTONS race-flight/projection mismatch: "
            f"race_flight_n={race_flight['n']} projected={projected_laps_fragment}"
        )
    if projected_laps_fragment > race_completed_fragment:
        raise ValueError(
            "PHOTONS projected race count exceeds physical completions: "
            f"projected={projected_laps_fragment} completed={race_completed_fragment}"
        )

    candidate_count = _require_int(
        science.get("candidate_count"), "photons.science.candidate_count"
    )
    accepted_count = _require_int(accepted.get("count"), "photons.science.accepted.count")
    excluded_count = _require_int(excluded.get("count"), "photons.science.excluded.count")
    seed_pending = science.get("seed_pending")
    if not isinstance(seed_pending, bool):
        raise ValueError("photons.science.seed_pending must be boolean")
    pending_count = 1 if seed_pending else 0

    if candidate_count != accepted_count + excluded_count + pending_count:
        raise ValueError(
            "PHOTONS science accounting mismatch: "
            f"candidate_count={candidate_count} accepted={accepted_count} "
            f"excluded={excluded_count} pending={pending_count}"
        )

    accepted_raw_n = _require_int(
        accepted_raw.get("n"), "photons.science.accepted.raw_cycles.n"
    )
    accepted_projected_n = _require_int(
        accepted_projected.get("n"), "photons.science.accepted.projected_lap_ns.n"
    )
    stats_reset_count = _require_int(
        stats.get("reset_count"), "photons.stats.reset_count"
    )
    stats_update_count = _require_int(
        stats.get("update_count"), "photons.stats.update_count", minimum=1
    )
    custody_lap_count = _require_int(
        stats.get("custody_lap_count"), "photons.stats.custody_lap_count"
    )
    custody_total_ns = _require_int(
        stats.get("custody_total_lap_gnss_ns"),
        "photons.stats.custody_total_lap_gnss_ns",
    )
    stats_lap_count = _require_int(stats.get("lap_count"), "photons.stats.lap_count")
    stats_total_ns = _require_int(
        stats.get("total_lap_gnss_ns"), "photons.stats.total_lap_gnss_ns"
    )
    stats_lap_n = _require_int(stats_lap_time.get("n"), "photons.stats.lap_time.n")
    if stats_lap_count > custody_lap_count or stats_total_ns > custody_total_ns:
        raise ValueError(
            "PHOTONS resettable statistics exceed monotonic custody: "
            f"reset_count={stats_reset_count} update_count={stats_update_count} "
            f"stats_n={stats_lap_count} custody_n={custody_lap_count} "
            f"stats_total={stats_total_ns} custody_total={custody_total_ns}"
        )
    accepted_witnesses = {
        accepted_count, accepted_raw_n, accepted_projected_n, stats_lap_count, stats_lap_n
    }
    if len(accepted_witnesses) != 1:
        raise ValueError(
            "PHOTONS accepted-population mismatch: "
            f"count={accepted_count} raw_n={accepted_raw_n} projected_n={accepted_projected_n} "
            f"stats_lap_count={stats_lap_count} stats_lap_n={stats_lap_n}"
        )

    excluded_raw_n = _require_int(
        excluded_raw.get("n"), "photons.science.excluded.raw_cycles.n"
    )
    excluded_projected_n = _require_int(
        excluded_projected.get("n"), "photons.science.excluded.projected_lap_ns.n"
    )
    if excluded_raw_n != excluded_count:
        raise ValueError(
            "PHOTONS excluded raw-population mismatch: "
            f"count={excluded_count} raw_n={excluded_raw_n}"
        )
    if excluded_projected_n > excluded_count:
        raise ValueError(
            "PHOTONS excluded projected population exceeds excluded count: "
            f"projected_n={excluded_projected_n} excluded={excluded_count}"
        )

    lifetime_reason_counts: Dict[str, int] = {}
    for name, value in reasons.items():
        if name.endswith("_this_fragment"):
            continue
        lifetime_reason_counts[name] = _require_int(
            value, f"photons.science.exclusion_reasons.{name}"
        )
    if not lifetime_reason_counts:
        raise ValueError("photons.science.exclusion_reasons has no lifetime reason counters")
    reason_total = sum(lifetime_reason_counts.values())
    if excluded_count != reason_total:
        raise ValueError(
            "PHOTONS exclusion-reason accounting mismatch: "
            f"excluded={excluded_count} reason_total={reason_total} "
            f"reasons={lifetime_reason_counts}"
        )

    projection_invalid = lifetime_reason_counts.get("projection_invalid")
    if projection_invalid is None:
        raise ValueError(
            "photons.science.exclusion_reasons missing projection_invalid counter"
        )

    attempts = _require_int(projection.get("attempt_count"), "photons.projection.attempt_count")
    successes = _require_int(projection.get("success_count"), "photons.projection.success_count")
    rejects = _require_int(projection.get("reject_count"), "photons.projection.reject_count")
    if attempts != successes + rejects:
        raise ValueError(
            "PHOTONS projection accounting mismatch: "
            f"attempts={attempts} success={successes} reject={rejects}"
        )
    if attempts != candidate_count:
        raise ValueError(
            "PHOTONS candidate/projection mismatch: "
            f"candidate_count={candidate_count} projection_attempts={attempts}"
        )
    if rejects != projection_invalid:
        raise ValueError(
            "PHOTONS projection-reject reason mismatch: "
            f"projection.reject_count={rejects} projection_invalid={projection_invalid}"
        )
    if successes != accepted_count + excluded_projected_n + pending_count:
        raise ValueError(
            "PHOTONS projected-population mismatch: "
            f"success={successes} accepted={accepted_count} "
            f"excluded_projected={excluded_projected_n} pending={pending_count}"
        )

    recovery_restored = _require_bool(
        recovery.get("restored"), "photons.recovery.restored"
    )
    recovery_pending = _require_bool(
        recovery.get("proof_pending"), "photons.recovery.proof_pending"
    )
    recovery_advanced = _require_bool(
        recovery.get("proof_advanced"), "photons.recovery.proof_advanced"
    )
    recovery_committed = _require_bool(
        recovery.get("proof_committed"), "photons.recovery.proof_committed"
    )
    fresh_ancestry = _require_bool(
        recovery.get("fresh_physical_ancestry"),
        "photons.recovery.fresh_physical_ancestry",
    )
    if not fresh_ancestry:
        raise ValueError("published PHOTONS fragment lacks fresh physical ancestry")
    for field in (
        "raw_lap_ring_restored",
        "partial_lap_restored",
        "pending_seed_restored",
        "predictor_restored",
        "in_flight_train_restored",
    ):
        if _require_bool(recovery.get(field), f"photons.recovery.{field}"):
            raise ValueError(f"PHOTONS recovery illegally restored {field}")
    if recovery_pending and recovery_committed:
        raise ValueError("PHOTONS recovery proof cannot be pending and committed")
    if recovery_advanced and not recovery_restored:
        raise ValueError("cold PHOTONS lineage cannot claim restored proof advance")

    if recovery_restored:
        generation = _require_int(
            recovery.get("generation"), "photons.recovery.generation", minimum=1
        )
        source_sequence = _require_int(
            recovery.get("source_sequence"),
            "photons.recovery.source_sequence",
            minimum=1,
        )
        source_reset = _require_int(
            recovery.get("source_reset_count"),
            "photons.recovery.source_reset_count",
        )
        source_update = _require_int(
            recovery.get("source_update_count"),
            "photons.recovery.source_update_count",
            minimum=1,
        )
        source_laps = _require_int(
            recovery.get("source_lap_count"),
            "photons.recovery.source_lap_count",
            minimum=1,
        )
        source_total = _require_int(
            recovery.get("source_total_lap_gnss_ns"),
            "photons.recovery.source_total_lap_gnss_ns",
            minimum=1,
        )
        source_custody_laps = _require_int(
            recovery.get("source_custody_lap_count"),
            "photons.recovery.source_custody_lap_count",
            minimum=source_laps,
        )
        source_custody_total = _require_int(
            recovery.get("source_custody_total_lap_gnss_ns"),
            "photons.recovery.source_custody_total_lap_gnss_ns",
            minimum=source_total,
        )
        accepted_delta = _require_int(
            recovery.get("accepted_lap_delta"),
            "photons.recovery.accepted_lap_delta",
        )
        custody_delta = _require_int(
            recovery.get("custody_lap_delta"),
            "photons.recovery.custody_lap_delta",
        )
        if sequence <= source_sequence or stats_reset_count < source_reset:
            raise ValueError(
                "post-restore PHOTONS chronology regressed behind source: "
                f"generation={generation} sequence={sequence}/{source_sequence} "
                f"reset={stats_reset_count}/{source_reset}"
            )

        if (
            custody_lap_count < source_custody_laps
            or custody_total_ns < source_custody_total
            or custody_delta != custody_lap_count - source_custody_laps
        ):
            raise ValueError("PHOTONS recovery custody testimony does not close")

        if stats_reset_count == source_reset:
            if stats_update_count <= source_update:
                raise ValueError(
                    "post-restore PHOTONS statistical chronology did not advance "
                    "within recovered epoch: "
                    f"generation={generation} reset={stats_reset_count} "
                    f"update={stats_update_count}/{source_update}"
                )
            if (
                stats_lap_count < source_laps
                or stats_total_ns < source_total
                or accepted_delta != stats_lap_count - source_laps
                or accepted_delta != custody_delta
            ):
                raise ValueError(
                    "PHOTONS same-epoch recovery source/delta testimony does not close"
                )
        else:
            # STATS_RESET starts a new resettable statistical epoch.  The old
            # recovery-source stats/update coordinates remain provenance only;
            # monotonic accepted-lap custody is the cross-epoch recovery witness.
            if accepted_delta != custody_delta:
                raise ValueError(
                    "PHOTONS cross-epoch recovery accepted/custody delta mismatch"
                )

        # proof_advanced is recovery-chronology testimony, not a claim that this
        # particular one-second row happened to accept a lap.  A lawful N+1 row
        # may contain zero accepted laps.  Once the proof is committed it remains
        # true across later STATS_RESET epochs as durable recovery provenance.
        expected_advanced = bool(
            recovery_committed
            or (
                stats_reset_count == source_reset
                and sequence > source_sequence
                and stats_update_count > source_update
            )
        )
        if recovery_advanced != expected_advanced:
            raise ValueError("PHOTONS recovery proof_advanced chronology verdict is inconsistent")

    pps_count_raw = projection.get("last_pps_sequence")
    pps_count = _require_int(
        pps_count_raw, "photons.projection.last_pps_sequence", minimum=0
    )
    return sequence, publish_count, (pps_count if pps_count > 0 else None), instrument


# ---------------------------------------------------------------------
# Literal Pi Better-Buckets recovery checkpoint
# ---------------------------------------------------------------------

def _ppb_zero_endpoint() -> Dict[str, Any]:
    return {
        "sequence": 0,
        "lap_count": 0,
        "total_lap_gnss_ns": 0,
    }


def _ppb_minute_key(sequence: int) -> int:
    return ((int(sequence) - 1) // 60) + 1 if sequence > 0 else 0


def _ppb_endpoint_from_payload(value: Any, *, path: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be an endpoint object")
    endpoint = {
        "sequence": _require_int(value.get("sequence"), f"{path}.sequence"),
        "lap_count": _require_int(value.get("lap_count"), f"{path}.lap_count"),
        "total_lap_gnss_ns": _require_int(
            value.get("total_lap_gnss_ns"), f"{path}.total_lap_gnss_ns"
        ),
    }
    if (endpoint["lap_count"] == 0) != (endpoint["total_lap_gnss_ns"] == 0):
        raise ValueError(f"{path} has inconsistent zero N/T")
    return endpoint


def _ppb_endpoints_equal(a: Any, b: Any) -> bool:
    if not isinstance(a, dict) or not isinstance(b, dict):
        return False
    try:
        return bool(
            int(a["sequence"]) == int(b["sequence"])
            and int(a["lap_count"]) == int(b["lap_count"])
            and int(a["total_lap_gnss_ns"]) == int(b["total_lap_gnss_ns"])
        )
    except (KeyError, TypeError, ValueError):
        return False


def _ppb_value_from_endpoints(
    current: Dict[str, Any], anchor: Dict[str, Any], lap_baseline_fs: int
) -> Dict[str, Any]:
    lap_count = int(current["lap_count"]) - int(anchor["lap_count"])
    total_ns = int(current["total_lap_gnss_ns"]) - int(anchor["total_lap_gnss_ns"])
    if lap_count <= 0 or total_ns <= 0 or lap_baseline_fs <= 0:
        raise ValueError(
            "Better-Buckets proof has nonpositive endpoint population/reference: "
            f"lap_count={lap_count} total_ns={total_ns} lap_baseline_fs={lap_baseline_fs}"
        )
    mean_fs = (float(total_ns) * 1_000_000.0) / float(lap_count)
    residual_ns = (mean_fs - float(lap_baseline_fs)) / 1_000_000.0
    return {
        "sample_count": lap_count,
        "ppb": residual_ns,
        "residual_ns": residual_ns,
    }


def _validate_firmware_ppb_checkpoint_delta(stats: Dict[str, Any]) -> Dict[str, Any]:
    """Validate one row-local PHOTONS checkpoint proof without mutating Pi state."""
    stats = _require_dict(stats, "photons.stats")
    raw = _require_dict(
        stats.get("rolling_ppb_checkpoint"),
        "photons.stats.rolling_ppb_checkpoint",
    )
    if raw.get("schema") != PHOTONS_PPB_FIRMWARE_DELTA_SCHEMA:
        raise ValueError(
            "unsupported PHOTONS Better-Buckets checkpoint schema "
            f"{raw.get('schema')!r}"
        )

    if stats.get("ppb_semantics") != PHOTONS_PPB_SEMANTICS:
        raise ValueError(
            f"unsupported PHOTONS Better-Buckets semantics {stats.get('ppb_semantics')!r}"
        )

    reset_count = _require_int(stats.get("reset_count"), "photons.stats.reset_count")
    update_count = _require_int(
        stats.get("update_count"), "photons.stats.update_count", minimum=1
    )
    current_sequence = _require_int(
        stats.get("rolling_ppb_current_sequence"),
        "photons.stats.rolling_ppb_current_sequence",
    )
    endpoint_admitted = _require_bool(
        stats.get("rolling_ppb_endpoint_admitted"),
        "photons.stats.rolling_ppb_endpoint_admitted",
    )
    interval_advanced = _require_bool(
        stats.get("rolling_ppb_interval_advanced"),
        "photons.stats.rolling_ppb_interval_advanced",
    )
    lap_baseline_fs = _require_int(
        stats.get("lap_baseline_fs"), "photons.stats.lap_baseline_fs", minimum=1
    )
    stats_lap_count = _require_int(
        stats.get("lap_count"), "photons.stats.lap_count"
    )
    stats_total_ns = _require_int(
        stats.get("total_lap_gnss_ns"), "photons.stats.total_lap_gnss_ns"
    )

    valid = _require_bool(raw.get("valid"), "rolling_ppb_checkpoint.valid")
    if valid != endpoint_admitted:
        raise ValueError(
            "PHOTONS checkpoint validity disagrees with endpoint admission: "
            f"checkpoint={valid} admitted={endpoint_admitted}"
        )
    if interval_advanced and not endpoint_admitted:
        raise ValueError("PHOTONS rolling interval advanced on an excluded endpoint")

    rolling_sequence = _require_int(
        raw.get("rolling_sequence"), "rolling_ppb_checkpoint.rolling_sequence"
    )
    second_count = _require_int(
        raw.get("second_count"), "rolling_ppb_checkpoint.second_count"
    )
    minute_count = _require_int(
        raw.get("minute_count"), "rolling_ppb_checkpoint.minute_count"
    )
    last_minute_key = _require_int(
        raw.get("last_minute_key"), "rolling_ppb_checkpoint.last_minute_key"
    )
    if second_count > PHOTONS_RECOVERY_SECOND_CAPACITY:
        raise ValueError(f"PHOTONS checkpoint second_count out of range: {second_count}")
    if minute_count > PHOTONS_RECOVERY_MINUTE_CAPACITY:
        raise ValueError(f"PHOTONS checkpoint minute_count out of range: {minute_count}")

    origin_valid = _require_bool(
        raw.get("origin_valid"), "rolling_ppb_checkpoint.origin_valid"
    )
    second_append_valid = _require_bool(
        raw.get("second_append_valid"),
        "rolling_ppb_checkpoint.second_append_valid",
    )
    minute_append_valid = _require_bool(
        raw.get("minute_append_valid"),
        "rolling_ppb_checkpoint.minute_append_valid",
    )

    proof: Dict[str, Any] = {}
    proof_checks = 0
    buckets = _require_dict(stats.get("ppb_buckets"), "photons.stats.ppb_buckets")
    windows = (
        ("10_min", PHOTONS_RECOVERY_10_MIN_SECONDS),
        ("60_min", PHOTONS_RECOVERY_60_MIN_SECONDS),
        ("8_hour", PHOTONS_RECOVERY_8_HOUR_SECONDS),
        ("24_hour", PHOTONS_RECOVERY_24_HOUR_SECONDS),
    )

    if not valid:
        if (
            rolling_sequence != 0
            or current_sequence != 0
            or second_count != 0
            or minute_count != 0
            or last_minute_key != 0
            or origin_valid
            or second_append_valid
            or minute_append_valid
        ):
            raise ValueError("invalid PHOTONS checkpoint publishes nonempty custody")
        for key, _seconds in windows:
            node = _require_dict(raw.get(key), f"rolling_ppb_checkpoint.{key}")
            if _require_bool(node.get("valid"), f"rolling_ppb_checkpoint.{key}.valid"):
                raise ValueError(f"invalid PHOTONS checkpoint publishes {key} proof")
            sample_count = _require_int(
                node.get("sample_count"),
                f"rolling_ppb_checkpoint.{key}.sample_count",
            )
            if sample_count != 0 or buckets.get(key) is not None:
                raise ValueError(f"invalid PHOTONS checkpoint publishes {key} population")
            proof[key] = {"valid": False, "sample_count": 0, "anchor": None}
        return {
            "schema": PHOTONS_PPB_FIRMWARE_DELTA_SCHEMA,
            "valid": False,
            "reset_count": reset_count,
            "update_count": update_count,
            "rolling_sequence": 0,
            "second_count": 0,
            "minute_count": 0,
            "last_minute_key": 0,
            "origin_valid": False,
            "origin": _ppb_zero_endpoint(),
            "current": _ppb_zero_endpoint(),
            "second_append_valid": False,
            "second_append": None,
            "minute_append_valid": False,
            "minute_append": None,
            "interval_advanced": False,
            "proof": proof,
            "proof_checks": 0,
        }

    if rolling_sequence != update_count or current_sequence != update_count:
        raise ValueError(
            "PHOTONS checkpoint chronology mismatch: "
            f"update={update_count} stats_current={current_sequence} "
            f"checkpoint={rolling_sequence}"
        )
    if second_count <= 0 or minute_count <= 0 or not origin_valid:
        raise ValueError("valid PHOTONS checkpoint lacks ring/origin custody")
    if last_minute_key != _ppb_minute_key(rolling_sequence):
        raise ValueError(
            "PHOTONS checkpoint minute-key mismatch: "
            f"sequence={rolling_sequence} key={last_minute_key}"
        )

    current = _ppb_endpoint_from_payload(
        raw.get("current"), path="rolling_ppb_checkpoint.current"
    )
    origin = _ppb_endpoint_from_payload(
        raw.get("origin"), path="rolling_ppb_checkpoint.origin"
    )
    if current["sequence"] != rolling_sequence:
        raise ValueError("PHOTONS checkpoint current endpoint identity mismatch")
    if (
        current["lap_count"] != stats_lap_count
        or current["total_lap_gnss_ns"] != stats_total_ns
    ):
        raise ValueError("PHOTONS checkpoint current endpoint disagrees with stats N/T")
    if origin != _ppb_zero_endpoint():
        raise ValueError("PHOTONS checkpoint statistical origin is not exact zero")

    if not second_append_valid:
        raise ValueError("valid PHOTONS checkpoint lacks second append testimony")
    second_append = _ppb_endpoint_from_payload(
        raw.get("second_append"), path="rolling_ppb_checkpoint.second_append"
    )
    if not _ppb_endpoints_equal(second_append, current):
        raise ValueError("PHOTONS second append is not the current endpoint")

    minute_append = None
    if minute_append_valid:
        minute_append = _ppb_endpoint_from_payload(
            raw.get("minute_append"), path="rolling_ppb_checkpoint.minute_append"
        )
        if not _ppb_endpoints_equal(minute_append, current):
            raise ValueError("PHOTONS minute append is not the current endpoint")

    for key, _seconds in windows:
        node = _require_dict(raw.get(key), f"rolling_ppb_checkpoint.{key}")
        node_valid = _require_bool(
            node.get("valid"), f"rolling_ppb_checkpoint.{key}.valid"
        )
        sample_count = _require_int(
            node.get("sample_count"),
            f"rolling_ppb_checkpoint.{key}.sample_count",
        )
        recorded = buckets.get(key)
        if not node_valid:
            if sample_count != 0 or recorded is not None:
                raise ValueError(
                    f"PHOTONS {key} availability disagrees with checkpoint proof"
                )
            proof[key] = {"valid": False, "sample_count": 0, "anchor": None}
            continue

        if sample_count <= 0:
            raise ValueError(f"PHOTONS {key} checkpoint proof has zero population")
        anchor = _ppb_endpoint_from_payload(
            node.get("anchor"), path=f"rolling_ppb_checkpoint.{key}.anchor"
        )
        computed = _ppb_value_from_endpoints(current, anchor, lap_baseline_fs)
        if computed["sample_count"] != sample_count:
            raise ValueError(
                f"PHOTONS {key} proof N mismatch: "
                f"published={sample_count} endpoint={computed['sample_count']}"
            )
        recorded = _require_dict(recorded, f"photons.stats.ppb_buckets.{key}")
        recorded_n = _require_int(
            recorded.get("sample_count"),
            f"photons.stats.ppb_buckets.{key}.sample_count",
            minimum=1,
        )
        recorded_ppb = _require_float(
            recorded.get("ppb"), f"photons.stats.ppb_buckets.{key}.ppb"
        )
        recorded_residual_ns = _require_float(
            recorded.get("residual_ns"), f"photons.stats.ppb_buckets.{key}.residual_ns"
        )
        if recorded_n != sample_count:
            raise ValueError(
                f"PHOTONS {key} bucket/proof N mismatch: "
                f"bucket={recorded_n} proof={sample_count}"
            )
        delta_ppb = float(computed["ppb"]) - recorded_ppb
        delta_residual_ns = float(computed["residual_ns"]) - recorded_residual_ns
        if abs(delta_ppb) > PHOTONS_RECOVERY_VERIFY_TOLERANCE:
            raise ValueError(
                "PHOTONS Better-Buckets row-local proof mismatch: "
                f"window={key} computed={computed['ppb']:.9f} "
                f"recorded={recorded_ppb:.9f} delta={delta_ppb:.9f}"
            )
        if abs(delta_residual_ns) > 0.000001:
            raise ValueError(
                "PHOTONS Better-Buckets residual proof mismatch: "
                f"window={key} computed={computed['residual_ns']:.6f} "
                f"recorded={recorded_residual_ns:.6f} delta={delta_residual_ns:.6f}"
            )
        proof[key] = {
            "valid": True,
            "sample_count": sample_count,
            "anchor": anchor,
            "computed_ppb": round(float(computed["ppb"]), 9),
            "recorded_ppb": round(recorded_ppb, 9),
            "delta_ppb": round(delta_ppb, 9),
            "recorded_residual_ns": round(recorded_residual_ns, 6),
            "computed_residual_ns": round(float(computed["residual_ns"]), 6),
            "delta_residual_ns": round(delta_residual_ns, 6),
        }
        proof_checks += 1

    return {
        "schema": PHOTONS_PPB_FIRMWARE_DELTA_SCHEMA,
        "valid": True,
        "reset_count": reset_count,
        "update_count": update_count,
        "lap_baseline_fs": lap_baseline_fs,
        "rolling_sequence": rolling_sequence,
        "second_count": second_count,
        "minute_count": minute_count,
        "last_minute_key": last_minute_key,
        "origin_valid": True,
        "origin": origin,
        "current": current,
        "second_append_valid": True,
        "second_append": second_append,
        "minute_append_valid": minute_append_valid,
        "minute_append": minute_append,
        "interval_advanced": interval_advanced,
        "proof": proof,
        "proof_checks": proof_checks,
    }


def _ppb_checkpoint_delta_signature(delta: Dict[str, Any]) -> str:
    stable = {
        key: delta.get(key)
        for key in (
            "schema",
            "valid",
            "reset_count",
            "update_count",
            "lap_baseline_fs",
            "rolling_sequence",
            "second_count",
            "minute_count",
            "last_minute_key",
            "origin_valid",
            "origin",
            "current",
            "second_append_valid",
            "second_append",
            "minute_append_valid",
            "minute_append",
            "interval_advanced",
            "proof",
        )
    }
    return json.dumps(stable, sort_keys=True, separators=(",", ":"))


def _ppb_checkpoint_new_runtime(
    *,
    reason: str,
    seed_source: str = "FIRMWARE_APPENDS",
    gap_count: int = 0,
    last_gap: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    return {
        "reset_count": None,
        "last_update_count": None,
        "last_delta_signature": None,
        "rolling_sequence": 0,
        "current_sequence": 0,
        "expected_second_count": 0,
        "expected_minute_count": 0,
        "last_minute_key": 0,
        "origin_valid": False,
        "origin": _ppb_zero_endpoint(),
        "current": _ppb_zero_endpoint(),
        "second_history": deque(maxlen=PHOTONS_RECOVERY_SECOND_CAPACITY),
        "minute_history": deque(maxlen=PHOTONS_RECOVERY_MINUTE_CAPACITY),
        "contiguous_from_update_count": None,
        "gap_count": int(gap_count),
        "last_gap": copy.deepcopy(last_gap),
        "status_reason": str(reason),
        "seed_source": str(seed_source),
        "seed_source_db_detail_id": None,
        "proof_checks": 0,
        "rolling_custody_lost": False,
    }


def _ppb_checkpoint_runtime_ensure_locked() -> Dict[str, Any]:
    global _ppb_checkpoint_runtime
    if _ppb_checkpoint_runtime is None:
        _ppb_checkpoint_runtime = _ppb_checkpoint_new_runtime(
            reason="NO_CHECKPOINT_SEED"
        )
    return _ppb_checkpoint_runtime


def _ppb_checkpoint_snapshot_locked(runtime: Dict[str, Any]) -> Dict[str, Any]:
    second_history = [copy.deepcopy(value) for value in runtime["second_history"]]
    minute_history = [copy.deepcopy(value) for value in runtime["minute_history"]]
    expected_second = int(runtime.get("expected_second_count") or 0)
    expected_minute = int(runtime.get("expected_minute_count") or 0)
    rolling_sequence = int(runtime.get("rolling_sequence") or 0)
    current_sequence = int(runtime.get("current_sequence") or 0)
    current = copy.deepcopy(runtime.get("current") or _ppb_zero_endpoint())

    complete = bool(
        not runtime.get("rolling_custody_lost")
        and current_sequence > 0
        and int(current.get("lap_count") or 0) > 0
        and int(current.get("total_lap_gnss_ns") or 0) > 0
        and runtime.get("origin_valid")
        and len(second_history) == expected_second
        and len(minute_history) == expected_minute
        and bool(second_history)
        and _ppb_endpoints_equal(second_history[-1], current)
        and int(runtime.get("last_minute_key") or 0)
        == _ppb_minute_key(current_sequence)
        and bool(minute_history)
        and _ppb_minute_key(int(minute_history[-1]["sequence"]))
        == int(runtime.get("last_minute_key") or 0)
    )

    if complete:
        status = "RECOVERABLE"
    elif runtime.get("rolling_custody_lost"):
        status = "ROLLING_CUSTODY_LOST"
    elif runtime.get("last_gap"):
        status = "WARMING_AFTER_OBSERVATION_GAP"
    else:
        status = "WARMING_FOR_COMPLETE_FIRMWARE_HISTORY"

    return {
        "schema": PHOTONS_PPB_PI_CHECKPOINT_SCHEMA,
        "source_schema": PHOTONS_PPB_FIRMWARE_DELTA_SCHEMA,
        "valid": True,
        "recoverable": complete,
        "status": status,
        "status_reason": runtime.get("status_reason"),
        "reset_count": runtime.get("reset_count"),
        "update_count": runtime.get("last_update_count"),
        "rolling_sequence": rolling_sequence,
        "current_sequence": current_sequence,
        "last_minute_key": int(runtime.get("last_minute_key") or 0),
        "origin_valid": bool(runtime.get("origin_valid")),
        "origin": copy.deepcopy(runtime.get("origin") or _ppb_zero_endpoint()),
        "current": current,
        "expected_second_count": expected_second,
        "expected_minute_count": expected_minute,
        "second_count": len(second_history),
        "minute_count": len(minute_history),
        "second_history": second_history,
        "minute_history": minute_history,
        "contiguous_from_update_count": runtime.get(
            "contiguous_from_update_count"
        ),
        "gap_count": int(runtime.get("gap_count") or 0),
        "last_gap": copy.deepcopy(runtime.get("last_gap")),
        "seed_source": runtime.get("seed_source"),
        "seed_source_db_detail_id": runtime.get("seed_source_db_detail_id"),
        "last_delta_signature": runtime.get("last_delta_signature"),
        "proof_checks": int(runtime.get("proof_checks") or 0),
    }


def _ppb_checkpoint_history_anchor_locked(
    runtime: Dict[str, Any], window_seconds: int, *, exact_second_history: bool
) -> Optional[Dict[str, Any]]:
    """Return the same bounded-ring anchor the firmware Better-Buckets court used."""
    current = runtime.get("current")
    if not isinstance(current, dict):
        return None
    current_sequence = int(current.get("sequence") or 0)
    if current_sequence <= 0:
        return None

    # The statistical epoch origin is separate testimony.  It is a lawful rolling
    # anchor only when it is literally present in the bounded producer ring.  This
    # matters after recovery of a truthful suffix: origin may remain sequence 0
    # while the oldest retained 24-hour minute endpoint is much newer.
    target = max(0, current_sequence - int(window_seconds))
    history = (
        runtime["second_history"]
        if exact_second_history
        else runtime["minute_history"]
    )
    for endpoint in history:
        sequence = int(endpoint.get("sequence") or 0)
        if target <= sequence < current_sequence:
            return endpoint
    return None


def _ppb_checkpoint_assert_history_matches_proof_locked(
    runtime: Dict[str, Any], delta: Dict[str, Any]
) -> None:
    for key, seconds, exact in (
        ("10_min", PHOTONS_RECOVERY_10_MIN_SECONDS, True),
        ("60_min", PHOTONS_RECOVERY_60_MIN_SECONDS, False),
        ("8_hour", PHOTONS_RECOVERY_8_HOUR_SECONDS, False),
        ("24_hour", PHOTONS_RECOVERY_24_HOUR_SECONDS, False),
    ):
        proof = (delta.get("proof") or {}).get(key)
        if not isinstance(proof, dict) or not proof.get("valid"):
            continue
        actual_anchor = _ppb_checkpoint_history_anchor_locked(
            runtime, seconds, exact_second_history=exact
        )
        if not _ppb_endpoints_equal(proof.get("anchor"), actual_anchor):
            raise RuntimeError(
                "Pi PHOTONS checkpoint ring disagrees with producer proof: "
                f"window={key} expected={proof.get('anchor')!r} "
                f"actual={actual_anchor!r}"
            )


def _ppb_checkpoint_ingest(stats: Dict[str, Any]) -> Dict[str, Any]:
    """Fold one verified producer delta into the literal Pi recovery ledger."""
    global _ppb_checkpoint_runtime
    global _ppb_checkpoint_rows_verified
    global _ppb_checkpoint_recoverable_rows
    global _ppb_checkpoint_warming_rows
    global _ppb_checkpoint_gap_count
    global _ppb_checkpoint_reacquire_request_count

    delta = _validate_firmware_ppb_checkpoint_delta(stats)
    signature = _ppb_checkpoint_delta_signature(delta)
    reset_count = int(delta["reset_count"])
    update_count = int(delta["update_count"])

    with _ppb_checkpoint_lock:
        runtime = _ppb_checkpoint_runtime_ensure_locked()
        previous_reset = runtime.get("reset_count")
        previous_update = runtime.get("last_update_count")

        if previous_reset is not None and int(previous_reset) != reset_count:
            gap = {
                "reason": "STATISTICS_EPOCH_REPLACED",
                "previous_reset_count": int(previous_reset),
                "previous_update_count": int(previous_update or 0),
                "observed_reset_count": reset_count,
                "observed_update_count": update_count,
            }
            _ppb_checkpoint_runtime = _ppb_checkpoint_new_runtime(
                reason="STATISTICS_EPOCH_REPLACED",
                gap_count=int(runtime.get("gap_count") or 0),
                last_gap=gap,
            )
            runtime = _ppb_checkpoint_runtime
            previous_update = None

        if previous_update is not None and update_count == int(previous_update):
            if runtime.get("last_delta_signature") not in (None, signature):
                raise RuntimeError(
                    "duplicate PHOTONS checkpoint identity changed producer testimony: "
                    f"reset={reset_count} update={update_count}"
                )
            runtime["last_delta_signature"] = signature
            runtime["proof_checks"] = int(delta.get("proof_checks") or 0)
            _ppb_checkpoint_rows_verified += 1
            return _ppb_checkpoint_snapshot_locked(runtime)

        if (
            previous_update is not None
            and update_count == int(previous_update) + 1
            and delta["valid"]
            and int(runtime.get("current_sequence") or 0) > 0
        ):
            previous_current = _require_dict(
                runtime.get("current"), "Pi checkpoint previous current"
            )
            current = delta["current"]
            if int(current["sequence"]) != int(previous_current["sequence"]) + 1:
                raise RuntimeError(
                    "adjacent PHOTONS checkpoint update has nonadjacent endpoint identity"
                )
            delta_laps = int(current["lap_count"]) - int(
                previous_current["lap_count"]
            )
            delta_ns = int(current["total_lap_gnss_ns"]) - int(
                previous_current["total_lap_gnss_ns"]
            )
            if delta_laps < 0 or delta_ns < 0:
                raise RuntimeError("adjacent PHOTONS checkpoint cumulative state regressed")
            if (delta_laps == 0) != (delta_ns == 0):
                raise RuntimeError("adjacent PHOTONS checkpoint delta N/T does not close")
            if bool(delta["interval_advanced"]) != bool(delta_laps > 0):
                raise RuntimeError(
                    "PHOTONS interval_advanced disagrees with cumulative endpoint delta"
                )

        if previous_update is not None and update_count != int(previous_update) + 1:
            gap = {
                "reason": "PI_OBSERVATION_GAP",
                "expected_update_count": int(previous_update) + 1,
                "observed_update_count": update_count,
                "missing_rows": max(0, update_count - int(previous_update) - 1),
            }
            _ppb_checkpoint_runtime = _ppb_checkpoint_new_runtime(
                reason="PI_OBSERVATION_GAP",
                gap_count=int(runtime.get("gap_count") or 0) + 1,
                last_gap=gap,
            )
            runtime = _ppb_checkpoint_runtime
            _ppb_checkpoint_gap_count += 1

            # A Pi observation gap destroys Pi-owned literal ring custody, but it
            # does not destroy the surviving Teensy producer's Better-Buckets
            # ancestry.  Startup live adoption already performs an explicit full-ring
            # export before campaign control opens.  During ordinary RUNNING state,
            # request the same read-only producer-authority reacquisition asynchronously
            # so the state worker can continue ingesting rows and meet the export court
            # at one exact live endpoint.  Never reconstruct the missing rows locally.
            if _campaign_control_ready.is_set():
                _ppb_checkpoint_reacquire_request_count += 1
                _ppb_checkpoint_reacquire_requested.set()

        if not delta["valid"]:
            gap = {
                "reason": "ROLLING_CUSTODY_LOST",
                "reset_count": reset_count,
                "update_count": update_count,
            }
            _ppb_checkpoint_runtime = _ppb_checkpoint_new_runtime(
                reason="ROLLING_CUSTODY_LOST",
                gap_count=int(runtime.get("gap_count") or 0) + 1,
                last_gap=gap,
            )
            runtime = _ppb_checkpoint_runtime
            runtime["reset_count"] = reset_count
            runtime["last_update_count"] = update_count
            runtime["last_delta_signature"] = signature
            runtime["rolling_sequence"] = update_count
            runtime["rolling_custody_lost"] = True
            _ppb_checkpoint_rows_verified += 1
            _ppb_checkpoint_warming_rows += 1
            _ppb_checkpoint_gap_count += 1
            return _ppb_checkpoint_snapshot_locked(runtime)

        if (
            int(delta["second_count"]) < len(runtime["second_history"])
            or int(delta["minute_count"]) < len(runtime["minute_history"])
        ):
            replacement = {
                "reason": "PRODUCER_RING_REPLACED",
                "previous_second_count": len(runtime["second_history"]),
                "previous_minute_count": len(runtime["minute_history"]),
                "observed_second_count": int(delta["second_count"]),
                "observed_minute_count": int(delta["minute_count"]),
                "observed_update_count": update_count,
            }
            _ppb_checkpoint_runtime = _ppb_checkpoint_new_runtime(
                reason="PRODUCER_RING_REPLACED",
                gap_count=int(runtime.get("gap_count") or 0),
                last_gap=replacement,
            )
            runtime = _ppb_checkpoint_runtime

        if runtime.get("contiguous_from_update_count") is None:
            runtime["contiguous_from_update_count"] = update_count

        # A fresh statistical epoch includes exact zero in both producer rings.
        if (
            not runtime["second_history"]
            and update_count == 1
            and int(delta["second_count"]) == 2
        ):
            runtime["second_history"].append(copy.deepcopy(delta["origin"]))
        if (
            not runtime["minute_history"]
            and update_count == 1
            and int(delta["minute_count"]) == 2
        ):
            runtime["minute_history"].append(copy.deepcopy(delta["origin"]))

        second_endpoint = copy.deepcopy(delta["second_append"])
        if runtime["second_history"] and int(second_endpoint["sequence"]) <= int(
            runtime["second_history"][-1]["sequence"]
        ):
            raise RuntimeError("PHOTONS second append is not newer than Pi ledger tail")
        runtime["second_history"].append(second_endpoint)

        if delta["minute_append_valid"]:
            minute_endpoint = copy.deepcopy(delta["minute_append"])
            if runtime["minute_history"] and int(minute_endpoint["sequence"]) <= int(
                runtime["minute_history"][-1]["sequence"]
            ):
                raise RuntimeError(
                    "PHOTONS minute append is not newer than Pi ledger tail"
                )
            runtime["minute_history"].append(minute_endpoint)

        runtime["reset_count"] = reset_count
        runtime["last_update_count"] = update_count
        runtime["last_delta_signature"] = signature
        runtime["rolling_sequence"] = update_count
        runtime["current_sequence"] = int(delta["rolling_sequence"])
        runtime["expected_second_count"] = int(delta["second_count"])
        runtime["expected_minute_count"] = int(delta["minute_count"])
        runtime["last_minute_key"] = int(delta["last_minute_key"])
        runtime["origin_valid"] = bool(delta["origin_valid"])
        runtime["origin"] = copy.deepcopy(delta["origin"])
        runtime["current"] = copy.deepcopy(delta["current"])
        runtime["proof_checks"] = int(delta.get("proof_checks") or 0)
        runtime["rolling_custody_lost"] = False

        if len(runtime["second_history"]) > int(delta["second_count"]):
            raise RuntimeError("Pi PHOTONS second history exceeds producer ring count")
        if len(runtime["minute_history"]) > int(delta["minute_count"]):
            raise RuntimeError("Pi PHOTONS minute history exceeds producer ring count")

        snapshot = _ppb_checkpoint_snapshot_locked(runtime)
        if snapshot["recoverable"]:
            _ppb_checkpoint_assert_history_matches_proof_locked(runtime, delta)
            runtime["status_reason"] = "COMPLETE_PRODUCER_RING_CUSTODY"
            _ppb_checkpoint_recoverable_rows += 1
        else:
            runtime["status_reason"] = "WAITING_FOR_UNSEEN_PRODUCER_HISTORY"
            _ppb_checkpoint_warming_rows += 1
        _ppb_checkpoint_rows_verified += 1
        return _ppb_checkpoint_snapshot_locked(runtime)


def _normalize_saved_ppb_checkpoint(value: Any) -> Dict[str, Any]:
    if not isinstance(value, dict) or value.get("schema") != PHOTONS_PPB_PI_CHECKPOINT_SCHEMA:
        raise ValueError("canonical PHOTONS row lacks the Pi Better-Buckets checkpoint")
    if not _require_bool(value.get("valid"), "ppb_restore_checkpoint.valid"):
        raise ValueError("saved Pi PHOTONS checkpoint is not valid")

    reset_count = _require_int(
        value.get("reset_count"), "ppb_restore_checkpoint.reset_count"
    )
    update_count = _require_int(
        value.get("update_count"), "ppb_restore_checkpoint.update_count", minimum=1
    )
    rolling_sequence = _require_int(
        value.get("rolling_sequence"), "ppb_restore_checkpoint.rolling_sequence"
    )
    current_sequence = _require_int(
        value.get("current_sequence"), "ppb_restore_checkpoint.current_sequence"
    )
    expected_second = _require_int(
        value.get("expected_second_count"),
        "ppb_restore_checkpoint.expected_second_count",
    )
    expected_minute = _require_int(
        value.get("expected_minute_count"),
        "ppb_restore_checkpoint.expected_minute_count",
    )
    last_minute_key = _require_int(
        value.get("last_minute_key"), "ppb_restore_checkpoint.last_minute_key"
    )
    if rolling_sequence != update_count or current_sequence > rolling_sequence:
        raise ValueError("saved Pi PHOTONS checkpoint chronology is invalid")
    if expected_second > PHOTONS_RECOVERY_SECOND_CAPACITY:
        raise ValueError("saved Pi PHOTONS second ring count is invalid")
    if expected_minute > PHOTONS_RECOVERY_MINUTE_CAPACITY:
        raise ValueError("saved Pi PHOTONS minute ring count is invalid")

    origin_valid = _require_bool(
        value.get("origin_valid"), "ppb_restore_checkpoint.origin_valid"
    )
    origin = _ppb_endpoint_from_payload(
        value.get("origin"), path="ppb_restore_checkpoint.origin"
    )
    current = _ppb_endpoint_from_payload(
        value.get("current"), path="ppb_restore_checkpoint.current"
    )
    if current["sequence"] != current_sequence:
        raise ValueError("saved Pi PHOTONS current endpoint identity is invalid")
    if origin_valid and origin != _ppb_zero_endpoint():
        raise ValueError("saved Pi PHOTONS origin is not exact zero")

    def history(name: str, capacity: int) -> List[Dict[str, Any]]:
        raw = value.get(name)
        if not isinstance(raw, list) or len(raw) > capacity:
            raise ValueError(f"saved Pi PHOTONS {name} is invalid")
        out: List[Dict[str, Any]] = []
        previous: Optional[Dict[str, Any]] = None
        for index, node in enumerate(raw):
            endpoint = _ppb_endpoint_from_payload(
                node, path=f"ppb_restore_checkpoint.{name}[{index}]"
            )
            if previous is not None and (
                endpoint["sequence"] <= previous["sequence"]
                or endpoint["lap_count"] < previous["lap_count"]
                or endpoint["total_lap_gnss_ns"]
                < previous["total_lap_gnss_ns"]
            ):
                raise ValueError(f"saved Pi PHOTONS {name} chronology regresses")
            if endpoint["sequence"] > current_sequence:
                raise ValueError(f"saved Pi PHOTONS {name} extends beyond current")
            out.append(endpoint)
            previous = endpoint
        return out

    second_history = history("second_history", PHOTONS_RECOVERY_SECOND_CAPACITY)
    minute_history = history("minute_history", PHOTONS_RECOVERY_MINUTE_CAPACITY)
    published_second_count = _require_int(
        value.get("second_count"), "ppb_restore_checkpoint.second_count"
    )
    published_minute_count = _require_int(
        value.get("minute_count"), "ppb_restore_checkpoint.minute_count"
    )
    if published_second_count != len(second_history):
        raise ValueError("saved Pi PHOTONS second_count disagrees with literal history")
    if published_minute_count != len(minute_history):
        raise ValueError("saved Pi PHOTONS minute_count disagrees with literal history")
    if len(second_history) > expected_second or len(minute_history) > expected_minute:
        raise ValueError("saved Pi PHOTONS histories exceed producer counts")

    runtime = _ppb_checkpoint_new_runtime(
        reason=str(value.get("status_reason") or "SAVED_CHECKPOINT"),
        seed_source=str(value.get("seed_source") or "DURABLE_CHECKPOINT"),
        gap_count=_require_int(value.get("gap_count"), "ppb_restore_checkpoint.gap_count"),
        last_gap=(
            copy.deepcopy(value.get("last_gap"))
            if isinstance(value.get("last_gap"), dict)
            else None
        ),
    )
    runtime["reset_count"] = reset_count
    runtime["last_update_count"] = update_count
    runtime["last_delta_signature"] = value.get("last_delta_signature")
    runtime["rolling_sequence"] = rolling_sequence
    runtime["current_sequence"] = current_sequence
    runtime["expected_second_count"] = expected_second
    runtime["expected_minute_count"] = expected_minute
    runtime["last_minute_key"] = last_minute_key
    runtime["origin_valid"] = origin_valid
    runtime["origin"] = origin
    runtime["current"] = current
    runtime["second_history"].extend(copy.deepcopy(second_history))
    runtime["minute_history"].extend(copy.deepcopy(minute_history))
    runtime["contiguous_from_update_count"] = value.get(
        "contiguous_from_update_count"
    )
    runtime["seed_source_db_detail_id"] = value.get("seed_source_db_detail_id")
    runtime["proof_checks"] = _require_int(
        value.get("proof_checks"), "ppb_restore_checkpoint.proof_checks"
    )
    runtime["rolling_custody_lost"] = str(value.get("status") or "") == "ROLLING_CUSTODY_LOST"
    normalized = _ppb_checkpoint_snapshot_locked(runtime)
    if _require_bool(value.get("recoverable"), "ppb_restore_checkpoint.recoverable") != bool(
        normalized["recoverable"]
    ):
        raise ValueError("saved Pi PHOTONS recoverable flag disagrees with history")
    return normalized


def _write_photons_recovery_config(cur: Any, checkpoint: Dict[str, Any]) -> None:
    """Persist the literal PHOTONS recovery image as one singleton config row."""
    normalized = _normalize_saved_ppb_checkpoint(checkpoint)
    encoded = json.dumps(normalized, separators=(",", ":"))
    cur.execute(
        "UPDATE config SET payload = %s::jsonb WHERE config_key = %s",
        (encoded, PHOTONS_RECOVERY_CONFIG_KEY),
    )
    if cur.rowcount > 1:
        raise RuntimeError(
            f"config.{PHOTONS_RECOVERY_CONFIG_KEY} is not a singleton: rows={cur.rowcount}"
        )
    if cur.rowcount == 0:
        cur.execute(
            "INSERT INTO config (config_key, payload) VALUES (%s, %s::jsonb)",
            (PHOTONS_RECOVERY_CONFIG_KEY, encoded),
        )
        if cur.rowcount != 1:
            raise RuntimeError(
                f"config.{PHOTONS_RECOVERY_CONFIG_KEY} insert did not create exactly one row"
            )


def _read_photons_recovery_config() -> Optional[Dict[str, Any]]:
    """Read config.PHOTONS_RECOVERY, or None before the first durable checkpoint."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            "SELECT payload FROM config WHERE config_key = %s",
            (PHOTONS_RECOVERY_CONFIG_KEY,),
        )
        row = cur.fetchone()
    if row is None:
        return None
    payload = row["payload"]
    if isinstance(payload, str):
        payload = json.loads(payload)
    return _normalize_saved_ppb_checkpoint(payload)


def _retire_orphaned_photons_recovery_config_if_domain_empty() -> Dict[str, Any]:
    """Retire stale singleton custody only when the durable LANTERN domain is empty.

    The singleton is continuation state for canonical PHOTONS rows, not an
    independent history source.  If every LANTERN master/detail has been removed,
    a leftover config.PHOTONS_RECOVERY row has no durable owner and must not
    influence the new database epoch.

    Any surviving LANTERN master or detail keeps the ordinary fail-closed recovery
    courts intact.
    """
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT
                (SELECT COUNT(*) FROM campaign_master WHERE campaign_type = %s) AS master_count,
                (SELECT COUNT(*) FROM campaign_detail WHERE campaign_type = %s) AS detail_count
            """,
            (CAMPAIGN_TYPE_LANTERN, CAMPAIGN_TYPE_LANTERN),
        )
        row = cur.fetchone()
        if not isinstance(row, dict):
            raise RuntimeError("PHOTONS empty-domain court returned no count row")

        master_count = int(row.get("master_count") or 0)
        detail_count = int(row.get("detail_count") or 0)
        if master_count != 0 or detail_count != 0:
            return {
                "domain_empty": False,
                "master_count": master_count,
                "detail_count": detail_count,
                "recovery_config_deleted": False,
            }

        cur.execute(
            "DELETE FROM config WHERE config_key = %s",
            (PHOTONS_RECOVERY_CONFIG_KEY,),
        )
        deleted = int(cur.rowcount or 0)
        if deleted > 1:
            raise RuntimeError(
                f"config.{PHOTONS_RECOVERY_CONFIG_KEY} is not a singleton: rows={deleted}"
            )

    return {
        "domain_empty": True,
        "master_count": 0,
        "detail_count": 0,
        "recovery_config_deleted": bool(deleted),
    }


def _photons_checkpoint_receipt_summary(
    checkpoint: Optional[Dict[str, Any]],
) -> Dict[str, Any]:
    """Fingerprint one exact private PHOTONS checkpoint without copying its rings."""
    if checkpoint is None:
        return {
            "schema": PHOTONS_PPB_PI_CHECKPOINT_SCHEMA,
            "sha256": None,
            "digest_available": False,
            "digest_reason": "SOURCE_CHECKPOINT_NOT_RETAINED_ACROSS_PRIOR_COMMIT",
        }
    saved = _normalize_saved_ppb_checkpoint(checkpoint)
    encoded = json.dumps(saved, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return {
        "schema": saved.get("schema"),
        "sha256": hashlib.sha256(encoded).hexdigest(),
        "digest_available": True,
        "reset_count": int(saved["reset_count"]),
        "update_count": int(saved["update_count"]),
        "current_sequence": int(saved["current_sequence"]),
        "recoverable": bool(saved.get("recoverable")),
        "status": str(saved.get("status") or ""),
        "second_count": int(saved.get("second_count") or 0),
        "expected_second_count": int(saved.get("expected_second_count") or 0),
        "minute_count": int(saved.get("minute_count") or 0),
        "expected_minute_count": int(saved.get("expected_minute_count") or 0),
        "gap_count": int(saved.get("gap_count") or 0),
    }


def _find_photons_recovery_source_detail_id(
    *,
    boundary_detail_id: int,
    source_sequence: int,
    source_reset_count: int,
    source_update_count: int,
) -> int:
    """Resolve firmware source identity to exactly one earlier canonical PHOTONS row."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id
            FROM campaign_detail
            WHERE campaign_type = %s
              AND id < %s
              AND payload #>> '{schema}' = %s
              AND payload #>> '{sequence}' = %s
              AND payload #>> '{photons,stats,reset_count}' = %s
              AND payload #>> '{photons,stats,update_count}' = %s
            ORDER BY id DESC
            LIMIT 2
            """,
            (
                CAMPAIGN_TYPE_LANTERN,
                int(boundary_detail_id),
                PHOTONS_SCHEMA,
                str(int(source_sequence)),
                str(int(source_reset_count)),
                str(int(source_update_count)),
            ),
        )
        rows = cur.fetchall()
    if len(rows) != 1:
        raise RuntimeError(
            "PHOTONS recovery source identity is not unique: "
            f"boundary={boundary_detail_id} sequence={source_sequence} "
            f"reset/update={source_reset_count}/{source_update_count} rows={len(rows)}"
        )
    return int(rows[0]["id"])


def _record_photons_recovery_receipt(
    *,
    proof: Dict[str, Any],
    recovery_mode: str,
    generation: int,
    source: Optional[Dict[str, Any]],
    source_identity: Optional[Dict[str, Any]] = None,
    source_checkpoint: Optional[Dict[str, Any]] = None,
    history: Optional[Dict[str, Any]] = None,
    producer_mutated: bool,
) -> Dict[str, Any]:
    """Attach one immutable Pi receipt to the durable PHOTONS recovery proof row."""
    boundary_id = _require_int(proof.get("detail_id"), "PHOTONS receipt proof.detail_id", minimum=1)
    boundary_sequence_expected = _require_int(
        proof.get("sequence"), "PHOTONS receipt proof.sequence", minimum=1
    )
    boundary_reset_expected = _require_int(
        proof.get("reset_count"), "PHOTONS receipt proof.reset_count"
    )
    boundary_update_expected = _require_int(
        proof.get("update_count"), "PHOTONS receipt proof.update_count", minimum=1
    )

    if source is not None:
        source_id = _require_int(source.get("db_detail_id"), "PHOTONS receipt source.detail_id", minimum=1)
        source_sequence = _require_int(source.get("sequence"), "PHOTONS receipt source.sequence", minimum=1)
        source_reset = _require_int(source.get("reset_count"), "PHOTONS receipt source.reset_count")
        source_update = _require_int(source.get("update_count"), "PHOTONS receipt source.update_count", minimum=1)
        if source_checkpoint is None:
            candidate = source.get("ppb_restore_checkpoint")
            source_checkpoint = candidate if isinstance(candidate, dict) else None
    else:
        identity = _require_dict(source_identity, "PHOTONS receipt source_identity")
        source_sequence = _require_int(identity.get("source_sequence"), "PHOTONS receipt source_sequence", minimum=1)
        source_reset = _require_int(identity.get("source_reset_count"), "PHOTONS receipt source_reset_count")
        source_update = _require_int(identity.get("source_update_count"), "PHOTONS receipt source_update_count", minimum=1)
        source_id = _find_photons_recovery_source_detail_id(
            boundary_detail_id=boundary_id,
            source_sequence=source_sequence,
            source_reset_count=source_reset,
            source_update_count=source_update,
        )

    mode = str(recovery_mode).strip().upper()
    exact_n_plus_1 = mode in {"HELD_RESTORE", "PENDING_RESTORE_PROOF"}
    checkpoint_summary = _photons_checkpoint_receipt_summary(source_checkpoint)
    if source_checkpoint is not None and (
        int(checkpoint_summary["reset_count"]) != source_reset
        or int(checkpoint_summary["update_count"]) != source_update
    ):
        raise RuntimeError("PHOTONS recovery receipt checkpoint chronology disagrees with source")

    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            "SELECT payload FROM campaign_detail WHERE id = %s AND campaign_type = %s FOR UPDATE",
            (boundary_id, CAMPAIGN_TYPE_LANTERN),
        )
        row = cur.fetchone()
        if row is None:
            raise RuntimeError(
                f"PHOTONS recovery receipt boundary detail_id={boundary_id} is missing"
            )
        boundary = row["payload"]
        if isinstance(boundary, str):
            boundary = json.loads(boundary)
        if not isinstance(boundary, dict) or boundary.get("schema") != PHOTONS_SCHEMA:
            raise RuntimeError("PHOTONS recovery receipt boundary is not PHOTONS_V1")
        instrument = _require_dict(boundary.get("photons"), "PHOTONS receipt boundary.photons")
        stats = _require_dict(instrument.get("stats"), "PHOTONS receipt boundary.stats")
        recovery = _require_dict(instrument.get("recovery"), "PHOTONS receipt boundary.recovery")
        boundary_sequence = _require_int(boundary.get("sequence"), "PHOTONS receipt boundary.sequence", minimum=1)
        boundary_reset = _require_int(stats.get("reset_count"), "PHOTONS receipt boundary.reset_count")
        boundary_update = _require_int(stats.get("update_count"), "PHOTONS receipt boundary.update_count", minimum=1)
        if (
            boundary_sequence != boundary_sequence_expected
            or boundary_reset != boundary_reset_expected
            or boundary_update != boundary_update_expected
        ):
            raise RuntimeError("PHOTONS recovery receipt proof identity changed after durability")
        if _require_int(recovery.get("generation"), "PHOTONS receipt generation") != int(generation):
            raise RuntimeError("PHOTONS recovery receipt generation disagrees with proof row")
        if _require_bool(recovery.get("fresh_physical_ancestry"), "PHOTONS receipt fresh ancestry") is not True:
            raise RuntimeError("PHOTONS recovery receipt boundary lacks fresh physical ancestry")
        if exact_n_plus_1 and not (
            boundary_sequence == source_sequence + 1
            and boundary_reset == source_reset
            and boundary_update == source_update + 1
        ):
            raise RuntimeError(
                "PHOTONS recovery receipt boundary is not exact source N+1: "
                f"source={source_sequence}/{source_reset}/{source_update} "
                f"boundary={boundary_sequence}/{boundary_reset}/{boundary_update}"
            )
        if not exact_n_plus_1 and not (
            boundary_sequence > source_sequence
            and boundary_reset == source_reset
            and boundary_update > source_update
        ):
            raise RuntimeError("PHOTONS live-adopt receipt boundary is not a monotonic descendant")

        history_receipt: Optional[Dict[str, Any]] = None
        if isinstance(history, dict):
            history_receipt = {
                "scope": history.get("history_scope"),
                "truncated": bool(history.get("history_truncated")),
                "staged_second_count": int(history.get("staged_second_count") or 0),
                "staged_minute_count": int(history.get("staged_minute_count") or 0),
                "surrendered_second_endpoints": int(
                    history.get("surrendered_second_endpoints") or 0
                ),
                "surrendered_minute_endpoints": int(
                    history.get("surrendered_minute_endpoints") or 0
                ),
            }

        campaign = boundary.get("campaign")
        campaign_receipt = None
        if isinstance(campaign, dict):
            campaign_receipt = {
                "campaign": str(campaign.get("campaign") or ""),
                "public_count": _require_int(
                    campaign.get("public_count"),
                    "PHOTONS receipt campaign.public_count",
                    minimum=1,
                ),
                "start_after_sequence": _require_int(
                    campaign.get("start_after_sequence"),
                    "PHOTONS receipt campaign.start_after_sequence",
                    minimum=1,
                ),
            }

        receipt: Dict[str, Any] = {
            "schema": PHOTONS_RECOVERY_RECEIPT_SCHEMA,
            "subsystem": "PHOTONS",
            "recorded_at_utc": _utc_now_z(),
            "recovery_mode": mode,
            "generation": int(generation),
            "producer_mutated": bool(producer_mutated),
            "source": {
                "detail_id": int(source_id),
                "sequence": int(source_sequence),
                "reset_count": int(source_reset),
                "update_count": int(source_update),
            },
            "checkpoint": checkpoint_summary,
            "boundary": {
                "detail_id": int(boundary_id),
                "sequence": int(boundary_sequence),
                "reset_count": int(boundary_reset),
                "update_count": int(boundary_update),
            },
            "proof": {
                "contract": (
                    "EXACT_SOURCE_N_PLUS_1" if exact_n_plus_1 else "MONOTONIC_LIVE_DESCENDANT"
                ),
                "durable": True,
                "fresh_physical_ancestry": True,
            },
        }
        if history_receipt is not None:
            receipt["history"] = history_receipt
        if campaign_receipt is not None:
            receipt["campaign"] = campaign_receipt

        existing = boundary.get("recovery_receipt")
        if existing is not None:
            if not isinstance(existing, dict):
                raise RuntimeError("PHOTONS recovery boundary has malformed recovery_receipt")
            existing_mode = str(existing.get("recovery_mode") or "").strip().upper()
            mode_compatible = bool(
                existing_mode == mode
                or (mode == "PENDING_RESTORE_PROOF" and existing_mode == "HELD_RESTORE")
            )
            same_identity = bool(
                existing.get("schema") == PHOTONS_RECOVERY_RECEIPT_SCHEMA
                and _require_int(
                    _require_dict(existing.get("source"), "existing PHOTONS receipt source").get("detail_id"),
                    "existing PHOTONS receipt source.detail_id",
                    minimum=1,
                ) == source_id
                and _require_int(
                    _require_dict(existing.get("boundary"), "existing PHOTONS receipt boundary").get("detail_id"),
                    "existing PHOTONS receipt boundary.detail_id",
                    minimum=1,
                ) == boundary_id
                and mode_compatible
            )
            if not same_identity:
                raise RuntimeError(
                    f"PHOTONS recovery boundary detail_id={boundary_id} already has a different receipt"
                )
            return copy.deepcopy(existing)

        cur.execute(
            """
            UPDATE campaign_detail
            SET payload = jsonb_set(payload, '{recovery_receipt}', %s::jsonb, true)
            WHERE id = %s AND campaign_type = %s
            """,
            (json.dumps(receipt, separators=(",", ":")), boundary_id, CAMPAIGN_TYPE_LANTERN),
        )
        if cur.rowcount != 1:
            raise RuntimeError(
                f"PHOTONS recovery receipt did not decorate exactly one boundary row: {boundary_id}"
            )

    logging.info(
        "🧾 [photons/recovery] durable recovery receipt recorded: mode=%s source_id=%d "
        "boundary_id=%d checkpoint_sha256=%s",
        mode,
        int(source_id),
        int(boundary_id),
        checkpoint_summary.get("sha256") or "UNAVAILABLE",
    )
    return receipt


def _photons_recovery_source_row(
    checkpoint: Dict[str, Any],
) -> Optional[Dict[str, Any]]:
    """Return the durable PHOTONS row whose statistics identity owns checkpoint."""
    saved = _normalize_saved_ppb_checkpoint(checkpoint)
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, campaign
            FROM campaign_detail
            WHERE campaign_type = %s
              AND payload #>> '{photons,stats,reset_count}' = %s
              AND payload #>> '{photons,stats,update_count}' = %s
            ORDER BY id DESC
            LIMIT 1
            """,
            (
                CAMPAIGN_TYPE_LANTERN,
                str(int(saved["reset_count"])),
                str(int(saved["update_count"])),
            ),
        )
        row = cur.fetchone()
    return copy.deepcopy(row) if isinstance(row, dict) else None


def _checkpoint_matches_photons_state(
    checkpoint: Dict[str, Any], photons: Dict[str, Any]
) -> bool:
    instrument = photons.get("photons") if isinstance(photons, dict) else None
    stats = instrument.get("stats") if isinstance(instrument, dict) else None
    current = checkpoint.get("current") if isinstance(checkpoint, dict) else None
    if not isinstance(stats, dict) or not isinstance(current, dict):
        return False
    return bool(
        _require_int(stats.get("reset_count"), "PHOTONS.stats.reset_count")
        == _require_int(checkpoint.get("reset_count"), "PHOTONS recovery reset_count")
        and _require_int(stats.get("update_count"), "PHOTONS.stats.update_count", minimum=1)
        == _require_int(checkpoint.get("update_count"), "PHOTONS recovery update_count", minimum=1)
        and _require_int(stats.get("rolling_ppb_current_sequence"), "PHOTONS.stats.rolling_ppb_current_sequence", minimum=1)
        == _require_int(checkpoint.get("current_sequence"), "PHOTONS recovery current_sequence", minimum=1)
        and _require_int(stats.get("lap_count"), "PHOTONS.stats.lap_count")
        == _require_int(current.get("lap_count"), "PHOTONS recovery current.lap_count")
        and _require_int(stats.get("total_lap_gnss_ns"), "PHOTONS.stats.total_lap_gnss_ns")
        == _require_int(current.get("total_lap_gnss_ns"), "PHOTONS recovery current.total_lap_gnss_ns")
    )


def _checkpoint_matches_photons_source(
    checkpoint: Dict[str, Any], source: Dict[str, Any]
) -> bool:
    current = checkpoint.get("current") if isinstance(checkpoint, dict) else None
    if not isinstance(current, dict):
        return False
    return bool(
        _require_int(checkpoint.get("reset_count"), "PHOTONS recovery reset_count")
        == int(source["reset_count"])
        and _require_int(checkpoint.get("update_count"), "PHOTONS recovery update_count", minimum=1)
        == int(source["update_count"])
        and _require_int(current.get("sequence"), "PHOTONS recovery current.sequence", minimum=1)
        == int(source["update_count"])
        and _require_int(current.get("lap_count"), "PHOTONS recovery current.lap_count", minimum=1)
        == int(source["lap_count"])
        and _require_int(current.get("total_lap_gnss_ns"), "PHOTONS recovery current.total_lap_gnss_ns", minimum=1)
        == int(source["total_lap_gnss_ns"])
    )


def _recovery_checkpoint_for_selected_row(
    row: Dict[str, Any], source: Dict[str, Any]
) -> Dict[str, Any]:
    """Load singleton recovery custody, migrating one legacy embedded row if needed."""
    checkpoint = _read_photons_recovery_config()
    if checkpoint is None:
        payload = row.get("payload")
        if isinstance(payload, str):
            payload = json.loads(payload)
        payload = _require_dict(payload, "legacy PHOTONS campaign_detail.payload")
        legacy = payload.get("ppb_restore_checkpoint")
        if not isinstance(legacy, dict):
            raise RuntimeError(
                f"config.{PHOTONS_RECOVERY_CONFIG_KEY} is missing and selected "
                f"detail_id={int(row['id'])} has no legacy embedded checkpoint"
            )
        checkpoint = _normalize_saved_ppb_checkpoint(legacy)
        if not _checkpoint_matches_photons_source(checkpoint, source):
            raise RuntimeError(
                "legacy PHOTONS checkpoint does not match selected durable source"
            )
        with open_db() as conn:
            _write_photons_recovery_config(conn.cursor(), checkpoint)
        logging.warning(
            "📦 [photons/ppb] migrated legacy embedded Better-Buckets checkpoint "
            "from detail_id=%d to config.%s",
            int(row["id"]),
            PHOTONS_RECOVERY_CONFIG_KEY,
        )
    if not _checkpoint_matches_photons_source(checkpoint, source):
        raise RuntimeError(
            f"config.{PHOTONS_RECOVERY_CONFIG_KEY} does not match selected "
            f"detail_id={int(row['id'])}"
        )
    return checkpoint


def _literal_recovery_history_from_source(source: Dict[str, Any]) -> Dict[str, Any]:
    """Return the exact durable PHOTONS history that may lawfully be staged.

    Aggregate N/T, Welfords, exclusions, chronology, and monotonic custody come
    from the selected canonical row. Better-Buckets history comes from the exact
    config.PHOTONS_RECOVERY singleton paired to that row's statistics identity.

    A complete checkpoint restores the complete bounded producer rings.  A
    structurally valid checkpoint that is merely warming after an observation
    gap may instead restore its exact retained suffix.  The missing older ring
    ancestry is explicitly surrendered; SQL is never consulted and no endpoint
    is synthesized.  A lost/empty/contradictory rolling lineage still fails.
    """
    raw = source.get("ppb_restore_checkpoint")
    saved = _normalize_saved_ppb_checkpoint(raw)

    current = _require_dict(saved.get("current"), "saved PHOTONS checkpoint current")
    expected = {
        "reset_count": int(source["reset_count"]),
        "update_count": int(source["update_count"]),
        "sequence": int(source["update_count"]),
        "lap_count": int(source["lap_count"]),
        "total_lap_gnss_ns": int(source["total_lap_gnss_ns"]),
    }
    observed = {
        "reset_count": int(saved["reset_count"]),
        "update_count": int(saved["update_count"]),
        "sequence": int(current.get("sequence") or 0),
        "lap_count": int(current.get("lap_count") or 0),
        "total_lap_gnss_ns": int(current.get("total_lap_gnss_ns") or 0),
    }
    if observed != expected:
        raise RuntimeError(
            "durable PHOTONS literal checkpoint does not close against its source row: "
            f"expected={expected!r} observed={observed!r}"
        )

    checkpoint_status = str(saved.get("status") or "")
    complete = bool(saved.get("recoverable"))
    if not complete and checkpoint_status not in {
        "WARMING_AFTER_OBSERVATION_GAP",
        "WARMING_FOR_COMPLETE_FIRMWARE_HISTORY",
    }:
        raise RuntimeError(
            "selected durable PHOTONS checkpoint cannot support literal-suffix restore: "
            f"detail_id={source.get('db_detail_id')} status={checkpoint_status!r} "
            f"second={saved.get('second_count')}/{saved.get('expected_second_count')} "
            f"minute={saved.get('minute_count')}/{saved.get('expected_minute_count')}"
        )

    second_history = copy.deepcopy(list(saved.get("second_history") or []))
    minute_history = copy.deepcopy(list(saved.get("minute_history") or []))
    expected_second = int(saved.get("expected_second_count") or 0)
    expected_minute = int(saved.get("expected_minute_count") or 0)
    source_update_count = int(source["update_count"])
    origin = copy.deepcopy(saved.get("origin") or _ppb_zero_endpoint())
    second_origin_promoted = False
    minute_origin_promoted = False

    # The exact epoch origin is producer-authored testimony carried separately
    # from the bounded rings.  When the statistical epoch is younger than a
    # window, target_sequence is zero and the origin is the only truthful anchor.
    # Promote that already-durable fact into a partial restore image when useful;
    # this is not reconstruction of any unseen intermediate endpoint.
    if not complete:
        if (
            source_update_count <= PHOTONS_RECOVERY_10_MIN_SECONDS
            and (not second_history or int(second_history[0]["sequence"]) != 0)
            and len(second_history) < expected_second
        ):
            second_history.insert(0, copy.deepcopy(origin))
            second_origin_promoted = True
        if (
            source_update_count <= PHOTONS_RECOVERY_24_HOUR_SECONDS
            and (not minute_history or int(minute_history[0]["sequence"]) != 0)
            and len(minute_history) < expected_minute
        ):
            minute_history.insert(0, copy.deepcopy(origin))
            minute_origin_promoted = True

    if not second_history or not minute_history:
        raise RuntimeError(
            "durable PHOTONS checkpoint has no literal Better-Buckets history to stage"
        )
    if not _ppb_endpoints_equal(second_history[-1], current):
        raise RuntimeError("PHOTONS literal second-history tail is not the source endpoint")

    if len(second_history) > expected_second or len(minute_history) > expected_minute:
        raise RuntimeError(
            "PHOTONS literal restore image exceeds the source producer ring counts"
        )
    missing_second = max(0, expected_second - len(second_history))
    missing_minute = max(0, expected_minute - len(minute_history))
    history_truncated = not complete
    if complete and (missing_second or missing_minute):
        raise RuntimeError("recoverable PHOTONS checkpoint unexpectedly lacks literal ring entries")

    if complete:
        history_scope = "COMPLETE_PRODUCER_RING"
    elif second_origin_promoted or minute_origin_promoted:
        history_scope = "LITERAL_SUFFIX_WITH_EXACT_EPOCH_ORIGIN"
    else:
        history_scope = "LITERAL_SUFFIX_ONLY"

    return {
        "schema": "PHOTONS_LITERAL_PPB_RESTORE_V1",
        "authority": "DURABLE_LITERAL_CHECKPOINT",
        "history_scope": history_scope,
        "history_truncated": history_truncated,
        "source_db_detail_id": int(source["db_detail_id"]),
        "source_reset_count": int(source["reset_count"]),
        "source_update_count": int(source["update_count"]),
        "checkpoint_status": checkpoint_status,
        "checkpoint_seed_source": saved.get("seed_source"),
        "gap_count": int(saved.get("gap_count") or 0),
        "source_expected_second_count": expected_second,
        "source_expected_minute_count": expected_minute,
        "staged_second_count": len(second_history),
        "staged_minute_count": len(minute_history),
        "surrendered_second_endpoints": missing_second,
        "surrendered_minute_endpoints": missing_minute,
        "second_origin_promoted": second_origin_promoted,
        "minute_origin_promoted": minute_origin_promoted,
        "second_history": second_history,
        "minute_history": minute_history,
    }


def _restore_ppb_checkpoint_runtime(
    checkpoint: Dict[str, Any],
    *,
    source_db_detail_id: Optional[int],
    reason: str = "RESTORED_FROM_DURABLE_PHOTONS",
    seed_source: str = "DURABLE_PI_CHECKPOINT",
) -> Dict[str, Any]:
    global _ppb_checkpoint_runtime
    saved = _normalize_saved_ppb_checkpoint(checkpoint)
    with _ppb_checkpoint_lock:
        runtime = _ppb_checkpoint_new_runtime(
            reason=str(reason),
            seed_source=str(seed_source),
            gap_count=int(saved.get("gap_count") or 0),
            last_gap=(
                saved.get("last_gap")
                if isinstance(saved.get("last_gap"), dict)
                else None
            ),
        )
        runtime["reset_count"] = int(saved["reset_count"])
        runtime["last_update_count"] = int(saved["update_count"])
        runtime["last_delta_signature"] = saved.get("last_delta_signature")
        runtime["rolling_sequence"] = int(saved["rolling_sequence"])
        runtime["current_sequence"] = int(saved["current_sequence"])
        runtime["expected_second_count"] = int(saved["expected_second_count"])
        runtime["expected_minute_count"] = int(saved["expected_minute_count"])
        runtime["last_minute_key"] = int(saved["last_minute_key"])
        runtime["origin_valid"] = bool(saved["origin_valid"])
        runtime["origin"] = copy.deepcopy(saved["origin"])
        runtime["current"] = copy.deepcopy(saved["current"])
        runtime["second_history"].extend(copy.deepcopy(saved["second_history"]))
        runtime["minute_history"].extend(copy.deepcopy(saved["minute_history"]))
        runtime["contiguous_from_update_count"] = saved.get(
            "contiguous_from_update_count"
        )
        runtime["seed_source_db_detail_id"] = (
            int(source_db_detail_id) if source_db_detail_id is not None else None
        )
        runtime["proof_checks"] = int(saved.get("proof_checks") or 0)
        runtime["rolling_custody_lost"] = saved.get("status") == "ROLLING_CUSTODY_LOST"
        _ppb_checkpoint_runtime = runtime
        return _ppb_checkpoint_snapshot_locked(runtime)


def _adopt_held_restore_checkpoint_runtime(
    *,
    source: Dict[str, Any],
    literal_history: Dict[str, Any],
) -> Dict[str, Any]:
    """Mirror the exact Better-Buckets rings just committed into a newborn producer.

    Before RECOVERY_COMMIT, Pi runtime describes the durable source producer,
    including that producer's advertised ring counts.  A literal-suffix held
    restore intentionally creates a new producer whose bounded rings contain only
    the endpoints actually staged.  After firmware accepts the commit, rebase the
    Pi's expected ring sizes to that exact installed image before workers consume
    the first N+1 row.

    This does not heal the historical observation gap.  ``last_gap`` and the
    retained suffix remain evidence; only the new producer's ring geometry is
    updated to the state we just proved and installed.
    """
    global _ppb_checkpoint_runtime

    second_history = copy.deepcopy(list(literal_history.get("second_history") or []))
    minute_history = copy.deepcopy(list(literal_history.get("minute_history") or []))
    if not second_history or not minute_history:
        raise RuntimeError("held PHOTONS restore committed without literal ring history")

    with _ppb_checkpoint_lock:
        runtime = _ppb_checkpoint_runtime_ensure_locked()
        current = _require_dict(runtime.get("current"), "held restore Pi checkpoint current")
        expected_identity = {
            "reset_count": int(source["reset_count"]),
            "update_count": int(source["update_count"]),
            "sequence": int(source["update_count"]),
            "lap_count": int(source["lap_count"]),
            "total_lap_gnss_ns": int(source["total_lap_gnss_ns"]),
        }
        observed_identity = {
            "reset_count": int(runtime.get("reset_count") or 0),
            "update_count": int(runtime.get("last_update_count") or 0),
            "sequence": int(current.get("sequence") or 0),
            "lap_count": int(current.get("lap_count") or 0),
            "total_lap_gnss_ns": int(current.get("total_lap_gnss_ns") or 0),
        }
        if observed_identity != expected_identity:
            raise RuntimeError(
                "Pi PHOTONS checkpoint changed while held restore was staged: "
                f"expected={expected_identity!r} observed={observed_identity!r}"
            )
        source_second_history = list(runtime["second_history"])
        source_minute_history = list(runtime["minute_history"])
        expected_origin = copy.deepcopy(runtime.get("origin") or _ppb_zero_endpoint())

        def staged_history_matches_source(
            staged_values: List[Dict[str, Any]],
            source_values: List[Dict[str, Any]],
            *,
            origin_promoted: bool,
        ) -> bool:
            if not origin_promoted:
                return staged_values == source_values
            return bool(
                staged_values
                and _ppb_endpoints_equal(staged_values[0], expected_origin)
                and staged_values[1:] == source_values
            )

        if not staged_history_matches_source(
            second_history,
            source_second_history,
            origin_promoted=bool(literal_history.get("second_origin_promoted")),
        ):
            raise RuntimeError("Pi PHOTONS second history changed while held restore was staged")
        if not staged_history_matches_source(
            minute_history,
            source_minute_history,
            origin_promoted=bool(literal_history.get("minute_origin_promoted")),
        ):
            raise RuntimeError("Pi PHOTONS minute history changed while held restore was staged")

        runtime["second_history"].clear()
        runtime["second_history"].extend(copy.deepcopy(second_history))
        runtime["minute_history"].clear()
        runtime["minute_history"].extend(copy.deepcopy(minute_history))
        runtime["expected_second_count"] = len(second_history)
        runtime["expected_minute_count"] = len(minute_history)
        runtime["seed_source"] = (
            "DURABLE_LITERAL_SUFFIX_HELD_RESTORE"
            if literal_history.get("history_truncated")
            else "DURABLE_COMPLETE_HELD_RESTORE"
        )
        runtime["status_reason"] = (
            "HELD_RESTORE_LITERAL_SUFFIX_INSTALLED"
            if literal_history.get("history_truncated")
            else "HELD_RESTORE_COMPLETE_RING_INSTALLED"
        )
        runtime["seed_source_db_detail_id"] = int(source["db_detail_id"])
        runtime["rolling_custody_lost"] = False
        _ppb_checkpoint_runtime = runtime
        return _ppb_checkpoint_snapshot_locked(runtime)


def _ppb_checkpoint_report_surface() -> Dict[str, Any]:
    with _ppb_checkpoint_lock:
        if _ppb_checkpoint_runtime is None:
            checkpoint = None
        else:
            checkpoint = _ppb_checkpoint_snapshot_locked(_ppb_checkpoint_runtime)
        return {
            "checkpoint": checkpoint,
            "rows_verified": _ppb_checkpoint_rows_verified,
            "recoverable_rows": _ppb_checkpoint_recoverable_rows,
            "warming_rows": _ppb_checkpoint_warming_rows,
            "gap_count": _ppb_checkpoint_gap_count,
            "ingest_failure_count": _ppb_checkpoint_ingest_failure_count,
            "last_ingest_failure": copy.deepcopy(_ppb_checkpoint_last_ingest_failure),
            "live_reacquire_requested": _ppb_checkpoint_reacquire_requested.is_set(),
            "live_reacquire_worker_started": _ppb_checkpoint_reacquire_worker_started.is_set(),
            "live_reacquire_request_count": _ppb_checkpoint_reacquire_request_count,
            "live_reacquire_success_count": _ppb_checkpoint_reacquire_success_count,
            "live_reacquire_failure_count": _ppb_checkpoint_reacquire_failure_count,
            "live_reacquire_last_result": copy.deepcopy(
                _ppb_checkpoint_reacquire_last_result
            ),
            "live_reacquire_last_failure": copy.deepcopy(
                _ppb_checkpoint_reacquire_last_failure
            ),
        }


def _make_photons(
    fragment: Payload, system_context: Dict[str, Any]
) -> Tuple[Payload, Dict[str, Any]]:
    """Form canonical PHOTONS_V1 from firmware testimony + authoritative SYSTEM context."""
    global _ppb_checkpoint_runtime
    global _ppb_checkpoint_ingest_failure_count
    global _ppb_checkpoint_last_ingest_failure

    sequence, publish_count, pps_count, instrument = _validate_photons_fragment(fragment)
    system_context = _require_dict(system_context, "SYSTEM.REPORT.payload")

    state: Payload = {
        "schema": PHOTONS_SCHEMA,
        "source_schema": fragment.get("schema"),
        "sequence": sequence,
        "publish_count": publish_count,
        "pps_count": pps_count,
        "published_at_utc": _utc_now_z(),
    }

    # SYSTEM-owned source payloads remain transitive and structurally intact.
    # Missing optional objects remain explicit empty objects; PHOTONS does not
    # synthesize replacement environmental, GNSS, power, or platform testimony.
    for field in SYSTEM_CONTEXT_FIELDS:
        value = system_context.get(field)
        state[field] = copy.deepcopy(value) if isinstance(value, dict) else {}

    # Preserve the exact Teensy-owned instrument subtree.  Context enrichment
    # never reaches inside or re-authors firmware optical science.
    state["photons"] = copy.deepcopy(instrument)
    stats = _require_dict(instrument.get("stats"), "PHOTONS_FRAGMENT.photons.stats")

    # Fold the producer-authored Better-Buckets delta before publication. The
    # complete literal recovery image is private continuation state committed to
    # config.PHOTONS_RECOVERY; canonical PHOTONS never transports the bounded ring.
    with _ppb_checkpoint_lock:
        checkpoint_before = copy.deepcopy(_ppb_checkpoint_runtime)
        try:
            checkpoint = _ppb_checkpoint_ingest(stats)
        except Exception as exc:
            _ppb_checkpoint_runtime = checkpoint_before
            _ppb_checkpoint_ingest_failure_count += 1
            failure = {
                "schema": "PHOTONS_PPB_CHECKPOINT_INGEST_ERROR_V1",
                "at_utc": _utc_now_z(),
                "sequence": sequence,
                "reset_count": stats.get("reset_count"),
                "update_count": stats.get("update_count"),
                "error": str(exc),
                "failure_count": _ppb_checkpoint_ingest_failure_count,
                "canonical_row_rejected": True,
            }
            _ppb_checkpoint_last_ingest_failure = copy.deepcopy(failure)
            logging.exception(
                "💥 [photons/ppb-checkpoint] literal checkpoint ingest failed; "
                "rejecting canonical PHOTONS row"
            )
            raise
    return state, checkpoint


# ---------------------------------------------------------------------
# Phase 5 durable recovery court
# ---------------------------------------------------------------------

def _recovery_status_set(state: str, **details: Any) -> None:
    global _recovery_status
    with _recovery_lock:
        current = dict(_recovery_status)
        current.update(details)
        current["schema"] = "PHOTONS_PI_RECOVERY_V1"
        current["enabled"] = True
        current["state"] = str(state)
        current["updated_at_utc"] = _utc_now_z()
        _recovery_status = current


def _new_recovery_generation() -> int:
    generation = int(time.monotonic_ns() & 0xFFFFFFFF)
    return generation if generation != 0 else 1


def _load_active_lantern_master() -> Optional[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT
                master.id,
                master.campaign,
                master.ts,
                master.payload,
                baseline.id AS baseline_campaign_id,
                baseline.campaign AS baseline_campaign
            FROM campaign_master AS master
            LEFT JOIN campaign_master AS baseline
              ON baseline.id = (master.payload ->> 'baseline_campaign_id')::bigint
             AND baseline.campaign_type = master.campaign_type
            WHERE master.campaign_type = %s
              AND master.active = true
            ORDER BY master.ts DESC, master.id DESC
            LIMIT 2
            """,
            (CAMPAIGN_TYPE_LANTERN,),
        )
        rows = cur.fetchall()

    if len(rows) > 1:
        raise RuntimeError("multiple active LANTERN campaign masters")
    if not rows:
        return None

    row = rows[0]
    payload = row["payload"]
    if isinstance(payload, str):
        payload = json.loads(payload)
    payload = _require_dict(payload, "active LANTERN campaign_master.payload")
    started_at = payload.get("started_at")
    if not started_at:
        raise RuntimeError("active LANTERN campaign master lacks started_at")

    out: Dict[str, Any] = {
        "campaign": str(row["campaign"]),
        "campaign_id": int(row["id"]),
        "started_at": str(started_at),
        "master_payload": copy.deepcopy(payload),
    }
    if row.get("baseline_campaign_id") is not None:
        out["baseline_campaign_id"] = int(row["baseline_campaign_id"])
    if row.get("baseline_campaign"):
        out["baseline_campaign"] = str(row["baseline_campaign"])
    return out


def _count_lantern_campaign_details(campaign_name: str) -> int:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT COUNT(*) AS count
            FROM campaign_detail
            WHERE campaign_type = %s
              AND campaign = %s
            """,
            (CAMPAIGN_TYPE_LANTERN, campaign_name),
        )
        row = cur.fetchone()
    return int(row["count"] if row else 0)


def _recovery_welford_args(prefix: str, state: Dict[str, Any]) -> Dict[str, Any]:
    return {
        f"{prefix}_n": int(state["n"]),
        f"{prefix}_mean": format(float(state["mean"]), ".17g"),
        f"{prefix}_m2": format(float(state["m2"]), ".17g"),
        f"{prefix}_min": format(float(state["min"]), ".17g"),
        f"{prefix}_max": format(float(state["max"]), ".17g"),
    }


def _canonical_recovery_state_from_row(
    row: Dict[str, Any],
    *,
    active_master: Optional[Dict[str, Any]],
    require_active_campaign: bool,
) -> Dict[str, Any]:
    if not bool(row.get("viable")):
        raise ValueError("campaign_detail.viable is false")

    payload = row.get("payload")
    if isinstance(payload, str):
        payload = json.loads(payload)
    state = copy.deepcopy(_require_dict(payload, "campaign_detail.payload"))
    if state.get("schema") != PHOTONS_SCHEMA:
        raise ValueError(f"unsupported canonical schema {state.get('schema')!r}")

    sequence = _require_int(state.get("sequence"), "PHOTONS.sequence", minimum=1)
    publish_count = _require_int(
        state.get("publish_count"), "PHOTONS.publish_count", minimum=1
    )
    db_sequence = row.get("sequence")
    if db_sequence is not None and _require_int(
        db_sequence, "campaign_detail.sequence", minimum=1
    ) != sequence:
        raise ValueError("campaign_detail sequence does not match canonical payload")
    instrument = _require_dict(state.get("photons"), "PHOTONS.photons")
    if instrument.get("schema") != PHOTONS_INSTRUMENT_SCHEMA:
        raise ValueError("canonical PHOTONS instrument schema mismatch")

    # Measurement-source migration is a hard scientific boundary.  Emulator-era
    # PHOTONS rows are valid historical evidence, but they may never be resurrected
    # as ancestors of the physical 1 kHz race population.  The operator must make
    # the one-time STOP/CLEAR + producer reboot cut before this firmware generation.
    # Refuse legacy durable ancestry rather than silently mixing synthetic laps with
    # real optical flights.
    if instrument.get("source") != "PD200T_REAL_RACE":
        raise ValueError(
            "durable PHOTONS source predates the real-race measurement epoch; "
            "STOP/CLEAR the legacy LANTERN domain and reboot the producer before "
            "starting PD200T_REAL_RACE"
        )
    race = _require_dict(instrument.get("race"), "PHOTONS.photons.race")
    if race.get("schema") != PHOTONS_RACE_SCHEMA:
        raise ValueError("durable PHOTONS race schema mismatch")
    durable_race_geometry = (
        _require_int(race.get("cadence_hz"), "PHOTONS.photons.race.cadence_hz", minimum=1),
        _require_int(race.get("cadence_ns"), "PHOTONS.photons.race.cadence_ns", minimum=1),
        _require_int(race.get("pulse_ns"), "PHOTONS.photons.race.pulse_ns", minimum=1),
    )
    if durable_race_geometry != (
        PHOTONS_RACE_CADENCE_HZ,
        PHOTONS_RACE_CADENCE_NS,
        PHOTONS_RACE_PULSE_NS,
    ):
        raise ValueError(
            "durable PHOTONS race geometry does not match this firmware epoch: "
            f"durable={durable_race_geometry!r}"
        )
    if (
        race.get("launch_surrogate") != "LD_ON_FALLING_EDGE"
        or race.get("flight_interpretation") != "ESTIMATED"
    ):
        raise ValueError("durable PHOTONS race launch/interpretation contract changed")

    if _require_bool(instrument.get("snapshot_ok"), "PHOTONS.photons.snapshot_ok") is not True:
        raise ValueError("canonical PHOTONS snapshot is not coherent")
    # Top-level instrument.valid is live measurement-readiness testimony, not
    # durable recovery authority.  In particular, the exact N+1 row after a
    # hard restore may truthfully publish valid=false while the fresh projection
    # anchor is still reacquiring.  Preserve/type-check that testimony here, but
    # let the explicit stats, custody, Better-Buckets, chronology, and physical-
    # ancestry courts below decide whether the row is resurrection-capable.
    _require_bool(instrument.get("valid"), "PHOTONS.photons.valid")

    stats = _require_dict(instrument.get("stats"), "PHOTONS.photons.stats")
    science = _require_dict(instrument.get("science"), "PHOTONS.photons.science")
    projection = _require_dict(
        instrument.get("projection"), "PHOTONS.photons.projection"
    )
    if stats.get("schema") != PHOTONS_STATS_SCHEMA:
        raise ValueError("canonical PHOTONS stats schema mismatch")
    if science.get("schema") != PHOTONS_SCIENCE_SCHEMA:
        raise ValueError("canonical PHOTONS science schema mismatch")
    if _require_bool(stats.get("valid"), "PHOTONS.photons.stats.valid") is not True:
        raise ValueError("canonical PHOTONS statistics are invalid")

    recovery = instrument.get("recovery")
    if recovery is not None:
        recovery = _require_dict(recovery, "PHOTONS.photons.recovery")
        if not _require_bool(
            recovery.get("fresh_physical_ancestry"),
            "PHOTONS.photons.recovery.fresh_physical_ancestry",
        ):
            raise ValueError("durable PHOTONS row lacks fresh physical ancestry")
        for field in (
            "raw_lap_ring_restored",
            "partial_lap_restored",
            "pending_seed_restored",
            "predictor_restored",
            "in_flight_train_restored",
        ):
            if _require_bool(
                recovery.get(field), f"PHOTONS.photons.recovery.{field}"
            ):
                raise ValueError(f"durable PHOTONS row illegally restored {field}")

    standard_lap_ps = _require_int(
        stats.get("standard_lap_ps"), "PHOTONS.photons.stats.standard_lap_ps", minimum=1
    )
    # The operator reference is not physical ancestry. Legacy durable rows may
    # predate femtosecond baseline testimony; their whole-ps mirror remains enough
    # to verify their own historical PPB while recovery restores only raw N/T.
    source_lap_baseline_fs = (
        _require_int(stats.get("lap_baseline_fs"), "PHOTONS.photons.stats.lap_baseline_fs", minimum=1)
        if stats.get("lap_baseline_fs") is not None
        else standard_lap_ps * 1000
    )

    reset_count = _require_int(
        stats.get("reset_count"), "PHOTONS.photons.stats.reset_count"
    )
    update_count = _require_int(
        stats.get("update_count"), "PHOTONS.photons.stats.update_count", minimum=1
    )
    lap_count = _require_int(
        stats.get("lap_count"), "PHOTONS.photons.stats.lap_count", minimum=1
    )
    total_ns = _require_int(
        stats.get("total_lap_gnss_ns"),
        "PHOTONS.photons.stats.total_lap_gnss_ns",
        minimum=1,
    )
    custody_lap_count = _require_int(
        stats.get("custody_lap_count"),
        "PHOTONS.photons.stats.custody_lap_count",
        minimum=lap_count,
    )
    custody_total_ns = _require_int(
        stats.get("custody_total_lap_gnss_ns"),
        "PHOTONS.photons.stats.custody_total_lap_gnss_ns",
        minimum=total_ns,
    )
    recorded_mean = _require_float(
        stats.get("mean_lap_ns"), "PHOTONS.photons.stats.mean_lap_ns"
    )
    ratio_mean = float(total_ns) / float(lap_count)
    if abs(recorded_mean - ratio_mean) > PHOTONS_RECOVERY_VERIFY_TOLERANCE:
        raise ValueError("canonical PHOTONS mean does not close with N/T")

    stats_lap = _validate_recovery_welford(
        stats.get("lap_time"), "PHOTONS.photons.stats.lap_time"
    )
    accepted = _require_dict(science.get("accepted"), "PHOTONS.science.accepted")
    excluded = _require_dict(science.get("excluded"), "PHOTONS.science.excluded")
    accepted_count = _require_int(
        accepted.get("count"), "PHOTONS.science.accepted.count", minimum=1
    )
    excluded_count = _require_int(
        excluded.get("count"), "PHOTONS.science.excluded.count"
    )
    accepted_raw = _validate_recovery_welford(
        accepted.get("raw_cycles"), "PHOTONS.science.accepted.raw_cycles"
    )
    accepted_projected = _validate_recovery_welford(
        accepted.get("projected_lap_ns"),
        "PHOTONS.science.accepted.projected_lap_ns",
    )
    excluded_raw = _validate_recovery_welford(
        excluded.get("raw_cycles"), "PHOTONS.science.excluded.raw_cycles"
    )
    excluded_projected = _validate_recovery_welford(
        excluded.get("projected_lap_ns"),
        "PHOTONS.science.excluded.projected_lap_ns",
    )
    if not _welford_equivalent(stats_lap, accepted_projected):
        raise ValueError("stats and accepted projected-lap Welfords differ")
    if (
        accepted_count != lap_count
        or accepted_raw["n"] != accepted_count
        or accepted_projected["n"] != accepted_count
        or excluded_raw["n"] != excluded_count
        or excluded_projected["n"] > excluded_count
    ):
        raise ValueError("canonical PHOTONS Welford population accounting fails")

    # N/T is exact integer sufficient state followed by one division.  The Welford
    # mean reaches the same mathematical quantity through millions of incremental
    # floating-point updates.  Their tiny numerical drift is corroborating evidence,
    # not a lawful reason to discard newer durable ancestry.
    welford_grand_ratio = _welford_grand_ratio_diagnostic(
        welford_mean_ns=float(accepted_projected["mean"]),
        ratio_mean_ns=ratio_mean,
        lap_baseline_fs=source_lap_baseline_fs,
    )

    seed_pending = _require_bool(
        science.get("seed_pending"), "PHOTONS.science.seed_pending"
    )
    pending_count = 1 if seed_pending else 0
    candidate_count = _require_int(
        science.get("candidate_count"), "PHOTONS.science.candidate_count"
    )
    if candidate_count != accepted_count + excluded_count + pending_count:
        raise ValueError("canonical PHOTONS candidate accounting fails")

    reasons = _require_dict(
        science.get("exclusion_reasons"), "PHOTONS.science.exclusion_reasons"
    )
    projection_invalid = _require_int(
        reasons.get("projection_invalid"), "PHOTONS.reasons.projection_invalid"
    )
    seed_disagreement = _require_int(
        reasons.get("seed_disagreement"), "PHOTONS.reasons.seed_disagreement"
    )
    raw_cycle_excursion = _require_int(
        reasons.get("raw_cycle_excursion"), "PHOTONS.reasons.raw_cycle_excursion"
    )
    if projection_invalid + seed_disagreement + raw_cycle_excursion != excluded_count:
        raise ValueError("canonical PHOTONS exclusion reasons do not close")

    attempts = _require_int(
        projection.get("attempt_count"), "PHOTONS.projection.attempt_count"
    )
    successes = _require_int(
        projection.get("success_count"), "PHOTONS.projection.success_count"
    )
    rejects = _require_int(
        projection.get("reject_count"), "PHOTONS.projection.reject_count"
    )
    if (
        attempts != candidate_count
        or attempts != successes + rejects
        or rejects != projection_invalid
        or successes != accepted_count + excluded_projected["n"] + pending_count
    ):
        raise ValueError("canonical PHOTONS projection accounting fails")

    current_sequence = _require_int(
        stats.get("rolling_ppb_current_sequence"),
        "PHOTONS.stats.rolling_ppb_current_sequence",
        minimum=1,
    )
    endpoint_admitted = _require_bool(
        stats.get("rolling_ppb_endpoint_admitted"),
        "PHOTONS.stats.rolling_ppb_endpoint_admitted",
    )
    if not endpoint_admitted or current_sequence != update_count:
        raise ValueError("canonical PHOTONS row is not a lawful PPB endpoint")

    total_bucket = _require_dict(
        _require_dict(stats.get("ppb_buckets"), "PHOTONS.stats.ppb_buckets").get("total"),
        "PHOTONS.stats.ppb_buckets.total",
    )
    total_bucket_n = _require_int(
        total_bucket.get("sample_count"), "PHOTONS.stats.ppb_buckets.total.sample_count"
    )
    total_bucket_ppb = _require_float(
        total_bucket.get("ppb"), "PHOTONS.stats.ppb_buckets.total.ppb"
    )
    source_ppb_semantics = stats.get("ppb_semantics")
    if source_ppb_semantics is None:
        # Pre-normalization durable rows used conventional fractional-lap PPB.
        # Their N/T and endpoint rings remain valid physical custody, so prove
        # the historical derived value under its own semantics and then restore
        # only the baseline-independent sufficient state.
        expected_total_ppb = (
            ((ratio_mean * 1_000_000.0) / float(source_lap_baseline_fs)) - 1.0
        ) * 1.0e9
    elif source_ppb_semantics == PHOTONS_PPB_SEMANTICS:
        expected_total_ppb = ratio_mean - (float(source_lap_baseline_fs) / 1_000_000.0)
    else:
        raise ValueError(
            f"unsupported durable PHOTONS PPB semantics {source_ppb_semantics!r}"
        )
    if total_bucket_n != lap_count or abs(total_bucket_ppb - expected_total_ppb) > PHOTONS_RECOVERY_VERIFY_TOLERANCE:
        raise ValueError("canonical PHOTONS TOTAL PPB does not close")

    campaign_restore: Optional[Dict[str, Any]] = None
    campaign = state.get("campaign")
    durable_campaign = str(row.get("campaign") or "")
    canonical_campaign = (
        str(campaign.get("campaign") or "") if isinstance(campaign, dict) else ""
    )
    if durable_campaign != canonical_campaign:
        raise ValueError("campaign_detail label does not match canonical campaign")
    if require_active_campaign:
        if active_master is None:
            raise ValueError("active campaign recovery requested without campaign master")
        campaign = _require_dict(campaign, "PHOTONS.campaign")
        if str(campaign.get("campaign") or "") != active_master["campaign"]:
            raise ValueError("canonical PHOTONS campaign does not match active master")
        campaign_id = _require_int(
            campaign.get("campaign_id"), "PHOTONS.campaign.campaign_id", minimum=1
        )
        if campaign_id != int(active_master["campaign_id"]):
            raise ValueError("canonical PHOTONS campaign ID does not match active master")
        if _require_bool(campaign.get("final"), "PHOTONS.campaign.final"):
            raise ValueError("active LANTERN recovery source is already final")
        start_after = _require_int(
            campaign.get("start_after_sequence"),
            "PHOTONS.campaign.start_after_sequence",
            minimum=1,
        )
        public_count = _require_int(
            campaign.get("public_count"), "PHOTONS.campaign.public_count", minimum=1
        )
        if start_after >= sequence:
            raise ValueError("active LANTERN start boundary is not before source row")
        campaign_stats = _require_dict(
            campaign.get("stats"), "PHOTONS.campaign.stats"
        )
        campaign_laps = _require_int(
            campaign_stats.get("lap_count"), "PHOTONS.campaign.stats.lap_count", minimum=1
        )
        campaign_total = _require_int(
            campaign_stats.get("total_lap_gnss_ns"),
            "PHOTONS.campaign.stats.total_lap_gnss_ns",
            minimum=1,
        )
        sample_count = _require_int(
            campaign_stats.get("sample_count"),
            "PHOTONS.campaign.stats.sample_count",
            minimum=1,
        )
        if sample_count != campaign_laps or campaign_laps > custody_lap_count or campaign_total > custody_total_ns:
            raise ValueError("active LANTERN campaign population is invalid")
        campaign_restore = {
            "campaign": active_master["campaign"],
            "campaign_id": int(active_master["campaign_id"]),
            "start_after_sequence": start_after,
            "public_count": public_count,
            "lap_count": campaign_laps,
            "total_lap_gnss_ns": campaign_total,
            "origin_lap_count": custody_lap_count - campaign_laps,
            "origin_total_lap_gnss_ns": custody_total_ns - campaign_total,
        }

    restore_args: Dict[str, Any] = {
        "restore_schema_version": PHOTONS_RECOVERY_SCHEMA_VERSION,
        "source_sequence": sequence,
        "source_publish_count": publish_count,
        "source_reset_count": reset_count,
        "source_update_count": update_count,
        "standard_lap_ps": int(_standard_lap_ps or 0),
        "stats_lap_count": lap_count,
        "stats_total_lap_gnss_ns": total_ns,
        "custody_lap_count": custody_lap_count,
        "custody_total_lap_gnss_ns": custody_total_ns,
        "accepted_count": accepted_count,
        "excluded_count": excluded_count,
        "projection_invalid": projection_invalid,
        "seed_disagreement": seed_disagreement,
        "raw_cycle_excursion": raw_cycle_excursion,
        "dropped_pending_seed_count": pending_count,
        "campaign_active": campaign_restore is not None,
    }
    restore_args.update(_recovery_welford_args("accepted_projected", accepted_projected))
    restore_args.update(_recovery_welford_args("accepted_raw", accepted_raw))
    restore_args.update(_recovery_welford_args("excluded_raw", excluded_raw))
    restore_args.update(_recovery_welford_args("excluded_projected", excluded_projected))
    if campaign_restore is not None:
        restore_args.update(
            {
                "campaign": campaign_restore["campaign"],
                "campaign_origin_lap_count": campaign_restore["origin_lap_count"],
                "campaign_origin_total_lap_gnss_ns": campaign_restore[
                    "origin_total_lap_gnss_ns"
                ],
                "campaign_start_after_sequence": campaign_restore[
                    "start_after_sequence"
                ],
                "campaign_public_count": campaign_restore["public_count"],
                "campaign_lap_count": campaign_restore["lap_count"],
                "campaign_total_lap_gnss_ns": campaign_restore[
                    "total_lap_gnss_ns"
                ],
            }
        )

    return {
        "canonical": state,
        "db_detail_id": int(row["id"]),
        "db_ts": str(row.get("ts") or ""),
        "sequence": sequence,
        "publish_count": publish_count,
        "reset_count": reset_count,
        "update_count": update_count,
        "lap_count": lap_count,
        "total_lap_gnss_ns": total_ns,
        "custody_lap_count": custody_lap_count,
        "custody_total_lap_gnss_ns": custody_total_ns,
        "welford_grand_ratio_diagnostic": welford_grand_ratio,
        "restore_args": restore_args,
        "campaign_restore": campaign_restore,
        "pending_seed_dropped": pending_count,
    }


@dataclass(frozen=True)
class _PhotonsRecoverySnapshot:
    """One explicit PHOTONS resurrection image at a durable one-second boundary.

    This object deliberately binds the canonical PHOTONS_V1 observation to the
    exact config.PHOTONS_RECOVERY singleton that owns its bounded Better-Buckets
    continuation state. ``prepared_restore`` is the existing validated projection
    consumed by the current executor; keeping it inside this object makes the
    canonical row + sidecar the one named recovery authority without changing the
    durable representation or recovery behavior in this step.
    """

    schema: str
    source_detail_id: int
    source_sequence: int
    source_campaign: Optional[str]
    canonical: Dict[str, Any]
    ppb_restore_checkpoint: Dict[str, Any]
    campaign: Optional[Dict[str, Any]]
    prepared_restore: Dict[str, Any]

    def restore_source(self) -> Dict[str, Any]:
        source = copy.deepcopy(self.prepared_restore)
        source["ppb_restore_checkpoint"] = copy.deepcopy(
            self.ppb_restore_checkpoint
        )
        return source


def _photons_recovery_snapshot_from_row(
    row: Dict[str, Any],
    *,
    active_master: Optional[Dict[str, Any]],
    require_active_campaign: bool,
) -> _PhotonsRecoverySnapshot:
    """Bind one canonical PHOTONS row to its exact singleton recovery sidecar."""
    source = _canonical_recovery_state_from_row(
        row,
        active_master=active_master,
        require_active_campaign=require_active_campaign,
    )
    checkpoint = _recovery_checkpoint_for_selected_row(row, source)
    if not _checkpoint_matches_photons_source(checkpoint, source):
        raise RuntimeError(
            f"config.{PHOTONS_RECOVERY_CONFIG_KEY} does not own selected "
            f"detail_id={int(source['db_detail_id'])}"
        )

    canonical = _require_dict(source.get("canonical"), "PHOTONS recovery canonical")
    campaign_raw = canonical.get("campaign")
    if campaign_raw is not None and not isinstance(campaign_raw, dict):
        raise RuntimeError(
            f"PHOTONS recovery source detail_id={int(source['db_detail_id'])} "
            "has malformed campaign decoration"
        )
    source_campaign_raw = row.get("campaign")
    source_campaign = (
        str(source_campaign_raw) if source_campaign_raw is not None else None
    )

    return _PhotonsRecoverySnapshot(
        schema=PHOTONS_RECOVERY_SNAPSHOT_SCHEMA,
        source_detail_id=int(source["db_detail_id"]),
        source_sequence=int(source["sequence"]),
        source_campaign=source_campaign,
        canonical=copy.deepcopy(canonical),
        ppb_restore_checkpoint=copy.deepcopy(checkpoint),
        campaign=copy.deepcopy(campaign_raw) if isinstance(campaign_raw, dict) else None,
        prepared_restore=copy.deepcopy(source),
    )


def _project_photons_recovery_snapshot(
    snapshot: _PhotonsRecoverySnapshot,
    now: datetime,
) -> Dict[str, Any]:
    """Purely translate one durable PHOTONS snapshot to desired state at ``now``.

    PHOTONS has no lawful elapsed-time synthesis for unobserved optical laps.
    Therefore the producer restoration image is intentionally identity-translated:
    aggregate statistics, custody counters, Welfords, LANTERN coordinates, and the
    literal Better-Buckets sidecar remain exactly as captured.  ``now`` names the
    requested convergence instant but does not manufacture physical ancestry.
    """
    if not isinstance(snapshot, _PhotonsRecoverySnapshot):
        raise TypeError("PHOTONS recovery projection requires _PhotonsRecoverySnapshot")
    if not isinstance(now, datetime) or now.tzinfo is None:
        raise ValueError("PHOTONS recovery projection requires timezone-aware now")

    now_utc = now.astimezone(timezone.utc)
    source = snapshot.restore_source()
    restore_args = source.get("restore_args")
    if not isinstance(restore_args, dict):
        raise ValueError("PHOTONS recovery snapshot lacks producer restore_args")

    return {
        "schema": PHOTONS_RECOVERY_DESIRED_STATE_SCHEMA,
        "source_snapshot_schema": snapshot.schema,
        "source_detail_id": int(snapshot.source_detail_id),
        "source_sequence": int(snapshot.source_sequence),
        "source_campaign": snapshot.source_campaign,
        "projected_at_utc": now_utc.isoformat().replace("+00:00", "Z"),
        "time_translation": "IDENTITY_NO_UNOBSERVED_OPTICAL_ANCESTRY",
        "restore_source": source,
        "producer_restore_args": copy.deepcopy(restore_args),
        "campaign": copy.deepcopy(snapshot.campaign),
        "ppb_restore_checkpoint": copy.deepcopy(snapshot.ppb_restore_checkpoint),
    }


def _load_newest_recoverable_photons_state(
    *,
    active_master: Optional[Dict[str, Any]],
    require_active_campaign: bool,
) -> Tuple[Optional[_PhotonsRecoverySnapshot], List[Dict[str, Any]], int]:
    """Return the newest declarative PHOTONS snapshot or fail on contradiction.

    Phase 3 forbids ancestry search-by-convenience.  Once durable PHOTONS history
    exists, a newer row that fails the recovery court is evidence of a contradiction,
    not permission to walk backward until an older row happens to pass.
    """
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        if require_active_campaign:
            assert active_master is not None
            cur.execute(
                """
                SELECT id, ts, campaign, viable, payload, sequence, pps_count
                FROM campaign_detail
                WHERE campaign_type = %s
                  AND campaign = %s
                  AND payload #>> '{photons,source}' = %s
                ORDER BY id DESC
                LIMIT 1
                """,
                (CAMPAIGN_TYPE_LANTERN, active_master["campaign"], "PD200T_REAL_RACE"),
            )
        else:
            cur.execute(
                """
                SELECT id, ts, campaign, viable, payload, sequence, pps_count
                FROM campaign_detail
                WHERE campaign_type = %s
                  AND payload #>> '{photons,source}' = %s
                ORDER BY id DESC
                LIMIT 1
                """,
                (CAMPAIGN_TYPE_LANTERN, "PD200T_REAL_RACE"),
            )
        row = cur.fetchone()

    if row is None:
        return None, [], 0

    try:
        snapshot = _photons_recovery_snapshot_from_row(
            row,
            active_master=active_master,
            require_active_campaign=require_active_campaign,
        )
    except Exception as exc:
        rejection = {"id": int(row["id"]), "reason": str(exc)}
        raise RuntimeError(
            "newest durable PHOTONS row failed recovery authority; refusing "
            "silent fallback to older scientific history: "
            f"{rejection!r}"
        ) from exc

    diagnostic = snapshot.prepared_restore.get("welford_grand_ratio_diagnostic")
    if isinstance(diagnostic, dict):
        log = logging.warning if diagnostic.get("notable") else logging.info
        log(
            "%s [photons/recovery] selected newest durable source id=%d update=%d "
            "Welford/grand-ratio drift=%.9f ppb limit=%.6f ppb; "
            "diagnostic only, authority unchanged",
            "⚠️" if diagnostic.get("notable") else "🧮",
            int(snapshot.source_detail_id),
            int(snapshot.prepared_restore["update_count"]),
            float(diagnostic.get("delta_ppb") or 0.0),
            float(diagnostic.get("diagnostic_limit_ppb") or 0.0),
        )
    return snapshot, [], 1


# ---------------------------------------------------------------------
# Phase 5 startup orchestration + durable proof custody
# ---------------------------------------------------------------------

def _count_current_lantern_details() -> int:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            "SELECT COUNT(*) AS count FROM campaign_detail WHERE campaign_type = %s",
            (CAMPAIGN_TYPE_LANTERN,),
        )
        row = cur.fetchone()
    return int(row["count"] if row else 0)


def _count_current_real_race_details() -> int:
    """Count durable rows belonging to the current physical race source epoch."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT COUNT(*) AS count
            FROM campaign_detail
            WHERE campaign_type = %s
              AND payload #>> '{photons,source}' = %s
            """,
            (CAMPAIGN_TYPE_LANTERN, "PD200T_REAL_RACE"),
        )
        row = cur.fetchone()
    return int(row["count"] if row else 0)


def _retire_legacy_recovery_for_real_race_epoch(
    *, legacy_detail_count: int
) -> Dict[str, Any]:
    """Cut incompatible continuation custody while preserving historical rows."""
    if legacy_detail_count <= 0:
        raise RuntimeError("real-race source migration requires legacy durable history")

    with open_db() as conn:
        cur = conn.cursor()
        cur.execute(
            "DELETE FROM config WHERE config_key = %s",
            (PHOTONS_RECOVERY_CONFIG_KEY,),
        )
        deleted = int(cur.rowcount or 0)
        if deleted > 1:
            raise RuntimeError(
                f"config.{PHOTONS_RECOVERY_CONFIG_KEY} is not a singleton: rows={deleted}"
            )

    migration = {
        "schema": "PHOTONS_SOURCE_EPOCH_MIGRATION_V1",
        "from_source": "LEGACY_EMULATOR_OR_PRE_RACE",
        "to_source": "PD200T_REAL_RACE",
        "historical_detail_rows_preserved": int(legacy_detail_count),
        "legacy_recovery_config_deleted": bool(deleted),
        "producer_action": "COLD_START_NEW_MEASUREMENT_EPOCH",
    }
    logging.warning(
        "🔬 [photons/recovery] source epoch migration: preserving %d historical "
        "LANTERN detail row(s), retiring legacy config.%s=%s, and cold-starting "
        "PD200T_REAL_RACE; no synthetic statistics will enter the new population",
        int(legacy_detail_count),
        PHOTONS_RECOVERY_CONFIG_KEY,
        bool(deleted),
    )
    return migration


def _send_teensy_recovery_command(
    command: str,
    *,
    args: Optional[Dict[str, Any]] = None,
    accepted_statuses: set[str],
) -> Dict[str, Any]:
    if args is None:
        response = send_command(
            machine="TEENSY",
            subsystem=SUBSYSTEM,
            command=command,
        )
    else:
        response = send_command(
            machine="TEENSY",
            subsystem=SUBSYSTEM,
            command=command,
            args=args,
        )
    payload = response.get("payload") if isinstance(response, dict) else None
    status = str(payload.get("status") or "") if isinstance(payload, dict) else ""
    if (
        not isinstance(response, dict)
        or not response.get("success")
        or not isinstance(payload, dict)
        or status not in accepted_statuses
    ):
        raise RuntimeError(
            f"Teensy PHOTONS.{command} rejected: status={status!r} response={response!r}"
        )
    return copy.deepcopy(payload)


def _fetch_teensy_recovery_report(
    *, require_baseline: bool = True
) -> Dict[str, Any]:
    response = send_command(
        machine="TEENSY",
        subsystem=SUBSYSTEM,
        command="REPORT_RECOVERY",
        retries=1,
        retry_delay_s=0.0,
    )
    payload = response.get("payload") if isinstance(response, dict) else None
    if not isinstance(response, dict) or not response.get("success") or not isinstance(payload, dict):
        raise RuntimeError(f"Teensy PHOTONS.REPORT_RECOVERY unavailable: {response!r}")
    if payload.get("schema") != "PHOTONS_RECOVERY_REPORT_V1":
        raise RuntimeError(
            f"unsupported Teensy PHOTONS recovery schema {payload.get('schema')!r}"
        )
    if _require_int(
        payload.get("restore_schema_version"),
        "PHOTONS.REPORT_RECOVERY.restore_schema_version",
    ) != PHOTONS_RECOVERY_SCHEMA_VERSION:
        raise RuntimeError("Teensy PHOTONS recovery schema version mismatch")
    baseline_configured = _require_bool(
        payload.get("standard_lap_configured"),
        "PHOTONS.REPORT_RECOVERY.standard_lap_configured",
    )
    if require_baseline and not baseline_configured:
        raise RuntimeError("Teensy PHOTONS recovery report lacks STANDARD_LAP_NS")
    baseline_fs = _require_int(
        payload.get("lap_baseline_fs"),
        "PHOTONS.REPORT_RECOVERY.lap_baseline_fs",
    )
    if baseline_configured:
        if baseline_fs <= 0:
            raise RuntimeError("Teensy PHOTONS recovery report has invalid LAP_BASELINE_NS")
        if _lap_baseline_fs is not None and baseline_fs != int(_lap_baseline_fs):
            raise RuntimeError(
                "Teensy/config PHOTONS LAP_BASELINE_NS mismatch during recovery: "
                f"teensy_fs={baseline_fs} config_fs={_lap_baseline_fs}"
            )
    elif baseline_fs != 0:
        raise RuntimeError(
            "Teensy PHOTONS recovery report publishes LAP_BASELINE_NS before configuration"
        )
    for field in (
        "publication_started",
        "staging_active",
        "restored",
        "proof_pending",
        "proof_committed",
        "proof_advanced_published",
        "fresh_physical_ancestry",
        "raw_lap_ring_restored",
        "partial_lap_restored",
        "pending_seed_restored",
        "predictor_restored",
        "in_flight_train_restored",
    ):
        _require_bool(payload.get(field), f"PHOTONS.REPORT_RECOVERY.{field}")
    for field in (
        "raw_lap_ring_restored",
        "partial_lap_restored",
        "pending_seed_restored",
        "predictor_restored",
        "in_flight_train_restored",
    ):
        if payload[field]:
            raise RuntimeError(f"Teensy recovery illegally restored {field}")
    for field in (
        "generation",
        "source_sequence",
        "source_publish_count",
        "source_reset_count",
        "source_update_count",
        "proof_sequence",
        "proof_update_count",
        "fragment_sequence",
        "publish_count",
        "stats_reset_count",
        "stats_update_count",
        "campaign_public_count",
    ):
        _require_int(payload.get(field), f"PHOTONS.REPORT_RECOVERY.{field}")
    for field in (
        "source_lap_count",
        "source_total_lap_gnss_ns",
        "source_custody_lap_count",
        "source_custody_total_lap_gnss_ns",
        "stats_lap_count",
        "stats_total_lap_gnss_ns",
        "custody_lap_count",
        "custody_total_lap_gnss_ns",
    ):
        _require_int(payload.get(field), f"PHOTONS.REPORT_RECOVERY.{field}")
    if payload["publication_started"] and not payload["fresh_physical_ancestry"]:
        raise RuntimeError("live PHOTONS recovery report lacks fresh physical ancestry")
    return copy.deepcopy(payload)


def _fetch_teensy_photons_report() -> Dict[str, Any]:
    response = send_command(
        machine="TEENSY",
        subsystem=SUBSYSTEM,
        command="REPORT",
        retries=1,
        retry_delay_s=0.0,
    )
    payload = response.get("payload") if isinstance(response, dict) else None
    if not isinstance(response, dict) or not response.get("success") or not isinstance(payload, dict):
        raise RuntimeError(f"Teensy PHOTONS.REPORT unavailable: {response!r}")
    if payload.get("schema") != "PHOTONS_REPORT_V3":
        raise RuntimeError(
            f"unsupported Teensy PHOTONS broad-report schema {payload.get('schema')!r}; "
            "PD200T_REAL_RACE firmware is required"
        )
    if (
        _require_int(payload.get("race_cadence_hz"), "PHOTONS.REPORT.race_cadence_hz", minimum=1)
        != PHOTONS_RACE_CADENCE_HZ
        or _require_int(payload.get("race_pulse_ns"), "PHOTONS.REPORT.race_pulse_ns", minimum=1)
        != PHOTONS_RACE_PULSE_NS
        or payload.get("race_launch_surrogate") != "LD_ON_FALLING_EDGE"
    ):
        raise RuntimeError("Teensy PHOTONS real-race geometry does not match Pi contract")
    _require_bool(payload.get("race_engine_active"), "PHOTONS.REPORT.race_engine_active")
    if not _require_bool(
        payload.get("standard_lap_configured"),
        "PHOTONS.REPORT.standard_lap_configured",
    ):
        raise RuntimeError("Teensy PHOTONS broad report lacks STANDARD_LAP_NS")
    baseline_fs = _require_int(
        payload.get("lap_baseline_fs"), "PHOTONS.REPORT.lap_baseline_fs", minimum=1
    )
    if _lap_baseline_fs is None or baseline_fs != int(_lap_baseline_fs):
        raise RuntimeError("Teensy PHOTONS broad-report LAP_BASELINE_NS mismatch")
    return copy.deepcopy(payload)


def _ppb_export_endpoint_from_payload(
    payload: Dict[str, Any], prefix: str
) -> Dict[str, Any]:
    endpoint = {
        "sequence": _require_int(
            payload.get(f"{prefix}_sequence"),
            f"PHOTONS.PPB_EXPORT.{prefix}_sequence",
        ),
        "lap_count": _require_int(
            payload.get(f"{prefix}_lap_count"),
            f"PHOTONS.PPB_EXPORT.{prefix}_lap_count",
        ),
        "total_lap_gnss_ns": _require_int(
            payload.get(f"{prefix}_total_lap_gnss_ns"),
            f"PHOTONS.PPB_EXPORT.{prefix}_total_lap_gnss_ns",
        ),
    }
    if (endpoint["lap_count"] == 0) != (endpoint["total_lap_gnss_ns"] == 0):
        raise RuntimeError(
            f"PHOTONS Better-Buckets export {prefix} has inconsistent zero N/T"
        )
    return endpoint


def _fetch_teensy_ppb_export_meta() -> Dict[str, Any]:
    response = send_command(
        machine="TEENSY",
        subsystem=SUBSYSTEM,
        command="PPB_EXPORT_META",
    )
    payload = response.get("payload") if isinstance(response, dict) else None
    payload = payload if isinstance(payload, dict) else {}
    if (
        not isinstance(response, dict)
        or not response.get("success")
        or payload.get("status") != "ppb_export_ready"
        or payload.get("schema") != "PHOTONS_PPB_FULL_RING_EXPORT_V1"
        or payload.get("read_only") is not True
    ):
        raise RuntimeError(f"Teensy PHOTONS Better-Buckets export META failed: {response!r}")

    meta: Dict[str, Any] = dict(payload)
    for key in (
        "reset_count",
        "update_count",
        "current_sequence",
        "second_count",
        "minute_count",
        "second_oldest_sequence",
        "second_newest_sequence",
        "minute_oldest_sequence",
        "minute_newest_sequence",
        "last_minute_key",
        "chunk_max_endpoints",
    ):
        meta[key] = _require_int(
            payload.get(key), f"PHOTONS.PPB_EXPORT_META.{key}"
        )

    lap_baseline_fs = _require_int(
        payload.get("lap_baseline_fs"),
        "PHOTONS.PPB_EXPORT_META.lap_baseline_fs",
        minimum=1,
    )
    if _lap_baseline_fs is None or lap_baseline_fs != int(_lap_baseline_fs):
        raise RuntimeError(
            "PHOTONS Better-Buckets export LAP_BASELINE_NS disagrees with config: "
            f"teensy_fs={lap_baseline_fs} config_fs={_lap_baseline_fs}"
        )
    if meta["update_count"] <= 0 or meta["current_sequence"] != meta["update_count"]:
        raise RuntimeError("PHOTONS Better-Buckets export current chronology is invalid")
    if not (0 < meta["second_count"] <= PHOTONS_RECOVERY_SECOND_CAPACITY):
        raise RuntimeError("PHOTONS Better-Buckets export second_count is invalid")
    if not (0 < meta["minute_count"] <= PHOTONS_RECOVERY_MINUTE_CAPACITY):
        raise RuntimeError("PHOTONS Better-Buckets export minute_count is invalid")
    if meta["second_newest_sequence"] < meta["second_oldest_sequence"]:
        raise RuntimeError("PHOTONS SECOND export bounds are invalid")
    if meta["minute_newest_sequence"] < meta["minute_oldest_sequence"]:
        raise RuntimeError("PHOTONS MINUTE export bounds are invalid")
    if meta["second_newest_sequence"] != meta["current_sequence"]:
        raise RuntimeError("PHOTONS SECOND export tail is not the current endpoint")
    if _ppb_minute_key(meta["minute_newest_sequence"]) != meta["last_minute_key"]:
        raise RuntimeError("PHOTONS MINUTE export tail does not close its minute key")

    meta["origin_valid"] = _require_bool(
        payload.get("origin_valid"), "PHOTONS.PPB_EXPORT_META.origin_valid"
    )
    if not meta["origin_valid"]:
        raise RuntimeError("PHOTONS Better-Buckets export lacks the statistical origin")
    meta["current"] = _ppb_export_endpoint_from_payload(payload, "current")
    meta["origin"] = _ppb_export_endpoint_from_payload(payload, "origin")
    if int(meta["current"]["sequence"]) != meta["current_sequence"]:
        raise RuntimeError("PHOTONS Better-Buckets export current identity mismatch")
    if meta["origin"] != _ppb_zero_endpoint():
        raise RuntimeError("PHOTONS Better-Buckets export origin is not exact zero")
    meta["chunk_max_endpoints"] = max(
        1,
        min(PHOTONS_RECOVERY_CHUNK_MAX_ENDPOINTS, int(meta["chunk_max_endpoints"])),
    )
    return meta


def _fetch_teensy_ppb_export_page(
    *,
    history: str,
    reset_count: int,
    before_sequence: int,
    count: int,
) -> List[Dict[str, Any]]:
    response = send_command(
        machine="TEENSY",
        subsystem=SUBSYSTEM,
        command="PPB_EXPORT_CHUNK",
        args={
            "history": history,
            "reset_count": int(reset_count),
            "before_sequence": int(before_sequence),
            "count": int(count),
        },
    )
    payload = response.get("payload") if isinstance(response, dict) else None
    payload = payload if isinstance(payload, dict) else {}
    if (
        not isinstance(response, dict)
        or not response.get("success")
        or payload.get("status") != "ppb_export_chunk"
        or payload.get("schema") != "PHOTONS_PPB_FULL_RING_EXPORT_V1"
        or str(payload.get("history") or "") != history
    ):
        raise RuntimeError(
            f"Teensy PHOTONS Better-Buckets export {history} chunk failed: {response!r}"
        )
    if _require_int(
        payload.get("reset_count"), "PHOTONS.PPB_EXPORT_CHUNK.reset_count"
    ) != int(reset_count):
        raise RuntimeError("PHOTONS Better-Buckets export chunk reset identity changed")
    if _require_int(
        payload.get("before_sequence"),
        "PHOTONS.PPB_EXPORT_CHUNK.before_sequence",
    ) != int(before_sequence):
        raise RuntimeError("PHOTONS Better-Buckets export chunk cursor echo changed")
    returned = _require_int(
        payload.get("count"), "PHOTONS.PPB_EXPORT_CHUNK.count"
    )
    if returned > int(count):
        raise RuntimeError("PHOTONS Better-Buckets export returned an oversized chunk")

    out: List[Dict[str, Any]] = []
    previous = int(before_sequence) if int(before_sequence) > 0 else None
    for index in range(returned):
        endpoint = _ppb_export_endpoint_from_payload(payload, f"e{index}")
        sequence = int(endpoint["sequence"])
        if previous is not None and sequence >= previous:
            raise RuntimeError(
                "PHOTONS Better-Buckets export chunk is not strictly newest-to-oldest"
            )
        previous = sequence
        out.append(endpoint)
    return out


def _ppb_export_collect_history(
    *,
    meta: Dict[str, Any],
    history: str,
    existing: List[Dict[str, Any]],
) -> Tuple[List[Dict[str, Any]], bool]:
    """Collect one live producer ring without freezing PHOTONS.

    Exact endpoints learned during a moving-window miss remain lawful facts and
    seed the next META attempt.  A Pi observation gap is therefore repaired by
    reacquiring producer custody, never by inventing the publications it missed.
    """
    if history == "SECOND":
        expected = int(meta["second_count"])
        oldest = int(meta["second_oldest_sequence"])
        newest = int(meta["second_newest_sequence"])
    elif history == "MINUTE":
        expected = int(meta["minute_count"])
        oldest = int(meta["minute_oldest_sequence"])
        newest = int(meta["minute_newest_sequence"])
    else:
        raise ValueError(f"unsupported PHOTONS Better-Buckets history {history!r}")

    if expected <= 0 or newest < oldest:
        raise RuntimeError(f"PHOTONS {history} export has invalid ring geometry")

    collected: Dict[int, Dict[str, Any]] = {}
    for endpoint in existing:
        if not isinstance(endpoint, dict):
            continue
        sequence = endpoint.get("sequence")
        if isinstance(sequence, bool) or not isinstance(sequence, int):
            continue
        if not (oldest <= int(sequence) <= newest):
            continue
        prior = collected.get(int(sequence))
        if prior is not None and not _ppb_endpoints_equal(prior, endpoint):
            raise RuntimeError(
                f"PHOTONS {history} existing endpoint identity changed at {sequence}"
            )
        collected[int(sequence)] = copy.deepcopy(endpoint)

    before_sequence = 0
    chunk_size = int(meta["chunk_max_endpoints"])
    while len(collected) < expected:
        page = _fetch_teensy_ppb_export_page(
            history=history,
            reset_count=int(meta["reset_count"]),
            before_sequence=before_sequence,
            count=chunk_size,
        )
        if not page:
            break
        for endpoint in page:
            sequence = int(endpoint["sequence"])
            if sequence < oldest:
                continue
            if sequence > newest:
                # PHOTONS remains live while Pi pages.  A newer endpoint than
                # this META window is expected transport-time motion, not a
                # contradiction; leave it for the next META rendezvous.
                continue
            prior = collected.get(sequence)
            if prior is not None and not _ppb_endpoints_equal(prior, endpoint):
                raise RuntimeError(
                    f"PHOTONS {history} producer endpoint changed at sequence={sequence}"
                )
            collected[sequence] = copy.deepcopy(endpoint)
        before_sequence = int(page[-1]["sequence"])
        if before_sequence == oldest:
            break

    ordered = [collected[key] for key in sorted(collected)]
    complete = bool(
        len(ordered) == expected
        and int(ordered[0]["sequence"]) == oldest
        and int(ordered[-1]["sequence"]) == newest
    )
    return ordered, complete


def _refresh_ppb_checkpoint_from_live_photons() -> Dict[str, Any]:
    """Reacquire exact Better-Buckets rings from a proved surviving PHOTONS producer."""
    started = time.monotonic()
    attempts = 0
    learned_second_history: List[Dict[str, Any]] = []
    learned_minute_history: List[Dict[str, Any]] = []

    while attempts < 12:
        attempts += 1
        meta = _fetch_teensy_ppb_export_meta()
        with _ppb_checkpoint_lock:
            runtime = _ppb_checkpoint_runtime_ensure_locked()
            runtime_snapshot = _ppb_checkpoint_snapshot_locked(runtime)

        if runtime_snapshot.get("reset_count") not in (None, meta["reset_count"]):
            raise RuntimeError(
                "proved PHOTONS export reset_count disagrees with Pi checkpoint: "
                f"pi={runtime_snapshot.get('reset_count')} teensy={meta['reset_count']}"
            )

        second_history, second_complete = _ppb_export_collect_history(
            meta=meta,
            history="SECOND",
            existing=(
                list(runtime_snapshot.get("second_history") or [])
                + learned_second_history
            ),
        )
        learned_second_history = copy.deepcopy(second_history)
        if not second_complete:
            logging.info(
                "⏳ [photons/ppb] live SECOND window moved during paging; "
                "retaining %d exact endpoints and retrying META (attempt=%d/12)",
                len(second_history),
                attempts,
            )
            time.sleep(0.05)
            continue

        minute_history, minute_complete = _ppb_export_collect_history(
            meta=meta,
            history="MINUTE",
            existing=(
                list(runtime_snapshot.get("minute_history") or [])
                + learned_minute_history
            ),
        )
        learned_minute_history = copy.deepcopy(minute_history)
        if not minute_complete:
            logging.info(
                "⏳ [photons/ppb] live MINUTE window moved during paging; "
                "retaining %d exact endpoints and retrying META (attempt=%d/12)",
                len(minute_history),
                attempts,
            )
            time.sleep(0.05)
            continue

        final_meta = _fetch_teensy_ppb_export_meta()
        stable_meta = all(
            final_meta.get(key) == meta.get(key)
            for key in (
                "reset_count",
                "update_count",
                "current_sequence",
                "second_count",
                "minute_count",
                "second_oldest_sequence",
                "second_newest_sequence",
                "minute_oldest_sequence",
                "minute_newest_sequence",
                "last_minute_key",
                "origin_valid",
            )
        ) and _ppb_endpoints_equal(final_meta.get("current"), meta.get("current"))
        if not stable_meta:
            time.sleep(0.05)
            continue

        retry_rendezvous = False
        with _ppb_checkpoint_lock:
            runtime = _ppb_checkpoint_runtime_ensure_locked()
            live_snapshot = _ppb_checkpoint_snapshot_locked(runtime)
            if (
                live_snapshot.get("reset_count") != meta["reset_count"]
                or live_snapshot.get("update_count") != meta["update_count"]
            ):
                retry_rendezvous = True
            else:
                if (
                    int(live_snapshot.get("current_sequence") or 0)
                    != int(meta["current_sequence"])
                    or not _ppb_endpoints_equal(
                        live_snapshot.get("current"), meta["current"]
                    )
                ):
                    raise RuntimeError(
                        "proved PHOTONS export current endpoint disagrees with Pi live testimony"
                    )
                if (
                    int(live_snapshot.get("expected_second_count") or 0)
                    != int(meta["second_count"])
                    or int(live_snapshot.get("expected_minute_count") or 0)
                    != int(meta["minute_count"])
                    or int(live_snapshot.get("last_minute_key") or 0)
                    != int(meta["last_minute_key"])
                ):
                    raise RuntimeError(
                        "proved PHOTONS export ring geometry disagrees with Pi live testimony"
                    )

                candidate = {
                    "schema": PHOTONS_PPB_PI_CHECKPOINT_SCHEMA,
                    "source_schema": PHOTONS_PPB_FIRMWARE_DELTA_SCHEMA,
                    "valid": True,
                    "recoverable": True,
                    "status": "RECOVERABLE",
                    "status_reason": "FULL_RING_REACQUIRED_FROM_LIVE_PHOTONS",
                    "reset_count": int(meta["reset_count"]),
                    "update_count": int(meta["update_count"]),
                    "rolling_sequence": int(meta["update_count"]),
                    "current_sequence": int(meta["current_sequence"]),
                    "last_minute_key": int(meta["last_minute_key"]),
                    "origin_valid": bool(meta["origin_valid"]),
                    "origin": copy.deepcopy(meta["origin"]),
                    "current": copy.deepcopy(meta["current"]),
                    "expected_second_count": int(meta["second_count"]),
                    "expected_minute_count": int(meta["minute_count"]),
                    "second_count": len(second_history),
                    "minute_count": len(minute_history),
                    "second_history": copy.deepcopy(second_history),
                    "minute_history": copy.deepcopy(minute_history),
                    "contiguous_from_update_count": int(second_history[0]["sequence"]),
                    "gap_count": int(live_snapshot.get("gap_count") or 0),
                    "last_gap": copy.deepcopy(live_snapshot.get("last_gap")),
                    "seed_source": "TEENSY_PHOTONS_FULL_RING_EXPORT",
                    "seed_source_db_detail_id": None,
                    "last_delta_signature": None,
                    "proof_checks": 0,
                }
                normalized = _normalize_saved_ppb_checkpoint(candidate)
                if not normalized.get("recoverable"):
                    raise RuntimeError(
                        "proved PHOTONS full-ring export did not form a recoverable checkpoint"
                    )
                refreshed = _restore_ppb_checkpoint_runtime(
                    normalized,
                    source_db_detail_id=None,
                    reason="FULL_RING_REACQUIRED_FROM_LIVE_PHOTONS",
                    seed_source="TEENSY_PHOTONS_FULL_RING_EXPORT",
                )

        if retry_rendezvous:
            time.sleep(0.05)
            continue

        result = {
            "refreshed": True,
            "basis": "TEENSY_PHOTONS_FULL_RING_EXPORT",
            "reset_count": int(meta["reset_count"]),
            "update_count": int(meta["update_count"]),
            "current_sequence": int(meta["current_sequence"]),
            "second_count": len(second_history),
            "minute_count": len(minute_history),
            "attempts": attempts,
            "waited_s": round(time.monotonic() - started, 3),
            "prior_recoverable": bool(runtime_snapshot.get("recoverable")),
            "prior_status": runtime_snapshot.get("status"),
            "gap_count": int(refreshed.get("gap_count") or 0),
        }
        logging.info(
            "📦 [photons/ppb] surviving producer full-ring custody reacquired: "
            "reset=%d update=%d second=%d/%d minute=%d/%d attempts=%d in %.3fs",
            int(meta["reset_count"]),
            int(meta["update_count"]),
            len(second_history),
            int(meta["second_count"]),
            len(minute_history),
            int(meta["minute_count"]),
            attempts,
            float(result["waited_s"]),
        )
        return result

    raise RuntimeError(
        "proved PHOTONS Better-Buckets export could not reach an exact live rendezvous "
        "after 12 attempts"
    )


def _ppb_checkpoint_reacquire_loop() -> None:
    """Repair Pi Better-Buckets custody from the surviving producer after a live gap."""
    global _ppb_checkpoint_reacquire_success_count
    global _ppb_checkpoint_reacquire_failure_count
    global _ppb_checkpoint_reacquire_last_result
    global _ppb_checkpoint_reacquire_last_failure

    _ppb_checkpoint_reacquire_worker_started.set()
    logging.info("🚀 [photons/ppb] live Better-Buckets reacquisition worker started")

    while True:
        _ppb_checkpoint_reacquire_requested.wait()

        # Clear before beginning, not after success.  If another observation gap
        # occurs while this export is in flight, that newer request remains set
        # and forces another exact producer rendezvous on the next loop.
        _ppb_checkpoint_reacquire_requested.clear()
        if _hard_failure_active():
            continue

        try:
            result = _refresh_ppb_checkpoint_from_live_photons()
        except Exception as exc:
            failure = {
                "failed_at_utc": _utc_now_z(),
                "error": str(exc),
            }
            with _ppb_checkpoint_lock:
                _ppb_checkpoint_reacquire_failure_count += 1
                _ppb_checkpoint_reacquire_last_failure = copy.deepcopy(failure)
            logging.exception(
                "⚠️ [photons/ppb] live Better-Buckets reacquisition failed; retrying"
            )
            if not _hard_failure_active():
                _ppb_checkpoint_reacquire_requested.set()
                time.sleep(PHOTONS_STATE_RETRY_S)
            continue

        # The in-memory ring is repaired first.  Do not call that repair durable
        # until the ordinary ordered persistence worker has crossed a canonical row
        # carrying a recoverable checkpoint from this repaired ancestry.  This keeps
        # config.PHOTONS_RECOVERY paired to campaign_detail through the existing
        # transaction and prevents an older queued partial snapshot from overwriting
        # a separately-written repair.
        repaired_reset = int(result["reset_count"])
        repaired_update = int(result["update_count"])
        durable_checkpoint: Optional[Dict[str, Any]] = None
        superseded = False
        while not _hard_failure_active():
            if _ppb_checkpoint_reacquire_requested.is_set():
                superseded = True
                break
            durable = _read_photons_recovery_config()
            if durable is not None:
                durable_reset = int(durable["reset_count"])
                durable_update = int(durable["update_count"])
                if durable_reset > repaired_reset:
                    # A newer statistical epoch superseded this repair.  Its own
                    # checkpoint ancestry is authoritative; never write backward.
                    superseded = True
                    break
                if durable_reset < repaired_reset:
                    # Ordered persistence has not crossed this statistical epoch yet.
                    time.sleep(PHOTONS_PERSIST_RETRY_S)
                    continue
                if durable_update >= repaired_update and durable.get("recoverable"):
                    durable_checkpoint = durable
                    break
            time.sleep(PHOTONS_PERSIST_RETRY_S)

        if superseded or durable_checkpoint is None:
            continue

        result = {
            **result,
            "durable_update_count": int(durable_checkpoint["update_count"]),
            "durable_status": str(durable_checkpoint.get("status") or ""),
        }
        with _ppb_checkpoint_lock:
            _ppb_checkpoint_reacquire_success_count += 1
            _ppb_checkpoint_reacquire_last_result = copy.deepcopy(result)
            _ppb_checkpoint_reacquire_last_failure = None
        logging.info(
            "✅ [photons/ppb] Better-Buckets custody repaired after a Pi observation gap: "
            "reset=%d producer update=%d durable update=%d; second=%d minute=%d; "
            "exact live rendezvous took %d attempt(s) in %.3fs",
            int(result["reset_count"]),
            int(result["update_count"]),
            int(result["durable_update_count"]),
            int(result["second_count"]),
            int(result["minute_count"]),
            int(result["attempts"]),
            float(result["waited_s"]),
        )


def _rehydrate_pi_campaign(
    active_master: Optional[Dict[str, Any]],
    *,
    source: Optional[Dict[str, Any]],
    live_report: Optional[Dict[str, Any]],
    allow_first_public_count_splice: bool,
) -> None:
    global _active_campaign
    global _closing_campaigns

    with _campaign_lock:
        _closing_campaigns = []
        if active_master is None:
            _active_campaign = None
            return

        start_after: Optional[int] = None
        public_count = 0
        if source is not None and isinstance(source.get("campaign_restore"), dict):
            campaign_restore = source["campaign_restore"]
            start_after = int(campaign_restore["start_after_sequence"])
            public_count = int(campaign_restore["public_count"])
        elif live_report is not None:
            state = str(live_report.get("campaign_state") or "").upper()
            if state == "ACTIVE":
                name = str(live_report.get("campaign") or "")
                if name != active_master["campaign"]:
                    raise RuntimeError(
                        "live Teensy LANTERN campaign does not match active master: "
                        f"teensy={name!r} db={active_master['campaign']!r}"
                    )
                start_after_raw = _require_int(
                    live_report.get("campaign_start_after_sequence"),
                    "PHOTONS.REPORT.campaign_start_after_sequence",
                    minimum=1,
                )
                public_count = _require_int(
                    live_report.get("campaign_public_count"),
                    "PHOTONS.REPORT.campaign_public_count",
                )
                start_after = int(start_after_raw)
            elif state == "START_PENDING":
                name = str(live_report.get("campaign") or "")
                if name != active_master["campaign"]:
                    raise RuntimeError("live pending LANTERN campaign name mismatch")
            elif state != "STOPPED":
                raise RuntimeError(
                    f"cannot reattach Pi ownership during Teensy campaign state {state!r}"
                )
        else:
            master_payload = _require_dict(
                active_master.get("master_payload"), "active campaign master payload"
            )
            report = master_payload.get("report")
            report = report if isinstance(report, dict) else {}
            raw_start = master_payload.get("start_after_sequence")
            if raw_start is None:
                raw_start = report.get("start_after_sequence")
            if raw_start is not None:
                start_after = _require_int(
                    raw_start, "active campaign durable start_after_sequence", minimum=1
                )
            raw_public = master_payload.get("firmware_public_count")
            if raw_public is None:
                raw_public = report.get("campaign_public_count")
            if raw_public is not None:
                public_count = _require_int(
                    raw_public, "active campaign durable public_count"
                )

        window: Dict[str, Any] = {
            "campaign": active_master["campaign"],
            "campaign_id": int(active_master["campaign_id"]),
            "started_at": active_master["started_at"],
            "start_after_sequence": start_after,
            "firmware_public_count": public_count,
        }
        if active_master.get("baseline_campaign_id") is not None:
            window["baseline_campaign_id"] = int(active_master["baseline_campaign_id"])
        if active_master.get("baseline_campaign"):
            window["baseline_campaign"] = str(active_master["baseline_campaign"])
        if allow_first_public_count_splice:
            window["restart_public_count_splice_pending"] = True
        _active_campaign = window


def _validate_live_teensy_against_durable_source(
    report: Dict[str, Any],
    source: Optional[Dict[str, Any]],
) -> Dict[str, int]:
    floor = {
        "sequence": _require_int(
            report.get("fragment_sequence"),
            "PHOTONS.REPORT_RECOVERY.fragment_sequence",
        ),
        "publish_count": _require_int(
            report.get("publish_count"), "PHOTONS.REPORT_RECOVERY.publish_count"
        ),
        "reset_count": _require_int(
            report.get("stats_reset_count"),
            "PHOTONS.REPORT_RECOVERY.stats_reset_count",
        ),
        "update_count": _require_int(
            report.get("stats_update_count"),
            "PHOTONS.REPORT_RECOVERY.stats_update_count",
        ),
        "lap_count": _require_int(
            report.get("stats_lap_count"),
            "PHOTONS.REPORT_RECOVERY.stats_lap_count",
        ),
        "total_lap_gnss_ns": _require_int(
            report.get("stats_total_lap_gnss_ns"),
            "PHOTONS.REPORT_RECOVERY.stats_total_lap_gnss_ns",
        ),
        "custody_lap_count": _require_int(
            report.get("custody_lap_count"),
            "PHOTONS.REPORT_RECOVERY.custody_lap_count",
        ),
        "custody_total_lap_gnss_ns": _require_int(
            report.get("custody_total_lap_gnss_ns"),
            "PHOTONS.REPORT_RECOVERY.custody_total_lap_gnss_ns",
        ),
    }
    if source is None:
        return floor
    if (
        floor["sequence"] < int(source["sequence"])
        or floor["publish_count"] < int(source["publish_count"])
        or floor["reset_count"] != int(source["reset_count"])
        or floor["update_count"] < int(source["update_count"])
        or floor["lap_count"] < int(source["lap_count"])
        or floor["total_lap_gnss_ns"] < int(source["total_lap_gnss_ns"])
        or floor["custody_lap_count"] < int(source["custody_lap_count"])
        or floor["custody_total_lap_gnss_ns"]
        < int(source["custody_total_lap_gnss_ns"])
    ):
        raise RuntimeError(
            "live Teensy PHOTONS state is not descended from the newest durable source"
        )
    return floor


def _recovery_proof_matches(
    photons: Dict[str, Any], expected: Dict[str, Any]
) -> bool:
    instrument = photons.get("photons")
    if not isinstance(instrument, dict):
        return False
    stats = instrument.get("stats")
    recovery = instrument.get("recovery")
    if not isinstance(stats, dict) or not isinstance(recovery, dict):
        return False
    try:
        sequence = _require_int(photons.get("sequence"), "proof sequence", minimum=1)
        update_count = _require_int(
            stats.get("update_count"), "proof update_count", minimum=1
        )
        reset_count = _require_int(stats.get("reset_count"), "proof reset_count")
        lap_count = _require_int(stats.get("lap_count"), "proof lap_count")
        total_ns = _require_int(
            stats.get("total_lap_gnss_ns"), "proof total_lap_gnss_ns"
        )
        custody_laps = _require_int(
            stats.get("custody_lap_count"), "proof custody_lap_count"
        )
        custody_total = _require_int(
            stats.get("custody_total_lap_gnss_ns"),
            "proof custody_total_lap_gnss_ns",
        )
        fresh = _require_bool(
            recovery.get("fresh_physical_ancestry"), "proof fresh_physical_ancestry"
        )
    except Exception:
        return False
    if not fresh:
        return False

    mode = str(expected["mode"])
    if mode in {"HELD_RESTORE", "FIRMWARE_PENDING_PROOF"}:
        try:
            if not _require_bool(recovery.get("restored"), "proof restored"):
                return False
            if not _require_bool(recovery.get("proof_advanced"), "proof advanced"):
                return False
            if _require_int(recovery.get("generation"), "proof generation") != int(
                expected["generation"]
            ):
                return False
            for field in (
                "source_reset_count",
                "source_sequence",
                "source_update_count",
                "source_lap_count",
                "source_total_lap_gnss_ns",
                "source_custody_lap_count",
                "source_custody_total_lap_gnss_ns",
            ):
                if _require_int(recovery.get(field), f"proof {field}") != int(
                    expected[field]
                ):
                    return False
        except Exception:
            return False
        exact_sequence = expected.get("exact_sequence")
        exact_update = expected.get("exact_update_count")
        if exact_sequence is not None and sequence != int(exact_sequence):
            return False
        if exact_update is not None and update_count != int(exact_update):
            return False
        return (
            reset_count == int(expected["source_reset_count"])
            and sequence == int(expected["source_sequence"]) + 1
            and update_count == int(expected["source_update_count"]) + 1
            and lap_count >= int(expected["source_lap_count"])
            and total_ns >= int(expected["source_total_lap_gnss_ns"])
            and custody_laps >= int(expected["source_custody_lap_count"])
            and custody_total >= int(expected["source_custody_total_lap_gnss_ns"])
        )

    if mode == "LIVE_PRODUCER_ADOPT":
        return (
            reset_count == int(expected["reset_count"])
            and sequence > int(expected["sequence"])
            and update_count > int(expected["update_count"])
            and lap_count > int(expected["lap_count"])
            and total_ns > int(expected["total_lap_gnss_ns"])
            and custody_laps > int(expected["custody_lap_count"])
            and custody_total > int(expected["custody_total_lap_gnss_ns"])
        )

    if mode == "COLD_START":
        try:
            restored = _require_bool(recovery.get("restored"), "cold proof restored")
            generation = _require_int(recovery.get("generation"), "cold proof generation")
        except Exception:
            return False
        return (
            not restored
            and generation == int(expected["generation"])
            and update_count >= 1
            and lap_count >= 1
            and total_ns >= 1
            and custody_laps >= 1
            and custody_total >= 1
        )

    raise RuntimeError(f"unknown PHOTONS recovery proof mode {mode!r}")


def _arm_recovery_proof(expected: Dict[str, Any]) -> None:
    global _recovery_proof_expected
    global _recovery_proof_persisted
    with _recovery_lock:
        _recovery_proof_expected = copy.deepcopy(expected)
        _recovery_proof_persisted = None
        _recovery_proof_durable.clear()


def _clear_recovery_proof_custody() -> None:
    """Forget only process-local proof expectation after an invalidated producer generation."""
    global _recovery_proof_expected
    global _recovery_proof_persisted
    with _recovery_lock:
        _recovery_proof_expected = None
        _recovery_proof_persisted = None
        _recovery_proof_durable.clear()


def _note_recovery_proof_persisted(
    photons: Dict[str, Any], detail_id: int
) -> None:
    global _recovery_proof_persisted
    with _recovery_lock:
        expected = copy.deepcopy(_recovery_proof_expected)
        if expected is None or _recovery_proof_durable.is_set():
            return
        if not _recovery_proof_matches(photons, expected):
            return
        instrument = _require_dict(photons.get("photons"), "persisted proof photons")
        stats = _require_dict(instrument.get("stats"), "persisted proof stats")
        recovery = _require_dict(instrument.get("recovery"), "persisted proof recovery")
        _recovery_proof_persisted = {
            "detail_id": int(detail_id),
            "persisted_at_utc": _utc_now_z(),
            "sequence": int(photons["sequence"]),
            "reset_count": int(stats["reset_count"]),
            "update_count": int(stats["update_count"]),
            "lap_count": int(stats["lap_count"]),
            "total_lap_gnss_ns": int(stats["total_lap_gnss_ns"]),
            "custody_lap_count": int(stats["custody_lap_count"]),
            "custody_total_lap_gnss_ns": int(
                stats["custody_total_lap_gnss_ns"]
            ),
            "generation": int(recovery.get("generation") or 0),
            "mode": expected["mode"],
        }
        _recovery_proof_durable.set()


def _find_durable_firmware_proof(
    *, generation: int, sequence: int, update_count: int
) -> Optional[Dict[str, Any]]:
    if generation <= 0 or sequence <= 0 or update_count <= 0:
        return None
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, payload
            FROM campaign_detail
            WHERE campaign_type = %s
              AND sequence = %s
            ORDER BY id DESC
            LIMIT 32
            """,
            (CAMPAIGN_TYPE_LANTERN, sequence),
        )
        rows = cur.fetchall()
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)
        if not isinstance(payload, dict):
            continue
        instrument = payload.get("photons")
        recovery = instrument.get("recovery") if isinstance(instrument, dict) else None
        stats = instrument.get("stats") if isinstance(instrument, dict) else None
        if not isinstance(recovery, dict) or not isinstance(stats, dict):
            continue
        try:
            if (
                _require_int(recovery.get("generation"), "durable proof generation")
                == generation
                and _require_int(payload.get("sequence"), "durable proof sequence")
                == sequence
                and _require_int(stats.get("update_count"), "durable proof update")
                == update_count
                and _require_bool(recovery.get("proof_advanced"), "durable proof advanced")
            ):
                return {"detail_id": int(row["id"]), "canonical": copy.deepcopy(payload)}
        except Exception:
            continue
    return None


def _wait_for_recovery_proof() -> Dict[str, Any]:
    """Wait for durable proof only; this operation cannot mutate the producer."""
    if not _recovery_proof_durable.wait(timeout=PHOTONS_RECOVERY_PROOF_TIMEOUT_S):
        with _recovery_lock:
            expected = copy.deepcopy(_recovery_proof_expected)
        raise RuntimeError(
            "timed out waiting for a durable advancing PHOTONS recovery proof: "
            f"expected={expected!r}"
        )
    with _recovery_lock:
        proof = copy.deepcopy(_recovery_proof_persisted)
    if not isinstance(proof, dict):
        raise RuntimeError("PHOTONS recovery proof event has no persisted identity")
    return proof


def _acknowledge_recovery_proof(proof: Dict[str, Any]) -> None:
    """Commit a prior producer restore proof; never called by live reuse."""
    _send_teensy_recovery_command(
        "RECOVERY_PROOF_ACK",
        args={
            "generation": int(proof["generation"]),
            "sequence": int(proof["sequence"]),
            "update_count": int(proof["update_count"]),
        },
        accepted_statuses={"recovery_proof_committed"},
    )


def _stage_recovery_history(
    *, generation: int, history: Dict[str, Any]
) -> Dict[str, Any]:
    second_history = list(history["second_history"])
    minute_history = list(history["minute_history"])
    _send_teensy_recovery_command(
        "RECOVERY_BEGIN",
        args={
            "restore_schema_version": PHOTONS_RECOVERY_SCHEMA_VERSION,
            "generation": generation,
            "second_count": len(second_history),
            "minute_count": len(minute_history),
        },
        accepted_statuses={"recovery_staging"},
    )
    chunk_count = 0
    for history_name, endpoints in (
        ("SECOND", second_history),
        ("MINUTE", minute_history),
    ):
        for offset in range(0, len(endpoints), PHOTONS_RECOVERY_CHUNK_MAX_ENDPOINTS):
            chunk = endpoints[
                offset : offset + PHOTONS_RECOVERY_CHUNK_MAX_ENDPOINTS
            ]
            args: Dict[str, Any] = {
                "generation": generation,
                "history": history_name,
                "count": len(chunk),
            }
            for index, endpoint in enumerate(chunk):
                args[f"e{index}_sequence"] = int(endpoint["sequence"])
                args[f"e{index}_lap_count"] = int(endpoint["lap_count"])
                args[f"e{index}_total_lap_gnss_ns"] = int(
                    endpoint["total_lap_gnss_ns"]
                )
            _send_teensy_recovery_command(
                "RECOVERY_CHUNK",
                args=args,
                accepted_statuses={"recovery_chunk_accepted"},
            )
            chunk_count += 1
    return {
        "second_count": len(second_history),
        "minute_count": len(minute_history),
        "chunk_count": chunk_count,
    }


def _best_effort_recovery_abort() -> None:
    try:
        _send_teensy_recovery_command(
            "RECOVERY_ABORT",
            accepted_statuses={"recovery_aborted"},
        )
    except Exception:
        logging.exception("⚠️ [photons] unable to abort held PHOTONS recovery staging")


def _mark_active_campaign_recovered(
    active_master: Optional[Dict[str, Any]],
    *,
    mode: str,
    generation: int,
    source: Optional[Dict[str, Any]],
    proof: Dict[str, Any],
) -> None:
    if active_master is None:
        return
    payload = {
        "restart_recovery_enabled": True,
        "recovered_at": _utc_now_z(),
        "recovery_mode": mode,
        "recovery_generation": generation,
        "recovery_source_detail_id": (
            int(source["db_detail_id"]) if source is not None else None
        ),
        "recovery_proof_detail_id": int(proof["detail_id"]),
        "recovery_proof_sequence": int(proof["sequence"]),
        "recovery_proof_update_count": int(proof["update_count"]),
        "physical_ancestry_restored": False,
    }
    with open_db() as conn:
        cur = conn.cursor()
        cur.execute(
            """
            UPDATE campaign_master
            SET payload = payload || %s::jsonb
            WHERE id = %s
              AND campaign_type = %s
              AND campaign = %s
              AND active = true
            """,
            (
                json.dumps(payload, separators=(",", ":")),
                int(active_master["campaign_id"]),
                CAMPAIGN_TYPE_LANTERN,
                active_master["campaign"],
            ),
        )
        if cur.rowcount != 1:
            raise RuntimeError("active LANTERN recovery metadata did not update exactly once")




def _adopt_live_lantern_state(
    active_master: Optional[Dict[str, Any]],
    *,
    live_report: Dict[str, Any],
    durable_cursor_source: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Adopt the surviving producer's current LANTERN state without mutating it.

    Producer campaign state is newer than the durable snapshot once producer
    continuity has been proved.  This function mutates only Pi-owned custody:
    process-local campaign state and, when the producer is already STOPPED, the
    stale active campaign_master flag.  It never sends a Teensy command.
    """
    global _active_campaign
    global _closing_campaigns

    state = str(live_report.get("campaign_state") or "").strip().upper()
    campaign = str(live_report.get("campaign") or "").strip()

    if active_master is None:
        if state != "STOPPED" or campaign:
            raise RuntimeError(
                "surviving PHOTONS producer owns a LANTERN campaign but PostgreSQL "
                "has no active campaign master"
            )
        _rehydrate_pi_campaign(
            None,
            source=None,
            live_report=live_report,
            allow_first_public_count_splice=False,
        )
        return {
            "state": "STOPPED",
            "campaign": None,
            "active_master": False,
            "master_mutated": False,
        }

    durable_campaign = str(active_master["campaign"])
    if state in {"ACTIVE", "START_PENDING"}:
        if campaign != durable_campaign:
            raise RuntimeError(
                "surviving PHOTONS producer campaign identity disagrees with active "
                f"master: producer={campaign!r} durable={durable_campaign!r}"
            )
        # The live report proves present producer ownership and the current physical
        # floor.  It is not, however, the Pi observation cursor: PUBSUB may already
        # hold an exact ordered prefix between the newest durable row and that live
        # floor.  Seed campaign chronology from the durable predecessor when one is
        # available so those queued producer-authored rows cross the ordinary strict
        # monotonic court in order.  If some publications truly were not observed,
        # the existing first-row splice records that forward gap explicitly.
        cursor_source: Optional[Dict[str, Any]] = None
        cursor_authority = "LIVE_PRODUCER_REPORT"
        if state == "ACTIVE" and isinstance(durable_cursor_source, dict):
            campaign_restore = durable_cursor_source.get("campaign_restore")
            if isinstance(campaign_restore, dict):
                source_campaign = str(campaign_restore.get("campaign") or "").strip()
                if source_campaign != durable_campaign:
                    raise RuntimeError(
                        "durable PHOTONS campaign cursor disagrees with active master: "
                        f"source={source_campaign!r} durable={durable_campaign!r}"
                    )
                source_start_after = _require_int(
                    campaign_restore.get("start_after_sequence"),
                    "durable PHOTONS campaign cursor start_after_sequence",
                    minimum=1,
                )
                source_public_count = _require_int(
                    campaign_restore.get("public_count"),
                    "durable PHOTONS campaign cursor public_count",
                    minimum=1,
                )
                live_start_after = _require_int(
                    live_report.get("campaign_start_after_sequence"),
                    "live PHOTONS campaign start_after_sequence",
                    minimum=1,
                )
                live_public_count = _require_int(
                    live_report.get("campaign_public_count"),
                    "live PHOTONS campaign public_count",
                    minimum=1,
                )
                if live_start_after != source_start_after:
                    raise RuntimeError(
                        "surviving PHOTONS campaign boundary disagrees with durable cursor: "
                        f"live={live_start_after} durable={source_start_after} "
                        f"campaign={durable_campaign!r}"
                    )
                if live_public_count < source_public_count:
                    raise RuntimeError(
                        "surviving PHOTONS campaign public-count regressed behind durable cursor: "
                        f"live={live_public_count} durable={source_public_count} "
                        f"campaign={durable_campaign!r}"
                    )
                cursor_source = durable_cursor_source
                cursor_authority = "DURABLE_RECOVERY_SOURCE"

        _rehydrate_pi_campaign(
            active_master,
            source=cursor_source,
            live_report=live_report,
            allow_first_public_count_splice=True,
        )
        return {
            "state": state,
            "campaign": campaign,
            "active_master": True,
            "master_mutated": False,
            "campaign_cursor_authority": cursor_authority,
            "campaign_cursor_source_detail_id": (
                int(durable_cursor_source["db_detail_id"])
                if cursor_source is not None
                else None
            ),
        }

    if state != "STOPPED":
        raise RuntimeError(
            f"surviving PHOTONS producer is in non-adoptable campaign state {state!r}"
        )
    if campaign and campaign != durable_campaign:
        raise RuntimeError(
            "stopped surviving PHOTONS producer retains a different campaign "
            f"identity: producer={campaign!r} durable={durable_campaign!r}"
        )

    adopted_at = _utc_now_z()
    with open_db() as conn:
        cur = conn.cursor()
        cur.execute(
            """
            UPDATE campaign_master
            SET active = false,
                payload = payload || jsonb_build_object(
                    'stopped_at', to_jsonb(%s::text),
                    'producer_state_adopted_at', to_jsonb(%s::text),
                    'producer_state_adoption', to_jsonb(%s::text)
                )
            WHERE id = %s
              AND campaign_type = %s
              AND campaign = %s
              AND active = true
            """,
            (
                adopted_at,
                adopted_at,
                "SURVIVING_PRODUCER_STOPPED",
                int(active_master["campaign_id"]),
                CAMPAIGN_TYPE_LANTERN,
                durable_campaign,
            ),
        )
        if cur.rowcount != 1:
            raise RuntimeError(
                "surviving PHOTONS STOPPED adoption did not retire exactly one "
                "active LANTERN campaign master"
            )

    with _campaign_lock:
        _active_campaign = None
        _closing_campaigns = []

    return {
        "state": "STOPPED",
        "campaign": durable_campaign,
        "active_master": False,
        "master_mutated": True,
        "adopted_at_utc": adopted_at,
    }


def _startup_finish_pending_recovery_proof(
    *,
    active_master: Optional[Dict[str, Any]],
    source: Optional[Dict[str, Any]],
    recovery_report: Dict[str, Any],
    broad_report: Dict[str, Any],
    skipped: List[Dict[str, Any]],
    rows_scanned: int,
) -> Dict[str, Any]:
    """Finish a previously committed restore; this is not producer reuse."""

    if not bool(recovery_report.get("proof_pending")):
        raise RuntimeError("pending PHOTONS recovery-proof continuation is not pending")
    if source is None:
        raise RuntimeError("pending PHOTONS recovery proof has no durable source")

    broad = _fetch_teensy_photons_report()
    floor = _validate_live_teensy_against_durable_source(recovery_report, source)
    generation = int(recovery_report["generation"])
    expected = {
        "mode": "FIRMWARE_PENDING_PROOF",
        "generation": generation,
        "source_sequence": int(recovery_report["source_sequence"]),
        "source_reset_count": int(recovery_report["source_reset_count"]),
        "source_update_count": int(recovery_report["source_update_count"]),
        "source_lap_count": int(recovery_report["source_lap_count"]),
        "source_total_lap_gnss_ns": int(
            recovery_report["source_total_lap_gnss_ns"]
        ),
        "source_custody_lap_count": int(
            recovery_report["source_custody_lap_count"]
        ),
        "source_custody_total_lap_gnss_ns": int(
            recovery_report["source_custody_total_lap_gnss_ns"]
        ),
    }
    _arm_recovery_proof(expected)

    if bool(recovery_report.get("proof_advanced_published")):
        durable = _find_durable_firmware_proof(
            generation=generation,
            sequence=int(recovery_report["proof_sequence"]),
            update_count=int(recovery_report["proof_update_count"]),
        )
        if durable is not None:
            _note_recovery_proof_persisted(
                durable["canonical"], int(durable["detail_id"])
            )

    preserved = _fragment_queue.qsize()
    if preserved:
        logging.info(
            "📥 [photons/recovery] preserving %d queued PHOTONS_FRAGMENT rows "
            "while completing prior restore proof",
            preserved,
        )
    campaign_adoption = _adopt_live_lantern_state(
        active_master,
        live_report=broad,
    )
    _start_workers()
    proof = _wait_for_recovery_proof()
    recovery_receipt = _record_photons_recovery_receipt(
        proof=proof,
        recovery_mode="PENDING_RESTORE_PROOF",
        generation=generation,
        source=None,
        source_identity=recovery_report,
        source_checkpoint=None,
        history=None,
        producer_mutated=True,
    )
    _acknowledge_recovery_proof(proof)
    ppb_live_refresh = _refresh_ppb_checkpoint_from_live_photons()

    if campaign_adoption["active_master"]:
        _mark_active_campaign_recovered(
            active_master,
            mode="PENDING_RESTORE_PROOF",
            generation=generation,
            source=source,
            proof=proof,
        )

    result = {
        "mode": "PENDING_RESTORE_PROOF",
        "generation": generation,
        "source_detail_id": int(source["db_detail_id"]),
        "source_rows_scanned": rows_scanned,
        "damaged_rows_skipped": copy.deepcopy(skipped),
        "ingress_rows_preserved_at_worker_start": preserved,
        "live_floor": floor,
        "proof": proof,
        "recovery_receipt": copy.deepcopy(recovery_receipt),
        "ppb_live_refresh": ppb_live_refresh,
        "campaign_adoption": campaign_adoption,
        "producer_reuse": False,
        "producer_mutated": True,
        "producer_mutation": "RECOVERY_PROOF_ACK_ONLY",
    }
    _recovery_status_set("COMPLETE", **result)
    return result


def _startup_held_restore(
    *,
    active_master: Optional[Dict[str, Any]],
    source: Dict[str, Any],
    literal_history: Dict[str, Any],
    skipped: List[Dict[str, Any]],
    rows_scanned: int,
) -> Dict[str, Any]:
    global _recovery_restore_count
    global _recovery_partial_history_restore_count
    generation = _new_recovery_generation()
    history_truncated = bool(literal_history.get("history_truncated"))

    # RECOVERY_COMMIT is the complete dead-producer desired-state cutover.  If a
    # durable active LANTERN exists, its firmware state must already be inside the
    # same producer restore image; a later recovery-specific START is forbidden.
    campaign_restore = source.get("campaign_restore")
    restore_args = source.get("restore_args")
    if not isinstance(restore_args, dict):
        raise RuntimeError("PHOTONS held restore source lacks producer restore_args")
    if active_master is not None:
        if not isinstance(campaign_restore, dict):
            raise RuntimeError(
                "active LANTERN is absent from PHOTONS RECOVERY_COMMIT desired state"
            )
        if restore_args.get("campaign_active") is not True:
            raise RuntimeError(
                "active LANTERN restore source does not set campaign_active=true"
            )
        if str(restore_args.get("campaign") or "") != str(active_master["campaign"]):
            raise RuntimeError(
                "PHOTONS RECOVERY_COMMIT campaign identity disagrees with active master"
            )
    elif campaign_restore is not None or restore_args.get("campaign_active") is not False:
        raise RuntimeError(
            "PHOTONS held restore carries campaign state without an active LANTERN master"
        )
    # HELD_RESTORE replaces physical ancestry rather than pretending the Pi saw
    # every firmware publication during restart/reacquisition.  The first
    # post-restore campaign row may therefore be a lawful forward splice from
    # the durable public_count.  _validate_firmware_campaign() independently
    # proves that the row still closes exactly against sequence-start_after.
    _rehydrate_pi_campaign(
        active_master,
        source=source,
        live_report=None,
        allow_first_public_count_splice=True,
    )
    drained = _drain_queue(_fragment_queue)
    expected = {
        "mode": "HELD_RESTORE",
        "generation": generation,
        "source_sequence": int(source["sequence"]),
        "source_reset_count": int(source["reset_count"]),
        "source_update_count": int(source["update_count"]),
        "source_lap_count": int(source["lap_count"]),
        "source_total_lap_gnss_ns": int(source["total_lap_gnss_ns"]),
        "source_custody_lap_count": int(source["custody_lap_count"]),
        "source_custody_total_lap_gnss_ns": int(
            source["custody_total_lap_gnss_ns"]
        ),
        "exact_sequence": int(source["sequence"]) + 1,
        "exact_update_count": int(source["update_count"]) + 1,
    }
    _arm_recovery_proof(expected)
    _recovery_status_set(
        "STAGING_HELD_RESTORE",
        mode="HELD_RESTORE",
        generation=generation,
        source_detail_id=int(source["db_detail_id"]),
        source_sequence=int(source["sequence"]),
        source_update_count=int(source["update_count"]),
        damaged_rows_skipped=copy.deepcopy(skipped),
        source_rows_scanned=rows_scanned,
        ingress_rows_drained=drained,
        ppb_restore_authority={
            "schema": literal_history.get("schema"),
            "authority": literal_history.get("authority"),
            "history_scope": literal_history.get("history_scope"),
            "history_truncated": history_truncated,
            "source_db_detail_id": literal_history.get("source_db_detail_id"),
            "source_update_count": literal_history.get("source_update_count"),
            "checkpoint_status": literal_history.get("checkpoint_status"),
            "checkpoint_seed_source": literal_history.get("checkpoint_seed_source"),
            "gap_count": literal_history.get("gap_count"),
            "source_expected_second_count": literal_history.get(
                "source_expected_second_count"
            ),
            "source_expected_minute_count": literal_history.get(
                "source_expected_minute_count"
            ),
            "staged_second_count": len(literal_history.get("second_history") or []),
            "staged_minute_count": len(literal_history.get("minute_history") or []),
            "surrendered_second_endpoints": literal_history.get(
                "surrendered_second_endpoints"
            ),
            "surrendered_minute_endpoints": literal_history.get(
                "surrendered_minute_endpoints"
            ),
            "second_origin_promoted": literal_history.get("second_origin_promoted"),
            "minute_origin_promoted": literal_history.get("minute_origin_promoted"),
        },
        exact_successor={
            "sequence": int(source["sequence"]) + 1,
            "update_count": int(source["update_count"]) + 1,
        },
    )
    try:
        staged = _stage_recovery_history(generation=generation, history=literal_history)
        commit_args = copy.deepcopy(restore_args)
        commit_args["generation"] = generation
        commit = _send_teensy_recovery_command(
            "RECOVERY_COMMIT",
            args=commit_args,
            accepted_statuses={"recovery_committed"},
        )
    except Exception:
        _best_effort_recovery_abort()
        raise

    # RECOVERY_COMMIT is the producer-side cutover.  If the durable source held
    # only a literal suffix, firmware now owns exactly that shorter bounded ring.
    # Mirror the installed geometry before releasing queued N+1 testimony to the
    # workers; otherwise Pi would still expect unseen entries from the dead
    # producer and would manufacture another observation-gap diagnosis.
    checkpoint_after_commit = _adopt_held_restore_checkpoint_runtime(
        source=source,
        literal_history=literal_history,
    )
    if _runtime_recovery_hold.is_set():
        _runtime_recovery_hold.clear()
        logging.info(
            "♻️ [photons/ambient restore] producer cutover committed; releasing "
            "new-lifetime PHOTONS_FRAGMENT testimony for exact N+1 proof"
        )
    if history_truncated:
        logging.warning(
            "🧾 [photons/recovery] held restore preserved exact aggregate ancestry "
            "while surrendering unseen Better-Buckets history: source_detail=%d "
            "staged_second=%d/%d staged_minute=%d/%d surrendered_second=%d "
            "surrendered_minute=%d exact_origin_promoted(second=%s minute=%s)",
            int(source["db_detail_id"]),
            int(literal_history.get("staged_second_count") or 0),
            int(literal_history.get("source_expected_second_count") or 0),
            int(literal_history.get("staged_minute_count") or 0),
            int(literal_history.get("source_expected_minute_count") or 0),
            int(literal_history.get("surrendered_second_endpoints") or 0),
            int(literal_history.get("surrendered_minute_endpoints") or 0),
            bool(literal_history.get("second_origin_promoted")),
            bool(literal_history.get("minute_origin_promoted")),
        )

    _start_workers()
    _recovery_status_set(
        "WAITING_FOR_DURABLE_PROOF",
        mode="HELD_RESTORE",
        generation=generation,
        source_detail_id=int(source["db_detail_id"]),
        staged=staged,
        firmware_commit=commit,
        ppb_history_scope=literal_history.get("history_scope"),
        ppb_history_truncated=history_truncated,
        ppb_checkpoint_after_commit=checkpoint_after_commit,
    )
    proof = _wait_for_recovery_proof()
    recovery_receipt = _record_photons_recovery_receipt(
        proof=proof,
        recovery_mode="HELD_RESTORE",
        generation=generation,
        source=source,
        source_checkpoint=source.get("ppb_restore_checkpoint"),
        history=literal_history,
        producer_mutated=True,
    )
    _acknowledge_recovery_proof(proof)
    _mark_active_campaign_recovered(
        active_master,
        mode="HELD_RESTORE",
        generation=generation,
        source=source,
        proof=proof,
    )
    campaign_restored_in_commit = bool(active_master is not None)
    with _state_lock:
        _recovery_restore_count += 1
        if history_truncated:
            _recovery_partial_history_restore_count += 1
    result = {
        "mode": "HELD_RESTORE",
        "generation": generation,
        "source_detail_id": int(source["db_detail_id"]),
        "source_sequence": int(source["sequence"]),
        "source_update_count": int(source["update_count"]),
        "pending_seed_dropped": int(source["pending_seed_dropped"]),
        "ppb_restore_authority": "DURABLE_LITERAL_CHECKPOINT",
        "ppb_history_scope": literal_history.get("history_scope"),
        "ppb_history_truncated": history_truncated,
        "surrendered_second_endpoints": int(
            literal_history.get("surrendered_second_endpoints") or 0
        ),
        "surrendered_minute_endpoints": int(
            literal_history.get("surrendered_minute_endpoints") or 0
        ),
        "proof_contract": "EXACT_SOURCE_N_PLUS_1",
        "proof": proof,
        "recovery_receipt": copy.deepcopy(recovery_receipt),
        "campaign_restored_in_commit": campaign_restored_in_commit,
        "staged": staged,
        "producer_mutated": True,
    }
    _recovery_status_set("COMPLETE", **result)
    return result



def _startup_cold_start(
    *,
    active_master: Optional[Dict[str, Any]],
    rows_seen: int,
    source_epoch_migration: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Create a fresh producer epoch without inventing continuity across sources."""
    global _recovery_cold_start_count

    migration = copy.deepcopy(source_epoch_migration)
    if rows_seen != 0 and not isinstance(migration, dict):
        raise RuntimeError("cold PHOTONS start was requested despite durable LANTERN rows")
    if isinstance(migration, dict):
        if active_master is not None:
            raise RuntimeError("real-race source migration requires the legacy LANTERN campaign STOPPED")
        if migration.get("to_source") != "PD200T_REAL_RACE":
            raise RuntimeError("unsupported PHOTONS source-epoch migration target")
        if int(migration.get("historical_detail_rows_preserved") or 0) != int(rows_seen):
            raise RuntimeError("PHOTONS source-epoch migration row accounting changed")

    generation = _new_recovery_generation()
    _rehydrate_pi_campaign(
        None,
        source=None,
        live_report=None,
        allow_first_public_count_splice=False,
    )
    drained = _drain_queue(_fragment_queue)
    _arm_recovery_proof({"mode": "COLD_START", "generation": generation})
    _recovery_status_set(
        "COLD_STARTING",
        mode="COLD_START",
        generation=generation,
        ingress_rows_drained=drained,
        source_epoch_migration=migration,
    )
    cold = _send_teensy_recovery_command(
        "RECOVERY_COLD_START",
        args={"generation": generation},
        accepted_statuses={"recovery_cold_start_committed"},
    )
    if _runtime_recovery_hold.is_set():
        _runtime_recovery_hold.clear()
        logging.info(
            "♻️ [photons/ambient restore] cold producer cutover committed; releasing "
            "new-lifetime PHOTONS_FRAGMENT testimony for first durable proof"
        )
    _start_workers()
    proof = _wait_for_recovery_proof()
    live = _fetch_teensy_photons_report()
    campaign_adoption = _adopt_live_lantern_state(
        active_master,
        live_report=live,
    )

    with _state_lock:
        _recovery_cold_start_count += 1
    result = {
        "mode": "COLD_START",
        "generation": generation,
        "firmware": cold,
        "proof": proof,
        "campaign_adoption": campaign_adoption,
        "source_epoch_migration": migration,
        "producer_mutated": True,
    }
    _recovery_status_set("COMPLETE", **result)
    return result



def _startup_adopt_live_producer(
    *,
    active_master: Optional[Dict[str, Any]],
    source: Optional[Dict[str, Any]],
    recovery_report: Dict[str, Any],
    broad_report: Dict[str, Any],
    skipped: List[Dict[str, Any]],
    rows_scanned: int,
) -> Dict[str, Any]:
    """Prove and adopt a surviving PHOTONS producer without mutating it.

    The caller owns one generation-pinned startup classification attempt and has
    already obtained both firmware reports.  Reusing those exact witnesses avoids
    two redundant Teensy RPCs during the busiest part of multi-process startup.
    """
    global _recovery_live_adopt_count

    if bool(recovery_report.get("proof_pending")):
        raise RuntimeError(
            "pending restored-producer proof is not eligible for zero-mutation reuse"
        )

    broad = copy.deepcopy(broad_report)
    floor = _validate_live_teensy_against_durable_source(recovery_report, source)

    campaign_adoption = _adopt_live_lantern_state(
        active_master,
        live_report=broad,
        durable_cursor_source=source,
    )

    expected = {"mode": "LIVE_PRODUCER_ADOPT", **floor}
    _arm_recovery_proof(expected)

    preserved = _fragment_queue.qsize()
    if preserved:
        logging.info(
            "📥 [photons/recovery] preserving %d queued PHOTONS_FRAGMENT rows "
            "during zero-mutation producer adoption",
            preserved,
        )
    _start_workers()
    proof = _wait_for_recovery_proof()
    ppb_live_refresh = _refresh_ppb_checkpoint_from_live_photons()

    generation = int(recovery_report["generation"])
    if campaign_adoption["active_master"]:
        _mark_active_campaign_recovered(
            active_master,
            mode="LIVE_PRODUCER_ADOPT",
            generation=generation,
            source=source,
            proof=proof,
        )

    with _state_lock:
        _recovery_live_adopt_count += 1
    result = {
        "mode": "LIVE_PRODUCER_ADOPT",
        "generation": generation,
        "source_detail_id": int(source["db_detail_id"]) if source else None,
        "source_rows_scanned": rows_scanned,
        "damaged_rows_skipped": copy.deepcopy(skipped),
        "ingress_rows_drained": 0,
        "ingress_rows_preserved_at_worker_start": preserved,
        "live_floor": floor,
        "proof": proof,
        "ppb_live_refresh": ppb_live_refresh,
        "campaign_adoption": campaign_adoption,
        "producer_reuse": True,
        "producer_mutated": False,
        "producer_mutation_commands": [],
    }
    _recovery_status_set("COMPLETE", **result)
    return result


def _perform_phase5_recovery() -> Dict[str, Any]:
    global _recovery_attempt_count
    global _recovery_failure_count

    with _state_lock:
        _recovery_attempt_count += 1
    _recovery_status_set("CLASSIFYING")
    try:
        empty_domain = _retire_orphaned_photons_recovery_config_if_domain_empty()
        if empty_domain.get("domain_empty"):
            logging.warning(
                "🧹 [photons/recovery] durable LANTERN domain is empty "
                "(campaign_master=0 campaign_detail=0); retired orphaned config.%s=%s "
                "and classifying startup as a fresh PHOTONS durability epoch",
                PHOTONS_RECOVERY_CONFIG_KEY,
                bool(empty_domain.get("recovery_config_deleted")),
            )

        active_master = _load_active_lantern_master()
        active_detail_count = (
            _count_lantern_campaign_details(active_master["campaign"])
            if active_master is not None
            else 0
        )
        total_detail_count = _count_current_lantern_details()
        real_race_detail_count = _count_current_real_race_details()
        require_active_campaign = active_master is not None and active_detail_count > 0
        snapshot, skipped, rows_scanned = _load_newest_recoverable_photons_state(
            active_master=active_master,
            require_active_campaign=require_active_campaign,
        )
        desired_state = (
            _project_photons_recovery_snapshot(
                snapshot, datetime.now(timezone.utc)
            )
            if snapshot is not None
            else None
        )
        source = (
            copy.deepcopy(desired_state["restore_source"])
            if isinstance(desired_state, dict)
            else None
        )
        source_epoch_migration: Optional[Dict[str, Any]] = None
        legacy_source_migration_required = False
        if snapshot is None and real_race_detail_count != 0:
            raise RuntimeError(
                "durable PD200T_REAL_RACE history exists but no row satisfies the recovery court: "
                f"rows_scanned={rows_scanned} skipped={skipped!r}"
            )
        if snapshot is None and total_detail_count != 0:
            if active_master is not None:
                raise RuntimeError(
                    "legacy LANTERN campaign is still active during PD200T_REAL_RACE source "
                    "migration; STOP it before deploying the real-race producer"
                )
            legacy_source_migration_required = True

        report = _fetch_teensy_recovery_report()
        if report["staging_active"] and not report["publication_started"]:
            _best_effort_recovery_abort()
            report = _fetch_teensy_recovery_report()
        if report["staging_active"]:
            raise RuntimeError("Teensy PHOTONS recovery staging remains active")

        # Pair this Pi generation only with the real-race firmware before touching
        # incompatible legacy continuation state.  If a previous process already
        # cold-started the new producer, live adoption is lawful.  A live producer
        # that claims restored ancestry in a legacy-only durable domain is exactly
        # the contamination we are preventing and requires a producer reboot.
        broad_preflight = _fetch_teensy_photons_report()
        if legacy_source_migration_required:
            if report["publication_started"] and report["restored"]:
                raise RuntimeError(
                    "live PD200T_REAL_RACE producer carries restored legacy PHOTONS ancestry; "
                    "reboot the Teensy before source-epoch migration"
                )
            source_epoch_migration = _retire_legacy_recovery_for_real_race_epoch(
                legacy_detail_count=total_detail_count
            )

        if snapshot is not None:
            # Seed live Pi recovery custody from the exact sidecar carried by the
            # explicit snapshot. Historical SQL rows have no role in Better-Buckets
            # resurrection or startup reconstruction.
            _restore_ppb_checkpoint_runtime(
                snapshot.ppb_restore_checkpoint,
                source_db_detail_id=int(snapshot.source_detail_id),
            )

        classification = {
            "active_campaign": active_master["campaign"] if active_master else None,
            "active_campaign_detail_count": active_detail_count,
            "total_detail_count": total_detail_count,
            "real_race_detail_count": real_race_detail_count,
            "source_epoch_migration": copy.deepcopy(source_epoch_migration),
            "source_detail_id": (
                int(snapshot.source_detail_id) if snapshot is not None else None
            ),
            "desired_state_schema": (
                desired_state.get("schema") if isinstance(desired_state, dict) else None
            ),
            "time_translation": (
                desired_state.get("time_translation")
                if isinstance(desired_state, dict)
                else None
            ),
            "source_welford_grand_ratio_diagnostic": (
                copy.deepcopy(source.get("welford_grand_ratio_diagnostic"))
                if source is not None
                else None
            ),
            "damaged_rows_skipped": copy.deepcopy(skipped),
            "rows_scanned": rows_scanned,
            "teensy_publication_started": bool(report["publication_started"]),
            "teensy_race_engine_active": bool(broad_preflight["race_engine_active"]),
        }
        _recovery_status_set("CLASSIFIED", **classification)

        if report["publication_started"] and report["proof_pending"]:
            return _startup_finish_pending_recovery_proof(
                active_master=active_master,
                source=source,
                recovery_report=report,
                broad_report=broad_preflight,
                skipped=skipped,
                rows_scanned=rows_scanned,
            )
        if report["publication_started"]:
            return _startup_adopt_live_producer(
                active_master=active_master,
                source=source,
                recovery_report=report,
                broad_report=broad_preflight,
                skipped=skipped,
                rows_scanned=rows_scanned,
            )
        if source is None:
            return _startup_cold_start(
                active_master=active_master,
                rows_seen=total_detail_count,
                source_epoch_migration=source_epoch_migration,
            )
        literal_history = _literal_recovery_history_from_source(source)
        return _startup_held_restore(
            active_master=active_master,
            source=source,
            literal_history=literal_history,
            skipped=skipped,
            rows_scanned=rows_scanned,
        )
    except Exception as exc:
        with _state_lock:
            _recovery_failure_count += 1
        _recovery_status_set("FAILED", error=str(exc))
        logging.exception("💥 [photons] Phase 5 durable recovery failed")
        raise


# ---------------------------------------------------------------------
# LANTERN campaign decoration + campaign_master read model
# ---------------------------------------------------------------------

def _latest_canonical_photons_snapshot() -> Dict[str, Any]:
    """Return the latest canonical PHOTONS observation or fail campaign admission."""
    with _state_lock:
        photons = copy.deepcopy(_latest_photons)
    if not isinstance(photons, dict):
        raise RuntimeError("PHOTONS canonical heartbeat is not available yet")
    if photons.get("schema") != PHOTONS_SCHEMA:
        raise RuntimeError(
            f"latest PHOTONS heartbeat has unexpected schema {photons.get('schema')!r}"
        )
    return photons


def _campaign_public_decoration(window: Dict[str, Any]) -> Dict[str, Any]:
    """Return Pi-owned durable identity/provenance for one LANTERN lifecycle."""
    out: Dict[str, Any] = {
        "schema": LANTERN_CAMPAIGN_SCHEMA,
        "campaign_type": CAMPAIGN_TYPE_LANTERN,
        "campaign": window["campaign"],
        "campaign_id": int(window["campaign_id"]),
        "started_at": window["started_at"],
    }
    if window.get("stopped_at"):
        out["stopped_at"] = str(window["stopped_at"])
    start_after = window.get("start_after_sequence")
    if start_after is not None:
        out["start_after_sequence"] = int(start_after)
    stop_after = window.get("stop_after_sequence")
    if stop_after is not None:
        out["stop_after_sequence"] = int(stop_after)
    baseline_id = window.get("baseline_campaign_id")
    baseline_name = window.get("baseline_campaign")
    if baseline_id is not None:
        out["baseline_campaign_id"] = int(baseline_id)
    if baseline_name:
        out["baseline_campaign"] = str(baseline_name)
    flash_cut_from = window.get("flash_cut_from")
    if flash_cut_from:
        out["flash_cut_from"] = str(flash_cut_from)
    restart_gap = window.get("restart_public_count_gap")
    if restart_gap is not None:
        out["restart_public_count_gap"] = int(restart_gap)
    observation_gap = window.get("public_count_observation_gap")
    if observation_gap is not None:
        out["public_count_observation_gap"] = int(observation_gap)
    return out


def _campaign_decoration_from_firmware(
    firmware_campaign: Dict[str, Any],
) -> Dict[str, Any]:
    """Merge Teensy-authored boundary/science with Pi-owned durable provenance."""
    global _closing_campaigns

    name = str(firmware_campaign["campaign"])
    start_after = int(firmware_campaign["start_after_sequence"])
    public_count = int(firmware_campaign["public_count"])
    final = bool(firmware_campaign["final"])

    with _campaign_lock:
        window: Optional[Dict[str, Any]] = None
        if _active_campaign is not None and _active_campaign.get("campaign") == name:
            window = _active_campaign
        if window is None:
            for candidate in _closing_campaigns:
                if candidate.get("campaign") == name:
                    window = candidate
                    break
        if window is None:
            raise ValueError(
                f"Teensy published LANTERN campaign {name!r} with no Pi lifecycle owner"
            )

        known_start_after = window.get("start_after_sequence")
        if known_start_after is None:
            window["start_after_sequence"] = start_after
        elif int(known_start_after) != start_after:
            raise ValueError(
                "Pi/Teensy LANTERN START boundary mismatch: "
                f"pi={known_start_after} teensy={start_after} campaign={name!r}"
            )

        previous_public_count = int(window.get("firmware_public_count") or 0)
        splice_pending = bool(window.get("restart_public_count_splice_pending"))
        if public_count <= previous_public_count:
            raise ValueError(
                "PHOTONS campaign public-count failed to advance: "
                f"previous={previous_public_count} current={public_count} campaign={name!r}"
            )

        public_gap = public_count - previous_public_count - 1
        window.pop("public_count_observation_gap", None)
        if public_gap > 0:
            if splice_pending:
                window["restart_public_count_gap"] = public_gap
            else:
                # The firmware boundary court already proved public_count ==
                # sequence-start_after_sequence.  A forward jump therefore means
                # Pi did not receive one or more physical campaign rows; preserve
                # that missing testimony as an explicit observation gap instead of
                # rejecting every descendant forever.
                window["public_count_observation_gap"] = public_gap
        if splice_pending:
            window.pop("restart_public_count_splice_pending", None)
        window["firmware_public_count"] = public_count

        if final:
            stop_after = int(firmware_campaign["stop_after_sequence"])
            window["stop_after_sequence"] = stop_after
            window["firmware_final_seen"] = True

        out = _campaign_public_decoration(window)
        out["source_schema"] = firmware_campaign["schema"]
        out["public_count"] = public_count
        out["final"] = final
        out["stats"] = copy.deepcopy(firmware_campaign.get("stats") or {})
        window.pop("public_count_observation_gap", None)

        if final:
            closing_id = int(window["campaign_id"])
            _closing_campaigns = [
                candidate
                for candidate in _closing_campaigns
                if int(candidate["campaign_id"]) != closing_id
            ]

        return out


def _lantern_report_from_photons(photons: Payload) -> Dict[str, Any]:
    """Build the campaign_master latest read model without recomputing science."""
    campaign = _require_dict(photons.get("campaign"), "PHOTONS.campaign")
    instrument = _require_dict(photons.get("photons"), "PHOTONS.photons")
    science = instrument.get("science")
    science = science if isinstance(science, dict) else {}
    accepted = science.get("accepted")
    accepted = accepted if isinstance(accepted, dict) else {}
    excluded = science.get("excluded")
    excluded = excluded if isinstance(excluded, dict) else {}
    stats = instrument.get("stats")
    stats = stats if isinstance(stats, dict) else {}
    race = instrument.get("race")
    race = race if isinstance(race, dict) else {}
    campaign_stats = campaign.get("stats")
    campaign_stats = campaign_stats if isinstance(campaign_stats, dict) else {}

    return {
        "schema": LANTERN_REPORT_SCHEMA,
        "campaign_type": CAMPAIGN_TYPE_LANTERN,
        "campaign": campaign.get("campaign"),
        "campaign_id": campaign.get("campaign_id"),
        "sequence": photons.get("sequence"),
        "pps_count": photons.get("pps_count"),
        "published_at_utc": photons.get("published_at_utc"),
        "instrument_source": instrument.get("source"),
        "campaign_public_count": campaign.get("public_count"),
        "campaign_final": campaign.get("final"),
        "campaign_stats": copy.deepcopy(campaign_stats),
        # These are exact firmware cumulative/read-model facts, not Pi-authored
        # campaign-local statistics.
        "candidate_count": science.get("candidate_count"),
        "accepted_count": accepted.get("count"),
        "excluded_count": excluded.get("count"),
        "instrument_stats": copy.deepcopy(stats),
        "race": copy.deepcopy(race),
        "location": copy.deepcopy(photons.get("location") or {}),
        "environment": copy.deepcopy(photons.get("environment") or {}),
    }


def _baseline_relation_for_active_campaign() -> Optional[Dict[str, Any]]:
    """Return the active LANTERN campaign and its referenced baseline, if any."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT
                current.id AS campaign_id,
                current.campaign AS campaign,
                baseline.id AS baseline_campaign_id,
                baseline.campaign AS baseline_campaign,
                baseline.payload AS baseline_payload
            FROM campaign_master AS current
            JOIN campaign_master AS baseline
              ON baseline.id = (current.payload ->> 'baseline_campaign_id')::bigint
             AND baseline.campaign_type = current.campaign_type
            WHERE current.campaign_type = %s
              AND current.active = true
            ORDER BY current.ts DESC, current.id DESC
            LIMIT 1
            """,
            (CAMPAIGN_TYPE_LANTERN,),
        )
        row = cur.fetchone()

    if row is None:
        return None

    payload = row["baseline_payload"]
    if isinstance(payload, str):
        payload = json.loads(payload)
    if not isinstance(payload, dict):
        raise RuntimeError("baseline LANTERN campaign payload is not an object")

    report = payload.get("report")
    if not isinstance(report, dict) or not report:
        raise RuntimeError(
            f"Baseline campaign '{row['baseline_campaign']}' has no report"
        )

    return {
        "campaign_id": int(row["campaign_id"]),
        "campaign": row["campaign"],
        "baseline_campaign_id": int(row["baseline_campaign_id"]),
        "baseline_campaign": row["baseline_campaign"],
        "baseline_report": report,
        "baseline_started_at": payload.get("started_at"),
        "baseline_stopped_at": payload.get("stopped_at"),
    }


def _persist_photons(
    photons: Payload, checkpoint: Dict[str, Any]
) -> int:
    """Persist one canonical PHOTONS row and advance the LANTERN read model.

    ``viable`` remains structural: individual firmware lap exclusions never make
    the one-second batch non-viable.  campaign_master is only a latest read model;
    campaign_detail remains the durable observation history.
    """
    if not _checkpoint_matches_photons_state(checkpoint, photons):
        raise RuntimeError(
            "PHOTONS recovery checkpoint identity does not match canonical observation"
        )
    sequence = photons["sequence"]
    pps_count = photons.get("pps_count")
    campaign = photons.get("campaign")
    campaign = campaign if isinstance(campaign, dict) else None
    campaign_name = (
        str(campaign.get("campaign") or "").strip()
        if campaign is not None
        else ""
    )
    campaign_id = (
        _require_int(campaign.get("campaign_id"), "PHOTONS.campaign.campaign_id", minimum=1)
        if campaign is not None
        else None
    )

    detail_id = 0
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            INSERT INTO campaign_detail (
                ts,
                campaign_type,
                campaign,
                viable,
                payload,
                sequence,
                pps_count
            )
            VALUES (
                %s,
                %s,
                %s,
                %s,
                %s::jsonb,
                %s,
                %s
            )
            RETURNING id
            """,
            (
                datetime.now(timezone.utc),
                CAMPAIGN_TYPE_LANTERN,
                campaign_name or None,
                True,
                json.dumps(photons, separators=(",", ":")),
                sequence,
                pps_count,
            ),
        )
        inserted = cur.fetchone()
        if not isinstance(inserted, dict) or inserted.get("id") is None:
            raise RuntimeError("PHOTONS campaign_detail insert returned no durable id")
        detail_id = int(inserted["id"])

        if campaign is not None:
            report = _lantern_report_from_photons(photons)
            boundary_payload: Dict[str, Any] = {
                "start_after_sequence": _require_int(
                    campaign.get("start_after_sequence"),
                    "PHOTONS.campaign.start_after_sequence",
                    minimum=1,
                ),
                "firmware_public_count": _require_int(
                    campaign.get("public_count"),
                    "PHOTONS.campaign.public_count",
                    minimum=1,
                ),
                "boundary_contract": "TEENSY_AUTHORED_PUBLISHED_FRAGMENT_BOUNDARY",
                "start_boundary_pending": False,
                "flash_cut_pending": False,
            }
            if campaign.get("final") is True:
                boundary_payload["stop_after_sequence"] = _require_int(
                    campaign.get("stop_after_sequence"),
                    "PHOTONS.campaign.stop_after_sequence",
                    minimum=1,
                )
                boundary_payload["stop_boundary_pending"] = False
                if campaign.get("stopped_at"):
                    boundary_payload["stopped_at"] = str(campaign["stopped_at"])

            cur.execute(
                """
                UPDATE campaign_master
                SET payload = jsonb_set(
                    payload,
                    '{report}',
                    %s::jsonb,
                    true
                ) || %s::jsonb
                WHERE id = %s
                  AND campaign_type = %s
                  AND campaign = %s
                """,
                (
                    json.dumps(report, separators=(",", ":")),
                    json.dumps(boundary_payload, separators=(",", ":")),
                    int(campaign_id),
                    CAMPAIGN_TYPE_LANTERN,
                    campaign_name,
                ),
            )
            if cur.rowcount != 1:
                raise RuntimeError(
                    "persisted LANTERN-labeled PHOTONS row has no matching campaign_master "
                    f"id={campaign_id} campaign={campaign_name!r}"
                )

            if campaign.get("final") is True:
                cur.execute(
                    """
                    UPDATE campaign_master
                    SET active = false
                    WHERE id = %s
                      AND campaign_type = %s
                      AND campaign = %s
                    """,
                    (int(campaign_id), CAMPAIGN_TYPE_LANTERN, campaign_name),
                )
                if cur.rowcount != 1:
                    raise RuntimeError(
                        "final LANTERN firmware row did not close exactly one campaign_master"
                    )

        _write_photons_recovery_config(cur, checkpoint)

    # Crossing the open_db context is the irreversible durability boundary.
    # Return the committed identity to the worker; post-commit recovery-proof
    # adjudication must never cause this scientific row to be inserted again.
    return detail_id


# ---------------------------------------------------------------------
# PHOTONS_FRAGMENT ingress + Phase-2 workers
# ---------------------------------------------------------------------

def _note_ingress_queue_depth() -> None:
    global _ingress_queue_depth_max
    depth = _fragment_queue.qsize()
    with _state_lock:
        if depth > _ingress_queue_depth_max:
            _ingress_queue_depth_max = depth


def _note_persist_queue_depth() -> None:
    global _persist_queue_depth_max
    depth = _persist_queue.qsize()
    with _state_lock:
        if depth > _persist_queue_depth_max:
            _persist_queue_depth_max = depth


def on_photons_fragment(fragment: Payload) -> None:
    """PUBSUB fast path: copy one firmware fragment into the ingress queue."""
    global _latest_fragment
    global _fragments_received
    global _fragments_queued
    global _hard_failure_ingress_dropped

    if _hard_failure_active() and not _repair_active():
        with _state_lock:
            _hard_failure_ingress_dropped += 1
        return

    copied = copy.deepcopy(fragment)
    with _state_lock:
        _fragments_received += 1
        _latest_fragment = copied

    # Both Phase-2 queues are unbounded, so put_nowait() does not turn a
    # downstream stall into PUBSUB backpressure.  Memory growth is observable
    # through REPORT queue-depth diagnostics.
    _fragment_queue.put_nowait(copied)
    with _state_lock:
        _fragments_queued += 1
    _note_ingress_queue_depth()


def _state_loop() -> None:
    """Attach SYSTEM context, validate, publish, then hand each row to persistence."""
    global _latest_photons
    global _fragments_processed
    global _fragments_rejected
    global _photons_published
    global _system_report_retry_count
    global _last_structural_rejection
    global _last_system_report_failure
    global _hard_failure_state_dropped

    _state_worker_started.set()
    logging.info("🚀 [photons] canonical PHOTONS_V1 state worker started")

    while True:
        fragment = None
        if _runtime_recovery_hold.is_set():
            time.sleep(0.01)
            continue
        with _maintenance_lock:
            try:
                fragment = _fragment_queue.get_nowait()
            except queue.Empty:
                pass
            if fragment is None:
                pass
            elif _hard_failure_active() and not _repair_active():
                with _state_lock:
                    _hard_failure_state_dropped += 1
            else:
                with _state_lock:
                    _fragments_processed += 1

                system_failure_logged = False
                while True:
                    if _hard_failure_active() and not _repair_active():
                        break
                    try:
                        system_context = _fetch_system_report()
                        break
                    except Exception as exc:
                        failure = {
                            "failed_at_utc": _utc_now_z(),
                            "sequence": fragment.get("sequence") if isinstance(fragment, dict) else None,
                            "reason": str(exc),
                        }
                        with _state_lock:
                            _system_report_retry_count += 1
                            _last_system_report_failure = failure
                        if not system_failure_logged:
                            logging.exception(
                                "⚠️ [photons] SYSTEM.REPORT unavailable for PHOTONS sequence=%s; retrying",
                                failure["sequence"],
                            )
                            system_failure_logged = True
                        time.sleep(PHOTONS_STATE_RETRY_S)

                if _hard_failure_active() and not _repair_active():
                    with _state_lock:
                        _hard_failure_state_dropped += 1
                    continue

                try:
                    photons, ppb_restore_checkpoint = _make_photons(
                        fragment, system_context
                    )

                    # Producer-declared rolling custody loss is not an ordinary
                    # science exclusion.  The current row cannot carry durable
                    # resurrection authority, and an active LANTERN campaign has
                    # also missed physical optical observations.  Stop before
                    # publishing or persisting this row so the newest durable
                    # campaign_detail remains the last trustworthy predecessor.
                    checkpoint = ppb_restore_checkpoint
                    if checkpoint.get("status") == "ROLLING_CUSTODY_LOST":
                        instrument = _require_dict(
                            photons.get("photons"), "PHOTONS.photons"
                        )
                        stats = _require_dict(
                            instrument.get("stats"), "PHOTONS.photons.stats"
                        )
                        projection = _require_dict(
                            instrument.get("projection"),
                            "PHOTONS.photons.projection",
                        )
                        interrupt = _require_dict(
                            instrument.get("interrupt"),
                            "PHOTONS.photons.interrupt",
                        )
                        _enter_hard_failure(
                            "producer_rolling_custody_lost",
                            {
                                "sequence": photons.get("sequence"),
                                "reset_count": stats.get("reset_count"),
                                "update_count": stats.get("update_count"),
                                "lap_count": stats.get("lap_count"),
                                "raw_lap_ring_overflow_count": projection.get(
                                    "queue_overflow_count"
                                ),
                                "interrupt_callback_missing_count": interrupt.get(
                                    "callback_missing_count"
                                ),
                                "interrupt_callback_missing_count_origin": interrupt.get(
                                    "callback_missing_count_origin"
                                ),
                                "interrupt_callback_missing_count_since_ancestry": interrupt.get(
                                    "callback_missing_count_since_ancestry"
                                ),
                                "interrupt_inactive_edge_count": interrupt.get(
                                    "inactive_edge_count"
                                ),
                                "interrupt_inactive_edge_count_origin": interrupt.get(
                                    "inactive_edge_count_origin"
                                ),
                                "interrupt_inactive_edge_count_since_ancestry": interrupt.get(
                                    "inactive_edge_count_since_ancestry"
                                ),
                                "checkpoint_status": checkpoint.get("status"),
                                "checkpoint_last_gap": copy.deepcopy(
                                    checkpoint.get("last_gap")
                                ),
                            },
                            source="PHOTONS_FRAGMENT_CUSTODY",
                        )
                        continue

                    firmware_campaign = _validate_firmware_campaign(
                        fragment, int(photons["sequence"])
                    )
                    if firmware_campaign is not None:
                        photons["campaign"] = _campaign_decoration_from_firmware(
                            firmware_campaign
                        )
                except Exception as exc:
                    rejection = {
                        "rejected_at_utc": _utc_now_z(),
                        "reason": str(exc),
                        "schema": fragment.get("schema") if isinstance(fragment, dict) else None,
                        "sequence": fragment.get("sequence") if isinstance(fragment, dict) else None,
                    }
                    with _state_lock:
                        _fragments_rejected += 1
                        _last_structural_rejection = rejection
                    logging.exception(
                        "💥 [photons] rejected PHOTONS fragment sequence=%s schema=%s: %s",
                        rejection.get("sequence"),
                        rejection.get("schema") or "unknown",
                        rejection.get("reason") or "structural/accounting court failure",
                    )
                    continue

                with _state_lock:
                    _latest_photons = copy.deepcopy(photons)

                publish(PHOTONS_TOPIC, photons)
                with _state_lock:
                    _photons_published += 1

                _persist_queue.put_nowait(
                    {
                        "photons": copy.deepcopy(photons),
                        "ppb_restore_checkpoint": copy.deepcopy(ppb_restore_checkpoint),
                    }
                )
                _note_persist_queue_depth()


        if fragment is None:
            time.sleep(0.01)


def _persistence_loop() -> None:
    """Persist canonical PHOTONS rows in order, retrying transient DB failures."""
    global _rows_persisted
    global _persistence_retry_count
    global _last_persistence_failure
    global _hard_failure_persistence_dropped

    _persistence_worker_started.set()
    logging.info("🚀 [photons] ordered PHOTONS persistence worker started")

    while True:
        persist_item = None
        photons = None
        checkpoint = None
        with _maintenance_lock:
            try:
                persist_item = _persist_queue.get_nowait()
            except queue.Empty:
                pass
            if persist_item is not None:
                photons = _require_dict(persist_item.get("photons"), "PHOTONS persistence item")
                checkpoint = _require_dict(
                    persist_item.get("ppb_restore_checkpoint"),
                    "PHOTONS persistence recovery checkpoint",
                )
            if photons is None:
                pass
            elif _hard_failure_active() and not _repair_active():
                with _state_lock:
                    _hard_failure_persistence_dropped += 1
            else:
                failure_logged = False
                while True:
                    try:
                        detail_id = _persist_photons(photons, checkpoint)
                        with _state_lock:
                            _rows_persisted += 1
                    except Exception as exc:
                        if _hard_failure_active() and not _repair_active():
                            with _state_lock:
                                _hard_failure_persistence_dropped += 1
                            break
                        failure = {
                            "failed_at_utc": _utc_now_z(),
                            "sequence": photons.get("sequence") if isinstance(photons, dict) else None,
                            "pps_count": photons.get("pps_count") if isinstance(photons, dict) else None,
                            "reason": str(exc),
                        }
                        with _state_lock:
                            _persistence_retry_count += 1
                            _last_persistence_failure = failure
                        if not failure_logged:
                            logging.exception(
                                "⚠️ [photons] LANTERN campaign_detail persistence failed for PHOTONS sequence=%s; retrying",
                                failure["sequence"],
                            )
                            failure_logged = True
                        time.sleep(PHOTONS_PERSIST_RETRY_S)
                        continue

                    # The campaign_detail row is already committed.  Recovery-proof
                    # adjudication is a separate custody stage: if its contract is
                    # internally inconsistent, fail hard rather than retrying the
                    # INSERT and manufacturing duplicate durable testimony.
                    try:
                        _note_recovery_proof_persisted(photons, detail_id)
                    except Exception as exc:
                        _enter_hard_failure(
                            "post_commit_recovery_proof_adjudication_failed",
                            {
                                "detail_id": int(detail_id),
                                "sequence": photons.get("sequence"),
                                "pps_count": photons.get("pps_count"),
                                "error": str(exc),
                            },
                            source="PHOTONS_PERSISTENCE_PROOF",
                        )

                    # STATS_RESET completion belongs to this same custody path.  The
                    # returned detail_id proves this exact canonical row is already
                    # durable; do not race the writer with a side-channel DB poll.
                    _note_stats_reset_persisted(photons, detail_id)
                    break


        if photons is None:
            time.sleep(0.01)



def _runtime_teensy_rpc_generation() -> Optional[int]:
    """Return one currently proved PUBSUB->Teensy RPC generation, or None."""
    feature = _read_startup_system_feature("PI.PUBSUB.TEENSY_RPC")
    if not feature.get("known") or str(feature.get("status") or "") != "NOMINAL":
        return None
    if feature.get("expired") is not False:
        return None
    generation = feature.get("generation")
    if isinstance(generation, bool) or not isinstance(generation, int) or generation <= 0:
        raise RuntimeError(
            f"PI.PUBSUB.TEENSY_RPC has malformed runtime generation: {feature!r}"
        )
    return int(generation)


def _wait_for_runtime_recovery_persistence_boundary() -> Dict[str, Any]:
    """Prove every pre-boundary canonical row has crossed ordered durability."""
    started = time.monotonic()
    last_log = started
    while True:
        if _hard_failure_active():
            raise RuntimeError("PHOTONS entered HARD_FAILURE before ambient persistence barrier")
        if _persist_queue.empty():
            # Empty alone is insufficient: the persistence worker may already own
            # an item.  Crossing _maintenance_lock proves that any such transaction
            # has completed before the recovery source is selected.
            with _maintenance_lock:
                if _persist_queue.empty():
                    return {
                        "waited_s": round(time.monotonic() - started, 3),
                        "persist_queue_depth": 0,
                    }
        now = time.monotonic()
        if now - last_log >= PHOTONS_RUNTIME_RECOVERY_BARRIER_LOG_S:
            logging.info(
                "⏳ [photons/ambient restore] waiting for pre-boundary persistence "
                "custody queue_depth=%d waited_s=%.1f",
                _persist_queue.qsize(),
                now - started,
            )
            last_log = now
        time.sleep(0.05)


def _runtime_reconcile_teensy_generation(previous_generation: int,
                                         observed_generation: int) -> None:
    """Classify one RPC generation boundary and resurrect only a newborn producer."""
    global _runtime_recovery_generation
    global _runtime_recovery_count
    global _runtime_recovery_surviving_reattach_count
    global _runtime_recovery_failure_count
    global _runtime_recovery_rows_retired
    global _runtime_recovery_last

    _runtime_recovery_hold.set()
    _campaign_control_ready.clear()
    _set_operational_state(
        OPERATIONAL_STATE_RECOVERING,
        reason="teensy_rpc_generation_changed",
        source="RUNTIME_GENERATION_MONITOR",
        details={
            "previous_generation": int(previous_generation),
            "observed_generation": int(observed_generation),
        },
    )
    with _state_lock:
        _runtime_recovery_count += 1

    logging.error(
        "🧭 [photons/ambient restore] Teensy RPC generation changed %d -> %d; "
        "suspending canonical authorship until producer continuity is classified",
        int(previous_generation),
        int(observed_generation),
    )

    try:
        barrier = _wait_for_runtime_recovery_persistence_boundary()
        report = _fetch_teensy_recovery_report(require_baseline=False)

        if bool(report.get("publication_started")):
            # Transport was replaced but the producer lifetime survived.  Do not
            # invoke recovery and do not disturb live Welford/Better-Buckets state.
            _runtime_recovery_hold.clear()
            _campaign_control_ready.set()
            _runtime_recovery_generation = int(observed_generation)
            with _state_lock:
                _runtime_recovery_surviving_reattach_count += 1
                _runtime_recovery_last = {
                    "mode": "SURVIVING_PRODUCER_REATTACH",
                    "previous_generation": int(previous_generation),
                    "observed_generation": int(observed_generation),
                    "persistence_barrier": copy.deepcopy(barrier),
                    "producer_mutated": False,
                    "completed_at_utc": _utc_now_z(),
                }
            _set_operational_state(
                OPERATIONAL_STATE_RUNNING,
                reason="surviving_producer_reattached",
                source="RUNTIME_GENERATION_MONITOR",
                details=copy.deepcopy(_runtime_recovery_last),
            )
            logging.info(
                "✅ [photons/ambient restore] transport generation=%d reattached to "
                "the surviving PHOTONS producer; no producer state was mutated",
                int(observed_generation),
            )
            return

        # A newborn Teensy is intentionally silent: firmware publication remains
        # held until this Pi supplies baseline + one explicit recovery verdict.
        # Retire any queued prefix that never crossed the canonical/durable court.
        retired = _drain_queue(_fragment_queue)
        with _state_lock:
            _runtime_recovery_rows_retired += int(retired)
        logging.error(
            "🧭 [photons/ambient restore] generation=%d is a newborn held producer; "
            "retired %d uncustodied ingress row(s) and restoring from the newest "
            "durable PHOTONS boundary",
            int(observed_generation),
            int(retired),
        )

        _configure_teensy_lap_baseline()
        recovery = _perform_phase5_recovery()
        current_generation = _runtime_teensy_rpc_generation()
        if current_generation != int(observed_generation):
            raise RuntimeError(
                "Teensy RPC generation changed again before PHOTONS ambient recovery completed: "
                f"expected={observed_generation} observed={current_generation}"
            )

        _runtime_recovery_hold.clear()
        _campaign_control_ready.set()
        _runtime_recovery_generation = int(observed_generation)
        with _state_lock:
            _runtime_recovery_last = {
                "mode": "AMBIENT_PRODUCER_RESURRECTION",
                "previous_generation": int(previous_generation),
                "observed_generation": int(observed_generation),
                "persistence_barrier": copy.deepcopy(barrier),
                "retired_ingress_rows": int(retired),
                "recovery": copy.deepcopy(recovery),
                "producer_mutated": bool(recovery.get("producer_mutated", True)),
                "completed_at_utc": _utc_now_z(),
            }
        _set_operational_state(
            OPERATIONAL_STATE_RUNNING,
            reason="ambient_producer_resurrection_complete",
            source="RUNTIME_GENERATION_MONITOR",
            details=copy.deepcopy(_runtime_recovery_last),
        )
        proof = recovery.get("proof") if isinstance(recovery.get("proof"), dict) else {}
        logging.info(
            "✅ [photons/ambient restore] producer resurrection complete on generation=%d: "
            "mode=%s source_detail=%s source_update=%s -> durable proof update=%s",
            int(observed_generation),
            recovery.get("mode") or "unknown",
            recovery.get("source_detail_id") or "none",
            recovery.get("source_update_count") or "n/a",
            proof.get("update_count") or "n/a",
        )
    except Exception:
        with _state_lock:
            _runtime_recovery_failure_count += 1
        raise


def _runtime_teensy_generation_monitor_loop() -> None:
    """Reconcile every proved PUBSUB generation change without replaying RPC intent."""
    global _runtime_recovery_generation

    _runtime_recovery_monitor_started.set()
    logging.info(
        "🚀 [photons] Teensy producer-lifetime monitor started generation=%d",
        int(_runtime_recovery_generation),
    )
    while True:
        if _hard_failure_active():
            time.sleep(PHOTONS_RUNTIME_RECOVERY_POLL_S)
            continue
        try:
            observed = _runtime_teensy_rpc_generation()
        except Exception:
            logging.exception("⚠️ [photons/ambient restore] runtime RPC-generation read failed")
            time.sleep(PHOTONS_RUNTIME_RECOVERY_POLL_S)
            continue
        if observed is None or observed == _runtime_recovery_generation:
            time.sleep(PHOTONS_RUNTIME_RECOVERY_POLL_S)
            continue

        previous = int(_runtime_recovery_generation)
        target = int(observed)
        try:
            _runtime_reconcile_teensy_generation(previous, target)
        except Exception as exc:
            # A second transport loss while recovery is in flight invalidates the
            # attempt but is not a scientific contradiction.  Keep authorship held
            # and wait for the next proved generation.  A failure on the still-live
            # target generation is semantic and therefore terminal.
            try:
                current = _runtime_teensy_rpc_generation()
            except Exception:
                current = None
            if current is None or current != target:
                logging.warning(
                    "⚠️ [photons/ambient restore] recovery on generation=%d was "
                    "invalidated by another transport boundary: %s; awaiting reattach",
                    target,
                    exc,
                )
                time.sleep(PHOTONS_RUNTIME_RECOVERY_POLL_S)
                continue
            _enter_hard_failure(
                "ambient_producer_recovery_failed",
                {
                    "previous_generation": previous,
                    "observed_generation": target,
                    "error": str(exc),
                    "recovery": _recovery_report_surface(),
                },
                source="PHOTONS_RUNTIME_RECOVERY",
            )


def _start_runtime_teensy_generation_monitor(initial_generation: int) -> None:
    global _runtime_recovery_generation
    if initial_generation <= 0:
        raise RuntimeError("PHOTONS runtime generation monitor requires positive generation")
    _runtime_recovery_generation = int(initial_generation)
    if _runtime_recovery_monitor_started.is_set():
        return
    threading.Thread(
        target=_runtime_teensy_generation_monitor_loop,
        daemon=True,
        name="photons-teensy-generation",
    ).start()

def _start_workers() -> None:
    if not _state_worker_started.is_set():
        threading.Thread(
            target=_state_loop,
            daemon=True,
            name="photons-state",
        ).start()
    if not _persistence_worker_started.is_set():
        threading.Thread(
            target=_persistence_loop,
            daemon=True,
            name="photons-persistence",
        ).start()
    if not _ppb_checkpoint_reacquire_worker_started.is_set():
        threading.Thread(
            target=_ppb_checkpoint_reacquire_loop,
            daemon=True,
            name="photons-ppb-reacquire",
        ).start()


def on_publication(topic: str, payload: Payload) -> None:
    """Execute one publication that PUBSUB already routed to PI:PHOTONS."""
    if topic != PHOTONS_FRAGMENT_TOPIC:
        raise RuntimeError(f"PI:PHOTONS received unexpected static route topic {topic!r}")
    on_photons_fragment(payload)


# ---------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------

def _campaign_control_gate(command: str) -> Optional[Dict[str, Any]]:
    if _hard_failure_active():
        return {
            "success": False,
            "message": f"{command} unavailable while PHOTONS is latched in HARD_FAILURE",
            "payload": {"operational_state": _operational_state_snapshot()},
        }
    if _campaign_control_ready.is_set():
        return None
    return {
        "success": False,
        "message": f"{command} unavailable while LANTERN startup classification is in progress",
    }


def cmd_start(args: Optional[dict]) -> dict:
    """START a cold LANTERN campaign or hot-cut the active campaign."""
    global _active_campaign
    global _closing_campaigns
    global _campaign_start_count
    global _flash_cut_count
    global _last_campaign_transition

    busy = _campaign_control_gate("START")
    if busy is not None:
        return busy

    campaign_name = str((args or {}).get("campaign") or "").strip()
    if not campaign_name:
        return {"success": False, "message": "START requires 'campaign' argument"}

    try:
        latest_photons = _latest_canonical_photons_snapshot()
    except Exception as exc:
        return {"success": False, "message": str(exc)}

    with _campaign_lock:
        if _closing_campaigns:
            return {
                "success": False,
                "message": "Previous LANTERN STOP/Flash Cut is still awaiting its final firmware row",
            }

        previous = copy.deepcopy(_active_campaign) if _active_campaign is not None else None
        flash_cut = previous is not None
        if flash_cut and str(previous.get("campaign") or "") == campaign_name:
            return {
                "success": False,
                "message": f"Campaign '{campaign_name}' is already active — choose a new name",
            }

        started_at = _utc_now_z()
        start_location = copy.deepcopy(latest_photons.get("location") or {})

        try:
            with open_db(row_dict=True) as conn:
                cur = conn.cursor()
                if not flash_cut:
                    cur.execute(
                        """
                        SELECT id, campaign
                        FROM campaign_master
                        WHERE campaign_type = %s
                          AND active = true
                        ORDER BY ts DESC, id DESC
                        LIMIT 1
                        """,
                        (CAMPAIGN_TYPE_LANTERN,),
                    )
                    active = cur.fetchone()
                    if active is not None:
                        return {
                            "success": False,
                            "message": (
                                f"Database already has active LANTERN campaign "
                                f"'{active['campaign']}' (id={active['id']})"
                            ),
                        }

                cur.execute(
                    """
                    SELECT id, active
                    FROM campaign_master
                    WHERE campaign_type = %s
                      AND campaign = %s
                    ORDER BY ts DESC, id DESC
                    LIMIT 1
                    """,
                    (CAMPAIGN_TYPE_LANTERN, campaign_name),
                )
                existing = cur.fetchone()
                if existing is not None:
                    return {
                        "success": False,
                        "message": (
                            f"Campaign '{campaign_name}' already exists "
                            f"(id={existing['id']}); DELETE it first or choose a new name"
                        ),
                    }

                if flash_cut:
                    cur.execute(
                        """
                        UPDATE campaign_master
                        SET active = false,
                            payload = payload || jsonb_build_object(
                                'stopped_at', to_jsonb(%s::text),
                                'stop_boundary_pending', to_jsonb(true),
                                'stop_boundary_contract',
                                    to_jsonb('TEENSY_FINAL_PUBLISHED_CAMPAIGN_FRAGMENT'::text),
                                'flash_cut_to', to_jsonb(%s::text)
                            )
                        WHERE id = %s
                          AND campaign_type = %s
                          AND campaign = %s
                          AND active = true
                        """,
                        (
                            started_at,
                            campaign_name,
                            int(previous["campaign_id"]),
                            CAMPAIGN_TYPE_LANTERN,
                            previous["campaign"],
                        ),
                    )
                    if cur.rowcount != 1:
                        raise RuntimeError("hot START did not retire exactly one active LANTERN master")

                master_payload = {
                    "schema": LANTERN_MASTER_SCHEMA,
                    "campaign_type": CAMPAIGN_TYPE_LANTERN,
                    "started_at": started_at,
                    "boundary_contract": "TEENSY_AUTHORED_PUBLISHED_FRAGMENT_BOUNDARY",
                    "start_boundary_pending": True,
                    "instrument_always_on": True,
                    "starts_physical_measurement": False,
                    "location": start_location,
                }
                if flash_cut:
                    master_payload.update({
                        "flash_cut_from": previous["campaign"],
                        "flash_cut_pending": True,
                        "flash_cut_armed_at": started_at,
                        "flash_cut_preserves_instrument_statistics": True,
                    })

                cur.execute(
                    """
                    INSERT INTO campaign_master (
                        campaign_type,
                        campaign,
                        active,
                        payload
                    )
                    VALUES (%s, %s, true, %s::jsonb)
                    RETURNING id
                    """,
                    (
                        CAMPAIGN_TYPE_LANTERN,
                        campaign_name,
                        json.dumps(master_payload, separators=(",", ":")),
                    ),
                )
                created = cur.fetchone()
                if created is None:
                    raise RuntimeError("LANTERN campaign_master insert returned no id")
                campaign_id = int(created["id"])
        except Exception as exc:
            logging.exception("❌ [photons] LANTERN START database preparation failed")
            return {"success": False, "message": str(exc)}

        closing: Optional[Dict[str, Any]] = None
        if flash_cut:
            closing = copy.deepcopy(previous)
            closing["stopped_at"] = started_at
            closing["stop_boundary_pending"] = True
            closing["flash_cut_to"] = campaign_name
            _closing_campaigns.append(closing)

        _active_campaign = {
            "campaign": campaign_name,
            "campaign_id": campaign_id,
            "started_at": started_at,
            "start_after_sequence": None,
            "firmware_public_count": 0,
            **({"flash_cut_from": previous["campaign"]} if flash_cut else {}),
        }

        try:
            teensy_response = _request_teensy_campaign_command(
                "START",
                campaign=campaign_name,
                accepted_statuses=TEENSY_CAMPAIGN_START_ACCEPTED_STATUSES,
            )
        except Exception as exc:
            # Firmware did not accept the boundary. Restore the Pi lifecycle and DB
            # atomically enough that no false campaign ownership remains visible.
            _active_campaign = previous
            if closing is not None:
                closing_id = int(closing["campaign_id"])
                _closing_campaigns = [
                    item for item in _closing_campaigns
                    if int(item["campaign_id"]) != closing_id
                ]
            try:
                with open_db() as conn:
                    cur = conn.cursor()
                    cur.execute(
                        """
                        DELETE FROM campaign_master
                        WHERE id = %s
                          AND campaign_type = %s
                          AND campaign = %s
                        """,
                        (campaign_id, CAMPAIGN_TYPE_LANTERN, campaign_name),
                    )
                    if flash_cut:
                        cur.execute(
                            """
                            UPDATE campaign_master
                            SET active = true,
                                payload = payload
                                    - 'stopped_at'
                                    - 'stop_boundary_pending'
                                    - 'stop_boundary_contract'
                                    - 'flash_cut_to'
                            WHERE id = %s
                              AND campaign_type = %s
                              AND campaign = %s
                            """,
                            (
                                int(previous["campaign_id"]),
                                CAMPAIGN_TYPE_LANTERN,
                                previous["campaign"],
                            ),
                        )
            except Exception:
                logging.exception(
                    "💥 [photons] failed to roll back LANTERN DB after Teensy START rejection"
                )
            logging.exception("❌ [photons] Teensy PHOTONS.START failed")
            return {"success": False, "message": str(exc)}

    teensy_payload = (
        teensy_response.get("payload")
        if isinstance(teensy_response.get("payload"), dict)
        else {}
    )
    transition = {
        "action": "FLASH_CUT" if flash_cut else "START",
        "at_utc": started_at,
        "campaign": campaign_name,
        "campaign_id": campaign_id,
        "previous_campaign": previous.get("campaign") if flash_cut else None,
        "boundary_pending": True,
        "teensy_status": teensy_payload.get("status"),
    }
    with _state_lock:
        _campaign_start_count += 1
        if flash_cut:
            _flash_cut_count += 1
        _last_campaign_transition = copy.deepcopy(transition)

    logging.info(
        "%s [photons] LANTERN %s campaign='%s' id=%d armed on Teensy; boundary pending",
        "⚡" if flash_cut else "▶️",
        "FLASH_CUT" if flash_cut else "START",
        campaign_name,
        campaign_id,
    )
    return {
        "success": True,
        "message": "OK",
        "payload": {
            "campaign_type": CAMPAIGN_TYPE_LANTERN,
            "campaign": campaign_name,
            "campaign_id": campaign_id,
            "flash_cut": flash_cut,
            "previous_campaign": previous.get("campaign") if flash_cut else None,
            "boundary_pending": True,
            "teensy_status": teensy_payload.get("status"),
            "instrument_always_on": True,
            "physical_measurement_changed": False,
            "instrument_statistics_preserved": True,
        },
    }


def cmd_flash_cut(args: Optional[dict]) -> dict:
    """Explicit CLOCKS-parity hot cut; START on an active campaign uses the same path."""
    busy = _campaign_control_gate("FLASH_CUT")
    if busy is not None:
        return busy
    with _campaign_lock:
        if _active_campaign is None:
            return {"success": False, "message": "FLASH_CUT requires an active LANTERN campaign"}
        if _closing_campaigns:
            return {
                "success": False,
                "message": "FLASH_CUT unavailable while a prior final boundary is pending",
            }
    return cmd_start(args)


def cmd_stop(_: Optional[dict]) -> dict:
    """STOP after one final Teensy-authored LANTERN fragment."""
    global _active_campaign
    global _closing_campaigns
    global _campaign_stop_count
    global _last_campaign_transition

    busy = _campaign_control_gate("STOP")
    if busy is not None:
        return busy

    with _campaign_lock:
        if _closing_campaigns:
            return {
                "success": False,
                "message": "STOP unavailable while a prior LANTERN final/Flash Cut boundary is pending",
            }
        if _active_campaign is None:
            return {"success": False, "message": "No active LANTERN campaign"}

        closing = copy.deepcopy(_active_campaign)
        stopped_at = _utc_now_z()
        closing["stopped_at"] = stopped_at
        closing["stop_boundary_pending"] = True

        try:
            teensy_response = _request_teensy_campaign_command(
                "STOP",
                accepted_statuses=TEENSY_CAMPAIGN_STOP_ACCEPTED_STATUSES,
            )
        except Exception as exc:
            logging.exception("❌ [photons] Teensy PHOTONS.STOP failed")
            return {"success": False, "message": str(exc)}

        # Firmware now owns the pending final boundary.  Move the Pi lifecycle to
        # the closing set before releasing the lock so the ordered state worker can
        # attach durable identity to that final firmware row.
        _closing_campaigns.append(closing)
        _active_campaign = None

        try:
            with open_db() as conn:
                cur = conn.cursor()
                cur.execute(
                    """
                    UPDATE campaign_master
                    SET active = false,
                        payload = payload || jsonb_build_object(
                            'stopped_at', to_jsonb(%s::text),
                            'stop_boundary_pending', to_jsonb(true),
                            'stop_boundary_contract',
                                to_jsonb('TEENSY_FINAL_PUBLISHED_CAMPAIGN_FRAGMENT'::text)
                        )
                    WHERE id = %s
                      AND campaign_type = %s
                      AND campaign = %s
                      AND active = true
                    """,
                    (
                        stopped_at,
                        int(closing["campaign_id"]),
                        CAMPAIGN_TYPE_LANTERN,
                        closing["campaign"],
                    ),
                )
                if cur.rowcount != 1:
                    raise RuntimeError(
                        "active LANTERN campaign_master was not stopped exactly once"
                    )
        except Exception as exc:
            # The physical STOP has already been accepted.  Keep the closing
            # lifecycle in memory so the final firmware row remains attributable;
            # persistence of that row will also force the master inactive.
            logging.exception(
                "⚠️ [photons] LANTERN STOP DB update failed after firmware accepted STOP; "
                "final campaign-row persistence will converge the master"
            )
            db_transition_error = str(exc)
        else:
            db_transition_error = None

    teensy_payload = (
        teensy_response.get("payload")
        if isinstance(teensy_response.get("payload"), dict)
        else {}
    )
    transition = {
        "action": "STOP",
        "at_utc": stopped_at,
        "campaign": closing["campaign"],
        "campaign_id": int(closing["campaign_id"]),
        "stop_boundary_pending": True,
        "teensy_status": teensy_payload.get("status"),
        "database_transition_error": db_transition_error,
    }
    with _state_lock:
        _campaign_stop_count += 1
        _last_campaign_transition = copy.deepcopy(transition)

    logging.info(
        "⏹️ [photons] LANTERN STOP campaign='%s' id=%d armed on Teensy; "
        "final firmware boundary pending",
        closing["campaign"],
        int(closing["campaign_id"]),
    )
    return {
        "success": True,
        "message": "OK",
        "payload": {
            "campaign_type": CAMPAIGN_TYPE_LANTERN,
            "campaign": closing["campaign"],
            "campaign_id": int(closing["campaign_id"]),
            "stop_boundary_pending": True,
            "teensy_status": teensy_payload.get("status"),
            "database_transition_error": db_transition_error,
            "instrument_always_on": True,
            "physical_measurement_changed": False,
        },
    }


def _pending_persist_campaign_names() -> set[str]:
    """Return campaign names already canonicalized but not yet persisted."""
    names: set[str] = set()
    with _persist_queue.mutex:
        pending = list(_persist_queue.queue)
    for item in pending:
        photons = item.get("photons") if isinstance(item, dict) else None
        campaign = photons.get("campaign") if isinstance(photons, dict) else None
        if isinstance(campaign, dict):
            name = str(campaign.get("campaign") or "").strip()
            if name:
                names.add(name)
    return names


def _drain_queue(q: queue.Queue) -> int:
    drained = 0
    while True:
        try:
            q.get_nowait()
        except queue.Empty:
            break
        drained += 1
    return drained


def _maintenance_requires_stopped(command: str) -> Optional[Dict[str, Any]]:
    busy = _campaign_control_gate(command)
    if busy is not None:
        return busy
    with _campaign_lock:
        if _active_campaign is not None:
            return {
                "success": False,
                "message": f"{command} requires LANTERN STOP; active campaign is '{_active_campaign['campaign']}'",
            }
        if _closing_campaigns:
            return {
                "success": False,
                "message": f"{command} unavailable while a final LANTERN firmware row is pending",
            }
    return None


def cmd_clear(_: Optional[dict]) -> Dict[str, Any]:
    """Delete all current LANTERN masters/details at one worker-serialized boundary."""
    global _clear_count
    global _last_maintenance

    busy = _maintenance_requires_stopped("CLEAR")
    if busy is not None:
        return busy

    with _maintenance_lock:
        ingress_drained = _drain_queue(_fragment_queue)
        persist_drained = _drain_queue(_persist_queue)
        try:
            with open_db() as conn:
                cur = conn.cursor()
                cur.execute(
                    "DELETE FROM campaign_detail WHERE campaign_type = %s",
                    (CAMPAIGN_TYPE_LANTERN,),
                )
                detail_count = int(cur.rowcount or 0)
                cur.execute(
                    "DELETE FROM campaign_master WHERE campaign_type = %s",
                    (CAMPAIGN_TYPE_LANTERN,),
                )
                master_count = int(cur.rowcount or 0)
                cur.execute(
                    "DELETE FROM config WHERE config_key = %s",
                    (PHOTONS_RECOVERY_CONFIG_KEY,),
                )
        except Exception as exc:
            logging.exception("❌ [photons] CLEAR failed")
            return {"success": False, "message": str(exc)}

    result = {
        "action": "CLEAR",
        "at_utc": _utc_now_z(),
        "campaign_type": CAMPAIGN_TYPE_LANTERN,
        "campaign_details_deleted": detail_count,
        "campaign_master_deleted": master_count,
        "fragment_ingress_drained": ingress_drained,
        "pending_persistence_drained": persist_drained,
        "legacy_photons_rows_retained": True,
        "instrument_measurement_continues": True,
    }
    with _state_lock:
        _clear_count += 1
        _last_maintenance = copy.deepcopy(result)
    logging.warning(
        "🗑️ [photons] LANTERN history cleared: %d detail row(s), %d campaign master row(s); "
        "%d queued ingress and %d pending persistence item(s) drained. "
        "The optical instrument continues measuring.",
        int(detail_count),
        int(master_count),
        int(ingress_drained),
        int(persist_drained),
    )
    return {"success": True, "message": "OK", "payload": result}


def cmd_delete(args: Optional[dict]) -> Dict[str, Any]:
    """Delete one stopped LANTERN campaign and its associated details."""
    global _delete_count
    global _last_maintenance

    campaign_name = str((args or {}).get("campaign") or "").strip()
    if not campaign_name:
        return {"success": False, "message": "DELETE requires 'campaign' argument"}

    busy = _campaign_control_gate("DELETE")
    if busy is not None:
        return busy
    with _campaign_lock:
        if _active_campaign is not None and _active_campaign.get("campaign") == campaign_name:
            return {"success": False, "message": f"Campaign '{campaign_name}' is active — STOP it first"}
        if any(item.get("campaign") == campaign_name for item in _closing_campaigns):
            return {"success": False, "message": f"Campaign '{campaign_name}' final firmware row is still pending"}

    recovery_checkpoint = _read_photons_recovery_config()
    if recovery_checkpoint is not None:
        recovery_source = _photons_recovery_source_row(recovery_checkpoint)
        if recovery_source is None:
            return {
                "success": False,
                "message": (
                    f"config.{PHOTONS_RECOVERY_CONFIG_KEY} has no matching durable "
                    "PHOTONS source; refusing campaign deletion"
                ),
            }
        if str(recovery_source.get("campaign") or "") == campaign_name:
            return {
                "success": False,
                "message": (
                    f"Campaign '{campaign_name}' still owns current PHOTONS recovery "
                    "custody — retry after the next ambient PHOTONS row"
                ),
            }

    with _maintenance_lock:
        if campaign_name in _pending_persist_campaign_names():
            return {
                "success": False,
                "message": f"Campaign '{campaign_name}' still has ordered persistence pending; retry shortly",
            }
        try:
            with open_db(row_dict=True) as conn:
                cur = conn.cursor()
                cur.execute(
                    """
                    SELECT DISTINCT ref.campaign
                    FROM campaign_master AS target
                    JOIN campaign_master AS ref
                      ON (ref.payload ->> 'baseline_campaign_id')::bigint = target.id
                    WHERE target.campaign_type = %s
                      AND target.campaign = %s
                    ORDER BY ref.campaign
                    """,
                    (CAMPAIGN_TYPE_LANTERN, campaign_name),
                )
                referenced_by = [str(row["campaign"]) for row in cur.fetchall()]
                if referenced_by:
                    return {
                        "success": False,
                        "message": (
                            f"Campaign '{campaign_name}' is used as a baseline by: "
                            + ", ".join(referenced_by)
                        ),
                    }
                cur.execute(
                    "DELETE FROM campaign_detail WHERE campaign_type = %s AND campaign = %s",
                    (CAMPAIGN_TYPE_LANTERN, campaign_name),
                )
                detail_count = int(cur.rowcount or 0)
                cur.execute(
                    "DELETE FROM campaign_master WHERE campaign_type = %s AND campaign = %s",
                    (CAMPAIGN_TYPE_LANTERN, campaign_name),
                )
                master_count = int(cur.rowcount or 0)
        except Exception as exc:
            logging.exception("❌ [photons] DELETE failed for campaign '%s'", campaign_name)
            return {"success": False, "message": str(exc)}

    if master_count == 0:
        return {"success": False, "message": f"No LANTERN campaign named '{campaign_name}'"}

    server_args = {"campaign_type": CAMPAIGN_TYPE_LANTERN, "campaign": campaign_name}
    try:
        server_resp = send_command(
            machine="SERVER", subsystem="SYSTEM", command="DELETE_CAMPAIGN", args=server_args
        )
    except Exception as exc:
        server_resp = {"success": False, "message": str(exc)}

    result = {
        "action": "DELETE",
        "at_utc": _utc_now_z(),
        "campaign_type": CAMPAIGN_TYPE_LANTERN,
        "campaign": campaign_name,
        "campaign_master_deleted": master_count,
        "campaign_details_deleted": detail_count,
        "server_delete_success": bool(server_resp.get("success")) if isinstance(server_resp, dict) else False,
    }
    with _state_lock:
        _delete_count += 1
        _last_maintenance = copy.deepcopy(result)
    return {"success": True, "message": "OK", "payload": result}


def cmd_truncate(_: Optional[dict]) -> Dict[str, Any]:
    """Destructively truncate all campaign history, regardless of subsystem type."""
    global _truncate_count
    global _last_maintenance

    busy = _maintenance_requires_stopped("TRUNCATE")
    if busy is not None:
        return busy

    with _maintenance_lock:
        ingress_drained = _drain_queue(_fragment_queue)
        persist_drained = _drain_queue(_persist_queue)
        try:
            with open_db() as conn:
                cur = conn.cursor()
                cur.execute(
                    "TRUNCATE TABLE campaign_detail, campaign_master RESTART IDENTITY"
                )
                cur.execute(
                    "DELETE FROM config WHERE config_key IN (%s, %s)",
                    ("CLOCKS_RECOVERY", PHOTONS_RECOVERY_CONFIG_KEY),
                )
        except Exception as exc:
            logging.exception("❌ [photons] TRUNCATE failed")
            return {"success": False, "message": str(exc)}

    server_args = {
        "source": "PHOTONS.TRUNCATE",
        "postgres_tables_truncated": ["campaign_detail", "campaign_master"],
        "postgres_identity_restarted": True,
        "ambient_campaign_details_retained": False,
    }
    try:
        server_resp = send_command(
            machine="SERVER", subsystem="SYSTEM", command="TRUNCATE", args=server_args
        )
    except Exception as exc:
        logging.exception("⚠️ [photons] SERVER.SYSTEM.TRUNCATE failed after local cutover")
        server_resp = {"success": False, "message": str(exc)}

    result = {
        "action": "TRUNCATE",
        "at_utc": _utc_now_z(),
        "tables_truncated": ["campaign_detail", "campaign_master"],
        "identity_restarted": True,
        "ambient_campaign_details_retained": False,
        "fragment_ingress_drained": ingress_drained,
        "pending_persistence_drained": persist_drained,
        "server_truncate_success": bool(server_resp.get("success")) if isinstance(server_resp, dict) else False,
        "server_truncate_message": server_resp.get("message") if isinstance(server_resp, dict) else None,
    }
    with _state_lock:
        _truncate_count += 1
        _last_maintenance = copy.deepcopy(result)
    logging.warning(
        "🧨 [photons] LANTERN database epoch truncated and identities restarted; "
        "%d queued ingress and %d pending persistence item(s) drained; "
        "server truncate success=%s",
        int(ingress_drained),
        int(persist_drained),
        bool(result["server_truncate_success"]),
    )
    return {"success": True, "message": "OK", "payload": result}


def cmd_set_lap_baseline_ns(args: Optional[dict]) -> Dict[str, Any]:
    """Persist and realize one operator-authored LAP_BASELINE_NS reference."""
    global _lap_baseline_ns
    global _lap_baseline_fs
    global _standard_lap_ns
    global _standard_lap_ps
    global _teensy_standard_configured
    global _lap_baseline_set_count
    global _last_maintenance

    busy = _campaign_control_gate("SET_LAP_BASELINE_NS")
    if busy is not None:
        return busy

    raw = (args or {}).get("ns")
    if raw is None:
        raw = (args or {}).get("lap_baseline_ns")
    if raw is None:
        return {"success": False, "message": "SET_LAP_BASELINE_NS requires ns=<nanoseconds>"}
    try:
        baseline_text, baseline_fs = _normalize_lap_baseline_ns(
            raw, path="SET_LAP_BASELINE_NS.ns"
        )
        standard_text, standard_ps = _compat_standard_from_baseline_fs(baseline_fs)
    except Exception as exc:
        return {"success": False, "message": str(exc)}

    requested_at = _utc_now_z()
    with _maintenance_lock:
        # Persist desired reference first. If transport realization is interrupted,
        # the next PHOTONS process start converges the Teensy from config.PHOTONS.
        try:
            with open_db() as conn:
                cur = conn.cursor()
                cur.execute(
                    """
                    UPDATE config
                    SET payload = payload || jsonb_build_object(
                        'lap_baseline_ns', %s::text,
                        'lap_baseline_fs', %s::bigint,
                        'lap_baseline_source', 'OPERATOR',
                        'lap_baseline_updated_at_utc', %s::text
                    )
                    WHERE config_key = 'PHOTONS'
                    """,
                    (baseline_text, baseline_fs, requested_at),
                )
                if cur.rowcount != 1:
                    raise RuntimeError("config.PHOTONS LAP_BASELINE_NS update did not affect exactly one row")
        except Exception as exc:
            logging.exception("❌ [photons] SET_LAP_BASELINE_NS persistence failed")
            return {"success": False, "message": str(exc)}

        try:
            response = send_command(
                machine="TEENSY",
                subsystem=SUBSYSTEM,
                command="SET_LAP_BASELINE_NS",
                args={"lap_baseline_ns": baseline_text},
            )
            payload = response.get("payload") if isinstance(response, dict) else None
            if (
                not isinstance(response, dict)
                or not response.get("success")
                or not isinstance(payload, dict)
                or payload.get("status") != "lap_baseline_set"
            ):
                raise RuntimeError(f"Teensy PHOTONS.SET_LAP_BASELINE_NS rejected: {response!r}")
            echoed_fs = _require_int(
                payload.get("lap_baseline_fs"),
                "PHOTONS.SET_LAP_BASELINE_NS.payload.lap_baseline_fs",
                minimum=1,
            )
            if echoed_fs != baseline_fs:
                raise RuntimeError(
                    f"Teensy LAP_BASELINE_NS echo mismatch: requested_fs={baseline_fs} echoed_fs={echoed_fs}"
                )
        except Exception as exc:
            failure = {
                "action": "SET_LAP_BASELINE_NS",
                "requested_at_utc": requested_at,
                "success": False,
                "config_persisted": True,
                "producer_realized": False,
                "lap_baseline_ns": baseline_text,
                "lap_baseline_fs": baseline_fs,
                "error": str(exc),
                "recovery_action": "retry command or restart PHOTONS to converge from config.PHOTONS",
            }
            with _state_lock:
                _last_maintenance = copy.deepcopy(failure)
            logging.exception("⚠️ [photons] LAP_BASELINE_NS persisted but Teensy realization failed")
            return {"success": False, "message": str(exc), "payload": failure}

        previous_text = _lap_baseline_ns
        previous_fs = _lap_baseline_fs
        _lap_baseline_ns = baseline_text
        _lap_baseline_fs = baseline_fs
        _standard_lap_ns = standard_text
        _standard_lap_ps = standard_ps
        _teensy_standard_configured = True

    result = {
        "action": "SET_LAP_BASELINE_NS",
        "requested_at_utc": requested_at,
        "success": True,
        "changed": previous_fs != baseline_fs,
        "previous_lap_baseline_ns": previous_text,
        "previous_lap_baseline_fs": previous_fs,
        "lap_baseline_ns": baseline_text,
        "lap_baseline_fs": baseline_fs,
        "config_key": "PHOTONS",
        "config_field": "lap_baseline_ns",
        "producer_realized": True,
        "physical_measurement_unchanged": True,
        "welford_preserved": True,
        "better_buckets_history_preserved": True,
        "effect": "REFERENCE_ONLY_NEXT_FRAGMENT_RECOMPUTES_PPB_AND_RESIDUALS",
    }
    with _state_lock:
        _lap_baseline_set_count += 1
        _last_maintenance = copy.deepcopy(result)
    logging.warning(
        "📐 [photons] LAP_BASELINE_NS %s -> %s ns (%d fs); physical N/T and Better-Buckets custody preserved",
        previous_text or "UNSET", baseline_text, baseline_fs,
    )
    return {"success": True, "message": "OK", "payload": result}


def cmd_set_lap_baseline_ns(args: Optional[dict]) -> Dict[str, Any]:
    """Persist and realize one operator-authored LAP_BASELINE_NS reference."""
    global _lap_baseline_ns
    global _lap_baseline_fs
    global _standard_lap_ns
    global _standard_lap_ps
    global _teensy_standard_configured
    global _lap_baseline_set_count
    global _last_maintenance

    busy = _campaign_control_gate("SET_LAP_BASELINE_NS")
    if busy is not None:
        return busy

    raw = (args or {}).get("ns")
    if raw is None:
        raw = (args or {}).get("lap_baseline_ns")
    if raw is None:
        return {"success": False, "message": "SET_LAP_BASELINE_NS requires ns=<nanoseconds>"}
    try:
        baseline_text, baseline_fs = _normalize_lap_baseline_ns(
            raw, path="SET_LAP_BASELINE_NS.ns"
        )
        standard_text, standard_ps = _compat_standard_from_baseline_fs(baseline_fs)
    except Exception as exc:
        return {"success": False, "message": str(exc)}

    requested_at = _utc_now_z()
    with _maintenance_lock:
        # Persist desired reference first. If transport realization is interrupted,
        # the next PHOTONS process start converges the Teensy from config.PHOTONS.
        try:
            with open_db() as conn:
                cur = conn.cursor()
                cur.execute(
                    """
                    UPDATE config
                    SET payload = payload || jsonb_build_object(
                        'lap_baseline_ns', %s::text,
                        'lap_baseline_fs', %s::bigint,
                        'lap_baseline_source', 'OPERATOR',
                        'lap_baseline_updated_at_utc', %s::text
                    )
                    WHERE config_key = 'PHOTONS'
                    """,
                    (baseline_text, baseline_fs, requested_at),
                )
                if cur.rowcount != 1:
                    raise RuntimeError("config.PHOTONS LAP_BASELINE_NS update did not affect exactly one row")
        except Exception as exc:
            logging.exception("❌ [photons] SET_LAP_BASELINE_NS persistence failed")
            return {"success": False, "message": str(exc)}

        try:
            response = send_command(
                machine="TEENSY",
                subsystem=SUBSYSTEM,
                command="SET_LAP_BASELINE_NS",
                args={"lap_baseline_ns": baseline_text},
            )
            payload = response.get("payload") if isinstance(response, dict) else None
            if (
                not isinstance(response, dict)
                or not response.get("success")
                or not isinstance(payload, dict)
                or payload.get("status") != "lap_baseline_set"
            ):
                raise RuntimeError(f"Teensy PHOTONS.SET_LAP_BASELINE_NS rejected: {response!r}")
            echoed_fs = _require_int(
                payload.get("lap_baseline_fs"),
                "PHOTONS.SET_LAP_BASELINE_NS.payload.lap_baseline_fs",
                minimum=1,
            )
            if echoed_fs != baseline_fs:
                raise RuntimeError(
                    f"Teensy LAP_BASELINE_NS echo mismatch: requested_fs={baseline_fs} echoed_fs={echoed_fs}"
                )
        except Exception as exc:
            failure = {
                "action": "SET_LAP_BASELINE_NS",
                "requested_at_utc": requested_at,
                "success": False,
                "config_persisted": True,
                "producer_realized": False,
                "lap_baseline_ns": baseline_text,
                "lap_baseline_fs": baseline_fs,
                "error": str(exc),
                "recovery_action": "retry command or restart PHOTONS to converge from config.PHOTONS",
            }
            with _state_lock:
                _last_maintenance = copy.deepcopy(failure)
            logging.exception("⚠️ [photons] LAP_BASELINE_NS persisted but Teensy realization failed")
            return {"success": False, "message": str(exc), "payload": failure}

        previous_text = _lap_baseline_ns
        previous_fs = _lap_baseline_fs
        _lap_baseline_ns = baseline_text
        _lap_baseline_fs = baseline_fs
        _standard_lap_ns = standard_text
        _standard_lap_ps = standard_ps
        _teensy_standard_configured = True

    result = {
        "action": "SET_LAP_BASELINE_NS",
        "requested_at_utc": requested_at,
        "success": True,
        "changed": previous_fs != baseline_fs,
        "previous_lap_baseline_ns": previous_text,
        "previous_lap_baseline_fs": previous_fs,
        "lap_baseline_ns": baseline_text,
        "lap_baseline_fs": baseline_fs,
        "config_key": "PHOTONS",
        "config_field": "lap_baseline_ns",
        "producer_realized": True,
        "physical_measurement_unchanged": True,
        "welford_preserved": True,
        "better_buckets_history_preserved": True,
        "effect": "REFERENCE_ONLY_NEXT_FRAGMENT_RECOMPUTES_PPB_AND_RESIDUALS",
    }
    with _state_lock:
        _lap_baseline_set_count += 1
        _last_maintenance = copy.deepcopy(result)
    logging.warning(
        "📐 [photons] LAP_BASELINE_NS %s -> %s ns (%d fs); physical N/T and Better-Buckets custody preserved",
        previous_text or "UNSET", baseline_text, baseline_fs,
    )
    return {"success": True, "message": "OK", "payload": result}


def cmd_set_baseline(args: Optional[dict]) -> Dict[str, Any]:
    """Relate the active LANTERN campaign to another campaign by durable ID."""
    global _baseline_set_count
    global _last_campaign_transition

    busy = _campaign_control_gate("SET_BASELINE")
    if busy is not None:
        return busy

    baseline_name = str((args or {}).get("campaign") or "").strip()
    if not baseline_name:
        return {"success": False, "message": "SET_BASELINE requires 'campaign' argument"}

    with _campaign_lock:
        if _active_campaign is None:
            return {
                "success": False,
                "message": "SET_BASELINE requires an active LANTERN campaign",
            }

        current = copy.deepcopy(_active_campaign)
        try:
            with open_db(row_dict=True) as conn:
                cur = conn.cursor()
                cur.execute(
                    """
                    SELECT id, campaign, active, payload
                    FROM campaign_master
                    WHERE campaign_type = %s
                      AND campaign = %s
                    ORDER BY ts DESC, id DESC
                    LIMIT 1
                    """,
                    (CAMPAIGN_TYPE_LANTERN, baseline_name),
                )
                baseline = cur.fetchone()
                if baseline is None:
                    return {
                        "success": False,
                        "message": f"No LANTERN campaign named '{baseline_name}'",
                    }
                if int(baseline["id"]) == int(current["campaign_id"]):
                    return {
                        "success": False,
                        "message": "A campaign cannot use itself as its baseline",
                    }
                if bool(baseline["active"]):
                    return {
                        "success": False,
                        "message": "Baseline campaign must be stopped before it can be referenced",
                    }

                baseline_payload = baseline["payload"]
                if isinstance(baseline_payload, str):
                    baseline_payload = json.loads(baseline_payload)
                baseline_report = (
                    baseline_payload.get("report")
                    if isinstance(baseline_payload, dict)
                    else None
                )
                if not isinstance(baseline_report, dict) or not baseline_report:
                    return {
                        "success": False,
                        "message": f"Campaign '{baseline_name}' has no persisted PHOTONS report",
                    }

                cur.execute(
                    """
                    UPDATE campaign_master
                    SET payload = jsonb_set(
                        payload,
                        '{baseline_campaign_id}',
                        to_jsonb(%s::bigint),
                        true
                    )
                    WHERE id = %s
                      AND campaign_type = %s
                      AND active = true
                    """,
                    (
                        int(baseline["id"]),
                        int(current["campaign_id"]),
                        CAMPAIGN_TYPE_LANTERN,
                    ),
                )
                if cur.rowcount != 1:
                    raise RuntimeError(
                        "active LANTERN baseline relationship was not updated exactly once"
                    )

        except Exception as exc:
            logging.exception("❌ [photons] SET_BASELINE failed")
            return {"success": False, "message": str(exc)}

        _active_campaign["baseline_campaign_id"] = int(baseline["id"])
        _active_campaign["baseline_campaign"] = str(baseline["campaign"])

    transition = {
        "action": "SET_BASELINE",
        "at_utc": _utc_now_z(),
        "campaign": current["campaign"],
        "campaign_id": int(current["campaign_id"]),
        "baseline_campaign": str(baseline["campaign"]),
        "baseline_campaign_id": int(baseline["id"]),
    }
    with _state_lock:
        _baseline_set_count += 1
        _last_campaign_transition = copy.deepcopy(transition)

    logging.info(
        "✅ [photons] LANTERN campaign '%s' baseline -> '%s'",
        current["campaign"],
        baseline["campaign"],
    )
    return {"success": True, "message": "OK", "payload": transition}


def cmd_baseline_info(_: Optional[dict]) -> Dict[str, Any]:
    """Return the active LANTERN campaign's baseline relationship."""
    try:
        relation = _baseline_relation_for_active_campaign()
    except Exception as exc:
        logging.exception("❌ [photons] BASELINE_INFO failed")
        return {"success": False, "message": str(exc)}

    if relation is None:
        return {
            "success": True,
            "message": "OK",
            "payload": {"baseline_set": False},
        }

    return {
        "success": True,
        "message": "OK",
        "payload": {"baseline_set": True, **relation},
    }


def cmd_list_campaigns(_: Optional[dict]) -> Dict[str, Any]:
    """List LANTERN campaign masters and baseline relationships."""
    try:
        with open_db(row_dict=True) as conn:
            cur = conn.cursor()
            cur.execute(
                """
                SELECT
                    master.id,
                    master.campaign_type,
                    master.campaign,
                    master.active,
                    master.ts,
                    master.payload,
                    baseline.campaign AS baseline_campaign
                FROM campaign_master AS master
                LEFT JOIN campaign_master AS baseline
                  ON baseline.id = (master.payload ->> 'baseline_campaign_id')::bigint
                 AND baseline.campaign_type = master.campaign_type
                WHERE master.campaign_type = %s
                ORDER BY master.ts ASC, master.id ASC
                """,
                (CAMPAIGN_TYPE_LANTERN,),
            )
            rows = cur.fetchall()
    except Exception as exc:
        logging.exception("❌ [photons] LIST_CAMPAIGNS query failed")
        return {"success": False, "message": str(exc)}

    campaigns: List[Dict[str, Any]] = []
    for row in rows:
        payload = row["payload"]
        if isinstance(payload, str):
            payload = json.loads(payload)
        payload = payload if isinstance(payload, dict) else {}
        report = payload.get("report")
        report = report if isinstance(report, dict) else {}
        instrument_stats = report.get("instrument_stats")
        instrument_stats = instrument_stats if isinstance(instrument_stats, dict) else {}
        race = report.get("race")
        race = race if isinstance(race, dict) else {}
        race_flight = race.get("flight_ns")
        race_flight = race_flight if isinstance(race_flight, dict) else {}

        campaigns.append(
            {
                "campaign_id": int(row["id"]),
                "campaign_type": row["campaign_type"],
                "campaign": row["campaign"],
                "active": bool(row["active"]),
                "baseline_campaign": row.get("baseline_campaign"),
                "started_at": payload.get("started_at"),
                "stopped_at": payload.get("stopped_at"),
                "interrupted_at": payload.get("interrupted_at"),
                "interruption_reason": payload.get("interruption_reason"),
                "start_after_sequence": payload.get("start_after_sequence"),
                "stop_after_sequence": payload.get("stop_after_sequence"),
                "location": payload.get("location"),
                "latest_sequence": report.get("sequence"),
                "latest_pps_count": report.get("pps_count"),
                "latest_accepted_count": report.get("accepted_count"),
                "latest_excluded_count": report.get("excluded_count"),
                # mean_flight_ns is the cumulative accepted population.  The
                # race_* values are the exact producer-authored latest 1 s batch.
                "latest_mean_flight_ns": instrument_stats.get("mean_flight_ns"),
                "latest_race_cadence_ticks": race.get("cadence_ticks_this_fragment"),
                "latest_race_attempts": race.get("attempts_this_fragment"),
                "latest_race_completed": race.get("completed_this_fragment"),
                "latest_race_missed": race.get("missed_this_fragment"),
                "latest_race_flight_mean_ns": race_flight.get("mean"),
                "latest_race_flight_stderr_ns": race_flight.get("stderr"),
                # Legacy field remains during the lap->race naming migration.
                "latest_mean_lap_ns": instrument_stats.get("mean_lap_ns"),
            }
        )

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "campaign_type": CAMPAIGN_TYPE_LANTERN,
            "count": len(campaigns),
            "campaigns": campaigns,
        },
    }


def _stats_reset_campaign_identity(state: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    campaign = state.get("campaign")
    if campaign is None:
        return None
    campaign = _require_dict(campaign, "PHOTONS.campaign")
    return {
        "campaign": str(campaign.get("campaign") or ""),
        "campaign_id": _require_int(
            campaign.get("campaign_id"), "PHOTONS.campaign.campaign_id", minimum=1
        ),
        "start_after_sequence": _require_int(
            campaign.get("start_after_sequence"),
            "PHOTONS.campaign.start_after_sequence",
            minimum=1,
        ),
    }


def _stats_epoch_snapshot_from_state(state: Dict[str, Any]) -> Dict[str, Any]:
    state = _require_dict(state, "PHOTONS")
    if state.get("schema") != PHOTONS_SCHEMA:
        raise ValueError(f"unsupported PHOTONS schema {state.get('schema')!r}")

    instrument = _require_dict(state.get("photons"), "PHOTONS.photons")
    stats = _require_dict(instrument.get("stats"), "PHOTONS.photons.stats")
    return {
        "sequence": _require_int(state.get("sequence"), "PHOTONS.sequence", minimum=1),
        "publish_count": _require_int(
            state.get("publish_count"), "PHOTONS.publish_count", minimum=1
        ),
        "reset_count": _require_int(
            stats.get("reset_count"), "PHOTONS.photons.stats.reset_count"
        ),
        "update_count": _require_int(
            stats.get("update_count"),
            "PHOTONS.photons.stats.update_count",
            minimum=1,
        ),
        "lap_baseline_fs": _require_int(
            stats.get("lap_baseline_fs"),
            "PHOTONS.photons.stats.lap_baseline_fs",
            minimum=1,
        ),
        "standard_lap_ps": _require_int(
            stats.get("standard_lap_ps"),
            "PHOTONS.photons.stats.standard_lap_ps",
            minimum=1,
        ),
        "lap_count": _require_int(
            stats.get("lap_count"), "PHOTONS.photons.stats.lap_count"
        ),
        "total_lap_gnss_ns": _require_int(
            stats.get("total_lap_gnss_ns"),
            "PHOTONS.photons.stats.total_lap_gnss_ns",
        ),
        "custody_lap_count": _require_int(
            stats.get("custody_lap_count"),
            "PHOTONS.photons.stats.custody_lap_count",
        ),
        "custody_total_lap_gnss_ns": _require_int(
            stats.get("custody_total_lap_gnss_ns"),
            "PHOTONS.photons.stats.custody_total_lap_gnss_ns",
        ),
        "campaign": _stats_reset_campaign_identity(state),
    }


def _latest_stats_epoch() -> Dict[str, Any]:
    with _state_lock:
        latest = copy.deepcopy(_latest_photons)
    if not isinstance(latest, dict):
        raise RuntimeError("PHOTONS canonical heartbeat is unavailable")
    return _stats_epoch_snapshot_from_state(latest)


def _stats_reset_birth_court(
    state: Dict[str, Any],
    *,
    before: Dict[str, Any],
    expected_reset_count: int,
) -> Dict[str, Any]:
    """Prove that one canonical PHOTONS row is row 1 of the requested stats epoch."""
    state = copy.deepcopy(_require_dict(state, "PHOTONS stats-reset birth row"))
    snapshot = _stats_epoch_snapshot_from_state(state)
    if snapshot["reset_count"] != expected_reset_count:
        raise ValueError(
            "PHOTONS STATS_RESET birth row has wrong reset_count: "
            f"expected={expected_reset_count} observed={snapshot['reset_count']}"
        )
    if snapshot["update_count"] != 1:
        raise ValueError(
            "PHOTONS STATS_RESET birth row is not update_count=1: "
            f"observed={snapshot['update_count']}"
        )
    if snapshot["sequence"] <= int(before["sequence"]):
        raise ValueError(
            "PHOTONS physical publication chronology did not advance across STATS_RESET"
        )
    if snapshot["lap_baseline_fs"] != int(before["lap_baseline_fs"]):
        raise ValueError("PHOTONS STATS_RESET changed LAP_BASELINE_NS")
    if snapshot["standard_lap_ps"] != int(before["standard_lap_ps"]):
        raise ValueError("PHOTONS STATS_RESET changed compatibility standard_lap_ps")
    if snapshot["custody_lap_count"] < int(before["custody_lap_count"]):
        raise ValueError("PHOTONS STATS_RESET regressed monotonic accepted-lap custody")
    if snapshot["custody_total_lap_gnss_ns"] < int(
        before["custody_total_lap_gnss_ns"]
    ):
        raise ValueError("PHOTONS STATS_RESET regressed monotonic lap-time custody")
    if snapshot["campaign"] != before.get("campaign"):
        raise ValueError(
            "PHOTONS STATS_RESET changed LANTERN identity/origin: "
            f"before={before.get('campaign')!r} after={snapshot['campaign']!r}"
        )

    instrument = _require_dict(state.get("photons"), "PHOTONS.photons")
    if _require_bool(instrument.get("snapshot_ok"), "PHOTONS.photons.snapshot_ok") is not True:
        raise ValueError("PHOTONS STATS_RESET birth row is not a coherent snapshot")
    if _require_bool(instrument.get("valid"), "PHOTONS.photons.valid") is not True:
        raise ValueError("PHOTONS STATS_RESET birth row is not instrument-valid")

    stats = _require_dict(instrument.get("stats"), "PHOTONS.photons.stats")
    science = _require_dict(instrument.get("science"), "PHOTONS.photons.science")
    projection = _require_dict(
        instrument.get("projection"), "PHOTONS.photons.projection"
    )
    raw_cycles = _require_dict(
        instrument.get("raw_cycles"), "PHOTONS.photons.raw_cycles"
    )
    accepted = _require_dict(science.get("accepted"), "PHOTONS.science.accepted")
    excluded = _require_dict(science.get("excluded"), "PHOTONS.science.excluded")
    reasons = _require_dict(
        science.get("exclusion_reasons"), "PHOTONS.science.exclusion_reasons"
    )

    lap_welford = _validate_recovery_welford(
        stats.get("lap_time"), "PHOTONS.stats.lap_time"
    )
    accepted_raw = _validate_recovery_welford(
        accepted.get("raw_cycles"), "PHOTONS.science.accepted.raw_cycles"
    )
    accepted_projected = _validate_recovery_welford(
        accepted.get("projected_lap_ns"),
        "PHOTONS.science.accepted.projected_lap_ns",
    )
    excluded_raw = _validate_recovery_welford(
        excluded.get("raw_cycles"), "PHOTONS.science.excluded.raw_cycles"
    )
    excluded_projected = _validate_recovery_welford(
        excluded.get("projected_lap_ns"),
        "PHOTONS.science.excluded.projected_lap_ns",
    )

    lap_count = int(snapshot["lap_count"])
    total_ns = int(snapshot["total_lap_gnss_ns"])
    accepted_count = _require_int(
        accepted.get("count"), "PHOTONS.science.accepted.count"
    )
    excluded_count = _require_int(
        excluded.get("count"), "PHOTONS.science.excluded.count"
    )
    pending_count = 1 if _require_bool(
        science.get("seed_pending"), "PHOTONS.science.seed_pending"
    ) else 0
    candidate_count = _require_int(
        science.get("candidate_count"), "PHOTONS.science.candidate_count"
    )

    accepted_populations = {
        lap_count,
        lap_welford["n"],
        accepted_count,
        accepted_raw["n"],
        accepted_projected["n"],
    }
    if len(accepted_populations) != 1:
        raise ValueError(
            "PHOTONS STATS_RESET birth accepted populations disagree: "
            f"{sorted(accepted_populations)!r}"
        )
    if not _welford_equivalent(lap_welford, accepted_projected):
        raise ValueError(
            "PHOTONS STATS_RESET birth lap/accepted-projected Welfords differ"
        )
    if excluded_raw["n"] != excluded_count:
        raise ValueError(
            "PHOTONS STATS_RESET birth excluded raw Welford population disagrees"
        )
    if excluded_projected["n"] > excluded_count:
        raise ValueError(
            "PHOTONS STATS_RESET birth excluded projected population exceeds exclusions"
        )
    if candidate_count != accepted_count + excluded_count + pending_count:
        raise ValueError("PHOTONS STATS_RESET birth candidate accounting does not close")

    completed_laps = _require_int(
        raw_cycles.get("completed_lap_count"),
        "PHOTONS.raw_cycles.completed_lap_count",
    )
    attempts = _require_int(
        projection.get("attempt_count"), "PHOTONS.projection.attempt_count"
    )
    successes = _require_int(
        projection.get("success_count"), "PHOTONS.projection.success_count"
    )
    rejects = _require_int(
        projection.get("reject_count"), "PHOTONS.projection.reject_count"
    )
    if completed_laps != candidate_count or attempts != candidate_count:
        raise ValueError(
            "PHOTONS STATS_RESET birth raw/projection populations do not start at epoch origin"
        )
    if attempts != successes + rejects:
        raise ValueError("PHOTONS STATS_RESET birth projection accounting does not close")

    reason_counts = {
        name: _require_int(value, f"PHOTONS.science.exclusion_reasons.{name}")
        for name, value in reasons.items()
        if not name.endswith("_this_fragment")
    }
    if sum(reason_counts.values()) != excluded_count:
        raise ValueError(
            "PHOTONS STATS_RESET birth exclusion-reason population does not close"
        )
    if rejects != reason_counts.get("projection_invalid", 0):
        raise ValueError(
            "PHOTONS STATS_RESET birth projection rejects disagree with reason custody"
        )

    welford_grand_ratio: Optional[Dict[str, Any]] = None
    if lap_count == 0:
        if total_ns != 0:
            raise ValueError("PHOTONS STATS_RESET birth has zero N with nonzero total")
    else:
        if total_ns == 0:
            raise ValueError("PHOTONS STATS_RESET birth has nonzero N with zero total")
        ratio_mean = float(total_ns) / float(lap_count)
        welford_grand_ratio = _welford_grand_ratio_diagnostic(
            welford_mean_ns=float(lap_welford["mean"]),
            ratio_mean_ns=ratio_mean,
            lap_baseline_fs=int(snapshot["lap_baseline_fs"]),
        )

    current_sequence = _require_int(
        stats.get("rolling_ppb_current_sequence"),
        "PHOTONS.stats.rolling_ppb_current_sequence",
        minimum=1,
    )
    admitted = _require_bool(
        stats.get("rolling_ppb_endpoint_admitted"),
        "PHOTONS.stats.rolling_ppb_endpoint_admitted",
    )
    if current_sequence != 1 or not admitted:
        raise ValueError(
            "PHOTONS STATS_RESET birth did not establish Better-Buckets endpoint 1"
        )

    buckets = _require_dict(stats.get("ppb_buckets"), "PHOTONS.stats.ppb_buckets")
    if lap_count == 0:
        if any(isinstance(buckets.get(name), dict) for name in PPB_BUCKET_NAMES):
            raise ValueError("PHOTONS STATS_RESET empty birth published nonempty PPB bucket")
        bucket_proof: Dict[str, Any] = {}
    else:
        bucket_proof = {}
        reference_ppb: Optional[float] = None
        for name in PPB_BUCKET_NAMES:
            bucket = _require_dict(
                buckets.get(name), f"PHOTONS.stats.ppb_buckets.{name}"
            )
            bucket_n = _require_int(
                bucket.get("sample_count"),
                f"PHOTONS.stats.ppb_buckets.{name}.sample_count",
                minimum=1,
            )
            bucket_ppb = _require_float(
                bucket.get("ppb"), f"PHOTONS.stats.ppb_buckets.{name}.ppb"
            )
            if bucket_n != lap_count:
                raise ValueError(
                    f"PHOTONS STATS_RESET birth {name} N={bucket_n} != lap_count={lap_count}"
                )
            if reference_ppb is None:
                reference_ppb = bucket_ppb
            elif abs(bucket_ppb - reference_ppb) > PHOTONS_RECOVERY_VERIFY_TOLERANCE:
                raise ValueError(
                    "PHOTONS STATS_RESET birth Better-Buckets do not share epoch origin"
                )
            bucket_proof[name] = {"sample_count": bucket_n, "ppb": bucket_ppb}

    return {
        **snapshot,
        "candidate_count": candidate_count,
        "accepted_count": accepted_count,
        "excluded_count": excluded_count,
        "pending_seed_count": pending_count,
        "projection_attempt_count": attempts,
        "projection_reject_count": rejects,
        "reason_counts": reason_counts,
        "welford_grand_ratio_diagnostic": copy.deepcopy(welford_grand_ratio),
        "ppb_buckets": bucket_proof,
        "custody_lap_delta_from_before": (
            int(snapshot["custody_lap_count"]) - int(before["custody_lap_count"])
        ),
        "custody_total_delta_from_before": (
            int(snapshot["custody_total_lap_gnss_ns"])
            - int(before["custody_total_lap_gnss_ns"])
        ),
    }


PPB_BUCKET_NAMES = ("10_min", "60_min", "8_hour", "24_hour", "total")


def _fetch_teensy_stats_reset_report() -> Dict[str, Any]:
    """Return the authoritative firmware STATS_RESET chronology surface."""
    response = send_command(
        machine="TEENSY",
        subsystem=SUBSYSTEM,
        command="REPORT_STATS",
        retries=1,
        retry_delay_s=0.0,
    )
    payload = response.get("payload") if isinstance(response, dict) else None
    if not isinstance(response, dict) or not response.get("success") or not isinstance(payload, dict):
        raise RuntimeError(f"Teensy PHOTONS.REPORT_STATS failed: {response!r}")
    return copy.deepcopy(payload)


def _recovery_report_surface() -> Dict[str, Any]:
    with _recovery_lock:
        payload = {
            "status": copy.deepcopy(_recovery_status),
            "proof_expected": copy.deepcopy(_recovery_proof_expected),
            "proof_persisted": copy.deepcopy(_recovery_proof_persisted),
            "proof_durable": _recovery_proof_durable.is_set(),
        }
    with _state_lock:
        payload.update(
            {
                "attempt_count": _recovery_attempt_count,
                "restore_count": _recovery_restore_count,
                "partial_history_restore_count": _recovery_partial_history_restore_count,
                "live_adopt_count": _recovery_live_adopt_count,
                "cold_start_count": _recovery_cold_start_count,
                "failure_count": _recovery_failure_count,
                "runtime_generation_monitor_started": _runtime_recovery_monitor_started.is_set(),
                "runtime_generation": _runtime_recovery_generation,
                "runtime_recovery_hold": _runtime_recovery_hold.is_set(),
                "runtime_recovery_count": _runtime_recovery_count,
                "runtime_surviving_reattach_count": _runtime_recovery_surviving_reattach_count,
                "runtime_failure_count": _runtime_recovery_failure_count,
                "runtime_rows_retired": _runtime_recovery_rows_retired,
                "runtime_last": copy.deepcopy(_runtime_recovery_last),
            }
        )
    return payload


def _pi_report_surface() -> Dict[str, Any]:
    with _state_lock:
        latest = copy.deepcopy(_latest_photons)
        state = {
            "fragments_received": _fragments_received,
            "fragments_processed": _fragments_processed,
            "fragments_rejected": _fragments_rejected,
            "photons_published": _photons_published,
            "rows_persisted": _rows_persisted,
            "ingress_queue_depth": _fragment_queue.qsize(),
            "persist_queue_depth": _persist_queue.qsize(),
            "lap_baseline_ns": _lap_baseline_ns,
            "lap_baseline_fs": _lap_baseline_fs,
            "standard_lap_ns": _standard_lap_ns,
            "standard_lap_ps": _standard_lap_ps,
            "teensy_standard_configured": _teensy_standard_configured,
            "latest_sequence": latest.get("sequence") if isinstance(latest, dict) else None,
            "operational_state": _operational_state_snapshot(),
            "hard_failure_entries": _hard_failure_entries,
            "hard_failure_ingress_dropped": _hard_failure_ingress_dropped,
            "hard_failure_state_dropped": _hard_failure_state_dropped,
            "hard_failure_persistence_dropped": _hard_failure_persistence_dropped,
            "last_hard_failure": copy.deepcopy(_last_hard_failure),
            "repair": {
                **copy.deepcopy(_last_repair or {}),
                "active": _repair_active(),
                "requests": _repair_requests,
                "success_count": _repair_success,
                "failure_count": _repair_failures,
            },
        }
    with _campaign_lock:
        state["lantern"] = {
            "active": _active_campaign is not None,
            "active_campaign": copy.deepcopy(_active_campaign),
            "closing_campaigns": copy.deepcopy(_closing_campaigns),
        }
    state["recovery"] = _recovery_report_surface()
    state["ppb_checkpoint"] = _ppb_checkpoint_report_surface()
    return state


def _combined_teensy_report(teensy_command: str, *, report_name: str) -> Dict[str, Any]:
    try:
        response = send_command(
            machine="TEENSY",
            subsystem=SUBSYSTEM,
            command=teensy_command,
        )
    except Exception as exc:
        return {
            "success": False,
            "message": f"Teensy PHOTONS.{teensy_command} failed: {exc}",
            "payload": {"report": report_name, "pi": _pi_report_surface()},
        }
    payload = response.get("payload") if isinstance(response, dict) else None
    if not isinstance(response, dict) or not response.get("success") or not isinstance(payload, dict):
        return {
            "success": False,
            "message": f"Teensy PHOTONS.{teensy_command} returned malformed response",
            "payload": {"report": report_name, "teensy_response": response, "pi": _pi_report_surface()},
        }
    return {
        "success": True,
        "message": "OK",
        "payload": {
            "report": report_name,
            "teensy": copy.deepcopy(payload),
            "pi": _pi_report_surface(),
        },
    }


def cmd_report_photons(_: Optional[dict]) -> Dict[str, Any]:
    global _report_photons_requests
    with _state_lock:
        _report_photons_requests += 1
    return _combined_teensy_report("REPORT_PHOTONS", report_name="PHOTONS_SYSTEM_INSTRUMENT")


def cmd_report_stats(_: Optional[dict]) -> Dict[str, Any]:
    global _report_stats_requests
    with _state_lock:
        _report_stats_requests += 1
    return _combined_teensy_report("REPORT_STATS", report_name="PHOTONS_SYSTEM_STATS")


def _stats_reset_terminal_failure(
    *,
    requested_at: str,
    stage: str,
    error: str,
    details: Optional[Dict[str, Any]] = None,
) -> None:
    """Close one asynchronous reset request as a proved terminal failure."""
    global _stats_reset_pending
    global _stats_reset_failures
    global _last_maintenance

    with _stats_reset_control_lock:
        pending = copy.deepcopy(_stats_reset_pending)
        if not isinstance(pending, dict) or pending.get("requested_at_utc") != requested_at:
            return
        failure = {
            "action": "STATS_RESET",
            "requested_at_utc": requested_at,
            "completed_at_utc": _utc_now_z(),
            "success": False,
            "stage": str(stage),
            "scope": "SYSTEMWIDE_PHOTONS_STATISTICS",
            "before": copy.deepcopy(pending.get("before")),
            "expected_reset_count": pending.get("expected_reset_count"),
            "teensy_before": copy.deepcopy(pending.get("teensy_before")),
            "teensy_request": copy.deepcopy(pending.get("teensy_request")),
            "error": str(error),
        }
        if details:
            failure.update(copy.deepcopy(details))
        _stats_reset_pending = None
        _stats_reset_in_progress.clear()

    with _state_lock:
        _stats_reset_failures += 1
        _last_maintenance = copy.deepcopy(failure)
    logging.error(
        "💥 [photons] asynchronous STATS_RESET failed during %s: %s "
        "(expected reset_count=%s)",
        failure.get("stage") or "unknown stage",
        failure.get("error") or "unspecified failure",
        failure.get("expected_reset_count"),
    )


def _stats_reset_arm_worker(requested_at: str, before: Dict[str, Any]) -> None:
    """Arm one firmware reset; normal ordered persistence proves its completion."""
    global _stats_reset_pending
    global _stats_reset_requests
    global _last_maintenance

    try:
        teensy_before = _fetch_teensy_stats_reset_report()
        firmware_before_reset_count = _require_int(
            teensy_before.get("reset_count"), "REPORT_STATS.reset_count"
        )
        reset_pending_before = _require_bool(
            teensy_before.get("reset_pending"), "REPORT_STATS.reset_pending"
        )
        if firmware_before_reset_count != int(before["reset_count"]):
            raise RuntimeError(
                "Pi/Teensy PHOTONS statistics epoch disagrees before STATS_RESET: "
                f"pi={before['reset_count']} teensy={firmware_before_reset_count}"
            )
        if reset_pending_before:
            raise RuntimeError("Teensy PHOTONS already has a STATS_RESET boundary pending")

        expected_reset_count = int(before["reset_count"]) + 1
        pending = {
            "action": "STATS_RESET",
            "requested_at_utc": requested_at,
            "success": None,
            "status": "statistics_reset_arming",
            "asynchronous": True,
            "scope": "SYSTEMWIDE_PHOTONS_STATISTICS",
            "before": copy.deepcopy(before),
            "expected_reset_count": expected_reset_count,
            "teensy_before": copy.deepcopy(teensy_before),
            "teensy_request": None,
            "completion_authority": "ORDERED_CANONICAL_PERSISTENCE",
            "completion_report": "PHOTONS_INFO.last_maintenance",
        }

        # Install the expected epoch before sending the producer command.  Hold this
        # lock through the tiny RPC so an exceptionally fast row-1 persistence cannot
        # adjudicate an incompletely armed transaction.
        with _stats_reset_control_lock:
            if not _stats_reset_in_progress.is_set():
                raise RuntimeError("PHOTONS STATS_RESET lost command custody before producer arming")
            _stats_reset_pending = copy.deepcopy(pending)
            with _state_lock:
                _stats_reset_requests += 1

            teensy_response = send_command(
                machine="TEENSY",
                subsystem=SUBSYSTEM,
                command="STATS_RESET",
            )
            teensy_payload = (
                teensy_response.get("payload") if isinstance(teensy_response, dict) else None
            )
            if (
                not isinstance(teensy_response, dict)
                or not teensy_response.get("success")
                or not isinstance(teensy_payload, dict)
            ):
                raise RuntimeError(f"Teensy PHOTONS.STATS_RESET rejected: {teensy_response!r}")
            if str(teensy_payload.get("status") or "") != "instrument_statistics_reset_requested":
                raise RuntimeError(
                    f"Teensy PHOTONS.STATS_RESET returned unexpected status: {teensy_payload!r}"
                )
            if _require_bool(teensy_payload.get("reset"), "STATS_RESET.reset") is not True:
                raise RuntimeError(
                    "Teensy PHOTONS.STATS_RESET did not acquire a fresh reset boundary"
                )
            if _require_bool(
                teensy_payload.get("reset_pending"), "STATS_RESET.reset_pending"
            ) is not True:
                raise RuntimeError("Teensy PHOTONS.STATS_RESET did not arm reset_pending")
            if str(teensy_payload.get("boundary") or "") != (
                "AFTER_NEXT_SUCCESSFULLY_PUBLISHED_FRAGMENT"
            ):
                raise RuntimeError(
                    f"Teensy PHOTONS.STATS_RESET returned unexpected boundary: {teensy_payload!r}"
                )

            pending["status"] = "statistics_reset_armed"
            pending["teensy_request"] = copy.deepcopy(teensy_payload)
            _stats_reset_pending = copy.deepcopy(pending)

        with _state_lock:
            _last_maintenance = copy.deepcopy(pending)
        logging.info(
            "📊 [photons] STATS_RESET armed asynchronously; awaiting durable row 1 "
            "for reset_count=%d",
            expected_reset_count,
        )
    except Exception as exc:
        # If failure occurred before the pending object was installed, install the
        # minimum request identity so the common terminal court can close custody.
        with _stats_reset_control_lock:
            if _stats_reset_in_progress.is_set() and _stats_reset_pending is None:
                _stats_reset_pending = {
                    "action": "STATS_RESET",
                    "requested_at_utc": requested_at,
                    "before": copy.deepcopy(before),
                    "expected_reset_count": int(before["reset_count"]) + 1,
                }
        _stats_reset_terminal_failure(
            requested_at=requested_at,
            stage="ARM_RESET_BOUNDARY",
            error=str(exc),
        )
        logging.exception("💥 [photons] asynchronous STATS_RESET producer arming failed")


def _note_stats_reset_persisted(photons: Dict[str, Any], detail_id: int) -> None:
    """Close STATS_RESET only when ordered persistence commits the exact birth row."""
    global _stats_reset_pending
    global _stats_reset_success
    global _stats_reset_failures
    global _last_maintenance

    if not _stats_reset_in_progress.is_set():
        return

    with _stats_reset_control_lock:
        pending = copy.deepcopy(_stats_reset_pending)
        if not isinstance(pending, dict):
            return
        if str(pending.get("status") or "") != "statistics_reset_armed":
            return

        requested_at = str(pending["requested_at_utc"])
        before = _require_dict(pending.get("before"), "pending STATS_RESET.before")
        expected_reset_count = _require_int(
            pending.get("expected_reset_count"), "pending STATS_RESET.expected_reset_count"
        )
        snapshot = _stats_epoch_snapshot_from_state(photons)

        # Rows already in flight before the firmware boundary remain valid old-epoch
        # testimony.  They do not complete or fail this request.
        if snapshot["reset_count"] < expected_reset_count:
            return

        if snapshot["reset_count"] > expected_reset_count:
            failure_reason = (
                "PHOTONS statistics skipped past requested reset epoch before durable birth: "
                f"expected={expected_reset_count} observed={snapshot['reset_count']}"
            )
        elif snapshot["update_count"] != 1:
            failure_reason = (
                "PHOTONS ordered persistence entered the requested reset epoch without "
                "durable update_count=1: "
                f"reset_count={expected_reset_count} observed_update={snapshot['update_count']}"
            )
        else:
            failure_reason = None

        if failure_reason is not None:
            failure = {
                "action": "STATS_RESET",
                "requested_at_utc": requested_at,
                "completed_at_utc": _utc_now_z(),
                "success": False,
                "stage": "PROVE_DURABLE_EPOCH_BIRTH",
                "scope": "SYSTEMWIDE_PHOTONS_STATISTICS",
                "before": copy.deepcopy(before),
                "expected_reset_count": expected_reset_count,
                "observed_durable_detail_id": int(detail_id),
                "observed_durable": copy.deepcopy(snapshot),
                "teensy_before": copy.deepcopy(pending.get("teensy_before")),
                "teensy_request": copy.deepcopy(pending.get("teensy_request")),
                "error": failure_reason,
            }
            _stats_reset_pending = None
            _stats_reset_in_progress.clear()
            with _state_lock:
                _stats_reset_failures += 1
                _last_maintenance = copy.deepcopy(failure)
            logging.error(
                "💥 [photons] STATS_RESET reached firmware but durable row-1 proof failed: "
                "%s (expected reset_count=%s observed detail_id=%s)",
                failure.get("error") or "unspecified proof failure",
                failure.get("expected_reset_count"),
                failure.get("observed_durable_detail_id"),
            )
            return

        try:
            birth = _stats_reset_birth_court(
                photons,
                before=before,
                expected_reset_count=expected_reset_count,
            )
            durable_birth = {"detail_id": int(detail_id), **birth}
            after = _latest_stats_epoch()
            if after["reset_count"] != expected_reset_count or after["update_count"] < 1:
                raise RuntimeError(
                    "latest PHOTONS state is not descended from the durable STATS_RESET birth row"
                )
            if after["sequence"] < durable_birth["sequence"]:
                raise RuntimeError("latest PHOTONS physical sequence regressed behind durable row 1")
            if after["lap_baseline_fs"] != before["lap_baseline_fs"]:
                raise RuntimeError("latest PHOTONS LAP_BASELINE_NS changed across STATS_RESET")
            if after["standard_lap_ps"] != before["standard_lap_ps"]:
                raise RuntimeError("latest PHOTONS compatibility standard_lap_ps changed across STATS_RESET")
            if after["campaign"] != before.get("campaign"):
                raise RuntimeError("latest PHOTONS LANTERN identity/origin changed across STATS_RESET")
            if after["custody_lap_count"] < durable_birth["custody_lap_count"]:
                raise RuntimeError("latest PHOTONS custody regressed behind durable row 1")
            if (
                after["custody_total_lap_gnss_ns"]
                < durable_birth["custody_total_lap_gnss_ns"]
            ):
                raise RuntimeError("latest PHOTONS custody total regressed behind durable row 1")
        except Exception as exc:
            failure = {
                "action": "STATS_RESET",
                "requested_at_utc": requested_at,
                "completed_at_utc": _utc_now_z(),
                "success": False,
                "stage": "VERIFY_DURABLE_EPOCH_BIRTH",
                "scope": "SYSTEMWIDE_PHOTONS_STATISTICS",
                "before": copy.deepcopy(before),
                "expected_reset_count": expected_reset_count,
                "durable_detail_id": int(detail_id),
                "teensy_before": copy.deepcopy(pending.get("teensy_before")),
                "teensy_request": copy.deepcopy(pending.get("teensy_request")),
                "error": str(exc),
            }
            _stats_reset_pending = None
            _stats_reset_in_progress.clear()
            with _state_lock:
                _stats_reset_failures += 1
                _last_maintenance = copy.deepcopy(failure)
            logging.exception("💥 [photons] STATS_RESET durable birth verification failed")
            return

        result = {
            "action": "STATS_RESET",
            "requested_at_utc": requested_at,
            "completed_at_utc": _utc_now_z(),
            "success": True,
            "scope": "SYSTEMWIDE_PHOTONS_STATISTICS",
            "completion_authority": "ORDERED_CANONICAL_PERSISTENCE",
            "before": copy.deepcopy(before),
            "epoch_birth": durable_birth,
            "after": after,
            "teensy_before": copy.deepcopy(pending.get("teensy_before")),
            "teensy_request": copy.deepcopy(pending.get("teensy_request")),
            "campaign_unchanged": True,
            "physical_measurement_unchanged": True,
            "physical_sequence_preserved": True,
            "monotonic_custody_preserved": True,
            "standard_lap_preserved": True,
            "first_durable_update_count": 1,
        }
        _stats_reset_pending = None
        _stats_reset_in_progress.clear()

    with _state_lock:
        _stats_reset_success += 1
        _last_maintenance = copy.deepcopy(result)
    logging.warning(
        "📊 [photons] STATS_RESET proved by ordered persistence reset_count=%d "
        "detail_id=%d sequence=%d N=%d",
        expected_reset_count,
        int(durable_birth["detail_id"]),
        int(durable_birth["sequence"]),
        int(durable_birth["lap_count"]),
    )


def cmd_stats_reset(_: Optional[dict]) -> Dict[str, Any]:
    """Accept one PHOTONS statistics reset and return before proof completion."""
    global _last_maintenance

    busy = _campaign_control_gate("STATS_RESET")
    if busy is not None:
        return busy
    with _campaign_lock:
        if _closing_campaigns or (
            _active_campaign is not None
            and _active_campaign.get("start_after_sequence") is None
        ):
            return {
                "success": False,
                "message": "STATS_RESET unavailable while a LANTERN boundary transition is pending",
            }

    # This is local-only and cheap; reject a command that cannot even name the
    # current canonical epoch before acquiring asynchronous reset custody.
    try:
        before = _latest_stats_epoch()
    except Exception as exc:
        return {"success": False, "message": str(exc)}

    with _stats_reset_control_lock:
        if _stats_reset_in_progress.is_set():
            return {
                "success": True,
                "message": "PHOTONS STATS_RESET is already in progress",
                "payload": {
                    "status": "statistics_reset_in_progress",
                    "asynchronous": True,
                    "before": before,
                },
            }

        requested_at = _utc_now_z()
        accepted = {
            "action": "STATS_RESET",
            "requested_at_utc": requested_at,
            "success": None,
            "status": "statistics_reset_requested",
            "asynchronous": True,
            "scope": "SYSTEMWIDE_PHOTONS_STATISTICS",
            "before": before,
            "completion_authority": "ORDERED_CANONICAL_PERSISTENCE",
            "completion_report": "PHOTONS_INFO.last_maintenance",
        }
        _stats_reset_in_progress.set()
        with _state_lock:
            _last_maintenance = copy.deepcopy(accepted)
        threading.Thread(
            target=_stats_reset_arm_worker,
            args=(requested_at, copy.deepcopy(before)),
            daemon=True,
            name="photons-stats-reset-arm",
        ).start()

    return {"success": True, "message": "OK", "payload": accepted}

def cmd_inject_problem(args: Optional[dict]) -> Dict[str, Any]:
    """Pass one diagnostic science-court injection request to Teensy PHOTONS."""
    global _inject_problem_requests
    global _inject_problem_failures
    with _state_lock:
        _inject_problem_requests += 1
    try:
        response = send_command(
            machine="TEENSY",
            subsystem=SUBSYSTEM,
            command="INJECT_PROBLEM",
            args=args or {},
        )
    except Exception as exc:
        with _state_lock:
            _inject_problem_failures += 1
        return {"success": False, "message": f"Teensy PHOTONS.INJECT_PROBLEM failed: {exc}"}
    if not isinstance(response, dict) or not response.get("success"):
        with _state_lock:
            _inject_problem_failures += 1
        return {
            "success": False,
            "message": "Teensy PHOTONS.INJECT_PROBLEM rejected",
            "payload": {"teensy": response},
        }
    return response


def cmd_report_recovery(_: Optional[dict]) -> Dict[str, Any]:
    response = _combined_teensy_report(
        "REPORT_RECOVERY", report_name="PHOTONS_SYSTEM_RECOVERY"
    )
    payload = response.get("payload") if isinstance(response, dict) else None
    if isinstance(payload, dict):
        payload["recovery"] = _recovery_report_surface()
    return response


def cmd_photons_info(_: Optional[dict]) -> Dict[str, Any]:
    with _state_lock:
        payload = {
            "schema": "PHOTONS_INFO_V1",
            "operational_state": _operational_state_snapshot(),
            "instrument_always_on": True,
            "teensy_science_authority": True,
            "pi_campaign_lifecycle_authority": True,
            "lap_baseline_ns": _lap_baseline_ns,
            "lap_baseline_fs": _lap_baseline_fs,
            "standard_lap_ns": _standard_lap_ns,
            "standard_lap_ps": _standard_lap_ps,
            "teensy_standard_configured": _teensy_standard_configured,
            "state_worker_started": _state_worker_started.is_set(),
            "persistence_worker_started": _persistence_worker_started.is_set(),
            "campaign_control_ready": _campaign_control_ready.is_set(),
            "startup_infrastructure_wait_count": _startup_infrastructure_wait_count,
            "startup_infrastructure_wait_seconds_last": _startup_infrastructure_wait_seconds_last,
            "startup_infrastructure": copy.deepcopy(_last_startup_infrastructure_wait),
            "fragments_received": _fragments_received,
            "fragments_processed": _fragments_processed,
            "fragments_rejected": _fragments_rejected,
            "rows_persisted": _rows_persisted,
            "ingress_queue_depth": _fragment_queue.qsize(),
            "persist_queue_depth": _persist_queue.qsize(),
            "stats_reset_requests": _stats_reset_requests,
            "stats_reset_success": _stats_reset_success,
            "stats_reset_failures": _stats_reset_failures,
            "stats_reset_in_progress": _stats_reset_in_progress.is_set(),
            "lap_baseline_set_count": _lap_baseline_set_count,
            "report_photons_requests": _report_photons_requests,
            "report_stats_requests": _report_stats_requests,
            "clear_count": _clear_count,
            "delete_count": _delete_count,
            "truncate_count": _truncate_count,
            "flash_cut_count": _flash_cut_count,
            "inject_problem_requests": _inject_problem_requests,
            "inject_problem_failures": _inject_problem_failures,
            "hard_failure_entries": _hard_failure_entries,
            "hard_failure_ingress_dropped": _hard_failure_ingress_dropped,
            "hard_failure_state_dropped": _hard_failure_state_dropped,
            "hard_failure_persistence_dropped": _hard_failure_persistence_dropped,
            "last_hard_failure": copy.deepcopy(_last_hard_failure),
            "repair": {
                **copy.deepcopy(_last_repair or {}),
                "active": _repair_active(),
                "requests": _repair_requests,
                "success_count": _repair_success,
                "failure_count": _repair_failures,
            },
            "last_maintenance": copy.deepcopy(_last_maintenance),
            "last_campaign_transition": copy.deepcopy(_last_campaign_transition),
            "last_structural_rejection": copy.deepcopy(_last_structural_rejection),
        }
    payload["recovery"] = _recovery_report_surface()
    with _campaign_lock:
        payload["lantern"] = {
            "active": _active_campaign is not None,
            "active_campaign": copy.deepcopy(_active_campaign),
            "closing_campaigns": copy.deepcopy(_closing_campaigns),
            "restart_recovery_enabled": True,
        }
    return {"success": True, "message": "OK", "payload": payload}


def cmd_report(_: Optional[dict]) -> dict:
    with _state_lock:
        payload = {
            "schema": "PHOTONS_REPORT_V5",
            "operational_state": _operational_state_snapshot(),
            "fragments_received": _fragments_received,
            "fragments_queued": _fragments_queued,
            "fragments_processed": _fragments_processed,
            "fragments_rejected": _fragments_rejected,
            "photons_published": _photons_published,
            "rows_persisted": _rows_persisted,
            "ingress_queue_depth": _fragment_queue.qsize(),
            "ingress_queue_depth_max": _ingress_queue_depth_max,
            "persist_queue_depth": _persist_queue.qsize(),
            "persist_queue_depth_max": _persist_queue_depth_max,
            "system_report_retry_count": _system_report_retry_count,
            "persistence_retry_count": _persistence_retry_count,
            "startup_infrastructure_wait_count": _startup_infrastructure_wait_count,
            "startup_infrastructure_wait_seconds_last": _startup_infrastructure_wait_seconds_last,
            "startup_infrastructure": copy.deepcopy(_last_startup_infrastructure_wait),
            "state_worker_started": _state_worker_started.is_set(),
            "persistence_worker_started": _persistence_worker_started.is_set(),
            "campaign_control_ready": _campaign_control_ready.is_set(),
            "campaign_start_count": _campaign_start_count,
            "campaign_stop_count": _campaign_stop_count,
            "baseline_set_count": _baseline_set_count,
            "lap_baseline_set_count": _lap_baseline_set_count,
            "stale_campaign_retire_count": _stale_campaign_retire_count,
            "stats_reset_requests": _stats_reset_requests,
            "stats_reset_success": _stats_reset_success,
            "stats_reset_failures": _stats_reset_failures,
            "report_photons_requests": _report_photons_requests,
            "report_stats_requests": _report_stats_requests,
            "clear_count": _clear_count,
            "delete_count": _delete_count,
            "truncate_count": _truncate_count,
            "flash_cut_count": _flash_cut_count,
            "inject_problem_requests": _inject_problem_requests,
            "inject_problem_failures": _inject_problem_failures,
            "last_maintenance": copy.deepcopy(_last_maintenance),
            "last_campaign_transition": copy.deepcopy(_last_campaign_transition),
            "last_structural_rejection": copy.deepcopy(_last_structural_rejection),
            "last_system_report_failure": copy.deepcopy(_last_system_report_failure),
            "last_persistence_failure": copy.deepcopy(_last_persistence_failure),
            "repair": {
                **copy.deepcopy(_last_repair or {}),
                "active": _repair_active(),
                "requests": _repair_requests,
                "success_count": _repair_success,
                "failure_count": _repair_failures,
            },
            "lap_baseline_ns": _lap_baseline_ns,
            "lap_baseline_fs": _lap_baseline_fs,
            "standard_lap_ns": _standard_lap_ns,
            "standard_lap_ps": _standard_lap_ps,
            "teensy_standard_configured": _teensy_standard_configured,
            "latest_fragment": copy.deepcopy(_latest_fragment),
            "latest_photons": copy.deepcopy(_latest_photons),
        }

    payload["recovery"] = _recovery_report_surface()
    with _campaign_lock:
        payload["lantern"] = {
            "campaign_type": CAMPAIGN_TYPE_LANTERN,
            "active": _active_campaign is not None,
            "active_campaign": (
                _campaign_public_decoration(_active_campaign)
                if _active_campaign is not None
                else None
            ),
            "closing_window_count": len(_closing_campaigns),
            "closing_windows": [
                _campaign_public_decoration(window)
                for window in _closing_campaigns
            ],
            "instrument_always_on": True,
            "campaign_changes_physical_measurement": False,
            "restart_recovery_enabled": True,
        }

    return {
        "success": True,
        "message": "OK",
        "payload": payload,
    }


def _release_after_repair(*, reason: str, details: Dict[str, Any]) -> Dict[str, Any]:
    """Clear a proved PHOTONS wedge and reopen ordinary canonical authorship."""
    global _operational_state
    global _last_hard_failure

    now_utc = _utc_now_z()
    with _hard_failure_lock:
        _hard_failure_event.clear()
        _last_hard_failure = None
        with _operational_state_lock:
            _operational_state = {
                "schema": OPERATIONAL_STATE_SCHEMA,
                "subsystem": "PHOTONS",
                "state": OPERATIONAL_STATE_RUNNING,
                "entered_at_utc": now_utc,
                "reason": str(reason),
                "source": "PHOTONS.REPAIR",
                "details": copy.deepcopy(details),
            }
            snapshot = copy.deepcopy(_operational_state)
    _runtime_recovery_hold.clear()
    _campaign_control_ready.set()
    return snapshot


def _repair_stats_reset_sync() -> Dict[str, Any]:
    """Run the existing ordered PHOTONS STATS_RESET contract and wait for its proof."""
    global _last_maintenance

    before = _latest_stats_epoch()
    requested_at = _utc_now_z()
    with _stats_reset_control_lock:
        if _stats_reset_in_progress.is_set():
            raise RuntimeError("PHOTONS STATS_RESET is already in progress")
        _stats_reset_in_progress.set()
        with _state_lock:
            _last_maintenance = {
                "action": "STATS_RESET",
                "requested_at_utc": requested_at,
                "success": None,
                "status": "statistics_reset_requested_by_repair",
                "source": "PHOTONS.REPAIR",
                "before": copy.deepcopy(before),
            }
    _stats_reset_arm_worker(requested_at, copy.deepcopy(before))

    deadline = time.monotonic() + PHOTONS_RECOVERY_PROOF_TIMEOUT_S
    while _stats_reset_in_progress.is_set():
        if time.monotonic() >= deadline:
            raise RuntimeError(
                f"PHOTONS.REPAIR STATS_RESET did not reach durable row 1 within {PHOTONS_RECOVERY_PROOF_TIMEOUT_S:.1f}s"
            )
        time.sleep(PHOTONS_PERSIST_RETRY_S)

    with _state_lock:
        result = copy.deepcopy(_last_maintenance or {})
    if result.get("action") != "STATS_RESET" or result.get("success") is not True:
        raise RuntimeError(f"PHOTONS.REPAIR implicit STATS_RESET failed: {result!r}")
    return result


def _repair_worker(*, trigger_state: Dict[str, Any], record: Dict[str, Any]) -> None:
    """Repair known PHOTONS wedges, escalating only when a narrower remedy fails."""
    global _repair_success, _repair_failures, _last_repair

    attempts: List[Dict[str, Any]] = []
    try:
        record["phase"] = "CLASSIFYING_LIVE_PRODUCER"
        with _state_lock:
            _last_repair = copy.deepcopy(record)

        _best_effort_recovery_abort()
        _clear_recovery_proof_custody()
        _runtime_recovery_hold.clear()
        _configure_teensy_lap_baseline()
        report = _fetch_teensy_recovery_report(require_baseline=True)
        record["producer_publication_started"] = bool(report.get("publication_started"))

        repaired_by = None
        repair_result: Dict[str, Any] = {}
        trigger_reason = str(trigger_state.get("reason") or "")
        force_stats_reset = trigger_reason == "producer_rolling_custody_lost"
        force_phase5 = trigger_reason in {
            "post_commit_recovery_proof_adjudication_failed",
            "ambient_producer_recovery_failed",
            "phase5_startup_recovery_failed",
        }

        if report.get("publication_started") and force_stats_reset:
            # Producer itself declared its rolling statistics custody unusable.
            # A read-only export cannot cure producer-authored invalidity; start a
            # new resettable statistics epoch while preserving physical lap custody.
            record["phase"] = "RESETTING_STATISTICS"
            with _state_lock:
                _last_repair = copy.deepcopy(record)
            stats_reset = _repair_stats_reset_sync()
            attempts.append({"strategy": "IMPLICIT_STATS_RESET", "success": True})
            repaired_by = "IMPLICIT_STATS_RESET"
            repair_result = {"stats_reset": stats_reset}
        elif force_phase5 or not report.get("publication_started"):
            # Recovery/proof wedges belong to the existing Phase-5 court. It will
            # decide live adoption, held resurrection, or cold start from current
            # producer testimony and durable custody.
            record["phase"] = "PHASE5_RECOVERY"
            with _state_lock:
                _last_repair = copy.deepcopy(record)
            recovery = _perform_phase5_recovery()
            attempts.append({"strategy": "PHASE5_RECOVERY", "success": True})
            repaired_by = "PHASE5_RECOVERY"
            repair_result = {"recovery": recovery}
        else:
            # Generic nonterminal wedge: first try the least destructive remedy,
            # exact live-producer ring reacquisition. Escalate to STATS_RESET only
            # if the producer cannot supply coherent bounded custody.
            try:
                record["phase"] = "REACQUIRING_LIVE_CUSTODY"
                with _state_lock:
                    _last_repair = copy.deepcopy(record)
                reacquired = _refresh_ppb_checkpoint_from_live_photons()
                attempts.append({"strategy": "LIVE_RING_REACQUIRE", "success": True})
                repaired_by = "LIVE_RING_REACQUIRE"
                repair_result = {"live_ring_reacquire": reacquired}
            except Exception as reacquire_exc:
                attempts.append({
                    "strategy": "LIVE_RING_REACQUIRE",
                    "success": False,
                    "error": str(reacquire_exc),
                })
                record["phase"] = "RESETTING_STATISTICS"
                with _state_lock:
                    _last_repair = copy.deepcopy(record)
                stats_reset = _repair_stats_reset_sync()
                attempts.append({"strategy": "IMPLICIT_STATS_RESET", "success": True})
                repaired_by = "IMPLICIT_STATS_RESET"
                repair_result = {"stats_reset": stats_reset}

        release_details = {
            "repaired_by": repaired_by,
            "attempts": copy.deepcopy(attempts),
            **copy.deepcopy(repair_result),
        }
        operational_state = _release_after_repair(
            reason="operator_repair_complete",
            details=release_details,
        )
        record.update({
            "phase": "SUCCEEDED",
            "completed_at_utc": _utc_now_z(),
            "success": True,
            "repaired_by": repaired_by,
            "attempts": copy.deepcopy(attempts),
            "result": copy.deepcopy(repair_result),
            "operational_state": operational_state,
            "next_action": "PHOTONS is usable now.",
        })
        with _state_lock:
            _repair_success += 1
            _last_repair = copy.deepcopy(record)
        logging.critical("🧯 [photons] REPAIR completed using %s", repaired_by)
    except Exception as exc:
        record.update({
            "phase": "FAILED",
            "completed_at_utc": _utc_now_z(),
            "success": False,
            "error": str(exc),
            "attempts": copy.deepcopy(attempts),
        })
        with _state_lock:
            _repair_failures += 1
            _last_repair = copy.deepcopy(record)
        if not _hard_failure_active():
            _enter_hard_failure(
                "operator_repair_failed",
                {"error": str(exc), "attempts": copy.deepcopy(attempts)},
                source="PHOTONS.REPAIR",
            )
        logging.exception("🛑 [photons] REPAIR failed; PHOTONS remains fail-closed")
    finally:
        _repair_event.clear()


def cmd_repair(_: Optional[dict]) -> Dict[str, Any]:
    """Diagnose and repair known PHOTONS wedges without operator mode arguments."""
    global _repair_requests, _repair_failures, _last_repair

    state = _operational_state_snapshot()
    state_name = str(state.get("state") or "").strip().upper()
    if (
        state_name == OPERATIONAL_STATE_RUNNING
        and _campaign_control_ready.is_set()
        and not _hard_failure_active()
        and not _runtime_recovery_hold.is_set()
    ):
        checkpoint = _ppb_checkpoint_report_surface().get("checkpoint")
        return {
            "success": True,
            "message": "PHOTONS is already RUNNING; no repair required",
            "payload": {
                "action": "NO_ACTION_HEALTHY",
                "operational_state": state,
                "campaign_control_ready": True,
                "checkpoint_recoverable": (
                    bool(checkpoint.get("recoverable")) if isinstance(checkpoint, dict) else None
                ),
                "checkpoint_status": (
                    checkpoint.get("status") if isinstance(checkpoint, dict) else None
                ),
            },
        }

    with _repair_lock:
        if _repair_active():
            with _state_lock:
                current = copy.deepcopy(_last_repair or {})
            return {
                "success": True,
                "message": "PHOTONS REPAIR is already in progress",
                "payload": current,
            }
        if _stats_reset_in_progress.is_set():
            return {
                "success": False,
                "message": "PHOTONS REPAIR deferred while an explicit STATS_RESET owns the statistics boundary",
            }

        requested_at = _utc_now_z()
        record = {
            "schema": "PI_PHOTONS_REPAIR_V1",
            "requested_at_utc": requested_at,
            "phase": "IN_PROGRESS",
            "success": None,
            "trigger_state": state_name,
            "trigger_reason": state.get("reason"),
            "operator_arguments_required": False,
            "implicit_statistics_reset_allowed": True,
            "repair_policy": "LIVE_REACQUIRE_THEN_STATS_RESET_OR_PHASE5",
            "next_action": "Wait for PHOTONS.REPORT.repair.success=true.",
        }
        with _state_lock:
            _repair_requests += 1
            _last_repair = copy.deepcopy(record)
        _repair_event.set()
        try:
            threading.Thread(
                target=_repair_worker,
                kwargs={"trigger_state": copy.deepcopy(state), "record": record},
                daemon=True,
                name="photons-repair",
            ).start()
        except Exception as exc:
            _repair_event.clear()
            record.update({
                "phase": "FAILED_TO_START",
                "completed_at_utc": _utc_now_z(),
                "success": False,
                "error": str(exc),
            })
            with _state_lock:
                _repair_failures += 1
                _last_repair = copy.deepcopy(record)
            return {
                "success": False,
                "message": f"PHOTONS REPAIR could not start repair worker: {exc}",
                "payload": record,
            }

    return {
        "success": True,
        "message": "PHOTONS REPAIR accepted",
        "payload": copy.deepcopy(record),
    }


COMMANDS = {
    "START": cmd_start,
    "FLASH_CUT": cmd_flash_cut,
    "STOP": cmd_stop,
    "REPORT": cmd_report,
    "REPORT_PHOTONS": cmd_report_photons,
    "REPORT_STATS": cmd_report_stats,
    "REPORT_RECOVERY": cmd_report_recovery,
    "STATS_RESET": cmd_stats_reset,
    "REPAIR": cmd_repair,
    "CLEAR": cmd_clear,
    "DELETE": cmd_delete,
    "TRUNCATE": cmd_truncate,
    "INJECT_PROBLEM": cmd_inject_problem,
    "SET_LAP_BASELINE_NS": cmd_set_lap_baseline_ns,
    "SET_BASELINE": cmd_set_baseline,
    "BASELINE_INFO": cmd_baseline_info,
    "LIST_CAMPAIGNS": cmd_list_campaigns,
    "PHOTONS_INFO": cmd_photons_info,
}

_HARD_FAILURE_READ_ONLY_COMMANDS = {
    "REPORT",
    "REPORT_PHOTONS",
    "REPORT_STATS",
    "REPORT_RECOVERY",
    "BASELINE_INFO",
    "LIST_CAMPAIGNS",
    "PHOTONS_INFO",
}


_STATS_RESET_ALLOWED_COMMANDS = _HARD_FAILURE_READ_ONLY_COMMANDS | {"STATS_RESET", "REPAIR"}


def _stats_reset_guard_command(
    command: str,
    handler,
):
    def guarded(args: Optional[dict]) -> Dict[str, Any]:
        if _stats_reset_in_progress.is_set() and command not in _STATS_RESET_ALLOWED_COMMANDS:
            return {
                "success": False,
                "message": (
                    f"PHOTONS.{command} refused while asynchronous STATS_RESET owns "
                    "the statistics epoch boundary"
                ),
                "payload": {
                    "stats_reset_in_progress": True,
                    "last_maintenance": copy.deepcopy(_last_maintenance or {}),
                },
            }
        return handler(args)

    return guarded


def _hard_failure_guard_command(
    command: str,
    handler,
):
    def guarded(args: Optional[dict]) -> Dict[str, Any]:
        if (
            _hard_failure_active()
            and command not in _HARD_FAILURE_READ_ONLY_COMMANDS
            and command != "REPAIR"
        ):
            return {
                "success": False,
                "message": f"PHOTONS.{command} refused: subsystem is latched in HARD_FAILURE",
                "payload": {"operational_state": _operational_state_snapshot()},
            }
        return handler(args)

    return guarded


COMMANDS = {
    name: _hard_failure_guard_command(
        name, _stats_reset_guard_command(name, handler)
    )
    for name, handler in COMMANDS.items()
}


# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def _startup_phase5_recovery_with_generation_retry() -> Tuple[Dict[str, Any], int]:
    """Run Phase 5 against one proved Teensy transport generation at a time.

    A generation change is infrastructure invalidation, not a PHOTONS scientific
    contradiction. Discard only process-local proof expectation, wait for the
    replacement RPC lease, reinstall the operator baseline, and classify again.
    Any failure while the same generation remains proved is still terminal.
    """
    while True:
        attempt_generation = _runtime_teensy_rpc_generation()
        if attempt_generation is None:
            _wait_for_startup_infrastructure()
            continue

        try:
            _configure_teensy_lap_baseline()
            recovery = _perform_phase5_recovery()
        except Exception as exc:
            current_generation = _runtime_teensy_rpc_generation()
            if (
                current_generation is not None
                and int(current_generation) == int(attempt_generation)
            ):
                raise

            _campaign_control_ready.clear()
            _runtime_recovery_hold.clear()
            _clear_recovery_proof_custody()
            _recovery_status_set(
                "WAITING_FOR_REATTACH",
                invalidated_generation=int(attempt_generation),
                observed_generation=(
                    int(current_generation)
                    if current_generation is not None
                    else None
                ),
                error=str(exc),
            )
            logging.warning(
                "⚠️ [photons/live restore] startup recovery on Teensy RPC "
                "generation=%d was invalidated by a transport boundary: %s; "
                "waiting for a proved replacement generation and reclassifying",
                int(attempt_generation),
                exc,
            )
            _wait_for_startup_infrastructure()
            continue

        current_generation = _runtime_teensy_rpc_generation()
        if (
            current_generation is not None
            and int(current_generation) == int(attempt_generation)
        ):
            return recovery, int(current_generation)

        # Phase 5 may have crossed durability immediately before the USB/producer
        # boundary. That completed transaction remains historical truth, but it
        # cannot authorize RUNNING against a different current Teensy lifetime.
        _campaign_control_ready.clear()
        _runtime_recovery_hold.clear()
        _clear_recovery_proof_custody()
        _recovery_status_set(
            "WAITING_FOR_REATTACH",
            invalidated_generation=int(attempt_generation),
            observed_generation=(
                int(current_generation)
                if current_generation is not None
                else None
            ),
            completed_attempt=True,
        )
        logging.warning(
            "⚠️ [photons/live restore] Phase 5 completed on Teensy RPC generation=%d "
            "but that generation was no longer current before RUNNING admission; "
            "reclassifying the replacement producer",
            int(attempt_generation),
        )
        _wait_for_startup_infrastructure()


def run() -> None:
    setup_logging()
    _hard_failure_event.clear()
    _repair_event.clear()
    _set_operational_state(
        OPERATIONAL_STATE_STARTING,
        reason="process_start",
        source="RUN",
    )
    _campaign_control_ready.clear()
    _recovery_proof_durable.clear()
    _ppb_checkpoint_reacquire_requested.clear()
    _runtime_recovery_hold.clear()

    logging.info(
        "[photons] starting canonical PHOTONS_V1 with Phase-3 literal durable recovery, "
        "authoritative SYSTEM context, ordered state/persistence workers, and "
        "Pi-owned LANTERN provenance ingress=%s publication=%s campaign_type=%s",
        PHOTONS_FRAGMENT_TOPIC,
        PHOTONS_TOPIC,
        CAMPAIGN_TYPE_LANTERN,
    )

    server_setup(
        subsystem=SUBSYSTEM,
        commands=COMMANDS,
        publication_handler=on_publication,
        blocking=False,
    )

    # Preserve the complete surviving-producer prefix while infrastructure comes
    # back after a Pi boot.  The command/publication surface is already open and
    # PHOTONS_FRAGMENT ingress is unbounded, but durable reads and Teensy commands
    # are forbidden until SYSTEM proves both application planes NOMINAL.
    _wait_for_startup_infrastructure()

    # Configuration is necessary but no longer sufficient to begin publication.
    # Firmware remains held until the Pi delivers one explicit recovery verdict:
    # aggregate restore (with complete rings or an exact literal suffix), live
    # reattachment, or scientifically empty cold start.
    _set_operational_state(
        OPERATIONAL_STATE_RECOVERING,
        reason="phase5_startup_recovery",
        source="RUN",
    )
    try:
        recovery, initial_generation = _startup_phase5_recovery_with_generation_retry()
    except Exception as exc:
        _enter_hard_failure(
            "phase5_startup_recovery_failed",
            {
                "error": str(exc),
                "recovery": _recovery_report_surface(),
            },
            source="PHOTONS_PHASE5_RECOVERY",
        )
    else:
        # START/STOP/baseline/maintenance control opens only after an advancing
        # post-restart row has crossed the complete ordered persistence transaction
        # on the same transport generation that was classified.
        _campaign_control_ready.set()
        _set_operational_state(
            OPERATIONAL_STATE_RUNNING,
            reason="phase5_recovery_complete",
            source="RUN",
        )
        proof = recovery.get("proof") if isinstance(recovery.get("proof"), dict) else {}
        _start_runtime_teensy_generation_monitor(initial_generation)
        logging.info(
            "✅ [photons] startup recovery complete: mode=%s producer_mutated=%s; "
            "source detail=%s update=%s -> durable proof update=%s; "
            "Better-Buckets history=%s%s; campaign_restored=%s",
            recovery.get("mode") or "unknown",
            recovery["producer_mutated"],
            recovery.get("source_detail_id") or "none",
            recovery.get("source_update_count") or "n/a",
            proof.get("update_count") or "n/a",
            recovery.get("ppb_history_scope") or "not applicable",
            " (truncated)" if recovery.get("ppb_history_truncated") else "",
            bool(recovery.get("campaign_restored_in_commit")),
        )

    logging.info(
        "🏁 [photons] entering main loop operational_state=%s",
        _operational_state_snapshot().get("state"),
    )
    while True:
        time.sleep(3600)


if __name__ == "__main__":
    run()
