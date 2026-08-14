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
untouched.  Durable recovery restores only aggregate sufficient state and bounded
PPB endpoint history; physical edge ancestry is always reacquired after restart.
"""

from __future__ import annotations

import copy
import json
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

PHOTONS_RECOVERY_SCHEMA_VERSION = 1
PHOTONS_RECOVERY_CHUNK_MAX_ENDPOINTS = 4
PHOTONS_RECOVERY_10_MIN_SECONDS = 10 * 60
PHOTONS_RECOVERY_60_MIN_SECONDS = 60 * 60
PHOTONS_RECOVERY_8_HOUR_SECONDS = 8 * 60 * 60
PHOTONS_RECOVERY_24_HOUR_SECONDS = 24 * 60 * 60
PHOTONS_RECOVERY_SECOND_CAPACITY = PHOTONS_RECOVERY_10_MIN_SECONDS + 1
PHOTONS_RECOVERY_MINUTE_CAPACITY = 24 * 60 + 2
PHOTONS_RECOVERY_REPLAY_MAX_ROWS = PHOTONS_RECOVERY_24_HOUR_SECONDS + 120
PHOTONS_RECOVERY_CURSOR_ITERSIZE = 256
PHOTONS_RECOVERY_PROOF_TIMEOUT_S = 180.0
PHOTONS_RECOVERY_VERIFY_TOLERANCE = 1.0e-6
PHOTONS_STATS_RESET_PROOF_TIMEOUT_S = 30.0

TEENSY_CAMPAIGN_START_ACCEPTED_STATUSES = {"start_requested", "flash_cut_requested"}
TEENSY_CAMPAIGN_STOP_ACCEPTED_STATUSES = {"stop_requested"}

# Phase-2 live data-plane queues.  maxsize=0 is intentionally unbounded: at the
# current 1 Hz instrument rate, temporary downstream stalls should create visible
# backlog rather than block PUBSUB ingress or silently discard valid testimony.
PHOTONS_INGRESS_QUEUE_MAXSIZE = 0
PHOTONS_PERSIST_QUEUE_MAXSIZE = 0
PHOTONS_STATE_RETRY_S = 0.25
PHOTONS_PERSIST_RETRY_S = 0.25

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
_persist_queue: queue.Queue[Payload] = queue.Queue(maxsize=PHOTONS_PERSIST_QUEUE_MAXSIZE)
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
_campaign_start_count = 0
_campaign_stop_count = 0
_baseline_set_count = 0
_stale_campaign_retire_count = 0
_stats_reset_requests = 0
_stats_reset_success = 0
_stats_reset_failures = 0
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
_recovery_live_reattach_count = 0
_recovery_cold_start_count = 0
_recovery_failure_count = 0

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

_standard_lap_ns: Optional[str] = None
_standard_lap_ps: Optional[int] = None
_teensy_standard_configured = False


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
            "🛑 [photons] HARD_FAILURE LATCHED — refusing automatic continuation. %s",
            json.dumps(snapshot, sort_keys=True, separators=(",", ":"), default=str),
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


def _load_standard_lap_ns() -> Tuple[str, int]:
    """Load the required config.PHOTONS optical reference exactly once at startup."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT payload ->> 'standard_lap_ns' AS standard_lap_ns
            FROM config
            WHERE config_key = 'PHOTONS'
            """
        )
        row = cur.fetchone()

    if row is None:
        raise RuntimeError("config.PHOTONS row is required")

    raw = row.get("standard_lap_ns")
    if raw is None:
        raise RuntimeError("config.PHOTONS.standard_lap_ns is required")

    try:
        value = Decimal(str(raw))
    except (InvalidOperation, ValueError) as exc:
        raise RuntimeError(
            f"config.PHOTONS.standard_lap_ns is not decimal: {raw!r}"
        ) from exc

    if not value.is_finite() or value <= 0:
        raise RuntimeError(
            f"config.PHOTONS.standard_lap_ns must be finite and > 0; got {raw!r}"
        )

    quantum = Decimal("0.001")
    normalized = value.quantize(quantum)
    if normalized != value:
        raise RuntimeError(
            "config.PHOTONS.standard_lap_ns may have at most three decimal places; "
            f"got {raw!r}"
        )

    standard_text = format(normalized, ".3f")
    standard_ps = int(normalized * 1000)
    if standard_ps <= 0:
        raise RuntimeError("config.PHOTONS.standard_lap_ns resolved to nonpositive picoseconds")
    return standard_text, standard_ps


def _configure_teensy_standard_lap() -> None:
    """Install config.PHOTONS.standard_lap_ns before firmware publication begins."""
    global _standard_lap_ns
    global _standard_lap_ps
    global _teensy_standard_configured

    standard_text, standard_ps = _load_standard_lap_ns()
    response = send_command(
        machine="TEENSY",
        subsystem=SUBSYSTEM,
        command="SET_STANDARD_LAP_NS",
        args={"standard_lap_ns": standard_text},
    )
    payload = response.get("payload") if isinstance(response, dict) else None
    if not isinstance(response, dict) or not response.get("success"):
        raise RuntimeError(f"Teensy PHOTONS.SET_STANDARD_LAP_NS failed: {response!r}")
    if not isinstance(payload, dict):
        raise RuntimeError(
            f"Teensy PHOTONS.SET_STANDARD_LAP_NS returned no payload: {response!r}"
        )
    if payload.get("standard_lap_configured") is not True:
        raise RuntimeError(
            f"Teensy did not confirm STANDARD_LAP_NS installation: {payload!r}"
        )
    echoed_ps = _require_int(
        payload.get("standard_lap_ps"),
        "PHOTONS.SET_STANDARD_LAP_NS.payload.standard_lap_ps",
        minimum=1,
    )
    if echoed_ps != standard_ps:
        raise RuntimeError(
            "Teensy STANDARD_LAP_NS echo mismatch: "
            f"configured_ps={standard_ps} echoed_ps={echoed_ps}"
        )

    _standard_lap_ns = standard_text
    _standard_lap_ps = standard_ps
    _teensy_standard_configured = True
    logging.info(
        "✅ [photons] installed STANDARD_LAP_NS=%s ns (%d ps) on Teensy",
        standard_text,
        standard_ps,
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

        expected_advanced = custody_delta > 0
        if recovery_advanced != expected_advanced:
            raise ValueError("PHOTONS recovery proof_advanced verdict is inconsistent")

    pps_count_raw = projection.get("last_pps_sequence")
    pps_count = _require_int(
        pps_count_raw, "photons.projection.last_pps_sequence", minimum=0
    )
    return sequence, publish_count, (pps_count if pps_count > 0 else None), instrument


def _make_photons(fragment: Payload, system_context: Dict[str, Any]) -> Payload:
    """Form canonical PHOTONS_V1 from firmware testimony + authoritative SYSTEM context."""
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
    return state



# ---------------------------------------------------------------------
# Phase 5 durable recovery court + Better-Buckets replay
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
    if _require_bool(instrument.get("snapshot_ok"), "PHOTONS.photons.snapshot_ok") is not True:
        raise ValueError("canonical PHOTONS snapshot is not coherent")
    if _require_bool(instrument.get("valid"), "PHOTONS.photons.valid") is not True:
        raise ValueError("canonical PHOTONS instrument is invalid")

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
    if _standard_lap_ps is None or standard_lap_ps != int(_standard_lap_ps):
        raise ValueError(
            "durable STANDARD_LAP_NS does not match config.PHOTONS: "
            f"durable_ps={standard_lap_ps} config_ps={_standard_lap_ps}"
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
    if abs(float(accepted_projected["mean"]) - ratio_mean) > PHOTONS_RECOVERY_VERIFY_TOLERANCE:
        raise ValueError("accepted projected-lap Welford mean does not close with N/T")

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
    expected_total_ppb = (
        ((ratio_mean * 1000.0) / float(standard_lap_ps)) - 1.0
    ) * 1.0e9
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
        "standard_lap_ps": standard_lap_ps,
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
        "restore_args": restore_args,
        "campaign_restore": campaign_restore,
        "pending_seed_dropped": pending_count,
    }


def _load_newest_recoverable_photons_state(
    *,
    active_master: Optional[Dict[str, Any]],
    require_active_campaign: bool,
) -> Tuple[Optional[Dict[str, Any]], List[Dict[str, Any]], int]:
    skipped: List[Dict[str, Any]] = []
    rows_seen = 0
    with open_db(row_dict=True) as conn:
        cur = conn.cursor(name="photons_recovery_source_scan")
        cur.itersize = PHOTONS_RECOVERY_CURSOR_ITERSIZE
        if require_active_campaign:
            assert active_master is not None
            cur.execute(
                """
                SELECT id, ts, campaign, viable, payload, sequence, pps_count
                FROM campaign_detail
                WHERE campaign_type = %s
                  AND campaign = %s
                ORDER BY id DESC
                """,
                (CAMPAIGN_TYPE_LANTERN, active_master["campaign"]),
            )
        else:
            cur.execute(
                """
                SELECT id, ts, campaign, viable, payload, sequence, pps_count
                FROM campaign_detail
                WHERE campaign_type = %s
                ORDER BY id DESC
                """,
                (CAMPAIGN_TYPE_LANTERN,),
            )
        for row in cur:
            rows_seen += 1
            try:
                source = _canonical_recovery_state_from_row(
                    row,
                    active_master=active_master,
                    require_active_campaign=require_active_campaign,
                )
            except Exception as exc:
                if len(skipped) < 32:
                    skipped.append({"id": int(row["id"]), "reason": str(exc)})
                continue
            return source, skipped, rows_seen
    return None, skipped, rows_seen


def _recovery_replay_endpoint(payload: Any) -> Dict[str, Any]:
    if isinstance(payload, str):
        payload = json.loads(payload)
    state = _require_dict(payload, "PPB replay payload")
    if state.get("schema") != PHOTONS_SCHEMA:
        raise ValueError("PPB replay canonical schema mismatch")
    instrument = _require_dict(state.get("photons"), "PPB replay photons")
    stats = _require_dict(instrument.get("stats"), "PPB replay stats")
    return {
        "reset_count": _require_int(stats.get("reset_count"), "PPB reset_count"),
        "sequence": _require_int(stats.get("update_count"), "PPB update_count", minimum=1),
        "current_sequence": _require_int(
            stats.get("rolling_ppb_current_sequence"),
            "PPB current_sequence",
            minimum=1,
        ),
        "admitted": _require_bool(
            stats.get("rolling_ppb_endpoint_admitted"), "PPB endpoint_admitted"
        ),
        "lap_count": _require_int(stats.get("lap_count"), "PPB lap_count", minimum=1),
        "total_lap_gnss_ns": _require_int(
            stats.get("total_lap_gnss_ns"), "PPB total_lap_gnss_ns", minimum=1
        ),
    }


def _photon_ppb_from_endpoints(
    anchor: Dict[str, Any], current: Dict[str, Any]
) -> Optional[Dict[str, Any]]:
    laps = int(current["lap_count"]) - int(anchor["lap_count"])
    total_ns = int(current["total_lap_gnss_ns"]) - int(anchor["total_lap_gnss_ns"])
    if laps == 0:
        if total_ns != 0:
            raise ValueError("PPB endpoint has zero laps and nonzero duration")
        return None
    if laps < 0 or total_ns <= 0 or _standard_lap_ps is None:
        raise ValueError("PPB endpoint subtraction is invalid")
    mean_ps = (float(total_ns) * 1000.0) / float(laps)
    return {
        "sample_count": laps,
        "ppb": (mean_ps / float(_standard_lap_ps) - 1.0) * 1.0e9,
    }


def _recovery_bucket_from_history(
    history: List[Dict[str, Any]],
    current: Dict[str, Any],
    window_seconds: int,
) -> Optional[Dict[str, Any]]:
    current_sequence = int(current["sequence"])
    target = max(0, current_sequence - int(window_seconds))
    for endpoint in history:
        sequence = int(endpoint["sequence"])
        if sequence >= target and sequence < current_sequence:
            return _photon_ppb_from_endpoints(endpoint, current)
    return None


def _verify_photons_ppb_replay(
    source: Dict[str, Any],
    *,
    all_history: List[Dict[str, Any]],
    second_history: List[Dict[str, Any]],
    minute_history: List[Dict[str, Any]],
) -> Dict[str, Any]:
    current = all_history[-1]
    stats = _require_dict(
        _require_dict(source["canonical"].get("photons"), "source photons").get("stats"),
        "source stats",
    )
    recorded_buckets = _require_dict(stats.get("ppb_buckets"), "source ppb_buckets")
    first_sequence = int(all_history[0]["sequence"])
    comparisons: Dict[str, Any] = {}
    checked = 0
    truncated = 0
    for name, seconds, history in (
        ("10_min", PHOTONS_RECOVERY_10_MIN_SECONDS, second_history),
        ("60_min", PHOTONS_RECOVERY_60_MIN_SECONDS, minute_history),
        ("8_hour", PHOTONS_RECOVERY_8_HOUR_SECONDS, minute_history),
        ("24_hour", PHOTONS_RECOVERY_24_HOUR_SECONDS, minute_history),
    ):
        current_sequence = int(current["sequence"])
        target = max(0, current_sequence - seconds)
        full_ancestry = first_sequence == 0 if target == 0 else first_sequence <= target
        computed = _recovery_bucket_from_history(history, current, seconds)
        recorded = recorded_buckets.get(name)
        if not full_ancestry:
            comparisons[name] = {
                "comparison": "SKIPPED_TRUNCATED_CONTIGUOUS_SUFFIX",
                "computed": copy.deepcopy(computed),
                "recorded": copy.deepcopy(recorded),
            }
            truncated += 1
            continue
        if computed is None and recorded is None:
            comparisons[name] = None
            continue
        if computed is None or not isinstance(recorded, dict):
            raise ValueError(f"PPB replay availability mismatch for {name}")
        recorded_n = _require_int(recorded.get("sample_count"), f"source {name}.sample_count")
        recorded_ppb = _require_float(recorded.get("ppb"), f"source {name}.ppb")
        if (
            recorded_n != int(computed["sample_count"])
            or abs(recorded_ppb - float(computed["ppb"])) > PHOTONS_RECOVERY_VERIFY_TOLERANCE
        ):
            raise ValueError(
                f"PPB replay mismatch for {name}: computed={computed!r} recorded={recorded!r}"
            )
        comparisons[name] = {
            "comparison": "VERIFIED",
            "sample_count": recorded_n,
            "delta_ppb": float(computed["ppb"]) - recorded_ppb,
        }
        checked += 1
    return {
        "checked": checked,
        "truncated_comparisons": truncated,
        "comparisons": comparisons,
    }


def _reconstruct_photons_ppb_history(source: Dict[str, Any]) -> Dict[str, Any]:
    source_id = int(source["db_detail_id"])
    source_reset = int(source["reset_count"])
    source_update = int(source["update_count"])
    expected_update = source_update
    descending: List[Dict[str, Any]] = []
    boundary: Optional[Dict[str, Any]] = None
    rows_scanned = 0

    with open_db(row_dict=True) as conn:
        cur = conn.cursor(name="photons_ppb_recovery_scan")
        cur.itersize = PHOTONS_RECOVERY_CURSOR_ITERSIZE
        cur.execute(
            """
            SELECT id, payload
            FROM campaign_detail
            WHERE campaign_type = %s
              AND id <= %s
            ORDER BY id DESC
            """,
            (CAMPAIGN_TYPE_LANTERN, source_id),
        )
        for row in cur:
            rows_scanned += 1
            try:
                endpoint = _recovery_replay_endpoint(row["payload"])
            except Exception as exc:
                boundary = {
                    "reason": "UNRECOVERABLE_DURABLE_ROW",
                    "id": int(row["id"]),
                    "error": str(exc),
                }
                break
            if int(endpoint["reset_count"]) != source_reset:
                boundary = {
                    "reason": "STATISTICS_EPOCH_BOUNDARY",
                    "id": int(row["id"]),
                }
                break
            observed_update = int(endpoint["sequence"])
            if observed_update != expected_update:
                boundary = {
                    "reason": "DURABLE_UPDATE_GAP_OR_DUPLICATE",
                    "id": int(row["id"]),
                    "expected_update_count": expected_update,
                    "observed_update_count": observed_update,
                }
                break
            if not endpoint["admitted"] or int(endpoint["current_sequence"]) != observed_update:
                boundary = {
                    "reason": "ROLLING_ANCESTRY_BOUNDARY",
                    "id": int(row["id"]),
                    "update_count": observed_update,
                }
                break
            descending.append(endpoint)
            if observed_update == 1 or len(descending) >= PHOTONS_RECOVERY_REPLAY_MAX_ROWS:
                break
            expected_update -= 1

    if not descending or int(descending[0]["sequence"]) != source_update:
        raise RuntimeError("PPB replay did not begin at the selected recovery source")
    all_history = list(reversed(descending))
    for previous, current in zip(all_history, all_history[1:]):
        if (
            int(current["sequence"]) != int(previous["sequence"]) + 1
            or int(current["lap_count"]) < int(previous["lap_count"])
            or int(current["total_lap_gnss_ns"]) < int(previous["total_lap_gnss_ns"])
        ):
            raise RuntimeError("PPB replay forward chronology is invalid")

    if int(all_history[0]["sequence"]) == 1:
        all_history.insert(
            0,
            {"sequence": 0, "lap_count": 0, "total_lap_gnss_ns": 0},
        )

    current = all_history[-1]
    if (
        int(current["sequence"]) != source_update
        or int(current["lap_count"]) != int(source["lap_count"])
        or int(current["total_lap_gnss_ns"]) != int(source["total_lap_gnss_ns"])
    ):
        raise RuntimeError("PPB replay current endpoint does not match recovery source")

    second_history = all_history[-PHOTONS_RECOVERY_SECOND_CAPACITY:]
    minute_history_all: List[Dict[str, Any]] = []
    last_key: Optional[int] = None
    for endpoint in all_history:
        sequence = int(endpoint["sequence"])
        minute_key = 0 if sequence == 0 else ((sequence - 1) // 60) + 1
        if minute_key != last_key:
            minute_history_all.append(endpoint)
            last_key = minute_key
    minute_history = minute_history_all[-PHOTONS_RECOVERY_MINUTE_CAPACITY:]
    verification = _verify_photons_ppb_replay(
        source,
        all_history=all_history,
        second_history=second_history,
        minute_history=minute_history,
    )
    return {
        "schema": "PHOTONS_PPB_RESTORE_V1",
        "source_db_detail_id": source_id,
        "source_reset_count": source_reset,
        "source_update_count": source_update,
        "rows_scanned": rows_scanned,
        "boundary": boundary,
        "history_truncated": int(all_history[0]["sequence"]) != 0,
        "second_history": copy.deepcopy(second_history),
        "minute_history": copy.deepcopy(minute_history),
        "verification": verification,
    }



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


def _fetch_teensy_recovery_report() -> Dict[str, Any]:
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
    if not _require_bool(
        payload.get("standard_lap_configured"),
        "PHOTONS.REPORT_RECOVERY.standard_lap_configured",
    ):
        raise RuntimeError("Teensy PHOTONS recovery report lacks STANDARD_LAP_NS")
    standard_ps = _require_int(
        payload.get("standard_lap_ps"),
        "PHOTONS.REPORT_RECOVERY.standard_lap_ps",
        minimum=1,
    )
    if _standard_lap_ps is None or standard_ps != int(_standard_lap_ps):
        raise RuntimeError(
            "Teensy/config PHOTONS standard mismatch during recovery: "
            f"teensy_ps={standard_ps} config_ps={_standard_lap_ps}"
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
    if not _require_bool(
        payload.get("standard_lap_configured"),
        "PHOTONS.REPORT.standard_lap_configured",
    ):
        raise RuntimeError("Teensy PHOTONS broad report lacks STANDARD_LAP_NS")
    standard_ps = _require_int(
        payload.get("standard_lap_ps"), "PHOTONS.REPORT.standard_lap_ps", minimum=1
    )
    if _standard_lap_ps is None or standard_ps != int(_standard_lap_ps):
        raise RuntimeError("Teensy PHOTONS broad-report standard mismatch")
    return copy.deepcopy(payload)


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
            sequence > int(expected["source_sequence"])
            and update_count > int(expected["source_update_count"])
            and lap_count > int(expected["source_lap_count"])
            and total_ns > int(expected["source_total_lap_gnss_ns"])
            and custody_laps > int(expected["source_custody_lap_count"])
            and custody_total > int(expected["source_custody_total_lap_gnss_ns"])
        )

    if mode == "LIVE_REATTACH":
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


def _wait_for_recovery_proof(*, acknowledge_firmware: bool) -> Dict[str, Any]:
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
    if acknowledge_firmware:
        _send_teensy_recovery_command(
            "RECOVERY_PROOF_ACK",
            args={
                "generation": int(proof["generation"]),
                "sequence": int(proof["sequence"]),
                "update_count": int(proof["update_count"]),
            },
            accepted_statuses={"recovery_proof_committed"},
        )
    return proof


def _stage_recovery_history(
    *, generation: int, replay: Dict[str, Any]
) -> Dict[str, Any]:
    second_history = list(replay["second_history"])
    minute_history = list(replay["minute_history"])
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


def _arm_existing_active_campaign_after_instrument_recovery(
    active_master: Optional[Dict[str, Any]],
) -> Optional[Dict[str, Any]]:
    global _last_campaign_transition
    if active_master is None:
        return None
    with _campaign_lock:
        if _active_campaign is None:
            raise RuntimeError("active LANTERN master was not rehydrated")
        if _active_campaign.get("start_after_sequence") is not None:
            return None
        campaign_name = str(_active_campaign["campaign"])
    response = _request_teensy_campaign_command(
        "START",
        campaign=campaign_name,
        accepted_statuses=TEENSY_CAMPAIGN_START_ACCEPTED_STATUSES,
    )
    transition = {
        "action": "RECOVERY_REARM_START",
        "at_utc": _utc_now_z(),
        "campaign": campaign_name,
        "campaign_id": int(active_master["campaign_id"]),
        "boundary_pending": True,
        "teensy_status": (response.get("payload") or {}).get("status"),
    }
    with _state_lock:
        _last_campaign_transition = copy.deepcopy(transition)
    logging.warning(
        "▶️ [photons] re-armed zero-row active LANTERN campaign '%s' after instrument recovery",
        campaign_name,
    )
    return transition


def _startup_held_restore(
    *,
    active_master: Optional[Dict[str, Any]],
    source: Dict[str, Any],
    replay: Dict[str, Any],
    skipped: List[Dict[str, Any]],
    rows_scanned: int,
) -> Dict[str, Any]:
    global _recovery_restore_count
    generation = _new_recovery_generation()
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
        "source_update_count": int(source["update_count"]),
        "source_lap_count": int(source["lap_count"]),
        "source_total_lap_gnss_ns": int(source["total_lap_gnss_ns"]),
        "source_custody_lap_count": int(source["custody_lap_count"]),
        "source_custody_total_lap_gnss_ns": int(
            source["custody_total_lap_gnss_ns"]
        ),
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
        ppb_replay={
            "rows_scanned": replay["rows_scanned"],
            "boundary": copy.deepcopy(replay["boundary"]),
            "history_truncated": replay["history_truncated"],
            "verification": copy.deepcopy(replay["verification"]),
        },
    )
    try:
        staged = _stage_recovery_history(generation=generation, replay=replay)
        commit_args = dict(source["restore_args"])
        commit_args["generation"] = generation
        commit = _send_teensy_recovery_command(
            "RECOVERY_COMMIT",
            args=commit_args,
            accepted_statuses={"recovery_committed"},
        )
    except Exception:
        _best_effort_recovery_abort()
        raise

    _start_workers()
    _recovery_status_set(
        "WAITING_FOR_DURABLE_PROOF",
        mode="HELD_RESTORE",
        generation=generation,
        source_detail_id=int(source["db_detail_id"]),
        staged=staged,
        firmware_commit=commit,
    )
    proof = _wait_for_recovery_proof(acknowledge_firmware=True)
    _mark_active_campaign_recovered(
        active_master,
        mode="HELD_RESTORE",
        generation=generation,
        source=source,
        proof=proof,
    )
    rearm = _arm_existing_active_campaign_after_instrument_recovery(active_master)
    with _state_lock:
        _recovery_restore_count += 1
    result = {
        "mode": "HELD_RESTORE",
        "generation": generation,
        "source_detail_id": int(source["db_detail_id"]),
        "source_sequence": int(source["sequence"]),
        "source_update_count": int(source["update_count"]),
        "pending_seed_dropped": int(source["pending_seed_dropped"]),
        "proof": proof,
        "campaign_rearm": rearm,
        "staged": staged,
    }
    _recovery_status_set("COMPLETE", **result)
    return result


def _startup_cold_start(
    *, active_master: Optional[Dict[str, Any]], rows_seen: int
) -> Dict[str, Any]:
    global _recovery_cold_start_count
    if rows_seen != 0:
        raise RuntimeError("cold PHOTONS start was requested despite durable LANTERN rows")
    generation = _new_recovery_generation()
    _rehydrate_pi_campaign(
        active_master,
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
    )
    cold = _send_teensy_recovery_command(
        "RECOVERY_COLD_START",
        args={"generation": generation},
        accepted_statuses={"recovery_cold_start_committed"},
    )
    _start_workers()
    proof = _wait_for_recovery_proof(acknowledge_firmware=False)
    _mark_active_campaign_recovered(
        active_master,
        mode="COLD_START",
        generation=generation,
        source=None,
        proof=proof,
    )
    rearm = _arm_existing_active_campaign_after_instrument_recovery(active_master)
    with _state_lock:
        _recovery_cold_start_count += 1
    result = {
        "mode": "COLD_START",
        "generation": generation,
        "firmware": cold,
        "proof": proof,
        "campaign_rearm": rearm,
    }
    _recovery_status_set("COMPLETE", **result)
    return result


def _startup_live_reattach(
    *,
    active_master: Optional[Dict[str, Any]],
    source: Optional[Dict[str, Any]],
    recovery_report: Dict[str, Any],
    skipped: List[Dict[str, Any]],
    rows_scanned: int,
) -> Dict[str, Any]:
    global _recovery_live_reattach_count
    broad = _fetch_teensy_photons_report()
    floor = _validate_live_teensy_against_durable_source(recovery_report, source)

    campaign_state = str(recovery_report.get("campaign_state") or "").upper()
    live_campaign = str(recovery_report.get("campaign") or "")
    if active_master is None:
        if campaign_state != "STOPPED" or live_campaign:
            raise RuntimeError(
                "live Teensy owns a LANTERN campaign but PostgreSQL has no active master"
            )
    else:
        if campaign_state in {"ACTIVE", "START_PENDING"}:
            if live_campaign != active_master["campaign"]:
                raise RuntimeError("live Teensy/DB active LANTERN campaign mismatch")
        elif campaign_state == "STOPPED":
            if _count_lantern_campaign_details(active_master["campaign"]) != 0:
                raise RuntimeError(
                    "active durable LANTERN campaign has history but live Teensy is stopped"
                )
        else:
            raise RuntimeError(
                f"live reattachment refuses transitional campaign state {campaign_state!r}"
            )

    proof_pending = bool(recovery_report["proof_pending"])
    proof_advanced = bool(recovery_report["proof_advanced_published"])
    generation = int(recovery_report["generation"])

    if proof_pending:
        if source is None:
            raise RuntimeError("live pending firmware proof has no durable recovery source")
        expected = {
            "mode": "FIRMWARE_PENDING_PROOF",
            "generation": generation,
            "source_sequence": int(recovery_report["source_sequence"]),
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
        first_proof_sequence = int(recovery_report["proof_sequence"])
        first_proof_update_count = int(recovery_report["proof_update_count"])
        _rehydrate_pi_campaign(
            active_master,
            source=source,
            live_report=broad,
            allow_first_public_count_splice=True,
        )
        _arm_recovery_proof(expected)
        durable = None
        if proof_advanced:
            durable = _find_durable_firmware_proof(
                generation=generation,
                sequence=first_proof_sequence,
                update_count=first_proof_update_count,
            )
        if durable is not None:
            _note_recovery_proof_persisted(
                durable["canonical"], int(durable["detail_id"])
            )
        _start_workers()
        proof = _wait_for_recovery_proof(acknowledge_firmware=True)
        mode = "LIVE_REATTACH_PENDING_PROOF"
        drained = 0
    else:
        drained = _drain_queue(_fragment_queue)
        recovery_report = _fetch_teensy_recovery_report()
        broad = _fetch_teensy_photons_report()
        floor = _validate_live_teensy_against_durable_source(recovery_report, source)
        _rehydrate_pi_campaign(
            active_master,
            source=source,
            live_report=broad,
            allow_first_public_count_splice=True,
        )
        expected = {"mode": "LIVE_REATTACH", **floor}
        _arm_recovery_proof(expected)
        _start_workers()
        proof = _wait_for_recovery_proof(acknowledge_firmware=False)
        mode = "LIVE_REATTACH"

    _mark_active_campaign_recovered(
        active_master,
        mode=mode,
        generation=generation,
        source=source,
        proof=proof,
    )
    rearm = None
    if campaign_state == "STOPPED":
        rearm = _arm_existing_active_campaign_after_instrument_recovery(active_master)
    with _state_lock:
        _recovery_live_reattach_count += 1
    result = {
        "mode": mode,
        "generation": generation,
        "source_detail_id": int(source["db_detail_id"]) if source else None,
        "source_rows_scanned": rows_scanned,
        "damaged_rows_skipped": copy.deepcopy(skipped),
        "ingress_rows_drained": drained,
        "live_floor": floor,
        "proof": proof,
        "campaign_rearm": rearm,
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
        active_master = _load_active_lantern_master()
        active_detail_count = (
            _count_lantern_campaign_details(active_master["campaign"])
            if active_master is not None
            else 0
        )
        total_detail_count = _count_current_lantern_details()
        require_active_campaign = active_master is not None and active_detail_count > 0
        source, skipped, rows_scanned = _load_newest_recoverable_photons_state(
            active_master=active_master,
            require_active_campaign=require_active_campaign,
        )
        if source is None and total_detail_count != 0:
            raise RuntimeError(
                "durable PHOTONS history exists but no row satisfies the recovery court: "
                f"rows_scanned={rows_scanned} skipped={skipped!r}"
            )

        report = _fetch_teensy_recovery_report()
        if report["staging_active"] and not report["publication_started"]:
            _best_effort_recovery_abort()
            report = _fetch_teensy_recovery_report()
        if report["staging_active"]:
            raise RuntimeError("Teensy PHOTONS recovery staging remains active")

        classification = {
            "active_campaign": active_master["campaign"] if active_master else None,
            "active_campaign_detail_count": active_detail_count,
            "total_detail_count": total_detail_count,
            "source_detail_id": int(source["db_detail_id"]) if source else None,
            "damaged_rows_skipped": copy.deepcopy(skipped),
            "rows_scanned": rows_scanned,
            "teensy_publication_started": bool(report["publication_started"]),
        }
        _recovery_status_set("CLASSIFIED", **classification)

        if report["publication_started"]:
            return _startup_live_reattach(
                active_master=active_master,
                source=source,
                recovery_report=report,
                skipped=skipped,
                rows_scanned=rows_scanned,
            )
        if source is None:
            return _startup_cold_start(
                active_master=active_master,
                rows_seen=total_detail_count,
            )
        replay = _reconstruct_photons_ppb_history(source)
        return _startup_held_restore(
            active_master=active_master,
            source=source,
            replay=replay,
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
        if public_count != previous_public_count + 1:
            if not splice_pending or public_count <= previous_public_count:
                raise ValueError(
                    "PHOTONS campaign public-count discontinuity: "
                    f"previous={previous_public_count} current={public_count} campaign={name!r}"
                )
            window["restart_public_count_gap"] = (
                public_count - previous_public_count - 1
            )
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


def _persist_photons(photons: Payload) -> None:
    """Persist one canonical PHOTONS row and advance the LANTERN read model.

    ``viable`` remains structural: individual firmware lap exclusions never make
    the one-second batch non-viable.  campaign_master is only a latest read model;
    campaign_detail remains the durable observation history.
    """
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

    # The open_db context has committed successfully.  Only now may this row
    # satisfy Phase-5 proof custody; an uncommitted INSERT is never a proof.
    _note_recovery_proof_persisted(photons, detail_id)


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

    if _hard_failure_active():
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
        with _maintenance_lock:
            try:
                fragment = _fragment_queue.get_nowait()
            except queue.Empty:
                pass
            if fragment is None:
                pass
            elif _hard_failure_active():
                with _state_lock:
                    _hard_failure_state_dropped += 1
            else:
                with _state_lock:
                    _fragments_processed += 1

                system_failure_logged = False
                while True:
                    if _hard_failure_active():
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

                if _hard_failure_active():
                    with _state_lock:
                        _hard_failure_state_dropped += 1
                    continue

                try:
                    photons = _make_photons(fragment, system_context)
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
                        "💥 [photons] PHOTONS_FRAGMENT rejected by structural/accounting court: %s",
                        rejection,
                    )
                    continue

                with _state_lock:
                    _latest_photons = copy.deepcopy(photons)

                publish(PHOTONS_TOPIC, photons)
                with _state_lock:
                    _photons_published += 1

                _persist_queue.put_nowait(copy.deepcopy(photons))
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
        photons = None
        with _maintenance_lock:
            try:
                photons = _persist_queue.get_nowait()
            except queue.Empty:
                pass
            if photons is None:
                pass
            elif _hard_failure_active():
                with _state_lock:
                    _hard_failure_persistence_dropped += 1
            else:
                failure_logged = False
                while True:
                    try:
                        _persist_photons(photons)
                        with _state_lock:
                            _rows_persisted += 1
                        break
                    except Exception as exc:
                        if _hard_failure_active():
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


        if photons is None:
            time.sleep(0.01)


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


SUBSCRIPTIONS = {
    PHOTONS_FRAGMENT_TOPIC: on_photons_fragment,
}


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
    for payload in pending:
        campaign = payload.get("campaign") if isinstance(payload, dict) else None
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
    logging.warning("🗑️ [photons] CLEAR: %s", result)
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
    logging.warning("🧨 [photons] TRUNCATE: %s", result)
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
                "latest_mean_lap_ns": (
                    (report.get("instrument_stats") or {}).get("mean_lap_ns")
                    if isinstance(report.get("instrument_stats"), dict)
                    else None
                ),
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
    if snapshot["standard_lap_ps"] != int(before["standard_lap_ps"]):
        raise ValueError("PHOTONS STATS_RESET changed immutable STANDARD_LAP_NS")
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

    if lap_count == 0:
        if total_ns != 0:
            raise ValueError("PHOTONS STATS_RESET birth has zero N with nonzero total")
    else:
        if total_ns == 0:
            raise ValueError("PHOTONS STATS_RESET birth has nonzero N with zero total")
        ratio_mean = float(total_ns) / float(lap_count)
        if abs(float(lap_welford["mean"]) - ratio_mean) > PHOTONS_RECOVERY_VERIFY_TOLERANCE:
            raise ValueError("PHOTONS STATS_RESET birth Welford mean does not close with N/T")

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


def _find_durable_stats_reset_birth(
    *,
    expected_reset_count: int,
) -> Optional[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, payload
            FROM campaign_detail
            WHERE campaign_type = %s
              AND payload #>> '{schema}' = %s
              AND payload #>> '{photons,stats,reset_count}' = %s
              AND payload #>> '{photons,stats,update_count}' = '1'
            ORDER BY id DESC
            LIMIT 2
            """,
            (
                CAMPAIGN_TYPE_LANTERN,
                PHOTONS_SCHEMA,
                str(expected_reset_count),
            ),
        )
        rows = cur.fetchall()
    if len(rows) > 1:
        raise RuntimeError(
            "PHOTONS STATS_RESET has multiple durable update_count=1 rows "
            f"for reset_count={expected_reset_count}"
        )
    if not rows:
        return None

    row = rows[0]
    payload = row["payload"]
    if isinstance(payload, str):
        payload = json.loads(payload)
    payload = copy.deepcopy(_require_dict(payload, "PHOTONS durable stats-reset birth"))
    return {"detail_id": int(row["id"]), "canonical": payload}


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


def _fetch_teensy_stats_reset_postcondition(
    *,
    expected_reset_count: int,
    request_count: int,
    prior_commit_count: int,
) -> Dict[str, Any]:
    payload = _fetch_teensy_stats_reset_report()

    reset_count = _require_int(payload.get("reset_count"), "REPORT_STATS.reset_count")
    update_count = _require_int(
        payload.get("update_count"), "REPORT_STATS.update_count", minimum=1
    )
    reset_request_count = _require_int(
        payload.get("reset_request_count"), "REPORT_STATS.reset_request_count"
    )
    reset_commit_count = _require_int(
        payload.get("reset_commit_count"), "REPORT_STATS.reset_commit_count"
    )
    reset_pending = _require_bool(
        payload.get("reset_pending"), "REPORT_STATS.reset_pending"
    )
    if reset_count != expected_reset_count:
        raise RuntimeError(
            "Teensy PHOTONS STATS_RESET postcondition has wrong reset_count: "
            f"expected={expected_reset_count} observed={reset_count}"
        )
    if update_count < 1 or reset_pending:
        raise RuntimeError(
            "Teensy PHOTONS STATS_RESET postcondition is not a committed live epoch"
        )
    if reset_request_count != request_count:
        raise RuntimeError(
            "Teensy PHOTONS STATS_RESET request-count ancestry changed unexpectedly: "
            f"requested={request_count} reported={reset_request_count}"
        )
    if reset_commit_count != prior_commit_count + 1:
        raise RuntimeError(
            "Teensy PHOTONS STATS_RESET commit count did not advance exactly once: "
            f"before={prior_commit_count} after={reset_commit_count}"
        )
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
                "live_reattach_count": _recovery_live_reattach_count,
                "cold_start_count": _recovery_cold_start_count,
                "failure_count": _recovery_failure_count,
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
        }
    with _campaign_lock:
        state["lantern"] = {
            "active": _active_campaign is not None,
            "active_campaign": copy.deepcopy(_active_campaign),
            "closing_campaigns": copy.deepcopy(_closing_campaigns),
        }
    state["recovery"] = _recovery_report_surface()
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


def cmd_stats_reset(_: Optional[dict]) -> Dict[str, Any]:
    """Reset PHOTONS statistics and prove exact canonical + durable epoch birth."""
    global _stats_reset_requests
    global _stats_reset_success
    global _stats_reset_failures
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

    try:
        before = _latest_stats_epoch()
    except Exception as exc:
        return {"success": False, "message": str(exc)}

    try:
        teensy_before = _fetch_teensy_stats_reset_report()
        firmware_before_reset_count = _require_int(
            teensy_before.get("reset_count"), "REPORT_STATS.reset_count"
        )
        request_count_before = _require_int(
            teensy_before.get("reset_request_count"), "REPORT_STATS.reset_request_count"
        )
        commit_count_before = _require_int(
            teensy_before.get("reset_commit_count"), "REPORT_STATS.reset_commit_count"
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
    except Exception as exc:
        return {"success": False, "message": str(exc)}

    expected_reset_count = int(before["reset_count"]) + 1
    expected_request_count = request_count_before + 1
    with _state_lock:
        _stats_reset_requests += 1
    requested_at = _utc_now_z()

    try:
        teensy_response = send_command(
            machine="TEENSY",
            subsystem=SUBSYSTEM,
            command="STATS_RESET",
        )
    except Exception as exc:
        with _state_lock:
            _stats_reset_failures += 1
        return {"success": False, "message": f"Teensy PHOTONS.STATS_RESET failed: {exc}"}

    teensy_payload = (
        teensy_response.get("payload") if isinstance(teensy_response, dict) else None
    )
    try:
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
        # The immediate command reply proves only that firmware armed a new
        # publication-boundary reset.  Epoch ancestry comes from REPORT_STATS
        # before the command and from the first published/durable row afterward;
        # do not duplicate that authority with optional echo fields here.
        if str(teensy_payload.get("boundary") or "") != (
            "AFTER_NEXT_SUCCESSFULLY_PUBLISHED_FRAGMENT"
        ):
            raise RuntimeError(
                f"Teensy PHOTONS.STATS_RESET returned unexpected boundary: {teensy_payload!r}"
            )
    except Exception as exc:
        failure = {
            "action": "STATS_RESET",
            "requested_at_utc": requested_at,
            "success": False,
            "stage": "ARM_RESET_BOUNDARY",
            "before": before,
            "teensy_before": copy.deepcopy(teensy_before),
            "expected_reset_count": expected_reset_count,
            "teensy": copy.deepcopy(teensy_response),
            "error": str(exc),
        }
        with _state_lock:
            _stats_reset_failures += 1
            _last_maintenance = copy.deepcopy(failure)
        logging.exception("💥 [photons] STATS_RESET boundary contract failed")
        return {"success": False, "message": str(exc), "payload": failure}

    deadline = time.monotonic() + PHOTONS_STATS_RESET_PROOF_TIMEOUT_S
    live_birth: Optional[Dict[str, Any]] = None
    durable_birth: Optional[Dict[str, Any]] = None
    last_live: Optional[Dict[str, Any]] = None
    proof_error: Optional[str] = None

    while time.monotonic() < deadline:
        try:
            with _state_lock:
                latest = copy.deepcopy(_latest_photons)
            if isinstance(latest, dict):
                latest_epoch = _stats_epoch_snapshot_from_state(latest)
                last_live = latest_epoch
                if latest_epoch["reset_count"] > expected_reset_count:
                    raise RuntimeError(
                        "PHOTONS statistics skipped past requested reset epoch: "
                        f"expected={expected_reset_count} observed={latest_epoch['reset_count']}"
                    )
                if (
                    latest_epoch["reset_count"] == expected_reset_count
                    and latest_epoch["update_count"] == 1
                    and live_birth is None
                ):
                    live_birth = _stats_reset_birth_court(
                        latest,
                        before=before,
                        expected_reset_count=expected_reset_count,
                    )

            durable = _find_durable_stats_reset_birth(
                expected_reset_count=expected_reset_count
            )
            if durable is not None:
                durable_proof = _stats_reset_birth_court(
                    durable["canonical"],
                    before=before,
                    expected_reset_count=expected_reset_count,
                )
                durable_birth = {
                    "detail_id": int(durable["detail_id"]),
                    **durable_proof,
                }
                if live_birth is not None and int(live_birth["sequence"]) != int(
                    durable_birth["sequence"]
                ):
                    raise RuntimeError(
                        "PHOTONS live/durable STATS_RESET row-1 identity mismatch: "
                        f"live_sequence={live_birth['sequence']} "
                        f"durable_sequence={durable_birth['sequence']}"
                    )
                break
        except Exception as exc:
            proof_error = str(exc)
            break
        time.sleep(0.05)

    if durable_birth is None:
        failure = {
            "action": "STATS_RESET",
            "requested_at_utc": requested_at,
            "completed_at_utc": _utc_now_z(),
            "success": False,
            "stage": "PROVE_DURABLE_EPOCH_BIRTH",
            "before": before,
            "expected_reset_count": expected_reset_count,
            "live_birth": live_birth,
            "last_live": last_live,
            "teensy": copy.deepcopy(teensy_payload),
            "error": proof_error,
            "timeout_s": PHOTONS_STATS_RESET_PROOF_TIMEOUT_S,
        }
        with _state_lock:
            _stats_reset_failures += 1
            _last_maintenance = copy.deepcopy(failure)
        logging.error("💥 [photons] STATS_RESET epoch-birth proof failed: %s", failure)
        return {
            "success": False,
            "message": (
                proof_error
                or "PHOTONS.STATS_RESET did not produce one proven durable update_count=1 row"
            ),
            "payload": failure,
        }

    try:
        teensy_after = _fetch_teensy_stats_reset_postcondition(
            expected_reset_count=expected_reset_count,
            request_count=expected_request_count,
            prior_commit_count=commit_count_before,
        )
        after = _latest_stats_epoch()
        if after["reset_count"] != expected_reset_count or after["update_count"] < 1:
            raise RuntimeError(
                "latest PHOTONS state is not descended from the proven STATS_RESET birth row"
            )
        if after["sequence"] < durable_birth["sequence"]:
            raise RuntimeError("latest PHOTONS physical sequence regressed behind durable row 1")
        if after["standard_lap_ps"] != before["standard_lap_ps"]:
            raise RuntimeError("latest PHOTONS STANDARD_LAP_NS changed across STATS_RESET")
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
            "stage": "VERIFY_POSTCONDITION",
            "before": before,
            "durable_birth": durable_birth,
            "last_live": last_live,
            "teensy": copy.deepcopy(teensy_payload),
            "error": str(exc),
        }
        with _state_lock:
            _stats_reset_failures += 1
            _last_maintenance = copy.deepcopy(failure)
        logging.exception("💥 [photons] STATS_RESET postcondition failed")
        return {"success": False, "message": str(exc), "payload": failure}

    result = {
        "action": "STATS_RESET",
        "requested_at_utc": requested_at,
        "completed_at_utc": _utc_now_z(),
        "success": True,
        "scope": "SYSTEMWIDE_PHOTONS_STATISTICS",
        "before": before,
        "epoch_birth": durable_birth,
        "after": after,
        "teensy_before": copy.deepcopy(teensy_before),
        "teensy_request": copy.deepcopy(teensy_payload),
        "teensy_after": teensy_after,
        "campaign_unchanged": True,
        "physical_measurement_unchanged": True,
        "physical_sequence_preserved": True,
        "monotonic_custody_preserved": True,
        "standard_lap_preserved": True,
        "first_durable_update_count": 1,
    }
    with _state_lock:
        _stats_reset_success += 1
        _last_maintenance = copy.deepcopy(result)
    logging.warning(
        "📊 [photons] STATS_RESET proved new durable epoch reset_count=%d "
        "detail_id=%d sequence=%d N=%d",
        expected_reset_count,
        int(durable_birth["detail_id"]),
        int(durable_birth["sequence"]),
        int(durable_birth["lap_count"]),
    )
    return {"success": True, "message": "OK", "payload": result}


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
            "standard_lap_ns": _standard_lap_ns,
            "standard_lap_ps": _standard_lap_ps,
            "teensy_standard_configured": _teensy_standard_configured,
            "state_worker_started": _state_worker_started.is_set(),
            "persistence_worker_started": _persistence_worker_started.is_set(),
            "campaign_control_ready": _campaign_control_ready.is_set(),
            "fragments_received": _fragments_received,
            "fragments_processed": _fragments_processed,
            "fragments_rejected": _fragments_rejected,
            "rows_persisted": _rows_persisted,
            "ingress_queue_depth": _fragment_queue.qsize(),
            "persist_queue_depth": _persist_queue.qsize(),
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
            "hard_failure_entries": _hard_failure_entries,
            "hard_failure_ingress_dropped": _hard_failure_ingress_dropped,
            "hard_failure_state_dropped": _hard_failure_state_dropped,
            "hard_failure_persistence_dropped": _hard_failure_persistence_dropped,
            "last_hard_failure": copy.deepcopy(_last_hard_failure),
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
            "state_worker_started": _state_worker_started.is_set(),
            "persistence_worker_started": _persistence_worker_started.is_set(),
            "campaign_control_ready": _campaign_control_ready.is_set(),
            "campaign_start_count": _campaign_start_count,
            "campaign_stop_count": _campaign_stop_count,
            "baseline_set_count": _baseline_set_count,
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


COMMANDS = {
    "START": cmd_start,
    "FLASH_CUT": cmd_flash_cut,
    "STOP": cmd_stop,
    "REPORT": cmd_report,
    "REPORT_PHOTONS": cmd_report_photons,
    "REPORT_STATS": cmd_report_stats,
    "REPORT_RECOVERY": cmd_report_recovery,
    "STATS_RESET": cmd_stats_reset,
    "CLEAR": cmd_clear,
    "DELETE": cmd_delete,
    "TRUNCATE": cmd_truncate,
    "INJECT_PROBLEM": cmd_inject_problem,
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


def _hard_failure_guard_command(
    command: str,
    handler,
):
    def guarded(args: Optional[dict]) -> Dict[str, Any]:
        if _hard_failure_active() and command not in _HARD_FAILURE_READ_ONLY_COMMANDS:
            return {
                "success": False,
                "message": f"PHOTONS.{command} refused: subsystem is latched in HARD_FAILURE",
                "payload": {"operational_state": _operational_state_snapshot()},
            }
        return handler(args)

    return guarded


COMMANDS = {
    name: _hard_failure_guard_command(name, handler)
    for name, handler in COMMANDS.items()
}


# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    setup_logging()
    _hard_failure_event.clear()
    _set_operational_state(
        OPERATIONAL_STATE_STARTING,
        reason="process_start",
        source="RUN",
    )
    _campaign_control_ready.clear()
    _recovery_proof_durable.clear()

    logging.info(
        "[photons] starting canonical PHOTONS_V1 with Phase-5 durable recovery, "
        "authoritative SYSTEM context, ordered state/persistence workers, and "
        "Pi-owned LANTERN provenance subscription=%s publication=%s campaign_type=%s",
        PHOTONS_FRAGMENT_TOPIC,
        PHOTONS_TOPIC,
        CAMPAIGN_TYPE_LANTERN,
    )

    server_setup(
        subsystem=SUBSYSTEM,
        commands=COMMANDS,
        subscriptions=SUBSCRIPTIONS,
        blocking=False,
    )

    # Configuration is necessary but no longer sufficient to begin publication.
    # Firmware remains held until the Pi delivers one explicit recovery verdict:
    # aggregate restore, live reattachment, or scientifically empty cold start.
    _set_operational_state(
        OPERATIONAL_STATE_RECOVERING,
        reason="phase5_startup_recovery",
        source="RUN",
    )
    try:
        _configure_teensy_standard_lap()
        recovery = _perform_phase5_recovery()
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
        # post-restart row has crossed the complete ordered persistence transaction.
        _campaign_control_ready.set()
        _set_operational_state(
            OPERATIONAL_STATE_RUNNING,
            reason="phase5_recovery_complete",
            source="RUN",
        )
        logging.info("✅ [photons] Phase-5 recovery complete: %s", recovery)

    logging.info(
        "🏁 [photons] entering main loop operational_state=%s",
        _operational_state_snapshot().get("state"),
    )
    while True:
        time.sleep(3600)


if __name__ == "__main__":
    run()
