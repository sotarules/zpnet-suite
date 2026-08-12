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
untouched.  Full optical/statistical restart recovery remains deferred to Phase 5.
"""

from __future__ import annotations

import copy
import json
import logging
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
LANTERN_RESTART_INTERRUPTION_REASON = "PHASE4_NO_RESTART_RECOVERY"

# Canonical Pi-side instrument schema and accepted firmware source schemas.
PHOTONS_SCHEMA = "PHOTONS_V1"
PHOTONS_FRAGMENT_SCHEMA = "PHOTONS_FRAGMENT_V1"
PHOTONS_INSTRUMENT_SCHEMA = "PHOTONS_INSTRUMENT_V1"
PHOTONS_SCIENCE_SCHEMA = "PHOTONS_SCIENCE_V2"
PHOTONS_STATS_SCHEMA = "PHOTONS_INSTRUMENT_STATS_V1"

TEENSY_CAMPAIGN_START_ACCEPTED_STATUSES = {"start_requested"}
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

_fragment_queue: queue.Queue[Payload] = queue.Queue(maxsize=PHOTONS_INGRESS_QUEUE_MAXSIZE)
_persist_queue: queue.Queue[Payload] = queue.Queue(maxsize=PHOTONS_PERSIST_QUEUE_MAXSIZE)
_state_worker_started = threading.Event()
_persistence_worker_started = threading.Event()
_campaign_control_ready = threading.Event()

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
_last_campaign_transition: Optional[Dict[str, Any]] = None
_last_structural_rejection: Optional[Dict[str, Any]] = None
_last_system_report_failure: Optional[Dict[str, Any]] = None
_last_persistence_failure: Optional[Dict[str, Any]] = None

_standard_lap_ns: Optional[str] = None
_standard_lap_ps: Optional[int] = None
_teensy_standard_configured = False


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

def _utc_now_z() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _require_dict(value: Any, path: str) -> Dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be an object")
    return value


def _require_int(value: Any, path: str, *, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"{path} must be an integer >= {minimum}; got {value!r}")
    return int(value)


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
    lifetime_lap_count = _require_int(
        instrument_stats.get("lap_count"), "PHOTONS_FRAGMENT.photons.stats.lap_count"
    )
    lifetime_total_ns = _require_int(
        instrument_stats.get("total_lap_gnss_ns"),
        "PHOTONS_FRAGMENT.photons.stats.total_lap_gnss_ns",
    )
    if lap_count > lifetime_lap_count or total_ns > lifetime_total_ns:
        raise ValueError(
            "PHOTONS campaign population exceeds always-on instrument population: "
            f"campaign_n={lap_count} lifetime_n={lifetime_lap_count} "
            f"campaign_total={total_ns} lifetime_total={lifetime_total_ns}"
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
    stats_lap_count = _require_int(stats.get("lap_count"), "photons.stats.lap_count")
    stats_lap_n = _require_int(stats_lap_time.get("n"), "photons.stats.lap_time.n")
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
        if public_count != previous_public_count + 1:
            raise ValueError(
                "PHOTONS campaign public-count discontinuity: "
                f"previous={previous_public_count} current={public_count} campaign={name!r}"
            )
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


def _retire_stale_active_campaigns() -> None:
    """Fail closed across restart until Phase 5 can prove campaign continuation.

    Full campaign/statistical resurrection is still deferred.  If this Pi process is
    restarted while a LANTERN master is active, close that master explicitly
    before the state worker begins decorating new rows.  PUBSUB ingress is already
    online, so fragments accumulate in the unbounded ingress queue while a
    temporary database outage is retried here.
    """
    global _stale_campaign_retire_count
    global _last_campaign_transition

    failure_logged = False
    while True:
        try:
            interrupted_at = _utc_now_z()
            with open_db() as conn:
                cur = conn.cursor()
                cur.execute(
                    """
                    UPDATE campaign_master
                    SET active = false,
                        payload = payload || jsonb_build_object(
                            'interrupted_at', to_jsonb(%s::text),
                            'interruption_reason', to_jsonb(%s::text)
                        )
                    WHERE campaign_type = %s
                      AND active = true
                    """,
                    (
                        interrupted_at,
                        LANTERN_RESTART_INTERRUPTION_REASON,
                        CAMPAIGN_TYPE_LANTERN,
                    ),
                )
                retired = int(cur.rowcount or 0)
            break
        except Exception:
            if not failure_logged:
                logging.exception(
                    "⚠️ [photons] unable to classify stale active LANTERN campaign(s); retrying"
                )
                failure_logged = True
            time.sleep(PHOTONS_PERSIST_RETRY_S)

    if retired:
        with _state_lock:
            _stale_campaign_retire_count += retired
            _last_campaign_transition = {
                "action": "RESTART_INTERRUPT",
                "at_utc": interrupted_at,
                "campaigns_retired": retired,
                "reason": LANTERN_RESTART_INTERRUPTION_REASON,
            }
        logging.warning(
            "⚠️ [photons] retired %d active LANTERN campaign(s) at restart; "
            "Phase 5 restart recovery is not yet enabled",
            retired,
        )


def _quiesce_stale_teensy_campaign() -> None:
    """Fail closed on Pi restart until firmware has no orphan LANTERN lifecycle."""
    deadline = time.monotonic() + 15.0
    stop_requested = False
    last_state = None

    while time.monotonic() < deadline:
        try:
            response = send_command(
                machine="TEENSY",
                subsystem=SUBSYSTEM,
                command="REPORT",
                retries=1,
                retry_delay_s=0.0,
            )
            payload = response.get("payload") if isinstance(response, dict) else None
            if (
                not isinstance(response, dict)
                or not response.get("success")
                or not isinstance(payload, dict)
            ):
                raise RuntimeError(f"Teensy PHOTONS.REPORT unavailable: {response!r}")
            state = str(payload.get("campaign_state") or "").upper()
        except Exception:
            time.sleep(PHOTONS_STATE_RETRY_S)
            continue

        last_state = state
        if state == "STOPPED":
            return
        if state == "START_PENDING":
            # START commits only on a successful pre-campaign publication.  Wait
            # for that lawful boundary rather than inventing a cancellation path.
            time.sleep(PHOTONS_STATE_RETRY_S)
            continue
        if state == "ACTIVE" and not stop_requested:
            _request_teensy_campaign_command(
                "STOP",
                accepted_statuses=TEENSY_CAMPAIGN_STOP_ACCEPTED_STATUSES,
            )
            stop_requested = True
            time.sleep(PHOTONS_STATE_RETRY_S)
            continue
        if state == "STOP_PENDING":
            stop_requested = True
            time.sleep(PHOTONS_STATE_RETRY_S)
            continue
        raise RuntimeError(f"unknown Teensy PHOTONS campaign_state {state!r}")

    raise RuntimeError(
        f"timed out quiescing stale Teensy PHOTONS campaign; last_state={last_state!r}"
    )


def _drain_stale_fragment_ingress() -> int:
    """Discard pre-worker restart backlog after stale firmware campaign closure."""
    drained = 0
    while True:
        try:
            _fragment_queue.get_nowait()
        except queue.Empty:
            break
        drained += 1
    return drained


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

    with open_db() as conn:
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

    _state_worker_started.set()
    logging.info("🚀 [photons] canonical PHOTONS_V1 state worker started")

    while True:
        fragment = _fragment_queue.get()
        with _state_lock:
            _fragments_processed += 1

        system_failure_logged = False
        while True:
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


def _persistence_loop() -> None:
    """Persist canonical PHOTONS rows in order, retrying transient DB failures."""
    global _rows_persisted
    global _persistence_retry_count
    global _last_persistence_failure

    _persistence_worker_started.set()
    logging.info("🚀 [photons] ordered PHOTONS persistence worker started")

    while True:
        photons = _persist_queue.get()
        failure_logged = False
        while True:
            try:
                _persist_photons(photons)
                with _state_lock:
                    _rows_persisted += 1
                break
            except Exception as exc:
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
    if _campaign_control_ready.is_set():
        return None
    return {
        "success": False,
        "message": f"{command} unavailable while LANTERN startup classification is in progress",
    }


def cmd_start(args: Optional[dict]) -> dict:
    """START one firmware-authored LANTERN recording population."""
    global _active_campaign
    global _campaign_start_count
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
        if _active_campaign is not None:
            return {
                "success": False,
                "message": (
                    f"Campaign '{_active_campaign['campaign']}' is already active; "
                    "STOP it before starting another LANTERN campaign"
                ),
            }
        if _closing_campaigns:
            return {
                "success": False,
                "message": "Previous LANTERN STOP is still awaiting its final firmware row",
            }

        started_at = _utc_now_z()
        start_location = copy.deepcopy(latest_photons.get("location") or {})

        try:
            with open_db(row_dict=True) as conn:
                cur = conn.cursor()
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
                            f"(id={existing['id']}); choose a new name"
                        ),
                    }

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

        # Publish the Pi lifecycle owner before arming firmware.  The same lock is
        # used by the ordered state worker, so no firmware campaign row can be
        # merged before this identity exists.
        _active_campaign = {
            "campaign": campaign_name,
            "campaign_id": campaign_id,
            "started_at": started_at,
            "start_after_sequence": None,
            "firmware_public_count": 0,
        }

        try:
            teensy_response = _request_teensy_campaign_command(
                "START",
                campaign=campaign_name,
                accepted_statuses=TEENSY_CAMPAIGN_START_ACCEPTED_STATUSES,
            )
        except Exception as exc:
            _active_campaign = None
            try:
                with open_db() as conn:
                    cur = conn.cursor()
                    cur.execute(
                        """
                        DELETE FROM campaign_master
                        WHERE id = %s
                          AND campaign_type = %s
                          AND campaign = %s
                          AND active = true
                        """,
                        (campaign_id, CAMPAIGN_TYPE_LANTERN, campaign_name),
                    )
                    if cur.rowcount != 1:
                        raise RuntimeError(
                            "failed START rollback did not delete exactly one campaign_master"
                        )
            except Exception:
                logging.exception(
                    "💥 [photons] failed to roll back LANTERN master after Teensy START rejection"
                )
            logging.exception("❌ [photons] Teensy PHOTONS.START failed")
            return {"success": False, "message": str(exc)}

    teensy_payload = (
        teensy_response.get("payload")
        if isinstance(teensy_response.get("payload"), dict)
        else {}
    )
    transition = {
        "action": "START",
        "at_utc": started_at,
        "campaign": campaign_name,
        "campaign_id": campaign_id,
        "boundary_pending": True,
        "teensy_status": teensy_payload.get("status"),
    }
    with _state_lock:
        _campaign_start_count += 1
        _last_campaign_transition = copy.deepcopy(transition)

    logging.info(
        "▶️ [photons] LANTERN START campaign='%s' id=%d armed on Teensy; "
        "firmware boundary pending (PHOTONS measurement remains continuously on)",
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
            "boundary_pending": True,
            "teensy_status": teensy_payload.get("status"),
            "instrument_always_on": True,
            "physical_measurement_changed": False,
        },
    }


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


def cmd_report(_: Optional[dict]) -> dict:
    with _state_lock:
        payload = {
            "schema": "PHOTONS_REPORT_V4",
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
            "restart_recovery_enabled": False,
        }

    return {
        "success": True,
        "message": "OK",
        "payload": payload,
    }


COMMANDS = {
    "START": cmd_start,
    "STOP": cmd_stop,
    "REPORT": cmd_report,
    "SET_BASELINE": cmd_set_baseline,
    "BASELINE_INFO": cmd_baseline_info,
    "LIST_CAMPAIGNS": cmd_list_campaigns,
}


# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    setup_logging()
    _campaign_control_ready.clear()

    logging.info(
        "[photons] starting canonical PHOTONS_V1 service with authoritative SYSTEM context, "
        "CLOCKS-shaped queued state/persistence workers, and Pi-owned LANTERN campaign labels "
        "subscription=%s publication=%s campaign_type=%s",
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

    # Bring the local subscription surface online first, then install the required
    # metrological reference.  The Teensy does not arm PHOTONS_FRAGMENT until this
    # command succeeds, so every emitted fragment is born ready-to-eat.
    _configure_teensy_standard_lap()

    # Full campaign/statistical resurrection remains deferred.  Fail closed on a
    # Pi-only restart: retire any stale DB master, bring any surviving Teensy
    # LANTERN lifecycle to STOPPED, then discard the queued pre-classification
    # backlog so the strict firmware/Pi campaign court starts from one clean row.
    _retire_stale_active_campaigns()
    _quiesce_stale_teensy_campaign()
    drained = _drain_stale_fragment_ingress()
    if drained:
        logging.warning(
            "⚠️ [photons] drained %d stale PHOTONS_FRAGMENT row(s) after restart campaign cleanup",
            drained,
        )
    _campaign_control_ready.set()
    _start_workers()

    logging.info("🏁 [photons] entering main loop")
    while True:
        time.sleep(3600)


if __name__ == "__main__":
    run()
