"""
ZPNet PHOTONS Process — PHOTONS_FRAGMENT ingress + canonical PHOTONS persistence.

Phase-1 contract:

    Teensy process_photons
        -> PHOTONS_FRAGMENT_V1
        -> Pi structural/accounting court
        -> canonical PHOTONS_V1
        -> PUBSUB PHOTONS
        -> campaign_detail

The Teensy remains the optical-science authority.  The Pi does not recompute,
smooth, repair, or re-adjudicate lap science.  It proves that the firmware
testimony is structurally self-consistent, then carries the exact Teensy-owned
``photons`` subtree into canonical PHOTONS.  Accepted and excluded lap
populations remain firmware facts; a fragment is not made non-viable merely
because it contains excluded laps.

LANTERN campaign semantics, SYSTEM context enrichment, asynchronous workers,
and recovery remain intentionally deferred to later bounded phases.
"""

from __future__ import annotations

import copy
import json
import logging
import threading
from datetime import datetime, timezone
from typing import Any, Dict, Optional, Tuple

from zpnet.processes.processes import (
    publish,
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

# campaign_detail is the generalized durable state table.  PHOTONS is the
# always-on optical instrument; LANTERN will later become optional campaign
# interpretation rather than a second instrument authority.
CAMPAIGN_TYPE_PHOTONS = "PHOTONS"

# Canonical Pi-side instrument schema and accepted firmware source schemas.
PHOTONS_SCHEMA = "PHOTONS_V1"
PHOTONS_FRAGMENT_SCHEMA = "PHOTONS_FRAGMENT_V1"
PHOTONS_INSTRUMENT_SCHEMA = "PHOTONS_INSTRUMENT_V1"
PHOTONS_SCIENCE_SCHEMA = "PHOTONS_SCIENCE_V2"
PHOTONS_STATS_SCHEMA = "PHOTONS_INSTRUMENT_STATS_V1"


# ---------------------------------------------------------------------
# Live state
# ---------------------------------------------------------------------

_state_lock = threading.Lock()

_latest_fragment: Optional[Payload] = None
_latest_photons: Optional[Payload] = None

_fragments_received = 0
_photons_published = 0
_rows_persisted = 0
_fragments_rejected = 0
_last_structural_rejection: Optional[Dict[str, Any]] = None


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


def _make_photons(fragment: Payload) -> Payload:
    """Form canonical PHOTONS_V1 from one structurally coherent firmware fragment."""
    sequence, publish_count, pps_count, instrument = _validate_photons_fragment(fragment)
    return {
        "schema": PHOTONS_SCHEMA,
        "source_schema": fragment.get("schema"),
        "sequence": sequence,
        "publish_count": publish_count,
        "pps_count": pps_count,
        "published_at_utc": _utc_now_z(),
        "photons": copy.deepcopy(instrument),
    }


def _persist_photons(photons: Payload) -> None:
    """
    Persist one structurally coherent canonical PHOTONS row.

    Phase 1 remains campaign-neutral.  ``viable`` means that the canonical
    instrument record passed the Pi structural/accounting court; it does not mean
    that every lap in the one-second batch entered the firmware science population.
    """
    sequence = photons["sequence"]
    pps_count = photons.get("pps_count")

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
                CAMPAIGN_TYPE_PHOTONS,
                None,
                True,
                json.dumps(photons, separators=(",", ":")),
                sequence,
                pps_count,
            ),
        )


# ---------------------------------------------------------------------
# PHOTONS_FRAGMENT ingress
# ---------------------------------------------------------------------

def on_photons_fragment(fragment: Payload) -> None:
    """
    Receive one Teensy-authored PHOTONS_FRAGMENT.

    Phase 1 deliberately keeps the synchronous execution path:

        receive -> structural court -> canonicalize -> publish -> persist

    Queue/worker decoupling belongs to Phase 2.
    """
    global _latest_fragment
    global _latest_photons
    global _fragments_received
    global _photons_published
    global _rows_persisted
    global _fragments_rejected
    global _last_structural_rejection

    with _state_lock:
        _fragments_received += 1
        _latest_fragment = copy.deepcopy(fragment)

    try:
        photons = _make_photons(fragment)
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
        return

    with _state_lock:
        _latest_photons = copy.deepcopy(photons)

    publish(PHOTONS_TOPIC, photons)

    with _state_lock:
        _photons_published += 1

    _persist_photons(photons)

    with _state_lock:
        _rows_persisted += 1


SUBSCRIPTIONS = {
    PHOTONS_FRAGMENT_TOPIC: on_photons_fragment,
}


# ---------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------

def cmd_report(_: Optional[dict]) -> dict:
    with _state_lock:
        payload = {
            "schema": "PHOTONS_REPORT_V1",
            "fragments_received": _fragments_received,
            "fragments_rejected": _fragments_rejected,
            "photons_published": _photons_published,
            "rows_persisted": _rows_persisted,
            "last_structural_rejection": copy.deepcopy(_last_structural_rejection),
            "latest_fragment": copy.deepcopy(_latest_fragment),
            "latest_photons": copy.deepcopy(_latest_photons),
        }

    return {
        "success": True,
        "message": "OK",
        "payload": payload,
    }


COMMANDS = {
    "REPORT": cmd_report,
}


# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    setup_logging()

    logging.info(
        "[photons] starting canonical PHOTONS_V1 service "
        "subscription=%s publication=%s campaign_type=%s",
        PHOTONS_FRAGMENT_TOPIC,
        PHOTONS_TOPIC,
        CAMPAIGN_TYPE_PHOTONS,
    )

    server_setup(
        subsystem=SUBSYSTEM,
        commands=COMMANDS,
        subscriptions=SUBSCRIPTIONS,
        blocking=True,
    )


if __name__ == "__main__":
    run()
