"""
ZPNet PHOTONS Process — PHOTONS_FRAGMENT ingress + canonical PHOTONS persistence.

This is intentionally a starter implementation.

Current contract:

    Teensy process_photons
        -> PHOTONS_FRAGMENT
        -> Pi zpnet-photons
        -> canonical PHOTONS
        -> PUBSUB PHOTONS
        -> campaign_detail

PHOTONS_FRAGMENT is still a bring-up schema, so this process deliberately does
very little interpretation.  It preserves the complete Teensy fragment inside
the canonical Pi wrapper and adds only Pi-owned envelope fields.

LANTERN campaign semantics, recovery, validation courts, optical statistics,
and final schema design are intentionally deferred.
"""

from __future__ import annotations

import copy
import json
import logging
import threading
from datetime import datetime, timezone
from typing import Any, Dict, Optional

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

# Explicitly provisional Pi-side wrapper schema.
PHOTONS_SCHEMA = "PHOTONS_V0"


# ---------------------------------------------------------------------
# Live state
# ---------------------------------------------------------------------

_state_lock = threading.Lock()

_latest_fragment: Optional[Payload] = None
_latest_photons: Optional[Payload] = None

_fragments_received = 0
_photons_published = 0
_rows_persisted = 0


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

def _utc_now_z() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _make_photons(fragment: Payload) -> Payload:
    """
    Form the provisional canonical Pi-side PHOTONS object.

    Important:
      * Preserve the complete firmware fragment verbatim beneath ``fragment``.
      * Do not infer optical science on the Pi.
      * The sequence mapping is temporary while PHOTONS_FRAGMENT remains V0.
    """
    return {
        "schema": PHOTONS_SCHEMA,
        "sequence": fragment.get("sequence"),
        "system_time": _utc_now_z(),
        "fragment": copy.deepcopy(fragment),
    }


def _persist_photons(photons: Payload) -> None:
    """
    Persist one provisional canonical PHOTONS row.

    The current mapping is intentionally simple:

      campaign_type = PHOTONS
      campaign      = NULL
      viable        = true
      payload       = canonical PHOTONS object
      sequence      = PHOTONS_FRAGMENT.sequence
      pps_count     = PHOTONS_FRAGMENT.last_pps_sequence

    These mappings are scaffolding and are expected to evolve with the real
    PHOTONS/LANTERN contract.
    """
    fragment = photons["fragment"]

    sequence = photons["sequence"]
    pps_count = fragment.get("last_pps_sequence")

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

    For the starter service the execution path is intentionally synchronous:

        receive -> wrap -> publish -> persist

    PHOTONS_FRAGMENT currently arrives once per second, so there is no reason
    yet to introduce queues, workers, retries, or recovery machinery.
    """
    global _latest_fragment
    global _latest_photons
    global _fragments_received
    global _photons_published
    global _rows_persisted

    photons = _make_photons(fragment)

    with _state_lock:
        _latest_fragment = copy.deepcopy(fragment)
        _latest_photons = copy.deepcopy(photons)
        _fragments_received += 1

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
            "schema": "PHOTONS_REPORT_V0",
            "fragments_received": _fragments_received,
            "photons_published": _photons_published,
            "rows_persisted": _rows_persisted,
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
        "[photons] starting starter PHOTONS service "
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
