"""
ZPNet CLOCKS Process — TIMEBASE Authority (Pi-side) — v9 Dual OCXO

Core contract (v2026-03+):

  CLOCKS is a pure traffic cop.  It owns NO clock state.  It receives
  TIMEBASE_FRAGMENT from the Teensy (containing GNSS, DWT, OCXO1, and OCXO2
  nanoseconds), decorates each fragment with environment snapshots and
  GF-8802 discipline data, computes Welford statistics, and persists
  the result as an immutable TIMEBASE row.

  Architecture:
    • Teensy owns: GNSS ns, DWT ns/cycles, OCXO1 ns, OCXO2 ns — in TIMEBASE_FRAGMENT
    • GNSS owns: GF-8802 discipline state — in GET_GNSS_INFO
    • CLOCKS owns: correlation, campaign lifecycle

  Four clock domains: GNSS (reference), DWT, OCXO1, OCXO2.

  v9: Prediction-aware statistics.

    The Teensy (process_clocks.cpp v10) now computes trend-aware
    prediction statistics for DWT, OCXO1, and OCXO2.  Instead of measuring
    residuals against a fixed nominal frequency (which conflates
    thermal drift range with measurement noise), the Teensy does
    linear extrapolation from the two most recent per-second deltas:

      predicted_delta = 2 * prev_delta - prev_prev_delta
      prediction_residual = actual_delta - predicted_delta

    Welford statistics on the prediction residual directly measure
    the instrument's actual interpolation uncertainty — "how wrong
    is my best estimate of this second's crystal rate?"

    The prediction residual stddev is the authoritative confidence
    metric for sub-second interpolation.  For DWT this is ~3-4
    cycles (3-4 ns); for OCXO1/OCXO2 ~1 tick (100 ns).

    Pi-side changes:
      • DWT, OCXO1, and OCXO2 prediction stats are passed through from the
        Teensy fragment and persisted in TIMEBASE records.
      • Raw deltas (dwt_delta_raw, ocxo1_delta_raw, ocxo2_delta_raw) are persisted.
      • Pi-side Welford residual trackers for DWT, OCXO1, OCXO2 are
        REMOVED — the Teensy's prediction stats are strictly
        superior and the Pi-side trackers were always second-hand.
      • GNSS Pi-side residual tracking is RETAINED as a stream
        health canary (GNSS is phase-coherent so residual == 0;
        any deviation indicates a problem).
      • The TIMEBASE stats block uses Teensy prediction stats
        for DWT/OCXO1/OCXO2 and Pi-side canary stats for GNSS.
      • The report clock blocks surface prediction stddev as the
        authoritative interpolation uncertainty.

  Recovery is symmetric across all four domains:

    projected_gnss_ns = next_pps_count * NS_PER_SECOND
    projected_ns = projected_gnss_ns * last_clock_ns // last_gnss_ns

  v7 recovery hardening (four fixes):

    1. Campaign deactivated immediately on recovery entry — processor
       thread ignores all fragments during the entire recovery window.
    2. Fragment queue drained after Teensy STOP in warm recovery —
       eliminates stale-fragment poisoning of Welford accumulators.
    3. Small pps_count mismatches (≤5) are soft-skipped with re-arm
       instead of triggering a cascading hard-fault → recovery loop.
    4. Welford reset moved to AFTER sync fragment arrives and
       armed_pps_count is set — zero chance of stale data.

  Flash-cut campaign switching:

    START while a campaign is active performs a seamless "flash cut"
    to the new campaign.  The PPS stream continues uninterrupted.
    OCXOs are never disturbed.  Nanosecond counters and pps_count
    reset to zero under the new campaign name.

  Fragment queue architecture (retained):

    Fragment reception is decoupled from processing:
      • on_timebase_fragment() — PUBSUB handler.  Fast path.
      • _process_loop() — dedicated thread.  Straight-through path.

Responsibilities:
  * Receive TIMEBASE_FRAGMENT from Teensy (queue-buffered)
  * Fetch GF-8802 discipline snapshot from GNSS (GET_GNSS_INFO)
  * Augment fragment with GNSS time, environment, and system time
  * Publish TIMEBASE
  * Persist TIMEBASE to PostgreSQL (append-only, best-effort)
  * Pass through Teensy prediction statistics (authoritative)
  * Denormalize augmented report into active campaign payload
  * Recover clocks after restart if campaign is active
  * Flash-cut to new campaign while running (seamless switch)
  * Command GNSS into Time Only mode when campaign has a location

Semantics:
  * No smoothing, inference, or filtering
  * TIMEBASE records are sacred and immutable
  * Derivative statistics (tau, ppb) live only in
    the campaign report — never in the TIMEBASE row itself
  * Prediction stats from the Teensy are passed through verbatim
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

GNSS_POLL_INTERVAL = 5
GNSS_WAIT_LOG_INTERVAL = 60

# Draconian sync waits
SYNC_FRAGMENT_TIMEOUT_S = 2.0
SYNC_RECOVER_TIMEOUT_S = 5.0
SYNC_POLL_S = 0.005
SYNC_LOG_INTERVAL_S = 0.25

# Fragment queue: maxsize=0 means unbounded
FRAGMENT_QUEUE_MAXSIZE = 0

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
    "hard_fault_no_active_campaign": 0,
    "hard_fault_sync_timeout": 0,
    "last_hard_fault": {},
    "hard_faults_total": 0,
    "auto_recovery_failures": 0,
    "pps_mismatch_soft_skips": 0,

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

    # GNSS stream health (Pi-side canary only)
    "gnss_residual_nonzero": 0,
    "last_gnss_residual_anomaly": {},
}

# ---------------------------------------------------------------------
# GNSS stream health canary — lightweight Pi-side check
# ---------------------------------------------------------------------
#
# GNSS ticks are exact by definition (10 MHz phase-coherent), so the
# per-second residual from the Teensy should always be 0.  We track
# the GNSS residual Pi-side purely as a stream health canary: if it
# ever goes nonzero, something is wrong with the PPS or VCLOCK path.
#
# DWT, OCXO1, and OCXO2 residual tracking is NO LONGER done Pi-side.  The
# Teensy's prediction statistics (v10) are strictly superior.
#

_gnss_last_ns: int = 0
_gnss_residual_valid: bool = False


def _gnss_canary_update(gnss_ns: int) -> Dict[str, Any]:
    """
    Update GNSS stream health canary.

    Returns a dict with canary state for embedding in TIMEBASE stats.
    """
    global _gnss_last_ns, _gnss_residual_valid

    result: Dict[str, Any] = {
        "stream_valid": False,
        "residual": 0,
    }

    if _gnss_last_ns > 0:
        delta = gnss_ns - _gnss_last_ns
        residual = delta - NS_PER_SECOND
        result["stream_valid"] = True
        result["residual"] = int(residual)

        if residual != 0:
            _diag["gnss_residual_nonzero"] += 1
            _diag["last_gnss_residual_anomaly"] = {
                "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "gnss_ns": int(gnss_ns),
                "delta": int(delta),
                "residual": int(residual),
            }

        _gnss_residual_valid = True
    else:
        _gnss_residual_valid = False

    _gnss_last_ns = gnss_ns
    return result


def _gnss_canary_reset() -> None:
    """Reset GNSS canary state (on campaign start/stop/recover)."""
    global _gnss_last_ns, _gnss_residual_valid
    _gnss_last_ns = 0
    _gnss_residual_valid = False


# ---------------------------------------------------------------------
# Local state (process lifetime)
# ---------------------------------------------------------------------

_campaign_active: bool = False

_last_pps_count_seen: Optional[int] = None

# What pps_count did we most recently tell the Teensy to use?
_armed_pps_count: Optional[int] = None

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




def _wait_for_pubsub_route(
    *,
    context: str,
    topic: str,
    timeout_s: float = 30.0,
    poll_s: float = 0.5,
) -> None:
    """
    Wait until the Teensy PUBSUB report shows a specific route.
    """
    logging.info(
        "⏳ [%s] @%s waiting for PUBSUB routing for topic '%s'...",
        context, system_time_z(), topic,
    )
    route_wait_t0 = time.monotonic()

    while True:
        elapsed_wait = time.monotonic() - route_wait_t0
        try:
            resp = send_command(machine="TEENSY", subsystem="PUBSUB", command="REPORT")
            if resp.get("success"):
                routes = resp.get("payload", {}).get("routes", [])
                found = any(
                    r.get("topic") == topic
                    for r in routes
                )
                if found:
                    logging.info(
                        "✅ [%s] @%s PUBSUB route confirmed for topic '%s' (%.1fs)",
                        context, system_time_z(), topic, elapsed_wait,
                    )
                    return
        except RuntimeError:
            raise
        except Exception:
            pass

        if elapsed_wait >= timeout_s:
            raise RuntimeError(
                f"{context} failed: PUBSUB route '{topic}' not found in {timeout_s}s"
            )
        time.sleep(poll_s)


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

        # Power state: extract Pi, Teensy, OCXO1 domains from i2c-2
        i2c2 = power.get("i2c-2", {})
        pi_power = i2c2.get("0x40", {})
        teensy_power = i2c2.get("0x45", {})
        ocxo1_power = i2c2.get("0x44", {})
        ocxo2_power = i2c2.get("0x41", {})

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
            "ocxo1_power": {
                "volts": ocxo1_power.get("volts"),
                "amps": ocxo1_power.get("amps"),
                "watts": ocxo1_power.get("watts"),
            },
            "ocxo2_power": {
                "volts": ocxo2_power.get("volts"),
                "amps": ocxo2_power.get("amps"),
                "watts": ocxo2_power.get("watts"),
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

    Returns the flat payload dict or None if unavailable.
    Best-effort — a miss here should never block TIMEBASE production.
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
        if _sync_event.wait(timeout=min(remaining, SYNC_POLL_S)):
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
    frag: Dict[str, Any],
    domain: str,
) -> Dict[str, Any]:
    """
    Build a clock domain block for the campaign report.

    tau is cumulative (total clock ns / total GNSS ns).
    ppb is instantaneous (per-second delta vs nominal).
    Prediction stats from Teensy are authoritative.

    Args:
        ns_now: cumulative nanoseconds in this domain
        gnss_ns: cumulative GNSS nanoseconds (reference)
        frag: the raw TIMEBASE_FRAGMENT payload
        domain: "gnss", "dwt", "ocxo1", or "ocxo2"
    """
    block: Dict[str, Any] = {
        "ns_now": int(ns_now),
        "tau": round(_compute_tau(int(ns_now), int(gnss_ns)), 12),
    }

    # Instantaneous PPB from per-second raw deltas
    if domain == "dwt":
        delta = frag.get("dwt_delta_raw")
        if delta is not None:
            block["ppb"] = round(((int(delta) - 1_008_000_000) / 1_008_000_000) * 1e9, 3)
        else:
            block["ppb"] = 0.0
        block["pred_residual"] = frag.get("dwt_pred_residual")
        block["pred_mean"] = frag.get("dwt_pred_mean")
        block["pred_stddev"] = frag.get("dwt_pred_stddev")
        block["pred_n"] = frag.get("dwt_pred_n")
        block["delta_raw"] = frag.get("dwt_delta_raw")
        block["pps_residual"] = frag.get("dwt_pps_residual")
    elif domain == "ocxo1":
        delta = frag.get("ocxo1_delta_raw")
        if delta is not None:
            block["ppb"] = round(_compute_ppb(int(ns_now), int(gnss_ns)), 3)
        else:
            block["ppb"] = 0.0
        block["pred_residual"] = frag.get("ocxo1_pred_residual")
        block["pred_mean"] = frag.get("ocxo1_pred_mean")
        block["pred_stddev"] = frag.get("ocxo1_pred_stddev")
        block["pred_n"] = frag.get("ocxo1_pred_n")
        block["delta_raw"] = frag.get("ocxo1_delta_raw")
        block["pps_residual"] = frag.get("ocxo1_pps_residual")
    elif domain == "ocxo2":
        delta = frag.get("ocxo2_delta_raw")
        if delta is not None:
            # ocxo2_delta_raw is in 20 MHz space (QTimer both edges)
            # Convert to 10 MHz equivalent for PPB calculation
            block["ppb"] = round(_compute_ppb(int(ns_now), int(gnss_ns)), 3)
        else:
            block["ppb"] = 0.0
        block["pred_residual"] = frag.get("ocxo2_pred_residual")
        block["pred_mean"] = frag.get("ocxo2_pred_mean")
        block["pred_stddev"] = frag.get("ocxo2_pred_stddev")
        block["pred_n"] = frag.get("ocxo2_pred_n")
        block["delta_raw"] = frag.get("ocxo2_delta_raw")
        block["pps_residual"] = frag.get("ocxo2_pps_residual")
    elif domain == "gnss":
        block["ppb"] = 0.0
        block["pps_residual"] = frag.get("gnss_pps_residual")

    return block


def _build_report(
    campaign_name: str,
    campaign_payload: Dict[str, Any],
    timebase: Dict[str, Any],
    frag: Dict[str, Any],
) -> Dict[str, Any]:
    gnss_ns = int(timebase.get("teensy_gnss_ns") or 0)
    dwt_ns = int(timebase.get("teensy_dwt_ns") or 0)
    ocxo1_ns = int(timebase.get("teensy_ocxo1_ns") or 0)
    ocxo2_ns = int(timebase.get("teensy_ocxo2_ns") or 0)

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
        "teensy_ocxo1_ns": timebase.get("teensy_ocxo1_ns"),
        "teensy_ocxo2_ns": timebase.get("teensy_ocxo2_ns"),

        # Clock domain blocks (tau, ppb, prediction stats)
        "gnss": _build_clock_block(gnss_ns, gnss_ns, frag, "gnss"),
        "dwt": _build_clock_block(dwt_ns, gnss_ns, frag, "dwt"),
        "ocxo1": _build_clock_block(ocxo1_ns, gnss_ns, frag, "ocxo1"),
        "ocxo2": _build_clock_block(ocxo2_ns, gnss_ns, frag, "ocxo2"),

        # DWT internals (from TIMEBASE record)
        "dwt_cycles_per_pps": timebase.get("dwt_cycles_per_pps"),
        "dwt_cyccnt_at_pps": timebase.get("dwt_cyccnt_at_pps"),

        # ISR residuals — all four domains (from TIMEBASE record)
        "isr_residual_gnss": timebase.get("isr_residual_gnss"),
        "isr_residual_dwt": timebase.get("isr_residual_dwt"),
        "isr_residual_ocxo1": timebase.get("isr_residual_ocxo1"),
        "isr_residual_ocxo2": timebase.get("isr_residual_ocxo2"),

        # Spin capture forensics (from TIMEBASE record)
        "spin_valid": timebase.get("spin_valid"),
        "spin_delta_cycles": timebase.get("spin_delta_cycles"),
        "spin_error_cycles": timebase.get("spin_error_cycles"),
        "spin_approach_cycles": timebase.get("spin_approach_cycles"),
        "spin_tdc_correction": timebase.get("spin_tdc_correction"),
        "spin_nano_timed_out": timebase.get("spin_nano_timed_out"),
        "spin_shadow_timed_out": timebase.get("spin_shadow_timed_out"),

        # PPS rejection diagnostics (from TIMEBASE record)
        "pps_rejected_total": timebase.get("pps_rejected_total"),
        "pps_rejected_remainder": timebase.get("pps_rejected_remainder"),
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
    Dedicated thread: pulls fragments from queue, decorates with
    environment and GNSS discipline data, builds TIMEBASE, persists.
    Runs forever.

    v9: Prediction-aware.  Teensy prediction stats are passed through
    as the authoritative interpolation uncertainty metrics.  Pi-side
    DWT/OCXO1/OCXO2 Welford trackers removed.
    """
    global _campaign_active, _armed_pps_count

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
            delta = int(pps_count) - armed
            _diag["pps_mismatch_soft_skips"] = _diag.get("pps_mismatch_soft_skips", 0) + 1

            with _sync_lock:
                sync_active = _sync_expected_pps is not None

            if abs(delta) <= 5:
                logging.warning(
                    "⚠️ [clocks] @%s pps_count mismatch: "
                    "armed=%d teensy=%d (off by %+d) sync_active=%s "
                    "— soft skip, re-arming",
                    sysclk, armed, int(pps_count), delta, sync_active,
                )
                _armed_pps_count = int(pps_count) + 1
                _diag["armed_pps_count"] = _armed_pps_count
                continue

            # Large jump (>5) is genuinely anomalous — hard fault
            _diag["hard_fault_pps_mismatch"] += 1
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

        # Advance armed expectation to next second
        _armed_pps_count = int(pps_count) + 1
        _diag["armed_pps_count"] = _armed_pps_count

        # --- Extract clock values ---
        system_time_utc = datetime.now(timezone.utc)
        system_time_str = system_time_utc.isoformat(timespec="microseconds")

        gnss_ns = int(frag.get("gnss_ns") or 0)
        dwt_ns = int(frag.get("dwt_ns") or 0)
        ocxo1_ns = int(frag.get("ocxo1_ns") or 0)
        ocxo2_ns = int(frag.get("ocxo2_ns") or 0)

        # --- GNSS stream health canary (Pi-side only) ---
        gnss_canary = _gnss_canary_update(gnss_ns) if gnss_ns > 0 else {"stream_valid": False, "residual": 0}

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
            "gnss_time_utc": system_time_z(),

            "pps_count": int(pps_count),

            # Teensy authoritative clock state
            "teensy_dwt_cycles": frag["dwt_cycles"],
            "teensy_dwt_ns": frag["dwt_ns"],
            "teensy_gnss_ns": frag["gnss_ns"],
            "teensy_ocxo1_ns": frag.get("ocxo1_ns"),
            "teensy_ocxo2_ns": frag.get("ocxo2_ns"),

            "teensy_pps_count": int(pps_count),

            # Teensy ISR residuals (raw counts)
            "isr_residual_gnss": frag.get("isr_residual_gnss"),
            "isr_residual_dwt": frag.get("isr_residual_dwt"),
            "isr_residual_ocxo1": frag.get("isr_residual_ocxo1"),
            "isr_residual_ocxo2": frag.get("isr_residual_ocxo2"),

            # Teensy interpolation anchors
            "dwt_cyccnt_at_pps": frag.get("dwt_cyccnt_at_pps"),
            "gpt2_at_pps": frag.get("gpt2_at_pps"),
            "dwt_cycles_per_pps": frag.get("dwt_cycles_per_pps"),

            # Raw per-second deltas (ground truth)
            "dwt_delta_raw": frag.get("dwt_delta_raw"),
            "ocxo1_delta_raw": frag.get("ocxo1_delta_raw"),
            "ocxo2_delta_raw": frag.get("ocxo2_delta_raw"),

            # Teensy PPS residuals (nominal-based, retained for continuity)
            "dwt_pps_residual": frag.get("dwt_pps_residual"),
            "gnss_pps_residual": frag.get("gnss_pps_residual"),
            "ocxo1_pps_residual": frag.get("ocxo1_pps_residual"),
            "ocxo2_pps_residual": frag.get("ocxo2_pps_residual"),

            # OCXO control state (both OCXOs)
            "ocxo1_dac": frag.get("ocxo1_dac"),
            "ocxo2_dac": frag.get("ocxo2_dac"),
            "calibrate_ocxo": frag.get("calibrate_ocxo"),
            "ocxo1_servo_adjustments": frag.get("ocxo1_servo_adjustments"),
            "ocxo2_servo_adjustments": frag.get("ocxo2_servo_adjustments"),

            # PPS rejection diagnostics
            "pps_rejected_total": frag.get("diag_pps_rejected_total"),
            "pps_rejected_remainder": frag.get("diag_pps_rejected_remainder"),

            # Spin capture forensics (v19 field names)
            "spin_valid": frag.get("spin_valid"),
            "spin_delta_cycles": frag.get("spin_delta_cycles"),
            "spin_error_cycles": frag.get("spin_error_cycles"),
            "spin_approach_cycles": frag.get("spin_approach_cycles"),
            "spin_isr_dwt": frag.get("spin_isr_dwt"),
            "spin_landed_dwt": frag.get("spin_landed_dwt"),
            "spin_shadow_dwt": frag.get("spin_shadow_dwt"),
            "spin_corrected_dwt": frag.get("spin_corrected_dwt"),
            "spin_tdc_correction": frag.get("spin_tdc_correction"),
            "spin_nano_timed_out": frag.get("spin_nano_timed_out"),
            "spin_shadow_timed_out": frag.get("spin_shadow_timed_out"),

            # Environment snapshot (correlated with this PPS edge)
            "environment": env_snapshot,

            # GF-8802 discipline snapshot (correlated with this PPS edge)
            "gnss": gnss_info,

            # --- Stats block ---
            #
            # v9: DWT, OCXO1, and OCXO2 stats come from Teensy prediction tracking
            # (authoritative).  GNSS stats are a Pi-side stream health
            # canary only.
            #
            "stats": {
                "gnss": {
                    "stream_valid": gnss_canary["stream_valid"],
                    "residual": gnss_canary["residual"],
                    "tau": round(_compute_tau(gnss_ns, gnss_ns), 12) if gnss_ns else None,
                    "ppb": round(_compute_ppb(gnss_ns, gnss_ns), 3) if gnss_ns else None,
                },
                "dwt": {
                    "pred_residual": frag.get("dwt_pred_residual"),
                    "pred_mean": frag.get("dwt_pred_mean"),
                    "pred_stddev": frag.get("dwt_pred_stddev"),
                    "pred_n": frag.get("dwt_pred_n"),
                    "delta_raw": frag.get("dwt_delta_raw"),
                    "pps_residual": frag.get("dwt_pps_residual"),
                    "tau": round(_compute_tau(dwt_ns, gnss_ns), 12) if gnss_ns else None,
                    "ppb": round(_compute_ppb(dwt_ns, gnss_ns), 3) if gnss_ns else None,
                },
                "ocxo1": {
                    "pred_residual": frag.get("ocxo1_pred_residual"),
                    "pred_mean": frag.get("ocxo1_pred_mean"),
                    "pred_stddev": frag.get("ocxo1_pred_stddev"),
                    "pred_n": frag.get("ocxo1_pred_n"),
                    "delta_raw": frag.get("ocxo1_delta_raw"),
                    "pps_residual": frag.get("ocxo1_pps_residual"),
                    "tau": round(_compute_tau(ocxo1_ns, gnss_ns), 12) if gnss_ns else None,
                    "ppb": round(_compute_ppb(ocxo1_ns, gnss_ns), 3) if gnss_ns else None,
                },
                "ocxo2": {
                    "pred_residual": frag.get("ocxo2_pred_residual"),
                    "pred_mean": frag.get("ocxo2_pred_mean"),
                    "pred_stddev": frag.get("ocxo2_pred_stddev"),
                    "pred_n": frag.get("ocxo2_pred_n"),
                    "delta_raw": frag.get("ocxo2_delta_raw"),
                    "pps_residual": frag.get("ocxo2_pps_residual"),
                    "tau": round(_compute_tau(ocxo2_ns, gnss_ns), 12) if gnss_ns else None,
                    "ppb": round(_compute_ppb(ocxo2_ns, gnss_ns), 3) if gnss_ns else None,
                },
            },
        }

        report = _build_report(campaign, campaign_payload, timebase, frag)

        publish("TIMEBASE", timebase)
        _persist_timebase(timebase, report)

        # Persist OCXO1/OCXO2 DAC fields into campaign payload (best-effort)
        teensy_ocxo1_dac = frag.get("ocxo1_dac")
        teensy_ocxo2_dac = frag.get("ocxo2_dac")
        teensy_calibrate = frag.get("calibrate_ocxo")
        if teensy_ocxo1_dac is not None or teensy_ocxo2_dac is not None:
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
                                "ocxo1_dac": float(teensy_ocxo1_dac) if teensy_ocxo1_dac is not None else None,
                                "ocxo2_dac": float(teensy_ocxo2_dac) if teensy_ocxo2_dac is not None else None,
                                "calibrate_ocxo": bool(teensy_calibrate),
                            }),
                            campaign,
                        ),
                    )
            except Exception:
                logging.exception("⚠️ [clocks] failed to persist OCXO DAC values (ignored)")

        # Persist calibrated DAC to SYSTEM config so it survives
        # across campaigns.  Only during active calibration.
        if teensy_calibrate and teensy_ocxo1_dac is not None:
            try:
                with open_db() as conn:
                    cur = conn.cursor()
                    cur.execute(
                        """
                        UPDATE config
                        SET payload = payload || %s::jsonb
                        WHERE config_key = 'SYSTEM'
                        """,
                        (json.dumps({"ocxo1_dac": float(teensy_ocxo1_dac), "ocxo2_dac": float(teensy_ocxo2_dac) if teensy_ocxo2_dac is not None else None}),),
                    )
            except Exception:
                logging.exception("⚠️ [clocks] failed to persist OCXO DAC values to config (ignored)")


# ---------------------------------------------------------------------
# Control-plane: START / STOP / CLEAR / RECOVER
# ---------------------------------------------------------------------


def _reset_trackers() -> None:
    global _last_pps_count_seen
    _last_pps_count_seen = None
    _gnss_canary_reset()


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
    START — v5 three-domain architecture with flash-cut support.
    """
    global _campaign_active, _armed_pps_count

    campaign = args.get("campaign") if args else None
    if not campaign:
        return {"success": False, "message": "START requires 'campaign' argument"}

    # --- Check for active campaign (determines cold start vs flash-cut) ---
    active_row = _get_active_campaign()
    flash_cut = active_row is not None

    current_location = _get_current_location()

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
    set_dac2 = args.get("set_dac2")
    calibrate_ocxo = bool(args.get("calibrate_ocxo"))

    # Read default DAC from config if not specified
    if set_dac is None or set_dac2 is None:
        try:
            with open_db(row_dict=True) as conn:
                cur = conn.cursor()
                cur.execute("SELECT payload FROM config WHERE config_key = 'SYSTEM'")
                row = cur.fetchone()
                if row:
                    if set_dac is None and row["payload"].get("ocxo1_dac") is not None:
                        set_dac = float(row["payload"]["ocxo1_dac"])
                        logging.info("🔧 [clocks] using ocxo1_dac=%s from SYSTEM config", set_dac)
                    if set_dac2 is None and row["payload"].get("ocxo2_dac") is not None:
                        set_dac2 = float(row["payload"]["ocxo2_dac"])
                        logging.info("🔧 [clocks] using ocxo2_dac=%s from SYSTEM config", set_dac2)
        except Exception:
            logging.exception("⚠️ [clocks] failed to read SYSTEM config (ignored)")

    campaign_payload = {
        "location": location,
        "started_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
    }
    if set_dac is not None:
        campaign_payload["ocxo1_dac"] = float(set_dac)
    if set_dac2 is not None:
        campaign_payload["ocxo2_dac"] = float(set_dac2)
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

        if location is None and prev_location:
            location = prev_location
            campaign_payload["location"] = location
            logging.info(
                "📡 [start] inheriting location '%s' from previous campaign",
                location,
            )

    # Deactivate prior campaigns + create new one
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

    _request_teensy_stop_best_effort()

    # Drain stale fragments
    while not _fragment_queue.empty():
        try:
            _fragment_queue.get_nowait()
        except queue.Empty:
            break

    _reset_trackers()

    # Prepare sync wait FIRST
    _begin_sync_wait(expected_pps=0)

    # Arm TEENSY
    logging.info("📡 [start] @%s arming TEENSY START: pps_count=0", system_time_z())
    teensy_args: Dict[str, Any] = {"campaign": campaign}
    if set_dac is not None:
        teensy_args["set_dac"] = str(set_dac)
    if set_dac2 is not None:
        teensy_args["set_dac2"] = str(set_dac2)
    if calibrate_ocxo:
        teensy_args["calibrate_ocxo"] = "true"

    _request_teensy_start(campaign=campaign, pps_count=0, args=teensy_args)
    logging.info("📡 [start] @%s TEENSY START returned", system_time_z())

    # Wait for sync fragment
    logging.info("📡 [start] @%s waiting for sync fragment (pps_count=0)...", system_time_z())

    try:
        frag0, waited_s = _end_sync_wait()
    except Exception:
        return {"success": False, "message": "HARD FAULT during START sync (see logs/diag)"}

    # v12.3: Accept pps_count=0 (the first real measurement fragment).
    # The sync wait captured it for verification, but it also went into
    # the queue.  Don't drain the queue — let pps_count=0 be processed
    # normally so teensy_dwt_ns reflects exactly one delta.
    _armed_pps_count = 0
    _diag["armed_pps_count"] = 0

    _reset_trackers()
    _campaign_active = True

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
            "set_dac2": set_dac2,
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
    RECOVER — v6 four-domain architecture.
    """
    global _campaign_active, _armed_pps_count

    # Immediately deactivate so the processor thread ignores all
    # fragments during recovery.
    _campaign_active = False
    _armed_pps_count = None

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

        # Cold restart
        _wait_for_preflight("recovery/cold")

        try:
            effective_location = _ensure_gnss_mode_for_current_location()
            if effective_location:
                logging.info("📡 [recovery/cold] GNSS ensured in TO mode for '%s'", effective_location)
            else:
                logging.info("📡 [recovery/cold] GNSS ensured in NORMAL mode")
        except Exception as e:
            raise RuntimeError(f"recovery/cold failed: {e}")

        _wait_for_pubsub_route(context="recovery/cold", topic="TIMEBASE_FRAGMENT")

        _request_teensy_stop_best_effort()

        while not _fragment_queue.empty():
            try:
                _fragment_queue.get_nowait()
            except queue.Empty:
                break

        _reset_trackers()

        _begin_sync_wait(expected_pps=0)

        teensy_args: Dict[str, Any] = {"campaign": campaign_name}
        recover_ocxo1_dac = campaign_payload.get("ocxo1_dac")
        recover_ocxo2_dac = campaign_payload.get("ocxo2_dac")
        recover_calibrate = campaign_payload.get("calibrate_ocxo", False)
        if recover_ocxo1_dac is not None:
            teensy_args["set_dac"] = str(float(recover_ocxo1_dac))
        if recover_ocxo2_dac is not None:
            teensy_args["set_dac2"] = str(float(recover_ocxo2_dac))
        if recover_calibrate:
            teensy_args["calibrate_ocxo"] = "true"

        logging.info("📡 [recovery/cold] @%s arming TEENSY START: pps_count=0", system_time_z())
        _request_teensy_start(campaign=campaign_name, pps_count=0, args=teensy_args)

        try:
            frag, waited_s = _end_sync_wait()
        except Exception:
            raise

        _armed_pps_count = 1
        _diag["armed_pps_count"] = 1

        while not _fragment_queue.empty():
            try:
                _fragment_queue.get_nowait()
            except queue.Empty:
                break

        _reset_trackers()
        _campaign_active = True

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
    last_ocxo1_ns = int(last_tb.get("teensy_ocxo1_ns") or 0)
    last_ocxo2_ns = int(last_tb.get("teensy_ocxo2_ns") or 0)

    last_gnss_time_str = last_tb.get("gnss_time_utc") or last_tb.get("system_time_utc") or system_time_z()

    if last_gnss_ns == 0:
        logging.warning("⚠️ [recovery] teensy_gnss_ns=0 — using dwt_ns as proxy")
        last_gnss_ns = last_dwt_ns

    logging.info(
        "📐 [recovery] LAST TIMEBASE:\n"
        "    pps_count  = %d\n"
        "    gnss_ns    = %d\n"
        "    dwt_ns     = %d\n"
        "    ocxo1_ns   = %d\n"
        "    ocxo2_ns   = %d\n"
        "    gnss_time  = %s",
        last_pps_count, last_gnss_ns, last_dwt_ns, last_ocxo1_ns, last_ocxo2_ns,
        last_gnss_time_str,
    )

    recover_ocxo1_dac = campaign_payload.get("ocxo1_dac")
    recover_ocxo2_dac = campaign_payload.get("ocxo2_dac")
    recover_calibrate = campaign_payload.get("calibrate_ocxo", False)

    if recover_ocxo1_dac is not None:
        logging.info(
            "🔧 [recovery] OCXO1 DAC: %s (calibrate=%s)",
            recover_ocxo1_dac, recover_calibrate
        )

    # ------------------------------------------------------------------
    # Step 2: Wait for preflight
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

    _wait_for_pubsub_route(context="recovery", topic="TIMEBASE_FRAGMENT")

    # ------------------------------------------------------------------
    # Stop Teensy, quiesce, snap to second boundary
    # ------------------------------------------------------------------
    logging.info("📡 [recovery] @%s stopping Teensy...", system_time_z())
    _request_teensy_stop_best_effort()
    time.sleep(2.0)

    _drained = 0
    while not _fragment_queue.empty():
        try:
            _fragment_queue.get_nowait()
            _drained += 1
        except queue.Empty:
            break
    if _drained > 0:
        logging.info("🧹 [recovery] drained %d stale fragments from queue", _drained)

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
    # Step 4-5: Symmetric projection
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

    projected_gnss_ns = next_pps_count * NS_PER_SECOND
    projected_dwt_ns = projected_gnss_ns * last_dwt_ns // last_gnss_ns if last_gnss_ns > 0 else projected_gnss_ns
    projected_ocxo1_ns = projected_gnss_ns * last_ocxo1_ns // last_gnss_ns if (last_gnss_ns > 0 and last_ocxo1_ns > 0) else 0
    projected_ocxo2_ns = projected_gnss_ns * last_ocxo2_ns // last_gnss_ns if (last_gnss_ns > 0 and last_ocxo2_ns > 0) else 0

    tau_dwt = last_dwt_ns / last_gnss_ns if last_gnss_ns > 0 else 1.0
    tau_ocxo1 = last_ocxo1_ns / last_gnss_ns if (last_gnss_ns > 0 and last_ocxo1_ns > 0) else 1.0
    tau_ocxo2 = last_ocxo2_ns / last_gnss_ns if (last_gnss_ns > 0 and last_ocxo2_ns > 0) else 1.0

    logging.info(
        "📐 [recovery] SYMMETRIC PROJECTION (all nanoseconds):\n"
        "    formula: projected_ns = projected_gnss_ns × last_clock_ns ÷ last_gnss_ns\n"
        "    projected_gnss_ns = next_pps_count(%d) × NS_PER_SECOND = %d\n"
        "    last_gnss_ns      = %d\n"
        "    ---\n"
        "    GNSS:  projected = %d  (tau = 1.000000000000)\n"
        "    DWT:   projected = %d  (tau = %.12f, last_dwt_ns = %d)\n"
        "    OCXO1: projected = %d  (tau = %.12f, last_ocxo1_ns = %d)",
        next_pps_count, projected_gnss_ns, last_gnss_ns,
        projected_gnss_ns,
        projected_dwt_ns, tau_dwt, last_dwt_ns,
        projected_ocxo1_ns, tau_ocxo1, last_ocxo1_ns,
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
        "projected_ocxo1_ns": int(projected_ocxo1_ns),
        "projected_ocxo2_ns": int(projected_ocxo2_ns),
        "tau_dwt": round(tau_dwt, 12),
        "tau_ocxo1": round(tau_ocxo1, 12),
        "tau_ocxo2": round(tau_ocxo2, 12),
    }

    # ------------------------------------------------------------------
    # Step 6: Begin sync wait, arm Teensy RECOVER
    # ------------------------------------------------------------------
    _begin_sync_wait(expected_pps=int(next_pps_count))

    logging.info(
        "📡 [recovery] @%s arming TEENSY RECOVER:\n"
        "    pps_count = %d\n"
        "    dwt_ns    = %d\n"
        "    gnss_ns   = %d\n"
        "    ocxo1_ns  = %d\n"
        "    ocxo2_ns  = %d",
        system_time_z(), next_pps_count,
        projected_dwt_ns, projected_gnss_ns, projected_ocxo1_ns, projected_ocxo2_ns,
    )

    teensy_recover_args: Dict[str, Any] = {
        "campaign": campaign_name,
        "dwt_ns": str(int(projected_dwt_ns)),
        "gnss_ns": str(int(projected_gnss_ns)),
        "ocxo1_ns": str(int(projected_ocxo1_ns)),
        "ocxo2_ns": str(int(projected_ocxo2_ns)),
    }
    if recover_ocxo1_dac is not None:
        teensy_recover_args["set_dac"] = str(float(recover_ocxo1_dac))
    if recover_ocxo2_dac is not None:
        teensy_recover_args["set_dac2"] = str(float(recover_ocxo2_dac))
    if recover_calibrate:
        teensy_recover_args["calibrate_ocxo"] = "true"

    _request_teensy_recover(int(next_pps_count), teensy_recover_args)

    try:
        frag, waited_s = _end_sync_wait(timeout_s=SYNC_RECOVER_TIMEOUT_S)
    except Exception:
        raise

    logging.info("✅ [recovery] @%s sync fragment received (waited=%.3fs)", system_time_z(), waited_s)

    teensy_pps_count = int(frag.get("teensy_pps_count", next_pps_count))
    overshoot = teensy_pps_count - next_pps_count

    if overshoot != 0:
        logging.info(
            "📐 [recovery] pps_count overshoot: teensy=%d projected=%d (overshoot=%+d)",
            teensy_pps_count, next_pps_count, overshoot,
        )

    _armed_pps_count = teensy_pps_count + 1
    _diag["armed_pps_count"] = _armed_pps_count

    _drained_post = 0
    while not _fragment_queue.empty():
        try:
            _fragment_queue.get_nowait()
            _drained_post += 1
        except queue.Empty:
            break
    if _drained_post > 0:
        logging.info("🧹 [recovery] drained %d fragments accumulated during sync wait", _drained_post)

    _reset_trackers()
    _campaign_active = True

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
    for key in ("gnss", "dwt", "ocxo1", "ocxo2"):
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


def _check_preflight() -> tuple[bool, list[str]]:
    """
    Check whether the system is ready to start or recover a campaign.
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

            if not gnss.get("time_valid"):
                reasons.append("GNSS time not valid (no satellite time/date)")

            lock_quality = gnss.get("lock_quality", "WEAK")
            if lock_quality == "WEAK":
                reasons.append(
                    f"GNSS lock quality is WEAK "
                    f"(satellites={gnss.get('satellites', '?')}, "
                    f"hdop={gnss.get('hdop', '?')})"
                )

            if not gnss.get("pps_valid"):
                reasons.append("GNSS PPS not valid (discipline loop not active)")
            else:
                discipline = gnss.get("discipline", {})
                freq_mode = discipline.get("freq_mode", -1)
                freq_mode_name = discipline.get("freq_mode_name", "UNKNOWN")
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

    ready = len(reasons) == 0
    return ready, reasons


def _wait_for_preflight(context: str = "recovery") -> None:
    """
    Block until all preflight prerequisites are met.

    On cluster restart, the GNSS process needs a few seconds to parse
    its first NMEA sentences.  We poll silently at 2-second intervals
    for the first PREFLIGHT_GRACE_S seconds before switching to the
    normal 30-second polling with logged warnings.
    """
    attempt = 0
    t0 = time.monotonic()

    GRACE_S = 15.0        # silent fast-poll window
    GRACE_POLL_S = 2.0    # poll interval during grace period

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

        if elapsed >= GRACE_S:
            # Past grace period — log warnings at normal interval
            reason_block = "\n".join(f"    • {r}" for r in reasons)
            logging.info(
                "%s @%s not ready for %s (check #%d, %.0fs elapsed):\n%s",
                PREFLIGHT_LOG_PREFIX, system_time_z(),
                context, attempt, elapsed, reason_block,
            )
            time.sleep(PREFLIGHT_POLL_INTERVAL_S)
        else:
            # Grace period — poll silently and quickly
            time.sleep(GRACE_POLL_S)


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

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "count": len(campaigns),
            "campaigns": campaigns
        },
    }


# ---------------------------------------------------------------------
# CLOCKS_INFO
# ---------------------------------------------------------------------


def cmd_clocks_info(_: Optional[dict]) -> Dict[str, Any]:
    payload = {
        "campaign_active": _campaign_active,
        "last_pps_count_seen": _last_pps_count_seen,
        "sync_expected_pps": _sync_expected_pps,
        "diag": _diag,
    }
    return {"success": True, "message": "OK", "payload": payload}


# ============================================================================
# cmd_set_dac
# ============================================================================

def cmd_set_dac(args: Optional[dict]) -> Dict[str, Any]:
    """
    SET_DAC(dac, [dac2])

    Update OCXO DAC values in the SYSTEM config record.
    'dac' sets OCXO1, 'dac2' sets OCXO2.  Either or both may be specified.
    """
    if not args or ("dac" not in args and "dac2" not in args):
        return {"success": False, "message": "SET_DAC requires 'dac' and/or 'dac2' argument"}

    update_blob: Dict[str, Any] = {}

    if "dac" in args:
        try:
            dac = float(args["dac"])
        except (ValueError, TypeError):
            return {"success": False, "message": f"Invalid dac value: {args['dac']}"}
        if dac < 0 or dac > 4095:
            return {"success": False, "message": f"DAC value {dac} out of range (0–4095)"}
        update_blob["ocxo1_dac"] = dac

    if "dac2" in args:
        try:
            dac2 = float(args["dac2"])
        except (ValueError, TypeError):
            return {"success": False, "message": f"Invalid dac2 value: {args['dac2']}"}
        if dac2 < 0 or dac2 > 4095:
            return {"success": False, "message": f"DAC2 value {dac2} out of range (0–4095)"}
        update_blob["ocxo2_dac"] = dac2

    try:
        with open_db() as conn:
            cur = conn.cursor()
            cur.execute(
                """
                UPDATE config
                SET payload = payload || %s::jsonb
                WHERE config_key = 'SYSTEM'
                """,
                (json.dumps(update_blob),),
            )
            if cur.rowcount == 0:
                return {"success": False, "message": "No SYSTEM config record found"}
    except Exception as e:
        logging.exception("❌ [clocks] SET_DAC failed")
        return {"success": False, "message": str(e)}

    logging.info("🔧 [clocks] SET_DAC: %s", update_blob)
    return {"success": True, "message": "OK", "payload": update_blob}


COMMANDS = {
    "START": cmd_start,
    "STOP": cmd_stop,
    "RESUME": cmd_resume,
    "REPORT": cmd_report,
    "CLEAR": cmd_clear,
    "DELETE": cmd_delete,
    "SET_DAC": cmd_set_dac,
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
        "🕐 [clocks] v9 — Prediction-aware statistics. Teensy v10 trend-aware tracking. "
        "Four clock domains: GNSS (reference), DWT, OCXO1, OCXO2. "
        "DWT/OCXO1/OCXO2 prediction stats from Teensy are authoritative. "
        "Pi-side Welford for DWT/OCXO1/OCXO2 removed. GNSS stream canary retained. "
        "Recovery: campaign deactivated + queue drained + soft-skip mismatch (≤5) + late reset. "
        "START while active performs seamless flash-cut to new campaign. "
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

    # Recover or stop stray Teensy
    row = _get_active_campaign()
    if row is None:
        _request_teensy_stop_best_effort()
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