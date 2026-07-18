"""
ZPNet CLOCKS Process — TIMEBASE Authority (Pi-side)

Core contract:

  CLOCKS is a traffic cop and final TIMEBASE arbiter. It owns no Teensy
  clock state. It receives one unified TIMEBASE_FRAGMENT candidate from
  the Teensy. That candidate contains the science fragment and an embedded
  forensics object. CLOCKS decorates the accepted candidate with Pi-owned
  environment, GF-8802, GNSS_RAW, and system-time evidence, then persists a
  compatible immutable TIMEBASE row shaped as {fragment, forensics}.

  There is no standalone TIMEBASE_FORENSICS subscription in this process and
  no two-topic pair join. The durable split between ``fragment`` and
  ``forensics`` is a PostgreSQL compatibility shape reconstructed from the
  single Teensy message.

  Architecture:
    • Teensy owns: PPS/VCLOCK identity, GNSS/VCLOCK ns, DWT cycles,
      OCXO1 ns, OCXO2 ns, firmware stats, and embedded forensics.
    • Pi owns: GNSS_RAW, GF-8802 correlation, environment correlation,
      campaign lifecycle, final acceptance court, recovery orchestration,
      and PostgreSQL persistence.
    • GNSS owns: receiver discipline state exposed through GET_GNSS_INFO.

  START behavior:

    START is asynchronous. The Pi creates/activates the campaign, sends the
    Teensy START command, and returns. The first public TIMEBASE row arrives
    later through the normal TIMEBASE_FRAGMENT processor. Firmware may privately
    acquire a lawful PPS0/start-prologue bookend before public PPS1; the Pi does
    not model those private candidates as skipped campaign rows.

  RECOVER behavior:

    RECOVER uses the last durable TIMEBASE row as the public base, sends a
    recover command to the Teensy, waits for the first Pi-accepted public row,
    and restores Pi-owned GNSS_RAW/Welford state immediately before that row is
    persisted. Timeline rows may be admitted while OCXO science is explicitly
    degraded/quarantined, but the final court still rejects malformed or
    incoherent candidates.

Responsibilities:
  * Receive unified TIMEBASE_FRAGMENT candidates from Teensy.
  * Adjudicate each candidate and log every rejected raw record.
  * Fetch GF-8802 discipline snapshots from GNSS.
  * Augment accepted rows with environment, GNSS_RAW, and system time.
  * Publish and persist TIMEBASE rows.
  * Denormalize the latest accepted TIMEBASE row into the active campaign.
  * Recover clocks after restart if a campaign is active.
  * Subscribe to WATCHDOG_ANOMALY and initiate Pi-side campaign recovery.
  * Flash-cut to a new campaign while preserving the hot Teensy stream.

Semantics:
  * No Pi-side smoothing, inference, or repair of Teensy clock state.
  * TIMEBASE records are immutable.
  * Gaps, jumps, regressions, and rejected rows are recorded as evidence.
  * WATCHDOG_ANOMALY is an explicit Teensy continuity surrender and starts
    Pi-side recovery from the latest canonical TIMEBASE row.
"""

from __future__ import annotations

import copy
import json
import logging
import math
import os
import queue
import threading
import subprocess
import time
from datetime import datetime, timezone
from logging.handlers import RotatingFileHandler
from typing import Dict, Any, List, Optional, Tuple

from zpnet.processes.processes import (
    server_setup,
    publish,
    send_command,
)
from zpnet.shared.constants import Payload
from zpnet.shared.db import open_db
from zpnet.shared.logger import setup_logging
from zpnet.shared.util import blocking_features, system_time_z
from zpnet.shared.events import create_event

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------

NS_PER_SECOND = 1_000_000_000

# Teensy DWT conversion constants mirror pnc/firmware/teensy/config.h.
# RECOVER still accepts a dwt_ns command argument, but current
# TIMEBASE_FRAGMENT_V3 publishes the DWT ledger in native cycles.
DWT_NS_NUM = 125
DWT_NS_DEN = 126
DWT_EXPECTED_PER_PPS = 1_008_000_000

GNSS_POLL_INTERVAL = 5
GNSS_WAIT_LOG_INTERVAL = 60

# Sync waits
#
# START/RECOVER are gated by readiness preflight. The Pi no longer expects
# fixed row burial/warmup suppression as part of normal campaign admission:
# the first public TIMEBASE row is supposed to be useful, and if it is not,
# the responsible readiness or handoff gate should be fixed.
RECOVERY_FIRST_PUBLIC_OFFSET = 1
SYNC_FRAGMENT_TIMEOUT_S = 35.0
SYNC_RECOVER_TIMEOUT_S = 45.0
SYNC_POLL_S = 0.005

# Recovery may legitimately consume hidden firmware candidates while
# Alpha/CounterLedger/PhaseLedger proves fresh OCXO custody.  This now bounds
# the wait for the first truthful timeline-admissible row, not for fully
# mature OCXO science.
SYNC_RECOVER_CLEAN_TIMEOUT_S = 90.0

# A recovery that produces no public TIMEBASE row inside this window is not
# merely "not clean yet".  Teensy should either publish a clean row or, after
# its bounded private reattach window, publish degraded rows that let the Pi
# observe liveness.  If the first row never appears, abort the firmware
# RECOVER lifecycle explicitly instead of recursively hard-faulting the Pi-side
# recovery thread.
RECOVERY_FIRST_ROW_TIMEOUT_S = 45.0

# If a new WATCHDOG_ANOMALY arrives while an auto-recovery attempt is already
# waiting for the first clean public row, the current attempt has been
# invalidated.  Abort that wait immediately, clean the Teensy RECOVER
# lifecycle, and retry from the latest durable TIMEBASE instead of sitting on a
# stale accepted-row wait until timeout.
AUTO_RECOVERY_MAX_ATTEMPTS = 3
AUTO_RECOVERY_RETRY_DELAY_S = 1.0

# Recovery admission is layered.  A truthful PPS/VCLOCK/GNSS/DWT timeline row
# is persistable even while OCXO clockface or refined science is still
# initializing.  The Teensy marks those rows explicitly; the Pi must not restart
# RECOVER merely because science_valid is false.
RECOVERY_ADMIT_DEGRADED_TIMELINE = True

# RECOVER transaction watchdog.  A Teensy USB/firmware reboot after CLOCKS.RECOVER
# has been accepted can erase the in-flight RECOVER lifecycle while the Pi is
# still waiting for the first public row.  Poll REPORT_RECOVERY during that
# wait and retry promptly if the recovered base identity disappears.
RECOVERY_SYNC_HEALTH_POLL_S = 2.0
RECOVERY_SYNC_HEALTH_GRACE_S = 3.0

SYNC_LOG_INTERVAL_S = 5.0

# TIMEBASE ingress queue: maxsize=0 means unbounded.
#
# The Teensy emits exactly one TIMEBASE_FRAGMENT candidate containing both the
# science fragment and an embedded ``forensics`` object. PostgreSQL still
# receives the compatible {fragment, forensics} TIMEBASE shape after Pi-side
# adjudication, but there is no standalone forensics route or two-topic pair
# join in the live architecture.
TIMEBASE_INGRESS_QUEUE_MAXSIZE = 0
TIMEBASE_FRAGMENT_TOPIC = "TIMEBASE_FRAGMENT"
CLOCKS_RECOVERY_STALLED_TOPIC = "CLOCKS_RECOVERY_STALLED"
TIMEBASE_CANDIDATE_ACCEPT = "ACCEPT"
TIMEBASE_CANDIDATE_SCIENCE_REJECT = "SCIENCE_REJECT"
TIMEBASE_CANDIDATE_DISPOSITIONS = {
    TIMEBASE_CANDIDATE_ACCEPT,
    TIMEBASE_CANDIDATE_SCIENCE_REJECT,
}

INVALID_TIMEBASE_LOG_PATH = os.environ.get(
    "ZPNET_INVALID_TIMEBASE_LOG_PATH",
    "/home/mule/zpnet/logs/clocks-invalid-timebase.jsonl",
)
INVALID_TIMEBASE_LOG_MAX_BYTES = 64 * 1024 * 1024
INVALID_TIMEBASE_LOG_BACKUP_COUNT = 4

# Final TIMEBASE courtroom.  This is the Pi-side last-mile acceptance gate:
# it evaluates the fully assembled TIMEBASE dictionary immediately before the
# row is published/persisted, so corruption introduced during final structure
# formation is caught before it becomes durable campaign truth.
TIMEBASE_FINAL_COURT_SOURCE = "PI_CLOCKS_FINAL_TIMEBASE_COURT"
TIMEBASE_FINAL_COURT_REASON = "timebase_final_court_violation"

# First final-court rule: if an OCXO Delta Raw interval is marked valid, then
# the final JSON must contain physically plausible one-second DWT intervals.
# Do not use a tight OCXO-vs-delayed-VCLOCK agreement gate as a fatal court:
# a real OCXO can be hundreds of cycles away from VCLOCK during startup,
# recovery, DAC settling, or servo correction.  Debug10 PPS 574 had
# clock=1 against a ~1,007,995,428-cycle reference; the broad plausibility
# band is the corruption trap that blocks that class before DB write.
TIMEBASE_FINAL_COURT_DWT_INTERVAL_MIN_CYCLES = 900_000_000
TIMEBASE_FINAL_COURT_DWT_INTERVAL_MAX_CYCLES = 1_100_000_000
# Non-fatal witness threshold only: records notable OCXO-vs-reference raw
# offsets for diagnostics, but never blocks TIMEBASE persistence by itself.
TIMEBASE_FINAL_COURT_DELTA_RAW_INTERVAL_GATE_CYCLES = 500

# Second final-court rule: after the first startup/recovery maturity rows, an
# OCXO lane may not publish as an all-zero science/public ledger.  A warm
# RECOVER can legitimately hide early private candidates while firmware
# reattaches OCXO custody, but once a public TIMEBASE row reaches the Pi,
# zero OCXO ns plus zero endpoints/intervals is lane absence, not science.
TIMEBASE_FINAL_COURT_OCXO_ZERO_MATURE_PUBLIC_COUNT = 2

# Final-court violations are row verdicts.  They are logged and dropped; a
# repeated scientific rejection remains a canonical campaign gap and does not
# become a recovery request merely by repetition.  These legacy constants stay
# visible in REPORT output during the transition, but no longer control recovery.
TIMEBASE_FINAL_COURT_ROW_FATAL_RULES = {
    "ocxo_science_valid",
}
TIMEBASE_FINAL_COURT_ROW_FATAL_ESCALATE_CONSECUTIVE = 3

# GNSS confession PPB candidates
#
# The GF-8802 reports pps_timing_error_ns in GET_GNSS_INFO.  The absolute
# level is a placement offset, not a frequency error.  The per-row change is
# the receiver's own confession that the PPS yardstick moved by that many ns
# during this one-second interval.  We keep the raw Teensy-authored PPB as the
# sacred default, but compute a PI-owned reference-normalized candidate beside
# it:
#
#   corrected_residual(t) = raw_residual(t)
#                           - [g_err(t) - g_err(t-1)]
#
# Large steps are treated as receiver solution redefinitions, not lawful
# steering corrections.  Do not correct across gaps or segment boundaries.
GNSS_CONFESSION_SEGMENT_STEP_GATE_NS = 50.0

# Canonical publication switch for the new PI-owned PPB object only.  The
# Teensy fragment and its stats.<lane>.ppb field are never rewritten here.
# Legal values: "RAW", "GNSS_CONFESSION".
GNSS_CONFESSION_PPB_PUBLISHED_SOURCE = "RAW"

# GNSS_RAW recovery sanity gate.  GNSS_RAW is a Pi-owned synthetic clockface
# accumulated from receiver drift_ppb.  Its accumulated tau/ppb should stay in
# the same order of magnitude as the restored drift Welford mean.  If an older
# recovery poisoned the synthetic clockface/reference ratio, rebuild the seed
# from the Welford mean instead of preserving the bad ratio.
GNSS_RAW_RECOVERY_REBUILD_GATE_PPB = 1000.0


def _gnss_confession_published_source() -> str:
    source = str(GNSS_CONFESSION_PPB_PUBLISHED_SOURCE or "RAW").upper()
    if source not in {"RAW", "GNSS_CONFESSION"}:
        return "RAW"
    return source

# Preflight is polled quickly so START follows readiness promptly, but the log
# remains quiet.  Pending prerequisites are summarized only after a short grace
# period, when the pending set changes, or at the periodic status interval.
PREFLIGHT_POLL_INTERVAL_S = 2.0
PREFLIGHT_QUIET_GRACE_S = 15.0
PREFLIGHT_STATUS_LOG_INTERVAL_S = 30.0
PREFLIGHT_LOG_PREFIX = "🛡️ [preflight]"
STARTUP_TEENSY_QUIET_DELAY_S = 20.0
TIMEBASE_SILENCE_TIMEOUT_S = 30.0
TIMEBASE_SILENCE_MONITOR_POLL_S = 1.0
TEENSY_HEALTH_RETRY_S = 60.0

# Async START/Flash Cut silence is not the same thing as a live campaign
# going dark. START/Flash Cut may still take several seconds while Teensy
# earns the first canonical row; the Pi waits patiently but expects to accept
# the first public row once it appears.
START_FIRST_FRAGMENT_TIMEOUT_S = 90.0
FLASH_CUT_FIRST_FRAGMENT_TIMEOUT_S = 180.0

# Feature-status campaign preflight.
#
# This first-pass gate deliberately uses the global PI SYSTEM feature tree,
# because Pi SYSTEM has the broadest horizon: Pi-local GNSS/host/power state
# plus imported Teensy-local timing readiness.  Runtime TIMEBASE row
# integrity, database command-contract checks, GNSS mode reconciliation, and
# recovery projection remain local CLOCKS logic.
#
# FLOORLINE and QTIMER_DWT_RULER are intentionally not campaign-admission
# gates.  Both remain valuable imported Teensy INTERRUPT feature surfaces, but
# they are diagnostic / quality witnesses whose current definitions can strobe
# during startup, recovery, and report-pressure windows.
#
# The Pi gate deliberately mirrors the Teensy CLOCKS campaign-custody gate.
# COUNTER32_LINEAGE and OCXO_PUBLIC_ORIGIN are admission prerequisites because
# Teensy START rejects while either is not NOMINAL.  STATIC_PREDICTION remains
# post-start evidence because it is not part of the Teensy admission court.
# CLOCKS also polls Teensy REPORT_GATE directly before START/RECOVER.  That
# report is the exact firmware admission authority.  When it is open, stale
# TEENSY.* INITIALIZING entries in PI SYSTEM's asynchronously imported feature
# mirror are retained as diagnostics but cannot veto the lawful transition.
FEATURE_PREFLIGHT_PROFILE = "CAMPAIGN_PREFLIGHT"
FEATURE_PREFLIGHT_REQUIRED = (
    "PI.SYSTEM.FEATURE_STATUS",
    "PI.SYSTEM.HOST",
    "PI.SYSTEM.POWER",
    "PI.SYSTEM.BATTERY",
    "PI.SYSTEM.TEENSY_FEATURE_IMPORT",
    "PI.GNSS.REPORT",
    "TEENSY.SYSTEM.FEATURE_STATUS",
    "TEENSY.INTERRUPT.PPS_VCLOCK_AUTHORITY",
    "TEENSY.INTERRUPT.QTIMER_COUNTER_CUSTODY",
    "TEENSY.INTERRUPT.COUNTER32_LINEAGE",
    "TEENSY.CLOCKS.DWT_CALIBRATION",
    "TEENSY.CLOCKS.SMARTZERO",
    "TEENSY.CLOCKS.ALPHA_EPOCH",
    "TEENSY.CLOCKS.OCXO_PUBLIC_ORIGIN",
)

FEATURE_PREFLIGHT_POST_START_EXPECTED = (
    "TEENSY.CLOCKS.STATIC_PREDICTION",
)

TEENSY_CAMPAIGN_GATE_LABEL_TO_FEATURE = {
    "T_FEATURE": "TEENSY.SYSTEM.FEATURE_STATUS",
    "PPS/V_AUTH": "TEENSY.INTERRUPT.PPS_VCLOCK_AUTHORITY",
    "QTIMER_CNT": "TEENSY.INTERRUPT.QTIMER_COUNTER_CUSTODY",
    "CTR32_LINE": "TEENSY.INTERRUPT.COUNTER32_LINEAGE",
    "DWT_CAL": "TEENSY.CLOCKS.DWT_CALIBRATION",
    "SMARTZERO": "TEENSY.CLOCKS.SMARTZERO",
    "ALPHA_EPOCH": "TEENSY.CLOCKS.ALPHA_EPOCH",
    "OCXO_ORIGIN": "TEENSY.CLOCKS.OCXO_PUBLIC_ORIGIN",
}

# ---------------------------------------------------------------------
# TIMEBASE ingress queue
# ---------------------------------------------------------------------

_fragment_queue: queue.Queue[Dict[str, Any]] = queue.Queue(maxsize=TIMEBASE_INGRESS_QUEUE_MAXSIZE)

# ---------------------------------------------------------------------
# Diagnostics (monotonic counters + last anomaly snapshots)
# ---------------------------------------------------------------------

_diag: Dict[str, Any] = {
    # Unified candidate ingress (PUBSUB handler — fast path)
    "timebase_candidates_received": 0,
    "timebase_candidates_queued": 0,
    "timebase_candidates_processed": 0,

    # Legacy fragment aliases retained for REPORT compatibility. In the live
    # architecture each fragment is the complete TIMEBASE candidate, including
    # embedded forensics.
    "fragments_received": 0,
    "fragments_queued": 0,
    "fragments_missing_teensy_pps_count": 0,     # legacy diagnostic alias
    "fragments_missing_teensy_pps_vclock_count": 0,

    # Unified TIMEBASE candidate processing (processor thread — slow path).
    "timebase_pieces_processed": 0,              # legacy alias: queue items
    "timebase_rows_completed": 0,
    "timebase_pairs_completed": 0,               # legacy alias for rows completed

    # TIMEBASE final acceptance court (processor thread — last gate)
    "timebase_final_court_checks": 0,
    "timebase_final_court_passed": 0,
    "timebase_final_court_blocked": 0,
    "timebase_final_court_event_enqueue_failures": 0,
    "timebase_final_court_recovery_started": 0,
    "last_timebase_final_court": {},
    "timebase_final_court_delta_raw_offset_observed": 0,
    "last_timebase_final_court_delta_raw_offset": {},
    "timebase_final_court_row_dropped": 0,
    "timebase_final_court_row_fatal_escalated": 0,
    "timebase_final_court_degraded_recovery_admitted": 0,
    "timebase_final_court_consecutive_row_fatal": 0,
    "timebase_final_court_row_fatal_escalate_threshold": TIMEBASE_FINAL_COURT_ROW_FATAL_ESCALATE_CONSECUTIVE,
    "last_timebase_final_court_row_drop": {},

    # Firmware-authored candidate disposition.  SCIENCE_REJECT is heartbeat
    # testimony, not a TIMEBASE row and not a recovery request.
    "firmware_science_reject_received": 0,
    "firmware_science_reject_logged": 0,
    "firmware_science_reject_log_failures": 0,
    "last_firmware_science_reject": {},

    # Dedicated invalid-TIMEBASE JSONL evidence log.
    "invalid_timebase_log_path": INVALID_TIMEBASE_LOG_PATH,
    "invalid_timebase_log_ready": False,
    "invalid_timebase_log_writes": 0,
    "invalid_timebase_log_failures": 0,
    "last_invalid_timebase_log": {},

    # Candidate processing (processor thread — slow path). Fragment keys are
    # legacy aliases because the route name remains TIMEBASE_FRAGMENT.
    "fragments_processed": 0,
    "fragments_ignored_no_campaign": 0,
    "timebase_candidates_ignored_no_campaign": 0,
    "queue_depth_max_seen": 0,
    "queue_depth_current": 0,

    # Fault/recovery accounting
    "hard_fault_no_active_campaign": 0,
    "hard_fault_sync_timeout": 0,
    "last_hard_fault": {},
    "hard_faults_total": 0,
    "auto_recovery_failures": 0,
    "auto_recovery_attempts": 0,
    "auto_recovery_interrupted": 0,
    "auto_recovery_retries": 0,
    "recovery_interruption_requests": 0,
    "last_recovery_interruption": {},
    "recovery_abort_requests": 0,
    "recovery_abort_success": 0,
    "recovery_abort_failures": 0,
    "last_recovery_abort": {},

    # Sync waits
    "sync_waits": 0,
    "sync_wait_success": 0,
    "sync_wait_seconds_total": 0.0,
    "sync_wait_seconds_last": 0.0,
    "last_sync_wait": {},

    # PPS/VCLOCK count continuity (campaign fact, as observed from Teensy)
    "pps_count_seen": 0,              # legacy diagnostic alias
    "pps_vclock_count_seen": 0,
    "pps_count_repeat": 0,
    "pps_count_jump": 0,
    "pps_count_regress": 0,
    "last_pps_count": None,          # legacy diagnostic alias
    "last_pps_vclock_count": None,
    "accepted_pps_count": None,      # legacy diagnostic alias
    "accepted_pps_vclock_count": None,

    # Asynchronous START / first fragment observation
    "start_async_requests": 0,
    "start_waiting_for_first_fragment": False,
    "start_requested_campaign": None,
    "start_requested_at_utc": None,
    "start_first_fragment_at_utc": None,
    "start_first_fragment_wait_s": None,
    "start_first_fragment_pps_vclock_count": None,
    "last_start_async": {},
    "teensy_start_responses": 0,
    "teensy_start_accepted": 0,
    "teensy_start_rejected": 0,
    "teensy_start_malformed": 0,
    "last_teensy_start_response": {},

    "last_pps_count_anomaly": {},    # legacy diagnostic alias
    "last_pps_vclock_count_anomaly": {},

    # Recovery accounting
    "recovery_checks": 0,
    "recovery_no_active_campaign": 0,
    "recovery_missing_timebase": 0,
    "recovery_missing_last_pps_count": 0,        # legacy diagnostic alias
    "recovery_missing_last_pps_vclock_count": 0,
    "recovery_elapsed_seconds_nonpositive": 0,
    "recovery_last_timebase_unrecoverable": 0,
    "recovery_last_timebase_scan_count": 0,
    "recovery_control_state_reassertions": 0,
    "last_recovery_control_state": {},
    "last_recovery": {},
    "recovery_transitional_rows_discarded": 0,
    "recovery_clean_timeouts": 0,
    "recovery_clean_stalls": 0,
    "recovery_degraded_rows_admitted": 0,
    "recovery_science_clean_rows_admitted": 0,
    "last_recovery_admission": {},
    "recovery_inflight_health_polls": 0,
    "recovery_inflight_health_empty": 0,
    "recovery_inflight_command_lost": 0,
    "last_recovery_inflight_health": {},
    "last_recovery_command_lost": {},
    "last_recovery_transitional_row": {},
    "last_recovery_clean_timeout": {},

    # GNSS wait
    "gnss_waits": 0,
    "gnss_wait_success": 0,
    "gnss_wait_seconds_total": 0.0,
    "gnss_wait_seconds_last": 0.0,
    "last_gnss_wait": {},

    # Feature-status campaign preflight
    "preflight_feature_checks": 0,
    "preflight_feature_blocked": 0,
    "preflight_feature_unavailable": 0,
    "preflight_teensy_mirror_bypass_count": 0,
    "last_preflight_teensy_mirror_bypass": {},
    "last_preflight_feature_gate": {},
    "teensy_campaign_gate_checks": 0,
    "teensy_campaign_gate_blocked": 0,
    "teensy_campaign_gate_unavailable": 0,
    "last_teensy_campaign_gate": {},
    "preflight_wait_log_count": 0,
    "last_preflight_wait": {},

    # GNSS discipline info fetch
    "gnss_info_requests": 0,
    "gnss_info_hits": 0,
    "gnss_info_misses": 0,

    # GNSS confession PPB candidates.  This is PI-owned reference-normalization
    # telemetry: raw Teensy fields remain untouched, but TIMEBASE carries a
    # parallel candidate showing what the OCXO residual/PPB would be if the
    # GF-8802's own PPS placement confession is subtracted.
    "gnss_confession_checks": 0,
    "gnss_confession_rows_corrected": 0,
    "gnss_confession_rows_missing": 0,
    "gnss_confession_segment_steps": 0,
    "gnss_confession_gaps": 0,
    "gnss_confession_total_abs_correction_ns": 0.0,
    "gnss_confession_first_error_ns": None,
    "gnss_confession_last_error_ns": None,
    "gnss_confession_published_source": _gnss_confession_published_source(),
    "last_gnss_confession": {},

    # GNSS_RAW recovery projection / sanity rebuild
    "gnss_raw_recovery_restore_count": 0,
    "gnss_raw_recovery_rebuild_count": 0,
    "last_gnss_raw_recovery": {},

    # GNSS stream health (Pi-side canary only)
    "gnss_residual_nonzero": 0,
    "last_gnss_residual_anomaly": {},

    # WATCHDOG_ANOMALY ingress / recovery
    "watchdog_anomalies_received": 0,
    "watchdog_anomaly_recovery_started": 0,
    "watchdog_anomaly_event_enqueue_failures": 0,
    "last_watchdog_anomaly": {},

    # Dedicated recovery-liveness anomaly.  This is observational and never
    # initiates RECOVER; restarting would destroy the evidence being awaited.
    "recovery_stalled_events_received": 0,
    "recovery_stalled_event_enqueue_failures": 0,
    "last_recovery_stalled": {},

    # CLOCKS startup lifecycle serialization
    "startup_control_ready": False,
    "startup_control_busy_rejections": 0,
    "last_startup_control_rejection": {},

    # TIMEBASE silence / Teensy restart detection
    "timebase_silence_monitor_started": False,
    "timebase_silence_checks": 0,
    "timebase_silence_detected": 0,
    "timebase_silence_recovery_started": 0,
    "teensy_health_probe_attempts": 0,
    "teensy_health_probe_failures": 0,
    "teensy_health_probe_success": 0,
    "last_timebase_activity": {},
    "last_timebase_silence": {},

    # Flash Cut lifecycle
    "flash_cut_requests": 0,
    "flash_cut_waiting": False,
    "flash_cut_from": None,
    "flash_cut_to": None,
    "flash_cut_requested_at_utc": None,
    "flash_cut_first_fragment_at_utc": None,
    "flash_cut_first_fragment_wait_s": None,
    "flash_cut_first_fragment_pps_vclock_count": None,
    "flash_cut_cold_recovery_deferred": 0,
    "last_flash_cut": {},
}


_invalid_timebase_logger = logging.getLogger("zpnet.clocks.invalid_timebase")
_invalid_timebase_logger.propagate = False
_invalid_timebase_logger_ready = False


def _setup_invalid_timebase_logger() -> None:
    """Create the dedicated rotating JSONL log used only for rejected rows."""
    global _invalid_timebase_logger_ready

    if _invalid_timebase_logger_ready:
        return

    try:
        parent = os.path.dirname(INVALID_TIMEBASE_LOG_PATH)
        if parent:
            os.makedirs(parent, exist_ok=True)

        handler = RotatingFileHandler(
            INVALID_TIMEBASE_LOG_PATH,
            maxBytes=INVALID_TIMEBASE_LOG_MAX_BYTES,
            backupCount=INVALID_TIMEBASE_LOG_BACKUP_COUNT,
            encoding="utf-8",
        )
        handler.setFormatter(logging.Formatter("%(message)s"))
        _invalid_timebase_logger.handlers.clear()
        _invalid_timebase_logger.addHandler(handler)
        _invalid_timebase_logger.setLevel(logging.INFO)
        _invalid_timebase_logger_ready = True
        _diag["invalid_timebase_log_path"] = INVALID_TIMEBASE_LOG_PATH
        _diag["invalid_timebase_log_ready"] = True
    except Exception:
        _invalid_timebase_logger_ready = False
        _diag["invalid_timebase_log_ready"] = False
        _diag["invalid_timebase_log_failures"] += 1
        logging.exception(
            "⚠️ [clocks] unable to initialize invalid TIMEBASE log at %s",
            INVALID_TIMEBASE_LOG_PATH,
        )


def _log_invalid_timebase(
    *,
    verdict: Dict[str, Any],
    raw_record: Dict[str, Any],
    assembled_timebase: Dict[str, Any],
) -> None:
    """Write one complete rejected candidate and its Pi verdict as JSONL."""
    if not _invalid_timebase_logger_ready:
        _setup_invalid_timebase_logger()

    entry = {
        "logged_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "classification": str(verdict.get("classification") or "DROP_ROW"),
        "verdict": verdict,
        "raw_record": raw_record,
        "assembled_timebase": assembled_timebase,
    }

    try:
        if not _invalid_timebase_logger_ready:
            raise RuntimeError("invalid TIMEBASE logger is unavailable")
        _invalid_timebase_logger.info(
            json.dumps(entry, sort_keys=True, separators=(",", ":"), default=str)
        )
        _diag["invalid_timebase_log_writes"] += 1
        _diag["last_invalid_timebase_log"] = {
            "logged_at_utc": entry["logged_at_utc"],
            "campaign": verdict.get("campaign"),
            "teensy_pps_vclock_count": verdict.get("teensy_pps_vclock_count"),
            "primary_rule": verdict.get("primary_rule"),
            "rationale": verdict.get("rationale"),
            "path": INVALID_TIMEBASE_LOG_PATH,
        }
    except Exception:
        _diag["invalid_timebase_log_failures"] += 1
        logging.exception(
            "⚠️ [clocks] failed to write rejected TIMEBASE evidence; row remains dropped"
        )


def _candidate_disposition(fragment: Dict[str, Any]) -> str:
    """Return the normalized firmware-authored candidate disposition."""
    raw = str(fragment.get("candidate_disposition") or TIMEBASE_CANDIDATE_ACCEPT)
    return raw.strip().upper()


def _drop_firmware_science_reject(
    *,
    campaign: str,
    pps_vclock_count: int,
    fragment: Dict[str, Any],
    raw_record: Dict[str, Any],
    assembled_timebase: Dict[str, Any],
) -> None:
    """Log one explicit firmware science rejection without publishing TIMEBASE."""
    reason_code = _as_int(fragment.get("candidate_reason_code")) or 0
    reason = str(fragment.get("candidate_reason") or "unspecified")
    source = str(fragment.get("candidate_source") or "UNKNOWN")
    lane = _as_int(fragment.get("candidate_lane")) or 0
    reject_mask = _as_int(fragment.get("candidate_reject_mask")) or 0
    details = {
        "detail0": _as_int(fragment.get("candidate_detail0")) or 0,
        "detail1": _as_int(fragment.get("candidate_detail1")) or 0,
        "detail2": _as_int(fragment.get("candidate_detail2")) or 0,
        "detail3": _as_int(fragment.get("candidate_detail3")) or 0,
    }
    verdict = {
        "schema": "PI_FIRMWARE_SCIENCE_REJECT_V1",
        "valid": False,
        "classification": "FIRMWARE_SCIENCE_REJECT",
        "court_classification": "DROP_ROW",
        "reason": "firmware_science_reject",
        "primary_rule": "candidate_disposition",
        "rationale": "firmware testified that this campaign second is not canonical science",
        "source": source,
        "source_process": "TEENSY_CLOCKS",
        "source_report": "TIMEBASE_FRAGMENT",
        "campaign": campaign,
        "teensy_pps_vclock_count": int(pps_vclock_count),
        "teensy_pps_count": int(pps_vclock_count),
        "pps_count": int(pps_vclock_count),
        "candidate_disposition": TIMEBASE_CANDIDATE_SCIENCE_REJECT,
        "candidate_use": fragment.get("candidate_use"),
        "candidate_reason_code": int(reason_code),
        "candidate_reason": reason,
        "candidate_source": source,
        "candidate_lane": int(lane),
        "candidate_reject_mask": int(reject_mask),
        "candidate_details": details,
        "timeline_valid": fragment.get("timeline_valid"),
        "ocxo_clockface_valid": fragment.get("ocxo_clockface_valid"),
        "ocxo_science_valid": fragment.get("ocxo_science_valid"),
        "recovery_requested": False,
        "invalid_timebase_log_path": INVALID_TIMEBASE_LOG_PATH,
    }

    _diag["firmware_science_reject_received"] += 1
    _diag["last_firmware_science_reject"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        **verdict,
    }
    before_writes = int(_diag.get("invalid_timebase_log_writes") or 0)
    _log_invalid_timebase(
        verdict=verdict,
        raw_record=raw_record,
        assembled_timebase=assembled_timebase,
    )
    if int(_diag.get("invalid_timebase_log_writes") or 0) > before_writes:
        _diag["firmware_science_reject_logged"] += 1
    else:
        _diag["firmware_science_reject_log_failures"] += 1

    logging.error(
        "🧪 [clocks] firmware SCIENCE_REJECT — campaign=%s count=%d "
        "source=%s reason=%s(%d) lane=%d mask=0x%08x details=%s; "
        "candidate remains heartbeat evidence but will NOT publish TIMEBASE or persist PostgreSQL. "
        "Full raw candidate follows: %s",
        campaign,
        int(pps_vclock_count),
        source,
        reason,
        int(reason_code),
        int(lane),
        int(reject_mask),
        details,
        json.dumps(raw_record, sort_keys=True, separators=(",", ":"), default=str),
    )


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
# GNSS_RAW — synthetic clock from GF-8802 clock drift (TPS1)
# ---------------------------------------------------------------------
#
# The GF-8802 reports its internal TCXO frequency deviation (drift_ppb)
# every second via the CRW/TPS1 sentence.  We synthesize a virtual
# clock by accumulating NS_PER_SECOND + drift_ppb each PPS.  This
# clock represents what an undisciplined crystal in the same physical
# environment experiences — the environmental forcing function made
# visible as a clock domain.
#
# drift_ppb is the GNSS positioning engine's measurement of its own
# 26 MHz TCXO error after internal servo correction.  It correlates
# with temperature, satellite geometry, and multipath conditions.
#
# GNSS_RAW is Pi-only: it does not exist in TIMEBASE_FRAGMENT.
#

_gnss_raw_ns: float = 0.0
_gnss_raw_n: int = 0
_gnss_raw_valid: bool = False

# Welford accumulator for GNSS_RAW drift_ppb residuals
_gnss_raw_welford_n: int = 0
_gnss_raw_welford_mean: float = 0.0
_gnss_raw_welford_m2: float = 0.0
_gnss_raw_welford_min: float = 1e30
_gnss_raw_welford_max: float = -1e30


# PI-owned GNSS-confession-corrected PPB candidates.  These are running means
# of one-second fast residuals, mirroring zpnet/tests/raw_ppb.py.  They do not
# rewrite Teensy-authored fragment.stats or fragment.<lane>.science.
_gnss_confession_prev_pps: Optional[int] = None
_gnss_confession_prev_error_ns: Optional[float] = None
_gnss_confession_first_error_ns: Optional[float] = None
_gnss_confession_raw_total: Dict[str, float] = {"ocxo1": 0.0, "ocxo2": 0.0}
_gnss_confession_corrected_total: Dict[str, float] = {"ocxo1": 0.0, "ocxo2": 0.0}
_gnss_confession_n: Dict[str, int] = {"ocxo1": 0, "ocxo2": 0}


def _gnss_confession_reset() -> None:
    """Reset PI-owned GNSS confession PPB candidate state."""
    global _gnss_confession_prev_pps, _gnss_confession_prev_error_ns
    global _gnss_confession_first_error_ns

    _gnss_confession_prev_pps = None
    _gnss_confession_prev_error_ns = None
    _gnss_confession_first_error_ns = None
    for lane in ("ocxo1", "ocxo2"):
        _gnss_confession_raw_total[lane] = 0.0
        _gnss_confession_corrected_total[lane] = 0.0
        _gnss_confession_n[lane] = 0


def _gnss_confession_error_ns(gnss_info: Optional[Dict[str, Any]]) -> Optional[float]:
    """Return GF-8802 pps_timing_error_ns as a finite float, if present."""
    if not isinstance(gnss_info, dict):
        return None

    value = gnss_info.get("pps_timing_error_ns")
    if value is None:
        discipline = gnss_info.get("discipline")
        if isinstance(discipline, dict):
            value = discipline.get("pps_timing_error_ns")
    if value is None:
        return None

    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def _ocxo_fast_residual_for_ppb_candidate(frag: Dict[str, Any], lane: str) -> Optional[float]:
    """Extract the Teensy-authored raw one-second OCXO fast residual."""
    science = _path_get(frag, f"{lane}.science")
    if not isinstance(science, dict):
        return None

    # Prefer the public/canonical fast residual surface, then fall back through
    # the explicitly raw Delta surface and older integer renderings.  This keeps
    # the candidate framework aligned with raw_ppb.py while avoiding any
    # recomputation in the CLOCKS traffic-cop layer.
    for key in (
        "fast_residual_ns_exact",
        "fast_residual_ns",
        "delta_raw_fast_residual_ns_exact",
        "delta_raw_fast_residual_ns",
    ):
        value = science.get(key)
        if value is None:
            continue
        try:
            out = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(out):
            return out
    return None


def _gnss_confession_update(
    *,
    pps_vclock_count: int,
    frag: Dict[str, Any],
    gnss_info: Optional[Dict[str, Any]],
) -> Dict[str, Any]:
    """Update and return PI-owned raw/corrected OCXO PPB candidates."""
    global _gnss_confession_prev_pps, _gnss_confession_prev_error_ns
    global _gnss_confession_first_error_ns

    _diag["gnss_confession_checks"] += 1

    public_count = int(pps_vclock_count)
    g_err = _gnss_confession_error_ns(gnss_info)
    gap = (
        _gnss_confession_prev_pps is not None
        and public_count != int(_gnss_confession_prev_pps) + 1
    )

    d_g: Optional[float] = None
    segment_step = False
    correction = 0.0
    reason = "ok"

    if g_err is None:
        _diag["gnss_confession_rows_missing"] += 1
        reason = "g_err_missing"
        # Missing confession breaks continuity; do not delta across it.
        _gnss_confession_prev_error_ns = None
    else:
        if _gnss_confession_first_error_ns is None:
            _gnss_confession_first_error_ns = g_err
            _diag["gnss_confession_first_error_ns"] = g_err
        _diag["gnss_confession_last_error_ns"] = g_err

        if gap:
            _diag["gnss_confession_gaps"] += 1
            reason = "pps_gap"
            _gnss_confession_prev_error_ns = None

        if _gnss_confession_prev_error_ns is not None:
            d_g = g_err - _gnss_confession_prev_error_ns
            if abs(d_g) >= GNSS_CONFESSION_SEGMENT_STEP_GATE_NS:
                segment_step = True
                d_g = None
                reason = "segment_step"
                _diag["gnss_confession_segment_steps"] += 1
            elif d_g != 0.0:
                correction = d_g
                reason = "corrected"
                _diag["gnss_confession_rows_corrected"] += 1
                _diag["gnss_confession_total_abs_correction_ns"] += abs(d_g)

    candidates: Dict[str, Any] = {
        "schema": "GNSS_CONFESSION_PPB_CANDIDATES_V1",
        "description": "PI-owned PPB candidates; raw Teensy PPB remains untouched",
        "canonical_fragment_unchanged": True,
        "published_source": _gnss_confession_published_source(),
        "candidate_sources": ["RAW", "GNSS_CONFESSION"],
        "segment_step_gate_ns": float(GNSS_CONFESSION_SEGMENT_STEP_GATE_NS),
        "pps_count": public_count,
        "gnss_confession": {
            "valid": g_err is not None,
            "pps_timing_error_ns": g_err,
            "delta_ns": d_g,
            "correction_ns": correction,
            "segment_step": segment_step,
            "gap": bool(gap),
            "reason": reason,
            "first_error_ns": _gnss_confession_first_error_ns,
            "net_error_ns": (
                None if g_err is None or _gnss_confession_first_error_ns is None
                else g_err - _gnss_confession_first_error_ns
            ),
        },
    }

    for lane in ("ocxo1", "ocxo2"):
        raw_residual = _ocxo_fast_residual_for_ppb_candidate(frag, lane)
        lane_obj: Dict[str, Any] = {
            "valid": raw_residual is not None,
            "raw_residual_ns": raw_residual,
            "gnss_confession_corrected_residual_ns": None,
            "raw_running_ppb": None,
            "gnss_confession_running_ppb": None,
            "published_ppb": None,
            "published_source": _gnss_confession_published_source(),
        }
        if raw_residual is not None:
            corrected_residual = raw_residual - correction
            _gnss_confession_n[lane] += 1
            _gnss_confession_raw_total[lane] += raw_residual
            _gnss_confession_corrected_total[lane] += corrected_residual
            n = _gnss_confession_n[lane]
            raw_ppb = _gnss_confession_raw_total[lane] / n
            corrected_ppb = _gnss_confession_corrected_total[lane] / n
            published_ppb = (
                corrected_ppb
                if _gnss_confession_published_source() == "GNSS_CONFESSION"
                else raw_ppb
            )
            lane_obj.update({
                "sample_count": n,
                "gnss_confession_corrected_residual_ns": corrected_residual,
                "raw_running_ppb": round(raw_ppb, 6),
                "gnss_confession_running_ppb": round(corrected_ppb, 6),
                "published_ppb": round(published_ppb, 6),
            })
        candidates[lane] = lane_obj

    _gnss_confession_prev_pps = public_count
    if g_err is not None:
        _gnss_confession_prev_error_ns = g_err

    _diag["gnss_confession_published_source"] = _gnss_confession_published_source()
    _diag["last_gnss_confession"] = candidates
    return candidates

def _gnss_raw_reset() -> None:
    """Reset GNSS_RAW accumulator (on campaign start/stop/recover)."""
    global _gnss_raw_ns, _gnss_raw_n, _gnss_raw_valid
    global _gnss_raw_welford_n, _gnss_raw_welford_mean, _gnss_raw_welford_m2
    global _gnss_raw_welford_min, _gnss_raw_welford_max
    _gnss_raw_ns = 0.0
    _gnss_raw_n = 0
    _gnss_raw_valid = False
    _gnss_raw_welford_n = 0
    _gnss_raw_welford_mean = 0.0
    _gnss_raw_welford_m2 = 0.0
    _gnss_raw_welford_min = 1e30
    _gnss_raw_welford_max = -1e30

# ---------------------------------------------------------------------
# Local state (process lifetime)
# ---------------------------------------------------------------------

_campaign_active: bool = False

# The command server is exposed early so PUBSUB can discover subscriptions, but
# START/RESUME must not race the boot DAC push and active-campaign recovery.
_startup_control_ready = threading.Event()

_last_pps_vclock_count_seen: Optional[int] = None

# Last PPS/VCLOCK count accepted into TIMEBASE processing.
# This is observational only; it is never used to reject a fragment.
_accepted_pps_vclock_count: Optional[int] = None

# TIMEBASE silence monitor.  This is intentionally process-local: if a
# campaign is active and the Teensy stops publishing TIMEBASE_FRAGMENT, CLOCKS
# treats the silence as a recoverable Teensy lifecycle event once communication
# returns.
_timebase_last_activity_monotonic: Optional[float] = None
_timebase_last_activity_utc: Optional[str] = None
_timebase_last_activity_topic: Optional[str] = None
_timebase_last_activity_pps_vclock_count: Optional[int] = None
_timebase_silence_recovery_active: bool = False

# Asynchronous START observation.  START no longer blocks waiting for the
# first TIMEBASE_FRAGMENT; this records the pending first-fragment wait so
# reports can show whether the Teensy has begun publishing.
_start_waiting_for_first_fragment: bool = False
_start_requested_campaign: Optional[str] = None
_start_requested_at_utc: Optional[str] = None
_start_requested_monotonic: Optional[float] = None
_start_first_fragment_at_utc: Optional[str] = None
_start_first_fragment_wait_s: Optional[float] = None
_start_first_fragment_pps_vclock_count: Optional[int] = None

# Flash Cut is a hot campaign namespace transition.  While this is pending,
# an active campaign with zero rows is expected, not a cold-recovery trigger.
_flash_cut_pending: bool = False
_flash_cut_from_campaign: Optional[str] = None
_flash_cut_to_campaign: Optional[str] = None
_flash_cut_requested_at_utc: Optional[str] = None
_flash_cut_requested_monotonic: Optional[float] = None

# Draconian sync: control-plane waits for a specific fragment PPS/VCLOCK count
_sync_lock = threading.Lock()
_sync_expected_pps_vclock: Optional[int] = None
_sync_event = threading.Event()
_sync_resume_event = threading.Event()
_sync_fragment: Optional[Dict[str, Any]] = None

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

_auto_recovery_in_progress: bool = False
_recovery_interruption_lock = threading.Lock()
_recovery_interruption_pending: bool = False
_recovery_interruption_details: Dict[str, Any] = {}


class RecoveryRetryableFailure(RuntimeError):
    """Base class for RECOVER failures that should be cleaned and retried."""

    def __init__(
        self,
        reason: str,
        details: Dict[str, Any],
        *,
        cleanup_sent: bool = False,
    ):
        super().__init__(f"{reason}: {details}")
        self.reason = reason
        self.details = details
        self.cleanup_sent = cleanup_sent


class RecoverySyncTimeout(RecoveryRetryableFailure):
    """Raised when a RECOVER lifecycle does not produce a public TIMEBASE row."""


class RecoveryCleanTimeout(RecoveryRetryableFailure):
    """Legacy name for a recovery that never reaches a timeline-admissible row."""


class RecoveryInterrupted(RecoveryRetryableFailure):
    """Raised when a new WATCHDOG_ANOMALY invalidates an active recovery attempt."""


class RecoveryCommandLost(RecoveryRetryableFailure):
    """Raised when Teensy loses the in-flight RECOVER command/lifecycle."""


def _note_recovery_interruption(reason: str, details: Dict[str, Any], *, source: str) -> None:
    """Mark the current recovery attempt invalidated by a new semantic fault."""
    global _recovery_interruption_pending, _recovery_interruption_details

    payload = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "reason": reason,
        "source": source,
        "details": details,
        "sync_expected_pps_vclock_count": _sync_expected_pps_vclock,
        "last_timebase_activity_utc": _timebase_last_activity_utc,
        "last_timebase_activity_topic": _timebase_last_activity_topic,
        "last_timebase_activity_pps_vclock_count": _timebase_last_activity_pps_vclock_count,
    }

    with _recovery_interruption_lock:
        _recovery_interruption_pending = True
        _recovery_interruption_details = payload

    _diag["recovery_interruption_requests"] = _diag.get("recovery_interruption_requests", 0) + 1
    _diag["last_recovery_interruption"] = payload
    logging.warning(
        "⚠️ [recovery] active recovery invalidated by %s: %s",
        source, reason,
    )


def _consume_recovery_interruption() -> Optional[Dict[str, Any]]:
    """Return and clear a pending active-recovery interruption, if any."""
    global _recovery_interruption_pending, _recovery_interruption_details

    with _recovery_interruption_lock:
        if not _recovery_interruption_pending:
            return None
        details = dict(_recovery_interruption_details)
        _recovery_interruption_pending = False
        _recovery_interruption_details = {}
        return details


def _raise_if_recovery_interrupted(context: str) -> None:
    interrupt = _consume_recovery_interruption()
    if interrupt is None:
        return

    details = {
        **interrupt,
        "context": context,
        "last_accepted_pps_vclock_count": _accepted_pps_vclock_count,
    }
    _clear_sync_wait()
    raise RecoveryInterrupted("recovery_interrupted_by_watchdog_anomaly", details)


def _note_timebase_activity(
    topic: str,
    pps_vclock_count: Optional[int],
) -> None:
    """
    Record receipt of one Teensy TIMEBASE candidate.

    Candidate receipt is the campaign heartbeat even when the Pi later rejects
    the row.  That keeps transport silence distinct from scientific invalidity.
    """
    global _timebase_last_activity_monotonic
    global _timebase_last_activity_utc
    global _timebase_last_activity_topic
    global _timebase_last_activity_pps_vclock_count

    count = None if pps_vclock_count is None else int(pps_vclock_count)
    now_utc = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    _timebase_last_activity_monotonic = time.monotonic()
    _timebase_last_activity_utc = now_utc
    _timebase_last_activity_topic = topic
    _timebase_last_activity_pps_vclock_count = count
    _diag["last_timebase_activity"] = {
        "ts_utc": now_utc,
        "topic": topic,
        "teensy_pps_vclock_count": count,
        "pps_count": count,
    }


def _arm_timebase_silence_watch(context: str) -> None:
    """
    Start or restart the silence timer when CLOCKS begins expecting TIMEBASE.

    This covers the interval before the first post-START/post-RECOVER fragment.
    Actual TIMEBASE publications replace this synthetic watch-arm marker through
    _note_timebase_activity().
    """
    global _timebase_last_activity_monotonic
    global _timebase_last_activity_utc
    global _timebase_last_activity_topic
    global _timebase_last_activity_pps_vclock_count

    now_utc = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    count = _accepted_pps_vclock_count
    _timebase_last_activity_monotonic = time.monotonic()
    _timebase_last_activity_utc = now_utc
    _timebase_last_activity_topic = f"{context}_WATCH_ARMED"
    _timebase_last_activity_pps_vclock_count = count
    _diag["last_timebase_activity"] = {
        "ts_utc": now_utc,
        "topic": _timebase_last_activity_topic,
        "teensy_pps_vclock_count": count,
        "pps_count": count,
        "synthetic_watch_arm": True,
    }




def _timebase_silence_timeout_for_current_state() -> float:
    """Return the silence timeout appropriate for the current campaign phase."""
    if _flash_cut_pending:
        return FLASH_CUT_FIRST_FRAGMENT_TIMEOUT_S
    if _start_waiting_for_first_fragment:
        return START_FIRST_FRAGMENT_TIMEOUT_S
    return TIMEBASE_SILENCE_TIMEOUT_S


def _mark_flash_cut_waiting(previous_campaign: str, campaign: str) -> None:
    """Record that the Pi has armed a hot campaign cut and is awaiting row #1."""
    global _flash_cut_pending, _flash_cut_from_campaign, _flash_cut_to_campaign
    global _flash_cut_requested_at_utc, _flash_cut_requested_monotonic

    now_utc = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")

    _flash_cut_pending = True
    _flash_cut_from_campaign = previous_campaign
    _flash_cut_to_campaign = campaign
    _flash_cut_requested_at_utc = now_utc
    _flash_cut_requested_monotonic = time.monotonic()

    _diag["flash_cut_requests"] = _diag.get("flash_cut_requests", 0) + 1
    _diag["flash_cut_waiting"] = True
    _diag["flash_cut_from"] = previous_campaign
    _diag["flash_cut_to"] = campaign
    _diag["flash_cut_requested_at_utc"] = now_utc
    _diag["flash_cut_first_fragment_at_utc"] = None
    _diag["flash_cut_first_fragment_wait_s"] = None
    _diag["flash_cut_first_fragment_pps_vclock_count"] = None
    _diag["last_flash_cut"] = {
        "from": previous_campaign,
        "to": campaign,
        "requested_at_utc": now_utc,
        "state": "WAITING_FOR_FIRST_FRAGMENT",
        "first_fragment_timeout_s": float(FLASH_CUT_FIRST_FRAGMENT_TIMEOUT_S),
    }


def _clear_flash_cut_wait_state() -> None:
    """Clear process-local Flash Cut wait state without erasing diagnostics."""
    global _flash_cut_pending, _flash_cut_from_campaign, _flash_cut_to_campaign
    global _flash_cut_requested_at_utc, _flash_cut_requested_monotonic

    _flash_cut_pending = False
    _flash_cut_from_campaign = None
    _flash_cut_to_campaign = None
    _flash_cut_requested_at_utc = None
    _flash_cut_requested_monotonic = None
    _diag["flash_cut_waiting"] = False
    _diag["flash_cut_from"] = None
    _diag["flash_cut_to"] = None


def _mark_flash_cut_committed_if_needed(*, campaign: str, pps_vclock_count: int) -> None:
    """Clear durable/local Flash Cut pending state once the first row is accepted."""
    global _flash_cut_pending
    global _flash_cut_from_campaign, _flash_cut_to_campaign
    global _flash_cut_requested_at_utc, _flash_cut_requested_monotonic

    now_utc = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    waited_s = (
        time.monotonic() - _flash_cut_requested_monotonic
        if _flash_cut_requested_monotonic is not None
        else None
    )

    try:
        with open_db() as conn:
            cur = conn.cursor()
            cur.execute(
                """
                UPDATE campaigns
                SET payload = (payload - 'flash_cut_pending')
                    || jsonb_build_object(
                        'flash_cut_committed_at', to_jsonb(%s::text),
                        'flash_cut_first_pps_vclock_count', to_jsonb(%s::int),
                        'flash_cut_first_pps_count', to_jsonb(%s::int)
                    )
                WHERE campaign = %s
                  AND active = true
                  AND payload ? 'flash_cut_pending'
                """,
                (now_utc, int(pps_vclock_count), int(pps_vclock_count), campaign),
            )
    except Exception:
        logging.exception("⚠️ [clocks] failed to clear flash_cut_pending marker (ignored)")

    if _flash_cut_pending and (_flash_cut_to_campaign is None or _flash_cut_to_campaign == campaign):
        _diag["flash_cut_waiting"] = False
        _diag["flash_cut_first_fragment_at_utc"] = now_utc
        _diag["flash_cut_first_fragment_wait_s"] = (
            None if waited_s is None else round(float(waited_s), 3)
        )
        _diag["flash_cut_first_fragment_pps_vclock_count"] = int(pps_vclock_count)
        _diag["last_flash_cut"] = {
            "from": _flash_cut_from_campaign,
            "to": campaign,
            "requested_at_utc": _flash_cut_requested_at_utc,
            "first_fragment_at_utc": now_utc,
            "waited_s": None if waited_s is None else round(float(waited_s), 3),
            "teensy_pps_vclock_count": int(pps_vclock_count),
            "state": "RUNNING",
        }
        _clear_flash_cut_wait_state()


def _campaign_payload_is_pending_flash_cut(payload: Dict[str, Any]) -> bool:
    return bool(payload.get("flash_cut_pending") and payload.get("flash_cut_from"))


def _flash_cut_pending_age_s(payload: Dict[str, Any]) -> Optional[float]:
    if _flash_cut_requested_monotonic is not None:
        return time.monotonic() - _flash_cut_requested_monotonic

    ts = payload.get("flash_cut_armed_at") or payload.get("started_at")
    if not isinstance(ts, str) or not ts:
        return None

    try:
        armed = datetime.fromisoformat(ts.replace("Z", "+00:00"))
        return (datetime.now(timezone.utc) - armed).total_seconds()
    except Exception:
        return None


def _reattach_pending_flash_cut_without_recovery(
    *,
    campaign_name: str,
    campaign_payload: Dict[str, Any],
) -> bool:
    """Do not cold-restart a zero-row campaign that is merely awaiting Flash Cut row #1."""
    global _campaign_active, _accepted_pps_vclock_count

    if not _campaign_payload_is_pending_flash_cut(campaign_payload):
        return False

    pending_age_s = _flash_cut_pending_age_s(campaign_payload)
    if pending_age_s is not None and pending_age_s >= FLASH_CUT_FIRST_FRAGMENT_TIMEOUT_S:
        logging.error(
            "💥 [recovery] pending Flash Cut campaign '%s' has no TIMEBASE rows after %.3fs "
            "(timeout=%.3fs); allowing cold recovery",
            campaign_name, float(pending_age_s), float(FLASH_CUT_FIRST_FRAGMENT_TIMEOUT_S),
        )
        return False

    previous_campaign = str(campaign_payload.get("flash_cut_from") or "")
    _diag["flash_cut_cold_recovery_deferred"] = _diag.get("flash_cut_cold_recovery_deferred", 0) + 1
    _diag["last_flash_cut"] = {
        "from": previous_campaign,
        "to": campaign_name,
        "state": "REATTACHED_ZERO_ROW_WAIT",
        "reattached_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "first_fragment_timeout_s": float(FLASH_CUT_FIRST_FRAGMENT_TIMEOUT_S),
        "pending_age_s": None if pending_age_s is None else round(float(pending_age_s), 3),
    }

    _reset_trackers()
    _clear_sync_wait()
    _accepted_pps_vclock_count = None
    _diag["accepted_pps_count"] = None
    _diag["accepted_pps_vclock_count"] = None
    _mark_start_waiting(campaign_name)
    _mark_flash_cut_waiting(previous_campaign, campaign_name)
    _campaign_active = True
    _arm_timebase_silence_watch("FLASH_CUT_REATTACH")

    logging.info(
        "⚡ [recovery] campaign '%s' has no TIMEBASE rows but is a pending Flash Cut child; "
        "reattaching to the hot Teensy stream instead of issuing STOP/START",
        campaign_name,
    )
    return True

def _teensy_clocks_health_ok() -> bool:
    """
    True when the Teensy CLOCKS command path responds to REPORT.

    Failed probes are deliberately silent in the log.  During a real Teensy
    reboot or USB/RawHID loss, this loop may run for minutes; the operator only
    needs the initial silence detection and the eventual recovery transition.
    """
    _diag["teensy_health_probe_attempts"] += 1
    try:
        resp = send_command(
            machine="TEENSY",
            subsystem="CLOCKS",
            command="REPORT",
            retries=1,
            retry_delay_s=0.0,
        )
        if resp.get("success"):
            _diag["teensy_health_probe_success"] += 1
            return True
    except Exception:
        pass

    _diag["teensy_health_probe_failures"] += 1
    return False


def _timebase_silence_recovery(reason: str, details: Dict[str, Any]) -> None:
    """
    Wait quietly for Teensy communication to return, then invoke campaign recovery.

    This extends boot-time campaign recovery to the live-Pi / rebooted-Teensy
    case.  The database active-campaign row remains the durable intent; the
    process-local campaign gate is lowered so stale or partial TIMEBASE candidates
    cannot be persisted while the Teensy is being recovered.
    """
    global _campaign_active, _auto_recovery_in_progress
    global _timebase_silence_recovery_active

    attempts = 0
    try:
        while True:
            if _teensy_clocks_health_ok():
                logging.info(
                    "✅ [clocks] @%s Teensy CLOCKS REPORT responded after TIMEBASE silence — "
                    "invoking campaign recovery",
                    system_time_z(),
                )
                attempts += 1
                _diag["auto_recovery_attempts"] = _diag.get("auto_recovery_attempts", 0) + 1
                _diag["timebase_silence_recovery_started"] += 1
                try:
                    _recover_campaign()
                    logging.info(
                        "✅ [clocks] @%s TIMEBASE silence recovery complete",
                        system_time_z(),
                    )
                    return
                except RecoveryRetryableFailure as e:
                    _diag["auto_recovery_failures"] = _diag.get("auto_recovery_failures", 0) + 1
                    if not getattr(e, "cleanup_sent", False):
                        _cleanup_after_recovery_failure(e.reason, e.details)

                    status = str((e.details or {}).get("status") or "")
                    terminal_firmware_rejection = (
                        status == "recover_rejected_interrupt_service_rearm"
                    )
                    attempts_exhausted = attempts >= int(AUTO_RECOVERY_MAX_ATTEMPTS)

                    if terminal_firmware_rejection or attempts_exhausted:
                        stop_reason = (
                            "terminal_firmware_rejection"
                            if terminal_firmware_rejection
                            else "attempt_limit"
                        )
                        _diag["auto_recovery_exhausted"] = (
                            _diag.get("auto_recovery_exhausted", 0) + 1
                        )
                        _diag["last_auto_recovery_exhausted"] = {
                            "ts_utc": datetime.now(timezone.utc)
                                .isoformat()
                                .replace("+00:00", "Z"),
                            "attempts": int(attempts),
                            "max_attempts": int(AUTO_RECOVERY_MAX_ATTEMPTS),
                            "stop_reason": stop_reason,
                            "failure_reason": e.reason,
                            "status": status or None,
                            "details": e.details,
                        }
                        logging.error(
                            "💥 [clocks] TIMEBASE silence recovery stopped after %d attempt(s): "
                            "reason=%s status=%s. The Teensy is commandable but cannot "
                            "re-enter RECOVER from its present interrupt-service state; "
                            "the durable campaign remains available for recovery after "
                            "firmware repair/reboot.",
                            attempts,
                            e.reason,
                            status or "unknown",
                            exc_info=True,
                        )
                        return

                    _diag["auto_recovery_retries"] = (
                        _diag.get("auto_recovery_retries", 0) + 1
                    )
                    logging.warning(
                        "⚠️ [clocks] TIMEBASE silence recovery retryable failure "
                        "(attempt %d/%d): %s details=%s — retrying after %.1fs",
                        attempts,
                        int(AUTO_RECOVERY_MAX_ATTEMPTS),
                        e.reason,
                        e.details,
                        float(AUTO_RECOVERY_RETRY_DELAY_S),
                        exc_info=True,
                    )
                    time.sleep(float(AUTO_RECOVERY_RETRY_DELAY_S))
                    continue
                except Exception:
                    _diag["auto_recovery_failures"] = _diag.get("auto_recovery_failures", 0) + 1
                    logging.exception(
                        "💥 [clocks] TIMEBASE silence recovery failed — "
                        "will retry after %.0fs",
                        TEENSY_HEALTH_RETRY_S,
                    )
                    time.sleep(TEENSY_HEALTH_RETRY_S)
                    continue

            time.sleep(TEENSY_HEALTH_RETRY_S)
    finally:
        _timebase_silence_recovery_active = False
        _auto_recovery_in_progress = False


def _begin_timebase_silence_recovery(age_s: float, *, timeout_s: float, phase: str) -> bool:
    """
    Start the live-Teensy-reboot recovery path if no other recovery is active.
    """
    global _campaign_active, _auto_recovery_in_progress
    global _timebase_silence_recovery_active

    if _auto_recovery_in_progress or _timebase_silence_recovery_active:
        return False

    now_utc = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    details = {
        "age_s": round(float(age_s), 3),
        "timeout_s": float(timeout_s),
        "campaign_phase": phase,
        "flash_cut_pending": bool(_flash_cut_pending),
        "start_waiting_for_first_fragment": bool(_start_waiting_for_first_fragment),
        "last_timebase_activity_utc": _timebase_last_activity_utc,
        "last_timebase_activity_topic": _timebase_last_activity_topic,
        "last_timebase_activity_pps_vclock_count": _timebase_last_activity_pps_vclock_count,
        "last_accepted_pps_vclock_count": _accepted_pps_vclock_count,
    }

    _diag["hard_faults_total"] = _diag.get("hard_faults_total", 0) + 1
    _diag["timebase_silence_detected"] += 1
    _diag["last_hard_fault"] = {
        "ts_utc": now_utc,
        "reason": "timebase_silence",
        "source": "TIMEBASE_SILENCE_MONITOR",
        "details": details,
    }
    _diag["last_timebase_silence"] = {
        "ts_utc": now_utc,
        **details,
    }

    logging.error(
        "💥 [clocks] TIMEBASE silence detected during active campaign "
        "(age=%.3fs timeout=%.3fs, last_topic=%s, last_pps_vclock_count=%s) — "
        "waiting for Teensy health check before recovery",
        float(age_s),
        float(timeout_s),
        _timebase_last_activity_topic,
        str(_timebase_last_activity_pps_vclock_count),
    )

    _campaign_active = False
    _auto_recovery_in_progress = True
    _timebase_silence_recovery_active = True

    t = threading.Thread(
        target=_timebase_silence_recovery,
        args=("timebase_silence", details),
        name="clocks-timebase-silence-recover",
        daemon=True,
    )
    t.start()
    return True


def _timebase_silence_monitor_loop() -> None:
    """
    Detect a live Teensy reboot/loss by absence of TIMEBASE publications.

    The monitor is intentionally quiet unless it detects the first silence
    threshold crossing.  Health-check failures after that point are silent and
    retried once per TEENSY_HEALTH_RETRY_S.
    """
    _diag["timebase_silence_monitor_started"] = True
    logging.info(
        "🚀 [clocks] TIMEBASE silence monitor started "
        "(timeout=%.1fs, health_retry=%.1fs)",
        TIMEBASE_SILENCE_TIMEOUT_S,
        TEENSY_HEALTH_RETRY_S,
    )

    while True:
        time.sleep(TIMEBASE_SILENCE_MONITOR_POLL_S)
        _diag["timebase_silence_checks"] += 1

        if not _campaign_active:
            continue
        if _auto_recovery_in_progress or _timebase_silence_recovery_active:
            continue
        if _sync_expected_pps_vclock is not None:
            continue
        if _timebase_last_activity_monotonic is None:
            # Avoid treating initial async START/cold boot wait as a reboot.
            # This monitor is for loss after the TIMEBASE stream has once been observed.
            continue

        age_s = time.monotonic() - _timebase_last_activity_monotonic
        timeout_s = _timebase_silence_timeout_for_current_state()
        if _flash_cut_pending:
            phase = "FLASH_CUT_ARMED"
        elif _start_waiting_for_first_fragment:
            phase = "START_WAITING_FOR_FIRST_FRAGMENT"
        else:
            phase = "RUNNING"

        if age_s >= timeout_s:
            _begin_timebase_silence_recovery(age_s, timeout_s=timeout_s, phase=phase)


def _begin_auto_recovery(reason: str, details: Dict[str, Any], *, source: str) -> bool:
    """
    Common auto-recovery launcher used by hard faults and WATCHDOG_ANOMALY.

    Returns True if a new recovery thread was started, False if one was
    already in progress.
    """
    global _campaign_active, _auto_recovery_in_progress

    _diag["hard_faults_total"] = _diag.get("hard_faults_total", 0) + 1
    _diag["last_hard_fault"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "reason": reason,
        "source": source,
        "details": details,
    }

    _campaign_active = False

    if _auto_recovery_in_progress:
        _note_recovery_interruption(reason, details, source=source)
        logging.error(
            "💥 [clocks] %s: %s — auto-recovery already in progress; current attempt will abort/retry",
            source, reason,
        )
        return False

    logging.error(
        "💥 [clocks] %s: %s details=%s — initiating auto-recovery",
        source, reason, details,
    )

    _auto_recovery_in_progress = True

    def _auto_recover():
        global _auto_recovery_in_progress
        try:
            for attempt in range(1, int(AUTO_RECOVERY_MAX_ATTEMPTS) + 1):
                _diag["auto_recovery_attempts"] = _diag.get("auto_recovery_attempts", 0) + 1
                try:
                    logging.info(
                        "🔄 [clocks] @%s auto-recovery starting (attempt %d/%d)...",
                        system_time_z(), attempt, int(AUTO_RECOVERY_MAX_ATTEMPTS),
                    )
                    _recover_campaign()
                    logging.info("✅ [clocks] @%s auto-recovery complete", system_time_z())
                    return
                except RecoveryRetryableFailure as e:
                    if isinstance(e, RecoveryInterrupted):
                        _diag["auto_recovery_interrupted"] = _diag.get("auto_recovery_interrupted", 0) + 1
                    logging.warning(
                        "⚠️ [clocks] auto-recovery attempt %d/%d retryable failure: %s details=%s",
                        attempt, int(AUTO_RECOVERY_MAX_ATTEMPTS), e.reason, e.details,
                    )
                    if not getattr(e, "cleanup_sent", False):
                        _cleanup_after_recovery_failure(e.reason, e.details)
                    if attempt >= int(AUTO_RECOVERY_MAX_ATTEMPTS):
                        raise RuntimeError(
                            f"auto-recovery retryable failure after {attempt} attempt(s): {e.reason}"
                        ) from e
                    _diag["auto_recovery_retries"] = _diag.get("auto_recovery_retries", 0) + 1
                    time.sleep(float(AUTO_RECOVERY_RETRY_DELAY_S))
                    continue
        except Exception as e:
            logging.exception("💥 [clocks] auto-recovery FAILED — campaign deactivated")
            _diag["auto_recovery_failures"] = _diag.get("auto_recovery_failures", 0) + 1
            _cleanup_after_recovery_failure(
                "auto_recovery_failed",
                {"error": str(e), "source": source, "reason": reason},
            )
        finally:
            _auto_recovery_in_progress = False

    t = threading.Thread(target=_auto_recover, name="clocks-auto-recover", daemon=True)
    t.start()
    return True

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


def _note_pps_vclock_count(teensy_pps_vclock_count: int) -> None:
    """Observational continuity check on the PPS/VCLOCK count stream from Teensy."""
    global _last_pps_vclock_count_seen

    k = int(teensy_pps_vclock_count)
    _diag["pps_count_seen"] += 1
    _diag["pps_vclock_count_seen"] += 1
    _diag["last_pps_count"] = k
    _diag["last_pps_vclock_count"] = k

    if _last_pps_vclock_count_seen is not None:
        anomaly: Optional[Dict[str, Any]] = None
        if k == _last_pps_vclock_count_seen:
            _diag["pps_count_repeat"] += 1
            anomaly = {
                "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "reason": "pps_count_repeat",
                "pps_vclock_count": k,
                "pps_count": k,
                "prev_pps_vclock_count": int(_last_pps_vclock_count_seen),
                "prev_pps_count": int(_last_pps_vclock_count_seen),
            }
        elif k < _last_pps_vclock_count_seen:
            _diag["pps_count_regress"] += 1
            anomaly = {
                "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "reason": "pps_count_regress",
                "pps_vclock_count": k,
                "pps_count": k,
                "prev_pps_vclock_count": int(_last_pps_vclock_count_seen),
                "prev_pps_count": int(_last_pps_vclock_count_seen),
            }
        elif k > (_last_pps_vclock_count_seen + 1):
            _diag["pps_count_jump"] += 1
            anomaly = {
                "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "reason": "pps_count_jump",
                "pps_vclock_count": k,
                "pps_count": k,
                "prev_pps_vclock_count": int(_last_pps_vclock_count_seen),
                "prev_pps_count": int(_last_pps_vclock_count_seen),
                "jump": int(k - _last_pps_vclock_count_seen),
            }

        if anomaly is not None:
            _diag["last_pps_vclock_count_anomaly"] = anomaly
            _diag["last_pps_count_anomaly"] = anomaly

    _last_pps_vclock_count_seen = k


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


def _persist_timebase(tb: Dict[str, Any]) -> None:
    """Append TIMEBASE row and denormalize as active campaign report."""
    try:
        report = dict(tb)
        report["campaign_state"] = "STARTED"

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


def _wait_for_timebase_routes(
    *,
    context: str,
    timeout_s: float = 30.0,
    poll_s: float = 0.5,
) -> None:
    """Confirm the unified TIMEBASE candidate route."""
    required = {TIMEBASE_FRAGMENT_TOPIC}
    t0 = time.monotonic()
    last_log = t0
    last_missing = sorted(required)
    logged_wait = False

    while True:
        now = time.monotonic()
        elapsed = now - t0
        try:
            resp = send_command(machine="TEENSY", subsystem="PUBSUB", command="REPORT")
            if resp.get("success"):
                routes = resp.get("payload", {}).get("routes", [])
                routed = {str(r.get("topic")) for r in routes if r.get("topic")}
                last_missing = sorted(required - routed)
                if not last_missing:
                    if logged_wait:
                        logging.info(
                            "✅ [%s] TIMEBASE routes ready after %.1fs",
                            context, elapsed,
                        )
                    return
        except RuntimeError:
            raise
        except Exception:
            pass

        if elapsed >= timeout_s:
            raise RuntimeError(
                f"{context} failed: missing PUBSUB route(s) {last_missing} after {timeout_s}s"
            )

        if elapsed >= 5.0 and (not logged_wait or now - last_log >= 30.0):
            logging.info(
                "⏳ [%s] waiting for TIMEBASE routes (%.0fs): %s",
                context, elapsed, ", ".join(last_missing) or "route report unavailable",
            )
            logged_wait = True
            last_log = now
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
    """Prepare to wait for the first Pi-accepted TIMEBASE candidate."""
    global _sync_expected_pps_vclock, _sync_fragment

    with _sync_lock:
        _sync_expected_pps_vclock = int(expected_pps)
        _sync_fragment = None
        _sync_event.clear()
        _sync_resume_event.clear()


def _clear_sync_wait() -> None:
    """Disarm any synchronous START/RECOVER wait latch."""
    global _sync_expected_pps_vclock, _sync_fragment

    with _sync_lock:
        _sync_expected_pps_vclock = None
        _sync_fragment = None
        _sync_event.clear()
        _sync_resume_event.set()


def _end_sync_wait(
    timeout_s: float = SYNC_FRAGMENT_TIMEOUT_S,
    *,
    recovery_monitor: Optional[Dict[str, Any]] = None,
) -> Tuple[Dict[str, Any], float]:
    """Block until the sync fragment arrives or timeout. Hard fault on timeout."""
    global _sync_expected_pps_vclock

    logging.info(
        "⏳ [recovery] waiting for first accepted TIMEBASE candidate >= %s",
        str(_sync_expected_pps_vclock),
    )

    t0 = time.monotonic()
    last_log = t0
    last_health_poll = t0
    _diag["sync_waits"] += 1

    while True:
        _raise_if_recovery_interrupted("sync_wait")

        now = time.monotonic()
        if recovery_monitor:
            sent_at = float(recovery_monitor.get("sent_monotonic") or t0)
            if (
                now - sent_at >= float(RECOVERY_SYNC_HEALTH_GRACE_S)
                and now - last_health_poll >= float(RECOVERY_SYNC_HEALTH_POLL_S)
            ):
                last_health_poll = now
                _check_recovery_inflight_monitor(recovery_monitor, context="sync_wait")

        remaining = timeout_s - (now - t0)
        if remaining <= 0:
            _diag["hard_fault_sync_timeout"] += 1
            details = {
                "expected_pps_vclock_count": _sync_expected_pps_vclock,
                "timeout_s": float(timeout_s),
                "waited_s": round(float(now - t0), 3),
                "last_timebase_activity_utc": _timebase_last_activity_utc,
                "last_timebase_activity_topic": _timebase_last_activity_topic,
                "last_timebase_activity_pps_vclock_count": _timebase_last_activity_pps_vclock_count,
                "last_accepted_pps_vclock_count": _accepted_pps_vclock_count,
            }
            _diag["last_sync_wait"] = {
                "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "reason": "sync_timeout_waiting_for_fragment",
                **details,
            }
            _clear_sync_wait()
            raise RecoverySyncTimeout("sync_timeout_waiting_for_fragment", details)

        if now - last_log >= SYNC_LOG_INTERVAL_S:
            logging.info(
                "⏳ [recovery] still waiting for first public row >= %s (%.1fs/%.0fs)",
                str(_sync_expected_pps_vclock),
                now - t0,
                timeout_s,
            )
            last_log = now

        if _sync_event.wait(timeout=min(remaining, SYNC_POLL_S)):
            break

    waited = time.monotonic() - t0
    _diag["sync_wait_success"] += 1
    _diag["sync_wait_seconds_total"] += float(waited)
    _diag["sync_wait_seconds_last"] = float(waited)
    _diag["last_sync_wait"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "expected_pps_vclock_count": int(_sync_expected_pps_vclock or -1),
        "waited_s": round(float(waited), 3),
        "timeout_s": float(timeout_s),
    }

    with _sync_lock:
        frag = dict(_sync_fragment or {})
        _sync_expected_pps_vclock = None
        return frag, float(waited)


def _signal_sync_candidate_if_needed(fragment: Dict[str, Any], pps_vclock_count: int) -> bool:
    """Satisfy RECOVER sync only after the Pi court accepts a candidate."""
    global _sync_fragment

    with _sync_lock:
        if _sync_expected_pps_vclock is None:
            return False

        match = int(pps_vclock_count) >= int(_sync_expected_pps_vclock)
        if match:
            logging.info(
                "✅ [recovery] first public TIMEBASE row observed: count=%d expected>=%d",
                int(pps_vclock_count), int(_sync_expected_pps_vclock),
            )
            _sync_fragment = dict(fragment)
            _sync_event.set()
            return True
        return False


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


WELFORD_RECOVERY_LANES = (
    ("dwt",         "dwt_welford",         ("stats.dwt.welford",)),
    ("vclock",      "vclock_welford",      ("stats.vclock.welford",)),
    ("ocxo1",       "ocxo1_welford",       ("stats.ocxo1.welford",)),
    ("ocxo2",       "ocxo2_welford",       ("stats.ocxo2.welford",)),
    ("pps_witness", "pps_witness_welford", ("stats.pps_witness.welford",)),
    ("ocxo1_dac",   "ocxo1_dac_welford",   ("stats.dac.ocxo1", "stats.ocxo1_dac.welford")),
    ("ocxo2_dac",   "ocxo2_dac_welford",   ("stats.dac.ocxo2", "stats.ocxo2_dac.welford")),
)


def _recover_welford_value(
    *,
    last_tb: Dict[str, Any],
    last_frag: Dict[str, Any],
    flat_prefix: str,
    nested_paths: Tuple[str, ...],
    field: str,
) -> Any:
    """Find one saved Teensy Welford field in the latest TIMEBASE row."""
    for path in nested_paths:
        obj = _path_get(last_frag, path)
        if isinstance(obj, dict) and obj.get(field) is not None:
            return obj.get(field)

    flat_key = f"{flat_prefix}_{field}"
    for source in (last_frag, last_tb):
        if isinstance(source, dict) and source.get(flat_key) is not None:
            return source.get(flat_key)

    return None


def _add_welford_recovery_args(
    teensy_args: Dict[str, Any],
    *,
    last_tb: Dict[str, Any],
    last_frag: Dict[str, Any],
) -> int:
    """Append recoverable Teensy Welford accumulators to CLOCKS.RECOVER args."""
    restored = 0

    for _lane, flat_prefix, nested_paths in WELFORD_RECOVERY_LANES:
        n_raw = _recover_welford_value(
            last_tb=last_tb,
            last_frag=last_frag,
            flat_prefix=flat_prefix,
            nested_paths=nested_paths,
            field="n",
        )
        n = _as_int(n_raw)
        if n is None:
            continue

        teensy_args[f"{flat_prefix}_n"] = str(int(n))
        for field in ("mean", "stddev", "stderr", "min", "max"):
            value = _recover_welford_value(
                last_tb=last_tb,
                last_frag=last_frag,
                flat_prefix=flat_prefix,
                nested_paths=nested_paths,
                field=field,
            )
            if value is not None:
                teensy_args[f"{flat_prefix}_{field}"] = str(value)
        restored += 1

    return restored


def _gnss_raw_recovery_project_seed(
    *,
    last_tb: Dict[str, Any],
    last_pps_vclock_count: int,
    seed_pps_vclock_count: int,
) -> Dict[str, Any]:
    """Project Pi-owned GNSS_RAW across a recovery gap without ratio poison.

    GNSS_RAW is not a Teensy clock.  It is a Pi synthetic clock accumulated as
    one nominal GNSS second plus the GF-8802 drift_ppb confession.  Therefore a
    warm recovery should project it from its own reference surface
    (gnss_raw_ref_ns) and restored drift mean, not from the campaign GNSS ratio.
    """
    extra = _report_extra_clocks(last_tb)

    last_ref_ns = _as_int(extra.get("gnss_raw_ref_ns") or last_tb.get("gnss_raw_ref_ns"))
    if last_ref_ns is None or last_ref_ns <= 0:
        last_ref_ns = int(last_pps_vclock_count) * NS_PER_SECOND

    last_raw_ns_f = _first_float(
        extra.get("gnss_raw_ns"),
        last_tb.get("gnss_raw_ns"),
    )
    mean_ppb = _first_float(
        extra.get("gnss_raw_welford_mean"),
        last_tb.get("gnss_raw_welford_mean"),
        extra.get("gnss_raw_drift_ppb"),
        last_tb.get("gnss_raw_drift_ppb"),
        0.0,
    ) or 0.0

    raw_source = "last_timebase_gnss_raw_ns"
    rebuilt = False
    last_raw_ppb = None

    if last_raw_ns_f is None or last_raw_ns_f <= 0.0:
        last_raw_ns_f = float(last_ref_ns) + mean_ppb * (float(last_ref_ns) / float(NS_PER_SECOND))
        raw_source = "rebuilt_missing_raw_from_welford_mean"
        rebuilt = True
    else:
        last_raw_ppb = _compute_ppb(int(round(last_raw_ns_f)), int(last_ref_ns)) if last_ref_ns > 0 else None
        if (
            last_raw_ppb is not None
            and math.isfinite(last_raw_ppb)
            and abs(float(last_raw_ppb) - float(mean_ppb)) > float(GNSS_RAW_RECOVERY_REBUILD_GATE_PPB)
        ):
            last_raw_ns_f = float(last_ref_ns) + mean_ppb * (float(last_ref_ns) / float(NS_PER_SECOND))
            raw_source = "rebuilt_incoherent_ratio_from_welford_mean"
            rebuilt = True

    gap_seconds = max(0, int(seed_pps_vclock_count) - int(last_pps_vclock_count))
    seed_ref_ns = int(last_ref_ns) + int(gap_seconds) * NS_PER_SECOND
    seed_raw_ns = int(round(last_raw_ns_f + float(gap_seconds) * (float(NS_PER_SECOND) + float(mean_ppb))))

    projected_ppb = _compute_ppb(seed_raw_ns, seed_ref_ns) if seed_ref_ns > 0 else 0.0
    return {
        "schema": "GNSS_RAW_RECOVERY_PROJECTION_V2",
        "source": raw_source,
        "rebuilt_from_welford_mean": bool(rebuilt),
        "rebuild_gate_ppb": float(GNSS_RAW_RECOVERY_REBUILD_GATE_PPB),
        "last_pps_vclock_count": int(last_pps_vclock_count),
        "seed_pps_vclock_count": int(seed_pps_vclock_count),
        "gap_seconds": int(gap_seconds),
        "last_ref_ns": int(last_ref_ns),
        "last_raw_ns": int(round(last_raw_ns_f)),
        "last_raw_ppb": None if last_raw_ppb is None else round(float(last_raw_ppb), 6),
        "welford_mean_ppb": round(float(mean_ppb), 6),
        "seed_ref_ns": int(seed_ref_ns),
        "seed_raw_ns": int(seed_raw_ns),
        "seed_raw_ppb": round(float(projected_ppb), 6),
    }


def _restore_gnss_raw_from_last_timebase(
    *,
    last_tb: Dict[str, Any],
    projected_gnss_ns: int,
    projected_gnss_raw_ns: int,
    projected_gnss_raw_ref_ns: int,
    projection_details: Optional[Dict[str, Any]] = None,
) -> bool:
    """Restore the Pi-owned GNSS_RAW synthetic clock and Welford accumulator.

    GNSS_RAW is deliberately Pi-only and is not sent to Teensy RECOVER.
    Restore it to the identity immediately BEFORE the first row the processor
    will persist; the processor then folds the first public row in exactly once.
    The Welford accumulator is restored from the last persisted TIMEBASE
    extra_clocks block; missing downtime samples are not invented.
    """
    global _gnss_raw_ns, _gnss_raw_n, _gnss_raw_valid
    global _gnss_raw_welford_n, _gnss_raw_welford_mean, _gnss_raw_welford_m2
    global _gnss_raw_welford_min, _gnss_raw_welford_max

    extra = _report_extra_clocks(last_tb)

    raw_ns = _first_float(
        projected_gnss_raw_ns if projected_gnss_raw_ns > 0 else None,
        extra.get("gnss_raw_ns"),
        last_tb.get("gnss_raw_ns"),
    )
    if raw_ns is not None:
        _gnss_raw_ns = float(raw_ns)

    raw_ref_ns = _as_int(
        projected_gnss_raw_ref_ns
        if projected_gnss_raw_ref_ns > 0
        else None
    )
    if raw_ref_ns is None:
        raw_ref_ns = _as_int(extra.get("gnss_raw_ref_ns") or last_tb.get("gnss_raw_ref_ns"))
    if raw_ref_ns is None or raw_ref_ns <= 0:
        raw_ref_ns = int(projected_gnss_ns) if projected_gnss_ns > 0 else 0

    if raw_ref_ns > 0:
        _gnss_raw_n = int(raw_ref_ns // NS_PER_SECOND)

    _gnss_raw_valid = _gnss_raw_ns > 0.0 and _gnss_raw_n > 0

    n = _as_int(extra.get("gnss_raw_welford_n") or last_tb.get("gnss_raw_welford_n"))
    mean = _first_float(extra.get("gnss_raw_welford_mean"), last_tb.get("gnss_raw_welford_mean")) or 0.0
    stddev = _first_float(extra.get("gnss_raw_welford_stddev"), last_tb.get("gnss_raw_welford_stddev"))
    stderr_value = _first_float(extra.get("gnss_raw_welford_stderr"), last_tb.get("gnss_raw_welford_stderr"))
    min_val = _first_float(extra.get("gnss_raw_welford_min"), last_tb.get("gnss_raw_welford_min"))
    max_val = _first_float(extra.get("gnss_raw_welford_max"), last_tb.get("gnss_raw_welford_max"))

    welford_restored = n is not None
    if n is not None:
        if stddev is None:
            stddev = (stderr_value * math.sqrt(float(n))) if stderr_value is not None and n > 0 else 0.0

        _gnss_raw_welford_n = int(n)
        _gnss_raw_welford_mean = float(mean)
        _gnss_raw_welford_m2 = (float(stddev) * float(stddev) * float(n - 1)) if n >= 2 else 0.0
        _gnss_raw_welford_min = float(min_val) if min_val is not None else float(mean)
        _gnss_raw_welford_max = float(max_val) if max_val is not None else float(mean)

    restored_ppb = _compute_ppb(int(round(_gnss_raw_ns)), int(raw_ref_ns)) if raw_ref_ns > 0 else 0.0
    restore_diag = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "projected_gnss_ns": int(projected_gnss_ns),
        "projected_gnss_raw_ns": int(projected_gnss_raw_ns),
        "projected_gnss_raw_ref_ns": int(projected_gnss_raw_ref_ns),
        "restored_raw_ns": int(round(_gnss_raw_ns)),
        "restored_ref_ns": int(raw_ref_ns),
        "restored_n": int(_gnss_raw_n),
        "restored_ppb": round(float(restored_ppb), 6),
        "welford_restored": bool(welford_restored),
        "welford_n": int(_gnss_raw_welford_n),
        "welford_mean": round(float(_gnss_raw_welford_mean), 6),
        "projection": projection_details or {},
    }
    _diag["gnss_raw_recovery_restore_count"] = _diag.get("gnss_raw_recovery_restore_count", 0) + 1
    if projection_details and projection_details.get("rebuilt_from_welford_mean"):
        _diag["gnss_raw_recovery_rebuild_count"] = _diag.get("gnss_raw_recovery_rebuild_count", 0) + 1
    _diag["last_gnss_raw_recovery"] = restore_diag

    return bool(_gnss_raw_valid)


def _recovery_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    return str(value).strip().lower() in ("1", "true", "yes", "on", "nominal", "ready")


def _fetch_teensy_recovery_status() -> Dict[str, Any]:
    """Return CLOCKS.REPORT_RECOVERY payload, or {} if temporarily unavailable."""
    try:
        resp = send_command(
            machine="TEENSY",
            subsystem="CLOCKS",
            command="REPORT_RECOVERY",
            retries=1,
            retry_delay_s=0.0,
        )
        if resp.get("success"):
            payload = resp.get("payload", {})
            return payload if isinstance(payload, dict) else {}
    except Exception:
        logging.debug("⚠️ [recovery] REPORT_RECOVERY poll failed (ignored)")
    return {}


def _recovery_inflight_status_compact(status: Dict[str, Any]) -> Dict[str, Any]:
    """Extract the flat RECOVER identity/lifecycle fields used by Pi."""
    generation = _as_int(status.get("recovery_generation"))
    if generation is None:
        generation = _as_int(status.get("request_count"))

    return {
        "report": status.get("report"),
        "schema": status.get("schema"),
        "campaign_state": status.get("campaign_state"),
        "campaign": status.get("campaign"),
        "campaign_seconds": _as_int(status.get("campaign_seconds")),
        "recover_lifecycle_active": _recovery_bool(status.get("recover_lifecycle_active")),
        "recover_lifecycle_reason": status.get("recover_lifecycle_reason"),
        "recover_interrupt_service_rearm_ok": _recovery_bool(
            status.get("recover_interrupt_service_rearm_ok")
        ),
        "recover_interrupt_service_rearm_count": _as_int(
            status.get("recover_interrupt_service_rearm_count")
        ),
        "recover_interrupt_service_rearm_failure_count": _as_int(
            status.get("recover_interrupt_service_rearm_failure_count")
        ),
        "recover_reattach_active": _recovery_bool(status.get("recover_reattach_active")),
        "recover_reattach_degraded_active": _recovery_bool(status.get("recover_reattach_degraded_active")),
        "recover_reattach_reason": status.get("recover_reattach_reason"),
        "recover_timeline_ready": _recovery_bool(status.get("recover_timeline_ready")),
        "recover_clockface_ready": _recovery_bool(status.get("recover_clockface_ready")),
        "recover_science_ready": _recovery_bool(status.get("recover_science_ready")),
        "recover_reattach_stalled": bool(
            _fragment_recovery_bool(
                status,
                "recover_reattach_stalled",
                "degraded_publication_stalled",
            )
        ),
        "degraded_no_progress_row_count": _as_int(status.get("degraded_no_progress_row_count")),
        "degraded_last_progress_public_count": _as_int(status.get("degraded_last_progress_public_count")),
        "recovery_generation": generation,
        "request_count": _as_int(status.get("request_count")),
        "base_count": _as_int(status.get("base_count")),
        "expected_first_public_count": _as_int(status.get("expected_first_public_count")),
        "last_public_count": _as_int(status.get("last_public_count")),
        "candidate_count": _as_int(status.get("candidate_count")),
        "hidden_candidate_count": _as_int(status.get("hidden_candidate_count")),
        "timebase_last_stage": _as_int(status.get("timebase_last_stage")),
        "timebase_last_stage_name": status.get("timebase_last_stage_name"),
    }


def _recovery_inflight_lost_reason(
    *,
    monitor: Dict[str, Any],
    status: Dict[str, Any],
) -> Optional[str]:
    """Return a retryable reason if Teensy no longer owns our RECOVER."""
    expected_base = int(monitor.get("recover_base_pps_vclock_count") or -1)
    expected_first = int(monitor.get("expected_first_public_pps_vclock_count") or -1)
    base_count = _as_int(status.get("base_count"))
    expected_first_reported = _as_int(status.get("expected_first_public_count"))
    request_count = _as_int(status.get("request_count"))
    expected_generation = _as_int(monitor.get("recovery_generation"))
    reported_generation = _as_int(status.get("recovery_generation"))
    if reported_generation is None:
        reported_generation = request_count

    lifecycle_active = _recovery_bool(status.get("recover_lifecycle_active"))
    reattach_active = _recovery_bool(status.get("recover_reattach_active"))
    degraded_active = _recovery_bool(status.get("recover_reattach_degraded_active"))
    warmup_active = _recovery_bool(status.get("warmup_active"))
    campaign_state = str(status.get("campaign_state") or "").upper()

    if (
        expected_generation is not None
        and expected_generation > 0
        and reported_generation is not None
        and reported_generation > 0
        and reported_generation != expected_generation
    ):
        return "recover_generation_mismatch"

    if base_count == expected_base and expected_first_reported == expected_first:
        return None

    if (base_count is not None and base_count > 0) or (
        expected_first_reported is not None and expected_first_reported > 0
    ):
        return "recover_identity_mismatch_after_teensy_reset"

    # After a reboot, REPORT_RECOVERY is available again but all RECOVER identity
    # fields fall back to zero/empty and the lifecycle is idle.  That means the
    # Pi is waiting for a command the firmware can no longer complete.
    if (
        request_count in (None, 0)
        and base_count in (None, 0)
        and expected_first_reported in (None, 0)
        and not lifecycle_active
        and not reattach_active
        and not degraded_active
        and not warmup_active
        and campaign_state in ("", "STOPPED", "IDLE")
    ):
        return "recover_command_lost_after_teensy_reset"

    return None


def _check_recovery_inflight_monitor(
    monitor: Dict[str, Any],
    *,
    context: str,
) -> None:
    """Poll Teensy RECOVER identity during sync wait and fail fast if lost."""
    _diag["recovery_inflight_health_polls"] = (
        _diag.get("recovery_inflight_health_polls", 0) + 1
    )

    status = _fetch_teensy_recovery_status()
    if not status:
        _diag["recovery_inflight_health_empty"] = (
            _diag.get("recovery_inflight_health_empty", 0) + 1
        )
        return

    compact = _recovery_inflight_status_compact(status)
    snapshot = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "context": context,
        "monitor": dict(monitor),
        "status": compact,
        "last_timebase_activity_utc": _timebase_last_activity_utc,
        "last_timebase_activity_topic": _timebase_last_activity_topic,
        "last_timebase_activity_pps_vclock_count": _timebase_last_activity_pps_vclock_count,
        "last_accepted_pps_vclock_count": _accepted_pps_vclock_count,
    }
    _diag["last_recovery_inflight_health"] = snapshot

    lost_reason = _recovery_inflight_lost_reason(monitor=monitor, status=status)
    if not lost_reason:
        return

    details = {
        **snapshot,
        "reason": lost_reason,
        "expected_pps_vclock_count": _sync_expected_pps_vclock,
    }
    _diag["recovery_inflight_command_lost"] = (
        _diag.get("recovery_inflight_command_lost", 0) + 1
    )
    _diag["last_recovery_command_lost"] = details
    _clear_sync_wait()
    raise RecoveryCommandLost(lost_reason, details)


def _fragment_ocxo_science_clean(fragment: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
    lanes: Dict[str, Any] = {}
    clean = True
    for lane in ("ocxo1", "ocxo2"):
        science = _path_get(fragment, f"{lane}.science")
        if not isinstance(science, dict):
            science = {}

        lane_status = {
            "science_valid": _recovery_bool(science.get("valid")),
            "antecedents_complete": _recovery_bool(science.get("antecedents_complete")),
            "delta_raw_valid": _recovery_bool(science.get("delta_raw_valid")),
            "residual_source": science.get("residual_source"),
            "public_count": _as_int(science.get("public_count")),
            "clock_interval_ns": _as_int(science.get("clock_interval_ns")),
            "gnss_interval_ns": _as_int(science.get("gnss_interval_ns")),
            "fast_residual_ns": _as_int(science.get("fast_residual_ns")),
        }
        lane_clean = bool(
            lane_status["science_valid"]
            and lane_status["antecedents_complete"]
            and lane_status["clock_interval_ns"] not in (None, 0)
            and lane_status["gnss_interval_ns"] not in (None, 0)
        )
        lane_status["clean"] = lane_clean
        lanes[lane] = lane_status
        clean = clean and lane_clean
    return clean, lanes


def _fragment_recovery_bool(fragment: Dict[str, Any], *keys: str) -> Optional[bool]:
    """Return the first explicitly present Boolean recovery field."""
    for key in keys:
        value = _path_get(fragment, key)
        if value is not None:
            return _recovery_bool(value)
    return None


def _fragment_recovery_any_true(
    fragment: Dict[str, Any],
    *keys: str,
) -> Optional[bool]:
    """OR explicit recovery flags without letting an early false hide a later true."""
    found = False
    for key in keys:
        value = _path_get(fragment, key)
        if value is None:
            continue
        found = True
        if _recovery_bool(value):
            return True
    return False if found else None


def _fragment_recovery_timeline_ready(
    fragment: Dict[str, Any],
    pps_vclock_count: int,
) -> bool:
    explicit = _fragment_recovery_bool(
        fragment,
        "recover_timeline_ready",
        "timeline_valid",
    )
    if explicit is not None:
        return explicit

    gnss_ns = _fragment_int(fragment, "gnss_ns", "gnss.ns", default=0)
    dwt_cycles = _fragment_int(
        fragment,
        "dwt_cycles",
        "dwt.cycles",
        "dwt.total_cycles",
        default=0,
    )
    return int(pps_vclock_count) > 0 and gnss_ns > 0 and dwt_cycles > 0


def _recovery_admission_verdict(
    *,
    fragment: Dict[str, Any],
    status: Dict[str, Any],
    pps_vclock_count: int,
) -> Tuple[bool, Dict[str, Any]]:
    """Decide whether a recovered row is truthful enough to persist.

    Timeline admission is intentionally weaker than OCXO science admission.
    A degraded row is admissible only when the Teensy explicitly identifies the
    transition; mere absence of science is never silently reclassified.
    """
    science_clean, lanes = _fragment_ocxo_science_clean(fragment)
    report_available = bool(status)

    fragment_active = _fragment_recovery_bool(
        fragment,
        "recover_reattach_active",
    )
    active = (
        fragment_active
        if fragment_active is not None
        else _recovery_bool(status.get("recover_reattach_active"))
    )

    fragment_degraded = _fragment_recovery_any_true(
        fragment,
        "recover_degraded_active",
        "recover_degraded_science_hold",
    )
    degraded = (
        fragment_degraded
        if fragment_degraded is not None
        else _recovery_bool(status.get("recover_reattach_degraded_active"))
    )

    stalled = bool(
        _fragment_recovery_bool(fragment, "recover_reattach_stalled")
        or _recovery_bool(
            status.get("recover_reattach_stalled")
            or status.get("degraded_publication_stalled")
        )
    )
    watchdog_blocked = _recovery_bool(status.get("watchdog_publication_blocked"))
    watchdog_active = _recovery_bool(status.get("watchdog_anomaly_active"))
    campaign_state = str(
        fragment.get("campaign_state")
        or status.get("campaign_state")
        or ""
    ).upper()

    timeline_ready = _fragment_recovery_timeline_ready(
        fragment,
        pps_vclock_count,
    )
    clockface_ready_explicit = _fragment_recovery_bool(
        fragment,
        "recover_clockface_ready",
        "ocxo_clockface_valid",
    )
    science_ready_explicit = _fragment_recovery_bool(
        fragment,
        "recover_science_ready",
        "ocxo_science_valid",
    )
    clockface_ready = (
        clockface_ready_explicit
        if clockface_ready_explicit is not None
        else _recovery_bool(status.get("recover_clockface_ready"))
    )
    science_ready = (
        science_ready_explicit
        if science_ready_explicit is not None
        else _recovery_bool(
            status.get("recover_science_ready")
            or status.get("reattach_ready")
            or status.get("recover_clean_ready")
        )
    )

    quarantine_active = bool(
        _fragment_recovery_bool(
            fragment,
            "recover_science_quarantine_active",
        )
    )
    quarantine_remaining = _as_int(
        fragment.get("recover_science_quarantine_remaining")
    )
    if quarantine_remaining is None:
        quarantine_remaining = _as_int(status.get("science_quarantine_remaining"))
    if quarantine_remaining is None:
        quarantine_remaining = 0
    quarantine_active = quarantine_active or quarantine_remaining != 0

    transition_active = bool(
        _fragment_recovery_bool(fragment, "recover_transition_active")
    )
    explicit_degraded = bool(
        transition_active
        or degraded
        or quarantine_active
    )

    fragment_generation = _fragment_int(
        fragment,
        "recover_generation",
        default=0,
    )
    status_generation = _as_int(status.get("recovery_generation"))
    if status_generation is None:
        status_generation = _as_int(status.get("request_count"))

    blocking_reasons: List[str] = []
    state_reasons: List[str] = []

    # REPORT_RECOVERY is useful corroboration, but the published fragment is the
    # publication authority. A transient command/report miss must not bury an
    # otherwise self-describing canonical row.
    if not report_available:
        state_reasons.append("report_recovery_unavailable")
    if active:
        blocking_reasons.append("reattach_private_hold_active")
    if campaign_state != "STARTED":
        blocking_reasons.append("campaign_not_started")
    if watchdog_blocked or watchdog_active:
        blocking_reasons.append("watchdog_blocked")
    if not timeline_ready:
        blocking_reasons.append("timeline_not_ready")
    if explicit_degraded and fragment_generation <= 0:
        blocking_reasons.append("degraded_row_missing_recovery_generation")
    if (
        fragment_generation > 0
        and status_generation is not None
        and status_generation > 0
        and fragment_generation != status_generation
    ):
        blocking_reasons.append("recovery_generation_mismatch")

    if degraded:
        state_reasons.append("degraded_publication_active")
    if quarantine_active:
        state_reasons.append("science_quarantine")
    if not clockface_ready:
        state_reasons.append("ocxo_clockface_not_ready")
    if not science_ready:
        state_reasons.append("ocxo_science_not_ready")
    if not science_clean:
        state_reasons.append("ocxo_science_not_clean")
    if stalled:
        state_reasons.append("ocxo_science_reattach_stalled")

    science_complete = bool(
        clockface_ready
        and science_ready
        and science_clean
        and not quarantine_active
    )
    degraded_timeline_admissible = bool(
        RECOVERY_ADMIT_DEGRADED_TIMELINE
        and explicit_degraded
        and timeline_ready
    )
    admissible = bool(
        not blocking_reasons
        and (science_complete or degraded_timeline_admissible)
    )
    fully_clean = bool(
        admissible
        and science_complete
        and not explicit_degraded
        and not stalled
    )

    if fully_clean:
        admission_mode = "clean_science"
    elif admissible:
        admission_mode = "degraded_timeline"
    else:
        admission_mode = "blocked"

    verdict = {
        "admissible": admissible,
        "admission_mode": admission_mode,
        "fully_clean": fully_clean,
        "degraded_timeline_admissible": degraded_timeline_admissible,
        "blocking_reasons": blocking_reasons,
        "state_reasons": state_reasons,
        # Compatibility alias retained for existing report consumers.
        "reasons": blocking_reasons + state_reasons,
        "pps_vclock_count": int(pps_vclock_count),
        "report_available": report_available,
        "recover_reattach_active": bool(active),
        "recover_reattach_degraded_active": bool(degraded),
        "recover_transition_active": transition_active,
        "recover_timeline_ready": timeline_ready,
        "recover_clockface_ready": bool(clockface_ready),
        "recover_science_ready": bool(science_ready),
        "recover_reattach_stalled": stalled,
        "explicit_degraded": explicit_degraded,
        "watchdog_blocked": watchdog_blocked or watchdog_active,
        "science_quarantine_active": quarantine_active,
        "science_quarantine_remaining": int(quarantine_remaining),
        "science_clean": science_clean,
        "lanes": lanes,
        "report_reason": (
            fragment.get("recover_reattach_reason")
            or status.get("recover_reattach_reason")
        ),
        "stall_reason": (
            status.get("recover_reattach_stall_reason")
            or status.get("degraded_publication_stall_reason")
            or status.get("degraded_stall_reason")
        ),
        "degraded_window_row_count": _as_int(
            status.get("recover_reattach_degraded_window_row_count")
            or status.get("degraded_window_row_count")
        ),
        "degraded_no_progress_row_count": _as_int(
            status.get("degraded_no_progress_row_count")
            or fragment.get("recover_no_progress_rows")
        ),
        "degraded_last_progress_public_count": _as_int(
            status.get("degraded_last_progress_public_count")
            or fragment.get("recover_last_progress_public_count")
        ),
        "degraded_stall_threshold": _as_int(
            status.get("recover_reattach_degraded_stall_threshold")
            or status.get("degraded_stall_threshold")
        ),
        "hidden_candidate_count": _as_int(status.get("hidden_candidate_count")),
        "last_release_public_count": _as_int(status.get("last_release_public_count")),
        "fragment_recovery_generation": int(fragment_generation),
        "status_recovery_generation": status_generation,
        "recovery_generation": (
            int(fragment_generation)
            if fragment_generation > 0
            else status_generation
        ),
    }
    return admissible, verdict


# ---------------------------------------------------------------------
# TIMEBASE_FRAGMENT schema helpers — PPS/VCLOCK canonical surface
# ---------------------------------------------------------------------

PPS_VCLOCK_COUNT_KEY = "teensy_pps_vclock_count"


def _path_get(mapping: Dict[str, Any], path: str) -> Any:
    """Return mapping[path] where path may be dotted, or None if absent."""
    cur: Any = mapping
    for part in path.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return None
        cur = cur.get(part)
    return cur


def _float_path(mapping: Dict[str, Any], path: str) -> Optional[float]:
    return _first_float(_path_get(mapping, path))


def _as_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _first_present_int(mapping: Dict[str, Any], *keys: str) -> Optional[int]:
    for key in keys:
        if key in mapping and mapping.get(key) is not None:
            value = _as_int(mapping.get(key))
            if value is not None:
                return value
    return None


def _extract_teensy_pps_vclock_count(publication: Dict[str, Any], *, topic: str = TIMEBASE_FRAGMENT_TOPIC) -> int:
    """
    Return the mandatory canonical one-second identity from a Teensy TIMEBASE publication.

    No fallbacks.  No ledger-derived repair.  If this field is missing or
    invalid, the publication is not a valid TIMEBASE candidate.  The explicit
    Teensy-authored PPS/VCLOCK count is the only identity surface.
    """
    if PPS_VCLOCK_COUNT_KEY not in publication or publication.get(PPS_VCLOCK_COUNT_KEY) is None:
        raise ValueError(f"{topic} missing mandatory teensy_pps_vclock_count")
    try:
        return int(publication[PPS_VCLOCK_COUNT_KEY])
    except (TypeError, ValueError) as e:
        raise ValueError(
            f"{topic} invalid teensy_pps_vclock_count={publication.get(PPS_VCLOCK_COUNT_KEY)!r}"
        ) from e


def _normalize_fragment_count_aliases(fragment: Dict[str, Any], count: int) -> None:
    """Preserve canonical identity and compatibility aliases in a fragment copy."""
    fragment["teensy_pps_vclock_count"] = int(count)
    fragment.setdefault("teensy_pps_count", int(count))
    fragment.setdefault("pps_count", int(count))


def _extract_last_timebase_count(last_tb: Dict[str, Any], fragment: Dict[str, Any]) -> int:
    """Return canonical count from a persisted TIMEBASE row or its fragment."""
    if fragment:
        return _extract_teensy_pps_vclock_count(fragment)
    if PPS_VCLOCK_COUNT_KEY not in last_tb or last_tb.get(PPS_VCLOCK_COUNT_KEY) is None:
        raise ValueError("persisted TIMEBASE missing mandatory teensy_pps_vclock_count")
    try:
        return int(last_tb[PPS_VCLOCK_COUNT_KEY])
    except (TypeError, ValueError) as e:
        raise ValueError(
            f"persisted TIMEBASE invalid teensy_pps_vclock_count={last_tb.get(PPS_VCLOCK_COUNT_KEY)!r}"
        ) from e


def _first_present_int_path(mapping: Dict[str, Any], *keys: str) -> Optional[int]:
    for key in keys:
        value = _path_get(mapping, key) if "." in key else mapping.get(key)
        if value is not None:
            parsed = _as_int(value)
            if parsed is not None:
                return parsed
    return None


def _fragment_int(fragment: Dict[str, Any], *keys: str, default: int = 0) -> int:
    """Extract an integer field from a TIMEBASE_FRAGMENT, including V3 nested paths."""
    value = _first_present_int_path(fragment, *keys)
    return int(value) if value is not None else int(default)


def _fragment_ns(fragment: Dict[str, Any], *keys: str, default: int = 0) -> int:
    """Extract an integer nanosecond ledger from a TIMEBASE_FRAGMENT, including V3 nested paths."""
    return _fragment_int(fragment, *keys, default=default)


# ---------------------------------------------------------------------
# Final TIMEBASE courtroom — last-mile persistence gate
# ---------------------------------------------------------------------


def _timebase_final_court_bool(value: Any) -> bool:
    """Interpret a final JSON boolean-ish field without trusting its type."""
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "y", "on")
    return False


def _timebase_final_court_add_violation(
    violations: List[Dict[str, Any]],
    *,
    rule: str,
    lane: str,
    message: str,
    fields: Dict[str, Any],
) -> None:
    violation: Dict[str, Any] = {
        "rule": rule,
        "lane": lane,
        "message": message,
        "severity": "fatal",
    }
    violation.update(fields)
    violations.append(violation)


def _timebase_final_court_check_candidate_envelope(
    timebase: Dict[str, Any],
    violations: List[Dict[str, Any]],
) -> None:
    """Validate the unified Teensy candidate before examining clock science."""
    fragment = timebase.get("fragment")
    forensics = timebase.get("forensics")

    if not isinstance(fragment, dict):
        _timebase_final_court_add_violation(
            violations,
            rule="candidate_fragment_present",
            lane="candidate",
            message="unified TIMEBASE candidate has no fragment object",
            fields={"bad_field": "fragment", "bad_value": fragment},
        )
        return

    disposition = _candidate_disposition(fragment)
    if disposition not in TIMEBASE_CANDIDATE_DISPOSITIONS:
        _timebase_final_court_add_violation(
            violations,
            rule="candidate_disposition_known",
            lane="candidate",
            message="candidate disposition is not recognized",
            fields={
                "candidate_disposition": fragment.get("candidate_disposition"),
                "allowed": sorted(TIMEBASE_CANDIDATE_DISPOSITIONS),
            },
        )

    if not isinstance(forensics, dict) or not forensics:
        _timebase_final_court_add_violation(
            violations,
            rule="embedded_forensics_present",
            lane="candidate",
            message="unified TIMEBASE candidate has no embedded forensics object",
            fields={
                "bad_field": "forensics",
                "bad_value": forensics,
                "fragment_forensics_embedded": fragment.get("forensics_embedded"),
            },
        )

    outer_count = _first_present_int_path(
        timebase,
        "teensy_pps_vclock_count",
        "pps_count",
    )
    fragment_count = _first_present_int_path(
        fragment,
        "teensy_pps_vclock_count",
        "pps_count",
    )
    forensics_count = (
        _first_present_int_path(
            forensics,
            "teensy_pps_vclock_count",
            "pps_count",
        )
        if isinstance(forensics, dict)
        else None
    )
    if (
        outer_count is None
        or fragment_count is None
        or forensics_count is None
        or outer_count != fragment_count
        or outer_count != forensics_count
    ):
        _timebase_final_court_add_violation(
            violations,
            rule="candidate_identity_consistent",
            lane="candidate",
            message="candidate, fragment, and forensics PPS/VCLOCK identities must agree",
            fields={
                "outer_count": outer_count,
                "fragment_count": fragment_count,
                "forensics_count": forensics_count,
                "identity_error": timebase.get("identity_error"),
            },
        )

    campaign = str(timebase.get("campaign") or "")
    fragment_campaign = str(fragment.get("campaign") or "")
    forensics_campaign = (
        str(forensics.get("campaign") or "")
        if isinstance(forensics, dict)
        else ""
    )
    if (
        not campaign
        or (fragment_campaign and fragment_campaign != campaign)
        or (forensics_campaign and forensics_campaign != campaign)
    ):
        _timebase_final_court_add_violation(
            violations,
            rule="candidate_campaign_consistent",
            lane="candidate",
            message="candidate campaign identity does not match the active campaign",
            fields={
                "active_campaign": campaign,
                "fragment_campaign": fragment_campaign,
                "forensics_campaign": forensics_campaign,
            },
        )

def _timebase_final_court_check_delta_raw_interval(
    timebase: Dict[str, Any],
    violations: List[Dict[str, Any]],
) -> None:
    """
    Validate the final OCXO Delta Raw interval fields exactly as published.

    This deliberately fishes values out of the final TIMEBASE dictionary rather
    than checking pre-assembly locals.  If Payload construction, JSON shaping,
    candidate shaping, or later decoration corrupts the final structure, this gate sees
    the same object that would otherwise be written to PostgreSQL.
    """
    for lane in ("ocxo1", "ocxo2"):
        science_path = f"fragment.{lane}.science"
        science = _path_get(timebase, science_path)
        if not isinstance(science, dict):
            continue

        if not _timebase_final_court_bool(science.get("delta_raw_valid")):
            continue

        clock_interval = _as_int(science.get("delta_raw_clock_interval_cycles"))
        reference_interval = _as_int(science.get("delta_raw_reference_interval_cycles"))
        public_count = _as_int(science.get("public_count"))
        delta_reference_public_count = _as_int(
            science.get("delta_reference_public_count")
        )
        delta_publication_public_count = _as_int(
            science.get("delta_publication_public_count")
        )

        common_fields: Dict[str, Any] = {
            "path": science_path,
            "public_count": public_count,
            "delta_reference_public_count": delta_reference_public_count,
            "delta_publication_public_count": delta_publication_public_count,
            "clock_interval_cycles": clock_interval,
            "reference_interval_cycles": reference_interval,
            "gate_cycles": TIMEBASE_FINAL_COURT_DELTA_RAW_INTERVAL_GATE_CYCLES,
            "min_plausible_cycles": TIMEBASE_FINAL_COURT_DWT_INTERVAL_MIN_CYCLES,
            "max_plausible_cycles": TIMEBASE_FINAL_COURT_DWT_INTERVAL_MAX_CYCLES,
        }

        missing = [
            name
            for name, value in (
                ("delta_raw_clock_interval_cycles", clock_interval),
                ("delta_raw_reference_interval_cycles", reference_interval),
            )
            if value is None
        ]
        if missing:
            fields = dict(common_fields)
            fields["missing_fields"] = missing
            _timebase_final_court_add_violation(
                violations,
                rule="delta_raw_interval_reasonable",
                lane=lane,
                message="delta_raw_valid is true but the final TIMEBASE JSON is missing required interval fields",
                fields=fields,
            )
            continue

        assert clock_interval is not None
        assert reference_interval is not None

        for name, value in (
            ("delta_raw_clock_interval_cycles", clock_interval),
            ("delta_raw_reference_interval_cycles", reference_interval),
        ):
            if (
                value < TIMEBASE_FINAL_COURT_DWT_INTERVAL_MIN_CYCLES
                or value > TIMEBASE_FINAL_COURT_DWT_INTERVAL_MAX_CYCLES
            ):
                fields = dict(common_fields)
                fields["bad_field"] = name
                fields["bad_value"] = value
                _timebase_final_court_add_violation(
                    violations,
                    rule="delta_raw_interval_reasonable",
                    lane=lane,
                    message="valid Delta Raw interval is outside the broad one-second DWT plausibility band",
                    fields=fields,
                )

        interval_error = int(clock_interval) - int(reference_interval)
        if abs(interval_error) > TIMEBASE_FINAL_COURT_DELTA_RAW_INTERVAL_GATE_CYCLES:
            # A large OCXO-vs-VCLOCK raw difference is diagnostic evidence, not
            # final-court guilt.  The OCXO may truly be running off-frequency
            # while still producing a perfectly sane one-second DWT interval.
            # The final court's fatal job here is to reject impossible interval
            # magnitudes such as 0/1/sentinel values in the final JSON.
            _diag["timebase_final_court_delta_raw_offset_observed"] = (
                _diag.get("timebase_final_court_delta_raw_offset_observed", 0) + 1
            )
            offset_fields = dict(common_fields)
            offset_fields.update({
                "rule": "delta_raw_interval_offset_observed",
                "lane": lane,
                "clock_minus_reference_cycles": interval_error,
                "abs_clock_minus_reference_cycles": abs(interval_error),
                "fatal": False,
            })
            _diag["last_timebase_final_court_delta_raw_offset"] = offset_fields


def _timebase_final_court_recovery_degraded_context(
    timebase: Dict[str, Any],
) -> bool:
    """True only for an explicitly marked, timeline-valid recovery row."""
    fragment = _path_get(timebase, "fragment")
    if not isinstance(fragment, dict):
        return False

    timeline_ready = _fragment_recovery_bool(
        fragment,
        "recover_timeline_ready",
        "timeline_valid",
    )
    degraded = _fragment_recovery_any_true(
        fragment,
        "recover_degraded_science_hold",
        "recover_degraded_active",
        "recover_science_quarantine_active",
        "recover_transition_active",
    )
    recovery_generation = _fragment_int(
        fragment,
        "recover_generation",
        default=0,
    )
    campaign_state = str(fragment.get("campaign_state") or "").upper()
    return bool(
        timeline_ready
        and degraded
        and recovery_generation > 0
        and campaign_state == "STARTED"
    )


def _timebase_final_court_check_ocxo_lane_alive(
    timebase: Dict[str, Any],
    violations: List[Dict[str, Any]],
) -> None:
    """Block mature public rows whose OCXO lane is entirely zero/missing.

    This is deliberately a final-object check.  Firmware may hold private
    recovery candidates while OCXO custody is being reattached, but a public
    TIMEBASE row with ocxoN.ns == 0 and no nonzero science endpoints/intervals
    is not a valid campaign row.  It is recovered lane absence.
    """
    public_count = _first_present_int_path(
        timebase,
        "teensy_pps_vclock_count",
        "fragment.teensy_pps_vclock_count",
        "pps_count",
    )
    if public_count is None:
        return
    if int(public_count) <= TIMEBASE_FINAL_COURT_OCXO_ZERO_MATURE_PUBLIC_COUNT:
        return

    recovery_degraded = _timebase_final_court_recovery_degraded_context(timebase)

    for lane in ("ocxo1", "ocxo2"):
        science_path = f"fragment.{lane}.science"
        science = _path_get(timebase, science_path)
        if not isinstance(science, dict):
            science = {}

        lane_ns = _first_present_int_path(
            timebase,
            f"fragment.{lane}.ns",
            f"fragment.gnss.{lane}_ns",
            f"fragment.{lane}_ns",
            f"teensy_{lane}_ns",
        )
        clockface_raw = _path_get(timebase, f"fragment.{lane}.clockface_valid")
        clockface_valid: Optional[bool] = (
            None if clockface_raw is None else _timebase_final_court_bool(clockface_raw)
        )

        public_count_in_science = _as_int(science.get("public_count"))
        clock_raw_dwt = _as_int(science.get("clock_raw_dwt_at_edge"))
        clock_floorline_dwt = _as_int(science.get("clock_floorline_dwt_at_edge"))
        clock_published_dwt = _as_int(science.get("clock_published_dwt_at_edge"))
        clock_observed_interval = _as_int(science.get("clock_observed_interval_cycles"))
        clock_floorline_interval = _as_int(science.get("clock_floorline_interval_cycles"))
        clock_interval_ns = _as_int(science.get("clock_interval_ns"))
        gnss_interval_ns = _as_int(science.get("gnss_interval_ns"))
        delta_raw_clock_interval = _as_int(science.get("delta_raw_clock_interval_cycles"))
        delta_floorline_clock_interval = _as_int(science.get("delta_floorline_clock_interval_cycles"))
        total_sample_count = _as_int(science.get("total_sample_count"))
        traditional_total_sample_count = _as_int(science.get("traditional_total_sample_count"))

        science_flags = {
            "science_valid": _timebase_final_court_bool(science.get("valid")),
            "antecedents_complete": _timebase_final_court_bool(science.get("antecedents_complete")),
            "clock_floorline_valid": _timebase_final_court_bool(science.get("clock_floorline_valid")),
            "delta_raw_valid": _timebase_final_court_bool(science.get("delta_raw_valid")),
            "delta_floorline_valid": _timebase_final_court_bool(science.get("delta_floorline_valid")),
            "traditional_valid": _timebase_final_court_bool(science.get("traditional_valid")),
            "total_valid": _timebase_final_court_bool(science.get("total_valid")),
            "traditional_total_valid": _timebase_final_court_bool(science.get("traditional_total_valid")),
        }

        evidence_values = (
            clock_raw_dwt,
            clock_floorline_dwt,
            clock_published_dwt,
            clock_observed_interval,
            clock_floorline_interval,
            clock_interval_ns,
            # The GNSS reference interval does not prove that this OCXO
            # lane contributed any evidence.  Counting it here would let an
            # all-zero OCXO lane pass merely because the reference second
            # was present.
            delta_raw_clock_interval,
            delta_floorline_clock_interval,
            total_sample_count,
            traditional_total_sample_count,
        )
        has_nonzero_science_evidence = any(
            value is not None and int(value) != 0 for value in evidence_values
        ) or any(science_flags.values())

        lane_ns_zero = lane_ns is None or int(lane_ns) == 0

        if (
            not recovery_degraded
            and (
                not science_flags["science_valid"]
                or not science_flags["antecedents_complete"]
            )
        ):
            _timebase_final_court_add_violation(
                violations,
                rule="ocxo_science_valid",
                lane=lane,
                message="mature public TIMEBASE row contains an OCXO clockface without valid OCXO science custody",
                fields={
                    "path": f"fragment.{lane}",
                    "science_path": science_path,
                    "public_count": int(public_count),
                    "science_public_count": public_count_in_science,
                    "mature_after_public_count": TIMEBASE_FINAL_COURT_OCXO_ZERO_MATURE_PUBLIC_COUNT,
                    "lane_ns": lane_ns,
                    "lane_ns_zero": lane_ns_zero,
                    "clockface_valid": clockface_valid,
                    "recovery_degraded": recovery_degraded,
                    "clock_raw_dwt_at_edge": clock_raw_dwt,
                    "clock_floorline_dwt_at_edge": clock_floorline_dwt,
                    "clock_published_dwt_at_edge": clock_published_dwt,
                    "clock_observed_interval_cycles": clock_observed_interval,
                    "clock_floorline_interval_cycles": clock_floorline_interval,
                    "clock_interval_ns": clock_interval_ns,
                    "gnss_interval_ns": gnss_interval_ns,
                    "delta_raw_clock_interval_cycles": delta_raw_clock_interval,
                    "delta_floorline_clock_interval_cycles": delta_floorline_clock_interval,
                    "total_sample_count": total_sample_count,
                    "traditional_total_sample_count": traditional_total_sample_count,
                    **science_flags,
                    "bad_field": f"fragment.{lane}.science.valid",
                    "bad_value": science.get("valid"),
                },
            )

        allow_missing_clockface = recovery_degraded and clockface_valid is not True
        if (
            lane_ns_zero
            and not has_nonzero_science_evidence
            and not allow_missing_clockface
        ):
            _timebase_final_court_add_violation(
                violations,
                rule="ocxo_lane_alive",
                lane=lane,
                message="mature public TIMEBASE row contains an all-zero OCXO lane after recovery/start maturity",
                fields={
                    "path": f"fragment.{lane}",
                    "science_path": science_path,
                    "public_count": int(public_count),
                    "science_public_count": public_count_in_science,
                    "mature_after_public_count": TIMEBASE_FINAL_COURT_OCXO_ZERO_MATURE_PUBLIC_COUNT,
                    "lane_ns": lane_ns,
                    "clockface_valid": clockface_valid,
                    "recovery_degraded": recovery_degraded,
                    "allow_missing_clockface": allow_missing_clockface,
                    "clock_raw_dwt_at_edge": clock_raw_dwt,
                    "clock_floorline_dwt_at_edge": clock_floorline_dwt,
                    "clock_published_dwt_at_edge": clock_published_dwt,
                    "clock_observed_interval_cycles": clock_observed_interval,
                    "clock_floorline_interval_cycles": clock_floorline_interval,
                    "clock_interval_ns": clock_interval_ns,
                    "gnss_interval_ns": gnss_interval_ns,
                    "delta_raw_clock_interval_cycles": delta_raw_clock_interval,
                    "delta_floorline_clock_interval_cycles": delta_floorline_clock_interval,
                    "total_sample_count": total_sample_count,
                    "traditional_total_sample_count": traditional_total_sample_count,
                    **science_flags,
                    "bad_field": f"fragment.{lane}.ns",
                    "bad_value": lane_ns,
                },
            )


def _timebase_final_court_evaluate(timebase: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
    """Return (ok, verdict) for the final TIMEBASE dictionary."""
    violations: List[Dict[str, Any]] = []
    _timebase_final_court_check_candidate_envelope(timebase, violations)
    _timebase_final_court_check_delta_raw_interval(timebase, violations)
    _timebase_final_court_check_ocxo_lane_alive(timebase, violations)
    recovery_degraded = _timebase_final_court_recovery_degraded_context(timebase)
    if recovery_degraded and not violations:
        _diag["timebase_final_court_degraded_recovery_admitted"] = (
            _diag.get("timebase_final_court_degraded_recovery_admitted", 0) + 1
        )

    count = _first_present_int_path(
        timebase,
        "teensy_pps_vclock_count",
        "fragment.teensy_pps_vclock_count",
        "pps_count",
    )
    primary = violations[0] if violations else {}
    ok = len(violations) == 0

    verdict: Dict[str, Any] = {
        "schema": "PI_TIMEBASE_FINAL_COURT_V2",
        "valid": ok,
        "classification": "ACCEPT" if ok else "DROP_ROW",
        "reason": TIMEBASE_FINAL_COURT_REASON,
        "primary_rule": primary.get("rule"),
        "rationale": primary.get("message") if primary else "candidate accepted",
        "source": TIMEBASE_FINAL_COURT_SOURCE,
        "source_process": "CLOCKS",
        "source_report": "TIMEBASE_FINAL_COURT",
        "campaign": timebase.get("campaign"),
        "teensy_pps_vclock_count": count,
        "teensy_pps_count": count,
        "pps_count": count,
        "timebase_schema": timebase.get("schema"),
        "fragment_schema": _path_get(timebase, "fragment.schema"),
        "timebase_message_version": timebase.get("timebase_message_version"),
        "teensy_evidence_invalid_mask": (
            _as_int(_path_get(timebase, "fragment.teensy_evidence_invalid_mask")) or 0
        ),
        "teensy_evidence_status": _path_get(timebase, "fragment.teensy_evidence_status"),
        "teensy_evidence_reason": _path_get(
            timebase, "fragment.teensy_evidence_reason"
        ),
        "timebase_pair_version": timebase.get("timebase_pair_version"),
        "rule_count": 4,
        "violation_count": len(violations),
        "recovery_degraded_context": recovery_degraded,
        "violations": violations,
    }

    return ok, verdict


_final_court_row_fatal_consecutive: int = 0
_final_court_row_fatal_signature: Optional[str] = None
_final_court_row_fatal_first_pps: Optional[int] = None
_final_court_row_fatal_last_pps: Optional[int] = None


def _timebase_final_court_violation_rules(verdict: Dict[str, Any]) -> List[str]:
    violations = verdict.get("violations")
    if not isinstance(violations, list):
        return []
    return [str(v.get("rule") or "") for v in violations if isinstance(v, dict)]


def _timebase_final_court_row_fatal(verdict: Dict[str, Any]) -> bool:
    rules = _timebase_final_court_violation_rules(verdict)
    return bool(rules) and all(rule in TIMEBASE_FINAL_COURT_ROW_FATAL_RULES for rule in rules)


def _timebase_final_court_signature(verdict: Dict[str, Any]) -> str:
    violations = verdict.get("violations")
    if not isinstance(violations, list):
        return ""
    parts: List[str] = []
    for v in violations:
        if not isinstance(v, dict):
            continue
        parts.append(f"{v.get('rule', '')}:{v.get('lane', '')}")
    return ",".join(sorted(parts))


def _timebase_final_court_reset_row_fatal_streak(reason: str) -> None:
    global _final_court_row_fatal_consecutive
    global _final_court_row_fatal_signature
    global _final_court_row_fatal_first_pps
    global _final_court_row_fatal_last_pps

    if _final_court_row_fatal_consecutive:
        _diag["last_timebase_final_court_row_drop"] = {
            **(_diag.get("last_timebase_final_court_row_drop") or {}),
            "reset_reason": reason,
            "reset_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        }

    _final_court_row_fatal_consecutive = 0
    _final_court_row_fatal_signature = None
    _final_court_row_fatal_first_pps = None
    _final_court_row_fatal_last_pps = None
    _diag["timebase_final_court_consecutive_row_fatal"] = 0


def _timebase_final_court_note_row_drop(verdict: Dict[str, Any]) -> bool:
    """Record a rejected-row streak for diagnostics only."""
    global _final_court_row_fatal_consecutive
    global _final_court_row_fatal_signature
    global _final_court_row_fatal_first_pps
    global _final_court_row_fatal_last_pps

    pps = _as_int(verdict.get("teensy_pps_vclock_count"))
    signature = _timebase_final_court_signature(verdict)

    if (
        _final_court_row_fatal_signature == signature
        and _final_court_row_fatal_last_pps is not None
        and pps is not None
        and int(pps) == int(_final_court_row_fatal_last_pps) + 1
    ):
        _final_court_row_fatal_consecutive += 1
    else:
        _final_court_row_fatal_consecutive = 1
        _final_court_row_fatal_signature = signature
        _final_court_row_fatal_first_pps = pps

    _final_court_row_fatal_last_pps = pps
    _diag["timebase_final_court_row_dropped"] = (
        _diag.get("timebase_final_court_row_dropped", 0) + 1
    )
    _diag["timebase_final_court_consecutive_row_fatal"] = (
        _final_court_row_fatal_consecutive
    )
    _diag["last_timebase_final_court_row_drop"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "campaign": verdict.get("campaign"),
        "teensy_pps_vclock_count": pps,
        "pps_count": pps,
        "signature": signature,
        "first_pps_vclock_count": _final_court_row_fatal_first_pps,
        "consecutive": _final_court_row_fatal_consecutive,
        "threshold": TIMEBASE_FINAL_COURT_ROW_FATAL_ESCALATE_CONSECUTIVE,
        "rules": _timebase_final_court_violation_rules(verdict),
        "verdict": verdict,
    }
    return (
        _final_court_row_fatal_consecutive
        >= TIMEBASE_FINAL_COURT_ROW_FATAL_ESCALATE_CONSECUTIVE
    )


def _timebase_final_court_block(
    verdict: Dict[str, Any],
    *,
    raw_record: Dict[str, Any],
    assembled_timebase: Dict[str, Any],
) -> None:
    """Log and drop one invalid row without converting it into recovery."""
    _diag["timebase_final_court_blocked"] += 1
    verdict["valid"] = False
    verdict["classification"] = "DROP_ROW"
    verdict["court_classification"] = "DROP_ROW"
    verdict["recovery_requested"] = False
    _diag["last_timebase_final_court"] = verdict

    _timebase_final_court_note_row_drop(verdict)
    _log_invalid_timebase(
        verdict=verdict,
        raw_record=raw_record,
        assembled_timebase=assembled_timebase,
    )

    logging.warning(
        "⚠️ [clocks] invalid TIMEBASE candidate dropped: campaign=%s pps=%s "
        "rule=%s rationale=%s",
        verdict.get("campaign"),
        verdict.get("teensy_pps_vclock_count"),
        verdict.get("primary_rule"),
        verdict.get("rationale"),
    )

    try:
        create_event("TIMEBASE_ROW_DROPPED", verdict)
    except Exception:
        _diag["timebase_final_court_event_enqueue_failures"] += 1
        logging.debug(
            "⚠️ [clocks] failed to enqueue TIMEBASE_ROW_DROPPED event",
            exc_info=True,
        )


def _dwt_cycles_to_recover_ns(cycles: int) -> int:
    """Convert a DWT cycle ledger to the current Teensy RECOVER dwt_ns argument.

    TIMEBASE_FRAGMENT_V3 publishes DWT as native cycles.  The current firmware
    RECOVER command still accepts dwt_ns and immediately converts it back to
    cycles.  Use a ceiling conversion so the round trip does not restore a
    cycle ledger that is one cycle below the projected value.
    """
    c = int(cycles)
    if c <= 0:
        return 0
    return (c * DWT_NS_NUM + DWT_NS_DEN - 1) // DWT_NS_DEN


def _dwt_recover_ns_to_cycles(ns: int) -> int:
    """Compatibility conversion for legacy persisted dwt_ns recovery rows."""
    n = int(ns)
    if n <= 0:
        return 0
    return (n * DWT_NS_DEN) // DWT_NS_NUM


def _report_fragment(report: Dict[str, Any]) -> Dict[str, Any]:
    frag = report.get("fragment")
    return frag if isinstance(frag, dict) else {}


def _report_extra_clocks(report: Dict[str, Any]) -> Dict[str, Any]:
    extra = report.get("extra_clocks")
    return extra if isinstance(extra, dict) else {}


def _report_forensics(report: Dict[str, Any]) -> Dict[str, Any]:
    forensics = report.get("forensics")
    return forensics if isinstance(forensics, dict) else {}


def _recovery_timebase_snapshot(tb: Dict[str, Any]) -> Dict[str, Any]:
    """Extract the recovery-critical truth from one persisted TIMEBASE row."""
    last_tb = dict(tb or {})
    last_frag = _report_fragment(last_tb)
    last_extra = _report_extra_clocks(last_tb)

    last_pps_vclock_count = int(_extract_last_timebase_count(last_tb, last_frag))

    last_gnss_ns = _fragment_ns(
        last_frag,
        "gnss.ns",
        "gnss.pps_vclock_ns",
        "gnss.vclock_ns",
        "pps_vclock_ns",
        "vclock_ns",
        "pps_ns",
        "gnss_ns",
        default=int(last_tb.get("teensy_gnss_ns") or 0),
    )

    legacy_last_dwt_ns = _fragment_ns(
        last_frag,
        "dwt.ns",
        "dwt_ns",
        default=int(last_tb.get("teensy_dwt_ns") or last_tb.get("dwt_ns") or 0),
    )
    last_dwt_cycles = _fragment_int(
        last_frag,
        "dwt.cycle_count_total",
        "dwt.cycles",
        "dwt.value",
        "dwt_cycle_count_total",
        "dwt_cycles",
        default=0,
    )
    if last_dwt_cycles == 0 and legacy_last_dwt_ns > 0:
        last_dwt_cycles = _dwt_recover_ns_to_cycles(legacy_last_dwt_ns)
    last_dwt_ns = (
        _dwt_cycles_to_recover_ns(last_dwt_cycles)
        if last_dwt_cycles > 0
        else legacy_last_dwt_ns
    )

    last_ocxo1_ns = _fragment_ns(
        last_frag,
        "ocxo1.ns",
        "gnss.ocxo1_ns",
        "ocxo1_ns",
        "ocxo1_ns_at_pps_vclock",
        default=int(last_tb.get("teensy_ocxo1_ns") or 0),
    )
    last_ocxo2_ns = _fragment_ns(
        last_frag,
        "ocxo2.ns",
        "gnss.ocxo2_ns",
        "ocxo2_ns",
        "ocxo2_ns_at_pps_vclock",
        default=int(last_tb.get("teensy_ocxo2_ns") or 0),
    )
    last_gnss_raw_ns = int(
        last_extra.get("gnss_raw_ns")
        or last_tb.get("gnss_raw_ns")
        or 0
    )
    last_gnss_raw_ref_ns = int(
        last_extra.get("gnss_raw_ref_ns")
        or last_tb.get("gnss_raw_ref_ns")
        or (last_pps_vclock_count * NS_PER_SECOND)
    )
    last_gnss_raw_welford_mean = _first_float(
        last_extra.get("gnss_raw_welford_mean"),
        last_tb.get("gnss_raw_welford_mean"),
        last_extra.get("gnss_raw_drift_ppb"),
        last_tb.get("gnss_raw_drift_ppb"),
        0.0,
    ) or 0.0
    last_gnss_time_str = (
        last_tb.get("gnss_time_utc")
        or last_tb.get("system_time_utc")
        or system_time_z()
    )

    recoverable = (
        last_pps_vclock_count > 0
        and last_gnss_ns > 0
        and last_dwt_cycles > 0
        and last_ocxo1_ns > 0
        and last_ocxo2_ns > 0
    )

    return {
        "last_tb": last_tb,
        "last_frag": last_frag,
        "last_pps_vclock_count": int(last_pps_vclock_count),
        "last_gnss_ns": int(last_gnss_ns),
        "legacy_last_dwt_ns": int(legacy_last_dwt_ns),
        "last_dwt_cycles": int(last_dwt_cycles),
        "last_dwt_ns": int(last_dwt_ns),
        "last_ocxo1_ns": int(last_ocxo1_ns),
        "last_ocxo2_ns": int(last_ocxo2_ns),
        "last_gnss_raw_ns": int(last_gnss_raw_ns),
        "last_gnss_raw_ref_ns": int(last_gnss_raw_ref_ns),
        "last_gnss_raw_welford_mean": float(last_gnss_raw_welford_mean),
        "last_gnss_time_str": str(last_gnss_time_str),
        "recoverable": bool(recoverable),
    }


def _load_last_recoverable_timebase(
    campaign_name: str,
    *,
    scan_limit: int = 64,
) -> Tuple[Dict[str, Any], Dict[str, Any], int]:
    """Return the newest TIMEBASE row usable as a warm-recovery base.

    The final court protects future rows, but older campaigns may already
    contain recovery-poison rows with zero OCXO ledgers.  Warm recovery must not
    project from those zeros.  Scan backward to the newest row whose GNSS, DWT,
    OCXO1, and OCXO2 ledgers are all nonzero; if none exists, return the newest
    row so the caller can fail loudly with full diagnostics.
    """
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY ts DESC
            LIMIT %s
            """,
            (campaign_name, int(scan_limit)),
        )
        rows = cur.fetchall()

    if not rows:
        raise LookupError("missing_timebase")

    newest_tb: Optional[Dict[str, Any]] = None
    newest_snapshot: Optional[Dict[str, Any]] = None
    newest_error: Optional[str] = None

    for skipped, row in enumerate(rows):
        tb = row["payload"]
        if isinstance(tb, str):
            tb = json.loads(tb)
        if not isinstance(tb, dict):
            continue

        try:
            snapshot = _recovery_timebase_snapshot(tb)
        except Exception as e:
            if newest_error is None:
                newest_error = str(e)
            continue

        if newest_tb is None:
            newest_tb = tb
            newest_snapshot = snapshot

        if snapshot.get("recoverable"):
            _diag["recovery_last_timebase_scan_count"] = (
                _diag.get("recovery_last_timebase_scan_count", 0) + int(skipped)
            )
            if skipped:
                _diag["recovery_last_timebase_unrecoverable"] = (
                    _diag.get("recovery_last_timebase_unrecoverable", 0) + int(skipped)
                )
                logging.warning(
                    "⚠️ [recovery] skipped %d latest TIMEBASE row(s) with unrecoverable ledgers; "
                    "using recoverable row at pps_vclock_count=%s",
                    skipped,
                    snapshot.get("last_pps_vclock_count"),
                )
            return tb, snapshot, int(skipped)

    if newest_tb is not None and newest_snapshot is not None:
        return newest_tb, newest_snapshot, 0

    raise RuntimeError(f"recovery failed: no parseable TIMEBASE payloads ({newest_error or 'unknown error'})")


def _normalize_recovery_servo_mode(value: Any) -> Optional[str]:
    if value is None:
        return None
    mode = str(value).strip().upper()
    if not mode:
        return None
    return mode if mode in ("MEAN", "TOTAL", "NOW", "OFF") else None


def _system_dither_args_from_config(cfg: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    boot_dither = cfg.get("dither")
    boot_dither_bool: Optional[bool] = None
    if isinstance(boot_dither, bool):
        boot_dither_bool = boot_dither
    elif isinstance(boot_dither, str):
        lowered = boot_dither.strip().lower()
        if lowered in ("true", "1", "yes", "on"):
            boot_dither_bool = True
        elif lowered in ("false", "0", "no", "off"):
            boot_dither_bool = False

    boot_rate_hz = None
    try:
        if cfg.get("dither_rate_hz") is not None:
            boot_rate_hz = int(cfg.get("dither_rate_hz"))
    except (TypeError, ValueError):
        boot_rate_hz = None

    if boot_dither_bool is None and boot_rate_hz is None:
        return None

    args: Dict[str, Any] = {"dither": boot_dither_bool if boot_dither_bool is not None else True}
    if boot_rate_hz is not None:
        args["rate_hz"] = int(boot_rate_hz)
    return args


def _reassert_system_dither(context: str, cfg: Optional[Dict[str, Any]] = None) -> None:
    """Best-effort reassertion of global dither state before START/RECOVER."""
    cfg = cfg if isinstance(cfg, dict) else _get_system_config()
    dither_args = _system_dither_args_from_config(cfg)
    if not dither_args:
        return

    try:
        resp = send_command(
            machine="TEENSY",
            subsystem="CLOCKS",
            command="DITHER",
            args=dither_args,
        )
        _diag["recovery_control_state_reassertions"] = (
            _diag.get("recovery_control_state_reassertions", 0) + 1
        )
        logging.info(
            "🔧 [%s] dither reassertion: %s resp=%s",
            context,
            dither_args,
            resp.get("message", "?"),
        )
    except Exception:
        logging.exception("⚠️ [%s] dither reassertion failed (ignored)", context)


def _recovery_control_state(
    *,
    campaign_payload: Dict[str, Any],
    last_tb: Optional[Dict[str, Any]] = None,
    last_frag: Optional[Dict[str, Any]] = None,
    system_cfg: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Choose START/RECOVER DAC and servo arguments from live truth first.

    Cold START is allowed to use operator/SYSTEM seed values.  Warm recovery is
    different: it should continue the last accepted campaign control state.
    Prefer the last good TIMEBASE DAC/servo surfaces, then SYSTEM, then the
    original campaign payload.
    """
    campaign_payload = campaign_payload if isinstance(campaign_payload, dict) else {}
    last_tb = last_tb if isinstance(last_tb, dict) else {}
    last_frag = last_frag if isinstance(last_frag, dict) else _report_fragment(last_tb)
    system_cfg = system_cfg if isinstance(system_cfg, dict) else _get_system_config()

    dac1, dac1_source = _first_float_with_source(
        ("last.fragment.dac.ocxo1_dac", _path_get(last_frag, "dac.ocxo1_dac")),
        ("last.fragment.dac.ocxo1.value", _path_get(last_frag, "dac.ocxo1.value")),
        ("last.fragment.ocxo1_dac", last_frag.get("ocxo1_dac")),
        ("last.forensics.dac.ocxo1.value", _path_get(last_tb, "forensics.dac.ocxo1.value")),
        ("SYSTEM.ocxo1_dac", system_cfg.get("ocxo1_dac")),
        ("campaign.payload.ocxo1_dac", campaign_payload.get("ocxo1_dac")),
    )
    dac2, dac2_source = _first_float_with_source(
        ("last.fragment.dac.ocxo2_dac", _path_get(last_frag, "dac.ocxo2_dac")),
        ("last.fragment.dac.ocxo2.value", _path_get(last_frag, "dac.ocxo2.value")),
        ("last.fragment.ocxo2_dac", last_frag.get("ocxo2_dac")),
        ("last.forensics.dac.ocxo2.value", _path_get(last_tb, "forensics.dac.ocxo2.value")),
        ("SYSTEM.ocxo2_dac", system_cfg.get("ocxo2_dac")),
        ("campaign.payload.ocxo2_dac", campaign_payload.get("ocxo2_dac")),
    )

    servo_mode = _normalize_recovery_servo_mode(
        _path_get(last_frag, "dac.calibrate_ocxo")
        or last_frag.get("servo_mode")
        or _path_get(last_tb, "forensics.dac.calibrate_ocxo")
        or campaign_payload.get("calibrate_ocxo")
    )
    if servo_mode is None:
        servo_active_raw = last_frag.get("servo_active")
        if servo_active_raw is not None and not _timebase_final_court_bool(servo_active_raw):
            servo_mode = "OFF"

    out: Dict[str, Any] = {
        "ocxo1_dac": dac1,
        "ocxo2_dac": dac2,
        "ocxo1_dac_source": dac1_source,
        "ocxo2_dac_source": dac2_source,
        "calibrate_ocxo": servo_mode,
        "calibrate_ocxo_source": (
            "last_timebase_or_campaign" if servo_mode is not None else None
        ),
        "dither_args": _system_dither_args_from_config(system_cfg),
    }
    return out


def _first_float(*values: Any) -> Optional[float]:
    for value in values:
        if value is None:
            continue
        try:
            f = float(value)
        except (TypeError, ValueError):
            continue
        if not math.isnan(f):
            return f
    return None


def _rounded(value: Optional[float], digits: int) -> Optional[float]:
    return None if value is None else round(float(value), digits)


def _gnss_raw_baseline_ppb_from_report(report: Dict[str, Any]) -> Optional[float]:
    """Return the baseline PPB value for the Pi-owned GNSS_RAW pseudo-clock.

    GNSS_RAW's accumulated tau/ppb can be invalid in older baseline rows when
    the synthetic ledger and reference ledger were restored with different
    cardinalities.  The Welford mean is the stable baseline statistic for the
    receiver-reported drift stream, and it is what the metrics panel already
    exposes as GN_RAW MEAN.
    """
    extra = _report_extra_clocks(report)
    return _first_float(
        extra.get("gnss_raw_welford_mean"),
        report.get("gnss_raw_welford_mean"),
        extra.get("gnss_raw_drift_ppb"),
        report.get("gnss_raw_drift_ppb"),
        extra.get("gnss_raw_ppb"),
        report.get("gnss_raw_ppb"),
    )


def _tau_from_ppb(ppb: Optional[float]) -> Optional[float]:
    if ppb is None:
        return None
    return 1.0 + float(ppb) / 1.0e9


def _baseline_ppb_from_report(report: Dict[str, Any]) -> Dict[str, float]:
    """Extract baseline PPB values from the current TIMEBASE-shaped report."""
    frag = _report_fragment(report)
    extra = _report_extra_clocks(report)

    candidates = {
        "gnss": 0.0,
        "vclock": _first_float(_path_get(frag, "stats.vclock.ppb"), frag.get("vclock_ppb"), report.get("vclock_ppb")),
        "gnss_raw": _gnss_raw_baseline_ppb_from_report(report),
        "dwt": _first_float(_path_get(frag, "stats.dwt.ppb"), frag.get("dwt_ppb"), report.get("dwt_ppb")),
        "ocxo1": _first_float(_path_get(frag, "stats.ocxo1.ppb"), frag.get("ocxo1_ppb"), report.get("ocxo1_ppb")),
        "ocxo2": _first_float(_path_get(frag, "stats.ocxo2.ppb"), frag.get("ocxo2_ppb"), report.get("ocxo2_ppb")),
    }

    return {k: round(v, 3) for k, v in candidates.items() if v is not None}


def _baseline_tau_from_report(report: Dict[str, Any]) -> Dict[str, float]:
    """Extract baseline tau values from the current TIMEBASE-shaped report."""
    frag = _report_fragment(report)
    extra = _report_extra_clocks(report)
    gnss_raw_ppb = _gnss_raw_baseline_ppb_from_report(report)

    candidates = {
        "gnss": 1.0,
        "vclock": _first_float(_path_get(frag, "stats.vclock.tau"), frag.get("vclock_tau"), report.get("vclock_tau")),
        "gnss_raw": _first_float(extra.get("gnss_raw_tau"), report.get("gnss_raw_tau")),
        "dwt": _first_float(_path_get(frag, "stats.dwt.tau"), frag.get("dwt_tau"), report.get("dwt_tau")),
        "ocxo1": _first_float(_path_get(frag, "stats.ocxo1.tau"), frag.get("ocxo1_tau"), report.get("ocxo1_tau")),
        "ocxo2": _first_float(_path_get(frag, "stats.ocxo2.tau"), frag.get("ocxo2_tau"), report.get("ocxo2_tau")),
    }
    candidates["gnss_raw"] = _tau_from_ppb(gnss_raw_ppb) or candidates.get("gnss_raw")

    return {k: round(v, 12) for k, v in candidates.items() if v is not None}


def _baseline_dac_from_report(report: Dict[str, Any]) -> Dict[str, float]:
    """Extract instantaneous DAC setpoints from the current TIMEBASE-shaped report."""
    frag = _report_fragment(report)
    forensics = _report_forensics(report)
    out: Dict[str, float] = {}
    for key in ("ocxo1", "ocxo2"):
        v = _first_float(
            _path_get(forensics, f"dac.{key}.value"),
            _path_get(frag, f"dac.{key}.value"),
            frag.get(f"{key}_dac"),
            report.get(f"{key}_dac"),
        )
        if v is not None:
            out[key] = round(v, 6)
    return out


def _baseline_dac_mean_from_report(report: Dict[str, Any]) -> Dict[str, float]:
    """Extract DAC mean values from the firmware Welford surface, falling back to current DAC."""
    frag = _report_fragment(report)
    current = _baseline_dac_from_report(report)
    out: Dict[str, float] = {}
    for key in ("ocxo1", "ocxo2"):
        current_v = current.get(key)
        v = _first_float(
            _path_get(frag, f"stats.dac.{key}.mean"),
            frag.get(f"{key}_dac_welford_mean"),
            report.get(f"{key}_dac_welford_mean"),
            report.get(f"{key}_dac_mean"),
            current_v,
        )
        if v == 0.0 and current_v is not None and current_v > 0.0:
            v = current_v
        if v is not None:
            out[key] = round(v, 6)
    return out


def _baseline_dac_stats_from_report(report: Dict[str, Any]) -> Dict[str, Dict[str, float | int]]:
    """Extract the full DAC Welford block for baseline audit/debug display."""
    frag = _report_fragment(report)
    out: Dict[str, Dict[str, float | int]] = {}
    for key in ("ocxo1", "ocxo2"):
        prefix = f"{key}_dac_welford"
        n = _as_int(_path_get(frag, f"stats.dac.{key}.n"))
        mean = _first_float(_path_get(frag, f"stats.dac.{key}.mean"))
        sd = _first_float(_path_get(frag, f"stats.dac.{key}.stddev"))
        se = _first_float(_path_get(frag, f"stats.dac.{key}.stderr"))
        mn = _first_float(_path_get(frag, f"stats.dac.{key}.min"))
        mx = _first_float(_path_get(frag, f"stats.dac.{key}.max"))

        n = n if n is not None else _as_int(frag.get(f"{prefix}_n"))
        mean = mean if mean is not None else _first_float(frag.get(f"{prefix}_mean"))
        sd = sd if sd is not None else _first_float(frag.get(f"{prefix}_stddev"))
        se = se if se is not None else _first_float(frag.get(f"{prefix}_stderr"))
        mn = mn if mn is not None else _first_float(frag.get(f"{prefix}_min"))
        mx = mx if mx is not None else _first_float(frag.get(f"{prefix}_max"))

        stats: Dict[str, float | int] = {}
        if n is not None:
            stats["n"] = int(n)
        if mean is not None:
            stats["mean"] = round(mean, 6)
        if sd is not None:
            stats["stddev"] = round(sd, 6)
        if se is not None:
            stats["stderr"] = round(se, 6)
        if mn is not None:
            stats["min"] = round(mn, 6)
        if mx is not None:
            stats["max"] = round(mx, 6)
        if stats:
            out[key] = stats
    return out


def _configured_boot_dacs(cfg: Dict[str, Any]) -> Tuple[Optional[float], Optional[float], str]:
    """
    Return DAC values to push to Teensy at boot.

    Precedence is intentionally explicit:

      1. SYSTEM ocxo*_dac — user-selected boot seed, written by SET_DAC
      2. SYSTEM baseline_dac_mean / baseline_dac — historical baseline fallback

    The processor thread deliberately stores live campaign DAC observations
    under last_ocxo*_dac so ordinary TIMEBASE ingestion cannot overwrite the
    operator-selected boot seed.
    """
    d1 = _first_float(cfg.get("ocxo1_dac"))
    d2 = _first_float(cfg.get("ocxo2_dac"))
    source = "system_ocxo_dac"

    if d1 is None or d2 is None:
        baseline_mean = cfg.get("baseline_dac_mean") if isinstance(cfg.get("baseline_dac_mean"), dict) else {}
        baseline_dac = cfg.get("baseline_dac") if isinstance(cfg.get("baseline_dac"), dict) else {}
        if d1 is None:
            d1 = _first_float(baseline_mean.get("ocxo1"), baseline_dac.get("ocxo1"))
            if d1 is not None:
                source = "baseline_dac"
        if d2 is None:
            d2 = _first_float(baseline_mean.get("ocxo2"), baseline_dac.get("ocxo2"))
            if d2 is not None:
                source = "baseline_dac"

    return d1, d2, source


def _normalize_start_args(args: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """Return START args with tolerant aliases normalized to firmware names."""
    out: Dict[str, Any] = dict(args or {})

    # The typo is common enough to be worth forgiving.  The firmware only
    # understands calibrate_ocxo, so normalize all accepted spellings here.
    if not out.get("calibrate_ocxo"):
        for alias in ("calibrate_oxco", "calibrate_oxo", "calibrate", "calibrate_ocxo_mode"):
            value = out.get(alias)
            if value is not None and str(value).strip():
                out["calibrate_ocxo"] = str(value).strip().upper()
                logging.warning(
                    "⚠️ [clocks] START argument '%s' is deprecated/forgiven; use calibrate_ocxo=%s",
                    alias, out["calibrate_ocxo"],
                )
                break
    elif isinstance(out.get("calibrate_ocxo"), str):
        out["calibrate_ocxo"] = out["calibrate_ocxo"].strip().upper()

    return out



# ---------------------------------------------------------------------
# Asynchronous START helpers
# ---------------------------------------------------------------------


def _mark_start_waiting(campaign: str) -> None:
    """Record that START returned before the first TIMEBASE_FRAGMENT."""
    global _start_waiting_for_first_fragment, _start_requested_campaign
    global _start_requested_at_utc, _start_requested_monotonic
    global _start_first_fragment_at_utc, _start_first_fragment_wait_s
    global _start_first_fragment_pps_vclock_count

    now_utc = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")

    _start_waiting_for_first_fragment = True
    _start_requested_campaign = campaign
    _start_requested_at_utc = now_utc
    _start_requested_monotonic = time.monotonic()
    _start_first_fragment_at_utc = None
    _start_first_fragment_wait_s = None
    _start_first_fragment_pps_vclock_count = None

    _diag["start_async_requests"] = _diag.get("start_async_requests", 0) + 1
    _diag["start_waiting_for_first_fragment"] = True
    _diag["start_requested_campaign"] = campaign
    _diag["start_requested_at_utc"] = now_utc
    _diag["start_first_fragment_at_utc"] = None
    _diag["start_first_fragment_wait_s"] = None
    _diag["start_first_fragment_pps_vclock_count"] = None
    _diag["last_start_async"] = {
        "campaign": campaign,
        "requested_at_utc": now_utc,
        "state": "WAITING_FOR_FIRST_FRAGMENT",
    }


def _mark_start_first_fragment_if_needed(
    *,
    campaign: str,
    pps_vclock_count: int,
) -> None:
    """Close the async START wait window on the first accepted fragment."""
    global _start_waiting_for_first_fragment, _start_requested_campaign
    global _start_requested_monotonic, _start_first_fragment_at_utc
    global _start_first_fragment_wait_s, _start_first_fragment_pps_vclock_count

    if not _start_waiting_for_first_fragment:
        return
    if _start_requested_campaign is not None and campaign != _start_requested_campaign:
        return

    now_utc = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    waited_s = (
        time.monotonic() - _start_requested_monotonic
        if _start_requested_monotonic is not None
        else 0.0
    )

    _start_waiting_for_first_fragment = False
    _start_first_fragment_at_utc = now_utc
    _start_first_fragment_wait_s = float(waited_s)
    _start_first_fragment_pps_vclock_count = int(pps_vclock_count)

    _diag["start_waiting_for_first_fragment"] = False
    _diag["start_first_fragment_at_utc"] = now_utc
    _diag["start_first_fragment_wait_s"] = round(float(waited_s), 3)
    _diag["start_first_fragment_pps_vclock_count"] = int(pps_vclock_count)
    _diag["last_start_async"] = {
        "campaign": campaign,
        "requested_at_utc": _start_requested_at_utc,
        "first_fragment_at_utc": now_utc,
        "waited_s": round(float(waited_s), 3),
        "teensy_pps_vclock_count": int(pps_vclock_count),
        "state": "RUNNING",
        "flash_cut": bool(_flash_cut_pending),
    }

    _mark_flash_cut_committed_if_needed(
        campaign=campaign,
        pps_vclock_count=int(pps_vclock_count),
    )

    logging.info(
        "✅ [start] @%s first TIMEBASE_FRAGMENT accepted asynchronously "
        "(campaign='%s', pps_vclock_count=%d, waited=%.3fs)",
        system_time_z(), campaign, int(pps_vclock_count), float(waited_s),
    )


def _start_status_payload() -> Dict[str, Any]:
    """Return compact async START state for REPORT/CLOCKS_INFO."""
    seconds_waiting = None
    if _start_waiting_for_first_fragment and _start_requested_monotonic is not None:
        seconds_waiting = round(time.monotonic() - _start_requested_monotonic, 3)

    return {
        "startup_control_ready": _startup_control_ready.is_set(),
        "waiting_for_first_fragment": bool(_start_waiting_for_first_fragment),
        "campaign": _start_requested_campaign,
        "requested_at_utc": _start_requested_at_utc,
        "seconds_waiting": seconds_waiting,
        "first_fragment_at_utc": _start_first_fragment_at_utc,
        "first_fragment_wait_s": (
            None if _start_first_fragment_wait_s is None
            else round(float(_start_first_fragment_wait_s), 3)
        ),
        "first_fragment_pps_vclock_count": _start_first_fragment_pps_vclock_count,
        "flash_cut_waiting": bool(_flash_cut_pending),
        "flash_cut_from": _flash_cut_from_campaign,
        "flash_cut_to": _flash_cut_to_campaign,
        "flash_cut_requested_at_utc": _flash_cut_requested_at_utc,
    }


def _startup_control_gate(operation: str) -> Optional[Dict[str, Any]]:
    """Reject lifecycle starts until boot reconciliation has completed.

    The command server is intentionally exposed before boot recovery so PUBSUB
    can discover routes.  START/RESUME, however, must not race the boot DAC push
    or the active-campaign recovery scan.  STOP, REPORT, CLEAR, and recovery
    abort remain available throughout startup.
    """
    if _startup_control_ready.is_set():
        return None

    now_utc = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    rejection = {
        "ts_utc": now_utc,
        "operation": str(operation),
        "status": "STARTUP_CONTROL_BUSY",
        "startup_control_ready": False,
    }
    _diag["startup_control_busy_rejections"] = (
        _diag.get("startup_control_busy_rejections", 0) + 1
    )
    _diag["last_startup_control_rejection"] = rejection
    return {
        "success": False,
        "message": (
            "CLOCKS startup control reconciliation is still in progress; "
            f"retry {operation} after startup_control_ready becomes true"
        ),
        "payload": rejection,
    }


def _clear_start_wait_state() -> None:
    """Clear pending async START observation state on STOP/CLEAR/recovery entry."""
    global _start_waiting_for_first_fragment, _start_requested_campaign
    global _start_requested_at_utc, _start_requested_monotonic
    global _start_first_fragment_at_utc, _start_first_fragment_wait_s
    global _start_first_fragment_pps_vclock_count

    _start_waiting_for_first_fragment = False
    _start_requested_campaign = None
    _start_requested_at_utc = None
    _start_requested_monotonic = None
    _start_first_fragment_at_utc = None
    _start_first_fragment_wait_s = None
    _start_first_fragment_pps_vclock_count = None

    _diag["start_waiting_for_first_fragment"] = False
    _diag["start_requested_campaign"] = None
    _diag["start_requested_at_utc"] = None


# ---------------------------------------------------------------------
# WATCHDOG_ANOMALY handler (PUBSUB — fast path)
# ---------------------------------------------------------------------


def on_watchdog_anomaly(payload: Payload) -> None:
    """
    PUBSUB handler for WATCHDOG_ANOMALY from Teensy CLOCKS.

    This is an explicit semantic surrender by the Teensy: campaign continuity
    is no longer being asserted. We enqueue a durable event and then initiate
    Pi-side recovery using the existing battle-tested protocol.
    """
    _diag["watchdog_anomalies_received"] += 1

    anomaly = dict(payload)
    _diag["last_watchdog_anomaly"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "reason": anomaly.get("reason"),
        "campaign": anomaly.get("campaign"),
        "sequence": anomaly.get("sequence"),
        "teensy_pps_vclock_count": (
            anomaly.get("teensy_pps_vclock_count")
            or anomaly.get("teensy_pps_count")
            or anomaly.get("pps_count")
        ),
        "campaign_seconds": anomaly.get("campaign_seconds"),
    }

    try:
        create_event("WATCHDOG_ANOMALY", anomaly)
    except Exception:
        _diag["watchdog_anomaly_event_enqueue_failures"] += 1
        logging.exception("⚠️ [clocks] failed to enqueue WATCHDOG_ANOMALY event")

    started = _begin_auto_recovery(
        "watchdog_anomaly",
        {"payload": anomaly},
        source="WATCHDOG_ANOMALY",
    )
    if started:
        _diag["watchdog_anomaly_recovery_started"] += 1


def on_recovery_stalled(payload: Payload) -> None:
    """Record a non-destructive OCXO recovery-liveness anomaly.

    This event means the Teensy timeline is alive but the OCXO science proof has
    stopped advancing.  It is deliberately not WATCHDOG_ANOMALY: restarting
    RECOVER here would destroy the very reattachment state being diagnosed.
    """
    _diag["recovery_stalled_events_received"] += 1
    # Compatibility counter: this is a science-cleanliness stall, but it no
    # longer aborts or restarts the live recovery lifecycle.
    _diag["recovery_clean_stalls"] = _diag.get("recovery_clean_stalls", 0) + 1

    stalled = dict(payload)
    snapshot = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "reason": stalled.get("reason"),
        "campaign": stalled.get("campaign"),
        "campaign_seconds": stalled.get("campaign_seconds"),
        "recovery_generation": stalled.get("recovery_generation"),
        "no_progress_rows": stalled.get("no_progress_rows"),
        "stall_threshold_rows": stalled.get("stall_threshold_rows"),
        "last_progress_public_count": stalled.get("last_progress_public_count"),
        "clockface_ready": stalled.get("clockface_ready"),
        "science_ready": stalled.get("science_ready"),
    }
    _diag["last_recovery_stalled"] = snapshot

    try:
        create_event(CLOCKS_RECOVERY_STALLED_TOPIC, stalled)
    except Exception:
        _diag["recovery_stalled_event_enqueue_failures"] += 1
        logging.exception(
            "⚠️ [recovery] failed to enqueue %s event",
            CLOCKS_RECOVERY_STALLED_TOPIC,
        )

    logging.error(
        "🧭 [recovery] OCXO science reattachment reports no progress "
        "(campaign=%s generation=%s rows=%s threshold=%s); "
        "timeline publication continues and RECOVER is not restarted",
        snapshot.get("campaign"),
        snapshot.get("recovery_generation"),
        snapshot.get("no_progress_rows"),
        snapshot.get("stall_threshold_rows"),
    )


def _enqueue_timebase_piece(topic: str, payload: Dict[str, Any]) -> None:
    """Enqueue one Teensy TIMEBASE candidate for processor-thread adjudication."""
    _fragment_queue.put({"topic": topic, "payload": dict(payload)})

    depth = _fragment_queue.qsize()
    _diag["queue_depth_current"] = depth
    if depth > _diag["queue_depth_max_seen"]:
        _diag["queue_depth_max_seen"] = depth


def _drain_timebase_ingress() -> int:
    """Drain queued TIMEBASE candidates that belong to an old lifecycle."""
    drained = 0
    while not _fragment_queue.empty():
        try:
            _fragment_queue.get_nowait()
            drained += 1
        except queue.Empty:
            break

    return drained


# ---------------------------------------------------------------------
# Fragment handler (PUBSUB — fast path, NEVER blocks)
# ---------------------------------------------------------------------


def on_timebase_fragment(payload: Payload) -> None:
    """
    PUBSUB handler for the unified Teensy TIMEBASE candidate.

    Fast path only: copy, note liveness, and enqueue.  Identity and scientific
    validity are adjudicated in the processor thread so even malformed rows are
    captured in the dedicated invalid-TIMEBASE log.
    """
    _diag["timebase_candidates_received"] += 1
    _diag["fragments_received"] += 1
    candidate = dict(payload)

    pps_vclock_count: Optional[int]
    try:
        pps_vclock_count = _extract_teensy_pps_vclock_count(
            candidate,
            topic=TIMEBASE_FRAGMENT_TOPIC,
        )
    except ValueError:
        pps_vclock_count = None
        _diag["fragments_missing_teensy_pps_count"] += 1
        _diag["fragments_missing_teensy_pps_vclock_count"] += 1

    _note_timebase_activity(TIMEBASE_FRAGMENT_TOPIC, pps_vclock_count)
    _enqueue_timebase_piece(TIMEBASE_FRAGMENT_TOPIC, candidate)
    _diag["timebase_candidates_queued"] += 1
    _diag["fragments_queued"] += 1


# ---------------------------------------------------------------------
# Processor thread (slow path — all the heavy lifting)
# ---------------------------------------------------------------------


def _process_loop() -> None:
    """
    Dedicated thread: pulls fragments from queue, decorates with
    environment and GNSS discipline data, builds TIMEBASE, persists.
    Runs forever.
    """
    global _campaign_active, _accepted_pps_vclock_count, _gnss_raw_ns, _gnss_raw_n, _gnss_raw_valid
    global _gnss_raw_welford_n, _gnss_raw_welford_mean, _gnss_raw_welford_m2
    global _gnss_raw_welford_min, _gnss_raw_welford_max

    logging.info("🚀 [clocks] processor thread started")

    while True:
        try:
            piece = _fragment_queue.get(timeout=0.25)
        except queue.Empty:
            continue

        _diag["timebase_pieces_processed"] += 1
        _diag["queue_depth_current"] = _fragment_queue.qsize()

        topic = str(piece.get("topic") or "")
        payload = piece.get("payload")
        if not isinstance(payload, dict):
            logging.error("💥 [clocks] processor received malformed TIMEBASE candidate: %s", piece)
            continue

        raw_record = copy.deepcopy(payload)
        frag = dict(payload)
        embedded_forensics = frag.pop("forensics", None)
        forensics = (
            dict(embedded_forensics)
            if isinstance(embedded_forensics, dict)
            else {}
        )

        try:
            pps_vclock_count = _extract_teensy_pps_vclock_count(
                frag,
                topic=topic or TIMEBASE_FRAGMENT_TOPIC,
            )
            _normalize_fragment_count_aliases(frag, pps_vclock_count)
            if forensics:
                _normalize_fragment_count_aliases(forensics, pps_vclock_count)
        except ValueError as exc:
            _diag["fragments_missing_teensy_pps_count"] += 1
            _diag["fragments_missing_teensy_pps_vclock_count"] += 1
            court_timebase = {
                "schema": "TIMEBASE_V3",
                "campaign": str(frag.get("campaign") or ""),
                "fragment": frag,
                "forensics": forensics,
                "identity_error": str(exc),
            }
            _diag["timebase_final_court_checks"] += 1
            _court_ok, court_verdict = _timebase_final_court_evaluate(court_timebase)
            _timebase_final_court_block(
                court_verdict,
                raw_record=raw_record,
                assembled_timebase=court_timebase,
            )
            continue

        _diag["timebase_candidates_processed"] += 1
        _diag["timebase_rows_completed"] += 1
        # Compatibility counter: one unified candidate is one completed former pair.
        _diag["timebase_pairs_completed"] += 1

        # The active DB campaign supplies the canonical namespace.  During
        # recovery _campaign_active may still be false while this row is being
        # used to satisfy the accepted-candidate sync latch.
        row = _get_active_campaign()
        campaign = row["campaign"] if row is not None else str(frag.get("campaign") or "")
        campaign_payload = row["payload"] if row is not None else {}

        # --- Final court before any Pi-owned state or recovery latch mutates ---
        system_time_utc = datetime.now(timezone.utc)
        system_time_str = system_time_utc.isoformat(timespec="microseconds")
        court_timebase = {
            "schema": "TIMEBASE_V3",
            "timebase_message_version": frag.get("timebase_message_version"),
            "timebase_pair_version": frag.get("timebase_pair_version"),
            "campaign": campaign,
            "teensy_pps_vclock_count": int(pps_vclock_count),
            "teensy_pps_count": int(pps_vclock_count),
            "pps_count": int(pps_vclock_count),
            "fragment": frag,
            "forensics": forensics,
        }

        if _candidate_disposition(frag) == TIMEBASE_CANDIDATE_SCIENCE_REJECT:
            # Receipt was already recorded as TIMEBASE heartbeat in the PUBSUB
            # fast path.  Track candidate identity, retain the complete evidence,
            # and stop before final court, recovery sync, GNSS_RAW, publication,
            # or PostgreSQL state can advance.
            _note_pps_vclock_count(int(pps_vclock_count))
            _timebase_final_court_reset_row_fatal_streak(
                "firmware_science_reject"
            )
            _drop_firmware_science_reject(
                campaign=campaign,
                pps_vclock_count=int(pps_vclock_count),
                fragment=frag,
                raw_record=raw_record,
                assembled_timebase=court_timebase,
            )
            continue

        _diag["timebase_final_court_checks"] += 1
        court_ok, court_verdict = _timebase_final_court_evaluate(court_timebase)
        if not court_ok:
            _timebase_final_court_block(
                court_verdict,
                raw_record=raw_record,
                assembled_timebase=court_timebase,
            )
            continue

        _timebase_final_court_reset_row_fatal_streak("final_court_passed")
        _diag["timebase_final_court_passed"] += 1

        sync_matched = _signal_sync_candidate_if_needed(frag, pps_vclock_count)
        if sync_matched and not _campaign_active:
            # RECOVER waits for the first Pi-accepted candidate, not merely the
            # first transport message.  Pi-owned recovery state is restored
            # before this accepted row is allowed to persist.
            if not _sync_resume_event.wait(timeout=SYNC_RECOVER_TIMEOUT_S):
                logging.error(
                    "💥 [recovery] timed out reopening processor for first accepted candidate count=%d",
                    int(pps_vclock_count),
                )
                continue

        if not _campaign_active:
            _diag["fragments_ignored_no_campaign"] += 1
            _diag["timebase_candidates_ignored_no_campaign"] += 1
            continue

        # Refresh after a recovery sync because the control thread may have
        # activated or switched the campaign while this processor was paused.
        row = _get_active_campaign()
        if row is None:
            _diag["hard_fault_no_active_campaign"] += 1
            try:
                _hard_fault(
                    "no_active_campaign",
                    {
                        "teensy_pps_vclock_count": int(pps_vclock_count),
                        "pps_count": int(pps_vclock_count),
                    },
                )
            except RuntimeError:
                continue

        campaign = row["campaign"]
        campaign_payload = row["payload"]
        if campaign != court_timebase["campaign"]:
            court_timebase["campaign"] = campaign
            _diag["timebase_final_court_checks"] += 1
            court_ok, court_verdict = _timebase_final_court_evaluate(court_timebase)
            if not court_ok:
                _timebase_final_court_block(
                    court_verdict,
                    raw_record=raw_record,
                    assembled_timebase=court_timebase,
                )
                continue

        _diag["fragments_processed"] += 1
        _note_pps_vclock_count(pps_vclock_count)

        # --- Accept Teensy PPS/VCLOCK count as observed truth ---
        #
        # Do not compare this candidate against an armed/expected Pi-side count.
        # The Pi is a TIMEBASE traffic cop here, not a campaign-count authority.
        _accepted_pps_vclock_count = int(pps_vclock_count)
        _diag["accepted_pps_count"] = _accepted_pps_vclock_count
        _diag["accepted_pps_vclock_count"] = _accepted_pps_vclock_count

        _mark_start_first_fragment_if_needed(
            campaign=campaign,
            pps_vclock_count=int(pps_vclock_count),
        )

        # --- Extract GNSS ns for stream health canary ---
        gnss_ns = int(frag.get("gnss_ns") or 0)

        # --- GNSS stream health canary (Pi-side diagnostic only) ---
        if gnss_ns > 0:
            _gnss_canary_update(gnss_ns)

        # --- Environment snapshot (best-effort, never blocks TIMEBASE) ---
        env_snapshot = _fetch_environment()

        # --- GNSS discipline snapshot (best-effort, never blocks TIMEBASE) ---
        gnss_info = _fetch_gnss_info()

        # --- GNSS_RAW synthetic clock ---
        gnss_raw_drift_ppb = None
        if gnss_info and isinstance(gnss_info, dict):
            v = gnss_info.get("clock_drift_ppb")
            if v is not None:
                try:
                    v_f = float(v)
                    if not math.isnan(v_f):
                        gnss_raw_drift_ppb = v_f
                except (ValueError, TypeError):
                    pass

        # Always accumulate one second; add drift correction when available
        _gnss_raw_ns += NS_PER_SECOND + (gnss_raw_drift_ppb if gnss_raw_drift_ppb is not None else 0.0)
        _gnss_raw_n += 1
        _gnss_raw_valid = True
        gnss_raw_ns_int = int(round(_gnss_raw_ns))
        gnss_raw_ref_ns = _gnss_raw_n * NS_PER_SECOND

        # Welford update on drift_ppb (treating it as the per-second residual)
        gnss_raw_pred_residual = gnss_raw_drift_ppb if gnss_raw_drift_ppb is not None else 0.0
        _gnss_raw_welford_n += 1
        d1 = gnss_raw_pred_residual - _gnss_raw_welford_mean
        _gnss_raw_welford_mean += d1 / _gnss_raw_welford_n
        d2 = gnss_raw_pred_residual - _gnss_raw_welford_mean
        _gnss_raw_welford_m2 += d1 * d2
        if gnss_raw_pred_residual < _gnss_raw_welford_min:
            _gnss_raw_welford_min = gnss_raw_pred_residual
        if gnss_raw_pred_residual > _gnss_raw_welford_max:
            _gnss_raw_welford_max = gnss_raw_pred_residual
        gnss_raw_welford_stddev = math.sqrt(
            _gnss_raw_welford_m2 / (_gnss_raw_welford_n - 1)) if _gnss_raw_welford_n >= 2 else 0.0
        gnss_raw_welford_stderr = (
            gnss_raw_welford_stddev / math.sqrt(_gnss_raw_welford_n)) if _gnss_raw_welford_n >= 2 else 0.0

        # --- GNSS confession PPB candidates ---
        # Computed here after unified-candidate final-court admission, because
        # only admitted rows may advance Pi-owned candidate state.
        ppb_candidates = _gnss_confession_update(
            pps_vclock_count=int(pps_vclock_count),
            frag=frag,
            gnss_info=gnss_info,
        )

        # --- Build TIMEBASE record ---
        timebase = {
            "schema": "TIMEBASE_V3",
            "timebase_message_version": frag.get("timebase_message_version"),
            "timebase_pair_version": frag.get("timebase_pair_version"),
            "campaign": campaign,
            "campaign_elapsed": _seconds_to_hms(int(pps_vclock_count)),
            "location": campaign_payload.get("location"),

            "system_time_utc": system_time_str,
            "gnss_time_utc": system_time_z(),

            "teensy_pps_vclock_count": int(pps_vclock_count),
            "teensy_pps_count": int(pps_vclock_count),   # legacy alias
            "pps_count": int(pps_vclock_count),          # legacy alias

            # Compatible durable shape reconstructed from one Teensy message.
            "fragment": frag,
            "forensics": forensics,

            # Environment snapshot (correlated with this PPS edge)
            "environment": env_snapshot,

            # GF-8802 discipline snapshot (correlated with this PPS edge)
            "gnss": gnss_info,

            # Pi-side synthetic clocks (not representable in TIMEBASE_FRAGMENT)
            "extra_clocks": {
                "gnss_raw_ns": gnss_raw_ns_int,
                "gnss_raw_ref_ns": gnss_raw_ref_ns,
                "gnss_raw_drift_ppb": gnss_raw_drift_ppb,
                "gnss_raw_tau": round(_compute_tau(gnss_raw_ns_int, gnss_raw_ref_ns), 12) if _gnss_raw_valid else None,
                "gnss_raw_ppb": round(_compute_ppb(gnss_raw_ns_int, gnss_raw_ref_ns), 3) if _gnss_raw_valid else None,
                "gnss_raw_welford_n": _gnss_raw_welford_n,
                "gnss_raw_welford_mean": round(_gnss_raw_welford_mean, 3),
                "gnss_raw_welford_stddev": round(gnss_raw_welford_stddev, 3),
                "gnss_raw_welford_stderr": round(gnss_raw_welford_stderr, 3),
                "gnss_raw_welford_min": round(_gnss_raw_welford_min, 3),
                "gnss_raw_welford_max": round(_gnss_raw_welford_max, 3),
            },

            # PI-owned candidate PPB surface.  This is the explicit "copy into
            # sacred field later" seam; fragment.stats remains Teensy-authored.
            "ppb_candidates": ppb_candidates,
        }

        publish("TIMEBASE", timebase)
        _persist_timebase(timebase)

        # Persist live desired DAC values as the SYSTEM boot seeds (best-effort).
        #
        # DAC Welfords are still useful servo-effort statistics, but they are
        # not actuator state.  With 1 kHz sigma-delta realization, the boot seed
        # is the Teensy-authored synthetic real DAC value that the dither layer
        # is honoring, not a historical mean that can lag the current target.
        ocxo1_dac_seed = _first_float(
            _path_get(frag, "dac.ocxo1_dac"),
            _path_get(frag, "dac.ocxo1.value"),
            frag.get("ocxo1_dac"),
        )
        ocxo2_dac_seed = _first_float(
            _path_get(frag, "dac.ocxo2_dac"),
            _path_get(frag, "dac.ocxo2.value"),
            frag.get("ocxo2_dac"),
        )

        if ocxo1_dac_seed is not None and ocxo2_dac_seed is not None:
            try:
                with open_db() as conn:
                    cur = conn.cursor()
                    cur.execute(
                        """
                        UPDATE config
                        SET timestamp = now(),
                            payload = jsonb_set(
                                jsonb_set(
                                    COALESCE(payload, '{}'::jsonb),
                                    '{ocxo1_dac}',
                                    to_jsonb(%s::double precision),
                                    true
                                ),
                                '{ocxo2_dac}',
                                to_jsonb(%s::double precision),
                                true
                            )
                        WHERE config_key = 'SYSTEM'
                        """,
                        (
                            round(float(ocxo1_dac_seed), 6),
                            round(float(ocxo2_dac_seed), 6),
                        ),
                    )
            except Exception:
                logging.exception("⚠️ [clocks] failed to persist DAC Welford means to SYSTEM config (ignored)")

# ---------------------------------------------------------------------
# Control-plane: START / STOP / CLEAR / RECOVER
# ---------------------------------------------------------------------


def _reset_trackers() -> int:
    global _last_pps_vclock_count_seen
    global _timebase_last_activity_monotonic
    global _timebase_last_activity_utc
    global _timebase_last_activity_topic
    global _timebase_last_activity_pps_vclock_count

    _last_pps_vclock_count_seen = None
    _timebase_last_activity_monotonic = None
    _timebase_last_activity_utc = None
    _timebase_last_activity_topic = None
    _timebase_last_activity_pps_vclock_count = None
    _diag["last_timebase_activity"] = {}
    _gnss_canary_reset()
    _gnss_raw_reset()
    _gnss_confession_reset()
    _timebase_final_court_reset_row_fatal_streak("trackers_reset")
    return _drain_timebase_ingress()

def _request_teensy_stop_best_effort() -> None:
    try:
        send_command(machine="TEENSY", subsystem="CLOCKS", command="STOP")
    except Exception:
        pass


class TeensyStartRejected(RuntimeError):
    """Raised when CLOCKS.START routes successfully but firmware rejects it."""

    def __init__(self, status: str, response: Dict[str, Any]):
        self.status = status
        self.response = response
        payload = response.get("payload") if isinstance(response, dict) else None
        error = payload.get("error") if isinstance(payload, dict) else None
        super().__init__(f"Teensy CLOCKS.START rejected: status={status!r} error={error!r}")


_TEENSY_START_ACCEPTED_STATUSES = {
    "start_pending_smartzero",
    "start_pending_smartzero_dac_fault",
    "flash_cut_requested",
    "flash_cut_requested_dac_fault_servo_off",
}


def _request_teensy_start(
    campaign: str,
    pps_vclock_count: int,
    args: Dict[str, Any],
) -> Dict[str, Any]:
    """Send CLOCKS.START and validate the firmware handler status.

    The outer RPC success/message fields prove only that the Teensy command
    router invoked the handler.  Campaign admission is expressed by the
    handler payload's status field and must be checked explicitly.
    """
    teensy_args = dict(args)
    # Teensy still accepts the legacy pps_count command argument.  Send the
    # explicit PPS/VCLOCK alias too; older firmware will ignore it.
    teensy_args.update({
        "campaign": campaign,
        "pps_count": str(int(pps_vclock_count)),
        "pps_vclock_count": str(int(pps_vclock_count)),
    })

    resp = send_command(
        machine="TEENSY",
        subsystem="CLOCKS",
        command="START",
        args=teensy_args,
    )
    _diag["teensy_start_responses"] = _diag.get("teensy_start_responses", 0) + 1

    payload = resp.get("payload") if isinstance(resp, dict) else None
    status = str(payload.get("status") or "") if isinstance(payload, dict) else ""
    evidence = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "campaign": campaign,
        "pps_vclock_count": int(pps_vclock_count),
        "outer_success": bool(resp.get("success")) if isinstance(resp, dict) else False,
        "outer_message": resp.get("message") if isinstance(resp, dict) else None,
        "status": status or None,
        "payload": payload,
    }
    _diag["last_teensy_start_response"] = evidence

    if not isinstance(resp, dict) or not resp.get("success"):
        _diag["teensy_start_rejected"] = _diag.get("teensy_start_rejected", 0) + 1
        logging.error(
            "💥 [start] Teensy CLOCKS.START transport/RPC failure: campaign='%s' response=%s",
            campaign,
            json.dumps(resp, sort_keys=True, default=str),
        )
        raise TeensyStartRejected(status or "outer_rpc_failure", resp if isinstance(resp, dict) else {})

    if not isinstance(payload, dict) or not status:
        _diag["teensy_start_malformed"] = _diag.get("teensy_start_malformed", 0) + 1
        logging.error(
            "💥 [start] Teensy CLOCKS.START returned no usable handler status: "
            "campaign='%s' outer_message=%s response=%s",
            campaign,
            resp.get("message"),
            json.dumps(resp, sort_keys=True, default=str),
        )
        raise TeensyStartRejected("missing_handler_status", resp)

    if status not in _TEENSY_START_ACCEPTED_STATUSES:
        _diag["teensy_start_rejected"] = _diag.get("teensy_start_rejected", 0) + 1
        logging.error(
            "💥 [start] Teensy CLOCKS.START REJECTED: campaign='%s' status='%s' "
            "error=%r outer_success=%s outer_message=%r handler_payload=%s",
            campaign,
            status,
            payload.get("error"),
            resp.get("success"),
            resp.get("message"),
            json.dumps(payload, sort_keys=True, default=str),
        )
        raise TeensyStartRejected(status, resp)

    _diag["teensy_start_accepted"] = _diag.get("teensy_start_accepted", 0) + 1
    return resp


_TEENSY_RECOVER_ACCEPTED_STATUSES = {
    "recover_requested",
    "recover_already_active",
    "recover_already_completed",
}


def _request_teensy_recover(
    pps_vclock_count: int,
    args: Dict[str, Any],
) -> Dict[str, Any]:
    """Send RECOVER and require an explicit Teensy lifecycle verdict."""
    teensy_args = dict(args)
    teensy_args["pps_count"] = str(int(pps_vclock_count))
    teensy_args["pps_vclock_count"] = str(int(pps_vclock_count))
    resp = send_command(
        machine="TEENSY",
        subsystem="CLOCKS",
        command="RECOVER",
        args=teensy_args,
    )
    outer_success = isinstance(resp, dict) and bool(resp.get("success"))
    payload = resp.get("payload") if isinstance(resp, dict) else None
    status = str(payload.get("status") or "") if isinstance(payload, dict) else ""
    if not outer_success or status not in _TEENSY_RECOVER_ACCEPTED_STATUSES:
        details = {
            "status": status or "missing_handler_status",
            "outer_success": outer_success,
            "outer_message": resp.get("message") if isinstance(resp, dict) else None,
            "payload": payload if isinstance(payload, dict) else {},
            "requested_base_count": int(pps_vclock_count),
        }
        raise RecoveryRetryableFailure("teensy_recover_rejected", details)
    return resp


def _request_teensy_recover_abort_best_effort(
    reason: str,
    details: Optional[Dict[str, Any]] = None,
) -> None:
    """Ask Teensy CLOCKS to clear any half-open RECOVER lifecycle state."""
    _diag["recovery_abort_requests"] = _diag.get("recovery_abort_requests", 0) + 1
    payload = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "reason": reason,
        "details": details or {},
    }
    try:
        resp = send_command(
            machine="TEENSY",
            subsystem="CLOCKS",
            command="RECOVER_ABORT",
            args={"reason": reason},
            retries=1,
            retry_delay_s=0.0,
        )
        payload["success"] = bool(resp.get("success"))
        payload["message"] = resp.get("message")
        payload["teensy_payload"] = resp.get("payload")
        if resp.get("success"):
            _diag["recovery_abort_success"] = _diag.get("recovery_abort_success", 0) + 1
            logging.info("🧯 [recovery] Teensy RECOVER_ABORT accepted: reason=%s", reason)
        else:
            _diag["recovery_abort_failures"] = _diag.get("recovery_abort_failures", 0) + 1
            logging.warning(
                "⚠️ [recovery] Teensy RECOVER_ABORT rejected/unsupported: reason=%s message=%s",
                reason, resp.get("message"),
            )
    except Exception as e:
        payload["success"] = False
        payload["error"] = str(e)
        _diag["recovery_abort_failures"] = _diag.get("recovery_abort_failures", 0) + 1
        logging.warning("⚠️ [recovery] Teensy RECOVER_ABORT failed/unsupported: %s", e)
    finally:
        _diag["last_recovery_abort"] = payload


def _cleanup_after_recovery_failure(reason: str, details: Dict[str, Any]) -> None:
    """Return Pi and Teensy to a commandable non-recovering state after RECOVER fails."""
    global _campaign_active

    _campaign_active = False
    _clear_sync_wait()
    _clear_start_wait_state()
    _clear_flash_cut_wait_state()
    drained = _drain_timebase_ingress()
    _request_teensy_recover_abort_best_effort(reason, details)
    _diag["last_recovery_abort"] = {
        **(_diag.get("last_recovery_abort") or {}),
        "pi_cleanup_reason": reason,
        "drained_timebase_pieces": int(drained),
    }


def cmd_recover_abort(args: Optional[dict]) -> Dict[str, Any]:
    """Manual operator escape hatch for a half-open RECOVER lifecycle."""
    reason = str((args or {}).get("reason") or "operator_recover_abort")
    details = {"operator_command": True}
    _cleanup_after_recovery_failure(reason, details)
    return {
        "success": True,
        "message": "OK",
        "payload": {
            "reason": reason,
            "last_recovery_abort": _diag.get("last_recovery_abort"),
            "campaign_active": bool(_campaign_active),
        },
    }


def cmd_start(args: Optional[dict]) -> dict:
    """
    START — asynchronous cold START or hot Flash Cut.

    The Pi prepares DB state and accepted-row ingress before arming Teensy, then
    accepts the very first TIMEBASE row. There is no Pi-side warmup
    or skipped-row model; if row #1 is unhealthy, the readiness gates or Teensy
    handoff should be fixed.
    """
    global _campaign_active, _accepted_pps_vclock_count

    startup_busy = _startup_control_gate("START")
    if startup_busy is not None:
        return startup_busy

    start_args = _normalize_start_args(args)
    campaign = start_args.get("campaign")
    if not campaign:
        return {"success": False, "message": "START requires 'campaign' argument"}

    active_row = _get_active_campaign()
    flash_cut = active_row is not None
    prev_campaign = active_row["campaign"] if flash_cut else None
    prev_payload = active_row["payload"] if flash_cut and isinstance(active_row.get("payload"), dict) else {}

    current_location = _get_current_location()

    try:
        _ensure_gnss_mode_for_current_location()
    except Exception as e:
        return {"success": False, "message": str(e)}

    _wait_for_preflight("flash_cut" if flash_cut else "start")

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
    set_dac1 = start_args.get("set_dac1")
    set_dac2 = start_args.get("set_dac2")
    calibrate_ocxo = start_args.get("calibrate_ocxo") or None
    if isinstance(calibrate_ocxo, str):
        calibrate_ocxo = calibrate_ocxo.strip().upper() or None
    if calibrate_ocxo and calibrate_ocxo not in ("MEAN", "TOTAL", "NOW", "OFF"):
        return {"success": False, "message": f"calibrate_ocxo must be 'MEAN' 'TOTAL' 'NOW' or 'OFF', got '{calibrate_ocxo}'"}

    dac_source = "operator" if (set_dac1 is not None or set_dac2 is not None) else "none"
    if not flash_cut and (set_dac1 is None or set_dac2 is None):
        try:
            with open_db(row_dict=True) as conn:
                cur = conn.cursor()
                cur.execute("SELECT payload FROM config WHERE config_key = 'SYSTEM'")
                row = cur.fetchone()
                if row:
                    if set_dac1 is None and row["payload"].get("ocxo1_dac") is not None:
                        set_dac1 = float(row["payload"]["ocxo1_dac"])
                        dac_source = "SYSTEM"
                    if set_dac2 is None and row["payload"].get("ocxo2_dac") is not None:
                        set_dac2 = float(row["payload"]["ocxo2_dac"])
                        dac_source = "SYSTEM"
        except Exception:
            logging.exception("⚠️ [clocks] failed to read SYSTEM config (ignored)")
    elif flash_cut and (set_dac1 is None or set_dac2 is None):
        dac_source = "operator+live" if (set_dac1 is not None or set_dac2 is not None) else "live"

    if flash_cut:
        prev_location = prev_payload.get("location")
        if location is None and prev_location:
            location = prev_location

    try:
        _wait_for_timebase_routes(context="flash_cut" if flash_cut else "start")
    except Exception as e:
        return {"success": False, "message": str(e)}

    logging.info(
        "%s [start] campaign='%s' mode=%s location=%s servo=%s dac1=%s dac2=%s dac_source=%s",
        "⚡" if flash_cut else "▶️",
        campaign,
        "FLASH_CUT" if flash_cut else "COLD_START",
        location or "none",
        calibrate_ocxo or "OFF",
        str(set_dac1),
        str(set_dac2),
        dac_source,
    )

    cutover_ts = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    campaign_payload = {
        "location": location,
        "started_at": cutover_ts,
    }
    if set_dac1 is not None:
        campaign_payload["ocxo1_dac"] = float(set_dac1)
    if set_dac2 is not None:
        campaign_payload["ocxo2_dac"] = float(set_dac2)
    if calibrate_ocxo is not None:
        # OFF is an explicit command in Flash Cut: stop servo but preserve the
        # current DAC/output state. Omitted means preserve current mode on hot
        # Teensy firmware and default OFF on cold firmware.
        campaign_payload["calibrate_ocxo"] = calibrate_ocxo

    if flash_cut:
        campaign_payload.update({
            "flash_cut_from": prev_campaign,
            "flash_cut_pending": True,
            "flash_cut_armed_at": cutover_ts,
            "flash_cut_preserves_teensy_state": True,
            "flash_cut_pi_mode": "HOT_START_NO_TEENSY_STOP",
        })

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

    if not flash_cut:
        _request_teensy_stop_best_effort()

    drained = _reset_trackers()
    if drained:
        logging.info("🧹 [start] drained %d stale TIMEBASE candidate(s) before arm", drained)
    _clear_sync_wait()

    _accepted_pps_vclock_count = None
    _diag["accepted_pps_count"] = None
    _diag["accepted_pps_vclock_count"] = None

    _mark_start_waiting(campaign)
    if flash_cut:
        _mark_flash_cut_waiting(str(prev_campaign), str(campaign))

    _campaign_active = True
    _arm_timebase_silence_watch("FLASH_CUT" if flash_cut else "START")

    teensy_args: Dict[str, Any] = {"campaign": campaign}
    if set_dac1 is not None:
        teensy_args["set_dac1"] = str(set_dac1)
    if set_dac2 is not None:
        teensy_args["set_dac2"] = str(set_dac2)
    if calibrate_ocxo is not None:
        teensy_args["calibrate_ocxo"] = calibrate_ocxo

    teensy_start_resp: Dict[str, Any] = {}
    try:
        teensy_start_resp = _request_teensy_start(
            campaign=campaign,
            pps_vclock_count=0,
            args=teensy_args,
        )
    except Exception as e:
        _campaign_active = False
        _accepted_pps_vclock_count = None
        _clear_start_wait_state()
        _clear_flash_cut_wait_state()

        failed_at = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
        failure_status = e.status if isinstance(e, TeensyStartRejected) else "teensy_start_command_failed"
        failure_response = e.response if isinstance(e, TeensyStartRejected) else {}
        failure_payload = (
            failure_response.get("payload")
            if isinstance(failure_response, dict)
            and isinstance(failure_response.get("payload"), dict)
            else {}
        )

        if not isinstance(e, TeensyStartRejected):
            logging.exception(
                "💥 [start] CLOCKS.START command failed unexpectedly: "
                "campaign='%s' mode=%s",
                campaign,
                "FLASH_CUT" if flash_cut else "COLD_START",
            )

        try:
            with open_db() as conn:
                cur = conn.cursor()
                cur.execute(
                    """
                    UPDATE campaigns
                    SET active = false,
                        payload = payload || jsonb_build_object(
                            'start_failed_at', to_jsonb(%s::text),
                            'start_failed_reason', to_jsonb(%s::text),
                            'start_failed_error', to_jsonb(%s::text),
                            'start_failed_teensy_payload', %s::jsonb
                        )
                    WHERE campaign = %s AND active = true
                    """,
                    (
                        failed_at,
                        str(failure_status),
                        str(failure_payload.get("error") or str(e)),
                        json.dumps(failure_payload),
                        campaign,
                    ),
                )
                if flash_cut and prev_campaign:
                    cur.execute(
                        """
                        UPDATE campaigns
                        SET active = true,
                            payload = payload - 'stopped_at'
                        WHERE campaign = %s
                        """,
                        (prev_campaign,),
                    )
        except Exception:
            logging.exception("⚠️ [clocks] failed to roll back DB after Teensy START rejection")

        return {
            "success": False,
            "message": f"Teensy CLOCKS.START rejected: {failure_status}",
            "payload": {
                "campaign": campaign,
                "flash_cut": flash_cut,
                "previous_campaign": prev_campaign,
                "teensy_status": failure_status,
                "teensy_error": failure_payload.get("error"),
                "teensy_payload": failure_payload,
                "outer_response": failure_response,
                "campaign_active": False,
                "database_rolled_back": True,
            },
        }

    teensy_start_payload = teensy_start_resp.get("payload", {})
    teensy_start_status = (
        str(teensy_start_payload.get("status") or "")
        if isinstance(teensy_start_payload, dict)
        else ""
    )
    logging.info(
        "✅ [start] @%s %s accepted by Teensy — campaign='%s' status='%s'; "
        "awaiting first TIMEBASE row",
        system_time_z(),
        "FLASH_CUT" if flash_cut else "START",
        campaign,
        teensy_start_status,
    )

    return {
        "success": True,
        "message": "START_REQUESTED",
        "payload": {
            "campaign": campaign,
            "location": location,
            "set_dac1": set_dac1,
            "set_dac2": set_dac2,
            "calibrate_ocxo": calibrate_ocxo,
            "waiting_for_first_fragment": True,
            "teensy_start_status": teensy_start_status,
            "teensy_start_payload": teensy_start_payload,
            "startup": _start_status_payload(),
            "flash_cut": flash_cut,
            "previous_campaign": prev_campaign,
            "sync_wait_removed": True,
            "teensy_stop_sent": not flash_cut,
            "default_dac_push_suppressed": bool(flash_cut),
            "first_fragment_timeout_s": (
                FLASH_CUT_FIRST_FRAGMENT_TIMEOUT_S if flash_cut else START_FIRST_FRAGMENT_TIMEOUT_S
            ),
            "skipped_records_expected": False,
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
    # STOP should be enough on current firmware.  RECOVER_ABORT is harmless if
    # supported and prevents old half-open RECOVER latches from surviving a Pi
    # STOP issued after a failed recovery attempt.
    _request_teensy_recover_abort_best_effort("pi_stop_cleanup")

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
    _clear_start_wait_state()
    _clear_flash_cut_wait_state()
    _drain_timebase_ingress()
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
        return {
            "success": True,
            "message": "OK",
            "payload": {
                "campaign_state": "IDLE",
                "startup": _start_status_payload(),
            },
        }

    payload = dict(row["payload"])
    payload["startup"] = _start_status_payload()
    return {"success": True, "message": "OK", "payload": payload}


def cmd_clear(_: Optional[dict]) -> dict:
    global _campaign_active

    _request_teensy_stop_best_effort()
    _request_teensy_recover_abort_best_effort("pi_clear_cleanup")

    try:
        with open_db() as conn:
            cur = conn.cursor()
            cur.execute("DELETE FROM timebase")
            tb_count = cur.rowcount
            cur.execute("DELETE FROM campaigns")
            camp_count = cur.rowcount

        _campaign_active = False
        _clear_start_wait_state()
        _clear_flash_cut_wait_state()
        _drain_timebase_ingress()
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

    startup_busy = _startup_control_gate("RESUME")
    if startup_busy is not None:
        return startup_busy

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
    RECOVER — v7 exact-first-row architecture.
    """
    global _campaign_active, _accepted_pps_vclock_count
    global _last_pps_vclock_count_seen

    # Immediately deactivate so the processor thread ignores all
    # fragments during recovery.
    _campaign_active = False
    _accepted_pps_vclock_count = None
    _clear_start_wait_state()

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
        if _reattach_pending_flash_cut_without_recovery(
            campaign_name=campaign_name,
            campaign_payload=campaign_payload,
        ):
            _diag["last_recovery"] = {
                "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "campaign": campaign_name,
                "mode": "flash_cut_zero_row_reattach",
                "sync_wait_removed": True,
                "waiting_for_first_fragment": True,
                "teensy_stop_sent": False,
            }
            return

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

        _wait_for_timebase_routes(context="recovery/cold")

        _request_teensy_stop_best_effort()

        drained = _reset_trackers()
        if drained:
            logging.info("🧹 [recovery/cold] drained %d stale TIMEBASE candidate(s)", drained)
        _clear_sync_wait()

        system_cfg = _get_system_config()
        control_state = _recovery_control_state(
            campaign_payload=campaign_payload,
            system_cfg=system_cfg,
        )
        _diag["last_recovery_control_state"] = {
            "context": "recovery/cold",
            **control_state,
        }
        _reassert_system_dither("recovery/cold", system_cfg)

        teensy_args: Dict[str, Any] = {"campaign": campaign_name}
        recover_ocxo1_dac = control_state.get("ocxo1_dac")
        recover_ocxo2_dac = control_state.get("ocxo2_dac")
        recover_calibrate = control_state.get("calibrate_ocxo")
        if recover_ocxo1_dac is not None:
            teensy_args["set_dac1"] = str(float(recover_ocxo1_dac))
        if recover_ocxo2_dac is not None:
            teensy_args["set_dac2"] = str(float(recover_ocxo2_dac))
        if recover_calibrate:
            teensy_args["calibrate_ocxo"] = str(recover_calibrate)

        _accepted_pps_vclock_count = None
        _diag["accepted_pps_count"] = None
        _diag["accepted_pps_vclock_count"] = None

        _mark_start_waiting(campaign_name)

        # Cold recovery has no prior TIMEBASE row to recover from.  Treat it as
        # an async START of the existing active campaign, matching cmd_start().
        # The normal processor thread will accept the first
        # TIMEBASE_FRAGMENT candidate whenever it arrives.
        _campaign_active = True
        _arm_timebase_silence_watch("RECOVERY_COLD_START")

        teensy_start_resp: Dict[str, Any] = {}
        try:
            teensy_start_resp = _request_teensy_start(
                campaign=campaign_name,
                pps_vclock_count=0,
                args=teensy_args,
            )
        except Exception:
            _campaign_active = False
            _accepted_pps_vclock_count = None
            _clear_start_wait_state()
            raise

        teensy_start_payload = teensy_start_resp.get("payload", {})
        teensy_start_status = (
            str(teensy_start_payload.get("status") or "")
            if isinstance(teensy_start_payload, dict)
            else ""
        )
        logging.info(
            "✅ [recovery/cold] START accepted: campaign='%s' status='%s'; "
            "awaiting first TIMEBASE row",
            campaign_name, teensy_start_status,
        )

        _diag["last_recovery"] = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "campaign": campaign_name,
            "mode": "cold_restart_async",
            "sync_wait_removed": True,
            "waiting_for_first_fragment": True,
        }
        return

    try:
        last_tb, recovery_snapshot, skipped_unrecoverable_rows = _load_last_recoverable_timebase(campaign_name)
    except LookupError:
        # Preserve the existing cold-restart path above for genuinely zero-row campaigns.
        raise RuntimeError(f"recovery failed: campaign '{campaign_name}' has no TIMEBASE rows")

    last_frag = recovery_snapshot["last_frag"]
    last_pps_vclock_count = int(recovery_snapshot["last_pps_vclock_count"])
    last_gnss_ns = int(recovery_snapshot["last_gnss_ns"])
    legacy_last_dwt_ns = int(recovery_snapshot["legacy_last_dwt_ns"])
    last_dwt_cycles = int(recovery_snapshot["last_dwt_cycles"])
    last_dwt_ns = int(recovery_snapshot["last_dwt_ns"])
    last_ocxo1_ns = int(recovery_snapshot["last_ocxo1_ns"])
    last_ocxo2_ns = int(recovery_snapshot["last_ocxo2_ns"])
    last_gnss_raw_ns = int(recovery_snapshot["last_gnss_raw_ns"])
    last_gnss_raw_ref_ns = int(recovery_snapshot.get("last_gnss_raw_ref_ns") or (last_pps_vclock_count * NS_PER_SECOND))
    last_gnss_raw_welford_mean = float(recovery_snapshot.get("last_gnss_raw_welford_mean") or 0.0)
    last_gnss_time_str = str(recovery_snapshot["last_gnss_time_str"])

    if not recovery_snapshot.get("recoverable"):
        raise RuntimeError(
            "recovery failed: newest TIMEBASE row is not warm-recoverable "
            f"(pps_vclock_count={last_pps_vclock_count}, gnss_ns={last_gnss_ns}, "
            f"dwt_cycles={last_dwt_cycles}, ocxo1_ns={last_ocxo1_ns}, "
            f"ocxo2_ns={last_ocxo2_ns})"
        )

    if last_gnss_ns == 0:
        logging.warning("⚠️ [recovery] teensy_gnss_ns=0 — using dwt_ns as proxy")
        last_gnss_ns = last_dwt_ns

    logging.info(
        "📐 [recovery] LAST TIMEBASE:\n"
        "    pps_vclock_count = %d\n"
        "    gnss_ns    = %d\n"
        "    dwt_cycles = %d\n"
        "    dwt_ns_arg = %d\n"
        "    ocxo1_ns   = %d\n"
        "    ocxo2_ns   = %d\n"
        "    gn_raw_ns  = %d\n"
        "    gn_raw_ref = %d\n"
        "    gn_raw_mean_ppb = %.6f\n"
        "    gnss_time  = %s",
        last_pps_vclock_count, last_gnss_ns, last_dwt_cycles, last_dwt_ns,
        last_ocxo1_ns, last_ocxo2_ns, last_gnss_raw_ns, last_gnss_raw_ref_ns,
        last_gnss_raw_welford_mean, last_gnss_time_str,
    )

    system_cfg = _get_system_config()
    control_state = _recovery_control_state(
        campaign_payload=campaign_payload,
        last_tb=last_tb,
        last_frag=last_frag,
        system_cfg=system_cfg,
    )
    recover_ocxo1_dac = control_state.get("ocxo1_dac")
    recover_ocxo2_dac = control_state.get("ocxo2_dac")
    recover_calibrate = control_state.get("calibrate_ocxo")
    _diag["last_recovery_control_state"] = {
        "context": "recovery",
        "skipped_unrecoverable_rows": int(skipped_unrecoverable_rows),
        **control_state,
    }

    logging.info(
        "🔧 [recovery] control state: ocxo1_dac=%s (%s) ocxo2_dac=%s (%s) servo=%s dither=%s",
        recover_ocxo1_dac,
        control_state.get("ocxo1_dac_source"),
        recover_ocxo2_dac,
        control_state.get("ocxo2_dac_source"),
        recover_calibrate or "OFF",
        control_state.get("dither_args"),
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

    # Warm recovery after a Teensy reboot must not depend on the exact
    # campaign-admission gate used by START.  REPORT_GATE may legitimately
    # report FEATURE_STATUS unseen immediately after reboot, which creates a
    # circular wait: the Pi refuses RECOVER until the Teensy has republished
    # readiness state, while the Teensy needs RECOVER to resume the campaign
    # publication lifecycle.
    #
    # Keep all concrete recovery safeguards below:
    #   * GNSS mode has already been reconciled above.
    #   * TIMEBASE PUBSUB routing is verified immediately below.
    #   * CLOCKS.RECOVER itself must return an explicitly accepted firmware
    #     lifecycle status in _request_teensy_recover().
    #
    # START, Flash Cut, and zero-row cold recovery remain preflight-gated.
    _diag["last_preflight_wait"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "context": "recovery",
        "status": "BYPASSED",
        "checks": 0,
        "waited_s": 0.0,
        "bypass_reason": "warm_recovery_after_teensy_reboot",
    }
    logging.info(
        "%s bypassing START admission gate for warm recovery; "
        "CLOCKS.RECOVER firmware verdict remains authoritative",
        PREFLIGHT_LOG_PREFIX,
    )

    _wait_for_timebase_routes(context="recovery")

    # ------------------------------------------------------------------
    # Quiesce Pi ingress without stopping the always-on Teensy clock lanes
    # ------------------------------------------------------------------
    #
    # A live TIMEBASE blackout is a publication/control-plane failure, not an
    # operator STOP.  CLOCKS.RECOVER is itself the firmware lifecycle boundary:
    # it enters RECOVERING at command acceptance and consumes the recovered
    # bases at the next PPS/VCLOCK edge.  Sending STOP here can strand a
    # still-running Teensy's OCXO subscriber/cadence service before RECOVER has
    # a chance to reattach it.  Reserve STOP/RECOVER_ABORT for explicit failed
    # recovery cleanup; warm recovery goes directly from the last durable row
    # to RECOVER, matching the post-flash/power-cycle transaction.
    logging.info(
        "📡 [recovery] @%s preserving Teensy clock service; "
        "quiescing Pi TIMEBASE ingress before direct RECOVER...",
        system_time_z(),
    )

    _drained = _drain_timebase_ingress()
    if _drained > 0:
        logging.info("🧹 [recovery] drained %d stale TIMEBASE candidate(s)", _drained)

    _reassert_system_dither("recovery", system_cfg)

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
    # Seed RECOVER at the campaign base count for the current GNSS second.
    # The first public row should then be the next PPS/VCLOCK identity. No
    # fixed skipped-row policy is modeled Pi-side anymore.
    recover_base_pps_vclock_count = last_pps_vclock_count + elapsed_seconds
    expected_first_public_pps_vclock_count = (
        recover_base_pps_vclock_count + RECOVERY_FIRST_PUBLIC_OFFSET
    )

    projected_gnss_ns = recover_base_pps_vclock_count * NS_PER_SECOND
    expected_first_public_gnss_ns = (
        expected_first_public_pps_vclock_count * NS_PER_SECOND
    )
    projected_dwt_cycles = (
        projected_gnss_ns * last_dwt_cycles // last_gnss_ns
        if (last_gnss_ns > 0 and last_dwt_cycles > 0)
        else 0
    )
    projected_dwt_ns = (
        _dwt_cycles_to_recover_ns(projected_dwt_cycles)
        if projected_dwt_cycles > 0
        else (projected_gnss_ns * last_dwt_ns // last_gnss_ns if last_gnss_ns > 0 else projected_gnss_ns)
    )
    projected_ocxo1_ns = projected_gnss_ns * last_ocxo1_ns // last_gnss_ns if (last_gnss_ns > 0 and last_ocxo1_ns > 0) else 0
    projected_ocxo2_ns = projected_gnss_ns * last_ocxo2_ns // last_gnss_ns if (last_gnss_ns > 0 and last_ocxo2_ns > 0) else 0

    # GNSS_RAW is Pi-owned and has its own reference ledger.  Do not project it
    # as last_gnss_raw_ns / last_gnss_ns; that preserves any poisoned
    # clockface/GNSS ratio across every recovery.  The actual seed immediately
    # before the first accepted public row is computed below after the clean
    # row is known.
    projected_gnss_raw_ns = 0
    projected_gnss_raw_ref_ns = 0

    tau_dwt = (
        last_dwt_cycles / ((last_gnss_ns * DWT_EXPECTED_PER_PPS) / NS_PER_SECOND)
        if last_gnss_ns > 0 and last_dwt_cycles > 0
        else 1.0
    )
    tau_ocxo1 = last_ocxo1_ns / last_gnss_ns if (last_gnss_ns > 0 and last_ocxo1_ns > 0) else 1.0
    tau_ocxo2 = last_ocxo2_ns / last_gnss_ns if (last_gnss_ns > 0 and last_ocxo2_ns > 0) else 1.0

    logging.info(
        "📐 [recovery] projection: last=%d elapsed=%d base=%d first_public=%d; "
        "gnss_base=%d dwt_cycles=%d ocxo1=%d ocxo2=%d",
        last_pps_vclock_count,
        elapsed_seconds,
        recover_base_pps_vclock_count,
        expected_first_public_pps_vclock_count,
        projected_gnss_ns,
        projected_dwt_cycles,
        projected_ocxo1_ns,
        projected_ocxo2_ns,
    )

    _diag["last_recovery"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "campaign": campaign_name,
        "last_pps_vclock_count": int(last_pps_vclock_count),
        "last_gnss_time": last_gnss_time_str,
        "current_gnss_time": current_gnss_time_str,
        "elapsed_seconds": int(elapsed_seconds),
        "recover_base_pps_vclock_count": int(recover_base_pps_vclock_count),
        "expected_first_public_pps_vclock_count": int(expected_first_public_pps_vclock_count),
        "expected_first_public_gnss_ns": int(expected_first_public_gnss_ns),
        # Legacy diagnostic alias: expected first public row, not the RECOVER seed.
        "next_pps_vclock_count": int(expected_first_public_pps_vclock_count),
        "projected_gnss_ns": int(projected_gnss_ns),
        "last_dwt_cycles": int(last_dwt_cycles),
        "projected_dwt_cycles": int(projected_dwt_cycles),
        "projected_dwt_ns": int(projected_dwt_ns),
        "projected_ocxo1_ns": int(projected_ocxo1_ns),
        "projected_ocxo2_ns": int(projected_ocxo2_ns),
        "last_gnss_raw_ns": int(last_gnss_raw_ns),
        "last_gnss_raw_ref_ns": int(last_gnss_raw_ref_ns),
        "last_gnss_raw_welford_mean": round(float(last_gnss_raw_welford_mean), 6),
        "skipped_unrecoverable_rows": int(skipped_unrecoverable_rows),
        "recover_ocxo1_dac": None if recover_ocxo1_dac is None else round(float(recover_ocxo1_dac), 6),
        "recover_ocxo2_dac": None if recover_ocxo2_dac is None else round(float(recover_ocxo2_dac), 6),
        "recover_calibrate_ocxo": recover_calibrate,
        "teensy_stop_sent": False,
        "teensy_recover_abort_sent": False,
        "interrupt_service_preserved": True,
        "tau_dwt": round(tau_dwt, 12),
        "tau_ocxo1": round(tau_ocxo1, 12),
        "tau_ocxo2": round(tau_ocxo2, 12),
    }

    # ------------------------------------------------------------------
    # Step 6: Begin sync wait, arm Teensy RECOVER
    # ------------------------------------------------------------------
    _begin_sync_wait(expected_pps=int(expected_first_public_pps_vclock_count))

    logging.info(
        "📡 [recovery] arming Teensy RECOVER: base_count=%d expected_first_public=%d",
        recover_base_pps_vclock_count, expected_first_public_pps_vclock_count,
    )

    teensy_recover_args: Dict[str, Any] = {
        "campaign": campaign_name,
        "dwt_ns": str(int(projected_dwt_ns)),
        "gnss_ns": str(int(projected_gnss_ns)),
        "ocxo1_ns": str(int(projected_ocxo1_ns)),
        "ocxo2_ns": str(int(projected_ocxo2_ns)),
    }

    recovered_welford_count = _add_welford_recovery_args(
        teensy_recover_args,
        last_tb=last_tb,
        last_frag=last_frag,
    )
    logging.info(
        "📊 [recovery] restored %d Teensy Welford accumulator payload(s) into RECOVER args",
        recovered_welford_count,
    )

    if recover_ocxo1_dac is not None:
        teensy_recover_args["set_dac1"] = str(float(recover_ocxo1_dac))
    if recover_ocxo2_dac is not None:
        teensy_recover_args["set_dac2"] = str(float(recover_ocxo2_dac))
    if recover_calibrate:
        teensy_recover_args["calibrate_ocxo"] = str(recover_calibrate)

    teensy_recover_resp = _request_teensy_recover(
        int(recover_base_pps_vclock_count),
        teensy_recover_args,
    )
    teensy_recover_payload = (
        teensy_recover_resp.get("payload", {})
        if isinstance(teensy_recover_resp, dict)
        else {}
    )

    recovery_monitor = {
        "campaign": campaign_name,
        "recover_base_pps_vclock_count": int(recover_base_pps_vclock_count),
        "expected_first_public_pps_vclock_count": int(expected_first_public_pps_vclock_count),
        "recovery_generation": _as_int(teensy_recover_payload.get("recovery_generation")),
        "recover_status": teensy_recover_payload.get("status"),
        "sent_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "sent_monotonic": time.monotonic(),
    }

    admission_wait_deadline = time.monotonic() + SYNC_RECOVER_CLEAN_TIMEOUT_S
    discarded_transitional_rows = 0
    recovery_admission_verdict: Dict[str, Any] = {}

    while True:
        remaining = admission_wait_deadline - time.monotonic()
        if remaining <= 0:
            reason = "recovery_timeline_admission_timeout"
            details = {
                "campaign": campaign_name,
                "expected_first_public_pps_vclock_count": int(expected_first_public_pps_vclock_count),
                "discarded_transitional_rows": int(discarded_transitional_rows),
                "last_admission_verdict": recovery_admission_verdict,
                "admission_timeout_s": float(SYNC_RECOVER_CLEAN_TIMEOUT_S),
            }
            _diag["recovery_clean_timeouts"] = _diag.get("recovery_clean_timeouts", 0) + 1
            _diag["last_recovery_clean_timeout"] = {"reason": reason, **details}
            _cleanup_after_recovery_failure(reason, details)
            raise RecoveryCleanTimeout(reason, details, cleanup_sent=True)

        try:
            frag, waited_s = _end_sync_wait(
                timeout_s=min(float(remaining), float(RECOVERY_FIRST_ROW_TIMEOUT_S)),
                recovery_monitor=recovery_monitor,
            )
        except RecoverySyncTimeout as e:
            reason = (
                "recovery_timeline_admission_timeout"
                if discarded_transitional_rows
                else e.reason
            )
            details = {
                **e.details,
                "campaign": campaign_name,
                "recover_base_pps_vclock_count": int(recover_base_pps_vclock_count),
                "expected_first_public_pps_vclock_count": int(expected_first_public_pps_vclock_count),
                "discarded_transitional_rows": int(discarded_transitional_rows),
                "last_admission_verdict": recovery_admission_verdict,
            }
            if discarded_transitional_rows:
                _diag["recovery_clean_timeouts"] = _diag.get("recovery_clean_timeouts", 0) + 1
                _diag["last_recovery_clean_timeout"] = {"reason": reason, **details}
                _cleanup_after_recovery_failure(reason, details)
                raise RecoveryCleanTimeout(reason, details, cleanup_sent=True) from e

            _cleanup_after_recovery_failure(reason, details)
            raise RecoverySyncTimeout(reason, details, cleanup_sent=True) from e

        _raise_if_recovery_interrupted("post_sync_row_before_admission_verdict")

        teensy_pps_vclock_count = _extract_teensy_pps_vclock_count(frag)
        first_public_offset = teensy_pps_vclock_count - expected_first_public_pps_vclock_count
        recovery_status = _fetch_teensy_recovery_status()
        admissible, recovery_admission_verdict = _recovery_admission_verdict(
            fragment=frag,
            status=recovery_status,
            pps_vclock_count=teensy_pps_vclock_count,
        )

        _diag["last_recovery_admission"] = recovery_admission_verdict
        if admissible:
            if recovery_admission_verdict.get("fully_clean"):
                _diag["recovery_science_clean_rows_admitted"] = (
                    _diag.get("recovery_science_clean_rows_admitted", 0) + 1
                )
                logging.info(
                    "✅ [recovery] clean public row admitted: count=%d expected=%d offset=%+d waited=%.3fs discarded=%d",
                    teensy_pps_vclock_count,
                    expected_first_public_pps_vclock_count,
                    first_public_offset,
                    waited_s,
                    discarded_transitional_rows,
                )
            else:
                _diag["recovery_degraded_rows_admitted"] = (
                    _diag.get("recovery_degraded_rows_admitted", 0) + 1
                )
                logging.warning(
                    "⚠️ [recovery] truthful degraded timeline row admitted: "
                    "count=%d expected=%d offset=%+d state=%s; OCXO science remains gated",
                    teensy_pps_vclock_count,
                    expected_first_public_pps_vclock_count,
                    first_public_offset,
                    recovery_admission_verdict.get("state_reasons"),
                )
            break

        discarded_transitional_rows += 1
        _diag["recovery_transitional_rows_discarded"] = (
            _diag.get("recovery_transitional_rows_discarded", 0) + 1
        )
        _diag["last_recovery_transitional_row"] = recovery_admission_verdict
        logging.warning(
            "⚠️ [recovery] discarding non-admissible row count=%d expected=%d "
            "offset=%+d blockers=%s state=%s status_reason=%s",
            teensy_pps_vclock_count,
            expected_first_public_pps_vclock_count,
            first_public_offset,
            recovery_admission_verdict.get("blocking_reasons"),
            recovery_admission_verdict.get("state_reasons"),
            recovery_admission_verdict.get("report_reason"),
        )

        # Release the processor thread to discard this row while campaign
        # ingestion is still closed, then arm the next accepted-row sync wait.
        _sync_resume_event.set()
        time.sleep(0.05)
        _begin_sync_wait(expected_pps=int(teensy_pps_vclock_count) + 1)

    # The processor is paused on this exact row. Restore Pi-owned state to
    # the identity immediately before it, then reopen campaign processing so
    # the sync row is persisted as the first recovered public TIMEBASE row.
    seed_pps_vclock_count = max(0, int(teensy_pps_vclock_count) - 1)
    seed_gnss_ns = seed_pps_vclock_count * NS_PER_SECOND
    gnss_raw_projection = _gnss_raw_recovery_project_seed(
        last_tb=last_tb,
        last_pps_vclock_count=int(last_pps_vclock_count),
        seed_pps_vclock_count=int(seed_pps_vclock_count),
    )
    seed_gnss_raw_ns = int(gnss_raw_projection.get("seed_raw_ns") or 0)
    seed_gnss_raw_ref_ns = int(gnss_raw_projection.get("seed_ref_ns") or seed_gnss_ns)

    gnss_raw_restored = _restore_gnss_raw_from_last_timebase(
        last_tb=last_tb,
        projected_gnss_ns=int(seed_gnss_ns),
        projected_gnss_raw_ns=int(seed_gnss_raw_ns),
        projected_gnss_raw_ref_ns=int(seed_gnss_raw_ref_ns),
        projection_details=gnss_raw_projection,
    )

    _accepted_pps_vclock_count = seed_pps_vclock_count
    _last_pps_vclock_count_seen = seed_pps_vclock_count
    _diag["accepted_pps_count"] = _accepted_pps_vclock_count
    _diag["accepted_pps_vclock_count"] = _accepted_pps_vclock_count

    _gnss_canary_reset()

    _diag["last_recovery"].update({
        "accepted_pps_vclock_count": int(teensy_pps_vclock_count),
        "seed_pps_vclock_count": int(seed_pps_vclock_count),
        "seed_gnss_ns": int(seed_gnss_ns),
        "seed_gnss_raw_ns": int(seed_gnss_raw_ns),
        "seed_gnss_raw_ref_ns": int(seed_gnss_raw_ref_ns),
        "gnss_raw_projection": gnss_raw_projection,
        "first_public_offset": int(first_public_offset),
        "skipped_records_expected": False,
        "gnss_raw_recovery_uses_seed_before_first_public": True,
        "clean_recovery_verdict": recovery_admission_verdict,
        "recovery_admission_verdict": recovery_admission_verdict,
        "recovery_fully_clean_at_admission": bool(recovery_admission_verdict.get("fully_clean")),
        "discarded_transitional_rows": int(discarded_transitional_rows),
        "gnss_raw_restored": bool(gnss_raw_restored),
    })

    _campaign_active = True
    _arm_timebase_silence_watch("RECOVERY_RESUME")
    _sync_resume_event.set()

    logging.info(
        "📊 [recovery] GNSS_RAW restored=%s seed_count=%d ref_ns=%d raw_ns=%d raw_ppb=%.6f source=%s first_public=%d offset=%+d",
        str(gnss_raw_restored),
        seed_pps_vclock_count,
        seed_gnss_raw_ref_ns,
        seed_gnss_raw_ns,
        float(gnss_raw_projection.get("seed_raw_ppb") or 0.0),
        gnss_raw_projection.get("source"),
        teensy_pps_vclock_count,
        first_public_offset,
    )
    logging.info(
        "✅ [recovery] campaign '%s' timeline recovered — TIMEBASE resumes with "
        "first public count=%d science_clean=%s",
        campaign_name,
        teensy_pps_vclock_count,
        bool(recovery_admission_verdict.get("fully_clean")),
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
                payload = row["payload"]
                return {
                    "baseline_id": payload["baseline_id"],
                    "baseline_ppb": payload.get("baseline_ppb", {}),
                    "baseline_tau": payload.get("baseline_tau", {}),
                    "baseline_dac": payload.get("baseline_dac", {}),
                    "baseline_dac_mean": payload.get("baseline_dac_mean", {}),
                    "baseline_dac_stats": payload.get("baseline_dac_stats", {}),
                    "baseline_campaign": payload.get("baseline_campaign"),
                    "baseline_pps_vclock_n": payload.get("baseline_pps_vclock_n"),
                    "baseline_pps_n": payload.get("baseline_pps_n"),
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
    if not isinstance(report, dict) or not report:
        return {"success": False, "message": f"Campaign {baseline_id} ('{row['campaign']}') has no report"}

    baseline_ppb = _baseline_ppb_from_report(report)
    if not baseline_ppb:
        return {"success": False, "message": f"Campaign {baseline_id} ('{row['campaign']}') report has no PPB data"}

    baseline_tau = _baseline_tau_from_report(report)
    baseline_dac = _baseline_dac_from_report(report)
    baseline_dac_mean = _baseline_dac_mean_from_report(report)
    baseline_dac_stats = _baseline_dac_stats_from_report(report)

    baseline_pps_vclock_n = _extract_last_timebase_count(report, _report_fragment(report))

    # A baseline should be self-contained, but it should also update the
    # boot DAC defaults.  Prefer firmware DAC means for startup setpoints;
    # fall back to instantaneous DAC values.
    ocxo1_boot_dac = _first_float(baseline_dac_mean.get("ocxo1"), baseline_dac.get("ocxo1"))
    ocxo2_boot_dac = _first_float(baseline_dac_mean.get("ocxo2"), baseline_dac.get("ocxo2"))

    baseline_blob: Dict[str, Any] = {
        "baseline_id": baseline_id,
        "baseline_ppb": baseline_ppb,
        "baseline_tau": baseline_tau,
        "baseline_dac": baseline_dac,
        "baseline_dac_mean": baseline_dac_mean,
        "baseline_dac_stats": baseline_dac_stats,
        "baseline_campaign": row["campaign"],
        "baseline_pps_vclock_n": baseline_pps_vclock_n,
        "baseline_pps_n": baseline_pps_vclock_n,  # legacy alias
    }
    if ocxo1_boot_dac is not None:
        baseline_blob["ocxo1_dac"] = round(ocxo1_boot_dac, 6)
    if ocxo2_boot_dac is not None:
        baseline_blob["ocxo2_dac"] = round(ocxo2_boot_dac, 6)

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
        "✅ [clocks] baseline set: id=%d campaign='%s' ppb=%s dac_mean=%s",
        baseline_id, row["campaign"], baseline_ppb, baseline_dac_mean,
    )
    return {"success": True, "message": "OK", "payload": baseline_blob}

# ---------------------------------------------------------------------
# Feature-status preflight gate
# ---------------------------------------------------------------------


def _fetch_system_features() -> Dict[str, Any]:
    """Fetch the global feature tree from Pi SYSTEM.  Command/response only."""
    resp = send_command(machine="PI", subsystem="SYSTEM", command="FEATURES")
    if not resp.get("success"):
        raise RuntimeError(f"PI SYSTEM FEATURES failed: {resp.get('message', '?')}")

    payload = resp.get("payload", {})
    if not isinstance(payload, dict):
        raise RuntimeError("PI SYSTEM FEATURES returned non-dict payload")

    return payload


def _feature_gate_reason(blocker: Dict[str, Any]) -> str:
    name = str(blocker.get("name") or "?")
    status = str(blocker.get("status") or "HOLD")
    detail = str(blocker.get("detail") or "").strip()
    if detail:
        return f"{name} is {status} ({detail})"
    return f"{name} is {status}"


def _check_feature_preflight(
    context: str,
    *,
    direct_teensy_gate_open: bool = False,
) -> tuple[bool, list[str]]:
    """Check the standardized feature-status campaign preflight profile.

    PI.SYSTEM.FEATURES is authoritative for Pi-local prerequisites.  Its
    TEENSY subtree is an asynchronously imported mirror, so it may temporarily
    retain INITIALIZING after the live Teensy gate has already opened.  When
    CLOCKS.REPORT_GATE directly proves the exact firmware admission gate open,
    stale TEENSY.* mirror blockers become diagnostic evidence rather than a
    second veto.
    """
    _diag["preflight_feature_checks"] += 1

    try:
        features = _fetch_system_features()
    except Exception as e:
        _diag["preflight_feature_unavailable"] += 1
        _diag["last_preflight_feature_gate"] = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "context": context,
            "profile": FEATURE_PREFLIGHT_PROFILE,
            "status": "UNAVAILABLE",
            "direct_teensy_gate_open": bool(direct_teensy_gate_open),
            "error": str(e),
        }
        return False, [f"{FEATURE_PREFLIGHT_PROFILE}: feature tree unavailable ({e})"]

    raw_blockers = blocking_features(features, FEATURE_PREFLIGHT_REQUIRED)
    compact_raw_blockers = [
        {
            "name": str(b.get("name") or "?"),
            "status": str(b.get("status") or "HOLD"),
            "detail": str(b.get("detail") or ""),
        }
        for b in raw_blockers
    ]

    bypassed_teensy_blockers: list[Dict[str, str]] = []
    compact_blockers = compact_raw_blockers
    if direct_teensy_gate_open:
        bypassed_teensy_blockers = [
            blocker
            for blocker in compact_raw_blockers
            if str(blocker.get("name") or "").startswith("TEENSY.")
        ]
        compact_blockers = [
            blocker
            for blocker in compact_raw_blockers
            if not str(blocker.get("name") or "").startswith("TEENSY.")
        ]

        if bypassed_teensy_blockers:
            _diag["preflight_teensy_mirror_bypass_count"] = (
                _diag.get("preflight_teensy_mirror_bypass_count", 0) + 1
            )
            _diag["last_preflight_teensy_mirror_bypass"] = {
                "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "context": context,
                "authority": "TEENSY.CLOCKS.REPORT_GATE",
                "blockers": bypassed_teensy_blockers,
            }

    if compact_blockers:
        _diag["preflight_feature_blocked"] += 1
        _diag["last_preflight_feature_gate"] = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "context": context,
            "profile": FEATURE_PREFLIGHT_PROFILE,
            "status": "BLOCKED",
            "required_count": len(FEATURE_PREFLIGHT_REQUIRED),
            "direct_teensy_gate_open": bool(direct_teensy_gate_open),
            "blockers": compact_blockers,
            "raw_blockers": compact_raw_blockers,
            "bypassed_teensy_blockers": bypassed_teensy_blockers,
        }
        return False, [
            f"{FEATURE_PREFLIGHT_PROFILE}: {_feature_gate_reason(blocker)}"
            for blocker in compact_blockers
        ]

    _diag["last_preflight_feature_gate"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "context": context,
        "profile": FEATURE_PREFLIGHT_PROFILE,
        "status": "NOMINAL",
        "required_count": len(FEATURE_PREFLIGHT_REQUIRED),
        "direct_teensy_gate_open": bool(direct_teensy_gate_open),
        "raw_blockers": compact_raw_blockers,
        "bypassed_teensy_blockers": bypassed_teensy_blockers,
        "teensy_mirror_reconciled": bool(bypassed_teensy_blockers),
    }
    return True, []


def _check_teensy_campaign_gate(context: str) -> tuple[bool, list[str]]:
    """Read the exact Teensy campaign gate that CLOCKS.START will enforce."""
    _diag["teensy_campaign_gate_checks"] = (
        _diag.get("teensy_campaign_gate_checks", 0) + 1
    )
    now_utc = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")

    try:
        resp = send_command(
            machine="TEENSY",
            subsystem="CLOCKS",
            command="REPORT_GATE",
            retries=1,
            retry_delay_s=0.0,
        )
    except Exception as e:
        _diag["teensy_campaign_gate_unavailable"] = (
            _diag.get("teensy_campaign_gate_unavailable", 0) + 1
        )
        _diag["last_teensy_campaign_gate"] = {
            "ts_utc": now_utc,
            "context": context,
            "status": "UNAVAILABLE",
            "error": str(e),
        }
        return False, [f"TEENSY.CLOCKS.CAMPAIGN_GATE unavailable ({e})"]

    payload = resp.get("payload") if isinstance(resp, dict) else None
    if not isinstance(resp, dict) or not resp.get("success") or not isinstance(payload, dict):
        _diag["teensy_campaign_gate_unavailable"] = (
            _diag.get("teensy_campaign_gate_unavailable", 0) + 1
        )
        _diag["last_teensy_campaign_gate"] = {
            "ts_utc": now_utc,
            "context": context,
            "status": "UNAVAILABLE",
            "outer_message": resp.get("message") if isinstance(resp, dict) else None,
        }
        return False, ["TEENSY.CLOCKS.CAMPAIGN_GATE report unavailable"]

    gate_open = _recovery_bool(payload.get("campaign_gate_open"))
    gate_reason = str(payload.get("campaign_gate_reason") or "campaign gate closed")
    first_problem = str(payload.get("campaign_gate_first_problem") or "")
    first_problem_feature = TEENSY_CAMPAIGN_GATE_LABEL_TO_FEATURE.get(
        first_problem,
        f"TEENSY.CLOCKS.{first_problem or 'CAMPAIGN_GATE'}",
    )
    snapshot = {
        "ts_utc": now_utc,
        "context": context,
        "status": "NOMINAL" if gate_open else "BLOCKED",
        "campaign_gate_open": bool(gate_open),
        "campaign_gate_seen": _recovery_bool(payload.get("campaign_gate_seen")),
        "campaign_gate_reason": gate_reason,
        "campaign_gate_first_problem": first_problem,
        "campaign_gate_first_problem_feature": first_problem_feature,
        "campaign_gate_update_count": _as_int(payload.get("campaign_gate_update_count")),
        "campaign_gate_transition_count": _as_int(payload.get("campaign_gate_transition_count")),
    }
    _diag["last_teensy_campaign_gate"] = snapshot

    if gate_open:
        return True, []

    _diag["teensy_campaign_gate_blocked"] = (
        _diag.get("teensy_campaign_gate_blocked", 0) + 1
    )
    return False, [f"{first_problem_feature}: {gate_reason}"]


def _preflight_wait_items(reasons: list[str]) -> list[str]:
    """Return a compact, stable list of pending prerequisites for the log."""
    items: list[str] = []

    feature_gate = _diag.get("last_preflight_feature_gate") or {}
    blockers = feature_gate.get("blockers")
    blocker_names: set[str] = set()
    if isinstance(blockers, list):
        for blocker in blockers:
            if not isinstance(blocker, dict):
                continue
            name = str(blocker.get("name") or "?")
            blocker_names.add(name)
            status = str(blocker.get("status") or "HOLD")
            detail = str(blocker.get("detail") or "").strip()
            item = f"{name}={status}"
            if detail:
                item += f" ({detail})"
            items.append(item)

    teensy_gate = _diag.get("last_teensy_campaign_gate") or {}
    if teensy_gate.get("status") == "BLOCKED":
        feature = str(
            teensy_gate.get("campaign_gate_first_problem_feature")
            or "TEENSY.CLOCKS.CAMPAIGN_GATE"
        )
        reason = str(
            teensy_gate.get("campaign_gate_reason") or "campaign gate closed"
        )
        if feature not in blocker_names:
            items.append(f"{feature}=BLOCKED ({reason})")
    elif teensy_gate.get("status") == "UNAVAILABLE":
        items.append("TEENSY.CLOCKS.CAMPAIGN_GATE=UNAVAILABLE")

    for reason in reasons:
        if reason.startswith(f"{FEATURE_PREFLIGHT_PROFILE}:"):
            if not blockers:
                items.append(reason)
            continue
        reason_feature = reason.split(":", 1)[0]
        if reason_feature in blocker_names:
            continue
        if reason_feature == str(
            teensy_gate.get("campaign_gate_first_problem_feature") or ""
        ):
            continue
        items.append(reason)

    # Preserve order while removing duplicates.
    return list(dict.fromkeys(items))


# ---------------------------------------------------------------------
# Preflight gate — prerequisites for campaign start/recovery
# ---------------------------------------------------------------------


def _check_preflight(context: str = "campaign") -> tuple[bool, list[str]]:
    """
    Check whether the system is ready to start or recover a campaign.
    """
    reasons: list[str] = []

    # -----------------------------------------------------------------
    # 0. Exact Teensy CLOCKS admission gate
    # -----------------------------------------------------------------
    # Poll the firmware authority first.  The PI SYSTEM feature tree contains
    # an asynchronously imported TEENSY mirror; an open direct gate is allowed
    # to reconcile stale TEENSY.* INITIALIZING entries in that mirror.
    teensy_gate_ready, teensy_gate_reasons = _check_teensy_campaign_gate(context)
    if not teensy_gate_ready:
        reasons.extend(teensy_gate_reasons)

    # -----------------------------------------------------------------
    # 0b. Standardized feature-status readiness profile
    # -----------------------------------------------------------------
    feature_ready, feature_reasons = _check_feature_preflight(
        context,
        direct_teensy_gate_open=teensy_gate_ready,
    )
    if not feature_ready:
        reasons.extend(feature_reasons)

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
    """Wait quietly until every Pi and Teensy campaign gate is open.

    Readiness is polled frequently so startup proceeds promptly.  The log is
    intentionally sparse: no normal-path success line, and while blocked only
    one compact pending summary after the grace period, when the pending set
    changes, or once per status interval.
    """
    attempt = 0
    t0 = time.monotonic()
    last_log_at = t0
    last_signature: Optional[Tuple[str, ...]] = None
    logged_wait = False

    while True:
        ready, reasons = _check_preflight(context)
        now = time.monotonic()
        elapsed = now - t0

        if ready:
            _diag["last_preflight_wait"] = {
                "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "context": context,
                "status": "NOMINAL",
                "checks": int(attempt + 1),
                "waited_s": round(float(elapsed), 3),
            }
            if logged_wait:
                logging.info(
                    "%s prerequisites ready for %s after %.1fs",
                    PREFLIGHT_LOG_PREFIX, context, elapsed,
                )
            return

        attempt += 1
        pending = _preflight_wait_items(reasons)
        signature = tuple(pending)
        should_log = elapsed >= PREFLIGHT_QUIET_GRACE_S and (
            not logged_wait
            or signature != last_signature
            or now - last_log_at >= PREFLIGHT_STATUS_LOG_INTERVAL_S
        )

        _diag["last_preflight_wait"] = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "context": context,
            "status": "WAITING",
            "checks": int(attempt),
            "waited_s": round(float(elapsed), 3),
            "pending": pending,
        }

        if should_log:
            _diag["preflight_wait_log_count"] = (
                _diag.get("preflight_wait_log_count", 0) + 1
            )
            logging.info(
                "%s waiting for %s (%.0fs): %s",
                PREFLIGHT_LOG_PREFIX,
                context,
                elapsed,
                "; ".join(pending) if pending else "prerequisites pending",
            )
            logged_wait = True
            last_log_at = now
            last_signature = signature

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
        "baseline_tau": info.get("baseline_tau", {}),
        "baseline_dac": info.get("baseline_dac", {}),
        "baseline_dac_mean": info.get("baseline_dac_mean", {}),
        "baseline_dac_stats": info.get("baseline_dac_stats", {}),
        "baseline_campaign": info.get("baseline_campaign"),
        "baseline_pps_vclock_n": info.get("baseline_pps_vclock_n"),
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

    server_args = {
        "campaign": campaign_name
    }
    send_command(machine="SERVER", subsystem="SYSTEM", command="DELETE_CAMPAIGN", args=server_args)

    return {
        "success": True, "message": "OK",
        "payload": {
            "campaign": campaign_name,
            "campaigns_deleted": camp_count,
            "timebase_deleted": tb_count,
        },
    }


def cmd_truncate(args: Optional[dict]) -> Dict[str, Any]:
    """
    TRUNCATE

    Drop all local campaign history: every row in Postgres campaigns and
    timebase.  This is intentionally broader than DELETE(campaign).  It refuses
    to run while a campaign is active; STOP first so CLOCKS, Teensy, Postgres,
    and the server-side mirror all cross a clean lifecycle boundary.

    After the local Postgres truncate succeeds, forward TRUNCATE to ZPNet Server
    so its MongoDB timebase collection can be dropped too.
    """
    del args

    global _campaign_active, _accepted_pps_vclock_count
    global _start_waiting_for_first_fragment, _start_requested_campaign
    global _start_requested_at_utc, _start_requested_monotonic
    global _start_first_fragment_at_utc, _start_first_fragment_wait_s
    global _start_first_fragment_pps_vclock_count

    active_row = _get_active_campaign()
    if active_row is not None:
        return {
            "success": False,
            "message": f"Campaign '{active_row['campaign']}' is active — STOP it first",
        }

    # Ensure no process-local ingestion or recovery latch can persist a stale
    # TIMEBASE row after the tables have been emptied.  This mirrors the
    # recovery discipline: lower the campaign gate first, then drain custody.
    _campaign_active = False
    _clear_sync_wait()
    _clear_flash_cut_wait_state()
    drained = _reset_trackers()
    _accepted_pps_vclock_count = None
    _diag["accepted_pps_count"] = None
    _diag["accepted_pps_vclock_count"] = None

    _start_waiting_for_first_fragment = False
    _start_requested_campaign = None
    _start_requested_at_utc = None
    _start_requested_monotonic = None
    _start_first_fragment_at_utc = None
    _start_first_fragment_wait_s = None
    _start_first_fragment_pps_vclock_count = None
    _diag["start_waiting_for_first_fragment"] = False

    _request_teensy_stop_best_effort()

    try:
        with open_db() as conn:
            cur = conn.cursor()
            cur.execute("SELECT COUNT(*) FROM timebase")
            tb_count = int(cur.fetchone()[0])
            cur.execute("SELECT COUNT(*) FROM campaigns")
            camp_count = int(cur.fetchone()[0])
            cur.execute("TRUNCATE TABLE timebase, campaigns RESTART IDENTITY")
    except Exception as e:
        logging.exception("❌ [clocks] TRUNCATE failed")
        return {"success": False, "message": str(e)}

    logging.warning(
        "🧨 [clocks] TRUNCATE: dropped all campaign history — %d campaign row(s), %d timebase row(s); drained=%d",
        camp_count, tb_count, drained,
    )

    server_args = {
        "source": "CLOCKS.TRUNCATE",
        "postgres_campaigns_deleted": camp_count,
        "postgres_timebase_deleted": tb_count,
    }
    try:
        server_resp = send_command(machine="SERVER", subsystem="SYSTEM", command="TRUNCATE", args=server_args)
    except Exception as e:
        logging.exception("⚠️ [clocks] SERVER.SYSTEM.TRUNCATE failed after local truncate")
        server_resp = {"success": False, "message": str(e)}

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "campaigns_deleted": camp_count,
            "timebase_deleted": tb_count,
            "local_ingress_drained": drained,
            "server_truncate_success": bool(server_resp.get("success")) if isinstance(server_resp, dict) else False,
            "server_truncate_message": server_resp.get("message") if isinstance(server_resp, dict) else None,
            "server_truncate_payload": server_resp.get("payload") if isinstance(server_resp, dict) else None,
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
            "teensy_pps_vclock_count": report.get("teensy_pps_vclock_count") or report.get("pps_count"),
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
        "last_pps_vclock_count_seen": _last_pps_vclock_count_seen,
        "accepted_pps_vclock_count": _accepted_pps_vclock_count,
        "sync_expected_pps_vclock": _sync_expected_pps_vclock,
        "sync_expected_pps": _sync_expected_pps_vclock,
        "startup": _start_status_payload(),
        "feature_preflight": {
            "profile": FEATURE_PREFLIGHT_PROFILE,
            "required_features": list(FEATURE_PREFLIGHT_REQUIRED),
            "post_start_expected_features": list(FEATURE_PREFLIGHT_POST_START_EXPECTED),
            "teensy_admission_authority": "TEENSY.CLOCKS.REPORT_GATE",
            "pi_teensy_feature_mirror_role": (
                "diagnostic when direct gate is open; fallback blocker otherwise"
            ),
        },
        "timebase_silence_monitor": {
            "timeout_s": TIMEBASE_SILENCE_TIMEOUT_S,
            "start_first_fragment_timeout_s": START_FIRST_FRAGMENT_TIMEOUT_S,
            "flash_cut_first_fragment_timeout_s": FLASH_CUT_FIRST_FRAGMENT_TIMEOUT_S,
            "current_effective_timeout_s": _timebase_silence_timeout_for_current_state(),
            "poll_s": TIMEBASE_SILENCE_MONITOR_POLL_S,
            "health_retry_s": TEENSY_HEALTH_RETRY_S,
            "last_activity_utc": _timebase_last_activity_utc,
            "last_activity_topic": _timebase_last_activity_topic,
            "last_activity_pps_vclock_count": _timebase_last_activity_pps_vclock_count,
            "recovery_active": _timebase_silence_recovery_active,
        },
        "flash_cut": {
            "pending": bool(_flash_cut_pending),
            "from": _flash_cut_from_campaign,
            "to": _flash_cut_to_campaign,
            "requested_at_utc": _flash_cut_requested_at_utc,
            "first_fragment_timeout_s": FLASH_CUT_FIRST_FRAGMENT_TIMEOUT_S,
        },
        "diag": _diag,
    }
    return {"success": True, "message": "OK", "payload": payload}


# ============================================================================
# cmd_set_dac
# ============================================================================

def _arg_first_present(args: Dict[str, Any], *names: str) -> Tuple[Optional[str], Any]:
    """Return (matched_name, value) for the first present argument key."""
    lower_map = {str(k).lower(): k for k in args.keys()}
    for name in names:
        actual = lower_map.get(name.lower())
        if actual is not None:
            return str(actual), args[actual]
    return None, None


def _parse_dac_arg(args: Dict[str, Any], label: str, *aliases: str) -> Tuple[bool, Optional[float], Optional[str]]:
    """Parse a DAC argument with tolerant aliases."""
    matched, raw = _arg_first_present(args, *aliases)
    if matched is None:
        return False, None, None

    try:
        value = float(raw)
    except (ValueError, TypeError):
        raise ValueError(f"Invalid {label} value for '{matched}': {raw!r}")

    if value < 0.0 or value > 65535.0:
        raise ValueError(f"{label} value {value} out of range (0–65535)")

    return True, value, matched


def _system_config_upsert(update_blob: Dict[str, Any]) -> None:
    """Merge update_blob into the SYSTEM config row, creating it if absent."""
    with open_db() as conn:
        cur = conn.cursor()
        cur.execute(
            """
            WITH updated AS (
                UPDATE config
                SET payload = COALESCE(payload, '{}'::jsonb) || %s::jsonb
                WHERE config_key = 'SYSTEM'
                RETURNING 1
            )
            INSERT INTO config (config_key, payload)
            SELECT 'SYSTEM', %s::jsonb
            WHERE NOT EXISTS (SELECT 1 FROM updated)
            """,
            (json.dumps(update_blob), json.dumps(update_blob)),
        )


def _manual_dac_config_blob(
    *,
    dac1: Optional[float],
    dac2: Optional[float],
    input_aliases: Dict[str, str],
) -> Dict[str, Any]:
    """
    Build the SYSTEM config payload for an operator DAC override.

    SET_DAC owns ocxo*_dac as the explicit next-run boot seed.  It also updates
    baseline_dac_mean / baseline_dac so legacy baseline fallback paths cannot
    resurrect an older mean from a previous campaign.
    """
    now_utc = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")

    existing = _get_system_config()
    baseline_dac_mean = dict(existing.get("baseline_dac_mean") or {})
    baseline_dac = dict(existing.get("baseline_dac") or {})

    manual_args: Dict[str, float] = {}

    update_blob: Dict[str, Any] = {
        "manual_dac_override": True,
        "manual_dac_override_at_utc": now_utc,
        "manual_dac_override_source": "CLOCKS.SET_DAC",
    }

    if dac1 is not None:
        dac1_rounded = round(float(dac1), 6)
        update_blob["ocxo1_dac"] = dac1_rounded
        baseline_dac_mean["ocxo1"] = dac1_rounded
        baseline_dac["ocxo1"] = dac1_rounded
        manual_args["dac1"] = dac1_rounded

    if dac2 is not None:
        dac2_rounded = round(float(dac2), 6)
        update_blob["ocxo2_dac"] = dac2_rounded
        baseline_dac_mean["ocxo2"] = dac2_rounded
        baseline_dac["ocxo2"] = dac2_rounded
        manual_args["dac2"] = dac2_rounded

    update_blob["manual_dac_override_args"] = manual_args
    update_blob["manual_dac_override_aliases"] = input_aliases

    update_blob["baseline_dac_mean"] = baseline_dac_mean
    update_blob["baseline_dac"] = baseline_dac

    return update_blob


def _get_campaign_row_by_name(campaign_name: str) -> Optional[Dict[str, Any]]:
    """Return the newest campaign row with decoded payload, or None."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, campaign, active, ts, payload
            FROM campaigns
            WHERE campaign = %s
            ORDER BY ts DESC
            LIMIT 1
            """,
            (campaign_name,),
        )
        row = cur.fetchone()

    if row is None:
        return None

    payload = row["payload"]
    if isinstance(payload, str):
        payload = json.loads(payload)

    return {
        "id": row["id"],
        "campaign": row["campaign"],
        "active": bool(row["active"]),
        "ts": row.get("ts") if isinstance(row, dict) else None,
        "payload": payload if isinstance(payload, dict) else {},
    }


def _first_float_with_source(*candidates: Tuple[str, Any]) -> Tuple[Optional[float], Optional[str]]:
    """Return the first parseable float and its descriptive source path."""
    for source, value in candidates:
        parsed = _first_float(value)
        if parsed is not None:
            return parsed, source
    return None, None


def _campaign_report_dac_values(
    campaign_payload: Dict[str, Any],
) -> Tuple[Optional[float], Optional[float], Dict[str, Optional[str]]]:
    """Extract OCXO DAC values from the campaign's last report fragment.

    The intended authority is campaigns.payload.report.fragment.dac, which is
    the last TIMEBASE_FRAGMENT denormalized into the CAMPAIGNS row.  Small
    legacy fallbacks are retained so older campaign rows can still seed the
    boot DAC defaults, but the returned source map makes the provenance visible.
    """
    report = campaign_payload.get("report")
    report_obj = report if isinstance(report, dict) else {}
    fragment = report_obj.get("fragment")
    frag = fragment if isinstance(fragment, dict) else {}

    dac1, dac1_source = _first_float_with_source(
        ("campaign.report.fragment.dac.ocxo1_dac", _path_get(frag, "dac.ocxo1_dac")),
        ("campaign.report.fragment.dac.ocxo1.value", _path_get(frag, "dac.ocxo1.value")),
        ("campaign.report.fragment.ocxo1_dac", frag.get("ocxo1_dac")),
        ("campaign.report.ocxo1_dac", report_obj.get("ocxo1_dac")),
        ("campaign.payload.ocxo1_dac", campaign_payload.get("ocxo1_dac")),
    )
    dac2, dac2_source = _first_float_with_source(
        ("campaign.report.fragment.dac.ocxo2_dac", _path_get(frag, "dac.ocxo2_dac")),
        ("campaign.report.fragment.dac.ocxo2.value", _path_get(frag, "dac.ocxo2.value")),
        ("campaign.report.fragment.ocxo2_dac", frag.get("ocxo2_dac")),
        ("campaign.report.ocxo2_dac", report_obj.get("ocxo2_dac")),
        ("campaign.payload.ocxo2_dac", campaign_payload.get("ocxo2_dac")),
    )

    return dac1, dac2, {"ocxo1": dac1_source, "ocxo2": dac2_source}


def _campaign_dac_config_blob(
    *,
    campaign_name: str,
    campaign_id: int,
    dac1: float,
    dac2: float,
    source_paths: Dict[str, Optional[str]],
    input_aliases: Dict[str, str],
) -> Dict[str, Any]:
    """Build a SYSTEM config payload from a historical campaign DAC snapshot."""
    now_utc = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")

    existing = _get_system_config()
    baseline_dac_mean = dict(existing.get("baseline_dac_mean") or {})
    baseline_dac = dict(existing.get("baseline_dac") or {})

    dac1_rounded = round(float(dac1), 6)
    dac2_rounded = round(float(dac2), 6)
    baseline_dac_mean["ocxo1"] = dac1_rounded
    baseline_dac_mean["ocxo2"] = dac2_rounded
    baseline_dac["ocxo1"] = dac1_rounded
    baseline_dac["ocxo2"] = dac2_rounded

    return {
        "ocxo1_dac": dac1_rounded,
        "ocxo2_dac": dac2_rounded,
        "baseline_dac_mean": baseline_dac_mean,
        "baseline_dac": baseline_dac,
        "manual_dac_override": True,
        "manual_dac_override_at_utc": now_utc,
        "manual_dac_override_source": "CLOCKS.SET_DAC_CAMPAIGN",
        "manual_dac_override_campaign": campaign_name,
        "manual_dac_override_campaign_id": int(campaign_id),
        "manual_dac_override_args": {
            "campaign": campaign_name,
            "dac1": dac1_rounded,
            "dac2": dac2_rounded,
        },
        "manual_dac_override_aliases": input_aliases,
        "manual_dac_override_dac_source": source_paths,
    }


def _cmd_set_dac_from_campaign(
    *,
    campaign_name: str,
    campaign_alias: Optional[str],
) -> Dict[str, Any]:
    """Set SYSTEM boot DAC defaults from a historical campaign's last fragment."""
    campaign_name = str(campaign_name or "").strip()
    if not campaign_name:
        return {"success": False, "message": "SET_DAC campaign=... requires a non-empty campaign name"}

    try:
        row = _get_campaign_row_by_name(campaign_name)
    except Exception as e:
        logging.exception("❌ [clocks] SET_DAC campaign lookup failed")
        return {"success": False, "message": str(e)}

    if row is None:
        return {"success": False, "message": f"No campaign named '{campaign_name}'"}

    payload = row.get("payload") if isinstance(row, dict) else {}
    payload = payload if isinstance(payload, dict) else {}
    report = payload.get("report")
    if not isinstance(report, dict) or not report:
        return {
            "success": False,
            "message": f"Campaign '{campaign_name}' has no denormalized report / last TIMEBASE fragment",
        }

    dac1, dac2, source_paths = _campaign_report_dac_values(payload)
    if dac1 is None or dac2 is None:
        missing = []
        if dac1 is None:
            missing.append("ocxo1_dac")
        if dac2 is None:
            missing.append("ocxo2_dac")
        return {
            "success": False,
            "message": (
                f"Campaign '{campaign_name}' last TIMEBASE fragment does not contain "
                f"required DAC value(s): {', '.join(missing)}"
            ),
            "payload": {"source_paths": source_paths},
        }

    update_blob = _campaign_dac_config_blob(
        campaign_name=str(row["campaign"]),
        campaign_id=int(row["id"]),
        dac1=float(dac1),
        dac2=float(dac2),
        source_paths=source_paths,
        input_aliases={"campaign": campaign_alias or "campaign"},
    )

    try:
        _system_config_upsert(update_blob)
    except Exception as e:
        logging.exception("❌ [clocks] SET_DAC campaign SYSTEM config update failed")
        return {"success": False, "message": str(e)}

    result_payload = {
        "source_campaign": str(row["campaign"]),
        "source_campaign_id": int(row["id"]),
        "source_campaign_active": bool(row.get("active")),
        "ocxo1_dac": update_blob["ocxo1_dac"],
        "ocxo2_dac": update_blob["ocxo2_dac"],
        "source_paths": source_paths,
        "baseline_dac": update_blob.get("baseline_dac", {}),
        "baseline_dac_mean": update_blob.get("baseline_dac_mean", {}),
        "manual_dac_override": True,
        "manual_dac_override_source": update_blob["manual_dac_override_source"],
        "manual_dac_override_at_utc": update_blob["manual_dac_override_at_utc"],
        "applies_to": "next START/boot/recovery DAC seed only; running Teensy DAC is not changed",
        "teensy_pushed_now": False,
    }

    logging.info("🔧 [clocks] SET_DAC campaign override: %s", result_payload)
    return {"success": True, "message": "OK", "payload": result_payload}


def cmd_set_dac(args: Optional[dict]) -> Dict[str, Any]:
    """
    SET_DAC(DAC1=foo, DAC2=bar) or SET_DAC(campaign=Payload4)

    Persist explicit OCXO DAC boot seeds in the SYSTEM config record.

    Accepted direct aliases:
      DAC1, dac1, set_dac1, ocxo1_dac
      DAC2, dac2, set_dac2, ocxo2_dac

    Accepted campaign-source form:
      campaign=<historical campaign name>

    Direct DAC arguments update SYSTEM config and push the running Teensy
    best-effort.  The campaign-source form updates SYSTEM config only: it
    copies the DAC values from the campaign row's last denormalized
    TIMEBASE_FRAGMENT so the next START/boot/recovery uses those defaults.

    Both forms intentionally update baseline_dac_mean / baseline_dac too, so
    older fallback paths cannot resurrect an unwanted servo-derived value.
    """
    if not args:
        return {
            "success": False,
            "message": "SET_DAC requires DAC1/DAC2 or campaign argument",
        }

    campaign_alias, campaign_value = _arg_first_present(args, "campaign")

    try:
        has_dac1, dac1, alias1 = _parse_dac_arg(
            args, "DAC1", "dac1", "DAC1", "set_dac1", "SET_DAC1", "ocxo1_dac", "OCXO1_DAC"
        )
        has_dac2, dac2, alias2 = _parse_dac_arg(
            args, "DAC2", "dac2", "DAC2", "set_dac2", "SET_DAC2", "ocxo2_dac", "OCXO2_DAC"
        )
    except ValueError as e:
        return {"success": False, "message": str(e)}

    if campaign_alias is not None:
        if has_dac1 or has_dac2:
            return {
                "success": False,
                "message": "SET_DAC campaign=... cannot be combined with explicit DAC values",
            }
        return _cmd_set_dac_from_campaign(
            campaign_name=str(campaign_value),
            campaign_alias=campaign_alias,
        )

    if not has_dac1 and not has_dac2:
        return {
            "success": False,
            "message": "SET_DAC requires DAC1/DAC2 or campaign argument",
        }

    input_aliases: Dict[str, str] = {}
    if alias1:
        input_aliases["dac1"] = alias1
    if alias2:
        input_aliases["dac2"] = alias2

    update_blob = _manual_dac_config_blob(
        dac1=dac1 if has_dac1 else None,
        dac2=dac2 if has_dac2 else None,
        input_aliases=input_aliases,
    )

    try:
        _system_config_upsert(update_blob)
    except Exception as e:
        logging.exception("❌ [clocks] SET_DAC failed")
        return {"success": False, "message": str(e)}

    teensy_pushed_now = False
    teensy_push_error = None
    teensy_message = None
    try:
        teensy_args: Dict[str, Any] = {}
        if has_dac1 and dac1 is not None:
            teensy_args["set_dac1"] = str(float(dac1))
        if has_dac2 and dac2 is not None:
            teensy_args["set_dac2"] = str(float(dac2))
        if teensy_args:
            resp = send_command(machine="TEENSY", subsystem="CLOCKS", command="SET_DAC", args=teensy_args)
            teensy_pushed_now = bool(resp.get("success"))
            teensy_message = resp.get("message")
    except Exception as e:
        teensy_push_error = str(e)
        logging.exception("⚠️ [clocks] SET_DAC persisted but Teensy push failed")

    payload = {
        "ocxo1_dac": update_blob.get("ocxo1_dac"),
        "ocxo2_dac": update_blob.get("ocxo2_dac"),
        "baseline_dac": update_blob.get("baseline_dac", {}),
        "baseline_dac_mean": update_blob.get("baseline_dac_mean", {}),
        "manual_dac_override": True,
        "manual_dac_override_args": update_blob.get("manual_dac_override_args", {}),
        "manual_dac_override_aliases": update_blob.get("manual_dac_override_aliases", {}),
        "manual_dac_override_at_utc": update_blob["manual_dac_override_at_utc"],
        "applies_to": "current Teensy desired DAC and next START/boot/recovery DAC seed",
        "teensy_pushed_now": teensy_pushed_now,
        "teensy_message": teensy_message,
        "teensy_push_error": teensy_push_error,
        "dither_realizes_desired": True,
    }

    logging.info("🔧 [clocks] SET_DAC boot seed override: %s", payload)
    return {"success": True, "message": "OK", "payload": payload}


def cmd_set_dither(args: Optional[dict]) -> Dict[str, Any]:
    """
    DITHER(dither, rate_hz=None)

    Update the global OCXO dithering flag and optional dither frequency in the
    SYSTEM config record.  This is a system-level setting, not a campaign
    attribute.
    """
    if not args or "dither" not in args:
        return {"success": False, "message": "DITHER requires 'dither' argument"}

    raw = args["dither"]
    if isinstance(raw, bool):
        dither = raw
    elif isinstance(raw, str):
        lowered = raw.strip().lower()
        if lowered in ("true", "1", "yes", "on"):
            dither = True
        elif lowered in ("false", "0", "no", "off"):
            dither = False
        else:
            return {"success": False, "message": f"Invalid dither value: {raw}"}
    else:
        return {"success": False, "message": f"Invalid dither value: {raw}"}

    rate_hz = None
    for key in ("rate_hz", "hz", "frequency_hz"):
        if key in args and args.get(key) is not None:
            try:
                rate_hz = int(args.get(key))
            except (TypeError, ValueError):
                return {"success": False, "message": f"Invalid dither rate for {key}: {args.get(key)!r}"}
            break

    if rate_hz is not None and not (1 <= rate_hz <= 1000):
        return {"success": False, "message": f"dither rate_hz must be 1..1000, got {rate_hz}"}

    update_blob = {"dither": dither}
    if rate_hz is not None:
        update_blob["dither_rate_hz"] = int(rate_hz)

    try:
        with open_db() as conn:
            cur = conn.cursor()
            cur.execute(
                """
                WITH updated AS (
                    UPDATE config
                    SET payload = COALESCE(payload, '{}'::jsonb) || %s::jsonb
                    WHERE config_key = 'SYSTEM'
                    RETURNING 1
                )
                INSERT INTO config (config_key, payload)
                SELECT 'SYSTEM', %s::jsonb
                WHERE NOT EXISTS (SELECT 1 FROM updated)
                """,
                (json.dumps(update_blob), json.dumps(update_blob)),
            )
    except Exception as e:
        logging.exception("❌ [clocks] DITHER failed")
        return {"success": False, "message": str(e)}

    try:
        teensy_args: Dict[str, Any] = {"dither": dither}
        if rate_hz is not None:
            teensy_args["rate_hz"] = int(rate_hz)
        teensy_resp = send_command(
            machine="TEENSY",
            subsystem="CLOCKS",
            command="DITHER",
            args=teensy_args,
        )
    except Exception as e:
        logging.exception("⚠️ [clocks] DITHER persisted but Teensy update failed")
        return {
            "success": False,
            "message": f"SYSTEM config updated but Teensy DITHER command failed: {e}",
            "payload": update_blob,
        }

    logging.info("🔧 [clocks] DITHER: %s resp=%s", update_blob, teensy_resp.get("message", "?"))
    return {
        "success": True,
        "message": "OK",
        "payload": {
            **update_blob,
            "teensy_message": teensy_resp.get("message"),
            "teensy_payload": teensy_resp.get("payload"),
        },
    }


COMMANDS = {
    "START": cmd_start,
    "STOP": cmd_stop,
    "RESUME": cmd_resume,
    "RECOVER_ABORT": cmd_recover_abort,
    "RECOVERY_ABORT": cmd_recover_abort,
    "REPORT": cmd_report,
    "CLEAR": cmd_clear,
    "DELETE": cmd_delete,
    "TRUNCATE": cmd_truncate,
    "SET_DAC": cmd_set_dac,
    "DITHER": cmd_set_dither,
    "SET_BASELINE": cmd_set_baseline,
    "BASELINE_INFO": cmd_baseline_info,
    "LIST_CAMPAIGNS": cmd_list_campaigns,
    "CLOCKS_INFO": cmd_clocks_info,
}

# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def startup_teensy_quiet_delay() -> None:
    """
    Let pubsub discover CLOCKS routes before boot pushes or campaign recovery.
    """
    logging.info(
        "⏳ [clocks] waiting %.1fs for pubsub routing and Teensy initialization "
        "before boot control-state push or campaign recovery",
        STARTUP_TEENSY_QUIET_DELAY_S,
    )
    time.sleep(STARTUP_TEENSY_QUIET_DELAY_S)
    logging.info(
        "✅ [clocks] startup quiet delay complete — boot control-state push "
        "and recovery may begin"
    )


def run() -> None:
    setup_logging()
    _setup_invalid_timebase_logger()

    _startup_control_ready.clear()
    _diag["startup_control_ready"] = False

    logging.info(
        "🕐 [clocks] unified PPS/VCLOCK TIMEBASE candidate schema. Teensy PPS/VCLOCK count is canonical. "
        "Four clock domains: GNSS (reference), DWT, OCXO1, OCXO2. "
        "The Teensy emits one TIMEBASE_FRAGMENT candidate containing fragment + embedded forensics; the Pi is the final row arbiter. "
        "Invalid candidates are logged in full and dropped as canonical gaps without automatic recovery. "
        "START while active performs seamless flash-cut to new campaign. "
        "Commands: START, STOP, RESUME, RECOVER_ABORT, REPORT, CLEAR, DELETE, TRUNCATE, SET_DAC, DITHER, "
        "SET_BASELINE, BASELINE_INFO, LIST_CAMPAIGNS, CLOCKS_INFO. "
        "Subscriptions: TIMEBASE_FRAGMENT, WATCHDOG_ANOMALY, CLOCKS_RECOVERY_STALLED."
    )

    # Start command + pubsub servers first, but hold off on active work.
    #
    # Pubsub can now query CLOCKS.SUBSCRIPTIONS and build routing while CLOCKS
    # avoids the dangerous post-flash behavior: boot DAC/DITHER pushes and
    # active campaign recovery colliding with other startup traffic.
    server_setup(
        subsystem="CLOCKS",
        commands=COMMANDS,
        subscriptions={
            "TIMEBASE_FRAGMENT": on_timebase_fragment,
            "WATCHDOG_ANOMALY": on_watchdog_anomaly,
            CLOCKS_RECOVERY_STALLED_TOPIC: on_recovery_stalled,
        },
        blocking=False,
    )

    startup_teensy_quiet_delay()

    # Foist global OCXO control configuration onto Teensy at boot.
    # Dither is a system-level setting, not campaign state, so apply it
    # before pushing calibrated DAC values. Then push the stored DACs so
    # the Teensy comes up in the correct control state before any campaign
    # starts or recovers.
    try:
        cfg = _get_system_config()

        boot_dither = cfg.get("dither")
        boot_dither_bool = None
        if isinstance(boot_dither, bool):
            boot_dither_bool = boot_dither
        elif isinstance(boot_dither, str):
            lowered = boot_dither.strip().lower()
            if lowered in ("true", "1", "yes", "on"):
                boot_dither_bool = True
            elif lowered in ("false", "0", "no", "off"):
                boot_dither_bool = False
        boot_rate_hz = None
        try:
            if cfg.get("dither_rate_hz") is not None:
                boot_rate_hz = int(cfg.get("dither_rate_hz"))
        except (TypeError, ValueError):
            boot_rate_hz = None
        if boot_dither_bool is not None or boot_rate_hz is not None:
            try:
                dither_args: Dict[str, Any] = {}
                if boot_dither_bool is not None:
                    dither_args["dither"] = boot_dither_bool
                else:
                    dither_args["dither"] = True
                if boot_rate_hz is not None:
                    dither_args["rate_hz"] = boot_rate_hz
                resp = send_command(
                    machine="TEENSY",
                    subsystem="CLOCKS",
                    command="DITHER",
                    args=dither_args,
                )
                logging.info(
                    "🔧 [clocks] boot dither push: dither=%s rate_hz=%s resp=%s",
                    boot_dither_bool, boot_rate_hz, resp.get("message", "?"),
                )
            except Exception:
                logging.exception("⚠️ [clocks] boot dither push failed (ignored)")

        boot_dac1, boot_dac2, boot_dac_source = _configured_boot_dacs(cfg)
        if boot_dac1 is not None or boot_dac2 is not None:
            dac_args: Dict[str, Any] = {}
            if boot_dac1 is not None:
                dac_args["set_dac1"] = str(float(boot_dac1))
            if boot_dac2 is not None:
                dac_args["set_dac2"] = str(float(boot_dac2))
            resp = send_command(
                machine="TEENSY", subsystem="CLOCKS", command="SET_DAC", args=dac_args,
            )
            logging.info(
                "🔧 [clocks] boot DAC push (%s): ocxo1=%s ocxo2=%s resp=%s",
                boot_dac_source, boot_dac1, boot_dac2, resp.get("message", "?"),
            )
        else:
            logging.info("🔧 [clocks] no DAC defaults or baseline DAC values in SYSTEM config — Teensy keeps defaults")
    except Exception:
        logging.exception("⚠️ [clocks] boot control-state push failed (Teensy keeps defaults)")

    # Start processor thread
    threading.Thread(
        target=_process_loop,
        daemon=True,
        name="clocks-processor",
    ).start()

    # Start live Teensy reboot / TIMEBASE silence monitor.
    threading.Thread(
        target=_timebase_silence_monitor_loop,
        daemon=True,
        name="clocks-timebase-silence-monitor",
    ).start()

    # Recover or stop stray Teensy. START/RESUME remain closed until this
    # reconciliation is complete, preventing a user START from creating a new
    # zero-row active campaign that this same boot path then mistakes for stale
    # recovery work.
    try:
        row = _get_active_campaign()
        if row is None:
            _request_teensy_stop_best_effort()
            try:
                effective_location = _ensure_gnss_mode_for_current_location()
                if effective_location:
                    logging.info(
                        "📡 [clocks] boot idle state — GNSS kept in TO mode for '%s'",
                        effective_location,
                    )
                else:
                    logging.info("📡 [clocks] boot idle state — GNSS in NORMAL mode")
            except Exception:
                logging.exception(
                    "⚠️ [clocks] failed to reconcile GNSS mode at boot idle"
                )
        else:
            try:
                _recover_campaign()
            except TeensyStartRejected as e:
                # _request_teensy_start already emitted the full handler evidence.
                logging.error(
                    "💥 [clocks] boot START rejected (%s); command interface remains available",
                    e.status,
                )
                _cleanup_after_recovery_failure(
                    "boot_start_rejected",
                    {"error": str(e), "status": e.status},
                )
            except Exception as e:
                logging.exception(
                    "💥 [clocks] boot recovery failed; CLOCKS command interface remains available"
                )
                _cleanup_after_recovery_failure(
                    "boot_recovery_failed",
                    {"error": str(e)},
                )
    except Exception as e:
        logging.exception(
            "💥 [clocks] boot lifecycle reconciliation failed; command interface remains available"
        )
        try:
            _cleanup_after_recovery_failure(
                "boot_lifecycle_reconciliation_failed",
                {"error": str(e)},
            )
        except Exception:
            logging.exception("⚠️ [clocks] boot lifecycle cleanup also failed")
    finally:
        _startup_control_ready.set()
        _diag["startup_control_ready"] = True
        logging.info(
            "✅ [clocks] startup control reconciliation complete — START/RESUME enabled"
        )

    # Block forever
    logging.info("🏁 [clocks] entering main loop")
    while True:
        time.sleep(3600)


if __name__ == "__main__":
    run()
