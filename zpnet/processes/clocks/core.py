"""
ZPNet CLOCKS Process — CLOCKS_FRAGMENT TEMPEST Ingress + Generalized Campaign Persistence (Pi-side)

Core contract:

  CLOCKS is the final TEMPEST candidate arbiter. It owns no Teensy clock
  state. It receives the always-on CLOCKS_FRAGMENT stream from the Teensy.
  During a TEMPEST campaign, CLOCKS_FRAGMENT carries an optional ``campaign``
  TEMPEST_FRAGMENT_V1 enrichment.  The always-on ``clocks`` object remains the
  sole instrument authority; the campaign object contains only campaign-relative
  identity/science.  CLOCKS adjudicates that delta and adds Pi-owned context
  without cloning the live instrument state.

  Architecture:
    • Teensy owns: physical PPS identity, GNSS/DWT/OCXO clockfaces,
      raw-cycle evidence, firmware statistics/control state, and TEMPEST science.
    • Pi owns: GNSS_RAW, GF-8802 correlation, environment correlation,
      campaign lifecycle, final acceptance court, recovery orchestration,
      and PostgreSQL persistence.
    • SYSTEM owns: current Pi/GNSS/environment/power context exposed by SYSTEM.REPORT.

  START behavior:

    START is asynchronous. The Pi creates/activates the campaign, sends the
    Teensy START command, and returns. The first public campaign row arrives
    later through the normal CLOCKS_FRAGMENT processor and becomes a typed
    TEMPEST campaign detail only after Pi adjudication. Firmware may privately
    acquire a lawful PPS0/start-prologue bookend before public PPS1; the Pi does
    not model those private candidates as skipped campaign rows.

  RECOVER behavior:

    RECOVER uses the last durable unified state detail with TEMPEST decoration as the public base, sends a
    recover command to the Teensy, waits for the first Pi-accepted public row,
    and restores Pi-owned GNSS_RAW/Welford state immediately before that row is
    persisted. Timeline rows may be admitted while OCXO science is explicitly
    degraded/quarantined, but the final court still rejects malformed or
    incoherent candidates.

Responsibilities:
  * Receive optional TEMPEST campaign enrichment from the unified CLOCKS_FRAGMENT stream.
  * Adjudicate each candidate: persist coherent audit rows and recover on structural loss.
  * Subscribe to predictive GF-8802 announcements and bind them to PPS-aligned rows.
  * Augment accepted rows with environment, GNSS_RAW, and system time.
  * Publish the live TIMEBASE topic and attach TEMPEST facts to unified state details.
  * Denormalize the latest accepted TEMPEST detail into the active campaign master.
  * Recover clocks after restart if a campaign is active.
  * Subscribe to WATCHDOG_ANOMALY and initiate Pi-side campaign recovery.
  * Flash-cut to a new campaign while preserving the hot Teensy stream.

Semantics:
  * No Pi-side smoothing, inference, or repair of Teensy clock state.
  * One physical second has one durable state detail; TEMPEST is a decoration.
  * Gaps, jumps, regressions, and science exclusions are recorded as evidence.
  * WATCHDOG_ANOMALY is an explicit Teensy continuity surrender and starts
    Pi-side recovery from the latest canonical TEMPEST campaign detail.
"""

from __future__ import annotations

import copy
import json
from collections import deque
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

CAMPAIGN_TYPE_TEMPEST = "TEMPEST"
CAMPAIGN_DETAIL_ATTACH_TIMEOUT_S = 5.0
CAMPAIGN_DETAIL_ATTACH_POLL_S = 0.05

# Teensy DWT conversion constants mirror pnc/firmware/teensy/config.h.
# RECOVER still accepts a dwt_ns command argument, but current
# TEMPEST_FRAGMENT_V1 publishes the DWT ledger in native cycles.
DWT_NS_NUM = 125
DWT_NS_DEN = 126
DWT_EXPECTED_PER_PPS = 1_008_000_000

GNSS_WAIT_LOG_INTERVAL = 60
GNSS_RAW_INFO_MAX_AGE_S = 2.5

# Sync waits
#
# Cold START, Flash Cut, and zero-row cold recovery are readiness-gated.
# Warm recovery uses its narrower recovery-specific lifecycle contract. The
# Pi no longer expects fixed row burial/warmup suppression during admission:
# the first public CLOCKS_FRAGMENT campaign delta is supposed to be useful, and if it is not,
# the responsible readiness or handoff gate should be fixed.
RECOVERY_FIRST_PUBLIC_OFFSET = 1
SYNC_FRAGMENT_TIMEOUT_S = 35.0
SYNC_RECOVER_TIMEOUT_S = 45.0
SYNC_POLL_S = 0.005

# Recovery may legitimately consume hidden firmware candidates while
# Alpha/CounterLedger/PhaseLedger proves fresh OCXO custody.  This now bounds
# the wait for the first truthful timeline-admissible row, not for fully
# mature OCXO science.
SYNC_RECOVER_CLEAN_TIMEOUT_S = 180.0

# A recovery that produces no accepted CLOCKS_FRAGMENT campaign delta inside this window is not
# merely "not clean yet".  Teensy should either publish a clean row or, after
# its bounded private reattach window, publish degraded rows that let the Pi
# observe liveness.  If the first row never appears, abort the firmware
# RECOVER lifecycle explicitly instead of recursively hard-faulting the Pi-side
# recovery thread.
RECOVERY_FIRST_ROW_TIMEOUT_S = 45.0

# A post-flash RECOVER may first need startup SmartZero to acquire and install a
# fresh local service epoch before firmware can launch the ordinary RECOVER grid
# rephase. Keep the normal live-reattach timeout tight, but allow the explicit
# COLD_BOOTSTRAP mode the full recovery-admission window.
RECOVERY_COLD_BOOTSTRAP_FIRST_ROW_TIMEOUT_S = 180.0

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
# The Teensy emits one CLOCKS_FRAGMENT_V4. ``clocks`` is canonical always-on
# instrument state; optional ``campaign`` is the TEMPEST-relative delta.  The
# outer sequence is physical-row identity while campaign.public_count is the
# campaign-relative recovery/publication count.
TIMEBASE_INGRESS_QUEUE_MAXSIZE = 0
CLOCKS_STATE_QUEUE_MAXSIZE = 3600
CLOCKS_STATE_RETRY_S = 0.25
TIMEBASE_FRAGMENT_TOPIC = "CLOCKS_FRAGMENT.campaign"
CLOCKS_FRAGMENT_TOPIC = "CLOCKS_FRAGMENT"
CLOCKS_TOPIC = "CLOCKS"
CLOCKS_BASELINE_REFRESH_S = 30.0
CLOCKS_PREFLIGHT_MAX_AGE_S = 5.0
CLOCKS_RECOVERY_STALLED_TOPIC = "CLOCKS_RECOVERY_STALLED"
TIMEBASE_CANDIDATE_ACCEPT = "ACCEPT"
TIMEBASE_CANDIDATE_SCIENCE_EXCLUDE = "SCIENCE_EXCLUDE"
TIMEBASE_CANDIDATE_DISPOSITIONS = {
    TIMEBASE_CANDIDATE_ACCEPT,
    TIMEBASE_CANDIDATE_SCIENCE_EXCLUDE,
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
TIMEBASE_FINAL_COURT_SOURCE = "PI_CLOCKS_FINAL_TEMPEST_COURT"
TIMEBASE_FINAL_COURT_VIOLATION_REASON = "tempest_final_court_violation"

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

# GNSS_RAW recovery sanity gate.  GNSS_RAW is a Pi-owned synthetic clockface
# accumulated from receiver drift_ppb.  Its accumulated tau/ppb should stay in
# the same order of magnitude as the restored drift Welford mean.  If an older
# recovery poisoned the synthetic clockface/reference ratio, rebuild the seed
# from the Welford mean instead of preserving the bad ratio.
GNSS_RAW_RECOVERY_REBUILD_GATE_PPB = 1000.0

# Preflight is polled quickly so START follows readiness promptly, but the log
# remains quiet.  Pending prerequisites are summarized only after a short grace
# period, when the pending set changes, or at the periodic status interval.
PREFLIGHT_POLL_INTERVAL_S = 2.0
PREFLIGHT_QUIET_GRACE_S = 15.0
PREFLIGHT_STATUS_LOG_INTERVAL_S = 30.0
PREFLIGHT_LOG_PREFIX = "🛡️ [preflight]"
STARTUP_TEENSY_QUIET_DELAY_S = 20.0
HOLISTIC_RESTORE_TIMEOUT_S = 60.0
HOLISTIC_RESTORE_COMMAND_RETRY_S = 10.0

# Better-Buckets restore replay. Alpha keeps exact-second endpoints for 10m and
# one first-admitted endpoint per minute for the longer windows. PostgreSQL is
# scanned only on recovery; ordinary CLOCKS remains lean.
PPB_REPLAY_10_MIN_SECONDS = 10 * 60
PPB_REPLAY_60_MIN_SECONDS = 60 * 60
PPB_REPLAY_8_HOUR_SECONDS = 8 * 60 * 60
PPB_REPLAY_24_HOUR_SECONDS = 24 * 60 * 60
PPB_REPLAY_MARGIN_SECONDS = 60
PPB_REPLAY_SECOND_CAPACITY = PPB_REPLAY_10_MIN_SECONDS + 1
PPB_REPLAY_MINUTE_CAPACITY = 24 * 60 + 2
PPB_REPLAY_CURSOR_ITERSIZE = 2048
PPB_RESTORE_CHUNK_ENDPOINTS = 4
PPB_REPLAY_VERIFY_TOLERANCE_PPB = 0.00001
TIMEBASE_SILENCE_TIMEOUT_S = 30.0
TIMEBASE_SILENCE_MONITOR_POLL_S = 1.0
TEENSY_HEALTH_RETRY_S = 60.0

# Async START/Flash Cut silence is not the same thing as a live campaign
# going dark. START/Flash Cut may still take several seconds while Teensy
# earns the first canonical row; the Pi waits patiently but expects to accept
# the first public row once it appears.
START_FIRST_FRAGMENT_TIMEOUT_S = 180.0
FLASH_CUT_FIRST_FRAGMENT_TIMEOUT_S = 180.0

# CLOCKS-backed campaign preflight.
#
# CLOCKS is the sole recurring readiness feed consumed by Pi CLOCKS.  The former
# FEATURE_STATUS / FEATURE_STATUS_FRAGMENT aggregate feeds no longer exist, so
# campaign admission must depend only on concrete readiness leaves carried in
# CLOCKS.features.  Runtime TIMEBASE row integrity, database command-contract
# checks, GNSS mode reconciliation, and recovery projection remain local CLOCKS
# logic.
#
# FLOORLINE and QTIMER_DWT_RULER are intentionally not campaign-admission
# gates.  Both remain valuable imported Teensy INTERRUPT feature surfaces, but
# they are diagnostic / quality witnesses whose current definitions can strobe
# during startup, recovery, and report-pressure windows.
#
# COUNTER32_LINEAGE and OCXO_PUBLIC_ORIGIN remain admission prerequisites because
# they protect physical identity/custody.  ALPHA_EPOCH subsumes SmartZero readiness:
# if SmartZero is scientifically blocking, ALPHA_EPOCH must not be NOMINAL.
# STATIC_PREDICTION remains post-start evidence.
#
# Pi CLOCKS owns the one global policy gate.  Teensy CLOCKS does not mirror or
# re-evaluate CLOCKS policy; it enforces command/state integrity, SmartZero,
# private PPS0/PhaseLedger maturity, watchdog custody, and the actual lifecycle
# command verdict.
FEATURE_PREFLIGHT_PROFILE = "CAMPAIGN_PREFLIGHT"
FEATURE_PREFLIGHT_REQUIRED = (
    # Concrete readiness leaves from the unified CLOCKS.features tree.
    # Do not gate on the retired FEATURE_STATUS aggregate nodes or the old
    # FEATURE_STATUS-driven Teensy import sentinel.
    "PI.SYSTEM.HOST",
    "PI.SYSTEM.POWER",
    "PI.GNSS.REPORT",
    "TEENSY.INTERRUPT.PPS_VCLOCK_AUTHORITY",
    "TEENSY.INTERRUPT.QTIMER_COUNTER_CUSTODY",
    "TEENSY.INTERRUPT.COUNTER32_LINEAGE",
    "TEENSY.CLOCKS.DWT_CALIBRATION",
    "TEENSY.CLOCKS.ALPHA_EPOCH",
    "TEENSY.CLOCKS.OCXO_PUBLIC_ORIGIN",
)

FEATURE_PREFLIGHT_POST_START_EXPECTED = (
    "TEENSY.CLOCKS.STATIC_PREDICTION",
)

# ---------------------------------------------------------------------
# TIMEBASE ingress queue
# ---------------------------------------------------------------------

_fragment_queue: queue.Queue[Dict[str, Any]] = queue.Queue(maxsize=TIMEBASE_INGRESS_QUEUE_MAXSIZE)
_clocks_state_queue: queue.Queue[Dict[str, Any]] = queue.Queue(maxsize=CLOCKS_STATE_QUEUE_MAXSIZE)
_clocks_persist_queue: queue.Queue[Dict[str, Any]] = queue.Queue(maxsize=CLOCKS_STATE_QUEUE_MAXSIZE)

# ---------------------------------------------------------------------
# Diagnostics (monotonic counters + last anomaly snapshots)
# ---------------------------------------------------------------------

_diag: Dict[str, Any] = {
    # Unified candidate ingress (PUBSUB handler — fast path)
    "timebase_candidates_received": 0,
    "timebase_candidates_queued": 0,
    "timebase_candidates_processed": 0,

    # Legacy fragment aliases retained for REPORT compatibility. In the live
    # architecture each V4 ``campaign`` object is the complete TEMPEST-relative candidate.
    "fragments_received": 0,
    "fragments_queued": 0,
    "fragments_missing_teensy_pps_count": 0,     # legacy diagnostic alias
    "fragments_missing_teensy_pps_vclock_count": 0,

    # Unified CLOCKS_FRAGMENT campaign-delta processing (processor thread — slow path).
    "timebase_pieces_processed": 0,              # legacy alias: queue items
    "timebase_rows_completed": 0,
    "timebase_pairs_completed": 0,               # legacy alias for rows completed

    # Unified state-detail attachment. CLOCKS decorates the SYSTEM-owned CLOCKS
    # row instead of inserting a second TEMPEST row for the same physical second.
    "campaign_detail_attach_attempts": 0,
    "campaign_detail_attach_retries": 0,
    "campaign_detail_attach_success": 0,
    "campaign_detail_attach_failures": 0,
    "last_campaign_detail_attach": {},

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
    "timebase_final_court_degraded_recovery_admitted": 0,
    "timebase_final_court_science_excluded": 0,
    "timebase_final_court_science_exclusion_violations": 0,
    "last_timebase_final_court_row_drop": {},

    # Firmware-authored whole-row science exclusions. Coherent rows are always
    # persisted; these counters explain audit-only admissions.
    "firmware_science_exclude_received": 0,
    "last_firmware_science_exclude": {},

    # Dedicated invalid-TIMEBASE JSONL evidence log.
    "invalid_timebase_log_path": INVALID_TIMEBASE_LOG_PATH,
    "invalid_timebase_log_ready": False,
    "invalid_timebase_log_writes": 0,
    "invalid_timebase_log_failures": 0,
    "last_invalid_timebase_log": {},

    # Candidate processing (processor thread — slow path). Fragment keys are
    # legacy aliases retained for REPORT compatibility.
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

    # CLOCKS-backed campaign preflight
    "preflight_feature_checks": 0,
    "preflight_feature_blocked": 0,
    "preflight_feature_unavailable": 0,
    "preflight_clocks_updates": 0,
    "preflight_clocks_malformed": 0,
    "preflight_clocks_missing_features": 0,
    "preflight_clocks_stale": 0,
    "last_preflight_clocks": {},
    "last_preflight_feature_gate": {},
    "preflight_wait_log_count": 0,
    "last_preflight_wait": {},

    # GNSS discipline info fetch
    "gnss_info_requests": 0,
    "gnss_info_hits": 0,
    "gnss_info_misses": 0,

    # GNSS_RAW recovery projection / sanity rebuild
    "gnss_raw_recovery_restore_count": 0,
    "gnss_raw_recovery_rebuild_count": 0,
    "last_gnss_raw_recovery": {},
    "gnss_raw_stats_poll_count": 0,
    "gnss_raw_stats_sample_count": 0,
    "gnss_raw_stats_missing_count": 0,
    "last_gnss_raw_stats_sample": {},
    "clocks_baseline_refresh_count": 0,
    "clocks_baseline_refresh_failures": 0,
    "stats_reset_requests": 0,
    "stats_reset_success": 0,
    "stats_reset_teensy_failures": 0,
    "last_stats_reset": {},
    "report_clocks_requests": 0,
    "report_stats_requests": 0,
    "report_transitive_failures": 0,
    "last_report_clocks": {},
    "last_report_stats": {},

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

    # SYSTEM-owned CLOCKS recovery may hand the Pi-owned GNSS_RAW state back
    # to CLOCKS before CLOCKS startup campaign reconciliation completes.

    # CLOCKS startup lifecycle serialization
    "startup_control_ready": False,
    "startup_control_busy_rejections": 0,
    "last_startup_control_rejection": {},

    # CLOCKS_FRAGMENT silence / Teensy restart detection
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
            "public_count": verdict.get("public_count"),
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


def _firmware_science_exclusion(fragment: Dict[str, Any]) -> Dict[str, Any]:
    """Return the firmware objection transcript, or an empty dict if admitted."""
    disposition = _candidate_disposition(fragment)
    explicit_excluded = _timebase_final_court_bool(
        fragment.get("science_excluded")
    )
    explicit_ineligible = (
        "science_eligible" in fragment
        and not _timebase_final_court_bool(fragment.get("science_eligible"))
    )
    explicit_control_ineligible = (
        "control_eligible" in fragment
        and not _timebase_final_court_bool(fragment.get("control_eligible"))
    )
    excluded = (
        disposition == TIMEBASE_CANDIDATE_SCIENCE_EXCLUDE
        or explicit_excluded
        or explicit_ineligible
        or explicit_control_ineligible
    )
    if not excluded:
        return {}

    return {
        "source": str(fragment.get("candidate_source") or "TEENSY_CLOCKS"),
        "reason_code": _as_int(fragment.get("candidate_reason_code")) or 0,
        "reason_name": str(
            fragment.get("candidate_reason_name")
            or fragment.get("candidate_reason")
            or "unspecified"
        ),
        "reason": str(fragment.get("candidate_reason") or "unspecified"),
        "lane": _as_int(fragment.get("candidate_lane")) or 0,
        "reject_mask": _as_int(fragment.get("candidate_reject_mask")) or 0,
        "objection_count": _as_int(fragment.get("candidate_objection_count")) or 1,
        "detail0": _as_int(fragment.get("candidate_detail0")) or 0,
        "detail1": _as_int(fragment.get("candidate_detail1")) or 0,
        "detail2": _as_int(fragment.get("candidate_detail2")) or 0,
        "detail3": _as_int(fragment.get("candidate_detail3")) or 0,
        "disposition": disposition,
        "science_eligible": _timebase_final_court_bool(
            fragment.get("science_eligible", not explicit_ineligible)
        ),
        "control_eligible": _timebase_final_court_bool(
            fragment.get("control_eligible", not explicit_control_ineligible)
        ),
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

# Process-lifetime GNSS_RAW instrument clockface.  Unlike the campaign view
# above, this advances continuously in the same 1 Hz loop that owns the
# always-on Welford, so an idle display still has a live synthetic clock.
_gnss_raw_instrument_ns: float = 0.0
_gnss_raw_instrument_n: int = 0
_gnss_raw_instrument_valid: bool = False

# Cached baseline decoration for canonical CLOCKS. Refreshing this slow-changing
# DB state is bounded and never occurs in a display repaint path.
_clocks_baseline_cache: Dict[str, Any] = {}
_clocks_baseline_refreshed_monotonic: Optional[float] = None

# GNSS_RAW statistics are Pi-owned and always-on. Campaign lifecycle may
# rebase the synthetic clockface, but it never resets this population.
_gnss_raw_stats_lock = threading.Lock()
_gnss_raw_welford_n: int = 0
_gnss_raw_welford_mean: float = 0.0
_gnss_raw_welford_m2: float = 0.0
_gnss_raw_welford_min: float = 1e30
_gnss_raw_welford_max: float = -1e30
_gnss_raw_latest_info: Dict[str, Any] = {}
_gnss_raw_latest_info_monotonic: Optional[float] = None

# Retain a short predictive-UTC history so completed PPS seconds remain
# selectable after the receiver has staged the following second.

def _gnss_raw_clock_reset() -> None:
    """Reset only the campaign/recovery synthetic clockface."""
    global _gnss_raw_ns, _gnss_raw_n, _gnss_raw_valid
    _gnss_raw_ns = 0.0
    _gnss_raw_n = 0
    _gnss_raw_valid = False


def _gnss_raw_welford_reset() -> Dict[str, Any]:
    """Reset only the Pi-owned always-on GNSS_RAW statistical population."""
    global _gnss_raw_welford_n, _gnss_raw_welford_mean, _gnss_raw_welford_m2
    global _gnss_raw_welford_min, _gnss_raw_welford_max
    with _gnss_raw_stats_lock:
        previous = {
            "n": int(_gnss_raw_welford_n),
            "mean": float(_gnss_raw_welford_mean),
            "m2": float(_gnss_raw_welford_m2),
            "min": float(_gnss_raw_welford_min) if _gnss_raw_welford_n else 0.0,
            "max": float(_gnss_raw_welford_max) if _gnss_raw_welford_n else 0.0,
        }
        _gnss_raw_welford_n = 0
        _gnss_raw_welford_mean = 0.0
        _gnss_raw_welford_m2 = 0.0
        _gnss_raw_welford_min = 1e30
        _gnss_raw_welford_max = -1e30
    return previous

# ---------------------------------------------------------------------
# Local state (process lifetime)
# ---------------------------------------------------------------------

_campaign_active: bool = False

# Warm recovery may resume on a truthful degraded timeline row while the
# firmware completes its deterministic OCXO science quarantine.  Arm a one-shot
# confirmation so the first later science-and-control eligible row closes the
# recovery narrative explicitly.
_post_recovery_science_confirmation_pending: bool = False
_post_recovery_science_confirmation_campaign: Optional[str] = None
_post_recovery_first_public_pps_vclock_count: Optional[int] = None

# CLOCKS owns the canonical state stream.  The worker publishes fresh state
# during startup restore, but persistence opens only after the holistic restore
# transaction reaches a terminal outcome.
_clocks_persistence_enabled = threading.Event()
_clocks_persistence_lock = threading.Lock()
_clocks_state_worker_started = threading.Event()
_clocks_state_enqueued = 0
_clocks_state_published = 0
_clocks_state_persisted = 0
_clocks_state_inserted = 0
_clocks_state_merged = 0
_clocks_state_dropped = 0
_last_clocks_state_sequence: Optional[int] = None
_last_clocks_state_monotonic: Optional[float] = None
_last_tempest_candidate_identity: Optional[Tuple[str, int]] = None
_last_tempest_candidate_monotonic: Optional[float] = None

# The command server is exposed early so PUBSUB can discover subscriptions, but
# START/RESUME must not race holistic startup reconciliation.
_startup_control_ready = threading.Event()

# Latest unified operational heartbeat.  CLOCKS consumes CLOCKS.features for
# campaign preflight; it never polls or subscribes to a feature-only side feed.
_clocks_lock = threading.Lock()
_latest_clocks: Dict[str, Any] = {}
_latest_clocks_received_monotonic: Optional[float] = None
_latest_clocks_received_utc: Optional[str] = None

_last_pps_vclock_count_seen: Optional[int] = None

# Last PPS/VCLOCK count accepted into TIMEBASE processing.
# This is observational only; it is never used to reject a fragment.
_accepted_pps_vclock_count: Optional[int] = None

# CLOCKS_FRAGMENT silence monitor.  This is intentionally process-local: if a
# campaign is active and the Teensy stops publishing CLOCKS_FRAGMENT campaign deltas, CLOCKS
# treats the silence as a recoverable Teensy lifecycle event once communication
# returns.
_timebase_last_activity_monotonic: Optional[float] = None
_timebase_last_activity_utc: Optional[str] = None
_timebase_last_activity_topic: Optional[str] = None
_timebase_last_activity_pps_vclock_count: Optional[int] = None
_timebase_silence_recovery_active: bool = False

# Asynchronous START observation.  START no longer blocks waiting for the
# first CLOCKS_FRAGMENT campaign delta; this records the pending wait so
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
    """Raised when RECOVER produces no accepted CLOCKS_FRAGMENT campaign delta."""


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
    Record receipt of one Teensy CLOCKS_FRAGMENT campaign-delta candidate.

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
    Start or restart the silence timer when CLOCKS expects CLOCKS_FRAGMENT campaign deltas.

    This covers the interval before the first post-START/post-RECOVER fragment.
    Actual CLOCKS_FRAGMENT campaign-delta receipts replace this synthetic watch-arm marker through
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
                UPDATE campaign_master
                SET payload = (payload - 'flash_cut_pending')
                    || jsonb_build_object(
                        'flash_cut_committed_at', to_jsonb(%s::text),
                        'flash_cut_first_pps_vclock_count', to_jsonb(%s::int),
                        'flash_cut_first_pps_count', to_jsonb(%s::int)
                    )
                WHERE campaign_type = %s
                  AND campaign = %s
                  AND active = true
                  AND payload ? 'flash_cut_pending'
                """,
                (
                    now_utc,
                    int(pps_vclock_count),
                    int(pps_vclock_count),
                    CAMPAIGN_TYPE_TEMPEST,
                    campaign,
                ),
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
            "💥 [recovery] pending Flash Cut campaign '%s' has no TEMPEST campaign details after %.3fs "
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
        "⚡ [recovery] campaign '%s' has no TEMPEST campaign details but is a pending Flash Cut child; "
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
    process-local campaign gate is lowered so stale or partial CLOCKS_FRAGMENT campaign-delta candidates
    cannot be persisted while the Teensy is being recovered.
    """
    global _campaign_active, _auto_recovery_in_progress
    global _timebase_silence_recovery_active

    attempts = 0
    try:
        while True:
            if _teensy_clocks_health_ok():
                logging.info(
                    "✅ [clocks] @%s Teensy CLOCKS REPORT responded after CLOCKS_FRAGMENT silence — "
                    "invoking campaign recovery",
                    system_time_z(),
                )
                attempts += 1
                _diag["auto_recovery_attempts"] = _diag.get("auto_recovery_attempts", 0) + 1
                _diag["timebase_silence_recovery_started"] += 1
                try:
                    _restore_active_campaign_state()
                    logging.info(
                        "✅ [clocks] @%s CLOCKS_FRAGMENT silence recovery complete",
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
                            "💥 [clocks] CLOCKS_FRAGMENT silence recovery stopped after %d attempt(s): "
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
                        "⚠️ [clocks] CLOCKS_FRAGMENT silence recovery retryable failure "
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
                        "💥 [clocks] CLOCKS_FRAGMENT silence recovery failed — "
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
        "💥 [clocks] CLOCKS_FRAGMENT silence detected during active campaign "
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
    Detect a live Teensy reboot/loss by absence of CLOCKS_FRAGMENT campaign deltas.

    The monitor is intentionally quiet unless it detects the first silence
    threshold crossing.  Health-check failures after that point are silent and
    retried once per TEENSY_HEALTH_RETRY_S.
    """
    _diag["timebase_silence_monitor_started"] = True
    logging.info(
        "🚀 [clocks] CLOCKS_FRAGMENT silence monitor started "
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
            # This monitor is for loss after the CLOCKS_FRAGMENT campaign stream has been observed.
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
                    _restore_active_campaign_state()
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
            _restore_active_campaign_state()
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
    """Return the active TEMPEST campaign master row, or None."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT campaign_type, campaign, payload
            FROM campaign_master
            WHERE campaign_type = %s
              AND active = true
            ORDER BY ts DESC
            LIMIT 1
            """,
            (CAMPAIGN_TYPE_TEMPEST,),
        )
        row = cur.fetchone()

    if row is None:
        return None

    payload = row["payload"]
    if isinstance(payload, str):
        payload = json.loads(payload)

    return {
        "campaign_type": row["campaign_type"],
        "campaign": row["campaign"],
        "payload": payload,
    }



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


def _clocks_payload(monitor: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    if not isinstance(monitor, dict):
        return {}
    clocks = monitor.get("clocks")
    return clocks if isinstance(clocks, dict) else {}



def _ppb_replay_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _ppb_zero_endpoint() -> Dict[str, Any]:
    return {
        "reference_ns": 0,
        "dwt_error_cycles": 0.0,
        "ocxo1_error_ns": 0,
        "ocxo2_error_ns": 0,
        "rolling_sequence": 0,
        "interval_count": 0,
    }


def _ppb_minute_key(rolling_sequence: int) -> int:
    return ((int(rolling_sequence) - 1) // 60) + 1 if rolling_sequence > 0 else 0


def _ppb_signed_delta(clock_delta: int, reference_delta: int) -> int:
    return (
        int(clock_delta - reference_delta)
        if clock_delta >= reference_delta
        else -int(reference_delta - clock_delta)
    )


def _ppb_raw_monotonic(previous: Dict[str, int], current: Dict[str, int]) -> bool:
    return (
        current["gnss_ns"] > previous["gnss_ns"]
        and current["dwt_cycles"] > previous["dwt_cycles"]
        and current["ocxo1_ns"] > previous["ocxo1_ns"]
        and current["ocxo2_ns"] > previous["ocxo2_ns"]
    )


def _replay_better_buckets_rows(
    rows,
    *,
    lower_sequence: int,
    target_update_count: int,
    target_current_sequence: int,
) -> Dict[str, Any]:
    """Reconstruct Alpha's bounded Better-Buckets state in one forward pass.

    rolling_sequence is canonical stats.update_count.  The first cursor row may
    begin before the oldest 24-hour anchor rather than at statistical epoch zero;
    any omitted cumulative prefix is a harmless common offset that cancels from
    every rolling subtraction.  interval_advanced preserves deliberate reboot /
    rejection cuts so continuity-adjusted clockfaces are never joined by guess.
    """
    if target_update_count < 0 or target_current_sequence < 0:
        raise ValueError("invalid Better-Buckets target chronology")
    if target_current_sequence > target_update_count:
        raise ValueError(
            "Better-Buckets current sequence exceeds stats update_count: "
            f"current={target_current_sequence} update_count={target_update_count}"
        )

    current = _ppb_zero_endpoint()
    origin: Optional[Dict[str, Any]] = None
    second_history = deque(maxlen=PPB_REPLAY_SECOND_CAPACITY)
    minute_history = deque(maxlen=PPB_REPLAY_MINUTE_CAPACITY)
    previous_raw: Optional[Dict[str, int]] = None
    last_admitted_sequence = 0
    last_minute_key = 0
    expected_sequence = int(lower_sequence)
    rows_scanned = 0
    admitted_rows = 0
    advanced_intervals = 0
    omitted_prefix_interval = False

    for row in rows:
        sequence = int(row["update_count"])
        if sequence != expected_sequence:
            raise ValueError(
                "Better-Buckets durable chronology gap: "
                f"expected={expected_sequence} observed={sequence}"
            )
        expected_sequence += 1
        rows_scanned += 1

        current_witness = int(row.get("current_sequence") or 0)
        admitted = _ppb_replay_bool(row.get("endpoint_admitted"))
        interval_advanced = _ppb_replay_bool(row.get("interval_advanced"))
        if interval_advanced and not admitted:
            raise ValueError(
                f"Better-Buckets row {sequence} advances interval while endpoint is excluded"
            )

        if not admitted:
            if last_admitted_sequence == 0:
                last_admitted_sequence = current_witness
            elif current_witness != last_admitted_sequence:
                raise ValueError(
                    "Better-Buckets excluded-row current-sequence drift: "
                    f"row={sequence} expected={last_admitted_sequence} "
                    f"observed={current_witness}"
                )
            previous_raw = None
            continue

        if current_witness != sequence:
            raise ValueError(
                "Better-Buckets admitted row does not own current sequence: "
                f"row={sequence} current={current_witness}"
            )

        raw = {
            "rolling_sequence": sequence,
            "gnss_ns": int(row["gnss_ns"]),
            "dwt_cycles": int(row["dwt_cycles"]),
            "ocxo1_ns": int(row["ocxo1_ns"]),
            "ocxo2_ns": int(row["ocxo2_ns"]),
        }
        if min(raw["gnss_ns"], raw["dwt_cycles"], raw["ocxo1_ns"], raw["ocxo2_ns"]) <= 0:
            raise ValueError(f"Better-Buckets row {sequence} has nonpositive clockface")

        next_endpoint = dict(current)
        next_endpoint["rolling_sequence"] = sequence

        if interval_advanced:
            if previous_raw is None:
                # A bounded replay may omit the interval entering its first row.
                # lower_sequence is at least one full minute before the oldest
                # possible 24h anchor, so this becomes only a common cumulative
                # offset and cannot affect any reconstructed rolling subtraction.
                if rows_scanned == 1 and lower_sequence > 1:
                    omitted_prefix_interval = True
                else:
                    raise ValueError(
                        f"Better-Buckets row {sequence} advances without an antecedent"
                    )
            else:
                if sequence != previous_raw["rolling_sequence"] + 1:
                    raise ValueError(
                        "Better-Buckets advanced interval is not adjacent: "
                        f"previous={previous_raw['rolling_sequence']} current={sequence}"
                    )
                if not _ppb_raw_monotonic(previous_raw, raw):
                    raise ValueError(
                        f"Better-Buckets row {sequence} claims advance across nonmonotonic clocks"
                    )
                reference_delta = raw["gnss_ns"] - previous_raw["gnss_ns"]
                dwt_delta = raw["dwt_cycles"] - previous_raw["dwt_cycles"]
                ocxo1_delta = raw["ocxo1_ns"] - previous_raw["ocxo1_ns"]
                ocxo2_delta = raw["ocxo2_ns"] - previous_raw["ocxo2_ns"]
                expected_dwt_delta = (
                    float(reference_delta) * float(DWT_EXPECTED_PER_PPS)
                ) / float(NS_PER_SECOND)
                next_endpoint["reference_ns"] += reference_delta
                next_endpoint["dwt_error_cycles"] += (
                    float(dwt_delta) - expected_dwt_delta
                )
                next_endpoint["ocxo1_error_ns"] += _ppb_signed_delta(
                    ocxo1_delta, reference_delta
                )
                next_endpoint["ocxo2_error_ns"] += _ppb_signed_delta(
                    ocxo2_delta, reference_delta
                )
                next_endpoint["interval_count"] += 1
                advanced_intervals += 1
        current = next_endpoint
        if origin is None:
            origin = dict(current)

        second_history.append(dict(current))
        minute_key = _ppb_minute_key(sequence)
        if minute_key != last_minute_key:
            minute_history.append(dict(current))
            last_minute_key = minute_key

        previous_raw = raw
        last_admitted_sequence = sequence
        admitted_rows += 1

    if target_current_sequence > 0:
        if rows_scanned == 0:
            raise ValueError("Better-Buckets replay returned no durable rows")
        if expected_sequence != target_current_sequence + 1:
            raise ValueError(
                "Better-Buckets replay ended before current endpoint: "
                f"next={expected_sequence} current={target_current_sequence}"
            )
        if last_admitted_sequence != target_current_sequence:
            raise ValueError(
                "Better-Buckets replay current endpoint mismatch: "
                f"replayed={last_admitted_sequence} target={target_current_sequence}"
            )
        if int(current.get("rolling_sequence") or 0) != target_current_sequence:
            raise ValueError("Better-Buckets cumulative current sequence mismatch")
        if origin is None:
            raise ValueError("Better-Buckets replay has current endpoint but no origin")

    return {
        "schema": "PI_BETTER_BUCKETS_RESTORE_V1",
        "rolling_sequence": int(target_update_count),
        "current_sequence": int(target_current_sequence),
        "lower_sequence": int(lower_sequence),
        "rows_scanned": int(rows_scanned),
        "admitted_rows": int(admitted_rows),
        "advanced_intervals": int(advanced_intervals),
        "omitted_prefix_interval": bool(omitted_prefix_interval),
        "last_minute_key": int(last_minute_key),
        "origin_valid": origin is not None,
        "origin": dict(origin) if origin is not None else _ppb_zero_endpoint(),
        "current": dict(current),
        "second_history": list(second_history),
        "minute_history": list(minute_history),
    }


def _ppb_replay_anchor(
    replay: Dict[str, Any],
    window_seconds: int,
    *,
    exact_second_history: bool,
) -> Optional[Dict[str, Any]]:
    current = replay.get("current") if isinstance(replay, dict) else None
    origin = replay.get("origin") if isinstance(replay, dict) else None
    if not isinstance(current, dict) or not replay.get("origin_valid"):
        return None
    current_sequence = int(current.get("rolling_sequence") or 0)
    origin_sequence = int(origin.get("rolling_sequence") or 0) if isinstance(origin, dict) else 0
    if current_sequence == 0 or origin_sequence >= current_sequence:
        return None
    if current_sequence <= int(window_seconds):
        return origin if isinstance(origin, dict) else None

    target = current_sequence - int(window_seconds)
    history = replay.get(
        "second_history" if exact_second_history else "minute_history"
    )
    if not isinstance(history, list):
        return None
    for endpoint in history:
        if not isinstance(endpoint, dict):
            continue
        sequence = int(endpoint.get("rolling_sequence") or 0)
        if sequence >= target and sequence < current_sequence:
            return endpoint
    return None


def _ppb_replay_bucket_value(
    replay: Dict[str, Any],
    lane: str,
    window_seconds: int,
    *,
    exact_second_history: bool,
) -> Optional[float]:
    current = replay.get("current")
    if not isinstance(current, dict):
        return None
    anchor = _ppb_replay_anchor(
        replay, window_seconds, exact_second_history=exact_second_history
    )
    if not isinstance(anchor, dict):
        return None

    interval_count = int(current["interval_count"]) - int(anchor["interval_count"])
    reference_ns = int(current["reference_ns"]) - int(anchor["reference_ns"])
    if interval_count <= 0 or reference_ns <= 0:
        return None

    if lane == "dwt":
        expected_cycles = (
            float(reference_ns) * float(DWT_EXPECTED_PER_PPS)
        ) / float(NS_PER_SECOND)
        if expected_cycles <= 0.0:
            return None
        error_cycles = (
            float(current["dwt_error_cycles"])
            - float(anchor["dwt_error_cycles"])
        )
        return error_cycles * 1.0e9 / expected_cycles
    if lane == "vclock":
        return 0.0
    if lane == "ocxo1":
        error_ns = int(current["ocxo1_error_ns"]) - int(anchor["ocxo1_error_ns"])
        return float(error_ns) * 1.0e9 / float(reference_ns)
    if lane == "ocxo2":
        error_ns = int(current["ocxo2_error_ns"]) - int(anchor["ocxo2_error_ns"])
        return float(error_ns) * 1.0e9 / float(reference_ns)
    raise ValueError(f"unsupported Better-Buckets lane {lane!r}")


def _verify_better_buckets_replay(
    detail: Dict[str, Any],
    replay: Dict[str, Any],
) -> Dict[str, Any]:
    """Prove reconstructed rings reproduce Alpha's last durable bucket values."""
    stats = _path_get(detail, "clocks.stats")
    if not isinstance(stats, dict):
        raise ValueError("Better-Buckets replay verification missing canonical stats")

    windows = (
        ("10_min", PPB_REPLAY_10_MIN_SECONDS, True),
        ("60_min", PPB_REPLAY_60_MIN_SECONDS, False),
        ("8_hour", PPB_REPLAY_8_HOUR_SECONDS, False),
        ("24_hour", PPB_REPLAY_24_HOUR_SECONDS, False),
    )
    checked = 0
    comparisons: Dict[str, Any] = {}
    for lane in ("dwt", "vclock", "ocxo1", "ocxo2"):
        lane_out: Dict[str, Any] = {}
        for key, seconds, exact in windows:
            computed = _ppb_replay_bucket_value(
                replay, lane, seconds, exact_second_history=exact
            )
            recorded = _path_get(stats, f"{lane}.ppb_buckets.{key}")
            if computed is None and recorded is None:
                lane_out[key] = None
                continue
            if computed is None or recorded is None:
                raise ValueError(
                    "Better-Buckets replay availability mismatch: "
                    f"lane={lane} window={key} computed={computed!r} recorded={recorded!r}"
                )
            recorded_value = float(recorded)
            delta = float(computed) - recorded_value
            if abs(delta) > PPB_REPLAY_VERIFY_TOLERANCE_PPB:
                raise ValueError(
                    "Better-Buckets replay value mismatch: "
                    f"lane={lane} window={key} computed={computed:.9f} "
                    f"recorded={recorded_value:.9f} delta={delta:.9f}"
                )
            lane_out[key] = {
                "computed": round(float(computed), 9),
                "recorded": round(recorded_value, 9),
                "delta": round(delta, 9),
            }
            checked += 1
        comparisons[lane] = lane_out
    return {"checked": checked, "comparisons": comparisons}


def _reconstruct_better_buckets_from_db(detail: Dict[str, Any]) -> Dict[str, Any]:
    clocks = _clocks_payload(detail)
    stats = clocks.get("stats") if isinstance(clocks, dict) else None
    if not isinstance(stats, dict):
        raise ValueError("canonical CLOCKS missing stats for Better-Buckets replay")

    reset_count = _as_int(stats.get("reset_count"))
    update_count = _as_int(stats.get("update_count"))
    current_sequence = _as_int(stats.get("rolling_ppb_current_sequence"))
    target_db_id = _as_int(detail.get("_db_detail_id"))
    if reset_count is None or reset_count < 0:
        raise ValueError("Better-Buckets replay missing reset_count")
    if update_count is None or update_count < 0:
        raise ValueError("Better-Buckets replay missing update_count")
    if current_sequence is None or current_sequence < 0 or current_sequence > update_count:
        raise ValueError("Better-Buckets replay missing/invalid current sequence")
    if target_db_id is None or target_db_id <= 0:
        raise ValueError("Better-Buckets replay missing durable campaign_detail id")

    if update_count == 0:
        return {
            "schema": "PI_BETTER_BUCKETS_RESTORE_V1",
            "rolling_sequence": 0,
            "current_sequence": 0,
            "lower_sequence": 0,
            "rows_scanned": 0,
            "admitted_rows": 0,
            "advanced_intervals": 0,
            "omitted_prefix_interval": False,
            "last_minute_key": 0,
            "origin_valid": False,
            "origin": _ppb_zero_endpoint(),
            "current": _ppb_zero_endpoint(),
            "second_history": [],
            "minute_history": [],
            "verification": {"checked": 0, "comparisons": {}},
        }

    if current_sequence == 0:
        replay = _replay_better_buckets_rows(
            [],
            lower_sequence=1,
            target_update_count=update_count,
            target_current_sequence=0,
        )
        replay["verification"] = _verify_better_buckets_replay(detail, replay)
        return replay

    lower_sequence = max(
        1,
        int(current_sequence)
        - PPB_REPLAY_24_HOUR_SECONDS
        - PPB_REPLAY_MARGIN_SECONDS,
    )

    with open_db(row_dict=True) as conn:
        conn.execute("SET TRANSACTION READ ONLY")
        cur = conn.cursor(name="clocks_ppb_restore_stream")
        cur.itersize = PPB_REPLAY_CURSOR_ITERSIZE
        cur.execute(
            """
            SELECT
                id,
                (payload #>> '{clocks,stats,update_count}')::bigint AS update_count,
                (payload #>> '{clocks,stats,rolling_ppb_current_sequence}')::bigint
                    AS current_sequence,
                payload #>> '{clocks,stats,rolling_ppb_endpoint_admitted}'
                    AS endpoint_admitted,
                payload #>> '{clocks,stats,rolling_ppb_interval_advanced}'
                    AS interval_advanced,
                (payload #>> '{clocks,clockfaces,gnss_ns}')::bigint AS gnss_ns,
                (payload #>> '{clocks,clockfaces,dwt_cycles}')::bigint AS dwt_cycles,
                (payload #>> '{clocks,clockfaces,ocxo1_ns}')::bigint AS ocxo1_ns,
                (payload #>> '{clocks,clockfaces,ocxo2_ns}')::bigint AS ocxo2_ns
            FROM campaign_detail
            WHERE campaign_type = %s
              AND id <= %s
              AND NOT (payload @> '{"holistic_restore_superseded":true}'::jsonb)
              AND payload #>> '{schema}' = 'CLOCKS_V4'
              AND payload #>> '{clocks,stats,schema}' = 'CLOCKS_INSTRUMENT_STATS_V4'
              AND (payload #>> '{clocks,stats,reset_count}')::bigint = %s
              AND (payload #>> '{clocks,stats,update_count}')::bigint
                    BETWEEN %s AND %s
            ORDER BY (payload #>> '{clocks,stats,update_count}')::bigint ASC,
                     id ASC
            """,
            (
                CAMPAIGN_TYPE_TEMPEST,
                int(target_db_id),
                int(reset_count),
                int(lower_sequence),
                int(current_sequence),
            ),
        )
        replay = _replay_better_buckets_rows(
            cur,
            lower_sequence=int(lower_sequence),
            target_update_count=int(update_count),
            target_current_sequence=int(current_sequence),
        )

    replay["reset_count"] = int(reset_count)
    replay["source_db_detail_id"] = int(target_db_id)
    replay["verification"] = _verify_better_buckets_replay(detail, replay)
    return replay


def _supersede_cold_bootstrap_rows(
    *,
    base_detail_id: int,
    campaign_name: str,
    first_public_count: int,
) -> Dict[str, Any]:
    """Mark pre-restore fresh-boot rows non-viable without deleting evidence.

    The first recovered campaign row has already passed through canonical CLOCKS
    persistence before the recovery processor signals it.  Every TEMPEST row
    inserted after the durable recovery source but before that row belongs to
    the superseded fresh-boot instrument and must never participate in a later
    Holistic Restore replay. ``viable`` remains campaign-science semantics; the
    explicit payload marker is the instrument-restore exclusion authority.
    """
    base_detail_id = int(base_detail_id)
    first_public_count = int(first_public_count)
    if base_detail_id <= 0 or first_public_count <= 0:
        raise ValueError("invalid cold-bootstrap supersede boundary")

    with _clocks_persistence_lock:
        with open_db(row_dict=True) as conn:
            cur = conn.cursor()
            cur.execute(
                """
                SELECT id
                FROM campaign_detail
                WHERE campaign_type = %s
                  AND campaign = %s
                  AND payload #>> '{campaign,public_count}' = %s
                ORDER BY id DESC
                LIMIT 1
                """,
                (CAMPAIGN_TYPE_TEMPEST, campaign_name, str(first_public_count)),
            )
            row = cur.fetchone()
            if row is None:
                raise RuntimeError(
                    "first recovered campaign row is not durably identifiable"
                )
            recovered_detail_id = int(row["id"])
            if recovered_detail_id <= base_detail_id:
                raise RuntimeError(
                    "cold-bootstrap recovered row does not follow recovery source"
                )
            cur.execute(
                """
                UPDATE campaign_detail
                SET viable = false,
                    payload = payload || jsonb_build_object(
                        'holistic_restore_superseded', true,
                        'holistic_restore_superseded_by_detail_id', %s::bigint
                    )
                WHERE campaign_type = %s
                  AND id > %s
                  AND id < %s
                """,
                (
                    recovered_detail_id,
                    CAMPAIGN_TYPE_TEMPEST,
                    base_detail_id,
                    recovered_detail_id,
                ),
            )
            superseded = int(cur.rowcount or 0)

    return {
        "base_detail_id": base_detail_id,
        "recovered_detail_id": recovered_detail_id,
        "rows_marked_nonviable": superseded,
    }


def _ppb_endpoint_command_args(prefix: str, endpoint: Dict[str, Any]) -> Dict[str, Any]:
    return {
        f"{prefix}_reference_ns": int(endpoint.get("reference_ns") or 0),
        f"{prefix}_dwt_error_cycles": float(endpoint.get("dwt_error_cycles") or 0.0),
        f"{prefix}_ocxo1_error_ns": int(endpoint.get("ocxo1_error_ns") or 0),
        f"{prefix}_ocxo2_error_ns": int(endpoint.get("ocxo2_error_ns") or 0),
        f"{prefix}_rolling_sequence": int(endpoint.get("rolling_sequence") or 0),
        f"{prefix}_interval_count": int(endpoint.get("interval_count") or 0),
    }


def _send_teensy_ppb_restore_command(
    command: str,
    args: Dict[str, Any],
    accepted_statuses: Tuple[str, ...],
) -> Dict[str, Any]:
    response = send_command(
        machine="TEENSY",
        subsystem="CLOCKS",
        command=command,
        args=args,
    )
    outer_success = isinstance(response, dict) and bool(response.get("success"))
    payload = response.get("payload") if isinstance(response, dict) else None
    payload = payload if isinstance(payload, dict) else {}
    status = str(payload.get("status") or "")
    if not outer_success or status not in accepted_statuses:
        raise RuntimeError(
            f"Teensy {command} rejected: status={status or 'missing'} "
            f"response={response!r}"
        )
    return payload


def _abort_teensy_ppb_restore_best_effort() -> None:
    try:
        send_command(
            machine="TEENSY",
            subsystem="CLOCKS",
            command="PPB_RESTORE_ABORT",
            retries=1,
            retry_delay_s=0.0,
        )
    except Exception:
        logging.debug("⚠️ [holistic restore] PPB_RESTORE_ABORT failed (ignored)")


def _stage_teensy_better_buckets(replay: Dict[str, Any]) -> Dict[str, Any]:
    rolling_sequence = int(replay.get("rolling_sequence") or 0)
    if rolling_sequence == 0:
        return {"staged": False, "reason": "empty_statistics_epoch"}

    second_history = replay.get("second_history")
    minute_history = replay.get("minute_history")
    if not isinstance(second_history, list) or not isinstance(minute_history, list):
        raise ValueError("Better-Buckets replay missing history arrays")

    begin_args: Dict[str, Any] = {
        "rolling_sequence": rolling_sequence,
        "second_count": len(second_history),
        "minute_count": len(minute_history),
        "last_minute_key": int(replay.get("last_minute_key") or 0),
        "origin_valid": bool(replay.get("origin_valid")),
    }
    begin_args.update(_ppb_endpoint_command_args("current", replay["current"]))
    if replay.get("origin_valid"):
        begin_args.update(_ppb_endpoint_command_args("origin", replay["origin"]))

    started = time.monotonic()
    try:
        begin_payload = _send_teensy_ppb_restore_command(
            "PPB_RESTORE_BEGIN",
            begin_args,
            ("ppb_restore_staging",),
        )
        firmware_chunk_max = _as_int(begin_payload.get("chunk_max_endpoints")) or 1
        chunk_size = max(1, min(PPB_RESTORE_CHUNK_ENDPOINTS, firmware_chunk_max))
        chunks = 0

        for history_name, history in (
            ("SECOND", second_history),
            ("MINUTE", minute_history),
        ):
            for offset in range(0, len(history), chunk_size):
                batch = history[offset:offset + chunk_size]
                chunk_args: Dict[str, Any] = {
                    "history": history_name,
                    "offset": int(offset),
                    "count": len(batch),
                }
                for index, endpoint in enumerate(batch):
                    chunk_args.update(
                        _ppb_endpoint_command_args(f"e{index}", endpoint)
                    )
                _send_teensy_ppb_restore_command(
                    "PPB_RESTORE_CHUNK",
                    chunk_args,
                    ("ppb_restore_chunk_accepted", "ppb_restore_chunk_duplicate"),
                )
                chunks += 1

        commit_payload = _send_teensy_ppb_restore_command(
            "PPB_RESTORE_COMMIT",
            {"rolling_sequence": rolling_sequence},
            ("ppb_restore_committed",),
        )
        return {
            "staged": True,
            "rolling_sequence": rolling_sequence,
            "second_count": len(second_history),
            "minute_count": len(minute_history),
            "chunks": chunks,
            "chunk_size": chunk_size,
            "waited_s": round(time.monotonic() - started, 3),
            "teensy": commit_payload,
            "replay_rows": int(replay.get("rows_scanned") or 0),
            "verification_checks": int(
                (replay.get("verification") or {}).get("checked") or 0
            ),
        }
    except Exception:
        _abort_teensy_ppb_restore_best_effort()
        raise


def _probe_teensy_recovery_epoch_ready() -> bool:
    """Return explicit Teensy epoch custody; never guess cold/live on no report."""
    last_status: Dict[str, Any] = {}
    for _ in range(4):
        status = _fetch_teensy_recovery_status()
        if status:
            last_status = status
            if "recover_epoch_ready" in status:
                return _recovery_bool(status.get("recover_epoch_ready"))
        time.sleep(0.25)
    raise RecoveryRetryableFailure(
        "recovery_epoch_probe_unavailable",
        {"report_recovery": last_status},
    )


def _canonical_restore_args(
    clocks: Dict[str, Any],
    *,
    include_control: bool = True,
) -> Dict[str, Any]:
    """Flatten canonical CLOCKS_V4 instrument state into Teensy restore fields.

    CLOCKS_FRAGMENT_V4 deliberately has no parallel ``restore_state`` mirror.
    Exact restart authority is the ordinary canonical instrument state:
    clockfaces + statistics + control.  The three interval-reject counters are
    firmware REPORT diagnostics, not canonical science; the current restore
    command still requires them, so they restart at zero.
    """
    if not isinstance(clocks, dict):
        raise ValueError("canonical CLOCKS restore source is not an object")

    instrument = clocks.get("clockfaces")
    stats = clocks.get("stats")
    control = clocks.get("control")
    auxiliary = stats.get("auxiliary_welford") if isinstance(stats, dict) else None

    if not isinstance(instrument, dict) or not isinstance(stats, dict):
        raise ValueError("canonical CLOCKS restore missing clockfaces/stats")
    if not isinstance(auxiliary, dict):
        raise ValueError("canonical CLOCKS restore missing auxiliary_welford")
    if include_control and not isinstance(control, dict):
        raise ValueError("canonical CLOCKS restore missing control")

    out: Dict[str, Any] = {
        "restore_schema_version": 3,
        "restore_instrument_gnss_ns": instrument.get("gnss_ns"),
        "restore_instrument_dwt_cycles": instrument.get("dwt_cycles"),
        "restore_instrument_ocxo1_ns": instrument.get("ocxo1_ns"),
        "restore_instrument_ocxo2_ns": instrument.get("ocxo2_ns"),
        "restore_stats_valid": stats.get("valid"),
        "restore_stats_completed_row_coherent": stats.get("completed_row_coherent"),
        "restore_stats_reset_count": stats.get("reset_count"),
        "restore_stats_update_count": stats.get("update_count"),
        # These counters are non-scientific diagnostics and are no longer
        # serialized in CLOCKS_INSTRUMENT_STATS_V4.
        "restore_stats_vclock_interval_reject_count": 0,
        "restore_stats_ocxo1_interval_reject_count": 0,
        "restore_stats_ocxo2_interval_reject_count": 0,
    }

    welfords = {
        "gnss": _path_get(stats, "gnss.welford"),
        "dwt": _path_get(stats, "dwt.welford"),
        "vclock": _path_get(stats, "vclock.welford"),
        "ocxo1": _path_get(stats, "ocxo1.welford"),
        "ocxo2": _path_get(stats, "ocxo2.welford"),
        "pps_witness": auxiliary.get("pps_witness"),
        "ocxo1_dac": auxiliary.get("ocxo1_dac"),
        "ocxo2_dac": auxiliary.get("ocxo2_dac"),
    }
    for name, wf in welfords.items():
        if not isinstance(wf, dict):
            raise ValueError(f"canonical CLOCKS restore missing {name} Welford")
        for field in ("n", "mean", "m2", "min", "max"):
            out[f"restore_{name}_welford_{field}"] = wf.get(field)

    for lane in ("ocxo1", "ocxo2"):
        tau = _path_get(stats, f"{lane}_tau_state")
        if not isinstance(tau, dict):
            raise ValueError(f"canonical CLOCKS restore missing {lane} TAU state")
        for field in (
            "valid", "reset_count", "sample_count", "interval_count",
            "reject_count", "gap_reset_count", "last_pps_sequence",
            "last_interval_pps_sequence", "first_refined_ns", "last_refined_ns",
            "last_fast_residual_ns", "cumulative_reference_ns",
            "cumulative_clock_ns", "cumulative_clock_ns_exact", "mean_x",
            "mean_y", "sxx", "sxy", "syy", "interval_mean_ppb",
            "interval_m2_ppb",
        ):
            out[f"restore_{lane}_tau_{field}"] = tau.get(field)

        if include_control:
            lane_state = control.get(lane)
            servo = lane_state.get("servo") if isinstance(lane_state, dict) else None
            if not isinstance(lane_state, dict) or not isinstance(servo, dict):
                raise ValueError(f"canonical CLOCKS restore missing {lane} control/servo state")
            out[f"restore_{lane}_dac_value"] = lane_state.get("target_code")
            field_map = {
                "servo_last_step": "last_step",
                "servo_last_residual": "last_residual",
                "servo_settle_count": "settle_count",
                "servo_adjustments": "adjustments",
                "servo_predictor_initialized": "predictor_initialized",
                "servo_last_raw_residual": "last_raw_residual",
                "servo_filtered_residual": "filtered_residual",
                "servo_filtered_slope": "filtered_slope",
                "servo_predicted_residual": "predicted_residual",
                "servo_predictor_updates": "predictor_updates",
            }
            for command_field, json_field in field_map.items():
                out[f"restore_{lane}_dac_{command_field}"] = servo.get(json_field)

    if include_control:
        out["restore_servo_mode"] = control.get("servo_mode") or "OFF"
        out["restore_dither_operator_enabled"] = control.get(
            "dither_operator_enabled", False
        )

    missing = [key for key, value in out.items() if value is None]
    if missing:
        raise ValueError(f"canonical CLOCKS restore missing fields: {missing}")
    return out


def _canonical_instrument_restore_ready(
    clocks: Dict[str, Any],
    *,
    include_control: bool = True,
) -> bool:
    """True only when canonical CLOCKS_V4 instrument state is restore authority."""
    if not isinstance(clocks, dict):
        return False
    if clocks.get("schema") != "CLOCKS_INSTRUMENT_STATE_V1":
        return False
    if clocks.get("source_schema") != "CLOCKS_INSTRUMENT_V1":
        return False
    if not bool(clocks.get("snapshot_ok")) or not bool(clocks.get("valid")):
        return False
    if not bool(clocks.get("completed_row_coherent")):
        return False

    completed = _as_int(clocks.get("completed_pps_sequence"))
    clockfaces = clocks.get("clockfaces")
    stats = clocks.get("stats")
    if completed is None or completed <= 0 or not isinstance(clockfaces, dict):
        return False
    if not isinstance(stats, dict):
        return False
    if stats.get("schema") != "CLOCKS_INSTRUMENT_STATS_V4":
        return False
    if not bool(stats.get("snapshot_ok")) or not bool(stats.get("valid")):
        return False
    if not bool(stats.get("completed_row_coherent")):
        return False
    if _as_int(stats.get("last_pps_sequence")) != completed:
        return False

    update_count = _as_int(stats.get("update_count"))
    current_sequence = _as_int(stats.get("rolling_ppb_current_sequence"))
    if update_count is None or update_count < 0:
        return False
    if current_sequence is None or current_sequence < 0 or current_sequence > update_count:
        return False
    if "rolling_ppb_endpoint_admitted" not in stats:
        return False
    if "rolling_ppb_interval_advanced" not in stats:
        return False
    endpoint_admitted = bool(stats.get("rolling_ppb_endpoint_admitted"))
    interval_advanced = bool(stats.get("rolling_ppb_interval_advanced"))
    if interval_advanced and not endpoint_admitted:
        return False
    if endpoint_admitted and current_sequence != update_count:
        return False
    if _as_int(clockfaces.get("pps_sequence")) != completed:
        return False
    for key in ("gnss_ns", "dwt_cycles", "ocxo1_ns", "ocxo2_ns"):
        value = _as_int(clockfaces.get(key))
        if value is None or value <= 0:
            return False

    if include_control:
        control = clocks.get("control")
        if not isinstance(control, dict) or control.get("schema") != "CLOCKS_CONTROL_V1":
            return False

    try:
        _canonical_restore_args(clocks, include_control=include_control)
    except (TypeError, ValueError):
        return False
    return True


def _restore_gnss_raw_payload(gnss_raw: Dict[str, Any]) -> Dict[str, Any]:
    """Restore the complete Pi-owned always-on GNSS_RAW state."""
    global _gnss_raw_instrument_ns, _gnss_raw_instrument_n
    global _gnss_raw_instrument_valid
    global _gnss_raw_welford_n, _gnss_raw_welford_mean, _gnss_raw_welford_m2
    global _gnss_raw_welford_min, _gnss_raw_welford_max

    if not isinstance(gnss_raw, dict):
        return {"restored": False, "reason": "GNSS_RAW state is not an object"}
    instrument = gnss_raw.get("instrument")
    welford = gnss_raw.get("welford")
    if not isinstance(instrument, dict) or not isinstance(welford, dict):
        return {"restored": False, "reason": "missing GNSS_RAW instrument/welford"}

    try:
        instrument_ns = float(instrument.get("ns"))
        instrument_n = int(instrument.get("clockface_n"))
        n = int(welford.get("n"))
        mean = float(welford.get("mean"))
        m2 = float(welford.get("m2"))
        min_val = float(welford.get("min")) if n else 1e30
        max_val = float(welford.get("max")) if n else -1e30
    except (TypeError, ValueError) as exc:
        return {"restored": False, "reason": f"invalid GNSS_RAW scalar: {exc}"}

    finite = all(math.isfinite(v) for v in (instrument_ns, mean, m2))
    if n:
        finite = finite and math.isfinite(min_val) and math.isfinite(max_val)
    if (
        not finite
        or instrument_ns < 0.0
        or instrument_n < 0
        or n < 0
        or m2 < 0.0
        or (n and min_val > max_val)
    ):
        return {"restored": False, "reason": "GNSS_RAW state failed numeric court"}

    with _gnss_raw_stats_lock:
        _gnss_raw_instrument_ns = instrument_ns
        _gnss_raw_instrument_n = instrument_n
        _gnss_raw_instrument_valid = bool(instrument.get("valid")) and instrument_n > 0
        _gnss_raw_welford_n = n
        _gnss_raw_welford_mean = mean
        _gnss_raw_welford_m2 = m2
        _gnss_raw_welford_min = min_val
        _gnss_raw_welford_max = max_val

    return {
        "restored": True,
        "instrument_ns": int(round(instrument_ns)),
        "instrument_n": instrument_n,
        "welford_n": n,
        "welford_mean": mean,
        "welford_m2": m2,
    }






def _clocks_gnss_raw_payload(state: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """Return the complete Pi-owned GNSS_RAW state carried by CLOCKS."""
    gnss_raw = _clocks_payload(state).get("gnss_raw")
    return copy.deepcopy(gnss_raw) if isinstance(gnss_raw, dict) else None


def _read_latest_recoverable_clocks_state(
    *,
    scan_limit: int = 64,
) -> Tuple[Optional[Dict[str, Any]], int]:
    """Return the newest durable CLOCKS_V4 state with canonical restore authority."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, payload
            FROM campaign_detail
            WHERE campaign_type = %s
              AND NOT (payload @> '{"holistic_restore_superseded":true}'::jsonb)
            ORDER BY id DESC
            LIMIT %s
            """,
            (CAMPAIGN_TYPE_TEMPEST, int(scan_limit)),
        )
        rows = cur.fetchall()

    for skipped, row in enumerate(rows):
        state = row.get("payload") if isinstance(row, dict) else row[1]
        if isinstance(state, str):
            try:
                state = json.loads(state)
            except Exception:
                continue
        if not isinstance(state, dict):
            continue
        clocks = _clocks_payload(state)
        if not _canonical_instrument_restore_ready(clocks, include_control=True):
            continue
        if _clocks_gnss_raw_payload(state) is None:
            continue
        restored = copy.deepcopy(state)
        restored["_db_detail_id"] = int(row.get("id") if isinstance(row, dict) else row[0])
        return restored, int(skipped)
    return None, len(rows)

def _seed_clocks_from_detail(detail: Dict[str, Any]) -> None:
    """Publish and cache the last durable CLOCKS state without persisting it again."""
    seeded = copy.deepcopy(detail)
    seeded["restored_display_seed"] = True
    seeded["restored_display_seed_at_utc"] = (
        datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    )
    _cache_clocks_state(seeded)
    publish(CLOCKS_TOPIC, seeded)


def _request_teensy_holistic_restore(
    clocks: Dict[str, Any],
    *,
    allow_ppb_stage_required: bool = False,
) -> Dict[str, Any]:
    """Request one complete firmware CLOCKS restore from canonical instrument state.

    With allow_ppb_stage_required=True this also acts as the lifecycle probe used
    before SQL replay: an idle V3 firmware may truthfully answer that Better-
    Buckets history must be staged first, while a live campaign still returns its
    ordinary busy/live-custody verdict without Alpha being touched.
    """
    deadline = time.monotonic() + HOLISTIC_RESTORE_COMMAND_RETRY_S
    restore_args = _canonical_restore_args(clocks, include_control=True)
    last_response: Any = None
    while True:
        response = send_command(
            machine="TEENSY",
            subsystem="CLOCKS",
            command="RESTORE_MONITOR",
            args=restore_args,
        )
        last_response = response
        outer_success = isinstance(response, dict) and bool(response.get("success"))
        payload = response.get("payload") if isinstance(response, dict) else None
        status = str(payload.get("status") or "") if isinstance(payload, dict) else ""
        if outer_success and status in _TEENSY_MONITOR_RESTORE_ACCEPTED_STATUSES:
            return response
        if (
            outer_success
            and allow_ppb_stage_required
            and status == "monitor_restore_requires_ppb_state"
        ):
            return response
        if outer_success and status == "monitor_restore_rejected_busy":
            campaign_state = str(payload.get("campaign_state") or "").strip().upper()
            if campaign_state == "STARTED":
                return response
            if time.monotonic() < deadline:
                time.sleep(0.25)
                continue
        raise RuntimeError(
            "Teensy CLOCKS holistic restore rejected: "
            f"status={status or 'missing_handler_status'} response={last_response!r}"
        )

def _holistic_restore_probe(state: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """Project the canonical V4 sufficient state used to prove restore convergence."""
    clocks = _clocks_payload(state)
    instrument = clocks.get("clockfaces")
    stats = clocks.get("stats")
    control = clocks.get("control")
    instrument = instrument if isinstance(instrument, dict) else {}
    stats = stats if isinstance(stats, dict) else {}
    control = control if isinstance(control, dict) else {}
    auxiliary = stats.get("auxiliary_welford")
    auxiliary = auxiliary if isinstance(auxiliary, dict) else {}

    welford_n: Dict[str, int] = {}
    for lane in ("gnss", "dwt", "vclock", "ocxo1", "ocxo2"):
        node = stats.get(lane)
        wf = node.get("welford") if isinstance(node, dict) else None
        try:
            welford_n[lane] = int(wf.get("n") or 0) if isinstance(wf, dict) else 0
        except (TypeError, ValueError):
            welford_n[lane] = 0
    for lane in ("pps_witness", "ocxo1_dac", "ocxo2_dac"):
        wf = auxiliary.get(lane)
        try:
            welford_n[lane] = int(wf.get("n") or 0) if isinstance(wf, dict) else 0
        except (TypeError, ValueError):
            welford_n[lane] = 0

    def int_value(value: Any) -> int:
        try:
            return int(value or 0)
        except (TypeError, ValueError):
            return 0

    def float_value(value: Any) -> Optional[float]:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        return result if math.isfinite(result) else None

    return {
        "instrument_gnss_ns": int_value(instrument.get("gnss_ns")),
        "instrument_dwt_cycles": int_value(instrument.get("dwt_cycles")),
        "instrument_ocxo1_ns": int_value(instrument.get("ocxo1_ns")),
        "instrument_ocxo2_ns": int_value(instrument.get("ocxo2_ns")),
        "welford_n": welford_n,
        "servo_mode": str(control.get("servo_mode") or "OFF").upper(),
        "dither_operator_enabled": bool(control.get("dither_operator_enabled")),
        "ocxo1_dac": float_value(_path_get(control, "ocxo1.target_code")),
        "ocxo2_dac": float_value(_path_get(control, "ocxo2.target_code")),
    }


def _holistic_restore_probe_satisfied(
    expected: Dict[str, Any],
    observed: Dict[str, Any],
) -> bool:
    for key in (
        "instrument_gnss_ns", "instrument_dwt_cycles",
        "instrument_ocxo1_ns", "instrument_ocxo2_ns",
    ):
        target = int(expected.get(key) or 0)
        if target > 0 and int(observed.get(key) or 0) < target:
            return False
    expected_n = expected.get("welford_n")
    observed_n = observed.get("welford_n")
    if isinstance(expected_n, dict) and isinstance(observed_n, dict):
        for key, target_value in expected_n.items():
            target = int(target_value or 0)
            if target > 0 and int(observed_n.get(key) or 0) < target:
                return False
    if str(observed.get("servo_mode") or "OFF").upper() != str(
        expected.get("servo_mode") or "OFF"
    ).upper():
        return False
    if bool(observed.get("dither_operator_enabled")) != bool(
        expected.get("dither_operator_enabled")
    ):
        return False
    for key in ("ocxo1_dac", "ocxo2_dac"):
        target = expected.get(key)
        value = observed.get(key)
        if target is not None and (
            value is None or abs(float(value) - float(target)) > 0.01
        ):
            return False
    return True


def _holistic_restore_pending_categories(
    expected: Dict[str, Any],
    observed: Dict[str, Any],
) -> List[str]:
    pending: List[str] = []
    for key, category in {
        "instrument_gnss_ns": "GNSS_CLOCKFACE",
        "instrument_dwt_cycles": "DWT_CLOCKFACE",
        "instrument_ocxo1_ns": "OCXO_CLOCKFACES",
        "instrument_ocxo2_ns": "OCXO_CLOCKFACES",
    }.items():
        target = int(expected.get(key) or 0)
        if target > 0 and int(observed.get(key) or 0) < target and category not in pending:
            pending.append(category)
    expected_n = expected.get("welford_n")
    observed_n = observed.get("welford_n") if isinstance(observed.get("welford_n"), dict) else {}
    if isinstance(expected_n, dict) and any(
        int(target or 0) > 0 and int(observed_n.get(key) or 0) < int(target or 0)
        for key, target in expected_n.items()
    ):
        pending.append("STATISTICS")
    if not _holistic_restore_probe_satisfied(
        {**expected, "instrument_gnss_ns": 0, "instrument_dwt_cycles": 0,
         "instrument_ocxo1_ns": 0, "instrument_ocxo2_ns": 0, "welford_n": {}},
        observed,
    ):
        pending.append("DAC_CONTROL")
    return pending or ["FRESH_CLOCKS_PROOF"]


def _wait_for_holistic_restore(
    detail: Dict[str, Any],
    *,
    requested_monotonic: float,
    timeout_s: float = HOLISTIC_RESTORE_TIMEOUT_S,
) -> Dict[str, Any]:
    expected = _holistic_restore_probe(detail)
    deadline = time.monotonic() + float(timeout_s)
    next_progress_log = requested_monotonic + 10.0
    last_observed: Dict[str, Any] = {}
    while time.monotonic() < deadline:
        with _clocks_lock:
            current = copy.deepcopy(_latest_clocks)
            received = _latest_clocks_received_monotonic
        if received is not None and received > requested_monotonic and current:
            last_observed = _holistic_restore_probe(current)
            if _holistic_restore_probe_satisfied(expected, last_observed):
                return {
                    "proved": True,
                    "expected": expected,
                    "observed": last_observed,
                    "clocks_sequence": current.get("sequence"),
                    "waited_s": round(time.monotonic() - requested_monotonic, 3),
                }
        now = time.monotonic()
        if now >= next_progress_log:
            logging.info(
                "⏳ [holistic restore] converging after %.1fs; waiting for %s",
                now - requested_monotonic,
                ", ".join(_holistic_restore_pending_categories(expected, last_observed)),
            )
            next_progress_log = now + 10.0
        time.sleep(0.1)
    raise TimeoutError(
        "timed out waiting for holistic CLOCKS restore proof "
        f"after {timeout_s:.1f}s; expected={expected!r} observed={last_observed!r}"
    )


def _restore_instrument_from_clocks(detail: Dict[str, Any]) -> Dict[str, Any]:
    clocks = _clocks_payload(detail)
    gnss_raw = _clocks_gnss_raw_payload(detail)
    if not _canonical_instrument_restore_ready(clocks, include_control=True) or gnss_raw is None:
        raise RuntimeError("CLOCKS detail lacks valid canonical instrument/GNSS_RAW restore state")

    # First use RESTORE_MONITOR as a lifecycle probe. A Pi-only restart while a
    # live campaign still owns the Teensy must not touch Alpha's healthy rolling
    # history. An idle rebooted Teensy explicitly answers that Better-Buckets
    # state is required, and only then does the Pi replay PostgreSQL history.
    requested_monotonic = time.monotonic()
    teensy_response = _request_teensy_holistic_restore(
        clocks,
        allow_ppb_stage_required=True,
    )
    payload = teensy_response.get("payload") if isinstance(teensy_response, dict) else None
    payload = payload if isinstance(payload, dict) else {}
    status = str(payload.get("status") or "")
    campaign_state = str(payload.get("campaign_state") or "").strip().upper()
    if status == "monitor_restore_rejected_busy" and campaign_state == "STARTED":
        return {
            "success": True,
            "mode": "LIVE_CAMPAIGN_CUSTODY",
            "teensy": payload,
            "proof": {"proved": True, "basis": "TEENSY_LIVE_CAMPAIGN_CUSTODY"},
        }

    ppb_stage: Dict[str, Any]
    if status == "monitor_restore_requires_ppb_state":
        # The durable row remains the UI seed while the one-time cursor replay
        # and bounded chunk transfer reconstruct Alpha's volatile rings.
        _seed_clocks_from_detail(detail)
        ppb_replay = _reconstruct_better_buckets_from_db(detail)
        ppb_stage = _stage_teensy_better_buckets(ppb_replay)
        logging.info(
            "♻️ [holistic restore] Better-Buckets replay: rows=%d admitted=%d "
            "second=%d minute=%d checks=%d stage_s=%s",
            int(ppb_replay.get("rows_scanned") or 0),
            int(ppb_replay.get("admitted_rows") or 0),
            len(ppb_replay.get("second_history") or []),
            len(ppb_replay.get("minute_history") or []),
            int((ppb_replay.get("verification") or {}).get("checked") or 0),
            ppb_stage.get("waited_s"),
        )

        requested_monotonic = time.monotonic()
        try:
            teensy_response = _request_teensy_holistic_restore(clocks)
        except Exception:
            _abort_teensy_ppb_restore_best_effort()
            raise
        payload = (
            teensy_response.get("payload")
            if isinstance(teensy_response, dict)
            else None
        )
        payload = payload if isinstance(payload, dict) else {}
        status = str(payload.get("status") or "")
    else:
        # update_count==0 needs no rolling history. An immediately accepted V3
        # restore with a mature population can also mean a prior interrupted Pi
        # attempt already committed the exact Alpha stage; do not replay it twice.
        _seed_clocks_from_detail(detail)
        stats_update_count = _as_int(_path_get(clocks, "stats.update_count")) or 0
        ppb_stage = {
            "staged": bool(stats_update_count > 0),
            "reason": (
                "already_committed_on_teensy"
                if stats_update_count > 0
                else "empty_statistics_epoch"
            ),
            "rolling_sequence": int(stats_update_count),
        }

    if status not in _TEENSY_MONITOR_RESTORE_ACCEPTED_STATUSES:
        raise RuntimeError(
            "Teensy CLOCKS holistic restore did not reach requested state: "
            f"status={status or 'missing_handler_status'} payload={payload!r}"
        )

    gnss_raw_result = _restore_gnss_raw_payload(gnss_raw)
    if not gnss_raw_result.get("restored"):
        _abort_teensy_ppb_restore_best_effort()
        raise RuntimeError(f"GNSS_RAW restore failed: {gnss_raw_result}")
    proof = _wait_for_holistic_restore(
        detail,
        requested_monotonic=requested_monotonic,
    )
    return {
        "success": True,
        "mode": "HOLISTIC_INSTRUMENT",
        "teensy": payload,
        "gnss_raw": gnss_raw_result,
        "better_buckets": ppb_stage,
        "proof": proof,
    }


def _holistic_restore() -> Dict[str, Any]:
    """Restore the CLOCKS subsystem from one canonical state record.

    Campaign execution is not a separate startup recovery universe.  It is one
    conditional aspect of the restored CLOCKS state.
    """
    active_campaign = _get_active_campaign()
    detail, skipped = _read_latest_recoverable_clocks_state()
    result: Dict[str, Any] = {
        "schema": "PI_CLOCKS_HOLISTIC_RESTORE_V1",
        "active_campaign": active_campaign.get("campaign") if active_campaign else None,
        "skipped_unrecoverable_details": int(skipped),
        "instrument": None,
        "campaign": None,
    }

    try:
        if detail is not None:
            logging.info(
                "♻️ [holistic restore] restoring canonical CLOCKS sequence=%s "
                "(skipped_unrecoverable=%d)",
                detail.get("sequence"), skipped,
            )
            result["instrument"] = _restore_instrument_from_clocks(detail)
        else:
            logging.info(
                "ℹ️ [holistic restore] no structured-recoverable CLOCKS detail; "
                "continuing from live instrument state"
            )
    finally:
        # Persistence always resumes after the one restore transaction terminates.
        # Recoverability scans skip incomplete rows, so a failed restore cannot
        # permanently close the writer or erase older durable authority.
        _clocks_persistence_enabled.set()

    if active_campaign is not None:
        result["campaign"] = _restore_active_campaign_state()

    result["completed_at_utc"] = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    return result

def _tempest_campaign_name(campaign: Dict[str, Any]) -> str:
    return str(campaign.get("name") or "").strip()


def _tempest_public_count(campaign: Dict[str, Any]) -> int:
    value = _as_int(campaign.get("public_count"))
    if value is None or value < 0:
        raise ValueError("TEMPEST campaign enrichment missing valid public_count")
    return int(value)


def _tempest_candidate_view(campaign: Dict[str, Any]) -> Dict[str, Any]:
    """Build a transient processor view from TEMPEST_FRAGMENT_V1.

    This adapter is intentionally *not persisted*.  It lets the existing Pi
    final-court/recovery algorithms consume normalized names while durable
    CLOCKS retains the exact V4 campaign delta.
    """
    if not isinstance(campaign, dict):
        raise ValueError("TEMPEST campaign enrichment is not an object")
    if campaign.get("schema") != "TEMPEST_FRAGMENT_V1":
        raise ValueError(
            f"unsupported TEMPEST campaign schema {campaign.get('schema')!r}"
        )

    name = _tempest_campaign_name(campaign)
    if not name:
        raise ValueError("TEMPEST campaign enrichment missing name")
    public_count = _tempest_public_count(campaign)
    clockfaces = campaign.get("clockfaces")
    status = campaign.get("status")
    disposition = campaign.get("disposition")
    recovery = campaign.get("recovery")
    clockfaces = clockfaces if isinstance(clockfaces, dict) else {}
    status = status if isinstance(status, dict) else {}
    disposition = disposition if isinstance(disposition, dict) else {}
    recovery = recovery if isinstance(recovery, dict) else {}

    view: Dict[str, Any] = {
        "schema": campaign.get("schema"),
        "campaign": name,
        "campaign_state": campaign.get("state"),
        "public_count": public_count,
        # Internal legacy variable names below mean campaign-relative public count,
        # never CLOCKS_FRAGMENT.sequence.
        "teensy_pps_vclock_count": public_count,
        "teensy_pps_count": public_count,
        "pps_count": public_count,
        "gnss_ns": clockfaces.get("gnss_ns"),
        "dwt_cycles": clockfaces.get("dwt_cycles"),
        "ocxo1_ns": clockfaces.get("ocxo1_ns"),
        "ocxo2_ns": clockfaces.get("ocxo2_ns"),
        "timeline_valid": status.get("timeline_valid"),
        "ocxo_clockface_valid": status.get("ocxo_clockface_valid"),
        "ocxo_science_valid": status.get("ocxo_science_valid"),
        "candidate_disposition": disposition.get("status") or "ACCEPT",
        "candidate_use": disposition.get("use"),
        "science_eligible": disposition.get("science_eligible"),
        "control_eligible": disposition.get("control_eligible"),
        "science_excluded": not bool(disposition.get("science_eligible", True)),
        "candidate_reason_code": disposition.get("reason_code"),
        "candidate_reason_name": disposition.get("reason_name"),
        "candidate_reason": disposition.get("reason_name"),
        "candidate_source": disposition.get("source"),
        "candidate_reject_mask": disposition.get("lane_mask"),
        "candidate_lane": disposition.get("lane_mask"),
        "ocxo1": copy.deepcopy(campaign.get("ocxo1") or {}),
        "ocxo2": copy.deepcopy(campaign.get("ocxo2") or {}),
        "stats": copy.deepcopy(campaign.get("stats") or {}),
    }

    if recovery:
        view.update({
            "recover_generation": recovery.get("generation"),
            "recover_transition_active": recovery.get("transition_active"),
            "recover_timeline_ready": recovery.get("timeline_ready"),
            "recover_clockface_ready": recovery.get("clockface_ready"),
            "recover_science_ready": recovery.get("science_ready"),
            "recover_degraded_active": recovery.get("degraded_active"),
            "recover_science_quarantine_active": recovery.get(
                "science_quarantine_active"
            ),
            "recover_science_quarantine_remaining": recovery.get(
                "science_quarantine_remaining"
            ),
            "recover_reattach_stalled": recovery.get("reattach_stalled"),
        })
    return view


def _tempest_adjudication(detail: Dict[str, Any]) -> Dict[str, Any]:
    """Return only Pi-owned TEMPEST facts added to the firmware campaign delta."""
    return {
        "schema": "TEMPEST_ADJUDICATION_V1",
        "sequence": detail.get("sequence"),
        "public_count": detail.get("public_count"),
        "viable": bool(detail.get("science_eligible")),
        "science_eligible": bool(detail.get("science_eligible")),
        "control_eligible": bool(detail.get("control_eligible")),
        "science_excluded": bool(detail.get("science_excluded")),
        "candidate_use": detail.get("candidate_use"),
        "final_court": copy.deepcopy(detail.get("final_court") or {}),
        "location": detail.get("location"),
        "extra_clocks": copy.deepcopy(detail.get("extra_clocks") or {}),
        "adjudicated_at_utc": detail.get("system_time_utc"),
    }


def _attach_tempest_to_state_detail(detail: Dict[str, Any]) -> None:
    """Attach Pi adjudication to the already-persisted same-sequence V4 campaign.

    The firmware-authored ``campaign`` delta remains intact.  CLOCKS adds one
    ``campaign.adjudication`` child; it does not replace or clone the campaign.
    """
    sequence = _as_int(detail.get("sequence"))
    public_count = _as_int(detail.get("public_count"))
    if sequence is None or sequence <= 0:
        raise ValueError("TEMPEST state-detail attachment missing physical sequence")
    if public_count is None or public_count <= 0:
        raise ValueError("TEMPEST state-detail attachment missing campaign public_count")

    viable = bool(detail.get("science_eligible"))
    campaign = str(detail["campaign"])
    adjudication = _tempest_adjudication(detail)

    # campaign_master is an intentional read model and may retain the complete
    # accepted TIMEBASE report for list/baseline presentation.
    report = dict(detail)
    report["campaign_type"] = CAMPAIGN_TYPE_TEMPEST
    report["campaign_state"] = "STARTED"

    deadline = time.monotonic() + CAMPAIGN_DETAIL_ATTACH_TIMEOUT_S
    attempts = 0
    last_error: Optional[BaseException] = None

    while True:
        attempts += 1
        _diag["campaign_detail_attach_attempts"] += 1
        try:
            with open_db(row_dict=True) as conn:
                cur = conn.cursor()
                cur.execute(
                    """
                    UPDATE campaign_detail
                    SET viable = %s,
                        payload = jsonb_set(
                            payload,
                            '{campaign,adjudication}',
                            %s::jsonb,
                            true
                        )
                    WHERE id = (
                        SELECT id
                        FROM campaign_detail
                        WHERE campaign_type = %s
                          AND campaign = %s
                          AND sequence = %s
                        ORDER BY id DESC
                        LIMIT 1
                    )
                    RETURNING id
                    """,
                    (
                        viable,
                        json.dumps(adjudication),
                        CAMPAIGN_TYPE_TEMPEST,
                        campaign,
                        int(sequence),
                    ),
                )
                attached = cur.fetchone()
                if attached is not None:
                    cur.execute(
                        """
                        UPDATE campaign_master
                        SET payload = payload || jsonb_build_object('report', %s::jsonb)
                        WHERE campaign_type = %s
                          AND campaign = %s
                          AND active = true
                        """,
                        (json.dumps(report), CAMPAIGN_TYPE_TEMPEST, campaign),
                    )

            if attached is not None:
                _diag["campaign_detail_attach_success"] += 1
                _diag["campaign_detail_attach_retries"] += max(0, attempts - 1)
                _diag["last_campaign_detail_attach"] = {
                    "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                    "campaign": campaign,
                    "sequence": int(sequence),
                    "public_count": int(public_count),
                    "attempts": attempts,
                    "success": True,
                }
                return
        except Exception as exc:
            last_error = exc

        if time.monotonic() >= deadline:
            _diag["campaign_detail_attach_failures"] += 1
            _diag["campaign_detail_attach_retries"] += max(0, attempts - 1)
            _diag["last_campaign_detail_attach"] = {
                "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "campaign": campaign,
                "sequence": int(sequence),
                "public_count": int(public_count),
                "attempts": attempts,
                "success": False,
                "error": str(last_error) if last_error is not None else "matching CLOCKS detail not found",
            }
            logging.error(
                "⚠️ [clocks] failed to attach TEMPEST adjudication: "
                "campaign=%s sequence=%d public_count=%d attempts=%d error=%s",
                campaign,
                int(sequence),
                int(public_count),
                attempts,
                str(last_error) if last_error is not None else "matching CLOCKS detail not found",
            )
            return

        time.sleep(CAMPAIGN_DETAIL_ATTACH_POLL_S)





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
    """Confirm the unified always-on CLOCKS_FRAGMENT ingress route."""
    required = {CLOCKS_FRAGMENT_TOPIC}
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
                            "✅ [%s] CLOCKS_FRAGMENT route ready after %.1fs",
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
                "⏳ [%s] waiting for CLOCKS_FRAGMENT route (%.0fs): %s",
                context, elapsed, ", ".join(last_missing) or "route report unavailable",
            )
            logged_wait = True
            last_log = now
        time.sleep(poll_s)


def _parse_utc(value: Any) -> Optional[datetime]:
    if not isinstance(value, str) or not value.strip():
        return None
    try:
        return datetime.fromisoformat(value.strip().replace("Z", "+00:00")).astimezone(timezone.utc)
    except (TypeError, ValueError):
        return None








def _wait_for_gnss_time() -> str:
    """Wait until SYSTEM.REPORT carries a valid current GNSS UTC identity."""
    _diag["gnss_waits"] += 1
    started = time.monotonic()
    next_log = started
    while True:
        try:
            system_context = _fetch_system_report()
            value = _system_gnss_time_utc(system_context)
            gnss = _system_gnss_info(system_context)
            if value and gnss.get("time_valid", True):
                waited = time.monotonic() - started
                _diag["gnss_wait_success"] += 1
                _diag["gnss_wait_seconds_total"] += float(waited)
                _diag["gnss_wait_seconds_last"] = float(waited)
                _diag["last_gnss_wait"] = {
                    "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                    "waited_s": round(waited, 3),
                    "source": "SYSTEM.REPORT",
                }
                return value
        except Exception:
            pass
        now = time.monotonic()
        if now >= next_log:
            logging.info(
                "⏳ [clocks] waiting for SYSTEM.REPORT GNSS time... (%.0fs elapsed)",
                now - started,
            )
            next_log = now + GNSS_WAIT_LOG_INTERVAL
        time.sleep(0.5)





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


def _system_gnss_info(system_context: Dict[str, Any]) -> Dict[str, Any]:
    gnss = system_context.get("gnss") if isinstance(system_context, dict) else None
    return copy.deepcopy(gnss) if isinstance(gnss, dict) else {}


def _system_gnss_time_utc(system_context: Dict[str, Any]) -> Optional[str]:
    gnss = _system_gnss_info(system_context)
    for key in ("gnss_time_utc", "next_utc"):
        value = gnss.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    date = str(gnss.get("date") or "").strip()
    clock = str(gnss.get("time") or "").strip()
    return f"{date}T{clock}Z" if date and clock else None


def _environment_from_system_report(
    system_context: Dict[str, Any],
) -> Dict[str, Any]:
    """Project the established TEMPEST environment shape from SYSTEM.REPORT."""
    env = system_context.get("environment") if isinstance(system_context.get("environment"), dict) else {}
    gnss = _system_gnss_info(system_context)
    gnss_clock = gnss.get("clock") if isinstance(gnss.get("clock"), dict) else {}
    pi = system_context.get("pi") if isinstance(system_context.get("pi"), dict) else {}
    power = system_context.get("power") if isinstance(system_context.get("power"), dict) else {}
    battery = system_context.get("battery") if isinstance(system_context.get("battery"), dict) else {}

    i2c2 = power.get("i2c-2") if isinstance(power.get("i2c-2"), dict) else {}
    def rail(address: str) -> Dict[str, Any]:
        value = i2c2.get(address)
        return value if isinstance(value, dict) else {}

    return {
        "ambient_temp_c": env.get("temperature_c"),
        "pi_temp_c": pi.get("cpu_temp_c"),
        "gnss_temp_c": gnss_clock.get("temperature_c", gnss.get("temperature_c")),
        "barometric_altitude_m": env.get("altitude_m"),
        "gnss_altitude_m": gnss.get("altitude_m"),
        "ellipsoid_height_m": gnss.get("ellipsoid_height_m"),
        "geoid_sep_m": gnss.get("geoid_sep_m"),
        "pressure_hpa": env.get("pressure_hpa"),
        "humidity_pct": env.get("humidity_pct"),
        "pi_power": {k: rail("0x40").get(k) for k in ("volts", "amps", "watts")},
        "teensy_power": {k: rail("0x45").get(k) for k in ("volts", "amps", "watts")},
        "ocxo1_power": {k: rail("0x44").get(k) for k in ("volts", "amps", "watts")},
        "ocxo2_power": {k: rail("0x41").get(k) for k in ("volts", "amps", "watts")},
        "battery": {
            "remaining_pct": battery.get("remaining_pct"),
            "tte_minutes": battery.get("tte_minutes"),
            "wh_remaining": battery.get("wh_remaining_estimate"),
            "health_state": battery.get("health_state"),
        },
    }



def _gnss_raw_drift_from_info(info: Optional[Dict[str, Any]]) -> Optional[float]:
    """Return one finite receiver drift sample, or None when unavailable."""
    if not isinstance(info, dict):
        return None
    value = info.get("clock_drift_ppb")
    if value is None:
        return None
    try:
        sample = float(value)
    except (TypeError, ValueError):
        return None
    return sample if math.isfinite(sample) else None


def _gnss_raw_welford_snapshot() -> Dict[str, Any]:
    """Return one coherent snapshot of the always-on Pi Welford."""
    with _gnss_raw_stats_lock:
        n = int(_gnss_raw_welford_n)
        mean = float(_gnss_raw_welford_mean)
        m2 = float(_gnss_raw_welford_m2)
        min_val = float(_gnss_raw_welford_min)
        max_val = float(_gnss_raw_welford_max)
    stddev = math.sqrt(m2 / (n - 1)) if n >= 2 else 0.0
    stderr = stddev / math.sqrt(n) if n >= 2 else 0.0
    return {
        "n": n,
        "mean": mean,
        "m2": m2,
        "stddev": stddev,
        "stderr": stderr,
        "min": min_val if n else 0.0,
        "max": max_val if n else 0.0,
    }




def _clocks_baseline_set_cache(value: Optional[Dict[str, Any]]) -> None:
    global _clocks_baseline_cache
    global _clocks_baseline_refreshed_monotonic
    _clocks_baseline_cache = dict(value or {})
    _clocks_baseline_refreshed_monotonic = time.monotonic()


def _clocks_baseline_snapshot(*, force: bool = False) -> Dict[str, Any]:
    """Return bounded-refresh baseline state for the ephemeral monitor feed."""
    now = time.monotonic()
    due = (
        force
        or _clocks_baseline_refreshed_monotonic is None
        or now - _clocks_baseline_refreshed_monotonic
            >= CLOCKS_BASELINE_REFRESH_S
    )
    if due:
        try:
            _clocks_baseline_set_cache(_get_baseline_from_config())
            _diag["clocks_baseline_refresh_count"] += 1
        except Exception:
            _diag["clocks_baseline_refresh_failures"] += 1
            logging.debug("CLOCKS baseline refresh failed", exc_info=True)
    return dict(_clocks_baseline_cache)


def _gnss_raw_state_snapshot() -> Dict[str, Any]:
    """Return Pi-owned always-on GNSS_RAW instrument state for canonical CLOCKS."""
    with _gnss_raw_stats_lock:
        latest_info = dict(_gnss_raw_latest_info)
        latest_received = _gnss_raw_latest_info_monotonic
        instrument_ns = int(round(_gnss_raw_instrument_ns))
        instrument_n = int(_gnss_raw_instrument_n)
        instrument_valid = bool(_gnss_raw_instrument_valid)
    welford = _gnss_raw_welford_snapshot()

    instrument_ref_ns = instrument_n * NS_PER_SECOND
    age_s = (
        None
        if latest_received is None
        else max(0.0, time.monotonic() - latest_received)
    )
    instrument = {
        "valid": instrument_valid and instrument_ref_ns > 0,
        "ns": instrument_ns,
        "ref_ns": instrument_ref_ns,
        "clockface_n": instrument_n,
        "tau": (
            round(_compute_tau(instrument_ns, instrument_ref_ns), 12)
            if instrument_valid and instrument_ref_ns > 0 else None
        ),
        "ppb": (
            round(_compute_ppb(instrument_ns, instrument_ref_ns), 3)
            if instrument_valid and instrument_ref_ns > 0 else None
        ),
    }
    return {
        "owner": "PI",
        "clock": "GNSS_RAW",
        "always_on_statistics": True,
        "instrument": instrument,
        "drift_ppb": _gnss_raw_drift_from_info(latest_info),
        "welford": welford,
        "latest_info_age_s": None if age_s is None else round(age_s, 3),
        "latest_info_fresh": bool(
            age_s is not None and age_s <= GNSS_RAW_INFO_MAX_AGE_S
        ),
    }



# ---------------------------------------------------------------------
# Draconian sync primitives
# ---------------------------------------------------------------------


def _begin_sync_wait(expected_pps: int) -> None:
    """Prepare to wait for the first Pi-accepted CLOCKS_FRAGMENT campaign delta."""
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
        "⏳ [recovery] waiting for first accepted CLOCKS_FRAGMENT campaign delta >= %s",
        str(_sync_expected_pps_vclock),
    )

    t0 = time.monotonic()
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
                "last_firmware_status": (
                    dict(recovery_monitor.get("last_firmware_status") or {})
                    if recovery_monitor
                    else {}
                ),
                "firmware_no_progress_s": (
                    round(float(recovery_monitor.get("firmware_no_progress_s") or 0.0), 3)
                    if recovery_monitor
                    else 0.0
                ),
            }
            _diag["last_sync_wait"] = {
                "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "reason": "sync_timeout_waiting_for_fragment",
                **details,
            }
            _clear_sync_wait()
            raise RecoverySyncTimeout("sync_timeout_waiting_for_fragment", details)

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
                "✅ [recovery] first accepted CLOCKS_FRAGMENT campaign delta observed: count=%d expected>=%d",
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
    global _gnss_raw_instrument_ns, _gnss_raw_instrument_n
    global _gnss_raw_instrument_valid
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

    saved_instrument_ns = _first_float(
        extra.get("gnss_raw_instrument_ns"),
        last_tb.get("gnss_raw_instrument_ns"),
    )
    saved_instrument_n = _as_int(
        extra.get("gnss_raw_instrument_n")
        or last_tb.get("gnss_raw_instrument_n")
    )
    instrument_restored = False
    if (saved_instrument_ns is not None and saved_instrument_ns >= 0.0
            and saved_instrument_n is not None and saved_instrument_n > 0):
        with _gnss_raw_stats_lock:
            if int(saved_instrument_n) > int(_gnss_raw_instrument_n):
                _gnss_raw_instrument_ns = float(saved_instrument_ns)
                _gnss_raw_instrument_n = int(saved_instrument_n)
                _gnss_raw_instrument_valid = bool(
                    extra.get("gnss_raw_instrument_valid", True)
                )
                instrument_restored = True

    n = _as_int(extra.get("gnss_raw_welford_n") or last_tb.get("gnss_raw_welford_n"))
    mean = _first_float(extra.get("gnss_raw_welford_mean"), last_tb.get("gnss_raw_welford_mean")) or 0.0
    m2_value = _first_float(
        extra.get("gnss_raw_welford_m2"),
        last_tb.get("gnss_raw_welford_m2"),
    )
    stddev = _first_float(extra.get("gnss_raw_welford_stddev"), last_tb.get("gnss_raw_welford_stddev"))
    stderr_value = _first_float(extra.get("gnss_raw_welford_stderr"), last_tb.get("gnss_raw_welford_stderr"))
    min_val = _first_float(extra.get("gnss_raw_welford_min"), last_tb.get("gnss_raw_welford_min"))
    max_val = _first_float(extra.get("gnss_raw_welford_max"), last_tb.get("gnss_raw_welford_max"))

    welford_restored = False
    if n is not None:
        if stddev is None:
            stddev = (stderr_value * math.sqrt(float(n))) if stderr_value is not None and n > 0 else 0.0

        # On process restart the live population is empty, so restore the
        # durable generation. During an in-process recovery the always-on
        # sampler may already be newer; never roll N or moments backward.
        with _gnss_raw_stats_lock:
            if int(n) > int(_gnss_raw_welford_n):
                _gnss_raw_welford_n = int(n)
                _gnss_raw_welford_mean = float(mean)
                _gnss_raw_welford_m2 = (
                    float(m2_value)
                    if m2_value is not None and math.isfinite(float(m2_value)) and float(m2_value) >= 0.0
                    else ((float(stddev) * float(stddev) * float(n - 1)) if n >= 2 else 0.0)
                )
                _gnss_raw_welford_min = float(min_val) if min_val is not None else float(mean)
                _gnss_raw_welford_max = float(max_val) if max_val is not None else float(mean)
                welford_restored = True

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
        "instrument_restored": bool(instrument_restored),
        "instrument_n": int(_gnss_raw_instrument_n),
        "instrument_ns": int(round(_gnss_raw_instrument_ns)),
        "welford_restored": bool(welford_restored),
        "welford_n": int(_gnss_raw_welford_snapshot()["n"]),
        "welford_mean": round(float(_gnss_raw_welford_snapshot()["mean"]), 6),
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
        "recover_mode": str(status.get("recover_mode") or "NONE").upper(),
        "recover_cold_bootstrap_active": _recovery_bool(
            status.get("recover_cold_bootstrap_active")
        ),
        "recover_cold_bootstrap_epoch_ready": _recovery_bool(
            status.get("recover_cold_bootstrap_epoch_ready")
        ),
        "recover_cold_bootstrap_begin_count": _as_int(
            status.get("recover_cold_bootstrap_begin_count")
        ),
        "recover_cold_bootstrap_wait_count": _as_int(
            status.get("recover_cold_bootstrap_wait_count")
        ),
        "recover_cold_bootstrap_ready_count": _as_int(
            status.get("recover_cold_bootstrap_ready_count")
        ),
        "recover_cold_bootstrap_commit_count": _as_int(
            status.get("recover_cold_bootstrap_commit_count")
        ),
        "recover_smartzero_running": _recovery_bool(
            status.get("recover_smartzero_running")
        ),
        "recover_smartzero_complete": _recovery_bool(
            status.get("recover_smartzero_complete")
        ),
        "recover_epoch_ready": _recovery_bool(status.get("recover_epoch_ready")),
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

    # Preserve a compact stage transcript in the caller-owned monitor.  RECOVER
    # identity can remain perfectly stable while firmware is stuck at one
    # lifecycle stage, so base/expected-count equality alone is not enough
    # observability.  This is diagnostic only: it never manufactures progress
    # or aborts a lawful slow bootstrap.
    now_monotonic = time.monotonic()
    progress_signature = (
        compact.get("recover_lifecycle_reason"),
        compact.get("recover_cold_bootstrap_epoch_ready"),
        compact.get("recover_cold_bootstrap_commit_count"),
        compact.get("recover_reattach_active"),
        compact.get("recover_reattach_degraded_active"),
        compact.get("recover_clockface_ready"),
        compact.get("recover_science_ready"),
        compact.get("candidate_count"),
        compact.get("last_public_count"),
        compact.get("timebase_last_stage"),
    )
    if progress_signature != monitor.get("firmware_progress_signature"):
        monitor["firmware_progress_signature"] = progress_signature
        monitor["firmware_last_progress_monotonic"] = now_monotonic
        monitor["firmware_last_progress_status"] = dict(compact)
    last_progress = float(
        monitor.get("firmware_last_progress_monotonic") or now_monotonic
    )
    monitor["firmware_no_progress_s"] = max(0.0, now_monotonic - last_progress)
    monitor["last_firmware_status"] = dict(compact)

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

    gnss_ns = _fragment_int(fragment, "gnss_ns", default=0)
    dwt_cycles = _fragment_int(fragment, "dwt_cycles", default=0)
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
# Canonical V4 schema helpers
# ---------------------------------------------------------------------

def _path_get(mapping: Dict[str, Any], path: str) -> Any:
    """Return mapping[path] where path may be dotted, or None if absent."""
    cur: Any = mapping
    for part in path.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return None
        cur = cur.get(part)
    return cur


def _as_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _extract_last_timebase_count(last_tb: Dict[str, Any], fragment: Dict[str, Any]) -> int:
    """Return the campaign-relative public count from a TIMEBASE_V4 view."""
    value = _as_int(last_tb.get("public_count"))
    if value is None and isinstance(fragment, dict):
        value = _as_int(fragment.get("public_count"))
    if value is None:
        raise ValueError("persisted TEMPEST detail missing campaign public_count")
    return int(value)


def _first_present_int_path(mapping: Dict[str, Any], *keys: str) -> Optional[int]:
    for key in keys:
        value = _path_get(mapping, key) if "." in key else mapping.get(key)
        if value is not None:
            parsed = _as_int(value)
            if parsed is not None:
                return parsed
    return None


def _fragment_int(fragment: Dict[str, Any], *keys: str, default: int = 0) -> int:
    """Extract an integer field from a transient TEMPEST candidate view."""
    value = _first_present_int_path(fragment, *keys)
    return int(value) if value is not None else int(default)


def _fragment_ns(fragment: Dict[str, Any], *keys: str, default: int = 0) -> int:
    """Extract an integer nanosecond ledger from a transient TEMPEST candidate view."""
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
    """Validate one transient TEMPEST_FRAGMENT_V1 candidate view."""
    fragment = timebase.get("fragment")
    if not isinstance(fragment, dict):
        _timebase_final_court_add_violation(
            violations,
            rule="candidate_fragment_present",
            lane="candidate",
            message="TEMPEST candidate has no fragment object",
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

    outer_count = _as_int(timebase.get("public_count"))
    fragment_count = _as_int(fragment.get("public_count"))
    if outer_count is None or fragment_count is None or outer_count != fragment_count:
        _timebase_final_court_add_violation(
            violations,
            rule="candidate_identity_consistent",
            lane="candidate",
            message="TEMPEST public_count must agree across candidate envelope",
            fields={
                "outer_public_count": outer_count,
                "fragment_public_count": fragment_count,
            },
        )

    campaign = str(timebase.get("campaign") or "")
    fragment_campaign = str(fragment.get("campaign") or "")
    if not campaign or not fragment_campaign or fragment_campaign != campaign:
        _timebase_final_court_add_violation(
            violations,
            rule="candidate_campaign_consistent",
            lane="candidate",
            message="TEMPEST campaign identity does not match the active campaign",
            fields={
                "active_campaign": campaign,
                "fragment_campaign": fragment_campaign,
            },
        )

def _timebase_final_court_check_delta_raw_interval(
    timebase: Dict[str, Any],
    violations: List[Dict[str, Any]],
) -> None:
    """Validate V4 Delta Raw intervals exactly as they would be persisted."""
    public_count = _as_int(timebase.get("public_count"))
    for lane in ("ocxo1", "ocxo2"):
        science_path = f"fragment.{lane}.science"
        science = _path_get(timebase, science_path)
        if not isinstance(science, dict):
            continue
        if not _timebase_final_court_bool(science.get("delta_raw_valid")):
            continue

        clock_interval = _as_int(science.get("delta_raw_clock_interval_cycles"))
        reference_interval = _as_int(science.get("delta_raw_reference_interval_cycles"))
        common_fields: Dict[str, Any] = {
            "path": science_path,
            "public_count": public_count,
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
                message="delta_raw_valid is true but V4 science lacks required interval fields",
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
            _diag["timebase_final_court_delta_raw_offset_observed"] = (
                _diag.get("timebase_final_court_delta_raw_offset_observed", 0) + 1
            )
            _diag["last_timebase_final_court_delta_raw_offset"] = {
                **common_fields,
                "rule": "delta_raw_interval_offset_observed",
                "lane": lane,
                "clock_minus_reference_cycles": interval_error,
                "abs_clock_minus_reference_cycles": abs(interval_error),
                "fatal": False,
            }


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
    """Reject mature V4 rows only when an OCXO lane has no truthful evidence."""
    public_count = _as_int(timebase.get("public_count"))
    if public_count is None or public_count <= TIMEBASE_FINAL_COURT_OCXO_ZERO_MATURE_PUBLIC_COUNT:
        return

    recovery_degraded = _timebase_final_court_recovery_degraded_context(timebase)
    fragment = timebase.get("fragment")
    if not isinstance(fragment, dict):
        return
    shared_clockface_valid_raw = fragment.get("ocxo_clockface_valid")
    shared_clockface_valid = (
        None
        if shared_clockface_valid_raw is None
        else _timebase_final_court_bool(shared_clockface_valid_raw)
    )

    for lane in ("ocxo1", "ocxo2"):
        science_path = f"fragment.{lane}.science"
        science = _path_get(timebase, science_path)
        science = science if isinstance(science, dict) else {}
        lane_ns = _as_int(fragment.get(f"{lane}_ns"))
        clock_interval_ns = _as_int(science.get("clock_interval_ns"))
        gnss_interval_ns = _as_int(science.get("gnss_interval_ns"))
        delta_raw_clock_interval = _as_int(science.get("delta_raw_clock_interval_cycles"))
        science_valid = _timebase_final_court_bool(science.get("valid"))
        science_worthy = _timebase_final_court_bool(
            science.get("science_worthy", science.get("valid"))
        )
        antecedents_complete = _timebase_final_court_bool(science.get("antecedents_complete"))
        delta_raw_valid = _timebase_final_court_bool(science.get("delta_raw_valid"))

        if not recovery_degraded and (not science_worthy or not antecedents_complete):
            _timebase_final_court_add_violation(
                violations,
                rule="ocxo_science_valid",
                lane=lane,
                message="mature TEMPEST row lacks valid OCXO science custody",
                fields={
                    "path": f"fragment.{lane}",
                    "science_path": science_path,
                    "public_count": int(public_count),
                    "lane_ns": lane_ns,
                    "ocxo_clockface_valid": shared_clockface_valid,
                    "recovery_degraded": recovery_degraded,
                    "science_valid": science_valid,
                    "science_worthy": science_worthy,
                    "antecedents_complete": antecedents_complete,
                    "clock_interval_ns": clock_interval_ns,
                    "gnss_interval_ns": gnss_interval_ns,
                    "delta_raw_valid": delta_raw_valid,
                    "delta_raw_clock_interval_cycles": delta_raw_clock_interval,
                    "bad_field": f"fragment.{lane}.science.valid",
                    "bad_value": science.get("valid"),
                },
            )

        lane_ns_zero = lane_ns is None or lane_ns == 0
        has_nonzero_science_evidence = bool(
            science_valid
            or antecedents_complete
            or (clock_interval_ns is not None and clock_interval_ns != 0)
            or (delta_raw_clock_interval is not None and delta_raw_clock_interval != 0)
        )
        allow_missing_clockface = recovery_degraded and shared_clockface_valid is not True
        if lane_ns_zero and not has_nonzero_science_evidence and not allow_missing_clockface:
            _timebase_final_court_add_violation(
                violations,
                rule="ocxo_lane_alive",
                lane=lane,
                message="mature TEMPEST row contains an all-zero OCXO lane",
                fields={
                    "path": f"fragment.{lane}",
                    "science_path": science_path,
                    "public_count": int(public_count),
                    "lane_ns": lane_ns,
                    "ocxo_clockface_valid": shared_clockface_valid,
                    "recovery_degraded": recovery_degraded,
                    "allow_missing_clockface": allow_missing_clockface,
                    "clock_interval_ns": clock_interval_ns,
                    "gnss_interval_ns": gnss_interval_ns,
                    "delta_raw_clock_interval_cycles": delta_raw_clock_interval,
                    "science_valid": science_valid,
                    "antecedents_complete": antecedents_complete,
                    "bad_field": f"fragment.{lane}_ns",
                    "bad_value": lane_ns,
                },
            )


def _timebase_final_court_evaluate(timebase: Dict[str, Any]) -> Tuple[bool, Dict[str, Any]]:
    """Return (structurally_accepted, verdict) under the mode-free contract."""
    structural_violations: List[Dict[str, Any]] = []
    science_violations: List[Dict[str, Any]] = []

    # Structural failures mean the final object cannot truthfully identify the
    # campaign second. They remain the only drop/recovery class.
    _timebase_final_court_check_candidate_envelope(
        timebase, structural_violations
    )

    # Scientific objections never bury a coherent row. They only remove science
    # and control eligibility while preserving the complete audit record.
    _timebase_final_court_check_delta_raw_interval(timebase, science_violations)
    _timebase_final_court_check_ocxo_lane_alive(timebase, science_violations)

    # A missing mature OCXO lane is not merely questionable science: the Pi can
    # no longer prove that the lane/timeline identity survived. Promote only
    # that rule to continuity surrender; ordinary science-custody failures stay
    # in the persist-and-exclude class.
    promoted = [
        violation
        for violation in science_violations
        if violation.get("rule") == "ocxo_lane_alive"
    ]
    if promoted:
        structural_violations.extend(promoted)
        science_violations = [
            violation
            for violation in science_violations
            if violation.get("rule") != "ocxo_lane_alive"
        ]

    for violation in structural_violations:
        violation["severity"] = "continuity_fatal"
    for violation in science_violations:
        violation["severity"] = "science_exclude"

    fragment = timebase.get("fragment")
    firmware_exclusion = (
        _firmware_science_exclusion(fragment)
        if isinstance(fragment, dict)
        else {}
    )
    science_excluded = bool(firmware_exclusion or science_violations)
    accepted = not structural_violations

    recovery_degraded = _timebase_final_court_recovery_degraded_context(timebase)
    if recovery_degraded and accepted:
        _diag["timebase_final_court_degraded_recovery_admitted"] = (
            _diag.get("timebase_final_court_degraded_recovery_admitted", 0) + 1
        )

    public_count = _first_present_int_path(
        timebase,
        "public_count",
        "fragment.public_count",
    )
    sequence = _as_int(timebase.get("sequence"))

    if structural_violations:
        primary = structural_violations[0]
    elif firmware_exclusion:
        primary = {
            "rule": "firmware_row_objection",
            "message": firmware_exclusion.get("reason") or "firmware excluded row",
        }
    elif science_violations:
        primary = science_violations[0]
    else:
        primary = {}

    classification = (
        "DROP_CONTINUITY_FATAL"
        if not accepted
        else "ACCEPT_SCIENCE_EXCLUDE"
        if science_excluded
        else "ACCEPT"
    )

    verdict: Dict[str, Any] = {
        "schema": "PI_TEMPEST_FINAL_COURT_V1",
        "valid": accepted,
        "continuity_valid": accepted,
        "science_valid": not science_excluded,
        "science_eligible": accepted and not science_excluded,
        "control_eligible": accepted and not science_excluded,
        "persist": accepted,
        "science_excluded": science_excluded,
        "candidate_use": "AUDIT_ONLY" if science_excluded else "SCIENCE_AND_CONTROL",
        "classification": classification,
        "reason": (
            TIMEBASE_FINAL_COURT_VIOLATION_REASON
            if not accepted
            else "science_excluded"
            if science_excluded
            else "candidate_accepted"
        ),
        "primary_rule": primary.get("rule"),
        "rationale": primary.get("message") if primary else "candidate accepted",
        "source": TIMEBASE_FINAL_COURT_SOURCE,
        "source_process": "CLOCKS",
        "source_report": "TEMPEST_FINAL_COURT",
        "campaign": timebase.get("campaign"),
        "sequence": sequence,
        "public_count": public_count,
        "timebase_schema": timebase.get("schema"),
        "fragment_schema": _path_get(timebase, "fragment.schema"),
        "rule_count": 4,
        "violation_count": (
            len(structural_violations)
            + len(science_violations)
            + (1 if firmware_exclusion else 0)
        ),
        "fatal_violation_count": len(structural_violations),
        "structural_violation_count": len(structural_violations),
        "science_violation_count": len(science_violations),
        "firmware_objection_count": int(
            firmware_exclusion.get("objection_count") or 0
        ),
        "recovery_degraded_context": recovery_degraded,
        "firmware_exclusion": firmware_exclusion,
        "violations": structural_violations + science_violations,
        "structural_violations": structural_violations,
        "science_violations": science_violations,
    }

    return accepted, verdict


def _timebase_final_court_block(
    verdict: Dict[str, Any],
    *,
    raw_record: Dict[str, Any],
    assembled_timebase: Dict[str, Any],
) -> None:
    """Drop one structurally incoherent row and surrender continuity."""
    _diag["timebase_final_court_blocked"] += 1
    _diag["timebase_final_court_row_dropped"] = (
        _diag.get("timebase_final_court_row_dropped", 0) + 1
    )
    verdict["valid"] = False
    verdict["continuity_valid"] = False
    verdict["science_valid"] = False
    verdict["science_eligible"] = False
    verdict["control_eligible"] = False
    verdict["persist"] = False
    verdict["classification"] = "DROP_CONTINUITY_FATAL"
    verdict["court_classification"] = "DROP_CONTINUITY_FATAL"
    verdict["continuity_fatal"] = True
    _diag["last_timebase_final_court"] = verdict
    _diag["last_timebase_final_court_row_drop"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "campaign": verdict.get("campaign"),
        "public_count": verdict.get("public_count"),
        "primary_rule": verdict.get("primary_rule"),
        "rationale": verdict.get("rationale"),
        "verdict": verdict,
    }

    _log_invalid_timebase(
        verdict=verdict,
        raw_record=raw_record,
        assembled_timebase=assembled_timebase,
    )

    recovery_started = False
    if _campaign_active or _auto_recovery_in_progress:
        recovery_started = _begin_auto_recovery(
            "timebase_structural_integrity_failure",
            {
                "campaign": verdict.get("campaign"),
                "public_count": verdict.get("public_count"),
                "primary_rule": verdict.get("primary_rule"),
                "rationale": verdict.get("rationale"),
                "structural_violations": verdict.get("structural_violations"),
            },
            source=TIMEBASE_FINAL_COURT_SOURCE,
        )
        if recovery_started:
            _diag["timebase_final_court_recovery_started"] += 1
    verdict["recovery_requested"] = recovery_started

    logging.error(
        "💥 [clocks] structurally incoherent TIMEBASE candidate dropped: "
        "campaign=%s pps=%s rule=%s rationale=%s recovery_started=%s",
        verdict.get("campaign"),
        verdict.get("public_count"),
        verdict.get("primary_rule"),
        verdict.get("rationale"),
        recovery_started,
    )

    try:
        create_event("TIMEBASE_CONTINUITY_FATAL", verdict)
    except Exception:
        _diag["timebase_final_court_event_enqueue_failures"] += 1
        logging.debug(
            "⚠️ [clocks] failed to enqueue TIMEBASE_CONTINUITY_FATAL event",
            exc_info=True,
        )


def _dwt_cycles_to_recover_ns(cycles: int) -> int:
    """Convert a DWT cycle ledger to the current Teensy RECOVER dwt_ns argument.

    TEMPEST_FRAGMENT_V1 publishes DWT as native cycles.  The current firmware
    RECOVER command still accepts dwt_ns and immediately converts it back to
    cycles.  Use a ceiling conversion so the round trip does not restore a
    cycle ledger that is one cycle below the projected value.
    """
    c = int(cycles)
    if c <= 0:
        return 0
    return (c * DWT_NS_NUM + DWT_NS_DEN - 1) // DWT_NS_DEN


def _report_fragment(report: Dict[str, Any]) -> Dict[str, Any]:
    frag = report.get("fragment")
    return frag if isinstance(frag, dict) else {}


def _report_extra_clocks(report: Dict[str, Any]) -> Dict[str, Any]:
    extra = report.get("extra_clocks")
    return extra if isinstance(extra, dict) else {}


def _recovery_timebase_snapshot(tb: Dict[str, Any]) -> Dict[str, Any]:
    """Extract warm-recovery truth from one persisted TIMEBASE_V4 view."""
    last_tb = dict(tb or {})
    last_frag = _report_fragment(last_tb)
    last_extra = _report_extra_clocks(last_tb)
    canonical_clocks = last_tb.get("clocks")
    canonical_clocks = (
        copy.deepcopy(canonical_clocks)
        if isinstance(canonical_clocks, dict)
        else {}
    )

    last_public_count = int(_extract_last_timebase_count(last_tb, last_frag))

    last_gnss_ns = _fragment_ns(
        last_frag,
        "clockfaces.gnss_ns",
        "gnss_ns",
        default=0,
    )
    last_dwt_cycles = _fragment_int(
        last_frag,
        "clockfaces.dwt_cycles",
        "dwt_cycles",
        default=0,
    )
    last_dwt_ns = _dwt_cycles_to_recover_ns(last_dwt_cycles) if last_dwt_cycles > 0 else 0
    last_ocxo1_ns = _fragment_ns(
        last_frag,
        "clockfaces.ocxo1_ns",
        "ocxo1_ns",
        default=0,
    )
    last_ocxo2_ns = _fragment_ns(
        last_frag,
        "clockfaces.ocxo2_ns",
        "ocxo2_ns",
        default=0,
    )

    last_gnss_raw_ns = int(
        last_extra.get("gnss_raw_ns")
        or last_tb.get("gnss_raw_ns")
        or 0
    )
    last_gnss_raw_ref_ns = int(
        last_extra.get("gnss_raw_ref_ns")
        or last_tb.get("gnss_raw_ref_ns")
        or (last_public_count * NS_PER_SECOND)
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

    canonical_restore_ok = _canonical_instrument_restore_ready(
        canonical_clocks,
        include_control=False,
    )

    recoverable = (
        last_public_count > 0
        and last_gnss_ns > 0
        and last_dwt_cycles > 0
        and last_ocxo1_ns > 0
        and last_ocxo2_ns > 0
        and canonical_restore_ok
    )

    return {
        "last_tb": last_tb,
        "last_frag": last_frag,
        "last_pps_vclock_count": int(last_public_count),
        "last_public_count": int(last_public_count),
        "last_gnss_ns": int(last_gnss_ns),
        "legacy_last_dwt_ns": 0,
        "last_dwt_cycles": int(last_dwt_cycles),
        "last_dwt_ns": int(last_dwt_ns),
        "last_ocxo1_ns": int(last_ocxo1_ns),
        "last_ocxo2_ns": int(last_ocxo2_ns),
        "last_gnss_raw_ns": int(last_gnss_raw_ns),
        "last_gnss_raw_ref_ns": int(last_gnss_raw_ref_ns),
        "last_gnss_raw_welford_mean": float(last_gnss_raw_welford_mean),
        "last_gnss_time_str": str(last_gnss_time_str),
        "canonical_clocks": canonical_clocks,
        "recoverable": bool(recoverable),
    }



def _state_campaign(state: Dict[str, Any]) -> Dict[str, Any]:
    """Return the canonical optional TEMPEST campaign delta from CLOCKS_V4."""
    campaign = state.get("campaign")
    return campaign if isinstance(campaign, dict) else {}


def _tempest_detail_from_state_snapshot(state: Dict[str, Any]) -> Dict[str, Any]:
    """Reconstruct the operator/recovery TIMEBASE_V4 view from one CLOCKS_V4 row."""
    campaign = _state_campaign(state)
    if not campaign:
        raise ValueError("state detail has no campaign enrichment")
    if campaign.get("schema") != "TEMPEST_FRAGMENT_V1":
        raise ValueError("state detail campaign is not TEMPEST_FRAGMENT_V1")

    adjudication = campaign.get("adjudication")
    if not isinstance(adjudication, dict):
        raise ValueError("state detail campaign has no Pi adjudication")

    fragment = copy.deepcopy(campaign)
    fragment.pop("adjudication", None)
    public_count = _tempest_public_count(fragment)
    sequence = _as_int(state.get("sequence"))
    if sequence is None or sequence <= 0:
        raise ValueError("state detail has no physical sequence")

    clocks = state.get("clocks")
    clocks = copy.deepcopy(clocks) if isinstance(clocks, dict) else {}
    campaign_name = _tempest_campaign_name(fragment)

    return {
        "schema": "TIMEBASE_V4",
        "campaign_type": CAMPAIGN_TYPE_TEMPEST,
        "campaign": campaign_name,
        "sequence": int(sequence),
        "public_count": int(public_count),
        "campaign_elapsed": _seconds_to_hms(int(public_count)),
        "science_eligible": bool(adjudication.get("science_eligible")),
        "control_eligible": bool(adjudication.get("control_eligible")),
        "persist": True,
        "science_excluded": bool(adjudication.get("science_excluded")),
        "candidate_use": adjudication.get("candidate_use"),
        "final_court": copy.deepcopy(adjudication.get("final_court") or {}),
        "location": adjudication.get("location"),
        "system_time_utc": (
            adjudication.get("adjudicated_at_utc")
            or state.get("published_at_utc")
        ),
        "gnss_time_utc": (
            clocks.get("gnss_time_utc")
            or _path_get(state, "gnss.gnss_time_utc")
            or _path_get(state, "gnss.next_utc")
        ),
        "fragment": fragment,
        "clocks": clocks,
        "environment": copy.deepcopy(state.get("environment")),
        "gnss": copy.deepcopy(state.get("gnss")),
        "extra_clocks": copy.deepcopy(adjudication.get("extra_clocks") or {}),
    }


def _count_tempest_state_details(campaign_name: str) -> int:
    """Return unified CLOCKS rows carrying a fully adjudicated TEMPEST delta."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT COUNT(*) AS cnt
            FROM campaign_detail
            WHERE campaign_type = %s
              AND campaign = %s
              AND payload #> '{campaign,adjudication}' IS NOT NULL
            """,
            (
                CAMPAIGN_TYPE_TEMPEST,
                campaign_name,
            ),
        )
        row = cur.fetchone()
    return int(row["cnt"] if row else 0)


def _load_last_recoverable_tempest_detail(
    campaign_name: str,
    *,
    scan_limit: int = 64,
) -> Tuple[Dict[str, Any], Dict[str, Any], int]:
    """Return the newest adjudicated CLOCKS_V4 TEMPEST state usable for recovery."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, payload
            FROM campaign_detail
            WHERE campaign_type = %s
              AND campaign = %s
              AND viable = true
              AND payload #> '{campaign,adjudication}' IS NOT NULL
            ORDER BY id DESC
            LIMIT %s
            """,
            (
                CAMPAIGN_TYPE_TEMPEST,
                campaign_name,
                int(scan_limit),
            ),
        )
        rows = cur.fetchall()

    if not rows:
        raise LookupError("missing_tempest_state_detail")

    newest_detail: Optional[Dict[str, Any]] = None
    newest_snapshot: Optional[Dict[str, Any]] = None
    newest_error: Optional[str] = None

    for skipped, row in enumerate(rows):
        state = row["payload"]
        if isinstance(state, str):
            state = json.loads(state)
        if not isinstance(state, dict):
            continue

        try:
            detail = _tempest_detail_from_state_snapshot(state)
            detail["_db_detail_id"] = int(row["id"])
            snapshot = _recovery_timebase_snapshot(detail)
        except Exception as exc:
            if newest_error is None:
                newest_error = str(exc)
            continue

        if newest_detail is None:
            newest_detail = detail
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
                    "⚠️ [recovery] skipped %d latest unified TEMPEST state "
                    "detail(s) with unrecoverable canonical state; using public_count=%s",
                    skipped,
                    snapshot.get("last_public_count"),
                )
            return detail, snapshot, int(skipped)

    if newest_detail is not None and newest_snapshot is not None:
        return newest_detail, newest_snapshot, 0

    raise RuntimeError(
        "recovery failed: no parseable unified TEMPEST state details "
        f"({newest_error or 'unknown error'})"
    )

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


def _firmware_total_ppb(
    fragment: Dict[str, Any],
    report: Dict[str, Any],
    lane: str,
) -> Optional[float]:
    """Return canonical always-on firmware TOTAL PPB for one lane."""
    return _first_float(
        _path_get(report, f"clocks.stats.{lane}.ppb_buckets.total"),
        _path_get(report, f"clocks.stats.{lane}.ppb"),
        _path_get(fragment, f"stats.{lane}.ppb_buckets.total"),
    )


def _baseline_ppb_from_report(report: Dict[str, Any]) -> Dict[str, float]:
    """Extract explicit TOTAL PPB values from a TIMEBASE-shaped report."""
    frag = _report_fragment(report)

    candidates = {
        "gnss": 0.0,
        "vclock": _firmware_total_ppb(frag, report, "vclock"),
        "gnss_raw": _gnss_raw_baseline_ppb_from_report(report),
        "dwt": _firmware_total_ppb(frag, report, "dwt"),
        "ocxo1": _firmware_total_ppb(frag, report, "ocxo1"),
        "ocxo2": _firmware_total_ppb(frag, report, "ocxo2"),
    }

    return {k: round(v, 3) for k, v in candidates.items() if v is not None}


def _baseline_tau_from_report(report: Dict[str, Any]) -> Dict[str, float]:
    """Extract tau values derived from the same explicit TOTAL population."""
    frag = _report_fragment(report)
    extra = _report_extra_clocks(report)
    gnss_raw_ppb = _gnss_raw_baseline_ppb_from_report(report)

    candidates = {
        "gnss": 1.0,
        "vclock": _tau_from_ppb(_firmware_total_ppb(frag, report, "vclock")),
        "gnss_raw": _first_float(extra.get("gnss_raw_tau"), report.get("gnss_raw_tau")),
        "dwt": _tau_from_ppb(_firmware_total_ppb(frag, report, "dwt")),
        "ocxo1": _tau_from_ppb(_firmware_total_ppb(frag, report, "ocxo1")),
        "ocxo2": _tau_from_ppb(_firmware_total_ppb(frag, report, "ocxo2")),
    }
    candidates["gnss_raw"] = _tau_from_ppb(gnss_raw_ppb) or candidates.get("gnss_raw")

    return {k: round(v, 12) for k, v in candidates.items() if v is not None}


def _baseline_dac_from_report(report: Dict[str, Any]) -> Dict[str, float]:
    """Extract canonical instantaneous DAC targets from TIMEBASE_V4.clocks.control."""
    out: Dict[str, float] = {}
    for key in ("ocxo1", "ocxo2"):
        value = _first_float(_path_get(report, f"clocks.control.{key}.target_code"))
        if value is not None:
            out[key] = round(value, 6)
    return out


def _baseline_dac_mean_from_report(report: Dict[str, Any]) -> Dict[str, float]:
    """Extract canonical DAC Welford means, falling back to live target code."""
    current = _baseline_dac_from_report(report)
    out: Dict[str, float] = {}
    for key in ("ocxo1", "ocxo2"):
        current_v = current.get(key)
        value = _first_float(
            _path_get(report, f"clocks.stats.auxiliary_welford.{key}_dac.mean"),
            current_v,
        )
        if value == 0.0 and current_v is not None and current_v > 0.0:
            value = current_v
        if value is not None:
            out[key] = round(value, 6)
    return out


def _baseline_dac_stats_from_report(report: Dict[str, Any]) -> Dict[str, Dict[str, float | int]]:
    """Extract canonical DAC Welford blocks for baseline audit/display."""
    out: Dict[str, Dict[str, float | int]] = {}
    for key in ("ocxo1", "ocxo2"):
        wf = _path_get(report, f"clocks.stats.auxiliary_welford.{key}_dac")
        if not isinstance(wf, dict):
            continue
        stats: Dict[str, float | int] = {}
        n = _as_int(wf.get("n"))
        if n is not None:
            stats["n"] = int(n)
        for field in ("mean", "stddev", "stderr", "min", "max"):
            value = _first_float(wf.get(field))
            if value is not None:
                stats[field] = round(value, 6)
        if stats:
            out[key] = stats
    return out


def _normalize_start_args(args: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """Return campaign lifecycle arguments without control-plane rewriting."""
    return dict(args or {})




# ---------------------------------------------------------------------
# Asynchronous START helpers
# ---------------------------------------------------------------------


def _mark_start_waiting(campaign: str) -> None:
    """Record that START returned before the first CLOCKS_FRAGMENT campaign delta."""
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
    """Close the START wait window on the first accepted MONITOR campaign row."""
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
        "✅ [start] @%s first CLOCKS campaign row accepted "
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
    or the holistic startup restore. STOP, REPORT, CLEAR, and recovery
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


def _enqueue_timebase_piece(
    topic: str,
    payload: Dict[str, Any],
    *,
    state_sequence: int,
    system_context: Optional[Dict[str, Any]] = None,
    clocks_fragment: Optional[Dict[str, Any]] = None,
) -> None:
    """Enqueue one already-persisted V4 campaign delta for TEMPEST adjudication."""
    _fragment_queue.put({
        "topic": topic,
        "payload": copy.deepcopy(payload),
        "state_sequence": int(state_sequence),
        "system_context": copy.deepcopy(system_context)
            if isinstance(system_context, dict) else {},
        "clocks_fragment": copy.deepcopy(clocks_fragment)
            if isinstance(clocks_fragment, dict) else {},
        "received_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
    })

    depth = _fragment_queue.qsize()
    _diag["queue_depth_current"] = depth
    if depth > _diag["queue_depth_max_seen"]:
        _diag["queue_depth_max_seen"] = depth



def _drain_timebase_ingress() -> int:
    """Drain queued CLOCKS_FRAGMENT campaign deltas from an old lifecycle."""
    drained = 0
    while not _fragment_queue.empty():
        try:
            _fragment_queue.get_nowait()
            drained += 1
        except queue.Empty:
            break

    return drained


def _drain_clocks_persistence_queue() -> int:
    """Discard not-yet-durable CLOCKS rows before destructive history commands."""
    drained = 0
    while not _clocks_persist_queue.empty():
        try:
            _clocks_persist_queue.get_nowait()
            drained += 1
        except queue.Empty:
            break
    return drained


# ---------------------------------------------------------------------
# Canonical CLOCKS state construction and persistence
# ---------------------------------------------------------------------

def _deep_merge_dicts(*values: Any) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for value in values:
        if not isinstance(value, dict):
            continue
        for key, item in value.items():
            if isinstance(item, dict) and isinstance(result.get(key), dict):
                result[key] = _deep_merge_dicts(result[key], item)
            else:
                result[key] = copy.deepcopy(item)
    return result


def _clocks_fragment_count(fragment: Dict[str, Any]) -> Optional[int]:
    """Return CLOCKS_FRAGMENT_V4 physical completed-row identity."""
    value = _as_int(fragment.get("sequence")) if isinstance(fragment, dict) else None
    return int(value) if value is not None else None


def _advance_gnss_raw_instrument(
    sequence: Optional[int],
    system_context: Dict[str, Any],
) -> None:
    """Advance GNSS_RAW exactly once per physical CLOCKS_FRAGMENT sequence."""
    global _last_clocks_state_sequence, _last_clocks_state_monotonic
    global _gnss_raw_welford_n, _gnss_raw_welford_mean, _gnss_raw_welford_m2
    global _gnss_raw_welford_min, _gnss_raw_welford_max
    global _gnss_raw_latest_info, _gnss_raw_latest_info_monotonic
    global _gnss_raw_instrument_ns, _gnss_raw_instrument_n, _gnss_raw_instrument_valid

    now = time.monotonic()
    if sequence is not None and _last_clocks_state_sequence is not None:
        if int(sequence) == int(_last_clocks_state_sequence):
            # V4 may strengthen an instrument-only row with campaign enrichment.
            # Same physical sequence is idempotent regardless of wall-clock delay.
            _last_clocks_state_monotonic = now
            return
        if int(sequence) < int(_last_clocks_state_sequence):
            logging.warning(
                "⚠️ [clocks] ignoring GNSS_RAW advance for regressed physical sequence "
                "%d < %d",
                int(sequence),
                int(_last_clocks_state_sequence),
            )
            return

    info = _system_gnss_info(system_context)
    sample = _gnss_raw_drift_from_info(info)
    with _gnss_raw_stats_lock:
        _gnss_raw_latest_info = dict(info)
        _gnss_raw_latest_info_monotonic = now
        _gnss_raw_instrument_ns += NS_PER_SECOND + (sample if sample is not None else 0.0)
        _gnss_raw_instrument_n += 1
        _gnss_raw_instrument_valid = True
        if sample is not None:
            _gnss_raw_welford_n += 1
            d1 = sample - _gnss_raw_welford_mean
            _gnss_raw_welford_mean += d1 / _gnss_raw_welford_n
            d2 = sample - _gnss_raw_welford_mean
            _gnss_raw_welford_m2 += d1 * d2
            _gnss_raw_welford_min = min(_gnss_raw_welford_min, sample)
            _gnss_raw_welford_max = max(_gnss_raw_welford_max, sample)
        n = int(_gnss_raw_welford_n)

    _diag["gnss_raw_stats_poll_count"] += 1
    if sample is None:
        _diag["gnss_raw_stats_missing_count"] += 1
    else:
        _diag["gnss_raw_stats_sample_count"] += 1
        _diag["last_gnss_raw_stats_sample"] = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "drift_ppb": round(float(sample), 6),
            "n": n,
            "source": "SYSTEM.REPORT",
        }

    if sequence is not None:
        _last_clocks_state_sequence = int(sequence)
        _last_clocks_state_monotonic = now



def _pi_clocks_state_snapshot() -> Dict[str, Any]:
    """Return Pi-owned CLOCKS enrichment without mirroring Teensy instrument state."""
    gnss_raw = _gnss_raw_state_snapshot()
    baseline = _clocks_baseline_snapshot()
    baseline_payload = (
        {"baseline_set": True, **baseline}
        if baseline
        else {"baseline_set": False}
    )
    return {
        "schema": "PI_CLOCKS_STATE_V4",
        "gnss_raw": gnss_raw,
        "baseline": baseline_payload,
        "stats_reset": {
            "requests": int(_diag.get("stats_reset_requests") or 0),
            "success": int(_diag.get("stats_reset_success") or 0),
            "last": _diag.get("last_stats_reset") or {},
        },
        "startup": _start_status_payload(),
    }


def _validate_clocks_fragment_v4(
    clocks_fragment: Dict[str, Any],
) -> Tuple[int, Dict[str, Any]]:
    """Return (physical_sequence, clocks) only for one coherent V4 instrument row."""
    if clocks_fragment.get("schema") != "CLOCKS_FRAGMENT_V4":
        raise ValueError(
            f"unsupported CLOCKS_FRAGMENT schema {clocks_fragment.get('schema')!r}"
        )

    sequence = _clocks_fragment_count(clocks_fragment)
    teensy_clocks = clocks_fragment.get("clocks")
    if sequence is None or sequence <= 0 or not isinstance(teensy_clocks, dict):
        raise ValueError("CLOCKS_FRAGMENT_V4 missing sequence/clocks")

    completed = _as_int(teensy_clocks.get("completed_pps_sequence"))
    clockface_sequence = _as_int(_path_get(teensy_clocks, "clockfaces.pps_sequence"))
    if (
        completed != int(sequence)
        or clockface_sequence != int(sequence)
        or not bool(teensy_clocks.get("completed_row_coherent"))
    ):
        raise ValueError(
            "CLOCKS_FRAGMENT_V4 identity/coherence violation: "
            f"outer={sequence} completed={completed} clockface={clockface_sequence} "
            f"coherent={teensy_clocks.get('completed_row_coherent')!r}"
        )

    stats = teensy_clocks.get("stats")
    if not isinstance(stats, dict) or stats.get("schema") != "CLOCKS_INSTRUMENT_STATS_V4":
        raise ValueError("CLOCKS_FRAGMENT_V4 missing CLOCKS_INSTRUMENT_STATS_V4")
    update_count = _as_int(stats.get("update_count"))
    current_sequence = _as_int(stats.get("rolling_ppb_current_sequence"))
    if update_count is None or update_count < 0:
        raise ValueError("CLOCKS_FRAGMENT_V4 invalid stats.update_count")
    if current_sequence is None or current_sequence < 0 or current_sequence > update_count:
        raise ValueError("CLOCKS_FRAGMENT_V4 invalid rolling_ppb_current_sequence")
    if "rolling_ppb_endpoint_admitted" not in stats or "rolling_ppb_interval_advanced" not in stats:
        raise ValueError("CLOCKS_FRAGMENT_V4 missing Better-Buckets custody witnesses")
    endpoint_admitted = bool(stats.get("rolling_ppb_endpoint_admitted"))
    interval_advanced = bool(stats.get("rolling_ppb_interval_advanced"))
    if interval_advanced and not endpoint_admitted:
        raise ValueError("CLOCKS_FRAGMENT_V4 interval advanced on excluded endpoint")
    if endpoint_admitted and current_sequence != update_count:
        raise ValueError(
            "CLOCKS_FRAGMENT_V4 admitted Better-Buckets endpoint does not own update_count"
        )
    return int(sequence), teensy_clocks


def _build_canonical_clocks_state(
    clocks_fragment: Dict[str, Any],
    system_context: Dict[str, Any],
) -> Dict[str, Any]:
    """Build canonical CLOCKS_V4 from one exact CLOCKS_FRAGMENT_V4 observation."""
    published_at = datetime.now(timezone.utc)
    published_at_utc = published_at.isoformat().replace("+00:00", "Z")
    sequence, teensy_clocks = _validate_clocks_fragment_v4(clocks_fragment)

    gnss = _system_gnss_info(system_context)
    pi_clocks = _pi_clocks_state_snapshot()

    # One canonical clocks object.  Teensy owns the real clocks; Pi adds only
    # the independently owned GNSS_RAW clock.  No clocks.pi mirror and no
    # duplicated CLOCKS_FRAGMENT evidence branch.
    clocks = copy.deepcopy(teensy_clocks)
    clocks["source_schema"] = teensy_clocks.get("schema")
    clocks["schema"] = "CLOCKS_INSTRUMENT_STATE_V1"
    clocks["gnss_raw"] = copy.deepcopy(pi_clocks.get("gnss_raw") or {})
    clocks["system_time_utc"] = published_at_utc
    clocks["gnss_time_utc"] = _system_gnss_time_utc(system_context)
    clocks["persisted"] = False
    clocks["ephemeral"] = True

    features = _deep_merge_dicts(
        system_context.get("features"),
        clocks_fragment.get("features"),
    )
    gnss_time_utc = _system_gnss_time_utc(system_context)

    state: Dict[str, Any] = {
        "schema": "CLOCKS_V4",
        "source_schema": clocks_fragment.get("schema"),
        "sequence": int(sequence),
        "published_at_utc": published_at_utc,
        # SYSTEM-owned source payloads remain transitive and structurally intact.
        "pi": copy.deepcopy(system_context.get("pi") or {}),
        "network": copy.deepcopy(system_context.get("network") or {}),
        "sensors": copy.deepcopy(system_context.get("sensors") or {}),
        "environment": copy.deepcopy(system_context.get("environment") or {}),
        "gnss": gnss,
        "gnss_monitor": {
            "source": "SYSTEM.REPORT",
            "sampled_at_utc": published_at_utc,
            "source_fresh": bool(gnss),
        },
        "time_sanity": {
            "schema": "CLOCKS_TIME_SANITY_V2",
            "gnss_utc": gnss_time_utc,
            "system_utc": published_at_utc,
            "gnss_source_fresh": bool(gnss),
        },
        "power": copy.deepcopy(system_context.get("power") or {}),
        "battery": copy.deepcopy(system_context.get("battery") or {}),
        "clocks": clocks,
        "features": features,
        "baseline": copy.deepcopy(pi_clocks.get("baseline") or {}),
        "stats_reset": copy.deepcopy(pi_clocks.get("stats_reset") or {}),
        "startup": copy.deepcopy(pi_clocks.get("startup") or {}),
        "complete_for_display": bool(teensy_clocks and pi_clocks.get("gnss_raw")),
    }

    campaign = clocks_fragment.get("campaign")
    if isinstance(campaign, dict):
        # The raw firmware delta is persisted exactly once at top level.
        state["campaign"] = copy.deepcopy(campaign)

    return state


def _clocks_detail_campaign(state: Dict[str, Any]) -> Optional[str]:
    campaign = _state_campaign(state)
    if campaign:
        name = _tempest_campaign_name(campaign)
        return name or None
    return None


def _clocks_detail_viable(state: Dict[str, Any]) -> bool:
    campaign = _state_campaign(state)
    if not campaign:
        return True
    disposition = campaign.get("disposition")
    if not isinstance(disposition, dict):
        return False
    return bool(disposition.get("science_eligible"))


def _persist_clocks_state(state: Dict[str, Any]) -> str:
    encoded = json.dumps(state, separators=(",", ":"), ensure_ascii=False)
    sequence = state.get("sequence")
    pps_count = sequence
    campaign = _clocks_detail_campaign(state)
    viable = _clocks_detail_viable(state)
    with open_db() as conn:
        cur = conn.cursor()
        cur.execute(
            """
            UPDATE campaign_detail
            SET campaign = %s,
                viable = %s,
                payload = %s::jsonb,
                sequence = %s,
                pps_count = %s
            WHERE id = (
                SELECT id
                FROM campaign_detail
                WHERE campaign_type = %s
                  AND sequence IS NOT DISTINCT FROM %s
                  AND pps_count IS NOT DISTINCT FROM %s
                  AND ts >= now() - interval '2 seconds'
                ORDER BY id DESC
                LIMIT 1
            )
            RETURNING id
            """,
            (
                campaign, viable, encoded, sequence, pps_count,
                CAMPAIGN_TYPE_TEMPEST, sequence, pps_count,
            ),
        )
        if cur.fetchone() is not None:
            return "merged"
        cur.execute(
            """
            INSERT INTO campaign_detail
                (campaign_type, campaign, viable, payload, sequence, pps_count)
            VALUES (%s, %s, %s, %s::jsonb, %s, %s)
            """,
            (
                CAMPAIGN_TYPE_TEMPEST, campaign, viable, encoded, sequence, pps_count,
            ),
        )
    return "inserted"


def _cache_clocks_state(state: Dict[str, Any]) -> None:
    global _latest_clocks, _latest_clocks_received_monotonic, _latest_clocks_received_utc
    with _clocks_lock:
        _latest_clocks = copy.deepcopy(state)
        _latest_clocks_received_monotonic = time.monotonic()
        _latest_clocks_received_utc = str(state.get("published_at_utc") or system_time_z())
    _diag["preflight_clocks_updates"] += 1


def _queue_clocks_state(fragment: Dict[str, Any]) -> None:
    global _clocks_state_enqueued, _clocks_state_dropped
    try:
        _clocks_state_queue.put_nowait(copy.deepcopy(fragment))
        _clocks_state_enqueued += 1
    except queue.Full:
        _clocks_state_dropped += 1
        logging.error(
            "💥 [clocks] canonical CLOCKS state queue full; dropping sequence=%s",
            _clocks_fragment_count(fragment),
        )


def _clocks_state_loop() -> None:
    """Build and publish CLOCKS_V4 without storage latency stalling the live feed."""
    global _clocks_state_published, _clocks_state_dropped

    _clocks_state_worker_started.set()
    logging.info("🚀 [clocks] canonical CLOCKS_V4 state worker started")
    while True:
        clocks_fragment = _clocks_state_queue.get()
        failure_logged = False
        while True:
            try:
                system_context = _fetch_system_report()
                break
            except Exception:
                if not failure_logged:
                    logging.exception(
                        "⚠️ [clocks] SYSTEM.REPORT unavailable for CLOCKS sequence=%s; retrying",
                        _clocks_fragment_count(clocks_fragment),
                    )
                    failure_logged = True
                time.sleep(CLOCKS_STATE_RETRY_S)

        sequence = _clocks_fragment_count(clocks_fragment)
        try:
            sequence, _ = _validate_clocks_fragment_v4(clocks_fragment)
            _advance_gnss_raw_instrument(sequence, system_context)
            state = _build_canonical_clocks_state(clocks_fragment, system_context)
        except Exception:
            _clocks_state_dropped += 1
            logging.exception(
                "💥 [clocks] rejecting malformed/incoherent CLOCKS_FRAGMENT_V4 "
                "sequence=%s without terminating the state worker",
                sequence,
            )
            continue

        _cache_clocks_state(state)
        publish(CLOCKS_TOPIC, state)
        _clocks_state_published += 1

        if not _clocks_persistence_enabled.is_set():
            continue

        _clocks_persist_queue.put({
            "state": state,
            "system_context": system_context,
            "clocks_fragment": clocks_fragment,
        })


def _clocks_persistence_loop() -> None:
    """Persist CLOCKS in order, then release embedded TEMPEST candidates."""
    global _clocks_state_persisted, _clocks_state_inserted, _clocks_state_merged
    global _last_tempest_candidate_identity, _last_tempest_candidate_monotonic

    logging.info("🚀 [clocks] CLOCKS persistence worker started")
    while True:
        item = _clocks_persist_queue.get()
        state = copy.deepcopy(item["state"])
        system_context = item["system_context"]
        clocks_fragment = item["clocks_fragment"]
        state_clocks = state.get("clocks")
        if isinstance(state_clocks, dict):
            state_clocks["persisted"] = True
            state_clocks["ephemeral"] = False

        failure_logged = False
        while True:
            try:
                with _clocks_persistence_lock:
                    disposition = _persist_clocks_state(state)
                _clocks_state_persisted += 1
                if disposition == "merged":
                    _clocks_state_merged += 1
                else:
                    _clocks_state_inserted += 1
                break
            except Exception:
                if not failure_logged:
                    logging.exception(
                        "⚠️ [clocks] campaign_detail persistence failed for CLOCKS sequence=%s; retrying",
                        state.get("sequence"),
                    )
                    failure_logged = True
                time.sleep(CLOCKS_STATE_RETRY_S)

        candidate = clocks_fragment.get("campaign")
        if not isinstance(candidate, dict):
            continue
        try:
            candidate_count = _tempest_public_count(candidate)
            candidate_name = _tempest_campaign_name(candidate)
        except ValueError:
            logging.exception("💥 [clocks] malformed V4 campaign enrichment after CLOCKS persistence")
            continue
        identity = (candidate_name, int(candidate_count))
        now = time.monotonic()
        if _last_tempest_candidate_identity == identity:
            _diag["pps_count_repeat"] += 1
            continue
        _last_tempest_candidate_identity = identity
        _last_tempest_candidate_monotonic = now

        _enqueue_timebase_piece(
            TIMEBASE_FRAGMENT_TOPIC,
            candidate,
            state_sequence=int(state.get("sequence") or 0),
            system_context=system_context,
            clocks_fragment=clocks_fragment,
        )
        _diag["timebase_candidates_received"] += 1
        _diag["timebase_candidates_queued"] += 1
        _diag["fragments_received"] += 1
        _diag["fragments_queued"] += 1




# ---------------------------------------------------------------------
# Fragment handler (PUBSUB — fast path, NEVER blocks)
# ---------------------------------------------------------------------


def on_clocks_fragment(payload: Payload) -> None:
    """Queue every exact CLOCKS_FRAGMENT_V4 for canonical CLOCKS construction."""
    if not isinstance(payload, dict):
        _diag["clocks_fragments_malformed"] = _diag.get("clocks_fragments_malformed", 0) + 1
        return

    fragment = dict(payload)
    if fragment.get("schema") != "CLOCKS_FRAGMENT_V4":
        _diag["clocks_fragments_malformed"] = _diag.get("clocks_fragments_malformed", 0) + 1
        logging.error(
            "💥 [clocks] unsupported CLOCKS_FRAGMENT schema=%r",
            fragment.get("schema"),
        )
        return

    campaign = fragment.get("campaign")
    if isinstance(campaign, dict):
        _diag["clocks_fragments_with_campaign"] = (
            _diag.get("clocks_fragments_with_campaign", 0) + 1
        )
        try:
            public_count = _tempest_public_count(campaign)
        except ValueError:
            public_count = None
        _note_timebase_activity(TIMEBASE_FRAGMENT_TOPIC, public_count)
    else:
        _diag["clocks_fragments_observation_only"] = (
            _diag.get("clocks_fragments_observation_only", 0) + 1
        )

    _queue_clocks_state(fragment)



# ---------------------------------------------------------------------
# Processor thread (slow path — all the heavy lifting)
# ---------------------------------------------------------------------


def _process_loop() -> None:
    """Adjudicate persisted V4 TEMPEST deltas without re-authoring Teensy science."""
    global _campaign_active, _accepted_pps_vclock_count, _gnss_raw_ns, _gnss_raw_n, _gnss_raw_valid
    global _post_recovery_science_confirmation_pending
    global _post_recovery_science_confirmation_campaign
    global _post_recovery_first_public_pps_vclock_count

    logging.info("🚀 [clocks] TEMPEST V4 processor thread started")

    while True:
        try:
            piece = _fragment_queue.get(timeout=0.25)
        except queue.Empty:
            continue

        _diag["timebase_pieces_processed"] += 1
        _diag["queue_depth_current"] = _fragment_queue.qsize()

        topic = str(piece.get("topic") or "")
        raw_campaign = piece.get("payload")
        state_sequence = _as_int(piece.get("state_sequence"))
        system_context = piece.get("system_context")
        clocks_fragment = piece.get("clocks_fragment")
        system_context = system_context if isinstance(system_context, dict) else {}
        clocks_fragment = clocks_fragment if isinstance(clocks_fragment, dict) else {}

        if not isinstance(raw_campaign, dict) or state_sequence is None or state_sequence <= 0:
            logging.error("💥 [clocks] processor received malformed V4 campaign piece: %s", piece)
            continue

        try:
            frag = _tempest_candidate_view(raw_campaign)
            public_count = _tempest_public_count(raw_campaign)
            firmware_campaign = _tempest_campaign_name(raw_campaign)
        except ValueError as exc:
            _diag["fragments_missing_teensy_pps_count"] += 1
            _diag["fragments_missing_teensy_pps_vclock_count"] += 1
            logging.error("💥 [clocks] malformed TEMPEST_FRAGMENT_V1: %s", exc)
            continue

        raw_record = copy.deepcopy(raw_campaign)
        _diag["timebase_candidates_processed"] += 1
        _diag["timebase_rows_completed"] += 1
        _diag["timebase_pairs_completed"] += 1

        row = _get_active_campaign()
        campaign = row["campaign"] if row is not None else firmware_campaign
        campaign_payload = row["payload"] if row is not None else {}

        # Final court works on a transient normalized view.  Durable state keeps
        # raw TEMPEST_FRAGMENT_V1 unchanged.
        court_timebase = {
            "schema": "TIMEBASE_COURT_V4",
            "campaign": campaign,
            "sequence": int(state_sequence),
            "public_count": int(public_count),
            "fragment": frag,
        }

        firmware_exclusion = _firmware_science_exclusion(frag)
        if firmware_exclusion:
            _diag["firmware_science_exclude_received"] += 1
            _diag["last_firmware_science_exclude"] = {
                "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "campaign": campaign,
                "public_count": int(public_count),
                **firmware_exclusion,
            }
            raw_cycles = _path_get(clocks_fragment, "clocks.raw_cycles")
            raw_cycles = raw_cycles if isinstance(raw_cycles, dict) else {}

            def _cycle_evidence(lane: str) -> str:
                sample = raw_cycles.get(lane)
                if not isinstance(sample, dict):
                    return f"{lane}=missing"
                return (
                    f"{lane}[obs={sample.get('observed_cycles')} "
                    f"prev={sample.get('previous_observed_cycles')} "
                    f"res={sample.get('residual_cycles')}]"
                )

            logging.warning(
                "🧪 [clocks] firmware science exclusion retained: "
                "campaign=%s public_count=%d reason=%s raw_cycles=%s",
                campaign,
                int(public_count),
                firmware_exclusion.get("reason"),
                " ".join(
                    _cycle_evidence(lane)
                    for lane in ("pps", "vclock", "ocxo1", "ocxo2")
                ),
            )

        _diag["timebase_final_court_checks"] += 1
        court_ok, court_verdict = _timebase_final_court_evaluate(court_timebase)
        if not court_ok:
            _timebase_final_court_block(
                court_verdict,
                raw_record=raw_record,
                assembled_timebase=court_timebase,
            )
            continue

        _diag["timebase_final_court_passed"] += 1
        _diag["last_timebase_final_court"] = court_verdict
        if court_verdict.get("science_excluded"):
            _diag["timebase_final_court_science_excluded"] += 1
            _diag["timebase_final_court_science_exclusion_violations"] += int(
                court_verdict.get("science_violation_count") or 0
            )

        sync_matched = _signal_sync_candidate_if_needed(frag, public_count)
        if sync_matched and not _campaign_active:
            if not _sync_resume_event.wait(timeout=SYNC_RECOVER_TIMEOUT_S):
                logging.error(
                    "💥 [recovery] timed out reopening processor for first accepted candidate public_count=%d",
                    int(public_count),
                )
                continue

        if not _campaign_active:
            _diag["fragments_ignored_no_campaign"] += 1
            _diag["timebase_candidates_ignored_no_campaign"] += 1
            continue

        row = _get_active_campaign()
        if row is None:
            _diag["hard_fault_no_active_campaign"] += 1
            try:
                _hard_fault(
                    "no_active_campaign",
                    {
                        "sequence": int(state_sequence),
                        "public_count": int(public_count),
                    },
                )
            except RuntimeError:
                continue

        campaign = row["campaign"]
        campaign_payload = row["payload"]
        if firmware_campaign and firmware_campaign != campaign:
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

        science_eligible = bool(court_verdict.get("science_eligible"))
        control_eligible = bool(court_verdict.get("control_eligible"))
        science_excluded = bool(court_verdict.get("science_excluded"))

        _diag["fragments_processed"] += 1
        _note_pps_vclock_count(public_count)

        # Internal historical variable name: this is campaign public_count.
        _accepted_pps_vclock_count = int(public_count)
        _diag["accepted_pps_count"] = _accepted_pps_vclock_count
        _diag["accepted_pps_vclock_count"] = _accepted_pps_vclock_count

        _mark_start_first_fragment_if_needed(
            campaign=campaign,
            pps_vclock_count=int(public_count),
        )

        gnss_ns = _fragment_ns(frag, "gnss_ns", default=0)
        if gnss_ns > 0:
            _gnss_canary_update(gnss_ns)

        env_snapshot = _environment_from_system_report(system_context)
        gnss_info = _system_gnss_info(system_context)
        gnss_raw_drift_ppb = _gnss_raw_drift_from_info(gnss_info)

        if science_eligible:
            _gnss_raw_ns += NS_PER_SECOND + (
                gnss_raw_drift_ppb if gnss_raw_drift_ppb is not None else 0.0
            )
            _gnss_raw_n += 1
            _gnss_raw_valid = True

        gnss_raw_ns_int = int(round(_gnss_raw_ns))
        gnss_raw_ref_ns = _gnss_raw_n * NS_PER_SECOND
        gnss_raw_welford = _gnss_raw_welford_snapshot()
        with _gnss_raw_stats_lock:
            gnss_raw_instrument_ns = int(round(_gnss_raw_instrument_ns))
            gnss_raw_instrument_n = int(_gnss_raw_instrument_n)
            gnss_raw_instrument_valid = bool(_gnss_raw_instrument_valid)

        system_time_utc = datetime.now(timezone.utc)
        system_time_str = system_time_utc.isoformat(timespec="microseconds")

        timebase = {
            "schema": "TIMEBASE_V4",
            "campaign_type": CAMPAIGN_TYPE_TEMPEST,
            "campaign": campaign,
            "sequence": int(state_sequence),
            "public_count": int(public_count),
            "campaign_elapsed": _seconds_to_hms(int(public_count)),
            "science_eligible": science_eligible,
            "control_eligible": control_eligible,
            "persist": True,
            "science_excluded": science_excluded,
            "candidate_use": "AUDIT_ONLY" if science_excluded else "SCIENCE_AND_CONTROL",
            "final_court": court_verdict,
            "location": campaign_payload.get("location"),
            "system_time_utc": system_time_str,
            "gnss_time_utc": (
                gnss_info.get("gnss_time_utc") if isinstance(gnss_info, dict) else None
            ),
            # Exact firmware campaign delta; no duplicated live instrument state.
            "fragment": copy.deepcopy(raw_campaign),
            # Campaign master/read-model and recovery helpers may inspect the
            # canonical live instrument without manufacturing restore_state.
            "clocks": copy.deepcopy(clocks_fragment.get("clocks") or {}),
            "environment": env_snapshot,
            "gnss": gnss_info,
            "extra_clocks": {
                "gnss_raw_ns": gnss_raw_ns_int,
                "gnss_raw_ref_ns": gnss_raw_ref_ns,
                "gnss_raw_instrument_ns": gnss_raw_instrument_ns,
                "gnss_raw_instrument_ref_ns": gnss_raw_instrument_n * NS_PER_SECOND,
                "gnss_raw_instrument_n": gnss_raw_instrument_n,
                "gnss_raw_instrument_valid": gnss_raw_instrument_valid,
                "gnss_raw_drift_ppb": gnss_raw_drift_ppb,
                "gnss_raw_tau": round(_compute_tau(gnss_raw_ns_int, gnss_raw_ref_ns), 12)
                    if _gnss_raw_valid else None,
                "gnss_raw_ppb": round(_compute_ppb(gnss_raw_ns_int, gnss_raw_ref_ns), 3)
                    if _gnss_raw_valid else None,
                "gnss_raw_welford_n": gnss_raw_welford["n"],
                "gnss_raw_welford_mean": gnss_raw_welford["mean"],
                "gnss_raw_welford_m2": gnss_raw_welford["m2"],
                "gnss_raw_welford_stddev": gnss_raw_welford["stddev"],
                "gnss_raw_welford_stderr": round(gnss_raw_welford["stderr"], 3),
                "gnss_raw_welford_min": round(gnss_raw_welford["min"], 3),
                "gnss_raw_welford_max": round(gnss_raw_welford["max"], 3),
            },
        }

        publish("TIMEBASE", timebase)
        _attach_tempest_to_state_detail(timebase)

        if (
            _post_recovery_science_confirmation_pending
            and science_eligible
            and control_eligible
            and not science_excluded
            and (
                _post_recovery_science_confirmation_campaign is None
                or campaign == _post_recovery_science_confirmation_campaign
            )
        ):
            recovery_first_count = _post_recovery_first_public_pps_vclock_count
            _post_recovery_science_confirmation_pending = False
            _post_recovery_science_confirmation_campaign = None
            _post_recovery_first_public_pps_vclock_count = None
            logging.info(
                "🏁 [recovery] campaign '%s' fully recovered — first "
                "science-and-control eligible row accepted at public_count=%d "
                "(timeline resumed at public_count=%s)",
                campaign,
                int(public_count),
                str(recovery_first_count),
            )

# ---------------------------------------------------------------------
# Control-plane: START / STOP / CLEAR / RECOVER
# ---------------------------------------------------------------------


def _reset_trackers() -> int:
    global _last_pps_vclock_count_seen
    global _last_tempest_candidate_identity, _last_tempest_candidate_monotonic
    global _post_recovery_science_confirmation_pending
    global _post_recovery_science_confirmation_campaign
    global _post_recovery_first_public_pps_vclock_count
    global _timebase_last_activity_monotonic
    global _timebase_last_activity_utc
    global _timebase_last_activity_topic
    global _timebase_last_activity_pps_vclock_count

    _last_pps_vclock_count_seen = None
    _last_tempest_candidate_identity = None
    _last_tempest_candidate_monotonic = None
    _post_recovery_science_confirmation_pending = False
    _post_recovery_science_confirmation_campaign = None
    _post_recovery_first_public_pps_vclock_count = None
    _timebase_last_activity_monotonic = None
    _timebase_last_activity_utc = None
    _timebase_last_activity_topic = None
    _timebase_last_activity_pps_vclock_count = None
    _diag["last_timebase_activity"] = {}
    _gnss_canary_reset()
    _gnss_raw_clock_reset()
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


# START is asynchronous.  These statuses mean that firmware accepted the
# lifecycle request; they do not claim that the first public CLOCKS_FRAGMENT campaign delta has
# already been emitted.  Keep the legacy SmartZero/Flash Cut vocabulary for
# older firmware while accepting the current always-on recording-boundary
# contract returned by CLOCKS.START.
_TEENSY_START_ACCEPTED_STATUSES = {
    "start_requested",
    "start_requested_dac_fault_servos_off",
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


_TEENSY_MONITOR_RESTORE_ACCEPTED_STATUSES = {
    "monitor_restore_requested",
}


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
    _abort_teensy_ppb_restore_best_effort()
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
    accepts the very first CLOCKS_FRAGMENT campaign delta. There is no Pi-side warmup
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

    retired_control_args = (
        "set_dac1", "set_dac2", "calibrate_ocxo",
        "SET_DAC1", "SET_DAC2", "OCXO1_DAC", "OCXO2_DAC",
        "calibrate_oxco", "calibrate_oxo", "calibrate",
        "calibrate_ocxo_mode",
    )
    supplied_control_args = [
        key for key in retired_control_args if key in start_args
    ]
    if supplied_control_args:
        return {
            "success": False,
            "message": (
                "START no longer accepts DAC or servo-calibration parameters: "
                + ", ".join(supplied_control_args)
                + ". Use the dedicated CLOCKS control commands before or after "
                  "campaign START."
            ),
        }

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
            """
            SELECT id, active
            FROM campaign_master
            WHERE campaign_type = %s AND campaign = %s
            """,
            (CAMPAIGN_TYPE_TEMPEST, campaign),
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

    if flash_cut:
        prev_location = prev_payload.get("location")
        if location is None and prev_location:
            location = prev_location

    try:
        _wait_for_timebase_routes(context="flash_cut" if flash_cut else "start")
    except Exception as e:
        return {"success": False, "message": str(e)}

    logging.info(
        "%s [start] campaign='%s' mode=%s location=%s",
        "⚡" if flash_cut else "▶️",
        campaign,
        "FLASH_CUT" if flash_cut else "COLD_START",
        location or "none",
    )

    cutover_ts = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    campaign_payload = {
        "campaign_type": CAMPAIGN_TYPE_TEMPEST,
        "location": location,
        "started_at": cutover_ts,
    }
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
            UPDATE campaign_master
            SET active = false,
                payload = payload || jsonb_build_object('stopped_at', to_jsonb(%s::text))
            WHERE campaign_type = %s
              AND active = true
            """,
            (cutover_ts, CAMPAIGN_TYPE_TEMPEST),
        )
        cur.execute(
            """
            INSERT INTO campaign_master (campaign_type, campaign, active, payload)
            VALUES (%s, %s, true, %s)
            """,
            (CAMPAIGN_TYPE_TEMPEST, campaign, json.dumps(campaign_payload)),
        )

    if not flash_cut:
        _request_teensy_stop_best_effort()

    drained = _reset_trackers()
    if drained:
        logging.info("🧹 [start] drained %d stale CLOCKS_FRAGMENT campaign delta(s) before arm", drained)
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
                    UPDATE campaign_master
                    SET active = false,
                        payload = payload || jsonb_build_object(
                            'start_failed_at', to_jsonb(%s::text),
                            'start_failed_reason', to_jsonb(%s::text),
                            'start_failed_error', to_jsonb(%s::text),
                            'start_failed_teensy_payload', %s::jsonb
                        )
                    WHERE campaign_type = %s
                      AND campaign = %s
                      AND active = true
                    """,
                    (
                        failed_at,
                        str(failure_status),
                        str(failure_payload.get("error") or str(e)),
                        json.dumps(failure_payload),
                        CAMPAIGN_TYPE_TEMPEST,
                        campaign,
                    ),
                )
                if flash_cut and prev_campaign:
                    cur.execute(
                        """
                        UPDATE campaign_master
                        SET active = true,
                            payload = payload - 'stopped_at'
                        WHERE campaign_type = %s
                          AND campaign = %s
                        """,
                        (CAMPAIGN_TYPE_TEMPEST, prev_campaign),
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
        "awaiting first CLOCKS_FRAGMENT campaign delta",
        system_time_z(),
        "FLASH_CUT" if flash_cut else "START",
        campaign,
        teensy_start_status,
    )

    return {
        "success": True,
        "message": "START_REQUESTED",
        "payload": {
            "campaign_type": CAMPAIGN_TYPE_TEMPEST,
            "campaign": campaign,
            "location": location,
            "waiting_for_first_fragment": True,
            "teensy_start_status": teensy_start_status,
            "teensy_start_payload": teensy_start_payload,
            "startup": _start_status_payload(),
            "flash_cut": flash_cut,
            "previous_campaign": prev_campaign,
            "sync_wait_removed": True,
            "teensy_stop_sent": not flash_cut,
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
            UPDATE campaign_master
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
            WHERE campaign_type = %s
              AND active = true
            """,
            (stopped_at, CAMPAIGN_TYPE_TEMPEST),
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


def _gnss_raw_clock_snapshot() -> Dict[str, Any]:
    """Return operator-facing GNSS_RAW state; campaign presentation is derived here."""
    canonical = _gnss_raw_state_snapshot()
    instrument = copy.deepcopy(canonical.get("instrument") or {})
    campaign_ref_ns = int(_gnss_raw_n) * NS_PER_SECOND
    campaign_ns = int(round(_gnss_raw_ns))
    campaign_valid = bool(_campaign_active and _gnss_raw_valid and campaign_ref_ns > 0)
    campaign = {
        "valid": campaign_valid,
        "ns": campaign_ns,
        "ref_ns": campaign_ref_ns,
        "clockface_n": int(_gnss_raw_n),
        "tau": (
            round(_compute_tau(campaign_ns, campaign_ref_ns), 12)
            if campaign_valid else None
        ),
        "ppb": (
            round(_compute_ppb(campaign_ns, campaign_ref_ns), 3)
            if campaign_valid else None
        ),
    }
    presentation = campaign if campaign_valid else instrument
    return {
        **canonical,
        "clockface_campaign_relative": True,
        "campaign": campaign,
        "presentation": {
            "mode": "CAMPAIGN" if campaign_valid else "INSTRUMENT",
            "basis": "CAMPAIGN_RELATIVE" if campaign_valid else "PI_PROCESS_LIFETIME",
            **copy.deepcopy(presentation),
        },
        # Compact legacy report surface remains derived, never persisted as
        # canonical restore authority.
        "valid": campaign_valid,
        "ns": campaign_ns,
        "ref_ns": campaign_ref_ns,
        "clockface_n": int(_gnss_raw_n),
        "tau": campaign.get("tau"),
        "ppb": campaign.get("ppb"),
    }


def _combined_teensy_report(teensy_command: str, *, report_name: str) -> Dict[str, Any]:
    """Return one operator-facing CLOCKS report spanning Teensy and Pi owners."""
    try:
        teensy_response = send_command(
            machine="TEENSY",
            subsystem="CLOCKS",
            command=teensy_command,
        )
    except Exception as exc:
        _diag["report_transitive_failures"] = _diag.get("report_transitive_failures", 0) + 1
        return {
            "success": False,
            "message": f"Teensy CLOCKS.{teensy_command} failed: {exc}",
            "payload": {
                "report": report_name,
                "scope": "SYSTEMWIDE_CLOCKS",
                "teensy_command": teensy_command,
                "error": str(exc),
                "pi_gnss_raw": _gnss_raw_clock_snapshot(),
            },
        }

    teensy_ok = isinstance(teensy_response, dict) and bool(teensy_response.get("success"))
    if not teensy_ok:
        _diag["report_transitive_failures"] = _diag.get("report_transitive_failures", 0) + 1
        return {
            "success": False,
            "message": f"Teensy CLOCKS.{teensy_command} rejected",
            "payload": {
                "report": report_name,
                "scope": "SYSTEMWIDE_CLOCKS",
                "teensy_command": teensy_command,
                "teensy": teensy_response,
                "pi_gnss_raw": _gnss_raw_clock_snapshot(),
            },
        }

    active = _get_active_campaign()
    campaign_payload = active.get("payload", {}) if isinstance(active, dict) else {}
    combined = {
        "schema": "CLOCKS_SYSTEM_REPORT_V1",
        "report": report_name,
        "scope": "SYSTEMWIDE_CLOCKS",
        "generated_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "campaign_active": bool(active),
        "campaign": active.get("campaign") if isinstance(active, dict) else None,
        "campaign_state": (
            "STARTED" if active else "IDLE"
        ),
        "startup": _start_status_payload(),
        "owners": {
            "teensy": ["GNSS", "VCLOCK", "DWT", "OCXO1", "OCXO2", "DAC"],
            "pi": ["GNSS_RAW"],
        },
        "teensy": teensy_response,
        "pi": {
            "gnss_raw": _gnss_raw_clock_snapshot(),
            "stats_reset": {
                "requests": int(_diag.get("stats_reset_requests") or 0),
                "success": int(_diag.get("stats_reset_success") or 0),
                "last": _diag.get("last_stats_reset") or {},
            },
            "campaign_report_present": bool(
                isinstance(campaign_payload, dict) and isinstance(campaign_payload.get("report"), dict)
            ),
        },
    }
    return {"success": True, "message": "OK", "payload": combined}


def cmd_report_clocks(_: Optional[dict]) -> Dict[str, Any]:
    """Return the compact systemwide clock/instrument report."""
    _diag["report_clocks_requests"] = _diag.get("report_clocks_requests", 0) + 1
    response = _combined_teensy_report("REPORT_CLOCKS", report_name="CLOCKS_SYSTEM_INSTRUMENT")
    _diag["last_report_clocks"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "success": bool(response.get("success")),
    }
    return response


def cmd_report_stats(_: Optional[dict]) -> Dict[str, Any]:
    """Return the detailed systemwide statistical report."""
    _diag["report_stats_requests"] = _diag.get("report_stats_requests", 0) + 1
    response = _combined_teensy_report("REPORT_STATS", report_name="CLOCKS_SYSTEM_STATS")
    _diag["last_report_stats"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "success": bool(response.get("success")),
    }
    return response


def cmd_stats_reset(_: Optional[dict]) -> Dict[str, Any]:
    """Reset all CLOCKS statistical populations as one operator command.

    Teensy owns the real-clock Welfords; Pi owns GNSS_RAW.  Require the
    Teensy reset to succeed before resetting Pi state so a transport failure
    cannot silently split the statistical epoch.
    """
    _diag["stats_reset_requests"] = _diag.get("stats_reset_requests", 0) + 1
    startup_busy = _startup_control_gate("STATS_RESET")
    if startup_busy is not None:
        return startup_busy

    requested_at = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    pi_before = _gnss_raw_welford_snapshot()

    try:
        teensy_response = send_command(
            machine="TEENSY",
            subsystem="CLOCKS",
            command="STATS_RESET",
        )
    except Exception as exc:
        _diag["stats_reset_teensy_failures"] = (
            _diag.get("stats_reset_teensy_failures", 0) + 1
        )
        failure = {
            "requested_at_utc": requested_at,
            "success": False,
            "stage": "TEENSY",
            "error": str(exc),
            "pi_gnss_raw_before": pi_before,
            "pi_reset_applied": False,
        }
        _diag["last_stats_reset"] = failure
        logging.exception("❌ [clocks] transitive STATS_RESET failed at Teensy; Pi stats preserved")
        return {
            "success": False,
            "message": f"Teensy CLOCKS.STATS_RESET failed: {exc}",
            "payload": failure,
        }

    teensy_ok = isinstance(teensy_response, dict) and bool(teensy_response.get("success"))
    if not teensy_ok:
        _diag["stats_reset_teensy_failures"] = (
            _diag.get("stats_reset_teensy_failures", 0) + 1
        )
        failure = {
            "requested_at_utc": requested_at,
            "success": False,
            "stage": "TEENSY",
            "teensy_response": teensy_response,
            "pi_gnss_raw_before": pi_before,
            "pi_reset_applied": False,
        }
        _diag["last_stats_reset"] = failure
        logging.error("❌ [clocks] Teensy rejected STATS_RESET; Pi stats preserved: %s", teensy_response)
        return {
            "success": False,
            "message": "Teensy CLOCKS.STATS_RESET rejected",
            "payload": failure,
        }

    pi_previous = _gnss_raw_welford_reset()
    pi_after = _gnss_raw_welford_snapshot()
    completed_at = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    result = {
        "requested_at_utc": requested_at,
        "completed_at_utc": completed_at,
        "success": True,
        "scope": "SYSTEMWIDE_CLOCK_STATISTICS",
        "campaign_unchanged": True,
        "clockfaces_unchanged": True,
        "teensy": teensy_response,
        "pi_gnss_raw_before": pi_previous,
        "pi_gnss_raw_after": pi_after,
    }
    _diag["stats_reset_success"] = _diag.get("stats_reset_success", 0) + 1
    _diag["last_stats_reset"] = result
    logging.warning(
        "📊 [clocks] transitive STATS_RESET complete: Teensy accepted; "
        "GNSS_RAW N %d -> %d; campaign and clockfaces preserved",
        int(pi_previous.get("n") or 0),
        int(pi_after.get("n") or 0),
    )
    return {"success": True, "message": "OK", "payload": result}




def cmd_report(_: Optional[dict]) -> Dict[str, Any]:
    row = _get_active_campaign()
    contract = {
        "mode_free": True,
        "coherent_rows_persist": True,
        "science_exclusions_are_audit_only": True,
        "continuity_fatal_triggers_recovery": True,
    }
    if not row:
        return {
            "success": True,
            "message": "OK",
            "payload": {
                "campaign_type": CAMPAIGN_TYPE_TEMPEST,
                "campaign_state": "IDLE",
                "integrity_contract": contract,
                "startup": _start_status_payload(),
            },
        }

    payload = dict(row["payload"])
    payload["campaign_type"] = row["campaign_type"]
    payload["campaign"] = row["campaign"]
    payload["integrity_contract"] = contract
    payload["startup"] = _start_status_payload()
    return {"success": True, "message": "OK", "payload": payload}


def cmd_clear(_: Optional[dict]) -> dict:
    """Delete all TEMPEST state and campaign history at one persistence boundary."""
    global _campaign_active

    _request_teensy_stop_best_effort()
    _request_teensy_recover_abort_best_effort("pi_clear_cleanup")
    _campaign_active = False
    _clear_start_wait_state()
    _clear_flash_cut_wait_state()
    candidate_drained = _drain_timebase_ingress()
    _reset_trackers()

    persistence_was_enabled = _clocks_persistence_enabled.is_set()
    _clocks_persistence_enabled.clear()
    try:
        with _clocks_persistence_lock:
            state_drained = _drain_clocks_persistence_queue()
            with open_db() as conn:
                cur = conn.cursor()
                cur.execute(
                    "DELETE FROM campaign_detail WHERE campaign_type = %s",
                    (CAMPAIGN_TYPE_TEMPEST,),
                )
                detail_count = cur.rowcount
                cur.execute(
                    "DELETE FROM campaign_master WHERE campaign_type = %s",
                    (CAMPAIGN_TYPE_TEMPEST,),
                )
                master_count = cur.rowcount
    except Exception as e:
        logging.exception("❌ [clocks] CLEAR failed")
        return {"success": False, "message": str(e)}
    finally:
        if persistence_was_enabled:
            _clocks_persistence_enabled.set()

    logging.info(
        "🗑️ [clocks] CLEAR: deleted %d TEMPEST details and %d masters; "
        "drained candidates=%d pending_states=%d",
        detail_count, master_count, candidate_drained, state_drained,
    )
    return {
        "success": True,
        "message": "OK",
        "payload": {
            "campaign_type": CAMPAIGN_TYPE_TEMPEST,
            "campaign_details_deleted": detail_count,
            "campaign_master_deleted": master_count,
            "candidate_ingress_drained": candidate_drained,
            "pending_state_rows_drained": state_drained,
        },
    }




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
            SELECT id, campaign_type, campaign, active, payload
            FROM campaign_master
            WHERE campaign_type = %s
              AND campaign = %s
            ORDER BY ts DESC
            LIMIT 1
            """,
            (CAMPAIGN_TYPE_TEMPEST, campaign_name),
        )
        row = cur.fetchone()

    if row is None:
        return {"success": False, "message": f"No campaign named '{campaign_name}'"}

    payload = row["payload"]
    if isinstance(payload, str):
        payload = json.loads(payload)

    if row["active"]:
        return {"success": False, "message": f"Campaign '{campaign_name}' is already active"}

    # Verify it has unified state rows with TEMPEST decoration.
    tb_count = _count_tempest_state_details(campaign_name)

    if tb_count == 0:
        return {
            "success": False,
            "message": f"Campaign '{campaign_name}' has no TEMPEST campaign details — use START instead",
        }

    # Re-activate
    resumed_at = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    with open_db() as conn:
        cur = conn.cursor()
        cur.execute(
            """
            UPDATE campaign_master
            SET active = false
            WHERE campaign_type = %s
              AND active = true
              AND campaign != %s
            """,
            (CAMPAIGN_TYPE_TEMPEST, campaign_name),
        )
        cur.execute(
            """
            UPDATE campaign_master
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
            WHERE campaign_type = %s
              AND campaign = %s
            """,
            (resumed_at, CAMPAIGN_TYPE_TEMPEST, campaign_name),
        )
        if cur.rowcount == 0:
            return {"success": False, "message": f"Failed to re-activate campaign '{campaign_name}'"}

    logging.info(
        "▶️ [clocks] RESUME: campaign '%s' re-activated (%d TEMPEST campaign details) — starting recovery...",
        campaign_name, tb_count,
    )

    try:
        _restore_active_campaign_state()
    except Exception as e:
        logging.exception("💥 [clocks] RESUME recovery failed for '%s'", campaign_name)
        _campaign_active = False
        try:
            with open_db() as conn:
                cur = conn.cursor()
                cur.execute(
                    """
                    UPDATE campaign_master
                    SET active = false,
                        payload = payload || jsonb_build_object(
                            'stopped_at', to_jsonb(%s::text),
                            'resume_failed', to_jsonb(%s::text)
                        )
                    WHERE campaign_type = %s
                      AND campaign = %s
                      AND active = true
                    """,
                    (
                        datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                        str(e),
                        CAMPAIGN_TYPE_TEMPEST,
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
            "campaign_type": CAMPAIGN_TYPE_TEMPEST,
            "campaign": campaign_name,
            "campaign_detail_rows": tb_count,
            "resumed_at": resumed_at,
        },
    }


# ---------------------------------------------------------------------
# Recovery — v4 Nanosecond Architecture
# ---------------------------------------------------------------------




def _restore_active_campaign_state() -> Dict[str, Any]:
    """
    RECOVER — v7 exact-first-row architecture.
    """
    global _campaign_active, _accepted_pps_vclock_count
    global _last_pps_vclock_count_seen
    global _post_recovery_science_confirmation_pending
    global _post_recovery_science_confirmation_campaign
    global _post_recovery_first_public_pps_vclock_count

    # Immediately deactivate so the processor thread ignores all
    # fragments during recovery. Any earlier one-shot completion notice belongs
    # to the superseded recovery generation and must not survive this entry.
    _post_recovery_science_confirmation_pending = False
    _post_recovery_science_confirmation_campaign = None
    _post_recovery_first_public_pps_vclock_count = None
    _campaign_active = False
    _accepted_pps_vclock_count = None
    _clear_start_wait_state()

    _diag["recovery_checks"] += 1

    row = _get_active_campaign()
    if row is None:
        _diag["recovery_no_active_campaign"] += 1
        logging.info("🔍 [recovery] no active campaign — nothing to restore")
        return {"restored": False, "reason": "no_active_campaign"}

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
    # Step 1: Establish whether unified TEMPEST state exists
    # ------------------------------------------------------------------
    if _count_tempest_state_details(campaign_name) == 0:
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
            return {
                "restored": True,
                "mode": "flash_cut_zero_row_reattach",
                "campaign": campaign_name,
            }

        logging.info(
            "ℹ️ [recovery] campaign '%s' has no TEMPEST campaign details — "
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
            logging.info("🧹 [recovery/cold] drained %d stale CLOCKS_FRAGMENT campaign delta(s)", drained)
        _clear_sync_wait()

        # Campaign recovery is recording-only. The live DAC, servo, and dither
        # state is generalized CLOCKS state and is not re-authored here.
        teensy_args: Dict[str, Any] = {"campaign": campaign_name}

        _accepted_pps_vclock_count = None
        _diag["accepted_pps_count"] = None
        _diag["accepted_pps_vclock_count"] = None

        _mark_start_waiting(campaign_name)

        # Cold recovery has no prior TEMPEST detail to recover from.  Treat it as
        # an async START of the existing active campaign, matching cmd_start().
        # The normal processor thread will accept the first
        # CLOCKS_FRAGMENT campaign delta whenever it arrives.
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
            "awaiting first CLOCKS_FRAGMENT campaign delta",
            campaign_name, teensy_start_status,
        )

        _diag["last_recovery"] = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "campaign": campaign_name,
            "mode": "cold_restart_async",
            "sync_wait_removed": True,
            "waiting_for_first_fragment": True,
        }
        return {"restored": True, "mode": "cold_restart_async", "campaign": campaign_name}

    try:
        last_tb, recovery_snapshot, skipped_unrecoverable_rows = _load_last_recoverable_tempest_detail(campaign_name)
    except LookupError:
        # Preserve the existing cold-restart path above for genuinely zero-row campaigns.
        raise RuntimeError(f"recovery failed: campaign '{campaign_name}' has no TEMPEST campaign details")

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
    canonical_recovery_clocks = recovery_snapshot.get("canonical_clocks")
    if not isinstance(canonical_recovery_clocks, dict):
        canonical_recovery_clocks = {}

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
        "📐 [recovery] LAST TIMEBASE:\r\n"
        "    pps_vclock_count = %d\r\n"
        "    gnss_ns    = %d\r\n"
        "    dwt_cycles = %d\r\n"
        "    dwt_ns_arg = %d\r\n"
        "    ocxo1_ns   = %d\r\n"
        "    ocxo2_ns   = %d\r\n"
        "    gn_raw_ns  = %d\r\n"
        "    gn_raw_ref = %d\r\n"
        "    gn_raw_mean_ppb = %.6f\r\n"
        "    gnss_time  = %s",
        last_pps_vclock_count, last_gnss_ns, last_dwt_cycles, last_dwt_ns,
        last_ocxo1_ns, last_ocxo2_ns, last_gnss_raw_ns, last_gnss_raw_ref_ns,
        last_gnss_raw_welford_mean, last_gnss_time_str,
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

    # Warm recovery deliberately bypasses the full START CLOCKS profile:
    # SmartZero/Alpha-epoch/OCXO-origin leaves may be exactly what RECOVER must
    # reconstruct after a Teensy reboot.  Requiring them here would create a
    # circular wait.
    #
    # Keep all concrete recovery safeguards below:
    #   * GNSS mode has already been reconciled above.
    #   * CLOCKS_FRAGMENT PUBSUB routing is verified immediately below.
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
    # Quiesce Pi ingress without issuing an operator STOP
    # ------------------------------------------------------------------
    #
    # CLOCKS.RECOVER is the firmware lifecycle boundary for both cases:
    #   * LIVE_REATTACH preserves an installed Teensy service epoch.
    #   * COLD_BOOTSTRAP creates a fresh SmartZero-backed local epoch after a
    #     flash/reboot, then maps it onto the durable campaign coordinates.
    #
    # Sending STOP here would destroy useful live-service custody and is also
    # unnecessary after a reboot. Preserve the campaign transaction, quiesce
    # only Pi ingress, and let firmware select the lawful recovery mode.
    logging.info(
        "📡 [recovery] @%s preserving durable campaign identity; "
        "quiescing Pi CLOCKS_FRAGMENT campaign ingress before direct RECOVER "
        "(firmware selects live reattach or cold bootstrap)...",
        system_time_z(),
    )

    _drained = _drain_timebase_ingress()
    if _drained > 0:
        logging.info("🧹 [recovery] drained %d stale CLOCKS_FRAGMENT campaign delta(s)", _drained)

    now_frac = time.time() % 1.0
    sleep_to_boundary = (1.0 - now_frac) + 0.050
    logging.info("⏳ [recovery] snapping to second boundary (sleeping %.3fs)", sleep_to_boundary)
    time.sleep(sleep_to_boundary)

    # ------------------------------------------------------------------
    # Step 3: Compute elapsed GNSS seconds
    # ------------------------------------------------------------------
    current_gnss_time_str = _wait_for_gnss_time()
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
        "📐 [recovery] GNSS ELAPSED:\r\n"
        "    last_gnss_time    = %s\r\n"
        "    current_gnss_time = %s\r\n"
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

    recovery_source_db_id = _as_int(last_tb.get("_db_detail_id"))
    _diag["last_recovery"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "campaign": campaign_name,
        "recovery_source_db_id": recovery_source_db_id,
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
        "teensy_stop_sent": False,
        "teensy_recover_abort_sent": False,
        "interrupt_service_preserved": None,
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

    epoch_ready_before_recover = _probe_teensy_recovery_epoch_ready()
    _diag["last_recovery"]["teensy_epoch_ready_before_recover"] = bool(
        epoch_ready_before_recover
    )
    cold_ppb_stage: Optional[Dict[str, Any]] = None
    if epoch_ready_before_recover:
        _diag["last_recovery"]["canonical_restore_present"] = False
        _diag["last_recovery"]["restore_schema_version"] = None
        logging.info(
            "📊 [recovery] live Alpha epoch confirmed; preserving Better Buckets "
            "and all global instrument statistics during LIVE_REATTACH"
        )
    else:
        cold_ppb_replay = _reconstruct_better_buckets_from_db(last_tb)
        cold_ppb_stage = _stage_teensy_better_buckets(cold_ppb_replay)
        teensy_recover_args.update(
            _canonical_restore_args(canonical_recovery_clocks, include_control=False)
        )
        _diag["last_recovery"]["canonical_restore_present"] = True
        _diag["last_recovery"]["restore_schema_version"] = 3
        _diag["last_recovery"]["better_buckets_restore"] = cold_ppb_stage
        logging.info(
            "📊 [recovery] cold Alpha epoch detected; staged Better Buckets "
            "rows=%d second=%d minute=%d before structured RECOVER",
            int(cold_ppb_replay.get("rows_scanned") or 0),
            len(cold_ppb_replay.get("second_history") or []),
            len(cold_ppb_replay.get("minute_history") or []),
        )

    try:
        teensy_recover_resp = _request_teensy_recover(
            int(recover_base_pps_vclock_count),
            teensy_recover_args,
        )
    except Exception:
        if cold_ppb_stage is not None:
            _abort_teensy_ppb_restore_best_effort()
        raise
    teensy_recover_payload = (
        teensy_recover_resp.get("payload", {})
        if isinstance(teensy_recover_resp, dict)
        else {}
    )

    recover_mode = str(
        teensy_recover_payload.get("recover_mode") or "LIVE_REATTACH"
    ).strip().upper()
    cold_bootstrap = recover_mode == "COLD_BOOTSTRAP"
    first_row_timeout_s = (
        RECOVERY_COLD_BOOTSTRAP_FIRST_ROW_TIMEOUT_S
        if cold_bootstrap
        else RECOVERY_FIRST_ROW_TIMEOUT_S
    )

    _diag["last_recovery"].update({
        "recover_mode": recover_mode,
        "cold_bootstrap": bool(cold_bootstrap),
        "interrupt_service_preserved": not cold_bootstrap,
        "recover_cold_bootstrap_active": bool(
            teensy_recover_payload.get("recover_cold_bootstrap_active")
        ),
        "recover_cold_bootstrap_epoch_ready": bool(
            teensy_recover_payload.get("recover_cold_bootstrap_epoch_ready")
        ),
        "first_row_timeout_s": float(first_row_timeout_s),
    })

    if cold_bootstrap:
        logging.info(
            "🧭 [recovery] Teensy selected COLD_BOOTSTRAP: preserving campaign "
            "base=%d while startup SmartZero installs a fresh local epoch",
            recover_base_pps_vclock_count,
        )
    else:
        logging.info(
            "🧭 [recovery] Teensy selected LIVE_REATTACH for campaign base=%d",
            recover_base_pps_vclock_count,
        )

    recovery_monitor = {
        "campaign": campaign_name,
        "recover_base_pps_vclock_count": int(recover_base_pps_vclock_count),
        "expected_first_public_pps_vclock_count": int(expected_first_public_pps_vclock_count),
        "recovery_generation": _as_int(teensy_recover_payload.get("recovery_generation")),
        "recover_status": teensy_recover_payload.get("status"),
        "recover_mode": recover_mode,
        "cold_bootstrap": bool(cold_bootstrap),
        "recover_cold_bootstrap_active": bool(
            teensy_recover_payload.get("recover_cold_bootstrap_active")
        ),
        "recover_cold_bootstrap_epoch_ready": bool(
            teensy_recover_payload.get("recover_cold_bootstrap_epoch_ready")
        ),
        "first_row_timeout_s": float(first_row_timeout_s),
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
                "recover_mode": recover_mode,
                "cold_bootstrap": bool(cold_bootstrap),
                "first_row_timeout_s": float(first_row_timeout_s),
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
                timeout_s=min(float(remaining), float(first_row_timeout_s)),
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
                "recover_mode": recover_mode,
                "cold_bootstrap": bool(cold_bootstrap),
                "first_row_timeout_s": float(first_row_timeout_s),
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

        teensy_pps_vclock_count = _as_int(frag.get("public_count"))
        if teensy_pps_vclock_count is None or teensy_pps_vclock_count <= 0:
            raise RecoveryRetryableFailure(
                "recovery_row_missing_public_count",
                {"fragment": frag},
            )
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

    cold_bootstrap_supersede: Optional[Dict[str, Any]] = None
    if cold_bootstrap:
        if recovery_source_db_id is None or recovery_source_db_id <= 0:
            raise RecoveryRetryableFailure(
                "cold_bootstrap_missing_recovery_source_db_id",
                {"campaign": campaign_name, "last_timebase": last_tb},
            )
        cold_bootstrap_supersede = _supersede_cold_bootstrap_rows(
            base_detail_id=int(recovery_source_db_id),
            campaign_name=campaign_name,
            first_public_count=int(teensy_pps_vclock_count),
        )
        _diag["last_recovery"]["cold_bootstrap_supersede"] = (
            cold_bootstrap_supersede
        )
        logging.info(
            "🧹 [recovery] cold-bootstrap evidence boundary: kept source id=%d, "
            "recovered id=%d, marked %d intervening row(s) non-viable",
            int(cold_bootstrap_supersede["base_detail_id"]),
            int(cold_bootstrap_supersede["recovered_detail_id"]),
            int(cold_bootstrap_supersede["rows_marked_nonviable"]),
        )

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

    fully_clean_at_admission = bool(recovery_admission_verdict.get("fully_clean"))
    _post_recovery_science_confirmation_pending = not fully_clean_at_admission
    _post_recovery_science_confirmation_campaign = (
        campaign_name if _post_recovery_science_confirmation_pending else None
    )
    _post_recovery_first_public_pps_vclock_count = (
        int(teensy_pps_vclock_count)
        if _post_recovery_science_confirmation_pending
        else None
    )

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
        "✅ [recovery] campaign '%s' timeline recovered — canonical TIMEBASE resumes with "
        "first public count=%d science_clean=%s",
        campaign_name,
        teensy_pps_vclock_count,
        bool(recovery_admission_verdict.get("fully_clean")),
    )
    return {
        "restored": True,
        "mode": "warm_recover",
        "campaign": campaign_name,
        "first_public_pps_vclock_count": int(teensy_pps_vclock_count),
        "science_clean": bool(recovery_admission_verdict.get("fully_clean")),
    }



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
                """
                SELECT id, campaign, payload
                FROM campaign_master
                WHERE campaign_type = %s AND id = %s
                """,
                (CAMPAIGN_TYPE_TEMPEST, baseline_id),
            )
        else:
            cur.execute(
                """
                SELECT id, campaign, payload
                FROM campaign_master
                WHERE campaign_type = %s AND campaign = %s
                ORDER BY ts DESC
                LIMIT 1
                """,
                (CAMPAIGN_TYPE_TEMPEST, campaign_name),
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

    # DAC values remain baseline science/metrics only. They do not become a
    # second persistence authority for the live instrument control plane.

    baseline_blob: Dict[str, Any] = {
        "baseline_id": baseline_id,
        "baseline_ppb": baseline_ppb,
        "baseline_tau": baseline_tau,
        "baseline_dac": baseline_dac,
        "baseline_dac_mean": baseline_dac_mean,
        "baseline_dac_stats": baseline_dac_stats,
        "baseline_campaign_type": CAMPAIGN_TYPE_TEMPEST,
        "baseline_campaign": row["campaign"],
        "baseline_pps_vclock_n": baseline_pps_vclock_n,
        "baseline_pps_n": baseline_pps_vclock_n,  # legacy alias
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

    _clocks_baseline_set_cache(baseline_blob)

    logging.info(
        "✅ [clocks] baseline set: id=%d campaign='%s' ppb=%s dac_mean=%s",
        baseline_id, row["campaign"], baseline_ppb, baseline_dac_mean,
    )
    return {"success": True, "message": "OK", "payload": baseline_blob}

# ---------------------------------------------------------------------
# Feature-status preflight gate
# ---------------------------------------------------------------------




def _clocks_features() -> Dict[str, Any]:
    """Return fresh CLOCKS.features or raise while the heartbeat is unavailable."""
    with _clocks_lock:
        monitor = copy.deepcopy(_latest_clocks)
        received_monotonic = _latest_clocks_received_monotonic
        received_utc = _latest_clocks_received_utc

    if received_monotonic is None or not monitor:
        raise RuntimeError("CLOCKS heartbeat not yet received")

    age_s = max(0.0, time.monotonic() - received_monotonic)
    if age_s > CLOCKS_PREFLIGHT_MAX_AGE_S:
        _diag["preflight_clocks_stale"] += 1
        _diag["last_preflight_clocks"] = {
            "status": "STALE",
            "received_at_utc": received_utc,
            "age_s": round(age_s, 3),
            "max_age_s": float(CLOCKS_PREFLIGHT_MAX_AGE_S),
        }
        raise RuntimeError(
            f"CLOCKS heartbeat stale ({age_s:.1f}s > {CLOCKS_PREFLIGHT_MAX_AGE_S:.1f}s)"
        )

    features = monitor.get("features")
    if not isinstance(features, dict) or not features:
        _diag["preflight_clocks_missing_features"] += 1
        raise RuntimeError("CLOCKS heartbeat has no feature tree")

    _diag["last_preflight_clocks"] = {
        "status": "NOMINAL",
        "received_at_utc": received_utc,
        "age_s": round(age_s, 3),
        "sequence": monitor.get("sequence"),
    }
    return features


def _feature_gate_reason(blocker: Dict[str, Any]) -> str:
    name = str(blocker.get("name") or "?")
    status = str(blocker.get("status") or "HOLD")
    detail = str(blocker.get("detail") or "").strip()
    if detail:
        return f"{name} is {status} ({detail})"
    return f"{name} is {status}"


def _check_feature_preflight(context: str) -> tuple[bool, list[str]]:
    """Check the single standardized CLOCKS campaign preflight profile.

    Every required leaf is evaluated from one fresh CLOCKS.features snapshot.
    There is no second firmware-policy cache and no mirror-bypass path.
    """
    _diag["preflight_feature_checks"] += 1

    try:
        features = _clocks_features()
    except Exception as e:
        _diag["preflight_feature_unavailable"] += 1
        _diag["last_preflight_feature_gate"] = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "context": context,
            "profile": FEATURE_PREFLIGHT_PROFILE,
            "status": "UNAVAILABLE",
            "error": str(e),
        }
        return False, [f"{FEATURE_PREFLIGHT_PROFILE}: CLOCKS feature tree unavailable ({e})"]

    raw_blockers = blocking_features(features, FEATURE_PREFLIGHT_REQUIRED)
    compact_raw_blockers = [
        {
            "name": str(b.get("name") or "?"),
            "status": str(b.get("status") or "HOLD"),
            "detail": str(b.get("detail") or ""),
        }
        for b in raw_blockers
    ]

    compact_blockers = compact_raw_blockers

    if compact_blockers:
        _diag["preflight_feature_blocked"] += 1
        _diag["last_preflight_feature_gate"] = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "context": context,
            "profile": FEATURE_PREFLIGHT_PROFILE,
            "status": "BLOCKED",
            "required_count": len(FEATURE_PREFLIGHT_REQUIRED),
            "blockers": compact_blockers,
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
        "blockers": [],
    }
    return True, []


def _preflight_wait_items(reasons: list[str]) -> list[str]:
    """Return a compact, stable list of pending CLOCKS prerequisites."""
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

    for reason in reasons:
        if reason.startswith(f"{FEATURE_PREFLIGHT_PROFILE}:"):
            if not blockers:
                items.append(reason)
            continue
        reason_feature = reason.split(":", 1)[0]
        if reason_feature not in blocker_names:
            items.append(reason)

    # Preserve order while removing duplicates.
    return list(dict.fromkeys(items))


# ---------------------------------------------------------------------
# Preflight gate — prerequisites for START / Flash Cut / cold recovery
# ---------------------------------------------------------------------


def _check_preflight(context: str = "campaign") -> tuple[bool, list[str]]:
    """Check the CLOCKS policy gate plus fresh local Pi prerequisites.

    This path is used for cold START, Flash Cut, and zero-row cold recovery.
    Warm recovery has its own narrower lifecycle contract.
    """
    reasons: list[str] = []

    # -----------------------------------------------------------------
    # 0. Single unified CLOCKS readiness profile
    # -----------------------------------------------------------------
    feature_ready, feature_reasons = _check_feature_preflight(context)
    if not feature_ready:
        reasons.extend(feature_reasons)

    # -----------------------------------------------------------------
    # 1. GNSS state carried by the canonical CLOCKS snapshot
    # -----------------------------------------------------------------
    try:
        with _clocks_lock:
            latest = copy.deepcopy(_latest_clocks)
        gnss = latest.get("gnss") if isinstance(latest.get("gnss"), dict) else {}
        if not gnss:
            reasons.append("SYSTEM.REPORT GNSS context unavailable")
        else:
            if not gnss.get("time_valid"):
                reasons.append("GNSS time not valid (no satellite time/date)")
            lock_quality = str(gnss.get("lock_quality") or "WEAK").upper()
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
                freq_mode = discipline.get("freq_mode", -1) if isinstance(discipline, dict) else -1
                freq_mode_name = discipline.get("freq_mode_name", "UNKNOWN") if isinstance(discipline, dict) else "UNKNOWN"
                if int(freq_mode) < 2:
                    reasons.append(
                        f"GNSS discipline not locked "
                        f"(freq_mode={freq_mode} '{freq_mode_name}', need at least COARSE_LOCK)"
                    )
    except Exception as e:
        reasons.append(f"SYSTEM.REPORT GNSS preflight failed: {e}")

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
    """Wait quietly until the unified CLOCKS readiness profile is open.

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
        "baseline_campaign_type": CAMPAIGN_TYPE_TEMPEST,
        "baseline_campaign": info.get("baseline_campaign"),
        "baseline_pps_vclock_n": info.get("baseline_pps_vclock_n"),
        "baseline_pps_n": info.get("baseline_pps_n"),
    }

    try:
        with open_db(row_dict=True) as conn:
            cur = conn.cursor()
            cur.execute(
                """
                SELECT id, campaign, payload
                FROM campaign_master
                WHERE campaign_type = %s AND id = %s
                """,
                (CAMPAIGN_TYPE_TEMPEST, baseline_id),
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
    """Delete one stopped TEMPEST campaign and every associated detail."""
    if not args or "campaign" not in args:
        return {"success": False, "message": "DELETE requires 'campaign' argument"}

    campaign_name = args["campaign"]

    row = _get_active_campaign()
    if row is not None and row["campaign"] == campaign_name:
        return {
            "success": False,
            "message": f"Campaign '{campaign_name}' is active — STOP it first",
        }

    try:
        with open_db() as conn:
            cur = conn.cursor()
            cur.execute(
                """
                DELETE FROM campaign_detail
                WHERE campaign_type = %s AND campaign = %s
                """,
                (CAMPAIGN_TYPE_TEMPEST, campaign_name),
            )
            detail_count = cur.rowcount
            cur.execute(
                """
                DELETE FROM campaign_master
                WHERE campaign_type = %s AND campaign = %s
                """,
                (CAMPAIGN_TYPE_TEMPEST, campaign_name),
            )
            master_count = cur.rowcount
    except Exception as e:
        logging.exception("❌ [clocks] DELETE failed for campaign '%s'", campaign_name)
        return {"success": False, "message": str(e)}

    if master_count == 0:
        return {"success": False, "message": f"No TEMPEST campaign named '{campaign_name}'"}

    logging.info(
        "🗑️ [clocks] DELETE: type=%s campaign='%s' — %d master row(s), "
        "%d associated detail row(s) deleted",
        CAMPAIGN_TYPE_TEMPEST,
        campaign_name,
        master_count,
        detail_count,
    )

    server_args = {
        "campaign_type": CAMPAIGN_TYPE_TEMPEST,
        "campaign": campaign_name,
    }
    send_command(
        machine="SERVER",
        subsystem="SYSTEM",
        command="DELETE_CAMPAIGN",
        args=server_args,
    )

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "campaign_type": CAMPAIGN_TYPE_TEMPEST,
            "campaign": campaign_name,
            "campaign_master_deleted": master_count,
            "campaign_details_deleted": detail_count,
        },
    }



def cmd_truncate(args: Optional[dict]) -> Dict[str, Any]:
    """Delete stopped TEMPEST campaign history while retaining ambient state."""
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

    _campaign_active = False
    _clear_sync_wait()
    _clear_flash_cut_wait_state()
    candidate_drained = _reset_trackers()
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

    persistence_was_enabled = _clocks_persistence_enabled.is_set()
    _clocks_persistence_enabled.clear()
    try:
        with _clocks_persistence_lock:
            state_drained = _drain_clocks_persistence_queue()
            with open_db() as conn:
                cur = conn.cursor()
                cur.execute(
                    "SELECT COUNT(*) FROM campaign_detail "
                    "WHERE campaign_type = %s AND campaign IS NOT NULL",
                    (CAMPAIGN_TYPE_TEMPEST,),
                )
                detail_count = int(cur.fetchone()[0])
                cur.execute(
                    "SELECT COUNT(*) FROM campaign_master WHERE campaign_type = %s",
                    (CAMPAIGN_TYPE_TEMPEST,),
                )
                master_count = int(cur.fetchone()[0])
                cur.execute(
                    "DELETE FROM campaign_detail "
                    "WHERE campaign_type = %s AND campaign IS NOT NULL",
                    (CAMPAIGN_TYPE_TEMPEST,),
                )
                cur.execute(
                    "DELETE FROM campaign_master WHERE campaign_type = %s",
                    (CAMPAIGN_TYPE_TEMPEST,),
                )
    except Exception as e:
        logging.exception("❌ [clocks] TRUNCATE failed")
        return {"success": False, "message": str(e)}
    finally:
        if persistence_was_enabled:
            _clocks_persistence_enabled.set()

    logging.warning(
        "🧨 [clocks] TRUNCATE: dropped %d TEMPEST masters and %d campaign-associated "
        "details; ambient state retained; drained candidates=%d pending_states=%d",
        master_count, detail_count, candidate_drained, state_drained,
    )

    server_args = {
        "source": "CLOCKS.TRUNCATE",
        "campaign_type": CAMPAIGN_TYPE_TEMPEST,
        "postgres_campaign_master_deleted": master_count,
        "postgres_campaign_details_deleted": detail_count,
        "ambient_campaign_details_retained": True,
    }
    try:
        server_resp = send_command(
            machine="SERVER",
            subsystem="SYSTEM",
            command="TRUNCATE",
            args=server_args,
        )
    except Exception as e:
        logging.exception("⚠️ [clocks] SERVER.SYSTEM.TRUNCATE failed after local cutover")
        server_resp = {"success": False, "message": str(e)}

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "campaign_type": CAMPAIGN_TYPE_TEMPEST,
            "campaign_master_deleted": master_count,
            "campaign_details_deleted": detail_count,
            "ambient_campaign_details_retained": True,
            "candidate_ingress_drained": candidate_drained,
            "pending_state_rows_drained": state_drained,
            "server_truncate_success": (
                bool(server_resp.get("success")) if isinstance(server_resp, dict) else False
            ),
            "server_truncate_message": (
                server_resp.get("message") if isinstance(server_resp, dict) else None
            ),
            "server_truncate_payload": (
                server_resp.get("payload") if isinstance(server_resp, dict) else None
            ),
        },
    }




# ---------------------------------------------------------------------
# LIST_CAMPAIGNS
# ---------------------------------------------------------------------


def cmd_list_campaigns(_: Optional[dict]) -> Dict[str, Any]:
    """List TEMPEST campaign master rows."""
    baseline_info = _get_baseline_from_config()
    baseline_campaign = baseline_info.get("baseline_campaign") if baseline_info else None

    try:
        with open_db(row_dict=True) as conn:
            cur = conn.cursor()
            cur.execute(
                """
                SELECT id, campaign_type, campaign, active, ts, payload
                FROM campaign_master
                WHERE campaign_type = %s
                ORDER BY ts ASC
                """,
                (CAMPAIGN_TYPE_TEMPEST,),
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
        is_baseline = row["campaign"] == baseline_campaign

        entry: Dict[str, Any] = {
            "campaign_type": row["campaign_type"],
            "campaign": row["campaign"],
            "active": bool(row["active"]),
            "baseline": is_baseline,
            "started_at": payload.get("started_at"),
            "stopped_at": payload.get("stopped_at"),
            "resumed_at": payload.get("resumed_at"),
            "location": payload.get("location"),
            "public_count": report.get("public_count"),
            "sequence": report.get("sequence"),
        }
        campaigns.append(entry)

    return {
        "success": True,
        "message": "OK",
        "payload": {
            "campaign_type": CAMPAIGN_TYPE_TEMPEST,
            "count": len(campaigns),
            "campaigns": campaigns,
        },
    }



# ---------------------------------------------------------------------
# CLOCKS_INFO
# ---------------------------------------------------------------------


def cmd_clocks_info(_: Optional[dict]) -> Dict[str, Any]:
    payload = {
        "campaign_active": _campaign_active,
        "integrity_contract": {
            "mode_free": True,
            "coherent_rows_persist": True,
            "science_exclusions_are_audit_only": True,
            "continuity_fatal_triggers_recovery": True,
        },
        "last_pps_vclock_count_seen": _last_pps_vclock_count_seen,
        "accepted_pps_vclock_count": _accepted_pps_vclock_count,
        "sync_expected_pps_vclock": _sync_expected_pps_vclock,
        "sync_expected_pps": _sync_expected_pps_vclock,
        "startup": _start_status_payload(),
        "feature_preflight": {
            "profile": FEATURE_PREFLIGHT_PROFILE,
            "required_features": list(FEATURE_PREFLIGHT_REQUIRED),
            "post_start_expected_features": list(FEATURE_PREFLIGHT_POST_START_EXPECTED),
            "admission_authority": "CLOCKS.features",
            "firmware_policy_gate_used": False,
            "firmware_start_contract": (
                "COMMAND_STATE_EPOCH_PRIVATE_HANDOFF"
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






def cmd_set_dac(args: Optional[dict]) -> Dict[str, Any]:
    """Set the live Teensy DAC target without creating a persistence side path."""
    if not args:
        return {"success": False, "message": "SET_DAC requires DAC1 and/or DAC2"}

    if "campaign" in args:
        return {
            "success": False,
            "message": (
                "SET_DAC campaign=... has been retired. DAC history remains "
                "available in TIMEBASE, while current control is explicit."
            ),
        }

    try:
        has_dac1, dac1, alias1 = _parse_dac_arg(
            args, "DAC1", "dac1", "DAC1", "set_dac1", "SET_DAC1",
            "ocxo1_dac", "OCXO1_DAC"
        )
        has_dac2, dac2, alias2 = _parse_dac_arg(
            args, "DAC2", "dac2", "DAC2", "set_dac2", "SET_DAC2",
            "ocxo2_dac", "OCXO2_DAC"
        )
    except ValueError as exc:
        return {"success": False, "message": str(exc)}

    if not has_dac1 and not has_dac2:
        return {"success": False, "message": "SET_DAC requires DAC1 and/or DAC2"}

    teensy_args: Dict[str, Any] = {}
    if has_dac1 and dac1 is not None:
        teensy_args["set_dac1"] = str(float(dac1))
    if has_dac2 and dac2 is not None:
        teensy_args["set_dac2"] = str(float(dac2))

    try:
        response = send_command(
            machine="TEENSY",
            subsystem="CLOCKS",
            command="SET_DAC",
            args=teensy_args,
        )
    except Exception as exc:
        logging.exception("❌ [clocks] live SET_DAC failed")
        return {"success": False, "message": str(exc)}

    payload = response.get("payload") if isinstance(response, dict) else None
    result = {
        "ocxo1_dac": float(dac1) if has_dac1 and dac1 is not None else None,
        "ocxo2_dac": float(dac2) if has_dac2 and dac2 is not None else None,
        "input_aliases": {
            **({"dac1": alias1} if alias1 else {}),
            **({"dac2": alias2} if alias2 else {}),
        },
        "persistence": "CLOCKS",
        "teensy_message": response.get("message") if isinstance(response, dict) else None,
        "teensy_payload": payload if isinstance(payload, dict) else {},
    }
    logging.info("🔧 [clocks] live SET_DAC: %s", result)
    return {
        "success": bool(response.get("success")) if isinstance(response, dict) else False,
        "message": response.get("message", "OK") if isinstance(response, dict) else "SET_DAC failed",
        "payload": result,
    }



def cmd_set_dither(args: Optional[dict]) -> Dict[str, Any]:
    """Set live Teensy dithering state; CLOCKS owns restart continuity."""
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
                return {
                    "success": False,
                    "message": f"Invalid dither rate for {key}: {args.get(key)!r}",
                }
            break

    if rate_hz is not None and not (1 <= rate_hz <= 1000):
        return {
            "success": False,
            "message": f"dither rate_hz must be 1..1000, got {rate_hz}",
        }

    teensy_args: Dict[str, Any] = {"dither": dither}
    if rate_hz is not None:
        teensy_args["rate_hz"] = int(rate_hz)

    try:
        response = send_command(
            machine="TEENSY",
            subsystem="CLOCKS",
            command="DITHER",
            args=teensy_args,
        )
    except Exception as exc:
        logging.exception("❌ [clocks] live DITHER update failed")
        return {"success": False, "message": str(exc)}

    logging.info(
        "🔧 [clocks] live DITHER: %s resp=%s",
        teensy_args,
        response.get("message", "?") if isinstance(response, dict) else "?",
    )
    return {
        "success": bool(response.get("success")) if isinstance(response, dict) else False,
        "message": response.get("message", "OK") if isinstance(response, dict) else "DITHER failed",
        "payload": {
            **teensy_args,
            "persistence": "CLOCKS",
            "teensy_payload": (
                response.get("payload", {}) if isinstance(response, dict) else {}
            ),
        },
    }



COMMANDS = {
    "START": cmd_start,
    "STOP": cmd_stop,
    "RESUME": cmd_resume,
    "RECOVER_ABORT": cmd_recover_abort,
    "RECOVERY_ABORT": cmd_recover_abort,
    "REPORT": cmd_report,
    "REPORT_CLOCKS": cmd_report_clocks,
    "REPORT_STATS": cmd_report_stats,
    "STATS_RESET": cmd_stats_reset,
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
    """Let PubSub, SYSTEM, and Teensy CLOCKS reach a queryable startup state."""
    logging.info(
        "⏳ [clocks] waiting %.1fs for pubsub routing, SYSTEM context, and Teensy initialization",
        STARTUP_TEENSY_QUIET_DELAY_S,
    )
    time.sleep(STARTUP_TEENSY_QUIET_DELAY_S)
    logging.info("✅ [clocks] startup quiet delay complete — holistic restore may begin")



def run() -> None:
    setup_logging()
    _setup_invalid_timebase_logger()

    _startup_control_ready.clear()
    _clocks_persistence_enabled.clear()
    _diag["startup_control_ready"] = False

    logging.info(
        "🕐 [clocks] CLOCKS owns CLOCKS_FRAGMENT ingestion, SYSTEM.REPORT context pull, "
        "canonical CLOCKS publication, campaign_detail persistence, TEMPEST adjudication, "
        "and holistic subsystem restore. Campaign execution is restored as CLOCKS state."
    )

    server_setup(
        subsystem="CLOCKS",
        commands=COMMANDS,
        subscriptions={
            CLOCKS_FRAGMENT_TOPIC: on_clocks_fragment,
            "WATCHDOG_ANOMALY": on_watchdog_anomaly,
            CLOCKS_RECOVERY_STALLED_TOPIC: on_recovery_stalled,
        },
        blocking=False,
    )

    # Start the live data plane immediately. Persistence remains closed, so the
    # startup stream can populate the UI and restore proof without authoring new
    # durable state before the one holistic transaction has consumed the old one.
    threading.Thread(
        target=_clocks_state_loop,
        daemon=True,
        name="clocks-state",
    ).start()
    threading.Thread(
        target=_clocks_persistence_loop,
        daemon=True,
        name="clocks-persistence",
    ).start()
    threading.Thread(
        target=_process_loop,
        daemon=True,
        name="clocks-tempest",
    ).start()

    startup_teensy_quiet_delay()

    try:
        result = _holistic_restore()
        logging.info("✅ [holistic restore] complete: %s", result)
    except TeensyStartRejected as exc:
        logging.error(
            "💥 [holistic restore] campaign START rejected (%s); live CLOCKS persistence remains open",
            exc.status,
        )
        _cleanup_after_recovery_failure(
            "holistic_start_rejected",
            {"error": str(exc), "status": exc.status},
        )
    except Exception as exc:
        logging.exception(
            "💥 [holistic restore] failed; live CLOCKS persistence remains open and commands remain available"
        )
        _clocks_persistence_enabled.set()
        try:
            _cleanup_after_recovery_failure(
                "holistic_restore_failed",
                {"error": str(exc)},
            )
        except Exception:
            logging.exception("⚠️ [holistic restore] cleanup also failed")
    finally:
        _clocks_persistence_enabled.set()
        _startup_control_ready.set()
        _diag["startup_control_ready"] = True
        logging.info("✅ [clocks] startup state reconciliation complete — START/RESUME enabled")

    threading.Thread(
        target=_timebase_silence_monitor_loop,
        daemon=True,
        name="clocks-timebase-silence-monitor",
    ).start()

    logging.info("🏁 [clocks] entering main loop")
    while True:
        time.sleep(3600)




if __name__ == "__main__":
    run()
