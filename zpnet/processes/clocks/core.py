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
      raw-cycle evidence, firmware statistics, and TEMPEST science.
    • Pi owns: GNSS_RAW, GF-8802 correlation, environment correlation,
      campaign lifecycle, final acceptance court, recovery orchestration,
      and PostgreSQL persistence.
    • SYSTEM owns: current Pi/GNSS/environment/power context and selected physical
      location, exposed by SYSTEM.REPORT and realized through GNSS.

  START behavior:

    START is asynchronous. The Pi creates/activates the campaign, sends the
    Teensy START command, and returns. The first public campaign row arrives
    later through the normal CLOCKS_FRAGMENT processor and becomes a typed
    TEMPEST campaign detail only after Pi adjudication. Firmware may privately
    acquire a lawful PPS0/start-prologue bookend before public PPS1; the Pi does
    not model those private candidates as skipped campaign rows.

  RECOVER behavior:

    Recovery has one top-level producer fork.  If the current Teensy proves lawful
    descent from the durable snapshot, Pi does not mutate it: it reacquires the
    producer-owned bounded rings, restores Pi-owned custody, and adopts the producer's
    current TEMPEST state.  Missing Pi observations remain missing; they are never
    replayed or reconstructed.

    If producer continuity is not proved, the current declarative CLOCKS_RECOVERY
    snapshot is projected to the current GNSS time and one structured DEAD_PRODUCER_RESTORE
    transaction restores canonical producer state, its literal Pi Better-Buckets
    checkpoint, and the raw TEMPEST state captured at that same durable boundary.
    PostgreSQL history is never replayed to reconstruct firmware statistics. The Pi
    waits for exact N+1
    producer proof and the first accepted projected public row before opening ordinary
    persistence. Timeline rows may be admitted while OCXO science is explicitly
    degraded/quarantined, but the final court still rejects malformed or incoherent
    candidates.

Responsibilities:
  * Receive optional TEMPEST campaign enrichment from the unified CLOCKS_FRAGMENT stream.
  * Adjudicate each candidate: persist coherent audit rows and recover on structural loss.
  * Subscribe to predictive GF-8802 announcements and bind them to PPS-aligned rows.
  * Augment accepted rows with environment, GNSS_RAW, and system time.
  * Publish canonical CLOCKS and attach adjudicated TEMPEST facts to unified state details.
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
import hashlib
import json
from collections import deque
from dataclasses import dataclass, field
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

from smbus2 import SMBus, i2c_msg

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

# OCXO DAC custody moved from Teensy CLOCKS to Pi CLOCKS.  These addresses are
# authoritative ZPNet pin-map identities: 0x4E -> OCXO1, 0x4C -> OCXO2.
DAC_I2C_BUS = 1
AD5693R_ADDR_OCXO1 = 0x4E
AD5693R_ADDR_OCXO2 = 0x4C
AD5693R_CMD_WRITE_INPUT_REG = 0x10
AD5693R_CMD_UPDATE_DAC_REG = 0x20
AD5693R_CMD_WRITE_DAC_AND_INPUT = 0x30
AD5693R_CMD_WRITE_CONTROL = 0x40
AD5693R_CTRL_INTERNAL_VREF_2X = 0x0800
AD5693R_INTERNAL_REF_VOLTAGE = 2.5
AD5693R_OUTPUT_GAIN = 2.0
AD5693R_OUTPUT_FULL_SCALE_VOLTAGE = 5.0
AD5693R_SAFE_MAX_OUTPUT_VOLTAGE = 3.3
AD5693R_CODE_SCALE = 65536.0
AD5693R_SAFE_MAX_HW_CODE = int(
    (AD5693R_SAFE_MAX_OUTPUT_VOLTAGE / AD5693R_OUTPUT_FULL_SCALE_VOLTAGE)
    * AD5693R_CODE_SCALE
)

# Mechanical port of the Teensy servo/dither tuning.  The dither waveform is
# one low-first one-second frame: floor(target), then at most one transition to
# floor(target)+1 for the fractional high dwell.  Thus each lane authors at
# most two DAC codes per second.
DAC_DITHER_FRAME_S = 1.0
DAC_DITHER_SLOTS_PER_FRAME = 1000
DAC_STATIC_SERVO_SERVICE_DELAY_S = 0.250
SERVO_MAX_STEP = 64.0
SERVO_TARGET_PPB = 0.100
SERVO_CAMP_SETTLE_SECONDS = 5
SERVO_TOTAL_MIN_SAMPLES = 10
SERVO_PPB_PER_DAC_LSB_ESTIMATE = 100.0
SERVO_CAMP_DEADBAND_PPB = 3.0
SERVO_SOFT_LANDING_PPB = 300.0
SERVO_CAMP_FILTER_ALPHA = 0.35
SERVO_CAMP_GAIN = 1.0
SERVO_10_MIN_FILTER_ALPHA = 1.0
SERVO_10_MIN_GAIN = 1.0
SERVO_10_MIN_DEADBAND_PPB = 0.025
SERVO_DITHER_OWNER_SETTLE_QUARANTINE_ROWS = 1
SERVO_DITHER_OWNER_FAILURE_BACKOFF_ROWS = 3
SERVO_TOTAL_RECENT_WINDOW_SAMPLES = 30
# TOTAL capture tuning: reacquire the post-actuation rate quickly and drive
# accumulated frequency debt down on a short horizon.  The coarse actuator
# remains bounded by SERVO_TOTAL_COARSE_MAX_STEP_LSB, so aggression comes from
# faster observation/control cadence and earlier coarse authority rather than
# an unbounded DAC jump.
SERVO_TOTAL_COARSE_MIN_SAMPLES = 3
SERVO_TOTAL_FINE_MIN_SAMPLES = 12
SERVO_TOTAL_COARSE_THRESHOLD_PPB = 5.0
SERVO_TOTAL_COARSE_EXIT_PPB = 1.0
SERVO_TOTAL_COARSE_RATE_ENTRY_PPB = 25.0
SERVO_TOTAL_COARSE_RATE_EXIT_PPB = 5.0
SERVO_TOTAL_COARSE_HORIZON_SECONDS = 60.0
SERVO_TOTAL_FINE_HORIZON_SECONDS = 180.0
SERVO_TOTAL_COARSE_MAX_RATE_PPB = 400.0
SERVO_TOTAL_FINE_MAX_RATE_PPB = 40.0
SERVO_TOTAL_COARSE_MAX_STEP_LSB = 4.0
SERVO_TOTAL_FINE_MAX_STEP_LSB = 0.5
SERVO_TOTAL_COARSE_ACTUATOR_GAIN = 1.0
SERVO_TOTAL_FINE_ACTUATOR_GAIN = 1.0
SERVO_TOTAL_COARSE_RATE_DEADBAND_PPB = 8.0
SERVO_TOTAL_FINE_RATE_DEADBAND_PPB = 0.5
SERVO_TOTAL_COARSE_STDERR_MULTIPLIER = 3.0
SERVO_TOTAL_FINE_STDERR_MULTIPLIER = 1.5
SERVO_TOTAL_POSITION_HOLD_PPB = 0.005
SERVO_TOTAL_NEAR_TARGET_RATE_HOLD_PPB = 2.0
SERVO_TOTAL_TARGET_CROSSING_LOOKAHEAD_SECONDS = 60.0
SERVO_TOTAL_DAC_CHANGE_EPSILON_LSB = 0.000001

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
# Dead-producer recovery uses its narrower recovery-specific lifecycle contract. The
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
# its bounded private proof window, publish degraded rows that let the Pi
# observe liveness.  If the first row never appears, abort the firmware
# RECOVER lifecycle explicitly instead of recursively hard-faulting the Pi-side
# recovery thread.
RECOVERY_FIRST_ROW_TIMEOUT_S = 45.0

# A post-flash RECOVER may first need startup SmartZero to acquire and install a
# fresh local service epoch before firmware can launch the ordinary RECOVER grid
# rephase. Dead-producer restore receives the full recovery-admission
# window; surviving producers never enter this mutating path.
RECOVERY_DEAD_PRODUCER_FIRST_ROW_TIMEOUT_S = 180.0

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
# proves fresh OCXO custody, but once a public TIMEBASE row reaches the Pi,
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
PREFLIGHT_POLL_INTERVAL_S = 1.0
PREFLIGHT_QUIET_GRACE_S = 30.0
PREFLIGHT_STATUS_LOG_INTERVAL_S = 60.0
PREFLIGHT_LOG_PREFIX = "🛡️ [preflight]"
STARTUP_LOCATION_RETRY_S = 5.0
STARTUP_LOCATION_STATUS_LOG_INTERVAL_S = 30.0
STARTUP_REQUIRED_GNSS_FREQ_MODE = 3       # GF-8802 TPS4 FINE_LOCK

# Application-level startup infrastructure admission.  SYSTEM owns these facts;
# CLOCKS consumes them before the first durable read or Teensy command.  Ingress
# remains open while the gate is closed so a surviving Alpha can continue sending
# exact CLOCKS_FRAGMENT testimony into startup custody.
STARTUP_INFRASTRUCTURE_REQUIRED = (
    "PI.SYSTEM.POSTGRES",
    "PI.PUBSUB.TEENSY_RPC",
)
STARTUP_INFRASTRUCTURE_POLL_S = 0.5
STARTUP_INFRASTRUCTURE_QUIET_GRACE_S = 5.0
STARTUP_INFRASTRUCTURE_STATUS_LOG_INTERVAL_S = 30.0
# Newborn Alpha acquisition is a physical startup phase, not part of the
# restored-row proof deadline.  Poll REPORT_RECOVERY until firmware says the
# current lifetime has an installed SmartZero-backed Alpha epoch.
STARTUP_ALPHA_EPOCH_POLL_S = 0.25
STARTUP_ALPHA_EPOCH_STATUS_LOG_INTERVAL_S = 10.0
# Producer survival must be decided from current-lifetime canonical testimony.
# A retained pre-flash row may be delivered immediately after firmware restart;
# receipt freshness is therefore insufficient. Wait through the bounded newborn-
# Alpha acquisition window for a row that actually entered CLOCKS after the current
# transport/session admission barrier.
STARTUP_SURVIVAL_FRESH_WITNESS_TIMEOUT_S = RECOVERY_DEAD_PRODUCER_FIRST_ROW_TIMEOUT_S
HOLISTIC_RESTORE_TIMEOUT_S = 60.0
HOLISTIC_RESTORE_COMMAND_RETRY_S = 10.0

# Better-Buckets recovery custody. Firmware publishes one compact, authoritative
# CLOCKS_PPB_CHECKPOINT_DELTA_V1 proof/delta on every CLOCKS_FRAGMENT. Pi CLOCKS
# validates that testimony and maintains the literal bounded endpoint rings. The
# complete recovery image is private durable continuation state in the singleton
# config.CLOCKS_RECOVERY row; canonical CLOCKS observations never carry the ring.
# Raw PostgreSQL history is never replayed to reconstruct Alpha state.
PPB_10_MIN_SECONDS = 10 * 60
PPB_60_MIN_SECONDS = 60 * 60
PPB_8_HOUR_SECONDS = 8 * 60 * 60
PPB_24_HOUR_SECONDS = 24 * 60 * 60
PPB_SECOND_CAPACITY = PPB_10_MIN_SECONDS + 1
PPB_MINUTE_CAPACITY = 24 * 60 + 2
PPB_RESTORE_CHUNK_ENDPOINTS = 4
PPB_PROOF_VERIFY_TOLERANCE_PPB = 0.00001
PPB_FIRMWARE_DELTA_SCHEMA = "CLOCKS_PPB_CHECKPOINT_DELTA_V1"
PPB_PI_CHECKPOINT_SCHEMA = "PI_CLOCKS_PPB_RESTORE_CHECKPOINT_V1"
CLOCKS_RECOVERY_CONFIG_KEY = "CLOCKS_RECOVERY"
CLOCKS_ALPHA_LINEAGE_CUTOFF_CONFIG_KEY = "CLOCKS_ALPHA_LINEAGE_CUTOFF"
CLOCKS_ALPHA_LINEAGE_CUTOFF_SCHEMA = "PI_CLOCKS_ALPHA_LINEAGE_CUTOFF_V1"
CLOCKS_RECOVERY_SNAPSHOT_SCHEMA = "PI_CLOCKS_RECOVERY_SNAPSHOT_V1"
CLOCKS_RECOVERY_DESIRED_STATE_SCHEMA = "PI_CLOCKS_RECOVERY_DESIRED_STATE_V1"
CLOCKS_RECOVERY_RECEIPT_SCHEMA = "PI_CLOCKS_RECOVERY_RECEIPT_V1"
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
    # FEATURE_STATUS-driven Teensy import sentinel. HOST is intentionally
    # telemetry-only: transient Pi load/temperature/memory pressure must not
    # block scientific campaign admission.
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

    # First-class subsystem operational state / terminal hard hold.
    "hard_failure_entries": 0,
    "hard_failure_ingress_dropped": 0,
    "hard_failure_state_dropped": 0,
    "hard_failure_persistence_dropped": 0,
    "hard_failure_campaign_rows_dropped": 0,
    "repair_requests": 0,
    "repair_success": 0,
    "repair_failures": 0,
    "last_repair": {},
    "last_hard_failure_hold": {},

    # Lawful Alpha lineage surrender. Exact resurrection may be impossible
    # without making the currently acquired clock substrate unusable.
    "alpha_lineage_surrenders": 0,
    "last_alpha_lineage_surrender": {},

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

    # Better-Buckets checkpoint custody. These counters describe validation and
    # completeness of the Pi-owned literal restore image; they never redefine
    # the Teensy statistical population.
    "ppb_checkpoint_rows_verified": 0,
    "ppb_checkpoint_recoverable_rows": 0,
    "ppb_checkpoint_warming_rows": 0,
    "ppb_checkpoint_gap_count": 0,
    "ppb_checkpoint_epoch_rebases": 0,
    "ppb_checkpoint_seed_loaded": 0,
    "ppb_checkpoint_seed_missing": 0,
    "ppb_checkpoint_live_refresh_attempts": 0,
    "ppb_checkpoint_live_refresh_success": 0,
    "ppb_checkpoint_live_refresh_failures": 0,
    "last_ppb_checkpoint_live_refresh": {},
    "ppb_restore_transition_rows_discarded": 0,
    "last_ppb_restore_transition_row": {},
    "last_ppb_checkpoint": {},

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
    "gnss_raw_physical_sequence_rebases": 0,
    "last_gnss_raw_physical_sequence_rebase": {},
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
    "watchdog_anomaly_startup_deferred": 0,
    "watchdog_anomaly_startup_reconciled": 0,
    "last_watchdog_anomaly": {},
    "last_startup_deferred_watchdog": {},

    # Dedicated recovery-liveness anomaly.  This is observational and never
    # initiates RECOVER; restarting would destroy the evidence being awaited.
    "recovery_stalled_events_received": 0,
    "recovery_stalled_event_enqueue_failures": 0,
    "recovery_proof_search_exclusions_suppressed": 0,
    "last_recovery_proof_warning": {},
    "last_recovery_stalled": {},

    # SYSTEM-owned CLOCKS recovery may hand the Pi-owned GNSS_RAW state back
    # to CLOCKS before CLOCKS startup campaign reconciliation completes.

    # CLOCKS startup lifecycle serialization
    "startup_control_ready": False,
    "startup_control_busy_rejections": 0,
    "last_startup_control_rejection": {},
    "startup_infrastructure_waits": 0,
    "startup_infrastructure_wait_seconds_last": 0.0,
    "last_startup_infrastructure_wait": {},
    "startup_location_waits": 0,
    "startup_location_wait_seconds_last": 0.0,
    "last_startup_location_wait": {},
    "last_startup_alpha_epoch_wait": {},
    "startup_custody_active": True,
    "startup_custody_depth": 0,
    "startup_custody_retained": 0,
    "startup_custody_released": 0,
    "startup_custody_quarantined": 0,
    "startup_custody_retired": 0,
    "startup_custody_last_sequence": None,
    "startup_physical_lifetime_boundaries": 0,
    "startup_preclassification_rows_non_authoritative": 0,
    "last_startup_physical_lifetime_boundary": {},
    "last_startup_custody_release": {},
    "last_startup_custody_quarantine": {},
    "last_startup_custody_retire": {},

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

    # Ambient instrument recovery. A physical Teensy lifetime replacement is
    # an Alpha custody event even when TEMPEST is stopped. These counters prove
    # that the instrument-only resurrection path ran without inventing a campaign.
    "ambient_instrument_recovery_started": 0,
    "ambient_instrument_recovery_completed": 0,
    "ambient_instrument_recovery_failures": 0,
    "ambient_instrument_recovery_rows_retired": 0,
    "last_ambient_instrument_recovery": {},

    # Live recovery persistence custody. Rows observed after continuity surrender
    # remain durable evidence, but are quarantined from restore authority until
    # the recovery transaction classifies the instrument lineage.
    "recovery_custody_active": False,
    "recovery_custody_generation": None,
    "recovery_custody_begin_count": 0,
    "recovery_custody_rows_quarantined": 0,
    "recovery_custody_rows_promoted": 0,
    "recovery_custody_rows_superseded": 0,
    "recovery_custody_physical_regressions": 0,
    "last_recovery_custody": {},

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
    "flash_cut_pre_cut_tail_retired": 0,
    "last_flash_cut_pre_cut_tail": {},
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


_EXPECTED_RECOVERY_PROOF_EXCLUSION_REASONS = {
    "alpha_counterledger_capture",
    "beta_recovery_science_hold",
}

def _firmware_exclusion_is_expected_recovery_proof_search(
    fragment: Dict[str, Any],
    exclusion: Dict[str, Any],
) -> bool:
    """True only for expected firmware custody holds during RECOVER proof search."""
    reason = str(
        exclusion.get("reason_name")
        or exclusion.get("reason")
        or ""
    ).strip().lower()
    if reason not in _EXPECTED_RECOVERY_PROOF_EXCLUSION_REASONS:
        return False

    # _tempest_candidate_view() adds recover_generation only when firmware
    # attached recovery testimony to this exact row.  That includes the boundary
    # row on which recovery state clears, so generation identity is a stronger and
    # quieter discriminator than requiring one of the transient flags to remain true.
    generation = _as_int(fragment.get("recover_generation"))
    return generation is not None and generation > 0


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
# Pi-owned OCXO DAC / servo / dither authority
# ---------------------------------------------------------------------
#
# Custody rule:
#   Teensy determines what happened; Pi CLOCKS determines what to do about it.
# Teensy CLOCKS_FRAGMENT supplies only timing/science evidence.  Pi CLOCKS owns
# AD5693R configuration/readback, DAC targets, servo state, fractional dither,
# control commands, and restart continuity.  Canonical CLOCKS is decorated with
# the familiar CLOCKS_CONTROL_V1 surface so downstream consumers need not care
# that actuator custody moved across the machine boundary.

DAC_AUTHOR_NONE = 0
DAC_AUTHOR_STARTUP_OBSERVED = 1
DAC_AUTHOR_EXPLICIT_COMMAND = 2
DAC_AUTHOR_SERVO = 3

SERVO_HOLD_NONE = 0
SERVO_HOLD_PENDING_COMMIT = 1
SERVO_HOLD_SETTLE_QUARANTINE = 2
SERVO_HOLD_COMMIT_FAULT_BACKOFF = 3
SERVO_HOLD_SMALL_STATIC_DELTA = 4


@dataclass
class _DacWelford:
    n: int = 0
    mean: float = 0.0
    m2: float = 0.0
    min_value: float = 0.0
    max_value: float = 0.0

    def reset(self) -> None:
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.min_value = 0.0
        self.max_value = 0.0

    def update(self, value: float) -> None:
        value = float(value)
        if self.n == 0:
            self.n = 1
            self.mean = value
            self.m2 = 0.0
            self.min_value = value
            self.max_value = value
            return
        self.n += 1
        delta = value - self.mean
        self.mean += delta / float(self.n)
        delta2 = value - self.mean
        self.m2 += delta * delta2
        self.min_value = min(self.min_value, value)
        self.max_value = max(self.max_value, value)

    def restore(self, payload: Any) -> None:
        if not isinstance(payload, dict):
            self.reset()
            return
        try:
            n = int(payload.get("n") or 0)
            mean = float(payload.get("mean") or 0.0)
            m2 = float(payload.get("m2") or 0.0)
            min_value = float(payload.get("min") or 0.0)
            max_value = float(payload.get("max") or 0.0)
        except (TypeError, ValueError):
            self.reset()
            return
        if n < 0 or not all(math.isfinite(v) for v in (mean, m2, min_value, max_value)):
            self.reset()
            return
        self.n = n
        self.mean = mean
        self.m2 = max(0.0, m2)
        self.min_value = min_value if n else 0.0
        self.max_value = max_value if n else 0.0

    def snapshot(self) -> Dict[str, Any]:
        n = int(self.n)
        variance = self.m2 / float(n - 1) if n > 1 else 0.0
        stddev = math.sqrt(max(0.0, variance))
        stderr = stddev / math.sqrt(float(n)) if n else 0.0
        return {
            "n": n,
            "mean": float(self.mean) if n else 0.0,
            "m2": float(self.m2) if n else 0.0,
            "stddev": float(stddev),
            "stderr": float(stderr),
            "min": float(self.min_value) if n else 0.0,
            "max": float(self.max_value) if n else 0.0,
        }


@dataclass
class _DacLaneState:
    name: str
    address: int
    target_code: float = 0.0
    hw_code: int = 0
    readback_valid: bool = False
    readback_code: int = 0
    reset_signature: bool = False
    configured: bool = False
    readback_count: int = 0
    readback_failures: int = 0
    adopt_count: int = 0
    same_code_skip_count: int = 0
    last_author_source: int = DAC_AUTHOR_NONE
    write_attempts: int = 0
    write_successes: int = 0
    write_failures: int = 0
    last_write_ok: bool = True
    last_failure_stage: int = 0
    last_attempted_hw_code: int = 0
    last_good_hw_code: int = 0

    dither_enabled: bool = False
    dither_active_this_frame: bool = False
    dither_current_phase_high: bool = False
    dither_low_code: int = 0
    dither_high_code: int = 0
    dither_high_ms: int = 0
    dither_last_frame_high_ms: int = 0
    dither_transition_due: Optional[float] = None
    dither_frame_count: int = 0
    dither_transition_count: int = 0
    dither_write_count: int = 0
    dither_write_failure_count: int = 0
    dither_skip_same_code_count: int = 0

    servo_last_step: float = 0.0
    servo_last_residual: float = 0.0
    servo_settle_count: int = 0
    servo_adjustments: int = 0
    servo_predictor_initialized: bool = False
    servo_last_raw_residual: float = 0.0
    servo_filtered_residual: float = 0.0
    servo_filtered_slope: float = 0.0
    servo_predicted_residual: float = 0.0
    servo_predictor_updates: int = 0
    servo_hold_count: int = 0
    servo_hold_reason: int = SERVO_HOLD_NONE
    servo_quarantine_reason: int = SERVO_HOLD_NONE
    servo_quarantine_remaining: int = 0
    servo_quarantine_begin_count: int = 0
    servo_quarantine_consumed_count: int = 0
    servo_commit_fault_hold_count: int = 0

    pacing_pending: bool = False
    pacing_pending_target: float = 0.0
    pacing_pending_step: float = 0.0
    pacing_pending_hw_code: int = 0
    pacing_pending_since_second: int = 0
    pacing_last_request_second: int = 0
    pacing_last_commit_second: int = 0
    pacing_intents: int = 0
    pacing_deferred_count: int = 0
    pacing_commit_count: int = 0
    pacing_skip_small_delta_count: int = 0
    static_service_due: Optional[float] = None

    total_initialized: bool = False
    total_basis: int = 0
    total_last_commit_count: int = 0
    total_last_dac_value: float = 0.0
    total_last_population_seconds: int = 0
    total_last_sample_control_second: int = 0
    total_recent: deque = field(
        default_factory=lambda: deque(maxlen=SERVO_TOTAL_RECENT_WINDOW_SAMPLES)
    )
    total_samples_since_reset: int = 0
    total_coarse_latched: bool = False

    welford: _DacWelford = field(default_factory=_DacWelford)


_dac_lock = threading.RLock()
_dac_actuator_lock = threading.Lock()
# DAC Welford statistics belong to the Teensy CLOCKS statistics epoch.  This
# lock serializes epoch transitions against the row consumer so a Teensy reset
# cannot leave Pi-owned DAC ancestry attached to a new Alpha epoch.
_dac_stats_epoch_lock = threading.Lock()
_dac_stats_reset_count: Optional[int] = None
_dac_stats_update_count: Optional[int] = None
_dac_stats_last_sequence: Optional[int] = None
_dac_stats_reset_fence_count: Optional[int] = None
# Operator STATS_RESET returns immediately; one background worker owns the
# existing transitive Teensy/Pi reset transaction until it reaches a terminal result.
_stats_reset_control_lock = threading.Lock()
_stats_reset_in_progress = threading.Event()
_dac_control_wakeup = threading.Event()
_dac_control_thread_started = threading.Event()
_dac_hardware_initialized = False
_dac_dither_operator_enabled = False
_dac_servo_mode = "OFF"
_dac_next_frame_monotonic = 0.0
_dac_control_second = 0

_dac_lanes: Dict[str, _DacLaneState] = {
    "ocxo1": _DacLaneState("ocxo1", AD5693R_ADDR_OCXO1),
    "ocxo2": _DacLaneState("ocxo2", AD5693R_ADDR_OCXO2),
}


def _dac_clamp_target(value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("DAC target must be finite")
    return min(float(AD5693R_SAFE_MAX_HW_CODE), max(0.0, value))


def _dac_rounded_hw_code(value: float) -> int:
    return int(_dac_clamp_target(value) + 0.5)


def _dac_voltage_from_code(value: float) -> float:
    return (_dac_clamp_target(value) / AD5693R_CODE_SCALE) * AD5693R_OUTPUT_FULL_SCALE_VOLTAGE


def _ad5693r_write_24(address: int, command: int, data: int) -> None:
    value = int(data) & 0xFFFF
    with SMBus(DAC_I2C_BUS) as bus:
        msg = i2c_msg.write(
            int(address),
            [int(command) & 0xFF, (value >> 8) & 0xFF, value & 0xFF],
        )
        bus.i2c_rdwr(msg)


def _ad5693r_write_command(address: int, command: int) -> None:
    with SMBus(DAC_I2C_BUS) as bus:
        msg = i2c_msg.write(int(address), [int(command) & 0xFF])
        bus.i2c_rdwr(msg)


def _ad5693r_read_input(address: int) -> int:
    with SMBus(DAC_I2C_BUS) as bus:
        msg = i2c_msg.read(int(address), 2)
        bus.i2c_rdwr(msg)
        data = list(msg)
    if len(data) != 2:
        raise OSError(f"AD5693R 0x{address:02X} returned {len(data)} bytes")
    return ((int(data[0]) & 0xFF) << 8) | (int(data[1]) & 0xFF)


def _dac_configure_lane(lane: _DacLaneState) -> bool:
    try:
        _ad5693r_write_24(
            lane.address,
            AD5693R_CMD_WRITE_CONTROL,
            AD5693R_CTRL_INTERNAL_VREF_2X,
        )
    except Exception:
        with _dac_lock:
            lane.configured = False
        logging.exception(
            "❌ [clocks/dac] failed to configure %s AD5693R at 0x%02X",
            lane.name,
            lane.address,
        )
        return False
    with _dac_lock:
        lane.configured = True
    return True


def _dac_read_and_adopt_lane(lane: _DacLaneState) -> bool:
    try:
        code = _ad5693r_read_input(lane.address)
    except Exception:
        with _dac_lock:
            lane.readback_failures += 1
            lane.readback_valid = False
        logging.exception(
            "❌ [clocks/dac] failed to read %s AD5693R input register at 0x%02X",
            lane.name,
            lane.address,
        )
        return False

    with _dac_lock:
        lane.readback_count += 1
        lane.readback_valid = True
        lane.readback_code = int(code)
        lane.hw_code = int(code)
        lane.reset_signature = int(code) == 0
        lane.target_code = _dac_clamp_target(float(code))
        lane.adopt_count += 1
        lane.last_author_source = DAC_AUTHOR_STARTUP_OBSERVED
        lane.last_write_ok = True
        lane.last_failure_stage = 0
        lane.last_good_hw_code = int(code)
    return True


def _dac_initialize_hardware() -> Dict[str, Any]:
    """Configure AD5693R control registers and adopt surviving input codes.

    Initialization is deliberately non-authoring with respect to VOUT: it never
    writes an input code and never issues UPDATE_DAC_REG.
    """
    global _dac_hardware_initialized
    result: Dict[str, Any] = {}
    all_ok = True
    for name in ("ocxo1", "ocxo2"):
        lane = _dac_lanes[name]
        configured = _dac_configure_lane(lane)
        observed = _dac_read_and_adopt_lane(lane) if configured else False
        all_ok = all_ok and configured and observed
        result[name] = {
            "address": f"0x{lane.address:02X}",
            "configured": bool(configured),
            "readback_valid": bool(observed),
            "readback_code": int(lane.readback_code) if observed else None,
        }
    _dac_hardware_initialized = bool(all_ok)
    logging.info(
        "🔧 [clocks/dac] Pi owns both OCXO DACs: "
        "OCXO1 code=%s configured=%s; OCXO2 code=%s configured=%s",
        result["ocxo1"].get("readback_code"),
        result["ocxo1"].get("configured"),
        result["ocxo2"].get("readback_code"),
        result["ocxo2"].get("configured"),
    )
    return result


def _dac_write_hw_code(
    lane: _DacLaneState,
    hw_code: int,
    *,
    author_source: int,
    latch_fault: bool = False,
) -> bool:
    code = max(0, min(AD5693R_SAFE_MAX_HW_CODE, int(hw_code)))
    with _dac_actuator_lock:
        with _dac_lock:
            lane.last_attempted_hw_code = code
            if lane.readback_valid and code == lane.hw_code:
                lane.same_code_skip_count += 1
                lane.dither_skip_same_code_count += 1
                lane.last_write_ok = True
                lane.last_failure_stage = 0
                return True
            lane.write_attempts += 1
            configured = lane.configured

        if not configured and not _dac_configure_lane(lane):
            with _dac_lock:
                lane.write_failures += 1
                lane.last_write_ok = False
                lane.last_failure_stage = 3
            return False

        try:
            _ad5693r_write_24(lane.address, AD5693R_CMD_WRITE_INPUT_REG, code)
        except Exception:
            with _dac_lock:
                lane.write_failures += 1
                lane.last_write_ok = False
                lane.last_failure_stage = 1
            logging.exception(
                "❌ [clocks/dac] %s write-input failed code=%d", lane.name, code
            )
            return False

        try:
            _ad5693r_write_command(lane.address, AD5693R_CMD_UPDATE_DAC_REG)
        except Exception:
            with _dac_lock:
                lane.write_failures += 1
                lane.last_write_ok = False
                lane.last_failure_stage = 2
            logging.exception(
                "❌ [clocks/dac] %s update-DAC failed code=%d", lane.name, code
            )
            return False

        with _dac_lock:
            lane.hw_code = code
            lane.readback_valid = True
            lane.readback_code = code
            lane.reset_signature = code == 0
            lane.last_author_source = int(author_source)
            lane.last_write_ok = True
            lane.last_failure_stage = 0
            lane.write_successes += 1
            lane.last_good_hw_code = code
        _ = latch_fault  # retained semantic argument; Pi reports faults directly.
        return True


def _dac_clear_pending_locked(lane: _DacLaneState) -> None:
    lane.pacing_pending = False
    lane.pacing_pending_target = 0.0
    lane.pacing_pending_step = 0.0
    lane.pacing_pending_hw_code = 0
    lane.pacing_pending_since_second = 0
    lane.static_service_due = None


def _dac_begin_quarantine_locked(lane: _DacLaneState, rows: int, reason: int) -> None:
    lane.servo_quarantine_remaining = int(rows)
    lane.servo_quarantine_reason = int(reason)
    lane.servo_hold_reason = int(reason)
    lane.servo_quarantine_begin_count += 1


def _dac_install_pending_servo_locked(lane: _DacLaneState) -> bool:
    if not lane.pacing_pending:
        return False
    target = lane.pacing_pending_target
    step = lane.pacing_pending_step
    request_second = lane.pacing_pending_since_second
    _dac_clear_pending_locked(lane)
    lane.target_code = target
    lane.servo_last_step = step
    lane.pacing_last_commit_second = request_second
    lane.pacing_commit_count += 1
    _dac_begin_quarantine_locked(
        lane,
        SERVO_DITHER_OWNER_SETTLE_QUARANTINE_ROWS,
        SERVO_HOLD_SETTLE_QUARANTINE,
    )
    return True


def _dac_queue_servo_target(lane: _DacLaneState, target: float, step: float) -> None:
    target = _dac_clamp_target(target)
    with _dac_lock:
        if lane.pacing_pending:
            lane.pacing_deferred_count += 1
        lane.pacing_pending = True
        lane.pacing_pending_target = target
        lane.pacing_pending_step = float(step)
        lane.pacing_pending_hw_code = _dac_rounded_hw_code(target)
        lane.pacing_pending_since_second = int(_dac_control_second)
        lane.pacing_last_request_second = int(_dac_control_second)
        lane.pacing_intents += 1
        lane.servo_last_step = float(step)
        lane.servo_adjustments += 1
        lane.servo_hold_reason = SERVO_HOLD_PENDING_COMMIT
        lane.static_service_due = (
            None
            if _dac_dither_operator_enabled
            else time.monotonic() + DAC_STATIC_SERVO_SERVICE_DELAY_S
        )
    _dac_control_wakeup.set()


def _dac_set_target_explicit(lane: _DacLaneState, value: float) -> bool:
    target = _dac_clamp_target(value)
    with _dac_lock:
        _dac_clear_pending_locked(lane)
        dither = _dac_dither_operator_enabled
    if dither:
        with _dac_lock:
            lane.target_code = target
            lane.last_write_ok = True
            lane.last_failure_stage = 0
        _dac_control_wakeup.set()
        return True

    hw_code = _dac_rounded_hw_code(target)
    if not _dac_write_hw_code(
        lane,
        hw_code,
        author_source=DAC_AUTHOR_EXPLICIT_COMMAND,
        latch_fault=True,
    ):
        return False
    with _dac_lock:
        lane.target_code = target
    return True


def _dac_program_dither_lane_locked(lane: _DacLaneState, frame_start: float) -> int:
    target = _dac_clamp_target(lane.target_code)
    low = int(math.floor(target))
    high = min(AD5693R_SAFE_MAX_HW_CODE, low + 1)
    frac = min(1.0, max(0.0, target - float(low)))
    high_ms = int(frac * DAC_DITHER_SLOTS_PER_FRAME + 0.5)
    if high_ms >= DAC_DITHER_SLOTS_PER_FRAME:
        low = high
        high_ms = 0
    if high == low or high_ms == 0:
        high = low
        high_ms = 0

    lane.dither_enabled = True
    lane.dither_low_code = low
    lane.dither_high_code = high
    lane.dither_high_ms = high_ms
    lane.dither_last_frame_high_ms = high_ms
    lane.dither_active_this_frame = high_ms > 0 and high != low
    lane.dither_current_phase_high = False
    lane.dither_frame_count += 1
    lane.dither_transition_due = (
        frame_start + (DAC_DITHER_SLOTS_PER_FRAME - high_ms) / 1000.0
        if lane.dither_active_this_frame
        else None
    )
    return low


def _dac_begin_dither_frame(frame_start: float) -> None:
    writes: List[Tuple[_DacLaneState, int]] = []
    with _dac_lock:
        if not _dac_dither_operator_enabled:
            return
        for lane in _dac_lanes.values():
            _dac_install_pending_servo_locked(lane)
            writes.append((lane, _dac_program_dither_lane_locked(lane, frame_start)))

    for lane, code in writes:
        ok = _dac_write_hw_code(
            lane,
            code,
            author_source=DAC_AUTHOR_SERVO if _dac_servo_mode != "OFF" else DAC_AUTHOR_EXPLICIT_COMMAND,
        )
        with _dac_lock:
            if ok:
                lane.dither_write_count += 1
            else:
                lane.dither_write_failure_count += 1
                _dac_begin_quarantine_locked(
                    lane,
                    SERVO_DITHER_OWNER_FAILURE_BACKOFF_ROWS,
                    SERVO_HOLD_COMMIT_FAULT_BACKOFF,
                )


def _dac_service_dither_transitions(now: float) -> None:
    writes: List[Tuple[_DacLaneState, int]] = []
    with _dac_lock:
        if not _dac_dither_operator_enabled:
            return
        for lane in _dac_lanes.values():
            due = lane.dither_transition_due
            if due is None or due > now or lane.dither_current_phase_high:
                continue
            lane.dither_transition_due = None
            lane.dither_current_phase_high = True
            lane.dither_transition_count += 1
            writes.append((lane, lane.dither_high_code))

    for lane, code in writes:
        ok = _dac_write_hw_code(
            lane,
            code,
            author_source=DAC_AUTHOR_SERVO if _dac_servo_mode != "OFF" else DAC_AUTHOR_EXPLICIT_COMMAND,
        )
        with _dac_lock:
            if ok:
                lane.dither_write_count += 1
            else:
                lane.dither_write_failure_count += 1
                _dac_begin_quarantine_locked(
                    lane,
                    SERVO_DITHER_OWNER_FAILURE_BACKOFF_ROWS,
                    SERVO_HOLD_COMMIT_FAULT_BACKOFF,
                )


def _dac_service_static_servo(now: float) -> None:
    writes: List[Tuple[_DacLaneState, int]] = []
    with _dac_lock:
        if _dac_dither_operator_enabled:
            return
        for lane in _dac_lanes.values():
            if not lane.pacing_pending or lane.static_service_due is None:
                continue
            if lane.static_service_due > now:
                continue
            _dac_install_pending_servo_locked(lane)
            writes.append((lane, _dac_rounded_hw_code(lane.target_code)))

    for lane, code in writes:
        ok = _dac_write_hw_code(lane, code, author_source=DAC_AUTHOR_SERVO)
        if not ok:
            with _dac_lock:
                _dac_begin_quarantine_locked(
                    lane,
                    SERVO_DITHER_OWNER_FAILURE_BACKOFF_ROWS,
                    SERVO_HOLD_COMMIT_FAULT_BACKOFF,
                )


def _dac_control_loop() -> None:
    global _dac_next_frame_monotonic
    _dac_control_thread_started.set()
    while True:
        if _hard_failure_active():
            _dac_control_wakeup.wait(timeout=1.0)
            _dac_control_wakeup.clear()
            continue

        now = time.monotonic()
        with _dac_lock:
            dither = _dac_dither_operator_enabled
            next_frame = _dac_next_frame_monotonic

        if dither:
            if next_frame <= 0.0 or now >= next_frame:
                frame_start = now
                _dac_begin_dither_frame(frame_start)
                with _dac_lock:
                    _dac_next_frame_monotonic = frame_start + DAC_DITHER_FRAME_S
                now = time.monotonic()
            _dac_service_dither_transitions(now)
        else:
            _dac_service_static_servo(now)

        with _dac_lock:
            deadlines: List[float] = []
            if _dac_dither_operator_enabled:
                if _dac_next_frame_monotonic > 0.0:
                    deadlines.append(_dac_next_frame_monotonic)
                for lane in _dac_lanes.values():
                    if lane.dither_transition_due is not None:
                        deadlines.append(lane.dither_transition_due)
            else:
                for lane in _dac_lanes.values():
                    if lane.pacing_pending and lane.static_service_due is not None:
                        deadlines.append(lane.static_service_due)
        timeout = 1.0
        if deadlines:
            timeout = max(0.001, min(deadlines) - time.monotonic())
        _dac_control_wakeup.wait(timeout=timeout)
        _dac_control_wakeup.clear()


def _dac_start_control_thread() -> None:
    if _dac_control_thread_started.is_set():
        return
    threading.Thread(
        target=_dac_control_loop,
        daemon=True,
        name="clocks-dac-control",
    ).start()


def _dac_set_dither_enabled(enabled: bool) -> None:
    global _dac_dither_operator_enabled, _dac_next_frame_monotonic
    with _dac_lock:
        _dac_dither_operator_enabled = bool(enabled)
        if enabled:
            _dac_next_frame_monotonic = time.monotonic()
            for lane in _dac_lanes.values():
                lane.dither_enabled = True
                lane.dither_transition_due = None
        else:
            _dac_next_frame_monotonic = 0.0
            for lane in _dac_lanes.values():
                lane.dither_enabled = False
                lane.dither_active_this_frame = False
                lane.dither_current_phase_high = False
                lane.dither_transition_due = None
                _dac_clear_pending_locked(lane)
    _dac_control_wakeup.set()


def _dac_reset_total_state_locked(lane: _DacLaneState) -> None:
    lane.total_initialized = False
    lane.total_basis = 0
    lane.total_last_commit_count = 0
    lane.total_last_dac_value = 0.0
    lane.total_last_population_seconds = 0
    lane.total_last_sample_control_second = 0
    lane.total_recent.clear()
    lane.total_samples_since_reset = 0
    lane.total_coarse_latched = False


def _dac_reset_servo_predictor_locked(lane: _DacLaneState) -> None:
    lane.servo_predictor_initialized = False
    lane.servo_last_raw_residual = 0.0
    lane.servo_filtered_residual = 0.0
    lane.servo_filtered_slope = 0.0
    lane.servo_predicted_residual = 0.0
    lane.servo_predictor_updates = 0
    lane.servo_settle_count = 0
    lane.servo_hold_reason = SERVO_HOLD_NONE
    lane.servo_quarantine_reason = SERVO_HOLD_NONE
    lane.servo_quarantine_remaining = 0
    _dac_reset_total_state_locked(lane)


def _dac_set_servo_mode(mode: str) -> Tuple[str, str]:
    global _dac_servo_mode
    normalized = str(mode or "").strip().upper()
    if normalized not in {"OFF", "TOTAL", "CAMP", "10-MIN"}:
        raise ValueError("SERVOS mode must be OFF, TOTAL, CAMP, or 10-MIN")
    with _dac_lock:
        previous = _dac_servo_mode
        if previous != normalized:
            for lane in _dac_lanes.values():
                _dac_reset_servo_predictor_locked(lane)
            _dac_servo_mode = normalized
        if normalized == "OFF":
            for lane in _dac_lanes.values():
                _dac_clear_pending_locked(lane)
                lane.servo_last_step = 0.0
    _dac_control_wakeup.set()
    return previous, normalized


def _servo_hold_active_locked(lane: _DacLaneState) -> bool:
    if lane.servo_quarantine_remaining:
        reason = lane.servo_quarantine_reason or SERVO_HOLD_SETTLE_QUARANTINE
        lane.servo_quarantine_remaining -= 1
        lane.servo_quarantine_consumed_count += 1
        lane.servo_hold_reason = reason
        lane.servo_hold_count += 1
        if reason == SERVO_HOLD_COMMIT_FAULT_BACKOFF:
            lane.servo_commit_fault_hold_count += 1
        return True
    lane.servo_hold_reason = SERVO_HOLD_NONE
    lane.servo_quarantine_reason = SERVO_HOLD_NONE
    return False


def _servo_soft_landing_scale(abs_ppb: float) -> float:
    if abs_ppb >= SERVO_SOFT_LANDING_PPB:
        return 1.0
    return 0.25 + 0.75 * (abs_ppb / SERVO_SOFT_LANDING_PPB)


def _servo_apply_step(lane: _DacLaneState, step: float) -> None:
    with _dac_lock:
        before = lane.target_code
        target = _dac_clamp_target(before + step)
        planned_step = target - before
        target_hw_code = _dac_rounded_hw_code(target)
        if abs(planned_step) < 0.000001:
            lane.pacing_skip_small_delta_count += 1
            lane.servo_last_step = 0.0
            lane.servo_hold_reason = SERVO_HOLD_SMALL_STATIC_DELTA
            return
        if not _dac_dither_operator_enabled and target_hw_code == lane.hw_code:
            lane.pacing_skip_small_delta_count += 1
            lane.servo_last_step = 0.0
            lane.servo_hold_reason = SERVO_HOLD_SMALL_STATIC_DELTA
            return
    _dac_queue_servo_target(lane, target, planned_step)


def _servo_slope(
    lane: _DacLaneState,
    *,
    selected_valid: bool,
    selected_ppb: float,
    filter_alpha: float,
    gain: float,
    deadband_ppb: float,
    use_soft_landing: bool,
) -> None:
    if not selected_valid:
        return
    with _dac_lock:
        if _servo_hold_active_locked(lane):
            return
        selected = float(selected_ppb)
        error_ppb = selected - SERVO_TARGET_PPB
        lane.servo_last_residual = selected
        if not lane.servo_predictor_initialized:
            lane.servo_predictor_initialized = True
            lane.servo_last_raw_residual = error_ppb
            lane.servo_filtered_residual = error_ppb
            lane.servo_filtered_slope = 0.0
            lane.servo_predicted_residual = error_ppb
            lane.servo_predictor_updates = 1
        else:
            raw_delta = error_ppb - lane.servo_last_raw_residual
            lane.servo_last_raw_residual = error_ppb
            lane.servo_filtered_residual = (
                (1.0 - filter_alpha) * lane.servo_filtered_residual
                + filter_alpha * error_ppb
            )
            lane.servo_filtered_slope = (
                (1.0 - filter_alpha) * lane.servo_filtered_slope
                + filter_alpha * raw_delta
            )
            lane.servo_predicted_residual = lane.servo_filtered_residual
            lane.servo_predictor_updates += 1
        control_ppb = lane.servo_predicted_residual

    abs_ppb = abs(control_ppb)
    if abs_ppb < deadband_ppb:
        return
    landing = _servo_soft_landing_scale(abs_ppb) if use_soft_landing else 1.0
    step = -control_ppb / SERVO_PPB_PER_DAC_LSB_ESTIMATE
    step *= gain * landing
    step = max(-SERVO_MAX_STEP, min(SERVO_MAX_STEP, step))
    if abs(step) < 0.000001:
        return
    _servo_apply_step(lane, step)


def _servo_total_window_clear_locked(lane: _DacLaneState) -> None:
    lane.total_recent.clear()
    lane.total_samples_since_reset = 0
    lane.total_last_sample_control_second = 0


def _servo_total_state_reset_locked(
    lane: _DacLaneState,
    basis: int,
    population_seconds: int,
) -> None:
    lane.total_initialized = True
    lane.total_basis = int(basis)
    lane.total_last_commit_count = int(lane.pacing_commit_count)
    lane.total_last_dac_value = float(lane.target_code)
    lane.total_last_population_seconds = int(population_seconds)
    _servo_total_window_clear_locked(lane)
    lane.total_coarse_latched = False


def _servo_total_sync_operating_point_locked(
    lane: _DacLaneState,
    basis: int,
    population_seconds: int,
) -> bool:
    if not lane.total_initialized:
        _servo_total_state_reset_locked(lane, basis, population_seconds)
        return False
    changed = (
        lane.total_basis != int(basis)
        or int(population_seconds) < lane.total_last_population_seconds
        or lane.total_last_commit_count != lane.pacing_commit_count
        or abs(lane.target_code - lane.total_last_dac_value)
        > SERVO_TOTAL_DAC_CHANGE_EPSILON_LSB
    )
    if changed:
        _servo_total_state_reset_locked(lane, basis, population_seconds)
        return True
    lane.total_last_population_seconds = int(population_seconds)
    return False


def _servo_total_push_recent_locked(lane: _DacLaneState, sample_ppb: float) -> None:
    if (
        lane.total_last_sample_control_second
        and _dac_control_second != lane.total_last_sample_control_second + 1
    ):
        _servo_total_window_clear_locked(lane)
    lane.total_last_sample_control_second = int(_dac_control_second)
    lane.total_recent.append(float(sample_ppb))
    lane.total_samples_since_reset += 1


def _servo_total_recent_mean_locked(lane: _DacLaneState) -> float:
    return sum(lane.total_recent) / float(len(lane.total_recent)) if lane.total_recent else 0.0


def _servo_total_recent_stderr_locked(lane: _DacLaneState) -> float:
    n = len(lane.total_recent)
    if n < 2:
        return 0.0
    mean = _servo_total_recent_mean_locked(lane)
    variance = sum((x - mean) ** 2 for x in lane.total_recent) / float(n - 1)
    return math.sqrt(max(0.0, variance) / float(n))


def _servo_total_requested_rate(total_ppb: float, population_seconds: int, coarse: bool) -> float:
    if population_seconds <= 0:
        return SERVO_TARGET_PPB
    horizon = SERVO_TOTAL_COARSE_HORIZON_SECONDS if coarse else SERVO_TOTAL_FINE_HORIZON_SECONDS
    max_rate = SERVO_TOTAL_COARSE_MAX_RATE_PPB if coarse else SERVO_TOTAL_FINE_MAX_RATE_PPB
    position_error = float(total_ppb) - SERVO_TARGET_PPB
    correction_rate = -position_error * (float(population_seconds) / horizon)
    correction_rate = max(-max_rate, min(max_rate, correction_rate))
    return SERVO_TARGET_PPB + correction_rate


def _servo_total_crosses_target(
    total_ppb: float,
    population_seconds: int,
    recent_mean_ppb: float,
) -> bool:
    position_error = float(total_ppb) - SERVO_TARGET_PPB
    recent_rate_error = float(recent_mean_ppb) - SERVO_TARGET_PPB
    if (
        population_seconds <= 0
        or position_error == 0.0
        or recent_rate_error == 0.0
        or position_error * recent_rate_error >= 0.0
    ):
        return False
    seconds_to_target = (
        -position_error * float(population_seconds) / recent_rate_error
    )
    return (
        0.0 <= seconds_to_target
        <= SERVO_TOTAL_TARGET_CROSSING_LOOKAHEAD_SECONDS
    )


def _servo_total(
    lane: _DacLaneState,
    *,
    total_valid: bool,
    total_ppb: float,
    population_seconds: int,
    interval_valid: bool,
    interval_ppb: float,
) -> None:
    if not total_valid or not interval_valid or population_seconds <= 0:
        return

    with _dac_lock:
        operating_point_changed = _servo_total_sync_operating_point_locked(
            lane, 2, population_seconds
        )
        if lane.pacing_pending:
            lane.servo_hold_reason = SERVO_HOLD_PENDING_COMMIT
            lane.servo_hold_count += 1
            return
        if _servo_hold_active_locked(lane):
            return
        if operating_point_changed:
            return

        _servo_total_push_recent_locked(lane, interval_ppb)
        recent_mean = _servo_total_recent_mean_locked(lane)
        recent_stderr = _servo_total_recent_stderr_locked(lane)
        position_error = float(total_ppb) - SERVO_TARGET_PPB
        recent_rate_error = float(recent_mean) - SERVO_TARGET_PPB
        fine_required = abs(position_error) * (
            float(population_seconds) / SERVO_TOTAL_FINE_HORIZON_SECONDS
        )
        if lane.total_coarse_latched:
            if (
                abs(position_error) <= SERVO_TOTAL_COARSE_EXIT_PPB
                and abs(recent_rate_error) <= SERVO_TOTAL_COARSE_RATE_EXIT_PPB
                and fine_required <= SERVO_TOTAL_FINE_MAX_RATE_PPB
            ):
                lane.total_coarse_latched = False
        elif (
            abs(position_error) >= SERVO_TOTAL_COARSE_THRESHOLD_PPB
            or abs(recent_rate_error) >= SERVO_TOTAL_COARSE_RATE_ENTRY_PPB
            or fine_required > SERVO_TOTAL_FINE_MAX_RATE_PPB
        ):
            lane.total_coarse_latched = True

        coarse = lane.total_coarse_latched
        required_samples = (
            SERVO_TOTAL_COARSE_MIN_SAMPLES if coarse else SERVO_TOTAL_FINE_MIN_SAMPLES
        )
        lane.servo_last_residual = float(total_ppb)
        lane.servo_last_raw_residual = float(interval_ppb)
        lane.servo_filtered_residual = float(recent_mean)
        lane.servo_predictor_initialized = True
        lane.servo_predictor_updates = int(lane.total_samples_since_reset)
        lane.servo_settle_count = len(lane.total_recent)
        if len(lane.total_recent) < required_samples:
            lane.servo_filtered_slope = 0.0
            lane.servo_predicted_residual = 0.0
            return

        requested_rate = _servo_total_requested_rate(total_ppb, population_seconds, coarse)
        if _servo_total_crosses_target(total_ppb, population_seconds, recent_mean):
            requested_rate = SERVO_TARGET_PPB
        rate_error = recent_mean - requested_rate
        stderr_multiplier = (
            SERVO_TOTAL_COARSE_STDERR_MULTIPLIER
            if coarse
            else SERVO_TOTAL_FINE_STDERR_MULTIPLIER
        )
        floor_deadband = (
            SERVO_TOTAL_COARSE_RATE_DEADBAND_PPB
            if coarse
            else SERVO_TOTAL_FINE_RATE_DEADBAND_PPB
        )
        rate_deadband = max(floor_deadband, stderr_multiplier * recent_stderr)
        lane.servo_filtered_slope = float(requested_rate)
        lane.servo_predicted_residual = float(rate_error)
        near_target_hold = max(
            rate_deadband, SERVO_TOTAL_NEAR_TARGET_RATE_HOLD_PPB
        )
        if (
            (
                abs(position_error) <= SERVO_TOTAL_POSITION_HOLD_PPB
                and abs(recent_rate_error) <= near_target_hold
            )
            or abs(rate_error) <= rate_deadband
        ):
            return
        actuator_gain = (
            SERVO_TOTAL_COARSE_ACTUATOR_GAIN if coarse else SERVO_TOTAL_FINE_ACTUATOR_GAIN
        )
        max_step = SERVO_TOTAL_COARSE_MAX_STEP_LSB if coarse else SERVO_TOTAL_FINE_MAX_STEP_LSB

    step = -rate_error / SERVO_PPB_PER_DAC_LSB_ESTIMATE * actuator_gain
    step = max(-max_step, min(max_step, step))
    if abs(step) >= 0.000001:
        _servo_apply_step(lane, step)


def _dac_servo_inputs(
    clocks: Dict[str, Any],
    campaign: Optional[Dict[str, Any]],
    lane_name: str,
) -> Dict[str, Any]:
    stats = clocks.get("stats") if isinstance(clocks, dict) else None
    raw_cycles = clocks.get("raw_cycles") if isinstance(clocks, dict) else None
    stats = stats if isinstance(stats, dict) else {}
    raw_cycles = raw_cycles if isinstance(raw_cycles, dict) else {}
    lane_stats = stats.get(lane_name)
    lane_stats = lane_stats if isinstance(lane_stats, dict) else {}
    buckets = lane_stats.get("ppb_buckets")
    buckets = buckets if isinstance(buckets, dict) else {}
    tau_state = stats.get(f"{lane_name}_tau_state")
    tau_state = tau_state if isinstance(tau_state, dict) else {}
    campaign = campaign if isinstance(campaign, dict) else {}
    campaign_stats = campaign.get("stats")
    campaign_stats = campaign_stats if isinstance(campaign_stats, dict) else {}
    campaign_ppb = campaign_stats.get("ppb")
    campaign_ppb = campaign_ppb if isinstance(campaign_ppb, dict) else {}
    campaign_started = str(campaign.get("state") or "").strip().upper() == "STARTED"
    reference = raw_cycles.get("vclock")
    observed = raw_cycles.get(lane_name)
    reference = reference if isinstance(reference, dict) else {}
    observed = observed if isinstance(observed, dict) else {}

    try:
        reference_cycles = int(reference.get("observed_cycles") or 0)
        clock_cycles = int(observed.get("observed_cycles") or 0)
    except (TypeError, ValueError):
        reference_cycles = 0
        clock_cycles = 0
    interval_valid = bool(
        reference.get("valid", reference_cycles > 0)
        and observed.get("valid", clock_cycles > 0)
        and reference_cycles > 0
        and clock_cycles > 0
    )
    interval_ppb = (
        (float(reference_cycles - clock_cycles) * 1.0e9) / float(reference_cycles)
        if interval_valid
        else 0.0
    )
    try:
        ten_min_ppb = float(buckets.get("10_min"))
        ten_min_finite = math.isfinite(ten_min_ppb)
    except (TypeError, ValueError):
        ten_min_ppb = 0.0
        ten_min_finite = False
    try:
        total_ppb = float(buckets.get("total"))
        total_finite = math.isfinite(total_ppb)
    except (TypeError, ValueError):
        total_ppb = 0.0
        total_finite = False
    try:
        camp_ppb = float(campaign_ppb.get(lane_name))
        camp_finite = math.isfinite(camp_ppb)
    except (TypeError, ValueError):
        camp_ppb = 0.0
        camp_finite = False
    try:
        population = int(tau_state.get("sample_count") or 0)
    except (TypeError, ValueError):
        population = 0

    return {
        "interval_valid": interval_valid,
        "interval_ppb": interval_ppb,
        "10_min_valid": interval_valid and ten_min_finite,
        "10_min_ppb": ten_min_ppb,
        "camp_valid": interval_valid and campaign_started and camp_finite,
        "camp_ppb": camp_ppb,
        "total_valid": (
            interval_valid and total_finite and population >= SERVO_TOTAL_MIN_SAMPLES
        ),
        "total_ppb": total_ppb,
        "total_population_seconds": population,
    }


def _dac_process_completed_row(
    teensy_clocks: Dict[str, Any],
    campaign: Optional[Dict[str, Any]],
    sequence: int,
) -> None:
    global _dac_control_second
    global _dac_stats_reset_count, _dac_stats_update_count
    global _dac_stats_last_sequence, _dac_stats_reset_fence_count

    stats = teensy_clocks.get("stats") if isinstance(teensy_clocks, dict) else None
    if not isinstance(stats, dict):
        return

    try:
        reset_count = int(stats["reset_count"])
        update_count = int(stats["update_count"])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(
            f"DAC statistics row lacks valid Teensy statistics chronology: {exc}"
        ) from exc
    if reset_count < 0 or update_count < 0:
        raise RuntimeError(
            "DAC statistics row has negative Teensy statistics chronology: "
            f"reset_count={reset_count} update_count={update_count}"
        )

    endpoint_admitted = bool(stats.get("rolling_ppb_endpoint_admitted"))

    with _dac_stats_epoch_lock:
        previous_reset = _dac_stats_reset_count
        previous_update = _dac_stats_update_count
        previous_sequence = _dac_stats_last_sequence

        # An explicit transitive STATS_RESET fences queued rows from the old
        # Teensy epoch.  Those rows remain valid historical testimony elsewhere,
        # but they must never repopulate the freshly reset Pi DAC Welfords.
        if _dac_stats_reset_fence_count is not None:
            if reset_count == _dac_stats_reset_fence_count:
                return
            _dac_reset_statistics()
            _dac_stats_reset_fence_count = None
            previous_reset = None
            previous_update = None
            previous_sequence = None

        epoch_replaced = bool(
            previous_reset is not None
            and previous_update is not None
            and (
                reset_count != previous_reset
                or update_count < previous_update
            )
        )
        if epoch_replaced:
            reset = _dac_reset_statistics()
            logging.warning(
                "📊 [clocks/dac] Teensy statistics epoch replaced; resetting Pi DAC "
                "Welfords before first row of new epoch: reset_count %s -> %s, "
                "update_count %s -> %s, sequence=%s, DAC N %s/%s -> 0/0",
                previous_reset,
                reset_count,
                previous_update,
                update_count,
                sequence,
                _path_get(reset, "before.ocxo1_dac.n"),
                _path_get(reset, "before.ocxo2_dac.n"),
            )

        if (
            previous_reset is not None
            and previous_update is not None
            and not epoch_replaced
        ):
            # reset_count/update_count is the exactly-once statistical identity.
            # Physical CLOCKS_FRAGMENT.sequence is boot-local and may lawfully
            # rebase across a Teensy reboot while restored statistics continue N+1.
            if update_count == previous_update:
                if previous_sequence != int(sequence):
                    raise RuntimeError(
                        "Teensy statistics chronology repeated on a different physical row: "
                        f"reset_count={reset_count} update_count={update_count} "
                        f"previous_sequence={previous_sequence} sequence={sequence}"
                    )
                return
            if previous_sequence == int(sequence):
                raise RuntimeError(
                    "Teensy statistics chronology advanced on the same physical row: "
                    f"reset_count={reset_count} previous_update_count={previous_update} "
                    f"update_count={update_count} sequence={sequence}"
                )

        _dac_stats_reset_count = int(reset_count)
        _dac_stats_update_count = int(update_count)
        _dac_stats_last_sequence = int(sequence)

        if not endpoint_admitted:
            return

        # Statistical chronology above has already proved this is a new admitted
        # observation.  Do not gate population custody on physical sequence: that
        # identity restarts at a Teensy reboot while restored update_count continues.
        with _dac_lock:
            _dac_control_second = int(update_count)
            for lane in _dac_lanes.values():
                lane.welford.update(lane.target_code)
            mode = _dac_servo_mode

    if mode == "OFF":
        return

    for lane_name, lane in _dac_lanes.items():
        inputs = _dac_servo_inputs(teensy_clocks, campaign, lane_name)
        if mode == "CAMP":
            if not inputs["camp_valid"]:
                continue
            with _dac_lock:
                lane.servo_settle_count += 1
                ready = lane.servo_settle_count >= SERVO_CAMP_SETTLE_SECONDS
                if ready:
                    lane.servo_settle_count = 0
            if ready:
                _servo_slope(
                    lane,
                    selected_valid=True,
                    selected_ppb=float(inputs["camp_ppb"]),
                    filter_alpha=SERVO_CAMP_FILTER_ALPHA,
                    gain=SERVO_CAMP_GAIN,
                    deadband_ppb=SERVO_CAMP_DEADBAND_PPB,
                    use_soft_landing=True,
                )
        elif mode == "TOTAL":
            _servo_total(
                lane,
                total_valid=bool(inputs["total_valid"]),
                total_ppb=float(inputs["total_ppb"]),
                population_seconds=int(inputs["total_population_seconds"]),
                interval_valid=bool(inputs["interval_valid"]),
                interval_ppb=float(inputs["interval_ppb"]),
            )
        elif mode == "10-MIN":
            with _dac_lock:
                lane.servo_settle_count = 0
            _servo_slope(
                lane,
                selected_valid=bool(inputs["10_min_valid"]),
                selected_ppb=float(inputs["10_min_ppb"]),
                filter_alpha=SERVO_10_MIN_FILTER_ALPHA,
                gain=SERVO_10_MIN_GAIN,
                deadband_ppb=SERVO_10_MIN_DEADBAND_PPB,
                use_soft_landing=False,
            )


def _dac_control_lane_snapshot(lane: _DacLaneState) -> Dict[str, Any]:
    return {
        "target_code": round(float(lane.target_code), 6),
        "hw_code": int(lane.hw_code),
        "readback_valid": bool(lane.readback_valid),
        "readback_code": int(lane.readback_code),
        "servo": {
            "last_step": float(lane.servo_last_step),
            "last_residual": float(lane.servo_last_residual),
            "settle_count": int(lane.servo_settle_count),
            "adjustments": int(lane.servo_adjustments),
            "predictor_initialized": bool(lane.servo_predictor_initialized),
            "last_raw_residual": float(lane.servo_last_raw_residual),
            "filtered_residual": float(lane.servo_filtered_residual),
            "filtered_slope": float(lane.servo_filtered_slope),
            "predicted_residual": float(lane.servo_predicted_residual),
            "predictor_updates": int(lane.servo_predictor_updates),
        },
    }


def _dac_control_snapshot() -> Dict[str, Any]:
    with _dac_lock:
        return {
            "schema": "CLOCKS_CONTROL_V1",
            "servo_mode": _dac_servo_mode,
            "servo_target_ppb": float(SERVO_TARGET_PPB),
            "servo_active": _dac_servo_mode != "OFF",
            "realization_mode": (
                "ONE_SECOND_FRACTIONAL_DITHER"
                if _dac_dither_operator_enabled
                else "STATIC_ROUNDED"
            ),
            "dither_operator_enabled": bool(_dac_dither_operator_enabled),
            "ocxo1": _dac_control_lane_snapshot(_dac_lanes["ocxo1"]),
            "ocxo2": _dac_control_lane_snapshot(_dac_lanes["ocxo2"]),
        }


def _dac_welford_snapshots() -> Dict[str, Dict[str, Any]]:
    with _dac_lock:
        return {
            "ocxo1_dac": _dac_lanes["ocxo1"].welford.snapshot(),
            "ocxo2_dac": _dac_lanes["ocxo2"].welford.snapshot(),
        }


def _dac_reset_statistics() -> Dict[str, Any]:
    with _dac_lock:
        before = _dac_welford_snapshots()
        for lane in _dac_lanes.values():
            lane.welford.reset()
        after = _dac_welford_snapshots()
    return {"before": before, "after": after}


def _dac_restore_control_from_clocks(clocks: Dict[str, Any], *, realize: bool) -> Dict[str, Any]:
    control = clocks.get("control") if isinstance(clocks, dict) else None
    stats = clocks.get("stats") if isinstance(clocks, dict) else None
    if not isinstance(control, dict) or control.get("schema") != "CLOCKS_CONTROL_V1":
        raise ValueError("canonical CLOCKS restore missing Pi DAC control state")
    stats = stats if isinstance(stats, dict) else {}
    auxiliary = stats.get("auxiliary_welford")
    auxiliary = auxiliary if isinstance(auxiliary, dict) else {}
    mode = str(control.get("servo_mode") or "OFF").upper()
    if mode not in {"OFF", "TOTAL", "CAMP", "10-MIN"}:
        raise ValueError(f"invalid restored servo mode {mode!r}")
    dither = bool(control.get("dither_operator_enabled"))

    restore_reset_count = _as_int(stats.get("reset_count"))
    restore_update_count = _as_int(stats.get("update_count"))
    restore_sequence = _as_int(clocks.get("completed_pps_sequence"))
    if (
        restore_reset_count is None
        or restore_reset_count < 0
        or restore_update_count is None
        or restore_update_count < 0
        or restore_sequence is None
        or restore_sequence <= 0
    ):
        raise ValueError("canonical CLOCKS restore missing DAC statistics epoch identity")

    with _dac_stats_epoch_lock:
        with _dac_lock:
            global _dac_servo_mode, _dac_dither_operator_enabled, _dac_next_frame_monotonic
            global _dac_stats_reset_count, _dac_stats_update_count
            global _dac_stats_last_sequence, _dac_stats_reset_fence_count
            _dac_servo_mode = mode
            _dac_dither_operator_enabled = dither
            _dac_next_frame_monotonic = time.monotonic() if dither else 0.0
            for lane_name, lane in _dac_lanes.items():
                saved = control.get(lane_name)
                saved = saved if isinstance(saved, dict) else {}
                servo = saved.get("servo")
                servo = servo if isinstance(servo, dict) else {}
                lane.target_code = _dac_clamp_target(float(saved.get("target_code") or 0.0))
                lane.servo_last_step = float(servo.get("last_step") or 0.0)
                lane.servo_last_residual = float(servo.get("last_residual") or 0.0)
                lane.servo_settle_count = int(servo.get("settle_count") or 0)
                lane.servo_adjustments = int(servo.get("adjustments") or 0)
                lane.servo_predictor_initialized = bool(servo.get("predictor_initialized"))
                lane.servo_last_raw_residual = float(servo.get("last_raw_residual") or 0.0)
                lane.servo_filtered_residual = float(servo.get("filtered_residual") or 0.0)
                lane.servo_filtered_slope = float(servo.get("filtered_slope") or 0.0)
                lane.servo_predicted_residual = float(servo.get("predicted_residual") or 0.0)
                lane.servo_predictor_updates = int(servo.get("predictor_updates") or 0)
                lane.dither_enabled = dither
                lane.dither_transition_due = None
                _dac_clear_pending_locked(lane)
                _dac_reset_total_state_locked(lane)
                lane.welford.restore(auxiliary.get(f"{lane_name}_dac"))

            # The restored DAC Welford belongs to this exact canonical Teensy
            # statistics epoch.  Discard any startup/fresh-boot chronology that
            # preceded the restore so the next N+1 row continues this ancestry.
            _dac_stats_reset_count = int(restore_reset_count)
            _dac_stats_update_count = int(restore_update_count)
            _dac_stats_last_sequence = int(restore_sequence)
            _dac_stats_reset_fence_count = None

    if realize:
        if dither:
            _dac_control_wakeup.set()
        else:
            for lane in _dac_lanes.values():
                if not _dac_write_hw_code(
                    lane,
                    _dac_rounded_hw_code(lane.target_code),
                    author_source=DAC_AUTHOR_EXPLICIT_COMMAND,
                    latch_fault=True,
                ):
                    raise RuntimeError(f"failed to restore {lane.name} DAC hardware")
    else:
        _dac_control_wakeup.set()

    return {
        "restored": True,
        "servo_mode": mode,
        "dither_operator_enabled": dither,
        "realized": bool(realize),
        "control": _dac_control_snapshot(),
    }


def _dac_info_payload() -> Dict[str, Any]:
    with _dac_lock:
        lanes: Dict[str, Any] = {}
        for name, lane in _dac_lanes.items():
            lanes[name] = {
                "address": f"0x{lane.address:02X}",
                "readback_valid": bool(lane.readback_valid),
                "readback_code": int(lane.readback_code),
                "reset_signature": bool(lane.reset_signature),
                "desired": float(lane.target_code),
                "hw_code": int(lane.hw_code),
                "safe_max_hw_code": int(AD5693R_SAFE_MAX_HW_CODE),
                "request_pending": bool(lane.pacing_pending),
                "pending_target": float(lane.pacing_pending_target),
                "last_author_source": int(lane.last_author_source),
                "readback_count": int(lane.readback_count),
                "readback_failures": int(lane.readback_failures),
                "adopt_count": int(lane.adopt_count),
                "same_code_skip_count": int(lane.same_code_skip_count),
                "write_attempts": int(lane.write_attempts),
                "write_successes": int(lane.write_successes),
                "write_failures": int(lane.write_failures),
                "dither_low_code": int(lane.dither_low_code),
                "dither_high_code": int(lane.dither_high_code),
                "dither_high_ms": int(lane.dither_high_ms),
                "dither_frame_count": int(lane.dither_frame_count),
                "dither_transition_count": int(lane.dither_transition_count),
            }
        return {
            "schema": "CLOCKS_DAC_CUSTODY_V1",
            "status": "ok",
            "owner": "PI.CLOCKS",
            "i2c_bus": DAC_I2C_BUS,
            "servo_mode": _dac_servo_mode,
            "servo_target_ppb": float(SERVO_TARGET_PPB),
            "servo_active": _dac_servo_mode != "OFF",
            "dither_operator_enabled": bool(_dac_dither_operator_enabled),
            "realization_mode": (
                "ONE_SECOND_FRACTIONAL_DITHER"
                if _dac_dither_operator_enabled
                else "STATIC_ROUNDED"
            ),
            "initialization_authored_output": False,
            **lanes,
        }


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

OPERATIONAL_STATE_SCHEMA = "PI_SUBSYSTEM_OPERATIONAL_STATE_V1"
OPERATIONAL_STATE_STARTING = "STARTING"
OPERATIONAL_STATE_RECOVERING = "RECOVERING"
OPERATIONAL_STATE_RUNNING = "RUNNING"
OPERATIONAL_STATE_HARD_FAILURE = "HARD_FAILURE"

_operational_state_lock = threading.Lock()
_operational_state: Dict[str, Any] = {
    "schema": OPERATIONAL_STATE_SCHEMA,
    "subsystem": "CLOCKS",
    "state": OPERATIONAL_STATE_STARTING,
    "entered_at_utc": None,
    "reason": "process_initialization",
    "source": "RUN",
    "details": {},
}
_hard_failure_event = threading.Event()
_hard_failure_lock = threading.Lock()
# HARD_FAILURE normally closes the entire scientific data plane.  One operator REPAIR transaction may temporarily reopen enough CLOCKS ingress and
# persistence to establish row 1 of a replacement statistics epoch.  REPAIR owns
# the authority to clear HARD_FAILURE after it proves a usable replacement lineage.
_hard_failure_stats_repair_event = threading.Event()
_hard_failure_stats_repair_lock = threading.Lock()
# REPAIR request ownership is separate from the narrow HARD_FAILURE data-plane
# override above.  A REPAIR command may arrive while boot reconciliation is still
# legitimately mutating startup custody.  In that case the request waits for the
# startup transaction to finish before it is allowed to reopen HARD_FAILURE ingress.
_repair_request_in_progress = threading.Event()
_startup_reconciliation_active = threading.Event()
_DESTRUCTIVE_REPAIR_REASONS = {
    "dac_restore_population_ancestry_impossible",
    "dac_recovery_boundary_population_ancestry_impossible",
    # Exact Alpha resurrection loss is handled automatically as a lawful lineage
    # surrender during lifecycle reconciliation; it is not a subsystem failure.
}

_campaign_active: bool = False

# Dead-producer recovery may resume on a truthful degraded timeline row while the
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
# Fresh-instrument startup opens the persistence boundary before STATS_RESET,
# then admits only the first row of the new Alpha statistics epoch.  This
# prevents both an undurable epoch prefix and pre-reset startup rows from
# becoming future holistic-restore authority.
_clocks_epoch_birth_pending = threading.Event()
_clocks_epoch_birth_committed = threading.Event()
_clocks_epoch_birth_prior_reset_count = -1
_clocks_epoch_birth_reset_count = -1

# A cold holistic restore must not use an ephemeral CLOCKS row as its proof of
# convergence.  While ordinary startup persistence remains closed, this narrow
# custody lane admits the exact first restored statistics row (N+1) and every
# following row in that restored epoch until general persistence opens.
_clocks_holistic_restore_proof_pending = threading.Event()
_clocks_holistic_restore_proof_committed = threading.Event()
_clocks_holistic_restore_proof_expected: Dict[str, Any] = {}
_clocks_holistic_restore_proof_reset_count = -1
_clocks_holistic_restore_proof_update_count = -1
_clocks_holistic_restore_proof_sequence: Optional[int] = None
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

# Pi-owned Better-Buckets resurrection image. The live object is guarded because
# startup reconciliation/recovery threads may reseed it while the CLOCKS state
# worker is ingesting producer-authored deltas. It contains only firmware facts
# and Pi custody metadata; no bucket value is authored here.
_ppb_checkpoint_lock = threading.RLock()
_ppb_checkpoint_runtime: Optional[Dict[str, Any]] = None

# While Pi CLOCKS is explicitly staging PPB_RESTORE_* into Alpha, firmware
# truthfully withholds a coherent rolling_ppb_checkpoint snapshot. The ingress
# handler tags rows received inside that transaction so the state worker can
# retire only those known transitional rows even if queue latency carries them
# past PPB_RESTORE_COMMIT. Outside this owned transaction, checkpoint valid=false
# remains an ordinary validation failure.
_ppb_restore_transaction_active = threading.Event()

# Ambient/no-campaign Alpha resurrection. ``active`` owns the recovery lifecycle;
# ``hold`` is the narrow state-worker gate that retires newborn-Alpha rows until
# the exact durable N+1 proof candidate appears. Pi-owned GNSS_RAW/control stay
# live; only Teensy-owned instrument/statistical custody is resurrected.
_ambient_instrument_recovery_active = threading.Event()
_ambient_instrument_recovery_hold = threading.Event()

# Cold/full startup Alpha resurrection has the same pre-mutation requirement as
# ambient resurrection: once newborn Alpha has been disproved, no row may touch
# Pi GNSS_RAW/DAC/Better-Buckets custody until the exact restored N+1 row arrives.
# The gate lock closes the race between the state worker's hold check and its
# subsequent mutations while the startup thread arms this hold.
_startup_instrument_restore_hold = threading.Event()
_clocks_state_mutation_gate_lock = threading.Lock()

# A physical CLOCKS_FRAGMENT sequence regression during unresolved startup proves
# that PUBSUB custody has crossed a Teensy boot boundary before holistic startup
# has classified the producer.  Keep the newborn stream visible to preflight and
# the lifecycle court, but do not let it mutate Alpha-derived Pi custody
# (DAC statistics / Better-Buckets) until that court resolves the lifetime.
_startup_physical_lifetime_unclassified = threading.Event()

# Startup persistence custody.  Valid CLOCKS rows may arrive while holistic
# restore deliberately keeps the ordinary writer closed.  Retain those exact
# canonical rows until the Teensy lifecycle probe classifies their relationship
# to Alpha.  A proved live continuation may release them normally; surviving-Alpha
# rows observed before Pi-owned state restoration are durably quarantined rather
# than discarded; cold/full-restore boundaries may still retire superseded epochs.
_startup_custody_lock = threading.Lock()
_startup_custody_active = True
_startup_custody_backlog = deque()

# Live recovery has its own durable custody boundary. Unlike startup custody,
# recovery candidates must still flow through canonical persistence before the
# TEMPEST processor can adjudicate them. Therefore these rows are persisted with
# an explicit restore-authority quarantine rather than held only in RAM. A Pi
# restart during RECOVER can then see the evidence without ever mistaking it for
# Alpha restore authority.
_recovery_custody_lock = threading.Lock()
_recovery_custody_active = False
_recovery_custody_generation: Optional[str] = None
_recovery_custody_entered_at_utc: Optional[str] = None
_recovery_custody_reason: Optional[str] = None
_recovery_custody_physical_sequence_regression = False
_recovery_custody_regression_witness: Dict[str, Any] = {}
# Exact private Better-Buckets image belonging to the newest CLOCKS row that has
# actually crossed the recovery-custody routing boundary. Finalization uses this
# rather than the mutable live runtime, which may already contain one later row
# blocked behind _recovery_custody_lock.
_recovery_custody_last_checkpoint: Optional[Dict[str, Any]] = None

# The command server is exposed early so PUBSUB can discover subscriptions, but
# START/RESUME must not race holistic startup reconciliation. A retained
# WATCHDOG_ANOMALY may also arrive as soon as PUBSUB reconnects; startup owns
# that evidence until holistic reconciliation has classified the Teensy.
_startup_control_ready = threading.Event()
_startup_watchdog_lock = threading.Lock()
_startup_watchdog_deferred = deque()

# Latest unified operational heartbeat.  CLOCKS consumes CLOCKS.features for
# campaign preflight; it never polls or subscribes to a feature-only side feed.
_clocks_lock = threading.Lock()
_latest_clocks: Dict[str, Any] = {}
# Private recovery custody paired with _latest_clocks.  Canonical CLOCKS no longer
# transports the literal Better-Buckets restore image, but restore/proof courts
# still need the exact checkpoint authored for that same observation.
_latest_clocks_ppb_restore_checkpoint: Optional[Dict[str, Any]] = None
_latest_clocks_received_monotonic: Optional[float] = None
_latest_clocks_received_utc: Optional[str] = None
# A startup survival witness must have entered CLOCKS after the current Teensy
# transport was admitted.  PUBSUB may replay an old producer tail into the new
# process before that point; state-worker processing time must never freshen it.
_startup_survival_ingress_barrier_monotonic: Optional[float] = None

_last_pps_vclock_count_seen: Optional[int] = None

# Last PPS/VCLOCK count accepted into TIMEBASE processing.
# This is observational only; it is never used to reject a fragment.
_accepted_pps_vclock_count: Optional[int] = None

# CLOCKS_FRAGMENT silence monitor. During an active campaign it remains the early
# transport-loss trigger. Independently, the state worker treats any proved physical
# sequence rebase with no active campaign as an instrument-only Alpha resurrection
# boundary, so always-on CLOCKS custody does not depend on TEMPEST lifecycle.
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


class HardFailureRequired(RuntimeError):
    """A proved condition for which CLOCKS must remain alive but stop authoring truth."""

    def __init__(self, reason: str, details: Dict[str, Any]):
        super().__init__(f"{reason}: {details}")
        self.reason = str(reason)
        self.details = copy.deepcopy(details)


class AlphaResurrectionImpossible(RuntimeError):
    """Exact old-Alpha continuity is dead; establish a new lineage instead."""

    def __init__(self, reason: str, details: Dict[str, Any]):
        super().__init__(f"{reason}: {details}")
        self.reason = str(reason)
        self.details = copy.deepcopy(details)


def _operational_state_snapshot() -> Dict[str, Any]:
    with _operational_state_lock:
        return copy.deepcopy(_operational_state)


def _hard_failure_active() -> bool:
    return _hard_failure_event.is_set()


def _hard_failure_stats_repair_active() -> bool:
    """True only while the explicit DAC-ancestry statistics repair owns the data plane."""
    return _hard_failure_stats_repair_event.is_set()


def _set_operational_state(
    state: str,
    *,
    reason: Optional[str] = None,
    source: Optional[str] = None,
    details: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Publish one process-local CLOCKS lifecycle state; HARD_FAILURE is latched."""
    global _operational_state

    normalized = str(state or "").strip().upper()
    if normalized not in {
        OPERATIONAL_STATE_STARTING,
        OPERATIONAL_STATE_RECOVERING,
        OPERATIONAL_STATE_RUNNING,
        OPERATIONAL_STATE_HARD_FAILURE,
    }:
        raise ValueError(f"unsupported CLOCKS operational state {state!r}")

    with _operational_state_lock:
        if (
            str(_operational_state.get("state") or "") == OPERATIONAL_STATE_HARD_FAILURE
            and normalized != OPERATIONAL_STATE_HARD_FAILURE
        ):
            return copy.deepcopy(_operational_state)
        _operational_state = {
            "schema": OPERATIONAL_STATE_SCHEMA,
            "subsystem": "CLOCKS",
            "state": normalized,
            "entered_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
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
    """Latch CLOCKS inert without exiting so systemd cannot erase the crime scene."""
    global _campaign_active

    with _hard_failure_lock:
        if _hard_failure_active():
            return _operational_state_snapshot()

        context = {
            "pid": os.getpid(),
            "campaign_active_process_local": bool(_campaign_active),
            "last_clocks_sequence": _last_clocks_state_sequence,
            "last_pps_vclock_count_seen": _last_pps_vclock_count_seen,
            "accepted_pps_vclock_count": _accepted_pps_vclock_count,
            "dac_stats_reset_count": _dac_stats_reset_count,
            "dac_stats_update_count": _dac_stats_update_count,
            "dac_stats_last_sequence": _dac_stats_last_sequence,
            "queues": {
                "clocks_state": _clocks_state_queue.qsize(),
                "clocks_persistence": _clocks_persist_queue.qsize(),
                "tempest": _fragment_queue.qsize(),
            },
            # Diagnostic snapshots are intentionally lock-free: HARD_FAILURE may be
            # entered while one of these custody locks is already held.
            "startup_custody_active": bool(_startup_custody_active),
            "startup_custody_depth": len(_startup_custody_backlog),
            "recovery_custody": {
                "active": bool(_recovery_custody_active),
                "generation": _recovery_custody_generation,
                "reason": _recovery_custody_reason,
                "physical_sequence_regression": bool(
                    _recovery_custody_physical_sequence_regression
                ),
                "regression_witness": copy.deepcopy(
                    _recovery_custody_regression_witness
                ),
            },
        }
        failure_details = {
            "failure": copy.deepcopy(details or {}),
            "context": context,
            "action": (
                "CLOCKS is latched in HARD_FAILURE. No new CLOCKS/TEMPEST testimony, "
                "automatic recovery, or DAC authorship will occur. Read-only reports remain available."
            ),
        }

        _hard_failure_event.set()
        _campaign_active = False
        _startup_control_ready.clear()
        _clocks_persistence_enabled.clear()
        _diag["startup_control_ready"] = False
        _diag["hard_failure_entries"] = _diag.get("hard_failure_entries", 0) + 1
        _dac_control_wakeup.set()

        snapshot = _set_operational_state(
            OPERATIONAL_STATE_HARD_FAILURE,
            reason=reason,
            source=source,
            details=failure_details,
        )
        _diag["last_hard_failure_hold"] = copy.deepcopy(snapshot)
        logging.critical(
            "🛑 [clocks] HARD_FAILURE LATCHED: reason=%s source=%s; "
            "last CLOCKS sequence=%s, queues clocks=%d persistence=%d TEMPEST=%d. "
            "Scientific authorship and DAC control are stopped; read-only reports remain available.",
            reason,
            source,
            context.get("last_clocks_sequence"),
            int(context["queues"]["clocks_state"]),
            int(context["queues"]["clocks_persistence"]),
            int(context["queues"]["tempest"]),
        )
        return snapshot


def _require_hard_failure(
    reason: str,
    details: Dict[str, Any],
    *,
    source: str,
) -> None:
    _enter_hard_failure(reason, details, source=source)
    raise HardFailureRequired(reason, details)


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


def _flash_cut_pre_cut_tail_authority(
    *,
    firmware_campaign: str,
    active_campaign: str,
    active_campaign_payload: Dict[str, Any],
) -> Optional[str]:
    """Return the authority proving a lawful predecessor tail, or None.

    The active campaign_master transition is the atomic Flash Cut authority.
    Process-local wait state is armed only after that DB transaction commits, so
    it is a useful witness but cannot be the sole court during the cutover race.
    """
    if _campaign_payload_is_pending_flash_cut(active_campaign_payload):
        durable_from = str(active_campaign_payload.get("flash_cut_from") or "")
        if (
            durable_from
            and firmware_campaign == durable_from
            and active_campaign != durable_from
        ):
            return "CAMPAIGN_MASTER"

    if (
        _flash_cut_pending
        and _flash_cut_from_campaign
        and _flash_cut_to_campaign
        and firmware_campaign == _flash_cut_from_campaign
        and active_campaign == _flash_cut_to_campaign
    ):
        return "PROCESS_LOCAL"

    return None


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

def _recovery_custody_snapshot_locked() -> Dict[str, Any]:
    return {
        "active": bool(_recovery_custody_active),
        "generation": _recovery_custody_generation,
        "entered_at_utc": _recovery_custody_entered_at_utc,
        "reason": _recovery_custody_reason,
        "physical_sequence_regression": bool(
            _recovery_custody_physical_sequence_regression
        ),
        "regression_witness": copy.deepcopy(_recovery_custody_regression_witness),
    }


def _recovery_custody_snapshot() -> Dict[str, Any]:
    with _recovery_custody_lock:
        return _recovery_custody_snapshot_locked()


def _begin_recovery_clocks_custody(
    reason: str,
    details: Dict[str, Any],
    *,
    physical_sequence_regression: bool = False,
) -> Dict[str, Any]:
    """Quarantine live-recovery CLOCKS rows from future restore authority.

    This is the live analogue of startup custody, but rows remain durably
    persisted because canonical CLOCKS persistence is the transport into the
    TEMPEST recovery court. ``holistic_restore_superseded`` is the established
    restore exclusion authority and makes a Pi crash during RECOVER fail-safe.
    """
    global _recovery_custody_active, _recovery_custody_generation
    global _recovery_custody_entered_at_utc, _recovery_custody_reason
    global _recovery_custody_physical_sequence_regression
    global _recovery_custody_regression_witness
    global _recovery_custody_last_checkpoint

    # Boot reconciliation already owns a stricter in-memory custody boundary.
    # Do not create a second lifecycle while startup is still classifying Teensy.
    if not _startup_control_ready.is_set():
        return {
            "active": False,
            "basis": "STARTUP_CUSTODY",
            "reason": str(reason),
        }

    now_utc = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    with _recovery_custody_lock:
        newly_started = not _recovery_custody_active
        if newly_started:
            _recovery_custody_active = True
            _recovery_custody_generation = f"{os.getpid()}-{time.time_ns()}"
            _recovery_custody_entered_at_utc = now_utc
            _recovery_custody_reason = str(reason)
            _recovery_custody_physical_sequence_regression = False
            _recovery_custody_regression_witness = {}
            _recovery_custody_last_checkpoint = None

        newly_proved_physical_regression = False
        if physical_sequence_regression:
            if not _recovery_custody_physical_sequence_regression:
                _diag["recovery_custody_physical_regressions"] = (
                    _diag.get("recovery_custody_physical_regressions", 0) + 1
                )
                _recovery_custody_physical_sequence_regression = True
                _recovery_custody_regression_witness = copy.deepcopy(details)
                newly_proved_physical_regression = True

        snapshot = _recovery_custody_snapshot_locked()

    if newly_started:
        _diag["recovery_custody_begin_count"] = (
            _diag.get("recovery_custody_begin_count", 0) + 1
        )
        logging.warning(
            "🧾 [recovery/custody] opened durable restore-authority quarantine: "
            "generation=%s reason=%s",
            snapshot.get("generation"),
            reason,
        )
    elif newly_proved_physical_regression:
        logging.error(
            "🧾 [recovery/custody] a CLOCKS sequence regression proves the Teensy restarted "
            "while recovery custody was open: generation=%s previous_sequence=%s "
            "observed_sequence=%s; this lifetime requires cold producer classification",
            snapshot.get("generation"),
            details.get("previous_sequence"),
            details.get("observed_sequence"),
        )

    _diag["recovery_custody_active"] = True
    _diag["recovery_custody_generation"] = snapshot.get("generation")
    _diag["last_recovery_custody"] = {
        **snapshot,
        "last_updated_at_utc": now_utc,
        "last_reason": str(reason),
    }
    return snapshot


def _recovery_custody_requires_cold_restore() -> bool:
    with _recovery_custody_lock:
        return bool(
            _recovery_custody_active
            and _recovery_custody_physical_sequence_regression
        )


def _decorate_recovery_custody_state_locked(
    state: Dict[str, Any], checkpoint: Dict[str, Any]
) -> bool:
    """Persist one recovery-custody row and retain its exact private checkpoint."""
    global _recovery_custody_last_checkpoint
    if not _recovery_custody_active:
        return False
    if not _recovery_custody_generation:
        raise RuntimeError("active recovery custody has no generation identity")
    normalized_checkpoint = _normalize_saved_ppb_checkpoint(checkpoint)
    if not _checkpoint_matches_clocks_state(normalized_checkpoint, state):
        raise RuntimeError(
            "recovery-custody CLOCKS row and private Better-Buckets checkpoint disagree"
        )

    state["holistic_restore_superseded"] = True
    state["recovery_custody"] = {
        "schema": "PI_CLOCKS_RECOVERY_CUSTODY_V1",
        "generation": _recovery_custody_generation,
        "entered_at_utc": _recovery_custody_entered_at_utc,
        "reason": _recovery_custody_reason,
        "classification": "UNCLASSIFIED_RECOVERY",
        "restore_authority": False,
        "physical_sequence_regression": bool(
            _recovery_custody_physical_sequence_regression
        ),
        "regression_witness": copy.deepcopy(_recovery_custody_regression_witness),
    }
    _recovery_custody_last_checkpoint = copy.deepcopy(normalized_checkpoint)
    _diag["recovery_custody_rows_quarantined"] = (
        _diag.get("recovery_custody_rows_quarantined", 0) + 1
    )
    return True


def _wait_for_clocks_persistence_barrier_locked() -> None:
    """Drain every CLOCKS persistence item routed before the held custody lock."""
    completion = threading.Event()
    _clocks_persist_queue.put({"recovery_custody_barrier": completion})
    if not completion.wait(timeout=HOLISTIC_RESTORE_TIMEOUT_S):
        raise RuntimeError(
            "CLOCKS persistence did not reach recovery-custody classification barrier "
            f"within {HOLISTIC_RESTORE_TIMEOUT_S:.1f}s"
        )


def _wait_for_clocks_persistence_barrier() -> None:
    """Prove every already-routed CLOCKS row is durable before choosing restore authority."""
    completion = threading.Event()
    _clocks_persist_queue.put({"recovery_custody_barrier": completion})
    if not completion.wait(timeout=HOLISTIC_RESTORE_TIMEOUT_S):
        raise RuntimeError(
            "CLOCKS persistence did not reach ambient instrument-recovery barrier "
            f"within {HOLISTIC_RESTORE_TIMEOUT_S:.1f}s"
        )


def _finalize_recovery_clocks_custody(
    *,
    last_tb: Dict[str, Any],
    campaign_name: str,
    first_public_count: int,
    recover_mode: str,
) -> Dict[str, Any]:
    """Classify durable custody rows at the first admitted RECOVER boundary.

    LIVE_PRODUCER_ADOPT without a physical lifetime regression promotes the whole
    custody generation: firmware has attested that Alpha remained live. A cold
    bootstrap, or any observed physical sequence regression, keeps every
    pre-boundary row superseded and promotes only the recovered boundary and
    later rows. The latter path additionally requires monotonic sufficient-state
    proof against the durable recovery source before restore authority is opened.
    """
    global _recovery_custody_active, _recovery_custody_generation
    global _recovery_custody_entered_at_utc, _recovery_custody_reason
    global _recovery_custody_physical_sequence_regression
    global _recovery_custody_regression_witness
    global _recovery_custody_last_checkpoint

    first_public_count = int(first_public_count)
    if first_public_count <= 0:
        raise ValueError("recovery custody finalization requires positive public_count")

    finalize_started = time.monotonic()
    with _recovery_custody_lock:
        if not _recovery_custody_active:
            return {"active": False, "classified": False, "reason": "no_live_custody"}
        generation = _recovery_custody_generation
        if not generation:
            raise RuntimeError("active recovery custody has no generation identity")
        source_detail_id = _as_int(last_tb.get("_db_detail_id"))
        if source_detail_id is None or source_detail_id <= 0:
            raise RuntimeError(
                "active recovery custody source lacks durable campaign_detail identity"
            )
        physical_regression = bool(_recovery_custody_physical_sequence_regression)
        entered_at_utc = _recovery_custody_entered_at_utc
        initial_reason = _recovery_custody_reason
        regression_witness = copy.deepcopy(_recovery_custody_regression_witness)

        # Captured only when its canonical row crossed this same custody lock into
        # the persistence FIFO. It therefore cannot name a later state-worker
        # mutation that is still blocked behind this lock.
        routed_checkpoint = copy.deepcopy(_recovery_custody_last_checkpoint)
        if not isinstance(routed_checkpoint, dict):
            raise RuntimeError(
                "active recovery custody has no routed Better-Buckets checkpoint"
            )
        routed_checkpoint = _normalize_saved_ppb_checkpoint(routed_checkpoint)

        # State routing takes this same lock through its persistence-queue put.
        # Holding it while waiting for the FIFO barrier therefore proves that no
        # row in this generation can appear after the classification transaction.
        barrier_started = time.monotonic()
        _wait_for_clocks_persistence_barrier_locked()
        barrier_wait_s = time.monotonic() - barrier_started

        source_stats = _clocks_payload(last_tb).get("stats")
        if not isinstance(source_stats, dict):
            raise RuntimeError("recovery custody source lacks canonical CLOCKS statistics")
        source_reset = _as_int(source_stats.get("reset_count"))
        source_update = _as_int(source_stats.get("update_count"))
        if source_reset is None or source_update is None:
            raise RuntimeError("recovery custody source lacks statistics chronology")

        transaction_started = time.monotonic()
        with _clocks_persistence_lock:
            with open_db(row_dict=True) as conn:
                cur = conn.cursor()
                boundary_select_started = time.monotonic()
                cur.execute(
                    """
                    SELECT id, payload
                    FROM campaign_detail
                    WHERE campaign_type = %s
                      AND campaign = %s
                      AND id > %s
                      AND payload #>> '{recovery_custody,generation}' = %s
                      AND payload #>> '{campaign,public_count}' = %s
                    ORDER BY id DESC
                    LIMIT 1
                    """,
                    (
                        CAMPAIGN_TYPE_TEMPEST,
                        campaign_name,
                        int(source_detail_id),
                        generation,
                        str(first_public_count),
                    ),
                )
                row = cur.fetchone()
                boundary_select_s = time.monotonic() - boundary_select_started
                if row is None:
                    raise RuntimeError(
                        "first admitted recovery row is not durably present in recovery custody"
                    )

                boundary_id = int(row["id"])
                boundary_state = row["payload"]
                if isinstance(boundary_state, str):
                    boundary_state = json.loads(boundary_state)
                if not isinstance(boundary_state, dict):
                    raise RuntimeError("recovery custody boundary payload is not an object")

                boundary_clocks = _clocks_payload(boundary_state)
                boundary_ancestry = _dac_restore_population_ancestry_court(
                    boundary_clocks
                )
                if (
                    boundary_ancestry.get("available")
                    and not boundary_ancestry.get("valid")
                ):
                    _require_hard_failure(
                        "dac_recovery_boundary_population_ancestry_impossible",
                        {
                            "court": "DAC_OCXO_WELFORD_POPULATION_ANCESTRY",
                            "db_detail_id": boundary_id,
                            "campaign": campaign_name,
                            "generation": generation,
                            **copy.deepcopy(boundary_ancestry),
                        },
                        source="CLOCKS_RECOVERY_CUSTODY_COURT",
                    )

                boundary_stats = boundary_clocks.get("stats")
                if not isinstance(boundary_stats, dict):
                    raise RuntimeError("recovery custody boundary lacks CLOCKS statistics")
                boundary_reset = _as_int(boundary_stats.get("reset_count"))
                boundary_update = _as_int(boundary_stats.get("update_count"))

                cold_or_rebased = bool(
                    str(recover_mode).upper() == "DEAD_PRODUCER_RESTORE"
                    or physical_regression
                )
                boundary_ppb_checkpoint: Optional[Dict[str, Any]] = None
                if cold_or_rebased:
                    try:
                        boundary_ppb_checkpoint = copy.deepcopy(routed_checkpoint)
                    except (TypeError, ValueError) as exc:
                        raise RecoveryRetryableFailure(
                            "recovery_boundary_ppb_checkpoint_invalid",
                            {
                                "campaign": campaign_name,
                                "generation": generation,
                                "recover_mode": str(recover_mode).upper(),
                                "boundary_detail_id": boundary_id,
                                "error": str(exc),
                            },
                        ) from exc
                    checkpoint_reset = _as_int(boundary_ppb_checkpoint.get("reset_count"))
                    checkpoint_update = _as_int(boundary_ppb_checkpoint.get("update_count"))
                    if (
                        not boundary_ppb_checkpoint.get("recoverable")
                        or checkpoint_reset != boundary_reset
                        or checkpoint_update is None
                        or boundary_update is None
                        or checkpoint_update < boundary_update
                    ):
                        raise RecoveryRetryableFailure(
                            "recovery_boundary_ppb_checkpoint_not_recoverable",
                            {
                                "campaign": campaign_name,
                                "generation": generation,
                                "recover_mode": str(recover_mode).upper(),
                                "boundary_detail_id": boundary_id,
                                "boundary_reset_count": boundary_reset,
                                "boundary_update_count": boundary_update,
                                "checkpoint_reset_count": checkpoint_reset,
                                "checkpoint_update_count": checkpoint_update,
                                "checkpoint_status": boundary_ppb_checkpoint.get("status"),
                                "second_count": boundary_ppb_checkpoint.get("second_count"),
                                "expected_second_count": boundary_ppb_checkpoint.get(
                                    "expected_second_count"
                                ),
                                "minute_count": boundary_ppb_checkpoint.get("minute_count"),
                                "expected_minute_count": boundary_ppb_checkpoint.get(
                                    "expected_minute_count"
                                ),
                            },
                        )

                lineage_proved = True
                if cold_or_rebased:
                    source_probe = _holistic_restore_probe(last_tb)
                    boundary_probe = _holistic_restore_probe(
                        boundary_state, boundary_ppb_checkpoint
                    )
                    # Recovery lineage is Teensy instrument/statistics custody.
                    # Pi DAC targets may lawfully move while RECOVER is active, so
                    # remove control equality from this proof without weakening the
                    # clockface/Welford monotonicity requirements.
                    source_lineage_probe = copy.deepcopy(source_probe)
                    source_lineage_probe["servo_mode"] = boundary_probe.get("servo_mode")
                    source_lineage_probe["dither_operator_enabled"] = boundary_probe.get(
                        "dither_operator_enabled"
                    )
                    source_lineage_probe["ocxo1_dac"] = None
                    source_lineage_probe["ocxo2_dac"] = None
                    lineage_proved = bool(
                        boundary_reset == source_reset
                        and boundary_update is not None
                        and boundary_update > source_update
                        and _holistic_restore_probe_satisfied(
                            source_lineage_probe, boundary_probe
                        )
                    )
                    if not lineage_proved:
                        raise RecoveryRetryableFailure(
                            "recovery_custody_instrument_lineage_not_restored",
                            {
                                "campaign": campaign_name,
                                "generation": generation,
                                "recover_mode": str(recover_mode).upper(),
                                "physical_sequence_regression": physical_regression,
                                "source_reset_count": source_reset,
                                "source_update_count": source_update,
                                "boundary_reset_count": boundary_reset,
                                "boundary_update_count": boundary_update,
                                "source_probe": source_probe,
                                "boundary_probe": boundary_probe,
                                "regression_witness": regression_witness,
                            },
                        )

                classified_at = datetime.now(timezone.utc).isoformat().replace(
                    "+00:00", "Z"
                )
                classification_update_started = time.monotonic()
                if cold_or_rebased:
                    cur.execute(
                        """
                        UPDATE campaign_detail
                        SET payload = jsonb_set(
                            jsonb_set(
                                payload,
                                '{recovery_custody,classification}',
                                to_jsonb(%s::text),
                                true
                            ),
                            '{recovery_custody,classified_at_utc}',
                            to_jsonb(%s::text),
                            true
                        )
                        WHERE campaign_type = %s
                          AND id > %s
                          AND id < %s
                          AND payload #>> '{recovery_custody,generation}' = %s
                        """,
                        (
                            "SUPERSEDED_PRE_RECOVERY",
                            classified_at,
                            CAMPAIGN_TYPE_TEMPEST,
                            int(source_detail_id),
                            boundary_id,
                            generation,
                        ),
                    )
                    superseded = int(cur.rowcount or 0)
                    promoted_classification = "RESTORED_RECOVERY_LINEAGE"
                    cur.execute(
                        """
                        UPDATE campaign_detail
                        SET payload = jsonb_set(
                            jsonb_set(
                                jsonb_set(
                                    payload - 'holistic_restore_superseded',
                                    '{recovery_custody,classification}',
                                    to_jsonb(%s::text),
                                    true
                                ),
                                '{recovery_custody,classified_at_utc}',
                                to_jsonb(%s::text),
                                true
                            ),
                            '{recovery_custody,restore_authority}',
                            'true'::jsonb,
                            true
                        )
                        WHERE campaign_type = %s
                          AND id >= %s
                          AND payload #>> '{recovery_custody,generation}' = %s
                        """,
                        (
                            promoted_classification,
                            classified_at,
                            CAMPAIGN_TYPE_TEMPEST,
                            boundary_id,
                            generation,
                        ),
                    )
                    promoted = int(cur.rowcount or 0)
                else:
                    superseded = 0
                    promoted_classification = "LIVE_PRODUCER_ADOPT_CONTINUITY"
                    cur.execute(
                        """
                        UPDATE campaign_detail
                        SET payload = jsonb_set(
                            jsonb_set(
                                jsonb_set(
                                    payload - 'holistic_restore_superseded',
                                    '{recovery_custody,classification}',
                                    to_jsonb(%s::text),
                                    true
                                ),
                                '{recovery_custody,classified_at_utc}',
                                to_jsonb(%s::text),
                                true
                            ),
                            '{recovery_custody,restore_authority}',
                            'true'::jsonb,
                            true
                        )
                        WHERE campaign_type = %s
                          AND id > %s
                          AND payload #>> '{recovery_custody,generation}' = %s
                        """,
                        (
                            promoted_classification,
                            classified_at,
                            CAMPAIGN_TYPE_TEMPEST,
                            int(source_detail_id),
                            generation,
                        ),
                    )
                    promoted = int(cur.rowcount or 0)
                classification_update_s = (
                    time.monotonic() - classification_update_started
                )

                routed_reset = _as_int(routed_checkpoint.get("reset_count"))
                routed_update = _as_int(routed_checkpoint.get("update_count"))
                if routed_reset is None or routed_update is None:
                    raise RuntimeError(
                        "routed recovery checkpoint lacks statistics identity"
                    )
                cur.execute(
                    """
                    SELECT id, payload
                    FROM campaign_detail
                    WHERE campaign_type = %s
                      AND id > %s
                      AND payload #>> '{recovery_custody,generation}' = %s
                      AND payload #>> '{recovery_custody,restore_authority}' = 'true'
                      AND payload #>> '{clocks,stats,reset_count}' = %s
                      AND payload #>> '{clocks,stats,update_count}' = %s
                      AND NOT (payload @> '{"holistic_restore_superseded":true}'::jsonb)
                    ORDER BY id DESC
                    LIMIT 1
                    """,
                    (
                        CAMPAIGN_TYPE_TEMPEST,
                        int(source_detail_id),
                        generation,
                        str(int(routed_reset)),
                        str(int(routed_update)),
                    ),
                )
                routed_row = cur.fetchone()
                if routed_row is None:
                    raise RuntimeError(
                        "classified recovery custody lacks the durable row owning the "
                        "last routed Better-Buckets checkpoint"
                    )
                routed_state = routed_row["payload"]
                if isinstance(routed_state, str):
                    routed_state = json.loads(routed_state)
                if not isinstance(routed_state, dict):
                    raise RuntimeError(
                        "classified recovery checkpoint owner payload is not an object"
                    )
                if not _clocks_state_owns_recovery_config(
                    routed_state, routed_checkpoint
                ):
                    raise RuntimeError(
                        "classified recovery checkpoint owner is not CLOCKS restore authority"
                    )
                recovery_config_detail_id = int(routed_row["id"])
                _write_clocks_recovery_config(
                    cur,
                    routed_checkpoint,
                    source_detail_id=recovery_config_detail_id,
                )
        transaction_s = time.monotonic() - transaction_started

        result = {
            "active": False,
            "classified": True,
            "generation": generation,
            "entered_at_utc": entered_at_utc,
            "classified_at_utc": classified_at,
            "reason": initial_reason,
            "recover_mode": str(recover_mode).upper(),
            "physical_sequence_regression": physical_regression,
            "regression_witness": regression_witness,
            "first_public_count": first_public_count,
            "source_detail_id": int(source_detail_id),
            "boundary_detail_id": boundary_id,
            "source_reset_count": int(source_reset),
            "source_update_count": int(source_update),
            "boundary_reset_count": boundary_reset,
            "boundary_update_count": boundary_update,
            "boundary_ppb_checkpoint_recoverable": bool(
                boundary_ppb_checkpoint.get("recoverable")
                if isinstance(boundary_ppb_checkpoint, dict)
                else not cold_or_rebased
            ),
            "lineage_proved": bool(lineage_proved),
            "promoted_classification": promoted_classification,
            "rows_promoted": promoted,
            "rows_superseded": superseded,
            "recovery_config_detail_id": recovery_config_detail_id,
            "recovery_config_update_count": _as_int(routed_checkpoint.get("update_count")),
            "timing": {
                "barrier_wait_s": round(float(barrier_wait_s), 6),
                "boundary_select_s": round(float(boundary_select_s), 6),
                "classification_update_s": round(
                    float(classification_update_s), 6
                ),
                "transaction_s": round(float(transaction_s), 6),
                "total_s": round(float(time.monotonic() - finalize_started), 6),
            },
        }

        _recovery_custody_active = False
        _recovery_custody_generation = None
        _recovery_custody_entered_at_utc = None
        _recovery_custody_reason = None
        _recovery_custody_physical_sequence_regression = False
        _recovery_custody_regression_witness = {}
        _recovery_custody_last_checkpoint = None

    _diag["recovery_custody_active"] = False
    _diag["recovery_custody_generation"] = None
    _diag["recovery_custody_rows_promoted"] = (
        _diag.get("recovery_custody_rows_promoted", 0) + int(promoted)
    )
    _diag["recovery_custody_rows_superseded"] = (
        _diag.get("recovery_custody_rows_superseded", 0) + int(superseded)
    )
    _diag["last_recovery_custody"] = copy.deepcopy(result)
    logging.info(
        "✅ [recovery/custody] recovery evidence classified: mode=%s generation=%s; "
        "%d row(s) promoted, %d superseded; source detail %d -> boundary %d; "
        "instrument lineage proved=%s in %.3fs",
        str(recover_mode).upper(),
        generation,
        promoted,
        superseded,
        int(source_detail_id),
        boundary_id,
        lineage_proved,
        float(result["timing"]["total_s"]),
    )
    return result


def _teensy_clocks_health_ok() -> bool:
    """
    True when the Teensy CLOCKS recovery command surface is alive.

    REPORT_RECOVERY is the firmware-owned liveness surface used throughout the
    RECOVER lifecycle.  Do not probe the nonexistent generic CLOCKS.REPORT
    command here: a fresh Teensy can be publishing CLOCKS_FRAGMENT normally while
    that stale command name makes live silence recovery wait forever.

    Failed probes are deliberately silent in the log.  During a real Teensy
    reboot or USB/RawHID loss, this loop may run for minutes; the operator only
    needs the initial silence detection and the eventual recovery transition.
    """
    _diag["teensy_health_probe_attempts"] += 1
    if _fetch_teensy_recovery_status():
        _diag["teensy_health_probe_success"] += 1
        return True

    _diag["teensy_health_probe_failures"] += 1
    return False


def _ambient_instrument_restore_proof_candidate(clocks_fragment: Dict[str, Any]) -> bool:
    """True only for the exact durable N+1 Alpha row armed by ambient recovery."""
    if not _clocks_holistic_restore_proof_pending.is_set():
        return False
    stats = _path_get(clocks_fragment, "clocks.stats")
    if not isinstance(stats, dict):
        return False
    reset_count = _as_int(stats.get("reset_count"))
    update_count = _as_int(stats.get("update_count"))
    return bool(
        reset_count == _clocks_holistic_restore_proof_reset_count
        and update_count == _clocks_holistic_restore_proof_update_count
    )


def _ambient_instrument_recovery(regression: Dict[str, Any]) -> None:
    """Resurrect Alpha after a physical reboot while no TEMPEST campaign is active.

    The Pi process survived, so Pi-owned GNSS_RAW and DAC/control state are not
    rewound. The persistence barrier makes the newest pre-reboot canonical row
    the exact durable Alpha authority; its literal Better-Buckets checkpoint is
    staged back into firmware and the first admitted row must be exact N+1.
    """
    source_detail_id: Optional[int] = None
    try:
        _set_operational_state(
            OPERATIONAL_STATE_RECOVERING,
            reason="ambient_physical_sequence_rebase",
            source="CLOCKS_PHYSICAL_LIFETIME_COURT",
            details=copy.deepcopy(regression),
        )

        # The triggering newborn row was never built. Flush every older row that
        # was already routed so PostgreSQL, Pi checkpoint custody, and DAC
        # chronology all name the same pre-reboot boundary.
        _wait_for_clocks_persistence_barrier()
        detail, skipped = _read_latest_recoverable_clocks_state()
        if detail is None:
            raise RuntimeError(
                "physical Teensy lifetime was replaced but no durable canonical "
                "CLOCKS restore authority exists"
            )

        source_detail_id = _as_int(detail.get("_db_detail_id"))
        clocks = _clocks_payload(detail)
        checkpoint = _require_alpha_resurrection_checkpoint(
            clocks,
            campaign="AMBIENT",
            recovery_source_db_id=source_detail_id,
        )

        source_stats = clocks.get("stats") if isinstance(clocks, dict) else None
        source_reset = _as_int(source_stats.get("reset_count")) if isinstance(source_stats, dict) else None
        source_update = _as_int(source_stats.get("update_count")) if isinstance(source_stats, dict) else None
        if source_reset is None or source_update is None:
            raise RuntimeError("ambient Alpha restore source lacks statistics chronology")

        # Reinstall the exact durable Pi checkpoint image before any restored row
        # can mutate it. No SQL history is replayed.
        _restore_ppb_checkpoint_runtime(
            checkpoint,
            source_db_detail_id=source_detail_id,
        )
        _arm_holistic_restore_persistence_proof(
            detail,
            preserve_live_pi_control=True,
        )

        requested_monotonic = time.monotonic()
        response = _request_teensy_holistic_restore(
            clocks,
            allow_ppb_stage_required=True,
        )
        payload = response.get("payload") if isinstance(response, dict) else None
        payload = payload if isinstance(payload, dict) else {}
        status = str(payload.get("status") or "")

        ppb_stage: Dict[str, Any] = {
            "staged": False,
            "reason": "already_committed_on_teensy",
            "rolling_sequence": int(checkpoint.get("rolling_sequence") or 0),
        }
        if status == "monitor_restore_requires_ppb_state":
            ppb_stage = _stage_teensy_better_buckets_checkpoint(checkpoint)
            logging.info(
                "♻️ [ambient restore] Better-Buckets checkpoint staged directly: "
                "second=%d minute=%d source_update=%d gap_count=%d stage_s=%s",
                len(checkpoint.get("second_history") or []),
                len(checkpoint.get("minute_history") or []),
                int(checkpoint.get("update_count") or 0),
                int(checkpoint.get("gap_count") or 0),
                ppb_stage.get("waited_s"),
            )
            requested_monotonic = time.monotonic()
            response = _request_teensy_holistic_restore(clocks)
            payload = response.get("payload") if isinstance(response, dict) else None
            payload = payload if isinstance(payload, dict) else {}
            status = str(payload.get("status") or "")

        if status not in _TEENSY_MONITOR_RESTORE_ACCEPTED_STATUSES:
            raise RuntimeError(
                "ambient Teensy CLOCKS restore did not reach requested state: "
                f"status={status or 'missing_handler_status'} payload={payload!r}"
            )

        proof = _wait_for_holistic_restore(
            detail,
            requested_monotonic=requested_monotonic,
            preserve_live_pi_control=True,
        )

        # The N+1 court proves the resurrected Alpha lifetime, but Pi may have
        # intentionally retired transitional rows while that proof was forming.
        # Reacquire Alpha's exact current bounded rings before ordinary delta
        # custody resumes so those deliberate omissions are never misclassified
        # as an uncontrolled PI_OBSERVATION_GAP.
        ppb_refresh = _refresh_ppb_checkpoint_from_proved_alpha()
        proof_sequence = _as_int(proof.get("durable_proof_sequence"))
        if proof_sequence is None or proof_sequence <= 0:
            raise RuntimeError("ambient Alpha resurrection proof lacks durable sequence identity")
        boundary_detail_id = _find_clocks_exact_recovery_boundary_detail_id(
            source_detail_id=int(source_detail_id),
            source_reset_count=int(source_reset),
            source_update_count=int(source_update),
            proof_sequence=int(proof_sequence),
        )
        recovery_receipt = _record_clocks_recovery_receipt(
            boundary_detail_id=boundary_detail_id,
            source_detail=detail,
            source_checkpoint=checkpoint,
            recovery_mode="AMBIENT_ALPHA_RESURRECTION",
            proof=proof,
            alpha_proof_detail_id=boundary_detail_id,
        )

        result = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "mode": "AMBIENT_ALPHA_RESURRECTION",
            "source_detail_id": source_detail_id,
            "skipped_unrecoverable_details": int(skipped),
            "source_reset_count": int(source_reset),
            "source_update_count": int(source_update),
            "expected_first_update_count": int(source_update) + 1,
            "ppb_stage": copy.deepcopy(ppb_stage),
            "ppb_post_resurrection_refresh": copy.deepcopy(ppb_refresh),
            "proof": copy.deepcopy(proof),
            "recovery_receipt": copy.deepcopy(recovery_receipt),
            "campaign_restored": False,
            "pi_gnss_raw_rewound": False,
            "pi_control_rewound": False,
            "physical_regression": copy.deepcopy(regression),
        }
        _diag["ambient_instrument_recovery_completed"] = (
            _diag.get("ambient_instrument_recovery_completed", 0) + 1
        )
        _diag["last_ambient_instrument_recovery"] = copy.deepcopy(result)
        _clocks_persistence_enabled.set()
        _set_operational_state(
            OPERATIONAL_STATE_RUNNING,
            reason="ambient_alpha_resurrection_complete",
            source="CLOCKS_PHYSICAL_LIFETIME_COURT",
            details=result,
        )
        logging.info(
            "✅ [ambient restore] Alpha resurrection proved from durable detail_id=%s: "
            "reset_count=%d update_count=%d->%d; no campaign lifecycle invoked",
            source_detail_id,
            int(source_reset),
            int(source_update),
            int(source_update) + 1,
        )
    except AlphaResurrectionImpossible as exc:
        try:
            surrender = _surrender_unresurrectable_alpha_lineage(
                verdict=exc,
                active_campaign=None,
                source="AMBIENT_PHYSICAL_LIFETIME_COURT",
            )
            result = {
                "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "mode": "AMBIENT_ALPHA_LINEAGE_SURRENDER",
                "success": True,
                "source_detail_id": source_detail_id,
                "physical_regression": copy.deepcopy(regression),
                "lineage_surrender": copy.deepcopy(surrender),
            }
            _diag["ambient_instrument_recovery_completed"] = (
                _diag.get("ambient_instrument_recovery_completed", 0) + 1
            )
            _diag["last_ambient_instrument_recovery"] = copy.deepcopy(result)
            _set_operational_state(
                OPERATIONAL_STATE_RUNNING,
                reason="ambient_alpha_lineage_surrender_complete",
                source="CLOCKS_PHYSICAL_LIFETIME_COURT",
                details=result,
            )
            logging.critical(
                "🧭 [ambient restore] old Alpha could not be resurrected exactly; "
                "history was preserved, old restore authority surrendered, and a "
                "fresh durable Alpha epoch is now running"
            )
        except Exception as surrender_exc:
            _diag["ambient_instrument_recovery_failures"] = (
                _diag.get("ambient_instrument_recovery_failures", 0) + 1
            )
            details = {
                "error": str(surrender_exc),
                "original_lineage_verdict": exc.reason,
                "original_lineage_evidence": copy.deepcopy(exc.details),
                "source_detail_id": source_detail_id,
                "physical_regression": copy.deepcopy(regression),
            }
            logging.exception(
                "💥 [ambient restore] lawful Alpha lineage surrender failed while "
                "establishing the replacement epoch — entering HARD_FAILURE"
            )
            _enter_hard_failure(
                "ambient_alpha_lineage_surrender_failed",
                details,
                source="CLOCKS_PHYSICAL_LIFETIME_COURT",
            )
        return
    except HardFailureRequired:
        return
    except Exception as exc:
        _diag["ambient_instrument_recovery_failures"] = (
            _diag.get("ambient_instrument_recovery_failures", 0) + 1
        )
        details = {
            "error": str(exc),
            "source_detail_id": source_detail_id,
            "physical_regression": copy.deepcopy(regression),
        }
        _diag["last_ambient_instrument_recovery"] = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "mode": "AMBIENT_ALPHA_RESURRECTION",
            "success": False,
            **copy.deepcopy(details),
        }
        logging.exception(
            "💥 [ambient restore] Alpha resurrection failed — entering HARD_FAILURE"
        )
        _enter_hard_failure(
            "ambient_alpha_resurrection_failed",
            details,
            source="CLOCKS_PHYSICAL_LIFETIME_COURT",
        )
    finally:
        _ambient_instrument_recovery_hold.clear()
        _ambient_instrument_recovery_active.clear()
        _clocks_holistic_restore_proof_pending.clear()


def _begin_ambient_instrument_recovery(
    previous_sequence: int,
    observed_sequence: int,
) -> bool:
    """Open instrument-only recovery on a proved ambient physical lifetime change."""
    if _hard_failure_active() or _ambient_instrument_recovery_active.is_set():
        return False
    if not _startup_control_ready.is_set() or _campaign_active:
        return False

    # _campaign_active is intentionally lowered as soon as campaign recovery owns
    # a continuity loss.  It is therefore an authorship gate, not proof that no
    # durable TEMPEST lifecycle exists.  A physical sequence regression observed
    # while recovery custody is open belongs to that one recovery transaction.
    # Let the row continue into _advance_gnss_raw_instrument(), which records the
    # regression as cold-producer custody, rather than starting a competing ambient
    # Alpha resurrection that can steal the exact N+1 persistence lane.
    recovery_custody = _recovery_custody_snapshot()
    if (
        _auto_recovery_in_progress
        or _timebase_silence_recovery_active
        or bool(recovery_custody.get("active"))
    ):
        deferred = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "previous_sequence": int(previous_sequence),
            "observed_sequence": int(observed_sequence),
            "recovery_custody": copy.deepcopy(recovery_custody),
        }
        _diag["ambient_instrument_recovery_deferred_to_recovery_custody"] = (
            _diag.get("ambient_instrument_recovery_deferred_to_recovery_custody", 0) + 1
        )
        _diag["last_ambient_instrument_recovery_deferred"] = copy.deepcopy(deferred)
        logging.warning(
            "🧭 [clocks] physical CLOCKS sequence rebased %d -> %d while live "
            "recovery custody generation=%s owns producer classification; deferring "
            "to that recovery instead of opening a competing ambient Alpha restore",
            int(previous_sequence),
            int(observed_sequence),
            recovery_custody.get("generation") or "pending",
        )
        return False

    regression = {
        "observed_sequence": int(observed_sequence),
        "previous_sequence": int(previous_sequence),
        "observed_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "campaign_active": False,
    }
    _ambient_instrument_recovery_active.set()
    _ambient_instrument_recovery_hold.set()
    _clocks_persistence_enabled.clear()
    _diag["ambient_instrument_recovery_started"] = (
        _diag.get("ambient_instrument_recovery_started", 0) + 1
    )
    _diag["last_ambient_instrument_recovery"] = {
        "ts_utc": regression["observed_at_utc"],
        "mode": "AMBIENT_ALPHA_RESURRECTION",
        "state": "STARTED",
        "physical_regression": copy.deepcopy(regression),
    }
    logging.error(
        "🧭 [ambient restore] physical CLOCKS sequence rebased %d -> %d with no "
        "active campaign; suspending canonical authorship and resurrecting Alpha "
        "from durable instrument custody",
        int(previous_sequence),
        int(observed_sequence),
    )
    threading.Thread(
        target=_ambient_instrument_recovery,
        args=(regression,),
        name="clocks-ambient-instrument-recover",
        daemon=True,
    ).start()
    return True


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
            if _hard_failure_active():
                return
            if _teensy_clocks_health_ok():
                logging.info(
                    "✅ [clocks] @%s Teensy CLOCKS REPORT_RECOVERY responded after CLOCKS_FRAGMENT silence — "
                    "invoking campaign recovery",
                    system_time_z(),
                )
                attempts += 1
                _diag["auto_recovery_attempts"] = _diag.get("auto_recovery_attempts", 0) + 1
                _diag["timebase_silence_recovery_started"] += 1
                try:
                    _restore_active_campaign_state()
                    _set_operational_state(
                        OPERATIONAL_STATE_RUNNING,
                        reason="timebase_silence_recovery_complete",
                        source="TIMEBASE_SILENCE_MONITOR",
                    )
                    logging.info(
                        "✅ [clocks] @%s CLOCKS_FRAGMENT silence recovery complete",
                        system_time_z(),
                    )
                    return
                except HardFailureRequired:
                    return
                except RecoveryRetryableFailure as e:
                    _diag["auto_recovery_failures"] = _diag.get("auto_recovery_failures", 0) + 1
                    if not getattr(e, "cleanup_sent", False):
                        _cleanup_after_recovery_failure(e.reason, e.details)

                    status = str((e.details or {}).get("status") or "")
                    terminal_firmware_rejection = (
                        status == "recover_rejected_dead_producer_epoch_prepare"
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
                            "reason=%s status=%s. Automatic continuation is no longer asserted.",
                            attempts,
                            e.reason,
                            status or "unknown",
                            exc_info=True,
                        )
                        _enter_hard_failure(
                            "timebase_silence_recovery_exhausted",
                            copy.deepcopy(_diag["last_auto_recovery_exhausted"]),
                            source="TIMEBASE_SILENCE_RECOVERY",
                        )
                        return

                    _diag["auto_recovery_retries"] = (
                        _diag.get("auto_recovery_retries", 0) + 1
                    )
                    logging.warning(
                        "⚠️ [clocks] CLOCKS stream recovery attempt %d/%d could not complete: %s "
                        "(firmware status=%s). The attempt was cleaned up and will retry after %.1fs",
                        attempts,
                        int(AUTO_RECOVERY_MAX_ATTEMPTS),
                        e.reason,
                        str((e.details or {}).get("status") or "not reported"),
                        float(AUTO_RECOVERY_RETRY_DELAY_S),
                        exc_info=True,
                    )
                    time.sleep(float(AUTO_RECOVERY_RETRY_DELAY_S))
                    continue
                except Exception:
                    if _hard_failure_active():
                        return
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

    if _hard_failure_active():
        return False
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

    _set_operational_state(
        OPERATIONAL_STATE_RECOVERING,
        reason="timebase_silence",
        source="TIMEBASE_SILENCE_MONITOR",
        details=details,
    )
    _begin_recovery_clocks_custody("timebase_silence", details)
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

        if _hard_failure_active():
            continue
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

    if _hard_failure_active():
        return False

    _set_operational_state(
        OPERATIONAL_STATE_RECOVERING,
        reason=reason,
        source=source,
        details=copy.deepcopy(details or {}),
    )
    _diag["hard_faults_total"] = _diag.get("hard_faults_total", 0) + 1
    _diag["last_hard_fault"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "reason": reason,
        "source": source,
        "details": details,
    }

    _begin_recovery_clocks_custody(
        reason,
        {"source": source, **dict(details or {})},
    )
    _campaign_active = False

    if _auto_recovery_in_progress:
        _note_recovery_interruption(reason, details, source=source)
        logging.error(
            "💥 [clocks] %s: %s — auto-recovery already in progress; current attempt will abort/retry",
            source, reason,
        )
        return False

    logging.error(
        "💥 [clocks] %s reported %s; CLOCKS is suspending campaign authorship "
        "and starting automatic recovery",
        source,
        reason,
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
                    _set_operational_state(
                        OPERATIONAL_STATE_RUNNING,
                        reason="auto_recovery_complete",
                        source=source,
                    )
                    logging.info("✅ [clocks] @%s auto-recovery complete", system_time_z())
                    return
                except RecoveryRetryableFailure as e:
                    if isinstance(e, RecoveryInterrupted):
                        _diag["auto_recovery_interrupted"] = _diag.get("auto_recovery_interrupted", 0) + 1
                    logging.warning(
                        "⚠️ [clocks] automatic recovery attempt %d/%d could not complete: %s "
                        "(firmware status=%s); cleaning the attempt before retry",
                        attempt,
                        int(AUTO_RECOVERY_MAX_ATTEMPTS),
                        e.reason,
                        str((e.details or {}).get("status") or "not reported"),
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
        except HardFailureRequired:
            # The proving court already latched HARD_FAILURE with full evidence.
            return
        except Exception as e:
            logging.exception("💥 [clocks] auto-recovery FAILED — entering HARD_FAILURE")
            _diag["auto_recovery_failures"] = _diag.get("auto_recovery_failures", 0) + 1
            try:
                _cleanup_after_recovery_failure(
                    "auto_recovery_failed",
                    {"error": str(e), "source": source, "reason": reason},
                )
            finally:
                _enter_hard_failure(
                    "auto_recovery_exhausted",
                    {"error": str(e), "trigger_reason": reason, "trigger_source": source},
                    source="AUTO_RECOVERY",
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

    _begin_recovery_clocks_custody(
        reason,
        {"source": "HARD_FAULT", **dict(details or {})},
    )
    _campaign_active = False

    if _auto_recovery_in_progress:
        logging.error(
            "💥 [clocks] HARD FAULT: %s — auto-recovery already in progress, skipping",
            reason,
        )
        raise RuntimeError(f"HARD FAULT: {reason} (recovery already in progress)")

    logging.error(
        "💥 [clocks] HARD FAULT: %s — campaign authorship is suspended and "
        "automatic recovery is starting (sequence=%s public_count=%s)",
        reason,
        details.get("sequence") if isinstance(details, dict) else None,
        details.get("public_count") if isinstance(details, dict) else None,
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




def _read_startup_system_feature(name: str) -> Dict[str, Any]:
    """Read one SYSTEM readiness leaf without converting malformed testimony to WAIT."""
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
    """Wait for real PostgreSQL and Teensy RPC readiness before startup custody work."""
    started = time.monotonic()
    last_log_at = started
    last_signature: Optional[Tuple[Tuple[str, str, bool], ...]] = None
    logged_wait = False
    checks = 0
    _diag["startup_infrastructure_waits"] += 1

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
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "status": "NOMINAL" if not blockers else "WAITING",
            "checks": int(checks),
            "waited_s": round(float(waited), 3),
            "required": copy.deepcopy(observed),
            "blockers": copy.deepcopy(blockers),
        }
        _diag["last_startup_infrastructure_wait"] = snapshot

        if not blockers:
            _diag["startup_infrastructure_wait_seconds_last"] = float(waited)
            if logged_wait:
                logging.info(
                    "✅ [clocks/startup] infrastructure admitted after %.1fs: "
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
                "⏳ [clocks/startup] waiting for application infrastructure "
                "(%.1fs): %s",
                waited,
                detail,
            )
            logged_wait = True
            last_log_at = now
            last_signature = signature

        time.sleep(STARTUP_INFRASTRUCTURE_POLL_S)


def _ensure_system_location(*, context: str) -> Dict[str, Any]:
    """Ask SYSTEM to reassert its durable location selection on the GF-8802."""
    response = send_command(
        machine="PI",
        subsystem="SYSTEM",
        command="ENSURE_LOCATION",
        args={"context": context},
    )
    if not isinstance(response, dict) or not response.get("success"):
        raise RuntimeError(
            f"SYSTEM.ENSURE_LOCATION failed during {context}: {response!r}"
        )

    payload = response.get("payload")
    if not isinstance(payload, dict):
        raise RuntimeError(
            f"SYSTEM.ENSURE_LOCATION returned malformed payload during {context}: "
            f"{response!r}"
        )

    location = payload.get("current_location")
    verified_mode = str(payload.get("verified_pos_mode_name") or "?")
    freq_mode = str(payload.get("freq_mode_name") or "?")
    logging.info(
        "📍 [clocks/location] %s: SYSTEM current_location=%s; "
        "GF-8802 verified position_mode=%s freq_mode=%s",
        context,
        location or "NONE",
        verified_mode,
        freq_mode,
    )
    return payload










def _wait_for_startup_location() -> Dict[str, Any]:
    """Patiently realize SYSTEM.current_location before cold-start admission."""
    started = time.monotonic()
    next_log = started
    attempts = 0
    last_error = "not attempted"
    _diag["startup_location_waits"] = _diag.get("startup_location_waits", 0) + 1

    while True:
        attempts += 1
        try:
            payload = _ensure_system_location(context="COLD_START_ADMISSION")
            waited = time.monotonic() - started
            result = {
                "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "status": "READY",
                "attempts": int(attempts),
                "waited_s": round(float(waited), 3),
                "current_location": payload.get("current_location"),
                "verified_pos_mode_name": payload.get("verified_pos_mode_name"),
                "freq_mode_name": payload.get("freq_mode_name"),
            }
            _diag["startup_location_wait_seconds_last"] = float(waited)
            _diag["last_startup_location_wait"] = result
            logging.info(
                "✅ [clocks/startup] SYSTEM location realized after %.1fs: "
                "current_location=%s GF-8802=%s freq_mode=%s",
                waited,
                payload.get("current_location") or "NONE",
                payload.get("verified_pos_mode_name") or "?",
                payload.get("freq_mode_name") or "?",
            )
            return payload
        except Exception as exc:
            last_error = str(exc)

        now = time.monotonic()
        waited = now - started
        _diag["last_startup_location_wait"] = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "status": "WAITING",
            "attempts": int(attempts),
            "waited_s": round(float(waited), 3),
            "last_error": last_error,
        }
        if now >= next_log:
            logging.info(
                "⏳ [clocks/startup] waiting for SYSTEM/GF-8802 location realization "
                "(%.0fs, attempt=%d): %s",
                waited,
                attempts,
                last_error,
            )
            next_log = now + STARTUP_LOCATION_STATUS_LOG_INTERVAL_S
        time.sleep(STARTUP_LOCATION_RETRY_S)


def _clocks_payload(monitor: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    if not isinstance(monitor, dict):
        return {}
    clocks = monitor.get("clocks")
    return clocks if isinstance(clocks, dict) else {}



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


def _ppb_endpoint_from_payload(value: Any, *, path: str) -> Dict[str, Any]:
    """Return one strict six-field producer endpoint, or raise on ambiguity."""
    if not isinstance(value, dict):
        raise ValueError(f"{path} is not an endpoint object")
    missing = [
        field
        for field in (
            "reference_ns", "dwt_error_cycles", "ocxo1_error_ns",
            "ocxo2_error_ns", "rolling_sequence", "interval_count",
        )
        if field not in value
    ]
    if missing:
        raise ValueError(f"{path} missing endpoint fields {missing}")
    try:
        endpoint = {
            "reference_ns": int(value["reference_ns"]),
            "dwt_error_cycles": float(value["dwt_error_cycles"]),
            "ocxo1_error_ns": int(value["ocxo1_error_ns"]),
            "ocxo2_error_ns": int(value["ocxo2_error_ns"]),
            "rolling_sequence": int(value["rolling_sequence"]),
            "interval_count": int(value["interval_count"]),
        }
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{path} has invalid endpoint scalar: {exc}") from exc
    if not math.isfinite(endpoint["dwt_error_cycles"]):
        raise ValueError(f"{path}.dwt_error_cycles is non-finite")
    for field in ("reference_ns", "rolling_sequence", "interval_count"):
        if endpoint[field] < 0:
            raise ValueError(f"{path}.{field} is negative")
    return endpoint


def _ppb_endpoints_equal(a: Any, b: Any) -> bool:
    if not isinstance(a, dict) or not isinstance(b, dict):
        return False
    try:
        return bool(
            int(a["reference_ns"]) == int(b["reference_ns"])
            and abs(float(a["dwt_error_cycles"]) - float(b["dwt_error_cycles"])) <= 1e-9
            and int(a["ocxo1_error_ns"]) == int(b["ocxo1_error_ns"])
            and int(a["ocxo2_error_ns"]) == int(b["ocxo2_error_ns"])
            and int(a["rolling_sequence"]) == int(b["rolling_sequence"])
            and int(a["interval_count"]) == int(b["interval_count"])
        )
    except (KeyError, TypeError, ValueError):
        return False


def _ppb_window_value_from_endpoints(
    current: Dict[str, Any],
    anchor: Dict[str, Any],
    lane: str,
) -> float:
    interval_count = int(current["interval_count"]) - int(anchor["interval_count"])
    reference_ns = int(current["reference_ns"]) - int(anchor["reference_ns"])
    if interval_count <= 0 or reference_ns <= 0:
        raise ValueError(
            f"Better-Buckets proof has nonpositive population lane={lane} "
            f"interval_count={interval_count} reference_ns={reference_ns}"
        )
    if lane == "dwt":
        expected_cycles = (
            float(reference_ns) * float(DWT_EXPECTED_PER_PPS)
        ) / float(NS_PER_SECOND)
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


def _validate_firmware_ppb_checkpoint_delta(stats: Dict[str, Any]) -> Dict[str, Any]:
    """Validate one row-local firmware proof/delta without mutating Pi state."""
    raw = stats.get("rolling_ppb_checkpoint") if isinstance(stats, dict) else None
    if not isinstance(raw, dict):
        raise ValueError("CLOCKS_FRAGMENT_V4 missing stats.rolling_ppb_checkpoint")
    if raw.get("schema") != PPB_FIRMWARE_DELTA_SCHEMA:
        raise ValueError(
            "unsupported Better-Buckets checkpoint schema "
            f"{raw.get('schema')!r}"
        )
    if not bool(raw.get("valid")):
        raise ValueError("firmware Better-Buckets checkpoint testimony is not valid")

    update_count = _as_int(stats.get("update_count"))
    current_sequence = _as_int(stats.get("rolling_ppb_current_sequence"))
    reset_count = _as_int(stats.get("reset_count"))
    if update_count is None or update_count < 0 or reset_count is None or reset_count < 0:
        raise ValueError("firmware Better-Buckets testimony lacks statistics chronology")
    if current_sequence is None or current_sequence < 0 or current_sequence > update_count:
        raise ValueError("firmware Better-Buckets testimony has invalid current sequence")

    rolling_sequence = _as_int(raw.get("rolling_sequence"))
    second_count = _as_int(raw.get("second_count"))
    minute_count = _as_int(raw.get("minute_count"))
    last_minute_key = _as_int(raw.get("last_minute_key"))
    if rolling_sequence is None or rolling_sequence < 0:
        raise ValueError("firmware Better-Buckets checkpoint missing rolling_sequence")
    if rolling_sequence != current_sequence:
        raise ValueError(
            "firmware Better-Buckets checkpoint current-sequence mismatch: "
            f"stats={current_sequence} checkpoint={rolling_sequence}"
        )
    if second_count is None or not (0 <= second_count <= PPB_SECOND_CAPACITY):
        raise ValueError(f"firmware Better-Buckets second_count invalid: {second_count}")
    if minute_count is None or not (0 <= minute_count <= PPB_MINUTE_CAPACITY):
        raise ValueError(f"firmware Better-Buckets minute_count invalid: {minute_count}")
    if last_minute_key is None or last_minute_key < 0:
        raise ValueError("firmware Better-Buckets last_minute_key invalid")

    current = _ppb_endpoint_from_payload(raw.get("current"), path="rolling_ppb_checkpoint.current")
    if int(current["rolling_sequence"]) != int(rolling_sequence):
        raise ValueError("firmware Better-Buckets current endpoint identity mismatch")

    origin_valid = bool(raw.get("origin_valid"))
    origin = (
        _ppb_endpoint_from_payload(raw.get("origin"), path="rolling_ppb_checkpoint.origin")
        if origin_valid
        else _ppb_zero_endpoint()
    )
    if origin_valid and int(origin["rolling_sequence"]) > int(current["rolling_sequence"]):
        raise ValueError("firmware Better-Buckets origin follows current endpoint")

    endpoint_admitted = bool(stats.get("rolling_ppb_endpoint_admitted"))
    interval_advanced = bool(stats.get("rolling_ppb_interval_advanced"))
    second_append_raw = raw.get("second_append")
    minute_append_raw = raw.get("minute_append")
    second_append_raw = second_append_raw if isinstance(second_append_raw, dict) else {}
    minute_append_raw = minute_append_raw if isinstance(minute_append_raw, dict) else {}
    second_append_valid = bool(second_append_raw.get("valid"))
    minute_append_valid = bool(minute_append_raw.get("valid"))
    second_append = (
        _ppb_endpoint_from_payload(
            second_append_raw.get("endpoint"),
            path="rolling_ppb_checkpoint.second_append.endpoint",
        )
        if second_append_valid
        else None
    )
    minute_append = (
        _ppb_endpoint_from_payload(
            minute_append_raw.get("endpoint"),
            path="rolling_ppb_checkpoint.minute_append.endpoint",
        )
        if minute_append_valid
        else None
    )

    if second_append_valid != endpoint_admitted:
        raise ValueError(
            "firmware Better-Buckets second append disagrees with endpoint admission: "
            f"append={second_append_valid} admitted={endpoint_admitted}"
        )
    if interval_advanced and not endpoint_admitted:
        raise ValueError("firmware Better-Buckets interval advanced on excluded endpoint")
    if second_append_valid and not _ppb_endpoints_equal(second_append, current):
        raise ValueError("firmware Better-Buckets second append is not the current endpoint")
    if minute_append_valid and not second_append_valid:
        raise ValueError("firmware Better-Buckets minute append exists without second append")
    if minute_append_valid and not _ppb_endpoints_equal(minute_append, current):
        raise ValueError("firmware Better-Buckets minute append is not the current endpoint")
    if rolling_sequence > 0:
        if second_count <= 0 or not origin_valid:
            raise ValueError("firmware Better-Buckets nonempty state lacks ring/origin custody")
        expected_minute_key = _ppb_minute_key(rolling_sequence)
        if minute_count > 0 and last_minute_key != expected_minute_key:
            raise ValueError(
                "firmware Better-Buckets minute-key mismatch: "
                f"expected={expected_minute_key} observed={last_minute_key}"
            )
    elif second_count != 0 or minute_count != 0:
        raise ValueError("firmware Better-Buckets empty current sequence has nonempty rings")

    windows = (
        ("10_min", PPB_10_MIN_SECONDS),
        ("60_min", PPB_60_MIN_SECONDS),
        ("8_hour", PPB_8_HOUR_SECONDS),
        ("24_hour", PPB_24_HOUR_SECONDS),
    )
    proof: Dict[str, Any] = {}
    checked = 0
    for key, _window_seconds in windows:
        node_raw = raw.get(key)
        if not isinstance(node_raw, dict):
            raise ValueError(f"firmware Better-Buckets proof missing {key}")
        valid = bool(node_raw.get("valid"))
        sample_count = _as_int(node_raw.get("sample_count"))
        anchor = None
        if valid:
            if sample_count is None or sample_count <= 0:
                raise ValueError(f"firmware Better-Buckets {key} proof has invalid sample_count")
            anchor = _ppb_endpoint_from_payload(
                node_raw.get("anchor"), path=f"rolling_ppb_checkpoint.{key}.anchor"
            )
            expected_samples = int(current["interval_count"]) - int(anchor["interval_count"])
            if expected_samples != sample_count:
                raise ValueError(
                    f"firmware Better-Buckets {key} proof sample-count mismatch: "
                    f"published={sample_count} endpoint_delta={expected_samples}"
                )
        lane_results: Dict[str, Any] = {}
        for lane in ("dwt", "vclock", "ocxo1", "ocxo2"):
            buckets = _path_get(stats, f"{lane}.ppb_buckets")
            buckets = buckets if isinstance(buckets, dict) else {}
            recorded = buckets.get(key)
            if not valid:
                if recorded is not None:
                    raise ValueError(
                        f"firmware Better-Buckets {lane}.{key} exists without a valid proof"
                    )
                lane_results[lane] = None
                continue
            assert anchor is not None
            computed = _ppb_window_value_from_endpoints(current, anchor, lane)
            try:
                recorded_value = float(recorded)
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"firmware Better-Buckets {lane}.{key} missing/non-numeric published value"
                ) from exc
            if not math.isfinite(recorded_value):
                raise ValueError(f"firmware Better-Buckets {lane}.{key} is non-finite")
            delta_ppb = computed - recorded_value
            if abs(delta_ppb) > PPB_PROOF_VERIFY_TOLERANCE_PPB:
                raise ValueError(
                    "firmware Better-Buckets row-local proof mismatch: "
                    f"lane={lane} window={key} computed={computed:.9f} "
                    f"recorded={recorded_value:.9f} delta={delta_ppb:.9f}"
                )
            lane_results[lane] = {
                "computed": round(computed, 9),
                "recorded": round(recorded_value, 9),
                "delta": round(delta_ppb, 9),
            }
            checked += 1
        proof[key] = {
            "valid": valid,
            "sample_count": int(sample_count or 0),
            "anchor": copy.deepcopy(anchor) if anchor is not None else None,
            "lanes": lane_results,
        }

    return {
        "schema": PPB_FIRMWARE_DELTA_SCHEMA,
        "reset_count": int(reset_count),
        "update_count": int(update_count),
        "rolling_sequence": int(rolling_sequence),
        "second_count": int(second_count),
        "minute_count": int(minute_count),
        "last_minute_key": int(last_minute_key),
        "origin_valid": bool(origin_valid),
        "origin": origin,
        "current": current,
        "second_append_valid": bool(second_append_valid),
        "second_append": second_append,
        "minute_append_valid": bool(minute_append_valid),
        "minute_append": minute_append,
        "proof": proof,
        "proof_checks": int(checked),
    }


def _ppb_checkpoint_delta_signature(delta: Dict[str, Any]) -> str:
    stable = {
        key: delta.get(key)
        for key in (
            "schema", "reset_count", "update_count", "rolling_sequence",
            "second_count", "minute_count", "last_minute_key", "origin_valid",
            "origin", "current", "second_append_valid", "second_append",
            "minute_append_valid", "minute_append", "proof",
        )
    }
    return json.dumps(stable, sort_keys=True, separators=(",", ":"), ensure_ascii=True)


def _ppb_checkpoint_new_runtime(
    *,
    reason: str,
    gap_count: int = 0,
    last_gap: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    return {
        "reset_count": None,
        "last_update_count": None,
        "last_delta_signature": None,
        # rolling_sequence is the PPB_RESTORE transaction identity and follows
        # stats.update_count. current_sequence is the latest admitted endpoint;
        # they differ lawfully when the newest statistical row is excluded.
        "rolling_sequence": 0,
        "current_sequence": 0,
        "expected_second_count": 0,
        "expected_minute_count": 0,
        "last_minute_key": 0,
        "origin_valid": False,
        "origin": _ppb_zero_endpoint(),
        "current": _ppb_zero_endpoint(),
        "second_history": deque(maxlen=PPB_SECOND_CAPACITY),
        "minute_history": deque(maxlen=PPB_MINUTE_CAPACITY),
        "contiguous_from_update_count": None,
        "gap_count": int(gap_count),
        "last_gap": copy.deepcopy(last_gap),
        "status_reason": str(reason),
        "seeded_from_durable": False,
        "seed_source_db_detail_id": None,
        "proof_checks": 0,
    }


def _ppb_checkpoint_runtime_ensure_locked() -> Dict[str, Any]:
    global _ppb_checkpoint_runtime
    if _ppb_checkpoint_runtime is None:
        _ppb_checkpoint_runtime = _ppb_checkpoint_new_runtime(reason="NO_DURABLE_CHECKPOINT_SEED")
    return _ppb_checkpoint_runtime


def _ppb_checkpoint_history_anchor_locked(
    runtime: Dict[str, Any],
    window_seconds: int,
    *,
    exact_second_history: bool,
) -> Optional[Dict[str, Any]]:
    current = runtime.get("current")
    origin = runtime.get("origin")
    if not isinstance(current, dict) or not runtime.get("origin_valid") or not isinstance(origin, dict):
        return None
    current_sequence = int(current.get("rolling_sequence") or 0)
    origin_sequence = int(origin.get("rolling_sequence") or 0)
    if current_sequence <= 0 or origin_sequence > current_sequence:
        return None
    if current_sequence - origin_sequence <= int(window_seconds):
        return origin
    target = current_sequence - int(window_seconds)
    history = runtime["second_history"] if exact_second_history else runtime["minute_history"]
    for endpoint in history:
        sequence = int(endpoint.get("rolling_sequence") or 0)
        if target <= sequence < current_sequence:
            return endpoint
    return None


def _ppb_checkpoint_snapshot_locked(runtime: Dict[str, Any]) -> Dict[str, Any]:
    second_history = [copy.deepcopy(value) for value in runtime["second_history"]]
    minute_history = [copy.deepcopy(value) for value in runtime["minute_history"]]
    expected_second = int(runtime.get("expected_second_count") or 0)
    expected_minute = int(runtime.get("expected_minute_count") or 0)
    rolling_sequence = int(runtime.get("rolling_sequence") or 0)
    current_sequence = int(runtime.get("current_sequence") or 0)

    complete_counts = (
        len(second_history) == expected_second
        and len(minute_history) == expected_minute
    )
    structure_complete = complete_counts
    if current_sequence > 0:
        structure_complete = bool(
            structure_complete
            and runtime.get("origin_valid")
            and second_history
            and _ppb_endpoints_equal(second_history[-1], runtime.get("current"))
            and (
                not minute_history
                or _ppb_minute_key(int(minute_history[-1]["rolling_sequence"]))
                    == int(runtime.get("last_minute_key") or 0)
            )
            and int(runtime.get("last_minute_key") or 0) == _ppb_minute_key(current_sequence)
        )
    elif expected_second != 0 or expected_minute != 0 or runtime.get("origin_valid"):
        structure_complete = False

    recoverable = bool(structure_complete)
    status = "RECOVERABLE" if recoverable else "WARMING_FOR_COMPLETE_FIRMWARE_HISTORY"
    if runtime.get("last_gap") and not recoverable:
        status = "WARMING_AFTER_OBSERVATION_GAP"

    return {
        "schema": PPB_PI_CHECKPOINT_SCHEMA,
        "source_schema": PPB_FIRMWARE_DELTA_SCHEMA,
        "valid": True,
        "recoverable": recoverable,
        "status": status,
        "status_reason": runtime.get("status_reason"),
        "reset_count": runtime.get("reset_count"),
        "update_count": runtime.get("last_update_count"),
        "rolling_sequence": rolling_sequence,
        "current_sequence": current_sequence,
        "last_minute_key": int(runtime.get("last_minute_key") or 0),
        "origin_valid": bool(runtime.get("origin_valid")),
        "origin": copy.deepcopy(runtime.get("origin") or _ppb_zero_endpoint()),
        "current": copy.deepcopy(runtime.get("current") or _ppb_zero_endpoint()),
        "expected_second_count": expected_second,
        "expected_minute_count": expected_minute,
        "second_count": len(second_history),
        "minute_count": len(minute_history),
        "second_history": second_history,
        "minute_history": minute_history,
        "contiguous_from_update_count": runtime.get("contiguous_from_update_count"),
        "gap_count": int(runtime.get("gap_count") or 0),
        "last_gap": copy.deepcopy(runtime.get("last_gap")),
        "seeded_from_durable": bool(runtime.get("seeded_from_durable")),
        "seed_source_db_detail_id": runtime.get("seed_source_db_detail_id"),
        "last_delta_signature": runtime.get("last_delta_signature"),
        "proof_checks": int(runtime.get("proof_checks") or 0),
    }


def _ppb_checkpoint_assert_history_matches_proof_locked(
    runtime: Dict[str, Any],
    delta: Dict[str, Any],
) -> None:
    windows = (
        ("10_min", PPB_10_MIN_SECONDS, True),
        ("60_min", PPB_60_MIN_SECONDS, False),
        ("8_hour", PPB_8_HOUR_SECONDS, False),
        ("24_hour", PPB_24_HOUR_SECONDS, False),
    )
    for key, seconds, exact in windows:
        proof = (delta.get("proof") or {}).get(key)
        if not isinstance(proof, dict) or not proof.get("valid"):
            continue
        expected_anchor = proof.get("anchor")
        actual_anchor = _ppb_checkpoint_history_anchor_locked(
            runtime, seconds, exact_second_history=exact
        )
        if not _ppb_endpoints_equal(expected_anchor, actual_anchor):
            raise RuntimeError(
                "Pi Better-Buckets checkpoint ring disagrees with producer proof: "
                f"window={key} expected_anchor={expected_anchor!r} "
                f"actual_anchor={actual_anchor!r}"
            )


def _ppb_checkpoint_ingest(stats: Dict[str, Any]) -> Dict[str, Any]:
    """Fold one verified producer delta into the literal Pi restore image."""
    global _ppb_checkpoint_runtime
    delta = _validate_firmware_ppb_checkpoint_delta(stats)
    signature = _ppb_checkpoint_delta_signature(delta)
    reset_count = int(delta["reset_count"])
    update_count = int(delta["update_count"])

    with _ppb_checkpoint_lock:
        runtime = _ppb_checkpoint_runtime_ensure_locked()
        previous_reset = runtime.get("reset_count")
        previous_update = runtime.get("last_update_count")

        if previous_reset is not None and (
            int(previous_reset) != reset_count
            or (previous_update is not None and update_count < int(previous_update))
        ):
            prior_gap_count = int(runtime.get("gap_count") or 0)
            last_gap = {
                "reason": "STATISTICS_EPOCH_REPLACED",
                "previous_reset_count": int(previous_reset),
                "previous_update_count": int(previous_update or 0),
                "observed_reset_count": reset_count,
                "observed_update_count": update_count,
            }
            _ppb_checkpoint_runtime = _ppb_checkpoint_new_runtime(
                reason="STATISTICS_EPOCH_REPLACED",
                gap_count=prior_gap_count,
                last_gap=last_gap,
            )
            runtime = _ppb_checkpoint_runtime
            _diag["ppb_checkpoint_epoch_rebases"] = (
                _diag.get("ppb_checkpoint_epoch_rebases", 0) + 1
            )
            previous_update = None

        if previous_update is not None and update_count == int(previous_update):
            prior_signature = runtime.get("last_delta_signature")
            if prior_signature is not None and prior_signature != signature:
                raise RuntimeError(
                    "duplicate Better-Buckets statistics identity changed producer testimony: "
                    f"reset_count={reset_count} update_count={update_count}"
                )
            runtime["last_delta_signature"] = signature
            runtime["proof_checks"] = int(delta.get("proof_checks") or 0)
            snapshot = _ppb_checkpoint_snapshot_locked(runtime)
            _diag["ppb_checkpoint_rows_verified"] += 1
            _diag["last_ppb_checkpoint"] = {
                "reset_count": reset_count,
                "update_count": update_count,
                "rolling_sequence": snapshot.get("rolling_sequence"),
                "recoverable": snapshot.get("recoverable"),
                "status": snapshot.get("status"),
                "duplicate": True,
            }
            return snapshot

        if previous_update is not None and update_count > int(previous_update) + 1:
            # A Pi observation gap always destroys exact second-ring custody: one or
            # more producer-authored second endpoints may have been appended while
            # CLOCKS was absent. Minute custody is narrower. Before the producer
            # minute ring reaches capacity, minute_count is an exact append counter;
            # therefore, if its only increase is the explicitly testified append on
            # this observed row, no minute endpoint was hidden inside the gap. Keep
            # the already-durable minute history in that proved case instead of
            # forcing it to age out over the full 24-hour ring. Once the producer
            # ring is full, count alone cannot distinguish append+evict, so fail
            # closed unless last_minute_key proves there was no append at all.
            previous_expected_minute = int(runtime.get("expected_minute_count") or 0)
            previous_last_minute_key = int(runtime.get("last_minute_key") or 0)
            previous_minute_history = [
                copy.deepcopy(endpoint) for endpoint in runtime["minute_history"]
            ]
            observed_minute_count = int(delta["minute_count"])
            observed_last_minute_key = int(delta["last_minute_key"])
            current_minute_append = bool(delta.get("minute_append_valid"))

            minute_history_preserved = False
            minute_preservation_reason = "UNSEEN_MINUTE_APPEND_NOT_EXCLUDED"
            if previous_expected_minute < PPB_MINUTE_CAPACITY:
                expected_visible_append_count = 1 if current_minute_append else 0
                observed_append_count = observed_minute_count - previous_expected_minute
                minute_history_preserved = bool(
                    observed_append_count == expected_visible_append_count
                    and observed_append_count >= 0
                    and len(previous_minute_history) <= previous_expected_minute
                )
                minute_preservation_reason = (
                    "MINUTE_COUNT_PROVES_NO_HIDDEN_APPEND"
                    if minute_history_preserved
                    else "MINUTE_COUNT_PROVES_OR_ALLOWS_HIDDEN_APPEND"
                )
            elif (
                not current_minute_append
                and observed_minute_count == previous_expected_minute
                and observed_last_minute_key == previous_last_minute_key
            ):
                minute_history_preserved = True
                minute_preservation_reason = "FULL_RING_KEY_PROVES_NO_APPEND"

            gap = {
                "reason": "PI_OBSERVATION_GAP",
                "expected_update_count": int(previous_update) + 1,
                "observed_update_count": update_count,
                "missing_rows": update_count - int(previous_update) - 1,
                "previous_expected_minute_count": previous_expected_minute,
                "observed_minute_count": observed_minute_count,
                "previous_last_minute_key": previous_last_minute_key,
                "observed_last_minute_key": observed_last_minute_key,
                "current_minute_append": current_minute_append,
                "minute_history_preserved": minute_history_preserved,
                "minute_preservation_reason": minute_preservation_reason,
            }
            prior_gap_count = int(runtime.get("gap_count") or 0) + 1
            _ppb_checkpoint_runtime = _ppb_checkpoint_new_runtime(
                reason="PI_OBSERVATION_GAP",
                gap_count=prior_gap_count,
                last_gap=gap,
            )
            runtime = _ppb_checkpoint_runtime
            if minute_history_preserved:
                runtime["minute_history"].extend(previous_minute_history)
            _diag["ppb_checkpoint_gap_count"] = (
                _diag.get("ppb_checkpoint_gap_count", 0) + 1
            )

        # A producer ring count may decrease only when Alpha's rolling state has
        # been replaced. If our saved image is larger than the producer's current
        # ring, discard it and warm again from authoritative appends.
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
            prior_gap_count = int(runtime.get("gap_count") or 0)
            _ppb_checkpoint_runtime = _ppb_checkpoint_new_runtime(
                reason="PRODUCER_RING_REPLACED",
                gap_count=prior_gap_count,
                last_gap=replacement,
            )
            runtime = _ppb_checkpoint_runtime

        if runtime.get("contiguous_from_update_count") is None:
            runtime["contiguous_from_update_count"] = update_count

        if delta.get("second_append_valid"):
            endpoint = copy.deepcopy(delta["second_append"])
            history = runtime["second_history"]
            if history and int(endpoint["rolling_sequence"]) <= int(history[-1]["rolling_sequence"]):
                raise RuntimeError(
                    "Better-Buckets second append is not strictly newer than Pi checkpoint tail"
                )
            history.append(endpoint)

        if delta.get("minute_append_valid"):
            endpoint = copy.deepcopy(delta["minute_append"])
            history = runtime["minute_history"]
            if history and int(endpoint["rolling_sequence"]) <= int(history[-1]["rolling_sequence"]):
                raise RuntimeError(
                    "Better-Buckets minute append is not strictly newer than Pi checkpoint tail"
                )
            history.append(endpoint)

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

        if len(runtime["second_history"]) > int(delta["second_count"]):
            raise RuntimeError("Pi Better-Buckets second history exceeds producer ring count")
        if len(runtime["minute_history"]) > int(delta["minute_count"]):
            raise RuntimeError("Pi Better-Buckets minute history exceeds producer ring count")

        snapshot = _ppb_checkpoint_snapshot_locked(runtime)
        if snapshot.get("recoverable"):
            _ppb_checkpoint_assert_history_matches_proof_locked(runtime, delta)
            runtime["status_reason"] = "COMPLETE_PRODUCER_RING_CUSTODY"
            snapshot = _ppb_checkpoint_snapshot_locked(runtime)
            _diag["ppb_checkpoint_recoverable_rows"] = (
                _diag.get("ppb_checkpoint_recoverable_rows", 0) + 1
            )
        else:
            runtime["status_reason"] = (
                "WAITING_FOR_UNSEEN_PRODUCER_HISTORY_TO_ROLL_OUT"
            )
            snapshot = _ppb_checkpoint_snapshot_locked(runtime)
            _diag["ppb_checkpoint_warming_rows"] = (
                _diag.get("ppb_checkpoint_warming_rows", 0) + 1
            )

        _diag["ppb_checkpoint_rows_verified"] += 1
        _diag["last_ppb_checkpoint"] = {
            "reset_count": reset_count,
            "update_count": update_count,
            "rolling_sequence": snapshot.get("rolling_sequence"),
            "current_sequence": snapshot.get("current_sequence"),
            "recoverable": snapshot.get("recoverable"),
            "status": snapshot.get("status"),
            "second_count": snapshot.get("second_count"),
            "expected_second_count": snapshot.get("expected_second_count"),
            "minute_count": snapshot.get("minute_count"),
            "expected_minute_count": snapshot.get("expected_minute_count"),
            "gap_count": snapshot.get("gap_count"),
        }
        return snapshot


def _normalize_saved_ppb_checkpoint(value: Any) -> Dict[str, Any]:
    """Validate one persisted Pi checkpoint and return canonical scalars/lists."""
    if not isinstance(value, dict) or value.get("schema") != PPB_PI_CHECKPOINT_SCHEMA:
        raise ValueError("canonical CLOCKS missing PI_CLOCKS_PPB_RESTORE_CHECKPOINT_V1")
    if not bool(value.get("valid")):
        raise ValueError("persisted Pi Better-Buckets checkpoint is not valid")

    reset_count = _as_int(value.get("reset_count"))
    update_count = _as_int(value.get("update_count"))
    rolling_sequence = _as_int(value.get("rolling_sequence"))
    current_sequence = _as_int(value.get("current_sequence"))
    expected_second = _as_int(value.get("expected_second_count"))
    expected_minute = _as_int(value.get("expected_minute_count"))
    last_minute_key = _as_int(value.get("last_minute_key"))
    if reset_count is None or reset_count < 0:
        raise ValueError("persisted Pi Better-Buckets checkpoint has invalid reset_count")
    if update_count is None or update_count < 0:
        raise ValueError("persisted Pi Better-Buckets checkpoint has invalid update_count")
    if rolling_sequence is None or rolling_sequence < 0 or rolling_sequence != update_count:
        raise ValueError("persisted Pi Better-Buckets checkpoint has invalid restore rolling_sequence")
    if current_sequence is None or current_sequence < 0 or current_sequence > rolling_sequence:
        raise ValueError("persisted Pi Better-Buckets checkpoint has invalid current_sequence")
    if expected_second is None or not (0 <= expected_second <= PPB_SECOND_CAPACITY):
        raise ValueError("persisted Pi Better-Buckets checkpoint has invalid expected_second_count")
    if expected_minute is None or not (0 <= expected_minute <= PPB_MINUTE_CAPACITY):
        raise ValueError("persisted Pi Better-Buckets checkpoint has invalid expected_minute_count")
    if last_minute_key is None or last_minute_key < 0:
        raise ValueError("persisted Pi Better-Buckets checkpoint has invalid last_minute_key")

    current = _ppb_endpoint_from_payload(value.get("current"), path="ppb_restore_checkpoint.current")
    origin_valid = bool(value.get("origin_valid"))
    origin = (
        _ppb_endpoint_from_payload(value.get("origin"), path="ppb_restore_checkpoint.origin")
        if origin_valid
        else _ppb_zero_endpoint()
    )
    if int(current["rolling_sequence"]) != current_sequence:
        raise ValueError("persisted Pi Better-Buckets current identity mismatch")
    if origin_valid and (
        int(origin["rolling_sequence"]) > current_sequence
        or int(origin["reference_ns"]) > int(current["reference_ns"])
        or int(origin["interval_count"]) > int(current["interval_count"])
    ):
        raise ValueError("persisted Pi Better-Buckets origin follows current state")

    def history(name: str, capacity: int) -> List[Dict[str, Any]]:
        raw_history = value.get(name)
        if not isinstance(raw_history, list) or len(raw_history) > capacity:
            raise ValueError(f"persisted Pi Better-Buckets {name} is invalid")
        out: List[Dict[str, Any]] = []
        previous_sequence = -1
        previous_reference = -1
        previous_intervals = -1
        for index, raw_endpoint in enumerate(raw_history):
            endpoint = _ppb_endpoint_from_payload(
                raw_endpoint, path=f"ppb_restore_checkpoint.{name}[{index}]"
            )
            sequence = int(endpoint["rolling_sequence"])
            reference_ns = int(endpoint["reference_ns"])
            interval_count = int(endpoint["interval_count"])
            if sequence <= previous_sequence:
                raise ValueError(f"persisted Pi Better-Buckets {name} is not chronological")
            if reference_ns < previous_reference or interval_count < previous_intervals:
                raise ValueError(f"persisted Pi Better-Buckets {name} cumulative state regresses")
            if sequence > current_sequence:
                raise ValueError(f"persisted Pi Better-Buckets {name} extends beyond current endpoint")
            previous_sequence = sequence
            previous_reference = reference_ns
            previous_intervals = interval_count
            out.append(endpoint)
        return out

    second_history = history("second_history", PPB_SECOND_CAPACITY)
    minute_history = history("minute_history", PPB_MINUTE_CAPACITY)
    if len(second_history) > expected_second or len(minute_history) > expected_minute:
        raise ValueError("persisted Pi Better-Buckets checkpoint exceeds producer ring counts")
    if rolling_sequence > 0 and second_history and not _ppb_endpoints_equal(second_history[-1], current):
        raise ValueError("persisted Pi Better-Buckets second tail is not current")

    computed_recoverable = bool(
        len(second_history) == expected_second
        and len(minute_history) == expected_minute
        and (
            current_sequence == 0
            and expected_second == 0
            and expected_minute == 0
            and not origin_valid
            or (
                current_sequence > 0
                and origin_valid
                and bool(second_history)
                and _ppb_endpoints_equal(second_history[-1], current)
                and last_minute_key == _ppb_minute_key(current_sequence)
                and (
                    not minute_history
                    or _ppb_minute_key(int(minute_history[-1]["rolling_sequence"]))
                        == last_minute_key
                )
            )
        )
    )
    if bool(value.get("recoverable")) != computed_recoverable:
        raise ValueError(
            "persisted Pi Better-Buckets recoverable flag disagrees with literal history"
        )

    durable_source_detail_id = _as_int(value.get("durable_source_detail_id"))
    if durable_source_detail_id is not None and durable_source_detail_id <= 0:
        raise ValueError(
            "persisted Pi Better-Buckets durable_source_detail_id must be positive"
        )

    return {
        "schema": PPB_PI_CHECKPOINT_SCHEMA,
        "source_schema": PPB_FIRMWARE_DELTA_SCHEMA,
        "valid": True,
        "recoverable": computed_recoverable,
        "status": value.get("status"),
        "status_reason": value.get("status_reason"),
        "reset_count": int(reset_count),
        "update_count": int(update_count),
        "rolling_sequence": int(rolling_sequence),
        "current_sequence": int(current_sequence),
        "last_minute_key": int(last_minute_key),
        "origin_valid": bool(origin_valid),
        "origin": origin,
        "current": current,
        "expected_second_count": int(expected_second),
        "expected_minute_count": int(expected_minute),
        "second_count": len(second_history),
        "minute_count": len(minute_history),
        "second_history": second_history,
        "minute_history": minute_history,
        "contiguous_from_update_count": _as_int(value.get("contiguous_from_update_count")),
        "gap_count": int(_as_int(value.get("gap_count")) or 0),
        "last_gap": copy.deepcopy(value.get("last_gap")),
        "seeded_from_durable": bool(value.get("seeded_from_durable")),
        "seed_source_db_detail_id": _as_int(value.get("seed_source_db_detail_id")),
        "durable_source_detail_id": durable_source_detail_id,
        "last_delta_signature": value.get("last_delta_signature"),
        "proof_checks": int(_as_int(value.get("proof_checks")) or 0),
    }


def _restore_ppb_checkpoint_runtime(
    checkpoint: Dict[str, Any],
    *,
    source_db_detail_id: Optional[int],
) -> Dict[str, Any]:
    """Install one persisted literal checkpoint into Pi process custody."""
    global _ppb_checkpoint_runtime
    saved = _normalize_saved_ppb_checkpoint(checkpoint)
    with _ppb_checkpoint_lock:
        runtime = _ppb_checkpoint_new_runtime(
            reason="RESTORED_FROM_DURABLE_CLOCKS",
            gap_count=int(saved.get("gap_count") or 0),
            last_gap=saved.get("last_gap") if isinstance(saved.get("last_gap"), dict) else None,
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
        runtime["contiguous_from_update_count"] = saved.get("contiguous_from_update_count")
        runtime["seeded_from_durable"] = True
        runtime["seed_source_db_detail_id"] = (
            int(source_db_detail_id) if source_db_detail_id is not None else None
        )
        runtime["proof_checks"] = int(saved.get("proof_checks") or 0)
        _ppb_checkpoint_runtime = runtime
        return _ppb_checkpoint_snapshot_locked(runtime)


def _write_clocks_recovery_config(
    cur: Any, checkpoint: Dict[str, Any], *, source_detail_id: int
) -> None:
    """Persist the literal CLOCKS recovery image and its exact durable owner."""
    detail_id = int(source_detail_id)
    if detail_id <= 0:
        raise ValueError("CLOCKS recovery source_detail_id must be positive")
    normalized = _normalize_saved_ppb_checkpoint(checkpoint)
    normalized["durable_source_detail_id"] = detail_id
    encoded = json.dumps(normalized, separators=(",", ":"), ensure_ascii=False)
    cur.execute(
        "UPDATE config SET payload = %s::jsonb WHERE config_key = %s",
        (encoded, CLOCKS_RECOVERY_CONFIG_KEY),
    )
    if cur.rowcount > 1:
        raise RuntimeError(
            f"config.{CLOCKS_RECOVERY_CONFIG_KEY} is not a singleton: rows={cur.rowcount}"
        )
    if cur.rowcount == 0:
        cur.execute(
            "INSERT INTO config (config_key, payload) VALUES (%s, %s::jsonb)",
            (CLOCKS_RECOVERY_CONFIG_KEY, encoded),
        )
        if cur.rowcount != 1:
            raise RuntimeError(
                f"config.{CLOCKS_RECOVERY_CONFIG_KEY} insert did not create exactly one row"
            )


def _read_clocks_recovery_config() -> Optional[Dict[str, Any]]:
    """Read the singleton CLOCKS recovery image, or None before first custody."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            "SELECT payload FROM config WHERE config_key = %s",
            (CLOCKS_RECOVERY_CONFIG_KEY,),
        )
        row = cur.fetchone()
    if row is None:
        return None
    payload = row["payload"]
    if isinstance(payload, str):
        payload = json.loads(payload)
    return _normalize_saved_ppb_checkpoint(payload)


def _retire_orphaned_clocks_recovery_config_if_domain_empty() -> Dict[str, Any]:
    """Retire stale singleton custody only when the durable TEMPEST domain is empty.

    ``config.CLOCKS_RECOVERY`` is meaningful only while its owning canonical
    CLOCKS row still exists.  An operator-level truncation can lawfully remove
    every TEMPEST master/detail while leaving config rows intact.  Treat that
    exact zero/zero topology as a new durable epoch, not as corrupted ancestry.

    Any nonempty TEMPEST history remains fail-closed: this helper does not delete
    or reinterpret recovery custody when even one master or detail survives.
    """
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT
                (SELECT COUNT(*) FROM campaign_master WHERE campaign_type = %s) AS master_count,
                (SELECT COUNT(*) FROM campaign_detail WHERE campaign_type = %s) AS detail_count
            """,
            (CAMPAIGN_TYPE_TEMPEST, CAMPAIGN_TYPE_TEMPEST),
        )
        row = cur.fetchone()
        if not isinstance(row, dict):
            raise RuntimeError("CLOCKS empty-domain court returned no count row")

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
            (CLOCKS_RECOVERY_CONFIG_KEY,),
        )
        deleted = int(cur.rowcount or 0)
        if deleted > 1:
            raise RuntimeError(
                f"config.{CLOCKS_RECOVERY_CONFIG_KEY} is not a singleton: rows={deleted}"
            )

    return {
        "domain_empty": True,
        "master_count": 0,
        "detail_count": 0,
        "recovery_config_deleted": bool(deleted),
    }


def _clocks_checkpoint_receipt_summary(checkpoint: Dict[str, Any]) -> Dict[str, Any]:
    """Fingerprint one exact private CLOCKS checkpoint without copying its rings."""
    saved = _normalize_saved_ppb_checkpoint(checkpoint)
    encoded = json.dumps(
        saved, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return {
        "schema": saved.get("schema"),
        "sha256": hashlib.sha256(encoded).hexdigest(),
        "durable_source_detail_id": _as_int(saved.get("durable_source_detail_id")),
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


def _find_clocks_exact_recovery_boundary_detail_id(
    *,
    source_detail_id: int,
    source_reset_count: int,
    source_update_count: int,
    proof_sequence: int,
) -> int:
    """Find the one durable exact-N+1 CLOCKS row admitted after a restore."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id
            FROM campaign_detail
            WHERE campaign_type = %s
              AND id > %s
              AND payload #>> '{schema}' = 'CLOCKS_V4'
              AND payload #>> '{sequence}' = %s
              AND payload #>> '{clocks,stats,reset_count}' = %s
              AND payload #>> '{clocks,stats,update_count}' = %s
            ORDER BY id ASC
            LIMIT 2
            """,
            (
                CAMPAIGN_TYPE_TEMPEST,
                int(source_detail_id),
                str(int(proof_sequence)),
                str(int(source_reset_count)),
                str(int(source_update_count) + 1),
            ),
        )
        rows = cur.fetchall()
    if len(rows) != 1:
        raise RuntimeError(
            "exact CLOCKS recovery boundary is not unique: "
            f"source_detail_id={source_detail_id} reset={source_reset_count} "
            f"update={source_update_count}->{source_update_count + 1} "
            f"sequence={proof_sequence} rows={len(rows)}"
        )
    return int(rows[0]["id"])


def _record_clocks_recovery_receipt(
    *,
    boundary_detail_id: int,
    source_detail: Dict[str, Any],
    source_checkpoint: Dict[str, Any],
    recovery_mode: str,
    proof: Dict[str, Any],
    alpha_proof_detail_id: Optional[int] = None,
    campaign_name: Optional[str] = None,
    campaign_boundary_detail_id: Optional[int] = None,
    source_public_count: Optional[int] = None,
    expected_first_public_count: Optional[int] = None,
    elapsed_seconds: Optional[int] = None,
) -> Dict[str, Any]:
    """Attach one immutable Pi recovery receipt to the authoritative CLOCKS boundary.

    The receipt is deliberately compact.  It fingerprints the exact private
    config.CLOCKS_RECOVERY image used for resurrection, but never copies the
    bounded endpoint rings back into canonical CLOCKS.
    """
    source_id = _as_int(source_detail.get("_db_detail_id"))
    if source_id is None or source_id <= 0:
        raise RuntimeError("CLOCKS recovery receipt source lacks durable detail identity")
    source_sequence = _as_int(source_detail.get("sequence"))
    source_clocks = _clocks_payload(source_detail)
    source_stats = source_clocks.get("stats") if isinstance(source_clocks, dict) else None
    source_reset = _as_int(source_stats.get("reset_count")) if isinstance(source_stats, dict) else None
    source_update = _as_int(source_stats.get("update_count")) if isinstance(source_stats, dict) else None
    if source_sequence is None or source_reset is None or source_update is None:
        raise RuntimeError("CLOCKS recovery receipt source lacks chronology")
    if proof.get("proved") is not True or proof.get("durable") is not True:
        raise RuntimeError("CLOCKS recovery receipt requires a proved durable Alpha successor")

    checkpoint_summary = _clocks_checkpoint_receipt_summary(source_checkpoint)
    if checkpoint_summary.get("durable_source_detail_id") not in (None, int(source_id)):
        raise RuntimeError(
            "CLOCKS recovery receipt checkpoint owner disagrees with source: "
            f"checkpoint={checkpoint_summary.get('durable_source_detail_id')} source={source_id}"
        )
    if (
        int(checkpoint_summary["reset_count"]) != int(source_reset)
        or int(checkpoint_summary["update_count"]) != int(source_update)
    ):
        raise RuntimeError("CLOCKS recovery receipt checkpoint chronology disagrees with source")

    boundary_id = int(boundary_detail_id)
    if boundary_id <= source_id:
        raise RuntimeError("CLOCKS recovery receipt boundary does not follow source")

    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            "SELECT payload FROM campaign_detail WHERE id = %s AND campaign_type = %s FOR UPDATE",
            (boundary_id, CAMPAIGN_TYPE_TEMPEST),
        )
        row = cur.fetchone()
        if row is None:
            raise RuntimeError(
                f"CLOCKS recovery receipt boundary detail_id={boundary_id} is missing"
            )
        boundary = row["payload"]
        if isinstance(boundary, str):
            boundary = json.loads(boundary)
        if not isinstance(boundary, dict) or boundary.get("schema") != "CLOCKS_V4":
            raise RuntimeError("CLOCKS recovery receipt boundary is not CLOCKS_V4")

        boundary_clocks = _clocks_payload(boundary)
        boundary_stats = boundary_clocks.get("stats") if isinstance(boundary_clocks, dict) else None
        boundary_sequence = _as_int(boundary.get("sequence"))
        boundary_reset = _as_int(boundary_stats.get("reset_count")) if isinstance(boundary_stats, dict) else None
        boundary_update = _as_int(boundary_stats.get("update_count")) if isinstance(boundary_stats, dict) else None
        if (
            boundary_sequence is None
            or boundary_reset != source_reset
            or boundary_update is None
            or boundary_update < source_update + 1
            or boundary.get("holistic_restore_superseded") is True
        ):
            raise RuntimeError(
                "CLOCKS recovery receipt resumed boundary is not authoritative descendant: "
                f"source={source_reset}/{source_update} "
                f"boundary={boundary_reset}/{boundary_update} sequence={boundary_sequence} "
                f"superseded={boundary.get('holistic_restore_superseded')}"
            )

        durable_proof_sequence = _as_int(proof.get("durable_proof_sequence"))
        if durable_proof_sequence is None or durable_proof_sequence <= 0:
            raise RuntimeError("CLOCKS recovery receipt proof lacks durable Alpha sequence")
        alpha_detail_id = (
            int(alpha_proof_detail_id)
            if alpha_proof_detail_id is not None
            else boundary_id
        )
        if alpha_detail_id > boundary_id:
            raise RuntimeError("CLOCKS Alpha proof row follows the resumed authority boundary")
        if alpha_detail_id == boundary_id:
            alpha_boundary = boundary
        else:
            cur.execute(
                "SELECT payload FROM campaign_detail WHERE id = %s AND campaign_type = %s",
                (alpha_detail_id, CAMPAIGN_TYPE_TEMPEST),
            )
            alpha_row = cur.fetchone()
            if alpha_row is None:
                raise RuntimeError(
                    f"CLOCKS exact Alpha proof detail_id={alpha_detail_id} is missing"
                )
            alpha_boundary = alpha_row["payload"]
            if isinstance(alpha_boundary, str):
                alpha_boundary = json.loads(alpha_boundary)
            if not isinstance(alpha_boundary, dict):
                raise RuntimeError("CLOCKS exact Alpha proof payload is not an object")
        alpha_clocks = _clocks_payload(alpha_boundary)
        alpha_stats = alpha_clocks.get("stats") if isinstance(alpha_clocks, dict) else None
        alpha_sequence = _as_int(alpha_boundary.get("sequence"))
        alpha_reset = _as_int(alpha_stats.get("reset_count")) if isinstance(alpha_stats, dict) else None
        alpha_update = _as_int(alpha_stats.get("update_count")) if isinstance(alpha_stats, dict) else None
        if not (
            alpha_sequence == durable_proof_sequence
            and alpha_reset == source_reset
            and alpha_update == source_update + 1
        ):
            raise RuntimeError(
                "CLOCKS recovery receipt exact Alpha proof does not close source N+1: "
                f"source={source_reset}/{source_update} "
                f"alpha={alpha_reset}/{alpha_update} sequence={alpha_sequence} "
                f"expected_sequence={durable_proof_sequence}"
            )

        campaign_receipt: Optional[Dict[str, Any]] = None
        if campaign_name is not None:
            campaign_boundary_id = (
                int(campaign_boundary_detail_id)
                if campaign_boundary_detail_id is not None
                else boundary_id
            )
            if campaign_boundary_id < boundary_id:
                raise RuntimeError(
                    "CLOCKS recovery receipt campaign boundary precedes Alpha boundary"
                )
            if campaign_boundary_id == boundary_id:
                campaign_boundary = boundary
            else:
                cur.execute(
                    "SELECT payload FROM campaign_detail WHERE id = %s AND campaign_type = %s",
                    (campaign_boundary_id, CAMPAIGN_TYPE_TEMPEST),
                )
                campaign_row = cur.fetchone()
                if campaign_row is None:
                    raise RuntimeError(
                        "CLOCKS recovery receipt first-public detail is missing: "
                        f"detail_id={campaign_boundary_id}"
                    )
                campaign_boundary = campaign_row["payload"]
                if isinstance(campaign_boundary, str):
                    campaign_boundary = json.loads(campaign_boundary)
                if not isinstance(campaign_boundary, dict):
                    raise RuntimeError("CLOCKS recovery first-public payload is not an object")

            campaign = _state_campaign(campaign_boundary)
            boundary_campaign = _tempest_campaign_name(campaign) if isinstance(campaign, dict) else ""
            actual_public = (
                _tempest_public_count(campaign)
                if isinstance(campaign, dict) and boundary_campaign
                else 0
            )
            if boundary_campaign != str(campaign_name):
                raise RuntimeError(
                    "CLOCKS recovery receipt boundary campaign mismatch: "
                    f"expected={campaign_name!r} observed={boundary_campaign!r}"
                )
            if (
                expected_first_public_count is not None
                and actual_public != int(expected_first_public_count)
            ):
                raise RuntimeError(
                    "CLOCKS recovery receipt first-public mismatch: "
                    f"expected={expected_first_public_count} actual={actual_public}"
                )
            campaign_receipt = {
                "campaign": str(campaign_name),
                "first_public_detail_id": campaign_boundary_id,
                "source_public_count": (
                    int(source_public_count) if source_public_count is not None else None
                ),
                "expected_first_public_count": (
                    int(expected_first_public_count)
                    if expected_first_public_count is not None
                    else None
                ),
                "actual_first_public_count": int(actual_public),
                "elapsed_seconds": int(elapsed_seconds) if elapsed_seconds is not None else None,
            }

        receipt: Dict[str, Any] = {
            "schema": CLOCKS_RECOVERY_RECEIPT_SCHEMA,
            "subsystem": "CLOCKS",
            "recorded_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "recovery_mode": str(recovery_mode).strip().upper(),
            "producer_transition": "RESURRECTED",
            "source": {
                "detail_id": int(source_id),
                "sequence": int(source_sequence),
                "reset_count": int(source_reset),
                "update_count": int(source_update),
            },
            "checkpoint": checkpoint_summary,
            "boundary": {
                "detail_id": boundary_id,
                "sequence": int(boundary_sequence),
                "reset_count": int(boundary_reset),
                "update_count": int(boundary_update),
            },
            "proof": {
                "contract": "EXACT_SOURCE_N_PLUS_1",
                "durable": bool(proof.get("durable")),
                "proved": bool(proof.get("proved")),
                "alpha_detail_id": int(alpha_detail_id),
                "alpha_sequence": int(alpha_sequence),
                "alpha_reset_count": int(alpha_reset),
                "alpha_update_count": int(alpha_update),
            },
        }
        if campaign_receipt is not None:
            receipt["campaign"] = campaign_receipt

        existing = boundary.get("recovery_receipt")
        if existing is not None:
            if not isinstance(existing, dict):
                raise RuntimeError("CLOCKS recovery boundary has malformed recovery_receipt")
            same_identity = bool(
                existing.get("schema") == CLOCKS_RECOVERY_RECEIPT_SCHEMA
                and _as_int(_path_get(existing, "source.detail_id")) == source_id
                and _as_int(_path_get(existing, "boundary.detail_id")) == boundary_id
                and str(existing.get("recovery_mode") or "").strip().upper()
                == str(recovery_mode).strip().upper()
            )
            if not same_identity:
                raise RuntimeError(
                    f"CLOCKS recovery boundary detail_id={boundary_id} already has a different receipt"
                )
            return copy.deepcopy(existing)

        cur.execute(
            """
            UPDATE campaign_detail
            SET payload = jsonb_set(payload, '{recovery_receipt}', %s::jsonb, true)
            WHERE id = %s AND campaign_type = %s
            """,
            (
                json.dumps(receipt, separators=(",", ":"), ensure_ascii=False),
                boundary_id,
                CAMPAIGN_TYPE_TEMPEST,
            ),
        )
        if cur.rowcount != 1:
            raise RuntimeError(
                f"CLOCKS recovery receipt did not decorate exactly one boundary row: {boundary_id}"
            )

    logging.info(
        "🧾 [recovery] durable CLOCKS recovery receipt recorded: mode=%s source_id=%d "
        "boundary_id=%d checkpoint_sha256=%s",
        str(recovery_mode).strip().upper(),
        int(source_id),
        boundary_id,
        checkpoint_summary["sha256"],
    )
    return receipt


def _write_alpha_lineage_cutoff_config(
    cur: Any,
    *,
    cutoff_detail_id: int,
    terminated_at_utc: str,
    reason: str,
    source: str,
    campaign: Optional[str],
) -> Dict[str, Any]:
    """Persist one O(1) boundary retiring every prior TEMPEST row as restore authority."""
    cutoff = int(cutoff_detail_id)
    if cutoff < 0:
        raise ValueError("Alpha lineage cutoff detail_id may not be negative")
    marker = {
        "schema": CLOCKS_ALPHA_LINEAGE_CUTOFF_SCHEMA,
        "cutoff_detail_id": cutoff,
        "terminated_at_utc": str(terminated_at_utc),
        "reason": str(reason),
        "source": str(source),
        "campaign": str(campaign) if campaign else None,
    }
    encoded = json.dumps(marker, separators=(",", ":"), ensure_ascii=False)
    cur.execute(
        "UPDATE config SET payload = %s::jsonb WHERE config_key = %s",
        (encoded, CLOCKS_ALPHA_LINEAGE_CUTOFF_CONFIG_KEY),
    )
    if cur.rowcount > 1:
        raise RuntimeError(
            f"config.{CLOCKS_ALPHA_LINEAGE_CUTOFF_CONFIG_KEY} is not a singleton: "
            f"rows={cur.rowcount}"
        )
    if cur.rowcount == 0:
        cur.execute(
            "INSERT INTO config (config_key, payload) VALUES (%s, %s::jsonb)",
            (CLOCKS_ALPHA_LINEAGE_CUTOFF_CONFIG_KEY, encoded),
        )
        if cur.rowcount != 1:
            raise RuntimeError(
                "Alpha lineage cutoff insert did not create exactly one config row"
            )
    return marker


def _read_alpha_lineage_cutoff_config() -> Optional[Dict[str, Any]]:
    """Return the durable global TEMPEST restore-authority cutoff, if one exists."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            "SELECT payload FROM config WHERE config_key = %s",
            (CLOCKS_ALPHA_LINEAGE_CUTOFF_CONFIG_KEY,),
        )
        row = cur.fetchone()
    if row is None:
        return None
    payload = row["payload"]
    if isinstance(payload, str):
        payload = json.loads(payload)
    if not isinstance(payload, dict) or payload.get("schema") != CLOCKS_ALPHA_LINEAGE_CUTOFF_SCHEMA:
        raise RuntimeError(
            f"config.{CLOCKS_ALPHA_LINEAGE_CUTOFF_CONFIG_KEY} is malformed"
        )
    cutoff = _as_int(payload.get("cutoff_detail_id"))
    if cutoff is None or cutoff < 0:
        raise RuntimeError(
            f"config.{CLOCKS_ALPHA_LINEAGE_CUTOFF_CONFIG_KEY} has invalid cutoff_detail_id"
        )
    result = copy.deepcopy(payload)
    result["cutoff_detail_id"] = int(cutoff)
    return result


def _alpha_lineage_cutoff_detail_id() -> int:
    marker = _read_alpha_lineage_cutoff_config()
    return int(marker.get("cutoff_detail_id") or 0) if marker is not None else 0


def _legacy_clocks_recovery_checkpoint() -> Optional[Dict[str, Any]]:
    """One-time migration bridge from pre-singleton CLOCKS rows."""
    cutoff_detail_id = _alpha_lineage_cutoff_detail_id()
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, campaign, payload
            FROM campaign_detail
            WHERE campaign_type = %s
              AND id > %s
              AND NOT (payload @> '{"holistic_restore_superseded":true}'::jsonb)
              AND payload #>> '{schema}' = 'CLOCKS_V4'
              AND payload #> '{clocks,ppb_restore_checkpoint}' IS NOT NULL
            ORDER BY id DESC
            LIMIT 64
            """,
            (CAMPAIGN_TYPE_TEMPEST, int(cutoff_detail_id)),
        )
        rows = cur.fetchall()

    for row in rows:
        state = row["payload"]
        if isinstance(state, str):
            state = json.loads(state)
        if not isinstance(state, dict):
            continue
        clocks = _clocks_payload(state)
        checkpoint = clocks.get("ppb_restore_checkpoint")
        if not isinstance(checkpoint, dict):
            continue
        if not _canonical_instrument_restore_ready(
            clocks, include_control=True, detail_id=int(row["id"])
        ):
            continue
        if _clocks_gnss_raw_payload(state) is None:
            continue
        normalized = _normalize_saved_ppb_checkpoint(checkpoint)
        source_detail_id = int(row["id"])
        with open_db() as conn:
            _write_clocks_recovery_config(
                conn.cursor(), normalized, source_detail_id=source_detail_id
            )
        normalized["durable_source_detail_id"] = source_detail_id
        logging.warning(
            "📦 [clocks/ppb] migrated legacy embedded Better-Buckets checkpoint "
            "from detail_id=%d to config.%s",
            int(row["id"]),
            CLOCKS_RECOVERY_CONFIG_KEY,
        )
        return normalized
    return None


def _load_or_migrate_clocks_recovery_checkpoint() -> Optional[Dict[str, Any]]:
    empty_domain = _retire_orphaned_clocks_recovery_config_if_domain_empty()
    if empty_domain.get("domain_empty"):
        logging.warning(
            "🧹 [clocks/ppb] durable TEMPEST domain is empty "
            "(campaign_master=0 campaign_detail=0); retired orphaned config.%s=%s "
            "and starting a fresh CLOCKS durability epoch",
            CLOCKS_RECOVERY_CONFIG_KEY,
            bool(empty_domain.get("recovery_config_deleted")),
        )
        return None

    checkpoint = _read_clocks_recovery_config()
    if checkpoint is None:
        return _legacy_clocks_recovery_checkpoint()
    if _as_int(checkpoint.get("durable_source_detail_id")) is not None:
        return checkpoint

    # Transitional repair for the first singleton implementation, which wrote the
    # complete recovery block but did not preserve the exact campaign_detail owner.
    # Reuse the already-bounded legacy embedded-row migration once; never search
    # historical JSONB statistics to rediscover an identity the writer already knew.
    logging.warning(
        "📦 [clocks/ppb] config.%s lacks durable_source_detail_id; "
        "upgrading from the newest legacy embedded checkpoint",
        CLOCKS_RECOVERY_CONFIG_KEY,
    )
    migrated = _legacy_clocks_recovery_checkpoint()
    if migrated is None:
        raise RuntimeError(
            f"config.{CLOCKS_RECOVERY_CONFIG_KEY} lacks durable_source_detail_id and "
            "no legacy embedded CLOCKS checkpoint remains to repair it"
        )
    return migrated


def _clocks_recovery_source_row(checkpoint: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """Return the exact durable CLOCKS row named by the recovery singleton."""
    saved = _normalize_saved_ppb_checkpoint(checkpoint)
    source_detail_id = _as_int(saved.get("durable_source_detail_id"))
    if source_detail_id is None or source_detail_id <= 0:
        raise RuntimeError(
            f"config.{CLOCKS_RECOVERY_CONFIG_KEY} lacks durable_source_detail_id"
        )
    cutoff_detail_id = _alpha_lineage_cutoff_detail_id()
    if cutoff_detail_id and int(source_detail_id) <= int(cutoff_detail_id):
        raise RuntimeError(
            f"config.{CLOCKS_RECOVERY_CONFIG_KEY} source detail_id={source_detail_id} "
            f"belongs to terminated Alpha lineage cutoff<={cutoff_detail_id}"
        )
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, campaign, payload
            FROM campaign_detail
            WHERE id = %s
              AND campaign_type = %s
            """,
            (int(source_detail_id), CAMPAIGN_TYPE_TEMPEST),
        )
        row = cur.fetchone()
    if row is None:
        return None
    state = row["payload"]
    if isinstance(state, str):
        state = json.loads(state)
    if not isinstance(state, dict) or state.get("schema") != "CLOCKS_V4":
        raise RuntimeError(
            f"config.{CLOCKS_RECOVERY_CONFIG_KEY} source detail_id={source_detail_id} "
            "is not a CLOCKS_V4 payload"
        )
    if state.get("holistic_restore_superseded") is True:
        raise RuntimeError(
            f"config.{CLOCKS_RECOVERY_CONFIG_KEY} source detail_id={source_detail_id} "
            "is superseded"
        )
    if not _checkpoint_matches_clocks_state(saved, state):
        raise RuntimeError(
            f"config.{CLOCKS_RECOVERY_CONFIG_KEY} source detail_id={source_detail_id} "
            "does not own the checkpoint statistics identity"
        )
    return copy.deepcopy(row)


def _checkpoint_matches_clocks_state(
    checkpoint: Dict[str, Any], state: Dict[str, Any]
) -> bool:
    stats = _clocks_payload(state).get("stats")
    if not isinstance(checkpoint, dict) or not isinstance(stats, dict):
        return False
    return bool(
        _as_int(stats.get("reset_count")) == _as_int(checkpoint.get("reset_count"))
        and _as_int(stats.get("update_count")) == _as_int(checkpoint.get("update_count"))
        and _as_int(stats.get("rolling_ppb_current_sequence"))
        == _as_int(checkpoint.get("current_sequence"))
    )


def _clocks_state_owns_recovery_config(
    state: Dict[str, Any], checkpoint: Dict[str, Any]
) -> bool:
    """True only when this durable observation is lawful singleton authority."""
    if state.get("holistic_restore_superseded") is True:
        return False
    if not _checkpoint_matches_clocks_state(checkpoint, state):
        raise RuntimeError(
            "CLOCKS recovery checkpoint identity does not match canonical observation"
        )
    clocks = _clocks_payload(state)
    if not _canonical_instrument_restore_ready(clocks, include_control=False):
        return False
    ancestry = _dac_restore_population_ancestry_court(clocks)
    if not ancestry.get("available") or not ancestry.get("valid"):
        return False
    control = clocks.get("control")
    if not isinstance(control, dict) or control.get("schema") != "CLOCKS_CONTROL_V1":
        return False
    return _clocks_gnss_raw_payload(state) is not None


def _seed_ppb_checkpoint_runtime_from_latest_durable() -> Dict[str, Any]:
    """Resume Pi checkpoint custody from config.CLOCKS_RECOVERY; never replay history."""
    global _ppb_checkpoint_runtime
    checkpoint = _load_or_migrate_clocks_recovery_checkpoint()
    if checkpoint is None:
        with _ppb_checkpoint_lock:
            _ppb_checkpoint_runtime = _ppb_checkpoint_new_runtime(
                reason="NO_DURABLE_CHECKPOINT_SEED"
            )
            snapshot = _ppb_checkpoint_snapshot_locked(_ppb_checkpoint_runtime)
        _diag["ppb_checkpoint_seed_missing"] = (
            _diag.get("ppb_checkpoint_seed_missing", 0) + 1
        )
        logging.warning(
            "📦 [clocks/ppb] config.%s does not exist yet; Pi will warm from "
            "producer-authored endpoint appends without SQL replay",
            CLOCKS_RECOVERY_CONFIG_KEY,
        )
        return snapshot

    row = _clocks_recovery_source_row(checkpoint)
    if row is None:
        raise RuntimeError(
            f"config.{CLOCKS_RECOVERY_CONFIG_KEY} has no matching durable CLOCKS row"
        )
    source_detail_id = int(row["id"])
    snapshot = _restore_ppb_checkpoint_runtime(
        checkpoint, source_db_detail_id=source_detail_id
    )
    _diag["ppb_checkpoint_seed_loaded"] = (
        _diag.get("ppb_checkpoint_seed_loaded", 0) + 1
    )
    logging.info(
        "📦 [clocks/ppb] restored Pi Better-Buckets checkpoint custody from "
        "config.%s aligned to detail_id=%d update_count=%s recoverable=%s "
        "second=%d/%d minute=%d/%d",
        CLOCKS_RECOVERY_CONFIG_KEY,
        source_detail_id,
        snapshot.get("update_count"),
        snapshot.get("recoverable"),
        int(snapshot.get("second_count") or 0),
        int(snapshot.get("expected_second_count") or 0),
        int(snapshot.get("minute_count") or 0),
        int(snapshot.get("expected_minute_count") or 0),
    )
    return snapshot


def _ppb_restore_checkpoint_from_clocks(clocks: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    checkpoint = clocks.get("ppb_restore_checkpoint") if isinstance(clocks, dict) else None
    return copy.deepcopy(checkpoint) if isinstance(checkpoint, dict) else None


def _require_recoverable_ppb_checkpoint(clocks: Dict[str, Any]) -> Dict[str, Any]:
    checkpoint = _ppb_restore_checkpoint_from_clocks(clocks)
    saved = _normalize_saved_ppb_checkpoint(checkpoint)
    if not saved.get("recoverable"):
        raise RuntimeError(
            "canonical CLOCKS Better-Buckets checkpoint is not yet recoverable; "
            f"status={saved.get('status')!r} "
            f"second={saved.get('second_count')}/{saved.get('expected_second_count')} "
            f"minute={saved.get('minute_count')}/{saved.get('expected_minute_count')} "
            "raw PostgreSQL replay is intentionally unavailable"
        )
    return saved


def _require_alpha_resurrection_checkpoint(
    clocks: Dict[str, Any],
    *,
    campaign: str,
    recovery_source_db_id: Optional[int],
) -> Dict[str, Any]:
    """Require exact durable Alpha state after live physical custody was lost.

    Missing producer-authored endpoints make *that lineage* impossible to resurrect,
    not the current CLOCKS substrate impossible to operate. Preserve the proof and
    raise a typed lineage verdict so the lifecycle owner can terminate old ancestry
    and establish a fresh Alpha without weakening this court or replaying SQL history.
    """
    checkpoint = _ppb_restore_checkpoint_from_clocks(clocks)
    try:
        saved = _normalize_saved_ppb_checkpoint(checkpoint)
    except (TypeError, ValueError) as exc:
        details = {
            "court": "ALPHA_DURABLE_RESURRECTION_AUTHORITY",
            "campaign": str(campaign),
            "recovery_source_db_id": recovery_source_db_id,
            "checkpoint_schema": (
                checkpoint.get("schema") if isinstance(checkpoint, dict) else None
            ),
            "checkpoint_error": str(exc),
            "physical_custody": _recovery_custody_snapshot(),
            "raw_postgresql_replay_available": False,
            "operator_action": (
                "Exact Alpha resurrection is impossible from the available durable "
                "state. Preserve the old observations as historical evidence, retire "
                "their restore authority, and establish a fresh Alpha epoch."
            ),
        }
        _clear_sync_wait()
        logging.critical(
            "🧭 [recovery] ALPHA RESURRECTION IMPOSSIBLE: durable Better-Buckets "
            "checkpoint is missing or invalid after physical custody loss; "
            "campaign=%s source_detail_id=%s error=%s. Exact old-lineage recovery "
            "is refused; lifecycle will surrender that ancestry and birth a new Alpha.",
            campaign,
            recovery_source_db_id,
            exc,
        )
        raise AlphaResurrectionImpossible(
            "alpha_resurrection_impossible_checkpoint_invalid",
            details,
        ) from exc

    if saved.get("recoverable"):
        return saved

    second_count = int(saved.get("second_count") or 0)
    expected_second = int(saved.get("expected_second_count") or 0)
    minute_count = int(saved.get("minute_count") or 0)
    expected_minute = int(saved.get("expected_minute_count") or 0)
    details = {
        "court": "ALPHA_DURABLE_RESURRECTION_AUTHORITY",
        "campaign": str(campaign),
        "recovery_source_db_id": recovery_source_db_id,
        "checkpoint_schema": saved.get("schema"),
        "checkpoint_status": saved.get("status"),
        "checkpoint_status_reason": saved.get("status_reason"),
        "checkpoint_reset_count": saved.get("reset_count"),
        "checkpoint_update_count": saved.get("update_count"),
        "second_count": second_count,
        "expected_second_count": expected_second,
        "missing_second_endpoints": max(0, expected_second - second_count),
        "minute_count": minute_count,
        "expected_minute_count": expected_minute,
        "missing_minute_endpoints": max(0, expected_minute - minute_count),
        "gap_count": int(saved.get("gap_count") or 0),
        "last_gap": copy.deepcopy(saved.get("last_gap")),
        "physical_custody": _recovery_custody_snapshot(),
        "raw_postgresql_replay_available": False,
        "information_loss_is_terminal": True,
        "operator_action": (
            "The previous Alpha RAM lifetime is gone and the durable checkpoint does "
            "not contain every producer-authored endpoint required for exact restore. "
            "Do not reconstruct it. Terminate that restore lineage and establish a "
            "fresh Alpha epoch while preserving the old observations as history."
        ),
    }
    _clear_sync_wait()
    logging.critical(
        "🧭 [recovery] ALPHA RESURRECTION IMPOSSIBLE: physical Teensy custody was "
        "lost and the durable Better-Buckets checkpoint is incomplete "
        "(status=%s second=%d/%d missing=%d minute=%d/%d missing=%d). "
        "The missing producer-authored endpoints no longer exist; raw PostgreSQL "
        "reconstruction is prohibited. Exact old-lineage recovery is refused; "
        "lifecycle will surrender that ancestry and birth a new Alpha.",
        saved.get("status"),
        second_count,
        expected_second,
        max(0, expected_second - second_count),
        minute_count,
        expected_minute,
        max(0, expected_minute - minute_count),
    )
    raise AlphaResurrectionImpossible(
        "alpha_resurrection_impossible_checkpoint_incomplete",
        details,
    )


def _supersede_dead_producer_restore_rows(
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


def _stage_teensy_better_buckets_checkpoint(checkpoint: Dict[str, Any]) -> Dict[str, Any]:
    """Send one literal saved Pi checkpoint back through Alpha's existing restore court."""
    saved = _normalize_saved_ppb_checkpoint(checkpoint)
    if not saved.get("recoverable"):
        raise ValueError("Better-Buckets checkpoint is not complete enough for Alpha restore")

    rolling_sequence = int(saved.get("rolling_sequence") or 0)
    if rolling_sequence == 0:
        return {
            "staged": False,
            "reason": "empty_statistics_epoch",
            "checkpoint_schema": PPB_PI_CHECKPOINT_SCHEMA,
        }

    second_history = saved["second_history"]
    minute_history = saved["minute_history"]
    begin_args: Dict[str, Any] = {
        "rolling_sequence": rolling_sequence,
        "second_count": len(second_history),
        "minute_count": len(minute_history),
        "last_minute_key": int(saved.get("last_minute_key") or 0),
        "origin_valid": bool(saved.get("origin_valid")),
    }
    begin_args.update(_ppb_endpoint_command_args("current", saved["current"]))
    if saved.get("origin_valid"):
        begin_args.update(_ppb_endpoint_command_args("origin", saved["origin"]))

    started = time.monotonic()
    _ppb_restore_transaction_active.set()
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
            "checkpoint_schema": PPB_PI_CHECKPOINT_SCHEMA,
            "rolling_sequence": rolling_sequence,
            "second_count": len(second_history),
            "minute_count": len(minute_history),
            "chunks": chunks,
            "chunk_size": chunk_size,
            "waited_s": round(time.monotonic() - started, 3),
            "teensy": commit_payload,
            "source_reset_count": int(saved.get("reset_count") or 0),
            "source_update_count": int(saved.get("update_count") or 0),
            "source_gap_count": int(saved.get("gap_count") or 0),
            "checkpoint_seed_source_db_detail_id": saved.get("seed_source_db_detail_id"),
        }
    except Exception:
        _abort_teensy_ppb_restore_best_effort()
        raise
    finally:
        _ppb_restore_transaction_active.clear()


def _ppb_export_endpoint_from_payload(payload: Dict[str, Any], prefix: str) -> Dict[str, Any]:
    """Parse one producer-authored endpoint from the read-only Alpha export."""
    required = (
        "reference_ns",
        "dwt_error_cycles",
        "ocxo1_error_ns",
        "ocxo2_error_ns",
        "rolling_sequence",
        "interval_count",
    )
    endpoint: Dict[str, Any] = {}
    for field in required:
        key = f"{prefix}_{field}"
        if key not in payload:
            raise RuntimeError(f"Better-Buckets export endpoint missing {key}")
        value = payload[key]
        try:
            endpoint[field] = (
                float(value) if field == "dwt_error_cycles" else int(value)
            )
        except (TypeError, ValueError) as exc:
            raise RuntimeError(
                f"Better-Buckets export endpoint has invalid {key}={value!r}"
            ) from exc
    return _ppb_endpoint_from_payload(
        endpoint, path=f"ppb_export.{prefix}"
    )


def _fetch_teensy_ppb_export_meta() -> Dict[str, Any]:
    response = send_command(
        machine="TEENSY",
        subsystem="CLOCKS",
        command="PPB_EXPORT_META",
        args={},
    )
    payload = response.get("payload") if isinstance(response, dict) else None
    payload = payload if isinstance(payload, dict) else {}
    if (
        not isinstance(response, dict)
        or not response.get("success")
        or payload.get("status") != "ppb_export_ready"
    ):
        raise RuntimeError(f"Teensy Better-Buckets export META failed: {response!r}")

    required_ints = (
        "reset_count", "update_count", "current_sequence",
        "second_count", "minute_count",
        "second_oldest_sequence", "second_newest_sequence",
        "minute_oldest_sequence", "minute_newest_sequence",
        "last_minute_key",
    )
    meta = dict(payload)
    for key in required_ints:
        value = _as_int(payload.get(key))
        if value is None or value < 0:
            raise RuntimeError(
                f"Teensy Better-Buckets export META has invalid {key}={payload.get(key)!r}"
            )
        meta[key] = int(value)
    if meta["second_count"] > PPB_SECOND_CAPACITY:
        raise RuntimeError("Teensy Better-Buckets export second_count exceeds Pi capacity")
    if meta["minute_count"] > PPB_MINUTE_CAPACITY:
        raise RuntimeError("Teensy Better-Buckets export minute_count exceeds Pi capacity")

    meta["origin_valid"] = _recovery_bool(payload.get("origin_valid"))
    meta["current"] = _ppb_export_endpoint_from_payload(payload, "current")
    meta["origin"] = (
        _ppb_export_endpoint_from_payload(payload, "origin")
        if meta["origin_valid"]
        else _ppb_zero_endpoint()
    )
    meta["chunk_max_endpoints"] = max(
        1, min(
            PPB_RESTORE_CHUNK_ENDPOINTS,
            _as_int(payload.get("chunk_max_endpoints")) or 1,
        )
    )
    if int(meta["current"]["rolling_sequence"]) != meta["current_sequence"]:
        raise RuntimeError("Teensy Better-Buckets export current identity mismatch")
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
        subsystem="CLOCKS",
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
    ):
        raise RuntimeError(
            f"Teensy Better-Buckets export {history} chunk failed: {response!r}"
        )
    returned = _as_int(payload.get("count"))
    if returned is None or returned < 0 or returned > int(count):
        raise RuntimeError("Teensy Better-Buckets export returned invalid chunk count")

    out: List[Dict[str, Any]] = []
    previous = int(before_sequence) if int(before_sequence) > 0 else None
    for index in range(int(returned)):
        endpoint = _ppb_export_endpoint_from_payload(payload, f"e{index}")
        sequence = int(endpoint["rolling_sequence"])
        if previous is not None and sequence >= previous:
            raise RuntimeError(
                "Teensy Better-Buckets export chunk is not strictly newest-to-oldest"
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
    """Collect producer endpoints for one META snapshot, retaining partial custody.

    Existing Pi endpoints seed the set, so a small observation gap normally
    downloads only the missing historical suffix/prefix rather than all 601
    seconds. Firmware pages by immutable rolling-sequence cursor; concurrent
    newer appends cannot rewrite an endpoint already returned.

    Alpha is intentionally not frozen. If the bounded producer ring advances
    while paging, return every exact endpoint learned together with complete=False.
    The caller carries those immutable endpoints into the next META window.
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
        raise ValueError(f"unsupported Better-Buckets export history {history!r}")

    if expected == 0:
        if oldest != 0 or newest != 0:
            raise RuntimeError(f"{history} export has empty count with nonempty bounds")
        return [], True
    if oldest <= 0 or newest < oldest:
        raise RuntimeError(f"{history} export has invalid sequence bounds")

    collected: Dict[int, Dict[str, Any]] = {}
    for endpoint in existing:
        sequence = _as_int(endpoint.get("rolling_sequence"))
        if sequence is None or not (oldest <= sequence <= newest):
            continue
        prior = collected.get(sequence)
        if prior is not None and not _ppb_endpoints_equal(prior, endpoint):
            raise RuntimeError(f"{history} existing endpoint identity changed at {sequence}")
        collected[sequence] = copy.deepcopy(endpoint)

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
            sequence = int(endpoint["rolling_sequence"])
            if sequence < oldest:
                continue
            if sequence > newest:
                raise RuntimeError(
                    f"{history} export crossed snapshot newest boundary: {sequence}>{newest}"
                )
            prior = collected.get(sequence)
            if prior is not None and not _ppb_endpoints_equal(prior, endpoint):
                raise RuntimeError(
                    f"{history} producer endpoint changed at rolling_sequence={sequence}"
                )
            collected[sequence] = copy.deepcopy(endpoint)
        before_sequence = int(page[-1]["rolling_sequence"])
        if before_sequence <= oldest:
            break

    ordered = [collected[key] for key in sorted(collected)]
    complete = bool(
        len(ordered) == expected
        and int(ordered[0]["rolling_sequence"]) == oldest
        and int(ordered[-1]["rolling_sequence"]) == newest
    )
    return ordered, complete


def _refresh_ppb_checkpoint_from_proved_alpha() -> Dict[str, Any]:
    """Reacquire exact Better-Buckets rings from a proved live Alpha.

    This is a read-only custody transfer. It is invoked only after either the
    independent surviving-Alpha lineage court or the exact durable resurrection
    N+1 court has proved the current Alpha lifetime. The final install occurs only
    when Pi's processed update_count exactly equals the firmware META snapshot, so
    queued live rows continue naturally at N+1 after the lock opens.
    """
    _diag["ppb_checkpoint_live_refresh_attempts"] = (
        _diag.get("ppb_checkpoint_live_refresh_attempts", 0) + 1
    )
    started = time.monotonic()
    attempts = 0
    learned_second_history: List[Dict[str, Any]] = []
    learned_minute_history: List[Dict[str, Any]] = []
    try:
        while True:
            attempts += 1
            meta = _fetch_teensy_ppb_export_meta()

            with _ppb_checkpoint_lock:
                runtime = _ppb_checkpoint_runtime_ensure_locked()
                runtime_snapshot = _ppb_checkpoint_snapshot_locked(runtime)

            if runtime_snapshot.get("reset_count") not in (None, meta["reset_count"]):
                raise RuntimeError(
                    "proved Alpha export reset_count disagrees with Pi checkpoint: "
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
                if attempts >= 12:
                    raise RuntimeError(
                        "proved Alpha SECOND export could not converge after cumulative "
                        f"moving-window custody: got={len(second_history)} "
                        f"expected={meta['second_count']} "
                        f"bounds={meta['second_oldest_sequence']}.."
                        f"{meta['second_newest_sequence']}"
                    )
                logging.info(
                    "⏳ [clocks/ppb] proved Alpha SECOND window moved during paging; "
                    "retaining %d exact endpoints and retrying current META "
                    "(attempt=%d/12)",
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
                if attempts >= 12:
                    raise RuntimeError(
                        "proved Alpha MINUTE export could not converge after cumulative "
                        f"moving-window custody: got={len(minute_history)} "
                        f"expected={meta['minute_count']} "
                        f"bounds={meta['minute_oldest_sequence']}.."
                        f"{meta['minute_newest_sequence']}"
                    )
                logging.info(
                    "⏳ [clocks/ppb] proved Alpha MINUTE window moved during paging; "
                    "retaining %d exact endpoints and retrying current META "
                    "(attempt=%d/12)",
                    len(minute_history),
                    attempts,
                )
                time.sleep(0.05)
                continue

            # Export may span one or more live seconds. Re-read META and rendezvous
            # only if the exact statistical identity is still the one we collected.
            final_meta = _fetch_teensy_ppb_export_meta()
            stable_meta = all(
                final_meta.get(key) == meta.get(key)
                for key in (
                    "reset_count", "update_count", "current_sequence",
                    "second_count", "minute_count",
                    "second_oldest_sequence", "second_newest_sequence",
                    "minute_oldest_sequence", "minute_newest_sequence",
                    "last_minute_key", "origin_valid",
                )
            ) and _ppb_endpoints_equal(final_meta.get("current"), meta.get("current"))
            if not stable_meta:
                # Keep the endpoints already learned in Pi runtime as ordinary
                # live custody and try the newer immutable window.
                if attempts >= 12:
                    raise RuntimeError(
                        "proved Alpha Better-Buckets export could not reach a stable META rendezvous"
                    )
                time.sleep(0.05)
                continue

            with _ppb_checkpoint_lock:
                runtime = _ppb_checkpoint_runtime_ensure_locked()
                live_snapshot = _ppb_checkpoint_snapshot_locked(runtime)
                live_reset = live_snapshot.get("reset_count")
                live_update = live_snapshot.get("update_count")

                if live_reset != meta["reset_count"] or live_update != meta["update_count"]:
                    if attempts >= 12:
                        raise RuntimeError(
                            "Pi Better-Buckets consumer did not rendezvous with Alpha export "
                            f"(pi={live_reset}/{live_update} teensy="
                            f"{meta['reset_count']}/{meta['update_count']})"
                        )
                    time.sleep(0.05)
                    continue

                # Current cumulative state must agree before historical custody is
                # allowed to replace the incomplete Pi rings.
                if (
                    int(live_snapshot.get("current_sequence") or 0) !=
                        int(meta["current_sequence"])
                    or not _ppb_endpoints_equal(
                        live_snapshot.get("current"), meta["current"]
                    )
                ):
                    raise RuntimeError(
                        "proved Alpha export current endpoint disagrees with Pi live testimony"
                    )

                candidate = {
                    "schema": PPB_PI_CHECKPOINT_SCHEMA,
                    "source_schema": PPB_FIRMWARE_DELTA_SCHEMA,
                    "valid": True,
                    "recoverable": True,
                    "status": "RECOVERABLE",
                    "status_reason": "FULL_RING_REACQUIRED_FROM_SURVIVING_ALPHA",
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
                    "contiguous_from_update_count": int(meta["update_count"]),
                    "gap_count": int(live_snapshot.get("gap_count") or 0),
                    "last_gap": copy.deepcopy(live_snapshot.get("last_gap")),
                    "seeded_from_durable": False,
                    "seed_source_db_detail_id": None,
                    "last_delta_signature": None,
                    "proof_checks": 0,
                }
                normalized = _normalize_saved_ppb_checkpoint(candidate)
                if not normalized.get("recoverable"):
                    raise RuntimeError(
                        "proved Alpha full-ring export did not form a recoverable checkpoint"
                    )

                refreshed = _restore_ppb_checkpoint_runtime(
                    normalized,
                    source_db_detail_id=None,
                )

            result = {
                "refreshed": True,
                "basis": "TEENSY_ALPHA_FULL_RING_EXPORT",
                "reset_count": int(meta["reset_count"]),
                "update_count": int(meta["update_count"]),
                "current_sequence": int(meta["current_sequence"]),
                "second_count": len(second_history),
                "minute_count": len(minute_history),
                "attempts": int(attempts),
                "waited_s": round(time.monotonic() - started, 3),
                "prior_recoverable": bool(runtime_snapshot.get("recoverable")),
                "prior_second_count": int(runtime_snapshot.get("second_count") or 0),
                "prior_minute_count": int(runtime_snapshot.get("minute_count") or 0),
                "gap_count": int(refreshed.get("gap_count") or 0),
            }
            _diag["ppb_checkpoint_live_refresh_success"] = (
                _diag.get("ppb_checkpoint_live_refresh_success", 0) + 1
            )
            _diag["last_ppb_checkpoint_live_refresh"] = copy.deepcopy(result)
            logging.info(
                "📦 [clocks/ppb] proved Alpha full-ring custody reacquired: "
                "reset=%d update=%d second=%d/%d minute=%d/%d attempts=%d in %.3fs",
                int(meta["reset_count"]),
                int(meta["update_count"]),
                len(second_history),
                int(meta["second_count"]),
                len(minute_history),
                int(meta["minute_count"]),
                int(attempts),
                float(result["waited_s"]),
            )
            return result
    except Exception as exc:
        _diag["ppb_checkpoint_live_refresh_failures"] = (
            _diag.get("ppb_checkpoint_live_refresh_failures", 0) + 1
        )
        _diag["last_ppb_checkpoint_live_refresh"] = {
            "refreshed": False,
            "error": str(exc),
            "attempts": int(attempts),
            "waited_s": round(time.monotonic() - started, 3),
        }
        raise


def _probe_teensy_recovery_epoch() -> Tuple[bool, Dict[str, Any]]:
    """Return explicit Teensy epoch readiness together with its lifecycle testimony."""
    last_status: Dict[str, Any] = {}
    for _ in range(4):
        status = _fetch_teensy_recovery_status()
        if status:
            last_status = status
            if "recover_epoch_ready" in status:
                return _recovery_bool(status.get("recover_epoch_ready")), dict(status)
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
        "restore_schema_version": 4,
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
                raise ValueError(f"canonical CLOCKS restore missing {lane} Pi control/servo state")
            try:
                target = float(lane_state.get("target_code"))
            except (TypeError, ValueError):
                raise ValueError(f"canonical CLOCKS restore has invalid {lane} target_code")
            if not math.isfinite(target) or not (0.0 <= target <= AD5693R_SAFE_MAX_HW_CODE):
                raise ValueError(f"canonical CLOCKS restore has unsafe {lane} target_code")

    if include_control:
        mode = str(control.get("servo_mode") or "OFF").upper()
        if mode not in {"OFF", "TOTAL", "CAMP", "10-MIN"}:
            raise ValueError(f"canonical CLOCKS restore has invalid servo mode {mode!r}")

    missing = [key for key, value in out.items() if value is None]
    if missing:
        raise ValueError(f"canonical CLOCKS restore missing fields: {missing}")
    return out


def _dac_restore_population_ancestry_court(clocks: Dict[str, Any]) -> Dict[str, Any]:
    """Prove that Pi DAC Welfords cannot predate their corresponding OCXO population."""
    stats = clocks.get("stats") if isinstance(clocks, dict) else None
    stats = stats if isinstance(stats, dict) else {}
    auxiliary = stats.get("auxiliary_welford")
    auxiliary = auxiliary if isinstance(auxiliary, dict) else {}

    lanes: Dict[str, Any] = {}
    for lane in ("ocxo1", "ocxo2"):
        ocxo_welford = _path_get(stats, f"{lane}.welford")
        dac_welford = auxiliary.get(f"{lane}_dac")
        if not isinstance(ocxo_welford, dict) or not isinstance(dac_welford, dict):
            return {
                "available": False,
                "valid": False,
                "reason": f"missing_{lane}_population_witness",
                "lanes": lanes,
            }
        ocxo_n = _as_int(ocxo_welford.get("n"))
        dac_n = _as_int(dac_welford.get("n"))
        if ocxo_n is None or dac_n is None or ocxo_n < 0 or dac_n < 0:
            return {
                "available": False,
                "valid": False,
                "reason": f"invalid_{lane}_population_witness",
                "lanes": lanes,
            }
        lanes[lane] = {
            "ocxo_n": int(ocxo_n),
            "dac_n": int(dac_n),
            "dac_minus_ocxo": int(dac_n - ocxo_n),
            "valid": bool(dac_n <= ocxo_n),
        }

    impossible = [lane for lane, witness in lanes.items() if not witness["valid"]]
    return {
        "available": True,
        "valid": not impossible,
        "reason": None if not impossible else "dac_population_exceeds_ocxo_population",
        "reset_count": _as_int(stats.get("reset_count")),
        "update_count": _as_int(stats.get("update_count")),
        "completed_pps_sequence": _as_int(clocks.get("completed_pps_sequence")),
        "lanes": lanes,
        "impossible_lanes": impossible,
    }


def _canonical_instrument_restore_ready(
    clocks: Dict[str, Any],
    *,
    include_control: bool = True,
    detail_id: Optional[int] = None,
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
        # Pi DAC Welfords are full holistic-control custody, not TEMPEST campaign
        # chronology.  A campaign-only recovery may lawfully use older Teensy
        # clock/statistics testimony without restoring the poisoned Pi DAC auxiliary
        # state; the later recovery-boundary court still forbids that poison from
        # becoming fresh restore authority.
        ancestry = _dac_restore_population_ancestry_court(clocks)
        if ancestry.get("available") and not ancestry.get("valid"):
            details = {
                "court": "DAC_OCXO_WELFORD_POPULATION_ANCESTRY",
                "db_detail_id": int(detail_id) if detail_id is not None else None,
                **copy.deepcopy(ancestry),
            }
            _require_hard_failure(
                "dac_restore_population_ancestry_impossible",
                details,
                source="CLOCKS_RESTORE_COURT",
            )
        if not ancestry.get("available"):
            # Pi DAC Welfords are part of full holistic restore authority. Older rows
            # that predate this custody surface may be skipped, but never synthesized.
            return False

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


@dataclass(frozen=True)
class _ClocksRecoverySnapshot:
    """One explicit CLOCKS resurrection image at a durable one-second boundary.

    This is an internal formalization only; it does not introduce a new durable
    representation. ``canonical`` is the exact CLOCKS_V4 campaign_detail payload
    and ``ppb_restore_checkpoint`` is the exact config.CLOCKS_RECOVERY sidecar
    owned by that same row. Campaign state is carried as part of the snapshot, not
    rediscovered as a separate recovery species.

    The existing recovery executor still consumes the historical process-local
    shape with the Better-Buckets checkpoint embedded under ``clocks``.
    ``restore_detail()`` is the single compatibility boundary that forms that
    legacy view from the explicit snapshot pair.
    """

    schema: str
    source_detail_id: int
    source_sequence: int
    source_campaign: Optional[str]
    canonical: Dict[str, Any]
    ppb_restore_checkpoint: Dict[str, Any]
    campaign: Optional[Dict[str, Any]]
    gnss_raw: Dict[str, Any]

    def restore_detail(self) -> Dict[str, Any]:
        restored = copy.deepcopy(self.canonical)
        _clocks_payload(restored)["ppb_restore_checkpoint"] = copy.deepcopy(
            self.ppb_restore_checkpoint
        )
        restored["_db_detail_id"] = int(self.source_detail_id)
        return restored


def _clocks_recovery_snapshot_from_authority(
    row: Dict[str, Any], checkpoint: Dict[str, Any]
) -> _ClocksRecoverySnapshot:
    """Bind one canonical CLOCKS row to its exact singleton recovery sidecar."""
    saved = _normalize_saved_ppb_checkpoint(checkpoint)
    row_id = _as_int(row.get("id"))
    if row_id is None or row_id <= 0:
        raise RuntimeError("CLOCKS recovery source has no positive durable detail identity")
    owner_id = _as_int(saved.get("durable_source_detail_id"))
    if owner_id != row_id:
        raise RuntimeError(
            f"config.{CLOCKS_RECOVERY_CONFIG_KEY} owner mismatch: "
            f"checkpoint={owner_id} row={row_id}"
        )

    state = row.get("payload")
    if isinstance(state, str):
        state = json.loads(state)
    if not isinstance(state, dict):
        raise RuntimeError("CLOCKS recovery source payload is not an object")
    state = copy.deepcopy(state)

    sequence = _as_int(state.get("sequence"))
    if sequence is None or sequence <= 0:
        raise RuntimeError(
            f"CLOCKS recovery source detail_id={row_id} lacks positive sequence identity"
        )

    clocks = _clocks_payload(state)
    if not _checkpoint_matches_clocks_state(saved, state):
        raise RuntimeError(
            f"config.{CLOCKS_RECOVERY_CONFIG_KEY} identity disagrees with detail_id={row_id}"
        )
    if not _canonical_instrument_restore_ready(
        clocks, include_control=True, detail_id=row_id
    ):
        raise RuntimeError(
            f"config.{CLOCKS_RECOVERY_CONFIG_KEY} points to non-restorable detail_id={row_id}"
        )

    gnss_raw = _clocks_gnss_raw_payload(state)
    if gnss_raw is None:
        raise RuntimeError(
            f"config.{CLOCKS_RECOVERY_CONFIG_KEY} source detail_id={row_id} lacks GNSS_RAW"
        )

    campaign_raw = state.get("campaign")
    if campaign_raw is not None and not isinstance(campaign_raw, dict):
        raise RuntimeError(
            f"CLOCKS recovery source detail_id={row_id} has malformed campaign decoration"
        )
    source_campaign_raw = row.get("campaign")
    source_campaign = (
        str(source_campaign_raw) if source_campaign_raw is not None else None
    )

    return _ClocksRecoverySnapshot(
        schema=CLOCKS_RECOVERY_SNAPSHOT_SCHEMA,
        source_detail_id=int(row_id),
        source_sequence=int(sequence),
        source_campaign=source_campaign,
        canonical=state,
        ppb_restore_checkpoint=copy.deepcopy(saved),
        campaign=copy.deepcopy(campaign_raw) if isinstance(campaign_raw, dict) else None,
        gnss_raw=copy.deepcopy(gnss_raw),
    )


def _load_clocks_recovery_snapshot() -> Optional[_ClocksRecoverySnapshot]:
    """Return the current declarative CLOCKS recovery snapshot, if one exists."""
    checkpoint = _load_or_migrate_clocks_recovery_checkpoint()
    if checkpoint is None:
        return None
    row = _clocks_recovery_source_row(checkpoint)
    if row is None:
        raise RuntimeError(
            f"config.{CLOCKS_RECOVERY_CONFIG_KEY} has no matching durable CLOCKS restore row"
        )
    return _clocks_recovery_snapshot_from_authority(row, checkpoint)


def _read_latest_recoverable_clocks_state(
    *,
    scan_limit: int = 64,
) -> Tuple[Optional[Dict[str, Any]], int]:
    """Compatibility view of the explicit CLOCKS recovery snapshot."""
    del scan_limit
    snapshot = _load_clocks_recovery_snapshot()
    return (snapshot.restore_detail() if snapshot is not None else None), 0

def _seed_clocks_from_detail(detail: Dict[str, Any]) -> None:
    """Publish/cache a compact durable CLOCKS seed without its private recovery ring."""
    seeded = copy.deepcopy(detail)
    checkpoint = _ppb_restore_checkpoint_from_clocks(_clocks_payload(seeded))
    _clocks_payload(seeded).pop("ppb_restore_checkpoint", None)
    seeded["restored_display_seed"] = True
    seeded["restored_display_seed_at_utc"] = (
        datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    )
    _cache_clocks_state(seeded, checkpoint)
    publish(CLOCKS_TOPIC, seeded)


def _request_teensy_holistic_restore(
    clocks: Dict[str, Any],
    *,
    allow_ppb_stage_required: bool = False,
) -> Dict[str, Any]:
    """Request one complete firmware CLOCKS restore from canonical instrument state.

    With allow_ppb_stage_required=True this also acts as the lifecycle probe used
    before checkpoint resurrection: idle firmware may truthfully answer that
    Better-Buckets state must be staged first, while a live campaign still returns
    its ordinary busy/live-custody verdict without Alpha being touched.
    """
    deadline = time.monotonic() + HOLISTIC_RESTORE_COMMAND_RETRY_S
    restore_args = _canonical_restore_args(clocks, include_control=False)
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

def _holistic_restore_probe(
    state: Optional[Dict[str, Any]],
    ppb_restore_checkpoint: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Project sufficient state used to prove restore convergence.

    The canonical CLOCKS observation is intentionally compact.  Better-Buckets
    resurrection custody travels beside it as a private Pi persistence/proof
    sidecar, so callers evaluating a live compact row must supply that checkpoint.
    Rehydrated durable recovery sources may still carry the checkpoint inside
    ``clocks`` process-locally.
    """
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

    ppb_checkpoint_recoverable = False
    ppb_checkpoint_update_count = 0
    ppb_checkpoint_current_sequence = 0
    checkpoint = (
        copy.deepcopy(ppb_restore_checkpoint)
        if isinstance(ppb_restore_checkpoint, dict)
        else _ppb_restore_checkpoint_from_clocks(clocks)
    )
    if checkpoint is not None:
        try:
            normalized_checkpoint = _normalize_saved_ppb_checkpoint(checkpoint)
            ppb_checkpoint_recoverable = bool(normalized_checkpoint.get("recoverable"))
            ppb_checkpoint_update_count = int(normalized_checkpoint.get("update_count") or 0)
            ppb_checkpoint_current_sequence = int(
                normalized_checkpoint.get("current_sequence") or 0
            )
        except (TypeError, ValueError):
            pass

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
        "ppb_checkpoint_recoverable": ppb_checkpoint_recoverable,
        "ppb_checkpoint_update_count": ppb_checkpoint_update_count,
        "ppb_checkpoint_current_sequence": ppb_checkpoint_current_sequence,
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
    if bool(expected.get("ppb_checkpoint_recoverable")):
        if not bool(observed.get("ppb_checkpoint_recoverable")):
            return False
        expected_ppb_update = int(expected.get("ppb_checkpoint_update_count") or 0)
        observed_ppb_update = int(observed.get("ppb_checkpoint_update_count") or 0)
        if observed_ppb_update < expected_ppb_update:
            return False
        expected_ppb_current = int(expected.get("ppb_checkpoint_current_sequence") or 0)
        observed_ppb_current = int(observed.get("ppb_checkpoint_current_sequence") or 0)
        if observed_ppb_current < expected_ppb_current:
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
    if bool(expected.get("ppb_checkpoint_recoverable")) and not bool(
        observed.get("ppb_checkpoint_recoverable")
    ):
        pending.append("BETTER_BUCKETS_CHECKPOINT")
    if not _holistic_restore_probe_satisfied(
        {**expected, "instrument_gnss_ns": 0, "instrument_dwt_cycles": 0,
         "instrument_ocxo1_ns": 0, "instrument_ocxo2_ns": 0, "welford_n": {}},
        observed,
    ):
        pending.append("DAC_CONTROL")
    return pending or ["FRESH_CLOCKS_PROOF"]


def _holistic_restore_expected_probe(
    detail: Dict[str, Any],
    *,
    preserve_live_pi_control: bool = False,
) -> Dict[str, Any]:
    """Return restore expectations, optionally preserving surviving Pi control custody."""
    expected = _holistic_restore_probe(detail)
    if preserve_live_pi_control:
        control = _dac_control_snapshot()
        expected["servo_mode"] = str(control.get("servo_mode") or "OFF").upper()
        expected["dither_operator_enabled"] = bool(control.get("dither_operator_enabled"))
        expected["ocxo1_dac"] = _first_float(_path_get(control, "ocxo1.target_code"))
        expected["ocxo2_dac"] = _first_float(_path_get(control, "ocxo2.target_code"))
    return expected


def _arm_holistic_restore_persistence_proof(
    detail: Dict[str, Any],
    *,
    preserve_live_pi_control: bool = False,
) -> None:
    """Require the first post-restore Alpha row to become durable evidence."""
    global _clocks_holistic_restore_proof_expected
    global _clocks_holistic_restore_proof_reset_count
    global _clocks_holistic_restore_proof_update_count
    global _clocks_holistic_restore_proof_sequence

    stats = _clocks_payload(detail).get("stats")
    reset_count = _as_int(stats.get("reset_count")) if isinstance(stats, dict) else None
    update_count = _as_int(stats.get("update_count")) if isinstance(stats, dict) else None
    if reset_count is None or reset_count < 0 or update_count is None or update_count < 0:
        raise RuntimeError("holistic restore source lacks usable statistics chronology")

    _clocks_holistic_restore_proof_expected = _holistic_restore_expected_probe(
        detail,
        preserve_live_pi_control=preserve_live_pi_control,
    )
    _clocks_holistic_restore_proof_reset_count = int(reset_count)
    _clocks_holistic_restore_proof_update_count = int(update_count) + 1
    _clocks_holistic_restore_proof_sequence = None
    _clocks_holistic_restore_proof_committed.clear()
    _clocks_holistic_restore_proof_pending.set()


def _log_holistic_restore_row_court(waited_s: float, *, phase: str) -> None:
    """Log Teensy Alpha's read-only row court while no fresh CLOCKS row exists."""
    try:
        response = send_command(
            machine="TEENSY",
            subsystem="CLOCKS",
            command="REPORT_ROW_COURT",
            retries=1,
            retry_delay_s=0.0,
        )
    except Exception as exc:
        logging.warning(
            "⚠️ [holistic restore] no fresh CLOCKS_FRAGMENT after %.1fs; "
            "REPORT_ROW_COURT failed during %s: %s",
            float(waited_s),
            phase,
            exc,
        )
        return

    payload = response.get("payload") if isinstance(response, dict) else None
    if not isinstance(response, dict) or not response.get("success") or not isinstance(payload, dict):
        logging.warning(
            "⚠️ [holistic restore] no fresh CLOCKS row after %.1fs and the firmware "
            "row court is unavailable during %s (success=%s message=%s)",
            float(waited_s),
            phase,
            bool(response.get("success")) if isinstance(response, dict) else False,
            response.get("message") if isinstance(response, dict) else "malformed response",
        )
        return

    logging.info(
        "⚖️ [holistic restore] no fresh CLOCKS row after %.1fs; firmware row court "
        "during %s reports status=%s sequence=%s ready=%s reason=%s",
        float(waited_s),
        phase,
        payload.get("status") or "not reported",
        payload.get("sequence") or payload.get("completed_pps_sequence") or "n/a",
        payload.get("ready") if "ready" in payload else payload.get("restore_court_ready"),
        payload.get("reason") or payload.get("status_reason") or "none",
    )


def _wait_for_holistic_restore(
    detail: Dict[str, Any],
    *,
    requested_monotonic: float,
    timeout_s: float = HOLISTIC_RESTORE_TIMEOUT_S,
    preserve_live_pi_control: bool = False,
) -> Dict[str, Any]:
    expected = _holistic_restore_expected_probe(
        detail,
        preserve_live_pi_control=preserve_live_pi_control,
    )
    deadline = time.monotonic() + float(timeout_s)
    next_progress_log = requested_monotonic + 10.0
    last_observed: Dict[str, Any] = {}
    while time.monotonic() < deadline:
        with _clocks_lock:
            current = copy.deepcopy(_latest_clocks)
            current_ppb_checkpoint = copy.deepcopy(
                _latest_clocks_ppb_restore_checkpoint
            )
            received = _latest_clocks_received_monotonic
        if received is not None and received > requested_monotonic and current:
            last_observed = _holistic_restore_probe(
                current, current_ppb_checkpoint
            )
            if (
                _holistic_restore_probe_satisfied(expected, last_observed)
                and _clocks_holistic_restore_proof_committed.is_set()
            ):
                return {
                    "proved": True,
                    "durable": True,
                    "expected": expected,
                    "observed": last_observed,
                    "clocks_sequence": current.get("sequence"),
                    "durable_proof_sequence": _clocks_holistic_restore_proof_sequence,
                    "waited_s": round(time.monotonic() - requested_monotonic, 3),
                }
        now = time.monotonic()
        if now >= next_progress_log:
            waited_s = now - requested_monotonic
            if not last_observed:
                _log_holistic_restore_row_court(waited_s, phase="progress")
            else:
                logging.info(
                    "⏳ [holistic restore] converging after %.1fs; waiting for %s",
                    waited_s,
                    ", ".join(_holistic_restore_pending_categories(expected, last_observed)),
                )
            next_progress_log = now + 10.0
        time.sleep(0.1)
    if not last_observed:
        _log_holistic_restore_row_court(
            time.monotonic() - requested_monotonic,
            phase="timeout",
        )
    raise TimeoutError(
        "timed out waiting for holistic CLOCKS restore proof "
        f"after {timeout_s:.1f}s; expected={expected!r} observed={last_observed!r}"
    )


def _wait_for_fresh_survival_witness() -> Tuple[Dict[str, Any], Optional[float], Optional[str]]:
    """Return current-session testimony sufficient to classify Alpha lifetime.

    PUBSUB may deliver a retained pre-flash producer tail into a newly started
    CLOCKS process.  The state worker can process that tail much later, so cache
    processing time is not producer-lifetime freshness.  A startup survival row
    is eligible only when its original PUBSUB ingress timestamp follows the
    current Teensy transport-admission barrier.

    During live recovery a proved boot-local physical sequence regression remains
    positive evidence that the prior producer lifetime ended and may return the
    best current testimony immediately.
    """
    deadline = time.monotonic() + STARTUP_SURVIVAL_FRESH_WITNESS_TIMEOUT_S
    while True:
        with _clocks_lock:
            state = copy.deepcopy(_latest_clocks)
            received_monotonic = _latest_clocks_received_monotonic
            received_utc = _latest_clocks_received_utc
        age_s = (
            None
            if received_monotonic is None
            else max(0.0, time.monotonic() - received_monotonic)
        )
        if _recovery_custody_requires_cold_restore():
            return state, age_s, received_utc
        current_session_ingress = bool(
            received_monotonic is not None
            and (
                _startup_survival_ingress_barrier_monotonic is None
                or received_monotonic > _startup_survival_ingress_barrier_monotonic
            )
        )
        if (
            state
            and current_session_ingress
            and age_s is not None
            and age_s <= CLOCKS_PREFLIGHT_MAX_AGE_S
        ):
            return state, age_s, received_utc
        if time.monotonic() >= deadline:
            raise RuntimeError(
                "cannot classify CLOCKS producer lifetime without current-session "
                "canonical testimony after the transport admission barrier within "
                f"{STARTUP_SURVIVAL_FRESH_WITNESS_TIMEOUT_S:.1f}s "
                f"(last_age_s={age_s} ingress={received_monotonic} "
                f"barrier={_startup_survival_ingress_barrier_monotonic})"
            )
        time.sleep(0.05)


def _alpha_lineage_reason_summary(reasons: List[str]) -> str:
    """Translate internal lineage reason codes into one operator-facing explanation."""
    reason_set = set(str(reason) for reason in reasons)
    parts: List[str] = []
    if "statistics_reset_count_changed" in reason_set:
        parts.append("the Teensy statistics epoch restarted")
    if "statistics_update_count_regressed" in reason_set:
        parts.append("the live statistics count is behind the durable count")
    if "better_buckets_current_sequence_regressed" in reason_set:
        parts.append("the live Better-Buckets history is younger than the durable history")
    faces = [str(r).split(":",1)[1] for r in reasons if str(r).startswith("clockface_regressed_or_missing:")]
    if faces:
        pretty={"gnss_ns":"GNSS","dwt_cycles":"DWT","ocxo1_ns":"OCXO1","ocxo2_ns":"OCXO2"}
        parts.append(f"{'/'.join(pretty.get(x,x) for x in faces)} clockfaces restarted or are missing")
    if "canonical_live_not_coherent_instrument_state" in reason_set:
        parts.append("the live CLOCKS row is not yet a coherent restore witness")
    if "canonical_live_stale" in reason_set:
        parts.append("the newest live CLOCKS row is stale")
    if "canonical_live_not_current_lifetime_proven_pre_epoch" in reason_set:
        parts.append("the current Alpha epoch is not installed yet")
    if "physical_sequence_regression_during_recovery_custody" in reason_set:
        parts.append("the boot-local CLOCKS sequence proved a new Teensy lifetime")
    known={"statistics_reset_count_changed","statistics_update_count_regressed","better_buckets_current_sequence_regressed","canonical_live_not_coherent_instrument_state","canonical_live_stale","canonical_live_not_current_lifetime_proven_pre_epoch","physical_sequence_regression_during_recovery_custody"}
    unknown=[str(r) for r in reasons if str(r) not in known and not str(r).startswith("clockface_regressed_or_missing:")]
    if unknown:
        parts.append("additional continuity evidence is incomplete")
    return "; ".join(parts) if parts else "no continuity objection"


def _alpha_survival_lineage_court(
    durable_clocks: Dict[str, Any],
    live_state: Dict[str, Any],
    *,
    live_age_s: Optional[float],
) -> Dict[str, Any]:
    """Prove that current Alpha state descends from the durable restore source.

    REPORT_RECOVERY describes the Alpha that is alive *now*.  After a Teensy
    reboot, a newborn Alpha can truthfully report ALPHA ownership, preserved
    local statistics, and no pending restore obligation.  Those facts do not
    prove that it is the same statistical lifetime PostgreSQL remembers.

    Survival therefore requires an independent fresh canonical CLOCKS witness:
    the durable statistics epoch must be unchanged, its monotonic population
    identities must not regress, and the four instrument clockfaces must be at
    or beyond their durable values.  Missing/ambiguous evidence fails closed into
    the ordinary RESTORE_MONITOR/instrument-restore path.
    """
    reasons: List[str] = []
    live_clocks = _clocks_payload(live_state)
    durable_stats = durable_clocks.get("stats") if isinstance(durable_clocks, dict) else None
    live_stats = live_clocks.get("stats") if isinstance(live_clocks, dict) else None
    durable_faces = (
        durable_clocks.get("clockfaces") if isinstance(durable_clocks, dict) else None
    )
    live_faces = live_clocks.get("clockfaces") if isinstance(live_clocks, dict) else None

    if not isinstance(live_state, dict) or not live_state:
        reasons.append("canonical_live_unavailable")
    elif bool(live_state.get("restored_display_seed")):
        reasons.append("canonical_live_is_durable_display_seed")
    if live_age_s is None:
        reasons.append("canonical_live_age_unknown")
    elif live_age_s > CLOCKS_PREFLIGHT_MAX_AGE_S:
        reasons.append("canonical_live_stale")

    live_restore_ready = False
    if isinstance(live_clocks, dict) and live_clocks:
        try:
            live_restore_ready = _canonical_instrument_restore_ready(
                live_clocks, include_control=False
            )
        except HardFailureRequired:
            raise
        except Exception:
            live_restore_ready = False
    if not live_restore_ready:
        reasons.append("canonical_live_not_coherent_instrument_state")

    durable_reset = (
        _as_int(durable_stats.get("reset_count"))
        if isinstance(durable_stats, dict)
        else None
    )
    durable_update = (
        _as_int(durable_stats.get("update_count"))
        if isinstance(durable_stats, dict)
        else None
    )
    live_reset = (
        _as_int(live_stats.get("reset_count"))
        if isinstance(live_stats, dict)
        else None
    )
    live_update = (
        _as_int(live_stats.get("update_count"))
        if isinstance(live_stats, dict)
        else None
    )

    if durable_reset is None or durable_update is None:
        reasons.append("durable_statistics_chronology_missing")
    if live_reset is None or live_update is None:
        reasons.append("canonical_live_statistics_chronology_missing")
    if durable_reset is not None and live_reset is not None and live_reset != durable_reset:
        reasons.append("statistics_reset_count_changed")
    if durable_update is not None and live_update is not None and live_update < durable_update:
        reasons.append("statistics_update_count_regressed")

    durable_current = (
        _as_int(durable_stats.get("rolling_ppb_current_sequence"))
        if isinstance(durable_stats, dict)
        else None
    )
    live_current = (
        _as_int(live_stats.get("rolling_ppb_current_sequence"))
        if isinstance(live_stats, dict)
        else None
    )
    if durable_current is None or live_current is None:
        reasons.append("better_buckets_current_sequence_missing")
    elif live_current < durable_current:
        reasons.append("better_buckets_current_sequence_regressed")

    clockfaces: Dict[str, Any] = {}
    for key in ("gnss_ns", "dwt_cycles", "ocxo1_ns", "ocxo2_ns"):
        durable_value = (
            _as_int(durable_faces.get(key)) if isinstance(durable_faces, dict) else None
        )
        live_value = _as_int(live_faces.get(key)) if isinstance(live_faces, dict) else None
        monotonic = bool(
            durable_value is not None
            and durable_value > 0
            and live_value is not None
            and live_value >= durable_value
        )
        clockfaces[key] = {
            "durable": durable_value,
            "live": live_value,
            "monotonic": monotonic,
        }
        if not monotonic:
            reasons.append(f"clockface_regressed_or_missing:{key}")

    return {
        "proved": not reasons,
        "reasons": reasons,
        "durable_reset_count": durable_reset,
        "durable_update_count": durable_update,
        "live_reset_count": live_reset,
        "live_update_count": live_update,
        "durable_ppb_current_sequence": durable_current,
        "live_ppb_current_sequence": live_current,
        "live_sequence": _as_int(live_state.get("sequence")) if isinstance(live_state, dict) else None,
        "live_age_s": None if live_age_s is None else round(float(live_age_s), 3),
        "live_restore_ready": bool(live_restore_ready),
        "clockfaces": clockfaces,
    }


@dataclass(frozen=True)
class _HolisticInstrumentVerdict:
    """Proved producer-continuity testimony carried through convergence.

    This is process-local recovery context, not persisted state.  ``alpha_*`` field
    names remain protocol vocabulary for the Teensy statistics owner; live reuse no
    longer creates a separate instrument/campaign recovery sequence from them.
    """

    alpha_disposition: str
    alpha_basis: str
    alpha_proof: Dict[str, Any]
    observed_campaign_name: str
    observed_campaign_state: str



def _restore_instrument_from_clocks(
    detail: Dict[str, Any],
) -> Tuple[Dict[str, Any], _HolisticInstrumentVerdict]:
    clocks = _clocks_payload(detail)
    gnss_raw = _clocks_gnss_raw_payload(detail)
    if not _canonical_instrument_restore_ready(
        clocks,
        include_control=True,
        detail_id=_as_int(detail.get("_db_detail_id")),
    ) or gnss_raw is None:
        raise RuntimeError("CLOCKS detail lacks valid canonical instrument/GNSS_RAW restore state")

    def complete_surviving_producer_proof(
        teensy_payload: Dict[str, Any],
        *,
        basis: str,
        alpha_proof: Dict[str, Any],
        observed_campaign_name: str,
        observed_campaign_state: str,
    ) -> Tuple[Dict[str, Any], _HolisticInstrumentVerdict]:
        """Return a continuity verdict without mutating producer or Pi custody."""
        result = {
            "success": True,
            "mode": "LIVE_PRODUCER_PROVED",
            "alpha_resurrected_this_startup": False,
            "teensy": copy.deepcopy(teensy_payload),
            "producer_mutated": False,
            "producer_mutation_commands": [],
            "proof": {
                "proved": True,
                "basis": basis,
                "snapshot_descendant": True,
            },
        }
        verdict = _HolisticInstrumentVerdict(
            alpha_disposition="SURVIVED",
            alpha_basis=basis,
            alpha_proof=copy.deepcopy(alpha_proof),
            observed_campaign_name=str(observed_campaign_name or "").strip(),
            observed_campaign_state=str(observed_campaign_state or "").strip().upper(),
        )
        return result, verdict

    # First ask the dedicated, non-mutating firmware lifecycle surface whether
    # the same campaign is already alive.  This is the strongest Pi-only restart
    # discriminator: if Alpha still owns that live campaign, Better-Buckets does
    # not need resurrection at all and an as-yet-warming Pi checkpoint must not
    # block restoration of Pi-owned DAC/GNSS_RAW custody.
    active_campaign = _get_active_campaign()
    active_campaign_name = (
        str(active_campaign.get("campaign") or "").strip()
        if isinstance(active_campaign, dict)
        else ""
    )

    # Pi-only restart fast path: the canonical CLOCKS_FRAGMENT stream is the
    # producer-survival court.  If one fresh row proves lawful descent from the
    # durable Alpha and no physical sequence regression has been observed, return
    # before issuing *any* Teensy RPC.
    startup_live_clocks, startup_live_age_s, startup_live_received_utc = (
        _wait_for_fresh_survival_witness()
    )
    startup_live_campaign = _state_campaign(startup_live_clocks)
    startup_live_campaign_name = (
        _tempest_campaign_name(startup_live_campaign)
        if isinstance(startup_live_campaign, dict) and startup_live_campaign
        else ""
    )
    startup_live_campaign_state = str(
        startup_live_campaign.get("state")
        if isinstance(startup_live_campaign, dict)
        else ""
    ).strip().upper()
    startup_alpha_lineage = _alpha_survival_lineage_court(
        clocks,
        startup_live_clocks,
        live_age_s=startup_live_age_s,
    )
    if (
        startup_alpha_lineage.get("proved") is True
        and not _recovery_custody_requires_cold_restore()
    ):
        producer_witness = {
            "source": "CLOCKS_FRAGMENT",
            "sequence": _as_int(startup_live_clocks.get("sequence")),
            "received_at_utc": startup_live_received_utc,
            "age_s": (
                None
                if startup_live_age_s is None
                else round(float(startup_live_age_s), 3)
            ),
            "campaign": startup_live_campaign_name or None,
            "campaign_state": startup_live_campaign_state or None,
            "statistics_reset_count": startup_alpha_lineage.get("live_reset_count"),
            "statistics_update_count": startup_alpha_lineage.get("live_update_count"),
            "teensy_rpc_used": False,
        }
        logging.info(
            "✅ [holistic restore/live probe] flowing CLOCKS_FRAGMENT proves surviving "
            "Alpha with zero Teensy RPC: statistics continue %s -> %s in reset epoch "
            "%s; live CLOCKS sequence=%s age=%.3fs campaign=%s/%s",
            startup_alpha_lineage.get("durable_update_count"),
            startup_alpha_lineage.get("live_update_count"),
            startup_alpha_lineage.get("live_reset_count"),
            _as_int(startup_live_clocks.get("sequence")),
            float(startup_live_age_s or 0.0),
            startup_live_campaign_name or "NONE",
            startup_live_campaign_state or "NONE",
        )
        return complete_surviving_producer_proof(
            producer_witness,
            basis="CLOCKS_FRAGMENT_DESCENDANT_SURVIVING_ALPHA",
            alpha_proof=startup_alpha_lineage,
            observed_campaign_name=startup_live_campaign_name,
            observed_campaign_state=startup_live_campaign_state,
        )

    # Only a producer that failed the canonical descendant court may enter the
    # existing firmware lifecycle/dead-producer court below.
    lifecycle_status = _fetch_teensy_recovery_status()
    firmware_campaign = str(lifecycle_status.get("campaign") or "").strip()
    firmware_campaign_state = str(
        lifecycle_status.get("campaign_state") or ""
    ).strip().upper()
    firmware_recovery_active = _recovery_bool(
        lifecycle_status.get("recover_lifecycle_active")
    )

    # Classify producer lifetime only from a fresh canonical observation. A
    # retained pre-flash row may arrive first and is not evidence of either
    # survival or death. Firmware epoch readiness plus this fresh witness form
    # the minimum startup custody court.
    startup_live_clocks, startup_live_age_s, startup_live_received_utc = (
        _wait_for_fresh_survival_witness()
    )
    startup_live_campaign = _state_campaign(startup_live_clocks)
    startup_live_campaign_name = (
        _tempest_campaign_name(startup_live_campaign)
        if isinstance(startup_live_campaign, dict) and startup_live_campaign
        else ""
    )
    startup_live_campaign_state = str(
        startup_live_campaign.get("state")
        if isinstance(startup_live_campaign, dict)
        else ""
    ).strip().upper()
    # Producer survival is independent of campaign execution. REPORT_RECOVERY
    # contributes present-tense producer custody; canonical lineage proof remains
    # the authority for deciding whether this exact producer survived.
    alpha_owner = str(
        lifecycle_status.get("instrument_statistics_owner") or ""
    ).strip().upper()
    alpha_preserved = _recovery_bool(
        lifecycle_status.get("instrument_statistics_preserved")
    )
    recover_epoch_ready_present = "recover_epoch_ready" in lifecycle_status
    recover_epoch_ready = _recovery_bool(
        lifecycle_status.get("recover_epoch_ready")
    )
    restore_court_ready_present = "restore_court_ready" in lifecycle_status
    restore_court_ready = _recovery_bool(
        lifecycle_status.get("restore_court_ready")
    )

    alpha_lineage = _alpha_survival_lineage_court(
        clocks,
        startup_live_clocks,
        live_age_s=startup_live_age_s,
    )

    # Receipt freshness is not producer-lifetime freshness. PUBSUB may deliver
    # retained pre-flash rows immediately after a new Teensy comes online. If
    # current firmware says its restore court is open but its producer epoch is
    # not installed yet, those rows cannot prove current-lifetime survival.
    current_lifetime_pre_epoch = bool(
        lifecycle_status
        and restore_court_ready_present
        and restore_court_ready
        and recover_epoch_ready_present
        and not recover_epoch_ready
    )
    if current_lifetime_pre_epoch:
        reasons = list(alpha_lineage.get("reasons") or [])
        reason = "canonical_live_not_current_lifetime_proven_pre_epoch"
        if reason not in reasons:
            reasons.append(reason)
        alpha_lineage["reasons"] = reasons
        alpha_lineage["proved"] = False
        alpha_lineage["current_lifetime_eligible"] = False
        alpha_lineage["current_lifetime_barrier"] = {
            "restore_court_ready": True,
            "recover_epoch_ready": False,
            "basis": "TEENSY_REPORT_RECOVERY_CURRENT_LIFETIME_PRE_EPOCH",
        }
    else:
        alpha_lineage["current_lifetime_eligible"] = True

    physical_reboot_custody = _recovery_custody_requires_cold_restore()
    if physical_reboot_custody:
        reasons = list(alpha_lineage.get("reasons") or [])
        reason = "physical_sequence_regression_during_recovery_custody"
        if reason not in reasons:
            reasons.append(reason)
        alpha_lineage["reasons"] = reasons
        alpha_lineage["proved"] = False
        alpha_lineage["current_lifetime_eligible"] = False
        alpha_lineage["physical_reboot_custody"] = _recovery_custody_snapshot()

        # A boot-local physical sequence regression is stronger than absence of a
        # fresh survival witness: it positively proves that the producer lifetime
        # changed.  If this durable source also owns the active campaign, do not
        # open the standalone RESTORE_MONITOR Alpha path at all.  Return LOST and
        # let _restore_active_campaign_state() perform the one combined producer +
        # TEMPEST DEAD_PRODUCER_RESTORE transaction.
        durable_campaign = _state_campaign(detail)
        durable_campaign_name = (
            _tempest_campaign_name(durable_campaign)
            if isinstance(durable_campaign, dict) and durable_campaign
            else ""
        )
        if active_campaign_name and durable_campaign_name == active_campaign_name:
            loss_proof = {
                "proved": True,
                "alpha_lost": True,
                "basis": "PHYSICAL_SEQUENCE_REGRESSION_DURING_RECOVERY_CUSTODY",
                "report_recovery": copy.deepcopy(lifecycle_status),
                "survival_lineage": copy.deepcopy(alpha_lineage),
                "source_detail_id": _as_int(detail.get("_db_detail_id")),
                "campaign": active_campaign_name,
            }
            logging.info(
                "🧭 [holistic restore] recovery custody positively proved a new Teensy "
                "lifetime; dead Alpha + durable campaign '%s' will converge only in "
                "the combined CLOCKS.RECOVER transaction",
                active_campaign_name,
            )
            return (
                {
                    "success": True,
                    "mode": "DEFERRED_COMBINED_ALPHA_CAMPAIGN_RESTORE",
                    "alpha_resurrected_this_startup": False,
                    "teensy_probe": copy.deepcopy(lifecycle_status),
                    "proof": copy.deepcopy(loss_proof),
                },
                _HolisticInstrumentVerdict(
                    alpha_disposition="LOST",
                    alpha_basis="PHYSICAL_SEQUENCE_REGRESSION_DURING_RECOVERY_CUSTODY",
                    alpha_proof=copy.deepcopy(loss_proof),
                    observed_campaign_name=firmware_campaign,
                    observed_campaign_state=firmware_campaign_state,
                ),
            )

    alpha_lineage_proved = bool(alpha_lineage.get("proved"))
    report_alpha_survived = bool(
        lifecycle_status
        and alpha_owner == "ALPHA"
        and alpha_preserved
        and recover_epoch_ready
        and not firmware_recovery_active
        and alpha_lineage_proved
    )

    report_decline_reasons: List[str] = []
    if not lifecycle_status:
        report_decline_reasons.append("report_recovery_unavailable_or_empty")
    if alpha_owner != "ALPHA":
        report_decline_reasons.append("instrument_statistics_owner_not_alpha")
    if not alpha_preserved:
        report_decline_reasons.append("instrument_statistics_not_preserved")
    if not recover_epoch_ready:
        report_decline_reasons.append("recover_epoch_not_ready")
    if firmware_recovery_active:
        report_decline_reasons.append("firmware_recovery_lifecycle_active")
    if not alpha_lineage_proved:
        report_decline_reasons.extend(
            f"alpha_lineage:{reason}" for reason in alpha_lineage.get("reasons", [])
        )

    lineage_reasons = list(alpha_lineage.get("reasons") or [])
    lineage_summary = _alpha_lineage_reason_summary(lineage_reasons)
    if report_alpha_survived:
        logging.info(
            "✅ [holistic restore/live probe] the current Teensy is the same Alpha producer: "
            "statistics continue %s -> %s in reset epoch %s; live CLOCKS sequence=%s "
            "is %.3fs old. Firmware campaign=%s/%s.",
            alpha_lineage.get("durable_update_count"), alpha_lineage.get("live_update_count"),
            alpha_lineage.get("live_reset_count"), _as_int(startup_live_clocks.get("sequence")),
            float(startup_live_age_s or 0.0), firmware_campaign or "none", firmware_campaign_state or "none",
        )
    else:
        logging.info(
            "🧭 [holistic restore/live probe] the current Teensy is not proved to be the durable Alpha producer: "
            "statistics reset=%s->%s update=%s->%s; %s. A restore decision is required.",
            alpha_lineage.get("durable_reset_count"), alpha_lineage.get("live_reset_count"),
            alpha_lineage.get("durable_update_count"), alpha_lineage.get("live_update_count"), lineage_summary,
        )

    if report_alpha_survived:
        logging.info(
            "♻️ [holistic restore] REPORT_RECOVERY proves surviving Alpha custody: "
            "owner=%s preserved=%s recover_epoch_ready=%s "
            "lineage_reset=%s update=%s->%s; firmware campaign=%s/%s Pi campaign=%s. "
            "Restoring Pi-owned state only.",
            alpha_owner,
            alpha_preserved,
            recover_epoch_ready,
            alpha_lineage.get("live_reset_count"),
            alpha_lineage.get("durable_update_count"),
            alpha_lineage.get("live_update_count"),
            firmware_campaign or "NONE",
            firmware_campaign_state or "NONE",
            active_campaign_name or "NONE",
        )
        return complete_surviving_producer_proof(
            lifecycle_status,
            basis="TEENSY_REPORT_RECOVERY_SURVIVING_ALPHA_CUSTODY",
            alpha_proof=alpha_lineage,
            observed_campaign_name=firmware_campaign,
            observed_campaign_state=firmware_campaign_state,
        )

    # A live-campaign identity was not proved above.  Arm durable N+1 proof custody
    # before asking RESTORE_MONITOR whether Alpha needs actual resurrection.
    # The ordinary writer remains closed, so fresh-boot rows are still withheld;
    # only the exact restored N+1 row can open this narrow path into PostgreSQL.
    _arm_holistic_restore_persistence_proof(detail)

    # RESTORE_MONITOR remains the authoritative cold/full-restore court.  It can
    # independently rediscover live campaign custody, or explicitly demand the
    # literal Better-Buckets checkpoint before Alpha is touched.
    requested_monotonic = time.monotonic()
    teensy_response = _request_teensy_holistic_restore(
        clocks,
        allow_ppb_stage_required=True,
    )
    payload = teensy_response.get("payload") if isinstance(teensy_response, dict) else None
    payload = payload if isinstance(payload, dict) else {}
    status = str(payload.get("status") or "")
    campaign_state = str(payload.get("campaign_state") or "").strip().upper()
    logging.info(
        "🧭 [holistic restore/live probe] firmware agrees that instrument recovery is required: "
        "status=%s; firmware campaign=%s/%s; live canonical campaign=%s/%s. "
        "The Pi will preserve the durable source and restore from it rather than adopt this newborn producer.",
        status or "not reported", str(payload.get("campaign") or "").strip() or "none",
        campaign_state or "none", startup_live_campaign_name or "none", startup_live_campaign_state or "none",
    )

    # When this exact durable CLOCKS observation also carries the active TEMPEST
    # state, defer all producer mutation to the one combined desired-state
    # DEAD_PRODUCER_RESTORE transaction.  Return an explicit LOST verdict here.
    durable_campaign = _state_campaign(detail)
    durable_campaign_name = (
        _tempest_campaign_name(durable_campaign)
        if isinstance(durable_campaign, dict) and durable_campaign
        else ""
    )
    if active_campaign_name and durable_campaign_name == active_campaign_name:
        loss_proof = {
            "proved": True,
            "alpha_lost": True,
            "basis": "DEAD_ALPHA_DEFERRED_TO_COMBINED_CAMPAIGN_RECOVER",
            "restore_monitor_status": status or None,
            "report_decline_reasons": copy.deepcopy(report_decline_reasons),
            "survival_lineage": copy.deepcopy(alpha_lineage),
            "source_detail_id": _as_int(detail.get("_db_detail_id")),
            "campaign": active_campaign_name,
        }
        logging.info(
            "🧭 [holistic restore] dead Alpha + durable campaign '%s' will converge "
            "in one CLOCKS.RECOVER transaction; skipping standalone Alpha restore",
            active_campaign_name,
        )
        return (
            {
                "success": True,
                "mode": "DEFERRED_COMBINED_ALPHA_CAMPAIGN_RESTORE",
                "alpha_resurrected_this_startup": False,
                "teensy_probe": copy.deepcopy(payload),
                "proof": copy.deepcopy(loss_proof),
            },
            _HolisticInstrumentVerdict(
                alpha_disposition="LOST",
                alpha_basis="DEAD_ALPHA_DEFERRED_TO_COMBINED_CAMPAIGN_RECOVER",
                alpha_proof=copy.deepcopy(loss_proof),
                observed_campaign_name=str(payload.get("campaign") or "").strip(),
                observed_campaign_state=campaign_state,
            ),
        )
    # Newborn Alpha has now been disproved. Atomically arm a pre-mutation hold
    # before retiring startup custody and reinstalling durable Pi DAC ancestry.
    # This closes the race that previously allowed one newborn row to reset the
    # restored DAC Welfords before the exact Alpha N+1 successor arrived.
    with _clocks_state_mutation_gate_lock:
        _startup_instrument_restore_hold.set()
        _startup_physical_lifetime_unclassified.clear()
        _retire_startup_clocks_custody("instrument_restore_required")
        pi_control = _dac_restore_control_from_clocks(clocks, realize=True)

    ppb_stage: Dict[str, Any]
    stats_update_count = _as_int(_path_get(clocks, "stats.update_count")) or 0
    saved_ppb_checkpoint: Optional[Dict[str, Any]] = None
    if stats_update_count > 0:
        # Newborn Alpha has already been disproved above.  At this point an
        # incomplete durable checkpoint is not a transient startup condition:
        # the missing producer endpoints died with the prior Teensy lifetime.
        # Use the terminal Alpha-resurrection court so HARD_FAILURE records the
        # exact information-loss species and can offer only the explicit
        # new-statistics-epoch repair path.
        saved_ppb_checkpoint = _require_alpha_resurrection_checkpoint(
            clocks,
            campaign=active_campaign_name or "AMBIENT_STARTUP",
            recovery_source_db_id=_as_int(detail.get("_db_detail_id")),
        )

    if status == "monitor_restore_requires_ppb_state":
        if saved_ppb_checkpoint is None:
            raise RuntimeError(
                "Teensy requires Better-Buckets state but canonical CLOCKS has no "
                "recoverable Pi checkpoint"
            )
        # Keep Pi's process-local custodian on the same exact image being returned
        # to Alpha. Startup/fresh-boot fragments observed before this verdict may
        # belong to a superseded firmware lifetime and must not contaminate it.
        _restore_ppb_checkpoint_runtime(
            saved_ppb_checkpoint,
            source_db_detail_id=_as_int(detail.get("_db_detail_id")),
        )
        _seed_clocks_from_detail(detail)
        ppb_stage = _stage_teensy_better_buckets_checkpoint(saved_ppb_checkpoint)
        logging.info(
            "♻️ [holistic restore] Better-Buckets checkpoint staged directly: "
            "second=%d minute=%d source_update=%d gap_count=%d stage_s=%s",
            len(saved_ppb_checkpoint.get("second_history") or []),
            len(saved_ppb_checkpoint.get("minute_history") or []),
            int(saved_ppb_checkpoint.get("update_count") or 0),
            int(saved_ppb_checkpoint.get("gap_count") or 0),
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
        # update_count==0 needs no rolling history. An immediately accepted restore
        # with a mature population can also mean a prior interrupted Pi attempt
        # already committed this exact checkpoint to Alpha.
        _seed_clocks_from_detail(detail)
        if saved_ppb_checkpoint is not None:
            _restore_ppb_checkpoint_runtime(
                saved_ppb_checkpoint,
                source_db_detail_id=_as_int(detail.get("_db_detail_id")),
            )
        ppb_stage = {
            "staged": bool(stats_update_count > 0),
            "reason": (
                "already_committed_on_teensy"
                if stats_update_count > 0
                else "empty_statistics_epoch"
            ),
            "rolling_sequence": int(
                saved_ppb_checkpoint.get("rolling_sequence")
                if saved_ppb_checkpoint is not None
                else 0
            ),
            "checkpoint_schema": PPB_PI_CHECKPOINT_SCHEMA,
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

    # Exact durable N+1 proves this resurrected Alpha is now the authoritative
    # live owner. Rows intentionally retired during PPB_RESTORE/SmartZero proof
    # formation must not leave Pi's bounded-ring custodian artificially warming.
    # Reacquire the exact current Alpha rings before campaign recovery proceeds.
    ppb_refresh = _refresh_ppb_checkpoint_from_proved_alpha()

    recovery_receipt: Optional[Dict[str, Any]] = None
    if saved_ppb_checkpoint is not None:
        source_detail_id = _as_int(detail.get("_db_detail_id"))
        source_stats = clocks.get("stats") if isinstance(clocks, dict) else None
        source_reset = (
            _as_int(source_stats.get("reset_count")) if isinstance(source_stats, dict) else None
        )
        source_update = (
            _as_int(source_stats.get("update_count")) if isinstance(source_stats, dict) else None
        )
        proof_sequence = _as_int(proof.get("durable_proof_sequence"))
        if (
            source_detail_id is None
            or source_reset is None
            or source_update is None
            or proof_sequence is None
        ):
            raise RuntimeError("startup Alpha resurrection receipt lacks exact proof identity")
        boundary_detail_id = _find_clocks_exact_recovery_boundary_detail_id(
            source_detail_id=int(source_detail_id),
            source_reset_count=int(source_reset),
            source_update_count=int(source_update),
            proof_sequence=int(proof_sequence),
        )
        recovery_receipt = _record_clocks_recovery_receipt(
            boundary_detail_id=boundary_detail_id,
            source_detail=detail,
            source_checkpoint=saved_ppb_checkpoint,
            recovery_mode="STARTUP_ALPHA_RESURRECTION",
            proof=proof,
            alpha_proof_detail_id=boundary_detail_id,
        )

    result = {
        "success": True,
        "mode": "HOLISTIC_INSTRUMENT",
        "alpha_resurrected_this_startup": True,
        "teensy": payload,
        "pi_control": pi_control,
        "gnss_raw": gnss_raw_result,
        "better_buckets": ppb_stage,
        "better_buckets_post_resurrection_refresh": ppb_refresh,
        "proof": proof,
        "recovery_receipt": copy.deepcopy(recovery_receipt),
    }
    verdict = _HolisticInstrumentVerdict(
        alpha_disposition="RESURRECTED",
        alpha_basis="HOLISTIC_ALPHA_N_PLUS_ONE_PROOF",
        alpha_proof=copy.deepcopy(proof),
        observed_campaign_name=str(payload.get("campaign") or "").strip(),
        observed_campaign_state=str(payload.get("campaign_state") or "").strip().upper(),
    )
    return result, verdict



def _finalize_live_recovery_custody_without_campaign(
    *,
    source_detail: Dict[str, Any],
) -> Dict[str, Any]:
    """Promote a proved surviving-producer custody generation with no live campaign."""
    global _recovery_custody_active, _recovery_custody_generation
    global _recovery_custody_entered_at_utc, _recovery_custody_reason
    global _recovery_custody_physical_sequence_regression
    global _recovery_custody_regression_witness
    global _recovery_custody_last_checkpoint

    with _recovery_custody_lock:
        if not _recovery_custody_active:
            return {"active": False, "classified": False, "reason": "no_live_custody"}
        if _recovery_custody_physical_sequence_regression:
            raise RuntimeError(
                "cannot adopt surviving producer after recovery custody proved a "
                "physical sequence regression"
            )
        generation = _recovery_custody_generation
        if not generation:
            raise RuntimeError("active recovery custody has no generation identity")
        source_detail_id = _as_int(source_detail.get("_db_detail_id"))
        if source_detail_id is None or source_detail_id <= 0:
            raise RuntimeError("live producer adoption source lacks durable detail identity")
        routed_checkpoint = copy.deepcopy(_recovery_custody_last_checkpoint)
        if routed_checkpoint is not None and not isinstance(routed_checkpoint, dict):
            raise RuntimeError(
                "active live-producer custody has malformed routed Better-Buckets checkpoint"
            )
        if isinstance(routed_checkpoint, dict):
            routed_checkpoint = _normalize_saved_ppb_checkpoint(routed_checkpoint)

        _wait_for_clocks_persistence_barrier_locked()
        classified_at = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
        with _clocks_persistence_lock:
            with open_db(row_dict=True) as conn:
                cur = conn.cursor()
                if routed_checkpoint is None:
                    cur.execute(
                        """
                        SELECT COUNT(*) AS row_count
                        FROM campaign_detail
                        WHERE campaign_type = %s
                          AND id > %s
                          AND payload #>> '{recovery_custody,generation}' = %s
                        """,
                        (
                            CAMPAIGN_TYPE_TEMPEST,
                            int(source_detail_id),
                            generation,
                        ),
                    )
                    row = cur.fetchone()
                    routed_row_count = int(row["row_count"] if row is not None else -1)
                    if routed_row_count != 0:
                        raise RuntimeError(
                            "live-producer custody has durable routed rows but no "
                            "routed Better-Buckets checkpoint: "
                            f"generation={generation} rows={routed_row_count}"
                        )
                    promoted = 0
                    classification = "LIVE_PRODUCER_ADOPT_NO_ROUTED_ROWS"
                    recovery_config_detail_id = None
                    recovery_config_update_count = None
                else:
                    cur.execute(
                        """
                        UPDATE campaign_detail
                        SET payload = jsonb_set(
                            jsonb_set(
                                jsonb_set(
                                    payload - 'holistic_restore_superseded',
                                    '{recovery_custody,classification}',
                                    to_jsonb(%s::text),
                                    true
                                ),
                                '{recovery_custody,classified_at_utc}',
                                to_jsonb(%s::text),
                                true
                            ),
                            '{recovery_custody,restore_authority}',
                            'true'::jsonb,
                            true
                        )
                        WHERE campaign_type = %s
                          AND id > %s
                          AND payload #>> '{recovery_custody,generation}' = %s
                        """,
                        (
                            "LIVE_PRODUCER_ADOPT_CONTINUITY",
                            classified_at,
                            CAMPAIGN_TYPE_TEMPEST,
                            int(source_detail_id),
                            generation,
                        ),
                    )
                    promoted = int(cur.rowcount or 0)

                    routed_reset = _as_int(routed_checkpoint.get("reset_count"))
                    routed_update = _as_int(routed_checkpoint.get("update_count"))
                    if routed_reset is None or routed_update is None:
                        raise RuntimeError(
                            "routed live-producer checkpoint lacks statistics identity"
                        )
                    cur.execute(
                        """
                        SELECT id, payload
                        FROM campaign_detail
                        WHERE campaign_type = %s
                          AND id > %s
                          AND payload #>> '{recovery_custody,generation}' = %s
                          AND payload #>> '{recovery_custody,restore_authority}' = 'true'
                          AND payload #>> '{clocks,stats,reset_count}' = %s
                          AND payload #>> '{clocks,stats,update_count}' = %s
                          AND NOT (payload @> '{"holistic_restore_superseded":true}'::jsonb)
                        ORDER BY id DESC
                        LIMIT 1
                        """,
                        (
                            CAMPAIGN_TYPE_TEMPEST,
                            int(source_detail_id),
                            generation,
                            str(int(routed_reset)),
                            str(int(routed_update)),
                        ),
                    )
                    routed_row = cur.fetchone()
                    if routed_row is None:
                        raise RuntimeError(
                            "live-producer custody lacks the durable row owning its "
                            "last routed Better-Buckets checkpoint"
                        )
                    routed_state = routed_row["payload"]
                    if isinstance(routed_state, str):
                        routed_state = json.loads(routed_state)
                    if not isinstance(routed_state, dict):
                        raise RuntimeError(
                            "live-producer checkpoint owner payload is not an object"
                        )
                    if not _clocks_state_owns_recovery_config(
                        routed_state, routed_checkpoint
                    ):
                        raise RuntimeError(
                            "live-producer checkpoint owner is not CLOCKS restore authority"
                        )
                    recovery_config_detail_id = int(routed_row["id"])
                    recovery_config_update_count = int(routed_update)
                    _write_clocks_recovery_config(
                        cur,
                        routed_checkpoint,
                        source_detail_id=recovery_config_detail_id,
                    )
                    classification = "LIVE_PRODUCER_ADOPT_CONTINUITY"

        result = {
            "active": False,
            "classified": True,
            "generation": generation,
            "classification": classification,
            "rows_promoted": promoted,
            "rows_superseded": 0,
            "source_detail_id": int(source_detail_id),
            "recovery_config_detail_id": recovery_config_detail_id,
            "recovery_config_update_count": recovery_config_update_count,
        }

        _recovery_custody_active = False
        _recovery_custody_generation = None
        _recovery_custody_entered_at_utc = None
        _recovery_custody_reason = None
        _recovery_custody_physical_sequence_regression = False
        _recovery_custody_regression_witness = {}
        _recovery_custody_last_checkpoint = None

    _diag["recovery_custody_active"] = False
    _diag["recovery_custody_generation"] = None
    _diag["recovery_custody_rows_promoted"] = (
        _diag.get("recovery_custody_rows_promoted", 0) + int(promoted)
    )
    _diag["last_recovery_custody"] = copy.deepcopy(result)
    logging.info(
        "✅ [recovery/custody] surviving producer adopted without campaign: "
        "generation=%s promoted=%d source_id=%d",
        generation,
        promoted,
        int(source_detail_id),
    )
    return result


def _rearm_surviving_clocks_campaign(
    *,
    snapshot_detail: Dict[str, Any],
    active_campaign: Dict[str, Any],
) -> Tuple[Dict[str, Any], Optional[Dict[str, Any]]]:
    """Restore Beta recording execution on a proved surviving Alpha producer.

    This is intentionally narrower than RECOVER.  The Alpha instrument/statistical
    lifetime has already been proved continuous, so the Teensy must not receive a
    structured instrument restore or a fresh SmartZero epoch.  Pi durable campaign
    intent remains authoritative; REARM projects only the campaign presentation
    coordinate across the observer outage, then waits for the exact projected N+1
    campaign row before reopening Pi campaign processing.
    """
    global _campaign_active, _accepted_pps_vclock_count
    global _last_pps_vclock_count_seen

    campaign_name = str(active_campaign.get("campaign") or "").strip()
    if not campaign_name:
        raise RuntimeError("surviving campaign rearm requires durable active campaign identity")

    source_detail_id = _as_int(snapshot_detail.get("_db_detail_id"))
    if source_detail_id is None or source_detail_id <= 0:
        raise RuntimeError("surviving campaign rearm source lacks durable CLOCKS identity")

    last_tb = _recovery_timebase_from_clocks_state(
        snapshot_detail,
        campaign_name,
        source_detail_id=int(source_detail_id),
        source_campaign=campaign_name,
    )
    recovery_snapshot = _recovery_timebase_snapshot(last_tb)
    if not recovery_snapshot.get("recoverable"):
        raise RuntimeError(
            "surviving Alpha campaign rearm lacks a recoverable durable campaign boundary"
        )

    last_durable_count = int(recovery_snapshot["last_pps_vclock_count"])
    last_gnss_time_str = str(recovery_snapshot["last_gnss_time_str"])

    drained = _drain_timebase_ingress()
    if drained:
        logging.info(
            "🧹 [rearm] drained %d stale campaign delta(s) before surviving-Alpha rearm",
            drained,
        )

    # Use the same GNSS-aligned projection boundary as dead-producer RECOVER.
    # The only difference is ownership: Alpha remains untouched and only Beta's
    # public campaign transform is reinstalled.
    now_frac = time.time() % 1.0
    sleep_to_boundary = (1.0 - now_frac) + 0.050
    time.sleep(sleep_to_boundary)
    current_gnss_time_str = _wait_for_gnss_time()
    current_gnss_utc = datetime.fromisoformat(
        current_gnss_time_str.replace("Z", "+00:00")
    )
    desired_state = _project_clocks_recovery_snapshot(
        recovery_snapshot, current_gnss_utc
    )

    elapsed_seconds = int(desired_state["elapsed_seconds"])
    base_count = int(desired_state["recover_base_pps_vclock_count"])
    expected_first = int(desired_state["expected_first_public_pps_vclock_count"])
    rearm_args = dict(desired_state["teensy_recover_args"])

    _begin_sync_wait(expected_pps=expected_first)
    try:
        response = _request_teensy_rearm(
            campaign_name,
            base_count,
            rearm_args,
        )
        payload = response.get("payload") if isinstance(response, dict) else None
        payload = payload if isinstance(payload, dict) else {}

        frag, waited_s = _end_sync_wait(timeout_s=SYNC_RECOVER_TIMEOUT_S)
        first_public_count = _as_int(frag.get("public_count"))
        if first_public_count != expected_first:
            raise RuntimeError(
                "surviving Alpha campaign REARM did not resume at exact projected N+1: "
                f"expected={expected_first} observed={first_public_count}"
            )

        seed_pps_vclock_count = int(first_public_count) - 1
        if seed_pps_vclock_count < last_durable_count:
            raise RuntimeError(
                "surviving Alpha campaign REARM baseline regressed behind durable state"
            )
        gnss_raw_projection = _gnss_raw_recovery_project_seed(
            last_tb=last_tb,
            last_pps_vclock_count=last_durable_count,
            seed_pps_vclock_count=seed_pps_vclock_count,
        )
        seed_gnss_ns = seed_pps_vclock_count * NS_PER_SECOND
        if not _restore_gnss_raw_from_last_timebase(
            last_tb=last_tb,
            projected_gnss_ns=int(seed_gnss_ns),
            projected_gnss_raw_ns=int(
                gnss_raw_projection.get("seed_raw_ns") or 0
            ),
            projected_gnss_raw_ref_ns=int(
                gnss_raw_projection.get("seed_ref_ns") or seed_gnss_ns
            ),
            projection_details=gnss_raw_projection,
        ):
            raise RuntimeError("surviving Alpha campaign REARM GNSS_RAW projection failed")

        recovery_custody: Optional[Dict[str, Any]] = None
        if _recovery_custody_snapshot().get("active"):
            recovery_custody = _finalize_recovery_clocks_custody(
                last_tb=last_tb,
                campaign_name=campaign_name,
                first_public_count=int(first_public_count),
                recover_mode="LIVE_PRODUCER_ADOPT",
            )

        _accepted_pps_vclock_count = seed_pps_vclock_count
        _last_pps_vclock_count_seen = seed_pps_vclock_count
        _diag["accepted_pps_count"] = seed_pps_vclock_count
        _diag["accepted_pps_vclock_count"] = seed_pps_vclock_count
        _gnss_canary_reset()
        _clear_start_wait_state()
        _campaign_active = True
        _arm_timebase_silence_watch("SURVIVING_ALPHA_CAMPAIGN_REARM")
        _sync_resume_event.set()

        adoption = {
            "state": "STARTED",
            "mode": "SURVIVING_ALPHA_CAMPAIGN_REARM",
            "campaign": campaign_name,
            "source_detail_id": int(source_detail_id),
            "last_durable_pps_vclock_count": int(last_durable_count),
            "last_durable_gnss_time": last_gnss_time_str,
            "current_gnss_time": current_gnss_time_str,
            "elapsed_seconds": int(elapsed_seconds),
            "baseline_pps_vclock_count": seed_pps_vclock_count,
            "first_public_pps_vclock_count": int(first_public_count),
            "waited_s": round(float(waited_s), 3),
            "gnss_raw_projection": gnss_raw_projection,
            "teensy": copy.deepcopy(payload),
            "alpha_mutated": False,
            "campaign_execution_mutated": True,
        }
        logging.info(
            "✅ [rearm] surviving Alpha preserved; campaign '%s' rearmed at "
            "base=%d first_public=%d after %d elapsed GNSS second(s)",
            campaign_name,
            seed_pps_vclock_count,
            int(first_public_count),
            elapsed_seconds,
        )
        return adoption, recovery_custody
    except Exception:
        _campaign_active = False
        _clear_sync_wait()
        raise


def _adopt_surviving_clocks_producer(
    *,
    snapshot_detail: Dict[str, Any],
    active_campaign: Optional[Dict[str, Any]],
    instrument_verdict: _HolisticInstrumentVerdict,
) -> Dict[str, Any]:
    """Prove and adopt a surviving CLOCKS producer without mutating it."""
    global _campaign_active, _accepted_pps_vclock_count
    global _last_pps_vclock_count_seen
    global _post_recovery_science_confirmation_pending
    global _post_recovery_science_confirmation_campaign
    global _post_recovery_first_public_pps_vclock_count

    if str(instrument_verdict.alpha_disposition).strip().upper() != "SURVIVED":
        raise RuntimeError("live CLOCKS adoption requires a surviving-producer verdict")
    carried = instrument_verdict.alpha_proof
    if not isinstance(carried, dict) or carried.get("proved") is not True:
        raise RuntimeError("live CLOCKS adoption lacks proved producer continuity")

    clocks = _clocks_payload(snapshot_detail)
    gnss_raw = _clocks_gnss_raw_payload(snapshot_detail)
    if not _canonical_instrument_restore_ready(
        clocks,
        include_control=True,
        detail_id=_as_int(snapshot_detail.get("_db_detail_id")),
    ) or gnss_raw is None:
        raise RuntimeError("live CLOCKS adoption snapshot lacks Pi recovery custody")

    with _clocks_lock:
        live_state = copy.deepcopy(_latest_clocks)
        live_received_monotonic = _latest_clocks_received_monotonic
        live_received_utc = _latest_clocks_received_utc
    live_age_s = (
        None
        if live_received_monotonic is None
        else max(0.0, time.monotonic() - live_received_monotonic)
    )
    descendant = _alpha_survival_lineage_court(
        clocks,
        live_state,
        live_age_s=live_age_s,
    )
    if descendant.get("proved") is not True:
        raise RuntimeError(
            "CLOCKS producer no longer proves lawful descent from the snapshot: "
            f"{descendant!r}"
        )

    # Everything below this point is Pi-owned state.  Better-Buckets resumes from
    # the durable singleton plus the ordinary producer-authored deltas already
    # carried on CLOCKS_FRAGMENT.  Do not query Alpha with PPB_EXPORT_* merely to
    # accelerate convergence after an observer gap.
    with _ppb_checkpoint_lock:
        ppb_runtime = _ppb_checkpoint_runtime_ensure_locked()
        ppb_snapshot = _ppb_checkpoint_snapshot_locked(ppb_runtime)
    ppb_refresh = {
        "refreshed": False,
        "basis": "CLOCKS_FRAGMENT_DELTA_CUSTODY",
        "teensy_rpc_used": False,
        "recoverable": bool(ppb_snapshot.get("recoverable")),
        "status": ppb_snapshot.get("status"),
        "reset_count": ppb_snapshot.get("reset_count"),
        "update_count": ppb_snapshot.get("update_count"),
        "second_count": int(ppb_snapshot.get("second_count") or 0),
        "expected_second_count": int(ppb_snapshot.get("expected_second_count") or 0),
        "minute_count": int(ppb_snapshot.get("minute_count") or 0),
        "expected_minute_count": int(ppb_snapshot.get("expected_minute_count") or 0),
        "gap_count": int(ppb_snapshot.get("gap_count") or 0),
    }
    pi_control = _dac_restore_control_from_clocks(clocks, realize=True)
    gnss_raw_result = _restore_gnss_raw_payload(gnss_raw)
    if not gnss_raw_result.get("restored"):
        raise RuntimeError(f"CLOCKS live-adopt GNSS_RAW restore failed: {gnss_raw_result}")

    _clocks_holistic_restore_proof_pending.clear()
    _clocks_holistic_restore_proof_committed.clear()
    startup_custody = _quarantine_startup_clocks_custody_surviving_alpha(
        "surviving_producer_pi_state_not_yet_restored"
    )

    _post_recovery_science_confirmation_pending = False
    _post_recovery_science_confirmation_campaign = None
    _post_recovery_first_public_pps_vclock_count = None

    campaign_name = (
        str(active_campaign.get("campaign") or "").strip()
        if isinstance(active_campaign, dict)
        else ""
    )
    live_campaign = _state_campaign(live_state)
    live_campaign_name = (
        _tempest_campaign_name(live_campaign)
        if isinstance(live_campaign, dict) and live_campaign
        else ""
    )
    live_campaign_state = str(
        live_campaign.get("state") if isinstance(live_campaign, dict) else ""
    ).strip().upper()
    live_public_count = _as_int(
        live_campaign.get("public_count") if isinstance(live_campaign, dict) else None
    )

    campaign_adoption: Dict[str, Any]
    recovery_custody: Optional[Dict[str, Any]] = None
    if live_campaign_state == "STARTED":
        if not campaign_name or live_campaign_name != campaign_name:
            raise RuntimeError(
                "surviving CLOCKS_FRAGMENT owns an active campaign that does not "
                "match durable Pi intent"
            )
        if (
            live_campaign_name != campaign_name
            or live_campaign_state != "STARTED"
            or live_public_count is None
            or live_public_count <= 0
            or live_age_s is None
            or live_age_s > CLOCKS_PREFLIGHT_MAX_AGE_S
        ):
            raise RuntimeError(
                "fresh canonical CLOCKS campaign testimony disappeared before adoption: "
                f"canonical={live_campaign_name or 'NONE'}/{live_campaign_state or 'NONE'} "
                f"public_count={live_public_count} age_s={live_age_s}"
            )

        expected_next = int(live_public_count) + 1
        _begin_sync_wait(expected_pps=expected_next)
        try:
            frag, waited_s = _end_sync_wait(timeout_s=SYNC_RECOVER_TIMEOUT_S)
            first_public_count = _as_int(frag.get("public_count"))
            if first_public_count is None or first_public_count < expected_next:
                raise RuntimeError(
                    "surviving CLOCKS handoff returned invalid campaign count: "
                    f"expected>={expected_next} observed={first_public_count}"
                )

            seed_pps_vclock_count = int(first_public_count) - 1
            last_tb, recovery_snapshot, skipped = (
                _load_last_recoverable_tempest_detail(campaign_name)
            )
            if not recovery_snapshot.get("recoverable"):
                raise RuntimeError(
                    "surviving CLOCKS campaign lacks a recoverable Pi GNSS_RAW baseline"
                )
            last_durable_count = int(recovery_snapshot["last_pps_vclock_count"])
            if seed_pps_vclock_count < last_durable_count:
                raise RuntimeError(
                    "surviving CLOCKS campaign baseline regressed behind durable state"
                )
            gnss_raw_projection = _gnss_raw_recovery_project_seed(
                last_tb=last_tb,
                last_pps_vclock_count=last_durable_count,
                seed_pps_vclock_count=seed_pps_vclock_count,
            )
            seed_gnss_ns = seed_pps_vclock_count * NS_PER_SECOND
            if not _restore_gnss_raw_from_last_timebase(
                last_tb=last_tb,
                projected_gnss_ns=int(seed_gnss_ns),
                projected_gnss_raw_ns=int(
                    gnss_raw_projection.get("seed_raw_ns") or 0
                ),
                projected_gnss_raw_ref_ns=int(
                    gnss_raw_projection.get("seed_ref_ns") or seed_gnss_ns
                ),
                projection_details=gnss_raw_projection,
            ):
                raise RuntimeError("surviving CLOCKS campaign GNSS_RAW projection failed")

            if _recovery_custody_snapshot().get("active"):
                recovery_custody = _finalize_recovery_clocks_custody(
                    last_tb=last_tb,
                    campaign_name=campaign_name,
                    first_public_count=int(first_public_count),
                    recover_mode="LIVE_PRODUCER_ADOPT",
                )

            _accepted_pps_vclock_count = seed_pps_vclock_count
            _last_pps_vclock_count_seen = seed_pps_vclock_count
            _diag["accepted_pps_count"] = seed_pps_vclock_count
            _diag["accepted_pps_vclock_count"] = seed_pps_vclock_count
            _gnss_canary_reset()
            _clear_start_wait_state()
            _campaign_active = True
            _arm_timebase_silence_watch("LIVE_PRODUCER_ADOPT")
            _sync_resume_event.set()

            campaign_adoption = {
                "state": "STARTED",
                "campaign": campaign_name,
                "baseline_pps_vclock_count": seed_pps_vclock_count,
                "first_public_pps_vclock_count": int(first_public_count),
                "waited_s": round(float(waited_s), 3),
                "skipped_unrecoverable_rows": int(skipped),
                "gnss_raw_projection": gnss_raw_projection,
            }
        except Exception:
            _campaign_active = False
            _clear_sync_wait()
            raise
    elif live_campaign_state in {"", "STOPPED", "IDLE", "NONE"}:
        if live_campaign_name and campaign_name and live_campaign_name != campaign_name:
            raise RuntimeError(
                "stopped surviving CLOCKS_FRAGMENT retains a different campaign "
                f"identity: producer={live_campaign_name!r} durable={campaign_name!r}"
            )

        if active_campaign is not None:
            raise RuntimeError(
                "durable TEMPEST intent is active but the proved surviving "
                "CLOCKS_FRAGMENT stream says Beta is stopped; zero-RPC live recovery "
                "refuses CLOCKS.REARM or any other Teensy command"
            )
        else:
            if _recovery_custody_snapshot().get("active"):
                recovery_custody = _finalize_live_recovery_custody_without_campaign(
                    source_detail=snapshot_detail,
                )

            _campaign_active = False
            _accepted_pps_vclock_count = None
            _last_pps_vclock_count_seen = None
            _diag["accepted_pps_count"] = None
            _diag["accepted_pps_vclock_count"] = None
            _clear_sync_wait()
            _clear_start_wait_state()
            _clear_flash_cut_wait_state()
            _gnss_canary_reset()
            campaign_adoption = {
                "state": "STOPPED",
                "campaign": live_campaign_name or None,
                "master_mutated": False,
                "alpha_mutated": False,
                "campaign_execution_mutated": False,
            }
    else:
        raise RuntimeError(
            f"surviving CLOCKS_FRAGMENT producer is in non-adoptable campaign state {live_campaign_state!r}"
        )

    result = {
        "success": True,
        "mode": "LIVE_PRODUCER_ADOPT",
        "producer_reuse": True,
        "producer_mutated": False,
        "producer_mutation_commands": [],
        "teensy_rpc_used": False,
        "alpha_mutated": False,
        "snapshot_detail_id": _as_int(snapshot_detail.get("_db_detail_id")),
        "producer_descendant_proof": descendant,
        # Compatibility field names; values come from CLOCKS_FRAGMENT, not RPC.
        "firmware_campaign_state": live_campaign_state or "STOPPED",
        "firmware_campaign": live_campaign_name or None,
        "campaign_witness_source": "CLOCKS_FRAGMENT",
        "canonical_probe_sequence": _as_int(live_state.get("sequence")),
        "canonical_probe_received_at_utc": live_received_utc,
        "canonical_probe_age_s": (
            None if live_age_s is None else round(float(live_age_s), 3)
        ),
        "better_buckets_live_refresh": ppb_refresh,
        "pi_control": pi_control,
        "gnss_raw": gnss_raw_result,
        "startup_custody": startup_custody,
        "recovery_custody": recovery_custody,
        "campaign_adoption": campaign_adoption,
    }
    _diag["last_recovery"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        **copy.deepcopy(result),
    }
    logging.info(
        "✅ [holistic restore] surviving Alpha/Beta adopted from CLOCKS_FRAGMENT with "
        "zero Teensy RPC: campaign=%s observed_state=%s",
        live_campaign_name or "NONE",
        live_campaign_state or "STOPPED",
    )
    return result



def _surrender_unresurrectable_alpha_lineage(
    *,
    verdict: AlphaResurrectionImpossible,
    active_campaign: Optional[Dict[str, Any]],
    source: str,
) -> Dict[str, Any]:
    """Terminate dead Alpha ancestry and birth a new durable statistics epoch.

    This is not repair of the old lineage. Every old CLOCKS/TEMPEST row remains
    historical evidence, but none may remain future restore authority. The active
    TEMPEST namespace, if any, ends at its last durable row. Only the singleton
    continuation image is deleted; then the existing transitive STATS_RESET epoch-
    birth path proves update_count=1 durably before ordinary service resumes.
    """
    global _campaign_active, _ppb_checkpoint_runtime
    global _accepted_pps_vclock_count, _last_pps_vclock_count_seen
    global _recovery_custody_active, _recovery_custody_generation
    global _recovery_custody_entered_at_utc, _recovery_custody_reason
    global _recovery_custody_physical_sequence_regression
    global _recovery_custody_regression_witness, _recovery_custody_last_checkpoint

    reason = str(verdict.reason)
    evidence = copy.deepcopy(verdict.details)
    surrendered_at = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    campaign_name = (
        str(active_campaign.get("campaign") or "").strip()
        if isinstance(active_campaign, dict)
        else ""
    )
    surrender_started = time.monotonic()

    def note_surrender_phase(phase: str, **detail: Any) -> None:
        snapshot = {
            "phase": str(phase),
            "elapsed_s": round(time.monotonic() - surrender_started, 3),
            "campaign": campaign_name or None,
            "source": str(source),
            **copy.deepcopy(detail),
        }
        _diag["last_alpha_lineage_surrender_progress"] = snapshot
        logging.info(
            "🔬 [clocks/lineage-surrender] phase=%s elapsed=%.3fs campaign=%s detail=%s",
            phase,
            float(snapshot["elapsed_s"]),
            campaign_name or "NONE",
            detail or {},
        )

    note_surrender_phase("BEGIN")
    _request_teensy_stop_best_effort()
    _request_teensy_recover_abort_best_effort("alpha_lineage_surrender")
    _campaign_active = False
    _clear_start_wait_state()
    _clear_flash_cut_wait_state()
    _clear_sync_wait()
    candidate_drained = _reset_trackers()
    _accepted_pps_vclock_count = None
    _last_pps_vclock_count_seen = None
    _diag["accepted_pps_count"] = None
    _diag["accepted_pps_vclock_count"] = None
    _gnss_canary_reset()

    # Freeze ordinary persistence while the old restore-authority namespace is
    # atomically retired. New epoch birth will reopen it before issuing STATS_RESET.
    _clocks_persistence_enabled.clear()
    _clocks_epoch_birth_pending.clear()
    _clocks_epoch_birth_committed.clear()
    _clocks_holistic_restore_proof_pending.clear()
    _clocks_holistic_restore_proof_committed.clear()
    _startup_instrument_restore_hold.clear()
    _startup_physical_lifetime_unclassified.clear()

    startup_custody = _retire_startup_clocks_custody(
        "alpha_resurrection_impossible_lineage_surrender"
    )

    recovery_lock_started = time.monotonic()
    note_surrender_phase("WAIT_RECOVERY_CUSTODY_LOCK")
    with _recovery_custody_lock:
        note_surrender_phase(
            "RECOVERY_CUSTODY_LOCK_ACQUIRED",
            waited_s=round(time.monotonic() - recovery_lock_started, 6),
        )
        recovery_custody = _recovery_custody_snapshot_locked()
        _recovery_custody_active = False
        _recovery_custody_generation = None
        _recovery_custody_entered_at_utc = None
        _recovery_custody_reason = None
        _recovery_custody_physical_sequence_regression = False
        _recovery_custody_regression_witness = {}
        _recovery_custody_last_checkpoint = None

    ppb_lock_started = time.monotonic()
    note_surrender_phase("WAIT_PPB_CHECKPOINT_LOCK")
    with _ppb_checkpoint_lock:
        note_surrender_phase(
            "PPB_CHECKPOINT_LOCK_ACQUIRED",
            waited_s=round(time.monotonic() - ppb_lock_started, 6),
        )
        _ppb_checkpoint_runtime = _ppb_checkpoint_new_runtime(
            reason="ALPHA_LINEAGE_SURRENDER_FRESH_EPOCH"
        )
    note_surrender_phase("PPB_CHECKPOINT_RESET")

    # A persistence item may already have been routed before the freeze. Drain it
    # under the writer lock, then make the durable cut in one DB transaction.  Do
    # not rewrite historical campaign_detail rows: one monotonic detail-id tombstone
    # retires the entire old restore-authority universe in O(1) database work.
    cut_started = time.monotonic()
    persistence_wait_started = time.monotonic()
    next_persistence_wait_log = persistence_wait_started + 10.0
    note_surrender_phase(
        "WAIT_PERSISTENCE_LOCK",
        lock_locked=_clocks_persistence_lock.locked(),
        persistence_worker=copy.deepcopy(_diag.get("clocks_persistence_worker") or {}),
    )
    while not _clocks_persistence_lock.acquire(timeout=1.0):
        now = time.monotonic()
        if now >= next_persistence_wait_log:
            worker = copy.deepcopy(_diag.get("clocks_persistence_worker") or {})
            waited = now - persistence_wait_started
            _diag["last_alpha_lineage_surrender_progress"] = {
                "phase": "WAIT_PERSISTENCE_LOCK",
                "elapsed_s": round(now - surrender_started, 3),
                "persistence_lock_waited_s": round(waited, 3),
                "campaign": campaign_name or None,
                "source": str(source),
                "persistence_worker": worker,
            }
            logging.warning(
                "🔬 [clocks/lineage-surrender] still waiting for CLOCKS persistence "
                "lock after %.1fs; worker_phase=%s worker_sequence=%s worker_detail=%s",
                waited,
                worker.get("phase") or "UNKNOWN",
                worker.get("sequence"),
                worker,
            )
            next_persistence_wait_log = now + 10.0
    try:
        note_surrender_phase(
            "PERSISTENCE_LOCK_ACQUIRED",
            waited_s=round(time.monotonic() - persistence_wait_started, 6),
        )
        state_drained = _drain_clocks_persistence_queue()
        note_surrender_phase("PERSISTENCE_QUEUE_DRAINED", rows=state_drained)
        note_surrender_phase("DB_OPEN")
        with open_db(row_dict=True) as conn:
            cur = conn.cursor()
            note_surrender_phase("DB_CUTOFF_SELECT")
            cur.execute(
                """
                SELECT id AS cutoff_detail_id
                FROM campaign_detail
                WHERE campaign_type = %s
                ORDER BY id DESC
                LIMIT 1
                """,
                (CAMPAIGN_TYPE_TEMPEST,),
            )
            cutoff_row = cur.fetchone()
            note_surrender_phase("DB_CUTOFF_SELECTED")
            lineage_cutoff_detail_id = (
                int(cutoff_row.get("cutoff_detail_id") or 0)
                if isinstance(cutoff_row, dict)
                else 0
            )

            master_rows_stopped = 0
            if campaign_name:
                note_surrender_phase("DB_CAMPAIGN_STOP")
                cur.execute(
                    """
                    UPDATE campaign_master
                    SET active = false,
                        payload = payload
                            || jsonb_build_object(
                                'stopped_at', to_jsonb(%s::text),
                                'continuity_surrendered', true,
                                'continuity_surrendered_at', to_jsonb(%s::text),
                                'continuity_surrender_reason', to_jsonb(%s::text),
                                'alpha_lineage_resurrection_impossible', true
                            )
                            || CASE
                                WHEN payload ? 'report'
                                THEN jsonb_build_object(
                                    'report',
                                    (payload->'report') || '{"campaign_state":"STOPPED"}'::jsonb
                                )
                                ELSE '{}'::jsonb
                               END
                    WHERE campaign_type = %s
                      AND campaign = %s
                      AND active = true
                    """,
                    (
                        surrendered_at,
                        surrendered_at,
                        reason,
                        CAMPAIGN_TYPE_TEMPEST,
                        campaign_name,
                    ),
                )
                master_rows_stopped = int(cur.rowcount or 0)
                note_surrender_phase("DB_CAMPAIGN_STOPPED", rows=master_rows_stopped)
                if master_rows_stopped != 1:
                    raise RuntimeError(
                        "Alpha lineage surrender did not retire exactly one active "
                        f"TEMPEST campaign {campaign_name!r}: rows={master_rows_stopped}"
                    )

            note_surrender_phase("DB_CUTOFF_WRITE")
            lineage_cutoff = _write_alpha_lineage_cutoff_config(
                cur,
                cutoff_detail_id=lineage_cutoff_detail_id,
                terminated_at_utc=surrendered_at,
                reason=reason,
                source=source,
                campaign=campaign_name or None,
            )
            note_surrender_phase(
                "DB_CUTOFF_WRITTEN", cutoff_detail_id=lineage_cutoff_detail_id
            )

            note_surrender_phase("DB_RECOVERY_CONFIG_DELETE")
            cur.execute(
                "DELETE FROM config WHERE config_key = %s",
                (CLOCKS_RECOVERY_CONFIG_KEY,),
            )
            recovery_config_deleted = int(cur.rowcount or 0)
            note_surrender_phase(
                "DB_RECOVERY_CONFIG_DELETED", rows=recovery_config_deleted
            )
            if recovery_config_deleted > 1:
                raise RuntimeError(
                    f"config.{CLOCKS_RECOVERY_CONFIG_KEY} is not a singleton: "
                    f"rows={recovery_config_deleted}"
                )
            note_surrender_phase("DB_COMMIT")
        note_surrender_phase("DB_COMMITTED")
    finally:
        _clocks_persistence_lock.release()
        note_surrender_phase("PERSISTENCE_LOCK_RELEASED")
    lineage_cut_s = time.monotonic() - cut_started
    historical_rows_superseded = 0

    logging.critical(
        "🧭 [clocks] Alpha continuity surrendered without deleting or rewriting history: "
        "reason=%s campaign=%s stopped=%d cutoff_detail_id=%d historical_rows_rewritten=0 "
        "recovery_config_deleted=%d cut_s=%.3f; establishing fresh durable statistics epoch",
        reason,
        campaign_name or "NONE",
        master_rows_stopped,
        lineage_cutoff_detail_id,
        recovery_config_deleted,
        lineage_cut_s,
    )

    epoch = _establish_fresh_durable_stats_epoch(
        source="CLOCKS.ALPHA_LINEAGE_SURRENDER",
    )

    result = {
        "schema": "PI_CLOCKS_ALPHA_LINEAGE_SURRENDER_V1",
        "surrendered_at_utc": surrendered_at,
        "reason": reason,
        "source": str(source),
        "old_lineage_preserved_as_history": True,
        "raw_postgresql_replay_used": False,
        "campaign": campaign_name or None,
        "campaign_stopped": bool(master_rows_stopped),
        "campaign_master_rows_stopped": master_rows_stopped,
        "historical_rows_superseded_for_restore": historical_rows_superseded,
        "historical_rows_rewritten": 0,
        "lineage_cutoff_detail_id": int(lineage_cutoff_detail_id),
        "lineage_cutoff": copy.deepcopy(lineage_cutoff),
        "lineage_cut_s": round(float(lineage_cut_s), 6),
        "recovery_config_deleted": bool(recovery_config_deleted),
        "candidate_ingress_drained": int(candidate_drained),
        "pending_state_rows_drained": int(state_drained),
        "startup_custody": copy.deepcopy(startup_custody),
        "recovery_custody_before_surrender": copy.deepcopy(recovery_custody),
        "failure_evidence": evidence,
        "fresh_epoch": copy.deepcopy(epoch),
        "next_action": "CLOCKS is running on a fresh Alpha; start a new TEMPEST campaign when desired.",
    }
    _diag["alpha_lineage_surrenders"] = _diag.get("alpha_lineage_surrenders", 0) + 1
    _diag["last_alpha_lineage_surrender"] = copy.deepcopy(result)
    return result


def _holistic_restore(
    *,
    preverified_location: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Converge CLOCKS from one durable snapshot and current producer testimony."""
    active_campaign = _get_active_campaign()
    detail, skipped = _read_latest_recoverable_clocks_state()
    result: Dict[str, Any] = {
        "schema": "PI_CLOCKS_HOLISTIC_RESTORE_V1",
        "active_campaign": active_campaign.get("campaign") if active_campaign else None,
        "skipped_unrecoverable_details": int(skipped),
        "location": None,
        "instrument": None,
        "campaign": None,
    }

    if preverified_location is not None:
        result["location"] = copy.deepcopy(preverified_location)

    if active_campaign is None:
        if preverified_location is None:
            logging.info(
                "📍 [holistic restore] no active campaign; restoring SYSTEM location/GF-8802 mode"
            )
            result["location"] = _ensure_system_location(context="HOLISTIC_RESTORE_IDLE")
        else:
            logging.info(
                "📍 [holistic restore] no active campaign; using cold-start verified "
                "SYSTEM location/GF-8802 testimony"
            )
    else:
        logging.info(
            "📍 [holistic restore] active campaign '%s'; using cold-start location "
            "testimony when available",
            active_campaign.get("campaign"),
        )

    instrument_verdict: Optional[_HolisticInstrumentVerdict] = None
    try:
        if detail is not None:
            logging.info(
                "♻️ [holistic restore] reconciling canonical CLOCKS sequence=%s "
                "(skipped_unrecoverable=%d)",
                detail.get("sequence"), skipped,
            )
            instrument_result, instrument_verdict = _restore_instrument_from_clocks(detail)
            result["instrument"] = instrument_result

            if (
                str(instrument_verdict.alpha_disposition).strip().upper()
                == "SURVIVED"
            ):
                adoption = _adopt_surviving_clocks_producer(
                    snapshot_detail=detail,
                    active_campaign=active_campaign,
                    instrument_verdict=instrument_verdict,
                )
                result["instrument"] = adoption
                result["campaign"] = copy.deepcopy(adoption.get("campaign_adoption"))
        else:
            logging.info(
                "ℹ️ [holistic restore] no structured-recoverable CLOCKS detail; "
                "establishing a fresh durable statistics epoch"
            )
            _retire_startup_clocks_custody("fresh_statistics_epoch")
            result["instrument"] = _establish_fresh_durable_stats_epoch()
    except AlphaResurrectionImpossible as exc:
        surrender = _surrender_unresurrectable_alpha_lineage(
            verdict=exc,
            active_campaign=active_campaign,
            source="STARTUP_INSTRUMENT_RESTORE",
        )
        result["instrument"] = {
            "success": True,
            "mode": "ALPHA_LINEAGE_SURRENDER_FRESH_EPOCH",
            "producer_reuse": False,
            "producer_resurrected_this_startup": False,
            "lineage_surrender": copy.deepcopy(surrender),
            "fresh_epoch": copy.deepcopy(surrender.get("fresh_epoch") or {}),
        }
        result["campaign"] = {
            "state": "STOPPED",
            "campaign": surrender.get("campaign"),
            "continuity_surrendered": True,
            "reason": exc.reason,
        }
        result["active_campaign"] = None
        active_campaign = None
    finally:
        if not _hard_failure_active() and not _startup_clocks_custody_unresolved():
            _clocks_persistence_enabled.set()
        _clocks_holistic_restore_proof_pending.clear()

    if (
        active_campaign is not None
        and instrument_verdict is not None
        and str(instrument_verdict.alpha_disposition).strip().upper() != "SURVIVED"
    ):
        try:
            campaign_result = _restore_active_campaign_state(
                preverified_location=preverified_location,
                startup_instrument_verdict=instrument_verdict,
            )
        except AlphaResurrectionImpossible as exc:
            surrender = _surrender_unresurrectable_alpha_lineage(
                verdict=exc,
                active_campaign=active_campaign,
                source="STARTUP_COMBINED_CAMPAIGN_RESTORE",
            )
            result["instrument"] = {
                "success": True,
                "mode": "ALPHA_LINEAGE_SURRENDER_FRESH_EPOCH",
                "producer_reuse": False,
                "producer_resurrected_this_startup": False,
                "lineage_surrender": copy.deepcopy(surrender),
                "fresh_epoch": copy.deepcopy(surrender.get("fresh_epoch") or {}),
            }
            result["campaign"] = {
                "state": "STOPPED",
                "campaign": surrender.get("campaign"),
                "continuity_surrendered": True,
                "reason": exc.reason,
            }
            result["active_campaign"] = None
            campaign_result = result["campaign"]
        else:
            result["campaign"] = campaign_result
        if (
            str(instrument_verdict.alpha_disposition).strip().upper() == "LOST"
            and isinstance(campaign_result.get("combined_instrument_restore"), dict)
        ):
            result["instrument"] = copy.deepcopy(
                campaign_result["combined_instrument_restore"]
            )

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
            "recover_proof_stalled": recovery.get("proof_stalled"),
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

    # campaign_master is an intentional read model and retains the latest accepted
    # campaign report. Baseline comparisons reference another master row by ID.
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
    """Satisfy RECOVER sync after the final court preserves a candidate row."""
    global _sync_fragment

    with _sync_lock:
        if _sync_expected_pps_vclock is None:
            return False

        match = int(pps_vclock_count) >= int(_sync_expected_pps_vclock)
        if match:
            logging.info(
                "✅ [recovery] first final-court-preserved CLOCKS_FRAGMENT candidate reached sync: count=%d expected>=%d",
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
    dead-producer recovery should project it from its own reference surface
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


def _fetch_teensy_smartzero_status() -> Dict[str, Any]:
    """Return INTERRUPT.REPORT_SMARTZERO payload for startup diagnostics only.

    SmartZero readiness remains owned entirely by firmware/CLOCKS. This helper
    is observational: callers use it only when emitting an already-due progress
    log, so diagnostic visibility cannot become a startup gate or high-rate RPC
    workload.
    """
    try:
        resp = send_command(
            machine="TEENSY",
            subsystem="INTERRUPT",
            command="REPORT_SMARTZERO",
            retries=1,
            retry_delay_s=0.0,
        )
        if resp.get("success"):
            payload = resp.get("payload", {})
            return payload if isinstance(payload, dict) else {}
    except Exception:
        logging.debug(
            "⚠️ [clocks/startup] REPORT_SMARTZERO diagnostic poll failed (ignored)"
        )
    return {}


def _smartzero_lane_progress(
    report: Dict[str, Any],
    prefix: str,
    label: str,
) -> Dict[str, Any]:
    """Extract one compact lane court from INTERRUPT.REPORT_SMARTZERO."""
    state = str(report.get(f"{prefix}_state") or "UNKNOWN").strip().upper()
    decision = str(report.get(f"{prefix}_last_decision") or "NONE").strip().upper()
    return {
        "lane": label,
        "state": state,
        "decision": decision,
        "samples": _as_int(report.get(f"{prefix}_sample_count")) or 0,
        "intervals": _as_int(report.get(f"{prefix}_valid_interval_count")) or 0,
        "quorum": _as_int(report.get(f"{prefix}_quorum_count")) or 0,
        "quorum_span": _as_int(report.get(f"{prefix}_quorum_span_cycles")) or 0,
        "closest_three_span": _as_int(
            report.get(f"{prefix}_closest_three_span_cycles")
        ) or 0,
        "accepted_interval": _as_int(
            report.get(f"{prefix}_accepted_interval_cycles")
        ) or 0,
        "accepted_history_age": _as_int(
            report.get(f"{prefix}_accepted_history_age")
        ) or 0,
    }


def _smartzero_progress_snapshot(report: Dict[str, Any]) -> Dict[str, Any]:
    """Normalize the three parallel SmartZero lane courts for Pi diagnostics."""
    if not isinstance(report, dict) or not report:
        return {}
    return {
        "report": report.get("report"),
        "algorithm": report.get("algorithm"),
        "phase": report.get("phase"),
        "running": _recovery_bool(report.get("running")),
        "complete": _recovery_bool(report.get("complete")),
        "vote_span_cycles": _as_int(report.get("vote_span_cycles")),
        "quorum_required": _as_int(report.get("quorum_required")),
        "lanes": [
            _smartzero_lane_progress(report, "vclock", "VCLOCK"),
            _smartzero_lane_progress(report, "ocxo1", "OCXO1"),
            _smartzero_lane_progress(report, "ocxo2", "OCXO2"),
        ],
    }


def _smartzero_progress_text(progress: Dict[str, Any]) -> str:
    """Render one single-line SmartZero lane summary for startup logs."""
    lanes = progress.get("lanes") if isinstance(progress, dict) else None
    if not isinstance(lanes, list) or not lanes:
        return "SmartZero lane detail unavailable"

    rendered: List[str] = []
    for lane in lanes:
        if not isinstance(lane, dict):
            continue
        state = str(lane.get("state") or "UNKNOWN")
        samples = int(lane.get("samples") or 0)
        intervals = int(lane.get("intervals") or 0)
        quorum = int(lane.get("quorum") or 0)
        closest = int(lane.get("closest_three_span") or 0)
        span = int(lane.get("quorum_span") or 0)
        if state == "LOCKED":
            rendered.append(
                f"{lane.get('lane')}={state} samples={samples} intervals={intervals} "
                f"quorum={quorum} span={span}"
            )
        else:
            closest_text = str(closest) if intervals >= 3 else "-"
            rendered.append(
                f"{lane.get('lane')}={state} samples={samples} intervals={intervals} "
                f"closest3={closest_text}"
            )
    return "; ".join(rendered) if rendered else "SmartZero lane detail unavailable"


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
        "recover_dead_producer_restore_active": _recovery_bool(
            status.get("recover_dead_producer_restore_active")
        ),
        "recover_dead_producer_restore_epoch_ready": _recovery_bool(
            status.get("recover_dead_producer_restore_epoch_ready")
        ),
        "recover_dead_producer_restore_begin_count": _as_int(
            status.get("recover_dead_producer_restore_begin_count")
        ),
        "recover_dead_producer_restore_wait_count": _as_int(
            status.get("recover_dead_producer_restore_wait_count")
        ),
        "recover_dead_producer_restore_ready_count": _as_int(
            status.get("recover_dead_producer_restore_ready_count")
        ),
        "recover_dead_producer_restore_commit_count": _as_int(
            status.get("recover_dead_producer_restore_commit_count")
        ),
        "recover_smartzero_running": _recovery_bool(
            status.get("recover_smartzero_running")
        ),
        "recover_smartzero_complete": _recovery_bool(
            status.get("recover_smartzero_complete")
        ),
        "recover_epoch_ready": _recovery_bool(status.get("recover_epoch_ready")),
        "recover_proof_active": _recovery_bool(status.get("recover_proof_active")),
        "recover_proof_degraded_active": _recovery_bool(status.get("recover_proof_degraded_active")),
        "recover_proof_reason": status.get("recover_proof_reason"),
        "recover_timeline_ready": _recovery_bool(status.get("recover_timeline_ready")),
        "recover_clockface_ready": _recovery_bool(status.get("recover_clockface_ready")),
        "recover_science_ready": _recovery_bool(status.get("recover_science_ready")),
        "recover_proof_stalled": bool(
            _fragment_recovery_bool(
                status,
                "recover_proof_stalled",
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
    proof_active = _recovery_bool(status.get("recover_proof_active"))
    degraded_active = _recovery_bool(status.get("recover_proof_degraded_active"))
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
        and not proof_active
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
        compact.get("recover_dead_producer_restore_epoch_ready"),
        compact.get("recover_dead_producer_restore_commit_count"),
        compact.get("recover_proof_active"),
        compact.get("recover_proof_degraded_active"),
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
    lane_science_clean, lanes = _fragment_ocxo_science_clean(fragment)
    firmware_exclusion = _firmware_science_exclusion(fragment)
    firmware_science_excluded = bool(firmware_exclusion)
    # Per-lane OCXO science can remain internally coherent on a row that the
    # firmware has explicitly excluded at the whole-row court (for example a
    # cross-rail cycle excursion).  Recovery may retain such a row as audit
    # evidence, but it may never call that row clean science.
    science_clean = bool(lane_science_clean and not firmware_science_excluded)
    report_available = bool(status)

    fragment_active = _fragment_recovery_bool(
        fragment,
        "recover_proof_active",
    )
    active = (
        fragment_active
        if fragment_active is not None
        else _recovery_bool(status.get("recover_proof_active"))
    )

    fragment_degraded = _fragment_recovery_any_true(
        fragment,
        "recover_degraded_active",
    )
    degraded = (
        fragment_degraded
        if fragment_degraded is not None
        else _recovery_bool(status.get("recover_proof_degraded_active"))
    )

    stalled = bool(
        _fragment_recovery_bool(fragment, "recover_proof_stalled")
        or _recovery_bool(
            status.get("recover_proof_stalled")
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
            or status.get("proof_ready")
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
        blocking_reasons.append("recovery_proof_private_hold_active")
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
    if firmware_science_excluded:
        state_reasons.append("firmware_science_excluded")
    if not science_clean:
        state_reasons.append("ocxo_science_not_clean")
    if stalled:
        state_reasons.append("ocxo_science_proof_stalled")

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
        "recover_proof_active": bool(active),
        "recover_proof_degraded_active": bool(degraded),
        "recover_transition_active": transition_active,
        "recover_timeline_ready": timeline_ready,
        "recover_clockface_ready": bool(clockface_ready),
        "recover_science_ready": bool(science_ready),
        "recover_proof_stalled": stalled,
        "explicit_degraded": explicit_degraded,
        "watchdog_blocked": watchdog_blocked or watchdog_active,
        "science_quarantine_active": quarantine_active,
        "science_quarantine_remaining": int(quarantine_remaining),
        "lane_science_clean": bool(lane_science_clean),
        "firmware_science_excluded": firmware_science_excluded,
        "firmware_science_exclusion": copy.deepcopy(firmware_exclusion),
        "science_clean": science_clean,
        "lanes": lanes,
        "report_reason": (
            fragment.get("recover_proof_reason")
            or status.get("recover_proof_reason")
        ),
        "stall_reason": (
            status.get("recover_proof_stall_reason")
            or status.get("degraded_publication_stall_reason")
            or status.get("degraded_stall_reason")
        ),
        "degraded_window_row_count": _as_int(
            status.get("recover_proof_degraded_window_row_count")
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
            status.get("recover_proof_degraded_stall_threshold")
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


def _recovery_project_ocxo_from_base(
    *,
    public_gnss_ns: int,
    recovered_gnss_ns: int,
    recovered_ocxo_ns: int,
) -> int:
    """Mirror the one-time recovered TEMPEST OCXO presentation projection."""
    public_gnss_ns = int(public_gnss_ns)
    recovered_gnss_ns = int(recovered_gnss_ns)
    recovered_ocxo_ns = int(recovered_ocxo_ns)
    if public_gnss_ns <= 0 or recovered_gnss_ns <= 0 or recovered_ocxo_ns <= 0:
        raise ValueError("recovery OCXO projection requires positive clockfaces")

    recovered_offset_ns = recovered_ocxo_ns - recovered_gnss_ns
    scale = float(public_gnss_ns) / float(recovered_gnss_ns)
    scaled_offset_ns = float(recovered_offset_ns) * scale
    projected_offset_ns = (
        int(scaled_offset_ns + 0.5)
        if scaled_offset_ns >= 0.0
        else int(scaled_offset_ns - 0.5)
    )
    projected = public_gnss_ns + projected_offset_ns
    if projected <= 0:
        raise ValueError("recovery OCXO projection produced a non-positive clockface")
    return int(projected)




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
    """Extract recovery truth from one TIMEBASE_V4 observation view."""
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
        detail_id=_as_int(last_tb.get("_db_detail_id")),
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



def _project_clocks_recovery_snapshot(
    snapshot: Dict[str, Any],
    now: datetime,
) -> Dict[str, Any]:
    """Purely translate one selected CLOCKS recovery snapshot to desired state.

    ``snapshot`` is observation-only input from ``_recovery_timebase_snapshot``.
    This function performs no database access, firmware I/O, topology inspection,
    logging, or process-state mutation.  Its only non-identity work is the
    unavoidable deterministic translation of a running TEMPEST timeline across
    elapsed GNSS time.
    """
    if not isinstance(snapshot, dict):
        raise TypeError("CLOCKS recovery projection requires a snapshot object")
    if not snapshot.get("recoverable"):
        raise ValueError("CLOCKS recovery projection requires a recoverable snapshot")
    if not isinstance(now, datetime) or now.tzinfo is None:
        raise ValueError("CLOCKS recovery projection requires timezone-aware now")

    now_utc = now.astimezone(timezone.utc)
    last_pps_vclock_count = int(snapshot["last_pps_vclock_count"])
    last_gnss_ns = int(snapshot["last_gnss_ns"])
    last_dwt_cycles = int(snapshot["last_dwt_cycles"])
    last_dwt_ns = int(snapshot["last_dwt_ns"])
    last_ocxo1_ns = int(snapshot["last_ocxo1_ns"])
    last_ocxo2_ns = int(snapshot["last_ocxo2_ns"])
    last_gnss_time_str = str(snapshot["last_gnss_time_str"])

    # Preserve the legacy recovery rule exactly.  In a formally recoverable V4
    # snapshot GNSS is positive, but retain the historical DWT fallback here so
    # observation parsing and projection remain behaviorally equivalent.
    projection_gnss_ns = last_gnss_ns if last_gnss_ns > 0 else last_dwt_ns

    try:
        last_gnss_utc = datetime.fromisoformat(
            last_gnss_time_str.replace("Z", "+00:00")
        )
    except ValueError as exc:
        raise ValueError(
            f"CLOCKS recovery snapshot has invalid GNSS UTC {last_gnss_time_str!r}"
        ) from exc
    if last_gnss_utc.tzinfo is None:
        raise ValueError("CLOCKS recovery snapshot GNSS UTC is timezone-naive")
    last_gnss_utc = last_gnss_utc.astimezone(timezone.utc)

    elapsed_seconds = int(round((now_utc - last_gnss_utc).total_seconds()))
    if elapsed_seconds <= 0:
        raise ValueError(
            "CLOCKS recovery projection requires positive elapsed GNSS time: "
            f"last={last_gnss_time_str} "
            f"current={now_utc.isoformat().replace('+00:00', 'Z')} "
            f"elapsed_seconds={elapsed_seconds}"
        )

    recover_base_pps_vclock_count = last_pps_vclock_count + elapsed_seconds
    expected_first_public_pps_vclock_count = (
        recover_base_pps_vclock_count + RECOVERY_FIRST_PUBLIC_OFFSET
    )

    projected_gnss_ns = recover_base_pps_vclock_count * NS_PER_SECOND
    expected_first_public_gnss_ns = (
        expected_first_public_pps_vclock_count * NS_PER_SECOND
    )
    projected_dwt_cycles = (
        projected_gnss_ns * last_dwt_cycles // projection_gnss_ns
        if projection_gnss_ns > 0 and last_dwt_cycles > 0
        else 0
    )
    projected_dwt_ns = (
        _dwt_cycles_to_recover_ns(projected_dwt_cycles)
        if projected_dwt_cycles > 0
        else (
            projected_gnss_ns * last_dwt_ns // projection_gnss_ns
            if projection_gnss_ns > 0
            else projected_gnss_ns
        )
    )
    projected_ocxo1_ns = (
        projected_gnss_ns * last_ocxo1_ns // projection_gnss_ns
        if projection_gnss_ns > 0 and last_ocxo1_ns > 0
        else 0
    )
    projected_ocxo2_ns = (
        projected_gnss_ns * last_ocxo2_ns // projection_gnss_ns
        if projection_gnss_ns > 0 and last_ocxo2_ns > 0
        else 0
    )
    expected_first_public_ocxo1_ns = _recovery_project_ocxo_from_base(
        public_gnss_ns=expected_first_public_gnss_ns,
        recovered_gnss_ns=projected_gnss_ns,
        recovered_ocxo_ns=projected_ocxo1_ns,
    )
    expected_first_public_ocxo2_ns = _recovery_project_ocxo_from_base(
        public_gnss_ns=expected_first_public_gnss_ns,
        recovered_gnss_ns=projected_gnss_ns,
        recovered_ocxo_ns=projected_ocxo2_ns,
    )

    tau_dwt = (
        last_dwt_cycles
        / ((projection_gnss_ns * DWT_EXPECTED_PER_PPS) / NS_PER_SECOND)
        if projection_gnss_ns > 0 and last_dwt_cycles > 0
        else 1.0
    )
    tau_ocxo1 = (
        last_ocxo1_ns / projection_gnss_ns
        if projection_gnss_ns > 0 and last_ocxo1_ns > 0
        else 1.0
    )
    tau_ocxo2 = (
        last_ocxo2_ns / projection_gnss_ns
        if projection_gnss_ns > 0 and last_ocxo2_ns > 0
        else 1.0
    )

    return {
        "schema": CLOCKS_RECOVERY_DESIRED_STATE_SCHEMA,
        "source": copy.deepcopy(snapshot),
        "projected_at_gnss_utc": now_utc.isoformat().replace("+00:00", "Z"),
        "elapsed_seconds": int(elapsed_seconds),
        "recover_base_pps_vclock_count": int(recover_base_pps_vclock_count),
        "expected_first_public_pps_vclock_count": int(
            expected_first_public_pps_vclock_count
        ),
        "restore_clockfaces": {
            "gnss_ns": int(projected_gnss_ns),
            "dwt_cycles": int(projected_dwt_cycles),
            "dwt_ns": int(projected_dwt_ns),
            "ocxo1_ns": int(projected_ocxo1_ns),
            "ocxo2_ns": int(projected_ocxo2_ns),
        },
        "expected_first_public_clockfaces": {
            "gnss_ns": int(expected_first_public_gnss_ns),
            "ocxo1_ns": int(expected_first_public_ocxo1_ns),
            "ocxo2_ns": int(expected_first_public_ocxo2_ns),
        },
        "tau": {
            "dwt": float(tau_dwt),
            "ocxo1": float(tau_ocxo1),
            "ocxo2": float(tau_ocxo2),
        },
        "teensy_recover_args": {
            "dwt_ns": str(int(projected_dwt_ns)),
            "gnss_ns": str(int(projected_gnss_ns)),
            "ocxo1_ns": str(int(projected_ocxo1_ns)),
            "ocxo2_ns": str(int(projected_ocxo2_ns)),
        },
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


def _recovery_timebase_from_clocks_state(
    state: Dict[str, Any],
    campaign_name: str,
    *,
    source_detail_id: int,
    source_campaign: Optional[str] = None,
) -> Dict[str, Any]:
    """Form one recovery observation from an already-bound durable CLOCKS state.

    The caller supplies the exact durable row identity.  This adapter performs no
    database lookup and therefore cannot drift to a newer no-campaign restore
    authority while startup persistence is reopening.
    """
    if not isinstance(state, dict):
        raise TypeError("CLOCKS recovery state must be an object")
    detail_id = int(source_detail_id)
    if detail_id <= 0:
        raise ValueError("CLOCKS recovery state requires positive durable detail identity")

    state = copy.deepcopy(state)
    campaign = copy.deepcopy(_state_campaign(state))
    if not isinstance(campaign, dict) or not campaign:
        raise ValueError("CLOCKS recovery state has no TEMPEST campaign state")
    if campaign.get("schema") != "TEMPEST_FRAGMENT_V1":
        raise ValueError(
            f"CLOCKS recovery state has unsupported TEMPEST schema {campaign.get('schema')!r}"
        )

    observed_campaign = _tempest_campaign_name(campaign)
    if observed_campaign != str(campaign_name):
        raise ValueError(
            "CLOCKS recovery state campaign disagrees with active master: "
            f"snapshot={observed_campaign!r} active={campaign_name!r}"
        )
    if source_campaign not in (None, observed_campaign):
        raise ValueError(
            "CLOCKS recovery row label disagrees with raw TEMPEST state: "
            f"row={source_campaign!r} campaign={observed_campaign!r}"
        )
    campaign_state = str(campaign.get("state") or "").strip().upper()
    public_count = _tempest_public_count(campaign)
    if campaign_state != "STARTED" or public_count <= 0:
        raise ValueError(
            "CLOCKS recovery state does not contain a running TEMPEST boundary: "
            f"campaign={observed_campaign!r} state={campaign_state!r} "
            f"public_count={public_count}"
        )

    source_sequence = _as_int(state.get("sequence"))
    if source_sequence is None or source_sequence <= 0:
        raise ValueError("CLOCKS recovery state lacks positive physical sequence identity")

    clocks = _clocks_payload(state)
    gnss_raw = _clocks_gnss_raw_payload(state)
    instrument = gnss_raw.get("instrument") if isinstance(gnss_raw, dict) else None
    welford = gnss_raw.get("welford") if isinstance(gnss_raw, dict) else None
    if not isinstance(instrument, dict) or not isinstance(welford, dict):
        raise ValueError("CLOCKS recovery state lacks complete Pi GNSS_RAW state")

    fragment = copy.deepcopy(campaign)
    fragment.pop("adjudication", None)

    extra_clocks = {
        "gnss_raw_instrument_ns": _as_int(instrument.get("ns")) or 0,
        "gnss_raw_instrument_ref_ns": _as_int(instrument.get("ref_ns")) or 0,
        "gnss_raw_instrument_n": _as_int(instrument.get("clockface_n")) or 0,
        "gnss_raw_instrument_valid": bool(instrument.get("valid")),
        "gnss_raw_drift_ppb": gnss_raw.get("drift_ppb"),
        "gnss_raw_welford_n": _as_int(welford.get("n")) or 0,
        "gnss_raw_welford_mean": _first_float(welford.get("mean"), 0.0) or 0.0,
        "gnss_raw_welford_m2": _first_float(welford.get("m2"), 0.0) or 0.0,
        "gnss_raw_welford_stddev": _first_float(welford.get("stddev"), 0.0) or 0.0,
        "gnss_raw_welford_stderr": _first_float(welford.get("stderr"), 0.0) or 0.0,
        "gnss_raw_welford_min": _first_float(welford.get("min"), 0.0) or 0.0,
        "gnss_raw_welford_max": _first_float(welford.get("max"), 0.0) or 0.0,
    }

    return {
        "schema": "TIMEBASE_V4",
        "campaign_type": CAMPAIGN_TYPE_TEMPEST,
        "campaign": observed_campaign,
        "sequence": int(source_sequence),
        "public_count": int(public_count),
        "campaign_elapsed": _seconds_to_hms(int(public_count)),
        "persist": True,
        "location": state.get("location"),
        "system_time_utc": state.get("published_at_utc"),
        "gnss_time_utc": (
            clocks.get("gnss_time_utc")
            or _path_get(state, "gnss.gnss_time_utc")
            or _path_get(state, "gnss.next_utc")
        ),
        "fragment": fragment,
        "clocks": copy.deepcopy(clocks),
        "environment": copy.deepcopy(state.get("environment")),
        "gnss": copy.deepcopy(state.get("gnss")),
        "extra_clocks": extra_clocks,
        "_db_detail_id": detail_id,
        "_recovery_authority": "CLOCKS_RECOVERY_SNAPSHOT",
        "_campaign_adjudication_required": False,
    }


def _recovery_timebase_from_clocks_snapshot(
    snapshot: _ClocksRecoverySnapshot,
    campaign_name: str,
) -> Dict[str, Any]:
    """Form dead-producer TEMPEST observation from one CLOCKS recovery snapshot."""
    if not isinstance(snapshot, _ClocksRecoverySnapshot):
        raise TypeError("CLOCKS dead recovery requires _ClocksRecoverySnapshot")
    state_sequence = _as_int(snapshot.canonical.get("sequence"))
    if state_sequence != int(snapshot.source_sequence):
        raise ValueError(
            "CLOCKS recovery snapshot sequence identity changed inside canonical state: "
            f"snapshot={snapshot.source_sequence} canonical={state_sequence}"
        )
    return _recovery_timebase_from_clocks_state(
        snapshot.canonical,
        campaign_name,
        source_detail_id=int(snapshot.source_detail_id),
        source_campaign=snapshot.source_campaign,
    )


def _has_tempest_state_details(campaign_name: str) -> bool:
    """Return whether one adjudicated TEMPEST state exists without counting history."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT 1 AS present
            FROM campaign_detail
            WHERE campaign_type = %s
              AND campaign = %s
              AND payload #> '{campaign,adjudication}' IS NOT NULL
            ORDER BY id DESC
            LIMIT 1
            """,
            (
                CAMPAIGN_TYPE_TEMPEST,
                campaign_name,
            ),
        )
        row = cur.fetchone()
    return row is not None


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
              AND NOT (payload @> '{"holistic_restore_superseded":true}'::jsonb)
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
        except HardFailureRequired:
            raise
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
    if _hard_failure_active():
        return {
            "success": False,
            "message": f"{operation} unavailable while CLOCKS is latched in HARD_FAILURE",
            "payload": {"operational_state": _operational_state_snapshot()},
        }
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


def _defer_startup_watchdog(anomaly: Dict[str, Any]) -> None:
    """Hold continuity-surrender testimony until startup owns recovery."""
    envelope = {
        "received_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "received_monotonic": time.monotonic(),
        "payload": copy.deepcopy(anomaly),
    }
    with _startup_watchdog_lock:
        _startup_watchdog_deferred.append(envelope)
        depth = len(_startup_watchdog_deferred)
    _diag["watchdog_anomaly_startup_deferred"] += 1
    _diag["last_startup_deferred_watchdog"] = {
        **copy.deepcopy(envelope),
        "depth": int(depth),
        "state": "DEFERRED_TO_STARTUP_RECONCILIATION",
    }
    logging.warning(
        "🧭 [clocks/startup] WATCHDOG_ANOMALY retained during startup; "
        "holistic reconciliation owns recovery (reason=%s campaign=%s depth=%d)",
        anomaly.get("reason"),
        anomaly.get("campaign"),
        depth,
    )


def _reconcile_deferred_startup_watchdogs() -> Dict[str, Any]:
    """Retire startup watchdog triggers after holistic state reconciliation."""
    with _startup_watchdog_lock:
        deferred = list(_startup_watchdog_deferred)
        _startup_watchdog_deferred.clear()

    if not deferred:
        return {"count": 0, "reconciled": False}

    _diag["watchdog_anomaly_startup_reconciled"] += len(deferred)
    latest = copy.deepcopy(deferred[-1])
    result = {
        "count": len(deferred),
        "reconciled": True,
        "latest": latest,
        "authority": "STARTUP_HOLISTIC_RECONCILIATION",
    }
    _diag["last_startup_deferred_watchdog"] = {
        **latest,
        "depth": 0,
        "state": "RECONCILED_BY_STARTUP",
        "reconciled_count": len(deferred),
    }
    logging.info(
        "✅ [clocks/startup] reconciled %d deferred WATCHDOG_ANOMALY "
        "trigger(s) inside the holistic startup transaction; no competing "
        "auto-recovery thread was launched",
        len(deferred),
    )
    return result


def _log_watchdog_anomaly_forensics(anomaly: Dict[str, Any]) -> None:
    """Emit one deliberately exhaustive log record for a continuity surrender."""
    now = time.monotonic()
    last_clocks_age_ms = (
        None
        if _latest_clocks_received_monotonic is None
        else round(max(0.0, now - _latest_clocks_received_monotonic) * 1000.0, 3)
    )
    testimony = {
        "received_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "pi_context": {
            "pid": os.getpid(),
            "thread": threading.current_thread().name,
            "operational_state": _operational_state_snapshot(),
            "startup_control_ready": _startup_control_ready.is_set(),
            "campaign_active": bool(_campaign_active),
            "auto_recovery_in_progress": bool(_auto_recovery_in_progress),
            "last_clocks_sequence": _last_clocks_state_sequence,
            "last_clocks_age_ms": last_clocks_age_ms,
            "last_pps_vclock_count_seen": _last_pps_vclock_count_seen,
            "accepted_pps_vclock_count": _accepted_pps_vclock_count,
            "queues": {
                "clocks_state": _clocks_state_queue.qsize(),
                "clocks_persistence": _clocks_persist_queue.qsize(),
                "tempest": _fragment_queue.qsize(),
            },
        },
        "teensy_payload": copy.deepcopy(anomaly),
    }
    encoded = json.dumps(testimony, indent=2, sort_keys=True, default=str)
    logging.error(
        "💥 [clocks] WATCHDOG_ANOMALY FORENSIC TESTIMONY — full event follows "
        "(json_bytes=%d):\n%s",
        len(encoded.encode("utf-8")),
        encoded,
    )


def on_watchdog_anomaly(payload: Payload) -> None:
    """
    PUBSUB handler for WATCHDOG_ANOMALY from Teensy CLOCKS.

    This is an explicit semantic surrender by the Teensy: campaign continuity
    is no longer being asserted. We enqueue a durable event immediately. Once
    startup reconciliation is complete it initiates ordinary auto-recovery;
    before that boundary the anomaly is retained as startup-owned evidence.
    """
    _diag["watchdog_anomalies_received"] += 1

    anomaly = dict(payload)
    _log_watchdog_anomaly_forensics(anomaly)
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

    if not _startup_control_ready.is_set():
        _defer_startup_watchdog(anomaly)
        return

    started = _begin_auto_recovery(
        "watchdog_anomaly",
        {"payload": anomaly},
        source="WATCHDOG_ANOMALY",
    )
    if started:
        _diag["watchdog_anomaly_recovery_started"] += 1


def on_recovery_stalled(payload: Payload) -> None:
    """Record a non-destructive OCXO recovery-proof/liveness anomaly.

    This event means the Teensy timeline is alive but OCXO science proof either
    exceeded its bounded proof-attempt expectation or stopped advancing.  It is
    deliberately not WATCHDOG_ANOMALY: restarting RECOVER here would destroy the
    very proof convergence state being diagnosed.
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
        "proof_attempts": stalled.get("proof_attempts"),
        "proof_warn_after_attempts": stalled.get("proof_warn_after_attempts"),
        "last_attempt_pps_sequence": stalled.get("last_attempt_pps_sequence"),
        "no_progress_rows": stalled.get("no_progress_rows"),
        "stall_threshold_rows": stalled.get("stall_threshold_rows"),
        "last_progress_public_count": stalled.get("last_progress_public_count"),
        "clockface_ready": stalled.get("clockface_ready"),
        "science_ready": stalled.get("science_ready"),
        "lanes": copy.deepcopy(stalled.get("lanes") or {}),
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

    if snapshot.get("reason") == "ocxo_science_proof_not_established":
        _diag["last_recovery_proof_warning"] = snapshot
        logging.error(
            "🧭 [recovery] proof-driven OCXO science release not established "
            "after %s attempt(s) (warn_after=%s, campaign=%s generation=%s); "
            "timeline publication continues and RECOVER is not restarted",
            snapshot.get("proof_attempts"),
            snapshot.get("proof_warn_after_attempts"),
            snapshot.get("campaign"),
            snapshot.get("recovery_generation"),
        )
    else:
        logging.error(
            "🧭 [recovery] OCXO science proof convergence reports no progress "
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
    """Advance Pi-owned GNSS_RAW exactly once per boot-local CLOCKS sequence."""
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
            previous_sequence = int(_last_clocks_state_sequence)
            regression = {
                "observed_sequence": int(sequence),
                "previous_sequence": previous_sequence,
                "observed_at_utc": datetime.now(timezone.utc)
                    .isoformat()
                    .replace("+00:00", "Z"),
            }
            recovery_context = bool(
                _startup_control_ready.is_set()
                and (
                    _campaign_active
                    or _auto_recovery_in_progress
                    or _timebase_silence_recovery_active
                )
            )
            if recovery_context:
                _begin_recovery_clocks_custody(
                    "physical_sequence_regression",
                    regression,
                    physical_sequence_regression=True,
                )

            # CLOCKS_FRAGMENT.sequence is a Teensy-boot-local delivery identity.
            # A regression therefore rebases only GNSS_RAW's exactly-once trigger;
            # it does not surrender the Pi-owned always-on GNSS_RAW clock/statistics.
            # This row is a real post-reboot observation, so consume it exactly once
            # below and let the next physical sequence advance normally.
            _last_clocks_state_sequence = int(sequence)
            _last_clocks_state_monotonic = now
            _diag["gnss_raw_physical_sequence_rebases"] = (
                _diag.get("gnss_raw_physical_sequence_rebases", 0) + 1
            )
            _diag["last_gnss_raw_physical_sequence_rebase"] = {
                **regression,
                "recovery_context": bool(recovery_context),
                "gnss_raw_population_preserved": True,
                "first_new_lifetime_row_consumed": True,
            }
            logging.warning(
                "🧭 [clocks] physical CLOCKS sequence rebased %d -> %d; "
                "preserving Pi GNSS_RAW population and consuming the first "
                "new-lifetime observation exactly once",
                previous_sequence,
                int(sequence),
            )

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
    return {
        "schema": "PI_CLOCKS_STATE_V4",
        "gnss_raw": gnss_raw,
        "stats_reset": {
            "requests": int(_diag.get("stats_reset_requests") or 0),
            "success": int(_diag.get("stats_reset_success") or 0),
            "in_progress": _stats_reset_in_progress.is_set(),
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

    # The producer's compact checkpoint testimony is part of the V4 scientific
    # contract now. Verify every published rolling bucket from the same row before
    # allowing that testimony to mutate Pi checkpoint custody.
    _validate_firmware_ppb_checkpoint_delta(stats)
    return int(sequence), teensy_clocks


def _build_canonical_clocks_state(
    clocks_fragment: Dict[str, Any],
    system_context: Dict[str, Any],
    *,
    mutate_alpha_custody: bool = True,
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    """Build canonical CLOCKS_V4 from one exact CLOCKS_FRAGMENT_V4 observation.

    During unresolved startup a proved physical-sequence regression means the
    fragment belongs to a newborn Teensy lifetime whose relationship to durable
    Alpha has not yet been classified.  Such rows remain live evidence, but they
    must not reset/advance Pi custody that is statistically descended from Alpha.
    """
    published_at = datetime.now(timezone.utc)
    published_at_utc = published_at.isoformat().replace("+00:00", "Z")
    sequence, teensy_clocks = _validate_clocks_fragment_v4(clocks_fragment)

    gnss = _system_gnss_info(system_context)

    # Consume Alpha-derived Pi custody only after producer lifetime is classified.
    # A pre-classification newborn row is still a lawful physical observation, but
    # allowing it to reset DAC chronology or replace the literal Better-Buckets
    # image would let an unclassified lifetime rewrite resurrection authority.
    if mutate_alpha_custody:
        _dac_process_completed_row(
            teensy_clocks,
            clocks_fragment.get("campaign"),
            sequence,
        )
    pi_clocks = _pi_clocks_state_snapshot()

    # One canonical clocks object.  Teensy owns the real clocks; Pi adds only
    # the independently owned GNSS_RAW clock.  No clocks.pi mirror and no
    # duplicated CLOCKS_FRAGMENT evidence branch.
    clocks = copy.deepcopy(teensy_clocks)
    clocks["source_schema"] = teensy_clocks.get("schema")
    clocks["schema"] = "CLOCKS_INSTRUMENT_STATE_V1"

    # DAC/control is Pi-authored.  Keep the established canonical JSON shape so
    # front-end consumers do not care that CLOCKS_FRAGMENT stopped carrying it.
    clocks["control"] = _dac_control_snapshot()
    ppb_restore_checkpoint: Optional[Dict[str, Any]] = None
    stats = clocks.get("stats")
    if isinstance(stats, dict):
        auxiliary = stats.get("auxiliary_welford")
        if not isinstance(auxiliary, dict):
            auxiliary = {}
            stats["auxiliary_welford"] = auxiliary
        auxiliary.update(_dac_welford_snapshots())

        # Pi owns persistence/recovery orchestration. Keep the literal bounded
        # resurrection image private: it is committed to config.CLOCKS_RECOVERY
        # beside the durable observation, never transported inside CLOCKS itself.
        if mutate_alpha_custody:
            ppb_restore_checkpoint = _ppb_checkpoint_ingest(stats)
        else:
            with _ppb_checkpoint_lock:
                runtime = _ppb_checkpoint_runtime_ensure_locked()
                ppb_restore_checkpoint = _ppb_checkpoint_snapshot_locked(runtime)

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
        "location": copy.deepcopy(system_context.get("location") or {}),
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
        "stats_reset": copy.deepcopy(pi_clocks.get("stats_reset") or {}),
        "startup": copy.deepcopy(pi_clocks.get("startup") or {}),
        "complete_for_display": bool(teensy_clocks and pi_clocks.get("gnss_raw")),
    }

    campaign = clocks_fragment.get("campaign")
    if isinstance(campaign, dict):
        # The raw firmware delta is persisted exactly once at top level.
        state["campaign"] = copy.deepcopy(campaign)

    if ppb_restore_checkpoint is None:
        raise RuntimeError("canonical CLOCKS row did not produce recovery custody")
    return state, ppb_restore_checkpoint


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


def _set_clocks_persistence_worker_phase(
    phase: str,
    *,
    sequence: Optional[int] = None,
    detail: Optional[Dict[str, Any]] = None,
) -> None:
    """Publish lock-free diagnostic testimony about the single persistence worker."""
    snapshot: Dict[str, Any] = {
        "phase": str(phase),
        "sequence": None if sequence is None else int(sequence),
        "at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "queue_depth": _clocks_persist_queue.qsize(),
    }
    if isinstance(detail, dict) and detail:
        snapshot.update(copy.deepcopy(detail))
    _diag["clocks_persistence_worker"] = snapshot


def _persist_clocks_state(
    state: Dict[str, Any], checkpoint: Dict[str, Any]
) -> str:
    encoded = json.dumps(state, separators=(",", ":"), ensure_ascii=False)
    sequence = state.get("sequence")
    pps_count = sequence
    campaign = _clocks_detail_campaign(state)
    viable = _clocks_detail_viable(state)
    persist_recovery = _clocks_state_owns_recovery_config(state, checkpoint)
    _set_clocks_persistence_worker_phase("DB_OPEN", sequence=_as_int(sequence))
    with open_db() as conn:
        cur = conn.cursor()
        _set_clocks_persistence_worker_phase("DB_MERGE_UPDATE", sequence=_as_int(sequence))
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
        _set_clocks_persistence_worker_phase("DB_MERGE_FETCH", sequence=_as_int(sequence))
        merged_row = cur.fetchone()
        if merged_row is not None:
            detail_id = int(
                merged_row["id"] if isinstance(merged_row, dict) else merged_row[0]
            )
            if persist_recovery:
                _set_clocks_persistence_worker_phase(
                    "DB_RECOVERY_CONFIG_MERGE", sequence=_as_int(sequence),
                    detail={"detail_id": detail_id},
                )
                _write_clocks_recovery_config(
                    cur, checkpoint, source_detail_id=detail_id
                )
            _set_clocks_persistence_worker_phase(
                "DB_COMMIT_MERGED", sequence=_as_int(sequence),
                detail={"detail_id": detail_id},
            )
            return "merged"
        _set_clocks_persistence_worker_phase("DB_INSERT", sequence=_as_int(sequence))
        cur.execute(
            """
            INSERT INTO campaign_detail
                (campaign_type, campaign, viable, payload, sequence, pps_count)
            VALUES (%s, %s, %s, %s::jsonb, %s, %s)
            RETURNING id
            """,
            (
                CAMPAIGN_TYPE_TEMPEST, campaign, viable, encoded, sequence, pps_count,
            ),
        )
        _set_clocks_persistence_worker_phase("DB_INSERT_FETCH", sequence=_as_int(sequence))
        inserted_row = cur.fetchone()
        if inserted_row is None:
            raise RuntimeError("CLOCKS insert returned no durable detail identity")
        detail_id = int(
            inserted_row["id"] if isinstance(inserted_row, dict) else inserted_row[0]
        )
        if persist_recovery:
            _set_clocks_persistence_worker_phase(
                "DB_RECOVERY_CONFIG_INSERT", sequence=_as_int(sequence),
                detail={"detail_id": detail_id},
            )
            _write_clocks_recovery_config(
                cur, checkpoint, source_detail_id=detail_id
            )
        _set_clocks_persistence_worker_phase(
            "DB_COMMIT_INSERT", sequence=_as_int(sequence),
            detail={"detail_id": detail_id},
        )
    return "inserted"


def _cache_clocks_state(
    state: Dict[str, Any],
    ppb_restore_checkpoint: Optional[Dict[str, Any]] = None,
    *,
    received_monotonic: Optional[float] = None,
    received_utc: Optional[str] = None,
) -> None:
    global _latest_clocks, _latest_clocks_ppb_restore_checkpoint
    global _latest_clocks_received_monotonic, _latest_clocks_received_utc
    with _clocks_lock:
        _latest_clocks = copy.deepcopy(state)
        _latest_clocks_ppb_restore_checkpoint = (
            copy.deepcopy(ppb_restore_checkpoint)
            if isinstance(ppb_restore_checkpoint, dict)
            else None
        )
        _latest_clocks_received_monotonic = (
            float(received_monotonic)
            if received_monotonic is not None
            else time.monotonic()
        )
        _latest_clocks_received_utc = str(
            received_utc or state.get("published_at_utc") or system_time_z()
        )
    _diag["preflight_clocks_updates"] += 1


def _queue_clocks_state(
    fragment: Dict[str, Any],
    *,
    ingress_monotonic: float,
    ingress_utc: str,
) -> None:
    global _clocks_state_enqueued, _clocks_state_dropped
    item = {
        "fragment": copy.deepcopy(fragment),
        "ingress_monotonic": float(ingress_monotonic),
        "ingress_utc": str(ingress_utc),
        "ppb_restore_transaction_ingress": _ppb_restore_transaction_active.is_set(),
    }
    try:
        _clocks_state_queue.put_nowait(item)
        _clocks_state_enqueued += 1
    except queue.Full:
        _clocks_state_dropped += 1
        logging.error(
            "💥 [clocks] canonical CLOCKS state queue full; dropping sequence=%s",
            _clocks_fragment_count(fragment),
        )


def _reset_startup_clocks_custody() -> None:
    global _startup_custody_active
    with _startup_custody_lock:
        _startup_custody_backlog.clear()
        _startup_custody_active = True
        _diag["startup_custody_active"] = True
        _diag["startup_custody_depth"] = 0
        _diag["startup_custody_retained"] = 0
        _diag["startup_custody_released"] = 0
        _diag["startup_custody_quarantined"] = 0
        _diag["startup_custody_retired"] = 0
        _diag["startup_custody_last_sequence"] = None
        _diag["startup_physical_lifetime_boundaries"] = 0
        _diag["startup_preclassification_rows_non_authoritative"] = 0
        _diag["last_startup_physical_lifetime_boundary"] = {}
        _diag["last_startup_custody_release"] = {}
        _diag["last_startup_custody_quarantine"] = {}
        _diag["last_startup_custody_retire"] = {}
    _startup_physical_lifetime_unclassified.clear()


def _retain_startup_clocks_item(item: Dict[str, Any]) -> bool:
    """Retain one exact canonical row while startup persistence is closed."""
    with _startup_custody_lock:
        if not _startup_custody_active or _clocks_persistence_enabled.is_set():
            return False
        _startup_custody_backlog.append(item)
        _diag["startup_custody_retained"] += 1
        _diag["startup_custody_depth"] = len(_startup_custody_backlog)
        _diag["startup_custody_last_sequence"] = item["state"].get("sequence")
        return True


def _release_startup_clocks_custody_live() -> Dict[str, Any]:
    """Durably flush retained live-Teensy rows before campaign reconciliation."""
    global _startup_custody_active

    if _startup_physical_lifetime_unclassified.is_set():
        raise RuntimeError(
            "cannot release startup CLOCKS custody as live after a proved physical "
            "sequence regression"
        )

    completion: Optional[threading.Event] = None
    with _startup_custody_lock:
        if not _startup_custody_active:
            return {"released": 0, "already_resolved": True}

        count = len(_startup_custody_backlog)
        first_sequence = (
            _startup_custody_backlog[0]["state"].get("sequence")
            if count
            else None
        )
        last_sequence = (
            _startup_custody_backlog[-1]["state"].get("sequence")
            if count
            else None
        )

        for index, retained in enumerate(_startup_custody_backlog):
            item = dict(retained)
            # These rows preserve instrument chronology only.  Pi campaign
            # adjudication/context was offline, so do not replay historical
            # campaign candidates into the live TEMPEST processor.
            item["release_campaign_candidate"] = False
            if index == count - 1:
                completion = threading.Event()
                item["persistence_completion"] = completion
            _clocks_persist_queue.put(item)

        _startup_custody_backlog.clear()
        _startup_custody_active = False
        _diag["startup_custody_active"] = False
        _diag["startup_custody_depth"] = 0

        # All retained rows are already ahead of any future live row in the
        # persistence queue.  Open ordinary persistence only after that order is
        # established.
        _clocks_persistence_enabled.set()

    if completion is not None and not completion.wait(timeout=HOLISTIC_RESTORE_TIMEOUT_S):
        raise RuntimeError(
            "startup CLOCKS custody did not durably flush before campaign recovery "
            f"within {HOLISTIC_RESTORE_TIMEOUT_S:.1f}s "
            f"(count={count} first_sequence={first_sequence} last_sequence={last_sequence})"
        )

    result = {
        "released": count,
        "first_sequence": first_sequence,
        "last_sequence": last_sequence,
        "instrument_only": True,
    }
    _diag["startup_custody_released"] += count
    _diag["last_startup_custody_release"] = dict(result)
    logging.info(
        "✅ [holistic restore] startup CLOCKS custody committed before campaign recovery: "
        "rows=%d first_sequence=%s last_sequence=%s",
        count, first_sequence, last_sequence,
    )
    return result


def _quarantine_startup_clocks_custody_surviving_alpha(
    reason: str,
) -> Dict[str, Any]:
    """Durably preserve startup rows that predate Pi-state restoration.

    Alpha has been proved continuous, so the producer observations belong to the
    surviving physical/statistical lifetime and must not disappear.  Their Pi-owned
    GNSS_RAW/DAC/checkpoint decorations were formed before durable Pi custody was
    restored, however, so those decorations are historical evidence only.  Persist
    the exact retained canonical rows in FIFO order, mark them ineligible as future
    holistic-restore authority, and never replay their embedded TEMPEST candidates.
    """
    global _startup_custody_active

    if _startup_physical_lifetime_unclassified.is_set():
        raise RuntimeError(
            "cannot classify startup CLOCKS custody as surviving Alpha after a proved "
            "physical sequence regression"
        )

    completion: Optional[threading.Event] = None
    classified_at_utc = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    with _startup_custody_lock:
        if not _startup_custody_active:
            return {"quarantined": 0, "already_resolved": True}

        count = len(_startup_custody_backlog)
        first_sequence = (
            _startup_custody_backlog[0]["state"].get("sequence")
            if count
            else None
        )
        last_sequence = (
            _startup_custody_backlog[-1]["state"].get("sequence")
            if count
            else None
        )

        for index, retained in enumerate(_startup_custody_backlog):
            item = copy.deepcopy(retained)
            state = item.get("state")
            if not isinstance(state, dict):
                raise RuntimeError("startup CLOCKS custody item lacks canonical state")

            state["holistic_restore_superseded"] = True
            state["startup_custody"] = {
                "schema": "PI_CLOCKS_STARTUP_CUSTODY_V1",
                "classification": "SURVIVING_ALPHA_PRE_PI_RESTORE",
                "classified_at_utc": classified_at_utc,
                "reason": str(reason),
                "producer_observation_preserved": True,
                "pi_state_authority": False,
                "restore_authority": False,
                "campaign_candidate_replayed": False,
            }
            item["state"] = state

            # The physical producer observation is preserved, but campaign
            # adjudication depended on a live Pi context that did not yet own
            # restored custody.  Do not manufacture that historical decision now.
            item["release_campaign_candidate"] = False
            if index == count - 1:
                completion = threading.Event()
                item["persistence_completion"] = completion
            _clocks_persist_queue.put(item)

        _startup_custody_backlog.clear()
        _startup_custody_active = False
        _diag["startup_custody_active"] = False
        _diag["startup_custody_depth"] = 0

        # Establish FIFO order before opening ordinary persistence.  Every future
        # live row therefore follows this preserved pre-restore prefix durably.
        _clocks_persistence_enabled.set()

    if completion is not None and not completion.wait(timeout=HOLISTIC_RESTORE_TIMEOUT_S):
        raise RuntimeError(
            "quarantined startup CLOCKS custody did not become durable "
            f"within {HOLISTIC_RESTORE_TIMEOUT_S:.1f}s "
            f"(count={count} first_sequence={first_sequence} last_sequence={last_sequence})"
        )

    result = {
        "quarantined": count,
        "first_sequence": first_sequence,
        "last_sequence": last_sequence,
        "classification": "SURVIVING_ALPHA_PRE_PI_RESTORE",
        "reason": str(reason),
        "producer_observation_preserved": True,
        "pi_state_authority": False,
        "restore_authority": False,
        "campaign_candidate_replayed": False,
    }
    _diag["startup_custody_quarantined"] = (
        _diag.get("startup_custody_quarantined", 0) + count
    )
    _diag["last_startup_custody_quarantine"] = copy.deepcopy(result)
    logging.info(
        "🧾 [holistic restore] startup CLOCKS evidence preserved under surviving Alpha: "
        "rows=%d first_sequence=%s last_sequence=%s classification=%s",
        count,
        first_sequence,
        last_sequence,
        result["classification"],
    )
    return result


def _retire_startup_clocks_custody(reason: str) -> Dict[str, Any]:
    """Retire pre-restore rows that do not belong to the restored instrument epoch."""
    global _startup_custody_active

    with _startup_custody_lock:
        count = len(_startup_custody_backlog)
        first_sequence = (
            _startup_custody_backlog[0]["state"].get("sequence")
            if count
            else None
        )
        last_sequence = (
            _startup_custody_backlog[-1]["state"].get("sequence")
            if count
            else None
        )
        _startup_custody_backlog.clear()
        _startup_custody_active = False
        _diag["startup_custody_active"] = False
        _diag["startup_custody_depth"] = 0
        _startup_physical_lifetime_unclassified.clear()

    result = {
        "retired": count,
        "first_sequence": first_sequence,
        "last_sequence": last_sequence,
        "reason": reason,
    }
    _diag["startup_custody_retired"] += count
    _diag["last_startup_custody_retire"] = dict(result)
    logging.info(
        "🧹 [holistic restore] startup CLOCKS custody retired before epoch replacement: "
        "rows=%d first_sequence=%s last_sequence=%s reason=%s",
        count, first_sequence, last_sequence, reason,
    )
    return result


def _startup_clocks_custody_unresolved() -> bool:
    with _startup_custody_lock:
        return bool(_startup_custody_active)


def _ppb_restore_transition_row_expected(
    clocks_fragment: Dict[str, Any],
    *,
    ingress_during_restore: bool,
) -> bool:
    """True only for the known incoherent PPB snapshot window we ourselves opened.

    PPB_RESTORE_BEGIN/CHUNK/COMMIT mutates Alpha's bounded rings transactionally.
    During staging firmware publishes the correct checkpoint schema with valid=false
    because no coherent whole-ring testimony exists yet. Such a row is expected
    only when it was received while Pi owned that restore transaction (or while the
    transaction is still active at consumption time). Every other valid=false row
    must continue through the strict validator and fail loudly.
    """
    if not (ingress_during_restore or _ppb_restore_transaction_active.is_set()):
        return False
    stats = _path_get(clocks_fragment, "clocks.stats")
    if not isinstance(stats, dict):
        return False
    checkpoint = stats.get("rolling_ppb_checkpoint")
    return bool(
        isinstance(checkpoint, dict)
        and checkpoint.get("schema") == PPB_FIRMWARE_DELTA_SCHEMA
        and checkpoint.get("valid") is False
    )


def _holistic_restore_proof_custody_for_state(
    state: Dict[str, Any],
    ppb_restore_checkpoint: Dict[str, Any],
) -> bool:
    """Nominate/recognize rows belonging to the active holistic N+1 proof lane.

    Recovery custody is a restore-authority classification, not a competing
    persistence path.  A row may therefore be both recovery-quarantined and the
    exact durable Alpha N+1 successor.  Nominate that proof identity before any
    routing branch that can enqueue-and-continue.
    """
    global _clocks_holistic_restore_proof_sequence

    if not _clocks_holistic_restore_proof_pending.is_set():
        return False

    stats = _clocks_payload(state).get("stats")
    update_count = (
        _as_int(stats.get("update_count"))
        if isinstance(stats, dict)
        else None
    )
    reset_count = (
        _as_int(stats.get("reset_count"))
        if isinstance(stats, dict)
        else None
    )

    if _clocks_holistic_restore_proof_sequence is None:
        observed = _holistic_restore_probe(state, ppb_restore_checkpoint)
        if not (
            reset_count == _clocks_holistic_restore_proof_reset_count
            and update_count == _clocks_holistic_restore_proof_update_count
            and _holistic_restore_probe_satisfied(
                _clocks_holistic_restore_proof_expected, observed
            )
        ):
            return False

        _clocks_holistic_restore_proof_sequence = int(state.get("sequence") or 0)
        logging.info(
            "✅ [holistic restore] first restored statistics row entered "
            "persistence custody: reset_count=%s update_count=%s sequence=%s",
            reset_count,
            update_count,
            state.get("sequence"),
        )
        if _ambient_instrument_recovery_active.is_set():
            _ambient_instrument_recovery_hold.clear()
            logging.info(
                "✅ [ambient restore] exact N+1 proof row admitted; "
                "releasing newborn-row hold at sequence=%s",
                state.get("sequence"),
            )
        if _startup_instrument_restore_hold.is_set():
            _startup_instrument_restore_hold.clear()
            logging.info(
                "✅ [holistic restore] exact startup N+1 proof row admitted; "
                "releasing newborn-row mutation hold at sequence=%s",
                state.get("sequence"),
            )
        return True

    return bool(
        reset_count == _clocks_holistic_restore_proof_reset_count
        and update_count is not None
        and update_count >= _clocks_holistic_restore_proof_update_count
    )


def _clocks_state_loop() -> None:
    """Build and publish CLOCKS_V4 without storage latency stalling the live feed."""
    global _clocks_state_published, _clocks_state_dropped
    global _clocks_epoch_birth_reset_count

    _clocks_state_worker_started.set()
    logging.info("🚀 [clocks] canonical CLOCKS_V4 state worker started")
    while True:
        queue_item = _clocks_state_queue.get()
        clocks_fragment = queue_item.get("fragment") if isinstance(queue_item, dict) else None
        ingress_monotonic = (
            queue_item.get("ingress_monotonic")
            if isinstance(queue_item, dict)
            else None
        )
        ingress_utc = (
            queue_item.get("ingress_utc")
            if isinstance(queue_item, dict)
            else None
        )
        ingress_during_ppb_restore = bool(
            queue_item.get("ppb_restore_transaction_ingress")
            if isinstance(queue_item, dict)
            else False
        )
        if not isinstance(clocks_fragment, dict):
            _clocks_state_dropped += 1
            logging.error("💥 [clocks] malformed private CLOCKS state queue envelope; dropping")
            continue
        if _hard_failure_active() and not _hard_failure_stats_repair_active():
            _clocks_state_dropped += 1
            _diag["hard_failure_state_dropped"] = (
                _diag.get("hard_failure_state_dropped", 0) + 1
            )
            continue

        failure_logged = False
        while True:
            if _hard_failure_active() and not _hard_failure_stats_repair_active():
                _clocks_state_dropped += 1
                _diag["hard_failure_state_dropped"] = (
                    _diag.get("hard_failure_state_dropped", 0) + 1
                )
                break
            try:
                system_context = _fetch_system_report()
                break
            except Exception as exc:
                # A sibling Pi process may simply not have opened its command
                # socket yet during startup/restart. That is an ordinary wait
                # state, not an anomaly: retry quietly without imposing timing
                # semantics on process startup. Preserve one loud diagnostic for
                # non-transport failures such as a malformed SYSTEM.REPORT reply.
                transport_unavailable = isinstance(exc.__cause__, OSError)
                if not transport_unavailable and not failure_logged:
                    logging.exception(
                        "⚠️ [clocks] SYSTEM.REPORT failed semantically for CLOCKS "
                        "sequence=%s; retrying",
                        _clocks_fragment_count(clocks_fragment),
                    )
                    failure_logged = True
                time.sleep(CLOCKS_STATE_RETRY_S)

        if _hard_failure_active() and not _hard_failure_stats_repair_active():
            continue

        sequence = _clocks_fragment_count(clocks_fragment)
        # Serialize the final pre-mutation admission check with startup's decision
        # to replace newborn Alpha. Once startup arms its hold under this lock, no
        # already-running worker iteration can slip through and reset Pi custody.
        with _clocks_state_mutation_gate_lock:
            if _ppb_restore_transition_row_expected(
                clocks_fragment,
                ingress_during_restore=ingress_during_ppb_restore,
            ):
                _clocks_state_dropped += 1
                _diag["ppb_restore_transition_rows_discarded"] = (
                    _diag.get("ppb_restore_transition_rows_discarded", 0) + 1
                )
                _diag["last_ppb_restore_transition_row"] = {
                    "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                    "sequence": sequence,
                    "ingress_during_restore": bool(ingress_during_ppb_restore),
                    "transaction_active_at_consume": _ppb_restore_transaction_active.is_set(),
                    "reason": "firmware_checkpoint_temporarily_invalid_during_pi_owned_ppb_restore",
                }
                logging.info(
                    "♻️ [holistic restore] retiring transitional CLOCKS_FRAGMENT sequence=%s "
                    "while Pi-owned Better-Buckets restore is in flight",
                    sequence,
                )
                continue

            # During Alpha resurrection, newborn-epoch rows are evidence of the lost
            # physical lifetime but are not canonical successors. Retire them before
            # they can advance GNSS_RAW, reset Pi DAC chronology, or replace the
            # literal PPB image. The sole exception is the exact durable N+1 proof.
            if (
                _ambient_instrument_recovery_hold.is_set()
                or _startup_instrument_restore_hold.is_set()
            ):
                if not _ambient_instrument_restore_proof_candidate(clocks_fragment):
                    _clocks_state_dropped += 1
                    if _ambient_instrument_recovery_hold.is_set():
                        _diag["ambient_instrument_recovery_rows_retired"] = (
                            _diag.get("ambient_instrument_recovery_rows_retired", 0) + 1
                        )
                    continue

            try:
                sequence, _ = _validate_clocks_fragment_v4(clocks_fragment)
                previous_sequence = _last_clocks_state_sequence
                startup_unresolved = _startup_clocks_custody_unresolved()
                startup_regression = bool(
                    startup_unresolved
                    and previous_sequence is not None
                    and int(sequence) < int(previous_sequence)
                )
                if startup_regression and not _startup_physical_lifetime_unclassified.is_set():
                    _startup_physical_lifetime_unclassified.set()
                    boundary = {
                        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                        "previous_sequence": int(previous_sequence),
                        "observed_sequence": int(sequence),
                        "startup_custody_depth": int(len(_startup_custody_backlog)),
                    }
                    _diag["startup_physical_lifetime_boundaries"] = (
                        _diag.get("startup_physical_lifetime_boundaries", 0) + 1
                    )
                    _diag["last_startup_physical_lifetime_boundary"] = copy.deepcopy(boundary)
                    logging.warning(
                        "🧭 [clocks/startup] physical CLOCKS sequence rebased %d -> %d "
                        "before lifecycle classification; newborn rows remain visible evidence "
                        "but cannot mutate Alpha-derived Pi custody",
                        int(previous_sequence),
                        int(sequence),
                    )

                if (
                    previous_sequence is not None
                    and int(sequence) < int(previous_sequence)
                    and not startup_unresolved
                    and _begin_ambient_instrument_recovery(
                        int(previous_sequence), int(sequence)
                    )
                ):
                    # The triggering newborn row is intentionally not consumed. The
                    # recovery thread will admit only the restored durable N+1 row.
                    _clocks_state_dropped += 1
                    _diag["ambient_instrument_recovery_rows_retired"] = (
                        _diag.get("ambient_instrument_recovery_rows_retired", 0) + 1
                    )
                    continue

                # GNSS_RAW is an independent Pi-owned witness, not Alpha statistical
                # custody. Continue its exactly-once observation across the boot
                # boundary while holding only Alpha-derived mutation.
                _advance_gnss_raw_instrument(sequence, system_context)
                preclassification_newborn = bool(
                    _startup_physical_lifetime_unclassified.is_set()
                    and _startup_clocks_custody_unresolved()
                )
                state, ppb_restore_checkpoint = _build_canonical_clocks_state(
                    clocks_fragment,
                    system_context,
                    mutate_alpha_custody=not preclassification_newborn,
                )
                if preclassification_newborn:
                    _diag["startup_preclassification_rows_non_authoritative"] = (
                        _diag.get("startup_preclassification_rows_non_authoritative", 0) + 1
                    )
                    state["startup_lifetime_unclassified"] = True
            except Exception:
                _clocks_state_dropped += 1
                logging.exception(
                    "💥 [clocks] rejecting malformed/incoherent CLOCKS_FRAGMENT_V4 "
                    "sequence=%s without terminating the state worker",
                    sequence,
                )
                continue

        _cache_clocks_state(
            state,
            ppb_restore_checkpoint,
            received_monotonic=(
                float(ingress_monotonic) if ingress_monotonic is not None else None
            ),
            received_utc=(str(ingress_utc) if ingress_utc is not None else None),
        )
        if not _hard_failure_active():
            publish(CLOCKS_TOPIC, state)
            _clocks_state_published += 1

        item = {
            "state": state,
            "system_context": system_context,
            "clocks_fragment": clocks_fragment,
            "ppb_restore_checkpoint": ppb_restore_checkpoint,
        }

        # Recovery custody and ordinary routing share one lock through the final
        # persistence-queue put. A continuity-surrender thread therefore cannot
        # open custody between our routing decision and durable enqueue.
        with _recovery_custody_lock:
            if _recovery_custody_active:
                # Recovery quarantine and holistic N+1 proof are orthogonal custody
                # dimensions. Nominate the exact proof row before this branch's
                # enqueue-and-continue so the persistence worker can close durable
                # FRESH_CLOCKS_PROOF for the same quarantined observation.
                _holistic_restore_proof_custody_for_state(
                    state, ppb_restore_checkpoint
                )

            if _decorate_recovery_custody_state_locked(
                state, ppb_restore_checkpoint
            ):
                item["state"] = state
                _clocks_persist_queue.put(item)
                continue

            # Until the Teensy lifecycle probe classifies this startup as live
            # continuation or cold/full restore, every valid row stays in exact
            # startup custody.  In particular, do not let an apparent N+1 row enter
            # the narrow restore-proof lane before we know an instrument restore is
            # actually required.
            if not _clocks_persistence_enabled.is_set():
                if _retain_startup_clocks_item(item):
                    continue

            restore_proof_custody = _holistic_restore_proof_custody_for_state(
                state, ppb_restore_checkpoint
            )

            if not restore_proof_custody and not _clocks_persistence_enabled.is_set():
                # A cold/full restore has classified and retired startup custody but
                # still keeps ordinary persistence closed.  Preserve the existing
                # narrow restore-proof/epoch-birth gates; do not splice transitional rows.
                continue

            if _clocks_epoch_birth_pending.is_set():
                stats = _clocks_payload(state).get("stats")
                update_count = (
                    _as_int(stats.get("update_count"))
                    if isinstance(stats, dict)
                    else None
                )
                reset_count = (
                    _as_int(stats.get("reset_count"))
                    if isinstance(stats, dict)
                    else None
                )
                if (
                    update_count != 1
                    or reset_count is None
                    or reset_count <= _clocks_epoch_birth_prior_reset_count
                ):
                    continue
                _clocks_epoch_birth_reset_count = int(reset_count)
                _clocks_epoch_birth_pending.clear()
                logging.info(
                    "✅ [clocks] fresh statistics epoch row 1 entered persistence custody: "
                    "reset_count=%s sequence=%s",
                    reset_count,
                    state.get("sequence"),
                )

            _clocks_persist_queue.put(item)


def _clocks_persistence_loop() -> None:
    """Persist CLOCKS in order, then release embedded TEMPEST candidates."""
    global _clocks_state_persisted, _clocks_state_inserted, _clocks_state_merged
    global _last_tempest_candidate_identity, _last_tempest_candidate_monotonic

    logging.info("🚀 [clocks] CLOCKS persistence worker started")
    while True:
        item = _clocks_persist_queue.get()
        recovery_barrier = item.get("recovery_custody_barrier")
        if recovery_barrier is not None:
            recovery_barrier.set()
            continue
        if _hard_failure_active() and not _hard_failure_stats_repair_active():
            completion = item.get("persistence_completion")
            if completion is not None:
                completion.set()
            _diag["hard_failure_persistence_dropped"] = (
                _diag.get("hard_failure_persistence_dropped", 0) + 1
            )
            continue
        state = copy.deepcopy(item["state"])
        system_context = item["system_context"]
        clocks_fragment = item["clocks_fragment"]
        ppb_restore_checkpoint = item["ppb_restore_checkpoint"]
        release_campaign_candidate = bool(item.get("release_campaign_candidate", True))
        persistence_completion = item.get("persistence_completion")
        state_clocks = state.get("clocks")
        if isinstance(state_clocks, dict):
            state_clocks["persisted"] = True
            state_clocks["ephemeral"] = False

        failure_logged = False
        while True:
            try:
                worker_sequence = _as_int(state.get("sequence"))
                _set_clocks_persistence_worker_phase("WAIT_LOCK", sequence=worker_sequence)
                with _clocks_persistence_lock:
                    _set_clocks_persistence_worker_phase("LOCK_ACQUIRED", sequence=worker_sequence)
                    disposition = _persist_clocks_state(
                        state, ppb_restore_checkpoint
                    )
                    _set_clocks_persistence_worker_phase(
                        "LOCK_RELEASING", sequence=worker_sequence,
                        detail={"disposition": disposition},
                    )
                _set_clocks_persistence_worker_phase(
                    "IDLE", sequence=worker_sequence,
                    detail={"last_disposition": disposition},
                )
                _clocks_state_persisted += 1
                if disposition == "merged":
                    _clocks_state_merged += 1
                else:
                    _clocks_state_inserted += 1

                stats = _clocks_payload(state).get("stats")
                if isinstance(stats, dict):
                    persisted_update_count = _as_int(stats.get("update_count"))
                    persisted_reset_count = _as_int(stats.get("reset_count"))
                    if (
                        persisted_update_count == 1
                        and persisted_reset_count == _clocks_epoch_birth_reset_count
                    ):
                        _clocks_epoch_birth_committed.set()
                    if (
                        _clocks_holistic_restore_proof_sequence is not None
                        and int(state.get("sequence") or 0)
                            == int(_clocks_holistic_restore_proof_sequence)
                        and persisted_reset_count
                            == _clocks_holistic_restore_proof_reset_count
                        and persisted_update_count
                            == _clocks_holistic_restore_proof_update_count
                    ):
                        _clocks_holistic_restore_proof_committed.set()
                break
            except Exception:
                if _hard_failure_active() and not _hard_failure_stats_repair_active():
                    _diag["hard_failure_persistence_dropped"] = (
                        _diag.get("hard_failure_persistence_dropped", 0) + 1
                    )
                    break
                if not failure_logged:
                    logging.exception(
                        "⚠️ [clocks] campaign_detail persistence failed for CLOCKS sequence=%s; retrying",
                        state.get("sequence"),
                    )
                    failure_logged = True
                time.sleep(CLOCKS_STATE_RETRY_S)

        if _hard_failure_active():
            if persistence_completion is not None:
                persistence_completion.set()
            continue

        if persistence_completion is not None:
            persistence_completion.set()

        if not release_campaign_candidate:
            continue

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
    if _hard_failure_active() and not _hard_failure_stats_repair_active():
        _diag["hard_failure_ingress_dropped"] = (
            _diag.get("hard_failure_ingress_dropped", 0) + 1
        )
        return

    if not isinstance(payload, dict):
        _diag["clocks_fragments_malformed"] = _diag.get("clocks_fragments_malformed", 0) + 1
        return

    ingress_monotonic = time.monotonic()
    ingress_utc = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
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
            campaign_name = _tempest_campaign_name(campaign)
        except ValueError:
            public_count = None
            campaign_name = ""
        if not (
            _flash_cut_pending
            and _flash_cut_from_campaign
            and campaign_name == _flash_cut_from_campaign
        ):
            _note_timebase_activity(TIMEBASE_FRAGMENT_TOPIC, public_count)
    else:
        _diag["clocks_fragments_observation_only"] = (
            _diag.get("clocks_fragments_observation_only", 0) + 1
        )

    _queue_clocks_state(
        fragment,
        ingress_monotonic=ingress_monotonic,
        ingress_utc=ingress_utc,
    )



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

        if _hard_failure_active():
            _diag["hard_failure_campaign_rows_dropped"] = (
                _diag.get("hard_failure_campaign_rows_dropped", 0) + 1
            )
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
            logging.error(
                "💥 [clocks] processor received a malformed TEMPEST campaign piece "
                "(type=%s keys=%s); row rejected",
                type(piece).__name__,
                ",".join(sorted(piece.keys())) if isinstance(piece, dict) else "none",
            )
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

        pre_cut_tail_authority = _flash_cut_pre_cut_tail_authority(
            firmware_campaign=firmware_campaign,
            active_campaign=campaign,
            active_campaign_payload=campaign_payload,
        )
        if pre_cut_tail_authority is not None:
            _diag["flash_cut_pre_cut_tail_retired"] = (
                _diag.get("flash_cut_pre_cut_tail_retired", 0) + 1
            )
            tail = {
                "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "from": firmware_campaign,
                "to": campaign,
                "state_sequence": int(state_sequence),
                "public_count": int(public_count),
                "classification": "LAWFUL_PRE_CUT_TAIL",
                "authority": pre_cut_tail_authority,
            }
            _diag["last_flash_cut_pre_cut_tail"] = tail
            logging.info(
                "⚡ [flash cut] retiring lawful pre-cut tail after canonical persistence: "
                "from=%s to=%s sequence=%d public_count=%d authority=%s",
                firmware_campaign, campaign, int(state_sequence), int(public_count),
                pre_cut_tail_authority,
            )
            continue

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
            expected_proof_search = (
                _firmware_exclusion_is_expected_recovery_proof_search(
                    frag, firmware_exclusion
                )
            )
            if expected_proof_search:
                # The row remains fully durable audit evidence.  Only the noisy
                # per-second warning is suppressed while firmware searches for
                # its first proof-driven post-recovery science interval.
                _diag["recovery_proof_search_exclusions_suppressed"] = (
                    _diag.get("recovery_proof_search_exclusions_suppressed", 0) + 1
                )
            else:
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

        # TIMEBASE_V4 is now only a transient TEMPEST adjudication/recovery view.
        # CLOCKS is the sole public canonical clock feed; attach the Pi court
        # result to the already-persisted same-sequence CLOCKS row instead of
        # publishing a second legacy TIMEBASE topic.
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
            "💥 [start] could not deliver CLOCKS.START for campaign '%s': "
            "RPC success=%s message=%s",
            campaign,
            bool(resp.get("success")) if isinstance(resp, dict) else False,
            resp.get("message") if isinstance(resp, dict) else "malformed response",
        )
        raise TeensyStartRejected(status or "outer_rpc_failure", resp if isinstance(resp, dict) else {})

    if not isinstance(payload, dict) or not status:
        _diag["teensy_start_malformed"] = _diag.get("teensy_start_malformed", 0) + 1
        logging.error(
            "💥 [start] Teensy received CLOCKS.START for campaign '%s' but returned "
            "no usable firmware status (message=%s)",
            campaign,
            resp.get("message"),
        )
        raise TeensyStartRejected("missing_handler_status", resp)

    if status not in _TEENSY_START_ACCEPTED_STATUSES:
        _diag["teensy_start_rejected"] = _diag.get("teensy_start_rejected", 0) + 1
        logging.error(
            "💥 [start] Teensy rejected CLOCKS.START for campaign '%s': "
            "firmware status=%s error=%s (RPC message=%s)",
            campaign,
            status,
            payload.get("error") or "not reported",
            resp.get("message") or "none",
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


_TEENSY_REARM_ACCEPTED_STATUSES = {
    "rearm_requested",
    "rearm_already_active",
    "rearm_already_completed",
}


def _request_teensy_rearm(
    campaign: str,
    pps_vclock_count: int,
    args: Dict[str, Any],
) -> Dict[str, Any]:
    """Rearm Beta campaign execution without mutating the proved live Alpha."""
    teensy_args = dict(args)
    teensy_args.update({
        "campaign": str(campaign),
        "pps_count": str(int(pps_vclock_count)),
        "pps_vclock_count": str(int(pps_vclock_count)),
    })
    response = send_command(
        machine="TEENSY",
        subsystem="CLOCKS",
        command="REARM",
        args=teensy_args,
    )
    payload = response.get("payload") if isinstance(response, dict) else None
    payload = payload if isinstance(payload, dict) else {}
    status = str(payload.get("status") or "")
    if (
        not isinstance(response, dict)
        or not response.get("success")
        or status not in _TEENSY_REARM_ACCEPTED_STATUSES
    ):
        raise RuntimeError(
            "Teensy CLOCKS.REARM rejected surviving-Alpha campaign restoration: "
            f"status={status or 'missing_handler_status'} response={response!r}"
        )
    if status == "rearm_requested":
        if str(payload.get("rearm_mode") or "").strip().upper() != (
            "SURVIVING_ALPHA_CAMPAIGN_REARM"
        ):
            raise RuntimeError(
                "Teensy CLOCKS.REARM accepted without the surviving-Alpha contract"
            )
        if payload.get("alpha_mutated") is not False:
            raise RuntimeError(
                "Teensy CLOCKS.REARM did not explicitly preserve Alpha custody"
            )
    return response


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
    if bool((_diag.get("last_recovery") or {}).get("combined_dead_producer_restore")):
        with _clocks_state_mutation_gate_lock:
            _clocks_holistic_restore_proof_pending.clear()
            _clocks_holistic_restore_proof_committed.clear()
            _startup_instrument_restore_hold.clear()
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

    try:
        location_status = _ensure_system_location(
            context="FLASH_CUT_START" if flash_cut else "CAMPAIGN_START"
        )
    except Exception as e:
        return {"success": False, "message": str(e)}

    location = location_status.get("current_location")
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

    # Campaign provenance snapshots the SYSTEM selection exactly.  NONE remains
    # NONE; a Flash Cut must never resurrect the previous campaign's location.
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
        else None
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
        location_status = _ensure_system_location(context="CAMPAIGN_STOP")
        logging.info(
            "📍 [clocks] stop location reconciliation complete: "
            "SYSTEM=%s campaign_snapshot=%s GF-8802=%s",
            location_status.get("current_location") or "NONE",
            stop_location or "NONE",
            location_status.get("verified_pos_mode_name") or "?",
        )
    except Exception:
        logging.exception("⚠️ [clocks] failed to reconcile SYSTEM location at stop (ignored)")

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
                "operational_state": _operational_state_snapshot(),
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
                "operational_state": _operational_state_snapshot(),
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
        "operational_state": _operational_state_snapshot(),
        "owners": {
            "teensy": ["GNSS", "VCLOCK", "DWT", "OCXO1", "OCXO2"],
            "pi": ["GNSS_RAW", "DAC", "SERVO", "DITHER"],
        },
        "teensy": teensy_response,
        "pi": {
            "gnss_raw": _gnss_raw_clock_snapshot(),
            "control": _dac_control_snapshot(),
            "dac": _dac_info_payload(),
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


def _current_live_stats_reset_count() -> int:
    """Return the reset_count of the latest canonical live CLOCKS state."""
    with _clocks_lock:
        state = copy.deepcopy(_latest_clocks)
    stats = _clocks_payload(state).get("stats")
    reset_count = (
        _as_int(stats.get("reset_count"))
        if isinstance(stats, dict)
        else None
    )
    if reset_count is None or reset_count < 0:
        raise RuntimeError("latest live CLOCKS state has no usable stats.reset_count")
    return int(reset_count)


def _perform_transitive_stats_reset(
    *,
    requested_at: str,
    pi_before: Dict[str, Any],
    source: str,
) -> Dict[str, Any]:
    """Reset Teensy-owned and Pi-owned CLOCKS statistics as one transaction."""
    global _dac_stats_reset_fence_count

    prior_reset_count = _current_live_stats_reset_count()
    with _dac_stats_epoch_lock:
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
                "source": source,
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
                "source": source,
                "teensy_response": teensy_response,
                "pi_gnss_raw_before": pi_before,
                "pi_reset_applied": False,
            }
            _diag["last_stats_reset"] = failure
            logging.error(
                "❌ [clocks] Teensy rejected STATS_RESET; Pi statistics were left untouched "
                "(success=%s status=%s message=%s)",
                bool(teensy_response.get("success")) if isinstance(teensy_response, dict) else False,
                _path_get(teensy_response, "payload.status") or "not reported",
                teensy_response.get("message") if isinstance(teensy_response, dict) else "malformed response",
            )
            return {
                "success": False,
                "message": "Teensy CLOCKS.STATS_RESET rejected",
                "payload": failure,
            }

        pi_previous = _gnss_raw_welford_reset()
        pi_after = _gnss_raw_welford_snapshot()
        dac_stats = _dac_reset_statistics()

        # Keep pre-reset rows already queued in the Pi from repopulating the new
        # DAC statistics epoch after this transaction releases the row consumer.
        # The first row carrying a different Teensy reset_count clears the fence.
        _dac_stats_reset_fence_count = int(prior_reset_count)

    completed_at = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    result = {
        "requested_at_utc": requested_at,
        "completed_at_utc": completed_at,
        "success": True,
        "source": source,
        "scope": "SYSTEMWIDE_CLOCK_STATISTICS",
        "campaign_unchanged": True,
        "clockfaces_unchanged": True,
        "teensy": teensy_response,
        "pi_gnss_raw_before": pi_previous,
        "pi_gnss_raw_after": pi_after,
        "pi_dac_welford": dac_stats,
    }
    _diag["stats_reset_success"] = _diag.get("stats_reset_success", 0) + 1
    _diag["last_stats_reset"] = result
    logging.warning(
        "📊 [clocks] transitive STATS_RESET complete (%s): Teensy accepted; "
        "GNSS_RAW N %d -> %d; campaign and clockfaces preserved",
        source,
        int(pi_previous.get("n") or 0),
        int(pi_after.get("n") or 0),
    )
    return {"success": True, "message": "OK", "payload": result}


def _establish_fresh_durable_stats_epoch(
    *,
    source: str = "HOLISTIC_RESTORE_EPOCH_BIRTH",
) -> Dict[str, Any]:
    """Align fresh Alpha epoch birth with the durable persistence boundary."""
    global _clocks_epoch_birth_prior_reset_count
    global _clocks_epoch_birth_reset_count

    prior_reset_count = _current_live_stats_reset_count()
    _clocks_epoch_birth_prior_reset_count = int(prior_reset_count)
    _clocks_epoch_birth_reset_count = -1
    _clocks_epoch_birth_committed.clear()

    _diag["stats_reset_requests"] = _diag.get("stats_reset_requests", 0) + 1
    requested_at = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    pi_before = _gnss_raw_welford_snapshot()

    # Open the writer before reset, but hold all rows until Alpha publishes row 1
    # of a strictly newer reset_count.  This removes both race directions:
    # pre-reset startup rows cannot become durable, and new-epoch row 1 cannot
    # disappear through a still-closed persistence boundary.
    _clocks_epoch_birth_pending.set()
    _clocks_persistence_enabled.set()

    response = _perform_transitive_stats_reset(
        requested_at=requested_at,
        pi_before=pi_before,
        source=source,
    )
    if not response.get("success"):
        raise RuntimeError(
            "fresh durable CLOCKS statistics epoch could not be established: "
            f"{response.get('message') or response!r}"
        )

    if not _clocks_epoch_birth_committed.wait(timeout=HOLISTIC_RESTORE_TIMEOUT_S):
        raise RuntimeError(
            "fresh CLOCKS statistics epoch did not durably commit update_count=1 "
            f"within {HOLISTIC_RESTORE_TIMEOUT_S:.1f}s "
            f"(prior_reset_count={prior_reset_count}, "
            f"observed_reset_count={_clocks_epoch_birth_reset_count})"
        )

    return {
        "restored": False,
        "fresh_epoch": True,
        "persistence_open": True,
        "reset_count": int(_clocks_epoch_birth_reset_count),
        "first_durable_update_count": 1,
        "stats_reset": response.get("payload") or {},
    }


def _stats_reset_worker(*, requested_at: str, pi_before: Dict[str, Any]) -> None:
    """Complete one accepted operator STATS_RESET after the command has returned."""
    try:
        response = _perform_transitive_stats_reset(
            requested_at=requested_at,
            pi_before=pi_before,
            source="CLOCKS.STATS_RESET",
        )
        if not isinstance(response, dict):
            raise RuntimeError(f"CLOCKS STATS_RESET worker returned malformed result: {response!r}")
        if not response.get("success"):
            logging.error(
                "💥 [clocks] asynchronous STATS_RESET did not complete: %s",
                response.get("message") or _path_get(response, "payload.error") or "unspecified failure",
            )
    except Exception as exc:
        failure = {
            "requested_at_utc": requested_at,
            "completed_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "success": False,
            "stage": "ASYNC_WORKER_EXCEPTION",
            "source": "CLOCKS.STATS_RESET",
            "error": str(exc),
            "pi_gnss_raw_before": copy.deepcopy(pi_before),
            "pi_reset_applied": False,
        }
        _diag["last_stats_reset"] = failure
        logging.exception("💥 [clocks] asynchronous STATS_RESET worker crashed")
    finally:
        _stats_reset_in_progress.clear()


def cmd_stats_reset(_: Optional[dict]) -> Dict[str, Any]:
    """Accept one CLOCKS statistics reset and return before reset completion."""
    _diag["stats_reset_requests"] = _diag.get("stats_reset_requests", 0) + 1
    startup_busy = _startup_control_gate("STATS_RESET")
    if startup_busy is not None:
        return startup_busy

    requested_at = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    pi_before = _gnss_raw_welford_snapshot()

    with _stats_reset_control_lock:
        if _stats_reset_in_progress.is_set():
            return {
                "success": True,
                "message": "CLOCKS STATS_RESET is already in progress",
                "payload": {
                    "status": "statistics_reset_in_progress",
                    "asynchronous": True,
                    "last": copy.deepcopy(_diag.get("last_stats_reset") or {}),
                },
            }

        accepted = {
            "requested_at_utc": requested_at,
            "success": None,
            "status": "statistics_reset_requested",
            "source": "CLOCKS.STATS_RESET",
            "scope": "SYSTEMWIDE_CLOCK_STATISTICS",
            "asynchronous": True,
            "campaign_unchanged": True,
            "pi_gnss_raw_before": copy.deepcopy(pi_before),
            "completion_report": "CLOCKS.REPORT_STATS.stats_reset.last",
        }
        _stats_reset_in_progress.set()
        _diag["last_stats_reset"] = copy.deepcopy(accepted)
        threading.Thread(
            target=_stats_reset_worker,
            kwargs={"requested_at": requested_at, "pi_before": copy.deepcopy(pi_before)},
            daemon=True,
            name="clocks-stats-reset",
        ).start()

    return {"success": True, "message": "OK", "payload": accepted}

def _destructive_repair_lineage_reset() -> Dict[str, Any]:
    """Irrevocably abandon every durable TEMPEST/CLOCKS restore ancestor.

    This is the terminal escape hatch for an information-theoretically dead
    statistics lineage.  The operator invoked CLOCKS.REPAIR; the repair planner selected a destructive
    statistics-lineage cut because no narrower known remedy is sufficient.
    preserving a campaign namespace or stale CLOCKS_RECOVERY owner across that
    boundary would merely allow the next process lifetime to rediscover the poison.
    """
    global _campaign_active, _ppb_checkpoint_runtime
    global _recovery_custody_active, _recovery_custody_generation
    global _recovery_custody_entered_at_utc, _recovery_custody_reason
    global _recovery_custody_physical_sequence_regression
    global _recovery_custody_regression_witness, _recovery_custody_last_checkpoint

    _request_teensy_stop_best_effort()
    _request_teensy_recover_abort_best_effort("operator_repair_destructive_lineage_reset")
    _campaign_active = False
    _clear_start_wait_state()
    _clear_flash_cut_wait_state()
    candidate_drained = _reset_trackers()

    _clocks_persistence_enabled.clear()
    _clocks_epoch_birth_pending.clear()
    _clocks_epoch_birth_committed.clear()
    _clocks_holistic_restore_proof_pending.clear()
    _clocks_holistic_restore_proof_committed.clear()
    _startup_instrument_restore_hold.clear()
    _startup_physical_lifetime_unclassified.clear()

    retired = _retire_startup_clocks_custody(
        "hard_failure_statistics_lineage_destructive_reset"
    )

    with _recovery_custody_lock:
        _recovery_custody_active = False
        _recovery_custody_generation = None
        _recovery_custody_entered_at_utc = None
        _recovery_custody_reason = None
        _recovery_custody_physical_sequence_regression = False
        _recovery_custody_regression_witness = {}
        _recovery_custody_last_checkpoint = None

    with _ppb_checkpoint_lock:
        _ppb_checkpoint_runtime = _ppb_checkpoint_new_runtime(
            reason="DESTRUCTIVE_REPAIR_LINEAGE_RESET"
        )

    with _clocks_persistence_lock:
        state_drained = _drain_clocks_persistence_queue()
        with open_db() as conn:
            cur = conn.cursor()
            cur.execute(
                "DELETE FROM campaign_detail WHERE campaign_type = %s",
                (CAMPAIGN_TYPE_TEMPEST,),
            )
            detail_count = int(cur.rowcount or 0)
            cur.execute(
                "DELETE FROM campaign_master WHERE campaign_type = %s",
                (CAMPAIGN_TYPE_TEMPEST,),
            )
            master_count = int(cur.rowcount or 0)
            cur.execute(
                "DELETE FROM config WHERE config_key = %s",
                (CLOCKS_RECOVERY_CONFIG_KEY,),
            )
            recovery_config_count = int(cur.rowcount or 0)

    return {
        "campaign_details_deleted": detail_count,
        "campaign_masters_deleted": master_count,
        "recovery_config_deleted": recovery_config_count,
        "candidate_ingress_drained": int(candidate_drained),
        "pending_state_rows_drained": int(state_drained),
        "startup_custody_retired": retired,
    }


def _release_after_repair(
    *,
    reason: str,
    details: Dict[str, Any],
) -> Dict[str, Any]:
    """Return this process to ordinary idle CLOCKS service after lineage amputation."""
    global _operational_state

    now_utc = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    with _hard_failure_lock:
        _hard_failure_event.clear()
        with _operational_state_lock:
            _operational_state = {
                "schema": OPERATIONAL_STATE_SCHEMA,
                "subsystem": "CLOCKS",
                "state": OPERATIONAL_STATE_RUNNING,
                "entered_at_utc": now_utc,
                "reason": str(reason),
                "source": "CLOCKS.REPAIR",
                "details": copy.deepcopy(details),
            }
            snapshot = copy.deepcopy(_operational_state)

    _startup_control_ready.set()
    _diag["startup_control_ready"] = True
    _clocks_persistence_enabled.set()
    _dac_control_wakeup.set()
    return snapshot


def _repair_fresh_epoch_witness(epoch: Dict[str, Any]) -> Dict[str, Any]:
    """Prove row 1 of the replacement statistics epoch is durably usable."""
    new_reset_count = int(epoch["reset_count"])
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, payload
            FROM campaign_detail
            WHERE campaign_type = %s
              AND payload #>> '{schema}' = 'CLOCKS_V4'
              AND (payload #>> '{clocks,stats,reset_count}')::bigint = %s
              AND (payload #>> '{clocks,stats,update_count}')::bigint = 1
            ORDER BY id DESC
            LIMIT 1
            """,
            (CAMPAIGN_TYPE_TEMPEST, new_reset_count),
        )
        row = cur.fetchone()

    if row is None:
        raise RuntimeError(
            "fresh statistics epoch reported durable row 1 but PostgreSQL witness is missing"
        )
    durable = row["payload"]
    if isinstance(durable, str):
        durable = json.loads(durable)
    if not isinstance(durable, dict):
        raise RuntimeError("fresh statistics epoch row 1 payload is not an object")

    ancestry = _dac_restore_population_ancestry_court(_clocks_payload(durable))
    if not ancestry.get("available") or not ancestry.get("valid"):
        raise RuntimeError(
            "fresh statistics epoch row 1 failed DAC/OCXO population ancestry court: "
            f"{ancestry!r}"
        )
    return {
        "new_reset_count": new_reset_count,
        "first_durable_update_count": 1,
        "first_durable_detail_id": int(row["id"]),
        "population_ancestry": copy.deepcopy(ancestry),
    }


def _hard_failure_stats_repair_worker(
    *,
    state: Dict[str, Any],
    repair_record: Dict[str, Any],
) -> None:
    """Execute one CLOCKS.REPAIR request without racing startup reconciliation.

    The command socket is intentionally live before holistic startup reconciliation
    completes.  A repair request received in that window is therefore a queued
    operator intent, not authority to run a second lineage transaction concurrently
    with the startup owner.  Wait for startup to reach a terminal state, then
    re-diagnose from current truth and only then open the narrow HARD_FAILURE repair
    data plane.
    """
    repaired = False
    reset_details: Dict[str, Any] = {}
    strategy = str(repair_record.get("strategy") or "")
    repair_plane_open = False
    try:
        if _startup_reconciliation_active.is_set():
            repair_record["phase"] = "WAITING_FOR_STARTUP_RECONCILIATION"
            repair_record["startup_reconciliation_active"] = True
            repair_record["next_action"] = (
                "REPAIR is queued behind the one startup reconciliation owner; "
                "no concurrent lineage mutation will be attempted."
            )
            _diag["last_repair"] = copy.deepcopy(repair_record)
            logging.warning(
                "🧯 [clocks] REPAIR queued while startup reconciliation owns CLOCKS; "
                "waiting for that transaction to reach a terminal state"
            )
            wait_started = time.monotonic()
            next_wait_log = wait_started + 10.0
            while _startup_reconciliation_active.is_set():
                now = time.monotonic()
                if now >= next_wait_log:
                    wait_state = _operational_state_snapshot()
                    wait_snapshot = {
                        "phase": "WAITING_FOR_STARTUP_RECONCILIATION",
                        "startup_reconciliation_active": True,
                        "waited_s": round(now - wait_started, 3),
                        "operational_state": wait_state,
                        "startup_control_ready": _startup_control_ready.is_set(),
                        "startup_custody_unresolved": _startup_clocks_custody_unresolved(),
                        "startup_instrument_restore_hold": _startup_instrument_restore_hold.is_set(),
                        "holistic_restore_proof_pending": _clocks_holistic_restore_proof_pending.is_set(),
                        "epoch_birth_pending": _clocks_epoch_birth_pending.is_set(),
                        "ppb_restore_transaction_active": _ppb_restore_transaction_active.is_set(),
                    }
                    repair_record.update(wait_snapshot)
                    repair_record["next_action"] = (
                        "Startup still owns reconciliation; REPAIR remains queued and will "
                        "begin automatically when that ownership boundary is released."
                    )
                    _diag["last_repair"] = copy.deepcopy(repair_record)
                    logging.warning(
                        "🧯 [clocks] REPAIR still queued after %.1fs: state=%s "
                        "startup_custody_unresolved=%s restore_hold=%s proof_pending=%s "
                        "epoch_birth_pending=%s ppb_restore_active=%s",
                        now - wait_started,
                        wait_state.get("state"),
                        wait_snapshot["startup_custody_unresolved"],
                        wait_snapshot["startup_instrument_restore_hold"],
                        wait_snapshot["holistic_restore_proof_pending"],
                        wait_snapshot["epoch_birth_pending"],
                        wait_snapshot["ppb_restore_transaction_active"],
                    )
                    next_wait_log = now + 10.0
                time.sleep(0.05)

            state = _operational_state_snapshot()
            state_name = str(state.get("state") or "").strip().upper()
            startup_ready = _startup_control_ready.is_set()
            if (
                state_name == OPERATIONAL_STATE_RUNNING
                and startup_ready
                and not _hard_failure_active()
            ):
                repaired = True
                repair_record.update({
                    "phase": "NO_ACTION_STARTUP_SELF_REPAIRED",
                    "completed_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                    "success": True,
                    "strategy": "NO_ACTION_STARTUP_SELF_REPAIRED",
                    "startup_reconciliation_active": False,
                    "operational_state": state,
                    "hard_failure_remains_latched": False,
                    "restart_required": False,
                    "next_action": "Startup reconciliation completed successfully; no additional repair was required.",
                })
                _diag["repair_success"] = _diag.get("repair_success", 0) + 1
                _diag["last_repair"] = copy.deepcopy(repair_record)
                logging.critical(
                    "🧯 [clocks] queued REPAIR closed as no-op: startup reconciliation repaired CLOCKS itself"
                )
                return

            # The startup owner has finished and left CLOCKS non-running. Re-plan
            # against the terminal state rather than executing the stale strategy
            # selected when the command first arrived.
            failure_reason = str(state.get("reason") or "")
            strategy = (
                "DESTRUCTIVE_STATS_LINEAGE_RESET"
                if _hard_failure_active() and failure_reason in _DESTRUCTIVE_REPAIR_REASONS
                else "PRESERVE_HISTORY_FRESH_ALPHA"
            )
            repair_record.update({
                "phase": "IN_PROGRESS",
                "strategy": strategy,
                "trigger_state_after_startup": state_name,
                "trigger_reason_after_startup": failure_reason,
                "startup_reconciliation_active": False,
                "startup_control_ready": startup_ready,
                "destructive": strategy == "DESTRUCTIVE_STATS_LINEAGE_RESET",
                "next_action": "Wait for CLOCKS.REPORT.repair.success=true.",
            })
            _diag["last_repair"] = copy.deepcopy(repair_record)

        # Only the actual repair transaction receives permission to pass the
        # HARD_FAILURE ingress/persistence gates.  Waiting requests never do.
        _hard_failure_stats_repair_event.set()
        repair_plane_open = True

        if strategy == "PRESERVE_HISTORY_FRESH_ALPHA":
            repair_record["phase"] = "SURRENDERING_BROKEN_LINEAGE"
            _diag["last_repair"] = copy.deepcopy(repair_record)
            active_campaign = _get_active_campaign()
            verdict = AlphaResurrectionImpossible(
                "operator_repair_lineage_surrender",
                {
                    "operational_state": copy.deepcopy(state),
                    "repair_strategy": strategy,
                    "startup_control_ready": _startup_control_ready.is_set(),
                },
            )
            surrender = _surrender_unresurrectable_alpha_lineage(
                verdict=verdict,
                active_campaign=active_campaign,
                source="CLOCKS.REPAIR",
            )
            epoch = copy.deepcopy(surrender.get("fresh_epoch") or {})
            if not epoch:
                raise RuntimeError("lineage surrender did not return a fresh statistics epoch")
            reset_details["lineage_surrender"] = copy.deepcopy(surrender)
        elif strategy == "DESTRUCTIVE_STATS_LINEAGE_RESET":
            repair_record["phase"] = "DESTROYING_DEAD_LINEAGE"
            _diag["last_repair"] = copy.deepcopy(repair_record)
            lineage_reset = _destructive_repair_lineage_reset()
            reset_details["destructive_lineage_reset"] = copy.deepcopy(lineage_reset)
            repair_record["phase"] = "RESETTING_STATISTICS"
            _diag["last_repair"] = copy.deepcopy(repair_record)
            epoch = _establish_fresh_durable_stats_epoch(source="CLOCKS.REPAIR")
        else:
            raise RuntimeError(f"unsupported CLOCKS.REPAIR strategy {strategy!r}")

        witness = _repair_fresh_epoch_witness(epoch)
        release_details = {
            **copy.deepcopy(reset_details),
            **copy.deepcopy(witness),
            "strategy": strategy,
        }
        operational_state = _release_after_repair(
            reason="operator_repair_complete",
            details=release_details,
        )
        repaired = True

        repair_record.update({
            "phase": "SUCCEEDED",
            "completed_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "success": True,
            **copy.deepcopy(witness),
            "stats_reset": copy.deepcopy(epoch.get("stats_reset") or {}),
            "repair_details": copy.deepcopy(reset_details),
            "hard_failure_remains_latched": False,
            "restart_required": False,
            "operational_state": operational_state,
            "next_action": "CLOCKS is usable now; start a new campaign when desired.",
        })
        _diag["repair_success"] = _diag.get("repair_success", 0) + 1
        _diag["last_repair"] = copy.deepcopy(repair_record)
        logging.critical(
            "🧯 [clocks] REPAIR completed: strategy=%s reset_count=%d detail_id=%d; service resumed",
            strategy,
            int(witness["new_reset_count"]),
            int(witness["first_durable_detail_id"]),
        )
    except Exception as exc:
        # REPAIR is the operator escape hatch. If the orderly lineage-preserving
        # repair cannot prove a replacement row, amputate the entire continuation
        # domain and reopen on the live producer rather than leaving CLOCKS wedged.
        logging.exception(
            "⚠️ [clocks] selected REPAIR strategy failed; falling back to empty-domain service restoration"
        )
        try:
            fallback_reset = _destructive_repair_lineage_reset()
            _clocks_epoch_birth_pending.clear()
            _clocks_epoch_birth_committed.clear()
            release_details = {
                "selected_strategy": strategy,
                "selected_strategy_error": str(exc),
                "fallback_lineage_reset": copy.deepcopy(fallback_reset),
                "durability_basis": "EMPTY_DOMAIN_NEXT_COHERENT_ROW",
            }
            operational_state = _release_after_repair(
                reason="operator_repair_empty_domain_fallback",
                details=release_details,
            )
            repaired = True
            repair_record.update({
                "phase": "SUCCEEDED_EMPTY_DOMAIN_FALLBACK",
                "completed_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "success": True,
                "selected_strategy_error": str(exc),
                "fallback_lineage_reset": copy.deepcopy(fallback_reset),
                "hard_failure_remains_latched": False,
                "restart_required": False,
                "operational_state": operational_state,
                "next_action": (
                    "CLOCKS is usable now; the next coherent live row begins the new durability domain."
                ),
            })
            _diag["repair_success"] = _diag.get("repair_success", 0) + 1
            _diag["last_repair"] = copy.deepcopy(repair_record)
            logging.critical(
                "🧯 [clocks] REPAIR recovered through empty-domain fallback; prior restore ancestry is retired"
            )
        except Exception as fallback_exc:
            repair_record.update({
                "phase": "FAILED",
                "completed_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "success": False,
                "error": str(exc),
                "fallback_error": str(fallback_exc),
            })
            _diag["repair_failures"] = _diag.get("repair_failures", 0) + 1
            _diag["last_repair"] = copy.deepcopy(repair_record)
            logging.exception("🛑 [clocks] REPAIR failed; CLOCKS remains fail-closed")
    finally:
        if repair_plane_open:
            _hard_failure_stats_repair_event.clear()
        _repair_request_in_progress.clear()
        _clocks_epoch_birth_pending.clear()
        if not repaired:
            _clocks_persistence_enabled.clear()


def cmd_repair(_: Optional[dict]) -> Dict[str, Any]:
    """Diagnose known CLOCKS wedges and execute the least-destructive known repair."""
    state = _operational_state_snapshot()
    state_name = str(state.get("state") or "").strip().upper()
    failure_reason = str(state.get("reason") or "")
    startup_ready = _startup_control_ready.is_set()

    if (
        state_name == OPERATIONAL_STATE_RUNNING
        and startup_ready
        and not _hard_failure_active()
    ):
        with _ppb_checkpoint_lock:
            checkpoint = _ppb_checkpoint_snapshot_locked(
                _ppb_checkpoint_runtime_ensure_locked()
            )
        return {
            "success": True,
            "message": "CLOCKS is already RUNNING; no repair required",
            "payload": {
                "action": "NO_ACTION_HEALTHY",
                "operational_state": state,
                "startup_control_ready": True,
                "checkpoint_recoverable": bool(checkpoint.get("recoverable")),
                "checkpoint_status": checkpoint.get("status"),
            },
        }

    with _hard_failure_stats_repair_lock:
        if _repair_request_in_progress.is_set():
            return {
                "success": True,
                "message": "CLOCKS REPAIR is already in progress",
                "payload": copy.deepcopy(_diag.get("last_repair") or {}),
            }
        if _stats_reset_in_progress.is_set():
            return {
                "success": False,
                "message": "CLOCKS REPAIR deferred while an explicit STATS_RESET owns the epoch boundary",
                "payload": {"last_stats_reset": copy.deepcopy(_diag.get("last_stats_reset") or {})},
            }
        if _auto_recovery_in_progress or _timebase_silence_recovery_active or _ambient_instrument_recovery_active.is_set():
            return {
                "success": False,
                "message": "CLOCKS REPAIR deferred while an automatic recovery transaction is actively progressing",
                "payload": {
                    "auto_recovery_in_progress": bool(_auto_recovery_in_progress),
                    "timebase_silence_recovery_active": bool(_timebase_silence_recovery_active),
                    "ambient_instrument_recovery_active": _ambient_instrument_recovery_active.is_set(),
                },
            }

        strategy = (
            "DESTRUCTIVE_STATS_LINEAGE_RESET"
            if _hard_failure_active() and failure_reason in _DESTRUCTIVE_REPAIR_REASONS
            else "PRESERVE_HISTORY_FRESH_ALPHA"
        )
        requested_at = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
        _diag["repair_requests"] = _diag.get("repair_requests", 0) + 1
        repair_record: Dict[str, Any] = {
            "schema": "PI_CLOCKS_REPAIR_V1",
            "requested_at_utc": requested_at,
            "phase": "IN_PROGRESS",
            "success": None,
            "strategy": strategy,
            "trigger_state": state_name,
            "trigger_reason": failure_reason,
            "startup_control_ready": startup_ready,
            "destructive": strategy == "DESTRUCTIVE_STATS_LINEAGE_RESET",
            "implicit_statistics_reset_allowed": True,
            "operator_arguments_required": False,
            "next_action": "Wait for CLOCKS.REPORT.repair.success=true.",
        }
        if _startup_reconciliation_active.is_set():
            repair_record["phase"] = "WAITING_FOR_STARTUP_RECONCILIATION"
            repair_record["startup_reconciliation_active"] = True
            repair_record["next_action"] = (
                "REPAIR is queued behind startup reconciliation; wait for CLOCKS.REPORT.repair.success=true."
            )
        _diag["last_repair"] = copy.deepcopy(repair_record)
        _repair_request_in_progress.set()
        try:
            threading.Thread(
                target=_hard_failure_stats_repair_worker,
                kwargs={
                    "state": copy.deepcopy(state),
                    "repair_record": repair_record,
                },
                daemon=True,
                name="clocks-repair",
            ).start()
        except Exception as exc:
            _repair_request_in_progress.clear()
            _hard_failure_stats_repair_event.clear()
            repair_record.update({
                "phase": "FAILED_TO_START",
                "success": False,
                "completed_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "error": str(exc),
            })
            _diag["repair_failures"] = _diag.get("repair_failures", 0) + 1
            _diag["last_repair"] = copy.deepcopy(repair_record)
            return {
                "success": False,
                "message": f"CLOCKS REPAIR could not start repair worker: {exc}",
                "payload": repair_record,
            }

    return {
        "success": True,
        "message": f"CLOCKS REPAIR accepted; selected {strategy}",
        "payload": copy.deepcopy(repair_record),
    }

def cmd_report(_: Optional[dict]) -> Dict[str, Any]:
    row = _get_active_campaign()
    with _ppb_checkpoint_lock:
        ppb_checkpoint = _ppb_checkpoint_snapshot_locked(
            _ppb_checkpoint_runtime_ensure_locked()
        )
    contract = {
        "mode_free": True,
        "coherent_rows_persist": True,
        "science_exclusions_are_audit_only": True,
        "continuity_fatal_triggers_recovery": True,
        "unresurrectable_old_lineage_starts_fresh_alpha": True,
        "raw_postgresql_reconstruction_prohibited": True,
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
                "operational_state": _operational_state_snapshot(),
                "repair": {
                    **copy.deepcopy(_diag.get("last_repair") or {}),
                    "active": _hard_failure_stats_repair_active(),
                },
                "ppb_restore_checkpoint": copy.deepcopy(ppb_checkpoint),
            },
        }

    payload = dict(row["payload"])
    payload["campaign_type"] = row["campaign_type"]
    payload["campaign"] = row["campaign"]
    payload["integrity_contract"] = contract
    payload["startup"] = _start_status_payload()
    payload["operational_state"] = _operational_state_snapshot()
    payload["repair"] = {
        **copy.deepcopy(_diag.get("last_repair") or {}),
        "active": _hard_failure_stats_repair_active(),
    }
    payload["ppb_restore_checkpoint"] = copy.deepcopy(ppb_checkpoint)
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
                cur.execute(
                    "DELETE FROM config WHERE config_key = %s",
                    (CLOCKS_RECOVERY_CONFIG_KEY,),
                )
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
    """Reject the retired recovery-based campaign RESUME operation.

    A stopped producer campaign is historical truth.  Recreating it used to rely
    on the now-retired campaign-only recovery/rearm path.  Automatic recovery may
    only prove-and-adopt a surviving producer or restore a dead producer from one
    complete desired state; it never manufactures a new campaign boundary.

    If a future operator-level RESUME is desired, it must be implemented as an
    explicit first-class lifecycle protocol with its own durable boundary rather
    than borrowing recovery machinery.
    """
    startup_busy = _startup_control_gate("RESUME")
    if startup_busy is not None:
        return startup_busy
    campaign = str((args or {}).get("campaign") or "").strip()
    return {
        "success": False,
        "message": (
            "RESUME is retired: stopped TEMPEST campaigns are not replayed by "
            "recovery. START a new campaign, or add a first-class resume lifecycle "
            "protocol with an explicit durable boundary."
        ),
        "payload": {
            "campaign_type": CAMPAIGN_TYPE_TEMPEST,
            "campaign": campaign or None,
            "producer_mutated": False,
            "retired_path": "CAMPAIGN_RECOVERY_REARM",
        },
    }


# ---------------------------------------------------------------------
# Recovery — v4 Nanosecond Architecture
# ---------------------------------------------------------------------




def _restore_active_campaign_state(
    *,
    preverified_location: Optional[Dict[str, Any]] = None,
    startup_instrument_verdict: Optional[_HolisticInstrumentVerdict] = None,
) -> Dict[str, Any]:
    """Restore one dead producer and its durable TEMPEST state atomically.

    Surviving producers never reach this executor: they exit through the generic
    prove-and-adopt branch.  This function therefore has one mutation mode only:
    project the durable snapshot, install producer + campaign desired state in one
    DEAD_PRODUCER_RESTORE transaction, prove exact producer N+1, and admit the projected
    first public TEMPEST row.
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

    if preverified_location is not None:
        location_status = copy.deepcopy(preverified_location)
    else:
        try:
            location_status = _ensure_system_location(context="CAMPAIGN_RECOVERY")
        except Exception as e:
            raise RuntimeError(f"recovery failed while restoring SYSTEM location: {e}")

    system_location = location_status.get("current_location")
    logging.info(
        "🔍 [recovery] active campaign='%s' campaign_location=%s "
        "SYSTEM current_location=%s GF-8802 position_mode=%s freq_mode=%s",
        campaign_name,
        campaign_location or "NONE",
        system_location or "NONE",
        location_status.get("verified_pos_mode_name") or "?",
        location_status.get("freq_mode_name") or "?",
    )

    # ------------------------------------------------------------------
    # Step 1: Bind one declarative CLOCKS recovery authority
    # ------------------------------------------------------------------
    combined_snapshot = _load_clocks_recovery_snapshot()
    snapshot_campaign = (
        copy.deepcopy(combined_snapshot.campaign)
        if combined_snapshot is not None and isinstance(combined_snapshot.campaign, dict)
        else None
    )
    snapshot_campaign_name = (
        _tempest_campaign_name(snapshot_campaign)
        if isinstance(snapshot_campaign, dict)
        and snapshot_campaign.get("schema") == "TEMPEST_FRAGMENT_V1"
        else ""
    )
    snapshot_campaign_state = (
        str(snapshot_campaign.get("state") or "").strip().upper()
        if isinstance(snapshot_campaign, dict)
        else ""
    )
    snapshot_public_count = (
        _tempest_public_count(snapshot_campaign)
        if isinstance(snapshot_campaign, dict)
        and snapshot_campaign.get("schema") == "TEMPEST_FRAGMENT_V1"
        and snapshot_campaign_name
        else 0
    )
    snapshot_owns_running_campaign = bool(
        combined_snapshot is not None
        and snapshot_campaign_name == campaign_name
        and snapshot_campaign_state == "STARTED"
        and snapshot_public_count > 0
    )

    if not snapshot_owns_running_campaign:
        _diag["recovery_missing_timebase"] += 1
        if _reattach_pending_flash_cut_without_recovery(
            campaign_name=campaign_name,
            campaign_payload=campaign_payload,
        ):
            _diag["last_recovery"] = {
                "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "campaign": campaign_name,
                "mode": "flash_cut_zero_row_reattach",
                "waiting_for_first_fragment": True,
                "producer_mutated": False,
            }
            return {
                "restored": True,
                "mode": "flash_cut_zero_row_reattach",
                "campaign": campaign_name,
                "producer_mutated": False,
            }

        # A campaign absent from the latest recovery snapshot has no producer-authored
        # durable boundary that can lawfully be recreated after producer loss. Before
        # retiring that Pi intent, give a surviving producer the generic prove/adopt
        # opportunity using the exact same instrument snapshot.
        if startup_instrument_verdict is None and combined_snapshot is not None:
            instrument_detail = combined_snapshot.restore_detail()
            _instrument_probe, startup_instrument_verdict = (
                _restore_instrument_from_clocks(instrument_detail)
            )
            if (
                str(startup_instrument_verdict.alpha_disposition).strip().upper()
                == "SURVIVED"
            ):
                return _adopt_surviving_clocks_producer(
                    snapshot_detail=instrument_detail,
                    active_campaign=row,
                    instrument_verdict=startup_instrument_verdict,
                )

        if combined_snapshot is None and _has_tempest_state_details(campaign_name):
            raise RuntimeError(
                "active TEMPEST has durable adjudicated history but no CLOCKS_RECOVERY "
                "snapshot; dead-producer desired state is undefined"
            )

        stopped_at = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
        with open_db() as conn:
            cur = conn.cursor()
            cur.execute(
                """
                UPDATE campaign_master
                SET active = false,
                    payload = payload || jsonb_build_object(
                        'stopped_at', to_jsonb(%s::text),
                        'recovery_not_replayed_at', to_jsonb(%s::text),
                        'recovery_not_replayed_reason', to_jsonb(%s::text)
                    )
                WHERE campaign_type = %s
                  AND campaign = %s
                  AND active = true
                """,
                (
                    stopped_at,
                    stopped_at,
                    "NO_DURABLE_PRODUCER_CAMPAIGN_BOUNDARY",
                    CAMPAIGN_TYPE_TEMPEST,
                    campaign_name,
                ),
            )
            if cur.rowcount != 1:
                raise RuntimeError(
                    "zero-row TEMPEST adoption did not retire exactly one active master"
                )
        _campaign_active = False
        _accepted_pps_vclock_count = None
        _clear_sync_wait()
        _clear_start_wait_state()
        _clear_flash_cut_wait_state()
        result = {
            "restored": False,
            "mode": "ZERO_ROW_CAMPAIGN_NOT_REPLAYED",
            "campaign": campaign_name,
            "producer_mutated": False,
            "reason": "no durable producer-authored campaign boundary",
            "snapshot_source_detail_id": (
                int(combined_snapshot.source_detail_id)
                if combined_snapshot is not None
                else None
            ),
            "snapshot_campaign": snapshot_campaign_name or None,
            "snapshot_campaign_state": snapshot_campaign_state or None,
            "snapshot_public_count": int(snapshot_public_count),
        }
        _diag["last_recovery"] = {
            "ts_utc": stopped_at,
            **copy.deepcopy(result),
        }
        logging.warning(
            "🧾 [recovery] active TEMPEST '%s' has no running producer-authored "
            "boundary in the latest CLOCKS_RECOVERY snapshot; retiring stale Pi "
            "intent instead of replaying START",
            campaign_name,
        )
        return result

    assert combined_snapshot is not None
    combined_recovery_detail = combined_snapshot.restore_detail()
    last_tb = _recovery_timebase_from_clocks_snapshot(
        combined_snapshot, campaign_name
    )
    recovery_snapshot = _recovery_timebase_snapshot(last_tb)
    skipped_unrecoverable_rows = 0

    last_frag = recovery_snapshot["last_frag"]
    last_pps_vclock_count = int(recovery_snapshot["last_pps_vclock_count"])
    last_gnss_ns = int(recovery_snapshot["last_gnss_ns"])
    legacy_last_dwt_ns = int(recovery_snapshot["legacy_last_dwt_ns"])
    last_dwt_cycles = int(recovery_snapshot["last_dwt_cycles"])
    last_dwt_ns = int(recovery_snapshot["last_dwt_ns"])
    last_ocxo1_ns = int(recovery_snapshot["last_ocxo1_ns"])
    last_ocxo2_ns = int(recovery_snapshot["last_ocxo2_ns"])
    last_gnss_raw_ns = int(recovery_snapshot["last_gnss_raw_ns"])
    last_gnss_raw_ref_ns = int(
        recovery_snapshot.get("last_gnss_raw_ref_ns")
        or (last_pps_vclock_count * NS_PER_SECOND)
    )
    last_gnss_raw_welford_mean = float(
        recovery_snapshot.get("last_gnss_raw_welford_mean") or 0.0
    )
    last_gnss_time_str = str(recovery_snapshot["last_gnss_time_str"])

    if not recovery_snapshot.get("recoverable"):
        raise RuntimeError(
            "recovery failed: CLOCKS_RECOVERY snapshot campaign state is not "
            "recoverable "
            f"(source_detail_id={combined_snapshot.source_detail_id}, "
            f"pps_vclock_count={last_pps_vclock_count}, gnss_ns={last_gnss_ns}, "
            f"dwt_cycles={last_dwt_cycles}, ocxo1_ns={last_ocxo1_ns}, "
            f"ocxo2_ns={last_ocxo2_ns})"
        )

    if last_gnss_ns == 0:
        logging.warning("⚠️ [recovery] teensy_gnss_ns=0 — using dwt_ns as proxy")
        last_gnss_ns = last_dwt_ns

    if startup_instrument_verdict is None:
        _instrument_probe, startup_instrument_verdict = _restore_instrument_from_clocks(
            combined_recovery_detail
        )

    producer_disposition = str(
        startup_instrument_verdict.alpha_disposition
        if startup_instrument_verdict is not None
        else ""
    ).strip().upper()
    if producer_disposition == "SURVIVED":
        return _adopt_surviving_clocks_producer(
            snapshot_detail=combined_recovery_detail,
            active_campaign=row,
            instrument_verdict=startup_instrument_verdict,
        )
    if producer_disposition != "LOST":
        raise RuntimeError(
            "active TEMPEST dead-producer recovery requires a LOST producer verdict; "
            f"got {producer_disposition or 'NONE'}"
        )

    logging.info(
        "📐 [recovery] durable source is public count=%d at GNSS time %s; "
        "instrument clockfaces GNSS=%d ns DWT=%d cycles OCXO1=%d ns OCXO2=%d ns. "
        "GNSS_RAW mean drift at that boundary was %.6f ppb.",
        last_pps_vclock_count, last_gnss_time_str, last_gnss_ns, last_dwt_cycles,
        last_ocxo1_ns, last_ocxo2_ns, last_gnss_raw_welford_mean,
    )

    # ------------------------------------------------------------------
    # Step 2: Wait for preflight
    # ------------------------------------------------------------------
    # Dead-producer recovery deliberately bypasses the full START CLOCKS profile:
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
    # START and Flash Cut remain preflight-gated.
    _diag["last_preflight_wait"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "context": "recovery",
        "status": "BYPASSED",
        "checks": 0,
        "waited_s": 0.0,
        "bypass_reason": "warm_recovery_after_teensy_reboot",
    }
    logging.info(
        "%s bypassing START admission gate for dead-producer restore; "
        "CLOCKS.RECOVER firmware verdict remains authoritative",
        PREFLIGHT_LOG_PREFIX,
    )

    _wait_for_timebase_routes(context="recovery")

    # ------------------------------------------------------------------
    # Quiesce Pi ingress without issuing an operator STOP
    # ------------------------------------------------------------------
    #
    # Surviving producers have already exited through the read-only adoption
    # branch.  CLOCKS.RECOVER is therefore used here only for dead-producer restore:
    # install a fresh SmartZero-backed producer epoch mapped onto durable
    # campaign coordinates.  No STOP/START or campaign-only recovery is issued.
    logging.info(
        "📡 [recovery] @%s preserving durable campaign identity; "
        "quiescing Pi CLOCKS_FRAGMENT campaign ingress before one dead-producer restore...",
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
    # Step 3-5: Pure snapshot -> desired-state projection
    # ------------------------------------------------------------------
    current_gnss_time_str = _wait_for_gnss_time()
    logging.info("📐 [recovery] current GNSS time: %s", current_gnss_time_str)
    current_gnss_utc = datetime.fromisoformat(
        current_gnss_time_str.replace("Z", "+00:00")
    )
    try:
        desired_state = _project_clocks_recovery_snapshot(
            recovery_snapshot, current_gnss_utc
        )
    except ValueError as exc:
        if "positive elapsed GNSS time" in str(exc):
            _diag["recovery_elapsed_seconds_nonpositive"] += 1
        raise RuntimeError(f"recovery projection failed: {exc}") from exc

    elapsed_seconds = int(desired_state["elapsed_seconds"])
    recover_base_pps_vclock_count = int(
        desired_state["recover_base_pps_vclock_count"]
    )
    expected_first_public_pps_vclock_count = int(
        desired_state["expected_first_public_pps_vclock_count"]
    )
    restore_clockfaces = desired_state["restore_clockfaces"]
    first_public_clockfaces = desired_state["expected_first_public_clockfaces"]
    tau = desired_state["tau"]
    projected_gnss_ns = int(restore_clockfaces["gnss_ns"])
    projected_dwt_cycles = int(restore_clockfaces["dwt_cycles"])
    projected_dwt_ns = int(restore_clockfaces["dwt_ns"])
    projected_ocxo1_ns = int(restore_clockfaces["ocxo1_ns"])
    projected_ocxo2_ns = int(restore_clockfaces["ocxo2_ns"])
    expected_first_public_gnss_ns = int(first_public_clockfaces["gnss_ns"])
    expected_first_public_ocxo1_ns = int(first_public_clockfaces["ocxo1_ns"])
    expected_first_public_ocxo2_ns = int(first_public_clockfaces["ocxo2_ns"])
    tau_dwt = float(tau["dwt"])
    tau_ocxo1 = float(tau["ocxo1"])
    tau_ocxo2 = float(tau["ocxo2"])

    logging.info(
        "📐 [recovery] GNSS says %d second(s) elapsed since the durable boundary (%s -> %s), "
        "so campaign count %d projects to restore base=%d and the first resumed public row must be %d.",
        elapsed_seconds, last_gnss_time_str, current_gnss_time_str, last_pps_vclock_count,
        recover_base_pps_vclock_count, expected_first_public_pps_vclock_count,
    )

    recovery_source_db_id = int(combined_snapshot.source_detail_id)
    _diag["last_recovery"] = {
        "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "campaign": campaign_name,
        "recovery_source_db_id": recovery_source_db_id,
        "recovery_authority": "CLOCKS_RECOVERY_SNAPSHOT",
        "campaign_adjudication_authority": "HISTORY_ONLY_NOT_REQUIRED",
        "desired_state_schema": desired_state.get("schema"),
        "last_pps_vclock_count": int(last_pps_vclock_count),
        "last_gnss_time": last_gnss_time_str,
        "current_gnss_time": current_gnss_time_str,
        "elapsed_seconds": int(elapsed_seconds),
        "recover_base_pps_vclock_count": int(recover_base_pps_vclock_count),
        "expected_first_public_pps_vclock_count": int(expected_first_public_pps_vclock_count),
        "expected_first_public_gnss_ns": int(expected_first_public_gnss_ns),
        "expected_first_public_ocxo1_ns": int(expected_first_public_ocxo1_ns),
        "expected_first_public_ocxo2_ns": int(expected_first_public_ocxo2_ns),
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
        **copy.deepcopy(desired_state["teensy_recover_args"]),
    }

    epoch_ready_before_recover, recovery_status_before = _probe_teensy_recovery_epoch()
    physical_reboot_custody = _recovery_custody_requires_cold_restore()
    reported_campaign_name = str(
        recovery_status_before.get("campaign") or ""
    ).strip()
    reported_campaign_state = str(
        recovery_status_before.get("campaign_state") or ""
    ).strip().upper()

    # Reaching this point means producer continuity was not proved. RECOVER now
    # has one meaning on both machines: install this complete dead-producer
    # desired state and prove its first successor.
    _diag["last_recovery"].update({
        "teensy_epoch_ready_before_recover": bool(epoch_ready_before_recover),
        "teensy_campaign_before_recover": reported_campaign_name or None,
        "teensy_campaign_state_before_recover": reported_campaign_state or None,
        "physical_reboot_custody": bool(physical_reboot_custody),
        "producer_disposition": producer_disposition,
        "producer_restore_required": True,
    })

    combined_startup_custody: Optional[Dict[str, Any]] = None
    combined_pi_control: Optional[Dict[str, Any]] = None
    combined_alpha_proof: Optional[Dict[str, Any]] = None
    combined_restore_requested_monotonic: Optional[float] = None

    combined_source_detail_id = int(combined_snapshot.source_detail_id)
    loss_source_detail_id = _as_int(
        (startup_instrument_verdict.alpha_proof or {}).get("source_detail_id")
        if startup_instrument_verdict is not None
        else None
    )
    if loss_source_detail_id != combined_source_detail_id:
        raise RuntimeError(
            "dead-producer recovery authority identity mismatch: "
            f"loss_source={loss_source_detail_id} "
            f"snapshot_source={combined_source_detail_id}"
        )

    combined_recovery_clocks = _clocks_payload(combined_recovery_detail)

    with _clocks_state_mutation_gate_lock:
        _startup_instrument_restore_hold.set()
        _startup_physical_lifetime_unclassified.clear()
        combined_startup_custody = _retire_startup_clocks_custody(
            "combined_dead_producer_restore"
        )
        combined_pi_control = _dac_restore_control_from_clocks(
            combined_recovery_clocks, realize=True
        )
    _seed_clocks_from_detail(combined_recovery_detail)
    _arm_holistic_restore_persistence_proof(combined_recovery_detail)

    cold_ppb_checkpoint = _require_alpha_resurrection_checkpoint(
        combined_recovery_clocks,
        campaign=campaign_name,
        recovery_source_db_id=recovery_source_db_id,
    )
    _restore_ppb_checkpoint_runtime(
        cold_ppb_checkpoint,
        source_db_detail_id=combined_source_detail_id,
    )
    cold_ppb_stage = _stage_teensy_better_buckets_checkpoint(
        cold_ppb_checkpoint
    )
    teensy_recover_args.update(
        _canonical_restore_args(combined_recovery_clocks, include_control=False)
    )
    _diag["last_recovery"].update({
        "canonical_restore_present": True,
        "restore_schema_version": 4,
        "better_buckets_restore": cold_ppb_stage,
    })
    logging.info(
        "♻️ [recovery] dead producer + TEMPEST '%s' prepared for one "
        "dead-producer desired-state transaction from detail_id=%s "
        "second=%d minute=%d source_update=%d",
        campaign_name,
        combined_source_detail_id,
        len(cold_ppb_checkpoint.get("second_history") or []),
        len(cold_ppb_checkpoint.get("minute_history") or []),
        int(cold_ppb_checkpoint.get("update_count") or 0),
    )

    try:
        combined_restore_requested_monotonic = time.monotonic()
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
        teensy_recover_payload.get("recover_mode") or ""
    ).strip().upper()
    if recover_mode != "DEAD_PRODUCER_RESTORE":
        raise RecoveryRetryableFailure(
            "dead_producer_restore_contract_mismatch",
            {
                "campaign": campaign_name,
                "recover_mode": recover_mode or None,
                "teensy_recover_payload": teensy_recover_payload,
            },
        )
    first_row_timeout_s = RECOVERY_DEAD_PRODUCER_FIRST_ROW_TIMEOUT_S

    if combined_restore_requested_monotonic is None:
        raise RuntimeError("combined CLOCKS restore has no command timestamp")
    if combined_recovery_detail is None:
        raise RuntimeError(
            "combined dead-producer restore lost its formal recovery snapshot"
        )
    combined_alpha_proof = _wait_for_holistic_restore(
        combined_recovery_detail,
        requested_monotonic=combined_restore_requested_monotonic,
        timeout_s=float(first_row_timeout_s),
    )
    logging.info(
        "✅ [recovery] dead-producer exact N+1 proof durable; the same "
        "DEAD_PRODUCER_RESTORE transaction is awaiting its projected TEMPEST first-public row"
    )

    _diag["last_recovery"].update({
        "recover_mode": recover_mode,
        "dead_producer_restore": True,
        "producer_restore_required": True,
        "combined_dead_producer_restore": True,
        "combined_alpha_proof": copy.deepcopy(combined_alpha_proof),
        "recover_dead_producer_restore_active": bool(
            teensy_recover_payload.get("recover_dead_producer_restore_active")
        ),
        "recover_dead_producer_restore_epoch_ready": bool(
            teensy_recover_payload.get("recover_dead_producer_restore_epoch_ready")
        ),
        "first_row_timeout_s": float(first_row_timeout_s),
    })

    logging.info(
        "🧭 [recovery] Teensy accepted dead-producer restore: preserving campaign "
        "base=%d while installing the projected desired state",
        recover_base_pps_vclock_count,
    )

    recovery_monitor = {
        "campaign": campaign_name,
        "recover_base_pps_vclock_count": int(recover_base_pps_vclock_count),
        "expected_first_public_pps_vclock_count": int(expected_first_public_pps_vclock_count),
        "recovery_generation": _as_int(teensy_recover_payload.get("recovery_generation")),
        "recover_status": teensy_recover_payload.get("status"),
        "recover_mode": recover_mode,
        "dead_producer_restore": True,
        "producer_restore_required": True,
        "recover_dead_producer_restore_active": bool(
            teensy_recover_payload.get("recover_dead_producer_restore_active")
        ),
        "recover_dead_producer_restore_epoch_ready": bool(
            teensy_recover_payload.get("recover_dead_producer_restore_epoch_ready")
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
                "dead_producer_restore": True,
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
                "dead_producer_restore": True,
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
                # Expected recovery proof search stays quiet.  The row is still
                # persisted and the complete admission verdict remains in diagnostics;
                # firmware emits CLOCKS_RECOVERY_STALLED if proof exceeds its bounded
                # attempt threshold.
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
            ",".join(recovery_admission_verdict.get("blocking_reasons") or []) or "none",
            ",".join(recovery_admission_verdict.get("state_reasons") or []) or "none",
            recovery_admission_verdict.get("report_reason") or "none",
        )

        # Release the processor thread to discard this row while campaign
        # ingestion is still closed, then arm the next accepted-row sync wait.
        _sync_resume_event.set()
        time.sleep(0.05)
        _begin_sync_wait(expected_pps=int(teensy_pps_vclock_count) + 1)

    if recovery_source_db_id is None or recovery_source_db_id <= 0:
        raise RecoveryRetryableFailure(
            "dead_producer_restore_missing_recovery_source_db_id",
            {"campaign": campaign_name, "last_timebase": last_tb},
        )
    dead_producer_restore_supersede = _supersede_dead_producer_restore_rows(
        base_detail_id=int(recovery_source_db_id),
        campaign_name=campaign_name,
        first_public_count=int(teensy_pps_vclock_count),
    )
    _diag["last_recovery"]["dead_producer_restore_supersede"] = (
        dead_producer_restore_supersede
    )
    logging.info(
        "🧹 [recovery] dead-producer evidence boundary: kept source id=%d, "
        "recovered id=%d, marked %d intervening row(s) non-viable",
        int(dead_producer_restore_supersede["base_detail_id"]),
        int(dead_producer_restore_supersede["recovered_detail_id"]),
        int(dead_producer_restore_supersede["rows_marked_nonviable"]),
    )

    recovery_custody = _finalize_recovery_clocks_custody(
        last_tb=last_tb,
        campaign_name=campaign_name,
        first_public_count=int(teensy_pps_vclock_count),
        recover_mode=recover_mode,
    )
    _diag["last_recovery"]["recovery_custody"] = recovery_custody

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

    alpha_proof_sequence = _as_int(combined_alpha_proof.get("durable_proof_sequence"))
    source_stats = combined_recovery_clocks.get("stats")
    source_reset_count = (
        _as_int(source_stats.get("reset_count")) if isinstance(source_stats, dict) else None
    )
    source_update_count = (
        _as_int(source_stats.get("update_count")) if isinstance(source_stats, dict) else None
    )
    if (
        alpha_proof_sequence is None
        or source_reset_count is None
        or source_update_count is None
    ):
        raise RuntimeError("dead-producer CLOCKS receipt lacks exact Alpha proof chronology")
    alpha_boundary_detail_id = _find_clocks_exact_recovery_boundary_detail_id(
        source_detail_id=int(combined_source_detail_id),
        source_reset_count=int(source_reset_count),
        source_update_count=int(source_update_count),
        proof_sequence=int(alpha_proof_sequence),
    )
    recovery_receipt = _record_clocks_recovery_receipt(
        boundary_detail_id=int(dead_producer_restore_supersede["recovered_detail_id"]),
        source_detail=combined_recovery_detail,
        source_checkpoint=cold_ppb_checkpoint,
        recovery_mode=recover_mode,
        proof=combined_alpha_proof,
        alpha_proof_detail_id=alpha_boundary_detail_id,
        campaign_name=campaign_name,
        source_public_count=int(last_pps_vclock_count),
        expected_first_public_count=int(expected_first_public_pps_vclock_count),
        elapsed_seconds=int(elapsed_seconds),
    )
    _diag["last_recovery"]["recovery_receipt"] = copy.deepcopy(recovery_receipt)

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
    combined_instrument_restore = {
        "success": True,
        "mode": "COMBINED_PRODUCER_CAMPAIGN_RESTORE",
        "producer_resurrected_this_startup": True,
        "firmware_recover_mode": recover_mode,
        "pi_control": copy.deepcopy(combined_pi_control),
        "startup_custody": copy.deepcopy(combined_startup_custody),
        "better_buckets": copy.deepcopy(cold_ppb_stage),
        "proof": copy.deepcopy(combined_alpha_proof),
    }
    # Exact producer N+1 and projected TEMPEST first-public have both closed.
    _clocks_persistence_enabled.set()
    _clocks_holistic_restore_proof_pending.clear()

    return {
        "restored": True,
        "mode": "DEAD_PRODUCER_RESTORE",
        "firmware_recover_mode": recover_mode,
        "combined_dead_producer_restore": True,
        "campaign": campaign_name,
        "first_public_pps_vclock_count": int(teensy_pps_vclock_count),
        "science_clean": bool(recovery_admission_verdict.get("fully_clean")),
        "combined_instrument_restore": combined_instrument_restore,
        "recovery_receipt": copy.deepcopy(recovery_receipt),
    }



# ---------------------------------------------------------------------
# BASELINE — campaign-to-campaign relationship
# ---------------------------------------------------------------------


def _baseline_relation_for_active_campaign() -> Optional[Dict[str, Any]]:
    """Return the active campaign and its referenced baseline campaign, if any."""
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT
                current.campaign AS campaign,
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
            (CAMPAIGN_TYPE_TEMPEST,),
        )
        row = cur.fetchone()

    if row is None:
        return None

    payload = row["baseline_payload"]
    if isinstance(payload, str):
        payload = json.loads(payload)
    if not isinstance(payload, dict):
        raise RuntimeError("baseline campaign payload is not an object")

    report = payload.get("report")
    if not isinstance(report, dict) or not report:
        raise RuntimeError(
            f"Baseline campaign '{row['baseline_campaign']}' has no report"
        )

    return {
        "campaign": row["campaign"],
        "baseline_campaign": row["baseline_campaign"],
        "baseline_report": report,
        "baseline_location": payload.get("location"),
        "baseline_started_at": payload.get("started_at"),
    }


def _remove_legacy_baseline_config(cur) -> None:
    """Remove retired copied-baseline fields while preserving other SYSTEM config."""
    cur.execute(
        """
        UPDATE config
        SET payload = payload
            - 'baseline_id'
            - 'baseline_ppb'
            - 'baseline_tau'
            - 'baseline_dac'
            - 'baseline_dac_mean'
            - 'baseline_dac_stats'
            - 'baseline_campaign_type'
            - 'baseline_campaign'
            - 'baseline_pps_vclock_n'
            - 'baseline_pps_n'
        WHERE config_key = 'SYSTEM'
        """
    )


def cmd_set_baseline(args: Optional[dict]) -> Dict[str, Any]:
    """Relate the active campaign to another campaign selected by name."""
    if not args or not str(args.get("campaign") or "").strip():
        return {"success": False, "message": "SET_BASELINE requires 'campaign' argument"}

    baseline_name = str(args["campaign"]).strip()

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
                (CAMPAIGN_TYPE_TEMPEST,),
            )
            current = cur.fetchone()
            if current is None:
                return {
                    "success": False,
                    "message": "SET_BASELINE requires an active campaign",
                }

            cur.execute(
                """
                SELECT id, campaign, payload
                FROM campaign_master
                WHERE campaign_type = %s
                  AND campaign = %s
                ORDER BY ts DESC, id DESC
                LIMIT 1
                """,
                (CAMPAIGN_TYPE_TEMPEST, baseline_name),
            )
            baseline = cur.fetchone()
            if baseline is None:
                return {
                    "success": False,
                    "message": f"No campaign named '{baseline_name}'",
                }

            if int(baseline["id"]) == int(current["id"]):
                return {
                    "success": False,
                    "message": "A campaign cannot use itself as its baseline",
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
                    "message": f"Campaign '{baseline['campaign']}' has no report",
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
                """,
                (int(baseline["id"]), int(current["id"])),
            )
            if cur.rowcount != 1:
                raise RuntimeError("active campaign baseline relationship was not updated")

            _remove_legacy_baseline_config(cur)

    except Exception as exc:
        logging.exception("❌ [clocks] failed to establish campaign baseline relationship")
        return {"success": False, "message": str(exc)}

    logging.info(
        "✅ [clocks] campaign '%s' baseline -> '%s'",
        current["campaign"],
        baseline["campaign"],
    )
    return {
        "success": True,
        "message": "OK",
        "payload": {
            "campaign": current["campaign"],
            "baseline_campaign": baseline["campaign"],
        },
    }


def cmd_baseline_info(_: Optional[dict]) -> Dict[str, Any]:
    """Return the active campaign's baseline relationship and referenced report."""
    try:
        relation = _baseline_relation_for_active_campaign()
    except Exception as exc:
        logging.exception("❌ [clocks] BASELINE_INFO failed")
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
        "payload": {
            "baseline_set": True,
            **relation,
        },
    }





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


def _wait_for_startup_alpha_epoch() -> Dict[str, Any]:
    """Wait for the current Teensy lifetime to earn its first Alpha epoch.

    Cold power-up may spend substantial time in SmartZero while GNSS, OCXOs,
    and their one-second repeatability evidence mature.  During that interval
    Alpha intentionally cannot author a canonical completed CLOCKS row.  Do not
    spend HOLISTIC_RESTORE_TIMEOUT_S against a proof that firmware is not yet
    physically capable of producing.

    REPORT_RECOVERY is the non-mutating lifecycle authority for this boundary.
    There is deliberately no fixed elapsed-time failure here: startup remains
    visibly INITIALIZING while physics progresses.  Once recover_epoch_ready is
    true, the ordinary CLOCKS preflight must still prove a fresh canonical row,
    FINE_LOCK, receiver mode, and chrony PPS before holistic restore begins.
    """
    t0 = time.monotonic()
    last_log_at = t0
    last_signature: Optional[Tuple[Any, ...]] = None
    smartzero_complete_announced = False
    checks = 0

    logging.info(
        "⏳ [clocks/startup] beginning SmartZero-backed Alpha epoch wait; "
        "firmware remains authoritative and Pi will report progress every %.0fs",
        STARTUP_ALPHA_EPOCH_STATUS_LOG_INTERVAL_S,
    )

    while True:
        checks += 1
        status = _fetch_teensy_recovery_status()
        now = time.monotonic()
        elapsed = now - t0

        recover_epoch_ready = _recovery_bool(status.get("recover_epoch_ready"))
        smartzero_running = _recovery_bool(status.get("recover_smartzero_running"))
        smartzero_complete = _recovery_bool(status.get("recover_smartzero_complete"))
        restore_court_ready = _recovery_bool(status.get("restore_court_ready"))
        firmware_campaign_state = str(
            status.get("campaign_state") or ""
        ).strip().upper()

        diag = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "status": "NOMINAL" if recover_epoch_ready else "WAITING",
            "checks": int(checks),
            "waited_s": round(float(elapsed), 3),
            "report_available": bool(status),
            "recover_epoch_ready": bool(recover_epoch_ready),
            "recover_smartzero_running": bool(smartzero_running),
            "recover_smartzero_complete": bool(smartzero_complete),
            "restore_court_ready": bool(restore_court_ready),
            "firmware_campaign_state": firmware_campaign_state or None,
        }
        _diag["last_startup_alpha_epoch_wait"] = diag

        if recover_epoch_ready:
            logging.info(
                "✅ [clocks/startup] SmartZero-backed Alpha epoch installed after %.1fs; "
                "continuing to full CLOCKS/GNSS/chrony preflight",
                elapsed,
            )
            return copy.deepcopy(diag)

        if smartzero_complete and not smartzero_complete_announced:
            logging.info(
                "✅ [clocks/startup] SmartZero proof achieved after %.1fs; "
                "waiting only for Alpha epoch installation",
                elapsed,
            )
            smartzero_complete_announced = True

        signature = (
            bool(status),
            bool(smartzero_running),
            bool(smartzero_complete),
            bool(restore_court_ready),
            firmware_campaign_state,
        )
        should_log = (
            signature != last_signature
            or now - last_log_at >= STARTUP_ALPHA_EPOCH_STATUS_LOG_INTERVAL_S
        )
        if should_log:
            smartzero_progress: Dict[str, Any] = {}
            if smartzero_running or smartzero_complete:
                smartzero_progress = _smartzero_progress_snapshot(
                    _fetch_teensy_smartzero_status()
                )
                if smartzero_progress:
                    diag["smartzero"] = copy.deepcopy(smartzero_progress)
                    _diag["last_startup_alpha_epoch_wait"] = copy.deepcopy(diag)

            if not status:
                detail = "REPORT_RECOVERY unavailable"
            elif smartzero_running:
                detail = _smartzero_progress_text(smartzero_progress)
            elif smartzero_complete:
                lane_detail = _smartzero_progress_text(smartzero_progress)
                detail = f"SmartZero complete; Alpha epoch install pending; {lane_detail}"
            else:
                detail = "Alpha epoch not ready"
            logging.info(
                "⏳ [clocks/startup] SmartZero/Alpha epoch still pending (%.1fs): %s",
                elapsed,
                detail,
            )
            last_log_at = now
            last_signature = signature

        time.sleep(STARTUP_ALPHA_EPOCH_POLL_S)


def _startup_restore_court_ready() -> Optional[Dict[str, Any]]:
    """Return explicit firmware testimony allowing startup to enter restore early.

    This is an instrument-lifecycle court, not a TEMPEST-only court. A newborn
    Teensy may require canonical CLOCKS resurrection even when no campaign is
    active. Admission still requires the current lifetime to have earned its own
    Alpha epoch and for firmware to say RESTORE_MONITOR may lawfully adjudicate
    the stopped instrument. The caller remains responsible for external GNSS/chrony
    readiness before using this court to bypass an absent canonical CLOCKS row.
    """
    active_campaign = _get_active_campaign()

    status = _fetch_teensy_recovery_status()
    if not status:
        return None

    firmware_campaign = str(status.get("campaign") or "").strip()
    firmware_state = str(status.get("campaign_state") or "").strip().upper()
    restore_court_ready = _recovery_bool(status.get("restore_court_ready"))
    recover_epoch_ready = _recovery_bool(status.get("recover_epoch_ready"))
    recovery_active = _recovery_bool(status.get("recover_lifecycle_active"))

    if (
        not firmware_campaign
        and firmware_state in {"", "STOPPED", "IDLE", "NONE"}
        and restore_court_ready
        and recover_epoch_ready
        and not recovery_active
    ):
        return {
            "basis": "TEENSY_REPORT_RECOVERY_RESTORE_COURT_READY",
            "pi_active_campaign": (
                str(active_campaign.get("campaign") or "")
                if isinstance(active_campaign, dict)
                else None
            ),
            "firmware_campaign": None,
            "firmware_campaign_state": firmware_state or None,
            "restore_court_ready": True,
            "recover_epoch_ready": True,
        }
    return None


def _preflight_reason_is_expected_clocks_absence(reason: str) -> bool:
    """True only for absence/staleness of the canonical CLOCKS heartbeat."""
    prefixes = (
        f"{FEATURE_PREFLIGHT_PROFILE}: CLOCKS feature tree unavailable "
        "(CLOCKS heartbeat not yet received",
        f"{FEATURE_PREFLIGHT_PROFILE}: CLOCKS feature tree unavailable "
        "(CLOCKS heartbeat stale",
    )
    return any(str(reason).startswith(prefix) for prefix in prefixes)


def _preflight_wait_is_expected_clocks_absence(pending: list[str]) -> bool:
    """True when startup is waiting only for the Teensy CLOCKS producer to exist."""
    return bool(pending) and all(
        _preflight_reason_is_expected_clocks_absence(item) for item in pending
    )


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


def _startup_restore_court_direct_gnss_reasons(
    context: str,
    *,
    required_gnss_freq_mode: Optional[int],
    required_gnss_freq_mode_name: Optional[str],
    required_receiver_mode: Optional[str],
) -> list[str]:
    """Re-prove GNSS readiness directly when canonical CLOCKS cannot yet exist.

    The ordinary preflight deliberately trusts GNSS decoration carried by canonical
    CLOCKS. During a newborn-instrument restore circularity that row may be exactly
    what cannot exist yet. REPORT_RECOVERY may open the restore court, but it does
    not replace external receiver testimony. Re-read SYSTEM.REPORT here and apply
    the same GNSS/receiver requirements before allowing the CLOCKS-heartbeat-only
    escape hatch. Chrony remains checked by _check_preflight().
    """
    reasons: list[str] = []
    try:
        system_context = _fetch_system_report()
        gnss = _system_gnss_info(system_context)
        location = (
            system_context.get("location")
            if isinstance(system_context.get("location"), dict)
            else {}
        )
        if not gnss:
            return ["restore-court SYSTEM.REPORT GNSS context unavailable"]

        if not gnss.get("time_valid"):
            reasons.append("restore-court GNSS time not valid (no satellite time/date)")

        lock_quality = str(gnss.get("lock_quality") or "WEAK").upper()
        if lock_quality == "WEAK":
            reasons.append(
                "restore-court GNSS lock quality is WEAK "
                f"(satellites={gnss.get('satellites', '?')}, hdop={gnss.get('hdop', '?')})"
            )

        if not gnss.get("pps_valid"):
            reasons.append("restore-court GNSS PPS not valid (discipline loop not active)")
        else:
            discipline = gnss.get("discipline", {})
            freq_mode = (
                _as_int(discipline.get("freq_mode"))
                if isinstance(discipline, dict)
                else None
            )
            freq_mode_name = (
                str(discipline.get("freq_mode_name") or "UNKNOWN")
                if isinstance(discipline, dict)
                else "UNKNOWN"
            )
            if required_gnss_freq_mode is None:
                if freq_mode is None or freq_mode < 2:
                    reasons.append(
                        "restore-court GNSS discipline not locked "
                        f"(freq_mode={freq_mode} '{freq_mode_name}', need at least COARSE_LOCK)"
                    )
            elif freq_mode != int(required_gnss_freq_mode):
                required_name = required_gnss_freq_mode_name or str(required_gnss_freq_mode)
                reasons.append(
                    f"restore-court GNSS discipline not ready for {context} "
                    f"(freq_mode={freq_mode} '{freq_mode_name}', need {required_name})"
                )

        if required_receiver_mode is not None:
            receiver_mode = str(location.get("receiver_mode") or "UNKNOWN").upper()
            expected_mode = str(required_receiver_mode).upper()
            if receiver_mode != expected_mode:
                reasons.append(
                    f"restore-court GF-8802 receiver mode is {receiver_mode}; "
                    f"need {expected_mode} for {context}"
                )
    except Exception as exc:
        reasons.append(f"restore-court SYSTEM.REPORT GNSS preflight failed: {exc}")
    return reasons


# ---------------------------------------------------------------------
# Preflight gate — prerequisites for START / Flash Cut / cold recovery
# ---------------------------------------------------------------------


def _check_preflight(
    context: str = "campaign",
    *,
    required_gnss_freq_mode: Optional[int] = None,
    required_gnss_freq_mode_name: Optional[str] = None,
    required_receiver_mode: Optional[str] = None,
) -> tuple[bool, list[str]]:
    """Check the CLOCKS policy gate plus fresh local Pi prerequisites.

    This path is used for cold START, Flash Cut, and zero-row cold recovery.
    Dead-producer recovery has its own narrower lifecycle contract.
    """
    reasons: list[str] = []

    # -----------------------------------------------------------------
    # 0. Single unified CLOCKS readiness profile
    # -----------------------------------------------------------------
    feature_ready, feature_reasons = _check_feature_preflight(context)
    if not feature_ready:
        reasons.extend(feature_reasons)

    # If the canonical CLOCKS heartbeat itself is unavailable or stale, its
    # SYSTEM/GNSS decoration is not current testimony. Do not reinterpret absent
    # decoration as physical GF-8802 weakness; the feature-tree reason above is
    # the complete blocker until a fresh producer row exists.
    canonical_clocks_available = not any(
        reason.startswith(
            f"{FEATURE_PREFLIGHT_PROFILE}: CLOCKS feature tree unavailable"
        )
        for reason in feature_reasons
    )

    # -----------------------------------------------------------------
    # 1. GNSS state carried by the canonical CLOCKS snapshot
    # -----------------------------------------------------------------
    if canonical_clocks_available:
        try:
            with _clocks_lock:
                latest = copy.deepcopy(_latest_clocks)
            gnss = latest.get("gnss") if isinstance(latest.get("gnss"), dict) else {}
            if not gnss:
                reasons.append(
                    "canonical CLOCKS live GNSS telemetry not yet populated from SYSTEM.REPORT"
                )
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
                    freq_mode = (
                        _as_int(discipline.get("freq_mode"))
                        if isinstance(discipline, dict)
                        else None
                    )
                    freq_mode_name = (
                        str(discipline.get("freq_mode_name") or "UNKNOWN")
                        if isinstance(discipline, dict)
                        else "UNKNOWN"
                    )
                    if required_gnss_freq_mode is None:
                        if freq_mode is None or freq_mode < 2:
                            reasons.append(
                                f"GNSS discipline not locked "
                                f"(freq_mode={freq_mode} '{freq_mode_name}', need at least COARSE_LOCK)"
                            )
                    elif freq_mode != int(required_gnss_freq_mode):
                        required_name = required_gnss_freq_mode_name or str(required_gnss_freq_mode)
                        reasons.append(
                            f"GNSS discipline not ready for {context} "
                            f"(freq_mode={freq_mode} '{freq_mode_name}', need {required_name})"
                        )

                if required_receiver_mode is not None:
                    location = (
                        latest.get("location")
                        if isinstance(latest.get("location"), dict)
                        else {}
                    )
                    receiver_mode = str(location.get("receiver_mode") or "UNKNOWN").upper()
                    expected_mode = str(required_receiver_mode).upper()
                    if receiver_mode != expected_mode:
                        reasons.append(
                            f"GF-8802 receiver mode is {receiver_mode}; "
                            f"need {expected_mode} for {context}"
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

def _wait_for_preflight(
    context: str = "recovery",
    *,
    required_gnss_freq_mode: Optional[int] = None,
    required_gnss_freq_mode_name: Optional[str] = None,
    required_receiver_mode: Optional[str] = None,
    allow_restore_court_entry: bool = False,
) -> None:
    """Wait quietly until the unified CLOCKS readiness profile is open.

    Readiness is polled frequently so startup proceeds promptly. The log is
    intentionally sparse: no success line when the gate is immediately open.
    A post-flash startup may enter the existing holistic restore court earlier
    when firmware explicitly proves that the durable active campaign no longer
    exists on Teensy and the recovery epoch is ready.

    When the only missing prerequisite is the canonical CLOCKS heartbeat, keep
    readiness stream-owned but restore the historical SmartZero operator view.
    After one 10-second visibility grace, issue only read-only diagnostic reports
    at the existing SmartZero status cadence; their contents never participate in
    preflight admission or producer-lifetime classification.
    """
    attempt = 0
    t0 = time.monotonic()
    last_log_at = t0
    last_signature: Optional[Tuple[str, ...]] = None
    logged_wait = False
    smartzero_diag_active = False

    while True:
        restore_entry = (
            _startup_restore_court_ready() if allow_restore_court_entry else None
        )
        ready, reasons = _check_preflight(
            context,
            required_gnss_freq_mode=required_gnss_freq_mode,
            required_gnss_freq_mode_name=required_gnss_freq_mode_name,
            required_receiver_mode=required_receiver_mode,
        )
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
            if smartzero_diag_active:
                final_progress = _smartzero_progress_snapshot(
                    _fetch_teensy_smartzero_status()
                )
                if final_progress:
                    logging.info(
                        "✅ [clocks/startup] SmartZero/Alpha startup boundary visible "
                        "after %.1fs: %s",
                        elapsed,
                        _smartzero_progress_text(final_progress),
                    )
            if logged_wait:
                logging.info(
                    "%s %s admitted after %.1fs: %s features NOMINAL; "
                    "CLOCKS heartbeat fresh; SYSTEM.REPORT live GNSS telemetry ready; "
                    "chrony PPS selected",
                    PREFLIGHT_LOG_PREFIX,
                    context,
                    elapsed,
                    FEATURE_PREFLIGHT_PROFILE,
                )
            return

        if restore_entry is not None:
            # The restore court may replace only the missing canonical CLOCKS
            # heartbeat/profile witness. Every independently checkable prerequisite
            # must still be nominal. In particular, _check_preflight() continues to
            # enforce chrony PPS, while this direct SYSTEM.REPORT pass re-proves the
            # GNSS facts that _check_preflight() intentionally cannot infer when
            # canonical CLOCKS is absent/stale.
            non_clocks_reasons = [
                reason
                for reason in reasons
                if not _preflight_reason_is_expected_clocks_absence(reason)
            ]
            direct_gnss_reasons = _startup_restore_court_direct_gnss_reasons(
                context,
                required_gnss_freq_mode=required_gnss_freq_mode,
                required_gnss_freq_mode_name=required_gnss_freq_mode_name,
                required_receiver_mode=required_receiver_mode,
            )
            if not non_clocks_reasons and not direct_gnss_reasons:
                _diag["last_preflight_wait"] = {
                    "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                    "context": context,
                    "status": "RESTORE_COURT",
                    "checks": int(attempt),
                    "waited_s": round(float(elapsed), 3),
                    "restore_entry": copy.deepcopy(restore_entry),
                    "bypassed_only": "CANONICAL_CLOCKS_HEARTBEAT_PROFILE",
                    "direct_system_gnss_ready": True,
                    "chrony_pps_ready": True,
                }
                logging.info(
                    "%s %s admitted to holistic restore court after %.1fs: "
                    "firmware epoch/court ready; direct SYSTEM GNSS and chrony PPS "
                    "nominal; canonical CLOCKS heartbeat is the only missing witness",
                    PREFLIGHT_LOG_PREFIX,
                    context,
                    elapsed,
                )
                return
            reasons = list(reasons) + direct_gnss_reasons

        attempt += 1
        pending = _preflight_wait_items(reasons)
        signature = tuple(pending)
        expected_clocks_absence = _preflight_wait_is_expected_clocks_absence(pending)
        smartzero_log_due = (
            context in {"startup", "holistic_startup"}
            and expected_clocks_absence
            and elapsed >= STARTUP_ALPHA_EPOCH_STATUS_LOG_INTERVAL_S
            and (
                not smartzero_diag_active
                or now - last_log_at >= STARTUP_ALPHA_EPOCH_STATUS_LOG_INTERVAL_S
            )
        )
        should_log = (
            not expected_clocks_absence
            and elapsed >= PREFLIGHT_QUIET_GRACE_S
            and (
                not logged_wait
                or signature != last_signature
                or now - last_log_at >= PREFLIGHT_STATUS_LOG_INTERVAL_S
            )
        )

        _diag["last_preflight_wait"] = {
            "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "context": context,
            "status": "WAITING",
            "checks": int(attempt),
            "waited_s": round(float(elapsed), 3),
            "pending": pending,
            "expected_clocks_absence": bool(expected_clocks_absence),
        }

        if smartzero_log_due:
            # The stream remains the admission authority. These RPCs restore only
            # the operator display that was lost when the blocking startup
            # _wait_for_startup_alpha_epoch() call was removed for live adoption.
            status = _fetch_teensy_recovery_status()
            smartzero_running = _recovery_bool(
                status.get("recover_smartzero_running")
            )
            smartzero_complete = _recovery_bool(
                status.get("recover_smartzero_complete")
            )
            smartzero_progress: Dict[str, Any] = {}
            if smartzero_running or smartzero_complete:
                smartzero_progress = _smartzero_progress_snapshot(
                    _fetch_teensy_smartzero_status()
                )

            if smartzero_running:
                detail = _smartzero_progress_text(smartzero_progress)
            elif smartzero_complete:
                detail = (
                    "SmartZero complete; Alpha epoch/CLOCKS row pending; "
                    f"{_smartzero_progress_text(smartzero_progress)}"
                )
            elif status:
                detail = "Alpha epoch/CLOCKS row pending; SmartZero not currently active"
            else:
                detail = "canonical CLOCKS row absent; SmartZero status unavailable"

            _diag["last_startup_alpha_epoch_wait"] = {
                "ts_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                "status": "WAITING",
                "waited_s": round(float(elapsed), 3),
                "report_available": bool(status),
                "recover_smartzero_running": bool(smartzero_running),
                "recover_smartzero_complete": bool(smartzero_complete),
                "smartzero": copy.deepcopy(smartzero_progress),
                "observational_only": True,
                "preflight_authority": "CLOCKS_FRAGMENT",
            }
            _diag["preflight_wait_log_count"] = (
                _diag.get("preflight_wait_log_count", 0) + 1
            )
            logging.info(
                "⏳ [clocks/startup] SmartZero/Alpha epoch still pending (%.1fs): %s",
                elapsed,
                detail,
            )
            smartzero_diag_active = True
            logged_wait = True
            last_log_at = now
            last_signature = signature

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

    recovery_checkpoint = _read_clocks_recovery_config()
    if recovery_checkpoint is not None:
        recovery_source = _clocks_recovery_source_row(recovery_checkpoint)
        if recovery_source is None:
            return {
                "success": False,
                "message": (
                    f"config.{CLOCKS_RECOVERY_CONFIG_KEY} has no matching durable "
                    "CLOCKS source; refusing campaign deletion"
                ),
            }
        if str(recovery_source.get("campaign") or "") == campaign_name:
            return {
                "success": False,
                "message": (
                    f"Campaign '{campaign_name}' still owns current CLOCKS recovery "
                    "custody — retry after the next ambient CLOCKS row"
                ),
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
                (CAMPAIGN_TYPE_TEMPEST, campaign_name),
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
    """Destructively truncate all campaign history, regardless of subsystem type."""
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
                    "TRUNCATE TABLE campaign_detail, campaign_master RESTART IDENTITY"
                )
                cur.execute(
                    "DELETE FROM config WHERE config_key IN (%s, %s, %s)",
                    (
                        CLOCKS_RECOVERY_CONFIG_KEY,
                        CLOCKS_ALPHA_LINEAGE_CUTOFF_CONFIG_KEY,
                        "PHOTONS_RECOVERY",
                    ),
                )
    except Exception as e:
        logging.exception("❌ [clocks] TRUNCATE failed")
        return {"success": False, "message": str(e)}
    finally:
        if persistence_was_enabled:
            _clocks_persistence_enabled.set()

    logging.warning(
        "🧨 [clocks] TRUNCATE: campaign_detail and campaign_master completely truncated; "
        "identity sequences restarted; drained candidates=%d pending_states=%d",
        candidate_drained, state_drained,
    )

    server_args = {
        "source": "CLOCKS.TRUNCATE",
        "postgres_tables_truncated": ["campaign_detail", "campaign_master"],
        "postgres_identity_restarted": True,
        "ambient_campaign_details_retained": False,
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
            "tables_truncated": ["campaign_detail", "campaign_master"],
            "identity_restarted": True,
            "ambient_campaign_details_retained": False,
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
    """List TEMPEST campaigns with each campaign's baseline relationship by name."""
    try:
        with open_db(row_dict=True) as conn:
            cur = conn.cursor()
            cur.execute(
                """
                SELECT
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

        entry: Dict[str, Any] = {
            "campaign_type": row["campaign_type"],
            "campaign": row["campaign"],
            "active": bool(row["active"]),
            "baseline_campaign": row.get("baseline_campaign"),
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
        "operational_state": _operational_state_snapshot(),
        "integrity_contract": {
            "mode_free": True,
            "coherent_rows_persist": True,
            "science_exclusions_are_audit_only": True,
            "continuity_fatal_triggers_recovery": True,
            "unprovable_continuity_latches_hard_failure": True,
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
    """Set Pi-owned OCXO DAC target(s); Teensy is not involved."""
    if not args:
        return {"success": False, "message": "SET_DAC requires DAC1 and/or DAC2"}
    if "campaign" in args:
        return {
            "success": False,
            "message": (
                "SET_DAC campaign=... has been retired. DAC history remains "
                "available in CLOCKS, while current control is explicit."
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

    ok1 = True
    ok2 = True
    if has_dac1 and dac1 is not None:
        ok1 = _dac_set_target_explicit(_dac_lanes["ocxo1"], dac1)
    if has_dac2 and dac2 is not None:
        ok2 = _dac_set_target_explicit(_dac_lanes["ocxo2"], dac2)

    with _dac_lock:
        lane1 = _dac_lanes["ocxo1"]
        lane2 = _dac_lanes["ocxo2"]
        result = {
            "ocxo1_dac": round(lane1.target_code, 6),
            "ocxo2_dac": round(lane2.target_code, 6),
            "ocxo1_dac_last_write_ok": bool(lane1.last_write_ok),
            "ocxo2_dac_last_write_ok": bool(lane2.last_write_ok),
            "ocxo1_dac_hw_code": int(lane1.hw_code),
            "ocxo2_dac_hw_code": int(lane2.hw_code),
            "ocxo1_dac_voltage": _dac_voltage_from_code(lane1.hw_code),
            "ocxo2_dac_voltage": _dac_voltage_from_code(lane2.hw_code),
            "realization_mode": (
                "ONE_SECOND_FRACTIONAL_DITHER"
                if _dac_dither_operator_enabled
                else "STATIC_ROUNDED"
            ),
            "reference_mode": "INTERNAL_VREF_2X",
            "external_vref_used": False,
            "internal_ref_voltage": AD5693R_INTERNAL_REF_VOLTAGE,
            "output_gain": AD5693R_OUTPUT_GAIN,
            "output_full_scale_voltage": AD5693R_OUTPUT_FULL_SCALE_VOLTAGE,
            "dac_code_scale": AD5693R_CODE_SCALE,
            "safe_max_output_voltage": AD5693R_SAFE_MAX_OUTPUT_VOLTAGE,
            "safe_max_hw_code": AD5693R_SAFE_MAX_HW_CODE,
            "static_rounded_only": not _dac_dither_operator_enabled,
            "fractional_stream_possible": bool(_dac_dither_operator_enabled),
            "recurring_timer_possible": bool(_dac_dither_operator_enabled),
            "dither_operator_enabled": bool(_dac_dither_operator_enabled),
            "owner": "PI.CLOCKS",
            "input_aliases": {
                **({"dac1": alias1} if alias1 else {}),
                **({"dac2": alias2} if alias2 else {}),
            },
            "persistence": "CLOCKS",
            "status": "ok" if ok1 and ok2 else "dac_write_fault",
        }
    logging.info(
        "🔧 [clocks] DAC outputs set by Pi: OCXO1 code=%d (%.4f V), "
        "OCXO2 code=%d (%.4f V), mode=%s status=%s",
        int(result["ocxo1_dac_hw_code"]),
        float(result["ocxo1_dac_voltage"]),
        int(result["ocxo2_dac_hw_code"]),
        float(result["ocxo2_dac_voltage"]),
        result["realization_mode"],
        result["status"],
    )
    return {
        "success": bool(ok1 and ok2),
        "message": "OK" if ok1 and ok2 else "DAC write fault",
        "payload": result,
    }


def cmd_servos(args: Optional[dict]) -> Dict[str, Any]:
    """Set the Pi-owned OCXO servo mode: OFF, TOTAL, CAMP, or 10-MIN."""
    args = args or {}
    _, raw = _arg_first_present(
        args,
        "servos", "SERVOS", "servo", "SERVO", "mode", "MODE", "calibrate_ocxo",
    )
    if raw is None:
        return {
            "success": False,
            "message": "SERVOS requires OFF, TOTAL, CAMP, or 10-MIN",
            "payload": {"servo_mode": _dac_servo_mode},
        }
    try:
        previous, requested = _dac_set_servo_mode(str(raw))
    except ValueError as exc:
        return {"success": False, "message": str(exc), "payload": {"servo_mode": _dac_servo_mode}}
    payload = {
        "status": "servos_updated",
        "previous_mode": previous,
        "requested_mode": requested,
        "servo_mode": requested,
        "servo_target_ppb": float(SERVO_TARGET_PPB),
        "effective_mode": requested,
        "pending_mode": requested,
        "request_pending": False,
        "owner": "PI.CLOCKS",
    }
    logging.info("🔧 [clocks] Pi-owned SERVOS: %s -> %s", previous, requested)
    return {"success": True, "message": "OK", "payload": payload}


def cmd_dither_enable(_: Optional[dict] = None) -> Dict[str, Any]:
    _dac_set_dither_enabled(True)
    payload = {
        "status": "dither_enabled",
        "enabled": True,
        "started": True,
        "service_pending": False,
        "write_context": "PI_CLOCKS_ONE_SECOND_LOW_FIRST_FRAME",
        "realization_mode": "ONE_SECOND_FRACTIONAL_DITHER",
        "owner": "PI.CLOCKS",
        "ocxo1": _dac_control_lane_snapshot(_dac_lanes["ocxo1"]),
        "ocxo2": _dac_control_lane_snapshot(_dac_lanes["ocxo2"]),
    }
    return {"success": True, "message": "OK", "payload": payload}


def cmd_dither_disable(_: Optional[dict] = None) -> Dict[str, Any]:
    # Mechanical parity with the current no-surprise-disable doctrine: stop the
    # waveform and clear queued motion without a final static DAC write.
    _dac_set_dither_enabled(False)
    payload = {
        "status": "dither_disabled_no_dac_write",
        "enabled": False,
        "started": False,
        "service_pending": False,
        "write_context": "PI_CLOCKS_ONE_SECOND_LOW_FIRST_FRAME",
        "realization_mode": "STATIC_ROUNDED",
        "owner": "PI.CLOCKS",
        "ocxo1": _dac_control_lane_snapshot(_dac_lanes["ocxo1"]),
        "ocxo2": _dac_control_lane_snapshot(_dac_lanes["ocxo2"]),
    }
    return {"success": True, "message": "OK", "payload": payload}


def cmd_set_dither(args: Optional[dict]) -> Dict[str, Any]:
    """Compatibility wrapper around Pi-owned DITHER_ENABLE/DITHER_DISABLE."""
    if not args or "dither" not in args:
        return {"success": False, "message": "DITHER requires 'dither' argument"}
    raw = args["dither"]
    if isinstance(raw, bool):
        enabled = raw
    elif isinstance(raw, str) and raw.strip().lower() in ("true", "1", "yes", "on"):
        enabled = True
    elif isinstance(raw, str) and raw.strip().lower() in ("false", "0", "no", "off"):
        enabled = False
    else:
        return {"success": False, "message": f"Invalid dither value: {raw}"}

    response = cmd_dither_enable() if enabled else cmd_dither_disable()
    if isinstance(response.get("payload"), dict):
        response["payload"]["persistence"] = "CLOCKS"
        if any(key in args for key in ("rate_hz", "hz", "frequency_hz")):
            response["payload"]["requested_rate_ignored"] = True
            response["payload"]["effective_frame_hz"] = 1.0
    return response


def cmd_dac_info(_: Optional[dict] = None) -> Dict[str, Any]:
    return {"success": True, "message": "OK", "payload": _dac_info_payload()}



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
    "REPAIR": cmd_repair,
    "CLEAR": cmd_clear,
    "DELETE": cmd_delete,
    "TRUNCATE": cmd_truncate,
    "SET_DAC": cmd_set_dac,
    "SERVOS": cmd_servos,
    "DITHER": cmd_set_dither,
    "DITHER_ENABLE": cmd_dither_enable,
    "DITHER_DISABLE": cmd_dither_disable,
    "DAC_INFO": cmd_dac_info,
    "SET_BASELINE": cmd_set_baseline,
    "BASELINE_INFO": cmd_baseline_info,
    "LIST_CAMPAIGNS": cmd_list_campaigns,
    "CLOCKS_INFO": cmd_clocks_info,
}

_HARD_FAILURE_READ_ONLY_COMMANDS = {
    "REPORT",
    "REPORT_CLOCKS",
    "REPORT_STATS",
    "DAC_INFO",
    "BASELINE_INFO",
    "LIST_CAMPAIGNS",
    "CLOCKS_INFO",
}
_HARD_FAILURE_OPERATOR_REPAIR_COMMANDS = {
    "REPAIR",
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
                    f"CLOCKS.{command} refused while asynchronous STATS_RESET owns "
                    "the statistics epoch boundary"
                ),
                "payload": {
                    "stats_reset_in_progress": True,
                    "last_stats_reset": copy.deepcopy(_diag.get("last_stats_reset") or {}),
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
            and command not in _HARD_FAILURE_OPERATOR_REPAIR_COMMANDS
        ):
            return {
                "success": False,
                "message": f"CLOCKS.{command} refused: subsystem is latched in HARD_FAILURE",
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


def on_publication(topic: str, payload: Dict[str, Any]) -> None:
    """Execute one publication that PUBSUB already routed to PI:CLOCKS."""
    if topic == CLOCKS_FRAGMENT_TOPIC:
        on_clocks_fragment(payload)
        return
    if topic == "WATCHDOG_ANOMALY":
        on_watchdog_anomaly(payload)
        return
    if topic == CLOCKS_RECOVERY_STALLED_TOPIC:
        on_recovery_stalled(payload)
        return
    raise RuntimeError(f"PI:CLOCKS received unexpected static route topic {topic!r}")


# ---------------------------------------------------------------------
# Entrypoint
# ---------------------------------------------------------------------

def run() -> None:
    global _startup_survival_ingress_barrier_monotonic

    setup_logging()
    _setup_invalid_timebase_logger()
    _hard_failure_event.clear()
    _hard_failure_stats_repair_event.clear()
    _repair_request_in_progress.clear()
    # From command-server exposure through the holistic finalizer there is exactly
    # one startup owner.  REPAIR requests may queue during this interval but may
    # not mutate lineage concurrently with it.
    _startup_reconciliation_active.set()
    _set_operational_state(
        OPERATIONAL_STATE_STARTING,
        reason="process_start",
        source="RUN",
    )

    _startup_control_ready.clear()
    with _startup_watchdog_lock:
        _startup_watchdog_deferred.clear()
    _clocks_persistence_enabled.clear()
    _reset_startup_clocks_custody()
    _clocks_epoch_birth_pending.clear()
    _clocks_epoch_birth_committed.clear()
    _clocks_holistic_restore_proof_pending.clear()
    _clocks_holistic_restore_proof_committed.clear()
    _startup_instrument_restore_hold.clear()
    _startup_physical_lifetime_unclassified.clear()
    _ambient_instrument_recovery_hold.clear()
    _startup_survival_ingress_barrier_monotonic = None
    _diag["startup_control_ready"] = False

    _dac_start_control_thread()
    _dac_initialize_hardware()

    # Expose CLOCKS command/publication ingress before any potentially slow
    # durable-state read.  PUBSUB topology is already static code truth; the
    # state worker below has not started yet, so arriving CLOCKS_FRAGMENT rows
    # can only queue and cannot mutate checkpoint custody before the durable seed.
    server_setup(
        subsystem="CLOCKS",
        commands=COMMANDS,
        publication_handler=on_publication,
        blocking=False,
    )

    # SYSTEM/PUBSUB readiness is application truth, not systemd ordering.  Keep
    # CLOCKS_FRAGMENT ingress open while waiting, but do not touch PostgreSQL or
    # issue Teensy RPC until those exact planes have been proved usable.
    _wait_for_startup_infrastructure()

    # Resume the Pi-owned literal Better-Buckets checkpoint from the singleton
    # config.CLOCKS_RECOVERY image before queued live ingestion is consumed.
    # The image is paired back to its exact durable CLOCKS statistics identity;
    # legacy embedded checkpoints are migrated once without history replay.
    _seed_ppb_checkpoint_runtime_from_latest_durable()

    logging.info(
        "🕐 [clocks] CLOCKS owns CLOCKS_FRAGMENT ingestion, SYSTEM.REPORT context pull, "
        "canonical CLOCKS publication, Pi-owned DAC/servo/dither actuation, campaign_detail "
        "persistence, TEMPEST adjudication, and holistic subsystem restore. "
        "Campaign execution is restored as CLOCKS state."
    )

    # Start the live data plane immediately. Ordinary persistence remains closed,
    # but exact canonical rows are retained in startup custody until the Teensy
    # lifecycle probe proves live continuation or a cold/full restore supersedes
    # them. No valid startup second is silently discarded.
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

    # Rows delivered before this point may be PUBSUB's retained tail from a
    # producer lifetime that disappeared during flash. Preserve them as evidence,
    # but never let later state-worker processing freshen them into survival proof.
    _startup_survival_ingress_barrier_monotonic = time.monotonic()
    logging.info(
        "🧭 [clocks/startup] armed current-session CLOCKS_FRAGMENT survival barrier; "
        "only rows entering PUBSUB/CLOCKS after worker admission can prove Alpha survived"
    )

    # First prove startup from the ordinary CLOCKS_FRAGMENT stream.  A surviving
    # Teensy therefore receives no lifecycle/status RPC at all during Pi-only
    # recovery.  If canonical descendant proof later fails, the existing dead-
    # producer branch is still free to wait for Alpha and use firmware recovery.
    startup_location = _wait_for_startup_location()
    required_receiver_mode = str(
        startup_location.get("verified_pos_mode_name") or ""
    ).upper()
    _wait_for_preflight(
        "holistic_startup",
        required_gnss_freq_mode=STARTUP_REQUIRED_GNSS_FREQ_MODE,
        required_gnss_freq_mode_name="FINE_LOCK",
        required_receiver_mode=required_receiver_mode,
        allow_restore_court_entry=False,
    )

    _set_operational_state(
        OPERATIONAL_STATE_RECOVERING,
        reason="startup_holistic_restore",
        source="RUN",
    )
    try:
        result = _holistic_restore(preverified_location=startup_location)
        result["startup_watchdog_reconciliation"] = (
            _reconcile_deferred_startup_watchdogs()
        )
        instrument = result.get("instrument") if isinstance(result.get("instrument"), dict) else {}
        campaign_result = result.get("campaign") if isinstance(result.get("campaign"), dict) else {}
        instrument_proof = instrument.get("proof") if isinstance(instrument.get("proof"), dict) else {}
        observed = instrument_proof.get("observed") if isinstance(instrument_proof.get("observed"), dict) else {}
        logging.info(
            "✅ [holistic restore] startup reconciliation complete: campaign=%s; "
            "producer=%s mode=%s; restored statistics update=%s; "
            "campaign first public=%s science_clean=%s",
            result.get("active_campaign") or "none",
            "resurrected" if instrument.get("producer_resurrected_this_startup") else "survived",
            instrument.get("mode") or "no instrument restore",
            observed.get("ppb_checkpoint_update_count") or observed.get("update_count") or "n/a",
            campaign_result.get("first_public_pps_vclock_count") or "n/a",
            campaign_result.get("science_clean") if campaign_result else "n/a",
        )
    except HardFailureRequired:
        # The restore court has already latched HARD_FAILURE with exact evidence.
        pass
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
        unresolved_custody = _startup_clocks_custody_unresolved()
        failure_details = {
            "error": str(exc),
            "startup_custody_unresolved": unresolved_custody,
            "startup_instrument_restore_hold": _startup_instrument_restore_hold.is_set(),
            "holistic_restore_proof_pending": _clocks_holistic_restore_proof_pending.is_set(),
            "holistic_restore_proof_committed": _clocks_holistic_restore_proof_committed.is_set(),
        }
        logging.exception(
            "💥 [holistic restore] failed; startup CLOCKS custody %s — latching HARD_FAILURE",
            "remains fail-closed" if unresolved_custody else "has been classified",
        )
        try:
            _cleanup_after_recovery_failure(
                "holistic_restore_failed",
                failure_details,
            )
        except Exception:
            logging.exception("⚠️ [holistic restore] cleanup also failed")
        _enter_hard_failure(
            "startup_holistic_restore_unproved",
            failure_details,
            source="RUN_HOLISTIC_RESTORE",
        )
    finally:
        # Startup reconciliation is a single-owner transaction.  The ownership
        # event is also the queue barrier for CLOCKS.REPAIR, so it must be released
        # even if the terminal finalizer itself discovers (or throws while reporting)
        # a failure.  Previously the clear lived *after* this finally block; any
        # exception in the finalizer could therefore strand REPAIR forever behind an
        # owner that no longer had executable work.
        try:
            if _hard_failure_active():
                _startup_control_ready.clear()
                _clocks_persistence_enabled.clear()
                _diag["startup_control_ready"] = False
                logging.critical(
                    "🛑 [clocks] startup entered HARD_FAILURE — service remains alive "
                    "for read-only diagnostics; persistence/control stay closed"
                )
            elif _startup_clocks_custody_unresolved():
                details = {
                    "startup_custody_unresolved": True,
                    "startup_instrument_restore_hold": _startup_instrument_restore_hold.is_set(),
                    "holistic_restore_proof_pending": _clocks_holistic_restore_proof_pending.is_set(),
                    "holistic_restore_proof_committed": _clocks_holistic_restore_proof_committed.is_set(),
                    "action": "Run CLOCKS.REPAIR; unresolved startup custody may not remain indefinitely RECOVERING.",
                }
                logging.error(
                    "💥 [clocks] startup reconciliation returned with unresolved custody — "
                    "promoting RECOVERING limbo to HARD_FAILURE so CLOCKS.REPAIR has explicit authority"
                )
                _enter_hard_failure(
                    "startup_reconciliation_custody_unresolved",
                    details,
                    source="RUN_HOLISTIC_RESTORE_FINALIZER",
                )
            else:
                _clocks_persistence_enabled.set()
                _startup_control_ready.set()
                _diag["startup_control_ready"] = True
                _set_operational_state(
                    OPERATIONAL_STATE_RUNNING,
                    reason="startup_reconciliation_complete",
                    source="RUN",
                )
                logging.info(
                    "✅ [clocks] startup state reconciliation complete — START/RESUME enabled"
                )
        finally:
            # This is the definitive ownership release.  A queued REPAIR worker may
            # re-diagnose only after this point, never concurrently with startup.
            _startup_reconciliation_active.clear()
            logging.info(
                "🧯 [clocks/startup] startup reconciliation ownership released: state=%s "
                "hard_failure=%s startup_control_ready=%s",
                _operational_state_snapshot().get("state"),
                _hard_failure_active(),
                _startup_control_ready.is_set(),
            )

    threading.Thread(
        target=_timebase_silence_monitor_loop,
        daemon=True,
        name="clocks-timebase-silence-monitor",
    ).start()

    logging.info(
        "🏁 [clocks] entering main loop operational_state=%s",
        _operational_state_snapshot().get("state"),
    )
    while True:
        time.sleep(3600)




if __name__ == "__main__":
    run()
