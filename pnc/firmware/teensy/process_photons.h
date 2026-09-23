#pragma once

#include <stdint.h>

// ============================================================================
// PHOTONS — optical instrument subsystem
// ============================================================================
//
// PHOTONS is the Teensy umbrella for photon-producing and photon-detecting
// hardware used by LANTERN and related fiber-optic experiments.
//
// Ownership:
//   • PHOTONS owns laser driver configuration/control and optical-device
//     telemetry.
//   • process_interrupt owns PD200T comparator edge capture and immutable
//     DWT-at-edge custody.
//   • PHOTONS consumes those edge facts and publishes PHOTONS_FRAGMENT.
//   • PHOTONS processing is foreground-only. Raw edge capture belongs to
//     process_interrupt; edge delivery runs as a bounded foreground service.
//     Scheduling/statistics transactions have one foreground owner at a time:
//     the cadence callback, 1 Hz fragment transaction,
//     one RPC command, or commissioning WAVE callback. These owners may never nest;
//     illegal overlap is a system-integrity fault rather than a recoverable busy
//     condition.
//   • foreground commits a complete launch record through an SPSC handoff;
//     foreground edge service owns live race state and the completed-race batch.
//     It may execute synchronously inside a boundary/fragment transaction; it
//     never acquires another owner or mutates the active scheduling transaction.
//     Batch, reference metadata and completion counters publish together.
//     Foreground copies both batch and runtime before acknowledging the result;
//     only then may it reuse launch storage. Publication generations may wrap.
//     Only foreground writes/resets the separate fragment accumulator.
//   • readiness and reports read completed handoffs, never live race fields.
//     A fragment uses the runtime acquired at its batch-drain boundary; later
//     raw IRQ arrivals cannot change its reference or counters during construction.
//     Runtime reports show the latest consumed completion plus the subsequent
//     foreground launch; out-of-window edge totals advance with completed handoffs.
//   • foreground edge service writes the boot-lifetime histogram, including origin
//     installation and seed replay. Cadence infers the origin from a completed
//     immutable seed set and returns it through a one-shot SPSC handoff.
//   • reports use producer-published typed snapshots. Foreground holds a slot
//     until its next acquisition; producer cannot reuse that slot while held.
//     Payload workspace is separate byte storage, never a snapshot overlay.
//
// PHOTONS_FRAGMENT is the canonical once-per-second optical instrument heartbeat.
// It remains lawful and continuous when the race engine is inactive or a second
// contains zero races; race absence is testimony, while fragment silence is a
// producer-health event. Physical testimony and interpreted statistics remain
// separate: each completed
// single-pass race preserves its raw DWT endpoints beside the GNSS-projected
// estimated flight interval, science-admission testimony, Welford sufficient
// state, and recovery totals.
//
// Independent launch cadence:
//   • initialization registers a TimePop foreground service (default 10 us);
//     each due service emits a nominal 200 ns MOD pulse, regardless of PD arrivals;
//     the next deadline is one configured interval after the actual launch DWT,
//     using the existing F_CPU_ACTUAL conversion to DWT cycles. START establishes
//     the first origin; late service emits one pulse and starts a fresh interval;
//   • measurement starts after recovery establishes statistical ancestry;
//   • foreground publishes an explicit 4000-10000 ns acquisition window at each
//     launch, clipped before the cadence deadline. Bounds use the nominal DWT
//     clock, independently of statistics/reset/recovery; science retains GNSS
//     projection and delay classification after capture;
//   • Priority 48 admits at most one in-window raw candidate per launch. Other
//     active-detector hits increment SPURIOUS (EARLY/DUPLICATE/LATE/UNARMED)
//     before queueing; foreground retains
//     science/delay classification. No optical processing executes at Priority 32;
//   • the next cadence service or PHOTONS_STOP closes an unanswered shot as
//     missed. Attempts = completed + missed + pending, with at most one pending;
//   • each launch pauses ONLY the detector IRQ, drains all captured edges against
//     the old shot, and clears uncaptured pending GPIO state. After the new launch
//     record is published, detector capture resumes without clearing a new return.
//     CLOCKS and Priority 32 remain live throughout. Raw queue overflow traps.
//     The timestamp is sampled after MOD HIGH;
//   • readiness is polled by TimePop, including while idle; no recurring grid
//     or cancellable one-shot appointment owns the next launch. Actual GPIO
//     timing still includes foreground service and interrupt latency. No catch-up
//     pulse burst. The legacy grid-deferral counter remains zero;
//   • START/STOP retain campaign semantics. PHOTONS_START/PHOTONS_STOP control
//     laser cadence independently; statistics and fragment publication continue.
//
// Commands:
//   • INIT                — reinitialize PHOTONS-owned optical I/O and force the
//                           active-high DRV200 MOD command LOW/idle
//   • DETECTOR_ACTIVATE   — commissioning-only activation of the already-subscribed PD200T
//                           interrupt lane; MOD remains LOW and no race/publisher starts
//   • START               — start a LANTERN campaign, or hot-cut an active campaign to a new name
//   • FLASH_CUT           — explicit hot campaign boundary preserving the always-on instrument epoch
//   • STOP                — request campaign closure; the next published campaign fragment is final
//   • PHOTONS_START [interval=N] — run laser cadence; N is ns, 10000..1000000000.
//                           Omitted interval preserves the current setting.
//   • PHOTONS_STOP        — cancel cadence, close pending shot, force MOD LOW.
//   • REPORT              — compact operational/device report including active-high MOD state,
//                           laser monitor and pin-34 interrupt custody; no PD OUT ADC telemetry
//   • WAVEON interval=N width=W — commissioning pulse train on LASER_MOD_PIN 35.
//                           Both parameters are required uint64 nanoseconds;
//                           0 < W < N. The former ns/full-cycle argument is retired.
//                           Example: interval=100000000 width=200 (10 Hz, ~200 ns HIGH).
//                           First pulse fires immediately after the timer is armed.
//                           TimePop owns recurring pulse-start cadence at interval N;
//                           each pulse holds HIGH using F_CPU_ACTUAL/DWT polling,
//                           then returns LOW before the command/callback returns.
//                           No falling-edge TimePop event and no IRQ masking.
//                           Width is approximate; GPIO/loop/IRQ latency may extend it.
//                           Foreground is occupied during HIGH; long widths delay
//                           commands and other callbacks. Long waits share PULSE's
//                           DWT-wrap handling/limits. Cadence retains TimePop's timing
//                           semantics; width < interval does not prove deadline fit.
//                           No manual receive record or race science is authored.
//                           Reply reports interval_ns, width_ns and output_level=LOW.
//   • WAVEOFF             — cancel the commissioning wave and force pin 35 LOW/idle.
//   • PULSE [ns=N]        — one active-high DRV200 MOD pulse. MOD must be LOW before
//                           the shot and returns LOW afterward. Default: 1000 ns.
//                           N is any positive uint64 integer; no policy duration cap.
//                           Uses F_CPU_ACTUAL and DWT polling with interrupts enabled;
//                           foreground remains occupied until MOD returns LOW. Long waits
//                           accumulate unsigned deltas across DWT wraps. An IRQ gap
//                           of a full DWT revolution cannot be recovered from DWT alone.
//                           Requested width is approximate; loop/write/IRQ latency may
//                           extend the physical HIGH interval. Measure it with the scope.
//                           Output: LASER_MOD_PIN 35. Detector input remains pin 34,
//                           read through process_interrupt's authoritative GPIO2 accessor.
//   • REPORT_PULSE        — latest manual-shot raw evidence (PHOTONS_PULSE_REPORT_V3):
//                           requested ns, target whole seconds + fractional cycles,
//                           CPU/DWT rate, write-bracketing start/end DWT and first
//                           observed detector edge after arming. pulse_wall_cycles is
//                           modulo 2^32, not a long-duration elapsed-time measurement.
//                           No GNSS lock is required; flight_time_valid remains false.
//                           Raw edges are not proof of optical return. Each accepted
//                           shot replaces the prior report; rejections preserve it.
//   • REPORT_PHOTONS      — compact always-on instrument + current CAMP report
//   • REPORT_STATS        — detailed statistical/court/Better-Buckets report
//   • REPORT_HISTOGRAM    — boot-lifetime raw-cycle histogram: 64 one-cycle bins
//                           per population, with midpoint inferred from 65 seeds.
//                           Returns the latest completed SPSC snapshot and requests
//                           a refresh; the 1 Hz fragment also requests refreshes.
//                           A request is fulfilled on the next completed race.
//                           Includes snapshot_sequence and raw snapshot_dwt;
//                           campaigns and STATS_RESET preserve acquisition/bins.
//   • REPORT_ENVELOPE     — alpha lower-envelope estimate for accepted autonomous races:
//                           per-fragment ranks 1%-10%, 128 one-cycle bins, latest
//                           completed histogram and eight recent fragment summaries.
//                           At least 1000 accepted races required; censored ranks
//                           have no estimate. Bin origin follows prior batch mean.
//                           Fractional COUNT weights do not interpolate within bins;
//                           a band inside one bin remains quantized with zero
//                           selected spread, not zero measurement uncertainty.
//                           Canonical LAP/SD, science admission and campaign sums
//                           retain their meanings. instrument.envelope carries
//                           the same compact testimony in every fragment.
//                           History resets with physical recovery; STATS_RESET,
//                           campaigns and cadence stop/start preserve ancestry.
//   • STATS_RESET         — reset the always-on statistical epoch without changing CAMP custody
//   • PPB_EXPORT_META     — read-only live Better-Buckets ring identity for Pi custody reacquisition
//   • PPB_EXPORT_CHUNK    — page immutable live SECOND/MINUTE endpoints without freezing PHOTONS
//   • RECOVERY_BEGIN      — stage bounded Better-Buckets history while publication remains held
//   • RECOVERY_CHUNK      — append one bounded SECOND or MINUTE endpoint chunk
//   • RECOVERY_COMMIT     — atomically install durable aggregate/campaign state and start fresh ancestry
//   • RECOVERY_ABORT      — discard staged recovery state without starting publication
//   • RECOVERY_COLD_START — start a genuinely empty instrument when no durable state exists
//   • RECOVERY_PROOF_ACK  — acknowledge that the first advancing post-restore row is durable
//   • REPORT_RECOVERY     — report staging, restored source, and physical-ancestry testimony
//   • INJECT_PROBLEM      — retained command identity; synthetic injection is unavailable
//                           after retirement of the emulator
//   • ON                  — force active-high DRV200 MOD HIGH continuously; this does not
//                           control the DRV200 hardware switch or DC bias-current setting
//   • OFF                 — cancel any commissioning wave and force DRV200 MOD LOW/idle
//                           All direct MOD controls cancel a pending manual receive arm without
//                           erasing its already-captured testimony. No race is started.
// ============================================================================

// Cumulative ISR-authored optical-edge state from PD200T TTL pin 34.
struct photons_toy_capture_t {
  uint32_t edge_count = 0;

  uint32_t last_edge_sequence = 0;
  uint32_t last_pps_sequence = 0;
  uint32_t last_dwt_at_edge = 0;
  uint32_t last_isr_entry_dwt_raw = 0;
  int32_t  isr_entry_to_edge_correction_cycles = 0;

  bool     interval_valid = false;
  uint32_t last_interval_cycles = 0;
  uint32_t min_interval_cycles = 0;
  uint32_t max_interval_cycles = 0;

  // Toy static carry-forward predictor:
  // prediction(n) = actual_interval(n - 1)
  bool     prediction_valid = false;
  uint32_t prediction_cycles = 0;
  int32_t  residual_cycles = 0;
};

// Once-per-second foreground snapshot published as PHOTONS_FRAGMENT.
struct photons_toy_fragment_t {
  uint32_t sequence = 0;
  uint32_t publish_count = 0;

  uint32_t edge_count_total = 0;
  uint32_t edges_this_second = 0;

  photons_toy_capture_t capture{};

  // Snapshot of process_interrupt's PHOTODIODE lane testimony.
  uint32_t interrupt_irq_count = 0;
  uint32_t interrupt_callback_count = 0;
  uint32_t interrupt_callback_missing_count = 0;
  uint32_t interrupt_inactive_edge_count = 0;
  uint32_t interrupt_source_pin = 0;
  uint32_t interrupt_last_callback_wall_cycles = 0;
  uint32_t interrupt_max_callback_wall_cycles = 0;
};

// Canonical statistical sufficient state.  This mirrors the CLOCKS Welford
// publication contract: n/mean/m2/min/max are sufficient for exact resurrection;
// stddev/stderr are derived convenience values carried in each fragment.
struct photons_fragment_welford_snapshot_t {
  uint64_t n = 0;
  double mean = 0.0;
  double m2 = 0.0;
  double stddev = 0.0;
  double stderr_value = 0.0;
  double min = 0.0;
  double max = 0.0;
};


// Alpha lower-envelope testimony for one autonomous accepted-race fragment.
// The estimate is the weighted mean of ranks 1%-10%, using actual integer DWT
// values and fractional COUNT weights at the two rank boundaries. Its selected
// SD describes the trimmed distribution, not uncertainty of the estimator.
// No recovered/campaign aggregate is replaced by this experimental statistic.
struct photons_envelope_snapshot_t {
  uint32_t sequence = 0;
  uint32_t dwt_cycles_per_second = 0;
  uint64_t accepted_count = 0;
  uint32_t origin_cycles = 0;
  uint64_t underflow = 0;
  uint64_t overflow = 0;
  double accepted_mean_cycles = 0.0;
  double accepted_sd_cycles = 0.0;
  double selected_mean_cycles = 0.0;
  double selected_sd_cycles = 0.0;
  uint32_t selected_first_cycles = 0;
  uint32_t selected_last_cycles = 0;
  uint32_t selected_bins = 0;
  uint32_t in_range_mode_cycles = 0;
  uint32_t in_range_mode_count = 0;
};


// Compact raw-cycle courtroom.  The last-lap static prediction answers the
// immediate "does this lap look like the prior lap?" question.  The fragment
// mean comparison is the optical analogue of CLOCKS raw_cycles at 1 Hz: a
// second-scale sanity surface that should remain very boring in steady state.
struct photons_fragment_raw_cycles_snapshot_t {
  bool valid = false;
  uint64_t completed_lap_count = 0;

  bool static_prediction_valid = false;
  uint32_t static_prediction_cycles = 0;
  uint32_t observed_cycles = 0;
  uint32_t previous_observed_cycles = 0;
  int32_t static_residual_cycles = 0;

  uint32_t laps_this_fragment = 0;
  uint64_t total_cycles_this_fragment = 0;
  double mean_cycles_this_fragment = 0.0;
  uint32_t min_cycles_this_fragment = 0;
  uint32_t max_cycles_this_fragment = 0;

  bool previous_fragment_mean_valid = false;
  double previous_fragment_mean_cycles = 0.0;
  double fragment_mean_residual_cycles = 0.0;
};


// Projection testimony for the most recently accepted lap plus lifetime
// admission counters. PHOTONS consumes a cached immutable PPS/VCLOCK anchor.
// Ordinary detector ISR work never calls TIME/CLOCKS or performs Payload/statistical
// work; the explicitly armed one-shot PULSE path retains its first edge as raw testimony.
struct photons_fragment_projection_snapshot_t {
  bool anchor_cache_valid = false;
  uint32_t anchor_pps_count = 0;
  uint32_t anchor_dwt_at_pps_vclock = 0;
  uint32_t anchor_dwt_cycles_per_second = 0;

  uint64_t attempt_count = 0;
  uint64_t success_count = 0;
  uint64_t reject_count = 0;
  uint64_t queue_overflow_count = 0;

  bool last_valid = false;
  uint32_t last_pps_sequence = 0;
  uint32_t last_start_dwt = 0;
  uint32_t last_end_dwt = 0;
  uint32_t last_raw_cycles = 0;
  uint64_t last_start_gnss_ns = 0;
  uint64_t last_end_gnss_ns = 0;
  uint64_t last_lap_gnss_ns = 0;
};


// Lap-level science court.  The lap is the unit of scientific admission; the
// once-per-second PHOTONS_FRAGMENT is only a batch/custody envelope.  Every
// survivable candidate remains visible, but only ACCEPT mutates the canonical
// scientific numerator/Welford and predictor.
//
// Accepted and excluded populations are both characterized.  The accepted
// projected-lap Welford is the same scientific population published in
// photons_fragment_stats_snapshot_t; excluded projected-lap statistics exist
// only when projection itself succeeded.  Raw-cycle statistics exist for both
// populations so rejected timing injuries remain quantitatively observable.
enum class photons_lap_science_disposition_t : uint8_t {
  NONE = 0,
  ACCEPT = 1,
  SCIENCE_EXCLUDE = 2,
  PENDING_SEED = 3,
};

enum class photons_lap_science_exclusion_reason_t : uint16_t {
  NONE = 0,
  PROJECTION_INVALID = 100,
  SEED_DISAGREEMENT = 200,
  RAW_CYCLE_EXCURSION = 300,
  ISR_DELAY = 400,
};


struct photons_lap_science_population_snapshot_t {
  // ACCEPT owns these fields directly.  For the excluded population they are
  // publication-only conveniences derived from the exclusion-reason ledger;
  // PHOTONS never advances a second mutable aggregate exclusion counter.
  uint64_t count = 0;
  uint32_t count_this_fragment = 0;

  // raw_cycles.n == count for every finalized candidate in this population.
  photons_fragment_welford_snapshot_t raw_cycles{};

  // Projection-invalid exclusions have no lawful GNSS duration, so for the
  // excluded population projected_lap_ns.n may be smaller than count.
  photons_fragment_welford_snapshot_t projected_lap_ns{};
};


struct photons_lap_science_reason_counts_snapshot_t {
  // Sole authority for the excluded population.  Every SCIENCE_EXCLUDE owns
  // exactly one reason, so excluded.count is sum(these three counters).
  uint64_t projection_invalid = 0;
  uint64_t seed_disagreement = 0;
  uint64_t raw_cycle_excursion = 0;
  uint64_t isr_delay = 0;

  uint32_t projection_invalid_this_fragment = 0;
  uint32_t seed_disagreement_this_fragment = 0;
  uint32_t raw_cycle_excursion_this_fragment = 0;
  uint32_t isr_delay_this_fragment = 0;
};


struct photons_lap_science_snapshot_t {
  bool valid = false;

  uint64_t candidate_count = 0;
  uint32_t candidates_this_fragment = 0;

  photons_lap_science_population_snapshot_t accepted{};
  photons_lap_science_population_snapshot_t excluded{};
  photons_lap_science_reason_counts_snapshot_t exclusion_reasons{};

  // Three mutually close clean flights establish the initial raw-cycle lineage.
  bool predictor_valid = false;
  uint32_t predictor_cycles = 0;
  uint32_t gate_cycles = 0;
  uint32_t reject_streak = 0;
  uint32_t max_reject_streak = 0;

  bool seed_pending = false;
  uint32_t seed_pending_count = 0;
  uint64_t seed_pending_candidate_index = 0;
  uint32_t seed_pending_raw_cycles = 0;
  uint64_t seed_pending_lap_gnss_ns = 0;

  // Last finalized candidate verdict.  A PENDING_SEED candidate is separately
  // visible above and has not yet entered either finalized population.
  uint64_t last_candidate_index = 0;
  uint8_t last_disposition_id = 0;
  uint16_t last_reason_code = 0;
  bool last_projection_valid = false;
  uint32_t last_pps_sequence = 0;
  uint32_t last_observed_cycles = 0;
  uint32_t last_prediction_cycles = 0;
  int32_t last_residual_cycles = 0;
  uint32_t last_gate_cycles = 0;
  uint64_t last_lap_gnss_ns = 0;
};


// One accepted-flight population. A zero sample_count means no measured mean;
// the serializer omits that bucket rather than publishing a zero lap duration.
struct photons_fragment_lap_value_snapshot_t {
  uint64_t sample_count = 0;
  double mean_lap_ns = 0.0;
};


// Instrument-owned rolling/lifetime mean lap durations in nanoseconds.
// LANTERN is a separate accepted-flight population, independent of these windows.
// Existing PPB-named checkpoint types below retain the count/time recovery wire
// protocol only; they carry no lap baseline or residual statistics.
struct photons_fragment_lap_buckets_snapshot_t {
  photons_fragment_lap_value_snapshot_t minute_10{};
  photons_fragment_lap_value_snapshot_t minute_60{};
  photons_fragment_lap_value_snapshot_t hour_8{};
  photons_fragment_lap_value_snapshot_t hour_24{};
  photons_fragment_lap_value_snapshot_t total{};
};


// Compact PHOTONS-authored Better-Buckets sufficient state carried at 1 Hz.
// Pi PHOTONS may use the append testimony to maintain a literal bounded recovery
// checkpoint, but it may not reconstruct or re-author producer endpoint state.
struct photons_fragment_ppb_endpoint_snapshot_t {
  uint32_t sequence = 0;
  uint64_t lap_count = 0;
  uint64_t total_lap_gnss_ns = 0;
};


struct photons_fragment_ppb_window_proof_snapshot_t {
  bool valid = false;
  uint64_t sample_count = 0;
  photons_fragment_ppb_endpoint_snapshot_t anchor{};
};


struct photons_fragment_ppb_checkpoint_delta_snapshot_t {
  bool valid = false;
  uint32_t rolling_sequence = 0;
  uint32_t second_count = 0;
  uint32_t minute_count = 0;
  uint32_t last_minute_key = 0;

  bool origin_valid = false;
  photons_fragment_ppb_endpoint_snapshot_t current{};
  photons_fragment_ppb_endpoint_snapshot_t origin{};

  photons_fragment_ppb_window_proof_snapshot_t minute_10{};
  photons_fragment_ppb_window_proof_snapshot_t minute_60{};
  photons_fragment_ppb_window_proof_snapshot_t hour_8{};
  photons_fragment_ppb_window_proof_snapshot_t hour_24{};

  bool second_append_valid = false;
  photons_fragment_ppb_endpoint_snapshot_t second_append{};
  bool minute_append_valid = false;
  photons_fragment_ppb_endpoint_snapshot_t minute_append{};
};


// Always-on optical statistics.  lap_count + total_lap_gnss_ns is the
// authoritative grand ratio for mean lap time.  Welford independently carries
// variance and doubles as a consistency witness for that ratio.  reset_count
// identifies the statistical epoch; update_count is the within-epoch logical
// chronology used by Better-Buckets and, later, durable replay.
struct photons_fragment_stats_snapshot_t {
  bool valid = false;
  uint32_t reset_count = 0;
  uint32_t update_count = 0;
  uint64_t lap_count = 0;
  uint64_t total_lap_gnss_ns = 0;

  // Monotonic accepted-lap custody survives STATS_RESET and is part of the
  // immutable publication value.  The serializer must never reach back into
  // live PHOTONS globals for these totals.
  uint64_t custody_lap_count = 0;
  uint64_t custody_total_lap_gnss_ns = 0;

  double mean_lap_ns = 0.0;
  photons_fragment_welford_snapshot_t lap_time_welford{};
  photons_fragment_lap_buckets_snapshot_t lap_buckets{};

  // Recovery-only Better-Buckets witnesses.  update_count is the logical
  // rolling chronology; current_sequence identifies the latest lawful endpoint.
  uint32_t rolling_ppb_current_sequence = 0;
  bool rolling_ppb_endpoint_admitted = false;
  bool rolling_ppb_interval_advanced = false;

  // Self-contained current-window proof plus exact ring append testimony.
  photons_fragment_ppb_checkpoint_delta_snapshot_t rolling_ppb_checkpoint{};
};


// Firmware-authored LANTERN campaign measurement.  Pi owns campaign lifecycle,
// and durable identity; PHOTONS owns the exact recording
// boundary and CAMP statistics.  Campaign N/T is based on monotonic custody
// totals that survive STATS_RESET, while the always-on statistical N/T above may
// begin a fresh epoch.  Thus campaign transitions and statistics resets are
// mutually non-destructive, matching CLOCKS Alpha/Beta ownership.
struct photons_fragment_campaign_snapshot_t {
  bool present = false;
  bool final = false;
  char campaign[64] = {0};
  uint32_t start_after_sequence = 0;
  uint32_t stop_after_sequence = 0;
  uint32_t public_count = 0;
  uint64_t lap_count = 0;
  uint64_t total_lap_gnss_ns = 0;
  double mean_lap_ns = 0.0;
};


// Durable recovery restores statistical sufficient state and logical chronology,
// never physical edge ancestry.  These fields make that negative contract
// directly testable in every post-restore PHOTONS_FRAGMENT.
struct photons_fragment_recovery_snapshot_t {
  bool restored = false;
  bool proof_pending = false;
  bool proof_advanced = false;
  bool proof_committed = false;
  uint32_t generation = 0;
  uint32_t source_sequence = 0;
  uint32_t source_publish_count = 0;
  uint32_t source_reset_count = 0;
  uint32_t source_update_count = 0;
  uint64_t source_lap_count = 0;
  uint64_t source_total_lap_gnss_ns = 0;
  uint64_t source_custody_lap_count = 0;
  uint64_t source_custody_total_lap_gnss_ns = 0;
  uint64_t accepted_lap_delta = 0;
  uint64_t custody_lap_delta = 0;
  bool fresh_physical_ancestry = false;
  bool raw_lap_ring_restored = false;
  bool partial_lap_restored = false;
  bool pending_seed_restored = false;
  bool predictor_restored = false;
  bool in_flight_train_restored = false;
};


// Canonical once-per-second PHOTONS handoff.  The always-on instrument subtree
// remains authoritative and campaign-independent.  Optional campaign testimony
// is a recording-relative sibling authored by firmware, matching CLOCKS_FRAGMENT:
// Pi may add durable campaign identity but never recomputes CAMP mean duration.
struct photons_fragment_snapshot_t {
  // SPURIOUS is a count of active-detector arrivals rejected before raw queue
  // admission, not a count of flights. One flight may have spurious arrivals
  // and still complete or be MISSED. Totals are boot-lifetime ISR testimony;
  // each fragment freezes a coherent snapshot and the delta since its predecessor.
  bool snapshot_ok = false;
  bool valid = false;
  uint32_t sequence = 0;
  uint32_t publish_count = 0;
  uint64_t fragment_period_ns = 0;

  uint32_t edge_count_total = 0;
  uint32_t edges_this_fragment = 0;

  // Legacy schema fields retained at zero while downstream consumers migrate
  // from the removed train/lap emulator terminology.
  uint32_t train_count = 0;
  uint32_t dead_lap_count = 0;
  uint64_t raw_lap_count = 0;
  uint32_t projected_laps_this_fragment = 0;

  // Real single-pass race telemetry. Lifetime counters are boot-local physical
  // testimony; the one-fragment Welford is derived only from completed projected
  // races in this exact PHOTONS_FRAGMENT. The publisher heartbeat is independent
  // of this producer lifecycle, so active=false with zero race counters is lawful.
  bool race_engine_active = false;
  uint32_t race_cadence_hz = 0;
  uint64_t race_cadence_ns = 0;
  uint32_t race_pending_return_count = 0;
  uint32_t race_pending_return_count_previous = 0;
  uint64_t race_pulse_ns = 0;
  uint64_t race_cadence_tick_count_total = 0;
  uint32_t race_cadence_ticks_this_fragment = 0;
  uint64_t race_attempt_count_total = 0;
  uint32_t race_attempts_this_fragment = 0;
  uint64_t race_completed_count_total = 0;
  uint32_t race_completed_this_fragment = 0;
  uint64_t race_missed_count_total = 0;
  uint32_t race_missed_this_fragment = 0;
  // Captured acquisition rejects outside science EXCL; missed subset below.
  uint64_t race_rejected_launch_timing_total = 0;
  uint64_t race_rejected_launch_timing_this_fragment = 0;
  uint64_t race_missed_launch_timing_total = 0;
  uint64_t race_missed_launch_timing_this_fragment = 0;
  uint64_t race_spurious_count_total = 0;
  uint64_t race_spurious_this_fragment = 0;
  uint64_t race_spurious_early_count_total = 0;
  uint64_t race_spurious_early_this_fragment = 0;
  uint64_t race_spurious_duplicate_count_total = 0;
  uint64_t race_spurious_duplicate_this_fragment = 0;
  uint64_t race_spurious_late_count_total = 0;
  uint64_t race_spurious_late_this_fragment = 0;
  uint64_t race_spurious_unarmed_count_total = 0;
  uint64_t race_spurious_unarmed_this_fragment = 0;
  // Nominal bounds plus effective inclusive cycle limits after cadence clipping.
  uint32_t race_capture_min_ns = 0;
  uint32_t race_capture_max_ns = 0;
  uint32_t race_capture_min_cycles = 0;
  uint32_t race_capture_max_cycles = 0;
  uint64_t race_skipped_not_quiet_total = 0;
  uint32_t race_skipped_not_quiet_this_fragment = 0;
  uint64_t race_skipped_projection_total = 0;
  uint32_t race_skipped_projection_this_fragment = 0;
  uint64_t race_invalid_endpoint_total = 0;
  uint32_t race_invalid_endpoint_this_fragment = 0;
  uint64_t race_enqueue_failure_total = 0;
  uint32_t race_enqueue_failure_this_fragment = 0;

  // Detector-return rejection testimony.
  uint64_t race_rejected_isr_delay_total = 0;
  uint32_t race_rejected_isr_delay_this_fragment = 0;
  uint64_t race_rejected_qtimer1_total = 0;
  uint64_t race_rejected_ocxo1_total = 0;
  uint64_t race_rejected_ocxo2_total = 0;
  uint64_t race_rejected_pps_total = 0;
  uint64_t race_rejected_continuation_total = 0;
  uint64_t race_rejected_unknown_total = 0;
  uint64_t race_rejected_excursion_total = 0;
  uint32_t race_rejected_excursion_this_fragment = 0;
  bool race_reference_valid = false;
  uint32_t race_reference_cycles = 0;
  uint32_t race_reference_gate_cycles = 0;
  uint32_t race_seed_count = 0;
  photons_fragment_welford_snapshot_t race_flight_this_fragment{};
  photons_envelope_snapshot_t envelope{};

  // Retired holdoff fields remain zero except holdoff_edges_total, which counts
  // arrivals outside an open receive window. No return schedules a launch.
  uint32_t race_pending_relaunch_count = 0;
  uint32_t race_pending_relaunch_count_previous = 0;
  uint32_t race_holdoff_ns = 0;
  uint32_t race_holdoff_cycles = 0;
  uint64_t race_holdoff_edges_total = 0;
  uint64_t race_holdoff_launches_total = 0;
  uint32_t race_holdoff_last_cycles = 0;
  uint32_t race_holdoff_min_cycles = 0;
  uint32_t race_holdoff_max_cycles = 0;

  photons_fragment_raw_cycles_snapshot_t raw_cycles{};
  photons_fragment_projection_snapshot_t projection{};
  photons_lap_science_snapshot_t science{};
  photons_fragment_stats_snapshot_t stats{};
  photons_fragment_campaign_snapshot_t campaign{};
  photons_fragment_recovery_snapshot_t recovery{};

  // Snapshot of process_interrupt's PHOTODIODE lane testimony. Lifetime
  // counters remain forensic evidence. Fresh physical ancestry snapshots the
  // two injury-counter origins; rolling custody judges only their unsigned
  // deltas since that boundary, so pre-publication startup edges cannot poison
  // every later fragment.
  bool     interrupt_subscribed = false;
  bool     interrupt_active = false;
  uint32_t interrupt_irq_count = 0;
  uint32_t interrupt_callback_count = 0;
  uint32_t interrupt_callback_missing_count = 0;
  bool     interrupt_ancestry_baseline_valid = false;
  uint32_t interrupt_callback_missing_origin = 0;
  uint32_t interrupt_callback_missing_since_ancestry = 0;
  uint32_t interrupt_inactive_edge_count = 0;
  uint32_t interrupt_inactive_edge_origin = 0;
  uint32_t interrupt_inactive_edge_since_ancestry = 0;
  uint32_t interrupt_source_pin = 0;
  uint32_t interrupt_last_callback_wall_cycles = 0;
  uint32_t interrupt_max_callback_wall_cycles = 0;

  // Direct copy of process_interrupt's PHOTODIODE Priority-0 blocker testimony.
  // The masks contain QTimer source bits only; GPIO6789 is intentionally absent
  // because the shared vector cannot distinguish PPS from pin 34 pending state.
  uint32_t interrupt_blocker_trace_count = 0;
  uint32_t interrupt_blocked_qtimer1_count = 0;
  uint32_t interrupt_blocked_ocxo1_count = 0;
  uint32_t interrupt_blocked_ocxo2_count = 0;
  uint32_t interrupt_last_blocker_wall_cycles = 0;
  uint32_t interrupt_max_blocker_wall_cycles = 0;
  uint32_t interrupt_last_qtimer_pending_at_entry_mask = 0;
  uint32_t interrupt_last_qtimer_pending_at_exit_mask = 0;
};

// Initialize PHOTONS state, subscribe to the detector, and start laser cadence.
// Recovery independently starts measurement and the 1 Hz fragment heartbeat.
// Must run after process_interrupt_init() and timepop_init().
void process_photons_init(void);

// Register the PHOTONS process command surface.
void process_photons_register(void);

