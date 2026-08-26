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
//
// PHOTONS_FRAGMENT is now the canonical once-per-second optical instrument
// handoff.  As in CLOCKS, physical testimony and interpreted statistics remain
// separate: raw DWT lap intervals are preserved beside GNSS-projected lap time,
// science-admission testimony, Welford sufficient state, and recovery totals.
//
// A projected lap is never silently discarded.  PHOTONS applies a CLOCKS-style
// ACCEPT / SCIENCE_EXCLUDE court before mutating scientific statistics.  Raw and
// projection testimony survives either verdict so downstream/Python analysis can
// inspect excluded observations.  The temporary emulator remains only a source
// of physical pin-34 edge facts while the PD200T is unavailable.
//
// Temporary bring-up emulator:
//   • pin 33 is a detector-emulator output physically looped to pin 34;
//   • process_interrupt still timestamps every resulting real pin-34 GPIO edge;
//   • PHOTONS emits a real ~20-DWT-cycle LD_ON launch pulse;
//   • one ordinary recurring TimePop timer supplies the coarse 10 ms edge cadence;
//     its callback advances emulator state only and never arms/cancels TimePop;
//   • physical loopback-edge timing remains visible as custody telemetry, while
//     emulator-only science laps are synthesized around STANDARD_LAP_NS with a
//     tiny bounded DWT-cycle variation so foreground scheduler jitter cannot
//     masquerade as optical propagation;
//   • each train emits exactly five physical detector edges:
//       hit 1 ignored by lap semantics,
//       hit 2 lap start,
//       hit 3 lap 1 end,
//       hit 4 lap 2 end,
//       hit 5 lap 3 end;
//   • the cadence tick after hit 5 is lap 4: no detector edge is emitted and
//     the next real laser pulse/train begins on that same recurring tick;
//   • when the emulator is enabled, pin 38/A14 PD OUT telemetry is synthesized;
//     with the physical PD200T selected, pin 38/A14 PD OUT is ADC-read directly;
//   • the emulator is always on and independent of campaign state;
//   • the TimePop cadence is bring-up transport timing, not optical truth.
//
// Commands:
//   • INIT                — reinitialize PHOTONS-owned optical hardware; laser is inhibited
//   • SET_STANDARD_LAP_NS — install the required optical PPB reference; fragment publication
//                           remains gated until this startup configuration is present
//   • START               — start a LANTERN campaign, or hot-cut an active campaign to a new name
//   • FLASH_CUT           — explicit hot campaign boundary preserving the always-on instrument epoch
//   • STOP                — request campaign closure; the next published campaign fragment is final
//   • REPORT              — compact operational/device report for laser, PD200T pin 38/A14
//                           PD OUT voltage, pin 34 comparator interrupt custody, publication
//                           identity, and hardware commissioning
//   • REPORT_PHOTONS      — compact always-on instrument + current CAMP report
//   • REPORT_STATS        — detailed statistical/court/Better-Buckets report
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
//   • INJECT_PROBLEM      — arm one synthetic lap excursion through the ordinary science court
//   • ON                  — permit laser emission through LD_ON
//   • OFF                 — inhibit laser emission through LD_ON
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
// admission counters.  PHOTONS consumes a cached immutable PPS/VCLOCK anchor;
// the detector ISR never calls TIME/CLOCKS or performs Payload/statistical work.
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
};


struct photons_lap_science_population_snapshot_t {
  uint64_t count = 0;
  uint32_t count_this_fragment = 0;

  // raw_cycles.n == count for every finalized candidate in this population.
  photons_fragment_welford_snapshot_t raw_cycles{};

  // Projection-invalid exclusions have no lawful GNSS duration, so for the
  // excluded population projected_lap_ns.n may be smaller than count.
  photons_fragment_welford_snapshot_t projected_lap_ns{};
};


struct photons_lap_science_reason_counts_snapshot_t {
  uint64_t projection_invalid = 0;
  uint64_t seed_disagreement = 0;
  uint64_t raw_cycle_excursion = 0;

  uint32_t projection_invalid_this_fragment = 0;
  uint32_t seed_disagreement_this_fragment = 0;
  uint32_t raw_cycle_excursion_this_fragment = 0;
};


struct photons_lap_science_snapshot_t {
  bool valid = false;

  uint64_t candidate_count = 0;
  uint32_t candidates_this_fragment = 0;

  photons_lap_science_population_snapshot_t accepted{};
  photons_lap_science_population_snapshot_t excluded{};
  photons_lap_science_reason_counts_snapshot_t exclusion_reasons{};

  // Two agreeing projected laps establish the initial raw-cycle lineage.
  bool predictor_valid = false;
  uint32_t predictor_cycles = 0;
  uint32_t gate_cycles = 0;
  uint32_t reject_streak = 0;
  uint32_t max_reject_streak = 0;

  bool seed_pending = false;
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


// One PHOTONS PPB population.  sample_count is accepted projected laps, not
// elapsed seconds.  A zero sample_count means that bucket is scientifically
// unavailable and must not be presented as a zero-PPB observation.
struct photons_fragment_ppb_value_snapshot_t {
  uint64_t sample_count = 0;
  double ppb = 0.0;
};


// Instrument-owned rolling/lifetime PPB populations.  LANTERN CAMP PPB is a
// separate firmware-authored campaign population below, matching CLOCKS' split
// between always-on instrument buckets and campaign-relative statistics.
struct photons_fragment_ppb_buckets_snapshot_t {
  photons_fragment_ppb_value_snapshot_t minute_10{};
  photons_fragment_ppb_value_snapshot_t minute_60{};
  photons_fragment_ppb_value_snapshot_t hour_8{};
  photons_fragment_ppb_value_snapshot_t hour_24{};
  photons_fragment_ppb_value_snapshot_t total{};
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
  uint64_t standard_lap_ps = 0;
  uint64_t lap_count = 0;
  uint64_t total_lap_gnss_ns = 0;
  double mean_lap_ns = 0.0;
  photons_fragment_welford_snapshot_t lap_time_welford{};
  photons_fragment_ppb_buckets_snapshot_t ppb_buckets{};

  // Recovery-only Better-Buckets witnesses.  update_count is the logical
  // rolling chronology; current_sequence identifies the latest lawful endpoint.
  uint32_t rolling_ppb_current_sequence = 0;
  bool rolling_ppb_endpoint_admitted = false;
  bool rolling_ppb_interval_advanced = false;

  // Self-contained current-window proof plus exact ring append testimony.
  photons_fragment_ppb_checkpoint_delta_snapshot_t rolling_ppb_checkpoint{};
};


// Firmware-authored LANTERN campaign measurement.  Pi owns campaign lifecycle,
// durable identity, and baseline relationships; PHOTONS owns the exact recording
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
  photons_fragment_ppb_value_snapshot_t ppb{};
};


// Baseline comparison is intentionally present in the schema before baseline
// control exists.  No fake zero baseline is authored: present/residual_valid
// remain false until a future LANTERN/baseline command supplies provenance.
struct photons_fragment_baseline_snapshot_t {
  bool present = false;
  bool residual_valid = false;
  double baseline_mean_lap_ns = 0.0;
  double mean_residual_ps = 0.0;
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
// Pi may add durable campaign ID/baseline provenance but never recomputes CAMP.
struct photons_fragment_snapshot_t {
  bool snapshot_ok = false;
  bool valid = false;
  uint32_t sequence = 0;
  uint32_t publish_count = 0;
  uint64_t fragment_period_ns = 0;

  uint32_t edge_count_total = 0;
  uint32_t edges_this_fragment = 0;

  uint32_t train_count = 0;
  uint32_t dead_lap_count = 0;
  uint64_t raw_lap_count = 0;
  uint32_t projected_laps_this_fragment = 0;

  photons_fragment_raw_cycles_snapshot_t raw_cycles{};
  photons_fragment_projection_snapshot_t projection{};
  photons_lap_science_snapshot_t science{};
  photons_fragment_stats_snapshot_t stats{};
  photons_fragment_campaign_snapshot_t campaign{};
  photons_fragment_baseline_snapshot_t baseline{};
  photons_fragment_recovery_snapshot_t recovery{};

  // Snapshot of process_interrupt's PHOTODIODE lane testimony.
  uint32_t interrupt_irq_count = 0;
  uint32_t interrupt_callback_count = 0;
  uint32_t interrupt_callback_missing_count = 0;
  uint32_t interrupt_inactive_edge_count = 0;
  uint32_t interrupt_source_pin = 0;
  uint32_t interrupt_last_callback_wall_cycles = 0;
  uint32_t interrupt_max_callback_wall_cycles = 0;
};

// Initialize PHOTONS runtime state, subscribe to the process_interrupt
// PHOTODIODE lane, start that lane, arm the 1 Hz toy fragment publisher, and
// start the temporary always-on detector emulator.
// Must run after process_interrupt_init() and timepop_init().
void process_photons_init(void);

// Register the PHOTONS process command surface.
void process_photons_register(void);

// Foreground/test accessors.  They return coherent snapshots without changing
// PHOTONS state.  The toy accessors are retained temporarily for compatibility;
// new consumers should use photons_fragment_snapshot().
bool photons_toy_capture_snapshot(photons_toy_capture_t* out);
bool photons_toy_fragment_snapshot(photons_toy_fragment_t* out);
bool photons_fragment_snapshot(photons_fragment_snapshot_t* out);
