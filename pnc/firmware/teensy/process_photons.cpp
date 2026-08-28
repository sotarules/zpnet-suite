#include "process_photons.h"

#include "config.h"
#include "events.h"
#include "payload.h"
#include "process.h"
#include "process_interrupt.h"
#include "publish.h"
#include "timepop.h"
#include "time.h"
#include "util.h"

#include <Arduino.h>
#include <Wire.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ============================================================================
// PHOTONS scaffold doctrine
// ============================================================================
//
// process_interrupt owns the physical PD200T comparator interrupt and the
// first-instruction DWT coordinate.
//
// PHOTONS consumes that immutable edge fact through the specialized high-rate
// PHOTODIODE subscription. ISR callbacks remain intentionally tiny: first-edge
// race/pulse latches plus scalar capture updates only; no Payload, publication,
// CLOCKS/TIME call, floating-point statistics, ADC work, or TimePop mutation.
//
// Once per second a foreground TimePop callback drains completed physical race
// records, projects each DWT endpoint pair through its launch-captured PPS/VCLOCK
// GNSS ruler, advances canonical Welford/ratio state, and publishes
// PHOTONS_FRAGMENT_V1. Physical/raw evidence is never erased by interpretation.
// ============================================================================

static constexpr uint64_t PHOTONS_FRAGMENT_PERIOD_NS = 1000000000ULL;
static constexpr uint64_t PHOTONS_NS_PER_SECOND = 1000000000ULL;

// Interim 1 kHz race handoff. 2048 entries provide a little over two seconds
// of worst-case headroom when every cadence cell completes. The future fast
// switch will require a batch sufficient-statistics path rather than scaling
// this raw ring. Overflow is explicit science data loss and remains fatal to
// the current statistical/campaign custody.
static constexpr uint32_t PHOTONS_LAP_RING_CAPACITY = 2048U;
static constexpr uint32_t PHOTONS_LAP_RING_MASK =
    PHOTONS_LAP_RING_CAPACITY - 1U;
static_assert((PHOTONS_LAP_RING_CAPACITY &
               (PHOTONS_LAP_RING_CAPACITY - 1U)) == 0U,
              "PHOTONS lap ring capacity must be a power of two");

static constexpr uint64_t PHOTONS_PROJECTION_MAX_AGE_NS = 3000000000ULL;

// Science admission is structural, not aesthetic.  Mirror CLOCKS philosophy:
// ordinary timing variation remains science; only truly exceptional interval
// injuries are excluded.  Use a deliberately broad +/-10% gate around the
// last accepted raw lap so the court acts as an integrity gate, not a quality
// filter.  Gate = max(last science-accepted raw lap / 10, 64 cycles).
static constexpr uint32_t PHOTONS_SCIENCE_GATE_DIVISOR = 10U;
static constexpr uint32_t PHOTONS_SCIENCE_GATE_MIN_CYCLES = 64U;

// CLOCKS-shaped Better-Buckets geometry.  Keep exact one-second endpoints for
// the trailing ten minutes and first-admitted-per-minute endpoints for the
// longer windows.  Every bucket still evaluates against the live current-second
// endpoint, so only the historical edge is minute-granular.  A recovery-installed
// literal suffix may contain an explicit chronology gap; window publication must
// prove that its requested old edge lies inside the newest contiguous suffix
// rather than silently relabeling a shorter available baseline.
static constexpr uint32_t PHOTONS_PPB_MINUTE_10_SECONDS = 10U * 60U;
static constexpr uint32_t PHOTONS_PPB_MINUTE_60_SECONDS = 60U * 60U;
static constexpr uint32_t PHOTONS_PPB_HOUR_8_SECONDS = 8U * 60U * 60U;
static constexpr uint32_t PHOTONS_PPB_HOUR_24_SECONDS = 24U * 60U * 60U;
static constexpr uint32_t PHOTONS_PPB_SECOND_CAPACITY =
    PHOTONS_PPB_MINUTE_10_SECONDS + 1U;
static constexpr uint32_t PHOTONS_PPB_MINUTE_CAPACITY = 24U * 60U + 2U;

// Durable recovery uses bounded, typed chunks so command Payload size remains
// predictable.  Aggregate state is installed only by RECOVERY_COMMIT after both
// histories have arrived and passed the firmware court.
static constexpr uint32_t PHOTONS_RECOVERY_SCHEMA_VERSION = 1U;
static constexpr uint32_t PHOTONS_RECOVERY_CHUNK_MAX_ENDPOINTS = 4U;


// ============================================================================
// Optical device control / telemetry
// ============================================================================
//
// PHOTONS is the umbrella owner for photon-producing and photon-detecting
// devices. process_interrupt still owns PD200T comparator edge custody.
//
// Laser driver: MP5491 / EV5491-C-00A
//
static constexpr uint8_t MP5491_ADDR = 0x66;

static constexpr uint8_t MP5491_REG_CTL0    = 0x00;
static constexpr uint8_t MP5491_REG_CTL1    = 0x01;
static constexpr uint8_t MP5491_REG_ID1_MSB = 0x07;
static constexpr uint8_t MP5491_REG_ID1_LSB = 0x08;

static constexpr uint8_t MP5491_SYSEN_BIT  = 0x80;
static constexpr uint8_t MP5491_ID_EN_BIT  = 0x80;
static constexpr uint8_t MP5491_ID1_EN_BIT = 0x08;

// Existing authoritative laser setting: ID1 = 20 mA, 0.25 mA / LSB.
static constexpr uint8_t PHOTONS_LASER_ID1_CURRENT_MSB = 0x14;
static constexpr uint8_t PHOTONS_LASER_ID1_CURRENT_LSB = 0x00;

static constexpr float PHOTONS_LASER_EMIT_THRESHOLD_V = 0.75f;
static constexpr uint64_t PHOTONS_PULSE_DEFAULT_NS = 1000ULL;

// Interim real-race geometry. One 1 ms cell contains one 91 us LD_ON pulse
// followed by a long quiet receive/recovery interval. The actual LD_ON falling
// edge is the temporary launch surrogate until the fast-switch daughterboard
// supplies a true fast optical launch edge.
static constexpr uint64_t PHOTONS_RACE_CADENCE_NS = 1000000ULL;
static constexpr uint32_t PHOTONS_RACE_CADENCE_HZ = 1000U;
static constexpr uint64_t PHOTONS_RACE_PULSE_NS = 91000ULL;
static_assert(PHOTONS_NS_PER_SECOND / PHOTONS_RACE_CADENCE_NS ==
                  PHOTONS_RACE_CADENCE_HZ,
              "PHOTONS race cadence constant mismatch");


// ============================================================================
// Real single-pass race engine
// ============================================================================
//
// TimePop owns the 1 kHz launch cadence. process_interrupt owns pin-34 DWT
// custody. The PHOTODIODE callback only latches the first eligible receive edge;
// the following cadence cell finalizes that race into the raw handoff ring.
// No synthetic detector edges or synthetic lap intervals remain.
// ============================================================================

struct photons_race_launch_slot_t {
  bool     valid = false;
  uint32_t sequence = 0U;
  uint32_t ld_on_start_dwt = 0U;
  bool     launch_surrogate_valid = false;
  uint32_t launch_surrogate_dwt = 0U;
  uint32_t target_high_cycles = 0U;
  uint32_t cadence_cell_cycles = 0U;
  uint32_t pulse_wall_cycles = 0U;

  bool     anchor_valid = false;
  uint32_t anchor_dwt_at_pps_vclock = 0U;
  uint32_t anchor_dwt_cycles_per_second = 0U;
  uint32_t anchor_pps_count = 0U;
};

struct photons_race_receive_value_t {
  bool     seen = false;
  uint32_t race_sequence = 0U;
  uint32_t edge_sequence = 0U;
  uint32_t pps_sequence = 0U;
  uint32_t finish_dwt = 0U;
};

struct photons_race_receive_state_t {
  volatile uint32_t generation = 0U;
  photons_race_receive_value_t value{};
};

struct photons_race_runtime_t {
  bool initialized = false;
  timepop_handle_t cadence_timer = TIMEPOP_INVALID_HANDLE;
  uint32_t sequence = 0U;
  uint64_t cadence_tick_count = 0ULL;
  uint64_t attempt_count = 0ULL;
  uint64_t completed_count = 0ULL;
  uint64_t missed_count = 0ULL;
  uint64_t skipped_not_quiet_count = 0ULL;
  uint64_t skipped_projection_count = 0ULL;
  uint64_t invalid_endpoint_count = 0ULL;
  uint64_t enqueue_failure_count = 0ULL;
};

static photons_race_runtime_t g_photons_race{};
static photons_race_launch_slot_t g_photons_race_launch{};
static photons_race_receive_state_t g_photons_race_receive{};
static volatile uint32_t g_race_armed_sequence = 0U;

static void photons_race_cadence_tick(
    timepop_ctx_t* ctx,
    timepop_diag_t* diag,
    void* user_data);

struct photons_device_snapshot_t {
  bool     laser_enabled = false;
  uint16_t laser_id1_raw = 0;
  float    laser_id1_current_ma = 0.0f;
  uint16_t laser_monitor_raw = 0;
  float    laser_monitor_v = 0.0f;
  bool     laser_emitting = false;

  int      photodiode_edge_level = 0;
  uint16_t photodiode_analog_raw = 0;
  float    photodiode_analog_v = 0.0f;
};

// ============================================================================
// Canonical raw-lap handoff / projection / statistics state
// ============================================================================

static inline void photons_memory_barrier(void) {
  __asm__ volatile("dmb" ::: "memory");
}


struct photons_projection_anchor_cache_t {
  volatile uint32_t seq = 0U;
  bool valid = false;
  uint32_t dwt_at_pps_vclock = 0U;
  uint32_t dwt_cycles_per_second = 0U;
  uint32_t pps_count = 0U;
};


struct photons_projection_anchor_value_t {
  bool valid = false;
  uint32_t dwt_at_pps_vclock = 0U;
  uint32_t dwt_cycles_per_second = 0U;
  uint32_t pps_count = 0U;
};


struct photons_raw_lap_record_t {
  uint32_t start_dwt = 0U;
  uint32_t end_dwt = 0U;
  uint32_t raw_cycles = 0U;
  uint32_t pps_sequence = 0U;

  bool anchor_valid = false;
  uint32_t anchor_dwt_at_pps_vclock = 0U;
  uint32_t anchor_dwt_cycles_per_second = 0U;
  uint32_t anchor_pps_count = 0U;
};

static_assert(
    sizeof(photons_raw_lap_record_t) * PHOTONS_LAP_RING_CAPACITY <=
        64U * 1024U,
    "PHOTONS raw-lap bring-up ring exceeds 64 KiB RAM2 budget");


struct photons_welford_state_t {
  uint64_t n = 0ULL;
  double mean = 0.0;
  double m2 = 0.0;
  double min_val = 0.0;
  double max_val = 0.0;
};


// One cumulative accepted-population endpoint.  Differences between two
// endpoints are exact sufficient statistics for a PHOTONS PPB population; no
// one-second means are averaged and no raw laps need to be retained.
struct photons_ppb_endpoint_t {
  uint32_t sequence = 0U;
  uint64_t lap_count = 0ULL;
  uint64_t total_lap_gnss_ns = 0ULL;
};

static_assert(
    sizeof(photons_ppb_endpoint_t) *
            (PHOTONS_PPB_SECOND_CAPACITY + PHOTONS_PPB_MINUTE_CAPACITY) <=
        64U * 1024U,
    "PHOTONS Better-Buckets history exceeds 64 KiB RAM2 budget");


struct photons_lap_science_candidate_t {
  bool valid = false;
  uint64_t candidate_index = 0ULL;
  uint32_t pps_sequence = 0U;
  uint32_t raw_cycles = 0U;
  uint64_t lap_gnss_ns = 0ULL;
};


static photons_projection_anchor_cache_t g_projection_anchor_cache{};
static photons_raw_lap_record_t
    g_raw_lap_ring[PHOTONS_LAP_RING_CAPACITY] DMAMEM = {};
static volatile uint32_t g_raw_lap_ring_write = 0U;
static volatile uint32_t g_raw_lap_ring_read = 0U;
static volatile uint32_t g_raw_lap_ring_overflow_count = 0U;
static volatile bool g_raw_lap_ring_data_loss = false;

static photons_fragment_raw_cycles_snapshot_t g_raw_cycles_state{};
static photons_fragment_projection_snapshot_t g_projection_state{};
// Runtime exclusion authority lives only in exclusion_reasons + Welford witnesses.
// excluded.count/count_this_fragment are left non-authoritative here and materialized
// only into immutable publication/report snapshots.
static photons_lap_science_snapshot_t g_photons_lap_science_state{};
static photons_lap_science_candidate_t g_photons_lap_science_seed_pending{};

// Accepted projected-lap time remains the canonical science population used by
// fragment.stats.  The parallel raw-cycle and excluded populations are courtroom
// testimony only; they never feed the scientific mean or predictor.
static photons_welford_state_t g_lap_time_welford{};
static photons_welford_state_t g_accepted_raw_cycles_welford{};
static photons_welford_state_t g_excluded_raw_cycles_welford{};
static photons_welford_state_t g_excluded_lap_time_welford{};
// Resettable always-on statistical epoch numerator.  STATS_RESET rebases this
// population without touching the monotonic campaign-custody ledgers below.
static uint64_t g_total_lap_gnss_ns = 0ULL;
static uint32_t g_photons_stats_reset_count = 0U;
static bool g_photons_stats_reset_pending = false;
static uint32_t g_photons_stats_reset_request_count = 0U;
static uint32_t g_photons_stats_reset_commit_count = 0U;

// Monotonic accepted-lap custody survives STATS_RESET.  LANTERN campaign origins
// and CAMP N/T are differences on this lineage, so an operator statistics reset
// cannot move or invalidate an active recording boundary.
static uint64_t g_photons_custody_lap_count = 0ULL;
static uint64_t g_photons_custody_total_lap_gnss_ns = 0ULL;

// Operator-authored lap reference. The exact authority is integer femtoseconds;
// standard_lap_ps remains only a deprecated whole-ps compatibility mirror.
// Re-referencing does not mutate physical N/T, Welford, or Better-Buckets custody.
static bool g_standard_lap_configured = false;
static uint64_t g_lap_baseline_fs = 0ULL;
static uint64_t g_standard_lap_ps = 0ULL;

// LANTERN lifecycle mirrors CLOCKS Beta conceptually: Pi owns the campaign
// lifecycle/name, while Teensy snapshots its own already-running cumulative
// accepted-lap state at a published fragment boundary and authors CAMP PPB.
enum class photons_campaign_state_t : uint8_t {
  STOPPED = 0,
  START_PENDING,
  ACTIVE,
  STOP_PENDING,
  FLASH_CUT_PENDING,
};

static photons_campaign_state_t g_photons_campaign_state =
    photons_campaign_state_t::STOPPED;
static char g_photons_campaign_name[64] = {0};
static uint64_t g_photons_campaign_origin_lap_count = 0ULL;
static uint64_t g_photons_campaign_origin_total_lap_gnss_ns = 0ULL;
static uint32_t g_photons_campaign_start_after_sequence = 0U;
static uint32_t g_photons_campaign_public_count = 0U;
static uint32_t g_photons_campaign_start_request_count = 0U;
static uint32_t g_photons_campaign_start_commit_count = 0U;
static uint32_t g_photons_campaign_stop_request_count = 0U;
static uint32_t g_photons_campaign_stop_commit_count = 0U;
static char g_photons_flash_cut_campaign_name[64] = {0};
static uint32_t g_photons_flash_cut_request_count = 0U;
static uint32_t g_photons_flash_cut_commit_count = 0U;
static uint32_t g_photons_flash_cut_reject_count = 0U;

// Recovery is a boot-local transaction.  A rebooted Teensy remains held after
// STANDARD_LAP_NS installation until the Pi either commits durable state or
// explicitly declares a cold start.  A Pi-only restart sees publication_started
// and reattaches without touching healthy live state.
struct photons_recovery_protocol_t {
  bool active = false;
  uint32_t generation = 0U;
  uint32_t expected_second_count = 0U;
  uint32_t accepted_second_count = 0U;
  uint32_t expected_minute_count = 0U;
  uint32_t accepted_minute_count = 0U;
  bool previous_second_valid = false;
  photons_ppb_endpoint_t previous_second{};
  bool previous_minute_valid = false;
  photons_ppb_endpoint_t previous_minute{};
};

struct photons_recovery_runtime_t {
  bool publication_started = false;
  bool restored = false;
  bool proof_pending = false;
  bool proof_committed = false;
  bool proof_advanced_published = false;
  uint32_t generation = 0U;
  uint32_t source_sequence = 0U;
  uint32_t source_publish_count = 0U;
  uint32_t source_reset_count = 0U;
  uint32_t source_update_count = 0U;
  uint64_t source_lap_count = 0ULL;
  uint64_t source_total_lap_gnss_ns = 0ULL;
  uint64_t source_custody_lap_count = 0ULL;
  uint64_t source_custody_total_lap_gnss_ns = 0ULL;
  uint32_t proof_sequence = 0U;
  uint32_t proof_update_count = 0U;
  uint32_t dropped_pending_seed_count = 0U;
  uint32_t begin_count = 0U;
  uint32_t chunk_count = 0U;
  uint32_t commit_count = 0U;
  uint32_t abort_count = 0U;
  uint32_t cold_start_count = 0U;
  uint32_t proof_ack_count = 0U;
  uint32_t reject_count = 0U;
};

static photons_recovery_protocol_t g_photons_recovery_protocol{};
static photons_recovery_runtime_t g_photons_recovery{};

static photons_ppb_endpoint_t
    g_photons_ppb_seconds[PHOTONS_PPB_SECOND_CAPACITY] DMAMEM = {};
static photons_ppb_endpoint_t
    g_photons_ppb_minutes[PHOTONS_PPB_MINUTE_CAPACITY] DMAMEM = {};
static uint32_t g_photons_ppb_seconds_head = 0U;
static uint32_t g_photons_ppb_seconds_count = 0U;
static uint32_t g_photons_ppb_minutes_head = 0U;
static uint32_t g_photons_ppb_minutes_count = 0U;
static uint32_t g_photons_ppb_last_minute_key = 0U;
static bool g_photons_ppb_previous_endpoint_valid = false;
static photons_ppb_endpoint_t g_photons_ppb_previous_endpoint{};
static uint32_t g_photons_stats_update_count = 0U;
// Physical publication chronology pairs with statistical update chronology in
// the exact recovery-successor court below.
static uint32_t g_fragment_sequence = 0U;
static uint32_t g_photons_ppb_current_sequence = 0U;
static bool g_photons_ppb_endpoint_admitted = false;
static bool g_photons_ppb_interval_advanced = false;
static bool g_photons_ppb_last_minute_appended = false;

static bool g_previous_fragment_mean_cycles_valid = false;
static double g_previous_fragment_mean_cycles = 0.0;


static void photons_welford_reset(photons_welford_state_t& w) {
  w.n = 0ULL;
  w.mean = 0.0;
  w.m2 = 0.0;
  w.min_val = 1.0e300;
  w.max_val = -1.0e300;
}


static void photons_welford_update(photons_welford_state_t& w, double sample) {
  w.n++;
  const double d1 = sample - w.mean;
  w.mean += d1 / (double)w.n;
  const double d2 = sample - w.mean;
  w.m2 += d1 * d2;
  if (sample < w.min_val) w.min_val = sample;
  if (sample > w.max_val) w.max_val = sample;
}


static double photons_welford_stddev(const photons_welford_state_t& w) {
  return (w.n >= 2ULL) ? sqrt(w.m2 / (double)(w.n - 1ULL)) : 0.0;
}


static double photons_welford_stderr(const photons_welford_state_t& w) {
  return (w.n >= 2ULL)
      ? photons_welford_stddev(w) / sqrt((double)w.n)
      : 0.0;
}


static photons_fragment_welford_snapshot_t photons_welford_snapshot(
    const photons_welford_state_t& w) {
  photons_fragment_welford_snapshot_t out{};
  out.n = w.n;
  out.mean = w.mean;
  out.m2 = w.m2;
  out.stddev = photons_welford_stddev(w);
  out.stderr_value = photons_welford_stderr(w);
  out.min = (w.n != 0ULL) ? w.min_val : 0.0;
  out.max = (w.n != 0ULL) ? w.max_val : 0.0;
  return out;
}


template <size_t N>
static void photons_ppb_ring_append(photons_ppb_endpoint_t (&ring)[N],
                                    uint32_t& head,
                                    uint32_t& count,
                                    const photons_ppb_endpoint_t& endpoint) {
  static_assert(N > 0U, "PHOTONS PPB ring must not be empty");
  if (count < (uint32_t)N) {
    const uint32_t index = (head + count) % (uint32_t)N;
    ring[index] = endpoint;
    count++;
    return;
  }

  ring[head] = endpoint;
  head = (head + 1U) % (uint32_t)N;
}


static uint32_t photons_ppb_minute_key(uint32_t sequence) {
  return sequence == 0U ? 0U : ((sequence - 1U) / 60U) + 1U;
}


template <size_t N>
static bool photons_ppb_ring_find_anchor(
    const photons_ppb_endpoint_t (&ring)[N],
    uint32_t head,
    uint32_t count,
    uint32_t target_sequence,
    uint32_t current_sequence,
    bool minute_history,
    photons_ppb_endpoint_t& out) {
  if (count > (uint32_t)N) __builtin_trap();
  if (count == 0U || current_sequence == 0U) return false;

  // A young statistical epoch may lawfully use exact zero as the old edge of a
  // window that reaches before epoch birth.  The zero endpoint is authoritative
  // aggregate testimony, so it remains usable even if a later observation gap
  // split the bounded history.
  if (target_sequence == 0U) {
    for (uint32_t offset = 0U; offset < count; offset++) {
      const photons_ppb_endpoint_t& candidate =
          ring[(head + offset) % (uint32_t)N];
      if (candidate.sequence == 0U) {
        out = candidate;
        return true;
      }
    }
    return false;
  }

  // Recovery may intentionally install an exact literal suffix after Pi missed
  // older producer appends.  A ring is therefore not automatically continuous
  // merely because its endpoints are monotonic.  Find the newest suffix whose
  // second identities (or minute keys) are adjacent all the way to the tail.
  // Windows whose target lies before that suffix are unavailable rather than
  // silently relabeled with a shorter baseline.
  uint32_t suffix_offset = 0U;
  bool previous_valid = false;
  uint32_t previous_sequence = 0U;

  for (uint32_t offset = 0U; offset < count; offset++) {
    const photons_ppb_endpoint_t& candidate =
        ring[(head + offset) % (uint32_t)N];

    bool follows = false;
    if (previous_valid) {
      if (minute_history) {
        follows =
            photons_ppb_minute_key(candidate.sequence) ==
            photons_ppb_minute_key(previous_sequence) + 1U;
      } else {
        follows = candidate.sequence == previous_sequence + 1U;
      }
    }

    if (!previous_valid || !follows) {
      suffix_offset = offset;
    }
    previous_valid = true;
    previous_sequence = candidate.sequence;
  }

  const photons_ppb_endpoint_t& coverage_start =
      ring[(head + suffix_offset) % (uint32_t)N];
  const bool target_covered = minute_history
      ? photons_ppb_minute_key(target_sequence) >=
            photons_ppb_minute_key(coverage_start.sequence)
      : target_sequence >= coverage_start.sequence;
  if (!target_covered) return false;

  for (uint32_t offset = suffix_offset; offset < count; offset++) {
    const photons_ppb_endpoint_t& candidate =
        ring[(head + offset) % (uint32_t)N];
    if (candidate.sequence >= target_sequence &&
        candidate.sequence < current_sequence) {
      out = candidate;
      return true;
    }
  }
  return false;
}


static void photons_ppb_windows_clear_history(void) {
  g_photons_ppb_seconds_head = 0U;
  g_photons_ppb_seconds_count = 0U;
  g_photons_ppb_minutes_head = 0U;
  g_photons_ppb_minutes_count = 0U;
  g_photons_ppb_last_minute_key = 0U;
  g_photons_ppb_previous_endpoint_valid = false;
  g_photons_ppb_previous_endpoint = photons_ppb_endpoint_t{};
  g_photons_ppb_current_sequence = 0U;
  g_photons_ppb_endpoint_admitted = false;
  g_photons_ppb_interval_advanced = false;
  g_photons_ppb_last_minute_appended = false;
}


static void photons_ppb_windows_seed_origin(void) {
  photons_ppb_windows_clear_history();

  const photons_ppb_endpoint_t origin{};
  photons_ppb_ring_append(
      g_photons_ppb_seconds,
      g_photons_ppb_seconds_head,
      g_photons_ppb_seconds_count,
      origin);
  photons_ppb_ring_append(
      g_photons_ppb_minutes,
      g_photons_ppb_minutes_head,
      g_photons_ppb_minutes_count,
      origin);
  g_photons_ppb_previous_endpoint = origin;
  g_photons_ppb_previous_endpoint_valid = true;
}


static void photons_recovery_protocol_clear(bool clear_histories) {
  g_photons_recovery_protocol = photons_recovery_protocol_t{};
  if (clear_histories) {
    photons_ppb_windows_clear_history();
  }
}


// Better-Buckets endpoints are cumulative chronology, not per-second samples.
// A lawful endpoint may therefore advance sequence while N/T remains exactly
// unchanged (including zero/zero before the first accepted lap).  The producer
// enforces the same invariant through delta_laps/delta_ns.
static bool photons_ppb_endpoint_population_consistent(
    const photons_ppb_endpoint_t& endpoint) {
  return (endpoint.lap_count == 0ULL) ==
         (endpoint.total_lap_gnss_ns == 0ULL);
}


static bool photons_recovery_stage_endpoint(bool minute_history,
                                            const photons_ppb_endpoint_t& endpoint) {
  photons_recovery_protocol_t& protocol = g_photons_recovery_protocol;
  if (!protocol.active) return false;

  uint32_t& accepted = minute_history
      ? protocol.accepted_minute_count
      : protocol.accepted_second_count;
  const uint32_t expected = minute_history
      ? protocol.expected_minute_count
      : protocol.expected_second_count;
  if (accepted >= expected) return false;

  bool& previous_valid = minute_history
      ? protocol.previous_minute_valid
      : protocol.previous_second_valid;
  photons_ppb_endpoint_t& previous = minute_history
      ? protocol.previous_minute
      : protocol.previous_second;

  if (!photons_ppb_endpoint_population_consistent(endpoint)) return false;
  if (endpoint.sequence == 0U &&
      (accepted != 0U || endpoint.lap_count != 0ULL)) {
    return false;
  }

  if (previous_valid) {
    if (endpoint.sequence <= previous.sequence ||
        endpoint.lap_count < previous.lap_count ||
        endpoint.total_lap_gnss_ns < previous.total_lap_gnss_ns) {
      return false;
    }
    if (minute_history && endpoint.sequence != 0U &&
        photons_ppb_minute_key(endpoint.sequence) <=
            photons_ppb_minute_key(previous.sequence)) {
      return false;
    }
  }

  if (minute_history) {
    photons_ppb_ring_append(
        g_photons_ppb_minutes,
        g_photons_ppb_minutes_head,
        g_photons_ppb_minutes_count,
        endpoint);
  } else {
    photons_ppb_ring_append(
        g_photons_ppb_seconds,
        g_photons_ppb_seconds_head,
        g_photons_ppb_seconds_count,
        endpoint);
  }

  previous = endpoint;
  previous_valid = true;
  accepted++;
  return true;
}


static double photons_observed_mean_fs(uint64_t total_lap_gnss_ns,
                                       uint64_t lap_count) {
  if (lap_count == 0ULL || total_lap_gnss_ns == 0ULL) __builtin_trap();

  // Divide first so TOTAL remains numerically well behaved after years of
  // accumulation. The quotient/remainder form avoids multiplying the lifetime
  // nanosecond numerator by 1,000,000 in uint64_t.
  const uint64_t whole_ns = total_lap_gnss_ns / lap_count;
  const uint64_t remainder_ns = total_lap_gnss_ns % lap_count;
  return (double)whole_ns * 1000000.0 +
      ((double)remainder_ns * 1000000.0) / (double)lap_count;
}




static double photons_residual_ns_from_population(
    uint64_t total_lap_gnss_ns, uint64_t lap_count) {
  if (!g_standard_lap_configured || g_lap_baseline_fs == 0ULL) {
    __builtin_trap();
  }
  const double observed_mean_fs =
      photons_observed_mean_fs(total_lap_gnss_ns, lap_count);
  return (observed_mean_fs - (double)g_lap_baseline_fs) / 1000000.0;
}


static double photons_ppb_from_population(uint64_t total_lap_gnss_ns,
                                          uint64_t lap_count) {
  // PHOTONS uses the system-wide nanosecond coordinate: one nanosecond of
  // mean baseline residual is one displayed PPB. The name is retained for
  // Better-Buckets symmetry with CLOCKS; there is no fractional-lap scaling.
  return photons_residual_ns_from_population(total_lap_gnss_ns, lap_count);
}


static photons_fragment_ppb_value_snapshot_t photons_ppb_bucket_between(
    const photons_ppb_endpoint_t& anchor,
    const photons_ppb_endpoint_t& current) {
  if (current.sequence <= anchor.sequence ||
      current.lap_count < anchor.lap_count ||
      current.total_lap_gnss_ns < anchor.total_lap_gnss_ns) {
    __builtin_trap();
  }

  const uint64_t lap_count = current.lap_count - anchor.lap_count;
  const uint64_t total_ns =
      current.total_lap_gnss_ns - anchor.total_lap_gnss_ns;
  if (lap_count == 0ULL) {
    if (total_ns != 0ULL) __builtin_trap();
    return photons_fragment_ppb_value_snapshot_t{};
  }
  if (total_ns == 0ULL) __builtin_trap();

  photons_fragment_ppb_value_snapshot_t out{};
  out.sample_count = lap_count;
  out.ppb = photons_ppb_from_population(total_ns, lap_count);
  out.residual_ns = photons_residual_ns_from_population(total_ns, lap_count);
  return out;
}


static photons_fragment_ppb_endpoint_snapshot_t photons_ppb_endpoint_snapshot(
    const photons_ppb_endpoint_t& endpoint) {
  photons_fragment_ppb_endpoint_snapshot_t out{};
  out.sequence = endpoint.sequence;
  out.lap_count = endpoint.lap_count;
  out.total_lap_gnss_ns = endpoint.total_lap_gnss_ns;
  return out;
}


// Read-only full-ring custody export for a surviving producer.  A restarted Pi
// may reacquire the exact endpoint rings that remain authoritative in PHOTONS
// RAM without freezing or mutating the instrument.  Cursor identities are the
// producer's rolling sequences, so a newer append cannot rewrite an endpoint
// already learned by the Pi.
template <size_t N>
static uint32_t photons_ppb_ring_oldest_sequence(
    const photons_ppb_endpoint_t (&ring)[N],
    uint32_t head,
    uint32_t count) {
  if (count == 0U || count > (uint32_t)N) return 0U;
  return ring[head].sequence;
}


template <size_t N>
static uint32_t photons_ppb_ring_newest_sequence(
    const photons_ppb_endpoint_t (&ring)[N],
    uint32_t head,
    uint32_t count) {
  if (count == 0U || count > (uint32_t)N) return 0U;
  const uint32_t newest =
      (head + count - 1U) % (uint32_t)N;
  return ring[newest].sequence;
}


template <size_t N>
static uint32_t photons_ppb_ring_export_chunk(
    const photons_ppb_endpoint_t (&ring)[N],
    uint32_t head,
    uint32_t ring_count,
    uint32_t before_sequence,
    photons_ppb_endpoint_t* out,
    uint32_t capacity) {
  if (!out || capacity == 0U || ring_count > (uint32_t)N) return 0U;

  uint32_t written = 0U;
  for (uint32_t age = 0U; age < ring_count && written < capacity; ++age) {
    const uint32_t index =
        (head + ring_count - 1U - age) % (uint32_t)N;
    const photons_ppb_endpoint_t& endpoint = ring[index];
    if (before_sequence != 0U && endpoint.sequence >= before_sequence) {
      continue;
    }
    out[written++] = endpoint;
  }
  return written;
}


template <size_t N>
static photons_fragment_ppb_window_proof_snapshot_t photons_ppb_window_proof(
    const photons_ppb_endpoint_t (&ring)[N],
    uint32_t head,
    uint32_t count,
    uint32_t window_seconds,
    bool minute_history,
    const photons_ppb_endpoint_t& current) {
  photons_fragment_ppb_window_proof_snapshot_t out{};
  const uint32_t target_sequence =
      current.sequence > window_seconds
          ? current.sequence - window_seconds
          : 0U;

  photons_ppb_endpoint_t anchor{};
  if (!photons_ppb_ring_find_anchor(
          ring, head, count, target_sequence, current.sequence,
          minute_history, anchor)) {
    return out;
  }

  const photons_fragment_ppb_value_snapshot_t value =
      photons_ppb_bucket_between(anchor, current);
  if (value.sample_count == 0ULL) return out;

  out.valid = true;
  out.sample_count = value.sample_count;
  out.anchor = photons_ppb_endpoint_snapshot(anchor);
  return out;
}


static photons_fragment_ppb_checkpoint_delta_snapshot_t
photons_ppb_checkpoint_delta_snapshot(void) {
  photons_fragment_ppb_checkpoint_delta_snapshot_t out{};
  if (!g_photons_ppb_endpoint_admitted ||
      !g_photons_ppb_previous_endpoint_valid) {
    return out;
  }

  const photons_ppb_endpoint_t current = g_photons_ppb_previous_endpoint;
  if (current.sequence == 0U ||
      current.sequence != g_photons_stats_update_count ||
      current.sequence != g_photons_ppb_current_sequence ||
      current.lap_count != g_lap_time_welford.n ||
      current.total_lap_gnss_ns != g_total_lap_gnss_ns ||
      g_photons_ppb_seconds_count == 0U ||
      g_photons_ppb_minutes_count == 0U) {
    __builtin_trap();
  }

  out.valid = true;
  out.rolling_sequence = current.sequence;
  out.second_count = g_photons_ppb_seconds_count;
  out.minute_count = g_photons_ppb_minutes_count;
  out.last_minute_key = g_photons_ppb_last_minute_key;
  out.current = photons_ppb_endpoint_snapshot(current);

  // The resettable PHOTONS statistical epoch always begins at exact zero N/T.
  // This producer-authored origin remains truthful even after the bounded rings
  // have aged it out; it is not reconstructed from PostgreSQL history.
  const photons_ppb_endpoint_t origin{};
  out.origin_valid = true;
  out.origin = photons_ppb_endpoint_snapshot(origin);

  out.minute_10 = photons_ppb_window_proof(
      g_photons_ppb_seconds,
      g_photons_ppb_seconds_head,
      g_photons_ppb_seconds_count,
      PHOTONS_PPB_MINUTE_10_SECONDS,
      false,
      current);
  out.minute_60 = photons_ppb_window_proof(
      g_photons_ppb_minutes,
      g_photons_ppb_minutes_head,
      g_photons_ppb_minutes_count,
      PHOTONS_PPB_MINUTE_60_SECONDS,
      true,
      current);
  out.hour_8 = photons_ppb_window_proof(
      g_photons_ppb_minutes,
      g_photons_ppb_minutes_head,
      g_photons_ppb_minutes_count,
      PHOTONS_PPB_HOUR_8_SECONDS,
      true,
      current);
  out.hour_24 = photons_ppb_window_proof(
      g_photons_ppb_minutes,
      g_photons_ppb_minutes_head,
      g_photons_ppb_minutes_count,
      PHOTONS_PPB_HOUR_24_SECONDS,
      true,
      current);

  out.second_append_valid = true;
  out.second_append = out.current;
  out.minute_append_valid = g_photons_ppb_last_minute_appended;
  if (out.minute_append_valid) {
    out.minute_append = out.current;
  }
  return out;
}


static void photons_ppb_windows_note_endpoint(uint32_t sequence,
                                              bool admitted,
                                              uint64_t lap_count,
                                              uint64_t total_lap_gnss_ns) {
  g_photons_ppb_endpoint_admitted = admitted;
  g_photons_ppb_interval_advanced = false;
  g_photons_ppb_last_minute_appended = false;

  if (!admitted) {
    // Loss of instrument custody is a hard rolling-history boundary.  Do not
    // retain an older anchor that a future implementation could bridge across.
    photons_ppb_windows_clear_history();
    return;
  }
  if (sequence == 0U) __builtin_trap();

  photons_ppb_endpoint_t endpoint{};
  endpoint.sequence = sequence;
  endpoint.lap_count = lap_count;
  endpoint.total_lap_gnss_ns = total_lap_gnss_ns;

  if (g_photons_ppb_previous_endpoint_valid) {
    const photons_ppb_endpoint_t& previous = g_photons_ppb_previous_endpoint;
    if (endpoint.sequence != previous.sequence + 1U ||
        endpoint.lap_count < previous.lap_count ||
        endpoint.total_lap_gnss_ns < previous.total_lap_gnss_ns) {
      __builtin_trap();
    }

    const uint64_t delta_laps = endpoint.lap_count - previous.lap_count;
    const uint64_t delta_ns =
        endpoint.total_lap_gnss_ns - previous.total_lap_gnss_ns;
    if (delta_laps == 0ULL) {
      if (delta_ns != 0ULL) __builtin_trap();
    } else {
      if (delta_ns == 0ULL) __builtin_trap();
      g_photons_ppb_interval_advanced = true;
    }
  }

  photons_ppb_ring_append(
      g_photons_ppb_seconds,
      g_photons_ppb_seconds_head,
      g_photons_ppb_seconds_count,
      endpoint);

  const uint32_t minute_key = photons_ppb_minute_key(sequence);
  if (minute_key != g_photons_ppb_last_minute_key) {
    photons_ppb_ring_append(
        g_photons_ppb_minutes,
        g_photons_ppb_minutes_head,
        g_photons_ppb_minutes_count,
        endpoint);
    g_photons_ppb_last_minute_key = minute_key;
    g_photons_ppb_last_minute_appended = true;
  }

  g_photons_ppb_previous_endpoint = endpoint;
  g_photons_ppb_previous_endpoint_valid = true;
  g_photons_ppb_current_sequence = sequence;
}


template <size_t N>
static photons_fragment_ppb_value_snapshot_t photons_ppb_window_snapshot(
    const photons_ppb_endpoint_t (&ring)[N],
    uint32_t head,
    uint32_t count,
    uint32_t window_seconds,
    bool minute_history,
    const photons_ppb_endpoint_t& current) {
  const uint32_t target_sequence =
      current.sequence > window_seconds
          ? current.sequence - window_seconds
          : 0U;
  photons_ppb_endpoint_t anchor{};
  if (!photons_ppb_ring_find_anchor(
          ring, head, count, target_sequence, current.sequence,
          minute_history, anchor)) {
    return photons_fragment_ppb_value_snapshot_t{};
  }
  return photons_ppb_bucket_between(anchor, current);
}


static photons_fragment_ppb_buckets_snapshot_t photons_ppb_buckets_snapshot(void) {
  photons_fragment_ppb_buckets_snapshot_t out{};
  if (!g_photons_ppb_endpoint_admitted ||
      !g_photons_ppb_previous_endpoint_valid) {
    return out;
  }

  const photons_ppb_endpoint_t current = g_photons_ppb_previous_endpoint;
  if (current.sequence != g_photons_stats_update_count ||
      current.lap_count != g_lap_time_welford.n ||
      current.total_lap_gnss_ns != g_total_lap_gnss_ns) {
    __builtin_trap();
  }

  out.minute_10 = photons_ppb_window_snapshot(
      g_photons_ppb_seconds,
      g_photons_ppb_seconds_head,
      g_photons_ppb_seconds_count,
      PHOTONS_PPB_MINUTE_10_SECONDS,
      false,
      current);
  out.minute_60 = photons_ppb_window_snapshot(
      g_photons_ppb_minutes,
      g_photons_ppb_minutes_head,
      g_photons_ppb_minutes_count,
      PHOTONS_PPB_MINUTE_60_SECONDS,
      true,
      current);
  out.hour_8 = photons_ppb_window_snapshot(
      g_photons_ppb_minutes,
      g_photons_ppb_minutes_head,
      g_photons_ppb_minutes_count,
      PHOTONS_PPB_HOUR_8_SECONDS,
      true,
      current);
  out.hour_24 = photons_ppb_window_snapshot(
      g_photons_ppb_minutes,
      g_photons_ppb_minutes_head,
      g_photons_ppb_minutes_count,
      PHOTONS_PPB_HOUR_24_SECONDS,
      true,
      current);

  if (current.lap_count != 0ULL) {
    out.total.sample_count = current.lap_count;
    out.total.ppb = photons_ppb_from_population(
        current.total_lap_gnss_ns, current.lap_count);
    out.total.residual_ns = photons_residual_ns_from_population(
        current.total_lap_gnss_ns, current.lap_count);
  } else if (current.total_lap_gnss_ns != 0ULL) {
    __builtin_trap();
  }
  return out;
}


static const char* photons_campaign_state_name(photons_campaign_state_t state) {
  switch (state) {
    case photons_campaign_state_t::STOPPED:       return "STOPPED";
    case photons_campaign_state_t::START_PENDING: return "START_PENDING";
    case photons_campaign_state_t::ACTIVE:        return "ACTIVE";
    case photons_campaign_state_t::STOP_PENDING:      return "STOP_PENDING";
    case photons_campaign_state_t::FLASH_CUT_PENDING: return "FLASH_CUT_PENDING";
    default:                                          return "UNKNOWN";
  }
}


static photons_fragment_campaign_snapshot_t photons_campaign_snapshot(
    uint32_t fragment_sequence) {
  photons_fragment_campaign_snapshot_t out{};

  const bool active =
      g_photons_campaign_state == photons_campaign_state_t::ACTIVE ||
      g_photons_campaign_state == photons_campaign_state_t::STOP_PENDING ||
      g_photons_campaign_state == photons_campaign_state_t::FLASH_CUT_PENDING;
  if (!active) return out;

  if (!g_photons_campaign_name[0] ||
      g_photons_campaign_start_after_sequence == 0U ||
      fragment_sequence <= g_photons_campaign_start_after_sequence ||
      g_photons_custody_lap_count < g_photons_campaign_origin_lap_count ||
      g_photons_custody_total_lap_gnss_ns <
          g_photons_campaign_origin_total_lap_gnss_ns) {
    __builtin_trap();
  }

  out.present = true;
  out.final =
      g_photons_campaign_state == photons_campaign_state_t::STOP_PENDING ||
      g_photons_campaign_state == photons_campaign_state_t::FLASH_CUT_PENDING;
  safeCopy(out.campaign, sizeof(out.campaign), g_photons_campaign_name);
  out.start_after_sequence = g_photons_campaign_start_after_sequence;
  out.stop_after_sequence = out.final ? fragment_sequence : 0U;

  // Campaign identity belongs to the physical PHOTONS row, not to transport
  // success.  A PUBSUB rejection must therefore leave a visible hole in public
  // campaign identity instead of compressing the next successful row backward.
  // START commits start_after_sequence only after a successfully published
  // private boundary, so every later physical row has one exact ordinal.
  out.public_count =
      fragment_sequence - g_photons_campaign_start_after_sequence;
  out.lap_count =
      g_photons_custody_lap_count - g_photons_campaign_origin_lap_count;
  out.total_lap_gnss_ns =
      g_photons_custody_total_lap_gnss_ns -
      g_photons_campaign_origin_total_lap_gnss_ns;

  if (out.lap_count == 0ULL) {
    if (out.total_lap_gnss_ns != 0ULL) __builtin_trap();
    return out;
  }
  if (out.total_lap_gnss_ns == 0ULL) __builtin_trap();

  out.mean_lap_ns =
      (double)out.total_lap_gnss_ns / (double)out.lap_count;
  out.ppb.sample_count = out.lap_count;
  out.ppb.ppb =
      photons_ppb_from_population(out.total_lap_gnss_ns, out.lap_count);
  out.ppb.residual_ns =
      photons_residual_ns_from_population(out.total_lap_gnss_ns, out.lap_count);
  return out;
}


// Commit lifecycle transitions only after the corresponding fragment has been
// accepted by PUBSUB.  START therefore uses one successfully published
// pre-campaign fragment as its private boundary; the next fragment is CAMP #1.
// STOP publishes one final campaign fragment, then closes the firmware window.
static void photons_campaign_commit_after_publish(
    const photons_fragment_snapshot_t& fragment) {
  if (fragment.campaign.present) {
    if (g_photons_campaign_state != photons_campaign_state_t::ACTIVE &&
        g_photons_campaign_state != photons_campaign_state_t::STOP_PENDING &&
        g_photons_campaign_state != photons_campaign_state_t::FLASH_CUT_PENDING) {
      __builtin_trap();
    }
    const uint32_t expected_public_count =
        fragment.sequence - g_photons_campaign_start_after_sequence;
    if (fragment.campaign.start_after_sequence !=
            g_photons_campaign_start_after_sequence ||
        fragment.campaign.public_count != expected_public_count ||
        fragment.campaign.public_count <= g_photons_campaign_public_count ||
        strcmp(fragment.campaign.campaign, g_photons_campaign_name) != 0) {
      __builtin_trap();
    }

    // This stores the last successfully transported campaign identity.  It may
    // jump by more than one after a rejected publication; that jump is custody
    // evidence and must never be renumbered away.
    g_photons_campaign_public_count = fragment.campaign.public_count;

    if (fragment.campaign.final) {
      if (fragment.campaign.stop_after_sequence != fragment.sequence) {
        __builtin_trap();
      }

      if (g_photons_campaign_state == photons_campaign_state_t::STOP_PENDING) {
        g_photons_campaign_stop_commit_count++;
        g_photons_campaign_state = photons_campaign_state_t::STOPPED;
        g_photons_campaign_name[0] = '\0';
        g_photons_campaign_origin_lap_count = 0ULL;
        g_photons_campaign_origin_total_lap_gnss_ns = 0ULL;
        g_photons_campaign_start_after_sequence = 0U;
        g_photons_campaign_public_count = 0U;
        return;
      }

      if (g_photons_campaign_state ==
          photons_campaign_state_t::FLASH_CUT_PENDING) {
        if (!g_photons_flash_cut_campaign_name[0]) __builtin_trap();
        safeCopy(g_photons_campaign_name, sizeof(g_photons_campaign_name),
                 g_photons_flash_cut_campaign_name);
        g_photons_flash_cut_campaign_name[0] = '\0';
        g_photons_campaign_origin_lap_count = g_photons_custody_lap_count;
        g_photons_campaign_origin_total_lap_gnss_ns =
            g_photons_custody_total_lap_gnss_ns;
        g_photons_campaign_start_after_sequence = fragment.sequence;
        g_photons_campaign_public_count = 0U;
        g_photons_flash_cut_commit_count++;
        g_photons_campaign_state = photons_campaign_state_t::ACTIVE;
        return;
      }

      __builtin_trap();
    }
    return;
  }

  if (g_photons_campaign_state == photons_campaign_state_t::START_PENDING) {
    if (!g_photons_campaign_name[0] || fragment.sequence == 0U) {
      __builtin_trap();
    }
    g_photons_campaign_origin_lap_count = g_photons_custody_lap_count;
    g_photons_campaign_origin_total_lap_gnss_ns =
        g_photons_custody_total_lap_gnss_ns;
    g_photons_campaign_start_after_sequence = fragment.sequence;
    g_photons_campaign_public_count = 0U;
    g_photons_campaign_start_commit_count++;
    g_photons_campaign_state = photons_campaign_state_t::ACTIVE;
  }
}


static void photons_instrument_statistics_reset_commit(void) {
  if (!g_standard_lap_configured || g_lap_baseline_fs == 0ULL) __builtin_trap();

  photons_welford_reset(g_lap_time_welford);
  photons_welford_reset(g_accepted_raw_cycles_welford);
  photons_welford_reset(g_excluded_raw_cycles_welford);
  photons_welford_reset(g_excluded_lap_time_welford);
  g_total_lap_gnss_ns = 0ULL;
  g_photons_stats_update_count = 0U;
  g_photons_stats_reset_count++;
  photons_ppb_windows_seed_origin();

  // Statistical/court epoch only: priority-0 capture and monotonic campaign
  // custody stay live.  Clear sticky statistical-loss state so the new epoch can
  // become valid if custody is healthy from this boundary forward.
  g_raw_cycles_state = photons_fragment_raw_cycles_snapshot_t{};
  g_projection_state = photons_fragment_projection_snapshot_t{};
  g_photons_lap_science_state = photons_lap_science_snapshot_t{};
  g_photons_lap_science_seed_pending = photons_lap_science_candidate_t{};
  g_raw_lap_ring_overflow_count = 0U;
  g_raw_lap_ring_data_loss = false;
  g_previous_fragment_mean_cycles_valid = false;
  g_previous_fragment_mean_cycles = 0.0;
  g_photons_stats_reset_pending = false;
  g_photons_stats_reset_commit_count++;
}


static void photons_stats_reset_commit_after_publish(void) {
  if (!g_photons_stats_reset_pending) return;
  photons_instrument_statistics_reset_commit();
}


static photons_fragment_recovery_snapshot_t photons_recovery_snapshot(void) {
  photons_fragment_recovery_snapshot_t out{};
  out.restored = g_photons_recovery.restored;
  out.proof_pending = g_photons_recovery.proof_pending;
  out.proof_committed = g_photons_recovery.proof_committed;
  out.generation = g_photons_recovery.generation;
  out.source_sequence = g_photons_recovery.source_sequence;
  out.source_publish_count = g_photons_recovery.source_publish_count;
  out.source_reset_count = g_photons_recovery.source_reset_count;
  out.source_update_count = g_photons_recovery.source_update_count;
  out.source_lap_count = g_photons_recovery.source_lap_count;
  out.source_total_lap_gnss_ns =
      g_photons_recovery.source_total_lap_gnss_ns;
  out.source_custody_lap_count =
      g_photons_recovery.source_custody_lap_count;
  out.source_custody_total_lap_gnss_ns =
      g_photons_recovery.source_custody_total_lap_gnss_ns;
  out.fresh_physical_ancestry =
      g_photons_recovery.publication_started;

  // These false values are part of the scientific contract, not placeholders.
  out.raw_lap_ring_restored = false;
  out.partial_lap_restored = false;
  out.pending_seed_restored = false;
  out.predictor_restored = false;
  out.in_flight_train_restored = false;

  if (!out.restored) return out;

  // Recovery-source custody is permanently comparable because custody survives
  // STATS_RESET.  The resettable instrument N/T is comparable only while the
  // current statistical epoch is the epoch that was restored.  Once reset_count
  // advances, the source stats remain historical provenance rather than a floor
  // for the new epoch.
  if (g_photons_stats_reset_count < out.source_reset_count ||
      g_photons_custody_lap_count < out.source_custody_lap_count ||
      g_photons_custody_total_lap_gnss_ns <
          out.source_custody_total_lap_gnss_ns) {
    __builtin_trap();
  }

  out.custody_lap_delta =
      g_photons_custody_lap_count - out.source_custody_lap_count;

  const bool source_stats_epoch_current =
      g_photons_stats_reset_count == out.source_reset_count;
  if (source_stats_epoch_current) {
    if (g_lap_time_welford.n < out.source_lap_count ||
        g_total_lap_gnss_ns < out.source_total_lap_gnss_ns) {
      __builtin_trap();
    }
    out.accepted_lap_delta = g_lap_time_welford.n - out.source_lap_count;
    if (out.accepted_lap_delta != out.custody_lap_delta) __builtin_trap();
  } else {
    // Custody increments exactly once for every accepted lap and is not reset.
    // It therefore remains the truthful accepted-lap delta after a later
    // statistical reset, without manufacturing a comparison between epochs.
    out.accepted_lap_delta = out.custody_lap_delta;
  }

  // Recovery proof is chronology testimony, not a requirement that this exact
  // one-second row happened to accept a lap.  While the restored statistics epoch
  // remains current, physical publication and logical update chronology must move
  // together.  A lawful source+1 row with zero accepted laps is still the exact
  // successor.  Once published, retain the verdict across later STATS_RESET epochs
  // as durable recovery provenance.
  bool chronology_advanced_now = false;
  if (source_stats_epoch_current) {
    if (g_fragment_sequence < out.source_sequence ||
        g_photons_stats_update_count < out.source_update_count) {
      __builtin_trap();
    }
    const uint32_t sequence_delta =
        g_fragment_sequence - out.source_sequence;
    const uint32_t update_delta =
        g_photons_stats_update_count - out.source_update_count;
    if (sequence_delta != update_delta) __builtin_trap();
    chronology_advanced_now = sequence_delta != 0U;
  }
  out.proof_advanced =
      g_photons_recovery.proof_advanced_published || chronology_advanced_now;
  return out;
}


static void photons_projection_anchor_refresh(void) {
  const time_anchor_snapshot_t anchor = time_anchor_snapshot();

  g_projection_anchor_cache.seq++;
  photons_memory_barrier();

  g_projection_anchor_cache.valid =
      anchor.ok &&
      anchor.valid &&
      anchor.pps_vclock_count != 0U &&
      anchor.dwt_cycles_per_pps_vclock_s != 0U;
  g_projection_anchor_cache.dwt_at_pps_vclock =
      anchor.dwt_at_pps_vclock;
  g_projection_anchor_cache.dwt_cycles_per_second =
      anchor.dwt_cycles_per_pps_vclock_s;
  g_projection_anchor_cache.pps_count = anchor.pps_vclock_count;

  photons_memory_barrier();
  g_projection_anchor_cache.seq++;
}


static bool photons_projection_anchor_snapshot(
    photons_projection_anchor_value_t& out) {
  out = photons_projection_anchor_value_t{};

  for (int attempt = 0; attempt < 3; ++attempt) {
    const uint32_t before = g_projection_anchor_cache.seq;
    if (before & 1U) continue;

    photons_memory_barrier();
    photons_projection_anchor_value_t local{};
    local.valid = g_projection_anchor_cache.valid;
    local.dwt_at_pps_vclock =
        g_projection_anchor_cache.dwt_at_pps_vclock;
    local.dwt_cycles_per_second =
        g_projection_anchor_cache.dwt_cycles_per_second;
    local.pps_count = g_projection_anchor_cache.pps_count;
    photons_memory_barrier();

    const uint32_t after = g_projection_anchor_cache.seq;
    if (before == after && !(after & 1U)) {
      out = local;
      return true;
    }
  }

  return false;
}


static bool photons_raw_lap_enqueue(const photons_raw_lap_record_t& record) {
  const uint32_t write = g_raw_lap_ring_write;
  const uint32_t next = (write + 1U) & PHOTONS_LAP_RING_MASK;

  if (next == g_raw_lap_ring_read) {
    g_raw_lap_ring_overflow_count++;
    g_raw_lap_ring_data_loss = true;
    return false;
  }

  g_raw_lap_ring[write] = record;
  photons_memory_barrier();
  g_raw_lap_ring_write = next;
  return true;
}

static bool photons_project_raw_lap(
    const photons_raw_lap_record_t& record,
    uint64_t& start_gnss_ns,
    uint64_t& end_gnss_ns,
    uint64_t& lap_gnss_ns) {
  start_gnss_ns = 0ULL;
  end_gnss_ns = 0ULL;
  lap_gnss_ns = 0ULL;

  if (!record.anchor_valid ||
      record.anchor_pps_count == 0U ||
      record.anchor_dwt_cycles_per_second == 0U) {
    return false;
  }

  const uint32_t cps = record.anchor_dwt_cycles_per_second;
  const uint32_t start_elapsed =
      record.start_dwt - record.anchor_dwt_at_pps_vclock;
  const uint32_t end_elapsed =
      record.end_dwt - record.anchor_dwt_at_pps_vclock;

  const uint64_t max_age_cycles =
      ((uint64_t)cps * PHOTONS_PROJECTION_MAX_AGE_NS +
       PHOTONS_NS_PER_SECOND - 1ULL) /
      PHOTONS_NS_PER_SECOND;

  if ((uint64_t)start_elapsed > max_age_cycles ||
      (uint64_t)end_elapsed > max_age_cycles) {
    return false;
  }

  const uint64_t base_gnss_ns =
      (uint64_t)(record.anchor_pps_count - 1U) *
      PHOTONS_NS_PER_SECOND;

  const uint64_t start_offset_ns =
      ((uint64_t)start_elapsed * PHOTONS_NS_PER_SECOND +
       (uint64_t)cps / 2ULL) /
      (uint64_t)cps;
  const uint64_t end_offset_ns =
      ((uint64_t)end_elapsed * PHOTONS_NS_PER_SECOND +
       (uint64_t)cps / 2ULL) /
      (uint64_t)cps;

  start_gnss_ns = base_gnss_ns + start_offset_ns;
  end_gnss_ns = base_gnss_ns + end_offset_ns;
  if (end_gnss_ns < start_gnss_ns) return false;

  lap_gnss_ns = end_gnss_ns - start_gnss_ns;
  return lap_gnss_ns != 0ULL;
}


static const char* photons_lap_science_disposition_name(uint8_t id) {
  switch ((photons_lap_science_disposition_t)id) {
    case photons_lap_science_disposition_t::ACCEPT:
      return "ACCEPT";
    case photons_lap_science_disposition_t::SCIENCE_EXCLUDE:
      return "SCIENCE_EXCLUDE";
    case photons_lap_science_disposition_t::PENDING_SEED:
      return "PENDING_SEED";
    default:
      return "NONE";
  }
}


static const char* photons_lap_science_reason_name(uint16_t code) {
  switch ((photons_lap_science_exclusion_reason_t)code) {
    case photons_lap_science_exclusion_reason_t::PROJECTION_INVALID:
      return "projection_invalid";
    case photons_lap_science_exclusion_reason_t::SEED_DISAGREEMENT:
      return "seed_disagreement";
    case photons_lap_science_exclusion_reason_t::RAW_CYCLE_EXCURSION:
      return "raw_cycle_excursion";
    default:
      return "none";
  }
}


static uint32_t photons_lap_science_gate_cycles(uint32_t prediction_cycles) {
  uint32_t gate = prediction_cycles / PHOTONS_SCIENCE_GATE_DIVISOR;
  if (gate < PHOTONS_SCIENCE_GATE_MIN_CYCLES) {
    gate = PHOTONS_SCIENCE_GATE_MIN_CYCLES;
  }
  return gate;
}


static int32_t photons_lap_science_signed_delta(uint32_t observed,
                                            uint32_t predicted) {
  const int64_t delta = (int64_t)(uint64_t)observed -
                        (int64_t)(uint64_t)predicted;
  if (delta > 2147483647LL) return 2147483647;
  if (delta < -2147483647LL - 1LL) return (-2147483647 - 1);
  return (int32_t)delta;
}


static uint64_t photons_lap_science_abs_delta(uint32_t observed,
                                          uint32_t predicted) {
  return (observed >= predicted)
      ? (uint64_t)(observed - predicted)
      : (uint64_t)(predicted - observed);
}


static void photons_lap_science_note_last(
    const photons_lap_science_candidate_t& candidate,
    photons_lap_science_disposition_t disposition,
    photons_lap_science_exclusion_reason_t reason,
    bool projection_valid,
    uint32_t prediction_cycles,
    int32_t residual_cycles,
    uint32_t gate_cycles) {
  g_photons_lap_science_state.last_candidate_index = candidate.candidate_index;
  g_photons_lap_science_state.last_disposition_id = (uint8_t)disposition;
  g_photons_lap_science_state.last_reason_code = (uint16_t)reason;
  g_photons_lap_science_state.last_projection_valid = projection_valid;
  g_photons_lap_science_state.last_pps_sequence = candidate.pps_sequence;
  g_photons_lap_science_state.last_observed_cycles = candidate.raw_cycles;
  g_photons_lap_science_state.last_prediction_cycles = prediction_cycles;
  g_photons_lap_science_state.last_residual_cycles = residual_cycles;
  g_photons_lap_science_state.last_gate_cycles = gate_cycles;
  g_photons_lap_science_state.last_lap_gnss_ns =
      projection_valid ? candidate.lap_gnss_ns : 0ULL;
}


static void photons_lap_science_refresh_seed_snapshot(void) {
  g_photons_lap_science_state.seed_pending = g_photons_lap_science_seed_pending.valid;
  g_photons_lap_science_state.seed_pending_candidate_index =
      g_photons_lap_science_seed_pending.valid
          ? g_photons_lap_science_seed_pending.candidate_index
          : 0ULL;
  g_photons_lap_science_state.seed_pending_raw_cycles =
      g_photons_lap_science_seed_pending.valid
          ? g_photons_lap_science_seed_pending.raw_cycles
          : 0U;
  g_photons_lap_science_state.seed_pending_lap_gnss_ns =
      g_photons_lap_science_seed_pending.valid
          ? g_photons_lap_science_seed_pending.lap_gnss_ns
          : 0ULL;
}


static void photons_lap_science_accept(
    const photons_lap_science_candidate_t& candidate,
    uint32_t prediction_cycles,
    int32_t residual_cycles,
    uint32_t gate_cycles) {
  g_photons_lap_science_state.accepted.count++;
  g_photons_lap_science_state.accepted.count_this_fragment++;
  g_photons_lap_science_state.reject_streak = 0U;

  photons_welford_update(
      g_accepted_raw_cycles_welford, (double)candidate.raw_cycles);
  g_total_lap_gnss_ns += candidate.lap_gnss_ns;
  photons_welford_update(
      g_lap_time_welford, (double)candidate.lap_gnss_ns);
  g_photons_custody_lap_count++;
  g_photons_custody_total_lap_gnss_ns += candidate.lap_gnss_ns;

  g_photons_lap_science_state.predictor_valid = true;
  g_photons_lap_science_state.predictor_cycles = candidate.raw_cycles;
  g_photons_lap_science_state.gate_cycles =
      photons_lap_science_gate_cycles(candidate.raw_cycles);

  photons_lap_science_note_last(
      candidate,
      photons_lap_science_disposition_t::ACCEPT,
      photons_lap_science_exclusion_reason_t::NONE,
      true,
      prediction_cycles,
      residual_cycles,
      gate_cycles);
}


static void photons_lap_science_count_exclusion_reason(
    photons_lap_science_exclusion_reason_t reason) {
  switch (reason) {
    case photons_lap_science_exclusion_reason_t::PROJECTION_INVALID:
      g_photons_lap_science_state.exclusion_reasons.projection_invalid++;
      g_photons_lap_science_state.exclusion_reasons.projection_invalid_this_fragment++;
      return;
    case photons_lap_science_exclusion_reason_t::SEED_DISAGREEMENT:
      g_photons_lap_science_state.exclusion_reasons.seed_disagreement++;
      g_photons_lap_science_state.exclusion_reasons.seed_disagreement_this_fragment++;
      return;
    case photons_lap_science_exclusion_reason_t::RAW_CYCLE_EXCURSION:
      g_photons_lap_science_state.exclusion_reasons.raw_cycle_excursion++;
      g_photons_lap_science_state.exclusion_reasons.raw_cycle_excursion_this_fragment++;
      return;
    case photons_lap_science_exclusion_reason_t::NONE:
    default:
      // Exclusion without an authored reason is a courtroom integrity failure.
      __builtin_trap();
  }
}


// Exclusion population has one authority: the authored reason ledger.  The
// aggregate excluded counts carried by PHOTONS_FRAGMENT remain convenient schema
// fields, but they are derived testimony and are never independently advanced.
static uint64_t photons_lap_science_excluded_count_from_reasons(
    const photons_lap_science_reason_counts_snapshot_t& reasons) {
  if (UINT64_MAX - reasons.projection_invalid < reasons.seed_disagreement) {
    __builtin_trap();
  }
  const uint64_t partial =
      reasons.projection_invalid + reasons.seed_disagreement;
  if (UINT64_MAX - partial < reasons.raw_cycle_excursion) {
    __builtin_trap();
  }
  return partial + reasons.raw_cycle_excursion;
}


static uint32_t photons_lap_science_excluded_this_fragment_from_reasons(
    const photons_lap_science_reason_counts_snapshot_t& reasons) {
  const uint64_t total =
      (uint64_t)reasons.projection_invalid_this_fragment +
      (uint64_t)reasons.seed_disagreement_this_fragment +
      (uint64_t)reasons.raw_cycle_excursion_this_fragment;
  if (total > (uint64_t)UINT32_MAX) __builtin_trap();
  return (uint32_t)total;
}


static void photons_lap_science_validate_exclusion_ledger(
    const photons_lap_science_snapshot_t& science) {
  const uint64_t excluded_count =
      photons_lap_science_excluded_count_from_reasons(
          science.exclusion_reasons);

  // Every finalized exclusion enters the raw-cycle Welford exactly once.  Its N
  // is an independent witness, not a second exclusion-count authority.
  if (science.excluded.raw_cycles.n != excluded_count ||
      science.excluded.projected_lap_ns.n > excluded_count) {
    __builtin_trap();
  }

  if (UINT64_MAX - science.accepted.count < excluded_count) {
    __builtin_trap();
  }
  const uint64_t finalized_count =
      science.accepted.count + excluded_count;
  const uint64_t pending_count = science.seed_pending ? 1ULL : 0ULL;
  if (UINT64_MAX - finalized_count < pending_count ||
      science.candidate_count != finalized_count + pending_count) {
    __builtin_trap();
  }
}


static void photons_lap_science_materialize_derived_exclusion_counts(
    photons_lap_science_snapshot_t& science) {
  photons_lap_science_validate_exclusion_ledger(science);
  science.excluded.count =
      photons_lap_science_excluded_count_from_reasons(
          science.exclusion_reasons);
  science.excluded.count_this_fragment =
      photons_lap_science_excluded_this_fragment_from_reasons(
          science.exclusion_reasons);
}


static void photons_lap_science_exclude(
    const photons_lap_science_candidate_t& candidate,
    photons_lap_science_exclusion_reason_t reason,
    bool projection_valid,
    uint32_t prediction_cycles,
    int32_t residual_cycles,
    uint32_t gate_cycles) {
  // The reason ledger is the sole exclusion-population authority.  Do not
  // maintain a parallel aggregate counter here; it is derived at snapshot time.
  photons_lap_science_count_exclusion_reason(reason);

  photons_welford_update(
      g_excluded_raw_cycles_welford, (double)candidate.raw_cycles);
  if (projection_valid) {
    photons_welford_update(
        g_excluded_lap_time_welford, (double)candidate.lap_gnss_ns);
  }

  g_photons_lap_science_state.reject_streak++;
  if (g_photons_lap_science_state.reject_streak > g_photons_lap_science_state.max_reject_streak) {
    g_photons_lap_science_state.max_reject_streak = g_photons_lap_science_state.reject_streak;
  }

  photons_lap_science_note_last(
      candidate,
      photons_lap_science_disposition_t::SCIENCE_EXCLUDE,
      reason,
      projection_valid,
      prediction_cycles,
      residual_cycles,
      gate_cycles);
}


static void photons_lap_science_projected_candidate(
    const photons_lap_science_candidate_t& candidate) {
  if (!g_photons_lap_science_state.predictor_valid) {
    if (!g_photons_lap_science_seed_pending.valid) {
      g_photons_lap_science_seed_pending = candidate;
      photons_lap_science_refresh_seed_snapshot();
      return;
    }

    const uint32_t prediction = g_photons_lap_science_seed_pending.raw_cycles;
    const uint32_t gate = photons_lap_science_gate_cycles(prediction);
    const int32_t residual =
        photons_lap_science_signed_delta(candidate.raw_cycles, prediction);
    const bool agrees =
        photons_lap_science_abs_delta(candidate.raw_cycles, prediction) <=
        (uint64_t)gate;

    if (agrees) {
      const photons_lap_science_candidate_t seed = g_photons_lap_science_seed_pending;
      g_photons_lap_science_seed_pending = photons_lap_science_candidate_t{};
      photons_lap_science_refresh_seed_snapshot();

      // The agreeing pair jointly establishes lineage.  Both projected laps
      // become science observations; the second becomes the carry-forward
      // static predictor for the next candidate.
      photons_lap_science_accept(seed, 0U, 0, 0U);
      photons_lap_science_accept(candidate, prediction, residual, gate);
      return;
    }

    // Neither member of a disagreeing seed pair is privileged.  Preserve both
    // as explicit exclusions and restart acquisition from the next candidate.
    const photons_lap_science_candidate_t seed = g_photons_lap_science_seed_pending;
    g_photons_lap_science_seed_pending = photons_lap_science_candidate_t{};
    photons_lap_science_refresh_seed_snapshot();

    photons_lap_science_exclude(
        seed,
        photons_lap_science_exclusion_reason_t::SEED_DISAGREEMENT,
        true,
        candidate.raw_cycles,
        photons_lap_science_signed_delta(seed.raw_cycles,
                                     candidate.raw_cycles),
        photons_lap_science_gate_cycles(candidate.raw_cycles));
    photons_lap_science_exclude(
        candidate,
        photons_lap_science_exclusion_reason_t::SEED_DISAGREEMENT,
        true,
        prediction,
        residual,
        gate);
    return;
  }

  const uint32_t prediction = g_photons_lap_science_state.predictor_cycles;
  const uint32_t gate = photons_lap_science_gate_cycles(prediction);
  const int32_t residual =
      photons_lap_science_signed_delta(candidate.raw_cycles, prediction);

  if (photons_lap_science_abs_delta(candidate.raw_cycles, prediction) <=
      (uint64_t)gate) {
    photons_lap_science_accept(candidate, prediction, residual, gate);
    return;
  }

  // Rejected observations never advance the predictor, so a single excursion
  // cannot poison the reference used to adjudicate the next physical lap.
  photons_lap_science_exclude(
      candidate,
      photons_lap_science_exclusion_reason_t::RAW_CYCLE_EXCURSION,
      true,
      prediction,
      residual,
      gate);
}


struct photons_fragment_drain_result_t {
  uint32_t raw_laps = 0U;
  uint32_t projected_laps = 0U;
  uint64_t total_cycles = 0ULL;
  uint32_t min_cycles = 0U;
  uint32_t max_cycles = 0U;
  photons_welford_state_t projected_flight_welford{};
};


static photons_fragment_drain_result_t photons_drain_raw_laps(void) {
  photons_fragment_drain_result_t result{};
  photons_welford_reset(result.projected_flight_welford);

  g_photons_lap_science_state.candidates_this_fragment = 0U;
  g_photons_lap_science_state.accepted.count_this_fragment = 0U;
  g_photons_lap_science_state.exclusion_reasons.projection_invalid_this_fragment = 0U;
  g_photons_lap_science_state.exclusion_reasons.seed_disagreement_this_fragment = 0U;
  g_photons_lap_science_state.exclusion_reasons.raw_cycle_excursion_this_fragment = 0U;

  photons_memory_barrier();
  const uint32_t write_snapshot = g_raw_lap_ring_write;

  while (g_raw_lap_ring_read != write_snapshot) {
    photons_memory_barrier();
    const photons_raw_lap_record_t record =
        g_raw_lap_ring[g_raw_lap_ring_read];
    g_raw_lap_ring_read =
        (g_raw_lap_ring_read + 1U) & PHOTONS_LAP_RING_MASK;

    result.raw_laps++;
    result.total_cycles += (uint64_t)record.raw_cycles;
    if (result.min_cycles == 0U || record.raw_cycles < result.min_cycles) {
      result.min_cycles = record.raw_cycles;
    }
    if (record.raw_cycles > result.max_cycles) {
      result.max_cycles = record.raw_cycles;
    }

    g_raw_cycles_state.completed_lap_count++;
    if (g_raw_cycles_state.completed_lap_count > 1ULL) {
      g_raw_cycles_state.static_prediction_valid = true;
      g_raw_cycles_state.previous_observed_cycles =
          g_raw_cycles_state.observed_cycles;
      g_raw_cycles_state.static_prediction_cycles =
          g_raw_cycles_state.previous_observed_cycles;
      g_raw_cycles_state.observed_cycles = record.raw_cycles;
      g_raw_cycles_state.static_residual_cycles =
          (int32_t)((int64_t)record.raw_cycles -
                    (int64_t)g_raw_cycles_state.static_prediction_cycles);
    } else {
      g_raw_cycles_state.static_prediction_valid = false;
      g_raw_cycles_state.previous_observed_cycles = 0U;
      g_raw_cycles_state.static_prediction_cycles = 0U;
      g_raw_cycles_state.observed_cycles = record.raw_cycles;
      g_raw_cycles_state.static_residual_cycles = 0;
    }

    g_projection_state.attempt_count++;
    g_photons_lap_science_state.candidate_count++;
    g_photons_lap_science_state.candidates_this_fragment++;
    photons_lap_science_candidate_t science_candidate{};
    science_candidate.valid = true;
    science_candidate.candidate_index = g_photons_lap_science_state.candidate_count;
    science_candidate.pps_sequence = record.pps_sequence;
    science_candidate.raw_cycles = record.raw_cycles;

    uint64_t start_gnss_ns = 0ULL;
    uint64_t end_gnss_ns = 0ULL;
    uint64_t lap_gnss_ns = 0ULL;
    if (!photons_project_raw_lap(
            record, start_gnss_ns, end_gnss_ns, lap_gnss_ns)) {
      g_projection_state.reject_count++;
      photons_lap_science_exclude(
          science_candidate,
          photons_lap_science_exclusion_reason_t::PROJECTION_INVALID,
          false,
          g_photons_lap_science_state.predictor_valid
              ? g_photons_lap_science_state.predictor_cycles
              : 0U,
          0,
          g_photons_lap_science_state.predictor_valid
              ? photons_lap_science_gate_cycles(
                    g_photons_lap_science_state.predictor_cycles)
              : 0U);
      continue;
    }

    g_projection_state.success_count++;
    g_projection_state.last_valid = true;
    g_projection_state.last_pps_sequence = record.pps_sequence;
    g_projection_state.last_start_dwt = record.start_dwt;
    g_projection_state.last_end_dwt = record.end_dwt;
    g_projection_state.last_raw_cycles = record.raw_cycles;
    g_projection_state.last_start_gnss_ns = start_gnss_ns;
    g_projection_state.last_end_gnss_ns = end_gnss_ns;
    g_projection_state.last_lap_gnss_ns = lap_gnss_ns;

    // Every successfully projected physical race enters the one-second metrics
    // Welford. The science court below independently decides whether it may
    // mutate the canonical lifetime population.
    photons_welford_update(
        result.projected_flight_welford, (double)lap_gnss_ns);

    science_candidate.lap_gnss_ns = lap_gnss_ns;
    photons_lap_science_projected_candidate(science_candidate);
    result.projected_laps++;
  }

  g_raw_cycles_state.valid =
      g_raw_cycles_state.completed_lap_count != 0ULL;
  g_raw_cycles_state.laps_this_fragment = result.raw_laps;
  g_raw_cycles_state.total_cycles_this_fragment = result.total_cycles;
  g_raw_cycles_state.min_cycles_this_fragment = result.min_cycles;
  g_raw_cycles_state.max_cycles_this_fragment = result.max_cycles;
  g_raw_cycles_state.mean_cycles_this_fragment =
      (result.raw_laps != 0U)
          ? (double)result.total_cycles / (double)result.raw_laps
          : 0.0;

  g_raw_cycles_state.previous_fragment_mean_valid =
      g_previous_fragment_mean_cycles_valid;
  g_raw_cycles_state.previous_fragment_mean_cycles =
      g_previous_fragment_mean_cycles;
  g_raw_cycles_state.fragment_mean_residual_cycles =
      (result.raw_laps != 0U && g_previous_fragment_mean_cycles_valid)
          ? g_raw_cycles_state.mean_cycles_this_fragment -
                g_previous_fragment_mean_cycles
          : 0.0;

  if (result.raw_laps != 0U) {
    g_previous_fragment_mean_cycles =
        g_raw_cycles_state.mean_cycles_this_fragment;
    g_previous_fragment_mean_cycles_valid = true;
  }

  g_projection_state.queue_overflow_count =
      (uint64_t)g_raw_lap_ring_overflow_count;

  photons_lap_science_refresh_seed_snapshot();
  g_photons_lap_science_state.valid =
      g_photons_lap_science_state.candidate_count != 0ULL;

  // Snapshot the two courtroom populations.  Accepted projected-lap time is
  // deliberately the same canonical Welford used by fragment.stats; no second
  // accepted scientific population exists.
  g_photons_lap_science_state.accepted.raw_cycles =
      photons_welford_snapshot(g_accepted_raw_cycles_welford);
  g_photons_lap_science_state.accepted.projected_lap_ns =
      photons_welford_snapshot(g_lap_time_welford);
  g_photons_lap_science_state.excluded.raw_cycles =
      photons_welford_snapshot(g_excluded_raw_cycles_welford);
  g_photons_lap_science_state.excluded.projected_lap_ns =
      photons_welford_snapshot(g_excluded_lap_time_welford);

  // The mutable runtime never owns excluded.count.  Prove the reason ledger
  // against the independent excluded raw-Welford population before it can leave
  // the science court.
  photons_lap_science_validate_exclusion_ledger(
      g_photons_lap_science_state);

  return result;
}


static void photons_i2c_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MP5491_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static uint8_t photons_i2c_read(uint8_t reg) {
  Wire.beginTransmission(MP5491_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MP5491_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

static float photons_adc_voltage(uint16_t raw) {
  return (raw / ADC_FS_COUNTS) * ADC_FS_VOLTS;
}

static void photons_laser_inhibit(void) {
  digitalWrite(LD_ON_PIN, LOW);
}


static bool photons_ns_to_dwt_cycles(uint64_t requested_ns,
                                     uint32_t dwt_cycles_per_second,
                                     uint32_t& out_cycles) {
  out_cycles = 0U;
  if (requested_ns == 0ULL || dwt_cycles_per_second == 0U) return false;

  const uint64_t whole_seconds = requested_ns / PHOTONS_NS_PER_SECOND;
  const uint64_t remainder_ns = requested_ns % PHOTONS_NS_PER_SECOND;
  if (whole_seconds > UINT64_MAX / (uint64_t)dwt_cycles_per_second) {
    return false;
  }

  const uint64_t whole_cycles =
      whole_seconds * (uint64_t)dwt_cycles_per_second;
  const uint64_t remainder_cycles =
      (remainder_ns * (uint64_t)dwt_cycles_per_second +
       PHOTONS_NS_PER_SECOND / 2ULL) /
      PHOTONS_NS_PER_SECOND;

  if (whole_cycles > (uint64_t)UINT32_MAX ||
      remainder_cycles > (uint64_t)UINT32_MAX ||
      whole_cycles > (uint64_t)UINT32_MAX - remainder_cycles) {
    return false;
  }

  const uint64_t total_cycles = whole_cycles + remainder_cycles;
  if (total_cycles == 0ULL) return false;
  out_cycles = (uint32_t)total_cycles;
  return true;
}

static void photons_race_receive_publish(
    const photons_race_receive_value_t& value) {
  g_photons_race_receive.generation++;
  photons_memory_barrier();
  g_photons_race_receive.value = value;
  photons_memory_barrier();
  g_photons_race_receive.generation++;
}

static bool photons_race_receive_snapshot(
    photons_race_receive_value_t& out) {
  for (;;) {
    const uint32_t before = g_photons_race_receive.generation;
    if (before & 1U) continue;

    photons_memory_barrier();
    const photons_race_receive_value_t snapshot =
        g_photons_race_receive.value;
    photons_memory_barrier();

    const uint32_t after = g_photons_race_receive.generation;
    if (before == after && !(after & 1U)) {
      out = snapshot;
      return true;
    }
  }
}

static void photons_race_observe_edge(
    const interrupt_photodiode_edge_t& edge) {
  const uint32_t race_sequence = g_race_armed_sequence;
  if (race_sequence == 0U) return;

  // First physical RISING edge wins. Disarm before publishing the receive latch
  // so comparator chatter cannot become additional race observations.
  g_race_armed_sequence = 0U;
  photons_memory_barrier();

  photons_race_receive_value_t value{};
  value.seen = true;
  value.race_sequence = race_sequence;
  value.edge_sequence = edge.sequence;
  value.pps_sequence = edge.pps_sequence;
  value.finish_dwt = edge.dwt_at_edge;
  photons_race_receive_publish(value);
}

static void photons_race_finalize_previous(void) {
  if (!g_photons_race_launch.valid) return;

  // The cadence boundary is the explicit receive timeout. Close the arm before
  // examining the latch so a late edge cannot bleed into the next race.
  g_race_armed_sequence = 0U;
  photons_memory_barrier();

  photons_race_receive_value_t receive{};
  (void)photons_race_receive_snapshot(receive);
  const photons_race_launch_slot_t launch = g_photons_race_launch;

  if (!launch.launch_surrogate_valid || !launch.anchor_valid ||
      launch.anchor_dwt_cycles_per_second == 0U ||
      launch.anchor_pps_count == 0U) {
    __builtin_trap();
  }

  if (!receive.seen) {
    g_photons_race.missed_count++;
  } else {
    if (receive.race_sequence != launch.sequence) __builtin_trap();

    // Every valid interim flight endpoint must occur after the actual LD_ON LOW
    // surrogate. At a 1 ms cadence the signed DWT difference is unambiguous even
    // across a 32-bit DWT wrap. An early receive is evidence, not a measurement.
    const uint32_t elapsed_from_cell_start =
        receive.finish_dwt - launch.ld_on_start_dwt;
    const int32_t signed_cycles =
        (int32_t)(receive.finish_dwt - launch.launch_surrogate_dwt);
    if (signed_cycles <= 0 ||
        elapsed_from_cell_start > launch.cadence_cell_cycles) {
      // Either the first edge preceded the actual LD_ON-low surrogate or a
      // delayed foreground cadence left the receive arm open beyond its 1 ms
      // cell. Preserve the injury as invalid endpoint testimony; never let it
      // enter the flight Welford.
      g_photons_race.invalid_endpoint_count++;
    } else {
      photons_raw_lap_record_t record{};
      record.start_dwt = launch.launch_surrogate_dwt;
      record.end_dwt = receive.finish_dwt;
      record.raw_cycles = (uint32_t)signed_cycles;
      record.pps_sequence = receive.pps_sequence;
      record.anchor_valid = true;
      record.anchor_dwt_at_pps_vclock =
          launch.anchor_dwt_at_pps_vclock;
      record.anchor_dwt_cycles_per_second =
          launch.anchor_dwt_cycles_per_second;
      record.anchor_pps_count = launch.anchor_pps_count;

      if (photons_raw_lap_enqueue(record)) {
        g_photons_race.completed_count++;
      } else {
        g_photons_race.enqueue_failure_count++;
      }
    }
  }

  g_photons_race_launch = photons_race_launch_slot_t{};
  photons_race_receive_publish(photons_race_receive_value_t{});
}

static void photons_race_cadence_tick(
    timepop_ctx_t* /*ctx*/,
    timepop_diag_t* /*diag*/,
    void* /*user_data*/) {
  g_photons_race.cadence_tick_count++;
  photons_race_finalize_previous();

  if (digitalRead(LD_ON_PIN) != LOW) {
    // PHOTONS exclusively owns LD_ON while the race engine is live. Inhibit
    // first for optical safety, then fail loudly on the ownership violation.
    photons_laser_inhibit();
    __builtin_trap();
  }

  if (digitalRead(PHOTODIODE_EDGE_PIN) != LOW) {
    g_photons_race.skipped_not_quiet_count++;
    return;
  }

  photons_projection_anchor_value_t anchor{};
  if (!photons_projection_anchor_snapshot(anchor) || !anchor.valid ||
      anchor.dwt_cycles_per_second == 0U || anchor.pps_count == 0U) {
    g_photons_race.skipped_projection_count++;
    return;
  }

  uint32_t target_high_cycles = 0U;
  uint32_t cadence_cell_cycles = 0U;
  if (!photons_ns_to_dwt_cycles(
          PHOTONS_RACE_PULSE_NS,
          anchor.dwt_cycles_per_second,
          target_high_cycles) ||
      !photons_ns_to_dwt_cycles(
          PHOTONS_RACE_CADENCE_NS,
          anchor.dwt_cycles_per_second,
          cadence_cell_cycles) ||
      target_high_cycles >= cadence_cell_cycles) {
    __builtin_trap();
  }

  g_photons_race.sequence++;
  if (g_photons_race.sequence == 0U) g_photons_race.sequence++;
  const uint32_t race_sequence = g_photons_race.sequence;

  photons_race_receive_publish(photons_race_receive_value_t{});
  photons_race_launch_slot_t launch{};
  launch.valid = true;
  launch.sequence = race_sequence;
  launch.target_high_cycles = target_high_cycles;
  launch.cadence_cell_cycles = cadence_cell_cycles;
  launch.anchor_valid = true;
  launch.anchor_dwt_at_pps_vclock = anchor.dwt_at_pps_vclock;
  launch.anchor_dwt_cycles_per_second = anchor.dwt_cycles_per_second;
  launch.anchor_pps_count = anchor.pps_count;

  // Publish all launch facts the foreground owns before opening the receive arm.
  // The ISR never reads this structure, but this ordering makes the ownership
  // boundary explicit and prevents future refactors from creating a half-launch.
  launch.ld_on_start_dwt = ARM_DWT_CYCCNT;
  g_photons_race_launch = launch;
  photons_memory_barrier();
  g_race_armed_sequence = race_sequence;
  photons_memory_barrier();

  digitalWriteFast(LD_ON_PIN, HIGH);
  const uint32_t high_start = ARM_DWT_CYCCNT;
  while ((uint32_t)(ARM_DWT_CYCCNT - high_start) < target_high_cycles) {
  }
  digitalWriteFast(LD_ON_PIN, LOW);
  const uint32_t ld_on_low_dwt = ARM_DWT_CYCCNT;

  g_photons_race_launch.launch_surrogate_dwt = ld_on_low_dwt;
  g_photons_race_launch.pulse_wall_cycles =
      ld_on_low_dwt - g_photons_race_launch.ld_on_start_dwt;
  if (g_photons_race_launch.pulse_wall_cycles >= cadence_cell_cycles) {
    // LD_ON is already LOW here. A pulse transaction that consumes the whole
    // cell is a scheduler/timing invariant failure, not a usable optical race.
    __builtin_trap();
  }
  photons_memory_barrier();
  g_photons_race_launch.launch_surrogate_valid = true;
  g_photons_race.attempt_count++;
}

static void photons_race_prepare(void) {
  g_race_armed_sequence = 0U;
  g_photons_race = photons_race_runtime_t{};
  g_photons_race_launch = photons_race_launch_slot_t{};
  g_photons_race_receive.generation = 0U;
  g_photons_race_receive.value = photons_race_receive_value_t{};
  g_photons_race.initialized = true;
}

static void photons_race_start(void) {
  if (!g_photons_race.initialized ||
      g_photons_race.cadence_timer != TIMEPOP_INVALID_HANDLE) {
    __builtin_trap();
  }
  if (digitalRead(LD_ON_PIN) != LOW) {
    photons_laser_inhibit();
    __builtin_trap();
  }

  g_photons_race.cadence_timer = timepop_arm(
      PHOTONS_RACE_CADENCE_NS,
      true,
      photons_race_cadence_tick,
      nullptr,
      "PHOTONS_RACE_1KHZ");
  if (g_photons_race.cadence_timer == TIMEPOP_INVALID_HANDLE) {
    __builtin_trap();
  }
}

static void photons_laser_initialize_hardware(void) {
  pinMode(LD_ON_PIN, OUTPUT);
  photons_laser_inhibit();

  pinMode(LASER_MONITOR_PIN, INPUT);
  pinMode(PHOTODIODE_ANALOG_PIN, INPUT);
  analogReadResolution(12);

  photons_i2c_write(MP5491_REG_CTL0, MP5491_SYSEN_BIT);

  const uint8_t ctl1 = photons_i2c_read(MP5491_REG_CTL1);
  photons_i2c_write(
      MP5491_REG_CTL1,
      MP5491_ID_EN_BIT | MP5491_ID1_EN_BIT | (ctl1 & 0x07));

  photons_i2c_write(
      MP5491_REG_ID1_MSB,
      PHOTONS_LASER_ID1_CURRENT_MSB);

  const uint8_t lsb = photons_i2c_read(MP5491_REG_ID1_LSB);
  photons_i2c_write(
      MP5491_REG_ID1_LSB,
      (lsb & ~0x03U) | PHOTONS_LASER_ID1_CURRENT_LSB);
}

static photons_device_snapshot_t photons_device_snapshot(void) {
  photons_device_snapshot_t out{};

  const uint8_t msb = photons_i2c_read(MP5491_REG_ID1_MSB);
  const uint8_t lsb = photons_i2c_read(MP5491_REG_ID1_LSB);
  out.laser_id1_raw = ((uint16_t)msb << 2) | (lsb & 0x03U);
  out.laser_id1_current_ma = out.laser_id1_raw * 0.25f;

  out.laser_enabled = digitalRead(LD_ON_PIN) == HIGH;

  out.laser_monitor_raw = analogRead(LASER_MONITOR_PIN);
  out.laser_monitor_v = photons_adc_voltage(out.laser_monitor_raw);
  out.laser_emitting =
      out.laser_monitor_v > PHOTONS_LASER_EMIT_THRESHOLD_V;

  // Ambient pin level is diagnostic only. Timing evidence comes exclusively
  // from process_interrupt's PHOTODIODE edge capture.
  out.photodiode_edge_level = digitalRead(PHOTODIODE_EDGE_PIN);

  // Pin 38/A14 is the real PD200T PD OUT commissioning telemetry. Timing
  // authority remains the independent pin-34 comparator edge.
  out.photodiode_analog_raw = analogRead(PHOTODIODE_ANALOG_PIN);
  out.photodiode_analog_v = photons_adc_voltage(out.photodiode_analog_raw);

  return out;
}

static void photons_emit_laser_initialization_event(void) {
  const photons_device_snapshot_t device = photons_device_snapshot();

  Payload p;
  p.add("id1_raw", device.laser_id1_raw);
  p.add("id1_current_ma",
        toFixedDecimal(device.laser_id1_current_ma, 6));
  p.add("pd_voltage", toFixedDecimal(device.laser_monitor_v, 6));
  p.add("laser_emitting", device.laser_emitting);
  enqueueEvent("LASER_INITIALIZATION", p);
}

// -----------------------------------------------------------------------------
// ISR-authored live state
// -----------------------------------------------------------------------------
//
// Single writer: PHOTODIODE callback.
// Foreground readers use generation as a tiny seqlock.  No interrupt masking is
// required, so PHOTONS reporting/publication never delays sovereign CLOCKS IRQs.
//
struct photons_live_state_t {
  volatile uint32_t generation = 0;
  photons_toy_capture_t capture{};

  bool     previous_edge_valid = false;
  uint32_t previous_dwt_at_edge = 0;

  bool     previous_interval_valid = false;
  uint32_t previous_interval_cycles = 0;
};

static photons_live_state_t g_photons_live{};

// -----------------------------------------------------------------------------
// Foreground-owned publication state
// -----------------------------------------------------------------------------

static photons_toy_fragment_t g_last_fragment{};
static photons_fragment_snapshot_t g_last_photons_fragment{};
static uint32_t g_publish_count = 0;
static uint32_t g_publish_reject_count = 0;
static uint32_t g_last_published_edge_count = 0;
static uint64_t g_last_fragment_race_cadence_tick_count = 0ULL;
static uint64_t g_last_fragment_race_attempt_count = 0ULL;
static uint64_t g_last_fragment_race_completed_count = 0ULL;
static uint64_t g_last_fragment_race_missed_count = 0ULL;
static uint64_t g_last_fragment_race_skipped_not_quiet_count = 0ULL;
static uint64_t g_last_fragment_race_skipped_projection_count = 0ULL;
static uint64_t g_last_fragment_race_invalid_endpoint_count = 0ULL;
static uint64_t g_last_fragment_race_enqueue_failure_count = 0ULL;

// process_interrupt injury counters are boot-lifetime forensic testimony.
// Fresh physical ancestry snapshots their origins; rolling custody judges only
// the unsigned deltas from this boundary so pre-publication startup edges remain
// visible without poisoning every later PHOTONS fragment.
struct photons_interrupt_ancestry_t {
  bool valid = false;
  uint32_t callback_missing_origin = 0U;
  uint32_t inactive_edge_origin = 0U;
};

static photons_interrupt_ancestry_t g_interrupt_ancestry{};

static bool g_initialized = false;
static bool g_subscription_ok = false;
static bool g_interrupt_started = false;

static timepop_handle_t g_fragment_timer = TIMEPOP_INVALID_HANDLE;

static inline void photons_compiler_barrier(void) {
  __asm__ volatile("" ::: "memory");
}

// -----------------------------------------------------------------------------
// One-shot physical pulse testimony
// -----------------------------------------------------------------------------
//
// Foreground owns launch authorship. process_interrupt owns the pin-34
// first-instruction DWT coordinate. PHOTONS admits only the first comparator
// RISING edge after a PULSE arm. The ISR stores only scalar edge facts; any
// GNSS projection is deferred to foreground reporting. Each new PULSE replaces
// the prior one-shot report state.
//
struct photons_pulse_launch_state_t {
  bool     valid = false;
  uint32_t sequence = 0U;
  uint64_t requested_ns = 0ULL;
  uint32_t target_high_cycles = 0U;
  uint32_t start_dwt = 0U;
  bool     start_gnss_valid = false;
  uint64_t start_gnss_ns = 0ULL;
  uint32_t pulse_wall_cycles = 0U;
  uint32_t callback_count_start = 0U;
};

struct photons_pulse_receive_value_t {
  bool     seen = false;
  uint32_t pulse_sequence = 0U;
  uint32_t edge_sequence = 0U;
  uint32_t pps_sequence = 0U;
  uint32_t finish_dwt = 0U;
};

struct photons_pulse_receive_state_t {
  volatile uint32_t generation = 0U;
  photons_pulse_receive_value_t value{};
};

static photons_pulse_launch_state_t g_last_pulse_launch{};
static photons_pulse_receive_state_t g_last_pulse_receive{};
static volatile uint32_t g_pulse_armed_sequence = 0U;
static uint32_t g_pulse_sequence = 0U;

static void photons_pulse_receive_publish(
    const photons_pulse_receive_value_t& value) {
  g_last_pulse_receive.generation++;
  photons_compiler_barrier();
  g_last_pulse_receive.value = value;
  photons_compiler_barrier();
  g_last_pulse_receive.generation++;
}

static bool photons_pulse_receive_snapshot(
    photons_pulse_receive_value_t* out) {
  if (!out) return false;

  for (;;) {
    const uint32_t before = g_last_pulse_receive.generation;
    if (before & 1U) continue;

    photons_compiler_barrier();
    const photons_pulse_receive_value_t snapshot =
        g_last_pulse_receive.value;
    photons_compiler_barrier();

    const uint32_t after = g_last_pulse_receive.generation;
    if (before == after && !(after & 1U)) {
      *out = snapshot;
      return true;
    }
  }
}

static void photons_pulse_observe_edge(
    const interrupt_photodiode_edge_t& edge) {
  const uint32_t pulse_sequence = g_pulse_armed_sequence;
  if (pulse_sequence == 0U) return;

  // First eligible RISING edge wins. Clear the latch before projection so
  // comparator chatter cannot overwrite the first-return testimony.
  g_pulse_armed_sequence = 0U;
  photons_compiler_barrier();

  photons_pulse_receive_value_t value{};
  value.seen = true;
  value.pulse_sequence = pulse_sequence;
  value.edge_sequence = edge.sequence;
  value.pps_sequence = edge.pps_sequence;
  value.finish_dwt = edge.dwt_at_edge;
  photons_pulse_receive_publish(value);
}


// ============================================================================
// High-rate PHOTODIODE subscriber
// ============================================================================

static void photons_on_photodiode_edge(
    const interrupt_photodiode_edge_t& edge,
    const interrupt_photodiode_diag_t& /*diag*/,
    void* /*user_data*/) {

  photons_race_observe_edge(edge);
  photons_pulse_observe_edge(edge);

  // Begin seqlock write: odd generation means foreground must retry.
  g_photons_live.generation++;
  photons_compiler_barrier();

  photons_toy_capture_t& c = g_photons_live.capture;

  c.edge_count++;
  c.last_edge_sequence = edge.sequence;
  c.last_pps_sequence = edge.pps_sequence;
  c.last_dwt_at_edge = edge.dwt_at_edge;
  c.last_isr_entry_dwt_raw = edge.isr_entry_dwt_raw;
  c.isr_entry_to_edge_correction_cycles =
      edge.isr_entry_to_edge_correction_cycles;

  if (g_photons_live.previous_edge_valid) {
    const uint32_t interval =
        edge.dwt_at_edge - g_photons_live.previous_dwt_at_edge;

    c.interval_valid = true;
    c.last_interval_cycles = interval;

    if (c.min_interval_cycles == 0U || interval < c.min_interval_cycles) {
      c.min_interval_cycles = interval;
    }
    if (interval > c.max_interval_cycles) {
      c.max_interval_cycles = interval;
    }

    // Toy predictor only.  The final PHOTONS engine will decide its own
    // acceptance/exclusion semantics.
    if (g_photons_live.previous_interval_valid) {
      c.prediction_valid = true;
      c.prediction_cycles = g_photons_live.previous_interval_cycles;
      c.residual_cycles =
          (int32_t)(interval - g_photons_live.previous_interval_cycles);
    } else {
      c.prediction_valid = false;
      c.prediction_cycles = 0U;
      c.residual_cycles = 0;
    }

    g_photons_live.previous_interval_cycles = interval;
    g_photons_live.previous_interval_valid = true;
  }

  g_photons_live.previous_dwt_at_edge = edge.dwt_at_edge;
  g_photons_live.previous_edge_valid = true;

  photons_compiler_barrier();
  g_photons_live.generation++;  // commit: even generation
}

// ============================================================================
// Coherent toy snapshots
// ============================================================================

bool photons_toy_capture_snapshot(photons_toy_capture_t* out) {
  if (!out) return false;

  for (;;) {
    const uint32_t before = g_photons_live.generation;
    if (before & 1U) continue;

    photons_compiler_barrier();
    const photons_toy_capture_t snapshot = g_photons_live.capture;
    photons_compiler_barrier();

    const uint32_t after = g_photons_live.generation;
    if (before == after && !(after & 1U)) {
      *out = snapshot;
      return true;
    }
  }
}

bool photons_toy_fragment_snapshot(photons_toy_fragment_t* out) {
  if (!out) return false;
  *out = g_last_fragment;
  return true;
}


bool photons_fragment_snapshot(photons_fragment_snapshot_t* out) {
  if (!out) return false;
  *out = g_last_photons_fragment;
  return out->snapshot_ok;
}

// ============================================================================
// PHOTONS_FRAGMENT canonical publication
// ============================================================================

// PHOTONS_FRAGMENT is a bounded, single-owner foreground serializer. Every
// schema node that can exceed Payload's inline store is explicitly FIXED; the
// few proven-inline leaves remain ordinary local Payloads. The two large
// canonical parents keep RAM2 backing, while bounded nested scratch uses RAM1
// so deterministic serialization does not consume transport's RAM2 heap runway.
//
// Capacity exhaustion is a schema-contract failure, not an invitation to grow:
// Payload records FIXED_CAPACITY and fails hard.
#define PHOTONS_FRAGMENT_FIXED_RAM2(name, capacity)                        \
  alignas(Payload::FIXED_STORAGE_ALIGNMENT)                               \
  static uint8_t name##_storage[capacity] DMAMEM;                         \
  static Payload name DMAMEM(                                             \
      Payload::StorageMode::FIXED,                                        \
      name##_storage,                                                     \
      sizeof(name##_storage))

#define PHOTONS_FRAGMENT_FIXED_RAM1(name, capacity)                        \
  alignas(Payload::FIXED_STORAGE_ALIGNMENT)                               \
  static uint8_t name##_storage[capacity];                                \
  static Payload name DMAMEM(                                             \
      Payload::StorageMode::FIXED,                                        \
      name##_storage,                                                     \
      sizeof(name##_storage))

PHOTONS_FRAGMENT_FIXED_RAM2(g_photons_fragment_root, 12288U);
PHOTONS_FRAGMENT_FIXED_RAM2(g_photons_fragment_instrument, 12288U);

PHOTONS_FRAGMENT_FIXED_RAM1(g_photons_fragment_race, 1536U);
PHOTONS_FRAGMENT_FIXED_RAM1(g_photons_fragment_raw_cycles, 1024U);
PHOTONS_FRAGMENT_FIXED_RAM1(g_photons_fragment_projection, 1024U);
PHOTONS_FRAGMENT_FIXED_RAM1(g_photons_fragment_science, 3072U);
PHOTONS_FRAGMENT_FIXED_RAM1(g_photons_fragment_science_accepted, 768U);
PHOTONS_FRAGMENT_FIXED_RAM1(g_photons_fragment_science_excluded, 768U);
PHOTONS_FRAGMENT_FIXED_RAM1(g_photons_fragment_science_reasons, 512U);
PHOTONS_FRAGMENT_FIXED_RAM1(g_photons_fragment_stats, 4096U);
PHOTONS_FRAGMENT_FIXED_RAM1(g_photons_fragment_welford, 512U);
PHOTONS_FRAGMENT_FIXED_RAM1(g_photons_fragment_ppb_buckets, 1024U);
PHOTONS_FRAGMENT_FIXED_RAM1(g_photons_fragment_ppb_checkpoint, 2048U);
PHOTONS_FRAGMENT_FIXED_RAM1(g_photons_fragment_campaign, 768U);
PHOTONS_FRAGMENT_FIXED_RAM1(g_photons_fragment_campaign_stats, 512U);
PHOTONS_FRAGMENT_FIXED_RAM1(g_photons_fragment_recovery, 1024U);
PHOTONS_FRAGMENT_FIXED_RAM1(g_photons_fragment_interrupt, 1280U);

#undef PHOTONS_FRAGMENT_FIXED_RAM1
#undef PHOTONS_FRAGMENT_FIXED_RAM2


static void photons_payload_add_welford(
    Payload& parent,
    const char* name,
    const photons_fragment_welford_snapshot_t& w) {
  Payload& obj = g_photons_fragment_welford;
  obj.clear();
  obj.add("n", w.n);
  obj.add("mean", toFixedDecimal(w.mean, 6));
  obj.add("m2", toFixedDecimal(w.m2, 6));
  obj.add("stddev", toFixedDecimal(w.stddev, 6));
  obj.add("stderr", toFixedDecimal(w.stderr_value, 6));
  obj.add("min", toFixedDecimal(w.min, 6));
  obj.add("max", toFixedDecimal(w.max, 6));
  parent.add_object(name, obj);
  obj.clear();
}


static void photons_payload_add_ppb_value(
    Payload& parent,
    const char* name,
    const photons_fragment_ppb_value_snapshot_t& value) {
  if (value.sample_count == 0ULL) return;

  Payload obj;
  obj.add("sample_count", value.sample_count);
  obj.add("ppb", toFixedDecimal(value.ppb, 6));
  obj.add("residual_ns", toFixedDecimal(value.residual_ns, 6));
  parent.add_object(name, obj);
}


static void photons_payload_add_ppb_endpoint(
    Payload& parent,
    const char* name,
    const photons_fragment_ppb_endpoint_snapshot_t& endpoint) {
  Payload obj;
  obj.add("sequence", endpoint.sequence);
  obj.add("lap_count", endpoint.lap_count);
  obj.add("total_lap_gnss_ns", endpoint.total_lap_gnss_ns);
  parent.add_object(name, obj);
}


static void photons_payload_add_ppb_window_proof(
    Payload& parent,
    const char* name,
    const photons_fragment_ppb_window_proof_snapshot_t& proof) {
  Payload obj;
  obj.add("valid", proof.valid);
  obj.add("sample_count", proof.sample_count);
  if (proof.valid) {
    photons_payload_add_ppb_endpoint(obj, "anchor", proof.anchor);
  }
  parent.add_object(name, obj);
}


static Payload& photons_fragment_payload(
    const photons_fragment_snapshot_t& f) {
  Payload& root = g_photons_fragment_root;
  Payload& instrument = g_photons_fragment_instrument;
  Payload& race = g_photons_fragment_race;
  Payload& raw = g_photons_fragment_raw_cycles;
  Payload& projection = g_photons_fragment_projection;
  Payload& science = g_photons_fragment_science;
  Payload& accepted = g_photons_fragment_science_accepted;
  Payload& excluded = g_photons_fragment_science_excluded;
  Payload& reasons = g_photons_fragment_science_reasons;
  Payload& stats = g_photons_fragment_stats;
  Payload& ppb_buckets = g_photons_fragment_ppb_buckets;
  Payload& ppb_checkpoint = g_photons_fragment_ppb_checkpoint;
  Payload& campaign = g_photons_fragment_campaign;
  Payload& campaign_stats = g_photons_fragment_campaign_stats;
  Payload baseline;
  Payload& recovery = g_photons_fragment_recovery;
  Payload& interrupt = g_photons_fragment_interrupt;

  root.clear();
  instrument.clear();
  race.clear();
  raw.clear();
  projection.clear();
  science.clear();
  accepted.clear();
  excluded.clear();
  reasons.clear();
  stats.clear();
  ppb_buckets.clear();
  ppb_checkpoint.clear();
  campaign.clear();
  campaign_stats.clear();
  recovery.clear();
  interrupt.clear();

  root.add("schema", "PHOTONS_FRAGMENT_V1");
  root.add("sequence", f.sequence);
  root.add("publish_count", f.publish_count);

  instrument.add("schema", "PHOTONS_INSTRUMENT_V1");
  instrument.add("snapshot_ok", f.snapshot_ok);
  instrument.add("valid", f.valid);
  instrument.add("fragment_period_ns", f.fragment_period_ns);
  instrument.add("source", "PD200T_REAL_RACE");
  instrument.add("edge_count_total", f.edge_count_total);
  instrument.add("edges_this_fragment", f.edges_this_fragment);
  // Legacy wire fields remain zero while downstream callers migrate naming.
  instrument.add("train_count", f.train_count);
  instrument.add("dead_lap_count", f.dead_lap_count);
  instrument.add("raw_lap_count", f.raw_lap_count);
  instrument.add("projected_laps_this_fragment",
                 f.projected_laps_this_fragment);

  race.add("schema", "PHOTONS_RACE_V1");
  race.add("cadence_hz", f.race_cadence_hz);
  race.add("cadence_ns", PHOTONS_RACE_CADENCE_NS);
  race.add("pulse_ns", f.race_pulse_ns);
  race.add("launch_surrogate", "LD_ON_FALLING_EDGE");
  race.add("flight_interpretation", "ESTIMATED");
  race.add("cadence_tick_count_total", f.race_cadence_tick_count_total);
  race.add("cadence_ticks_this_fragment", f.race_cadence_ticks_this_fragment);
  race.add("attempt_count_total", f.race_attempt_count_total);
  race.add("attempts_this_fragment", f.race_attempts_this_fragment);
  race.add("completed_count_total", f.race_completed_count_total);
  race.add("completed_this_fragment", f.race_completed_this_fragment);
  race.add("missed_count_total", f.race_missed_count_total);
  race.add("missed_this_fragment", f.race_missed_this_fragment);
  race.add("skipped_not_quiet_total", f.race_skipped_not_quiet_total);
  race.add("skipped_not_quiet_this_fragment",
           f.race_skipped_not_quiet_this_fragment);
  race.add("skipped_projection_total", f.race_skipped_projection_total);
  race.add("skipped_projection_this_fragment",
           f.race_skipped_projection_this_fragment);
  race.add("invalid_endpoint_total", f.race_invalid_endpoint_total);
  race.add("invalid_endpoint_this_fragment",
           f.race_invalid_endpoint_this_fragment);
  race.add("enqueue_failure_total", f.race_enqueue_failure_total);
  race.add("enqueue_failure_this_fragment",
           f.race_enqueue_failure_this_fragment);
  photons_payload_add_welford(
      race, "flight_ns", f.race_flight_this_fragment);
  instrument.add_object("race", race);
  race.clear();

  raw.add("valid", f.raw_cycles.valid);
  raw.add("completed_lap_count", f.raw_cycles.completed_lap_count);
  raw.add("static_prediction_valid",
          f.raw_cycles.static_prediction_valid);
  raw.add("static_prediction_cycles",
          f.raw_cycles.static_prediction_cycles);
  raw.add("observed_cycles", f.raw_cycles.observed_cycles);
  raw.add("previous_observed_cycles",
          f.raw_cycles.previous_observed_cycles);
  raw.add("static_residual_cycles",
          f.raw_cycles.static_residual_cycles);
  raw.add("laps_this_fragment", f.raw_cycles.laps_this_fragment);
  raw.add("total_cycles_this_fragment",
          f.raw_cycles.total_cycles_this_fragment);
  raw.add("mean_cycles_this_fragment",
          toFixedDecimal(f.raw_cycles.mean_cycles_this_fragment, 6));
  raw.add("min_cycles_this_fragment",
          f.raw_cycles.min_cycles_this_fragment);
  raw.add("max_cycles_this_fragment",
          f.raw_cycles.max_cycles_this_fragment);
  raw.add("previous_fragment_mean_valid",
          f.raw_cycles.previous_fragment_mean_valid);
  raw.add("previous_fragment_mean_cycles",
          toFixedDecimal(f.raw_cycles.previous_fragment_mean_cycles, 6));
  raw.add("fragment_mean_residual_cycles",
          toFixedDecimal(f.raw_cycles.fragment_mean_residual_cycles, 6));
  instrument.add_object("raw_cycles", raw);
  raw.clear();

  projection.add("anchor_cache_valid",
                 f.projection.anchor_cache_valid);
  projection.add("anchor_pps_count",
                 f.projection.anchor_pps_count);
  projection.add("anchor_dwt_at_pps_vclock",
                 f.projection.anchor_dwt_at_pps_vclock);
  projection.add("anchor_dwt_cycles_per_second",
                 f.projection.anchor_dwt_cycles_per_second);
  projection.add("attempt_count", f.projection.attempt_count);
  projection.add("success_count", f.projection.success_count);
  projection.add("reject_count", f.projection.reject_count);
  projection.add("queue_overflow_count",
                 f.projection.queue_overflow_count);
  projection.add("last_valid", f.projection.last_valid);
  projection.add("last_pps_sequence",
                 f.projection.last_pps_sequence);
  projection.add("last_start_dwt", f.projection.last_start_dwt);
  projection.add("last_end_dwt", f.projection.last_end_dwt);
  projection.add("last_raw_cycles", f.projection.last_raw_cycles);
  projection.add("last_start_gnss_ns",
                 f.projection.last_start_gnss_ns);
  projection.add("last_end_gnss_ns",
                 f.projection.last_end_gnss_ns);
  projection.add("last_lap_gnss_ns",
                 f.projection.last_lap_gnss_ns);
  instrument.add_object("projection", projection);
  projection.clear();

  science.add("schema", "PHOTONS_SCIENCE_V2");
  science.add("valid", f.science.valid);
  science.add("candidate_count", f.science.candidate_count);
  science.add("candidates_this_fragment",
              f.science.candidates_this_fragment);

  accepted.add("count", f.science.accepted.count);
  accepted.add("count_this_fragment",
               f.science.accepted.count_this_fragment);
  photons_payload_add_welford(
      accepted, "raw_cycles", f.science.accepted.raw_cycles);
  photons_payload_add_welford(
      accepted, "projected_lap_ns", f.science.accepted.projected_lap_ns);
  science.add_object("accepted", accepted);
  accepted.clear();

  // Re-derive on the wire boundary as well: even the convenience fields in the
  // immutable snapshot are not granted exclusion-population authority.
  excluded.add(
      "count",
      photons_lap_science_excluded_count_from_reasons(
          f.science.exclusion_reasons));
  excluded.add(
      "count_this_fragment",
      photons_lap_science_excluded_this_fragment_from_reasons(
          f.science.exclusion_reasons));
  photons_payload_add_welford(
      excluded, "raw_cycles", f.science.excluded.raw_cycles);
  photons_payload_add_welford(
      excluded, "projected_lap_ns", f.science.excluded.projected_lap_ns);
  science.add_object("excluded", excluded);
  excluded.clear();

  reasons.add("projection_invalid",
              f.science.exclusion_reasons.projection_invalid);
  reasons.add("seed_disagreement",
              f.science.exclusion_reasons.seed_disagreement);
  reasons.add("raw_cycle_excursion",
              f.science.exclusion_reasons.raw_cycle_excursion);
  reasons.add("projection_invalid_this_fragment",
              f.science.exclusion_reasons.projection_invalid_this_fragment);
  reasons.add("seed_disagreement_this_fragment",
              f.science.exclusion_reasons.seed_disagreement_this_fragment);
  reasons.add("raw_cycle_excursion_this_fragment",
              f.science.exclusion_reasons.raw_cycle_excursion_this_fragment);
  science.add_object("exclusion_reasons", reasons);
  reasons.clear();

  science.add("predictor_valid", f.science.predictor_valid);
  science.add("predictor_cycles", f.science.predictor_cycles);
  science.add("gate_cycles", f.science.gate_cycles);
  science.add("gate_divisor", PHOTONS_SCIENCE_GATE_DIVISOR);
  science.add("gate_min_cycles", PHOTONS_SCIENCE_GATE_MIN_CYCLES);
  science.add("reject_streak", f.science.reject_streak);
  science.add("max_reject_streak", f.science.max_reject_streak);
  science.add("seed_pending", f.science.seed_pending);
  science.add("seed_pending_candidate_index",
              f.science.seed_pending_candidate_index);
  science.add("seed_pending_raw_cycles",
              f.science.seed_pending_raw_cycles);
  science.add("seed_pending_lap_gnss_ns",
              f.science.seed_pending_lap_gnss_ns);
  science.add("last_candidate_index",
              f.science.last_candidate_index);
  science.add("last_disposition_id",
              (uint32_t)f.science.last_disposition_id);
  science.add("last_disposition",
              photons_lap_science_disposition_name(
                  f.science.last_disposition_id));
  science.add("last_reason_code",
              (uint32_t)f.science.last_reason_code);
  science.add("last_reason",
              photons_lap_science_reason_name(
                  f.science.last_reason_code));
  science.add("last_projection_valid",
              f.science.last_projection_valid);
  science.add("last_pps_sequence",
              f.science.last_pps_sequence);
  science.add("last_observed_cycles",
              f.science.last_observed_cycles);
  science.add("last_prediction_cycles",
              f.science.last_prediction_cycles);
  science.add("last_residual_cycles",
              f.science.last_residual_cycles);
  science.add("last_gate_cycles",
              f.science.last_gate_cycles);
  science.add("last_lap_gnss_ns",
              f.science.last_lap_gnss_ns);
  instrument.add_object("science", science);
  science.clear();

  stats.add("schema", "PHOTONS_INSTRUMENT_STATS_V1");
  stats.add("ppb_semantics", "LAP_BASELINE_NS_OFFSET_V1");
  stats.add("valid", f.stats.valid);
  stats.add("reset_count", f.stats.reset_count);
  stats.add("update_count", f.stats.update_count);
  stats.add("lap_baseline_fs", f.stats.lap_baseline_fs);
  stats.add("lap_baseline_ns",
            toFixedDecimal((double)f.stats.lap_baseline_fs / 1000000.0, 6));
  // Deprecated compatibility mirror; reference arithmetic never uses this field.
  stats.add("standard_lap_ps", f.stats.standard_lap_ps);
  stats.add("standard_lap_ns",
            toFixedDecimal((double)f.stats.standard_lap_ps / 1000.0, 3));
  stats.add("custody_lap_count", g_photons_custody_lap_count);
  stats.add("custody_total_lap_gnss_ns", g_photons_custody_total_lap_gnss_ns);
  stats.add("lap_count", f.stats.lap_count);
  stats.add("total_lap_gnss_ns", f.stats.total_lap_gnss_ns);
  stats.add("mean_lap_ns", toFixedDecimal(f.stats.mean_lap_ns, 6));
  stats.add("race_count", f.stats.lap_count);
  stats.add("mean_flight_ns", toFixedDecimal(f.stats.mean_lap_ns, 6));
  photons_payload_add_welford(
      stats, "lap_time", f.stats.lap_time_welford);

  photons_payload_add_ppb_value(
      ppb_buckets, "10_min", f.stats.ppb_buckets.minute_10);
  photons_payload_add_ppb_value(
      ppb_buckets, "60_min", f.stats.ppb_buckets.minute_60);
  photons_payload_add_ppb_value(
      ppb_buckets, "8_hour", f.stats.ppb_buckets.hour_8);
  photons_payload_add_ppb_value(
      ppb_buckets, "24_hour", f.stats.ppb_buckets.hour_24);
  photons_payload_add_ppb_value(
      ppb_buckets, "total", f.stats.ppb_buckets.total);
  stats.add_object("ppb_buckets", ppb_buckets);
  ppb_buckets.clear();

  stats.add("rolling_ppb_current_sequence",
            f.stats.rolling_ppb_current_sequence);
  stats.add("rolling_ppb_endpoint_admitted",
            f.stats.rolling_ppb_endpoint_admitted);
  stats.add("rolling_ppb_interval_advanced",
            f.stats.rolling_ppb_interval_advanced);

  const photons_fragment_ppb_checkpoint_delta_snapshot_t& checkpoint =
      f.stats.rolling_ppb_checkpoint;
  ppb_checkpoint.add("schema", "PHOTONS_PPB_CHECKPOINT_DELTA_V1");
  ppb_checkpoint.add("valid", checkpoint.valid);
  ppb_checkpoint.add("rolling_sequence", checkpoint.rolling_sequence);
  ppb_checkpoint.add("second_count", checkpoint.second_count);
  ppb_checkpoint.add("minute_count", checkpoint.minute_count);
  ppb_checkpoint.add("last_minute_key", checkpoint.last_minute_key);
  ppb_checkpoint.add("origin_valid", checkpoint.origin_valid);
  if (checkpoint.valid) {
    photons_payload_add_ppb_endpoint(
        ppb_checkpoint, "current", checkpoint.current);
  }
  if (checkpoint.origin_valid) {
    photons_payload_add_ppb_endpoint(
        ppb_checkpoint, "origin", checkpoint.origin);
  }
  photons_payload_add_ppb_window_proof(
      ppb_checkpoint, "10_min", checkpoint.minute_10);
  photons_payload_add_ppb_window_proof(
      ppb_checkpoint, "60_min", checkpoint.minute_60);
  photons_payload_add_ppb_window_proof(
      ppb_checkpoint, "8_hour", checkpoint.hour_8);
  photons_payload_add_ppb_window_proof(
      ppb_checkpoint, "24_hour", checkpoint.hour_24);
  ppb_checkpoint.add("second_append_valid", checkpoint.second_append_valid);
  if (checkpoint.second_append_valid) {
    photons_payload_add_ppb_endpoint(
        ppb_checkpoint, "second_append", checkpoint.second_append);
  }
  ppb_checkpoint.add("minute_append_valid", checkpoint.minute_append_valid);
  if (checkpoint.minute_append_valid) {
    photons_payload_add_ppb_endpoint(
        ppb_checkpoint, "minute_append", checkpoint.minute_append);
  }
  stats.add_object("rolling_ppb_checkpoint", ppb_checkpoint);
  ppb_checkpoint.clear();

  instrument.add_object("stats", stats);
  stats.clear();

  baseline.add("present", f.baseline.present);
  baseline.add("residual_valid", f.baseline.residual_valid);
  if (f.baseline.present) {
    baseline.add("baseline_mean_lap_ns",
                 toFixedDecimal(f.baseline.baseline_mean_lap_ns, 6));
  }
  if (f.baseline.residual_valid) {
    baseline.add("mean_residual_ns",
                 toFixedDecimal(f.baseline.mean_residual_ns, 6));
  }
  instrument.add_object("baseline", baseline);

  recovery.add("restored", f.recovery.restored);
  recovery.add("proof_pending", f.recovery.proof_pending);
  recovery.add("proof_advanced", f.recovery.proof_advanced);
  recovery.add("proof_committed", f.recovery.proof_committed);
  recovery.add("generation", f.recovery.generation);
  recovery.add("source_sequence", f.recovery.source_sequence);
  recovery.add("source_publish_count", f.recovery.source_publish_count);
  recovery.add("source_reset_count", f.recovery.source_reset_count);
  recovery.add("source_update_count", f.recovery.source_update_count);
  recovery.add("source_lap_count", f.recovery.source_lap_count);
  recovery.add("source_total_lap_gnss_ns",
               f.recovery.source_total_lap_gnss_ns);
  recovery.add("source_custody_lap_count",
               f.recovery.source_custody_lap_count);
  recovery.add("source_custody_total_lap_gnss_ns",
               f.recovery.source_custody_total_lap_gnss_ns);
  recovery.add("accepted_lap_delta", f.recovery.accepted_lap_delta);
  recovery.add("custody_lap_delta", f.recovery.custody_lap_delta);
  recovery.add("fresh_physical_ancestry",
               f.recovery.fresh_physical_ancestry);
  recovery.add("raw_lap_ring_restored",
               f.recovery.raw_lap_ring_restored);
  recovery.add("partial_lap_restored",
               f.recovery.partial_lap_restored);
  recovery.add("pending_seed_restored",
               f.recovery.pending_seed_restored);
  recovery.add("predictor_restored", f.recovery.predictor_restored);
  recovery.add("in_flight_train_restored",
               f.recovery.in_flight_train_restored);
  instrument.add_object("recovery", recovery);
  recovery.clear();

  interrupt.add("subscribed", f.interrupt_subscribed);
  interrupt.add("active", f.interrupt_active);
  interrupt.add("irq_count", f.interrupt_irq_count);
  interrupt.add("callback_count", f.interrupt_callback_count);
  interrupt.add("callback_missing_count",
                f.interrupt_callback_missing_count);
  interrupt.add("ancestry_baseline_valid",
                f.interrupt_ancestry_baseline_valid);
  interrupt.add("callback_missing_count_origin",
                f.interrupt_callback_missing_origin);
  interrupt.add("callback_missing_count_since_ancestry",
                f.interrupt_callback_missing_since_ancestry);
  interrupt.add("inactive_edge_count",
                f.interrupt_inactive_edge_count);
  interrupt.add("inactive_edge_count_origin",
                f.interrupt_inactive_edge_origin);
  interrupt.add("inactive_edge_count_since_ancestry",
                f.interrupt_inactive_edge_since_ancestry);
  interrupt.add("source_pin", f.interrupt_source_pin);
  interrupt.add("last_callback_wall_cycles",
                f.interrupt_last_callback_wall_cycles);
  interrupt.add("max_callback_wall_cycles",
                f.interrupt_max_callback_wall_cycles);
  interrupt.add("blocker_trace_count", f.interrupt_blocker_trace_count);
  interrupt.add("blocked_qtimer1_count", f.interrupt_blocked_qtimer1_count);
  interrupt.add("blocked_ocxo1_count", f.interrupt_blocked_ocxo1_count);
  interrupt.add("blocked_ocxo2_count", f.interrupt_blocked_ocxo2_count);
  interrupt.add("last_blocker_wall_cycles",
                f.interrupt_last_blocker_wall_cycles);
  interrupt.add("max_blocker_wall_cycles",
                f.interrupt_max_blocker_wall_cycles);
  interrupt.add("last_qtimer_pending_at_entry_mask",
                f.interrupt_last_qtimer_pending_at_entry_mask);
  interrupt.add("last_qtimer_pending_at_exit_mask",
                f.interrupt_last_qtimer_pending_at_exit_mask);
  instrument.add_object("interrupt", interrupt);
  interrupt.clear();

  root.add_object("photons", instrument);
  instrument.clear();

  if (f.campaign.present) {
    campaign.add("schema", "LANTERN_FRAGMENT_V1");
    campaign.add("campaign", f.campaign.campaign);
    campaign.add("start_after_sequence", f.campaign.start_after_sequence);
    campaign.add("public_count", f.campaign.public_count);
    campaign.add("final", f.campaign.final);
    if (f.campaign.final) {
      campaign.add("stop_after_sequence", f.campaign.stop_after_sequence);
    }

    campaign_stats.add("lap_count", f.campaign.lap_count);
    campaign_stats.add("total_lap_gnss_ns", f.campaign.total_lap_gnss_ns);
    if (f.campaign.lap_count != 0ULL) {
      campaign_stats.add("mean_lap_ns",
                         toFixedDecimal(f.campaign.mean_lap_ns, 6));
      campaign_stats.add("race_count", f.campaign.lap_count);
      campaign_stats.add("mean_flight_ns",
                         toFixedDecimal(f.campaign.mean_lap_ns, 6));
      campaign_stats.add("sample_count", f.campaign.ppb.sample_count);
      campaign_stats.add("ppb", toFixedDecimal(f.campaign.ppb.ppb, 6));
      campaign_stats.add("residual_ns",
                         toFixedDecimal(f.campaign.ppb.residual_ns, 6));
    }
    campaign.add_object("stats", campaign_stats);
    campaign_stats.clear();
    root.add_object("campaign", campaign);
    campaign.clear();
  }

  return root;
}


static void photons_fragment_tick(
    timepop_ctx_t* /*ctx*/,
    timepop_diag_t* /*diag*/,
    void* /*user_data*/) {

  // PHOTONS_FRAGMENT is ready-to-eat testimony.  Until the Pi supplies the
  // required standard lap, PHOTONS has no authority to publish interpreted
  // optical statistics.
  if (!g_standard_lap_configured || g_lap_baseline_fs == 0ULL ||
      !g_photons_recovery.publication_started) return;

  // Refresh the PHOTONS-owned immutable copy for laps that will arrive after
  // this boundary.  Records already in the ring carry the anchor that was
  // current when their physical DWT endpoints were observed.
  photons_projection_anchor_refresh();

  photons_toy_capture_t capture{};
  if (!photons_toy_capture_snapshot(&capture)) return;

  interrupt_photodiode_diag_t interrupt_diag{};
  if (!interrupt_photodiode_snapshot(&interrupt_diag) ||
      !g_interrupt_ancestry.valid) {
    __builtin_trap();
  }
  const uint32_t interrupt_callback_missing_since_ancestry =
      interrupt_diag.callback_missing_count -
      g_interrupt_ancestry.callback_missing_origin;
  const uint32_t interrupt_inactive_edge_since_ancestry =
      interrupt_diag.inactive_edge_count -
      g_interrupt_ancestry.inactive_edge_origin;

  const photons_fragment_drain_result_t drain =
      photons_drain_raw_laps();

  photons_projection_anchor_value_t current_anchor{};
  const bool current_anchor_snapshot_ok =
      photons_projection_anchor_snapshot(current_anchor);
  g_projection_state.anchor_cache_valid =
      current_anchor_snapshot_ok && current_anchor.valid;
  g_projection_state.anchor_pps_count = current_anchor.pps_count;
  g_projection_state.anchor_dwt_at_pps_vclock =
      current_anchor.dwt_at_pps_vclock;
  g_projection_state.anchor_dwt_cycles_per_second =
      current_anchor.dwt_cycles_per_second;

  photons_fragment_snapshot_t fragment{};
  fragment.snapshot_ok = true;
  fragment.sequence = ++g_fragment_sequence;
  fragment.publish_count = g_publish_count + 1U;
  fragment.fragment_period_ns = PHOTONS_FRAGMENT_PERIOD_NS;

  fragment.edge_count_total = capture.edge_count;
  fragment.edges_this_fragment =
      capture.edge_count - g_last_published_edge_count;

  fragment.train_count = 0U;
  fragment.dead_lap_count = 0U;
  fragment.raw_lap_count = g_raw_cycles_state.completed_lap_count;
  fragment.projected_laps_this_fragment = drain.projected_laps;

  fragment.race_cadence_hz = PHOTONS_RACE_CADENCE_HZ;
  fragment.race_pulse_ns = PHOTONS_RACE_PULSE_NS;
  fragment.race_cadence_tick_count_total = g_photons_race.cadence_tick_count;
  fragment.race_cadence_ticks_this_fragment = (uint32_t)(
      g_photons_race.cadence_tick_count -
      g_last_fragment_race_cadence_tick_count);
  fragment.race_attempt_count_total = g_photons_race.attempt_count;
  fragment.race_attempts_this_fragment = (uint32_t)(
      g_photons_race.attempt_count - g_last_fragment_race_attempt_count);
  fragment.race_completed_count_total = g_photons_race.completed_count;
  fragment.race_completed_this_fragment = (uint32_t)(
      g_photons_race.completed_count - g_last_fragment_race_completed_count);
  fragment.race_missed_count_total = g_photons_race.missed_count;
  fragment.race_missed_this_fragment = (uint32_t)(
      g_photons_race.missed_count - g_last_fragment_race_missed_count);
  fragment.race_skipped_not_quiet_total =
      g_photons_race.skipped_not_quiet_count;
  fragment.race_skipped_not_quiet_this_fragment = (uint32_t)(
      g_photons_race.skipped_not_quiet_count -
      g_last_fragment_race_skipped_not_quiet_count);
  fragment.race_skipped_projection_total =
      g_photons_race.skipped_projection_count;
  fragment.race_skipped_projection_this_fragment = (uint32_t)(
      g_photons_race.skipped_projection_count -
      g_last_fragment_race_skipped_projection_count);
  fragment.race_invalid_endpoint_total =
      g_photons_race.invalid_endpoint_count;
  fragment.race_invalid_endpoint_this_fragment = (uint32_t)(
      g_photons_race.invalid_endpoint_count -
      g_last_fragment_race_invalid_endpoint_count);
  fragment.race_enqueue_failure_total = g_photons_race.enqueue_failure_count;
  fragment.race_enqueue_failure_this_fragment = (uint32_t)(
      g_photons_race.enqueue_failure_count -
      g_last_fragment_race_enqueue_failure_count);
  fragment.race_flight_this_fragment =
      photons_welford_snapshot(drain.projected_flight_welford);

  fragment.raw_cycles = g_raw_cycles_state;
  fragment.projection = g_projection_state;
  fragment.science = g_photons_lap_science_state;
  // Materialize schema convenience counts only in the immutable publication
  // snapshot.  They are functions of the reason ledger, never mutable authority.
  photons_lap_science_materialize_derived_exclusion_counts(fragment.science);

  fragment.stats.reset_count = g_photons_stats_reset_count;
  fragment.stats.update_count = ++g_photons_stats_update_count;
  fragment.stats.lap_baseline_fs = g_lap_baseline_fs;
  fragment.stats.standard_lap_ps = g_standard_lap_ps;
  fragment.stats.lap_count = g_lap_time_welford.n;
  fragment.stats.total_lap_gnss_ns = g_total_lap_gnss_ns;
  fragment.stats.mean_lap_ns =
      (g_lap_time_welford.n != 0ULL)
          ? (double)g_total_lap_gnss_ns /
                (double)g_lap_time_welford.n
          : 0.0;
  fragment.stats.lap_time_welford =
      photons_welford_snapshot(g_lap_time_welford);
  fragment.stats.valid =
      g_lap_time_welford.n != 0ULL &&
      !g_raw_lap_ring_data_loss;

  // A lawful one-second cumulative endpoint may contain zero accepted laps.
  // Ordinary SCIENCE_EXCLUDE observations therefore do not break rolling
  // ancestry. Actual custody loss does: raw-ring overflow or a post-ancestry
  // missing/inactive PHOTODIODE callback clears rolling history rather than
  // bridging it. Boot-lifetime injury counters remain visible separately.
  const bool ppb_endpoint_admitted =
      !g_raw_lap_ring_data_loss &&
      interrupt_callback_missing_since_ancestry == 0U &&
      interrupt_inactive_edge_since_ancestry == 0U;
  photons_ppb_windows_note_endpoint(
      fragment.stats.update_count,
      ppb_endpoint_admitted,
      fragment.stats.lap_count,
      fragment.stats.total_lap_gnss_ns);
  fragment.stats.ppb_buckets = photons_ppb_buckets_snapshot();
  fragment.stats.rolling_ppb_current_sequence =
      g_photons_ppb_current_sequence;
  fragment.stats.rolling_ppb_endpoint_admitted =
      g_photons_ppb_endpoint_admitted;
  fragment.stats.rolling_ppb_interval_advanced =
      g_photons_ppb_interval_advanced;
  fragment.stats.rolling_ppb_checkpoint =
      photons_ppb_checkpoint_delta_snapshot();

  // Optional LANTERN campaign testimony is firmware-authored from the same
  // cumulative accepted-lap authority as TOTAL.  START/STOP never reset the
  // always-on instrument; they only select a recording-relative subtraction
  // origin, exactly like CLOCKS campaign offsets.
  fragment.campaign = photons_campaign_snapshot(fragment.sequence);

  // LAP_BASELINE_NS is an operator-authored coordinate reference, independent
  // of campaign-to-campaign baseline provenance. Re-reference the current exact
  // accepted N/T without mutating any physical/statistical custody.
  fragment.baseline = photons_fragment_baseline_snapshot_t{};
  fragment.baseline.present =
      g_standard_lap_configured && g_lap_baseline_fs != 0ULL;
  if (fragment.baseline.present) {
    fragment.baseline.baseline_mean_lap_ns =
        (double)g_lap_baseline_fs / 1000000.0;
    if (fragment.stats.lap_count != 0ULL) {
      fragment.baseline.residual_valid = true;
      fragment.baseline.mean_residual_ns =
          photons_residual_ns_from_population(
              fragment.stats.total_lap_gnss_ns, fragment.stats.lap_count);
    }
  }
  fragment.recovery = photons_recovery_snapshot();

  fragment.interrupt_subscribed = interrupt_diag.subscribed;
  fragment.interrupt_active = interrupt_diag.active;
  fragment.interrupt_irq_count = interrupt_diag.irq_count;
  fragment.interrupt_callback_count = interrupt_diag.callback_count;
  fragment.interrupt_callback_missing_count =
      interrupt_diag.callback_missing_count;
  fragment.interrupt_ancestry_baseline_valid =
      g_interrupt_ancestry.valid;
  fragment.interrupt_callback_missing_origin =
      g_interrupt_ancestry.callback_missing_origin;
  fragment.interrupt_callback_missing_since_ancestry =
      interrupt_callback_missing_since_ancestry;
  fragment.interrupt_inactive_edge_count =
      interrupt_diag.inactive_edge_count;
  fragment.interrupt_inactive_edge_origin =
      g_interrupt_ancestry.inactive_edge_origin;
  fragment.interrupt_inactive_edge_since_ancestry =
      interrupt_inactive_edge_since_ancestry;
  fragment.interrupt_source_pin = interrupt_diag.source_pin;
  fragment.interrupt_last_callback_wall_cycles =
      interrupt_diag.last_callback_wall_cycles;
  fragment.interrupt_max_callback_wall_cycles =
      interrupt_diag.max_callback_wall_cycles;
  fragment.interrupt_blocker_trace_count = interrupt_diag.blocker_trace_count;
  fragment.interrupt_blocked_qtimer1_count = interrupt_diag.blocked_qtimer1_count;
  fragment.interrupt_blocked_ocxo1_count = interrupt_diag.blocked_ocxo1_count;
  fragment.interrupt_blocked_ocxo2_count = interrupt_diag.blocked_ocxo2_count;
  fragment.interrupt_last_blocker_wall_cycles =
      interrupt_diag.last_blocker_wall_cycles;
  fragment.interrupt_max_blocker_wall_cycles =
      interrupt_diag.max_blocker_wall_cycles;
  fragment.interrupt_last_qtimer_pending_at_entry_mask =
      interrupt_diag.last_qtimer_pending_at_entry_mask;
  fragment.interrupt_last_qtimer_pending_at_exit_mask =
      interrupt_diag.last_qtimer_pending_at_exit_mask;

  fragment.valid =
      fragment.stats.valid &&
      fragment.projection.anchor_cache_valid &&
      fragment.interrupt_ancestry_baseline_valid &&
      fragment.interrupt_callback_missing_since_ancestry == 0U &&
      fragment.interrupt_inactive_edge_since_ancestry == 0U;

  // Keep the historical toy snapshot alive only as a compatibility shell.
  photons_toy_fragment_t toy{};
  toy.sequence = fragment.sequence;
  toy.publish_count = fragment.publish_count;
  toy.edge_count_total = capture.edge_count;
  toy.edges_this_second = fragment.edges_this_fragment;
  toy.capture = capture;
  toy.interrupt_irq_count = interrupt_diag.irq_count;
  toy.interrupt_callback_count = interrupt_diag.callback_count;
  toy.interrupt_callback_missing_count =
      interrupt_diag.callback_missing_count;
  toy.interrupt_inactive_edge_count =
      interrupt_diag.inactive_edge_count;
  toy.interrupt_source_pin = interrupt_diag.source_pin;
  toy.interrupt_last_callback_wall_cycles =
      interrupt_diag.last_callback_wall_cycles;
  toy.interrupt_max_callback_wall_cycles =
      interrupt_diag.max_callback_wall_cycles;

  // Author both foreground snapshots before transport admission so REPORT can
  // show exactly what PHOTONS attempted to publish.
  g_last_fragment = toy;
  g_last_photons_fragment = fragment;
  g_last_published_edge_count = capture.edge_count;
  g_last_fragment_race_cadence_tick_count = g_photons_race.cadence_tick_count;
  g_last_fragment_race_attempt_count = g_photons_race.attempt_count;
  g_last_fragment_race_completed_count = g_photons_race.completed_count;
  g_last_fragment_race_missed_count = g_photons_race.missed_count;
  g_last_fragment_race_skipped_not_quiet_count =
      g_photons_race.skipped_not_quiet_count;
  g_last_fragment_race_skipped_projection_count =
      g_photons_race.skipped_projection_count;
  g_last_fragment_race_invalid_endpoint_count =
      g_photons_race.invalid_endpoint_count;
  g_last_fragment_race_enqueue_failure_count =
      g_photons_race.enqueue_failure_count;

  Payload& payload = photons_fragment_payload(fragment);
  if (publish("PHOTONS_FRAGMENT", payload)) {
    g_publish_count++;
    g_last_fragment.publish_count = g_publish_count;
    g_last_photons_fragment.publish_count = g_publish_count;
    if (fragment.recovery.restored &&
        fragment.recovery.proof_pending &&
        fragment.recovery.proof_advanced &&
        !g_photons_recovery.proof_advanced_published) {
      g_photons_recovery.proof_sequence = fragment.sequence;
      g_photons_recovery.proof_update_count = fragment.stats.update_count;
      g_photons_recovery.proof_advanced_published = true;
    }
    photons_campaign_commit_after_publish(fragment);
    photons_stats_reset_commit_after_publish();
  } else {
    g_publish_reject_count++;
  }
}


static void photons_recovery_clear_physical_ancestry(void) {
  // Discard every observation that could have begun before the recovery
  // boundary.  Aggregate statistics are installed separately; none of these
  // boot-local physical facts may cross the outage.
  photons_memory_barrier();
  g_raw_lap_ring_read = g_raw_lap_ring_write;
  g_raw_lap_ring_overflow_count = 0U;
  g_raw_lap_ring_data_loss = false;
  g_photons_lap_science_seed_pending = photons_lap_science_candidate_t{};
  g_previous_fragment_mean_cycles_valid = false;
  g_previous_fragment_mean_cycles = 0.0;

  g_photons_live.generation++;
  photons_compiler_barrier();
  g_photons_live.previous_edge_valid = false;
  g_photons_live.previous_dwt_at_edge = 0U;
  g_photons_live.previous_interval_valid = false;
  g_photons_live.previous_interval_cycles = 0U;
  g_photons_live.capture.interval_valid = false;
  g_photons_live.capture.last_interval_cycles = 0U;
  g_photons_live.capture.prediction_valid = false;
  g_photons_live.capture.prediction_cycles = 0U;
  g_photons_live.capture.residual_cycles = 0;
  photons_compiler_barrier();
  g_photons_live.generation++;

  photons_race_prepare();
  g_projection_anchor_cache = photons_projection_anchor_cache_t{};
  photons_projection_anchor_refresh();

  interrupt_photodiode_diag_t interrupt_diag{};
  g_interrupt_ancestry = photons_interrupt_ancestry_t{};
  if (!interrupt_photodiode_snapshot(&interrupt_diag)) __builtin_trap();
  g_interrupt_ancestry.callback_missing_origin =
      interrupt_diag.callback_missing_count;
  g_interrupt_ancestry.inactive_edge_origin =
      interrupt_diag.inactive_edge_count;
  photons_memory_barrier();
  g_interrupt_ancestry.valid = true;

  photons_toy_capture_t capture{};
  if (!photons_toy_capture_snapshot(&capture)) __builtin_trap();
  g_last_published_edge_count = capture.edge_count;
  g_last_fragment_race_cadence_tick_count = 0ULL;
  g_last_fragment_race_attempt_count = 0ULL;
  g_last_fragment_race_completed_count = 0ULL;
  g_last_fragment_race_missed_count = 0ULL;
  g_last_fragment_race_skipped_not_quiet_count = 0ULL;
  g_last_fragment_race_skipped_projection_count = 0ULL;
  g_last_fragment_race_invalid_endpoint_count = 0ULL;
  g_last_fragment_race_enqueue_failure_count = 0ULL;
  g_last_fragment = photons_toy_fragment_t{};
  g_last_photons_fragment = photons_fragment_snapshot_t{};
}


static void photons_start_publication(void) {
  if (!g_standard_lap_configured || g_lap_baseline_fs == 0ULL ||
      g_photons_recovery.publication_started ||
      g_fragment_timer != TIMEPOP_INVALID_HANDLE ||
      !g_photons_ppb_previous_endpoint_valid ||
      !g_interrupt_ancestry.valid) {
    __builtin_trap();
  }

  g_fragment_timer = timepop_arm(
      PHOTONS_FRAGMENT_PERIOD_NS,
      true,
      photons_fragment_tick,
      nullptr,
      "PHOTONS_FRAGMENT");
  if (g_fragment_timer == TIMEPOP_INVALID_HANDLE) __builtin_trap();

  g_photons_recovery.publication_started = true;
  // A commissioning PULSE may have been armed before recovery/publication. It
  // has no authority to consume an edge once the recurring race engine starts.
  g_pulse_armed_sequence = 0U;
  photons_pulse_receive_publish(photons_pulse_receive_value_t{});
  g_last_pulse_launch = photons_pulse_launch_state_t{};
  photons_race_start();
}


// ============================================================================
// Initialization
// ============================================================================

FLASHMEM void process_photons_init(void) {
  if (g_initialized) return;

  g_photons_live = photons_live_state_t{};
  g_last_fragment = photons_toy_fragment_t{};
  g_last_photons_fragment = photons_fragment_snapshot_t{};
  g_fragment_sequence = 0U;
  g_publish_count = 0U;
  g_publish_reject_count = 0U;
  g_last_published_edge_count = 0U;
  g_last_fragment_race_cadence_tick_count = 0ULL;
  g_last_fragment_race_attempt_count = 0ULL;
  g_last_fragment_race_completed_count = 0ULL;
  g_last_fragment_race_missed_count = 0ULL;
  g_last_fragment_race_skipped_not_quiet_count = 0ULL;
  g_last_fragment_race_skipped_projection_count = 0ULL;
  g_last_fragment_race_invalid_endpoint_count = 0ULL;
  g_last_fragment_race_enqueue_failure_count = 0ULL;
  g_interrupt_ancestry = photons_interrupt_ancestry_t{};
  g_fragment_timer = TIMEPOP_INVALID_HANDLE;
  g_photons_recovery_protocol = photons_recovery_protocol_t{};
  g_photons_recovery = photons_recovery_runtime_t{};
  g_standard_lap_configured = false;
  g_lap_baseline_fs = 0ULL;
  g_standard_lap_ps = 0ULL;
  g_photons_campaign_state = photons_campaign_state_t::STOPPED;
  g_photons_campaign_name[0] = '\0';
  g_photons_campaign_origin_lap_count = 0ULL;
  g_photons_campaign_origin_total_lap_gnss_ns = 0ULL;
  g_photons_campaign_start_after_sequence = 0U;
  g_photons_campaign_public_count = 0U;
  g_photons_campaign_start_request_count = 0U;
  g_photons_campaign_start_commit_count = 0U;
  g_photons_campaign_stop_request_count = 0U;
  g_photons_campaign_stop_commit_count = 0U;
  g_photons_flash_cut_campaign_name[0] = '\0';
  g_photons_flash_cut_request_count = 0U;
  g_photons_flash_cut_commit_count = 0U;
  g_photons_flash_cut_reject_count = 0U;

  g_projection_anchor_cache = photons_projection_anchor_cache_t{};
  g_raw_lap_ring_write = 0U;
  g_raw_lap_ring_read = 0U;
  g_raw_lap_ring_overflow_count = 0U;
  g_raw_lap_ring_data_loss = false;
  g_raw_cycles_state = photons_fragment_raw_cycles_snapshot_t{};
  g_projection_state = photons_fragment_projection_snapshot_t{};
  g_photons_lap_science_state = photons_lap_science_snapshot_t{};
  g_photons_lap_science_seed_pending = photons_lap_science_candidate_t{};
  photons_welford_reset(g_lap_time_welford);
  photons_welford_reset(g_accepted_raw_cycles_welford);
  photons_welford_reset(g_excluded_raw_cycles_welford);
  photons_welford_reset(g_excluded_lap_time_welford);
  g_total_lap_gnss_ns = 0ULL;
  g_photons_stats_reset_count = 0U;
  g_photons_stats_reset_pending = false;
  g_photons_stats_reset_request_count = 0U;
  g_photons_stats_reset_commit_count = 0U;
  g_photons_custody_lap_count = 0ULL;
  g_photons_custody_total_lap_gnss_ns = 0ULL;
  g_photons_stats_update_count = 0U;
  photons_ppb_windows_clear_history();
  g_previous_fragment_mean_cycles_valid = false;
  g_previous_fragment_mean_cycles = 0.0;

  photons_race_prepare();
  photons_laser_initialize_hardware();
  photons_emit_laser_initialization_event();

  interrupt_photodiode_subscription_t subscription{};
  subscription.on_edge = photons_on_photodiode_edge;
  subscription.user_data = nullptr;

  g_subscription_ok = interrupt_photodiode_subscribe(subscription);
  g_interrupt_started =
      g_subscription_ok &&
      interrupt_start(interrupt_subscriber_kind_t::PHOTODIODE);

  // Prime the PHOTONS-owned projection cache before the race engine begins. TIME
  // may still be initializing; invalidity is preserved and early cadence cells
  // are skipped rather than projected through an invented ruler.
  photons_projection_anchor_refresh();

  // The fragment publisher and real race cadence are intentionally not started
  // here. SET_LAP_BASELINE_NS installs the current operator reference. A later
  // RECOVERY_COMMIT or RECOVERY_COLD_START establishes the statistical origin,
  // clears boot-local physical ancestry, and starts both timers exactly once.
  g_initialized = true;
}

// ============================================================================
// Commands
// ============================================================================

// Parse the operator-authored LAP_BASELINE_NS exactly into integer femtoseconds.
// One or more integer digits, '.', and exactly six fractional digits are required.
static bool photons_parse_lap_baseline_ns(const char* text,
                                          uint64_t& lap_baseline_fs) {
  lap_baseline_fs = 0ULL;
  if (!text || !*text) return false;

  const char* p = text;
  uint64_t whole_ns = 0ULL;
  uint32_t integer_digits = 0U;
  while (*p >= '0' && *p <= '9') {
    const uint32_t digit = (uint32_t)(*p - '0');
    if (whole_ns > (UINT64_MAX - (uint64_t)digit) / 10ULL) return false;
    whole_ns = whole_ns * 10ULL + (uint64_t)digit;
    integer_digits++;
    p++;
  }
  if (integer_digits == 0U || *p != '.') return false;
  p++;

  uint32_t fractional_fs = 0U;
  for (uint32_t i = 0U; i < 6U; i++) {
    if (*p < '0' || *p > '9') return false;
    fractional_fs = fractional_fs * 10U + (uint32_t)(*p - '0');
    p++;
  }
  if (*p != '\0') return false;
  if (whole_ns > (UINT64_MAX - (uint64_t)fractional_fs) / 1000000ULL) {
    return false;
  }

  lap_baseline_fs = whole_ns * 1000000ULL + (uint64_t)fractional_fs;
  return lap_baseline_fs != 0ULL;
}


static void photons_install_lap_baseline_fs(uint64_t requested_fs) {
  if (requested_fs == 0ULL) __builtin_trap();
  g_lap_baseline_fs = requested_fs;
  // Deprecated compatibility mirror only. Round to the nearest whole ps.
  g_standard_lap_ps = requested_fs / 1000ULL +
      ((requested_fs % 1000ULL) >= 500ULL ? 1ULL : 0ULL);
  if (g_standard_lap_ps == 0ULL) __builtin_trap();
  photons_memory_barrier();
  g_standard_lap_configured = true;
}


static FLASHMEM Payload cmd_set_lap_baseline_ns(const Payload& args) {
  const char* text = args.getString("lap_baseline_ns");
  uint64_t requested_fs = 0ULL;
  if (!photons_parse_lap_baseline_ns(text, requested_fs)) {
    Payload err;
    err.add("status", "lap_baseline_rejected_contract");
    err.add("error",
            "LAP_BASELINE_NS must be positive fixed decimal with exactly six fractional digits");
    return err;
  }
  if (g_photons_recovery_protocol.active || g_photons_recovery.proof_pending ||
      g_photons_stats_reset_pending) {
    Payload err;
    err.add("status", "lap_baseline_rejected_transition");
    err.add("error", "PHOTONS recovery/statistics transition owns the reference boundary");
    return err;
  }

  const bool was_configured = g_standard_lap_configured;
  const uint64_t previous_fs = g_lap_baseline_fs;
  const bool changed = !was_configured || requested_fs != previous_fs;
  if (changed) photons_install_lap_baseline_fs(requested_fs);

  Payload p;
  p.add("status", "lap_baseline_set");
  p.add("changed", changed);
  p.add("lap_baseline_configured", g_standard_lap_configured);
  p.add("lap_baseline_fs", g_lap_baseline_fs);
  p.add("lap_baseline_ns",
        toFixedDecimal((double)g_lap_baseline_fs / 1000000.0, 6));
  p.add("previous_lap_baseline_fs", was_configured ? previous_fs : 0ULL);
  p.add("publication_started", g_photons_recovery.publication_started);
  p.add("measurement_history_preserved", true);
  p.add("better_buckets_history_preserved", true);
  p.add("campaign_state", photons_campaign_state_name(g_photons_campaign_state));
  return p;
}


// Legacy 3-decimal startup alias. It may establish the reference on a newborn
// producer, but it may not overwrite an exact six-decimal baseline.
static bool photons_parse_standard_lap_ns(const char* text,
                                          uint64_t& standard_lap_ps) {
  standard_lap_ps = 0ULL;
  if (!text || !*text) return false;

  const char* p = text;
  uint64_t whole_ns = 0ULL;
  uint32_t integer_digits = 0U;
  while (*p >= '0' && *p <= '9') {
    const uint32_t digit = (uint32_t)(*p - '0');
    if (whole_ns > (UINT64_MAX - (uint64_t)digit) / 10ULL) return false;
    whole_ns = whole_ns * 10ULL + (uint64_t)digit;
    integer_digits++;
    p++;
  }
  if (integer_digits == 0U || *p != '.') return false;
  p++;

  uint32_t fractional_ps = 0U;
  for (uint32_t i = 0U; i < 3U; i++) {
    if (*p < '0' || *p > '9') return false;
    fractional_ps = fractional_ps * 10U + (uint32_t)(*p - '0');
    p++;
  }
  if (*p != '\0') return false;
  if (whole_ns > (UINT64_MAX - (uint64_t)fractional_ps) / 1000ULL) return false;
  standard_lap_ps = whole_ns * 1000ULL + (uint64_t)fractional_ps;
  return standard_lap_ps != 0ULL;
}


static FLASHMEM Payload cmd_set_standard_lap_ns(const Payload& args) {
  const char* text = args.getString("standard_lap_ns");
  uint64_t requested_ps = 0ULL;
  if (!photons_parse_standard_lap_ns(text, requested_ps) ||
      requested_ps > UINT64_MAX / 1000ULL) {
    __builtin_trap();
  }
  const uint64_t requested_fs = requested_ps * 1000ULL;

  if (g_standard_lap_configured) {
    if (requested_fs != g_lap_baseline_fs) __builtin_trap();
  } else {
    photons_install_lap_baseline_fs(requested_fs);
  }

  Payload p;
  p.add("standard_lap_configured", true);
  p.add("publication_started", g_photons_recovery.publication_started);
  p.add("recovery_verdict_required", !g_photons_recovery.publication_started);
  p.add("standard_lap_ps", g_standard_lap_ps);
  p.add("standard_lap_ns",
        toFixedDecimal((double)g_standard_lap_ps / 1000.0, 3));
  p.add("lap_baseline_fs", g_lap_baseline_fs);
  p.add("lap_baseline_ns",
        toFixedDecimal((double)g_lap_baseline_fs / 1000000.0, 6));
  return p;
}


static bool photons_recovery_get_u32(const Payload& args,
                                     const char* key,
                                     uint32_t& out) {
  return args.has(key) && args.tryGetUInt(key, out);
}


static bool photons_recovery_get_u64(const Payload& args,
                                     const char* key,
                                     uint64_t& out) {
  return args.has(key) && args.tryGetUInt64(key, out);
}


static void photons_ppb_export_add_endpoint(
    Payload& p,
    const char* prefix,
    const photons_ppb_endpoint_t& endpoint) {
  char key[80];
  snprintf(key, sizeof(key), "%s_sequence", prefix);
  p.add(key, endpoint.sequence);
  snprintf(key, sizeof(key), "%s_lap_count", prefix);
  p.add(key, endpoint.lap_count);
  snprintf(key, sizeof(key), "%s_total_lap_gnss_ns", prefix);
  p.add(key, endpoint.total_lap_gnss_ns);
}


static bool photons_ppb_export_live_ready(void) {
  return g_photons_recovery.publication_started &&
         !g_photons_recovery_protocol.active &&
         g_photons_ppb_endpoint_admitted &&
         g_photons_ppb_previous_endpoint_valid &&
         g_photons_ppb_current_sequence != 0U &&
         g_photons_ppb_current_sequence == g_photons_stats_update_count &&
         g_photons_ppb_previous_endpoint.sequence ==
             g_photons_ppb_current_sequence &&
         g_photons_ppb_seconds_count != 0U &&
         g_photons_ppb_minutes_count != 0U;
}


static FLASHMEM Payload cmd_ppb_export_meta(const Payload& /*args*/) {
  if (!photons_ppb_export_live_ready()) {
    Payload err;
    err.add("status", "ppb_export_snapshot_unavailable");
    err.add("error", "live PHOTONS Better-Buckets custody is unavailable");
    err.add("publication_started", g_photons_recovery.publication_started);
    err.add("staging_active", g_photons_recovery_protocol.active);
    err.add("endpoint_admitted", g_photons_ppb_endpoint_admitted);
    return err;
  }

  const photons_ppb_endpoint_t current = g_photons_ppb_previous_endpoint;
  const photons_ppb_endpoint_t origin{};
  Payload p;
  p.add("status", "ppb_export_ready");
  p.add("schema", "PHOTONS_PPB_FULL_RING_EXPORT_V1");
  p.add("read_only", true);
  p.add("reset_count", g_photons_stats_reset_count);
  p.add("update_count", g_photons_stats_update_count);
  p.add("current_sequence", g_photons_ppb_current_sequence);
  p.add("second_count", g_photons_ppb_seconds_count);
  p.add("minute_count", g_photons_ppb_minutes_count);
  p.add("second_oldest_sequence", photons_ppb_ring_oldest_sequence(
      g_photons_ppb_seconds, g_photons_ppb_seconds_head,
      g_photons_ppb_seconds_count));
  p.add("second_newest_sequence", photons_ppb_ring_newest_sequence(
      g_photons_ppb_seconds, g_photons_ppb_seconds_head,
      g_photons_ppb_seconds_count));
  p.add("minute_oldest_sequence", photons_ppb_ring_oldest_sequence(
      g_photons_ppb_minutes, g_photons_ppb_minutes_head,
      g_photons_ppb_minutes_count));
  p.add("minute_newest_sequence", photons_ppb_ring_newest_sequence(
      g_photons_ppb_minutes, g_photons_ppb_minutes_head,
      g_photons_ppb_minutes_count));
  p.add("last_minute_key", g_photons_ppb_last_minute_key);
  p.add("origin_valid", true);
  p.add("lap_baseline_fs", g_lap_baseline_fs);
  p.add("lap_baseline_ns",
        toFixedDecimal((double)g_lap_baseline_fs / 1000000.0, 6));
  p.add("standard_lap_ps", g_standard_lap_ps);
  p.add("chunk_max_endpoints", PHOTONS_RECOVERY_CHUNK_MAX_ENDPOINTS);
  photons_ppb_export_add_endpoint(p, "current", current);
  photons_ppb_export_add_endpoint(p, "origin", origin);
  return p;
}


static FLASHMEM Payload cmd_ppb_export_chunk(const Payload& args) {
  uint32_t reset_count = 0U;
  uint32_t before_sequence = 0U;
  uint32_t count = 0U;
  const char* history = args.getString("history");
  const bool seconds = history && !strcmp(history, "SECOND");
  const bool minutes = history && !strcmp(history, "MINUTE");

  if ((!seconds && !minutes) ||
      !photons_recovery_get_u32(args, "reset_count", reset_count) ||
      !photons_recovery_get_u32(args, "before_sequence", before_sequence) ||
      !photons_recovery_get_u32(args, "count", count) ||
      count == 0U || count > PHOTONS_RECOVERY_CHUNK_MAX_ENDPOINTS) {
    Payload err;
    err.add("status", "ppb_export_chunk_rejected_contract");
    err.add("error", "invalid PHOTONS Better-Buckets export request");
    return err;
  }

  if (!photons_ppb_export_live_ready()) {
    Payload err;
    err.add("status", "ppb_export_chunk_rejected_unavailable");
    err.add("error", "live PHOTONS Better-Buckets custody is unavailable");
    return err;
  }
  if (reset_count != g_photons_stats_reset_count) {
    Payload err;
    err.add("status", "ppb_export_chunk_rejected_reset");
    err.add("error", "PHOTONS Better-Buckets reset identity changed");
    err.add("requested_reset_count", reset_count);
    err.add("observed_reset_count", g_photons_stats_reset_count);
    return err;
  }

  photons_ppb_endpoint_t endpoints[PHOTONS_RECOVERY_CHUNK_MAX_ENDPOINTS]{};
  const uint32_t returned = seconds
      ? photons_ppb_ring_export_chunk(
            g_photons_ppb_seconds, g_photons_ppb_seconds_head,
            g_photons_ppb_seconds_count, before_sequence, endpoints, count)
      : photons_ppb_ring_export_chunk(
            g_photons_ppb_minutes, g_photons_ppb_minutes_head,
            g_photons_ppb_minutes_count, before_sequence, endpoints, count);

  Payload p;
  p.add("status", "ppb_export_chunk");
  p.add("schema", "PHOTONS_PPB_FULL_RING_EXPORT_V1");
  p.add("history", history);
  p.add("reset_count", reset_count);
  p.add("before_sequence", before_sequence);
  p.add("count", returned);
  for (uint32_t i = 0U; i < returned; ++i) {
    char prefix[16];
    snprintf(prefix, sizeof(prefix), "e%lu", (unsigned long)i);
    photons_ppb_export_add_endpoint(p, prefix, endpoints[i]);
  }
  return p;
}


static bool photons_recovery_get_bool(const Payload& args,
                                      const char* key,
                                      bool& out) {
  return args.has(key) && args.tryGetBool(key, out);
}


static bool photons_recovery_get_double(const Payload& args,
                                        const char* key,
                                        double& out) {
  if (!args.has(key)) return false;
  const char* text = args.getString(key);
  if (!text || !*text) return false;
  errno = 0;
  char* end = nullptr;
  const double parsed = strtod(text, &end);
  if (errno == ERANGE || !end || *end != '\0' || !isfinite(parsed)) {
    return false;
  }
  out = parsed;
  return true;
}


static bool photons_recovery_get_welford(const Payload& args,
                                         const char* prefix,
                                         photons_welford_state_t& out) {
  char key[80];
  uint64_t n = 0ULL;
  double mean = 0.0;
  double m2 = 0.0;
  double min_val = 0.0;
  double max_val = 0.0;

  snprintf(key, sizeof(key), "%s_n", prefix);
  if (!photons_recovery_get_u64(args, key, n)) return false;
  snprintf(key, sizeof(key), "%s_mean", prefix);
  if (!photons_recovery_get_double(args, key, mean)) return false;
  snprintf(key, sizeof(key), "%s_m2", prefix);
  if (!photons_recovery_get_double(args, key, m2)) return false;
  snprintf(key, sizeof(key), "%s_min", prefix);
  if (!photons_recovery_get_double(args, key, min_val)) return false;
  snprintf(key, sizeof(key), "%s_max", prefix);
  if (!photons_recovery_get_double(args, key, max_val)) return false;

  if (n == 0ULL) {
    photons_welford_reset(out);
    return mean == 0.0 && m2 == 0.0 && min_val == 0.0 && max_val == 0.0;
  }
  if (m2 < 0.0 || min_val > max_val || mean < min_val || mean > max_val) {
    return false;
  }

  out.n = n;
  out.mean = mean;
  out.m2 = m2;
  out.min_val = min_val;
  out.max_val = max_val;
  return true;
}


static Payload photons_recovery_reject(const char* status,
                                       const char* error) {
  g_photons_recovery.reject_count++;
  Payload p;
  p.add("status", status ? status : "recovery_rejected");
  p.add("error", error ? error : "recovery command rejected");
  p.add("publication_started", g_photons_recovery.publication_started);
  p.add("staging_active", g_photons_recovery_protocol.active);
  p.add("generation", g_photons_recovery.generation);
  return p;
}


static FLASHMEM Payload cmd_recovery_begin(const Payload& args) {
  if (!g_standard_lap_configured || g_lap_baseline_fs == 0ULL) {
    return photons_recovery_reject(
        "recovery_begin_rejected_standard_missing",
        "LAP_BASELINE_NS must be installed before recovery");
  }
  if (g_photons_recovery.publication_started) {
    return photons_recovery_reject(
        "recovery_begin_rejected_live",
        "PHOTONS publication is already live; use live reattachment");
  }
  if (g_photons_recovery_protocol.active) {
    return photons_recovery_reject(
        "recovery_begin_rejected_staging_active",
        "a recovery transaction is already staged");
  }

  uint32_t version = 0U;
  uint32_t generation = 0U;
  uint32_t second_count = 0U;
  uint32_t minute_count = 0U;
  if (!photons_recovery_get_u32(args, "restore_schema_version", version) ||
      version != PHOTONS_RECOVERY_SCHEMA_VERSION ||
      !photons_recovery_get_u32(args, "generation", generation) ||
      generation == 0U ||
      !photons_recovery_get_u32(args, "second_count", second_count) ||
      second_count == 0U ||
      second_count > PHOTONS_PPB_SECOND_CAPACITY ||
      !photons_recovery_get_u32(args, "minute_count", minute_count) ||
      minute_count == 0U ||
      minute_count > PHOTONS_PPB_MINUTE_CAPACITY) {
    return photons_recovery_reject(
        "recovery_begin_rejected_contract",
        "invalid recovery schema, generation, or history counts");
  }

  photons_recovery_protocol_clear(true);
  g_photons_recovery_protocol.active = true;
  g_photons_recovery_protocol.generation = generation;
  g_photons_recovery_protocol.expected_second_count = second_count;
  g_photons_recovery_protocol.expected_minute_count = minute_count;
  g_photons_recovery.begin_count++;

  Payload p;
  p.add("status", "recovery_staging");
  p.add("restore_schema_version", PHOTONS_RECOVERY_SCHEMA_VERSION);
  p.add("generation", generation);
  p.add("second_count", second_count);
  p.add("minute_count", minute_count);
  p.add("publication_started", false);
  return p;
}


static bool photons_recovery_endpoint_follows(
    bool minute_history,
    const photons_ppb_endpoint_t& previous,
    bool previous_valid,
    uint32_t accepted_count,
    const photons_ppb_endpoint_t& endpoint) {
  if (!photons_ppb_endpoint_population_consistent(endpoint)) return false;
  if (endpoint.sequence == 0U) {
    return accepted_count == 0U && endpoint.lap_count == 0ULL;
  }
  if (!previous_valid) return true;
  if (endpoint.sequence <= previous.sequence ||
      endpoint.lap_count < previous.lap_count ||
      endpoint.total_lap_gnss_ns < previous.total_lap_gnss_ns) {
    return false;
  }
  return !minute_history ||
      photons_ppb_minute_key(endpoint.sequence) >
          photons_ppb_minute_key(previous.sequence);
}


static FLASHMEM Payload cmd_recovery_chunk(const Payload& args) {
  photons_recovery_protocol_t& protocol = g_photons_recovery_protocol;
  if (!protocol.active || g_photons_recovery.publication_started) {
    return photons_recovery_reject(
        "recovery_chunk_rejected_not_staging",
        "RECOVERY_BEGIN must own a held transaction");
  }

  uint32_t generation = 0U;
  uint32_t count = 0U;
  const char* history = args.getString("history");
  const bool minute_history = history && !strcmp(history, "MINUTE");
  const bool second_history = history && !strcmp(history, "SECOND");
  if ((!minute_history && !second_history) ||
      !photons_recovery_get_u32(args, "generation", generation) ||
      generation != protocol.generation ||
      !photons_recovery_get_u32(args, "count", count) ||
      count == 0U || count > PHOTONS_RECOVERY_CHUNK_MAX_ENDPOINTS) {
    return photons_recovery_reject(
        "recovery_chunk_rejected_contract",
        "invalid history, generation, or chunk count");
  }

  const uint32_t accepted_before = minute_history
      ? protocol.accepted_minute_count
      : protocol.accepted_second_count;
  const uint32_t expected = minute_history
      ? protocol.expected_minute_count
      : protocol.expected_second_count;
  if (accepted_before + count > expected) {
    return photons_recovery_reject(
        "recovery_chunk_rejected_overflow",
        "chunk exceeds the declared history count");
  }

  photons_ppb_endpoint_t endpoints[PHOTONS_RECOVERY_CHUNK_MAX_ENDPOINTS]{};
  photons_ppb_endpoint_t previous = minute_history
      ? protocol.previous_minute
      : protocol.previous_second;
  bool previous_valid = minute_history
      ? protocol.previous_minute_valid
      : protocol.previous_second_valid;

  for (uint32_t i = 0U; i < count; i++) {
    char key[64];
    snprintf(key, sizeof(key), "e%u_sequence", (unsigned int)i);
    if (!photons_recovery_get_u32(args, key, endpoints[i].sequence)) {
      return photons_recovery_reject(
          "recovery_chunk_rejected_endpoint",
          "endpoint sequence is missing or malformed");
    }
    snprintf(key, sizeof(key), "e%u_lap_count", (unsigned int)i);
    if (!photons_recovery_get_u64(args, key, endpoints[i].lap_count)) {
      return photons_recovery_reject(
          "recovery_chunk_rejected_endpoint",
          "endpoint lap_count is missing or malformed");
    }
    snprintf(key, sizeof(key), "e%u_total_lap_gnss_ns", (unsigned int)i);
    if (!photons_recovery_get_u64(
            args, key, endpoints[i].total_lap_gnss_ns)) {
      return photons_recovery_reject(
          "recovery_chunk_rejected_endpoint",
          "endpoint total_lap_gnss_ns is missing or malformed");
    }
    if (!photons_recovery_endpoint_follows(
            minute_history,
            previous,
            previous_valid,
            accepted_before + i,
            endpoints[i])) {
      Payload rejected = photons_recovery_reject(
          "recovery_chunk_rejected_chronology",
          "endpoint chronology or cumulative population is invalid");
      rejected.add("history", minute_history ? "MINUTE" : "SECOND");
      rejected.add("staging_generation", protocol.generation);
      rejected.add("chunk_index", i);
      rejected.add("accepted_before", accepted_before);
      rejected.add("endpoint_sequence", endpoints[i].sequence);
      rejected.add("endpoint_lap_count", endpoints[i].lap_count);
      rejected.add("endpoint_total_lap_gnss_ns",
                   endpoints[i].total_lap_gnss_ns);
      rejected.add("previous_valid", previous_valid);
      if (previous_valid) {
        rejected.add("previous_sequence", previous.sequence);
        rejected.add("previous_lap_count", previous.lap_count);
        rejected.add("previous_total_lap_gnss_ns",
                     previous.total_lap_gnss_ns);
      }
      return rejected;
    }
    previous = endpoints[i];
    previous_valid = true;
  }

  for (uint32_t i = 0U; i < count; i++) {
    if (!photons_recovery_stage_endpoint(minute_history, endpoints[i])) {
      __builtin_trap();
    }
  }
  g_photons_recovery.chunk_count++;

  Payload p;
  p.add("status", "recovery_chunk_accepted");
  p.add("generation", generation);
  p.add("history", minute_history ? "MINUTE" : "SECOND");
  p.add("chunk_count", count);
  p.add("second_accepted", protocol.accepted_second_count);
  p.add("second_expected", protocol.expected_second_count);
  p.add("minute_accepted", protocol.accepted_minute_count);
  p.add("minute_expected", protocol.expected_minute_count);
  return p;
}


static FLASHMEM Payload cmd_recovery_abort(const Payload& /*args*/) {
  if (g_photons_recovery.publication_started) {
    return photons_recovery_reject(
        "recovery_abort_rejected_live",
        "live PHOTONS state cannot be aborted");
  }
  photons_recovery_protocol_clear(true);
  g_photons_recovery.abort_count++;
  Payload p;
  p.add("status", "recovery_aborted");
  p.add("publication_started", false);
  return p;
}


static void photons_recovery_install_science_state(
    uint64_t accepted_count,
    uint64_t excluded_count,
    uint64_t projection_invalid,
    uint64_t seed_disagreement,
    uint64_t raw_cycle_excursion,
    const photons_welford_state_t& accepted_projected,
    const photons_welford_state_t& accepted_raw,
    const photons_welford_state_t& excluded_raw,
    const photons_welford_state_t& excluded_projected) {
  photons_lap_science_reason_counts_snapshot_t recovery_reasons{};
  recovery_reasons.projection_invalid = projection_invalid;
  recovery_reasons.seed_disagreement = seed_disagreement;
  recovery_reasons.raw_cycle_excursion = raw_cycle_excursion;
  const uint64_t derived_excluded_count =
      photons_lap_science_excluded_count_from_reasons(recovery_reasons);
  if (excluded_count != derived_excluded_count ||
      excluded_raw.n != derived_excluded_count ||
      excluded_projected.n > derived_excluded_count ||
      UINT64_MAX - accepted_count < derived_excluded_count) {
    __builtin_trap();
  }
  const uint64_t candidate_count =
      accepted_count + derived_excluded_count;

  g_raw_cycles_state = photons_fragment_raw_cycles_snapshot_t{};
  g_raw_cycles_state.valid = candidate_count != 0ULL;
  g_raw_cycles_state.completed_lap_count = candidate_count;

  g_projection_state = photons_fragment_projection_snapshot_t{};
  g_projection_state.attempt_count = candidate_count;
  g_projection_state.reject_count = projection_invalid;
  g_projection_state.success_count = candidate_count - projection_invalid;

  g_photons_lap_science_state = photons_lap_science_snapshot_t{};
  g_photons_lap_science_state.valid = candidate_count != 0ULL;
  g_photons_lap_science_state.candidate_count = candidate_count;
  g_photons_lap_science_state.accepted.count = accepted_count;
  // excluded_count is recovery testimony only.  The installed producer derives
  // its aggregate exclusion population from the three reason counters below.
  g_photons_lap_science_state.exclusion_reasons.projection_invalid =
      projection_invalid;
  g_photons_lap_science_state.exclusion_reasons.seed_disagreement =
      seed_disagreement;
  g_photons_lap_science_state.exclusion_reasons.raw_cycle_excursion =
      raw_cycle_excursion;
  g_photons_lap_science_state.accepted.raw_cycles =
      photons_welford_snapshot(accepted_raw);
  g_photons_lap_science_state.accepted.projected_lap_ns =
      photons_welford_snapshot(accepted_projected);
  g_photons_lap_science_state.excluded.raw_cycles =
      photons_welford_snapshot(excluded_raw);
  g_photons_lap_science_state.excluded.projected_lap_ns =
      photons_welford_snapshot(excluded_projected);

  // Predictor, last-candidate testimony, reject streak, and pending seed are
  // intentionally fresh.  The first post-restart laps reacquire them physically.
  g_photons_lap_science_seed_pending = photons_lap_science_candidate_t{};

  // RECOVERY_COMMIT already proved excluded_count == sum(reasons).  Prove the
  // installed state again from its sole authority and independent raw-Welford N;
  // do not install the redundant aggregate as producer state.
  photons_lap_science_validate_exclusion_ledger(
      g_photons_lap_science_state);
}


static FLASHMEM Payload cmd_recovery_commit(const Payload& args) {
  photons_recovery_protocol_t& protocol = g_photons_recovery_protocol;
  if (!protocol.active || g_photons_recovery.publication_started) {
    return photons_recovery_reject(
        "recovery_commit_rejected_not_staging",
        "RECOVERY_BEGIN must own a complete held transaction");
  }
  if (protocol.accepted_second_count != protocol.expected_second_count ||
      protocol.accepted_minute_count != protocol.expected_minute_count ||
      !protocol.previous_second_valid || !protocol.previous_minute_valid) {
    return photons_recovery_reject(
        "recovery_commit_rejected_history_incomplete",
        "declared Better-Buckets history is incomplete");
  }

  uint32_t version = 0U;
  uint32_t generation = 0U;
  uint32_t source_sequence = 0U;
  uint32_t source_publish_count = 0U;
  uint32_t source_reset_count = 0U;
  uint32_t source_update_count = 0U;
  uint64_t standard_lap_ps = 0ULL;
  uint64_t stats_lap_count = 0ULL;
  uint64_t stats_total_ns = 0ULL;
  uint64_t custody_lap_count = 0ULL;
  uint64_t custody_total_ns = 0ULL;
  uint64_t accepted_count = 0ULL;
  uint64_t excluded_count = 0ULL;
  uint64_t projection_invalid = 0ULL;
  uint64_t seed_disagreement = 0ULL;
  uint64_t raw_cycle_excursion = 0ULL;
  uint32_t dropped_pending_seed_count = 0U;
  bool campaign_active = false;

  photons_welford_state_t accepted_projected{};
  photons_welford_state_t accepted_raw{};
  photons_welford_state_t excluded_raw{};
  photons_welford_state_t excluded_projected{};

  const bool scalar_ok =
      photons_recovery_get_u32(args, "restore_schema_version", version) &&
      version == PHOTONS_RECOVERY_SCHEMA_VERSION &&
      photons_recovery_get_u32(args, "generation", generation) &&
      generation == protocol.generation &&
      photons_recovery_get_u32(args, "source_sequence", source_sequence) &&
      source_sequence != 0U &&
      photons_recovery_get_u32(
          args, "source_publish_count", source_publish_count) &&
      source_publish_count != 0U &&
      photons_recovery_get_u32(
          args, "source_reset_count", source_reset_count) &&
      photons_recovery_get_u32(
          args, "source_update_count", source_update_count) &&
      source_update_count != 0U &&
      photons_recovery_get_u64(args, "standard_lap_ps", standard_lap_ps) &&
      standard_lap_ps == g_standard_lap_ps &&
      photons_recovery_get_u64(args, "stats_lap_count", stats_lap_count) &&
      photons_recovery_get_u64(
          args, "stats_total_lap_gnss_ns", stats_total_ns) &&
      photons_recovery_get_u64(
          args, "custody_lap_count", custody_lap_count) &&
      photons_recovery_get_u64(
          args, "custody_total_lap_gnss_ns", custody_total_ns) &&
      photons_recovery_get_u64(args, "accepted_count", accepted_count) &&
      photons_recovery_get_u64(args, "excluded_count", excluded_count) &&
      photons_recovery_get_u64(
          args, "projection_invalid", projection_invalid) &&
      photons_recovery_get_u64(
          args, "seed_disagreement", seed_disagreement) &&
      photons_recovery_get_u64(
          args, "raw_cycle_excursion", raw_cycle_excursion) &&
      photons_recovery_get_u32(
          args, "dropped_pending_seed_count", dropped_pending_seed_count) &&
      dropped_pending_seed_count <= 1U &&
      photons_recovery_get_bool(args, "campaign_active", campaign_active) &&
      photons_recovery_get_welford(
          args, "accepted_projected", accepted_projected) &&
      photons_recovery_get_welford(args, "accepted_raw", accepted_raw) &&
      photons_recovery_get_welford(args, "excluded_raw", excluded_raw) &&
      photons_recovery_get_welford(
          args, "excluded_projected", excluded_projected);
  if (!scalar_ok) {
    return photons_recovery_reject(
        "recovery_commit_rejected_state",
        "aggregate recovery state is missing, malformed, or mismatched");
  }

  photons_lap_science_reason_counts_snapshot_t recovery_reasons{};
  recovery_reasons.projection_invalid = projection_invalid;
  recovery_reasons.seed_disagreement = seed_disagreement;
  recovery_reasons.raw_cycle_excursion = raw_cycle_excursion;
  const uint64_t derived_excluded_count =
      photons_lap_science_excluded_count_from_reasons(recovery_reasons);

  if (accepted_count == 0ULL || stats_lap_count != accepted_count ||
      stats_total_ns == 0ULL || custody_lap_count < stats_lap_count ||
      custody_total_ns < stats_total_ns ||
      accepted_projected.n != accepted_count ||
      accepted_raw.n != accepted_count ||
      excluded_count != derived_excluded_count ||
      excluded_raw.n != derived_excluded_count ||
      excluded_projected.n > derived_excluded_count) {
    return photons_recovery_reject(
        "recovery_commit_rejected_accounting",
        "aggregate population accounting does not close");
  }

  // N/T and the accepted projected-lap Welford describe the same population,
  // but they reach the mean through different numerical paths: N/T performs one
  // division over exact integer sufficient state, while Welford accumulates
  // millions of floating-point updates.  Their tiny rounding drift is therefore
  // corroborative testimony, not a custody identity.  Exact recovery integrity
  // is proved by the population/accounting court above and by the staged endpoint
  // N/T target below; do not reject truthful ancestry on cross-algorithm drift.

  const photons_ppb_endpoint_t& current = protocol.previous_second;
  if (current.sequence != source_update_count ||
      current.lap_count != stats_lap_count ||
      current.total_lap_gnss_ns != stats_total_ns ||
      protocol.previous_minute.sequence > source_update_count) {
    return photons_recovery_reject(
        "recovery_commit_rejected_history_target",
        "staged history does not terminate at the aggregate source state");
  }

  char campaign_name[64] = {0};
  uint64_t campaign_origin_lap_count = 0ULL;
  uint64_t campaign_origin_total_ns = 0ULL;
  uint32_t campaign_start_after_sequence = 0U;
  uint32_t campaign_public_count = 0U;
  uint64_t campaign_lap_count = 0ULL;
  uint64_t campaign_total_ns = 0ULL;
  if (campaign_active) {
    const char* supplied_name = args.getString("campaign");
    if (!supplied_name || !*supplied_name ||
        strlen(supplied_name) >= sizeof(campaign_name) ||
        !photons_recovery_get_u64(
            args, "campaign_origin_lap_count", campaign_origin_lap_count) ||
        !photons_recovery_get_u64(
            args, "campaign_origin_total_lap_gnss_ns",
            campaign_origin_total_ns) ||
        !photons_recovery_get_u32(
            args, "campaign_start_after_sequence",
            campaign_start_after_sequence) ||
        campaign_start_after_sequence == 0U ||
        campaign_start_after_sequence >= source_sequence ||
        !photons_recovery_get_u32(
            args, "campaign_public_count", campaign_public_count) ||
        campaign_public_count == 0U ||
        campaign_public_count !=
            source_sequence - campaign_start_after_sequence ||
        !photons_recovery_get_u64(
            args, "campaign_lap_count", campaign_lap_count) ||
        !photons_recovery_get_u64(
            args, "campaign_total_lap_gnss_ns", campaign_total_ns) ||
        campaign_origin_lap_count > custody_lap_count ||
        campaign_origin_total_ns > custody_total_ns ||
        custody_lap_count - campaign_origin_lap_count != campaign_lap_count ||
        custody_total_ns - campaign_origin_total_ns != campaign_total_ns) {
      return photons_recovery_reject(
          "recovery_commit_rejected_campaign",
          "active campaign recovery state does not close");
    }
    safeCopy(campaign_name, sizeof(campaign_name), supplied_name);
  }

  // All parsing and accounting courts have passed.  From here to publication
  // start, installation is one foreground transaction and no PHOTONS timer runs.
  g_lap_time_welford = accepted_projected;
  g_accepted_raw_cycles_welford = accepted_raw;
  g_excluded_raw_cycles_welford = excluded_raw;
  g_excluded_lap_time_welford = excluded_projected;
  g_total_lap_gnss_ns = stats_total_ns;
  g_photons_custody_lap_count = custody_lap_count;
  g_photons_custody_total_lap_gnss_ns = custody_total_ns;
  g_photons_stats_reset_count = source_reset_count;
  g_photons_stats_update_count = source_update_count;
  g_photons_ppb_previous_endpoint = current;
  g_photons_ppb_previous_endpoint_valid = true;
  g_photons_ppb_current_sequence = source_update_count;
  g_photons_ppb_endpoint_admitted = true;
  g_photons_ppb_interval_advanced = false;
  g_photons_ppb_last_minute_key = photons_ppb_minute_key(source_update_count);

  photons_recovery_install_science_state(
      accepted_count,
      excluded_count,
      projection_invalid,
      seed_disagreement,
      raw_cycle_excursion,
      accepted_projected,
      accepted_raw,
      excluded_raw,
      excluded_projected);

  g_fragment_sequence = source_sequence;
  g_publish_count = source_publish_count;
  g_publish_reject_count = 0U;
  g_photons_stats_reset_pending = false;

  if (campaign_active) {
    safeCopy(g_photons_campaign_name, sizeof(g_photons_campaign_name),
             campaign_name);
    g_photons_campaign_origin_lap_count = campaign_origin_lap_count;
    g_photons_campaign_origin_total_lap_gnss_ns = campaign_origin_total_ns;
    g_photons_campaign_start_after_sequence = campaign_start_after_sequence;
    g_photons_campaign_public_count = campaign_public_count;
    g_photons_campaign_state = photons_campaign_state_t::ACTIVE;
  } else {
    g_photons_campaign_state = photons_campaign_state_t::STOPPED;
    g_photons_campaign_name[0] = '\0';
    g_photons_campaign_origin_lap_count = 0ULL;
    g_photons_campaign_origin_total_lap_gnss_ns = 0ULL;
    g_photons_campaign_start_after_sequence = 0U;
    g_photons_campaign_public_count = 0U;
  }
  g_photons_flash_cut_campaign_name[0] = '\0';

  g_photons_recovery.restored = true;
  g_photons_recovery.proof_pending = true;
  g_photons_recovery.proof_committed = false;
  g_photons_recovery.proof_advanced_published = false;
  g_photons_recovery.generation = generation;
  g_photons_recovery.source_sequence = source_sequence;
  g_photons_recovery.source_publish_count = source_publish_count;
  g_photons_recovery.source_reset_count = source_reset_count;
  g_photons_recovery.source_update_count = source_update_count;
  g_photons_recovery.source_lap_count = stats_lap_count;
  g_photons_recovery.source_total_lap_gnss_ns = stats_total_ns;
  g_photons_recovery.source_custody_lap_count = custody_lap_count;
  g_photons_recovery.source_custody_total_lap_gnss_ns = custody_total_ns;
  g_photons_recovery.proof_sequence = 0U;
  g_photons_recovery.proof_update_count = 0U;
  g_photons_recovery.dropped_pending_seed_count = dropped_pending_seed_count;
  g_photons_recovery.commit_count++;

  photons_recovery_protocol_clear(false);
  photons_recovery_clear_physical_ancestry();
  photons_start_publication();

  Payload p;
  p.add("status", "recovery_committed");
  p.add("generation", generation);
  p.add("source_sequence", source_sequence);
  p.add("source_update_count", source_update_count);
  p.add("restored_lap_count", stats_lap_count);
  p.add("restored_total_lap_gnss_ns", stats_total_ns);
  p.add("restored_custody_lap_count", custody_lap_count);
  p.add("campaign_active", campaign_active);
  if (campaign_active) p.add("campaign", campaign_name);
  p.add("publication_started", true);
  p.add("fresh_physical_ancestry", true);
  p.add("pending_seed_restored", false);
  p.add("raw_lap_ring_restored", false);
  p.add("proof_pending", true);
  return p;
}


static FLASHMEM Payload cmd_recovery_cold_start(const Payload& args) {
  if (!g_standard_lap_configured || g_lap_baseline_fs == 0ULL) {
    return photons_recovery_reject(
        "recovery_cold_start_rejected_standard_missing",
        "LAP_BASELINE_NS must be installed before cold start");
  }
  if (g_photons_recovery.publication_started ||
      g_photons_recovery_protocol.active) {
    return photons_recovery_reject(
        "recovery_cold_start_rejected_busy",
        "cold start requires a held instrument with no staged restore");
  }

  uint32_t generation = 0U;
  if (!photons_recovery_get_u32(args, "generation", generation) ||
      generation == 0U) {
    return photons_recovery_reject(
        "recovery_cold_start_rejected_generation",
        "cold start requires a nonzero generation");
  }
  if (g_lap_time_welford.n != 0ULL || g_total_lap_gnss_ns != 0ULL ||
      g_photons_custody_lap_count != 0ULL ||
      g_photons_custody_total_lap_gnss_ns != 0ULL) {
    __builtin_trap();
  }

  photons_ppb_windows_seed_origin();
  g_photons_recovery.restored = false;
  g_photons_recovery.proof_pending = false;
  g_photons_recovery.proof_committed = false;
  g_photons_recovery.proof_advanced_published = false;
  g_photons_recovery.generation = generation;
  g_photons_recovery.source_sequence = 0U;
  g_photons_recovery.source_publish_count = 0U;
  g_photons_recovery.source_reset_count = 0U;
  g_photons_recovery.source_update_count = 0U;
  g_photons_recovery.source_lap_count = 0ULL;
  g_photons_recovery.source_total_lap_gnss_ns = 0ULL;
  g_photons_recovery.source_custody_lap_count = 0ULL;
  g_photons_recovery.source_custody_total_lap_gnss_ns = 0ULL;
  g_photons_recovery.proof_sequence = 0U;
  g_photons_recovery.proof_update_count = 0U;
  g_photons_recovery.dropped_pending_seed_count = 0U;
  g_photons_recovery.cold_start_count++;

  photons_recovery_clear_physical_ancestry();
  photons_start_publication();

  Payload p;
  p.add("status", "recovery_cold_start_committed");
  p.add("generation", generation);
  p.add("publication_started", true);
  p.add("restored", false);
  p.add("fresh_physical_ancestry", true);
  return p;
}


static FLASHMEM Payload cmd_recovery_proof_ack(const Payload& args) {
  uint32_t generation = 0U;
  uint32_t sequence = 0U;
  uint32_t update_count = 0U;
  const bool parsed =
      photons_recovery_get_u32(args, "generation", generation) &&
      photons_recovery_get_u32(args, "sequence", sequence) &&
      photons_recovery_get_u32(args, "update_count", update_count);
  const bool same_lineage =
      parsed &&
      g_photons_recovery.restored &&
      g_photons_recovery.proof_pending &&
      g_photons_recovery.proof_advanced_published &&
      generation == g_photons_recovery.generation &&
      g_photons_recovery.proof_sequence ==
          g_photons_recovery.source_sequence + 1U &&
      g_photons_recovery.proof_update_count ==
          g_photons_recovery.source_update_count + 1U &&
      sequence == g_photons_recovery.proof_sequence &&
      update_count == g_photons_recovery.proof_update_count &&
      sequence <= g_fragment_sequence &&
      update_count <= g_photons_stats_update_count;
  if (!same_lineage) {
    return photons_recovery_reject(
        "recovery_proof_ack_rejected",
        "durable proof is not the exact first recovery successor row");
  }

  g_photons_recovery.proof_pending = false;
  g_photons_recovery.proof_committed = true;
  g_photons_recovery.proof_ack_count++;

  Payload p;
  p.add("status", "recovery_proof_committed");
  p.add("generation", generation);
  p.add("first_proof_sequence", g_photons_recovery.proof_sequence);
  p.add("first_proof_update_count", g_photons_recovery.proof_update_count);
  p.add("durable_proof_sequence", sequence);
  p.add("durable_proof_update_count", update_count);
  p.add("proof_pending", false);
  p.add("proof_committed", true);
  return p;
}


static FLASHMEM Payload cmd_report_recovery(const Payload& /*args*/) {
  Payload p;
  p.add("report", "PHOTONS_RECOVERY");
  p.add("schema", "PHOTONS_RECOVERY_REPORT_V1");
  p.add("restore_schema_version", PHOTONS_RECOVERY_SCHEMA_VERSION);
  p.add("standard_lap_configured", g_standard_lap_configured);
  p.add("lap_baseline_fs", g_lap_baseline_fs);
  p.add("lap_baseline_ns",
        toFixedDecimal((double)g_lap_baseline_fs / 1000000.0, 6));
  p.add("standard_lap_ps", g_standard_lap_ps);
  p.add("publication_started", g_photons_recovery.publication_started);
  p.add("staging_active", g_photons_recovery_protocol.active);
  p.add("staging_generation", g_photons_recovery_protocol.generation);
  p.add("staging_second_expected",
        g_photons_recovery_protocol.expected_second_count);
  p.add("staging_second_accepted",
        g_photons_recovery_protocol.accepted_second_count);
  p.add("staging_minute_expected",
        g_photons_recovery_protocol.expected_minute_count);
  p.add("staging_minute_accepted",
        g_photons_recovery_protocol.accepted_minute_count);
  p.add("restored", g_photons_recovery.restored);
  p.add("proof_pending", g_photons_recovery.proof_pending);
  p.add("proof_committed", g_photons_recovery.proof_committed);
  p.add("proof_advanced_published",
        g_photons_recovery.proof_advanced_published);
  p.add("generation", g_photons_recovery.generation);
  p.add("source_sequence", g_photons_recovery.source_sequence);
  p.add("source_publish_count", g_photons_recovery.source_publish_count);
  p.add("source_reset_count", g_photons_recovery.source_reset_count);
  p.add("source_update_count", g_photons_recovery.source_update_count);
  p.add("source_lap_count", g_photons_recovery.source_lap_count);
  p.add("source_total_lap_gnss_ns",
        g_photons_recovery.source_total_lap_gnss_ns);
  p.add("source_custody_lap_count",
        g_photons_recovery.source_custody_lap_count);
  p.add("source_custody_total_lap_gnss_ns",
        g_photons_recovery.source_custody_total_lap_gnss_ns);
  p.add("proof_sequence", g_photons_recovery.proof_sequence);
  p.add("proof_update_count", g_photons_recovery.proof_update_count);
  p.add("dropped_pending_seed_count",
        g_photons_recovery.dropped_pending_seed_count);
  p.add("fresh_physical_ancestry",
        g_photons_recovery.publication_started);
  p.add("raw_lap_ring_restored", false);
  p.add("partial_lap_restored", false);
  p.add("pending_seed_restored", false);
  p.add("predictor_restored", false);
  p.add("in_flight_train_restored", false);
  p.add("fragment_sequence", g_fragment_sequence);
  p.add("publish_count", g_publish_count);
  p.add("stats_reset_count", g_photons_stats_reset_count);
  p.add("stats_update_count", g_photons_stats_update_count);
  p.add("stats_lap_count", g_lap_time_welford.n);
  p.add("stats_total_lap_gnss_ns", g_total_lap_gnss_ns);
  p.add("custody_lap_count", g_photons_custody_lap_count);
  p.add("custody_total_lap_gnss_ns",
        g_photons_custody_total_lap_gnss_ns);
  p.add("campaign_state", photons_campaign_state_name(g_photons_campaign_state));
  p.add("campaign", g_photons_campaign_name);
  p.add("campaign_public_count", g_photons_campaign_public_count);
  p.add("begin_count", g_photons_recovery.begin_count);
  p.add("chunk_count", g_photons_recovery.chunk_count);
  p.add("commit_count", g_photons_recovery.commit_count);
  p.add("abort_count", g_photons_recovery.abort_count);
  p.add("cold_start_count", g_photons_recovery.cold_start_count);
  p.add("proof_ack_count", g_photons_recovery.proof_ack_count);
  p.add("reject_count", g_photons_recovery.reject_count);
  return p;
}


static FLASHMEM Payload cmd_flash_cut(const Payload& args) {
  if (!g_photons_recovery.publication_started ||
      g_photons_recovery.proof_pending) {
    return photons_recovery_reject(
        "flash_cut_rejected_recovery_pending",
        "PHOTONS recovery verdict is not complete");
  }

  const char* name = args.getString("campaign");
  if (!name || !*name) {
    g_photons_flash_cut_reject_count++;
    Payload err;
    err.add("status", "flash_cut_rejected_missing_campaign");
    err.add("error", "missing campaign");
    return err;
  }
  if (strlen(name) >= sizeof(g_photons_flash_cut_campaign_name)) {
    g_photons_flash_cut_reject_count++;
    Payload err;
    err.add("status", "flash_cut_rejected_campaign_name_too_long");
    err.add("error", "campaign name exceeds firmware capacity");
    return err;
  }
  if (g_photons_campaign_state != photons_campaign_state_t::ACTIVE) {
    g_photons_flash_cut_reject_count++;
    Payload err;
    err.add("status", "flash_cut_rejected_not_active");
    err.add("state", photons_campaign_state_name(g_photons_campaign_state));
    return err;
  }
  if (!strcmp(name, g_photons_campaign_name)) {
    g_photons_flash_cut_reject_count++;
    Payload err;
    err.add("status", "flash_cut_rejected_same_campaign");
    err.add("campaign", g_photons_campaign_name);
    return err;
  }

  safeCopy(g_photons_flash_cut_campaign_name,
           sizeof(g_photons_flash_cut_campaign_name), name);
  g_photons_flash_cut_request_count++;
  g_photons_campaign_state = photons_campaign_state_t::FLASH_CUT_PENDING;

  Payload p;
  p.add("status", "flash_cut_requested");
  p.add("current_campaign", g_photons_campaign_name);
  p.add("campaign", g_photons_flash_cut_campaign_name);
  p.add("boundary_contract",
        "NEXT_SUCCESSFULLY_PUBLISHED_OLD_CAMPAIGN_FRAGMENT_IS_FINAL_AND_NEW_PRIVATE_ORIGIN");
  p.add("instrument_always_on", true);
  p.add("statistics_preserved", true);
  return p;
}


static FLASHMEM Payload cmd_start(const Payload& args) {
  if (!g_photons_recovery.publication_started ||
      g_photons_recovery.proof_pending) {
    return photons_recovery_reject(
        "start_rejected_recovery_pending",
        "PHOTONS recovery verdict is not complete");
  }

  const char* name = args.getString("campaign");
  if (!name || !*name) {
    Payload err;
    err.add("status", "start_rejected_missing_campaign");
    err.add("error", "missing campaign");
    return err;
  }
  if (strlen(name) >= sizeof(g_photons_campaign_name)) {
    Payload err;
    err.add("status", "start_rejected_campaign_name_too_long");
    err.add("error", "campaign name exceeds firmware capacity");
    return err;
  }
  if (g_photons_campaign_state == photons_campaign_state_t::ACTIVE) {
    return cmd_flash_cut(args);
  }
  if (g_photons_campaign_state != photons_campaign_state_t::STOPPED) {
    Payload err;
    err.add("status", "start_rejected_campaign_busy");
    err.add("state", photons_campaign_state_name(g_photons_campaign_state));
    return err;
  }

  safeCopy(g_photons_campaign_name, sizeof(g_photons_campaign_name), name);
  g_photons_campaign_start_request_count++;
  g_photons_campaign_state = photons_campaign_state_t::START_PENDING;

  Payload p;
  p.add("status", "start_requested");
  p.add("campaign", g_photons_campaign_name);
  p.add("boundary_contract", "NEXT_SUCCESSFULLY_PUBLISHED_FRAGMENT_IS_PRIVATE_ORIGIN");
  return p;
}


static FLASHMEM Payload cmd_stop(const Payload& /*args*/) {
  if (!g_photons_recovery.publication_started ||
      g_photons_recovery.proof_pending) {
    return photons_recovery_reject(
        "stop_rejected_recovery_pending",
        "PHOTONS recovery verdict is not complete");
  }
  if (g_photons_campaign_state == photons_campaign_state_t::START_PENDING) {
    Payload err;
    err.add("status", "stop_rejected_start_pending");
    err.add("campaign", g_photons_campaign_name);
    return err;
  }
  if (g_photons_campaign_state == photons_campaign_state_t::STOPPED) {
    Payload err;
    err.add("status", "stop_rejected_no_campaign");
    return err;
  }
  if (g_photons_campaign_state == photons_campaign_state_t::STOP_PENDING) {
    Payload p;
    p.add("status", "stop_requested");
    p.add("campaign", g_photons_campaign_name);
    return p;
  }
  if (g_photons_campaign_state == photons_campaign_state_t::FLASH_CUT_PENDING) {
    Payload err;
    err.add("status", "stop_rejected_flash_cut_pending");
    err.add("campaign", g_photons_campaign_name);
    err.add("next_campaign", g_photons_flash_cut_campaign_name);
    return err;
  }
  if (g_photons_campaign_state != photons_campaign_state_t::ACTIVE) {
    __builtin_trap();
  }

  g_photons_campaign_stop_request_count++;
  g_photons_campaign_state = photons_campaign_state_t::STOP_PENDING;

  Payload p;
  p.add("status", "stop_requested");
  p.add("campaign", g_photons_campaign_name);
  p.add("boundary_contract", "NEXT_SUCCESSFULLY_PUBLISHED_CAMPAIGN_FRAGMENT_IS_FINAL");
  return p;
}


static void photons_payload_add_flat_ppb_bucket(
    Payload& p,
    const char* prefix,
    const photons_fragment_ppb_value_snapshot_t& value) {
  char key[48];
  snprintf(key, sizeof(key), "%s_n", prefix);
  p.add(key, value.sample_count);
  if (value.sample_count != 0ULL) {
    snprintf(key, sizeof(key), "%s_ppb", prefix);
    p.add(key, toFixedDecimal(value.ppb, 6));
    snprintf(key, sizeof(key), "%s_residual_ns", prefix);
    p.add(key, toFixedDecimal(value.residual_ns, 6));
  }
}


static FLASHMEM Payload cmd_report_photons(const Payload& /*args*/) {
  photons_fragment_snapshot_t canonical{};
  (void)photons_fragment_snapshot(&canonical);

  Payload p;
  p.add("report", "PHOTONS_INSTRUMENT");
  p.add("schema", "PHOTONS_INSTRUMENT_REPORT_V1");
  p.add("ppb_semantics", "LAP_BASELINE_NS_OFFSET_V1");
  p.add("instrument_always_on", true);
  p.add("instrument_owner", "TEENSY.PHOTONS");
  p.add("publication_started", g_photons_recovery.publication_started);
  p.add("recovery_restored", g_photons_recovery.restored);
  p.add("recovery_proof_pending", g_photons_recovery.proof_pending);
  p.add("recovery_proof_committed", g_photons_recovery.proof_committed);
  p.add("recovery_generation", g_photons_recovery.generation);
  p.add("snapshot_ok", canonical.snapshot_ok);
  p.add("valid", canonical.valid);
  p.add("standard_lap_configured", g_standard_lap_configured);
  p.add("lap_baseline_configured", g_standard_lap_configured);
  if (g_standard_lap_configured) {
    p.add("lap_baseline_fs", g_lap_baseline_fs);
    p.add("lap_baseline_ns",
          toFixedDecimal((double)g_lap_baseline_fs / 1000000.0, 6));
    p.add("standard_lap_ps", g_standard_lap_ps);
    p.add("standard_lap_ns",
          toFixedDecimal((double)g_standard_lap_ps / 1000.0, 3));
  }
  p.add("stats_reset_count", canonical.stats.reset_count);
  p.add("stats_update_count", canonical.stats.update_count);
  p.add("stats_reset_pending", g_photons_stats_reset_pending);
  p.add("race_engine_active",
        g_photons_race.cadence_timer != TIMEPOP_INVALID_HANDLE);
  p.add("race_cadence_hz", PHOTONS_RACE_CADENCE_HZ);
  p.add("race_pulse_ns", PHOTONS_RACE_PULSE_NS);
  p.add("race_launch_surrogate", "LD_ON_FALLING_EDGE");
  p.add("race_cadence_tick_count_total", canonical.race_cadence_tick_count_total);
  p.add("race_cadence_ticks_this_fragment", canonical.race_cadence_ticks_this_fragment);
  p.add("race_attempt_count_total", canonical.race_attempt_count_total);
  p.add("race_attempts_this_fragment", canonical.race_attempts_this_fragment);
  p.add("race_completed_count_total", canonical.race_completed_count_total);
  p.add("race_completed_this_fragment", canonical.race_completed_this_fragment);
  p.add("race_missed_count_total", canonical.race_missed_count_total);
  p.add("race_missed_this_fragment", canonical.race_missed_this_fragment);
  p.add("race_skipped_not_quiet_total", canonical.race_skipped_not_quiet_total);
  p.add("race_skipped_projection_total", canonical.race_skipped_projection_total);
  p.add("race_invalid_endpoint_total", canonical.race_invalid_endpoint_total);
  p.add("race_enqueue_failure_total", canonical.race_enqueue_failure_total);
  p.add("race_flight_n_this_fragment", canonical.race_flight_this_fragment.n);
  p.add("race_flight_mean_ns_this_fragment",
        toFixedDecimal(canonical.race_flight_this_fragment.mean, 6));
  p.add("race_flight_stddev_ns_this_fragment",
        toFixedDecimal(canonical.race_flight_this_fragment.stddev, 6));
  p.add("race_flight_stderr_ns_this_fragment",
        toFixedDecimal(canonical.race_flight_this_fragment.stderr_value, 6));
  p.add("interrupt_subscribed", canonical.interrupt_subscribed);
  p.add("interrupt_active", canonical.interrupt_active);
  p.add("interrupt_blocker_trace_count", canonical.interrupt_blocker_trace_count);
  p.add("interrupt_blocked_ocxo2_count", canonical.interrupt_blocked_ocxo2_count);
  p.add("interrupt_last_blocker_wall_cycles",
        canonical.interrupt_last_blocker_wall_cycles);
  p.add("interrupt_max_blocker_wall_cycles",
        canonical.interrupt_max_blocker_wall_cycles);
  p.add("interrupt_last_qtimer_pending_at_entry_mask",
        canonical.interrupt_last_qtimer_pending_at_entry_mask);
  p.add("interrupt_last_qtimer_pending_at_exit_mask",
        canonical.interrupt_last_qtimer_pending_at_exit_mask);
  p.add("race_count", canonical.stats.lap_count);
  p.add("total_flight_gnss_ns", canonical.stats.total_lap_gnss_ns);
  p.add("mean_flight_ns", toFixedDecimal(canonical.stats.mean_lap_ns, 6));
  // Legacy aliases remain during the schema-name migration.
  p.add("lap_count", canonical.stats.lap_count);
  p.add("total_lap_gnss_ns", canonical.stats.total_lap_gnss_ns);
  p.add("mean_lap_ns", toFixedDecimal(canonical.stats.mean_lap_ns, 6));
  p.add("lap_baseline_ns",
        toFixedDecimal((double)canonical.stats.lap_baseline_fs / 1000000.0, 6));
  if (canonical.baseline.residual_valid) {
    p.add("mean_residual_ns",
          toFixedDecimal(canonical.baseline.mean_residual_ns, 6));
  }
  photons_payload_add_flat_ppb_bucket(p, "ppb_10_min",
                                      canonical.stats.ppb_buckets.minute_10);
  photons_payload_add_flat_ppb_bucket(p, "ppb_60_min",
                                      canonical.stats.ppb_buckets.minute_60);
  photons_payload_add_flat_ppb_bucket(p, "ppb_8_hour",
                                      canonical.stats.ppb_buckets.hour_8);
  photons_payload_add_flat_ppb_bucket(p, "ppb_24_hour",
                                      canonical.stats.ppb_buckets.hour_24);
  photons_payload_add_flat_ppb_bucket(p, "ppb_total",
                                      canonical.stats.ppb_buckets.total);
  p.add("campaign_state", photons_campaign_state_name(g_photons_campaign_state));
  if (g_photons_campaign_name[0]) p.add("campaign", g_photons_campaign_name);
  p.add("campaign_public_count", g_photons_campaign_public_count);
  p.add("stats_epoch_current",
        canonical.stats.reset_count == g_photons_stats_reset_count);
  if (canonical.campaign.present &&
      !strcmp(canonical.campaign.campaign, g_photons_campaign_name) &&
      canonical.campaign.ppb.sample_count != 0ULL) {
    p.add("campaign_race_count", canonical.campaign.lap_count);
    p.add("campaign_mean_flight_ns",
          toFixedDecimal(canonical.campaign.mean_lap_ns, 6));
    p.add("campaign_lap_count", canonical.campaign.lap_count);
    p.add("campaign_mean_lap_ns",
          toFixedDecimal(canonical.campaign.mean_lap_ns, 6));
    p.add("campaign_ppb", toFixedDecimal(canonical.campaign.ppb.ppb, 6));
    p.add("campaign_residual_ns",
          toFixedDecimal(canonical.campaign.ppb.residual_ns, 6));
  }
  p.add("custody_lap_count", g_photons_custody_lap_count);
  p.add("custody_total_lap_gnss_ns", g_photons_custody_total_lap_gnss_ns);
  return p;
}


static FLASHMEM Payload cmd_report_stats(const Payload& /*args*/) {
  photons_fragment_snapshot_t canonical{};
  (void)photons_fragment_snapshot(&canonical);

  Payload p;
  p.add("report", "PHOTONS_STATS");
  p.add("schema", "PHOTONS_INSTRUMENT_STATS_REPORT_V1");
  p.add("ppb_semantics", "LAP_BASELINE_NS_OFFSET_V1");
  p.add("publication_started", g_photons_recovery.publication_started);
  p.add("recovery_restored", g_photons_recovery.restored);
  p.add("recovery_proof_pending", g_photons_recovery.proof_pending);
  p.add("recovery_proof_sequence", g_photons_recovery.proof_sequence);
  p.add("recovery_proof_update_count",
        g_photons_recovery.proof_update_count);
  p.add("snapshot_ok", canonical.snapshot_ok);
  p.add("valid", canonical.stats.valid);
  p.add("raw_lap_ring_capacity", PHOTONS_LAP_RING_CAPACITY);
  p.add("raw_lap_ring_overflow_count",
        (uint32_t)g_raw_lap_ring_overflow_count);
  p.add("raw_lap_ring_data_loss",
        (bool)g_raw_lap_ring_data_loss);
  p.add("reset_count", canonical.stats.reset_count);
  p.add("update_count", canonical.stats.update_count);
  p.add("reset_pending", g_photons_stats_reset_pending);
  p.add("reset_request_count", g_photons_stats_reset_request_count);
  p.add("reset_commit_count", g_photons_stats_reset_commit_count);
  p.add("lap_count", canonical.stats.lap_count);
  p.add("total_lap_gnss_ns", canonical.stats.total_lap_gnss_ns);
  p.add("race_count", canonical.stats.lap_count);
  p.add("mean_flight_ns", toFixedDecimal(canonical.stats.mean_lap_ns, 6));
  p.add("mean_lap_ns", toFixedDecimal(canonical.stats.mean_lap_ns, 6));
  p.add("lap_baseline_fs", canonical.stats.lap_baseline_fs);
  p.add("lap_baseline_ns",
        toFixedDecimal((double)canonical.stats.lap_baseline_fs / 1000000.0, 6));
  if (canonical.baseline.residual_valid) {
    p.add("mean_residual_ns",
          toFixedDecimal(canonical.baseline.mean_residual_ns, 6));
  }
  p.add("lap_welford_n", canonical.stats.lap_time_welford.n);
  p.add("lap_welford_mean",
        toFixedDecimal(canonical.stats.lap_time_welford.mean, 6));
  p.add("lap_welford_m2",
        toFixedDecimal(canonical.stats.lap_time_welford.m2, 6));
  p.add("lap_welford_stddev",
        toFixedDecimal(canonical.stats.lap_time_welford.stddev, 6));
  p.add("lap_welford_stderr",
        toFixedDecimal(canonical.stats.lap_time_welford.stderr_value, 6));
  p.add("science_candidate_count", canonical.science.candidate_count);
  p.add("science_accepted_count", canonical.science.accepted.count);
  p.add(
      "science_excluded_count",
      photons_lap_science_excluded_count_from_reasons(
          canonical.science.exclusion_reasons));
  p.add("science_exclusion_projection_invalid",
        canonical.science.exclusion_reasons.projection_invalid);
  p.add("science_exclusion_seed_disagreement",
        canonical.science.exclusion_reasons.seed_disagreement);
  p.add("science_exclusion_raw_cycle_excursion",
        canonical.science.exclusion_reasons.raw_cycle_excursion);
  photons_payload_add_flat_ppb_bucket(p, "ppb_10_min",
                                      canonical.stats.ppb_buckets.minute_10);
  photons_payload_add_flat_ppb_bucket(p, "ppb_60_min",
                                      canonical.stats.ppb_buckets.minute_60);
  photons_payload_add_flat_ppb_bucket(p, "ppb_8_hour",
                                      canonical.stats.ppb_buckets.hour_8);
  photons_payload_add_flat_ppb_bucket(p, "ppb_24_hour",
                                      canonical.stats.ppb_buckets.hour_24);
  photons_payload_add_flat_ppb_bucket(p, "ppb_total",
                                      canonical.stats.ppb_buckets.total);
  p.add("rolling_ppb_current_sequence",
        canonical.stats.rolling_ppb_current_sequence);
  p.add("rolling_ppb_endpoint_admitted",
        canonical.stats.rolling_ppb_endpoint_admitted);
  p.add("rolling_ppb_interval_advanced",
        canonical.stats.rolling_ppb_interval_advanced);
  p.add("custody_lap_count", g_photons_custody_lap_count);
  p.add("custody_total_lap_gnss_ns", g_photons_custody_total_lap_gnss_ns);
  p.add("campaign_state", photons_campaign_state_name(g_photons_campaign_state));
  p.add("stats_epoch_current",
        canonical.stats.reset_count == g_photons_stats_reset_count);
  if (canonical.campaign.present &&
      !strcmp(canonical.campaign.campaign, g_photons_campaign_name)) {
    p.add("campaign", canonical.campaign.campaign);
    p.add("campaign_public_count", canonical.campaign.public_count);
    p.add("campaign_lap_count", canonical.campaign.lap_count);
    p.add("campaign_total_lap_gnss_ns", canonical.campaign.total_lap_gnss_ns);
    if (canonical.campaign.ppb.sample_count != 0ULL) {
        p.add("campaign_ppb", toFixedDecimal(canonical.campaign.ppb.ppb, 6));
      p.add("campaign_residual_ns",
            toFixedDecimal(canonical.campaign.ppb.residual_ns, 6));
    }
  }
  return p;
}


static FLASHMEM Payload cmd_stats_reset(const Payload& /*args*/) {
  if (!g_photons_recovery.publication_started ||
      g_photons_recovery.proof_pending) {
    return photons_recovery_reject(
        "stats_reset_rejected_recovery_pending",
        "PHOTONS recovery verdict is not complete");
  }

  Payload p;
  if (g_photons_stats_reset_pending) {
    p.add("status", "instrument_statistics_reset_requested");
    p.add("reset_pending", true);
    p.add("current_reset_count", g_photons_stats_reset_count);
    p.add("expected_reset_count", g_photons_stats_reset_count + 1U);
    p.add("request_count", g_photons_stats_reset_request_count);
    p.add("commit_count", g_photons_stats_reset_commit_count);
    return p;
  }

  g_photons_stats_reset_pending = true;
  g_photons_stats_reset_request_count++;
  p.add("status", "instrument_statistics_reset_requested");
  p.add("reset", true);
  p.add("reset_pending", true);
  p.add("current_reset_count", g_photons_stats_reset_count);
  p.add("expected_reset_count", g_photons_stats_reset_count + 1U);
  p.add("request_count", g_photons_stats_reset_request_count);
  p.add("commit_count", g_photons_stats_reset_commit_count);
  p.add("boundary", "AFTER_NEXT_SUCCESSFULLY_PUBLISHED_FRAGMENT");
  p.add("campaign_changed", false);
  p.add("custody_preserved", true);
  p.add("standard_lap_preserved", true);
  p.add("lap_baseline_preserved", true);
  p.add("next_report", "REPORT_STATS");
  return p;
}

static FLASHMEM Payload cmd_inject_problem(const Payload& /*args*/) {
  Payload p;
  p.add("status", "inject_problem_rejected_source");
  p.add("error",
        "synthetic excursion injection was retired with the PHOTONS emulator");
  p.add("source", "PD200T_REAL_RACE");
  p.add("physical_measurement_modified", false);
  return p;
}

static FLASHMEM Payload cmd_report(const Payload& /*args*/) {
  photons_fragment_snapshot_t canonical{};
  (void)photons_fragment_snapshot(&canonical);

  interrupt_photodiode_diag_t interrupt_diag{};
  (void)interrupt_photodiode_snapshot(&interrupt_diag);
  const photons_device_snapshot_t device = photons_device_snapshot();

  Payload p;
  p.add("report", "PHOTONS");
  p.add("schema", "PHOTONS_REPORT_V3");
  p.add("ppb_semantics", "LAP_BASELINE_NS_OFFSET_V1");
  p.add("initialized", g_initialized);
  p.add("publication_started", g_photons_recovery.publication_started);
  p.add("standard_lap_configured", g_standard_lap_configured);
  p.add("lap_baseline_configured", g_standard_lap_configured);
  if (g_standard_lap_configured) {
    p.add("lap_baseline_fs", g_lap_baseline_fs);
    p.add("lap_baseline_ns",
          toFixedDecimal((double)g_lap_baseline_fs / 1000000.0, 6));
    p.add("standard_lap_ps", g_standard_lap_ps);
  }
  p.add("recovery_restored", g_photons_recovery.restored);
  p.add("recovery_proof_pending", g_photons_recovery.proof_pending);
  p.add("recovery_proof_committed", g_photons_recovery.proof_committed);
  p.add("recovery_generation", g_photons_recovery.generation);

  p.add("campaign_state", photons_campaign_state_name(g_photons_campaign_state));
  if (g_photons_campaign_name[0]) {
    p.add("campaign", g_photons_campaign_name);
    p.add("campaign_start_after_sequence",
          g_photons_campaign_start_after_sequence);
  }
  p.add("campaign_public_count", g_photons_campaign_public_count);

  p.add("fragment_sequence", canonical.sequence);
  p.add("fragment_valid", canonical.valid);
  p.add("race_engine_active",
        g_photons_race.cadence_timer != TIMEPOP_INVALID_HANDLE);
  p.add("race_cadence_hz", PHOTONS_RACE_CADENCE_HZ);
  p.add("race_pulse_ns", PHOTONS_RACE_PULSE_NS);
  p.add("race_launch_surrogate", "LD_ON_FALLING_EDGE");
  p.add("race_cadence_tick_count_total", g_photons_race.cadence_tick_count);
  p.add("race_cadence_ticks_this_fragment", canonical.race_cadence_ticks_this_fragment);
  p.add("race_attempt_count_total", g_photons_race.attempt_count);
  p.add("race_completed_count_total", g_photons_race.completed_count);
  p.add("race_missed_count_total", g_photons_race.missed_count);
  p.add("race_skipped_not_quiet_total",
        g_photons_race.skipped_not_quiet_count);
  p.add("race_skipped_projection_total",
        g_photons_race.skipped_projection_count);
  p.add("race_invalid_endpoint_total",
        g_photons_race.invalid_endpoint_count);
  p.add("race_enqueue_failure_total", g_photons_race.enqueue_failure_count);
  p.add("race_attempts_this_fragment", canonical.race_attempts_this_fragment);
  p.add("race_completed_this_fragment", canonical.race_completed_this_fragment);
  p.add("race_missed_this_fragment", canonical.race_missed_this_fragment);
  p.add("race_flight_n_this_fragment", canonical.race_flight_this_fragment.n);
  if (canonical.race_flight_this_fragment.n != 0ULL) {
    p.add("race_flight_mean_ns_this_fragment",
          toFixedDecimal(canonical.race_flight_this_fragment.mean, 6));
    p.add("race_flight_stddev_ns_this_fragment",
          toFixedDecimal(canonical.race_flight_this_fragment.stddev, 6));
    p.add("race_flight_stderr_ns_this_fragment",
          toFixedDecimal(canonical.race_flight_this_fragment.stderr_value, 6));
  }
  p.add("race_count_total_science", canonical.stats.lap_count);
  if (canonical.stats.lap_count != 0ULL) {
    p.add("mean_flight_ns", toFixedDecimal(canonical.stats.mean_lap_ns, 6));
  }

  p.add("interrupt_subscribed", interrupt_diag.subscribed);
  p.add("interrupt_active", interrupt_diag.active);
  p.add("interrupt_callback_count", interrupt_diag.callback_count);
  p.add("interrupt_callback_missing_count",
        interrupt_diag.callback_missing_count);
  p.add("interrupt_ancestry_baseline_valid", g_interrupt_ancestry.valid);
  if (g_interrupt_ancestry.valid) {
    p.add("interrupt_callback_missing_count_origin",
          g_interrupt_ancestry.callback_missing_origin);
    p.add("interrupt_callback_missing_count_since_ancestry",
          interrupt_diag.callback_missing_count -
              g_interrupt_ancestry.callback_missing_origin);
    p.add("interrupt_inactive_edge_count_origin",
          g_interrupt_ancestry.inactive_edge_origin);
    p.add("interrupt_inactive_edge_count_since_ancestry",
          interrupt_diag.inactive_edge_count -
              g_interrupt_ancestry.inactive_edge_origin);
  }
  p.add("interrupt_inactive_edge_count", interrupt_diag.inactive_edge_count);
  p.add("interrupt_blocker_trace_count", interrupt_diag.blocker_trace_count);
  p.add("interrupt_blocked_qtimer1_count", interrupt_diag.blocked_qtimer1_count);
  p.add("interrupt_blocked_ocxo1_count", interrupt_diag.blocked_ocxo1_count);
  p.add("interrupt_blocked_ocxo2_count", interrupt_diag.blocked_ocxo2_count);
  p.add("interrupt_last_blocker_wall_cycles",
        interrupt_diag.last_blocker_wall_cycles);
  p.add("interrupt_max_blocker_wall_cycles",
        interrupt_diag.max_blocker_wall_cycles);
  p.add("interrupt_last_qtimer_pending_at_entry_mask",
        interrupt_diag.last_qtimer_pending_at_entry_mask);
  p.add("interrupt_last_qtimer_pending_at_exit_mask",
        interrupt_diag.last_qtimer_pending_at_exit_mask);
  p.add("photodiode_edge_level", device.photodiode_edge_level);
  p.add("photodiode_analog_v",
        toFixedDecimal(device.photodiode_analog_v, 6));
  p.add("laser_monitor_v", toFixedDecimal(device.laser_monitor_v, 6));
  return p;
}

static FLASHMEM Payload cmd_report_pulse(const Payload& /*args*/) {
  interrupt_photodiode_diag_t interrupt_diag{};
  (void)interrupt_photodiode_snapshot(&interrupt_diag);

  photons_pulse_receive_value_t pulse_receive{};
  (void)photons_pulse_receive_snapshot(&pulse_receive);
  const photons_pulse_launch_state_t pulse_launch = g_last_pulse_launch;

  Payload p;
  p.add("report", "PHOTONS_PULSE");
  p.add("schema", "PHOTONS_PULSE_REPORT_V2");
  p.add("race_engine_active",
        g_photons_race.cadence_timer != TIMEPOP_INVALID_HANDLE);
  p.add("interrupt_callback_count", interrupt_diag.callback_count);
  p.add("interrupt_callback_missing_count",
        interrupt_diag.callback_missing_count);
  p.add("pulse_available", pulse_launch.valid);
  if (!pulse_launch.valid) return p;

  p.add("pulse_sequence", pulse_launch.sequence);
  p.add("pulse_requested_ns", pulse_launch.requested_ns);
  p.add("pulse_wall_cycles", pulse_launch.pulse_wall_cycles);
  p.add("pulse_armed", g_pulse_armed_sequence == pulse_launch.sequence);
  p.add("pulse_callback_delta",
        interrupt_diag.callback_count - pulse_launch.callback_count_start);

  const bool finish_seen =
      pulse_receive.seen &&
      pulse_receive.pulse_sequence == pulse_launch.sequence;
  p.add("receive_seen", finish_seen);

  uint64_t finish_gnss_ns = 0ULL;
  const bool finish_gnss_valid =
      finish_seen &&
      time_clock_ns_at_dwt(time_clock_id_t::GNSS,
                           pulse_receive.finish_dwt,
                           &finish_gnss_ns);
  const bool flight_time_valid =
      pulse_launch.start_gnss_valid &&
      finish_gnss_valid &&
      finish_gnss_ns >= pulse_launch.start_gnss_ns;
  p.add("flight_time_valid", flight_time_valid);
  if (flight_time_valid) {
    p.add("flight_time_gnss_ns",
          finish_gnss_ns - pulse_launch.start_gnss_ns);
  }
  return p;
}

static FLASHMEM Payload cmd_init(const Payload& /*args*/) {
  if (g_photons_race.cadence_timer != TIMEPOP_INVALID_HANDLE) {
    Payload p;
    p.add("status", "init_rejected_race_engine_active");
    return p;
  }
  photons_laser_initialize_hardware();
  photons_emit_laser_initialization_event();
  return ok_payload();
}

static FLASHMEM Payload cmd_pulse(const Payload& args) {
  if (g_photons_race.cadence_timer != TIMEPOP_INVALID_HANDLE) {
    Payload p;
    p.add("status", "pulse_rejected_race_engine_active");
    return p;
  }

  if (digitalRead(LD_ON_PIN) != LOW) {
    Payload p;
    p.add("status", "pulse_rejected_laser_not_inhibited");
    return p;
  }

  if (digitalRead(PHOTODIODE_EDGE_PIN) != LOW) {
    Payload p;
    p.add("status", "pulse_rejected_detector_not_quiet");
    return p;
  }

  // Manual PULSE remains a commissioning command. It requires the live GNSS/DWT
  // ruler for width conversion and report projection but performs no ADC work
  // inside the commanded HIGH interval.
  time_clock_projection_t projection_preflight{};
  if (!time_clock_projection(time_clock_id_t::GNSS,
                             &projection_preflight)) {
    Payload p;
    p.add("status", "pulse_rejected_gnss_projection_unavailable");
    return p;
  }

  uint64_t requested_ns = PHOTONS_PULSE_DEFAULT_NS;
  if (args.has("ns") && !args.tryGetUInt64("ns", requested_ns)) {
    Payload p;
    p.add("status", "pulse_rejected_ns_invalid");
    return p;
  }

  uint32_t target_high_cycles = 0U;
  if (!photons_ns_to_dwt_cycles(
          requested_ns,
          projection_preflight.dwt_cycles_per_second,
          target_high_cycles)) {
    Payload p;
    p.add("status", "pulse_rejected_ns_out_of_range");
    p.add("requested_ns", requested_ns);
    return p;
  }

  interrupt_photodiode_diag_t interrupt_before{};
  if (!interrupt_photodiode_snapshot(&interrupt_before)) {
    Payload p;
    p.add("status", "pulse_rejected_interrupt_snapshot_unavailable");
    return p;
  }

  const uint32_t overwritten_pending_sequence = g_pulse_armed_sequence;
  g_pulse_armed_sequence = 0U;
  photons_compiler_barrier();

  g_pulse_sequence++;
  if (g_pulse_sequence == 0U) g_pulse_sequence++;
  const uint32_t pulse_sequence = g_pulse_sequence;

  g_last_pulse_launch = photons_pulse_launch_state_t{};
  photons_pulse_receive_publish(photons_pulse_receive_value_t{});
  g_pulse_armed_sequence = pulse_sequence;
  photons_compiler_barrier();

  const uint32_t start_dwt = ARM_DWT_CYCCNT;
  digitalWriteFast(LD_ON_PIN, HIGH);
  const uint32_t high_start = ARM_DWT_CYCCNT;
  while ((uint32_t)(ARM_DWT_CYCCNT - high_start) < target_high_cycles) {
  }
  digitalWriteFast(LD_ON_PIN, LOW);
  const uint32_t pulse_wall_cycles = ARM_DWT_CYCCNT - start_dwt;

  uint64_t start_gnss_ns = 0ULL;
  const bool start_gnss_valid =
      time_clock_ns_at_dwt(time_clock_id_t::GNSS,
                           start_dwt,
                           &start_gnss_ns);

  g_last_pulse_launch.sequence = pulse_sequence;
  g_last_pulse_launch.requested_ns = requested_ns;
  g_last_pulse_launch.target_high_cycles = target_high_cycles;
  g_last_pulse_launch.start_dwt = start_dwt;
  g_last_pulse_launch.start_gnss_valid = start_gnss_valid;
  g_last_pulse_launch.start_gnss_ns =
      start_gnss_valid ? start_gnss_ns : 0ULL;
  g_last_pulse_launch.pulse_wall_cycles = pulse_wall_cycles;
  g_last_pulse_launch.callback_count_start = interrupt_before.callback_count;
  photons_compiler_barrier();
  g_last_pulse_launch.valid = true;

  Payload p;
  p.add("status", "pulse_fired");
  p.add("pulse_sequence", pulse_sequence);
  p.add("pulse_requested_ns", requested_ns);
  p.add("pulse_target_high_cycles", target_high_cycles);
  p.add("pulse_wall_cycles", pulse_wall_cycles);
  p.add("pulse_start_dwt", start_dwt);
  p.add("pulse_start_gnss_valid", start_gnss_valid);
  if (start_gnss_valid) p.add("pulse_start_gnss_ns", start_gnss_ns);
  p.add("previous_receive_pending", overwritten_pending_sequence != 0U);
  if (overwritten_pending_sequence != 0U) {
    p.add("overwritten_pending_sequence", overwritten_pending_sequence);
  }
  return p;
}

static FLASHMEM Payload cmd_on(const Payload& /*args*/) {
  if (g_photons_race.cadence_timer != TIMEPOP_INVALID_HANDLE) {
    Payload p;
    p.add("status", "on_rejected_race_engine_active");
    return p;
  }
  digitalWrite(LD_ON_PIN, HIGH);

  Payload ev;
  ev.add("action", "allow_emission");
  enqueueEvent("LASER_ON", ev);

  return ok_payload();
}

static FLASHMEM Payload cmd_off(const Payload& /*args*/) {
  if (g_photons_race.cadence_timer != TIMEPOP_INVALID_HANDLE) {
    Payload p;
    p.add("status", "off_rejected_race_engine_active");
    return p;
  }
  photons_laser_inhibit();

  Payload ev;
  ev.add("action", "inhibit_emission");
  enqueueEvent("LASER_OFF", ev);

  return ok_payload();
}

// ============================================================================
// Registration
// ============================================================================

static const process_command_entry_t PHOTONS_COMMANDS[] = {
  { "INIT",                cmd_init                },
  { "SET_LAP_BASELINE_NS", cmd_set_lap_baseline_ns },
  { "SET_STANDARD_LAP_NS", cmd_set_standard_lap_ns },
  { "START",               cmd_start               },
  { "FLASH_CUT",           cmd_flash_cut           },
  { "STOP",                cmd_stop                },
  { "REPORT",              cmd_report              },
  { "REPORT_PULSE",        cmd_report_pulse        },
  { "REPORT_PHOTONS",      cmd_report_photons      },
  { "REPORT_STATS",        cmd_report_stats        },
  { "STATS_RESET",         cmd_stats_reset         },
  { "PPB_EXPORT_META",     cmd_ppb_export_meta     },
  { "PPB_EXPORT_CHUNK",    cmd_ppb_export_chunk    },
  { "RECOVERY_BEGIN",      cmd_recovery_begin      },
  { "RECOVERY_CHUNK",      cmd_recovery_chunk      },
  { "RECOVERY_COMMIT",     cmd_recovery_commit     },
  { "RECOVERY_ABORT",      cmd_recovery_abort      },
  { "RECOVERY_COLD_START", cmd_recovery_cold_start },
  { "RECOVERY_PROOF_ACK",  cmd_recovery_proof_ack  },
  { "REPORT_RECOVERY",     cmd_report_recovery     },
  { "INJECT_PROBLEM",      cmd_inject_problem      },
  { "PULSE",               cmd_pulse               },
  { "ON",                  cmd_on                  },
  { "OFF",                 cmd_off                 },
  { nullptr, nullptr }
};

static const process_vtable_t PHOTONS_PROCESS = {
  .process_id = "PHOTONS",
  .commands   = PHOTONS_COMMANDS,
};

FLASHMEM void process_photons_register(void) {
  process_register("PHOTONS", &PHOTONS_PROCESS);
}
