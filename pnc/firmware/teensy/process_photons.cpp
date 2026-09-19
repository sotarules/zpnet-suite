#include "process_photons.h"

#include "config.h"
#include "payload.h"
#include "process.h"
#include "process_interrupt.h"
#include "publish.h"
#include "timepop.h"
#include "time.h"
#include "util.h"

#include <Arduino.h>
#include <errno.h>
#include <math.h>
#include <stddef.h>
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

// i.MX RT1062 RAM2 is write-back cached in 32-byte lines.  PHOTONS places only
// large value stores/history in RAM2; align every PHOTONS-owned RAM2 object to a
// complete cache-line boundary so unrelated ownership domains never share one
// cache line merely because the linker packed them together.
static constexpr size_t PHOTONS_RAM2_CACHE_LINE_BYTES = 32U;
static_assert((PHOTONS_RAM2_CACHE_LINE_BYTES &
               (PHOTONS_RAM2_CACHE_LINE_BYTES - 1U)) == 0U,
              "PHOTONS RAM2 cache-line alignment must be a power of two");
static_assert((PHOTONS_RAM2_CACHE_LINE_BYTES %
               Payload::FIXED_STORAGE_ALIGNMENT) == 0U,
              "PHOTONS RAM2 alignment must satisfy Payload fixed storage");

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
// Laser source: Koheron DRV200-A-40 driven through the TC4427 MDM.
// Teensy owns only active-high MOD on pin 35 plus the historical monitor-PD
// ADC on pin 20. DRV200 bias current and hardware enable are local controls.

static constexpr float PHOTONS_LASER_EMIT_THRESHOLD_V = 0.75f;
static constexpr uint64_t PHOTONS_PULSE_DEFAULT_NS = 1000ULL;

// LANTERN V1.0 physical geometry. No recurring race scheduler exists.
static constexpr uint64_t PHOTONS_RACE_PULSE_NS = 200ULL;
// Minimum settling time after classification, measured in nominal CPU cycles.
// TimePop ALAP dispatch may extend this interval; no CH2 appointment is armed.
// Use a 1 ms holdoff to measure sensitivity to inter-race settling time.
static constexpr uint32_t PHOTONS_RACE_HOLDOFF_NS = 1000000U;
static constexpr uint64_t PHOTONS_RACE_CADENCE_NS = 0ULL;
static constexpr uint32_t PHOTONS_RACE_CADENCE_HZ = 0U;
static constexpr uint32_t PHOTONS_RACE_SEED_HISTORY = 8U;
static constexpr uint32_t PHOTONS_RACE_SEED_QUORUM = 3U;

struct photons_race_batch_t {
  uint64_t accepted_count = 0ULL;
  uint64_t accepted_sum_cycles = 0ULL;
  uint64_t accepted_sumsq_cycles = 0ULL;
  uint32_t accepted_min_cycles = 0U;
  uint32_t accepted_max_cycles = 0U;
  uint64_t rejected_count = 0ULL;
  uint64_t rejected_sum_cycles = 0ULL;
  uint64_t rejected_sumsq_cycles = 0ULL;
  uint32_t rejected_min_cycles = 0U;
  uint32_t rejected_max_cycles = 0U;
  uint64_t rejected_isr_delay = 0ULL;
  uint64_t rejected_excursion = 0ULL;
};

struct photons_race_runtime_t {
  bool initialized = false;
  bool active = false;
  bool primed = false;
  bool first_return_seen = false;
  uint32_t launch_dwt = 0U;
  uint32_t sequence = 0U;
  uint64_t cadence_tick_count = 0ULL; // retired wire compatibility
  uint64_t attempt_count = 0ULL;
  uint64_t completed_count = 0ULL;
  uint64_t missed_count = 0ULL;
  uint64_t skipped_not_quiet_count = 0ULL;
  uint64_t skipped_projection_count = 0ULL;
  uint64_t invalid_endpoint_count = 0ULL;
  uint64_t enqueue_failure_count = 0ULL;
  uint32_t seed_cycles[PHOTONS_RACE_SEED_HISTORY]{};
  uint32_t seed_count = 0U;
  bool reference_valid = false;
  uint32_t reference_cycles = 0U;
  uint32_t reference_gate_cycles = 0U;
  uint64_t rejected_isr_delay_count = 0ULL;
  uint64_t rejected_qtimer1_count = 0ULL;
  uint64_t rejected_ocxo1_count = 0ULL;
  uint64_t rejected_ocxo2_count = 0ULL;
  uint64_t rejected_pps_count = 0ULL;
  uint64_t rejected_continuation_count = 0ULL;
  uint64_t rejected_unknown_count = 0ULL;
  uint64_t rejected_excursion_count = 0ULL;
  uint32_t holdoff_started_dwt = 0U;
  uint32_t holdoff_cycles = 0U;
  uint64_t holdoff_edges = 0ULL;
  uint64_t holdoff_launches = 0ULL;
  uint32_t holdoff_last_cycles = 0U;
  uint32_t holdoff_min_cycles = 0U;
  uint32_t holdoff_max_cycles = 0U;
};

static photons_race_runtime_t g_photons_race{};
// Continuation alone writes this completed-race mailbox. A new launch is
// forbidden until foreground has merged it and released its generation.
static photons_race_batch_t g_photons_race_batch{};
static uint32_t g_photons_race_batch_published = 0U; // continuation writer
static uint32_t g_photons_race_batch_consumed = 0U;  // foreground writer
static photons_race_batch_t g_photons_foreground_batch{};
static_assert(__atomic_always_lock_free(sizeof(uint32_t), nullptr),
              "PHOTONS handoff words must be lock-free");

struct photons_device_snapshot_t {
  int      laser_mod_level = LOW;
  uint16_t laser_monitor_raw = 0;
  float    laser_monitor_v = 0.0f;
  bool     laser_emitting = false;

  int      photodiode_edge_level = 0;
};

// ============================================================================
// Canonical raw-lap handoff / projection / statistics state
// ============================================================================

static inline void photons_memory_barrier(void) {
  __asm__ volatile("dmb" ::: "memory");
}


// All non-ISR PHOTONS mutation belongs to one foreground execution domain.
// TimePop and the command dispatcher currently serialize that domain, but that
// scheduler property is not sufficient custody: a future local callback or
// dispatch refactor must not make two PHOTONS mutation transactions legal.
//
// The owner court therefore makes the architectural rule executable:
//   * FRAGMENT owns drain -> snapshot -> Payload render -> synchronous publish
//     -> post-publish commit as one indivisible foreground transaction;
//   * COMMAND owns every PHOTONS RPC handler from entry through returned Payload
//     construction;
//   * WAVE owns each commissioning pulse callback, including its HIGH wait;
//   * FOREGROUND_SERVICE owns completed-race consumption, histogram origin
//     inference, and the next physical launch as one transaction.
//
// ISR code never touches this owner.  Its only cross-context communication is
// through the dedicated one-writer arm scalars / generation mailboxes below.
enum class photons_foreground_owner_t : uint8_t {
  NONE = 0U,
  FRAGMENT = 2U,
  COMMAND = 3U,
  WAVE = 4U,
  FOREGROUND_SERVICE = 5U,
};

// The ownership court must be stronger than the scheduler assumption it is
// checking.  Use one non-spinning atomic claim so a future preempting foreground
// path cannot pass a check-then-store window and become a second writer.
static volatile uint32_t g_photons_foreground_owner =
    (uint32_t)photons_foreground_owner_t::NONE;

static inline void photons_foreground_owner_acquire(
    photons_foreground_owner_t owner) {
  if (owner == photons_foreground_owner_t::NONE) __builtin_trap();
  uint32_t expected = (uint32_t)photons_foreground_owner_t::NONE;
  if (!__atomic_compare_exchange_n(
          &g_photons_foreground_owner,
          &expected,
          (uint32_t)owner,
          false,
          __ATOMIC_ACQ_REL,
          __ATOMIC_ACQUIRE)) {
    __builtin_trap();
  }
  photons_memory_barrier();
}

static inline void photons_foreground_owner_release(
    photons_foreground_owner_t owner) {
  photons_memory_barrier();
  if (owner == photons_foreground_owner_t::NONE ||
      __atomic_load_n(&g_photons_foreground_owner, __ATOMIC_ACQUIRE) !=
          (uint32_t)owner) {
    __builtin_trap();
  }
  __atomic_store_n(&g_photons_foreground_owner,
                   (uint32_t)photons_foreground_owner_t::NONE,
                   __ATOMIC_RELEASE);
  photons_memory_barrier();
}

static inline void photons_foreground_owner_assert(
    photons_foreground_owner_t owner) {
  if (owner == photons_foreground_owner_t::NONE ||
      __atomic_load_n(&g_photons_foreground_owner, __ATOMIC_ACQUIRE) !=
          (uint32_t)owner) {
    __builtin_trap();
  }
}

class photons_foreground_custody_t {
 public:
  explicit photons_foreground_custody_t(photons_foreground_owner_t owner)
      : owner_(owner) {
    photons_foreground_owner_acquire(owner_);
  }

  ~photons_foreground_custody_t() {
    photons_foreground_owner_release(owner_);
  }

  photons_foreground_custody_t(const photons_foreground_custody_t&) = delete;
  photons_foreground_custody_t& operator=(
      const photons_foreground_custody_t&) = delete;

 private:
  photons_foreground_owner_t owner_;
};


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
alignas(PHOTONS_RAM2_CACHE_LINE_BYTES)
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

alignas(PHOTONS_RAM2_CACHE_LINE_BYTES)
static photons_ppb_endpoint_t
    g_photons_ppb_seconds[PHOTONS_PPB_SECOND_CAPACITY] DMAMEM = {};
alignas(PHOTONS_RAM2_CACHE_LINE_BYTES)
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


static void photons_welford_merge_batch(photons_welford_state_t& w,
                                        uint64_t n,
                                        double mean,
                                        double m2,
                                        double min_val,
                                        double max_val) {
  if (n == 0ULL) return;
  if (w.n == 0ULL) {
    w.n = n; w.mean = mean; w.m2 = m2;
    w.min_val = min_val; w.max_val = max_val;
    return;
  }
  const uint64_t old_n = w.n;
  const uint64_t total_n = old_n + n;
  const double delta = mean - w.mean;
  w.mean += delta * ((double)n / (double)total_n);
  w.m2 += m2 + delta * delta *
      ((double)old_n * (double)n / (double)total_n);
  w.n = total_n;
  if (min_val < w.min_val) w.min_val = min_val;
  if (max_val > w.max_val) w.max_val = max_val;
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
  photons_foreground_owner_assert(photons_foreground_owner_t::FRAGMENT);
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
  photons_foreground_owner_assert(photons_foreground_owner_t::FRAGMENT);
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
  photons_foreground_owner_assert(photons_foreground_owner_t::FRAGMENT);
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
    case photons_lap_science_exclusion_reason_t::ISR_DELAY:
      return "isr_delay";
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
  g_photons_lap_science_state.seed_pending_count =
      g_photons_lap_science_seed_pending.valid ? 1U : 0U;
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
    case photons_lap_science_exclusion_reason_t::ISR_DELAY:
      g_photons_lap_science_state.exclusion_reasons.isr_delay++;
      g_photons_lap_science_state.exclusion_reasons.isr_delay_this_fragment++;
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
  const uint64_t partial2 = partial + reasons.raw_cycle_excursion;
  if (UINT64_MAX - partial2 < reasons.isr_delay) __builtin_trap();
  return partial2 + reasons.isr_delay;
}


static uint32_t photons_lap_science_excluded_this_fragment_from_reasons(
    const photons_lap_science_reason_counts_snapshot_t& reasons) {
  const uint64_t total =
      (uint64_t)reasons.projection_invalid_this_fragment +
      (uint64_t)reasons.seed_disagreement_this_fragment +
      (uint64_t)reasons.raw_cycle_excursion_this_fragment +
      (uint64_t)reasons.isr_delay_this_fragment;
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
  const uint64_t pending_count = (uint64_t)science.seed_pending_count;
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


static inline uint32_t photons_priority32_guard_enter(void);
static inline void photons_priority32_guard_exit(uint32_t prior);

static void photons_batch_add_checked(uint64_t& total, uint64_t value) {
  if (value > UINT64_MAX - total) __builtin_trap();
  total += value;
}

// Exactly one unconsumed completion is possible: only foreground launches the
// next race. Sequence subtraction is intentionally modulo 2^32, including wrap.
static void photons_race_batch_consume(void) {
  const uint32_t owner =
      __atomic_load_n(&g_photons_foreground_owner, __ATOMIC_ACQUIRE);
  if (owner != (uint32_t)photons_foreground_owner_t::FOREGROUND_SERVICE &&
      owner != (uint32_t)photons_foreground_owner_t::FRAGMENT) {
    __builtin_trap();
  }
  const uint32_t published =
      __atomic_load_n(&g_photons_race_batch_published, __ATOMIC_ACQUIRE);
  const uint32_t consumed =
      __atomic_load_n(&g_photons_race_batch_consumed, __ATOMIC_RELAXED);
  if (published == consumed) return;
  if ((uint32_t)(published - consumed) != 1U) __builtin_trap();

  const auto& source = g_photons_race_batch;
  auto& total = g_photons_foreground_batch;
  if (source.accepted_count != 0ULL) {
    if (total.accepted_count == 0ULL ||
        source.accepted_min_cycles < total.accepted_min_cycles)
      total.accepted_min_cycles = source.accepted_min_cycles;
    if (source.accepted_max_cycles > total.accepted_max_cycles)
      total.accepted_max_cycles = source.accepted_max_cycles;
  }
  if (source.rejected_count != 0ULL) {
    if (total.rejected_count == 0ULL ||
        source.rejected_min_cycles < total.rejected_min_cycles)
      total.rejected_min_cycles = source.rejected_min_cycles;
    if (source.rejected_max_cycles > total.rejected_max_cycles)
      total.rejected_max_cycles = source.rejected_max_cycles;
  }
  photons_batch_add_checked(total.accepted_count, source.accepted_count);
  photons_batch_add_checked(total.accepted_sum_cycles, source.accepted_sum_cycles);
  photons_batch_add_checked(total.accepted_sumsq_cycles, source.accepted_sumsq_cycles);
  photons_batch_add_checked(total.rejected_count, source.rejected_count);
  photons_batch_add_checked(total.rejected_sum_cycles, source.rejected_sum_cycles);
  photons_batch_add_checked(total.rejected_sumsq_cycles, source.rejected_sumsq_cycles);
  photons_batch_add_checked(total.rejected_isr_delay, source.rejected_isr_delay);
  photons_batch_add_checked(total.rejected_excursion, source.rejected_excursion);
  // Release only after the last read. The producer may now reuse its mailbox.
  __atomic_store_n(&g_photons_race_batch_consumed, published, __ATOMIC_RELEASE);
}

static photons_race_batch_t photons_race_batch_take(void) {
  photons_foreground_owner_assert(photons_foreground_owner_t::FRAGMENT);
  photons_race_batch_consume();
  const photons_race_batch_t out = g_photons_foreground_batch;
  g_photons_foreground_batch = photons_race_batch_t{};
  return out;
}

static double photons_batch_m2(uint64_t n, uint64_t sum, uint64_t sumsq) {
  if (n < 2ULL) return 0.0;
  const double dsum = (double)sum;
  double m2 = (double)sumsq - (dsum * dsum / (double)n);
  if (m2 < 0.0 && m2 > -0.5) m2 = 0.0;
  if (m2 < 0.0) __builtin_trap();
  return m2;
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
  photons_foreground_owner_assert(photons_foreground_owner_t::FRAGMENT);
  photons_fragment_drain_result_t result{};
  photons_welford_reset(result.projected_flight_welford);

  g_photons_lap_science_state.candidates_this_fragment = 0U;
  g_photons_lap_science_state.accepted.count_this_fragment = 0U;
  g_photons_lap_science_state.exclusion_reasons.projection_invalid_this_fragment = 0U;
  g_photons_lap_science_state.exclusion_reasons.seed_disagreement_this_fragment = 0U;
  g_photons_lap_science_state.exclusion_reasons.raw_cycle_excursion_this_fragment = 0U;
  g_photons_lap_science_state.exclusion_reasons.isr_delay_this_fragment = 0U;

  const photons_race_batch_t race_batch = photons_race_batch_take();
  const uint64_t race_finalized =
      race_batch.accepted_count + race_batch.rejected_count;
  if (race_finalized > (uint64_t)UINT32_MAX) __builtin_trap();

  if (race_batch.accepted_count != 0ULL) {
    const uint32_t cps = interrupt_dynamic_cps();
    if (cps == 0U) __builtin_trap();
    const double n = (double)race_batch.accepted_count;
    const double mean_cycles = (double)race_batch.accepted_sum_cycles / n;
    const double m2_cycles = photons_batch_m2(
        race_batch.accepted_count,
        race_batch.accepted_sum_cycles,
        race_batch.accepted_sumsq_cycles);
    const double ns_per_cycle =
        (double)PHOTONS_NS_PER_SECOND / (double)cps;
    const double mean_ns = mean_cycles * ns_per_cycle;
    const double m2_ns = m2_cycles * ns_per_cycle * ns_per_cycle;

    photons_welford_merge_batch(
        g_accepted_raw_cycles_welford,
        race_batch.accepted_count,
        mean_cycles,
        m2_cycles,
        (double)race_batch.accepted_min_cycles,
        (double)race_batch.accepted_max_cycles);
    photons_welford_merge_batch(
        g_lap_time_welford,
        race_batch.accepted_count,
        mean_ns,
        m2_ns,
        (double)race_batch.accepted_min_cycles * ns_per_cycle,
        (double)race_batch.accepted_max_cycles * ns_per_cycle);
    photons_welford_merge_batch(
        result.projected_flight_welford,
        race_batch.accepted_count,
        mean_ns,
        m2_ns,
        (double)race_batch.accepted_min_cycles * ns_per_cycle,
        (double)race_batch.accepted_max_cycles * ns_per_cycle);

    const uint64_t accepted_ns = (uint64_t)(
        ((long double)race_batch.accepted_sum_cycles *
         (long double)PHOTONS_NS_PER_SECOND / (long double)cps) + 0.5L);
    g_total_lap_gnss_ns += accepted_ns;
    g_photons_custody_lap_count += race_batch.accepted_count;
    g_photons_custody_total_lap_gnss_ns += accepted_ns;
    g_photons_lap_science_state.accepted.count += race_batch.accepted_count;
    g_photons_lap_science_state.accepted.count_this_fragment +=
        (uint32_t)race_batch.accepted_count;
  }

  if (race_batch.rejected_count != 0ULL) {
    const double n = (double)race_batch.rejected_count;
    const double mean_cycles = (double)race_batch.rejected_sum_cycles / n;
    const double m2_cycles = photons_batch_m2(
        race_batch.rejected_count,
        race_batch.rejected_sum_cycles,
        race_batch.rejected_sumsq_cycles);
    photons_welford_merge_batch(
        g_excluded_raw_cycles_welford,
        race_batch.rejected_count,
        mean_cycles,
        m2_cycles,
        (double)race_batch.rejected_min_cycles,
        (double)race_batch.rejected_max_cycles);
    g_photons_lap_science_state.exclusion_reasons.isr_delay +=
        race_batch.rejected_isr_delay;
    g_photons_lap_science_state.exclusion_reasons.isr_delay_this_fragment +=
        (uint32_t)race_batch.rejected_isr_delay;
    g_photons_lap_science_state.exclusion_reasons.raw_cycle_excursion +=
        race_batch.rejected_excursion;
    g_photons_lap_science_state.exclusion_reasons.raw_cycle_excursion_this_fragment +=
        (uint32_t)race_batch.rejected_excursion;
  }

  g_photons_lap_science_state.candidate_count += race_finalized;
  g_photons_lap_science_state.candidates_this_fragment +=
      (uint32_t)race_finalized;
  g_raw_cycles_state.completed_lap_count += race_finalized;
  // Autonomous raw exclusions never enter projection custody.  Only accepted
  // flights are converted into projected flight-time statistics.
  g_projection_state.attempt_count += race_batch.accepted_count;
  g_projection_state.success_count += race_batch.accepted_count;
  g_photons_lap_science_state.predictor_valid = g_photons_race.reference_valid;
  g_photons_lap_science_state.predictor_cycles = g_photons_race.reference_cycles;
  g_photons_lap_science_state.gate_cycles =
      g_photons_race.reference_gate_cycles;
  g_photons_lap_science_state.seed_pending = false;
  g_photons_lap_science_state.seed_pending_count = 0U;

  result.raw_laps += (uint32_t)race_finalized;
  result.projected_laps += (uint32_t)race_batch.accepted_count;
  result.total_cycles += race_batch.accepted_sum_cycles +
                         race_batch.rejected_sum_cycles;
  if (race_batch.accepted_min_cycles != 0U) result.min_cycles = race_batch.accepted_min_cycles;
  if (race_batch.rejected_min_cycles != 0U &&
      (result.min_cycles == 0U || race_batch.rejected_min_cycles < result.min_cycles)) {
    result.min_cycles = race_batch.rejected_min_cycles;
  }
  if (race_batch.accepted_max_cycles > result.max_cycles) {
    result.max_cycles = race_batch.accepted_max_cycles;
  }
  if (race_batch.rejected_max_cycles > result.max_cycles) {
    result.max_cycles = race_batch.rejected_max_cycles;
  }

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


static float photons_adc_voltage(uint16_t raw) {
  return (raw / ADC_FS_COUNTS) * ADC_FS_VOLTS;
}

// The retired EV5491 coarse-source and active-low MOSFET gate are gone. Pin 35
// is now a single active-high DRV200 MOD command through the non-inverting
// TC4427 MDM: LOW = zero added modulation, HIGH = positive modulation.
//
// Commissioning WAVE owns pin 35 until WAVEOFF or another direct MOD command
// explicitly cancels the recurring timer. TimePop schedules pulse starts at the
// requested interval; each callback holds HIGH with a DWT loop and returns LOW.
static timepop_handle_t g_photons_wave_timer = TIMEPOP_INVALID_HANDLE;
static uint64_t g_photons_wave_interval_ns = 0ULL;
static uint64_t g_photons_wave_width_ns = 0ULL;

static void photons_wave_emit_pulse(uint64_t requested_ns);

static void photons_wave_tick(
    timepop_ctx_t* ctx,
    timepop_diag_t* /*diag*/,
    void* /*user_data*/) {
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::WAVE);
  if (!ctx || g_photons_wave_timer == TIMEPOP_INVALID_HANDLE ||
      ctx->handle != g_photons_wave_timer) {
    __builtin_trap();
  }

  photons_wave_emit_pulse(g_photons_wave_width_ns);
}

static void photons_wave_cancel(void) {
  if (g_photons_wave_timer == TIMEPOP_INVALID_HANDLE) return;
  const timepop_handle_t handle = g_photons_wave_timer;
  if (!timepop_cancel(handle)) __builtin_trap();
  g_photons_wave_timer = TIMEPOP_INVALID_HANDLE;
  g_photons_wave_interval_ns = 0ULL;
  g_photons_wave_width_ns = 0ULL;
}

static void photons_laser_mod_idle(void) {
  photons_wave_cancel();
  digitalWriteFast(LASER_MOD_PIN, LOW);
}

static inline uint32_t photons_priority32_guard_enter(void) {
  uint32_t prior = 0U;
  __asm__ volatile ("mrs %0, basepri" : "=r" (prior) :: "memory");
  const uint32_t mask_priority32 = 32U;
  if (prior == 0U || prior > mask_priority32) {
    __asm__ volatile ("msr basepri, %0" :: "r" (mask_priority32) : "memory");
  }
  photons_memory_barrier();
  return prior;
}

static inline void photons_priority32_guard_exit(uint32_t prior) {
  photons_memory_barrier();
  __asm__ volatile ("msr basepri, %0" :: "r" (prior) : "memory");
}

// Always-on raw-cycle histogram. Its lifetime is the firmware boot, independent
// of campaigns and STATS_RESET. Continuation is its ONLY writer, including the
// origin commit and seed replay. Foreground receives immutable seeds, returns
// the inferred origin, and reads separately published report snapshots.
static constexpr uint32_t PHOTONS_HISTOGRAM_BINS = 64U;
static constexpr uint32_t PHOTONS_HISTOGRAM_SEEDS = 65U;
struct photons_histogram_population_t {
  uint64_t bins[PHOTONS_HISTOGRAM_BINS]{};
  uint64_t underflow = 0ULL;
  uint64_t overflow = 0ULL;
  uint64_t acquisition_unbinned = 0ULL;
};
struct photons_histogram_t {
  uint32_t seed_count = 0U;
  uint32_t seeds[PHOTONS_HISTOGRAM_SEEDS]{};
  uint32_t origin_cycles = 0U; // zero denotes acquisition; lawful origin is positive
  uint64_t warmup_returns = 0ULL;
  photons_histogram_population_t unattributed{};
  photons_histogram_population_t delayed{};
};
static photons_histogram_t g_photons_histogram{};

// One-shot SPSC exchange. Once published, seeds are immutable for this boot.
static uint32_t g_photons_histogram_seeds_published = 0U; // continuation writer
static uint32_t g_photons_histogram_origin_request = 0U; // foreground writer

struct photons_histogram_snapshot_t {
  uint32_t seed_count = 0U;
  uint32_t origin_cycles = 0U;
  uint32_t sequence = 0U;
  uint32_t dwt = 0U;
  uint64_t warmup_returns = 0ULL;
  photons_histogram_population_t unattributed{};
  photons_histogram_population_t delayed{};
};
struct alignas(PHOTONS_RAM2_CACHE_LINE_BYTES) photons_histogram_slot_t {
  photons_histogram_snapshot_t value{};
};
static photons_histogram_slot_t g_photons_histogram_slots[2] DMAMEM;
static_assert(sizeof(g_photons_histogram_slots) == 2240U,
              "Review the histogram snapshot RAM2 budget after layout changes");
static uint32_t g_photons_histogram_snapshot_published = 0U; // producer writer
static uint32_t g_photons_histogram_snapshot_consumed = 0U;  // consumer writer
static uint32_t g_photons_histogram_snapshot_requested = 1U; // consumer writer

// Dedicated byte stores belong only to COMMAND custody. Payload objects remain
// ordinary scoped objects; no object storage is overlaid or reinterpreted.
struct alignas(PHOTONS_RAM2_CACHE_LINE_BYTES) photons_histogram_report_store_t {
  alignas(PHOTONS_RAM2_CACHE_LINE_BYTES) uint8_t root[6144];
  alignas(PHOTONS_RAM2_CACHE_LINE_BYTES) uint8_t population[2048];
  alignas(PHOTONS_RAM2_CACHE_LINE_BYTES) uint8_t bins[2304];
};
static photons_histogram_report_store_t g_photons_histogram_report_store DMAMEM;
static_assert(sizeof(photons_histogram_report_store_t) == 10496U,
              "Review the histogram report RAM2 budget after layout changes");

static void photons_histogram_initialize_handoffs(void) {
  // RAM2 is NOLOAD. Construct lawful empty snapshot values explicitly before
  // registering the detector callback; only continuation writes slots afterward.
  g_photons_histogram_slots[0].value = photons_histogram_snapshot_t{};
  g_photons_histogram_slots[1].value = photons_histogram_snapshot_t{};
}

static void photons_histogram_publish_snapshot(void) {
  const uint32_t published =
      __atomic_load_n(&g_photons_histogram_snapshot_published, __ATOMIC_RELAXED);
  const uint32_t requested =
      __atomic_load_n(&g_photons_histogram_snapshot_requested, __ATOMIC_ACQUIRE);
  if (requested == published) return;
  if ((uint32_t)(requested - published) != 1U) __builtin_trap();
  if (__atomic_load_n(&g_photons_histogram_snapshot_consumed, __ATOMIC_ACQUIRE) !=
      published) return; // previous publication is still held by foreground

  const uint32_t next = published + 1U;
  auto& out = g_photons_histogram_slots[next & 1U].value;
  out.seed_count = g_photons_histogram.seed_count;
  out.origin_cycles = g_photons_histogram.origin_cycles;
  out.sequence = next;
  out.dwt = ARM_DWT_CYCCNT;
  out.warmup_returns = g_photons_histogram.warmup_returns;
  out.unattributed = g_photons_histogram.unattributed;
  out.delayed = g_photons_histogram.delayed;
  __atomic_store_n(&g_photons_histogram_snapshot_published, next, __ATOMIC_RELEASE);
}

static const photons_histogram_snapshot_t& photons_histogram_snapshot_acquire(void) {
  const uint32_t owner =
      __atomic_load_n(&g_photons_foreground_owner, __ATOMIC_ACQUIRE);
  if (owner != (uint32_t)photons_foreground_owner_t::FRAGMENT &&
      owner != (uint32_t)photons_foreground_owner_t::COMMAND) {
    __builtin_trap();
  }
  const uint32_t published =
      __atomic_load_n(&g_photons_histogram_snapshot_published, __ATOMIC_ACQUIRE);
  const uint32_t consumed =
      __atomic_load_n(&g_photons_histogram_snapshot_consumed, __ATOMIC_RELAXED);
  if ((uint32_t)(published - consumed) > 1U) __builtin_trap();
  // Release the PREVIOUS slot. Producer may fill the other slot, but cannot
  // reuse this one until a later serialized foreground transaction advances
  // consumed again. Never call acquire twice while retaining an earlier view.
  __atomic_store_n(&g_photons_histogram_snapshot_consumed, published, __ATOMIC_RELEASE);
  __atomic_store_n(&g_photons_histogram_snapshot_requested, published + 1U,
                   __ATOMIC_RELEASE);
  return g_photons_histogram_slots[published & 1U].value;
}

static void photons_histogram_increment(uint64_t& count) {
  if (++count == 0ULL) __builtin_trap();
}

static void photons_histogram_bin(photons_histogram_population_t& population,
                                  uint32_t raw_cycles) {
  const uint32_t origin = g_photons_histogram.origin_cycles;
  if (raw_cycles < origin) photons_histogram_increment(population.underflow);
  else if (raw_cycles - origin >= PHOTONS_HISTOGRAM_BINS)
    photons_histogram_increment(population.overflow);
  else photons_histogram_increment(population.bins[raw_cycles - origin]);
}

static void photons_histogram_observe(uint32_t raw_cycles, bool delayed,
                                      bool warmup) {
  if (warmup) {
    photons_histogram_increment(g_photons_histogram.warmup_returns);
    return;
  }
  auto& population = delayed ? g_photons_histogram.delayed
                             : g_photons_histogram.unattributed;
  if (g_photons_histogram.origin_cycles != 0U) {
    photons_histogram_bin(population, raw_cycles);
  } else if (!delayed &&
             g_photons_histogram.seed_count < PHOTONS_HISTOGRAM_SEEDS) {
    g_photons_histogram.seeds[g_photons_histogram.seed_count++] = raw_cycles;
    if (g_photons_histogram.seed_count == PHOTONS_HISTOGRAM_SEEDS)
      __atomic_store_n(&g_photons_histogram_seeds_published, 1U, __ATOMIC_RELEASE);
  } else {
    photons_histogram_increment(population.acquisition_unbinned);
  }
}

// One-time median inference happens in foreground, never in the edge callback.
// Acquire the completed immutable seed buffer; return only the inferred origin.
// Any returns while inference runs remain counted as acquisition_unbinned.
static void photons_histogram_acquire(void) {
  photons_foreground_owner_assert(photons_foreground_owner_t::FOREGROUND_SERVICE);
  if (__atomic_load_n(&g_photons_histogram_origin_request, __ATOMIC_RELAXED) != 0U)
    return;
  if (__atomic_load_n(&g_photons_histogram_seeds_published, __ATOMIC_ACQUIRE) == 0U)
    return;
  uint32_t ordered[PHOTONS_HISTOGRAM_SEEDS];
  for (uint32_t i = 0U; i < PHOTONS_HISTOGRAM_SEEDS; ++i)
    ordered[i] = g_photons_histogram.seeds[i];
  for (uint32_t i = 1U; i < PHOTONS_HISTOGRAM_SEEDS; ++i) {
    const uint32_t value = ordered[i];
    uint32_t j = i;
    while (j != 0U && ordered[j - 1U] > value) {
      ordered[j] = ordered[j - 1U]; --j;
    }
    ordered[j] = value;
  }
  const uint32_t midpoint = ordered[PHOTONS_HISTOGRAM_SEEDS / 2U];
  if (midpoint <= 32U || midpoint > UINT32_MAX - 31U) __builtin_trap();
  __atomic_store_n(&g_photons_histogram_origin_request, midpoint - 32U,
                   __ATOMIC_RELEASE);
}

// Called only by continuation before observing its next completed race.
static void photons_histogram_commit_origin(void) {
  if (g_photons_histogram.origin_cycles != 0U) return;
  const uint32_t origin =
      __atomic_load_n(&g_photons_histogram_origin_request, __ATOMIC_ACQUIRE);
  if (origin == 0U) return;
  if (g_photons_histogram.seed_count != PHOTONS_HISTOGRAM_SEEDS ||
      origin > UINT32_MAX - (PHOTONS_HISTOGRAM_BINS - 1U)) __builtin_trap();
  g_photons_histogram.origin_cycles = origin;
  for (uint32_t i = 0U; i < PHOTONS_HISTOGRAM_SEEDS; ++i)
    photons_histogram_bin(g_photons_histogram.unattributed,
                          g_photons_histogram.seeds[i]);
}

static void photons_histogram_payload_population(
    Payload& parent, const char* name,
    const photons_histogram_population_t& population) {
  photons_foreground_owner_assert(photons_foreground_owner_t::COMMAND);
  auto& store = g_photons_histogram_report_store;
  Payload p(Payload::StorageMode::FIXED, store.population, sizeof(store.population));
  p.add("underflow", population.underflow);
  p.add("overflow", population.overflow);
  p.add("acquisition_unbinned", population.acquisition_unbinned);
  Payload bins(Payload::StorageMode::FIXED, store.bins, sizeof(store.bins));
  for (uint32_t i = 0U; i < PHOTONS_HISTOGRAM_BINS; ++i) {
    char key[8];
    snprintf(key, sizeof(key), "b%02u", (unsigned)i);
    bins.add(key, population.bins[i]);
  }
  p.add_object("bins", bins);
  parent.add_object(name, p);
}

static FLASHMEM Payload cmd_report_histogram(const Payload& /*args*/) {
  const photons_foreground_custody_t custody(photons_foreground_owner_t::COMMAND);
  const auto& h = photons_histogram_snapshot_acquire();
  auto& store = g_photons_histogram_report_store;
  Payload p(Payload::StorageMode::FIXED, store.root, sizeof(store.root));
  p.add("schema", "PHOTONS_HISTOGRAM_V1");
  p.add("scope", "FIRMWARE_BOOT");
  p.add("snapshot_policy", "LATEST_COMPLETED_SPSC_PUBLICATION");
  p.add("snapshot_sequence", h.sequence);
  p.add("snapshot_dwt", h.dwt);
  p.add("state", h.origin_cycles == 0U ? "ACQUIRING" : "ACCUMULATING");
  p.add("bin_width_cycles", 1U);
  p.add("bin_count", PHOTONS_HISTOGRAM_BINS);
  p.add("seed_target", PHOTONS_HISTOGRAM_SEEDS);
  p.add("seed_count", h.seed_count);
  p.add("warmup_returns", h.warmup_returns);
  if (h.origin_cycles != 0U) {
    p.add("origin_cycles", h.origin_cycles);
    p.add("midpoint_cycles", h.origin_cycles + 32U);
  }
  photons_histogram_payload_population(p, "unattributed", h.unattributed);
  photons_histogram_payload_population(p, "delayed", h.delayed);
  // Copy to an owning response before COMMAND custody releases the workspace.
  // Explicit copy construction prevents NRVO from returning a borrowed store.
  return Payload(p);
}

static photons_race_runtime_t photons_race_runtime_snapshot(void) {
  const uint32_t prior = photons_priority32_guard_enter();
  const photons_race_runtime_t out = g_photons_race;
  photons_priority32_guard_exit(prior);
  return out;
}

static uint32_t photons_race_pending_relaunch(
    const photons_race_runtime_t& race) {
  const uint32_t pending = race.active && !race.primed ? 1U : 0U;
  if (race.attempt_count > race.completed_count ||
      race.completed_count - race.attempt_count != pending ||
      race.holdoff_launches != race.attempt_count) {
    __builtin_trap();
  }
  return pending;
}

static void photons_race_snapshot_relaunch_accounting(
    photons_fragment_snapshot_t& fragment,
    const photons_race_runtime_t& race,
    uint64_t previous_completed, uint64_t previous_attempts) {
  if (previous_attempts > previous_completed ||
      previous_completed - previous_attempts > 1ULL ||
      previous_completed > race.completed_count ||
      previous_attempts > race.attempt_count) {
    __builtin_trap();
  }
  fragment.race_pending_relaunch_count = photons_race_pending_relaunch(race);
  fragment.race_pending_relaunch_count_previous =
      (uint32_t)(previous_completed - previous_attempts);
}

static void photons_race_batch_note(bool accepted, uint32_t raw_cycles) {
  photons_race_batch_t& b = g_photons_race_batch;
  uint64_t& count = accepted ? b.accepted_count : b.rejected_count;
  uint64_t& sum = accepted ? b.accepted_sum_cycles : b.rejected_sum_cycles;
  uint64_t& sumsq = accepted ? b.accepted_sumsq_cycles : b.rejected_sumsq_cycles;
  uint32_t& minv = accepted ? b.accepted_min_cycles : b.rejected_min_cycles;
  uint32_t& maxv = accepted ? b.accepted_max_cycles : b.rejected_max_cycles;
  if (count == 0ULL || raw_cycles < minv) minv = raw_cycles;
  photons_batch_add_checked(count, 1ULL);
  photons_batch_add_checked(sum, raw_cycles);
  photons_batch_add_checked(sumsq, (uint64_t)raw_cycles * (uint64_t)raw_cycles);
  if (raw_cycles > maxv) maxv = raw_cycles;
}

static void photons_race_note_delay(interrupt_delay_cause_t cause) {
  g_photons_race.rejected_isr_delay_count++;
  g_photons_race_batch.rejected_isr_delay++;
  switch (cause) {
    case interrupt_delay_cause_t::VCLOCK_TIMEPOP: g_photons_race.rejected_qtimer1_count++; break;
    case interrupt_delay_cause_t::OCXO1: g_photons_race.rejected_ocxo1_count++; break;
    case interrupt_delay_cause_t::OCXO2: g_photons_race.rejected_ocxo2_count++; break;
    case interrupt_delay_cause_t::PPS: g_photons_race.rejected_pps_count++; break;
    case interrupt_delay_cause_t::CONTINUATION: g_photons_race.rejected_continuation_count++; break;
    default: g_photons_race.rejected_unknown_count++; break;
  }
}

static bool photons_race_try_lock_reference(void) {
  const uint32_t count = g_photons_race.seed_count;
  if (count < PHOTONS_RACE_SEED_QUORUM) return false;
  uint32_t ordered[PHOTONS_RACE_SEED_HISTORY]{};
  for (uint32_t i = 0U; i < count; ++i) ordered[i] = g_photons_race.seed_cycles[i];
  for (uint32_t i = 1U; i < count; ++i) {
    const uint32_t value = ordered[i];
    uint32_t j = i;
    while (j > 0U && ordered[j - 1U] > value) {
      ordered[j] = ordered[j - 1U]; --j;
    }
    ordered[j] = value;
  }
  for (uint32_t i = 0U; i + PHOTONS_RACE_SEED_QUORUM <= count; ++i) {
    const uint32_t low = ordered[i];
    const uint32_t high = ordered[i + PHOTONS_RACE_SEED_QUORUM - 1U];
    const uint32_t gate = photons_lap_science_gate_cycles(low);
    if ((uint64_t)(high - low) > (uint64_t)gate) continue;
    g_photons_race.reference_cycles = ordered[i + 1U];
    g_photons_race.reference_gate_cycles =
        photons_lap_science_gate_cycles(g_photons_race.reference_cycles);
    g_photons_race.reference_valid = true;
    return true;
  }
  return false;
}

static void photons_race_seed_observe(uint32_t raw_cycles) {
  if (g_photons_race.seed_count < PHOTONS_RACE_SEED_HISTORY) {
    g_photons_race.seed_cycles[g_photons_race.seed_count++] = raw_cycles;
  } else {
    for (uint32_t i = 1U; i < PHOTONS_RACE_SEED_HISTORY; ++i) {
      g_photons_race.seed_cycles[i - 1U] = g_photons_race.seed_cycles[i];
    }
    g_photons_race.seed_cycles[PHOTONS_RACE_SEED_HISTORY - 1U] = raw_cycles;
  }
  (void)photons_race_try_lock_reference();
}

static uint32_t photons_race_launch_200ns(void) {
  if (digitalRead(LASER_MOD_PIN) != LOW) __builtin_trap();
  const uint32_t cps = F_CPU_ACTUAL;
  if (cps == 0U) __builtin_trap();
  const uint32_t width_cycles = (uint32_t)(
      ((uint64_t)cps * PHOTONS_RACE_PULSE_NS + 500000000ULL) / 1000000000ULL);
  digitalWriteFast(LASER_MOD_PIN, HIGH);
  const uint32_t launch_dwt = ARM_DWT_CYCCNT;
  while ((uint32_t)(ARM_DWT_CYCCNT - launch_dwt) < width_cycles) {}
  digitalWriteFast(LASER_MOD_PIN, LOW);
  return launch_dwt;
}

static void photons_race_observe_edge(
    const interrupt_photodiode_edge_t& edge) {
  if (!g_photons_race.active) return;
  if (!g_photons_race.primed) {
    // No pulse is in flight: retain these edges as holdoff diagnostics, not laps.
    g_photons_race.holdoff_edges++;
    return;
  }
  const uint32_t published =
      __atomic_load_n(&g_photons_race_batch_published, __ATOMIC_RELAXED);
  if (__atomic_load_n(&g_photons_race_batch_consumed, __ATOMIC_ACQUIRE) !=
      published) __builtin_trap();
  // Producer alone clears its slot, after foreground has released it. Warmup
  // and reference-acquisition returns also publish a (possibly empty) batch.
  g_photons_race_batch = photons_race_batch_t{};
  g_photons_race.primed = false;
  const uint32_t raw_cycles = edge.dwt_at_edge - g_photons_race.launch_dwt;
  g_photons_race.completed_count++;
  photons_histogram_commit_origin();
  photons_histogram_observe(raw_cycles,
      edge.interrupt_delay.valid && edge.interrupt_delay.delayed,
      !g_photons_race.first_return_seen);

  if (!g_photons_race.first_return_seen) {
    g_photons_race.first_return_seen = true;
  } else if (edge.interrupt_delay.valid && edge.interrupt_delay.delayed) {
    photons_race_batch_note(false, raw_cycles);
    photons_race_note_delay(edge.interrupt_delay.delayed_by);
  } else if (!g_photons_race.reference_valid) {
    photons_race_seed_observe(raw_cycles);
    if (g_photons_race.reference_valid) {
      for (uint32_t i = 0U; i < g_photons_race.seed_count; ++i) {
        const uint32_t seed = g_photons_race.seed_cycles[i];
        if (photons_lap_science_abs_delta(seed, g_photons_race.reference_cycles) <=
            (uint64_t)g_photons_race.reference_gate_cycles) {
          photons_race_batch_note(true, seed);
        }
      }
    }
  } else {
    const bool accepted =
        photons_lap_science_abs_delta(raw_cycles, g_photons_race.reference_cycles) <=
        (uint64_t)g_photons_race.reference_gate_cycles;
    photons_race_batch_note(accepted, raw_cycles);
    if (!accepted) {
      g_photons_race.rejected_excursion_count++;
      g_photons_race_batch.rejected_excursion++;
    }
  }

  photons_histogram_publish_snapshot();
  __atomic_store_n(&g_photons_race_batch_published, published + 1U,
                   __ATOMIC_RELEASE);
  // End this race. Only foreground TimePop dispatch may launch its successor,
  // after consuming the immutable completed batch above.
  g_photons_race.holdoff_started_dwt = ARM_DWT_CYCCNT;
}

static void photons_race_relaunch(void) {
  photons_foreground_owner_assert(photons_foreground_owner_t::FOREGROUND_SERVICE);
  if (__atomic_load_n(&g_photons_race_batch_published, __ATOMIC_ACQUIRE) !=
      __atomic_load_n(&g_photons_race_batch_consumed, __ATOMIC_RELAXED))
    __builtin_trap();
  const uint32_t prior = photons_priority32_guard_enter();
  if (!g_photons_race.active || g_photons_race.primed ||
      (uint32_t)(ARM_DWT_CYCCNT - g_photons_race.holdoff_started_dwt) <
          g_photons_race.holdoff_cycles) {
    __builtin_trap();
  }
  g_photons_race.sequence++;
  if (g_photons_race.sequence == 0U) g_photons_race.sequence++;
  // Capture the physical launch here; readiness time is not a launch timestamp.
  g_photons_race.launch_dwt = photons_race_launch_200ns();
  const uint32_t elapsed =
      g_photons_race.launch_dwt - g_photons_race.holdoff_started_dwt;
  g_photons_race.attempt_count++;
  g_photons_race.primed = true;
  photons_priority32_guard_exit(prior);
  // Foreground owns these diagnostics. A subsequent return may preempt them,
  // but the local elapsed value already belongs to this completed holdoff.
  g_photons_race.holdoff_last_cycles = elapsed;
  if (g_photons_race.holdoff_launches == 0ULL ||
      elapsed < g_photons_race.holdoff_min_cycles) {
    g_photons_race.holdoff_min_cycles = elapsed;
  }
  if (elapsed > g_photons_race.holdoff_max_cycles) {
    g_photons_race.holdoff_max_cycles = elapsed;
  }
  g_photons_race.holdoff_launches++;
}

static bool photons_relaunch_ready(void* /*user_data*/) {
  // TimePop polls this in foreground, including while a photon is in flight.
  // Do not mask Priority-48 detector capture merely to ask whether work is ready.
  // Refresh the ISR-owned fields before reading them. If continuation clears
  // primed during this check, it finishes writing holdoff_started_dwt before
  // foreground resumes. Seeing the old primed=true only defers one readiness
  // check. Once primed=false is observed, neither it nor the holdoff origin can
  // change again until this serialized foreground domain launches the next race.
  photons_memory_barrier();
  if (!g_photons_race.active || g_photons_race.primed) return false;
  // Read the holdoff origin only AFTER observing the completed-race state.
  photons_memory_barrier();
  return (uint32_t)(ARM_DWT_CYCCNT - g_photons_race.holdoff_started_dwt) >=
      g_photons_race.holdoff_cycles;
}

static void photons_foreground_service(void* /*user_data*/) {
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::FOREGROUND_SERVICE);
  if (!photons_relaunch_ready(nullptr)) __builtin_trap();
  photons_histogram_acquire();
  // Check completion BEFORE consumption: a return can preempt this service.
  // Once ready is true, no new race can complete until this foreground domain
  // launches it, so the mailbox cannot change underneath this transaction.
  photons_race_batch_consume();
  // TimePop invokes this registered service after scheduled/deferred work.
  // Consume and launch under the same foreground custody, with no per-race
  // scheduler mutation and no priority-16 mask. CLOCKS may preempt the launch.
  photons_race_relaunch();
}

static void photons_race_prepare(void) {
  if (g_photons_race.active ||
      __atomic_load_n(&g_photons_race_batch_published, __ATOMIC_ACQUIRE) !=
          __atomic_load_n(&g_photons_race_batch_consumed, __ATOMIC_RELAXED)) {
    __builtin_trap();
  }
  const uint32_t preserved_sequence = g_photons_race.sequence;
  g_photons_race = photons_race_runtime_t{};
  g_photons_race.sequence = preserved_sequence;
  g_photons_race.initialized = true;
  const uint32_t cps = F_CPU_ACTUAL;
  if (cps == 0U) __builtin_trap();
  g_photons_race.holdoff_cycles = (uint32_t)(
      ((uint64_t)cps * PHOTONS_RACE_HOLDOFF_NS + 999999999ULL) /
      1000000000ULL);
  // No foreground reset of producer storage or publication generations.
  g_photons_foreground_batch = photons_race_batch_t{};
}

static void photons_race_start_autonomous(void) {
  interrupt_photodiode_diag_t interrupt_diag{};
  if (!g_photons_race.initialized ||
      !interrupt_photodiode_snapshot(&interrupt_diag) ||
      !interrupt_diag.active) {
    __builtin_trap();
  }
  if (g_photons_race.active) return;
  photons_wave_cancel();
  if (digitalRead(LASER_MOD_PIN) != LOW) __builtin_trap();
  g_photons_race.active = true;
  g_photons_race.primed = true;
  g_photons_race.first_return_seen = false;
  g_photons_race.launch_dwt = photons_race_launch_200ns();
}


static void photons_laser_initialize_hardware(void) {
  // Runtime PHOTONS custody is active-high: preload LOW before enabling output
  // drive, then prove the modulation line is idle. Earliest boot custody in
  // process_interrupt.cpp establishes the same LOW/no-modulation invariant.
  photons_wave_cancel();
  digitalWrite(LASER_MOD_PIN, LOW);
  pinMode(LASER_MOD_PIN, OUTPUT);
  if (digitalRead(LASER_MOD_PIN) != LOW) __builtin_trap();

  pinMode(LASER_MONITOR_PIN, INPUT);
  analogReadResolution(12);
}

static photons_device_snapshot_t photons_device_snapshot(void) {
  photons_device_snapshot_t out{};

  out.laser_mod_level = digitalRead(LASER_MOD_PIN);
  out.laser_monitor_raw = analogRead(LASER_MONITOR_PIN);
  out.laser_monitor_v = photons_adc_voltage(out.laser_monitor_raw);
  out.laser_emitting =
      out.laser_monitor_v > PHOTONS_LASER_EMIT_THRESHOLD_V;

  // Ambient pin level is diagnostic only. Timing evidence comes exclusively
  // from process_interrupt's PHOTODIODE edge capture.
  out.photodiode_edge_level = digitalRead(PHOTODIODE_EDGE_PIN);

  return out;
}

// -----------------------------------------------------------------------------
// ISR-authored live state
// -----------------------------------------------------------------------------
//
// Single writer: PHOTODIODE callback. Foreground never resets or edits this
// state after the subscription is live; recovery establishes new ancestry by
// rebasing foreground origins instead of rewriting ISR testimony. Foreground
// readers use generation as a tiny seqlock. No interrupt masking is required, so
// PHOTONS reporting/publication never delays sovereign CLOCKS IRQs.
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
// Large foreground-only canonical values live in RAM2, not on MSP/DTCM stack.
// A generation protects the completed publication value so a future scheduling
// change cannot turn REPORT into a torn reader. process_photons_init() explicitly
// establishes every DMAMEM object because RAM2 is NOLOAD.
static volatile uint32_t g_last_fragment_generation = 0U;
alignas(PHOTONS_RAM2_CACHE_LINE_BYTES)
static photons_fragment_snapshot_t g_last_photons_fragment DMAMEM = {};
alignas(PHOTONS_RAM2_CACHE_LINE_BYTES)
static photons_fragment_snapshot_t g_photons_fragment_build DMAMEM = {};
alignas(PHOTONS_RAM2_CACHE_LINE_BYTES)
static photons_fragment_snapshot_t g_photons_report_snapshot DMAMEM = {};

// PHOTONS_FRAGMENT publication is one foreground transaction.  The producer
// first freezes every live/runtime dependency into a complete value snapshot;
// Payload construction and publish() then consume only that immutable local
// value.  Unlike CLOCKS, PHOTONS has no retry lifetime or asynchronous consumer
// requiring a publication queue between capture and serialization.

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
static uint64_t g_last_fragment_race_rejected_isr_delay_count = 0ULL;
static uint64_t g_last_fragment_race_rejected_excursion_count = 0ULL;

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

// -----------------------------------------------------------------------------
// One-shot physical pulse testimony
// -----------------------------------------------------------------------------
//
// Foreground owns launch authorship. process_interrupt owns the pin-34
// first-instruction DWT coordinate. PHOTONS admits only the first comparator
// RISING edge after a PULSE arm. The ISR stores only scalar edge facts; manual
// reports preserve those raw coordinates without late GNSS reprojection.
// Each new PULSE replaces the prior one-shot report state.
//
// Split seconds from fractional cycles so every positive uint64_t ns request
// is representable, even when its total cycle count would exceed uint64_t.
struct photons_pulse_width_t {
  uint64_t whole_seconds = 0ULL;
  uint32_t tail_cycles = 0U;
};

static photons_pulse_width_t photons_pulse_width(
    uint64_t requested_ns, uint32_t dwt_cycles_per_second) {
  photons_pulse_width_t width{};
  width.whole_seconds = requested_ns / PHOTONS_NS_PER_SECOND;
  const uint64_t remainder_ns = requested_ns % PHOTONS_NS_PER_SECOND;
  // Round upward to the next DWT cycle. This product cannot overflow uint64_t:
  // remainder_ns < 1e9 and the cycle rate is uint32_t.
  width.tail_cycles = (uint32_t)(
      (remainder_ns * (uint64_t)dwt_cycles_per_second +
       PHOTONS_NS_PER_SECOND - 1ULL) / PHOTONS_NS_PER_SECOND);
  if (width.tail_cycles == dwt_cycles_per_second) {
    width.whole_seconds++;
    width.tail_cycles = 0U;
  }
  return width;
}

// Long requests count successive unsigned DWT deltas across counter wraps.
// Interrupt time counts toward the wait; an interruption spanning a complete
// 32-bit DWT revolution is inherently unobservable and extends the pulse.
// This intentionally blocks foreground dispatch, without yield or IRQ masking.
static void photons_pulse_wait_long(
    const photons_pulse_width_t& width,
    uint32_t dwt_cycles_per_second,
    uint32_t previous_dwt) {
  uint64_t seconds_left = width.whole_seconds;
  uint64_t fractional_cycles = 0ULL;
  do {
    const uint32_t now = ARM_DWT_CYCCNT;
    fractional_cycles += (uint32_t)(now - previous_dwt);
    previous_dwt = now;
    if (fractional_cycles >= dwt_cycles_per_second) {
      const uint64_t elapsed_seconds =
          fractional_cycles / dwt_cycles_per_second;
      if (elapsed_seconds > seconds_left) return;
      seconds_left -= elapsed_seconds;
      fractional_cycles %= dwt_cycles_per_second;
    }
  } while (seconds_left != 0ULL || fractional_cycles < width.tail_cycles);
}

// WAVE uses the same approximate width conversion/wait as manual PULSE, but
// does not arm a receive record or modify one-shot testimony. No TimePop call,
// yield, Payload construction, or interrupt masking occurs while MOD is HIGH.
static void photons_wave_emit_pulse(uint64_t requested_ns) {
  const uint32_t dwt_cycles_per_second = F_CPU_ACTUAL;
  if (requested_ns == 0ULL || dwt_cycles_per_second == 0U) __builtin_trap();
  const photons_pulse_width_t width =
      photons_pulse_width(requested_ns, dwt_cycles_per_second);

  digitalWriteFast(LASER_MOD_PIN, HIGH);
  const uint32_t high_start = ARM_DWT_CYCCNT;
  if (width.whole_seconds == 0ULL) {
    while ((uint32_t)(ARM_DWT_CYCCNT - high_start) < width.tail_cycles) {
    }
  } else {
    photons_pulse_wait_long(width, dwt_cycles_per_second, high_start);
  }
  digitalWriteFast(LASER_MOD_PIN, LOW);
  if (digitalRead(LASER_MOD_PIN) != LOW) __builtin_trap();
}

struct photons_pulse_launch_state_t {
  bool     valid = false;
  uint32_t sequence = 0U;
  uint64_t requested_ns = 0ULL;
  photons_pulse_width_t target{};
  uint32_t dwt_cycles_per_second = 0U;
  uint32_t start_dwt = 0U;
  uint32_t end_dwt = 0U;
  // Raw write-bracketing interval, modulo 2^32; not a long-duration clock.
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
// Foreground is the sole writer of the pulse arm/launch state. The detector
// callback is the sole writer of the receive mailbox. Stale receive testimony
// remains immutable until a later ISR observation replaces it and is admitted
// only by exact pulse_sequence identity.
static photons_pulse_receive_state_t g_last_pulse_receive{};
static volatile uint32_t g_pulse_armed_sequence = 0U;
static uint32_t g_pulse_sequence = 0U;

static void photons_pulse_receive_isr_publish(
    const photons_pulse_receive_value_t& value) {
  g_last_pulse_receive.generation++;
  photons_memory_barrier();
  g_last_pulse_receive.value = value;
  photons_memory_barrier();
  g_last_pulse_receive.generation++;
}

static bool photons_pulse_receive_snapshot(
    photons_pulse_receive_value_t* out) {
  if (!out) return false;

  for (;;) {
    const uint32_t before = g_last_pulse_receive.generation;
    if (before & 1U) continue;

    photons_memory_barrier();
    const photons_pulse_receive_value_t snapshot =
        g_last_pulse_receive.value;
    photons_memory_barrier();

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

  // Foreground owns the arm scalar. First-edge-wins is an ISR-local mailbox
  // rule, so the callback never writes foreground state.
  if (g_last_pulse_receive.value.seen &&
      g_last_pulse_receive.value.pulse_sequence == pulse_sequence) {
    return;
  }

  photons_pulse_receive_value_t value{};
  value.seen = true;
  value.pulse_sequence = pulse_sequence;
  value.edge_sequence = edge.sequence;
  value.pps_sequence = edge.pps_sequence;
  value.finish_dwt = edge.dwt_at_edge;
  photons_pulse_receive_isr_publish(value);
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
  photons_memory_barrier();

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

  photons_memory_barrier();
  g_photons_live.generation++;  // commit: even generation
}

// ============================================================================
// Coherent toy snapshots
// ============================================================================

static bool photons_toy_capture_snapshot(photons_toy_capture_t* out) {
  if (!out) return false;

  for (;;) {
    const uint32_t before = g_photons_live.generation;
    if (before & 1U) continue;

    photons_memory_barrier();
    const photons_toy_capture_t snapshot = g_photons_live.capture;
    photons_memory_barrier();

    const uint32_t after = g_photons_live.generation;
    if (before == after && !(after & 1U)) {
      *out = snapshot;
      return true;
    }
  }
}

static void photons_last_fragment_store(
    const photons_toy_fragment_t& toy,
    const photons_fragment_snapshot_t& fragment) {
  // Foreground is the sole writer. Generation still makes the completed value a
  // proper publication boundary if command/report scheduling is ever refactored.
  g_last_fragment_generation++;
  photons_memory_barrier();
  g_last_fragment = toy;
  g_last_photons_fragment = fragment;
  photons_memory_barrier();
  g_last_fragment_generation++;
}


static void photons_last_fragment_reset(void) {
  g_last_fragment_generation++;
  photons_memory_barrier();
  g_last_fragment = photons_toy_fragment_t{};
  g_last_photons_fragment = photons_fragment_snapshot_t{};
  photons_memory_barrier();
  g_last_fragment_generation++;
}


static bool photons_fragment_snapshot(photons_fragment_snapshot_t* out) {
  if (!out) return false;
  for (;;) {
    const uint32_t before = g_last_fragment_generation;
    if (before & 1U) continue;
    photons_memory_barrier();
    *out = g_last_photons_fragment;
    photons_memory_barrier();
    const uint32_t after = g_last_fragment_generation;
    if (before == after && !(after & 1U)) return out->snapshot_ok;
  }
}


static const photons_fragment_snapshot_t& photons_report_fragment_snapshot(void) {
  // REPORT commands are foreground-serialized, so one dedicated RAM2 scratch is
  // sufficient. The source copy itself remains generation-validated in case the
  // completed-fragment writer ever moves to a preempting execution context.
  (void)photons_fragment_snapshot(&g_photons_report_snapshot);
  return g_photons_report_snapshot;
}

// ============================================================================
// PHOTONS_FRAGMENT canonical publication
// ============================================================================

// PHOTONS_FRAGMENT is a bounded, single-owner foreground serializer. Every
// schema node that can exceed Payload's inline store is explicitly FIXED; the
// few proven-inline leaves remain ordinary local Payloads. Dedicated canonical
// and nested byte stores live in cache-line-aligned RAM2. Every Payload control block --
// pointer/capacity guards, mutation generation, and contract fingerprint -- lives
// in ordinary RAM1 with its sole foreground owner.  A RAM2 backing-store injury
// therefore cannot silently rewrite the ownership metadata that is meant to
// detect it. Fixed-store constructors initialize each byte store before use;
// RAM2 placement does not depend on NOLOAD storage being zero at boot.
//
// Capacity exhaustion is a schema-contract failure, not an invitation to grow:
// Payload records FIXED_CAPACITY and fails hard.
#define PHOTONS_FRAGMENT_FIXED_RAM2(name, capacity)                        \
  static_assert(((capacity) % PHOTONS_RAM2_CACHE_LINE_BYTES) == 0U,       \
                "PHOTONS RAM2 fixed store must fill cache lines");        \
  alignas(PHOTONS_RAM2_CACHE_LINE_BYTES)                                  \
  static uint8_t name##_storage[capacity] DMAMEM;                         \
  static Payload name(                                                    \
      Payload::StorageMode::FIXED,                                        \
      name##_storage,                                                     \
      sizeof(name##_storage))

static constexpr size_t PHOTONS_FRAGMENT_ROOT_CAPACITY = 12288U;
static constexpr size_t PHOTONS_FRAGMENT_ROOT_GUARD_WORDS =
    PHOTONS_RAM2_CACHE_LINE_BYTES / sizeof(uint32_t);
static constexpr size_t PHOTONS_FRAGMENT_ROOT_SECTOR_BYTES = 1024U;
static constexpr size_t PHOTONS_FRAGMENT_ROOT_SECTOR_COUNT =
    PHOTONS_FRAGMENT_ROOT_CAPACITY / PHOTONS_FRAGMENT_ROOT_SECTOR_BYTES;
static constexpr uint32_t PHOTONS_FRAGMENT_ROOT_GUARD_BEFORE = 0x50524742UL; // 'PRGB'
static constexpr uint32_t PHOTONS_FRAGMENT_ROOT_GUARD_AFTER  = 0x50524741UL; // 'PRGA'
static constexpr uint32_t PHOTONS_FRAGMENT_ROOT_WITNESS_MAGIC = 0x50525731UL; // 'PRW1'
static constexpr uint32_t PHOTONS_FRAGMENT_ROOT_WITNESS_SCHEMA_VERSION = 1U;
static constexpr uint32_t PHOTONS_FRAGMENT_ROOT_HASH_OFFSET = 2166136261UL;
static constexpr uint32_t PHOTONS_FRAGMENT_ROOT_HASH_PRIME = 16777619UL;

static_assert((PHOTONS_FRAGMENT_ROOT_CAPACITY %
               PHOTONS_RAM2_CACHE_LINE_BYTES) == 0U,
              "PHOTONS root store must fill complete cache lines");
static_assert((PHOTONS_FRAGMENT_ROOT_CAPACITY %
               PHOTONS_FRAGMENT_ROOT_SECTOR_BYTES) == 0U,
              "PHOTONS root witness sectors must divide the root store");
static_assert(PHOTONS_FRAGMENT_ROOT_SECTOR_COUNT <= 32U,
              "PHOTONS root sector mismatch mask exceeds uint32_t");

struct alignas(PHOTONS_RAM2_CACHE_LINE_BYTES)
photons_fragment_root_storage_t {
  uint32_t guard_before[PHOTONS_FRAGMENT_ROOT_GUARD_WORDS];
  uint8_t storage[PHOTONS_FRAGMENT_ROOT_CAPACITY];
  uint32_t guard_after[PHOTONS_FRAGMENT_ROOT_GUARD_WORDS];
};

static_assert(offsetof(photons_fragment_root_storage_t, storage) ==
                  PHOTONS_RAM2_CACHE_LINE_BYTES,
              "PHOTONS root body must begin on its own cache line");
static_assert((offsetof(photons_fragment_root_storage_t, guard_after) %
               PHOTONS_RAM2_CACHE_LINE_BYTES) == 0U,
              "PHOTONS trailing root guard must begin on a cache line");
static_assert((sizeof(photons_fragment_root_storage_t) %
               PHOTONS_RAM2_CACHE_LINE_BYTES) == 0U,
              "PHOTONS guarded root region must fill cache lines");

static photons_fragment_root_storage_t g_photons_fragment_root_region DMAMEM;
static Payload g_photons_fragment_root(
    Payload::StorageMode::FIXED,
    g_photons_fragment_root_region.storage,
    sizeof(g_photons_fragment_root_region.storage));
PHOTONS_FRAGMENT_FIXED_RAM2(g_photons_fragment_instrument, 12288U);

// LANTERN V1.0 adds autonomous-race rejection/reference testimony to the
// PHOTONS_RACE object.  The old 1536-byte store was sized for the 25-field
// cadence-era object. Include the holdoff and pending-successor witnesses plus
// the nested flight_ns Welford. Keep explicit fixed custody with schema headroom.
PHOTONS_FRAGMENT_FIXED_RAM2(g_photons_fragment_race, 3072U);
PHOTONS_FRAGMENT_FIXED_RAM2(g_photons_fragment_raw_cycles, 1024U);
PHOTONS_FRAGMENT_FIXED_RAM2(g_photons_fragment_projection, 1024U);
PHOTONS_FRAGMENT_FIXED_RAM2(g_photons_fragment_science, 3072U);
PHOTONS_FRAGMENT_FIXED_RAM2(g_photons_fragment_science_accepted, 768U);
PHOTONS_FRAGMENT_FIXED_RAM2(g_photons_fragment_science_excluded, 768U);
PHOTONS_FRAGMENT_FIXED_RAM2(g_photons_fragment_science_reasons, 512U);
PHOTONS_FRAGMENT_FIXED_RAM2(g_photons_fragment_stats, 4096U);
PHOTONS_FRAGMENT_FIXED_RAM2(g_photons_fragment_welford, 512U);
PHOTONS_FRAGMENT_FIXED_RAM2(g_photons_fragment_ppb_buckets, 1024U);
PHOTONS_FRAGMENT_FIXED_RAM2(g_photons_fragment_ppb_checkpoint, 2048U);
PHOTONS_FRAGMENT_FIXED_RAM2(g_photons_fragment_campaign, 768U);
PHOTONS_FRAGMENT_FIXED_RAM2(g_photons_fragment_campaign_stats, 512U);
PHOTONS_FRAGMENT_FIXED_RAM2(g_photons_fragment_recovery, 1024U);
PHOTONS_FRAGMENT_FIXED_RAM2(g_photons_fragment_interrupt, 1280U);

#undef PHOTONS_FRAGMENT_FIXED_RAM2


enum class photons_fragment_root_mismatch_stage_t : uint32_t {
  NONE = 0U,
  RENDER = 1U,
  PUBLISH = 2U,
  POST_PUBLISH = 3U,
  QUIESCENT = 4U,
};

struct photons_fragment_root_snapshot_t {
  bool valid = false;
  uint32_t fragment_sequence = 0U;
  uint32_t dwt = 0U;
  uint32_t hash = 0U;
  uint32_t sector_hash[PHOTONS_FRAGMENT_ROOT_SECTOR_COUNT] = {};
  uint32_t first_line[PHOTONS_FRAGMENT_ROOT_GUARD_WORDS] = {};
  uint32_t last_line[PHOTONS_FRAGMENT_ROOT_GUARD_WORDS] = {};
  uint32_t guard_failure_mask = 0U;  // bit 0=before, bit 1=after
  uint32_t guard_index = 0xFFFFFFFFUL;
  uint32_t guard_expected = 0U;
  uint32_t guard_observed = 0U;
};

struct alignas(PHOTONS_RAM2_CACHE_LINE_BYTES)
photons_fragment_root_mismatch_record_t {
  uint32_t magic;
  uint32_t magic_inv;
  uint32_t schema_version;
  uint32_t record_size;
  uint32_t sequence;
  uint32_t sequence_inv;
  uint32_t stage;
  uint32_t fragment_sequence;
  uint32_t expected_fragment_sequence;
  uint32_t dwt;
  uint32_t storage_begin;
  uint32_t storage_end;
  uint32_t expected_hash;
  uint32_t observed_hash;
  uint32_t sector_mismatch_mask;
  uint32_t guard_failure_mask;
  uint32_t guard_index;
  uint32_t guard_expected;
  uint32_t guard_observed;
  uint32_t expected_first_line[PHOTONS_FRAGMENT_ROOT_GUARD_WORDS];
  uint32_t observed_first_line[PHOTONS_FRAGMENT_ROOT_GUARD_WORDS];
  uint32_t expected_last_line[PHOTONS_FRAGMENT_ROOT_GUARD_WORDS];
  uint32_t observed_last_line[PHOTONS_FRAGMENT_ROOT_GUARD_WORDS];
  uint32_t reserved[5];
};

static_assert(sizeof(photons_fragment_root_mismatch_record_t) == 224U,
              "PHOTONS root mismatch record geometry changed");
static_assert((sizeof(photons_fragment_root_mismatch_record_t) %
               PHOTONS_RAM2_CACHE_LINE_BYTES) == 0U,
              "PHOTONS root mismatch record must fill cache lines");

// The expected quiescent image is current-boot RAM1 state.  Only an actual
// mismatch is retained in RAM2.  Thus ordinary publication does not add another
// recurring retained write/cache-maintenance path.
static photons_fragment_root_snapshot_t g_photons_fragment_root_quiescent{};
static photons_fragment_root_mismatch_record_t
    g_photons_fragment_root_mismatch_retained DMAMEM;

static FLASHMEM const char* photons_fragment_root_mismatch_stage_name(uint32_t stage) {
  switch ((photons_fragment_root_mismatch_stage_t)stage) {
    case photons_fragment_root_mismatch_stage_t::RENDER:       return "RENDER";
    case photons_fragment_root_mismatch_stage_t::PUBLISH:      return "PUBLISH";
    case photons_fragment_root_mismatch_stage_t::POST_PUBLISH: return "POST_PUBLISH";
    case photons_fragment_root_mismatch_stage_t::QUIESCENT:    return "QUIESCENT";
    default:                                                   return "NONE";
  }
}

static FLASHMEM bool photons_fragment_root_mismatch_record_valid(
    const photons_fragment_root_mismatch_record_t& record) {
  return record.magic == PHOTONS_FRAGMENT_ROOT_WITNESS_MAGIC &&
      record.magic_inv == ~PHOTONS_FRAGMENT_ROOT_WITNESS_MAGIC &&
      record.schema_version == PHOTONS_FRAGMENT_ROOT_WITNESS_SCHEMA_VERSION &&
      record.record_size == sizeof(record) &&
      record.sequence != 0U &&
      (record.sequence ^ record.sequence_inv) == 0xFFFFFFFFUL;
}

static FLASHMEM void photons_fragment_root_initialize_runtime(void) {
  for (size_t i = 0U; i < PHOTONS_FRAGMENT_ROOT_GUARD_WORDS; ++i) {
    g_photons_fragment_root_region.guard_before[i] =
        PHOTONS_FRAGMENT_ROOT_GUARD_BEFORE;
    g_photons_fragment_root_region.guard_after[i] =
        PHOTONS_FRAGMENT_ROOT_GUARD_AFTER;
  }
  g_photons_fragment_root_quiescent = photons_fragment_root_snapshot_t{};
  photons_memory_barrier();
}

// The root body is declared as byte storage.  Preserve that type identity even
// for diagnostic cache-line testimony: assemble words from bytes rather than
// manufacturing uint32_t objects over the same storage with a pointer cast.
static inline uint32_t photons_load_u32_le_volatile(
    const volatile uint8_t* bytes) {
  if (!bytes) __builtin_trap();
  return (uint32_t)bytes[0] |
      ((uint32_t)bytes[1] << 8) |
      ((uint32_t)bytes[2] << 16) |
      ((uint32_t)bytes[3] << 24);
}

static FLASHMEM void photons_fragment_root_snapshot_capture(
    uint32_t fragment_sequence,
    photons_fragment_root_snapshot_t* out) {
  if (!out) __builtin_trap();
  *out = photons_fragment_root_snapshot_t{};
  out->valid = true;
  out->fragment_sequence = fragment_sequence;
  out->dwt = ARM_DWT_CYCCNT;

  bool guard_recorded = false;
  for (size_t i = 0U; i < PHOTONS_FRAGMENT_ROOT_GUARD_WORDS; ++i) {
    const volatile uint32_t* const guard =
        &g_photons_fragment_root_region.guard_before[i];
    const uint32_t observed = *guard;
    if (observed == PHOTONS_FRAGMENT_ROOT_GUARD_BEFORE) continue;
    out->guard_failure_mask |= 1U << 0;
    if (!guard_recorded) {
      out->guard_index = (uint32_t)i;
      out->guard_expected = PHOTONS_FRAGMENT_ROOT_GUARD_BEFORE;
      out->guard_observed = observed;
      guard_recorded = true;
    }
  }
  for (size_t i = 0U; i < PHOTONS_FRAGMENT_ROOT_GUARD_WORDS; ++i) {
    const volatile uint32_t* const guard =
        &g_photons_fragment_root_region.guard_after[i];
    const uint32_t observed = *guard;
    if (observed == PHOTONS_FRAGMENT_ROOT_GUARD_AFTER) continue;
    out->guard_failure_mask |= 1U << 1;
    if (!guard_recorded) {
      out->guard_index = (uint32_t)i;
      out->guard_expected = PHOTONS_FRAGMENT_ROOT_GUARD_AFTER;
      out->guard_observed = observed;
      guard_recorded = true;
    }
  }

  const volatile uint8_t* const storage =
      g_photons_fragment_root_region.storage;
  uint32_t whole_hash = PHOTONS_FRAGMENT_ROOT_HASH_OFFSET;
  for (size_t sector = 0U;
       sector < PHOTONS_FRAGMENT_ROOT_SECTOR_COUNT;
       ++sector) {
    uint32_t sector_hash = PHOTONS_FRAGMENT_ROOT_HASH_OFFSET;
    const size_t base = sector * PHOTONS_FRAGMENT_ROOT_SECTOR_BYTES;
    for (size_t offset = 0U;
         offset < PHOTONS_FRAGMENT_ROOT_SECTOR_BYTES;
         ++offset) {
      const uint8_t value = storage[base + offset];
      whole_hash = (whole_hash ^ value) * PHOTONS_FRAGMENT_ROOT_HASH_PRIME;
      sector_hash = (sector_hash ^ value) * PHOTONS_FRAGMENT_ROOT_HASH_PRIME;
    }
    out->sector_hash[sector] = sector_hash;
  }
  out->hash = whole_hash;

  const volatile uint8_t* const first = storage;
  const volatile uint8_t* const last =
      storage + PHOTONS_FRAGMENT_ROOT_CAPACITY -
      PHOTONS_RAM2_CACHE_LINE_BYTES;
  for (size_t i = 0U; i < PHOTONS_FRAGMENT_ROOT_GUARD_WORDS; ++i) {
    const size_t byte_offset = i * sizeof(uint32_t);
    out->first_line[i] =
        photons_load_u32_le_volatile(first + byte_offset);
    out->last_line[i] =
        photons_load_u32_le_volatile(last + byte_offset);
  }
}

static FLASHMEM uint32_t photons_fragment_root_sector_mismatch_mask(
    const photons_fragment_root_snapshot_t& expected,
    const photons_fragment_root_snapshot_t& observed) {
  if (!expected.valid || !observed.valid) return 0U;
  uint32_t mask = 0U;
  for (size_t i = 0U; i < PHOTONS_FRAGMENT_ROOT_SECTOR_COUNT; ++i) {
    if (expected.sector_hash[i] != observed.sector_hash[i]) {
      mask |= 1UL << i;
    }
  }
  return mask;
}

static FLASHMEM void photons_fragment_root_mismatch_commit(
    photons_fragment_root_mismatch_stage_t stage,
    uint32_t fragment_sequence,
    const photons_fragment_root_snapshot_t& expected,
    const photons_fragment_root_snapshot_t& observed) {
  const bool previous_valid = photons_fragment_root_mismatch_record_valid(
      g_photons_fragment_root_mismatch_retained);
  uint32_t sequence = previous_valid
      ? g_photons_fragment_root_mismatch_retained.sequence + 1U
      : 1U;
  if (sequence == 0U) sequence = 1U;

  photons_fragment_root_mismatch_record_t record{};
  record.schema_version = PHOTONS_FRAGMENT_ROOT_WITNESS_SCHEMA_VERSION;
  record.record_size = sizeof(record);
  record.sequence = sequence;
  record.sequence_inv = ~sequence;
  record.stage = (uint32_t)stage;
  record.fragment_sequence = fragment_sequence;
  record.expected_fragment_sequence = expected.fragment_sequence;
  record.dwt = ARM_DWT_CYCCNT;
  record.storage_begin = (uint32_t)(uintptr_t)
      g_photons_fragment_root_region.storage;
  record.storage_end = record.storage_begin + PHOTONS_FRAGMENT_ROOT_CAPACITY;
  record.expected_hash = expected.valid ? expected.hash : 0U;
  record.observed_hash = observed.valid ? observed.hash : 0U;
  record.sector_mismatch_mask =
      photons_fragment_root_sector_mismatch_mask(expected, observed);
  record.guard_failure_mask = observed.guard_failure_mask;
  record.guard_index = observed.guard_index;
  record.guard_expected = observed.guard_expected;
  record.guard_observed = observed.guard_observed;
  if (expected.valid) {
    for (size_t i = 0U; i < PHOTONS_FRAGMENT_ROOT_GUARD_WORDS; ++i) {
      record.expected_first_line[i] = expected.first_line[i];
      record.expected_last_line[i] = expected.last_line[i];
    }
  }
  if (observed.valid) {
    for (size_t i = 0U; i < PHOTONS_FRAGMENT_ROOT_GUARD_WORDS; ++i) {
      record.observed_first_line[i] = observed.first_line[i];
      record.observed_last_line[i] = observed.last_line[i];
    }
  }

  // Commit envelope last.  The record occupies complete cache lines, so this
  // flush cannot sweep an unrelated RAM2 owner into the persistence operation.
  g_photons_fragment_root_mismatch_retained.magic = 0U;
  g_photons_fragment_root_mismatch_retained.magic_inv = 0U;
  photons_memory_barrier();
  arm_dcache_flush(&g_photons_fragment_root_mismatch_retained,
                   PHOTONS_RAM2_CACHE_LINE_BYTES);
  __asm__ volatile("dsb\nisb" ::: "memory");

  g_photons_fragment_root_mismatch_retained = record;
  photons_memory_barrier();
  g_photons_fragment_root_mismatch_retained.magic_inv =
      ~PHOTONS_FRAGMENT_ROOT_WITNESS_MAGIC;
  g_photons_fragment_root_mismatch_retained.magic =
      PHOTONS_FRAGMENT_ROOT_WITNESS_MAGIC;
  photons_memory_barrier();
  arm_dcache_flush(&g_photons_fragment_root_mismatch_retained,
                   sizeof(g_photons_fragment_root_mismatch_retained));
  __asm__ volatile("dsb\nisb" ::: "memory");
}

static FLASHMEM void photons_fragment_root_fail(
    photons_fragment_root_mismatch_stage_t stage,
    uint32_t fragment_sequence,
    const photons_fragment_root_snapshot_t& expected,
    const photons_fragment_root_snapshot_t& observed) {
  photons_fragment_root_mismatch_commit(
      stage, fragment_sequence, expected, observed);

  // Preserve the established Payload fatal testimony whenever the document's
  // own contract is also broken.  A guard-only or slack-byte injury is outside
  // Payload's semantic document, but is still an ownership violation and traps.
  if (!g_photons_fragment_root.contract_valid()) {
    g_photons_fragment_root.clear();  // does not return on contract injury
  }
  __builtin_trap();
}

static FLASHMEM void photons_fragment_root_verify_expected(
    photons_fragment_root_mismatch_stage_t stage,
    uint32_t fragment_sequence,
    const photons_fragment_root_snapshot_t& expected,
    photons_fragment_root_snapshot_t* observed_out) {
  photons_fragment_root_snapshot_t observed{};
  photons_fragment_root_snapshot_capture(fragment_sequence, &observed);
  const uint32_t sector_mismatch =
      photons_fragment_root_sector_mismatch_mask(expected, observed);
  if (!expected.valid ||
      expected.guard_failure_mask != 0U ||
      observed.guard_failure_mask != 0U ||
      expected.hash != observed.hash ||
      sector_mismatch != 0U) {
    photons_fragment_root_fail(stage, fragment_sequence, expected, observed);
  }
  if (observed_out) *observed_out = observed;
}

static FLASHMEM void photons_fragment_root_verify_quiescent(
    uint32_t next_fragment_sequence) {
  photons_fragment_root_snapshot_t observed{};
  photons_fragment_root_snapshot_capture(next_fragment_sequence, &observed);

  if (!g_photons_fragment_root_quiescent.valid) {
    if (observed.guard_failure_mask != 0U) {
      const photons_fragment_root_snapshot_t expected{};
      photons_fragment_root_fail(
          photons_fragment_root_mismatch_stage_t::QUIESCENT,
          next_fragment_sequence,
          expected,
          observed);
    }
    return;
  }

  const photons_fragment_root_snapshot_t expected =
      g_photons_fragment_root_quiescent;
  const uint32_t sector_mismatch =
      photons_fragment_root_sector_mismatch_mask(expected, observed);
  if (expected.guard_failure_mask != 0U ||
      observed.guard_failure_mask != 0U ||
      expected.hash != observed.hash ||
      sector_mismatch != 0U) {
    photons_fragment_root_fail(
        photons_fragment_root_mismatch_stage_t::QUIESCENT,
        next_fragment_sequence,
        expected,
        observed);
  }

  // The next root.clear() is now the sole lawful mutation boundary.
  g_photons_fragment_root_quiescent.valid = false;
  photons_memory_barrier();
}

static FLASHMEM void photons_fragment_root_add_report(Payload& parent) {
  photons_foreground_owner_assert(photons_foreground_owner_t::COMMAND);
  Payload custody;
  custody.add("schema", "PHOTONS_ROOT_STORAGE_CUSTODY_V1");
  custody.add("storage_begin", (uint32_t)(uintptr_t)
      g_photons_fragment_root_region.storage);
  custody.add("storage_end", (uint32_t)(uintptr_t)
      (g_photons_fragment_root_region.storage +
       PHOTONS_FRAGMENT_ROOT_CAPACITY));
  custody.add("storage_capacity", (uint32_t)PHOTONS_FRAGMENT_ROOT_CAPACITY);
  custody.add("guard_before_address", (uint32_t)(uintptr_t)
      g_photons_fragment_root_region.guard_before);
  custody.add("guard_after_address", (uint32_t)(uintptr_t)
      g_photons_fragment_root_region.guard_after);
  custody.add("quiescent_witness_valid",
              g_photons_fragment_root_quiescent.valid);
  custody.add("quiescent_fragment_sequence",
              g_photons_fragment_root_quiescent.fragment_sequence);
  custody.add("quiescent_hash", g_photons_fragment_root_quiescent.hash);

  const bool mismatch_valid = photons_fragment_root_mismatch_record_valid(
      g_photons_fragment_root_mismatch_retained);
  custody.add("retained_mismatch_valid", mismatch_valid);
  if (mismatch_valid) {
    const photons_fragment_root_mismatch_record_t& mismatch =
        g_photons_fragment_root_mismatch_retained;
    custody.add("retained_mismatch_sequence", mismatch.sequence);
    custody.add("retained_mismatch_stage_id", mismatch.stage);
    custody.add("retained_mismatch_stage",
                photons_fragment_root_mismatch_stage_name(mismatch.stage));
    custody.add("retained_fragment_sequence", mismatch.fragment_sequence);
    custody.add("retained_expected_fragment_sequence",
                mismatch.expected_fragment_sequence);
    custody.add("retained_expected_hash", mismatch.expected_hash);
    custody.add("retained_observed_hash", mismatch.observed_hash);
    custody.add("retained_sector_mismatch_mask",
                mismatch.sector_mismatch_mask);
    custody.add("retained_guard_failure_mask",
                mismatch.guard_failure_mask);
    custody.add("retained_guard_index", mismatch.guard_index);
    custody.add("retained_guard_expected", mismatch.guard_expected);
    custody.add("retained_guard_observed", mismatch.guard_observed);
    custody.add("retained_dwt", mismatch.dwt);
  }
  parent.add_object("root_storage_custody", custody);
}


static void photons_payload_add_welford(
    Payload& parent,
    const char* name,
    const photons_fragment_welford_snapshot_t& w) {
  photons_foreground_owner_assert(photons_foreground_owner_t::FRAGMENT);
  Payload& obj = g_photons_fragment_welford;
  obj.clear();
  obj.add("n", w.n);
  obj.add("mean", toFixedDecimal(w.mean, 6));
  // M2 grows with population and squared excursions; it is not bounded by
  // the fixed-decimal whole-part limit. Preserve all binary64 recovery digits.
  obj.add("m2", toScientificDecimal(w.m2));
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
  photons_foreground_owner_assert(photons_foreground_owner_t::FRAGMENT);
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

  photons_fragment_root_verify_quiescent(f.sequence);
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
  race.add("accounting", "RETURN_HOLDOFF_V1");
  race.add("pending_relaunch_count", f.race_pending_relaunch_count);
  race.add("pending_relaunch_count_previous", f.race_pending_relaunch_count_previous);
  race.add("active", f.race_engine_active);
  race.add("cadence_hz", f.race_cadence_hz);
  race.add("cadence_ns", PHOTONS_RACE_CADENCE_NS);
  race.add("pulse_ns", f.race_pulse_ns);
  race.add("launch_surrogate", "DRV200_MOD_HIGH_EDGE_OBSERVED");
  race.add("flight_interpretation", "OBSERVED_DWT_ENDPOINTS");
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
  // Step-2 contract probe only. The producer does not yet own a mutable
  // backpressure counter and race authorship is unchanged. Publish explicit
  // zero testimony so Pi cadence accounting can prove the new wire shape
  // independently of any runtime-state or acquisition-behavior change.
  race.add("skipped_backpressure_total", (uint64_t)0ULL);
  race.add("skipped_backpressure_this_fragment", (uint32_t)0U);
  race.add("invalid_endpoint_total", f.race_invalid_endpoint_total);
  race.add("invalid_endpoint_this_fragment",
           f.race_invalid_endpoint_this_fragment);
  race.add("enqueue_failure_total", f.race_enqueue_failure_total);
  race.add("enqueue_failure_this_fragment",
           f.race_enqueue_failure_this_fragment);
  race.add("rejected_isr_delay_total", f.race_rejected_isr_delay_total);
  race.add("rejected_isr_delay_this_fragment",
           f.race_rejected_isr_delay_this_fragment);
  race.add("rejected_qtimer1_total", f.race_rejected_qtimer1_total);
  race.add("rejected_ocxo1_total", f.race_rejected_ocxo1_total);
  race.add("rejected_ocxo2_total", f.race_rejected_ocxo2_total);
  race.add("rejected_pps_total", f.race_rejected_pps_total);
  race.add("rejected_continuation_total", f.race_rejected_continuation_total);
  race.add("rejected_unknown_total", f.race_rejected_unknown_total);
  race.add("rejected_excursion_total", f.race_rejected_excursion_total);
  race.add("rejected_excursion_this_fragment",
           f.race_rejected_excursion_this_fragment);
  race.add("reference_valid", f.race_reference_valid);
  race.add("reference_cycles", f.race_reference_cycles);
  race.add("reference_gate_cycles", f.race_reference_gate_cycles);
  race.add("seed_count", f.race_seed_count);
  race.add("holdoff_ns", f.race_holdoff_ns);
  race.add("holdoff_cycles", f.race_holdoff_cycles);
  race.add("holdoff_edges_total", f.race_holdoff_edges_total);
  race.add("holdoff_launches_total", f.race_holdoff_launches_total);
  race.add("holdoff_last_cycles", f.race_holdoff_last_cycles);
  race.add("holdoff_min_cycles", f.race_holdoff_min_cycles);
  race.add("holdoff_max_cycles", f.race_holdoff_max_cycles);
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
  reasons.add("isr_delay",
              f.science.exclusion_reasons.isr_delay);
  reasons.add("projection_invalid_this_fragment",
              f.science.exclusion_reasons.projection_invalid_this_fragment);
  reasons.add("seed_disagreement_this_fragment",
              f.science.exclusion_reasons.seed_disagreement_this_fragment);
  reasons.add("raw_cycle_excursion_this_fragment",
              f.science.exclusion_reasons.raw_cycle_excursion_this_fragment);
  reasons.add("isr_delay_this_fragment",
              f.science.exclusion_reasons.isr_delay_this_fragment);
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
  science.add("seed_pending_count", f.science.seed_pending_count);
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
  stats.add("custody_lap_count", f.stats.custody_lap_count);
  stats.add("custody_total_lap_gnss_ns",
            f.stats.custody_total_lap_gnss_ns);
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


// The 1 Hz publication transaction runs from flash; race/ISR paths stay in ITCM.
static FLASHMEM void photons_fragment_tick(
    timepop_ctx_t* /*ctx*/,
    timepop_diag_t* /*diag*/,
    void* /*user_data*/) {

  // PHOTONS_FRAGMENT is ready-to-eat testimony.  Until the Pi supplies the
  // required standard lap, PHOTONS has no authority to publish interpreted
  // optical statistics.
  if (!g_standard_lap_configured || g_lap_baseline_fs == 0ULL ||
      !g_photons_recovery.publication_started) return;

  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::FRAGMENT);

  // Request a producer-owned snapshot at fragment cadence. Reports use the
  // latest completed publication; neither caller copies the live histogram.
  (void)photons_histogram_snapshot_acquire();

  // Refresh the PHOTONS-owned immutable copy for laps that will arrive after
  // this boundary.  Records already in the ring carry the anchor that was
  // current when their physical DWT endpoints were observed.
  photons_projection_anchor_refresh();

  photons_toy_capture_t capture{};
  if (!photons_toy_capture_snapshot(&capture)) __builtin_trap();

  interrupt_photodiode_diag_t interrupt_diag{};
  if (!interrupt_photodiode_snapshot(&interrupt_diag)) __builtin_trap();
  const photons_interrupt_ancestry_t interrupt_ancestry = g_interrupt_ancestry;
  if (!interrupt_ancestry.valid) __builtin_trap();
  const uint32_t interrupt_callback_missing_since_ancestry =
      interrupt_diag.callback_missing_count -
      interrupt_ancestry.callback_missing_origin;
  const uint32_t interrupt_inactive_edge_since_ancestry =
      interrupt_diag.inactive_edge_count -
      interrupt_ancestry.inactive_edge_origin;

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

  const photons_race_runtime_t race = photons_race_runtime_snapshot();

  // Publication is the PHOTONS heartbeat; race production is an independent
  // lifecycle. Freeze that distinction into the immutable fragment so a zero-race
  // second is explicit testimony rather than ambiguous silence.
  const bool race_engine_active =
      race.active;

  // This complete foreground-owned value is the publication custody boundary.
  // Keep the large canonical object in a dedicated RAM2 build slot rather than
  // placing ~1.7 KiB on MSP. This callback is the slot's sole owner; it completes
  // the value before either the canonical snapshot or Payload serializer sees it.
  // From the first Payload mutation onward, the serializer may read only this
  // frozen value plus constants/pure formatting helpers.
  photons_fragment_snapshot_t& fragment = g_photons_fragment_build;
  fragment = photons_fragment_snapshot_t{};
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

  fragment.race_engine_active = race_engine_active;
  photons_race_snapshot_relaunch_accounting(
      fragment, race, g_last_fragment_race_completed_count,
      g_last_fragment_race_attempt_count);
  fragment.race_cadence_hz = PHOTONS_RACE_CADENCE_HZ;
  fragment.race_pulse_ns = PHOTONS_RACE_PULSE_NS;
  fragment.race_cadence_tick_count_total = race.cadence_tick_count;
  fragment.race_cadence_ticks_this_fragment = (uint32_t)(
      race.cadence_tick_count -
      g_last_fragment_race_cadence_tick_count);
  fragment.race_attempt_count_total = race.attempt_count;
  fragment.race_attempts_this_fragment = (uint32_t)(
      race.attempt_count - g_last_fragment_race_attempt_count);
  fragment.race_completed_count_total = race.completed_count;
  fragment.race_completed_this_fragment = (uint32_t)(
      race.completed_count - g_last_fragment_race_completed_count);
  fragment.race_missed_count_total = race.missed_count;
  fragment.race_missed_this_fragment = (uint32_t)(
      race.missed_count - g_last_fragment_race_missed_count);
  fragment.race_skipped_not_quiet_total =
      race.skipped_not_quiet_count;
  fragment.race_skipped_not_quiet_this_fragment = (uint32_t)(
      race.skipped_not_quiet_count -
      g_last_fragment_race_skipped_not_quiet_count);
  fragment.race_skipped_projection_total =
      race.skipped_projection_count;
  fragment.race_skipped_projection_this_fragment = (uint32_t)(
      race.skipped_projection_count -
      g_last_fragment_race_skipped_projection_count);
  fragment.race_invalid_endpoint_total =
      race.invalid_endpoint_count;
  fragment.race_invalid_endpoint_this_fragment = (uint32_t)(
      race.invalid_endpoint_count -
      g_last_fragment_race_invalid_endpoint_count);
  fragment.race_enqueue_failure_total = race.enqueue_failure_count;
  fragment.race_enqueue_failure_this_fragment = (uint32_t)(
      race.enqueue_failure_count -
      g_last_fragment_race_enqueue_failure_count);
  fragment.race_rejected_isr_delay_total = race.rejected_isr_delay_count;
  fragment.race_rejected_isr_delay_this_fragment = (uint32_t)(
      race.rejected_isr_delay_count -
      g_last_fragment_race_rejected_isr_delay_count);
  fragment.race_rejected_qtimer1_total = race.rejected_qtimer1_count;
  fragment.race_rejected_ocxo1_total = race.rejected_ocxo1_count;
  fragment.race_rejected_ocxo2_total = race.rejected_ocxo2_count;
  fragment.race_rejected_pps_total = race.rejected_pps_count;
  fragment.race_rejected_continuation_total = race.rejected_continuation_count;
  fragment.race_rejected_unknown_total = race.rejected_unknown_count;
  fragment.race_rejected_excursion_total = race.rejected_excursion_count;
  fragment.race_rejected_excursion_this_fragment = (uint32_t)(
      race.rejected_excursion_count -
      g_last_fragment_race_rejected_excursion_count);
  fragment.race_reference_valid = race.reference_valid;
  fragment.race_reference_cycles = race.reference_cycles;
  fragment.race_reference_gate_cycles = race.reference_gate_cycles;
  fragment.race_seed_count = race.seed_count;
  fragment.race_holdoff_ns = PHOTONS_RACE_HOLDOFF_NS;
  fragment.race_holdoff_cycles = race.holdoff_cycles;
  fragment.race_holdoff_edges_total = race.holdoff_edges;
  fragment.race_holdoff_launches_total = race.holdoff_launches;
  fragment.race_holdoff_last_cycles = race.holdoff_last_cycles;
  fragment.race_holdoff_min_cycles = race.holdoff_min_cycles;
  fragment.race_holdoff_max_cycles = race.holdoff_max_cycles;
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
  fragment.stats.custody_lap_count = g_photons_custody_lap_count;
  fragment.stats.custody_total_lap_gnss_ns =
      g_photons_custody_total_lap_gnss_ns;
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
      interrupt_ancestry.valid;
  fragment.interrupt_callback_missing_origin =
      interrupt_ancestry.callback_missing_origin;
  fragment.interrupt_callback_missing_since_ancestry =
      interrupt_callback_missing_since_ancestry;
  fragment.interrupt_inactive_edge_count =
      interrupt_diag.inactive_edge_count;
  fragment.interrupt_inactive_edge_origin =
      interrupt_ancestry.inactive_edge_origin;
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
  photons_last_fragment_store(toy, fragment);
  g_last_published_edge_count = capture.edge_count;
  g_last_fragment_race_cadence_tick_count =
      fragment.race_cadence_tick_count_total;
  g_last_fragment_race_attempt_count = fragment.race_attempt_count_total;
  g_last_fragment_race_completed_count = fragment.race_completed_count_total;
  g_last_fragment_race_missed_count = fragment.race_missed_count_total;
  g_last_fragment_race_skipped_not_quiet_count =
      fragment.race_skipped_not_quiet_total;
  g_last_fragment_race_skipped_projection_count =
      fragment.race_skipped_projection_total;
  g_last_fragment_race_invalid_endpoint_count =
      fragment.race_invalid_endpoint_total;
  g_last_fragment_race_enqueue_failure_count =
      fragment.race_enqueue_failure_total;
  g_last_fragment_race_rejected_isr_delay_count =
      fragment.race_rejected_isr_delay_total;
  g_last_fragment_race_rejected_excursion_count =
      fragment.race_rejected_excursion_total;

  // Payload remains a renderer of this frozen value.  Keep the completed root
  // populated after publish: the next one-second root.clear() intentionally
  // remains an integrity canary for any out-of-band RAM2 mutation.
  Payload& payload = photons_fragment_payload(fragment);

  photons_fragment_root_snapshot_t root_before_publish{};
  photons_fragment_root_snapshot_capture(
      fragment.sequence, &root_before_publish);
  if (root_before_publish.guard_failure_mask != 0U) {
    photons_fragment_root_fail(
        photons_fragment_root_mismatch_stage_t::RENDER,
        fragment.sequence,
        photons_fragment_root_snapshot_t{},
        root_before_publish);
  }

  const bool published = publish("PHOTONS_FRAGMENT", payload);

  // publish() is a synchronous custody boundary: local fan-out may only borrow
  // the immutable value and transport must finish copying it into its own wire
  // allocation before returning.  Prove both the complete fixed byte store and
  // Payload's semantic contract before any post-publish PHOTONS mutation.
  photons_fragment_root_snapshot_t root_after_publish{};
  photons_fragment_root_verify_expected(
      photons_fragment_root_mismatch_stage_t::PUBLISH,
      fragment.sequence,
      root_before_publish,
      &root_after_publish);
  if (!payload.contract_valid()) {
    payload.clear();  // does not return when the preservation court is broken
  }

  if (published) {
    g_publish_count++;
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

  // FRAGMENT custody includes every post-publish lifecycle/statistics commit.
  // Prove those operations did not touch the completed root, then establish the
  // exact image that must remain immutable until next second's clear boundary.
  photons_fragment_root_snapshot_t root_at_release{};
  photons_fragment_root_verify_expected(
      photons_fragment_root_mismatch_stage_t::POST_PUBLISH,
      fragment.sequence,
      root_after_publish,
      &root_at_release);
  g_photons_fragment_root_quiescent = root_at_release;
  photons_memory_barrier();
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

  // g_photons_live is ISR-owned cumulative testimony. Recovery must never become
  // a second writer to its seqlock. Reclaim ancestry by draining foreground
  // custody and rebasing publication origins on a coherent live snapshot below.
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
  g_last_fragment_race_rejected_isr_delay_count = 0ULL;
  g_last_fragment_race_rejected_excursion_count = 0ULL;
  photons_last_fragment_reset();
}


static void photons_start_fragment_publisher(void) {
  if (!g_standard_lap_configured || g_lap_baseline_fs == 0ULL ||
      g_photons_recovery.publication_started ||
      g_fragment_timer != TIMEPOP_INVALID_HANDLE ||
      !g_photons_ppb_previous_endpoint_valid ||
      !g_interrupt_started ||
      !g_interrupt_ancestry.valid ||
      g_photons_race.active) {
    __builtin_trap();
  }

  g_fragment_timer = timepop_arm(
      PHOTONS_FRAGMENT_PERIOD_NS,
      true,
      photons_fragment_tick,
      nullptr,
      "PHOTONS_FRAGMENT");
  if (g_fragment_timer == TIMEPOP_INVALID_HANDLE) __builtin_trap();

  // PHOTONS_FRAGMENT is the always-on instrument heartbeat. A lawful recovery
  // verdict starts publication only; race production remains independently held.
  // Zero races in a second are therefore explicit canonical testimony, not silence.
  g_photons_recovery.publication_started = true;
  photons_race_start_autonomous();
}


// ============================================================================
// Initialization
// ============================================================================

FLASHMEM void process_photons_init(void) {
  if (g_initialized) return;

  // Register once. TimePop yields idle at the holdoff boundary and services
  // consumption/relaunch in foreground without allocating an ALAP mailbox.
  timepop_register_foreground_service(
      photons_relaunch_ready, photons_foreground_service, nullptr);

  // Initialization runs before PHOTONS publishes any foreground work. Establish
  // the one foreground mutation domain explicitly before the ISR subscription
  // becomes live.
  __atomic_store_n(&g_photons_foreground_owner,
                   (uint32_t)photons_foreground_owner_t::NONE,
                   __ATOMIC_RELEASE);
  photons_fragment_root_initialize_runtime();
  photons_histogram_initialize_handoffs();

  // This is the only foreground initialization of ISR-owned live capture and it
  // occurs before the PHOTODIODE subscription exists. Step 2 deliberately binds
  // the callback without activating the detector lane; once a later explicit
  // activation occurs, only the detector callback may mutate g_photons_live.
  g_photons_live = photons_live_state_t{};
  g_last_fragment_generation = 0U;
  photons_last_fragment_reset();
  g_photons_fragment_build = photons_fragment_snapshot_t{};
  g_photons_report_snapshot = photons_fragment_snapshot_t{};
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
  g_last_fragment_race_rejected_isr_delay_count = 0ULL;
  g_last_fragment_race_rejected_excursion_count = 0ULL;

  // Establish cross-context mailboxes before the PHOTODIODE subscription exists.
  // Subscription alone does not make the detector callback live. After a later
  // explicit activation, only the detector callback may mutate receive mailbox
  // contents; foreground owns only the arm/launch side of each handoff.
  g_pulse_armed_sequence = 0U;
  g_pulse_sequence = 0U;
  g_last_pulse_launch = photons_pulse_launch_state_t{};
  g_last_pulse_receive.generation = 0U;
  g_last_pulse_receive.value = photons_pulse_receive_value_t{};

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

  interrupt_photodiode_subscription_t subscription{};
  subscription.on_edge = photons_on_photodiode_edge;
  subscription.user_data = nullptr;

  g_subscription_ok = interrupt_photodiode_subscribe(subscription);
  // Step 2 separates callback identity from hardware activation. The physical
  // PHOTODIODE lane remains inactive until a later commissioning step explicitly
  // starts it; ordinary ZPNet operation therefore cannot enter PHOTONS ISR work.
  g_interrupt_started = false;

  // Prime the PHOTONS-owned projection cache before the race engine begins. TIME
  // may still be initializing; invalidity is preserved and early cadence cells
  // are skipped rather than projected through an invented ruler.
  photons_projection_anchor_refresh();

  // The fragment publisher and real race cadence are intentionally not started
  // here. SET_LAP_BASELINE_NS installs the current operator reference. A later
  // RECOVERY_COMMIT or RECOVERY_COLD_START establishes the statistical origin,
  // clears boot-local physical ancestry, and starts the 1 Hz PHOTONS_FRAGMENT
  // heartbeat exactly once. Race production remains separately held until an
  // explicit later commissioning step starts it.
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
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);
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
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);
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
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);
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
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);
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
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);
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
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);
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
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);
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
    uint64_t isr_delay,
    const photons_welford_state_t& accepted_projected,
    const photons_welford_state_t& accepted_raw,
    const photons_welford_state_t& excluded_raw,
    const photons_welford_state_t& excluded_projected) {
  photons_lap_science_reason_counts_snapshot_t recovery_reasons{};
  recovery_reasons.projection_invalid = projection_invalid;
  recovery_reasons.seed_disagreement = seed_disagreement;
  recovery_reasons.raw_cycle_excursion = raw_cycle_excursion;
  recovery_reasons.isr_delay = isr_delay;
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

  // Restore the same projection custody used by the autonomous race path:
  // raw exclusions contributed candidates, but never projection attempts.
  const uint64_t preprojection_exclusions =
      seed_disagreement + raw_cycle_excursion + isr_delay;
  g_projection_state = photons_fragment_projection_snapshot_t{};
  g_projection_state.attempt_count = candidate_count - preprojection_exclusions;
  g_projection_state.reject_count = projection_invalid;
  g_projection_state.success_count =
      g_projection_state.attempt_count - projection_invalid;

  g_photons_lap_science_state = photons_lap_science_snapshot_t{};
  g_photons_lap_science_state.valid = candidate_count != 0ULL;
  g_photons_lap_science_state.candidate_count = candidate_count;
  g_photons_lap_science_state.accepted.count = accepted_count;
  // excluded_count is recovery testimony only.  The installed producer derives
  // its aggregate exclusion population from the four reason counters below.
  g_photons_lap_science_state.exclusion_reasons.projection_invalid =
      projection_invalid;
  g_photons_lap_science_state.exclusion_reasons.seed_disagreement =
      seed_disagreement;
  g_photons_lap_science_state.exclusion_reasons.raw_cycle_excursion =
      raw_cycle_excursion;
  g_photons_lap_science_state.exclusion_reasons.isr_delay =
      isr_delay;
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
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);
  photons_recovery_protocol_t& protocol = g_photons_recovery_protocol;
  if (!protocol.active || g_photons_recovery.publication_started) {
    return photons_recovery_reject(
        "recovery_commit_rejected_not_staging",
        "RECOVERY_BEGIN must own a complete held transaction");
  }
  if (!g_interrupt_started) {
    return photons_recovery_reject(
        "recovery_commit_rejected_detector_inactive",
        "PHOTODIODE must be explicitly activated before publication");
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
  uint64_t isr_delay = 0ULL;
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
      photons_recovery_get_u64(args, "isr_delay", isr_delay) &&
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
  recovery_reasons.isr_delay = isr_delay;
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
      isr_delay,
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
  photons_start_fragment_publisher();

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
  p.add("race_engine_active", false);
  p.add("fresh_physical_ancestry", true);
  p.add("pending_seed_restored", false);
  p.add("raw_lap_ring_restored", false);
  p.add("proof_pending", true);
  return p;
}


static FLASHMEM Payload cmd_recovery_cold_start(const Payload& args) {
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);
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
  if (!g_interrupt_started) {
    return photons_recovery_reject(
        "recovery_cold_start_rejected_detector_inactive",
        "PHOTODIODE must be explicitly activated before cold start");
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
  photons_start_fragment_publisher();

  Payload p;
  p.add("status", "recovery_cold_start_committed");
  p.add("generation", generation);
  p.add("publication_started", true);
  p.add("race_engine_active", false);
  p.add("restored", false);
  p.add("fresh_physical_ancestry", true);
  return p;
}


static FLASHMEM Payload cmd_recovery_proof_ack(const Payload& args) {
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);
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


static constexpr size_t PHOTONS_OPERATIONAL_REPORT_RESERVE_BYTES = 4096U;

static void photons_prepare_operational_report(Payload& p) {
  // Substantial command reports should not mutate through Payload's small inline
  // representation and then grow mid-document. Establish one complete backing
  // store before the first semantic mutation.
  p.reserve(PHOTONS_OPERATIONAL_REPORT_RESERVE_BYTES);
}


static FLASHMEM Payload cmd_report_recovery(const Payload& /*args*/) {
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);
  Payload p;
  photons_prepare_operational_report(p);
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


static FLASHMEM Payload photons_flash_cut_command_body(const Payload& args) {
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


static FLASHMEM Payload cmd_flash_cut(const Payload& args) {
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);
  return photons_flash_cut_command_body(args);
}


static FLASHMEM Payload cmd_start(const Payload& args) {
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);
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
    return photons_flash_cut_command_body(args);
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
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);
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
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);
  // Both the canonical publisher and command dispatcher are foreground-owned;
  // use the immutable last-completed value directly instead of placing another
  // ~1.7 KiB copy on MSP.
  const photons_fragment_snapshot_t& canonical =
      photons_report_fragment_snapshot();

  Payload p;
  photons_prepare_operational_report(p);
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
  p.add("race_engine_active", canonical.race_engine_active);
  p.add("race_accounting", "RETURN_HOLDOFF_V1");
  p.add("race_pending_relaunch_count", canonical.race_pending_relaunch_count);
  p.add("race_pending_relaunch_count_previous", canonical.race_pending_relaunch_count_previous);
  p.add("race_holdoff_ns", canonical.race_holdoff_ns);
  p.add("race_holdoff_cycles", canonical.race_holdoff_cycles);
  p.add("race_holdoff_edges_total", canonical.race_holdoff_edges_total);
  p.add("race_holdoff_launches_total", canonical.race_holdoff_launches_total);
  p.add("race_holdoff_last_cycles", canonical.race_holdoff_last_cycles);
  p.add("race_holdoff_min_cycles", canonical.race_holdoff_min_cycles);
  p.add("race_holdoff_max_cycles", canonical.race_holdoff_max_cycles);
  p.add("race_cadence_hz", PHOTONS_RACE_CADENCE_HZ);
  p.add("race_pulse_ns", PHOTONS_RACE_PULSE_NS);
  p.add("race_launch_surrogate", "DRV200_MOD_HIGH_EDGE_OBSERVED");
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
  photons_fragment_root_add_report(p);
  return p;
}


static FLASHMEM Payload cmd_report_stats(const Payload& /*args*/) {
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);
  const photons_fragment_snapshot_t& canonical =
      photons_report_fragment_snapshot();

  Payload p;
  photons_prepare_operational_report(p);
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
        toScientificDecimal(canonical.stats.lap_time_welford.m2));
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
  p.add("science_exclusion_isr_delay",
        canonical.science.exclusion_reasons.isr_delay);
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
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);
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
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);
  Payload p;
  p.add("status", "inject_problem_rejected_source");
  p.add("error",
        "synthetic excursion injection was retired with the PHOTONS emulator");
  p.add("source", "PD200T_REAL_RACE");
  p.add("physical_measurement_modified", false);
  return p;
}

static FLASHMEM Payload cmd_report(const Payload& /*args*/) {
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);

  // Bring-up REPORT remains compact until publication begins. The DRV200 path is
  // active-HIGH modulation: LOW is the firmware-authored idle state. Step 3 may
  // activate the detector while modulation remains idle; that commissioning state
  // must not fall through into the broad-report serializer merely because pin-34
  // interrupt custody became live. Keep this path observational: it reports
  // detector activity and modulation state without starting publication, race
  // cadence, pulse generation, or any recovery transition.
  if (!g_photons_recovery.publication_started) {
    interrupt_photodiode_diag_t interrupt_diag{};
    if (!interrupt_photodiode_snapshot(&interrupt_diag) ||
        interrupt_diag.active != g_interrupt_started) {
      __builtin_trap();
    }

    const photons_device_snapshot_t device = photons_device_snapshot();

    Payload p;
    p.add("report", "PHOTONS");
    p.add("schema", "PHOTONS_BRINGUP_REPORT_V4");
    p.add("initialized", g_initialized);
    p.add("publication_started", false);
    p.add("interrupt_subscribed", interrupt_diag.subscribed);
    p.add("interrupt_active", interrupt_diag.active);
    p.add("interrupt_irq_count", interrupt_diag.irq_count);
    p.add("interrupt_callback_count", interrupt_diag.callback_count);
    p.add("interrupt_inactive_edge_count", interrupt_diag.inactive_edge_count);
    p.add("laser_mod_level", device.laser_mod_level);
    p.add("laser_mod_active_high", true);
    p.add("laser_mod_active", device.laser_mod_level == HIGH);
    p.add("laser_monitor_v", toFixedDecimal(device.laser_monitor_v, 6));
    p.add("laser_emitting", device.laser_emitting);
    p.add("race_engine_active",
          g_photons_race.active);
    return p;
  }

  const photons_fragment_snapshot_t& canonical =
      photons_report_fragment_snapshot();
  const photons_race_runtime_t race = photons_race_runtime_snapshot();

  interrupt_photodiode_diag_t interrupt_diag{};
  (void)interrupt_photodiode_snapshot(&interrupt_diag);
  const photons_device_snapshot_t device = photons_device_snapshot();

  Payload p;
  // Do not eagerly reserve 4 KiB for REPORT. The historical deterministic
  // REPORT failure entered Payload integrity machinery from this large-response
  // path, and the eager reserve forces a heap-backed representation before the
  // first semantic field exists. Preserve the exact REPORT schema while letting
  // Payload grow transactionally only if the document actually crosses inline
  // capacity. Other detailed report commands remain unchanged in this bounded fix.
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
        g_photons_race.active);
  p.add("race_cadence_hz", PHOTONS_RACE_CADENCE_HZ);
  p.add("race_pulse_ns", PHOTONS_RACE_PULSE_NS);
  p.add("race_launch_surrogate", "DRV200_MOD_HIGH_EDGE_OBSERVED");
  p.add("race_cadence_tick_count_total", race.cadence_tick_count);
  p.add("race_cadence_ticks_this_fragment", canonical.race_cadence_ticks_this_fragment);
  p.add("race_accounting", "RETURN_HOLDOFF_V1");
  p.add("race_pending_relaunch_count", photons_race_pending_relaunch(race));
  p.add("race_holdoff_ns", PHOTONS_RACE_HOLDOFF_NS);
  p.add("race_holdoff_cycles", race.holdoff_cycles);
  p.add("race_holdoff_edges_total", race.holdoff_edges);
  p.add("race_holdoff_launches_total", race.holdoff_launches);
  p.add("race_holdoff_last_cycles", race.holdoff_last_cycles);
  p.add("race_holdoff_min_cycles", race.holdoff_min_cycles);
  p.add("race_holdoff_max_cycles", race.holdoff_max_cycles);
  p.add("race_attempt_count_total", race.attempt_count);
  p.add("race_completed_count_total", race.completed_count);
  p.add("race_missed_count_total", race.missed_count);
  p.add("race_skipped_not_quiet_total", race.skipped_not_quiet_count);
  p.add("race_skipped_projection_total", race.skipped_projection_count);
  p.add("race_invalid_endpoint_total", race.invalid_endpoint_count);
  p.add("race_enqueue_failure_total", race.enqueue_failure_count);
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
  p.add("laser_mod_level", device.laser_mod_level);
  p.add("laser_mod_active_high", true);
  p.add("laser_mod_active", device.laser_mod_level == HIGH);
  p.add("laser_monitor_v", toFixedDecimal(device.laser_monitor_v, 6));
  p.add("laser_emitting", device.laser_emitting);
  return p;
}

static FLASHMEM Payload cmd_report_pulse(const Payload& /*args*/) {
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);
  interrupt_photodiode_diag_t interrupt_diag{};
  (void)interrupt_photodiode_snapshot(&interrupt_diag);

  photons_pulse_receive_value_t pulse_receive{};
  (void)photons_pulse_receive_snapshot(&pulse_receive);
  const photons_pulse_launch_state_t pulse_launch = g_last_pulse_launch;

  Payload p;
  photons_prepare_operational_report(p);
  p.add("report", "PHOTONS_PULSE");
  p.add("schema", "PHOTONS_PULSE_REPORT_V3");
  p.add("race_engine_active",
        g_photons_race.active);
  p.add("interrupt_callback_count", interrupt_diag.callback_count);
  p.add("interrupt_callback_missing_count",
        interrupt_diag.callback_missing_count);
  p.add("pulse_available", pulse_launch.valid);
  if (!pulse_launch.valid) return p;

  p.add("pulse_sequence", pulse_launch.sequence);
  p.add("pulse_requested_ns", pulse_launch.requested_ns);
  p.add("pulse_target_whole_seconds", pulse_launch.target.whole_seconds);
  p.add("pulse_target_tail_cycles", pulse_launch.target.tail_cycles);
  p.add("pulse_dwt_cycles_per_second", pulse_launch.dwt_cycles_per_second);
  p.add("pulse_start_dwt", pulse_launch.start_dwt);
  p.add("pulse_end_dwt", pulse_launch.end_dwt);
  p.add("pulse_wall_cycles", pulse_launch.pulse_wall_cycles);
  p.add("pulse_wall_cycles_semantics", "WRITE_BRACKET_MODULO_2_32");
  p.add("pulse_launch_surrogate", "MOD_HIGH_WRITE");
  const bool finish_seen =
      pulse_receive.seen &&
      pulse_receive.pulse_sequence == pulse_launch.sequence;
  p.add("pulse_armed",
        g_pulse_armed_sequence == pulse_launch.sequence && !finish_seen);
  p.add("pulse_callback_delta",
        interrupt_diag.callback_count - pulse_launch.callback_count_start);
  p.add("receive_seen", finish_seen);

  if (finish_seen) {
    p.add("receive_edge_sequence", pulse_receive.edge_sequence);
    p.add("receive_pps_sequence", pulse_receive.pps_sequence);
    p.add("receive_dwt", pulse_receive.finish_dwt);
  }
  // An arbitrarily late REPORT cannot project a retained 32-bit coordinate
  // through the current GNSS anchor without risking a false wrap attribution.
  // This manual scope command reports raw evidence, not race flight science.
  p.add("flight_time_valid", false);
  p.add("timing_semantics", "RAW_DWT_SCOPE_COMMISSIONING");
  return p;
}

static FLASHMEM Payload cmd_detector_activate(const Payload& /*args*/) {
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);

  if (g_photons_recovery.publication_started ||
      g_photons_race.active) {
    Payload p;
    p.add("status", "detector_activate_rejected_instrument_running");
    return p;
  }

  // Detector commissioning may activate only the PD200T lane. Keep the new
  // active-high DRV200 modulation command at its LOW/idle level.
  photons_laser_mod_idle();

  if (!g_initialized || !g_subscription_ok) __builtin_trap();

  if (!g_interrupt_started) {
    (void)interrupt_start(interrupt_subscriber_kind_t::PHOTODIODE);

    interrupt_photodiode_diag_t started_diag{};
    if (!interrupt_photodiode_snapshot(&started_diag) ||
        !started_diag.subscribed || !started_diag.active) {
      __builtin_trap();
    }
    g_interrupt_started = true;
  }

  interrupt_photodiode_diag_t interrupt_diag{};
  if (!interrupt_photodiode_snapshot(&interrupt_diag) ||
      !interrupt_diag.subscribed || !interrupt_diag.active) {
    __builtin_trap();
  }
  if (digitalRead(LASER_MOD_PIN) != LOW ||
      g_photons_recovery.publication_started ||
      g_photons_race.active) {
    photons_laser_mod_idle();
    __builtin_trap();
  }

  Payload p;
  p.add("status", "detector_activated");
  p.add("interrupt_subscribed", true);
  p.add("interrupt_active", true);
  p.add("laser_mod_level", LOW);
  p.add("laser_mod_active_high", true);
  p.add("publication_started", false);
  p.add("race_engine_active", false);
  return p;
}

static FLASHMEM Payload cmd_init(const Payload& /*args*/) {
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);
  if (g_photons_race.active) {
    Payload p;
    p.add("status", "init_rejected_race_engine_active");
    return p;
  }
  photons_laser_initialize_hardware();
  return ok_payload();
}

static FLASHMEM Payload cmd_wave_on(const Payload& args) {
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);

  if (g_photons_race.active) {
    Payload p;
    p.add("status", "wave_on_rejected_race_engine_active");
    return p;
  }

  uint64_t interval_ns = 0ULL;
  uint64_t width_ns = 0ULL;
  if (args.has("ns") ||
      !args.has("interval") || !args.tryGetUInt64("interval", interval_ns) ||
      !args.has("width") || !args.tryGetUInt64("width", width_ns) ||
      interval_ns == 0ULL || width_ns == 0ULL || width_ns >= interval_ns) {
    Payload p;
    p.add("status", "wave_on_rejected_timing_invalid");
    p.add("error", "Use interval and width in ns: uint64 integers with 0 < width < interval; ns is retired");
    return p;
  }
  if (!g_initialized) __builtin_trap();

  photons_laser_mod_idle();
  g_pulse_armed_sequence = 0U;
  photons_memory_barrier();

  g_photons_wave_interval_ns = interval_ns;
  g_photons_wave_width_ns = width_ns;
  g_photons_wave_timer = timepop_arm(
      interval_ns,
      true,
      photons_wave_tick,
      nullptr,
      "PHOTONS_WAVE");
  if (g_photons_wave_timer == TIMEPOP_INVALID_HANDLE) {
    g_photons_wave_interval_ns = 0ULL;
    g_photons_wave_width_ns = 0ULL;
    Payload p;
    p.add("status", "wave_on_rejected_timer_unavailable");
    return p;
  }

  // Preserve the immediate first launch. Every later launch belongs to the
  // recurring TimePop interval, never to a separately scheduled falling edge.
  // Width is approximate: GPIO/loop overhead and IRQ service can extend HIGH.
  photons_wave_emit_pulse(width_ns);

  Payload p;
  p.add("status", "wave_started");
  p.add("interval_ns", interval_ns);
  p.add("width_ns", width_ns);
  p.add("timing_semantics", "TIMEPOP_INTERVAL_DWT_BUSY_WAIT_WIDTH");
  p.add("output_pin", (uint32_t)LASER_MOD_PIN);
  p.add("output_level", LOW);
  return p;
}

static FLASHMEM Payload cmd_wave_off(const Payload& /*args*/) {
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);

  const uint64_t previous_interval_ns = g_photons_wave_interval_ns;
  const uint64_t previous_width_ns = g_photons_wave_width_ns;
  const bool was_running = g_photons_wave_timer != TIMEPOP_INVALID_HANDLE;
  photons_laser_mod_idle();

  Payload p;
  p.add("status", "wave_stopped");
  p.add("was_running", was_running);
  p.add("previous_interval_ns", previous_interval_ns);
  p.add("previous_width_ns", previous_width_ns);
  p.add("output_pin", (uint32_t)LASER_MOD_PIN);
  p.add("output_level", LOW);
  return p;
}

static FLASHMEM Payload cmd_pulse(const Payload& args) {
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);

  if (g_photons_race.active) {
    Payload p;
    p.add("status", "pulse_rejected_race_engine_active");
    return p;
  }
  if (g_photons_wave_timer != TIMEPOP_INVALID_HANDLE) {
    Payload p;
    p.add("status", "pulse_rejected_wave_active");
    p.add("error", "Run PHOTONS.WAVEOFF before PULSE");
    return p;
  }

  // Manual PULSE owns one active-high MOD excursion from the LOW/idle state.
  if (!g_initialized) __builtin_trap();
  if (digitalRead(LASER_MOD_PIN) != LOW) {
    Payload p;
    p.add("status", "pulse_rejected_mod_not_idle");
    p.add("error", "Run PHOTONS.OFF before PULSE");
    return p;
  }

  // process_interrupt owns pin 34's GPIO2 remap and its ambient-level read.
  if (interrupt_photodiode_level_high()) {
    Payload p;
    p.add("status", "pulse_rejected_detector_not_quiet");
    return p;
  }

  uint64_t requested_ns = PHOTONS_PULSE_DEFAULT_NS;
  if ((args.has("ns") && !args.tryGetUInt64("ns", requested_ns)) ||
      requested_ns == 0ULL) {
    Payload p;
    p.add("status", "pulse_rejected_ns_invalid");
    p.add("error", "ns must be a positive uint64 integer");
    return p;
  }

  // Ballpark scope timing uses the current CPU/DWT rate; GNSS lock is not a
  // prerequisite. All conversion is complete before driving active-high MOD.
  const uint32_t dwt_cycles_per_second = F_CPU_ACTUAL;
  if (dwt_cycles_per_second == 0U) __builtin_trap();
  const photons_pulse_width_t width =
      photons_pulse_width(requested_ns, dwt_cycles_per_second);

  interrupt_photodiode_diag_t interrupt_before{};
  if (!interrupt_photodiode_snapshot(&interrupt_before)) {
    Payload p;
    p.add("status", "pulse_rejected_interrupt_snapshot_unavailable");
    return p;
  }
  if (!interrupt_before.subscribed || !interrupt_before.active) {
    Payload p;
    p.add("status", "pulse_rejected_detector_inactive");
    return p;
  }

  // Close the old arm first, then snapshot the ISR-owned mailbox. If an edge had
  // already entered the callback under the old arm it completes before foreground
  // resumes; any later callback observes arm=0. This gives one exact pending verdict.
  const uint32_t previous_armed_sequence = g_pulse_armed_sequence;
  g_pulse_armed_sequence = 0U;
  photons_memory_barrier();
  photons_pulse_receive_value_t previous_receive{};
  (void)photons_pulse_receive_snapshot(&previous_receive);
  const bool previous_receive_pending =
      previous_armed_sequence != 0U &&
      !(previous_receive.seen &&
        previous_receive.pulse_sequence == previous_armed_sequence);

  g_pulse_sequence++;
  if (g_pulse_sequence == 0U) g_pulse_sequence++;
  const uint32_t pulse_sequence = g_pulse_sequence;

  g_last_pulse_launch = photons_pulse_launch_state_t{};
  g_pulse_armed_sequence = pulse_sequence;
  photons_memory_barrier();

  const uint32_t start_dwt = ARM_DWT_CYCCNT;
  digitalWriteFast(LASER_MOD_PIN, HIGH);
  const uint32_t high_start = ARM_DWT_CYCCNT;
  if (width.whole_seconds == 0ULL) {
    // Keep the usual 20-100 ns path to a 32-bit DWT poll. IRQs stay enabled;
    // loop/write overhead and interruptions may extend the physical HIGH time.
    while ((uint32_t)(ARM_DWT_CYCCNT - high_start) < width.tail_cycles) {
    }
  } else {
    photons_pulse_wait_long(width, dwt_cycles_per_second, high_start);
  }
  digitalWriteFast(LASER_MOD_PIN, LOW);
  const uint32_t end_dwt = ARM_DWT_CYCCNT;
  const uint32_t pulse_wall_cycles = end_dwt - start_dwt;
  // Return to LOW/idle before any report construction or further foreground work.
  if (digitalRead(LASER_MOD_PIN) != LOW) {
    photons_laser_mod_idle();
    __builtin_trap();
  }

  g_last_pulse_launch.sequence = pulse_sequence;
  g_last_pulse_launch.requested_ns = requested_ns;
  g_last_pulse_launch.target = width;
  g_last_pulse_launch.dwt_cycles_per_second = dwt_cycles_per_second;
  g_last_pulse_launch.start_dwt = start_dwt;
  g_last_pulse_launch.end_dwt = end_dwt;
  g_last_pulse_launch.pulse_wall_cycles = pulse_wall_cycles;
  g_last_pulse_launch.callback_count_start = interrupt_before.callback_count;
  photons_memory_barrier();
  g_last_pulse_launch.valid = true;

  Payload p;
  p.add("status", "pulse_fired");
  p.add("pulse_sequence", pulse_sequence);
  p.add("pulse_requested_ns", requested_ns);
  p.add("pulse_target_whole_seconds", width.whole_seconds);
  p.add("pulse_target_tail_cycles", width.tail_cycles);
  p.add("pulse_dwt_cycles_per_second", dwt_cycles_per_second);
  p.add("pulse_wall_cycles", pulse_wall_cycles);
  p.add("pulse_wall_cycles_semantics", "WRITE_BRACKET_MODULO_2_32");
  p.add("pulse_start_dwt", start_dwt);
  p.add("pulse_end_dwt", end_dwt);
  p.add("pulse_launch_surrogate", "MOD_HIGH_WRITE");
  p.add("laser_mod_level", LOW);
  p.add("laser_mod_active_high", true);
  p.add("previous_receive_pending", previous_receive_pending);
  if (previous_receive_pending) {
    p.add("overwritten_pending_sequence", previous_armed_sequence);
  }
  return p;
}

static FLASHMEM Payload cmd_on(const Payload& /*args*/) {
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);
  if (g_photons_race.active) {
    Payload p;
    p.add("status", "on_rejected_race_engine_active");
    return p;
  }
  if (g_photons_wave_timer != TIMEPOP_INVALID_HANDLE) {
    Payload p;
    p.add("status", "on_rejected_wave_active");
    p.add("error", "Run PHOTONS.WAVEOFF before ON");
    return p;
  }
  if (!g_initialized) __builtin_trap();

  g_pulse_armed_sequence = 0U;
  photons_memory_barrier();
  digitalWriteFast(LASER_MOD_PIN, HIGH);
  if (digitalRead(LASER_MOD_PIN) != HIGH) {
    photons_laser_mod_idle();
    __builtin_trap();
  }

  Payload p;
  p.add("status", "modulation_on");
  p.add("laser_mod_level", HIGH);
  p.add("laser_mod_active_high", true);
  p.add("driver_bias_controlled_by_firmware", false);
  return p;
}

static FLASHMEM Payload cmd_off(const Payload& /*args*/) {
  const photons_foreground_custody_t custody(
      photons_foreground_owner_t::COMMAND);
  if (g_photons_race.active) {
    Payload p;
    p.add("status", "off_rejected_race_engine_active");
    return p;
  }
  if (!g_initialized) __builtin_trap();

  g_pulse_armed_sequence = 0U;
  photons_memory_barrier();
  photons_laser_mod_idle();
  if (digitalRead(LASER_MOD_PIN) != LOW) __builtin_trap();

  Payload p;
  p.add("status", "modulation_off");
  p.add("laser_mod_level", LOW);
  p.add("laser_mod_active_high", true);
  p.add("driver_bias_controlled_by_firmware", false);
  return p;
}

// ============================================================================
// Registration
// ============================================================================

static const process_command_entry_t PHOTONS_COMMANDS[] = {
  { "REPORT_HISTOGRAM",    cmd_report_histogram    },
  { "INIT",                cmd_init                },
  { "DETECTOR_ACTIVATE",   cmd_detector_activate   },
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
  { "WAVEON",              cmd_wave_on             },
  { "WAVEOFF",             cmd_wave_off            },
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
