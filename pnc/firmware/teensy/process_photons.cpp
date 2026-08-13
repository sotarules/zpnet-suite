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
#include <math.h>
#include <stdio.h>
#include <string.h>

// ============================================================================
// PHOTONS scaffold doctrine
// ============================================================================
//
// process_interrupt owns the physical PD200T comparator interrupt and the
// first-instruction DWT coordinate.
//
// PHOTONS consumes that immutable edge fact through the specialized high-rate
// PHOTODIODE subscription.  The ISR callback below remains intentionally tiny:
// scalar/ring updates only, no Payload, no publication, no CLOCKS/TIME call,
// no floating-point statistics, and no TimePop mutation.
//
// Once per second a normal foreground TimePop callback drains completed raw lap
// records, projects each pair of observed DWT endpoints through the cached
// PPS/VCLOCK GNSS ruler, advances canonical Welford/ratio state, and publishes
// PHOTONS_FRAGMENT_V1.  Physical/raw evidence is never erased by interpretation.
// ============================================================================

static constexpr uint64_t PHOTONS_FRAGMENT_PERIOD_NS = 1000000000ULL;
static constexpr uint64_t PHOTONS_NS_PER_SECOND = 1000000000ULL;

// Bring-up raw-lap handoff.  At the current 10 ms emulator cadence this holds
// several seconds of completed laps.  A future high-rate PD200T engine may
// replace this ring with a batch sufficient-statistics path; overflow is
// explicit science data loss and therefore invalidates the lifetime stats.
static constexpr uint32_t PHOTONS_LAP_RING_CAPACITY = 256U;
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
// endpoint, so only the historical edge is minute-granular.
static constexpr uint32_t PHOTONS_PPB_MINUTE_10_SECONDS = 10U * 60U;
static constexpr uint32_t PHOTONS_PPB_MINUTE_60_SECONDS = 60U * 60U;
static constexpr uint32_t PHOTONS_PPB_HOUR_8_SECONDS = 8U * 60U * 60U;
static constexpr uint32_t PHOTONS_PPB_HOUR_24_SECONDS = 24U * 60U * 60U;
static constexpr uint32_t PHOTONS_PPB_SECOND_CAPACITY =
    PHOTONS_PPB_MINUTE_10_SECONDS + 1U;
static constexpr uint32_t PHOTONS_PPB_MINUTE_CAPACITY = 24U * 60U + 2U;


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


// ============================================================================
// Temporary PD200T physical emulator
// ============================================================================
//
// Until the Koheron PD200T arrives, PHOTONS runs an always-on physical emulator
// below the interrupt custody boundary:
//
//   pin 33 OUTPUT --> physical jumper --> pin 34 PHOTODIODE_EDGE_PIN
//
// PHOTONS never calls its own detector callback.  Every emulated detector edge
// is a real electrical rising edge on pin 34 and therefore enters through
// process_interrupt exactly as the future PD200T comparator will.
//
// The optical launch itself is real: LD_ON is pulsed for approximately 20 ns.
// One ordinary recurring TimePop timer provides only the coarse 10 ms physical
// edge cadence.  Foreground scheduling latency is NOT optical propagation.
// For emulator-only science, each completed physical loopback edge is paired
// with a synthetic start coordinate whose DWT separation is STANDARD_LAP_NS
// converted through the current GNSS/DWT ruler plus a tiny zero-mean cycle
// dither.  Physical edge timing remains visible in emulator/capture telemetry.
//
// One emulator train is fixed and explicit:
//
//   laser pulse
//   hit 1  -- physical pin-33 -> pin-34 edge; ignored by lap semantics
//   hit 2  -- lap start
//   hit 3  -- lap 1 end
//   hit 4  -- lap 2 end
//   hit 5  -- lap 3 end
//   dead lap -- one full synthetic-lap delay with no detector edge
//   repeat at the next laser pulse
//
// Each cadence tick advances exactly one state.  Hit 1 is emitted immediately
// after the real laser pulse; hit 2 follows on the next cadence tick.  After hit 5,
// the next tick is the dead lap: no detector edge is emitted and the following
// train begins on that same tick.  Every hit still traverses the physical pin-33
// jumper and real pin-34 ISR.
//
// PD200T MON on pin 38/A14 is not sampled while this emulator is installed.
// Instead a plausible synthetic ADC value is maintained as slow telemetry.
// Delete this entire block when the physical PD200T becomes authoritative.
// ============================================================================

static constexpr bool PHOTONS_EMULATOR_ENABLED = true;
static constexpr int PHOTONS_EMULATOR_EDGE_OUTPUT_PIN = 33;
static constexpr uint32_t PHOTONS_EMULATOR_LASER_PULSE_CYCLES = 20U;
static constexpr uint32_t PHOTONS_EMULATOR_CADENCE_NS = 10000000U;  // synthetic 10 ms
static constexpr uint32_t PHOTONS_EMULATOR_HITS_PER_TRAIN = 5U;
static constexpr uint32_t PHOTONS_EMULATOR_MEASURED_LAPS = 3U;

// Temporary standard-locked science jitter.  One DWT cycle is approximately
// one nanosecond on this build.  The pattern is deliberately tiny and has an
// exact zero sum, so it exercises per-lap timing variation without building a
// fake frequency offset into Better-Buckets.
static constexpr int8_t PHOTONS_EMULATOR_LAP_JITTER_CYCLES[] = {
  -3, 1, 2, -1, 3, -2, 0
};
static constexpr uint32_t PHOTONS_EMULATOR_LAP_JITTER_COUNT =
    sizeof(PHOTONS_EMULATOR_LAP_JITTER_CYCLES) /
    sizeof(PHOTONS_EMULATOR_LAP_JITTER_CYCLES[0]);
static_assert((-3 + 1 + 2 - 1 + 3 - 2 + 0) == 0,
              "PHOTONS emulator lap jitter must be zero-mean");

// Provisional PD200T MON range for bring-up only.  1200-1800 ADC counts maps to
// roughly 0.97-1.45 V on the Teensy 3.3 V / 12-bit ADC scale.
static constexpr uint16_t PHOTONS_EMULATOR_MON_RAW_MIN = 1200U;
static constexpr uint16_t PHOTONS_EMULATOR_MON_RAW_MAX = 1800U;

enum class photons_emulator_stage_t : uint8_t {
  IDLE = 0,
  WAIT_HIT2,
  WAIT_HIT3,
  WAIT_HIT4,
  WAIT_HIT5,
  DEAD_LAP,
};


struct photons_emulator_state_t {
  bool initialized = false;
  bool train_active = false;

  timepop_handle_t cadence_timer = TIMEPOP_INVALID_HANDLE;
  uint32_t cadence_timer_arm_count = 0U;
  uint32_t cadence_timer_arm_fail_count = 0U;
  uint32_t cadence_tick_count = 0U;

  photons_emulator_stage_t stage = photons_emulator_stage_t::IDLE;
  uint32_t train_count = 0U;
  uint32_t relaunch_count = 0U;

  uint32_t laser_pulse_count = 0U;
  uint32_t last_laser_pulse_cycles = 0U;
  uint32_t max_laser_pulse_cycles = 0U;

  uint32_t generated_edge_count = 0U;
  uint32_t expected_hit_ordinal = 0U;
  uint32_t unexpected_edge_count = 0U;
  uint32_t state_error_count = 0U;

  uint32_t hit1_count = 0U;
  uint32_t hit2_count = 0U;
  uint32_t hit3_count = 0U;
  uint32_t hit4_count = 0U;
  uint32_t hit5_count = 0U;

  uint32_t lap_start_count = 0U;
  uint32_t lap1_complete_count = 0U;
  uint32_t lap2_complete_count = 0U;
  uint32_t lap3_complete_count = 0U;
  uint32_t dead_lap_count = 0U;

  uint32_t lap_start_dwt = 0U;
  uint32_t previous_lap_endpoint_dwt = 0U;
  // Physical TimePop-to-TimePop loopback spacing.  These remain telemetry only;
  // emulator science below is standard-locked and does not consume them.
  uint32_t lap1_last_cycles = 0U;
  uint32_t lap2_last_cycles = 0U;
  uint32_t lap3_last_cycles = 0U;

  bool synthetic_lap_ready = false;
  uint32_t synthetic_target_cycles = 0U;
  int32_t synthetic_last_jitter_cycles = 0;
  uint32_t synthetic_last_lap_cycles = 0U;
  uint32_t synthetic_lap_count = 0U;
  uint32_t synthetic_jitter_index = 0U;
  uint32_t synthetic_target_unavailable_count = 0U;
  uint32_t projection_wait_count = 0U;

  uint16_t mon_raw = PHOTONS_EMULATOR_MON_RAW_MIN;
};


static photons_emulator_state_t g_photons_emulator{};

static void photons_emulator_cadence_tick(
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
  uint16_t photodiode_mon_raw = 0;
  float    photodiode_mon_v = 0.0f;
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

// Required metrological authority installed by SET_STANDARD_LAP_NS before the
// fragment/statistics clock begins.
static bool g_standard_lap_configured = false;
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

// One-shot diagnostic fault injection.  The current implementation is deliberately
// emulator-only: it enlarges one synthetic raw lap by 25%, causing the ordinary
// RAW_CYCLE_EXCURSION court to fire while physical pin/ISR custody stays real.
static volatile uint32_t g_photons_problem_injection_armed = 0U;
static uint32_t g_photons_problem_injection_arm_count = 0U;
static uint32_t g_photons_problem_injection_fire_count = 0U;
static uint32_t g_photons_problem_injection_last_extra_cycles = 0U;

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
static uint32_t g_photons_ppb_current_sequence = 0U;
static bool g_photons_ppb_endpoint_admitted = false;
static bool g_photons_ppb_interval_advanced = false;

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


template <size_t N>
static bool photons_ppb_ring_find_anchor(
    const photons_ppb_endpoint_t (&ring)[N],
    uint32_t head,
    uint32_t count,
    uint32_t target_sequence,
    uint32_t current_sequence,
    photons_ppb_endpoint_t& out) {
  if (count > (uint32_t)N) __builtin_trap();

  for (uint32_t offset = 0U; offset < count; offset++) {
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


static uint32_t photons_ppb_minute_key(uint32_t sequence) {
  return sequence == 0U ? 0U : ((sequence - 1U) / 60U) + 1U;
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


static double photons_ppb_from_population(uint64_t total_lap_gnss_ns,
                                          uint64_t lap_count) {
  if (!g_standard_lap_configured || g_standard_lap_ps == 0ULL ||
      lap_count == 0ULL || total_lap_gnss_ns == 0ULL) {
    __builtin_trap();
  }

  // Divide first so TOTAL remains numerically well behaved after years of
  // accumulation.  The quotient/remainder form avoids multiplying the lifetime
  // nanosecond numerator by 1000 in uint64_t.
  const uint64_t whole_ns = total_lap_gnss_ns / lap_count;
  const uint64_t remainder_ns = total_lap_gnss_ns % lap_count;
  const double observed_mean_ps =
      (double)whole_ns * 1000.0 +
      ((double)remainder_ns * 1000.0) / (double)lap_count;
  return (observed_mean_ps / (double)g_standard_lap_ps - 1.0) * 1.0e9;
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
  return out;
}


static void photons_ppb_windows_note_endpoint(uint32_t sequence,
                                              bool admitted,
                                              uint64_t lap_count,
                                              uint64_t total_lap_gnss_ns) {
  g_photons_ppb_endpoint_admitted = admitted;
  g_photons_ppb_interval_advanced = false;

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
    const photons_ppb_endpoint_t& current) {
  const uint32_t target_sequence =
      current.sequence > window_seconds
          ? current.sequence - window_seconds
          : 0U;
  photons_ppb_endpoint_t anchor{};
  if (!photons_ppb_ring_find_anchor(
          ring, head, count, target_sequence, current.sequence, anchor)) {
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
      current);
  out.minute_60 = photons_ppb_window_snapshot(
      g_photons_ppb_minutes,
      g_photons_ppb_minutes_head,
      g_photons_ppb_minutes_count,
      PHOTONS_PPB_MINUTE_60_SECONDS,
      current);
  out.hour_8 = photons_ppb_window_snapshot(
      g_photons_ppb_minutes,
      g_photons_ppb_minutes_head,
      g_photons_ppb_minutes_count,
      PHOTONS_PPB_HOUR_8_SECONDS,
      current);
  out.hour_24 = photons_ppb_window_snapshot(
      g_photons_ppb_minutes,
      g_photons_ppb_minutes_head,
      g_photons_ppb_minutes_count,
      PHOTONS_PPB_HOUR_24_SECONDS,
      current);

  if (current.lap_count != 0ULL) {
    out.total.sample_count = current.lap_count;
    out.total.ppb = photons_ppb_from_population(
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
  out.public_count = g_photons_campaign_public_count + 1U;
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
    if (fragment.campaign.public_count !=
        g_photons_campaign_public_count + 1U ||
        strcmp(fragment.campaign.campaign, g_photons_campaign_name) != 0) {
      __builtin_trap();
    }

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
  if (!g_standard_lap_configured || g_standard_lap_ps == 0ULL) __builtin_trap();

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
  __atomic_store_n(&g_photons_problem_injection_armed, 0U, __ATOMIC_RELEASE);

  g_photons_stats_reset_pending = false;
  g_photons_stats_reset_commit_count++;
}


static void photons_stats_reset_commit_after_publish(void) {
  if (!g_photons_stats_reset_pending) return;
  photons_instrument_statistics_reset_commit();
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


static bool photons_raw_lap_enqueue(uint32_t start_dwt,
                                    uint32_t end_dwt,
                                    uint32_t raw_cycles,
                                    uint32_t pps_sequence) {
  const uint32_t write = g_raw_lap_ring_write;
  const uint32_t next = (write + 1U) & PHOTONS_LAP_RING_MASK;

  if (next == g_raw_lap_ring_read) {
    g_raw_lap_ring_overflow_count++;
    g_raw_lap_ring_data_loss = true;
    return false;
  }

  photons_projection_anchor_value_t anchor{};
  const bool anchor_snapshot_ok =
      photons_projection_anchor_snapshot(anchor);

  photons_raw_lap_record_t record{};
  record.start_dwt = start_dwt;
  record.end_dwt = end_dwt;
  record.raw_cycles = raw_cycles;
  record.pps_sequence = pps_sequence;
  record.anchor_valid = anchor_snapshot_ok && anchor.valid;
  record.anchor_dwt_at_pps_vclock = anchor.dwt_at_pps_vclock;
  record.anchor_dwt_cycles_per_second = anchor.dwt_cycles_per_second;
  record.anchor_pps_count = anchor.pps_count;

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


static void photons_lap_science_exclude(
    const photons_lap_science_candidate_t& candidate,
    photons_lap_science_exclusion_reason_t reason,
    bool projection_valid,
    uint32_t prediction_cycles,
    int32_t residual_cycles,
    uint32_t gate_cycles) {
  g_photons_lap_science_state.excluded.count++;
  g_photons_lap_science_state.excluded.count_this_fragment++;
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
};


static photons_fragment_drain_result_t photons_drain_raw_laps(void) {
  photons_fragment_drain_result_t result{};

  g_photons_lap_science_state.candidates_this_fragment = 0U;
  g_photons_lap_science_state.accepted.count_this_fragment = 0U;
  g_photons_lap_science_state.excluded.count_this_fragment = 0U;
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


static const char* photons_emulator_stage_name(
    photons_emulator_stage_t stage) {
  switch (stage) {
    case photons_emulator_stage_t::IDLE:      return "IDLE";
    case photons_emulator_stage_t::WAIT_HIT2: return "WAIT_HIT2";
    case photons_emulator_stage_t::WAIT_HIT3: return "WAIT_HIT3";
    case photons_emulator_stage_t::WAIT_HIT4: return "WAIT_HIT4";
    case photons_emulator_stage_t::WAIT_HIT5: return "WAIT_HIT5";
    case photons_emulator_stage_t::DEAD_LAP:  return "DEAD_LAP";
    default:                                  return "UNKNOWN";
  }
}


static void photons_emulator_refresh_mon(void) {
  // Slow synthetic telemetry only.  Deterministic bounded variation avoids
  // touching an ADC merely to make the temporary PD200T MON surface move.
  static uint16_t step = 0U;
  const uint16_t span =
      PHOTONS_EMULATOR_MON_RAW_MAX - PHOTONS_EMULATOR_MON_RAW_MIN + 1U;
  step = (uint16_t)((step + 137U) % span);
  g_photons_emulator.mon_raw =
      (uint16_t)(PHOTONS_EMULATOR_MON_RAW_MIN + step);
}

static bool photons_emulator_projection_ready(void) {
  if (!g_standard_lap_configured || g_standard_lap_ps == 0ULL) {
    __builtin_trap();
  }

  photons_projection_anchor_value_t anchor{};
  return photons_projection_anchor_snapshot(anchor) &&
         anchor.valid &&
         anchor.dwt_cycles_per_second != 0U;
}


static bool photons_emulator_prepare_synthetic_lap(void) {
  if (!g_standard_lap_configured || g_standard_lap_ps == 0ULL) {
    __builtin_trap();
  }

  photons_projection_anchor_value_t anchor{};
  if (!photons_projection_anchor_snapshot(anchor) ||
      !anchor.valid ||
      anchor.dwt_cycles_per_second == 0U) {
    g_photons_emulator.synthetic_lap_ready = false;
    g_photons_emulator.synthetic_target_unavailable_count++;
    return false;
  }

  // This floating conversion occurs in the ordinary foreground TimePop callback,
  // never in the PHOTODIODE ISR callback.  Double precision is far below one
  // DWT cycle at this scale; the rounded integer is what crosses into ISR state.
  const double exact_target_cycles =
      ((double)g_standard_lap_ps *
       (double)anchor.dwt_cycles_per_second) / 1.0e12;
  if (!(exact_target_cycles >= 1.0) ||
      exact_target_cycles > 4294967290.0) {
    __builtin_trap();
  }

  const uint32_t target_cycles = (uint32_t)(exact_target_cycles + 0.5);
  const int32_t jitter_cycles =
      (int32_t)PHOTONS_EMULATOR_LAP_JITTER_CYCLES[
          g_photons_emulator.synthetic_jitter_index %
          PHOTONS_EMULATOR_LAP_JITTER_COUNT];
  g_photons_emulator.synthetic_jitter_index++;

  int64_t lap_cycles =
      (int64_t)(uint64_t)target_cycles + (int64_t)jitter_cycles;

  uint32_t injection_state = 2U;
  if (__atomic_compare_exchange_n(&g_photons_problem_injection_armed,
                                  &injection_state,
                                  0U,
                                  false,
                                  __ATOMIC_ACQ_REL,
                                  __ATOMIC_ACQUIRE)) {
    const uint32_t extra_cycles = target_cycles / 4U;
    if (extra_cycles == 0U) __builtin_trap();
    lap_cycles += (int64_t)(uint64_t)extra_cycles;
    g_photons_problem_injection_last_extra_cycles = extra_cycles;
    g_photons_problem_injection_fire_count++;
  }

  if (lap_cycles <= 0LL || lap_cycles > 4294967295LL) {
    __builtin_trap();
  }

  g_photons_emulator.synthetic_target_cycles = target_cycles;
  g_photons_emulator.synthetic_last_jitter_cycles = jitter_cycles;
  g_photons_emulator.synthetic_last_lap_cycles = (uint32_t)lap_cycles;
  g_photons_emulator.synthetic_lap_ready = true;
  return true;
}


static void photons_emulator_enqueue_synthetic_lap(
    const interrupt_photodiode_edge_t& edge) {
  if (!g_photons_emulator.synthetic_lap_ready ||
      g_photons_emulator.synthetic_last_lap_cycles == 0U) {
    __builtin_trap();
  }

  // Anchor the synthetic interval at the real physical loopback edge.  Only the
  // start coordinate is moved.  The raw-lap handoff and downstream projection
  // therefore exercise the normal PHOTONS path with a realistic tiny interval
  // error while the actual TimePop spacing remains separately observable.
  const uint32_t end_dwt = edge.dwt_at_edge;
  const uint32_t raw_cycles = g_photons_emulator.synthetic_last_lap_cycles;
  const uint32_t start_dwt = end_dwt - raw_cycles;
  if (photons_raw_lap_enqueue(
          start_dwt, end_dwt, raw_cycles, edge.pps_sequence)) {
    g_photons_emulator.synthetic_lap_count++;
  }
  g_photons_emulator.synthetic_lap_ready = false;
}



static void photons_emulator_emit_detector_edge(uint32_t hit_ordinal) {
  // The ordinal is emulator intent only.  The resulting observation is still a
  // real electrical edge on pin 34 whose timestamp belongs to process_interrupt.
  g_photons_emulator.expected_hit_ordinal = hit_ordinal;
  digitalWriteFast(PHOTONS_EMULATOR_EDGE_OUTPUT_PIN, HIGH);
  digitalWriteFast(PHOTONS_EMULATOR_EDGE_OUTPUT_PIN, LOW);
  g_photons_emulator.generated_edge_count++;
}

static void photons_emulator_emit_laser_pulse(void) {
  // DWT runs at approximately 1.008 GHz, so twenty cycles is approximately
  // 19.8 ns.  Measure the complete commanded HIGH/LOW transaction, not merely
  // the busy-wait body, so REPORT shows the actual software pulse wall time.
  const uint32_t pulse_start = ARM_DWT_CYCCNT;
  digitalWriteFast(LD_ON_PIN, HIGH);
  const uint32_t high_start = ARM_DWT_CYCCNT;
  while ((uint32_t)(ARM_DWT_CYCCNT - high_start) <
         PHOTONS_EMULATOR_LASER_PULSE_CYCLES) {
  }
  digitalWriteFast(LD_ON_PIN, LOW);
  const uint32_t pulse_cycles = ARM_DWT_CYCCNT - pulse_start;

  g_photons_emulator.laser_pulse_count++;
  g_photons_emulator.last_laser_pulse_cycles = pulse_cycles;
  if (pulse_cycles > g_photons_emulator.max_laser_pulse_cycles) {
    g_photons_emulator.max_laser_pulse_cycles = pulse_cycles;
  }
}

static void photons_emulator_begin_train(void) {
  if (g_photons_emulator.train_count != 0U) {
    g_photons_emulator.relaunch_count++;
  }

  g_photons_emulator.train_count++;
  g_photons_emulator.train_active = true;
  g_photons_emulator.stage = photons_emulator_stage_t::WAIT_HIT2;
  g_photons_emulator.expected_hit_ordinal = 0U;
  g_photons_emulator.lap_start_dwt = 0U;
  g_photons_emulator.previous_lap_endpoint_dwt = 0U;

  photons_emulator_refresh_mon();

  // Step 1: real laser pulse.
  photons_emulator_emit_laser_pulse();

  // Step 2: hit 1.  It remains a real pin-34 observation but is intentionally
  // ignored as a lap endpoint by photons_emulator_observe_edge().
  photons_emulator_emit_detector_edge(1U);
}


static void photons_emulator_cadence_tick(
    timepop_ctx_t* /*ctx*/,
    timepop_diag_t* /*diag*/,
    void* /*user_data*/) {
  g_photons_emulator.cadence_tick_count++;

  switch (g_photons_emulator.stage) {
    case photons_emulator_stage_t::IDLE:
      // Do not begin a synthetic train until the normal PHOTONS projection ruler
      // exists.  This avoids manufacturing a startup population that is known
      // to be unprojectable.
      if (!photons_emulator_projection_ready()) {
        g_photons_emulator.projection_wait_count++;
        return;
      }
      photons_emulator_begin_train();
      return;

    case photons_emulator_stage_t::WAIT_HIT2:
      photons_emulator_refresh_mon();
      photons_emulator_emit_detector_edge(2U);
      g_photons_emulator.stage = photons_emulator_stage_t::WAIT_HIT3;
      return;

    case photons_emulator_stage_t::WAIT_HIT3:
      photons_emulator_refresh_mon();
      if (!photons_emulator_prepare_synthetic_lap()) return;
      photons_emulator_emit_detector_edge(3U);
      g_photons_emulator.stage = photons_emulator_stage_t::WAIT_HIT4;
      return;

    case photons_emulator_stage_t::WAIT_HIT4:
      photons_emulator_refresh_mon();
      if (!photons_emulator_prepare_synthetic_lap()) return;
      photons_emulator_emit_detector_edge(4U);
      g_photons_emulator.stage = photons_emulator_stage_t::WAIT_HIT5;
      return;

    case photons_emulator_stage_t::WAIT_HIT5:
      photons_emulator_refresh_mon();
      if (!photons_emulator_prepare_synthetic_lap()) return;
      photons_emulator_emit_detector_edge(5U);
      g_photons_emulator.train_active = false;
      g_photons_emulator.stage = photons_emulator_stage_t::DEAD_LAP;
      return;

    case photons_emulator_stage_t::DEAD_LAP:
      // This cadence cell deliberately emits no detector edge.  It represents
      // the missing fourth lap, then starts the next train immediately.
      g_photons_emulator.dead_lap_count++;
      photons_emulator_begin_train();
      return;

    default:
      g_photons_emulator.state_error_count++;
      g_photons_emulator.train_active = false;
      g_photons_emulator.expected_hit_ordinal = 0U;
      g_photons_emulator.stage = photons_emulator_stage_t::IDLE;
      return;
  }
}


static void photons_emulator_prepare(void) {
  g_photons_emulator = photons_emulator_state_t{};

  pinMode(PHOTONS_EMULATOR_EDGE_OUTPUT_PIN, OUTPUT);
  digitalWriteFast(PHOTONS_EMULATOR_EDGE_OUTPUT_PIN, LOW);
  photons_emulator_refresh_mon();
  g_photons_emulator.initialized = true;
}

static void photons_emulator_start(void) {
  g_photons_emulator.cadence_timer_arm_count++;
  g_photons_emulator.cadence_timer =
      timepop_arm(
          PHOTONS_EMULATOR_CADENCE_NS,
          true,
          photons_emulator_cadence_tick,
          nullptr,
          "PHOTONS_EMULATOR_CADENCE");
  if (g_photons_emulator.cadence_timer == TIMEPOP_INVALID_HANDLE) {
    g_photons_emulator.cadence_timer_arm_fail_count++;
  }
}

static void photons_laser_initialize_hardware(void) {
  pinMode(LD_ON_PIN, OUTPUT);
  photons_laser_inhibit();

  pinMode(LASER_MONITOR_PIN, INPUT);
  pinMode(PHOTODIODE_MON_PIN, INPUT);
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

  // PD200T MON is emulated until the physical detector arrives.  Do not read
  // pin 38/A14 in this bring-up configuration.
  out.photodiode_mon_raw = g_photons_emulator.mon_raw;
  out.photodiode_mon_v = photons_adc_voltage(out.photodiode_mon_raw);

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
static uint32_t g_fragment_sequence = 0;
static uint32_t g_publish_count = 0;
static uint32_t g_publish_reject_count = 0;
static uint32_t g_last_published_edge_count = 0;

static bool g_initialized = false;
static bool g_subscription_ok = false;
static bool g_interrupt_started = false;

static timepop_handle_t g_fragment_timer = TIMEPOP_INVALID_HANDLE;

static inline void photons_compiler_barrier(void) {
  __asm__ volatile("" ::: "memory");
}

static void photons_emulator_observe_edge(
    const interrupt_photodiode_edge_t& edge) {
  if (!PHOTONS_EMULATOR_ENABLED) return;

  const uint32_t hit = g_photons_emulator.expected_hit_ordinal;
  g_photons_emulator.expected_hit_ordinal = 0U;

  switch (hit) {
    case 1U:
      g_photons_emulator.hit1_count++;
      // Deliberately ignored for lap measurement.
      return;

    case 2U:
      g_photons_emulator.hit2_count++;
      g_photons_emulator.lap_start_count++;
      g_photons_emulator.lap_start_dwt = edge.dwt_at_edge;
      g_photons_emulator.previous_lap_endpoint_dwt = edge.dwt_at_edge;
      return;

    case 3U:
      g_photons_emulator.hit3_count++;
      if (g_photons_emulator.previous_lap_endpoint_dwt == 0U) {
        g_photons_emulator.unexpected_edge_count++;
        return;
      }
      {
        const uint32_t start_dwt =
            g_photons_emulator.previous_lap_endpoint_dwt;
        const uint32_t raw_cycles = edge.dwt_at_edge - start_dwt;
        g_photons_emulator.lap1_last_cycles = raw_cycles;
        photons_emulator_enqueue_synthetic_lap(edge);
      }
      g_photons_emulator.previous_lap_endpoint_dwt = edge.dwt_at_edge;
      g_photons_emulator.lap1_complete_count++;
      return;

    case 4U:
      g_photons_emulator.hit4_count++;
      if (g_photons_emulator.previous_lap_endpoint_dwt == 0U) {
        g_photons_emulator.unexpected_edge_count++;
        return;
      }
      {
        const uint32_t start_dwt =
            g_photons_emulator.previous_lap_endpoint_dwt;
        const uint32_t raw_cycles = edge.dwt_at_edge - start_dwt;
        g_photons_emulator.lap2_last_cycles = raw_cycles;
        photons_emulator_enqueue_synthetic_lap(edge);
      }
      g_photons_emulator.previous_lap_endpoint_dwt = edge.dwt_at_edge;
      g_photons_emulator.lap2_complete_count++;
      return;

    case 5U:
      g_photons_emulator.hit5_count++;
      if (g_photons_emulator.previous_lap_endpoint_dwt == 0U) {
        g_photons_emulator.unexpected_edge_count++;
        return;
      }
      {
        const uint32_t start_dwt =
            g_photons_emulator.previous_lap_endpoint_dwt;
        const uint32_t raw_cycles = edge.dwt_at_edge - start_dwt;
        g_photons_emulator.lap3_last_cycles = raw_cycles;
        photons_emulator_enqueue_synthetic_lap(edge);
      }
      g_photons_emulator.previous_lap_endpoint_dwt = edge.dwt_at_edge;
      g_photons_emulator.lap3_complete_count++;
      return;

    default:
      // A physical photodiode edge without an emulator-authored role is still
      // preserved by the ordinary PHOTONS capture path below.  It is merely
      // unexpected with respect to this temporary state machine.
      g_photons_emulator.unexpected_edge_count++;
      return;
  }
}


// ============================================================================
// High-rate PHOTODIODE subscriber
// ============================================================================

static void photons_on_photodiode_edge(
    const interrupt_photodiode_edge_t& edge,
    const interrupt_photodiode_diag_t& /*diag*/,
    void* /*user_data*/) {

  // Begin seqlock write: odd generation means foreground must retry.
  g_photons_live.generation++;
  photons_compiler_barrier();

  photons_toy_capture_t& c = g_photons_live.capture;

  photons_emulator_observe_edge(edge);

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

// Reused RAM2 Payload headers.  CLOCKS learned the hard way that rich nested
// diagnostics do not belong on the small DTCM stack.
static Payload g_photons_fragment_root DMAMEM;
static Payload g_photons_fragment_instrument DMAMEM;
static Payload g_photons_fragment_raw_cycles DMAMEM;
static Payload g_photons_fragment_projection DMAMEM;
static Payload g_photons_fragment_science DMAMEM;
static Payload g_photons_fragment_science_accepted DMAMEM;
static Payload g_photons_fragment_science_excluded DMAMEM;
static Payload g_photons_fragment_science_reasons DMAMEM;
static Payload g_photons_fragment_stats DMAMEM;
static Payload g_photons_fragment_welford DMAMEM;
static Payload g_photons_fragment_ppb_buckets DMAMEM;
static Payload g_photons_fragment_ppb_value DMAMEM;
static Payload g_photons_fragment_campaign DMAMEM;
static Payload g_photons_fragment_campaign_stats DMAMEM;
static Payload g_photons_fragment_baseline DMAMEM;
static Payload g_photons_fragment_interrupt DMAMEM;


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

  Payload& obj = g_photons_fragment_ppb_value;
  obj.clear();
  obj.add("sample_count", value.sample_count);
  obj.add("ppb", toFixedDecimal(value.ppb, 6));
  parent.add_object(name, obj);
  obj.clear();
}


static Payload& photons_fragment_payload(
    const photons_fragment_snapshot_t& f) {
  Payload& root = g_photons_fragment_root;
  Payload& instrument = g_photons_fragment_instrument;
  Payload& raw = g_photons_fragment_raw_cycles;
  Payload& projection = g_photons_fragment_projection;
  Payload& science = g_photons_fragment_science;
  Payload& accepted = g_photons_fragment_science_accepted;
  Payload& excluded = g_photons_fragment_science_excluded;
  Payload& reasons = g_photons_fragment_science_reasons;
  Payload& stats = g_photons_fragment_stats;
  Payload& ppb_buckets = g_photons_fragment_ppb_buckets;
  Payload& campaign = g_photons_fragment_campaign;
  Payload& campaign_stats = g_photons_fragment_campaign_stats;
  Payload& baseline = g_photons_fragment_baseline;
  Payload& interrupt = g_photons_fragment_interrupt;

  root.clear();
  instrument.clear();
  raw.clear();
  projection.clear();
  science.clear();
  accepted.clear();
  excluded.clear();
  reasons.clear();
  stats.clear();
  ppb_buckets.clear();
  campaign.clear();
  campaign_stats.clear();
  baseline.clear();
  interrupt.clear();

  root.add("schema", "PHOTONS_FRAGMENT_V1");
  root.add("sequence", f.sequence);
  root.add("publish_count", f.publish_count);

  instrument.add("schema", "PHOTONS_INSTRUMENT_V1");
  instrument.add("snapshot_ok", f.snapshot_ok);
  instrument.add("valid", f.valid);
  instrument.add("fragment_period_ns", f.fragment_period_ns);
  instrument.add("source",
                 PHOTONS_EMULATOR_ENABLED ? "EMULATOR" : "PD200T");
  instrument.add("edge_count_total", f.edge_count_total);
  instrument.add("edges_this_fragment", f.edges_this_fragment);
  instrument.add("train_count", f.train_count);
  instrument.add("dead_lap_count", f.dead_lap_count);
  instrument.add("raw_lap_count", f.raw_lap_count);
  instrument.add("projected_laps_this_fragment",
                 f.projected_laps_this_fragment);

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

  excluded.add("count", f.science.excluded.count);
  excluded.add("count_this_fragment",
               f.science.excluded.count_this_fragment);
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
  stats.add("valid", f.stats.valid);
  stats.add("reset_count", f.stats.reset_count);
  stats.add("update_count", f.stats.update_count);
  stats.add("standard_lap_ps", f.stats.standard_lap_ps);
  stats.add("standard_lap_ns",
            toFixedDecimal((double)f.stats.standard_lap_ps / 1000.0, 3));
  stats.add("custody_lap_count", g_photons_custody_lap_count);
  stats.add("custody_total_lap_gnss_ns", g_photons_custody_total_lap_gnss_ns);
  stats.add("lap_count", f.stats.lap_count);
  stats.add("total_lap_gnss_ns", f.stats.total_lap_gnss_ns);
  stats.add("mean_lap_ns", toFixedDecimal(f.stats.mean_lap_ns, 6));
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

  instrument.add_object("stats", stats);
  stats.clear();

  baseline.add("present", f.baseline.present);
  baseline.add("residual_valid", f.baseline.residual_valid);
  if (f.baseline.present) {
    baseline.add("baseline_mean_lap_ns",
                 toFixedDecimal(f.baseline.baseline_mean_lap_ns, 6));
  }
  if (f.baseline.residual_valid) {
    baseline.add("mean_residual_ps",
                 toFixedDecimal(f.baseline.mean_residual_ps, 3));
  }
  instrument.add_object("baseline", baseline);
  baseline.clear();

  interrupt.add("irq_count", f.interrupt_irq_count);
  interrupt.add("callback_count", f.interrupt_callback_count);
  interrupt.add("callback_missing_count",
                f.interrupt_callback_missing_count);
  interrupt.add("inactive_edge_count",
                f.interrupt_inactive_edge_count);
  interrupt.add("source_pin", f.interrupt_source_pin);
  interrupt.add("last_callback_wall_cycles",
                f.interrupt_last_callback_wall_cycles);
  interrupt.add("max_callback_wall_cycles",
                f.interrupt_max_callback_wall_cycles);
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
      campaign_stats.add("sample_count", f.campaign.ppb.sample_count);
      campaign_stats.add("ppb", toFixedDecimal(f.campaign.ppb.ppb, 6));
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
  if (!g_standard_lap_configured) return;

  // Refresh the PHOTONS-owned immutable copy for laps that will arrive after
  // this boundary.  Records already in the ring carry the anchor that was
  // current when their physical DWT endpoints were observed.
  photons_projection_anchor_refresh();

  photons_toy_capture_t capture{};
  if (!photons_toy_capture_snapshot(&capture)) return;

  interrupt_photodiode_diag_t interrupt_diag{};
  (void)interrupt_photodiode_snapshot(&interrupt_diag);

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

  fragment.train_count = g_photons_emulator.train_count;
  fragment.dead_lap_count = g_photons_emulator.dead_lap_count;
  fragment.raw_lap_count = g_raw_cycles_state.completed_lap_count;
  fragment.projected_laps_this_fragment = drain.projected_laps;

  fragment.raw_cycles = g_raw_cycles_state;
  fragment.projection = g_projection_state;
  fragment.science = g_photons_lap_science_state;

  fragment.stats.reset_count = g_photons_stats_reset_count;
  fragment.stats.update_count = ++g_photons_stats_update_count;
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
  // ancestry.  Actual custody loss does: raw-ring overflow or a missing/inactive
  // PHOTODIODE callback clears the rolling history rather than bridging it.
  const bool ppb_endpoint_admitted =
      !g_raw_lap_ring_data_loss &&
      interrupt_diag.callback_missing_count == 0U &&
      interrupt_diag.inactive_edge_count == 0U;
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

  // Optional LANTERN campaign testimony is firmware-authored from the same
  // cumulative accepted-lap authority as TOTAL.  START/STOP never reset the
  // always-on instrument; they only select a recording-relative subtraction
  // origin, exactly like CLOCKS campaign offsets.
  fragment.campaign = photons_campaign_snapshot(fragment.sequence);

  // Baseline control/provenance is deliberately deferred.  False validity is
  // part of the canonical schema rather than a fabricated zero baseline.
  fragment.baseline = photons_fragment_baseline_snapshot_t{};

  fragment.interrupt_irq_count = interrupt_diag.irq_count;
  fragment.interrupt_callback_count = interrupt_diag.callback_count;
  fragment.interrupt_callback_missing_count =
      interrupt_diag.callback_missing_count;
  fragment.interrupt_inactive_edge_count =
      interrupt_diag.inactive_edge_count;
  fragment.interrupt_source_pin = interrupt_diag.source_pin;
  fragment.interrupt_last_callback_wall_cycles =
      interrupt_diag.last_callback_wall_cycles;
  fragment.interrupt_max_callback_wall_cycles =
      interrupt_diag.max_callback_wall_cycles;

  fragment.valid =
      fragment.stats.valid &&
      fragment.projection.anchor_cache_valid &&
      fragment.interrupt_callback_missing_count == 0U &&
      fragment.interrupt_inactive_edge_count == 0U;

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

  Payload& payload = photons_fragment_payload(fragment);
  if (publish("PHOTONS_FRAGMENT", payload)) {
    g_publish_count++;
    g_last_fragment.publish_count = g_publish_count;
    g_last_photons_fragment.publish_count = g_publish_count;
    photons_campaign_commit_after_publish(fragment);
    photons_stats_reset_commit_after_publish();
  } else {
    g_publish_reject_count++;
  }
}


// ============================================================================
// Initialization
// ============================================================================

void process_photons_init(void) {
  if (g_initialized) return;

  g_photons_live = photons_live_state_t{};
  g_last_fragment = photons_toy_fragment_t{};
  g_last_photons_fragment = photons_fragment_snapshot_t{};
  g_fragment_sequence = 0U;
  g_publish_count = 0U;
  g_publish_reject_count = 0U;
  g_last_published_edge_count = 0U;
  g_standard_lap_configured = false;
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
  __atomic_store_n(&g_photons_problem_injection_armed, 0U, __ATOMIC_RELEASE);
  g_photons_problem_injection_arm_count = 0U;
  g_photons_problem_injection_fire_count = 0U;
  g_photons_problem_injection_last_extra_cycles = 0U;

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

  photons_emulator_prepare();
  photons_laser_initialize_hardware();
  photons_emit_laser_initialization_event();

  interrupt_photodiode_subscription_t subscription{};
  subscription.on_edge = photons_on_photodiode_edge;
  subscription.user_data = nullptr;

  g_subscription_ok = interrupt_photodiode_subscribe(subscription);
  g_interrupt_started =
      g_subscription_ok &&
      interrupt_start(interrupt_subscriber_kind_t::PHOTODIODE);

  // Prime the PHOTONS-owned projection cache before the emulator begins.  TIME
  // may still be initializing; invalidity is preserved and early laps are
  // rejected rather than projected through an invented ruler.
  photons_projection_anchor_refresh();

  // The fragment publisher and temporary emulator are intentionally not started
  // here.  SET_STANDARD_LAP_NS establishes the statistics/publication origin.

  // The temporary emulator is intentionally not started here.  Starting it
  // before STANDARD_LAP_NS would create a hidden pre-configuration science
  // population.  SET_STANDARD_LAP_NS establishes the statistics origin and
  // starts the emulator exactly once.
  g_initialized = true;
}

// ============================================================================
// Commands
// ============================================================================

// Parse the Pi-authored fixed-decimal nanosecond reference exactly into
// picoseconds.  The command contract is deliberately strict: one or more
// integer digits, '.', and exactly three fractional digits.
static bool photons_parse_standard_lap_ns(const char* text,
                                          uint64_t& standard_lap_ps) {
  standard_lap_ps = 0ULL;
  if (!text || !*text) return false;

  const char* p = text;
  uint64_t whole_ns = 0ULL;
  uint32_t integer_digits = 0U;
  while (*p >= '0' && *p <= '9') {
    const uint32_t digit = (uint32_t)(*p - '0');
    if (whole_ns > (UINT64_MAX - (uint64_t)digit) / 10ULL) {
      return false;
    }
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
  if (whole_ns > (UINT64_MAX - (uint64_t)fractional_ps) / 1000ULL) {
    return false;
  }

  standard_lap_ps = whole_ns * 1000ULL + (uint64_t)fractional_ps;
  return standard_lap_ps != 0ULL;
}


static Payload cmd_set_standard_lap_ns(const Payload& args) {
  const char* text = args.getString("standard_lap_ns");
  uint64_t requested_ps = 0ULL;
  if (!photons_parse_standard_lap_ns(text, requested_ps)) {
    // Malformed metrological configuration is a program/integration
    // failure, not scientific invalidity.  Do not invent a default.
    __builtin_trap();
  }

  if (g_standard_lap_configured) {
    if (requested_ps != g_standard_lap_ps) {
      // The reference is immutable for one Teensy runtime.  A changed
      // standard requires a restart/new statistics lineage.
      __builtin_trap();
    }
  } else {
    g_standard_lap_ps = requested_ps;

    // Configuration is the clean PHOTONS statistics origin.  The exact zero
    // endpoint makes the first published second part of young rolling windows
    // without inventing a pre-configuration population.
    photons_ppb_windows_seed_origin();

    g_fragment_timer = timepop_arm(
        PHOTONS_FRAGMENT_PERIOD_NS,
        true,
        photons_fragment_tick,
        nullptr,
        "PHOTONS_FRAGMENT");
    if (g_fragment_timer == TIMEPOP_INVALID_HANDLE) {
      __builtin_trap();
    }

    photons_memory_barrier();
    g_standard_lap_configured = true;

    if (PHOTONS_EMULATOR_ENABLED) {
      photons_emulator_start();
    }
  }

  Payload p;
  p.add("standard_lap_configured", true);
  p.add("standard_lap_ps", g_standard_lap_ps);
  p.add("standard_lap_ns",
        toFixedDecimal((double)g_standard_lap_ps / 1000.0, 3));
  return p;
}


static Payload cmd_flash_cut(const Payload& args) {
  if (!g_standard_lap_configured) __builtin_trap();

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


static Payload cmd_start(const Payload& args) {
  if (!g_standard_lap_configured) {
    __builtin_trap();
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


static Payload cmd_stop(const Payload& /*args*/) {
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
  }
}


static Payload cmd_report_photons(const Payload& /*args*/) {
  photons_fragment_snapshot_t canonical{};
  (void)photons_fragment_snapshot(&canonical);

  Payload p;
  p.add("report", "PHOTONS_INSTRUMENT");
  p.add("schema", "PHOTONS_INSTRUMENT_REPORT_V1");
  p.add("instrument_always_on", true);
  p.add("instrument_owner", "TEENSY.PHOTONS");
  p.add("snapshot_ok", canonical.snapshot_ok);
  p.add("valid", canonical.valid);
  p.add("standard_lap_configured", g_standard_lap_configured);
  if (g_standard_lap_configured) {
    p.add("standard_lap_ps", g_standard_lap_ps);
    p.add("standard_lap_ns",
          toFixedDecimal((double)g_standard_lap_ps / 1000.0, 3));
  }
  p.add("stats_reset_count", canonical.stats.reset_count);
  p.add("stats_update_count", canonical.stats.update_count);
  p.add("stats_reset_pending", g_photons_stats_reset_pending);
  p.add("lap_count", canonical.stats.lap_count);
  p.add("total_lap_gnss_ns", canonical.stats.total_lap_gnss_ns);
  p.add("mean_lap_ns", toFixedDecimal(canonical.stats.mean_lap_ns, 6));
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
    p.add("campaign_lap_count", canonical.campaign.lap_count);
    p.add("campaign_mean_lap_ns",
          toFixedDecimal(canonical.campaign.mean_lap_ns, 6));
    p.add("campaign_ppb", toFixedDecimal(canonical.campaign.ppb.ppb, 6));
  }
  p.add("custody_lap_count", g_photons_custody_lap_count);
  p.add("custody_total_lap_gnss_ns", g_photons_custody_total_lap_gnss_ns);
  return p;
}


static Payload cmd_report_stats(const Payload& /*args*/) {
  photons_fragment_snapshot_t canonical{};
  (void)photons_fragment_snapshot(&canonical);

  Payload p;
  p.add("report", "PHOTONS_STATS");
  p.add("schema", "PHOTONS_INSTRUMENT_STATS_REPORT_V1");
  p.add("snapshot_ok", canonical.snapshot_ok);
  p.add("valid", canonical.stats.valid);
  p.add("reset_count", canonical.stats.reset_count);
  p.add("update_count", canonical.stats.update_count);
  p.add("reset_pending", g_photons_stats_reset_pending);
  p.add("reset_request_count", g_photons_stats_reset_request_count);
  p.add("reset_commit_count", g_photons_stats_reset_commit_count);
  p.add("lap_count", canonical.stats.lap_count);
  p.add("total_lap_gnss_ns", canonical.stats.total_lap_gnss_ns);
  p.add("mean_lap_ns", toFixedDecimal(canonical.stats.mean_lap_ns, 6));
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
  p.add("science_excluded_count", canonical.science.excluded.count);
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
    }
  }
  return p;
}


static Payload cmd_stats_reset(const Payload& /*args*/) {
  if (!g_standard_lap_configured) __builtin_trap();

  Payload p;
  if (g_photons_stats_reset_pending) {
    p.add("status", "instrument_statistics_reset_requested");
    p.add("reset_pending", true);
    p.add("request_count", g_photons_stats_reset_request_count);
    p.add("commit_count", g_photons_stats_reset_commit_count);
    return p;
  }

  g_photons_stats_reset_pending = true;
  g_photons_stats_reset_request_count++;
  p.add("status", "instrument_statistics_reset_requested");
  p.add("reset", true);
  p.add("reset_pending", true);
  p.add("boundary", "AFTER_NEXT_SUCCESSFULLY_PUBLISHED_FRAGMENT");
  p.add("campaign_changed", false);
  p.add("custody_preserved", true);
  p.add("standard_lap_preserved", true);
  p.add("next_report", "REPORT_PHOTONS");
  return p;
}


static Payload cmd_inject_problem(const Payload& args) {
  const char* type = args.getString("type");
  if (!type || strcmp(type, "excursion") != 0) {
    Payload err;
    err.add("status", "inject_problem_rejected_type");
    err.add("error", type && *type ? "unsupported problem type"
                                  : "missing problem type");
    err.add("supported", "excursion");
    return err;
  }
  if (!PHOTONS_EMULATOR_ENABLED) {
    Payload err;
    err.add("status", "inject_problem_rejected_source");
    err.add("error", "excursion injection currently requires PHOTONS emulator");
    return err;
  }
  if (g_photons_campaign_state != photons_campaign_state_t::ACTIVE) {
    Payload err;
    err.add("status", "inject_problem_rejected_campaign_state");
    err.add("state", photons_campaign_state_name(g_photons_campaign_state));
    return err;
  }

  uint32_t expected = 0U;
  if (!__atomic_compare_exchange_n(&g_photons_problem_injection_armed,
                                   &expected,
                                   2U,
                                   false,
                                   __ATOMIC_ACQ_REL,
                                   __ATOMIC_ACQUIRE)) {
    Payload err;
    err.add("status", "inject_problem_rejected_already_armed");
    err.add("armed_state", expected);
    return err;
  }

  g_photons_problem_injection_arm_count++;
  Payload p;
  p.add("status", "inject_problem_armed");
  p.add("type", "excursion");
  p.add("one_shot", true);
  p.add("source", "EMULATOR_SYNTHETIC_RAW_LAP");
  p.add("magnitude", "PLUS_25_PERCENT_TARGET_CYCLES");
  p.add("physical_edge_custody_modified", false);
  p.add("boundary", "next_emulator_science_lap");
  p.add("campaign", g_photons_campaign_name);
  p.add("arm_count", g_photons_problem_injection_arm_count);
  p.add("fire_count", g_photons_problem_injection_fire_count);
  return p;
}


static Payload cmd_report(const Payload& /*args*/) {
  photons_toy_capture_t capture{};
  (void)photons_toy_capture_snapshot(&capture);

  photons_toy_fragment_t fragment{};
  (void)photons_toy_fragment_snapshot(&fragment);

  photons_fragment_snapshot_t canonical{};
  (void)photons_fragment_snapshot(&canonical);

  interrupt_photodiode_diag_t interrupt_diag{};
  (void)interrupt_photodiode_snapshot(&interrupt_diag);

  const photons_device_snapshot_t device = photons_device_snapshot();

  Payload p;

  p.add("initialized", g_initialized);
  p.add("subscription_ok", g_subscription_ok);
  p.add("interrupt_started", g_interrupt_started);
  p.add("fragment_timer_armed",
        g_fragment_timer != TIMEPOP_INVALID_HANDLE);
  p.add("standard_lap_configured", g_standard_lap_configured);
  p.add("fragment_publication_ready", g_standard_lap_configured);
  if (g_standard_lap_configured) {
    p.add("standard_lap_ps", g_standard_lap_ps);
    p.add("standard_lap_ns",
          toFixedDecimal((double)g_standard_lap_ps / 1000.0, 3));
  }

  p.add("campaign_state", photons_campaign_state_name(g_photons_campaign_state));
  p.add("campaign_active",
        g_photons_campaign_state == photons_campaign_state_t::ACTIVE ||
        g_photons_campaign_state == photons_campaign_state_t::STOP_PENDING ||
        g_photons_campaign_state == photons_campaign_state_t::FLASH_CUT_PENDING);
  if (g_photons_campaign_name[0]) {
    p.add("campaign", g_photons_campaign_name);
  }
  p.add("campaign_start_after_sequence",
        g_photons_campaign_start_after_sequence);
  p.add("campaign_public_count", g_photons_campaign_public_count);
  if (g_photons_flash_cut_campaign_name[0]) {
    p.add("flash_cut_campaign", g_photons_flash_cut_campaign_name);
  }
  p.add("flash_cut_request_count", g_photons_flash_cut_request_count);
  p.add("flash_cut_commit_count", g_photons_flash_cut_commit_count);
  p.add("flash_cut_reject_count", g_photons_flash_cut_reject_count);
  p.add("campaign_origin_lap_count", g_photons_campaign_origin_lap_count);
  p.add("campaign_origin_total_lap_gnss_ns",
        g_photons_campaign_origin_total_lap_gnss_ns);
  p.add("campaign_start_request_count",
        g_photons_campaign_start_request_count);
  p.add("campaign_start_commit_count",
        g_photons_campaign_start_commit_count);
  p.add("campaign_stop_request_count",
        g_photons_campaign_stop_request_count);
  p.add("campaign_stop_commit_count",
        g_photons_campaign_stop_commit_count);
  if (canonical.campaign.present) {
    p.add("campaign_snapshot_public_count", canonical.campaign.public_count);
    p.add("campaign_snapshot_final", canonical.campaign.final);
    p.add("campaign_snapshot_lap_count", canonical.campaign.lap_count);
    p.add("campaign_snapshot_total_lap_gnss_ns",
          canonical.campaign.total_lap_gnss_ns);
    p.add("campaign_snapshot_mean_lap_ns",
          toFixedDecimal(canonical.campaign.mean_lap_ns, 6));
    if (canonical.campaign.ppb.sample_count != 0ULL) {
      p.add("campaign_snapshot_ppb",
            toFixedDecimal(canonical.campaign.ppb.ppb, 6));
    }
  }

  p.add("fragment_schema", "PHOTONS_FRAGMENT_V1");
  p.add("fragment_snapshot_ok", canonical.snapshot_ok);
  p.add("fragment_valid", canonical.valid);
  p.add("fragment_raw_lap_count", canonical.raw_lap_count);
  p.add("fragment_projected_laps_this_fragment",
        canonical.projected_laps_this_fragment);

  p.add("raw_cycles_completed_lap_count",
        canonical.raw_cycles.completed_lap_count);
  p.add("raw_cycles_observed_cycles",
        canonical.raw_cycles.observed_cycles);
  p.add("raw_cycles_previous_observed_cycles",
        canonical.raw_cycles.previous_observed_cycles);
  p.add("raw_cycles_static_prediction_valid",
        canonical.raw_cycles.static_prediction_valid);
  p.add("raw_cycles_static_prediction_cycles",
        canonical.raw_cycles.static_prediction_cycles);
  p.add("raw_cycles_static_residual_cycles",
        canonical.raw_cycles.static_residual_cycles);
  p.add("raw_cycles_mean_cycles_this_fragment",
        toFixedDecimal(canonical.raw_cycles.mean_cycles_this_fragment, 6));
  p.add("raw_cycles_previous_fragment_mean_valid",
        canonical.raw_cycles.previous_fragment_mean_valid);
  p.add("raw_cycles_previous_fragment_mean_cycles",
        toFixedDecimal(canonical.raw_cycles.previous_fragment_mean_cycles, 6));
  p.add("raw_cycles_fragment_mean_residual_cycles",
        toFixedDecimal(canonical.raw_cycles.fragment_mean_residual_cycles, 6));

  p.add("projection_anchor_cache_valid",
        canonical.projection.anchor_cache_valid);
  p.add("projection_anchor_pps_count",
        canonical.projection.anchor_pps_count);
  p.add("projection_anchor_dwt_cycles_per_second",
        canonical.projection.anchor_dwt_cycles_per_second);
  p.add("projection_attempt_count",
        canonical.projection.attempt_count);
  p.add("projection_success_count",
        canonical.projection.success_count);
  p.add("projection_reject_count",
        canonical.projection.reject_count);
  p.add("projection_queue_overflow_count",
        canonical.projection.queue_overflow_count);
  p.add("projection_last_valid",
        canonical.projection.last_valid);
  p.add("projection_last_start_dwt",
        canonical.projection.last_start_dwt);
  p.add("projection_last_end_dwt",
        canonical.projection.last_end_dwt);
  p.add("projection_last_lap_gnss_ns",
        canonical.projection.last_lap_gnss_ns);

  p.add("science_schema", "PHOTONS_SCIENCE_V2");
  p.add("science_valid", canonical.science.valid);
  p.add("science_candidate_count",
        canonical.science.candidate_count);
  p.add("science_candidates_this_fragment",
        canonical.science.candidates_this_fragment);

  p.add("science_accepted_count", canonical.science.accepted.count);
  p.add("science_accepted_this_fragment",
        canonical.science.accepted.count_this_fragment);
  p.add("science_accepted_raw_cycles_n",
        canonical.science.accepted.raw_cycles.n);
  p.add("science_accepted_raw_cycles_mean",
        toFixedDecimal(canonical.science.accepted.raw_cycles.mean, 6));
  p.add("science_accepted_raw_cycles_stddev",
        toFixedDecimal(canonical.science.accepted.raw_cycles.stddev, 6));
  p.add("science_accepted_raw_cycles_min",
        toFixedDecimal(canonical.science.accepted.raw_cycles.min, 6));
  p.add("science_accepted_raw_cycles_max",
        toFixedDecimal(canonical.science.accepted.raw_cycles.max, 6));
  p.add("science_accepted_projected_lap_ns_n",
        canonical.science.accepted.projected_lap_ns.n);
  p.add("science_accepted_projected_lap_ns_mean",
        toFixedDecimal(canonical.science.accepted.projected_lap_ns.mean, 6));
  p.add("science_accepted_projected_lap_ns_stddev",
        toFixedDecimal(canonical.science.accepted.projected_lap_ns.stddev, 6));
  p.add("science_accepted_projected_lap_ns_min",
        toFixedDecimal(canonical.science.accepted.projected_lap_ns.min, 6));
  p.add("science_accepted_projected_lap_ns_max",
        toFixedDecimal(canonical.science.accepted.projected_lap_ns.max, 6));

  p.add("science_excluded_count", canonical.science.excluded.count);
  p.add("science_excluded_this_fragment",
        canonical.science.excluded.count_this_fragment);
  p.add("science_excluded_raw_cycles_n",
        canonical.science.excluded.raw_cycles.n);
  p.add("science_excluded_raw_cycles_mean",
        toFixedDecimal(canonical.science.excluded.raw_cycles.mean, 6));
  p.add("science_excluded_raw_cycles_stddev",
        toFixedDecimal(canonical.science.excluded.raw_cycles.stddev, 6));
  p.add("science_excluded_raw_cycles_min",
        toFixedDecimal(canonical.science.excluded.raw_cycles.min, 6));
  p.add("science_excluded_raw_cycles_max",
        toFixedDecimal(canonical.science.excluded.raw_cycles.max, 6));
  p.add("science_excluded_projected_lap_ns_n",
        canonical.science.excluded.projected_lap_ns.n);
  p.add("science_excluded_projected_lap_ns_mean",
        toFixedDecimal(canonical.science.excluded.projected_lap_ns.mean, 6));
  p.add("science_excluded_projected_lap_ns_stddev",
        toFixedDecimal(canonical.science.excluded.projected_lap_ns.stddev, 6));
  p.add("science_excluded_projected_lap_ns_min",
        toFixedDecimal(canonical.science.excluded.projected_lap_ns.min, 6));
  p.add("science_excluded_projected_lap_ns_max",
        toFixedDecimal(canonical.science.excluded.projected_lap_ns.max, 6));

  p.add("science_exclusion_projection_invalid",
        canonical.science.exclusion_reasons.projection_invalid);
  p.add("science_exclusion_seed_disagreement",
        canonical.science.exclusion_reasons.seed_disagreement);
  p.add("science_exclusion_raw_cycle_excursion",
        canonical.science.exclusion_reasons.raw_cycle_excursion);
  p.add("science_exclusion_projection_invalid_this_fragment",
        canonical.science.exclusion_reasons.projection_invalid_this_fragment);
  p.add("science_exclusion_seed_disagreement_this_fragment",
        canonical.science.exclusion_reasons.seed_disagreement_this_fragment);
  p.add("science_exclusion_raw_cycle_excursion_this_fragment",
        canonical.science.exclusion_reasons.raw_cycle_excursion_this_fragment);
  p.add("science_predictor_valid",
        canonical.science.predictor_valid);
  p.add("science_predictor_cycles",
        canonical.science.predictor_cycles);
  p.add("science_gate_cycles",
        canonical.science.gate_cycles);
  p.add("science_reject_streak",
        canonical.science.reject_streak);
  p.add("science_max_reject_streak",
        canonical.science.max_reject_streak);
  p.add("science_seed_pending",
        canonical.science.seed_pending);
  p.add("science_last_disposition",
        photons_lap_science_disposition_name(
            canonical.science.last_disposition_id));
  p.add("science_last_reason",
        photons_lap_science_reason_name(
            canonical.science.last_reason_code));
  p.add("science_last_observed_cycles",
        canonical.science.last_observed_cycles);
  p.add("science_last_prediction_cycles",
        canonical.science.last_prediction_cycles);
  p.add("science_last_residual_cycles",
        canonical.science.last_residual_cycles);
  p.add("science_last_gate_cycles",
        canonical.science.last_gate_cycles);

  p.add("stats_valid", canonical.stats.valid);
  p.add("stats_reset_count", canonical.stats.reset_count);
  p.add("stats_reset_pending", g_photons_stats_reset_pending);
  p.add("stats_reset_request_count", g_photons_stats_reset_request_count);
  p.add("stats_reset_commit_count", g_photons_stats_reset_commit_count);
  p.add("stats_lap_count", canonical.stats.lap_count);
  p.add("stats_total_lap_gnss_ns",
        canonical.stats.total_lap_gnss_ns);
  p.add("stats_mean_lap_ns",
        toFixedDecimal(canonical.stats.mean_lap_ns, 6));
  p.add("stats_lap_welford_n",
        canonical.stats.lap_time_welford.n);
  p.add("stats_lap_welford_mean",
        toFixedDecimal(canonical.stats.lap_time_welford.mean, 6));
  p.add("stats_lap_welford_m2",
        toFixedDecimal(canonical.stats.lap_time_welford.m2, 6));
  p.add("stats_lap_welford_stddev",
        toFixedDecimal(canonical.stats.lap_time_welford.stddev, 6));
  p.add("stats_lap_welford_stderr",
        toFixedDecimal(canonical.stats.lap_time_welford.stderr_value, 6));
  p.add("stats_lap_welford_min",
        toFixedDecimal(canonical.stats.lap_time_welford.min, 6));
  p.add("stats_lap_welford_max",
        toFixedDecimal(canonical.stats.lap_time_welford.max, 6));
  p.add("stats_update_count", canonical.stats.update_count);
  p.add("stats_rolling_ppb_current_sequence",
        canonical.stats.rolling_ppb_current_sequence);
  p.add("stats_rolling_ppb_endpoint_admitted",
        canonical.stats.rolling_ppb_endpoint_admitted);
  p.add("stats_rolling_ppb_interval_advanced",
        canonical.stats.rolling_ppb_interval_advanced);
  p.add("stats_ppb_second_history_count", g_photons_ppb_seconds_count);
  p.add("stats_ppb_minute_history_count", g_photons_ppb_minutes_count);

  p.add("stats_ppb_10_min_n",
        canonical.stats.ppb_buckets.minute_10.sample_count);
  if (canonical.stats.ppb_buckets.minute_10.sample_count != 0ULL) {
    p.add("stats_ppb_10_min",
          toFixedDecimal(canonical.stats.ppb_buckets.minute_10.ppb, 6));
  }
  p.add("stats_ppb_60_min_n",
        canonical.stats.ppb_buckets.minute_60.sample_count);
  if (canonical.stats.ppb_buckets.minute_60.sample_count != 0ULL) {
    p.add("stats_ppb_60_min",
          toFixedDecimal(canonical.stats.ppb_buckets.minute_60.ppb, 6));
  }
  p.add("stats_ppb_8_hour_n",
        canonical.stats.ppb_buckets.hour_8.sample_count);
  if (canonical.stats.ppb_buckets.hour_8.sample_count != 0ULL) {
    p.add("stats_ppb_8_hour",
          toFixedDecimal(canonical.stats.ppb_buckets.hour_8.ppb, 6));
  }
  p.add("stats_ppb_24_hour_n",
        canonical.stats.ppb_buckets.hour_24.sample_count);
  if (canonical.stats.ppb_buckets.hour_24.sample_count != 0ULL) {
    p.add("stats_ppb_24_hour",
          toFixedDecimal(canonical.stats.ppb_buckets.hour_24.ppb, 6));
  }
  p.add("custody_lap_count", g_photons_custody_lap_count);
  p.add("custody_total_lap_gnss_ns", g_photons_custody_total_lap_gnss_ns);
  p.add("problem_injection_armed",
        __atomic_load_n(&g_photons_problem_injection_armed, __ATOMIC_ACQUIRE) == 2U);
  p.add("problem_injection_arm_count", g_photons_problem_injection_arm_count);
  p.add("problem_injection_fire_count", g_photons_problem_injection_fire_count);
  p.add("problem_injection_last_extra_cycles",
        g_photons_problem_injection_last_extra_cycles);

  p.add("stats_ppb_total_n",
        canonical.stats.ppb_buckets.total.sample_count);
  if (canonical.stats.ppb_buckets.total.sample_count != 0ULL) {
    p.add("stats_ppb_total",
          toFixedDecimal(canonical.stats.ppb_buckets.total.ppb, 6));
  }
  p.add("baseline_present", canonical.baseline.present);
  p.add("baseline_residual_valid",
        canonical.baseline.residual_valid);

  p.add("laser_enabled", device.laser_enabled);
  p.add("laser_id1_raw", device.laser_id1_raw);
  p.add("laser_id1_current_ma",
        toFixedDecimal(device.laser_id1_current_ma, 6));
  p.add("laser_monitor_raw", device.laser_monitor_raw);
  p.add("laser_monitor_v",
        toFixedDecimal(device.laser_monitor_v, 6));
  p.add("laser_emitting", device.laser_emitting);

  p.add("photodiode_edge_pin", PHOTODIODE_EDGE_PIN);
  p.add("photodiode_edge_level", device.photodiode_edge_level);
  p.add("photodiode_mon_pin", PHOTODIODE_MON_PIN);
  p.add("photodiode_mon_raw", device.photodiode_mon_raw);
  p.add("photodiode_mon_v",
        toFixedDecimal(device.photodiode_mon_v, 6));
  p.add("photodiode_mon_emulated", PHOTONS_EMULATOR_ENABLED);

  p.add("emulator_enabled", PHOTONS_EMULATOR_ENABLED);
  p.add("emulator_initialized", g_photons_emulator.initialized);
  p.add("emulator_edge_output_pin", PHOTONS_EMULATOR_EDGE_OUTPUT_PIN);
  p.add("emulator_loopback_input_pin", PHOTODIODE_EDGE_PIN);
  p.add("emulator_timing_fidelity",
        "STANDARD_LOCKED_SYNTHETIC_SCIENCE_WITH_PHYSICAL_EDGE_CUSTODY");
  p.add("emulator_cadence_strategy", "ONE_RECURRING_TIMEPOP_TIMER");
  if (g_standard_lap_configured) {
    p.add("emulator_nominal_lap_ns",
          toFixedDecimal((double)g_standard_lap_ps / 1000.0, 3));
  }
  p.add("emulator_cadence_period_ns", PHOTONS_EMULATOR_CADENCE_NS);
  p.add("emulator_projection_wait_count",
        g_photons_emulator.projection_wait_count);
  p.add("emulator_synthetic_target_unavailable_count",
        g_photons_emulator.synthetic_target_unavailable_count);
  p.add("emulator_synthetic_target_cycles",
        g_photons_emulator.synthetic_target_cycles);
  p.add("emulator_synthetic_last_jitter_cycles",
        g_photons_emulator.synthetic_last_jitter_cycles);
  p.add("emulator_synthetic_last_lap_cycles",
        g_photons_emulator.synthetic_last_lap_cycles);
  p.add("emulator_synthetic_lap_count",
        g_photons_emulator.synthetic_lap_count);
  p.add("emulator_cadence_tick_count", g_photons_emulator.cadence_tick_count);
  p.add("emulator_hits_per_train", PHOTONS_EMULATOR_HITS_PER_TRAIN);
  p.add("emulator_measured_laps", PHOTONS_EMULATOR_MEASURED_LAPS);
  p.add("emulator_stage", photons_emulator_stage_name(g_photons_emulator.stage));
  p.add("emulator_train_active", g_photons_emulator.train_active);
  p.add("emulator_train_count", g_photons_emulator.train_count);
  p.add("emulator_relaunch_count", g_photons_emulator.relaunch_count);

  p.add("emulator_laser_pulse_requested_cycles",
        PHOTONS_EMULATOR_LASER_PULSE_CYCLES);
  p.add("emulator_laser_pulse_count", g_photons_emulator.laser_pulse_count);
  p.add("emulator_last_laser_pulse_cycles",
        g_photons_emulator.last_laser_pulse_cycles);
  p.add("emulator_max_laser_pulse_cycles",
        g_photons_emulator.max_laser_pulse_cycles);

  p.add("emulator_generated_edge_count",
        g_photons_emulator.generated_edge_count);
  p.add("emulator_expected_hit_ordinal",
        g_photons_emulator.expected_hit_ordinal);
  p.add("emulator_unexpected_edge_count",
        g_photons_emulator.unexpected_edge_count);
  p.add("emulator_state_error_count",
        g_photons_emulator.state_error_count);
  p.add("emulator_hit1_count", g_photons_emulator.hit1_count);
  p.add("emulator_hit2_count", g_photons_emulator.hit2_count);
  p.add("emulator_hit3_count", g_photons_emulator.hit3_count);
  p.add("emulator_hit4_count", g_photons_emulator.hit4_count);
  p.add("emulator_hit5_count", g_photons_emulator.hit5_count);
  const uint32_t emulator_observed_hit_count =
      g_photons_emulator.hit1_count +
      g_photons_emulator.hit2_count +
      g_photons_emulator.hit3_count +
      g_photons_emulator.hit4_count +
      g_photons_emulator.hit5_count;
  p.add("emulator_observed_hit_count", emulator_observed_hit_count);

  p.add("emulator_lap_start_count", g_photons_emulator.lap_start_count);
  p.add("emulator_lap1_complete_count",
        g_photons_emulator.lap1_complete_count);
  p.add("emulator_lap2_complete_count",
        g_photons_emulator.lap2_complete_count);
  p.add("emulator_lap3_complete_count",
        g_photons_emulator.lap3_complete_count);
  p.add("emulator_dead_lap_count", g_photons_emulator.dead_lap_count);
  p.add("emulator_lap_start_dwt", g_photons_emulator.lap_start_dwt);
  p.add("emulator_lap1_last_cycles", g_photons_emulator.lap1_last_cycles);
  p.add("emulator_lap2_last_cycles", g_photons_emulator.lap2_last_cycles);
  p.add("emulator_lap3_last_cycles", g_photons_emulator.lap3_last_cycles);

  p.add("emulator_cadence_timer_arm_count",
        g_photons_emulator.cadence_timer_arm_count);
  p.add("emulator_cadence_timer_arm_fail_count",
        g_photons_emulator.cadence_timer_arm_fail_count);
  p.add("emulator_cadence_timer_handle_valid",
        g_photons_emulator.cadence_timer != TIMEPOP_INVALID_HANDLE);

  p.add("capture_edge_count", capture.edge_count);
  p.add("capture_last_edge_sequence", capture.last_edge_sequence);
  p.add("capture_last_pps_sequence", capture.last_pps_sequence);
  p.add("capture_last_dwt_at_edge", capture.last_dwt_at_edge);
  p.add("capture_last_isr_entry_dwt_raw", capture.last_isr_entry_dwt_raw);
  p.add("capture_isr_entry_to_edge_correction_cycles",
        capture.isr_entry_to_edge_correction_cycles);

  p.add("capture_interval_valid", capture.interval_valid);
  p.add("capture_last_interval_cycles", capture.last_interval_cycles);
  p.add("capture_min_interval_cycles", capture.min_interval_cycles);
  p.add("capture_max_interval_cycles", capture.max_interval_cycles);

  p.add("capture_prediction_valid", capture.prediction_valid);
  p.add("capture_prediction_cycles", capture.prediction_cycles);
  p.add("capture_residual_cycles", capture.residual_cycles);

  p.add("fragment_sequence", fragment.sequence);
  p.add("fragment_publish_count", g_publish_count);
  p.add("fragment_publish_reject_count", g_publish_reject_count);
  p.add("fragment_edge_count_total", fragment.edge_count_total);
  p.add("fragment_edges_this_second", fragment.edges_this_second);

  p.add("interrupt_subscribed", interrupt_diag.subscribed);
  p.add("interrupt_active", interrupt_diag.active);
  p.add("interrupt_irq_count", interrupt_diag.irq_count);
  p.add("interrupt_callback_count", interrupt_diag.callback_count);
  p.add("interrupt_callback_missing_count",
        interrupt_diag.callback_missing_count);
  p.add("interrupt_inactive_edge_count",
        interrupt_diag.inactive_edge_count);
  p.add("interrupt_source_pin", interrupt_diag.source_pin);
  p.add("interrupt_last_callback_wall_cycles",
        interrupt_diag.last_callback_wall_cycles);
  p.add("interrupt_max_callback_wall_cycles",
        interrupt_diag.max_callback_wall_cycles);

  p.add("emulator_custody_counts_match",
        g_photons_emulator.generated_edge_count ==
            interrupt_diag.irq_count &&
        g_photons_emulator.generated_edge_count ==
            interrupt_diag.callback_count &&
        g_photons_emulator.generated_edge_count ==
            capture.edge_count &&
        g_photons_emulator.generated_edge_count ==
            emulator_observed_hit_count);

  return p;
}

static Payload cmd_init(const Payload& /*args*/) {
  photons_laser_initialize_hardware();
  photons_emit_laser_initialization_event();
  return ok_payload();
}

static Payload cmd_on(const Payload& /*args*/) {
  digitalWrite(LD_ON_PIN, HIGH);

  Payload ev;
  ev.add("action", "allow_emission");
  enqueueEvent("LASER_ON", ev);

  return ok_payload();
}

static Payload cmd_off(const Payload& /*args*/) {
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
  { "SET_STANDARD_LAP_NS", cmd_set_standard_lap_ns },
  { "START",               cmd_start               },
  { "FLASH_CUT",           cmd_flash_cut           },
  { "STOP",                cmd_stop                },
  { "REPORT",              cmd_report              },
  { "REPORT_PHOTONS",      cmd_report_photons      },
  { "REPORT_STATS",        cmd_report_stats        },
  { "STATS_RESET",         cmd_stats_reset         },
  { "INJECT_PROBLEM",      cmd_inject_problem      },
  { "ON",                  cmd_on                  },
  { "OFF",                 cmd_off                 },
  { nullptr, nullptr }
};

static const process_vtable_t PHOTONS_PROCESS = {
  .process_id = "PHOTONS",
  .commands = PHOTONS_COMMANDS,
  .subscriptions = nullptr,
};

void process_photons_register(void) {
  process_register("PHOTONS", &PHOTONS_PROCESS);
}
