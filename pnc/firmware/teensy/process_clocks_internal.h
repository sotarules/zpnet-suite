// ============================================================================
// process_clocks_internal.h — Shared Internal State (Alpha ↔ Beta)
// ============================================================================
//
// Nanosecond-first clock model.
//
// Core intent:
//
//   • GNSS nanoseconds are the canonical public truth.
//   • DWT remains the bridge substrate and is the only clock with a live
//     next-second adjustment term.
//   • OCXO1 / OCXO2 are modeled by:
//       - last observed edge in GNSS ns
//       - last observed local nanosecond count at that edge
//       - predicted local nanoseconds per GNSS second
//   • PPS computes the canonical at-PPS values that feed TIMEBASE and now().
//   • Diagnostics belong in process_interrupt and reports, not in sacred state.
//
// ============================================================================

#pragma once

#include "config.h"
#include "payload.h"
#include "time.h"
#include "process_interrupt.h"

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <stdio.h>

// ============================================================================
// DWT register defines
// ============================================================================

#define DEMCR               (*(volatile uint32_t *)0xE000EDFC)
#define DEMCR_TRCENA        (1 << 24)
#define DWT_CTRL            (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT          (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL_CYCCNTENA  (1 << 0)

// ============================================================================
// DWT nanosecond conversion helpers
// ============================================================================

static inline uint64_t dwt_cycles_to_ns(uint64_t cycles) {
  return (cycles * DWT_NS_NUM) / DWT_NS_DEN;
}

static inline uint64_t dwt_ns_to_cycles(uint64_t ns) {
  return (ns * DWT_NS_DEN) / DWT_NS_NUM;
}

// ============================================================================
// Always-on DWT↔GNSS anchor state (alpha-owned, beta-readable)
// ============================================================================

extern volatile uint64_t g_gnss_ns_count_at_pps;

extern volatile uint32_t g_dwt_cycle_count_at_pps;
extern volatile uint32_t g_dwt_cycle_count_next_second_prediction;
extern volatile int32_t  g_dwt_cycle_count_next_second_adjustment;
extern volatile uint64_t g_dwt_model_pps_count;

// ============================================================================
// OCXO canonical state (alpha-owned, beta-readable)
// ============================================================================

struct ocxo_clock_state_t {
  volatile uint64_t gnss_ns_at_edge;
  volatile uint64_t ns_count_at_edge;
  volatile uint64_t ns_count_next_second_prediction;
  volatile uint64_t ns_count_at_pps;
};

extern ocxo_clock_state_t g_ocxo1_clock;
extern ocxo_clock_state_t g_ocxo2_clock;

// ============================================================================
// Lightweight measurement state
// ============================================================================

struct ocxo_measurement_t {
  volatile int64_t  second_residual_ns;
  volatile uint64_t prev_gnss_ns_at_edge;
};

extern ocxo_measurement_t g_ocxo1_measurement;
extern ocxo_measurement_t g_ocxo2_measurement;

// ============================================================================
// Interrupt diag snapshots (alpha-owned, beta-readable)
// ============================================================================

extern interrupt_capture_diag_t g_pps_interrupt_diag;
extern interrupt_capture_diag_t g_ocxo1_interrupt_diag;
extern interrupt_capture_diag_t g_ocxo2_interrupt_diag;

static inline void clocks_capture_interrupt_diag(interrupt_capture_diag_t& dst,
                                                 const interrupt_capture_diag_t* src) {
  if (!src) {
    dst = interrupt_capture_diag_t{};
    dst.enabled = false;
    return;
  }
  dst = *src;
}

static inline void clocks_payload_add_interrupt_diag(Payload& p,
                                                     const char* prefix,
                                                     const interrupt_capture_diag_t& diag) {
  char key[96];

  auto add_u32 = [&](const char* suffix, uint32_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };

  auto add_u64 = [&](const char* suffix, uint64_t value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };

  auto add_bool = [&](const char* suffix, bool value) {
    snprintf(key, sizeof(key), "%s_%s", prefix, suffix);
    p.add(key, value);
  };

  add_bool("enabled", diag.enabled);

  add_u32("provider", (uint32_t)diag.provider);
  add_u32("lane", (uint32_t)diag.lane);
  add_u32("kind", (uint32_t)diag.kind);

  add_u64("prespin_target_gnss_ns", diag.prespin_target_gnss_ns);
  add_u64("event_target_gnss_ns", diag.event_target_gnss_ns);

  add_bool("prespin_active", diag.prespin_active);
  add_bool("prespin_fired", diag.prespin_fired);
  add_bool("prespin_timed_out", diag.prespin_timed_out);

  add_u32("shadow_dwt", diag.shadow_dwt);
  add_u32("dwt_isr_entry_raw", diag.dwt_isr_entry_raw);
  add_u32("approach_cycles", diag.approach_cycles);

  add_u32("dwt_at_event_adjusted", diag.dwt_at_event_adjusted);
  add_u64("gnss_ns_at_event", diag.gnss_ns_at_event);
  add_u32("counter32_at_event", diag.counter32_at_event);
  add_u32("dwt_event_correction_cycles", diag.dwt_event_correction_cycles);

  add_u32("prespin_arm_count", diag.prespin_arm_count);
  add_u32("prespin_complete_count", diag.prespin_complete_count);
  add_u32("prespin_timeout_count", diag.prespin_timeout_count);
  add_u32("anomaly_count", diag.anomaly_count);
}

// ============================================================================
// AD5693R init
// ============================================================================

extern bool g_ad5693r_init_ok;

// ============================================================================
// OCXO DAC state — dual oscillator
// ============================================================================

struct ocxo_dac_state_t {
  double   dac_fractional;
  uint16_t dac_hw_code;
  uint32_t dac_min;
  uint32_t dac_max;
  double   servo_last_step;
  double   servo_last_residual;
  uint32_t servo_settle_count;
  uint32_t servo_adjustments;
};

extern ocxo_dac_state_t ocxo1_dac;
extern ocxo_dac_state_t ocxo2_dac;

// ============================================================================
// OCXO servo mode
// ============================================================================

enum class servo_mode_t : uint8_t {
  OFF   = 0,
  MEAN  = 1,
  TOTAL = 2,
  NOW   = 3,
};

extern servo_mode_t calibrate_ocxo_mode;

const char* servo_mode_str(servo_mode_t mode);
servo_mode_t servo_mode_parse(const char* s);

static constexpr int32_t  SERVO_MAX_STEP       = 64;
static constexpr uint32_t SERVO_SETTLE_SECONDS = 5;
static constexpr uint32_t SERVO_MIN_SAMPLES    = 10;

void ocxo_dac_set(ocxo_dac_state_t& s, double value);

// ============================================================================
// Campaign state
// ============================================================================

enum class clocks_campaign_state_t {
  STOPPED,
  STARTED
};

extern volatile clocks_campaign_state_t campaign_state;
extern char     campaign_name[64];
extern uint64_t campaign_seconds;

extern volatile bool request_start;
extern volatile bool request_stop;
extern volatile bool request_recover;
extern volatile bool request_zero;

extern uint64_t recover_dwt_ns;
extern uint64_t recover_gnss_ns;
extern uint64_t recover_ocxo1_ns;
extern uint64_t recover_ocxo2_ns;

// ============================================================================
// Minimal residual tracking
// ============================================================================

struct pps_residual_t {
  int64_t  residual;
  uint64_t n;
  double   mean;
  double   m2;
};

extern pps_residual_t residual_dwt;
extern pps_residual_t residual_ocxo1;
extern pps_residual_t residual_ocxo2;

void residual_reset(pps_residual_t& r);
void residual_update_sample(pps_residual_t& r, int64_t residual);
double residual_stddev(const pps_residual_t& r);
double residual_stderr(const pps_residual_t& r);

// ============================================================================
// DAC Welford tracking
// ============================================================================

struct dac_welford_t {
  uint64_t n;
  double   mean;
  double   m2;
  double   min_val;
  double   max_val;
};

extern dac_welford_t dac_welford_ocxo1;
extern dac_welford_t dac_welford_ocxo2;

void dac_welford_reset(dac_welford_t& w);
void dac_welford_update(dac_welford_t& w, double value);
double dac_welford_stddev(const dac_welford_t& w);
double dac_welford_stderr(const dac_welford_t& w);

// ============================================================================
// Rolling counters / always-on helpers
// ============================================================================

extern uint64_t dwt_rolling_64;
extern uint32_t dwt_rolling_32;

extern uint64_t gnss_rolling_raw_64;
extern uint32_t gnss_rolling_32;

extern uint64_t ocxo1_rolling_64;
extern uint32_t ocxo1_rolling_32;

extern uint64_t ocxo2_rolling_64;
extern uint32_t ocxo2_rolling_32;

// ============================================================================
// Campaign-scoped accumulators
// ============================================================================

extern uint64_t dwt_cycles_64;
extern uint64_t gnss_raw_64;
extern uint64_t ocxo1_ticks_64;
extern uint64_t ocxo2_ticks_64;

// ============================================================================
// QTimer1 read helper (clocks-owned helper for GNSS live reads)
// ============================================================================

uint32_t qtimer1_read_32(void);

// ============================================================================
// Simple helpers
// ============================================================================

static inline uint32_t dwt_effective_cycles_per_second(void) {
  return (uint32_t)((int64_t)g_dwt_cycle_count_next_second_prediction +
                    (int64_t)g_dwt_cycle_count_next_second_adjustment);
}

// ============================================================================
// Watchdog anomaly latch
// ============================================================================

extern volatile bool     watchdog_anomaly_active;
extern volatile bool     watchdog_anomaly_publish_pending;
extern volatile uint32_t watchdog_anomaly_sequence;
extern char              watchdog_anomaly_reason[64];
extern volatile uint32_t watchdog_anomaly_detail0;
extern volatile uint32_t watchdog_anomaly_detail1;
extern volatile uint32_t watchdog_anomaly_detail2;
extern volatile uint32_t watchdog_anomaly_detail3;
extern volatile uint32_t watchdog_anomaly_trigger_dwt;

// ============================================================================
// Beta entry points
// ============================================================================

void clocks_beta_pps(void);
void clocks_watchdog_anomaly(const char* reason,
                             uint32_t detail0 = 0,
                             uint32_t detail1 = 0,
                             uint32_t detail2 = 0,
                             uint32_t detail3 = 0);

// ============================================================================
// Zeroing
// ============================================================================

void clocks_zero_all(void);