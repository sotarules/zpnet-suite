// ============================================================================
// process_clocks.cpp — ZPNet CLOCKS (Teensy) — v10 Trend-Aware Statistics
// ============================================================================
//
// CLOCKS is a dormant, PPS-anchored measurement instrument.
//
// • Boots fully silent
// • Does NOT track PPS unless a campaign is active or pending
// • Nanosecond clocks are ZERO unless a campaign is STARTED
// • All three counters are always live (capability, not narration)
//
// START / STOP / RECOVER are *requests*.
// All authoritative state transitions occur on the PPS boundary.
//
// Clock domains:
//
//   DWT    — ARM Cortex-M7 cycle counter, 1008 MHz, internal
//   GNSS   — GF-8802 10 MHz VCLOCK, external via GPT2 pin 14
//   OCXO   — AOCJY1-A 10 MHz oven oscillator, external via GPT1 pin 25
//
// v9: ISR-synchronized anchors (spin loop eliminated).
//
//   Both DWT and GPT2 PPS anchors are now captured in the PPS ISR
//   at entry — two back-to-back register reads, ~2 ns apart.  Both
//   are ~50 ns after the true PPS edge due to ARM interrupt entry
//   overhead, but they share the same latency, so the skew between
//   them is effectively zero.
//
//   This eliminates the pre-PPS fine timer, the spin loop, BASEPRI
//   masking, dispatch_shadow_dwt, isr_captured_shadow_dwt, and all
//   associated complexity (~137 lines removed).
//
//   The ISR entry latency (~50 ns) is deterministic and stable, as
//   proven by months of tdc_analyzer data.  For sub-second
//   interpolation, the latency cancels: both the PPS anchor and the
//   timepop event anchor share the same ISR entry overhead pattern,
//   and the per-event correction uses fire_ns (measured by hardware)
//   to remove it exactly.
//
//   Interpolation uses the last-second DWT delta as the denominator
//   (dwt_cycles_per_pps), tracking the crystal in real time.  Thermal
//   drift (~0.3 cycles/second) is invisible because the prior second's
//   rate matches the current second to within the 7 ns hardware jitter
//   floor.
//
//   The deterministic overload timebase_gnss_ns_from_dwt() allows
//   callers with ISR-captured DWT values to compute GNSS nanoseconds
//   without any live register reads or torn-read loops.
//
// v10: Trend-aware prediction statistics.
//
//   The per-second DWT and OCXO deltas exhibit slow thermal drift —
//   the crystal frequency changes by a few cycles per second over
//   minutes.  The existing Welford residual tracker measures the
//   spread of all residuals relative to the nominal frequency, which
//   conflates thermal drift range with measurement noise.
//
//   v10 adds a trend-aware prediction layer:
//
//     predicted_delta = prev_delta + (prev_delta - prev_prev_delta)
//     prediction_residual = actual_delta - predicted_delta
//
//   This is linear extrapolation from the two most recent deltas.
//   Welford statistics on the prediction residual directly measure
//   the instrument's actual interpolation uncertainty — "how wrong
//   is my best estimate of this second's crystal rate?"
//
//   The prediction residual stddev is the authoritative confidence
//   metric for sub-second interpolation.  For DWT this is expected
//   to be ~2-4 cycles (2-4 ns); for OCXO ~1-3 ticks (100-300 ns).
//
//   Additionally, dwt_cycles_per_pps now uses the PREDICTED count
//   instead of the raw prior count, improving interpolation accuracy
//   when the crystal is actively drifting.
//
//   New TIMEBASE_FRAGMENT fields:
//     dwt_pred_residual      — latest prediction error (cycles)
//     dwt_pred_mean          — running mean of prediction errors
//     dwt_pred_stddev        — stddev of prediction errors
//     dwt_pred_n             — sample count
//     ocxo_pred_residual     — latest prediction error (ticks)
//     ocxo_pred_mean         — running mean of prediction errors
//     ocxo_pred_stddev       — stddev of prediction errors
//     ocxo_pred_n            — sample count
//
// ============================================================================

#include "debug.h"
#include "timebase.h"
#include "process_clocks.h"
#include "process_timepop.h"

#include "payload.h"
#include "publish.h"
#include "timepop.h"
#include "config.h"
#include "util.h"

#include <Arduino.h>
#include "imxrt.h"

#include <math.h>
#include <stdlib.h>

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
// TimePop delay constants (nanoseconds)
// ============================================================================

static constexpr uint64_t OCXO_DITHER_NS   =     1000000ULL;  //   1 ms
static constexpr uint64_t PPS_RELAY_OFF_NS  =   500000000ULL;  // 500 ms

// ============================================================================
// Campaign State
// ============================================================================

enum class clocks_campaign_state_t {
  STOPPED,
  STARTED
};

static volatile clocks_campaign_state_t campaign_state =
  clocks_campaign_state_t::STOPPED;

static char campaign_name[64] = {0};

static volatile bool request_start   = false;
static volatile bool request_stop    = false;
static volatile bool request_recover = false;

static uint64_t recover_dwt_ns  = 0;
static uint64_t recover_gnss_ns = 0;
static uint64_t recover_ocxo_ns = 0;

static uint64_t campaign_seconds = 0;

// ============================================================================
// OCXO DAC state
// ============================================================================

static double   ocxo_dac_fractional = (double)OCXO_DAC_DEFAULT;

static uint32_t dither_cycle  = 0;
static constexpr uint32_t DITHER_PERIOD = 1000;

static bool     calibrate_ocxo_active = false;
static int32_t  servo_step            = 0;
static double   servo_last_residual   = 0.0;
static uint32_t servo_settle_count    = 0;
static uint32_t servo_adjustments     = 0;

static constexpr int32_t  SERVO_MAX_STEP       = 64;
static constexpr uint32_t SERVO_SETTLE_SECONDS = 5;
static constexpr uint32_t SERVO_MIN_SAMPLES    = 10;

static void ocxo_dac_set(double value) {
  if (value < (double)OCXO_DAC_MIN) value = (double)OCXO_DAC_MIN;
  if (value > (double)OCXO_DAC_MAX) value = (double)OCXO_DAC_MAX;
  ocxo_dac_fractional = value;
}

// ============================================================================
// PPS relay to Pi — 500 ms pulse on GNSS_PPS_RELAY (pin 32)
// ============================================================================

static volatile bool relay_arm_pending  = false;
static volatile bool relay_timer_active = false;

static void pps_relay_deassert(timepop_ctx_t*, void*) {
  digitalWriteFast(GNSS_PPS_RELAY, LOW);
  relay_timer_active = false;
}

// ============================================================================
// ISR-level hardware snapshots and residuals (canary diagnostics)
// ============================================================================

static volatile uint32_t isr_snap_dwt  = 0;
static volatile uint32_t isr_snap_gnss = 0;
static volatile uint32_t isr_snap_ocxo = 0;

static volatile uint32_t isr_prev_dwt  = 0;
static volatile uint32_t isr_prev_gnss = 0;
static volatile uint32_t isr_prev_ocxo = 0;

static volatile int32_t  isr_residual_dwt   = 0;
static volatile int32_t  isr_residual_gnss  = 0;
static volatile int32_t  isr_residual_ocxo  = 0;
static volatile bool     isr_residual_valid = false;

static constexpr uint32_t ISR_DWT_EXPECTED  = 1008000000u;
static constexpr uint32_t ISR_GNSS_EXPECTED =   10000000u;
static constexpr uint32_t ISR_OCXO_EXPECTED =   10000000u;

// ============================================================================
// PPS VCLOCK validation — reject spurious edges
// ============================================================================

static constexpr uint32_t PPS_VCLOCK_TOLERANCE = 5;  // ±500 ns at 10 MHz

static volatile uint32_t diag_pps_rejected_total     = 0;
static volatile uint32_t diag_pps_rejected_remainder = 0;

// ============================================================================
// PPS state
// ============================================================================

static volatile bool     pps_fired = false;

// ============================================================================
// PPS residual tracking — Welford's, callback-level (EXISTING)
// ============================================================================

static constexpr int64_t DWT_EXPECTED_PER_PPS  = 1008000000LL;
static constexpr int64_t GNSS_EXPECTED_PER_PPS =   10000000LL;
static constexpr int64_t OCXO_EXPECTED_PER_PPS =   10000000LL;

struct pps_residual_t {
  uint64_t ticks_at_last_pps;
  int64_t  delta;
  int64_t  residual;
  bool     valid;
  uint64_t n;
  double   mean;
  double   m2;
};

static pps_residual_t residual_dwt  = {};
static pps_residual_t residual_gnss = {};
static pps_residual_t residual_ocxo = {};

static void residual_reset(pps_residual_t& r) {
  r.ticks_at_last_pps = 0;
  r.delta             = 0;
  r.residual          = 0;
  r.valid             = false;
  r.n                 = 0;
  r.mean              = 0.0;
  r.m2                = 0.0;
}

static void residual_update(pps_residual_t& r,
                            uint64_t ticks_now,
                            int64_t  expected) {
  if (r.ticks_at_last_pps > 0) {
    r.delta    = (int64_t)(ticks_now - r.ticks_at_last_pps);
    r.residual = r.delta - expected;
    r.valid    = true;
    r.n++;
    const double x      = (double)r.residual;
    const double delta  = x - r.mean;
    r.mean += delta / (double)r.n;
    const double delta2 = x - r.mean;
    r.m2 += delta * delta2;
  } else {
    r.delta    = 0;
    r.residual = 0;
    r.valid    = false;
  }
  r.ticks_at_last_pps = ticks_now;
}

static inline double residual_stddev(const pps_residual_t& r) {
  return (r.n >= 2) ? sqrt(r.m2 / (double)(r.n - 1)) : 0.0;
}

static inline double residual_stderr(const pps_residual_t& r) {
  return (r.n >= 2) ? sqrt(r.m2 / (double)(r.n - 1)) / sqrt((double)r.n) : 0.0;
}

// ============================================================================
// Trend-aware prediction tracking (v10)
// ============================================================================
//
// For DWT and OCXO, we track:
//   - The previous raw delta (prev_delta)
//   - The delta before that (prev_prev_delta), for trend computation
//   - Whether we have enough history to predict (need 2 prior deltas)
//   - A predicted value for the current second
//   - Welford statistics on the prediction residual (actual - predicted)
//
// The prediction is simple linear extrapolation:
//   predicted = prev_delta + (prev_delta - prev_prev_delta)
//             = 2 * prev_delta - prev_prev_delta
//
// This captures the first derivative of thermal drift.  For a crystal
// drifting at ~2 cycles/second/second, the prediction residual will be
// near zero (the trend is the friend).
//
// GNSS ticks are exact by definition (10 MHz phase-coherent) so no
// prediction is needed.
//

struct prediction_tracker_t {
  // History (raw deltas, in native units: cycles for DWT, ticks for OCXO)
  uint32_t prev_delta;
  uint32_t prev_prev_delta;
  uint32_t predicted;           // predicted delta for THIS second
  int32_t  pred_residual;       // actual - predicted (latest)
  uint8_t  history_count;       // 0, 1, or 2+ (need 2 to predict)
  bool     predicted_valid;     // true if we have a prediction for this second

  // Welford on prediction residuals
  uint64_t n;
  double   mean;
  double   m2;
};

static prediction_tracker_t pred_dwt  = {};
static prediction_tracker_t pred_ocxo = {};

static void prediction_reset(prediction_tracker_t& p) {
  p.prev_delta      = 0;
  p.prev_prev_delta = 0;
  p.predicted       = 0;
  p.pred_residual   = 0;
  p.history_count   = 0;
  p.predicted_valid = false;
  p.n               = 0;
  p.mean            = 0.0;
  p.m2              = 0.0;
}

// Call AFTER computing this second's raw delta.
// Updates prediction stats and shifts history for next second.
static void prediction_update(prediction_tracker_t& p, uint32_t actual_delta) {

  if (p.predicted_valid) {
    // We had a prediction for this second — score it
    p.pred_residual = (int32_t)actual_delta - (int32_t)p.predicted;

    p.n++;
    const double x  = (double)p.pred_residual;
    const double d1 = x - p.mean;
    p.mean += d1 / (double)p.n;
    const double d2 = x - p.mean;
    p.m2 += d1 * d2;
  }

  // Shift history: current becomes previous, previous becomes prev_prev
  p.prev_prev_delta = p.prev_delta;
  p.prev_delta      = actual_delta;

  if (p.history_count < 2) {
    p.history_count++;
  }

  // Compute prediction for NEXT second (available to callers immediately)
  if (p.history_count >= 2) {
    // Linear extrapolation: next = current + (current - previous)
    // = 2 * current - previous
    // Done in signed to handle the case where previous > current
    int64_t pred = 2 * (int64_t)p.prev_delta - (int64_t)p.prev_prev_delta;
    // Clamp to positive (a negative cycle count is nonsensical)
    p.predicted = (pred > 0) ? (uint32_t)pred : p.prev_delta;
    p.predicted_valid = true;
  } else {
    p.predicted_valid = false;
  }
}

static inline double prediction_stddev(const prediction_tracker_t& p) {
  return (p.n >= 2) ? sqrt(p.m2 / (double)(p.n - 1)) : 0.0;
}

static inline double prediction_stderr(const prediction_tracker_t& p) {
  return (p.n >= 2) ? sqrt(p.m2 / (double)(p.n - 1)) / sqrt((double)p.n) : 0.0;
}

// ============================================================================
// 64-bit accumulators — built from successive raw 32-bit deltas
// ============================================================================

static uint64_t dwt_cycles_64      = 0;
static uint32_t prev_dwt_at_pps    = 0;

static uint64_t ocxo_ticks_64      = 0;
static uint32_t prev_ocxo_at_pps   = 0;

// ============================================================================
// Rolling 64-bit extensions for general use (TimePop, reports)
// ============================================================================

static uint64_t dwt_rolling_64  = 0;
static uint32_t dwt_rolling_32  = 0;

// ============================================================================
// DWT (CPU cycle counter — 1008 MHz internal)
// ============================================================================

#define DEMCR               (*(volatile uint32_t *)0xE000EDFC)
#define DEMCR_TRCENA        (1 << 24)
#define DWT_CTRL            (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT          (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL_CYCCNTENA  (1 << 0)

static inline void dwt_enable(void) {
  DEMCR |= DEMCR_TRCENA;
  DWT_CYCCNT = 0;
  DWT_CTRL |= DWT_CTRL_CYCCNTENA;
  dwt_rolling_32 = DWT_CYCCNT;
}

uint64_t clocks_dwt_cycles_now(void) {
  uint32_t now = DWT_CYCCNT;
  dwt_rolling_64 += (uint32_t)(now - dwt_rolling_32);
  dwt_rolling_32 = now;
  return dwt_rolling_64;
}

uint64_t clocks_dwt_ns_now(void) {
  return dwt_cycles_to_ns(clocks_dwt_cycles_now());
}

// ============================================================================
// GNSS VCLOCK (10 MHz external via GPT2 — raw, polled)
// ============================================================================

static uint64_t gnss_ticks_64 = 0;
static uint32_t gnss_last_32  = 0;
static bool     gpt2_armed    = false;

static inline void enable_gpt2(void) {
  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON);
}

static void arm_gpt2_external(void) {
  if (gpt2_armed) return;

  enable_gpt2();

  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 8;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 =
      IOMUXC_PAD_HYS | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_DSE(4);
  IOMUXC_GPT2_IPP_IND_CLKIN_SELECT_INPUT = 1;

  GPT2_CR = 0;
  GPT2_SR = 0x3F;
  GPT2_PR = 0;
  GPT2_CR = GPT_CR_CLKSRC(3) | GPT_CR_FRR;
  GPT2_CR |= GPT_CR_EN;

  gnss_last_32 = GPT2_CNT;
  gpt2_armed   = true;
}

uint64_t clocks_gnss_ticks_now(void) {
  uint32_t now = GPT2_CNT;
  gnss_ticks_64 += (uint32_t)(now - gnss_last_32);
  gnss_last_32 = now;
  return gnss_ticks_64;
}

uint64_t clocks_gnss_ns_now(void) {
  if (campaign_state != clocks_campaign_state_t::STARTED) return 0;
  return clocks_gnss_ticks_now() * 100ull;
}

// ============================================================================
// OCXO (10 MHz external via GPT1 — raw, polled)
// ============================================================================

static bool gpt1_armed = false;

static inline void enable_gpt1(void) {
  CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON);
}

static uint64_t ocxo_rolling_64 = 0;
static uint32_t ocxo_rolling_32 = 0;

static void arm_gpt1_external(void) {
  if (gpt1_armed) return;

  enable_gpt1();

  *(portConfigRegister(25)) = 1;

  GPT1_CR = 0;
  GPT1_SR = 0x3F;
  GPT1_PR = 0;
  GPT1_CR = GPT_CR_CLKSRC(3) | GPT_CR_FRR;
  GPT1_CR |= GPT_CR_EN;

  ocxo_rolling_32 = GPT1_CNT;
  gpt1_armed      = true;
}

uint64_t clocks_ocxo_ticks_now(void) {
  uint32_t now = GPT1_CNT;
  ocxo_rolling_64 += (uint32_t)(now - ocxo_rolling_32);
  ocxo_rolling_32 = now;
  return ocxo_rolling_64;
}

uint64_t clocks_ocxo_ns_now(void) {
  if (campaign_state != clocks_campaign_state_t::STARTED) return 0;
  return clocks_ocxo_ticks_now() * 100ull;
}

// ============================================================================
// Zeroing (campaign-scoped — fresh start only, NOT used for recovery)
// ============================================================================

static void clocks_zero_all(void) {
  timebase_invalidate();

  dwt_cycles_64    = 0;
  prev_dwt_at_pps  = isr_snap_dwt - ISR_ENTRY_DWT_CYCLES;

  ocxo_ticks_64    = 0;
  prev_ocxo_at_pps = isr_snap_ocxo;

  campaign_seconds = 0;

  dwt_rolling_64  = 0;  dwt_rolling_32  = DWT_CYCCNT;
  gnss_ticks_64   = 0;  gnss_last_32    = GPT2_CNT;
  ocxo_rolling_64 = 0;  ocxo_rolling_32 = GPT1_CNT;

  residual_reset(residual_dwt);
  residual_reset(residual_gnss);
  residual_reset(residual_ocxo);

  // v10: reset prediction trackers
  prediction_reset(pred_dwt);
  prediction_reset(pred_ocxo);

  isr_prev_dwt       = isr_snap_dwt;
  isr_prev_gnss      = isr_snap_gnss;
  isr_prev_ocxo      = isr_snap_ocxo;
  isr_residual_valid = false;

  pps_fired = false;
}

// ============================================================================
// OCXO calibration servo
// ============================================================================

static void ocxo_calibration_servo(void) {
  if (!calibrate_ocxo_active) return;
  if (residual_ocxo.n < SERVO_MIN_SAMPLES) return;

  servo_settle_count++;
  if (servo_settle_count < SERVO_SETTLE_SECONDS) return;

  double mean_residual = residual_ocxo.mean;
  servo_last_residual  = mean_residual;

  if (fabs(mean_residual) < 0.01) return;

  double step = -mean_residual * 0.5;
  if (step >  (double)SERVO_MAX_STEP) step =  (double)SERVO_MAX_STEP;
  if (step < -(double)SERVO_MAX_STEP) step = -(double)SERVO_MAX_STEP;

  ocxo_dac_set(ocxo_dac_fractional + step);

  servo_step = (int32_t)(step >= 0.0 ? step + 0.5 : step - 0.5);
  servo_adjustments++;
  servo_settle_count = 0;

  residual_reset(residual_ocxo);
}

// ============================================================================
// PPS handling — deferred ASAP callback (scheduled context)
// ============================================================================

static volatile bool pps_scheduled = false;

static void pps_asap_callback(timepop_ctx_t*, void*) {

  if (relay_arm_pending) {
    relay_arm_pending = false;
    if (!relay_timer_active) {
      relay_timer_active = true;
      timepop_arm(PPS_RELAY_OFF_NS, false, pps_relay_deassert, nullptr, "pps-relay-off");
    }
  }

  if (request_stop) {
    campaign_state = clocks_campaign_state_t::STOPPED;
    request_stop   = false;
    pps_fired      = true;
    calibrate_ocxo_active = false;
    timebase_invalidate();
  }

  if (request_recover) {

    timebase_invalidate();

    dwt_cycles_64    = dwt_ns_to_cycles(recover_dwt_ns);
    prev_dwt_at_pps  = isr_snap_dwt - ISR_ENTRY_DWT_CYCLES;

    ocxo_ticks_64    = recover_ocxo_ns / 100ull;
    prev_ocxo_at_pps = isr_snap_ocxo;

    gnss_ticks_64 = recover_gnss_ns / 100ull;
    gnss_last_32  = isr_snap_gnss;

    campaign_seconds = recover_gnss_ns / 1000000000ull;

    dwt_rolling_64  = 0;  dwt_rolling_32  = DWT_CYCCNT;
    ocxo_rolling_64 = 0;  ocxo_rolling_32 = GPT1_CNT;

    residual_reset(residual_dwt);
    residual_reset(residual_gnss);
    residual_reset(residual_ocxo);

    // v10: reset prediction trackers on recovery
    prediction_reset(pred_dwt);
    prediction_reset(pred_ocxo);

    isr_residual_valid = false;
    pps_fired          = true;

    campaign_state  = clocks_campaign_state_t::STARTED;
    request_recover = false;
  }

  if (request_start) {
    clocks_zero_all();
    campaign_state = clocks_campaign_state_t::STARTED;
    request_start  = false;
  }

  if (campaign_state == clocks_campaign_state_t::STARTED) {

    if (!isr_residual_valid) {
      isr_residual_valid = true;
    }

    pps_fired = false;

    // ── Clock values at PPS edge — direct delta accumulation ──

    // GNSS: arithmetic (phase coherent — exact by definition)
    const uint64_t pps_gnss_ns = campaign_seconds * NS_PER_SECOND;

    // DWT: ISR capture, corrected to PPS edge.
    uint32_t dwt_raw_at_pps = isr_snap_dwt - ISR_ENTRY_DWT_CYCLES;
    uint32_t dwt_delta = dwt_raw_at_pps - prev_dwt_at_pps;
    dwt_cycles_64   += dwt_delta;
    prev_dwt_at_pps  = dwt_raw_at_pps;

    const uint64_t pps_dwt_cycles = dwt_cycles_64;
    const uint64_t pps_dwt_ns     = dwt_cycles_to_ns(pps_dwt_cycles);

    // OCXO: delta from ISR snapshot
    uint32_t ocxo_delta = isr_snap_ocxo - prev_ocxo_at_pps;
    ocxo_ticks_64    += ocxo_delta;
    prev_ocxo_at_pps  = isr_snap_ocxo;

    const uint64_t pps_ocxo_ticks = ocxo_ticks_64;
    const uint64_t pps_ocxo_ns    = pps_ocxo_ticks * 100ull;

    // GNSS ticks for residual tracking (phase coherent)
    const uint64_t pps_gnss_ticks = campaign_seconds * (uint64_t)ISR_GNSS_EXPECTED;

    // ── Existing residual updates (unchanged) ──
    residual_update(residual_dwt,  pps_dwt_cycles,  DWT_EXPECTED_PER_PPS);
    residual_update(residual_gnss, pps_gnss_ticks,  GNSS_EXPECTED_PER_PPS);
    residual_update(residual_ocxo, pps_ocxo_ticks,  OCXO_EXPECTED_PER_PPS);

    // ── v10: Prediction tracking ──
    //
    // prediction_update() scores the prediction we made last second
    // (if any), shifts history, and computes a prediction for NEXT
    // second.  The predicted value is immediately available as
    // pred_dwt.predicted / pred_ocxo.predicted.
    //
    prediction_update(pred_dwt,  dwt_delta);
    prediction_update(pred_ocxo, ocxo_delta);

    ocxo_calibration_servo();

    // ── Build and publish TIMEBASE_FRAGMENT ──
    Payload p;
    p.add("campaign",         campaign_name);

    p.add("dwt_cycles",       pps_dwt_cycles);
    p.add("dwt_ns",           pps_dwt_ns);
    p.add("gnss_ns",          pps_gnss_ns);
    p.add("ocxo_ns",          pps_ocxo_ns);
    p.add("teensy_pps_count", campaign_seconds);
    p.add("gnss_lock",        digitalRead(GNSS_LOCK_PIN));
    p.add("dwt_cyccnt_at_pps", (uint32_t)dwt_raw_at_pps);
    p.add("gpt2_at_pps",      (uint32_t)isr_snap_gnss);

    // DWT cycles in the second that just completed.
    //
    // v10: If the prediction tracker has a valid prediction, use it
    // as the interpolation denominator.  The predicted value accounts
    // for the crystal's thermal drift trend and is a better estimate
    // of the current second's true rate than the raw prior delta.
    //
    // On pps_count == 0 there is no prior second, so use nominal.
    // During the first two seconds (history_count < 2), fall back
    // to the raw delta.
    if (campaign_seconds == 0) {
      p.add("dwt_cycles_per_pps", (uint64_t)DWT_EXPECTED_PER_PPS);
    } else if (pred_dwt.predicted_valid) {
      p.add("dwt_cycles_per_pps", (uint64_t)pred_dwt.predicted);
    } else {
      p.add("dwt_cycles_per_pps", (uint64_t)dwt_delta);
    }

    // Raw delta (always published for diagnostics and raw_cycles tool)
    p.add("dwt_delta_raw",    (uint64_t)dwt_delta);
    p.add("ocxo_delta_raw",   (uint64_t)ocxo_delta);

    // Existing residual stats
    p.add("dwt_pps_residual",  residual_dwt.residual);
    p.add("gnss_pps_residual", residual_gnss.residual);
    p.add("ocxo_pps_residual", residual_ocxo.residual);

    p.add("isr_residual_dwt",   isr_residual_dwt);
    p.add("isr_residual_gnss",  isr_residual_gnss);
    p.add("isr_residual_ocxo",  isr_residual_ocxo);
    p.add("isr_residual_valid", isr_residual_valid);

    // ── v10: Prediction statistics ──
    p.add("dwt_pred_residual",  pred_dwt.pred_residual);
    p.add("dwt_pred_mean",      pred_dwt.mean);
    p.add("dwt_pred_stddev",    prediction_stddev(pred_dwt));
    p.add("dwt_pred_n",         pred_dwt.n);

    p.add("ocxo_pred_residual", pred_ocxo.pred_residual);
    p.add("ocxo_pred_mean",     pred_ocxo.mean);
    p.add("ocxo_pred_stddev",   prediction_stddev(pred_ocxo));
    p.add("ocxo_pred_n",        pred_ocxo.n);

    // OCXO control state
    p.add("ocxo_dac",          ocxo_dac_fractional);
    p.add("calibrate_ocxo",    calibrate_ocxo_active);
    p.add("servo_adjustments", servo_adjustments);

    p.add("diag_pps_rejected_total",     diag_pps_rejected_total);
    p.add("diag_pps_rejected_remainder", diag_pps_rejected_remainder);

    publish("TIMEBASE_FRAGMENT", p);

    if (campaign_state == clocks_campaign_state_t::STARTED) {
      campaign_seconds++;
    }
  }

  pps_scheduled = false;
}

// ============================================================================
// PPS handling — ISR (minimum latency)
// ============================================================================

static void pps_isr(void) {

  const uint32_t snap_dwt  = DWT_CYCCNT;
  const uint32_t snap_gnss = GPT2_CNT;
  const uint32_t snap_ocxo = GPT1_CNT;

  // ── VCLOCK validation: reject spurious PPS edges ──
  if (isr_residual_valid) {
    uint32_t elapsed = snap_gnss - isr_prev_gnss;
    uint32_t remainder = elapsed % ISR_GNSS_EXPECTED;
    if (remainder > PPS_VCLOCK_TOLERANCE &&
        remainder < (ISR_GNSS_EXPECTED - PPS_VCLOCK_TOLERANCE)) {
      diag_pps_rejected_total++;
      diag_pps_rejected_remainder = remainder;
      return;
    }
  }

  pps_fired = true;

  digitalWriteFast(GNSS_PPS_RELAY, HIGH);
  relay_arm_pending = true;

  if (isr_residual_valid) {
    isr_residual_dwt  = (int32_t)(snap_dwt  - isr_prev_dwt)  - (int32_t)ISR_DWT_EXPECTED;
    isr_residual_gnss = (int32_t)(snap_gnss - isr_prev_gnss) - (int32_t)ISR_GNSS_EXPECTED;
    isr_residual_ocxo = (int32_t)(snap_ocxo - isr_prev_ocxo) - (int32_t)ISR_OCXO_EXPECTED;
  }

  isr_prev_dwt  = isr_snap_dwt  = snap_dwt;
  isr_prev_gnss = isr_snap_gnss = snap_gnss;
  isr_prev_ocxo = isr_snap_ocxo = snap_ocxo;

  if (pps_scheduled) return;
  pps_scheduled = true;

  timepop_arm(0, false, pps_asap_callback, nullptr, "pps");
}

// ============================================================================
// Dither callback — runs forever at 1 kHz
// ============================================================================

static void ocxo_dither_cb(timepop_ctx_t*, void*) {
  uint32_t base      = (uint32_t)ocxo_dac_fractional;
  double   frac      = ocxo_dac_fractional - (double)base;
  uint32_t threshold = (uint32_t)(frac * DITHER_PERIOD);

  dither_cycle = (dither_cycle + 1) % DITHER_PERIOD;

  uint32_t output = (dither_cycle < threshold) ? base + 1 : base;
  if (output > OCXO_DAC_MAX) output = OCXO_DAC_MAX;
  analogWrite(OCXO_CTL_PIN, output);
}

// ============================================================================
// Commands
// ============================================================================

static Payload cmd_start(const Payload& args) {
  const char* name = args.getString("campaign");
  if (!name || !*name) {
    Payload err;
    err.add("error", "missing campaign");
    return err;
  }

  safeCopy(campaign_name, sizeof(campaign_name), name);

  double dac_val;
  if (args.tryGetDouble("set_dac", dac_val)) ocxo_dac_set(dac_val);

  calibrate_ocxo_active = args.has("calibrate_ocxo");
  if (calibrate_ocxo_active) {
    servo_step          = 0;
    servo_last_residual = 0;
    servo_settle_count  = 0;
    servo_adjustments   = 0;
  }

  request_start = true;
  request_stop  = false;

  Payload p;
  p.add("status",         "start_requested");
  p.add("ocxo_dac",       ocxo_dac_fractional);
  p.add("calibrate_ocxo", calibrate_ocxo_active);
  return p;
}

static Payload cmd_stop(const Payload&) {
  request_stop  = true;
  request_start = false;
  Payload p;
  p.add("status", "stop_requested");
  return p;
}

static Payload cmd_recover(const Payload& args) {
  const char* s_dwt_ns = args.getString("dwt_ns");
  const char* s_gnss   = args.getString("gnss_ns");
  const char* s_ocxo   = args.getString("ocxo_ns");

  if (!s_dwt_ns || !s_gnss || !s_ocxo) {
    Payload err;
    err.add("error", "missing recovery parameters (dwt_ns, gnss_ns, ocxo_ns)");
    return err;
  }

  const char* name = args.getString("campaign");
  if (name && *name) {
    safeCopy(campaign_name, sizeof(campaign_name), name);
  }

  recover_dwt_ns  = strtoull(s_dwt_ns, nullptr, 10);
  recover_gnss_ns = strtoull(s_gnss,   nullptr, 10);
  recover_ocxo_ns = strtoull(s_ocxo,   nullptr, 10);

  double dac_val;
  if (args.tryGetDouble("set_dac", dac_val)) ocxo_dac_set(dac_val);

  calibrate_ocxo_active = args.has("calibrate_ocxo");
  if (calibrate_ocxo_active) {
    servo_step          = 0;
    servo_last_residual = 0.0;
    servo_settle_count  = 0;
  }

  request_recover = true;
  request_start   = false;
  request_stop    = false;

  Payload p;
  p.add("status", "recover_requested");
  return p;
}

// ============================================================================
// Helper — residual stats into a Payload
// ============================================================================

static void report_residual(Payload& p, const char* prefix,
                            const pps_residual_t& r) {
  char key[48];
  snprintf(key, sizeof(key), "%s_pps_valid",    prefix); p.add(key, r.valid);
  snprintf(key, sizeof(key), "%s_pps_delta",    prefix); p.add(key, r.delta);
  snprintf(key, sizeof(key), "%s_pps_residual", prefix); p.add(key, r.residual);
  snprintf(key, sizeof(key), "%s_pps_n",        prefix); p.add(key, r.n);
  snprintf(key, sizeof(key), "%s_pps_mean",     prefix); p.add_fmt(key, "%.3f", r.mean);
  snprintf(key, sizeof(key), "%s_pps_stddev",   prefix); p.add_fmt(key, "%.3f", residual_stddev(r));
  snprintf(key, sizeof(key), "%s_pps_stderr",   prefix); p.add_fmt(key, "%.3f", residual_stderr(r));
}

// ============================================================================
// Helper — prediction stats into a Payload (v10)
// ============================================================================

static void report_prediction(Payload& p, const char* prefix,
                              const prediction_tracker_t& t) {
  char key[48];
  snprintf(key, sizeof(key), "%s_pred_residual", prefix); p.add(key, t.pred_residual);
  snprintf(key, sizeof(key), "%s_pred_n",        prefix); p.add(key, t.n);
  snprintf(key, sizeof(key), "%s_pred_mean",     prefix); p.add_fmt(key, "%.3f", t.mean);
  snprintf(key, sizeof(key), "%s_pred_stddev",   prefix); p.add_fmt(key, "%.3f", prediction_stddev(t));
  snprintf(key, sizeof(key), "%s_pred_stderr",   prefix); p.add_fmt(key, "%.3f", prediction_stderr(t));
  snprintf(key, sizeof(key), "%s_pred_valid",    prefix); p.add(key, t.predicted_valid);
  snprintf(key, sizeof(key), "%s_pred_predicted", prefix); p.add(key, t.predicted);
  snprintf(key, sizeof(key), "%s_delta_raw",     prefix); p.add(key, t.prev_delta);
}

// ============================================================================
// REPORT — authoritative operational surface
// ============================================================================

static Payload cmd_report(const Payload&) {
  Payload p;

  p.add("campaign_state",
    campaign_state == clocks_campaign_state_t::STARTED ? "STARTED" : "STOPPED");

  if (campaign_state == clocks_campaign_state_t::STARTED) {
    p.add("campaign",         campaign_name);
    p.add("campaign_seconds", campaign_seconds);
  }

  p.add("request_start",   request_start);
  p.add("request_stop",    request_stop);
  p.add("request_recover", request_recover);

  const uint64_t dwt_cycles = clocks_dwt_cycles_now();
  const uint64_t dwt_ns     = clocks_dwt_ns_now();
  const uint64_t gnss_ns    = clocks_gnss_ns_now();
  const uint64_t ocxo_ns    = clocks_ocxo_ns_now();

  p.add("dwt_cycles_now", dwt_cycles);
  p.add("dwt_ns_now",     dwt_ns);
  p.add("gnss_ns_now",    gnss_ns);
  p.add("ocxo_ns_now",    ocxo_ns);
  p.add("gnss_lock",      digitalRead(GNSS_LOCK_PIN));

  const int64_t tb_gnss = timebase_now_ns(timebase_domain_t::GNSS);
  const int64_t tb_dwt  = timebase_now_ns(timebase_domain_t::DWT);
  const int64_t tb_ocxo = timebase_now_ns(timebase_domain_t::OCXO);

  p.add("timebase_gnss_ns", tb_gnss);
  p.add("timebase_dwt_ns",  tb_dwt);
  p.add("timebase_ocxo_ns", tb_ocxo);
  p.add("timebase_valid",   timebase_valid());

  report_residual(p, "dwt",  residual_dwt);
  report_residual(p, "gnss", residual_gnss);
  report_residual(p, "ocxo", residual_ocxo);

  // v10: prediction stats in REPORT
  report_prediction(p, "dwt",  pred_dwt);
  report_prediction(p, "ocxo", pred_ocxo);

  p.add("isr_residual_dwt",   isr_residual_dwt);
  p.add("isr_residual_gnss",  isr_residual_gnss);
  p.add("isr_residual_ocxo",  isr_residual_ocxo);
  p.add("isr_residual_valid", (bool)isr_residual_valid);

  p.add("ocxo_dac",            ocxo_dac_fractional);
  p.add("calibrate_ocxo",      calibrate_ocxo_active);
  p.add("servo_adjustments",   servo_adjustments);
  p.add("servo_step",          servo_step);
  p.add("servo_last_residual", servo_last_residual);
  p.add("servo_settle_count",  servo_settle_count);

  if (campaign_state == clocks_campaign_state_t::STARTED && gnss_ns > 0) {
    const double tau_dwt  = (double)dwt_ns  / (double)gnss_ns;
    const double tau_ocxo = (double)ocxo_ns / (double)gnss_ns;
    p.add_fmt("tau_dwt",  "%.12f", tau_dwt);
    p.add_fmt("tau_ocxo", "%.12f", tau_ocxo);
    p.add_fmt("dwt_ppb",  "%.3f",
      ((double)((int64_t)dwt_ns  - (int64_t)gnss_ns) / (double)gnss_ns) * 1e9);
    p.add_fmt("ocxo_ppb", "%.3f",
      ((double)((int64_t)ocxo_ns - (int64_t)gnss_ns) / (double)gnss_ns) * 1e9);
  } else {
    p.add("tau_dwt",  0.0);
    p.add("tau_ocxo", 0.0);
    p.add("dwt_ppb",  0.0);
    p.add("ocxo_ppb", 0.0);
  }

  return p;
}

// ============================================================================
// CLOCKS_INFO — forensic / diagnostic surface
// ============================================================================

static Payload cmd_clocks_info(const Payload&) {
  Payload p;

  p.add("campaign_state",
    campaign_state == clocks_campaign_state_t::STARTED ? "STARTED" : "STOPPED");

  if (campaign_state == clocks_campaign_state_t::STARTED) {
    p.add("campaign",         campaign_name);
    p.add("campaign_seconds", campaign_seconds);
  }

  const uint64_t dwt_cycles = clocks_dwt_cycles_now();
  const uint64_t dwt_ns     = clocks_dwt_ns_now();
  const uint64_t gnss_ns    = clocks_gnss_ns_now();
  const uint64_t ocxo_ns    = clocks_ocxo_ns_now();

  p.add("dwt_cycles_now", dwt_cycles);
  p.add("dwt_ns_now",     dwt_ns);
  p.add("gnss_ns_now",    gnss_ns);
  p.add("ocxo_ns_now",    ocxo_ns);

  const int64_t tb_gnss = timebase_now_ns(timebase_domain_t::GNSS);
  const int64_t tb_dwt  = timebase_now_ns(timebase_domain_t::DWT);
  const int64_t tb_ocxo = timebase_now_ns(timebase_domain_t::OCXO);

  const bool tb_valid            = timebase_valid();
  const bool tb_conversion_valid = timebase_conversion_valid();

  p.add("timebase_gnss_ns",          tb_gnss);
  p.add("timebase_dwt_ns",           tb_dwt);
  p.add("timebase_ocxo_ns",          tb_ocxo);
  p.add("timebase_valid",            tb_valid);
  p.add("timebase_conversion_valid", tb_conversion_valid);

  const timebase_fragment_t* tb_frag = timebase_last_fragment();

  if (tb_frag && tb_frag->valid) {
    p.add("timebase_fragment_valid",             true);
    p.add("timebase_fragment_gnss_ns",           (uint64_t)tb_frag->gnss_ns);
    p.add("timebase_fragment_dwt_ns",            (uint64_t)tb_frag->dwt_ns);
    p.add("timebase_fragment_dwt_cycles",        (uint64_t)tb_frag->dwt_cycles);
    p.add("timebase_fragment_ocxo_ns",           (uint64_t)tb_frag->ocxo_ns);
    p.add("timebase_fragment_pps_count",         (uint32_t)tb_frag->pps_count);
    p.add("timebase_fragment_isr_residual_dwt",  (int32_t)tb_frag->isr_residual_dwt);
    p.add("timebase_fragment_isr_residual_gnss", (int32_t)tb_frag->isr_residual_gnss);
    p.add("timebase_fragment_isr_residual_ocxo", (int32_t)tb_frag->isr_residual_ocxo);
  } else {
    p.add("timebase_fragment_valid", false);
  }

  p.add("pps_rejected_total",     diag_pps_rejected_total);
  p.add("pps_rejected_remainder", diag_pps_rejected_remainder);

  return p;
}

// ============================================================================
// INTERP_TEST — Zero-gap interpolation accuracy test
// ============================================================================

static Payload cmd_interp_test(const Payload& args) {
  Payload p;

  if (campaign_state != clocks_campaign_state_t::STARTED) {
    p.add("error", "no active campaign");
    return p;
  }

  const int32_t count = args.getInt("count", 10);
  const int32_t n = (count < 1) ? 1 : (count > 100) ? 100 : count;

  int64_t  w_n     = 0;
  double   w_mean  = 0.0;
  double   w_m2    = 0.0;
  int64_t  err_min = INT64_MAX;
  int64_t  err_max = INT64_MIN;
  int32_t  outside_100ns = 0;

  uint64_t s0_frag_gnss_ns          = 0;
  uint32_t s0_frag_dwt_cyccnt_at_pps = 0;
  uint32_t s0_frag_dwt_cycles_per_pps = 0;
  uint32_t s0_frag_gpt2_at_pps      = 0;
  uint32_t s0_pps_count             = 0;

  uint32_t s0_dwt_cyccnt_now        = 0;
  uint32_t s0_gpt2_now              = 0;
  uint32_t s0_dwt_elapsed           = 0;
  uint64_t s0_dwt_ns_into_second    = 0;
  uint64_t s0_interp_gnss_ns        = 0;

  uint32_t s0_gpt2_since_pps        = 0;
  uint64_t s0_vclock_ns_into_second = 0;
  uint64_t s0_vclock_gnss_ns        = 0;

  int64_t  s0_error_ns              = 0;
  double   s0_position              = 0.0;

  for (int32_t i = 0; i < n; i++) {

    const timebase_fragment_t* frag = timebase_last_fragment();
    if (!frag || !frag->valid) {
      p.add("error", "timebase not valid");
      return p;
    }

    const uint64_t frag_gnss_ns            = frag->gnss_ns;
    const uint32_t frag_pps_count          = frag->pps_count;
    const uint32_t frag_dwt_cyccnt_at_pps  = frag->dwt_cyccnt_at_pps;
    const uint32_t frag_dwt_cycles_per_pps = (uint32_t)frag->dwt_cycles_per_pps;
    const uint32_t frag_gpt2_at_pps        = frag->gpt2_at_pps;

    if (frag_dwt_cycles_per_pps == 0 || frag_gpt2_at_pps == 0) {
      p.add("error", "fragment missing dwt_cycles_per_pps or gpt2_at_pps");
      return p;
    }

    const uint32_t dwt_now  = ARM_DWT_CYCCNT;
    const uint32_t gpt2_now = GPT2_CNT;

    const int64_t interp_signed = timebase_gnss_ns_from_dwt(
      dwt_now,
      frag_gnss_ns,
      frag_dwt_cyccnt_at_pps,
      frag_dwt_cycles_per_pps
    );
    if (interp_signed < 0) {
      p.add("error", "timebase_gnss_ns_from_dwt failed");
      return p;
    }
    const uint64_t interp_gnss_ns = (uint64_t)interp_signed;

    const uint32_t gpt2_since_pps = gpt2_now - frag_gpt2_at_pps;
    const uint64_t vclock_ns_into_second = (uint64_t)gpt2_since_pps * 100ULL;
    const uint64_t vclock_gnss_ns = frag_gnss_ns + vclock_ns_into_second;

    const int64_t error_ns = (int64_t)interp_gnss_ns - (int64_t)vclock_gnss_ns;

    const uint32_t dwt_elapsed = dwt_now - frag_dwt_cyccnt_at_pps;
    const uint64_t dwt_ns_into_second =
      (uint64_t)dwt_elapsed * 1000000000ULL / (uint64_t)frag_dwt_cycles_per_pps;

    w_n++;
    const double x  = (double)error_ns;
    const double d1 = x - w_mean;
    w_mean += d1 / (double)w_n;
    const double d2 = x - w_mean;
    w_m2 += d1 * d2;

    if (error_ns < err_min) err_min = error_ns;
    if (error_ns > err_max) err_max = error_ns;
    if (error_ns > 100 || error_ns < -100) outside_100ns++;

    if (i == 0) {
      s0_frag_gnss_ns            = frag_gnss_ns;
      s0_frag_dwt_cyccnt_at_pps  = frag_dwt_cyccnt_at_pps;
      s0_frag_dwt_cycles_per_pps = frag_dwt_cycles_per_pps;
      s0_frag_gpt2_at_pps        = frag_gpt2_at_pps;
      s0_pps_count               = frag_pps_count;

      s0_dwt_cyccnt_now          = dwt_now;
      s0_gpt2_now                = gpt2_now;
      s0_dwt_elapsed             = dwt_elapsed;
      s0_dwt_ns_into_second      = dwt_ns_into_second;
      s0_interp_gnss_ns          = interp_gnss_ns;

      s0_gpt2_since_pps          = gpt2_since_pps;
      s0_vclock_ns_into_second   = vclock_ns_into_second;
      s0_vclock_gnss_ns          = vclock_gnss_ns;

      s0_error_ns                = error_ns;
      s0_position                = (double)vclock_ns_into_second / 1000000000.0;
    }
  }

  const double w_stddev = (w_n >= 2) ? sqrt(w_m2 / (double)(w_n - 1)) : 0.0;
  const double w_stderr = (w_n >= 2) ? w_stddev / sqrt((double)w_n)    : 0.0;

  p.add("samples",         (int32_t)w_n);
  p.add("error_mean_ns",   w_mean);
  p.add("error_stddev_ns", w_stddev);
  p.add("error_stderr_ns", w_stderr);
  p.add("error_min_ns",    err_min);
  p.add("error_max_ns",    err_max);
  p.add("outside_100ns",   outside_100ns);

  p.add("s0_frag_gnss_ns",            s0_frag_gnss_ns);
  p.add("s0_frag_dwt_cyccnt_at_pps",  s0_frag_dwt_cyccnt_at_pps);
  p.add("s0_frag_dwt_cycles_per_pps", s0_frag_dwt_cycles_per_pps);
  p.add("s0_frag_gpt2_at_pps",        s0_frag_gpt2_at_pps);
  p.add("s0_pps_count",               s0_pps_count);

  p.add("s0_dwt_cyccnt_now",          s0_dwt_cyccnt_now);
  p.add("s0_gpt2_now",                s0_gpt2_now);

  p.add("s0_dwt_elapsed",             s0_dwt_elapsed);
  p.add("s0_dwt_ns_into_second",      s0_dwt_ns_into_second);
  p.add("s0_interp_gnss_ns",          s0_interp_gnss_ns);

  p.add("s0_gpt2_since_pps",          s0_gpt2_since_pps);
  p.add("s0_vclock_ns_into_second",   s0_vclock_ns_into_second);
  p.add("s0_vclock_gnss_ns",          s0_vclock_gnss_ns);

  p.add("s0_error_ns",                s0_error_ns);
  p.add("s0_position",                s0_position);

  p.add("campaign_seconds",           campaign_seconds);
  p.add("pps_rejected_total",         diag_pps_rejected_total);

  return p;
}

// ============================================================================
// INTERP_PROOF — Long-running interpolation accuracy proof
// ============================================================================

static int64_t  proof_n       = 0;
static double   proof_mean    = 0.0;
static double   proof_m2      = 0.0;
static int64_t  proof_err_min = 0;
static int64_t  proof_err_max = 0;
static int32_t  proof_outside_100 = 0;

static double   proof_pos_mean = 0.0;
static double   proof_pos_m2   = 0.0;
static double   proof_covar    = 0.0;

static Payload cmd_interp_proof(const Payload& args) {
  Payload p;

  if (campaign_state != clocks_campaign_state_t::STARTED) {
    p.add("error", "no active campaign");
    return p;
  }

  if (args.getInt("reset", 0)) {
    proof_n       = 0;
    proof_mean    = 0.0;
    proof_m2      = 0.0;
    proof_err_min = 0;
    proof_err_max = 0;
    proof_outside_100 = 0;
    proof_pos_mean = 0.0;
    proof_pos_m2   = 0.0;
    proof_covar    = 0.0;
  }

  const int64_t interp_signed = timebase_now_gnss_ns();
  if (interp_signed < 0) {
    p.add("error", "timebase_now_gnss_ns failed");
    return p;
  }
  const uint64_t interp_gnss_ns = (uint64_t)interp_signed;

  const uint32_t gpt2_now = GPT2_CNT;

  const timebase_fragment_t* frag = timebase_last_fragment();
  if (!frag || !frag->valid) {
    p.add("error", "timebase not valid");
    return p;
  }

  const uint64_t frag_gnss_ns = frag->gnss_ns;
  const uint32_t isr_gnss_val = isr_snap_gnss;
  const uint32_t gnss_ticks_since_pps = gpt2_now - isr_gnss_val;
  const uint64_t vclock_ns_into_second = (uint64_t)gnss_ticks_since_pps * 100ULL;
  const uint64_t vclock_gnss_ns = frag_gnss_ns + vclock_ns_into_second;

  const int64_t error_ns = (int64_t)interp_gnss_ns - (int64_t)vclock_gnss_ns;

  const double position = (double)vclock_ns_into_second / 1000000000.0;

  proof_n++;
  const double x  = (double)error_ns;
  const double d1 = x - proof_mean;
  proof_mean += d1 / (double)proof_n;
  const double d2 = x - proof_mean;
  proof_m2 += d1 * d2;

  if (proof_n == 1) {
    proof_err_min = error_ns;
    proof_err_max = error_ns;
  } else {
    if (error_ns < proof_err_min) proof_err_min = error_ns;
    if (error_ns > proof_err_max) proof_err_max = error_ns;
  }
  if (error_ns > 100 || error_ns < -100) proof_outside_100++;

  const double pd1 = position - proof_pos_mean;
  proof_pos_mean += pd1 / (double)proof_n;
  const double pd2 = position - proof_pos_mean;
  proof_pos_m2 += pd1 * pd2;

  proof_covar += d2 * pd1;

  const double stddev = (proof_n >= 2) ? sqrt(proof_m2 / (double)(proof_n - 1)) : 0.0;
  const double stderr_val = (proof_n >= 2) ? stddev / sqrt((double)proof_n) : 0.0;
  const double pos_stddev = (proof_n >= 2) ? sqrt(proof_pos_m2 / (double)(proof_n - 1)) : 0.0;

  const double correlation = (proof_n >= 2 && stddev > 0.0 && pos_stddev > 0.0)
    ? (proof_covar / (double)(proof_n - 1)) / (stddev * pos_stddev)
    : 0.0;

  p.add("n",               (int32_t)proof_n);
  p.add("error_ns",        error_ns);
  p.add("mean_ns",         proof_mean);
  p.add("stddev_ns",       stddev);
  p.add("stderr_ns",       stderr_val);
  p.add("min_ns",          proof_err_min);
  p.add("max_ns",          proof_err_max);
  p.add("outside_100ns",   proof_outside_100);

  p.add("position",        position);
  p.add("pos_mean",        proof_pos_mean);
  p.add("pos_stddev",      pos_stddev);
  p.add("correlation",     correlation);

  p.add("campaign_seconds", campaign_seconds);
  p.add("pps_count",       frag->pps_count);

  return p;
}

static const process_command_entry_t CLOCKS_COMMANDS[] = {
  { "START",        cmd_start        },
  { "STOP",         cmd_stop         },
  { "RECOVER",      cmd_recover      },
  { "REPORT",       cmd_report       },
  { "CLOCKS_INFO",  cmd_clocks_info  },
  { "INTERP_TEST",  cmd_interp_test  },
  { "INTERP_PROOF", cmd_interp_proof },
  { nullptr,        nullptr          }
};

static const process_subscription_entry_t CLOCKS_SUBSCRIPTIONS[] = {
  { "TIMEBASE_FRAGMENT", on_timebase_fragment },
  { nullptr, nullptr },
};

const process_subscription_entry_t* timebase_subscriptions(void) {
  return CLOCKS_SUBSCRIPTIONS;
}

static const process_vtable_t CLOCKS_PROCESS = {
  .process_id    = "CLOCKS",
  .commands      = CLOCKS_COMMANDS,
  .subscriptions = CLOCKS_SUBSCRIPTIONS
};

void process_clocks_register(void) {
  process_register("CLOCKS", &CLOCKS_PROCESS);
}

// ============================================================================
// Initialization — Phase 1 (hardware only, no TimePop dependency)
// ============================================================================

void process_clocks_init_hardware(void) {
  dwt_enable();
  arm_gpt2_external();
  arm_gpt1_external();
}

// ============================================================================
// Initialization — Phase 2 (full lifecycle, requires TimePop)
// ============================================================================

void process_clocks_init(void) {

  timebase_init();

  analogWriteResolution(12);
  ocxo_dac_set((double)OCXO_DAC_DEFAULT);

  timepop_arm(OCXO_DITHER_NS, true, ocxo_dither_cb, nullptr, "ocxo-dither");

  pinMode(GNSS_PPS_PIN,   INPUT);
  pinMode(GNSS_LOCK_PIN,  INPUT);
  pinMode(GNSS_PPS_RELAY, OUTPUT);
  digitalWriteFast(GNSS_PPS_RELAY, LOW);

  attachInterrupt(digitalPinToInterrupt(GNSS_PPS_PIN), pps_isr, RISING);
  NVIC_SET_PRIORITY(IRQ_GPIO6789, 0);
}