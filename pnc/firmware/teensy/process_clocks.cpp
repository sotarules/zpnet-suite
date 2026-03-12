// ============================================================================
// process_clocks.cpp — ZPNet CLOCKS (Teensy) — v11.1 Dual OCXO (QTimer edge fix)
// ============================================================================
//
// CLOCKS is a dormant, PPS-anchored measurement instrument.
//
// • Boots fully silent
// • Does NOT track PPS unless a campaign is active or pending
// • Nanosecond clocks are ZERO unless a campaign is STARTED
// • All four counters are always live (capability, not narration)
//
// START / STOP / RECOVER are *requests*.
// All authoritative state transitions occur on the PPS boundary.
//
// Clock domains:
//
//   DWT    — ARM Cortex-M7 cycle counter, 1008 MHz, internal
//   GNSS   — GF-8802 10 MHz VCLOCK, external via GPT2 pin 14
//   OCXO1  — AOCJY1-A #1 10 MHz, external via GPT1 pin 25
//   OCXO2  — AOCJY1-A #2 10 MHz, external via QTimer1 ch0+ch1 pin 10
//
// v11: Dual OCXO architecture.
//
//   OCXO2 is a second independent oven-controlled oscillator on a
//   separate power domain.  It is counted by QTimer1 (16-bit ch0
//   cascaded with ch1 for 32-bit range, 64-bit via delta accumulation).
//
//   Both OCXOs have independent DAC trim (PWM + dither), independent
//   prediction trackers, independent residual trackers, and independent
//   accumulation state.
//
//   Calibration mode is unified: when calibrate_ocxo is active, BOTH
//   OCXOs are servoed simultaneously toward 10,000,000 ticks/second.
//   Each has its own servo state (DAC value, step, settle count) but
//   the enable/disable is a single flag.
//
//   The dither callback runs at 1 kHz and writes both PWM pins in
//   the same invocation.
//
// v11.1: QTimer dual-edge correction.
//
//   The i.MX RT1062 QTimer in CM(2) mode ("count rising edges of
//   primary source") counts BOTH rising and falling edges of the
//   external 10 MHz signal, producing ~20 MHz raw counts.  This is
//   a known hardware behavior — the GPT timers (used for GNSS and
//   OCXO1) do not exhibit this because their external clock input
//   path (CLKSRC=3) has dedicated edge selection logic.
//
//   Fix: all OCXO2 raw QTimer counts are divided by 2 at the point
//   of conversion to 10 MHz ticks.  The raw 20 MHz values are
//   preserved in ISR snapshots and isr_residual_ocxo2 for hardware
//   diagnostics.  The divided values flow into the 64-bit tick
//   accumulator, prediction tracker, and nanosecond conversion —
//   making OCXO2 ticks semantically identical to OCXO1 ticks.
//
//   The ±0.5 tick quantization from integer division is negligible:
//   50 ns against a prediction stddev of ~100 ns.
//
// v10.1: Prediction Welford initialization guard (unchanged).
// v10:   Trend-aware prediction statistics (unchanged).
// v9:    ISR-synchronized anchors (unchanged).
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

static uint64_t recover_dwt_ns   = 0;
static uint64_t recover_gnss_ns  = 0;
static uint64_t recover_ocxo1_ns = 0;
static uint64_t recover_ocxo2_ns = 0;

static uint64_t campaign_seconds = 0;

// ============================================================================
// OCXO DAC state — dual oscillator
// ============================================================================

struct ocxo_dac_state_t {
  double   dac_fractional;
  uint32_t dither_cycle;
  int32_t  servo_step;
  double   servo_last_residual;
  uint32_t servo_settle_count;
  uint32_t servo_adjustments;
};

static ocxo_dac_state_t ocxo1_dac = {
  (double)OCXO1_DAC_DEFAULT, 0, 0, 0.0, 0, 0
};

static ocxo_dac_state_t ocxo2_dac = {
  (double)OCXO2_DAC_DEFAULT, 0, 0, 0.0, 0, 0
};

static constexpr uint32_t DITHER_PERIOD = 1000;

static bool     calibrate_ocxo_active = false;

static constexpr int32_t  SERVO_MAX_STEP       = 64;
static constexpr uint32_t SERVO_SETTLE_SECONDS = 5;
static constexpr uint32_t SERVO_MIN_SAMPLES    = 10;

static void ocxo_dac_set(ocxo_dac_state_t& s, double value) {
  if (value < (double)OCXO1_DAC_MIN) value = (double)OCXO1_DAC_MIN;
  if (value > (double)OCXO1_DAC_MAX) value = (double)OCXO1_DAC_MAX;
  s.dac_fractional = value;
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
// ISR-level hardware snapshots and residuals
// ============================================================================

static volatile uint32_t isr_snap_dwt   = 0;
static volatile uint32_t isr_snap_gnss  = 0;
static volatile uint32_t isr_snap_ocxo1 = 0;
static volatile uint32_t isr_snap_ocxo2 = 0;

static volatile uint32_t isr_prev_dwt   = 0;
static volatile uint32_t isr_prev_gnss  = 0;
static volatile uint32_t isr_prev_ocxo1 = 0;
static volatile uint32_t isr_prev_ocxo2 = 0;

static volatile int32_t  isr_residual_dwt   = 0;
static volatile int32_t  isr_residual_gnss  = 0;
static volatile int32_t  isr_residual_ocxo1 = 0;
static volatile int32_t  isr_residual_ocxo2 = 0;
static volatile bool     isr_residual_valid = false;

static constexpr uint32_t ISR_DWT_EXPECTED  = DWT_EXPECTED_PER_PPS;
static constexpr uint32_t ISR_GNSS_EXPECTED = TICKS_10MHZ_PER_SECOND;
static constexpr uint32_t ISR_OCXO_EXPECTED = TICKS_10MHZ_PER_SECOND;

// QTimer1 counts both edges of the 10 MHz signal (hardware behavior).
// Raw counts are 2x the actual 10 MHz tick rate.
static constexpr uint32_t OCXO2_EDGE_DIVISOR    = 2;
static constexpr uint32_t ISR_OCXO2_RAW_EXPECTED = TICKS_10MHZ_PER_SECOND * OCXO2_EDGE_DIVISOR;

// ============================================================================
// PPS VCLOCK validation — reject spurious edges
// ============================================================================

static volatile uint32_t diag_pps_rejected_total     = 0;
static volatile uint32_t diag_pps_rejected_remainder = 0;

// ============================================================================
// PPS state
// ============================================================================

static volatile bool     pps_fired = false;

// ============================================================================
// PPS residual tracking — Welford's, callback-level
// ============================================================================

static constexpr int64_t DWT_EXPECTED_PER_PPS_I  = (int64_t)DWT_EXPECTED_PER_PPS;
static constexpr int64_t GNSS_EXPECTED_PER_PPS   = (int64_t)TICKS_10MHZ_PER_SECOND;
static constexpr int64_t OCXO_EXPECTED_PER_PPS   = (int64_t)TICKS_10MHZ_PER_SECOND;

struct pps_residual_t {
  uint64_t ticks_at_last_pps;
  int64_t  delta;
  int64_t  residual;
  bool     valid;
  uint64_t n;
  double   mean;
  double   m2;
};

static pps_residual_t residual_dwt   = {};
static pps_residual_t residual_gnss  = {};
static pps_residual_t residual_ocxo1 = {};
static pps_residual_t residual_ocxo2 = {};

static void residual_reset(pps_residual_t& r) {
  r.ticks_at_last_pps = 0;
  r.delta             = 0;
  r.residual          = 0;
  r.valid             = false;
  r.n                 = 0;
  r.mean              = 0.0;
  r.m2                = 0.0;
}

static void residual_update(pps_residual_t& r, uint64_t ticks_now, int64_t expected) {
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
// Trend-aware prediction tracking (v10, v10.1)
// ============================================================================

struct prediction_tracker_t {
  uint32_t prev_delta;
  uint32_t prev_prev_delta;
  uint32_t predicted;
  int32_t  pred_residual;
  uint8_t  history_count;
  bool     predicted_valid;
  uint64_t n;
  double   mean;
  double   m2;
};

static prediction_tracker_t pred_dwt   = {};
static prediction_tracker_t pred_ocxo1 = {};
static prediction_tracker_t pred_ocxo2 = {};

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

static void prediction_update(prediction_tracker_t& p, uint32_t actual_delta) {
  if (p.predicted_valid && p.history_count >= 3) {
    p.pred_residual = (int32_t)actual_delta - (int32_t)p.predicted;
    p.n++;
    const double x  = (double)p.pred_residual;
    const double d1 = x - p.mean;
    p.mean += d1 / (double)p.n;
    const double d2 = x - p.mean;
    p.m2 += d1 * d2;
  }
  p.prev_prev_delta = p.prev_delta;
  p.prev_delta      = actual_delta;
  if (p.history_count < 255) p.history_count++;
  if (p.history_count >= 2) {
    int64_t pred = 2 * (int64_t)p.prev_delta - (int64_t)p.prev_prev_delta;
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

static uint64_t dwt_cycles_64       = 0;
static uint32_t prev_dwt_at_pps     = 0;

static uint64_t ocxo1_ticks_64      = 0;
static uint32_t prev_ocxo1_at_pps   = 0;

static uint64_t ocxo2_ticks_64      = 0;
static uint32_t prev_ocxo2_at_pps   = 0;

// ============================================================================
// Rolling 64-bit extensions for general use
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
// GNSS VCLOCK (10 MHz external via GPT2)
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
// OCXO1 (10 MHz external via GPT1)
// ============================================================================

static bool gpt1_armed = false;

static inline void enable_gpt1(void) {
  CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON);
}

static uint64_t ocxo1_rolling_64 = 0;
static uint32_t ocxo1_rolling_32 = 0;

static void arm_gpt1_external(void) {
  if (gpt1_armed) return;
  enable_gpt1();
  *(portConfigRegister(OCXO1_10MHZ_PIN)) = 1;
  GPT1_CR = 0;
  GPT1_SR = 0x3F;
  GPT1_PR = 0;
  GPT1_CR = GPT_CR_CLKSRC(3) | GPT_CR_FRR;
  GPT1_CR |= GPT_CR_EN;
  ocxo1_rolling_32 = GPT1_CNT;
  gpt1_armed       = true;
}

uint64_t clocks_ocxo1_ticks_now(void) {
  uint32_t now = GPT1_CNT;
  ocxo1_rolling_64 += (uint32_t)(now - ocxo1_rolling_32);
  ocxo1_rolling_32 = now;
  return ocxo1_rolling_64;
}

uint64_t clocks_ocxo1_ns_now(void) {
  if (campaign_state != clocks_campaign_state_t::STARTED) return 0;
  return clocks_ocxo1_ticks_now() * 100ull;
}

// ============================================================================
// OCXO2 (10 MHz external via QTimer1 ch0+ch1 — cascaded 32-bit)
//
// IMPORTANT: QTimer CM(2) counts both rising and falling edges,
// producing ~20 MHz raw counts for a 10 MHz input.  All conversions
// from raw QTimer counts to 10 MHz ticks divide by OCXO2_EDGE_DIVISOR.
// ISR snapshots and isr_residual_ocxo2 remain in raw 20 MHz space.
// ============================================================================

static bool qtimer1_armed = false;

static uint64_t ocxo2_rolling_64 = 0;
static uint32_t ocxo2_rolling_32 = 0;

static inline uint32_t qtimer1_read_32(void) {
  const uint16_t lo = IMXRT_TMR1.CH[0].CNTR;
  const uint16_t hi = IMXRT_TMR1.CH[1].CNTR;
  return ((uint32_t)hi << 16) | (uint32_t)lo;
}

static void arm_qtimer1_external(void) {
  if (qtimer1_armed) return;

  CCM_CCGR6 |= CCM_CCGR6_QTIMER1(CCM_CCGR_ON);

  // Pin 10 = GPIO_B0_00, ALT1 = QTIMER1_TIMER0
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 1;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 =
      IOMUXC_PAD_HYS | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_DSE(4);

  // Only touch channels 0 and 1 (our cascade pair).
  // Do NOT touch channels 2 and 3 — pin 11 (OCXO2 CTL) uses
  // FlexPWM1 submodule 2, which shares the QTimer1 module.
  // Zeroing CH[2] clobbers pin 11's PWM output.
  IMXRT_TMR1.CH[0].CTRL = 0;
  IMXRT_TMR1.CH[1].CTRL = 0;

  IMXRT_TMR1.CH[0].CNTR = 0;
  IMXRT_TMR1.CH[1].CNTR = 0;

  IMXRT_TMR1.CH[0].SCTRL  = 0;
  IMXRT_TMR1.CH[0].CSCTRL = 0;
  IMXRT_TMR1.CH[1].SCTRL  = 0;
  IMXRT_TMR1.CH[1].CSCTRL = 0;

  IMXRT_TMR1.CH[0].LOAD = 0;
  IMXRT_TMR1.CH[1].LOAD = 0;

  IMXRT_TMR1.CH[0].COMP1 = 0xFFFF;
  IMXRT_TMR1.CH[1].COMP1 = 0xFFFF;
  IMXRT_TMR1.CH[0].CMPLD1 = 0xFFFF;
  IMXRT_TMR1.CH[1].CMPLD1 = 0xFFFF;

  // ch0: count edges of external pin (CM=2, PCS=0)
  // NOTE: CM(2) counts BOTH edges — raw rate is 2x input frequency
  IMXRT_TMR1.CH[0].CTRL = TMR_CTRL_CM(2) | TMR_CTRL_PCS(0);

  // ch1: cascade from ch0 overflow (CM=7)
  IMXRT_TMR1.CH[1].CTRL = TMR_CTRL_CM(7);

  ocxo2_rolling_32 = qtimer1_read_32();
  qtimer1_armed    = true;
}

uint64_t clocks_ocxo2_ticks_now(void) {
  uint32_t now = qtimer1_read_32();
  ocxo2_rolling_64 += (uint32_t)(now - ocxo2_rolling_32) / OCXO2_EDGE_DIVISOR;
  ocxo2_rolling_32 = now;
  return ocxo2_rolling_64;
}

uint64_t clocks_ocxo2_ns_now(void) {
  if (campaign_state != clocks_campaign_state_t::STARTED) return 0;
  return clocks_ocxo2_ticks_now() * 100ull;
}

// ============================================================================
// Zeroing (campaign-scoped — fresh start only)
// ============================================================================

static void clocks_zero_all(void) {
  timebase_invalidate();

  dwt_cycles_64     = 0;
  prev_dwt_at_pps   = isr_snap_dwt - ISR_ENTRY_DWT_CYCLES;

  ocxo1_ticks_64    = 0;
  prev_ocxo1_at_pps = isr_snap_ocxo1;

  ocxo2_ticks_64    = 0;
  prev_ocxo2_at_pps = isr_snap_ocxo2;

  campaign_seconds  = 0;

  dwt_rolling_64    = 0;  dwt_rolling_32    = DWT_CYCCNT;
  gnss_ticks_64     = 0;  gnss_last_32      = GPT2_CNT;
  ocxo1_rolling_64  = 0;  ocxo1_rolling_32  = GPT1_CNT;
  ocxo2_rolling_64  = 0;  ocxo2_rolling_32  = qtimer1_read_32();

  residual_reset(residual_dwt);
  residual_reset(residual_gnss);
  residual_reset(residual_ocxo1);
  residual_reset(residual_ocxo2);

  prediction_reset(pred_dwt);
  prediction_reset(pred_ocxo1);
  prediction_reset(pred_ocxo2);

  isr_prev_dwt       = isr_snap_dwt;
  isr_prev_gnss      = isr_snap_gnss;
  isr_prev_ocxo1     = isr_snap_ocxo1;
  isr_prev_ocxo2     = isr_snap_ocxo2;
  isr_residual_valid = false;

  pps_fired = false;
}

// ============================================================================
// OCXO calibration servo — unified for both OCXOs
// ============================================================================

static void ocxo_calibration_servo_one(ocxo_dac_state_t& dac, pps_residual_t& res) {
  if (res.n < SERVO_MIN_SAMPLES) return;

  dac.servo_settle_count++;
  if (dac.servo_settle_count < SERVO_SETTLE_SECONDS) return;

  double mean_residual = res.mean;
  dac.servo_last_residual = mean_residual;

  if (fabs(mean_residual) < 0.01) return;

  double step = -mean_residual * 0.5;
  if (step >  (double)SERVO_MAX_STEP) step =  (double)SERVO_MAX_STEP;
  if (step < -(double)SERVO_MAX_STEP) step = -(double)SERVO_MAX_STEP;

  ocxo_dac_set(dac, dac.dac_fractional + step);

  dac.servo_step = (int32_t)(step >= 0.0 ? step + 0.5 : step - 0.5);
  dac.servo_adjustments++;
  dac.servo_settle_count = 0;

  residual_reset(res);
}

static void ocxo_calibration_servo(void) {
  if (!calibrate_ocxo_active) return;
  ocxo_calibration_servo_one(ocxo1_dac, residual_ocxo1);
  ocxo_calibration_servo_one(ocxo2_dac, residual_ocxo2);
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

    dwt_cycles_64     = dwt_ns_to_cycles(recover_dwt_ns);
    prev_dwt_at_pps   = isr_snap_dwt - ISR_ENTRY_DWT_CYCLES;

    ocxo1_ticks_64    = recover_ocxo1_ns / 100ull;
    prev_ocxo1_at_pps = isr_snap_ocxo1;

    ocxo2_ticks_64    = recover_ocxo2_ns / 100ull;
    prev_ocxo2_at_pps = isr_snap_ocxo2;

    gnss_ticks_64 = recover_gnss_ns / 100ull;
    gnss_last_32  = isr_snap_gnss;

    campaign_seconds = recover_gnss_ns / 1000000000ull;

    dwt_rolling_64    = 0;  dwt_rolling_32    = DWT_CYCCNT;
    ocxo1_rolling_64  = 0;  ocxo1_rolling_32  = GPT1_CNT;
    ocxo2_rolling_64  = 0;  ocxo2_rolling_32  = qtimer1_read_32();  // raw 20 MHz space

    residual_reset(residual_dwt);
    residual_reset(residual_gnss);
    residual_reset(residual_ocxo1);
    residual_reset(residual_ocxo2);

    prediction_reset(pred_dwt);
    prediction_reset(pred_ocxo1);
    prediction_reset(pred_ocxo2);

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

    const uint64_t pps_gnss_ns = campaign_seconds * NS_PER_SECOND;

    uint32_t dwt_raw_at_pps = isr_snap_dwt - ISR_ENTRY_DWT_CYCLES;
    uint32_t dwt_delta = dwt_raw_at_pps - prev_dwt_at_pps;
    dwt_cycles_64   += dwt_delta;
    prev_dwt_at_pps  = dwt_raw_at_pps;

    const uint64_t pps_dwt_cycles = dwt_cycles_64;
    const uint64_t pps_dwt_ns     = dwt_cycles_to_ns(pps_dwt_cycles);

    uint32_t ocxo1_delta = isr_snap_ocxo1 - prev_ocxo1_at_pps;
    ocxo1_ticks_64    += ocxo1_delta;
    prev_ocxo1_at_pps  = isr_snap_ocxo1;

    const uint64_t pps_ocxo1_ticks = ocxo1_ticks_64;
    const uint64_t pps_ocxo1_ns    = pps_ocxo1_ticks * 100ull;

    // OCXO2: QTimer raw counts are 2x (both edges).  Divide to get
    // 10 MHz ticks.  Raw value preserved for delta_raw diagnostic.
    uint32_t ocxo2_raw_delta = isr_snap_ocxo2 - prev_ocxo2_at_pps;
    uint32_t ocxo2_delta     = ocxo2_raw_delta / OCXO2_EDGE_DIVISOR;
    ocxo2_ticks_64    += ocxo2_delta;
    prev_ocxo2_at_pps  = isr_snap_ocxo2;

    const uint64_t pps_ocxo2_ticks = ocxo2_ticks_64;
    const uint64_t pps_ocxo2_ns    = pps_ocxo2_ticks * 100ull;

    const uint64_t pps_gnss_ticks = campaign_seconds * (uint64_t)ISR_GNSS_EXPECTED;

    // ── Residual updates ──
    residual_update(residual_dwt,   pps_dwt_cycles,   DWT_EXPECTED_PER_PPS_I);
    residual_update(residual_gnss,  pps_gnss_ticks,   GNSS_EXPECTED_PER_PPS);
    residual_update(residual_ocxo1, pps_ocxo1_ticks,  OCXO_EXPECTED_PER_PPS);
    residual_update(residual_ocxo2, pps_ocxo2_ticks,  OCXO_EXPECTED_PER_PPS);

    // ── Prediction tracking ──
    prediction_update(pred_dwt,   dwt_delta);
    prediction_update(pred_ocxo1, ocxo1_delta);
    prediction_update(pred_ocxo2, ocxo2_delta);

    ocxo_calibration_servo();

    // ── Build and publish TIMEBASE_FRAGMENT ──
    Payload p;
    p.add("campaign",         campaign_name);

    p.add("dwt_cycles",       pps_dwt_cycles);
    p.add("dwt_ns",           pps_dwt_ns);
    p.add("gnss_ns",          pps_gnss_ns);
    p.add("ocxo1_ns",         pps_ocxo1_ns);
    p.add("ocxo2_ns",         pps_ocxo2_ns);
    p.add("teensy_pps_count", campaign_seconds);
    p.add("gnss_lock",        digitalRead(GNSS_LOCK_PIN));
    p.add("dwt_cyccnt_at_pps", (uint32_t)dwt_raw_at_pps);
    p.add("gpt2_at_pps",      (uint32_t)isr_snap_gnss);

    if (campaign_seconds == 0) {
      p.add("dwt_cycles_per_pps", (uint64_t)DWT_EXPECTED_PER_PPS);
    } else if (pred_dwt.predicted_valid) {
      p.add("dwt_cycles_per_pps", (uint64_t)pred_dwt.predicted);
    } else {
      p.add("dwt_cycles_per_pps", (uint64_t)dwt_delta);
    }

    p.add("dwt_delta_raw",    (uint64_t)dwt_delta);
    p.add("ocxo1_delta_raw",  (uint64_t)ocxo1_delta);
    p.add("ocxo2_delta_raw",  (uint64_t)ocxo2_raw_delta);

    p.add("dwt_pps_residual",   residual_dwt.residual);
    p.add("gnss_pps_residual",  residual_gnss.residual);
    p.add("ocxo1_pps_residual", residual_ocxo1.residual);
    p.add("ocxo2_pps_residual", residual_ocxo2.residual);

    p.add("isr_residual_dwt",   isr_residual_dwt);
    p.add("isr_residual_gnss",  isr_residual_gnss);
    p.add("isr_residual_ocxo1", isr_residual_ocxo1);
    p.add("isr_residual_ocxo2", isr_residual_ocxo2);
    p.add("isr_residual_valid", isr_residual_valid);

    p.add("dwt_pred_residual",  pred_dwt.pred_residual);
    p.add("dwt_pred_mean",      pred_dwt.mean);
    p.add("dwt_pred_stddev",    prediction_stddev(pred_dwt));
    p.add("dwt_pred_n",         pred_dwt.n);

    p.add("ocxo1_pred_residual", pred_ocxo1.pred_residual);
    p.add("ocxo1_pred_mean",     pred_ocxo1.mean);
    p.add("ocxo1_pred_stddev",   prediction_stddev(pred_ocxo1));
    p.add("ocxo1_pred_n",        pred_ocxo1.n);

    p.add("ocxo2_pred_residual", pred_ocxo2.pred_residual);
    p.add("ocxo2_pred_mean",     pred_ocxo2.mean);
    p.add("ocxo2_pred_stddev",   prediction_stddev(pred_ocxo2));
    p.add("ocxo2_pred_n",        pred_ocxo2.n);

    p.add("ocxo1_dac",         ocxo1_dac.dac_fractional);
    p.add("ocxo2_dac",         ocxo2_dac.dac_fractional);
    p.add("calibrate_ocxo",    calibrate_ocxo_active);
    p.add("ocxo1_servo_adjustments", ocxo1_dac.servo_adjustments);
    p.add("ocxo2_servo_adjustments", ocxo2_dac.servo_adjustments);

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

  const uint32_t snap_dwt   = DWT_CYCCNT;
  const uint32_t snap_gnss  = GPT2_CNT;
  const uint32_t snap_ocxo1 = GPT1_CNT;
  const uint32_t snap_ocxo2 = qtimer1_read_32();

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
    isr_residual_dwt   = (int32_t)(snap_dwt   - isr_prev_dwt)   - (int32_t)ISR_DWT_EXPECTED;
    isr_residual_gnss  = (int32_t)(snap_gnss  - isr_prev_gnss)  - (int32_t)ISR_GNSS_EXPECTED;
    isr_residual_ocxo1 = (int32_t)(snap_ocxo1 - isr_prev_ocxo1) - (int32_t)ISR_OCXO_EXPECTED;
    isr_residual_ocxo2 = (int32_t)(snap_ocxo2 - isr_prev_ocxo2) - (int32_t)ISR_OCXO2_RAW_EXPECTED;
  }

  isr_prev_dwt   = isr_snap_dwt   = snap_dwt;
  isr_prev_gnss  = isr_snap_gnss  = snap_gnss;
  isr_prev_ocxo1 = isr_snap_ocxo1 = snap_ocxo1;
  isr_prev_ocxo2 = isr_snap_ocxo2 = snap_ocxo2;

  if (pps_scheduled) return;
  pps_scheduled = true;

  timepop_arm(0, false, pps_asap_callback, nullptr, "pps");
}

// ============================================================================
// Dither callback — runs forever at 1 kHz, writes both OCXO PWM pins
// ============================================================================

static void ocxo_dither_one(ocxo_dac_state_t& s, int pin) {
  uint32_t base      = (uint32_t)s.dac_fractional;
  double   frac      = s.dac_fractional - (double)base;
  uint32_t threshold = (uint32_t)(frac * DITHER_PERIOD);

  s.dither_cycle = (s.dither_cycle + 1) % DITHER_PERIOD;

  uint32_t output = (s.dither_cycle < threshold) ? base + 1 : base;
  if (output > OCXO1_DAC_MAX) output = OCXO1_DAC_MAX;
  analogWrite(pin, output);
}

static void ocxo_dither_cb(timepop_ctx_t*, void*) {
  ocxo_dither_one(ocxo1_dac, OCXO1_CTL_PIN);
  ocxo_dither_one(ocxo2_dac, OCXO2_CTL_PIN);
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
  if (args.tryGetDouble("set_dac", dac_val))  ocxo_dac_set(ocxo1_dac, dac_val);
  if (args.tryGetDouble("set_dac2", dac_val)) ocxo_dac_set(ocxo2_dac, dac_val);

  calibrate_ocxo_active = args.has("calibrate_ocxo");
  if (calibrate_ocxo_active) {
    ocxo1_dac.servo_step = 0; ocxo1_dac.servo_last_residual = 0.0;
    ocxo1_dac.servo_settle_count = 0; ocxo1_dac.servo_adjustments = 0;
    ocxo2_dac.servo_step = 0; ocxo2_dac.servo_last_residual = 0.0;
    ocxo2_dac.servo_settle_count = 0; ocxo2_dac.servo_adjustments = 0;
  }

  request_start = true;
  request_stop  = false;

  Payload p;
  p.add("status",         "start_requested");
  p.add("ocxo1_dac",      ocxo1_dac.dac_fractional);
  p.add("ocxo2_dac",      ocxo2_dac.dac_fractional);
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
  const char* s_ocxo1  = args.getString("ocxo1_ns");
  const char* s_ocxo2  = args.getString("ocxo2_ns");

  if (!s_dwt_ns || !s_gnss || !s_ocxo1 || !s_ocxo2) {
    Payload err;
    err.add("error", "missing recovery parameters (dwt_ns, gnss_ns, ocxo1_ns, ocxo2_ns)");
    return err;
  }

  const char* name = args.getString("campaign");
  if (name && *name) safeCopy(campaign_name, sizeof(campaign_name), name);

  recover_dwt_ns   = strtoull(s_dwt_ns, nullptr, 10);
  recover_gnss_ns  = strtoull(s_gnss,   nullptr, 10);
  recover_ocxo1_ns = strtoull(s_ocxo1,  nullptr, 10);
  recover_ocxo2_ns = strtoull(s_ocxo2,  nullptr, 10);

  double dac_val;
  if (args.tryGetDouble("set_dac", dac_val))  ocxo_dac_set(ocxo1_dac, dac_val);
  if (args.tryGetDouble("set_dac2", dac_val)) ocxo_dac_set(ocxo2_dac, dac_val);

  calibrate_ocxo_active = args.has("calibrate_ocxo");
  if (calibrate_ocxo_active) {
    ocxo1_dac.servo_step = 0; ocxo1_dac.servo_last_residual = 0.0;
    ocxo1_dac.servo_settle_count = 0;
    ocxo2_dac.servo_step = 0; ocxo2_dac.servo_last_residual = 0.0;
    ocxo2_dac.servo_settle_count = 0;
  }

  request_recover = true;
  request_start   = false;
  request_stop    = false;

  Payload p;
  p.add("status", "recover_requested");
  return p;
}

// ============================================================================
// Helpers — residual & prediction stats into a Payload
// ============================================================================

static void report_residual(Payload& p, const char* prefix, const pps_residual_t& r) {
  char key[48];
  snprintf(key, sizeof(key), "%s_pps_valid",    prefix); p.add(key, r.valid);
  snprintf(key, sizeof(key), "%s_pps_delta",    prefix); p.add(key, r.delta);
  snprintf(key, sizeof(key), "%s_pps_residual", prefix); p.add(key, r.residual);
  snprintf(key, sizeof(key), "%s_pps_n",        prefix); p.add(key, r.n);
  snprintf(key, sizeof(key), "%s_pps_mean",     prefix); p.add_fmt(key, "%.3f", r.mean);
  snprintf(key, sizeof(key), "%s_pps_stddev",   prefix); p.add_fmt(key, "%.3f", residual_stddev(r));
  snprintf(key, sizeof(key), "%s_pps_stderr",   prefix); p.add_fmt(key, "%.3f", residual_stderr(r));
}

static void report_prediction(Payload& p, const char* prefix, const prediction_tracker_t& t) {
  char key[48];
  snprintf(key, sizeof(key), "%s_pred_residual",  prefix); p.add(key, t.pred_residual);
  snprintf(key, sizeof(key), "%s_pred_n",         prefix); p.add(key, t.n);
  snprintf(key, sizeof(key), "%s_pred_mean",      prefix); p.add_fmt(key, "%.3f", t.mean);
  snprintf(key, sizeof(key), "%s_pred_stddev",    prefix); p.add_fmt(key, "%.3f", prediction_stddev(t));
  snprintf(key, sizeof(key), "%s_pred_stderr",    prefix); p.add_fmt(key, "%.3f", prediction_stderr(t));
  snprintf(key, sizeof(key), "%s_pred_valid",     prefix); p.add(key, t.predicted_valid);
  snprintf(key, sizeof(key), "%s_pred_predicted", prefix); p.add(key, t.predicted);
  snprintf(key, sizeof(key), "%s_delta_raw",      prefix); p.add(key, t.prev_delta);
}

// ============================================================================
// REPORT
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
  const uint64_t ocxo1_ns   = clocks_ocxo1_ns_now();
  const uint64_t ocxo2_ns   = clocks_ocxo2_ns_now();

  p.add("dwt_cycles_now", dwt_cycles);
  p.add("dwt_ns_now",     dwt_ns);
  p.add("gnss_ns_now",    gnss_ns);
  p.add("ocxo1_ns_now",   ocxo1_ns);
  p.add("ocxo2_ns_now",   ocxo2_ns);
  p.add("gnss_lock",      digitalRead(GNSS_LOCK_PIN));

  const int64_t tb_gnss  = timebase_now_ns(timebase_domain_t::GNSS);
  const int64_t tb_dwt   = timebase_now_ns(timebase_domain_t::DWT);
  const int64_t tb_ocxo1 = timebase_now_ns(timebase_domain_t::OCXO1);
  const int64_t tb_ocxo2 = timebase_now_ns(timebase_domain_t::OCXO2);

  p.add("timebase_gnss_ns",  tb_gnss);
  p.add("timebase_dwt_ns",   tb_dwt);
  p.add("timebase_ocxo1_ns", tb_ocxo1);
  p.add("timebase_ocxo2_ns", tb_ocxo2);
  p.add("timebase_valid",    timebase_valid());

  report_residual(p, "dwt",   residual_dwt);
  report_residual(p, "gnss",  residual_gnss);
  report_residual(p, "ocxo1", residual_ocxo1);
  report_residual(p, "ocxo2", residual_ocxo2);

  report_prediction(p, "dwt",   pred_dwt);
  report_prediction(p, "ocxo1", pred_ocxo1);
  report_prediction(p, "ocxo2", pred_ocxo2);

  p.add("isr_residual_dwt",   isr_residual_dwt);
  p.add("isr_residual_gnss",  isr_residual_gnss);
  p.add("isr_residual_ocxo1", isr_residual_ocxo1);
  p.add("isr_residual_ocxo2", isr_residual_ocxo2);
  p.add("isr_residual_valid", (bool)isr_residual_valid);

  p.add("ocxo1_dac",               ocxo1_dac.dac_fractional);
  p.add("ocxo2_dac",               ocxo2_dac.dac_fractional);
  p.add("calibrate_ocxo",          calibrate_ocxo_active);
  p.add("ocxo1_servo_adjustments", ocxo1_dac.servo_adjustments);
  p.add("ocxo2_servo_adjustments", ocxo2_dac.servo_adjustments);
  p.add("ocxo1_servo_step",        ocxo1_dac.servo_step);
  p.add("ocxo2_servo_step",        ocxo2_dac.servo_step);
  p.add("ocxo1_servo_last_residual", ocxo1_dac.servo_last_residual);
  p.add("ocxo2_servo_last_residual", ocxo2_dac.servo_last_residual);
  p.add("ocxo1_servo_settle_count",  ocxo1_dac.servo_settle_count);
  p.add("ocxo2_servo_settle_count",  ocxo2_dac.servo_settle_count);

  if (campaign_state == clocks_campaign_state_t::STARTED && gnss_ns > 0) {
    p.add_fmt("tau_dwt",   "%.12f", (double)dwt_ns   / (double)gnss_ns);
    p.add_fmt("tau_ocxo1", "%.12f", (double)ocxo1_ns / (double)gnss_ns);
    p.add_fmt("tau_ocxo2", "%.12f", (double)ocxo2_ns / (double)gnss_ns);
    p.add_fmt("dwt_ppb",   "%.3f", ((double)((int64_t)dwt_ns   - (int64_t)gnss_ns) / (double)gnss_ns) * 1e9);
    p.add_fmt("ocxo1_ppb", "%.3f", ((double)((int64_t)ocxo1_ns - (int64_t)gnss_ns) / (double)gnss_ns) * 1e9);
    p.add_fmt("ocxo2_ppb", "%.3f", ((double)((int64_t)ocxo2_ns - (int64_t)gnss_ns) / (double)gnss_ns) * 1e9);
  } else {
    p.add("tau_dwt", 0.0); p.add("tau_ocxo1", 0.0); p.add("tau_ocxo2", 0.0);
    p.add("dwt_ppb", 0.0); p.add("ocxo1_ppb", 0.0); p.add("ocxo2_ppb", 0.0);
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

  p.add("dwt_cycles_now", clocks_dwt_cycles_now());
  p.add("dwt_ns_now",     clocks_dwt_ns_now());
  p.add("gnss_ns_now",    clocks_gnss_ns_now());
  p.add("ocxo1_ns_now",   clocks_ocxo1_ns_now());
  p.add("ocxo2_ns_now",   clocks_ocxo2_ns_now());

  p.add("timebase_gnss_ns",  timebase_now_ns(timebase_domain_t::GNSS));
  p.add("timebase_dwt_ns",   timebase_now_ns(timebase_domain_t::DWT));
  p.add("timebase_ocxo1_ns", timebase_now_ns(timebase_domain_t::OCXO1));
  p.add("timebase_ocxo2_ns", timebase_now_ns(timebase_domain_t::OCXO2));
  p.add("timebase_valid",            timebase_valid());
  p.add("timebase_conversion_valid", timebase_conversion_valid());

  const timebase_fragment_t* tb_frag = timebase_last_fragment();
  if (tb_frag && tb_frag->valid) {
    p.add("timebase_fragment_valid",              true);
    p.add("timebase_fragment_gnss_ns",            (uint64_t)tb_frag->gnss_ns);
    p.add("timebase_fragment_dwt_ns",             (uint64_t)tb_frag->dwt_ns);
    p.add("timebase_fragment_dwt_cycles",         (uint64_t)tb_frag->dwt_cycles);
    p.add("timebase_fragment_ocxo1_ns",           (uint64_t)tb_frag->ocxo1_ns);
    p.add("timebase_fragment_ocxo2_ns",           (uint64_t)tb_frag->ocxo2_ns);
    p.add("timebase_fragment_pps_count",          (uint32_t)tb_frag->pps_count);
    p.add("timebase_fragment_isr_residual_dwt",   (int32_t)tb_frag->isr_residual_dwt);
    p.add("timebase_fragment_isr_residual_gnss",  (int32_t)tb_frag->isr_residual_gnss);
    p.add("timebase_fragment_isr_residual_ocxo1", (int32_t)tb_frag->isr_residual_ocxo1);
    p.add("timebase_fragment_isr_residual_ocxo2", (int32_t)tb_frag->isr_residual_ocxo2);
  } else {
    p.add("timebase_fragment_valid", false);
  }

  p.add("pps_rejected_total",     diag_pps_rejected_total);
  p.add("pps_rejected_remainder", diag_pps_rejected_remainder);

  return p;
}

// ============================================================================
// INTERP_TEST — unchanged (DWT/GNSS only, OCXO not involved)
// ============================================================================

static Payload cmd_interp_test(const Payload& args) {
  Payload p;

  if (campaign_state != clocks_campaign_state_t::STARTED) {
    p.add("error", "no active campaign");
    return p;
  }

  const int32_t count = args.getInt("count", 10);
  const int32_t n = (count < 1) ? 1 : (count > 100) ? 100 : count;

  int64_t  w_n = 0; double w_mean = 0.0, w_m2 = 0.0;
  int64_t  err_min = INT64_MAX, err_max = INT64_MIN;
  int32_t  outside_100ns = 0;

  uint64_t s0_frag_gnss_ns = 0; uint32_t s0_frag_dwt_cyccnt_at_pps = 0;
  uint32_t s0_frag_dwt_cycles_per_pps = 0, s0_frag_gpt2_at_pps = 0, s0_pps_count = 0;
  uint32_t s0_dwt_cyccnt_now = 0, s0_gpt2_now = 0, s0_dwt_elapsed = 0;
  uint64_t s0_dwt_ns_into_second = 0, s0_interp_gnss_ns = 0;
  uint32_t s0_gpt2_since_pps = 0;
  uint64_t s0_vclock_ns_into_second = 0, s0_vclock_gnss_ns = 0;
  int64_t  s0_error_ns = 0; double s0_position = 0.0;

  for (int32_t i = 0; i < n; i++) {
    const timebase_fragment_t* frag = timebase_last_fragment();
    if (!frag || !frag->valid) { p.add("error", "timebase not valid"); return p; }

    const uint64_t frag_gnss_ns            = frag->gnss_ns;
    const uint32_t frag_pps_count          = frag->pps_count;
    const uint32_t frag_dwt_cyccnt_at_pps  = frag->dwt_cyccnt_at_pps;
    const uint32_t frag_dwt_cycles_per_pps = (uint32_t)frag->dwt_cycles_per_pps;
    const uint32_t frag_gpt2_at_pps        = frag->gpt2_at_pps;

    if (frag_dwt_cycles_per_pps == 0 || frag_gpt2_at_pps == 0) {
      p.add("error", "fragment missing dwt_cycles_per_pps or gpt2_at_pps"); return p;
    }

    const uint32_t dwt_now  = ARM_DWT_CYCCNT;
    const uint32_t gpt2_now = GPT2_CNT;

    const int64_t interp_signed = timebase_gnss_ns_from_dwt(
      dwt_now, frag_gnss_ns, frag_dwt_cyccnt_at_pps, frag_dwt_cycles_per_pps);
    if (interp_signed < 0) { p.add("error", "timebase_gnss_ns_from_dwt failed"); return p; }
    const uint64_t interp_gnss_ns = (uint64_t)interp_signed;

    const uint32_t gpt2_since_pps = gpt2_now - frag_gpt2_at_pps;
    const uint64_t vclock_ns_into_second = (uint64_t)gpt2_since_pps * 100ULL;
    const uint64_t vclock_gnss_ns = frag_gnss_ns + vclock_ns_into_second;
    const int64_t error_ns = (int64_t)interp_gnss_ns - (int64_t)vclock_gnss_ns;
    const uint32_t dwt_elapsed = dwt_now - frag_dwt_cyccnt_at_pps;
    const uint64_t dwt_ns_into_second =
      (uint64_t)dwt_elapsed * 1000000000ULL / (uint64_t)frag_dwt_cycles_per_pps;

    w_n++;
    const double x = (double)error_ns, d1 = x - w_mean;
    w_mean += d1 / (double)w_n;
    w_m2 += (x - w_mean) * d1;
    if (error_ns < err_min) err_min = error_ns;
    if (error_ns > err_max) err_max = error_ns;
    if (error_ns > 100 || error_ns < -100) outside_100ns++;

    if (i == 0) {
      s0_frag_gnss_ns = frag_gnss_ns; s0_frag_dwt_cyccnt_at_pps = frag_dwt_cyccnt_at_pps;
      s0_frag_dwt_cycles_per_pps = frag_dwt_cycles_per_pps; s0_frag_gpt2_at_pps = frag_gpt2_at_pps;
      s0_pps_count = frag_pps_count; s0_dwt_cyccnt_now = dwt_now; s0_gpt2_now = gpt2_now;
      s0_dwt_elapsed = dwt_elapsed; s0_dwt_ns_into_second = dwt_ns_into_second;
      s0_interp_gnss_ns = interp_gnss_ns; s0_gpt2_since_pps = gpt2_since_pps;
      s0_vclock_ns_into_second = vclock_ns_into_second; s0_vclock_gnss_ns = vclock_gnss_ns;
      s0_error_ns = error_ns; s0_position = (double)vclock_ns_into_second / 1000000000.0;
    }
  }

  const double w_stddev = (w_n >= 2) ? sqrt(w_m2 / (double)(w_n - 1)) : 0.0;
  p.add("samples", (int32_t)w_n); p.add("error_mean_ns", w_mean);
  p.add("error_stddev_ns", w_stddev);
  p.add("error_stderr_ns", (w_n >= 2) ? w_stddev / sqrt((double)w_n) : 0.0);
  p.add("error_min_ns", err_min); p.add("error_max_ns", err_max);
  p.add("outside_100ns", outside_100ns);
  p.add("s0_frag_gnss_ns", s0_frag_gnss_ns);
  p.add("s0_frag_dwt_cyccnt_at_pps", s0_frag_dwt_cyccnt_at_pps);
  p.add("s0_frag_dwt_cycles_per_pps", s0_frag_dwt_cycles_per_pps);
  p.add("s0_frag_gpt2_at_pps", s0_frag_gpt2_at_pps);
  p.add("s0_pps_count", s0_pps_count);
  p.add("s0_dwt_cyccnt_now", s0_dwt_cyccnt_now); p.add("s0_gpt2_now", s0_gpt2_now);
  p.add("s0_dwt_elapsed", s0_dwt_elapsed);
  p.add("s0_dwt_ns_into_second", s0_dwt_ns_into_second);
  p.add("s0_interp_gnss_ns", s0_interp_gnss_ns);
  p.add("s0_gpt2_since_pps", s0_gpt2_since_pps);
  p.add("s0_vclock_ns_into_second", s0_vclock_ns_into_second);
  p.add("s0_vclock_gnss_ns", s0_vclock_gnss_ns);
  p.add("s0_error_ns", s0_error_ns); p.add("s0_position", s0_position);
  p.add("campaign_seconds", campaign_seconds);
  p.add("pps_rejected_total", diag_pps_rejected_total);
  return p;
}

// ============================================================================
// INTERP_PROOF — unchanged
// ============================================================================

static int64_t proof_n = 0; static double proof_mean = 0.0, proof_m2 = 0.0;
static int64_t proof_err_min = 0, proof_err_max = 0;
static int32_t proof_outside_100 = 0;
static double  proof_pos_mean = 0.0, proof_pos_m2 = 0.0, proof_covar = 0.0;

static Payload cmd_interp_proof(const Payload& args) {
  Payload p;
  if (campaign_state != clocks_campaign_state_t::STARTED) {
    p.add("error", "no active campaign"); return p;
  }
  if (args.getInt("reset", 0)) {
    proof_n = 0; proof_mean = 0.0; proof_m2 = 0.0;
    proof_err_min = 0; proof_err_max = 0; proof_outside_100 = 0;
    proof_pos_mean = 0.0; proof_pos_m2 = 0.0; proof_covar = 0.0;
  }
  const int64_t interp_signed = timebase_now_gnss_ns();
  if (interp_signed < 0) { p.add("error", "timebase_now_gnss_ns failed"); return p; }
  const uint64_t interp_gnss_ns = (uint64_t)interp_signed;
  const uint32_t gpt2_now = GPT2_CNT;
  const timebase_fragment_t* frag = timebase_last_fragment();
  if (!frag || !frag->valid) { p.add("error", "timebase not valid"); return p; }
  const uint64_t frag_gnss_ns = frag->gnss_ns;
  const uint32_t gnss_ticks_since_pps = gpt2_now - isr_snap_gnss;
  const uint64_t vclock_ns_into_second = (uint64_t)gnss_ticks_since_pps * 100ULL;
  const uint64_t vclock_gnss_ns = frag_gnss_ns + vclock_ns_into_second;
  const int64_t error_ns = (int64_t)interp_gnss_ns - (int64_t)vclock_gnss_ns;
  const double position = (double)vclock_ns_into_second / 1000000000.0;

  proof_n++;
  const double x = (double)error_ns, d1 = x - proof_mean;
  proof_mean += d1 / (double)proof_n;
  const double d2 = x - proof_mean;
  proof_m2 += d1 * d2;
  if (proof_n == 1) { proof_err_min = error_ns; proof_err_max = error_ns; }
  else { if (error_ns < proof_err_min) proof_err_min = error_ns; if (error_ns > proof_err_max) proof_err_max = error_ns; }
  if (error_ns > 100 || error_ns < -100) proof_outside_100++;
  const double pd1 = position - proof_pos_mean;
  proof_pos_mean += pd1 / (double)proof_n;
  proof_pos_m2 += (position - proof_pos_mean) * pd1;
  proof_covar += d2 * pd1;

  const double stddev = (proof_n >= 2) ? sqrt(proof_m2 / (double)(proof_n - 1)) : 0.0;
  const double pos_stddev = (proof_n >= 2) ? sqrt(proof_pos_m2 / (double)(proof_n - 1)) : 0.0;
  const double correlation = (proof_n >= 2 && stddev > 0.0 && pos_stddev > 0.0)
    ? (proof_covar / (double)(proof_n - 1)) / (stddev * pos_stddev) : 0.0;

  p.add("n", (int32_t)proof_n); p.add("error_ns", error_ns);
  p.add("mean_ns", proof_mean); p.add("stddev_ns", stddev);
  p.add("stderr_ns", (proof_n >= 2) ? stddev / sqrt((double)proof_n) : 0.0);
  p.add("min_ns", proof_err_min); p.add("max_ns", proof_err_max);
  p.add("outside_100ns", proof_outside_100);
  p.add("position", position); p.add("pos_mean", proof_pos_mean);
  p.add("pos_stddev", pos_stddev); p.add("correlation", correlation);
  p.add("campaign_seconds", campaign_seconds); p.add("pps_count", frag->pps_count);
  return p;
}

// ============================================================================
// Process registration
// ============================================================================

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
  arm_qtimer1_external();
}

// ============================================================================
// Initialization — Phase 2 (full lifecycle, requires TimePop)
// ============================================================================

void process_clocks_init(void) {

  timebase_init();

  analogWriteResolution(12);

  ocxo_dac_set(ocxo1_dac, (double)OCXO1_DAC_DEFAULT);
  ocxo_dac_set(ocxo2_dac, (double)OCXO2_DAC_DEFAULT);

  timepop_arm(OCXO_DITHER_NS, true, ocxo_dither_cb, nullptr, "ocxo-dither");

  pinMode(GNSS_PPS_PIN,   INPUT);
  pinMode(GNSS_LOCK_PIN,  INPUT);
  pinMode(GNSS_PPS_RELAY, OUTPUT);
  digitalWriteFast(GNSS_PPS_RELAY, LOW);

  attachInterrupt(digitalPinToInterrupt(GNSS_PPS_PIN), pps_isr, RISING);
  NVIC_SET_PRIORITY(IRQ_GPIO6789, 0);
}