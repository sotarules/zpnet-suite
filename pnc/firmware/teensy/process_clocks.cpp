// ============================================================================
// process_clocks.cpp — ZPNet CLOCKS (Teensy) — v7 Direct Delta Accumulation
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
// v7: Direct delta accumulation.
//
//   64-bit accumulators are built from successive raw 32-bit register
//   deltas at each PPS edge.  No advance-then-rewind.  No intermediate
//   reads can corrupt the accumulator.
//
//   The v6 advance-then-rewind pattern called clocks_xxx_now() to
//   advance the accumulator to "now", then subtracted a rewind to
//   get the value "at the PPS edge".  But between the PPS ISR and
//   the ASAP callback, other code could call clocks_xxx_now() —
//   shifting the accumulator's phase and corrupting the rewind by
//   ±600 cycles.  This produced the oscillating discrepancy seen
//   in the cycle_analyzer.
//
//   Fix: at each PPS, compute the raw 32-bit delta from the previous
//   PPS and add it directly to the 64-bit accumulator.  The accumulator
//   is touched ONLY here, ONCE per second, with exact hardware deltas.
//
//   For DWT: delta = dispatch_shadow_dwt - prev_dwt_at_pps
//   For OCXO: delta = isr_snap_ocxo - prev_ocxo_at_pps
//   For GNSS: pure arithmetic (campaign_seconds × constant)
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
static constexpr uint64_t PRE_PPS_NS        =   999000000ULL;  // 999 ms

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
//
// The ISR captures raw 32-bit register values for three purposes:
//   1. VCLOCK validation gate (isr_snap_gnss vs isr_prev_gnss)
//   2. ISR residual canaries (should be near-zero for GNSS, stable for DWT/OCXO)
//   3. OCXO delta accumulation (isr_snap_ocxo is the per-PPS anchor)
//
// DWT uses the spin loop shadow, not the ISR capture.
// Neither DWT nor OCXO uses advance-then-rewind.

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
// Pre-PPS spin loop — DWT shadow capture
// ============================================================================
//
// The spin loop runs in the last ~1 ms before the PPS edge with
// BASEPRI masking.  It continuously writes DWT_CYCCNT to
// dispatch_shadow_dwt.  When the PPS ISR fires, pps_fired goes true
// and the loop exits.  The last value written IS the DWT cycle count
// closest to the true PPS edge — typically within ~50 ns.
//
// This value is used DIRECTLY as the DWT cycle count at PPS.
// No TDC correction, no overhead subtraction, no quantization.

static volatile uint32_t dispatch_shadow_dwt     = 0;
static volatile bool     pps_fired               = false;
static volatile uint32_t isr_captured_shadow_dwt = 0;
static volatile bool     dispatch_shadow_valid   = false;
static volatile bool     dispatch_timeout        = false;

static volatile int32_t  pre_pps_approach_ns     = -1;

// ============================================================================
// Dispatch profiling diagnostics
// ============================================================================

static volatile uint32_t diag_coarse_fire_count  = 0;
static volatile uint32_t diag_fine_fire_count    = 0;
static volatile uint32_t diag_fine_late_count    = 0;
static volatile uint32_t diag_spin_count         = 0;
static volatile uint32_t diag_timeout_count      = 0;
static volatile bool     diag_fine_was_late      = false;
static volatile uint32_t diag_spin_iterations    = 0;

// ============================================================================
// PPS residual tracking — Welford's, callback-level
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
// 64-bit accumulators — built from successive raw 32-bit deltas
// ============================================================================
//
// At each PPS edge, we compute the raw 32-bit delta from the previous
// edge and add it to the 64-bit accumulator.  The accumulator is
// touched ONLY in the ASAP callback, ONCE per second.
//
// DWT: uses spin loop shadow (dispatch_shadow_dwt)
// OCXO: uses ISR snapshot (isr_snap_ocxo)
// GNSS: pure arithmetic (campaign_seconds × constant)
//
// prev_dwt_at_pps and prev_ocxo_at_pps hold the raw 32-bit register
// value from the previous PPS edge, for delta computation.

static uint64_t dwt_cycles_64      = 0;
static uint32_t prev_dwt_at_pps    = 0;

static uint64_t ocxo_ticks_64      = 0;
static uint32_t prev_ocxo_at_pps   = 0;

// ============================================================================
// Rolling 64-bit extensions for general use (TimePop, reports)
// ============================================================================
//
// These are the "anytime" 64-bit extensions for code that needs to
// read DWT/GNSS/OCXO outside the PPS callback.  They use the
// traditional advance-on-read pattern and are NOT used for TIMEBASE.

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
//
// The rolling 64-bit extension is maintained for TimePop (which uses
// GPT2 for scheduling).  For TIMEBASE, GNSS ns is pure arithmetic:
// gnss_ns = campaign_seconds × NS_PER_SECOND (phase coherence).

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

  // Zero the TIMEBASE accumulators and anchor at this PPS edge
  dwt_cycles_64    = 0;
  prev_dwt_at_pps  = dispatch_shadow_valid ? dispatch_shadow_dwt : isr_snap_dwt;

  ocxo_ticks_64    = 0;
  prev_ocxo_at_pps = isr_snap_ocxo;

  campaign_seconds = 0;

  // Zero the rolling extensions (for reports / TimePop)
  dwt_rolling_64  = 0;  dwt_rolling_32  = DWT_CYCCNT;
  gnss_ticks_64   = 0;  gnss_last_32    = GPT2_CNT;
  ocxo_rolling_64 = 0;  ocxo_rolling_32 = GPT1_CNT;

  residual_reset(residual_dwt);
  residual_reset(residual_gnss);
  residual_reset(residual_ocxo);

  isr_prev_dwt       = isr_snap_dwt;
  isr_prev_gnss      = isr_snap_gnss;
  isr_prev_ocxo      = isr_snap_ocxo;
  isr_residual_valid = false;

  dispatch_shadow_dwt     = 0;
  isr_captured_shadow_dwt = 0;
  dispatch_shadow_valid   = false;
  dispatch_timeout        = false;
  pps_fired               = false;
  pre_pps_approach_ns     = -1;
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
// Pre-PPS dispatch latency profiling — spin loop
// ============================================================================

static inline void mask_low_priority_irqs(uint8_t pri) {
  __asm volatile("MSR BASEPRI, %0" : : "r"(pri) : "memory");
}

static inline void unmask_all_irqs() {
  __asm volatile("MSR BASEPRI, %0" : : "r"(0) : "memory");
}

static void pre_pps_fine_cb(timepop_ctx_t*, void*) {
  diag_fine_fire_count++;

  uint32_t ticks_since_pps = (uint32_t)(GPT2_CNT - isr_snap_gnss);
  pre_pps_approach_ns      = (int32_t)(ISR_GNSS_EXPECTED - ticks_since_pps) * 100;

  if (pps_fired) {
    diag_fine_late_count++;
    diag_fine_was_late    = true;
    diag_spin_iterations  = 0;
    dispatch_shadow_valid = false;
    return;
  }

  diag_fine_was_late = false;
  diag_spin_count++;

  mask_low_priority_irqs(32);
  for (;;) {
    dispatch_shadow_dwt = DWT_CYCCNT;
    if (pps_fired) break;
  }
  unmask_all_irqs();

  dispatch_shadow_valid = true;
  dispatch_timeout      = false;
  diag_spin_iterations  = 0;
}

static void pre_pps_arm(void) {
  timepop_arm(PRE_PPS_NS, false, pre_pps_fine_cb, nullptr, "pre-pps");
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
    timepop_cancel_by_name("pre-pps");
    dispatch_shadow_valid = false;
    pps_fired             = true;
    calibrate_ocxo_active = false;
    timebase_invalidate();
  }

  if (request_recover) {

    timebase_invalidate();

    // Seed the accumulators from recovered nanosecond values
    dwt_cycles_64    = dwt_ns_to_cycles(recover_dwt_ns);
    prev_dwt_at_pps  = dispatch_shadow_valid ? dispatch_shadow_dwt : isr_snap_dwt;

    ocxo_ticks_64    = recover_ocxo_ns / 100ull;
    prev_ocxo_at_pps = isr_snap_ocxo;

    gnss_ticks_64 = recover_gnss_ns / 100ull;
    gnss_last_32  = isr_snap_gnss;

    campaign_seconds = recover_gnss_ns / 1000000000ull;

    // Reset rolling extensions
    dwt_rolling_64  = 0;  dwt_rolling_32  = DWT_CYCCNT;
    ocxo_rolling_64 = 0;  ocxo_rolling_32 = GPT1_CNT;

    residual_reset(residual_dwt);
    residual_reset(residual_gnss);
    residual_reset(residual_ocxo);

    isr_residual_valid     = false;
    dispatch_shadow_valid  = false;
    pps_fired              = true;
    pre_pps_approach_ns    = -1;

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

    if (dispatch_timeout) {
      pre_pps_approach_ns = -1;
      dispatch_timeout    = false;
    }

    pre_pps_arm();
    pps_fired = false;

    // ── Clock values at PPS edge — direct delta accumulation ──
    //
    // Each accumulator advances by the exact raw 32-bit delta since
    // the previous PPS edge.  No advance-then-rewind.  No intermediate
    // reads.  The accumulator is touched ONLY here, ONCE per second.

    // GNSS: arithmetic (phase coherent — exact by definition)
    const uint64_t pps_gnss_ns = campaign_seconds * NS_PER_SECOND;

    // DWT: delta from spin loop shadow (or ISR fallback)
    uint32_t dwt_raw_at_pps;
    if (dispatch_shadow_valid) {
      dwt_raw_at_pps = dispatch_shadow_dwt;
    } else {
      dwt_raw_at_pps = isr_snap_dwt;
    }
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

    dispatch_shadow_valid = false;

    // ── Dispatch delta diagnostic ──
    int64_t dispatch_delta_ns = -1;
    if (isr_captured_shadow_dwt != 0) {
      uint32_t delta_cycles = isr_snap_dwt - isr_captured_shadow_dwt;
      dispatch_delta_ns = (int64_t)((uint64_t)delta_cycles * DWT_NS_NUM / DWT_NS_DEN);
    }

    // ── Residual updates ──
    residual_update(residual_dwt,  pps_dwt_cycles,  DWT_EXPECTED_PER_PPS);
    residual_update(residual_gnss, pps_gnss_ticks,  GNSS_EXPECTED_PER_PPS);
    residual_update(residual_ocxo, pps_ocxo_ticks,  OCXO_EXPECTED_PER_PPS);

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

    p.add("dwt_pps_residual",  residual_dwt.residual);
    p.add("gnss_pps_residual", residual_gnss.residual);
    p.add("ocxo_pps_residual", residual_ocxo.residual);

    p.add("isr_residual_dwt",   isr_residual_dwt);
    p.add("isr_residual_gnss",  isr_residual_gnss);
    p.add("isr_residual_ocxo",  isr_residual_ocxo);
    p.add("isr_residual_valid", isr_residual_valid);

    p.add("dispatch_delta_ns",   dispatch_delta_ns);
    p.add("pre_pps_approach_ns", pre_pps_approach_ns);
    p.add("dispatch_timeout",    (bool)dispatch_timeout);
    p.add("diag_fine_was_late",  diag_fine_was_late);

    p.add("ocxo_dac",          ocxo_dac_fractional);
    p.add("calibrate_ocxo",    calibrate_ocxo_active);
    p.add("servo_adjustments", servo_adjustments);

    p.add("diag_coarse_fires",  diag_coarse_fire_count);
    p.add("diag_fine_fires",    diag_fine_fire_count);
    p.add("diag_late",          diag_fine_late_count);
    p.add("diag_spins",         diag_spin_count);
    p.add("diag_timeouts",      diag_timeout_count);
    p.add("diag_spin_iters",    diag_spin_iterations);

    p.add("diag_raw_isr_cyc",    (uint32_t)isr_snap_dwt);
    p.add("diag_raw_shadow_cyc", (uint32_t)dwt_raw_at_pps);

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

  isr_captured_shadow_dwt = dispatch_shadow_dwt;
  pps_fired               = true;

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

  p.add("isr_residual_dwt",   isr_residual_dwt);
  p.add("isr_residual_gnss",  isr_residual_gnss);
  p.add("isr_residual_ocxo",  isr_residual_ocxo);
  p.add("isr_residual_valid", (bool)isr_residual_valid);

  p.add("dispatch_delta_ns",   (int64_t)-1);  // last value not retained across seconds
  p.add("pre_pps_approach_ns", pre_pps_approach_ns);
  p.add("dispatch_timeout",    (bool)dispatch_timeout);

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

  p.add("diag_coarse_fires",  diag_coarse_fire_count);
  p.add("diag_fine_fires",    diag_fine_fire_count);
  p.add("diag_late",          diag_fine_late_count);
  p.add("diag_spins",         diag_spin_count);
  p.add("diag_timeouts",      diag_timeout_count);
  p.add("diag_fine_was_late", diag_fine_was_late);
  p.add("diag_spin_iters",    diag_spin_iterations);

  p.add("pps_rejected_total",     diag_pps_rejected_total);
  p.add("pps_rejected_remainder", diag_pps_rejected_remainder);

  return p;
}

// ============================================================================
// INTERP_TEST — Interpolation accuracy verification
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

  uint64_t first_frag_gnss_ns    = 0;
  uint64_t first_frag_dwt_cycles = 0;
  uint32_t first_frag_pps_count  = 0;
  uint32_t first_dwt_now         = 0;
  uint64_t first_interp_gnss_ns  = 0;
  uint32_t first_gpt2_snap       = 0;
  uint64_t first_vclock_ns       = 0;
  int64_t  first_error_ns        = 0;

  for (int32_t i = 0; i < n; i++) {

    const timebase_fragment_t* frag = timebase_last_fragment();
    if (!frag || !frag->valid) {
      p.add("error", "timebase not valid");
      return p;
    }

    const uint64_t frag_gnss_ns    = frag->gnss_ns;
    const uint64_t frag_dwt_cycles = frag->dwt_cycles;
    const uint32_t frag_pps_count  = frag->pps_count;

    const uint32_t dwt_now_32  = DWT_CYCCNT;
    const uint32_t gpt2_snap   = GPT2_CNT;

    const int64_t interp_gnss_ns_signed = timebase_now_gnss_ns();
    if (interp_gnss_ns_signed < 0) {
      p.add("error", "timebase_now_gnss_ns failed");
      return p;
    }
    const uint64_t interp_gnss_ns = (uint64_t)interp_gnss_ns_signed;

    const uint32_t gnss_ticks_since_pps = gpt2_snap - isr_snap_gnss;
    const uint64_t gnss_ns_into_second  = (uint64_t)gnss_ticks_since_pps * 100ULL;
    const uint64_t vclock_gnss_ns       = frag_gnss_ns + gnss_ns_into_second;

    const int64_t error_ns = (int64_t)interp_gnss_ns - (int64_t)vclock_gnss_ns;

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
      first_frag_gnss_ns    = frag_gnss_ns;
      first_frag_dwt_cycles = frag_dwt_cycles;
      first_frag_pps_count  = frag_pps_count;
      first_dwt_now         = dwt_now_32;
      first_gpt2_snap       = gpt2_snap;
      first_interp_gnss_ns  = interp_gnss_ns;
      first_vclock_ns       = vclock_gnss_ns;
      first_error_ns        = error_ns;
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

  p.add("s0_frag_gnss_ns",       first_frag_gnss_ns);
  p.add("s0_frag_dwt_cycles",    first_frag_dwt_cycles);
  p.add("s0_frag_pps_count",     first_frag_pps_count);
  p.add("s0_dwt_cyccnt",         first_dwt_now);
  p.add("s0_gpt2_cnt",           first_gpt2_snap);
  p.add("s0_interp_gnss_ns",     first_interp_gnss_ns);
  p.add("s0_vclock_gnss_ns",     first_vclock_ns);
  p.add("s0_error_ns",           first_error_ns);

  p.add("isr_snap_gnss",         (uint32_t)isr_snap_gnss);
  p.add("campaign_seconds",      campaign_seconds);
  p.add("pps_rejected_total",    diag_pps_rejected_total);

  return p;
}

// ============================================================================
// Process registration
// ============================================================================

static const process_command_entry_t CLOCKS_COMMANDS[] = {
  { "START",       cmd_start       },
  { "STOP",        cmd_stop        },
  { "RECOVER",     cmd_recover     },
  { "REPORT",      cmd_report      },
  { "CLOCKS_INFO", cmd_clocks_info },
  { "INTERP_TEST", cmd_interp_test },
  { nullptr,       nullptr         }
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

  // TIMEPULSE retired. Keep relay pin quiescent.
  pinMode(GNSS_10KHZ_RELAY, OUTPUT);
  digitalWriteFast(GNSS_10KHZ_RELAY, LOW);
}