// ============================================================================
// process_clocks.cpp — ZPNet CLOCKS (Teensy) — v9 ISR-Synchronized Anchors
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
// Pre-PPS spin loop — REMOVED (v9)
// ============================================================================
//
// The spin loop has been eliminated.  Both clock anchors (DWT and GPT2)
// are now captured in the PPS ISR at entry.  They are ~50 ns after the
// true PPS edge but synchronized with each other (~2 ns apart).
//
// The ISR entry latency is deterministic and stable (~50 ns as proven
// by months of tdc_analyzer data).  Since both anchors share the same
// latency, the skew between them is effectively zero.
//
// This eliminates: the fine timer, the spin loop, BASEPRI masking,
// dispatch_shadow_dwt, isr_captured_shadow_dwt, dispatch_shadow_valid,
// dispatch_timeout, and the associated complexity.

static volatile bool     pps_fired = false;

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
// DWT: uses ISR-latched pre-edge shadow (isr_captured_shadow_dwt)
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
  prev_dwt_at_pps  = isr_snap_dwt;

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

    // Seed the accumulators from recovered nanosecond values
    dwt_cycles_64    = dwt_ns_to_cycles(recover_dwt_ns);
    prev_dwt_at_pps  = isr_snap_dwt;

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
    //
    // Each accumulator advances by the exact raw 32-bit delta since
    // the previous PPS edge.  No advance-then-rewind.  No intermediate
    // reads.  The accumulator is touched ONLY here, ONCE per second.
    //
    // v9: DWT uses isr_snap_dwt directly (ISR capture, ~50 ns after edge).
    // Both DWT and GPT2 anchors come from the same ISR entry, so they
    // are synchronized with each other.  The ~50 ns ISR entry latency
    // is shared and cancels in any DWT-vs-GPT2 comparison.

    // GNSS: arithmetic (phase coherent — exact by definition)
    const uint64_t pps_gnss_ns = campaign_seconds * NS_PER_SECOND;

    // DWT: ISR capture, corrected to PPS edge.
    // isr_snap_dwt is captured ~52 cycles after the PPS edge.
    // Subtract ISR_ENTRY_DWT_CYCLES to recover the DWT count at
    // the true PPS moment — the same instant gnss_ns represents.
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
    p.add("gpt2_at_pps",      (uint32_t)isr_snap_gnss);

    // DWT cycles in the second that just completed — the raw hardware
    // delta, not an average.  This is the best available estimate of
    // the crystal's current frequency for sub-second interpolation.
    // Jitter is ~7 ns (the DWT measurement floor).
    // On pps_count == 0 there is no prior second, so use nominal.
    p.add("dwt_cycles_per_pps", (campaign_seconds > 0) ? (uint64_t)dwt_delta : (uint64_t)DWT_EXPECTED_PER_PPS);

    p.add("dwt_pps_residual",  residual_dwt.residual);
    p.add("gnss_pps_residual", residual_gnss.residual);
    p.add("ocxo_pps_residual", residual_ocxo.residual);

    p.add("isr_residual_dwt",   isr_residual_dwt);
    p.add("isr_residual_gnss",  isr_residual_gnss);
    p.add("isr_residual_ocxo",  isr_residual_ocxo);
    p.add("isr_residual_valid", isr_residual_valid);

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
//
// Captures DWT_CYCCNT and GPT2_CNT back-to-back (~2 ns apart), then:
//
//   1. Feeds the DWT value to timebase_gnss_ns_from_dwt() — the
//      deterministic overload with no live reads, no torn-read loop.
//
//   2. Computes VCLOCK truth from the GPT2 value:
//      truth = frag.gnss_ns + (gpt2_now - frag.gpt2_at_pps) × 100
//
//   3. Error = interpolated - truth.
//
// The two register reads are separated by ~2 instructions = ~2 ns.
// This eliminates the -86 ns systematic bias from the previous version
// where timebase_now_gnss_ns() read DWT internally, separated from
// the GPT2 read by the function's execution time.
//
// The error now reflects purely the interpolation math accuracy —
// the mismatch between the prior second's DWT rate and the current
// second's true rate.

static Payload cmd_interp_test(const Payload& args) {
  Payload p;

  if (campaign_state != clocks_campaign_state_t::STARTED) {
    p.add("error", "no active campaign");
    return p;
  }

  const int32_t count = args.getInt("count", 10);
  const int32_t n = (count < 1) ? 1 : (count > 100) ? 100 : count;

  // Welford accumulators for error
  int64_t  w_n     = 0;
  double   w_mean  = 0.0;
  double   w_m2    = 0.0;
  int64_t  err_min = INT64_MAX;
  int64_t  err_max = INT64_MIN;
  int32_t  outside_100ns = 0;

  // First sample forensics
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

    // ── Read fragment ──
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

    // ── Simultaneous capture: DWT and GPT2 back-to-back (~2 ns gap) ──
    const uint32_t dwt_now  = ARM_DWT_CYCCNT;
    const uint32_t gpt2_now = GPT2_CNT;

    // ── Interpolated GNSS ns via deterministic overload ──
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

    // ── VCLOCK truth from GPT2 ──
    const uint32_t gpt2_since_pps = gpt2_now - frag_gpt2_at_pps;
    const uint64_t vclock_ns_into_second = (uint64_t)gpt2_since_pps * 100ULL;
    const uint64_t vclock_gnss_ns = frag_gnss_ns + vclock_ns_into_second;

    // ── Error ──
    const int64_t error_ns = (int64_t)interp_gnss_ns - (int64_t)vclock_gnss_ns;

    // ── DWT interim values ──
    const uint32_t dwt_elapsed = dwt_now - frag_dwt_cyccnt_at_pps;
    const uint64_t dwt_ns_into_second =
      (uint64_t)dwt_elapsed * 1000000000ULL / (uint64_t)frag_dwt_cycles_per_pps;

    // ── Welford ──
    w_n++;
    const double x  = (double)error_ns;
    const double d1 = x - w_mean;
    w_mean += d1 / (double)w_n;
    const double d2 = x - w_mean;
    w_m2 += d1 * d2;

    if (error_ns < err_min) err_min = error_ns;
    if (error_ns > err_max) err_max = error_ns;
    if (error_ns > 100 || error_ns < -100) outside_100ns++;

    // ── Capture first sample detail ──
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

  // ── Build response ──

  const double w_stddev = (w_n >= 2) ? sqrt(w_m2 / (double)(w_n - 1)) : 0.0;
  const double w_stderr = (w_n >= 2) ? w_stddev / sqrt((double)w_n)    : 0.0;

  // Stats
  p.add("samples",         (int32_t)w_n);
  p.add("error_mean_ns",   w_mean);
  p.add("error_stddev_ns", w_stddev);
  p.add("error_stderr_ns", w_stderr);
  p.add("error_min_ns",    err_min);
  p.add("error_max_ns",    err_max);
  p.add("outside_100ns",   outside_100ns);

  // Fragment state
  p.add("s0_frag_gnss_ns",            s0_frag_gnss_ns);
  p.add("s0_frag_dwt_cyccnt_at_pps",  s0_frag_dwt_cyccnt_at_pps);
  p.add("s0_frag_dwt_cycles_per_pps", s0_frag_dwt_cycles_per_pps);
  p.add("s0_frag_gpt2_at_pps",        s0_frag_gpt2_at_pps);
  p.add("s0_pps_count",               s0_pps_count);

  // Raw hardware reads
  p.add("s0_dwt_cyccnt_now",          s0_dwt_cyccnt_now);
  p.add("s0_gpt2_now",                s0_gpt2_now);

  // DWT interpolation chain
  p.add("s0_dwt_elapsed",             s0_dwt_elapsed);
  p.add("s0_dwt_ns_into_second",      s0_dwt_ns_into_second);
  p.add("s0_interp_gnss_ns",          s0_interp_gnss_ns);

  // VCLOCK truth chain
  p.add("s0_gpt2_since_pps",          s0_gpt2_since_pps);
  p.add("s0_vclock_ns_into_second",   s0_vclock_ns_into_second);
  p.add("s0_vclock_gnss_ns",          s0_vclock_gnss_ns);

  // Error + position
  p.add("s0_error_ns",                s0_error_ns);
  p.add("s0_position",                s0_position);

  // Context
  p.add("campaign_seconds",           campaign_seconds);
  p.add("pps_rejected_total",         diag_pps_rejected_total);

  return p;
}

// ============================================================================
// INTERP_PROOF — Long-running interpolation accuracy proof
// ============================================================================
//
// Takes ONE sample per invocation and accumulates Welford stats across
// calls.  Designed to be called once per second (or at any interval)
// from the Pi side.  The position within the second varies naturally
// with command arrival time, providing coverage across the full second.
//
// Commands:
//   INTERP_PROOF               — take one sample, return running stats
//   INTERP_PROOF reset=1       — clear accumulators, then take one sample
//
// If the math is correct:
//   - mean converges to zero (or a small physical constant)
//   - stddev converges to the jitter floor (~30 ns)
//   - stderr shrinks as 1/sqrt(N)
//   - no correlation between error and position in second
//
// If there's a systematic bug:
//   - mean converges to a nonzero value
//   - stderr gets small enough to prove the offset is real

static int64_t  proof_n       = 0;
static double   proof_mean    = 0.0;
static double   proof_m2      = 0.0;
static int64_t  proof_err_min = 0;
static int64_t  proof_err_max = 0;
static int32_t  proof_outside_100 = 0;

// Position tracking — to detect correlation with position
static double   proof_pos_mean = 0.0;   // mean position in second (0..1)
static double   proof_pos_m2   = 0.0;
static double   proof_covar    = 0.0;   // covariance(error, position)

static Payload cmd_interp_proof(const Payload& args) {
  Payload p;

  if (campaign_state != clocks_campaign_state_t::STARTED) {
    p.add("error", "no active campaign");
    return p;
  }

  // Reset if requested
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

  // ── Take one sample ──

  // Interpolated GNSS ns (the code under test)
  const int64_t interp_signed = timebase_now_gnss_ns();
  if (interp_signed < 0) {
    p.add("error", "timebase_now_gnss_ns failed");
    return p;
  }
  const uint64_t interp_gnss_ns = (uint64_t)interp_signed;

  // VCLOCK truth — captured immediately after
  const uint32_t gpt2_now = GPT2_CNT;

  // Fragment for base
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

  // Position in second (0.0 to 1.0)
  const double position = (double)vclock_ns_into_second / 1000000000.0;

  // ── Welford update for error ──
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

  // ── Welford update for position + covariance ──
  const double pd1 = position - proof_pos_mean;
  proof_pos_mean += pd1 / (double)proof_n;
  const double pd2 = position - proof_pos_mean;
  proof_pos_m2 += pd1 * pd2;

  // Online covariance: Co(n) += (x - mean_x_new) * (pos - mean_pos_old)
  proof_covar += d2 * pd1;

  // ── Compute stats ──
  const double stddev = (proof_n >= 2) ? sqrt(proof_m2 / (double)(proof_n - 1)) : 0.0;
  const double stderr_val = (proof_n >= 2) ? stddev / sqrt((double)proof_n) : 0.0;
  const double pos_stddev = (proof_n >= 2) ? sqrt(proof_pos_m2 / (double)(proof_n - 1)) : 0.0;

  // Correlation coefficient: r = covar / (stddev_error * stddev_position)
  const double correlation = (proof_n >= 2 && stddev > 0.0 && pos_stddev > 0.0)
    ? (proof_covar / (double)(proof_n - 1)) / (stddev * pos_stddev)
    : 0.0;

  // ── Build response ──
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

  // TIMEPULSE retired. Keep relay pin quiescent.
  pinMode(GNSS_10KHZ_RELAY, OUTPUT);
  digitalWriteFast(GNSS_10KHZ_RELAY, LOW);
}