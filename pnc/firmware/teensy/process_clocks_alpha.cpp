// ============================================================================
// process_clocks_alpha.cpp — Always-On Physics Layer
// ============================================================================
//
// v14: Symmetric GPT architecture for both OCXOs.
//
//   OCXO1 on GPT1  (pin 25, GPIO_AD_B0_13, ALT1, 10 MHz single-edge).
//   OCXO2 on GPT2  (pin 14, GPIO_AD_B1_02, ALT8, 10 MHz single-edge).
//   GNSS  on QTimer1 ch0+ch1 (pin 10, GPIO_B0_00, ALT1, 20 MHz dual-edge).
//
//   Both OCXOs have clean, deterministic single-edge counting.
//   GNSS VCLOCK on QTimer1 serves as a PPS sanity check only.
//   The dual-edge alternation on GNSS is harmless — GNSS time
//   authority derives from PPS count, not VCLOCK tick precision.
//
//   GPT2 no longer runs on internal 24 MHz — it now counts OCXO2
//   externally.  TimePop scheduling uses DWT-based deadline
//   conversion (unchanged from v13).
//
// ============================================================================

#include "process_clocks_internal.h"
#include "process_clocks.h"
#include "process_timepop.h"

#include "debug.h"
#include "timebase.h"
#include "time.h"

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
// TimePop delay constants (nanoseconds)
// ============================================================================

static constexpr uint64_t OCXO_DITHER_NS   =     1000000ULL;  //   1 ms
static constexpr uint64_t PPS_RELAY_OFF_NS  =   500000000ULL;  // 500 ms

// ============================================================================
// Continuous DWT-to-GNSS Calibration (campaign-independent)
// ============================================================================

volatile uint32_t  g_dwt_at_last_pps      = 0;
volatile uint32_t  g_dwt_cycles_per_gnss_s = 0;
volatile bool      g_dwt_cal_has_prev      = false;
volatile bool      g_dwt_cal_valid         = false;
volatile uint64_t  g_dwt_cal_pps_count     = 0;

uint32_t clocks_dwt_cycles_per_gnss_second(void) {
  return g_dwt_cal_valid ? g_dwt_cycles_per_gnss_s : 0;
}

bool clocks_dwt_calibration_valid(void) {
  return g_dwt_cal_valid;
}

// ============================================================================
// OCXO DAC state — definitions
// ============================================================================

ocxo_dac_state_t ocxo1_dac = {
  (double)OCXO1_DAC_DEFAULT, 0, 0, 0.0, 0, 0
};

ocxo_dac_state_t ocxo2_dac = {
  (double)OCXO2_DAC_DEFAULT, 0, 0, 0.0, 0, 0
};

bool calibrate_ocxo_active = false;

void ocxo_dac_set(ocxo_dac_state_t& s, double value) {
  if (value < (double)OCXO1_DAC_MIN) value = (double)OCXO1_DAC_MIN;
  if (value > (double)OCXO1_DAC_MAX) value = (double)OCXO1_DAC_MAX;
  s.dac_fractional = value;
}

// ============================================================================
// PPS relay to Pi
// ============================================================================

volatile bool relay_arm_pending  = false;
volatile bool relay_timer_active = false;

static void pps_relay_deassert(timepop_ctx_t*, void*) {
  digitalWriteFast(GNSS_PPS_RELAY, LOW);
  relay_timer_active = false;
}

// ============================================================================
// ISR-level hardware snapshots and residuals
// ============================================================================

volatile uint32_t isr_snap_dwt   = 0;
volatile uint32_t isr_snap_gnss  = 0;
volatile uint32_t isr_snap_ocxo1 = 0;
volatile uint32_t isr_snap_ocxo2 = 0;

volatile uint32_t isr_prev_dwt   = 0;
volatile uint32_t isr_prev_gnss  = 0;
volatile uint32_t isr_prev_ocxo1 = 0;
volatile uint32_t isr_prev_ocxo2 = 0;

volatile int32_t  isr_residual_dwt   = 0;
volatile int32_t  isr_residual_gnss  = 0;
volatile int32_t  isr_residual_ocxo1 = 0;
volatile int32_t  isr_residual_ocxo2 = 0;
volatile bool     isr_residual_valid = false;

// ============================================================================
// PPS diagnostics
// ============================================================================

volatile uint32_t diag_pps_rejected_total     = 0;
volatile uint32_t diag_pps_rejected_remainder = 0;

// ============================================================================
// PPS state
// ============================================================================

volatile bool pps_fired = false;
volatile bool pps_scheduled = false;

// ============================================================================
// Rolling 64-bit extensions
// ============================================================================

uint64_t dwt_rolling_64  = 0;
uint32_t dwt_rolling_32  = 0;

// ============================================================================
// DWT (CPU cycle counter — 1008 MHz internal)
// ============================================================================

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
// GNSS VCLOCK (10 MHz external via QTimer1 ch0+ch1 — cascaded 32-bit)
//
// Pin 10 = GPIO_B0_00, IOMUXC ALT1 = QTIMER1_TIMER0
//
// QTimer CM(2) counts both rising and falling edges, producing
// ~20 MHz raw counts for a 10 MHz input.  The dual-edge alternation
// is harmless for GNSS — time authority comes from PPS count.
// VCLOCK serves only as a PPS sanity check.
//
// ch0+ch1 cascade for 32-bit range.
//
// IMPORTANT: Do NOT touch channels 2 and 3 — pin 11 (OCXO2 CTL) uses
// FlexPWM1 submodule 2, which shares the QTimer1 module.
// ============================================================================

static bool qtimer1_armed = false;

uint64_t gnss_rolling_raw_64 = 0;
uint32_t gnss_rolling_32     = 0;

uint32_t qtimer1_read_32(void) {
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

  gnss_rolling_32 = qtimer1_read_32();
  qtimer1_armed   = true;
}

uint64_t clocks_gnss_ticks_now(void) {
  uint32_t now = qtimer1_read_32();
  gnss_rolling_raw_64 += (uint32_t)(now - gnss_rolling_32);
  gnss_rolling_32 = now;
  return gnss_rolling_raw_64 / GNSS_EDGE_DIVISOR;
}

uint64_t clocks_gnss_ns_now(void) {
  if (campaign_state != clocks_campaign_state_t::STARTED) return 0;
  return clocks_gnss_ticks_now() * 100ull;
}

// ============================================================================
// OCXO1 (10 MHz external via GPT1 — pin 25)
//
// GPT1 counts single edges at 10 MHz.  32-bit native, no cascade.
// Pin 25 = GPIO_AD_B0_13, IOMUX ALT1 = GPT1_CLK.
// ============================================================================

static bool gpt1_armed = false;

static inline void enable_gpt1(void) {
  CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON);
}

uint64_t ocxo1_rolling_64 = 0;
uint32_t ocxo1_rolling_32 = 0;

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
// OCXO2 (10 MHz external via GPT2 — pin 14)
//
// GPT2 counts single edges at 10 MHz.  32-bit native, no cascade.
// Pin 14 = GPIO_AD_B1_02, IOMUX ALT8 = GPT2_CLK.
//
// This is the same pin and IOMUX configuration that previously
// counted GNSS VCLOCK.  The only change is what signal is wired
// to the pin.
// ============================================================================

static bool gpt2_armed = false;

static inline void enable_gpt2(void) {
  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON);
}

uint64_t ocxo2_rolling_64 = 0;
uint32_t ocxo2_rolling_32 = 0;

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
  ocxo2_rolling_32 = GPT2_CNT;
  gpt2_armed       = true;
}

uint64_t clocks_ocxo2_ticks_now(void) {
  uint32_t now = GPT2_CNT;
  ocxo2_rolling_64 += (uint32_t)(now - ocxo2_rolling_32);
  ocxo2_rolling_32 = now;
  return ocxo2_rolling_64;
}

uint64_t clocks_ocxo2_ns_now(void) {
  if (campaign_state != clocks_campaign_state_t::STARTED) return 0;
  return clocks_ocxo2_ticks_now() * 100ull;
}

// ============================================================================
// PPS handling — deferred ASAP callback (scheduled context)
// ============================================================================

static void pps_asap_callback(timepop_ctx_t*, void*) {

  // ── Continuous DWT calibration (always runs, campaign-independent) ──
  {
    const uint32_t dwt_corrected = isr_snap_dwt - ISR_ENTRY_DWT_CYCLES;

    if (g_dwt_cal_has_prev) {
      g_dwt_cycles_per_gnss_s = dwt_corrected - g_dwt_at_last_pps;
      g_dwt_cal_valid = true;
    }

    g_dwt_at_last_pps  = dwt_corrected;
    g_dwt_cal_has_prev = true;
    g_dwt_cal_pps_count++;
  }

  // ── Universal GNSS time anchor (campaign-independent) ──
  time_pps_update(
    isr_snap_dwt - ISR_ENTRY_DWT_CYCLES,
    g_dwt_cal_valid ? g_dwt_cycles_per_gnss_s : 0
  );

  if (relay_arm_pending) {
    relay_arm_pending = false;
    if (!relay_timer_active) {
      relay_timer_active = true;
      timepop_arm(PPS_RELAY_OFF_NS, false, pps_relay_deassert, nullptr, "pps-relay-off");
    }
  }

  // ── Campaign-scoped processing (beta) ──
  clocks_beta_pps();

  pps_scheduled = false;
}

// ============================================================================
// PPS handling — ISR (minimum latency)
//
// Read order: DWT first (highest frequency, most latency-sensitive),
// then the two GPTs (OCXO1, OCXO2), then QTimer1 (GNSS).
// ============================================================================

static void pps_isr(void) {

  const uint32_t snap_dwt   = DWT_CYCCNT;
  const uint32_t snap_ocxo1 = GPT1_CNT;
  const uint32_t snap_ocxo2 = GPT2_CNT;
  const uint32_t snap_gnss  = qtimer1_read_32();

  // PPS validation uses GNSS raw count (20 MHz dual-edge).
  // The alternation means isr_residual_gnss will oscillate,
  // but PPS_VCLOCK_TOLERANCE accommodates this.
  if (isr_residual_valid) {
    uint32_t elapsed = snap_gnss - isr_prev_gnss;
    uint32_t remainder = elapsed % ISR_GNSS_RAW_EXPECTED;
    if (remainder > PPS_VCLOCK_TOLERANCE &&
        remainder < (ISR_GNSS_RAW_EXPECTED - PPS_VCLOCK_TOLERANCE)) {
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
    isr_residual_gnss  = (int32_t)(snap_gnss  - isr_prev_gnss)  - (int32_t)ISR_GNSS_RAW_EXPECTED;
    isr_residual_ocxo1 = (int32_t)(snap_ocxo1 - isr_prev_ocxo1) - (int32_t)ISR_OCXO1_EXPECTED;
    isr_residual_ocxo2 = (int32_t)(snap_ocxo2 - isr_prev_ocxo2) - (int32_t)ISR_OCXO2_EXPECTED;
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
// Initialization — Phase 1 (hardware only, no TimePop dependency)
// ============================================================================

void process_clocks_init_hardware(void) {
  dwt_enable();
  arm_gpt1_external();      // OCXO1 10 MHz (GPT1, pin 25)
  arm_gpt2_external();      // OCXO2 10 MHz (GPT2, pin 14)
  arm_qtimer1_external();   // GNSS VCLOCK (QTimer1 ch0, pin 10)
}

// ============================================================================
// Initialization — Phase 2 (full lifecycle, requires TimePop)
// ============================================================================

void process_clocks_init(void) {

  time_init();
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