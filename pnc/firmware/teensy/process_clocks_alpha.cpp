// ============================================================================
// process_clocks_alpha.cpp — Always-On Physics Layer
// ============================================================================
//
// v29: OCXO phase capture — shadow-write TDC architecture.
//
//   Replaces the fixed ISR-latency-subtraction model (v28) with the
//   same shadow-write TDC technique used for PPS edge capture.
//
//   The PPS ISR arms GPT1/GPT2 output compares with a generous offset
//   (OCXO_PHASE_ARM_OFFSET_TICKS) so the compare fires AFTER the ASAP
//   callback has started its shadow-write loop.  The ASAP callback
//   spins a tight loop writing DWT_CYCCNT to a shared volatile until
//   both GPT ISRs have fired (or timeout).
//
//   When each GPT ISR fires (priority 0 — no temporal conflict with
//   PPS because the compare target is well after PPS), it captures
//   both DWT_CYCCNT and the current shadow value.  The delta
//   (isr_dwt - shadow_dwt) provides per-sample ISR latency
//   characterization, enabling cycle-accurate TDC correction.
//
//   The arming MUST happen in pps_isr (not the ASAP callback) because
//   the OCXO counters advance continuously — by the time the ASAP
//   callback runs, snap_ocxo + 50 may already be in the past.
//   The larger offset (150 ticks = 15 µs) ensures the compare fires
//   after the ASAP callback has started the shadow loop (~2-3 µs
//   after PPS).
//
// v26: OCXO phase-first capture.
// v24: OCXO phase capture — DWT-interpolated edge timing.
// v23: 10 MHz single-edge QTimer1 migration.
// v22: WATCHDOG_ANOMALY stop-and-yield architecture.
// v21: QTimer1 torn-read prevention.
// v19: Shadow-loop timeout protection + consistent timeout naming.
// v18: Spin Capture with shadow-write loop — TDC at 1008 MHz.
// v16: PPS watchdog — self-healing pps_scheduled stall.
// v15: DAC dither telemetry + random walk prediction model.
// v14: Symmetric GPT architecture for both OCXOs.
//
// ============================================================================

#include "tdc_correction.h"
#include "isr_dwt_compensate.h"

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
#include "ad5693r.h"

#include <Arduino.h>
#include "imxrt.h"

#include <math.h>
#include <stdlib.h>

// ============================================================================
// TimePop delay constants (nanoseconds)
// ============================================================================

static constexpr int64_t SPIN_EARLY_NS = 10000LL;  // 5 µs — shadow loop needs margin

// ============================================================================
// TimePop delay constants (nanoseconds)
// ============================================================================

static constexpr uint64_t PPS_RELAY_OFF_NS  =   500000000ULL;  // 500 ms

// ============================================================================
// PPS watchdog — anomaly thresholds
// ============================================================================

static constexpr uint32_t PPS_WATCHDOG_THRESHOLD = 3;

// ============================================================================
// PPS rejection watchdog — consecutive rejection anomaly threshold
// ============================================================================

static constexpr uint32_t PPS_REJECT_RECOVERY_THRESHOLD = 10;

// ============================================================================
// Spin capture timeout — shadow-write loop safety net
// ============================================================================

static constexpr uint32_t SPIN_LOOP_TIMEOUT_CYCLES = 100800;  // 100 µs at 1008 MHz

// ============================================================================
// OCXO phase capture — shadow-write spin timeout
//
// The OCXO shadow loop runs in the ASAP callback waiting for GPT
// ISRs that were armed by pps_isr.  The GPT compares fire at
// snap_ocxo + OCXO_PHASE_ARM_OFFSET_TICKS (~15 µs after PPS).
// The ASAP callback starts ~2-3 µs after PPS, so the shadow loop
// runs for ~12-13 µs before the first GPT fires.  50 µs timeout
// provides generous margin.
// ============================================================================

static constexpr uint32_t OCXO_PHASE_SPIN_TIMEOUT_CYCLES = 50400;  // 50 µs at 1008 MHz

// ============================================================================
// Spin Capture — nano-precise DWT anchoring with shadow-write TDC
// ============================================================================

volatile uint32_t dispatch_shadow_dwt     = 0;
volatile uint32_t isr_captured_shadow_dwt = 0;

volatile uint32_t dbg_post_loop_dwt       = 0;
volatile uint32_t dbg_post_loop_shadow    = 0;
volatile uint32_t dbg_post_loop_isr_cap   = 0;
volatile uint32_t dbg_post_loop_isr_snap  = 0;

spin_capture_t pps_spin = {};

static void pps_spin_callback(timepop_ctx_t* ctx, void*) {
  pps_spin.landed_dwt     = ctx->fire_dwt_cyccnt;
  pps_spin.landed_gnss_ns = ctx->fire_gnss_ns;
  pps_spin.spin_error     = (int32_t)(ctx->fire_dwt_cyccnt - pps_spin.target_dwt);

  if (ctx->nano_timeout) {
    pps_spin.nano_timed_out = true;
    pps_spin.nano_timeouts++;
    pps_spin.completed = true;
    return;
  }

  pps_spin.nano_timed_out = false;

  const uint32_t spin_start = DWT_CYCCNT;
  for (;;) {
    dispatch_shadow_dwt = DWT_CYCCNT;
    if (pps_fired) break;
    if ((DWT_CYCCNT - spin_start) > SPIN_LOOP_TIMEOUT_CYCLES) {
      pps_spin.shadow_timed_out = true;
      pps_spin.shadow_timeouts++;
      pps_spin.completed = true;
      return;
    }
  }

  pps_spin.shadow_dwt       = isr_captured_shadow_dwt;
  pps_spin.shadow_timed_out = false;
  pps_spin.completed        = true;
  pps_spin.completions++;

  // ── DEBUG: shadow capture forensics ──
  dbg_post_loop_dwt     = DWT_CYCCNT;
  dbg_post_loop_shadow  = dispatch_shadow_dwt;
  dbg_post_loop_isr_cap = isr_captured_shadow_dwt;
  dbg_post_loop_isr_snap = isr_snap_dwt;
}

static void pps_spin_arm(void) {
  if (!g_dwt_cal_valid) return;
  if (!time_valid()) return;

  int64_t now_ns = time_gnss_ns_now();
  if (now_ns < 0) return;

  int64_t target_gnss_ns = now_ns + (int64_t)NS_PER_SECOND - SPIN_EARLY_NS;
  uint32_t target_dwt    = time_gnss_ns_to_dwt(target_gnss_ns);

  pps_spin.target_dwt       = target_dwt;
  pps_spin.target_gnss_ns   = target_gnss_ns;
  pps_spin.completed        = false;
  pps_spin.nano_timed_out   = false;
  pps_spin.shadow_timed_out = false;
  pps_spin.arms++;

  pps_fired = false;

  timepop_handle_t h = timepop_arm_ns(
      target_gnss_ns, target_dwt,
      pps_spin_callback, nullptr, "pps-spin",
      true  // ISR callback: shadow-write loop must run in ISR context
  );

  if (h == TIMEPOP_INVALID_HANDLE) {
    pps_spin.armed = false;
    pps_spin.arm_failures++;
  } else {
    pps_spin.armed = true;
  }
}

static void pps_spin_complete(uint32_t isr_dwt_snap) {
  if (!pps_spin.completed) {
    pps_spin.valid = false;
    if (pps_spin.armed) {
      pps_spin.misses++;
      if (pps_spin.handle != TIMEPOP_INVALID_HANDLE) timepop_cancel(pps_spin.handle);
    }
    pps_spin.handle = TIMEPOP_INVALID_HANDLE;
    return;
  }

  if (pps_spin.nano_timed_out || pps_spin.shadow_timed_out) {
    pps_spin.valid = false;
    pps_spin.armed = false;
    pps_spin.handle = TIMEPOP_INVALID_HANDLE;
    return;
  }

  pps_spin.isr_dwt         = isr_dwt_snap;
  pps_spin.delta_cycles     = (int32_t)(isr_dwt_snap - pps_spin.shadow_dwt);
  pps_spin.approach_cycles  = (int32_t)(isr_dwt_snap - pps_spin.landed_dwt);

  int32_t correction = -1;
  pps_spin.corrected_dwt = tdc_correct(
    pps_spin.shadow_dwt,
    pps_spin.delta_cycles,
    correction
  );
  pps_spin.tdc_correction = correction;

  pps_spin.valid  = (correction >= 0);
  pps_spin.armed  = false;
  pps_spin.handle = TIMEPOP_INVALID_HANDLE;
}

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

bool g_ad5693r_init_ok = false;

// ============================================================================
// OCXO DAC state — definitions
// ============================================================================

ocxo_dac_state_t ocxo1_dac = {
  (double)AD5693R_DAC_DEFAULT, AD5693R_DAC_DEFAULT, 0, 65535,
  0, 0.0, 0, 0
};

ocxo_dac_state_t ocxo2_dac = {
  (double)AD5693R_DAC_DEFAULT, AD5693R_DAC_DEFAULT, 0, 65535,
  0, 0.0, 0, 0
};

servo_mode_t calibrate_ocxo_mode = servo_mode_t::OFF;

const char* servo_mode_str(servo_mode_t mode) {
  switch (mode) {
  case servo_mode_t::MEAN:  return "MEAN";
  case servo_mode_t::TOTAL: return "TOTAL";
  case servo_mode_t::NOW:   return "NOW";
  default:                  return "OFF";
  }
}

servo_mode_t servo_mode_parse(const char* s) {
  if (!s || !*s) return servo_mode_t::OFF;
  if (strcmp(s, "MEAN")  == 0) return servo_mode_t::MEAN;
  if (strcmp(s, "TOTAL") == 0) return servo_mode_t::TOTAL;
  if (strcmp(s, "NOW")   == 0) return servo_mode_t::NOW;
  return servo_mode_t::OFF;
}

void ocxo_dac_set(ocxo_dac_state_t& s, double value) {
  if (value < (double)s.dac_min) value = (double)s.dac_min;
  if (value > (double)s.dac_max) value = (double)s.dac_max;
  s.dac_fractional = value;
  s.dac_hw_code = (uint16_t)value;

  if (&s == &ocxo1_dac) {
    ad5693r_write_input(AD5693R_ADDR_OCXO1, s.dac_hw_code);
    ad5693r_update_dac(AD5693R_ADDR_OCXO1);
  } else {
    ad5693r_write_input(AD5693R_ADDR_OCXO2, s.dac_hw_code);
    ad5693r_update_dac(AD5693R_ADDR_OCXO2);
  }
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

volatile uint32_t diag_pps_scheduled_stuck     = 0;
volatile uint32_t diag_pps_watchdog_recoveries = 0;
volatile uint32_t diag_pps_asap_arm_failures   = 0;
volatile uint32_t diag_pps_asap_armed          = 0;
volatile uint32_t diag_pps_asap_dispatched     = 0;
volatile uint32_t diag_pps_stuck_since_dwt     = 0;
volatile uint32_t diag_pps_stuck_max           = 0;

volatile uint32_t diag_pps_reject_consecutive  = 0;
volatile uint32_t diag_pps_reject_recoveries   = 0;
volatile uint32_t diag_pps_reject_max_run      = 0;

// ============================================================================
// PPS state
// ============================================================================

volatile bool pps_fired = false;
volatile bool pps_scheduled = false;

// ============================================================================
// OCXO phase capture — v29 shadow-write TDC architecture
//
// Each GPT fires a one-shot output-compare ISR on the first OCXO
// edge after PPS.  The ISR captures DWT_CYCCNT and the current shadow
// DWT value from the ASAP callback's shadow-write loop.  The delta
// (isr_dwt - shadow_dwt) provides per-sample ISR latency for TDC
// correction, identical to the PPS spin capture mechanism.
//
// GPT arming happens in pps_isr (while OCXO counters are close to
// their snapshot values).  The shadow-write loop runs in the ASAP
// callback.  The arm offset must be large enough that the compare
// fires AFTER the ASAP callback has started the shadow loop.
// ============================================================================

// ── Shared shadow variable for OCXO phase spin loop ──
volatile uint32_t ocxo_phase_shadow_dwt = 0;

// ── GPT ISR capture state ──
volatile uint32_t ocxo1_phase_isr_dwt      = 0;
volatile uint32_t ocxo1_phase_shadow_dwt   = 0;
volatile bool     ocxo1_phase_captured     = false;
volatile uint32_t ocxo1_phase_gpt_at_fire  = 0;

volatile uint32_t ocxo2_phase_isr_dwt      = 0;
volatile uint32_t ocxo2_phase_shadow_dwt   = 0;
volatile bool     ocxo2_phase_captured     = false;
volatile uint32_t ocxo2_phase_gpt_at_fire  = 0;

// ── Phase capture diagnostics ──
volatile uint32_t diag_ocxo1_phase_captures = 0;
volatile uint32_t diag_ocxo2_phase_captures = 0;
volatile uint32_t diag_ocxo1_phase_misses   = 0;
volatile uint32_t diag_ocxo2_phase_misses   = 0;

// ── Phase spin diagnostics ──
volatile uint32_t diag_ocxo_phase_spin_timeouts = 0;

// ── Simplified phase result (computed in pps_asap_callback) ──
ocxo_phase_capture_t ocxo_phase = {};

volatile uint32_t diag_gpt1_isr_fires = 0;
volatile uint32_t diag_gpt2_isr_fires = 0;

// ============================================================================
// GPT1 output-compare ISR — OCXO1 edge capture
//
// Fires once per PPS on the first OCXO1 edge at or after the armed
// compare value.  DWT_CYCCNT is the very first instruction.
// Priority 0 — no nesting overhead, no temporal conflict with PPS.
// ============================================================================

static void gpt1_phase_isr(void) {
  const uint32_t dwt_raw = DWT_CYCCNT;
  const uint32_t shadow  = ocxo_phase_shadow_dwt;
  const uint32_t gpt     = GPT1_CNT;

  GPT1_SR = GPT_SR_OF1;
  GPT1_IR &= ~GPT_IR_OF1IE;

  ocxo1_phase_isr_dwt    = dwt_raw;
  ocxo1_phase_shadow_dwt = shadow;
  ocxo1_phase_gpt_at_fire = gpt;
  ocxo1_phase_captured    = true;

  diag_gpt1_isr_fires++;
}

// ============================================================================
// GPT2 output-compare ISR — OCXO2 edge capture
// ============================================================================

static void gpt2_phase_isr(void) {
  const uint32_t dwt_raw = DWT_CYCCNT;
  const uint32_t shadow  = ocxo_phase_shadow_dwt;
  const uint32_t gpt     = GPT2_CNT;

  GPT2_SR = GPT_SR_OF1;
  GPT2_IR &= ~GPT_IR_OF1IE;

  ocxo2_phase_isr_dwt    = dwt_raw;
  ocxo2_phase_shadow_dwt = shadow;
  ocxo2_phase_gpt_at_fire = gpt;
  ocxo2_phase_captured    = true;

  diag_gpt2_isr_fires++;
}

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
// v23: Single-edge counting (CM=1, 10 MHz).
//
//   QTimer1 CH0 counts rising edges only of the GNSS 10 MHz signal.
//   CH1 cascades from CH0 overflow to form a 32-bit counter.
//   One tick = one GNSS cycle = 100 ns.
//
//   The raw counter value IS the 10 MHz tick count — no divisor needed.
//   32-bit wrap: 429.5 seconds at 10 MHz.
//
// v21: Torn-read prevention.
//
//   QTimer1 channels 0 and 1 form a cascaded 32-bit counter, but
//   reading two 16-bit registers is not atomic.  If the low counter
//   (ch0) rolls over between reading ch1 (high) and ch0 (low), the
//   combined value is off by up to 65,536 ticks.
//
//   The high-low-high pattern detects this: read the high word,
//   then the low word, then the high word again.  If the two high
//   reads differ, the low word rolled over — re-read it.  The
//   second low read is consistent with the new (second) high value.
//
//   Cost: one extra 16-bit register read in the common case (~1 ns).
//   In the rare rollover case, two extra reads (~2 ns).
//
//   This is called from the PPS ISR (priority 0) and from scheduled
//   context (clocks_gnss_ticks_now, INTERP_PROOF, etc.).  Both
//   contexts benefit from the protection.
// ============================================================================

static bool qtimer1_armed = false;

uint64_t gnss_rolling_raw_64 = 0;
uint32_t gnss_rolling_32     = 0;

// ============================================================================
// QTimer1 read diagnostics
// ============================================================================

volatile uint32_t diag_qread_total            = 0;
volatile uint32_t diag_qread_same_hi          = 0;
volatile uint32_t diag_qread_retry_hi_changed = 0;

volatile uint32_t diag_qread_last_hi1 = 0;
volatile uint32_t diag_qread_last_hi2 = 0;
volatile uint32_t diag_qread_last_lo  = 0;
volatile uint32_t diag_qread_last_lo2 = 0;

uint32_t qtimer1_read_32(void) {
  diag_qread_total++;

  const uint16_t lo1 = IMXRT_TMR1.CH[0].CNTR;   // latches CH1.HOLD
  const uint16_t hi1 = IMXRT_TMR1.CH[1].HOLD;

  const uint16_t lo2 = IMXRT_TMR1.CH[0].CNTR;   // relatches CH1.HOLD
  const uint16_t hi2 = IMXRT_TMR1.CH[1].HOLD;

  diag_qread_last_hi1 = hi1;
  diag_qread_last_hi2 = hi2;
  diag_qread_last_lo  = lo1;
  diag_qread_last_lo2 = lo2;

  if (hi1 != hi2) {
    diag_qread_retry_hi_changed++;
    return ((uint32_t)hi2 << 16) | (uint32_t)lo2;
  }

  diag_qread_same_hi++;
  return ((uint32_t)hi1 << 16) | (uint32_t)lo1;
}

static void arm_qtimer1_external(void) {
  if (qtimer1_armed) return;

  CCM_CCGR6 |= CCM_CCGR6_QTIMER1(CCM_CCGR_ON);

  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 1;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 =
      IOMUXC_PAD_HYS | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_DSE(4);

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

  // v23: CM(1) = count rising edges only = 10 MHz.
  // Previously CM(2) = dual-edge = 20 MHz.
  IMXRT_TMR1.CH[0].CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);
  IMXRT_TMR1.CH[1].CTRL = TMR_CTRL_CM(7);

  // CH0 and CH1 are passive counter channels only.  TimePop compare
  // uses CH2.  CH3 is unallocated.  Keep CH0 compare interrupts
  // disabled permanently so the 32-bit counter path remains sovereign.
  IMXRT_TMR1.CH[0].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  IMXRT_TMR1.CH[0].CSCTRL |=  TMR_CSCTRL_TCF1;

  // CH1 is cascade-only — silence its compare and overflow alarms permanently.
  IMXRT_TMR1.CH[1].CSCTRL = 0;
  IMXRT_TMR1.CH[1].SCTRL  &= ~(TMR_SCTRL_TOFIE | TMR_SCTRL_IEF | TMR_SCTRL_IEFIE);

  gnss_rolling_32 = qtimer1_read_32();
  qtimer1_armed   = true;
}

// v23: Raw counter IS 10 MHz ticks — no divisor needed.
uint64_t clocks_gnss_ticks_now(void) {
  uint32_t now = qtimer1_read_32();
  gnss_rolling_raw_64 += (uint32_t)(now - gnss_rolling_32);
  gnss_rolling_32 = now;
  return gnss_rolling_raw_64;
}

uint64_t clocks_gnss_ns_now(void) {
  if (campaign_state != clocks_campaign_state_t::STARTED) return 0;
  return clocks_gnss_ticks_now() * 100ull;
}

// ============================================================================
// OCXO1 (10 MHz external via GPT1 — pin 25)
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

  // ── Spin capture: complete the current capture, arm the next ──
  // Must run FIRST so pps_spin.corrected_dwt is available for
  // the calibration and time anchor blocks below.
  pps_spin_complete(isr_snap_dwt);
  pps_spin_arm();

  // ── Best-available DWT at the true PPS edge ──
  // TDC-corrected when spin capture succeeded; ISR-compensated fallback otherwise.
  const uint32_t dwt_at_pps_edge =
    (pps_spin.valid && pps_spin.tdc_correction >= 0)
      ? pps_spin.corrected_dwt
      : (isr_snap_dwt - TDC_FIXED_OVERHEAD);

  // ── Continuous DWT calibration (always runs, campaign-independent) ──
  {
    if (g_dwt_cal_has_prev) {
      g_dwt_cycles_per_gnss_s = dwt_at_pps_edge - g_dwt_at_last_pps;
      g_dwt_cal_valid = true;
    }

    g_dwt_at_last_pps  = dwt_at_pps_edge;
    g_dwt_cal_has_prev = true;
    g_dwt_cal_pps_count++;
  }

  // ── Universal GNSS time anchor (campaign-independent) ──
  time_pps_update(
    dwt_at_pps_edge,
    g_dwt_cal_valid ? g_dwt_cycles_per_gnss_s : 0,
    isr_snap_gnss
  );

  // ── OCXO phase capture — shadow-write TDC ──
  //
  // GPT output compares were armed by pps_isr with a generous offset.
  // We now spin a shadow-write loop until both ISRs fire.  The GPT
  // ISRs capture the shadow DWT alongside their own DWT read, giving
  // us the before/after pair needed for TDC correction.
  //
  // This runs in scheduled context.  GPT ISRs at priority 0 preempt
  // cleanly.  The arm offset guarantees the compares fire after
  // this loop starts.

  {
    ocxo_phase.dwt_at_pps = dwt_at_pps_edge;

    // Per-shot clean slate for beta-owned derived fields.
    ocxo_phase.pps_gnss_ns            = 0;
    ocxo_phase.ocxo1_edge_gnss_ns     = 0;
    ocxo_phase.ocxo2_edge_gnss_ns     = 0;
    ocxo_phase.ocxo1_gnss_ns_per_pps  = 0;
    ocxo_phase.ocxo2_gnss_ns_per_pps  = 0;
    ocxo_phase.ocxo1_residual_ns      = 0;
    ocxo_phase.ocxo2_residual_ns      = 0;
    ocxo_phase.residual_valid         = false;
    ocxo_phase.ocxo1_dwt_elapsed      = 0;
    ocxo_phase.ocxo2_dwt_elapsed      = 0;
    ocxo_phase.ocxo1_elapsed_ns       = 0;
    ocxo_phase.ocxo2_elapsed_ns       = 0;
    ocxo_phase.ocxo1_phase_offset_ns  = 0;
    ocxo_phase.ocxo2_phase_offset_ns  = 0;

    // ── Shadow-write spin loop — unified for both OCXOs ──
    //
    // Continuously write DWT_CYCCNT to the shared shadow variable.
    // Each GPT ISR captures the shadow at preemption, giving us a
    // before/after pair for TDC correction.  The loop terminates
    // when both ISRs have fired, or on timeout.
    const uint32_t spin_start = DWT_CYCCNT;
    bool spin_timed_out = false;

    while (!ocxo1_phase_captured || !ocxo2_phase_captured) {
      ocxo_phase_shadow_dwt = DWT_CYCCNT;
      if ((DWT_CYCCNT - spin_start) > OCXO_PHASE_SPIN_TIMEOUT_CYCLES) {
        spin_timed_out = true;
        diag_ocxo_phase_spin_timeouts++;
        break;
      }
    }

    // ── Harvest captures ──
    if (ocxo1_phase_captured && !spin_timed_out) {
      ocxo_phase.ocxo1_isr_dwt     = ocxo1_phase_isr_dwt;
      ocxo_phase.ocxo1_shadow_dwt  = ocxo1_phase_shadow_dwt;
      ocxo_phase.ocxo1_gpt_at_fire = ocxo1_phase_gpt_at_fire;
      ocxo_phase.ocxo1_delta_cycles =
        (int32_t)(ocxo1_phase_isr_dwt - ocxo1_phase_shadow_dwt);
      ocxo_phase.ocxo1_captured    = true;
      ocxo_phase.ocxo1_valid       = true;

      diag_ocxo1_phase_captures++;
    } else {
      ocxo_phase.ocxo1_captured = false;
      ocxo_phase.ocxo1_valid    = false;

      diag_ocxo1_phase_misses++;
    }

    if (ocxo2_phase_captured && !spin_timed_out) {
      ocxo_phase.ocxo2_isr_dwt     = ocxo2_phase_isr_dwt;
      ocxo_phase.ocxo2_shadow_dwt  = ocxo2_phase_shadow_dwt;
      ocxo_phase.ocxo2_gpt_at_fire = ocxo2_phase_gpt_at_fire;
      ocxo_phase.ocxo2_delta_cycles =
        (int32_t)(ocxo2_phase_isr_dwt - ocxo2_phase_shadow_dwt);
      ocxo_phase.ocxo2_captured    = true;
      ocxo_phase.ocxo2_valid       = true;

      diag_ocxo2_phase_captures++;
    } else {
      ocxo_phase.ocxo2_captured = false;
      ocxo_phase.ocxo2_valid    = false;

      diag_ocxo2_phase_misses++;
    }

    // Detector valid only if BOTH edges captured
    ocxo_phase.detector_valid =
      ocxo_phase.ocxo1_valid && ocxo_phase.ocxo2_valid;
  }

  // ── Campaign-scoped processing (beta) ──
  // Must run AFTER phase harvest so beta consumes this PPS's edge captures.
  clocks_beta_pps();

  diag_pps_asap_dispatched++;
  pps_scheduled = false;
}

static void pps_isr(void)
{
  const uint32_t snap_dwt   = DWT_CYCCNT;
  const uint32_t snap_ocxo1 = GPT1_CNT;
  const uint32_t snap_ocxo2 = GPT2_CNT;
  const uint32_t snap_gnss  = qtimer1_read_32();

  // ── PPS relay pulse to Pi — unconditional, never gated ──
  digitalWriteFast(GNSS_PPS_RELAY, HIGH);
  if (!relay_timer_active) {
    relay_timer_active = true;
    timepop_arm(PPS_RELAY_OFF_NS, false, pps_relay_deassert, nullptr, "pps-relay-off");
  }

  // ── PPS edge validation — reject spurious edges ──
  //
  // At 10 MHz, elapsed between valid PPS edges should be exactly
  // 10,000,000 ± PPS_VCLOCK_TOLERANCE ticks.  Any remainder outside
  // that window indicates a spurious interrupt, not a real PPS edge.
  if (isr_residual_valid) {
    uint32_t elapsed = snap_gnss - isr_prev_gnss;
    uint32_t remainder = elapsed % ISR_GNSS_RAW_EXPECTED;
    if (remainder > PPS_VCLOCK_TOLERANCE &&
        remainder < (ISR_GNSS_RAW_EXPECTED - PPS_VCLOCK_TOLERANCE)) {

      diag_pps_rejected_total++;
      diag_pps_rejected_remainder = remainder;
      diag_pps_reject_consecutive++;

      if (diag_pps_reject_consecutive > diag_pps_reject_max_run) {
        diag_pps_reject_max_run = diag_pps_reject_consecutive;
      }

      if (diag_pps_reject_consecutive >= PPS_REJECT_RECOVERY_THRESHOLD
          && campaign_state == clocks_campaign_state_t::STARTED) {
        diag_pps_reject_recoveries++;
        clocks_watchdog_anomaly(
          "pps_reject_threshold",
          remainder,
          diag_pps_reject_consecutive,
          snap_gnss,
          isr_prev_gnss
        );
      }

      return;
    }
  }

  diag_pps_reject_consecutive = 0;

  // ── Arm GPT output compares for OCXO phase capture ──
  //
  // Must happen in PPS ISR while OCXO counters are close to their
  // snapshot values.  The shadow-write loop runs later in the ASAP
  // callback; the arm offset must be large enough that the compare
  // fires AFTER the shadow loop starts (~2-3 µs dispatch latency
  // + ~1 µs of ASAP callback preamble).
  ocxo1_phase_captured = false;
  ocxo2_phase_captured = false;

  GPT1_OCR1 = snap_ocxo1 + OCXO_PHASE_ARM_OFFSET_TICKS;
  GPT1_SR   = GPT_SR_OF1;
  GPT1_IR  |= GPT_IR_OF1IE;

  GPT2_OCR1 = snap_ocxo2 + OCXO_PHASE_ARM_OFFSET_TICKS;
  GPT2_SR   = GPT_SR_OF1;
  GPT2_IR  |= GPT_IR_OF1IE;

  // ── Capture spin loop shadow before signaling pps_fired ──
  isr_captured_shadow_dwt = dispatch_shadow_dwt;
  pps_fired = true;

  // ── ISR-level residuals ──
  //
  // All residuals are now in native units:
  //   DWT:   cycles  (expected ~1,008,000,000)
  //   GNSS:  10 MHz ticks (expected 10,000,000)
  //   OCXO1: 10 MHz ticks (expected 10,000,000)
  //   OCXO2: 10 MHz ticks (expected 10,000,000)
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



  // ── v16: PPS watchdog ──
  if (pps_scheduled) {
    diag_pps_scheduled_stuck++;

    if (diag_pps_scheduled_stuck == 1) {
      diag_pps_stuck_since_dwt = snap_dwt;
    }

    if (diag_pps_scheduled_stuck > diag_pps_stuck_max) {
      diag_pps_stuck_max = diag_pps_scheduled_stuck;
    }

    if (diag_pps_scheduled_stuck < PPS_WATCHDOG_THRESHOLD) {
      return;
    }

    diag_pps_watchdog_recoveries++;
    clocks_watchdog_anomaly(
      "pps_scheduled_stuck",
      diag_pps_scheduled_stuck,
      diag_pps_asap_armed,
      diag_pps_asap_dispatched,
      diag_pps_stuck_since_dwt
    );
    return;
  } else {
    diag_pps_scheduled_stuck = 0;
    diag_pps_stuck_since_dwt = 0;
  }

  pps_scheduled = true;

  // Mark first PPS seen for residual tracking on next edge.
  if (!isr_residual_valid) {
    isr_residual_valid = true;
  }

  timepop_handle_t h = timepop_arm(0, false, pps_asap_callback, nullptr, "pps");
  if (h == TIMEPOP_INVALID_HANDLE) {
    pps_scheduled = false;
    diag_pps_asap_arm_failures++;
  } else {
    diag_pps_asap_armed++;
  }
}

// ============================================================================
// Initialization — Phase 1 (hardware only, no TimePop dependency)
// ============================================================================

void process_clocks_init_hardware(void) {
  dwt_enable();
  arm_gpt1_external();      // OCXO1 10 MHz (GPT1, pin 25)
  arm_gpt2_external();      // OCXO2 10 MHz (GPT2, pin 14)
  arm_qtimer1_external();   // GNSS VCLOCK 10 MHz (QTimer1 ch0+ch1, pin 10)
}

// ============================================================================
// Initialization — Phase 2 (full lifecycle, requires TimePop)
// ============================================================================

void process_clocks_init(void) {

  time_init();
  timebase_init();

  g_ad5693r_init_ok = ad5693r_init();

  ocxo_dac_set(ocxo1_dac, (double)AD5693R_DAC_DEFAULT);
  ocxo_dac_set(ocxo2_dac, (double)AD5693R_DAC_DEFAULT);

  pinMode(GNSS_PPS_PIN,   INPUT);
  pinMode(GNSS_LOCK_PIN,  INPUT);
  pinMode(GNSS_PPS_RELAY, OUTPUT);
  digitalWriteFast(GNSS_PPS_RELAY, LOW);

  // ── Install GPT ISR vectors for OCXO phase capture ──
  GPT1_SR = 0x3F;
  GPT2_SR = 0x3F;
  GPT1_IR = 0;
  GPT2_IR = 0;

  attachInterruptVector(IRQ_GPT1, gpt1_phase_isr);
  NVIC_SET_PRIORITY(IRQ_GPT1, 0);
  NVIC_ENABLE_IRQ(IRQ_GPT1);

  attachInterruptVector(IRQ_GPT2, gpt2_phase_isr);
  NVIC_SET_PRIORITY(IRQ_GPT2, 0);
  NVIC_ENABLE_IRQ(IRQ_GPT2);

  attachInterrupt(digitalPinToInterrupt(GNSS_PPS_PIN), pps_isr, RISING);
  NVIC_SET_PRIORITY(IRQ_GPIO6789, 0);
}