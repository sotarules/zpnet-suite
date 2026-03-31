// ============================================================================
// process_clocks_alpha.cpp — Always-On Physics Layer
// ============================================================================

#include "process_clocks_internal.h"
#include "process_clocks.h"
#include "process_timepop.h"
#include "process_interrupt.h"
#include "tdc_correction.h"

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

static constexpr int64_t SPIN_EARLY_NS = 10000LL;
static constexpr uint64_t PPS_RELAY_OFF_NS  = 500000000ULL;

static constexpr uint32_t PPS_WATCHDOG_THRESHOLD = 3;
static constexpr uint32_t PPS_REJECT_RECOVERY_THRESHOLD = 10;

static constexpr uint32_t SPIN_LOOP_TIMEOUT_CYCLES = 100800;
static constexpr uint32_t OCXO_PHASE_SPIN_TIMEOUT_CYCLES = 50400;

static constexpr uint64_t TIME_TEST_DELAY_MIN_NS = 100000000ULL;
static constexpr uint64_t TIME_TEST_DELAY_MAX_NS = 900000000ULL;

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

// ============================================================================
// TIME_TEST — state and interrupt capture integration
// ============================================================================

time_test_capture_t time_test = {};
volatile uint32_t time_test_shadow_dwt = 0;
static volatile uint32_t time_test_next_counter32_target = 0;

// Forward
static interrupt_next_target_t time_test_interrupt_event_handler(
    const interrupt_event_t& event,
    const interrupt_capture_diag_t* diag,
    void* user_data);

// ============================================================================
// PPS spin callback
// ============================================================================

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

  dbg_post_loop_dwt      = DWT_CYCCNT;
  dbg_post_loop_shadow   = dispatch_shadow_dwt;
  dbg_post_loop_isr_cap  = isr_captured_shadow_dwt;
  dbg_post_loop_isr_snap = isr_snap_dwt;
}

// ============================================================================
// TIME_TEST — new subscriber callback
// ============================================================================

static interrupt_next_target_t time_test_interrupt_event_handler(
    const interrupt_event_t& event,
    const interrupt_capture_diag_t* diag,
    void*) {

  time_test.captured                   = true;
  time_test.dwt_at_event               = event.dwt_at_event;
  time_test.gnss_ns_at_event           = event.gnss_ns_at_event;
  time_test.counter32_at_event         = event.counter32_at_event;
  time_test.dwt_event_correction_cycles = event.dwt_event_correction_cycles;

  time_test.dwt_isr_entry_raw          = event.raw.dwt_isr_entry_raw;
  time_test.dwt_after_capture          = event.raw.dwt_after_capture;
  time_test.raw_compare16              = event.raw.compare16;
  time_test.raw_verify_low16           = event.raw.verify_low16;
  time_test.raw_verify_high16          = event.raw.verify_high16;

  if (diag) {
    time_test.shadow_valid                     = diag->shadow_valid;
    time_test.shadow_dwt                       = diag->shadow_dwt;
    time_test.shadow_to_isr_entry_cycles       = diag->shadow_to_isr_entry_cycles;
    time_test.dwt_at_event_adjusted            = diag->dwt_at_event_adjusted;

    time_test.candidate_correction_default_cycles = diag->candidate_correction_default_cycles;
    time_test.candidate_correction_live_cycles    = diag->candidate_correction_live_cycles;
    time_test.dwt_event_correction_cycles_diag    = diag->dwt_event_correction_cycles;
    time_test.used_live_profile                   = diag->used_live_profile;
    time_test.used_default_profile                = diag->used_default_profile;

    time_test.expected_low16                 = diag->expected_low16;
    time_test.expected_high16                = diag->expected_high16;
    time_test.verify_high16_matches          = diag->verify_high16_matches;
    time_test.verify_high16_is_previous      = diag->verify_high16_is_previous;
  } else {
    time_test.shadow_valid = false;
    time_test.shadow_dwt = 0;
    time_test.shadow_to_isr_entry_cycles = 0;
    time_test.dwt_at_event_adjusted = 0;

    time_test.candidate_correction_default_cycles = 0;
    time_test.candidate_correction_live_cycles = 0;
    time_test.dwt_event_correction_cycles_diag = 0;
    time_test.used_live_profile = false;
    time_test.used_default_profile = false;

    time_test.expected_low16 = 0;
    time_test.expected_high16 = 0;
    time_test.verify_high16_matches = false;
    time_test.verify_high16_is_previous = false;
  }

  time_test.ch3_isr_fires++;

  time_anchor_snapshot_t snap = time_anchor_snapshot();
  if (!snap.ok || !snap.valid) {
    time_test.tests_time_invalid++;
    interrupt_next_target_t out {};
    out.schedule_next = false;
    out.target_gnss_ns = 0;
    return out;
  }

  time_test.diag_anchor_qtimer    = snap.qtimer_at_pps;
  time_test.diag_anchor_pps_count = snap.pps_count;
  time_test.diag_ticks_since_pps  = time_test.counter32_at_event - snap.qtimer_at_pps;

  const uint32_t ticks_since_pps = time_test.counter32_at_event - snap.qtimer_at_pps;
  const int64_t pps_gnss_ns = (int64_t)(snap.pps_count - 1) * 1000000000LL;

  time_test.vclock_gnss_ns = pps_gnss_ns + (int64_t)ticks_since_pps * 100LL;
  time_test.residual_ns = (int32_t)((int64_t)time_test.gnss_ns_at_event - time_test.vclock_gnss_ns);

  time_test.tests_valid++;

  interrupt_next_target_t out {};
  out.schedule_next = false;
  out.target_gnss_ns = 0;
  return out;
}

// ============================================================================
// TIME_TEST — schedule a new one-shot request
// ============================================================================

static void time_test_arm(void) {
  if (!g_dwt_cal_valid) return;
  if (!time_valid()) return;

  time_test.tests_run++;

  uint64_t range = TIME_TEST_DELAY_MAX_NS - TIME_TEST_DELAY_MIN_NS;
  uint64_t delay_ns = TIME_TEST_DELAY_MIN_NS + (random() % range);

  int64_t now_ns = time_gnss_ns_now();
  if (now_ns < 0) {
    time_test.tests_time_invalid++;
    return;
  }

  int64_t target_gnss_ns = now_ns + (int64_t)delay_ns;
  uint32_t target_dwt = time_gnss_ns_to_dwt(target_gnss_ns);

  time_anchor_snapshot_t snap = time_anchor_snapshot();
  if (!snap.ok || !snap.valid) {
    time_test.tests_time_invalid++;
    return;
  }

  const uint64_t ns_since_pps = (uint64_t)(target_gnss_ns -
      ((int64_t)(snap.pps_count - 1) * 1000000000LL));

  const uint32_t ticks_since_pps = (uint32_t)(ns_since_pps / 100ULL);
  time_test_next_counter32_target = snap.qtimer_at_pps + ticks_since_pps;

  time_test_shadow_dwt = 0;
  const uint32_t spin_start = DWT_CYCCNT;
  while ((DWT_CYCCNT - spin_start) < 1000) {
    time_test_shadow_dwt = DWT_CYCCNT;
    break;
  }

  interrupt_schedule_target(interrupt_subscriber_kind_t::TIME_TEST,
                            (uint64_t)target_gnss_ns);
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
      true
  );

  if (h == TIMEPOP_INVALID_HANDLE) {
    pps_spin.armed = false;
    pps_spin.arm_failures++;
  } else {
    pps_spin.armed = true;
    pps_spin.handle = h;
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
  pps_spin.approach_cycles = (int32_t)(isr_dwt_snap - pps_spin.landed_dwt);
  pps_spin.edge_dwt        = pps_dwt_at_edge(isr_dwt_snap);

  pps_spin.valid  = true;
  pps_spin.armed  = false;
  pps_spin.handle = TIMEPOP_INVALID_HANDLE;
}

// ============================================================================
// Continuous DWT-to-GNSS Calibration (campaign-independent)
// ============================================================================

volatile uint32_t  g_dwt_at_last_pps       = 0;
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

volatile uint32_t diag_pps_correct_dwt_ocxo1 = 0;
volatile uint32_t diag_pps_correct_dwt_ocxo2 = 0;
volatile uint32_t diag_pps_correct_dwt_gnss  = 0;

// ============================================================================
// PPS state
// ============================================================================

volatile bool pps_fired = false;
volatile bool pps_scheduled = false;

// ============================================================================
// OCXO phase capture — v29 shadow-write TDC architecture
// ============================================================================

volatile uint32_t ocxo_phase_shadow_dwt = 0;

volatile uint32_t ocxo1_phase_isr_dwt      = 0;
volatile uint32_t ocxo1_phase_shadow_dwt   = 0;
volatile bool     ocxo1_phase_captured     = false;
volatile uint32_t ocxo1_phase_gpt_at_fire  = 0;

volatile uint32_t ocxo2_phase_isr_dwt      = 0;
volatile uint32_t ocxo2_phase_shadow_dwt   = 0;
volatile bool     ocxo2_phase_captured     = false;
volatile uint32_t ocxo2_phase_gpt_at_fire  = 0;

volatile uint32_t diag_ocxo1_phase_captures = 0;
volatile uint32_t diag_ocxo2_phase_captures = 0;
volatile uint32_t diag_ocxo1_phase_misses   = 0;
volatile uint32_t diag_ocxo2_phase_misses   = 0;

volatile uint32_t diag_ocxo_phase_spin_timeouts = 0;

ocxo_phase_capture_t ocxo_phase = {};

volatile uint32_t diag_gpt1_isr_fires = 0;
volatile uint32_t diag_gpt2_isr_fires = 0;

// ============================================================================
// GPT1 output-compare ISR — OCXO1 edge capture
// ============================================================================

static void gpt1_phase_isr(void) {
  const uint32_t dwt_raw = DWT_CYCCNT;
  const uint32_t shadow  = ocxo_phase_shadow_dwt;
  const uint32_t gpt     = GPT1_CNT;

  GPT1_SR = GPT_SR_OF1;
  GPT1_IR &= ~GPT_IR_OF1IE;

  ocxo1_phase_isr_dwt     = dwt_raw;
  ocxo1_phase_shadow_dwt  = shadow;
  ocxo1_phase_gpt_at_fire = gpt;
  ocxo1_phase_captured    = true;

  diag_gpt1_isr_fires++;
}

static void gpt2_phase_isr(void) {
  const uint32_t dwt_raw = DWT_CYCCNT;
  const uint32_t shadow  = ocxo_phase_shadow_dwt;
  const uint32_t gpt     = GPT2_CNT;

  GPT2_SR = GPT_SR_OF1;
  GPT2_IR &= ~GPT_IR_OF1IE;

  ocxo2_phase_isr_dwt     = dwt_raw;
  ocxo2_phase_shadow_dwt  = shadow;
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

  const uint16_t lo1 = IMXRT_TMR1.CH[0].CNTR;
  const uint16_t hi1 = IMXRT_TMR1.CH[1].HOLD;

  const uint16_t lo2 = IMXRT_TMR1.CH[0].CNTR;
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

  IMXRT_TMR1.CH[0].CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);
  IMXRT_TMR1.CH[1].CTRL = TMR_CTRL_CM(7);

  IMXRT_TMR1.CH[0].CSCTRL &= ~TMR_CSCTRL_TCF1EN;
  IMXRT_TMR1.CH[0].CSCTRL |=  TMR_CSCTRL_TCF1;

  IMXRT_TMR1.CH[1].CSCTRL = 0;
  IMXRT_TMR1.CH[1].SCTRL  &= ~(TMR_SCTRL_TOFIE | TMR_SCTRL_IEF | TMR_SCTRL_IEFIE);

  gnss_rolling_32 = qtimer1_read_32();
  qtimer1_armed   = true;
}

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

  pps_spin_complete(isr_snap_dwt);
  pps_spin_arm();

  const uint32_t dwt_at_pps_edge = pps_dwt_at_edge(isr_snap_dwt);

  {
    if (g_dwt_cal_has_prev) {
      g_dwt_cycles_per_gnss_s = dwt_at_pps_edge - g_dwt_at_last_pps;
      g_dwt_cal_valid = true;
    }

    g_dwt_at_last_pps  = dwt_at_pps_edge;
    g_dwt_cal_has_prev = true;
    g_dwt_cal_pps_count++;
  }

  time_pps_update(
    dwt_at_pps_edge,
    g_dwt_cal_valid ? g_dwt_cycles_per_gnss_s : 0,
    isr_snap_gnss
  );

  {
    ocxo_phase.dwt_at_pps = dwt_at_pps_edge;

    ocxo_phase.pps_gnss_ns            = 0;
    ocxo_phase.ocxo1_edge_dwt         = 0;
    ocxo_phase.ocxo2_edge_dwt         = 0;
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

    if (ocxo1_phase_captured && !spin_timed_out) {
      ocxo_phase.ocxo1_isr_dwt     = ocxo1_phase_isr_dwt;
      ocxo_phase.ocxo1_shadow_dwt  = ocxo1_phase_shadow_dwt;
      ocxo_phase.ocxo1_edge_dwt    = gpt_dwt_at_edge(ocxo1_phase_isr_dwt);
      ocxo_phase.ocxo1_gpt_at_fire = ocxo1_phase_gpt_at_fire;
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
      ocxo_phase.ocxo2_edge_dwt    = gpt_dwt_at_edge(ocxo2_phase_isr_dwt);
      ocxo_phase.ocxo2_gpt_at_fire = ocxo2_phase_gpt_at_fire;
      ocxo_phase.ocxo2_captured    = true;
      ocxo_phase.ocxo2_valid       = true;

      diag_ocxo2_phase_captures++;
    } else {
      ocxo_phase.ocxo2_captured = false;
      ocxo_phase.ocxo2_valid    = false;

      diag_ocxo2_phase_misses++;
    }

    ocxo_phase.detector_valid =
      ocxo_phase.ocxo1_valid && ocxo_phase.ocxo2_valid;
  }

  time_test_arm();

  clocks_beta_pps();

  diag_pps_asap_dispatched++;
  pps_scheduled = false;
}

static void pps_isr(void)  {

  const uint32_t snap_dwt   = DWT_CYCCNT;
  const uint32_t snap_ocxo1 = pps_correct_10mhz(GPT1_CNT, snap_dwt, &diag_pps_correct_dwt_ocxo1);
  const uint32_t snap_ocxo2 = pps_correct_10mhz(GPT2_CNT, snap_dwt, &diag_pps_correct_dwt_ocxo2);
  const uint32_t snap_gnss  = pps_correct_10mhz(qtimer1_read_32(), snap_dwt, &diag_pps_correct_dwt_gnss);

  digitalWriteFast(GNSS_PPS_RELAY, HIGH);
  if (!relay_timer_active) {
    relay_timer_active = true;
    timepop_arm(PPS_RELAY_OFF_NS, false, pps_relay_deassert, nullptr, "pps-relay-off");
  }

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

  ocxo1_phase_captured = false;
  ocxo2_phase_captured = false;

  GPT1_OCR1 = snap_ocxo1 + OCXO_PHASE_ARM_OFFSET_TICKS;
  GPT1_SR   = GPT_SR_OF1;
  GPT1_IR  |= GPT_IR_OF1IE;

  GPT2_OCR1 = snap_ocxo2 + OCXO_PHASE_ARM_OFFSET_TICKS;
  GPT2_SR   = GPT_SR_OF1;
  GPT2_IR  |= GPT_IR_OF1IE;

  isr_captured_shadow_dwt = dispatch_shadow_dwt;
  pps_fired = true;

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
  arm_gpt1_external();
  arm_gpt2_external();
  arm_qtimer1_external();
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

  interrupt_subscription_t time_test_sub {};
  time_test_sub.kind = interrupt_subscriber_kind_t::TIME_TEST;
  time_test_sub.on_event = time_test_interrupt_event_handler;
  time_test_sub.user_data = nullptr;
  interrupt_subscribe(time_test_sub);
  interrupt_start(interrupt_subscriber_kind_t::TIME_TEST);
}