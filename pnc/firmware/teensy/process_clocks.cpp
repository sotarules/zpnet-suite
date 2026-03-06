// ============================================================================
// process_clocks.cpp — ZPNet CLOCKS (Teensy)
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
// All three counters are structurally identical:
//   • Hardware register counts autonomously
//   • Software periodically latches and extends to 64-bit
//   • No prescaler, no ISR, no interrupts
//
// At 10 MHz the GPT counters wrap every ~429 seconds (~7 minutes).
// The 1-second PPS publish guarantees we read them well before wrap.
//
// 64-bit extension architecture:
//
//   Each clock domain has exactly ONE function that touches the
//   64-bit accumulator: clocks_xxx_now().  This function:
//     1) Reads the live 32-bit hardware register
//     2) Computes (uint32_t)(now - last_32) — always positive,
//        wrap-safe as long as called within 2^31 ticks
//     3) Adds the delta to the 64-bit accumulator
//     4) Updates last_32
//     5) Returns the 64-bit value
//
//   This function can be called from ANY context at ANY time:
//   ISR, callback, command handler, diagnostic.  Multiple calls
//   per PPS interval are harmless — each just ratchets forward.
//
//   To obtain the 64-bit value at a PAST moment (e.g., the PPS
//   edge captured in the ISR), the PPS callback:
//     1) Calls _now() to advance the accumulator to the present
//     2) Computes (uint32_t)(last_32 - isr_snapshot) to get the
//        small number of ticks between the snapshot and now
//     3) Subtracts from the 64-bit value: snap_64 = now_64 - delta
//
//   This subtraction is always safe because the delta is small
//   (microseconds between ISR and callback) and unambiguous.
//
// ISR-level hardware snapshot and raw residuals:
//
//   All three hardware counters (DWT_CYCCNT, GPT2_CNT, GPT1_CNT)
//   are latched inside the PPS ISR itself — the absolute closest
//   point to the PPS rising edge that software can reach.
//
//   Raw PPS-to-PPS residuals are computed directly in the ISR from
//   consecutive 32-bit snapshots:
//
//     residual = (snap_now - snap_prev) - expected_ticks
//
//   These "ISR residuals" are published in the TIMEBASE_FRAGMENT
//   as isr_residual_dwt/gnss/ocxo.  They represent the purest
//   possible measurement — no 64-bit extension, no callback
//   latency, no processing of any kind between the PPS edge and
//   the subtraction.
//
//   The existing Welford-based residual tracking (computed from
//   64-bit extended values in the callback) is retained for
//   running statistics.
//
// PPS relay to Pi:
//
//   The raw GNSS PPS from the GF-8802 arrives on pin 1 via
//   shielded twisted pair — this path achieves sub-nanosecond
//   measurement noise and must not be compromised.
//
//   The Pi needs a PPS signal on GPIO 18 for chrony discipline.
//   Rather than splitting the GF-8802 signal (which would
//   degrade the Teensy's pristine capture), the Teensy relays
//   the PPS from pin 32 (GNSS_PPS_RELAY).
//
//   Immediately after the three hardware snapshots in the ISR,
//   pin 32 is driven HIGH.  The ISR sets a flag; the ASAP
//   callback (running in scheduled context) arms a one-shot
//   TimePop that fires 500 ms later to drive pin 32 LOW.
//
//   timepop_arm is NEVER called from ISR context.  Its internal
//   noInterrupts/interrupts pair can re-enable interrupts mid-ISR,
//   causing the PIT to nest into the PPS handler and corrupt
//   timer state.  All timepop arming happens in scheduled context.
//
//   The relay latency is deterministic (~20-30 ns after the
//   snapshots, ~30-50 ns after the true PPS edge) and constant.
//   This is well within chrony's discipline capability.
//
// Recovery:
//
//   When the Pi restarts mid-campaign, it reads the last TIMEBASE
//   from Postgres, projects all clock values forward using tau,
//   and sends RECOVER with the projected dwt_ns, gnss_ns, and
//   ocxo_ns — all nanosecond values.  The Teensy derives its own
//   internal cycle/tick counts from the nanosecond values using
//   its known conversion ratios (DWT: ns * 126/125, GNSS/OCXO:
//   ns / 100).  At the next PPS edge, the Teensy loads these
//   values into its 64-bit accumulators, latches the current
//   hardware register positions, transitions to STARTED, and
//   resumes publishing TIMEBASE_FRAGMENTs.  The clocks continue
//   as if the interruption never happened.
//
//   The cycle count is an internal implementation detail of the
//   Teensy — CLOCKS on the Pi speaks only nanoseconds.  This
//   makes recovery symmetric across all clock domains: the same
//   formula (last_ns + elapsed * tau) computes the projected
//   nanosecond value for every domain.
//
// Pre-PPS Software TDC (Time-to-Digital Converter):
//
//   Two-stage timer chain + spin loop + deterministic correction.
//
//   Stage 1 (coarse, PIT0): 998 ms after PPS.
//   Stage 2 (fine, PIT1):   999 µs after coarse.
//   Spin loop:              ~400 µs until PPS edge.
//
//   The spin loop continuously writes DWT_CYCCNT to a volatile
//   shadow.  The PPS ISR captures the shadow atomically, then
//   sets pps_fired to break the loop.
//
//   The delta (ISR snapshot - captured shadow) reveals which
//   instruction was executing when the interrupt fired.  A
//   deterministic correction table (derived from disassembly
//   analysis) converts this into the true PPS-edge DWT value.
//
//   Result: PPS edge timestamped to ±1 DWT cycle (~0.99 ns)
//   on a $30 microcontroller.  0.04% CPU cost.
//
// OCXO DAC Control:
//
//   The AOCJY1-A OCXO frequency is trimmed via Pin 1 (CTL),
//   driven by Teensy DAC on pin 22 (A22).  12-bit resolution,
//   0–3.3V output directly connected to the OCXO's 10 kΩ
//   control input.
//
//   The DAC is initialized at boot to a default midpoint value.
//   The current DAC value is always published in TIMEBASE_FRAGMENT
//   regardless of campaign or calibration state.
//
//   Optional calibration servo: when a campaign is started with
//   the calibrate_ocxo flag, a perpetual software servo adjusts
//   the DAC to continuously drive the OCXO residual toward zero
//   (tau → 1.0).  The servo never converges — it keeps tracking
//   to compensate for thermal drift, supply changes, and aging.
//
//   The Pi persists the converged DAC value and sends it back
//   via set_dac on subsequent campaign starts.  The Teensy
//   never remembers anything across reboots.
//
// TIMEPULSE (10 KHz high-resolution clock domain interpolation):
//
//   GPT2 Output Compare 1 fires every TIMEPULSE_GNSS_HALF_PERIOD
//   (500 GNSS ticks = 50 µs) and toggles GNSS_10KHZ_RELAY.
//   This produces a 10 KHz square wave (full period = 1,000 GNSS
//   ticks = 100 µs).
//
//   On the rising edge only (when toggle transitions to HIGH),
//   the ISR:
//     1) Captures DWT_CYCCNT
//     2) Increments the quantized GNSS clock by TIMEPULSE_NS_PER_TICK
//        (100,000 ns = 100 µs)
//     3) Writes both values to a volatile anchor struct
//
//   This anchor is the TIMEPULSE "publication" on the Teensy:
//   a direct memory read, zero allocation, zero dispatch.
//   Timebase (and any future consumer) reads it synchronously.
//
//   TIMEPULSE diagnostics are published once per PPS as part of
//   TIMEBASE_FRAGMENT (fields prefixed timepulse_).  The separate
//   TIMEPULSE_TEENSY message has been retired — consumers should
//   subscribe to TIMEBASE_FRAGMENT instead.
//
//   At every PPS edge, the PPS ISR captures the three hardware
//   counters.  The 10 KHz grid is inherently phase-coherent with
//   PPS because it is derived from the same GNSS VCLOCK by a
//   fixed integer division.  No realignment is performed.
//
//   PPS observation model:
//
//   TIMEPULSE is pure math on a proven-coherent foundation.
//   The 10 MHz VCLOCK is phase-locked to PPS (empirically
//   verified: zero residual on every sample).  GPT2 output
//   compare divides by 1,000.  The ISR increments gnss_ns
//   by TIMEPULSE_NS_PER_TICK on each rising edge.  This
//   produces exactly 10,000 ticks per second and gnss_ns
//   advances by exactly 1,000,000,000 ns.  Correct by
//   construction.  No compensation, no snapping, no
//   synthetic corrections of any kind.
//
//   At each PPS, the ASAP callback observes the state of
//   the anchor as a diagnostic.  timepulse_phase_error_ns
//   reports the difference between gnss_ns and the expected
//   value.  A nonzero phase error is evidence of a problem
//   below TIMEPULSE, not something to correct at this level.
//
//   timepulse_ticks_at_pps records the number of rising edges
//   the ISR delivered in the preceding second.  This is a pure
//   observation.
//
//   The quantized clock is zeroed on campaign START, restored on
//   RECOVER, and publication is gated on campaign state.  The
//   10 KHz hardware signal runs continuously from boot regardless
//   of campaign state.
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

// ============================================================================
// DWT nanosecond conversion helpers
// ============================================================================

static inline uint64_t dwt_cycles_to_ns(uint64_t cycles) {
  return (cycles * DWT_NS_NUM) / DWT_NS_DEN;
}

static inline uint32_t dwt_cycles_to_ns_32(uint32_t cycles) {
  return (cycles * (uint32_t)DWT_NS_NUM) / (uint32_t)DWT_NS_DEN;
}

static inline uint64_t dwt_ns_to_cycles(uint64_t ns) {
  return (ns * DWT_NS_DEN) / DWT_NS_NUM;
}

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
// TIMEPULSE — 10 KHz Anchor for Clock Domain Interpolation
// ============================================================================
//
// TIMEPULSE is pure math.  The ISR increments gnss_ns on each
// rising edge.  At PPS we observe — we do not correct.
// timepulse_ticks_at_pps and timepulse_phase_error_ns are
// read-only diagnostics.
//

struct timepulse_anchor_t {
  volatile uint64_t gnss_ns;
  volatile uint32_t dwt_snap;
  volatile bool     valid;
};

static timepulse_anchor_t timepulse_anchor = {0, 0, false};

static volatile uint32_t timepulse_tick_count      = 0;
static volatile uint32_t timepulse_tick_in_second  = 0;

// ============================================================================
// TIMEPULSE diagnostic counters
// ============================================================================

// ISR rising-edge count for the preceding second (diagnostic).
static volatile uint32_t tp_diag_ticks_at_pps      = 0;

static volatile int64_t  tp_diag_phase_error_ns    = 0;

// ============================================================================
// ISR-level Timebase self-check diagnostics
// ============================================================================
//
// These are captured in ISR context immediately after the relevant
// Timebase state is established or observed.
//
//   • TIMEPULSE ISR:
//       - call timebase_now_gnss_ns() immediately after anchor update
//       - compare the returned value to the just-written anchor
//
//   • PPS ISR:
//       - call timebase_now_gnss_ns() immediately after raw snapshots
//       - inspect how close the interpolated GNSS "now" is to the
//         1-second boundary
//

static volatile int64_t  diag_timebase_tp_now_gnss_ns           = -1;
static volatile int64_t  diag_timebase_tp_now_minus_anchor_ns   = -1;
static volatile int64_t  diag_timebase_tp_now_mod_tick_ns       = -1;

static volatile int64_t  diag_timebase_pps_now_gnss_ns          = -1;
static volatile int64_t  diag_timebase_pps_now_mod_1s_ns        = -1;

// ============================================================================
// ISR-level hardware snapshots and raw residuals
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
// Pre-PPS dispatch latency profiling (DWT spin loop)
// ============================================================================

static volatile uint32_t dispatch_shadow_dwt     = 0;
static volatile bool     pps_fired               = false;
static volatile uint32_t isr_captured_shadow_dwt = 0;
static volatile bool     dispatch_shadow_valid   = false;
static volatile bool     dispatch_timeout        = false;

static volatile int64_t  dispatch_shadow_ns    = -1;
static volatile int64_t  dispatch_isr_ns       = -1;
static volatile int64_t  dispatch_delta_ns     = -1;
static volatile int32_t  pre_pps_approach_ns   = -1;

static volatile int64_t  pps_edge_ns            = -1;
static volatile int32_t  pps_edge_correction_ns = -1;
static volatile bool     pps_edge_valid         = false;

// ============================================================================
// Software TDC correction table
// ============================================================================

static constexpr bool     TDC_NEEDS_RECALIBRATION = false;
static constexpr uint32_t TDC_FIXED_OVERHEAD      = 48;
static constexpr uint32_t TDC_LOOP_CYCLES         = 1;
static constexpr uint32_t TDC_MAX_CORRECTION      = 5;

static inline int32_t tdc_correction_cycles(uint32_t delta_cycles) {
  if (TDC_NEEDS_RECALIBRATION) return -1;
  if (delta_cycles < TDC_FIXED_OVERHEAD) return -1;
  uint32_t correction = delta_cycles - TDC_FIXED_OVERHEAD;
  if (correction >= TDC_MAX_CORRECTION) return -1;
  return (int32_t)correction;
}

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
// DWT (CPU cycle counter — 1008 MHz internal)
// ============================================================================

#define DEMCR               (*(volatile uint32_t *)0xE000EDFC)
#define DEMCR_TRCENA        (1 << 24)
#define DWT_CTRL            (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT          (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL_CYCCNTENA  (1 << 0)

static uint64_t dwt_cycles_64 = 0;
static uint32_t dwt_last_32   = 0;

static inline void dwt_enable(void) {
  DEMCR |= DEMCR_TRCENA;
  DWT_CYCCNT = 0;
  DWT_CTRL |= DWT_CTRL_CYCCNTENA;
  dwt_last_32 = DWT_CYCCNT;
}

uint64_t clocks_dwt_cycles_now(void) {
  uint32_t now = DWT_CYCCNT;
  dwt_cycles_64 += (uint32_t)(now - dwt_last_32);
  dwt_last_32 = now;
  return dwt_cycles_64;
}

uint64_t clocks_dwt_ns_now(void) {
  return dwt_cycles_to_ns(clocks_dwt_cycles_now());
}

// ============================================================================
// 10 KHz GNSS Relay — GPT2 Output Compare Channel 1 ISR
// ============================================================================

static volatile bool timepulse_relay_state = false;

static void gpt2_compare_isr(void) {
  GPT2_SR = GPT_SR_OF1;

  timepulse_relay_state = !timepulse_relay_state;
  digitalWriteFast(GNSS_10KHZ_RELAY, timepulse_relay_state ? HIGH : LOW);

  GPT2_OCR1 += TIMEPULSE_GNSS_HALF_PERIOD;

  if (!timepulse_relay_state) return;  // falling edge — nothing to do

  if (campaign_state != clocks_campaign_state_t::STARTED) return;

  const uint32_t snap_dwt = DWT_CYCCNT;

  timepulse_anchor.gnss_ns  += TIMEPULSE_NS_PER_TICK;
  timepulse_anchor.dwt_snap = snap_dwt;
  timepulse_anchor.valid    = true;

  timebase_update_anchor(timepulse_anchor.gnss_ns, snap_dwt);

  // Immediate Timebase self-check at the TIMEPULSE edge.
  const int64_t tb_now = timebase_now_gnss_ns();
  diag_timebase_tp_now_gnss_ns = tb_now;
  if (tb_now >= 0) {
    diag_timebase_tp_now_minus_anchor_ns =
      tb_now - (int64_t)timepulse_anchor.gnss_ns;
    diag_timebase_tp_now_mod_tick_ns =
      tb_now % (int64_t)TIMEPULSE_NS_PER_TICK;
  } else {
    diag_timebase_tp_now_minus_anchor_ns = -1;
    diag_timebase_tp_now_mod_tick_ns = -1;
  }

  timepulse_tick_count++;
  timepulse_tick_in_second++;
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

  GPT2_OCR1 = GPT2_CNT + TIMEPULSE_GNSS_HALF_PERIOD;
  GPT2_IR   = GPT_IR_OF1IE;

  attachInterruptVector(IRQ_GPT2, gpt2_compare_isr);
  NVIC_SET_PRIORITY(IRQ_GPT2, 16);
  NVIC_ENABLE_IRQ(IRQ_GPT2);

  gpt2_armed = true;
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

static uint64_t ocxo_ticks_64 = 0;
static uint32_t ocxo_last_32  = 0;
static bool     gpt1_armed    = false;

static inline void enable_gpt1(void) {
  CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON);
}

static void arm_gpt1_external(void) {
  if (gpt1_armed) return;

  enable_gpt1();

  *(portConfigRegister(25)) = 1;

  GPT1_CR = 0;
  GPT1_SR = 0x3F;
  GPT1_PR = 0;
  GPT1_CR = GPT_CR_CLKSRC(3) | GPT_CR_FRR;
  GPT1_CR |= GPT_CR_EN;

  ocxo_last_32 = GPT1_CNT;
  gpt1_armed   = true;
}

uint64_t clocks_ocxo_ticks_now(void) {
  uint32_t now = GPT1_CNT;
  ocxo_ticks_64 += (uint32_t)(now - ocxo_last_32);
  ocxo_last_32 = now;
  return ocxo_ticks_64;
}

uint64_t clocks_ocxo_ns_now(void) {
  if (campaign_state != clocks_campaign_state_t::STARTED) return 0;
  return clocks_ocxo_ticks_now() * 100ull;
}

// ============================================================================
// Advance-then-rewind: recover 64-bit value at a past ISR snapshot
// ============================================================================

static uint64_t dwt_at_pps_edge(void) {
  uint64_t now_64 = clocks_dwt_cycles_now();
  uint32_t rewind = (uint32_t)(dwt_last_32 - isr_snap_dwt);
  return now_64 - rewind;
}

static uint64_t gnss_at_pps_edge(void) {
  uint64_t now_64 = clocks_gnss_ticks_now();
  uint32_t rewind = (uint32_t)(gnss_last_32 - isr_snap_gnss);
  return now_64 - rewind;
}

static uint64_t ocxo_at_pps_edge(void) {
  uint64_t now_64 = clocks_ocxo_ticks_now();
  uint32_t rewind = (uint32_t)(ocxo_last_32 - isr_snap_ocxo);
  return now_64 - rewind;
}

// ============================================================================
// Zeroing (campaign-scoped — fresh start only, NOT used for recovery)
// ============================================================================

static void clocks_zero_all(void) {
  timebase_invalidate();

  clocks_dwt_cycles_now();
  clocks_gnss_ticks_now();
  clocks_ocxo_ticks_now();

  dwt_cycles_64 = 0;  dwt_last_32  = isr_snap_dwt;
  gnss_ticks_64 = 0;  gnss_last_32 = isr_snap_gnss;
  ocxo_ticks_64 = 0;  ocxo_last_32 = isr_snap_ocxo;

  campaign_seconds = 0;

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
  dispatch_shadow_ns      = -1;
  dispatch_isr_ns         = -1;
  dispatch_delta_ns       = -1;
  pps_edge_ns             = -1;
  pps_edge_correction_ns  = -1;
  pps_edge_valid          = false;
  pre_pps_approach_ns     = -1;

  timepulse_anchor.gnss_ns  = 0;
  timepulse_anchor.dwt_snap = 0;
  timepulse_anchor.valid    = false;
  timepulse_tick_count      = 0;
  timepulse_tick_in_second  = 0;
  tp_diag_phase_error_ns    = 0;
  tp_diag_ticks_at_pps      = 0;

  diag_timebase_tp_now_gnss_ns         = -1;
  diag_timebase_tp_now_minus_anchor_ns = -1;
  diag_timebase_tp_now_mod_tick_ns     = -1;
  diag_timebase_pps_now_gnss_ns        = -1;
  diag_timebase_pps_now_mod_1s_ns      = -1;
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
// Pre-PPS dispatch latency profiling — two-stage spin loop
// ============================================================================

static inline void mask_low_priority_irqs(uint8_t pri) {
  __asm volatile("MSR BASEPRI, %0" : : "r"(pri) : "memory");
}

static inline void unmask_all_irqs() {
  __asm volatile("MSR BASEPRI, %0" : : "r"(0) : "memory");
}

static volatile uint32_t diag_approach_gpt2_cnt     = 0;
static volatile uint32_t diag_approach_gnss_last32  = 0;
static volatile uint64_t diag_approach_gnss_ticks64 = 0;
static volatile uint64_t diag_approach_gnss_ns_now  = 0;
static volatile uint64_t diag_approach_into_second  = 0;

static void pre_pps_fine_cb(timepop_ctx_t*, void*) {
  diag_fine_fire_count++;

  uint32_t ticks_since_pps = (uint32_t)(GPT2_CNT - isr_snap_gnss);
  pre_pps_approach_ns      = (int32_t)(ISR_GNSS_EXPECTED - ticks_since_pps) * 100;

  diag_approach_gpt2_cnt     = GPT2_CNT;
  diag_approach_gnss_last32  = gnss_last_32;
  diag_approach_gnss_ticks64 = gnss_ticks_64;
  diag_approach_gnss_ns_now  = (uint64_t)ticks_since_pps * 100;
  diag_approach_into_second  = ticks_since_pps;

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
  timepop_arm(TIMEPOP_CLASS_PRE_PPS_COARSE, false, pre_pps_fine_cb, nullptr, "pre-pps");
}

// ============================================================================
// PPS handling — Phase 2: ASAP callback (scheduled context)
// ============================================================================

static volatile bool pps_scheduled = false;

static void pps_asap_callback(timepop_ctx_t*, void*) {

  if (relay_arm_pending) {
    relay_arm_pending = false;
    if (!relay_timer_active) {
      relay_timer_active = true;
      timepop_arm(TIMEPOP_CLASS_PPS_RELAY, false, pps_relay_deassert, nullptr, "pps-relay-off");
    }
  }

  // --------------------------------------------------------
  // Campaign state transitions
  // --------------------------------------------------------

  if (request_stop) {
    campaign_state = clocks_campaign_state_t::STOPPED;
    request_stop   = false;
    timepop_cancel(TIMEPOP_CLASS_PRE_PPS_COARSE);
    dispatch_shadow_valid = false;
    pps_fired             = true;
    calibrate_ocxo_active = false;
    timebase_invalidate();
  }

  if (request_recover) {

    timebase_invalidate();

    dwt_cycles_64 = dwt_ns_to_cycles(recover_dwt_ns);
    dwt_last_32   = isr_snap_dwt;

    gnss_ticks_64 = recover_gnss_ns / 100ull;
    gnss_last_32  = isr_snap_gnss;

    ocxo_ticks_64 = recover_ocxo_ns / 100ull;
    ocxo_last_32  = isr_snap_ocxo;

    campaign_seconds = recover_gnss_ns / 1000000000ull;

    residual_reset(residual_dwt);
    residual_reset(residual_gnss);
    residual_reset(residual_ocxo);

    isr_residual_valid     = false;
    dispatch_shadow_valid  = false;
    pps_fired              = true;
    dispatch_shadow_ns     = -1;
    dispatch_isr_ns        = -1;
    dispatch_delta_ns      = -1;
    pps_edge_ns            = -1;
    pps_edge_correction_ns = -1;
    pps_edge_valid         = false;
    pre_pps_approach_ns    = -1;

    uint64_t recovered_tp_ns =
      (recover_gnss_ns / TIMEPULSE_NS_PER_TICK) * TIMEPULSE_NS_PER_TICK;

    timepulse_anchor.gnss_ns  = recovered_tp_ns;
    timepulse_anchor.dwt_snap = isr_snap_dwt;
    timepulse_anchor.valid    = true;
    timepulse_tick_count      = (uint32_t)(recovered_tp_ns / TIMEPULSE_NS_PER_TICK);
    timepulse_tick_in_second  = 0;
    tp_diag_phase_error_ns    = 0;
    tp_diag_ticks_at_pps      = 0;

    diag_timebase_tp_now_gnss_ns         = -1;
    diag_timebase_tp_now_minus_anchor_ns = -1;
    diag_timebase_tp_now_mod_tick_ns     = -1;
    diag_timebase_pps_now_gnss_ns        = -1;
    diag_timebase_pps_now_mod_1s_ns      = -1;

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

    // --------------------------------------------------------
    // TIMEPULSE PPS observation
    // --------------------------------------------------------

    tp_diag_ticks_at_pps     = timepulse_tick_in_second;
    timepulse_tick_in_second = 0;

    if (timepulse_anchor.valid) {
      uint64_t expected = campaign_seconds * NS_PER_SECOND;
      tp_diag_phase_error_ns = (int64_t)timepulse_anchor.gnss_ns
                             - (int64_t)expected;
    }

    // --------------------------------------------------------
    // Pre-PPS approach time
    // --------------------------------------------------------

    if (dispatch_timeout) {
      pre_pps_approach_ns = -1;
      dispatch_timeout    = false;
    }

    pre_pps_arm();
    pps_fired = false;

    // --------------------------------------------------------
    // Recover 64-bit values at the PPS edge
    // --------------------------------------------------------

    const uint64_t snap_dwt_cycles = dwt_at_pps_edge();
    const uint64_t snap_gnss_ticks = gnss_at_pps_edge();
    const uint64_t snap_ocxo_ticks = ocxo_at_pps_edge();

    const uint64_t snap_dwt_ns  = dwt_cycles_to_ns(snap_dwt_cycles);
    const uint64_t snap_gnss_ns = snap_gnss_ticks * 100ull;
    const uint64_t snap_ocxo_ns = snap_ocxo_ticks * 100ull;

    // --------------------------------------------------------
    // Software TDC correction
    // --------------------------------------------------------

    uint64_t pps_dwt_cycles = snap_dwt_cycles;
    uint64_t pps_dwt_ns     = snap_dwt_ns;

    if (dispatch_shadow_valid) {
      uint32_t delta_cycles = isr_snap_dwt - isr_captured_shadow_dwt;
      int64_t  delta_ns     = (int64_t)dwt_cycles_to_ns_32(delta_cycles);

      dispatch_isr_ns    = (int64_t)snap_dwt_ns;
      dispatch_delta_ns  = delta_ns;
      dispatch_shadow_ns = (int64_t)snap_dwt_ns - delta_ns;

      int32_t correction = tdc_correction_cycles(delta_cycles);
      if (correction >= 0) {
        uint32_t overhead_cycles = delta_cycles - (uint32_t)correction;
        pps_dwt_cycles         = snap_dwt_cycles - overhead_cycles;
        pps_dwt_ns             = dwt_cycles_to_ns(pps_dwt_cycles);
        pps_edge_ns            = (int64_t)pps_dwt_ns;
        pps_edge_correction_ns = (int32_t)dwt_cycles_to_ns_32(correction);
        pps_edge_valid         = true;
      } else {
        pps_edge_ns            = -1;
        pps_edge_correction_ns = -1;
        pps_edge_valid         = false;
      }

      dispatch_shadow_valid = false;
    } else {
      dispatch_shadow_ns     = -1;
      dispatch_isr_ns        = -1;
      dispatch_delta_ns      = -1;
      pps_edge_ns            = -1;
      pps_edge_correction_ns = -1;
      pps_edge_valid         = false;
      dispatch_timeout       = false;
    }

    // --------------------------------------------------------
    // Welford residual tracking
    // --------------------------------------------------------

    residual_update(residual_dwt,  pps_dwt_cycles,  DWT_EXPECTED_PER_PPS);
    residual_update(residual_gnss, snap_gnss_ticks, GNSS_EXPECTED_PER_PPS);
    residual_update(residual_ocxo, snap_ocxo_ticks, OCXO_EXPECTED_PER_PPS);

    ocxo_calibration_servo();

    // --------------------------------------------------------
    // Publish TIMEBASE_FRAGMENT
    // --------------------------------------------------------

    Payload p;
    p.add("campaign",         campaign_name);

    p.add("dwt_cycles",       pps_dwt_cycles);
    p.add("dwt_ns",           pps_dwt_ns);
    p.add("gnss_ns",          snap_gnss_ns);
    p.add("ocxo_ns",          snap_ocxo_ns);
    p.add("teensy_pps_count", campaign_seconds);
    p.add("gnss_lock",        digitalRead(GNSS_LOCK_PIN));

    p.add("dwt_pps_residual",  residual_dwt.residual);
    p.add("gnss_pps_residual", residual_gnss.residual);
    p.add("ocxo_pps_residual", residual_ocxo.residual);

    p.add("isr_residual_dwt",   isr_residual_dwt);
    p.add("isr_residual_gnss",  isr_residual_gnss);
    p.add("isr_residual_ocxo",  isr_residual_ocxo);
    p.add("isr_residual_valid", isr_residual_valid);

    p.add("dispatch_shadow_ns",  dispatch_shadow_ns);
    p.add("dispatch_isr_ns",     dispatch_isr_ns);
    p.add("dispatch_delta_ns",   dispatch_delta_ns);
    p.add("pre_pps_approach_ns", pre_pps_approach_ns);
    p.add("dispatch_timeout",    (bool)dispatch_timeout);

    p.add("pps_edge_valid",         pps_edge_valid);
    p.add("pps_edge_correction_ns", pps_edge_correction_ns);
    p.add("tdc_needs_recal",        TDC_NEEDS_RECALIBRATION);

    p.add("ocxo_dac",          ocxo_dac_fractional);
    p.add("calibrate_ocxo",    calibrate_ocxo_active);
    p.add("servo_adjustments", servo_adjustments);

    p.add("diag_coarse_fires",  diag_coarse_fire_count);
    p.add("diag_fine_fires",    diag_fine_fire_count);
    p.add("diag_late",          diag_fine_late_count);
    p.add("diag_spins",         diag_spin_count);
    p.add("diag_timeouts",      diag_timeout_count);
    p.add("diag_fine_was_late", diag_fine_was_late);
    p.add("diag_spin_iters",    diag_spin_iterations);

    p.add("diag_raw_isr_cyc",    (uint32_t)isr_snap_dwt);
    p.add("diag_raw_shadow_cyc", (uint32_t)isr_captured_shadow_dwt);
    p.add("diag_isr_dwt_cycles", snap_dwt_cycles);
    p.add("diag_isr_dwt_ns",     snap_dwt_ns);

    p.add("dbg_approach_gpt2_cnt",    diag_approach_gpt2_cnt);
    p.add("dbg_approach_gnss_last32", diag_approach_gnss_last32);
    p.add("dbg_approach_gnss_t64",    diag_approach_gnss_ticks64);
    p.add("dbg_approach_gnss_ns_now", diag_approach_gnss_ns_now);
    p.add("dbg_approach_into_second", diag_approach_into_second);

    p.add("timepulse_ticks_at_pps",   tp_diag_ticks_at_pps);
    p.add("timepulse_phase_error_ns", tp_diag_phase_error_ns);
    p.add("timepulse_tick_count",     timepulse_tick_count);
    p.add("timepulse_valid",          timepulse_anchor.valid);

    publish("TIMEBASE_FRAGMENT", p);

    if (campaign_state == clocks_campaign_state_t::STARTED) {
      campaign_seconds++;
    }
  }

  pps_scheduled = false;
}

// ============================================================================
// PPS handling — Phase 1: ISR (minimum latency)
// ============================================================================

static void pps_isr(void) {

  const uint32_t snap_dwt  = DWT_CYCCNT;
  const uint32_t snap_gnss = GPT2_CNT;
  const uint32_t snap_ocxo = GPT1_CNT;

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

  // Immediate Timebase self-check at the PPS edge.
  const int64_t tb_now = timebase_now_gnss_ns();
  diag_timebase_pps_now_gnss_ns = tb_now;
  if (tb_now >= 0) {
    diag_timebase_pps_now_mod_1s_ns =
      tb_now % (int64_t)NS_PER_SECOND;
  } else {
    diag_timebase_pps_now_mod_1s_ns = -1;
  }

  if (pps_scheduled) return;
  pps_scheduled = true;

  timepop_arm(TIMEPOP_CLASS_ASAP, false, pps_asap_callback, nullptr, "pps");
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

  // Lightweight Timebase surface
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

  p.add("dispatch_shadow_ns",  dispatch_shadow_ns);
  p.add("dispatch_isr_ns",     dispatch_isr_ns);
  p.add("dispatch_delta_ns",   dispatch_delta_ns);
  p.add("pre_pps_approach_ns", pre_pps_approach_ns);
  p.add("dispatch_timeout",    (bool)dispatch_timeout);

  p.add("pps_edge_valid",         pps_edge_valid);
  p.add("pps_edge_correction_ns", pps_edge_correction_ns);
  p.add("tdc_needs_recal",        TDC_NEEDS_RECALIBRATION);

  p.add("ocxo_dac",            ocxo_dac_fractional);
  p.add("calibrate_ocxo",      calibrate_ocxo_active);
  p.add("servo_adjustments",   servo_adjustments);
  p.add("servo_step",          servo_step);
  p.add("servo_last_residual", servo_last_residual);
  p.add("servo_settle_count",  servo_settle_count);

  p.add("timepulse_ticks_at_pps",   tp_diag_ticks_at_pps);
  p.add("timepulse_phase_error_ns", tp_diag_phase_error_ns);
  p.add("timepulse_tick_count",     timepulse_tick_count);
  p.add("timepulse_valid",          timepulse_anchor.valid);

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

  // --------------------------------------------------------------------------
  // TIMEBASE FORENSICS
  // --------------------------------------------------------------------------

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

  if (tb_gnss >= 0) {
    p.add("timebase_gnss_minus_gnss_now_ns", (int64_t)tb_gnss - (int64_t)gnss_ns);
  } else {
    p.add("timebase_gnss_minus_gnss_now_ns", (int64_t)-1);
  }

  if (tb_dwt >= 0) {
    p.add("timebase_dwt_minus_dwt_now_ns", (int64_t)tb_dwt - (int64_t)dwt_ns);
  } else {
    p.add("timebase_dwt_minus_dwt_now_ns", (int64_t)-1);
  }

  if (tb_ocxo >= 0) {
    p.add("timebase_ocxo_minus_ocxo_now_ns", (int64_t)tb_ocxo - (int64_t)ocxo_ns);
  } else {
    p.add("timebase_ocxo_minus_ocxo_now_ns", (int64_t)-1);
  }

  const timebase_fragment_t* tb_frag = timebase_last_fragment();

  if (tb_frag) {
    p.add("timebase_fragment_valid",             (bool)tb_frag->valid);
    p.add("timebase_fragment_gnss_ns",           (uint64_t)tb_frag->gnss_ns);
    p.add("timebase_fragment_dwt_ns",            (uint64_t)tb_frag->dwt_ns);
    p.add("timebase_fragment_dwt_cycles",        (uint64_t)tb_frag->dwt_cycles);
    p.add("timebase_fragment_ocxo_ns",           (uint64_t)tb_frag->ocxo_ns);
    p.add("timebase_fragment_pps_count",         (uint32_t)tb_frag->pps_count);
    p.add("timebase_fragment_isr_residual_dwt",  (int32_t)tb_frag->isr_residual_dwt);
    p.add("timebase_fragment_isr_residual_gnss", (int32_t)tb_frag->isr_residual_gnss);
    p.add("timebase_fragment_isr_residual_ocxo", (int32_t)tb_frag->isr_residual_ocxo);

    if (tb_gnss >= 0 && tb_frag->valid) {
      p.add("timebase_gnss_minus_fragment_gnss_ns",
            (int64_t)tb_gnss - (int64_t)tb_frag->gnss_ns);
    } else {
      p.add("timebase_gnss_minus_fragment_gnss_ns", (int64_t)-1);
    }

    if (tb_dwt >= 0 && tb_frag->valid) {
      p.add("timebase_dwt_minus_fragment_dwt_ns",
            (int64_t)tb_dwt - (int64_t)tb_frag->dwt_ns);
    } else {
      p.add("timebase_dwt_minus_fragment_dwt_ns", (int64_t)-1);
    }

    if (tb_ocxo >= 0 && tb_frag->valid) {
      p.add("timebase_ocxo_minus_fragment_ocxo_ns",
            (int64_t)tb_ocxo - (int64_t)tb_frag->ocxo_ns);
    } else {
      p.add("timebase_ocxo_minus_fragment_ocxo_ns", (int64_t)-1);
    }
  } else {
    p.add("timebase_fragment_valid", false);
    p.add("timebase_fragment_gnss_ns", (uint64_t)0);
    p.add("timebase_fragment_dwt_ns", (uint64_t)0);
    p.add("timebase_fragment_dwt_cycles", (uint64_t)0);
    p.add("timebase_fragment_ocxo_ns", (uint64_t)0);
    p.add("timebase_fragment_pps_count", (uint32_t)0);
    p.add("timebase_fragment_isr_residual_dwt", (int32_t)0);
    p.add("timebase_fragment_isr_residual_gnss", (int32_t)0);
    p.add("timebase_fragment_isr_residual_ocxo", (int32_t)0);
    p.add("timebase_gnss_minus_fragment_gnss_ns", (int64_t)-1);
    p.add("timebase_dwt_minus_fragment_dwt_ns", (int64_t)-1);
    p.add("timebase_ocxo_minus_fragment_ocxo_ns", (int64_t)-1);
  }

  const int64_t tb_conv_gnss_to_dwt  =
    timebase_convert_ns(gnss_ns, timebase_domain_t::GNSS, timebase_domain_t::DWT);
  const int64_t tb_conv_gnss_to_ocxo =
    timebase_convert_ns(gnss_ns, timebase_domain_t::GNSS, timebase_domain_t::OCXO);
  const int64_t tb_conv_dwt_to_gnss  =
    timebase_convert_ns(dwt_ns, timebase_domain_t::DWT, timebase_domain_t::GNSS);
  const int64_t tb_conv_ocxo_to_gnss =
    timebase_convert_ns(ocxo_ns, timebase_domain_t::OCXO, timebase_domain_t::GNSS);

  p.add("timebase_conv_gnss_to_dwt_ns",   tb_conv_gnss_to_dwt);
  p.add("timebase_conv_gnss_to_ocxo_ns",  tb_conv_gnss_to_ocxo);
  p.add("timebase_conv_dwt_to_gnss_ns",   tb_conv_dwt_to_gnss);
  p.add("timebase_conv_ocxo_to_gnss_ns",  tb_conv_ocxo_to_gnss);

  if (tb_dwt >= 0 && tb_conv_gnss_to_dwt >= 0) {
    p.add("timebase_dwt_minus_conv_gnss_to_dwt_ns",
          (int64_t)tb_dwt - (int64_t)tb_conv_gnss_to_dwt);
  } else {
    p.add("timebase_dwt_minus_conv_gnss_to_dwt_ns", (int64_t)-1);
  }

  if (tb_ocxo >= 0 && tb_conv_gnss_to_ocxo >= 0) {
    p.add("timebase_ocxo_minus_conv_gnss_to_ocxo_ns",
          (int64_t)tb_ocxo - (int64_t)tb_conv_gnss_to_ocxo);
  } else {
    p.add("timebase_ocxo_minus_conv_gnss_to_ocxo_ns", (int64_t)-1);
  }

  // --------------------------------------------------------------------------
  // ISR-level Timebase self-check diagnostics
  // --------------------------------------------------------------------------

  p.add("diag_timebase_tp_now_gnss_ns",         diag_timebase_tp_now_gnss_ns);
  p.add("diag_timebase_tp_now_minus_anchor_ns", diag_timebase_tp_now_minus_anchor_ns);
  p.add("diag_timebase_tp_now_mod_tick_ns",     diag_timebase_tp_now_mod_tick_ns);

  p.add("diag_timebase_pps_now_gnss_ns",        diag_timebase_pps_now_gnss_ns);
  p.add("diag_timebase_pps_now_mod_1s_ns",      diag_timebase_pps_now_mod_1s_ns);

  p.add("diag_coarse_fires",  diag_coarse_fire_count);
  p.add("diag_fine_fires",    diag_fine_fire_count);
  p.add("diag_late",          diag_fine_late_count);
  p.add("diag_spins",         diag_spin_count);
  p.add("diag_timeouts",      diag_timeout_count);
  p.add("diag_fine_was_late", diag_fine_was_late);
  p.add("diag_spin_iters",    diag_spin_iterations);

  return p;
}
// ============================================================================
// Process registration
// ============================================================================

static const process_command_entry_t CLOCKS_COMMANDS[] = {
  { "START",      cmd_start      },
  { "STOP",       cmd_stop       },
  { "RECOVER",    cmd_recover    },
  { "REPORT",     cmd_report     },
  { "CLOCKS_INFO", cmd_clocks_info },
  { nullptr,      nullptr        }
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
// Initialization
// ============================================================================

void process_clocks_init(void) {

  timebase_init();

  analogWriteResolution(12);
  ocxo_dac_set((double)OCXO_DAC_DEFAULT);

  timepop_arm(TIMEPOP_CLASS_OCXO_DITHER, true, ocxo_dither_cb, nullptr, "ocxo-dither");

  dwt_enable();
  arm_gpt2_external();
  arm_gpt1_external();

  pinMode(GNSS_PPS_PIN,   INPUT);
  pinMode(GNSS_LOCK_PIN,  INPUT);
  pinMode(GNSS_PPS_RELAY, OUTPUT);
  digitalWriteFast(GNSS_PPS_RELAY, LOW);

  attachInterrupt(digitalPinToInterrupt(GNSS_PPS_PIN), pps_isr, RISING);
  NVIC_SET_PRIORITY(IRQ_GPIO6789, 0);

  pinMode(GNSS_10KHZ_RELAY, OUTPUT);
  digitalWriteFast(GNSS_10KHZ_RELAY, LOW);
}