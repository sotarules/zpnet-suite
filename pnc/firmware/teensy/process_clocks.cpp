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
//   A once-per-PPS diagnostic is published as TIMEPULSE_TEENSY
//   via the normal publish() path for remote monitoring.
//
//   At every PPS edge, the PPS ISR realigns GPT2_OCR1 so that
//   the 10 KHz grid remains phase-coherent with PPS.  The toggle
//   state is reset to HIGH (logically), pin forced LOW, so the
//   first post-PPS compare match produces a falling edge (no
//   TIMEPULSE action) and the second produces a rising edge —
//   the first TIMEPULSE tick of the new second, at exactly
//   +100 µs after PPS.
//
//   The quantized clock is zeroed on campaign START, restored on
//   RECOVER, and publication is gated on campaign state.  The
//   10 KHz hardware signal runs continuously from boot regardless
//   of campaign state.
//
// ============================================================================

#include "debug.h"
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
//
// At 1008 MHz: 1 cycle = 125/126 ns (exact rational)
//
// DWT_NS_NUM and DWT_NS_DEN are defined in config.h.
// These helpers centralize the conversion so the ratio appears
// in exactly one place.
//
// Overflow note:
//   cycles * 125 overflows uint64_t at 1.47 × 10^17 cycles
//   = ~4.6 years of continuous campaign time.  Safe for all
//   realistic campaign durations.
//

static inline uint64_t dwt_cycles_to_ns(uint64_t cycles) {
  return (cycles * DWT_NS_NUM) / DWT_NS_DEN;
}

// 32-bit variant for small deltas (dispatch latency, corrections).
// Safe for deltas up to ~34 million cycles (~34 ms at 1008 MHz).
static inline uint32_t dwt_cycles_to_ns_32(uint32_t cycles) {
  return (cycles * (uint32_t)DWT_NS_NUM) / (uint32_t)DWT_NS_DEN;
}

// Inverse: convert nanoseconds to DWT cycles.
// At 1008 MHz: cycles = ns * 126 / 125 (exact rational inverse).
// Used during recovery to derive the internal cycle accumulator
// from the nanosecond value supplied by the Pi.
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

// Request flags (applied at PPS boundary)
static volatile bool request_start   = false;
static volatile bool request_stop    = false;
static volatile bool request_recover = false;

// Recovery parameters — all nanoseconds (supplied by Pi).
// The Teensy derives its own internal cycle/tick counts from these
// using its known conversion ratios at the PPS boundary.
static uint64_t recover_dwt_ns  = 0;
static uint64_t recover_gnss_ns = 0;
static uint64_t recover_ocxo_ns = 0;

// Campaign-scoped PPS second counter
static uint64_t campaign_seconds = 0;

// ============================================================================
// 1. Replace the OCXO DAC state block (was lines 358–376)
// ============================================================================

static double   ocxo_dac_fractional = (double)OCXO_DAC_DEFAULT;

// Dither state — driven by TIMEPOP_CLASS_OCXO_DITHER at 1 kHz
static uint32_t dither_cycle  = 0;
static constexpr uint32_t DITHER_PERIOD = 1000;

// Calibration state (unchanged names, servo now adjusts the double)
static bool     calibrate_ocxo_active = false;
static int32_t  servo_step            = 0;       // last step applied (for diagnostics)
static double   servo_last_residual   = 0.0;     // last mean residual used for decision
static uint32_t servo_settle_count    = 0;       // PPS cycles since last DAC change
static uint32_t servo_adjustments     = 0;       // total DAC adjustments made (monotonic)

// Servo tuning constants (unchanged)
static constexpr int32_t  SERVO_MAX_STEP       = 64;   // maximum single DAC step
static constexpr uint32_t SERVO_SETTLE_SECONDS = 5;    // seconds to wait after DAC change
static constexpr uint32_t SERVO_MIN_SAMPLES    = 10;   // minimum residual samples before acting

// Set the fractional DAC value.  Clamped to valid range.
// The dither callback handles all physical DAC writes.
static void ocxo_dac_set(double value) {
  if (value < (double)OCXO_DAC_MIN) value = (double)OCXO_DAC_MIN;
  if (value > (double)OCXO_DAC_MAX) value = (double)OCXO_DAC_MAX;
  ocxo_dac_fractional = value;
}

// ============================================================================
// PPS relay to Pi — 500 ms pulse on GNSS_PPS_RELAY (pin 32)
// ============================================================================
//
// The relay pin is driven HIGH in the PPS ISR immediately after
// the three hardware snapshots.  The ISR sets relay_arm_pending
// to request that the deassert timer be armed from scheduled
// context.  The ASAP callback checks this flag and arms the
// one-shot before doing any other work.
//
// The relay fires on EVERY PPS edge, regardless of campaign state.
// chrony needs continuous PPS discipline even when no campaign
// is running.
//

static volatile bool relay_arm_pending = false;
static volatile bool relay_timer_active = false;

static void pps_relay_deassert(timepop_ctx_t*, void*) {
  digitalWriteFast(GNSS_PPS_RELAY, LOW);
  relay_timer_active = false;
}

// ============================================================================
// TIMEPULSE — 10 KHz Anchor for Clock Domain Interpolation
// ============================================================================
//
// The TIMEPULSE anchor is a volatile struct written by the 10 KHz
// GPT2 compare ISR on every rising edge, and read synchronously
// by Timebase (or any other consumer) via direct memory access.
//
// This is NOT a pub/sub publication.  On the Teensy, TIMEPULSE
// is a memory-resident fact.  A once-per-PPS diagnostic summary
// is published as "TIMEPULSE_TEENSY" for remote monitoring only.
//
// The quantized GNSS clock (timepulse_gnss_ns) increments by
// TIMEPULSE_NS_PER_TICK (100,000 ns) on each rising edge.
// It is zeroed on campaign START, restored on RECOVER, and
// gated on campaign state.
//
// The DWT snapshot (timepulse_dwt_snap) is the raw 32-bit
// DWT_CYCCNT captured at the same instant as the quantized
// clock increment.  This is not 64-bit extended — the ISR
// must remain minimal.
//

struct timepulse_anchor_t {
  volatile uint64_t gnss_ns;     // quantized GNSS nanosecond clock
  volatile uint32_t dwt_snap;    // raw DWT_CYCCNT at this rising edge
  volatile bool     valid;       // true after first rising edge in campaign
};

static timepulse_anchor_t timepulse_anchor = {0, 0, false};

// Monotonic tick count since campaign start (rising edges only)
static volatile uint32_t timepulse_tick_count = 0;

// Tick-within-second counter: 0..9999, resets at each PPS.
// At PPS boundary this should be 10000 (exactly one second of
// ticks).  Any other value indicates a phase error.
static volatile uint32_t timepulse_tick_in_second = 0;

// ============================================================================
// TIMEPULSE diagnostic counters (published once per PPS)
// ============================================================================

// PPS phase realignment count (monotonic, never reset)
static volatile uint32_t tp_diag_pps_realign_count = 0;

// Ticks-in-second at last PPS: captured in the ASAP callback
// before resetting.  Should be 10000 (one second of ticks).
static volatile uint32_t tp_diag_ticks_at_pps = 0;

// Phase error in nanoseconds: how far the quantized clock has
// drifted from the expected value at PPS.  Computed in the ASAP
// callback.  Should be zero for a perfectly locked system.
static volatile int64_t  tp_diag_phase_error_ns = 0;

// ============================================================================
// ISR-level hardware snapshots and raw residuals
// ============================================================================
//
// Written ONLY inside pps_isr(), read ONLY in the deferred callback.
// The pps_scheduled flag ensures they are consumed before the next
// PPS can overwrite them.
//

// Current PPS snapshots (raw 32-bit register reads)
static volatile uint32_t isr_snap_dwt  = 0;
static volatile uint32_t isr_snap_gnss = 0;
static volatile uint32_t isr_snap_ocxo = 0;

// Previous PPS snapshots (for ISR-level delta computation)
static volatile uint32_t isr_prev_dwt  = 0;
static volatile uint32_t isr_prev_gnss = 0;
static volatile uint32_t isr_prev_ocxo = 0;

// Raw ISR residuals: (snap - prev) - expected
// Computed entirely in the ISR, zero processing overhead.
// isr_residual_valid is false until the second PPS of a campaign.
static volatile int32_t  isr_residual_dwt  = 0;
static volatile int32_t  isr_residual_gnss = 0;
static volatile int32_t  isr_residual_ocxo = 0;
static volatile bool     isr_residual_valid = false;

// Expected 32-bit tick counts per PPS interval
// DWT @ 1008 MHz, GNSS @ 10 MHz, OCXO @ 10 MHz
static constexpr uint32_t ISR_DWT_EXPECTED  = 1008000000u;
static constexpr uint32_t ISR_GNSS_EXPECTED =   10000000u;
static constexpr uint32_t ISR_OCXO_EXPECTED =   10000000u;

// ============================================================================
// Pre-PPS dispatch latency profiling (DWT spin loop)
// ============================================================================

// Written by the spin loop continuously
static volatile uint32_t dispatch_shadow_dwt    = 0;

// Set by PPS ISR to break the spin loop
static volatile bool     pps_fired              = false;

// Captured by the PPS ISR: the shadow value at the moment the
// ISR fires.  The spin loop can clobber dispatch_shadow_dwt
// after the ISR returns (between the pps_fired check and the
// DWT write), so the ISR grabs it while we're still in ISR
// context and the loop is frozen.
static volatile uint32_t isr_captured_shadow_dwt = 0;

// Set by the spin loop on exit, cleared by the ASAP callback
static volatile bool     dispatch_shadow_valid  = false;
static volatile bool     dispatch_timeout       = false;

// Computed results, published in TIMEBASE_FRAGMENT.
// All values in 64-bit extended DWT nanoseconds.
// -1 = not yet available.
static volatile int64_t  dispatch_shadow_ns     = -1;  // last DWT ns from spin loop
static volatile int64_t  dispatch_isr_ns        = -1;  // DWT ns captured in ISR
static volatile int64_t  dispatch_delta_ns      = -1;  // ISR dispatch latency
static volatile int32_t  pre_pps_approach_ns    = -1;  // spin loop entry to PPS edge

// Corrected PPS edge — the source of truth.
// Reconstructed from shadow + TDC correction, expressed in
// the 64-bit extended DWT nanosecond clock.
static volatile int64_t  pps_edge_ns            = -1;
static volatile int32_t  pps_edge_correction_ns = -1;  // correction applied (for diagnostics)
static volatile bool     pps_edge_valid         = false;

// GNSS snapshot at spin loop entry (for approach computation)
static volatile uint32_t pre_pps_entry_gnss     = 0;

// ============================================================================
// Software TDC (Time-to-Digital Converter) correction table
// ============================================================================

static constexpr bool     TDC_NEEDS_RECALIBRATION = false;

static constexpr uint32_t TDC_FIXED_OVERHEAD   = 48;  // cycles: interrupt entry + ISR prologue
static constexpr uint32_t TDC_LOOP_CYCLES      = 1;   // total cycles per loop iteration
static constexpr uint32_t TDC_MAX_CORRECTION   = 5;   // shadow_to_edge can be 0..4, reject >=5

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

// Monotonic counters — never reset, always increment
static volatile uint32_t diag_coarse_fire_count = 0;
static volatile uint32_t diag_fine_fire_count   = 0;
static volatile uint32_t diag_fine_late_count   = 0;
static volatile uint32_t diag_spin_count        = 0;
static volatile uint32_t diag_timeout_count     = 0;

// Per-cycle flags
static volatile bool     diag_fine_was_late     = false;
static volatile uint32_t diag_spin_iterations   = 0;

// ============================================================================
// PPS residual tracking — per-clock-domain (Welford's, callback-level)
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
    const double x     = (double)r.residual;
    const double delta = x - r.mean;
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
// 64-bit Clock Extension — Unified Architecture
// ============================================================================

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
//
// GPT2 counts the GNSS 10 MHz VCLOCK at 1:1 (no prescaler).
// Output Compare 1 fires every TIMEPULSE_GNSS_HALF_PERIOD
// (500 GNSS ticks = 50 µs).
//
// The ISR toggles GNSS_10KHZ_RELAY on each invocation, producing
// a 5 KHz square wave (50% duty) — equivalently, 10,000 edges
// per second with a full period of TIMEPULSE_GNSS_FULL_PERIOD
// (1,000 GNSS ticks = 100 µs).
//
// TIMEPULSE logic executes on the RISING EDGE ONLY:
//   1) Capture DWT_CYCCNT
//   2) Increment quantized GNSS clock by TIMEPULSE_NS_PER_TICK
//   3) Write anchor struct
//
// The falling edge invocation does nothing beyond toggling the
// pin and advancing the compare register.
//
// Phase alignment with PPS: the PPS ISR resets GPT2_OCR1 and
// the toggle state on every PPS edge so that the 10 KHz grid
// is phase-coherent with the 1 Hz PPS boundary.  The PPS ISR
// has higher NVIC priority (0) than GPT2 (16), so it always
// preempts.
//
// Runs continuously from boot, independent of campaign state.
// TIMEPULSE publication (anchor writes) is campaign-gated.
//

static volatile bool timepulse_relay_state = false;

static void gpt2_compare_isr(void) {

  // Clear Output Compare 1 flag (write 1 to clear)
  GPT2_SR = GPT_SR_OF1;

  // Toggle the 10 KHz relay pin
  timepulse_relay_state = !timepulse_relay_state;
  digitalWriteFast(GNSS_10KHZ_RELAY, timepulse_relay_state ? HIGH : LOW);

  // Advance compare register by half-period.
  // This is wrap-safe: GPT2 is a 32-bit free-running counter
  // and unsigned addition handles the wrap naturally.
  GPT2_OCR1 += TIMEPULSE_GNSS_HALF_PERIOD;

  // ----------------------------------------------------------------
  // TIMEPULSE: rising edge only, campaign-gated
  //
  // On the rising edge (relay just went HIGH), capture the DWT
  // cycle count and advance the quantized GNSS clock.  This is
  // the anchor that Timebase will use for interpolation.
  //
  // The falling edge (relay just went LOW) exits here.
  // ----------------------------------------------------------------

  if (!timepulse_relay_state) return;  // falling edge — nothing to do

  if (campaign_state != clocks_campaign_state_t::STARTED) return;

  const uint32_t snap_dwt = DWT_CYCCNT;

  timepulse_anchor.gnss_ns  += TIMEPULSE_NS_PER_TICK;
  timepulse_anchor.dwt_snap  = snap_dwt;
  timepulse_anchor.valid     = true;

  timepulse_tick_count++;
  timepulse_tick_in_second++;
}

// ============================================================================
// GNSS VCLOCK (10 MHz external via GPT2 — raw, polled)
// ============================================================================

static uint64_t gnss_ticks_64 = 0;
static uint32_t gnss_last_32  = 0;

static bool gpt2_armed = false;

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
  GPT2_SR = 0x3F;                    // Clear all status flags
  GPT2_PR = 0;
  GPT2_CR = GPT_CR_CLKSRC(3) | GPT_CR_FRR;
  GPT2_CR |= GPT_CR_EN;

  gnss_last_32 = GPT2_CNT;

  // ----------------------------------------------------------
  // 10 KHz relay: configure Output Compare Channel 1
  //
  // Set initial compare value relative to current counter.
  // The ISR will advance by TIMEPULSE_GNSS_HALF_PERIOD on each match.
  // ----------------------------------------------------------

  GPT2_OCR1 = GPT2_CNT + TIMEPULSE_GNSS_HALF_PERIOD;

  // Enable Output Compare 1 interrupt
  GPT2_IR = GPT_IR_OF1IE;

  // Install GPT2 ISR and enable in NVIC.
  // Priority 16: below PPS (priority 0) but above normal.
  // This ensures PPS ISR always preempts the 10 KHz ISR,
  // which is critical for PPS phase realignment.
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

static bool gpt1_armed = false;

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
  gpt1_armed = true;
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
  clocks_dwt_cycles_now();
  clocks_gnss_ticks_now();
  clocks_ocxo_ticks_now();

  dwt_cycles_64 = 0;
  dwt_last_32   = isr_snap_dwt;

  gnss_ticks_64 = 0;
  gnss_last_32  = isr_snap_gnss;

  ocxo_ticks_64 = 0;
  ocxo_last_32  = isr_snap_ocxo;

  campaign_seconds = 0;

  residual_reset(residual_dwt);
  residual_reset(residual_gnss);
  residual_reset(residual_ocxo);

  isr_prev_dwt  = isr_snap_dwt;
  isr_prev_gnss = isr_snap_gnss;
  isr_prev_ocxo = isr_snap_ocxo;
  isr_residual_valid = false;

  dispatch_shadow_dwt      = 0;
  isr_captured_shadow_dwt  = 0;
  dispatch_shadow_valid    = false;
  dispatch_timeout         = false;
  pps_fired                = false;
  dispatch_shadow_ns       = -1;
  dispatch_isr_ns          = -1;
  dispatch_delta_ns        = -1;
  pps_edge_ns              = -1;
  pps_edge_correction_ns   = -1;
  pps_edge_valid           = false;
  pre_pps_approach_ns      = -1;
  pre_pps_entry_gnss       = 0;

  // Zero TIMEPULSE quantized clock and diagnostics.
  // The 10 KHz ISR will begin incrementing from zero on the
  // next rising edge (which occurs ~100 µs after this PPS).
  timepulse_anchor.gnss_ns  = 0;
  timepulse_anchor.dwt_snap = 0;
  timepulse_anchor.valid    = false;
  timepulse_tick_count      = 0;
  timepulse_tick_in_second  = 0;
  tp_diag_phase_error_ns    = 0;
  tp_diag_ticks_at_pps      = 0;
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
  servo_last_residual = mean_residual;

  if (fabs(mean_residual) < 0.01) return;

  double step = -mean_residual * 0.5;

  if (step > (double)SERVO_MAX_STEP) step = (double)SERVO_MAX_STEP;
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

static void pre_pps_fine_cb(timepop_ctx_t*, void*) {

  diag_fine_fire_count++;

  // Snapshot GNSS at entry for approach-time measurement
  pre_pps_entry_gnss = GPT2_CNT;

  // Check if PPS already fired before we got here.
  // If so, we arrived too late — don't enter the spin loop.
  if (pps_fired) {
    diag_fine_late_count++;
    diag_fine_was_late = true;
    diag_spin_iterations = 0;
    dispatch_shadow_valid = false;
    return;
  }

  diag_fine_was_late = false;

  diag_spin_count++;

  // --------------------------------------------------------
  // Tightest possible spin loop: ONLY capture DWT_CYCCNT.
  //
  // The loop body is: load DWT, store to volatile, load
  // pps_fired, compare, branch.  ~5 instructions, ~6 cycles,
  // ~6 ns per iteration at 1008 MHz.
  //
  // The PPS ISR preempts this loop, captures the shadow value
  // atomically via isr_captured_shadow_dwt, then sets pps_fired.
  // When the ISR returns, the loop sees pps_fired and exits.
  //
  // WARNING: No timeout.  If PPS doesn't arrive, this hangs
  // the Teensy.  Acceptable because PPS is GPS-disciplined.
  // --------------------------------------------------------

  // Block all interrupts with priority >= 32 (allows only PPS at 0)
  mask_low_priority_irqs(32);

  for (;;) {
      dispatch_shadow_dwt = DWT_CYCCNT;
      if (pps_fired) break;
  }

  // Restore normal interrupt handling
  unmask_all_irqs();

  // Normal exit: PPS ISR fired while we were spinning.
  // isr_captured_shadow_dwt holds the true pre-ISR shadow.
  dispatch_shadow_valid = true;
  dispatch_timeout = false;
  diag_spin_iterations = 0;  // not counted in tight loop
}

static void pre_pps_coarse_cb(timepop_ctx_t*, void*) {
  diag_coarse_fire_count++;
  timepop_arm(
    TIMEPOP_CLASS_PRE_PPS,
    false,
    pre_pps_fine_cb,
    nullptr,
    "pre-pps-fine"
  );
}

static void pre_pps_arm(void) {
  timepop_arm(
    TIMEPOP_CLASS_PRE_PPS_COARSE,
    false,
    pre_pps_coarse_cb,
    nullptr,
    "pre-pps-coarse"
  );
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
    request_stop = false;

    timepop_cancel(TIMEPOP_CLASS_PRE_PPS_COARSE);
    timepop_cancel(TIMEPOP_CLASS_PRE_PPS);
    dispatch_shadow_valid = false;
    pps_fired = true;

    calibrate_ocxo_active = false;
  }

  if (request_recover) {
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

    isr_residual_valid = false;

    dispatch_shadow_valid = false;
    pps_fired = true;
    dispatch_shadow_ns     = -1;
    dispatch_isr_ns        = -1;
    dispatch_delta_ns      = -1;
    pps_edge_ns            = -1;
    pps_edge_correction_ns = -1;
    pps_edge_valid         = false;
    pre_pps_approach_ns    = -1;

    // TIMEPULSE recovery: restore quantized clock.
    // Round down to nearest TIMEPULSE_NS_PER_TICK boundary.
    uint64_t recovered_tp_ns =
      (recover_gnss_ns / TIMEPULSE_NS_PER_TICK) * TIMEPULSE_NS_PER_TICK;

    timepulse_anchor.gnss_ns  = recovered_tp_ns;
    timepulse_anchor.dwt_snap = isr_snap_dwt;
    timepulse_anchor.valid    = true;
    timepulse_tick_count      = (uint32_t)(recovered_tp_ns / TIMEPULSE_NS_PER_TICK);
    timepulse_tick_in_second  = 0;
    tp_diag_phase_error_ns    = 0;
    tp_diag_ticks_at_pps      = 0;

    campaign_state = clocks_campaign_state_t::STARTED;
    request_recover = false;
  }

  if (request_start) {
    clocks_zero_all();
    campaign_state = clocks_campaign_state_t::STARTED;
    request_start = false;
  }

  if (campaign_state == clocks_campaign_state_t::STARTED) {

    if (!isr_residual_valid) {
      isr_residual_valid = true;
    }

    // --------------------------------------------------------
    // TIMEPULSE PPS diagnostics
    // --------------------------------------------------------

    tp_diag_ticks_at_pps = timepulse_tick_in_second;

    // Deliver the tick that was eaten by PPS phase realignment.
    // The PPS ISR clears the GPT2 compare flag to protect the
    // realigned OCR1 value, which prevents the 10,000th ISR
    // invocation from running.  The time genuinely elapsed, so
    // we account for it here.
    if (timepulse_anchor.valid) {
      timepulse_anchor.gnss_ns += TIMEPULSE_NS_PER_TICK;
      timepulse_anchor.dwt_snap = isr_snap_dwt;  // PPS edge DWT is the best available
      timepulse_tick_count++;
      timepulse_tick_in_second++;
    }

    timepulse_tick_in_second = 0;

    if (timepulse_anchor.valid) {
      uint64_t remainder = timepulse_anchor.gnss_ns % NS_PER_SECOND;
      if (remainder == 0) {
        tp_diag_phase_error_ns = 0;
      } else {
        tp_diag_phase_error_ns = (int64_t)remainder;
        if (remainder > NS_PER_SECOND / 2) {
          tp_diag_phase_error_ns = (int64_t)remainder - (int64_t)NS_PER_SECOND;
        }
      }
    }

    // --------------------------------------------------------
    // Pre-PPS approach time
    // --------------------------------------------------------

    if (dispatch_shadow_valid) {
      int32_t approach_ticks = (int32_t)(isr_snap_gnss - pre_pps_entry_gnss);
      pre_pps_approach_ns = approach_ticks * 100;
    } else if (dispatch_timeout) {
      pre_pps_approach_ns = -1;
      dispatch_timeout    = false;
    }

    // --------------------------------------------------------
    // Arm pre-PPS chain for the NEXT PPS edge
    // --------------------------------------------------------

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
    // Pre-PPS dispatch latency and Software TDC correction
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
        pps_dwt_cycles       = snap_dwt_cycles - overhead_cycles;
        pps_dwt_ns           = dwt_cycles_to_ns(pps_dwt_cycles);

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
    // Callback-level Welford residual tracking
    // --------------------------------------------------------

    residual_update(residual_dwt,  pps_dwt_cycles, DWT_EXPECTED_PER_PPS);
    residual_update(residual_gnss, snap_gnss_ticks, GNSS_EXPECTED_PER_PPS);
    residual_update(residual_ocxo, snap_ocxo_ticks, OCXO_EXPECTED_PER_PPS);

    // --------------------------------------------------------
    // OCXO calibration servo (if active)
    // --------------------------------------------------------

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

    p.add("isr_residual_dwt",  isr_residual_dwt);
    p.add("isr_residual_gnss", isr_residual_gnss);
    p.add("isr_residual_ocxo", isr_residual_ocxo);
    p.add("isr_residual_valid", isr_residual_valid);

    p.add("dispatch_shadow_ns",  dispatch_shadow_ns);
    p.add("dispatch_isr_ns",     dispatch_isr_ns);
    p.add("dispatch_delta_ns",   dispatch_delta_ns);
    p.add("pre_pps_approach_ns", pre_pps_approach_ns);
    p.add("dispatch_timeout",    (bool)dispatch_timeout);

    p.add("pps_edge_valid",         pps_edge_valid);
    p.add("pps_edge_correction_ns", pps_edge_correction_ns);

    p.add("tdc_needs_recal",        TDC_NEEDS_RECALIBRATION);

    p.add("ocxo_dac",              ocxo_dac_fractional);
    p.add("calibrate_ocxo",        calibrate_ocxo_active);
    p.add("servo_adjustments",     servo_adjustments);

    p.add("diag_coarse_fires",   diag_coarse_fire_count);
    p.add("diag_fine_fires",     diag_fine_fire_count);
    p.add("diag_late",           diag_fine_late_count);
    p.add("diag_spins",          diag_spin_count);
    p.add("diag_timeouts",       diag_timeout_count);
    p.add("diag_fine_was_late",  diag_fine_was_late);
    p.add("diag_spin_iters",     diag_spin_iterations);

    p.add("diag_raw_isr_cyc",    (uint32_t)isr_snap_dwt);
    p.add("diag_raw_shadow_cyc", (uint32_t)isr_captured_shadow_dwt);

    p.add("diag_isr_dwt_cycles", snap_dwt_cycles);
    p.add("diag_isr_dwt_ns",     snap_dwt_ns);

    publish("TIMEBASE_FRAGMENT", p);

    // --------------------------------------------------------
    // Publish TIMEPULSE_TEENSY — once-per-PPS diagnostic
    // --------------------------------------------------------

    {
      Payload tp;

      tp.add("gnss_ns",              timepulse_anchor.gnss_ns);
      tp.add("dwt_snap",             timepulse_anchor.dwt_snap);
      tp.add("valid",                timepulse_anchor.valid);

      tp.add("tick_count",           timepulse_tick_count);
      tp.add("ticks_at_pps",         tp_diag_ticks_at_pps);

      tp.add("phase_error_ns",       tp_diag_phase_error_ns);
      tp.add("pps_realign_count",    tp_diag_pps_realign_count);

      tp.add("campaign",             campaign_name);
      tp.add("campaign_seconds",     campaign_seconds);

      publish("TIMEPULSE_TEENSY", tp);
    }

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
  pps_fired = true;

  digitalWriteFast(GNSS_PPS_RELAY, HIGH);
  relay_arm_pending = true;

  // ============================================================
  // TIMEPULSE phase realignment.
  //
  // Reset the GPT2 output compare register so the 10 KHz grid
  // is anchored to this PPS edge.
  //
  // Toggle state logic:
  //   timepulse_relay_state = true  (as if pin was HIGH)
  //   digitalWriteFast(LOW)         (force pin LOW now)
  //   First match at +500 ticks:    toggle → false, pin LOW
  //     ISR sees false → returns (falling edge, no TIMEPULSE)
  //   Second match at +1000 ticks:  toggle → true, pin HIGH
  //     ISR sees true → TIMEPULSE fires (first tick of second)
  //
  // This puts the first TIMEPULSE tick at exactly +1000 GNSS
  // ticks (100 µs) after the PPS edge.
  //
  // Clear the output compare flag to prevent a stale pending
  // match from firing immediately.
  // ============================================================

  GPT2_OCR1 = snap_gnss + TIMEPULSE_GNSS_HALF_PERIOD;
  GPT2_SR = GPT_SR_OF1;  // clear any pending compare match
  timepulse_relay_state = true;
  digitalWriteFast(GNSS_10KHZ_RELAY, LOW);

  tp_diag_pps_realign_count++;

  // ============================================================
  // Raw ISR-level residuals
  // ============================================================

  if (isr_residual_valid) {
    isr_residual_dwt  = (int32_t)(snap_dwt  - isr_prev_dwt)  - (int32_t)ISR_DWT_EXPECTED;
    isr_residual_gnss = (int32_t)(snap_gnss - isr_prev_gnss) - (int32_t)ISR_GNSS_EXPECTED;
    isr_residual_ocxo = (int32_t)(snap_ocxo - isr_prev_ocxo) - (int32_t)ISR_OCXO_EXPECTED;
  }

  isr_prev_dwt  = snap_dwt;
  isr_prev_gnss = snap_gnss;
  isr_prev_ocxo = snap_ocxo;

  isr_snap_dwt  = snap_dwt;
  isr_snap_gnss = snap_gnss;
  isr_snap_ocxo = snap_ocxo;

  // ============================================================
  // Schedule ASAP callback
  // ============================================================

  if (pps_scheduled) return;
  pps_scheduled = true;

  timepop_arm(
    TIMEPOP_CLASS_ASAP,
    false,
    pps_asap_callback,
    nullptr,
    "pps"
  );
}

// ============================================================================
// Dither callback — armed in process_clocks_init(), runs forever
// ============================================================================

static void ocxo_dither_cb(timepop_ctx_t*, void*) {
  uint32_t base = (uint32_t)ocxo_dac_fractional;
  double frac = ocxo_dac_fractional - (double)base;
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
  if (args.tryGetDouble("set_dac", dac_val)) {
    ocxo_dac_set(dac_val);
  }

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
  p.add("status", "start_requested");
  p.add("ocxo_dac", ocxo_dac_fractional);
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

  recover_dwt_ns  = strtoull(s_dwt_ns, nullptr, 10);
  recover_gnss_ns = strtoull(s_gnss,   nullptr, 10);
  recover_ocxo_ns = strtoull(s_ocxo,   nullptr, 10);

  double dac_val;
  if (args.tryGetDouble("set_dac", dac_val)) {
    ocxo_dac_set(dac_val);
  }

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
// Helper — emit residual stats for one clock domain into a Payload
// ============================================================================

static void report_residual(Payload& p,
                            const char* prefix,
                            const pps_residual_t& r) {

  char key[48];

  snprintf(key, sizeof(key), "%s_pps_valid", prefix);
  p.add(key, r.valid);

  snprintf(key, sizeof(key), "%s_pps_delta", prefix);
  p.add(key, r.delta);

  snprintf(key, sizeof(key), "%s_pps_residual", prefix);
  p.add(key, r.residual);

  snprintf(key, sizeof(key), "%s_pps_n", prefix);
  p.add(key, r.n);

  snprintf(key, sizeof(key), "%s_pps_mean", prefix);
  p.add_fmt(key, "%.3f", r.mean);

  snprintf(key, sizeof(key), "%s_pps_stddev", prefix);
  p.add_fmt(key, "%.3f", residual_stddev(r));

  snprintf(key, sizeof(key), "%s_pps_stderr", prefix);
  p.add_fmt(key, "%.3f", residual_stderr(r));
}

// ============================================================================
// REPORT — diagnostic introspection only
// ============================================================================

static Payload cmd_report(const Payload&) {

  Payload p;

  p.add(
    "campaign_state",
    campaign_state == clocks_campaign_state_t::STARTED
      ? "STARTED"
      : "STOPPED"
  );

  if (campaign_state == clocks_campaign_state_t::STARTED) {
    p.add("campaign", campaign_name);
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

  p.add("gnss_lock", digitalRead(GNSS_LOCK_PIN));

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

  p.add("ocxo_dac",              ocxo_dac_fractional);
  p.add("calibrate_ocxo",        calibrate_ocxo_active);
  p.add("servo_adjustments",     servo_adjustments);
  p.add("servo_step",             servo_step);
  p.add("servo_last_residual",    servo_last_residual);
  p.add("servo_settle_count",     servo_settle_count);

  p.add("diag_coarse_fires",   diag_coarse_fire_count);
  p.add("diag_fine_fires",     diag_fine_fire_count);
  p.add("diag_late",           diag_fine_late_count);
  p.add("diag_spins",          diag_spin_count);
  p.add("diag_timeouts",       diag_timeout_count);
  p.add("diag_fine_was_late",  diag_fine_was_late);
  p.add("diag_spin_iters",     diag_spin_iterations);

  // TIMEPULSE diagnostics
  p.add("tp_gnss_ns",            timepulse_anchor.gnss_ns);
  p.add("tp_dwt_snap",           timepulse_anchor.dwt_snap);
  p.add("tp_valid",              timepulse_anchor.valid);
  p.add("tp_tick_count",         timepulse_tick_count);
  p.add("tp_ticks_at_pps",       tp_diag_ticks_at_pps);
  p.add("tp_phase_error_ns",     tp_diag_phase_error_ns);
  p.add("tp_pps_realign_count",  tp_diag_pps_realign_count);

  if (campaign_state == clocks_campaign_state_t::STARTED && gnss_ns > 0) {

    const double tau_dwt  = (double)dwt_ns  / (double)gnss_ns;
    const double tau_ocxo = (double)ocxo_ns / (double)gnss_ns;

    p.add_fmt("tau_dwt",  "%.12f", tau_dwt);
    p.add_fmt("tau_ocxo", "%.12f", tau_ocxo);

    const double ppb_dwt =
      ((double)((int64_t)dwt_ns - (int64_t)gnss_ns) / (double)gnss_ns) * 1e9;

    const double ppb_ocxo =
      ((double)((int64_t)ocxo_ns - (int64_t)gnss_ns) / (double)gnss_ns) * 1e9;

    p.add_fmt("dwt_ppb",  "%.3f", ppb_dwt);
    p.add_fmt("ocxo_ppb", "%.3f", ppb_ocxo);

  } else {
    p.add("tau_dwt",  0.0);
    p.add("tau_ocxo", 0.0);
    p.add("dwt_ppb",  0.0);
    p.add("ocxo_ppb", 0.0);
  }

  return p;
}


// ============================================================================
// Process registration
// ============================================================================

static const process_command_entry_t CLOCKS_COMMANDS[] = {
  { "START",   cmd_start   },
  { "STOP",    cmd_stop    },
  { "RECOVER", cmd_recover },
  { "REPORT",  cmd_report  },
  { nullptr,   nullptr     }
};

static const process_vtable_t CLOCKS_PROCESS = {
  .process_id    = "CLOCKS",
  .commands      = CLOCKS_COMMANDS,
  .subscriptions = nullptr
};

void process_clocks_register(void) {
  process_register("CLOCKS", &CLOCKS_PROCESS);
}

// ============================================================================
// Initialization
// ============================================================================

void process_clocks_init(void) {

  analogWriteResolution(12);
  ocxo_dac_set((double)OCXO_DAC_DEFAULT);

  timepop_arm(
    TIMEPOP_CLASS_OCXO_DITHER,
    true,
    ocxo_dither_cb,
    nullptr,
    "ocxo-dither"
  );

  dwt_enable();
  arm_gpt2_external();    // GNSS VCLOCK -> GPT2 pin 14, raw 10 MHz, polled
  arm_gpt1_external();    // OCXO 10 MHz -> GPT1 pin 25, raw 10 MHz, polled

  pinMode(GNSS_PPS_PIN, INPUT);
  pinMode(GNSS_LOCK_PIN, INPUT);

  pinMode(GNSS_PPS_RELAY, OUTPUT);
  digitalWriteFast(GNSS_PPS_RELAY, LOW);

  attachInterrupt(
    digitalPinToInterrupt(GNSS_PPS_PIN),
    pps_isr,
    RISING
  );

  NVIC_SET_PRIORITY(IRQ_GPIO6789, 0);  // PPS from GF-8802

  // 10 KHz GNSS relay output to Pi (pin 9 -> Pi GPIO 25)
  // Driven by GPT2 Output Compare ISR at 10 KHz toggle rate.
  // TIMEPULSE anchor writes are campaign-gated inside the ISR.
  pinMode(GNSS_10KHZ_RELAY, OUTPUT);
  digitalWriteFast(GNSS_10KHZ_RELAY, LOW);
}