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
// OCXO DAC Control (Teensy pin 22 / A22 → OCXO CTL pin)
// ============================================================================
//
// The DAC is initialized at boot and always active.
// The current DAC value is published in every TIMEBASE_FRAGMENT.
//
// Calibration mode: when calibrate_ocxo_active is true, the PPS
// callback runs a perpetual servo loop that adjusts the DAC to
// drive the OCXO residual toward zero (tau → 1.0).
//
// The servo NEVER converges.  It continuously tracks the OCXO
// residual and adjusts the DAC as needed.  This compensates for
// thermal drift, supply voltage changes, and aging over the
// lifetime of a campaign.
//
// Algorithm (proportional with clamping):
//
//   1) Wait for SERVO_MIN_SAMPLES of fresh Welford data
//   2) Wait SERVO_SETTLE_SECONDS after the last DAC change
//   3) Read the current mean residual (ticks per PPS)
//   4) If mean is zero, do nothing (but keep watching)
//   5) Compute step = -mean_residual (1:1 conservative ratio)
//   6) Clamp step to ±SERVO_MAX_STEP
//   7) Apply step, reset Welford, reset settle counter
//
// The 1:1 ratio is conservative (actual sensitivity is ~1.6
// ticks per DAC count).  This prevents overshoot and lets the
// servo approach zero asymptotically.
//
// The servo resets Welford stats after each DAC change so only
// post-change data informs the next decision.
//

static uint32_t ocxo_dac_value = OCXO_DAC_DEFAULT;

// Calibration state
static bool     calibrate_ocxo_active = false;
static int32_t  servo_step            = 0;       // last step applied (for diagnostics)
static int32_t  servo_last_residual   = 0;       // last mean residual used for decision
static uint32_t servo_settle_count    = 0;       // PPS cycles since last DAC change
static uint32_t servo_adjustments     = 0;       // total DAC adjustments made (monotonic)

// Servo tuning constants
static constexpr int32_t  SERVO_MAX_STEP       = 64;   // maximum single DAC step
static constexpr uint32_t SERVO_SETTLE_SECONDS = 5;    // seconds to wait after DAC change
static constexpr uint32_t SERVO_MIN_SAMPLES    = 10;   // minimum residual samples before acting

// Helper: write DAC value, clamped to valid range
static void ocxo_dac_write(uint32_t value) {
  if (value > OCXO_DAC_MAX) value = OCXO_DAC_MAX;
  ocxo_dac_value = value;
  analogWrite(OCXO_CTL_PIN, ocxo_dac_value);
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
//
// The fine-stage callback enters a tight spin loop that
// continuously copies DWT_CYCCNT to a volatile shadow variable.
// When the PPS ISR fires, it sets pps_fired = true, breaking
// the loop.  The difference between the ISR's DWT snapshot and
// the shadow value is the ISR dispatch latency.
//
// The spin loop has a defensive 500 ms timeout (504,000,000 DWT
// cycles at 1008 MHz).  If the PPS doesn't arrive in time, the
// loop exits and flags the failure.
//
// Results are expressed in nanoseconds using DWT's 125/126 ns/cycle
// conversion, and published in TIMEBASE_FRAGMENT.
//

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
//
// The spin loop is a tight instruction sequence at 1008 MHz.
//
// *** IMPORTANT: These constants were derived at 600 MHz. ***
// *** They MUST be re-derived from disassembly at 1008 MHz. ***
//
// At 1008 MHz the pipeline timing changes:
//   • Instruction throughput increases (more cycles per second)
//   • But the instruction SEQUENCE is identical (same binary)
//   • The loop is still the same 5 instructions
//   • Interrupt entry overhead changes (may be fewer/more cycles
//     due to flash wait states and bus arbitration at new speed)
//
// To re-derive:
//   1. Compile at 1008 MHz
//   2. Disassemble the spin loop (arm-none-eabi-objdump -d)
//   3. Verify instruction sequence is unchanged
//   4. Measure delta_cycles empirically over many PPS edges
//   5. The histogram of delta_cycles will cluster at N values
//      separated by the loop cycle count
//   6. TDC_FIXED_OVERHEAD = minimum observed delta_cycles
//   7. TDC_LOOP_CYCLES = spacing between clusters
//
// Until re-derivation, TDC correction is DISABLED by setting
// TDC_NEEDS_RECALIBRATION = true.  The ISR snapshot is used
// as fallback (same behavior as when dispatch_shadow_valid
// is false).
//

static constexpr bool     TDC_NEEDS_RECALIBRATION = false;

static constexpr uint32_t TDC_FIXED_OVERHEAD   = 48;  // cycles: interrupt entry + ISR prologue
static constexpr uint32_t TDC_LOOP_CYCLES      = 1;   // total cycles per loop iteration
static constexpr uint32_t TDC_MAX_CORRECTION   = 5;   // shadow_to_edge can be 0..4, reject >=5

// Convert delta_cycles to shadow-to-edge correction in cycles.
// Returns the correction, or -1 if delta_cycles is out of range
// or TDC needs recalibration.
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
//
// These counters and flags help diagnose why some PPS cycles
// show stale shadow values (~1.43s delta) instead of the
// expected ~80-90 ns dispatch latency.
//

// Monotonic counters — never reset, always increment
static volatile uint32_t diag_coarse_fire_count = 0;  // coarse callback invocations
static volatile uint32_t diag_fine_fire_count   = 0;  // fine callback invocations
static volatile uint32_t diag_fine_late_count   = 0;  // fine arrived with pps_fired=true (too late)
static volatile uint32_t diag_spin_count        = 0;  // times we entered the spin loop
static volatile uint32_t diag_timeout_count     = 0;  // spin loop timeouts

// Per-cycle flags — set by callbacks, consumed by ASAP publisher
// These describe what happened on THIS PPS cycle.
static volatile bool     diag_fine_was_late     = false;  // pps_fired was true at fine entry
static volatile uint32_t diag_spin_iterations   = 0;      // how many loop iterations we did

// ============================================================================
// PPS residual tracking — per-clock-domain (Welford's, callback-level)
// ============================================================================
//
// These use the 64-bit extended values from the callback for running
// statistics.  The ISR residuals above are the raw, unprocessed
// single-sample values.
//

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
//
// Each clock domain has exactly ONE state pair: {ticks_64, last_32}.
// Exactly ONE function mutates this state: clocks_xxx_now().
//
// clocks_xxx_now() reads the LIVE hardware register, computes the
// unsigned 32-bit delta from last_32, accumulates into ticks_64,
// and updates last_32.  This is safe to call from ANY context
// (ISR, callback, command handler, diagnostic) at ANY time, with
// no ordering constraints.
//
// The only requirement is that it be called at least once per
// 2^31 ticks of the hardware counter (to avoid wrap ambiguity):
//   DWT  @ 1008 MHz: every ~2.13 seconds
//   GNSS @   10 MHz: every ~214 seconds
//   OCXO @   10 MHz: every ~214 seconds
//
// The 1-second PPS callback satisfies this with margin.
//

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
  GPT2_SR = 0x3F;
  GPT2_PR = 0;
  GPT2_CR = GPT_CR_CLKSRC(3) | GPT_CR_FRR;
  GPT2_CR |= GPT_CR_EN;

  gnss_last_32 = GPT2_CNT;
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
//
// Each of these calls the domain's _now() to advance the accumulator
// to the present, then rewinds by the small delta between the ISR
// snapshot and the current register value.
//
// This is the ONLY way to obtain the 64-bit value at the PPS edge.
// The _now() call is always safe and always moves forward.
// The rewind is always a small positive number (< 1 second of ticks).
//

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
//
// Zeroing must leave last_32 pointing at the ISR snapshot so that
// the next _now() call correctly computes the delta from the PPS
// edge.  The 64-bit accumulator starts at 0 — meaning "zero ticks
// have elapsed since campaign start."
//

static void clocks_zero_all(void) {
  // Advance accumulators to present first (safe, monotonic)
  clocks_dwt_cycles_now();
  clocks_gnss_ticks_now();
  clocks_ocxo_ticks_now();

  // Zero the accumulators, anchor last_32 at the ISR snapshot
  dwt_cycles_64 = 0;
  dwt_last_32   = isr_snap_dwt;

  gnss_ticks_64 = 0;
  gnss_last_32  = isr_snap_gnss;

  ocxo_ticks_64 = 0;
  ocxo_last_32  = isr_snap_ocxo;

  campaign_seconds = 0;

  // Reset callback-level residual tracking
  residual_reset(residual_dwt);
  residual_reset(residual_gnss);
  residual_reset(residual_ocxo);

  // Reset ISR-level residual tracking
  isr_prev_dwt  = isr_snap_dwt;
  isr_prev_gnss = isr_snap_gnss;
  isr_prev_ocxo = isr_snap_ocxo;
  isr_residual_valid = false;

  // Reset pre-PPS dispatch profiling
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
}

// ============================================================================
// OCXO Calibration Servo (Perpetual Proportional Tracker)
// ============================================================================
//
// Called once per PPS from the ASAP callback, only when
// calibrate_ocxo_active is true and campaign is STARTED.
//
// Uses the callback-level OCXO residual (Welford) because it provides
// a running mean that filters single-sample noise.
//
// The servo NEVER stops.  It continuously evaluates the mean
// residual and adjusts the DAC to drive it toward zero.  When
// the mean is already zero, it does nothing but keeps watching.
// If the OCXO drifts (thermal, supply, aging), the servo
// corrects on the next evaluation cycle.
//
// Pull slope is positive: higher voltage = higher frequency.
// So: negative mean (slow) → increase DAC, positive mean (fast) → decrease.
//

static void ocxo_calibration_servo(void) {

  if (!calibrate_ocxo_active) return;

  // Wait for enough samples to have a meaningful mean
  if (residual_ocxo.n < SERVO_MIN_SAMPLES) return;

  // Wait for the OCXO to settle after the last DAC change
  servo_settle_count++;
  if (servo_settle_count < SERVO_SETTLE_SECONDS) return;

  // Read the current mean residual (ticks per PPS interval).
  // This is a double — critical for sub-tick resolution.
  // At 10 MHz, 1 tick = 100 ns, so a mean of -0.966 is -96.6 ppb.
  // Truncating to int32_t would lose this entirely.
  // Negative = OCXO running slow, positive = running fast.
  double mean_residual = residual_ocxo.mean;

  // Record for diagnostics (truncated, but the real decision uses the double)
  servo_last_residual = (int32_t)mean_residual;

  // Dead band: if |mean| < 0.1 ticks (10 ppb at 10 MHz), don't adjust.
  // The OCXO is close enough — avoid chasing noise.
  // But keep watching: don't reset Welford, don't reset settle.
  // The mean will continue to refine with more samples.
  if (fabs(mean_residual) < 0.1) return;

  // Proportional step: estimate DAC change from current residual.
  // Empirical sensitivity: ~1.6 ticks per DAC count at 10 MHz.
  // Use conservative 1:1 ratio to avoid overshoot.
  // Round away from zero so sub-tick drift always produces a step.
  // Negative residual (slow) → positive step (increase voltage).
  // Positive residual (fast) → negative step (decrease voltage).
  int32_t step;
  if (mean_residual < 0.0) {
    step = (int32_t)((-mean_residual) + 0.5);
    if (step < 1) step = 1;
  } else {
    step = -(int32_t)(mean_residual + 0.5);
    if (step > -1) step = -1;
  }

  // Clamp to SERVO_MAX_STEP to bound the maximum single move
  if (step > SERVO_MAX_STEP) step = SERVO_MAX_STEP;
  if (step < -SERVO_MAX_STEP) step = -SERVO_MAX_STEP;

  // Apply the step
  int32_t new_dac = (int32_t)ocxo_dac_value + step;
  if (new_dac < (int32_t)OCXO_DAC_MIN) new_dac = (int32_t)OCXO_DAC_MIN;
  if (new_dac > (int32_t)OCXO_DAC_MAX) new_dac = (int32_t)OCXO_DAC_MAX;

  ocxo_dac_write((uint32_t)new_dac);

  // Record for diagnostics
  servo_step = step;
  servo_adjustments++;

  // Reset settle counter — wait for OCXO to respond
  servo_settle_count = 0;

  // Reset Welford stats so the next decision uses only post-change data
  residual_reset(residual_ocxo);
}

// ============================================================================
// Pre-PPS dispatch latency profiling — two-stage spin loop
// ============================================================================
//
// Two-stage timer chain landing in a tight DWT spin loop:
//
//   Stage 1 (coarse, ms, PIT0):  998 ms after PPS.
//     Dispatched ~998.3 ms.  Arms stage 2.
//
//   Stage 2 (fine, µs, PIT1):    999 µs after coarse dispatch.
//     Fires ~999.3 ms.  Dispatched ~999.6 ms.
//     Enters the spin loop.
//
//   Spin loop: ~400 µs until PPS at 1000 ms.
//     Continuously writes DWT_CYCCNT to dispatch_shadow_dwt.
//     PPS ISR captures the shadow, sets pps_fired, loop exits.
//
// The TDC correction table (see above) converts the measured
// delta into the true PPS edge DWT value.
//
// CPU cost: ~400 µs per second = ~0.04%.
//

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
  // Stage 1 complete.  Arm stage 2 (µs precision).
  timepop_arm(
    TIMEPOP_CLASS_PRE_PPS,
    false,                    // one-shot
    pre_pps_fine_cb,
    nullptr,
    "pre-pps-fine"
  );
}

// Helper: arm the coarse stage (called from PPS ASAP callback)
static void pre_pps_arm(void) {
  timepop_arm(
    TIMEPOP_CLASS_PRE_PPS_COARSE,
    false,                    // one-shot
    pre_pps_coarse_cb,
    nullptr,
    "pre-pps-coarse"
  );
}

// ============================================================================
// PPS handling — Phase 2: ASAP callback (scheduled context)
// ============================================================================
//
// This is the deferred callback invoked by the PPS ISR via timepop.
// It runs in scheduled context (not ISR), so timepop_arm is safe.
//
// Responsibilities:
//   1. Arm the PPS relay deassert timer (always, regardless of state)
//   2. Process campaign state transitions (start/stop/recover)
//   3. Recover 64-bit values at the PPS edge via advance-then-rewind
//   4. Compute dispatch latency and Software TDC correction
//   5. Update Welford residual statistics
//   6. Run OCXO calibration servo
//   7. Build and publish TIMEBASE_FRAGMENT
//

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

    // Cancel any pending pre-PPS timers
    timepop_cancel(TIMEPOP_CLASS_PRE_PPS_COARSE);
    timepop_cancel(TIMEPOP_CLASS_PRE_PPS);
    dispatch_shadow_valid = false;
    pps_fired = true;  // break spin loop if it's running

    // Stop calibration servo (DAC value is preserved)
    calibrate_ocxo_active = false;
  }

  if (request_recover) {
    // --------------------------------------------------------
    // Recovery: load nanosecond values into accumulators.
    //
    // The Pi supplies dwt_ns, gnss_ns, ocxo_ns — all nanoseconds.
    // The Teensy derives internal cycle/tick counts:
    //   DWT:  cycles = ns * 126 / 125  (dwt_ns_to_cycles)
    //   GNSS: ticks  = ns / 100
    //   OCXO: ticks  = ns / 100
    //
    // Anchor last_32 at the ISR snapshot so the next _now()
    // call correctly accumulates from the PPS edge forward.
    // --------------------------------------------------------

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

    // ISR residuals will become valid on the next PPS
    isr_residual_valid = false;

    // Reset pre-PPS dispatch profiling
    dispatch_shadow_valid = false;
    pps_fired = true;  // break spin loop if running
    dispatch_shadow_ns     = -1;
    dispatch_isr_ns        = -1;
    dispatch_delta_ns      = -1;
    pps_edge_ns            = -1;
    pps_edge_correction_ns = -1;
    pps_edge_valid         = false;
    pre_pps_approach_ns    = -1;

    campaign_state = clocks_campaign_state_t::STARTED;
    request_recover = false;
  }

  if (request_start) {
    clocks_zero_all();
    campaign_state = clocks_campaign_state_t::STARTED;
    request_start = false;
  }

  if (campaign_state == clocks_campaign_state_t::STARTED) {

    // Mark ISR residuals as valid for the NEXT PPS
    // (the first PPS after start has no prior snapshot
    // to delta against — this flag was cleared by
    // clocks_zero_all or recover, so the ISR won't
    // compute residuals until the second PPS)
    if (!isr_residual_valid) {
      isr_residual_valid = true;
    }

    // --------------------------------------------------------
    // Pre-PPS approach time (computed before 64-bit extension
    // because it uses raw 32-bit GNSS snapshots only)
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

    // Clear pps_fired so the spin callback doesn't see the
    // stale flag from THIS PPS.  The spin callback will set
    // it to false again at entry, but if it checks before
    // clearing, it would see true and bail out as "late".
    pps_fired = false;

    // --------------------------------------------------------
    // Recover 64-bit values at the PPS edge
    //
    // Uses advance-then-rewind: each _at_pps_edge() call
    // first advances the accumulator to the present via
    // _now(), then rewinds by the small delta between the
    // live register and the ISR snapshot.
    //
    // This is safe regardless of any intervening _now()
    // calls (e.g. from cmd_report) because _now() only
    // moves forward, and the rewind is always the correct
    // small positive offset from isr_snap to last_32.
    // --------------------------------------------------------

    const uint64_t snap_dwt_cycles = dwt_at_pps_edge();
    const uint64_t snap_gnss_ticks = gnss_at_pps_edge();
    const uint64_t snap_ocxo_ticks = ocxo_at_pps_edge();

    const uint64_t snap_dwt_ns  = dwt_cycles_to_ns(snap_dwt_cycles);
    const uint64_t snap_gnss_ns = snap_gnss_ticks * 100ull;
    const uint64_t snap_ocxo_ns = snap_ocxo_ticks * 100ull;

    // --------------------------------------------------------
    // Pre-PPS dispatch latency and Software TDC correction.
    //
    // The spin loop + ISR capture gives us two DWT values:
    //   isr_snap_dwt           — DWT_CYCCNT read in ISR
    //   isr_captured_shadow_dwt — last shadow before ISR
    //
    // delta_cycles = isr - shadow tells us which instruction
    // was executing when the interrupt fired.  Subtracting
    // the fixed overhead gives the correction: cycles between
    // shadow write and true PPS edge.
    //
    // The corrected PPS edge becomes the definitive DWT
    // value for this PPS.  All downstream calculations
    // (residuals, tau, published dwt_cycles/dwt_ns) use it.
    //
    // NOTE: TDC correction is DISABLED until the correction
    // table is re-derived at 1008 MHz.  The ISR snapshot
    // is used as fallback.  Dispatch latency diagnostics
    // are still computed and published for analysis.
    // --------------------------------------------------------

    // Start with ISR values as fallback
    uint64_t pps_dwt_cycles = snap_dwt_cycles;
    uint64_t pps_dwt_ns     = snap_dwt_ns;

    if (dispatch_shadow_valid) {
      uint32_t delta_cycles = isr_snap_dwt - isr_captured_shadow_dwt;
      int64_t  delta_ns     = (int64_t)dwt_cycles_to_ns_32(delta_cycles);

      // Dispatch profiling values (diagnostic)
      dispatch_isr_ns    = (int64_t)snap_dwt_ns;
      dispatch_delta_ns  = delta_ns;
      dispatch_shadow_ns = (int64_t)snap_dwt_ns - delta_ns;

      int32_t correction = tdc_correction_cycles(delta_cycles);
      if (correction >= 0) {
        // Valid correction: reconstruct the true PPS edge.
        //
        // The snap_dwt_cycles is the 64-bit value at the ISR
        // snapshot.  The correction tells us the ISR was
        // (overhead) cycles after the true PPS edge:
        uint32_t overhead_cycles = delta_cycles - (uint32_t)correction;
        pps_dwt_cycles       = snap_dwt_cycles - overhead_cycles;
        pps_dwt_ns           = dwt_cycles_to_ns(pps_dwt_cycles);

        pps_edge_ns            = (int64_t)pps_dwt_ns;
        pps_edge_correction_ns = (int32_t)dwt_cycles_to_ns_32(correction);
        pps_edge_valid         = true;
      } else {
        // delta_cycles out of expected range (or TDC needs
        // recalibration) — use ISR value
        pps_edge_ns            = -1;
        pps_edge_correction_ns = -1;
        pps_edge_valid         = false;
      }

      dispatch_shadow_valid = false;
    } else {
      // Spin loop didn't run this cycle (late, timeout, or not armed).
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
    //
    // Uses the TDC-corrected DWT cycles when available.
    // GNSS and OCXO are unaffected (no dispatch correction).
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

    // Definitive DWT state at PPS edge (TDC-corrected when valid,
    // ISR snapshot as fallback).  All downstream consumers use these.
    p.add("dwt_cycles",       pps_dwt_cycles);
    p.add("dwt_ns",           pps_dwt_ns);
    p.add("gnss_ns",          snap_gnss_ns);
    p.add("ocxo_ns",          snap_ocxo_ns);
    p.add("teensy_pps_count", campaign_seconds);
    p.add("gnss_lock",        digitalRead(GNSS_LOCK_PIN));

    // Callback-level residuals (from 64-bit extended values)
    p.add("dwt_pps_residual",  residual_dwt.residual);
    p.add("gnss_pps_residual", residual_gnss.residual);
    p.add("ocxo_pps_residual", residual_ocxo.residual);

    // ISR-level raw residuals (from 32-bit register snapshots)
    // These are the purest measurement — computed in the ISR
    // with zero processing between PPS edge and subtraction.
    p.add("isr_residual_dwt",  isr_residual_dwt);
    p.add("isr_residual_gnss", isr_residual_gnss);
    p.add("isr_residual_ocxo", isr_residual_ocxo);
    p.add("isr_residual_valid", isr_residual_valid);

    // Pre-PPS dispatch latency profiling (nanoseconds).
    // All three values use the 64-bit extended DWT clock.
    // shadow = last spin loop DWT ns before ISR fired
    // isr = DWT ns captured as first ISR instruction
    // delta = isr - shadow = true dispatch latency
    // approach = GNSS ns from spin loop entry to PPS edge
    // -1 = not yet available or timeout.
    p.add("dispatch_shadow_ns",  dispatch_shadow_ns);
    p.add("dispatch_isr_ns",     dispatch_isr_ns);
    p.add("dispatch_delta_ns",   dispatch_delta_ns);
    p.add("pre_pps_approach_ns", pre_pps_approach_ns);
    p.add("dispatch_timeout",    (bool)dispatch_timeout);

    // Software TDC correction diagnostics.
    // When pps_edge_valid is true, dwt_cycles and dwt_ns above
    // are the TDC-corrected values (source of truth).
    // When false, they fall back to the ISR snapshot.
    p.add("pps_edge_valid",         pps_edge_valid);
    p.add("pps_edge_correction_ns", pps_edge_correction_ns);

    // TDC recalibration flag — true means TDC constants are
    // stale (from 600 MHz) and correction is disabled.
    p.add("tdc_needs_recal",        TDC_NEEDS_RECALIBRATION);

    // OCXO DAC control state (always present, unconditional)
    p.add("ocxo_dac",              ocxo_dac_value);
    p.add("calibrate_ocxo",        calibrate_ocxo_active);
    p.add("servo_adjustments",     servo_adjustments);

    // Dispatch profiling diagnostics
    p.add("diag_coarse_fires",   diag_coarse_fire_count);
    p.add("diag_fine_fires",     diag_fine_fire_count);
    p.add("diag_late",           diag_fine_late_count);
    p.add("diag_spins",          diag_spin_count);
    p.add("diag_timeouts",       diag_timeout_count);
    p.add("diag_fine_was_late",  diag_fine_was_late);
    p.add("diag_spin_iters",     diag_spin_iterations);

    // Raw 32-bit DWT cycle values for debugging.
    p.add("diag_raw_isr_cyc",    (uint32_t)isr_snap_dwt);
    p.add("diag_raw_shadow_cyc", (uint32_t)isr_captured_shadow_dwt);

    // Raw ISR snapshot (64-bit extended, before TDC correction).
    // These are what dwt_cycles/dwt_ns used to be before the TDC.
    p.add("diag_isr_dwt_cycles", snap_dwt_cycles);
    p.add("diag_isr_dwt_ns",     snap_dwt_ns);

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
//
// Two-phase architecture:
//
//   Phase 1 (ISR): Snapshot all hardware counters, compute raw
//   PPS-to-PPS residuals, and assert PPS relay HIGH — all
//   immediately at the PPS rising edge.  Three register reads
//   (~6 ns total at 1008 MHz), three subtractions, one
//   digitalWriteFast.  Set relay_arm_pending flag for deferred
//   timer arming.
//   NO timepop_arm calls from ISR context.
//
//   Phase 2 (timepop ASAP callback): pps_asap_callback() above.
//

static void pps_isr(void) {

  // ============================================================
  // PHASE 1: Hardware snapshot — absolute minimum latency
  //
  // Three register reads within ~6 ns of each other at 1008 MHz.
  // This is as close to the PPS edge as software can get.
  // ============================================================

  const uint32_t snap_dwt  = DWT_CYCCNT;
  const uint32_t snap_gnss = GPT2_CNT;
  const uint32_t snap_ocxo = GPT1_CNT;

  // ============================================================
  // Capture the spin loop's shadow value, then break the loop.
  //
  // The spin loop continuously writes DWT_CYCCNT to
  // dispatch_shadow_dwt.  When the ISR fires, the loop is
  // frozen mid-iteration.  We grab the shadow NOW, while the
  // loop can't touch it.
  //
  // After the ISR returns, the loop may resume and execute one
  // more write (if the interrupt landed between the pps_fired
  // check and the DWT write).  That clobbers dispatch_shadow_dwt
  // with a post-ISR value.  But isr_captured_shadow_dwt is safe.
  //
  // The ASAP callback uses isr_captured_shadow_dwt (not
  // dispatch_shadow_dwt) for the delta computation.
  // ============================================================

  isr_captured_shadow_dwt = dispatch_shadow_dwt;
  pps_fired = true;

  // ============================================================
  // PPS relay to Pi — assert HIGH immediately after snapshots.
  //
  // This runs on every PPS, regardless of campaign state.
  // chrony needs continuous PPS for system clock discipline.
  //
  // digitalWriteFast compiles to a single GPIO register write
  // (~1 ns at 1008 MHz).  The relay edge follows the true PPS
  // edge by ~30-50 ns (ISR entry + 3 snapshots + this write).
  //
  // The deassert timer is armed from the ASAP callback — NEVER
  // from ISR context.  timepop_arm's noInterrupts/interrupts
  // pair would re-enable interrupts mid-ISR, allowing PIT to
  // nest and corrupt timer state.
  // ============================================================

  digitalWriteFast(GNSS_PPS_RELAY, HIGH);
  relay_arm_pending = true;

  // ============================================================
  // Raw ISR-level residuals — computed here, zero overhead.
  //
  // Uses unsigned 32-bit subtraction (wrap-safe) cast to signed
  // 32-bit, then subtract expected.  At 10 MHz the delta is
  // 10,000,000 which fits easily in int32_t.  At 1008 MHz the
  // delta is 1,008,000,000 which also fits in int32_t (max ~2.1B).
  // ============================================================

  if (isr_residual_valid) {
    isr_residual_dwt  = (int32_t)(snap_dwt  - isr_prev_dwt)  - (int32_t)ISR_DWT_EXPECTED;
    isr_residual_gnss = (int32_t)(snap_gnss - isr_prev_gnss) - (int32_t)ISR_GNSS_EXPECTED;
    isr_residual_ocxo = (int32_t)(snap_ocxo - isr_prev_ocxo) - (int32_t)ISR_OCXO_EXPECTED;
  }

  // Store current as previous for next PPS
  isr_prev_dwt  = snap_dwt;
  isr_prev_gnss = snap_gnss;
  isr_prev_ocxo = snap_ocxo;

  // Store for callback's advance-then-rewind
  isr_snap_dwt  = snap_dwt;
  isr_snap_gnss = snap_gnss;
  isr_snap_ocxo = snap_ocxo;

  // ============================================================
  // Always schedule ASAP callback — the relay deassert timer
  // must be armed from scheduled context on every PPS.
  // Campaign-gated work happens inside the callback.
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

  // ----------------------------------------------------------
  // Optional: set_dac — apply a known DAC value immediately.
  // The Pi persists the last good DAC value and sends it back
  // on campaign start so the Teensy doesn't have to remember.
  // ----------------------------------------------------------
  uint32_t dac_val;
  if (args.tryGetUInt("set_dac", dac_val)) {
    ocxo_dac_write(dac_val);
  }

  // ----------------------------------------------------------
  // Optional: calibrate_ocxo — enable servo loop.
  // When present (any value or no value), the servo will
  // iteratively adjust the DAC to drive OCXO tau toward 1.0.
  // Can be combined with set_dac to start near a known-good
  // value and fine-tune from there.
  // ----------------------------------------------------------
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
  p.add("ocxo_dac", ocxo_dac_value);
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

  // All recovery parameters are nanoseconds.
  // The Teensy derives its own internal cycle/tick counts
  // from these values at the PPS boundary.
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

  // Restore DAC value if provided (Pi persists this)
  uint32_t dac_val;
  if (args.tryGetUInt("set_dac", dac_val)) {
    ocxo_dac_write(dac_val);
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
//
// cmd_report calls _now() for each domain.  This is always safe:
// _now() advances the accumulator forward monotonically.  The PPS
// callback's advance-then-rewind is unaffected because the rewind
// is computed from (last_32 - isr_snap), which is correct regardless
// of how many times _now() has been called in between.
//

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

  // These _now() calls are always safe — they advance the
  // accumulator forward monotonically.  No interaction with
  // the PPS callback's advance-then-rewind pattern.
  const uint64_t dwt_cycles = clocks_dwt_cycles_now();
  const uint64_t dwt_ns     = clocks_dwt_ns_now();
  const uint64_t gnss_ns    = clocks_gnss_ns_now();
  const uint64_t ocxo_ns    = clocks_ocxo_ns_now();

  p.add("dwt_cycles_now", dwt_cycles);
  p.add("dwt_ns_now",     dwt_ns);
  p.add("gnss_ns_now",    gnss_ns);
  p.add("ocxo_ns_now",    ocxo_ns);

  p.add("gnss_lock", digitalRead(GNSS_LOCK_PIN));

  // Callback-level residual statistics
  report_residual(p, "dwt",  residual_dwt);
  report_residual(p, "gnss", residual_gnss);
  report_residual(p, "ocxo", residual_ocxo);

  // ISR-level raw residuals (most recent PPS)
  p.add("isr_residual_dwt",   isr_residual_dwt);
  p.add("isr_residual_gnss",  isr_residual_gnss);
  p.add("isr_residual_ocxo",  isr_residual_ocxo);
  p.add("isr_residual_valid", (bool)isr_residual_valid);

  // Pre-PPS dispatch latency profiling
  p.add("dispatch_shadow_ns",  dispatch_shadow_ns);
  p.add("dispatch_isr_ns",     dispatch_isr_ns);
  p.add("dispatch_delta_ns",   dispatch_delta_ns);
  p.add("pre_pps_approach_ns", pre_pps_approach_ns);
  p.add("dispatch_timeout",    (bool)dispatch_timeout);

  // Software TDC correction diagnostics
  p.add("pps_edge_valid",         pps_edge_valid);
  p.add("pps_edge_correction_ns", pps_edge_correction_ns);
  p.add("tdc_needs_recal",        TDC_NEEDS_RECALIBRATION);

  // OCXO DAC control state
  p.add("ocxo_dac",              ocxo_dac_value);
  p.add("calibrate_ocxo",        calibrate_ocxo_active);
  p.add("servo_adjustments",     servo_adjustments);
  p.add("servo_step",             servo_step);
  p.add("servo_last_residual",    servo_last_residual);
  p.add("servo_settle_count",     servo_settle_count);

  // Dispatch profiling diagnostics
  p.add("diag_coarse_fires",   diag_coarse_fire_count);
  p.add("diag_fine_fires",     diag_fine_fire_count);
  p.add("diag_late",           diag_fine_late_count);
  p.add("diag_spins",          diag_spin_count);
  p.add("diag_timeouts",       diag_timeout_count);
  p.add("diag_fine_was_late",  diag_fine_was_late);
  p.add("diag_spin_iters",     diag_spin_iterations);

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

  // OCXO DAC control output (pin 22 / A22)
  // Initialize before clock capture so the OCXO starts receiving
  // a defined control voltage immediately.
  analogWriteResolution(12);
  ocxo_dac_write(OCXO_DAC_DEFAULT);

  dwt_enable();
  arm_gpt2_external();    // GNSS VCLOCK -> GPT2 pin 14, raw 10 MHz, polled
  arm_gpt1_external();    // OCXO 10 MHz -> GPT1 pin 25, raw 10 MHz, polled

  // PPS input from GF-8802 (pin 1, shielded twisted pair)
  pinMode(GNSS_PPS_PIN, INPUT);
  pinMode(GNSS_LOCK_PIN, INPUT);

  // PPS relay output to Pi (pin 32 -> Pi GPIO 18)
  // Configured as OUTPUT, initially LOW.
  // Driven HIGH in ISR, deasserted LOW by TimePop after 500 ms.
  pinMode(GNSS_PPS_RELAY, OUTPUT);
  digitalWriteFast(GNSS_PPS_RELAY, LOW);

  attachInterrupt(
    digitalPinToInterrupt(GNSS_PPS_PIN),
    pps_isr,
    RISING
  );

  NVIC_SET_PRIORITY(IRQ_GPIO6789, 0);  // This is PPS from GF-8802
}