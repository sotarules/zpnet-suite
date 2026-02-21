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
//   DWT    — ARM Cortex-M7 cycle counter, 600 MHz, internal
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
//   The relay latency is deterministic (~30-50 ns after the
//   snapshots, ~50-70 ns after the true PPS edge) and constant.
//   This is well within chrony's discipline capability.
//
// Recovery:
//
//   When the Pi restarts mid-campaign, it reads the last TIMEBASE
//   from Postgres, projects all clock values forward using tau,
//   and sends RECOVER with the projected dwt_cycles, gnss_ns, and
//   ocxo_ns.  At the next PPS edge, the Teensy loads these values
//   into its 64-bit accumulators, latches the current hardware
//   register positions, transitions to STARTED, and resumes
//   publishing TIMEBASE_FRAGMENTs.  The clocks continue as if
//   the interruption never happened.
//
// Pre-PPS Approach Timing (experimental):
//
//   To characterize ISR dispatch latency, we need to know how
//   close to the PPS edge our software actually executes.  This
//   is measured with a two-stage timer chain:
//
//     Stage 1 (PRE_PPS_COARSE, ms):  999 ms after PPS
//     Stage 2 (PRE_PPS, µs):         999 µs after stage 1
//
//   Stage 2 snapshots GPT2_CNT (GNSS VCLK, 10 MHz) into a
//   volatile shadow variable.  When the next PPS ISR fires,
//   the delta between the ISR snapshot and the shadow gives
//   the "approach time" — how many GNSS nanoseconds elapsed
//   between our closest pre-PPS observation and the actual
//   PPS edge.
//
//   Expected value: ~1 µs (1000 ns) = ~10 GNSS ticks.
//   This is purely observational — no existing behavior changes.
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

// Recovery parameters (authoritative, supplied by Pi)
static uint64_t recover_dwt_cycles = 0;
static uint64_t recover_gnss_ns    = 0;
static uint64_t recover_ocxo_ns    = 0;

// Campaign-scoped PPS second counter
static uint64_t campaign_seconds = 0;

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
static constexpr uint32_t ISR_DWT_EXPECTED  = 600000000u;
static constexpr uint32_t ISR_GNSS_EXPECTED =  10000000u;
static constexpr uint32_t ISR_OCXO_EXPECTED =  10000000u;

// ============================================================================
// Pre-PPS approach timing (experimental)
// ============================================================================
//
// Shadow GNSS snapshot captured by the µs-stage callback,
// ~1 µs before the expected PPS edge.
//
// Written by pre_pps_fine_cb(), read by the PPS ASAP callback.
// pre_pps_approach_ns holds the computed result for publishing.
//

static volatile uint32_t pre_pps_shadow_gnss  = 0;
static volatile bool     pre_pps_shadow_valid = false;
static volatile int32_t  pre_pps_approach_ns  = -1;

// ============================================================================
// PPS residual tracking — per-clock-domain (Welford's, callback-level)
// ============================================================================
//
// These use the 64-bit extended values from the callback for running
// statistics.  The ISR residuals above are the raw, unprocessed
// single-sample values.
//

static constexpr int64_t DWT_EXPECTED_PER_PPS  = 600000000LL;
static constexpr int64_t GNSS_EXPECTED_PER_PPS =  10000000LL;
static constexpr int64_t OCXO_EXPECTED_PER_PPS =  10000000LL;

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
// DWT (CPU cycle counter — 600 MHz internal)
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

static uint64_t dwt_extend_snapshot(uint32_t snap) {
  dwt_cycles_64 += (uint32_t)(snap - dwt_last_32);
  dwt_last_32 = snap;
  return dwt_cycles_64;
}

uint64_t clocks_dwt_ns_now(void) {
  return (clocks_dwt_cycles_now() * 5ull) / 3ull;
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

static uint64_t gnss_extend_snapshot(uint32_t snap) {
  gnss_ticks_64 += (uint32_t)(snap - gnss_last_32);
  gnss_last_32 = snap;
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

static uint64_t ocxo_extend_snapshot(uint32_t snap) {
  ocxo_ticks_64 += (uint32_t)(snap - ocxo_last_32);
  ocxo_last_32 = snap;
  return ocxo_ticks_64;
}

uint64_t clocks_ocxo_ns_now(void) {
  if (campaign_state != clocks_campaign_state_t::STARTED) return 0;
  return clocks_ocxo_ticks_now() * 100ull;
}

// ============================================================================
// Zeroing (campaign-scoped — fresh start only, NOT used for recovery)
// ============================================================================

static void clocks_zero_all(void) {
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

  // Reset pre-PPS approach timing
  pre_pps_shadow_gnss  = 0;
  pre_pps_shadow_valid = false;
  pre_pps_approach_ns  = -1;
}

// ============================================================================
// Pre-PPS approach timing — two-stage callback chain
// ============================================================================
//
// Stage 1 (coarse, ms):  Armed after each PPS.  Fires 999 ms later.
//                         Its only job is to arm stage 2.
//
// Stage 2 (fine, µs):    Armed by stage 1.  Fires 999 µs later
//                         (~1 µs before the next expected PPS edge).
//                         Snapshots GPT2_CNT into pre_pps_shadow_gnss.
//
// The next PPS ISR captures isr_snap_gnss.  The ASAP callback
// computes the approach time:
//
//   approach_ns = (isr_snap_gnss - pre_pps_shadow_gnss) * 100
//
// This tells us how many GNSS nanoseconds elapsed between our
// closest pre-PPS observation and the actual PPS edge.
//

static void pre_pps_fine_cb(timepop_ctx_t*, void*) {
  // Snapshot GNSS VCLK — this is our closest observation
  // before the upcoming PPS edge.
  pre_pps_shadow_gnss = GPT2_CNT;
  pre_pps_shadow_valid = true;
}

static void pre_pps_coarse_cb(timepop_ctx_t*, void*) {
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
static void pre_pps_arm_chain(void) {
  timepop_arm(
    TIMEPOP_CLASS_PRE_PPS_COARSE,
    false,                    // one-shot
    pre_pps_coarse_cb,
    nullptr,
    "pre-pps-coarse"
  );
}

// ============================================================================
// PPS handling
// ============================================================================
//
// Two-phase architecture:
//
//   Phase 1 (ISR): Snapshot all hardware counters, compute raw
//   PPS-to-PPS residuals, and assert PPS relay HIGH — all
//   immediately at the PPS rising edge.  Three register reads
//   (~10 ns total), three subtractions, one digitalWriteFast.
//   Set relay_arm_pending flag for deferred timer arming.
//   NO timepop_arm calls from ISR context.
//
//   Phase 2 (timepop ASAP callback): Arm the relay deassert
//   timer first (always, regardless of campaign state), then
//   extend snapshots to 64-bit, process state transitions,
//   compute Welford statistics, build and publish the
//   TIMEBASE_FRAGMENT.
//

static volatile bool pps_scheduled = false;

static void pps_isr(void) {

  // ============================================================
  // PHASE 1: Hardware snapshot — absolute minimum latency
  //
  // Three register reads within ~10 ns of each other at 600 MHz.
  // This is as close to the PPS edge as software can get.
  // ============================================================

  const uint32_t snap_dwt  = DWT_CYCCNT;
  const uint32_t snap_gnss = GPT2_CNT;
  const uint32_t snap_ocxo = GPT1_CNT;

  // ============================================================
  // PPS relay to Pi — assert HIGH immediately after snapshots.
  //
  // This runs on every PPS, regardless of campaign state.
  // chrony needs continuous PPS for system clock discipline.
  //
  // digitalWriteFast compiles to a single GPIO register write
  // (~3 ns at 600 MHz).  The relay edge follows the true PPS
  // edge by ~50-70 ns (ISR entry + 3 snapshots + this write).
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
  // 10,000,000 which fits easily in int32_t.  At 600 MHz the
  // delta is 600,000,000 which also fits in int32_t (max ~2.1B).
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

  // Store for callback's 64-bit extension
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
    [](timepop_ctx_t*, void*) {

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
        pre_pps_shadow_valid = false;
      }

      if (request_recover) {
        dwt_cycles_64 = recover_dwt_cycles;
        gnss_ticks_64 = recover_gnss_ns / 100ull;
        ocxo_ticks_64 = recover_ocxo_ns / 100ull;

        dwt_last_32  = isr_snap_dwt;
        gnss_last_32 = isr_snap_gnss;
        ocxo_last_32 = isr_snap_ocxo;

        campaign_seconds = recover_gnss_ns / 1000000000ull;

        residual_reset(residual_dwt);
        residual_reset(residual_gnss);
        residual_reset(residual_ocxo);

        // ISR residuals will become valid on the next PPS
        isr_residual_valid = false;

        // Reset pre-PPS approach timing
        pre_pps_shadow_valid = false;
        pre_pps_approach_ns  = -1;

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
        // Pre-PPS approach: compute result from previous cycle
        // --------------------------------------------------------

        if (pre_pps_shadow_valid) {
          // GNSS VCLK is 10 MHz → 100 ns per tick
          int32_t delta_ticks = (int32_t)(isr_snap_gnss - pre_pps_shadow_gnss);
          pre_pps_approach_ns = delta_ticks * 100;
          pre_pps_shadow_valid = false;
        }

        // --------------------------------------------------------
        // Arm pre-PPS chain for the NEXT PPS edge
        // --------------------------------------------------------

        pre_pps_arm_chain();

        // --------------------------------------------------------
        // Extend ISR snapshots to 64-bit
        // --------------------------------------------------------

        const uint64_t snap_dwt_cycles = dwt_extend_snapshot(isr_snap_dwt);
        const uint64_t snap_dwt_ns     = (snap_dwt_cycles * 5ull) / 3ull;
        const uint64_t snap_gnss_ticks = gnss_extend_snapshot(isr_snap_gnss);
        const uint64_t snap_gnss_ns    = snap_gnss_ticks * 100ull;
        const uint64_t snap_ocxo_ticks = ocxo_extend_snapshot(isr_snap_ocxo);
        const uint64_t snap_ocxo_ns    = snap_ocxo_ticks * 100ull;

        // --------------------------------------------------------
        // Callback-level Welford residual tracking
        // --------------------------------------------------------

        residual_update(residual_dwt,  snap_dwt_cycles, DWT_EXPECTED_PER_PPS);
        residual_update(residual_gnss, snap_gnss_ticks, GNSS_EXPECTED_PER_PPS);
        residual_update(residual_ocxo, snap_ocxo_ticks, OCXO_EXPECTED_PER_PPS);

        // --------------------------------------------------------
        // Publish TIMEBASE_FRAGMENT
        // --------------------------------------------------------

        Payload p;
        p.add("campaign",         campaign_name);
        p.add("dwt_cycles",       snap_dwt_cycles);
        p.add("dwt_ns",           snap_dwt_ns);
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

        // Pre-PPS approach timing (experimental)
        // How many GNSS nanoseconds between our closest pre-PPS
        // observation and the actual PPS edge.
        // -1 = not yet available (first PPS of campaign).
        p.add("pre_pps_approach_ns", pre_pps_approach_ns);

        publish("TIMEBASE_FRAGMENT", p);

        if (campaign_state == clocks_campaign_state_t::STARTED) {
          campaign_seconds++;
        }
      }

      pps_scheduled = false;
    },
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
  request_start = true;
  request_stop  = false;

  Payload p;
  p.add("status", "start_requested");
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

  const char* s_dwt  = args.getString("dwt_cycles");
  const char* s_gnss = args.getString("gnss_ns");
  const char* s_ocxo = args.getString("ocxo_ns");

  if (!s_dwt || !s_gnss || !s_ocxo) {
    Payload err;
    err.add("error", "missing recovery parameters");
    return err;
  }

  recover_dwt_cycles = strtoull(s_dwt,  nullptr, 10);
  recover_gnss_ns    = strtoull(s_gnss, nullptr, 10);
  recover_ocxo_ns    = strtoull(s_ocxo, nullptr, 10);

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

  // Callback-level residual statistics
  report_residual(p, "dwt",  residual_dwt);
  report_residual(p, "gnss", residual_gnss);
  report_residual(p, "ocxo", residual_ocxo);

  // ISR-level raw residuals (most recent PPS)
  p.add("isr_residual_dwt",   isr_residual_dwt);
  p.add("isr_residual_gnss",  isr_residual_gnss);
  p.add("isr_residual_ocxo",  isr_residual_ocxo);
  p.add("isr_residual_valid", (bool)isr_residual_valid);

  // Pre-PPS approach timing
  p.add("pre_pps_approach_ns", pre_pps_approach_ns);

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
}