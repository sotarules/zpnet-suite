// =============================================================
// FILE: clock.cpp
// =============================================================
//
// ZPNet Clock Subsystem — Lazy Reconciliation + Cooperative Guard Tick
//
// Philosophy:
//   • Synthetic clocks are correct-on-demand.
//   • We do NOT simulate tick-by-tick time passage.
//   • Each *_now() reconciles the underlying 32-bit hardware counter
//     into a monotonic 64-bit accumulator using wrap-safe deltas.
//   • A single system-level guard tick runs cooperatively (TimePop)
//     to prevent rollover loss in the absence of queries.
//
// Design constraints:
//   • NO ISRs
//   • NO asynchronous mutation
//   • NO prescalers (unless required by a specific source)
//   • NO per-process clock ownership
//
// All clock state is mutated in a single causal execution context.
//
// Author: The Mule + GPT
//

#include "clock.h"
#include "timepop.h"

#include <Arduino.h>
#include "imxrt.h"   // GPT1_CNT / GPT2_CNT and CCM/IOMUXC regs on Teensy 4.x

// --------------------------------------------------------------
// DWT Registers (core-local)
// --------------------------------------------------------------
#define DEMCR               (*(volatile uint32_t *)0xE000EDFC)
#define DEMCR_TRCENA        (1 << 24)

#define DWT_CTRL            (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT          (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL_CYCCNTENA  (1 << 0)

// --------------------------------------------------------------
// Internal synthetic time state (monotonic 64-bit)
// --------------------------------------------------------------
static uint64_t dwt_acc   = 0;
static uint32_t dwt_last  = 0;

static uint64_t gnss_acc  = 0;
static uint32_t gnss_last = 0;

static uint64_t ocxo_acc  = 0;
static uint32_t ocxo_last = 0;

// --------------------------------------------------------------
// Arm-once flags
// --------------------------------------------------------------
static volatile bool gpt1_armed = false;
static volatile bool gpt2_armed = false;

// --------------------------------------------------------------
// Guard tick (forward declaration)
// --------------------------------------------------------------
static void clock_guard_tick(void*);

// --------------------------------------------------------------
// DWT enable
// --------------------------------------------------------------
static void dwt_enable() {
  DEMCR |= DEMCR_TRCENA;
  DWT_CYCCNT = 0;
  DWT_CTRL |= DWT_CTRL_CYCCNTENA;
  dwt_last = DWT_CYCCNT;
}

// --------------------------------------------------------------
// Clock gating — *BUS* gates only (surgical, proven-safe)
//
// GPT2 BUS gate lives in CCGR0 on Teensy 4.1 (per your known-good code).
// GPT1 BUS gate is typically CCGR1_GPT1_BUS in Teensy headers.
// --------------------------------------------------------------
static inline void enable_gpt1_bus_gate() {
#if defined(CCM_CCGR1_GPT1_BUS)
  CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON);
#else
# error "Missing CCM_CCGR1_GPT1_BUS macro (unexpected for Teensy 4.x core)"
#endif
}

static inline void enable_gpt2_bus_gate() {
#if defined(CCM_CCGR0_GPT2_BUS)
  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON);
#else
# error "Missing CCM_CCGR0_GPT2_BUS macro (unexpected for Teensy 4.x core)"
#endif
}

// --------------------------------------------------------------
// GPT2: external clock on pin 14 (GPIO_AD_B1_02, ALT8 -> GPT2_CLK)
//
// This is lifted directly from your known-good snippet.
// Invariant: NEVER write GPT2_CNT in external clock mode.
// --------------------------------------------------------------
static inline void gpt2_arm_external_clock_pin14() {
  if (gpt2_armed) return;

  enable_gpt2_bus_gate();

  // ALT8 = GPT2 external clock input
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 8;

  // Conservative pad control:
  //  - Input enabled
  //  - Hysteresis enabled
  //  - Keeper enabled
  //  - Medium drive strength
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 =
      IOMUXC_PAD_HYS |
      IOMUXC_PAD_PKE |
      IOMUXC_PAD_PUE |
      IOMUXC_PAD_DSE(4);

  // Route pad to GPT2 clock input
  IOMUXC_GPT2_IPP_IND_CLKIN_SELECT_INPUT = 1;

  // Disable GPT2 before configuration
  GPT2_CR = 0;

  // Clear all status flags
  GPT2_SR = 0x3F;

  // Select external clock source: CLKSRC = 3 -> external
  GPT2_CR = GPT_CR_CLKSRC(3);

  // Enable GPT2 (do once, never disable)
  GPT2_CR |= GPT_CR_EN;

  gpt2_armed = true;
}

// --------------------------------------------------------------
// GPT1: safe baseline free-run (internal peripheral clock)
//
// This is intentionally NOT external-clocked until we have the exact pin/mux.
// It still gives you a stable counter source for ocxo_now() semantics.
// --------------------------------------------------------------
static inline void gpt1_arm_internal_freerun() {
  if (gpt1_armed) return;

  enable_gpt1_bus_gate();

  // Minimal, safe free-run configuration (internal clock)
  GPT1_CR = 0;
  GPT1_SR = 0x3F;
  GPT1_PR = 0;

  // CLKSRC = 1 (peripheral clock), FRR = free-run
  GPT1_CR = GPT_CR_CLKSRC(1) | GPT_CR_FRR;
  GPT1_CR |= GPT_CR_EN;

  gpt1_armed = true;
}

// --------------------------------------------------------------
// Seeding
// --------------------------------------------------------------
static void gnss_seed() {
  gnss_last = GPT2_CNT;   // read-only
}

static void ocxo_seed() {
  ocxo_last = GPT1_CNT;
}

// --------------------------------------------------------------
// Public initialization
//
// Must be called AFTER serial_init() and BEFORE sustained RPC traffic.
// --------------------------------------------------------------
void clock_init() {
  dwt_enable();

  // Arm GPT2 using the known-good external-clock configuration.
  gpt2_arm_external_clock_pin14();

  // Arm GPT1 in safe internal mode for now.
  // Later: replace this with a GPT1 external-clock armer once pin/mux is finalized.
  gpt1_arm_internal_freerun();

  // Seed synthetic accumulators
  gnss_seed();
  ocxo_seed();

  // Guard tick remains optional; turn it on once GPT paths are stable.
  /*
  timepop_schedule(
    2000,
    TIMEPOP_UNITS_MILLISECONDS,
    clock_guard_tick,
    nullptr,
    "clock-guard"
  );
  */
}

// --------------------------------------------------------------
// Guard tick (cooperative, minimal, neutral)
// --------------------------------------------------------------
static void clock_guard_tick(void*) {
  (void)dwt_now();
  (void)gnss_now();
  (void)ocxo_now();

  timepop_schedule(
    2000,
    TIMEPOP_UNITS_MILLISECONDS,
    clock_guard_tick,
    nullptr,
    "clock-guard"
  );
}

// --------------------------------------------------------------
// Query (lazy reconciliation + wrap-safe delta)
// --------------------------------------------------------------
uint64_t dwt_now() {
  uint32_t now   = DWT_CYCCNT;
  uint32_t delta = now - dwt_last;
  dwt_acc  += (uint64_t)delta;
  dwt_last  = now;
  return dwt_acc;
}

uint64_t gnss_now() {
  // External-clocked GPT2: read-only counter.
  if (!gpt2_armed) return gnss_acc;

  uint32_t now   = GPT2_CNT;
  uint32_t delta = now - gnss_last;
  gnss_acc += (uint64_t)delta;
  gnss_last = now;
  return gnss_acc;
}

uint64_t ocxo_now() {
  if (!gpt1_armed) return ocxo_acc;

  uint32_t now   = GPT1_CNT;
  uint32_t delta = now - ocxo_last;
  ocxo_acc += (uint64_t)delta;
  ocxo_last = now;
  return ocxo_acc;
}

// --------------------------------------------------------------
// Optional zeroing
// --------------------------------------------------------------
void dwt_zero() {
  dwt_acc  = 0;
  dwt_last = DWT_CYCCNT;
}

void gnss_zero() {
  gnss_acc  = 0;
  gnss_last = gpt2_armed ? GPT2_CNT : 0;
}

void ocxo_zero() {
  ocxo_acc  = 0;
  ocxo_last = gpt1_armed ? GPT1_CNT : 0;
}
