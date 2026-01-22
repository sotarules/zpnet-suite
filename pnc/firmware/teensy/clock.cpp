// =============================================================
// FILE: clock.cpp
// =============================================================
//
// ZPNet Clock Subsystem — Hardware-Prescaled Edge Provider
//
// Responsibilities:
//   • Arm GPT hardware safely
//   • Enable hardware prescaling (÷1000 → 10 kHz)
//   • Maintain monotonic 64-bit synthetic clocks
//   • Maintain diagnostic prescaled tick counters
//   • Deliver prescaled edge interrupts to SmartPOP
//
// NON-RESPONSIBILITIES:
//   • No scheduling policy
//   • No client callbacks
//   • No SmartPOP logic
//
// Author: The Mule + GPT
//

#include "clock.h"
#include "timepop.h"
#include "smartpop.h"   // forward dependency (implemented later)

#include <Arduino.h>
#include "imxrt.h"

// --------------------------------------------------------------
// DWT Registers (core-local)
// --------------------------------------------------------------
#define DEMCR               (*(volatile uint32_t *)0xE000EDFC)
#define DEMCR_TRCENA        (1 << 24)

#define DWT_CTRL            (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT          (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL_CYCCNTENA  (1 << 0)

// --------------------------------------------------------------
// Configuration
// --------------------------------------------------------------
#define GPT_PRESCALE_DIVIDER   1000u
#define GPT_PRESCALE_VALUE     (GPT_PRESCALE_DIVIDER - 1)

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
// Diagnostic prescaled tick counters (10 kHz domains)
// --------------------------------------------------------------
static volatile uint64_t gnss_prescaled_ticks = 0;
static volatile uint64_t ocxo_prescaled_ticks = 0;

// --------------------------------------------------------------
// Arm-once flags
// --------------------------------------------------------------
static volatile bool gpt1_armed = false;
static volatile bool gpt2_armed = false;

// --------------------------------------------------------------
// Forward declarations
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
// Clock gating — BUS gates only (proven-safe)
// --------------------------------------------------------------
static inline void enable_gpt1_bus_gate() {
#if defined(CCM_CCGR1_GPT1_BUS)
  CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON);
#else
# error "Missing CCM_CCGR1_GPT1_BUS"
#endif
}

static inline void enable_gpt2_bus_gate() {
#if defined(CCM_CCGR0_GPT2_BUS)
  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON);
#else
# error "Missing CCM_CCGR0_GPT2_BUS"
#endif
}

// --------------------------------------------------------------
// GPT2 — GNSS external clock, prescaled in hardware (10 MHz → 10 kHz)
// --------------------------------------------------------------
static inline void gpt2_arm_external_clock_pin14() {
  if (gpt2_armed) return;

  enable_gpt2_bus_gate();

  // Pin 14 = GPIO_AD_B1_02, ALT8 → GPT2_CLK
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 8;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 =
      IOMUXC_PAD_HYS |
      IOMUXC_PAD_PKE |
      IOMUXC_PAD_PUE |
      IOMUXC_PAD_DSE(4);

  IOMUXC_GPT2_IPP_IND_CLKIN_SELECT_INPUT = 1;

  GPT2_CR = 0;
  GPT2_SR = 0x3F;
  GPT2_PR = GPT_PRESCALE_VALUE;      // ÷1000 hardware prescaler
  GPT2_CR = GPT_CR_CLKSRC(3);         // external clock
  GPT2_CR |= GPT_CR_EN;

  gpt2_armed = true;
}

// --------------------------------------------------------------
// GPT1 — OCXO baseline free-run, prescaled in hardware
// --------------------------------------------------------------
static inline void gpt1_arm_internal_freerun() {
  if (gpt1_armed) return;

  enable_gpt1_bus_gate();

  GPT1_CR = 0;
  GPT1_SR = 0x3F;
  GPT1_PR = GPT_PRESCALE_VALUE;      // ÷1000 hardware prescaler
  GPT1_CR = GPT_CR_CLKSRC(1) | GPT_CR_FRR;
  GPT1_CR |= GPT_CR_EN;

  gpt1_armed = true;
}

// --------------------------------------------------------------
// Seeding
// --------------------------------------------------------------
static void gnss_seed() { gnss_last = GPT2_CNT; }
static void ocxo_seed() { ocxo_last = GPT1_CNT; }

// --------------------------------------------------------------
// Public initialization
// --------------------------------------------------------------
void clock_init() {
  dwt_enable();

  gpt2_arm_external_clock_pin14();
  gpt1_arm_internal_freerun();

  gnss_seed();
  ocxo_seed();

  // Guard tick prevents rollover loss (lazy reconciliation)
  timepop_schedule(
    2000,
    TIMEPOP_UNITS_MILLISECONDS,
    clock_guard_tick,
    nullptr,
    "clock-guard"
  );
}

// --------------------------------------------------------------
// Guard tick — prevents rollover loss
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
// GPT interrupt delegation + diagnostic tick counting
// --------------------------------------------------------------

extern "C" void gpt1_isr(void) {
  GPT1_SR = 0x3F;          // clear all status flags
  ocxo_prescaled_ticks++; // diagnostic
  smartpop_ocxo_tick();   // delegate
}

extern "C" void gpt2_isr(void) {
  GPT2_SR = 0x3F;          // clear all status flags
  gnss_prescaled_ticks++; // diagnostic
  smartpop_gnss_tick();   // delegate
}

// --------------------------------------------------------------
// Query — lazy reconciliation (prescaled domains)
// --------------------------------------------------------------
uint64_t dwt_now() {
  uint32_t now   = DWT_CYCCNT;
  uint32_t delta = now - dwt_last;
  dwt_acc  += (uint64_t)delta;
  dwt_last  = now;
  return dwt_acc;
}

uint64_t gnss_now() {
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
// Diagnostic prescaled tick accessors
// --------------------------------------------------------------
uint64_t clock_gnss_prescaled_ticks() {
  return gnss_prescaled_ticks;
}

uint64_t clock_ocxo_prescaled_ticks() {
  return ocxo_prescaled_ticks;
}

// --------------------------------------------------------------
// Optional zeroing (synthetic + prescaled domains)
// --------------------------------------------------------------
void dwt_zero() {
  dwt_acc  = 0;
  dwt_last = DWT_CYCCNT;
}

void gnss_zero() {
  gnss_acc             = 0;
  gnss_last            = gpt2_armed ? GPT2_CNT : 0;
  gnss_prescaled_ticks = 0;
}

void ocxo_zero() {
  ocxo_acc             = 0;
  ocxo_last            = gpt1_armed ? GPT1_CNT : 0;
  ocxo_prescaled_ticks = 0;
}
