// =============================================================
// FILE: clock.cpp
// =============================================================
//
// ZPNet Clock Subsystem — Hardware-Prescaled Edge Provider
//
// Provides:
//   • Raw monotonic ledgers (DWT, GNSS 10kHz, OCXO 10kHz)
//   • Prescaled hardware edge delivery
//   • Synthetic nanosecond clocks (integer math, reconciled)
//   • SmartPOP edge dispatch
//
// Author: The Mule + GPT
//

#include "clock.h"
#include "timepop.h"
#include "smartpop.h"

#include <Arduino.h>
#include "imxrt.h"

// --------------------------------------------------------------
// DWT Registers
// --------------------------------------------------------------
#define DEMCR               (*(volatile uint32_t *)0xE000EDFC)
#define DEMCR_TRCENA        (1 << 24)

#define DWT_CTRL            (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT          (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL_CYCCNTENA  (1 << 0)

// --------------------------------------------------------------
// Prescale configuration
// --------------------------------------------------------------
#define GPT_PRESCALE_DIVIDER   1000u
#define GPT_PRESCALE_VALUE     (GPT_PRESCALE_DIVIDER - 1)

// --------------------------------------------------------------
// Canonical ledgers
// --------------------------------------------------------------

// Raw DWT cycles (≈600 MHz), wrap-safe
static uint64_t dwt_cycles_64 = 0;
static uint32_t dwt_cycles_last = 0;

// GNSS / OCXO prescaled tick domains (10 kHz)
static volatile uint64_t gnss_10khz_ticks = 0;
static volatile uint64_t ocxo_10khz_ticks = 0;

// --------------------------------------------------------------
// Interpolation baselines (captured at prescaled edges)
// --------------------------------------------------------------
static volatile uint32_t gnss_dwt_baseline = 0;
static volatile uint32_t ocxo_dwt_baseline = 0;

// --------------------------------------------------------------
// Arm flags
// --------------------------------------------------------------
static bool gpt1_armed = false;
static bool gpt2_armed = false;

// --------------------------------------------------------------
// Forward declarations
// --------------------------------------------------------------
static void clock_guard_tick(timepop_ctx_t*, void*);
extern "C" void gpt1_isr(void);
extern "C" void gpt2_isr(void);

// --------------------------------------------------------------
// DWT enable
// --------------------------------------------------------------
static void dwt_enable() {
  DEMCR |= DEMCR_TRCENA;
  DWT_CYCCNT = 0;
  DWT_CTRL |= DWT_CTRL_CYCCNTENA;
  dwt_cycles_last = DWT_CYCCNT;
}

// --------------------------------------------------------------
// Clock gating
// --------------------------------------------------------------
static inline void enable_gpt1_bus_gate() {
  CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON);
}

static inline void enable_gpt2_bus_gate() {
  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON);
}

// --------------------------------------------------------------
// GPT2 — GNSS external clock (10 MHz → 10 kHz)
// --------------------------------------------------------------
static void gpt2_arm_external_clock_pin14() {
  if (gpt2_armed) return;

  enable_gpt2_bus_gate();

  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 8;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 =
      IOMUXC_PAD_HYS | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_DSE(4);

  IOMUXC_GPT2_IPP_IND_CLKIN_SELECT_INPUT = 1;

  GPT2_CR = 0;
  GPT2_SR = 0x3F;
  GPT2_PR = GPT_PRESCALE_VALUE;
  GPT2_CR = GPT_CR_CLKSRC(3) | GPT_CR_FRR;

  GPT2_OCR1 = GPT2_CNT + 1;
  GPT2_IR |= GPT_IR_OF1IE;

  attachInterruptVector(IRQ_GPT2, gpt2_isr);
  NVIC_ENABLE_IRQ(IRQ_GPT2);

  GPT2_CR |= GPT_CR_EN;
  gpt2_armed = true;
}

// --------------------------------------------------------------
// GPT1 — OCXO prescaled free-run
// --------------------------------------------------------------
static void gpt1_arm_internal_freerun() {
  if (gpt1_armed) return;

  enable_gpt1_bus_gate();

  GPT1_CR = 0;
  GPT1_SR = 0x3F;
  GPT1_PR = GPT_PRESCALE_VALUE;
  GPT1_CR = GPT_CR_CLKSRC(1) | GPT_CR_FRR;

  GPT1_OCR1 = GPT1_CNT + 1;
  GPT1_IR  |= GPT_IR_OF1IE;

  attachInterruptVector(IRQ_GPT1, gpt1_isr);
  NVIC_ENABLE_IRQ(IRQ_GPT1);

  GPT1_CR |= GPT_CR_EN;
  gpt1_armed = true;
}

// --------------------------------------------------------------
// Init
// --------------------------------------------------------------
void clock_init() {
  dwt_enable();
  gpt2_arm_external_clock_pin14();
  gpt1_arm_internal_freerun();

  timepop_arm(
    TIMEPOP_CLASS_GUARD,
    true,                 // recurring
    clock_guard_tick,
    nullptr,
    "clock-guard"
  );
}

// --------------------------------------------------------------
// Guard tick
// --------------------------------------------------------------
static void clock_guard_tick(timepop_ctx_t*, void*) {
  (void)clock_dwt_cycles_now();
}

// --------------------------------------------------------------
// GPT ISRs — prescaled edge truth
// --------------------------------------------------------------
extern "C" void gpt2_isr(void) {
  GPT2_SR = GPT_SR_OF1;
  GPT2_OCR1 += 1;

  gnss_10khz_ticks++;
  gnss_dwt_baseline = DWT_CYCCNT;

  smartpop_gnss_tick();
}

extern "C" void gpt1_isr(void) {
  GPT1_SR = GPT_SR_OF1;
  GPT1_OCR1 += 1;

  ocxo_10khz_ticks++;
  ocxo_dwt_baseline = DWT_CYCCNT;

  smartpop_ocxo_tick();
}

// --------------------------------------------------------------
// Raw accessors
// --------------------------------------------------------------
uint64_t clock_dwt_cycles_now() {
  uint32_t now = DWT_CYCCNT;
  dwt_cycles_64 += (uint32_t)(now - dwt_cycles_last);
  dwt_cycles_last = now;
  return dwt_cycles_64;
}

uint64_t clock_gnss_10khz_ticks() { return gnss_10khz_ticks; }
uint64_t clock_ocxo_10khz_ticks() { return ocxo_10khz_ticks; }

// --------------------------------------------------------------
// Synthetic nanosecond clocks (integer math)
// --------------------------------------------------------------
static constexpr uint64_t NS_PER_10KHZ_TICK = 100000; // 100 µs

uint64_t clock_dwt_ns_now() {
  // ns = cycles * (5 / 3)
  return (clock_dwt_cycles_now() * 5ull) / 3ull;
}

uint64_t clock_gnss_ns_now() {
  uint64_t base_ns = gnss_10khz_ticks * NS_PER_10KHZ_TICK;
  uint32_t delta_cycles = DWT_CYCCNT - gnss_dwt_baseline;

  uint64_t interp_ns = (delta_cycles * 5ull) / 3ull;
  if (interp_ns > NS_PER_10KHZ_TICK) interp_ns = NS_PER_10KHZ_TICK;

  return base_ns + interp_ns;
}

uint64_t clock_ocxo_ns_now() {
  uint64_t base_ns = ocxo_10khz_ticks * NS_PER_10KHZ_TICK;
  uint32_t delta_cycles = DWT_CYCCNT - ocxo_dwt_baseline;

  uint64_t interp_ns = (delta_cycles * 5ull) / 3ull;
  if (interp_ns > NS_PER_10KHZ_TICK) interp_ns = NS_PER_10KHZ_TICK;

  return base_ns + interp_ns;
}

// --------------------------------------------------------------
// Zeroing
// --------------------------------------------------------------

static uint64_t gnss_zero_ns = 0;

void clock_zero_all() {
  dwt_cycles_64 = 0;
  dwt_cycles_last = DWT_CYCCNT;

  gnss_10khz_ticks = 0;
  ocxo_10khz_ticks = 0;

  gnss_dwt_baseline = DWT_CYCCNT;
  ocxo_dwt_baseline = DWT_CYCCNT;

  // Record authoritative zero epoch
  gnss_zero_ns = clock_gnss_ns_now();
}

// --------------------------------------------------------------
// Zero-time accessor
// --------------------------------------------------------------

uint64_t clock_gnss_zero_ns(void) {
  return gnss_zero_ns;
}