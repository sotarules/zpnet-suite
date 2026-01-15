#include <Arduino.h>
#include <stddef.h>

#include "imxrt.h"
#include "config.h"
#include "gpt_count.h"
#include "dwt_clock.h"
#include "event_bus.h"

// =============================================================
// INTERNAL STATE
// =============================================================

// GNSS edge gates
static volatile bool start_edge_seen = false;
static volatile bool end_edge_seen   = false;

// Synthetic DWT state (64-bit)
static uint32_t dwt_last = 0;
static uint64_t dwt_acc  = 0;

// =============================================================
// SYNTHETIC DWT — MAINTENANCE PUMP
// =============================================================
// Must be called periodically (<< 7 seconds)

void dwt_pump(void) {
  uint32_t now = dwt_clock_read();
  uint32_t delta = now - dwt_last;   // wrap-safe
  dwt_acc += (uint64_t)delta;
  dwt_last = now;
}

// =============================================================
// SYNTHETIC DWT — READ (NO SIDE EFFECTS)
// =============================================================

static inline uint64_t dwt_read_64(void) {
  return dwt_acc;
}

// =============================================================
// ISR: START edge (phase gate only)
// =============================================================

static void gpt_confirm_start_isr() {
  start_edge_seen = true;
  detachInterrupt(GNSS_VCLK_PIN);
}

// =============================================================
// ISR: END edge (phase gate only)
// =============================================================

static void gpt_confirm_end_isr() {
  end_edge_seen = true;
  detachInterrupt(GNSS_VCLK_PIN);
}

// =============================================================
// CONFIRM — GNSS ↔ CPU coherence sentinel
// =============================================================

uint64_t gpt_count_confirm(
    uint64_t target_ext_ticks,
    uint64_t* cpu_cycles_out,
    double*    ratio_out,
    int64_t*   error_cycles_out
) {
  // -----------------------------------------------------------
  // Defensive defaults
  // -----------------------------------------------------------
  if (cpu_cycles_out)   *cpu_cycles_out   = 0;
  if (ratio_out)        *ratio_out        = 0.0;
  if (error_cycles_out) *error_cycles_out = 0;

  start_edge_seen = false;
  end_edge_seen   = false;

  // -----------------------------------------------------------
  // Ensure DWT is enabled and primed
  // -----------------------------------------------------------
  dwt_clock_init();

  // Prime synthetic counter once
  dwt_last = dwt_clock_read();
  dwt_acc  = 0;

  // ===========================================================
  // PHASE 0 — wait for START GNSS edge
  // ===========================================================
  attachInterrupt(GNSS_VCLK_PIN, gpt_confirm_start_isr, RISING);
  while (!start_edge_seen) {
    dwt_pump();   // keep DWT alive while waiting
  }

  uint64_t start_cycles = dwt_read_64();

  // ===========================================================
  // PHASE 1 — GNSS pin → GPT2 external clock
  // ===========================================================

  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON);

  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 8;   // ALT8 = GPT2_CLK
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 =
      IOMUXC_PAD_HYS |
      IOMUXC_PAD_PKE |
      IOMUXC_PAD_PUE |
      IOMUXC_PAD_DSE(4);

  IOMUXC_GPT2_IPP_IND_CLKIN_SELECT_INPUT = 1;

  delayMicroseconds(1);

  // ===========================================================
  // PHASE 2 — GPT2 external counting
  // ===========================================================

  GPT2_CR = 0;
  GPT2_SR = 0x3F;
  GPT2_CR = GPT_CR_CLKSRC(3);
  GPT2_CR |= GPT_CR_EN;

  uint32_t gpt_start = GPT2_CNT;

  while ((uint64_t)(GPT2_CNT - gpt_start) < target_ext_ticks) {
    dwt_pump();   // maintain synthetic DWT during long runs
  }

  // ===========================================================
  // PHASE 3 — restore GPIO + wait for END GNSS edge
  // ===========================================================

  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 5;   // ALT5 = GPIO
  delayMicroseconds(1);

  GPT2_CR &= ~GPT_CR_EN;

  attachInterrupt(GNSS_VCLK_PIN, gpt_confirm_end_isr, RISING);
  while (!end_edge_seen) {
    dwt_pump();   // keep synthetic DWT coherent
  }

  uint64_t end_cycles = dwt_read_64();

  // ===========================================================
  // PHASE 4 — compute results
  // ===========================================================

  uint64_t delta_cpu_cycles = end_cycles - start_cycles;

  if (cpu_cycles_out) {
    *cpu_cycles_out = delta_cpu_cycles;
  }

  if (ratio_out && delta_cpu_cycles > 0) {
    *ratio_out =
        (double)target_ext_ticks /
        (double)delta_cpu_cycles;
  }

  int64_t ideal_cpu_cycles =
      (int64_t)target_ext_ticks * 60;

  if (error_cycles_out) {
    *error_cycles_out =
        (int64_t)delta_cpu_cycles - ideal_cpu_cycles;
  }

  return target_ext_ticks;
}
