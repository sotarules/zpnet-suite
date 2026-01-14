#include <Arduino.h>
#include <stddef.h>

#include "imxrt.h"
#include "config.h"
#include "gpt_count.h"
#include "dwt_clock.h"
#include "event_bus.h"

// =============================================================
// INTERNAL STATE (minimal, explicit)
// =============================================================

static volatile bool start_captured = false;
static volatile bool end_captured   = false;

static volatile uint32_t start_cycles = 0;
static volatile uint32_t end_cycles   = 0;

// =============================================================
// ISR: START edge (GPIO ownership)
// =============================================================
static void gpt_confirm_start_isr() {
  start_cycles = dwt_clock_read();
  start_captured = true;
  detachInterrupt(GNSS_VCLK_PIN);
  enqueueEvent("GPT_ISR_START", "\"note\":\"start edge observed\"");
}

// =============================================================
// ISR: END edge (GPIO ownership)
// =============================================================
static void gpt_confirm_end_isr() {
  end_cycles = dwt_clock_read();
  end_captured = true;
  detachInterrupt(GNSS_VCLK_PIN);
  enqueueEvent("GPT_ISR_END", "\"note\":\"end edge observed\"");
}

// =============================================================
// CONFIRM — single-shot GNSS ↔ CPU coherence sentinel
// =============================================================
uint32_t gpt_count_confirm(
    uint32_t target_ext_ticks,
    uint32_t* cpu_cycles_out,
    float*    ratio_out
) {
  // -----------------------------------------------------------
  // Defensive defaults
  // -----------------------------------------------------------
  if (cpu_cycles_out) *cpu_cycles_out = 0;
  if (ratio_out)      *ratio_out = 0.0f;

  // -----------------------------------------------------------
  // Enable CPU cycle counter (observer only)
  // -----------------------------------------------------------
  dwt_clock_init();

  start_captured = false;
  end_captured   = false;

  // ===========================================================
  // PHASE 0 — capture START edge as GPIO interrupt
  // ===========================================================
  attachInterrupt(GNSS_VCLK_PIN, gpt_confirm_start_isr, RISING);
  while (!start_captured) {
    // tight spin — waiting for GNSS edge
  }

  // ===========================================================
  // PHASE 1 — take ownership of pin as GPT2 external clock
  // ===========================================================

  // Enable GPT2 BUS clock
  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON);

  // Mux GNSS_VCLK_PIN (GPIO_AD_B1_02) → GPT2_CLK (ALT8)
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 8;

  // Conservative pad control
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 =
      IOMUXC_PAD_HYS |
      IOMUXC_PAD_PKE |
      IOMUXC_PAD_PUE |
      IOMUXC_PAD_DSE(4);

  // Explicit input select for GPT2 clock
  IOMUXC_GPT2_IPP_IND_CLKIN_SELECT_INPUT = 1;

  // Allow mux + pad logic to settle
  delayMicroseconds(1);

  // ===========================================================
  // PHASE 2 — initialize GPT2 for external counting
  // ===========================================================

  GPT2_CR = 0;          // disable before config
  GPT2_SR = 0x3F;       // clear all status flags

  // CLKSRC = 3 → external clock
  GPT2_CR = GPT_CR_CLKSRC(3);

  // Enable GPT2
  GPT2_CR |= GPT_CR_EN;

  // Capture baseline (DO NOT WRITE GPT2_CNT)
  uint32_t gpt_start = GPT2_CNT;

  {
    String body;
    body += "\"gpt_start\":";
    body += gpt_start;
    enqueueEvent("GPT_CNT_SEEN", body);
  }

  // ===========================================================
  // PHASE 3 — wait for external tick delta
  // ===========================================================
  while ((uint32_t)(GPT2_CNT - gpt_start) < target_ext_ticks) {
    // tight spin — external clock is the authority
  }

  enqueueEvent("GPT_CNT_DONE", "\"ok\":true");

  // ===========================================================
  // PHASE 4 — restore pin to GPIO ownership
  // ===========================================================
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 5; // ALT5 = GPIO

  enqueueEvent("GPT_GPIO_PIN_RESTORED", "\"ok\":true");

  delayMicroseconds(1);

  // Disable GPT2
  GPT2_CR &= ~GPT_CR_EN;

  enqueueEvent("GPT_COUNT_DISARMED", "\"ok\":true");

  // ===========================================================
  // PHASE 5 — capture END edge as GPIO interrupt
  // ===========================================================
  attachInterrupt(GNSS_VCLK_PIN, gpt_confirm_end_isr, RISING);
  while (!end_captured) {
    // tight spin
  }

  enqueueEvent("GPT_FINAL COMPUTATIONS", "\"ok\":true");

  // ===========================================================
  // PHASE 6 — compute delta and ratio
  // ===========================================================
  uint32_t delta_cpu = end_cycles - start_cycles;

  if (cpu_cycles_out) {
    *cpu_cycles_out = delta_cpu;
  }

  if (ratio_out && delta_cpu > 0) {
    *ratio_out = (float)target_ext_ticks / (float)delta_cpu;
  }

  enqueueEvent("GPT_COUNT_CONFIRM", "\"complete\":true");

  return target_ext_ticks;
}
