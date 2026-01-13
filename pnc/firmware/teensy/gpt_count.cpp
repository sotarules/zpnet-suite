#include <stddef.h>
#include "imxrt.h"
#include "config.h"
#include "gpt_count.h"
#include "dwt_clock.h"
#include "event_bus.h"

#include <Arduino.h>

// --------------------------------------------------------------
// Internal state
// --------------------------------------------------------------
static volatile bool gpt2_armed = false;

volatile bool start_captured = false;
volatile bool end_captured   = false;

volatile uint32_t start_cycles = 0;
volatile uint32_t end_cycles   = 0;

// --------------------------------------------------------------
// Clock gating
//
// GPT2 requires only the BUS clock for external counting.
// --------------------------------------------------------------
static inline void enable_gpt2_clock_gate() {
  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON);
}

// --------------------------------------------------------------
// Pad mux + input select
//
// External pin 14:
//   GPIO_AD_B1_02
//   ALT8  -> GPT2_CLK
//
// NOTE:
//   Input select MUST be configured explicitly.
// --------------------------------------------------------------
static inline void gpt2_configure_external_clock_pin() {
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
}

static inline void gpt2_restore_gpio_pin() {
  // Mux GNSS_VCLK_PIN back to GPIO
  // Example (adjust ALT as per your SoC):
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 5; // ALT5 = GPIO
}

// --------------------------------------------------------------
// GPT2 initialization (external clock, read-only counter)
//
// IMPORTANT INVARIANT:
//   GPT2_CNT MUST NEVER be written in external clock mode.
// --------------------------------------------------------------
static inline void gpt2_init_external_clock() {
  // Disable GPT2 before configuration
  GPT2_CR = 0;

  // Clear all status flags
  GPT2_SR = 0x3F;

  // Select external clock source
  // CLKSRC = 3 -> external
  GPT2_CR = GPT_CR_CLKSRC(3);

  // Enable GPT2
  GPT2_CR |= GPT_CR_EN;
}

// --------------------------------------------------------------
// Public API
// --------------------------------------------------------------
void gpt_count_arm() {
  if (gpt2_armed) return;

  enable_gpt2_clock_gate();
  gpt2_configure_external_clock_pin();
  gpt2_init_external_clock();

  gpt2_armed = true;
}

void gpt_count_disarm() {
  if (!gpt2_armed) return;

  // Disable counting, leave configuration intact
  GPT2_CR &= ~GPT_CR_EN;

  gpt2_armed = false;
}

uint32_t gpt_count_read() {
  if (!gpt2_armed) return 0;
  return GPT2_CNT;
}

bool gpt_count_status() {
  return gpt2_armed;
}

static void gpt_confirm_start_isr() {
  start_cycles = dwt_clock_read();
  start_captured = true;
  detachInterrupt(GNSS_VCLK_PIN);
  enqueueEvent("GPT_ISR_START", "\"note\":\"start edge observed\"");
}

static void gpt_confirm_end_isr() {
  end_cycles = dwt_clock_read();
  end_captured = true;
  detachInterrupt(GNSS_VCLK_PIN);
  enqueueEvent("GPT_ISR_END", "\"note\":\"end edge observed\"");
}

uint32_t gpt_count_confirm(
    uint32_t target_ext_ticks,   // still unused
    uint32_t* cpu_cycles_out,
    float* ratio_out
) {
  // Defensive defaults
  if (cpu_cycles_out) *cpu_cycles_out = 0;
  if (ratio_out)      *ratio_out = 0.0f;

  // Enable cycle counter (observer only)
  dwt_clock_init();

  // Reset visible state
  start_captured = false;
  end_captured = false;

  // ------------------------------------------------------------
  // PHASE 0: GPIO start edge (proven working)
  // ------------------------------------------------------------
  attachInterrupt(GNSS_VCLK_PIN, gpt_confirm_start_isr, RISING);

  while (!start_captured) {
    // spin until first edge observed
  }

  // Optional: report captured cycle count
  if (cpu_cycles_out) {
    *cpu_cycles_out = start_cycles;
  }

  // ------------------------------------------------------------
  // PHASE 1: Remux pin to GPT and see if counter ticks
  // ------------------------------------------------------------

  // Take explicit ownership of the pin as GPT clock
  gpt2_configure_external_clock_pin();

  // Allow pad logic to settle
  delayMicroseconds(1);

  // Arm GPT (counter starts at 0)
  gpt_count_arm();

  // Capture baseline (DO NOT WRITE GPT2_CNT)
  uint32_t gpt_start = GPT2_CNT;

  {
    String body;
    body += "\"gpt_start\":";
    body += gpt_start;
    enqueueEvent("GPT_CNT_SEEN", body);
  }

  // Give it a very short observation window
  delayMicroseconds(10);

  // Wait for delta, not absolute value
  while ((uint32_t)(GPT2_CNT - gpt_start) < target_ext_ticks) { }

  enqueueEvent("GPT_CNT_DONE", "\"ok\":true");

  gpt2_restore_gpio_pin();

  enqueueEvent("GPT_GPIO_PIN_RESTORED", "\"ok\":true");

  delayMicroseconds(1);

  gpt_count_disarm();

  enqueueEvent("GPT_COUNT_DISARMED", "\"ok\":true");

  attachInterrupt(GNSS_VCLK_PIN, gpt_confirm_end_isr, RISING);
  while (!end_captured) {
    // spin until second edge observed}
  }

  enqueueEvent("GPT_FINAL COMPUTATIONS", "\"ok\":true");

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



