#include "gpt_count.h"
#include "imxrt.h"
#include "dwt_clock.h"

#include <Arduino.h>

// --------------------------------------------------------------
// Internal state
// --------------------------------------------------------------
static volatile bool gpt2_armed = false;

// --------------------------------------------------------------
// Clock gating
//
// GPT2 requires only the BUS clock for external counting.
// SERIAL clock is intentionally omitted.
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

uint32_t gpt_count_confirm(
    uint32_t window_cpu_cycles,
    uint32_t* cpu_cycles_out,
    float* ratio_out
) {
  // Defensive defaults
  if (cpu_cycles_out) *cpu_cycles_out = 0;
  if (ratio_out)      *ratio_out = 0.0f;

  // Ensure DWT cycle counter is enabled
  dwt_clock_init();

  // Arm GPT2 (idempotent, owns setup)
  gpt_count_arm();

  noInterrupts();

  uint32_t start_cpu = dwt_clock_read();
  uint32_t start_ext = GPT2_CNT;

  // Busy-wait until CPU-cycle window elapses
  while ((uint32_t)(dwt_clock_read() - start_cpu) < window_cpu_cycles) {
    // intentional spin — no sleeping, no SysTick
  }

  uint32_t end_cpu = dwt_clock_read();
  uint32_t end_ext = GPT2_CNT;

  interrupts();

  // Disarm via public API (respect ownership boundary)
  gpt_count_disarm();

  // Compute deltas
  uint32_t delta_cpu = end_cpu - start_cpu;
  uint32_t delta_ext = end_ext - start_ext;

  if (cpu_cycles_out) {
    *cpu_cycles_out = delta_cpu;
  }

  if (ratio_out && delta_cpu > 0) {
    *ratio_out = (float)delta_ext / (float)delta_cpu;
  }

  return delta_ext;
}

