#include "gpt_count.h"
#include "imxrt.h"

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

  // Enable GPT2 (do once, never disable)
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

uint32_t gpt_count_read() {
  if (!gpt2_armed) return 0;
  return GPT2_CNT;
}

bool gpt_count_status() {
  return gpt2_armed;
}
