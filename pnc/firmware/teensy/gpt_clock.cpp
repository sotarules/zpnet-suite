#include "gpt_clock.h"

// ⚠️ Dangerous territory is isolated here on purpose
#include "imxrt.h"

// --------------------------------------------------------------
// Internal state
// --------------------------------------------------------------
static bool gpt_initialized = false;
static bool gpt_running     = false;

// --------------------------------------------------------------
// Initialize GPT1 in the safest possible configuration
// --------------------------------------------------------------
void gpt_clock_init() {
  if (gpt_initialized) return;

  // ------------------------------------------------------------
  // Enable clock gating for GPT (Teensy macro enables GPT1+GPT2)
  // ------------------------------------------------------------
  CCM_CCGR1 |= CCM_CCGR1_GPT(CCM_CCGR_ON);

  // ------------------------------------------------------------
  // Reset GPT1
  // ------------------------------------------------------------
  GPT1_CR = GPT_CR_SWR;
  while (GPT1_CR & GPT_CR_SWR) {
    // wait for reset to complete
  }

  // ------------------------------------------------------------
  // Configure GPT1
  //
  // - Free-running mode
  // - Enable mode after reset
  // - Internal peripheral clock (SAFE)
  // ------------------------------------------------------------
  GPT1_PR = 0;  // no prescaler
  GPT1_CR =
      GPT_CR_FRR |      // free-running
      GPT_CR_ENMOD |   // enable mode after reset
      (0 << 6);        // clock source = peripheral clock (SAFE)

  GPT1_CNT = 0;

  gpt_initialized = true;
}

// --------------------------------------------------------------
// Start counting
// --------------------------------------------------------------
void gpt_clock_start() {
  if (!gpt_initialized) return;
  if (gpt_running) return;

  GPT1_CNT = 0;
  GPT1_CR |= GPT_CR_EN;

  gpt_running = true;
}

// --------------------------------------------------------------
// Stop counting
// --------------------------------------------------------------
void gpt_clock_stop() {
  if (!gpt_running) return;

  GPT1_CR &= ~GPT_CR_EN;
  gpt_running = false;
}

// --------------------------------------------------------------
// Read current count
// --------------------------------------------------------------
uint32_t gpt_clock_read() {
  if (!gpt_initialized) return 0;
  return GPT1_CNT;
}
