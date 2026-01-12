#include "qtimer.h"
#include "imxrt.h"

// --------------------------------------------------------------
// Internal state
// --------------------------------------------------------------
static volatile bool qtimer_armed = false;

// --------------------------------------------------------------
// Clock gate helper
//
// QTIMER3 lives in CCM_CCGR6
// --------------------------------------------------------------
static inline void enable_qtimer_clock_gate() {
  CCM_CCGR6 |= CCM_CCGR6_QTIMER3(CCM_CCGR_ON);
}

// --------------------------------------------------------------
// Pad muxing for QTIMER3_TIMER1
//
// Pad: GPIO_AD_B1_09 (Teensy pin 25)
// ALT3 = QTIMER3_TIMER1
// --------------------------------------------------------------
static inline void qtimer_configure_input_pin() {
  // Select ALT3 = QTIMER3_TIMER1
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_09 = 3;

  // Electrical characteristics:
  //  - Input enabled
  //  - Hysteresis for clean edges
  //  - Pull-up to define idle when floating
  //  - Medium drive strength (safe default)
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_09 =
      IOMUXC_PAD_HYS |
      IOMUXC_PAD_PKE |
      IOMUXC_PAD_PUE |
      IOMUXC_PAD_DSE(4);
}

// --------------------------------------------------------------
// Low-level safe arm
//
// QTIMER is explicitly designed for external counting.
// No shared core usage.
// No catastrophic side effects.
// --------------------------------------------------------------
static inline void qtimer_safe_arm() {
  // Disable timer (clear count mode)
  TMR3_CTRL1 &= ~0x7;   // CM = 0 (disabled)

  // Clear counter
  TMR3_CNTR1 = 0;

  // Free-running compare values
  TMR3_COMP11 = 0xFFFF;
  TMR3_COMP21 = 0xFFFF;

  // Reset control/status
  TMR3_SCTRL1 = 0;
  TMR3_CSCTRL1 = 0;

  // Configure:
  // CM  = 1 → count rising edges
  // PCS = 4 → external pin input (QTIMER3_TIMER1)
  TMR3_CTRL1 =
      TMR_CTRL_CM(1) |
      TMR_CTRL_PCS(4);
}

// --------------------------------------------------------------
// Safe disarm
// --------------------------------------------------------------
static inline void qtimer_safe_disarm() {
  TMR3_CTRL1 &= ~0x7;   // Clear CM field → disabled
}

// --------------------------------------------------------------
// Public API
// --------------------------------------------------------------
void qtimer_arm() {
  if (qtimer_armed) return;

  enable_qtimer_clock_gate();
  qtimer_configure_input_pin();
  qtimer_safe_arm();

  qtimer_armed = true;
}

void qtimer_disarm() {
  if (!qtimer_armed) return;

  qtimer_safe_disarm();
  qtimer_armed = false;
}

uint32_t qtimer_read() {
  if (!qtimer_armed) return 0;
  return TMR3_CNTR1;
}

void qtimer_clear() {
  if (!qtimer_armed) return;

  // Disable counting
  TMR3_CTRL1 &= ~0x7;   // CM = 0

  // Clear counter
  TMR3_CNTR1 = 0;

  // Re-enable edge counting on external pin
  TMR3_CTRL1 =
      TMR_CTRL_CM(1) |
      TMR_CTRL_PCS(4);
}

bool qtimer_status() {
  return qtimer_armed;
}
