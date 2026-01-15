#include "dwt_clock.h"
#include <Arduino.h>

// --------------------------------------------------------------
// Cortex-M7 DWT registers (core-local, USB-safe)
// --------------------------------------------------------------
#define DEMCR            (*(volatile uint32_t *)0xE000EDFC)
#define DEMCR_TRCENA     (1 << 24)

#define DWT_CTRL         (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT       (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL_CYCCNTENA (1 << 0)

// --------------------------------------------------------------
// Initialize DWT cycle counter
// --------------------------------------------------------------
void dwt_clock_init() {
  // Enable trace and debug block
  DEMCR |= DEMCR_TRCENA;

  // Reset cycle counter
  DWT_CYCCNT = 0;

  // Enable cycle counter
  DWT_CTRL |= DWT_CTRL_CYCCNTENA;
}

// --------------------------------------------------------------
// Read current cycle count
// --------------------------------------------------------------
uint32_t dwt_clock_read() {
  return DWT_CYCCNT;
}
