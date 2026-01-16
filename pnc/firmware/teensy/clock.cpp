#include "clock.h"
#include "event_bus.h"
#include <Arduino.h>
#include <IntervalTimer.h>

// --------------------------------------------------------------
// DWT Registers (core-local, USB-safe)
// --------------------------------------------------------------
#define DEMCR               (*(volatile uint32_t *)0xE000EDFC)
#define DEMCR_TRCENA        (1 << 24)

#define DWT_CTRL            (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT          (*(volatile uint32_t *)0xE0001004)
#define DWT_CTRL_CYCCNTENA  (1 << 0)

// --------------------------------------------------------------
// GPT1 / GPT2 Hardware Counter Reads
// --------------------------------------------------------------
//#define GPT1_CNT (*(volatile uint32_t *)0x401EC010)  // OCXO (REF)
//#define GPT2_CNT (*(volatile uint32_t *)0x401EC090)  // GNSS (VCLOCK)

// --------------------------------------------------------------
// Internal synthetic time state
// --------------------------------------------------------------

static uint64_t dwt_acc     = 0;
static uint32_t dwt_last    = 0;

static uint64_t gnss_acc    = 0;
static uint32_t gnss_last   = 0;

static uint64_t ocxo_acc    = 0;
static uint32_t ocxo_last   = 0;

// --------------------------------------------------------------
// DWT Guard Timer (auto-pumps every 500 ms)
// --------------------------------------------------------------

static IntervalTimer dwt_guard_timer;

static void dwt_guard_tick() {
  (void)dwt_now();
}

// --------------------------------------------------------------
// Initialization
// --------------------------------------------------------------

void dwt_init() {
  DEMCR |= DEMCR_TRCENA;
  DWT_CYCCNT = 0;
  DWT_CTRL |= DWT_CTRL_CYCCNTENA;
  dwt_last = DWT_CYCCNT;

  // Start background pumping every 500ms
  dwt_guard_timer.begin(dwt_guard_tick, 2000000);  // 2 seconds
}

// GNSS initialization stub
/*
void gnss_init() {
  gnss_last = GPT2_CNT;
}
*/

// OCXO initialization (used)
void ocxo_init() {
  ocxo_last = GPT1_CNT;
}

// --------------------------------------------------------------
// Query (with implicit pump)
// --------------------------------------------------------------

uint64_t dwt_now() {
  uint32_t now = DWT_CYCCNT;
  uint32_t delta = now - dwt_last;
  dwt_acc += delta;
  dwt_last = now;
  return dwt_acc;
}

uint64_t gnss_now() {
  uint32_t now = GPT2_CNT;
  uint32_t delta = now - gnss_last;
  gnss_acc += delta;
  gnss_last = now;
  return gnss_acc;
}

uint64_t ocxo_now() {
  uint32_t now = GPT1_CNT;
  uint32_t delta = now - ocxo_last;
  ocxo_acc += delta;
  ocxo_last = now;
  return ocxo_acc;
}

// --------------------------------------------------------------
// Optional zeroing
// --------------------------------------------------------------

void dwt_zero()   { dwt_acc = 0; }
void gnss_zero()  { gnss_acc = 0; }
void ocxo_zero()  { ocxo_acc = 0; }
