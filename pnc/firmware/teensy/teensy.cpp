// teensy.cpp — ZPNet Runtime + Morse Fault Annunciator
//
// Fault reporting is LED-only, allocator-free, and unconditional.
// No logging, no transport, no heap, no scheduler dependency.
//

#include "config.h"
#include "memory_info.h"
#include "timepop.h"
#include "events.h"
#include "cpu_usage.h"
#include "process.h"
#include "transport.h"
#include "payload.h"
#include "debug.h"

#include "process_clocks.h"
#include "process_events.h"
#include "process_timepop.h"
#include "process_laser.h"
#include "process_photodiode.h"
#include "process_system.h"
#include "process_pubsub.h"
#include "process_performance.h"

#include <Arduino.h>
#include <Wire.h>

// ============================================================================
// DWT CLOCK DETERMINISM CONFIGURATION
// ============================================================================
//
// Goal: maximize DWT cycle counter stability for precision timing.
//
// 1. Overclock to 1.008 GHz — ~1 ns per DWT tick, maximum resolution.
//    The Teensy 4.1 (i.MX RT1062) supports this with adequate cooling.
//    set_arm_clock() reconfigures PLL1 and adjusts DCDC voltage.
//
// 2. Disable all ARM sleep modes — prevent core clock gating.
//    SLEEPONEXIT and SLEEPDEEP in SCB_SCR can gate the core clock
//    between interrupts, introducing DWT counter stalls.
//
// 3. F_CPU_ACTUAL is updated automatically by set_arm_clock().
//    All downstream code that uses F_CPU_ACTUAL will pick up the
//    new frequency.  Hardcoded 600 MHz constants (e.g. in
//    process_clocks.cpp) must be updated separately.
//
// THERMAL NOTE:
//    1.008 GHz + heatsink = safe continuous operation.
//    CPU temperature is monitored via SYSTEM.REPORT (tempmonGetTemp).
//
// DWT NS CONVERSION NOTE:
//    At 600 MHz: 1 cycle = 5/3 ns (exact)
//    At 1008 MHz: 1 cycle = 125/126 ns (exact rational, ~0.9921 ns)
//
//    The existing (cycles * 5) / 3 conversion in process_clocks.cpp
//    MUST be updated.  See DWT_NS_NUM / DWT_NS_DEN in config.h.
//
// ============================================================================

// Declared in Teensyduino core — reconfigures PLL1 + DCDC voltage
extern "C" uint32_t set_arm_clock(uint32_t frequency);

static void maximize_dwt_determinism(void) {

  // ----------------------------------------------------------
  // 1. Overclock to 1.008 GHz
  //
  // set_arm_clock() handles:
  //   • PLL1 (ARM_PLL) reconfiguration
  //   • DCDC target voltage increase for stable 1 GHz operation
  //   • F_CPU_ACTUAL global variable update
  //   • AHB/IPG clock divider adjustment
  //
  // The DWT cycle counter (DWT_CYCCNT) ticks at the core clock
  // rate, so this immediately gives us ~1 ns resolution.
  // ----------------------------------------------------------

  set_arm_clock(1008000000);

  // ----------------------------------------------------------
  // 2. Disable ARM sleep modes
  //
  // The Cortex-M7 System Control Register (SCR) has two bits
  // that can gate the core clock:
  //
  //   SLEEPONEXIT — enter sleep automatically on ISR return
  //   SLEEPDEEP   — enter deep sleep on WFI/WFE
  //
  // When the core clock is gated, DWT_CYCCNT stops counting.
  // This introduces non-deterministic gaps in the cycle counter
  // that appear as timing jitter.
  //
  // Clearing both bits ensures the core clock runs continuously.
  // This wastes power but that's irrelevant for our use case.
  // ----------------------------------------------------------

  SCB_SCR &= ~(SCB_SCR_SLEEPONEXIT | SCB_SCR_SLEEPDEEP);

  // ----------------------------------------------------------
  // 3. Verify DWT is enabled and running
  //
  // This is belt-and-suspenders — process_clocks_init_hardware()
  // also enables DWT.  But we want the counter running from the
  // earliest possible moment.
  // ----------------------------------------------------------

  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
}

// ============================================================================
// LED MORSE PRIMITIVES (FAULT-SAFE, DWT-BASED)
// ============================================================================
//
// These use DWT cycle-counting for timing, NOT delay()/millis().
// This is critical because:
//   • Hard fault handlers run with interrupts disabled
//   • SysTick ISR cannot fire → millis() is frozen → delay() hangs
//   • DWT cycle counter runs regardless of interrupt state
//

static constexpr uint32_t DOT_MS  = 150;
static constexpr uint32_t DASH_MS = 450;
static constexpr uint32_t GAP_MS  = 150;
static constexpr uint32_t LETTER_GAP_MS = 600;
static constexpr uint32_t REPEAT_GAP_MS = 1500;

static inline void led_on()  { digitalWrite(LED_BUILTIN, HIGH); }
static inline void led_off() { digitalWrite(LED_BUILTIN, LOW);  }

// DWT busy-wait: works with interrupts disabled, at any clock speed
static void dwt_spin_ms(uint32_t ms) {
  if (ms == 0) return;

  // Ensure DWT is running (may be called before maximize_dwt_determinism)
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  uint32_t freq = F_CPU_ACTUAL;
  if (freq == 0) freq = 600000000UL;

  uint32_t cycles_per_ms = freq / 1000;

  for (uint32_t i = 0; i < ms; i++) {
    uint32_t start = ARM_DWT_CYCCNT;
    while ((ARM_DWT_CYCCNT - start) < cycles_per_ms) {
      // spin
    }
  }
}

static void dot() {
  led_on();  dwt_spin_ms(DOT_MS);
  led_off(); dwt_spin_ms(GAP_MS);
}

static void dash() {
  led_on();  dwt_spin_ms(DASH_MS);
  led_off(); dwt_spin_ms(GAP_MS);
}

// ============================================================================
// MORSE FAULT PATTERNS
// ============================================================================

[[noreturn]]
static void fault_morse_hardfault() {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    dot(); dot(); dot(); dot();          // H
    dwt_spin_ms(REPEAT_GAP_MS);
  }
}

[[noreturn]]
static void fault_morse_memmanage() {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    dash(); dash();                      // M
    dwt_spin_ms(REPEAT_GAP_MS);
  }
}

[[noreturn]]
static void fault_morse_busfault() {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    dash(); dot(); dot(); dot();         // B
    dwt_spin_ms(REPEAT_GAP_MS);
  }
}

[[noreturn]]
static void fault_morse_usagefault() {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    dot(); dot(); dash();                // U
    dwt_spin_ms(REPEAT_GAP_MS);
  }
}

// ============================================================================
// ARM CORTEX-M FAULT HANDLERS
// ============================================================================

extern "C" {

void HardFault_Handler(void)   { fault_morse_hardfault(); }
void MemManage_Handler(void)  { fault_morse_memmanage(); }
void BusFault_Handler(void)   { fault_morse_busfault(); }
void UsageFault_Handler(void) { fault_morse_usagefault(); }

} // extern "C"

// ============================================================================
// ZPNet Runtime Initialization
// ============================================================================
//
// Boot order is critical.  The dependency chain is:
//
//   1. maximize_dwt_determinism()     — CPU clock, DWT counter
//   2. memory_info_init()             — stack sentinel painting
//   3. cpu_usage_init()               — DWT accounting baseline
//   4. process_clocks_init_hardware() — starts GPT2 (GNSS 10 MHz),
//                                       GPT1 (OCXO), DWT
//   5. timepop_init()                 — installs GPT2 OCR1 ISR
//                                       (GPT2 MUST be running)
//   6. transport_init()               — arms RX/TX timers
//                                       (TimePop MUST be ready)
//   7. debug_init()                   — transport-routed logging
//   8. process framework + subsystems — everything else
//
// ============================================================================

void setup() {

  // ----------------------------------------------------------
  // Phase 0: Pre-transport boot (NO DEBUG LOGGING ALLOWED)
  // ----------------------------------------------------------

  delay(100);            // USB settle (not relied upon for faults)

  // *** DWT DETERMINISM: overclock + disable sleep FIRST ***
  // This must happen before ANY timing-sensitive initialization.
  // set_arm_clock() adjusts PLL1 and DCDC voltage.
  // All subsequent code runs at 1.008 GHz.
  maximize_dwt_determinism();

  debug_blink("911");    // unconditional startup indicator (LED-only)

  pinMode(LED_BUILTIN, OUTPUT);
  led_off();

  // ----------------------------------------------------------
  // Get I2C devices working
  // ----------------------------------------------------------

  Wire.begin();

  // ----------------------------------------------------------
  // Phase 1: Core instrumentation + hardware clocks
  // ----------------------------------------------------------

  memory_info_init();
  cpu_usage_init();

  // Start GPT2/GPT1/DWT hardware BEFORE TimePop.
  // TimePop uses GPT2 output compare — the counter must be
  // running before timepop_init() installs its ISR.
  process_clocks_init_hardware();

  // Now TimePop can safely install its OCR1 compare on GPT2.
  timepop_init();

  // ----------------------------------------------------------
  // Phase 2: Transport boundary comes alive
  // ----------------------------------------------------------

  transport_register_receive_callback(
    TRAFFIC_REQUEST_RESPONSE,
    process_command
  );

  transport_register_receive_callback(
    TRAFFIC_PUBLISH_SUBSCRIBE,
    process_publish_dispatch
  );

  transport_init();

  // ----------------------------------------------------------
  // Phase 3: Debug subsystem becomes valid (transport-routed)
  // ----------------------------------------------------------

  debug_init();

  debug_log("boot", "setup begin");

  // Log clock configuration for forensic verification
  debug_log("boot.cpu_mhz", (uint32_t)(F_CPU_ACTUAL / 1000000UL));

  // ----------------------------------------------------------
  // Process framework
  // ----------------------------------------------------------

  debug_log("boot", "process_init");
  process_init();
  debug_log("boot", "process_init done");

  // ----------------------------------------------------------
  // Events subsystem
  // ----------------------------------------------------------

  debug_log("boot", "process_events_init");
  process_events_init();
  debug_log("boot", "process_events_init done");

  debug_log("boot", "process_events_register");
  process_events_register();
  debug_log("boot", "process_events_register done");

  // ----------------------------------------------------------
  // Boot event
  // ----------------------------------------------------------

  debug_log("boot", "enqueue TEENSY_BOOT");
  Payload ev;
  ev.add("status", "READY");
  ev.add("cpu_mhz", (uint32_t)(F_CPU_ACTUAL / 1000000UL));
  enqueueEvent("TEENSY_BOOT", ev);
  debug_log("boot", "enqueue TEENSY_BOOT done");

  // ----------------------------------------------------------
  // Process registration
  // ----------------------------------------------------------

  debug_log("boot", "process_timepop_register");
  process_timepop_register();
  debug_log("boot", "process_timepop_register done");

  debug_log("boot", "process_clocks_init");
  process_clocks_init();
  debug_log("boot", "process_clocks_init done");

  debug_log("boot", "process_clocks_register");
  process_clocks_register();
  debug_log("boot", "process_clocks_register done");

  debug_log("boot", "process_laser_init");
  process_laser_init();
  debug_log("boot", "process_laser_init done");

  debug_log("boot", "process_laser_register");
  process_laser_register();
  debug_log("boot", "process_laser_register done");

  debug_log("boot", "process_photodiode_init");
  process_photodiode_init();
  debug_log("boot", "process_photodiode_init done");

  debug_log("boot", "process_photodiode_register");
  process_photodiode_register();
  debug_log("boot", "process_photodiode_register done");

  debug_log("boot", "process_pubsub_register");
  process_pubsub_register();
  debug_log("boot", "process_pubsub_register done");

  debug_log("boot", "process_system_register");
  process_system_register();
  debug_log("boot", "process_system_register done");

  debug_log("boot", "process_performance_register");
  process_performance_register();
  debug_log("boot", "process_performance_register done");

  // ----------------------------------------------------------
  // CPU usage sampling
  // ----------------------------------------------------------

  debug_log("boot", "cpu_usage_init_timer");
  cpu_usage_init_timer();
  debug_log("boot", "cpu_usage_init_timer done");

  debug_log("boot", "setup complete");
}

// ============================================================================
// Runtime loop
// ============================================================================

void loop() {
  timepop_dispatch();
}