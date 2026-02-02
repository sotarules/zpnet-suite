// teensy.cpp — ZPNet Runtime + Morse Fault Annunciator
//
// Fault reporting is LED-only, allocator-free, and unconditional.
// No logging, no transport, no heap, no scheduler dependency.
//

#include "config.h"

#include "clock.h"
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
#include "process_tempest.h"
#include "process_system.h"
#include "process_pubsub.h"

#include <Arduino.h>

// ============================================================================
// LED MORSE PRIMITIVES (FAULT-SAFE)
// ============================================================================

static constexpr uint32_t DOT_MS  = 150;
static constexpr uint32_t DASH_MS = 450;
static constexpr uint32_t GAP_MS  = 150;
static constexpr uint32_t LETTER_GAP_MS = 600;
static constexpr uint32_t REPEAT_GAP_MS = 1500;

static inline void led_on()  { digitalWrite(LED_BUILTIN, HIGH); }
static inline void led_off() { digitalWrite(LED_BUILTIN, LOW);  }

static void dot() {
  led_on();  delay(DOT_MS);
  led_off(); delay(GAP_MS);
}

static void dash() {
  led_on();  delay(DASH_MS);
  led_off(); delay(GAP_MS);
}

// ============================================================================
// MORSE FAULT PATTERNS
// ============================================================================

[[noreturn]]
static void fault_morse_hardfault() {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    dot(); dot(); dot(); dot();          // H
    delay(REPEAT_GAP_MS);
  }
}

[[noreturn]]
static void fault_morse_memmanage() {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    dash(); dash();                      // M
    delay(REPEAT_GAP_MS);
  }
}

[[noreturn]]
static void fault_morse_busfault() {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    dash(); dot(); dot(); dot();         // B
    delay(REPEAT_GAP_MS);
  }
}

[[noreturn]]
static void fault_morse_usagefault() {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    dot(); dot(); dash();                // U
    delay(REPEAT_GAP_MS);
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

void setup() {

  // ----------------------------------------------------------
  // Phase 0: Pre-transport boot (NO DEBUG LOGGING ALLOWED)
  // ----------------------------------------------------------

  delay(100);            // USB settle (not relied upon for faults)
  debug_blink("911");    // unconditional startup indicator (LED-only)

  pinMode(LED_BUILTIN, OUTPUT);
  led_off();

  // ----------------------------------------------------------
  // Phase 1: Core instrumentation (still no debug_log)
  // ----------------------------------------------------------

  cpu_usage_init();
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

  // ----------------------------------------------------------
  // Clock subsystem
  // ----------------------------------------------------------

  debug_log("boot", "clock_init");
  clock_init();
  debug_log("boot", "clock_init done");

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
  enqueueEvent("TEENSY_BOOT", ev);
  debug_log("boot", "enqueue TEENSY_BOOT done");

  // ----------------------------------------------------------
  // Process registration
  // ----------------------------------------------------------

  debug_log("boot", "process_timepop_register");
  process_timepop_register();
  debug_log("boot", "process_timepop_register done");

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

  debug_log("boot", "process_tempest_register");
  process_tempest_register();
  debug_log("boot", "process_tempest_register done");

  debug_log("boot", "process_pubsub_register");
  process_pubsub_register();
  debug_log("boot", "process_pubsub_register done");

  debug_log("boot", "process_system_register");
  process_system_register();
  debug_log("boot", "process_system_register done");


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