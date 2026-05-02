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
#include "epoch.h"

#include "process_interrupt.h"
#include "process_clocks.h"
#include "process_time.h"
#include "process_events.h"
#include "process_timepop.h"
#include "process_laser.h"
#include "process_photodiode.h"
#include "process_system.h"
#include "process_pubsub.h"
#include "process_performance.h"
#include "process_witness.h"
#include "process_epoch.h"

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
//    new frequency.
//
// THERMAL NOTE:
//    1.008 GHz + heatsink = safe continuous operation.
//    CPU temperature is monitored via SYSTEM.REPORT (tempmonGetTemp).
//
// DWT NS CONVERSION NOTE:
//    At 600 MHz: 1 cycle = 5/3 ns (exact)
//    At 1008 MHz: 1 cycle = 125/126 ns (exact rational, ~0.9921 ns)
//
// ============================================================================

// Declared in Teensyduino core — reconfigures PLL1 + DCDC voltage
extern "C" uint32_t set_arm_clock(uint32_t frequency);

static void maximize_dwt_determinism(void) {

  // ----------------------------------------------------------
  // 1. Overclock to 1.008 GHz
  // ----------------------------------------------------------
  set_arm_clock(1008000000);

  // ----------------------------------------------------------
  // 2. Disable ARM sleep modes
  // ----------------------------------------------------------
  SCB_SCR &= ~(SCB_SCR_SLEEPONEXIT | SCB_SCR_SLEEPDEEP);

  // ----------------------------------------------------------
  // 3. Verify DWT is enabled and running
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

void HardFault_Handler(void)  { fault_morse_hardfault(); }
void MemManage_Handler(void)  { fault_morse_memmanage(); }
void BusFault_Handler(void)   { fault_morse_busfault(); }
void UsageFault_Handler(void) { fault_morse_usagefault(); }

} // extern "C"

// ============================================================================
// ZPNet Runtime Initialization
// ============================================================================
//
// Boot order is critical. The dependency chain is:
//
//   1. maximize_dwt_determinism()        — CPU clock, DWT counter
//   2. memory_info_init()                — stack sentinel painting
//   3. cpu_usage_init()                  — DWT accounting baseline
//   4. process_interrupt_init_hardware() — QTimer1 CH0/CH1/CH2/CH3 +
//                                          QTimer3 CH2/CH3 hardware setup
//                                          (all compare channels start
//                                          disabled; no spurious ISRs)
//   5. timepop_init()                    — slot tables + register
//                                          QTimer1 CH2 handler with
//                                          process_interrupt
//   6. process_clocks_init_hardware()    — DWT enable, rolling helper baselines
//   7. transport_init()                  — arms RX/TX timers
//   8. debug_init()                      — transport-routed logging
//   9. process framework + subsystems    — everything else
//  10. process_interrupt_init()          — runtime subscriber tables
//  11. process_interrupt_enable_irqs()   — ISR vectors + NVIC enable;
//                                          IRQ_QTIMER1, IRQ_QTIMER3,
//                                          IRQ_GPIO6789 all go live.
//                                          Any pending CH2 TCF1 from
//                                          step 5's schedule_next()
//                                          fires immediately and
//                                          dispatches to TimePop.
//  12. process_clocks_init()             — subscribes VCLOCK/OCXO,
//                                          registers PPS edge dispatch,
//                                          requests startup epoch zero,
//                                          starts providers (which arms
//                                          their compare channels)
//
// Sacred ordering invariants (post QTimer1 unification refactor):
//
//   • process_interrupt_init_hardware() must run before timepop_init()
//     because timepop_init() programs QTimer1 CH2's compare register
//     and assumes the channel hardware is already initialized.
//
//   • timepop_init() must call interrupt_register_qtimer1_ch2_handler()
//     before process_interrupt_enable_irqs() so the CH2 dispatcher
//     finds a registered handler when IRQ_QTIMER1 goes live.  Holds
//     trivially under this boot order — timepop_init is in phase 1,
//     enable_irqs is much later.
//
//   • process_clocks_init() can safely run after enable_irqs because
//     OCXO and VCLOCK compare channels remain in disabled-compare
//     state until their respective interrupt_start() arms them.  No
//     spurious one-second events fire between enable_irqs and the
//     first interrupt_start().
//
// Sacred epoch note:
//
//   Startup zero is not a special one-off regime.
//   The clocks subsystem requests a normal epoch zero during init, and the
//   first lawful PPS edge consummates it exactly the same way as explicit
//   ZERO and START piggybacked-on-zero do later.
//
// ============================================================================

void setup() {

  // ----------------------------------------------------------
  // Phase 0: Pre-transport boot (NO DEBUG LOGGING ALLOWED)
  // ----------------------------------------------------------

  delay(100);            // USB settle (not relied upon for faults)

  // DWT determinism first.
  maximize_dwt_determinism();

  debug_blink("911");    // unconditional startup indicator (LED-only)

  pinMode(LED_BUILTIN, OUTPUT);
  led_off();

  // ----------------------------------------------------------
  // Get I2C devices working
  // ----------------------------------------------------------

  Wire.begin();

  // ----------------------------------------------------------
  // Phase 1: Core instrumentation + early timing hardware
  // ----------------------------------------------------------

  memory_info_init();
  cpu_usage_init();

  // process_interrupt owns all interrupt-vector-bearing hardware:
  // QTimer1 (CH0/CH1/CH2/CH3) and QTimer3 (OCXO CH2/CH3).  All
  // compare channels start with their compares disabled — no spurious
  // ISRs can fire between this call and process_interrupt_enable_irqs().
  process_interrupt_init_hardware();

  // TimePop sets up its slot tables and registers its QTimer1 CH2
  // handler with process_interrupt.  No NVIC work happens here; the
  // CH2 IRQ goes live later in process_interrupt_enable_irqs(), and
  // any pending TCF1 from this init's first schedule_next() fires
  // the dispatcher then.
  timepop_init();

  // clocks owns DWT-local init and helper rolling baselines.
  // These helper baselines are not sacred epoch truth; startup epoch zero
  // will be established later on the first lawful PPS edge.
  process_clocks_init_hardware();

  // Witness
  process_witness_init_hardware();

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
  // Interrupt subsystem — runtime + ISR vectors
  // ----------------------------------------------------------

  debug_log("boot", "process_interrupt_init");
  process_interrupt_init();
  debug_log("boot", "process_interrupt_init done");

  debug_log("boot", "process_interrupt_enable_irqs");
  process_interrupt_enable_irqs();
  debug_log("boot", "process_interrupt_enable_irqs done");

  debug_log("boot", "process_interrupt_register");
  process_interrupt_register();
  debug_log("boot", "process_interrupt_register done");

  // ----------------------------------------------------------
  // Witness
  // ----------------------------------------------------------

  debug_log("boot", "process_witness_init");
  process_witness_init();
  debug_log("boot", "process_witness_init done");

  debug_log("boot", "process_witness_enable_irqs");
  process_witness_enable_irqs();
  debug_log("boot", "process_witness_enable_irqs done");

  debug_log("boot", "process_witness_register");
  process_witness_register();
  debug_log("boot", "process_witness_register done");

  // ----------------------------------------------------------
  // Time process registration
  // ----------------------------------------------------------

  debug_log("boot", "process_epoch_init");
  process_epoch_init();
  debug_log("boot", "process_epoch_init done");

  debug_log("boot", "process_epoch_register");
  process_epoch_register();
  debug_log("boot", "process_epoch_register done");

  debug_log("boot", "process_time_init");
  process_time_init();
  debug_log("boot", "process_time_init done");

  debug_log("boot", "process_time_register");
  process_time_register();
  debug_log("boot", "process_time_register done");

  // ----------------------------------------------------------
  // TimePop process registration
  // ----------------------------------------------------------

  debug_log("boot", "process_timepop_register");
  process_timepop_register();
  debug_log("boot", "process_timepop_register done");

  // ----------------------------------------------------------
  // Clocks subsystem
  // ----------------------------------------------------------
  //
  // process_clocks_init():
  //   • initializes time/timebase
  //   • subscribes PPS/OCXO interrupt consumers
  //   • requests startup epoch zero
  //   • starts PPS/OCXO providers
  //
  // The first lawful PPS edge after this point establishes canonical zero
  // for the nanosecond clocks.
  // ----------------------------------------------------------

  debug_log("boot", "process_clocks_init");
  process_clocks_init();
  debug_log("boot", "process_clocks_init done");

  debug_log("boot", "process_clocks_register");
  process_clocks_register();
  debug_log("boot", "process_clocks_register done");

  // ----------------------------------------------------------
  // Remaining subsystems
  // ----------------------------------------------------------

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