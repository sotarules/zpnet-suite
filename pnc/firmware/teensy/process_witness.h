// ============================================================================
// process_witness.h
// ============================================================================
//
// Local hardware witness / latency-test process.
//
// Current ROUND_TRIP scheduler model:
//   • one deterministic 1 Hz supervisor
//   • 200 ms  GPIO HIGH
//   • 400 ms  GPIO LOW
//   • 600 ms  QTIMER HIGH
//   • 800 ms  QTIMER LOW
//
// Current command surface:
//   .tw witness round_trip   (or transport equivalent for process WITNESS)
//
// process_witness owns:
//   • stimulus pin 24
//   • GPIO sink pin 26
//   • QTimer2 CH0 on pin 13
//   • IRQ_QTIMER2
//
// process_interrupt does not own or observe this hardware.
//
// ============================================================================

#pragma once

void process_witness_init_hardware(void);
void process_witness_init(void);
void process_witness_enable_irqs(void);
void process_witness_register(void);

// GPIO ISR is attached dynamically by process_witness.
void witness_gpio_isr(void);
