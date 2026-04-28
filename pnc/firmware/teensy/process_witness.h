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
//   .tw witness edge         (PPS/PPS_VCLOCK heartbeat + last edge facts)
//   .tw witness bridge       (DWT/GNSS bridge check using interrupt-owned QTimer1 CH1)
//   .tw witness round_trip   (local stimulate-through-ISR latency)
//
// process_witness owns the local round-trip hardware:
//   • stimulus pin 24
//   • GPIO sink pin 26
//   • QTimer2 CH0 on pin 13
//   • IRQ_QTIMER2
//
// BRIDGE does not touch QTimer1 registers.  It registers as a hosted client
// of process_interrupt's QTimer1 CH1 compare service and receives already-
// authored event facts from process_interrupt.
//
// ============================================================================

#pragma once

void process_witness_init_hardware(void);
void process_witness_init(void);
void process_witness_enable_irqs(void);
void process_witness_register(void);

// GPIO ISR is attached dynamically by process_witness.
void witness_gpio_isr(void);
