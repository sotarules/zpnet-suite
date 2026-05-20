// ============================================================================
// process_witness.h
// ============================================================================
//
// Local hardware witness / latency-test process.
//
// Always-on model:
//   • witness initializes once and runs continuously
//   • no START/STOP commands
//   • BRIDGE and PPS_PHASE are regular TimePop scheduled clients
//   • no QTimer1 CH1 compare plumbing is used by witness
//
// ROUND_TRIP keeps its local low-level GPIO hardware because that is the thing
// being measured:
//   • stimulus pin 24
//   • GPIO sink pin 26
//
// QTimer2 is deliberately not used by witness. OCXO1 owns QTimer2 CH0 through
// process_interrupt.
//
// ============================================================================

#pragma once

void process_witness_init_hardware(void);
void process_witness_init(void);
void process_witness_enable_irqs(void);
void process_witness_register(void);

// GPIO ISR is attached dynamically by process_witness.
void witness_gpio_isr(void);
