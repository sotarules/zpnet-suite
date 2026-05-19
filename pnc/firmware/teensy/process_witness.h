// ============================================================================
// process_witness.h
// ============================================================================
//
// Local hardware witness / latency-test process.
//
// Always-on model:
//   • witness initializes once and runs continuously
//   • no global START/STOP commands
//   • BRIDGE and PPS_PHASE are regular TimePop scheduled clients
//   • BRIDGE/PPS_PHASE do not use QTimer1 CH1 compare plumbing
//   • VCLOCK_START may temporarily use process_interrupt's hosted QTimer1 CH1
//     compare rail as a side-bell witness without taking over VCLOCK/CH2
//
// Special CLOCK_WITNESS mode:
//   • OCXO_START / OCXO_STOP temporarily take over the OCXO timer lanes
//   • VCLOCK_START / VCLOCK_STOP use process_interrupt-hosted QTimer1 CH1
//   • OCXO mode is only for branches where process_interrupt has been defeated
//   • QTimer compare handlers only latch edge facts and request TimePop ASAP work
//   • all test lanes publish historical interval facts on CLOCK_WITNESS
//
// ROUND_TRIP keeps its local low-level GPIO hardware because that is the thing
// being measured:
//   • stimulus pin 24
//   • GPIO sink pin 26
//
// In normal operation QTimer2/QTimer3 are deliberately not used by witness.
// In OCXO witness mode, witness mirrors process_interrupt's current OCXO
// mapping: OCXO1 on QTimer2 CH0 and OCXO2 on QTimer3 CH3.  VCLOCK witness
// mode does not own QTimer1; it registers with process_interrupt's CH1 hosted
// compare API and leaves the sovereign QTimer1 CH2 TimePop rail untouched.
//
// ============================================================================

#pragma once

void process_witness_init_hardware(void);
void process_witness_init(void);
void process_witness_enable_irqs(void);
void process_witness_register(void);

// GPIO ISR is attached dynamically by process_witness.
void witness_gpio_isr(void);
