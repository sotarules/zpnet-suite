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
// ROUND_TRIP keeps its local low-level hardware because that is the thing being
// measured:
//   • stimulus pin 24
//   • GPIO sink pin 26
//   • QTimer2 CH0 on pin 13
//   • IRQ_QTIMER2
//
// Command surface:
//   .tw witness report        (hardware/timer state)
//   .tw witness edge          (PPS/PPS_VCLOCK heartbeat + last edge facts)
//   .tw witness bridge        (DWT/GNSS bridge check using TimePop fire facts)
//   .tw witness pps_phase     (PPS/VCLOCK phase using TimePop witness facts)
//   .tw witness gpio_delay    (simple digitalWriteFast HIGH stimulus cost)
//   .tw witness qtimer_read   (GPIO ISR-to-VCLOCK counter observation delay/cost)
//   .tw witness dwt_read      (DWT read-cost measurement)
//   .tw witness entry_latency (PPS spin-shadow ISR entry latency)
//   .tw witness round_trip    (local stimulate-through-ISR latency)
//
// ============================================================================

#pragma once

void process_witness_init_hardware(void);
void process_witness_init(void);
void process_witness_enable_irqs(void);
void process_witness_register(void);

// GPIO ISR is attached dynamically by process_witness.
void witness_gpio_isr(void);
