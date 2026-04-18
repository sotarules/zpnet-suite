#pragma once

#include "payload.h"
#include <Arduino.h>

// --------------------------------------------------------------
// Generic utility helpers
// --------------------------------------------------------------
//
// These functions have no domain meaning.
// They exist to support other modules cleanly.
//

// Safe bounded string copy (always null-terminated)
void safeCopy(char* dst, size_t dst_sz, const char* src);

// Escape a C string for JSON inclusion
String jsonEscape(const char* s);

// Append a float key/value pair to a JSON body fragment
void appendFloatKV(
  String& b,
  const char* key,
  float value,
  int decimals = 5
);

// CPU temperature in Celsius (best-effort)
float cpuTempC();

// Internal voltage reference estimate (volts)
float readVrefVolts();

// Free heap memory (bytes)
uint32_t freeHeapBytes();

uint32_t maxAllocBytes();

// ============================================================================
// ARM primitives
// ============================================================================
//
// Teensy's Arduino core does not expose CMSIS intrinsics through <Arduino.h>,
// so we wrap the single-instruction ARM operations we need.  Each of these
// compiles to exactly one machine instruction.  They exist as named inlines
// so calling code can read as ordinary C.

// Data Memory Barrier — orders memory accesses before vs. after this point.
// Used when publishing lock-free shared snapshots.
static inline void dmb(void) {
  __asm volatile("dmb" ::: "memory");
}

// Data Synchronization Barrier — waits for all pending memory operations
// to complete before the next instruction.  Stronger than dmb.
static inline void dsb(void) {
  __asm volatile("dsb" ::: "memory");
}

// Instruction Synchronization Barrier — flushes the pipeline so subsequent
// instructions are fetched after any pending context change (e.g. BASEPRI
// write takes effect before the next load).
static inline void isb(void) {
  __asm volatile("isb" ::: "memory");
}

// Read the current BASEPRI value (interrupt-mask threshold).
static inline uint32_t read_basepri(void) {
  uint32_t v;
  __asm volatile("mrs %0, basepri" : "=r" (v));
  return v;
}

// Write BASEPRI.  Value 0 disables masking; non-zero masks interrupts at
// that priority level and below.
static inline void write_basepri(uint32_t v) {
  __asm volatile("msr basepri, %0" :: "r" (v) : "memory");
}

// ============================================================================
// Interrupt priority masking (BASEPRI-based critical sections)
// ============================================================================
//
// PRIMASK-based critical sections (noInterrupts() / interrupts()) disable
// ALL interrupts including the PPS GPIO ISR at priority 0.  That defeats
// PPS Witness determinism: any such critical section that happens to span
// a PPS edge delays counter32 capture by the full duration of the section.
//
// BASEPRI-based critical sections mask interrupts BELOW a priority
// threshold while leaving higher-priority ISRs fully operational.  By
// setting BASEPRI to mask priority level 1 and below, the PPS GPIO ISR
// at level 0 remains sovereign and always preempts promptly.
//
// This requires that interrupt priorities are actually separated:
//   IRQ_GPIO6789  → NVIC_SET_PRIORITY level 0  — PPS
//   IRQ_QTIMER1   → NVIC_SET_PRIORITY level 1  — TimePop + VCLOCK cadence
//   IRQ_QTIMER3   → NVIC_SET_PRIORITY level 2  — OCXO cadences
//
// On iMXRT1062 the NVIC priority field is 4 bits, stored in the upper
// nibble of the 8-bit IPR byte.  The BASEPRI register uses the same
// encoding.  Value 16 = level 1.
//
// Usage:
//
//   const uint32_t saved = critical_enter();
//   // ... critical section ...
//   critical_exit(saved);
//
// Save / restore is deliberate.  Some critical sections may be entered
// from ISR context (e.g. the GPIO ISR at level 0 calling
// timepop_arm_asap).  Nested sections must not unmask higher priorities
// on exit, only return to the previously-held mask.
//
// ============================================================================

static constexpr uint32_t BASEPRI_MASK_BELOW_PPS = 16;

static inline uint32_t critical_enter(void) {
  const uint32_t saved = read_basepri();
  write_basepri(BASEPRI_MASK_BELOW_PPS);
  dsb();
  isb();
  return saved;
}

static inline void critical_exit(uint32_t saved) {
  write_basepri(saved);
}