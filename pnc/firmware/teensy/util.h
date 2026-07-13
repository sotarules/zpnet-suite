#pragma once

#include <Arduino.h>
#include <stdint.h>

// --------------------------------------------------------------
// Generic utility helpers
// --------------------------------------------------------------
//
// These functions have no domain meaning.
// They exist to support other modules cleanly.
//

// ============================================================================
// Fixed-decimal publication boundary
// ============================================================================
//
// Scientific code may continue to use float/double internally.  Before a value
// crosses into Payload, convert it here into an integer-only decimal
// decomposition.  Payload receives only whole/fractional digits plus metadata;
// no float or double enters its construction/serialization ABI.
//
// Conversion preserves the former Payload fixed-format contract:
//   * decimal places are clamped to [0, 12]
//   * rounding is half away from zero
//   * negative values that round to zero are rendered without a minus sign
//   * NaN, infinity, and magnitudes above 9e18 are represented as invalid
//
// The structure deliberately does not combine whole and fractional digits into
// one scaled int64_t.  Large nanosecond clockfaces can therefore retain up to
// 12 decimal places without overflowing an intermediate scaled integer.

static constexpr uint8_t FIXED_DECIMAL_MAX_PLACES = 12U;

enum class fixed_decimal_status_t : uint8_t {
  VALID = 0,
  NAN_VALUE = 1,
  POSITIVE_INFINITY = 2,
  NEGATIVE_INFINITY = 3,
  OUT_OF_RANGE = 4,
};

struct fixed_decimal_t {
  uint64_t whole;
  uint64_t fractional;
  uint64_t source_bits;  // Original IEEE-754 evidence; never interpreted by Payload.
  uint8_t decimal_places;
  uint8_t negative;
  fixed_decimal_status_t status;

  bool valid() const {
    return status == fixed_decimal_status_t::VALID;
  }
};

// Convert a floating-point science value into an integer-only decimal object.
// The floating-point work occurs in the caller/util layer, before Payload is
// entered.  Payload::add(const fixed_decimal_t&) performs only integer work.
fixed_decimal_t toFixedDecimal(double value, int decimal_places);

const char* fixedDecimalStatusName(fixed_decimal_status_t status);

// Safe bounded string copy (always null-terminated)
void safeCopy(char* dst, size_t dst_sz, const char* src);

// Escape a C string for JSON inclusion
String jsonEscape(const char* s);

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