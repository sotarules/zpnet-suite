#pragma once

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// ad5693r.h — AD5693R 16-Bit I2C DAC Driver
// ============================================================================
//
// Two AD5693R DACs on I2C Bus 1 (Wire), one per OCXO:
//
//   0x4C  (A0 = LOW)   → OCXO1 CTL
//   0x4E  (A0 = HIGH)  → OCXO2 CTL
//
// Each device has an internal 2.5 V reference.  With gain = 2×,
// output spans 0–5 V, matching the AOCJY1-A CTL input range.
//
// I2C protocol (3-byte write after address):
//
//   Byte 0:  Command byte
//   Byte 1:  Data MSB (D15–D8)
//   Byte 2:  Data LSB (D7–D0)
//
// Commands:
//
//   0x30  Write DAC and Input Register (immediate output update)
//   0x40  Write Control Register
//
// Control register format (16-bit data field):
//
//   D15 D14 | D13  | D12  | D11 D10 | D9–D0
//   PD1 PD0 | REF  | GAIN | reserved | don't care
//
//   PD = 00 (normal mode)
//   REF = 1 (internal reference enabled)
//   GAIN = 1 (2× gain → 0–5 V output)
//
//   Control word = 0x3000
//
// LDAC is tied to GND on the Adafruit breakout, so every write
// to the DAC register immediately updates the analog output.
//
// ============================================================================

// I2C addresses
static constexpr uint8_t AD5693R_ADDR_OCXO1 = 0x4C;   // A0 = LOW
static constexpr uint8_t AD5693R_ADDR_OCXO2 = 0x4E;   // A0 = HIGH

// DAC range
static constexpr uint16_t AD5693R_DAC_MIN = 0;
static constexpr uint16_t AD5693R_DAC_MAX = 65535;

// Default DAC value at init (midscale → ~2.5 V)
static constexpr uint16_t AD5693R_DAC_DEFAULT = 32768;

// ============================================================================
// Lifecycle
// ============================================================================

// Initialize both DACs: configure control registers (internal ref,
// gain = 2×, normal mode) and write the default output value.
//
// Wire.begin() must have been called before this.
// Returns true if both DACs acknowledged.
bool ad5693r_init(void);

// ============================================================================
// Output
// ============================================================================

// Write a 16-bit value to a specific DAC.  Immediate analog update.
// Returns true if the device acknowledged.
bool ad5693r_write(uint8_t addr, uint16_t value);

// Convenience: write to OCXO1 DAC
static inline bool ad5693r_write_ocxo1(uint16_t value) {
  return ad5693r_write(AD5693R_ADDR_OCXO1, value);
}

// Convenience: write to OCXO2 DAC
static inline bool ad5693r_write_ocxo2(uint16_t value) {
  return ad5693r_write(AD5693R_ADDR_OCXO2, value);
}

// Write the control register directly (for diagnostics)
bool ad5693r_write_ctrl(uint8_t addr, uint16_t control_word);