#pragma once

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// ad5693r.h — AD5693R 16-Bit I2C DAC Driver
// ============================================================================
//
// Two AD5693R DACs on I2C Bus 1 (Wire), one per OCXO:
//
//   0x4E (A0 = HIGH)   → OCXO1 CTL
//   0x4C (A0 = LOW)    → OCXO2 CTL
//
// This driver is configured for INTERNAL VREF operation. The external VREF
// pin is deliberately not used for OCXO control because the shared external
// reference topology produced common-mode EFC modulation in hardware tests.
//
// The intended control word is:
//
//   PD1:PD0 = 00   normal mode
//   REF     = 0    internal reference enabled
//   GAIN    = 1    2× gain, output span ≈ 0..5 V
//
//   Control word = 0x0800
//
// I2C write protocol (3 bytes after address):
//
//   Byte 0: command byte
//   Byte 1: data MSB (D15-D8)
//   Byte 2: data LSB (D7-D0)
//
// Exposed commands used by this driver:
//
//   0x10  Write Input Register
//   0x20  Update DAC Register
//   0x30  Write DAC and Input Register
//   0x40  Write Control Register
//
// We expose both "write DAC+input" and explicit "update DAC" so
// diagnostics can test whether output latching is the problem.
//
// ============================================================================

// I2C addresses
static constexpr uint8_t AD5693R_ADDR_OCXO1 = 0x4E;   // A0 = HIGH
static constexpr uint8_t AD5693R_ADDR_OCXO2 = 0x4C;   // A0 = LOW

// DAC range
static constexpr uint16_t AD5693R_DAC_MIN = 0;
static constexpr uint16_t AD5693R_DAC_MAX = 65535;

// Default DAC value at init
static constexpr uint16_t AD5693R_DAC_DEFAULT = 32768;

// Control words.  The active production configuration is internal VREF with
// 2× output gain, yielding an approximate 0..5 V full-scale span from the
// AD5693R's internal 2.5 V reference.  The external-VREF word is retained only
// as an explicit diagnostic escape hatch for bench tests.
static constexpr uint16_t AD5693R_CTRL_INTERNAL_VREF_2X = 0x0800;
static constexpr uint16_t AD5693R_CTRL_EXTERNAL_VREF_1X = 0x1000;
static constexpr uint16_t AD5693R_CTRL_ACTIVE = AD5693R_CTRL_INTERNAL_VREF_2X;

// ============================================================================
// Lifecycle
// ============================================================================

// Initialize both DACs: configure control registers for internal VREF,
// 2× gain, normal mode, then write the default value and force an
// explicit DAC update.
//
// Wire.begin() must have been called before this.
// Returns true if both DACs acknowledged all required writes.
bool ad5693r_init(void);

// ============================================================================
// Output
// ============================================================================

// Write a 16-bit value to the input register only.
// Returns true if the device acknowledged.
bool ad5693r_write_input(uint8_t addr, uint16_t value);

// Update the DAC register from the current input register.
// Returns true if the device acknowledged.
bool ad5693r_update_dac(uint8_t addr);

// Write a 16-bit value to both DAC and input register.
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

// Read back the input register
bool ad5693r_read_input_register(uint8_t addr, uint16_t& out_value);