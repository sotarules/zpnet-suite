#include "ad5693r.h"

#include <Wire.h>

// ============================================================================
// ad5693r.cpp — AD5693R 16-Bit I2C DAC Driver
// ============================================================================
//
// Minimal, defenseless driver.  All I2C failures propagate as
// return values.  No retry logic, no exception handling.
//
// Uses Wire (Bus 1, pins 18/19) — shared with INA260s, BME280,
// and EV5491 laser controller.
//
// ============================================================================

// Command bytes (AD5693R datasheet Table 12)
static constexpr uint8_t CMD_WRITE_DAC_AND_INPUT = 0x30;
static constexpr uint8_t CMD_WRITE_CONTROL       = 0x40;

// Control register value:
//   PD1:PD0  = 00  (normal mode)
//   REF      = 1   (internal 2.5 V reference enabled)
//   GAIN     = 1   (2× gain → 0–5 V output span)
//
//   Bits: [00] [1] [1] [0000000000]
//   = 0x3000
static constexpr uint16_t CONTROL_WORD = 0x3000;

// ============================================================================
// Low-level I2C write: command byte + 16-bit data
// ============================================================================

static bool i2c_write_24(uint8_t addr, uint8_t cmd, uint16_t data) {
  Wire.beginTransmission(addr);
  Wire.write(cmd);
  Wire.write((uint8_t)(data >> 8));    // MSB
  Wire.write((uint8_t)(data & 0xFF));  // LSB
  return Wire.endTransmission() == 0;
}

// ============================================================================
// Configure one DAC: internal ref, 2× gain, normal mode
// ============================================================================

bool ad5693r_write_ctrl(uint8_t addr, uint16_t control_word) {
  return i2c_write_24(addr, CMD_WRITE_CONTROL, control_word);
}

static bool ad5693r_configure(uint8_t addr) {
  return ad5693r_write_ctrl(addr, CONTROL_WORD);
}

// Software reset: command 0x60, data 0x0000
static bool ad5693r_reset(uint8_t addr) {
  return i2c_write_24(addr, 0x60, 0x0000);
}

// ============================================================================
// Public API
// ============================================================================

bool ad5693r_init(void) {
  ad5693r_reset(AD5693R_ADDR_OCXO1);
  ad5693r_reset(AD5693R_ADDR_OCXO2);
  delay(1);  // allow POR to complete after soft reset

  bool ok1 = ad5693r_configure(AD5693R_ADDR_OCXO1);
  bool ok2 = ad5693r_configure(AD5693R_ADDR_OCXO2);

  // Write default (midscale) to both outputs
  if (ok1) ad5693r_write(AD5693R_ADDR_OCXO1, AD5693R_DAC_DEFAULT);
  if (ok2) ad5693r_write(AD5693R_ADDR_OCXO2, AD5693R_DAC_DEFAULT);

  return ok1 && ok2;
}

bool ad5693r_write(uint8_t addr, uint16_t value) {
  return i2c_write_24(addr, CMD_WRITE_DAC_AND_INPUT, value);
}