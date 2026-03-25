#include "ad5693r.h"

#include <Wire.h>

// ============================================================================
// ad5693r.cpp — AD5693R 16-Bit I2C DAC Driver
// ============================================================================
//
// Minimal, defenseless driver. All I2C failures propagate as
// return values. No retry logic, no exception handling.
//
// Uses Wire (Bus 1, pins 18/19) — shared with INA260s, BME280,
// and EV5491 laser controller.
//
// This configuration targets EXTERNAL VREF operation.
//
// AD5693R control register bits:
//   D15 = RESET
//   D14 = PD1
//   D13 = PD0
//   D12 = REF   (0 = internal ref enabled, 1 = internal ref disabled)
//   D11 = GAIN  (0 = 1x, 1 = 2x)
//
// ============================================================================

// Command bytes
static constexpr uint8_t CMD_WRITE_INPUT_REG      = 0x10;
static constexpr uint8_t CMD_UPDATE_DAC_REG       = 0x20;
static constexpr uint8_t CMD_WRITE_DAC_AND_INPUT  = 0x30;
static constexpr uint8_t CMD_WRITE_CONTROL        = 0x40;

// ============================================================================
// Low-level I2C helpers
// ============================================================================

static bool i2c_write_24(uint8_t addr, uint8_t cmd, uint16_t data) {
  Wire.beginTransmission(addr);
  Wire.write(cmd);
  Wire.write((uint8_t)(data >> 8));
  Wire.write((uint8_t)(data & 0xFF));
  return Wire.endTransmission() == 0;
}

static bool i2c_write_cmd_only(uint8_t addr, uint8_t cmd) {
  Wire.beginTransmission(addr);
  Wire.write(cmd);
  return Wire.endTransmission() == 0;
}

// ============================================================================
// Public API
// ============================================================================

bool ad5693r_write_ctrl(uint8_t addr, uint16_t control_word) {
  return i2c_write_24(addr, CMD_WRITE_CONTROL, control_word);
}

bool ad5693r_write_input(uint8_t addr, uint16_t value) {
  return i2c_write_24(addr, CMD_WRITE_INPUT_REG, value);
}

bool ad5693r_update_dac(uint8_t addr) {
  return i2c_write_cmd_only(addr, CMD_UPDATE_DAC_REG);
}

bool ad5693r_write(uint8_t addr, uint16_t value) {
  return i2c_write_24(addr, CMD_WRITE_DAC_AND_INPUT, value);
}

bool ad5693r_read_input_register(uint8_t addr, uint16_t& out_value) {
  Wire.requestFrom((int)addr, 2);
  if (Wire.available() != 2) {
    return false;
  }

  const uint8_t msb = Wire.read();
  const uint8_t lsb = Wire.read();
  out_value = ((uint16_t)msb << 8) | (uint16_t)lsb;
  return true;
}

static bool ad5693r_configure(uint8_t addr) {
  return ad5693r_write_ctrl(addr, AD5693R_CTRL_EXTERNAL_VREF_1X);
}

static bool ad5693r_prime_default(uint8_t addr) {
  // Be explicit during bring-up:
  //   1) write input register
  //   2) force DAC update
  //
  // This avoids assuming that "write DAC and input" is behaving
  // the way we think on the current hardware.
  if (!ad5693r_write_input(addr, AD5693R_DAC_DEFAULT)) {
    return false;
  }
  if (!ad5693r_update_dac(addr)) {
    return false;
  }
  return true;
}

bool ad5693r_init(void) {
  const bool ok1_cfg = ad5693r_configure(AD5693R_ADDR_OCXO1);
  const bool ok2_cfg = ad5693r_configure(AD5693R_ADDR_OCXO2);

  const bool ok1_val = ok1_cfg ? ad5693r_prime_default(AD5693R_ADDR_OCXO1) : false;
  const bool ok2_val = ok2_cfg ? ad5693r_prime_default(AD5693R_ADDR_OCXO2) : false;

  return ok1_cfg && ok2_cfg && ok1_val && ok2_val;
}