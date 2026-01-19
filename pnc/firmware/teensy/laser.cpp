#include "config.h"
#include "laser.h"
#include "event_bus.h"
#include <Arduino.h>
#include <Wire.h>

// ================================================================
// MP5491 Definitions
// ================================================================
static constexpr uint8_t MP5491_ADDR = 0x66;

// Registers
static constexpr uint8_t REG_CTL0     = 0x00;
static constexpr uint8_t REG_CTL1     = 0x01;
static constexpr uint8_t REG_CTL2     = 0x03;
static constexpr uint8_t REG_ID1_MSB  = 0x07;
static constexpr uint8_t REG_ID1_LSB  = 0x08;
static constexpr uint8_t REG_STATUS1  = 0x2E;
static constexpr uint8_t REG_STATUS2  = 0x2F;
static constexpr uint8_t REG_STATUS3  = 0x30;
static constexpr uint8_t REG_STATUS4  = 0x31;
static constexpr uint8_t REG_INT      = 0x32;

// Bit masks
static constexpr uint8_t SYSEN_BIT  = 0x80;   // CTL0.D7
static constexpr uint8_t ID_EN_BIT  = 0x80;   // CTL1.D7
static constexpr uint8_t ID1_EN_BIT = 0x08;   // CTL1.D3

// ================================================================
// Laser configuration (authoritative)
// ================================================================

// ID1 current = 20 mA
// Resolution = 0.25 mA / LSB
// RawCode = 80 -> MSB = 0x14, LSB[1:0] = 0
static constexpr uint8_t ID1_CURRENT_MSB = 0x14;
static constexpr uint8_t ID1_CURRENT_LSB = 0x00;

// ================================================================
// I2C helpers
// ================================================================
static void i2c_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MP5491_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static uint8_t i2c_read(uint8_t reg) {
  Wire.beginTransmission(MP5491_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MP5491_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

// ================================================================
// Laser Initialization (authoritative)
// ================================================================
void laser_init() {
  enqueueEvent("LASER_INIT_ENTER", "\"stage\":\"laser_init\"");

  // ---- Hardware gate: default inhibit ----
  pinMode(LD_ON_PIN, OUTPUT);
  digitalWrite(LD_ON_PIN, LOW);  // HARD INHIBIT

  Wire.begin();

  // ---- Force authoritative configuration ----
  i2c_write(REG_CTL0, SYSEN_BIT);

  uint8_t ctl1 = i2c_read(REG_CTL1);
  i2c_write(REG_CTL1, ID_EN_BIT | ID1_EN_BIT | (ctl1 & 0x07));

  i2c_write(REG_ID1_MSB, ID1_CURRENT_MSB);

  uint8_t id1_lsb = i2c_read(REG_ID1_LSB);
  i2c_write(REG_ID1_LSB, (id1_lsb & ~0x03) | ID1_CURRENT_LSB);

  // ---- Read back everything for ground truth ----
  uint8_t ctl0     = i2c_read(REG_CTL0);
  uint8_t ctl1_rb  = i2c_read(REG_CTL1);
  uint8_t ctl2     = i2c_read(REG_CTL2);
  uint8_t id1_msb  = i2c_read(REG_ID1_MSB);
  uint8_t id1_lsb2 = i2c_read(REG_ID1_LSB);
  uint8_t st1      = i2c_read(REG_STATUS1);
  uint8_t st2      = i2c_read(REG_STATUS2);
  uint8_t st3      = i2c_read(REG_STATUS3);
  uint8_t st4      = i2c_read(REG_STATUS4);
  uint8_t intr     = i2c_read(REG_INT);

  uint16_t raw = (id1_msb << 2) | (id1_lsb2 & 0x03);
  double current_ma = raw * 0.25;

  // ---- Emit canonical initialization event ----
  String body;
  body += "\"CTL0\":";
  body += ctl0;
  body += ",\"CTL1\":";
  body += ctl1_rb;
  body += ",\"CTL2\":";
  body += ctl2;
  body += ",\"ID1_MSB\":";
  body += id1_msb;
  body += ",\"ID1_LSB\":";
  body += id1_lsb2;
  body += ",\"ID1_raw\":";
  body += raw;
  body += ",\"ID1_current_ma\":";
  {
    char buf[32];
    snprintf(buf, sizeof(buf), "%.2f", current_ma);
    body += buf;
  }
  body += ",\"STATUS1\":";
  body += st1;
  body += ",\"STATUS2\":";
  body += st2;
  body += ",\"STATUS3\":";
  body += st3;
  body += ",\"STATUS4\":";
  body += st4;
  body += ",\"INT\":";
  body += intr;
  body += ",\"SYSEN\":";
  body += (ctl0 & SYSEN_BIT) ? "true" : "false";
  body += ",\"ID_EN\":";
  body += (ctl1_rb & ID_EN_BIT) ? "true" : "false";
  body += ",\"ID1_EN\":";
  body += (ctl1_rb & ID1_EN_BIT) ? "true" : "false";
  body += ",\"LD_ON_DEFAULT\":\"LOW\"";

  enqueueEvent("LASER_INITIALIZATION", body);
}

// ================================================================
// Runtime control
// ================================================================
void laser_on() {
  enqueueEvent("LASER_ON", "\"action\":\"allow_emission\"");
  digitalWrite(LD_ON_PIN, HIGH);
}

void laser_off() {
  enqueueEvent("LASER_OFF", "\"action\":\"inhibit_emission\"");
  digitalWrite(LD_ON_PIN, LOW);
}

