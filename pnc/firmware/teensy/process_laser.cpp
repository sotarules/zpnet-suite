#include "process_laser.h"

#include "config.h"
#include "events.h"
#include "payload.h"
#include "process.h"

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
static constexpr uint8_t SYSEN_BIT  = 0x80;
static constexpr uint8_t ID_EN_BIT  = 0x80;
static constexpr uint8_t ID1_EN_BIT = 0x08;

// ================================================================
// Authoritative Laser Configuration
// ================================================================

// ID1 current = 20 mA
// Resolution = 0.25 mA / LSB
// RawCode = 80 → MSB = 0x14, LSB = 0
static constexpr uint8_t ID1_CURRENT_MSB = 0x14;
static constexpr uint8_t ID1_CURRENT_LSB = 0x00;

// Emission classification threshold (authoritative)
static constexpr float LASER_EMIT_THRESHOLD_V = 0.75f;

// ================================================================
// Helpers
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

static void laser_inhibit(void) {
  digitalWrite(LD_ON_PIN, LOW);
}

static float read_pd_voltage(void) {
  uint16_t raw = analogRead(LASER_MONITOR_PIN);
  return (raw / 4095.0f) * 3.3f;
}

// ================================================================
// Public initialization (authoritative, idempotent)
// ================================================================

void process_laser_init(void) {

  pinMode(LD_ON_PIN, OUTPUT);
  laser_inhibit();

  pinMode(LASER_MONITOR_PIN, INPUT);
  analogReadResolution(12);

  Wire.begin();

  // Authoritative configuration
  i2c_write(REG_CTL0, SYSEN_BIT);

  uint8_t ctl1 = i2c_read(REG_CTL1);
  i2c_write(REG_CTL1, ID_EN_BIT | ID1_EN_BIT | (ctl1 & 0x07));

  i2c_write(REG_ID1_MSB, ID1_CURRENT_MSB);

  uint8_t lsb = i2c_read(REG_ID1_LSB);
  i2c_write(REG_ID1_LSB, (lsb & ~0x03) | ID1_CURRENT_LSB);

  // ------------------------------------------------------------
  // Forensic initialization snapshot
  // ------------------------------------------------------------
  uint8_t msb = i2c_read(REG_ID1_MSB);
  uint8_t lsb2 = i2c_read(REG_ID1_LSB);

  uint16_t id1_raw = (msb << 2) | (lsb2 & 0x03);
  float id1_current_ma = id1_raw * 0.25f;

  float pd_voltage = read_pd_voltage();

  Payload p;
  p.add("id1_raw", id1_raw);
  p.add("id1_current_ma", id1_current_ma);
  p.add("pd_voltage", pd_voltage);
  p.add("laser_emitting", pd_voltage > LASER_EMIT_THRESHOLD_V);

  enqueueEvent("LASER_INITIALIZATION", p);
}

// ================================================================
// Commands
// ================================================================

// ------------------------------------------------------------
// INIT — explicit initialization wrapper
// ------------------------------------------------------------
static const Payload* cmd_init(const char* /*args_json*/) {
  process_laser_init();
  return nullptr;
}

// ------------------------------------------------------------
// REPORT — authoritative laser snapshot (stateless)
// ------------------------------------------------------------
static const Payload* cmd_report(const char* /*args_json*/) {

  static Payload p;
  p.clear();

  uint8_t msb = i2c_read(REG_ID1_MSB);
  uint8_t lsb = i2c_read(REG_ID1_LSB);

  uint16_t id1_raw = (msb << 2) | (lsb & 0x03);
  float id1_current_ma = id1_raw * 0.25f;

  float pd_voltage = read_pd_voltage();

  p.add("id1_current_ma", id1_current_ma);
  p.add("pd_voltage", pd_voltage);
  p.add("laser_emitting", pd_voltage > LASER_EMIT_THRESHOLD_V);

  return &p;
}

// ------------------------------------------------------------
// ON — permit emission
// ------------------------------------------------------------
static const Payload* cmd_on(const char* /*args_json*/) {

  Payload ev;
  ev.add("action", "allow_emission");
  enqueueEvent("LASER_ON", ev);

  digitalWrite(LD_ON_PIN, HIGH);
  return nullptr;
}

// ------------------------------------------------------------
// OFF — inhibit emission
// ------------------------------------------------------------
static const Payload* cmd_off(const char* /*args_json*/) {

  Payload ev;
  ev.add("action", "inhibit_emission");
  enqueueEvent("LASER_OFF", ev);

  digitalWrite(LD_ON_PIN, LOW);
  return nullptr;
}

// ================================================================
// Registration
// ================================================================

static const process_command_entry_t LASER_COMMANDS[] = {
  { "INIT",   cmd_init   },
  { "REPORT", cmd_report },
  { "ON",     cmd_on     },
  { "OFF",    cmd_off    },
};

static const process_vtable_t LASER_PROCESS = {
  .name = "LASER",
  .query = nullptr,
  .commands = LASER_COMMANDS,
  .command_count = 4,
};

void process_laser_register(void) {
  process_register("LASER", &LASER_PROCESS);
}
