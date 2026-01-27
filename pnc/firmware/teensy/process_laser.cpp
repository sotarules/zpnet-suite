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

// ================================================================
// Laser State (authoritative snapshot)
// ================================================================

struct laser_state_t {
  uint16_t id1_raw = 0;
  float    id1_current_ma = NAN;

  uint8_t ctl0 = 0;
  uint8_t ctl1 = 0;
  uint8_t ctl2 = 0;

  uint8_t status1 = 0;
  uint8_t status2 = 0;
  uint8_t status3 = 0;
  uint8_t status4 = 0;
  uint8_t intr    = 0;

  uint16_t pd_adc_raw = 0;
  float    pd_voltage = NAN;
};

static laser_state_t LASER;

// ================================================================
// Initialization / Snapshot
// ================================================================

static void laser_snapshot(void) {
  LASER.ctl0 = i2c_read(REG_CTL0);
  LASER.ctl1 = i2c_read(REG_CTL1);
  LASER.ctl2 = i2c_read(REG_CTL2);

  uint8_t msb = i2c_read(REG_ID1_MSB);
  uint8_t lsb = i2c_read(REG_ID1_LSB);

  LASER.id1_raw = (msb << 2) | (lsb & 0x03);
  LASER.id1_current_ma = LASER.id1_raw * 0.25f;

  LASER.status1 = i2c_read(REG_STATUS1);
  LASER.status2 = i2c_read(REG_STATUS2);
  LASER.status3 = i2c_read(REG_STATUS3);
  LASER.status4 = i2c_read(REG_STATUS4);
  LASER.intr    = i2c_read(REG_INT);

  LASER.pd_adc_raw = analogRead(LASER_MONITOR_PIN);
  LASER.pd_voltage = (LASER.pd_adc_raw / 4095.0f) * 3.3f;
}

static void laser_emit_init_event(void) {
  Payload p;

  p.add("CTL0", LASER.ctl0);
  p.add("CTL1", LASER.ctl1);
  p.add("CTL2", LASER.ctl2);

  p.add("id1_raw", LASER.id1_raw);
  p.add("id1_current_ma", LASER.id1_current_ma);

  p.add("STATUS1", LASER.status1);
  p.add("STATUS2", LASER.status2);
  p.add("STATUS3", LASER.status3);
  p.add("STATUS4", LASER.status4);
  p.add("INT", LASER.intr);

  p.add("pd_adc_raw", LASER.pd_adc_raw);
  p.add("pd_voltage", LASER.pd_voltage);

  p.add("laser_emitting", LASER.pd_voltage > 0.75f);

  enqueueEvent("LASER_INITIALIZATION", p);
}

// ================================================================
// Lifecycle
// ================================================================

static bool laser_start(void) {
  {
    Payload p;
    p.add("stage", "process_start");
    enqueueEvent("LASER_INIT_ENTER", p);
  }

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

  laser_snapshot();
  laser_emit_init_event();

  return true;
}

static void laser_stop(void) {
  laser_inhibit();

  Payload p;
  p.add("action", "inhibit");
  enqueueEvent("LASER_STOP", p);
}

// ================================================================
// Commands
// ================================================================
//
// New command contract (assumed):
//   • Handlers return const Payload* (nullptr means "no payload")
//   • Framework serializes Payload and wraps response envelope
//

// ------------------------------------------------------------
// REPORT — return current laser state snapshot
// ------------------------------------------------------------
static const Payload* cmd_report(const char* /*args_json*/) {

  // Refresh authoritative snapshot
  laser_snapshot();

  // Persistent payload storage (safe to return pointer)
  static Payload p;
  p.clear();

  // Prefer numeric JSON values by default
  // Use add_fmt only when a human-facing precision policy is desired.
  p.add("id1_current_ma", LASER.id1_current_ma);
  p.add("pd_voltage", LASER.pd_voltage);
  p.add("laser_emitting", LASER.pd_voltage > 0.5f);

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

  // Side-effect only, no payload
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

  // Side-effect only, no payload
  return nullptr;
}

// ================================================================
// Registration
// ================================================================

static const process_command_entry_t LASER_COMMANDS[] = {
  { "REPORT", cmd_report },
  { "ON",     cmd_on     },
  { "OFF",    cmd_off    },
};

static const process_vtable_t LASER_PROCESS = {
  .name = "LASER",
  .start = laser_start,
  .stop = laser_stop,
  .query = nullptr,
  .commands = LASER_COMMANDS,
  .command_count = 3,
};

void process_laser_register(void) {
  process_register(PROCESS_TYPE_LASER, &LASER_PROCESS);
}
