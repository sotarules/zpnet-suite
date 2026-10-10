#include "integer_only.h"
#include "max31865.h"
#include "config.h"
#include "timepop.h"
#include <Arduino.h>
#include <SPI.h>

namespace {
constexpr uint8_t CONFIG = 0x10; // 3-wire, normally off, 60 Hz rejection
constexpr uint8_t BIAS = 0x80;
constexpr uint8_t ONE_SHOT = 0x20;
constexpr uint8_t CLEAR_FAULT = 0x02;
// Adafruit PT1000 board: 100 nF RTD input capacitor. 10 ms exceeds
// 10.5 RC time constants + 1 ms across the probe's operating range.
constexpr uint32_t BIAS_SETTLE_MS = 10;
constexpr uint32_t CONVERSION_MS = 65; // > 55 ms maximum at 60 Hz
constexpr uint32_t PERIOD_MS = 1000;
constexpr uint32_t SPI_HZ = 500000;
constexpr uint32_t RREF_OHMS = 4300;
constexpr uint32_t R0_OHMS = 1000;
static_assert(MAX31865_SPI_BUS_INDEX == 1, "RTD driver owns SPI1");

enum class Phase : uint8_t { START, OFF_SETTLE, SETTLE, CONVERT };
Phase phase = Phase::START;
uint32_t deadline_ms = 0;
uint32_t cycle_started_ms = 0;
rtd_snapshot_t snapshot;
uint8_t pre_clear_registers[8] = {};
rtd_spi_trace_t pre_clear_spi;
rtd_spi_trace_t check_spi;
rtd_spi_trace_t registers_spi;

// SPI1 is LPSPI3 on Teensy 4.1. Read only: never pop RDR, acknowledge SR,
// flush FIFOs, or mask interrupts while collecting diagnostic evidence.
inline rtd_spi_state_t spi_state() {
  rtd_spi_state_t state;
  state.sr = IMXRT_LPSPI3_S.SR;
  state.fsr = IMXRT_LPSPI3_S.FSR;
  return state;
}

void select() {
  SPI1.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE1));
  digitalWrite(MAX31865_CS_PIN, LOW);
  // Datasheet: CS-to-clock setup >= 400 ns. Keep interrupts enabled.
  delayNanoseconds(1000);
}
void deselect() {
  // Datasheet: final clock-to-CS hold >= 100 ns; CS inactive >= 400 ns.
  delayNanoseconds(1000);
  digitalWrite(MAX31865_CS_PIN, HIGH);
  delayNanoseconds(1000);
  SPI1.endTransaction();
}
void write_config(uint8_t value) {
  select();
  SPI1.transfer(0x80);
  SPI1.transfer(value);
  deselect();
}
uint8_t read_config(rtd_spi_trace_t* trace = nullptr) {
  if (trace) trace->before = spi_state();
  select();
  SPI1.transfer(0x00);
  const uint8_t value = SPI1.transfer(0);
  if (trace) trace->after = spi_state();
  deselect();
  return value;
}

FLASHMEM void read_registers(uint8_t (&registers)[8], rtd_spi_trace_t& trace) {
  trace.before = spi_state();
  select();
  SPI1.transfer(0x00);
  for (uint8_t& value : registers) value = SPI1.transfer(0);
  trace.after = spi_state();
  deselect();
}

// Keep this cold copy out of the RAM-resident finish() even under optimization.
FLASHMEM __attribute__((noinline)) void retain_failure_episode(rtd_status_t status) {
  // A successful acquisition ends an episode; repeated failures cannot replace
  // its initiating evidence. Keep boot's first failure as a separate record.
  if (snapshot.status != rtd_status_t::OK &&
      snapshot.status != rtd_status_t::INITIALIZING) return;
  ++snapshot.failure_episodes;
  rtd_failure_t& first = snapshot.latest_failure_episode;
  first.attempt = snapshot.attempts;
  first.captured_at_ms = millis();
  first.sample_sequence = snapshot.sample_sequence;
  first.status = status;
  first.check = snapshot.check;
  first.checked_config = snapshot.checked_config;
  for (unsigned i = 0; i < 8; ++i) {
    first.pre_clear_registers[i] = pre_clear_registers[i];
    first.registers[i] = snapshot.registers[i];
  }
  first.trace_count = snapshot.trace_count;
  for (unsigned i = 0; i < first.trace_count; ++i)
    first.config_trace[i] = snapshot.config_trace[i];
  first.pre_clear_spi = pre_clear_spi;
  first.check_spi = check_spi;
  first.registers_spi = registers_spi;
  first.spi_cr = IMXRT_LPSPI3_S.CR;
  first.spi_tcr = IMXRT_LPSPI3_S.TCR;
  first.spi_ccr = IMXRT_LPSPI3_S.CCR;
  first.spi_cfgr1 = IMXRT_LPSPI3_S.CFGR1;
  first.ccm_cbcmr = CCM_CBCMR;
  if (snapshot.first_failure.attempt == 0) snapshot.first_failure = first;
}

// IEC 60751 Callendar-Van Dusen relation, solved with six Newton steps.
// Includes the negative-temperature C term; range -200..850 C.
// No binary floating point, sqrt cancellation, or approximate PT100 polynomial.
Double ratio_at(Double t) {
  const Double a = 0.0039083_D;
  const Double b = -0.0000005775_D;
  const Double c = -0.000000000004183_D;
  Double r = 1_D + a*t + b*t*t;
  if (t < 0_D) r += c*(t - 100_D)*t*t*t;
  return r;
}
bool temperature_from_ratio(Double ratio, Double& out) {
  if (ratio < ratio_at(-200_D) || ratio > ratio_at(850_D)) return false;
  Double t = (ratio - 1_D) / 0.0039083_D;
  for (unsigned i = 0; i < 6; ++i) {
    Double derivative = 0.0039083_D - 0.000001155_D*t;
    if (t < 0_D)
      derivative += -0.000000000004183_D*(4_D*t*t*t - 300_D*t*t);
    t -= (ratio_at(t) - ratio) / derivative;
  }
  out = t;
  return true;
}

void finish(rtd_status_t status) {
  if (snapshot.register_count == 1) {
    // Preserve the failed single-byte check and its SPI trace separately.
    // This full read happens before bias-off cleanup or the next fault clear.
    read_registers(snapshot.registers, registers_spi);
    snapshot.register_count = 8;
  }
  snapshot.fault_status = snapshot.registers[7] & 0xfc;
  if (status != rtd_status_t::OK) retain_failure_episode(status);
  write_config(CONFIG); // bias off between readings, including fault cases
  snapshot.cleanup_config = read_config();
  snapshot.status = status;
  if (status != rtd_status_t::OK) ++snapshot.errors;
  phase = Phase::START;
  // Never catch up by issuing a burst of conversions after foreground delays.
  const uint32_t now = millis();
  deadline_ms = (uint32_t)(now - cycle_started_ms) >= PERIOD_MS
      ? now + PERIOD_MS : cycle_started_ms + PERIOD_MS;
}

bool ready(void*) {
  return (int32_t)((uint32_t)millis() - deadline_ms) >= 0;
}
void service(void*) {
  if (phase == Phase::START) {
    cycle_started_ms = millis();
    ++snapshot.attempts;
    snapshot.trace_attempt = snapshot.attempts;
    snapshot.trace_count = 0;
    snapshot.config_trace[snapshot.trace_count++] = read_config();
    // Explicitly request normally-off, then allow a full conversion interval.
    // Writing D5=0 is not assumed to abort an in-progress conversion.
    write_config(CONFIG);
    snapshot.config_trace[snapshot.trace_count++] = read_config();
    phase = Phase::OFF_SETTLE;
    deadline_ms = millis() + CONVERSION_MS;
    return;
  }
  if (phase == Phase::OFF_SETTLE) {
    // Read all registers before the retry's first CLEAR_FAULT write, preserving
    // an input fault even if that write clears it before the later check fails.
    read_registers(pre_clear_registers, pre_clear_spi);
    snapshot.config_trace[snapshot.trace_count++] = pre_clear_registers[0];
    // Reassert thresholds every cycle so a sensor-only reset recovers without
    // rebooting Teensy. SPI has no ACK; verify configuration and thresholds.
    select();
    SPI1.transfer(0x83);
    SPI1.transfer(0xff); SPI1.transfer(0xff); // high threshold
    SPI1.transfer(0x00); SPI1.transfer(0x00); // low threshold
    deselect();
    write_config(CONFIG | BIAS | CLEAR_FAULT);
    snapshot.config_trace[snapshot.trace_count++] = read_config();
    phase = Phase::SETTLE;
    deadline_ms = millis() + BIAS_SETTLE_MS;
    return;
  }
  if (phase == Phase::SETTLE) {
    snapshot.registers[0] = read_config(&check_spi);
    snapshot.checked_config = snapshot.registers[0];
    snapshot.config_trace[snapshot.trace_count++] = snapshot.registers[0];
    snapshot.register_count = 1;
    snapshot.check_attempt = snapshot.attempts;
    snapshot.check = "BIAS_CONFIG";
    if (snapshot.registers[0] != (CONFIG | BIAS)) {
      finish(rtd_status_t::SPI_ERROR);
      return;
    }
    write_config(CONFIG | BIAS | ONE_SHOT);
    snapshot.config_trace[snapshot.trace_count++] = read_config();
    phase = Phase::CONVERT;
    deadline_ms = millis() + CONVERSION_MS;
    return;
  }

  uint8_t registers[8];
  read_registers(registers, check_spi);
  registers_spi = check_spi;
  for (unsigned i = 0; i < 8; ++i) snapshot.registers[i] = registers[i];
  snapshot.register_count = 8;
  snapshot.checked_config = registers[0];
  snapshot.check_attempt = snapshot.attempts;
  snapshot.check = "CONVERSION_CONFIG";
  if (registers[0] != (CONFIG | BIAS)) {
    finish(rtd_status_t::SPI_ERROR);
    return;
  }
  // MAX31865 Table 6: threshold register bit D0 is don't-care, not data.
  snapshot.check = "HIGH_THRESHOLD";
  if (registers[3] != 0xff || (registers[4] & 0xfe) != 0xfe) {
    finish(rtd_status_t::SPI_ERROR);
    return;
  }
  snapshot.check = "LOW_THRESHOLD";
  if (registers[5] != 0 || (registers[6] & 0xfe) != 0) {
    finish(rtd_status_t::SPI_ERROR);
    return;
  }
  // Table 7: only D7..D2 are defined fault indicators. Preserve all raw bits
  // in diagnostics, but do not interpret the two don't-care bits as faults.
  snapshot.fault_status = registers[7] & 0xfc;
  snapshot.check = "RTD_FAULT";
  const uint16_t word = (uint16_t(registers[1]) << 8) | registers[2];
  snapshot.raw_code = word >> 1;
  if ((word & 1) || snapshot.fault_status) {
    finish(rtd_status_t::RTD_FAULT);
    return;
  }
  const Double resistance = Double(snapshot.raw_code) * Double(RREF_OHMS) / 32768_D;
  Double temperature;
  if (!temperature_from_ratio(resistance / Double(R0_OHMS), temperature)) {
    snapshot.check = "RESISTANCE_RANGE";
    finish(rtd_status_t::OUT_OF_RANGE);
    return;
  }
  snapshot.resistance_ohms = resistance;
  snapshot.temperature_c = temperature;
  snapshot.sampled_at_ms = millis();
  ++snapshot.sample_sequence;
  snapshot.check = "OK";
  finish(rtd_status_t::OK);
}
} // namespace

void max31865_init() {
  // Set the inactive output latch before enabling the CS output driver.
  digitalWrite(MAX31865_CS_PIN, HIGH);
  pinMode(MAX31865_CS_PIN, OUTPUT);
  // Must precede begin(): the library's default MISO1 is GNSS PPS pin 1.
  SPI1.setMOSI(MAX31865_MOSI_PIN);
  SPI1.setMISO(MAX31865_MISO_PIN);
  SPI1.setSCK(MAX31865_SCK_PIN);
  SPI1.begin();
  deadline_ms = millis();
  timepop_register_foreground_service(ready, service, nullptr);
}

const rtd_snapshot_t& max31865_snapshot() { return snapshot; }
const char* max31865_status_name(rtd_status_t status) {
  switch (status) {
    case rtd_status_t::OK: return "OK";
    case rtd_status_t::SPI_ERROR: return "SPI_ERROR";
    case rtd_status_t::RTD_FAULT: return "RTD_FAULT";
    case rtd_status_t::OUT_OF_RANGE: return "OUT_OF_RANGE";
    default: return "INITIALIZING";
  }
}
