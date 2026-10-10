#pragma once

#include <stdint.h>
#include "double.h"

// Foreground-owned PT1000 acquisition. No ISR, allocation, delay(), or native FP.
enum class rtd_status_t : uint8_t {
  INITIALIZING, OK, SPI_ERROR, RTD_FAULT, OUT_OF_RANGE
};

struct rtd_snapshot_t {
  rtd_status_t status = rtd_status_t::INITIALIZING;
  uint32_t sample_sequence = 0;
  uint32_t attempts = 0;
  uint32_t errors = 0;
  uint32_t sampled_at_ms = 0; // millis() at result readout, not UTC
  uint16_t raw_code = 0;     // 15-bit resistance ratio
  uint8_t fault_status = 0;  // MAX31865 register 7
  // Last check evidence, captured before cleanup writes; foreground only.
  const char* check = "NOT_READ";
  uint32_t check_attempt = 0;
  uint8_t register_count = 0;
  uint8_t registers[8] = {};
  // Current attempt trace; count identifies the populated prefix.
  uint32_t trace_attempt = 0;
  uint8_t trace_count = 0;
  uint8_t config_trace[6] = {};
  uint8_t cleanup_config = 0; // belongs to check_attempt, not an unfinished trace
  Double resistance_ohms;
  Double temperature_c;
};

void max31865_init(); // once, after timepop_init(); explicitly selects SPI1 pins
const rtd_snapshot_t& max31865_snapshot(); // read-only, foreground only
const char* max31865_status_name(rtd_status_t status);
