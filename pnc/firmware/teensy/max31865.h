#pragma once

#include <stdint.h>
#include "double.h"

// Foreground-owned PT1000 acquisition. No ISR, allocation, delay(), or native FP.
enum class rtd_status_t : uint8_t {
  INITIALIZING, OK, SPI_ERROR, RTD_FAULT, OUT_OF_RANGE
};

struct rtd_spi_state_t {
  uint32_t sr = 0;
  uint32_t fsr = 0;
};

struct rtd_spi_trace_t {
  rtd_spi_state_t before; // before beginTransaction()
  rtd_spi_state_t after;  // after final transfer(), before CS hold/release
};

// Failure-onset evidence. attempt == 0 means none yet.
// Retained across retries and successful recovery; foreground-owned RAM1.
struct rtd_failure_t {
  uint32_t attempt = 0;
  uint32_t captured_at_ms = 0;
  uint32_t sample_sequence = 0;
  rtd_status_t status = rtd_status_t::INITIALIZING;
  const char* check = "NOT_READ";
  uint8_t checked_config = 0;
  uint8_t pre_clear_registers[8] = {};
  uint8_t registers[8] = {};
  uint8_t trace_count = 0;
  uint8_t config_trace[6] = {};
  rtd_spi_trace_t pre_clear_spi;
  rtd_spi_trace_t check_spi;
  rtd_spi_trace_t registers_spi;
  uint32_t spi_cr = 0;
  uint32_t spi_tcr = 0;
  uint32_t spi_ccr = 0;
  uint32_t spi_cfgr1 = 0;
  uint32_t ccm_cbcmr = 0;
};

struct rtd_snapshot_t {
  rtd_status_t status = rtd_status_t::INITIALIZING;
  uint32_t sample_sequence = 0;
  uint32_t attempts = 0;
  uint32_t errors = 0;
  uint32_t sampled_at_ms = 0; // millis() at result readout, not UTC
  uint16_t raw_code = 0;     // 15-bit resistance ratio
  uint8_t fault_status = 0;  // Last completed check's register 7, D7..D2
  // Last check evidence, captured before cleanup writes; foreground only.
  const char* check = "NOT_READ";
  uint32_t check_attempt = 0;
  uint8_t register_count = 0;
  uint8_t registers[8] = {};
  uint8_t checked_config = 0; // Original check, before diagnostic rereads
  rtd_failure_t first_failure;
  rtd_failure_t latest_failure_episode; // Onset after boot or a successful sample
  uint32_t failure_episodes = 0;
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
