#pragma once

#include <stdint.h>

// =============================================================
// TEMPEST — Cooperative Timing Process (Teensy side)
// =============================================================
//
// First-pass port of gpt_count.*
// Blocking, loop-based, GNSS-anchored
//
// DO NOT REFACTOR YET.
// =============================================================

// -------------------------------------------------------------
// CONFIRM
// -------------------------------------------------------------
// Single GNSS-anchored Monte Carlo draw
// seconds = wall-clock GNSS seconds
// -------------------------------------------------------------

uint64_t tempest_confirm(
    uint32_t seconds,
    uint64_t* cpu_cycles_out,
    double*   ratio_out,
    int64_t*  error_cycles_out
);

// -------------------------------------------------------------
// BASELINE
// -------------------------------------------------------------
// Discovers stable standard error using repeated 1s CONFIRM
// -------------------------------------------------------------

bool tempest_discover_standard_error(
    uint32_t* samples_out,
    int32_t*  min_error_out,
    int32_t*  max_error_out,
    int32_t*  med_error_out,
    int32_t*  std_error_out,
    double*   stddev_out
);

// -------------------------------------------------------------
// TAU
// -------------------------------------------------------------
// Full Monte Carlo tau profiling run
// total_seconds = profiling duration
// -------------------------------------------------------------

bool tempest_tau_profile(uint32_t total_seconds);

// -------------------------------------------------------------
// Process entry (command dispatcher)
// -------------------------------------------------------------

bool process_tempest_handle_command(
    const char* proc_cmd,
    const char* args_json,
    String&     reply_json
);

void process_tempest_register(void);
