#pragma once
#include <stdint.h>
#include <stdbool.h>

// --------------------------------------------------------------
// CONFIRM — SINGLE GNSS-ANCHORED SAMPLE (SECONDS-BASED)
//
// Semantics:
//   • Measures CPU state transitions over an exact GNSS-defined
//     interval expressed in seconds
//   • GNSS VCLOCK (10 MHz) defines the authoritative window
//   • CPU acts strictly as observer
//   • Jitter is preserved intentionally
//
// This function is a single Monte Carlo draw.
// --------------------------------------------------------------
uint64_t gpt_count_confirm(
    uint32_t seconds,
    uint64_t* cpu_cycles_out,
    double*   ratio_out,
    int64_t*  error_cycles_out
);

// --------------------------------------------------------------
// STANDARD ERROR DISCOVERY — SELF-CALIBRATING BASELINE
//
// Semantics:
//   • Repeatedly invokes gpt_count_confirm() to observe the
//     absolute GNSS-anchored timing error of the local system
//   • Does NOT compute τ
//   • Does NOT perform residual subtraction
//   • Does NOT rely on user-specified warm-up durations
//
// The function runs until the absolute error distribution
// converges. Convergence is detected by observing stabilization
// of the distribution center (median) over a rolling window.
//
// This establishes the system's steady-state bias ("standard
// error") and characterizes its spread.
//
// Returned quantities are expressed in CPU cycles and represent
// absolute error relative to the ideal GNSS-defined interval.
//
// Outputs:
//   • samples_out   — total number of samples observed
//   • min_error_out — minimum observed absolute error
//   • max_error_out — maximum observed absolute error
//   • med_error_out — median absolute error (robust center)
//   • std_error_out — locked standard error (cycles)
//   • stddev_out    — standard deviation of absolute error
//
// All output pointers are optional and may be nullptr.
//
// Returns:
//   • true  — baseline successfully discovered
//   • false — unrecoverable failure (should be rare)
//
// This function defines the transition from discovery to
// precision measurement in TEMPEST.
// --------------------------------------------------------------
bool gpt_discover_standard_error(
    uint32_t* samples_out,
    int32_t*  min_error_out,
    int32_t*  max_error_out,
    int32_t*  med_error_out,
    int32_t*  std_error_out,
    double*   stddev_out
);

// --------------------------------------------------------------
// τ (TAU) PROFILING — MONTE CARLO LAYER (SELF-CALIBRATING)
//
// Built on top of gpt_count_confirm().
//
// Core idea:
//   • One GNSS-anchored window = one Monte Carlo sample
//   • Each sample yields an absolute error in CPU cycles
//   • A stable baseline ("standard error") is discovered first
//   • Residuals about that baseline are histogrammed at 1-cycle
//     granularity (no downsampling)
//   • The histogram reduces to a scalar τ plus diagnostics
//
// Instrument posture:
//   • Warm-up is not a duration. Warm-up is convergence.
//   • Baseline bias is discovered (median), then locked.
//   • Outliers are rejected via a fixed gate derived from the
//     baseline standard deviation.
//   • τ is computed from the mean absolute error:
//
//       mean_error = baseline_error + mean_residual
//       τ = ideal_cycles / (ideal_cycles + mean_error)
//
// Normalization:
//   • τ ≈ 1.0  → baseline tempo
//   • τ > 1.0  → faster local tempo
//   • τ < 1.0  → slower local tempo
//
// Parameters:
//   • total_seconds  — total profiling duration (seconds)
//
// Compatibility notes:
//   • span_cycles and bucket_width are retained for API stability,
//     but are ignored by the current implementation.
//   • Histogram span and gating are derived automatically from the
//     discovered baseline σ (typically span = ±4σ, gate = ±3σ).
//   • Histogram bins are 1 cycle wide by design.
// --------------------------------------------------------------
bool gpt_tau_profile(
    uint32_t total_seconds
);


