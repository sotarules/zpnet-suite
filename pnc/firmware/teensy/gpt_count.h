#pragma once
#include <stdint.h>
#include <stdbool.h>

// --------------------------------------------------------------
// GPT2 edge-anchored confirmation (LEGACY, UNCHANGED)
//
// This function:
//   • Arms GPT2 internally (idempotent)
//   • Anchors measurement to GNSS edges via GPIO interrupt
//   • Measures CPU cycles between two external clock edges
//   • Uses symmetric ISR entry to cancel latency
//   • Disarms GPT2 before returning
//
// Measurement semantics:
//   • Window is defined by external clock, not CPU time
//   • CPU acts strictly as observer
//
// This remains a SINGLE-SAMPLE primitive.
// --------------------------------------------------------------
uint64_t gpt_count_confirm(
    uint64_t  target_ext_ticks,
    uint64_t* cpu_cycles_out,
    double*   ratio_out,
    int64_t*  error_cycles_out
);

// --------------------------------------------------------------
// τ (TAU) PROFILING — MONTE CARLO LAYER (NEW)
//
// This is a higher-level wrapper built on top of
// gpt_count_confirm().
//
// Semantics:
//   • One GNSS-anchored window = one Monte Carlo sample
//   • Samples accumulate into a bounded histogram
//   • Histogram reduces to a scalar τ
//   • τ is dimensionless and normalized:
//
//       τ ≈ 1.0  → baseline (sea level)
//       τ > 1.0  → faster local tempo
//       τ < 1.0  → slower local tempo
//
// Architectural guarantees:
//   • GNSS remains authoritative
//   • No mid-window GPT manipulation
//   • Legacy confirm logic is preserved intact
//   • Noise is treated as signal
//
// Parameters:
//   • total_seconds  — total profiling duration
//   • sample_seconds — GNSS-defined window per sample
//   • span_cycles    — ±cycle range captured by histogram
//   • bucket_width   — downsampling factor (cycles per bin)
//
// Emits:
//   • TAU_RESULT events periodically (durable)
//
// Returns:
//   • true  on successful completion
//   • false on invalid parameters
// --------------------------------------------------------------
bool gpt_tau_profile(
    uint32_t total_seconds,
    uint32_t sample_seconds,
    int32_t  span_cycles,
    uint32_t bucket_width
);

