#pragma once
#include <stdint.h>

// --------------------------------------------------------------
// GPT2 edge-anchored confirmation
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
// --------------------------------------------------------------
uint32_t gpt_count_confirm(
    uint32_t target_ext_ticks,
    uint32_t* cpu_cycles_out,
    float* ratio_out
);
