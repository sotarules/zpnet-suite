#pragma once
#include <stdint.h>

// --------------------------------------------------------------
// GPT2 external pulse counter
//
// Design goals:
//   • Explicit GPT2 ownership
//   • External clock via pad + input select
//   • Free-running external edge ledger
//   • Pollable counter (no interrupts)
//   • Confirm logic uses edge-anchored timing
// --------------------------------------------------------------

// ---- GPT2 ----------------------------------------------------
void     gpt_count_arm();
void     gpt_count_disarm();
uint32_t gpt_count_read();
bool     gpt_count_status();

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
