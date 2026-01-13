#pragma once
#include <stdint.h>

// --------------------------------------------------------------
// GPT2 external pulse counter
//
// Design goals:
//   • Explicit GPT2 ownership
//   • External clock via pad + input select
//   • No interrupts required
//   • Pollable free-running counter
//   • Symmetric ARM / DISARM lifecycle
// --------------------------------------------------------------

// ---- GPT2 ----------------------------------------------------
void     gpt_count_arm();
void     gpt_count_disarm();
uint32_t gpt_count_read();
bool     gpt_count_status();

// --------------------------------------------------------------
// GPT2 external clock confirmation
//
// This function:
//   • Arms GPT2 internally
//   • Measures over a CPU-cycle-defined window
//   • Disarms GPT2 before returning
//   • Never depends on prior GPT state
// --------------------------------------------------------------
uint32_t gpt_count_confirm(
    uint32_t window_cpu_cycles,
    uint32_t* cpu_cycles_out,
    float* ratio_out
);

