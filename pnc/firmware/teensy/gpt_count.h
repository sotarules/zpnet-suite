#pragma once
#include <stdint.h>

// --------------------------------------------------------------
// GPT2 external pulse counter (FreqCount-style)
//
// Design goals:
//   • Explicit GPT2 ownership
//   • External clock via pad + input select
//   • No interrupts required
//   • Pollable free-running counter
//   • API-compatible with qtimer
// --------------------------------------------------------------

// ---- GPT2 ----------------------------------------------------
void     gpt_count_arm();
uint32_t gpt_count_read();
bool     gpt_count_status();
