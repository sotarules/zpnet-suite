#pragma once
#include <stdint.h>

// --------------------------------------------------------------
// Quad Timer (QTIMER) external pulse counter
//
// Design goals:
//   • User-domain peripheral (safe on Teensy 4.1)
//   • Explicit pad muxing and pad control
//   • No interrupts
//   • Pollable free-running counter
//   • Minimal, auditable lifecycle
// --------------------------------------------------------------

// ---- QTIMER3 TIMER1 ------------------------------------------
void     qtimer_arm();
void     qtimer_disarm();
uint32_t qtimer_read();
void     qtimer_clear();
bool     qtimer_status();
