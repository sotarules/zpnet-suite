#pragma once

#include <Arduino.h>

// -----------------------------------------------------------------------------
// Debug logging (USB Serial only)
//
// This module provides a simple, imperative debug output channel
// over USB Serial (Serial).
//
// Design goals:
//   • Out-of-band (does not use transport or event bus)
//   • No scheduling, no buffering, no framing
//   • Intended for human visibility via `screen`
//   • Safe to call from normal code paths (NOT ISRs)
//
// -----------------------------------------------------------------------------

// Initialize USB debug serial (call from setup())
void debug_init();

// Log a debug message
//   name: short tag (e.g. "CMD", "EVENTBUS", "PROCESS")
//   body: free-form message (JSON OK, but not required)
void debug_log(const char* name, const char* body);
