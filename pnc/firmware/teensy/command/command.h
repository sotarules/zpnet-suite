#pragma once

#include <Arduino.h>

// --------------------------------------------------------------
// Command execution interface
// --------------------------------------------------------------
//
// Commands arrive as single-line JSON objects containing "cmd".
//
// Optional arguments may be supplied via additional JSON fields.
//
// Examples:
//
//   { "cmd": "TEMPO.START" }
//   { "cmd": "GPT.CONFIRM", "seconds": 10 }
//
// Semantics:
//
//   CMD        → enqueue durable event(s)
//   CMD?       → immediate, non-destructive reply (no queue)
//   TERMINAL   → irreversible state change (no return)
//
// This module:
//   • Parses commands
//   • Routes them to subsystems
//   • Emits events or immediate replies
//
// It does NOT:
//   • Own the loop
//   • Perform timing-critical work
//   • Maintain state beyond parsing
//

// Execute a single command line (null-terminated)
void command_exec(const char* line);
