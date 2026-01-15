#pragma once

#include <Arduino.h>

// --------------------------------------------------------------
// Command execution interface
// --------------------------------------------------------------
//
// Commands arrive as single-line JSON objects containing "cmd".
//
// Optional arguments may be supplied via an "args" object.
//
// Examples:
//
//   { "cmd": "TEMPO.START" }
//   { "cmd": "TEMPO.START", "args": { "altitude_m": 1234.5 } }
//
// Semantics:
//   CMD        → enqueue durable event(s)
//   CMD?       → immediate, non-destructive reply (no queue)
//   TERMINAL   → irreversible state change (no return)
//
// This module performs parsing and dispatch only.
// All domain logic lives in subsystems.
//

// Execute a single command line (null-terminated)
void command_exec(const char* line);
