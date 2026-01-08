#pragma once

#include <Arduino.h>

// --------------------------------------------------------------
// System lifecycle and terminal state
// --------------------------------------------------------------
//
// This module owns irreversible transitions.
// Once shutdown is requested, no further behavior is permitted.
//

// Initialize system-level state
void system_init();

// Request terminal shutdown (no return)
void system_request_shutdown();

// Enter permanent quiescence (never returns)
void system_enter_quiescence();

// Query terminal state
bool system_is_shutdown();
