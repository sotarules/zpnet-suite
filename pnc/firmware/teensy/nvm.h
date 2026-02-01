// ============================================================================
// FILE: nvm.h
// ============================================================================
//
// NVM — Durable Key / Payload Store
//
// Semantics:
//   • Key → complete Payload (JSON object)
//   • Writes clobber existing values
//   • Reads reconstruct full Payload
//   • No partial updates
//   • No implicit defaults
//
// Backend:
//   • LittleFS on Teensy 4.1 QSPI flash
//
// ============================================================================

#pragma once

#include "payload.h"

// ------------------------------------------------------------
// Lifecycle
// ------------------------------------------------------------

// Initialize NVM subsystem.
// Must be called once during boot before use.
bool nvm_init(void);

// ------------------------------------------------------------
// CRUD
// ------------------------------------------------------------

// Read payload for key.
// Returns false if key does not exist or data is invalid.
bool nvm_read(
  const char* key,
  Payload&    out
);

// Write payload for key (clobbers any existing value).
// Returns false on failure.
bool nvm_write(
  const char* key,
  const Payload& value
);

// Delete key if present.
// Returns true if key existed and was removed.
bool nvm_delete(
  const char* key
);

// ------------------------------------------------------------
// Introspection
// ------------------------------------------------------------

// Check whether a key exists.
bool nvm_exists(const char* key);

// Emit diagnostic dump (debug only).
void nvm_debug_dump(void);