#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

#include "payload.h"

// --------------------------------------------------
// Command Handler Contract (AUTHORITATIVE)
// --------------------------------------------------
//
// Canonical contract:
//
//   • Command handlers return `const Payload*`
//   • Returning nullptr means:
//       - side-effect only
//       - no payload field is emitted in the response
//   • Handlers receive structured arguments as Payload
//   • Handlers MUST NOT perform serialization
//

using process_command_fn =
  const Payload* (*)(const Payload& args);

// --------------------------------------------------
// Process Command Declaration
// --------------------------------------------------

typedef struct {
  const char*          name;
  process_command_fn   handler;
} process_command_entry_t;

// --------------------------------------------------
// Process VTable (Declarative)
// --------------------------------------------------
//
// A process is a named command surface.
// It has no implicit lifecycle.
//

typedef struct {
  const char* name;

  // Optional introspection (legacy / transitional)
  String (*query)(void);

  // Command surface
  const process_command_entry_t* commands;
  size_t                         command_count;

} process_vtable_t;

// --------------------------------------------------
// Public API
// --------------------------------------------------

// Reset the process registry (called once at boot)
void process_init(void);

// Register a process command surface
bool process_register(
  const char* process_id,
  const process_vtable_t* vtable
);

// --------------------------------------------------
// Registry Introspection (READ-ONLY, DIAGNOSTIC)
// --------------------------------------------------
//
// These functions expose the registered process set
// for SYSTEM-level inspection only.
//

size_t      process_get_count(void);
const char* process_get_name(size_t idx);

// --------------------------------------------------
// Transport-facing command processor
// --------------------------------------------------
//
// This function IS the semantic entry point for
// TRAFFIC_REQUEST_RESPONSE.
//

void process_command(const Payload& request);
