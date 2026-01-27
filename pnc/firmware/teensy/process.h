#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

#include "payload.h"

// --------------------------------------------------
// Command Response Contract
// --------------------------------------------------
//
// Canonical contract:
//
//   • Command handlers return `const Payload*`
//   • Returning nullptr means:
//       - side-effect only
//       - no payload field is emitted in the response
//   • Handlers MUST NOT perform serialization
//

typedef const Payload* (*process_command_fn)(
  const char* args_json   // may be nullptr
);

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
// If a process requires initialization or shutdown,
// it must expose explicit commands to do so.
//

typedef struct {
  const char* name;

  // Optional introspection (legacy / transitional)
  // NOTE: Prefer explicit REPORT commands instead.
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

bool process_register(
  const char* process_id,
  const process_vtable_t* vtable
);


// Canonical command invocation
// Builds full JSON response envelope internally.
void process_command(
  const char*    process_id,
  const char*    cmd_name,
  const char*    args_json,
  String&        out_response
);

// Registry introspection (presence only)
String process_list_json(void);

