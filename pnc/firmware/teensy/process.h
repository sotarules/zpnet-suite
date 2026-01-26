#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

// --------------------------------------------------
// Process Types
// --------------------------------------------------
typedef enum {
  PROCESS_TYPE_NONE = 0,
  PROCESS_TYPE_CLOCKS,
  PROCESS_TYPE_EVENTS,
  PROCESS_TYPE_TIMEPOP,
  PROCESS_TYPE_LASER,
  PROCESS_TYPE_PHOTODIODE,
  PROCESS_TYPE_TEMPEST,
  PROCESS_TYPE_LANTERN,
  PROCESS_TYPE_SYSTEM
} process_type_t;

// --------------------------------------------------
// Process State
// --------------------------------------------------
typedef enum {
  PROCESS_STATE_STOPPED = 0,
  PROCESS_STATE_RUNNING,
  PROCESS_STATE_ERROR,
} process_state_t;

// --------------------------------------------------
// Command Response Contract
// --------------------------------------------------

typedef const String* (*process_command_fn)(
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
// Process VTable
// --------------------------------------------------
//
// Each process explicitly declares its command surface.
// No command name inference, no parser heuristics.
//

typedef struct {
  const char* name;

  // Lifecycle
  bool (*start)(void);
  void (*stop)(void);

  // Introspection (optional but recommended)
  String (*query)(void);

  // Command surface
  const process_command_entry_t* commands;
  size_t                         command_count;

} process_vtable_t;

// --------------------------------------------------
// Public API
// --------------------------------------------------

void process_init(void);

bool process_register(
  process_type_t type,
  const process_vtable_t* vtable
);

bool process_start(process_type_t type);
bool process_stop(process_type_t type);

// Canonical command invocation
void process_command(
  process_type_t type,
  const char*    cmd_name,
  const char*    args_json,
  String&        out_response
);

// Registry introspection
String process_list_json(void);

// --------------------------------------------------
// Subsystem name → process type mapping
// --------------------------------------------------
//
// Returns true if the subsystem name is recognized and sets `out`.
// Returns false for unknown subsystem names.
//
bool process_type_from_name(
  const char* subsystem,
  process_type_t& out
);
