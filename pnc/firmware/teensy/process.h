#pragma once

#include <Arduino.h>
#include <stdint.h>

// --------------------------------------------------
// Process Types
// --------------------------------------------------
typedef enum {
  PROCESS_TYPE_NONE = 0,
  PROCESS_TYPE_GNSS,
  PROCESS_TYPE_TEMPEST,
  PROCESS_TYPE_LANTERN,
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
// Process VTable
// --------------------------------------------------
typedef struct {
  const char* name;
  bool   (*start)(void);
  void   (*stop)(void);
  String (*query)(void);
  String (*command)(const char* cmd, const char* args);
} process_vtable_t;

// --------------------------------------------------
// Public API (DECLARATIONS ONLY)
// --------------------------------------------------
void process_init(void);

bool process_register(
  process_type_t type,
  const process_vtable_t* vtable
);

bool process_start(process_type_t type);
bool process_stop(process_type_t type);

bool process_query(
  process_type_t type,
  String& out_body
);

bool process_command(
  process_type_t type,
  const char* cmd,
  const char* args,
  String& out_body
);

String process_list_json(void);
