#include "process.h"
#include "events.h"
#include "payload.h"

#include <Arduino.h>
#include <string.h>

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

static const size_t MAX_PROCESSES = 8;

// -----------------------------------------------------------------------------
// Subsystem → process mapping (shared, canonical)
// -----------------------------------------------------------------------------

bool process_type_from_name(
  const char* subsystem,
  process_type_t& out
) {
  if (strcmp(subsystem, "CLOCKS")     == 0) { out = PROCESS_TYPE_CLOCKS;     return true; }
  if (strcmp(subsystem, "EVENTS")     == 0) { out = PROCESS_TYPE_EVENTS;     return true; }
  if (strcmp(subsystem, "TIMEPOP")    == 0) { out = PROCESS_TYPE_TIMEPOP;    return true; }
  if (strcmp(subsystem, "LASER")      == 0) { out = PROCESS_TYPE_LASER;      return true; }
  if (strcmp(subsystem, "PHOTODIODE") == 0) { out = PROCESS_TYPE_PHOTODIODE; return true; }
  if (strcmp(subsystem, "TEMPEST")    == 0) { out = PROCESS_TYPE_TEMPEST;    return true; }
  if (strcmp(subsystem, "LANTERN")    == 0) { out = PROCESS_TYPE_LANTERN;    return true; }
  if (strcmp(subsystem, "SYSTEM")     == 0) { out = PROCESS_TYPE_SYSTEM;     return true; }

  return false;  // programmer error → caller should not recover
}

// -----------------------------------------------------------------------------
// Internal Registry Entry
// -----------------------------------------------------------------------------

typedef struct {
  process_type_t          type;
  process_state_t         state;
  const process_vtable_t* vtable;
} process_entry_t;

// -----------------------------------------------------------------------------
// Registry State
// -----------------------------------------------------------------------------

static process_entry_t registry[MAX_PROCESSES];
static size_t registry_count = 0;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

static process_entry_t* find_process(process_type_t type) {
  for (size_t i = 0; i < registry_count; ++i) {
    if (registry[i].type == type) {
      return &registry[i];
    }
  }
  return nullptr;
}

static const process_command_entry_t*
find_command(const process_vtable_t* vtable, const char* name) {
  for (size_t i = 0; i < vtable->command_count; ++i) {
    if (strcmp(vtable->commands[i].name, name) == 0) {
      return &vtable->commands[i];
    }
  }
  return nullptr;
}

// -----------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------

void process_init(void) {
  registry_count = 0;
}

// -----------------------------------------------------------------------------
// Registration
// -----------------------------------------------------------------------------

bool process_register(
  process_type_t type,
  const process_vtable_t* vtable
) {
  if (registry_count >= MAX_PROCESSES) {
    return false;
  }

  if (find_process(type)) {
    return false;
  }

  registry[registry_count].type   = type;
  registry[registry_count].state  = PROCESS_STATE_STOPPED;
  registry[registry_count].vtable = vtable;
  registry_count++;

  return true;
}

// -----------------------------------------------------------------------------
// Start / Stop
// -----------------------------------------------------------------------------

bool process_start(process_type_t type) {
  process_entry_t* p = find_process(type);
  if (!p) return false;

  if (p->state == PROCESS_STATE_RUNNING) {
    return true;
  }

  bool ok = p->vtable->start();

  Payload ev;
  ev.add("name", p->vtable->name);

  if (ok) {
    p->state = PROCESS_STATE_RUNNING;
    enqueueEvent("PROCESS_STARTED", ev);
  } else {
    p->state = PROCESS_STATE_ERROR;
    enqueueEvent("PROCESS_ERROR", ev);
  }

  return ok;
}

bool process_stop(process_type_t type) {
  process_entry_t* p = find_process(type);
  if (!p) return false;

  if (p->state != PROCESS_STATE_RUNNING) {
    return true;
  }

  p->vtable->stop();
  p->state = PROCESS_STATE_STOPPED;

  Payload ev;
  ev.add("name", p->vtable->name);
  enqueueEvent("PROCESS_STOPPED", ev);

  return true;
}

// -----------------------------------------------------------------------------
// Command Dispatch (Canonical)
// -----------------------------------------------------------------------------

void process_command(
  process_type_t type,
  const char*    cmd_name,
  const char*    args_json,
  String&        out_response
) {
  process_entry_t* p = find_process(type);
  if (!p) {
    out_response = "{\"success\":false,\"message\":\"unknown subsystem\"}";
    return;
  }

  const process_command_entry_t* entry =
      find_command(p->vtable, cmd_name);

  if (!entry || !entry->handler) {
    out_response = "{\"success\":false,\"message\":\"unknown command\"}";
    return;
  }

  // Invoke handler (new contract)
  const Payload* payload = entry->handler(args_json);

  // -----------------------------------------------------------------
  // Build response envelope (single authority)
  // -----------------------------------------------------------------

  out_response = "{";
  out_response += "\"success\":true,\"message\":\"OK\"";

  if (payload) {
    out_response += ",\"payload\":";
    out_response += payload->to_json();
  }

  out_response += "}";
}

// -----------------------------------------------------------------------------
// Registry Introspection
// -----------------------------------------------------------------------------

String process_list_json(void) {

  Payload root;

  // Build array manually (Payload is object-only by design)
  String arr;
  arr += "[";

  for (size_t i = 0; i < registry_count; ++i) {
    if (i) arr += ",";

    Payload p;
    p.add("name", registry[i].vtable->name);

    switch (registry[i].state) {
      case PROCESS_STATE_RUNNING: p.add("state", "RUNNING"); break;
      case PROCESS_STATE_STOPPED: p.add("state", "STOPPED"); break;
      case PROCESS_STATE_ERROR:   p.add("state", "ERROR");   break;
    }

    arr += p.to_json();
  }

  arr += "]";

  // Root object
  String out;
  out += "{";
  out += "\"processes\":";
  out += arr;
  out += "}";

  return out;
}
