#include "process.h"
#include "payload.h"

#include <Arduino.h>
#include <string.h>

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

static const size_t MAX_PROCESSES = 8;

// -----------------------------------------------------------------------------
// Internal Registry Entry
// -----------------------------------------------------------------------------

typedef struct {
  const char*             id;      // subsystem name, e.g. "CLOCKS"
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

static process_entry_t* find_process(const char* process_id) {
  if (!process_id) return nullptr;

  for (size_t i = 0; i < registry_count; ++i) {
    if (strcmp(registry[i].id, process_id) == 0) {
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
// Registry Initialization
// -----------------------------------------------------------------------------

void process_init(void) {
  registry_count = 0;
}

// -----------------------------------------------------------------------------
// Registration
// -----------------------------------------------------------------------------

bool process_register(
  const char*             process_id,
  const process_vtable_t* vtable
) {
  if (!process_id || !vtable || !vtable->name) {
    return false;
  }

  // Enforce single-identity invariant:
  // process_id must equal vtable->name
  if (strcmp(process_id, vtable->name) != 0) {
    return false;
  }

  if (registry_count >= MAX_PROCESSES) {
    return false;
  }

  if (find_process(process_id)) {
    return false;
  }

  registry[registry_count].id     = process_id;
  registry[registry_count].vtable = vtable;
  registry_count++;

  return true;
}

// -----------------------------------------------------------------------------
// Command Dispatch (Canonical)
// -----------------------------------------------------------------------------

void process_command(
  const char* process_id,
  const char* cmd_name,
  const char* args_json,
  String&     out_response
) {
  process_entry_t* p = find_process(process_id);
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

  // Invoke handler
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

  // Build array manually (Payload is object-only by design)
  String arr;
  arr += "[";

  for (size_t i = 0; i < registry_count; ++i) {
    if (i) arr += ",";

    Payload p;
    p.add("name", registry[i].id);

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
