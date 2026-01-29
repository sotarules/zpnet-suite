#include "process.h"

#include "transport.h"
#include "debug.h"

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

  // Enforce single-identity invariant
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
// Registry Introspection (READ-ONLY)
// -----------------------------------------------------------------------------

size_t process_get_count(void) {
  return registry_count;
}

const char* process_get_name(size_t idx) {
  if (idx >= registry_count) return nullptr;
  return registry[idx].id;
}

// -----------------------------------------------------------------------------
// Unified REQUEST / RESPONSE command processor
// -----------------------------------------------------------------------------

void process_command(const Payload& request) {

  Payload response;

  // ---------------------------------------------------------
  // Extract routing fields
  // ---------------------------------------------------------

  const char* process = request.getString("process");
  const char* command = request.getString("command");

  // ---------------------------------------------------------
  // Resolve subsystem
  // ---------------------------------------------------------

  process_entry_t* p = find_process(process);
  if (!p) {
    response.add("success", false);
    response.add("message", "unknown subsystem");
    transport_send(TRAFFIC_REQUEST_RESPONSE, response);
    return;
  }

  // ---------------------------------------------------------
  // Resolve command
  // ---------------------------------------------------------

  const process_command_entry_t* entry =
      find_command(p->vtable, command);

  if (!entry || !entry->handler) {
    response.add("success", false);
    response.add("message", "unknown command");
    transport_send(TRAFFIC_REQUEST_RESPONSE, response);
    return;
  }

  // ---------------------------------------------------------
  // Extract args (structured, optional)
  // ---------------------------------------------------------

  Payload args;
  if (request.has("args")) {
    args = request.getPayload("args");
  }

  // ---------------------------------------------------------
  // Invoke command handler
  // ---------------------------------------------------------

  const Payload* payload = entry->handler(args);

  // ---------------------------------------------------------
  // Construct canonical success envelope
  // ---------------------------------------------------------

  response.add("success", true);
  response.add("message", "OK");

  if (payload) {
    response.add_object("payload", *payload);
  }

  // ---------------------------------------------------------
  // Emit response
  // ---------------------------------------------------------

  transport_send(
    TRAFFIC_REQUEST_RESPONSE,
    response
  );
}
