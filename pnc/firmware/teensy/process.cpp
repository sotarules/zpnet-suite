#include "process.h"
#include "util.h"
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
//
// Semantics:
//   • Accepts a fully parsed semantic request Payload
//   • Routes by subsystem + command
//   • Invokes the registered handler with optional args Payload
//   • Emits a canonical response envelope via transport
//
// Notes:
//   • request.getString() returns a pointer backed by a shared static buffer
//     in the current Payload implementation, so we COPY into String immediately.
//   • We validate subsystem existence BEFORE attempting to use p->vtable.
// -----------------------------------------------------------------------------

void process_command(const Payload& request) {

  Payload response;

  debug_log("process_command", "command received");
  debug_log_payload("process_command", request);

  // ---------------------------------------------------------
  // Extract routing fields (COPY IMMEDIATELY)
  // ---------------------------------------------------------

  String subsystem = request.getString("subsystem");
  String command   = request.getString("command");

  debug_log("subsystem", subsystem.c_str());
  debug_log("command", command.c_str());

  // Optional forensic dump (remove when stable)
  request.debug_dump("process_command request");

  // ---------------------------------------------------------
  // Resolve subsystem
  // ---------------------------------------------------------

  process_entry_t* p = find_process(subsystem.c_str());
  if (!p) {
    debug_log("process_command", "***SUBSYSTEM NOT FOUND***");
    response.add("success", false);
    response.add("message", "unknown subsystem");
    transport_send(TRAFFIC_REQUEST_RESPONSE, response);
    return;
  }

  // ---------------------------------------------------------
  // Resolve command
  // ---------------------------------------------------------

  debug_log("process_command", "resolving command");

  const process_command_entry_t* entry =
      find_command(p->vtable, command.c_str());

  if (!entry || !entry->handler) {
    debug_log("process_command", "***COMMAND NOT FOUND***");
    response.add("success", false);
    response.add("message", "unknown command");
    transport_send(TRAFFIC_REQUEST_RESPONSE, response);
    return;
  }

  debug_log("process_command", "executing command");

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
  if (payload) {
    String json = payload->to_json();
    debug_log("process_command.payload", json.c_str());
  } else {
    debug_log("process_command.payload", "(null)");
  }

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
