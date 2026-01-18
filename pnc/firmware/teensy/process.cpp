#include "process.h"
#include "event_bus.h"

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

static const size_t MAX_PROCESSES = 8;

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
  if (!vtable || !name) return nullptr;

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
  if (!vtable || !vtable->name || !vtable->start || !vtable->stop) {
    return false;
  }

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

  String body;
  body += "\"name\":\"";
  body += p->vtable->name;
  body += "\"";

  if (ok) {
    p->state = PROCESS_STATE_RUNNING;
    enqueueEvent("PROCESS_STARTED", body);
  } else {
    p->state = PROCESS_STATE_ERROR;
    enqueueEvent("PROCESS_ERROR", body);
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

  String body;
  body += "\"name\":\"";
  body += p->vtable->name;
  body += "\"";

  enqueueEvent("PROCESS_STOPPED", body);
  return true;
}

// -----------------------------------------------------------------------------
// Introspection
// -----------------------------------------------------------------------------

bool process_query(
  process_type_t type,
  String& out_body
) {
  process_entry_t* p = find_process(type);
  if (!p || !p->vtable->query) return false;

  out_body = "{";
  out_body = "\"name\":\"";
  out_body += p->vtable->name;
  out_body += "\",\"state\":\"";

  switch (p->state) {
    case PROCESS_STATE_RUNNING: out_body += "RUNNING"; break;
    case PROCESS_STATE_STOPPED: out_body += "STOPPED"; break;
    case PROCESS_STATE_ERROR:   out_body += "ERROR";   break;
  }

  out_body += "\"";

  String inner = p->vtable->query();
  if (inner.length()) {
    out_body += ",";
    out_body += inner;
  }

  out_body += "}";

  return true;
}

// -----------------------------------------------------------------------------
// Command Dispatch (Canonical)
// -----------------------------------------------------------------------------

bool process_command(
  process_type_t type,
  const char*    cmd_name,
  const char*    args_json,
  String&        out_response
) {
  process_entry_t* p = find_process(type);
  if (!p || !p->vtable || !p->vtable->commands) {
    out_response =
      "{\"success\":false,\"message\":\"process not found\"}";
    return false;
  }

  const process_command_entry_t* entry =
      find_command(p->vtable, cmd_name);

  if (!entry || !entry->handler) {
    out_response =
      "{\"success\":false,\"message\":\"unrecognized command\"}";
    return true;   // handled, but unsuccessful
  }

  // Invoke command handler
  out_response = entry->handler(args_json);
  return true;
}

// -----------------------------------------------------------------------------
// Registry Introspection
// -----------------------------------------------------------------------------

String process_list_json(void) {

  String out;
  out += "{";

  out += "\"processes\":[";

  for (size_t i = 0; i < registry_count; ++i) {
    if (i) out += ",";

    out += "{";
    out += "\"name\":\"";
    out += registry[i].vtable->name;
    out += "\",\"state\":\"";

    switch (registry[i].state) {
      case PROCESS_STATE_RUNNING: out += "RUNNING"; break;
      case PROCESS_STATE_STOPPED: out += "STOPPED"; break;
      case PROCESS_STATE_ERROR:   out += "ERROR";   break;
    }

    out += "\"}";
  }

  out += "]";

  out += "}";
  return out;
}
