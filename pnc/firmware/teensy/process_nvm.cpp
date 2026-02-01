// ============================================================================
// FILE: process_nvm.cpp
// ============================================================================
//
// NVM Process — Durable KV Store Control Plane
//
// Commands:
//   • READ
//   • WRITE
//   • DELETE
//   • EXISTS
//   • LIST   (diagnostic)
//
// ============================================================================

#include "process_nvm.h"

#include "process.h"
#include "payload.h"
#include "nvm.h"

// ------------------------------------------------------------
// READ
// ------------------------------------------------------------
static Payload cmd_read(const Payload& args) {

  Payload resp;

  if (!args.has("key")) {
    resp.add("error", "missing key");
    return resp;
  }

  const char* key = args.getString("key");
  Payload out;

  if (!nvm_read(key, out)) {
    resp.add("error", "not found");
    return resp;
  }

  return out;
}

// ------------------------------------------------------------
// WRITE
// ------------------------------------------------------------
static Payload cmd_write(const Payload& args) {

  if (!args.has("key") || !args.has("payload")) {
    Payload err;
    err.add("error", "missing key or payload");
    return err;
  }

  const char* key = args.getString("key");
  Payload value = args.getPayload("payload");

  if (!nvm_write(key, value)) {
    Payload err;
    err.add("error", "write failed");
    return err;
  }

  return ok_payload();
}

// ------------------------------------------------------------
// DELETE
// ------------------------------------------------------------
static Payload cmd_delete(const Payload& args) {

  Payload resp;

  if (!args.has("key")) {
    resp.add("error", "missing key");
    return resp;
  }

  const char* key = args.getString("key");
  resp.add("deleted", nvm_delete(key));
  return resp;
}

// ------------------------------------------------------------
// EXISTS
// ------------------------------------------------------------
static Payload cmd_exists(const Payload& args) {

  Payload resp;

  if (!args.has("key")) {
    resp.add("error", "missing key");
    return resp;
  }

  const char* key = args.getString("key");
  resp.add("exists", nvm_exists(key));
  return resp;
}

// ------------------------------------------------------------
// LIST (diagnostic)
// ------------------------------------------------------------
static Payload cmd_list(const Payload&) {
  nvm_debug_dump();
  return ok_payload();
}

// ------------------------------------------------------------
// Registration
// ------------------------------------------------------------

static const process_command_entry_t NVM_COMMANDS[] = {
  { "READ",   cmd_read   },
  { "WRITE",  cmd_write  },
  { "DELETE", cmd_delete },
  { "EXISTS", cmd_exists },
  { "LIST",   cmd_list   },
  { nullptr,  nullptr }
};

static const process_vtable_t NVM_PROCESS = {
  .process_id    = "NVM",
  .commands      = NVM_COMMANDS,
  .subscriptions = nullptr,
};

void process_nvm_register(void) {
  process_register("NVM", &NVM_PROCESS);
}
