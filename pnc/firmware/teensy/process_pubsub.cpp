// =============================================================
// FILE: process_pubsub.cpp
// =============================================================
//
// PUBSUB Process — NVM-backed subscription control plane
//
// Responsibilities:
//   • Ingress for TRAFFIC_PUBLISH_SUBSCRIBE (0xD2)
//   • Control-plane mutation of subscription state (NVM)
//   • Delegation to process_publish_dispatch()
//
// Non-responsibilities:
//   • Routing
//   • Fan-out
//   • Handler ownership
//   • Transport policy
//
// =============================================================

#include "process_pubsub.h"

#include "process.h"
#include "publish.h"
#include "payload.h"
#include "nvm.h"
#include "debug.h"

#include <string.h>

// -------------------------------------------------------------
// NVM key (authoritative)
// -------------------------------------------------------------

static constexpr const char* NVM_SUBSCRIPTIONS_KEY = "subscriptions";

// ================================================================
// Ingress: publications arriving from Pi
// ================================================================
//
// Payload shape (authoritative):
//   {
//     "topic":   "...",
//     "payload": { ... }
//   }
//
// Registered with transport for TRAFFIC_PUBLISH_SUBSCRIBE.
//

void process_publish_dispatch(const Payload& message) {

  if (!message.has("topic")) {
    return;
  }

  const char* topic = message.getString("topic");
  if (!topic || !*topic) {
    return;
  }

  Payload payload;
  if (message.has("payload")) {
    payload = message.getPayload("payload");
  }

  // Semantic local delivery only (NVM-backed in process.cpp)
  process_publish_dispatch(topic, payload);
}

// ================================================================
// Helpers (NVM subscription mutation)
// ================================================================

static Payload load_subscriptions() {
  Payload subs;
  nvm_read(NVM_SUBSCRIPTIONS_KEY, subs);
  return subs;
}

static bool save_subscriptions(const Payload& subs) {
  return nvm_write(NVM_SUBSCRIPTIONS_KEY, subs);
}

static bool topic_exists(const PayloadArray& arr, const char* topic) {
  for (size_t i = 0; i < arr.size(); i++) {
    Payload e = arr.get(i);
    const char* t = e.getString("topic");
    if (t && strcmp(t, topic) == 0) {
      return true;
    }
  }
  return false;
}

// ================================================================
// Commands
// ================================================================

// ------------------------------------------------------------
// PUBLISH — invoke local + Pi-forward publish path
// ------------------------------------------------------------
static Payload cmd_publish(const Payload& args) {

  if (!args.has("topic")) {
    Payload err;
    err.add("error", "missing topic");
    return err;
  }

  const char* topic = args.getString("topic");
  if (!topic || !*topic) {
    Payload err;
    err.add("error", "invalid topic");
    return err;
  }

  Payload payload;
  if (args.has("payload")) {
    payload = args.getPayload("payload");
  }

  publish(topic, payload);
  return ok_payload();
}

// ------------------------------------------------------------
// SUBSCRIBE — persist subscription in NVM
//
// Args:
//   {
//     "process": "...",
//     "topic":   "..."
//   }
// ------------------------------------------------------------
static Payload cmd_subscribe(const Payload& args) {

  if (!args.has("process") || !args.has("topic")) {
    Payload err;
    err.add("error", "missing process or topic");
    return err;
  }

  const char* process = args.getString("process");
  const char* topic   = args.getString("topic");

  if (!process || !*process || !topic || !*topic) {
    Payload err;
    err.add("error", "invalid process or topic");
    return err;
  }

  Payload subs = load_subscriptions();

  PayloadArray arr;
  if (subs.has(process)) {
    arr = subs.getArray(process);
  }

  if (!topic_exists(arr, topic)) {
    Payload entry;
    entry.add("topic", topic);
    arr.add(entry);
  }

  subs.add_array(process, arr);
  save_subscriptions(subs);

  Payload resp;
  resp.add("status", "subscribed");
  resp.add("process", process);
  resp.add("topic", topic);
  return resp;
}

// ------------------------------------------------------------
// UNSUBSCRIBE — remove subscription from NVM
//
// Args:
//   {
//     "process": "...",
//     "topic":   "..."
//   }
// ------------------------------------------------------------
static Payload cmd_unsubscribe(const Payload& args) {

  if (!args.has("process") || !args.has("topic")) {
    Payload err;
    err.add("error", "missing process or topic");
    return err;
  }

  const char* process = args.getString("process");
  const char* topic   = args.getString("topic");

  Payload subs;
  if (!nvm_read(NVM_SUBSCRIPTIONS_KEY, subs)) {
    Payload err;
    err.add("error", "no subscriptions defined");
    return err;
  }

  if (!subs.has(process)) {
    Payload err;
    err.add("error", "process not subscribed");
    return err;
  }

  PayloadArray old_arr = subs.getArray(process);
  PayloadArray new_arr;

  for (size_t i = 0; i < old_arr.size(); i++) {
    Payload e = old_arr.get(i);
    const char* t = e.getString("topic");
    if (t && strcmp(t, topic) != 0) {
      new_arr.add(e);
    }
  }

  subs.add_array(process, new_arr);
  save_subscriptions(subs);

  Payload resp;
  resp.add("status", "unsubscribed");
  resp.add("process", process);
  resp.add("topic", topic);
  return resp;
}

// ------------------------------------------------------------
// REPORT — reflect NVM-backed subscription truth
// ------------------------------------------------------------
static Payload cmd_report(const Payload&) {

  Payload resp;
  Payload subs;

  if (!nvm_read(NVM_SUBSCRIPTIONS_KEY, subs)) {
    resp.add("subscriptions", "none");
    return resp;
  }

  resp.add_object("subscriptions", subs);
  return resp;
}

// ================================================================
// Registration
// ================================================================

static const process_command_entry_t PUBSUB_COMMANDS[] = {
  { "PUBLISH",     cmd_publish     },
  { "SUBSCRIBE",   cmd_subscribe   },
  { "UNSUBSCRIBE", cmd_unsubscribe },
  { "REPORT",      cmd_report      },
  { nullptr,       nullptr         }
};

static const process_vtable_t PUBSUB_PROCESS = {
  .process_id    = "PUBSUB",
  .commands      = PUBSUB_COMMANDS,
  .subscriptions = nullptr
};

void process_pubsub_register(void) {
  process_register("PUBSUB", &PUBSUB_PROCESS);
}
