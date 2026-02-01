// =============================================================
// FILE: process_pubsub.cpp
// =============================================================
//
// PUBSUB Process — Teensy-side pub/sub control surface
//
// Responsibilities:
//   • Ingress for TRAFFIC_PUBLISH_SUBSCRIBE (0xD2)
//   • Diagnostic / experimental command surface
//   • Delegation to process_publish_dispatch()
//
// Non-responsibilities:
//   • Routing
//   • Fan-out to Pi processes
//   • Transport policy
//   • Subscription mutation
//
// =============================================================

#include "process_pubsub.h"

#include "process.h"
#include "publish.h"
#include "payload.h"
#include "debug.h"

#include <string.h>

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
// This function is registered directly with transport
// for TRAFFIC_PUBLISH_SUBSCRIBE.
//

void process_publish_dispatch(const Payload& message) {

  // Must have topic
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

  // Semantic local delivery only
  process_publish_dispatch(topic, payload);
}

// ================================================================
// Commands
// ================================================================

// ------------------------------------------------------------
// PUBLISH — invoke local + Pi-forward publish path
//
// Args:
//   {
//     "topic":   "...",
//     "payload": { ... }
//   }
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

  // Reuse canonical publish path
  publish(topic, payload);

  return ok_payload();
}

// ------------------------------------------------------------
// SUBSCRIBE — acknowledge intent only (diagnostic)
//
// Semantics:
//   • Experimental / non-authoritative
//   • Does NOT mutate subscription tables
//   • Exists for future control-plane exploration
// ------------------------------------------------------------
static Payload cmd_subscribe(const Payload& args) {

  Payload p;

  if (!args.has("topic")) {
    p.add("error", "missing topic");
    return p;
  }

  const char* topic = args.getString("topic");
  if (!topic || !*topic) {
    p.add("error", "invalid topic");
    return p;
  }

  p.add("status", "ACK");
  p.add("topic", topic);

  debug_log("pubsub.subscribe", topic);

  return p;
}

// ------------------------------------------------------------
// REPORT — pub/sub introspection snapshot
//
// Semantics:
//   • Stateless
//   • Read-only
//   • Reflects declared subscriptions only
// ------------------------------------------------------------
static Payload cmd_report(const Payload&) {

  Payload resp;
  PayloadArray processes;

  for (size_t i = 0; i < process_get_count(); i++) {

    const process_vtable_t* v = process_get_vtable(i);
    if (!v) continue;

    Payload proc;
    proc.add("name", v->process_id);

    PayloadArray subs;

    if (v->subscriptions) {
      for (const process_subscription_entry_t* s = v->subscriptions;
           s->topic;
           ++s) {
        Payload entry;
        entry.add("topic", s->topic);
        subs.add(entry);
      }
    }

    proc.add_array("subscriptions", subs);
    processes.add(proc);
  }

  resp.add_array("processes", processes);
  return resp;
}

// ================================================================
// Registration
// ================================================================

static const process_command_entry_t PUBSUB_COMMANDS[] = {
  { "PUBLISH",   cmd_publish   },
  { "SUBSCRIBE", cmd_subscribe },
  { "REPORT",    cmd_report    },
  { nullptr,     nullptr       }   // terminator
};

static const process_vtable_t PUBSUB_PROCESS = {
  .process_id   = "PUBSUB",
  .commands     = PUBSUB_COMMANDS,
  .subscriptions = nullptr
};

void process_pubsub_register(void) {
  process_register("PUBSUB", &PUBSUB_PROCESS);
}