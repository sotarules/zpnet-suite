// =============================================================
// FILE: process_pubsub.cpp
// =============================================================
//
// PUBSUB Process — Teensy-side pub/sub control surface
//
// Responsibilities:
//   • Ingress for TRAFFIC_PUBLISH_SUBSCRIBE (0xD2)
//   • Command-oriented experimentation surface
//   • Delegation to publish() / publish_local()
//
// Non-responsibilities:
//   • Routing
//   • Fan-out to Pi processes
//   • Transport policy
//
// =============================================================

#include "process_pubsub.h"

#include "publish.h"
#include "process.h"
#include "payload.h"
#include "debug.h"

#include <string.h>

// ================================================================
// Ingress: publications arriving from Pi
// ================================================================

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

  // Local delivery only — NEVER forward back to Pi
  publish_local(topic, payload);
}

// ================================================================
// Commands
// ================================================================

// ------------------------------------------------------------
// PUBLISH — invoke publish() directly
//
// Args:
//   {
//     "topic": "...",
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

  publish(topic, payload);
  return ok_payload();
}

// ------------------------------------------------------------
// SUBSCRIBE — command-oriented subscription control
//
// Semantics (initial):
//   • Experimental / diagnostic
//   • Acknowledges intent
//   • Does NOT mutate static subscription tables yet
//
// Future:
//   • Persist override to NVM
//   • Rebuild subscription view
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

  // Acknowledge only (no mutation yet)
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
//   • Diagnostic only
// ------------------------------------------------------------
static Payload cmd_report(const Payload&) {

  Payload resp;
  PayloadArray processes;

  for (size_t i = 0; i < process_get_count(); i++) {

    const process_vtable_t* v = process_get_vtable(i);
    if (!v) continue;

    Payload proc;
    proc.add("name", v->name);

    PayloadArray subs;

    if (v->subscriptions) {
      for (size_t s = 0; s < v->subscription_count; s++) {
        Payload entry;
        entry.add("topic", v->subscriptions[s].topic);
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
  { "REPORT",    cmd_report    }
};

static const process_vtable_t PUBSUB_PROCESS = {
  .name          = "PUBSUB",
  .query         = nullptr,
  .commands      = PUBSUB_COMMANDS,
  .command_count = sizeof(PUBSUB_COMMANDS) / sizeof(PUBSUB_COMMANDS[0]),
  .subscriptions = nullptr,
  .subscription_count = 0,
  .on_message    = nullptr
};

void process_pubsub_register(void) {
  process_register("PUBSUB", &PUBSUB_PROCESS);
}