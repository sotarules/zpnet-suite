#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

#include "payload.h"

// ============================================================================
// ZPNet Process Framework — Declarative Contract
// ============================================================================
//
// A process is a named semantic participant that may:
//
//   • Accept COMMANDS (request/response)
//   • Receive SUBSCRIPTIONS (pub/sub delivery)
//
// Commands and subscriptions are symmetric:
//   • Each name maps to exactly one handler
//   • No shared multiplexers
//   • No implicit routing
//
// The framework owns sizing, iteration, and dispatch.
// Process authors declare intent only.
//
// ============================================================================


// --------------------------------------------------
// Command Handler Contract
// --------------------------------------------------
//
// Invoked synchronously in scheduled (non-ISR) context.
// Returns a semantic Payload (may be empty).
//
using process_command_fn =
  Payload (*)(const Payload& args);


// --------------------------------------------------
// Subscription Handler Contract
// --------------------------------------------------
//
// Invoked synchronously on topic delivery.
// No return value. Side effects only.
//
using process_subscription_fn =
  void (*)(const Payload& payload);


// --------------------------------------------------
// Command Declaration
// --------------------------------------------------

typedef struct {
  const char*        name;
  process_command_fn handler;
} process_command_entry_t;


// --------------------------------------------------
// Subscription Declaration
// --------------------------------------------------

typedef struct {
  const char*              topic;
  process_subscription_fn  handler;
} process_subscription_entry_t;


// --------------------------------------------------
// Process VTable (Declarative)
// --------------------------------------------------
//
// NOTE:
//   • No counts
//   • No query surface
//   • No shared message handler
//
// Array termination is determined by the framework.
//
typedef struct {
  const char* process_id;

  const process_command_entry_t*      commands;
  const process_subscription_entry_t* subscriptions;

} process_vtable_t;


// ============================================================================
// Framework API
// ============================================================================


// --------------------------------------------------
// Lifecycle
// --------------------------------------------------

// Initialize the process registry.
// Must be called once at system startup.
void process_init(void);


// --------------------------------------------------
// Registration
// --------------------------------------------------
//
// Registers a process with the system.
// process_id MUST match vtable->process_id.
//
bool process_register(
  const char*             process_id,
  const process_vtable_t* vtable
);


// --------------------------------------------------
// Registry Introspection (READ-ONLY)
// --------------------------------------------------

size_t      process_get_count(void);
const char* process_get_name(size_t idx);

// Internal framework use only
const process_vtable_t* process_get_vtable(size_t idx);

const process_vtable_t* process_get_vtable_by_name(const char* name);

// --------------------------------------------------
// Transport-facing entry points
// --------------------------------------------------

// REQUEST / RESPONSE ingress
void process_command(const Payload& request);

// PUB / SUB ingress (called by transport / pubsub)
void process_publish_dispatch(
  const char* topic,
  const Payload& payload
);


// --------------------------------------------------
// Canonical helpers
// --------------------------------------------------

// Standard success payload
Payload ok_payload();
