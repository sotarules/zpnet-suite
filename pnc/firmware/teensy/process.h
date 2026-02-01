#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

#include "payload.h"

// --------------------------------------------------
// Command Handler Contract (AUTHORITATIVE)
// --------------------------------------------------

using process_command_fn =
  Payload (*)(const Payload& args);

// --------------------------------------------------
// Topic Message Handler Contract
// --------------------------------------------------

using process_message_fn =
  void (*)(const char* topic, const Payload& payload);

// --------------------------------------------------
// Process Command Declaration
// --------------------------------------------------

typedef struct {
  const char*        name;
  process_command_fn handler;
} process_command_entry_t;

// --------------------------------------------------
// Process Subscription Declaration
// --------------------------------------------------

typedef struct {
  const char* topic;
} process_subscription_entry_t;

// --------------------------------------------------
// Process VTable (Declarative)
// --------------------------------------------------

typedef struct {
  const char* name;

  String (*query)(void);

  const process_command_entry_t* commands;
  size_t                         command_count;

  const process_subscription_entry_t* subscriptions;
  size_t                              subscription_count;

  process_message_fn                  on_message;

} process_vtable_t;

// --------------------------------------------------
// Public API
// --------------------------------------------------

void process_init(void);

bool process_register(
  const char* process_id,
  const process_vtable_t* vtable
);

// --------------------------------------------------
// Registry Introspection (READ-ONLY)
// --------------------------------------------------

size_t      process_get_count(void);
const char* process_get_name(size_t idx);

// *** NEW (read-only, internal use) ***
const process_vtable_t* process_get_vtable(size_t idx);

// --------------------------------------------------
// Transport-facing command processor
// --------------------------------------------------

void process_command(const Payload& request);

// --------------------------------------------------
// Canonical helpers
// --------------------------------------------------

Payload ok_payload();