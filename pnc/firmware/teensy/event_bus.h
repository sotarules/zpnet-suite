// ============================================================================
// FILE: event_bus.h
// ============================================================================
#pragma once

#include <Arduino.h>
#include "config.h"

// --------------------------------------------------------------
// Event bus interface
// --------------------------------------------------------------
//
// The event bus is the sole durable exit path for truth.
// Events are enqueued opportunistically and emitted ONLY
// after an explicit drain request.
//
// New ZPNet kernel standard:
//   • event_bus owns its own timing via TimePop
//   • no polling in zpnet_loop()
//   • no work in ISR
//   • tick emits nothing unless drain requested
//
// --------------------------------------------------------------

// --------------------------------------------------------------
// Event item
// --------------------------------------------------------------
struct EventItem {
  char type[EVT_TYPE_MAX];
  char body[EVT_BODY_MAX];
};

// --------------------------------------------------------------
// TimePop-owned lifecycle (NEW)
// --------------------------------------------------------------

// Initialize the event bus tick (must be called from setup()).
void event_bus_init();

// Request a drain (idempotent). Actual drain occurs in scheduled context.
void event_bus_request_drain();

// --------------------------------------------------------------
// Public API (existing)
// --------------------------------------------------------------

// Enqueue a durable event (no serial output)
void enqueueEvent(const char* type, const String& body);

// Drain all queued events immediately to Serial
// Emits EVENTS_BEGIN / EVENTS_END framing
//
// Note: Under the new kernel standard, callers should prefer
// event_bus_request_drain() so that draining occurs in scheduled context.
// drainEventsNow() remains available for compatibility.
void drainEventsNow();

// Convenience helpers (queue-only)
void enqueueAckEvent(const char* cmd);
void enqueueErrEvent(const char* cmd, const char* msg);

// Diagnostics
uint32_t eventQueueDepth();
uint32_t eventDroppedCount();
