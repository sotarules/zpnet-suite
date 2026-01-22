// ============================================================================
// FILE: event_bus.h
// ============================================================================
//
// ZPNet Event Bus (Durable Truth Queue)
//
// Core contract:
//   • Events are durable facts.
//   • Events are enqueued opportunistically.
//   • NOTHING is emitted unless an explicit drain is requested.
//   • Draining occurs ONLY in scheduled (non-ISR) context.
//
// TimePop integration (new kernel standard):
//   • The event bus tick is armed via timepop_arm(TIMEPOP_CLASS_EVENTBUS, ...).
//   • The event bus does NOT self-schedule.
//   • The tick performs work only when g_drain_requested is set.
//
// ============================================================================

#pragma once

#include <Arduino.h>
#include "config.h"

// --------------------------------------------------------------
// Event item (ring buffer entry)
// --------------------------------------------------------------
struct EventItem {
  char type[EVT_TYPE_MAX];
  char body[EVT_BODY_MAX];
};

// --------------------------------------------------------------
// TimePop-owned lifecycle
// --------------------------------------------------------------

// Initialize the event bus tick (must be called from setup()).
// Arms a recurring TimePop timer (TIMEPOP_CLASS_EVENTBUS).
void event_bus_init(void);

// Request a drain (idempotent).
// The actual drain is performed by the event bus TimePop tick.
void event_bus_request_drain(void);

// --------------------------------------------------------------
// Public API (durable truth)
// --------------------------------------------------------------

// Enqueue a durable event (no serial output).
// "body" must be a valid JSON fragment WITHOUT outer braces.
void enqueueEvent(const char* type, const String& body);

// Drain all queued events immediately.
// Emits protocol-control framing (EVENTS_BEGIN / EVENTS_END).
//
// NOTE:
//   This function is safe and remains available for compatibility.
//   Under the new kernel standard, prefer event_bus_request_drain()
//   so draining occurs in scheduled context.
void drainEventsNow(void);

// Convenience helpers (queue-only)
void enqueueAckEvent(const char* cmd);
void enqueueErrEvent(const char* cmd, const char* msg);

// Diagnostics
uint32_t eventQueueDepth(void);
uint32_t eventDroppedCount(void);
