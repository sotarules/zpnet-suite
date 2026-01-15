#pragma once

#include <Arduino.h>
#include "config/config.h"

// --------------------------------------------------------------
// Event bus interface
// --------------------------------------------------------------
//
// The event bus is the sole durable exit path for truth.
// Events are enqueued opportunistically and emitted ONLY
// during an explicit drain command.
//
// No unsolicited serial output is permitted.
//

// --------------------------------------------------------------
// Event item
// --------------------------------------------------------------
struct EventItem {
  char type[EVT_TYPE_MAX];
  char body[EVT_BODY_MAX];
};

// --------------------------------------------------------------
// Public API
// --------------------------------------------------------------

// Enqueue a durable event (no serial output)
void enqueueEvent(const char* type, const String& body);

// Drain all queued events immediately to Serial
// Emits EVENTS_BEGIN / EVENTS_END framing
void drainEventsNow();

// Convenience helpers (queue-only)
void enqueueAckEvent(const char* cmd);
void enqueueErrEvent(const char* cmd, const char* msg);

// Diagnostics
uint32_t eventQueueDepth();
uint32_t eventDroppedCount();
