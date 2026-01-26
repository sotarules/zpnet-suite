// ============================================================================
// FILE: events.h
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

// Enqueue a durable event (no serial output).
// "body" must be a valid JSON fragment WITHOUT outer braces.
void enqueueEvent(const char* type, const String& body);

