// ============================================================================
// FILE: events.h
// ============================================================================
//
// ZPNet Event Bus (Durable Truth Queue)
//
// Core contract:
//   • Events are durable facts.
//   • Events are enqueued opportunistically.
//   • Events carry structured meaning via Payload.
//   • Serialization occurs exactly once, at enqueue time.
//   • NOTHING is emitted unless an explicit drain is requested.
//   • Draining occurs ONLY in scheduled (non-ISR) context.
//
// TimePop integration (kernel standard):
//   • The event bus tick is armed via timepop_arm(TIMEPOP_CLASS_EVENTBUS, ...).
//   • The event bus does NOT self-schedule.
//   • The tick performs work only when g_drain_requested is set.
//
// ============================================================================

#pragma once

#include <Arduino.h>
#include "config.h"
#include "payload.h"

// --------------------------------------------------------------
// Enqueue a durable event
// --------------------------------------------------------------
//
// Contract:
//   • `type` is a stable event identifier
//   • `payload` is a complete semantic object
//   • `payload` may be empty (no fields)
//   • JSON serialization is owned by the event bus
//   • Callers MUST NOT construct JSON
//
void enqueueEvent(const char* type, const Payload& payload);
