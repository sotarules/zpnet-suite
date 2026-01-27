// ============================================================================
// FILE: process_events.h
// ============================================================================
//
// ZPNet EVENTS Process
//
// Role:
//   • Maintains a bounded FIFO of durable system events
//   • Acts as the sole clearinghouse for immutable fact narration
//
// Model:
//   • EVENTS is an existential subsystem, not a lifecycle-managed process
//   • It does not "start" or "stop"
//   • It exists as soon as the firmware is executing
//
// Initialization:
//   • process_events_init() explicitly binds EVENTS to the TIMEPOP substrate
//   • This arms the EVENTBUS scheduling class required for event draining
//   • Initialization must occur after timepop_init()
//   • Initialization has no side effects beyond enabling observability
//
// Semantics:
//   • enqueueEvent() records immutable facts (event type + Payload)
//   • Events are serialized exactly once, at enqueue time
//   • EVENTS.GET returns all queued events and clears the queue
//   • No streaming, framing, polling, or aggregation occurs here
//
// Guarantees:
//   • Events are delivered in FIFO order
//   • Events are never reinterpreted or modified after enqueue
//   • Event overflow is a visible downstream condition, not hidden recovery
//
// Notes:
//   • EVENTS does not own time; it binds to it explicitly
//   • EVENTS does not depend on process lifecycle ordering
//   • EVENTS is deliberately boring and correct
//
// Author: The Mule + GPT
// ============================================================================

#pragma once

#include "payload.h"

// Explicit initialization: bind EVENTS to TIMEPOP scheduling
void process_events_init(void);

// Register EVENTS command surface
void process_events_register(void);

// Durable event enqueue (authoritative)
void enqueueEvent(const char* type, const Payload& payload);
