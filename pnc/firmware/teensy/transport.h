#pragma once

#include <stdint.h>
#include <stddef.h>

#include "payload.h"

// -------------------------------------------------------------
// Transport limits (authoritative)
// -------------------------------------------------------------
//
// Maximum size of a fully reassembled semantic message
// (after delimitation, before fragmentation).
//
// This bounds:
//   • RX reassembly buffer
//   • TX staging buffer
//
// MUST be large enough to hold the largest expected Payload.
// MUST be bounded to preserve memory integrity.
//

static constexpr size_t TRANSPORT_MAX_MESSAGE = 10 * 1024;

// -------------------------------------------------------------
// Traffic types (authoritative)
// -------------------------------------------------------------
//
// Traffic types are DATA, not API shape.
//

static constexpr uint8_t TRAFFIC_DEBUG             = 0xD0;
static constexpr uint8_t TRAFFIC_REQUEST_RESPONSE  = 0xD1;
static constexpr uint8_t TRAFFIC_PUBLISH_SUBSCRIBE = 0xD2;

// -------------------------------------------------------------
// Application receive callback
// -------------------------------------------------------------
//
// Invoked exactly once per fully reassembled and
// semantically decoded message.
//

using transport_receive_cb_t =
  void (*)(const Payload&);

// -------------------------------------------------------------
// Receive registration
// -------------------------------------------------------------
//
// Registers a callback for a specific traffic type.
// Transport performs all decoding and routing.
//

void transport_register_receive_callback(
  uint8_t traffic,
  transport_receive_cb_t cb
);

// -------------------------------------------------------------
// Send API (semantic entry point)
// -------------------------------------------------------------
//
// Sends ONE complete semantic message.
// The caller provides meaning as a Payload.
// Transport performs delimitation, fragmentation,
// and physical I/O.
//

void transport_send(
  uint8_t traffic,
  const Payload& payload
);

// -------------------------------------------------------------
// Lifecycle
// -------------------------------------------------------------
//
// Initializes transport and arms RX polling.
// Safe to call once during setup().
//

void transport_init(void);
