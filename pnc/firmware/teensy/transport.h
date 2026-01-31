#pragma once

#include <stdint.h>
#include <stddef.h>

#include "payload.h"

// =============================================================
// Compile-time transport selection (authoritative)
// =============================================================
//
// Transport backend selection is determined by ONE of:
//
//   1. Explicit ZPNET override (preferred for non-Arduino builds):
//        - ZPNET_TRANSPORT_HID
//        - ZPNET_TRANSPORT_SERIAL
//
//   2. Teensy USB build configuration (authoritative on Teensy):
//        - USB_SERIAL
//        - USB_RAWHID
//        - USB_SERIAL_HID
//        - USB_RAWHID_SERIAL
//
// Exactly ONE backend must be selected.
//

#if defined(ZPNET_TRANSPORT_HID)
  #define ZPNET_TRANSPORT_SELECTED_HID

#elif defined(ZPNET_TRANSPORT_SERIAL)
  #define ZPNET_TRANSPORT_SELECTED_SERIAL

#elif defined(USB_RAWHID) || defined(USB_RAWHID_SERIAL)
  #define ZPNET_TRANSPORT_SELECTED_HID

#elif defined(USB_SERIAL) || defined(USB_SERIAL_HID)
  #define ZPNET_TRANSPORT_SELECTED_SERIAL

#else
  #error "No transport backend defined (ZPNET_TRANSPORT_* or USB_*)"
#endif

// =============================================================
// Transport limits (authoritative)
// =============================================================

static constexpr size_t TRANSPORT_MAX_MESSAGE = 10 * 1024;

// =============================================================
// Physical transport invariants
// =============================================================

static constexpr size_t TRANSPORT_BLOCK_SIZE = 64;

// =============================================================
// Traffic types (authoritative)
// =============================================================

static constexpr uint8_t TRAFFIC_DEBUG             = 0xD0;
static constexpr uint8_t TRAFFIC_REQUEST_RESPONSE  = 0xD1;
static constexpr uint8_t TRAFFIC_PUBLISH_SUBSCRIBE = 0xD2;

// =============================================================
// Application receive callback
// =============================================================

using transport_receive_cb_t =
  void (*)(const Payload&);

// =============================================================
// Receive registration
// =============================================================

void transport_register_receive_callback(
  uint8_t traffic,
  transport_receive_cb_t cb
);

// =============================================================
// Send API (semantic entry point)
// =============================================================

void transport_send(
  uint8_t traffic,
  const Payload& payload
);

// =============================================================
// Block API (physical transport boundary)
// =============================================================

bool transport_send_block(
  const uint8_t block[TRANSPORT_BLOCK_SIZE]
);

bool transport_recv_block(
  uint8_t block[TRANSPORT_BLOCK_SIZE]
);

// =============================================================
// Lifecycle
// =============================================================

void transport_init(void);