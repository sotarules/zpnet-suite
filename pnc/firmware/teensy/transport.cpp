// transport.cpp
//
// ZPNet Transport Layer (Teensy side)
//
// This file owns the entire byte ↔ meaning boundary.
// All invariants are assumed to be correct by construction.
//
#include "transport.h"

#include "config.h"
#include "debug.h"
#include "timepop.h"
#include "events.h"

#include <Arduino.h>
#include <usb_rawhid.h>
#include <string.h>
#include <stdio.h>

// -------------------------------------------------------------
// Constants
// -------------------------------------------------------------

static constexpr size_t   HID_PACKET_SIZE   = 64;
static constexpr uint32_t HID_RX_TIMEOUT_MS = 0;

// Legacy framing (REQUEST_RESPONSE only)
static const char ETX_SEQ[]     = "<ETX>";
static constexpr size_t ETX_LEN = 5;

// -------------------------------------------------------------
// Receive callback table
// -------------------------------------------------------------

static void (*recv_cb[256])(const Payload&) = { nullptr };

// -------------------------------------------------------------
// RX reassembly state
// -------------------------------------------------------------

static uint8_t rx_buf[TRANSPORT_MAX_MESSAGE];
static size_t  rx_len = 0;

// -------------------------------------------------------------
// Registration
// -------------------------------------------------------------

void transport_register_receive_callback(
  uint8_t traffic,
  void (*cb)(const Payload&)
) {
  recv_cb[traffic] = cb;
}

// -------------------------------------------------------------
// PRIVATE: fragmentation
// -------------------------------------------------------------

static void fragment_and_send(
  uint8_t traffic,
  const uint8_t* buf,
  size_t len
) {
  uint8_t pkt[HID_PACKET_SIZE];
  size_t offset = 0;

  // First packet: traffic byte + payload
  pkt[0] = traffic;

  size_t first_chunk =
    (len + 1 > HID_PACKET_SIZE)
      ? HID_PACKET_SIZE - 1
      : len;

  memcpy(pkt + 1, buf, first_chunk);
  memset(pkt + 1 + first_chunk, 0,
         HID_PACKET_SIZE - (first_chunk + 1));

  RawHID.send(pkt, 50);
  offset += first_chunk;

  // Subsequent packets
  while (offset < len) {
    size_t remaining = len - offset;
    size_t chunk =
      (remaining > HID_PACKET_SIZE)
        ? HID_PACKET_SIZE
        : remaining;

    memcpy(pkt, buf + offset, chunk);
    memset(pkt + chunk, 0, HID_PACKET_SIZE - chunk);

    RawHID.send(pkt, 50);
    offset += chunk;
  }
}

// -------------------------------------------------------------
// PRIVATE: send-side delimitation (JsonView-based)
// -------------------------------------------------------------
//
// REQUEST_RESPONSE:
//   <STX=n>{JSON}<ETX>
//
// All other traffic:
//   {JSON}
//
// BOUNDS:
//   We MUST NOT write beyond out_buf capacity (TRANSPORT_MAX_MESSAGE).
//   On overflow, emit a small explicit error JSON.
// -------------------------------------------------------------

static void handle_send_delimiters(
  uint8_t traffic,
  const Payload& payload,
  uint8_t* out_buf,
  size_t& out_len
) {
  out_len = 0;

  // Primary path: transient view (no heap)
  JsonView v = payload.json_view();
  const size_t json_len = v.len;

  if (traffic == TRAFFIC_REQUEST_RESPONSE) {

    char header[32];
    int header_len = snprintf(
      header,
      sizeof(header),
      "<STX=%u>",
      (unsigned)json_len
    );

    if (header_len <= 0) {
      emit_system_error(
        "transport",
        "transport.cpp",
        "handle_send_delimiters",
        "header_len <= 0 - message not sent"
      );
      return;
    }

    const size_t total_len =
      (size_t)header_len + json_len + ETX_LEN;

    if (total_len > TRANSPORT_MAX_MESSAGE) {
      emit_system_error(
        "transport",
        "transport.cpp",
        "handle_send_delimiters",
        "total_len > TRANSPORT_MAX_MESSAGE - message not sent"
      );
      return;
    }

    memcpy(out_buf, header, (size_t)header_len);
    memcpy(out_buf + (size_t)header_len, v.data, json_len);
    memcpy(out_buf + (size_t)header_len + json_len, ETX_SEQ, ETX_LEN);

    out_len = total_len;
    return;
  }

  // All other traffic: raw JSON
  if (json_len > TRANSPORT_MAX_MESSAGE) {
    emit_system_error(
      "transport",
      "transport.cpp",
      "handle_send_delimiters",
      "json_len > TRANSPORT_MAX_MESSAGE - message not sent"
    );
    return;
  }

  memcpy(out_buf, v.data, json_len);
  out_len = json_len;
}

// -------------------------------------------------------------
// Public send API
// -------------------------------------------------------------

void transport_send(
  uint8_t traffic,
  const Payload& payload
) {
  static uint8_t send_buf[TRANSPORT_MAX_MESSAGE];
  size_t send_len = 0;

  handle_send_delimiters(traffic, payload, send_buf, send_len);

  if (send_len == 0) {
    emit_system_error(
      "transport",
      "transport.cpp",
      "transport_send",
      "send_len == 0 - transport_send aborted"
    );
    return;    // Nothing to send (should be rare; treated as best-effort drop)
  }

  fragment_and_send(traffic, send_buf, send_len);
}

// -------------------------------------------------------------
// PRIVATE: receive-side delimitation
// -------------------------------------------------------------
//
// Expected formats:
//
// REQUEST_RESPONSE:
//   <STX=n>{JSON}<ETX>
//
// All other traffic:
//   {JSON}
//
// This function MUST:
//   • Never read past buf[len]
//   • Never assume framing is well-formed
//   • Fail silently (caller owns observability)
// -------------------------------------------------------------

static void handle_receive_delimiters(
  uint8_t traffic,
  const uint8_t* buf,
  size_t len,
  Payload& out
) {
  out.clear();

  if (traffic != TRAFFIC_REQUEST_RESPONSE) {
    out.parseJSON(buf, len);
    return;
  }

  // ---- MINIMUM SAFE GUARD ----
  if (len < 6) {
    return;
  }

  // ---- VERIFY PREFIX ----
  if (buf[0] != '<' ||
      buf[1] != 'S' ||
      buf[2] != 'T' ||
      buf[3] != 'X' ||
      buf[4] != '=') {
    return;
  }

  size_t i = 5;
  size_t json_len = 0;

  while (i < len && buf[i] >= '0' && buf[i] <= '9') {
    json_len = (json_len * 10) + (buf[i] - '0');
    i++;
  }

  if (i >= len || buf[i] != '>') {
    return;
  }

  i++;  // skip '>'

  if (i + json_len > len) {
    return;
  }

  out.parseJSON(buf + i, json_len);
}

// -------------------------------------------------------------
// RX semantic dispatch
// -------------------------------------------------------------

static void handle_complete_message(
  const uint8_t* msg,
  size_t len
) {
  if (!msg || len < 1) {
    emit_system_error(
      "transport",
      "transport.cpp",
      "handle_complete_message",
      "!msg || len < 1 - invalid message ignored"
    );
    return;
  }

  uint8_t traffic = msg[0];

  const uint8_t* payload = msg + 1;
  size_t payload_len = len - 1;

  if (!recv_cb[traffic]) {
    return;
  }

  Payload p;
  handle_receive_delimiters(traffic, payload, payload_len, p);
  recv_cb[traffic](p);
}

// -------------------------------------------------------------
// RX tick
// -------------------------------------------------------------

static void transport_rx_tick(
  timepop_ctx_t*,
  void*
) {
  uint8_t pkt[HID_PACKET_SIZE];

  int n = RawHID.recv(pkt, HID_RX_TIMEOUT_MS);
  if (n <= 0) return;

  // ---------------------------------------------------------
  // Bounded reassembly (never overflow rx_buf)
  // ---------------------------------------------------------
  if (rx_len + (size_t)n > TRANSPORT_MAX_MESSAGE) {
    emit_system_error(
      "transport",
      "transport.cpp",
      "transport_rx_tick",
      "rx_len + n > TRANSPORT_MAX_MESSAGE - incoming data ignored"
    );
    rx_len = 0;
    return;
  }

  memcpy(rx_buf + rx_len, pkt, (size_t)n);
  rx_len += (size_t)n;

  // ---------------------------------------------------------
  // Explicit framing termination ONLY
  // ---------------------------------------------------------
  //
  // We do NOT infer message completion from zero padding.
  // A message is complete iff an explicit ETX is present.
  //
  // This applies only to framed traffic (REQUEST_RESPONSE).
  //

  // Minimum length to contain "<ETX>"
  if (rx_len < ETX_LEN) {
    return;
  }

  // Scan tail for ETX sequence
  for (size_t i = 0; i + ETX_LEN <= rx_len; i++) {
    if (memcmp(rx_buf + i, ETX_SEQ, ETX_LEN) == 0) {

      // Complete framed message found
      handle_complete_message(rx_buf, rx_len);
      rx_len = 0;
      return;
    }
  }

  // No complete frame yet — wait for more data
}

// -------------------------------------------------------------
// Lifecycle
// -------------------------------------------------------------

void transport_init(void) {

  debug_log("transport", "*init*");

  timepop_arm(
    TIMEPOP_CLASS_RX_POLL,
    true,
    transport_rx_tick,
    nullptr,
    "transport-rx"
  );
}
