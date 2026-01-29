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
static const char ETX_SEQ[]    = "<ETX>";
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
// PRIVATE: send-side delimitation
// -------------------------------------------------------------

static void handle_send_delimiters(
  uint8_t traffic,
  const Payload& payload,
  uint8_t* out_buf,
  size_t& out_len
) {
  String json = payload.to_json();
  const size_t json_len = json.length();

  if (traffic == TRAFFIC_REQUEST_RESPONSE) {

    char header[32];
    int header_len = snprintf(
      header,
      sizeof(header),
      "<STX=%u>",
      (unsigned)json_len
    );

    memcpy(out_buf, header, (size_t)header_len);
    memcpy(out_buf + header_len, json.c_str(), json_len);
    memcpy(out_buf + header_len + json_len, ETX_SEQ, ETX_LEN);

    out_len = (size_t)header_len + json_len + ETX_LEN;
    return;
  }

  // All other traffic: raw JSON
  memcpy(out_buf, json.c_str(), json_len);
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
  fragment_and_send(traffic, send_buf, send_len);
}

// -------------------------------------------------------------
// PRIVATE: receive-side delimitation
// -------------------------------------------------------------

static void handle_receive_delimiters(
  uint8_t traffic,
  const uint8_t* buf,
  size_t len,
  Payload& out
) {
  if (traffic == TRAFFIC_REQUEST_RESPONSE) {

    // <STX=n>
    size_t i = 5;
    int json_len = 0;

    while (buf[i] >= '0' && buf[i] <= '9') {
      json_len = json_len * 10 + (buf[i] - '0');
      i++;
    }

    size_t json_start = i + 1;
    out.parseJSON(buf + json_start, (size_t)json_len);
    return;
  }

  // All other traffic
  out.parseJSON(buf, len);
}

// -------------------------------------------------------------
// RX semantic dispatch
// -------------------------------------------------------------

static void handle_complete_message(
  const uint8_t* msg,
  size_t len
) {
  uint8_t traffic = msg[0];
  const uint8_t* payload = msg + 1;
  size_t payload_len = len - 1;

  if (!recv_cb[traffic]) {
    debug_log("transport", "RX with no registered callback");
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

  size_t end = HID_PACKET_SIZE;
  while (end > 0 && pkt[end - 1] == 0x00) {
    end--;
  }

  bool has_padding = (end < HID_PACKET_SIZE);

  memcpy(rx_buf + rx_len, pkt, end);
  rx_len += end;

  if (!has_padding) return;

  handle_complete_message(rx_buf, rx_len);
  rx_len = 0;
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
