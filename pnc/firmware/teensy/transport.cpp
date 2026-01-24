#include "transport.h"
#include "config.h"
#include "debug.h"
#include "timepop.h"
#include "command.h"
#include "event_bus.h"

#include <Arduino.h>
#include <usb_rawhid.h>
#include <string.h>
#include <stdio.h>

// -------------------------------------------------------------
// Configuration
// -------------------------------------------------------------
//
// RawHID policy:
//   • Fixed 64-byte payloads
//   • Message spans one or more frames
//   • Final frame contains at least one 0x00 padding byte
//   • Padding terminates the message
//
// Protocol policy (unchanged):
//   • Transport messages are framed as <STX=N> ... <ETX>
//   • JSON payload extracted and passed to command_exec()
//

static constexpr size_t   HID_PACKET_SIZE   = 64;
static constexpr uint32_t HID_RX_TIMEOUT_MS = 0;

// -------------------------------------------------------------
// Protocol framing constants (unchanged)
// -------------------------------------------------------------

static constexpr uint8_t ASCII_LT = (uint8_t)'<';
static const char STX_PREFIX[] = "<STX=";
static const char ETX_SEQ[]    = "<ETX>";
static constexpr size_t ETX_LEN = 5;

// -------------------------------------------------------------
// RawHID RX state (message assembly only)
// -------------------------------------------------------------

static uint8_t raw_msg_buf[TRANSPORT_MAX_MESSAGE];
static size_t  raw_msg_len = 0;

// -------------------------------------------------------------
// Protocol parsing (unchanged semantics, refactored entry)
// -------------------------------------------------------------

static void process_complete_message(const uint8_t* msg, size_t len) {

  // We expect transport messages to begin with '<'
  if (len < 11 || msg[0] != ASCII_LT) {
    return;
  }

  // Must start with "<STX="
  if (memcmp(msg, STX_PREFIX, 5) != 0) {
    return;
  }

  // Parse decimal length
  size_t i = 5;
  int payload_len = 0;

  while (i < len && msg[i] >= '0' && msg[i] <= '9') {
    payload_len = payload_len * 10 + (msg[i] - '0');
    i++;
  }

  if (i >= len || msg[i] != '>') {
    return;
  }

  if (payload_len <= 0 || (size_t)payload_len > TRANSPORT_MAX_PAYLOAD) {
    return;
  }

  size_t json_start = i + 1;
  size_t json_end   = json_start + (size_t)payload_len;
  size_t etx_pos    = json_end;

  if (json_end + ETX_LEN > len) {
    return;
  }

  // Verify "<ETX>"
  if (memcmp(msg + etx_pos, ETX_SEQ, ETX_LEN) != 0) {
    return;
  }

  // Extract JSON into null-terminated buffer
  static char json[TRANSPORT_MAX_PAYLOAD + 1];
  memcpy(json, msg + json_start, (size_t)payload_len);
  json[payload_len] = '\0';

  // Dispatch (unchanged behavior)
  command_exec(json);
}

// -------------------------------------------------------------
// RX tick (TimePop-owned)
// -------------------------------------------------------------
//
// Responsibilities:
//   • Read one RawHID frame
//   • Append meaningful bytes to message buffer
//   • Detect padding → message complete
//   • Parse exactly one protocol frame
//

static void transport_rx_tick(timepop_ctx_t*, void*) {

  uint8_t hid[HID_PACKET_SIZE];
  int n = RawHID.recv(hid, HID_RX_TIMEOUT_MS);
  if (n <= 0) return;

  // Detect right-side zero padding
  size_t end = HID_PACKET_SIZE;
  while (end > 0 && hid[end - 1] == 0x00) {
    end--;
  }
  bool has_padding = (end < HID_PACKET_SIZE);

  // Append meaningful bytes
  if (end > 0) {
    if (raw_msg_len + end > sizeof(raw_msg_buf)) {
      // Hard reset on overflow (same net effect as previous resync)
      raw_msg_len = 0;
      return;
    }

    memcpy(raw_msg_buf + raw_msg_len, hid, end);
    raw_msg_len += end;
  }

  // If not final frame, wait for more
  if (!has_padding) {
    return;
  }

  // -----------------------------------------------------------
  // Complete RawHID message assembled
  // -----------------------------------------------------------

  process_complete_message(raw_msg_buf, raw_msg_len);

  // Reset for next message
  raw_msg_len = 0;
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

// -------------------------------------------------------------
// TX (unchanged behavior, clarified intent)
// -------------------------------------------------------------
//
// Still sends <STX=N>, payload, <ETX> as before.
// RawHID framing is responsible only for chunking + padding.
//

static inline void hid_send_message_bytes(const uint8_t* data, size_t len) {

  uint8_t pkt[HID_PACKET_SIZE];
  size_t offset = 0;

  while (offset < len) {

    size_t chunk = len - offset;
    if (chunk > HID_PACKET_SIZE) {
      chunk = HID_PACKET_SIZE;
    }

    memset(pkt, 0, sizeof(pkt));
    memcpy(pkt, data + offset, chunk);

    RawHID.send(pkt, 50);
    offset += chunk;
  }
}

void transport_send_frame(const char* payload, size_t length) {

  if (!payload || length == 0) return;
  if (length > TRANSPORT_MAX_PAYLOAD) return;

  // ------------------------------------------------------------
  // Construct full framed message contiguously:
  //   <STX=N> + payload + <ETX>
  // ------------------------------------------------------------

  uint8_t frame[TRANSPORT_MAX_MESSAGE];
  size_t  frame_len = 0;

  // --- Header ---
  int header_len = snprintf(
      (char*)frame,
      sizeof(frame),
      "<STX=%u>",
      (unsigned)length
  );

  if (header_len <= 0) return;
  frame_len += (size_t)header_len;

  // --- Payload ---
  memcpy(frame + frame_len, payload, length);
  frame_len += length;

  // --- ETX ---
  static constexpr char ETX[] = "<ETX>";
  memcpy(frame + frame_len, ETX, sizeof(ETX) - 1);
  frame_len += sizeof(ETX) - 1;

  // ------------------------------------------------------------
  // Send exactly ONE RawHID message
  // ------------------------------------------------------------
  hid_send_message_bytes(frame, frame_len);
}


void transport_send_frame(const char* payload) {
  if (!payload) return;
  transport_send_frame(payload, strlen(payload));
}
