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

static constexpr size_t   HID_PACKET_SIZE   = 64;
static constexpr uint32_t HID_RX_TIMEOUT_MS = 0;

// Report IDs
static constexpr uint8_t HID_REPORT_TRANSPORT = 0x00;

// -------------------------------------------------------------
// RX parser state machine
// -------------------------------------------------------------

enum RxState {
  RX_IDLE,
  RX_STX,
  RX_LEN,
  RX_PAYLOAD,
  RX_ETX
};

static RxState rx_state = RX_IDLE;

static size_t expected_len = 0;
static size_t received_len = 0;
static size_t etx_pos      = 0;

static const char ETX_SEQ[] = "<ETX>";

// -------------------------------------------------------------
// Helpers
// -------------------------------------------------------------

static inline void rx_reset() {
  rx_state     = RX_IDLE;
  expected_len = 0;
  received_len = 0;
  etx_pos      = 0;
}

static inline bool is_digit(char c) {
  return (c >= '0' && c <= '9');
}

static inline bool is_printable(char c) {
  return (c >= 32 && c <= 126);
}

// -------------------------------------------------------------
// RX scheduling (TimePop-owned)
// -------------------------------------------------------------

static void transport_rx_tick(timepop_ctx_t*, void*) {

  // Persistent binary accumulator
  static uint8_t buf[512];
  static size_t  len = 0;

  uint8_t hid[HID_PACKET_SIZE];
  int n = RawHID.recv(hid, HID_RX_TIMEOUT_MS);
  if (n <= 0) return;

  // Append HID payload (binary-safe)
  for (int i = 0; i < n && len < sizeof(buf); i++) {
    buf[len++] = hid[i];
  }

  // We need at least "<STX=1><ETX>" to proceed
  if (len < 11) return;

  // Scan buffer for frame start
  for (size_t i = 0; i + 5 < len; i++) {

    // Match "<STX="
    if (buf[i] != '<' ||
        buf[i+1] != 'S' ||
        buf[i+2] != 'T' ||
        buf[i+3] != 'X' ||
        buf[i+4] != '=') {
      continue;
    }

    // Parse decimal length
    size_t j = i + 5;
    int msg_len = 0;

    while (j < len && buf[j] >= '0' && buf[j] <= '9') {
      msg_len = msg_len * 10 + (buf[j] - '0');
      j++;
    }

    // Expect '>'
    if (j >= len || buf[j] != '>') {
      return; // header incomplete
    }

    size_t json_start = j + 1;

    // Need JSON + "<ETX>"
    if (json_start + msg_len + 5 > len) {
      return; // frame incomplete
    }

    // Verify "<ETX>"
    size_t etx = json_start + msg_len;
    if (buf[etx]     != '<' ||
        buf[etx + 1] != 'E' ||
        buf[etx + 2] != 'T' ||
        buf[etx + 3] != 'X' ||
        buf[etx + 4] != '>') {
      // Malformed frame: drop everything up to '<'
      memmove(buf, buf + i + 1, len - (i + 1));
      len -= (i + 1);
      return;
    }

    // Null-terminate JSON in place
    static char json[TRANSPORT_MAX_PAYLOAD + 1];
    if ((size_t)msg_len >= sizeof(json)) {
      len = 0;
      return;
    }

    memcpy(json, buf + json_start, msg_len);
    json[msg_len] = '\0';

    // Dispatch
    command_exec(json);

    // Consume frame from buffer
    size_t consumed = etx + 5;
    memmove(buf, buf + consumed, len - consumed);
    len -= consumed;

    return; // one frame per TimePop (intentional)
  }
}


// -------------------------------------------------------------
// Lifecycle
// -------------------------------------------------------------

void transport_init(void) {

  rx_reset();

  debug_log("transport", "*init*");

  // Arm RX ingestion loop (TimePop owns cadence)
  timepop_arm(
    TIMEPOP_CLASS_RX_POLL,
    true,
    transport_rx_tick,
    nullptr,
    "transport-rx"
  );
}

// -------------------------------------------------------------
// TX (framed egress)
// -------------------------------------------------------------

void transport_send_frame(const char* payload, size_t length) {

  if (!payload || length == 0) return;
  if (length > TRANSPORT_MAX_PAYLOAD) return;

  char header[16];
  int header_len =
      snprintf(header, sizeof(header), "<STX=%u>", (unsigned)length);

  static const char etx[] = "<ETX>";

  auto send_bytes = [&](const uint8_t* data, size_t len) {
    if (!data || !len) return;

    uint8_t pkt[HID_PACKET_SIZE];
    size_t offset = 0;

    while (offset < len) {
      size_t chunk = len - offset;
      if (chunk > HID_PACKET_SIZE - 1) {
        chunk = HID_PACKET_SIZE - 1;
      }

      memset(pkt, 0, sizeof(pkt));
      pkt[0] = HID_REPORT_TRANSPORT;   // report ID
      memcpy(pkt + 1, data + offset, chunk);

      RawHID.send(pkt, 50);
      offset += chunk;
    }
  };

  send_bytes((const uint8_t*)header, header_len);
  send_bytes((const uint8_t*)payload, length);
  send_bytes((const uint8_t*)etx, sizeof(etx) - 1);
}

void transport_send_frame(const char* payload) {
  if (!payload) return;
  transport_send_frame(payload, strlen(payload));
}
