// transport.cpp — RECAPPED: HID vs SERIAL bifurcated at IO boundary
// -------------------------------------------------------------------
// HID: Uses 64-byte blocks via RawHID
// SERIAL: Uses byte stream with ETX-delimited framing
// Shared: semantic framing, JSON view, RX dispatch, registration

#include "transport.h"
#include "config.h"
#include "debug.h"
#include "timepop.h"
#include "events.h"

#include <Arduino.h>
#include <string.h>
#include <stdio.h>

// =============================================================
// Constants
// =============================================================

static constexpr uint32_t RX_TIMEOUT_MS = 0;
static constexpr char     ETX_SEQ[]     = "<ETX>";
static constexpr size_t   ETX_LEN       = 5;

static constexpr size_t STREAM_RX_BUF_MAX = 2048;  // for serial reassembly

// =============================================================
// State
// =============================================================

static void (*recv_cb[256])(const Payload&) = { nullptr };
static uint8_t rx_buf[STREAM_RX_BUF_MAX];
static size_t  rx_len = 0;

// =============================================================
// Transport Fingerprint
// =============================================================

#if defined(ZPNET_TRANSPORT_SELECTED_HID)
static const char* BUILD_FINGERPINT = "__FP__XXZPNET_HIDXX__";
#elif defined(ZPNET_TRANSPORT_SELECTED_SERIAL)
static const char* BUILD_FINGERPINT = "__FP__XXZPNET_SERIALXX__";
#endif

// =============================================================
// Backend: SEND
// =============================================================

#if defined(ZPNET_TRANSPORT_SELECTED_HID)

bool transport_send_block(const uint8_t block[TRANSPORT_BLOCK_SIZE]) {
  return RawHID.send(block, 50) > 0;
}

#elif defined(ZPNET_TRANSPORT_SELECTED_SERIAL)

bool transport_send_block(const uint8_t block[TRANSPORT_BLOCK_SIZE]) {
  size_t n = Serial.write(block, TRANSPORT_BLOCK_SIZE);
  Serial.flush();
  return n == TRANSPORT_BLOCK_SIZE;
}

#endif

// =============================================================
// Backend: RECEIVE
// =============================================================

#if defined(ZPNET_TRANSPORT_SELECTED_HID)

bool transport_recv_block(uint8_t block[TRANSPORT_BLOCK_SIZE]) {
  return RawHID.recv(block, RX_TIMEOUT_MS) > 0;
}

#elif defined(ZPNET_TRANSPORT_SELECTED_SERIAL)

bool transport_recv_block(uint8_t block[TRANSPORT_BLOCK_SIZE]) {

  static uint8_t buf[TRANSPORT_BLOCK_SIZE];
  static size_t count = 0;

  while (Serial.available() && count < TRANSPORT_BLOCK_SIZE) {
    uint8_t b = Serial.read();
    buf[count++] = b;

    char hex_buf[8];
    snprintf(hex_buf, sizeof(hex_buf), "0x%02X", b);
  }

  if (count < TRANSPORT_BLOCK_SIZE) return false;

  memcpy(block, buf, TRANSPORT_BLOCK_SIZE);
  count = 0;
  return true;
}

#endif

// =============================================================
// Registration
// =============================================================

void transport_register_receive_callback(
  uint8_t traffic,
  void (*cb)(const Payload&)
) {
  recv_cb[traffic] = cb;
}

// =============================================================
// SEND FRAGMENTATION (shared)
// =============================================================

static void fragment_and_send(
  uint8_t traffic,
  const uint8_t* buf,
  size_t len
) {
  uint8_t pkt[TRANSPORT_BLOCK_SIZE];
  size_t offset = 0;

  pkt[0] = traffic;

  size_t first_chunk = (len + 1 > TRANSPORT_BLOCK_SIZE)
    ? TRANSPORT_BLOCK_SIZE - 1
    : len;

  memcpy(pkt + 1, buf, first_chunk);
  memset(pkt + 1 + first_chunk, 0, TRANSPORT_BLOCK_SIZE - (first_chunk + 1));
  transport_send_block(pkt);
  offset += first_chunk;

  while (offset < len) {
    size_t chunk = min(len - offset, TRANSPORT_BLOCK_SIZE);
    memcpy(pkt, buf + offset, chunk);
    memset(pkt + chunk, 0, TRANSPORT_BLOCK_SIZE - chunk);
    transport_send_block(pkt);
    offset += chunk;
  }
}

// =============================================================
// SEND ENTRY POINT
// =============================================================

void transport_send(
  uint8_t traffic,
  const Payload& payload
) {
  static uint8_t send_buf[TRANSPORT_MAX_MESSAGE];
  size_t send_len = 0;

  JsonView v = payload.json_view();
  const size_t json_len = v.len;

  // All semantic traffic is framed
  char header[32];
  int header_len = snprintf(header, sizeof(header), "<STX=%u>", (unsigned)json_len);
  if (header_len <= 0) return;

  const size_t total_len = header_len + json_len + ETX_LEN;
  if (total_len > TRANSPORT_MAX_MESSAGE) return;

  memcpy(send_buf, header, header_len);
  memcpy(send_buf + header_len, v.data, json_len);
  memcpy(send_buf + header_len + json_len, ETX_SEQ, ETX_LEN);
  send_len = total_len;

  fragment_and_send(traffic, send_buf, send_len);
}

// =============================================================
// RECEIVE: Delimiter Scan + Dispatch (shared)
// =============================================================

static void handle_receive_delimiters(
  uint8_t traffic,
  const uint8_t* buf,
  size_t len,
  Payload& out
) {
  out.clear();

  if (len < 6) return;
  if (buf[0] != '<' || buf[1] != 'S' || buf[2] != 'T' || buf[3] != 'X' || buf[4] != '=') return;

  size_t i = 5;
  size_t json_len = 0;
  while (i < len && isdigit(buf[i])) {
    json_len = (json_len * 10) + (buf[i] - '0');
    i++;
  }

  if (i >= len || buf[i] != '>') return;
  i++;
  if (i + json_len > len) return;

  out.parseJSON(buf + i, json_len);
}

static void handle_complete_message(const uint8_t* msg, size_t len) {
  if (!msg || len < 1) return;

  uint8_t traffic = msg[0];
  const uint8_t* payload = msg + 1;
  size_t payload_len = len - 1;

  if (!recv_cb[traffic]) return;

  Payload p;
  handle_receive_delimiters(traffic, payload, payload_len, p);
  recv_cb[traffic](p);
}

// =============================================================
// RX Tick (shared)
// =============================================================

static void transport_rx_tick(timepop_ctx_t*, void*) {

  uint8_t pkt[TRANSPORT_BLOCK_SIZE];

  if (!transport_recv_block(pkt)) return;

  if (rx_len + TRANSPORT_BLOCK_SIZE > STREAM_RX_BUF_MAX) {
    rx_len = 0;
    return;
  }

  memcpy(rx_buf + rx_len, pkt, TRANSPORT_BLOCK_SIZE);
  rx_len += TRANSPORT_BLOCK_SIZE;

  if (rx_len < ETX_LEN) return;

  for (size_t i = 0; i + ETX_LEN <= rx_len; i++) {
    if (memcmp(rx_buf + i, ETX_SEQ, ETX_LEN) == 0) {
      handle_complete_message(rx_buf, rx_len);
      rx_len = 0;
      return;
    }
  }
}

// =============================================================
// Lifecycle
// =============================================================

void transport_init(void) {
  debug_log("fingerprint", BUILD_FINGERPINT);

#if defined(ZPNET_TRANSPORT_SELECTED_SERIAL)
  Serial.begin(115200);
#endif

  debug_log("transport", "*init*");

  timepop_arm(
    TIMEPOP_CLASS_RX_POLL,
    true,
    transport_rx_tick,
    nullptr,
    "transport-rx"
  );
}
