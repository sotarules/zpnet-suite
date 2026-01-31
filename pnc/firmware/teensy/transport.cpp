// transport.cpp — ZPNet Transport (Teensy)
// ---------------------------------------
// HID: fixed 64-byte record device
// SERIAL: unstructured byte stream
//
// Shared above framing boundary:
//   • traffic byte semantics
//   • <STX=n> JSON <ETX> framing
//   • message-atomic dispatch
//
// Physical truth is respected below.
//

#include "transport.h"
#include "config.h"
#include "debug.h"
#include "timepop.h"

#include <Arduino.h>
#include <string.h>
#include <ctype.h>

// =============================================================
// Constants
// =============================================================

static constexpr size_t TRANSPORT_BLOCK_SIZE = 64;

static constexpr char   ETX_SEQ[] = "<ETX>";
static constexpr size_t ETX_LEN  = 5;

static constexpr size_t RX_BUF_MAX = 2048;

// =============================================================
// State
// =============================================================

static transport_receive_cb_t recv_cb[256] = { nullptr };

static uint8_t rx_buf[RX_BUF_MAX];
static size_t  rx_len = 0;

static bool    rx_have_traffic = false;
static uint8_t rx_traffic = 0;

// =============================================================
// Helpers
// =============================================================

static inline bool is_valid_traffic(uint8_t b) {
  return
    b == TRAFFIC_DEBUG ||
    b == TRAFFIC_REQUEST_RESPONSE ||
    b == TRAFFIC_PUBLISH_SUBSCRIBE;
}

// =============================================================
// Transport fingerprint
// =============================================================

#if defined(ZPNET_TRANSPORT_SELECTED_HID)
static const char* BUILD_FINGERPRINT = "__FP__ZPNET_HID__";
#elif defined(ZPNET_TRANSPORT_SELECTED_SERIAL)
static const char* BUILD_FINGERPRINT = "__FP__ZPNET_SERIAL__";
#endif

// =============================================================
// SEND: Physical backend
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
// Registration
// =============================================================

void transport_register_receive_callback(
  uint8_t traffic,
  transport_receive_cb_t cb
) {
  recv_cb[traffic] = cb;
}

// =============================================================
// SEND: Fragmentation (shared)
// =============================================================

static void fragment_and_send(
  uint8_t traffic,
  const uint8_t* buf,
  size_t len
) {
  uint8_t pkt[TRANSPORT_BLOCK_SIZE];
  size_t offset = 0;

  // First block carries traffic byte
  pkt[0] = traffic;

  size_t first = min(len, TRANSPORT_BLOCK_SIZE - 1);
  memcpy(pkt + 1, buf, first);

  size_t padding_len = TRANSPORT_BLOCK_SIZE - (1 + first);
  memset(pkt + 1 + first, 0, padding_len);

  transport_send_block(pkt);

  offset += first;

  // Continuation blocks
  while (offset < len) {
    size_t remaining_bytes = len - offset;
    size_t chunk = min(remaining_bytes, TRANSPORT_BLOCK_SIZE);
    memcpy(pkt, buf + offset, chunk);
    memset(pkt + chunk, 0, TRANSPORT_BLOCK_SIZE - chunk);
    transport_send_block(pkt);
    offset += chunk;
  }
}

// =============================================================
// SEND: Semantic entry
// =============================================================

void transport_send(
  uint8_t traffic,
  const Payload& payload
) {
  static uint8_t send_buf[TRANSPORT_MAX_MESSAGE];

  JsonView v = payload.json_view();
  const size_t json_len = v.len;

  char header[32];
  int header_len = snprintf(header, sizeof(header),
                            "<STX=%u>", (unsigned)json_len);
  if (header_len <= 0) return;

  const size_t total_len = header_len + json_len + ETX_LEN;
  if (total_len > TRANSPORT_MAX_MESSAGE) return;

  memcpy(send_buf, header, header_len);
  memcpy(send_buf + header_len, v.data, json_len);
  memcpy(send_buf + header_len + json_len, ETX_SEQ, ETX_LEN);

  fragment_and_send(traffic, send_buf, total_len);
}

// =============================================================
// RECEIVE: Message completion + dispatch (shared)
// =============================================================

static void handle_receive_delimiters(
  uint8_t, const uint8_t* buf, size_t len, Payload& out
) {
  size_t i = 5, n = 0;                // skip "<STX="
  while (buf[i] >= '0' && buf[i] <= '9')
    n = n * 10 + (buf[i++] - '0');
  out.parseJSON(buf + i + 1, n);      // skip '>' and parse JSON
}

static void dispatch_if_complete() {
  if (rx_len < ETX_LEN) return;

  for (size_t i = 0; i + ETX_LEN <= rx_len; i++) {
    if (memcmp(rx_buf + i, ETX_SEQ, ETX_LEN) == 0) {

      if (!rx_have_traffic) {
        rx_len = 0;
        return;
      }

      Payload p;
      handle_receive_delimiters(
        rx_traffic,
        rx_buf,
        rx_len,
        p
      );

      if (recv_cb[rx_traffic]) {
        recv_cb[rx_traffic](p);
      }

      rx_len = 0;
      rx_have_traffic = false;
      return;
    }
  }
}

// =============================================================
// RECEIVE: HID (record dispensing)
// =============================================================

#if defined(ZPNET_TRANSPORT_SELECTED_HID)

static void rx_hid_tick() {
  uint8_t block[TRANSPORT_BLOCK_SIZE];
  if (RawHID.recv(block, 0) <= 0) return;

  size_t offset = 0;

  if (!rx_have_traffic) {
    rx_traffic = block[0];
    rx_have_traffic = true;
    offset = 1;
  }

  size_t copy = TRANSPORT_BLOCK_SIZE - offset;
  if (rx_len + copy > RX_BUF_MAX) {
    rx_len = 0;
    rx_have_traffic = false;
    return;
  }

  memcpy(rx_buf + rx_len, block + offset, copy);
  rx_len += copy;

  dispatch_if_complete();
}

#endif

// =============================================================
// RECEIVE: SERIAL (stream accumulation)
// =============================================================

#if defined(ZPNET_TRANSPORT_SELECTED_SERIAL)

static void rx_serial_tick() {
  while (Serial.available()) {
    uint8_t b = Serial.read();

    // ---------------------------------------------------------
    // Awaiting traffic byte (message start)
    // ---------------------------------------------------------
    if (!rx_have_traffic) {

      // Ignore noise until we see a valid traffic classifier
      if (!is_valid_traffic(b)) {
        continue;
      }

      rx_traffic = b;
      rx_have_traffic = true;
      rx_len = 0;  // explicit reset: beginning of a new message
      continue;
    }

    // ---------------------------------------------------------
    // Accumulating payload bytes
    // ---------------------------------------------------------
    if (rx_len >= RX_BUF_MAX) {
      // Overflow → abandon message atomically
      rx_len = 0;
      rx_have_traffic = false;
      return;
    }

    rx_buf[rx_len++] = b;
    dispatch_if_complete();
  }
}

#endif


// =============================================================
// RX Poll
// =============================================================

static void transport_rx_tick(timepop_ctx_t*, void*) {

#if defined(ZPNET_TRANSPORT_SELECTED_HID)
  rx_hid_tick();
#elif defined(ZPNET_TRANSPORT_SELECTED_SERIAL)
  rx_serial_tick();
#endif

}

// =============================================================
// Lifecycle
// =============================================================

void transport_init(void) {

#if defined(ZPNET_TRANSPORT_SELECTED_SERIAL)
  Serial.begin(115200);
#endif

  debug_log("transport", BUILD_FINGERPRINT);

  timepop_arm(
    TIMEPOP_CLASS_RX_POLL,
    true,
    transport_rx_tick,
    nullptr,
    "transport-rx"
  );
}