// transport.cpp — ZPNet Transport (Teensy)
// ---------------------------------------
// HID: fixed 64-byte record device
// SERIAL: unstructured byte stream
//
// Architecture (truthful split):
//   • HID TX
//   • HID RX
//   • SERIAL TX
//   • SERIAL RX
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
#include "process_performance.h"

#include <Arduino.h>
#include <string.h>
#include <ctype.h>

// =============================================================
// Constants
// =============================================================

static constexpr size_t TRANSPORT_BLOCK_SIZE = 64;
static constexpr size_t RX_BUF_MAX = 2048;

static constexpr char STX_SEQ[] = "<STX=";
static constexpr size_t STX_LEN = 5;

static constexpr char   ETX_SEQ[] = "<ETX>";
static constexpr size_t ETX_LEN  = 5;

// =============================================================
// State (shared semantic)
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
// SEND: Physical backends (explicit split)
// =============================================================
//
// HID TX remains block-based (mandatory 64-byte reports).
// SERIAL TX becomes stream-truthful: write EXACT byte lengths,
// no padding, no block fiction.
//   [traffic][<STX=n>JSON<ETX>]
//

#if defined(ZPNET_TRANSPORT_SELECTED_HID)

static inline bool hid_send_block(const uint8_t block[TRANSPORT_BLOCK_SIZE]) {
  return RawHID.send(block, 50) > 0;
}

#elif defined(ZPNET_TRANSPORT_SELECTED_SERIAL)

static inline bool serial_send_bytes(const uint8_t* buf, size_t len) {
  size_t n = Serial.write(buf, len);
  Serial.flush();
  return n == len;
}

#endif

// =============================================================
// Registration (shared)
// =============================================================

void transport_register_receive_callback(
  uint8_t traffic,
  transport_receive_cb_t cb
) {
  recv_cb[traffic] = cb;
}

// =============================================================
// SEND: Fragmentation (HID-only)
// =============================================================
//
// Fragmentation exists to satisfy HID’s fixed 64-byte record device.
// SERIAL TX bypasses this entirely (stream-truthful).
//

#if defined(ZPNET_TRANSPORT_SELECTED_HID)

static void fragment_and_send_hid(
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
  memset(pkt + 1 + first, 0, TRANSPORT_BLOCK_SIZE - (1 + first));

  hid_send_block(pkt);
  offset += first;

  // Continuation blocks
  while (offset < len) {
    size_t remaining_bytes = len - offset;
    size_t chunk = min(remaining_bytes, TRANSPORT_BLOCK_SIZE);

    memcpy(pkt, buf + offset, chunk);
    memset(pkt + chunk, 0, TRANSPORT_BLOCK_SIZE - chunk);

    hid_send_block(pkt);
    offset += chunk;
  }
}

#endif

// =============================================================
// SEND: Scheduled context payload
// =============================================================

struct transport_send_ctx_t {
  uint8_t traffic;
  Payload payload;
};

// =============================================================
// SEND: Physical implementation (scheduled context ONLY)
// =============================================================

static void transport_send_physical(
  uint8_t traffic,
  const Payload& payload
) {
  static uint8_t send_buf[TRANSPORT_MAX_MESSAGE];

  JsonView v = payload.json_view();
  const size_t json_len = v.len;

  char header[32];
  int header_len = snprintf(
    header, sizeof(header),
    "<STX=%u>", (unsigned)json_len
  );
  if (header_len <= 0) return;

  const size_t total_len =
    (size_t)header_len + json_len + ETX_LEN;

  if (total_len > TRANSPORT_MAX_MESSAGE) return;

  memcpy(send_buf, header, (size_t)header_len);
  memcpy(send_buf + (size_t)header_len, v.data, json_len);
  memcpy(send_buf + (size_t)header_len + json_len, ETX_SEQ, ETX_LEN);

#if defined(ZPNET_TRANSPORT_SELECTED_HID)

  fragment_and_send_hid(traffic, send_buf, total_len);

#elif defined(ZPNET_TRANSPORT_SELECTED_SERIAL)

  static uint8_t serial_tx_buf[1 + TRANSPORT_MAX_MESSAGE];

  serial_tx_buf[0] = traffic;
  memcpy(serial_tx_buf + 1, send_buf, total_len);

  serial_send_bytes(serial_tx_buf, 1 + total_len);

#endif
}

// =============================================================
// SEND: Semantic entry (NON-OPTIONAL scheduled isolation)
// =============================================================

void transport_send(
  uint8_t traffic,
  const Payload& payload
) {
  // Allocate explicit send context
  auto* ctx = new transport_send_ctx_t{
    traffic,
    payload.clone()
  };

  timepop_arm(
    TIMEPOP_CLASS_ASAP,
    false,
    [](timepop_ctx_t*, void* user_ctx) {

      auto* send = static_cast<transport_send_ctx_t*>(user_ctx);

      transport_send_physical(
        send->traffic,
        send->payload
      );

      delete send;
    },
    ctx,
    "transport-send"
  );
}

// =============================================================
// RECEIVE: Delimiter + dispatch (length-authoritative, shared)
// =============================================================
//
// Framing is EXACTLY:
//
//   <STX=n> JSON_BYTES <ETX>
//
// Rules (match transport.py):
//   • We only accept frames that begin with "<STX=" at rx_buf[0].
//   • We parse declared length n from the header.
//   • We require that exactly n JSON bytes follow '>'.
//   • We require "<ETX>" to appear exactly at (json_start + n).
//   • We dispatch exactly those n bytes (not "whatever precedes ETX").
//   • If framing is corrupt, we reset and resync hard (adversarial).
//
// Note: This intentionally does NOT “scan for ETX anywhere”.
//       It uses declared length as the authority.
//
static void rx_reset_hard() {
  rx_len = 0;
  rx_have_traffic = false;
  // rx_traffic is "latched" only when rx_have_traffic == true.
}

static void dispatch_if_complete() {
  // Must have traffic before we consider the buffer meaningful.
  if (!rx_have_traffic) {
    rx_len = 0;
    return;
  }

  // Need at least "<STX=0><ETX>" => 5 + 1 + 1 + 5 = 12 bytes
  // (We'll be stricter below; this is just a fast guard.)
  if (rx_len < (STX_LEN + 3 + ETX_LEN)) return;

  // Frame must start with "<STX="
  if (memcmp(rx_buf, STX_SEQ, STX_LEN) != 0) {
    rx_reset_hard();
    return;
  }

  // Parse header: "<STX=" + digits + ">"
  // Find '>' and parse the digits in between.
  size_t i = STX_LEN;        // position after "<STX="
  size_t declared_len = 0;
  bool   saw_digit = false;

  // Parse decimal digits until we hit '>'
  while (i < rx_len && rx_buf[i] != '>') {
    uint8_t c = rx_buf[i];

    if (c < '0' || c > '9') {
      // Bad header character: adversarial reset
      rx_reset_hard();
      return;
    }

    saw_digit = true;

    // Safe-ish decimal accumulate (cap to prevent overflow / nonsense)
    declared_len = declared_len * 10 + (size_t)(c - '0');
    if (declared_len > TRANSPORT_MAX_MESSAGE) {
      rx_reset_hard();
      return;
    }

    ++i;
  }

  // Must have a closing '>' and at least one digit
  if (!saw_digit) return;                 // incomplete header (no digits yet)
  if (i >= rx_len) return;                // incomplete header (no '>' yet)

  const size_t header_end = i;            // index of '>'
  const size_t json_start = header_end + 1;

  // Now we know exactly how many bytes must be present:
  //   json_start + declared_len + ETX_LEN
  const size_t required_total = json_start + declared_len + ETX_LEN;

  if (required_total > RX_BUF_MAX) {
    rx_reset_hard();
    return;
  }

  // Not enough bytes yet: keep accumulating
  if (rx_len < required_total) return;

  // ETX must appear EXACTLY after the declared JSON length
  const size_t etx_pos = json_start + declared_len;
  if (memcmp(rx_buf + etx_pos, ETX_SEQ, ETX_LEN) != 0) {
    rx_reset_hard();
    return;
  }

  // We have one complete, well-formed frame. Parse EXACTLY declared_len bytes.
  Payload p;
  p.parseJSON(rx_buf + json_start, declared_len);

  if (recv_cb[rx_traffic]) {
    recv_cb[rx_traffic](p);
  }

  // Reset receive state after dispatch.
  // If you ever support back-to-back frames in the same rx_buf,
  // this becomes a "consume" + memmove; but for now you reset hard.
  rx_reset_hard();
}


// =============================================================
// RECEIVE: HID RX (record-based)
// =============================================================

#if defined(ZPNET_TRANSPORT_SELECTED_HID)

static void rx_hid_tick() {

  transport_rx_entered();

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
// RECEIVE: SERIAL RX (stream-based)
// =============================================================

#if defined(ZPNET_TRANSPORT_SELECTED_SERIAL)

static void rx_serial_tick() {

  transport_rx_entered();

  while (Serial.available()) {
    uint8_t b = Serial.read();

    // Await traffic byte
    if (!rx_have_traffic) {
      if (!is_valid_traffic(b)) continue;

      rx_traffic = b;
      rx_have_traffic = true;
      rx_len = 0;  // explicit reset: beginning of a new message
      continue;
    }

    // Accumulate payload bytes
    if (rx_len >= RX_BUF_MAX) {
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
// RX Poll (timepop)
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
