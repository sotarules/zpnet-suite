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
static constexpr size_t RX_BUF_MAX           = 2048;

static constexpr char   STX_SEQ[] = "<STX=";
static constexpr size_t STX_LEN   = 5;

static constexpr char   ETX_SEQ[] = "<ETX>";
static constexpr size_t ETX_LEN   = 5;

// =============================================================
// State (shared semantic)
// =============================================================

static transport_receive_cb_t recv_cb[256] = { nullptr };

static uint8_t rx_buf[RX_BUF_MAX];
static size_t  rx_len = 0;

static bool    rx_have_traffic = false;
static uint8_t rx_traffic      = 0;

// =============================================================
// RX instrumentation (monotonic, non-perturbing)
// =============================================================
//
// These counters exist to prove transport behavior without
// emitting any debug traffic or altering RX/TX timing.
//
// All fields are monotonic.
// Best-effort snapshots are exported via transport_get_info().
//

static volatile uint32_t rx_blocks_total        = 0;
static volatile uint32_t rx_bytes_total         = 0;

static volatile uint32_t rx_frames_complete     = 0;
static volatile uint32_t rx_frames_dispatched   = 0;

static volatile uint32_t rx_reset_hard_count    = 0;
static volatile uint32_t rx_bad_stx_count       = 0;
static volatile uint32_t rx_bad_etx_count       = 0;
static volatile uint32_t rx_len_overflow_count  = 0;

static volatile uint32_t rx_overlap_count       = 0;

static volatile uint32_t rx_expected_traffic_missing     = 0;

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
  rx_reset_hard_count++;
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

  // Need at least "<STX=0>" to even begin parsing header.
  // (ETX is NOT required yet.)
  if (rx_len < (STX_LEN + 2)) return;

  // Frame must start with "<STX="
  if (memcmp(rx_buf, STX_SEQ, STX_LEN) != 0) {
    rx_bad_stx_count++;
    rx_reset_hard();
    return;
  }

  // Parse header: "<STX=" + digits + ">"
  size_t i = STX_LEN;        // position after "<STX="
  size_t declared_len = 0;
  bool   saw_digit = false;

  while (i < rx_len && rx_buf[i] != '>') {
    uint8_t c = rx_buf[i];

    if (c < '0' || c > '9') {
      // Corrupt header (not incomplete)
      rx_len_overflow_count++;
      rx_reset_hard();
      return;
    }

    saw_digit = true;
    declared_len = declared_len * 10 + (size_t)(c - '0');

    if (declared_len > TRANSPORT_MAX_MESSAGE) {
      rx_len_overflow_count++;
      rx_reset_hard();
      return;
    }

    ++i;
  }

  // Header incomplete — wait for more bytes
  if (!saw_digit) return;
  if (i >= rx_len) return;

  const size_t header_end = i;            // index of '>'
  const size_t json_start = header_end + 1;

  // Total bytes required for a COMPLETE frame
  const size_t required_total =
    json_start + declared_len + ETX_LEN;

  // Impossible framing — real error
  if (required_total > RX_BUF_MAX) {
    rx_len_overflow_count++;
    rx_reset_hard();
    return;
  }

  // Frame not complete yet — THIS IS THE KEY FIX
  // Missing ETX bytes are NOT an error.
  if (rx_len < required_total) {
    return;
  }

  // At this point ETX bytes are fully present.
  // Now and ONLY now may we validate them.
  const size_t etx_pos = json_start + declared_len;
  if (memcmp(rx_buf + etx_pos, ETX_SEQ, ETX_LEN) != 0) {
    rx_bad_etx_count++;
    debug_log("missing etx", rx_buf, rx_len);
    rx_reset_hard();
    return;
  }

  // We have one complete, well-formed frame.
  rx_frames_complete++;

  Payload p;
  p.parseJSON(rx_buf + json_start, declared_len);

  if (recv_cb[rx_traffic]) {
    rx_frames_dispatched++;
    recv_cb[rx_traffic](p);
  }

  // Reset receive state after dispatch.
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

  rx_blocks_total++;

  size_t offset = 0;

  if (!rx_have_traffic) {

    if (!is_valid_traffic(block[0])) {
      rx_expected_traffic_missing++;  // better name than invalid_traffic
      rx_reset_hard();                // or: return; or: discard block
      return;
    }

    rx_traffic = block[0];
    rx_have_traffic = true;
    offset = 1;
  }

  size_t copy = TRANSPORT_BLOCK_SIZE - offset;

  // Accounting: bytes appended into RX buffer (not including traffic byte)
  rx_bytes_total += (uint32_t)copy;

  if (rx_len + copy > RX_BUF_MAX) {
    rx_len = 0;
    rx_have_traffic = false;
    rx_reset_hard();
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
    uint8_t b = (uint8_t)Serial.read();

    // Await traffic byte
    if (!rx_have_traffic) {
      if (!is_valid_traffic(b)) {
        rx_expected_traffic_missing++;
        continue;
      }
      rx_traffic = b;
      rx_have_traffic = true;
      rx_len = 0;  // explicit reset: beginning of a new message
      continue;
    }

    // Optional overlap signal:
    // JSON is ASCII; a traffic byte (0xD0..0xD2) here strongly suggests
    // a new frame arrived before the previous completed.
    if (is_valid_traffic(b)) {
      rx_overlap_count++;
    }

    // Accumulate payload bytes
    if (rx_len >= RX_BUF_MAX) {
      rx_reset_hard();
      return;
    }

    rx_buf[rx_len++] = b;
    rx_bytes_total++;

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
// Transport diagnostics access (public)
// =============================================================

void transport_get_info(
  transport_info_t* out
) {
  if (!out) return;

  out->rx_blocks_total        = rx_blocks_total;
  out->rx_bytes_total         = rx_bytes_total;

  out->rx_frames_complete     = rx_frames_complete;
  out->rx_frames_dispatched   = rx_frames_dispatched;

  out->rx_reset_hard          = rx_reset_hard_count;

  out->rx_bad_stx             = rx_bad_stx_count;
  out->rx_bad_etx             = rx_bad_etx_count;
  out->rx_len_overflow        = rx_len_overflow_count;

  out->rx_overlap             = rx_overlap_count;
  out->rx_expected_traffic_missing = rx_expected_traffic_missing;
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
