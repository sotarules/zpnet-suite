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
// IMPORTANT (Teensy RawHID semantics):
//   • RawHID.send()/recv() operate on a fixed 64-byte PAYLOAD.
//   • There is NO explicit report-id byte visible here.
//   • Multiplexing must be done in-band.
//   • In our policy:
//       - Debug channel uses an in-band marker 0xD0 (handled in debug.cpp).
//       - Transport frames are plain ASCII beginning with "<STX=".
//

static constexpr size_t   HID_PACKET_SIZE   = 64;
static constexpr uint32_t HID_RX_TIMEOUT_MS = 0;

// -------------------------------------------------------------
// Framing constants
// -------------------------------------------------------------

static constexpr uint8_t ASCII_LT = (uint8_t)'<';
static const char ETX_SEQ[] = "<ETX>";
static constexpr size_t ETX_LEN = 5;

// -------------------------------------------------------------
// RX ingestion (TimePop-owned)
// -------------------------------------------------------------
//
// Strategy:
//   • Accumulate RawHID payload bytes into a binary-safe buffer.
//   • Ignore zero padding bytes (host pads HID reports).
//   • Scan for "<STX=" inside the buffer.
//   • When a full frame is present, extract JSON and dispatch.
//   • Consume exactly one frame per TimePop tick (intentional throttling).
//
// Hardening:
//   • If buffer approaches capacity, compact to the next '<' or reset.
//   • If malformed framing is detected, drop forward to re-sync.

static inline void compact_or_reset(uint8_t* buf, size_t& len) {
  if (len < sizeof(buf)) return;

  // Find next plausible frame start
  size_t stx = 0;
  while (stx < len && buf[stx] != ASCII_LT) stx++;

  if (stx >= len) {
    len = 0;
    return;
  }

  // Compact from that point
  memmove(buf, buf + stx, len - stx);
  len -= stx;
}

static void transport_rx_tick(timepop_ctx_t*, void*) {

  static uint8_t buf[512];
  static size_t  len = 0;

  uint8_t hid[HID_PACKET_SIZE];
  int n = RawHID.recv(hid, HID_RX_TIMEOUT_MS);
  if (n <= 0) return;

  // -----------------------------------------------------------
  // Append HID payload bytes (binary-safe), skipping zero padding
  // -----------------------------------------------------------
  for (int i = 0; i < n; i++) {
    uint8_t b = hid[i];

    // Host-side HID writes are zero-padded; transport is ASCII framed.
    if (b == 0x00) continue;

    if (len >= sizeof(buf)) {
      compact_or_reset(buf, len);
      if (len >= sizeof(buf)) {
        // Still full -> hard reset
        len = 0;
      }
    }

    buf[len++] = b;
  }

  // Need at least "<STX=1><ETX>" (11 bytes) before we can possibly parse
  if (len < 11) return;

  // -----------------------------------------------------------
  // Scan for a framed message start
  // -----------------------------------------------------------
  for (size_t i = 0; i + 5 < len; i++) {

    // Match "<STX="
    if (buf[i] != '<' ||
        buf[i + 1] != 'S' ||
        buf[i + 2] != 'T' ||
        buf[i + 3] != 'X' ||
        buf[i + 4] != '=') {
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
    if (j >= len) {
      // Header incomplete (need more bytes)
      return;
    }
    if (buf[j] != '>') {
      // Malformed header -> drop up to the next byte after '<' and resync
      memmove(buf, buf + i + 1, len - (i + 1));
      len -= (i + 1);
      return;
    }

    // Bounds check payload
    if (msg_len <= 0 || (size_t)msg_len > TRANSPORT_MAX_PAYLOAD) {
      // Impossible or out-of-policy length -> resync
      memmove(buf, buf + i + 1, len - (i + 1));
      len -= (i + 1);
      return;
    }

    size_t json_start = j + 1;

    // Need JSON + "<ETX>"
    if (json_start + (size_t)msg_len + ETX_LEN > len) {
      // Frame incomplete (need more bytes)
      return;
    }

    // Verify "<ETX>"
    size_t etx = json_start + (size_t)msg_len;
    if (buf[etx]     != '<' ||
        buf[etx + 1] != 'E' ||
        buf[etx + 2] != 'T' ||
        buf[etx + 3] != 'X' ||
        buf[etx + 4] != '>') {
      // Malformed frame -> drop forward to next '<'
      memmove(buf, buf + i + 1, len - (i + 1));
      len -= (i + 1);
      return;
    }

    // Copy JSON into a null-terminated buffer
    static char json[TRANSPORT_MAX_PAYLOAD + 1];
    memcpy(json, buf + json_start, (size_t)msg_len);
    json[msg_len] = '\0';

    // Dispatch to command layer
    command_exec(json);

    // Consume frame bytes from accumulator
    size_t consumed = etx + ETX_LEN;
    memmove(buf, buf + consumed, len - consumed);
    len -= consumed;

    // One frame per tick (intentional throttling)
    return;
  }

  // If we scanned the buffer and found no '<STX=', trim leading noise.
  // Keep a small tail in case "<STX=" straddles a boundary.
  if (len > 64) {
    size_t keep = 64;
    memmove(buf, buf + (len - keep), keep);
    len = keep;
  }
}

// -------------------------------------------------------------
// Lifecycle
// -------------------------------------------------------------

void transport_init(void) {

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
//
// IMPORTANT:
//   • We send RAW 64-byte payload packets via RawHID.
//   • No synthetic "report id" byte is included.
//   • Host will classify debug vs transport in-band:
//
//        debug   : payload[0] == 0xD0   (debug.cpp)
//        transport: payload[0] == '<'   ("<STX=")
//
// Hardening:
//   • Always send full 64-byte packets, zero padded.
//   • Bound payload size to TRANSPORT_MAX_PAYLOAD.

static inline void hid_send_payload_bytes(const uint8_t* data, size_t len) {
  if (!data || len == 0) return;

  uint8_t pkt[HID_PACKET_SIZE];
  size_t offset = 0;

  while (offset < len) {
    size_t chunk = len - offset;
    if (chunk > HID_PACKET_SIZE) {
      chunk = HID_PACKET_SIZE;
    }

    memset(pkt, 0, sizeof(pkt));
    memcpy(pkt, data + offset, chunk);

    // RawHID payload is always 64 bytes.
    RawHID.send(pkt, 50);

    offset += chunk;
  }
}

void transport_send_frame(const char* payload, size_t length) {

  if (!payload || length == 0) return;
  if (length > TRANSPORT_MAX_PAYLOAD) return;

  char header[16];
  int header_len =
      snprintf(header, sizeof(header), "<STX=%u>", (unsigned)length);

  static const char etx[] = "<ETX>";

  hid_send_payload_bytes((const uint8_t*)header, (size_t)header_len);
  hid_send_payload_bytes((const uint8_t*)payload, length);
  hid_send_payload_bytes((const uint8_t*)etx, sizeof(etx) - 1);
}

void transport_send_frame(const char* payload) {
  if (!payload) return;
  transport_send_frame(payload, strlen(payload));
}
