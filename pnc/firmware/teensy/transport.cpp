#include "transport.h"
#include "config.h"
#include "debug.h"
#include "timepop.h"
#include "events.h"
#include "process.h"

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
// Protocol parsing + dispatch
// -------------------------------------------------------------

static void process_complete_message(const uint8_t* msg, size_t len) {

  // ---------------- protocol framing ----------------

  if (len < 11 || msg[0] != ASCII_LT) return;
  if (memcmp(msg, STX_PREFIX, 5) != 0) return;

  size_t i = 5;
  int payload_len = 0;

  while (i < len && msg[i] >= '0' && msg[i] <= '9') {
    payload_len = payload_len * 10 + (msg[i] - '0');
    i++;
  }

  if (i >= len || msg[i] != '>') return;
  if (payload_len <= 0 || (size_t)payload_len > TRANSPORT_MAX_PAYLOAD) return;

  size_t json_start = i + 1;
  size_t json_end   = json_start + (size_t)payload_len;

  if (json_end + ETX_LEN > len) return;
  if (memcmp(msg + json_end, ETX_SEQ, ETX_LEN) != 0) return;

  // ---------------- JSON extraction ----------------

  static char json[TRANSPORT_MAX_PAYLOAD + 1];
  memcpy(json, msg + json_start, (size_t)payload_len);
  json[payload_len] = '\0';

  // ---------------- minimal semantic extraction ----------------

  const char* subsystem = strstr(json, "\"subsystem\"");
  const char* command   = strstr(json, "\"command\"");

  if (!subsystem || !command) {
    // malformed message — do not recover
    return;
  }

  // Extract subsystem string
  subsystem = strchr(subsystem, ':') + 1;
  while (*subsystem == ' ' || *subsystem == '\"') subsystem++;
  const char* subsystem_end = strchr(subsystem, '\"');

  char subsystem_name[32];
  size_t sub_len = subsystem_end - subsystem;
  memcpy(subsystem_name, subsystem, sub_len);
  subsystem_name[sub_len] = '\0';

  // Extract command string
  command = strchr(command, ':') + 1;
  while (*command == ' ' || *command == '\"') command++;
  const char* command_end = strchr(command, '\"');

  char command_name[32];
  size_t cmd_len = command_end - command;
  memcpy(command_name, command, cmd_len);
  command_name[cmd_len] = '\0';

  // Optional args
  const char* args_json = nullptr;
  const char* a = strstr(json, "\"args\"");
  if (a) {
    a = strchr(a, ':');
    if (a) {
      a++;
      while (*a == ' ') a++;
      if (*a == '{') {
        args_json = a;
      }
    }
  }

  // Optional req_id
  int req_id = -1;
  const char* r = strstr(json, "\"req_id\"");
  if (r) {
    r = strchr(r, ':');
    if (r) {
      req_id = atoi(r + 1);
    }
  }

  // ---------------- process dispatch ----------------

  String inner_response;
  process_command(
    subsystem_name,
    command_name,
    args_json,
    inner_response
  );

  // ---------------- response envelope ----------------

  String response;
  if (req_id >= 0) {
    response += "{";
    response += "\"req_id\":";
    response += req_id;
    response += ",";
    response += inner_response.substring(1);
  } else {
    response = inner_response;
  }

  transport_send_frame(response.c_str(), response.length());
}

// -------------------------------------------------------------
// RX tick (TimePop-owned)
// -------------------------------------------------------------

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
      raw_msg_len = 0;
      return;
    }

    memcpy(raw_msg_buf + raw_msg_len, hid, end);
    raw_msg_len += end;
  }

  if (!has_padding) {
    return;
  }

  process_complete_message(raw_msg_buf, raw_msg_len);

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
// TX helpers (unchanged)
// -------------------------------------------------------------

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

  uint8_t frame[TRANSPORT_MAX_MESSAGE];
  size_t  frame_len = 0;

  int header_len = snprintf(
      (char*)frame,
      sizeof(frame),
      "<STX=%u>",
      (unsigned)length
  );

  if (header_len <= 0) return;
  frame_len += (size_t)header_len;

  memcpy(frame + frame_len, payload, length);
  frame_len += length;

  static constexpr char ETX[] = "<ETX>";
  memcpy(frame + frame_len, ETX, sizeof(ETX) - 1);
  frame_len += sizeof(ETX) - 1;

  hid_send_message_bytes(frame, frame_len);
}

void transport_send_frame(const char* payload) {
  if (!payload) return;
  transport_send_frame(payload, strlen(payload));
}
