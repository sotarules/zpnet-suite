#pragma once

#include "transport.h"

// ------------------------------------------------------------------
// IMPTRACE — Imperative Forensic Trace Channel
//
// Properties:
//   • Framed (transport_send_frame)
//   • Non-durable (bypasses event bus)
//   • Non-semantic (not telemetry)
//   • Compile-time gated
//   • Safe in tight loops
//   • Safe during partial failure
//
// Payload convention (intentionally minimal):
//   {"imptrace":"X"}   where X is a single ASCII marker
// ------------------------------------------------------------------

// Uncomment ONLY during forensic debugging
#define IMPTRACE_ENABLED

#ifdef IMPTRACE_ENABLED

static inline void imptrace_char(char c) {
  char buf[20];
  buf[0] = '{';
  buf[1] = '"';
  buf[2] = 'i';
  buf[3] = 'm';
  buf[4] = 'p';
  buf[5] = 't';
  buf[6] = 'r';
  buf[7] = 'a';
  buf[8] = 'c';
  buf[9] = 'e';
  buf[10] = '"';
  buf[11] = ':';
  buf[12] = '"';
  buf[13] = c;
  buf[14] = '"';
  buf[15] = '}';
  // length = 16 bytes
  transport_send_frame(buf, 16);
}

#define IMPTRACE_CHAR(c) imptrace_char(c)

#else

#define IMPTRACE_CHAR(c) do {} while (0)

#endif
