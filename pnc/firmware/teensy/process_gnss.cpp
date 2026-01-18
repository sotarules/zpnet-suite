#include "process_gnss.h"

#include "config.h"
#include "event_bus.h"
#include "timepop.h"
#include "util.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

// ================================================================
// Configuration
// ================================================================

static const uint32_t GNSS_POLL_INTERVAL_MS = 10;
static const unsigned long GNSS_BUDGET_MS  = 2;
static const uint32_t GNSS_MAX_BYTES       = 512;

// ================================================================
// GNSS State (Authoritative)
// ================================================================

static char last_sentence[GNSS_LINE_MAX] = "";
static char last_crw[GNSS_LINE_MAX] = "";
static char last_crx[GNSS_LINE_MAX] = "";
static char last_crz[GNSS_LINE_MAX] = "";
static char last_zda[GNSS_LINE_MAX] = "";
static char last_rmc[GNSS_LINE_MAX] = "";
static char last_gga[GNSS_LINE_MAX] = "";

static float latitude_deg  = NAN;
static float longitude_deg = NAN;

static char   lineBuf[GNSS_LINE_MAX];
static size_t lineLen = 0;
static unsigned long lastGnssByteMs = 0;

// ================================================================
// Helpers
// ================================================================

static inline bool startsWith(const char* s, const char* p) {
  return strncmp(s, p, strlen(p)) == 0;
}

static float nmeaLatLonToDeg(const char* ddmm, const char* hemi) {
  if (!ddmm || !hemi) return NAN;

  double v = atof(ddmm);
  if (v <= 0.0) return NAN;

  double deg = floor(v / 100.0);
  double min = v - deg * 100.0;
  double out = deg + min / 60.0;

  if (hemi[0] == 'S' || hemi[0] == 'W') out = -out;
  return (float)out;
}

static void parseRMC(const char* l) {
  char lat[16], latH[4], lon[16], lonH[4];

  if (sscanf(
        l,
        "%*[^,],%*[^,],A,%15[^,],%3[^,],%15[^,],%3[^,]",
        lat, latH, lon, lonH
      ) == 4) {
    latitude_deg  = nmeaLatLonToDeg(lat, latH);
    longitude_deg = nmeaLatLonToDeg(lon, lonH);
  }
}

// Extract TPS mode from PERDCR* sentences (e.g. TPS4)
static void extractDisciplineMode(const char* line, String& out) {
  if (!line) return;

  const char* p = strchr(line, ',');
  if (!p) return;
  p++;

  const char* q = strchr(p, ',');
  if (!q) return;

  size_t len = q - p;
  if (len == 0 || len >= 16) return;

  char buf[16];
  memcpy(buf, p, len);
  buf[len] = '\0';

  out = buf;
}

static void ingestGnssLine(const char* line) {
  if (!line || !*line) return;

  safeCopy(last_sentence, sizeof(last_sentence), line);

  if      (startsWith(line, "$PERDCRW,")) safeCopy(last_crw, sizeof(last_crw), line);
  else if (startsWith(line, "$PERDCRX,")) safeCopy(last_crx, sizeof(last_crx), line);
  else if (startsWith(line, "$PERDCRZ,")) safeCopy(last_crz, sizeof(last_crz), line);
  else if (startsWith(line, "$GPZDA,"))   safeCopy(last_zda, sizeof(last_zda), line);
  else if (startsWith(line, "$GNRMC,") ||
           startsWith(line, "$GPRMC,")) {
    safeCopy(last_rmc, sizeof(last_rmc), line);
    parseRMC(line);
  }
  else if (startsWith(line, "$GNGGA,") ||
           startsWith(line, "$GPGGA,")) {
    safeCopy(last_gga, sizeof(last_gga), line);
  }
}

// ================================================================
// TimePop-driven ingestion (GOOD CITIZEN)
// ================================================================

static void gnss_poll_tick(void*) {
  const unsigned long start_ms = millis();
  uint32_t bytes = 0;

  while (Serial1.available()) {
    char c = Serial1.read();
    lastGnssByteMs = millis();

    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen) ingestGnssLine(lineBuf);
      lineLen = 0;
    }
    else if (c != '\r' && lineLen < GNSS_LINE_MAX - 1) {
      lineBuf[lineLen++] = c;
    }

    bytes++;
    if (bytes >= GNSS_MAX_BYTES) break;
    if (millis() - start_ms >= GNSS_BUDGET_MS) break;
  }

  if (lineLen && (millis() - lastGnssByteMs) > GNSS_SILENCE_FLUSH_MS) {
    lineBuf[lineLen] = '\0';
    ingestGnssLine(lineBuf);
    lineLen = 0;
  }

  timepop_schedule(
    GNSS_POLL_INTERVAL_MS,
    TIMEPOP_UNITS_MILLISECONDS,
    gnss_poll_tick,
    nullptr,
    "gnss-poll"
  );
}

// ================================================================
// Lifecycle
// ================================================================

static bool gnss_start(void) {
  Serial1.begin(GNSSDO_BAUD);

  lineLen = 0;
  lastGnssByteMs = 0;

  timepop_schedule(
    GNSS_POLL_INTERVAL_MS,
    TIMEPOP_UNITS_MILLISECONDS,
    gnss_poll_tick,
    nullptr,
    "gnss-poll"
  );

  return true;
}

static void gnss_stop(void) {
  // Passive stop
}

// ================================================================
// Introspection (PROCESS.QUERY)
// ================================================================

static String gnss_query(void) {
  String b;
  b += "\"buffer_len\":";
  b += lineLen;
  b += ",\"last_byte_ms\":";
  b += lastGnssByteMs;
  return b;
}

// ================================================================
// Command Implementation — REPORT (CommandResponse with payload object)
// ================================================================

static String cmd_report(const char* /*args*/) {
  // ------------------------------------------------------------
  // Build payload object (flat GNSS report)
  // ------------------------------------------------------------
  String payload;
  payload += "{";

  bool first = true;
  auto sep = [&]() {
    if (!first) payload += ",";
    first = false;
  };

  // Last raw sentence
  if (last_sentence[0]) {
    sep();
    payload += "\"last_sentence\":\"";
    payload += jsonEscape(last_sentence);
    payload += "\"";

    // Sentence type
    if (last_sentence[0] == '$') {
      const char* end = strchr(last_sentence, ',');
      if (end) {
        size_t len = (size_t)(end - last_sentence - 1);
        if (len > 0 && len < 16) {
          char buf[16];
          memcpy(buf, last_sentence + 1, len);
          buf[len] = '\0';

          sep();
          payload += "\"sentence_type\":\"";
          payload += buf;
          payload += "\"";
        }
      }
    }

    // Discipline mode (TPS*)
    if (startsWith(last_sentence, "$PERDCR")) {
      String mode;
      extractDisciplineMode(last_sentence, mode);
      if (mode.length()) {
        sep();
        payload += "\"discipline_mode\":\"";
        payload += mode;
        payload += "\"";
      }
    }
  }

  // Parsed position
  if (!isnan(latitude_deg) && !isnan(longitude_deg)) {
    sep();
    payload += "\"latitude_deg\":";
    payload += latitude_deg;

    sep();
    payload += "\"longitude_deg\":";
    payload += longitude_deg;
  }

  // Canonical raw sentences (trace-friendly)
  if (last_rmc[0]) {
    sep();
    payload += "\"raw_rmc\":\"";
    payload += jsonEscape(last_rmc);
    payload += "\"";
  }

  if (last_zda[0]) {
    sep();
    payload += "\"raw_zda\":\"";
    payload += jsonEscape(last_zda);
    payload += "\"";
  }

  if (last_crz[0]) {
    sep();
    payload += "\"raw_crz\":\"";
    payload += jsonEscape(last_crz);
    payload += "\"";
  }

  payload += "}";

  return payload;
}

// ================================================================
// Command Table
// ================================================================

static const process_command_entry_t GNSS_COMMANDS[] = {
  { "REPORT", cmd_report },
};

// ================================================================
// Process Registration
// ================================================================

static const process_vtable_t GNSS_PROCESS = {
  .name          = "GNSS",
  .start         = gnss_start,
  .stop          = gnss_stop,
  .query         = gnss_query,
  .commands      = GNSS_COMMANDS,
  .command_count = sizeof(GNSS_COMMANDS) / sizeof(GNSS_COMMANDS[0]),
};

void process_gnss_register(void) {
  process_register(PROCESS_TYPE_GNSS, &GNSS_PROCESS);
}
