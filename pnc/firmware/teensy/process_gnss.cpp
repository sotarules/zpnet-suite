#include "process_gnss.h"

#include "config.h"
#include "event_bus.h"
#include "timepop.h"
#include "util.h"

#include <Arduino.h>
#include <math.h>

// ================================================================
// Configuration
// ================================================================

// GNSS ingestion cadence (ms)
static const uint32_t GNSS_POLL_INTERVAL_MS = 10;

// Budget constraints (preserved from legacy)
static const unsigned long GNSS_BUDGET_MS = 2;
static const uint32_t GNSS_MAX_BYTES = 512;

// ================================================================
// Legacy GNSS state (PRESERVED)
// ================================================================

// Sentence buffers (last-seen truth)
static char last_sentence[GNSS_LINE_MAX] = "";
static char last_crw[GNSS_LINE_MAX] = "";
static char last_crx[GNSS_LINE_MAX] = "";
static char last_crz[GNSS_LINE_MAX] = "";
static char last_zda[GNSS_LINE_MAX] = "";
static char last_rmc[GNSS_LINE_MAX] = "";
static char last_gga[GNSS_LINE_MAX] = "";

// Parsed authoritative fields
static float latitude_deg  = NAN;
static float longitude_deg = NAN;

// GNSS serial ingestion state
static char   lineBuf[GNSS_LINE_MAX];
static size_t lineLen = 0;
static unsigned long lastGnssByteMs = 0;

// ================================================================
// Helpers (PRESERVED)
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
// TimePop-driven ingestion
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

  // Reschedule (self-owned cadence)
  timepop_schedule(
    GNSS_POLL_INTERVAL_MS,
    TIMEPOP_UNITS_MILLISECONDS,
    gnss_poll_tick,
    nullptr,
    "gnss-poll"
  );
}

// ================================================================
// Process lifecycle
// ================================================================

static bool gnss_start(void) {
  Serial1.begin(GNSSDO_BAUD);

  lineLen = 0;
  lastGnssByteMs = 0;

  // Arm first ingestion tick
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
  // No hard shutdown of Serial1 required
  // Process simply ceases scheduling
}

// ================================================================
// QUERY (FULL INTROSPECTION)
// ================================================================

static String gnss_query(void) {
  String b;

  if (last_sentence[0]) {
    b += "\"raw_sentence\":\"";
    b += jsonEscape(last_sentence);
    b += "\"";
  }

  if (!isnan(latitude_deg) && !isnan(longitude_deg)) {
    if (b.length()) b += ",";
    b += "\"latitude_deg\":";
    b += latitude_deg;
    b += ",\"longitude_deg\":";
    b += longitude_deg;
  }

  if (last_zda[0]) {
    if (b.length()) b += ",";
    b += "\"raw_zda\":\"";
    b += jsonEscape(last_zda);
    b += "\"";
  }

  if (last_rmc[0]) {
    if (b.length()) b += ",";
    b += "\"raw_rmc\":\"";
    b += jsonEscape(last_rmc);
    b += "\"";
  }

  // Visibility for debugging
  if (b.length()) b += ",";
  b += "\"buffer_len\":";
  b += lineLen;
  b += ",\"last_byte_ms\":";
  b += lastGnssByteMs;

  return b;
}

// ================================================================
// COMMANDS
// ================================================================

static String gnss_command(const char* cmd, const char* /*args*/) {

  enqueueEvent("GNSS_STATUS", "\"status\":\"fire\"");

  if (strcmp(cmd, "EMIT_STATUS") == 0) {

    if (last_sentence[0]) {
      String body;
      body += "\"raw_sentence\":\"";
      body += jsonEscape(last_sentence);
      body += "\"";

      enqueueEvent("GNSS_STATUS", body);
    }

    return "\"ok\":true";
  }

  if (strcmp(cmd, "EMIT_DATA") == 0) {

    String body;
    bool have = false;

    if (last_rmc[0]) {
      body += "\"raw_rmc\":\"";
      body += jsonEscape(last_rmc);
      body += "\"";
      have = true;
    }

    if (last_zda[0]) {
      if (have) body += ",";
      body += "\"raw_zda\":\"";
      body += jsonEscape(last_zda);
      body += "\"";
      have = true;
    }

    if (!isnan(latitude_deg) && !isnan(longitude_deg)) {
      if (have) body += ",";
      body += "\"latitude_deg\":";
      body += latitude_deg;
      body += ",\"longitude_deg\":";
      body += longitude_deg;
    }

    enqueueEvent("GNSS_DATA", body);

    return "\"ok\":true";
  }

  // Unknown process command
  return "\"error\":\"unknown gnss command\"";
}


// ================================================================
// Process registration
// ================================================================

static const process_vtable_t GNSS_PROCESS = {
  .name    = "GNSS",
  .start   = gnss_start,
  .stop    = gnss_stop,
  .query   = gnss_query,
  .command = gnss_command
};

void process_gnss_register(void) {
  process_register(PROCESS_TYPE_GNSS, &GNSS_PROCESS);
}
