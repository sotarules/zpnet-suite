#include "gnss.h"

#include "config.h"
#include "event_bus.h"
#include "util.h"

#include <Arduino.h>
#include <math.h>

// --------------------------------------------------------------
// Sentence buffers
// --------------------------------------------------------------
static char last_sentence[GNSS_LINE_MAX] = "";

static char last_crw[GNSS_LINE_MAX] = "";
static char last_crx[GNSS_LINE_MAX] = "";
static char last_crz[GNSS_LINE_MAX] = "";
static char last_zda[GNSS_LINE_MAX] = "";
static char last_rmc[GNSS_LINE_MAX] = "";
static char last_gga[GNSS_LINE_MAX] = "";

// --------------------------------------------------------------
// Parsed authoritative fields
// --------------------------------------------------------------
static float latitude_deg  = NAN;
static float longitude_deg = NAN;

// --------------------------------------------------------------
// GNSS serial ingestion state
// --------------------------------------------------------------
static char lineBuf[GNSS_LINE_MAX];
static size_t lineLen = 0;
static unsigned long lastGnssByteMs = 0;

// --------------------------------------------------------------
// PPS / VCLOCK
// --------------------------------------------------------------
static volatile bool     pps_seen   = false;
static volatile uint32_t pps_cycles = 0;
static volatile uint32_t pps_count  = 0;

static uint32_t last_pps_emit_ms = 0;

// --------------------------------------------------------------
// Helpers
// --------------------------------------------------------------
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

  if (sscanf(l,
             "%*[^,],%*[^,],A,%15[^,],%3[^,],%15[^,],%3[^,]",
             lat, latH, lon, lonH) == 4) {
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

// --------------------------------------------------------------
// Public API
// --------------------------------------------------------------
void gnss_init() {
  Serial1.begin(GNSSDO_BAUD);

  pinMode(GNSS_PPS_PIN, INPUT);
  pinMode(GNSS_VCLK_PIN, INPUT);

  // Enable ARM DWT cycle counter (Teensy 4.1)
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
  ARM_DWT_CYCCNT = 0;

  attachInterrupt(
    digitalPinToInterrupt(GNSS_PPS_PIN),
    []() {
      pps_cycles = ARM_DWT_CYCCNT;
      pps_count++;
      pps_seen = true;
    },
    RISING
  );
}

void gnss_poll() {
  const unsigned long start_ms = millis();
  const unsigned long BUDGET_MS = 2;
  const uint32_t MAX_BYTES = 512;
  uint32_t bytes = 0;

  while (Serial1.available()) {
    char c = Serial1.read();
    lastGnssByteMs = millis();

    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen) ingestGnssLine(lineBuf);
      lineLen = 0;
    } else if (c != '\r' && lineLen < GNSS_LINE_MAX - 1) {
      lineBuf[lineLen++] = c;
    }

    bytes++;
    if (bytes >= MAX_BYTES) break;
    if (millis() - start_ms >= BUDGET_MS) break;
  }

  if (lineLen && (millis() - lastGnssByteMs) > GNSS_SILENCE_FLUSH_MS) {
    lineBuf[lineLen] = '\0';
    ingestGnssLine(lineBuf);
    lineLen = 0;
  }
}

void gnss_handle_pps() {
  bool seen = false;
  uint32_t cyc = 0;
  uint32_t cnt = 0;

  noInterrupts();
  if (pps_seen) {
    seen = true;
    cyc = pps_cycles;
    cnt = pps_count;
    pps_seen = false;
  }
  interrupts();

  if (!seen) return;

  uint32_t now_ms = millis();
  if (now_ms - last_pps_emit_ms < PPS_EMIT_INTERVAL_MS) return;

  last_pps_emit_ms = now_ms;

  uint32_t vclk_cycles = ARM_DWT_CYCCNT;
  int vclk_level = digitalRead(GNSS_VCLK_PIN);

  String b;
  b += "\"millis\":";
  b += millis();
  b += ",\"pps_count\":";
  b += cnt;
  b += ",\"cycles\":";
  b += cyc;
  b += ",\"vclk_cycles\":";
  b += vclk_cycles;
  b += ",\"vclk_level\":";
  b += vclk_level;
  b += ",\"emit_interval_ms\":";
  b += PPS_EMIT_INTERVAL_MS;

  enqueueEvent("GNSS_PPS", b);

  noInterrupts();
  pps_count = 0;
  interrupts();
}

String buildGnssStatusBody() {
  String b;

  if (last_sentence[0]) {
    b += "\"raw_sentence\":\"";
    b += jsonEscape(last_sentence);
    b += "\"";
  }

  return b;
}

String buildGnssDataBody() {
  String b;

  if (!isnan(latitude_deg) && !isnan(longitude_deg)) {
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

  return b;
}
