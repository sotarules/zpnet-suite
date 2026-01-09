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
// Tempo profiling state
// --------------------------------------------------------------
// NOTE:
// GNSS 10 MHz counters are intentionally inert in this version.
// They will be driven by a hardware timer in a future revision.
//
static uint64_t gnss_10mhz_ticks_minute = 0;
static uint64_t gnss_10mhz_ticks_total  = 0;

// Placeholder for future Kyocera OCXO
static uint64_t ocxo_10mhz_ticks_minute = 0;
static uint64_t ocxo_10mhz_ticks_total  = 0;

static uint64_t teensy_cycles_minute = 0;
static uint64_t teensy_cycles_total  = 0;

static uint32_t last_cycle_snapshot = 0;

static bool     tempo_active = false;
static bool     tempo_stop_requested = false;

static float    tempo_altitude_m = NAN;
static uint32_t tempo_start_ms = 0;

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
  pinMode(GNSS_VCLK_PIN, INPUT);

  // Enable ARM DWT cycle counter (600 MHz)
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
  ARM_DWT_CYCCNT = 0;

  // GNSS VCLOCK handling intentionally disabled here.
  // Hardware timer integration will reintroduce counting safely.
}

void gnss_poll() {
  // ------------------------------------------------------------
  // GNSS serial ingestion
  // ------------------------------------------------------------
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

  // ------------------------------------------------------------
  // Tempo profiling (Teensy cycles only, GNSS stubbed)
  // ------------------------------------------------------------
  if (!tempo_active) return;

  uint32_t now_cycles = ARM_DWT_CYCCNT;
  uint32_t delta = now_cycles - last_cycle_snapshot;
  last_cycle_snapshot = now_cycles;

  teensy_cycles_minute += delta;
  teensy_cycles_total  += delta;

  if (teensy_cycles_minute < 600000000ULL) return;

  // Emit TEMPO_PROFILE (GNSS/OCXO counts intentionally zero)
  String b;
  b += "\"run_start_ms\":";
  b += tempo_start_ms;

  if (!isnan(tempo_altitude_m)) {
    b += ",\"altitude_m\":";
    b += tempo_altitude_m;
  }

  b += ",\"teensy_cycles_minute\":";
  b += teensy_cycles_minute;
  b += ",\"gnss_10mhz_minute\":";
  b += gnss_10mhz_ticks_minute;
  b += ",\"ocxo_10mhz_minute\":";
  b += ocxo_10mhz_ticks_minute;

  b += ",\"teensy_cycles_total\":";
  b += teensy_cycles_total;
  b += ",\"gnss_10mhz_total\":";
  b += gnss_10mhz_ticks_total;
  b += ",\"ocxo_10mhz_total\":";
  b += ocxo_10mhz_ticks_total;

  enqueueEvent("TEMPO_PROFILE", b);

  // Reset minute buckets
  teensy_cycles_minute = 0;
  gnss_10mhz_ticks_minute = 0;
  ocxo_10mhz_ticks_minute = 0;

  if (tempo_stop_requested) {
    tempo_active = false;
    tempo_stop_requested = false;
  }
}

void gnss_tempo_start(float altitude_m) {
  gnss_10mhz_ticks_minute = 0;
  gnss_10mhz_ticks_total  = 0;
  ocxo_10mhz_ticks_minute = 0;
  ocxo_10mhz_ticks_total  = 0;

  teensy_cycles_minute = 0;
  teensy_cycles_total  = 0;

  tempo_altitude_m = altitude_m;
  tempo_start_ms = millis();

  last_cycle_snapshot = ARM_DWT_CYCCNT;

  tempo_active = true;
  tempo_stop_requested = false;
}

void gnss_tempo_stop() {
  if (tempo_active) {
    tempo_stop_requested = true;
  }
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
