#include "process_gnss.h"
#include "debug.h"
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
// Authoritative GNSS State
// ================================================================

struct gnss_state_t {

  // --- Time / Date (UTC, RMC-only authority) ---
  bool has_time = false;
  bool has_date = false;
  uint8_t  hour = 0, minute = 0, second = 0;
  uint16_t year = 0;
  uint8_t  month = 0, day = 0;

  // --- Position ---
  bool has_fix = false;
  uint8_t fix_quality = 0;
  uint8_t fix_type = 0;
  uint8_t satellites = 0;

  float latitude_deg  = NAN;
  float longitude_deg = NAN;
  float altitude_m    = NAN;

  // --- Navigation ---
  float speed_knots = NAN;
  float course_deg  = NAN;

  // --- Discipline ---
  bool has_discipline = false;
  char discipline_mode[8] = "";

  // --- Raw provenance ---
  char last_sentence[GNSS_LINE_MAX] = "";
  char last_rmc[GNSS_LINE_MAX] = "";
  char last_gga[GNSS_LINE_MAX] = "";
  char last_gsa[GNSS_LINE_MAX] = "";
  char last_zda[GNSS_LINE_MAX] = "";
  char last_gsv[GNSS_LINE_MAX] = "";
  char last_crw[GNSS_LINE_MAX] = "";
  char last_crx[GNSS_LINE_MAX] = "";
  char last_crz[GNSS_LINE_MAX] = "";

  // --- Liveness ---
  unsigned long last_rx_ms = 0;
};

static gnss_state_t GNSS;

// ================================================================
// Line Buffer
// ================================================================

static char   lineBuf[GNSS_LINE_MAX];
static size_t lineLen = 0;

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

// ================================================================
// Sentence Parsers (RMC is king)
// ================================================================

static void parseRMC(const char* l) {
  char time[16], date[16], lat[16], latH[4], lon[16], lonH[4];
  float spd = NAN, crs = NAN;

  if (sscanf(
        l,
        "%*[^,],%15[^,],A,%15[^,],%3[^,],%15[^,],%3[^,],%f,%f,%15[^,]",
        time, lat, latH, lon, lonH, &spd, &crs, date
      ) >= 7) {

    GNSS.latitude_deg  = nmeaLatLonToDeg(lat, latH);
    GNSS.longitude_deg = nmeaLatLonToDeg(lon, lonH);
    GNSS.speed_knots   = spd;
    GNSS.course_deg    = crs;
    GNSS.has_fix       = true;

    // --- Time HHMMSS ---
    if (strlen(time) >= 6) {
      GNSS.hour   = (time[0]-'0')*10 + (time[1]-'0');
      GNSS.minute = (time[2]-'0')*10 + (time[3]-'0');
      GNSS.second = (time[4]-'0')*10 + (time[5]-'0');
      GNSS.has_time = true;
    }

    // --- Date DDMMYY ---
    if (strlen(date) == 6) {
      GNSS.day   = (date[0]-'0')*10 + (date[1]-'0');
      GNSS.month = (date[2]-'0')*10 + (date[3]-'0');
      GNSS.year  = 2000 + (date[4]-'0')*10 + (date[5]-'0');
      GNSS.has_date = true;
    }
  }
}

static void parseGGA(const char* l) {
  int fixq = 0, sats = 0;
  float alt = NAN;
  if (sscanf(l, "%*[^,],%*[^,],%*[^,],%*[^,],%*[^,],%d,%d,%*[^,],%f",
             &fixq, &sats, &alt) == 3) {
    GNSS.fix_quality = fixq;
    GNSS.satellites  = sats;
    GNSS.altitude_m  = alt;
    GNSS.has_fix     = fixq > 0;
  }
}

static void parseGSA(const char* l) {
  int fixType = 0;
  if (sscanf(l, "%*[^,],%*[^,],%d", &fixType) == 1) {
    GNSS.fix_type = fixType;
  }
}

static void extractDiscipline(const char* l) {
  const char* p = strchr(l, ',');
  if (!p) return;
  p++;
  const char* q = strchr(p, ',');
  if (!q) return;
  size_t len = q - p;
  if (len > 0 && len < sizeof(GNSS.discipline_mode)) {
    memcpy(GNSS.discipline_mode, p, len);
    GNSS.discipline_mode[len] = '\0';
    GNSS.has_discipline = true;
  }
}

// ================================================================
// Ingestion
// ================================================================

static void ingestGnssLine(const char* line) {
  if (!line || !*line) return;

  debug_log("GNSS: ", line);

  safeCopy(GNSS.last_sentence, sizeof(GNSS.last_sentence), line);
  GNSS.last_rx_ms = millis();

  // --- CRZ envelope ---
  if (startsWith(line, "$PERDCRZ,")) {
    safeCopy(GNSS.last_crz, sizeof(GNSS.last_crz), line);
    extractDiscipline(line);

    // Look for embedded RMC
    const char* rmc = strstr(line, "$GNRMC,");
    if (rmc) {
      safeCopy(GNSS.last_rmc, sizeof(GNSS.last_rmc), rmc);
      parseRMC(rmc);
    }
    return;
  }

  // --- Plain sentences ---
  if (startsWith(line, "$GNRMC,") || startsWith(line, "$GPRMC,")) {
    safeCopy(GNSS.last_rmc, sizeof(GNSS.last_rmc), line);
    parseRMC(line);
  }
  else if (startsWith(line, "$GNGGA,") || startsWith(line, "$GPGGA,")) {
    safeCopy(GNSS.last_gga, sizeof(GNSS.last_gga), line);
    parseGGA(line);
  }
  else if (startsWith(line, "$GNGSA,")) {
    safeCopy(GNSS.last_gsa, sizeof(GNSS.last_gsa), line);
    parseGSA(line);
  }
  else if (startsWith(line, "$GPZDA,")) {
    // Raw only — do NOT parse
    safeCopy(GNSS.last_zda, sizeof(GNSS.last_zda), line);
  }
  else if (startsWith(line, "$GPGSV,") || startsWith(line, "$GAGSV,")) {
    safeCopy(GNSS.last_gsv, sizeof(GNSS.last_gsv), line);
  }
  else if (startsWith(line, "$PERDCRW,")) safeCopy(GNSS.last_crw, sizeof(GNSS.last_crw), line);
  else if (startsWith(line, "$PERDCRX,")) safeCopy(GNSS.last_crx, sizeof(GNSS.last_crx), line);
}

// ================================================================
// TimePop Poll
// ================================================================

static void gnss_poll_tick(void*) {
  unsigned long start_ms = millis();
  uint32_t bytes = 0;

  while (Serial1.available()) {
    char c = Serial1.read();
    GNSS.last_rx_ms = millis();

    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen) ingestGnssLine(lineBuf);
      lineLen = 0;
    }
    else if (c != '\r' && lineLen < GNSS_LINE_MAX - 1) {
      lineBuf[lineLen++] = c;
    }

    if (++bytes >= GNSS_MAX_BYTES) break;
    if (millis() - start_ms >= GNSS_BUDGET_MS) break;
  }

  timepop_schedule(GNSS_POLL_INTERVAL_MS,
                   TIMEPOP_UNITS_MILLISECONDS,
                   gnss_poll_tick,
                   nullptr,
                   "gnss-poll");
}

// ================================================================
// Lifecycle
// ================================================================

static bool gnss_start(void) {
  Serial1.begin(GNSSDO_BAUD);
  lineLen = 0;
  timepop_schedule(GNSS_POLL_INTERVAL_MS,
                   TIMEPOP_UNITS_MILLISECONDS,
                   gnss_poll_tick,
                   nullptr,
                   "gnss-poll");
  return true;
}

static void gnss_stop(void) {}

// ================================================================
// REPORT
// ================================================================

static String cmd_report(const char*) {
  String p = "{";
  bool first = true;
  auto sep = [&](){ if (!first) p += ","; first = false; };

  if (GNSS.has_time) {
    char tbuf[9];
    snprintf(tbuf, sizeof(tbuf), "%02u:%02u:%02u",
             GNSS.hour, GNSS.minute, GNSS.second);
    sep(); p += "\"time\":\""; p += tbuf; p += "\"";
  }

  if (GNSS.has_date) {
    char dbuf[11];
    snprintf(dbuf, sizeof(dbuf), "%04u-%02u-%02u",
             GNSS.year, GNSS.month, GNSS.day);
    sep(); p += "\"date\":\""; p += dbuf; p += "\"";
  }

  if (!isnan(GNSS.latitude_deg)) {
    sep(); p += "\"latitude_deg\":"; p += GNSS.latitude_deg;
    sep(); p += "\"longitude_deg\":"; p += GNSS.longitude_deg;
  }

  if (!isnan(GNSS.altitude_m)) {
    sep(); p += "\"altitude_m\":"; p += GNSS.altitude_m;
  }

  if (GNSS.has_discipline) {
    sep(); p += "\"discipline\":\""; p += GNSS.discipline_mode; p += "\"";
  }

  sep(); p += "\"raw\":{";
  bool f2=true;
  auto rsep=[&](){ if(!f2)p+=','; f2=false; };
  if(GNSS.last_rmc[0]){rsep();p+="\"rmc\":\""+jsonEscape(GNSS.last_rmc)+"\"";}
  if(GNSS.last_gga[0]){rsep();p+="\"gga\":\""+jsonEscape(GNSS.last_gga)+"\"";}
  if(GNSS.last_gsa[0]){rsep();p+="\"gsa\":\""+jsonEscape(GNSS.last_gsa)+"\"";}
  if(GNSS.last_zda[0]){rsep();p+="\"zda\":\""+jsonEscape(GNSS.last_zda)+"\"";}
  if(GNSS.last_crz[0]){rsep();p+="\"crz\":\""+jsonEscape(GNSS.last_crz)+"\"";}
  p += "}";

  p += "}";
  return p;
}

// ================================================================
// Registration
// ================================================================

static const process_command_entry_t GNSS_COMMANDS[] = {
  { "REPORT", cmd_report },
};

static const process_vtable_t GNSS_PROCESS = {
  .name = "GNSS",
  .start = gnss_start,
  .stop = gnss_stop,
  .query = nullptr,
  .commands = GNSS_COMMANDS,
  .command_count = 1,
};

void process_gnss_register(void) {
  process_register(PROCESS_TYPE_GNSS, &GNSS_PROCESS);
}
