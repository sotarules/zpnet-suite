#pragma once

#include <Arduino.h>

// --------------------------------------------------------------
// GNSS ingestion and tempo profiling
// --------------------------------------------------------------
//
// Owns:
//   • GNSS serial ingestion
//   • Sentence retention and parsing
//   • VCLOCK (10 MHz) tick counting
//   • Tempo profiling state machine
//
// Does NOT:
//   • Infer lock state
//   • Depend on PPS
//   • Perform aggregation outside its domain
//

// Initialize GNSS subsystem (Serial1, pins, counters)
void gnss_init();

// Opportunistic serial ingestion (call every loop)
void gnss_poll();

// Telemetry builders (JSON body fragments)
String buildGnssStatusBody();
String buildGnssDataBody();

void gnss_diagnostic_poll_loop();

