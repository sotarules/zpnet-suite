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

// Tempo profiling control
void gnss_tempo_start(float altitude_m);
void gnss_tempo_stop();

// Telemetry builders (JSON body fragments)
String buildGnssStatusBody();
String buildGnssDataBody();
