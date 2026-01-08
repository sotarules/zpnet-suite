#pragma once

#include <Arduino.h>

// --------------------------------------------------------------
// GNSS ingestion and timing
// --------------------------------------------------------------
//
// Owns:
//   • GNSS serial ingestion
//   • Sentence retention and parsing
//   • PPS ISR and counters
//   • VCLOCK (10 MHz) sampling
//
// Exposes only builders and polling hooks.
//

// Initialize GNSS subsystem (Serial1, pins, ISRs)
void gnss_init();

// Opportunistic serial ingestion (call every loop)
void gnss_poll();

// Handle PPS capture and rate-limited event emission
void gnss_handle_pps();

// Telemetry builders (JSON body fragments)
String buildGnssStatusBody();
String buildGnssDataBody();
