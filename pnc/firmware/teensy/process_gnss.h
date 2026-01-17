#pragma once

#include "process.h"
#include <Arduino.h>

/**
 * ------------------------------------------------------------------
 * GNSS Process (process_gnss)
 *
 * Owns:
 *   • Serial1 ingestion (CF-8802)
 *   • Sentence buffering
 *   • NMEA parsing
 *   • GNSS-derived observables
 *
 * This process replaces legacy gnss.cpp polling.
 *
 * Behavior:
 *   • Interrupt / TimePop driven
 *   • No loop polling
 *   • Fully introspectable via QUERY
 *
 * ------------------------------------------------------------------
 */

// Register GNSS process with the process registry
void process_gnss_register(void);
