#pragma once

#include "process.h"
#include <Arduino.h>

/**
 * ------------------------------------------------------------------
 * GNSS Process (process_gnss)
 *
 * Role:
 *   • Owns GNSS ingestion and internal state
 *   • Exposes authoritative GNSS state via commands
 *
 * Contract:
 *   • Commands are declared explicitly
 *   • Every command returns a CommandResponse JSON object
 *   • No command/query syntactic distinction
 *
 * Commands:
 *   • STATUS — return current GNSS status snapshot
 *   • DATA   — return current GNSS data snapshot
 *
 * ------------------------------------------------------------------
 */

// Register GNSS process with the process registry
void process_gnss_register(void);
