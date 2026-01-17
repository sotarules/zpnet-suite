#include "debug.h"
#include <Arduino.h>

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------
//
// NOTE:
//   This is a *hardware UART* (Serial2), NOT USB CDC.
//   There is no concept of host attachment or readiness.
//   Once begin() is called, TX is electrically live.
//

static constexpr uint32_t DEBUG_BAUD = 115200;

// Small settle delay after UART enable (defensive, cheap)
static constexpr uint32_t DEBUG_SETTLE_MS = 10;

// -----------------------------------------------------------------------------
// Initialization
// -----------------------------------------------------------------------------

void debug_init() {
    // Enable UART peripheral and drive TX pin
    Serial2.begin(DEBUG_BAUD);

    // Allow hardware to settle (no host semantics involved)
    delay(DEBUG_SETTLE_MS);

    // Emit a visible, unconditional banner
    Serial2.println();
    Serial2.println("=== ZPNet Serial Debug Online ===");
}

// -----------------------------------------------------------------------------
// Logging
// -----------------------------------------------------------------------------

void debug_log(const char* name, const char* body) {
    // IMPORTANT:
    //   Do NOT test `if (!Serial2)`.
    //   UARTs are always electrically present once begun.

    Serial2.print("[");
    Serial2.print(millis());
    Serial2.print("] ");

    if (name && *name) {
        Serial2.print(name);
        Serial2.print(": ");
    }

    if (body && *body) {
        Serial2.print(body);
    }

    Serial2.println();
}
