/**
 * -----------------------------------------------------------------------------
 *  ZPNet Arduino Entry Point (teensy.ino)
 * -----------------------------------------------------------------------------
 *
 *  This file provides the Arduino-required entry points:
 *
 *    - setup()
 *    - loop()
 *
 *  IMPORTANT:
 *  ----------
 *  This file contains *no logic*.
 *
 *  It exists solely to delegate execution authority to the ZPNet runtime
 *  implemented in zpnet.cpp.
 *
 *  Architectural rules:
 *   - All initialization occurs in zpnet_setup()
 *   - All steady-state execution occurs in zpnet_loop()
 *   - No work is permitted here
 *   - No abstractions are introduced here
 *
 *  Any behavior added to this file is an architectural violation.
 *
 * -----------------------------------------------------------------------------
 */

#include "zpnet.h"

// -----------------------------------------------------------------------------
// Arduino entry points
// -----------------------------------------------------------------------------

void setup() {
  zpnet_setup();
}

void loop() {
  zpnet_loop();
}

