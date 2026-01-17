/**
 * -----------------------------------------------------------------------------
 *  ZPNet Runtime Interface (zpnet.h)
 * -----------------------------------------------------------------------------
 *
 *  This header defines the public interface to the ZPNet production runtime.
 *
 *  ZPNet is an interrupt-driven execution harness designed for deterministic,
 *  traceable, and causality-enforced experimental systems.
 *
 *  ROLE OF THIS FILE:
 *  ------------------
 *  - Acts as the sole public entry point into the ZPNet runtime
 *  - Declares lifecycle hooks without exposing implementation details
 *  - Establishes a strict boundary between Arduino scaffolding and ZPNet logic
 *
 *  ARCHITECTURAL PRINCIPLES:
 *  -------------------------
 *  - All execution is initiated by hardware interrupts or scheduled time pops
 *  - No polling-based dispatch is permitted
 *  - No subsystem may directly invoke another subsystem
 *  - Each subsystem is responsible for scheduling its own next causal step
 *
 *  WHAT THIS FILE IS NOT:
 *  ----------------------
 *  - It does not contain runtime logic
 *  - It does not define scheduling behavior
 *  - It does not expose internal data structures
 *  - It does not include Arduino headers
 *
 *  Those responsibilities belong exclusively to the implementation
 *  in `zpnet.cpp` and associated runtime modules.
 *
 *  STAGED DEPLOYMENT MODEL:
 *  ------------------------
 *  During development, the ZPNet runtime may be compiled and linked
 *  without being connected to the Arduino `setup()` / `loop()` lifecycle.
 *
 *  When ready, the Arduino harness (`zpnet.ino`) will delegate:
 *
 *      setup() -> zpnet_setup()
 *      loop()  -> zpnet_loop()
 *
 *  enabling a controlled and reversible transition to production behavior.
 *
 * -----------------------------------------------------------------------------
 */

#pragma once

// -----------------------------------------------------------------------------
// ZPNet Runtime Lifecycle Interface
// -----------------------------------------------------------------------------

/**
 * Initialize the ZPNet runtime.
 *
 * This function is responsible for:
 *  - Initializing core runtime services (timers, registries, queues)
 *  - Registering Teensy Processes
 *  - Establishing interrupt-driven execution contracts
 *
 * It must not perform any polling, blocking, or opportunistic execution.
 */
void zpnet_setup();

/**
 * Advance the ZPNet runtime.
 *
 * This function is called repeatedly by the Arduino runtime once
 * the system is active. It must remain lightweight and non-blocking.
 *
 * All meaningful work must be driven by interrupt-triggered state
 * changes or scheduled time pops.
 */
void zpnet_loop();
