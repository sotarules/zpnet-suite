#pragma once

#include <stdint.h>
#include "process.h"

/**
 * ============================================================================
 * PERFORMANCE Process — System Performance Observatory
 * ============================================================================
 *
 * Role:
 *   • Owns performance-related instrumentation and introspection
 *   • Provides authoritative, query-driven visibility into system behavior
 *   • Does NOT publish, infer, or enforce policy
 *
 * Initial Instrument:
 *   • Transport RX opportunity starvation histogram
 *
 * Philosophy:
 *   • Measure availability, not traffic
 *   • Record quietly, report explicitly
 *   • No background emission
 *   • No architectural side effects
 *
 * ============================================================================
 */

// -----------------------------------------------------------------------------
// RX instrumentation hook
// -----------------------------------------------------------------------------
//
// This MUST be called at a point where RX opportunity is known to exist.
// Intended placement:
//   • Bottom of transport RX polling loop
//   • Or immediately after RX polling completes
//
// Semantics:
//   • Records the time since the last RX opportunity
//   • Updates starvation histogram
//
void transport_rx_entered(void);

// -----------------------------------------------------------------------------
// Process registration
// -----------------------------------------------------------------------------

void process_performance_register(void);
