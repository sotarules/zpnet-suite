"""
ZPNet Timebase — Singleton time interpolation from TIMEBASE stream.

Usage:
    from zpnet.shared.timebase import timebase

    # PUBSUB handler sets the foundation each second:
    def on_timebase(payload):
        timebase.set(payload)

    # Experiment reads current time in any clock domain:
    gnss_ns = timebase.now("GNSS", pi_cycle_count)
    dwt_ns  = timebase.now("DWT",  pi_cycle_count)
    ocxo_ns = timebase.now("OCXO", pi_cycle_count)
    pi_ns   = timebase.now("PI",   pi_cycle_count)

Contract:
    • Timebase is a singleton — one instance per process.
    • set() is called by the PUBSUB handler thread (writer).
    • now() is called by experiment threads (readers).
    • Thread safety via snapshot-under-lock: the lock is held only
      long enough to copy the foundation, then math happens outside.
    • The caller supplies the Pi cycle count (CNTVCT_EL0 / corrected).
      This is intentional — experiments may want to interpolate at a
      captured cycle count from the recent past, not just "right now".
    • now() returns None if the foundation is not yet established
      (need two consecutive TIMEBASE records to compute a delta).
    • All arithmetic is integer where possible.  The only float is
      the interpolation ratio, which is unavoidable.

Interpolation:
    Given a TIMEBASE foundation (values at PPS edge N):
        domain_ns_at_edge = the clock domain's ns value
        pi_cycles_at_edge = pi_corrected at that edge

    And the prior edge (N-1):
        domain_ns_per_pps = domain_ns_at_edge - domain_ns_at_prior_edge
        pi_cycles_per_pps = pi_cycles_at_edge - pi_cycles_at_prior_edge

    Then for a given pi_cycle_count:
        elapsed_cycles = pi_cycle_count - pi_cycles_at_edge
        ratio = domain_ns_per_pps / pi_cycles_per_pps
        now_ns = domain_ns_at_edge + int(elapsed_cycles * ratio)

    This gives nanosecond-resolution time in any clock domain,
    interpolated between PPS edges using the Pi's free-running counter
    as the interpolation ruler.
"""

from __future__ import annotations

import threading
from typing import Dict, Any, Optional


# ---------------------------------------------------------------------
# Clock domain registry
# ---------------------------------------------------------------------

# Maps domain name -> TIMEBASE payload key for the ns value
_DOMAIN_NS_KEY: Dict[str, str] = {
    "GNSS": "teensy_gnss_ns",
    "DWT":  "teensy_dwt_ns",
    "OCXO": "teensy_ocxo_ns",
    "PI":   "pi_ns",
}

# The Pi cycle count key in TIMEBASE (corrected CNTVCT_EL0)
_PI_CYCLES_KEY = "pi_corrected"

# Valid domain names for error messages
_VALID_DOMAINS = tuple(_DOMAIN_NS_KEY.keys())


# ---------------------------------------------------------------------
# Foundation snapshot (immutable once created)
# ---------------------------------------------------------------------

class _Foundation:
    """
    Immutable snapshot of two consecutive TIMEBASE records,
    providing everything needed for interpolation.
    """

    __slots__ = (
        "pps_count",
        "pi_cycles",
        "pi_cycles_per_pps",
        "domain_ns",
        "domain_ns_per_pps",
    )

    def __init__(
        self,
        pps_count: int,
        pi_cycles: int,
        pi_cycles_per_pps: int,
        domain_ns: Dict[str, int],
        domain_ns_per_pps: Dict[str, int],
    ) -> None:
        self.pps_count = pps_count
        self.pi_cycles = pi_cycles
        self.pi_cycles_per_pps = pi_cycles_per_pps
        self.domain_ns = domain_ns
        self.domain_ns_per_pps = domain_ns_per_pps


# ---------------------------------------------------------------------
# Timebase singleton
# ---------------------------------------------------------------------

class Timebase:
    """
    Singleton time interpolation engine.

    Thread-safe: set() and now() can be called from different threads.
    The lock is held only for the brief snapshot copy.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._foundation: Optional[_Foundation] = None
        self._prev_pi_cycles: Optional[int] = None
        self._prev_domain_ns: Optional[Dict[str, int]] = None
        self._pps_count: Optional[int] = None
        self._records_received: int = 0

    def set(self, payload: Dict[str, Any]) -> None:
        """
        Ingest a TIMEBASE record.  Called by the PUBSUB handler.

        The first record establishes the "prior" edge.  The second
        record (and every subsequent one) builds a complete foundation
        that enables interpolation.
        """
        # Extract Pi cycle count at this edge
        pi_cycles_raw = payload.get(_PI_CYCLES_KEY)
        if pi_cycles_raw is None:
            return
        pi_cycles = int(pi_cycles_raw)

        pps_count_raw = payload.get("pps_count")
        if pps_count_raw is None:
            return
        pps_count = int(pps_count_raw)

        # Extract all domain ns values
        current_domain_ns: Dict[str, int] = {}
        for domain, key in _DOMAIN_NS_KEY.items():
            val = payload.get(key)
            if val is not None:
                current_domain_ns[domain] = int(val)

        with self._lock:
            self._records_received += 1

            if self._prev_pi_cycles is not None and self._prev_domain_ns is not None:
                # We have a prior edge — compute deltas and build foundation
                pi_cycles_per_pps = pi_cycles - self._prev_pi_cycles

                if pi_cycles_per_pps > 0:
                    domain_ns_per_pps: Dict[str, int] = {}
                    for domain, ns_now in current_domain_ns.items():
                        ns_prev = self._prev_domain_ns.get(domain)
                        if ns_prev is not None:
                            domain_ns_per_pps[domain] = ns_now - ns_prev

                    self._foundation = _Foundation(
                        pps_count=pps_count,
                        pi_cycles=pi_cycles,
                        pi_cycles_per_pps=pi_cycles_per_pps,
                        domain_ns=dict(current_domain_ns),
                        domain_ns_per_pps=domain_ns_per_pps,
                    )

            # Current becomes prior for next edge
            self._prev_pi_cycles = pi_cycles
            self._prev_domain_ns = dict(current_domain_ns)
            self._pps_count = pps_count

    def now(self, domain: str, pi_cycle_count: int) -> Optional[int]:
        """
        Interpolate current time in the given clock domain.

        Args:
            domain: One of "GNSS", "DWT", "OCXO", "PI"
            pi_cycle_count: Current (or captured) Pi CNTVCT_EL0 value
                            (corrected, matching pi_corrected in TIMEBASE)

        Returns:
            Nanoseconds in the requested clock domain, or None if the
            foundation is not yet established.

        Raises:
            ValueError: If domain is not recognized.
        """
        if domain not in _DOMAIN_NS_KEY:
            raise ValueError(
                f"Unknown clock domain '{domain}'. "
                f"Valid domains: {_VALID_DOMAINS}"
            )

        # Snapshot foundation under lock
        with self._lock:
            f = self._foundation

        if f is None:
            return None

        # Domain must be present in this foundation
        domain_ns_at_edge = f.domain_ns.get(domain)
        domain_delta = f.domain_ns_per_pps.get(domain)
        if domain_ns_at_edge is None or domain_delta is None:
            return None

        # Interpolate
        elapsed_cycles = pi_cycle_count - f.pi_cycles
        # Integer math with a single float multiply for the ratio
        ratio = domain_delta / f.pi_cycles_per_pps
        return domain_ns_at_edge + int(elapsed_cycles * ratio)

    def age_cycles(self, pi_cycle_count: int) -> Optional[int]:
        """
        How many Pi cycles since the last TIMEBASE edge?

        Useful for experiments to gauge staleness of the foundation.
        Returns None if no foundation exists.
        """
        with self._lock:
            f = self._foundation

        if f is None:
            return None

        return pi_cycle_count - f.pi_cycles

    def age_ns(self, pi_cycle_count: int) -> Optional[int]:
        """
        How many nanoseconds since the last TIMEBASE edge?

        Convenience wrapper around age_cycles using the Pi domain rate.
        Returns None if no foundation exists.
        """
        with self._lock:
            f = self._foundation

        if f is None:
            return None

        elapsed_cycles = pi_cycle_count - f.pi_cycles
        pi_delta = f.domain_ns_per_pps.get("PI")
        if pi_delta is None or f.pi_cycles_per_pps == 0:
            return None

        ratio = pi_delta / f.pi_cycles_per_pps
        return int(elapsed_cycles * ratio)

    @property
    def ready(self) -> bool:
        """True if the foundation is established and now() will return values."""
        with self._lock:
            return self._foundation is not None

    @property
    def pps_count(self) -> Optional[int]:
        """The pps_count of the current foundation edge, or None."""
        with self._lock:
            f = self._foundation
        return f.pps_count if f is not None else None

    @property
    def records_received(self) -> int:
        """Total TIMEBASE records ingested (diagnostic)."""
        with self._lock:
            return self._records_received

    def to_dict(self) -> Dict[str, Any]:
        """Diagnostic snapshot of current state."""
        with self._lock:
            f = self._foundation
            return {
                "ready": f is not None,
                "records_received": self._records_received,
                "pps_count": f.pps_count if f else None,
                "pi_cycles_at_edge": f.pi_cycles if f else None,
                "pi_cycles_per_pps": f.pi_cycles_per_pps if f else None,
                "domain_ns": dict(f.domain_ns) if f else None,
                "domain_ns_per_pps": dict(f.domain_ns_per_pps) if f else None,
            }


# ---------------------------------------------------------------------
# Module-level singleton
# ---------------------------------------------------------------------

timebase = Timebase()