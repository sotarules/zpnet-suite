"""
apply_ocxo_truthful_names.py

Truthful-naming pass for ZPNet CLOCKS OCXO surfaces.

Run from the zpnet-suite repository root:

    python3 apply_ocxo_truthful_names.py

Intent:
  - Preserve behavior.
  - Rename OCXO nominal/self-ledger surfaces so they no longer look like
    GNSS-measured OCXO elapsed time.
  - Leave bridge-derived measurement names such as gnss_ns_between_edges and
    second_residual_ns intact, because those are the science/servo quantities.

This is deliberately a flat token-replacement script. It is meant to be easy
to audit with git diff.
"""

from __future__ import annotations

from pathlib import Path

FILES = [
    Path("pnc/firmware/teensy/process_clocks_alpha.cpp"),
    Path("pnc/firmware/teensy/process_clocks_beta.cpp"),
    Path("pnc/firmware/teensy/process_clocks_internal.h"),
    Path("pnc/firmware/teensy/process_clocks.h"),
]

REPLACEMENTS = [
    # Alpha-owned OCXO nominal ledgers. These are not GNSS-measured elapsed
    # OCXO-edge spacing; they are nominal/self-ledger coordinates.
    ("g_ocxo1_ns_at_pps_vclock", "g_ocxo1_nominal_ns_at_pps_vclock"),
    ("g_ocxo2_ns_at_pps_vclock", "g_ocxo2_nominal_ns_at_pps_vclock"),

    # Public accessors. Keep "nominal" in the name so call sites do not mistake
    # these for bridge-measured OCXO-vs-GNSS elapsed time.
    ("clocks_ocxo1_ns_now", "clocks_ocxo1_nominal_ns_now"),
    ("clocks_ocxo2_ns_now", "clocks_ocxo2_nominal_ns_now"),
    ("clocks_ocxo1_ticks_now", "clocks_ocxo1_nominal_ticks_now"),
    ("clocks_ocxo2_ticks_now", "clocks_ocxo2_nominal_ticks_now"),

    # Beta-facing shadow totals. These are legacy/public mirror totals, not the
    # bridge-derived per-second residual measurement.
    ("ocxo1_ticks_64", "ocxo1_nominal_ticks_64"),
    ("ocxo2_ticks_64", "ocxo2_nominal_ticks_64"),

    # Shared measured-clock compatibility struct. This is the big truth fix:
    # the field that advances by exactly 1e9 per edge is a nominal self-ledger.
    ("ns_count_at_edge", "nominal_ns_count_at_edge"),
    ("ns_count_at_pps_vclock", "nominal_ns_count_at_pps_vclock"),

    # Forensics: make counter-derived nanoseconds explicit as nominal/counter
    # surfaces, distinct from bridge-derived GNSS elapsed time.
    ("logical_ns64_since_zero", "nominal_ns64_since_zero"),
    ("ns_from_counter32_epoch", "nominal_ns_from_counter32_epoch"),
    ("counter_ns_between_edges", "counter_nominal_ns_between_edges"),
]

COMMENT_REPLACEMENTS = [
    (
        "CLOCKS owns the canonical time ledgers on the Teensy.",
        "CLOCKS owns the Teensy timing ledgers and bridge-measured residual surfaces."
    ),
    (
        "  • Synthetic nanosecond clocks",
        "  • Nominal clock ledgers and bridge-derived measured residuals"
    ),
    (
        "  // Alpha uses this as the DWT span for OCXO measured-ns ledgers.",
        "  // Alpha uses this as the DWT span for OCXO edge projection / residual measurement."
    ),
    (
        "  //                                     nanosecond counter sampled at the same",
        "  //                                     nominal nanosecond self-ledger sampled at the same"
    ),
    (
        "  // Canonical 64-bit nanosecond clocks at the most recent selected PPS/VCLOCK",
        "  // Canonical GNSS and nominal OCXO 64-bit surfaces at the most recent selected PPS/VCLOCK"
    ),
    (
        "  // count sampled at the latest PPS/VCLOCK edge.",
        "  // self-ledger sampled at the latest PPS/VCLOCK edge."
    ),
    (
        "  // Cumulative authored ns count at the edge being applied.  Advances",
        "  // Nominal self-ledger ns count at the edge being applied.  Advances"
    ),
    (
        "  // Compatibility mirror of the canonical Alpha-owned epoch-relative ns",
        "  // Compatibility mirror of the Alpha-owned epoch-relative nominal ns"
    ),
    (
        "  // (gnss_ns_at_edge − ns_count_at_edge).  For VCLOCK this should be",
        "  // (gnss_ns_at_edge − nominal_ns_count_at_edge).  For VCLOCK this should be"
    ),
    (
        "  // These snapshots expose the last event alpha consumed for each measured\n"
        "  // clock, plus the exact epoch-relative counter32->ns arithmetic applied before\n"
        "  // handing the event to process_time's generalized projection model.",
        "  // These snapshots expose the last event alpha consumed for each measured\n"
        "  // clock, plus the exact epoch-relative counter32->nominal-ns arithmetic applied before\n"
        "  // handing the event to process_time's generalized projection model."
    ),
]

def main() -> None:
    changed_files: list[Path] = []

    for path in FILES:
        if not path.exists():
            raise FileNotFoundError(f"Expected file not found: {path}")

        original = path.read_text()
        updated = original

        for old, new in REPLACEMENTS:
            updated = updated.replace(old, new)

        for old, new in COMMENT_REPLACEMENTS:
            updated = updated.replace(old, new)

        if updated != original:
            path.write_text(updated)
            changed_files.append(path)

    if not changed_files:
        print("No changes made; all truthful-naming replacements already appear applied.")
        return

    print("Updated:")
    for path in changed_files:
        print(f"  {path}")

    print()
    print("Next:")
    print("  git diff -- pnc/firmware/teensy/process_clocks_alpha.cpp \\")
    print("             pnc/firmware/teensy/process_clocks_beta.cpp \\")
    print("             pnc/firmware/teensy/process_clocks_internal.h \\")
    print("             pnc/firmware/teensy/process_clocks.h")
    print()
    print("Then build/flash in the usual way.")

if __name__ == "__main__":
    main()
