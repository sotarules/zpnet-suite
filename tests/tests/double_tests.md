# Python tests for the Double migration

Apply this incremental patch from the repository root. It adds only files under
root `tests/`. Delete the previous firmware-local tests directory separately as
planned; these tests have no dependency on that directory or any C++ test runner.

Run with the repository's Python environment, from the repository root:

```sh
python -m tests.double_source_audit
python -m tests.double_audit MY_TEMPEST_CAMPAIGN 1000
python -m tests.double_audit MY_LANTERN_CAMPAIGN 1000 --type LANTERN
```

The campaign audit follows the supplied `raw_cycles` report pattern: `main(argv)`,
the shared `open_db(row_dict=True)` connection, a read-only transaction, a named
server cursor, compact JSON projections, per-row findings and a final summary.
Rows are ordered by database ID; PUBLIC is informational, not an adjacency proof.
It selects the exact campaign and type. Missing/malformed populations fail.
An empty selection or exclusively empty populations is inconclusive and exits 1.
Limit 0 (the default) scans all selected rows. `--after-id N` resumes after a
database ID; `--batch-size N` controls cursor batches; `--failures-only` reduces
output. Database access uses the same configuration/dependencies as existing tests.
Exit codes: 0 means checks passed, 1 means failures/insufficient evidence, and
2 means an invocation or database error.

The campaign audit checks actual persisted CLOCKS and PHOTONS Welford objects:
unsigned counts, finite decimal fields, empty/singleton rules, mean within
extrema, nonnegative dispersion, and standard deviation/error against M2 and N.
PHOTONS' two published views of accepted lap time must also agree. It uses
60-digit Python Decimal calculations and permits the firmware's publication
rounding (including CLOCKS' 12-place M2 and both instruments' 6-place standard
deviation/error), plus a small relative allowance of 1e-13. JSON is fetched as
text to avoid introducing binary floats in this audit. Precision already lost
upstream in Pi processing or storage cannot be recovered.

These are published-state consistency checks. They do not execute Double's C++
operators, prove the original sample populations, test recurrence across recovery
boundaries, or prove that the stored rows came from a particular firmware build.
Use campaigns collected with the new firmware to assess this change. With CLOCKS
and PHOTONS disabled there may be no active scientific populations to audit.

The source audit is a lexical regression guard for application sources, allowing
Double `_D` literals and the existing deleted native-float Payload overloads.
It does not expand macros, inspect vendor libraries, or inspect generated machine
code. It requires only Python's standard library.

Offline tests of the Python validators (no database or compiler):

```sh
python -m tests.double_source_audit --self-test
python -m tests.double_audit --self-test
```

If the earlier patch added this line to the Teensy makefile:

```make
@python3 ../../../tests/decimal/audit.py
```

replace it with `@python3 ../../../tests/double_source_audit.py`, or remove it if
tests should run independently. That firmware edit is deliberately outside this
tests-only patch. No campaign/database audit is required during firmware builds.
