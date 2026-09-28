"""ZPNet Double audit — stream published firmware Welford populations.

Run from the repository root: python -m tests.double_audit CAMPAIGN [limit]
Use --type LANTERN for PHOTONS; the default is TEMPEST / CLOCKS.
Only compact sufficient-state objects are transferred from campaign_detail.
Decimal checks the stored numbers without introducing Python binary rounding.
This checks published consistency, not the C++ operators or raw sample history.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation, localcontext
from typing import Any, Dict, Iterator, List, Sequence


D = Decimal
FIELDS = ("mean", "m2", "stddev", "stderr", "min", "max")
CLOCKS_PATHS = {
    name: "clocks,stats," + name + ",welford"
    for name in ("gnss", "dwt", "vclock", "ocxo1", "ocxo2")
}
CLOCKS_PATHS["pps_witness"] = "clocks,stats,auxiliary_welford,pps_witness"
PHOTONS_PATHS = {
    "lap_time": "photons,stats,lap_time",
    "accepted_cycles": "photons,science,accepted,raw_cycles",
    "accepted_ns": "photons,science,accepted,projected_lap_ns",
    "excluded_cycles": "photons,science,excluded,raw_cycles",
    "excluded_ns": "photons,science,excluded,projected_lap_ns",
}


@dataclass
class Result:
    populations: int
    active: int
    errors: List[str]


def number(value: Any, where: str) -> Decimal:
    # Reject floats: SQL must return text, decoded with parse_float=Decimal.
    if isinstance(value, bool) or not isinstance(value, (int, str, Decimal)):
        raise ValueError(f"{where}: expected a decimal number, got {value!r}")
    try:
        out = D(value)
    except InvalidOperation as exc:
        raise ValueError(f"{where}: invalid decimal {value!r}") from exc
    if not out.is_finite() or abs(out) >= D("1e385"):
        raise ValueError(f"{where}: nonfinite or outside Double range")
    return out


def population(value: Any, kind: str) -> int:
    if not isinstance(value, dict):
        raise ValueError("missing Welford object")
    n = value.get("n")
    if isinstance(n, bool) or not isinstance(n, int) or not 0 <= n < 2**64:
        raise ValueError("n must be an unsigned 64-bit integer")
    v = {key: number(value.get(key), key) for key in FIELDS}
    if n == 0:
        if any(v.values()):
            raise ValueError("empty population has nonzero statistics")
        return n
    if v["m2"] < 0 or v["stddev"] < 0 or v["stderr"] < 0:
        raise ValueError("negative M2, standard deviation, or standard error")
    if not v["min"] <= v["mean"] <= v["max"]:
        raise ValueError("mean is outside min/max")
    if n == 1:
        if v["min"] != v["mean"] or v["mean"] != v["max"]:
            raise ValueError("singleton min/mean/max disagree")
        if any(v[key] for key in ("m2", "stddev", "stderr")):
            raise ValueError("singleton has nonzero dispersion")
        return n

    # CLOCKS publishes M2 at 12 decimal places; PHOTONS uses scientific
    # notation. Allow that quantization BEFORE sqrt, especially near zero.
    # The relative allowance covers 16-digit Double and persisted Pi rounding.
    m2_error = abs(v["m2"]) * D("1e-13")
    if kind == "TEMPEST":
        m2_error += D("5e-13")
    lower = (max(D(0), v["m2"] - m2_error) / (n - 1)).sqrt()
    upper = ((v["m2"] + m2_error) / (n - 1)).sqrt()
    for key, divisor in (("stddev", D(1)), ("stderr", D(n).sqrt())):
        lo, hi = lower / divisor, upper / divisor
        tolerance = D("5e-7") + max(abs(lo), abs(hi)) * D("1e-13")
        if not lo - tolerance <= v[key] <= hi + tolerance:
            raise ValueError(
                f"{key}={v[key]} outside [{lo}, {hi}] +/- {tolerance}"
            )
    return n


def audit_row(row: Dict[str, Any], kind: str) -> Result:
    paths = CLOCKS_PATHS if kind == "TEMPEST" else PHOTONS_PATHS
    expected_schema = "CLOCKS_V4" if kind == "TEMPEST" else "PHOTONS_V1"
    errors: List[str] = []
    if row.get("schema") != expected_schema:
        errors.append(f"schema: expected {expected_schema}, got {row.get('schema')!r}")
    try:
        values = json.loads(row["welfords"], parse_float=D, parse_constant=D)
        if not isinstance(values, dict):
            raise ValueError("projected Welfords must be an object")
    except (ValueError, TypeError, KeyError) as exc:
        return Result(0, 0, errors + [f"payload: {exc}"])
    active = 0
    with localcontext() as ctx:
        ctx.prec = 60
        for name in paths:
            try:
                active += population(values.get(name), kind) > 0
            except (ValueError, InvalidOperation) as exc:
                errors.append(f"{name}: {exc}")
        if kind == "LANTERN" and values.get("lap_time") != values.get("accepted_ns"):
            errors.append("lap_time and accepted_ns disagree (same firmware population)")
    return Result(len(paths), active, errors)


def query(args: argparse.Namespace) -> tuple[str, tuple[Any, ...]]:
    paths = CLOCKS_PATHS if args.type == "TEMPEST" else PHOTONS_PATHS
    # Only fixed, source-owned names/paths are interpolated; user input is bound.
    pairs = ",\n".join(
        f"'{name}', payload #> '{{{path}}}'" for name, path in paths.items()
    )
    sql = f"""
        SELECT id, ts, payload ->> 'schema' AS schema,
               payload #>> '{{campaign,public_count}}' AS public_count,
               jsonb_build_object({pairs})::text AS welfords
        FROM campaign_detail
        WHERE campaign_type = %s AND campaign = %s AND id > %s
        ORDER BY id ASC
    """
    params: List[Any] = [args.type, args.campaign, args.after_id]
    if args.limit:
        sql += " LIMIT %s"
        params.append(args.limit)
    return sql, tuple(params)


def iter_rows(args: argparse.Namespace) -> Iterator[Dict[str, Any]]:
    # Keep offline self-tests and --help independent of psycopg/database setup.
    from zpnet.shared.db import open_db

    sql, params = query(args)
    with open_db(row_dict=True) as conn:
        conn.execute("SET TRANSACTION READ ONLY")
        with conn.cursor(name="double_audit_stream") as cur:
            cur.itersize = args.batch_size
            cur.execute(sql, params)
            yield from cur


def self_test() -> None:
    import copy

    good = {"n": 3, "mean": "2", "m2": "2", "stddev": "1",
            "stderr": "0.577350", "min": "1", "max": "3"}
    checks = 0
    for kind, paths, schema in (
        ("TEMPEST", CLOCKS_PATHS, "CLOCKS_V4"),
        ("LANTERN", PHOTONS_PATHS, "PHOTONS_V1"),
    ):
        def check(values: dict, valid: bool) -> None:
            nonlocal checks
            result = audit_row({"schema": schema, "welfords": json.dumps(values)}, kind)
            if bool(result.errors) == valid:
                raise AssertionError((kind, valid, result))
            checks += 1

        values = {name: dict(good) for name in paths}
        check(values, True)
        first = next(iter(paths))
        for field, bad in (("n", -1), ("n", True), ("n", 3.5),
                           ("mean", "NaN"), ("m2", "Infinity"),
                           ("m2", "-2"), ("stddev", "2"),
                           ("stderr", "0.1"), ("min", "4")):
            broken = copy.deepcopy(values)
            broken[first][field] = bad
            check(broken, False)
        broken = copy.deepcopy(values)
        del broken[first]["m2"]
        check(broken, False)
        broken = copy.deepcopy(values)
        del broken[first]
        check(broken, False)
        for n in (0, 1):
            uniform = {"n": n, **{key: "0" for key in FIELDS}}
            check({name: dict(uniform) for name in paths}, True)
            broken = {name: dict(uniform) for name in paths}
            broken[first]["m2"] = "1"
            check(broken, False)
    # SQL text decoding must retain more precision than a Python float.
    precise = json.loads('{"x": 1000000000.000001}', parse_float=D)
    if precise["x"] != D("1000000000.000001"):
        raise AssertionError("decimal JSON precision lost")
    # Rounded CLOCKS M2=0 can legitimately accompany stddev=0.000001.
    tiny = dict(good, n=2, mean="0.0000005", m2="0",
                min="0", max="0.000001", stddev="0.000001", stderr="0.000001")
    with localcontext() as ctx:
        ctx.prec = 60
        population(tiny, "TEMPEST")
    print(f"PASS: {checks + 2} offline validator cases; no firmware execution")


def main(argv: Sequence[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign", nargs="?")
    parser.add_argument("limit", nargs="?", type=int, default=0,
                        help="maximum rows; 0 scans the campaign")
    parser.add_argument("--type", choices=("TEMPEST", "LANTERN"), default="TEMPEST")
    parser.add_argument("--after-id", type=int, default=0, help="exclusive database row ID")
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--failures-only", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args(argv[1:])
    if args.self_test:
        self_test()
        return 0
    if not args.campaign:
        parser.error("campaign is required unless --self-test is used")
    if args.limit < 0 or args.after_id < 0 or args.batch_size < 1:
        parser.error("limit/after-id must be nonnegative; batch-size must be positive")
    print(f"Double audit | {args.type} | campaign={args.campaign}")
    print(f"{'ROW ID':>12} {'PUBLIC':>12} {'CHECKED':>8} {'ACTIVE':>8} RESULT")
    rows = failed = populations = active = 0
    try:
        for row in iter_rows(args):
            result = audit_row(row, args.type)
            rows += 1
            populations += result.populations
            active += result.active
            failed += bool(result.errors)
            if result.errors or not args.failures_only:
                print(f"{row['id']:>12} {str(row.get('public_count') or '-'):>12} "
                      f"{result.populations:>8} {result.active:>8} "
                      f"{'FAIL' if result.errors else 'PASS'}")
                for error in result.errors:
                    print(f"  {error}")
    except Exception as exc:
        print(f"ERROR after {rows} rows: {exc}", file=sys.stderr)
        return 2
    print(f"Rows={rows} populations={populations} active={active} failed_rows={failed}")
    if rows == 0 or active == 0:
        print("INCONCLUSIVE: no nonempty firmware populations were checked")
        return 1
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
