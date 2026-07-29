"""ZPNet raw_delta — streamed PhaseLedger versus Delta Cycles science audit.

The report reads TIMEBASE in indexed ``timebase.pps_count`` order through a
named PostgreSQL cursor. Each JSON payload is decoded, reduced to the two OCXO
science lanes, printed, and discarded. Memory use is therefore independent of
campaign length.

No JSONB path is used for ordering or filtering, and the SQL deliberately uses
no COALESCE expression. The scalar database identity must agree with the
immutable identity carried inside the payload.

For each OCXO lane the report shows:

    ph_res
        PhaseLedger fast residual for the current adjacent row, derived from
        successive public OCXO and GNSS clockfaces:

            (current_ocxo_ns - previous_ocxo_ns)
              - (current_gnss_ns - previous_gnss_ns)

        Positive means the OCXO clockface advanced fast.

    dc_res
        Delta Cycles same-row science residual from
        ``ocxoN.science.fast_residual_ns_exact``. Positive means the physical
        OCXO second was short, hence the oscillator ran fast.

    ph_ppb
        Campaign-wide PPB implied by all accepted PhaseLedger interval inputs
        seen by this report:

            sum(phase_clock_advance) / sum(gnss_elapsed)

    dc_ppb
        Campaign-wide PPB implied by all accepted Delta Cycles interval inputs
        seen by this report:

            sum(gnss_interval) / sum(delta_clock_duration)

    tb_ppb
        The PPB already published in ``stats.ocxoN.ppb``. In current
        TIMEBASE_FRAGMENT_V5 this is the public clockface ratio and is included
        as an audit reference; it is not used by either report accumulator.

The two accumulated estimators intentionally preserve their own native
semantics. PhaseLedger contributes clockface advance, while Delta Cycles
contributes the measured real-time duration of one OCXO second.
"""

from __future__ import annotations

import json
import sys
import time
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation, localcontext
from typing import Any, Dict, Iterator, Optional, Sequence, Tuple

from zpnet.shared.db import open_db


LANES: Tuple[str, ...] = ("OCXO1", "OCXO2")
LANE_KEYS = {"OCXO1": "ocxo1", "OCXO2": "ocxo2"}
NS_PER_SECOND = Decimal("1000000000")
DEFAULT_BATCH_SIZE = 16
DEFAULT_PAUSE_MS = 25


@dataclass
class LaneAccumulator:
    phase_samples: int = 0
    phase_gnss_total_ns: Decimal = Decimal(0)
    phase_clock_total_ns: Decimal = Decimal(0)

    delta_samples: int = 0
    delta_gnss_total_ns: Decimal = Decimal(0)
    delta_clock_total_ns: Decimal = Decimal(0)

    def add_phase(self, gnss_interval_ns: Decimal, clock_interval_ns: Decimal) -> None:
        if gnss_interval_ns <= 0 or clock_interval_ns <= 0:
            return
        self.phase_samples += 1
        self.phase_gnss_total_ns += gnss_interval_ns
        self.phase_clock_total_ns += clock_interval_ns

    def add_delta(self, gnss_interval_ns: Decimal, clock_interval_ns: Decimal) -> None:
        if gnss_interval_ns <= 0 or clock_interval_ns <= 0:
            return
        self.delta_samples += 1
        self.delta_gnss_total_ns += gnss_interval_ns
        self.delta_clock_total_ns += clock_interval_ns

    def phase_ppb(self) -> Optional[Decimal]:
        if self.phase_gnss_total_ns <= 0:
            return None
        return (
            self.phase_clock_total_ns / self.phase_gnss_total_ns - Decimal(1)
        ) * NS_PER_SECOND

    def delta_ppb(self) -> Optional[Decimal]:
        if self.delta_clock_total_ns <= 0:
            return None
        return (
            self.delta_gnss_total_ns / self.delta_clock_total_ns - Decimal(1)
        ) * NS_PER_SECOND


@dataclass
class LaneRow:
    phase_residual_ns: Optional[Decimal] = None
    delta_residual_ns: Optional[Decimal] = None
    phase_ppb: Optional[Decimal] = None
    delta_ppb: Optional[Decimal] = None
    timebase_ppb: Optional[Decimal] = None
    phase_samples: int = 0
    delta_samples: int = 0
    clockface_valid: Optional[bool] = None
    delta_valid: Optional[bool] = None
    science_worthy: Optional[bool] = None
    antecedents_complete: Optional[bool] = None


@dataclass
class AuditRow:
    count: int
    previous_count: Optional[int]
    count_delta: Optional[int]
    gap: bool
    recovery_boundary: bool
    disposition: str
    use_directive: str
    timeline_valid: Optional[bool]
    lanes: Dict[str, LaneRow]


def d(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def i(value: Any) -> Optional[int]:
    if value is None or isinstance(value, bool):
        return None
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return None


def dec(value: Any) -> Optional[Decimal]:
    if value is None or isinstance(value, bool):
        return None
    try:
        parsed = Decimal(str(value))
    except (InvalidOperation, ValueError, TypeError):
        return None
    return parsed if parsed.is_finite() else None


def b(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "on", "nominal", "ready"}:
            return True
        if normalized in {"0", "false", "no", "off", "hold", "anomaly"}:
            return False
    return None


def first_int(*values: Any) -> Optional[int]:
    for value in values:
        parsed = i(value)
        if parsed is not None:
            return parsed
    return None


def first_decimal(*values: Any) -> Optional[Decimal]:
    for value in values:
        parsed = dec(value)
        if parsed is not None:
            return parsed
    return None


def root(payload: Dict[str, Any]) -> Dict[str, Any]:
    inner = d(payload.get("payload"))
    return inner or payload


def fragment(payload: Dict[str, Any]) -> Dict[str, Any]:
    value = root(payload)
    return d(value.get("fragment")) or value


def _payload_pps_count(payload: Dict[str, Any], frag: Dict[str, Any]) -> Optional[int]:
    value = root(payload)
    return first_int(
        frag.get("teensy_pps_vclock_count"),
        frag.get("pps_count"),
        frag.get("campaign_seconds"),
        value.get("teensy_pps_vclock_count"),
        value.get("pps_count"),
    )


def _assert_campaign_indexed(cur: Any, campaign: str) -> None:
    cur.execute(
        """
        SELECT count(*) AS missing_count
        FROM timebase
        WHERE campaign = %s AND pps_count IS NULL
        """,
        (campaign,),
    )
    row = cur.fetchone()
    missing = int(row["missing_count"] if row else 0)
    if missing:
        raise RuntimeError(
            f"campaign {campaign!r} has {missing:,} rows with NULL "
            "timebase.pps_count; backfill the scalar identity before reporting"
        )


def iter_payloads(
    campaign: str,
    *,
    skip: int,
    limit: int,
    batch_size: int,
    pause_ms: int,
) -> Iterator[Tuple[int, int, Dict[str, Any]]]:
    """Yield ``(db_id, pps_count, payload)`` through a true server-side cursor."""
    with open_db(row_dict=True) as conn:
        conn.execute("SET TRANSACTION READ ONLY")
        check = conn.cursor()
        _assert_campaign_indexed(check, campaign)
        check.close()

        sql = """
            SELECT id, pps_count, payload
            FROM timebase
            WHERE campaign = %s
        """
        params: list[Any] = [campaign]
        if skip > 0:
            sql += " AND pps_count > %s"
            params.append(skip)
        sql += " ORDER BY pps_count ASC, id ASC"
        if limit > 0:
            sql += " LIMIT %s"
            params.append(limit)

        cur = conn.cursor(name="raw_delta_stream")
        cur.itersize = max(1, batch_size)
        cur.execute(sql, tuple(params))

        fetched_in_batch = 0
        for row in cur:
            payload = row["payload"]
            if isinstance(payload, str):
                payload = json.loads(payload)
            if not isinstance(payload, dict):
                continue

            db_count = int(row["pps_count"])
            frag = fragment(payload)
            payload_count = _payload_pps_count(payload, frag)
            if payload_count is not None and payload_count != db_count:
                raise ValueError(
                    "TIMEBASE relational/payload PPS mismatch: "
                    f"id={row['id']} db={db_count} payload={payload_count}"
                )

            yield int(row["id"]), db_count, payload

            fetched_in_batch += 1
            if fetched_in_batch >= max(1, batch_size):
                fetched_in_batch = 0
                if pause_ms > 0:
                    time.sleep(pause_ms / 1000.0)


def explicit_recovery_row(frag: Dict[str, Any]) -> bool:
    reason = str(frag.get("recover_reattach_reason") or "").strip().lower()
    return bool(
        reason not in {"", "idle", "none", "ok"}
        or b(frag.get("recover_degraded_active")) is True
        or b(frag.get("recover_degraded_science_hold")) is True
        or b(frag.get("recover_science_quarantine_active")) is True
        or b(frag.get("recover_transition_active")) is True
    )


def published_ppb(frag: Dict[str, Any], lane_key: str) -> Optional[Decimal]:
    stats_lane = d(d(frag.get("stats")).get(lane_key))
    direct = first_decimal(stats_lane.get("ppb"))
    if direct is not None:
        return direct

    # Older payloads sometimes nested frequency fields or carried the total in
    # the science object. These are compatibility fallbacks only.
    frequency = d(stats_lane.get("frequency"))
    direct = first_decimal(frequency.get("ppb"))
    if direct is not None:
        return direct

    science = d(d(frag.get(lane_key)).get("science"))
    direct = first_decimal(science.get("total_ppb"))
    if direct is not None:
        return direct

    # Final fallback: reproduce the public clockface ratio directly.
    gnss_ns = first_decimal(d(frag.get("gnss")).get("ns"))
    clock_ns = first_decimal(d(frag.get(lane_key)).get("ns"))
    if gnss_ns is None or clock_ns is None or gnss_ns <= 0:
        return None
    return (clock_ns / gnss_ns - Decimal(1)) * NS_PER_SECOND


def _delta_interval_from_science(
    science: Dict[str, Any],
) -> Tuple[Optional[Decimal], Optional[Decimal], Optional[Decimal]]:
    """Return ``(fast_residual, gnss_interval, clock_duration)``."""
    residual = first_decimal(
        science.get("fast_residual_ns_exact"),
        science.get("fast_residual_ns"),
        science.get("delta_raw_fast_residual_ns_exact"),
        science.get("delta_raw_fast_residual_ns"),
    )
    gnss_interval = first_decimal(
        science.get("gnss_interval_ns_exact"),
        science.get("gnss_interval_ns"),
    )
    clock_interval = first_decimal(
        science.get("clock_interval_ns_exact"),
        science.get("clock_interval_ns"),
    )

    if gnss_interval is None:
        gnss_interval = NS_PER_SECOND
    if clock_interval is None and residual is not None:
        # Delta fast residual is positive when the physical OCXO second is
        # shorter than the GNSS second.
        clock_interval = gnss_interval - residual
    if residual is None and clock_interval is not None:
        residual = gnss_interval - clock_interval

    return residual, gnss_interval, clock_interval


def _phase_interval_from_clockfaces(
    frag: Dict[str, Any],
    lane_key: str,
    previous_gnss_ns: Optional[Decimal],
    previous_clock_ns: Optional[Decimal],
    adjacent: bool,
) -> Tuple[
    Optional[Decimal],
    Optional[Decimal],
    Optional[Decimal],
    Optional[Decimal],
    Optional[Decimal],
]:
    """Return residual/intervals plus current clockface values.

    The result is ``(fast_residual, gnss_interval, clock_advance,
    current_gnss_ns, current_clock_ns)``.
    """
    current_gnss_ns = first_decimal(d(frag.get("gnss")).get("ns"))
    lane = d(frag.get(lane_key))
    current_clock_ns = first_decimal(lane.get("ns"))

    if (
        not adjacent
        or previous_gnss_ns is None
        or previous_clock_ns is None
        or current_gnss_ns is None
        or current_clock_ns is None
    ):
        return None, None, None, current_gnss_ns, current_clock_ns

    gnss_interval = current_gnss_ns - previous_gnss_ns
    clock_advance = current_clock_ns - previous_clock_ns
    if gnss_interval <= 0 or clock_advance <= 0:
        return None, None, None, current_gnss_ns, current_clock_ns

    return (
        clock_advance - gnss_interval,
        gnss_interval,
        clock_advance,
        current_gnss_ns,
        current_clock_ns,
    )


def build_row(
    payload: Dict[str, Any],
    db_count: int,
    previous_count: Optional[int],
    previous_gnss_ns: Optional[Decimal],
    previous_clock_ns: Dict[str, Optional[Decimal]],
    accumulators: Dict[str, LaneAccumulator],
) -> Tuple[AuditRow, Optional[Decimal]]:
    frag = fragment(payload)
    delta = None if previous_count is None else db_count - previous_count
    gap = delta is not None and delta != 1
    recovery = bool(gap and delta is not None and delta > 1 and explicit_recovery_row(frag))

    disposition = str(frag.get("candidate_disposition") or "ACCEPT").upper()
    use_directive = str(
        frag.get("candidate_use_directive")
        or frag.get("use_directive")
        or "CANONICAL_CANDIDATE"
    ).upper()
    timeline_valid = b(frag.get("timeline_valid"))

    row_permitted = (
        disposition == "ACCEPT"
        and use_directive not in {"DO_NOT_USE", "REJECT"}
        and timeline_valid is not False
        and not explicit_recovery_row(frag)
    )
    adjacent = delta == 1 and row_permitted

    current_gnss_ns = first_decimal(d(frag.get("gnss")).get("ns"))
    lanes: Dict[str, LaneRow] = {}

    for lane_name in LANES:
        lane_key = LANE_KEYS[lane_name]
        lane = d(frag.get(lane_key))
        science = d(lane.get("science"))

        (
            phase_residual,
            phase_gnss_interval,
            phase_clock_interval,
            observed_gnss_ns,
            observed_clock_ns,
        ) = _phase_interval_from_clockfaces(
            frag,
            lane_key,
            previous_gnss_ns,
            previous_clock_ns[lane_name],
            adjacent,
        )

        clockface_valid = b(lane.get("clockface_valid"))
        delta_valid = b(science.get("delta_raw_valid"))
        if delta_valid is None:
            delta_valid = b(science.get("valid"))
        science_worthy = b(science.get("science_worthy"))
        antecedents_complete = b(science.get("antecedents_complete"))

        phase_usable = (
            row_permitted
            and adjacent
            and clockface_valid is not False
            and phase_residual is not None
            and phase_gnss_interval is not None
            and phase_clock_interval is not None
        )
        if phase_usable:
            accumulators[lane_name].add_phase(
                phase_gnss_interval, phase_clock_interval
            )

        (
            delta_residual,
            delta_gnss_interval,
            delta_clock_interval,
        ) = _delta_interval_from_science(science)

        delta_usable = (
            row_permitted
            and delta_valid is True
            and science_worthy is not False
            and antecedents_complete is not False
            and delta_residual is not None
            and delta_gnss_interval is not None
            and delta_clock_interval is not None
        )
        if delta_usable:
            accumulators[lane_name].add_delta(
                delta_gnss_interval, delta_clock_interval
            )

        lanes[lane_name] = LaneRow(
            phase_residual_ns=phase_residual if phase_usable else None,
            delta_residual_ns=delta_residual if delta_usable else None,
            phase_ppb=accumulators[lane_name].phase_ppb(),
            delta_ppb=accumulators[lane_name].delta_ppb(),
            timebase_ppb=published_ppb(frag, lane_key),
            phase_samples=accumulators[lane_name].phase_samples,
            delta_samples=accumulators[lane_name].delta_samples,
            clockface_valid=clockface_valid,
            delta_valid=delta_valid,
            science_worthy=science_worthy,
            antecedents_complete=antecedents_complete,
        )

        # Always advance the observation baseline. A later row is considered a
        # usable PhaseLedger interval only when the scalar PPS identities are
        # adjacent and the row passes the gates above.
        if observed_clock_ns is not None:
            previous_clock_ns[lane_name] = observed_clock_ns
        elif d(frag.get(lane_key)).get("ns") is None:
            previous_clock_ns[lane_name] = None

        if observed_gnss_ns is not None:
            current_gnss_ns = observed_gnss_ns

    return (
        AuditRow(
            count=db_count,
            previous_count=previous_count,
            count_delta=delta,
            gap=gap,
            recovery_boundary=recovery,
            disposition=disposition,
            use_directive=use_directive,
            timeline_valid=timeline_valid,
            lanes=lanes,
        ),
        current_gnss_ns,
    )


def row_note(row: AuditRow, selected: Sequence[str]) -> str:
    if row.recovery_boundary:
        skipped = (row.count_delta - 1) if row.count_delta is not None else 0
        return f"RECOVER GAP ({skipped} skipped)"
    if row.gap:
        return f"NON-ADJACENT GAP delta={row.count_delta}"

    notes: list[str] = []
    if row.disposition != "ACCEPT":
        notes.append(row.disposition)
    if row.use_directive in {"DO_NOT_USE", "REJECT"}:
        notes.append(row.use_directive)
    if row.timeline_valid is False:
        notes.append("TIMELINE_INVALID")

    for name in selected:
        lane = row.lanes[name]
        if lane.phase_residual_ns is None:
            notes.append(f"{name}:PHASE_SEED_OR_INVALID")
        if lane.delta_residual_ns is None:
            if lane.delta_valid is False:
                notes.append(f"{name}:DELTA_INVALID")
            elif lane.science_worthy is False:
                notes.append(f"{name}:NOT_SCIENCE_WORTHY")
            elif lane.antecedents_complete is False:
                notes.append(f"{name}:ANTECEDENTS_INCOMPLETE")
            else:
                notes.append(f"{name}:DELTA_MISSING")

    return " | ".join(notes) if notes else "OK"


def fmt_decimal(
    value: Optional[Decimal],
    width: int,
    *,
    places: int,
    signed: bool = False,
) -> str:
    if value is None:
        text = "---"
    else:
        with localcontext() as ctx:
            ctx.prec = 32
            if signed:
                text = f"{value:+,.{places}f}"
            else:
                text = f"{value:,.{places}f}"
    return f"{text:>{width}}"


def fmt_int(value: int, width: int) -> str:
    return f"{value:>{width},d}"


def parse(
    argv: Sequence[str],
) -> Tuple[str, int, int, Optional[str], int, int]:
    if len(argv) < 2:
        raise SystemExit(
            "Usage: raw_delta CAMPAIGN [limit] [clock] [--skip N] "
            "[--batch-size N] [--pause-ms N]"
        )

    campaign = argv[1]
    limit = 0
    skip = 0
    clock: Optional[str] = None
    batch_size = DEFAULT_BATCH_SIZE
    pause_ms = DEFAULT_PAUSE_MS
    positional: list[str] = []

    idx = 2
    while idx < len(argv):
        arg = argv[idx]
        if arg in {"--skip", "--clock", "--limit", "--batch-size", "--pause-ms"}:
            if idx + 1 >= len(argv):
                raise SystemExit(f"{arg} requires a value")
            value = argv[idx + 1]
            idx += 2
            if arg == "--skip":
                skip = int(value)
            elif arg == "--clock":
                clock = value.upper()
            elif arg == "--limit":
                limit = int(value)
            elif arg == "--batch-size":
                batch_size = int(value)
            else:
                pause_ms = int(value)
        elif "=" in arg and arg.startswith("--"):
            key, value = arg.split("=", 1)
            if key == "--skip":
                skip = int(value)
            elif key == "--clock":
                clock = value.upper()
            elif key == "--limit":
                limit = int(value)
            elif key == "--batch-size":
                batch_size = int(value)
            elif key == "--pause-ms":
                pause_ms = int(value)
            else:
                raise SystemExit(f"unknown option {key}")
            idx += 1
        else:
            positional.append(arg)
            idx += 1

    for arg in positional:
        if arg.upper() in LANES:
            clock = arg.upper()
        else:
            limit = int(arg)

    if min(skip, limit, pause_ms) < 0 or batch_size <= 0:
        raise SystemExit(
            "skip/limit/pause must be nonnegative; batch-size must be positive"
        )
    if clock is not None and clock not in LANES:
        raise SystemExit("clock must be OCXO1 or OCXO2")

    return campaign, limit, skip, clock, batch_size, pause_ms


def main(argv: Sequence[str]) -> None:
    campaign, limit, skip, clock, batch_size, pause_ms = parse(argv)
    selected = (clock,) if clock else LANES

    print(
        f"Campaign: {campaign}  view={clock or 'BOTH'}  "
        f"mode=CAMPAIGN_WIDE server_batch={batch_size} pause_ms={pause_ms}"
    )
    print(
        "Residuals are ns/second (positive = fast). "
        "ph_ppb and dc_ppb are independently accumulated by this report; "
        "tb_ppb is the published TIMEBASE value."
    )

    header = [f"{'pps':>7}"]
    for name in selected:
        prefix = "o1" if name == "OCXO1" else "o2"
        header += [
            f"{prefix + '_ph_res':>12}",
            f"{prefix + '_dc_res':>12}",
            f"{prefix + '_ph_ppb':>12}",
            f"{prefix + '_dc_ppb':>12}",
            f"{prefix + '_tb_ppb':>12}",
        ]
    header.append("NOTE")
    print("  ".join(header))
    print("  ".join("─" * len(field) for field in header))

    previous_count: Optional[int] = None
    previous_gnss_ns: Optional[Decimal] = None
    previous_clock_ns: Dict[str, Optional[Decimal]] = {
        lane: None for lane in LANES
    }
    accumulators = {lane: LaneAccumulator() for lane in LANES}

    processed = 0
    displayed = 0

    for _db_id, db_count, payload in iter_payloads(
        campaign,
        skip=skip,
        limit=limit,
        batch_size=batch_size,
        pause_ms=pause_ms,
    ):
        row, current_gnss_ns = build_row(
            payload,
            db_count,
            previous_count,
            previous_gnss_ns,
            previous_clock_ns,
            accumulators,
        )
        previous_count = db_count
        if current_gnss_ns is not None:
            previous_gnss_ns = current_gnss_ns
        processed += 1

        fields = [fmt_int(row.count, 7)]
        for name in selected:
            lane = row.lanes[name]
            fields += [
                fmt_decimal(
                    lane.phase_residual_ns, 12, places=3, signed=True
                ),
                fmt_decimal(
                    lane.delta_residual_ns, 12, places=3, signed=True
                ),
                fmt_decimal(lane.phase_ppb, 12, places=3, signed=True),
                fmt_decimal(lane.delta_ppb, 12, places=3, signed=True),
                fmt_decimal(lane.timebase_ppb, 12, places=3, signed=True),
            ]
        fields.append(row_note(row, selected))
        print("  ".join(fields))
        displayed += 1

    print(f"\nProcessed {processed:,} rows; displayed {displayed:,}.")
    for name in selected:
        acc = accumulators[name]
        print(
            f"{name}: phase_samples={acc.phase_samples:,} "
            f"delta_samples={acc.delta_samples:,} "
            f"phase_ppb={fmt_decimal(acc.phase_ppb(), 0, places=6, signed=True).strip()} "
            f"delta_ppb={fmt_decimal(acc.delta_ppb(), 0, places=6, signed=True).strip()}"
        )


if __name__ == "__main__":
    main(sys.argv)
