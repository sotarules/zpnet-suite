"""ZPNet DAC/OCXO population ancestry audit.

Stream canonical CLOCKS_V4 rows from campaign_detail and report only changes in
DAC-vs-OCXO Welford population ancestry.  The purpose is forensic: locate the
exact durable row where a DAC population became inconsistent with its
corresponding OCXO population.

Rows are ordered by PostgreSQL id because durable insertion chronology is the
recovery-lineage authority.  GNSS UTC is displayed only as scientific witness
context.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Iterator, Optional, Sequence

from zpnet.shared.db import open_db

CAMPAIGN_TYPE = "TEMPEST"
CLOCKS_SCHEMA = "CLOCKS_V4"
DEFAULT_BATCH_SIZE = 256
PROGRESS_EVERY = 100_000
GNSS_UTC_SQL = """
COALESCE(
    NULLIF(payload #>> '{clocks,gnss_time_utc}', ''),
    NULLIF(payload #>> '{gnss,gnss_time_utc}', '')
)
"""


@dataclass(frozen=True)
class PopulationRow:
    db_id: int
    gnss_utc: Optional[str]
    sequence: Optional[int]
    campaign: Optional[str]
    reset_count: Optional[int]
    update_count: Optional[int]
    ocxo1_n: Optional[int]
    dac1_n: Optional[int]
    ocxo2_n: Optional[int]
    dac2_n: Optional[int]
    superseded: bool
    recovery_classification: Optional[str]

    @property
    def delta1(self) -> Optional[int]:
        if self.ocxo1_n is None or self.dac1_n is None:
            return None
        return self.dac1_n - self.ocxo1_n

    @property
    def delta2(self) -> Optional[int]:
        if self.ocxo2_n is None or self.dac2_n is None:
            return None
        return self.dac2_n - self.ocxo2_n


def optional_int(value: Any, *, db_id: int, field: str) -> Optional[int]:
    if value is None:
        return None
    if isinstance(value, bool):
        raise ValueError(f"campaign_detail id={db_id} has invalid {field}: {value!r}")
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"campaign_detail id={db_id} has invalid {field}: {value!r}"
        ) from exc
    if result < 0:
        raise ValueError(f"campaign_detail id={db_id} has negative {field}: {result}")
    return result


def parse_utc_arg(value: str) -> datetime:
    raw = str(value or "").strip()
    if not raw:
        raise argparse.ArgumentTypeError("UTC timestamp may not be empty")
    try:
        parsed = datetime.fromisoformat(raw.replace("Z", "+00:00"))
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"invalid UTC timestamp {value!r}; use ISO-8601, e.g. 2026-08-13T22:02:45Z"
        ) from exc
    if parsed.tzinfo is None:
        raise argparse.ArgumentTypeError(
            f"timestamp {value!r} has no timezone; append Z for UTC"
        )
    return parsed.astimezone(timezone.utc)


def iter_rows(
    batch_size: int,
    *,
    start_utc: Optional[datetime],
    end_utc: Optional[datetime],
) -> Iterator[PopulationRow]:
    range_sql = ""
    params: list[Any] = [CAMPAIGN_TYPE, CLOCKS_SCHEMA]
    if start_utc is not None:
        range_sql += f"\n          AND ({GNSS_UTC_SQL})::timestamptz >= %s"
        params.append(start_utc)
    if end_utc is not None:
        range_sql += f"\n          AND ({GNSS_UTC_SQL})::timestamptz <= %s"
        params.append(end_utc)

    sql = f"""
        SELECT
            id,
            sequence,
            campaign,
            {GNSS_UTC_SQL} AS gnss_utc,
            payload #>> '{{clocks,stats,reset_count}}' AS reset_count,
            payload #>> '{{clocks,stats,update_count}}' AS update_count,
            payload #>> '{{clocks,stats,ocxo1,welford,n}}' AS ocxo1_n,
            payload #>> '{{clocks,stats,auxiliary_welford,ocxo1_dac,n}}' AS dac1_n,
            payload #>> '{{clocks,stats,ocxo2,welford,n}}' AS ocxo2_n,
            payload #>> '{{clocks,stats,auxiliary_welford,ocxo2_dac,n}}' AS dac2_n,
            COALESCE((payload #>> '{{holistic_restore_superseded}}')::boolean, false)
                AS superseded,
            NULLIF(payload #>> '{{recovery_custody,classification}}', '')
                AS recovery_classification
        FROM campaign_detail
        WHERE campaign_type = %s
          AND payload #>> '{{schema}}' = %s
          {range_sql}
        ORDER BY id ASC
    """

    with open_db(row_dict=True) as conn:
        conn.execute("SET TRANSACTION READ ONLY")
        cur = conn.cursor(name="dac_population_audit_stream")
        cur.itersize = batch_size
        cur.execute(sql, tuple(params))

        for row in cur:
            db_id = int(row["id"])
            yield PopulationRow(
                db_id=db_id,
                gnss_utc=row["gnss_utc"],
                sequence=optional_int(row["sequence"], db_id=db_id, field="sequence"),
                campaign=row["campaign"],
                reset_count=optional_int(
                    row["reset_count"], db_id=db_id, field="stats.reset_count"
                ),
                update_count=optional_int(
                    row["update_count"], db_id=db_id, field="stats.update_count"
                ),
                ocxo1_n=optional_int(
                    row["ocxo1_n"], db_id=db_id, field="OCXO1 Welford N"
                ),
                dac1_n=optional_int(
                    row["dac1_n"], db_id=db_id, field="OCXO1 DAC Welford N"
                ),
                ocxo2_n=optional_int(
                    row["ocxo2_n"], db_id=db_id, field="OCXO2 Welford N"
                ),
                dac2_n=optional_int(
                    row["dac2_n"], db_id=db_id, field="OCXO2 DAC Welford N"
                ),
                superseded=bool(row["superseded"]),
                recovery_classification=row["recovery_classification"],
            )


def fmt_int(value: Optional[int], width: int) -> str:
    return f"{value:{width}d}" if value is not None else f"{'---':>{width}}"


def fmt_delta(value: Optional[int]) -> str:
    return f"{value:+10d}" if value is not None else f"{'---':>10}"


def print_header(
    batch_size: int,
    *,
    start_utc: Optional[datetime],
    end_utc: Optional[datetime],
) -> None:
    print(
        f"DAC population ancestry audit   source=CLOCKS_V4   "
        f"order=campaign_detail.id   server_batch={batch_size}"
    )
    if start_utc is not None or end_utc is not None:
        start_text = start_utc.strftime("%Y-%m-%d %H:%M:%S") if start_utc else "-infinity"
        end_text = end_utc.strftime("%Y-%m-%d %H:%M:%S") if end_utc else "+infinity"
        print(f"Range UTC: {start_text} .. {end_text}")
    print()
    print(
        f"{'EVENT':8} {'DB_ID':>9}  {'GNSS UTC':20}  {'SEQ':>8}  "
        f"{'RESET':>5} {'UPDATE':>8}  "
        f"{'O1_N':>8} {'D1_N':>8} {'D1-O1':>10}  "
        f"{'O2_N':>8} {'D2_N':>8} {'D2-O2':>10}  "
        f"{'CAMPAIGN':14} {'RESTORE'}"
    )
    print(
        f"{'─' * 8} {'─' * 9}  {'─' * 20}  {'─' * 8}  "
        f"{'─' * 5} {'─' * 8}  "
        f"{'─' * 8} {'─' * 8} {'─' * 10}  "
        f"{'─' * 8} {'─' * 8} {'─' * 10}  "
        f"{'─' * 14} {'─' * 20}"
    )


def print_row(event: str, row: PopulationRow) -> None:
    restore_bits = []
    if row.superseded:
        restore_bits.append("SUPERSEDED")
    if row.recovery_classification:
        restore_bits.append(row.recovery_classification)
    restore = ",".join(restore_bits) if restore_bits else "---"
    stamp = (row.gnss_utc or "---")[:20]
    campaign = (row.campaign or "---")[:14]

    print(
        f"{event:8} {row.db_id:9d}  {stamp:20}  "
        f"{fmt_int(row.sequence, 8)}  "
        f"{fmt_int(row.reset_count, 5)} {fmt_int(row.update_count, 8)}  "
        f"{fmt_int(row.ocxo1_n, 8)} {fmt_int(row.dac1_n, 8)} {fmt_delta(row.delta1)}  "
        f"{fmt_int(row.ocxo2_n, 8)} {fmt_int(row.dac2_n, 8)} {fmt_delta(row.delta2)}  "
        f"{campaign:14} {restore}",
        flush=True,
    )


def report(
    batch_size: int,
    *,
    start_utc: Optional[datetime],
    end_utc: Optional[datetime],
) -> None:
    print_header(batch_size, start_utc=start_utc, end_utc=end_utc)

    previous_delta: Optional[tuple[Optional[int], Optional[int]]] = None
    previous_reset: Optional[int] = None
    previous_availability: Optional[tuple[bool, bool, bool, bool]] = None
    scanned = 0
    displayed = 0
    comparable = 0
    first_comparable_seen = False

    for row in iter_rows(batch_size, start_utc=start_utc, end_utc=end_utc):
        scanned += 1
        availability = (
            row.ocxo1_n is not None,
            row.dac1_n is not None,
            row.ocxo2_n is not None,
            row.dac2_n is not None,
        )
        delta = (row.delta1, row.delta2)
        if row.delta1 is not None and row.delta2 is not None:
            comparable += 1

        events = []
        if previous_availability is None or availability != previous_availability:
            events.append("AVAIL")
        if previous_reset is not None and row.reset_count != previous_reset:
            events.append("RESET")
        if row.delta1 is not None and row.delta2 is not None:
            if not first_comparable_seen:
                events.append("FIRST")
                first_comparable_seen = True
            elif previous_delta != delta:
                events.append("DELTA")

        if events:
            print_row("+".join(events), row)
            displayed += 1

        previous_availability = availability
        previous_reset = row.reset_count
        if row.delta1 is not None and row.delta2 is not None:
            previous_delta = delta

        if scanned % PROGRESS_EVERY == 0:
            print(f"... scanned {scanned:,} CLOCKS_V4 rows", flush=True)

    if scanned == 0:
        raise RuntimeError("No CLOCKS_V4 rows found in campaign_detail")

    print(
        f"\nScanned {scanned:,} CLOCKS_V4 rows; "
        f"{comparable:,} had all four populations; displayed {displayed:,} transitions."
    )


def parse(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Locate durable DAC-vs-OCXO Welford population ancestry changes in "
            "CLOCKS_V4 history."
        )
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=DEFAULT_BATCH_SIZE,
        help=f"server-side cursor fetch size (default: {DEFAULT_BATCH_SIZE})",
    )
    parser.add_argument(
        "--start-utc",
        type=parse_utc_arg,
        help="inclusive GNSS UTC lower bound, e.g. 2026-08-13T22:02:45Z",
    )
    parser.add_argument(
        "--end-utc",
        type=parse_utc_arg,
        help="inclusive GNSS UTC upper bound, e.g. 2026-08-13T22:03:40Z",
    )
    args = parser.parse_args(argv[1:])
    if args.batch_size <= 0:
        parser.error("--batch-size must be positive")
    if (
        args.start_utc is not None
        and args.end_utc is not None
        and args.end_utc < args.start_utc
    ):
        parser.error("--end-utc must be greater than or equal to --start-utc")
    return args


def main(argv: Sequence[str]) -> None:
    args = parse(argv)
    report(
        args.batch_size,
        start_utc=args.start_utc,
        end_utc=args.end_utc,
    )


if __name__ == "__main__":
    main(sys.argv)
