"""ZPNet frequency_relaxation — streaming OCXO frequency-relaxation profile.

The report reduces CLOCKS ``campaign_detail`` history to one endpoint per
elapsed hour.  It is deliberately descriptive: no smoothing, fitting, or
interpretation is applied to the published PPB values.

Two modes are supported:

  frequency_relaxation.py CAMPAIGN
      Reads one TEMPEST campaign and reports the CLOCKS 60-minute PPB bucket.

  frequency_relaxation.py
      Reads all CLOCKS_V4 campaign_detail rows and reports the same CLOCKS
      60-minute PPB bucket, ignoring campaign boundaries.

Optional --start-utc/--end-utc bounds restrict either mode to an inclusive
GNSS-UTC range before the server-side cursor streams rows.

Rows are streamed through a named PostgreSQL cursor.  Only the latest
observation in the current elapsed-hour bucket is retained, so memory use is
independent of database history length.  When the first observation from the
next hour arrives, the completed hour is printed immediately.  The final
partial hour is printed at EOF.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from typing import Any, Iterator, Optional, Sequence

from zpnet.shared.db import open_db

CAMPAIGN_TYPE = "TEMPEST"
CAMPAIGN_SCHEMA = "TEMPEST_FRAGMENT_V1"
CLOCKS_SCHEMA = "CLOCKS_V4"
DEFAULT_BATCH_SIZE = 256
PPS_COUNT_SQL = """
NULLIF(
    payload #>> '{campaign,public_count}',
    ''
)::bigint
"""
GNSS_UTC_SQL = """
COALESCE(
    NULLIF(payload #>> '{clocks,gnss_time_utc}', ''),
    NULLIF(payload #>> '{gnss,gnss_time_utc}', '')
)
"""


@dataclass(frozen=True)
class Observation:
    db_id: int
    utc: datetime
    ocxo1_ppb: float
    ocxo2_ppb: float


@dataclass
class ScanStats:
    rows_scanned: int = 0
    rows_skipped: int = 0
    unavailable_60_min_ppb_rows: int = 0
    unavailable_gnss_utc_rows: int = 0


def parse_utc(value: Any, db_id: int) -> datetime:
    if not isinstance(value, str) or not value.endswith("Z"):
        raise ValueError(
            f"campaign_detail id={db_id} has no valid observation UTC: {value!r}"
        )
    parsed = datetime.fromisoformat(value[:-1] + "+00:00")
    if parsed.utcoffset() != timedelta(0):
        raise ValueError(
            f"campaign_detail id={db_id} observation UTC is not UTC: {value!r}"
        )
    return parsed.astimezone(timezone.utc)


def require_ppb(value: Any, db_id: int, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(
            f"campaign_detail id={db_id} has invalid {path}: {value!r}"
        )
    return float(value)


def _assert_campaign_indexed(cur: Any, campaign: str) -> None:
    cur.execute(
        f"""
        SELECT count(*) AS missing_count
        FROM campaign_detail
        WHERE campaign_type = %s
          AND campaign = %s
          AND payload #>> '{{campaign,schema}}' = %s
          AND {PPS_COUNT_SQL} IS NULL
        """,
        (CAMPAIGN_TYPE, campaign, CAMPAIGN_SCHEMA),
    )
    row = cur.fetchone()
    if row is None:
        raise RuntimeError("campaign index preflight returned no row")
    missing = int(row["missing_count"])
    if missing != 0:
        raise RuntimeError(
            f"TEMPEST campaign {campaign!r} has {missing:,} decorated rows without "
            "campaign.public_count"
        )


def iter_observations(
    campaign: Optional[str],
    *,
    batch_size: int,
    scan_stats: ScanStats,
    start_utc: Optional[datetime],
    end_utc: Optional[datetime],
) -> Iterator[Observation]:
    """Stream canonical CLOCKS 60-minute PPB observations from campaign_detail."""
    ocxo1_sql = "(payload #>> '{clocks,stats,ocxo1,ppb_buckets,60_min}')::double precision"
    ocxo2_sql = "(payload #>> '{clocks,stats,ocxo2,ppb_buckets,60_min}')::double precision"
    ppb_label = "60-MIN PPB"

    if campaign is None:
        where_terms = ["payload #>> '{schema}' = %s"]
        params_list: list[Any] = [CLOCKS_SCHEMA]
        order_sql = f"ORDER BY ({GNSS_UTC_SQL})::timestamptz ASC NULLS LAST, id ASC"
    else:
        where_terms = [
            "campaign_type = %s",
            "campaign = %s",
            "payload #>> '{campaign,schema}' = %s",
            f"{PPS_COUNT_SQL} IS NOT NULL",
        ]
        params_list = [CAMPAIGN_TYPE, campaign, CAMPAIGN_SCHEMA]
        order_sql = f"ORDER BY {PPS_COUNT_SQL} ASC, id ASC"

    if start_utc is not None:
        where_terms.append(f"({GNSS_UTC_SQL})::timestamptz >= %s")
        params_list.append(start_utc)
    if end_utc is not None:
        where_terms.append(f"({GNSS_UTC_SQL})::timestamptz <= %s")
        params_list.append(end_utc)

    where_sql = "WHERE " + "\n          AND ".join(where_terms)
    params = tuple(params_list)

    sql = f"""
        SELECT
            id,
            {GNSS_UTC_SQL} AS observation_utc,
            {ocxo1_sql} AS ocxo1_ppb,
            {ocxo2_sql} AS ocxo2_ppb
        FROM campaign_detail
        {where_sql}
        {order_sql}
    """

    with open_db(row_dict=True) as conn:
        conn.execute("SET TRANSACTION READ ONLY")
        if campaign is not None:
            check = conn.cursor()
            _assert_campaign_indexed(check, campaign)
            check.close()

        cur = conn.cursor(name="frequency_relaxation_stream")
        cur.itersize = batch_size
        cur.execute(sql, params)

        for row in cur:
            scan_stats.rows_scanned += 1
            db_id = int(row["id"])
            gnss_utc_raw = row["observation_utc"]
            ocxo1_raw = row["ocxo1_ppb"]
            ocxo2_raw = row["ocxo2_ppb"]
            skip_row = False

            if campaign is None:
                if ocxo1_raw is None and ocxo2_raw is None:
                    scan_stats.unavailable_60_min_ppb_rows += 1
                    skip_row = True
                elif ocxo1_raw is None or ocxo2_raw is None:
                    raise ValueError(
                        f"campaign_detail id={db_id} has asymmetric 60-MIN PPB availability: "
                        f"ocxo1={ocxo1_raw!r} ocxo2={ocxo2_raw!r}"
                    )
                else:
                    ocxo1 = require_ppb(ocxo1_raw, db_id, f"OCXO1 {ppb_label}")
                    ocxo2 = require_ppb(ocxo2_raw, db_id, f"OCXO2 {ppb_label}")
            else:
                ocxo1 = require_ppb(ocxo1_raw, db_id, f"OCXO1 {ppb_label}")
                ocxo2 = require_ppb(ocxo2_raw, db_id, f"OCXO2 {ppb_label}")

            if gnss_utc_raw is None:
                scan_stats.unavailable_gnss_utc_rows += 1
                skip_row = True

            if skip_row:
                scan_stats.rows_skipped += 1
                continue

            yield Observation(
                db_id=db_id,
                utc=parse_utc(gnss_utc_raw, db_id),
                ocxo1_ppb=ocxo1,
                ocxo2_ppb=ocxo2,
            )


def print_header(
    campaign: Optional[str],
    batch_size: int,
    origin: datetime,
    start_utc: Optional[datetime],
    end_utc: Optional[datetime],
) -> None:
    if campaign is None:
        print(f"Mode: GLOBAL   source=60-MIN PPB   server_batch={batch_size}")
    else:
        print(
            f"Mode: CAMPAIGN campaign={campaign}   source=60-MIN PPB   "
            f"server_batch={batch_size}"
        )
    print(f"Start UTC: {origin.strftime('%Y-%m-%d %H:%M:%S')}")
    if start_utc is not None or end_utc is not None:
        start_label = start_utc.strftime('%Y-%m-%d %H:%M:%S') if start_utc else "OPEN"
        end_label = end_utc.strftime('%Y-%m-%d %H:%M:%S') if end_utc else "OPEN"
        print(f"Range UTC: {start_label} .. {end_label}")
    print()
    print(f"{'HOUR':>6}  {'END UTC':19}  {'OCXO1 PPB':>13}  {'OCXO2 PPB':>13}")
    print(f"{'─' * 6}  {'─' * 19}  {'─' * 13}  {'─' * 13}")


def print_endpoint(hour_index: int, observation: Observation) -> None:
    stamp = observation.utc.strftime("%Y-%m-%d %H:%M:%S")
    print(
        f"{hour_index:6d}  {stamp}  "
        f"{observation.ocxo1_ppb:+13.6f}  {observation.ocxo2_ppb:+13.6f}",
        flush=True,
    )


def report(
    campaign: Optional[str],
    batch_size: int,
    start_utc: Optional[datetime],
    end_utc: Optional[datetime],
) -> None:
    scan_stats = ScanStats()
    observations = iter_observations(
        campaign,
        batch_size=batch_size,
        scan_stats=scan_stats,
        start_utc=start_utc,
        end_utc=end_utc,
    )
    try:
        first = next(observations)
    except StopIteration:
        scope = "global CLOCKS_V4 history" if campaign is None else f"campaign {campaign!r}"
        raise RuntimeError(f"No campaign_detail rows found for {scope}")

    origin = first.utc
    previous_utc = first.utc
    current_hour = 0
    endpoint = first
    processed = 1
    displayed = 0

    print_header(campaign, batch_size, origin, start_utc, end_utc)

    for observation in observations:
        processed += 1

        if observation.utc < previous_utc:
            raise RuntimeError(
                "campaign_detail chronology moved backward: "
                f"id={observation.db_id} utc={observation.utc.isoformat()} "
                f"previous={previous_utc.isoformat()}"
            )
        previous_utc = observation.utc

        elapsed_seconds = (observation.utc - origin).total_seconds()
        hour_index = int(elapsed_seconds // 3600.0)
        if hour_index != current_hour:
            print_endpoint(current_hour, endpoint)
            displayed += 1
            current_hour = hour_index

        endpoint = observation

    print_endpoint(current_hour, endpoint)
    displayed += 1

    if campaign is None:
        print(
            f"\nScanned {scan_stats.rows_scanned:,} CLOCKS_V4 rows; "
            f"analyzed {processed:,}; skipped {scan_stats.rows_skipped:,} rows "
            f"(60-MIN PPB unavailable={scan_stats.unavailable_60_min_ppb_rows:,}, "
            f"GNSS UTC unavailable={scan_stats.unavailable_gnss_utc_rows:,}); "
            f"displayed {displayed:,} hourly endpoints."
        )
    elif scan_stats.rows_skipped:
        print(
            f"\nScanned {scan_stats.rows_scanned:,} campaign rows; analyzed {processed:,}; "
            f"skipped {scan_stats.rows_skipped:,} rows with GNSS UTC unavailable; "
            f"displayed {displayed:,} hourly endpoints."
        )
    else:
        print(f"\nProcessed {processed:,} rows; displayed {displayed:,} hourly endpoints.")


def parse_cli_utc(value: str) -> datetime:
    """Parse one explicit UTC command-line boundary."""
    raw = str(value).strip()
    if not raw.endswith("Z"):
        raise argparse.ArgumentTypeError(
            f"UTC boundary must end in Z (example 2026-08-13T22:03:00Z): {value!r}"
        )
    try:
        parsed = datetime.fromisoformat(raw[:-1] + "+00:00")
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid UTC boundary {value!r}: {exc}") from exc
    if parsed.utcoffset() != timedelta(0):
        raise argparse.ArgumentTypeError(f"UTC boundary is not UTC: {value!r}")
    return parsed.astimezone(timezone.utc)


def parse(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Stream OCXO frequency relaxation as hourly CLOCKS 60-MIN PPB endpoints. "
            "Omit CAMPAIGN for global history; provide CAMPAIGN to scope the same statistic."
        )
    )
    parser.add_argument(
        "campaign",
        nargs="?",
        help="TEMPEST campaign name; omit for global CLOCKS history",
    )
    parser.add_argument(
        "--start-utc",
        type=parse_cli_utc,
        help="inclusive GNSS UTC lower bound, e.g. 2026-08-13T22:02:45Z",
    )
    parser.add_argument(
        "--end-utc",
        type=parse_cli_utc,
        help="inclusive GNSS UTC upper bound, e.g. 2026-08-13T22:03:40Z",
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=DEFAULT_BATCH_SIZE,
        help=f"server-side cursor fetch size (default: {DEFAULT_BATCH_SIZE})",
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
    report(args.campaign, args.batch_size, args.start_utc, args.end_utc)


if __name__ == "__main__":
    main(sys.argv)
