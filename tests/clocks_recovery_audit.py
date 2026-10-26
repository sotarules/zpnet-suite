"""Audit the durable CLOCKS recovery boundary after a TEMPEST silence failure.

    zt clocks_recovery_audit NewOcxos1
    zt clocks_recovery_audit NewOcxos1 --rows 96 --neighbors 12

Print one copyable JSONL report on STDOUT. Read-only: no repairs, firmware
commands, database writes, fits, or alteration of CLOCKS custody. Independent,
paced PostgreSQL transactions use the same timeouts as zpnet.shared.report_db.
Each page is selected by relational campaign/id before projecting JSONB fields.
main(argv) receives argv[0], as required by the zt runner.
"""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import sys
import time
from typing import Any

from zpnet.shared.db import open_db
from zpnet.shared.report_db import add_read_arguments, validate_read_arguments

VERSION = "2026-10-26.1"
CAMPAIGN_TYPE = "TEMPEST"
CONFIG_KEYS = (
    "CLOCKS_RECOVERY",
    "CLOCKS_RESTORE_SUPERSEDED_RANGES",
    "CLOCKS_ALPHA_LINEAGE_CUTOFF",
)

# Only project recovery/lineage fields, not the large nested producer payload.
DETAIL_COLUMNS = """
    id, campaign, sequence, viable,
    payload #>> '{schema}' AS schema,
    payload #>> '{published_at_utc}' AS published_at_utc,
    payload #>> '{campaign,state}' AS campaign_state,
    payload #>> '{campaign,public_count}' AS public_count,
    payload #>> '{campaign,adjudication,science_eligible}' AS science_eligible,
    payload #>> '{clocks,stats,reset_count}' AS reset_count,
    payload #>> '{clocks,stats,update_count}' AS update_count,
    payload #>> '{holistic_restore_superseded}' AS superseded,
    payload #>> '{recovery_custody,generation}' AS recovery_generation,
    payload #>> '{recovery_custody,classification}' AS recovery_classification,
    payload #>> '{recovery_custody,restore_authority}' AS recovery_restore_authority,
    payload #>> '{recovery_custody,reason}' AS recovery_reason,
    payload #>> '{startup_custody,classification}' AS startup_classification,
    payload #>> '{startup_custody,restore_authority}' AS startup_restore_authority,
    payload #>> '{clocks,gnss_time_utc}' AS clocks_gnss_time_utc,
    payload #>> '{clocks,stats,ocxo1,welford,n}' AS ocxo1_n,
    payload #>> '{clocks,stats,ocxo2,welford,n}' AS ocxo2_n,
    payload #>> '{clocks,stats,auxiliary_welford,ocxo1_dac,n}' AS dac1_n,
    payload #>> '{clocks,stats,auxiliary_welford,ocxo2_dac,n}' AS dac2_n,
    (payload #> '{campaign,adjudication}' IS NOT NULL) AS adjudicated,
    (payload ? 'recovery_receipt') AS has_recovery_receipt
"""


def compact(value: Any) -> str:
    return json.dumps(value, ensure_ascii=True, allow_nan=False, separators=(",", ":"), default=str)


def emit(kind: str, **fields: Any) -> None:
    print(compact({"type": kind, **fields}), flush=True)


def as_object(value: Any, *, context: str) -> dict[str, Any]:
    if isinstance(value, str):
        value = json.loads(value)
    if not isinstance(value, dict):
        raise ValueError(f"{context} must be a JSON object, got {type(value).__name__}")
    return value


def as_id(value: Any, *, context: str) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{context} must be a positive integer")
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{context} must be a positive integer: {value!r}") from exc
    if result <= 0:
        raise ValueError(f"{context} must be positive: {value!r}")
    return result


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign", help="exact active/historical TEMPEST campaign name")
    parser.add_argument("--rows", type=int, default=64,
                        help="latest campaign rows to inspect (default 64; max 256)")
    parser.add_argument("--neighbors", type=int, default=10,
                        help="rows on each side of DAC boundary (default 10; max 32)")
    add_read_arguments(parser)
    args = parser.parse_args(argv[1:])
    validate_read_arguments(parser, args)
    if not args.campaign.strip():
        parser.error("campaign must be nonempty")
    if not 1 <= args.rows <= 256:
        parser.error("--rows must be 1..256")
    if not 0 <= args.neighbors <= 32:
        parser.error("--neighbors must be 0..32")
    args.campaign = args.campaign.strip()
    return args


class PacedReadOnly:
    """Small reads with the zpnet.shared.report_db transaction/pace doctrine."""

    def __init__(self, args: argparse.Namespace):
        self.pause = args.batch_pause
        self.batch_size = args.batch_size
        self.last_finished: float | None = None
        self.reads = 0

    def query(self, sql: str, params: tuple[Any, ...] = ()) -> list[dict[str, Any]]:
        if self.last_finished is not None:
            time.sleep(max(0.0, self.pause - (time.monotonic() - self.last_finished)))
        try:
            with open_db(row_dict=True) as conn:
                conn.execute("SET TRANSACTION READ ONLY")
                conn.execute("SET LOCAL statement_timeout = '5s'")
                conn.execute("SET LOCAL lock_timeout = '500ms'")
                conn.execute("SET LOCAL work_mem = '4MB'")
                conn.execute("SET LOCAL max_parallel_workers_per_gather = 0")
                conn.execute("SET LOCAL jit = off")
                result = conn.execute(sql, params).fetchall()
        finally:
            self.last_finished = time.monotonic()
        self.reads += 1
        return result

    def detail(self, detail_id: int) -> dict[str, Any] | None:
        rows = self.query(
            f"SELECT {DETAIL_COLUMNS} FROM campaign_detail "
            "WHERE campaign_type = %s AND id = %s",
            (CAMPAIGN_TYPE, detail_id),
        )
        return rows[0] if rows else None

    def latest(self, campaign: str, total: int) -> list[dict[str, Any]]:
        rows: list[dict[str, Any]] = []
        before: int | None = None
        while len(rows) < total:
            page_size = min(self.batch_size, total - len(rows))
            if before is None:
                sql = (
                    "WITH page AS MATERIALIZED (SELECT id,campaign,sequence,viable,payload "
                    "FROM campaign_detail WHERE campaign_type = %s AND campaign = %s "
                    "ORDER BY id DESC LIMIT %s) "
                    f"SELECT {DETAIL_COLUMNS} FROM page ORDER BY id DESC"
                )
                params = (CAMPAIGN_TYPE, campaign, page_size)
            else:
                sql = (
                    "WITH page AS MATERIALIZED (SELECT id,campaign,sequence,viable,payload "
                    "FROM campaign_detail WHERE campaign_type = %s AND campaign = %s "
                    "AND id < %s ORDER BY id DESC LIMIT %s) "
                    f"SELECT {DETAIL_COLUMNS} FROM page ORDER BY id DESC"
                )
                params = (CAMPAIGN_TYPE, campaign, before, page_size)
            page = self.query(sql, params)
            if not page:
                break
            rows.extend(page)
            before = int(page[-1]["id"])
            if len(page) < page_size:
                break
        return rows

    def neighbors(self, campaign: str, detail_id: int, n: int) -> list[dict[str, Any]]:
        if n == 0:
            return []
        sql = (
            "WITH page AS MATERIALIZED (SELECT id,campaign,sequence,viable,payload "
            "FROM campaign_detail WHERE campaign_type = %s AND campaign = %s "
            "AND id < %s ORDER BY id DESC LIMIT %s) "
            f"SELECT {DETAIL_COLUMNS} FROM page ORDER BY id DESC"
        )
        before = self.query(sql, (CAMPAIGN_TYPE, campaign, detail_id, n))
        sql = (
            "WITH page AS MATERIALIZED (SELECT id,campaign,sequence,viable,payload "
            "FROM campaign_detail WHERE campaign_type = %s AND campaign = %s "
            "AND id > %s ORDER BY id ASC LIMIT %s) "
            f"SELECT {DETAIL_COLUMNS} FROM page ORDER BY id ASC"
        )
        after = self.query(sql, (CAMPAIGN_TYPE, campaign, detail_id, n))
        return list(reversed(before)) + after


def config_view(config_key: str, raw: Any) -> dict[str, Any]:
    data = as_object(raw, context=f"config.{config_key}")
    if config_key == "CLOCKS_RECOVERY":
        keys = (
            "schema", "durable_source_detail_id", "recoverable", "status",
            "reset_count", "update_count", "current_sequence", "rolling_sequence",
            "second_count", "expected_second_count", "minute_count", "expected_minute_count",
        )
    elif config_key == "CLOCKS_RESTORE_SUPERSEDED_RANGES":
        keys = ("schema", "ranges")
    else:
        keys = ("schema", "cutoff_detail_id", "reason", "updated_at_utc")
    return {key: data.get(key) for key in keys if key in data}


def report(args: argparse.Namespace) -> None:
    db = PacedReadOnly(args)
    emit("header", schema="CLOCKS_RECOVERY_AUDIT_V1", version=VERSION,
         created_utc=datetime.now(timezone.utc).isoformat(),
         campaign=args.campaign, read_only=True,
         batch_size=args.batch_size, batch_pause_s=args.batch_pause,
         recent_rows=args.rows, neighbors=args.neighbors)

    masters = db.query(
        "SELECT id,active,ts,payload FROM campaign_master "
        "WHERE campaign_type = %s AND campaign = %s",
        (CAMPAIGN_TYPE, args.campaign),
    )
    if len(masters) != 1:
        raise RuntimeError(f"expected one campaign master for {args.campaign!r}; got {len(masters)}")
    master = masters[0]
    payload = as_object(master["payload"], context="campaign master payload")
    dac_raw = payload.get("campaign_dac")
    dac = as_object(dac_raw, context="campaign_dac") if dac_raw is not None else None
    bound_id = as_id(dac["last_detail_id"], context="campaign_dac.last_detail_id") if dac else None
    emit("master", id=master["id"], active=master["active"], ts=master["ts"],
         started_at=payload.get("started_at"), stopped_at=payload.get("stopped_at"),
         location=payload.get("location"), flash_cut_from=payload.get("flash_cut_from"),
         campaign_dac=dac)

    config_rows = db.query(
        "SELECT config_key,payload FROM config WHERE config_key IN (%s,%s,%s)",
        CONFIG_KEYS,
    )
    configs = {row["config_key"]: config_view(row["config_key"], row["payload"])
               for row in config_rows}
    for name in CONFIG_KEYS:
        emit("config", key=name, present=name in configs, data=configs.get(name))
    recovery_source = configs.get("CLOCKS_RECOVERY", {}).get("durable_source_detail_id")

    recent = db.latest(args.campaign, args.rows)
    counts = Counter()
    for row in recent:
        counts["scanned"] += 1
        counts["superseded"] += row["superseded"] == "true"
        counts["recovery_custody"] += row["recovery_generation"] is not None
        counts["startup_custody"] += row["startup_classification"] is not None
        counts["adjudicated"] += bool(row["adjudicated"])
        emit("recent", **row)

    if bound_id is not None:
        bound = db.detail(bound_id)
        if bound is None:
            emit("bound", id=bound_id, missing=True)
        else:
            emit("bound", matches_campaign=bound["campaign"] == args.campaign, **bound)
            for row in db.neighbors(args.campaign, bound_id, args.neighbors):
                if row["id"] not in {x["id"] for x in recent}:
                    emit("neighbor", **row)

    if recovery_source is not None:
        source_id = as_id(recovery_source, context="CLOCKS_RECOVERY.durable_source_detail_id")
        if source_id != bound_id:
            source = db.detail(source_id)
            emit("alpha_source", id=source_id, missing=source is None,
                 detail=source)

    ranges = configs.get("CLOCKS_RESTORE_SUPERSEDED_RANGES", {}).get("ranges") or []
    if bound_id is not None:
        bound = db.detail(bound_id)
        emit("boundary_court", bound_id=bound_id,
             bound_is_superseded=(bound or {}).get("superseded") == "true",
             bound_recovery_classification=(bound or {}).get("recovery_classification"),
             bound_startup_classification=(bound or {}).get("startup_classification"),
             master_dac_last_sequence=(dac or {}).get("last_sequence"),
             bound_sequence=(bound or {}).get("sequence"),
             master_dac_last_public_count=(dac or {}).get("last_public_count"),
             bound_public_count=(bound or {}).get("public_count"),
             in_exclusive_restore_interval=any(
                 int(entry["source_detail_id"]) < bound_id < int(entry["first_recovered_detail_id"])
                 for entry in ranges
             ))
    emit("summary", **dict(counts), database_reads=db.reads,
         current_master_active=master["active"],
         bound_id=bound_id, recovery_source_id=recovery_source,
         note="Observations only. Superseded rows are never promoted or repaired by this tool.")


def main(argv: list[str]) -> int:
    report(parse_args(argv))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
