"""ZPNet raw_cycles — conservative server-streamed four-rail audit.

The report reads TEMPEST-decorated ``campaign_detail`` rows in campaign-public
PPS order through a named PostgreSQL cursor. Each unified state payload is
decoded, audited, printed, and discarded. Memory use is therefore independent
of campaign length.

The SQL explicitly selects ``campaign_type='TEMPEST'`` and derives the campaign
PPS identity from ``payload.campaign``. That identity must agree with the
immutable identity carried inside the merged campaign view.
"""

from __future__ import annotations

import json
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterator, Optional, Sequence, Tuple

from zpnet.shared.db import open_db

RAILS: Tuple[str, ...] = ("PPS", "VCLOCK", "OCXO1", "OCXO2")
RAIL_KEYS = {name: name.lower() for name in RAILS}
DEFAULT_GATE_CYCLES = 500
EXPLANATION_GATE_CYCLES = 16
PLAUSIBLE_MIN = 900_000_000
PLAUSIBLE_MAX = 1_100_000_000
DEFAULT_BATCH_SIZE = 16
DEFAULT_PAUSE_MS = 25
CAMPAIGN_TYPE = "TEMPEST"
PPS_COUNT_SQL = """
NULLIF(
    payload #>> '{campaign,public_count}',
    ''
)::bigint
"""


@dataclass
class Rail:
    observed: Optional[int] = None
    published_previous: Optional[int] = None
    published_residual: Optional[int] = None
    computed_previous: Optional[int] = None
    computed_residual: Optional[int] = None
    previous_difference: Optional[int] = None
    residual_difference: Optional[int] = None
    valid: Optional[bool] = None
    delay_status: str = "UNKNOWN"
    delay_by: str = "UNKNOWN"
    residual_delay_valid: Optional[bool] = None
    residual_delay_cycles: Optional[int] = None
    residual_delay_by: str = "UNKNOWN"
    residual_after_delay_cycles: Optional[int] = None
    delay_explains_residual: Optional[bool] = None


@dataclass
class AuditRow:
    count: int
    previous_count: Optional[int]
    count_delta: Optional[int]
    gap: bool
    recovery_boundary: bool
    disposition: str
    timeline_valid: Optional[bool]
    rails: Dict[str, Rail]


def d(v: Any) -> Dict[str, Any]:
    return v if isinstance(v, dict) else {}


def i(v: Any) -> Optional[int]:
    if v is None or isinstance(v, bool):
        return None
    try:
        return int(v)
    except (TypeError, ValueError, OverflowError):
        return None


def b(v: Any) -> Optional[bool]:
    if isinstance(v, bool):
        return v
    if isinstance(v, int):
        return bool(v)
    if isinstance(v, str):
        s = v.strip().lower()
        if s in {"1", "true", "yes", "on", "nominal", "ready"}:
            return True
        if s in {"0", "false", "no", "off", "hold", "anomaly"}:
            return False
    return None


def first_int(*values: Any) -> Optional[int]:
    for value in values:
        parsed = i(value)
        if parsed is not None:
            return parsed
    return None


def root(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Return one persisted CLOCKS_V4 state detail."""
    if not isinstance(payload, dict):
        return {}
    if payload.get("schema") == "CLOCKS_V4":
        return payload
    inner = d(payload.get("payload"))
    return inner if inner.get("schema") == "CLOCKS_V4" else payload


def campaign_view(payload: Dict[str, Any]) -> Dict[str, Any]:
    campaign = d(root(payload).get("campaign"))
    return campaign if campaign.get("schema") == "TEMPEST_FRAGMENT_V1" else {}


def clocks_view(payload: Dict[str, Any]) -> Dict[str, Any]:
    return d(root(payload).get("clocks"))


def _payload_pps_count(payload: Dict[str, Any], campaign: Dict[str, Any]) -> Optional[int]:
    return i(campaign.get("public_count"))


def _assert_campaign_indexed(cur: Any, campaign: str) -> None:
    cur.execute(
        f"""
        SELECT count(*) AS missing_count
        FROM campaign_detail
        WHERE campaign_type = %s
          AND campaign = %s
          AND payload #>> '{{campaign,schema}}' = 'TEMPEST_FRAGMENT_V1'
          AND {PPS_COUNT_SQL} IS NULL
        """,
        (CAMPAIGN_TYPE, campaign),
    )
    row = cur.fetchone()
    missing = int(row["missing_count"] if row else 0)
    if missing:
        raise RuntimeError(
            f"TEMPEST campaign {campaign!r} has {missing:,} decorated rows without "
            "campaign.public_count"
        )


def iter_payloads(
    campaign: str,
    *,
    skip: int,
    limit: int,
    batch_size: int,
    pause_ms: int,
) -> Iterator[Tuple[int, int, Dict[str, Any]]]:
    """Yield ``(db_id, campaign_pps_count, payload)`` using a server-side cursor."""
    with open_db(row_dict=True) as conn:
        conn.execute("SET TRANSACTION READ ONLY")
        check = conn.cursor()
        _assert_campaign_indexed(check, campaign)
        check.close()

        sql = f"""
            SELECT id, {PPS_COUNT_SQL} AS pps_count, payload
            FROM campaign_detail
            WHERE campaign_type = %s
              AND campaign = %s
              AND payload #>> '{{campaign,schema}}' = 'TEMPEST_FRAGMENT_V1'
              AND {PPS_COUNT_SQL} IS NOT NULL
        """
        params: list[Any] = [CAMPAIGN_TYPE, campaign]
        if skip > 0:
            sql += f" AND {PPS_COUNT_SQL} > %s"
            params.append(skip)
        sql += f" ORDER BY {PPS_COUNT_SQL} ASC, id ASC"
        if limit > 0:
            sql += " LIMIT %s"
            params.append(limit)

        cur = conn.cursor(name="raw_cycles_stream")
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
            campaign = campaign_view(payload)
            payload_count = _payload_pps_count(payload, campaign)
            if payload_count is not None and payload_count != db_count:
                raise ValueError(
                    "campaign_detail relational/payload campaign PPS mismatch: "
                    f"id={row['id']} db={db_count} payload={payload_count}"
                )

            yield int(row["id"]), db_count, payload

            fetched_in_batch += 1
            if fetched_in_batch >= max(1, batch_size):
                fetched_in_batch = 0
                if pause_ms > 0:
                    time.sleep(pause_ms / 1000.0)


def explicit_recovery_row(campaign: Dict[str, Any]) -> bool:
    recovery = d(campaign.get("recovery"))
    if not recovery:
        return False
    return bool(
        b(recovery.get("transition_active")) is True
        or b(recovery.get("degraded_active")) is True
        or b(recovery.get("science_quarantine_active")) is True
        or b(recovery.get("reattach_stalled")) is True
    )


def build_row(
    payload: Dict[str, Any],
    db_count: int,
    previous_count: Optional[int],
    previous_observed: Dict[str, Optional[int]],
) -> AuditRow:
    campaign = campaign_view(payload)
    clocks = clocks_view(payload)
    raw = d(clocks.get("raw_cycles"))
    status = d(campaign.get("status"))
    disposition = d(campaign.get("disposition"))

    delta = None if previous_count is None else db_count - previous_count
    gap = delta is not None and delta != 1
    recovery = bool(gap and delta is not None and delta > 1 and explicit_recovery_row(campaign))

    rails: Dict[str, Rail] = {}

    for name in RAILS:
        obj = d(raw.get(RAIL_KEYS[name]))
        observed = i(obj.get("observed_cycles"))
        published_previous = i(obj.get("previous_observed_cycles"))
        published_residual = i(obj.get("residual_cycles"))
        computed_previous = None if gap else previous_observed[name]
        computed_residual = (
            observed - computed_previous
            if observed is not None and computed_previous is not None
            else None
        )
        # CLOCKS_V4 carries the compact ISR-delay verdict beside
        # each raw-cycle rail.  The normalized V4 raw-cycle rail is the sole evidence source.
        delay_status = str(obj.get("delay_status") or "UNKNOWN").upper()
        delay_by = str(obj.get("delay_by") or "UNKNOWN").upper()
        residual_delay_valid = b(obj.get("residual_delay_valid"))
        residual_delay_cycles = i(obj.get("residual_delay_cycles"))
        residual_delay_by = str(obj.get("residual_delay_by") or "UNKNOWN").upper()
        delay_explains_residual = b(obj.get("delay_explains_residual"))

        normalized = (
            computed_residual - residual_delay_cycles
            if computed_residual is not None
            and residual_delay_valid is True
            and residual_delay_cycles is not None
            else None
        )

        rails[name] = Rail(
            observed=observed,
            published_previous=published_previous,
            published_residual=published_residual,
            computed_previous=computed_previous,
            computed_residual=computed_residual,
            previous_difference=(
                published_previous - computed_previous
                if published_previous is not None and computed_previous is not None
                else None
            ),
            residual_difference=(
                published_residual - computed_residual
                if published_residual is not None and computed_residual is not None
                else None
            ),
            valid=b(obj.get("valid")),
            delay_status=delay_status,
            delay_by=delay_by,
            residual_delay_valid=residual_delay_valid,
            residual_delay_cycles=residual_delay_cycles,
            residual_delay_by=residual_delay_by,
            residual_after_delay_cycles=normalized,
            delay_explains_residual=delay_explains_residual,
        )
        previous_observed[name] = observed

    return AuditRow(
        count=db_count,
        previous_count=previous_count,
        count_delta=delta,
        gap=gap,
        recovery_boundary=recovery,
        disposition=str(disposition.get("status") or "ACCEPT").upper(),
        timeline_valid=b(status.get("timeline_valid")),
        rails=rails,
    )


def row_note(row: AuditRow, selected: Sequence[str], gate: int) -> str:
    if row.recovery_boundary:
        skipped = (row.count_delta - 1) if row.count_delta is not None else 0
        return f"RECOVER GAP ({skipped} skipped)"
    if row.gap:
        return f"NON-ADJACENT GAP delta={row.count_delta}"

    notes: list[str] = []
    if row.disposition != "ACCEPT":
        notes.append(row.disposition)
    if row.timeline_valid is False:
        notes.append("TIMELINE_INVALID")

    for name in selected:
        rail = row.rails[name]
        if rail.observed is None:
            notes.append(f"{name}:MISSING")
        elif not PLAUSIBLE_MIN <= rail.observed <= PLAUSIBLE_MAX:
            notes.append(f"{name}:IMPLAUSIBLE")
        if rail.valid is False:
            notes.append(f"{name}:INVALID")
        if rail.previous_difference not in (None, 0):
            notes.append(f"{name}:PREV_DIFF={rail.previous_difference:+d}")
        if rail.residual_difference not in (None, 0):
            notes.append(f"{name}:RES_DIFF={rail.residual_difference:+d}")
        if rail.computed_residual is not None and abs(rail.computed_residual) > gate:
            normalized = rail.residual_after_delay_cycles
            explained = (
                rail.residual_delay_valid is True
                and normalized is not None
                and (
                    rail.delay_explains_residual is True
                    or (
                        rail.delay_explains_residual is None
                        and abs(normalized) <= EXPLANATION_GATE_CYCLES
                    )
                )
            )
            if explained:
                notes.append(
                    f"{name}:RAW={rail.computed_residual:+d} EXPLAINED "
                    f"NORM={normalized:+d}"
                )
            else:
                notes.append(f"{name}:RAW={rail.computed_residual:+d}")
        if rail.delay_status == "DELAYED":
            notes.append(f"{name}:DELAYED_BY_{rail.delay_by}")
        elif rail.delay_status not in {"ON_TIME", "UNKNOWN"}:
            notes.append(f"{name}:{rail.delay_status}")

    return " | ".join(notes) if notes else "ON_TIME"


def fmt(value: Optional[int], width: int, signed: bool = False) -> str:
    if value is None:
        text = "---"
    else:
        text = f"{value:+,d}" if signed else f"{value:,d}"
    return f"{text:>{width}}"


def parse(argv: Sequence[str]) -> tuple[str, int, int, Optional[str], bool, int, int, int]:
    if len(argv) < 2:
        raise SystemExit(
            "Usage: raw_cycles CAMPAIGN [limit] [clock] [--skip N] "
            "[--pathology-only] [--pathology-gate N] "
            "[--batch-size N] [--pause-ms N]"
        )
    campaign = argv[1]
    limit = 0
    skip = 0
    clock: Optional[str] = None
    pathology_only = False
    gate = DEFAULT_GATE_CYCLES
    batch_size = DEFAULT_BATCH_SIZE
    pause_ms = DEFAULT_PAUSE_MS
    positional: list[str] = []

    idx = 2
    while idx < len(argv):
        arg = argv[idx]
        if arg == "--pathology-only":
            pathology_only = True
            idx += 1
        elif arg in {"--skip", "--pathology-gate", "--clock", "--limit",
                     "--batch-size", "--pause-ms"}:
            if idx + 1 >= len(argv):
                raise SystemExit(f"{arg} requires a value")
            value = argv[idx + 1]
            idx += 2
            if arg == "--skip":
                skip = int(value)
            elif arg == "--pathology-gate":
                gate = int(value)
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
            elif key == "--pathology-gate":
                gate = int(value)
            elif key == "--clock":
                clock = value.upper()
            elif key == "--limit":
                limit = int(value)
            elif key == "--batch-size":
                batch_size = int(value)
            elif key == "--pause-ms":
                pause_ms = int(value)
            idx += 1
        elif arg in {"--align-ocxo", "--delay-pps-vclock", "--slip"}:
            idx += 1
        else:
            positional.append(arg)
            idx += 1

    for arg in positional:
        if arg.upper() in RAILS:
            clock = arg.upper()
        else:
            limit = int(arg)

    if min(skip, limit, pause_ms) < 0 or batch_size <= 0:
        raise SystemExit("skip/limit/pause must be nonnegative; batch-size must be positive")
    if clock is not None and clock not in RAILS:
        raise SystemExit("clock must be PPS, VCLOCK, OCXO1, or OCXO2")
    return campaign, limit, skip, clock, pathology_only, gate, batch_size, pause_ms


def main(argv: Sequence[str]) -> None:
    campaign, limit, skip, clock, pathology_only, gate, batch_size, pause_ms = parse(argv)
    selected = (clock,) if clock else RAILS

    print(
        f"Campaign: {campaign}  view={clock or 'ALL'}  "
        f"server_batch={batch_size} pause_ms={pause_ms}"
    )
    header = [f"{'pps':>7}"]
    for name in selected:
        prefix = {"PPS": "p", "VCLOCK": "v", "OCXO1": "o1", "OCXO2": "o2"}[name]
        header += [f"{prefix + '_cyc':>13}", f"{prefix + '_raw':>9}"]
    header.append("NOTE")
    print("  ".join(header))
    print("  ".join("─" * len(field) for field in header))

    previous_count: Optional[int] = None
    previous_observed: Dict[str, Optional[int]] = {name: None for name in RAILS}
    processed = 0
    displayed = 0

    for _db_id, db_count, payload in iter_payloads(
        campaign,
        skip=skip,
        limit=limit,
        batch_size=batch_size,
        pause_ms=pause_ms,
    ):
        row = build_row(payload, db_count, previous_count, previous_observed)
        previous_count = db_count
        processed += 1

        note = row_note(row, selected, gate)
        pathological = note != "ON_TIME"
        if pathology_only and not pathological:
            continue

        fields = [fmt(row.count, 7)]
        for name in selected:
            rail = row.rails[name]
            fields += [
                fmt(rail.observed, 13),
                fmt(None if row.gap else rail.published_residual, 9, signed=True),
            ]
        fields.append(note)
        print("  ".join(fields))
        displayed += 1

    print(f"\nProcessed {processed:,} rows; displayed {displayed:,}.")


if __name__ == "__main__":
    main(sys.argv)
