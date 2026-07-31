"""ZPNet campaign_analyzer — conservative one-pass streaming audit.

This replacement is intentionally modest.  It uses ``timebase.pps_count`` as
the only relational ordering identity, reads through a named PostgreSQL cursor,
decodes one large JSON payload at a time, folds it into compact online state,
and discards it immediately.

It does not retain a full campaign of expanded JSON dictionaries.  Memory use is
bounded by the current payload, the previous compact row, online accumulators,
and a small bounded set of anomaly examples.
"""

from __future__ import annotations

import json
import math
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Iterator, Optional, Sequence, Tuple

from zpnet.shared.db import open_db

NS_PER_SECOND = 1_000_000_000
DWT_EXPECTED_PER_PPS = 1_008_000_000
DEFAULT_BATCH_SIZE = 8
DEFAULT_PAUSE_MS = 50
MAX_EXAMPLES = 12
OCXO_SECOND_WARN_NS = 500
OCXO_SECOND_ALARM_NS = 10_000
PPB_ABSOLUTE_ALARM = 10_000.0
PPB_STEP_ALARM = 100.0


def d(v: Any) -> Dict[str, Any]:
    return v if isinstance(v, dict) else {}


def i(v: Any) -> Optional[int]:
    if v is None or isinstance(v, bool):
        return None
    try:
        return int(v)
    except (TypeError, ValueError, OverflowError):
        return None


def f(v: Any) -> Optional[float]:
    if v is None:
        return None
    try:
        out = float(v)
    except (TypeError, ValueError, OverflowError):
        return None
    return out if math.isfinite(out) else None


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


def path(obj: Dict[str, Any], dotted: str) -> Any:
    cur: Any = obj
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return None
        cur = cur[part]
    return cur


def first_int(*values: Any) -> Optional[int]:
    for value in values:
        out = i(value)
        if out is not None:
            return out
    return None


def first_float(*values: Any) -> Optional[float]:
    for value in values:
        out = f(value)
        if out is not None:
            return out
    return None


def first_bool(*values: Any) -> Optional[bool]:
    for value in values:
        out = b(value)
        if out is not None:
            return out
    return None


def science_disposition(payload: Dict[str, Any],
                        frag: Dict[str, Any]) -> tuple[bool, bool, str]:
    """Resolve completed TIMEBASE science custody.

    The Pi final court is authoritative when present.  Top-level TIMEBASE and
    firmware-fragment fields are compatibility witnesses for older rows.
    Unknown legacy rows remain admitted unless an explicit exclusion signal is
    present; this preserves historical analyzer behavior without allowing a
    known SCIENCE_EXCLUDE row into science statistics or courts.
    """
    court = d(payload.get("final_court"))

    excluded = first_bool(
        court.get("science_excluded"),
        payload.get("science_excluded"),
        frag.get("science_excluded"),
    )
    eligible = first_bool(
        court.get("science_eligible"),
        payload.get("science_eligible"),
        frag.get("science_eligible"),
    )

    candidate_use = str(
        court.get("candidate_use")
        or payload.get("candidate_use")
        or frag.get("candidate_use")
        or ""
    ).strip().upper()
    classification = str(court.get("classification") or "").strip().upper()
    disposition = str(frag.get("candidate_disposition") or "").strip().upper()

    explicit_exclusion = bool(
        excluded is True
        or eligible is False
        or candidate_use in {"AUDIT_ONLY", "SCIENCE_EXCLUDE", "EXCLUDE"}
        or classification in {"SCIENCE_EXCLUDE", "AUDIT_ONLY"}
        or disposition in {"SCIENCE_EXCLUDE", "AUDIT_ONLY"}
    )
    if explicit_exclusion:
        reason = str(
            court.get("rationale")
            or court.get("primary_rule")
            or frag.get("candidate_reason")
            or frag.get("candidate_reason_name")
            or candidate_use
            or classification
            or disposition
            or "science excluded"
        )
        return False, True, reason

    if eligible is True or excluded is False or candidate_use == "SCIENCE_AND_CONTROL":
        return True, False, "science admitted"

    return True, False, "legacy row without explicit science disposition"


@dataclass
class RunningStats:
    n: int = 0
    mean: float = 0.0
    m2: float = 0.0
    min_value: float = math.inf
    max_value: float = -math.inf

    def update(self, value: float) -> None:
        self.n += 1
        delta = value - self.mean
        self.mean += delta / self.n
        self.m2 += delta * (value - self.mean)
        self.min_value = min(self.min_value, value)
        self.max_value = max(self.max_value, value)

    @property
    def stddev(self) -> float:
        return math.sqrt(self.m2 / (self.n - 1)) if self.n >= 2 else 0.0

    def text(self, digits: int = 3) -> str:
        if self.n == 0:
            return "no data"
        return (
            f"n={self.n:,} mean={self.mean:+.{digits}f} "
            f"sd={self.stddev:.{digits}f} "
            f"min={self.min_value:+.{digits}f} max={self.max_value:+.{digits}f}"
        )


@dataclass
class CompactRow:
    db_id: int
    pps: int
    ts: str
    campaign: str
    gnss_ns: Optional[int]
    vclock_ns: Optional[int]
    dwt_cycles: Optional[int]
    ocxo1_ns: Optional[int]
    ocxo2_ns: Optional[int]
    ocxo1_interval_ns: Optional[int]
    ocxo2_interval_ns: Optional[int]
    ocxo1_residual_ns: Optional[int]
    ocxo2_residual_ns: Optional[int]
    ocxo1_valid: Optional[bool]
    ocxo2_valid: Optional[bool]
    dwt_ppb: Optional[float]
    vclock_ppb: Optional[float]
    ocxo1_ppb: Optional[float]
    ocxo2_ppb: Optional[float]
    science_eligible: bool
    science_excluded: bool
    science_reason: str
    recovery_evidence: bool
    recovery_hold: bool
    timeline_ready: Optional[bool]
    clockface_ready: Optional[bool]
    science_ready: Optional[bool]
    welford_n: Dict[str, Optional[int]]
    welford_sd: Dict[str, Optional[float]]
    servo_mode: str
    location: str
    environment: Dict[str, Any]
    final_forensics: Dict[str, Any]


@dataclass
class Audit:
    campaign: str
    first_pps: Optional[int] = None
    last_pps: Optional[int] = None
    first_ts: str = ""
    last_ts: str = ""
    rows: int = 0
    science_rows: int = 0
    science_excluded_rows: int = 0
    expected_rows: int = 0
    gaps: int = 0
    recovery_gaps: int = 0
    unclassified_gaps: int = 0
    repeats: int = 0
    regressions: int = 0
    payload_identity_mismatches: int = 0
    timeline_errors: int = 0
    gnss_identity_errors: int = 0
    gnss_adjacent_errors: int = 0
    dwt_nonmonotonic: int = 0
    ocxo1_interval_alarms: int = 0
    ocxo2_interval_alarms: int = 0
    ocxo1_ledger_errors: int = 0
    ocxo2_ledger_errors: int = 0
    ppb_alarms: int = 0
    ppb_steps: int = 0
    welford_regressions: int = 0
    welford_hold_changes: int = 0
    examples: Dict[str, list[str]] = field(default_factory=dict)
    stats: Dict[str, RunningStats] = field(default_factory=lambda: {
        "dwt_delta": RunningStats(),
        "ocxo1_interval": RunningStats(),
        "ocxo2_interval": RunningStats(),
        "ocxo1_residual": RunningStats(),
        "ocxo2_residual": RunningStats(),
        "ocxo_divergence": RunningStats(),
        "dwt_ppb": RunningStats(),
        "vclock_ppb": RunningStats(),
        "ocxo1_ppb": RunningStats(),
        "ocxo2_ppb": RunningStats(),
    })
    final_row: Optional[CompactRow] = None

    def note(self, kind: str, message: str) -> None:
        bucket = self.examples.setdefault(kind, [])
        if len(bucket) < MAX_EXAMPLES:
            bucket.append(message)


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
            f"campaign {campaign!r} has {missing:,} rows with NULL timebase.pps_count; "
            "backfill the scalar identity before analysis"
        )


def iter_rows(
    campaign: str,
    *,
    batch_size: int,
    pause_ms: int,
    limit: int,
) -> Iterator[Tuple[int, int, str, Dict[str, Any]]]:
    with open_db(row_dict=True) as conn:
        conn.execute("SET TRANSACTION READ ONLY")
        check = conn.cursor()
        _assert_campaign_indexed(check, campaign)
        check.close()

        sql = """
            SELECT id, ts, pps_count, payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY pps_count ASC, id ASC
        """
        params: list[Any] = [campaign]
        if limit > 0:
            sql += " LIMIT %s"
            params.append(limit)

        cur = conn.cursor(name="campaign_analyzer_stream")
        cur.itersize = max(1, batch_size)
        cur.execute(sql, tuple(params))

        in_batch = 0
        for row in cur:
            payload = row["payload"]
            if isinstance(payload, str):
                payload = json.loads(payload)
            if not isinstance(payload, dict):
                continue
            yield int(row["id"]), int(row["pps_count"]), str(row["ts"]), payload
            in_batch += 1
            if in_batch >= max(1, batch_size):
                in_batch = 0
                if pause_ms > 0:
                    time.sleep(pause_ms / 1000.0)


def compact(db_id: int, db_pps: int, ts: str, payload: Dict[str, Any]) -> CompactRow:
    frag = d(payload.get("fragment"))
    if not frag:
        frag = payload
    forensic = d(payload.get("forensics")) or d(frag.get("forensics"))

    payload_pps = first_int(
        payload.get("teensy_pps_vclock_count"),
        payload.get("pps_count"),
        frag.get("teensy_pps_vclock_count"),
        frag.get("pps_count"),
    )
    if payload_pps is not None and payload_pps != db_pps:
        raise ValueError(
            f"TIMEBASE PPS identity mismatch id={db_id} db={db_pps} payload={payload_pps}"
        )

    def ocxo(prefix: str) -> tuple[Optional[int], Optional[int], Optional[int], Optional[bool]]:
        science = d(path(frag, f"{prefix}.science"))
        residual = d(path(frag, f"{prefix}.pps_residual"))
        ns = first_int(path(frag, f"{prefix}.ns"), path(frag, f"gnss.{prefix}_ns"))
        interval = first_int(
            residual.get("clock_interval_ns"),
            science.get("clock_interval_ns"),
        )
        fast = first_int(
            residual.get("fast_residual_ns"),
            science.get("fast_residual_ns"),
        )
        valid = b(residual.get("valid"))
        if valid is None:
            valid = b(science.get("valid"))
        return ns, interval, fast, valid

    o1_ns, o1_int, o1_res, o1_valid = ocxo("ocxo1")
    o2_ns, o2_int, o2_res, o2_valid = ocxo("ocxo2")
    row_science_eligible, row_science_excluded, row_science_reason = (
        science_disposition(payload, frag)
    )

    reason = str(frag.get("recover_reattach_reason") or "").strip().lower()
    recovery_evidence = bool(
        reason not in {"", "idle", "none", "ok"}
        or b(frag.get("recover_transition_active")) is True
        or b(frag.get("recover_degraded_active")) is True
        or b(frag.get("recover_science_quarantine_active")) is True
    )
    recovery_hold = bool(
        b(frag.get("recover_degraded_science_hold")) is True
        or b(frag.get("recover_science_quarantine_active")) is True
        or (
            b(frag.get("recover_degraded_active")) is True
            and b(frag.get("recover_science_ready")) is not True
        )
    )

    welford_n: Dict[str, Optional[int]] = {}
    welford_sd: Dict[str, Optional[float]] = {}
    for lane in ("dwt", "vclock", "ocxo1", "ocxo2"):
        welford_n[lane] = first_int(
            path(frag, f"stats.{lane}.welford.n"),
            path(frag, f"stats.{lane}.n"),
            frag.get(f"{lane}_welford_n"),
        )
        welford_sd[lane] = first_float(
            path(frag, f"stats.{lane}.welford.stddev"),
            path(frag, f"stats.{lane}.stddev"),
            frag.get(f"{lane}_welford_stddev"),
        )

    return CompactRow(
        db_id=db_id,
        pps=db_pps,
        ts=ts,
        campaign=str(payload.get("campaign") or frag.get("campaign") or ""),
        gnss_ns=first_int(path(frag, "gnss.ns"), frag.get("gnss_ns")),
        vclock_ns=first_int(path(frag, "vclock.ns"), path(frag, "gnss.ns")),
        dwt_cycles=first_int(
            path(frag, "dwt.cycle_count_total"),
            path(frag, "dwt.cycles"),
            frag.get("dwt_cycle_count_total"),
        ),
        ocxo1_ns=o1_ns,
        ocxo2_ns=o2_ns,
        ocxo1_interval_ns=o1_int,
        ocxo2_interval_ns=o2_int,
        ocxo1_residual_ns=o1_res,
        ocxo2_residual_ns=o2_res,
        ocxo1_valid=o1_valid,
        ocxo2_valid=o2_valid,
        dwt_ppb=first_float(path(frag, "stats.dwt.ppb"), frag.get("dwt_ppb")),
        vclock_ppb=first_float(path(frag, "stats.vclock.ppb"), frag.get("vclock_ppb")),
        ocxo1_ppb=first_float(path(frag, "stats.ocxo1.ppb"), frag.get("ocxo1_ppb")),
        ocxo2_ppb=first_float(path(frag, "stats.ocxo2.ppb"), frag.get("ocxo2_ppb")),
        science_eligible=row_science_eligible,
        science_excluded=row_science_excluded,
        science_reason=row_science_reason,
        recovery_evidence=recovery_evidence,
        recovery_hold=recovery_hold,
        timeline_ready=b(frag.get("recover_timeline_ready")),
        clockface_ready=b(frag.get("recover_clockface_ready")),
        science_ready=b(frag.get("recover_science_ready")),
        welford_n=welford_n,
        welford_sd=welford_sd,
        servo_mode=str(
            path(frag, "dac.servo_mode")
            or frag.get("servo_mode")
            or payload.get("servo_mode")
            or "?"
        ),
        location=str(payload.get("location") or "?"),
        environment=d(payload.get("environment")),
        final_forensics=forensic,
    )


def update(audit: Audit, prev: Optional[CompactRow], cur: CompactRow) -> None:
    audit.rows += 1
    if cur.science_eligible:
        audit.science_rows += 1
    else:
        audit.science_excluded_rows += 1
        audit.note("Science excluded", f"pps={cur.pps} reason={cur.science_reason}")
    audit.first_pps = cur.pps if audit.first_pps is None else audit.first_pps
    audit.last_pps = cur.pps
    audit.first_ts = cur.ts if not audit.first_ts else audit.first_ts
    audit.last_ts = cur.ts
    audit.final_row = cur

    if cur.gnss_ns is not None and cur.gnss_ns != cur.pps * NS_PER_SECOND:
        audit.gnss_identity_errors += 1
        audit.note("GNSS identity", f"pps={cur.pps} gnss={cur.gnss_ns}")

    for key, value in (
        ("dwt_ppb", cur.dwt_ppb),
        ("vclock_ppb", cur.vclock_ppb),
        ("ocxo1_ppb", cur.ocxo1_ppb),
        ("ocxo2_ppb", cur.ocxo2_ppb),
    ):
        if value is not None and cur.science_eligible:
            audit.stats[key].update(value)
            if abs(value) > PPB_ABSOLUTE_ALARM:
                audit.ppb_alarms += 1
                audit.note("PPB absolute", f"pps={cur.pps} {key}={value:+.3f}")

    for lane, interval, residual, valid in (
        ("ocxo1", cur.ocxo1_interval_ns, cur.ocxo1_residual_ns, cur.ocxo1_valid),
        ("ocxo2", cur.ocxo2_interval_ns, cur.ocxo2_residual_ns, cur.ocxo2_valid),
    ):
        if interval is not None and valid is True and cur.science_eligible:
            audit.stats[f"{lane}_interval"].update(float(interval))
            deviation = interval - NS_PER_SECOND
            if abs(deviation) > OCXO_SECOND_ALARM_NS:
                setattr(audit, f"{lane}_interval_alarms",
                        getattr(audit, f"{lane}_interval_alarms") + 1)
                audit.note(
                    f"{lane.upper()} interval",
                    f"pps={cur.pps} interval={interval} deviation={deviation:+d} valid={valid}",
                )
        if residual is not None and valid is True and cur.science_eligible:
            audit.stats[f"{lane}_residual"].update(float(residual))

    if (
        cur.ocxo1_residual_ns is not None
        and cur.ocxo2_residual_ns is not None
        and cur.ocxo1_valid is True
        and cur.ocxo2_valid is True
        and cur.science_eligible
    ):
        audit.stats["ocxo_divergence"].update(
            float(cur.ocxo1_residual_ns - cur.ocxo2_residual_ns)
        )

    if prev is None:
        return

    delta_pps = cur.pps - prev.pps
    if delta_pps == 0:
        audit.repeats += 1
        audit.note("PPS repeat", f"pps={cur.pps}")
    elif delta_pps < 0:
        audit.regressions += 1
        audit.note("PPS regression", f"{prev.pps}->{cur.pps}")
    elif delta_pps > 1:
        audit.gaps += 1
        if cur.recovery_evidence:
            audit.recovery_gaps += 1
        else:
            audit.unclassified_gaps += 1
            audit.note("Unclassified gap", f"{prev.pps}->{cur.pps}")

    if delta_pps != 1:
        return

    if prev.gnss_ns is not None and cur.gnss_ns is not None:
        if cur.gnss_ns - prev.gnss_ns != NS_PER_SECOND:
            audit.gnss_adjacent_errors += 1
            audit.note(
                "GNSS adjacent",
                f"pps={cur.pps} delta={cur.gnss_ns - prev.gnss_ns}",
            )

    if prev.dwt_cycles is not None and cur.dwt_cycles is not None:
        dwt_delta = cur.dwt_cycles - prev.dwt_cycles
        if prev.science_eligible and cur.science_eligible:
            audit.stats["dwt_delta"].update(float(dwt_delta))
        if dwt_delta <= 0:
            audit.dwt_nonmonotonic += 1
            audit.note("DWT nonmonotonic", f"pps={cur.pps} delta={dwt_delta}")

    for lane in ("ocxo1", "ocxo2"):
        prev_ns = getattr(prev, f"{lane}_ns")
        cur_ns = getattr(cur, f"{lane}_ns")
        interval = getattr(cur, f"{lane}_interval_ns")
        if prev_ns is not None and cur_ns is not None and interval is not None:
            error = (cur_ns - prev_ns) - interval
            if (
                abs(error) > OCXO_SECOND_WARN_NS
                and prev.science_eligible
                and cur.science_eligible
                and getattr(prev, f"{lane}_valid") is True
                and getattr(cur, f"{lane}_valid") is True
            ):
                setattr(audit, f"{lane}_ledger_errors",
                        getattr(audit, f"{lane}_ledger_errors") + 1)
                audit.note(
                    f"{lane.upper()} ledger",
                    f"pps={cur.pps} ledger_delta={cur_ns - prev_ns} interval={interval} error={error:+d}",
                )

    for lane in ("dwt", "vclock", "ocxo1", "ocxo2"):
        pn = prev.welford_n.get(lane)
        cn = cur.welford_n.get(lane)
        if pn is None or cn is None:
            continue
        if cn < pn:
            audit.welford_regressions += 1
            audit.note("Welford regression", f"pps={cur.pps} {lane} {pn}->{cn}")
        if cur.science_excluded and cn != pn:
            audit.welford_hold_changes += 1
            audit.note("Welford changed while science excluded",
                       f"pps={cur.pps} {lane} {pn}->{cn}")

    for lane in ("dwt", "vclock", "ocxo1", "ocxo2"):
        pv = getattr(prev, f"{lane}_ppb")
        cv = getattr(cur, f"{lane}_ppb")
        if (
            prev.science_eligible
            and cur.science_eligible
            and pv is not None
            and cv is not None
            and abs(cv - pv) > PPB_STEP_ALARM
        ):
            audit.ppb_steps += 1
            audit.note("PPB step", f"pps={cur.pps} {lane} {pv:+.3f}->{cv:+.3f}")


def print_report(audit: Audit, batch_size: int, pause_ms: int) -> None:
    if audit.rows == 0:
        print(f"No TIMEBASE rows found for campaign {audit.campaign!r}")
        return

    assert audit.first_pps is not None and audit.last_pps is not None
    audit.expected_rows = audit.last_pps - audit.first_pps + 1
    final = audit.final_row
    assert final is not None

    print("=" * 78)
    print(f"CAMPAIGN ANALYSIS: {audit.campaign}")
    print("=" * 78)
    print(f"  Streaming policy:  named cursor, batch={batch_size}, pause={pause_ms} ms")
    print(f"  Database ordering: timebase.pps_count, id")
    print(f"  Time range:        {audit.first_ts} -> {audit.last_ts}")
    print(f"  PPS range:         {audit.first_pps} -> {audit.last_pps}")
    print(f"  Actual rows:       {audit.rows:,}")
    print(f"  Science rows:      {audit.science_rows:,}")
    print(f"  Science excluded:  {audit.science_excluded_rows:,}")
    print(f"  Expected rows:     {audit.expected_rows:,}")
    print(f"  Missing identities:{audit.expected_rows - audit.rows:>10,d}")
    print(f"  Recovery gaps:     {audit.recovery_gaps:,}")
    print(f"  Unclassified gaps: {audit.unclassified_gaps:,}")
    print(f"  Repeats/regress:   {audit.repeats:,}/{audit.regressions:,}")
    print(f"  Final servo mode:  {final.servo_mode}")
    print(f"  Location:          {final.location}")

    print("\nONLINE STATISTICS (SCIENCE-ELIGIBLE ROWS)")
    for key in (
        "dwt_delta",
        "ocxo1_interval",
        "ocxo2_interval",
        "ocxo1_residual",
        "ocxo2_residual",
        "ocxo_divergence",
        "dwt_ppb",
        "vclock_ppb",
        "ocxo1_ppb",
        "ocxo2_ppb",
    ):
        print(f"  {key:<18s} {audit.stats[key].text(6 if 'ppb' in key else 3)}")

    print("\nCOURTS")
    print("  Science courts use only science-eligible rows; timeline courts use all rows.")
    courts = [
        ("GNSS identity errors", audit.gnss_identity_errors),
        ("GNSS adjacent errors", audit.gnss_adjacent_errors),
        ("DWT non-monotonic", audit.dwt_nonmonotonic),
        ("OCXO1 interval alarms", audit.ocxo1_interval_alarms),
        ("OCXO2 interval alarms", audit.ocxo2_interval_alarms),
        ("OCXO1 ledger errors", audit.ocxo1_ledger_errors),
        ("OCXO2 ledger errors", audit.ocxo2_ledger_errors),
        ("PPB absolute alarms", audit.ppb_alarms),
        ("PPB steps", audit.ppb_steps),
        ("Welford regressions", audit.welford_regressions),
        ("Welford hold changes", audit.welford_hold_changes),
    ]
    total = 0
    for label, count in courts:
        total += count
        print(f"  {label:<28s} {count:,}")

    if audit.examples:
        print("\nBOUNDED EXAMPLES")
        for kind, examples in audit.examples.items():
            print(f"  [{kind}]")
            for example in examples:
                print(f"    {example}")

    print("\nFINAL CONTEXT")
    if final.environment:
        print(f"  Environment keys: {', '.join(sorted(final.environment.keys()))}")
    else:
        print("  Environment: no data")
    if final.final_forensics:
        print(
            "  Forensics: "
            f"schema={final.final_forensics.get('schema') or final.final_forensics.get('micro_schema') or '?'}"
        )
    else:
        print("  Forensics: no data")

    print()
    if total or audit.unclassified_gaps or audit.repeats or audit.regressions:
        print("VERDICT: ANOMALIES FOUND IN ADMITTED SCIENCE OR TIMELINE")
    elif audit.recovery_gaps or audit.science_excluded_rows:
        details = []
        if audit.recovery_gaps:
            details.append(f"{audit.recovery_gaps} RECOVERY GAP(S)")
        if audit.science_excluded_rows:
            details.append(f"{audit.science_excluded_rows} QUARANTINED ROW(S)")
        print(f"VERDICT: SCIENCE CLEAN WITH {' AND '.join(details)}")
    else:
        print("VERDICT: CLEAN")


def list_campaigns() -> None:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT t.campaign,
                   bool_or(c.active) AS active,
                   count(*) AS tb_count,
                   min(t.pps_count) AS pps_min,
                   max(t.pps_count) AS pps_max,
                   count(*) FILTER (WHERE t.pps_count IS NULL) AS pps_missing,
                   max(t.ts) AS last_ts
            FROM timebase t
            LEFT JOIN campaigns c USING (campaign)
            GROUP BY t.campaign
            ORDER BY max(t.ts) DESC
            LIMIT 20
            """
        )
        rows = cur.fetchall()

    print("Available campaigns:")
    print(f"  {'CAMPAIGN':<20s} {'ACTIVE':>7s} {'RECORDS':>9s} {'PPS RANGE':>22s}")
    for row in rows:
        missing = int(row.get("pps_missing") or 0)
        range_text = (
            f"UNINDEXED:{missing}"
            if missing
            else f"{row.get('pps_min')}-{row.get('pps_max')}"
        )
        print(
            f"  {row['campaign']:<20s} "
            f"{('>' if row['active'] else ''):>7s} "
            f"{int(row['tb_count']):>9,d} {range_text:>22s}"
        )


def parse(argv: Sequence[str]) -> tuple[str, int, int, int]:
    if len(argv) < 2:
        list_campaigns()
        raise SystemExit(1)

    campaign = argv[1]
    batch_size = DEFAULT_BATCH_SIZE
    pause_ms = DEFAULT_PAUSE_MS
    limit = 0

    idx = 2
    while idx < len(argv):
        arg = argv[idx]
        if arg in {"--batch-size", "--pause-ms", "--limit"}:
            if idx + 1 >= len(argv):
                raise SystemExit(f"{arg} requires a value")
            value = int(argv[idx + 1])
            idx += 2
            if arg == "--batch-size":
                batch_size = value
            elif arg == "--pause-ms":
                pause_ms = value
            else:
                limit = value
        elif "=" in arg:
            key, value = arg.split("=", 1)
            if key == "--batch-size":
                batch_size = int(value)
            elif key == "--pause-ms":
                pause_ms = int(value)
            elif key == "--limit":
                limit = int(value)
            else:
                raise SystemExit(f"unknown option: {key}")
            idx += 1
        else:
            raise SystemExit(f"unknown argument: {arg}")

    if batch_size <= 0 or pause_ms < 0 or limit < 0:
        raise SystemExit("batch-size must be positive; pause-ms and limit nonnegative")
    return campaign, batch_size, pause_ms, limit


def main(argv: Sequence[str]) -> None:
    campaign, batch_size, pause_ms, limit = parse(argv)
    audit = Audit(campaign=campaign)
    previous: Optional[CompactRow] = None

    for db_id, db_pps, ts, payload in iter_rows(
        campaign,
        batch_size=batch_size,
        pause_ms=pause_ms,
        limit=limit,
    ):
        row = compact(db_id, db_pps, ts, payload)
        update(audit, previous, row)
        previous = row

    print_report(audit, batch_size, pause_ms)


if __name__ == "__main__":
    main(sys.argv)
