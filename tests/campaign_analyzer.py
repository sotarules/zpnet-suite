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
from collections import Counter
from dataclasses import dataclass, field
from typing import Any, Dict, Iterator, Optional, Sequence, Tuple

from zpnet.shared.db import open_db

NS_PER_SECOND = 1_000_000_000
DWT_EXPECTED_PER_PPS = 1_008_000_000
DEFAULT_BATCH_SIZE = 8
DEFAULT_PAUSE_MS = 50
MAX_EXAMPLES = 12
OCXO_SCIENCE_OUTLIER_NS = 50
OCXO_SECOND_WARN_NS = 500
OCXO_SECOND_ALARM_NS = 10_000
PPB_ABSOLUTE_ALARM = 10_000.0
PPB_STEP_ALARM = 100.0
SNAPSHOT_LANES = ("pps", "vclock", "ocxo1", "ocxo2")


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

    min_pps: Optional[int] = None
    max_pps: Optional[int] = None

    def update(self, value: float, pps: Optional[int] = None) -> None:
        self.n += 1
        delta = value - self.mean
        self.mean += delta / self.n
        self.m2 += delta * (value - self.mean)
        if value < self.min_value:
            self.min_value = value
            self.min_pps = pps
        if value > self.max_value:
            self.max_value = value
            self.max_pps = pps

    @property
    def stddev(self) -> float:
        return math.sqrt(self.m2 / (self.n - 1)) if self.n >= 2 else 0.0

    def text(self, digits: int = 3) -> str:
        if self.n == 0:
            return "no data"
        return (
            f"n={self.n:,} mean={self.mean:+.{digits}f} "
            f"sd={self.stddev:.{digits}f} "
            f"min={self.min_value:+.{digits}f}@{self.min_pps} "
            f"max={self.max_value:+.{digits}f}@{self.max_pps}"
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
    snapshot_contract_present: bool
    snapshot_contract_missing: Tuple[str, ...]
    stats_snapshot_ok: Optional[bool]
    stats_valid: Optional[bool]
    stats_completed_row_coherent: Optional[bool]
    raw_snapshot_ok: Dict[str, Optional[bool]]
    raw_valid: Dict[str, Optional[bool]]
    forensics_snapshot_ok: Dict[str, Optional[bool]]
    delay_detail_present: Dict[str, Optional[bool]]
    delay_status: Dict[str, str]
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
    gnss_identity_offsets_ns: Counter[int] = field(default_factory=Counter)
    dwt_nonmonotonic: int = 0
    ocxo1_interval_alarms: int = 0
    ocxo2_interval_alarms: int = 0
    ocxo1_science_outliers: int = 0
    ocxo2_science_outliers: int = 0
    divergence_science_outliers: int = 0
    ocxo1_ledger_errors: int = 0
    ocxo2_ledger_errors: int = 0
    ppb_alarms: int = 0
    ppb_steps: int = 0
    welford_regressions: int = 0
    welford_hold_changes: int = 0
    snapshot_contract_rows: int = 0
    snapshot_clean_rows: int = 0
    snapshot_legacy_rows: int = 0
    first_snapshot_contract_pps: Optional[int] = None
    last_snapshot_contract_pps: Optional[int] = None
    snapshot_missing_after_activation: int = 0
    snapshot_contract_omissions: Counter[str] = field(default_factory=Counter)
    snapshot_contract_violations: int = 0
    snapshot_contract_violation_rows: int = 0
    snapshot_failure_rows: int = 0
    snapshot_failure_science_rows: int = 0
    stats_snapshot_failures: int = 0
    prediction_snapshot_failures: Counter[str] = field(default_factory=Counter)
    forensics_snapshot_failures: Counter[str] = field(default_factory=Counter)
    coherent_immature_snapshots: Counter[str] = field(default_factory=Counter)
    coherent_immature_rows: int = 0
    last_coherent_immature_pps: Optional[int] = None
    science_event_rows: int = 0
    last_science_event_pps: Optional[int] = None
    examples: Dict[str, list[str]] = field(default_factory=dict)
    missing_identities: int = 0
    recovery_missing_identities: int = 0
    unclassified_missing_identities: int = 0
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

    def note_science_event(self, pps: int) -> None:
        """Count one anomalous PPS once even when several courts fire."""
        if self.last_science_event_pps != pps:
            self.science_event_rows += 1
            self.last_science_event_pps = pps


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

    stats_obj = d(frag.get("stats"))
    raw_cycles = d(frag.get("raw_cycles"))
    raw_lanes = {lane: d(raw_cycles.get(lane)) for lane in SNAPSHOT_LANES}

    stats_snapshot_ok = b(stats_obj.get("snapshot_ok"))
    stats_valid = b(stats_obj.get("valid"))
    stats_completed_row_coherent = b(stats_obj.get("completed_row_coherent"))
    raw_snapshot_ok = {
        lane: b(raw_lanes[lane].get("snapshot_ok")) for lane in SNAPSHOT_LANES
    }
    raw_valid = {
        lane: b(raw_lanes[lane].get("valid")) for lane in SNAPSHOT_LANES
    }
    forensics_snapshot_ok = {
        lane: b(raw_lanes[lane].get("forensics_snapshot_ok"))
        for lane in SNAPSHOT_LANES
    }
    delay_detail_present = {
        lane: b(raw_lanes[lane].get("delay_detail_present"))
        for lane in SNAPSHOT_LANES
    }
    delay_status = {
        lane: str(raw_lanes[lane].get("delay_status") or "").strip().upper()
        for lane in SNAPSHOT_LANES
    }

    snapshot_contract_present = bool(
        "snapshot_ok" in stats_obj
        or any(
            "snapshot_ok" in raw_lanes[lane]
            or "forensics_snapshot_ok" in raw_lanes[lane]
            or "delay_detail_present" in raw_lanes[lane]
            for lane in SNAPSHOT_LANES
        )
    )
    snapshot_contract_missing: list[str] = []
    if snapshot_contract_present:
        for name, value in (
            ("stats.snapshot_ok", stats_snapshot_ok),
            ("stats.valid", stats_valid),
            ("stats.completed_row_coherent", stats_completed_row_coherent),
        ):
            if value is None:
                snapshot_contract_missing.append(name)
        for lane in SNAPSHOT_LANES:
            for name, value in (
                (f"raw_cycles.{lane}.snapshot_ok", raw_snapshot_ok[lane]),
                (f"raw_cycles.{lane}.valid", raw_valid[lane]),
                (
                    f"raw_cycles.{lane}.forensics_snapshot_ok",
                    forensics_snapshot_ok[lane],
                ),
                (
                    f"raw_cycles.{lane}.delay_detail_present",
                    delay_detail_present[lane],
                ),
            ):
                if value is None:
                    snapshot_contract_missing.append(name)
            if not delay_status[lane]:
                snapshot_contract_missing.append(f"raw_cycles.{lane}.delay_status")

    # Legacy rows retain historical behavior.  Once the explicit contract is
    # present, statistics are admissible only from a coherent, valid completed
    # statistics row.
    stats_usable = (
        stats_valid is not False
        if not snapshot_contract_present
        else (
            stats_snapshot_ok is True
            and stats_valid is True
            and stats_completed_row_coherent is True
        )
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
        welford_n[lane] = (
            first_int(
                path(frag, f"stats.{lane}.welford.n"),
                path(frag, f"stats.{lane}.n"),
                frag.get(f"{lane}_welford_n"),
            )
            if stats_usable
            else None
        )
        welford_sd[lane] = (
            first_float(
                path(frag, f"stats.{lane}.welford.stddev"),
                path(frag, f"stats.{lane}.stddev"),
                frag.get(f"{lane}_welford_stddev"),
            )
            if stats_usable
            else None
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
        dwt_ppb=(
            first_float(path(frag, "stats.dwt.ppb"), frag.get("dwt_ppb"))
            if stats_usable
            else None
        ),
        vclock_ppb=(
            first_float(path(frag, "stats.vclock.ppb"), frag.get("vclock_ppb"))
            if stats_usable
            else None
        ),
        ocxo1_ppb=(
            first_float(path(frag, "stats.ocxo1.ppb"), frag.get("ocxo1_ppb"))
            if stats_usable
            else None
        ),
        ocxo2_ppb=(
            first_float(path(frag, "stats.ocxo2.ppb"), frag.get("ocxo2_ppb"))
            if stats_usable
            else None
        ),
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
        snapshot_contract_present=snapshot_contract_present,
        snapshot_contract_missing=tuple(snapshot_contract_missing),
        stats_snapshot_ok=stats_snapshot_ok,
        stats_valid=stats_valid,
        stats_completed_row_coherent=stats_completed_row_coherent,
        raw_snapshot_ok=raw_snapshot_ok,
        raw_valid=raw_valid,
        forensics_snapshot_ok=forensics_snapshot_ok,
        delay_detail_present=delay_detail_present,
        delay_status=delay_status,
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


def update_snapshot_contract(audit: Audit, cur: CompactRow) -> None:
    """Audit explicit snapshot acquisition separately from scientific validity."""
    if not cur.snapshot_contract_present:
        if audit.first_snapshot_contract_pps is None:
            audit.snapshot_legacy_rows += 1
        else:
            audit.snapshot_missing_after_activation += 1
            audit.note(
                "Snapshot contract missing",
                f"pps={cur.pps} contract absent after "
                f"first_pps={audit.first_snapshot_contract_pps}",
            )
        return

    audit.snapshot_contract_rows += 1
    if audit.first_snapshot_contract_pps is None:
        audit.first_snapshot_contract_pps = cur.pps
    audit.last_snapshot_contract_pps = cur.pps

    row_has_omission = bool(cur.snapshot_contract_missing)
    for field_name in cur.snapshot_contract_missing:
        audit.snapshot_contract_omissions[field_name] += 1
        audit.note(
            "Snapshot field missing/invalid",
            f"pps={cur.pps} field={field_name}",
        )

    row_failed = False
    row_immature = False
    if cur.stats_snapshot_ok is False:
        audit.stats_snapshot_failures += 1
        row_failed = True
        audit.note("Statistics snapshot unavailable", f"pps={cur.pps}")
    elif cur.stats_snapshot_ok is True and (
        cur.stats_valid is False or cur.stats_completed_row_coherent is False
    ):
        audit.coherent_immature_snapshots["stats"] += 1
        row_immature = True
        audit.note(
            "Coherent but immature snapshot",
            f"pps={cur.pps} scope=stats valid={cur.stats_valid} "
            f"completed_row_coherent={cur.stats_completed_row_coherent}",
        )

    contradictions: list[str] = []
    if cur.stats_snapshot_ok is False and cur.stats_valid is True:
        contradictions.append("stats snapshot_ok=false but valid=true")
    if (
        cur.stats_snapshot_ok is False
        and cur.stats_completed_row_coherent is True
    ):
        contradictions.append(
            "stats snapshot_ok=false but completed_row_coherent=true"
        )

    for lane in SNAPSHOT_LANES:
        if cur.raw_snapshot_ok[lane] is False:
            audit.prediction_snapshot_failures[lane] += 1
            row_failed = True
            audit.note(
                "Prediction snapshot unavailable",
                f"pps={cur.pps} lane={lane}",
            )
        elif cur.raw_snapshot_ok[lane] is True and cur.raw_valid[lane] is False:
            audit.coherent_immature_snapshots[lane] += 1
            row_immature = True
            audit.note(
                "Coherent but immature snapshot",
                f"pps={cur.pps} scope=prediction lane={lane} valid=false",
            )

        if cur.forensics_snapshot_ok[lane] is False:
            audit.forensics_snapshot_failures[lane] += 1
            row_failed = True
            audit.note(
                "Forensics snapshot unavailable",
                f"pps={cur.pps} lane={lane}",
            )

        if cur.raw_snapshot_ok[lane] is False and cur.raw_valid[lane] is True:
            contradictions.append(
                f"{lane} snapshot_ok=false but valid=true"
            )
        if (
            cur.forensics_snapshot_ok[lane] is False
            and cur.delay_detail_present[lane] is True
        ):
            contradictions.append(
                f"{lane} forensics_snapshot_ok=false but delay_detail_present=true"
            )
        if (
            cur.forensics_snapshot_ok[lane] is False
            and cur.delay_status[lane]
            and cur.delay_status[lane] != "SNAPSHOT_UNAVAILABLE"
        ):
            contradictions.append(
                f"{lane} forensics_snapshot_ok=false but delay_status={cur.delay_status[lane]}"
            )

    if contradictions:
        audit.snapshot_contract_violation_rows += 1
    for detail in contradictions:
        audit.snapshot_contract_violations += 1
        audit.note("Snapshot contract violation", f"pps={cur.pps} {detail}")

    if row_immature and audit.last_coherent_immature_pps != cur.pps:
        audit.coherent_immature_rows += 1
        audit.last_coherent_immature_pps = cur.pps

    if not row_failed and not row_has_omission and not contradictions:
        audit.snapshot_clean_rows += 1

    if row_failed:
        audit.snapshot_failure_rows += 1
        if cur.science_eligible:
            audit.snapshot_failure_science_rows += 1


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
    update_snapshot_contract(audit, cur)

    if cur.gnss_ns is not None:
        identity_offset = cur.gnss_ns - cur.pps * NS_PER_SECOND
        audit.gnss_identity_offsets_ns[identity_offset] += 1
        if identity_offset != 0:
            audit.gnss_identity_errors += 1
            audit.note(
                "GNSS identity",
                f"pps={cur.pps} gnss={cur.gnss_ns} offset_ns={identity_offset:+d}",
            )

    for key, value in (
        ("dwt_ppb", cur.dwt_ppb),
        ("vclock_ppb", cur.vclock_ppb),
        ("ocxo1_ppb", cur.ocxo1_ppb),
        ("ocxo2_ppb", cur.ocxo2_ppb),
    ):
        if value is not None and cur.science_eligible:
            audit.stats[key].update(value, cur.pps)
            if abs(value) > PPB_ABSOLUTE_ALARM:
                audit.ppb_alarms += 1
                audit.note_science_event(cur.pps)
                audit.note("PPB absolute", f"pps={cur.pps} {key}={value:+.3f}")

    for lane, interval, residual, valid in (
        ("ocxo1", cur.ocxo1_interval_ns, cur.ocxo1_residual_ns, cur.ocxo1_valid),
        ("ocxo2", cur.ocxo2_interval_ns, cur.ocxo2_residual_ns, cur.ocxo2_valid),
    ):
        if interval is not None and valid is True and cur.science_eligible:
            audit.stats[f"{lane}_interval"].update(float(interval), cur.pps)
            deviation = interval - NS_PER_SECOND
            if abs(deviation) > OCXO_SECOND_ALARM_NS:
                setattr(audit, f"{lane}_interval_alarms",
                        getattr(audit, f"{lane}_interval_alarms") + 1)
                audit.note_science_event(cur.pps)
                audit.note(
                    f"{lane.upper()} interval",
                    f"pps={cur.pps} interval={interval} deviation={deviation:+d} valid={valid}",
                )
        if residual is not None and valid is True and cur.science_eligible:
            audit.stats[f"{lane}_residual"].update(float(residual), cur.pps)

        if interval is not None and valid is True and cur.science_eligible:
            deviation = interval - NS_PER_SECOND
            if abs(deviation) > OCXO_SCIENCE_OUTLIER_NS:
                attr = f"{lane}_science_outliers"
                setattr(audit, attr, getattr(audit, attr) + 1)
                audit.note_science_event(cur.pps)
                previous_context = (
                    "none"
                    if prev is None
                    else (
                        f"pps={prev.pps} eligible={prev.science_eligible} "
                        f"excluded={prev.science_excluded} valid={getattr(prev, f'{lane}_valid')}"
                    )
                )
                audit.note(
                    f"Admitted {lane.upper()} interval outlier",
                    f"pps={cur.pps} interval={interval} deviation={deviation:+d} "
                    f"residual={residual} previous=[{previous_context}]",
                )

    if (
        cur.ocxo1_residual_ns is not None
        and cur.ocxo2_residual_ns is not None
        and cur.ocxo1_valid is True
        and cur.ocxo2_valid is True
        and cur.science_eligible
    ):
        divergence = cur.ocxo1_residual_ns - cur.ocxo2_residual_ns
        audit.stats["ocxo_divergence"].update(float(divergence), cur.pps)
        if abs(divergence) > OCXO_SCIENCE_OUTLIER_NS:
            audit.divergence_science_outliers += 1
            audit.note_science_event(cur.pps)
            audit.note(
                "Admitted OCXO divergence outlier",
                f"pps={cur.pps} divergence={divergence:+d} "
                f"ocxo1_residual={cur.ocxo1_residual_ns:+d} "
                f"ocxo2_residual={cur.ocxo2_residual_ns:+d}",
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
        missing = delta_pps - 1
        audit.gaps += 1
        audit.missing_identities += missing
        if cur.recovery_evidence:
            audit.recovery_gaps += 1
            audit.recovery_missing_identities += missing
            audit.note(
                "Recovery gap",
                f"{prev.pps}->{cur.pps} missing={missing} "
                f"timeline_ready={cur.timeline_ready} clockface_ready={cur.clockface_ready} "
                f"science_ready={cur.science_ready}",
            )
        else:
            audit.unclassified_gaps += 1
            audit.unclassified_missing_identities += missing
            audit.note("Unclassified gap", f"{prev.pps}->{cur.pps} missing={missing}")

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
            audit.stats["dwt_delta"].update(float(dwt_delta), cur.pps)
        if dwt_delta <= 0:
            audit.dwt_nonmonotonic += 1
            audit.note_science_event(cur.pps)
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
                audit.note_science_event(cur.pps)
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
            audit.note_science_event(cur.pps)
            audit.note("Welford regression", f"pps={cur.pps} {lane} {pn}->{cn}")
        if cur.science_excluded and cn != pn:
            audit.welford_hold_changes += 1
            audit.note_science_event(cur.pps)
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
            audit.note_science_event(cur.pps)
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
    computed_missing = audit.expected_rows - audit.rows
    print(f"  Missing identities:{computed_missing:>10,d}")
    print(f"    Recovery-classified:   {audit.recovery_missing_identities:>10,d}")
    print(f"    Unclassified:          {audit.unclassified_missing_identities:>10,d}")
    print(f"  Recovery gap ranges:     {audit.recovery_gaps:,}")
    print(f"  Unclassified gap ranges: {audit.unclassified_gaps:,}")
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

    print("\nSNAPSHOT CONTRACT")
    print(f"  Contract-bearing rows:        {audit.snapshot_contract_rows:,}")
    print(f"  Clean contract rows:          {audit.snapshot_clean_rows:,}")
    print(f"  Legacy rows before contract:  {audit.snapshot_legacy_rows:,}")
    first_contract = (
        str(audit.first_snapshot_contract_pps)
        if audit.first_snapshot_contract_pps is not None
        else "not observed"
    )
    last_contract = (
        str(audit.last_snapshot_contract_pps)
        if audit.last_snapshot_contract_pps is not None
        else "not observed"
    )
    print(f"  First contract PPS:           {first_contract}")
    print(f"  Last contract PPS:            {last_contract}")
    if audit.first_snapshot_contract_pps is not None:
        contract_expected = audit.last_pps - audit.first_snapshot_contract_pps + 1
        contract_coverage = (
            100.0 * audit.snapshot_clean_rows / contract_expected
            if contract_expected > 0
            else 0.0
        )
        print(f"  Expected identity span:       {contract_expected:,}")
        print(f"  Clean identity coverage:      {contract_coverage:.3f}%")
    print(
        f"  Missing after activation:     {audit.snapshot_missing_after_activation:,}"
    )

    omission_count = sum(audit.snapshot_contract_omissions.values())
    print("  Acquisition failures:")
    print(f"    Statistics                   {audit.stats_snapshot_failures:,}")
    print("    Prediction:")
    for lane in SNAPSHOT_LANES:
        print(f"      {lane.upper():<8s} {audit.prediction_snapshot_failures[lane]:,}")
    print("    Forensics:")
    for lane in SNAPSHOT_LANES:
        print(f"      {lane.upper():<8s} {audit.forensics_snapshot_failures[lane]:,}")
    print(f"    Failure-bearing PPS          {audit.snapshot_failure_rows:,}")
    print(
        f"    Science-admitted failure PPS: {audit.snapshot_failure_science_rows:>8,d}"
    )

    immature_count = sum(audit.coherent_immature_snapshots.values())
    print("  Coherent acquisition; scientific state immature:")
    print(f"    Unique PPS events            {audit.coherent_immature_rows:,}")
    print(f"    Scope/lane observations      {immature_count:,}")

    print("  Contract integrity:")
    print(f"    Missing/invalid fields       {omission_count:,}")
    print(f"    Violation-bearing PPS        {audit.snapshot_contract_violation_rows:,}")
    print(f"    Contract violations          {audit.snapshot_contract_violations:,}")

    print("\nCOURTS")
    print("  Science courts use only science-eligible rows; timeline courts use all rows.")
    courts = [
        ("GNSS identity errors", audit.gnss_identity_errors),
        ("GNSS adjacent errors", audit.gnss_adjacent_errors),
        ("DWT non-monotonic", audit.dwt_nonmonotonic),
        ("OCXO1 interval alarms", audit.ocxo1_interval_alarms),
        ("OCXO2 interval alarms", audit.ocxo2_interval_alarms),
        ("OCXO1 admitted outliers", audit.ocxo1_science_outliers),
        ("OCXO2 admitted outliers", audit.ocxo2_science_outliers),
        ("OCXO divergence outliers", audit.divergence_science_outliers),
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
    print(f"  {'Unique science-event PPS':<28s} {audit.science_event_rows:,}")

    print("\nGNSS IDENTITY DIAGNOSIS")
    if audit.gnss_identity_offsets_ns:
        ranked_offsets = audit.gnss_identity_offsets_ns.most_common(5)
        for offset_ns, count in ranked_offsets:
            seconds = offset_ns / NS_PER_SECOND
            print(
                f"  offset={offset_ns:+d} ns ({seconds:+.9f} s) rows={count:,}"
            )
        nonzero = [(offset, count) for offset, count in ranked_offsets if offset != 0]
        if audit.gnss_identity_errors and len(nonzero) == 1:
            offset, count = nonzero[0]
            if count == audit.gnss_identity_errors:
                print(
                    "  Classification: systematic fixed identity offset; "
                    "GNSS adjacency may remain lawful even though the PPS/GNSS contract disagrees."
                )
        elif audit.gnss_identity_errors:
            print("  Classification: mixed identity offsets; inspect bounded examples.")
        else:
            print("  Classification: identity contract clean.")
    else:
        print("  No GNSS identity data")

    if audit.examples:
        print("\nBOUNDED EXAMPLES")
        for kind, examples in audit.examples.items():
            print(f"  [{kind}]")
            for example in examples:
                print(f"    {example}")

    science_confidence = (
        100.0 * max(0, audit.science_rows - audit.science_event_rows) / audit.science_rows
        if audit.science_rows > 0
        else 0.0
    )
    continuity_confidence = (
        100.0
        * max(0, audit.expected_rows - audit.unclassified_missing_identities)
        / audit.expected_rows
        if audit.expected_rows > 0
        else 0.0
    )
    snapshot_confidence: Optional[float] = None
    if audit.first_snapshot_contract_pps is not None:
        contract_expected = audit.last_pps - audit.first_snapshot_contract_pps + 1
        snapshot_confidence = (
            100.0 * audit.snapshot_clean_rows / contract_expected
            if contract_expected > 0
            else 0.0
        )

    contract_integrity = (
        audit.snapshot_missing_after_activation == 0
        and not audit.snapshot_contract_omissions
        and audit.snapshot_contract_violations == 0
    )
    statistics_integrity = (
        audit.welford_regressions == 0
        and audit.welford_hold_changes == 0
    )
    quarantine_integrity = (
        audit.snapshot_failure_science_rows == 0
        and audit.welford_hold_changes == 0
    )
    confidence_components = [science_confidence, continuity_confidence]
    if snapshot_confidence is not None:
        confidence_components.append(snapshot_confidence)
    campaign_confidence = min(confidence_components) if confidence_components else 0.0
    if not contract_integrity or not statistics_integrity or not quarantine_integrity:
        campaign_confidence = 0.0

    print("\nCAMPAIGN CONFIDENCE (DETERMINISTIC FLOOR)")
    print(f"  Science custody:             {science_confidence:8.3f}%")
    if snapshot_confidence is None:
        print("  Snapshot custody:                 N/A (legacy)")
    else:
        print(f"  Snapshot custody:            {snapshot_confidence:8.3f}%")
    print(f"  Timeline continuity:         {continuity_confidence:8.3f}%")
    print(
        f"  Contract integrity:          {'PASS' if contract_integrity else 'FAIL'}"
    )
    print(
        f"  Statistics integrity:        {'PASS' if statistics_integrity else 'FAIL'}"
    )
    print(
        f"  Quarantine integrity:        {'PASS' if quarantine_integrity else 'FAIL'}"
    )
    print(f"  Campaign confidence:         {campaign_confidence:8.3f}%")
    print("  Rule: lowest measurable custody percentage; integrity failure forces 0%")

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

    print("\nVERDICTS")
    science_anomalies = sum((
        audit.dwt_nonmonotonic,
        audit.ocxo1_interval_alarms,
        audit.ocxo2_interval_alarms,
        audit.ocxo1_science_outliers,
        audit.ocxo2_science_outliers,
        audit.divergence_science_outliers,
        audit.ocxo1_ledger_errors,
        audit.ocxo2_ledger_errors,
        audit.ppb_alarms,
        audit.ppb_steps,
        audit.welford_regressions,
        audit.welford_hold_changes,
    ))
    timeline_anomalies = sum((
        audit.gnss_identity_errors,
        audit.gnss_adjacent_errors,
        audit.repeats,
        audit.regressions,
        audit.unclassified_gaps,
    ))
    snapshot_acquisition_failures = (
        audit.stats_snapshot_failures
        + sum(audit.prediction_snapshot_failures.values())
        + sum(audit.forensics_snapshot_failures.values())
    )
    snapshot_contract_errors = (
        audit.snapshot_missing_after_activation
        + sum(audit.snapshot_contract_omissions.values())
        + audit.snapshot_contract_violations
    )

    if audit.snapshot_contract_rows == 0:
        print("  SNAPSHOTS:  LEGACY CAMPAIGN; EXPLICIT CONTRACT NOT OBSERVED")
    elif snapshot_contract_errors:
        print(
            "  SNAPSHOTS:  CONTRACT VIOLATION "
            f"({snapshot_contract_errors:,} finding(s))"
        )
    elif snapshot_acquisition_failures:
        print(
            "  SNAPSHOTS:  ACQUISITION FAILURES OBSERVED "
            f"({snapshot_acquisition_failures:,} failure(s) across "
            f"{audit.snapshot_failure_rows:,} row(s)); unavailable values withheld"
        )
    else:
        print("  SNAPSHOTS:  CLEAN")

    if science_anomalies:
        print(
            "  SCIENCE:    ANOMALIES IN ADMITTED SCIENCE "
            f"({audit.science_event_rows:,} unique PPS event(s), "
            f"{science_anomalies:,} court finding(s))"
        )
    else:
        print("  SCIENCE:    CLEAN")

    if timeline_anomalies:
        fixed_nonzero = [
            (offset, count)
            for offset, count in audit.gnss_identity_offsets_ns.items()
            if offset != 0
        ]
        systematic_identity = (
            audit.gnss_identity_errors > 0
            and len(fixed_nonzero) == 1
            and fixed_nonzero[0][1] == audit.gnss_identity_errors
            and audit.gnss_adjacent_errors == 0
            and audit.repeats == 0
            and audit.regressions == 0
            and audit.unclassified_gaps == 0
        )
        if systematic_identity:
            offset, _ = fixed_nonzero[0]
            print(
                "  TIMELINE:   SYSTEMATIC GNSS IDENTITY CONTRACT OFFSET "
                f"({offset:+d} ns); adjacency and ordering remain clean"
            )
        else:
            print(f"  TIMELINE:   ANOMALIES ({timeline_anomalies:,} court finding(s))")
    else:
        print("  TIMELINE:   CLEAN")

    if audit.unclassified_missing_identities:
        print(
            "  CONTINUITY: UNEXPLAINED LOSS "
            f"({audit.unclassified_missing_identities:,} missing identity/identities)"
        )
    elif audit.recovery_missing_identities:
        print(
            "  CONTINUITY: ALL MISSING IDENTITIES CLASSIFIED AS RECOVERY DOWNTIME "
            f"({audit.recovery_missing_identities:,} across {audit.recovery_gaps} range(s))"
        )
    else:
        print("  CONTINUITY: COMPLETE")

    if audit.science_excluded_rows:
        print(
            "  QUARANTINE: ACTIVE AND EFFECTIVE "
            f"({audit.science_excluded_rows:,} row(s) retained for audit and excluded)"
        )
    else:
        print("  QUARANTINE: NO EXCLUSIONS")

    if snapshot_contract_errors:
        print("  OVERALL:    SNAPSHOT CONTRACT INVESTIGATION REQUIRED")
    elif science_anomalies:
        print("  OVERALL:    SCIENCE INVESTIGATION REQUIRED")
    elif timeline_anomalies:
        print("  OVERALL:    SCIENCE CLEAN; TIMELINE CONTRACT REQUIRES RECONCILIATION")
    elif (
        snapshot_acquisition_failures
        or audit.recovery_missing_identities
        or audit.science_excluded_rows
    ):
        print(
            "  OVERALL:    SCIENCE CLEAN WITH TRUTHFUL "
            "SNAPSHOT/RECOVERY/QUARANTINE EVENTS"
        )
    else:
        print("  OVERALL:    CLEAN")


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
