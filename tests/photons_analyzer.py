"""ZPNet photons_analyzer — proof-ledger audit for canonical PHOTONS_V1 / LANTERN.

PHOTONS is simpler than CLOCKS: the canonical row already carries the physical
instrument state, accepted-science populations, producer Better-Buckets proof,
Pi literal resurrection custody, explicit recovery ancestry, and the LANTERN
campaign projection.

The analyzer never reconstructs missing history. Adjacent durable rows receive
exact recurrence courts; gaps receive monotonic/bounded courts; restart/recovery
boundaries must be proved by producer-authored ancestry or are reported.
"""

from __future__ import annotations

import json
import math
import sys
from collections import Counter
from dataclasses import dataclass, field
from typing import Any, Dict, Iterator, List, Optional, Sequence, Tuple

from zpnet.shared.db import open_db


PHOTONS_SCHEMA = "PHOTONS_V1"
INSTRUMENT_SCHEMA = "PHOTONS_INSTRUMENT_V1"
STATS_SCHEMA = "PHOTONS_INSTRUMENT_STATS_V1"
SCIENCE_SCHEMA = "PHOTONS_SCIENCE_V2"
FW_PPB_SCHEMA = "PHOTONS_PPB_CHECKPOINT_DELTA_V1"
PI_PPB_SCHEMA = "PI_PHOTONS_PPB_RESTORE_CHECKPOINT_V1"
LANTERN_SCHEMA = "LANTERN_CAMPAIGN_V1"
CAMPAIGN_TYPE = "LANTERN"

STANDARD_LAP_NS = 10_000_000.0
DEFAULT_BATCH_SIZE = 512
MAX_EXAMPLES = 18
PPB_TOL = 2.0e-6
MEAN_TOL = 2.0e-6
WELFORD_ABS_TOL = 2.0e-8
WELFORD_REL_TOL = 2.0e-12

PPB_WINDOWS = ("10_min", "60_min", "8_hour", "24_hour")
SECOND_CAPACITY = 601
MINUTE_CAPACITY = 1442


def d(v: Any) -> Dict[str, Any]:
    return v if isinstance(v, dict) else {}


def req_dict(v: Any, where: str) -> Dict[str, Any]:
    if not isinstance(v, dict):
        raise ValueError(f"{where} must be an object")
    return v


def req_list(v: Any, where: str) -> List[Any]:
    if not isinstance(v, list):
        raise ValueError(f"{where} must be an array")
    return v


def req_int(v: Any, where: str, minimum: Optional[int] = None) -> int:
    if isinstance(v, bool) or not isinstance(v, int):
        raise ValueError(f"{where} must be an integer; got {v!r}")
    out = int(v)
    if minimum is not None and out < minimum:
        raise ValueError(f"{where} must be >= {minimum}; got {out}")
    return out


def opt_int(v: Any) -> Optional[int]:
    if v is None or isinstance(v, bool):
        return None
    try:
        return int(v)
    except (TypeError, ValueError, OverflowError):
        return None


def req_float(v: Any, where: str) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise ValueError(f"{where} must be numeric; got {v!r}")
    out = float(v)
    if not math.isfinite(out):
        raise ValueError(f"{where} must be finite; got {v!r}")
    return out


def opt_float(v: Any) -> Optional[float]:
    if v is None or isinstance(v, bool):
        return None
    try:
        out = float(v)
    except (TypeError, ValueError, OverflowError):
        return None
    return out if math.isfinite(out) else None


def close(a: float, b: float, abs_tol: float, rel_tol: float = 0.0) -> bool:
    return math.isclose(a, b, abs_tol=abs_tol, rel_tol=rel_tol)


def minute_key(sequence: int) -> int:
    return 0 if sequence <= 0 else ((sequence - 1) // 60) + 1


@dataclass(frozen=True)
class Welford:
    n: int
    mean: float
    m2: float
    min_value: float
    max_value: float

    @classmethod
    def parse(cls, obj: Any, where: str) -> "Welford":
        src = req_dict(obj, where)
        out = cls(
            n=req_int(src.get("n"), f"{where}.n", 0),
            mean=req_float(src.get("mean"), f"{where}.mean"),
            m2=req_float(src.get("m2"), f"{where}.m2"),
            min_value=req_float(src.get("min"), f"{where}.min"),
            max_value=req_float(src.get("max"), f"{where}.max"),
        )
        if out.n == 0:
            if any(v != 0.0 for v in (out.mean, out.m2, out.min_value, out.max_value)):
                raise ValueError(f"{where}: empty Welford publishes non-zero sufficient state")
        else:
            if out.m2 < -WELFORD_ABS_TOL:
                raise ValueError(f"{where}: negative m2")
            if out.min_value > out.max_value:
                raise ValueError(f"{where}: min > max")
            if not (out.min_value - WELFORD_ABS_TOL <= out.mean <= out.max_value + WELFORD_ABS_TOL):
                raise ValueError(f"{where}: mean outside min/max")
        return out

    def monotonic_population_after(self, prev: "Welford") -> bool:
        return self.n >= prev.n


@dataclass(frozen=True)
class Endpoint:
    sequence: int
    lap_count: int
    total_lap_gnss_ns: int

    @classmethod
    def parse(cls, obj: Any, where: str) -> "Endpoint":
        src = req_dict(obj, where)
        return cls(
            sequence=req_int(src.get("sequence"), f"{where}.sequence", 0),
            lap_count=req_int(src.get("lap_count"), f"{where}.lap_count", 0),
            total_lap_gnss_ns=req_int(src.get("total_lap_gnss_ns"), f"{where}.total_lap_gnss_ns", 0),
        )

    def monotonic_after(self, prev: "Endpoint") -> bool:
        return (
            self.sequence >= prev.sequence
            and self.lap_count >= prev.lap_count
            and self.total_lap_gnss_ns >= prev.total_lap_gnss_ns
        )


def ppb_from_endpoints(cur: Endpoint, anchor: Endpoint) -> float:
    laps = cur.lap_count - anchor.lap_count
    total_ns = cur.total_lap_gnss_ns - anchor.total_lap_gnss_ns
    if laps <= 0 or total_ns <= 0:
        raise ValueError("PPB proof has non-positive lap interval")
    expected_ns = float(laps) * STANDARD_LAP_NS
    return (float(total_ns) / expected_ns - 1.0) * 1.0e9


@dataclass
class Row:
    db_id: int
    ts: str
    sequence: int
    publish_count: int
    pps_count: int

    reset_count: int
    update_count: int
    rolling_sequence: int
    lap_count: int
    total_lap_gnss_ns: int
    custody_lap_count: int
    custody_total_lap_gnss_ns: int
    mean_lap_ns: float
    snapshot_ok: bool

    accepted_count: int
    accepted_this: int
    candidate_count: int
    candidates_this: int
    excluded_count: int
    excluded_this: int
    projected_lap_welford: Welford
    raw_cycles_welford: Welford

    endpoint_admitted: bool
    interval_advanced: bool
    fw_current: Endpoint
    fw_origin: Endpoint
    fw_origin_valid: bool
    fw_second_append: Optional[Endpoint]
    fw_minute_append: Optional[Endpoint]
    fw_second_count: int
    fw_minute_count: int
    fw_last_minute_key: int
    buckets: Dict[str, Tuple[float, int]]
    fw_proofs: Dict[str, Tuple[Endpoint, int, bool]]

    pi_recoverable: Optional[bool]
    pi_current: Optional[Endpoint]
    pi_second_count: Optional[int]
    pi_minute_count: Optional[int]
    pi_second_tail: Optional[Endpoint]
    pi_minute_tail: Optional[Endpoint]
    pi_seed_db_id: Optional[int]

    recovery: Dict[str, Any]

    campaign: Optional[str]
    campaign_public_count: Optional[int]
    campaign_start_after_sequence: Optional[int]
    campaign_lap_count: Optional[int]
    campaign_total_lap_gnss_ns: Optional[int]
    campaign_sample_count: Optional[int]
    campaign_mean_lap_ns: Optional[float]
    campaign_ppb: Optional[float]

    @classmethod
    def parse(cls, db_id: int, ts: str, payload: Dict[str, Any]) -> "Row":
        root = payload
        if root.get("schema") != PHOTONS_SCHEMA:
            inner = d(root.get("payload"))
            if inner.get("schema") == PHOTONS_SCHEMA:
                root = inner
        if root.get("schema") != PHOTONS_SCHEMA:
            raise ValueError(f"db_id={db_id}: not {PHOTONS_SCHEMA}")

        instr = req_dict(root.get("photons"), f"db_id={db_id}.photons")
        if instr.get("schema") != INSTRUMENT_SCHEMA:
            raise ValueError(f"db_id={db_id}: photons schema mismatch")
        stats = req_dict(instr.get("stats"), f"db_id={db_id}.photons.stats")
        science = req_dict(instr.get("science"), f"db_id={db_id}.photons.science")
        if stats.get("schema") != STATS_SCHEMA or science.get("schema") != SCIENCE_SCHEMA:
            raise ValueError(f"db_id={db_id}: stats/science schema mismatch")

        accepted = req_dict(science.get("accepted"), f"db_id={db_id}.science.accepted")
        excluded = req_dict(science.get("excluded"), f"db_id={db_id}.science.excluded")
        fw = req_dict(stats.get("rolling_ppb_checkpoint"), f"db_id={db_id}.rolling_ppb_checkpoint")
        if fw.get("schema") != FW_PPB_SCHEMA or fw.get("valid") is not True:
            raise ValueError(f"db_id={db_id}: invalid firmware PPB checkpoint")

        fw_cur = Endpoint.parse(fw.get("current"), f"db_id={db_id}.fw.current")
        fw_origin_valid = fw.get("origin_valid") is True
        fw_origin = Endpoint.parse(fw.get("origin"), f"db_id={db_id}.fw.origin") if fw_origin_valid else Endpoint(0, 0, 0)

        def append_endpoint(name: str) -> Optional[Endpoint]:
            if fw.get(f"{name}_valid") is not True:
                return None
            return Endpoint.parse(fw.get(name), f"db_id={db_id}.fw.{name}")

        buckets_obj = req_dict(stats.get("ppb_buckets"), f"db_id={db_id}.ppb_buckets")
        buckets: Dict[str, Tuple[float, int]] = {}
        for name in (*PPB_WINDOWS, "total"):
            node = req_dict(buckets_obj.get(name), f"db_id={db_id}.ppb_buckets.{name}")
            buckets[name] = (
                req_float(node.get("ppb"), f"db_id={db_id}.ppb_buckets.{name}.ppb"),
                req_int(node.get("sample_count"), f"db_id={db_id}.ppb_buckets.{name}.sample_count", 0),
            )

        fw_proofs: Dict[str, Tuple[Endpoint, int, bool]] = {}
        for name in PPB_WINDOWS:
            node = req_dict(fw.get(name), f"db_id={db_id}.fw.{name}")
            valid = node.get("valid") is True
            sample_count = req_int(node.get("sample_count"), f"db_id={db_id}.fw.{name}.sample_count", 0)
            anchor = Endpoint.parse(node.get("anchor"), f"db_id={db_id}.fw.{name}.anchor") if valid else Endpoint(0, 0, 0)
            fw_proofs[name] = (anchor, sample_count, valid)

        pi = root.get("ppb_restore_checkpoint")
        pi_recoverable: Optional[bool] = None
        pi_current = None
        pi_second_count = None
        pi_minute_count = None
        pi_second_tail = None
        pi_minute_tail = None
        pi_seed_db_id = None
        if pi is not None:
            pi = req_dict(pi, f"db_id={db_id}.ppb_restore_checkpoint")
            if pi.get("schema") != PI_PPB_SCHEMA or pi.get("valid") is not True:
                raise ValueError(f"db_id={db_id}: invalid Pi PPB checkpoint")
            pi_recoverable = pi.get("recoverable") is True
            pi_current = Endpoint.parse(pi.get("current"), f"db_id={db_id}.pi.current")
            seconds = req_list(pi.get("second_history"), f"db_id={db_id}.pi.second_history")
            minutes = req_list(pi.get("minute_history"), f"db_id={db_id}.pi.minute_history")
            if len(seconds) > SECOND_CAPACITY or len(minutes) > MINUTE_CAPACITY:
                raise ValueError(f"db_id={db_id}: Pi PPB ring exceeds capacity")
            pi_second_count = len(seconds)
            pi_minute_count = len(minutes)
            if seconds:
                pi_second_tail = Endpoint.parse(seconds[-1], f"db_id={db_id}.pi.second_tail")
            if minutes:
                pi_minute_tail = Endpoint.parse(minutes[-1], f"db_id={db_id}.pi.minute_tail")
            pi_seed_db_id = opt_int(pi.get("seed_source_db_detail_id"))

        campaign_obj = d(root.get("campaign"))
        campaign = None
        public_count = None
        start_after = None
        c_lap = c_total = c_samples = None
        c_mean = c_ppb = None
        if campaign_obj:
            if campaign_obj.get("schema") != LANTERN_SCHEMA:
                raise ValueError(f"db_id={db_id}: campaign schema mismatch")
            campaign = str(campaign_obj.get("campaign") or "").strip() or None
            public_count = opt_int(campaign_obj.get("public_count"))
            start_after = opt_int(campaign_obj.get("start_after_sequence"))
            cstats = d(campaign_obj.get("stats"))
            c_lap = opt_int(cstats.get("lap_count"))
            c_total = opt_int(cstats.get("total_lap_gnss_ns"))
            c_samples = opt_int(cstats.get("sample_count"))
            c_mean = opt_float(cstats.get("mean_lap_ns"))
            c_ppb = opt_float(cstats.get("ppb"))

        return cls(
            db_id=db_id,
            ts=ts,
            sequence=req_int(root.get("sequence"), f"db_id={db_id}.sequence", 1),
            publish_count=req_int(root.get("publish_count"), f"db_id={db_id}.publish_count", 0),
            pps_count=req_int(root.get("pps_count"), f"db_id={db_id}.pps_count", 0),
            reset_count=req_int(stats.get("reset_count"), f"db_id={db_id}.reset_count", 0),
            update_count=req_int(stats.get("update_count"), f"db_id={db_id}.update_count", 0),
            rolling_sequence=req_int(stats.get("rolling_ppb_current_sequence"), f"db_id={db_id}.rolling_sequence", 0),
            lap_count=req_int(stats.get("lap_count"), f"db_id={db_id}.lap_count", 0),
            total_lap_gnss_ns=req_int(stats.get("total_lap_gnss_ns"), f"db_id={db_id}.total_lap_gnss_ns", 0),
            custody_lap_count=req_int(stats.get("custody_lap_count"), f"db_id={db_id}.custody_lap_count", 0),
            custody_total_lap_gnss_ns=req_int(stats.get("custody_total_lap_gnss_ns"), f"db_id={db_id}.custody_total_lap_gnss_ns", 0),
            mean_lap_ns=req_float(stats.get("mean_lap_ns"), f"db_id={db_id}.mean_lap_ns"),
            snapshot_ok=instr.get("snapshot_ok") is True,
            accepted_count=req_int(accepted.get("count"), f"db_id={db_id}.accepted.count", 0),
            accepted_this=req_int(accepted.get("count_this_fragment"), f"db_id={db_id}.accepted.count_this_fragment", 0),
            candidate_count=req_int(science.get("candidate_count"), f"db_id={db_id}.candidate_count", 0),
            candidates_this=req_int(science.get("candidates_this_fragment"), f"db_id={db_id}.candidates_this_fragment", 0),
            excluded_count=req_int(excluded.get("count"), f"db_id={db_id}.excluded.count", 0),
            excluded_this=req_int(excluded.get("count_this_fragment"), f"db_id={db_id}.excluded.count_this_fragment", 0),
            projected_lap_welford=Welford.parse(accepted.get("projected_lap_ns"), f"db_id={db_id}.accepted.projected_lap_ns"),
            raw_cycles_welford=Welford.parse(accepted.get("raw_cycles"), f"db_id={db_id}.accepted.raw_cycles"),
            endpoint_admitted=stats.get("rolling_ppb_endpoint_admitted") is True,
            interval_advanced=stats.get("rolling_ppb_interval_advanced") is True,
            fw_current=fw_cur,
            fw_origin=fw_origin,
            fw_origin_valid=fw_origin_valid,
            fw_second_append=append_endpoint("second_append"),
            fw_minute_append=append_endpoint("minute_append"),
            fw_second_count=req_int(fw.get("second_count"), f"db_id={db_id}.fw.second_count", 0),
            fw_minute_count=req_int(fw.get("minute_count"), f"db_id={db_id}.fw.minute_count", 0),
            fw_last_minute_key=req_int(fw.get("last_minute_key"), f"db_id={db_id}.fw.last_minute_key", 0),
            buckets=buckets,
            fw_proofs=fw_proofs,
            pi_recoverable=pi_recoverable,
            pi_current=pi_current,
            pi_second_count=pi_second_count,
            pi_minute_count=pi_minute_count,
            pi_second_tail=pi_second_tail,
            pi_minute_tail=pi_minute_tail,
            pi_seed_db_id=pi_seed_db_id,
            recovery=d(instr.get("recovery")),
            campaign=campaign,
            campaign_public_count=public_count,
            campaign_start_after_sequence=start_after,
            campaign_lap_count=c_lap,
            campaign_total_lap_gnss_ns=c_total,
            campaign_sample_count=c_samples,
            campaign_mean_lap_ns=c_mean,
            campaign_ppb=c_ppb,
        )


@dataclass
class Proof:
    passed: int = 0
    bounded: int = 0
    failed: int = 0
    unavailable: int = 0

    def mark(self, status: str) -> None:
        setattr(self, status, getattr(self, status) + 1)


@dataclass
class Audit:
    scope: str
    rows: int = 0
    problems: Counter[str] = field(default_factory=Counter)
    events: Counter[str] = field(default_factory=Counter)
    examples: Dict[str, List[str]] = field(default_factory=dict)
    proofs: Dict[str, Proof] = field(default_factory=dict)
    boundaries: Counter[str] = field(default_factory=Counter)
    epochs: Counter[int] = field(default_factory=Counter)
    campaigns: Counter[str] = field(default_factory=Counter)

    first_id: Optional[int] = None
    last_id: Optional[int] = None
    first_ts: str = ""
    last_ts: str = ""

    durable_gaps: int = 0
    missing_rows: int = 0
    campaign_splices: int = 0
    ppb_window_checks: int = 0
    pi_checkpoint_checks: int = 0
    exact_adjacent_rows: int = 0

    last_row: Optional[Row] = None
    last_campaign: Dict[str, Row] = field(default_factory=dict)

    def proof(self, name: str, status: str) -> None:
        self.proofs.setdefault(name, Proof()).mark(status)

    def note(self, bucket: str, msg: str) -> None:
        arr = self.examples.setdefault(bucket, [])
        if len(arr) < MAX_EXAMPLES:
            arr.append(msg)

    def problem(self, kind: str, row: Row, detail: str) -> None:
        self.problems[kind] += 1
        self.note(kind, f"db_id={row.db_id} seq={row.sequence} {detail}")

    def event(self, kind: str, row: Row, detail: str) -> None:
        self.events[kind] += 1
        self.note(f"event:{kind}", f"db_id={row.db_id} seq={row.sequence} {detail}")


def check_row(audit: Audit, row: Row) -> None:
    failed = False
    if not row.snapshot_ok:
        audit.problem("snapshot_not_ok", row, "snapshot_ok=false")
        failed = True
    if row.rolling_sequence != row.update_count:
        audit.problem("rolling_update_identity", row, f"rolling={row.rolling_sequence} update={row.update_count}")
        failed = True
    if row.fw_current.sequence != row.rolling_sequence:
        audit.problem("fw_current_identity", row, f"current.sequence={row.fw_current.sequence} rolling={row.rolling_sequence}")
        failed = True
    if row.fw_current.lap_count != row.lap_count or row.fw_current.total_lap_gnss_ns != row.total_lap_gnss_ns:
        audit.problem("fw_current_stats_identity", row, "firmware endpoint != stats lap totals")
        failed = True

    if row.accepted_count != row.lap_count or row.projected_lap_welford.n != row.lap_count:
        audit.problem("science_accepted_identity", row, f"accepted={row.accepted_count} welford_n={row.projected_lap_welford.n} lap_count={row.lap_count}")
        failed = True
    if row.candidate_count != row.accepted_count + row.excluded_count:
        audit.problem("science_candidate_accounting", row, f"candidate={row.candidate_count} accepted={row.accepted_count} excluded={row.excluded_count}")
        failed = True
    if row.candidates_this != row.accepted_this + row.excluded_this:
        audit.problem("science_fragment_accounting", row, f"this={row.candidates_this} accepted={row.accepted_this} excluded={row.excluded_this}")
        failed = True

    if row.lap_count > 0:
        expected_mean = row.total_lap_gnss_ns / row.lap_count
        if not close(row.mean_lap_ns, expected_mean, MEAN_TOL):
            audit.problem("mean_lap_total", row, f"mean={row.mean_lap_ns:.9f} expected={expected_mean:.9f}")
            failed = True
        total_ppb = (expected_mean / STANDARD_LAP_NS - 1.0) * 1.0e9
        published_total, total_samples = row.buckets["total"]
        if total_samples != row.lap_count or not close(published_total, total_ppb, PPB_TOL):
            audit.problem("total_ppb", row, f"published={published_total:.9f}/{total_samples} expected={total_ppb:.9f}/{row.lap_count}")
            failed = True

    if row.fw_second_append is not None and row.fw_second_append != row.fw_current:
        audit.problem("fw_second_append", row, "second append != current")
        failed = True
    if row.fw_minute_append is not None and row.fw_minute_append != row.fw_current:
        audit.problem("fw_minute_append", row, "minute append != current")
        failed = True

    for name in PPB_WINDOWS:
        anchor, sample_count, valid = row.fw_proofs[name]
        published, published_samples = row.buckets[name]
        if not valid:
            audit.proof("ppb_windows", "unavailable")
            continue
        expected_samples = row.fw_current.lap_count - anchor.lap_count
        if sample_count != expected_samples or published_samples != sample_count:
            audit.problem("ppb_sample_count", row, f"{name} fw={sample_count} published={published_samples} endpoint={expected_samples}")
            audit.proof("ppb_windows", "failed")
            continue
        try:
            computed = ppb_from_endpoints(row.fw_current, anchor)
        except ValueError as exc:
            audit.problem("ppb_math", row, f"{name}: {exc}")
            audit.proof("ppb_windows", "failed")
            continue
        audit.ppb_window_checks += 1
        if not close(published, computed, PPB_TOL):
            audit.problem("ppb_window", row, f"{name} published={published:.9f} computed={computed:.9f}")
            audit.proof("ppb_windows", "failed")
        else:
            audit.proof("ppb_windows", "passed")

    if row.pi_current is None:
        audit.proof("pi_restore_checkpoint", "unavailable")
    else:
        pi_failed = False
        audit.pi_checkpoint_checks += 1
        if row.pi_current != row.fw_current:
            audit.problem("pi_fw_current", row, "Pi current != firmware current")
            pi_failed = True
        if row.pi_second_tail is not None and row.pi_second_tail != row.fw_current:
            audit.problem("pi_second_tail", row, "Pi second tail != firmware current")
            pi_failed = True
        if row.fw_minute_append is not None and row.pi_minute_tail != row.fw_current:
            audit.problem("pi_minute_tail", row, "firmware minute append != Pi minute tail")
            pi_failed = True
        audit.proof("pi_restore_checkpoint", "failed" if pi_failed else ("passed" if row.pi_recoverable else "bounded"))

    if row.campaign is not None:
        c_failed = False
        if row.campaign_public_count is None or row.campaign_public_count <= 0:
            audit.problem("campaign_identity", row, f"public_count={row.campaign_public_count}")
            c_failed = True
        if (
            row.campaign_public_count is not None
            and row.campaign_start_after_sequence is not None
            and row.campaign_public_count != row.sequence - row.campaign_start_after_sequence
        ):
            audit.problem(
                "campaign_public_source_identity",
                row,
                f"public_count={row.campaign_public_count} sequence={row.sequence} "
                f"start_after_sequence={row.campaign_start_after_sequence}",
            )
            c_failed = True
        if row.campaign_sample_count != row.campaign_lap_count:
            audit.problem("campaign_sample_identity", row, f"samples={row.campaign_sample_count} laps={row.campaign_lap_count}")
            c_failed = True
        if row.campaign_lap_count and row.campaign_total_lap_gnss_ns is not None:
            expected_mean = row.campaign_total_lap_gnss_ns / row.campaign_lap_count
            expected_ppb = (expected_mean / STANDARD_LAP_NS - 1.0) * 1.0e9
            if row.campaign_mean_lap_ns is None or not close(row.campaign_mean_lap_ns, expected_mean, MEAN_TOL):
                audit.problem("campaign_mean", row, f"published={row.campaign_mean_lap_ns} expected={expected_mean:.9f}")
                c_failed = True
            if row.campaign_ppb is None or not close(row.campaign_ppb, expected_ppb, PPB_TOL):
                audit.problem("campaign_ppb", row, f"published={row.campaign_ppb} expected={expected_ppb:.9f}")
                c_failed = True
        audit.proof("campaign_row", "failed" if c_failed else "passed")

    audit.proof("row_internal", "failed" if failed else "passed")


def classify(prev: Row, cur: Row) -> str:
    if cur.reset_count > prev.reset_count:
        return "STATS_RESET"
    if cur.reset_count < prev.reset_count:
        return "STATS_RESET_REGRESSION"
    delta = cur.sequence - prev.sequence
    if delta == 1 and cur.update_count == prev.update_count + 1:
        return "NORMAL_CONTINUATION"
    if delta > 1:
        return "DURABLE_OBSERVATION_GAP"
    if delta < 0:
        recovery = cur.recovery
        if recovery.get("restored") is True and recovery.get("proof_committed") is True:
            return "PHYSICAL_REBOOT_PROVED_RECOVERY"
        if cur.update_count == 1:
            return "PHYSICAL_REBOOT_NEW_EPOCH"
        return "PHYSICAL_REBOOT_UNPROVED"
    return "ADJACENT_CHRONOLOGY_MISMATCH"


def check_adjacent(audit: Audit, prev: Row, cur: Row) -> str:
    boundary = classify(prev, cur)
    audit.boundaries[boundary] += 1

    if boundary == "NORMAL_CONTINUATION":
        failed = False
        if cur.lap_count - prev.lap_count != cur.accepted_this:
            audit.problem("accepted_lap_delta", cur, f"lap {prev.lap_count}->{cur.lap_count} accepted_this={cur.accepted_this}")
            failed = True
        if cur.candidate_count - prev.candidate_count != cur.candidates_this:
            audit.problem("candidate_delta", cur, f"candidate {prev.candidate_count}->{cur.candidate_count} this={cur.candidates_this}")
            failed = True
        if cur.excluded_count - prev.excluded_count != cur.excluded_this:
            audit.problem("excluded_delta", cur, f"excluded {prev.excluded_count}->{cur.excluded_count} this={cur.excluded_this}")
            failed = True
        if cur.custody_lap_count < prev.custody_lap_count:
            audit.problem("custody_lap_regression", cur, f"{prev.custody_lap_count}->{cur.custody_lap_count}")
            failed = True
        audit.exact_adjacent_rows += 1
        audit.proof("instrument_lineage", "failed" if failed else "passed")

    elif boundary == "DURABLE_OBSERVATION_GAP":
        delta = cur.sequence - prev.sequence
        audit.durable_gaps += 1
        audit.missing_rows += delta - 1
        audit.event("durable_observation_gap", cur, f"sequence {prev.sequence}->{cur.sequence} missing={delta - 1}")
        monotonic = (
            cur.update_count > prev.update_count
            and cur.lap_count >= prev.lap_count
            and cur.total_lap_gnss_ns >= prev.total_lap_gnss_ns
            and cur.custody_lap_count >= prev.custody_lap_count
        )
        if not monotonic:
            audit.problem("gap_nonmonotonic", cur, f"stats {prev.update_count}/{prev.lap_count}->{cur.update_count}/{cur.lap_count}")
            audit.proof("instrument_lineage", "failed")
        else:
            audit.proof("instrument_lineage", "bounded")

    elif boundary == "STATS_RESET":
        if cur.update_count != 1 or cur.rolling_sequence not in (0, 1):
            audit.problem("stats_reset_birth", cur, f"reset={prev.reset_count}->{cur.reset_count} update={cur.update_count} rolling={cur.rolling_sequence}")
            audit.proof("instrument_lineage", "failed")
        else:
            audit.event("stats_epoch", cur, f"reset_count {prev.reset_count}->{cur.reset_count}")
            audit.proof("instrument_lineage", "passed")

    elif boundary == "PHYSICAL_REBOOT_PROVED_RECOVERY":
        r = cur.recovery
        source_update = opt_int(r.get("source_update_count"))
        source_laps = opt_int(r.get("source_lap_count"))
        source_total = opt_int(r.get("source_total_lap_gnss_ns"))
        failed = False
        if source_update is None or cur.update_count <= source_update:
            audit.problem("recovery_update", cur, f"source={source_update} current={cur.update_count}")
            failed = True
        if source_laps is None or cur.lap_count < source_laps:
            audit.problem("recovery_laps", cur, f"source={source_laps} current={cur.lap_count}")
            failed = True
        if source_total is None or cur.total_lap_gnss_ns < source_total:
            audit.problem("recovery_total", cur, f"source={source_total} current={cur.total_lap_gnss_ns}")
            failed = True
        audit.event("proved_recovery", cur, f"generation={r.get('generation')} source_update={source_update} current_update={cur.update_count}")
        audit.proof("recovery_lineage", "failed" if failed else "passed")

    elif boundary == "PHYSICAL_REBOOT_NEW_EPOCH":
        audit.event("physical_reboot_new_epoch", cur, f"update_count={cur.update_count}")
        audit.proof("instrument_lineage", "bounded")

    else:
        audit.problem("unproved_boundary", cur, f"{boundary} seq {prev.sequence}->{cur.sequence} update {prev.update_count}->{cur.update_count}")
        audit.proof("instrument_lineage", "failed")

    return boundary


def check_campaign_timeline(audit: Audit, row: Row, boundary: Optional[str]) -> None:
    if row.campaign is None or row.campaign_public_count is None:
        return
    prev = audit.last_campaign.get(row.campaign)
    if prev is not None and prev.campaign_public_count is not None:
        delta = row.campaign_public_count - prev.campaign_public_count
        if delta <= 0:
            audit.problem("campaign_public_chronology", row, f"{prev.campaign_public_count}->{row.campaign_public_count}")
            audit.proof("campaign_timeline", "failed")
        elif delta == 1:
            audit.proof("campaign_timeline", "passed")
        else:
            audit.campaign_splices += 1
            lawful = boundary in {
                "DURABLE_OBSERVATION_GAP",
                "PHYSICAL_REBOOT_PROVED_RECOVERY",
                "PHYSICAL_REBOOT_NEW_EPOCH",
            }
            audit.event("campaign_forward_splice", row, f"{prev.campaign_public_count}->{row.campaign_public_count} delta={delta} lawful={lawful}")
            audit.proof("campaign_timeline", "passed" if lawful else "failed")
            if not lawful:
                audit.problem("campaign_unexplained_splice", row, f"{prev.campaign_public_count}->{row.campaign_public_count}")
    audit.last_campaign[row.campaign] = row


def process(audit: Audit, row: Row) -> None:
    audit.rows += 1
    audit.first_id = row.db_id if audit.first_id is None else audit.first_id
    audit.last_id = row.db_id
    audit.first_ts = row.ts if not audit.first_ts else audit.first_ts
    audit.last_ts = row.ts
    audit.epochs[row.reset_count] += 1
    if row.campaign:
        audit.campaigns[row.campaign] += 1

    check_row(audit, row)
    boundary = None
    if audit.last_row is not None:
        boundary = check_adjacent(audit, audit.last_row, row)
    check_campaign_timeline(audit, row, boundary)
    audit.last_row = row


@dataclass(frozen=True)
class Args:
    campaign: Optional[str]
    all_rows: bool
    from_id: int
    to_id: int
    batch_size: int
    limit: int


def parse_args(argv: Sequence[str]) -> Args:
    campaign = None
    all_rows = False
    from_id = 0
    to_id = 0
    batch_size = DEFAULT_BATCH_SIZE
    limit = 0
    i = 1
    while i < len(argv):
        arg = argv[i]
        if arg == "--all":
            all_rows = True
            i += 1
            continue
        if arg in {"--from-id", "--to-id", "--batch-size", "--limit"}:
            if i + 1 >= len(argv):
                raise SystemExit(f"{arg} requires a value")
            value = int(argv[i + 1])
            i += 2
            if arg == "--from-id":
                from_id = value
            elif arg == "--to-id":
                to_id = value
            elif arg == "--batch-size":
                batch_size = value
            else:
                limit = value
            continue
        if arg.startswith("--"):
            raise SystemExit(f"unknown option {arg}")
        if campaign is not None:
            raise SystemExit("only one campaign may be supplied")
        campaign = arg
        i += 1
    if all_rows and campaign is not None:
        raise SystemExit("choose --all or CAMPAIGN")
    if campaign is None and not all_rows and any(v > 0 for v in (from_id, to_id, limit)):
        all_rows = True
    if batch_size <= 0 or min(from_id, to_id, limit) < 0:
        raise SystemExit("numeric options invalid")
    return Args(campaign, all_rows, from_id, to_id, batch_size, limit)


def campaign_bounds(campaign: str) -> Tuple[int, int]:
    with open_db(row_dict=True) as conn:
        conn.execute("SET TRANSACTION READ ONLY")
        cur = conn.cursor()
        vals = []
        for order in ("ASC", "DESC"):
            cur.execute(
                f"""
                SELECT id
                FROM campaign_detail
                WHERE campaign_type = %s
                  AND campaign = %s
                  AND payload #>> '{{schema}}' = %s
                ORDER BY id {order}
                LIMIT 1
                """,
                (CAMPAIGN_TYPE, campaign, PHOTONS_SCHEMA),
            )
            row = cur.fetchone()
            vals.append(opt_int(row.get("id")) if row else None)
    if vals[0] is None or vals[1] is None:
        raise SystemExit(f"no PHOTONS_V1 rows found for LANTERN campaign {campaign!r}")
    return vals[0], vals[1]


def iter_rows(args: Args) -> Iterator[Tuple[int, str, Dict[str, Any]]]:
    lo = args.from_id
    hi = args.to_id
    if args.campaign:
        c_lo, c_hi = campaign_bounds(args.campaign)
        lo = max(lo, c_lo) if lo else c_lo
        hi = min(hi, c_hi) if hi else c_hi

    where = ["campaign_type = %s", "payload #>> '{schema}' = %s"]
    params: List[Any] = [CAMPAIGN_TYPE, PHOTONS_SCHEMA]
    if lo:
        where.append("id >= %s")
        params.append(lo)
    if hi:
        where.append("id <= %s")
        params.append(hi)

    sql = f"""
        SELECT id, ts, payload
        FROM campaign_detail
        WHERE {' AND '.join(where)}
        ORDER BY id ASC
    """
    if args.limit:
        sql += " LIMIT %s"
        params.append(args.limit)

    with open_db(row_dict=True) as conn:
        conn.execute("SET TRANSACTION READ ONLY")
        cur = conn.cursor(name="photons_analyzer")
        cur.itersize = args.batch_size
        cur.execute(sql, tuple(params))
        for row in cur:
            payload = row["payload"]
            if isinstance(payload, str):
                payload = json.loads(payload)
            yield int(row["id"]), str(row["ts"]), payload


def proof_status(p: Proof) -> str:
    if p.failed:
        return "FAIL"
    if p.passed and not p.bounded and not p.unavailable:
        return "EXACT"
    if p.passed or p.bounded:
        return "PROVED/BOUNDED"
    return "NOT OBSERVED"


def print_report(a: Audit) -> None:
    print("=" * 96)
    print(f"PHOTONS / LANTERN PROOF LEDGER: {a.scope}")
    print("=" * 96)
    print(f"Rows:                  {a.rows:,}")
    print(f"DB id:                 {a.first_id} -> {a.last_id}")
    print(f"Database time:         {a.first_ts} -> {a.last_ts}")
    print(f"Durable gaps:          {a.durable_gaps:,} ({a.missing_rows:,} absent row(s))")
    print(f"Campaign splices:      {a.campaign_splices:,}")
    print(f"Exact adjacent rows:   {a.exact_adjacent_rows:,}")
    print(f"PPB window proofs:     {a.ppb_window_checks:,}")
    print(f"Pi checkpoint checks:  {a.pi_checkpoint_checks:,}")

    print("\nSTATISTICAL EPOCHS")
    for k, v in sorted(a.epochs.items()):
        print(f"  reset_count={k:<6d} rows={v:,}")

    print("\nBOUNDARY SPECIES")
    for k, v in a.boundaries.most_common():
        print(f"  {k:<42s} {v:,}")
    if not a.boundaries:
        print("  none")

    print("\nCAMPAIGNS OBSERVED")
    for k, v in a.campaigns.most_common():
        print(f"  {k:<28s} {v:,}")
    if not a.campaigns:
        print("  none")

    print("\nPROOF LEDGER")
    for name in (
        "row_internal",
        "instrument_lineage",
        "recovery_lineage",
        "ppb_windows",
        "pi_restore_checkpoint",
        "campaign_row",
        "campaign_timeline",
    ):
        p = a.proofs.get(name, Proof())
        print(f"  {name:<30s} {proof_status(p):<15s} exact={p.passed:,} bounded={p.bounded:,} unavailable={p.unavailable:,} fail={p.failed:,}")

    print("\nEVENTS")
    for k, v in sorted(a.events.items()):
        print(f"  {k:<42s} {v:,}")
    if not a.events:
        print("  none")

    print("\nSTRICT FINDINGS")
    total = sum(a.problems.values())
    if not a.problems:
        print("  NONE")
    else:
        for k, v in sorted(a.problems.items()):
            print(f"  {k:<42s} {v:,}")
    print(f"  {'TOTAL':<42s} {total:,}")

    if a.examples:
        print("\nBOUNDED EVIDENCE / EXAMPLES")
        for k, vals in a.examples.items():
            print(f"  [{k}]")
            for val in vals:
                print(f"    {val}")

    print("\nVERDICT")
    if total:
        print(f"  INVESTIGATE — {total:,} strict PHOTONS/LANTERN proof failure(s).")
    else:
        bounded = sum(p.bounded + p.unavailable for p in a.proofs.values())
        if bounded:
            print("  STRUCTURALLY CLEAN WITH EXPLICIT LIMITS — every strict court closed; gaps or unavailable proof species remain explicit.")
        else:
            print("  PHOTONS/LANTERN EXACT — every observed instrument, PPB, recovery, and campaign theorem closed.")


def main(argv: Sequence[str]) -> int:
    args = parse_args(argv)
    if not args.all_rows and args.campaign is None:
        print("Usage: photons_analyzer.py CAMPAIGN | --all [--from-id N --to-id N --limit N]")
        return 0

    audit = Audit(scope=args.campaign or "ALL PHOTONS_V1")
    for db_id, ts, payload in iter_rows(args):
        try:
            row = Row.parse(db_id, ts, payload)
        except Exception as exc:
            audit.rows += 1
            audit.problems["row_parse"] += 1
            audit.note("row_parse", f"db_id={db_id} ts={ts} {type(exc).__name__}: {exc}")
            continue
        process(audit, row)

    print_report(audit)
    return 1 if audit.problems else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
