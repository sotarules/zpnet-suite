"""ZPNet photons_analyzer — proof-ledger audit for canonical PHOTONS_V1 / LANTERN.

PHOTONS is simpler than CLOCKS: the canonical row carries the physical instrument
state, accepted-science populations, producer Better-Buckets proof, explicit
recovery ancestry, and the LANTERN campaign projection.  Pi's literal bounded
resurrection image is private continuation state in config.PHOTONS_RECOVERY; modern
canonical rows do not copy it.  Successful held restores instead stamp the exact
durable proof row with a compact immutable ``recovery_receipt`` that binds source,
private-checkpoint fingerprint, exact N+1, and campaign coordinates.

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
RECOVERY_RECEIPT_SCHEMA = "PI_PHOTONS_RECOVERY_RECEIPT_V1"
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


def valid_sha256(value: Any) -> bool:
    text = str(value or "").strip().lower()
    return len(text) == 64 and all(ch in "0123456789abcdef" for ch in text)


def parse_recovery_receipt_optional(obj: Any, where: str) -> Optional[Dict[str, Any]]:
    if obj is None:
        return None
    raw = req_dict(obj, where)
    if raw.get("schema") != RECOVERY_RECEIPT_SCHEMA:
        raise ValueError(
            f"{where} schema={raw.get('schema')!r}, expected {RECOVERY_RECEIPT_SCHEMA}"
        )
    if str(raw.get("subsystem") or "").strip().upper() != "PHOTONS":
        raise ValueError(f"{where}.subsystem must be PHOTONS")
    mode = str(raw.get("recovery_mode") or "").strip().upper()
    if mode not in {"HELD_RESTORE", "PENDING_RESTORE_PROOF"}:
        raise ValueError(f"{where}.recovery_mode={mode!r} is not a receipt-bearing restore mode")
    req_int(raw.get("generation"), f"{where}.generation", 1)
    if not isinstance(raw.get("producer_mutated"), bool):
        raise ValueError(f"{where}.producer_mutated must be boolean")

    source = req_dict(raw.get("source"), f"{where}.source")
    req_int(source.get("detail_id"), f"{where}.source.detail_id", 1)
    req_int(source.get("sequence"), f"{where}.source.sequence", 1)
    req_int(source.get("reset_count"), f"{where}.source.reset_count", 0)
    req_int(source.get("update_count"), f"{where}.source.update_count", 1)

    checkpoint = req_dict(raw.get("checkpoint"), f"{where}.checkpoint")
    if checkpoint.get("schema") != PI_PPB_SCHEMA:
        raise ValueError(f"{where}.checkpoint schema mismatch")
    digest_available = checkpoint.get("digest_available")
    if not isinstance(digest_available, bool):
        raise ValueError(f"{where}.checkpoint.digest_available must be boolean")
    if digest_available:
        req_int(checkpoint.get("reset_count"), f"{where}.checkpoint.reset_count", 0)
        req_int(checkpoint.get("update_count"), f"{where}.checkpoint.update_count", 1)
        req_int(checkpoint.get("current_sequence"), f"{where}.checkpoint.current_sequence", 0)
        if not isinstance(checkpoint.get("recoverable"), bool):
            raise ValueError(f"{where}.checkpoint.recoverable must be boolean")
        for key in ("second_count", "expected_second_count", "minute_count", "expected_minute_count", "gap_count"):
            req_int(checkpoint.get(key), f"{where}.checkpoint.{key}", 0)

    boundary = req_dict(raw.get("boundary"), f"{where}.boundary")
    req_int(boundary.get("detail_id"), f"{where}.boundary.detail_id", 1)
    req_int(boundary.get("sequence"), f"{where}.boundary.sequence", 1)
    req_int(boundary.get("reset_count"), f"{where}.boundary.reset_count", 0)
    req_int(boundary.get("update_count"), f"{where}.boundary.update_count", 1)

    proof = req_dict(raw.get("proof"), f"{where}.proof")
    if str(proof.get("contract") or "") != "EXACT_SOURCE_N_PLUS_1":
        raise ValueError(f"{where}.proof.contract must be EXACT_SOURCE_N_PLUS_1")
    if not isinstance(proof.get("durable"), bool) or not isinstance(
        proof.get("fresh_physical_ancestry"), bool
    ):
        raise ValueError(f"{where}.proof durable/fresh_physical_ancestry must be boolean")

    history = raw.get("history")
    if history is not None:
        history = req_dict(history, f"{where}.history")
        if not isinstance(history.get("truncated"), bool):
            raise ValueError(f"{where}.history.truncated must be boolean")
        for key in (
            "staged_second_count", "staged_minute_count",
            "surrendered_second_endpoints", "surrendered_minute_endpoints",
        ):
            req_int(history.get(key), f"{where}.history.{key}", 0)
    campaign = raw.get("campaign")
    if campaign is not None:
        campaign = req_dict(campaign, f"{where}.campaign")
        if not str(campaign.get("campaign") or "").strip():
            raise ValueError(f"{where}.campaign.campaign missing")
        req_int(campaign.get("public_count"), f"{where}.campaign.public_count", 1)
        req_int(campaign.get("start_after_sequence"), f"{where}.campaign.start_after_sequence", 1)
    return raw


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
    pps_count: Optional[int]

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
    buckets: Dict[str, Optional[Tuple[float, int]]]
    fw_proofs: Dict[str, Tuple[Endpoint, int, bool]]

    pi_recoverable: Optional[bool]
    pi_current: Optional[Endpoint]
    pi_second_count: Optional[int]
    pi_minute_count: Optional[int]
    pi_second_tail: Optional[Endpoint]
    pi_minute_tail: Optional[Endpoint]
    pi_seed_db_id: Optional[int]

    recovery: Dict[str, Any]
    recovery_receipt: Optional[Dict[str, Any]]

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

        fw_proofs: Dict[str, Tuple[Endpoint, int, bool]] = {}
        for name in PPB_WINDOWS:
            node = req_dict(fw.get(name), f"db_id={db_id}.fw.{name}")
            valid = node.get("valid") is True
            sample_count = req_int(node.get("sample_count"), f"db_id={db_id}.fw.{name}.sample_count", 0)
            anchor = Endpoint.parse(node.get("anchor"), f"db_id={db_id}.fw.{name}.anchor") if valid else Endpoint(0, 0, 0)
            fw_proofs[name] = (anchor, sample_count, valid)

        buckets_obj = req_dict(stats.get("ppb_buckets"), f"db_id={db_id}.ppb_buckets")
        buckets: Dict[str, Optional[Tuple[float, int]]] = {}
        for name in PPB_WINDOWS:
            node = buckets_obj.get(name)
            proof_valid = fw_proofs[name][2]
            if node is None:
                if proof_valid:
                    raise ValueError(
                        f"db_id={db_id}.ppb_buckets.{name} absent while firmware proof is valid"
                    )
                buckets[name] = None
                continue
            node = req_dict(node, f"db_id={db_id}.ppb_buckets.{name}")
            buckets[name] = (
                req_float(node.get("ppb"), f"db_id={db_id}.ppb_buckets.{name}.ppb"),
                req_int(node.get("sample_count"), f"db_id={db_id}.ppb_buckets.{name}.sample_count", 0),
            )
            if not proof_valid:
                raise ValueError(
                    f"db_id={db_id}.ppb_buckets.{name} published while firmware proof is invalid"
                )

        total_node = req_dict(buckets_obj.get("total"), f"db_id={db_id}.ppb_buckets.total")
        buckets["total"] = (
            req_float(total_node.get("ppb"), f"db_id={db_id}.ppb_buckets.total.ppb"),
            req_int(total_node.get("sample_count"), f"db_id={db_id}.ppb_buckets.total.sample_count", 0),
        )

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
            if pi.get("__analyzer_compact_history") is True:
                pi_second_count = req_int(
                    pi.get("__analyzer_second_history_count"),
                    f"db_id={db_id}.pi.__analyzer_second_history_count",
                    0,
                )
                pi_minute_count = req_int(
                    pi.get("__analyzer_minute_history_count"),
                    f"db_id={db_id}.pi.__analyzer_minute_history_count",
                    0,
                )
                if pi_second_count > SECOND_CAPACITY or pi_minute_count > MINUTE_CAPACITY:
                    raise ValueError(f"db_id={db_id}: Pi PPB ring exceeds capacity")
                if pi_second_count:
                    pi_second_tail = Endpoint.parse(
                        pi.get("__analyzer_second_history_tail"), f"db_id={db_id}.pi.second_tail"
                    )
                if pi_minute_count:
                    pi_minute_tail = Endpoint.parse(
                        pi.get("__analyzer_minute_history_tail"), f"db_id={db_id}.pi.minute_tail"
                    )
            else:
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
            pps_count=opt_int(root.get("pps_count")),
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
            recovery_receipt=parse_recovery_receipt_optional(
                root.get("recovery_receipt"), f"db_id={db_id}.recovery_receipt"
            ),
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
    recovery_receipts: int = 0
    recovery_receipt_modes: Counter[str] = field(default_factory=Counter)

    last_row: Optional[Row] = None
    last_campaign: Dict[str, Row] = field(default_factory=dict)
    last_recovery_generation: Optional[int] = None
    pending_recovery_generation: Optional[int] = None
    row_identity_by_db_id: Dict[int, Dict[str, Any]] = field(default_factory=dict)
    receipt_status_by_db_id: Dict[int, str] = field(default_factory=dict)

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
        total_bucket = row.buckets["total"]
        assert total_bucket is not None
        published_total, total_samples = total_bucket
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
        bucket = row.buckets[name]
        if not valid:
            if bucket is not None:
                audit.problem("ppb_value_without_valid_proof", row, f"{name} published while producer proof is invalid")
                audit.proof("ppb_windows", "failed")
            else:
                audit.proof("ppb_windows", "unavailable")
            continue
        if bucket is None:
            audit.problem("ppb_value_missing_with_valid_proof", row, f"{name} producer proof is valid but bucket is absent")
            audit.proof("ppb_windows", "failed")
            continue
        published, published_samples = bucket
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
        # Modern PHOTONS keeps literal Pi recovery rings in config.PHOTONS_RECOVERY.
        # Their absence from canonical PHOTONS is intentional; recovery_receipt is
        # the historical attestation of the private image actually used.
        pass
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
        audit.proof("legacy_embedded_pi_checkpoint", "failed" if pi_failed else ("passed" if row.pi_recoverable else "bounded"))

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



def check_recovery_receipt(audit: Audit, row: Row) -> str:
    """Prove one immutable Pi recovery receipt against its canonical proof row."""
    receipt = row.recovery_receipt
    if receipt is None:
        return "absent"

    audit.recovery_receipts += 1
    mode = str(receipt.get("recovery_mode") or "").strip().upper() or "UNKNOWN"
    audit.recovery_receipt_modes[mode] += 1
    failed = False
    bounded = False
    if mode not in {"HELD_RESTORE", "PENDING_RESTORE_PROOF"}:
        audit.problem("recovery_receipt_mode", row, f"mode={mode!r}")
        failed = True
    if receipt.get("producer_mutated") is not True:
        audit.problem(
            "recovery_receipt_producer_mutation",
            row,
            f"producer_mutated={receipt.get('producer_mutated')!r}",
        )
        failed = True

    src = req_dict(receipt.get("source"), f"db_id={row.db_id}.receipt.source")
    checkpoint = req_dict(receipt.get("checkpoint"), f"db_id={row.db_id}.receipt.checkpoint")
    boundary = req_dict(receipt.get("boundary"), f"db_id={row.db_id}.receipt.boundary")
    proof = req_dict(receipt.get("proof"), f"db_id={row.db_id}.receipt.proof")

    source_id = req_int(src.get("detail_id"), "receipt.source.detail_id", 1)
    source_sequence = req_int(src.get("sequence"), "receipt.source.sequence", 1)
    source_reset = req_int(src.get("reset_count"), "receipt.source.reset_count", 0)
    source_update = req_int(src.get("update_count"), "receipt.source.update_count", 1)

    source_identity = audit.row_identity_by_db_id.get(source_id)
    if source_identity is None:
        bounded = True
        audit.event(
            "recovery_receipt_source_out_of_scope",
            row,
            f"mode={mode} source_detail_id={source_id}",
        )
    elif (
        int(source_identity["sequence"]) != source_sequence
        or int(source_identity["reset_count"]) != source_reset
        or int(source_identity["update_count"]) != source_update
    ):
        audit.problem(
            "recovery_receipt_source_identity",
            row,
            f"receipt={source_id}:{source_sequence}/{source_reset}/{source_update} durable={source_identity}",
        )
        failed = True

    if (
        req_int(boundary.get("detail_id"), "receipt.boundary.detail_id", 1) != row.db_id
        or req_int(boundary.get("sequence"), "receipt.boundary.sequence", 1) != row.sequence
        or req_int(boundary.get("reset_count"), "receipt.boundary.reset_count", 0) != row.reset_count
        or req_int(boundary.get("update_count"), "receipt.boundary.update_count", 1) != row.update_count
    ):
        audit.problem("recovery_receipt_boundary_identity", row, f"boundary={boundary!r}")
        failed = True

    generation = opt_int(receipt.get("generation"))
    if generation is None or generation <= 0 or generation != opt_int(row.recovery.get("generation")):
        audit.problem(
            "recovery_receipt_generation",
            row,
            f"receipt={generation} firmware={row.recovery.get('generation')}",
        )
        failed = True

    if (
        str(proof.get("contract") or "") != "EXACT_SOURCE_N_PLUS_1"
        or proof.get("durable") is not True
        or proof.get("fresh_physical_ancestry") is not True
    ):
        audit.problem("recovery_receipt_proof_contract", row, f"proof={proof!r}")
        failed = True
    if not (
        row.sequence == source_sequence + 1
        and row.reset_count == source_reset
        and row.update_count == source_update + 1
    ):
        audit.problem(
            "recovery_receipt_exact_n_plus_1",
            row,
            f"source={source_sequence}/{source_reset}/{source_update} "
            f"boundary={row.sequence}/{row.reset_count}/{row.update_count}",
        )
        failed = True

    # Cross-check the receipt against firmware-authored recovery ancestry on the
    # same proof row.  Receipt proof is durable before ACK, so proof_pending=true
    # here is lawful; later canonical testimony closes the ACK transaction.
    r = row.recovery
    if (
        r.get("restored") is not True
        or r.get("fresh_physical_ancestry") is not True
        or opt_int(r.get("source_sequence")) != source_sequence
        or opt_int(r.get("source_reset_count")) != source_reset
        or opt_int(r.get("source_update_count")) != source_update
    ):
        audit.problem("recovery_receipt_firmware_ancestry", row, f"recovery={r!r}")
        failed = True

    if str(checkpoint.get("schema") or "") != PI_PPB_SCHEMA:
        audit.problem("recovery_receipt_checkpoint_schema", row, str(checkpoint.get("schema")))
        failed = True
    digest_available = checkpoint.get("digest_available")
    if digest_available is True:
        if not valid_sha256(checkpoint.get("sha256")):
            audit.problem(
                "recovery_receipt_checkpoint_digest", row, f"sha256={checkpoint.get('sha256')!r}"
            )
            failed = True
        checkpoint_reset = req_int(checkpoint.get("reset_count"), "receipt.checkpoint.reset_count", 0)
        checkpoint_update = req_int(checkpoint.get("update_count"), "receipt.checkpoint.update_count", 1)
        checkpoint_current = req_int(
            checkpoint.get("current_sequence"), "receipt.checkpoint.current_sequence", 0
        )
        second_count = req_int(checkpoint.get("second_count"), "receipt.checkpoint.second_count", 0)
        expected_second = req_int(
            checkpoint.get("expected_second_count"), "receipt.checkpoint.expected_second_count", 0
        )
        minute_count = req_int(checkpoint.get("minute_count"), "receipt.checkpoint.minute_count", 0)
        expected_minute = req_int(
            checkpoint.get("expected_minute_count"), "receipt.checkpoint.expected_minute_count", 0
        )
        if (
            checkpoint_reset != source_reset
            or checkpoint_update != source_update
            or checkpoint_current != source_update
            or second_count > SECOND_CAPACITY
            or minute_count > MINUTE_CAPACITY
            or expected_second > SECOND_CAPACITY
            or expected_minute > MINUTE_CAPACITY
            or second_count > expected_second
            or minute_count > expected_minute
            or (
                checkpoint.get("recoverable") is True
                and (second_count != expected_second or minute_count != expected_minute)
            )
        ):
            audit.problem(
                "recovery_receipt_checkpoint_identity",
                row,
                f"source={source_reset}/{source_update} checkpoint={checkpoint_reset}/{checkpoint_update}/"
                f"{checkpoint_current} second={second_count}/{expected_second} minute={minute_count}/{expected_minute}",
            )
            failed = True
    elif digest_available is False:
        if checkpoint.get("sha256") is not None or not str(checkpoint.get("digest_reason") or "").strip():
            audit.problem("recovery_receipt_checkpoint_digest_state", row, f"checkpoint={checkpoint!r}")
            failed = True
        else:
            bounded = True
            audit.event(
                "recovery_receipt_checkpoint_digest_unavailable",
                row,
                str(checkpoint.get("digest_reason")),
            )
    else:
        audit.problem("recovery_receipt_checkpoint_digest_state", row, f"checkpoint={checkpoint!r}")
        failed = True

    history = receipt.get("history")
    if mode == "HELD_RESTORE" and history is None:
        audit.problem("recovery_receipt_history_missing", row, "HELD_RESTORE receipt lacks history summary")
        failed = True
    if history is not None:
        h = req_dict(history, "receipt.history")
        staged_second = req_int(h.get("staged_second_count"), "receipt.history.staged_second_count", 0)
        staged_minute = req_int(h.get("staged_minute_count"), "receipt.history.staged_minute_count", 0)
        surrendered_second = req_int(
            h.get("surrendered_second_endpoints"), "receipt.history.surrendered_second_endpoints", 0
        )
        surrendered_minute = req_int(
            h.get("surrendered_minute_endpoints"), "receipt.history.surrendered_minute_endpoints", 0
        )
        truncated = h.get("truncated") is True
        if staged_second > SECOND_CAPACITY or staged_minute > MINUTE_CAPACITY:
            audit.problem("recovery_receipt_history_capacity", row, f"history={h!r}")
            failed = True
        if not truncated and (surrendered_second or surrendered_minute):
            audit.problem("recovery_receipt_history_surrender", row, f"history={h!r}")
            failed = True
        if digest_available is True:
            expected_second = req_int(
                checkpoint.get("expected_second_count"), "receipt.checkpoint.expected_second_count", 0
            )
            expected_minute = req_int(
                checkpoint.get("expected_minute_count"), "receipt.checkpoint.expected_minute_count", 0
            )
            if (
                staged_second + surrendered_second != expected_second
                or staged_minute + surrendered_minute != expected_minute
                or bool(checkpoint.get("recoverable")) == truncated
            ):
                audit.problem(
                    "recovery_receipt_history_geometry",
                    row,
                    f"checkpoint_recoverable={checkpoint.get('recoverable')} truncated={truncated} "
                    f"staged/surrendered={staged_second}+{surrendered_second}/"
                    f"{staged_minute}+{surrendered_minute} expected={expected_second}/{expected_minute}",
                )
                failed = True
        if truncated:
            audit.event(
                "recovery_receipt_literal_suffix",
                row,
                f"scope={h.get('scope')} staged={staged_second}/{staged_minute} "
                f"surrendered={surrendered_second}/{surrendered_minute}",
            )

    if mode == "HELD_RESTORE" and digest_available is not True:
        audit.problem(
            "recovery_receipt_checkpoint_digest_missing",
            row,
            "HELD_RESTORE must fingerprint its exact source checkpoint",
        )
        failed = True

    campaign = receipt.get("campaign")
    if campaign is not None:
        c = req_dict(campaign, "receipt.campaign")
        if (
            str(c.get("campaign") or "") != (row.campaign or "")
            or req_int(c.get("public_count"), "receipt.campaign.public_count", 1)
            != row.campaign_public_count
            or req_int(c.get("start_after_sequence"), "receipt.campaign.start_after_sequence", 1)
            != row.campaign_start_after_sequence
        ):
            audit.problem("recovery_receipt_campaign_identity", row, f"campaign={c!r}")
            failed = True

    status = "failed" if failed else "bounded" if bounded else "passed"
    audit.receipt_status_by_db_id[row.db_id] = status
    audit.proof("recovery_receipt", status)
    audit.event(
        "recovery_receipt",
        row,
        f"mode={mode} source={source_id} boundary={row.db_id} generation={generation} "
        f"status={status}",
    )
    return status


def check_recovery_testimony(audit: Audit, row: Row) -> None:
    r = row.recovery
    if not r or r.get("restored") is not True:
        return

    generation = opt_int(r.get("generation"))
    if generation is None:
        audit.problem("recovery_generation_missing", row, "restored=true without generation")
        audit.proof("recovery_lineage", "failed")
        return

    source_sequence = opt_int(r.get("source_sequence"))
    source_update = opt_int(r.get("source_update_count"))
    source_reset = opt_int(r.get("source_reset_count"))
    source_laps = opt_int(r.get("source_lap_count"))
    source_total = opt_int(r.get("source_total_lap_gnss_ns"))

    exact_n_plus_1 = (
        source_sequence is not None
        and source_update is not None
        and source_reset is not None
        and source_laps is not None
        and source_total is not None
        and row.sequence == source_sequence + 1
        and row.update_count == source_update + 1
        and row.reset_count == source_reset
        and row.lap_count >= source_laps
        and row.total_lap_gnss_ns >= source_total
        and r.get("fresh_physical_ancestry") is True
    )

    committed = r.get("proof_committed") is True and r.get("proof_pending") is not True
    pending = r.get("proof_pending") is True and r.get("proof_committed") is not True

    # Recovery testimony persists on every later row. The first generation seen
    # in a campaign is historical context unless this row is itself source N+1.
    if audit.last_recovery_generation is None and not exact_n_plus_1:
        audit.last_recovery_generation = generation
        audit.event(
            "recovery_context_preexisting",
            row,
            f"generation={generation} source_sequence={source_sequence} current_sequence={row.sequence}",
        )
        return

    # A newly restored source N+1 row may be published before the recovery proof
    # transaction commits. Follow the same generation until that proof commits.
    if generation == audit.last_recovery_generation:
        if audit.pending_recovery_generation != generation:
            return
        if committed:
            audit.event(
                "recovery_proof_committed",
                row,
                f"generation={generation} source_sequence={source_sequence} current_sequence={row.sequence}",
            )
            audit.proof("recovery_lineage", "passed")
            audit.pending_recovery_generation = None
        elif not pending:
            audit.problem(
                "recovery_proof_state",
                row,
                f"generation={generation} pending episode became contradictory: "
                f"proof_pending={r.get('proof_pending')} proof_committed={r.get('proof_committed')}",
            )
            audit.proof("recovery_lineage", "failed")
            audit.pending_recovery_generation = None
        return

    audit.last_recovery_generation = generation

    if exact_n_plus_1:
        if committed:
            audit.event(
                "recovery_exact_n_plus_1",
                row,
                f"generation={generation} source_sequence={source_sequence}->{row.sequence} "
                f"source_update={source_update}->{row.update_count} proof=committed",
            )
            audit.proof("recovery_lineage", "passed")
            return
        if pending:
            audit.event(
                "recovery_exact_n_plus_1",
                row,
                f"generation={generation} source_sequence={source_sequence}->{row.sequence} "
                f"source_update={source_update}->{row.update_count} proof=pending",
            )
            audit.proof("recovery_lineage", "bounded")
            audit.pending_recovery_generation = generation
            return
        audit.problem(
            "recovery_proof_state",
            row,
            f"generation={generation} exact N+1 ancestry but proof_pending={r.get('proof_pending')} "
            f"proof_committed={r.get('proof_committed')}",
        )
        audit.proof("recovery_lineage", "failed")
        return

    audit.problem(
        "recovery_ancestry",
        row,
        f"new generation={generation} source seq/update/reset={source_sequence}/{source_update}/{source_reset} "
        f"current={row.sequence}/{row.update_count}/{row.reset_count}",
    )
    audit.proof("recovery_lineage", "failed")

def classify(prev: Row, cur: Row) -> str:
    # Receipt-bearing N+1 is a producer-resurrection boundary even though the
    # restored sequence itself is intentionally continuous with the durable source.
    if cur.recovery_receipt is not None:
        return "RECOVERY_RECEIPT_EXACT_N_PLUS_1"
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
        if cur.recovery_receipt is not None:
            return "PHYSICAL_REBOOT_RECEIPTED_RECOVERY"
        if recovery.get("restored") is True and recovery.get("proof_committed") is True:
            return "PHYSICAL_REBOOT_PROVED_RECOVERY"
        if cur.update_count == 1:
            return "PHYSICAL_REBOOT_NEW_EPOCH"
        return "PHYSICAL_REBOOT_UNPROVED"
    return "ADJACENT_CHRONOLOGY_MISMATCH"


def check_adjacent(audit: Audit, prev: Row, cur: Row) -> str:
    boundary = classify(prev, cur)
    audit.boundaries[boundary] += 1

    if boundary == "RECOVERY_RECEIPT_EXACT_N_PLUS_1":
        r = cur.recovery
        source_sequence = opt_int(r.get("source_sequence"))
        source_update = opt_int(r.get("source_update_count"))
        source_reset = opt_int(r.get("source_reset_count"))
        source_laps = opt_int(r.get("source_lap_count"))
        source_total = opt_int(r.get("source_total_lap_gnss_ns"))
        failed = bool(audit.receipt_status_by_db_id.get(cur.db_id) == "failed")
        if (
            source_sequence is None
            or source_update is None
            or source_reset is None
            or source_laps is None
            or source_total is None
            or cur.sequence != source_sequence + 1
            or cur.update_count != source_update + 1
            or cur.reset_count != source_reset
            or cur.lap_count < source_laps
            or cur.total_lap_gnss_ns < source_total
        ):
            audit.problem(
                "recovery_receipt_boundary_ancestry",
                cur,
                f"source={source_sequence}/{source_reset}/{source_update}/{source_laps}/{source_total} "
                f"boundary={cur.sequence}/{cur.reset_count}/{cur.update_count}/{cur.lap_count}/{cur.total_lap_gnss_ns}",
            )
            failed = True
        audit.event(
            "receipted_recovery_boundary",
            cur,
            f"generation={r.get('generation')} source_sequence={source_sequence}->{cur.sequence} "
            f"source_update={source_update}->{cur.update_count}",
        )
        audit.proof("instrument_lineage", "failed" if failed else "passed")

    elif boundary == "NORMAL_CONTINUATION":
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

    elif boundary in {"PHYSICAL_REBOOT_PROVED_RECOVERY", "PHYSICAL_REBOOT_RECEIPTED_RECOVERY"}:
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
        audit.event("proved_recovery_boundary", cur, f"generation={r.get('generation')} source_update={source_update} current_update={cur.update_count}")
        if failed:
            audit.proof("recovery_lineage", "failed")

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
            lawful = bool(
                boundary in {
                    "DURABLE_OBSERVATION_GAP",
                    "PHYSICAL_REBOOT_PROVED_RECOVERY",
                    "PHYSICAL_REBOOT_RECEIPTED_RECOVERY",
                    "RECOVERY_RECEIPT_EXACT_N_PLUS_1",
                    "PHYSICAL_REBOOT_NEW_EPOCH",
                }
                or row.recovery_receipt is not None
            )
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
    audit.row_identity_by_db_id[row.db_id] = {
        "sequence": row.sequence,
        "reset_count": row.reset_count,
        "update_count": row.update_count,
        "campaign": row.campaign,
        "public_count": row.campaign_public_count,
    }
    if row.recovery_receipt is not None:
        check_recovery_receipt(audit, row)
    check_recovery_testimony(audit, row)
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
    """Stream only proof-surface JSON, not the enlarged canonical payload."""
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
        SELECT
            id,
            ts,
            payload -> 'sequence' AS sequence_payload,
            payload -> 'publish_count' AS publish_count_payload,
            payload -> 'pps_count' AS pps_count_payload,
            payload -> 'campaign' AS campaign_payload,
            payload -> 'recovery_receipt' AS recovery_receipt_payload,
            payload #> '{{photons,schema}}' AS photons_schema_payload,
            payload #> '{{photons,snapshot_ok}}' AS snapshot_ok_payload,
            payload #> '{{photons,stats}}' AS stats_payload,
            payload #> '{{photons,science}}' AS science_payload,
            payload #> '{{photons,recovery}}' AS recovery_payload,
            (payload -> 'ppb_restore_checkpoint') - 'second_history' - 'minute_history'
                AS ppb_restore_checkpoint_payload,
            jsonb_array_length(payload #> '{{ppb_restore_checkpoint,second_history}}')
                AS ppb_second_history_count,
            jsonb_array_length(payload #> '{{ppb_restore_checkpoint,minute_history}}')
                AS ppb_minute_history_count,
            (payload #> '{{ppb_restore_checkpoint,second_history}}') -> -1
                AS ppb_second_history_tail,
            (payload #> '{{ppb_restore_checkpoint,minute_history}}') -> -1
                AS ppb_minute_history_tail
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
        def json_value(value: Any) -> Any:
            if isinstance(value, str):
                stripped = value.lstrip()
                if stripped.startswith("{") or stripped.startswith("["):
                    return json.loads(value)
            return value

        for row in cur:
            checkpoint = json_value(row["ppb_restore_checkpoint_payload"])
            if checkpoint is not None:
                checkpoint = req_dict(checkpoint, "SQL-projected ppb_restore_checkpoint")
                checkpoint["__analyzer_compact_history"] = True
                checkpoint["__analyzer_second_history_count"] = row["ppb_second_history_count"]
                checkpoint["__analyzer_minute_history_count"] = row["ppb_minute_history_count"]
                checkpoint["__analyzer_second_history_tail"] = json_value(row["ppb_second_history_tail"])
                checkpoint["__analyzer_minute_history_tail"] = json_value(row["ppb_minute_history_tail"])

            photons: Dict[str, Any] = {
                "schema": json_value(row["photons_schema_payload"]),
                "snapshot_ok": json_value(row["snapshot_ok_payload"]),
                "stats": json_value(row["stats_payload"]),
                "science": json_value(row["science_payload"]),
                "recovery": json_value(row["recovery_payload"]),
            }
            payload: Dict[str, Any] = {
                "schema": PHOTONS_SCHEMA,
                "sequence": json_value(row["sequence_payload"]),
                "publish_count": json_value(row["publish_count_payload"]),
                "pps_count": json_value(row["pps_count_payload"]),
                "photons": photons,
                "ppb_restore_checkpoint": checkpoint,
            }
            campaign = json_value(row["campaign_payload"])
            if campaign is not None:
                payload["campaign"] = campaign

            recovery_receipt = json_value(row["recovery_receipt_payload"])
            if recovery_receipt is not None:
                payload["recovery_receipt"] = recovery_receipt

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
    print(f"Legacy embedded Pi checkpoint checks: {a.pi_checkpoint_checks:,}")
    print(f"Recovery receipts:     {a.recovery_receipts:,}")

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

    print("\nRECOVERY RECEIPTS")
    if a.recovery_receipt_modes:
        for mode, count in a.recovery_receipt_modes.most_common():
            print(f"  {mode:<42s} {count:,}")
    else:
        print("  none (pre-receipt history may still be bounded by firmware ancestry)")

    print("\nPROOF LEDGER")
    for name in (
        "row_internal",
        "instrument_lineage",
        "recovery_lineage",
        "recovery_receipt",
        "ppb_windows",
        "legacy_embedded_pi_checkpoint",
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
