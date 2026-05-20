"""
ZPNet Clock Witness Cycles — live CLOCK_WITNESS cycle audit.

This is the unified live-stream sibling of tests/raw_cycles.py.  Instead of
reading TIMEBASE rows from Postgres, it registers a temporary Pi-side pub/sub
process, subscribes to CLOCK_WITNESS, and prints compact real-time rows for the
witness lanes that are actually producing new historical edge facts.

CLOCK_WITNESS currently carries up to three lanes:

  • VCLOCK — hosted QTimer1 CH1 side-bell witness, without taking over the
             sovereign QTimer1 CH2 / TimePop rail.
  • OCXO1  — QTimer2 CH0 witness mode, after process_interrupt releases lane.
  • OCXO2  — QTimer3 CH3 witness mode, after process_interrupt releases lane.

The firmware publishes historical edge facts.  The report cadence is
VCLOCK/TimePop time; the measured data is the already-completed interval from
one 10,000,000-count edge to the next.

Usage, from the zpnet-suite repo root on the Pi:

    python tests/clock_witness_cycles.py
    python tests/clock_witness_cycles.py --lane all
    python tests/clock_witness_cycles.py --lane vclock
    python tests/clock_witness_cycles.py --lane ocxo1
    python tests/clock_witness_cycles.py --lane ocxo2
    python tests/clock_witness_cycles.py --lane ocxo
    python tests/clock_witness_cycles.py --limit 200
    python tests/clock_witness_cycles.py --include-stale
    python tests/clock_witness_cycles.py --debug-shape
    python tests/clock_witness_cycles.py --no-refresh

Typical firmware setup:

    .tc witness vclock_start

    .tc interrupt ocxo_stop lane=both
    .tc witness ocxo_start lane=both

    python tests/clock_witness_cycles.py

Notes:
  • This tool creates a temporary PID-unique PI subsystem named CLOCK_WITNESS_CYCLES_<pid>.
  • By default it asks PUBSUB to REFRESH so the route to this process is live.
  • It does not touch Postgres.
  • Stale lanes are not counted in statistics unless --include-stale is used.
  • Freshness is inferred from edge sequence / edge total changes, so the tool
    still works if firmware omits or clears new_since_last_publish.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Optional

# When run as python tests/clock_witness_cycles.py, make repo root importable.
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(THIS_DIR)
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from zpnet.processes.processes import send_command, server_setup  # noqa: E402


SUBSYSTEM_BASE = "CLOCK_WITNESS_CYCLES"
TOPIC = "CLOCK_WITNESS"

DWT_CYCLE_NS = 125.0 / 126.0  # 1.008 GHz DWT
EXPECTED_COUNTS_PER_SECOND = 10_000_000


class Welford:
    __slots__ = ("n", "mean", "m2", "min_val", "max_val")

    def __init__(self) -> None:
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.min_val = float("inf")
        self.max_val = float("-inf")

    def update(self, x: float) -> None:
        self.n += 1
        d1 = x - self.mean
        self.mean += d1 / self.n
        d2 = x - self.mean
        self.m2 += d1 * d2
        self.min_val = min(self.min_val, x)
        self.max_val = max(self.max_val, x)

    @property
    def stddev(self) -> float:
        return math.sqrt(self.m2 / (self.n - 1)) if self.n >= 2 else 0.0

    @property
    def stderr(self) -> float:
        return self.stddev / math.sqrt(self.n) if self.n >= 2 else 0.0


@dataclass(frozen=True)
class LaneSample:
    key: str
    name: str
    enabled: bool
    initialized: bool
    sequence: Optional[int]
    valid: bool
    delta_valid: bool
    residual_valid: bool
    dwt_at_edge: Optional[int]
    actual: Optional[int]
    pred: Optional[int]
    residual: Optional[int]
    service_offset: Optional[int]
    total_at_edge: Optional[int]
    total_at_event: Optional[int]
    count_delta: Optional[int]
    target: Optional[int]
    event_counter: Optional[int]
    event_residual_ticks: Optional[int]
    raw_dwt: Optional[int]
    compare_armed: bool
    irq_pending: bool
    edge_count: Optional[int]
    irq_count: Optional[int]
    new_since_last_publish: bool
    inferred_fresh: bool


def _as_int(v: Any) -> Optional[int]:
    if v is None:
        return None
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def _as_bool(v: Any) -> bool:
    return bool(v) if v is not None else False


def _fmt_int(v: Optional[int], width: int = 0, signed: bool = False) -> str:
    if v is None:
        s = "---"
    else:
        s = f"{v:+,d}" if signed else f"{v:,d}"
    return f"{s:>{width}s}" if width else s


def _fmt_str(v: str, width: int = 0) -> str:
    if len(v) > width and width:
        v = v[:width]
    return f"{v:<{width}s}" if width else v


def _cycles_to_ns(cycles: Optional[int | float]) -> Optional[float]:
    if cycles is None:
        return None
    return float(cycles) * DWT_CYCLE_NS


def _print_welford(name: str, w: Welford, unit: str = "cycles", decimals: int = 3) -> None:
    if w.n == 0:
        print(f"  {name:<42s} no samples")
        return
    print(
        f"  {name:<42s} "
        f"n={w.n:>7,d}  "
        f"mean={w.mean:+14,.{decimals}f} {unit:<6s}  "
        f"sd={w.stddev:11,.{decimals}f}  "
        f"se={w.stderr:10,.{decimals}f}  "
        f"min={w.min_val:+14,.{decimals}f}  "
        f"max={w.max_val:+14,.{decimals}f}"
    )


def _root(payload: Dict[str, Any]) -> Dict[str, Any]:
    # process runtime supplies the payload directly, but this also tolerates
    # copied listener lines shaped as {"topic": ..., "payload": ...}.
    if not isinstance(payload, dict):
        return {}
    inner = payload.get("payload")
    return inner if isinstance(inner, dict) else payload


def _state(payload: Dict[str, Any]) -> Dict[str, Any]:
    root = _root(payload)
    state = root.get("state")
    return state if isinstance(state, dict) else root



def _find_lane_dict(obj: Dict[str, Any], lane_key: str, depth: int = 0) -> Dict[str, Any]:
    """Find a lane object even if firmware nests CLOCK_WITNESS state.

    Expected new shape is state.vclock / state.ocxo1 / state.ocxo2, but early
    firmware builds may publish state.clock_witness.vclock, state.ocxo_witness
    or another one-level/two-level wrapper.  The lane identity is strong enough
    that a bounded recursive search is safe and makes the tool useful during
    firmware churn.
    """
    if not isinstance(obj, dict) or depth > 4:
        return {}

    direct = obj.get(lane_key)
    if isinstance(direct, dict):
        return direct

    aliases = {
        "vclock": ("vclock", "vclk", "vclock_witness"),
        "ocxo1": ("ocxo1", "o1"),
        "ocxo2": ("ocxo2", "o2"),
    }
    for key in aliases.get(lane_key, (lane_key,)):
        candidate = obj.get(key)
        if isinstance(candidate, dict):
            return candidate

    for value in obj.values():
        if isinstance(value, dict):
            found = _find_lane_dict(value, lane_key, depth + 1)
            if found:
                return found

    return {}


def _debug_shape(obj: Any, max_depth: int = 3, depth: int = 0) -> Any:
    if depth >= max_depth:
        return "..."
    if isinstance(obj, dict):
        out: Dict[str, Any] = {}
        for key, value in list(obj.items())[:20]:
            if isinstance(value, dict):
                out[key] = _debug_shape(value, max_depth, depth + 1)
            elif isinstance(value, list):
                out[key] = f"list[{len(value)}]"
            else:
                out[key] = type(value).__name__
        return out
    return type(obj).__name__


def _counts_per_second(payload: Dict[str, Any]) -> int:
    root = _root(payload)
    return _as_int(root.get("counts_per_second")) or EXPECTED_COUNTS_PER_SECOND


def _lane_name(lane: Dict[str, Any], lane_key: str) -> str:
    name = lane.get("name")
    if isinstance(name, str) and name:
        return name
    return lane_key.upper()


def _lane_sample(
    state: Dict[str, Any],
    lane_key: str,
    last_seen_total: Optional[int],
    last_seen_sequence: Optional[int],
) -> LaneSample:
    lane = _find_lane_dict(state, lane_key)
    last = lane.get("last")
    if not isinstance(last, dict):
        last = {}

    if lane_key == "vclock":
        total_at_edge = _as_int(last.get("clock_total_at_edge"))
        if total_at_edge is None:
            total_at_edge = _as_int(last.get("vclock_total_at_edge"))

        total_at_event = _as_int(last.get("clock_total_at_event"))
        if total_at_event is None:
            total_at_event = _as_int(last.get("vclock_total_at_event"))

        target = _as_int(last.get("target_counter32"))
        event_counter = _as_int(last.get("counter32_at_event"))
        event_residual = _as_int(last.get("counter32_residual_ticks"))
        raw_dwt = _as_int(last.get("isr_entry_dwt_raw"))

    else:
        total_at_edge = _as_int(last.get("ocxo_total_at_edge"))
        total_at_event = _as_int(last.get("ocxo_total_at_isr"))
        target = _as_int(last.get("target_low16"))
        event_counter = _as_int(last.get("counter_low16_at_isr"))
        event_residual = _as_int(last.get("service_offset_ticks"))
        raw_dwt = None

    sequence = _as_int(last.get("sequence"))

    count_delta = None
    if total_at_edge is not None and last_seen_total is not None:
        count_delta = total_at_edge - last_seen_total

    # Firmware supplies new_since_last_publish when it can, but the Python tool
    # must not depend on that flag.  Infer freshness from the identity fields
    # that actually matter for historical edge facts.
    inferred_fresh = False
    if _as_bool(last.get("valid")):
        if sequence is not None and sequence != last_seen_sequence:
            inferred_fresh = True
        elif total_at_edge is not None and total_at_edge != last_seen_total:
            inferred_fresh = True

    service_offset = _as_int(last.get("service_offset_ticks"))
    if service_offset is None:
        service_offset = event_residual

    return LaneSample(
        key=lane_key,
        name=_lane_name(lane, lane_key),
        enabled=_as_bool(lane.get("enabled")),
        initialized=_as_bool(lane.get("initialized")),
        sequence=sequence,
        valid=_as_bool(last.get("valid")),
        delta_valid=_as_bool(last.get("delta_valid")),
        residual_valid=_as_bool(last.get("residual_valid")),
        dwt_at_edge=_as_int(last.get("dwt_at_edge")),
        actual=_as_int(last.get("dwt_delta_cycles")),
        pred=_as_int(last.get("dwt_pred_cycles")),
        residual=_as_int(last.get("dwt_residual_cycles")),
        service_offset=service_offset,
        total_at_edge=total_at_edge,
        total_at_event=total_at_event,
        count_delta=count_delta,
        target=target,
        event_counter=event_counter,
        event_residual_ticks=event_residual,
        raw_dwt=raw_dwt,
        compare_armed=_as_bool(lane.get("compare_armed")),
        irq_pending=_as_bool(lane.get("irq_pending")),
        edge_count=_as_int(lane.get("edge_count")),
        irq_count=_as_int(lane.get("irq_count")),
        new_since_last_publish=_as_bool(lane.get("new_since_last_publish")),
        inferred_fresh=inferred_fresh,
    )


def _selected_lanes(lane_filter: str) -> tuple[str, ...]:
    f = lane_filter.lower()
    if f in ("all", "clock", "clocks"):
        return ("vclock", "ocxo1", "ocxo2")
    if f in ("ocxo", "ocxos", "both", "12"):
        return ("ocxo1", "ocxo2")
    if f in ("v", "vclock", "vclk"):
        return ("vclock",)
    if f in ("1", "o1", "ocxo1"):
        return ("ocxo1",)
    if f in ("2", "o2", "ocxo2"):
        return ("ocxo2",)
    return ("vclock", "ocxo1", "ocxo2")


def _socket_dir() -> str:
    return "/tmp"


def _command_socket_path(subsystem: str) -> str:
    return f"{_socket_dir()}/zpnet-{subsystem.lower()}-command.sock"


def _pubsub_socket_path(subsystem: str) -> str:
    return f"{_socket_dir()}/zpnet-{subsystem.lower()}-pubsub.sock"


def _wait_for_local_sockets(subsystem: str, timeout_s: float = 3.0) -> None:
    deadline = time.monotonic() + timeout_s
    command_path = _command_socket_path(subsystem)
    pubsub_path = _pubsub_socket_path(subsystem)

    while time.monotonic() < deadline:
        if os.path.exists(command_path) and os.path.exists(pubsub_path):
            return
        time.sleep(0.05)

    raise RuntimeError(
        "local process sockets did not appear: "
        f"{command_path!r}, {pubsub_path!r}"
    )


def _route_is_live(subsystem: str) -> bool:
    resp = send_command(machine="PI", subsystem="PUBSUB", command="REPORT")
    routes = resp.get("payload", {}).get("routes", [])
    for row in routes:
        if (
            row.get("machine") == "PI"
            and row.get("subsystem") == subsystem
            and row.get("topic") == TOPIC
        ):
            return True
    return False


def _refresh_until_routed(subsystem: str, attempts: int = 5) -> None:
    last_resp: Dict[str, Any] | None = None

    for attempt in range(1, attempts + 1):
        time.sleep(0.2)
        last_resp = send_command(machine="PI", subsystem="PUBSUB", command="REFRESH")
        if not last_resp.get("success", False):
            continue
        if _route_is_live(subsystem):
            print(
                f"PUBSUB route confirmed: {TOPIC} -> PI:{subsystem}",
                flush=True,
            )
            return

    raise RuntimeError(
        f"PUBSUB route was not installed for {TOPIC} -> PI:{subsystem}; "
        f"last_refresh={last_resp}"
    )


class LiveAudit:
    def __init__(self, *, lane_filter: str, limit: int, raw: bool, include_stale: bool, debug_shape: bool) -> None:
        self.lane_filter = lane_filter
        self.lanes = _selected_lanes(lane_filter)
        self.limit = limit
        self.raw = raw
        self.include_stale = include_stale
        self.debug_shape = debug_shape
        self.shape_printed = False
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.publish_count = 0
        self.rows_printed = 0
        self.header_printed = False

        self.last_total_by_lane: Dict[str, Optional[int]] = {
            "vclock": None,
            "ocxo1": None,
            "ocxo2": None,
        }
        self.last_sequence_by_lane: Dict[str, Optional[int]] = {
            "vclock": None,
            "ocxo1": None,
            "ocxo2": None,
        }

        self.stats: Dict[str, Welford] = {}
        for lane in ("vclock", "ocxo1", "ocxo2"):
            self.stats[f"{lane}_actual"] = Welford()
            self.stats[f"{lane}_residual"] = Welford()
            self.stats[f"{lane}_service"] = Welford()
            self.stats[f"{lane}_count_delta"] = Welford()

        self.stats["vclock_minus_ocxo1_actual"] = Welford()
        self.stats["vclock_minus_ocxo2_actual"] = Welford()
        self.stats["ocxo1_minus_ocxo2_actual"] = Welford()
        self.stats["vclock_minus_ocxo1_residual"] = Welford()
        self.stats["vclock_minus_ocxo2_residual"] = Welford()
        self.stats["ocxo1_minus_ocxo2_residual"] = Welford()

    def handle(self, payload: Dict[str, Any]) -> None:
        with self.lock:
            self._handle_locked(payload)

    def _handle_locked(self, payload: Dict[str, Any]) -> None:
        if self.raw:
            print(json.dumps(payload, separators=(",", ":")), flush=True)

        self.publish_count += 1
        state = _state(payload)
        counts_per_second = _counts_per_second(payload)

        if self.debug_shape and not self.shape_printed:
            print("First CLOCK_WITNESS payload shape:")
            print(json.dumps(_debug_shape(payload), indent=2, sort_keys=True))
            print("Resolved state shape:")
            print(json.dumps(_debug_shape(state), indent=2, sort_keys=True))
            self.shape_printed = True

        samples = {
            lane: _lane_sample(
                state,
                lane,
                self.last_total_by_lane[lane],
                self.last_sequence_by_lane[lane],
            )
            for lane in ("vclock", "ocxo1", "ocxo2")
        }

        selected_samples = [samples[k] for k in self.lanes]
        selected_fresh = [s for s in selected_samples if s.inferred_fresh or s.new_since_last_publish]

        # Update identity trackers after the freshness decision for this
        # publication has been made.
        for lane_key, sample in samples.items():
            if sample.valid:
                if sample.total_at_edge is not None:
                    self.last_total_by_lane[lane_key] = sample.total_at_edge
                if sample.sequence is not None:
                    self.last_sequence_by_lane[lane_key] = sample.sequence

        if not self.include_stale and not selected_fresh:
            if self.publish_count == 1 or (self.publish_count % 10) == 0:
                if not self.header_printed:
                    self._print_header(counts_per_second)
                    self.header_printed = True
                found = ",".join(
                    f"{lane}:{'yes' if _find_lane_dict(state, lane) else 'no'}"
                    for lane in self.lanes
                )
                print(
                    f"{self.publish_count:>6d} {'no-new':<7s} "
                    f"received CLOCK_WITNESS but no selected lane advanced "
                    f"(lanes {found})",
                    flush=True,
                )
            return

        if not self.header_printed:
            self._print_header(counts_per_second)
            self.header_printed = True

        stat_samples = selected_samples if self.include_stale else selected_fresh
        self._update_stats(samples, stat_samples)

        for sample in selected_samples:
            if self.include_stale or sample.inferred_fresh or sample.new_since_last_publish:
                self._print_lane_row(self.publish_count, sample)

        if len(self.lanes) > 1:
            self._print_delta_row(self.publish_count, samples)

        self.rows_printed += 1
        if self.limit and self.rows_printed >= self.limit:
            self.stop_event.set()

    def _update_stats(self, all_samples: Dict[str, LaneSample], stat_samples: Iterable[LaneSample]) -> None:
        stat_keys = {s.key for s in stat_samples}

        for sample in stat_samples:
            key = sample.key
            if sample.delta_valid and sample.actual is not None:
                self.stats[f"{key}_actual"].update(float(sample.actual))
            if sample.residual_valid and sample.residual is not None:
                self.stats[f"{key}_residual"].update(float(sample.residual))
            if sample.valid and sample.service_offset is not None:
                self.stats[f"{key}_service"].update(float(sample.service_offset))
            if sample.count_delta is not None:
                self.stats[f"{key}_count_delta"].update(float(sample.count_delta))

        self._update_pair_stats(
            "vclock", "ocxo1", "vclock_minus_ocxo1", all_samples, stat_keys
        )
        self._update_pair_stats(
            "vclock", "ocxo2", "vclock_minus_ocxo2", all_samples, stat_keys
        )
        self._update_pair_stats(
            "ocxo1", "ocxo2", "ocxo1_minus_ocxo2", all_samples, stat_keys
        )

    def _update_pair_stats(
        self,
        left_key: str,
        right_key: str,
        stat_prefix: str,
        samples: Dict[str, LaneSample],
        stat_keys: set[str],
    ) -> None:
        if left_key not in stat_keys or right_key not in stat_keys:
            return

        left = samples[left_key]
        right = samples[right_key]

        if (
            left.delta_valid
            and right.delta_valid
            and left.actual is not None
            and right.actual is not None
        ):
            self.stats[f"{stat_prefix}_actual"].update(float(left.actual - right.actual))

        if (
            left.residual_valid
            and right.residual_valid
            and left.residual is not None
            and right.residual is not None
        ):
            self.stats[f"{stat_prefix}_residual"].update(float(left.residual - right.residual))

    def _print_header(self, counts_per_second: int) -> None:
        print("Live CLOCK_WITNESS cycle audit")
        print("══════════════════════════════")
        print("  Rows are live pub/sub snapshots from CLOCK_WITNESS.")
        print("  act/pred/res are DWT cycles over one completed 10,000,000-count clock interval.")
        print("  svc is service offset in that lane's count ticks at ISR/event capture.")
        print("  cΔ is the lane total-count delta between fresh completed edges; it should be 10,000,000.")
        print("  stale lane snapshots are skipped unless --include-stale is set.")
        print("  freshness is inferred from sequence/edge-total changes, not only firmware markers.")
        print(f"  counts_per_second={counts_per_second:,}  lane_filter={self.lane_filter}")
        print()

        header = (
            f"{'pub':>6s} {'lane':<7s} {'seq':>7s} "
            f"{'act':>13s} {'pred':>13s} {'res':>8s} "
            f"{'svc':>6s} {'cΔ':>11s} {'edge_total':>14s} {'event_total':>14s} "
            f"{'target':>11s} {'event':>11s} {'dwt_edge':>13s} "
            f"{'new':>5s} {'arm':>5s} {'irqp':>5s}"
        )
        print(header)
        print("─" * len(header))

    def _print_lane_row(self, pub: int, sample: LaneSample) -> None:
        print(
            f"{pub:>6d} "
            f"{_fmt_str(sample.name.lower(), 7)} "
            f"{_fmt_int(sample.sequence, 7)} "
            f"{_fmt_int(sample.actual if sample.delta_valid else None, 13)} "
            f"{_fmt_int(sample.pred if sample.residual_valid else None, 13)} "
            f"{_fmt_int(sample.residual if sample.residual_valid else None, 8, signed=True)} "
            f"{_fmt_int(sample.service_offset if sample.valid else None, 6, signed=True)} "
            f"{_fmt_int(sample.count_delta, 11)} "
            f"{_fmt_int(sample.total_at_edge if sample.valid else None, 14)} "
            f"{_fmt_int(sample.total_at_event if sample.valid else None, 14)} "
            f"{_fmt_int(sample.target if sample.valid else None, 11)} "
            f"{_fmt_int(sample.event_counter if sample.valid else None, 11)} "
            f"{_fmt_int(sample.dwt_at_edge if sample.valid else None, 13)} "
            f"{('True' if (sample.new_since_last_publish or sample.inferred_fresh) else 'False'):>5s} "
            f"{str(sample.compare_armed):>5s} "
            f"{str(sample.irq_pending):>5s}",
            flush=True,
        )

    def _print_delta_row(self, pub: int, samples: Dict[str, LaneSample]) -> None:
        pairs = (
            ("vclock", "ocxo1", "v-o1"),
            ("vclock", "ocxo2", "v-o2"),
            ("ocxo1", "ocxo2", "o1-o2"),
        )

        parts = []
        for left_key, right_key, label in pairs:
            if left_key not in self.lanes or right_key not in self.lanes:
                continue
            left = samples[left_key]
            right = samples[right_key]
            if not (
                (left.inferred_fresh or left.new_since_last_publish)
                and (right.inferred_fresh or right.new_since_last_publish)
            ):
                continue
            if left.delta_valid and right.delta_valid and left.actual is not None and right.actual is not None:
                da = left.actual - right.actual
            else:
                da = None
            if (
                left.residual_valid
                and right.residual_valid
                and left.residual is not None
                and right.residual is not None
            ):
                dr = left.residual - right.residual
            else:
                dr = None
            parts.append(f"{label}: Δact={_fmt_int(da, signed=True)} Δres={_fmt_int(dr, signed=True)}")

        if parts:
            print(f"{pub:>6d} {'Δ':<7s} " + " | ".join(parts), flush=True)

    def print_summary(self) -> None:
        with self.lock:
            print()
            print("Summary")
            print("═══════")
            print(f"  publishes received                         = {self.publish_count:,}")
            print(f"  row groups printed                         = {self.rows_printed:,}")
            print(f"  stale included in stats                    = {self.include_stale}")

            for lane in ("vclock", "ocxo1", "ocxo2"):
                _print_welford(f"{lane.upper()} DWT actual", self.stats[f"{lane}_actual"])
                _print_welford(f"{lane.upper()} DWT residual", self.stats[f"{lane}_residual"])
                _print_welford(f"{lane.upper()} service offset", self.stats[f"{lane}_service"], unit="ticks")
                _print_welford(f"{lane.upper()} count delta", self.stats[f"{lane}_count_delta"], unit="ticks")

            _print_welford("VCLOCK actual - OCXO1 actual", self.stats["vclock_minus_ocxo1_actual"])
            _print_welford("VCLOCK actual - OCXO2 actual", self.stats["vclock_minus_ocxo2_actual"])
            _print_welford("OCXO1 actual - OCXO2 actual", self.stats["ocxo1_minus_ocxo2_actual"])
            _print_welford("VCLOCK residual - OCXO1 residual", self.stats["vclock_minus_ocxo1_residual"])
            _print_welford("VCLOCK residual - OCXO2 residual", self.stats["vclock_minus_ocxo2_residual"])
            _print_welford("OCXO1 residual - OCXO2 residual", self.stats["ocxo1_minus_ocxo2_residual"])


def _refresh_pubsub(subsystem: str) -> None:
    _wait_for_local_sockets(subsystem)
    _refresh_until_routed(subsystem)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live CLOCK_WITNESS raw-cycle stream formatter")
    parser.add_argument(
        "--lane",
        default="all",
        choices=(
            "all",
            "clock",
            "clocks",
            "v",
            "vclock",
            "vclk",
            "ocxo",
            "ocxos",
            "both",
            "12",
            "1",
            "2",
            "o1",
            "o2",
            "ocxo1",
            "ocxo2",
        ),
        help="which witness lane(s) to print",
    )
    parser.add_argument("--limit", type=int, default=0, help="stop after N printed row groups")
    parser.add_argument("--raw", action="store_true", help="also dump raw payload JSON before formatted rows")
    parser.add_argument("--include-stale", action="store_true", help="print and count stale lane snapshots")
    parser.add_argument("--debug-shape", action="store_true", help="print first payload shape summary")
    parser.add_argument("--no-refresh", action="store_true", help="do not ask PUBSUB to REFRESH routes")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    audit = LiveAudit(
        lane_filter=args.lane,
        limit=args.limit,
        raw=args.raw,
        include_stale=args.include_stale,
        debug_shape=args.debug_shape,
    )

    def handle_sigint(signum: int, frame: Any) -> None:
        audit.stop_event.set()

    signal.signal(signal.SIGINT, handle_sigint)
    signal.signal(signal.SIGTERM, handle_sigint)

    subsystem = f"{SUBSYSTEM_BASE}_{os.getpid()}"

    server_setup(
        subsystem=subsystem,
        commands={},
        subscriptions={TOPIC: audit.handle},
        blocking=False,
    )

    if not args.no_refresh:
        _refresh_pubsub(subsystem)
    else:
        print(
            "Skipping PUBSUB REFRESH; ensure the route exists manually: "
            f"{TOPIC} -> PI:{subsystem}",
            flush=True,
        )

    print(f"Subscribed to {TOPIC} as PI:{subsystem}. Waiting for publications...", flush=True)

    while not audit.stop_event.is_set():
        time.sleep(0.1)

    audit.print_summary()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
