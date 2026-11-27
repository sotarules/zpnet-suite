"""Regression checks for the 10-MIN comparison convention.

Run with: zt ten_min_population_symmetry_test
Only reads local source files; no database writes, network RPCs or hardware I/O.
Prints one concise result directly to STDOUT.
"""
from __future__ import annotations

import ast
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def source(name):
    return (ROOT / name).read_text()


def function(tree, name):
    return next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name)


def isolate(tree, names, namespace):
    nodes = [function(tree, name) for name in names]
    future = ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)
    module = ast.fix_missing_locations(ast.Module(body=[future, *nodes], type_ignores=[]))
    exec(compile(module, "<ten-minute-test>", "exec"), namespace)
    return namespace


def test_photons_bookends():
    tree = ast.parse(source("zpnet/processes/photons/core.py"))
    ns = isolate(tree, ["_lap_value_from_endpoints", "_lap_bookend", "_missed_start_lap",
                        "_campaign_public_decoration"], {
        "PHOTONS_RECOVERY_60_SEC_SECONDS": 60,
        "PHOTONS_RECOVERY_10_MIN_SECONDS": 600,
        "LANTERN_CAMPAIGN_SCHEMA": "LANTERN_CAMPAIGN_V1",
        "CAMPAIGN_TYPE_LANTERN": "LANTERN",
    })
    row = {"photons": {"stats": {"lap_buckets": {
        "60_sec": {"mean_lap_ns": 5147.4},
        "10_min": {"mean_lap_ns": 5147.6},
    }}}}
    assert ns["_lap_bookend"](row, "10_min") == 5147.6
    assert ns["_lap_bookend"](row, "60_sec") == 5147.4
    assert ns["_lap_bookend"]({"lap_60s_ns": 5146.0}, "60_sec") == 5146.0
    # At sequence 700, the trailing 600-second mean contains 550 seconds
    # at 5100 ns and 50 seconds at 5200 ns; the 60-SEC result differs.
    endpoints = [{"sequence": s, "lap_count": s * 100,
                  "total_lap_gnss_ns": (min(s, 650) * 100 * 5100
                                          + max(0, s - 650) * 100 * 5200)}
                 for s in range(701)]
    checkpoint = {"second_history": endpoints[100:]}
    ten = ns["_missed_start_lap"](checkpoint, 700, "10_min")
    minute = ns["_missed_start_lap"](checkpoint, 700, "60_sec")
    assert abs(ten - (550 * 5100 + 50 * 5200) / 600) < 1e-9
    assert abs(minute - (10 * 5100 + 50 * 5200) / 60) < 1e-9
    assert ns["_missed_start_lap"]({"second_history": endpoints[200:]}, 700, "10_min") is None
    decoration = ns["_campaign_public_decoration"]({
        "campaign": "Sample", "campaign_id": 1, "started_at": "UTC",
        "lap_bookend_window": "10_min"})
    assert decoration["lap_bookend_window"] == "10_min"
    assert ns["_campaign_public_decoration"]({
        "campaign": "Historic", "campaign_id": 2, "started_at": "UTC"
    })["lap_bookend_window"] == "60_sec"
    code = source("zpnet/processes/photons/core.py")
    for required in ('"lap_bookend_window": LANTERN_LAP_BOOKEND_WINDOW',
                     '"lap_bookend_window": master_payload.get("lap_bookend_window", "60_sec")',
                     '"latest_lap_ns": _lap_bookend(',
                     '"lap_bookend_window": payload.get("lap_bookend_window", "60_sec")'):
        assert required in code, required


def test_stats():
    tree = ast.parse(source("zpnet/stats/core.py"))
    class Last:
        def __init__(self, row): self.row = row
        def get(self): return self.row
    closed_lantern = {"campaign": "Previous", "start_lap_ns": 5100.0,
                      "end_lap_ns": 5110.0, "lap_bookend_window": "60_sec"}
    closed_tempest = {"campaign": "Previous", "start_ppb": {"ocxo1": 0, "ocxo2": 0},
                      "end_ppb": {"ocxo1": 1.0, "ocxo2": 2.0},
                      "ppb_bookend_window": "60_sec"}
    class Cache:
        def __init__(self, clocks, photons): self.data = {"CLOCKS": clocks, "PHOTONS": photons}
        def age_s(self, topic): return 0 if self.data[topic] is not None else None
        def get(self, topic): return self.data[topic]
    ns = isolate(tree, ["field", "number", "temperature", "fresh", "difference",
                        "altitude_bookends", "tempest_snapshot", "lantern_snapshot",
                        "snapshot"], {
        "math": math, "FEED_MAX_AGE_S": 4.0,
        "altitude_text": lambda value, signed=False: "---" if value is None else str(value),
        "last_lantern": Last(closed_lantern),
        "last_tempest": Last(closed_tempest),
    })
    clocks = {"clocks": {"stats": {
        "ocxo1": {"ppb_buckets": {"60_sec": -2, "10_min": -0.4}},
        "ocxo2": {"ppb_buckets": {"60_sec": -3, "10_min": -0.5}},
    }, "system_time_utc": "2026-11-27T20:00:00Z"},
        "campaign": {"state": "STARTED", "name": "Now", "start_ppb": {
            "ocxo1": 0.1, "ocxo2": 0.2}, "ppb_bookend_window": "10_min"}}
    photons = {"photons": {"stats": {"lap_buckets": {
        "60_sec": {"mean_lap_ns": 5159.0},
        "10_min": {"mean_lap_ns": 5147.5}}}},
        "campaign": {"campaign": "Now", "start_lap_ns": 5147.1,
                     "lap_bookend_window": "10_min"}}
    data = ns["snapshot"](Cache(clocks, photons))
    assert data["lap"] == "5,147.500000"
    assert data["ocxo1_10"] == "-0.400" and data["ocxo2_10"] == "-0.500"
    assert data["tempest"]["ocxo1"]["end"] == "-0.400"
    assert data["lantern"]["end_lap"] == "5,147.500000"
    assert data["lantern"]["lap_window"] == "10-MIN"
    past = ns["snapshot"](Cache(None, None))
    assert past["tempest"]["name"] == "Previous"
    assert past["lantern"]["name"] == "Previous"
    assert past["lantern"]["lap_window"] == "60-SEC"
    assert past["lap"] is None and not past["photons_live"]


def test_metrics_html():
    metrics = source("zpnet/metrics/readout_blocks.py")
    tree = ast.parse(metrics)
    ns = isolate(tree, ["_photons_summary_header", "_photons_summary_line"], {
        "PHOTONS_MEAN_WIDTH": 14,
        "_fmt": lambda value, spec, width: "---" if value is None else format(value, spec),
    })
    assert "60-SEC" in ns["_photons_summary_header"]()
    assert "10-MIN" in ns["_photons_summary_header"]()
    line = ns["_photons_summary_line"]({"seconds": 20, "buckets": {
        "60_sec": 5148.1, "10_min": 5147.7}})
    assert "5,148.100000" in line and "5,147.700000" in line
    assert "LAP WIN" in metrics
    html = source("zpnet/stats/index.html")
    assert "PHOTON LAP · 10-MIN" in html
    assert "OCXO1 · 10-MIN" in html and "OCXO2 · 10-MIN" in html
    assert "OCXO1 · 60-SEC" not in html
    assert html.index('id="ocxo1_10"') < html.index('id="lantern"')
    assert html.index('id="ocxo2_10"') < html.index('id="tempest"')



def test_photons_campaign_metrics_decoder():
    """Exercise the actual producer decoder feeding live and campaign tables.

    The firmware publishes both populations, but a consumer with only 60_sec
    in PPB_BUCKET_KEYS silently discards 10_min and then the campaign table
    throws KeyError('10_min'). Verify the full read/display path instead of
    supplying synthetic summaries with both keys already populated.
    """
    metrics = source("zpnet/metrics/readout_blocks.py")
    tree = ast.parse(metrics)
    keys_assignment = next(
        node for node in tree.body if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "PPB_BUCKET_KEYS"
                for target in node.targets)
    )
    keys = ast.literal_eval(keys_assignment.value)
    assert keys == ("60_sec", "10_min"), "PHOTONS decoder must expose both windows"

    live = {
        "sequence": 101,
        "photons": {"stats": {
            "update_count": 101,
            "lap_semantics": "MEAN_LAP_NS_V1",
            "lap_buckets": {
                "60_sec": {"sample_count": 4500, "mean_lap_ns": 5155.123456},
                "10_min": {"sample_count": 45000, "mean_lap_ns": 5147.456789},
            },
        }},
        "campaign": {
            "campaign": "Bookend10", "campaign_id": 76,
            "start_lap_ns": 5147.4, "start_altitude_m": 5.2,
            "lap_bookend_window": "10_min", "public_count": 12,
        },
        "gnss": {"altitude_m": 5.4},
    }
    rows = [
        {"id": 76, "campaign": "Bookend10", "active": True,
         "lap_bookend_window": "10_min", "interrupted_at": None,
         "start_lap_ns": 5147.4, "start_altitude_m": 5.2,
         "end_lap_ns": None, "end_altitude_m": None,
         "lap_buckets": {"10_min": 5147.456789},
         "last_sequence": 100, "seconds": 11},
        {"id": 72, "campaign": "Previous60", "active": False,
         "lap_bookend_window": "60_sec", "interrupted_at": None,
         "start_lap_ns": 5146.5, "start_altitude_m": 5.0,
         "end_lap_ns": 5147.9, "end_altitude_m": 5.3,
         "lap_buckets": {}, "last_sequence": 99, "seconds": 61},
    ]

    class ReadModel:
        def get(self, *, campaigns=False):
            assert campaigns
            return rows, None, None

    def fmt(value, spec, width):
        return "---".rjust(width) if value is None else format(value, spec)

    namespace = {
        "math": math,
        "PPB_BUCKET_KEYS": keys,
        "PHOTONS_MEAN_WIDTH": 14,
        "_json_object": lambda value: value or {},
        "_to_int": lambda value: int(value) if value is not None else None,
        "_to_float": lambda value: float(value) if value is not None else None,
        "_PHOTONS_READ_MODEL": ReadModel(),
        "_get_photons_snapshot": lambda: live,
        "_campaign_cell": lambda name, active, width: name.ljust(width),
        "_campaign_bookend_cells": lambda start, end, *rest:
            f"{start:,.6f} {end:,.6f}" if start is not None and end is not None else "---",
        "_fmt": fmt,
    }
    isolate(tree, ["_photons_producer_lap_buckets", "_photons_summary",
                   "_photons_summary_line", "photons_campaigns_readout"], namespace)

    decoded = namespace["_photons_producer_lap_buckets"](live)
    assert decoded == {"60_sec": 5155.123456, "10_min": 5147.456789}
    summary = namespace["_photons_summary"](live)
    assert "5,147.456789" in namespace["_photons_summary_line"](summary)

    # Historic 60-SEC-only instrument images have no 10-MIN testimony. Keep
    # the missing window empty rather than inventing or copying measurements.
    historical = {"photons": {"stats": {
        "lap_semantics": "MEAN_LAP_NS_V1",
        "lap_buckets": {
            "60_sec": {"sample_count": 600, "mean_lap_ns": 5147.9},
        },
    }}}
    assert namespace["_photons_producer_lap_buckets"](
        historical, historical=True) == {"60_sec": 5147.9, "10_min": None}

    campaign_lines = namespace["photons_campaigns_readout"]()
    assert "10-MIN" in "\n".join(campaign_lines)
    assert "5,147.456789" in "\n".join(campaign_lines), campaign_lines
    assert "Previous60" in "\n".join(campaign_lines)
    assert "60-SEC" in "\n".join(campaign_lines)

def main(argv):
    assert len(argv) == 1, "usage: zt ten_min_population_symmetry_test"
    test_photons_bookends()
    test_stats()
    test_metrics_html()
    test_photons_campaign_metrics_decoder()
    print("10-MIN POPULATION SYMMETRY: PHOTONS/CLOCKS live; "
          "LANTERN persistence; historical windows; Metrics; STATS PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
