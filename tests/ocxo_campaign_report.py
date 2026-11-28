"""Run on the Pi with the ZPNet Python environment. Writes a report to stdout."""
import sys
from collections import deque

LANES = ("ocxo1", "ocxo2")
CAMPAIGNS = ("PhoToNordic1", "NordicFresh2")

QUERY = """
SELECT id, ts, viable, payload->>'sequence' AS seq,
       payload#>>'{clocks,stats,reset_count}' AS epoch,
       payload#>>'{campaign,public_count}' AS pps,
       payload#>'{campaign,clockfaces}' AS faces,
       payload#>'{campaign,start_ppb}' AS baseline,
       payload#>>'{gnss,altitude_m}' AS alt,
       payload#>'{clocks,stats,ocxo1,ppb_buckets}' AS ocxo1,
       payload#>'{clocks,stats,ocxo2,ppb_buckets}' AS ocxo2,
       payload->>'holistic_restore_superseded' AS superseded,
       payload#>>'{campaign,status,timeline_valid}' AS timeline,
       payload#>>'{campaign,disposition,science_eligible}' AS science
FROM campaign_detail WHERE campaign_type = 'TEMPEST' AND campaign = %s
ORDER BY id
"""


def value(x):
    return "      ." if x is None else f"{float(x):+8.3f}"


def emit(point, label):
    row, pairs = point
    print(f"{label:12} {str(row['ts']):32} {str(row['pps']):>7} "
          f"{str(row['alt'] or '.'):>9} " +
          " | ".join(" ".join(value(x) for x in lane) for lane in pairs))


def report(rows, master):
    print("master snapshot:", master)
    print("Each lane: saved start | published 60s | published 600s | "
          "rebuilt 600s | published 600s minus row start (PPB)")
    print("Rows show first, every 60th recorded row, baseline changes, last, and extrema.")
    window = deque(maxlen=601)
    prev = last = None
    peaks = [None, None]
    breaks = 0
    for n, row in enumerate(rows, 1):
        f = row["faces"] or {}
        usable = (row["viable"] and row["superseded"] != "true"
                  and row["timeline"] != "false" and row["science"] != "false"
                  and all(f.get(k) is not None for k in ("gnss_ns","ocxo1_ns","ocxo2_ns"))
                  and row["seq"] is not None and row["pps"] is not None)
        if prev is not None and (not usable or not window or
                int(row["seq"]) != int(prev["seq"]) + 1 or
                int(row["pps"]) != int(prev["pps"]) + 1 or row["epoch"] != prev["epoch"] or
                any(int(f[k]) <= window[-1][i] for i,k in enumerate(("gnss_ns","ocxo1_ns","ocxo2_ns")))):
            breaks += bool(window)
            window.clear()
        if usable:
            window.append(tuple(int(f[k]) for k in ("gnss_ns","ocxo1_ns","ocxo2_ns")))
        pairs = []
        for i, lane in enumerate(LANES):
            bucket = row[lane] or {}
            start = (row["baseline"] or {}).get(lane)
            published = bucket.get("10_min")
            delta = None if start is None or published is None else float(published) - float(start)
            rebuilt = None
            if len(window) == 601:
                gnss = window[-1][0] - window[0][0]
                ocxo = window[-1][i+1] - window[0][i+1]
                rebuilt = (ocxo - gnss) * 1e9 / gnss
            pairs.append((start, bucket.get("60_sec"), published, rebuilt, delta))
        last = row, pairs
        changed = prev is not None and prev["baseline"] != row["baseline"]
        if n == 1 or changed or n % 60 == 0:
            emit(last, "start change" if changed else "sample")
        for i, pair in enumerate(pairs):
            delta = pair[-1]
            if usable and delta is not None and (peaks[i] is None or abs(delta) > peaks[i][0]):
                peaks[i] = abs(delta), last
        prev = row
    if last:
        emit(last, "last")
        for lane, peak in zip(LANES, peaks):
            if peak:
                emit(peak[1], lane + " peak")
        print(f"{n} detail rows; {breaks} reconstruction continuity breaks")
    else:
        print("No TEMPEST detail rows.")


def main():
    from zpnet.shared.db import open_db
    with open_db(row_dict=True) as db:
        db.execute("SET TRANSACTION ISOLATION LEVEL REPEATABLE READ, READ ONLY")
        for name in sys.argv[1:] or CAMPAIGNS:
            print("\n", name)
            master = db.execute("""
                SELECT active, payload->'start_ppb' AS start, payload->'latest_ppb' AS latest,
                       payload->'end_ppb' AS ending FROM campaign_master
                WHERE campaign_type='TEMPEST' AND campaign=%s
            """, (name,)).fetchone()
            with db.cursor(name="ocxo_detail_report") as cur:
                cur.execute(QUERY, (name,))
                report(cur, master)


if __name__ == "__main__":
    main()
