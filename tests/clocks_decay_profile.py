"""Inspect recorded Pi decay fits and their applied voltage correction.

This is an operator report, not a test or a command that changes the instrument.
It reads one compact durable snapshot; it never scans the campaign's history or
refits cumulative DAC means. All rendered rate conversions use exact fractions.
"""

from __future__ import annotations

import argparse
import json
from fractions import Fraction

from zpnet.shared.clocks_decay import LANES, NS, PPB_PER_CODE, VOLTS_PER_CODE, fraction


def recorded_state(campaign: str | None) -> dict:
    from zpnet.shared.db import open_db

    with open_db(row_dict=True) as db:
        db.execute("SET TRANSACTION READ ONLY")
        db.execute("SET LOCAL statement_timeout = '15s'")
        with db.cursor() as cur:
            if campaign is not None:
                # The campaign master is the same recorded projection used by
                # the list. A stopped campaign never borrows today's model.
                cur.execute(
                    """
                    SELECT id, ts, campaign, active,
                           payload #> '{report,clocks,decay}' AS decay,
                           payload #> '{report,clocks,dac_baseline}' AS baseline,
                           payload -> 'campaign_dac' AS campaign_dac,
                           payload #>> '{report,sequence}' AS sequence
                    FROM campaign_master
                    WHERE campaign_type = 'TEMPEST' AND campaign = %s
                    """,
                    (campaign,),
                )
            else:
                cur.execute(
                    """
                    SELECT id, ts, sequence, campaign,
                           payload #> '{clocks,decay}' AS decay,
                           payload #> '{clocks,dac_baseline}' AS baseline,
                           payload #> '{clocks,decay_system}' AS decay_system,
                           payload #> '{clocks,stats,auxiliary_welford}' AS populations,
                           payload #>> '{clocks,gnss_time_utc}' AS gnss_time_utc
                    FROM campaign_detail
                    WHERE campaign_type = 'TEMPEST'
                      AND payload ->> 'schema' = 'CLOCKS_V4'
                      AND payload -> 'holistic_restore_superseded'
                          IS DISTINCT FROM 'true'::jsonb
                    ORDER BY id DESC
                    LIMIT 1
                    """
                )
            row = cur.fetchone()
    if row is None:
        raise LookupError(f"No recorded TEMPEST campaign {campaign!r}" if campaign else
                          "No recorded CLOCKS state")
    return row


def fixed(value: Fraction | None, digits: int = 6, *, signed: bool = True) -> str:
    if value is None:
        return "---"
    scaled = round(value * 10 ** digits)
    whole, decimals = divmod(abs(scaled), 10 ** digits)
    sign = "-" if scaled < 0 else "+" if signed else ""
    return f"{sign}{whole}.{decimals:0{digits}d}"


def rate_cells(rate: Fraction, lane: str) -> tuple[str, str]:
    return (fixed(rate * VOLTS_PER_CODE * NS),
            fixed(-rate * PPB_PER_CODE[lane] * 86400))


def delta_cells(row: dict, lane: str, campaign: bool) -> tuple[str, str, str]:
    baseline = row.get("baseline")
    if campaign:
        recording = row.get("campaign_dac") or {}
        population = recording.get(lane) or {}
        sums = recording.get("decay_sum_codes") or {}
        correction_sum = fraction(sums[lane]) if lane in sums else Fraction(0)
    else:
        population = (row.get("populations") or {}).get(f"{lane}_dac") or {}
        lane_decay = (row.get("decay_system") or {}).get(lane) or {}
        correction_sum = fraction(lane_decay["sum"]) if "sum" in lane_decay else Fraction(0)
    if baseline is None or not population.get("n"):
        return "---", "---", "---"
    mean = Fraction(str(population["mean"]))
    base = baseline[lane]
    raw_delta = -PPB_PER_CODE[lane] * (mean - Fraction(str(base["mean_code"])))
    base_correction = fraction(base["decay_mean_code"]) if "decay_mean_code" in base else Fraction(0)
    adjustment = PPB_PER_CODE[lane] * (correction_sum / population["n"] - base_correction)
    return fixed(raw_delta), fixed(adjustment), fixed(raw_delta + adjustment)


def show(row: dict, campaign: str | None) -> None:
    scope = f"campaign {campaign}" if campaign else "system DAC population"
    print(f"TEMPEST decay profile  {scope}")
    if campaign:
        detail_id = (row.get("campaign_dac") or {}).get("last_detail_id", "---")
        print(f"Campaign master {row['id']}  latest DAC detail {detail_id}  sequence {row.get('sequence')}")
    else:
        print(f"Recorded row {row['id']}  sequence {row.get('sequence')}  timestamp {row['ts']}")
    decay = row.get("decay")
    if decay is None:
        print("This recorded state predates decay profiling.")
        return
    print(f"DECAY_PROFILE={'ON' if decay['profile_enabled'] else 'OFF'}  "
          f"selected window {decay['window_hours']:g} h  model {decay['model_id']}")
    print(f"Learning session {decay['session_id']}")
    print(f"GNSS clock ns {decay['last_gnss_ns']}  freeze GNSS ns {decay['anchor_gnss_ns']}")
    elapsed = Fraction(decay["time_ns"] - decay["anchor_ns"], NS)
    print(f"Observed GNSS seconds since model anchor {fixed(elapsed, 3, signed=False)}  "
          f"clock discontinuities {decay['discontinuities']}")
    print(f"Samples skipped outside 10-MIN servo mode: {decay['skipped_servo_samples']}")
    if row.get("gnss_time_utc"):
        print(f"GNSS UTC {row['gnss_time_utc']}")

    print("\nApplied linear model (positive nV/s predicts increasing control voltage)")
    print(f"{'OCXO':<7}{'nV/s':>16}{'equiv ppb/day':>18}")
    for lane in LANES:
        rate, ppb = rate_cells(fraction(decay["slope_codes_per_second"][lane]), lane)
        print(f"{lane.upper():<7}{rate:>16}{ppb:>18}")

    label = "Learning fits" if decay["profile_enabled"] else "Retained training fits"
    print(f"\n{label} (completed minute blocks; actual span can be shorter than window)")
    print(f"{'WINDOW':>7}{'SPAN h':>10}{'N':>10}{'BLOCKS':>9}  "
          f"{'OCXO':<7}{'nV/s':>16}{'ppb/day':>16}{'RMS ppb':>14}")
    fits = decay.get("fits") or {}
    for window in sorted(fits, key=float):
        fit = fits[window]
        if fit is None:
            print(f"{window + 'h':>7}  No completed fit yet")
            continue
        span = Fraction(fit["last_ns"] - fit["first_ns"], NS * 3600)
        for lane in LANES:
            rate, ppb = rate_cells(fraction(fit[lane]["slope_codes_per_second"]), lane)
            rms = fit[lane]["minute_residual_rms_ppb"]
            print(f"{window + 'h':>7}{fixed(span, 3, signed=False):>10}"
                  f"{fit['n']:>10}{fit['blocks']:>9}  {lane.upper():<7}"
                  f"{rate:>16}{ppb:>16}{rms:>14.6f}")
    if not fits:
        print("No completed fits in this learning session.")
    accepted = decay.get("accepted_fit")
    if accepted:
        print(f"Accepted fit: {accepted['n']:,} observations, {accepted['blocks']} blocks, "
              f"{accepted['span_seconds'] / 3600:.3f} hours")

    print("\nVoltage-derived DELTA_PPB for the selected population")
    print(f"{'OCXO':<7}{'RAW ppb':>16}{'ADDED ppb':>16}{'NORMALIZED ppb':>18}")
    for lane in LANES:
        raw, added, normalized = delta_cells(row, lane, campaign is not None)
        print(f"{lane.upper():<7}{raw:>16}{added:>16}{normalized:>18}")
    print("\nSix decimals of nV/s resolve one femtovolt/second; this is representation, not accuracy.")
    print("RMS describes minute-mean residuals, not independent-sample uncertainty or forecast error.")
    print("The active model stays fixed during learning; turning profiling off accepts the selected fit.")
    print("Freezing changes the slope continuously; setting a matching DAC baseline establishes zero.")
    print("Corrections are averaged over the same observations as the voltage mean; raw voltages remain recorded.")
    print("History before installation or the first freeze has zero adjustment; a fresh stationary campaign")
    print("after freezing avoids mixing that history into the normalized mean and its new DAC baseline.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign", nargs="?", help="named campaign; omitted selects the system DAC population")
    parser.add_argument("--json", action="store_true", help="print the compact recorded data with exact coefficients")
    args = parser.parse_args()
    try:
        row = recorded_state(args.campaign)
    except LookupError as exc:
        parser.exit(1, f"{exc}\n")
    if args.json:
        print(json.dumps(row, indent=2, default=str))
    else:
        show(row, args.campaign)


if __name__ == "__main__":
    main()
