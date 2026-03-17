"""
ZPNet Metrics Readout Blocks — Combined Clocks Panel

Designed for full-screen terminal display over SSH.
Each readout returns a list of lines (not a generator)
so the curses display can measure and position them.

The combined clocks panel shows everything the dashboard
spreads across four separate readouts in a single dense view:
  • Campaign status and elapsed time
  • Tau and PPB for all four clock domains
  • Prediction statistics (residual, mean, stddev, n)
  • Baseline comparison (if set)
  • OCXO servo/DAC state

Invariants:
  • No database access
  • No aggregates
  • No events
  • No inference beyond explicit tests
  • Defenseless (exceptions propagate to outer fault barrier)

Author: The Mule + GPT
"""

from zpnet.processes.processes import send_command

# ---------------------------------------------------------------------
# Data fetchers (same RPC as dashboard)
# ---------------------------------------------------------------------

def _get_system_snapshot() -> dict:
    return send_command(machine="PI", subsystem="SYSTEM", command="REPORT")["payload"]


def _get_teensy_clocks_report() -> dict:
    return send_command(machine="TEENSY", subsystem="CLOCKS", command="REPORT")["payload"]


def _get_pi_clocks_report() -> dict:
    return send_command(machine="PI", subsystem="CLOCKS", command="REPORT")["payload"]


def _get_clocks_baseline() -> dict | None:
    try:
        resp = send_command(machine="PI", subsystem="CLOCKS", command="BASELINE_INFO")
        if resp.get("success"):
            p = resp.get("payload", {})
            if p.get("baseline_set"):
                return p
    except Exception:
        pass
    return None


# ---------------------------------------------------------------------
# Clock domains (shared constant)
# ---------------------------------------------------------------------

_CLOCK_DOMAINS = [("GNSS", "gnss"), ("DWT", "dwt"), ("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]


# ---------------------------------------------------------------------
# Status header — always visible on row 0
# ---------------------------------------------------------------------

def status_header() -> str:
    """
    Single-line system status for the top of the screen.

    Shows: network, battery voltage, Pi health, Teensy health, GNSS lock.
    """
    try:
        s = _get_system_snapshot()

        net = s.get("network", {}).get("network_status", "?")
        pi_health = s.get("pi", {}).get("health_state", "?")
        teensy_health = s.get("teensy", {}).get("health_state", "?")
        gnss_lock = s.get("gnss", {}).get("lock_quality", "?")

        # Find battery voltage from power rails
        bat_v = "?"
        power = s.get("power", {})
        for bus_key, devices in power.items():
            if not bus_key.startswith("i2c-"):
                continue
            if isinstance(devices, dict):
                for rail in devices.values():
                    if isinstance(rail, dict) and rail.get("label", "").lower() == "battery":
                        bat_v = f"{rail['volts']:.2f}V"

        return (
            f" NET: {net}"
            f"  BAT: {bat_v}"
            f"  PI: {pi_health}"
            f"  TEENSY: {teensy_health}"
            f"  GNSS: {gnss_lock}"
        )

    except Exception:
        return " STATUS: UNAVAILABLE"


# ---------------------------------------------------------------------
# Combined clocks readout
# ---------------------------------------------------------------------

def clocks_combined_readout() -> list[str]:
    """
    Combined clocks panel — all clock data in one dense view.

    Returns a list of lines ready for curses positioning.
    """
    lines = []

    # ==============================================================
    # Campaign header
    # ==============================================================
    try:
        p = _get_pi_clocks_report()
    except Exception:
        return ["CLOCKS: UNAVAILABLE"]

    state = p.get("report", {}).get("campaign_state") or p.get("campaign_state", "IDLE")
    if state != "STARTED":
        return [f"CLOCKS: {state}"]

    r = p["report"]
    campaign = r.get("campaign", "?")
    elapsed = r.get("campaign_elapsed", "00:00:00")
    n = r.get("pps_count", 0)

    lines.append(f"CAMPAIGN: {campaign}  ELAPSED: {elapsed}  n={n}")
    lines.append("")

    # ==============================================================
    # Tau + PPB table
    # ==============================================================
    lines.append(f"{'CLK':<6} {'TAU':>16} {'PPB':>10}")
    lines.append(f"{'GNSS':<6} {'1.0000000000':>16} {'0.000':>10}")

    for name, key in _CLOCK_DOMAINS[1:]:
        blk = r.get(key, {})
        tau = blk.get("tau", 0.0)
        ppb = blk.get("ppb", 0.0)
        lines.append(f"{name:<6} {tau:>16.10f} {ppb:>10.3f}")

    lines.append("")

    # ==============================================================
    # Prediction statistics
    # ==============================================================
    lines.append(f"{'CLK':<6} {'RES':>6} {'MEAN':>8} {'SD':>8} {'N':>6}")

    gnss_blk = r.get("gnss", {})
    gnss_res = gnss_blk.get("pps_residual", 0)
    lines.append(f"{'GNSS':<6} {gnss_res:>6} {'---':>8} {'---':>8} {'---':>6}")

    for name, key in _CLOCK_DOMAINS[1:]:
        blk = r.get(key, {})
        pred_n = blk.get("pred_n")

        if pred_n is not None and pred_n > 0:
            res = blk.get("pred_residual", 0)
            mean = blk.get("pred_mean", 0.0)
            stddev = blk.get("pred_stddev", 0.0)
            lines.append(f"{name:<6} {res:>6} {mean:>8.3f} {stddev:>8.3f} {pred_n:>6}")
        else:
            lines.append(f"{name:<6} {'---':>6} {'---':>8} {'---':>8} {'---':>6}")

    lines.append("")

    # ==============================================================
    # Baseline comparison (if set)
    # ==============================================================
    baseline = _get_clocks_baseline()
    if baseline is not None:
        baseline_ppb = baseline.get("baseline_ppb", {})
        baseline_id = baseline.get("baseline_id", "?")
        baseline_campaign = baseline.get("baseline_campaign", "?")

        lines.append(f"BASELINE: {baseline_campaign} (#{baseline_id})")
        lines.append(f"{'CLK':<6} {'BASE':>10} {'NOW':>10} {'DELTA':>10}")

        for name, key in _CLOCK_DOMAINS:
            blk = r.get(key, {})
            base_ppb = baseline_ppb.get(key)
            now_ppb = blk.get("ppb")

            if base_ppb is not None and now_ppb is not None:
                delta = now_ppb - base_ppb
                lines.append(f"{name:<6} {base_ppb:>10.3f} {now_ppb:>10.3f} {delta:>+10.3f}")
            else:
                lines.append(f"{name:<6} {'---':>10} {'---':>10} {'---':>10}")
    else:
        lines.append("BASELINE: NOT SET")

    lines.append("")

    # ==============================================================
    # OCXO servo status
    # ==============================================================
    try:
        t = _get_teensy_clocks_report()
    except Exception:
        lines.append("SERVO: UNAVAILABLE")
        return lines

    cal = t.get("calibrate_ocxo", False)
    lines.append(f"SERVO: {'CALIBRATING' if cal else 'IDLE'}")

    for label, dac_key, adj_key, res_key in [
        ("OCXO1", "ocxo1_dac", "ocxo1_servo_adjustments", "isr_residual_ocxo1"),
        ("OCXO2", "ocxo2_dac", "ocxo2_servo_adjustments", "isr_residual_ocxo2"),
    ]:
        dac = t.get(dac_key)
        adj = t.get(adj_key, 0)
        res = t.get(res_key)

        if dac is not None:
            res_str = f"{res:+d}" if res is not None else "---"
            lines.append(f"{label:<6} DAC={dac:>10.3f}  ADJ={adj:>4}  RES={res_str}")
        else:
            lines.append(f"{label:<6} ---")

    return lines


# ---------------------------------------------------------------------
# Readout registry — ordered list of (name, callable)
# ---------------------------------------------------------------------

READOUTS = [
    ("CLOCKS", clocks_combined_readout),
]