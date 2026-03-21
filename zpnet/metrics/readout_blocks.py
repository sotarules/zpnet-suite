"""
ZPNet Metrics Readout Blocks — Dense Clocks Panel (v7 GNSS_RAW)

Designed for full-screen terminal display over SSH at 1080p.

Data sources:
  • Pi CLOCKS REPORT → campaign payload with report dict (tau, ppb,
    prediction stats, timestamps, DWT clock blocks)
  • Teensy CLOCKS REPORT → spin capture, ISR residuals, DWT internals,
    PPS diagnostics, servo state
  • Pi SYSTEM REPORT → GNSS, environment, power

v7 changes:
  • GNSS_RAW synthetic clock domain added right after GNSS row.
    GNSS_RAW is a Pi-side clock synthesized from the GF-8802's
    clock_drift_ppb (TPS1).  Full Welford stats (RES/MEAN/SD/N).
  • Five clock domains: GNSS, GN_RAW, DWT, OCXO1, OCXO2.

v6 changes:
  • DWT line: CYCLES/PPS → PREDICTED, DELTA_RAW → ACTUAL
    PREDICTED is the random walk prediction for the upcoming second.
    ACTUAL is the measured DWT cycles for the last second.

Author: The Mule + Claude
"""

from zpnet.processes.processes import send_command

# ---------------------------------------------------------------------
# Data fetchers
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
# Clock domains
# ---------------------------------------------------------------------

_CLOCK_DOMAINS = [
    ("GNSS", "gnss"),
    ("GN_RAW", "gnss_raw"),
    ("DWT", "dwt"),
    ("OCXO1", "ocxo1"),
    ("OCXO2", "ocxo2"),
]


# ---------------------------------------------------------------------
# Formatting helpers
# ---------------------------------------------------------------------

def _fmt(value, fmt_str, width, fallback="---"):
    if value is None:
        return f"{fallback:>{width}}"
    return f"{value:{fmt_str}}"


def _sign_int(value, width):
    if value is None:
        return f"{'---':>{width}}"
    return f"{value:>+{width}d}"


def _comma_int(value, width):
    if value is None:
        return f"{'---':>{width}}"
    return f"{value:>{width},d}"


def _bool_str(value):
    if value is None:
        return "---"
    return "YES" if value else "NO"


# ---------------------------------------------------------------------
# Status header — always visible on row 0
# ---------------------------------------------------------------------

def status_header() -> str:
    try:
        s = _get_system_snapshot()

        net = s.get("network", {}).get("network_status", "?")
        pi_health = s.get("pi", {}).get("health_state", "?")
        teensy_health = s.get("teensy", {}).get("health_state", "?")
        gnss_lock = s.get("gnss", {}).get("lock_quality", "?")

        bat_v = "?"
        power = s.get("power", {})
        for bus_key, devices in power.items():
            if not bus_key.startswith("i2c-"):
                continue
            if isinstance(devices, dict):
                for rail in devices.values():
                    if isinstance(rail, dict) and rail.get("label", "").lower() == "battery":
                        bat_v = f"{rail['volts']:.2f}V"

        battery = s.get("battery", {})
        pct = battery.get("remaining_pct")
        bat_pct = f"{pct:.0f}%" if pct is not None else "?"

        return (
            f" NET: {net}"
            f"  BAT: {bat_v} {bat_pct}"
            f"  PI: {pi_health}"
            f"  TEENSY: {teensy_health}"
            f"  GNSS: {gnss_lock}"
        )

    except Exception:
        return " STATUS: UNAVAILABLE"


# ---------------------------------------------------------------------
# Combined clocks readout — dense wide-format
# ---------------------------------------------------------------------

def clocks_combined_readout() -> list[str]:
    lines = []

    # ==============================================================
    # Fetch Pi clocks report (campaign payload with report dict)
    # ==============================================================
    try:
        p = _get_pi_clocks_report()
    except Exception:
        return ["CLOCKS: UNAVAILABLE"]

    state = p.get("report", {}).get("campaign_state") or p.get("campaign_state", "IDLE")
    if state != "STARTED":
        return [f"CLOCKS: {state}"]

    # r = the report dict (tau, ppb, timestamps, clock blocks)
    r = p["report"]
    campaign = r.get("campaign", "?")
    elapsed = r.get("campaign_elapsed", "00:00:00")
    n = r.get("pps_count", 0)

    # ==============================================================
    # Fetch Teensy clocks report (spin, ISR, DWT internals, servo)
    # ==============================================================
    try:
        t = _get_teensy_clocks_report()
    except Exception:
        t = {}

    cal = t.get("calibrate_ocxo", "OFF")

    # ==============================================================
    # Fetch baseline
    # ==============================================================
    baseline = _get_clocks_baseline()
    baseline_ppb = baseline.get("baseline_ppb", {}) if baseline else {}
    baseline_id = baseline.get("baseline_id", "?") if baseline else None
    baseline_campaign = baseline.get("baseline_campaign", "?") if baseline else None

    # ==============================================================
    # Campaign header
    # ==============================================================
    servo_str = cal if cal and cal != "OFF" else "IDLE"
    baseline_str = f"BASELINE: #{baseline_id} ({baseline_campaign})" if baseline_id else "BASELINE: NONE"

    lines.append(
        f"CAMPAIGN: {campaign}  ELAPSED: {elapsed}  n={n}"
        f"    SERVO: {servo_str}"
        f"    {baseline_str}"
    )
    lines.append("")

    # ==============================================================
    # Clock domain table — one row per domain
    # ==============================================================

    lines.append(
        f"{'CLK':<6}"
        f"{'TAU':>18}"
        f"{'PPB':>10}"
        f"{'RAW':>14}"
        f"{'RES':>6}"
        f"{'MEAN':>8}"
        f"{'SD':>8}"
        f"{'N':>6}"
        f"  │"
        f"{'BASE':>10}"
        f"{'NOW':>10}"
        f"{'DELTA':>10}"
        f"  │ EXTRA"
    )

    # GNSS row
    gnss_blk = r.get("gnss", {})
    gnss_res = gnss_blk.get("pps_residual", 0)
    gnss_base = baseline_ppb.get("gnss")
    gnss_now = gnss_blk.get("ppb", 0.0)

    if gnss_base is not None and gnss_now is not None:
        gnss_delta = gnss_now - gnss_base
        gnss_comp = f"{gnss_base:>10.3f}{gnss_now:>10.3f}{gnss_delta:>+10.3f}"
    else:
        gnss_comp = f"{'---':>10}{'---':>10}{'---':>10}"

    lines.append(
        f"{'GNSS':<6}"
        f"{'1.000000000000':>18}"
        f"{'0.000':>10}"
        f"{'---':>14}"
        f"{gnss_res:>6}"
        f"{'---':>8}"
        f"{'---':>8}"
        f"{'---':>6}"
        f"  │"
        f"{gnss_comp}"
        f"  │ stream={'OK' if gnss_blk.get('stream_valid', True) else 'BAD'}"
    )

    # GN_RAW row — GNSS_RAW synthetic clock (Pi-side Welford on drift_ppb)
    gnss_raw_blk = r.get("gnss_raw", {})
    gnss_raw_tau = gnss_raw_blk.get("tau")
    gnss_raw_ppb = gnss_raw_blk.get("ppb")
    gnss_raw_res = gnss_raw_blk.get("pred_residual")
    gnss_raw_mean = gnss_raw_blk.get("pred_mean")
    gnss_raw_sd = gnss_raw_blk.get("pred_stddev")
    gnss_raw_n = gnss_raw_blk.get("pred_n")
    gnss_raw_drift = gnss_raw_blk.get("drift_ppb")
    gnss_raw_base = baseline_ppb.get("gnss_raw")
    gnss_raw_now = gnss_raw_blk.get("ppb")

    if gnss_raw_base is not None and gnss_raw_now is not None:
        gnss_raw_delta = gnss_raw_now - gnss_raw_base
        gnss_raw_comp = f"{gnss_raw_base:>10.3f}{gnss_raw_now:>10.3f}{gnss_raw_delta:>+10.3f}"
    else:
        gnss_raw_comp = f"{'---':>10}{'---':>10}{'---':>10}"

    lines.append(
        f"{'GN_RAW':<6}"
        f"{_fmt(gnss_raw_tau, '>18.12f', 18)}"
        f"{_fmt(gnss_raw_ppb, '>10.3f', 10)}"
        f"{'---':>14}"
        f"{_fmt(gnss_raw_res, '>6.1f', 6)}"
        f"{_fmt(gnss_raw_mean, '>8.3f', 8)}"
        f"{_fmt(gnss_raw_sd, '>8.3f', 8)}"
        f"{_fmt(gnss_raw_n, '>6d', 6)}"
        f"  │"
        f"{gnss_raw_comp}"
        f"  │"
    )

    # DWT row
    dwt_blk = r.get("dwt", {})
    dwt_tau = dwt_blk.get("tau", 0.0)
    dwt_ppb = dwt_blk.get("ppb", 0.0)
    dwt_raw = dwt_blk.get("delta_raw")
    dwt_res = dwt_blk.get("pred_residual")
    dwt_mean = dwt_blk.get("pred_mean")
    dwt_sd = dwt_blk.get("pred_stddev")
    dwt_n = dwt_blk.get("pred_n")
    dwt_base = baseline_ppb.get("dwt")
    dwt_now = dwt_blk.get("ppb")

    if dwt_base is not None and dwt_now is not None:
        dwt_delta = dwt_now - dwt_base
        dwt_comp = f"{dwt_base:>10.3f}{dwt_now:>10.3f}{dwt_delta:>+10.3f}"
    else:
        dwt_comp = f"{'---':>10}{'---':>10}{'---':>10}"

    lines.append(
        f"{'DWT':<6}"
        f"{dwt_tau:>18.12f}"
        f"{dwt_ppb:>10.3f}"
        f"{_comma_int(dwt_raw, 14)}"
        f"{_sign_int(dwt_res, 6)}"
        f"{_fmt(dwt_mean, '>8.3f', 8)}"
        f"{_fmt(dwt_sd, '>8.3f', 8)}"
        f"{_fmt(dwt_n, '>6d', 6)}"
        f"  │"
        f"{dwt_comp}"
        f"  │"
    )

    # OCXO1 and OCXO2 rows
    for name, key, dac_key, adj_key in [
        ("OCXO1", "ocxo1", "ocxo1_dac", "ocxo1_servo_adjustments"),
        ("OCXO2", "ocxo2", "ocxo2_dac", "ocxo2_servo_adjustments"),
    ]:
        blk = r.get(key, {})
        tau = blk.get("tau", 0.0)
        ppb = blk.get("ppb", 0.0)
        raw = blk.get("delta_raw")
        res = blk.get("pred_residual")
        mean = blk.get("pred_mean")
        sd = blk.get("pred_stddev")
        pred_n = blk.get("pred_n")
        base_ppb = baseline_ppb.get(key)
        now_ppb = blk.get("ppb")

        if base_ppb is not None and now_ppb is not None:
            delta = now_ppb - base_ppb
            comp = f"{base_ppb:>10.3f}{now_ppb:>10.3f}{delta:>+10.3f}"
        else:
            comp = f"{'---':>10}{'---':>10}{'---':>10}"

        dac = t.get(dac_key)
        adj = t.get(adj_key, 0)
        extra = (
            f" DAC={dac:>10.3f} ADJ={adj:>4}" if dac is not None else ""
        )

        lines.append(
            f"{name:<6}"
            f"{tau:>18.12f}"
            f"{ppb:>10.3f}"
            f"{_comma_int(raw, 14)}"
            f"{_sign_int(res, 6)}"
            f"{_fmt(mean, '>8.3f', 8)}"
            f"{_fmt(sd, '>8.3f', 8)}"
            f"{_fmt(pred_n, '>6d', 6)}"
            f"  │"
            f"{comp}"
            f"  │"
            f"{extra}"
        )

    lines.append("")

    # ==============================================================
    # TIME — from Pi report dict (r)
    # ==============================================================
    gnss_time = r.get("gnss_time_utc", "---")
    system_time = r.get("system_time_utc", "---")

    lines.append(f"TIME  GNSS: {gnss_time}    SYSTEM: {system_time}")
    lines.append("")

    # ==============================================================
    # DWT — predicted vs actual, with PPS residual
    #
    # PREDICTED: dwt_cycles_per_pps — the random walk prediction for
    #   the upcoming second (predicted = prev_delta).  This is what
    #   the system expects the DWT counter to advance by in the next
    #   PPS interval.
    #
    # ACTUAL: dwt.delta_raw — the measured DWT cycles for the last
    #   completed second.
    #
    # The difference (ACTUAL - PREDICTED) is the prediction residual.
    # ==============================================================
    dwt_cycles_per_pps = r.get("dwt_cycles_per_pps")
    dwt_cyccnt_at_pps = r.get("dwt_cyccnt_at_pps")
    dwt_r_delta_raw = r.get("dwt", {}).get("delta_raw")
    dwt_r_pps_residual = r.get("dwt", {}).get("pps_residual")

    lines.append(
        f"DWT   PREDICTED: {_comma_int(dwt_cycles_per_pps, 14)}"
        f"    ACTUAL: {_comma_int(dwt_r_delta_raw, 14)}"
        f"    PPS_RESIDUAL: {_sign_int(dwt_r_pps_residual, 6)}"
        f"    CYCCNT@PPS: {_comma_int(dwt_cyccnt_at_pps, 14)}"
    )

    # ── Crown jewels: authoritative 64-bit accumulators at last PPS ──
    dwt_cycles_64 = r.get("teensy_dwt_cycles")
    gnss_ns_64 = n * 1_000_000_000 if n else None   # GNSS is phase-coherent: exactly n seconds

    lines.append(
        f"      DWT_CYCLES: {_comma_int(dwt_cycles_64, 20)}"
        f"    GNSS_NS: {_comma_int(gnss_ns_64, 20)}"
    )
    lines.append("")

    # ==============================================================
    # SPIN & PPS — from Pi report dict (r)
    # ==============================================================
    spin_valid = r.get("spin_valid")
    spin_approach = r.get("spin_approach_cycles")
    spin_delta = r.get("spin_delta_cycles")
    spin_error = r.get("spin_error_cycles")
    spin_tdc = r.get("spin_tdc_correction")
    spin_nano_to = r.get("spin_nano_timed_out")
    spin_shadow_to = r.get("spin_shadow_timed_out")

    lines.append(
        f"SPIN  VALID: {_bool_str(spin_valid)}"
        f"    APPROACH: {_comma_int(spin_approach, 8)}"
        f"    DELTA: {_fmt(spin_delta, '>4d', 4)}"
        f"    ERROR: {_fmt(spin_error, '>4d', 4)}"
        f"    TDC: {_fmt(spin_tdc, '>2d', 2)}"
        f"    NANO_TO: {_bool_str(spin_nano_to)}"
        f"    SHADOW_TO: {_bool_str(spin_shadow_to)}"
    )

    # ISR residuals — from Pi report dict
    isr_gnss = r.get("isr_residual_gnss")
    isr_dwt = r.get("isr_residual_dwt")
    isr_ocxo1 = r.get("isr_residual_ocxo1")
    isr_ocxo2 = r.get("isr_residual_ocxo2")

    lines.append(
        f"ISR   GNSS: {_sign_int(isr_gnss, 6)}"
        f"    DWT: {_sign_int(isr_dwt, 6)}"
        f"    OCXO1: {_sign_int(isr_ocxo1, 6)}"
        f"    OCXO2: {_sign_int(isr_ocxo2, 6)}"
    )

    # PPS diagnostics — from Pi report dict
    pps_rej_total = r.get("pps_rejected_total")
    pps_rej_rem = r.get("pps_rejected_remainder")

    lines.append(
        f"PPS   REJECTED: {_fmt(pps_rej_total, '>4d', 4)}"
        f"    REMAINDER: {_fmt(pps_rej_rem, '>4d', 4)}"
    )

    lines.append("")

    # ==============================================================
    # GNSS Status + Position — from SYSTEM snapshot
    # ==============================================================
    try:
        snapshot = _get_system_snapshot()
    except Exception:
        snapshot = {}

    gnss = snapshot.get("gnss", {})

    discipline = gnss.get("discipline", {})
    freq_mode_name = discipline.get("freq_mode_name", "?") if isinstance(discipline, dict) else str(discipline)

    survey = gnss.get("survey_mode", {})
    pos_mode = survey.get("receiver_mode", "?") if isinstance(survey, dict) else "?"

    integrity = gnss.get("integrity", {})
    traim = integrity.get("traim", "?") if isinstance(integrity, dict) else "?"

    sats = gnss.get("satellites", "?")
    hdop = gnss.get("hdop")
    hdop_str = f"{hdop:.2f}" if hdop is not None else "---"

    lat = gnss.get("latitude_deg")
    lon = gnss.get("longitude_deg")
    alt_gnss = gnss.get("altitude_m")
    ellipsoid = gnss.get("ellipsoid_height_m")
    geoid = gnss.get("geoid_sep_m")

    gnss_line = (
        f"GNSS  MODE: {pos_mode}"
        f"  DISC: {freq_mode_name}"
        f"  SATS: {sats}"
        f"  HDOP: {hdop_str}"
        f"  TRAIM: {traim}"
    )

    pos_parts = []
    if lat is not None and lon is not None:
        pos_parts.append(f"LAT: {lat:.6f}")
        pos_parts.append(f"LON: {lon:.6f}")
    if alt_gnss is not None:
        pos_parts.append(f"ALT(MSL): {alt_gnss:.1f}m")
    if ellipsoid is not None:
        pos_parts.append(f"ELLIP: {ellipsoid:.1f}m")
    if geoid is not None:
        pos_parts.append(f"GEOID: {geoid:.1f}m")

    if pos_parts:
        gnss_line += "  " + "  ".join(pos_parts)

    lines.append(gnss_line)
    lines.append("")

    # ==============================================================
    # Environment
    # ==============================================================
    env = snapshot.get("environment", {})
    pi_data = snapshot.get("pi", {})
    teensy_data = snapshot.get("teensy", {})

    temp_c = env.get("temperature_c")
    humidity = env.get("humidity_pct")
    pressure = env.get("pressure_hpa")
    baro_alt = env.get("altitude_m")
    pi_temp = pi_data.get("cpu_temp_c")
    teensy_temp = teensy_data.get("cpu_temp_c")

    env_parts = []
    if temp_c is not None:
        env_parts.append(f"AMBIENT: {temp_c:.1f}°C")
    if humidity is not None:
        env_parts.append(f"RH: {humidity:.0f}%")
    if pressure is not None:
        env_parts.append(f"BARO: {pressure:.1f}hPa")
    if baro_alt is not None:
        env_parts.append(f"BARO_ALT: {baro_alt:.1f}m")
    if pi_temp is not None:
        env_parts.append(f"PI: {pi_temp:.1f}°C")
    if teensy_temp is not None:
        env_parts.append(f"TEENSY: {teensy_temp:.1f}°C")

    lines.append("ENV   " + "  ".join(env_parts))
    lines.append("")

    # ==============================================================
    # Power — tabular (Pi, Teensy, OCXO1, OCXO2 domains)
    # ==============================================================
    power = snapshot.get("power", {})

    def _find_rail(label_match: str) -> dict | None:
        for bus_key, devices in power.items():
            if not bus_key.startswith("i2c-"):
                continue
            if isinstance(devices, dict):
                for rail in devices.values():
                    if isinstance(rail, dict) and label_match.lower() in rail.get("label", "").lower():
                        return rail
        return None

    power_rails = [
        ("PI DOMAIN",     _find_rail("pi domain")),
        ("TEENSY DOMAIN", _find_rail("teensy")),
        ("OCXO1 DOMAIN",  _find_rail("ocxo1")),
        ("OCXO2 DOMAIN",  _find_rail("ocxo2")),
    ]

    lines.append(f"{'POWER':<18}{'V':>10}{'MA':>10}{'W':>10}")
    for label, rail in power_rails:
        if rail:
            lines.append(
                f"{label:<18}"
                f"{rail['volts']:>10.3f}"
                f"{rail['amps']:>10.1f}"
                f"{rail['watts']:>10.3f}"
            )
        else:
            lines.append(f"{label:<18}{'---':>10}{'---':>10}{'---':>10}")

    return lines


# ---------------------------------------------------------------------
# Readout registry
# ---------------------------------------------------------------------

READOUTS = [
    ("CLOCKS", clocks_combined_readout),
]