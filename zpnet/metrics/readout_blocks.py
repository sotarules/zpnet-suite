"""
ZPNet Metrics Readout Blocks — Dense Clocks Panel (v10 Fragment-Aware)

Designed for full-screen terminal display over SSH at 1080p.

Data sources:
  • Pi CLOCKS REPORT → campaign payload with report dict (tau, ppb,
    prediction stats, timestamps, DWT clock blocks)
  • The report dict now contains a 'fragment' sub-dict with the raw
    TIMEBASE_FRAGMENT fields.  All spin, ISR, phase, and servo fields
    live inside the fragment.
  • Pi SYSTEM REPORT → GNSS, environment, power

v11 changes:
  • DAC statistics panel MEAN column widened by one space so the
    numeric values align cleanly under the header.
  • DWT two-line section realigned so PREDICTED sits over DWT_CYCLES
    and ACTUAL sits over GNSS_NS, with numeric values right-aligned.
  • Legacy TDC-era display fields removed from the metrics panel:
    SPIN DELTA / ERROR / TDC, OCXO phase deltas, and CAL Δ_TDC.

v10 changes:
  • Fragment-aware field access: all spin capture, ISR residual,
    phase detector, and PPS diagnostic fields now read from the
    'fragment' sub-dict inside the Pi report, falling back to
    top-level for backward compatibility.
  • OCXO EXTRA column updated for the phase-edge model:
    shows DAC, PHASE, EDGE_NS, and RES.
  • NOW servo CAL section updated for phase-edge model (no more
    match_prev / winner / iteration fields).
  • PHASE section added: per-OCXO phase detector summary.

v9 changes:
  • OCXO NOW-servo detail rows updated for the phase/edge model.
  • Removed old missed-delta (MD) display from CAL rows.
  • CAL rows now surface PHASE_NS and EDGE_GNSS instead of legacy MD.
  • OCXO clock-domain EXTRA column now shows PHASE=, EDGE_NS=, and RES=.

v8 changes:
  • OCXO EXTRA column now shows DAC= as the actual integer hardware
    code written to the AD5693R (dac_hw), not the servo's fractional
    accumulator.  The fractional value remains visible in the DAC
    Welford MEAN column where it belongs.
  • Pipe column separators removed for cleaner alignment.

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


def _safe_lines(fn):
    try:
        return fn()
    except Exception as e:
        return [f"ERROR: {e}"]



# ---------------------------------------------------------------------
# Fragment-aware field access
# ---------------------------------------------------------------------

def _frag(r: dict, key: str, default=None):
    """
    Read a field from the report dict, checking the fragment sub-dict
    first (new TIMEBASE format), then top-level (backward compat).
    """
    frag = r.get("fragment")
    if frag is not None:
        val = frag.get(key)
        if val is not None:
            return val
    val = r.get(key)
    return val if val is not None else default


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

    cal = _frag(r, "calibrate_ocxo", t.get("calibrate_ocxo", "OFF"))

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
        f"{'MEAN':>9}"
        f"{'SD':>8}"
        f"{'N':>6}"
        f"  "
        f"{'BASE':>10}"
        f"{'NOW':>10}"
        f"{'DELTA':>10}"
        f"   EXTRA"
    )

    # GNSS row
    gnss_blk = r.get("gnss", {})
    gnss_res = gnss_blk.get("pps_residual", 0)
    gnss_base = baseline_ppb.get("gnss")
    gnss_now = gnss_blk.get("ppb", 0.0)

    if gnss_base is not None and gnss_now is not None:
        gnss_delta = gnss_now - gnss_base
        gnss_comp = f"{_fmt(gnss_base, '>10.3f', 10)}{_fmt(gnss_now, '>10.3f', 10)}{_fmt(gnss_delta, '>+10.3f', 10)}"
    else:
        gnss_comp = f"{'---':>10}{'---':>10}{'---':>10}"

    lines.append(
        f"{'GNSS':<6}"
        f"{'1.000000000000':>18}"
        f"{'0.000':>10}"
        f"{'---':>14}"
        f"{_fmt(gnss_res, '>6d', 6)}"
        f" "
        f"{'---':>8}"
        f"{'---':>8}"
        f"{'---':>6}"
        f"  "
        f"{gnss_comp}"
        f"   stream={'OK' if gnss_blk.get('stream_valid', True) else 'BAD'}"
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
        gnss_raw_comp = f"{_fmt(gnss_raw_base, '>10.3f', 10)}{_fmt(gnss_raw_now, '>10.3f', 10)}{_fmt(gnss_raw_delta, '>+10.3f', 10)}"
    else:
        gnss_raw_comp = f"{'---':>10}{'---':>10}{'---':>10}"

    lines.append(
        f"{'GN_RAW':<6}"
        f"{_fmt(gnss_raw_tau, '>18.12f', 18)}"
        f"{_fmt(gnss_raw_ppb, '>10.3f', 10)}"
        f"{'---':>14}"
        f"{_fmt(gnss_raw_res, '>6.1f', 6)}"
        f" "
        f"{_fmt(gnss_raw_mean, '>8.3f', 8)}"
        f"{_fmt(gnss_raw_sd, '>8.3f', 8)}"
        f"{_fmt(gnss_raw_n, '>6d', 6)}"
        f"  "
        f"{gnss_raw_comp}"
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
        dwt_comp = f"{_fmt(dwt_base, '>10.3f', 10)}{_fmt(dwt_now, '>10.3f', 10)}{_fmt(dwt_delta, '>+10.3f', 10)}"
    else:
        dwt_comp = f"{'---':>10}{'---':>10}{'---':>10}"

    lines.append(
        f"{'DWT':<6}"
        f"{_fmt(dwt_tau, '>18.12f', 18)}"
        f"{_fmt(dwt_ppb, '>10.3f', 10)}"
        f"{_comma_int(dwt_raw, 14)}"
        f"{_sign_int(dwt_res, 6)}"
        f" "
        f"{_fmt(dwt_mean, '>8.3f', 8)}"
        f"{_fmt(dwt_sd, '>8.3f', 8)}"
        f"{_fmt(dwt_n, '>6d', 6)}"
        f"  "
        f"{dwt_comp}"
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
            comp = f"{_fmt(base_ppb, '>10.3f', 10)}{_fmt(now_ppb, '>10.3f', 10)}{_fmt(delta, '>+10.3f', 10)}"
        else:
            comp = f"{'---':>10}{'---':>10}{'---':>10}"

        # New fragment/report shape: surface DAC, edge time, PPS count, prediction, and per-second residual.
        dac_now = blk.get("dac", _frag(r, f"{key}_dac"))
        gnss_ns_at_edge = blk.get("gnss_ns_at_edge", _frag(r, f"{key}_gnss_ns_at_edge"))
        ns_count_at_edge = blk.get("ns_count_at_edge", _frag(r, f"{key}_ns_count_at_edge"))
        ns_count_at_pps = blk.get("ns_count_at_pps", _frag(r, f"{key}_ns_count_at_pps"))
        next_pred = blk.get("ns_count_next_second_prediction", _frag(r, f"{key}_ns_count_next_second_prediction"))
        residual_ns = blk.get("second_residual_ns", _frag(r, f"{key}_second_residual_ns"))
        extra_parts = []
        if dac_now is not None:
            dac_now_f = float(dac_now)
            dac_volts = dac_now_f * 3.002 / 65535.0
            extra_parts.append(f"DAC={dac_now_f:>9.3f} {dac_volts:.5f}V")
        if gnss_ns_at_edge is not None:
            extra_parts.append(f"EDGE={int(gnss_ns_at_edge)}")
        if ns_count_at_edge is not None:
            extra_parts.append(f"EDGE_NS={int(ns_count_at_edge)}")
        if next_pred is not None:
            extra_parts.append(f"PRED={int(next_pred)}")
        if residual_ns is not None:
            extra_parts.append(f"RES={int(residual_ns):+d}ns")
        if ns_count_at_pps is not None:
            extra_parts.append(f"PPS_NS={int(ns_count_at_pps)}")
        extra = "   " + " ".join(extra_parts)

        lines.append(
            f"{name:<6}"
            f"{_fmt(tau, '>18.12f', 18)}"
            f"{_fmt(ppb, '>10.3f', 10)}"
            f"{_comma_int(raw, 14)}"
            f"{_sign_int(res, 6)}"
            f" "
            f"{_fmt(mean, '>8.3f', 8)}"
            f"{_fmt(sd, '>8.3f', 8)}"
            f"{_fmt(pred_n, '>6d', 6)}"
            f"  "
            f"{comp}"
            f"{extra}"
        )

    # ==============================================================
    # DAC Welford — campaign-cumulative statistics (TEMPEST DAC test)
    # ==============================================================
    baseline_dac_mean = baseline.get("baseline_dac_mean", {}) if baseline else {}

    lines.append("")

    lines.append(
        f"{'DAC':<6}"
        f"{'':>18}"
        f"{'':>10}"
        f"{'':>14}"
        f"{'':>6}"
        f"{'MEAN':>9}"
        f"{'SD':>8}"
        f"{'N':>6}"
        f"  "
        f"{'BASE':>10}"
        f"{'NOW':>10}"
        f"{'DELTA':>10}"
    )

    for name, key in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
        blk = r.get(key, {})
        dac_n = blk.get("dac_n")
        dac_mean = blk.get("dac_mean")
        dac_sd = blk.get("dac_stddev")

        base_dac = baseline_dac_mean.get(key)
        if base_dac is not None and dac_mean is not None:
            dac_delta = dac_mean - base_dac
            dac_comp = f"{_fmt(base_dac, '>10.3f', 10)}{_fmt(dac_mean, '>10.3f', 10)}{_fmt(dac_delta, '>+10.3f', 10)}"
        else:
            dac_comp = f"{'---':>10}{'---':>10}{'---':>10}"

        if dac_n is not None and dac_n > 0:
            lines.append(
                f"{name:<6}"
                f"{'':>18}"
                f"{'':>10}"
                f"{'':>14}"
                f"{'':>6}"
                f"{_fmt(dac_mean, '>9.3f', 9)}"
                f"{_fmt(dac_sd, '>8.3f', 8)}"
                f"{dac_n:>6}"
                f"  "
                f"{dac_comp}"
            )
        else:
            lines.append(
                f"{name:<6}"
                f"{'':>18}"
                f"{'':>10}"
                f"{'':>14}"
                f"{'':>6}"
                f"{'---':>9}"
                f"{'---':>8}"
                f"{'---':>6}"
                f"  "
                f"{dac_comp}"
            )

    lines.append("")

    # ==============================================================
    # NOW servo — per-second calibration detail
    # ==============================================================
    if cal == "NOW":
        vref = 3.002

        lines.append(
            f"{'CAL':<6}"
            f"{'SEC_RES':>12}"
            f"{'PRED_NS':>14}"
            f"{'EDGE_GNSS':>20}"
            f"  "
            f"{'DAC':>10}"
            f"{'V_OUT':>10}"
            f"{'PPS_NS':>16}"
        )

        for name, key in [("OCXO1", "ocxo1"), ("OCXO2", "ocxo2")]:
            blk = r.get(key, {})
            residual = blk.get("second_residual_ns")
            pred_ns = blk.get("ns_count_next_second_prediction")
            edge_gnss_ns = blk.get("gnss_ns_at_edge")
            dac_now = blk.get("dac")
            pps_ns = blk.get("ns_count_at_pps")

            if residual is not None or pred_ns is not None or edge_gnss_ns is not None:
                dac_now_f = float(dac_now) if dac_now is not None else None
                v_now = (dac_now_f * vref / 65535.0) if dac_now_f is not None else None
                lines.append(
                    f"{name:<6}"
                    f"{_sign_int(residual, 12)}"
                    f"{_comma_int(pred_ns, 14)}"
                    f"{_comma_int(edge_gnss_ns, 20)}"
                    f"  "
                    f"{_fmt(dac_now_f, '>10.3f', 10)}"
                    f"{_fmt(v_now, '>10.5f', 10)}"
                    f"{_comma_int(pps_ns, 16)}"
                )
            else:
                lines.append(f"{name:<6}  --- (waiting for residual stream)")

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
    # ==============================================================
    dwt_cycles_per_pps = r.get("dwt", {}).get("next_second_prediction", _frag(r, "dwt_cycle_count_next_second_prediction"))
    dwt_cyccnt_at_pps = _frag(r, "dwt_cycle_count_at_pps")
    dwt_r_delta_raw = r.get("dwt", {}).get("delta_raw")
    dwt_r_pps_residual = r.get("dwt", {}).get("pps_residual")

    dwt_label_w = 12
    dwt_num_w = 20

    lines.append(
        f"DWT   "
        f"{'PREDICTED:':>{dwt_label_w}} {_comma_int(dwt_cycles_per_pps, dwt_num_w)}"
        f"    {'ACTUAL:':<{dwt_label_w}} {_comma_int(dwt_r_delta_raw, dwt_num_w)}"
        f"    {'PPS_RESIDUAL:':>13} {_sign_int(dwt_r_pps_residual, 6)}"
        f"    {'CYCCNT@PPS:':>11} {_comma_int(dwt_cyccnt_at_pps, 14)}"
    )

    # ── Crown jewels: authoritative 64-bit accumulators at last PPS ──
    dwt_cycles_64 = r.get("teensy_dwt_cycles")
    gnss_ns_64 = n * 1_000_000_000 if n else None

    lines.append(
        f"      "
        f"{'DWT_CYCLES:':>{dwt_label_w}} {_comma_int(dwt_cycles_64, dwt_num_w)}"
        f"    {'GNSS_NS:':<{dwt_label_w}} {_comma_int(gnss_ns_64, dwt_num_w)}"
    )
    lines.append("")

    # ==============================================================
    # INTERRUPT diagnostics — from fragment
    # ==============================================================
    lines.append(
        f"DIAG  "
        f"PPS app={_comma_int(_frag(r, 'pps_diag_approach_cycles'), 6)}"
        f" sh={_comma_int(_frag(r, 'pps_diag_shadow_dwt'), 10)}"
        f" an={_comma_int(_frag(r, 'pps_diag_anomaly_count'), 4)}"
        f"    "
        f"OX1 app={_comma_int(_frag(r, 'ocxo1_diag_approach_cycles'), 6)}"
        f" sh={_comma_int(_frag(r, 'ocxo1_diag_shadow_dwt'), 10)}"
        f" an={_comma_int(_frag(r, 'ocxo1_diag_anomaly_count'), 4)}"
        f"    "
        f"OX2 app={_comma_int(_frag(r, 'ocxo2_diag_approach_cycles'), 6)}"
        f" sh={_comma_int(_frag(r, 'ocxo2_diag_shadow_dwt'), 10)}"
        f" an={_comma_int(_frag(r, 'ocxo2_diag_anomaly_count'), 4)}"
    )
    lines.append(
        f"PRE   "
        f"PPS arm={_comma_int(_frag(r, 'pps_diag_prespin_arm_count'), 6)}"
        f" to={_comma_int(_frag(r, 'pps_diag_prespin_timeout_count'), 4)}"
        f"    "
        f"OX1 arm={_comma_int(_frag(r, 'ocxo1_diag_prespin_arm_count'), 6)}"
        f" to={_comma_int(_frag(r, 'ocxo1_diag_prespin_timeout_count'), 4)}"
        f"    "
        f"OX2 arm={_comma_int(_frag(r, 'ocxo2_diag_prespin_arm_count'), 6)}"
        f" to={_comma_int(_frag(r, 'ocxo2_diag_prespin_timeout_count'), 4)}"
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
    ("CLOCKS", lambda: _safe_lines(clocks_combined_readout)),
]
