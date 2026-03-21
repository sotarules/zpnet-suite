"""
ZPNet OCXO Aging Decay Analyzer

Characterizes what happens to OCXO frequency when the servo is OFF —
the natural drift that Holdover must compensate for.

When GNSS disappears, the OCXOs are on their own.  This tool examines
campaigns where calibrate_ocxo = OFF (or any campaign, really) and
models the PPB trajectory over time to answer:

  1. What is the shape of the drift?  Linear?  Exponential?  Phased?
  2. What is the drift rate, and is it accelerating or decelerating?
  3. How closely do the two OCXOs track each other?
  4. Can we fit a simple model that predicts PPB at time T?
  5. What prediction error accumulates from that model?

Models fitted:
  - Linear:      ppb(t) = a·t + b
  - Quadratic:   ppb(t) = a·t² + b·t + c
  - Exponential: ppb(t) = A·(1 - e^(-t/τ)) + c  (thermal settling)

The exponential model is the one most likely to match OCXO physics:
  - A fast initial drift as the crystal adjusts post-servo
  - Asymptotic settling toward a steady-state aging rate
  - The time constant τ tells you how long until the transient dies

For Holdover, the critical output is:
  - The model coefficients (so we can extrapolate during GNSS loss)
  - The residual error (so we know our uncertainty budget per second)
  - The inter-OCXO divergence (so we know when cross-checking fails)

Usage:
    python -m zpnet.tests.aging_decay <campaign_name> [--servo-ok]
    .zt aging_decay FreeRun1
    .zt aging_decay Shakeout10 --servo-ok

By default, warns if the servo was active (since the drift is masked).
Pass --servo-ok to analyze servo-active campaigns anyway (useful for
examining the residual drift the servo couldn't fully correct).
"""

from __future__ import annotations

import json
import math
import sys
from typing import Any, Dict, List, Optional, Tuple

from zpnet.shared.db import open_db


# ─────────────────────────────────────────────────────────────────────
# Welford online accumulator
# ─────────────────────────────────────────────────────────────────────

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
        if x < self.min_val:
            self.min_val = x
        if x > self.max_val:
            self.max_val = x

    @property
    def stddev(self) -> float:
        return math.sqrt(self.m2 / (self.n - 1)) if self.n >= 2 else 0.0

    @property
    def stderr(self) -> float:
        return self.stddev / math.sqrt(self.n) if self.n >= 2 else 0.0


# ─────────────────────────────────────────────────────────────────────
# Database fetch
# ─────────────────────────────────────────────────────────────────────

def fetch_timebase(campaign: str) -> List[Dict[str, Any]]:
    with open_db(row_dict=True) as conn:
        cur = conn.cursor()
        cur.execute(
            """
            SELECT payload
            FROM timebase
            WHERE campaign = %s
            ORDER BY (payload->>'pps_count')::int ASC
            """,
            (campaign,),
        )
        rows = cur.fetchall()

    result = []
    for row in rows:
        p = row["payload"]
        if isinstance(p, str):
            p = json.loads(p)
        result.append(p)
    return result


# ─────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────

def safe_float(val: Any) -> Optional[float]:
    if val is None:
        return None
    try:
        return float(val)
    except (ValueError, TypeError):
        return None


def safe_int(val: Any) -> Optional[int]:
    if val is None:
        return None
    try:
        return int(val)
    except (ValueError, TypeError):
        return None


def seconds_to_hms(s: int) -> str:
    h = s // 3600
    m = (s % 3600) // 60
    sec = s % 60
    return f"{h:02d}:{m:02d}:{sec:02d}"


def compute_ppb(ocxo_ns: int, gnss_ns: int) -> Optional[float]:
    if gnss_ns == 0:
        return None
    return ((ocxo_ns - gnss_ns) / gnss_ns) * 1e9


# ─────────────────────────────────────────────────────────────────────
# Model fitting — pure Python, no numpy dependency
# ─────────────────────────────────────────────────────────────────────

def fit_linear(ts: List[float], ys: List[float]) -> Tuple[float, float]:
    """
    Least-squares linear fit: y = a·t + b
    Returns (a, b).
    """
    n = len(ts)
    sum_t = sum(ts)
    sum_y = sum(ys)
    sum_tt = sum(t * t for t in ts)
    sum_ty = sum(t * y for t, y in zip(ts, ys))

    denom = n * sum_tt - sum_t * sum_t
    if abs(denom) < 1e-30:
        return 0.0, (sum_y / n if n > 0 else 0.0)

    a = (n * sum_ty - sum_t * sum_y) / denom
    b = (sum_y * sum_tt - sum_t * sum_ty) / denom
    return a, b


def fit_quadratic(ts: List[float], ys: List[float]) -> Tuple[float, float, float]:
    """
    Least-squares quadratic fit: y = a·t² + b·t + c
    Returns (a, b, c).

    Uses normal equations solved via Cramer's rule (3x3).
    """
    n = len(ts)
    s1 = sum(ts)
    s2 = sum(t * t for t in ts)
    s3 = sum(t * t * t for t in ts)
    s4 = sum(t * t * t * t for t in ts)
    sy = sum(ys)
    sty = sum(t * y for t, y in zip(ts, ys))
    st2y = sum(t * t * y for t, y in zip(ts, ys))

    # Normal equations:  [[s4 s3 s2] [s3 s2 s1] [s2 s1 n]] · [a b c]' = [st2y sty sy]'
    # Cramer's rule
    det_m = s4 * (s2 * n - s1 * s1) - s3 * (s3 * n - s1 * s2) + s2 * (s3 * s1 - s2 * s2)

    if abs(det_m) < 1e-30:
        a_lin, b_lin = fit_linear(ts, ys)
        return 0.0, a_lin, b_lin

    det_a = st2y * (s2 * n - s1 * s1) - s3 * (sty * n - s1 * sy) + s2 * (sty * s1 - s2 * sy)
    det_b = s4 * (sty * n - s1 * sy) - st2y * (s3 * n - s1 * s2) + s2 * (s3 * sy - sty * s2)
    det_c = s4 * (s2 * sy - sty * s1) - s3 * (s3 * sy - sty * s2) + st2y * (s3 * s1 - s2 * s2)

    return det_a / det_m, det_b / det_m, det_c / det_m


def fit_exponential(ts: List[float], ys: List[float]) -> Tuple[float, float, float, bool]:
    """
    Fit: y = A·(1 - e^(-t/τ)) + c

    This is a nonlinear model.  We use a simple iterative approach:
    grid search over τ, then solve the resulting linear system for A and c.

    Returns (A, tau, c, converged).
    """
    if len(ts) < 10:
        return 0.0, 1.0, 0.0, False

    t_max = max(ts)
    if t_max < 1.0:
        return 0.0, 1.0, 0.0, False

    best_sse = float("inf")
    best_A = 0.0
    best_tau = 1.0
    best_c = 0.0

    # Grid search over tau from 60s to t_max
    # Use logarithmic spacing for broad coverage
    tau_candidates = []
    tau_val = 60.0
    while tau_val < t_max * 2:
        tau_candidates.append(tau_val)
        tau_val *= 1.2

    for tau in tau_candidates:
        # For fixed tau, y = A·(1 - e^(-t/τ)) + c is linear in A and c
        # Let u_i = 1 - e^(-t_i/τ)
        # Then y_i = A·u_i + c
        # This is a standard linear fit in (u, y) space

        us = [1.0 - math.exp(-t / tau) for t in ts]

        n = len(us)
        su = sum(us)
        sy = sum(ys)
        suu = sum(u * u for u in us)
        suy = sum(u * y for u, y in zip(us, ys))

        denom = n * suu - su * su
        if abs(denom) < 1e-30:
            continue

        A = (n * suy - su * sy) / denom
        c = (sy * suu - su * suy) / denom

        sse = sum((y - (A * u + c)) ** 2 for u, y in zip(us, ys))

        if sse < best_sse:
            best_sse = sse
            best_A = A
            best_tau = tau
            best_c = c

    # Refine: narrow search around best tau
    lo = best_tau / 1.5
    hi = best_tau * 1.5
    for _ in range(50):
        mid1 = lo + (hi - lo) / 3
        mid2 = hi - (hi - lo) / 3

        def sse_at_tau(tau):
            us = [1.0 - math.exp(-t / tau) for t in ts]
            n = len(us)
            su = sum(us)
            sy = sum(ys)
            suu = sum(u * u for u in us)
            suy = sum(u * y for u, y in zip(us, ys))
            denom = n * suu - su * su
            if abs(denom) < 1e-30:
                return float("inf"), 0.0, 0.0
            A = (n * suy - su * sy) / denom
            c = (sy * suu - su * suy) / denom
            sse = sum((y - (A * u + c)) ** 2 for u, y in zip(us, ys))
            return sse, A, c

        sse1, A1, c1 = sse_at_tau(mid1)
        sse2, A2, c2 = sse_at_tau(mid2)

        if sse1 < sse2:
            hi = mid2
            if sse1 < best_sse:
                best_sse = sse1
                best_A = A1
                best_tau = mid1
                best_c = c1
        else:
            lo = mid1
            if sse2 < best_sse:
                best_sse = sse2
                best_A = A2
                best_tau = mid2
                best_c = c2

    return best_A, best_tau, best_c, True


def compute_residuals(ts: List[float], ys: List[float],
                      predict_fn) -> Tuple[Welford, float]:
    """
    Compute prediction residuals and max absolute error.
    Returns (welford_stats, max_abs_error).
    """
    w = Welford()
    max_abs = 0.0

    for t, y in zip(ts, ys):
        predicted = predict_fn(t)
        residual = y - predicted
        w.update(residual)
        if abs(residual) > max_abs:
            max_abs = abs(residual)

    return w, max_abs


# ─────────────────────────────────────────────────────────────────────
# Holdover error accumulation
# ─────────────────────────────────────────────────────────────────────

def compute_holdover_error(ts: List[float], ys: List[float],
                           predict_fn) -> List[Tuple[int, float, float]]:
    """
    Simulate holdover: at each second, the PPB prediction error
    accumulates into a nanosecond time error.

    PPB is parts-per-billion, so each second of holdover at X ppb
    error accumulates X nanoseconds of time error.

    Returns checkpoints: [(elapsed_s, accumulated_ns_error, ppb_error)]
    at 1min, 5min, 10min, 30min, 1hr, 2hr, 4hr, 8hr, 12hr, 24hr.
    """
    checkpoints_s = [60, 300, 600, 1800, 3600, 7200, 14400, 28800, 43200, 86400]
    results = []
    accum_ns = 0.0
    checkpoint_idx = 0

    for i, (t, y) in enumerate(zip(ts, ys)):
        predicted_ppb = predict_fn(t)
        ppb_error = y - predicted_ppb
        accum_ns += ppb_error  # 1 ppb for 1 second = 1 ns

        elapsed = int(t)
        while checkpoint_idx < len(checkpoints_s) and elapsed >= checkpoints_s[checkpoint_idx]:
            results.append((checkpoints_s[checkpoint_idx], accum_ns, ppb_error))
            checkpoint_idx += 1

    return results


# ─────────────────────────────────────────────────────────────────────
# Phased drift detection
# ─────────────────────────────────────────────────────────────────────

def detect_phases(ts: List[float], ys: List[float],
                  window: int = 300) -> List[Dict[str, Any]]:
    """
    Slide a window across the data and compute local drift rate (ppb/hour).
    Returns a list of phase descriptors where the drift rate changes
    significantly.
    """
    if len(ts) < window * 2:
        return []

    phases = []
    step = max(1, window // 2)

    prev_rate = None
    phase_start = 0

    for i in range(0, len(ts) - window, step):
        chunk_t = ts[i:i + window]
        chunk_y = ys[i:i + window]
        a, _ = fit_linear(chunk_t, chunk_y)
        rate_ppb_per_hour = a * 3600.0

        if prev_rate is not None:
            # Significant change: rate changed by more than 50% or flipped sign
            rate_changed = (
                abs(rate_ppb_per_hour - prev_rate) > max(0.5, abs(prev_rate) * 0.5)
                or (prev_rate * rate_ppb_per_hour < 0 and abs(prev_rate) > 0.1)
            )

            if rate_changed:
                phases.append({
                    "start_s": int(ts[phase_start]),
                    "end_s": int(ts[i]),
                    "rate_ppb_hr": prev_rate,
                    "duration_s": int(ts[i] - ts[phase_start]),
                })
                phase_start = i

        prev_rate = rate_ppb_per_hour

    # Final phase
    if prev_rate is not None:
        phases.append({
            "start_s": int(ts[phase_start]),
            "end_s": int(ts[-1]),
            "rate_ppb_hr": prev_rate,
            "duration_s": int(ts[-1] - ts[phase_start]),
        })

    return phases


# ─────────────────────────────────────────────────────────────────────
# Main analysis
# ─────────────────────────────────────────────────────────────────────

def analyze(campaign: str, servo_ok: bool = False) -> None:
    rows = fetch_timebase(campaign)

    if not rows:
        print(f"No TIMEBASE rows found for campaign '{campaign}'")
        return

    total = len(rows)
    first_pps = safe_int(rows[0].get("pps_count") or rows[0].get("teensy_pps_count")) or 0
    last_pps = safe_int(rows[-1].get("pps_count") or rows[-1].get("teensy_pps_count")) or 0
    campaign_seconds = last_pps - first_pps
    first_ts = rows[0].get("gnss_time_utc") or rows[0].get("system_time_utc", "?")
    last_ts = rows[-1].get("gnss_time_utc") or rows[-1].get("system_time_utc", "?")

    # Check servo state
    servo_modes = set()
    for row in rows:
        mode = row.get("calibrate_ocxo")
        if mode and mode != "OFF":
            servo_modes.add(mode)

    print("=" * 90)
    print(f"OCXO AGING DECAY ANALYSIS — Campaign: {campaign}")
    print("=" * 90)
    print()
    print(f"  Time range:     {first_ts}")
    print(f"               -> {last_ts}")
    print(f"  Records:        {total}")
    print(f"  Duration:       {seconds_to_hms(campaign_seconds)} ({campaign_seconds:,} seconds)")

    if servo_modes:
        print(f"  Servo:          {', '.join(sorted(servo_modes))}")
        if not servo_ok:
            print()
            print("  ⚠️  Servo was ACTIVE during this campaign.")
            print("     The drift curve reflects servo-corrected behavior, not natural aging.")
            print("     For true aging characterization, run with servo OFF.")
            print("     Pass --servo-ok to analyze anyway.")
            print()
    else:
        print(f"  Servo:          OFF (free-running — ideal for aging analysis)")

    # ── Extract PPB time series ──

    ppb1_series: List[Tuple[float, float]] = []
    ppb2_series: List[Tuple[float, float]] = []

    for row in rows:
        pps = safe_int(row.get("pps_count") or row.get("teensy_pps_count"))
        gnss_ns = safe_int(row.get("teensy_gnss_ns"))
        o1_ns = safe_int(row.get("teensy_ocxo1_ns"))
        o2_ns = safe_int(row.get("teensy_ocxo2_ns"))

        if pps is None or gnss_ns is None:
            continue

        t = float(pps - first_pps)

        if o1_ns:
            ppb1 = compute_ppb(o1_ns, gnss_ns)
            if ppb1 is not None:
                ppb1_series.append((t, ppb1))

        if o2_ns:
            ppb2 = compute_ppb(o2_ns, gnss_ns)
            if ppb2 is not None:
                ppb2_series.append((t, ppb2))

    for label, series in [("OCXO1", ppb1_series), ("OCXO2", ppb2_series)]:
        if len(series) < 20:
            print(f"\n  [{label}] Insufficient data ({len(series)} points)")
            continue

        ts = [t for t, _ in series]
        ys = [y for _, y in series]

        print()
        print("=" * 90)
        print(f"  [{label}] — {len(series)} samples over {seconds_to_hms(int(ts[-1]))} ")
        print("=" * 90)

        # ── Raw trajectory ──

        print()
        print(f"  RAW TRAJECTORY")
        print(f"    First PPB:     {ys[0]:+.3f}")
        print(f"    Last PPB:      {ys[-1]:+.3f}")
        print(f"    Total drift:   {ys[-1] - ys[0]:+.3f} ppb")

        w = Welford()
        for y in ys:
            w.update(y)
        print(f"    Mean PPB:      {w.mean:+.3f}")
        print(f"    Stddev:        {w.stddev:.3f}")
        print(f"    Range:         {w.min_val:+.3f} -> {w.max_val:+.3f}")

        # ── Linear model ──

        a_lin, b_lin = fit_linear(ts, ys)
        res_lin, max_lin = compute_residuals(ts, ys, lambda t: a_lin * t + b_lin)

        print()
        print(f"  LINEAR MODEL:  ppb(t) = {a_lin:+.9f}·t + {b_lin:+.6f}")
        print(f"    Drift rate:    {a_lin * 3600:+.6f} ppb/hour")
        print(f"    Residual sd:   {res_lin.stddev:.3f} ppb")
        print(f"    Max |error|:   {max_lin:.3f} ppb")

        # ── Quadratic model ──

        a_q, b_q, c_q = fit_quadratic(ts, ys)
        res_quad, max_quad = compute_residuals(ts, ys, lambda t: a_q * t * t + b_q * t + c_q)

        print()
        print(f"  QUADRATIC MODEL:  ppb(t) = {a_q:+.12f}·t² + {b_q:+.9f}·t + {c_q:+.6f}")

        # Express the curvature in human terms
        if abs(a_q) > 1e-15:
            accel_ppb_hr2 = a_q * 3600 * 3600
            print(f"    Acceleration:  {accel_ppb_hr2:+.6f} ppb/hr²")
        initial_rate = b_q * 3600
        print(f"    Initial rate:  {initial_rate:+.6f} ppb/hour")
        print(f"    Residual sd:   {res_quad.stddev:.3f} ppb")
        print(f"    Max |error|:   {max_quad:.3f} ppb")

        improvement = res_lin.stddev / res_quad.stddev if res_quad.stddev > 0 else 0
        print(f"    vs linear:     {improvement:.2f}x improvement")

        # ── Exponential model ──

        A_exp, tau_exp, c_exp, exp_ok = fit_exponential(ts, ys)

        if exp_ok:
            def exp_predict(t):
                return A_exp * (1.0 - math.exp(-t / tau_exp)) + c_exp

            res_exp, max_exp = compute_residuals(ts, ys, exp_predict)

            print()
            print(f"  EXPONENTIAL MODEL:  ppb(t) = {A_exp:+.6f}·(1 - e^(-t/{tau_exp:.1f})) + {c_exp:+.6f}")
            print(f"    Amplitude A:   {A_exp:+.6f} ppb  (total thermal excursion)")
            print(f"    Time const τ:  {tau_exp:.1f} s  ({tau_exp / 60:.1f} min, {tau_exp / 3600:.2f} hr)")
            print(f"    Offset c:      {c_exp:+.6f} ppb  (initial PPB at t=0)")
            print(f"    Asymptote:     {A_exp + c_exp:+.6f} ppb  (steady-state)")
            print(f"    Residual sd:   {res_exp.stddev:.3f} ppb")
            print(f"    Max |error|:   {max_exp:.3f} ppb")

            improvement_exp = res_lin.stddev / res_exp.stddev if res_exp.stddev > 0 else 0
            print(f"    vs linear:     {improvement_exp:.2f}x improvement")

        # ── Phase detection ──

        phases = detect_phases(ts, ys)
        if phases and len(phases) > 1:
            print()
            print(f"  DRIFT PHASES")
            print(f"    {'PHASE':>6s}  {'START':>8s}  {'END':>8s}  {'DURATION':>10s}  {'RATE':>12s}")
            print(f"    {'─' * 6}  {'─' * 8}  {'─' * 8}  {'─' * 10}  {'─' * 12}")
            for i, ph in enumerate(phases):
                print(f"    {f'P{i}':<6s}"
                      f"  {seconds_to_hms(ph['start_s']):>8s}"
                      f"  {seconds_to_hms(ph['end_s']):>8s}"
                      f"  {seconds_to_hms(ph['duration_s']):>10s}"
                      f"  {ph['rate_ppb_hr']:>+10.3f}/hr")

        # ── Best model selection ──

        models = [("Linear", res_lin.stddev, max_lin)]
        models.append(("Quadratic", res_quad.stddev, max_quad))
        if exp_ok:
            models.append(("Exponential", res_exp.stddev, max_exp))

        models.sort(key=lambda m: m[1])
        best = models[0]

        print()
        print(f"  BEST FIT: {best[0]}  (residual sd = {best[1]:.3f} ppb, max |err| = {best[2]:.3f} ppb)")

        # ── Holdover error projection ──

        # Use the best model for holdover projection
        if best[0] == "Linear":
            predict_fn = lambda t: a_lin * t + b_lin
        elif best[0] == "Quadratic":
            predict_fn = lambda t: a_q * t * t + b_q * t + c_q
        elif best[0] == "Exponential" and exp_ok:
            predict_fn = exp_predict
        else:
            predict_fn = lambda t: a_lin * t + b_lin

        holdover = compute_holdover_error(ts, ys, predict_fn)

        if holdover:
            print()
            print(f"  HOLDOVER ERROR ACCUMULATION (using {best[0]} model)")
            print(f"    {'ELAPSED':>10s}  {'ACCUM ERROR':>14s}  {'PPB ERROR':>12s}")
            print(f"    {'─' * 10}  {'─' * 14}  {'─' * 12}")

            for elapsed_s, accum_ns, ppb_err in holdover:
                if elapsed_s < 3600:
                    elapsed_str = f"{elapsed_s // 60}min"
                else:
                    elapsed_str = f"{elapsed_s / 3600:.0f}hr"

                # Format accumulated error sensibly
                abs_ns = abs(accum_ns)
                if abs_ns < 1000:
                    err_str = f"{accum_ns:+.1f} ns"
                elif abs_ns < 1000000:
                    err_str = f"{accum_ns / 1000:+.2f} µs"
                else:
                    err_str = f"{accum_ns / 1000000:+.3f} ms"

                print(f"    {elapsed_str:>10s}  {err_str:>14s}  {ppb_err:>+10.3f} ppb")

    # ── Inter-OCXO correlation ──

    if ppb1_series and ppb2_series:
        print()
        print("=" * 90)
        print("INTER-OCXO AGING CORRELATION")
        print("=" * 90)

        min_len = min(len(ppb1_series), len(ppb2_series))
        ys1 = [y for _, y in ppb1_series[:min_len]]
        ys2 = [y for _, y in ppb2_series[:min_len]]

        # Correlation of PPB trajectories
        m1 = sum(ys1) / min_len
        m2 = sum(ys2) / min_len
        cov = sum((a - m1) * (b - m2) for a, b in zip(ys1, ys2)) / (min_len - 1) if min_len >= 2 else 0
        s1 = math.sqrt(sum((a - m1)**2 for a in ys1) / (min_len - 1)) if min_len >= 2 else 0
        s2 = math.sqrt(sum((b - m2)**2 for b in ys2) / (min_len - 1)) if min_len >= 2 else 0
        corr = cov / (s1 * s2) if (s1 > 0 and s2 > 0) else 0

        print(f"\n  PPB correlation:       r = {corr:+.6f}")

        # Divergence
        diff_w = Welford()
        for a, b in zip(ys1, ys2):
            diff_w.update(a - b)

        print(f"  PPB divergence mean:   {diff_w.mean:+.3f} ppb")
        print(f"  PPB divergence sd:     {diff_w.stddev:.3f} ppb")
        print(f"  PPB divergence range:  {diff_w.min_val:+.3f} -> {diff_w.max_val:+.3f}")

        # Divergence rate
        diff_series = [(ppb1_series[i][0], ys1[i] - ys2[i]) for i in range(min_len)]
        diff_ts = [t for t, _ in diff_series]
        diff_ys = [y for _, y in diff_series]
        a_div, _ = fit_linear(diff_ts, diff_ys)

        print(f"  Divergence rate:       {a_div * 3600:+.6f} ppb/hr")

        if abs(corr) > 0.95:
            print(f"\n  → Excellent tracking — single model may suffice for both OCXOs")
        elif abs(corr) > 0.8:
            print(f"\n  → Good tracking — OCXOs share dominant drift mode")
        elif abs(corr) > 0.5:
            print(f"\n  → Moderate tracking — independent models recommended")
        else:
            print(f"\n  → Weak tracking — OCXOs aging independently")

        # Cross-check validity window
        # How long until divergence exceeds a threshold?
        if abs(a_div * 3600) > 0.001:
            for threshold_ns in [100, 1000, 10000]:
                # Each ppb of divergence per second = 1 ns/s
                # Divergence rate is ppb/s = ns/s²
                # Time to accumulate threshold: t = sqrt(2·threshold / rate) if rate > 0
                rate_ns_per_s2 = abs(a_div)  # ppb/s ≈ ns/s²
                if rate_ns_per_s2 > 0:
                    t_cross = math.sqrt(2 * threshold_ns / rate_ns_per_s2)
                    print(f"  Cross-check valid to ±{threshold_ns} ns:  ~{seconds_to_hms(int(t_cross))}")

    print()
    print("=" * 90)
    print("END OF AGING DECAY ANALYSIS")
    print("=" * 90)


# ─────────────────────────────────────────────────────────────────────
# Available campaigns listing
# ─────────────────────────────────────────────────────────────────────

def list_campaigns() -> None:
    try:
        with open_db(row_dict=True) as conn:
            cur = conn.cursor()
            cur.execute(
                """
                SELECT campaign, count(*) as cnt,
                       min((payload->>'pps_count')::int) as pps_min,
                       max((payload->>'pps_count')::int) as pps_max
                FROM timebase
                GROUP BY campaign
                ORDER BY max(ts) DESC
                LIMIT 15
                """
            )
            rows = cur.fetchall()
        if rows:
            print("Available campaigns:")
            print(f"  {'CAMPAIGN':<24s} {'RECORDS':>8s} {'DURATION':>10s}")
            print(f"  {'─' * 24} {'─' * 8} {'─' * 10}")
            for r in rows:
                dur = (r["pps_max"] or 0) - (r["pps_min"] or 0)
                print(f"  {r['campaign']:<24s} {r['cnt']:>8d} {seconds_to_hms(dur):>10s}")
    except Exception:
        pass


def main():
    if len(sys.argv) < 2:
        print("Usage: aging_decay <campaign_name> [--servo-ok]")
        print()
        print("  Analyzes OCXO frequency drift over time to characterize")
        print("  natural aging behavior for Holdover prediction.")
        print()
        print("  --servo-ok    Suppress warning when servo was active")
        print()
        list_campaigns()
        sys.exit(1)

    campaign = sys.argv[1]
    servo_ok = "--servo-ok" in sys.argv
    analyze(campaign, servo_ok=servo_ok)


if __name__ == "__main__":
    main()