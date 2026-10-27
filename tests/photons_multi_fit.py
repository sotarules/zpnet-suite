"""Read-only, multi-campaign persisted voltage-window calibration.

Usage: zt photons_multi_fit HeatCool3 HeatCool4 [--block-seconds 300]
Fits linear, quadratic, and cubic raw-LAP response using the exact persisted
model-authoritative HIGH5/HIGH60 predictor. --predictor high60 requests the
retained comparison witness explicitly. No alternate smoothing; no writes to database or firmware.
"""
import argparse
import json
import math
import statistics as st
import sys
from collections import defaultdict

from zpnet.shared.report_db import add_read_arguments, validate_read_arguments, campaign_rows

from zpnet.shared.photons_voltage import voltage_adjustment, voltage_window, VOLTAGE_MODELS

REF = 475.725600719
WINDOW_ACQUISITION_IDS = {
    'PD200T_HIGH60_LINEAR_V1': 0x4E4F5232,  # HeatCool1 / NOR2
    'PD200T_HIGH60_LINEAR_V2': 0x4E4F5233,  # HighLow1 / NOR3
    'PD200T_HIGH60_QUADRATIC_V1': 0x4E4F5234,  # HighLow2 trial / NOR4
    'PD200T_HIGH5_QUADRATIC_V1': 0x4E4F5235,  # HighLow3 trial / NOR5
}


def get_rows(name, args):
    projection = """
        SELECT id, schema #>> '{}' AS report_schema, jsonb_build_object(
      'time',published_at_utc, 'sequence',sequence,
      'photons',jsonb_build_object(
        'snapshot_ok',photons #> '{snapshot_ok}',
        'fragment_elapsed_ms',photons #> '{fragment_elapsed_ms}',
        'stats',jsonb_build_object('reset_count',photons #> '{stats,reset_count}',
                                  'acquisition_id',photons #> '{stats,acquisition_id}'),
        'pd_out',jsonb_build_object('sample_count',photons #> '{pd_out,sample_count}',
                                   'high_window',photons #> '{pd_out,high_window}',
                                   'high5_window',photons #> '{pd_out,high5_window}'),
        'normalization',photons #> '{voltage_normalization}',
        'flight',photons #> '{race,flight_ns}'),
        'measurement',measurement) AS p
    """
    for row in campaign_rows(name, projection, args):
        yield row['id'], json.loads(row['p']) if isinstance(row['p'], str) else row['p']


def observation(ident, p, predictor='authority'):
    f = p['photons']
    norm = f['normalization']
    if f['snapshot_ok'] is not True:
        return None
    if norm is not None:
        model = norm['model_id']
        if model not in VOLTAGE_MODELS:
            raise ValueError(f'{ident}: unknown voltage model {model}')
        if int(f['stats']['acquisition_id']) != VOLTAGE_MODELS[model][0]:
            raise ValueError(f'{ident}: voltage model/acquisition identity mismatch')
        if model in WINDOW_ACQUISITION_IDS:
            voltage_window(norm, f['pd_out'])
    # Legacy NOR1/raw rows retain the report's historical HIGH60 comparison.
    selected = ('high5' if norm is not None and norm['model_id'] == 'PD200T_HIGH5_QUADRATIC_V1'
                else 'high60') if predictor == 'authority' else predictor
    key, target = ('high5_window', 5000) if selected == 'high5' else ('high_window', 60000)
    w = f['pd_out'][key]
    if w is None:
        raise ValueError(f'{ident}: {selected.upper()} was not recorded; choose a recorded predictor')
    if int(w['target_ms']) != target:
        raise ValueError(f'{ident}: predictor window duration mismatch')
    if int(w['elapsed_ms']) < target or int(w['sample_count']) < 1:
        return None
    voltage = float(w['high_mean_mv'])
    pub = f['flight']
    if norm is not None:
        expected = voltage_adjustment(norm, norm['high_mv'])
        if not math.isclose(float(norm['adjustment_ns']), expected, rel_tol=0.0, abs_tol=1e-6):
            raise ValueError(f'{ident}: voltage formula closure failure')
        raw = norm['raw_flight_ns']
        n = int(raw['n'])
        if n == 0:
            return None
        if int(pub['n']) != n or abs(float(raw['mean'])+float(norm['adjustment_ns'])-float(pub['mean'])) > 1e-6:
            raise ValueError(f'{ident}: LAP closure failure')
        raw_mean = float(raw['mean'])
    else:
        measurement = p['measurement']
        if not measurement or measurement['raw_flight_ns'] is None:
            return None
        raw = measurement['raw_flight_ns']
        n = int(raw['n'])
        if n == 0:
            return None
        raw_mean = float(raw['mean'])
        if int(pub['n']) != n:
            raise ValueError(f'{ident}: raw/published count mismatch')
    from datetime import datetime
    timestamp = datetime.fromisoformat(p['time'].replace('Z','+00:00')).timestamp()
    return (int(p['sequence']),int(f['stats']['reset_count']),timestamp,
            voltage,raw_mean,float(pub['mean']),n,int(f['fragment_elapsed_ms'])/1000,
            selected,int(f['stats']['acquisition_id']))


def blocks(name, seconds, args):
    output, group, previous = [], [], None
    predictors = set()
    seen, omitted = 0, 0
    def flush():
        nonlocal group, omitted
        if group:
            duration = sum(x[7] for x in group)
            if duration >= .9*seconds:
                n = sum(x[6] for x in group)
                output.append((name, sum(x[3]*x[6] for x in group)/n,
                               sum(x[4]*x[6] for x in group)/n,
                               sum(x[5]*x[6] for x in group)/n,
                               tuple(sum((x[3]-args.reference_mv)**power*x[6] for x in group)/n
                                     for power in (1, 2, 3)), group[0][8]))
            else:
                omitted += len(group)
            group = []
    for ident,p in get_rows(name, args):
        seen += 1
        r = observation(ident,p,args.predictor)
        if r is None:
            flush(); previous = None; continue
        predictors.add(r[8])
        if previous and (r[0] != previous[0]+1 or r[1] != previous[1]
                         or r[8:] != previous[8:]
                         or not 0 < r[2]-previous[2] < 3):
            flush()
        group.append(r)
        if sum(x[7] for x in group) >= seconds:
            flush()
        previous = r
    flush()
    print(f'  {name}: rows={seen} complete_blocks={len(output)} omitted_partial_fragments={omitted}; predictors={sorted(predictors)}')
    return output


def linear_solve(matrix, rhs):
    a = [list(row)+[y] for row,y in zip(matrix,rhs)]
    n = len(a)
    for j in range(n):
        pivot = max(range(j,n),key=lambda i:abs(a[i][j]))
        a[j],a[pivot] = a[pivot],a[j]
        if abs(a[j][j]) < 1e-11:
            raise ValueError('insufficient independent voltage variation')
        for i in range(j+1,n):
            t = a[i][j]/a[j][j]
            for k in range(j,n+1):
                a[i][k] -= t*a[j][k]
    result = [0.0]*n
    for i in range(n-1,-1,-1):
        result[i] = (a[i][-1]-sum(a[i][j]*result[j] for j in range(i+1,n)))/a[i][i]
    return result


def fit(rows, degree, reference):
    scale = max(max(r[1] for r in rows)-min(r[1] for r in rows),1.0)
    groups = defaultdict(list)
    for campaign,v,y,_,moments,_source in rows:
        # E[x^2] is not E[x]^2: pool per-fragment polynomial features exactly.
        groups[campaign].append(([moments[k-1]/scale**k for k in range(1,degree+1)],y))
    matrix = [[0.0]*degree for _ in range(degree)]
    rhs = [0.0]*degree
    for group in groups.values():
        xm = [st.fmean(r[0][i] for r in group) for i in range(degree)]
        ym = st.fmean(r[1] for r in group)
        for x,y in group:
            dx = [x[i]-xm[i] for i in range(degree)]
            for i in range(degree):
                rhs[i] += dx[i]*(y-ym)
                for j in range(degree):
                    matrix[i][j] += dx[i]*dx[j]
    return [c/scale**(i+1) for i,c in enumerate(linear_solve(matrix,rhs))]


def correction(v, coeff, ref):
    x = v-ref
    return -sum(c*x**(i+1) for i,c in enumerate(coeff))


def evaluate(rows, coeff, ref):
    groups = defaultdict(list)
    for name,v,raw,pub,moments,_source in rows:
        adjustment = -sum(c*moments[i] for i,c in enumerate(coeff))
        groups[name].append((v,raw,pub,raw+adjustment))
    results = {}
    for name,group in groups.items():
        candidate = [r[3] for r in group]
        errors = [x-candidate[0] for x in candidate]
        span = lambda values: max(values)-min(values)
        results[name] = (math.sqrt(st.fmean(e*e for e in errors)),
                         span([r[1] for r in group]),span([r[2] for r in group]),
                         span(candidate),candidate[-1]-candidate[0],
                         min(r[0] for r in group),max(r[0] for r in group))
    return results


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('campaigns',nargs='+')
    parser.add_argument('--block-seconds',type=int,default=300)
    parser.add_argument('--max-degree',type=int,choices=(1,2,3),default=3)
    parser.add_argument('--reference-mv',type=float,default=REF)
    parser.add_argument("--predictor", choices=("authority", "high5", "high60"), default="authority")
    add_read_arguments(parser)
    a = parser.parse_args(argv[1:])
    validate_read_arguments(parser, a)
    if len(set(a.campaigns)) != len(a.campaigns) or a.block_seconds < 10 or not math.isfinite(a.reference_mv):
        parser.error('invalid campaign names, block duration, or reference')
    print('PHOTONS MULTI-CAMPAIGN VOLTAGE REGRESSION 2026-10-27.1')
    print(f'READ ONLY; persisted {a.predictor} predictor; block={a.block_seconds}s reference={a.reference_mv:.9f}mV')
    rows = [r for name in a.campaigns for r in blocks(name,a.block_seconds,a)]
    predictors = {r[5] for r in rows}
    if len(predictors) > 1:
        raise ValueError('do not combine HIGH5 and HIGH60 fits; select --predictor high60 for a common retained witness')
    label = next(iter(predictors)).upper() if predictors else a.predictor.upper()
    print(f'Predictor: {label}; polynomial features evaluated per fragment before pooling.')
    counts = {name: sum(r[0] == name for r in rows) for name in a.campaigns}
    if any(n < 2 for n in counts.values()):
        raise ValueError('each campaign needs at least two complete blocks of window-qualified data; ' + str(counts) + '. Use --block-seconds 60 for shorter campaigns, or collect longer runs.')
    candidates = {}
    for degree in range(1,a.max_degree+1):
        try:
            coeff = fit(rows,degree,a.reference_mv)
            train = evaluate(rows,coeff,a.reference_mv)
            folds = []
            if len(a.campaigns)>1:
                for holdout in a.campaigns:
                    c = fit([r for r in rows if r[0]!=holdout],degree,a.reference_mv)
                    folds.append(evaluate([r for r in rows if r[0]==holdout],c,a.reference_mv)[holdout][0])
            train_rms = math.sqrt(st.fmean(v[0]**2 for v in train.values()))
            cv_rms = math.sqrt(st.fmean(x*x for x in folds)) if folds else None
            candidates[degree] = (coeff,train,train_rms,cv_rms)
            print(f'degree={degree}: train drift RMS={train_rms:.6f} ns; '
                  + (f'held-out-campaign RMS={cv_rms:.6f} ns' if cv_rms is not None else 'no held-out campaign'))
        except ValueError as exc:
            print(f'degree={degree}: unidentifiable ({exc})')
    if not candidates:
        raise ValueError('no identifiable model')
    best = min(candidates,key=lambda d: candidates[d][3] if len(a.campaigns)>1 else candidates[d][2])
    coeff,train,_,_ = candidates[best]
    print(f'\nSELECTED degree={best} (selection={"leave-one-campaign-out" if len(a.campaigns)>1 else "training only; not validated"})')
    print(f'x = {label}_MV - {a.reference_mv:.9f}')
    print('CORRECTION A(V) = '+' '.join(f'{-c:+.12g}*x^{i+1}' for i,c in enumerate(coeff))+' ns')
    print('A(V) is relative to reference; any constant shift leaves drift unchanged.')
    for name in a.campaigns:
        rms,raw,pub,corrected,delta,vmin,vmax = train[name]
        print(f'{name}: {label}={vmin:.3f}..{vmax:.3f}mV; raw span={raw:.6f}ns; '
              f'deployed span={pub:.6f}ns; candidate span={corrected:.6f}ns; '
              f'candidate end-start={delta:+.6f}ns; candidate RMS={rms:.6f}ns')
    print('Cautions: campaign-specific offsets removed; only within-campaign drift assessed.')
    print('Trailing windows lag ramps and overlap; successive predictors are correlated.')
    print('Only rows with persisted complete-window voltage and raw-LAP witnesses used. Test the frozen formula on new campaigns.')
    print('No data, model coefficients, or producer state modified.')


if __name__ == '__main__':
    main(sys.argv)

