#!/usr/bin/env python3
"""
Attribute tracking error to individual adaptive features, from a live log.

Reads a control CSV containing the adaptive-feature trace columns (see
fsae_control/telemetry_logger.py's ADAPTIVE_COLUMNS) and reports, per corner,
which features actually fired and how hard -- so a wide corner can be pinned
on a specific multiplier instead of guessed at.

    python3 -m tuner.checks.analyze_adaptive_log <control_csv>

The key column is `corner_demand`: kappa_max_abs divided by the curvature the
FSDS lateral-acceleration ceiling permits at the current speed. Above 1.0 the
corner is not achievable at that speed *no matter how the weights are set*,
so a wide line there is a speed-profile problem and tuning Q/R cannot fix it.
The summary splits corners on exactly that boundary, because the two groups
need opposite responses.

Older logs predate these columns; this prints a clear message and exits
rather than half-working.
"""
import csv
import sys

import numpy as np

# Multipliers grouped by the weight they act on, for the per-corner breakdown.
GROUPS = {
    'Q_ey':    ['m_Q_ey_approach', 'm_Q_ey_straight', 'm_Q_ey_uturn', 'm_Q_ey_soften'],
    'Q_epsi':  ['m_Q_epsi_approach', 'm_Q_epsi_exit', 'm_Q_epsi_straight', 'm_Q_epsi_uturn'],
    'Q_r':     ['m_Q_r_relax', 'm_Q_r_straight', 'm_Q_r_uturn'],
    'R_steer': ['m_R_speed', 'm_R_straight'],
    'R_rate':  ['m_Rrate_corner', 'm_Rrate_antihunt'],
}


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(l for l in f if not l.startswith('#')))
    if not rows:
        sys.exit(f'{path}: no data rows')
    if 'corner_demand' not in rows[0]:
        sys.exit(f'{path}: no adaptive-feature trace columns -- this log predates '
                 'them. Re-run the car with the current fsae_control build.')
    out = {}
    for k in rows[0]:
        vals = [r[k] for r in rows]
        try:
            out[k] = np.array([float(v) if v != '' else np.nan for v in vals])
        except ValueError:
            pass
    return out


def corners(d, thresh=0.03, min_dur=0.7):
    """Segment the run into corners by the curvature the car actually held."""
    t, v, r = d['t'], d['v_actual'], d['yaw_rate']
    kap = np.where(np.abs(v) > 1.0, r / np.maximum(np.abs(v), 1e-6), 0.0)
    ks = np.convolve(np.abs(kap), np.ones(11) / 11, mode='same')
    inc, segs, i = ks > thresh, [], 0
    while i < len(inc):
        if inc[i]:
            j = i
            while j < len(inc) and inc[j]:
                j += 1
            if t[j - 1] - t[i] > min_dur:
                segs.append((i, j, np.sign(np.mean(kap[i:j]))))
            i = j
        else:
            i += 1
    return segs


def main(path):
    d = load(path)
    t, ey = d['t'], d['e_y']
    segs = corners(d)
    print(f'{path}\n{len(t)} steps, {t[-1] - t[0]:.1f}s, {len(segs)} corners\n')

    # Approach window = the 1.5 s before the car is measurably turning. That is
    # where anticipation either happens or doesn't, so the multipliers are
    # averaged there rather than over the whole corner.
    print(f"{'#':>3} {'t0':>6} {'dem':>5} {'kmax':>6} {'v':>5} {'ey_pk':>6} {'wide':>5} "
          f"{'Qey':>6} {'Qepsi':>6} {'Qr':>6} {'Rrt':>6} {'sat%':>5} {'uturn':>5}")
    rows = []
    for n, (i, j, sgn) in enumerate(segs):
        a = max(0, i - 30)                       # ~1.5 s at 20 Hz
        seg = ey[i:j]
        ey_pk = seg[np.argmax(np.abs(seg))]
        wide = (ey_pk * sgn) < 0 and abs(ey_pk) > 0.4
        dem = np.nanmax(d['corner_demand'][a:i]) if i > a else np.nan
        sat = float(np.mean(np.abs(d['steer_deg'][i:j]) > 24.0) * 100)
        row = dict(
            n=n, t0=t[i], dem=dem, kmax=np.nanmax(d['kappa_max_abs'][a:j]),
            v=np.nanmean(d['v_actual'][a:i]) if i > a else np.nan,
            ey_pk=ey_pk, wide=wide, sat=sat,
            uturn=np.nanmax(d['uturn_severity'][a:j]),
            Qey=np.nanmean(d['Q_ey_eff'][a:i]), Qepsi=np.nanmean(d['Q_epsi_eff'][a:i]),
            Qr=np.nanmean(d['Q_r_eff'][a:i]), Rrt=np.nanmean(d['Rrate_steer_eff'][a:i]),
        )
        rows.append(row)
        print(f"{n:>3} {row['t0']:>6.1f} {dem:>5.2f} {row['kmax']:>6.3f} {row['v']:>5.1f} "
              f"{ey_pk:>6.2f} {str(wide):>5} {row['Qey']:>6.2f} {row['Qepsi']:>6.2f} "
              f"{row['Qr']:>6.3f} {row['Rrt']:>6.2f} {sat:>5.1f} {row['uturn']:>5.2f}")

    # Feasible vs infeasible: the split that decides whether to tune weights at
    # all. Averaging across it hides the distinction and invites tuning a gain
    # to fix corners that are speed-limited.
    feas = [r for r in rows if r['dem'] <= 1.0]
    infeas = [r for r in rows if r['dem'] > 1.0]
    print(f"\n{'':─<78}")
    for nm, g in (('FEASIBLE   (demand<=1)', feas), ('INFEASIBLE (demand>1)', infeas)):
        if not g:
            continue
        w = sum(r['wide'] for r in g)
        print(f'{nm}: {len(g):>2} corners, {w} wide, mean |ey_pk| '
              f"{np.mean([abs(r['ey_pk']) for r in g]):.3f}, "
              f"mean sat {np.mean([r['sat'] for r in g]):.1f}%")
    if infeas:
        print('\n  Corners with demand>1 exceed the FSDS lateral-accel ceiling at '
              'their\n  entry speed. Weight tuning cannot fix these -- the speed '
              'profile must\n  brake earlier or target a lower corner speed.')

    # Per-feature activity. A multiplier that never leaves 1.0 is dead code on
    # this track; one pinned at its limit has no headroom left to give.
    print(f"\n{'':─<78}\nper-feature activity over the whole run:")
    print(f"{'multiplier':>20} {'min':>7} {'mean':>7} {'max':>7}  {'active%':>7}")
    for grp, keys in GROUPS.items():
        for k in keys:
            x = d[k][np.isfinite(d[k])]
            if not len(x):
                continue
            act = float(np.mean(np.abs(x - 1.0) > 0.02) * 100)
            flag = ''
            if act < 1.0:
                flag = '  <- never fires'
            elif np.percentile(np.abs(x - 1.0), 10) > 0 and act > 99:
                flag = '  <- always on'
            print(f'{k:>20} {x.min():>7.3f} {x.mean():>7.3f} {x.max():>7.3f}  {act:>6.1f}%{flag}')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
