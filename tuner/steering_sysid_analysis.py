"""Analyse an open-loop steering system-ID log and identify the gap mechanism.

Companion to the `steering_sysid` ROS node
(`ros2/src/fsae_planning/control/fsae_control/fsae_control/steering_sysid.py`),
which drives FSDS directly with fixed steering at fixed speeds, bypassing the
MPC.  See `docs/planning_control_sync.md` -> "MEASURED: the car's yaw response
is ~3x weaker than commanded" for why.

Reports, for each (speed, steering) test point, the achieved steering ratio

    s = delta_achieved / delta_commanded,   delta_achieved = atan(L*yaw_rate/v)

and then reads the pattern in s to name the mechanism:

  * s flat in speed AND angle      -> constant rack-scale error
  * s falls with speed             -> speed-scaled steering map in FSDS
  * s falls with angle             -> nonlinear rack, or tyre saturation
                                      (disambiguated by |a_lat| vs grip)
  * yaw decays while steer is held -> yaw damping / stability control

It also fits the wheelbase L implied by the data.  L is only a scale factor on
delta_achieved, so a wrong L inflates or deflates every s equally -- fitting it
shows whether a geometry mismatch alone could explain the deficit.

Usage:
    python -m tuner.steering_sysid_analysis <steering_sysid_log.csv>
"""
import sys

import numpy as np

# Nominal wheelbase (m): model.vehicle_physics lf + lr.
WHEELBASE = 1.55

# Peak lateral acceleration the car is known to reach (m/s2), measured from
# live lap logs.  Used to tell tyre saturation from a steering-map error.
GRIP_CEILING = 12.0


def load(path):
    with open(path) as fh:
        lines = [ln for ln in fh if not ln.startswith('#')]
    header = lines[0].strip().split(',')
    rows = []
    for ln in lines[1:]:
        parts = ln.strip().split(',')
        if len(parts) == len(header):
            rows.append(parts)
    cols = {}
    for i, name in enumerate(header):
        raw = [r[i] for r in rows]
        if name == 'phase':
            cols[name] = np.array(raw)
        else:
            cols[name] = np.array([float(x) if x else np.nan for x in raw])
    return cols


def summarise_points(d, L=WHEELBASE):
    """Collapse each RECORD window into one steady-state measurement."""
    rec = d['phase'] == 'record'
    if not rec.any():
        return []
    # Split into contiguous RECORD blocks.
    idx = np.flatnonzero(rec)
    splits = np.flatnonzero(np.diff(idx) > 1)
    blocks = np.split(idx, splits + 1)

    pts = []
    for b in blocks:
        if len(b) < 5:
            continue
        v = float(np.mean(d['v_actual'][b]))
        r = float(np.mean(d['yaw_rate'][b]))
        sn = float(np.mean(d['steer_norm'][b]))
        dc = float(np.mean(d['delta_cmd_rad'][b]))
        if abs(v) < 0.5 or abs(dc) < 1e-4:
            continue
        d_ach = math_atan(L * r / v)
        # Yaw drift across the window: a steady point should have none.
        half = len(b) // 2
        drift = float(np.mean(np.abs(d['yaw_rate'][b[half:]]))
                      - np.mean(np.abs(d['yaw_rate'][b[:half]])))
        pts.append({
            'v': v, 'steer_norm': sn, 'delta_cmd': dc,
            'yaw_rate': r, 'delta_ach': d_ach,
            's': d_ach / dc, 'a_lat': abs(v * r), 'drift': drift,
            'n': len(b),
        })
    return pts


def math_atan(x):
    return float(np.arctan(x))


def fit_wheelbase(pts):
    """Fit L such that atan(L*r/v) best matches delta_cmd (s == 1)."""
    if len(pts) < 3:
        return None
    grid = np.linspace(0.5, 6.0, 5501)
    best, bestL = None, None
    for L in grid:
        err = 0.0
        for p in pts:
            ach = np.arctan(L * p['yaw_rate'] / p['v'])
            err += (ach - p['delta_cmd']) ** 2
        if best is None or err < best:
            best, bestL = err, L
    return float(bestL)


def _trend(xs, ys):
    """Sign and strength of a linear trend; returns (slope, correlation)."""
    xs, ys = np.asarray(xs, float), np.asarray(ys, float)
    if len(xs) < 3 or np.allclose(xs, xs[0]):
        return 0.0, 0.0
    slope = float(np.polyfit(xs, ys, 1)[0])
    corr = float(np.corrcoef(xs, ys)[0, 1])
    return slope, corr


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1
    path = argv[1]
    d = load(path)
    pts = summarise_points(d)
    if not pts:
        print("No usable RECORD windows in this log.")
        print("Did the run complete, and did the car reach the target speeds?")
        return 1

    print(f"=== {path.split('/')[-1]} ===")
    print(f"{len(pts)} steady test points  (assumed L = {WHEELBASE} m)\n")

    # A car wedged against a wall still emits RECORD rows, but they are all
    # zero speed / zero yaw.  Fitting those yields R2 = nan for every model,
    # and the ranking would then hand back a confident, meaningless verdict.
    moving = [p for p in pts if abs(p['v']) > 1.0 and abs(p['yaw_rate']) > 1e-3]
    if len(moving) < 3:
        print("!! The car was not moving during the RECORD windows "
              f"({len(moving)} of {len(pts)} points have real motion).")
        print("   This usually means it stalled against a wall or never "
              "reached the target speed.")
        print("   No verdict: the log contains no usable measurement.")
        return 1
    if len(moving) < len(pts):
        print(f"   (ignoring {len(pts) - len(moving)} stationary point(s))\n")
        pts = moving
    print(f"  {'v':>6} {'steer_n':>8} {'d_cmd':>7} {'d_ach':>7} {'s':>6} "
          f"{'a_lat':>6} {'drift':>7}")
    for p in sorted(pts, key=lambda q: (q['v'], abs(q['steer_norm']))):
        print(f"  {p['v']:6.2f} {p['steer_norm']:8.2f} "
              f"{np.degrees(p['delta_cmd']):7.2f} "
              f"{np.degrees(p['delta_ach']):7.2f} {p['s']:6.3f} "
              f"{p['a_lat']:6.2f} {p['drift']:+7.3f}")

    s_all = np.array([p['s'] for p in pts])
    print(f"\n  s: mean={s_all.mean():.3f} min={s_all.min():.3f} "
          f"max={s_all.max():.3f} spread={s_all.max()-s_all.min():.3f}")

    # --- trend in speed, at comparable steering angle -------------------
    print("\n--- s vs SPEED (grouped by |steer_norm|) ---")
    speed_slopes = []
    for sn in sorted({round(abs(p['steer_norm']), 2) for p in pts}):
        g = [p for p in pts if abs(round(abs(p['steer_norm']), 2) - sn) < 1e-6]
        if len(g) < 3:
            continue
        sl, co = _trend([p['v'] for p in g], [p['s'] for p in g])
        speed_slopes.append(sl)
        print(f"  |steer_norm|={sn:.2f}  n={len(g)}  "
              f"ds/dv={sl:+.4f} per m/s  corr={co:+.2f}")

    # --- trend in steering angle, at comparable speed -------------------
    print("\n--- s vs STEERING (grouped by target speed) ---")
    angle_slopes = []
    for v in sorted({round(p['v']) for p in pts}):
        g = [p for p in pts if abs(round(p['v']) - v) < 1e-6]
        if len(g) < 3:
            continue
        sl, co = _trend([abs(p['steer_norm']) for p in g], [p['s'] for p in g])
        angle_slopes.append(sl)
        print(f"  v~{v:.0f} m/s  n={len(g)}  ds/dsteer={sl:+.4f}  corr={co:+.2f}")

    # --- implied wheelbase ---------------------------------------------
    Lfit = fit_wheelbase(pts)
    if Lfit:
        print(f"\n--- implied wheelbase ---")
        print(f"  L that would make s == 1: {Lfit:.2f} m "
              f"(nominal {WHEELBASE} m, ratio {Lfit/WHEELBASE:.2f}x)")
        if Lfit > 2.5:
            print("  -> implausible for this car; geometry alone cannot "
                  "explain the deficit")

    # --- yaw damping ----------------------------------------------------
    drifts = np.array([p['drift'] for p in pts])
    print(f"\n--- yaw drift while steering held ---")
    print(f"  mean={drifts.mean():+.4f} rad/s across the record window")
    if drifts.mean() < -0.02:
        print("  -> yaw DECAYS under held steering: suggests yaw damping / "
              "stability control in FSDS")

    # --- model comparison ------------------------------------------------
    # A falling s is NOT by itself diagnostic: a speed-scaled rack, genuine
    # understeer and tyre saturation all produce one.  They differ in FORM, so
    # fit each candidate to the achieved yaw rate (a common target, so the R^2
    # values are directly comparable) and let the data choose.
    v = np.array([p['v'] for p in pts])
    dc = np.array([p['delta_cmd'] for p in pts])
    rr = np.array([p['yaw_rate'] for p in pts])

    def r2(pred):
        ss = float(np.sum((rr - rr.mean()) ** 2))
        return 1.0 - float(np.sum((rr - pred) ** 2)) / ss if ss > 0 else np.nan

    def best(fn, grid):
        errs = [float(np.sum((rr - fn(g)) ** 2)) for g in grid]
        g = grid[int(np.argmin(errs))]
        return g, r2(fn(g))

    models = {}
    models['neutral (s=1)'] = (None, r2(v * dc / WHEELBASE))
    k, sc = best(lambda s: s * v * dc / WHEELBASE, np.linspace(0.05, 1.5, 1451))
    models['constant scale'] = (f'scale={k:.3f}', sc)
    # Speed-scaled rack: effective angle shrinks LINEARLY with speed.
    k, sc = best(lambda c: v * (dc * np.maximum(1.0 - c * v, 0.0)) / WHEELBASE,
                 np.linspace(0.0, 0.15, 1501))
    models['speed-scaled rack'] = (f'c={k:.4f} per m/s', sc)
    # Understeer: the v^2 law.
    k, sc = best(lambda K: v * dc / (WHEELBASE + K * v ** 2),
                 np.linspace(0.0, 0.20, 2001))
    models['understeer (v^2)'] = (f'K_us={k:.4f} charV='
                                  f'{np.sqrt(WHEELBASE/k) if k > 0 else float("inf"):.1f}', sc)
    # Saturation: yaw capped by a lateral-acceleration ceiling.
    k, sc = best(lambda A: np.sign(dc) * np.minimum(
        np.abs(v * dc / WHEELBASE), A / np.maximum(v, 0.1)),
        np.linspace(4.0, 25.0, 2101))
    models['grip saturation'] = (f'a_lat_max={k:.1f}', sc)

    print("\n--- candidate models (fitted to achieved yaw rate) ---")
    print(f"  {'model':<20} {'R2':>8}  params")
    ranked = sorted(models.items(), key=lambda kv: -(kv[1][1] if np.isfinite(kv[1][1]) else -9))
    for name, (prm, sc) in ranked:
        print(f"  {name:<20} {sc:8.3f}  {prm or ''}")

    # --- verdict ---------------------------------------------------------
    print("\n=== VERDICT ===")
    winner, (wparams, wr2) = ranked[0]
    runner, (_, rr2) = ranked[1] if len(ranked) > 1 else (None, (None, -9))
    margin = wr2 - rr2

    print(f"  best fit: {winner} (R2={wr2:.3f}, {wparams or 'no params'})")
    if margin < 0.05:
        print(f"  WARNING: '{runner}' is within {margin:.3f} R2 -- this log")
        print("  does not separate them. Widen the speed range (the models")
        print("  differ most at the extremes) and re-run.")

    if winner == 'neutral (s=1)':
        print("  FSDS delivers the commanded angle. The deficit seen in lap")
        print("  logs is NOT in the steering path -- look at the controller")
        print("  or the reference it is tracking.")
    elif winner == 'constant scale':
        print(f"  CONSTANT RACK-SCALE ERROR: FSDS's true lock is about "
              f"{25.0 * s_all.mean():.1f} deg,")
        print("  not the assumed 25. Fix MAX_STEER_RAD in fsds_bridge,")
        print("  mpc_core and control_utils together.")
    elif winner == 'speed-scaled rack':
        print("  SPEED-SCALED STEERING MAP inside FSDS: the simulator reduces")
        print("  effective lock as speed rises. The offline plant models no")
        print("  such thing. Model it in the plant, do NOT bend tyre params.")
    elif winner == 'understeer (v^2)':
        print("  GENUINE UNDERSTEER: the deficit follows the v^2 law, so it is")
        print("  vehicle dynamics, not a command-path scaling error. Note the")
        print("  offline plant cannot reach the live K_us with physical tyre")
        print("  parameters (needs C_f ~10% of physical) -- so if this wins,")
        print("  the mismatch is in mass/geometry/load transfer, not grip.")
    else:
        # A lateral-acceleration ceiling has two very different causes, and
        # they are told apart by WHERE the ceiling sits -- not by counting
        # points against a hardcoded grip figure (an earlier version did that
        # and printed "TYRE SATURATION ... 0/16 points near the ceiling",
        # which is self-contradictory whenever the fitted ceiling is well
        # below GRIP_CEILING).
        fitted = None
        try:
            fitted = float(str(wparams).split('=')[1])
        except Exception:
            pass
        print("  YAW IS CAPPED: more steering stops producing more cornering.")
        if fitted is not None:
            print(f"  fitted ceiling {fitted:.1f} m/s2 vs known peak "
                  f"{GRIP_CEILING:.1f} m/s2 on this car")
            if fitted < 0.8 * GRIP_CEILING:
                print("  -> the cap is FAR BELOW the car's demonstrated grip, so")
                print("     this is NOT tyre saturation. Something is limiting")
                print("     yaw directly -- e.g. a stability-control or yaw-")
                print("     damping term inside FSDS that the offline plant")
                print("     does not model. Check whether the cap engages above")
                print("     a threshold speed (the s-vs-speed table above): a")
                print("     tyre limit would not depend on speed.")
            else:
                print("  -> consistent with real tyre saturation; compare the")
                print("     fitted ceiling against the plant's peak a_lat.")
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
