"""Analyse an open-loop braking system-ID log: what deceleration does FSDS
actually deliver for a given brake command, at a given speed?

Companion to the `brake_sysid` ROS node
(`ros2/src/fsae_planning/control/fsae_control/fsae_control/brake_sysid.py`),
which drives FSDS directly with fixed throttle/brake, bypassing the MPC. See
that node's own docstring for why this measurement exists: two NMPC
speed-profile attempts were rejected downstream of an unverified assumption
about achievable braking, and a scan of ~90 closed-loop NMPC logs found
achieved deceleration usually MEETS OR EXCEEDS commanded `a_cmd` -- the
opposite of the lateral case that motivated `alat_ceiling`. This script
turns a sweep into the actual achieved-vs-commanded relationship, fit the
same way `alat_ceiling_flat/slope/intercept` was: from measurement, not
assumed.

For each (approach_speed, brake_cmd) test point, fits a straight line to
`v_actual` over that point's RECORD window (`d(v)/dt`) to get one achieved
deceleration value, then reports:
  * the achieved-vs-commanded ratio, by speed bin (mirrors the closed-loop
    scan's own table so the two are directly comparable)
  * whether achieved deceleration is roughly FLAT in speed (a genuine
    per-speed ceiling, like alat_ceiling) or falls off in a way that fits a
    flat-plus-slope law -- fit both and let the data choose, the same
    disambiguation steering_sysid_analysis.py already does for the lateral
    case, rather than assuming one shape.

Usage:
    python -m tuner.checks.brake_sysid_analysis <brake_sysid_log.csv>
"""
import sys

import numpy as np

from tuner.csv_log import load_columns


def load(path):
    return load_columns(path, string_columns=('phase',))


def summarise_points(d):
    """Collapse each RECORD window into one (v0, brake_cmd, achieved_decel)."""
    rec = d['phase'] == 'record'
    if not rec.any():
        return []
    idx = np.flatnonzero(rec)
    splits = np.flatnonzero(np.diff(idx) > 1)
    blocks = np.split(idx, splits + 1)

    pts = []
    for b in blocks:
        if len(b) < 5:
            continue
        t_full = d['t'][b]
        v_full = d['v_actual'][b]
        v0 = float(v_full[0])
        b_cmd = float(np.mean(d['brake_cmd'][b]))
        if v0 < 0.5 or b_cmd < 1e-3:
            continue
        # brake_sysid.py's RECORD phase keeps running (v_actual pinned near
        # 0, still logged) for up to RECORD_S after the car actually stops,
        # so the window can contain a long flat tail once STOP_SPEED is
        # reached. Fitting the WHOLE window drags the slope toward zero and
        # badly underestimates the true deceleration -- trim to the portion
        # before the car first drops near standstill.
        stopped = np.flatnonzero(v_full < 0.3)
        cut = int(stopped[0]) if len(stopped) else len(b)
        t, v = t_full[:cut], v_full[:cut]
        # Straight-line fit to v(t) over the (trimmed) window: its slope IS
        # the achieved deceleration, more robust to per-tick odom noise than
        # a two-point finite difference (the same reason the closed-loop
        # scan's own dv/dt windows were noisy at 0.25-0.3s).
        if len(np.unique(t)) < 3:
            continue
        slope = float(np.polyfit(t - t[0], v, 1)[0])
        v_cmd_equiv = float(np.mean(d['a_cmd_equiv'][b]))
        pts.append({
            'v0': v0, 'brake_cmd': b_cmd, 'decel': slope,
            'a_cmd_equiv': v_cmd_equiv, 'n': len(b),
        })
    return pts


def _trend(xs, ys):
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
        print("Did the run complete, and did the car reach the approach speeds?")
        return 1

    print(f"=== {path.split('/')[-1]} ===")
    print(f"{len(pts)} braking test points\n")

    moving = [p for p in pts if p['v0'] > 1.0]
    if len(moving) < 3:
        print("!! Too few moving test points "
              f"({len(moving)} of {len(pts)}) for a verdict.")
        return 1
    pts = moving

    print(f"  {'v0':>6} {'brake':>6} {'a_cmd_equiv':>11} {'achieved':>9} {'ratio':>6}")
    for p in sorted(pts, key=lambda q: (q['v0'], q['brake_cmd'])):
        ratio = abs(p['decel']) / max(abs(p['a_cmd_equiv']), 1e-6)
        print(f"  {p['v0']:6.2f} {p['brake_cmd']:6.2f} "
              f"{p['a_cmd_equiv']:11.2f} {p['decel']:9.2f} {ratio:6.2f}")

    ratios = np.array([abs(p['decel']) / max(abs(p['a_cmd_equiv']), 1e-6) for p in pts])
    print(f"\n  achieved/commanded ratio: mean={ratios.mean():.2f} "
          f"median={np.median(ratios):.2f}")
    shortfall = ratios < 1.0
    print(f"  points with a real shortfall (ratio<1): {shortfall.sum()}/{len(pts)} "
          f"({100*shortfall.mean():.0f}%)")

    print("\n--- achieved deceleration by approach-speed bin ---")
    bins = [(0, 5), (5, 8), (8, 11), (11, 14), (14, 20)]
    for lo, hi in bins:
        g = [p for p in pts if lo <= p['v0'] < hi]
        if not g:
            continue
        dec = np.array([p['decel'] for p in g])
        print(f"  v in [{lo},{hi}): n={len(g):2d}  "
              f"achieved mean={dec.mean():6.2f}  min={dec.min():6.2f}  max={dec.max():6.2f}")

    # --- does achieved deceleration fall off with speed? ------------------
    v0 = np.array([p['v0'] for p in pts])
    dec = np.array([abs(p['decel']) for p in pts])
    sl, co = _trend(v0, dec)
    print(f"\n--- achieved |decel| vs approach speed ---")
    print(f"  slope={sl:+.3f} (m/s^2 per m/s)  corr={co:+.2f}")
    if abs(co) < 0.3:
        print("  -> roughly FLAT in speed: consistent with a plain magnitude "
              "ceiling, same form as alat_ceiling_flat alone.")
    elif sl < 0:
        print("  -> achieved braking WEAKENS at higher speed: consider a "
              "flat+slope law (alat_ceiling's own shape) rather than one "
              "flat number.")
    else:
        print("  -> achieved braking STRENGTHENS at higher speed: NOT the "
              "alat_ceiling shape (which falls or holds flat, never rises) "
              "-- do not force-fit that form here.")

    # --- fit flat-only vs flat+slope, let R^2 choose ----------------------
    def r2(pred):
        ss = float(np.sum((dec - dec.mean()) ** 2))
        return 1.0 - float(np.sum((dec - pred) ** 2)) / ss if ss > 0 else np.nan

    flat = float(np.median(dec))
    r2_flat = r2(np.full_like(dec, flat))
    A = np.vstack([v0, np.ones_like(v0)]).T
    slope_fit, intercept_fit = np.linalg.lstsq(A, dec, rcond=None)[0]
    r2_line = r2(slope_fit * v0 + intercept_fit)

    print(f"\n--- candidate ceiling laws ---")
    print(f"  flat only:        ceiling={flat:.2f} m/s^2          R2={r2_flat:.3f}")
    print(f"  flat + slope:     {intercept_fit:.2f} + {slope_fit:.3f}*v   R2={r2_line:.3f}")
    if r2_line - r2_flat > 0.1:
        print("  -> speed-dependent form fits meaningfully better; use "
              f"flat={intercept_fit:.2f}, slope={slope_fit:.3f} if a ceiling "
              "is warranted at all (see the shortfall percentage above -- "
              "a ceiling is only warranted if shortfalls are common, not rare).")
    else:
        print("  -> no meaningful improvement from a speed-dependent term; "
              f"a flat {flat:.2f} m/s^2 (or none, if shortfalls are rare) "
              "is sufficient.")

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
