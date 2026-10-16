"""Locate the body point FSDS reports as the car's pose, from logged laps.

At low speed and low lateral acceleration tyre slip is small, so the rear
axle has almost no sideways velocity.  A point x_p ahead of the rear axle then
moves sideways at v_y = r * x_p, so v_y / r reads off x_p directly:

  * ~0       -> the pose is the rear axle
  * ~lr      -> the pose is the CG (lr = 0.85 in the controllers)

v_y comes from central-differenced logged positions rotated into the body
frame (the control CSVs do not log v_y).  Yaw rate r is taken from the same
pose (differenced car_yaw), not the logged yaw_rate column, so both come from
one source with one timestamp; the logged yaw_rate column reads ~11% higher
than the pose-derived rate and is reported alongside for comparison.  Rear
slip grows with speed and pulls v_y / r down, so the low-speed bins and the
v -> 0 intercept are the estimate; higher bins are a consistency check.

Usage:
    python -m tuner.investigations.pose_point_check <control_csv> [...]
"""
import sys

import numpy as np
import pandas as pd

# Samples whose 2-tick span falls outside this window straddle a dropped or
# stalled pose (see periodic_pose_teleport_investigation.md) and are skipped.
DT_MIN, DT_MAX = 0.06, 0.15
ALAT_MAX = 3.0     # m/s^2, keeps tyre slip small
R_MIN = 0.3        # rad/s, avoids dividing by near-zero yaw rate
SPEED_BINS = [(1, 3), (3, 5), (5, 7)]


def _samples(path):
    d = pd.read_csv(path, comment='#')
    if len(d) < 50:
        return None
    t = d.t.values
    x, y = d.car_x.values, d.car_y.values
    yaw = np.unwrap(d.car_yaw.values)
    dt = t[2:] - t[:-2]
    vxw = (x[2:] - x[:-2]) / dt
    vyw = (y[2:] - y[:-2]) / dt
    psi = yaw[1:-1]
    vx = vxw * np.cos(psi) + vyw * np.sin(psi)
    vy = -vxw * np.sin(psi) + vyw * np.cos(psi)
    r_pose = (yaw[2:] - yaw[:-2]) / dt
    r_log = d.yaw_rate.values[1:-1]
    keep = ((dt > DT_MIN) & (dt < DT_MAX) & (vx > 1.0)
            & (np.abs(r_log) > R_MIN) & (np.abs(vx * r_log) < ALAT_MAX))
    return vx[keep], vy[keep], r_pose[keep], r_log[keep]


def main(paths):
    parts = [s for s in map(_samples, paths) if s is not None]
    if not parts:
        print('no usable samples')
        return
    vx, vy, r_pose, r_log = (np.concatenate(c) for c in zip(*parts))
    print(f'{len(vx)} samples from {len(parts)} logs; '
          f'median r_pose / r_logged = {np.median(r_pose / r_log):.3f}')
    for r, name in [(r_pose, 'pose-derived r'), (r_log, 'logged yaw_rate')]:
        # vy = x_p * r + b * vx * r: x_p is the v -> 0 intercept.
        coef, *_ = np.linalg.lstsq(np.c_[r, vx * r], vy, rcond=None)
        print(f'\n{name}: v -> 0 intercept x_p = {coef[0]:.3f} m '
              f'(slope {coef[1]:+.4f} m per m/s)')
        for lo, hi in SPEED_BINS:
            m = (vx > lo) & (vx < hi)
            if m.sum() < 10:
                continue
            ls = np.sum(vy[m] * r[m]) / np.sum(r[m] ** 2)
            print(f'  v_x {lo}-{hi} m/s  n={m.sum():5d}  v_y/r = {ls:.3f} m')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    main(sys.argv[1:])
