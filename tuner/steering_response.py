"""Measure a car's steering->yaw response from a live telemetry CSV.

Diagnostic for the open sim-to-real gap: the live car's yaw response to a
steering command is ~3x weaker than commanded and weakens with speed, while
the offline plant is near neutral-steer.  See
`docs/planning_control_sync.md` -> "MEASURED: the car's yaw response is ~3x
weaker than commanded" for the findings this script produced.

Method — invert the kinematic bicycle on logged yaw rate:

    delta_achieved = atan(L * yaw_rate / v)

then compare against the logged `delta_cmd`.  Candidate response models are
scored against a COMMON target (achieved yaw rate) so their R^2 values are
directly comparable; scoring against delta_achieved instead inflates R^2 for
every model, because both sides scale with steering angle.

Usage:
    python -m tuner.steering_response <control_log.csv> [...]

Run it on any new live log to check whether the gap has moved.  The key
outputs are `K_us` (understeer, s^2) and the full-lock deficit; the offline
plant currently measures K_us ~ 0.0006 against the car's ~0.040.
"""
import sys

import numpy as np

from tuner.csv_log import load_columns

# Wheelbase (m): must match model.vehicle_physics lf + lr.
WHEELBASE = 1.55

# Quasi-steady gate.  The bicycle inversion is only valid once the car has
# settled; these bounds admit rows where the command and the yaw rate are both
# roughly stationary.  Deliberately not tighter: on a hunting car, stricter
# gates (a sustained multi-tick hold) return zero rows.
MAX_CMD_RATE_RAD_S = np.radians(20.0)
MAX_YAW_ACCEL = 0.6
MIN_SPEED = 3.0
MIN_STEER_RAD = np.radians(2.0)


def load_control_log(path):
    """Read a telemetry CSV, skipping the leading '#' metadata block."""
    return load_columns(path)


def _r2(y, pred):
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    if ss_tot <= 0:
        return float('nan')
    return 1.0 - float(np.sum((y - pred) ** 2)) / ss_tot


def quasi_steady_mask(t, v, r, delta):
    dt = np.maximum(np.gradient(t), 1e-3)
    d_delta = np.gradient(delta) / dt
    d_r = np.gradient(r) / dt
    return (
        np.isfinite(v) & np.isfinite(r) & np.isfinite(delta)
        & (v > MIN_SPEED)
        & (np.abs(delta) > MIN_STEER_RAD)
        & (np.abs(d_delta) < MAX_CMD_RATE_RAD_S)
        & (np.abs(d_r) < MAX_YAW_ACCEL)
    )


def fit_understeer(v, r, delta, L=WHEELBASE):
    """Fit K_us in the steady-state bicycle r = v*delta / (L + K_us * v^2).

    Returns (K_us, characteristic_speed, r2_on_yaw_rate).
    """
    def model(K):
        return v * delta / (L + K * v ** 2)

    # 1-D search: cheap, robust, and avoids a scipy dependency here.
    grid = np.linspace(-0.01, 0.20, 4201)
    errs = [float(np.sum((r - model(K)) ** 2)) for K in grid]
    K = float(grid[int(np.argmin(errs))])
    char_v = float(np.sqrt(L / K)) if K > 0 else float('inf')
    return K, char_v, _r2(r, model(K))


def report(path, L=WHEELBASE):
    d = load_control_log(path)
    t, v, r = d['t'], d['v_actual'], d['yaw_rate']
    delta = d['delta_cmd']

    print(f"=== {path.split('/')[-1]} ===")

    # Channel validation: yaw_rate must agree with the pose derivative,
    # otherwise every number below is measuring a logging bug.
    if 'car_yaw' in d:
        dyaw = np.gradient(np.unwrap(d['car_yaw'])) / np.maximum(np.gradient(t), 1e-3)
        m = np.isfinite(dyaw) & np.isfinite(r) & (v > MIN_SPEED)
        if m.sum() > 20:
            slope = float(np.sum(r[m] * dyaw[m]) / np.sum(r[m] * r[m]))
            corr = float(np.corrcoef(r[m], dyaw[m])[0, 1])
            print(f"  yaw_rate vs d(car_yaw)/dt: slope={slope:.3f} corr={corr:.3f}"
                  f"{'  <-- SUSPECT' if abs(slope - 1) > 0.2 or corr < 0.8 else ''}")

    m = quasi_steady_mask(t, v, r, delta)
    print(f"  quasi-steady points: {int(m.sum())} / {len(t)}")
    if m.sum() < 30:
        print("  !! too few points for a reliable fit")
        return
    vs, rs, ds = v[m], r[m], delta[m]

    # Neutral steer is what the offline plant does; a negative R^2 means the
    # offline plant predicts the car's yaw worse than guessing the mean.
    print(f"  R2 neutral-steer model  : {_r2(rs, vs * ds / L):7.3f}")
    X = vs * ds / L
    s_const = float(np.sum(X * rs) / np.sum(X * X))
    print(f"  R2 constant-scale model : {_r2(rs, s_const * X):7.3f}  (s={s_const:.3f})")
    K, char_v, r2u = fit_understeer(vs, rs, ds, L)
    print(f"  R2 understeer model     : {r2u:7.3f}  "
          f"K_us={K:.5f} s^2  char speed={char_v:.2f} m/s")

    # Full lock is the sharpest single statistic: it needs no model fit.
    sat = (np.abs(delta) >= np.radians(24.9)) & (v > MIN_SPEED) & np.isfinite(r)
    if sat.sum() > 10:
        v_m = float(v[sat].mean())
        r_m = float(np.abs(r[sat]).mean())
        ach = np.degrees(np.arctan(L * r_m / v_m))
        print(f"  AT FULL LOCK: n={int(sat.sum())} v={v_m:.2f} m/s "
              f"|r|={r_m:.3f} rad/s -> achieved {ach:.2f} deg "
              f"(deficit x{25.0 / max(ach, 1e-6):.2f})")
        print(f"               |a_lat| there = {v_m * r_m:.2f} m/s2 "
              f"(tyre saturation would need ~12)")


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1
    for p in argv[1:]:
        report(p)
        print()
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
