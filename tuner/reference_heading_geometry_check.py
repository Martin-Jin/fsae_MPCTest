"""
Is the reference-heading swing measured in S12.8 caused by the ONLINE
planner's per-tick rebuild, or is it just what the track's own geometry
requires -- present even against a fixed, offline-optimal centreline?

Why this exists
----------------
S12.8 found both stacks chase a reference heading (d(ref_psi)/dt,
reconstructed as car_yaw - e_psi) that swings far faster than the car can
yaw, and that this drives 78-100% of heading-error growth. Every concrete
mechanism proposed since then has been tested and individually found too
small to be the cause:

  - S14: blend_paths' reset-bypass fires 0/1038 times on the recorded map.
  - S14.1: blended-path heading rate vs. rebuild distance, r=0.10-0.15 --
    "explains ~1-2% of variance... more likely intrinsic to the geometry
    the centreline is fitting" (its own stated null-result interpretation,
    never directly checked).
  - S19/its correction: the anchor-to-nearest-midpoint window artifact is
    real but tiny (3/4160 ticks, 0.07%).

If none of the planner's own instabilities explain the swing, the remaining
candidate is the one S14.1 already named but did not test directly: the
swing is simply what a car following this track's real curvature requires,
and "reference swings faster than the car can yaw" is a geometry fact, not
a planner defect.

This script tests that directly rather than by elimination. The rollout
already carries a second, independent reference alongside the planner's
online centreline: path_X/path_Y/path_Psi, the fixed global spline fit
once from the full recorded cone map (sim/track_io.py::load_recorded_track),
used for e_psi_true specifically BECAUSE it does not depend on the planner's
per-tick FOV-limited rebuild (see rollout_core.py's own comment: "the
REFERENCE is the planner's cone-derived, FOV-limited, EMA-blended
centreline rather than path_X/path_Y"). Reconstructing
ref_psi_true = psi - e_psi_true gives the heading of that fixed reference
at the car's tracked position on every tick, with the planner's rebuild
loop entirely out of the loop. If d(ref_psi_true)/dt is already as fast as
d(ref_psi)/dt (the planner's online reference), the swing is geometry, full
stop -- no further planner-side mechanism can be responsible for it. If
d(ref_psi_true)/dt is much slower, the online planner is adding heading-rate
on top of real geometry, and the planner's spatial fit (centerline_planner.py,
boundary.py) is reopened as the cause after all.

Usage
-----
    python3 -m tuner.reference_heading_geometry_check
"""
import os
import sys

os.environ.setdefault("MPLBACKEND", "Agg")

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))


def main():
    from model.vehicle_physics import VehicleParams
    from tuner.recorded_map_rollout import DEFAULT_MAP
    from sim.track_io import load_recorded_track
    from sim.rollout_core import compute_step_budget, run_core_rollout
    from settings import N_HORIZON, Q_diag, R_diag, R_rate_diag, ROLLOUT_EPS, ROLLOUT_MAX_ITER
    from tuner.offline_tuner import get_cached_model

    path_X, path_Y, path_Psi, path_v, blue, yellow = load_recorded_track(DEFAULT_MAP)
    dyn_max, num_steps = compute_step_budget(path_X, path_Y, path_v)
    params = VehicleParams()
    Q = np.diag(Q_diag)
    R = np.diag(R_diag)
    R_rate = np.diag(R_rate_diag)
    u_min = np.array([-params.max_steer, params.max_accel_brake])
    u_max = np.array([params.max_steer, params.max_accel])
    rollout = run_core_rollout(
        path_X, path_Y, path_Psi, path_v, blue, yellow,
        Q, R, R_rate, u_min, u_max, params,
        max_steps=num_steps, dynamic_max_steps=dyn_max,
        use_planner=True, model_lookup=get_cached_model,
        n_horizon=N_HORIZON, eps=ROLLOUT_EPS, max_iter=ROLLOUT_MAX_ITER,
        want_history=True,
    )

    h = rollout["history"]
    psi = np.asarray(h["psi"], float)
    e_psi = np.asarray(h["e_psi"], float)
    e_psi_true = np.asarray(h["e_psi_true"], float)
    dt = 0.05

    ref_psi_planner = np.unwrap(psi - e_psi)
    ref_psi_geom = np.unwrap(psi - e_psi_true)

    dref_planner = np.degrees(np.gradient(ref_psi_planner, dt))
    dref_geom = np.degrees(np.gradient(ref_psi_geom, dt))

    def stats(x):
        a = np.abs(x)
        return a.mean(), np.percentile(a, 90), np.percentile(a, 99), a.max()

    m_p, p90_p, p99_p, max_p = stats(dref_planner)
    m_g, p90_g, p99_g, max_g = stats(dref_geom)

    print(f"n ticks = {len(psi)}")
    print()
    print("=== |d(ref_psi)/dt|, deg/s ===")
    print(f"{'':32s} {'mean':>8s} {'p90':>8s} {'p99':>8s} {'max':>8s}")
    print(f"{'planner online reference':32s} {m_p:8.1f} {p90_p:8.1f} {p99_p:8.1f} {max_p:8.1f}")
    print(f"{'fixed geometric reference':32s} {m_g:8.1f} {p90_g:8.1f} {p99_g:8.1f} {max_g:8.1f}")
    print(f"{'ratio (planner / geometric)':32s} {m_p/m_g:8.2f} {p90_p/p90_g:8.2f} "
          f"{p99_p/p99_g:8.2f} {max_p/max_g:8.2f}")
    print()

    corr = np.corrcoef(dref_planner, dref_geom)[0, 1]
    print(f"Correlation(planner rate, geometric rate) across ticks = {corr:.3f}")
    print()
    print("If the fixed geometric reference already swings nearly as fast as")
    print("the planner's online one (ratio near 1, high correlation), S12.8's")
    print("swing is a property of the TRACK, not the planner's per-tick")
    print("rebuild -- no further planner-side mechanism can explain it. If the")
    print("planner's rate is much higher (ratio >> 1) or poorly correlated,")
    print("the planner is adding heading-rate on top of real geometry, and its")
    print("spatial fit (centerline_planner.py / boundary.py) is reopened.")


if __name__ == "__main__":
    main()
