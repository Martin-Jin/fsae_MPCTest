"""
Cross-check: are ticks where the planner's online reference heading swings
much faster than a fixed geometric reference demands explained by the
anchor/seed-jump artifact in the boundary-planner's midpoint selection, or
by a distinct mechanism?

Why this exists
----------------
A small fraction of ticks show the planner's online reference heading
swinging several times faster than a fixed geometric reference would
demand, and those ticks carry a much higher immediate steering-saturation
rate. A plausible but unconfirmed cause is the seed-midpoint anchor-jump
artifact: filter_cones_window's forward cutoff can drop the nearest
surviving midpoint as the car's pose crosses it, causing the path-build
seed to jump discontinuously. This script is the cross-check for that
hypothesis.

It re-runs the rollout, wraps build_path_walls() non-invasively (same
approach used to instrument blend_paths() elsewhere), and captures per-tick:
  - the identity (position) of the nearest-ahead midpoint that seeds
    _build_wall_path's chain
  - the raw (pre-blend) centreline's near-field tangent direction

then reports, for each high-excess tick, whether the seed midpoint changed
discontinuously at that tick (the seed-jump mechanism) or not (something
else -- e.g. a genuine sign-reversal corner transition, or a different
planner mechanism entirely).

Usage
-----
    python3 -m tuner.reference_excess_mechanism_check
"""
import os
import sys

os.environ.setdefault("MPLBACKEND", "Agg")

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

import planning.boundary as boundary  # noqa: E402
import sim.sim_track as sim_track  # noqa: E402

_orig_build_path_walls = boundary.build_path_walls

# One entry per build_path_walls() call, aligned to sim ticks.
_seed_log = []


def _seed_midpoint(midpoints, car_pos, car_yaw):
    """Same seed rule as _build_wall_path: nearest midpoint ahead of the car
    in the heading frame, falling back to nearest overall."""
    if len(midpoints) == 0:
        return None
    cos_y, sin_y = np.cos(car_yaw), np.sin(car_yaw)
    heading = np.array([cos_y, sin_y])
    rel = midpoints - car_pos
    fwd = rel @ heading
    dist = np.linalg.norm(rel, axis=1)
    ahead = fwd > 0.0
    if ahead.any():
        idx = np.where(ahead)[0][np.argmin(dist[ahead])]
    else:
        idx = int(np.argmin(dist))
    return midpoints[idx]


def _wrapped(blue_cones, yellow_cones, car_pos, car_yaw, *a, **kw):
    out = _orig_build_path_walls(blue_cones, yellow_cones, car_pos, car_yaw, *a, **kw)
    cl, blue_segs, yellow_segs, midpoints = out
    seed = _seed_midpoint(midpoints, np.asarray(car_pos), car_yaw)
    near_tangent = None
    if cl is not None and len(cl) >= 3:
        v = cl[2] - cl[0]
        if np.linalg.norm(v) > 1e-6:
            near_tangent = float(np.degrees(np.arctan2(v[1], v[0])))
    _seed_log.append((seed, near_tangent))
    return out


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

    boundary.build_path_walls = _wrapped
    sim_track.build_path_walls = _wrapped
    try:
        rollout = run_core_rollout(
            path_X, path_Y, path_Psi, path_v, blue, yellow,
            Q, R, R_rate, u_min, u_max, params,
            max_steps=num_steps, dynamic_max_steps=dyn_max,
            use_planner=True, model_lookup=get_cached_model,
            n_horizon=N_HORIZON, eps=ROLLOUT_EPS, max_iter=ROLLOUT_MAX_ITER,
            want_history=True,
        )
    finally:
        boundary.build_path_walls = _orig_build_path_walls
        sim_track.build_path_walls = _orig_build_path_walls

    h = rollout["history"]
    psi = np.asarray(h["psi"], float)
    e_psi = np.asarray(h["e_psi"], float)
    e_psi_true = np.asarray(h["e_psi_true"], float)
    u_steer = np.asarray(h["u_steer"], float)
    dt = 0.05

    n = min(len(psi), len(_seed_log))
    psi, e_psi, e_psi_true, u_steer = psi[:n], e_psi[:n], e_psi_true[:n], u_steer[:n]
    seeds = [_seed_log[i][0] for i in range(n)]
    raw_tangent = [_seed_log[i][1] for i in range(n)]

    ref_psi_planner = np.unwrap(psi - e_psi)
    ref_psi_geom = np.unwrap(psi - e_psi_true)
    dref_planner = np.degrees(np.gradient(ref_psi_planner, dt))
    dref_geom = np.degrees(np.gradient(ref_psi_geom, dt))
    excess = np.abs(dref_planner) - np.abs(dref_geom)

    # Seed-jump distance tick-to-tick (the anchor-jump mechanism signature).
    seed_jump = np.full(n, np.nan)
    for i in range(1, n):
        if seeds[i] is not None and seeds[i - 1] is not None:
            seed_jump[i] = float(np.linalg.norm(seeds[i] - seeds[i - 1]))

    hi = np.where(excess > 30.0)[0]
    print(f"n ticks = {n}, high-excess ticks (>30 deg/s) = {len(hi)}")
    print()
    print(f"{'tick':>5s} {'t':>7s} {'excess':>8s} {'seed_jump':>10s} "
          f"{'raw_tan':>9s} {'u_steer':>8s}")
    for i in hi:
        sj = seed_jump[i]
        sj_s = f"{sj:.2f}" if np.isfinite(sj) else "n/a"
        rt = raw_tangent[i]
        rt_s = f"{rt:8.1f}" if rt is not None else "     n/a"
        print(f"{i:5d} {i*dt:7.2f} {excess[i]:8.1f} {sj_s:>10s} {rt_s} {u_steer[i]:8.3f}")

    print()
    # A seed jump >1.0 m in one tick is the anchor-jump signature (a
    # discontinuous jump in the seed midpoint far larger than the car's own
    # motion that tick); flag ticks meeting that bar.
    seed_jump_like = np.isfinite(seed_jump) & (seed_jump > 1.0)
    overlap = np.intersect1d(hi, np.where(seed_jump_like)[0])
    print(f"High-excess ticks explained by a >1m seed-midpoint jump: "
          f"{len(overlap)} / {len(hi)}")
    if len(hi) > 0:
        print(f"  -> {100*len(overlap)/len(hi):.0f}% of high-excess ticks "
              f"coincide with a seed discontinuity")
    print()
    print("Ticks NOT explained by a seed jump point to a distinct mechanism")
    print("(e.g. a genuine sign-reversal corner transition, or something in")
    print("the chain-walk / blend step rather than the seed selection).")


if __name__ == "__main__":
    main()
