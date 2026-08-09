"""
Does the BLENDED (non-bypassed) centreline itself swing faster than the car
can yaw, and does that correlate with how much the rebuild moved that tick?

Why this exists
----------------
tuner/blend_reset_diagnostics.py showed blend_paths()'s reset_dist=2.0m
bypass essentially never fires on the recorded map -- so the *bypass*
cannot explain the reference-heading swings observed (both stacks chase a
reference swinging faster than either car can yaw, which drives most of
the heading-error growth). That leaves two live possibilities for the
*blended* (alpha=0.4 EMA) path, which is what actually gets published on
nearly every recorded-map tick:

  1. alpha=0.4 blending is itself insufficient -- even a moderate,
     sub-threshold rebuild jump, blended at 40% weight per tick, could
     still produce a heading rate above what the car can yaw if it
     recurs on consecutive ticks (an EMA smooths magnitude, not rate).
  2. The reference-heading swing is NOT caused by rebuild-to-rebuild
     jumps at all -- it could be intrinsic to curvature the *car itself
     is following* (a tight corner has a fast-changing tangent purely
     from geometry, independent of any planner instability). This is
     the null result: if swing rate does NOT correlate with rebuild
     magnitude, the swing is a property of the track, not the planner's
     temporal instability, and this whole line of inquiry closes.

This script measures both by re-running the recorded-map rollout with the
same non-invasive blend_paths wrapper as blend_reset_diagnostics.py (still
does not alter behaviour), this time recording the per-tick rebuild
distance alongside the published reference heading (reconstructed from
history the same way tuner/live_vs_sim_diagnostics.py's reference_quality()
does: ref_psi = car_yaw - e_psi), then correlating d(ref_psi)/dt against
the rebuild distance at the same tick.

Usage
-----
    python3 -m tuner.reference_heading_vs_rebuild
"""
import os
import sys

os.environ.setdefault("MPLBACKEND", "Agg")

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

import planning.path_utils as path_utils  # noqa: E402
import sim.sim_track as sim_track  # noqa: E402

_orig_blend_paths = path_utils.blend_paths


def main():
    from model.vehicle_physics import VehicleParams
    from tuner.recorded_map_rollout import DEFAULT_MAP
    from sim.track_io import load_recorded_track
    from sim.rollout_core import compute_step_budget, run_core_rollout
    from settings import N_HORIZON, Q_diag, R_diag, R_rate_diag, ROLLOUT_EPS, ROLLOUT_MAX_ITER
    from tuner.offline_tuner import get_cached_model

    rebuild_dists = []  # one entry per blend_paths call, aligned to sim ticks

    def wrapped(prev, new, car_pos, alpha=0.4, ds=0.5, horizon=15.0, reset_dist=2.0):
        out = _orig_blend_paths(prev, new, car_pos, alpha=alpha, ds=ds,
                                 horizon=horizon, reset_dist=reset_dist)
        d = float("nan")
        if prev is not None and len(prev) >= 2 and len(new) >= 2:
            n = int(horizon / ds) + 1
            r_new = path_utils._resample_forward(new, car_pos, ds, n)
            r_prev = path_utils._resample_forward(prev, car_pos, ds, n)
            if r_new is not None and r_prev is not None:
                d = float(np.mean(np.linalg.norm(r_new - r_prev, axis=1)))
        rebuild_dists.append(d)
        return out

    path_utils.blend_paths = wrapped
    sim_track.blend_paths = wrapped
    try:
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
    finally:
        path_utils.blend_paths = _orig_blend_paths
        sim_track.blend_paths = _orig_blend_paths

    h = rollout["history"]
    psi = np.asarray(h["psi"], float)
    e_psi = np.asarray(h["e_psi"], float)
    dt = 0.05

    n = min(len(psi), len(rebuild_dists))
    psi, e_psi = psi[:n], e_psi[:n]
    rebuild_d = np.array(rebuild_dists[:n])

    ref_psi = np.unwrap(psi - e_psi)
    dref = np.gradient(ref_psi, dt)
    dref_deg = np.degrees(dref)

    ok = np.isfinite(rebuild_d) & np.isfinite(dref_deg)
    print(f"n ticks (with a previous path to rebuild against) = {ok.sum()} / {n}")
    print()
    print("=== Reference-heading rate on the recorded map (post-blend, "
          "what the controller actually tracks) ===")
    print(f"  |d(ref_psi)/dt|  mean {np.abs(dref_deg[ok]).mean():7.1f}  "
          f"p90 {np.percentile(np.abs(dref_deg[ok]), 90):7.1f}  "
          f"p99 {np.percentile(np.abs(dref_deg[ok]), 99):7.1f}  "
          f"max {np.abs(dref_deg[ok]).max():7.1f}  deg/s")
    print(f"  rebuild distance   mean {rebuild_d[ok].mean():7.3f}  "
          f"p90 {np.percentile(rebuild_d[ok], 90):7.3f}  "
          f"p99 {np.percentile(rebuild_d[ok], 99):7.3f}  "
          f"max {rebuild_d[ok].max():7.3f}  m")
    print()

    corr = np.corrcoef(rebuild_d[ok], np.abs(dref_deg[ok]))[0, 1]
    print(f"Correlation(rebuild distance, |d(ref_psi)/dt|) = {corr:.3f}")

    # Split into high-rebuild vs low-rebuild ticks, compare heading rate
    thresh = np.percentile(rebuild_d[ok], 90)
    hi = ok & (rebuild_d > thresh)
    lo = ok & (rebuild_d <= thresh)
    print(f"\nTop-10% rebuild-distance ticks (> {thresh:.3f} m), n={hi.sum()}:")
    print(f"  |d(ref_psi)/dt| mean {np.abs(dref_deg[hi]).mean():.1f} deg/s")
    print(f"Bottom-90% rebuild-distance ticks, n={lo.sum()}:")
    print(f"  |d(ref_psi)/dt| mean {np.abs(dref_deg[lo]).mean():.1f} deg/s")

    print()
    print("If the correlation is weak and the hi/lo means are close, the")
    print("reference-heading swing is a property of the TRACK the car is")
    print("following (geometry-driven), not of rebuild-to-rebuild planner")
    print("instability -- closing this line of inquiry. If hi >> lo, the")
    print("blended path IS carrying rebuild noise into the reference even")
    print("below the reset_dist bypass threshold.")


if __name__ == "__main__":
    main()
