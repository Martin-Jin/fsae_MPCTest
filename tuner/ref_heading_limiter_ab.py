"""
A/B test: does REF_HEADING_RATE_LIMIT (settings.py) close any of the
steering-saturation gap between sim and live?

Sweeps REF_HEADING_RISE_RATE against the disabled baseline on the recorded
map. Patches sim.rollout_core's already-imported module attributes directly
(same requirement as CONE_NOISE_ENABLED/SLAM_NOISE_ENABLED — patching the
settings module after import has no effect on names imported via
`from settings import X`).

Usage
-----
    python3 -m tuner.ref_heading_limiter_ab
"""
import os
os.environ.setdefault("MPLBACKEND", "Agg")

import numpy as np


def run_once(rate_deg_s):
    import sim.rollout_core as rc
    from model.vehicle_physics import VehicleParams
    from tuner.recorded_map_rollout import DEFAULT_MAP
    from sim.track_io import load_recorded_track
    from settings import N_HORIZON, Q_diag, R_diag, R_rate_diag, ROLLOUT_EPS, ROLLOUT_MAX_ITER
    from tuner.offline_tuner import get_cached_model

    rc.REF_HEADING_RATE_LIMIT_ENABLED = rate_deg_s is not None
    if rate_deg_s is not None:
        rc.REF_HEADING_RISE_RATE = rate_deg_s

    path_X, path_Y, path_Psi, path_v, blue, yellow = load_recorded_track(DEFAULT_MAP)
    dyn_max, num_steps = rc.compute_step_budget(path_X, path_Y, path_v)
    params = VehicleParams()
    Q = np.diag(Q_diag)
    R = np.diag(R_diag)
    R_rate = np.diag(R_rate_diag)
    u_min = np.array([-params.max_steer, params.max_accel_brake])
    u_max = np.array([params.max_steer, params.max_accel])
    rollout = rc.run_core_rollout(
        path_X, path_Y, path_Psi, path_v, blue, yellow,
        Q, R, R_rate, u_min, u_max, params,
        max_steps=num_steps, dynamic_max_steps=dyn_max,
        use_planner=True, model_lookup=get_cached_model,
        n_horizon=N_HORIZON, eps=ROLLOUT_EPS, max_iter=ROLLOUT_MAX_ITER,
        want_history=True,
    )
    h = rollout["history"]
    u_steer = np.asarray(h["u_steer"], float)
    e_psi = np.asarray(h["e_psi"], float)
    e_psi_true = np.asarray(h["e_psi_true"], float)
    max_steer = params.max_steer
    sat = np.abs(u_steer) > 0.98 * max_steer

    delta_cmd = np.diff(u_steer)
    reversals = np.sum(np.diff(np.sign(np.where(np.abs(delta_cmd) > 1e-4, delta_cmd, 0))) != 0)
    dt = 0.05
    duration_s = len(u_steer) * dt

    return {
        "rate": rate_deg_s,
        "sat_pct": 100.0 * sat.mean(),
        "e_psi_mean": np.degrees(np.abs(e_psi)).mean(),
        "e_psi_p90": np.percentile(np.degrees(np.abs(e_psi)), 90),
        "e_psi_true_mean": np.degrees(np.abs(e_psi_true)).mean(),
        "reversals_per_s": reversals / duration_s,
        "score": rollout.get("score"),
        "dnf": h.get("failed", False),
        "offtrack": h.get("offtrack", False),
        "fail_reason": h.get("fail_reason"),
        "steps": len(u_steer),
    }


def main():
    configs = [None, 120.0, 110.0, 100.0, 95.0, 90.0, 85.0, 80.0, 75.0, 70.0, 65.0, 60.0]
    print(f"{'rate(deg/s)':>12s} {'sat%':>7s} {'e_psi_mean':>11s} {'e_psi_p90':>10s} "
          f"{'e_psi_true_mean':>16s} {'rev/s':>7s} {'steps':>6s} {'dnf':>5s}")
    for rate in configs:
        r = run_once(rate)
        label = "OFF (baseline)" if rate is None else f"{rate:.0f}"
        print(f"{label:>12s} {r['sat_pct']:7.2f} {r['e_psi_mean']:11.2f} "
              f"{r['e_psi_p90']:10.2f} {r['e_psi_true_mean']:16.2f} "
              f"{r['reversals_per_s']:7.2f} {r['steps']:6d} {str(r['dnf']):>5s}")
        if r["dnf"] or r["offtrack"]:
            print(f"             -> DNF/offtrack: {r['fail_reason']}")


if __name__ == "__main__":
    main()
