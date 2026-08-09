"""
Does REF_HEADING_RATE_LIMIT's recorded-map improvement (tuner/ref_heading_limiter_ab.py)
hold across settings.VALIDATION_SUITE, or is it a one-map artifact?

Per the standing rig-validation lesson -- a plausible-looking improvement on
a single recorded map can hide a regression a wider sweep would catch --
any candidate improvement must be checked against the synthetic suite
before being trusted, not just the one recorded map.

Usage
-----
    python3 -m tuner.ref_heading_limiter_suite_check
"""
import os
os.environ.setdefault("MPLBACKEND", "Agg")

import numpy as np

SAT_FRAC = 0.98


def run_one(path_name, rate_deg_s):
    import sim.rollout_core as rc
    from model.vehicle_physics import VehicleParams
    from settings import N_HORIZON, Q_diag, R_diag, R_rate_diag, ROLLOUT_EPS, ROLLOUT_MAX_ITER
    from tuner.offline_tuner import get_cached_model, SYNTHETIC_PATHS

    rc.REF_HEADING_RATE_LIMIT_ENABLED = rate_deg_s is not None
    if rate_deg_s is not None:
        rc.REF_HEADING_RISE_RATE = rate_deg_s

    path_X, path_Y, path_Psi, path_v, blue, yellow = SYNTHETIC_PATHS[path_name]
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
    steer = np.asarray(h.get("u_steer", []), float)
    if steer.size == 0:
        return dict(sat=float("nan"), dnf=bool(h.get("failed", False)),
                    offtrack=bool(h.get("offtrack", False)))
    sat = float(np.mean(np.abs(steer) > SAT_FRAC * params.max_steer)) * 100.0
    return dict(sat=sat, dnf=bool(h.get("failed", False)),
                offtrack=bool(h.get("offtrack", False)),
                fail_reason=h.get("fail_reason"))


def main():
    from settings import VALIDATION_SUITE

    configs = [("OFF (baseline)", None), ("70 deg/s", 70.0), ("65 deg/s", 65.0)]

    results = {label: [] for label, _ in configs}
    for label, rate in configs:
        print(f"=== {label} ===")
        for name in VALIDATION_SUITE:
            r = run_one(name, rate)
            results[label].append(r["sat"])
            flag = ""
            if r["dnf"] or r["offtrack"]:
                flag = f"  <-- DNF/offtrack: {r.get('fail_reason')}"
            print(f"  {name:<20} sat={r['sat']:6.2f}%{flag}")
        arr = np.array(results[label])
        print(f"  {'MEAN':<20} sat={arr.mean():6.2f}%  std={arr.std():6.2f}%")
        print()

    print("=== Per-path diff vs baseline ===")
    base = np.array(results["OFF (baseline)"])
    for label, _ in configs[1:]:
        d = np.array(results[label]) - base
        print(f"  {label:<12} " +
              " ".join(f"{n.replace('PATH_', ''):<14}={v:+6.1f}"
                       for n, v in zip(VALIDATION_SUITE, d)) +
              f"   mean {d.mean():+6.2f}")


if __name__ == "__main__":
    main()
