"""
Do several individually-small-effect factors combine into something real when
applied together, rather than one at a time?

Why this exists
----------------
gap_attribution_ledger tests every plant/ceiling factor ONE AT A TIME, and
each individual effect there is smaller than the suite's own run-to-run
std -- individually indistinguishable from noise. But that says nothing
about whether several small, independent, non-redundant effects combine
additively (or worse) when stacked, which single-factor A/B testing can
never reveal.

This combines three factors that are each already-modelled (not invented
here) and target DIFFERENT subsystems, so they are not redundant with each
other or with the eliminated mu/C_f tyre-fit approach (CLAUDE.md's standing
warning is specifically about imitating the ceiling's effect via tyre grip
-- none of these three do that):

  1. Ceiling level lowered to 6.5 (already measured individually in
     gap_attribution_ledger, with no DNF on the recorded map).
  2. SLAM_NOISE_ENABLED=True at its documented defaults -- models the real
     car's localisation error (FSDS itself has perfect pose; this is off by
     default specifically because it targets the REAL car, which is exactly
     what we're trying to explain here).
  3. CONE_NOISE_ENABLED=True at its documented defaults -- models real
     cone-detector position jitter (also off by default for the same
     "FSDS itself has none" reason).

None of these three individually moved the aggregate number much. This
script checks all 2^3 = 8 combinations (including each alone, for a direct
comparison against the single-factor point estimates) on both the recorded
map (live-comparable, n=1) and VALIDATION_SUITE (variance-comparable, n=5),
with DNF checked in every cell -- a combination that looks good on the
recorded map alone can still hide a DNF the suite would catch.

Usage
-----
    python3 -m tuner.combined_factors_sweep
"""
import os
os.environ.setdefault("MPLBACKEND", "Agg")

import itertools
import numpy as np

SAT_FRAC = 0.98


def make_params(ceiling_low, slam_on, cone_on):
    from model.vehicle_physics import VehicleParams
    p = VehicleParams()
    if ceiling_low:
        p.alat_ceiling = 6.5
    return p


def run_recorded_map(params, slam_on, cone_on):
    import settings as S
    from sim.rollout_core import run_core_rollout, compute_step_budget
    from tuner.recorded_map_rollout import DEFAULT_MAP
    from sim.track_io import load_recorded_track
    from tuner.offline_tuner import get_cached_model

    # SLAM_NOISE_ENABLED / CONE_NOISE_ENABLED are read via `from settings
    # import X` inside sim/rollout_core.py at import time, so patching the
    # settings module after import has no effect there -- must patch the
    # already-imported binding in sim.rollout_core directly (same
    # requirement documented for every other *_ENABLED flag in this repo).
    import sim.rollout_core as rc
    rc.SLAM_NOISE_ENABLED = slam_on
    rc.CONE_NOISE_ENABLED = cone_on

    path_X, path_Y, path_Psi, path_v, blue, yellow = load_recorded_track(DEFAULT_MAP)
    dyn_max, num_steps = compute_step_budget(path_X, path_Y, path_v)
    Q = np.diag(S.Q_diag)
    R = np.diag(S.R_diag)
    R_rate = np.diag(S.R_rate_diag)
    u_min = np.array([-params.max_steer, params.max_accel_brake])
    u_max = np.array([params.max_steer, params.max_accel])
    rollout = rc.run_core_rollout(
        path_X, path_Y, path_Psi, path_v, blue, yellow,
        Q, R, R_rate, u_min, u_max, params,
        max_steps=num_steps, dynamic_max_steps=dyn_max,
        use_planner=True, model_lookup=get_cached_model,
        n_horizon=S.N_HORIZON, eps=S.ROLLOUT_EPS, max_iter=S.ROLLOUT_MAX_ITER,
        want_history=True,
    )
    h = rollout["history"]
    steer = np.asarray(h.get("u_steer", []), float)
    sat = float(np.mean(np.abs(steer) > SAT_FRAC * params.max_steer)) * 100.0
    return dict(sat=sat, dnf=bool(h.get("failed", False)),
                offtrack=bool(h.get("offtrack", False)),
                fail_reason=h.get("fail_reason"))


def run_suite(params, slam_on, cone_on):
    import settings as S
    import sim.rollout_core as rc
    from tuner.offline_tuner import get_cached_model, SYNTHETIC_PATHS

    rc.SLAM_NOISE_ENABLED = slam_on
    rc.CONE_NOISE_ENABLED = cone_on

    sats, dnfs = [], []
    for name in S.VALIDATION_SUITE:
        path_X, path_Y, path_Psi, path_v, blue, yellow = SYNTHETIC_PATHS[name]
        dyn_max, num_steps = rc.compute_step_budget(path_X, path_Y, path_v)
        Q = np.diag(S.Q_diag)
        R = np.diag(S.R_diag)
        R_rate = np.diag(S.R_rate_diag)
        u_min = np.array([-params.max_steer, params.max_accel_brake])
        u_max = np.array([params.max_steer, params.max_accel])
        rollout = rc.run_core_rollout(
            path_X, path_Y, path_Psi, path_v, blue, yellow,
            Q, R, R_rate, u_min, u_max, params,
            max_steps=num_steps, dynamic_max_steps=dyn_max,
            use_planner=True, model_lookup=get_cached_model,
            n_horizon=S.N_HORIZON, eps=S.ROLLOUT_EPS, max_iter=S.ROLLOUT_MAX_ITER,
            want_history=True,
        )
        h = rollout["history"]
        steer = np.asarray(h.get("u_steer", []), float)
        sat = float(np.mean(np.abs(steer) > SAT_FRAC * params.max_steer)) * 100.0
        sats.append(sat)
        dnfs.append(bool(h.get("failed", False)) or bool(h.get("offtrack", False)))
    return dict(mean=float(np.mean(sats)), std=float(np.std(sats)),
                per_path=dict(zip(S.VALIDATION_SUITE, sats)),
                n_dnf=sum(dnfs), dnf_paths=[n for n, d in zip(S.VALIDATION_SUITE, dnfs) if d])


def main():
    import settings as S
    live_sat_range = "21.1 - 28.0"  # observed range of live steering-saturation %

    print(f"LIVE saturation observed so far: {live_sat_range}%\n")
    print(f"{'ceiling6.5':>11} {'slam':>5} {'cone':>5}   "
          f"{'rec.map':>8} {'dnf?':>5}   {'suite mean':>11} {'suite std':>10} {'suite dnf':>10}")

    results = []
    for ceiling_low, slam_on, cone_on in itertools.product([False, True], repeat=3):
        params = make_params(ceiling_low, slam_on, cone_on)
        rec = run_recorded_map(params, slam_on, cone_on)
        suite = run_suite(params, slam_on, cone_on)
        results.append((ceiling_low, slam_on, cone_on, rec, suite))
        dnf_flag = "YES" if (rec["dnf"] or rec["offtrack"]) else "no"
        print(f"{str(ceiling_low):>11} {str(slam_on):>5} {str(cone_on):>5}   "
              f"{rec['sat']:8.2f} {dnf_flag:>5}   "
              f"{suite['mean']:11.2f} {suite['std']:10.2f} "
              f"{suite['n_dnf']:>9d}/5")
        if rec["dnf"] or rec["offtrack"]:
            print(f"             -> recorded-map DNF: {rec['fail_reason']}")
        if suite["n_dnf"] > 0:
            print(f"             -> suite DNF on: {suite['dnf_paths']}")

    print()
    print("Baseline (all False) is the reference point. Compare each")
    print("combination's recorded-map sat% against it, and check the suite")
    print("std to judge whether any increase is distinguishable from noise")
    print("(individual single-factor effects were not distinguishable from")
    print("the suite std -- the same standard applies here).")


if __name__ == "__main__":
    main()
