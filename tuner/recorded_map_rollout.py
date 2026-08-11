"""
Run a closed-loop rollout on the RECORDED map, headless, and print the metrics
the sim-to-real comparison tables are quoted in.

Why this exists
---------------
The `comp test map 3` baselines in docs/ (steering saturation, |e_psi|, a_lat
max) were produced ad hoc and were not reproducible from the repo — the
recorded track could only be loaded through the GUI's button. That made the
single most important number in the whole investigation impossible to re-check
after a plant change. This script makes it one command.

Usage
-----
    python3 -m tuner.recorded_map_rollout
    python3 -m tuner.recorded_map_rollout --planner
    python3 -m tuner.recorded_map_rollout --mode pi --gain 450 --ceiling 6.6
    python3 -m tuner.recorded_map_rollout --no-ceiling

Default is the oracle/precomputed path, matching settings.USE_PLANNER=False
— no planner/perception in the loop, and speed
comes from the recorded track's own oracle profile. Pass --planner to run
the planner-in-loop rollout instead (live-built centreline, perception
mistakes included).

Compare the output against the live car, measured on the same recorded map
with the same tuned gains, PLANNER-IN-LOOP (--planner) — the default
oracle-path run is not directly comparable to this table, since the live car
always runs its own online planner:

    steering saturation  21.1 %
    |e_psi| mean / p90   15.9 / 42.0 deg
    a_lat max            12.34 m/s2
    a_lat > 7.5           9.8 %
    reversals/s           1.62
"""
import argparse
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from model.vehicle_physics import VehicleParams  # noqa: E402
from settings import (  # noqa: E402
    N_HORIZON, Q_diag, R_diag, R_rate_diag, ROLLOUT_EPS, ROLLOUT_MAX_ITER,
)
from sim.rollout_core import compute_step_budget, run_core_rollout  # noqa: E402
from sim.track_io import load_recorded_track  # noqa: E402
from tuner.offline_tuner import get_cached_model  # noqa: E402

from tracks import cone_map_path, resolve_map_arg  # noqa: E402

# The map every baseline in docs/ is quoted on, kept at tracks/comp_test_map_3/
# so the numbers stay directly comparable across runs. Eight other tuner
# scripts import this constant rather than re-deriving the path -- repointing
# it here moves all of them together, which is the reason it stays a module
# constant.
DEFAULT_MAP = cone_map_path()

LIVE = {
    "steer_sat_pct": 21.1, "e_psi_mean": 15.9, "e_psi_p90": 42.0,
    "alat_max": 12.34, "alat_over_pct": 9.8, "reversals_per_s": 1.62,
}


def run(map_path, params, use_planner=False, continue_after_dnf=False):
    path_X, path_Y, path_Psi, path_v, blue, yellow = load_recorded_track(map_path)
    dyn_max, num_steps = compute_step_budget(path_X, path_Y, path_v)

    Q = np.diag(Q_diag)
    R = np.diag(R_diag)
    R_rate = np.diag(R_rate_diag)
    u_min = np.array([-params.max_steer, params.max_accel_brake])
    u_max = np.array([params.max_steer, params.max_accel])

    return run_core_rollout(
        path_X, path_Y, path_Psi, path_v, blue, yellow,
        Q, R, R_rate, u_min, u_max, params,
        max_steps=num_steps, dynamic_max_steps=dyn_max,
        use_planner=use_planner, model_lookup=get_cached_model,
        n_horizon=N_HORIZON, eps=ROLLOUT_EPS, max_iter=ROLLOUT_MAX_ITER,
        want_history=True, continue_after_dnf=continue_after_dnf,
    )


def summarise(rollout, params, dt=0.05):
    h = rollout["history"]
    m = rollout["metrics_result"]
    v = np.asarray(h["v"], float)
    r = np.asarray(h["r"], float)
    alat = np.abs(v * r)
    # "e_psi" is the CONTROLLER's view (error against the planner's centreline)
    # and is what the live telemetry logs, so it is the live-comparable column.
    # "e_psi_true" is against the global oracle path, which the car never sees.
    e_psi = np.degrees(np.abs(np.asarray(h["e_psi"], float)))
    e_psi_true = np.degrees(np.abs(np.asarray(h["e_psi_true"], float)))

    steer = np.asarray(h.get("u_steer", []), float)
    if steer.size:
        sat = float(np.mean(np.abs(steer) > 0.95 * params.max_steer)) * 100.0
        sign = np.sign(steer)
        nz = sign[sign != 0]
        reversals = float(np.sum(np.diff(nz) != 0)) / max(len(steer) * dt, 1e-9)
    else:
        sat = float(m.get("steering_sat_ratio", float("nan"))) * 100.0
        reversals = float("nan")

    return {
        "steer_sat_pct": sat,
        "e_psi_mean": float(e_psi.mean()),
        "e_psi_p90": float(np.percentile(e_psi, 90)),
        "e_psi_true_mean": float(e_psi_true.mean()),
        "e_psi_true_p90": float(np.percentile(e_psi_true, 90)),
        "alat_max": float(alat.max()),
        # Per-tick effective ceiling, not the flat floor -- alat_ceiling is
        # now speed-dependent above ~10.7 m/s (see VehicleParams.alat_ceiling_at).
        "alat_over_pct": float(np.mean(
            alat > np.array([params.alat_ceiling_at(vi) for vi in v]))) * 100.0,
        "reversals_per_s": reversals,
        "score": float(rollout["composite_score"]),
        "progress": float(rollout["progress"]),
        "dnf": bool(rollout["dnf"]),
        "offtrack": bool(rollout["offtrack"]),
        "n_steps": len(v),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--map", default=None,
                    help="Track name under tracks/, or an explicit cone_map.json "
                         "path (default: the comp_test_map_3 baseline map).")
    ap.add_argument("--mode", choices=("p", "pi"))
    ap.add_argument("--gain", type=float)
    ap.add_argument("--tau", type=float)
    ap.add_argument("--ceiling", type=float)
    ap.add_argument("--no-ceiling", action="store_true")
    ap.add_argument("--planner", action="store_true",
                    help="run the planner-in-loop instead of tracking the "
                         "precomputed/oracle path (default: oracle path, "
                         "matching settings.USE_PLANNER=False)")
    ap.add_argument("--continue-after-dnf", action="store_true",
                    help="keep stepping past a DNF trigger (off-track/stall) "
                         "instead of stopping, to see whether/how the car "
                         "recovers over the rest of the map")
    args = ap.parse_args()

    p = VehicleParams()
    if args.no_ceiling:
        p.alat_ceiling_enabled = False
    for attr, val in (("alat_ceiling_mode", args.mode),
                      ("alat_ceiling_gain", args.gain),
                      ("alat_ceiling_tau", args.tau),
                      ("alat_ceiling", args.ceiling)):
        if val is not None:
            setattr(p, attr, val)

    map_path = resolve_map_arg(args.map)
    # Name the containing directory, not the file: under tracks/ every map is
    # called cone_map.json, so the basename alone identifies nothing.
    print(f"map      : {os.path.basename(os.path.dirname(map_path))}")
    print(f"ceiling  : enabled={p.alat_ceiling_enabled} "
          f"mode={p.alat_ceiling_mode} level={p.alat_ceiling} "
          f"gain={p.alat_ceiling_gain} tau={p.alat_ceiling_tau}")
    print(f"planner  : {'planner-in-loop' if args.planner else 'oracle path'}\n")

    s = summarise(run(map_path, p, use_planner=args.planner,
                      continue_after_dnf=args.continue_after_dnf), p)

    print(f"{'metric':<22}{'sim':>10}{'live':>10}{'sim/live':>10}")
    print("-" * 52)
    for key, label in (("steer_sat_pct", "steering sat %"),
                       ("e_psi_mean", "|e_psi| mean (deg)"),
                       ("e_psi_p90", "|e_psi| p90 (deg)"),
                       ("alat_max", "a_lat max"),
                       ("alat_over_pct", "a_lat > ceiling %"),
                       ("reversals_per_s", "reversals/s")):
        sv, lv = s[key], LIVE[key]
        ratio = sv / lv if lv else float("nan")
        print(f"{label:<22}{sv:10.2f}{lv:10.2f}{ratio:10.2f}")

    print(f"\n(vs oracle path, not live-comparable: |e_psi_true| mean "
          f"{s['e_psi_true_mean']:.2f} / p90 {s['e_psi_true_p90']:.2f} deg)")
    print(f"\nscore {s['score']:.3f}   progress {s['progress']:.3f}   "
          f"steps {s['n_steps']}   dnf={s['dnf']}   offtrack={s['offtrack']}")


if __name__ == "__main__":
    main()
