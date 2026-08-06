"""
Quantify how much of the sim-to-real SATURATION gap each tested factor closes,
with real run-to-run variance instead of a single point estimate.

Why this exists
----------------
Every factor tested so far (ceiling law, alat_ceiling_tau, ceiling LEVEL,
fsds_bridge a_cmd) was reported as a single before/after number on the one
recorded map. That is a point estimate with no error bar: "6.3% -> 6.7%" could
be signal or could be map-specific noise, and there was no way to tell.

The recorded map itself cannot supply that variance — every rollout on it is
fully deterministic by design (SLAM noise, delay jitter and pose-hold all use
FIXED seeds, see settings.SLAM_NOISE_SEED / DELAY_JITTER_SEED / POSE_HOLD_SEED,
so CMA-ES sees a repeatable score per candidate). Re-running the same map
twice would report exactly zero variance, which would be a false confidence
signal, not a real one.

Instead this uses settings.VALIDATION_SUITE — 5 synthetic paths with
different geometry (spiral, sudden turn, hairpin, FS corner, micro-slalom) —
as the repeat axis. That measures whether a factor's effect on saturation is
consistent across track geometry or an artifact of one map's specific corners.
It does NOT reproduce the live car (these paths have no live-log counterpart);
the recorded map stays the only live-comparable number. This is purely for
attributing how much each factor moves the SIM, with a real error bar.

Usage
-----
    python3 -m tuner.gap_attribution_ledger
"""
import os
import sys

# tuner.offline_tuner imports `cma`, which optionally imports matplotlib.pyplot
# inside a try/except -- but that import chain has been observed to crash the
# whole process outright (not just fail the try/except) depending on whatever
# backend matplotlib picks by default in this environment. Force a headless
# backend before anything downstream can touch matplotlib.
os.environ.setdefault("MPLBACKEND", "Agg")

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from model.vehicle_physics import VehicleParams  # noqa: E402
from settings import (  # noqa: E402
    N_HORIZON, Q_diag, R_diag, R_rate_diag, ROLLOUT_EPS, ROLLOUT_MAX_ITER,
    VALIDATION_SUITE,
)
from sim.rollout_core import compute_step_budget, run_core_rollout  # noqa: E402
from tuner.offline_tuner import SYNTHETIC_PATHS, get_cached_model  # noqa: E402
from tuner.recorded_map_rollout import (  # noqa: E402
    DEFAULT_MAP, LIVE, run as run_recorded_map, summarise as summarise_recorded,
)

SAT_FRAC = 0.95


def run_synthetic(path_name, params, use_planner=True):
    path_X, path_Y, path_Psi, path_v, blue, yellow = SYNTHETIC_PATHS[path_name]
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
        want_history=True,
    )


def summarise(rollout, params, dt=0.05):
    h = rollout["history"]
    steer = np.asarray(h.get("u_steer", []), float)
    if steer.size == 0:
        return dict(steer_sat_pct=float("nan"), dnf=bool(rollout["dnf"]),
                    offtrack=bool(rollout["offtrack"]),
                    progress=float(rollout["progress"]))
    sat = float(np.mean(np.abs(steer) > SAT_FRAC * params.max_steer)) * 100.0
    return dict(steer_sat_pct=sat, dnf=bool(rollout["dnf"]),
                offtrack=bool(rollout["offtrack"]),
                progress=float(rollout["progress"]))


def suite_saturation(params_fn, label):
    """Run every VALIDATION_SUITE path with params_fn() and report mean/std."""
    sats, dnfs, offs = [], [], []
    for name in VALIDATION_SUITE:
        p = params_fn()
        s = summarise(run_synthetic(name, p), p)
        sats.append(s["steer_sat_pct"])
        dnfs.append(s["dnf"])
        offs.append(s["offtrack"])
    sats = np.array(sats)
    print(f"  {label:<32} per-path: " +
          " ".join(f"{n.replace('PATH_', ''):<14}={v:5.1f}%"
                   for n, v in zip(VALIDATION_SUITE, sats)))
    print(f"  {'':32} mean {sats.mean():5.2f}%  std {sats.std():5.2f}%  "
          f"dnf {sum(dnfs)}/{len(dnfs)}  offtrack {sum(offs)}/{len(offs)}")
    return sats


def recorded_map_saturation(params_fn):
    p = params_fn()
    s = summarise_recorded(run_recorded_map(DEFAULT_MAP, p), p)
    return s["steer_sat_pct"]


def baseline_params():
    return VehicleParams()


def no_ceiling_params():
    p = VehicleParams()
    p.alat_ceiling_enabled = False
    return p


def proportional_law_params():
    p = VehicleParams()
    p.alat_ceiling_mode = 'p'
    p.alat_ceiling_gain = 700.0
    return p


def integral_law_old_tau_params():
    p = VehicleParams()
    p.alat_ceiling_mode = 'pi'
    p.alat_ceiling_gain = 450.0
    p.alat_ceiling_tau = 0.25
    return p


def shipped_params():
    return VehicleParams()  # current defaults: pi, gain=450, tau=0.40


def lower_ceiling_65_params():
    p = VehicleParams()
    p.alat_ceiling = 6.5
    return p


def main():
    live_sat = LIVE["steer_sat_pct"]
    print(f"LIVE saturation (recorded map): {live_sat:.2f}%\n")

    factors = [
        ("A. no ceiling at all (pre-2026-08-06 baseline)", no_ceiling_params),
        ("B. ceiling, PROPORTIONAL law (gain=700, tau=0.25)", proportional_law_params),
        ("C. ceiling, INTEGRAL law, OLD tau=0.25 (gain=450)", integral_law_old_tau_params),
        ("D. ceiling, INTEGRAL law, MEASURED tau=0.40 (shipped)", shipped_params),
        ("E. ceiling LOWERED to 6.5 (capability probe)", lower_ceiling_65_params),
    ]

    print("=== Recorded map (single run, live-comparable; this is what all")
    print("    prior write-ups quoted) ===")
    rec = {}
    for label, fn in factors:
        v = recorded_map_saturation(fn)
        rec[label] = v
        print(f"  {label:<52} sat={v:6.2f}%  (vs live {live_sat:.1f}%)")

    print()
    print("=== VALIDATION_SUITE (5 synthetic paths; measures whether the")
    print("    effect is consistent across geometry, or a one-map artifact.")
    print("    No live comparison exists for these paths.) ===\n")
    suite = {}
    for label, fn in factors:
        suite[label] = suite_saturation(fn, label)
        print()

    print("=== Paired per-path diffs vs factor A (no ceiling) ===")
    print("Unpaired mean+/-std can hide that an effect is concentrated on a")
    print("subset of geometries rather than uniform across the suite.")
    a_sats = suite[factors[0][0]]
    for label, _ in factors[1:]:
        d = suite[label] - a_sats
        print(f"  {label:<52} " +
              " ".join(f"{n.replace('PATH_', ''):<14}={v:+5.1f}"
                       for n, v in zip(VALIDATION_SUITE, d)) +
              f"   mean {d.mean():+5.2f}  std {d.std():5.2f}")
    print()

    print("=== Attribution: recorded-map delta vs suite mean+/-std ===")
    print(f"{'factor':<52}{'rec.map':>9}{'suite mean':>12}{'suite std':>11}")
    baseline_rec = rec[factors[0][0]]
    for label, _ in factors:
        d_rec = rec[label] - baseline_rec
        print(f"{label:<52}{rec[label]:9.2f}{suite[label].mean():12.2f}"
              f"{suite[label].std():11.2f}")

    gap = live_sat - baseline_rec
    print(f"\nGap to close (live - no-ceiling baseline, MEASURED TODAY "
          f"with current code): {gap:.2f} pp")
    print("NOTE: the no-ceiling baseline here (measured with this session's")
    print("code, including the curvature_speed parity fix) does NOT match")
    print("the historical '4.4%' figure quoted in earlier doc revisions --")
    print("that number predates recorded_map_rollout.py and multiple since-")
    print("fixed bugs, so it is not the same quantity and must not be mixed")
    print("with the numbers below.")
    print(f"{'step':<52}{'closes (pp)':>12}{'closes (%)':>11}{'suite std (pp)':>16}")
    prev = baseline_rec
    for label, _ in factors[1:]:
        step = rec[label] - prev
        pct = 100.0 * step / gap if gap else float("nan")
        print(f"{label:<52}{step:12.2f}{pct:11.1f}{suite[label].std():16.2f}")
        prev = rec[label]
    remaining = live_sat - prev
    print(f"\n{'REMAINING UNEXPLAINED':<52}{remaining:12.2f}"
          f"{100.0*remaining/gap:11.1f}")


if __name__ == "__main__":
    main()
