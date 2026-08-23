#!/usr/bin/env python3
"""
tuner/steering_chatter_check.py — reproduce and measure the NMPC steering
chatter symptom documented in docs/logs/steering_chatter_investigation.md.

Runs a closed-loop NMPC (or LTV-QP, for comparison) rollout on
comp_test_map_3 with a static precomputed path (use_planner=False, no SLAM
noise / pose-hold / delay-jitter), and reports tick-to-tick steering
chatter metrics: std of the per-tick delta, mean|delta|, and the
sign-flip rate of that delta.

Any settings.py constant can be overridden for one run via --set NAME=VALUE
(repeatable), e.g. to reproduce the investigation doc's sweeps:

    python -m tuner.steering_chatter_check --set Q_diag=[6.0,0.8,1.65,1.20,5.40,0.0,0.0,0.0]
    python -m tuner.steering_chatter_check --set NMPC_SQP_ITERS=2
    python -m tuner.steering_chatter_check --controller ltv

Overrides are applied to the `settings` module BEFORE `sim.rollout_core` is
imported, which is required for any constant that module imports by bare
name (USE_NMPC, NMPC_HORIZON, NMPC_SQP_ITERS, NMPC_SOLVE_BUDGET_MS,
NMPC_TRUST_DELTA_RAD, NMPC_JAC_SUBSTEPS, TERMINAL_Q_SCALE, and others) --
see docs/logs/steering_chatter_investigation.md's "Reproduction script"
section for why this ordering matters and CLAUDE.md's `fsae_MPCTest`
import-time-binding note. Run this script directly (a fresh process each
time), not by importing it into a longer-lived process, or later overrides
in the same process will not take effect on already-imported names.

Run: python -m tuner.steering_chatter_check [--controller nmpc|ltv]
                                             [--set NAME=VALUE ...]
"""
import argparse
import ast
import sys

import numpy as np


def _parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--controller', choices=['nmpc', 'ltv'], default='nmpc')
    p.add_argument('--set', action='append', default=[], metavar='NAME=VALUE',
                    help='override a settings.py constant before import; '
                         'repeatable. VALUE is parsed with ast.literal_eval, '
                         'so lists/floats/bools all work, e.g. '
                         '--set NMPC_HORIZON=25 --set "R_rate_diag=[5.0,2.25]"')
    return p.parse_args()


def main():
    args = _parse_args()

    import settings
    for spec in args.set:
        name, _, value = spec.partition('=')
        if not _:
            raise SystemExit(f'--set expects NAME=VALUE, got: {spec!r}')
        setattr(settings, name, ast.literal_eval(value))

    # Imported AFTER settings overrides are applied -- rollout_core.py pulls
    # several NMPC_* constants in by bare name at its own import time.
    from model.vehicle_physics import VehicleParams
    from model.bicycle_model import get_8state_discrete_model
    from sim.rollout_core import compute_step_budget, run_core_rollout
    from sim.track_io import load_recorded_track
    from tracks import cone_map_path

    def model_lookup(vx, dt):
        return get_8state_discrete_model(vx, dt)

    path_X, path_Y, path_Psi, path_v, blue, yellow = load_recorded_track(cone_map_path())
    dyn_max, num_steps = compute_step_budget(path_X, path_Y, path_v)
    vp = VehicleParams()
    Q = np.diag(settings.Q_diag)
    R = np.diag(settings.R_diag)
    R_rate = np.diag(settings.R_rate_diag)
    u_min = np.array([-vp.max_steer, vp.max_accel_brake])
    u_max = np.array([vp.max_steer, vp.max_accel])
    use_nmpc = args.controller == 'nmpc'

    r = run_core_rollout(
        path_X, path_Y, path_Psi, path_v, blue, yellow,
        Q, R, R_rate, u_min, u_max, vp,
        max_steps=num_steps, dynamic_max_steps=dyn_max,
        use_planner=False, model_lookup=model_lookup,
        want_history=True, use_nmpc=use_nmpc,
    )
    h = r['history']
    steer_deg = np.degrees(np.asarray(h['u_steer'], float))
    d = np.diff(steer_deg)
    sign_flips = int(np.sum(np.diff(np.sign(d)) != 0))
    e_y = np.asarray(h['e_y'], float)

    print(f"controller={args.controller} overrides={args.set or '(none)'}")
    print(f"  n_steps={len(steer_deg)}")
    print(f"  steer d(tick) std={np.std(d):.4f} deg, mean|d|={np.mean(np.abs(d)):.4f} deg")
    print(f"  sign-flip rate: {100 * sign_flips / len(d):.1f}% ({sign_flips}/{len(d)} ticks)")
    print(f"  |e_y| mean={np.mean(np.abs(e_y)):.4f} p90={np.percentile(np.abs(e_y), 90):.4f}")
    print(f"  score={r['composite_score']:.4f} dnf={r['dnf']} offtrack={r['offtrack']}")
    if 'nmpc_status' in h:
        statuses = h['nmpc_status']
        n_bad = sum(1 for s in statuses if s not in ('solved', None))
        if n_bad:
            print(f"  WARNING: {n_bad}/{len(statuses)} ticks had non-'solved' "
                  f"nmpc_status (rejected / max-iterations / budget) -- chatter "
                  f"metrics on a run with solver failures are not comparable "
                  f"to a clean run; see docs/logs/steering_chatter_investigation.md's "
                  f"NMPC_HORIZON=35 caveat before trusting a low-chatter result "
                  f"that also has solver failures.")


if __name__ == '__main__':
    main()
