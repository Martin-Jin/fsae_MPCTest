#!/usr/bin/env python3
"""
tuner/nmpc_offline_check.py — offline validation for controller/nmpc_optimiser.py,
mirroring the live repo's ros2/.../test/nmpc_offline_check.py structure so the
two can be read/compared side by side (see docs/tuning.md's NMPC section).

Run: python -m tuner.nmpc_offline_check
"""
import math
import sys

import numpy as np

from controller import nmpc_optimiser as no
from model.vehicle_physics import VehicleParams
from controller.optimiser import solve_mpc
from model.bicycle_model import get_8state_discrete_model

FAILURES = []


def check(name, ok, detail=''):
    print(f'  [{"PASS" if ok else "FAIL"}] {name}' + (f' — {detail}' if detail else ''))
    if not ok:
        FAILURES.append(name)


def synthetic_corner(straight_m=60.0, radius=13.0, arc_deg=90.0, ds=0.25,
                     ramp_m=15.0, run_out_m=30.0):
    pts = [np.zeros(2)]
    psi = 0.0
    for _ in range(int(straight_m / ds)):
        pts.append(pts[-1] + ds * np.array([math.cos(psi), math.sin(psi)]))
    k_max = 1.0 / radius
    n_ramp = int(ramp_m / ds)
    for i in range(n_ramp):
        psi += k_max * (i + 1) / n_ramp * ds
        pts.append(pts[-1] + ds * np.array([math.cos(psi), math.sin(psi)]))
    for _ in range(int(math.radians(arc_deg) / (k_max * ds))):
        psi += k_max * ds
        pts.append(pts[-1] + ds * np.array([math.cos(psi), math.sin(psi)]))
    for _ in range(int(run_out_m / ds)):
        pts.append(pts[-1] + ds * np.array([math.cos(psi), math.sin(psi)]))
    return np.array(pts)


def make_ctrl(N=20, sqp_iters=1, budget_ms=1e6, vp=None):
    vp = vp or VehicleParams()
    u_min = np.array([-vp.max_steer, vp.max_accel_brake])
    u_max = np.array([vp.max_steer, vp.max_accel])
    du_max = np.array([vp.max_steer_rate * 0.05, 0.6])
    return no.NMPCController(
        dt=0.05, N=N, vehicle_params=vp, u_min=u_min, u_max=u_max, du_max=du_max,
        q_e_y=6.0, q_e_yd=0.8, q_e_psi=1.6, q_epsi_dot=1.0, q_e_v=5.55,
        r_delta=1.8, r_a_accel=3.0, r_a_brake=0.5, r_rate_delta=2.5, r_rate_a=2.4,
        sqp_iters=sqp_iters, solve_budget_ms=budget_ms,
    )


def converge(ctrl, x0, v_ref, iters=25):
    ref = ctrl._ref
    U = np.zeros((ctrl.N, no.NU))
    X = ctrl._rollout(x0, U, ref)
    H = no._outputs(X, ref, ctrl.plant, v_ref)
    costs = [ctrl._cost(X, U, H)]
    for _ in range(iters):
        dU, _status = ctrl._solve_step(X, U, ref, v_ref)
        if dU is None:
            break
        step, improved = 1.0, False
        for _bt in range(5):
            U_t = np.clip(U + step * dU, ctrl.u_min, ctrl.u_max)
            X_t = ctrl._rollout(x0, U_t, ref)
            H_t = no._outputs(X_t, ref, ctrl.plant, v_ref)
            c_t = ctrl._cost(X_t, U_t, H_t)
            if c_t <= costs[-1]:
                U, X, H = U_t, X_t, H_t
                costs.append(c_t)
                improved = True
                break
            step *= 0.5
        if not improved:
            break
    return U, costs


def test_model_parity():
    print('\n1. MODEL PARITY (scalar rollout vs vectorised Jacobian path)')
    ref = no.PathReference(synthetic_corner())
    vp = VehicleParams()
    p = no._Plant(vp)
    rng = np.random.default_rng(7)
    worst = worst_k = 0.0
    for _ in range(300):
        x = np.array([rng.uniform(0, ref.total), rng.uniform(-2, 2),
                      rng.uniform(-0.6, 0.6), rng.uniform(0, 20),
                      rng.uniform(-1, 1), rng.uniform(-1, 1),
                      rng.uniform(-vp.max_steer, vp.max_steer),
                      rng.uniform(vp.max_accel_brake, vp.max_accel)])
        u = np.array([rng.uniform(-vp.max_steer, vp.max_steer),
                      rng.uniform(vp.max_accel_brake, vp.max_accel)])
        worst_k = max(worst_k, abs(ref.kappa_scalar(x[0])
                                  - float(ref.kappa_at(np.array([x[0]]))[0])))
        for n_sub in (1, 2, 3):
            a = no._step(x[None, :], u[None, :], ref, p, 0.05, n_sub)[0]
            b = np.array(no._step_scalar(x, u, ref, p, 0.05, n_sub))
            worst = max(worst, float(np.max(np.abs(a - b) / np.maximum(np.abs(a), 1.0))))
    check('_step_scalar == _step', worst < 1e-12, f'worst relative diff {worst:.2e}')
    check('kappa_scalar == kappa_at', worst_k < 1e-12, f'worst abs diff {worst_k:.2e}')


def test_convergence():
    print('\n2. SQP CONVERGENCE (cost must decrease monotonically)')
    path = synthetic_corner()
    ctrl = make_ctrl()
    ctrl.path_reference(path)
    cases = [
        ('on-line straight, v=14', np.array([40.0, 0.0, 0.0, 14.0, 0, 0, 0, 0])),
        ('e_y=+1 m straight, v=14', np.array([40.0, 1.0, 0.0, 14.0, 0, 0, 0, 0])),
        ('corner approach s=55, v=14', np.array([55.0, 0.0, 0.0, 14.0, 0, 0, 0, 0])),
        ('mid-corner s=80, v=9', np.array([80.0, 0.0, 0.0, 9.0, 0, 0, 0, 0])),
    ]
    for label, x0 in cases:
        ctrl.reset()
        U, costs = converge(ctrl, x0, x0[no.IDX_VX])
        mono = all(costs[i + 1] <= costs[i] + 1e-12 for i in range(len(costs) - 1))
        check(f'{label}', mono and len(costs) > 1,
              f'cost {costs[0]:.4g} -> {costs[-1]:.4g} in {len(costs) - 1} iters, '
              f'steer[0]={math.degrees(U[0, 0]):+.2f} deg')


def test_turn_in():
    print('\n3. TURN-IN and WRONG-DIRECTION (linear LTV-QP vs the NMPC, dead on-line, left bend ahead)')
    path = synthetic_corner(straight_m=60.0)
    arc = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(path, axis=0).T))])
    vp = VehicleParams()
    u_min = np.array([-vp.max_steer, vp.max_accel_brake])
    u_max = np.array([vp.max_steer, vp.max_accel])
    du_max = np.array([vp.max_steer_rate * 0.05, 0.6])
    Q = np.diag([6.0, 0.8, 1.6, 0.70, 5.55, 0.0, 0.0, 0.0])
    R = np.diag([1.8, 0.77])

    ltv_all_zero = True
    nmpc_plans = []
    worst_wrong = 0.0
    print(f'     {"dist to bend":>12s} {"v":>5s} | {"LTV steer[0]":>12s} '
          f'{"NMPC steer[0]":>13s} {"NMPC horizon peak":>18s}')
    for v in (8.0, 14.0):
        for d in (20.0, 12.0, 6.0, 2.0):
            s_car = 60.0 - d
            i = min(max(int(np.searchsorted(arc, s_car)), 1), len(path) - 2)
            seg = path[i + 1] - path[i]
            yaw = math.atan2(seg[1], seg[0])
            car_pos = path[i] - vp.lf * np.array([math.cos(yaw), math.sin(yaw)])

            Ad, Bd = get_8state_discrete_model(v, 0.05)
            x0_ltv = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            res = solve_mpc(x0_ltv, Ad, Bd, 35, Q, R, u_min, u_max,
                            du_max=du_max, silent=True, return_status=True)
            ltv_delta = res[0][0] if res is not None else float('nan')
            ltv_all_zero &= abs(ltv_delta) < 1e-9

            ctrl = make_ctrl()
            s0, e_y, e_psi, _idx, _yaw = ctrl.path_reference(path).project(
                car_pos + vp.lf * np.array([math.cos(yaw), math.sin(yaw)]), yaw)
            x0 = np.array([s0, e_y, e_psi, v, 0.0, 0.0, 0.0, 0.0])
            U, _c = converge(ctrl, x0, v)
            steer = U[:, 0]
            peak = steer.max()
            reach = v * ctrl.N * ctrl.dt
            nmpc_plans.append((d <= reach, peak))
            neg = np.minimum(steer, 0.0)
            worst_wrong = min(worst_wrong, neg.min())
            print(f'     {d:12.1f} {v:5.1f} | {math.degrees(ltv_delta):11.3f}d '
                  f'{math.degrees(U[0, 0]):12.3f}d {math.degrees(peak):17.2f}d')
    check('LTV-QP commands exactly 0 deg with e_y=e_psi=0 (the structural gap)',
          ltv_all_zero, 'confirms the gap this controller exists to close')
    in_reach = [pk for within, pk in nmpc_plans if within]
    check('NMPC plans real steering for every bend INSIDE its horizon',
          bool(in_reach) and min(in_reach) > math.radians(0.5),
          f'{len(in_reach)} in-reach cases, smallest horizon peak '
          f'{math.degrees(min(in_reach)):.2f} deg')
    check('no sustained wrong-direction transient',
          abs(math.degrees(worst_wrong)) < 2.0,
          f'deepest wrong-direction command {math.degrees(worst_wrong):.2f} deg')


def test_closed_loop():
    print('\n4. CLOSED LOOP (recorded comp_test_map_3, LTV-QP vs NMPC, identical weights)')
    try:
        from tracks import cone_map_path
        from sim.track_io import load_recorded_track
    except Exception as exc:
        print(f'  [SKIP] could not load the recorded track: {exc}')
        return
    from sim.rollout_core import run_core_rollout, compute_step_budget
    import settings as S  # noqa: F401 (Q_diag/R_diag/etc. still read from here)

    path_X, path_Y, path_Psi, path_v, blue, yellow = load_recorded_track(cone_map_path())
    dyn_max, num_steps = compute_step_budget(path_X, path_Y, path_v)
    vp = VehicleParams()
    Q = np.diag(S.Q_diag)
    R = np.diag(S.R_diag)
    R_rate = np.diag(S.R_rate_diag)
    u_min = np.array([-vp.max_steer, vp.max_accel_brake])
    u_max = np.array([vp.max_steer, vp.max_accel])

    def model_lookup(vx, dt):
        return get_8state_discrete_model(vx, dt)

    results = {}
    for use_nmpc, label in ((False, 'LTV-QP (as shipped)'), (True, 'NMPC (settings.py defaults)')):
        # Pass use_nmpc as an explicit CALL argument, not by mutating
        # settings.USE_NMPC -- rollout_core.py imports USE_NMPC as a bare
        # name (`from settings import USE_NMPC`), so reassigning the
        # settings MODULE's attribute afterward has no effect on that
        # already-bound name. This is exactly why run_core_rollout() takes
        # use_nmpc as its own parameter (mirroring use_planner=USE_PLANNER),
        # not something to rely on toggling via settings.py at runtime.
        r = run_core_rollout(
            path_X, path_Y, path_Psi, path_v, blue, yellow,
            Q, R, R_rate, u_min, u_max, vp,
            max_steps=num_steps, dynamic_max_steps=dyn_max,
            use_planner=False, model_lookup=model_lookup,
            want_history=True, use_nmpc=use_nmpc,
        )
        results[label] = r
        h = r['history']
        ey = np.abs(np.array(h['e_y_true']))
        ep = np.degrees(np.abs(np.array(h['e_psi_true'])))
        sat = np.mean(np.abs(np.array(h['u_steer'])) >= u_max[0] - 1e-6)
        print(f'  {label:<28s} |e_y| mean {ey.mean():.3f} p90 {np.percentile(ey, 90):.3f} | '
              f'|e_psi| mean {ep.mean():5.2f} | sat {100*sat:5.1f}% | '
              f'n_ran {len(h["X"])} | reached_end={r["reached_end"]} dnf={r["dnf"]}')
    check('NMPC completes without DNF', not results['NMPC (settings.py defaults)']['dnf'])
    ltv_h = results['LTV-QP (as shipped)']['history']
    nmpc_h = results['NMPC (settings.py defaults)']['history']
    identical = (np.array(ltv_h['u_steer']) == np.array(nmpc_h['u_steer'])).all() if len(
        ltv_h['u_steer']) == len(nmpc_h['u_steer']) else False
    check('NMPC and LTV-QP produce genuinely DIFFERENT commands (sanity: '
          'catches the use_nmpc-not-actually-toggled class of bug)',
          not identical, 'confirms use_nmpc reached the controller, not just the label')


def main():
    print('nmpc_offline_check (offline sim) — see controller/nmpc_optimiser.py')
    test_model_parity()
    test_convergence()
    test_turn_in()
    test_closed_loop()
    print('\n' + ('ALL CHECKS PASSED' if not FAILURES
                  else f'{len(FAILURES)} FAILED: ' + ', '.join(FAILURES)))
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
