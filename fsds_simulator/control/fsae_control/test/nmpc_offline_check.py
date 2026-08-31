#!/usr/bin/env python3
"""
nmpc_offline_check.py — reproducible offline validation for nmpc_core's
NONLINEAR MPC (see late_turn_in_investigation.md Part 16).

Run it any time:

    python3 ros2/src/fsae_planning/control/fsae_control/test/nmpc_offline_check.py

No ROS session and no FSDS session are needed. Six checks, in the order they
are worth trusting:

  1. MODEL PARITY      — the scalar rollout fast path (_step_scalar, used for
                         the sequential horizon rollout) must agree with the
                         vectorised _step (used for the finite-difference
                         Jacobians) to machine precision. They are hand-mirrored
                         copies of one model, so a divergence here is a
                         silent-wrong-prediction bug.
  2. JACOBIANS         — the forward finite differences the SQP actually uses
                         vs central differences at a larger step: catches a
                         badly-sized perturbation or a non-smooth term.
  3. SQP CONVERGENCE   — cost must decrease monotonically to a plateau from a
                         cold start, at several operating points.
  4. TURN-IN           — the whole point. Car placed EXACTLY on the line
                         (e_y = e_psi = 0) approaching a known bend: the LTV-QP
                         must command 0.000 deg (its model cannot see the bend),
                         the NMPC must plan real steering.
  5. WRONG DIRECTION   — the failure mode that killed Parts 2/7/15. Checks the
                         converged input trajectory for a sustained
                         wrong-direction transient, and reports its magnitude
                         rather than just pass/fail.
  6. CLOSED LOOP       — SKIPPED unless a sibling fsae_MPCTest checkout is
                         present, since it uses that repo's 25-state Pacejka
                         plant (including the FSDS a_lat ceiling) and the real
                         comp_test_map_3 raceline. Runs the LTV-QP and the NMPC
                         through the SAME plant with the SAME weights and prints
                         tracking error, steering saturation, lap time, per-tick
                         solve time, and per-corner turn-in distance.

Exit status is 0 if every non-skipped check passes.
"""
import math
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)                     # .../fsae_control
sys.path.insert(0, _PKG)
# Repo root: .../ros2/src/fsae_planning/control/fsae_control -> up 5
_FSAE_PLANNING = os.path.dirname(os.path.dirname(_PKG))
_ROS2 = os.path.dirname(os.path.dirname(_FSAE_PLANNING))
_REPO = os.path.dirname(_ROS2)
_MPCTEST = os.path.join(_REPO, 'fsae_MPCTest')
TRACK_CSV = os.path.join(_FSAE_PLANNING, 'tracks', 'comp_test_map_3', 'raceline.csv')

from fsae_control.mpc import nmpc_core as nc                        # noqa: E402
from fsae_control.mpc.mpc_core import MAX_STEER_RAD, MPCController   # noqa: E402
from fsae_control.mpc.mpc_params import MPCParams                    # noqa: E402
from fsae_control.mpc.nmpc_core import NMPCController                # noqa: E402
from fsae_control.mpc.nmpc_params import NMPCParams                  # noqa: E402

FAILURES = []


def check(name, ok, detail=''):
    print(f'  [{"PASS" if ok else "FAIL"}] {name}' + (f' — {detail}' if detail else ''))
    if not ok:
        FAILURES.append(name)


def synthetic_corner(straight_m=60.0, radius=13.0, arc_deg=90.0, ds=0.25,
                     ramp_m=15.0, run_out_m=30.0):
    """Straight -> linear curvature ramp -> constant radius -> run-out."""
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


def make_nmpc(path, iters=1, N=None, **mpc_kw):
    npar = NMPCParams(nmpc_sqp_iters=iters, nmpc_solve_budget_ms=1e6,
                      **({'nmpc_horizon': N} if N else {}))
    ctrl = NMPCController(dt=0.05, params=MPCParams(**mpc_kw), nmpc=npar)
    ctrl.set_static_path(path)
    return ctrl


def converge(ctrl, x0, v_ref, iters=25):
    """Iterate the SQP to convergence from a cold start; return (U, costs)."""
    ref = ctrl._static_ref
    U = np.zeros((ctrl.N, nc.NU))
    X = ctrl._rollout(x0, U, ref)
    H = nc._outputs(X, ref, ctrl.plant, v_ref)
    costs = [ctrl._cost(X, U, H)]
    for _ in range(iters):
        dU, _status = ctrl._solve_step(X, U, ref, v_ref)
        if dU is None:
            break
        step, improved = 1.0, False
        for _bt in range(5):
            U_t = np.clip(U + step * dU, ctrl.u_min, ctrl.u_max)
            X_t = ctrl._rollout(x0, U_t, ref)
            H_t = nc._outputs(X_t, ref, ctrl.plant, v_ref)
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


# ── 1. model parity ──────────────────────────────────────────────────────
def test_model_parity():
    print('\n1. MODEL PARITY (scalar rollout vs vectorised Jacobian path)')
    ref = nc.PathReference(synthetic_corner())
    p = nc._Plant()
    rng = np.random.default_rng(7)
    worst = worst_k = 0.0
    for _ in range(300):
        x = np.array([rng.uniform(0, ref.total), rng.uniform(-2, 2),
                      rng.uniform(-0.6, 0.6), rng.uniform(0, 20),
                      rng.uniform(-1, 1), rng.uniform(-1, 1),
                      rng.uniform(-MAX_STEER_RAD, MAX_STEER_RAD),
                      rng.uniform(-7, 12)])
        u = np.array([rng.uniform(-MAX_STEER_RAD, MAX_STEER_RAD), rng.uniform(-7, 12)])
        worst_k = max(worst_k, abs(ref.kappa_scalar(x[0])
                                  - float(ref.kappa_at(np.array([x[0]]))[0])))
        for n_sub in (1, 2, 3):
            a = nc._step(x[None, :], u[None, :], ref, p, 0.05, n_sub)[0]
            b = np.array(nc._step_scalar(x, u, ref, p, 0.05, n_sub))
            worst = max(worst, float(np.max(np.abs(a - b) / np.maximum(np.abs(a), 1.0))))
    check('_step_scalar == _step', worst < 1e-12, f'worst relative diff {worst:.2e}')
    check('kappa_scalar == kappa_at', worst_k < 1e-12, f'worst abs diff {worst_k:.2e}')


# ── 2. Jacobians ─────────────────────────────────────────────────────────
def test_jacobians():
    print('\n2. JACOBIANS (forward FD as used by the SQP vs central FD)')
    ref = nc.PathReference(synthetic_corner())
    p = nc._Plant()
    rng = np.random.default_rng(1)
    M = 6
    X = np.zeros((M, nc.NX))
    X[:, nc.IDX_S] = np.linspace(30, 60, M)
    X[:, nc.IDX_EY] = rng.uniform(-0.5, 0.5, M)
    X[:, nc.IDX_EPSI] = rng.uniform(-0.1, 0.1, M)
    X[:, nc.IDX_VX] = np.linspace(2.0, 17.0, M)
    X[:, nc.IDX_VY] = rng.uniform(-0.3, 0.3, M)
    X[:, nc.IDX_R] = rng.uniform(-0.3, 0.3, M)
    X[:, nc.IDX_DELTA] = rng.uniform(-0.2, 0.2, M)
    X[:, nc.IDX_A] = rng.uniform(-3, 3, M)
    U = np.column_stack([rng.uniform(-0.3, 0.3, M), rng.uniform(-5, 5, M)])

    def step(Xa, Ua):
        return nc._step(Xa, Ua, ref, p, 0.05, 2)

    F0 = step(X, U)
    worst = 0.0
    for j in range(nc.NX):
        e = nc._FD_EPS_X[j]
        Xp = X.copy()
        Xp[:, j] += e
        fwd = (step(Xp, U) - F0) / e
        ec = 1e-5
        Xa, Xb = X.copy(), X.copy()
        Xa[:, j] += ec
        Xb[:, j] -= ec
        ctr = (step(Xa, U) - step(Xb, U)) / (2 * ec)
        worst = max(worst, np.abs(fwd - ctr).max() / max(np.abs(ctr).max(), 1e-6))
    for j in range(nc.NU):
        e = nc._FD_EPS_U[j]
        Up = U.copy()
        Up[:, j] += e
        fwd = (step(X, Up) - F0) / e
        ec = 1e-5
        Ua, Ub = U.copy(), U.copy()
        Ua[:, j] += ec
        Ub[:, j] -= ec
        ctr = (step(X, Ua) - step(X, Ub)) / (2 * ec)
        worst = max(worst, np.abs(fwd - ctr).max() / max(np.abs(ctr).max(), 1e-6))
    # 1e-3 is loose on purpose: the s-column differences the local slope of a
    # piecewise-linear kappa(s), so forward and central differences legitimately
    # disagree near a grid breakpoint.
    check('forward FD matches central FD', worst < 1e-3,
          f'worst relative discrepancy {worst:.2e}')


# ── 3. SQP convergence ───────────────────────────────────────────────────
def test_convergence():
    print('\n3. SQP CONVERGENCE (cost must decrease monotonically)')
    path = synthetic_corner()
    ctrl = make_nmpc(path)
    cases = [
        ('on-line straight, v=14', np.array([40.0, 0.0, 0.0, 14.0, 0, 0, 0, 0])),
        ('e_y=+1 m straight, v=14', np.array([40.0, 1.0, 0.0, 14.0, 0, 0, 0, 0])),
        ('corner approach s=55, v=14', np.array([55.0, 0.0, 0.0, 14.0, 0, 0, 0, 0])),
        ('mid-corner s=80, v=9', np.array([80.0, 0.0, 0.0, 9.0, 0, 0, 0, 0])),
    ]
    for label, x0 in cases:
        ctrl.reset()
        U, costs = converge(ctrl, x0, x0[nc.IDX_VX])
        mono = all(costs[i + 1] <= costs[i] + 1e-12 for i in range(len(costs) - 1))
        check(f'{label}', mono and len(costs) > 1,
              f'cost {costs[0]:.4g} -> {costs[-1]:.4g} in {len(costs) - 1} iters, '
              f'steer[0]={math.degrees(U[0, 0]):+.2f} deg')


# ── 4/5. turn-in and wrong-direction ─────────────────────────────────────
def test_turn_in():
    print('\n4/5. TURN-IN and WRONG-DIRECTION (car dead on-line, left bend ahead)')
    path = synthetic_corner(straight_m=60.0)
    arc = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(path, axis=0).T))])
    print(f'     {"dist to bend":>12s} {"v":>5s} | {"QP steer[0]":>11s} '
          f'{"NMPC steer[0]":>13s} {"NMPC horizon peak":>18s} {"worst wrong-dir":>16s}')
    qp_all_zero = True
    nmpc_plans = []      # (bend within horizon reach?, horizon peak steer)
    worst_wrong = 0.0
    for v in (8.0, 14.0):
        for d in (20.0, 12.0, 6.0, 2.0):
            s_car = 60.0 - d
            i = min(max(int(np.searchsorted(arc, s_car)), 1), len(path) - 2)
            seg = path[i + 1] - path[i]
            yaw = math.atan2(seg[1], seg[0])
            car_pos = path[i] - 0.70 * np.array([math.cos(yaw), math.sin(yaw)])

            qp = MPCController(dt=0.05, N=35)
            qp.compute(path, car_pos, yaw, v, v)
            qp_delta = qp.last_telemetry['delta_cmd']
            qp_all_zero &= abs(qp_delta) < 1e-9

            ctrl = make_nmpc(path)
            s0, e_y, e_psi, _idx, _yaw = ctrl._static_ref.project(
                car_pos + 0.70 * np.array([math.cos(yaw), math.sin(yaw)]), yaw)
            x0 = np.array([s0, e_y, e_psi, v, 0.0, 0.0, 0.0, 0.0])
            U, _c = converge(ctrl, x0, v)
            steer = U[:, 0]
            peak = steer.max()
            # A bend further away than v*N*dt is outside the prediction horizon
            # entirely, and planning zero steering for it is CORRECT, not a
            # failure — no controller can anticipate past its own horizon. Only
            # the in-reach cases are asserted on. (At the default N=20 / 1.0 s
            # this means ~8 m of reach at 8 m/s and ~14 m at 14 m/s, which is
            # why the 20 m rows read 0.00 deg.)
            reach = v * ctrl.N * ctrl.dt
            nmpc_plans.append((d <= reach, peak))
            # Wrong direction (bend is LEFT, so correct sign is positive):
            # the deepest negative excursion, and how many consecutive steps
            # it lasts. Parts 2/7/15's failure was ~7 consecutive steps of a
            # comparable magnitude to the eventual correct-direction command.
            neg = np.minimum(steer, 0.0)
            worst_wrong = min(worst_wrong, neg.min()) if neg.min() < worst_wrong else worst_wrong
            runs, cur = 0, 0
            for val in steer:
                cur = cur + 1 if val < -math.radians(0.5) else 0
                runs = max(runs, cur)
            print(f'     {d:12.1f} {v:5.1f} | {math.degrees(qp_delta):10.3f}d '
                  f'{math.degrees(U[0, 0]):12.3f}d {math.degrees(peak):17.2f}d '
                  f'{math.degrees(neg.min()):13.2f}d x{runs}')
    check('LTV-QP commands exactly 0 deg with e_y=e_psi=0 (the structural gap)',
          qp_all_zero, 'confirms the gap this controller exists to close')
    in_reach = [pk for within, pk in nmpc_plans if within]
    out_reach = [pk for within, pk in nmpc_plans if not within]
    check('NMPC plans real steering for every bend INSIDE its horizon',
          bool(in_reach) and min(in_reach) > math.radians(0.5),
          f'{len(in_reach)} in-reach cases, smallest horizon peak '
          f'{math.degrees(min(in_reach)):.2f} deg'
          + (f'; {len(out_reach)} case(s) beyond v*N*dt reach correctly plan '
             f'{math.degrees(max(out_reach)):.2f} deg' if out_reach else ''))
    check('no sustained wrong-direction transient',
          abs(math.degrees(worst_wrong)) < 2.0,
          f'deepest wrong-direction command {math.degrees(worst_wrong):.2f} deg '
          '(Parts 2/7/15 failed with multi-step excursions comparable to the '
          'correct-direction peak)')


# ── 6. closed loop ───────────────────────────────────────────────────────
def load_track():
    rows = []
    with open(TRACK_CSV) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or line[0].isalpha():
                continue
            rows.append([float(v) for v in line.split(',')])
    a = np.array(rows)
    return np.column_stack([a[:, 0], a[:, 1]]), (a[:, 4] if a.shape[1] >= 5 else a[:, 3])


def run_closed_loop(kind, vp, path, v_prof, n_steps=1600, dt=0.05, nmpc=None):
    params = MPCParams()
    if kind == 'qp':
        ctrl = MPCController(dt=dt, N=35, params=params)
    else:
        ctrl = NMPCController(dt=dt, params=params, nmpc=nmpc or NMPCParams())
        ctrl.set_static_path(path)
    p = vp.VehicleParams()
    seg = path[1] - path[0]
    st = vp.init_plant_state(path[0, 0], path[0, 1], math.atan2(seg[1], seg[0]),
                             vx0=float(v_prof[0]))
    arc = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(path, axis=0).T))])
    rec = {k: [] for k in ('e_y', 'e_psi', 'delta', 'ms', 's', 'sat')}
    last = 0
    for k in range(n_steps):
        pos = np.array([st[vp.IDX_X], st[vp.IDX_Y]])
        j = int(np.argmin(np.linalg.norm(path[last:last + 60] - pos, axis=1))) + last
        last = j
        t0 = time.perf_counter()
        ctrl.compute(path, pos, float(st[vp.IDX_PSI]), float(st[vp.IDX_VX]),
                     float(v_prof[j]), car_yaw_rate=float(st[vp.IDX_R]),
                     car_vy=float(st[vp.IDX_VY]))
        ms = (time.perf_counter() - t0) * 1e3
        tel = ctrl.last_telemetry
        st = vp.step_nonlinear_plant(st, [tel['delta_cmd'], tel['a_cmd']], dt, p)
        rec['e_y'].append(tel['e_y'])
        rec['e_psi'].append(tel['e_psi'])
        rec['delta'].append(tel['delta_cmd'])
        rec['ms'].append(ms)
        rec['s'].append(arc[j])
        rec['sat'].append(abs(tel['delta_cmd']) >= MAX_STEER_RAD - 1e-6)
        if abs(tel['e_y']) > 6.0:
            rec['offtrack'] = k * dt
            break
        if j >= len(path) - 3:
            rec['lap_s'] = k * dt
            break
    out = {k: (np.array(v) if isinstance(v, list) else v) for k, v in rec.items()}
    return out


def summarise(name, R):
    ey = np.abs(R['e_y'])
    ep = np.degrees(np.abs(R['e_psi']))
    line = (f'  {name:<20s} |e_y| mean {ey.mean():.3f} p90 {np.percentile(ey, 90):.3f} '
            f'max {ey.max():.3f} | |e_psi| mean {ep.mean():5.2f} p90 '
            f'{np.percentile(ep, 90):5.2f} | sat {100 * R["sat"].mean():5.1f}% | '
            f'solve mean {R["ms"].mean():5.1f} p95 {np.percentile(R["ms"], 95):5.1f} '
            f'max {R["ms"].max():5.1f} ms')
    if 'lap_s' in R:
        line += f' | LAP {R["lap_s"]:.1f} s'
    if 'offtrack' in R:
        line += f' | OFFTRACK at {R["offtrack"]:.1f} s'
    print(line)
    return R


def test_closed_loop():
    print('\n6. CLOSED LOOP (fsae_MPCTest Pacejka plant, real comp_test_map_3 raceline)')
    if not os.path.isdir(_MPCTEST) or not os.path.isfile(TRACK_CSV):
        print(f'  [SKIP] needs {_MPCTEST} and {TRACK_CSV}')
        return
    sys.path.insert(0, _MPCTEST)
    try:
        from model import vehicle_physics as vp
    except ImportError as exc:
        print(f'  [SKIP] could not import fsae_MPCTest model.vehicle_physics: {exc}')
        return
    path, v_prof = load_track()
    res = {}
    for kind, label, npar in (('qp', 'LTV-QP (shipped)', None),
                              ('nmpc', 'NMPC (defaults)', NMPCParams())):
        res[kind] = summarise(label, run_closed_loop(kind, vp, path, v_prof, nmpc=npar))
    check('NMPC completes the lap', 'lap_s' in res['nmpc'],
          'offtrack' not in res['nmpc'] and 'no offtrack excursion')
    check('NMPC saturates steering less than the LTV-QP',
          res['nmpc']['sat'].mean() <= res['qp']['sat'].mean(),
          f'{100 * res["nmpc"]["sat"].mean():.1f}% vs {100 * res["qp"]["sat"].mean():.1f}%')
    check('NMPC solve time fits the 50 ms control period',
          np.percentile(res['nmpc']['ms'], 95) < 25.0,
          f'p95 {np.percentile(res["nmpc"]["ms"], 95):.1f} ms')

    # Per-corner turn-in distance.
    ref = nc.PathReference(path)
    s_grid, kap = ref.s_kappa, ref.kappa
    mask = np.abs(kap) > 0.01          # = MPCParams.adaptive_q_lookahead_peak_hysteresis
    runs, i = [], 0
    while i < len(mask):
        if mask[i]:
            j = i
            while j + 1 < len(mask) and mask[j + 1]:
                j += 1
            if s_grid[j] - s_grid[i] > 5.0:
                runs.append((s_grid[i], s_grid[j]))
            i = j + 1
        else:
            i += 1
    print(f'  turn-in point per corner (arc length where |steer| first exceeds 25% of that '
          f'corner\'s peak, relative to the corner start; negative = before it):')
    deltas = []
    for ci, (s0, s1) in enumerate(runs):
        lo = max(runs[ci - 1][1] if ci else 0.0, s0 - 40.0)
        row = []
        for kind in ('qp', 'nmpc'):
            R = res[kind]
            win = (R['s'] >= lo) & (R['s'] <= s1)
            if not win.any():
                row.append(float('nan'))
                continue
            d = np.abs(R['delta'])
            idx = np.where(win & (d >= 0.25 * d[win].max()))[0]
            row.append(R['s'][idx[0]] - s0 if len(idx) else float('nan'))
        if not any(math.isnan(v) for v in row):
            deltas.append(row[0] - row[1])
            print(f'    corner at s={s0:6.1f} m: QP {row[0]:+6.1f} m, NMPC {row[1]:+6.1f} m '
                  f'-> NMPC earlier by {row[0] - row[1]:5.1f} m')
    if deltas:
        deltas = np.array(deltas)
        check('NMPC turns in earlier on every corner', bool((deltas >= 0).all()),
              f'mean {deltas.mean():.1f} m, median {np.median(deltas):.1f} m earlier '
              f'on {int((deltas > 0).sum())}/{len(deltas)}')


def main():
    print('nmpc_offline_check — see late_turn_in_investigation.md Part 16')
    test_model_parity()
    test_jacobians()
    test_convergence()
    test_turn_in()
    test_closed_loop()
    print('\n' + ('ALL CHECKS PASSED' if not FAILURES
                  else f'{len(FAILURES)} FAILED: ' + ', '.join(FAILURES)))
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())
