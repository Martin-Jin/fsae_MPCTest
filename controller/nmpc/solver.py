"""
controller/nmpc/solver.py — `NMPCController`: QP construction, Gauss-Newton
SQP iteration via OSQP, and the per-tick `compute_step` entry point.
"""

import math

import numpy as np
import scipy.sparse as sp

from controller.model_utils import (
    steer_rate_anti_hunt, reversal_penalty_boost, _corner_factor, _blend,
)
from controller.nmpc.layout import (
    IDX_S, IDX_EY, IDX_EPSI, IDX_VX, NX, NU, NH_TRACKING, NH_PROGRESS,
    _FD_EPS_X, _FD_EPS_U,
)
from controller.nmpc.reference import PathReference
from controller.nmpc.dynamics import _Plant, _step_scalar, _step
from controller.nmpc.outputs import _outputs
from controller.nmpc.weight_schedule import _rrate_zone_scale, _rrate_stage_ramp

try:
    import osqp
except ImportError as _exc:      # pragma: no cover - see README's dependency list
    osqp = None
    _OSQP_IMPORT_ERROR = _exc


def _csc_pattern(mask):
    """Fixed-sparsity-pattern CSC matrix + (row,col) index arrays for
    writing into its .data in CSC order (OSQP requires a stable pattern
    across update() calls)."""
    m = sp.csc_matrix(mask.astype(np.float64))
    rows = m.indices.copy()
    cols = np.zeros_like(rows)
    for j in range(m.shape[1]):
        cols[m.indptr[j]:m.indptr[j + 1]] = j
    return m, rows, cols


class NMPCController:
    """
    Offline counterpart of the live side's `nmpc_core.NMPCController`. See
    this module's own docstring for the relationship between the two, and
    the live module's docstring for the full model/solver explanation
    (identical here) — not repeated per-method here to avoid the two
    docstrings drifting apart in wording while the code itself is kept in
    sync by hand.
    """

    def __init__(
        self, dt, N, vehicle_params,
        u_min, u_max, du_max,
        q_e_y, q_e_yd, q_e_psi, q_epsi_dot, q_e_v,
        r_delta, r_a_accel, r_a_brake, r_rate_delta, r_rate_a,
        terminal_scale=1.0,
        sqp_iters=1, solve_budget_ms=25.0,
        # Both 4: the (v_y, r) lateral dynamics stiffen as 1/v_x and anything
        # lower is RK4-unstable at low speed, freezing the solver at exactly
        # zero output. See docs/logs/nmpc_low_speed_accel_stall_investigation.md
        rk_substeps=4, jac_substeps=4,
        # Speed gates: both the Jacobian's sensitivity AND the rollout itself
        # are RK4-unstable only in a narrow low-speed band, not the whole 0.1-25
        # m/s envelope, so the full substep count is only needed there. See
        # _jacobians'/_rollout's own docstrings for the measured envelopes
        # (different fast values: the rollout's instability is confined to a
        # narrower band than the Jacobian's, and 2 substeps is the one count
        # confirmed unstable there, so its fast value is 3, not 2).
        jac_gate_speed=8.0, jac_substeps_fast=2,
        rk_gate_speed=4.0, rk_substeps_fast=3,
        # Standstill steering damping (default OFF): at v_x=0 steering cannot
        # move the car, but the SQP's cost is summed over the whole horizon
        # and the predicted v_x leaves zero by stage 1, so without this the
        # optimiser pre-commits U[0] toward what helps later stages and the
        # car launches already turned. See _r_delta_stage0.
        standstill_steer_damp_enabled=False, standstill_speed=0.5,
        standstill_fade_speed=3.0, standstill_steer_r_scale=20.0,
        trust_delta_rad=math.radians(9.0), trust_a=0.6, backtrack_max=2,
        track_halfwidth=3.5, slack_weight=10000.0, slack_linear_weight=0.0,
        osqp_max_iter=500, osqp_eps=1e-4,
        alat_ceiling_enabled=True,
        alat_flat=7.5, alat_slope=0.47, alat_intercept=2.46,
        spline_reference_enabled=True,
        friction_circle_enabled=False,
        steer_rate_anti_hunt_enabled=False,
        corner_rrate_blend_enabled=False,
        corner_factor_k=8.0,
        rrate_steer_straight=2.0,
        rrate_steer_corner=1.25,
        reversal_penalty_enabled=False,
        reversal_penalty_boost_max=4.0,
        reversal_penalty_k=8.0,
        rrate_stage_ramp_enabled=False,
        rrate_stage_near=0.15,
        rrate_zone_enabled=False,
        rrate_zone_boost_straight=2.0,
        rrate_zone_ease_approach=0.35,
        rrate_zone_floor_corner=0.15,
        rjerk_delta=0.0,
        rjerk_a=0.0,
        latency_compensation_enabled=False,
        latency_compensation_ms=25.0,
        kappa_rate_max=2.0,
        # Progress term (EXPERIMENTAL, default off) -- see
        # docs/logs/nmpc_progress_term_investigation.md. When enabled, row 4
        # of h() switches from the two-sided v_x-v_ref residual to a
        # one-sided speed-CAP hinge (q_e_v below then weights that hinge,
        # not a symmetric target-tracking error), and a 6th row rewards
        # progress toward an unreachable arc-length target -- see _outputs'
        # docstring for why both are written as least-squares residuals
        # rather than the textbook two-sided/linear forms.
        progress_enabled=False,
        q_progress=0.0,
        progress_reach=1.5,
        progress_v_min=0.5,
    ):
        if osqp is None:      # pragma: no cover - dependency guard
            raise ImportError(
                f'controller.nmpc requires osqp (already a dependency of '
                f'controller/lmpc/solve.py via cvxpy): {_OSQP_IMPORT_ERROR!r}'
            )
        self.dt = float(dt)
        self.N = int(N)
        if self.N < 2:
            raise ValueError(f'NMPC horizon must be >= 2 (got {self.N})')

        self.plant = _Plant(
            vehicle_params, alat_ceiling_enabled=alat_ceiling_enabled,
            alat_flat=alat_flat, alat_slope=alat_slope, alat_intercept=alat_intercept,
        )
        self.lf, self.lr = self.plant.lf, self.plant.lr

        # ── Experimental feature flags (see settings.py's NMPC_* comments) ──
        self.spline_reference_enabled = bool(spline_reference_enabled)
        self.friction_circle_enabled = bool(friction_circle_enabled)
        self.latency_compensation_enabled = bool(latency_compensation_enabled)
        self.latency_compensation_ms = float(latency_compensation_ms)
        self.kappa_rate_max = float(kappa_rate_max)
        # See NH_TRACKING/NH_PROGRESS and _outputs' docstring. Fixed for the
        # controller's lifetime (read once here), never toggled per-tick --
        # w_out's length and _build_qp's fixed sparsity both depend on it.
        self.progress_enabled = bool(progress_enabled)
        self.NH = NH_PROGRESS if self.progress_enabled else NH_TRACKING
        self.q_progress = float(q_progress)
        self.progress_reach = float(progress_reach)
        self.progress_v_min = float(progress_v_min)
        # EXPERIMENTAL, unvalidated for the NMPC -- see settings.py's
        # NMPC_STEER_RATE_ANTI_HUNT_ENABLED comment. Independent of any
        # LTV-QP-side anti-hunt flag.
        self.steer_rate_anti_hunt_enabled = bool(steer_rate_anti_hunt_enabled)
        # Alternative to the above, not a composition with it -- see
        # settings.py's NMPC_CORNER_RRATE_BLEND_ENABLED comment. Takes
        # priority over steer_rate_anti_hunt_enabled if both are set.
        self.corner_rrate_blend_enabled = bool(corner_rrate_blend_enabled)
        self.corner_factor_k = float(corner_factor_k)
        self.rrate_steer_straight = float(rrate_steer_straight)
        self.rrate_steer_corner = float(rrate_steer_corner)
        # EXPERIMENTAL, default off -- see settings.py's
        # NMPC_REVERSAL_PENALTY_ENABLED comment. Unlike the two flags above,
        # this one COMPOSES with either of them (it is keyed on u_prev, a
        # different signal from curvature/e_y/e_psi), so it is not an
        # alternative to them; see compute_step()'s rrate_steer_current.
        self.reversal_penalty_enabled = bool(reversal_penalty_enabled)
        self.reversal_penalty_boost_max = float(reversal_penalty_boost_max)
        self.reversal_penalty_k = float(reversal_penalty_k)
        # EXPERIMENTAL, default off. Discounts the steering-rate cost at the
        # NEAR horizon stages so a first turn-in input is cheap while a
        # sustained oscillation still pays full price -- see
        # _rrate_stage_ramp's docstring. Composes with all three flags above:
        # they set the rate weight's MAGNITUDE, this shapes it across STAGES.
        self.rrate_stage_ramp_enabled = bool(rrate_stage_ramp_enabled)
        self.rrate_stage_near = float(rrate_stage_near)
        # EXPERIMENTAL, default off. Continuous three-zone schedule on the
        # steering-rate cost: boost on a true straight, ease on the approach
        # to a corner the HORIZON can see, floor through the corner itself.
        # See _rrate_zone_scale. Unlike the corner blend it uses the
        # horizon's predicted curvature as well as the current value, so the
        # ease can lead turn-in rather than arriving with it.
        self.rrate_zone_enabled = bool(rrate_zone_enabled)
        self.rrate_zone_boost_straight = float(rrate_zone_boost_straight)
        self.rrate_zone_ease_approach = float(rrate_zone_ease_approach)
        self.rrate_zone_floor_corner = float(rrate_zone_floor_corner)
        # Steering/accel JERK weights (second difference of the input). 0.0
        # (default) disables the term entirely -- _E2rE2 is left None and
        # nothing is added to the Hessian, so the flag-off path is exactly
        # the pre-feature QP. See _build_qp's _E2 comment.
        self.rjerk_delta = float(rjerk_delta)
        self.rjerk_a = float(rjerk_a)
        if self.friction_circle_enabled:
            # F_max = m * ceiling(v_x) / 2 per axle: the measured ceiling law
            # bounds TOTAL lateral force (F_yf*cos(d) + F_yr) / m, split
            # evenly across the two axles as a simple, symmetric per-axle
            # cap (the soft mechanism in _f/_f_scalar scales both axles by
            # the SAME factor too, so this keeps the same even-split
            # convention rather than inventing a front/rear bias). This is a
            # HARD, ADDITIONAL bound alongside (not instead of) that
            # existing soft tanh saturation.
            self._fmax_flat = 0.5 * self.plant.m * alat_flat
            self._fmax_slope = 0.5 * self.plant.m * alat_slope
            self._fmax_intercept = 0.5 * self.plant.m * alat_intercept

        self.u_min = np.asarray(u_min, dtype=float)
        self.u_max = np.asarray(u_max, dtype=float)
        self.du_max = np.asarray(du_max, dtype=float)

        # q_e_v weights the speed-CAP hinge in progress mode (row 4, see
        # _outputs), not a two-sided target-tracking error -- same slot,
        # different regressor, expect it to need its own value rather than
        # inheriting the tracking-mode tuned q_e_v unchanged.
        w_out = [q_e_y, q_e_yd, q_e_psi, q_epsi_dot, q_e_v]
        if self.progress_enabled:
            w_out.append(self.q_progress)
        self.w_out = np.array(w_out, dtype=float)
        self.r_delta = float(r_delta)
        self.r_a_accel = float(r_a_accel)
        self.r_a_brake = float(r_a_brake)
        self.r_rate = np.array([r_rate_delta, r_rate_a], dtype=float)
        self.terminal_scale = float(terminal_scale)

        self.sqp_iters = int(sqp_iters)
        self.solve_budget_ms = float(solve_budget_ms)
        self.rk_substeps = int(rk_substeps)
        self.jac_substeps = max(1, int(jac_substeps))
        self.jac_gate_speed = float(jac_gate_speed)
        self.jac_substeps_fast = max(1, int(jac_substeps_fast))
        self.rk_gate_speed = float(rk_gate_speed)
        self.rk_substeps_fast = max(1, int(rk_substeps_fast))
        self.standstill_steer_damp_enabled = bool(standstill_steer_damp_enabled)
        self.standstill_speed = float(standstill_speed)
        self.standstill_fade_speed = float(standstill_fade_speed)
        self.standstill_steer_r_scale = float(standstill_steer_r_scale)
        self.trust_delta_rad = float(trust_delta_rad)
        self.trust_a = float(trust_a)
        self.backtrack_max = int(backtrack_max)
        self.track_halfwidth = float(track_halfwidth)
        self.slack_weight = float(slack_weight)
        # Linear (exact-penalty) component of the soft track-boundary cost,
        # ADDITIONAL to the existing quadratic slack_weight term -- see
        # _solve_step's q[idx] line. 0.0 (default) is a pure no-op: a purely
        # quadratic penalty has ZERO gradient at zero violation, so small
        # violations are nearly free, which matters once a progress reward
        # (progress_enabled) gives the solver an unbounded incentive to find
        # them. Liniger's MPCC reference implementation carries both terms
        # (sc_quad_track AND sc_lin_track) for exactly this reason.
        self.slack_linear_weight = float(slack_linear_weight)
        self.osqp_max_iter = int(osqp_max_iter)
        self.osqp_eps = float(osqp_eps)

        self._delta_act = 0.0
        self._a_act = 0.0
        self._u_prev = np.zeros(NU)
        self._u_prev2 = np.zeros(NU)  # command two ticks ago, for the jerk anchor
        self._U = np.zeros((self.N, NU))
        self._have_warm_start = False

        self._ref = None
        self._ref_signature = None

        self._qp = None
        self._build_qp()

    # ------------------------------------------------------------------
    def path_reference(self, path, dense_step=0.5, smooth_w=3, kappa_clip=0.5,
                       spline_reference_enabled=True):
        """
        Return the PathReference for `path`, rebuilding only when the path's
        signature (endpoints/length) has changed since the last call — so a
        fixed oracle path (USE_PLANNER=False) costs this once, while a
        planner-built centreline (USE_PLANNER=True, which changes every
        tick) is rebuilt each time it actually changes. Mirrors the live
        module's per-tick caching exactly.
        """
        path = np.asarray(path, dtype=float)
        sig = (
            len(path),
            float(path[0, 0]), float(path[0, 1]),
            float(path[-1, 0]), float(path[-1, 1]),
        )
        if self._ref is not None and self._ref_signature == sig:
            return self._ref
        prev = self._ref
        self._ref = PathReference(
            path, dense_step=dense_step, smooth_w=smooth_w, kappa_clip=kappa_clip,
            spline_reference_enabled=spline_reference_enabled,
        )
        self._ref_signature = sig
        self._rate_limit_kappa(self._ref, prev)
        return self._ref

    def _rate_limit_kappa(self, ref, prev):
        """
        Cap kappa(s)'s tick-to-tick change against the last rebuild's
        profile. Mirrors the live nmpc_core.py's own _rate_limit_kappa --
        see that method's docstring for the mechanism and
        NMPCParams.nmpc_kappa_rate_max's for why. Mutates ref.kappa/
        ref._k_list in place; must run before anything reads them.
        """
        if self.kappa_rate_max <= 0.0 or prev is None:
            return
        max_step = self.kappa_rate_max * self.dt
        prev_on_grid = np.interp(ref.s_kappa, prev.s_kappa, prev.kappa)
        ref.kappa = np.clip(ref.kappa, prev_on_grid - max_step, prev_on_grid + max_step)
        ref._k_list = [float(v) for v in np.atleast_1d(ref.kappa)]

    def reset(self):
        self._delta_act = 0.0
        self._a_act = 0.0
        self._u_prev = np.zeros(NU)
        self._u_prev2 = np.zeros(NU)  # command two ticks ago, for the jerk anchor
        self._U = np.zeros((self.N, NU))
        self._have_warm_start = False

    # ------------------------------------------------------------------
    def _build_qp(self):
        """Allocate the condensed QP once with fixed sparsity — see the live
        nmpc_core.py's _build_qp for the constraint-row layout (box/slew/
        soft-track-boundary rows); identical here.

        Friction-circle rows (self.friction_circle_enabled, see
        NMPCParams.nmpc_friction_circle_enabled): one two-sided
        (-F_max <= ... <= F_max) row per axle per stage = 2 axles * N
        stages, dense in dU (same reasoning as the soft-track rows above —
        stage k's tyre force depends on every earlier input through S).
        Read ONCE here, at construction time, like _use_slack — NOT
        per-tick — since it changes the QP's fixed sparsity pattern. When
        False, n_rows/nz and every array below are IDENTICAL to before this
        feature existed."""
        N = self.N
        n_du = NU * N
        self._use_slack = self.track_halfwidth > 0.0
        n_slack = N if self._use_slack else 0
        nz = n_du + n_slack
        n_fric = 2 * N if self.friction_circle_enabled else 0

        E = np.zeros((n_du, n_du))
        for k in range(N):
            E[k * NU:(k + 1) * NU, k * NU:(k + 1) * NU] = np.eye(NU)
            if k > 0:
                E[k * NU:(k + 1) * NU, (k - 1) * NU:k * NU] = -np.eye(NU)
        self._E = E
        self._Rr_flat = np.tile(self.r_rate, N)
        self._ErE = E.T @ (self._Rr_flat[:, None] * E)
        # Second-difference operator for the steering-JERK penalty
        # (rjerk_delta > 0). E2 = E @ E: applying the first-difference
        # operator twice gives du_k - du_{k-1}, i.e. steering ACCELERATION.
        #
        # WHY this term exists: the plain rate cost charges by |du|, which is
        # the same for a sustained ramp into a corner as for one leg of an
        # oscillation, so it cannot suppress hunting without also resisting
        # turn-in. Measured on live data, direction REVERSALS carry ~4.3x the
        # |d2| of same-direction ramps versus only ~1.9x the |d1|, so |d2|
        # separates the two roughly twice as sharply. A steady ramp scores
        # near zero here and is nearly free; an alternating wiggle is
        # expensive. See docs/steering_turn_in_upgrade_options.md (Option 4).
        #
        # No OSQP sparsity change: p_mask[:n_du,:n_du] is already a dense
        # upper triangle, so E2'RE2 adds no new nonzeros to the pattern.
        self._E2 = E @ E
        rj = np.tile(np.array([self.rjerk_delta, self.rjerk_a]), N)
        self._E2rE2 = (self._E2.T @ (rj[:, None] * self._E2)
                       if (self.rjerk_delta or self.rjerk_a) else None)

        p_mask = np.zeros((nz, nz), dtype=bool)
        p_mask[:n_du, :n_du] = np.triu(np.ones((n_du, n_du), dtype=bool))
        if n_slack:
            idx = np.arange(n_du, n_du + n_slack)
            p_mask[idx, idx] = True
        P, p_rows, p_cols = _csc_pattern(p_mask)

        n_rows = 2 * n_du + (3 * N if self._use_slack else 0) + n_fric
        a_mask = np.zeros((n_rows, nz), dtype=bool)
        a_mask[:n_du, :n_du] = np.eye(n_du, dtype=bool)
        a_mask[n_du:2 * n_du, :n_du] = E != 0.0
        if self._use_slack:
            r0 = 2 * n_du
            a_mask[r0:r0 + 2 * N, :n_du] = True
            for k in range(N):
                a_mask[r0 + k, n_du + k] = True
                a_mask[r0 + N + k, n_du + k] = True
                a_mask[r0 + 2 * N + k, n_du + k] = True
        if n_fric:
            rf0 = 2 * n_du + (3 * N if self._use_slack else 0)
            a_mask[rf0:rf0 + n_fric, :n_du] = True
        A, a_rows, a_cols = _csc_pattern(a_mask)

        q = np.zeros(nz)
        l = np.full(n_rows, -np.inf)
        u = np.full(n_rows, np.inf)

        prob = osqp.OSQP()
        settings = dict(
            verbose=False, eps_abs=self.osqp_eps, eps_rel=self.osqp_eps,
            max_iter=self.osqp_max_iter,
        )
        try:
            prob.setup(P, q, A, l, u, warm_starting=True, polishing=False, **settings)
        except TypeError:      # pragma: no cover - osqp < 1.0 naming
            prob.setup(P, q, A, l, u, warm_start=True, polish=False, **settings)

        self._qp = dict(
            prob=prob, P=P, A=A,
            p_rows=p_rows, p_cols=p_cols,
            a_rows=a_rows, a_cols=a_cols,
            n_du=n_du, n_slack=n_slack, n_fric=n_fric,
            nz=nz, n_rows=n_rows,
        )

    # ------------------------------------------------------------------
    def _rollout(self, x0, U, ref):
        """Roll the nonlinear model forward from the measured state under
        the current input guess, scalar fast path — see the live
        nmpc_core.py's _rollout for why this makes the QP's dynamics defect
        exactly zero (the linearisation point is always feasible).

        rk_substeps is SPEED-GATED per stage, same technique as
        nmpc_jac_substeps/nmpc_jac_gate_speed (see _jacobians). Measured
        directly (infinitesimal perturbation propagated through this same
        rollout, not just the sensitivity Jacobian): 2 substeps diverges
        (up to ~260x growth) across roughly 2.25-3.75 m/s, but 3 is fully
        converged everywhere tested in that band and above -- the fast
        value here is 3, not 2, specifically because 2 is the one count
        confirmed unstable. Gated per-stage on that stage's OWN predicted
        v_x (unlike the Jacobian's single whole-horizon gate), since this
        function builds X incrementally and a stage's speed can cross the
        gate mid-horizon on a hard launch/brake."""
        N = self.N
        X = np.empty((N + 1, NX))
        X[0] = x0
        xk = [float(v) for v in x0]
        p, dt = self.plant, self.dt
        for k in range(N):
            n_sub = self.rk_substeps
            if xk[IDX_VX] >= self.rk_gate_speed:
                n_sub = min(n_sub, self.rk_substeps_fast)
            xk = _step_scalar(xk, U[k], ref, p, dt, n_sub)
            X[k + 1] = xk
        return X

    def _jacobians(self, X, U, ref):
        """Finite-difference the one-step dynamics Jacobians A_k/B_k,
        vectorised across all horizon stages at once — see the live
        nmpc_core.py's _jacobians for why finite-differencing (not
        hand-derived), the nmpc_jac_substeps accuracy/cost tradeoff, and the
        jac_gate_speed/jac_substeps_fast speed gate this mirrors (gated on the
        horizon's slowest predicted stage, not instantaneous speed, so it
        changes rarely)."""
        N = self.N
        Xs = X[:N]
        p, dt = self.plant, self.dt
        n_sub = self.jac_substeps
        if float(X[:N, IDX_VX].min()) >= self.jac_gate_speed:
            n_sub = min(n_sub, self.jac_substeps_fast)
        F0 = _step(Xs, U, ref, p, dt, n_sub)
        A = np.empty((N, NX, NX))
        B = np.empty((N, NX, NU))
        for j in range(NX):
            Xp = Xs.copy()
            Xp[:, j] += _FD_EPS_X[j]
            A[:, :, j] = (_step(Xp, U, ref, p, dt, n_sub) - F0) / _FD_EPS_X[j]
        for j in range(NU):
            Up = U.copy()
            Up[:, j] += _FD_EPS_U[j]
            B[:, :, j] = (_step(Xs, Up, ref, p, dt, n_sub) - F0) / _FD_EPS_U[j]
        return A, B

    def _output_jacobians(self, X, ref, v_ref, v_cap=None, s_target_N=None):
        """Finite-difference the stage-output Jacobians C_k (h(x) w.r.t.
        state) — see the live nmpc_core.py's _output_jacobians; identical
        here.

        When self.friction_circle_enabled, H0/C carry NH_FRICTION extra rows
        (F_yf, F_yr — see _outputs' docstring), riding along through this
        SAME finite-difference pass at no extra rollout cost. Shape is
        (stages, NH, NX) when the flag is False, IDENTICAL to before this
        feature existed.

        When self.progress_enabled, v_cap/s_target_N are threaded through to
        _outputs unchanged (see that function's docstring) -- perturbing
        X[:, IDX_S] here naturally captures h_prog's sensitivity to the
        terminal arc length through the SAME pass, no extra rollout."""
        H0 = _outputs(X, ref, self.plant, v_ref,
                      friction_circle_enabled=self.friction_circle_enabled,
                      progress_enabled=self.progress_enabled,
                      v_cap=v_cap, s_target_N=s_target_N,
                      progress_v_min=self.progress_v_min)
        n_rows = H0.shape[1]
        C = np.empty((X.shape[0], n_rows, NX))
        for j in range(NX):
            Xp = X.copy()
            Xp[:, j] += _FD_EPS_X[j]
            Hp = _outputs(Xp, ref, self.plant, v_ref,
                         friction_circle_enabled=self.friction_circle_enabled,
                         progress_enabled=self.progress_enabled,
                         v_cap=v_cap, s_target_N=s_target_N,
                         progress_v_min=self.progress_v_min)
            C[:, :, j] = (Hp - H0) / _FD_EPS_X[j]
        return H0, C

    def _r_delta_stage0(self, X):
        """Stage 0's steering-effort weight for this tick: r_delta normally,
        scaled up while the car is slow and faded linearly back to 1x
        between standstill_speed and standstill_fade_speed -- see the live
        nmpc_core.py's _r_delta_stage0 for the mechanism and for why the
        fade replaced a hard cutoff. Shared by
        _solve_step and _cost so the QP and the line search cannot score
        different objectives."""
        if not self.standstill_steer_damp_enabled:
            return self.r_delta
        v = float(X[0, IDX_VX])
        lo, hi = self.standstill_speed, self.standstill_fade_speed
        if v <= lo:
            scale = self.standstill_steer_r_scale
        elif v >= hi or hi <= lo:
            scale = 1.0
        else:
            frac = (v - lo) / (hi - lo)
            scale = self.standstill_steer_r_scale + (
                1.0 - self.standstill_steer_r_scale) * frac
        return self.r_delta * scale

    def _cost(self, X, U, H):
        """True nonlinear cost at a candidate (X, U) — used for the
        backtracking check after each SQP step; see the live nmpc_core.py's
        _cost for the Gauss-Newton stage-output weighting this mirrors.

        H may carry NH_FRICTION extra (unweighted) columns when
        friction_circle_enabled -- sliced down to the original NH cost rows
        here so w (len NH) always broadcasts correctly and the objective
        itself never includes the friction rows, per the feature's spec."""
        w = self.w_out
        Hc = H[:, :self.NH]
        stage = float(np.sum(w * Hc[:-1] ** 2)) + float(
            self.terminal_scale * np.sum(w * Hc[-1] ** 2))
        a = U[:, 1]
        # Stage 0's steering weight can differ from the rest (standstill
        # damping) -- see _r_delta_stage0. Written as the flat term plus a
        # stage-0 correction so the flag-off path is bit-identical to the
        # original single-multiply expression.
        eff = float(self.r_delta * np.sum(U[:, 0] ** 2)
                    + (self._r_delta_stage0(X) - self.r_delta) * U[0, 0] ** 2
                    + self.r_a_accel * np.sum(np.maximum(a, 0.0) ** 2)
                    + self.r_a_brake * np.sum(np.minimum(a, 0.0) ** 2))
        du = np.vstack([U[0] - self._u_prev, np.diff(U, axis=0)])
        # Score the rate term with self._Rr_flat -- the SAME per-stage weight
        # vector the QP's own Hessian (_ErE) is built from -- not the flat
        # self.r_rate. Any mechanism that reshapes the rate weight (the
        # corner blend, anti-hunt, the reversal penalty, or the per-stage
        # ramp) writes _Rr_flat; if this line used self.r_rate instead, the
        # backtracking line search would be scoring a DIFFERENT objective
        # from the one the QP minimised and could reject genuinely improving
        # steps. Falls back to the flat tile when _Rr_flat is absent.
        _rr = getattr(self, '_Rr_flat', None)
        if _rr is None or _rr.shape[0] != du.size:
            rate = float(np.sum(self.r_rate * du ** 2))
        else:
            rate = float(np.sum(_rr * du.reshape(-1) ** 2))
        jerk = 0.0
        if self._E2rE2 is not None:
            # Same objective the QP minimises (see _solve_step's jerk block),
            # so the backtracking line search cannot reject a step the QP
            # considers improving.
            d2 = np.vstack([
                du[0] - (self._u_prev - self._u_prev2),
                np.diff(du, axis=0),
            ])
            jerk = float(np.sum(np.array([self.rjerk_delta, self.rjerk_a]) * d2 ** 2))
        slack = 0.0
        if self._use_slack:
            over = np.maximum(np.abs(X[1:, IDX_EY]) - self.track_halfwidth, 0.0)
            # Quadratic + linear, matching _solve_step's q[idx] line -- the
            # QP's own slack variable is driven to exactly this `over` at
            # the optimum (nothing else rewards it being larger), so scoring
            # it here as max(0, |e_y|-hw) is the true-cost equivalent of the
            # QP's own slack decision variable, not an approximation of it.
            slack = float(self.slack_weight * np.sum(over ** 2)
                          + self.slack_linear_weight * np.sum(over))
        return stage + eff + rate + jerk + slack

    def _project_feasible(self, U):
        """Project onto input bounds + per-step slew feasibility from
        `self._u_prev` forward, so `dU=0` is always feasible and the SQP
        subproblem is unconditionally feasible by construction — see the
        live module's identical method for why this matters (a warm start
        that violates the slew limit can make the whole subproblem
        primal-infeasible, and OSQP returns a finite-but-meaningless answer
        in that case)."""
        Up = np.clip(np.asarray(U, dtype=float), self.u_min, self.u_max)
        prev = self._u_prev
        for k in range(Up.shape[0]):
            Up[k] = np.clip(Up[k], prev - self.du_max, prev + self.du_max)
            prev = Up[k]
        return Up

    def _solve_step(self, X, U, ref, v_ref, v_cap=None, s_target_N=None):
        """One Gauss-Newton SQP iteration: condense, solve the QP, return dU
        and the OSQP status. Because X was rolled forward from the measured
        state (see _rollout), the linearised dynamics have ZERO defect, so
        the condensed sensitivities alone describe the subproblem exactly —
        see the live nmpc_core.py's _solve_step; identical here.

        When self.friction_circle_enabled, H/C carry NH_FRICTION extra
        (unweighted) rows (see _outputs) -- G/g below are built from ONLY
        the first NH rows (the cost), and the friction rows are sliced out
        separately further down to build the hard QP constraint.

        v_cap/s_target_N are ignored unless self.progress_enabled -- see
        _outputs' docstring."""
        N = self.N
        qp = self._qp
        n_du, n_slack, nz, n_rows = (
            qp['n_du'], qp['n_slack'], qp['nz'], qp['n_rows'])

        A_k, B_k = self._jacobians(X, U, ref)
        H, C = self._output_jacobians(X, ref, v_ref, v_cap=v_cap, s_target_N=s_target_N)
        Hc, Cc = H[:, :self.NH], C[:, :self.NH, :]

        S = np.zeros((N + 1, NX, n_du))
        for k in range(N):
            S[k + 1] = A_k[k] @ S[k]
            S[k + 1][:, k * NU:(k + 1) * NU] += B_k[k]

        sw = np.sqrt(self.w_out)
        scale = np.ones(N + 1)
        scale[N] = math.sqrt(max(self.terminal_scale, 0.0))
        WC = (sw[None, :, None] * Cc) * scale[:, None, None]
        G = np.einsum('kij,kjl->kil', WC, S).reshape((N + 1) * self.NH, n_du)
        g = ((sw[None, :] * Hc) * scale[:, None]).reshape(-1)

        ru = np.empty((N, NU))
        ru[:, 0] = self.r_delta
        # Stage 0 only, and only while measurably stationary -- see
        # _r_delta_stage0. _cost applies the same weight, so the line search
        # scores the objective this QP actually minimises.
        ru[0, 0] = self._r_delta_stage0(X)
        ru[:, 1] = np.where(U[:, 1] >= 0.0, self.r_a_accel, self.r_a_brake)
        ru_flat = ru.reshape(-1)
        u_flat = U.reshape(-1)

        e_rate = self._E @ u_flat
        e_rate[:NU] -= self._u_prev

        Hess = G.T @ G + np.diag(ru_flat) + self._ErE
        grad = G.T @ g + ru_flat * u_flat + self._E.T @ (self._Rr_flat * e_rate)
        if self._E2rE2 is not None:
            # Steering-JERK term: ||E2 u - d2_anchor||^2_Rj contributes
            # E2'RjE2 to the Hessian and E2'Rj(E2 u - d2_anchor) to the
            # gradient. The anchor carries the LAST TWO commands into step 0's
            # second difference, the same way _u_prev anchors the first
            # difference -- without it the term is blind to a reversal that
            # spans the tick boundary, which is exactly what it exists to
            # catch.
            e_jerk = self._E2 @ u_flat
            e_jerk[:NU] -= (2.0 * self._u_prev - self._u_prev2)
            e_jerk[NU:2 * NU] += self._u_prev
            rj = np.tile(np.array([self.rjerk_delta, self.rjerk_a]), N)
            Hess = Hess + self._E2rE2
            grad = grad + self._E2.T @ (rj * e_jerk)

        P_dense = np.zeros((nz, nz))
        P_dense[:n_du, :n_du] = 2.0 * Hess
        q = np.zeros(nz)
        q[:n_du] = 2.0 * grad
        if n_slack:
            idx = np.arange(n_du, n_du + n_slack)
            P_dense[idx, idx] = 2.0 * self.slack_weight
            q[idx] = self.slack_linear_weight

        A_dense = np.zeros((n_rows, nz))
        l = np.empty(n_rows)
        u = np.empty(n_rows)

        A_dense[:n_du, :n_du] = np.eye(n_du)
        tr = np.tile(np.array([self.trust_delta_rad, self.trust_a]), N)
        lo = np.maximum(np.tile(self.u_min, N) - u_flat, -tr)
        hi = np.minimum(np.tile(self.u_max, N) - u_flat, tr)
        lo = np.minimum(lo, hi)
        l[:n_du], u[:n_du] = lo, hi

        A_dense[n_du:2 * n_du, :n_du] = self._E
        du_flat = np.tile(self.du_max, N)
        l[n_du:2 * n_du] = -du_flat - e_rate
        u[n_du:2 * n_du] = du_flat - e_rate

        if n_slack:
            r0 = 2 * n_du
            hw = self.track_halfwidth
            S_ey = S[1:, IDX_EY, :]
            ey = X[1:, IDX_EY]
            A_dense[r0:r0 + N, :n_du] = S_ey
            A_dense[r0:r0 + N, n_du:] = -np.eye(N)
            l[r0:r0 + N] = -np.inf
            u[r0:r0 + N] = hw - ey
            A_dense[r0 + N:r0 + 2 * N, :n_du] = S_ey
            A_dense[r0 + N:r0 + 2 * N, n_du:] = np.eye(N)
            l[r0 + N:r0 + 2 * N] = -hw - ey
            u[r0 + N:r0 + 2 * N] = np.inf
            A_dense[r0 + 2 * N:r0 + 3 * N, n_du:] = np.eye(N)
            l[r0 + 2 * N:r0 + 3 * N] = 0.0
            u[r0 + 2 * N:r0 + 3 * N] = np.inf

        n_fric = qp['n_fric']
        if n_fric:
            # Hard |F_yf|, |F_yr| <= F_max bound, ADDITIONAL to the existing
            # soft alat-ceiling saturation inside _f/_f_scalar (untouched).
            # F_axle(x0) + dF/dU_flat @ dU, linearised at the current
            # iterate exactly like the soft-track rows above -- dF/dU_flat
            # is C's two extra rows (dF/dx, "for free" from
            # _output_jacobians) composed with the SAME S = dx/dU_flat the
            # cost rows already use. A symmetric two-sided bound needs only
            # ONE row per axle per stage (both l and u set), hence n_fric =
            # 2 (axles) * N (stages), not 4*N.
            rf0 = 2 * n_du + (3 * N if n_slack else 0)
            F0 = H[1:, self.NH:self.NH + 2]          # (N, 2): F_yf, F_yr at x0
            dF_dU = np.einsum('kij,kjl->kil', C[1:, self.NH:self.NH + 2, :], S[1:])  # (N,2,n_du)
            v_x_pred = X[1:, IDX_VX]
            F_max = np.maximum(self._fmax_flat,
                               self._fmax_slope * np.abs(v_x_pred) + self._fmax_intercept)
            # Rows rf0 .. rf0+N-1: front axle.
            A_dense[rf0:rf0 + N, :n_du] = dF_dU[:, 0, :]
            l[rf0:rf0 + N] = -F_max - F0[:, 0]
            u[rf0:rf0 + N] = F_max - F0[:, 0]
            # Rows rf0+N .. rf0+2N-1: rear axle.
            A_dense[rf0 + N:rf0 + 2 * N, :n_du] = dF_dU[:, 1, :]
            l[rf0 + N:rf0 + 2 * N] = -F_max - F0[:, 1]
            u[rf0 + N:rf0 + 2 * N] = F_max - F0[:, 1]


        qp['prob'].update(
            Px=P_dense[qp['p_rows'], qp['p_cols']],
            Ax=A_dense[qp['a_rows'], qp['a_cols']],
            q=q, l=l, u=u,
        )
        res = qp['prob'].solve()
        status = str(res.info.status).lower()
        ok = ('solved' in status) or ('maximum iterations' in status)
        if not ok or res.x is None or not np.all(np.isfinite(res.x[:n_du])):
            return None, status
        return res.x[:n_du].reshape(N, NU), status

    # ------------------------------------------------------------------
    def compute_step(
        self, path, car_pos, car_yaw, car_speed, desired_speed,
        car_yaw_rate=0.0, car_vy=0.0, pending_cmds=None,
        dense_step=0.5, smooth_w=3, kappa_clip=0.5,
        step_index=0,
    ):
        """
        One NMPC control step. Deliberately DIFFERENT calling convention from
        the live module's `compute()`: returns raw `(u_opt, status_dict)`
        with `u_opt = [delta_cmd (rad), a_cmd (m/s^2)]` — matching
        `controller/optimiser.solve_mpc()`'s own return convention — rather
        than normalised (steering, throttle, brake) FSDS units, since
        `sim/rollout_core.py`'s `step_nonlinear_plant()` (unlike the live
        ROS node) wants the raw physical command directly.

        `pending_cmds` (list of `[delta_cmd, a_cmd]` arrays, oldest first) is
        used for delay compensation via a NONLINEAR rollforward, exactly
        like the live module's `_u_history`-based rollforward — but is taken
        as an explicit argument here rather than derived internally from a
        `pose_age_s` measurement, because `sim/rollout_core.py`'s existing
        delay-JITTER model (settings.DELAY_JITTER_STEPS) already perturbs
        the BELIEVED pending-command list directly (see that module's
        "Delay-estimation error" comment) — reusing that list is more
        faithful to what the live car's noisy pose-timestamp model is
        actually standing in for than re-deriving a second, independent
        noise source here.

        `step_index`: 0 on the rollout's first tick (skips warm-start, same
        as `solve_mpc`'s own `warm_start=(step != 0)`).
        """
        import time
        t0 = time.perf_counter()

        ref = self.path_reference(
            path, dense_step=dense_step, smooth_w=smooth_w, kappa_clip=kappa_clip,
            spline_reference_enabled=self.spline_reference_enabled,
        )
        if ref.total < 1e-3:
            return np.array([self._u_prev[0], self.u_min[1]]), {
                'iters': 0, 'status': 'no-path', 'cost': float('nan'),
                'corner_frac': 0.0,
            }

        fa = np.asarray(car_pos, dtype=float) + self.lf * np.array(
            [math.cos(car_yaw), math.sin(car_yaw)])
        s0, e_y, e_psi, base_idx, path_yaw = ref.project(fa, car_yaw)
        x0 = np.array([
            s0, e_y, e_psi, max(float(car_speed), 0.0), float(car_vy),
            float(car_yaw_rate), self._delta_act, self._a_act,
        ])

        # Corner-blend / anti-hunt (EXPERIMENTAL, default off) -- mirrors
        # nmpc_core.py's own block exactly: ALTERNATIVES, not composed (blend
        # takes priority when both are enabled). Same signal (current
        # kappa/e_y/e_psi), same functions (model_utils, imported not
        # reimplemented), computed once per compute_step() call and applied
        # UNIFORMLY across the whole horizon for this tick's solve. When both
        # flags are off (default), self._Rr_flat/self._ErE are untouched
        # here, so behaviour is byte-identical to before either existed.
        # Always computed (not gated behind corner_rrate_blend_enabled) --
        # this is a general current-curvature signal other mechanisms key
        # off via the returned diag dict, independent of whether the R_rate
        # weight-blend feature itself is active.
        kappa_now = float(ref.kappa_at(np.array([s0]))[0])
        corner_frac = _corner_factor(kappa_now, self.corner_factor_k)
        m_rrate_antihunt = 1.0
        # Tracks R_rate[0,0]'s running value through the if/elif AND the
        # reversal-penalty composition below, so the reversal penalty (which
        # applies regardless of which branch ran) boosts whatever value is
        # actually current rather than always the pre-if/elif base -- the same
        # silent-discard bug already found and fixed in mpc_core.py's own
        # corner-blend/anti-hunt composition. Mirrors nmpc_core.py.
        rrate_steer_current = float(self.r_rate[0])
        if self.corner_rrate_blend_enabled:
            rrate_blend = _blend(self.rrate_steer_straight, self.rrate_steer_corner, corner_frac)
            rrate_steer_current = float(rrate_blend)
        elif self.steer_rate_anti_hunt_enabled:
            R2 = steer_rate_anti_hunt(
                kappa_now, e_y, np.diag(self.r_rate), enabled=True, e_psi=e_psi,
            )
            m_rrate_antihunt = (
                float(R2[0, 0] / self.r_rate[0]) if self.r_rate[0] else 1.0)
            rrate_steer_current = float(R2[0, 0])

        m_rrate_reversal = 1.0
        if self.reversal_penalty_enabled:
            R3 = reversal_penalty_boost(
                float(self._u_prev[0]), np.diag([rrate_steer_current, self.r_rate[1]]),
                enabled=True, boost_max=self.reversal_penalty_boost_max,
                k=self.reversal_penalty_k,
            )
            m_rrate_reversal = (
                float(R3[0, 0] / rrate_steer_current) if rrate_steer_current else 1.0)
            rrate_steer_current = float(R3[0, 0])

        if (self.corner_rrate_blend_enabled or self.steer_rate_anti_hunt_enabled
                or self.reversal_penalty_enabled or self.rrate_stage_ramp_enabled
                or self.rrate_zone_enabled):
            r_rate_tick = np.array([rrate_steer_current, self.r_rate[1]])
            Rr_flat = np.tile(r_rate_tick, self.N)
            if self.rrate_stage_ramp_enabled:
                Rr_flat = (Rr_flat.reshape(self.N, NU)
                           * _rrate_stage_ramp(self.N, self.rrate_stage_near)[:, None]
                           ).reshape(-1)
            self._Rr_flat = Rr_flat
            self._ErE = self._E.T @ (Rr_flat[:, None] * self._E)

        if pending_cmds:
            xk = [float(v) for v in x0]
            for u_hist in pending_cmds:
                xk = _step_scalar(xk, u_hist, ref, self.plant, self.dt, self.rk_substeps)
            x0 = np.array(xk)

        # Latency compensation (EXPERIMENTAL, mirrors nmpc_core.py's own
        # block): same nonlinear rollforward as the pending_cmds block just
        # above, but forward past "now" using the LAST APPLIED command held
        # constant, to compensate for THIS solve's own wall-clock time
        # rather than pose staleness. Held constant, not extrapolated -- the
        # true future command is exactly what this solve is trying to
        # determine. See settings.py's NMPC_LATENCY_COMPENSATION_* comments.
        if self.latency_compensation_enabled:
            # math.floor(x + 0.5), not round(): round() is round-half-to-
            # even, so exactly 25 ms at dt=0.05 s (0.5 steps) rounds to 0,
            # silently disabling this at its own default value -- caught
            # live 2026-09-15 (n_latency logged 0 for an entire run at the
            # 25 ms default).
            n_latency = int(math.floor(
                self.latency_compensation_ms * 1e-3 / self.dt + 0.5))
            if n_latency > 0:
                xk = [float(v) for v in x0]
                for _ in range(n_latency):
                    xk = _step_scalar(xk, self._u_prev, ref, self.plant, self.dt,
                                      self.rk_substeps)
                x0 = np.array(xk)

        if self._have_warm_start and step_index != 0:
            U = np.vstack([self._U[1:], self._U[-1:]])
        else:
            U = np.tile(self._u_prev, (self.N, 1))
        U = self._project_feasible(U)

        budget_s = self.solve_budget_ms * 1e-3
        X = self._rollout(x0, U, ref)

        # Three-zone rate schedule (rrate_zone_enabled). Applied HERE rather
        # than in the block above because it needs kappa across the PREDICTED
        # horizon, which only exists once X has been rolled out. Recomputing
        # _ErE costs one (n_du x n_du) product per tick, the same cost the
        # other rate-reshaping flags already pay.
        m_rrate_zone = 1.0
        if self.rrate_zone_enabled:
            kap_h = ref.kappa_at(X[:, IDX_S])
            m_rrate_zone = _rrate_zone_scale(
                kappa_now, float(np.abs(kap_h).max()), self.corner_factor_k,
                self.rrate_zone_boost_straight, self.rrate_zone_ease_approach,
                self.rrate_zone_floor_corner)
            self._Rr_flat = self._Rr_flat * np.tile(
                np.array([m_rrate_zone, 1.0]), self.N)
            self._ErE = self._E.T @ (self._Rr_flat[:, None] * self._E)

        # Progress mode: `desired_speed` becomes a CAP (row 4 of h(), see
        # _outputs) rather than a two-sided target, and s_target_N is an
        # UNREACHABLE arc-length goal that turns "maximise s" into a
        # GN-native least-squares residual -- see _outputs' docstring for why
        # a bare linear reward is unsafe for this solver. Computed ONCE per
        # tick (not per backtracking trial): it depends only on
        # s0/desired_speed/v_x, all fixed for this whole compute_step() call.
        #
        # The unreachable gap is v_cap*N*dt*progress_reach, EXCEPT it is
        # floored at a KINEMATIC distance (0.5*a_max*(N*dt)^2*progress_reach,
        # the horizon's own maximum reach under full commanded accel from
        # rest) rather than scaled off v_cap alone. This matters specifically
        # at launch: SPEED_TARGET_DEFICIT_MAX holds v_cap near ~2.5 m/s while
        # the car is still stationary (see settings.py), which on its own
        # makes v_cap*N*dt*reach a few metres -- too close for the residual
        # to stay "unreachable" once a_cmd approaches the ~2.3 m/s^2 needed
        # to break static friction (measured: F_stiction=600N / m=255kg in
        # model/vehicle_physics.py). The reward then saturates at a tiny gap
        # and settles well below the accel stiction needs, so the car never
        # launches (measured live in this repo's own offline rollout: a_cmd
        # plateaus at ~0.6-1.2 forever, v_x stays exactly 0). The kinematic
        # floor is a property of the CONTROLLER's own known authority
        # (u_max[1], already available), not the plant's stiction constant --
        # deliberately not coupled to that, since a real car's launch
        # friction is not something the cost design should need to know.
        # Anchored on X[0, IDX_S], NOT the raw projected s0: when delay
        # compensation or pending_cmds rolled x0 forward above, the rollout
        # starts further along the path than s0 measured, and anchoring the
        # target to the stale s0 shrinks the gap by exactly that rollforward
        # distance -- enough to stop the car launching at all when the gap is
        # already small (measured: a_cmd plateaus ~1.3, below the ~2.3 needed
        # for stiction). X[0] is the same trajectory the residual scores.
        v_cap = desired_speed
        s_target_N = None
        if self.progress_enabled:
            gap = v_cap * self.N * self.dt * self.progress_reach
            kinematic_floor = (0.5 * self.u_max[1] * (self.N * self.dt) ** 2
                              * self.progress_reach)
            s_target_N = float(X[0, IDX_S]) + max(gap, kinematic_floor)

        H = _outputs(X, ref, self.plant, desired_speed,
                     friction_circle_enabled=self.friction_circle_enabled,
                     progress_enabled=self.progress_enabled,
                     v_cap=v_cap, s_target_N=s_target_N,
                     progress_v_min=self.progress_v_min)
        cost = self._cost(X, U, H)
        iters = 0
        status = 'warm-start-only'
        for _ in range(max(1, self.sqp_iters)):
            if time.perf_counter() - t0 > budget_s:
                status = 'budget'
                break
            dU, status = self._solve_step(X, U, ref, desired_speed,
                                          v_cap=v_cap, s_target_N=s_target_N)
            if dU is None:
                break
            step = 1.0
            accepted = False
            for _bt in range(max(0, self.backtrack_max) + 1):
                # Budget check also lives here, not just above the outer loop:
                # at the shipped sqp_iters=1 the outer check can only ever stop
                # the single iteration from starting, never interrupt the
                # Jacobian build or this backtracking search, which is where
                # per-tick time actually varies (each trial re-rolls out the
                # full horizon). _bt > 0 keeps the first trial unconditional so
                # the common on-budget case is untouched. Mirrors nmpc_core.py.
                if _bt > 0 and time.perf_counter() - t0 > budget_s:
                    break
                U_try = np.clip(U + step * dU, self.u_min, self.u_max)
                X_try = self._rollout(x0, U_try, ref)
                H_try = _outputs(X_try, ref, self.plant, desired_speed,
                                 friction_circle_enabled=self.friction_circle_enabled,
                                 progress_enabled=self.progress_enabled,
                                 v_cap=v_cap, s_target_N=s_target_N,
                                 progress_v_min=self.progress_v_min)
                cost_try = self._cost(X_try, U_try, H_try)
                if cost_try <= cost:
                    U, X, H, cost = U_try, X_try, H_try, cost_try
                    accepted = True
                    break
                step *= 0.5
            iters += 1
            if not accepted:
                status = 'rejected'
                break

        self._U = U
        self._have_warm_start = True

        u_opt = np.clip(U[0], self.u_min, self.u_max)
        u_opt = np.clip(u_opt, self._u_prev - self.du_max, self._u_prev + self.du_max)

        exp_delta = math.exp(-self.dt / self.plant.tau_delta)
        exp_a = math.exp(-self.dt / self.plant.tau_a)
        self._delta_act = self._delta_act * exp_delta + u_opt[0] * (1.0 - exp_delta)
        self._a_act = self._a_act * exp_a + u_opt[1] * (1.0 - exp_a)
        self._u_prev2 = self._u_prev.copy()
        self._u_prev = u_opt.copy()

        diag = {
            'iters': iters,
            'status': status,
            'solved': status.lower().startswith('solved'),
            'cost': float(cost),
            's0': float(s0),
            'e_y': float(e_y),
            'e_psi': float(e_psi),
            'kappa': float(ref.kappa_scalar(s0)),
            'kappa_max_abs': float(np.abs(ref.kappa_at(X[:, IDX_S])).max()),
            'pred_ey_end': float(X[-1, IDX_EY]),
            'pred_epsi_end': float(X[-1, IDX_EPSI]),
            'pred_ey_max_abs': float(np.abs(X[:, IDX_EY]).max()),
            'solve_ms': (time.perf_counter() - t0) * 1e3,
            'corner_frac': corner_frac,
            # Same keys/semantics as nmpc_core.py's last_telemetry:
            # per-mechanism multipliers, plus the FINAL fully-composed
            # R_rate[0,0] actually used this tick (post corner-blend AND
            # anti-hunt AND reversal-penalty), not just one stage in isolation.
            'm_Rrate_antihunt': m_rrate_antihunt,
            'm_Rrate_reversal': m_rrate_reversal,
            'm_Rrate_zone': m_rrate_zone,
            'Rrate_steer_corner_blend': rrate_steer_current,
        }
        if self.friction_circle_enabled:
            # H's two extra (unweighted) rows -- realized per-axle force at
            # the FINAL accepted trajectory, see _outputs' docstring.
            diag['nmpc_fyf_max_abs'] = float(np.abs(H[:, self.NH]).max())
            diag['nmpc_fyr_max_abs'] = float(np.abs(H[:, self.NH + 1]).max())
        if self.progress_enabled:
            # Required by every A/B per the plan: v_cap_active distinguishes
            # "car chose to go slower than the cap" from "cap is binding",
            # s_target_gap_end is how far short of the (deliberately
            # unreachable) progress target the horizon ends -- a stuck/
            # regressing solve shows up as this GROWING tick over tick, not
            # shrinking. a_cmd itself is already u_opt[1], logged by the
            # caller (rollout_core.py) alongside this dict, not duplicated
            # here.
            diag['v_cap'] = float(v_cap if np.isscalar(v_cap) else v_cap[0])
            diag['speed_cap_over'] = float(max(0.0, float(x0[IDX_VX]) - diag['v_cap']))
            diag['s_target_gap_end'] = float(s_target_N - X[-1, IDX_S])
        return u_opt, diag
