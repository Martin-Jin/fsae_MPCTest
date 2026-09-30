"""
controller/nmpc/qp_model.py — `_QPModelMixin`: the NMPC's QP structure and cost.

PURPOSE
-------
Builds the fixed-sparsity OSQP problem once (`_build_qp`), and evaluates the
horizon rollout, its state/output Jacobians and the cost that the SQP loop
in sqp_step.py linearises around. Split from solver.py only to keep files
readable; the live `nmpc.solver.NMPCController` defines all of these methods
on one class, and the names here match it one to one.

USED BY
-------
  solver.py: `NMPCController` inherits this mixin. `__init__` calls
  `_build_qp`; `compute_step` calls `_rollout` and `_cost`.
"""

import numpy as np
import scipy.sparse as sp

from controller.nmpc.layout import (
    IDX_EY, IDX_VX, NX, NU, _FD_EPS_X, _FD_EPS_U,
)
from controller.nmpc.dynamics import _step_scalar, _step
from controller.nmpc.outputs import _outputs

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


class _QPModelMixin:
    """QP construction, horizon rollout, Jacobians and cost for NMPCController."""

    # ------------------------------------------------------------------
    def _build_qp(self):
        """Allocate the condensed QP once with fixed sparsity — see the live
        the live nmpc/qp_model.py's _build_qp for the constraint-row layout (box/slew/
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
        # expensive. See docs/reference/control_mechanisms.md ("Input-jerk cost").
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
        the live nmpc/qp_model.py's _rollout for why this makes the QP's dynamics defect
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
        the live nmpc/qp_model.py's _jacobians for why finite-differencing (not
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
        state) — see the live nmpc/qp_model.py's _output_jacobians; identical
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
        the live nmpc/qp_model.py's _r_delta_stage0 for the mechanism and for why the
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
        backtracking check after each SQP step; see the live nmpc/sqp_step.py's
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
