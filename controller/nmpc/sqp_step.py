"""
controller/nmpc/sqp_step.py — `_SQPStepMixin`: one Gauss-Newton SQP step.

PURPOSE
-------
`_solve_step` linearises the horizon about the current (X, U) trajectory,
updates the OSQP problem built in qp_model.py and returns the control
correction `dU`; `_project_feasible` clips a candidate trajectory back
inside the input and rate limits. Split from solver.py only to keep files
readable; the names match the live `nmpc_core.NMPCController` one to one.

USED BY
-------
  solver.py: `NMPCController` inherits this mixin; `compute_step` runs the
  SQP loop by calling `_solve_step` and `_project_feasible`.
"""

import math

import numpy as np

from controller.nmpc.layout import IDX_EY, IDX_VX, NX, NU


class _SQPStepMixin:
    """One SQP iteration and feasibility projection for NMPCController."""

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
