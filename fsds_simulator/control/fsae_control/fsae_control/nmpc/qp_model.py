"""
fsae_control/nmpc/qp_model.py — `_QPModelMixin`: QP structure and cost

Builds the fixed-sparsity OSQP problem once (`_build_qp`) and evaluates the
horizon rollout, Jacobians and cost (`_cost`, `_cost_breakdown`) the SQP loop in
sqp_step.py linearises around. Split from solver.py only for readability; method
names match the offline controller/nmpc/qp_model.py one to one.
"""

import numpy as np
import scipy.sparse as sp

try:
    import osqp
except ImportError as _exc:      # pragma: no cover - see package.xml
    osqp = None
    _OSQP_IMPORT_ERROR = _exc

from fsae_control.nmpc.layout import (
    IDX_EY,
    IDX_VX,
    NU,
    NX,
    _FD_EPS_U,
    _FD_EPS_X,
)
from fsae_control.nmpc.outputs import _outputs
from fsae_control.nmpc.dynamics import _step, _step_scalar


def _csc_pattern(mask):
    """
    Build a CSC matrix with an explicit, fixed sparsity pattern from a boolean
    mask, plus the (row, col) index arrays that write into its .data in CSC
    order.

    OSQP requires every update() to keep the pattern it was set up with, so the
    pattern is fixed once and only the data array is refilled per solve. Going
    through a mask (rather than converting a dense value array) is what
    guarantees a structural zero stays structurally present instead of being
    dropped by scipy.
    """
    m = sp.csc_matrix(mask.astype(np.float64))
    rows = m.indices.copy()
    cols = np.zeros_like(rows)
    for j in range(m.shape[1]):
        cols[m.indptr[j]:m.indptr[j + 1]] = j
    return m, rows, cols


class _QPModelMixin:
    """QP construction, horizon rollout, Jacobians and cost for NMPCController."""

    # ------------------------------------------------------------------
    # QP subproblem (condensed, dense, fixed sparsity)
    # ------------------------------------------------------------------
    def _build_qp(self) -> None:
        """
        Allocate the condensed QP once: variables
        z = [dU (nu*N); slack (N)], with the constraint rows

            (1) box/trust region on dU                       nu*N rows
            (2) input slew rate |u_k - u_{k-1}| <= du_max     nu*N rows
            (3) e_y_k - slack_k <= +halfwidth                 N rows
            (4) e_y_k + slack_k >= -halfwidth                 N rows
            (5) slack >= 0                                    N rows

        Rows (3)-(5) and the slack variables are omitted entirely when
        nmpc_track_halfwidth <= 0. The pattern never changes after this, so
        each solve only rewrites P.data / A.data / q / l / u.

        Friction-circle rows (self.friction_circle_enabled, see
        NMPCParams.nmpc_friction_circle_enabled): one two-sided
        (-F_max <= ... <= F_max) row per axle per stage = 2 axles * N
        stages, dense in dU (same reasoning as the soft-track rows above —
        stage k's tyre force depends on every earlier input through S).
        Read ONCE here, at construction time, like _use_slack — NOT
        per-tick — since it changes the QP's fixed sparsity pattern. When
        False, n_rows/nz and every array below are IDENTICAL to before this
        feature existed.
        """
        N = self.N
        n_du = NU * N
        self._use_slack = self.nmpc.nmpc_track_halfwidth > 0.0
        n_slack = N if self._use_slack else 0
        nz = n_du + n_slack
        n_fric = 2 * N if self.friction_circle_enabled else 0

        # First-difference operator E: diff_k = u_k - u_{k-1} (u_{-1} = u_prev).
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
        # The plain rate cost charges by |du|, identical for a sustained ramp
        # into a corner and for one leg of an oscillation, so it cannot damp
        # hunting without also resisting turn-in. Measured live, reversals
        # carry ~4.3x the |d2| of same-direction ramps vs only ~1.9x the
        # |d1|. A steady ramp is therefore nearly free here; a wiggle is not.
        # No OSQP sparsity change: p_mask[:n_du,:n_du] is already a dense
        # upper triangle, so this adds no new nonzeros to the pattern.
        self._E2 = E @ E
        rj = np.tile(np.array([self.rjerk_delta, self.rjerk_a]), N)
        self._E2rE2 = (self._E2.T @ (rj[:, None] * self._E2)
                       if (self.rjerk_delta or self.rjerk_a) else None)

        # P pattern: dense upper triangle over dU, plus the slack diagonals.
        p_mask = np.zeros((nz, nz), dtype=bool)
        p_mask[:n_du, :n_du] = np.triu(np.ones((n_du, n_du), dtype=bool))
        if n_slack:
            idx = np.arange(n_du, n_du + n_slack)
            p_mask[idx, idx] = True
        P, p_rows, p_cols = _csc_pattern(p_mask)

        # A pattern.
        n_rows = 2 * n_du + (3 * N if self._use_slack else 0) + n_fric
        a_mask = np.zeros((n_rows, nz), dtype=bool)
        a_mask[:n_du, :n_du] = np.eye(n_du, dtype=bool)
        a_mask[n_du:2 * n_du, :n_du] = E != 0.0
        if self._use_slack:
            r0 = 2 * n_du
            # Track rows are structurally dense in dU: stage k's e_y depends on
            # every earlier input, and marking the (structurally zero) later
            # columns present too keeps this pattern trivially fixed.
            a_mask[r0:r0 + 2 * N, :n_du] = True
            for k in range(N):
                a_mask[r0 + k, n_du + k] = True                 # -slack_k
                a_mask[r0 + N + k, n_du + k] = True              # +slack_k
                a_mask[r0 + 2 * N + k, n_du + k] = True          # slack_k >= 0
        if n_fric:
            rf0 = 2 * n_du + (3 * N if self._use_slack else 0)
            a_mask[rf0:rf0 + n_fric, :n_du] = True
        A, a_rows, a_cols = _csc_pattern(a_mask)

        q = np.zeros(nz)
        l = np.full(n_rows, -np.inf)
        u = np.full(n_rows, np.inf)

        prob = osqp.OSQP()
        settings = dict(
            verbose=False,
            eps_abs=self.nmpc.nmpc_osqp_eps,
            eps_rel=self.nmpc.nmpc_osqp_eps,
            max_iter=int(self.nmpc.nmpc_osqp_max_iter),
        )
        try:
            prob.setup(P, q, A, l, u, warm_starting=True, polishing=False,
                       **settings)
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
    # Prediction / linearisation
    # ------------------------------------------------------------------
    def _rollout(self, x0, U, ref):
        """
        Nonlinear forward simulation of the whole horizon from x0 under U.
        Returns X with shape (N+1, NX). Sequential by construction (each stage
        depends on the previous one), so this is the one part of a Gauss-Newton
        iteration that cannot be vectorised across stages — hence the scalar
        _step_scalar fast path (see its docstring: 1 ms here versus 17 ms
        through the vectorised form).

        nmpc_rk_substeps is SPEED-GATED per stage, same technique as
        nmpc_jac_substeps/nmpc_jac_gate_speed (see _jacobians). Measured
        directly (infinitesimal perturbation propagated through this same
        rollout, not just the sensitivity Jacobian, across all 8 states and
        several control-sequence shapes): 2 substeps diverges (up to ~260x
        growth) across roughly 2.25-3.75 m/s, but 3 is fully converged
        (<=1.6x growth) everywhere tested in and above that band. The fast
        value here is nmpc_rk_substeps_fast=3, not 2, specifically because 2
        is the one count confirmed unstable. Gated per-stage on that stage's
        OWN predicted v_x (unlike the Jacobian's single whole-horizon gate),
        since this function builds X incrementally and a stage's speed can
        cross the gate mid-horizon on a hard launch/brake.
        """
        N = self.N
        X = np.empty((N + 1, NX))
        X[0] = x0
        xk = [float(v) for v in x0]
        p, dt = self.plant, self.dt
        for k in range(N):
            n_sub = self.nmpc.nmpc_rk_substeps
            if xk[IDX_VX] >= self.nmpc.nmpc_rk_gate_speed:
                n_sub = min(n_sub, self.nmpc.nmpc_rk_substeps_fast)
            xk = _step_scalar(xk, U[k], ref, p, dt, n_sub)
            X[k + 1] = xk
        return X

    def _jacobians(self, X, U, ref):
        """
        One-step Jacobians A_k = d x_{k+1}/d x_k and B_k = d x_{k+1}/d u_k for
        every stage, by forward finite differences — vectorised over stages, so
        each perturbation direction costs ONE batched one-step integration of
        all N stages rather than N scalar ones (10 batched steps total).

        The substep count is SPEED-GATED. A_k/B_k only supply the QP's STEP
        DIRECTION and never the predicted trajectory, so a coarser sensitivity
        costs at most a slightly worse step that the trust region absorbs. That
        argument is sound about ACCURACY and wrong about STABILITY, which is
        what bit when this was a flat 1: the (v_y, r) sub-dynamics stiffen as
        1/v_x, so below roughly 3.5 m/s too few substeps make this integration
        divergent rather than inaccurate, and a divergent A_k does not degrade
        gracefully -- it compounds through the condensing loop and leaves a
        Hessian whose only representable solution is exactly zero.

        The instability is confined to LOW SPEED, so the fix does not have to
        be. Measured max|A_k| against the converged (4-substep) value:

            v_x    js=1      js=2      js=4
            2.5    2.41e2    7.00e1    1.00
            3.0    1.32e2    1.57e1    1.13
            5.0    2.25e1    1.48      1.98
            8.0    4.06      3.07      3.18
            14.0   4.01      4.90      4.93
            20.0   5.69      6.03      6.04

        At and above ~8 m/s two substeps track the converged value closely with
        no divergence, so nmpc_jac_substeps_fast is used there and the full
        nmpc_jac_substeps only below the gate. js=1 is NOT a safe fast value
        even at speed: it does not diverge, but it is badly inaccurate (1.30 vs
        a converged 3.85 at 10 m/s).

        An analytic Jacobian would NOT permit a lower count either. The
        variational equation propagated through RK4 has the same stability
        region as the nominal ODE (verified: both stay stable to lambda*h =
        -2.785 and both diverge at -3.0), so the substep floor is a property of
        RK4 sensitivity propagation, not of finite differencing.

        The gate keys off the SLOWEST stage in the predicted horizon, not the
        current speed, so it changes rarely -- a per-tick flip in Jacobian
        fidelity would perturb the warm start and become its own disturbance.
        Setting nmpc_jac_substeps_fast == nmpc_jac_substeps disables the gate
        exactly.
        """
        N = self.N
        Xs = X[:N]
        p, dt = self.plant, self.dt
        n_sub = max(1, int(self.nmpc.nmpc_jac_substeps))
        n_sub_fast = max(1, int(self.nmpc.nmpc_jac_substeps_fast))
        if float(X[:, IDX_VX].min()) >= self.nmpc.nmpc_jac_gate_speed:
            n_sub = min(n_sub, n_sub_fast)
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
        """
        Output Jacobians C_k = d h/d x at every stage (including the terminal
        one), same vectorised forward-difference scheme as _jacobians.

        When self.friction_circle_enabled, H0/C carry NH_FRICTION extra rows
        (F_yf, F_yr — see _outputs' docstring), riding along through this
        SAME finite-difference pass at no extra rollout cost. Shape is
        (stages, NH, NX) when the flag is False, IDENTICAL to before this
        feature existed.

        When self.progress_enabled, v_cap/s_target_N are threaded through to
        _outputs unchanged (see that function's docstring) -- perturbing
        X[:, IDX_S] here naturally captures h_prog's sensitivity to the
        terminal arc length through the SAME pass, no extra rollout.
        """
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
        """
        Stage 0's steering-effort weight for this tick: r_delta normally,
        scaled up while the car is slow, FADED OUT over a speed band rather
        than switched off at a threshold.

        At v_x = 0 steering cannot move the car (r_kin = v_x*tan(d)/L = 0,
        dynamic branch blended out), but the SQP minimises one cost summed
        over the whole horizon, and the predicted v_x leaves zero by stage 1.
        Nothing else in the objective distinguishes a stage that cannot act
        from one that will act shortly, so without this the optimiser
        pre-commits U[0] toward whatever helps the later stages and the car
        launches already steering. Keyed on the MEASURED speed (X[0] is x0),
        never a predicted one.

        The multiplier is held at nmpc_standstill_steer_r_scale below
        nmpc_standstill_speed, then ramped linearly to 1.0 (no damping) at
        nmpc_standstill_fade_speed. A hard release instead of a fade puts the
        full weight change into a single tick exactly when the car is most
        sensitive: measured live, steering ran from -1.8 to -12.9 deg over
        the six ticks immediately after the release, roughly -2 deg/tick
        sustained, which is the launch excursion this damping exists to
        prevent, reappearing a moment later. Setting fade_speed <= speed
        restores the old hard cutoff.

        Shared by _solve_step (which builds the QP) and _cost (which scores
        backtracking trials) so the two cannot disagree -- scoring a
        different objective from the one the QP minimised is exactly the
        failure mode _cost's own _Rr_flat comment below warns about.
        """
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
        """
        True (nonlinear) cost of a candidate trajectory — used only by the
        backtracking test, so it must match the QP's objective term for term:
        weighted output residuals with the terminal scale, input effort with
        the accel/brake split, input rate against u_prev, and the soft-track
        slack penalty at its optimal value for this trajectory (max(0, |e_y| -
        halfwidth), which is what the QP's slack would be).

        H may carry NH_FRICTION extra (unweighted) columns when
        friction_circle_enabled -- sliced down to the original NH cost rows
        here so w (len NH) always broadcasts correctly and the objective
        itself never includes the friction rows, per the feature's spec.
        """
        w = self.w_out
        H = H[:, :self.NH]
        stage = float(np.sum(w * H[:-1] ** 2)) + float(
            self.terminal_scale * np.sum(w * H[-1] ** 2))
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
        # vector the QP's Hessian (_ErE) is built from -- not the flat
        # self.r_rate. Any mechanism that reshapes the rate weight (corner
        # blend, anti-hunt, reversal penalty, stage ramp) writes _Rr_flat; if
        # this used self.r_rate the backtracking line search would score a
        # DIFFERENT objective from the one the QP minimised and could reject
        # genuinely improving steps. Falls back to the flat tile if absent.
        _rr = getattr(self, '_Rr_flat', None)
        if _rr is None or _rr.shape[0] != du.size:
            rate = float(np.sum(self.r_rate * du ** 2))
        else:
            rate = float(np.sum(_rr * du.reshape(-1) ** 2))
        jerk = 0.0
        if self._E2rE2 is not None:
            # Same objective the QP minimises (see _solve_step's jerk block).
            d2 = np.vstack([du[0] - (self._u_prev - self._u_prev2),
                            np.diff(du, axis=0)])
            jerk = float(np.sum(np.array([self.rjerk_delta, self.rjerk_a]) * d2 ** 2))
        slack = 0.0
        if self._use_slack:
            over = np.maximum(np.abs(X[1:, IDX_EY]) - self.nmpc.nmpc_track_halfwidth,
                              0.0)
            # Quadratic + linear, matching _solve_step's q[idx] line -- the
            # QP's own slack variable is driven to exactly this `over` at
            # the optimum (nothing else rewards it being larger), so scoring
            # it here as max(0, |e_y|-hw) is the true-cost equivalent of the
            # QP's slack decision variable, not an approximation of it.
            slack = float(self.nmpc.nmpc_slack_weight * np.sum(over ** 2)
                          + self.nmpc.nmpc_slack_linear_weight * np.sum(over))
        return stage + eff + rate + jerk + slack

    def _cost_breakdown(self, X, U, H, u_prev, u_prev2):
        """
        Same arithmetic as _cost(), split into named components instead of
        summed to one scalar. Debug-only (live_viz.py's weighted-error
        panel): called once per tick on the final accepted trajectory, never
        inside the backtracking loop, so it cannot affect which step gets
        accepted. Keep in sync with _cost() by hand if that method's terms
        ever change; duplicated rather than refactoring _cost() to return
        both, to avoid touching the hot backtracking path at all.

        u_prev/u_prev2 are taken as explicit arguments rather than read from
        self._u_prev/self._u_prev2: compute() calls this AFTER already
        advancing both (for the next tick's warm start), so reading the
        instance attributes here would score du/the jerk term against the
        wrong reference and silently distort the rate/jerk split -- pass
        the same u_prev/u_prev2 _cost()/_solve_step() actually solved
        against (compute()'s local, pre-advance copies).
        """
        w = self.w_out
        H = H[:, :self.NH]
        # The progress row is reported separately below rather than folded
        # into row 4, which would make a large progress residual masquerade
        # as a speed-tracking error in the display.
        out_terms = {
            'e_y':      float(w[0] * H[0, 0] ** 2),
            'e_yd':     float(w[1] * H[0, 1] ** 2),
            'e_psi':    float(w[2] * H[0, 2] ** 2),
            'yaw_rate': float(w[3] * H[0, 3] ** 2),
        }
        # Row 4's KEY follows the mode, matching mpc_controller.py's step-0
        # bar: 'e_v' is a two-sided speed error, 'v_cap_hinge' is the
        # one-sided cap penalty that sits at exactly 0 whenever the car is
        # under the cap. Same slot, different quantity, so the same name for
        # both would report "no speed error" on a lap where speed is not
        # being tracked at all.
        out_terms['v_cap_hinge' if self.progress_enabled else 'e_v'] = float(
            w[4] * H[0, 4] ** 2)
        if self.progress_enabled:
            # Scored at the TERMINAL stage (H[-1]), not stage 0: h_prog is
            # zero at every other stage by construction (see _outputs), so
            # reading H[0, 5] would always report exactly 0.
            out_terms['progress'] = float(w[5] * H[-1, 5] ** 2)
        stage = float(np.sum(w * H[:-1] ** 2)) + float(
            self.terminal_scale * np.sum(w * H[-1] ** 2))
        a = U[:, 1]
        eff = float(self.r_delta * np.sum(U[:, 0] ** 2)
                    + (self._r_delta_stage0(X) - self.r_delta) * U[0, 0] ** 2
                    + self.r_a_accel * np.sum(np.maximum(a, 0.0) ** 2)
                    + self.r_a_brake * np.sum(np.minimum(a, 0.0) ** 2))
        du = np.vstack([U[0] - u_prev, np.diff(U, axis=0)])
        _rr = getattr(self, '_Rr_flat', None)
        if _rr is None or _rr.shape[0] != du.size:
            rate = float(np.sum(self.r_rate * du ** 2))
        else:
            rate = float(np.sum(_rr * du.reshape(-1) ** 2))
        jerk = 0.0
        if self._E2rE2 is not None:
            d2 = np.vstack([du[0] - (u_prev - u_prev2),
                            np.diff(du, axis=0)])
            jerk = float(np.sum(np.array([self.rjerk_delta, self.rjerk_a]) * d2 ** 2))
        slack = 0.0
        if self._use_slack:
            over = np.maximum(np.abs(X[1:, IDX_EY]) - self.nmpc.nmpc_track_halfwidth,
                               0.0)
            slack = float(self.nmpc.nmpc_slack_weight * np.sum(over ** 2)
                          + self.nmpc.nmpc_slack_linear_weight * np.sum(over))

        # Per-term horizon-summed breakdown (unlike stage/eff/rate above,
        # which merge all 5 output terms / both inputs together) -- for
        # live_viz.py's horizon panel. _rr reshaped to (N, nu) rather than
        # flattened: np.tile(self.r_rate, N) interleaves [steer, accel] per
        # stage, so column 0/1 of the reshape is exactly steer/accel's own
        # per-stage weight, matching du's own (N, nu) column layout.
        # Row 4/5 names follow the mode, exactly as the step-0 breakdown
        # above does -- this tuple used to be hardcoded to the 5 tracking
        # rows, so in progress mode the horizon panel silently mislabelled
        # the cap hinge as 'e_v' AND omitted the progress row entirely.
        # Built from self.NH so it cannot fall out of step with w_out again.
        steer_names = (('e_y', 'e_yd', 'e_psi', 'yaw_rate', 'v_cap_hinge',
                        'progress') if self.progress_enabled
                       else ('e_y', 'e_yd', 'e_psi', 'yaw_rate', 'e_v'))
        horizon_terms = {
            name: float(w[i] * np.sum(H[:-1, i] ** 2)
                        + self.terminal_scale * w[i] * H[-1, i] ** 2)
            for i, name in enumerate(steer_names)
        }
        horizon_terms['steer_effort'] = float(
            self.r_delta * np.sum(U[:, 0] ** 2)
            + (self._r_delta_stage0(X) - self.r_delta) * U[0, 0] ** 2)
        horizon_terms['accel_effort'] = float(
            self.r_a_accel * np.sum(np.maximum(a, 0.0) ** 2)
            + self.r_a_brake * np.sum(np.minimum(a, 0.0) ** 2))
        if _rr is None or _rr.shape[0] != du.size:
            rate_cols = self.r_rate[None, :] * du ** 2
        else:
            rate_cols = _rr.reshape(du.shape) * du ** 2
        horizon_terms['delta_u_steer'] = float(np.sum(rate_cols[:, 0]))
        horizon_terms['delta_u_accel'] = float(np.sum(rate_cols[:, 1]))

        return {
            'step0_terms': out_terms,     # step-0 output errors, for the per-error bar graph
            'stage_total': stage,         # full-horizon output-tracking cost
            'effort_total': eff,          # full-horizon steering/accel effort cost
            'rate_total': rate,           # full-horizon input rate-of-change cost
            'jerk_total': jerk,           # full-horizon input jerk cost (0.0 if disabled)
            'slack_total': slack,         # full-horizon soft-boundary slack cost (0.0 if disabled)
            'horizon_terms': horizon_terms,  # per-term horizon sums, for live_viz.py's horizon panel
        }
