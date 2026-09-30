"""
fsae_control/nmpc/solver.py — `NMPCController`

Construction, `reset`, static-path handling, `compute` (the per-tick entry point)
and the path-reference helpers. The QP structure and cost live in qp_model.py
(`_QPModelMixin`), the SQP step in sqp_step.py (`_SQPStepMixin`); this class
inherits both, so every method keeps its name and `self` state.
"""

import math
import time

import numpy as np

from fsae_control.mpc.mpc_core import (
    MAX_ACCEL,
    MAX_BRAKE,
    MAX_STEER_RAD,
    _steer_rate_anti_hunt,
    _corner_factor,
    _blend,
    _reversal_penalty_boost,
)
from fsae_control.mpc.mpc_params import DEFAULT_MPC_PARAMS, MPCParams
from fsae_control.mpc.nmpc_params import DEFAULT_NMPC_PARAMS, NMPCParams

try:
    import osqp
except ImportError as _exc:      # pragma: no cover - see package.xml
    osqp = None
    _OSQP_IMPORT_ERROR = _exc

from fsae_control.nmpc.layout import (
    IDX_EPSI,
    IDX_EY,
    IDX_R,
    IDX_S,
    IDX_VX,
    NH_PROGRESS,
    NH_TRACKING,
    NU,
    NX,
)
from fsae_control.nmpc.reference import PathReference
from fsae_control.nmpc.dynamics import _Plant, _step_scalar
from fsae_control.nmpc.outputs import _outputs
from fsae_control.nmpc.weight_schedule import (
    _rrate_stage_ramp,
    _rrate_zone_scale,
)
from fsae_control.nmpc.qp_model import _QPModelMixin
from fsae_control.nmpc.sqp_step import _SQPStepMixin


class NMPCController(_QPModelMixin, _SQPStepMixin):
    """
    Frenet-frame nonlinear MPC, drop-in compatible with
    mpc_core.MPCController's node-facing surface: compute(), reset(),
    set_static_path(), set_heading_profile(), last_telemetry, a_max,
    a_max_brake.
    """

    def __init__(
        self,
        dt: float = 0.05,
        N: int | None = None,
        params: MPCParams | None = None,
        nmpc: NMPCParams | None = None,
        logger=None,
    ) -> None:
        """
        dt must equal the calling node's control period (0.05 s), as for
        MPCController. N defaults to NMPCParams.nmpc_horizon (35, matching
        MPCController's own horizon so horizon length is never a confound when
        comparing the two); an explicit N overrides it. `logger` is an optional
        rclpy logger for the one-time informational messages; print() is used
        when it is None so this module stays importable/testable outside ROS.
        """
        if osqp is None:      # pragma: no cover - dependency guard
            raise ImportError(
                'nmpc_core requires osqp (already a documented dependency of '
                f'mpc_core via cvxpy — see package.xml): {_OSQP_IMPORT_ERROR!r}'
            )

        self.dt = float(dt)
        self.params = params if params is not None else DEFAULT_MPC_PARAMS
        self.nmpc = nmpc if nmpc is not None else DEFAULT_NMPC_PARAMS
        self._logger = logger
        self.N = int(self.nmpc.nmpc_horizon if N is None else N)
        if self.N < 2:
            raise ValueError(f'NMPC horizon must be >= 2 (got {self.N})')

        self.nx, self.nu = NX, NU
        nm0 = nmpc if nmpc is not None else DEFAULT_NMPC_PARAMS
        # alat_ceiling_flat/_slope/_intercept are FSDS's measured plant
        # constant (the sustained lateral-accel ceiling law), not a tuning
        # weight -- MPCParams no longer carries them (removed along with the
        # whole corner_demand/lookahead-gain-scheduling family they used to
        # parameterise), so _Plant's own hardcoded defaults (7.5/0.47/2.46,
        # the same measured values) are used directly here, the same
        # precedent this class already follows for lf/lr/m/Iz/Cf/Cr. Only
        # nmpc_alat_ceiling_enabled (a genuine NMPC-only on/off switch, not a
        # shared plant constant) still comes from NMPCParams.
        self.plant = _Plant(alat_ceiling_enabled=bool(nm0.nmpc_alat_ceiling_enabled))
        # Convenience aliases so telemetry/geometry code reads like mpc_core's.
        self.lf, self.lr = self.plant.lf, self.plant.lr

        # ── Experimental feature flags (see nmpc_params.py's comments) ───
        self.spline_reference_enabled = bool(nm0.nmpc_spline_reference_enabled)
        self.friction_circle_enabled = bool(nm0.nmpc_friction_circle_enabled)
        # See NH_TRACKING/NH_PROGRESS and _outputs' docstring. Fixed for the
        # controller's lifetime (read once here), never toggled per-tick --
        # w_out's length and _build_qp's fixed sparsity both depend on it.
        self.progress_enabled = bool(nm0.nmpc_progress_enabled)
        self.NH = NH_PROGRESS if self.progress_enabled else NH_TRACKING
        self.progress_reach = float(nm0.nmpc_progress_reach)
        self.progress_v_min = float(nm0.nmpc_progress_v_min)
        if self.friction_circle_enabled:
            # F_max = m * ceiling(v_x) / 2 per axle: the measured ceiling law
            # bounds TOTAL lateral force (F_yf*cos(d) + F_yr) / m, split
            # evenly across the two axles as a simple, symmetric per-axle
            # cap (the soft mechanism in _f/_f_scalar scales both axles by
            # the SAME factor too, so this keeps the same even-split
            # convention rather than inventing a front/rear bias). This is a
            # HARD, ADDITIONAL bound alongside (not instead of) that
            # existing soft tanh saturation.
            self._fmax_flat = 0.5 * self.plant.m * self.plant.alat_ceiling_flat
            self._fmax_slope = 0.5 * self.plant.m * self.plant.alat_ceiling_slope
            self._fmax_intercept = 0.5 * self.plant.m * self.plant.alat_ceiling_intercept

        # ── Limits: identical to MPCController's ────────────────────────
        self.a_max = MAX_ACCEL
        self.a_max_brake = MAX_BRAKE
        self.u_min = np.array([-MAX_STEER_RAD, -self.a_max_brake])
        self.u_max = np.array([MAX_STEER_RAD, self.a_max])
        # 180 deg/s * dt, matching MPCController's du_max exactly (see that
        # constructor's long note on why it is 180 and not 190).
        self.du_max = np.array([math.radians(180.0) * self.dt, 0.6])

        # ── Weights: MPCParams, with per-NMPC overrides ─────────────────
        # The override fields (nmpc_q_e_y, ...) live in MPCParams itself
        # (see mpc_params.py's "NMPC weight overrides" section), alongside the base
        # fields they inherit from when left at -1.0. NMPCParams (self.nmpc)
        # no longer carries any weight field at all — only structural/
        # solver settings.
        def _pick(override, inherited):
            return float(inherited if override is None or override < 0.0 else override)

        pm = self.params
        # In progress mode nmpc_q_e_v weights the speed-CAP hinge (row 4,
        # see _outputs), not a two-sided target-tracking error -- same slot,
        # different regressor, so expect it to need its own value rather
        # than inheriting the tracking-mode tuned q_e_v unchanged.
        w_out = [
            _pick(pm.nmpc_q_e_y,      pm.q_e_y),
            _pick(pm.nmpc_q_e_yd,     pm.q_e_yd),
            _pick(pm.nmpc_q_e_psi,    pm.q_e_psi),
            _pick(pm.nmpc_q_epsi_dot, pm.q_r),
            _pick(pm.nmpc_q_e_v,      pm.q_e_v),
        ]
        if self.progress_enabled:
            w_out.append(float(pm.nmpc_q_progress))
        self.w_out = np.array(w_out)
        self.r_delta = _pick(pm.nmpc_r_delta, pm.r_delta)
        self.r_a_accel = _pick(pm.nmpc_r_a_accel, pm.r_a_accel)
        self.r_a_brake = _pick(pm.nmpc_r_a_brake, pm.r_a_brake)
        self.r_rate = np.array([
            _pick(pm.nmpc_r_rate_delta, pm.r_rate_delta),
            _pick(pm.nmpc_r_rate_a, pm.r_rate_a),
        ])
        self.terminal_scale = _pick(pm.nmpc_terminal_scale, pm.terminal_q_scale)

        # Independent of steer_rate_anti_hunt_enabled (the LTV-QP's own
        # flag) — see nmpc_params.py/mpc_params.py's field comments and the
        # module docstring's "WHAT THIS CONTROLLER DELIBERATELY DOES NOT DO"
        # for why this is opt-in and separately defaulted False.
        self.steer_rate_anti_hunt_enabled = bool(pm.nmpc_steer_rate_anti_hunt_enabled)
        self.anti_hunt_boost_max = _pick(pm.nmpc_anti_hunt_boost_max, pm.anti_hunt_boost_max)

        # Composes with EITHER of the two flags above (not an alternative to
        # either) -- see mpc_params.py's nmpc_reversal_penalty_enabled field
        # comment for why this one's signal (last tick's actual steering)
        # doesn't double-count with curvature/e_y/e_psi-keyed mechanisms.
        self.reversal_penalty_enabled = bool(pm.nmpc_reversal_penalty_enabled)
        self.reversal_penalty_boost_max = _pick(
            pm.nmpc_reversal_penalty_boost_max, pm.reversal_penalty_boost_max)
        self.reversal_penalty_k = _pick(pm.nmpc_reversal_penalty_k, pm.reversal_penalty_k)

        # EXPERIMENTAL, default off. Discounts the steering-rate cost at the
        # NEAR horizon stages -- see _rrate_stage_ramp's docstring, including
        # why it is NOT a fix for the shallow-corner jerk. Composes with the
        # flags above: they set the rate weight's MAGNITUDE, this shapes it
        # across STAGES.
        self.rrate_stage_ramp_enabled = bool(pm.nmpc_rrate_stage_ramp_enabled)
        self.rrate_stage_near = float(pm.nmpc_rrate_stage_near)

        # EXPERIMENTAL, default off. Continuous three-zone schedule on the
        # steering-rate cost: boost on a true straight, ease on the approach
        # to a corner the HORIZON can see, floor through the corner itself.
        # See _rrate_zone_scale. MULTIPLIES r_rate_delta (unlike the corner
        # blend, which overwrites it).
        self.rrate_zone_enabled = bool(pm.nmpc_rrate_zone_enabled)
        self.rrate_zone_boost_straight = float(pm.nmpc_rrate_zone_boost_straight)
        self.rrate_zone_ease_approach = float(pm.nmpc_rrate_zone_ease_approach)
        self.rrate_zone_floor_corner = float(pm.nmpc_rrate_zone_floor_corner)
        # Steering/accel JERK weights (second difference of the input). 0.0
        # (default) disables the term entirely -- see _build_qp's _E2 comment.
        self.rjerk_delta = float(pm.nmpc_rjerk_delta)
        self.rjerk_a = float(pm.nmpc_rjerk_a)

        # Alternative to the above, not a composition with it -- see
        # mpc_params.py's nmpc_corner_rrate_blend_enabled field comment.
        self.corner_rrate_blend_enabled = bool(pm.nmpc_corner_rrate_blend_enabled)
        self.corner_factor_k = _pick(pm.nmpc_corner_factor_k, pm.corner_factor_k)
        self.rrate_steer_straight = _pick(pm.nmpc_rrate_steer_straight, pm.rrate_steer_straight)
        self.rrate_steer_corner = _pick(pm.nmpc_rrate_steer_corner, pm.rrate_steer_corner)

        # Standstill steering damping -- see the field docstrings in
        # nmpc_params.py for the mechanism, and _solve_step/_cost for where
        # the stage-0 weight is actually applied.
        self.standstill_steer_damp_enabled = bool(
            self.nmpc.nmpc_standstill_steer_damp_enabled)
        self.standstill_speed = float(self.nmpc.nmpc_standstill_speed)
        self.standstill_fade_speed = float(self.nmpc.nmpc_standstill_fade_speed)
        self.standstill_steer_r_scale = float(
            self.nmpc.nmpc_standstill_steer_r_scale)

        # ── Continuity memory (mirrors MPCController's) ─────────────────
        self._delta_act = 0.0
        self._a_act = 0.0
        self._u_prev = np.zeros(NU)
        self._u_prev2 = np.zeros(NU)  # command two ticks ago, for the jerk anchor
        self._v_des_filtered: float | None = None
        self._U = np.zeros((self.N, NU))     # warm-start input trajectory
        self._have_warm_start = False
        self._u_history: list[np.ndarray] = []
        self._pose_age_filtered: float | None = None
        self._n_delay = 0

        self._ref: PathReference | None = None
        self._ref_signature = None
        self._static_ref: PathReference | None = None
        self._heading_profile_warned = False

        self.last_telemetry: dict = {}
        self._qp = None
        self._build_qp()

    # ------------------------------------------------------------------
    # Node-facing hooks (same names/semantics as MPCController's)
    # ------------------------------------------------------------------
    def _log(self, msg: str) -> None:
        if self._logger is not None:      # pragma: no cover - ROS path
            self._logger.info(msg)
        else:
            print(f'[nmpc_core] {msg}')

    def set_static_path(self, path) -> None:
        """
        Precompute the arc-length/curvature reference for a fixed path, once,
        at load time. Called by the owning node exactly where it calls
        MPCController.set_static_path(); compute() never builds this itself for
        a static path (it looks the cached one up by signature).

        path=None clears it (live-planner mode), in which case compute() builds
        a reference per tick from whatever path it is handed.
        """
        if path is None or len(np.asarray(path)) < 3:
            self._static_ref = None
            return
        try:
            self._static_ref = PathReference(
                path,
                dense_step=self.nmpc.nmpc_curvature_dense_step,
                smooth_w=self.nmpc.nmpc_curvature_smooth_w,
                kappa_clip=self.nmpc.nmpc_kappa_clip,
                spline_reference_enabled=self.spline_reference_enabled,
            )
            k = self._static_ref.kappa
            self._log(
                f'NMPC path reference built: {self._static_ref.total:.1f} m, '
                f'{len(k)} curvature samples, |kappa| max {np.abs(k).max():.4f} 1/m.'
            )
        except (ValueError, IndexError) as exc:   # pragma: no cover - defensive
            self._log(f'PathReference build failed ({exc}); will rebuild per tick.')
            self._static_ref = None

    def set_heading_profile(self, psi_target) -> None:
        """
        Accepted and IGNORED — see the module docstring. The shaped
        heading-lead profile (Part 8/9) is a workaround for the missing
        curvature term this controller models directly, so applying both would
        double-count the anticipation. Logged once so a launch config that
        enables use_precomputed_heading_profile alongside use_nmpc does not
        silently do nothing.
        """
        if psi_target is not None and not self._heading_profile_warned:
            self._heading_profile_warned = True
            self._log(
                'use_precomputed_heading_profile is IGNORED by the NMPC: its '
                'Frenet model already carries the path curvature the shaped '
                'heading lead exists to approximate.'
            )

    def reset(self) -> None:
        """Clear continuity state, as MPCController.reset() does."""
        self._delta_act = 0.0
        self._a_act = 0.0
        self._u_prev = np.zeros(NU)
        self._u_prev2 = np.zeros(NU)  # command two ticks ago, for the jerk anchor
        self._v_des_filtered = None
        self._U = np.zeros((self.N, NU))
        self._have_warm_start = False
        self._u_history.clear()
        self._pose_age_filtered = None
        self._n_delay = 0
        # _ref/_ref_signature themselves already carry the "previous tick's
        # kappa profile" _rate_limit_kappa needs, via _path_reference; clear
        # them too so a reset doesn't rate-limit the NEXT path's kappa
        # against a profile from before the reset.
        self._ref = None
        self._ref_signature = None

    # ------------------------------------------------------------------
    # Delay compensation
    # ------------------------------------------------------------------
    def _update_n_delay(self, pose_age_s: float) -> int:
        """
        Filtered, hysteresis-gated rollforward depth from a measured pose age.

        Deliberate duplicate of MPCController._update_n_delay (same MPCParams
        fields, same arithmetic): that method is bound to the LTV-QP
        controller's own state, and refactoring it out would mean editing
        mpc_core.py's live solve path, which this feature is specifically
        designed not to touch. Keep the two in sync if either changes.
        """
        age = max(0.0, float(pose_age_s))
        cap = self.params.max_delay_compensation_steps
        if self._pose_age_filtered is None:
            self._pose_age_filtered = age
            self._n_delay = int(np.clip(round(age / self.dt), 0, cap))
            return self._n_delay
        self._pose_age_filtered += (
            self.params.pose_age_lp_alpha * (age - self._pose_age_filtered))
        steps_f = self._pose_age_filtered / self.dt
        if abs(steps_f - self._n_delay) > 0.5 + self.params.n_delay_hysteresis:
            self._n_delay = int(np.clip(round(steps_f), 0, cap))
        return self._n_delay

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------
    def compute(
        self,
        path: np.ndarray,
        car_pos: np.ndarray,
        car_yaw: float,
        car_speed: float,
        desired_speed: float,
        car_yaw_rate: float = 0.0,
        pose_age_s: float = 0.0,
        car_vy: float = 0.0,
    ) -> tuple[float, float, float]:
        """
        Run one NMPC control step. Signature, units, return convention
        ((steering in [-1,1], throttle in [0,1], brake in [0,1])) and the
        short-path guard are all identical to MPCController.compute(), so the
        calling nodes need no branch beyond which object they construct.
        """
        if path is None or len(path) < 3:
            return 0.0, 0.0, 0.5

        t0 = time.perf_counter()

        # Same first-order target-speed filter as MPCController.compute().
        # Now a tunable launch param (nmpc_v_des_filter_alpha) rather than a
        # hardcoded constant -- see that field's own docstring in
        # nmpc_params.py for the mechanism and the 2026-09-15 live-tuning
        # history (raising it has repeatedly made overall performance
        # worse, not better; see planner_only_lap2_corner_spinout.md).
        alpha = float(self.nmpc.nmpc_v_des_filter_alpha)
        if self._v_des_filtered is None:
            self._v_des_filtered = desired_speed
        self._v_des_filtered += alpha * (desired_speed - self._v_des_filtered)
        v_ref = float(self._v_des_filtered)

        ref = self._path_reference(path)
        if ref is None or ref.total < 1e-3:
            return 0.0, 0.0, 0.5

        # ── Measured Frenet state ──────────────────────────────────────
        fa = np.asarray(car_pos, dtype=float) + self.lf * np.array(
            [math.cos(car_yaw), math.sin(car_yaw)])
        s0, e_y, e_psi, base_idx, path_yaw = ref.project(fa, car_yaw)
        x0 = np.array([
            s0, e_y, e_psi,
            max(float(car_speed), 0.0), float(car_vy), float(car_yaw_rate),
            self._delta_act, self._a_act,
        ])

        # ── Corner-blend / anti-hunt (EXPERIMENTAL, default off — see module
        # docstring) — ALTERNATIVES, not composed: the corner-factor blend
        # takes priority when both are enabled (skips anti-hunt entirely in
        # that case). Same signals/functions as the LTV-QP path (imported
        # verbatim from mpc_core, not reimplemented). Computed once per
        # compute() call (this tick's measured state), and applied UNIFORMLY
        # across the whole horizon for this tick's solve — not a function of
        # horizon step, so neither schedules a future obligation the way the
        # deleted lookahead family did. self._Rr_flat/self._ErE are
        # ordinarily fixed at _build_qp() time; when both flags are off
        # (default), neither is touched here, so behaviour is byte-identical
        # to before either feature existed.
        kappa_now = float(ref.kappa_at(np.array([s0]))[0])
        m_rrate_antihunt = 1.0
        # Always computed (not gated behind corner_rrate_blend_enabled) --
        # this is a general current-curvature signal other mechanisms also
        # key off via last_telemetry, independent of whether the R_rate
        # weight-blend feature itself is active.
        corner_frac = _corner_factor(kappa_now, self.corner_factor_k)
        rrate_steer_corner_blend = float(self.r_rate[0])
        # Tracks R_rate[0,0]'s running value through the if/elif AND the
        # reversal-penalty composition below, so the reversal penalty (which
        # applies regardless of which branch ran) boosts whatever value is
        # actually current rather than always the pre-if/elif base -- the
        # same silent-discard bug already found and fixed twice tonight in
        # mpc_core.py's own corner-blend/anti-hunt composition.
        rrate_steer_current = rrate_steer_corner_blend
        if self.corner_rrate_blend_enabled:
            rrate_steer_corner_blend = _blend(
                self.rrate_steer_straight, self.rrate_steer_corner, corner_frac)
            rrate_steer_current = rrate_steer_corner_blend
        elif self.steer_rate_anti_hunt_enabled:
            R2 = _steer_rate_anti_hunt(
                kappa_now, e_y, np.diag(self.r_rate), True,
                e_psi=e_psi, boost_max=self.anti_hunt_boost_max,
            )
            m_rrate_antihunt = float(R2[0, 0] / self.r_rate[0]) if self.r_rate[0] else 1.0
            rrate_steer_current = float(R2[0, 0])

        m_rrate_reversal = 1.0
        if self.reversal_penalty_enabled:
            R3 = _reversal_penalty_boost(
                float(self._u_prev[0]), np.diag([rrate_steer_current, self.r_rate[1]]),
                True, boost_max=self.reversal_penalty_boost_max, k=self.reversal_penalty_k,
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

        # ── Delay compensation (nonlinear rollforward) ──────────────────
        # Same trigger/depth logic as the LTV-QP path, but rolled forward
        # through the NONLINEAR model instead of predict_ahead()'s linear
        # Ad/Bd — the model is right here, so there is no reason to use a
        # linearisation for it.
        if self.params.delay_compensation_enabled:
            n_delay = self._update_n_delay(pose_age_s)
            if n_delay > 0 and self._u_history:
                xk = [float(v) for v in x0]
                for u_hist in self._u_history[-n_delay:]:
                    xk = _step_scalar(xk, u_hist, ref, self.plant, self.dt,
                                      self.nmpc.nmpc_rk_substeps)
                x0 = np.array(xk)
        else:
            n_delay = 0

        # ── Latency compensation (nonlinear rollforward, EXPERIMENTAL) ───
        # Same mechanism as delay compensation above, but forward past "now"
        # instead of backward from a stale pose: the solve about to run
        # takes real wall-clock time, so u[0] does not land until roughly
        # nmpc_latency_compensation_ms after x0 was measured. Rolled forward
        # holding the LAST APPLIED command constant (self._u_prev), not a
        # guessed future command, since the future command is exactly what
        # this solve is trying to determine. See nmpc_params.py's field
        # docstring for why this is off by default.
        n_latency = 0
        if self.nmpc.nmpc_latency_compensation_enabled:
            cap = self.params.max_delay_compensation_steps
            # math.floor(x + 0.5), not round(): Python's round() is
            # round-half-to-even, so exactly 25 ms at dt=0.05 s (0.5 steps)
            # rounds to 0, silently disabling this at its own default value
            # -- caught live 2026-09-15 (n_latency logged 0 for an entire
            # run at the 25 ms default).
            n_latency = int(np.clip(
                math.floor(self.nmpc.nmpc_latency_compensation_ms * 1e-3 / self.dt + 0.5),
                0, cap))
            if n_latency > 0:
                xk = [float(v) for v in x0]
                for _ in range(n_latency):
                    xk = _step_scalar(xk, self._u_prev, ref, self.plant, self.dt,
                                      self.nmpc.nmpc_rk_substeps)
                x0 = np.array(xk)

        # ── Warm start: shift the previous solution one step ────────────
        if self._have_warm_start:
            U = np.vstack([self._U[1:], self._U[-1:]])
        else:
            U = np.tile(self._u_prev, (self.N, 1))
        U = self._project_feasible(U)

        # ── Gauss-Newton SQP ───────────────────────────────────────────
        budget_s = self.nmpc.nmpc_solve_budget_ms * 1e-3
        X = self._rollout(x0, U, ref)

        # Three-zone rate schedule (rrate_zone_enabled). Applied HERE, not in
        # the block above, because it needs kappa across the PREDICTED horizon,
        # which only exists once X has been rolled out.
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
        # Progress mode: v_ref becomes a CAP (row 4 of h(), see _outputs)
        # rather than a two-sided target, and s_target_N is an UNREACHABLE
        # arc-length goal that turns "maximise s" into a GN-native
        # least-squares residual -- see _outputs' docstring for why a bare
        # linear reward is unsafe for this solver. Computed ONCE per tick
        # (not per backtracking trial): it depends only on X[0]/v_ref, both
        # fixed for this whole compute() call.
        #
        # The unreachable gap is v_cap*N*dt*progress_reach, EXCEPT it is
        # floored at a KINEMATIC distance (0.5*a_max*(N*dt)^2*progress_reach,
        # the horizon's own maximum reach under full commanded accel from
        # rest) rather than scaled off v_cap alone. This matters specifically
        # at launch: SPEED_TARGET_DEFICIT_MAX holds v_cap low while the car
        # is still stationary, which on its own makes the gap a few metres --
        # too close for the residual to stay "unreachable" once a_cmd
        # approaches what static friction needs, so the reward saturates and
        # the car never launches (measured offline: a_cmd plateaus ~1.3,
        # v_x stays exactly 0 forever). The floor is a property of the
        # CONTROLLER's own known authority (u_max[1]), not the plant's
        # stiction constant, deliberately: a real car's launch friction is
        # not something the cost design should need to know.
        #
        # Anchored on X[0, IDX_S], NOT the raw projected s0: when delay
        # compensation rolled x0 forward above, the rollout starts further
        # along the path than s0 measured, and anchoring to the stale s0
        # shrinks the gap by exactly that rollforward distance.
        v_cap = v_ref
        s_target_N = None
        if self.progress_enabled:
            gap = v_cap * self.N * self.dt * self.progress_reach
            kinematic_floor = (0.5 * self.u_max[1] * (self.N * self.dt) ** 2
                               * self.progress_reach)
            s_target_N = float(X[0, IDX_S]) + max(gap, kinematic_floor)

        H = _outputs(X, ref, self.plant, v_ref,
                     friction_circle_enabled=self.friction_circle_enabled,
                     progress_enabled=self.progress_enabled,
                     v_cap=v_cap, s_target_N=s_target_N,
                     progress_v_min=self.progress_v_min)
        cost = self._cost(X, U, H)
        iters = 0
        status = 'warm-start-only'
        for _ in range(max(1, int(self.nmpc.nmpc_sqp_iters))):
            if time.perf_counter() - t0 > budget_s:
                status = 'budget'
                break
            dU, status = self._solve_step(X, U, ref, v_ref,
                                          v_cap=v_cap, s_target_N=s_target_N)
            if dU is None:
                break
            step = 1.0
            accepted = False
            for _bt in range(max(0, int(self.nmpc.nmpc_backtrack_max)) + 1):
                U_try = np.clip(U + step * dU, self.u_min, self.u_max)
                X_try = self._rollout(x0, U_try, ref)
                H_try = _outputs(X_try, ref, self.plant, v_ref,
                                 friction_circle_enabled=self.friction_circle_enabled,
                                 progress_enabled=self.progress_enabled,
                                 v_cap=v_cap, s_target_N=s_target_N,
                                 progress_v_min=self.progress_v_min)
                cost_try = self._cost(X_try, U_try, H_try)
                # ONLY a genuine improvement is accepted. Accepting the
                # smallest trial regardless (an earlier version of this loop
                # did, "so a tick always makes progress") means a bad search
                # direction is written into the warm start and carried into the
                # next tick — which is how a single failed subproblem grew into
                # a divergent wrong-way full-lock ramp in offline testing.
                # Rejecting leaves U at the previous tick's shifted solution,
                # which is always feasible and always sane.
                if cost_try <= cost:
                    U, X, H, cost = U_try, X_try, H_try, cost_try
                    accepted = True
                    break
                step *= 0.5
            iters += 1
            if not accepted:
                status = 'rejected'
                break
            # U + Delta is slew-feasible by construction (Delta = 0 is feasible
            # and the QP's rate rows are linear, so any fraction of an accepted
            # step is too), and the clip above cannot break that when both
            # endpoints already respect the input bounds.

        self._U = U
        self._have_warm_start = True

        # ── Ship u[0], hard-limited exactly as the LTV-QP path is ──────
        u_opt = np.clip(U[0], self.u_min, self.u_max)
        u_opt = np.clip(u_opt, self._u_prev - self.du_max, self._u_prev + self.du_max)
        self._u_history.append(u_opt.copy())
        if len(self._u_history) > self.params.max_delay_compensation_steps:
            del self._u_history[:-self.params.max_delay_compensation_steps]

        exp_delta = math.exp(-self.dt / self.plant.tau_delta)
        exp_a = math.exp(-self.dt / self.plant.tau_a)
        self._delta_act = self._delta_act * exp_delta + u_opt[0] * (1.0 - exp_delta)
        self._a_act = self._a_act * exp_a + u_opt[1] * (1.0 - exp_a)
        # Captured before self._u_prev/self._u_prev2 advance below -- these
        # are the exact values _cost()/_solve_step() solved against this
        # tick, needed by _cost_breakdown() afterward (see its docstring).
        u_prev_for_breakdown, u_prev2_for_breakdown = self._u_prev, self._u_prev2
        self._u_prev2 = self._u_prev.copy()
        self._u_prev = u_opt.copy()

        delta_cmd = float(np.clip(u_opt[0], -MAX_STEER_RAD, MAX_STEER_RAD))
        a_cmd = float(u_opt[1])
        steering = float(np.clip(-delta_cmd / MAX_STEER_RAD, -1.0, 1.0))
        if a_cmd >= 0.0:
            throttle = float(np.clip(a_cmd / self.a_max, 0.0, 1.0))
            brake = 0.0
        else:
            throttle = 0.0
            brake = float(np.clip(-a_cmd / self.a_max_brake, 0.0, 1.0))

        solve_ms = (time.perf_counter() - t0) * 1e3
        cost_breakdown = self._cost_breakdown(
            X, U, H, u_prev_for_breakdown, u_prev2_for_breakdown)

        # Telemetry: the keys mpc_core publishes keep their exact meaning (so
        # every existing offline analysis script and telemetry_logger column
        # still works), plus nmpc_* diagnostics. kappa/kappa_max_abs are
        # recomputed here from the SAME kappa(s) reference the prediction used
        # — kappa at the car, and peak |kappa| over the horizon's own predicted
        # arc length, which is the NMPC's natural analogue of the LTV-QP's
        # speed-scaled lookahead window.
        kap_horizon = ref.kappa_at(X[:, IDX_S])
        self.last_telemetry = {
            'e_y': float(e_y),
            'e_psi': float(e_psi),
            'e_v': float(car_speed - v_ref),
            'kappa': kappa_now,
            'base_idx': int(base_idx),
            'kappa_max_abs': float(np.abs(kap_horizon).max()),
            'm_Rrate_antihunt': m_rrate_antihunt,
            'm_Rrate_reversal': m_rrate_reversal,
            'm_Rrate_zone': m_rrate_zone,
            'corner_frac': corner_frac,
            # The FINAL, fully-composed R_rate[0,0] actually used this tick
            # -- same semantic as mpc_core.py's own Rrate_steer_corner_blend
            # column (post corner-blend AND anti-hunt AND reversal-penalty),
            # not just the corner-blend stage in isolation.
            'Rrate_steer_corner_blend': rrate_steer_current,
            'pose_age_s': float(pose_age_s),
            'n_delay': int(n_delay),
            'n_latency': int(n_latency),
            'solve_ms': float(solve_ms),
            'car_speed': float(car_speed),
            'desired_speed': float(v_ref),
            'steering': steering,
            'throttle': throttle,
            'brake': brake,
            'delta_cmd': delta_cmd,
            'a_cmd': a_cmd,
            'delta_act': float(self._delta_act),
            'a_act': float(self._a_act),
            # e_yd/yaw_rate: raw step-0 output-error terms with no equivalent
            # top-level key elsewhere in this dict (e_y/e_psi/e_v already
            # exist above) -- see _cost_breakdown()'s step0_terms for the
            # weighted version. e_yd read from H (the same output vector the
            # cost is built from), not X, since it has no dedicated state.
            'e_yd': float(H[0, 1]),
            'yaw_rate': float(X[0, IDX_R]),
            # Actual step-0 rate-of-change the rate cost penalised this tick
            # (this tick's issued command vs the previous one), matching
            # mpc_core.py's delta_u_steer/delta_u_accel definition exactly.
            'delta_u_steer': float(self._u_prev[0] - self._u_prev2[0]),
            'delta_u_accel': float(self._u_prev[1] - self._u_prev2[1]),
            # Debug-only weighted breakdown for live_viz.py's bar graph, see
            # _cost_breakdown()'s own docstring for what each key means.
            'cost_breakdown': cost_breakdown,
            # NMPC-specific: see telemetry_logger.NMPC_COLUMNS.
            'nmpc_iters': int(iters),
            'nmpc_cost': float(cost),
            'total_cost': float(cost),  # alias: same key mpc_core.py uses
            'nmpc_status': 1.0 if status.lower().startswith('solved') else 0.0,
            'nmpc_s0': float(s0),
            'nmpc_kappa_horizon_end': float(kap_horizon[-1]),
            'nmpc_pred_ey_end': float(X[-1, IDX_EY]),
            'nmpc_pred_epsi_end': float(X[-1, IDX_EPSI]),
            'nmpc_pred_ey_max_abs': float(np.abs(X[:, IDX_EY]).max()),
            # Full predicted horizon in Cartesian (x, y), for live
            # visualisation only (live_viz.py): not logged to CSV (that's
            # what the scalar nmpc_pred_* summaries above are for), and
            # cheap relative to the solve itself, one xy_at() call over N
            # points already computed by the rollout.
            'nmpc_pred_xy': ref.xy_at(X[:, IDX_S], X[:, IDX_EY]),
        }
        if self.friction_circle_enabled:
            # H's two extra (unweighted) rows -- realized per-axle force at
            # the FINAL accepted trajectory, see _outputs' docstring.
            self.last_telemetry['nmpc_fyf_max_abs'] = float(np.abs(H[:, self.NH]).max())
            self.last_telemetry['nmpc_fyr_max_abs'] = float(np.abs(H[:, self.NH + 1]).max())
        if self.progress_enabled:
            # nmpc_speed_cap_over distinguishes "car chose to go slower than
            # the cap" (0.0) from "cap is binding" (>0), the single most
            # useful signal for whether the cap or the progress reward is in
            # charge on a given tick. nmpc_s_target_gap_end is how far short
            # of the (deliberately unreachable) target the horizon ends: a
            # stuck or regressing solve shows this GROWING tick over tick,
            # not shrinking, which is what the offline launch stall looked
            # like before it was fixed.
            self.last_telemetry['nmpc_v_cap'] = float(v_cap)
            self.last_telemetry['nmpc_speed_cap_over'] = float(
                max(0.0, float(X[0, IDX_VX]) - v_cap))
            self.last_telemetry['nmpc_s_target_gap_end'] = float(
                s_target_N - X[-1, IDX_S])
        return steering, throttle, brake

    def _path_reference(self, path) -> PathReference | None:
        """
        Return the PathReference for this tick's path: the one built at load
        time by set_static_path() when its signature still matches (the static
        case — zero per-tick cost), otherwise a cached per-tick rebuild that is
        only redone when the path actually changes (live-planner mode).
        """
        path = np.asarray(path, dtype=float)
        sig = (
            len(path),
            float(path[0, 0]), float(path[0, 1]),
            float(path[-1, 0]), float(path[-1, 1]),
        )
        if self._static_ref is not None and self._static_ref.signature[:5] == sig:
            return self._static_ref
        if self._ref is not None and self._ref_signature == sig:
            return self._ref
        prev = self._ref
        try:
            self._ref = PathReference(
                path,
                dense_step=self.nmpc.nmpc_curvature_dense_step,
                smooth_w=self.nmpc.nmpc_curvature_smooth_w,
                kappa_clip=self.nmpc.nmpc_kappa_clip,
                spline_reference_enabled=self.spline_reference_enabled,
            )
        except (ValueError, IndexError):    # pragma: no cover - defensive
            return None
        self._ref_signature = sig
        self._rate_limit_kappa(self._ref, prev)
        return self._ref

    def _rate_limit_kappa(self, ref: PathReference, prev: PathReference | None) -> None:
        """
        Cap kappa(s)'s tick-to-tick change against the LAST rebuild's profile,
        live-planner mode only (never called for a static/precomputed path,
        see set_static_path()'s own branch in _path_reference).

        Mirrors V_CURV_FALL_RATE (mpc_params.py/mpc_controller.py) but for
        the curvature PROFILE the NMPC's whole horizon predicts against,
        rather than a single scalar speed target. See
        NMPCParams.nmpc_kappa_rate_max's docstring for why: a live-planner
        path's re-resolved corner geometry can move an order of magnitude
        faster tick to tick than a precomputed one, and that's exactly what
        a rejected SQP step (which holds this tick's plan unchanged for
        50 ms) is most exposed to.

        Mutates `ref.kappa`/`ref._k_list` in place and re-derives every
        cached field kappa_scalar's fast path reads, so this MUST run before
        anything downstream (kappa_now, the SQP rollout, telemetry) touches
        `ref`. `ref.psi_ref` is deliberately left untouched: it comes from
        the same smoothed samples as kappa (see PathReference's own
        docstring on why they must describe one reference), so rate-limiting
        kappa alone without also touching psi_ref would desynchronise the
        two and reintroduce the e_psi/e_psi_dot mismatch that motivated
        computing them together in the first place.
        """
        rate = float(self.nmpc.nmpc_kappa_rate_max)
        if rate <= 0.0 or prev is None:
            return
        max_step = rate * self.dt
        prev_on_grid = np.interp(ref.s_kappa, prev.s_kappa, prev.kappa)
        ref.kappa = np.clip(ref.kappa, prev_on_grid - max_step, prev_on_grid + max_step)
        ref._k_list = [float(v) for v in np.atleast_1d(ref.kappa)]
