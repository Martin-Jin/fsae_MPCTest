"""
fsae_control/nmpc/ — NONLINEAR model-predictive path-tracking controller
(Frenet-frame / curvilinear coordinates, Gauss-Newton SQP, condensed dense QP
subproblem solved by OSQP).

This is a SECOND, independently selectable controller. It does not modify,
subclass or import behaviour from lmpc.controller.MPCController's solve path: that
LTV-QP controller remains the default and is completely untouched. Selection
happens at node construction time via NMPCParams.use_nmpc (default False) —
see mpc_controller.py.

WHY IT EXISTS: MPCController's prediction model has no term for the path
itself bending (`e_psi_dot = r`, missing `- kappa(s)*s_dot`), so with the car
dead on-line approaching a corner the QP predicts staying on-line forever and
no cost weighting can produce turn-in before real error exists — measured at
exactly 0.000 deg commanded across 8 synthetic states. Here `kappa(s)` is
looked up against arc length `s`, which is itself a STATE driven by the car's
predicted motion, not indexed by horizon step — so unlike three earlier
attempts to inject curvature as exogenous horizon data (all producing a
wrong-direction transient, see below), the obligation isn't schedulable.
Full derivation, the model equations, the cost construction, and the
real-time SQP structure (roll-forward feasibility, condensing, warm-start,
solve-time budget): `docs/reference/control_mechanisms.md`'s "Nonlinear MPC (`use_nmpc`): the second controller"
section and `late_turn_in_investigation.md` Part 16 (§16.1 gap, §16.3 model,
§16.5 correctness checks, §16.7 solve time/horizon-iteration sweep).

State/input vectors: x = [s, e_y, e_psi, v_x, v_y, r, delta_act, a_act],
u = [delta_cmd, a_cmd]. Every vehicle constant (lf, lr, m, Iz, Cf, Cr,
tau_delta, tau_a, MAX_STEER_RAD, MAX_ACCEL, MAX_BRAKE, du_max) is taken from
MPCController unchanged, including its kinematic/dynamic blend breakpoints —
no new physical constant is introduced. Error measurement (front-axle
projection, e_psi wrap, e_y_dot) is identical to `MPCController._error_state`
so an NMPC log means exactly what an LTV-QP log means. Cost weights come from
the SAME MPCParams the LTV-QP uses — see nmpc_params.py for the per-NMPC
override fields and the one weight whose meaning changes (q_r -> heading-error
rate, not absolute yaw rate).

WHAT THIS CONTROLLER DELIBERATELY DOES NOT DO
---------------------------------------------
* No adaptive gain schedule (lmpc.controller's corner-factor/heading-error-asymmetry
  stack) — that exists to synthesise anticipation a curvature-blind model
  can't produce; layering it on a curvature-aware model would double-count an
  effect that's now structural. Left untouched on the LTV-QP path. The one
  exception is steer_rate_anti_hunt (see EXPERIMENTAL ADDITIONS below) — it
  only ever makes steering-RATE more expensive when already
  centred/aligned/uncurving, the opposite direction from anticipation, so it
  is offered as a separate, independently-defaulted-False opt-in rather than
  assumed exempt from this section's reasoning.
* No shaped heading-lead profile. set_heading_profile() is accepted (so the
  nodes need no branch) and IGNORED with a one-time log line — that profile is
  a workaround for the same missing curvature term this controller closes
  structurally.
* No speed profile over the horizon. v_ref is the caller's single, already
  low-passed desired_speed held constant across the horizon, same as the
  LTV-QP gets — deliberately not addressing the (separate) braking-lag
  problem here, so a live A/B isolates the lateral-model change alone. A
  per-stage speed profile lookup (both a cost-term version and a
  constraint version) was tried and rejected on the car twice; see
  `docs/logs/nmpc_speed_limit_investigation.md` and
  `late_turn_in_investigation.md` Part 16 §16.7.

EXPERIMENTAL ADDITIONS (all additive, all default to preserve the above
exactly)
---------------------------------------------------------------------------
* PathReference.__init__ now fits an analytic CubicSpline to the raw
  waypoints by default (NMPCParams.nmpc_spline_reference_enabled, default
  True) instead of the old dense-resample + moving-average +
  finite-difference pipeline — see PathReference's own docstring. The old
  path is kept (not deleted) and selectable via the flag for A/B comparison.
* nmpc_friction_circle_enabled (default False): an ADDITIONAL hard
  |F_yf|/|F_yr| <= F_max QP constraint on top of (not instead of) the
  existing soft alat-ceiling saturation below — see NH_FRICTION and
  _build_qp/_solve_step.
* nmpc_steer_rate_anti_hunt_enabled (MPCParams field, default False):
  reuses lmpc.adaptive_gains._steer_rate_anti_hunt verbatim (imported, not
  reimplemented, so it is byte-identical by construction) to scale
  R_rate[0,0] up to nmpc_anti_hunt_boost_max (inherits anti_hunt_boost_max)
  when the CURRENT state is centred/aligned/uncurving. Computed once per
  compute() call from the measured state (not per SQP iteration, not a
  function of horizon step), so it is fresh every tick with no cross-tick
  memory — unlike a temporal low-pass filter on the output, this adds no
  lag. See "WHAT THIS CONTROLLER DELIBERATELY DOES NOT DO" above for why
  this is scoped separately from the rest of the gain-schedule family.
* nmpc_corner_rrate_blend_enabled (MPCParams field, default False): a
  narrower, ALTERNATIVE port of lmpc.controller's corner_factor family — blends
  R_rate[0,0] between nmpc_rrate_steer_straight/_corner by CURRENT curvature
  alone (lmpc.adaptive_gains._corner_factor/_blend, imported verbatim). Unlike the rest
  of that family (Q[e_y]/Q[e_psi]/Q[r]/R[steer], deliberately excluded
  above), only R_rate[steer] is touched here, to limit how much of the
  "no adaptive gain schedule" reasoning this overrides. NOT composed with
  nmpc_steer_rate_anti_hunt_enabled — takes priority over it when both are
  set; the two are meant to be used one at a time.

MODULE MAP
----------
  layout.py           IDX_*/NX/NU/NH_* layout, FD step sizes, _DENOM_FLOOR, _wrap
  reference.py        PathReference
  dynamics.py         _Plant, _tyre_forces, _f, _f_scalar, _step_scalar, _step
  outputs.py          _outputs (cost residual rows)
  weight_schedule.py  _rrate_zone_scale, _rrate_stage_ramp
  qp_model.py         _csc_pattern, _QPModelMixin (QP build, rollout, Jacobians, cost)
  sqp_step.py         _SQPStepMixin (_solve_step, _project_feasible)
  solver.py           NMPCController (__init__, reset, set_static_path, compute, ...)
The offline controller/nmpc/ package mirrors these files one to one.
"""

from fsae_control.nmpc.layout import (  # noqa: F401
    IDX_S, IDX_EY, IDX_EPSI, IDX_VX, IDX_VY, IDX_R, IDX_DELTA, IDX_A, NX, NU, NH_TRACKING, NH_PROGRESS, NH_FRICTION, _FD_EPS_X, _FD_EPS_U, _DENOM_FLOOR, _wrap,
)
from fsae_control.nmpc.reference import PathReference  # noqa: F401
from fsae_control.nmpc.dynamics import (  # noqa: F401
    _Plant, _tyre_forces, _f, _f_scalar, _step_scalar, _step,
)
from fsae_control.nmpc.outputs import _outputs  # noqa: F401
from fsae_control.nmpc.weight_schedule import (  # noqa: F401
    _rrate_zone_scale, _rrate_stage_ramp,
)
from fsae_control.nmpc.solver import NMPCController  # noqa: F401
