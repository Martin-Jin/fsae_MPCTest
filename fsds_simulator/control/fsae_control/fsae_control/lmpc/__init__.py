"""
fsae_control/lmpc/ — Live MPC Path-Tracking Controller for FSDS

PURPOSE
-------
Provides MPCController, the class mpc_controller.py uses to turn a planner
path + current vehicle state into steering/throttle/brake at 20 Hz. That
node's `standalone_output` parameter picks how the result is used:
false forwards only steering through the shared cmd_vel interface;
true uses the full (steering, throttle, brake) triple directly — see that
file's own docstring for why. It is a self-contained, "live-solve" re-implementation
of the same linear time-varying MPC formulated generically in optimiser.py /
bicycle_model.py for the offline tuner and simulator (both in the
fsae_MPCTest repo), designed for 100% numerical parity with that offline
pipeline so that weights tuned there transfer directly to the real/simulated
vehicle.

  States  x : [e_y, e_yd, e_psi, r, e_v, e_a, delta_act, a_act]   (8,)
  Inputs  u : [delta_cmd (rad), a_cmd (m/s2)]                      (2,)

HOW IT WORKS
------------
Each call to MPCController.compute() runs the full MPC pipeline:
  1. Low-pass filter the incoming desired_speed (_v_des_filtered) to avoid
     feeding step changes into the MPC's speed-error state.
  2. _error_state() — project the vehicle's front axle onto the nearest
     path segment to get Frenet-style tracking errors (e_y, e_psi, e_v) and
     a short-lookahead curvature estimate (kappa), then assemble the 8-state
     vector x0 (reusing the controller's own actuator-lag memory for the
     delta_act/a_act entries, since those aren't directly measurable).
  3. _discrete_model() — build the speed-blended kinematic/dynamic bicycle
     model and ZOH-discretise it (mirrors bicycle_model.get_8state_discrete_model,
     duplicated locally so the live controller has no simulation dependencies).
  4. Gain-schedule R via the module-level _adaptive_R_scaling helper
     (mirrors model_utils.py's adaptive_R_scaling — duplicated here for
     the same reason).
  5. _solve_qp() — inject the above into a persistent, parameterised CVXPY
     problem (built once in _build_qp, reused via warm-start) and solve with
     OSQP, falling back to Clarabel, then to a full-brake command (holding
     the last steering angle) if both solvers fail.
  6. Integrate the actuator lag states exactly (ZOH, not Euler) so
     delta_act/a_act stay consistent even though dt (0.05s) is comparable
     to tau_a (0.02s).
  7. Convert [delta_cmd, a_cmd] into FSDS's normalised
     [steering, throttle, brake] command triple and populate
     self.last_telemetry for the caller's telemetry logging.

PARITY WITH THE OFFLINE PIPELINE
---------------------------------
_adaptive_R_scaling/_discrete_model here are intentionally
near-identical duplicates of model_utils.py / bicycle_model.py, and
_build_qp's cost/constraint formulation is a near-identical duplicate of
optimiser.py's init_parameterized_mpc (same +/-3.5 m soft lane bound, same
W_slack=10000, same step-0/subsequent rate-cost split), plus a hard
per-step slew-rate constraint on [delta_cmd, a_cmd] (self.du_max) enforced
in addition to the soft R_rate cost. Any change to the cost/constraint
structure in one location should be mirrored in the other, or the weights
tuned by offline_tuner.py will no longer transfer faithfully to the live
controller.

USED BY
-------
  mpc_controller.py — constructs an MPCController(dt=0.05, N=35) in
                    __init__ and calls .compute() every 20 Hz tick,
                    .reset() on stale path / cone-brake fail-safes.

MODULE MAP
----------
  constants.py       MAX_STEER_RAD, MAX_ACCEL, MAX_BRAKE
  predict.py         predict_ahead (delay-compensation rollforward)
  adaptive_gains.py  R/Q scaling, anti-hunt, reversal boost, corner-factor family
  controller.py      MPCController
"""

from fsae_control.lmpc.constants import MAX_STEER_RAD, MAX_ACCEL, MAX_BRAKE  # noqa: F401
from fsae_control.lmpc.predict import predict_ahead  # noqa: F401
from fsae_control.lmpc.adaptive_gains import (  # noqa: F401
    _adaptive_R_scaling,
    _steer_rate_anti_hunt,
    _reversal_penalty_boost,
    _adaptive_Q_scaling,
    _curvature,
    _corner_factor,
    _blend,
    _low_speed_corner_boost,
)
from fsae_control.lmpc.controller import MPCController  # noqa: F401
