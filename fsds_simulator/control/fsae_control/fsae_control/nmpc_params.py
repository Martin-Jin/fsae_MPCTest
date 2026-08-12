# Title: nmpc_params.py

"""
nmpc_params.py — tunables for the NONLINEAR MPC path-tracking controller
(nmpc_core.NMPCController). See late_turn_in_investigation.md Part 16 for the
research/decision record behind the formulation.

WHY A SEPARATE DATACLASS FROM MPCParams
---------------------------------------
Every field of mpc_params.MPCParams carries a hard numeric-parity obligation
against fsae_MPCTest/settings.py (CLAUDE.md's "Single source of truth for MPC
tuning, per side" — ~56 values, kept identical by hand). The NMPC is a NEW,
default-OFF second controller that has no offline counterpart yet, so adding
its knobs to MPCParams would immediately create parity debt against a
settings.py that has nothing to mirror them with. Keeping them here means:

  * MPCParams <-> settings.py's existing field-by-field mapping is untouched
    (nothing to add to planning_control_sync.md's numeric-parity table),
  * the NMPC can be retuned live without editing a file that the offline
    tuner also owns.

The COST WEIGHTS are deliberately NOT duplicated here. NMPCController reads
q_e_y / q_e_yd / q_e_psi / q_r / q_e_v / r_delta / r_a_accel / r_a_brake /
r_rate_delta / r_rate_a / terminal_q_scale straight out of the MPCParams
instance the node already builds, so the LTV-QP's tuned weights are the
NMPC's starting point rather than a fresh guess. Each of those has an
override field below (`nmpc_q_e_y`, ...) whose sentinel value -1.0 means
"inherit from MPCParams"; set one to a real value to diverge only that one
weight. That is the intended retuning surface.

ONE WEIGHT DOES NOT TRANSFER WITH IDENTICAL MEANING: q_r. In the LTV-QP it
weights absolute yaw rate `r`; in the Frenet NMPC it weights the
heading-error RATE `r - kappa*s_dot` (see nmpc_core._outputs). Penalising
absolute `r` in a curvature-aware model would penalise the yaw rate the car
MUST hold to follow a corner (r = kappa*v), i.e. it would fight cornering —
which is the exact failure this controller exists to remove. Same number,
different regressor: expect to re-sweep it. See Part 16 §16.3 choice (1).

PROVENANCE OF EVERY DEFAULT BELOW
---------------------------------
No default here is a fresh guess. Each is either (a) copied from an existing
validated value in mpc_core.py / control_utils.py (noted per field), or
(b) a solver/structural setting measured in Part 16 §16.7, or (c) an explicit
guard whose value is chosen to be inert on this car's real operating range
(also noted). Anything that is genuinely unvalidated says so in its `desc`.
"""

from dataclasses import dataclass, field, fields
import math


@dataclass
class NMPCParams:
    # ── Master switch ───────────────────────────────────────────────────
    # Default False: the LTV-QP MPCController stays the shipped controller.
    # Same rollout posture as use_precomputed_corner_map /
    # use_precomputed_heading_profile (land off, prove live, then consider
    # flipping) — and unlike those two, flipping this one swaps the whole
    # optimiser, so it is the most conservative default available.
    use_nmpc: bool = field(default=False, metadata={
        "unit": "bool",
        "desc": "true -> use nmpc_core.NMPCController (Frenet-frame nonlinear MPC) "
                "instead of mpc_core.MPCController (LTV-QP). Default false.",
    })

    # ── Horizon / real-time budget ──────────────────────────────────────
    # MEASURED, not assumed (Part 16 §16.7's horizon sweep, closed-loop on
    # comp_test_map_3 against fsae_MPCTest's Pacejka plant, identical weights):
    #
    #   N   |e_y| mean / p90   |e_psi| mean   steer sat   lap    solve p95
    #   35   0.437 / 1.200        6.45         0.2%      40.7 s   20.1 ms
    #   25   0.314 / 0.775        5.99         1.3%      41.5 s   13.6 ms
    #   20   0.277 / 0.686        5.84         0.8%      42.0 s   12.4 ms
    #   15   0.254 / 0.597        5.73         0.9%      42.8 s   10.5 ms
    #  (QP   0.400 / 1.451        5.92        12.5%      43.1 s   10.2 ms)
    #
    # 20 (= 1.0 s) is chosen over 35 deliberately, even though 35 matches
    # MPCController's horizon: a LONGER horizon measured WORSE on tracking
    # here, because the prediction model is optimistic (linear tyres, no
    # suspension/relaxation) and that mismatch compounds over 1.75 s. 1.0 s at
    # this track's speeds is 12-17 m of anticipation, comparable to the LTV-QP's
    # own speed-scaled lookahead window. Raise it back toward 35 only with a
    # measurement, not on the assumption that more horizon is better.
    nmpc_horizon: int = field(default=20, metadata={
        "unit": "steps",
        "desc": "prediction horizon in steps (20 * dt=0.05 = 1.0 s). Measured "
                "better than 35 (MPCController's value) on tracking error — see "
                "the sweep table above and Part 16 §16.7",
    })
    # 1, not 2: measured slightly BETTER as well as ~40% cheaper (see §16.7).
    # A single Gauss-Newton iteration per tick is the standard real-time-
    # iteration scheme — the warm start carries convergence across ticks, and
    # a converged-per-tick solution exploits the optimistic model harder.
    nmpc_sqp_iters: int = field(default=1, metadata={
        "unit": "iterations",
        "desc": "max Gauss-Newton SQP iterations per control tick (real-time-"
                "iteration style: the previous tick's shifted solution is the "
                "warm start, so one iteration per tick still converges across "
                "ticks). 1 measured better AND cheaper than 2 — see Part 16 §16.7",
    })
    nmpc_solve_budget_ms: float = field(default=25.0, metadata={
        "unit": "ms",
        "desc": "wall-clock budget per tick; SQP stops early (shipping the best "
                "feasible iterate) once exceeded. Half of the 50 ms control "
                "period, leaving the rest of the tick for the node",
    })
    nmpc_rk_substeps: int = field(default=2, metadata={
        "unit": "substeps",
        "desc": "RK4 substeps per dt in the prediction rollout. 2 is needed "
                "because tau_a=0.02 s is stiff against dt=0.05 s (lambda*dt = "
                "-2.5, near RK4's real-axis stability edge)",
    })

    nmpc_jac_substeps: int = field(default=1, metadata={
        "unit": "substeps",
        "desc": "RK4 substeps used when finite-differencing the QP's A_k/B_k "
                "sensitivities. Deliberately coarser than nmpc_rk_substeps: "
                "these only set the SQP STEP DIRECTION, never the predicted "
                "trajectory, and this is the dominant cost per iteration "
                "(see nmpc_core._jacobians)",
    })

    # ── SQP step control ────────────────────────────────────────────────
    nmpc_trust_delta_rad: float = field(default=math.radians(9.0), metadata={
        "unit": "rad",
        "desc": "per-iteration trust region on each stage's steering deviation. "
                "9 deg = MPCController's own du_max steering slew per tick "
                "(180 deg/s * 0.05 s) — reused, not invented",
    })
    nmpc_trust_a: float = field(default=0.6, metadata={
        "unit": "m/s^2",
        "desc": "per-iteration trust region on each stage's accel deviation. "
                "0.6 = MPCController's du_max[1] — reused, not invented",
    })
    nmpc_backtrack_max: int = field(default=2, metadata={
        "unit": "halvings",
        "desc": "max step halvings if a full SQP step increases the true "
                "nonlinear cost (divergence guard). 0 disables backtracking",
    })

    # ── Soft track constraint (mirrors MPCController's own) ─────────────
    nmpc_track_halfwidth: float = field(default=3.5, metadata={
        "unit": "m",
        "desc": "soft |e_y| bound with slack, copied from _build_qp's existing "
                "+-3.5 m literal. <=0 removes the constraint (and its slack "
                "variables) entirely",
    })
    nmpc_slack_weight: float = field(default=10000.0, metadata={
        "unit": "1/m^2",
        "desc": "penalty on the track-bound slack, copied from _build_qp's "
                "existing W_slack = 10000.0",
    })

    # ── Curvature reference construction ────────────────────────────────
    # Both defaults are control_utils.curvature_speed()'s existing denoise
    # precedent (dense_step = 0.5 m, moving-average width 3), not new
    # smoothing constants. This matters more here than for the QP: with
    # kappa inside the PREDICTION, a spurious centreline spike (the known
    # open planner defect, CLAUDE.md) would be predicted as a real bend.
    nmpc_curvature_dense_step: float = field(default=0.5, metadata={
        "unit": "m",
        "desc": "arc-length resampling step for the kappa(s) reference "
                "(= curvature_speed()'s dense_step)",
    })
    nmpc_curvature_smooth_w: int = field(default=3, metadata={
        "unit": "samples",
        "desc": "moving-average width applied before differencing headings "
                "(= curvature_speed()'s w). 1 disables smoothing",
    })
    nmpc_kappa_clip: float = field(default=0.5, metadata={
        "unit": "1/m",
        "desc": "hard clamp on |kappa(s)|. GUARD, not a tuning knob: 0.5 = a "
                "2 m radius, the tightest corner curvature_speed()'s own "
                "docstring contemplates, so it is inert on any real track "
                "line and only catches a degenerate/spiking path",
    })

    nmpc_alat_ceiling_enabled: bool = field(default=True, metadata={
        "unit": "bool",
        "desc": "include FSDS's measured sustained lateral-acceleration ceiling "
                "(MPCParams.alat_ceiling_flat/_slope/_intercept, the same law "
                "mpc_core._alat_ceiling_at uses) as a smooth saturation of the "
                "prediction's tyre forces. True is correct for FSDS; set False "
                "for real-vehicle work, mirroring "
                "model/vehicle_physics.VehicleParams.alat_ceiling_enabled",
    })

    # ── Cost-weight overrides (-1.0 = inherit from MPCParams) ───────────
    # See the module docstring: these exist so the NMPC can be retuned
    # without touching MPCParams (which carries settings.py parity duty).
    nmpc_q_e_y: float = field(default=-1.0, metadata={"unit": "1/m^2", "desc": "override MPCParams.q_e_y for the NMPC only (-1 = inherit)"})
    nmpc_q_e_yd: float = field(default=-1.0, metadata={"unit": "1/(m/s)^2", "desc": "override MPCParams.q_e_yd (-1 = inherit)"})
    nmpc_q_e_psi: float = field(default=-1.0, metadata={"unit": "1/rad^2", "desc": "override MPCParams.q_e_psi (-1 = inherit)"})
    nmpc_q_epsi_dot: float = field(default=-1.0, metadata={
        "unit": "1/(rad/s)^2",
        "desc": "override MPCParams.q_r (-1 = inherit). NOTE this weights "
                "HEADING-ERROR rate (r - kappa*s_dot), not absolute yaw rate — "
                "see the module docstring",
    })
    nmpc_q_e_v: float = field(default=-1.0, metadata={"unit": "1/(m/s)^2", "desc": "override MPCParams.q_e_v (-1 = inherit)"})
    nmpc_r_delta: float = field(default=-1.0, metadata={"unit": "1/rad^2", "desc": "override MPCParams.r_delta (-1 = inherit)"})
    nmpc_r_a_accel: float = field(default=-1.0, metadata={"unit": "1/(m/s^2)^2", "desc": "override MPCParams.r_a_accel (-1 = inherit)"})
    nmpc_r_a_brake: float = field(default=-1.0, metadata={"unit": "1/(m/s^2)^2", "desc": "override MPCParams.r_a_brake (-1 = inherit)"})
    nmpc_r_rate_delta: float = field(default=-1.0, metadata={"unit": "1/rad^2", "desc": "override MPCParams.r_rate_delta (-1 = inherit)"})
    nmpc_r_rate_a: float = field(default=-1.0, metadata={"unit": "1/(m/s^2)^2", "desc": "override MPCParams.r_rate_a (-1 = inherit)"})
    nmpc_terminal_scale: float = field(default=-1.0, metadata={"unit": "unitless", "desc": "override MPCParams.terminal_q_scale (-1 = inherit)"})

    # ── Solver tolerance ────────────────────────────────────────────────
    nmpc_osqp_max_iter: int = field(default=500, metadata={
        "unit": "iterations",
        "desc": "OSQP iteration cap for the SQP subproblem. Bounded well below "
                "MPCController's 8000 on purpose: this is a step DIRECTION that "
                "the true-cost backtracking test validates before it is kept, "
                "so a hard subproblem should cost bounded time and be retried "
                "next tick rather than blow the 50 ms control period (an "
                "uncapped version spent up to 90 ms in single ticks — Part 16 "
                "§16.7)",
    })
    nmpc_osqp_eps: float = field(default=1e-4, metadata={
        "unit": "unitless",
        "desc": "OSQP eps_abs/eps_rel for the SQP subproblem. Looser than "
                "MPCController's 1e-5 on purpose: an SQP subproblem is a STEP "
                "direction that the next iteration corrects, not the final "
                "answer, so sub-1e-4 accuracy buys nothing and costs iterations",
    })


DEFAULT_NMPC_PARAMS = NMPCParams()

# (name, default, metadata) tuples in declaration order — the single source
# both the ROS2 declare_parameters() calls and control.launch.py's launch-arg
# generation build from, mirroring mpc_params.MPC_PARAM_FIELDS exactly.
NMPC_PARAM_FIELDS = tuple(
    (f.name, getattr(DEFAULT_NMPC_PARAMS, f.name), f.metadata)
    for f in fields(NMPCParams)
)


def declare_nmpc_params(node) -> None:
    """
    declare_parameters() every NMPCParams field on `node`, defaulting to
    DEFAULT_NMPC_PARAMS. Mirrors mpc_params.declare_mpc_params().
    """
    node.declare_parameters(
        namespace='',
        parameters=[(name, default) for name, default, _meta in NMPC_PARAM_FIELDS],
    )


def nmpc_params_from_node(node) -> NMPCParams:
    """
    Read back every NMPCParams field from `node`'s already-declared ROS2
    parameters into a fresh NMPCParams. Mirrors
    mpc_params.mpc_params_from_node() (bool/int fields use their ROS2-typed
    accessor; everything else is a plain float).
    """
    kwargs = {}
    for f in fields(NMPCParams):
        value = node.get_parameter(f.name).get_parameter_value()
        if f.type is bool:
            kwargs[f.name] = value.bool_value
        elif f.type is int:
            kwargs[f.name] = value.integer_value
        else:
            kwargs[f.name] = value.double_value
    return NMPCParams(**kwargs)
