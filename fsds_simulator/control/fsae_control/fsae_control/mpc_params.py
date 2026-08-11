# Title: mpc_params.py

"""
mpc_params.py — Single source of truth for MPCController's tunable
weights/gains/flags, on the LIVE side.

PURPOSE
-------
Every numeric weight, adaptive-gain shape constant, and feature-enable flag
that mpc_core.py's MPCController used to hardcode inline now lives here as
one dataclass, MPCParams. mpc_core.py accepts an MPCParams instance
(defaulting to DEFAULT_MPC_PARAMS if none is given) instead of reading bare
module-level constants or local list literals — see mpc_core.py's own
docstring for how this is threaded through.

mpc_controller.py / mpc_controller_standalone.py declare_parameters() every
field of MPCParams (using DEFAULT_MPC_PARAMS as the ROS default), read the
values back, and build a fresh MPCParams(...) to hand to MPCController — see
those files for the ROS2 launch-parameter wiring. control.launch.py /
sim.launch.py / fsae_params.yaml / launch_all.sh expose the same fields as
launch-time overrides — see FIELDS below for the metadata that generates
those launch args mechanically instead of by hand (35 near-identical
DeclareLaunchArgument blocks is itself a drift risk).

PARITY WITH THE OFFLINE PIPELINE
---------------------------------
Every field here must stay numerically identical to fsae_MPCTest/settings.py's
matching constant (CLAUDE.md's plant/model parity rule) — see that file's
own comments for the tuning history behind each default. fsae_MPCTest cannot
import this file (it is not on the live car's PYTHONPATH — the standing
"no settings.py-on-the-car" rule, from the other direction), so the two are
kept in sync by hand, the same as Q_diag/R_diag/R_rate_diag always have been.

This is a pure mechanical relocation of existing values — no default below
changes prior behaviour when MPCController is constructed with no override.
"""

from dataclasses import dataclass, field, fields
import math


@dataclass
class MPCParams:
    # ── Core cost weights ───────────────────────────────────────────────
    # Q_diag index -> state penalised (states x are [e_y, e_yd, e_psi, r,
    # e_v, e_a, delta_act, a_act]; see mpc_core.py module docstring):
    q_e_y:   float = field(default=5.20, metadata={"unit": "1/m^2",   "desc": "lateral deviation from path centreline"})
    q_e_yd:  float = field(default=0.2,  metadata={"unit": "1/(m/s)^2", "desc": "rate of change of lateral deviation"})
    q_e_psi: float = field(default=1.52, metadata={"unit": "1/rad^2", "desc": "heading error relative to path tangent"})
    q_r:     float = field(default=0.50, metadata={"unit": "1/(rad/s)^2", "desc": "yaw rate"})
    q_e_v:   float = field(default=5.0,  metadata={"unit": "1/(m/s)^2", "desc": "speed error: car_speed - desired_speed"})
    # R_diag index -> input penalised (inputs u are [delta_cmd, a_cmd]):
    r_delta: float = field(default=1.16, metadata={"unit": "1/rad^2",     "desc": "steering command effort"})
    r_a:     float = field(default=0.77, metadata={"unit": "1/(m/s^2)^2", "desc": "acceleration command effort"})
    # R_rate_diag index -> input RATE-OF-CHANGE penalised (tick-to-tick jerk):
    r_rate_delta: float = field(default=2.1, metadata={"unit": "1/(rad/s)^2",     "desc": "steering rate of change"})
    r_rate_a:     float = field(default=2.6, metadata={"unit": "1/(m/s^3)^2",     "desc": "acceleration rate of change"})
    # Extra weight on the final predicted state x[:,N]. 1.0 = no-op, the
    # only value ever validated against the Q_diag/R_diag/R_rate_diag above.
    terminal_q_scale: float = field(default=1.0, metadata={"unit": "unitless", "desc": "extra weight on terminal predicted state"})

    # ── Feature enable/disable flags ────────────────────────────────────
    adaptive_q_scaling_enabled: bool = field(default=True, metadata={"desc": "soften Q[0,0] near centreline to reduce small-error hunting"})
    adaptive_q_lookahead_enabled: bool = field(default=True, metadata={"desc": "curvature-lookahead Q boosts/relaxations approaching/exiting corners"})
    adaptive_q_demand_normalised: bool = field(default=True, metadata={"desc": "score corners by grip DEMAND (speed-aware) instead of raw curvature"})
    steer_effort_straight_boost_enabled: bool = field(default=True, metadata={"desc": "make R[0,0] (steering effort) expensive on a clear straight"})
    steer_rate_anti_hunt_enabled: bool = field(default=True, metadata={"desc": "extra R_rate[0,0] penalty when centred/aligned/uncurving"})
    adaptive_r_rate_enable_in_corners: bool = field(default=True, metadata={"desc": "keep R_rate softening active in corners (continuous, no cutoff)"})
    delay_compensation_enabled: bool = field(default=True, metadata={"desc": "roll x0 forward through pending commands via predict_ahead()"})
    ref_heading_rate_limit_enabled: bool = field(default=False, metadata={"desc": "cap how fast the tracked reference heading may change per tick"})

    # ── Reference-heading rate limit ────────────────────────────────────
    ref_heading_rise_rate_deg_s: float = field(default=90.0, metadata={"unit": "deg/s", "desc": "max rate the reference heading may change, if enabled"})

    # ── Delay compensation ──────────────────────────────────────────────
    max_delay_compensation_steps: int = field(default=3, metadata={"unit": "steps", "desc": "cap on predict_ahead() rollforward depth"})
    predict_epsi_clip: float = field(default=0.5, metadata={"unit": "rad", "desc": "small-angle bound used inside predict_ahead()"})

    # ── n_delay stabilisation ────────────────────────────────────────────
    pose_age_lp_alpha: float = field(default=0.15, metadata={"unit": "unitless", "desc": "per-tick low-pass coefficient on pose_age_s"})
    n_delay_hysteresis: float = field(default=0.25, metadata={"unit": "steps", "desc": "deadband either side of an n_delay bin boundary"})

    # ── Lookahead corner-anticipation Q-boost ───────────────────────────
    adaptive_q_lookahead_time_s: float = field(default=1.13, metadata={"unit": "s", "desc": "speed -> lookahead distance"})
    adaptive_q_lookahead_dist_min: float = field(default=3.0, metadata={"unit": "m", "desc": "lookahead distance clamp floor"})
    adaptive_q_lookahead_dist_max: float = field(default=17.0, metadata={"unit": "m", "desc": "lookahead distance clamp ceiling"})
    adaptive_q_lookahead_q_boost_max: float = field(default=2.0, metadata={"unit": "unitless", "desc": "max Q[0,0] multiplier approaching a corner"})
    adaptive_q_lookahead_k_approach: float = field(default=8.0, metadata={"unit": "unitless", "desc": "legacy (non-demand-normalised) approach ramp sharpness"})

    # ── Demand-normalised corner scoring ────────────────────────────────
    alat_ceiling_flat: float = field(default=7.5, metadata={"unit": "m/s^2", "desc": "low-speed floor of the FSDS lateral-accel ceiling law"})
    alat_ceiling_slope: float = field(default=0.47, metadata={"unit": "(m/s^2)/(m/s)", "desc": "ceiling law slope vs speed"})
    alat_ceiling_intercept: float = field(default=2.46, metadata={"unit": "m/s^2", "desc": "ceiling law intercept"})
    adaptive_q_demand_half: float = field(default=0.5, metadata={"unit": "unitless", "desc": "corner demand at which a boost reaches half its max"})
    adaptive_q_lookahead_epsi_boost_max: float = field(default=1.5, metadata={"unit": "unitless", "desc": "max Q[2,2] multiplier exiting a corner"})
    adaptive_q_lookahead_epsi_approach_boost_max: float = field(default=1.5, metadata={"unit": "unitless", "desc": "max Q[2,2] multiplier approaching a corner"})
    adaptive_q_lookahead_k_epsi_approach: float = field(default=8.0, metadata={"unit": "unitless", "desc": "legacy (non-demand-normalised) epsi-approach ramp sharpness"})

    # ── U-turn detector (accumulated heading change) ────────────────────
    adaptive_q_uturn_heading_thresh_rad: float = field(default=math.radians(60.0), metadata={"unit": "rad", "desc": "engage past this accumulated heading change"})
    adaptive_q_uturn_heading_sat_rad: float = field(default=math.radians(120.0), metadata={"unit": "rad", "desc": "fully saturated by this much accumulated heading change"})
    adaptive_q_uturn_ey_boost_max: float = field(default=1.6, metadata={"unit": "unitless", "desc": "extra Q[0,0] multiplier at full U-turn detection"})
    adaptive_q_uturn_epsi_boost_max: float = field(default=1.6, metadata={"unit": "unitless", "desc": "extra Q[2,2] multiplier at full U-turn detection"})
    adaptive_q_uturn_r_relax_floor: float = field(default=0.6, metadata={"unit": "unitless", "desc": "Q[3,3] multiplier at full U-turn detection"})
    adaptive_q_lookahead_exit_decay_dist: float = field(default=5.0, metadata={"unit": "m", "desc": "exit boost taper distance floor (see _time_s/_dist_max for the speed-scaled ceiling)"})
    adaptive_q_lookahead_exit_decay_time_s: float = field(default=2.5, metadata={"unit": "s", "desc": "speed -> exit boost taper distance, mirrors adaptive_q_lookahead_time_s"})
    adaptive_q_lookahead_exit_decay_dist_max: float = field(default=25.0, metadata={"unit": "m", "desc": "exit boost taper distance clamp ceiling"})
    adaptive_q_lookahead_k_exit_norm: float = field(default=0.05, metadata={"unit": "1/m", "desc": "normalises exit boost by corner sharpness"})
    adaptive_q_lookahead_peak_hysteresis: float = field(default=0.01, metadata={"unit": "1/m", "desc": "\"cleared\" threshold that re-arms peak detection"})

    # ── Yaw-rate relaxation approaching a corner ────────────────────────
    adaptive_q_lookahead_r_floor: float = field(default=0.5, metadata={"unit": "unitless", "desc": "min Q[3,3] multiplier at high lookahead curvature"})
    adaptive_q_lookahead_k_r_relax: float = field(default=8.0, metadata={"unit": "unitless", "desc": "legacy (non-demand-normalised) yaw-rate relax ramp sharpness"})

    # ── Straight-line Q[2,2]/Q[3,3] boosts ──────────────────────────────
    adaptive_q_straight_epsi_boost_max: float = field(default=1.1, metadata={"unit": "unitless", "desc": "max Q[2,2] multiplier on a clear straight"})
    adaptive_q_straight_r_boost_max: float = field(default=1.5, metadata={"unit": "unitless", "desc": "max Q[3,3] multiplier on a clear straight"})
    adaptive_q_straight_k: float = field(default=8.0, metadata={"unit": "unitless", "desc": "shared fade-out sharpness vs lookahead curvature"})

    # ── Straight-line Q[0,0] floor ───────────────────────────────────────
    adaptive_q_straight_ey_floor: float = field(default=0.7, metadata={"unit": "unitless", "desc": "min Q[0,0] multiplier on a clear straight"})
    adaptive_q_straight_ey_k: float = field(default=20.0, metadata={"unit": "unitless", "desc": "ramp sharpness vs lookahead curvature"})

    # ── Straight-line R[0,0] (steering effort) boost ────────────────────
    steer_effort_straight_boost_max: float = field(default=1.5, metadata={"unit": "unitless", "desc": "max R[0,0] multiplier on a clear straight"})
    steer_effort_straight_k: float = field(default=20.0, metadata={"unit": "unitless", "desc": "sharp fade-out vs lookahead curvature"})

    # ── Straight-line R_rate[0,0] (steering rate) anti-hunt boost ───────
    anti_hunt_boost_max: float = field(default=6.0, metadata={"unit": "unitless", "desc": "ceiling on the steer_rate_anti_hunt multiplier"})

    # ── Adaptive R_rate corner softening floors ─────────────────────────
    adaptive_r_rate_during_floor: float = field(default=0.625, metadata={"unit": "unitless", "desc": "R_rate[0,0] floor driven by CURRENT-position curvature"})
    adaptive_r_rate_entering_floor: float = field(default=0.85, metadata={"unit": "unitless", "desc": "R_rate[0,0] floor driven by LOOKAHEAD curvature"})
    adaptive_r_rate_k_entering: float = field(default=4.0, metadata={"unit": "unitless", "desc": "ramp sharpness of the entering floor vs lookahead curvature"})


DEFAULT_MPC_PARAMS = MPCParams()

# (name, default, metadata) tuples for every field, in declaration order —
# the single source the launch-arg generation (control.launch.py) and the
# ROS2 declare_parameters() calls (mpc_controller.py / mpc_controller_standalone.py)
# both build from, so the dataclass, the YAML defaults, and the launch args
# can't silently drift against each other.
MPC_PARAM_FIELDS = tuple(
    (f.name, getattr(DEFAULT_MPC_PARAMS, f.name), f.metadata)
    for f in fields(MPCParams)
)


def declare_mpc_params(node) -> None:
    """
    declare_parameters() every MPCParams field on `node`, defaulting to
    DEFAULT_MPC_PARAMS. Shared by mpc_controller.py / mpc_controller_standalone.py
    so neither has to hand-write 56 declare_parameters entries (and risk them
    drifting from MPCParams' own defaults/names).
    """
    node.declare_parameters(
        namespace='',
        parameters=[(name, default) for name, default, _meta in MPC_PARAM_FIELDS],
    )


def mpc_params_from_node(node) -> MPCParams:
    """
    Read back every MPCParams field from `node`'s already-declared ROS2
    parameters (see declare_mpc_params above) into a fresh MPCParams
    instance. bool/int fields are read with their ROS2-typed accessor;
    every other field in MPCParams is a plain float.
    """
    kwargs = {}
    for f in fields(MPCParams):
        value = node.get_parameter(f.name).get_parameter_value()
        if f.type is bool:
            kwargs[f.name] = value.bool_value
        elif f.type is int:
            kwargs[f.name] = value.integer_value
        else:
            kwargs[f.name] = value.double_value
    return MPCParams(**kwargs)
