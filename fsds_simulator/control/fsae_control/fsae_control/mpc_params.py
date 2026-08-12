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
    q_e_y:   float = field(default=6.35, metadata={"unit": "1/m^2",   "desc": "lateral deviation from path centreline"})
    q_e_yd:  float = field(default=0.5,  metadata={"unit": "1/(m/s)^2", "desc": "rate of change of lateral deviation"})
    q_e_psi: float = field(default=1.65, metadata={"unit": "1/rad^2", "desc": "heading error relative to path tangent"})
    q_r:     float = field(default=1.0, metadata={"unit": "1/(rad/s)^2", "desc": "yaw rate"})
    q_e_v:   float = field(default=5.40,  metadata={"unit": "1/(m/s)^2", "desc": "speed error: car_speed - desired_speed"})
    # R_diag index -> input penalised (inputs u are [delta_cmd, a_cmd]):
    r_delta: float = field(default=1.8, metadata={"unit": "1/rad^2",     "desc": "steering command effort"})
    # a_cmd>=0 (accel) and a_cmd<0 (brake) get independent effort weights
    # instead of one weight applied symmetrically to |a_cmd| -- a single
    # shared weight cannot be tuned for acceleration and braking
    # independently. See mpc_core.py's _build_qp/_solve_qp for the
    # cp.pos/cp.neg split and planning_control_sync.md's "Accel/brake
    # effort weight split" section for the diagnosis.
    r_a_accel: float = field(default=2.25, metadata={"unit": "1/(m/s^2)^2", "desc": "acceleration command effort, a_cmd >= 0"})
    r_a_brake: float = field(default=0.5, metadata={"unit": "1/(m/s^2)^2", "desc": "acceleration command effort, a_cmd < 0 (braking)"})
    # R_rate_diag index -> input RATE-OF-CHANGE penalised (tick-to-tick jerk):
    r_rate_delta: float = field(default=2.5, metadata={"unit": "1/(rad/s)^2",     "desc": "steering rate of change"})
    r_rate_a:     float = field(default=2.25, metadata={"unit": "1/(m/s^3)^2",     "desc": "acceleration rate of change"})
    # Extra weight on the final predicted state x[:,N]. 1.0 = no-op, the
    # only value ever validated against the Q_diag/R_diag/R_rate_diag above.
    terminal_q_scale: float = field(default=1.0, metadata={"unit": "unitless", "desc": "extra weight on terminal predicted state"})

    # ── Feature enable/disable flags ────────────────────────────────────
    adaptive_q_scaling_enabled: bool = field(default=True, metadata={"desc": "soften Q[0,0] near centreline to reduce small-error hunting"})
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

    # ── Adaptive R_rate corner softening floor ──────────────────────────
    adaptive_r_rate_during_floor: float = field(default=0.625, metadata={"unit": "unitless", "desc": "R_rate[0,0] floor driven by CURRENT-position curvature"})

    # ── Straight-line R_rate[0,0] (steering rate) anti-hunt boost ───────
    anti_hunt_boost_max: float = field(default=6.0, metadata={"unit": "unitless", "desc": "ceiling on the steer_rate_anti_hunt multiplier"})

    # ── Current-state corner-factor scheduler ────────────────────────────
    # Replaces the whole deleted lookahead gain-scheduling family (see
    # mpc_core.py's module comment): _corner_factor(kappa, corner_factor_k)
    # is a single continuous 0 (straight) -> 1 (full corner) curve of
    # CURRENT |kappa| only -- no forward scan, symmetric on entry/exit.
    # k=8.0 matches the deleted lookahead mechanisms' own default sharpness
    # (8.0) -- same curve shape, now applied to the current-position signal
    # instead of a forward-scanned one.
    corner_factor_k: float = field(default=8.0, metadata={"unit": "unitless", "desc": "corner_factor curve sharpness vs CURRENT |kappa|"})

    # Q[0,0] (e_y): straight/corner blend endpoints. Corner value ~= 2x the
    # straight value, the same order of magnitude boost the deleted
    # lookahead mechanism used to apply multiplicatively, now a
    # straight-line blend between two fixed endpoints instead of a
    # multiplier on a variable base.
    q_ey_straight: float = field(default=4.5, metadata={"unit": "1/m^2", "desc": "Q[0,0] on a clear straight (corner_frac=0)"})
    q_ey_corner: float = field(default=9.0, metadata={"unit": "1/m^2", "desc": "Q[0,0] at full corner (corner_frac=1)"})

    # Q[2,2] (e_psi): straight/corner blend endpoints. Corner value ~= 1.5x
    # the straight value.
    q_epsi_straight: float = field(default=1.5, metadata={"unit": "1/rad^2", "desc": "Q[2,2] on a clear straight (corner_frac=0)"})
    q_epsi_corner: float = field(default=3.0, metadata={"unit": "1/rad^2", "desc": "Q[2,2] at full corner (corner_frac=1)"})

    # Q[3,3] (r, yaw rate): RELAXES in-corner (corner value LOWER than
    # straight, at half the straight value) -- the MPC needs to be able to
    # rotate fast enough in-corner to hit the tighter Q[0,0]/Q[2,2] targets
    # above, so yaw-rate penalty must come DOWN as the other two go UP.
    q_r_straight: float = field(default=1.0, metadata={"unit": "1/(rad/s)^2", "desc": "Q[3,3] on a clear straight (corner_frac=0)"})
    q_r_corner: float = field(default=0.5, metadata={"unit": "1/(rad/s)^2", "desc": "Q[3,3] at full corner (corner_frac=1)"})

    # R_rate[0,0] (steering rate cost): RELAXES in-corner, same direction as
    # Q[3,3] above and for the same reason (the car must be able to steer
    # fast enough to hit the tighter lateral/heading targets); higher on
    # straights to discourage hunting.
    rrate_steer_straight: float = field(default=2.0, metadata={"unit": "1/(rad/s)^2", "desc": "R_rate[0,0] on a clear straight (corner_frac=0)"})
    rrate_steer_corner: float = field(default=1.25, metadata={"unit": "1/(rad/s)^2", "desc": "R_rate[0,0] at full corner (corner_frac=1)"})

    # R[0,0] (steering effort): a SPECIAL case, blended toward a MIDDLE
    # value rather than the same corner-floor extreme as R_rate/Q[3,3] --
    # per the user's own framing, steering effort "should be somewhere in
    # between the two extremes to discourage saturation": too cheap and the
    # MPC commands large swings that saturate the 25deg stop; too
    # expensive (the straight-line value, already speed-scaled up by
    # _adaptive_R_scaling) and turn-in is sluggish. r_steer_corner_mid sits
    # roughly halfway between this file's r_delta baseline (1.8) and a
    # relaxed floor around 0.9 -- i.e. relaxed vs. the (speed-scaled)
    # straight value, but clamped above where R_rate/Q[3,3] bottom out, not
    # all the way down with them.
    r_steer_corner_mid: float = field(default=1.35, metadata={"unit": "1/rad^2", "desc": "R[0,0] blend target at full corner (corner_frac=1) -- a MIDDLE value, not the same low extreme as R_rate/Q[3,3]"})

    # ── Low-speed-in-corner extra boost ──────────────────────────────────
    # A corner-GATED mechanism -- unlike a speed-only gate, this is
    # corner-gated. This only ever adds to corner_frac (see
    # _low_speed_corner_boost in mpc_core.py), so it is an exact no-op on
    # any straight regardless of speed -- "be able to turn even more at low
    # speed during turning" per the user's own framing, not "penalise
    # steering rate whenever slow".
    low_speed_corner_boost_v_half: float = field(default=4.0, metadata={"unit": "m/s", "desc": "speed at which the low-speed corner boost has decayed to half its max_extra"})
    low_speed_corner_boost_max_extra: float = field(default=0.3, metadata={"unit": "unitless", "desc": "max extra corner_frac added at car_speed=0, fully inside a corner"})

    # ── Heading-error-driven accel/brake asymmetry ───────────────────────
    # Always-on, independent of the corner-factor scheduler above: scales
    # r_a_accel/r_a_brake by a continuous 0->1 fraction of CURRENT |e_psi|
    # (x0[2]) -- see mpc_core.py's compute() for the exact blend. Not
    # gain-scheduled off a forward scan; purely reactive to the car's own
    # current heading error.
    epsi_ra_half_rad: float = field(default=math.radians(10.0), metadata={"unit": "rad", "desc": "|e_psi| at which the accel/brake asymmetry reaches half its max effect"})
    epsi_ra_accel_boost_max: float = field(default=2.0, metadata={"unit": "unitless", "desc": "max multiplier on r_a_accel (more expensive to accelerate) at large |e_psi|"})
    epsi_ra_brake_floor: float = field(default=0.5, metadata={"unit": "unitless", "desc": "min multiplier on r_a_brake (cheaper to brake) at large |e_psi|"})

    # ── NMPC weight overrides (nmpc_core.NMPCController only) ────────────
    # NMPC weight overrides live here; see nmpc_params.py for the
    # structural/solver fields that remain there. These are the
    # Frenet-frame nonlinear MPC's cost weights, kept alongside every OTHER
    # weight in this file rather than split across two dataclasses, now
    # that fsae_MPCTest's own offline NMPC port (controller/nmpc_optimiser.py)
    # gives them a real settings.py parity partner (see that file's "NMPC
    # weight overrides" section) the same way every other field in this
    # file already has.
    #
    # -1.0 = inherit the field of the SAME ROW below (q_e_y -> nmpc_q_e_y,
    # r_delta -> nmpc_r_delta, ...), so the NMPC starts from the LTV-QP's
    # tuned weights rather than a fresh guess, and a plain launch with
    # every nmpc_* field left at -1.0 reproduces that inheritance exactly.
    # Set one to a real value to diverge ONLY that weight for the NMPC
    # without touching the LTV-QP's own tuned set. nmpc_core.NMPCController
    # is what reads these (see its __init__'s `_pick(override, inherited)`).
    #
    # nmpc_q_epsi_dot is the ONE weight whose MEANING differs from its row
    # above (q_r): the nonlinear model's matching output is heading-error
    # RATE (r - kappa(s)*s_dot), not absolute yaw rate — penalising absolute
    # yaw rate in a curvature-aware model would penalise the yaw rate the
    # car MUST hold to follow a corner (r = kappa*v), the exact failure the
    # NMPC exists to remove. Same slot, different regressor: expect this
    # one to need its own sweep rather than inheriting q_r unchanged. See
    # late_turn_in_investigation.md Part 16 §16.3 choice (1).
    nmpc_q_e_y: float = field(default=-1.0, metadata={"unit": "1/m^2", "desc": "override q_e_y for the NMPC only (-1 = inherit)"})
    nmpc_q_e_yd: float = field(default=-1.0, metadata={"unit": "1/(m/s)^2", "desc": "override q_e_yd (-1 = inherit)"})
    nmpc_q_e_psi: float = field(default=-1.0, metadata={"unit": "1/rad^2", "desc": "override q_e_psi (-1 = inherit)"})
    nmpc_q_epsi_dot: float = field(default=-1.0, metadata={
        "unit": "1/(rad/s)^2",
        "desc": "override q_r (-1 = inherit). NOTE this weights HEADING-ERROR "
                "rate (r - kappa*s_dot), not absolute yaw rate — see this "
                "section's own comment above",
    })
    nmpc_q_e_v: float = field(default=-1.0, metadata={"unit": "1/(m/s)^2", "desc": "override q_e_v (-1 = inherit)"})
    nmpc_r_delta: float = field(default=-1.0, metadata={"unit": "1/rad^2", "desc": "override r_delta (-1 = inherit)"})
    nmpc_r_a_accel: float = field(default=-1.0, metadata={"unit": "1/(m/s^2)^2", "desc": "override r_a_accel (-1 = inherit)"})
    nmpc_r_a_brake: float = field(default=-1.0, metadata={"unit": "1/(m/s^2)^2", "desc": "override r_a_brake (-1 = inherit)"})
    nmpc_r_rate_delta: float = field(default=-1.0, metadata={"unit": "1/rad^2", "desc": "override r_rate_delta (-1 = inherit)"})
    nmpc_r_rate_a: float = field(default=-1.0, metadata={"unit": "1/(m/s^2)^2", "desc": "override r_rate_a (-1 = inherit)"})
    nmpc_terminal_scale: float = field(default=-1.0, metadata={"unit": "unitless", "desc": "override terminal_q_scale (-1 = inherit)"})


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
