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
    q_e_v:   float = field(default=4.0,  metadata={"unit": "1/(m/s)^2", "desc": "speed error: car_speed - desired_speed"})
    # R_diag index -> input penalised (inputs u are [delta_cmd, a_cmd]):
    r_delta: float = field(default=1.6, metadata={"unit": "1/rad^2",     "desc": "steering command effort"})
    # a_cmd>=0 (accel) and a_cmd<0 (brake) get independent effort weights
    # instead of one weight applied symmetrically to |a_cmd| -- a single
    # shared weight cannot be tuned for acceleration and braking
    # independently. See mpc_core.py's _build_qp/_solve_qp for the
    # cp.pos/cp.neg split and planning_control_sync.md's "Accel/brake
    # effort weight split" section for the diagnosis.
    r_a_accel: float = field(default=1.0, metadata={"unit": "1/(m/s^2)^2", "desc": "acceleration command effort, a_cmd >= 0"})
    r_a_brake: float = field(default=0.6, metadata={"unit": "1/(m/s^2)^2", "desc": "acceleration command effort, a_cmd < 0 (braking)"})
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
    lookahead_steer_effort_relax_enabled: bool = field(default=True, metadata={"desc": "make R[0,0] (steering effort) CHEAPER approaching a corner, so turn-in isn't fighting the speed-based effort penalty"})
    # Without this, the QP's own prediction is BLIND to the path bending
    # ahead: Ad/Bd contain no path-curvature term, so with e_y = e_psi = 0
    # (car dead on-line approaching a corner) the 35-step rollout predicts
    # staying at 0 forever and the optimal plan is "go straight" no matter
    # how cheap steering is made. Every other lookahead mechanism only
    # REWEIGHTS an existing error, so none of them can fix that. This feeds
    # the reference path's real curvature into the dynamics constraint as a
    # per-step forcing term, so the predicted trajectory bends the way the
    # road actually does. See mpc_core.py's _curvature_horizon_profile.
    # DISABLED 2026-08-12 -- found structurally unsound, not just weak.
    # Isolated QP tests (clean synthetic corner, no other mechanisms, no
    # noise) showed: at gain=1.0 (physically-exact) the forcing is too weak
    # to matter (predicted |e_psi| deviation stays sub-1deg against a real
    # ~8deg/s corner, so its effect on delta_cmd is noise-scale, sub-1deg,
    # and its SIGN at that scale is essentially arbitrary -- explains "still
    # doesn't turn early"). Raising curvature_forcing_gain to compensate
    # does NOT help: at gain~6 the QP finds it optimal to swing steering
    # hard AWAY from the corner first (predicted e_psi driven to -24deg,
    # e_y to -2.3m) before reversing hard toward it -- explains the
    # "drifts right before some left corners" symptom. Only gain~20 (20x
    # physically-exact) flips the net sign correct, well past saturation
    # and net worse than gain=1. Root cause: injecting curvature as a
    # dynamics DISTURBANCE (x_{k+1}=Ax_k+Bu_k+w_k) gives the QP freedom to
    # choose HOW to spend that disturbance across the whole predicted
    # trajectory -- it is minimizing total quadratic cost, not "tracking
    # the bend," so an transient overshoot-then-correct trajectory can look
    # cheaper than a direct one. A forcing term on the dynamics is the
    # wrong mechanism for this; the fix needs the reference/error definition
    # itself to reflect curvature (e.g. curving the reference heading used
    # to compute e_psi), not an artificial disturbance term feeding the
    # same recursion the QP is optimizing over. See
    # planning_control_sync.md's "Curvature-forcing term" section for the
    # full numeric trace. Code kept in place (curvature_horizon_profile,
    # the w parameter in _build_qp/_solve_qp) for a future redesign -- do
    # not re-enable by flipping this flag without re-deriving the mechanism.
    curvature_forcing_enabled: bool = field(default=False, metadata={"desc": "feed reference-path curvature into the QP's dynamics so its prediction can 'see' an upcoming corner -- DISABLED 2026-08-12, structurally unsound (see field comment)"})
    steer_rate_anti_hunt_enabled: bool = field(default=True, metadata={"desc": "extra R_rate[0,0] penalty when centred/aligned/uncurving"})
    adaptive_r_rate_enable_in_corners: bool = field(default=True, metadata={"desc": "keep R_rate softening active in corners (continuous, no cutoff)"})
    delay_compensation_enabled: bool = field(default=True, metadata={"desc": "roll x0 forward through pending commands via predict_ahead()"})
    ref_heading_rate_limit_enabled: bool = field(default=False, metadata={"desc": "cap how fast the tracked reference heading may change per tick"})
    low_speed_steer_rate_boost_enabled: bool = field(default=False, metadata={"desc": "extra R_rate[0,0] penalty at low speed, INVERTED from Stanley's cheap-at-low-speed shape -- disabled 2026-08-12, live testing found it also taxes turn-in (fast steering-rate change), not just post-exit overcorrection, since it only gates on speed with no curvature/lookahead awareness to tell the two apart"})

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
    # Raised 17.0 -> 25.0 on 2026-08-12 (matching adaptive_q_lookahead_exit_
    # decay_dist_max, which was already 25.0 -- the approach-side ceiling
    # was tighter than the exit-side one for no documented reason). At
    # typical corner-approach speed (16-17 m/s) the desired lookahead
    # (car_speed * adaptive_q_lookahead_time_s) is 18.3-19.4 m, which the
    # old 17.0 m ceiling was silently clamping down to under 1s of lead
    # time regardless of how fast the car was actually going. This is a
    # pure QP-cost-scheduling parameter (how far the Q[0,0]/Q[2,2]
    # corner-anticipation boosts scan ahead), not a sensing/FOV limit --
    # raising it is safe on a precomputed path where the whole route is
    # already known. Does not manufacture tracking error the way
    # curvature_forcing_enabled tried to (see that field's comment) --
    # this only widens the window these boosts can react within once real
    # error/curvature appears in it.
    adaptive_q_lookahead_dist_max: float = field(default=25.0, metadata={"unit": "m", "desc": "lookahead distance clamp ceiling -- raised from 17.0 on 2026-08-12, see field comment"})
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

    # ── Steering-effort relaxation approaching a corner ─────────────────
    # See _lookahead_steer_effort_relax's docstring in mpc_core.py for the
    # mechanism and the gap it closes (neither _adaptive_R_scaling's speed
    # penalty nor _steer_effort_straight_boost ever pushes R[0,0] below
    # baseline for an approaching corner).
    adaptive_q_lookahead_steer_relax_floor: float = field(default=0.5, metadata={"unit": "unitless", "desc": "min R[0,0] multiplier at high corner demand"})

    # ── Curvature forcing term in the QP dynamics ───────────────────────
    # Scales the -v_x*kappa*dt forcing term applied to the predicted e_psi
    # (see curvature_forcing_enabled above and mpc_core.py's
    # _curvature_horizon_profile). 1.0 = the physically-correct Frenet
    # value, which is the right starting point; lower values under-apply
    # the effect (car anticipates less), higher values over-apply it (car
    # anticipates a corner more aggressively than the geometry warrants,
    # which will turn in EARLY rather than late). Provided as a tuning knob
    # because 1.0 assumes the reference path and the speed held across the
    # horizon are both accurate; if the planner's centreline is noisy (see
    # the known centreline-curvature-spike defect) a value below 1.0 may
    # track better in practice than the theoretically-exact one.
    curvature_forcing_gain: float = field(default=1.0, metadata={"unit": "unitless", "desc": "scale on the curvature forcing term; 1.0 = physically exact"})

    # ── Straight-line Q[2,2]/Q[3,3] boosts ──────────────────────────────
    adaptive_q_straight_epsi_boost_max: float = field(default=1.1, metadata={"unit": "unitless", "desc": "max Q[2,2] multiplier on a clear straight"})
    adaptive_q_straight_r_boost_max: float = field(default=1.5, metadata={"unit": "unitless", "desc": "max Q[3,3] multiplier on a clear straight"})
    adaptive_q_straight_k: float = field(default=8.0, metadata={"unit": "unitless", "desc": "shared fade-out sharpness vs lookahead curvature"})

    # ── Straight-line Q[0,0] floor ───────────────────────────────────────
    adaptive_q_straight_ey_floor: float = field(default=0.7, metadata={"unit": "unitless", "desc": "min Q[0,0] multiplier on a clear straight"})
    # Lowered 20.0 -> 8.0 (matching adaptive_q_straight_k, the Q[2,2]/Q[3,3]
    # sibling boosts' shared fade sharpness) on 2026-08-12: the old k=20 snap
    # -back to full lateral weight was so sharp relative to the corner that
    # the car could still be drifting off-line on approach when it fired,
    # arriving at the corner already offset from the planned entry point --
    # "entering the corner at the wrong place" per live driving feedback.
    # k=8 recovers Q[0,0] toward baseline earlier/more gradually relative to
    # the lookahead window, giving the car more distance to re-centre before
    # turn-in. Untested live; re-check straight-line hunting hasn't
    # returned if this is later revisited.
    adaptive_q_straight_ey_k: float = field(default=8.0, metadata={"unit": "unitless", "desc": "ramp sharpness vs lookahead curvature"})

    # ── Straight-line R[0,0] (steering effort) boost ────────────────────
    steer_effort_straight_boost_max: float = field(default=1.5, metadata={"unit": "unitless", "desc": "max R[0,0] multiplier on a clear straight"})
    steer_effort_straight_k: float = field(default=20.0, metadata={"unit": "unitless", "desc": "sharp fade-out vs lookahead curvature"})

    # ── Straight-line R_rate[0,0] (steering rate) anti-hunt boost ───────
    anti_hunt_boost_max: float = field(default=6.0, metadata={"unit": "unitless", "desc": "ceiling on the steer_rate_anti_hunt multiplier"})
    # Added 2026-08-12 so steer_rate_anti_hunt relaxes when a real corner is
    # detected ahead (kappa_max_abs), not just when current kappa/e_y/e_psi
    # are all small -- otherwise anti-hunt actively cancels the early,
    # deliberately-small steering corrections curvature_forcing_enabled
    # produces on approach. Originally set to k=60 (matching the existing
    # current-kappa term, boost_kappa) but a same-day live test showed that
    # was far too eager: 1/(1+60*kappa_max_abs) already halves anti-hunt
    # damping at kappa_max_abs=0.02 -- a corner still well outside the
    # window curvature_forcing actually needs help in. That let a
    # pre-existing, already-documented pose-noise oscillation (see
    # predict_ahead()'s "Delay compensation" note: noise compounding through
    # the rollforward causes +-5-10 deg/tick steering thrash at small
    # e_y/e_psi) get bigger at EVERY curvature level, not just where the
    # forcing term needed room -- reproducing "still turns in late" (the
    # extra swing is noise, not net commitment) plus a new symptom, brief
    # wrong-direction flicks right before some corners. Lowered to k=15 so
    # the gate stays inert until a corner is meaningfully close/sharp
    # (kappa_max_abs 0.05-0.15+), rather than firing on the first faint
    # lookahead signal. Re-tune from a live log with m_Rrate_antihunt and
    # kappa_max_abs side by side, not in isolation.
    anti_hunt_k_lookahead: float = field(default=15.0, metadata={"unit": "unitless", "desc": "anti-hunt fade sharpness vs LOOKAHEAD curvature (kappa_max_abs)"})

    # ── Adaptive R_rate corner softening floors ─────────────────────────
    adaptive_r_rate_during_floor: float = field(default=0.625, metadata={"unit": "unitless", "desc": "R_rate[0,0] floor driven by CURRENT-position curvature"})
    adaptive_r_rate_entering_floor: float = field(default=0.85, metadata={"unit": "unitless", "desc": "R_rate[0,0] floor driven by LOOKAHEAD curvature"})
    adaptive_r_rate_k_entering: float = field(default=4.0, metadata={"unit": "unitless", "desc": "ramp sharpness of the entering floor vs lookahead curvature"})

    # ── Low-speed R_rate[0,0] (steering rate) boost ─────────────────────
    # See _low_speed_steer_rate_boost's docstring in mpc_core.py for the
    # mechanism and the 2026-08-12 log evidence (low-speed post-corner-exit
    # steering wobble while accelerating) that motivated it.
    low_speed_steer_rate_boost_max: float = field(default=2.5, metadata={"unit": "unitless", "desc": "R_rate[0,0] multiplier at vx=0, decaying toward 1.0 as speed rises"})
    low_speed_steer_rate_boost_k: float = field(default=0.35, metadata={"unit": "s/m", "desc": "decay sharpness of the low-speed boost vs speed"})


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
