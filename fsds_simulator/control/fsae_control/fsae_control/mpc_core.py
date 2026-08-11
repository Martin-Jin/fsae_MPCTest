# Title: mpc_core.py

# NOTE (fsae_control port): this is a near-verbatim copy of
# fsae_MPCTest/"fsds simulator"/control_utils.py, kept byte-for-byte in the MPC
# math so the offline-tuned weights (Q/R/R_rate) still transfer.  The ONLY
# intentional change is MAX_STEER_RAD 35deg -> 25deg to match this stack's
# physical steering limit (see fsae_control.control_utils.MAX_STEER_RAD and
# fsds_bridge.MAX_STEER_RAD); the MPC now plans steering the car can actually
# deliver.  If you re-sync from upstream, re-apply that one change.

"""
mpc_core.py — Live MPC Path-Tracking Controller for FSDS

PURPOSE
-------
Provides MPCController, the single class both mpc_controller.py and
mpc_controller_standalone.py use to turn a planner path + current vehicle
state into steering/throttle/brake at 20 Hz (mpc_controller.py forwards only
steering through the shared cmd_vel interface; mpc_controller_standalone.py
uses the full (steering, throttle, brake) triple directly — see that file's
own docstring for why). It is a self-contained, "live-solve" re-implementation
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
  4. Gain-schedule R and R_rate via the module-level _adaptive_R_scaling /
     _adaptive_R_rate helpers (mirrors model_utils.py's adaptive_R_scaling /
     adaptive_R_rate — duplicated here for the same reason).
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
_adaptive_R_scaling/_adaptive_R_rate/_discrete_model here are intentionally
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
  mpc_controller.py and mpc_controller_standalone.py — each constructs its
                    own MPCController(dt=0.05, N=35) in __init__ and calls
                    .compute() every 20 Hz tick, .reset() on stale path /
                    cone-brake fail-safes.
"""

import math
import time
from collections import deque

import cvxpy as cp
import numpy as np
from scipy.linalg import expm

from fsae_control.mpc_params import DEFAULT_MPC_PARAMS, MPCParams

# Maximum physical steering deflection.  25deg matches this stack's limit
# (fsae_control.control_utils / fsds_bridge); upstream used 35deg.
MAX_STEER_RAD: float = math.radians(25.0)
MAX_ACCEL: float = 12.0
MAX_BRAKE: float = 7.0

# ── Reference-heading rate limit ─────────────────────────────────────────────
# Mirrors fsae_MPCTest/sim/rollout_core.py's REF_HEADING_RATE_LIMIT_ENABLED /
# REF_HEADING_RISE_RATE / _rate_limit_ref_psi — keep all three in sync.
# Caps how fast the tracked reference heading (path_yaw in _error_state) may
# change per tick, symmetric in both directions — unlike the speed-target
# rise limiter, there is no "always safe" direction for a heading reference.
# Disabled: tried live and made steering saturation worse (holding the
# reference back during turn-in leaves a larger heading deficit to claw back
# later). Re-test against a synthetic slalom path offline before re-enabling.
# Moved to MPCParams.ref_heading_rate_limit_enabled /
# .ref_heading_rise_rate_deg_s (mpc_params.py) — see those fields for the
# current values.

# ── Delay compensation ──────────────────────────────────────────────────────
# Real delay (perception + planning + control + actuation latency) is
# unknown and time-varying, unlike fsae_MPCTest's simulator-only fixed
# DELAY_STEPS. compute() is instead told how OLD the pose it's solving
# against is (pose_age_s, measured from the pose message's own timestamp —
# see mpc_controller.py/_pose_cb) and converts that into a step count itself.
# See predict_ahead() below for the same small-angle-clip rollforward
# validated in fsae_MPCTest/sim/rollout_core.py.
# Lowered 6 -> 3 on 2026-08-10. Measured on
# mpc_standalone_control_1786313570.csv, bucketing steering-reversal rate by
# the n_delay actually used that tick:
#     n_delay=0 -> 0.9% reversals (557 ticks)
#     n_delay=6 -> 32.8% reversals (472 ticks)
# i.e. a ~36x difference, with 41% of the run pinned at the cap. This is the
# same mechanism the DELAY_COMPENSATION_ENABLED note below records (18x
# higher reversal rate in high-n_delay windows): predict_ahead() iterates
# the linear model n_delay times with NO ground-truth correction, so pose
# noise compounds through every extra matrix multiply and the QP faithfully
# tracks the resulting jitter into the steering command. On that log the
# thrash consumed the whole corner-approach phase (steer swinging +-5-10 deg
# per tick at e_y/e_psi ~0), so the car committed to turn-in late and ran
# wide -- a failure that no Q/R weight change can fix, because the command
# is already saturated by noise before the corner starts.
# Disabling compensation outright was tried and was WORSE (see
# DELAY_COMPENSATION_ENABLED's RESULT note), so this caps rollforward DEPTH
# instead: still compensates typical latency, but halves the noise
# compounding.
#
# RESULT (2026-08-10, mpc_standalone_control_1786313826.csv): clearly
# better, and the best result of that tuning session.
#     peak |e_y|   2.52-2.80 m (three cap=6 runs) -> 1.71 m
#     rms  e_y     0.51-0.78 m                    -> 0.39 m
#     steering reversal rate  10.6-15.5%          -> 6.3%
# The cap=6 runs spanned a 20x range of median pose_age_s (0.022-0.497 s)
# and still clustered at 2.5-2.8 m peak error, so 1.71 m sits well outside
# that spread -- this is a real effect, not run-to-run variance. The new
# run's own pose_age_s median (0.204 s) was mid-range, not unusually easy.
#
# CAVEAT: this validates the n_delay -> reversal-rate mechanism, NOT the
# separate (and unsupported) claim that pose latency explained a
# sudden-corner regression -- a same-settings control run showed peak |e_y|
# essentially unchanged (2.796 vs 2.773) across a 20x pose_age swing. The
# real fix is still stabilising pose_age_s upstream, not living on a
# truncated rollforward.
# Moved to MPCParams.max_delay_compensation_steps (the cap: a bigger measured
# age is clamped, not trusted blindly) and MPCParams.predict_epsi_clip (rad,
# the small-angle bound used in predict_ahead) — see mpc_params.py for the
# current values.

# ── n_delay stabilisation ───────────────────────────────────────────────────
# pose_age_s is noisy: control-loop jitter makes a raw round(pose_age_s / dt)
# flip between adjacent step counts tick to tick. Each flip changes how many
# commands predict_ahead() rolls x0 through, so x0 jumps discontinuously
# between rollforward depths on consecutive solves — injecting step changes
# into the state the QP sees at the control rate. Two guards, applied in
# compute():
#   1. Low-pass pose_age_s so a single late message can't move the step count.
#   2. Hysteresis on the resulting integer: only change n_delay when the
#      filtered estimate is clearly past the midpoint of the current bin, so
#      an age hovering near a boundary doesn't dither.
# Moved to MPCParams.pose_age_lp_alpha (per-tick low-pass on pose_age_s,
# ~0.3 s settle at 20 Hz) and MPCParams.n_delay_hysteresis (steps of deadband
# either side of a bin boundary) — see mpc_params.py for the current values.

# ── Lookahead corner-anticipation Q-boost ───────────────────────────────────
# TEMPORARY/EXPERIMENTAL, NOT VALIDATED. Addresses a documented failure mode
# of _adaptive_Q_scaling (see ADAPTIVE_Q_SCALING_ENABLED's comment below):
# that function only looks at the CURRENT |e_y|, so a well-tracked straight
# right before a corner looks identical to a straight that stays straight --
# it can discount Q[0,0] right as the car needs full lateral authority to
# turn in ("late turn-in"). This adds a curvature-lookahead term that boosts
# Q[0,0] (lateral error) as an upcoming corner approaches, then boosts
# Q[2,2] (heading error) as the car exits it, decaying back to baseline once
# clear -- without a discontinuous cutoff (see the adaptive_R_rate
# enable_in_corners/kappa_straight history above for why a threshold-based
# cutoff on curvature risks spiking QP solver iterations at the crossing).
# A third term, _lookahead_yaw_rate_relax, RELAXES Q[3,3] (yaw rate r) on
# the same approach signal: Q[3,3] penalises how fast the car rotates
# regardless of whether that rotation is a genuine, wanted turn-in, so a
# high baseline weight there can itself be part of why turn-in looks late
# or slow even once Q[0,0]/Q[2,2] are pulling the right way.
#
# Mirrors model_utils.py's _lookahead_curvature_profile /
# adaptive_Q_lookahead in fsae_MPCTest — keep all constants in sync.
#
# Composition with ADAPTIVE_Q_SCALING_ENABLED: the lookahead boost is
# applied to Q FIRST (Q[0,0] *= boost_approach, Q[2,2] *= boost_exit), then
# _adaptive_Q_scaling's centred-softening multiplies on top of that result.
# This means a corner boost is never silently cancelled by the
# centred-softening floor, but the two also never fight via a discontinuous
# override -- both stay continuous.
#
# S-curve handling: the approach boost uses max(|kappa|) (not signed max)
# over the lookahead window, so a right-hander immediately after a
# left-hander contributes exactly as much boost as either alone -- sign
# never cancels magnitude. The exit-decay peak-tracker (_update_lookahead_peak
# below) detects LOCAL peaks via a rising-edge-after-a-clear rule, not "any
# new global maximum" -- comparing against the running maximum would
# silently fail to re-trigger the exit boost on a second corner of equal or
# LESSER curvature than an earlier one, an entirely ordinary case (most
# tracks reuse corner radii), not just a rare S-curve edge case. So a second
# corner arriving after the lookahead window has cleared always starts a
# fresh decay cycle regardless of its magnitude relative to the first
# corner's; the two boosts (Q[0,0] approach, Q[2,2] exit) apply to different
# cost terms and never need to be merged against each other either way.
# Moved to MPCParams.adaptive_q_lookahead_time_s (speed -> lookahead
# distance), .adaptive_q_lookahead_dist_min (m, clamp floor for a
# near-stationary car) and .adaptive_q_lookahead_dist_max (m, clamp ceiling)
# — see mpc_params.py for the current values.
# Lengthened 2026-08-10 (0.6s/10m -> 1.13s/17m, ~17m at a 15 m/s cruise)
# after a live log showed the car turning in too gradually/too late into a
# corner -- steer only started ramping ~0.6s before saturating at 25 deg
# (MAX_STEER_RAD) while e_y grew to -1.86m, i.e. the old window gave the
# approach boost too little advance runway before the QP had to react to
# CURRENT curvature anyway. A wider window spans a tight chicane's whole
# corner sequence and sees it as one continuous curvature feature, merging
# what should be two separate approach/exit cycles into one -- see
# _update_lookahead_peak's docstring; this known limitation is now WORSE at
# 17m than it was at 10m, watch for it on closely-spaced corners.
# Moved to MPCParams.adaptive_q_lookahead_q_boost_max (max Q[0,0] multiplier
# approaching a corner) and .adaptive_q_lookahead_k_approach (ramp sharpness
# of the approach boost vs kappa_max_abs) — see mpc_params.py for the current
# values.
# k_approach is LEGACY, only used when adaptive_q_demand_normalised is False.
# Kept for A/B comparison; see the demand-normalisation note directly below
# for why a raw gain on kappa was the wrong parameterisation.

# ── Demand-normalised corner scoring ────────────────────────────────────────
# TEMPORARY/EXPERIMENTAL, NOT VALIDATED. Added 2026-08-10 after finding that
# every boost here was mis-scaled against real corner radii. The old curve,
# 1 - 1/(1 + K*kappa) with K=8, responds like this:
#     kappa=0.025 (R=40m, gradual sweeper) -> 17% of full boost
#     kappa=0.083 (R=12m, the logged U-turn) -> 40%
#     kappa=0.200 (R= 5m, sudden/tight)    -> 62%
#     kappa=0.500 (R= 2m, BELOW the 3.3m steering limit -- unreachable) -> 80%
# i.e. the whole usable range of real corners sits in the flat low-response
# part of the curve, and the nominal Q_BOOST_MAX is never approached. That
# is why raising Q_BOOST_MAX 1.5 -> 2.0 produced almost no visible change:
# most of the increase lives at curvatures the car can never drive.
#
# The fix is to stop scoring corners by RAW curvature and instead score them
# by DEMAND: kappa_max_abs / kappa_limit(v), where kappa_limit is the
# tightest curvature the car can actually hold at its current speed before
# FSDS's lateral-acceleration ceiling binds (kappa_limit = a_lat_max / v^2).
# demand ~1.0 means "this corner needs everything I have at this speed".
# This is scale-free and speed-aware in one step, so a gradual sweeper taken
# fast and a tight U-turn taken slow are judged by the same criterion --
# which is what makes one set of constants work across sharp / gradual /
# U-turn cases instead of needing a hand-tuned threshold per corner type.
#
# a_lat law mirrors model/vehicle_physics.py's alat_ceiling_at():
# max(alat_ceiling, slope*v + intercept), fitted from the measured sweep.
# Keep in sync with that file (CLAUDE.md's plant/model parity rule).
# Moved to MPCParams.adaptive_q_demand_normalised (False restores the legacy
# raw-kappa curve) and MPCParams.alat_ceiling_flat / .alat_ceiling_slope /
# .alat_ceiling_intercept (the fitted a_lat ceiling law) — see mpc_params.py
# for the current values.
#
# adaptive_q_demand_half is the demand at which a boost reaches half of its
# configured maximum. 0.5 = "half the car's available cornering" -- so a
# corner needing most of the grip available at this speed gets most of the
# boost, and an easy corner gets little, with the full range actually
# reachable (unlike the legacy curve). Also in mpc_params.py.
#
# adaptive_q_lookahead_epsi_boost_max is the max Q[2,2] multiplier exiting a
# corner. The Q[2,2] APPROACH boost
# (adaptive_q_lookahead_epsi_approach_boost_max, distinct from the EXIT boost)
# was added 2026-08-10 per a report that short/sudden corners show
# insufficient lateral AND heading commitment; until then heading error only
# got anticipatory help on exit, none before/during turn-in.
# adaptive_q_lookahead_k_epsi_approach is its ramp sharpness vs kappa_max_abs.
# All three live in mpc_params.py.

# ── U-turn detector (accumulated heading change) ────────────────────────────
# TEMPORARY/EXPERIMENTAL, NOT VALIDATED. Added 2026-08-10 after a live log
# showed a long GRADUAL U-turn (~150 deg of total heading change) getting
# under-boosted: every other mechanism here keys off kappa_max_abs, i.e.
# peak curvature MAGNITUDE, and a large-radius U-turn's peak curvature looks
# like a mild bend even though it demands a huge total rotation.
# _lookahead_curvature_profile now also returns accumulated |heading change|
# over the window; this scales an EXTRA turn-in boost from it.
#
# IMPORTANT SCOPE NOTE: on the log that motivated this, the controller was
# already at the full 25 deg steering stop for ~1.3 s through the corner
# with e_psi ~20 deg behind, and achieved curvature at full lock varied 6x
# with speed alone (0.058 1/m at 8.8 m/s vs 0.32 at 4.0 m/s) -- i.e. the
# binding constraint mid-corner was FSDS's lateral-acceleration ceiling
# (~5 m/s^2 sustained there), NOT steering angle. No Q boost can add
# steering that is already saturated. This detector therefore only helps
# BEFORE the corner, while steering is still unsaturated (turning in
# earlier, entering with better heading) -- it cannot stop the car running
# wide once at full lock. The actual fix for that is a slower entry
# (speed-profile side, deliberately out of scope per the session's
# instruction not to touch it).
# Threshold is 60 deg, not the 90 deg a "U-turn" suggests, because it is
# measured WITHIN the lookahead window, not over the whole corner: 17 m of
# arc at the ~12 m radius of the logged U-turn only subtends ~81 deg, so a
# 90 deg threshold could never fire on approach (verified numerically on a
# synthetic long-straight -> gradual 150 deg U-turn). 60 deg of accumulated
# heading change inside 17 m still clearly separates a sustained U-turn
# from an ordinary corner, and fires early enough to matter while steering
# is still unsaturated. Saturation kept at 120 deg for the same reason.
# Moved to MPCParams.adaptive_q_uturn_heading_thresh_rad (engage past this
# accumulated heading change), .adaptive_q_uturn_heading_sat_rad (fully
# saturated by this much), .adaptive_q_uturn_ey_boost_max (extra Q[0,0]
# multiplier at full U-turn detection), .adaptive_q_uturn_epsi_boost_max
# (same for Q[2,2]) and .adaptive_q_uturn_r_relax_floor (Q[3,3] yaw-rate
# multiplier at full detection).
# Also .adaptive_q_lookahead_exit_decay_dist (m, exit boost tapers to 0 over
# this distance), .adaptive_q_lookahead_k_exit_norm (1/m, normalises the exit
# boost by corner sharpness) and .adaptive_q_lookahead_peak_hysteresis (1/m,
# the "cleared" threshold that re-arms peak detection).
# See mpc_params.py for the current values.

# Yaw-rate (Q[3,3], state r) relaxation approaching a corner: added
# 2026-08-10 alongside a late/slow turn-in complaint -- Q[3,3] penalises
# HOW FAST the car is rotating regardless of whether that rotation is
# correcting a real heading error, so a high baseline weight here can make
# the MPC reluctant to commit to the fast yaw-rate turn-in genuinely
# needs. Uses the SAME kappa_max_abs lookahead signal as the Q[0,0]/Q[2,2]
# boosts above (not the current-position-only kappa _adaptive_R_rate uses)
# so the relaxation is in effect BEFORE the car reaches the corner, not
# just once it's already turning -- this is what actually addresses "turns
# late", since a purely reactive relaxation cannot anticipate anything.
# Floor, not a cutoff: continuous saturating curve, same shape as
# _adaptive_R_rate's own floor -- no discontinuity at any threshold.
# Moved to MPCParams.adaptive_q_lookahead_r_floor (min Q[3,3] multiplier at
# high lookahead curvature) and .adaptive_q_lookahead_k_r_relax (ramp
# sharpness of the yaw-rate relax vs kappa_max_abs) — see mpc_params.py for
# the current values.

# Straight-line Q[2,2]/Q[3,3] boosts (see _lookahead_straight_boost) --
# added 2026-08-10 to keep the car pointed straighter and damp yaw wander
# when no corner is near, fading to 1.0 (no-op) as kappa_max_abs rises.
# Q[2,2]'s ceiling is kept small on purpose -- see _lookahead_straight_boost's
# docstring for why a heading-error boost risks amplifying small-mismatch
# hunting in a way a yaw-RATE boost does not.
# Moved to MPCParams.adaptive_q_straight_epsi_boost_max (max Q[2,2] multiplier
# on a clear straight), .adaptive_q_straight_r_boost_max (same for Q[3,3]) and
# .adaptive_q_straight_k (shared fade-out sharpness vs kappa_max_abs) — see
# mpc_params.py for the current values.

# _lookahead_straight_lateral_reduce's Q[0,0] floor on a clear straight --
# added 2026-08-10 after residual hunting persisted despite the boosts
# above; distinct signal from _adaptive_Q_scaling's |e_y|-based softening
# (see that function's own docstring for why the two aren't redundant).
# K raised 8.0 -> 20.0 the same day: at K=8 the reduction was still holding
# Q[0,0] down to ~0.84x at kappa_max_abs=0.02 and ~1.0x only by ~0.05,
# i.e. actively suppressing lateral authority through exactly the early
# corner-approach window where turn-in should be starting, reported live as
# still turning slightly late. K=20 keeps the full 0.7x floor on a
# genuinely clear straight (anti-hunt benefit intact) but recovers toward
# 1.0x much sooner once any curvature is detected ahead.
# Moved to MPCParams.adaptive_q_straight_ey_floor (min Q[0,0] multiplier on a
# clear straight) and .adaptive_q_straight_ey_k (ramp sharpness vs
# kappa_max_abs) — see mpc_params.py for the current values.

# _steer_effort_straight_boost's R[0,0] (steering EFFORT -- how far the
# wheel is turned -- as distinct from R_rate[0,0]'s RATE of change, which
# _steer_rate_anti_hunt already boosts) straight-line boost -- added
# 2026-08-10 alongside a request to make steering rate AND steering itself
# expensive on a clear straight, i.e. this is the second half of that
# request. K deliberately much sharper than the Q-side straight boosts
# above (8.0): the request was specifically for a SHARP ramp-down as
# curvature is detected ahead, so this boost is essentially gone as soon as
# a real turn is being asked for rather than lingering and fighting turn-in
# the way a slower fade risks.
# Moved to MPCParams.steer_effort_straight_boost_max (max R[0,0] multiplier on
# a clear straight) and .steer_effort_straight_k (sharp fade-out vs
# kappa_max_abs) — see mpc_params.py for the current values.

# _steer_rate_anti_hunt's straight-line R_rate[0,0] boost ceiling. Raised
# 2026-08-10 from 3.0 (see that function's docstring for why the switch to
# a continuous curve happened at the same time), then again same day to 6.0
# per a follow-up request for an even stronger straight-line penalty.
# Moved to MPCParams.anti_hunt_boost_max (mpc_params.py) — see that file's
# field for the current value.

# _adaptive_R_rate's "during a corner" floor, driven by current-position
# kappa. Raised 0.55 -> 0.625 on 2026-08-10 (see that function's docstring)
# after entering_floor=0.85 exposed mid-corner steering-rate hunt that the
# old, deeper 0.55 floor was too permissive to damp.
# Moved to MPCParams.adaptive_r_rate_during_floor (mpc_params.py) — see that
# file's field for the current value.

# _adaptive_R_rate's "entering a corner" floor (see that function's
# kappa_max_abs docstring) -- shallower than the during-a-corner floor above,
# which stays driven by current-position kappa. Added 2026-08-10.
# Lowered 8.0 -> 4.0 on 2026-08-10: at 8.0 even mild upcoming curvature
# (kappa_max_abs ~0.02, a gentle bend) already triggered meaningful
# reduction, reported as R_rate "swinging a bit too much prematurely
# before corners" -- lowering the gain delays onset, requiring sharper
# detected curvature before the entering floor starts pulling scale down.
# Moved to MPCParams.adaptive_r_rate_entering_floor and
# .adaptive_r_rate_k_entering (the ramp sharpness of the entering floor vs
# kappa_max_abs), both in mpc_params.py — see those fields for the current
# values.


def predict_ahead(
    x0: np.ndarray,
    Ad: np.ndarray,
    Bd: np.ndarray,
    pending_cmds,
    epsi_clip: float = 0.5,
) -> np.ndarray:
    """
    Roll the linear error-state model forward through commands already
    issued but not yet reflected in the measured pose, so the MPC solves
    against the state it will actually face instead of a stale x0.

    pending_cmds must be ordered oldest-first (the order they were issued).
    Mirrors fsae_MPCTest/sim/rollout_core.py's predict_ahead() exactly,
    including the e_psi clip — see that function's docstring for why the
    clip is needed (the e_psi -> e_y_dot coupling in Ad is only valid for
    small angles, and this rollforward has no per-step ground-truth
    correction the way the closed-loop MPC horizon does).

    epsi_clip (rad, ~28.6 deg at the default) is that small-angle bound; the
    default matches MPCParams.predict_epsi_clip, which MPCController passes
    explicitly.
    """
    x_p = x0.copy()
    for u in pending_cmds:
        x_p[2] = np.clip(x_p[2], -epsi_clip, epsi_clip)
        x_p = Ad @ x_p + Bd @ u
    return x_p

# ---------------------------------------------------------------------------
# Adaptive gain helpers
# ---------------------------------------------------------------------------

def _adaptive_R_scaling(vx: float, R_base: np.ndarray) -> np.ndarray:
    """
    Speed-dependent steering cost with a saturating (Michaelis-Menten) scale.
    steer_scale = 1 + (1.5 * vx) / (6.0 + vx)

    accel_scale disabled (fixed at 1.0) 2026-08-10: at accel_scale = 1 + 0.05*vx,
    R[1,1] rose with speed exactly where corner-entry braking needed to be
    strongest, and relaxed again as the car decelerated mid-approach even
    though heading error/curvature were still climbing -- fighting the
    braking R_diag[1] tuning was trying to loosen. R[1,1] is now governed by
    R_diag[1] alone, independent of vx.
    """
    vx = max(vx, 0.5)
    steer_scale = 1.0 + (1.5 * vx) / (6.0 + vx)
    accel_scale = 1.0
    R = R_base.copy()
    R[0, 0] *= steer_scale
    R[1, 1] *= accel_scale
    return R


def _adaptive_R_rate(
    kappa: float,
    R_rate_base: np.ndarray,
    enable_in_corners: bool = True,
    kappa_max_abs: float = 0.0,
    during_floor: float = 0.625,
    entering_floor: float = 0.85,
    k_entering: float = 4.0,
) -> np.ndarray:
    """
    Curvature-dependent steering-jerk softening: relaxes the steering
    rate-of-change cost in sharp corners (floor during_floor, default
    MPCParams.adaptive_r_rate_during_floor)
    so the controller isn't over-penalised for the extra steering rate a
    tight corner demands. Mirrors model_utils.adaptive_R_rate in
    fsae_MPCTest — keep both floors in sync manually.

    enable_in_corners: TEMPORARY/EXPERIMENTAL, NOT VALIDATED. True
    (default) preserves the softening above -- R_rate reduction stays
    active in corners. False uses a kappa_straight=0.03 "cornering" cutoff:
    once |kappa| exceeds it, softening is switched off and R_rate[0,0] gets
    the full, unscaled baseline cost instead -- deliberately undoing the
    softening this function exists to provide. (Renamed from
    disable_in_corners, whose True/False polarity was inverted from what
    the name suggested.)

    kappa_max_abs: TEMPORARY/EXPERIMENTAL, NOT VALIDATED. Optional lookahead
    curvature (same signal ADAPTIVE_Q_LOOKAHEAD uses, kappa_max_abs=0.0 =
    off/no-op). Added 2026-08-10 per a request to scale R_rate[0,0] down
    "significantly during the turn vs entering it": the CURRENT-position
    kappa this function already used only reflects "during" (the car is
    already turning); kappa_max_abs adds a distinct, SMALLER reduction for
    "entering" (a corner is ahead but the car hasn't reached it yet, so
    current kappa is still ~0). The two floors combine via min() (the more
    aggressive reduction wins) rather than multiplying, so a car already
    mid-corner (current kappa driving scale toward during_floor) isn't
    additionally discounted by the shallower entering_floor -- min() also
    keeps this continuous everywhere
    since both floors are themselves continuous functions of their inputs.
    during_floor raised 0.55 -> 0.625 on 2026-08-10: a live
    run with entering_floor=0.85 showed steering oscillating through zero
    several times per second mid-corner (e.g. +25 -> -20 -> +14 -> -9 deg
    across ~0.3s) while e_y/e_psi stayed small -- classic under-damped
    steering-rate hunt, not a tracking-error problem, so the fix is less
    softening of the rate cost while actually turning, not more lateral/
    heading authority.
    """
    kappa_straight = 0.03
    if not enable_in_corners and abs(kappa) > kappa_straight:
        scale = 1.0
    else:
        during_scale  = max(during_floor, 1.0 / (1.0 + 3.0 * abs(kappa)))
        entering_scale = max(
            entering_floor,
            1.0 / (1.0 + k_entering * kappa_max_abs),
        )
        scale = min(during_scale, entering_scale)
    R = R_rate_base.copy()
    R[0, 0] *= scale
    return R


def _steer_rate_anti_hunt(
    kappa: float,
    e_y: float,
    R_rate_base: np.ndarray,
    enabled: bool,
    e_psi: float = 0.0,
    boost_max: float = 6.0,
) -> np.ndarray:
    """
    TEMPORARY/EXPERIMENTAL, NOT VALIDATED: heavily penalise steering
    rate-of-change on top of _adaptive_R_rate's existing curvature softening,
    strongest when the car is centred (|e_y| small), well-aligned (|e_psi|
    small), AND not currently curving (kappa small). "Corner ahead" is NOT
    a path lookahead here -- it reuses the same causal, current-curvature
    kappa as _adaptive_R_rate, so it cannot anticipate a corner before the
    car is already turning into it. Mirrors model_utils.steer_rate_anti_hunt
    in fsae_MPCTest -- keep both constants in sync. enabled=False returns
    R_rate_base untouched.

    Continuous, not a hard AND-gated threshold (raised from a 3.0x cliff to
    a 4.5x ceiling on 2026-08-10 per a request to penalise straight-line
    hunting more strongly -- kept continuous rather than just raising the
    old step, since a bigger discontinuous jump would only worsen the same
    QP-solver-iteration-spike risk the enable_in_corners/kappa_straight
    history above already found from threshold cutoffs on curvature).
    boost_kappa, boost_ey, and boost_epsi each saturate independently toward
    1.0 as their input shrinks toward 0 (same saturating-curve style as
    _adaptive_R_rate's own floor); their product is the applied scale, so
    the full boost_max (default MPCParams.anti_hunt_boost_max) only applies
    when ALL THREE are near their
    "straight, centred, and aligned" ideal, and it fades smoothly -- never
    snaps -- as any one of them grows.

    e_psi (radians, NOT the degrees used in telemetry/logging -- same units
    _error_state's x0[2]/e_psi already use internally) added 2026-08-10:
    without it, a car that enters a straight MISALIGNED (large |e_psi|,
    small |e_y| -- e.g. just exited a corner still pointed the wrong way)
    would get the full straight-line boost anyway, since kappa/e_y alone
    can't distinguish "straight and correctly aligned" from "straight but
    needs to yaw back into line" -- making exactly the correction it needs
    artificially expensive. k_epsi=23.0 sets half-fade at ~2.5 deg of e_psi.
    """
    if not enabled:
        return R_rate_base
    k_kappa, k_ey, k_epsi = 60.0, 30.0, 23.0
    boost_kappa = 1.0 / (1.0 + k_kappa * abs(kappa))
    boost_ey    = 1.0 / (1.0 + k_ey * abs(e_y))
    boost_epsi  = 1.0 / (1.0 + k_epsi * abs(e_psi))
    scale = 1.0 + (boost_max - 1.0) * boost_kappa * boost_ey * boost_epsi
    R = R_rate_base.copy()
    R[0, 0] *= scale
    return R


def _adaptive_Q_scaling(e_y: float, Q_base: np.ndarray, enabled: bool) -> np.ndarray:
    """
    Soften the lateral-error cost Q[0,0] when already close to the
    centreline, to reduce small-error hunting/chatter. Mirrors
    model_utils.adaptive_Q_scaling in fsae_MPCTest — see that function's
    docstring for the full mechanism and why this is disabled by default.
    enabled=False returns Q_base untouched.
    """
    if not enabled:
        return Q_base
    ey_lo, ey_hi, floor = 0.05, 0.3, 0.5
    ey_abs = abs(e_y)
    if ey_abs >= ey_hi:
        scale = 1.0
    elif ey_abs <= ey_lo:
        scale = floor
    else:
        scale = floor + (1.0 - floor) * (ey_abs - ey_lo) / (ey_hi - ey_lo)
    Q = Q_base.copy()
    Q[0, 0] *= scale
    return Q


def _alat_ceiling_at(
    v: float,
    ceiling_flat: float = 7.5,
    ceiling_slope: float = 0.47,
    ceiling_intercept: float = 2.46,
) -> float:
    """
    FSDS's sustained lateral-acceleration ceiling at speed v (m/s^2).
    Mirrors model/vehicle_physics.py's alat_ceiling_at() -- keep in sync
    (CLAUDE.md's plant/model parity rule). Defaults match
    MPCParams.alat_ceiling_flat / _slope / _intercept.
    """
    return max(ceiling_flat, ceiling_slope * abs(v) + ceiling_intercept)


def _corner_demand(
    kappa_max_abs: float,
    car_speed: float,
    ceiling_flat: float = 7.5,
    ceiling_slope: float = 0.47,
    ceiling_intercept: float = 2.46,
) -> float:
    """
    "How much of the car's available cornering does the path ahead demand at
    the current speed?" -- kappa_max_abs / kappa_limit(v), where
    kappa_limit = a_lat_ceiling(v) / v^2 is the tightest curvature holdable
    before the ceiling binds.

    0 = straight, ~1 = the corner needs everything available at this speed,
    >1 = cannot be held at this speed (must slow). Scale-free and
    speed-aware, so one set of constants covers gradual sweepers, tight
    corners and U-turns instead of needing a per-corner-type threshold --
    see the demand-normalisation note at module scope (and
    MPCParams.adaptive_q_demand_normalised) for why raw kappa was the
    wrong parameterisation. Returns 0.0 below a small speed so a stationary
    or crawling car does not report infinite demand.

    ceiling_flat/slope/intercept parameterise the a_lat ceiling law passed
    through to _alat_ceiling_at; defaults match MPCParams.
    """
    v = abs(car_speed)
    if v < 1.0 or kappa_max_abs <= 0.0:
        return 0.0
    kappa_limit = _alat_ceiling_at(
        v, ceiling_flat, ceiling_slope, ceiling_intercept
    ) / (v * v)
    if kappa_limit <= 1e-9:
        return 0.0
    return float(kappa_max_abs / kappa_limit)


def _demand_frac(demand: float, demand_half: float = 0.5) -> float:
    """
    Map a 0..inf corner demand onto a 0..1 boost fraction via the same
    saturating shape used elsewhere, but with the half-response point set in
    DEMAND units (demand_half, default MPCParams.adaptive_q_demand_half)
    rather than raw curvature -- so the configured maxima are actually
    reachable on real corners.
    """
    h = max(demand_half, 1e-6)
    return demand / (demand + h)


def _lookahead_approach_boost(
    kappa_max_abs: float,
    car_speed: float = 0.0,
    boost_max: float = 2.0,
    demand_normalised: bool = True,
    k_approach: float = 8.0,
    demand_half: float = 0.5,
    ceiling_flat: float = 7.5,
    ceiling_slope: float = 0.47,
    ceiling_intercept: float = 2.46,
) -> float:
    """
    Continuous (no threshold/discontinuity) multiplier on Q[0,0] (lateral
    error) that rises smoothly as the corner ahead gets more demanding, so
    the controller commits lateral authority BEFORE the car is off-centre
    approaching a corner -- see the lookahead Q-boost module comment for
    the motivating failure mode. 1.0 with no corner ahead; saturates toward
    boost_max (default MPCParams.adaptive_q_lookahead_q_boost_max).

    With demand_normalised (default, MPCParams.adaptive_q_demand_normalised)
    the input is corner DEMAND
    (see _corner_demand) rather than raw curvature, which is what makes the
    same constants work for gradual/sharp/U-turn corners and makes the
    configured ceiling actually reachable. Set that flag False to restore
    the legacy raw-kappa curve (gain k_approach) for A/B comparison.
    """
    if demand_normalised:
        frac = _demand_frac(
            _corner_demand(
                kappa_max_abs, car_speed,
                ceiling_flat, ceiling_slope, ceiling_intercept,
            ),
            demand_half,
        )
    else:
        frac = 1.0 - 1.0 / (1.0 + k_approach * kappa_max_abs)
    return 1.0 + (boost_max - 1.0) * frac


def _lookahead_epsi_approach_boost(
    kappa_max_abs: float,
    car_speed: float = 0.0,
    boost_max: float = 1.5,
    demand_normalised: bool = True,
    k_epsi_approach: float = 8.0,
    demand_half: float = 0.5,
    ceiling_flat: float = 7.5,
    ceiling_slope: float = 0.47,
    ceiling_intercept: float = 2.46,
) -> float:
    """
    Continuous (no threshold/discontinuity) multiplier on Q[2,2] (heading
    error) that rises smoothly as the largest curvature within the
    lookahead window grows -- same shape and signal as
    _lookahead_approach_boost, applied to heading instead of lateral error.
    Added 2026-08-10 alongside a report that short/sudden corners show
    insufficient commitment on BOTH lateral and heading error: until now
    Q[2,2] only got anticipatory help from _lookahead_exit_boost (which
    only activates AFTER a peak has already been recorded, decaying
    through the exit), so heading error had no boost at all before/during
    turn-in. Composes multiplicatively with _lookahead_exit_boost on the
    same Q[2,2] entry -- the two are never both large at once in practice
    (approach rises while still approaching a peak; exit only starts
    contributing once a peak has been latched and decays afterward) but
    multiplying rather than taking a max keeps this continuous either way.

    boost_max defaults to
    MPCParams.adaptive_q_lookahead_epsi_approach_boost_max; k_epsi_approach is
    the legacy raw-kappa gain, used only when demand_normalised is False.
    """
    if demand_normalised:
        frac = _demand_frac(
            _corner_demand(
                kappa_max_abs, car_speed,
                ceiling_flat, ceiling_slope, ceiling_intercept,
            ),
            demand_half,
        )
    else:
        frac = 1.0 - 1.0 / (1.0 + k_epsi_approach * kappa_max_abs)
    return 1.0 + (boost_max - 1.0) * frac


def _uturn_severity(
    heading_change_abs: float,
    thresh_rad: float = math.radians(60.0),
    sat_rad: float = math.radians(120.0),
) -> float:
    """
    Continuous 0..1 "how much of a U-turn is coming" score from the
    accumulated |heading change| over the lookahead window (see
    _lookahead_curvature_profile). 0 below thresh_rad (default
    MPCParams.adaptive_q_uturn_heading_thresh_rad -- ordinary corners score
    nothing, so this never disturbs the already-working sudden-corner
    behaviour), ramping linearly to 1.0 at sat_rad (default
    MPCParams.adaptive_q_uturn_heading_sat_rad). Clamped both ends, so it is
    continuous everywhere and bounded regardless of how extreme the path is.
    """
    lo = thresh_rad
    hi = sat_rad
    if hi <= lo:
        return 0.0
    return float(np.clip((abs(heading_change_abs) - lo) / (hi - lo), 0.0, 1.0))


def _uturn_boost(severity: float, boost_max: float) -> float:
    """
    Blend 1.0 -> boost_max by a 0..1 severity from _uturn_severity. Used for
    both the Q[0,0] and Q[2,2] U-turn boosts (boost_max > 1) and the Q[3,3]
    yaw-rate relaxation (boost_max < 1, blending downward instead).
    """
    return 1.0 + (boost_max - 1.0) * severity


def _lookahead_exit_boost(
    last_peak_kappa_abs: float,
    dist_since_peak: float,
    decay_dist: float = 5.0,
    k_exit_norm: float = 0.05,
    boost_max: float = 1.5,
) -> float:
    """
    Continuous multiplier on Q[2,2] (heading error) that is largest right at
    a corner's peak curvature and decays linearly to 1.0 over decay_dist
    (default MPCParams.adaptive_q_lookahead_exit_decay_dist) metres of travel
    afterward, scaled by how sharp that corner was (a gentle bend gets less
    exit correction than a hairpin, via k_exit_norm =
    MPCParams.adaptive_q_lookahead_k_exit_norm). Returns 1.0 (no-op) once
    fully decayed or if no peak has been recorded yet
    (dist_since_peak == inf). boost_max defaults to
    MPCParams.adaptive_q_lookahead_epsi_boost_max.
    """
    if dist_since_peak >= decay_dist or not math.isfinite(dist_since_peak):
        return 1.0
    decay_frac = 1.0 - dist_since_peak / decay_dist
    sharpness = last_peak_kappa_abs / (last_peak_kappa_abs + k_exit_norm)
    return 1.0 + (boost_max - 1.0) * decay_frac * sharpness


def _lookahead_yaw_rate_relax(
    kappa_max_abs: float,
    car_speed: float = 0.0,
    floor: float = 0.5,
    demand_normalised: bool = True,
    k_r_relax: float = 8.0,
    demand_half: float = 0.5,
    ceiling_flat: float = 7.5,
    ceiling_slope: float = 0.47,
    ceiling_intercept: float = 2.46,
) -> float:
    """
    Continuous (no threshold/discontinuity) multiplier on Q[3,3] (yaw rate r)
    that FALLS smoothly as the largest curvature within the lookahead window
    grows -- the mirror image of _lookahead_approach_boost's rise, applied to
    yaw-rate penalty instead of lateral-error penalty. 1.0 at kappa_max_abs=0
    (no corner ahead, full yaw-rate damping as tuned); floors toward
    `floor` (default MPCParams.adaptive_q_lookahead_r_floor) as curvature rises, so the MPC is less
    penalised for the fast rotation a real turn-in needs, and -- because this
    uses the same forward lookahead as the Q[0,0] boost, not the car's
    current-position kappa -- the relaxation is already in effect before the
    car reaches the corner, addressing "turns late/slowly" rather than only
    reacting once already mid-turn.

    k_r_relax is the legacy raw-kappa gain, used only when demand_normalised
    is False.
    """
    if demand_normalised:
        frac = _demand_frac(
            _corner_demand(
                kappa_max_abs, car_speed,
                ceiling_flat, ceiling_slope, ceiling_intercept,
            ),
            demand_half,
        )
    else:
        frac = 1.0 - 1.0 / (1.0 + k_r_relax * kappa_max_abs)
    return 1.0 - (1.0 - floor) * frac


def _lookahead_straight_lateral_reduce(
    kappa_max_abs: float,
    ey_floor: float = 0.7,
    ey_k: float = 20.0,
) -> float:
    """
    Continuous (no threshold/discontinuity) multiplier on Q[0,0] (lateral
    error) that RISES from ey_floor (default
    MPCParams.adaptive_q_straight_ey_floor) toward 1.0 as the
    largest curvature within the lookahead window grows -- i.e. reduced
    lateral-error cost on a genuinely clear straight (kappa_max_abs~0),
    fading back to full baseline weight as a corner enters the lookahead
    window, where it composes multiplicatively with
    _lookahead_approach_boost's rise above 1.0. Reuses
    _lookahead_straight_boost's exact formula with a sub-1.0 ceiling --
    that function's derivation makes no assumption boost_max > 1.0, it is
    simply "blend from boost_max at kappa_max_abs=0 to 1.0 as kappa_max_abs
    grows", which is exactly the shape needed here, just reducing instead
    of boosting.

    Added 2026-08-10 after residual hunting persisted despite the
    Q[2,2]/Q[3,3] straight-line boosts and a stronger anti-hunt boost ceiling
    (MPCParams.anti_hunt_boost_max) --
    distinct from (and composes multiplicatively with)
    _adaptive_Q_scaling's existing |e_y|-based softening, which triggers on
    "currently centred" regardless of curvature; this instead triggers on
    "no corner anywhere near", independent of the car's instantaneous
    lateral error.
    """
    return _lookahead_straight_boost(kappa_max_abs, ey_floor, ey_k)


def _lookahead_straight_boost(kappa_max_abs: float, boost_max: float, k: float) -> float:
    """
    Continuous (no threshold/discontinuity) multiplier that FALLS from
    boost_max toward 1.0 as the largest curvature within the lookahead
    window grows -- the mirror image of _lookahead_yaw_rate_relax's fall
    (which relaxes a cost approaching a corner), used here to instead RAISE
    a cost on straights and fade it back to baseline as a corner is
    detected ahead. Shared helper for Q[2,2] (heading error), Q[3,3] (yaw
    rate), and R[0,0] (steering effort) straight-line boosts -- added
    2026-08-10 per requests to keep the car pointed straighter, damp yaw
    wander, and discourage steering deflection at all when no corner is
    near. Q[2,2]'s boost_max is kept deliberately small (see
    MPCParams.adaptive_q_straight_epsi_boost_max) since a stronger heading-error
    weight on straights amplifies the QP's reaction to ordinary heading
    noise, i.e. the exact small-error hunting _adaptive_Q_scaling exists to
    fight elsewhere -- Q[3,3]/R[0,0] (less prone to that particular
    noise-driven chatter) can safely take a larger boost. k controls how
    sharply the boost fades as curvature is detected ahead -- R[0,0]'s k is
    set much higher than Q[2,2]/Q[3,3]'s so it collapses to baseline almost
    as soon as a real turn is asked for, rather than lingering into it.
    """
    return 1.0 + (boost_max - 1.0) * (1.0 / (1.0 + k * kappa_max_abs))


def _steer_effort_straight_boost(
    kappa_max_abs: float,
    boost_max: float = 1.5,
    k: float = 20.0,
) -> float:
    """
    R[0,0] (steering EFFORT -- how far the wheel is turned -- NOT
    rate-of-change; that's R_rate[0,0], boosted separately by
    _steer_rate_anti_hunt) instance of _lookahead_straight_boost: 1.5x on a
    clear straight, fading sharply (K=20, much sharper than the Q-side
    straight boosts) toward 1.0 as a corner enters the lookahead window.
    Composes with _adaptive_R_scaling's existing speed-dependent scaling on
    R[0,0] via multiplication. boost_max/k default to
    MPCParams.steer_effort_straight_boost_max / .steer_effort_straight_k.
    """
    return _lookahead_straight_boost(kappa_max_abs, boost_max, k)


def _curvature(path: np.ndarray, idx: int) -> float:
    """
    Estimate signed path curvature (1/m) at waypoint idx via finite-difference.
    """
    if idx <= 0 or idx >= len(path) - 1:
        return 0.0
    s_prev = path[idx]     - path[idx - 1]
    s_next = path[idx + 1] - path[idx]
    yaw_p  = math.atan2(s_prev[1], s_prev[0])
    yaw_n  = math.atan2(s_next[1], s_next[0])
    dpsi   = math.atan2(math.sin(yaw_n - yaw_p), math.cos(yaw_n - yaw_p))
    ds     = (np.linalg.norm(s_prev) + np.linalg.norm(s_next)) * 0.5
    return dpsi / ds if ds > 1e-6 else 0.0


def _lookahead_curvature_profile(
    path: np.ndarray, base_idx: int, lookahead_dist: float
) -> tuple[float, int, float, float]:
    """
    Scan forward from base_idx up to lookahead_dist (arc length, m) and
    return (kappa_max_abs, idx_at_peak, dist_at_peak, heading_change_abs):
    the largest-MAGNITUDE signed curvature found in the window, the waypoint
    index it occurred at, how far ahead (m) that point is, and the TOTAL
    accumulated |heading change| (rad) across the window.

    Uses |kappa|, not signed kappa, so an S-curve's second corner (opposite
    sign to the first) contributes exactly as much as either corner alone --
    sign never cancels magnitude here. See ADAPTIVE_Q_LOOKAHEAD's module
    comment for why this matters.

    heading_change_abs (added 2026-08-10) is the U-turn discriminator: a
    long, GRADUAL U-turn has unremarkable peak curvature (large radius) but
    a very large total heading change, so kappa_max_abs alone scores it like
    a mild bend and under-boosts turn-in. Summing |kappa| * ds across the
    window captures "how much total rotation is coming" independently of how
    sharp any single point is. Measured on a live log, the U-turn that
    saturated steering showed ~2.6 rad (~150 deg) of accumulated change while
    its peak kappa stayed modest.

    Returns (0.0, base_idx, 0.0, 0.0) if the window can't be scanned (path
    too short / already at the end).
    """
    accumulated   = 0.0
    kappa_max_abs = 0.0
    idx_at_peak   = base_idx
    dist_at_peak  = 0.0
    heading_change_abs = 0.0
    for i in range(base_idx, len(path) - 1):
        ds = float(np.linalg.norm(path[i + 1] - path[i]))
        accumulated += ds
        if accumulated > lookahead_dist:
            break
        k = abs(_curvature(path, i + 1))
        heading_change_abs += k * ds
        if k > kappa_max_abs:
            kappa_max_abs = k
            idx_at_peak   = i + 1
            dist_at_peak  = accumulated
    return kappa_max_abs, idx_at_peak, dist_at_peak, heading_change_abs


# ---------------------------------------------------------------------------
# MPC Controller
# ---------------------------------------------------------------------------

class MPCController:
    """
    Linear time-varying MPC for combined lateral and longitudinal path tracking.
    """
    def __init__(
        self,
        dt: float = 0.05,
        N:  int   = 35,
        params: MPCParams = None,
    ) -> None:
        """
        Parameters
        ----------
        dt : float
            Control/prediction timestep (s). Must equal the calling node's
            control timer period (0.05 s / 20 Hz in both mpc_controller.py
            and mpc_controller_standalone.py) so the discretised model's
            predictions align with real elapsed time.
        N : int
            MPC horizon length in steps (35 -> 1.75 s lookahead at dt=0.05).
            Must match settings.N_HORIZON for tuned weights to transfer.
        params : MPCParams, optional
            Every tunable weight/gain/flag (see mpc_params.py). Defaults to
            DEFAULT_MPC_PARAMS, whose field defaults are numerically identical
            to the values this constructor used to hardcode inline — so
            MPCController() with no params behaves exactly as before.

        Vehicle geometry/dynamics constants (lf, lr, m, Iz, Cf, Cr,
        tau_delta, tau_a) are hardcoded here rather than imported from
        vehicle_physics.VehicleParams — keep these in sync manually if the
        plant model is retuned, since this is the "live" copy used on the
        real/simulated vehicle.
        """
        self.dt = dt
        self.N  = N
        self.params = params if params is not None else DEFAULT_MPC_PARAMS

        # ── Vehicle geometry & dynamics  ─────
        # FSDS-matched values. Mass 255 kg is confirmed from the sim
        # (docs/vehicle_model.md). The true lf/lr, Iz and Cf/Cr are NOT in the
        # FSDS repo (they live in git-LFS .uasset binaries), so these are chosen
        # from physical reasoning rather than read off:
        #   - lf < lr (CoG biased toward the front axle) makes the bicycle model
        #     UNDERSTEER (understeer gradient K_us > 0), i.e. stable at every
        #     speed — a needless oversteer stability risk otherwise, on a car
        #     whose real balance we can't measure.
        #   - Iz ~= m*lf*lr (~152) is the standard yaw-inertia estimate.
        # Wheelbase L = lf + lr = 1.55 m is an estimate for a FS car (the FSDS
        # collision box is 1.80 m long — an upper bound on car length, not the
        # wheelbase). Refine all of these via system-ID on the running sim.
        # Must match vehicle_physics.VehicleParams' lf/lr/Iz (CLAUDE.md's
        # plant/model parity rule) — keep in sync manually.
        self.lf = 0.70
        self.lr = 0.85
        self.m  = 255.0
        self.Iz = 150.0
        # Cf/Cr mirror vehicle_physics.VehicleParams.Cf/Cr — the linear
        # cornering stiffness matched to the Pacejka curve's initial slope
        # (C_eff = mu_eff * Fz_nominal * B * C * D). If the tyre model in
        # vehicle_physics.py changes, recompute these from VehicleParams()
        # and paste the new values here; don't hand-edit them independently.
        self.Cf = 29155.47766921484
        self.Cr = 19512.3421655211
        self.tau_delta = 0.08
        self.tau_a     = 0.02

        self.nx = 8
        self.nu = 2

        # Values live in mpc_params.py (MPCParams.q_*/r_*/r_rate_*); keep them
        # numerically identical to fsae_MPCTest/settings.py's
        # Q_diag/R_diag/R_rate_diag and the same three lists in
        # fsds_simulator's copy of this file (see CLAUDE.md's parity rule).
        #
        # Q_diag index -> state penalised (states x are [e_y, e_yd, e_psi, r,
        # e_v, e_a, delta_act, a_act], see the module docstring above):
        #   [0] e_y        lateral deviation from path centreline (m)
        #   [1] e_yd       rate of change of lateral deviation (m/s)
        #   [2] e_psi      heading error relative to path tangent (rad)
        #   [3] r          yaw rate (rad/s)
        #   [4] e_v        speed error: car_speed - desired_speed (m/s)
        #   [5] e_a        unused (always 0.0, kept for structural consistency only)
        #   [6] delta_act  actuator-lagged steering angle (rad) -- always 0.0
        #   [7] a_act      actuator-lagged acceleration (m/s^2) -- always 0.0
        Q_diag      = [
            self.params.q_e_y,
            self.params.q_e_yd,
            self.params.q_e_psi,
            self.params.q_r,
            self.params.q_e_v,
            0.0,   # e_a       — always 0.0, not parameterised (see above)
            0.0,   # delta_act — always 0.0, not parameterised
            0.0,   # a_act     — always 0.0, not parameterised
        ]
        # R_diag index -> input penalised (inputs u are [delta_cmd, a_cmd]):
        #   [0] delta_cmd  steering command effort (rad)
        #   [1] a_cmd      acceleration command effort (m/s^2)
        R_diag      = [self.params.r_delta, self.params.r_a]
        # R_rate_diag index -> input RATE-OF-CHANGE penalised (tick-to-tick
        # jerk, not the input itself):
        #   [0] delta_cmd  steering rate of change
        #   [1] a_cmd      acceleration rate of change
        R_rate_diag = [self.params.r_rate_delta, self.params.r_rate_a]

        self.Q      = np.diag(Q_diag)
        self.R      = np.diag(R_diag)
        self.R_rate = np.diag(R_rate_diag)  

        # ── Hard actuator limits ───────────────────────────────────────
        # Matched exactly to the offline tuner/vehicle plant capabilities
        self.a_max = MAX_ACCEL
        self.a_max_brake = MAX_BRAKE
        self.u_min = np.array([-MAX_STEER_RAD, -self.a_max_brake]) 
        self.u_max = np.array([ MAX_STEER_RAD,  self.a_max])
        
        # Hard per-step slew-rate limit on [delta_cmd, a_cmd], enforced in
        # _build_qp in addition to the soft R_rate cost.
        #
        # Steering: expressed as a RATE (deg/s) x dt rather than a fixed
        # per-step angle, so the physical meaning survives a change of dt.
        # 180 deg/s was set just under the plant's measured achievable
        # roadwheel rate (~200 deg/s, via system-ID) — see
        # fsae_MPCTest/docs/planning_control_sync.md's "Slew-rate limit
        # (du_max)" section for the full history (80->180 deg/s fixed a
        # limit cycle that was pinning 41% of live control steps).
        #
        # Was briefly raised to 190 deg/s on 2026-08-10 (see git history/
        # session notes) to relieve mid-corner clamping at exactly
        # 9.00 deg/tick (180 deg/s * 0.05s dt), but a subsequent run showed
        # WORSE jitter/late-turn-in/control_smooth_rms/jerk_rms with several
        # other same-day changes also active (adaptive_q_lookahead ruled
        # out by disabling it and still seeing the regression) -- reverted
        # to 180 pending isolation of which change actually caused it.
        # UNMEASURED either way -- the sync doc explicitly says to refine
        # this "via system-ID on the running sim", not by picking a number;
        # re-measure properly before trusting either value long-term, and
        # update both sides (this file + fsae_MPCTest's controller/
        # optimiser.py / vehicle_physics.py du_max) together per that doc's
        # parity rule.
        MAX_STEER_RATE_RAD_S: float = math.radians(180.0)
        self.du_max = np.array([MAX_STEER_RATE_RAD_S * self.dt, 0.6])

        # TERMINAL_Q_SCALE (settings.py, fsae_MPCTest) — extra weight on the
        # final predicted state x[:,N], on top of the uniform per-step weight
        # it already gets. 1.0 = no-op, matching every weight set tuned
        # against this controller so far. Inlined here per the standing
        # no-settings.py-on-the-car rule (now via MPCParams.terminal_q_scale
        # in mpc_params.py); must be kept numerically identical to
        # fsae_MPCTest's copy.
        self.terminal_scale = self.params.terminal_q_scale

        # ADAPTIVE_Q_SCALING_ENABLED (settings.py, fsae_MPCTest) — see
        # _adaptive_Q_scaling above. Disabled: a live A/B test found it
        # coincided with a bad late-turn-in episode (the lateral-error cost
        # was discounted at exactly the moment the controller needed to
        # commit to steering authority early). Not proven solely causal;
        # re-verify before re-enabling.
        self.adaptive_q_scaling_enabled = self.params.adaptive_q_scaling_enabled

        # ADAPTIVE_Q_LOOKAHEAD_ENABLED (settings.py, fsae_MPCTest) — see the
        # ADAPTIVE_Q_LOOKAHEAD module comment above for the full mechanism.
        # TEMPORARY/EXPERIMENTAL, NOT VALIDATED: added 2026-08-09 to address
        # adaptive_Q_scaling's documented late-turn-in failure mode via
        # curvature lookahead instead. Was disabled 2026-08-10 while
        # isolating a jitter/jerk regression -- that isolation run (this
        # flag False, all else unchanged) showed the regression PERSISTED,
        # ruling this mechanism out as the cause. Re-enabled the same day
        # per a direct report that the car is not turning in early enough
        # at corners (the exact failure mode this mechanism exists to
        # address) and is pinning MAX_STEER_RAD (25 deg, confirmed as
        # FSDS's real hardware limit, upstream issue #270) for over a
        # second in at least one corner as a result. The earlier 2026-08-09
        # e_y-excursion concern (approach boost vs. braking priority) is
        # still not independently re-verified -- watch for it specifically.
        self.adaptive_q_lookahead_enabled = self.params.adaptive_q_lookahead_enabled

        # STEER_EFFORT_STRAIGHT_BOOST_ENABLED (settings.py, fsae_MPCTest) —
        # see _steer_effort_straight_boost above. Added 2026-08-10 alongside
        # a request to make steering EFFORT (R[0,0], not just its rate of
        # change -- that's _steer_rate_anti_hunt, separate) expensive on a
        # clear straight too. Renamed from adaptive_r_steer_straight_boost*
        # the same day after the old name was confused for the
        # rate-of-change mechanism. NOT VALIDATED -- enabled 2026-08-10 for
        # live testing.
        self.steer_effort_straight_boost_enabled = (
            self.params.steer_effort_straight_boost_enabled
        )

        # STEER_RATE_ANTI_HUNT_ENABLED (settings.py, fsae_MPCTest) — see
        # _steer_rate_anti_hunt above. Temporary experiment requested to
        # suppress low-error steering hunt; NOT validated against a live
        # log or VALIDATION_SUITE. Default off, inlined per the standing
        # no-settings.py-on-the-car rule; keep in sync with fsae_MPCTest's
        # copy while this is being tried.
        self.steer_rate_anti_hunt_enabled = self.params.steer_rate_anti_hunt_enabled

        # ADAPTIVE_R_RATE_ENABLE_IN_CORNERS (settings.py, fsae_MPCTest) —
        # see _adaptive_R_rate's enable_in_corners param above (renamed from
        # disable_in_corners, whose polarity was inverted from what the name
        # suggested). True (default) keeps R_rate reduction ACTIVE in
        # corners via the continuous curve (no threshold, no discontinuity).
        # False uses the kappa_straight cutoff to switch softening off past
        # it, restoring full baseline R_rate[0,0] -- tried 2026-08-09
        # (kappa_straight raised to 0.1 first) but caused severe lag
        # specifically in corners -- the discontinuous R_rate[0,0] jump at
        # the kappa_straight crossing likely spikes QP solver iterations /
        # invalidates warm-starts every tick near the threshold. Reverted
        # the same day. Inlined per the standing no-settings.py-on-the-car
        # rule; keep in sync with fsae_MPCTest's copy while this is being
        # tried.
        self.adaptive_r_rate_enable_in_corners = (
            self.params.adaptive_r_rate_enable_in_corners
        )

        # DELAY_COMPENSATION_ENABLED — TEMPORARY/EXPERIMENTAL, NOT VALIDATED.
        # Master switch for _update_n_delay()/predict_ahead() (see both
        # above). Added 2026-08-09 to test a hypothesis: n_delay was observed
        # swinging 0->6 in ~10s blocks on a live standalone log
        # (mpc_standalone_control_1786273193.csv), and steering-reversal rate
        # on straight sections was 18x higher during high-n_delay windows
        # (33.3% vs 1.8%) despite e_y barely moving -- i.e. predict_ahead()'s
        # rollforward may itself be injecting the chatter rather than
        # compensating for real lag. False (set here) skips
        # _update_n_delay()/predict_ahead() entirely: x0 is solved raw, with
        # zero delay compensation, to isolate whether this mechanism is the
        # cause. Not a fix either way -- if disabling it removes the
        # chatter, the next step is to find why pose_age_s/n_delay swing so
        # widely in the first place, not to leave compensation off
        # permanently.
        #
        # RESULT (2026-08-09, mpc_standalone_control_1786274337.csv): worse,
        # not better. rmse 0.497->0.677, steering_sat_ratio 0.267->0.517,
        # |e_psi| up to 125 deg, half the run pinned at the 25 deg steer
        # limit. pose_age_s did NOT go away with compensation off (still
        # swinging ~0-0.4s in the same ~10s blocks) -- it just went
        # uncorrected instead of jittering the correction. Reversal rate did
        # drop (0.067->0.029), so the mechanism isn't imaginary, but the net
        # effect of disabling compensation outright is worse than the
        # chatter it was meant to isolate. Restored to True; the real fix is
        # stabilising pose_age_s upstream, not leaving this off.
        self.delay_compensation_enabled = self.params.delay_compensation_enabled

        # ── Continuity memory ─────────────────────────────────────────
        self._delta_act:      float      = 0.0
        self._a_act:          float      = 0.0
        self._u_prev:         np.ndarray = np.zeros(self.nu)
        self._v_des_filtered: float | None = None

        # Previous tick's LIMITED reference heading (rad, unwrapped-compatible
        # — see _rate_limit_ref_psi), for
        # MPCParams.ref_heading_rate_limit_enabled. None
        # on the first tick after start/reset: the raw value passes through
        # unlimited, mirroring _v_des_filtered's None handling above.
        self._ref_psi_prev: float | None = None

        # ADAPTIVE_Q_LOOKAHEAD peak-tracker (see module comment above and
        # _update_lookahead_peak below). _dist_since_peak starts at +inf so
        # _lookahead_exit_boost is a no-op (1.0) until the first corner peak
        # is ever seen. Tracks LOCAL peaks (via _armed_for_next_peak, which
        # re-arms once the lookahead window clears near zero after a
        # corner) rather than a global running maximum -- a global maximum
        # would silently fail to re-trigger the exit boost on a second
        # corner of equal or lesser curvature than an earlier one, which is
        # a completely ordinary case (most tracks reuse corner radii), not
        # just a rare S-curve edge case.
        self._last_peak_kappa_abs: float = 0.0
        self._dist_since_peak: float = float("inf")
        self._armed_for_next_peak: bool = True

        # Rolling history of recently issued u_opt, oldest-first, used by
        # predict_ahead() to roll x0 forward through however many steps the
        # measured pose_age_s indicates (see compute()). Sized to the delay
        # cap since older entries are never needed.
        self._u_history: deque = deque(
            maxlen=self.params.max_delay_compensation_steps
        )

        # Filtered pose age and the currently-committed rollforward depth.
        # See the MPCParams.pose_age_lp_alpha / .n_delay_hysteresis notes above.
        self._pose_age_filtered: float | None = None
        self._n_delay: int = 0

        self.last_telemetry: dict = {}
        self._qp: dict | None = None

    def _build_qp(self) -> None:
        """
        Constructs the CVXPY problem using parameters. Built once to maximize 20Hz throughput.
        Matches optimiser.py exactly, including soft track boundaries.
        """
        nx, nu, N = self.nx, self.nu, self.N

        Ad_p    = cp.Parameter((nx, nx), name="Ad")
        Bd_p    = cp.Parameter((nx, nu), name="Bd")
        x0_p    = cp.Parameter(nx,       name="x0")
        uprev_p = cp.Parameter(nu,       name="u_prev")
        
        sqrtQ_param  = cp.Parameter((nx, 1), nonneg=True, name="sqrtQ")
        sqrtR_param  = cp.Parameter((nu, 1), nonneg=True, name="sqrtR")
        sqrtRr_param = cp.Parameter((nu, 1), nonneg=True, name="sqrtRr")
        weighted_u_prev_param = cp.Parameter(nu, name="weighted_u_prev")

        x     = cp.Variable((nx, N + 1))
        u     = cp.Variable((nu, N))
        slack = cp.Variable(N)  # Soft lane boundary constraint

        W_slack = 10000.0

        # Dynamics constraints
        constraints = [
            x[:, 0] == x0_p,
            x[:, 1:] == Ad_p @ x[:, :-1] + Bd_p @ u,
            u >= self.u_min[:, None],
            u <= self.u_max[:, None],
            x[0, :-1] <=  3.5 + slack,
            x[0, :-1] >= -3.5 - slack,
            u[:, 0] - uprev_p <=  self.du_max,
            u[:, 0] - uprev_p >= -self.du_max,
        ]

        if N > 1:
            du_hard = cp.diff(u, axis=1)
            constraints += [
                du_hard <=  self.du_max[:, None],
                du_hard >= -self.du_max[:, None],
            ]

        # Cost Formulation (Exact match to optimiser.py)
        cost  = cp.sum(cp.sum_squares(cp.multiply(sqrtQ_param, x)))
        # Terminal cost: extra weight on x[:,N] only. self.terminal_scale=1.0
        # makes this a no-op (see __init__ for the full explanation).
        if self.terminal_scale != 1.0:
            cost += (self.terminal_scale - 1.0) * cp.sum_squares(
                cp.multiply(sqrtQ_param[:, 0], x[:, N])
            )
        cost += cp.sum(cp.sum_squares(cp.multiply(sqrtR_param, u)))
        cost += W_slack * cp.sum_squares(slack)
        
        # Step-0 rate cost
        cost += cp.sum_squares(cp.multiply(sqrtRr_param[:, 0], u[:, 0]) - weighted_u_prev_param)

        # Subsequent rate cost
        if N > 1:
            du = cp.diff(u, axis=1)
            cost += cp.sum(cp.sum_squares(cp.multiply(sqrtRr_param, du)))

        prob = cp.Problem(cp.Minimize(cost), constraints)

        self._qp = {
            "prob":  prob,
            "Ad":    Ad_p,
            "Bd":    Bd_p,
            "x0":    x0_p,
            "u_prev": uprev_p,
            "sqrtQ": sqrtQ_param,
            "sqrtR": sqrtR_param,
            "sqrtRr": sqrtRr_param,
            "weighted_u_prev": weighted_u_prev_param,
            "u":     u,
        }

    def _discrete_model(self, v_x: float) -> tuple[np.ndarray, np.ndarray]:
        """
        ZOH exact discretization of the bicycle model.
        Forces dense sparsity pattern with epsilon to prevent OSQP reallocation.
        """
        v_x_safe = max(0.01, abs(v_x))
        m, Iz, lf, lr = self.m, self.Iz, self.lf, self.lr
        Cf, Cr        = self.Cf, self.Cr
        td, ta, dt    = self.tau_delta, self.tau_a, self.dt

        A_kin = np.ones((self.nx, self.nx)) * 1e-12
        A_dyn = np.ones((self.nx, self.nx)) * 1e-12

        A_kin[0, 2] = v_x_safe
        A_kin[2, 6] = v_x_safe / (lf + lr) 
        A_kin[4, 5] = 1.0
        A_kin[5, 7] = 1.0
        A_kin[6, 6] = -1.0 / td
        A_kin[7, 7] = -1.0 / ta

        A_dyn[0, 1] = 1.0
        A_dyn[1, 1] = -(2 * Cf + 2 * Cr) / (m * v_x_safe)
        A_dyn[1, 2] = (2 * Cf + 2 * Cr) / m
        A_dyn[1, 3] = (-2 * Cf * lf + 2 * Cr * lr) / (m * v_x_safe)
        A_dyn[1, 6] = (2 * Cf) / m
        A_dyn[2, 3] = 1.0
        A_dyn[3, 1] = (-2 * Cf * lf + 2 * Cr * lr) / (Iz * v_x_safe)
        A_dyn[3, 2] = (2 * Cf * lf - 2 * Cr * lr) / Iz
        A_dyn[3, 3] = -(2 * Cf * lf**2 + 2 * Cr * lr**2) / (Iz * v_x_safe)
        A_dyn[3, 6] = (2 * Cf * lf) / Iz
        A_dyn[4, 5] = 1.0   
        A_dyn[5, 7] = 1.0   
        A_dyn[6, 6] = -1.0 / td   
        A_dyn[7, 7] = -1.0 / ta   

        B = np.ones((self.nx, self.nu)) * 1e-12
        B[6, 0] = 1.0 / td
        B[7, 1] = 1.0 / ta

        alpha = np.clip((v_x - 1.0) / (2.5 - 1.0), 0.0, 1.0)
        A_c = (1.0 - alpha) * A_kin + alpha * A_dyn
        
        n_aug = self.nx + self.nu 
        M     = np.zeros((n_aug, n_aug))
        M[: self.nx, : self.nx] = A_c
        M[: self.nx, self.nx :] = B 

        eM = expm(M * dt)
        return eM[: self.nx, : self.nx], eM[: self.nx, self.nx :]

    def _error_state(
        self,
        path:          np.ndarray,
        car_pos:       np.ndarray,
        car_yaw:       float,
        car_speed:     float,
        car_yaw_rate:  float,
        desired_speed: float,
        car_vy:        float = 0.0,
    ) -> tuple[np.ndarray, float, dict]:
        """
        Calculates exact Frenet tracking errors to match offline evaluation.
        """
        fa = car_pos + self.lf * np.array([math.cos(car_yaw), math.sin(car_yaw)])
        base_dists = np.linalg.norm(path - fa, axis=1)
        base_idx   = int(np.argmin(base_dists))

        if base_idx < len(path) - 1:
            seg = path[base_idx + 1] - path[base_idx]
        else:
            seg = path[base_idx]     - path[base_idx - 1]

        seg_len = float(np.linalg.norm(seg))
        if seg_len < 1e-6:
            return np.zeros(self.nx), 0.0, {}

        # Orientation of the path segment
        path_yaw = math.atan2(seg[1], seg[0])

        # Perpendicular projection for lateral error (matches vehicle_physics.py's
        # plant_to_tracking_error(): e_y = e_y_proj directly) — NOT the full
        # Euclidean distance to the nearest path point, which only equals the
        # correct value when that point happens to be exactly abeam of the
        # car (otherwise overstates e_y, e.g. a 5 m along-track / 0.2 m
        # lateral offset would read back as e_y ~= 5.0 instead of 0.2).
        dx = fa[0] - path[base_idx][0]
        dy = fa[1] - path[base_idx][1]
        e_y_proj = dy * math.cos(path_yaw) - dx * math.sin(path_yaw)
        e_y = e_y_proj

        # ── Reference-heading rate limit (ref_heading_rate_limit_enabled) ──
        # Mirrors fsae_MPCTest/sim/rollout_core.py's planner branch exactly:
        # only e_psi is recomputed from the limited reference; e_y above is
        # left untouched (matches the offline choice not to also limit
        # lateral tracking). See the module-level comment about the
        # reference-heading rate limit for the mechanism and measurements.
        if self.params.ref_heading_rate_limit_enabled:
            if self._ref_psi_prev is None:
                path_yaw_limited = path_yaw
            else:
                max_step = (
                    math.radians(self.params.ref_heading_rise_rate_deg_s) * self.dt
                )
                delta = math.atan2(
                    math.sin(path_yaw - self._ref_psi_prev),
                    math.cos(path_yaw - self._ref_psi_prev),
                )
                delta = max(-max_step, min(max_step, delta))
                path_yaw_limited = self._ref_psi_prev + delta
            self._ref_psi_prev = path_yaw_limited
            path_yaw = path_yaw_limited

        # Heading error wrapped to [-pi, pi]
        e_psi = math.atan2(math.sin(car_yaw - path_yaw), math.cos(car_yaw - path_yaw))
        # Matches vehicle_physics.py's plant_to_tracking_error /
        # rollout_core.py's identical inline formula: e_y_dot needs both
        # body-frame velocity components, not just forward speed — omitting
        # car_vy silently drops this term whenever the car has real sideslip
        # (car_vy defaults to 0.0 for callers that don't measure it).
        e_yd  = car_speed * math.sin(e_psi) + car_vy * math.cos(e_psi)

        # Preview curvature lookup
        preview_dist = 1.0
        preview_idx  = base_idx
        accumulated  = 0.0
        for i in range(base_idx, len(path) - 1):
            accumulated += float(np.linalg.norm(path[i + 1] - path[i]))
            if accumulated >= preview_dist:
                preview_idx = i + 1
                break
        kappa = _curvature(path, preview_idx)

        # ── Lookahead curvature scan (adaptive_q_lookahead) ────────────────
        # Distinct from the single-point preview kappa above (which drives
        # _adaptive_R_rate/_steer_rate_anti_hunt): this scans the whole
        # speed-scaled window ahead for the largest |curvature|, so the Q
        # boost can anticipate a corner before the car reaches it. See the
        # module comment about the lookahead corner-anticipation Q-boost, and
        # MPCParams.adaptive_q_lookahead_time_s / _dist_min / _dist_max.
        lookahead_dist = float(np.clip(
            car_speed * self.params.adaptive_q_lookahead_time_s,
            self.params.adaptive_q_lookahead_dist_min,
            self.params.adaptive_q_lookahead_dist_max,
        ))
        (kappa_max_abs, lookahead_idx, lookahead_peak_dist,
         lookahead_heading_change) = _lookahead_curvature_profile(
            path, base_idx, lookahead_dist
        )

        x0 = np.array([
            e_y,
            e_yd,
            e_psi,
            car_yaw_rate,
            car_speed - desired_speed,
            0.0,
            self._delta_act,
            self._a_act,
        ])

        dbg = {
            "e_y":        e_y,
            "e_psi":      e_psi,
            "e_v":        x0[4],
            "kappa":      kappa,
            "base_idx":   base_idx,
            "preview_idx": preview_idx,
            "kappa_max_abs":       kappa_max_abs,
            "lookahead_idx":       lookahead_idx,
            "lookahead_peak_dist": lookahead_peak_dist,
            "lookahead_heading_change": lookahead_heading_change,
        }
        return x0, kappa, dbg

    def _update_n_delay(self, pose_age_s: float) -> int:
        """
        Convert a noisy measured pose age into a stable rollforward depth.

        Low-passes pose_age_s, then moves the committed integer step count
        only when the filtered estimate has clearly crossed a bin boundary
        (by more than MPCParams.n_delay_hysteresis steps). Without this, ordinary
        control-loop jitter flips n_delay between adjacent values every few
        ticks, and each flip discontinuously changes how far predict_ahead()
        rolls x0 forward — feeding a step disturbance into the QP at the
        control rate. Returns the step count to use this tick.
        """
        age = max(0.0, float(pose_age_s))

        if self._pose_age_filtered is None:
            # First sample: adopt it outright rather than easing up from zero,
            # so startup doesn't spend ~0.3 s under-compensating.
            self._pose_age_filtered = age
            self._n_delay = int(np.clip(
                round(age / self.dt),
                0, self.params.max_delay_compensation_steps))
            return self._n_delay

        self._pose_age_filtered += (
            self.params.pose_age_lp_alpha * (age - self._pose_age_filtered)
        )

        steps_f = self._pose_age_filtered / self.dt
        # Only leave the current bin once the estimate is past its edge plus
        # the deadband; otherwise hold, so an age sitting near a boundary
        # produces a constant n_delay instead of dithering.
        if abs(steps_f - self._n_delay) > 0.5 + self.params.n_delay_hysteresis:
            self._n_delay = int(np.clip(
                round(steps_f), 0, self.params.max_delay_compensation_steps))

        return self._n_delay

    def _update_lookahead_peak(self, kappa: float, car_speed: float) -> float:
        """
        Track distance travelled since the last LOCAL peak in CURRENT-POSITION
        curvature, for _lookahead_exit_boost's decay.

        Keyed on `kappa` (the ~1m preview curvature at the car's own
        position, same signal _adaptive_R_rate/_steer_rate_anti_hunt use),
        NOT kappa_max_abs (the full speed-scaled lookahead window,
        default 10-17m ahead at speed) -- changed 2026-08-11 after live
        telemetry showed the decay clock was firing 10-17m before the car
        physically reached the corner (kappa_max_abs detects a corner the
        moment it enters the far lookahead window, not when the car is
        actually in it), so _lookahead_exit_boost's whole 5m decay window
        (default MPCParams.adaptive_q_lookahead_exit_decay_dist) had already
        elapsed by the time the car was anywhere near the physical apex --
        the exit-heading boost was structurally inert on essentially every
        real corner taken at speed, not merely mistimed. kappa peaks at the
        car's own physical apex by construction, so decay now starts from
        the moment that matters.

        This is a rising-edge-after-a-clear detector, not "any new global
        maximum": _armed_for_next_peak only goes True again once kappa has
        dropped back to <= MPCParams.adaptive_q_lookahead_peak_hysteresis
        (i.e. the car itself is genuinely on a straight, not just that the
        far lookahead window is clear), and a peak only latches while armed.
        Comparing against the running maximum instead
        (kappa > self._last_peak_kappa_abs) would silently fail to
        re-trigger on a second corner of equal or LESSER curvature than an
        earlier one -- an ordinary case (most tracks reuse corner radii),
        not just a rare S-curve edge case. This way each distinct corner
        gets its own fresh decay cycle regardless of how its curvature
        compares to any previous corner's.

        Returns the updated _dist_since_peak for this tick.
        """
        kappa_abs = abs(kappa)
        if kappa_abs <= self.params.adaptive_q_lookahead_peak_hysteresis:
            self._armed_for_next_peak = True
        elif self._armed_for_next_peak:
            # Genuine new corner: the car was on a straight last tick (or
            # this is the very first corner ever seen) and is now curving --
            # this is the only case that resets the decay clock to 0.
            self._last_peak_kappa_abs = kappa_abs
            self._dist_since_peak = 0.0
            self._armed_for_next_peak = False
        elif kappa_abs > self._last_peak_kappa_abs:
            # Still inside the same corner but curvature is still rising
            # (approaching the apex) -- update the tracked peak MAGNITUDE (so
            # _lookahead_exit_boost's sharpness term stays accurate) without
            # touching the distance clock: the car has been continuously
            # turning this whole time, not restarting, so resetting distance
            # here would make boost_exit wobble non-monotonically through
            # corner entry instead of decaying cleanly after the true apex.
            self._last_peak_kappa_abs = kappa_abs
        self._dist_since_peak += abs(car_speed) * self.dt
        return self._dist_since_peak

    def _solve_qp(
        self,
        x0: np.ndarray,
        Ad: np.ndarray,
        Bd: np.ndarray,
        R_scaled:      np.ndarray,
        R_rate_scaled: np.ndarray,
        Q_scaled:      np.ndarray | None = None,
    ) -> np.ndarray:
        """
        Solves the MPC optimization problem utilizing warm starts.
        """
        if self._qp is None:
            self._build_qp()

        qp = self._qp
        qp["Ad"].value = Ad
        qp["Bd"].value = Bd
        qp["x0"].value = x0
        qp["u_prev"].value = self._u_prev

        # Format arrays for cp.sum_squares element-wise multiplication
        Q_for_solve = self.Q if Q_scaled is None else Q_scaled
        sqrtQ  = np.sqrt(np.clip(np.diag(Q_for_solve), 1e-6, 1e6))
        sqrtR  = np.sqrt(np.clip(np.diag(R_scaled), 1e-6, 1e6))
        sqrtRr = np.sqrt(np.clip(np.diag(R_rate_scaled), 1e-6, 1e6))
        
        qp["sqrtQ"].value = sqrtQ[:, None]
        qp["sqrtR"].value = sqrtR[:, None]
        qp["sqrtRr"].value = sqrtRr[:, None]
        qp["weighted_u_prev"].value = sqrtRr * self._u_prev

        # ── Primary solve: OSQP ──────
        qp["prob"].solve(
            solver=cp.OSQP,
            verbose=False,
            warm_start=True,
            eps_abs=1e-5,
            eps_rel=1e-5,
            max_iter=8000,
        )

        status = qp["prob"].status
        u_val  = qp["u"][:, 0].value

        if status == cp.OPTIMAL_INACCURATE and u_val is not None:
            print("[MPC] Warning: OSQP OPTIMAL_INACCURATE — Proceeding with viable solution.")
            return u_val.copy()

        if status == cp.OPTIMAL and u_val is not None:
            return u_val.copy()

        # ── Fallback: Clarabel ────────────────────────────────────────
        try:
            qp["prob"].solve(solver=cp.CLARABEL, verbose=False)
            status_fb = qp["prob"].status
            u_val_fb  = qp["u"][:, 0].value
            if status_fb in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE) and u_val_fb is not None:
                print("[MPC] Warning: OSQP failed, Clarabel succeeded.")
                return u_val_fb.copy()
        except cp.error.SolverError as exc:
            print(f"[MPC] Warning: Clarabel also failed: {exc!r}")

        return np.array([self._u_prev[0], -self.a_max_brake])

    def compute(
        self,
        path:          np.ndarray,
        car_pos:       np.ndarray,
        car_yaw:       float,
        car_speed:     float,
        desired_speed: float,
        car_yaw_rate:  float = 0.0,
        pose_age_s:    float = 0.0,
        car_vy:        float = 0.0,
    ) -> tuple[float, float, float]:
        """
        Run one full MPC control step: extract tracking error -> discretise
        the plant model at the current speed -> gain-schedule R/R_rate ->
        solve the QP -> integrate actuator lag -> convert to FSDS units.

        Parameters
        ----------
        path : np.ndarray, shape (n, 2)
            Planner centreline waypoints [x, y] in the global frame.
        car_pos : np.ndarray, shape (2,)
            Vehicle rear-axle-reference position [x, y] (global frame);
            front axle position is derived inside _error_state via self.lf.
        car_yaw : float
            Vehicle heading (rad, global frame).
        car_speed : float
            Vehicle forward (body-frame vx) speed (m/s); see _odom_cb note on
            how this is measured upstream. Used for the plant discretisation
            and gain scheduling, which are both parameterised on forward
            speed alone — see rollout_core.py's identical vx_true usage.
        desired_speed : float
            Planner's requested speed (m/s); low-pass filtered internally.
        car_yaw_rate : float, optional
            Measured yaw rate (rad/s), defaults to 0.0 if unavailable.
        car_vy : float, optional
            Body-frame lateral velocity (m/s), defaults to 0.0. Used only in
            _error_state's e_yd = vx*sin(e_psi) + vy*cos(e_psi) (matches
            vehicle_physics.py's plant_to_tracking_error / rollout_core.py's
            identical inline formula exactly). Callers that only have a
            speed magnitude (no separate vx/vy) should leave this at 0.0
            rather than passing the magnitude here.
        pose_age_s : float, optional
            How long ago (s) the pose above was actually measured (from the
            pose message's own timestamp, not callback receipt time — see
            mpc_controller.py's _pose_cb/_control_step). Converted to a step
            count and used to roll x0 forward through the commands already
            issued but not yet reflected in this pose (predict_ahead()),
            compensating for real, unknown/time-varying delay instead of
            assuming the measured state is current. Defaults to 0.0 (no
            compensation) for callers that don't measure it.

        Returns
        -------
        (steering, throttle, brake) : tuple of float
            steering in [-1, 1] (FSDS ControlCommand convention),
            throttle in [0, 1], brake in [0, 1] (throttle/brake mutually
            exclusive, split by the sign of a_cmd).

        Guard: if the path has fewer than 2 points, immediately returns a
        neutral/mild-braking command (0.0, 0.0, 0.5) without touching the
        QP or any internal state — the calling node's own path-staleness
        check is expected to normally catch this first
        (mpc_controller_standalone.py's Phase 2, or mpc_controller.py's
        equivalent stale-path guard).
        """
        if len(path) < 2:
            return 0.0, 0.0, 0.5   

        # Filter target speed to prevent impulse requests.
        alpha = 0.08
        if self._v_des_filtered is None:
            self._v_des_filtered = desired_speed
        self._v_des_filtered += alpha * (desired_speed - self._v_des_filtered)
        desired_speed = self._v_des_filtered

        x0, kappa, dbg = self._error_state(
            path, car_pos, car_yaw, car_speed, car_yaw_rate, desired_speed, car_vy,
        )

        Ad, Bd = self._discrete_model(car_speed)

        # ── Delay compensation ───────────────────────────────────────────
        # x0 reflects the pose as measured pose_age_s seconds ago. Roll it
        # forward through however many of the recently-issued commands
        # (_u_history) fall within that window, so the QP solves against
        # the state it will actually face rather than a stale one. Clamp
        # the step count rather than trusting an arbitrarily large measured
        # age blindly (e.g. a perception hiccup) — see
        # MPCParams.max_delay_compensation_steps.
        # The step count is filtered and hysteresis-gated rather than taken
        # raw from this tick's pose_age_s — see the MPCParams.pose_age_lp_alpha /
        # .n_delay_hysteresis notes at module scope for why a jittering
        # n_delay is itself a source of oscillation.
        if self.delay_compensation_enabled:
            n_delay = self._update_n_delay(pose_age_s)
            if n_delay > 0 and len(self._u_history) > 0:
                pending_cmds = list(self._u_history)[-n_delay:]
                x0 = predict_ahead(
                    x0, Ad, Bd, pending_cmds,
                    epsi_clip=self.params.predict_epsi_clip,
                )
        else:
            n_delay = 0

        # kappa_max_abs (lookahead) is computed unconditionally in
        # _error_state, independent of adaptive_q_lookahead_enabled -- safe
        # to read here for _adaptive_R_rate's entering-floor even though
        # the Q-boost application below is still gated on that flag.
        kappa_max_abs = dbg.get("kappa_max_abs", 0.0)

        # ── Adaptive-feature trace ────────────────────────────────────────
        # Every adaptive multiplier below is recorded into `adapt` as it is
        # applied, then merged into last_telemetry so a live log shows WHICH
        # feature fired and BY HOW MUCH on every tick -- not just the
        # resulting error. Without this, attributing a tracking failure to a
        # specific boost meant re-deriving each function's value by hand from
        # kappa/speed after the fact, which is how the demand-scaling
        # saturation bug (all real corners sitting in the flat 17-62% region
        # of the old 1-1/(1+8k) curve) went unnoticed for as long as it did.
        # These are pure observations of values already computed -- adding or
        # removing a trace entry must never change a control output.
        adapt: dict[str, float] = {}

        R_scaled = _adaptive_R_scaling(car_speed, self.R)
        adapt["m_R_speed"] = float(R_scaled[0, 0] / self.R[0, 0]) if self.R[0, 0] else 1.0
        if self.steer_effort_straight_boost_enabled:
            R_scaled = R_scaled.copy()
            m = _steer_effort_straight_boost(
                kappa_max_abs,
                boost_max=self.params.steer_effort_straight_boost_max,
                k=self.params.steer_effort_straight_k,
            )
            R_scaled[0, 0] *= m
            adapt["m_R_straight"] = m
        else:
            adapt["m_R_straight"] = 1.0
        R_rate_scaled = _adaptive_R_rate(
            kappa, self.R_rate,
            enable_in_corners=self.adaptive_r_rate_enable_in_corners,
            kappa_max_abs=kappa_max_abs,
            during_floor=self.params.adaptive_r_rate_during_floor,
            entering_floor=self.params.adaptive_r_rate_entering_floor,
            k_entering=self.params.adaptive_r_rate_k_entering,
        )
        adapt["m_Rrate_corner"] = (
            float(R_rate_scaled[0, 0] / self.R_rate[0, 0]) if self.R_rate[0, 0] else 1.0
        )
        _rr_before_hunt = float(R_rate_scaled[0, 0])
        R_rate_scaled = _steer_rate_anti_hunt(
            kappa, x0[0], R_rate_scaled, self.steer_rate_anti_hunt_enabled,
            e_psi=x0[2],
            boost_max=self.params.anti_hunt_boost_max,
        )
        adapt["m_Rrate_antihunt"] = (
            float(R_rate_scaled[0, 0] / _rr_before_hunt) if _rr_before_hunt else 1.0
        )
        # ── Lookahead corner-anticipation Q-boost (adaptive_q_lookahead) ───
        # Applied to self.Q FIRST, then _adaptive_Q_scaling's centred-
        # softening multiplies on top of the result below -- see the module
        # comment about the lookahead corner-anticipation Q-boost for why this
        # ordering avoids the corner boost being silently cancelled by the
        # centred-softening floor while keeping both continuous.
        Q_base = self.Q
        if self.adaptive_q_lookahead_enabled:
            self._update_lookahead_peak(kappa, car_speed)
            Q_base = self.Q.copy()
            m = _lookahead_approach_boost(
                kappa_max_abs, car_speed,
                boost_max=self.params.adaptive_q_lookahead_q_boost_max,
                demand_normalised=self.params.adaptive_q_demand_normalised,
                k_approach=self.params.adaptive_q_lookahead_k_approach,
                demand_half=self.params.adaptive_q_demand_half,
                ceiling_flat=self.params.alat_ceiling_flat,
                ceiling_slope=self.params.alat_ceiling_slope,
                ceiling_intercept=self.params.alat_ceiling_intercept,
            )
            Q_base[0, 0] *= m
            adapt["m_Q_ey_approach"] = m
            m = _lookahead_straight_lateral_reduce(
                kappa_max_abs,
                ey_floor=self.params.adaptive_q_straight_ey_floor,
                ey_k=self.params.adaptive_q_straight_ey_k,
            )
            Q_base[0, 0] *= m
            adapt["m_Q_ey_straight"] = m
            m = _lookahead_epsi_approach_boost(
                kappa_max_abs, car_speed,
                boost_max=self.params.adaptive_q_lookahead_epsi_approach_boost_max,
                demand_normalised=self.params.adaptive_q_demand_normalised,
                k_epsi_approach=self.params.adaptive_q_lookahead_k_epsi_approach,
                demand_half=self.params.adaptive_q_demand_half,
                ceiling_flat=self.params.alat_ceiling_flat,
                ceiling_slope=self.params.alat_ceiling_slope,
                ceiling_intercept=self.params.alat_ceiling_intercept,
            )
            Q_base[2, 2] *= m
            adapt["m_Q_epsi_approach"] = m
            m = _lookahead_exit_boost(
                self._last_peak_kappa_abs, self._dist_since_peak,
                decay_dist=self.params.adaptive_q_lookahead_exit_decay_dist,
                k_exit_norm=self.params.adaptive_q_lookahead_k_exit_norm,
                boost_max=self.params.adaptive_q_lookahead_epsi_boost_max,
            )
            Q_base[2, 2] *= m
            adapt["m_Q_epsi_exit"] = m
            m = _lookahead_straight_boost(
                kappa_max_abs,
                self.params.adaptive_q_straight_epsi_boost_max,
                self.params.adaptive_q_straight_k,
            )
            Q_base[2, 2] *= m
            adapt["m_Q_epsi_straight"] = m
            m = _lookahead_yaw_rate_relax(
                kappa_max_abs, car_speed,
                floor=self.params.adaptive_q_lookahead_r_floor,
                demand_normalised=self.params.adaptive_q_demand_normalised,
                k_r_relax=self.params.adaptive_q_lookahead_k_r_relax,
                demand_half=self.params.adaptive_q_demand_half,
                ceiling_flat=self.params.alat_ceiling_flat,
                ceiling_slope=self.params.alat_ceiling_slope,
                ceiling_intercept=self.params.alat_ceiling_intercept,
            )
            Q_base[3, 3] *= m
            adapt["m_Q_r_relax"] = m
            m = _lookahead_straight_boost(
                kappa_max_abs,
                self.params.adaptive_q_straight_r_boost_max,
                self.params.adaptive_q_straight_k,
            )
            Q_base[3, 3] *= m
            adapt["m_Q_r_straight"] = m

            # ── U-turn extra commitment (see MPCParams.adaptive_q_uturn_*) ──
            # Scored from accumulated heading change, NOT peak curvature, so
            # a long gradual U-turn is no longer under-boosted just because
            # its radius is large. Zero below 90 deg, so ordinary corners
            # (including the sudden-corner case that already works) are
            # completely unaffected. Only meaningful while steering is still
            # unsaturated on approach -- see the scope note on the constants.
            uturn = _uturn_severity(
                dbg.get("lookahead_heading_change", 0.0),
                thresh_rad=self.params.adaptive_q_uturn_heading_thresh_rad,
                sat_rad=self.params.adaptive_q_uturn_heading_sat_rad,
            )
            adapt["uturn_severity"] = uturn
            if uturn > 0.0:
                m = _uturn_boost(uturn, self.params.adaptive_q_uturn_ey_boost_max)
                Q_base[0, 0] *= m
                adapt["m_Q_ey_uturn"] = m
                m = _uturn_boost(uturn, self.params.adaptive_q_uturn_epsi_boost_max)
                Q_base[2, 2] *= m
                adapt["m_Q_epsi_uturn"] = m
                m = _uturn_boost(uturn, self.params.adaptive_q_uturn_r_relax_floor)
                Q_base[3, 3] *= m
                adapt["m_Q_r_uturn"] = m

        # x0[0] is e_y (delay-compensated, if n_delay>0 above rolled it
        # forward) -- compute() has no bare e_y in scope, only x0.
        Q_scaled      = _adaptive_Q_scaling(x0[0], Q_base, self.adaptive_q_scaling_enabled)
        adapt["m_Q_ey_soften"] = (
            float(Q_scaled[0, 0] / Q_base[0, 0]) if Q_base[0, 0] else 1.0
        )

        # Defaults for the U-turn entries, which are only written when the
        # detector actually fires -- a missing key would otherwise become an
        # empty CSV cell and silently break any downstream numeric parse.
        adapt.setdefault("m_Q_ey_uturn", 1.0)
        adapt.setdefault("m_Q_epsi_uturn", 1.0)
        adapt.setdefault("m_Q_r_uturn", 1.0)
        adapt.setdefault("uturn_severity", 0.0)

        # Corner demand (see _corner_demand): kappa_max_abs expressed as a
        # fraction of what the FSDS lateral-acceleration ceiling actually
        # permits at this speed. This is the single most diagnostic scalar in
        # the log -- demand > 1 means the corner ahead is not achievable at
        # the current speed no matter how the weights are set, so a wide line
        # there is a speed-profile problem, not a gain problem.
        _demand = _corner_demand(
            kappa_max_abs, car_speed,
            self.params.alat_ceiling_flat,
            self.params.alat_ceiling_slope,
            self.params.alat_ceiling_intercept,
        )
        adapt["corner_demand"] = _demand
        adapt["demand_frac"]   = _demand_frac(
            _demand, self.params.adaptive_q_demand_half
        )
        adapt["alat_ceiling"]  = _alat_ceiling_at(
            car_speed,
            self.params.alat_ceiling_flat,
            self.params.alat_ceiling_slope,
            self.params.alat_ceiling_intercept,
        )
        adapt["dist_since_peak"] = float(
            min(self._dist_since_peak, 999.0)   # starts at +inf; keep CSV finite
        )
        adapt["last_peak_kappa"] = float(self._last_peak_kappa_abs)

        # Absolute post-scaling weights actually handed to the QP. The
        # multipliers above say how hard each feature pushed; these say where
        # the weights ended up, which is what makes two runs comparable even
        # if a base weight in self.Q changed between them.
        adapt["Q_ey_eff"]   = float(Q_scaled[0, 0])
        adapt["Q_epsi_eff"] = float(Q_scaled[2, 2])
        adapt["Q_r_eff"]    = float(Q_scaled[3, 3])
        adapt["R_steer_eff"]      = float(R_scaled[0, 0])
        adapt["Rrate_steer_eff"]  = float(R_rate_scaled[0, 0])

        # Wall-clock the QP so the log can distinguish "the solver is slow"
        # from "the pipeline upstream of us is slow" — see solve_ms in
        # telemetry_logger's column reference.
        _t_solve0 = time.perf_counter()
        u_opt = self._solve_qp(x0, Ad, Bd, R_scaled, R_rate_scaled, Q_scaled)
        solve_ms = (time.perf_counter() - _t_solve0) * 1e3
        self._u_history.append(u_opt.copy())

        # ── EXACT ZOH ACTUATOR INTEGRATION ────────────────────────────
        # Prevents explicit Euler instability when dt > tau_a
        exp_delta = math.exp(-self.dt / self.tau_delta)
        exp_a     = math.exp(-self.dt / self.tau_a)
        
        self._delta_act = self._delta_act * exp_delta + u_opt[0] * (1.0 - exp_delta)
        self._a_act     = self._a_act * exp_a         + u_opt[1] * (1.0 - exp_a)
        
        self._u_prev    = u_opt.copy()
        # ──────────────────────────────────────────────────────────────

        delta_cmd = float(np.clip(u_opt[0], -MAX_STEER_RAD, MAX_STEER_RAD))
        a_cmd     = float(u_opt[1])
        steering  = float(np.clip(-delta_cmd / MAX_STEER_RAD, -1.0, 1.0))

        if a_cmd >= 0.0:
            throttle = float(np.clip(a_cmd / self.a_max, 0.0, 1.0))
            brake    = 0.0
        else:
            throttle = 0.0
            brake    = float(np.clip(-a_cmd / self.a_max_brake, 0.0, 1.0))

        self.last_telemetry = {
            **dbg,
            # Per-feature adaptive multipliers + demand diagnostics; see the
            # "Adaptive-feature trace" comment above compute()'s R_scaled.
            **adapt,
            # Delay diagnostics — the controller's own view of how stale its
            # inputs were and how far it rolled the state forward to compensate.
            # Logged so a live run can be checked against the offline sim's
            # DELAY_STEPS assumption instead of it being taken on trust.
            "pose_age_s":    float(pose_age_s),
            "n_delay":       int(n_delay),
            "solve_ms":      float(solve_ms),
            "car_speed":     car_speed,
            "desired_speed": desired_speed,
            "steering":      steering,
            "throttle":      throttle,
            "brake":         brake,
            "delta_cmd":     delta_cmd,
            "a_cmd":         a_cmd,
            "delta_act":     self._delta_act,
            "a_act":         self._a_act,
        }

        return steering, throttle, brake

    def reset(self) -> None:
        """
        Clears the controller's internal state history, forcing the QP solver
        to discard its warm start and actuator lag tracking. 
        """
        self._delta_act       = 0.0
        self._a_act           = 0.0
        self._u_prev          = np.zeros(self.nu)
        self._v_des_filtered  = None
        self._ref_psi_prev    = None
        self._u_history.clear()
        self._pose_age_filtered = None
        self._n_delay           = 0