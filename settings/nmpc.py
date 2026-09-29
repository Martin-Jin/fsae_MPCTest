"""
settings/nmpc.py — Nonlinear MPC (NMPC): weight overrides (-1.0 = inherit
the LMPC weight above), solver internals (horizon, SQP iterations, RK/
Jacobian substeps), the alat-ceiling plant model, and every NMPC-only
feature flag (rate-shaping zones, progress term, latency compensation).
"""
import numpy as np

# ------------------------------------------------------------------------------
# Nonlinear MPC (NMPC) — a SECOND controller (controller/nmpc/)
# ------------------------------------------------------------------------------
# [NMPC only] USE_NMPC — "Which controller does the closed-loop rollout actually solve?"
# False (default) = controller/optimiser.py's solve_mpc(), the linear
# time-varying QP everything above this section tunes. True =
# controller/nmpc_optimiser.py's NMPCController, a Frenet-frame NONLINEAR MPC
# (arc length is a state, path curvature kappa(s) is looked up from it
# directly, instead of the linear model's e_psi_dot = yaw-rate-only). See
# docs/junior_project_mpc_docs.md's §4.2 for the plain-language explanation
# of why the linear model needs this at all, and docs/tuning.md's NMPC
# section for the tuning surface.
#
# Closes a structural gap the linear model has: with the car exactly on-line
# and on-heading approaching a corner, the linear model's own horizon
# rollout predicts staying at 0 forever (checked directly, not assumed —
# see tuner/nmpc_offline_check.py's turn-in test), so no amount of cost
# reweighting can make it commit to steering before real tracking error
# exists. The WHOLE adaptive-gain-shape section below (and every
# ADAPTIVE_*_ENABLED flag) exists to synthesise anticipation this linear
# model cannot produce on its own -- none of it applies when USE_NMPC=True
# (the nonlinear model anticipates structurally, so layering the same
# mechanisms on top would double-count an effect that's now built in).
#
# Mirrors the live side's `use_nmpc` node parameter (mpc_params.py's "NMPC
# weight overrides" section handles the WEIGHTS below; this file's NMPC_*
# constants mirror nmpc_params.py's remaining structural/solver fields) --
# kept numerically identical by hand, the same discipline as every other
# constant in this file, per CLAUDE.md's parity rule. Land this off; prove
# it live before flipping the live side's default.
USE_NMPC = False

# ── NMPC weight overrides ────────────────────────────────────────────────
# [NMPC only] -1.0 = inherit the corresponding Q_diag/R_diag/R_rate_diag/TERMINAL_Q_SCALE
# entry above (the SAME weight set a tuner run passes to run_core_rollout,
# so a CMA-ES sweep reaches the NMPC's weights exactly the way it reaches
# the LTV-QP's). Set a real value to diverge only that one weight for the
# NMPC without touching the LTV-QP's tuned set.
#
# NMPC_Q_EPSI_DOT is the one weight whose MEANING differs from its LTV-QP
# counterpart (Q_diag[3]): the nonlinear model's 4th output is heading-error
# RATE (r - kappa(s)*s_dot), not absolute yaw rate. Penalising absolute yaw
# rate in a curvature-aware model would penalise the yaw rate the car MUST
# hold to follow a corner (r = kappa*v) -- the exact opposite of what's
# wanted. Same slot, different regressor: expect this one to need its own
# sweep rather than inheriting Q_diag[3] unchanged.
# 7.5 (above the inherited 6.35) is the live-tested value and the current
# live default: score-neutral-to-better (0.454-0.486 across three runs) with a
# lower peak lateral error. Keep in step with launch_all.sh's NMPC_Q_E_Y.
#
# CAUTION on measuring this: raw drift-episode COUNTS across runs of different
# length are misleading (a 89.9 s run shows ~48 episodes where a 53 s run
# shows ~29 for the SAME config). Rate-normalised, every configuration tried
# on this track sits at ~31.5 drift episodes/min. Normalise by duration, and
# treat one run's lap time as noisy -- the same config gave 53.29 s and
# 47.99 s.
NMPC_Q_E_Y       = -1.0

NMPC_Q_E_YD      = -1.0

NMPC_Q_E_PSI     = -1.0

NMPC_Q_EPSI_DOT  = -1.0

NMPC_Q_E_V       = -1.0

NMPC_R_DELTA     = -1.0

NMPC_R_A_ACCEL   = -1.0

NMPC_R_A_BRAKE   = -1.0

NMPC_R_RATE_DELTA = -1.0

NMPC_R_RATE_A    = -1.0

NMPC_TERMINAL_SCALE = -1.0

# [NMPC only, EXPERIMENTAL] Reuses model_utils.steer_rate_anti_hunt
# (the same function STEER_RATE_ANTI_HUNT_ENABLED above already gates for the
# LTV-QP) on the NMPC too -- independent flag, not inherited, since the live
# nmpc_core.py module docstring documents a deliberate decision NOT to port
# mpc_core's adaptive gain-schedule family onto the curvature-aware NMPC
# (double-count risk); anti-hunt is offered separately because it only ever
# makes steering-rate MORE expensive when already centred/aligned/uncurving,
# the opposite direction from anticipation. UNVALIDATED for the NMPC. Note:
# model_utils.steer_rate_anti_hunt hardcodes boost_max=6.0 internally (no
# settings.py override exists for EITHER controller's use of it, pre-existing
# limitation, not introduced here) -- ANTI_HUNT_BOOST_MAX has no live
# offline-side constant to parameterise this with yet.
NMPC_STEER_RATE_ANTI_HUNT_ENABLED = False

# [NMPC only, EXPERIMENTAL] A narrower, ALTERNATIVE port of
# the LTV-QP's corner_factor family -- blends R_rate[0,0] between
# NMPC_RRATE_STEER_STRAIGHT/_CORNER by CURRENT curvature alone
# (model_utils._corner_factor/_blend, imported not reimplemented). Unlike the
# rest of that family (Q_EY/Q_EPSI/Q_R/R_STEER, deliberately excluded above),
# only R_rate[steer] is touched, to limit how much of the "no adaptive gain
# schedule" reasoning this overrides. NOT composed with
# NMPC_STEER_RATE_ANTI_HUNT_ENABLED -- takes priority over it when both are
# set; use one or the other, not both. UNVALIDATED for the NMPC.
NMPC_CORNER_RRATE_BLEND_ENABLED = False

# Saturation rate of _corner_factor = 1 - 1/(1 + k*|kappa|), shared by the
# corner blend above and NMPC_RRATE_ZONE_* below. CORNER_FACTOR_K's 8.0 is
# calibrated for the LTV-QP's soft Q-blending, where partial engagement is
# fine. The zone schedule instead NEEDS this to saturate, because its corner
# floor is only reached as corner_frac -> 1, and at k=8 that is unreachable
# on a track whose tightest corner is |kappa|~0.2: corner_frac needs
# |kappa|=1.125 to reach 0.9, so it tops out near 0.63 and the zone
# degenerates into a mild global rate boost.
#
# Scale this with the TRACK's max curvature, not by feel:
#   k ~= target_corner_frac / ((1 - target_corner_frac) * kappa_max)
# CAUTION: raising this past 27 does not fix mid-corner lateral drift --
# k=60 was live-tested WORSE (see `docs/reference/README.md`'s "Three-zone rate
# schedule"). It lowers the corner weight as intended but drift barely moves,
# which is the evidence that the steering-RATE cost is not what limits
# turn-in on this track.
# 27.0 puts |kappa|=0.209 (comp_test_map_3's tightest) at corner_frac 0.85.
NMPC_CORNER_FACTOR_K = 27.0

NMPC_RRATE_STEER_STRAIGHT = -1.0   # -1 = inherit RRATE_STEER_STRAIGHT

NMPC_RRATE_STEER_CORNER = -1.0     # -1 = inherit RRATE_STEER_CORNER

# [NMPC only, EXPERIMENTAL] Applies
# model_utils.reversal_penalty_boost (the same function
# REVERSAL_PENALTY_ENABLED above already gates for the LTV-QP) to the NMPC too
# -- gain-scheduled per tick from LAST tick's steering command and applied
# uniformly across the horizon, exactly like the two flags above. Unlike those
# two, this one COMPOSES with either of them rather than replacing them: it is
# keyed on u_prev, not curvature/e_y/e_psi, so all three multipliers stack onto
# the same R_rate[0,0] (see nmpc_optimiser.compute_step's rrate_steer_current).
# Mirrors the live MPCParams.nmpc_reversal_penalty_* override fields.
# UNVALIDATED on the car; offline-A/B'd only.
# [NMPC only, EXPERIMENTAL] Discount the steering-RATE cost at the NEAR
# horizon stages (a linear ramp from NMPC_RRATE_STAGE_NEAR at stage 0 to 1.0
# at the last stage), so a first turn-in input is cheap while a sustained
# oscillation still pays close to full price -- see
# controller/nmpc_optimiser.py::_rrate_stage_ramp for the reasoning, and
# docs/steering_turn_in_upgrade_options.md (Option 1) for why this is keyed
# on horizon POSITION rather than measured curvature/error.
# NEAR = 1.0 is an exact no-op. Composes with the three flags below (they set
# the rate weight's magnitude; this shapes it across stages).
# [NMPC only, EXPERIMENTAL] Continuous three-zone schedule on the
# steering-RATE cost, driven by CURRENT curvature and the peak curvature the
# HORIZON predicts ahead: boost on a true straight, ease on the approach to a
# corner the horizon can see, floor through the corner itself. Smooth surface,
# no thresholds -- degrades to the corner value on a continuously-winding
# road. See controller/nmpc_optimiser.py::_rrate_zone_scale.
# Multiplies whatever r_rate_delta is (unlike NMPC_CORNER_RRATE_BLEND_ENABLED,
# which OVERWRITES it), so it composes with the shipped 52.5 rather than
# discarding it.
# [NMPC only, EXPERIMENTAL] Steering-JERK weight: penalises the SECOND
# difference of the steering command (steering acceleration) instead of only
# the first. A steady ramp into a corner has near-zero second difference and
# is nearly free; an alternating wiggle is expensive. Measured on live data,
# reversals carry ~4.3x the |d2| of same-direction ramps vs only ~1.9x the
# |d1|, so this separates chatter from turn-in about twice as sharply as the
# rate cost can. 0.0 disables the term entirely (no Hessian contribution).
# See controller/nmpc_optimiser.py::_build_qp's _E2 comment.
NMPC_RJERK_DELTA = 150.0

NMPC_RJERK_A = 0.0

# Three-zone steering-rate schedule: boost on straights, ease when a corner
# is visible ahead (the turn-in release), floor mid-corner. MULTIPLIES
# R_rate_diag[0] rather than overwriting it, so it composes with the tuned
# 52.5 instead of discarding it.
#
# CAUTION: the endpoints below are only REACHED if NMPC_CORNER_FACTOR_K
# saturates over the track's curvature range -- see its comment above. Read
# a run's m_Rrate_zone column before concluding these values did anything;
# if it never approaches FLOOR_CORNER, k is the thing to fix, not these.
NMPC_RRATE_ZONE_ENABLED = True

NMPC_RRATE_ZONE_BOOST_STRAIGHT = 2.0    # x r_rate on a true straight

NMPC_RRATE_ZONE_EASE_APPROACH = 0.8    # x r_rate when a corner is AHEAD but not here yet (0.35 DNFs offline -- see `docs/reference/`)

NMPC_RRATE_ZONE_FLOOR_CORNER = 0.15     # x r_rate mid-corner

NMPC_RRATE_STAGE_RAMP_ENABLED = False

NMPC_RRATE_STAGE_NEAR = 0.15

NMPC_REVERSAL_PENALTY_ENABLED = False

NMPC_REVERSAL_PENALTY_BOOST_MAX = -1.0  # -1 = inherit REVERSAL_PENALTY_BOOST_MAX

NMPC_REVERSAL_PENALTY_K = -1.0          # -1 = inherit REVERSAL_PENALTY_K

# ── Structural / solver settings ─────────────────────────────────────────
# [NMPC only] No LTV-QP counterpart to inherit from (there's no "linear horizon length"
# concept these could default to) -- these are genuine NMPC-only constants,
# each measured rather than guessed; see the live repo's
# late_turn_in_investigation.md Part 16 §16.7 for the sweep behind the
# horizon/iteration defaults specifically.
NMPC_HORIZON = 20                          # steps (x DT=0.05s = 1.0s). Measured BETTER than

                                            # N_HORIZON=35 on tracking: the model is optimistic
                                            # (linear tyres, no suspension), and that mismatch
                                            # compounds over a longer horizon.
NMPC_SQP_ITERS = 1                         # Gauss-Newton iterations/tick (real-time-iteration

                                            # style -- the warm start carries convergence
                                            # across ticks). Measured better AND ~2x cheaper
                                            # than 2.
NMPC_SOLVE_BUDGET_MS = 25.0                # wall-clock budget/tick; ships the best feasible

                                            # iterate rather than overrunning DT=0.05s. Checked
                                            # both before an SQP iteration and before each
                                            # backtracking trial (the latter is what actually
                                            # bounds tick time at sqp_iters=1, fixed 2026-09-14).
NMPC_RK_SUBSTEPS = 4                       # RK4 substeps in the prediction rollout. Two stiff

                                            # modes set this: tau_a=0.02s against DT=0.05s (2
                                            # covers that), and the (v_y, r) lateral dynamics,
                                            # which stiffen as 1/v_x. Was 2, found outright
                                            # UNSTABLE (infinitesimal-perturbation growth up to
                                            # ~260x over the horizon, measured directly via
                                            # _rollout, not just the Jacobian) across roughly
                                            # 2.25-3.75 m/s, making the prediction garbage and
                                            # freezing the controller at exactly zero output.
                                            # CORRECTED 2026-09-15: an earlier note here claimed
                                            # 3 substeps also fails at 2.50 m/s exactly -- a
                                            # direct re-measurement (same perturbation-growth
                                            # test, all 8 states individually perturbed, several
                                            # control-sequence shapes) found 3 substeps fully
                                            # stable (<=1.6x growth) everywhere tested in and
                                            # around that band. The original claim's basis is not
                                            # reproduced; treat 4 as still the safe default below
                                            # NMPC_RK_GATE_SPEED, but 3 is confirmed safe above
                                            # it (see NMPC_RK_SUBSTEPS_FAST). See docs/logs/
                                            # nmpc_low_speed_accel_stall_investigation.md.
NMPC_JAC_SUBSTEPS = 4                       # RK4 substeps for the QP's sensitivity Jacobians

                                            # only (never the prediction itself). Was 1, found
                                            # numerically unstable below ~6.5-7 m/s (not just
                                            # less accurate) -- see docs/logs/
                                            # nmpc_low_speed_accel_stall_investigation.md.
NMPC_JAC_GATE_SPEED = 8.0                   # at/above this speed (slowest predicted horizon

                                            # stage, not instantaneous), NMPC_JAC_SUBSTEPS_FAST
                                            # is used instead of NMPC_JAC_SUBSTEPS -- the
                                            # instability above is confined to low speed
                                            # (measured max|A_k| 2.41e2 at 2.5 m/s vs 4.06 at
                                            # 8 m/s, js=1 vs converged js=4), so the fix's cost
                                            # need not apply where it was never needed.
NMPC_JAC_SUBSTEPS_FAST = 2                  # tracks the converged (4-substep) sensitivity

                                            # closely with no divergence at/above the gate speed
                                            # (3.07 vs 3.18 at 8 m/s, 4.90 vs 4.93 at 14 m/s). 1
                                            # is NOT safe here: inaccurate rather than unstable
                                            # at speed (1.30 vs converged 3.85 at 10 m/s).
NMPC_RK_GATE_SPEED = 4.0                    # same technique as NMPC_JAC_GATE_SPEED, applied to

                                            # the ROLLOUT itself (per predicted stage, not the
                                            # whole horizon, since _rollout builds X
                                            # incrementally). The rollout's instability is
                                            # confined to a NARROWER band than the Jacobian's
                                            # (measured stable, <=1x growth, at and above ~3.75
                                            # m/s vs the Jacobian's ~8 m/s), so this gate opens
                                            # earlier.
NMPC_RK_SUBSTEPS_FAST = 3                   # NOT 2 -- 2 is the one substep count confirmed

                                            # unstable in the 2.25-3.75 m/s band. 3 is fully
                                            # converged (<=1.6x growth) everywhere measured at
                                            # and above the gate speed.
NMPC_STANDSTILL_STEER_DAMP_ENABLED = True   # damp stage-0 steering effort while the car is

                                            # measurably stationary. At v_x=0 steering cannot
                                            # move the car, but the SQP minimises one cost
                                            # summed over the whole horizon and the predicted
                                            # v_x leaves zero by stage 1, so the optimiser
                                            # pre-commits U[0] toward what helps later stages
                                            # and the car launches already turned (measured
                                            # live: ~-6.8 deg of steer built up over the ~1s
                                            # before the car physically moves).
NMPC_STANDSTILL_SPEED = 0.5                 # m/s -- below this MEASURED speed, stage 0 only

                                            # is damped. Keyed on the measurement so it
                                            # disengages as soon as the car moves.
NMPC_STANDSTILL_FADE_SPEED = 3.0            # m/s -- speed at which the damping has faded

                                            # fully back to 1x. Held at full scale below
                                            # NMPC_STANDSTILL_SPEED, ramped linearly to 1.0
                                            # here, so the weight never changes in one step.
                                            # A hard release put the whole change into a
                                            # single tick where the car is most sensitive:
                                            # measured live, steering ran -1.8 to -12.9 deg
                                            # over the six ticks right after the release.
                                            # Set <= NMPC_STANDSTILL_SPEED for a hard cutoff.
NMPC_STANDSTILL_STEER_R_SCALE = 200.0       # multiplier on r_delta for stage 0 only. Stage-0-

                                            # only is a weaker lever than raising r_delta
                                            # across the horizon, so it needs a larger number:
                                            # pre-load -6.71 deg at 1x, -2.41 at 20x, -0.33 at
                                            # 200x, halving per doubling. Live-validated at
                                            # 200x -- peak steer through the fade band
                                            # 10.68 -> 6.79 deg, reversals 5.78 -> 3.82 pct.
                                            # Tuning value, re-check live.
NMPC_TRUST_DELTA_RAD = np.radians(9.0)      # per-iteration steering trust region = MAX_STEER's

                                            # own slew-rate limit per tick (180 deg/s * DT) --
                                            # reused, not invented.
NMPC_TRUST_A = 0.6                          # per-iteration accel trust region = du_max[1].

NMPC_BACKTRACK_MAX = 2                      # step halvings if a full SQP step increases the

                                            # true nonlinear cost (divergence guard).
NMPC_TRACK_HALFWIDTH = 3.35                  # soft |e_y| bound with slack (both quadratic and

                                            # linear), matching controller/optimiser.py's LTV-QP
                                            # +-3.5m literal. Was narrowed to 3.0 on 2026-09-21 for
                                            # the progress-term experiment, then REVERTED the same
                                            # day: this field is read unconditionally (not gated on
                                            # progress_enabled), so narrowing it also tightened
                                            # ordinary tracking mode and measurably hurt it.
NMPC_SLACK_WEIGHT = 10000.0                 # matches controller/optimiser.py's W_SLACK.

NMPC_CURVATURE_DENSE_STEP = 0.5             # kappa(s)/heading-reference smoothing -- same

NMPC_CURVATURE_SMOOTH_W = 3                 # denoise precedent as sim/speed_profile.py's

                                            # curvature_speed() (dense_step=0.5, w=3), not new
                                            # smoothing constants.
NMPC_KAPPA_CLIP = 0.5                       # hard |kappa(s)| clamp -- a 2m-radius guard,

                                            # inert on any real track line, only catches a
                                            # degenerate/spiking path.
NMPC_KAPPA_RATE_MAX = 2.0                   # max tick-to-tick change of kappa(s) at matching

                                            # arc-length samples, live-planner mode only. Live
                                            # planner measured moving the horizon's own curvature
                                            # up to 0.49 1/m/tick (40x a precomputed path) during
                                            # the lap-2 corner stall (planner_only_lap2_corner_
                                            # spinout.md); 2.0 (0.1 1/m per 50ms tick) is ~8x the
                                            # precomputed-path noise floor but ~5x tighter than
                                            # that spike. Not yet live-validated. (1/m per s)
NMPC_OSQP_MAX_ITER = 500                    # bounded well below solve_mpc's ~8000: this is a

                                            # step DIRECTION validated by the backtracking cost
                                            # check before being kept, so a hard subproblem
                                            # should cost bounded time and be retried next tick.
NMPC_OSQP_EPS = 1e-4                        # looser than solve_mpc's 1e-5 on purpose -- a step

                                            # direction the next iteration corrects doesn't need
                                            # sub-1e-4 accuracy.
NMPC_ALAT_CEILING_ENABLED = True            # model FSDS's measured sustained a_lat ceiling

                                            # (ALAT_CEILING_FLAT/_SLOPE/_INTERCEPT below) inside
                                            # the NMPC's own prediction. NOT optional on FSDS:
                                            # without it the linear-tyre prediction believes it
                                            # can hold any corner at any speed, and the car spins
                                            # mid-lap (see Part 16 §16.6, live repo). False only
                                            # for real-vehicle work where that ceiling doesn't
                                            # exist.

# [NMPC only] NMPC_SPLINE_REFERENCE_ENABLED -- True (default): PathReference builds kappa(s)/
# psi_ref(s) from an analytic CubicSpline fit to the raw waypoints (x(s), y(s)
# each independently splined over cumulative arc length), instead of the old
# dense-resample + moving-average + finite-difference pipeline. A strict
# numerical-quality improvement to the documented "centreline curvature
# spikes" defect (see CLAUDE.md) with no new coupling to solver dynamics, so
# it defaults ON unlike the two flags below -- but is still flagged so the
# old moving-average path (kept, not deleted) can be A/B'd against it if a
# regression shows up. False restores the pre-existing behaviour exactly.
NMPC_SPLINE_REFERENCE_ENABLED = True

# [NMPC only] NMPC_FRICTION_CIRCLE_ENABLED -- False (default, EXPERIMENTAL): add a HARD
# per-axle |F_yf|/|F_yr| bound to the condensed QP (on top of, not instead
# of, the existing SOFT alat-ceiling tanh saturation inside _f/_f_scalar --
# see CLAUDE.md's strong warning against touching that mechanism, which this
# does not). The bound is derived from the SAME measured ceiling law
# (ALAT_CEILING_FLAT/_SLOPE/_INTERCEPT) via F_max = m * ceiling(v_x) / 2 per
# axle (see nmpc_optimiser.py's _fmax_flat/_fmax_slope/_fmax_intercept for
# the exact conversion). When False, _build_qp/_outputs/_output_jacobians/
# _solve_step are all IDENTICAL (same array shapes, same QP dimensions) to
# before this feature existed -- not just "the extra rows are empty".
NMPC_FRICTION_CIRCLE_ENABLED = False

# [NMPC only] NMPC_PROGRESS_ENABLED -- False (default, EXPERIMENTAL): let the
# NMPC choose its own speed via an arc-length progress reward instead of
# tracking an externally supplied v_ref. See
# docs/logs/nmpc_progress_term_investigation.md for the full design and why
# this revisits a previously-deferred (not rejected-on-merit) idea from
# late_turn_in_investigation.md §16.2.
#
# When True: row 4 of the NMPC's cost switches from the two-sided speed-
# error residual (v_x - v_ref, symmetric, pulls the car UP to a target) to a
# ONE-SIDED cap (penalises v_x only ABOVE desired_speed, nothing pulls it
# up), and a 6th cost row rewards progress toward an unreachable arc-length
# target (NMPC_Q_PROGRESS below), written as a least-squares residual so it
# stays native to the Gauss-Newton solver (see nmpc_optimiser.py's
# _outputs() docstring -- a bare linear reward contributes nothing to the
# Hessian and lets a single SQP step bang to a bound). desired_speed (the
# existing curvature-limited/precomputed-profile pipeline, UNCHANGED) keeps
# acting as the cap, so the scoring baseline (time_bonus, LapProgressTracker)
# stays valid -- this is NOT full MPCC with the speed reference removed.
#
# NMPC_Q_E_V above then weights the CAP hinge, not a two-sided tracking
# error -- same slot, different regressor, needs its own value rather than
# inheriting the tracking-mode tuned q_e_v unchanged.
#
# Two landmines this design answers (both established by LIVE failures, not
# theory, see control_mechanisms.md's "Horizon speed profile" writeup):
#   1. A cost summed over the WHOLE horizon lost the safety property that
#      makes kappa(s) safe (nmpc_horizon_speed_profile_enabled, removed).
#      Here the progress reward only ever scores the TERMINAL stage, so a
#      later stage's target cannot pay for an earlier stage's violation.
#   2. A hard per-stage speed constraint reported itself satisfied (0.0
#      violation) while the real car was measurably over target
#      (nmpc_speed_limit_enabled, removed) -- the model's own prediction was
#      wrong, not the constraint mechanism. The cap here is a soft penalty
#      with gradient everywhere, not a hard inequality that can go inert.
NMPC_PROGRESS_ENABLED = False

NMPC_Q_PROGRESS = 4.25        # weight on the progress-reward row (row 5), UNTUNED

NMPC_PROGRESS_REACH = 3.0    # s_target_N = s0 + max(v_cap*N*DT*REACH,

                             # 0.5*a_max*(N*DT)^2*REACH); >1 keeps it always
                             # out of reach so minimising the residual is
                             # monotone-equivalent to maximising s_N. The
                             # second (kinematic) term floors the gap at
                             # launch, when v_cap is deliberately small
                             # (SPEED_TARGET_DEFICIT_MAX) and v_cap*N*DT*REACH
                             # alone would be reachable almost immediately --
                             # measured to stall the car indefinitely below
                             # the ~2.3 m/s^2 needed to break static friction
                             # at REACH=1.5 with no floor. 2.0 gives ~1.3x
                             # margin over the measured 11 m minimum at this
                             # weight set; see nmpc_optimiser.py's
                             # compute_step() comment for the full mechanism.
NMPC_PROGRESS_V_MIN = 3.0    # hard-ish low-speed floor (hinge, same row/weight

                             # as the cap): defence against the standstill
                             # trivial solution, compounded by the known
                             # v_x=0 tyre-force bug elsewhere in this model.
                             # Liniger's MPCC reference uses 0.05; this repo's
                             # cars sit higher off the mark than an RC car so
                             # a slightly larger floor is a reasonable start.
NMPC_SLACK_LINEAR_WEIGHT = 500.0   # additional LINEAR term on the soft track-

                             # boundary slack, on top of the existing
                             # quadratic NMPC_SLACK_WEIGHT. 0.0 (default) is
                             # a no-op. A purely quadratic penalty has ZERO
                             # gradient at zero violation, which matters once
                             # NMPC_PROGRESS_ENABLED gives the solver an
                             # unbounded incentive to find that gap (corner-
                             # cutting: s_dot rises for e_y toward the inside
                             # of a bend, a direct analytic incentive). Only
                             # meaningful with NMPC_PROGRESS_ENABLED; harmless
                             # otherwise since nothing then rewards violating
                             # the boundary in the first place.

# [NMPC only] NMPC_LATENCY_COMPENSATION_ENABLED -- False (default, EXPERIMENTAL): roll x0
# forward by NMPC_LATENCY_COMPENSATION_MS (held at the last applied control, same
# nonlinear _step_scalar rollforward the existing pose-age delay compensation
# uses) before linearising, instead of around x0 as measured at the START of
# the tick. Compensates for the SOLVE's own wall-clock time (up to
# NMPC_SOLVE_BUDGET_MS), not pose staleness (that's the separate, already-
# existing delay_compensation_enabled/pose_age_s mechanism). Held constant,
# not extrapolated: the true future command is exactly what this tick's
# solve is trying to determine, so guessing it would add error rather than
# remove it. Not yet live-validated.
NMPC_LATENCY_COMPENSATION_ENABLED = False

NMPC_LATENCY_COMPENSATION_MS = 25.0         # defaults to NMPC_SOLVE_BUDGET_MS; rounded to the

                                            # nearest whole DT step and capped by
                                            # MAX_DELAY_COMPENSATION_STEPS, same as n_delay.

# NMPC_V_DES_FILTER_ALPHA: parity placeholder only, NOT YET WIRED IN. Live's
# nmpc_core.py low-pass-filters the incoming speed target before its cost
# function sees it (nmpc_params.py's nmpc_v_des_filter_alpha, default 0.09,
# the best result of a live tuning sweep -- see that field's own docstring and
# planner_only_lap2_corner_spinout.md). controller/nmpc_optimiser.py's
# compute_step() has NO equivalent: it feeds desired_speed straight into
# _outputs()/_solve_step() unfiltered every call. This constant exists only
# so the live default has a matching offline record per CLAUDE.md's parity
# rule; it has no effect until/unless an equivalent filter is actually
# added to nmpc_optimiser.py.
NMPC_V_DES_FILTER_ALPHA = 0.09

# FSDS's fitted sustained lateral-acceleration ceiling law,
# a_lat_max(v) = max(FLAT, SLOPE * |v| + INTERCEPT) in m/s^2. This is a
# MEASURED property of the simulator (see CLAUDE.md's "dynamically-enforced
# lateral-acceleration ceiling"), not a free tuning knob. Still load-bearing
# for the NMPC's own in-prediction ceiling model (NMPC_ALAT_CEILING_ENABLED
# above) even though the LTV-QP no longer has a lookahead
# demand-normalisation reading these directly. Must stay in sync with
# model/vehicle_physics.py's alat_ceiling_at().
ALAT_CEILING_FLAT = 7.5

ALAT_CEILING_SLOPE = 0.47

ALAT_CEILING_INTERCEPT = 2.46
