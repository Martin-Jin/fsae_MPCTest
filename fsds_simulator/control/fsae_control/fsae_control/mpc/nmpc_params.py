# Title: nmpc_params.py

"""
nmpc_params.py — STRUCTURAL/SOLVER tunables for the NONLINEAR MPC
path-tracking controller (nmpc_core.NMPCController). See
late_turn_in_investigation.md Part 16 for the research/decision record
behind the formulation.

WHAT LIVES HERE VS. IN mpc_params.py
--------------------------------------------------------
NMPC weight overrides live in mpc_params.py's MPCParams; this file holds
only structural/solver fields with no LTV-QP analogue. Do not re-add
weight fields here — see mpc_params.py instead.

What remains here are fields that have NO LTV-QP analogue to inherit
from — horizon length, SQP iteration count, trust-region size, solver
tolerances, curvature-smoothing parameters. These are genuinely NMPC-only
mechanics, not "MPC weights" in the sense the rest of MPCParams's fields
are.

PARITY, NOW THAT AN OFFLINE NMPC EXISTS
-----------------------------------------
Before fsae_MPCTest's offline NMPC port (controller/nmpc_optimiser.py),
these structural fields had no offline counterpart, so CLAUDE.md's
numeric-parity rule didn't apply to them. It does now: every field below
has a matching settings.py NMPC_* constant (see that file's "Nonlinear MPC
(NMPC)" section), kept numerically identical by hand — the same
discipline as Q_diag/R_diag/R_rate_diag always have been. Extend both
sides together if either changes.

ONE WEIGHT (now in mpc_params.py) DOES NOT TRANSFER WITH IDENTICAL
MEANING: q_r / nmpc_q_epsi_dot. In the LTV-QP it weights absolute yaw rate
`r`; in the Frenet NMPC it weights the heading-error RATE
`r - kappa*s_dot` (see nmpc_core._outputs). Penalising absolute `r` in a
curvature-aware model would penalise the yaw rate the car MUST hold to
follow a corner (r = kappa*v), i.e. it would fight cornering — which is
the exact failure this controller exists to remove. Same number,
different regressor: expect to re-sweep it. See Part 16 §16.3 choice (1),
and mpc_params.py's own docstring for the field itself.

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
        "controller": "nmpc_only",
    })

    # ── Horizon / real-time budget ──────────────────────────────────────
    # MEASURED, not assumed — see Part 16 §16.7 for the horizon-choice sweep.
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
        "desc": "prediction horizon in steps (20 * dt=0.05 = 1.0 s). "
                "See Part 16 §16.7 for the horizon choice.",
        "controller": "nmpc_only",
    })
    # 1, not 2 — see §16.7. A single Gauss-Newton iteration per tick is the
    # standard real-time-iteration scheme — the warm start carries
    # convergence across ticks, and a converged-per-tick solution exploits
    # the optimistic model harder.
    nmpc_sqp_iters: int = field(default=1, metadata={
        "unit": "iterations",
        "desc": "max Gauss-Newton SQP iterations per control tick (real-time-"
                "iteration style: the previous tick's shifted solution is the "
                "warm start, so one iteration per tick still converges across "
                "ticks). See Part 16 §16.7 for the iteration-count choice.",
        "controller": "nmpc_only",
    })
    nmpc_solve_budget_ms: float = field(default=25.0, metadata={
        "unit": "ms",
        "desc": "wall-clock budget per tick; SQP stops early (shipping the best "
                "feasible iterate) once exceeded. Half of the 50 ms control "
                "period, leaving the rest of the tick for the node",
        "controller": "nmpc_only",
    })
    nmpc_rk_substeps: int = field(default=4, metadata={
        "unit": "substeps",
        "desc": "RK4 substeps per dt in the prediction rollout. Two separate "
                "stiff modes set this, and only the first was accounted for "
                "originally: tau_a=0.02 s against dt=0.05 s (lambda*dt = -2.5, "
                "near RK4's real-axis stability edge), which 2 substeps "
                "covers, AND the (v_y, r) lateral sub-dynamics, whose "
                "eigenvalues scale as 1/v_x and so get STIFFER as the car "
                "slows. At 2 substeps the rollout is outright unstable (not "
                "merely inaccurate) across roughly 2.25-3.5 m/s: an "
                "infinitesimal disturbance is amplified ~6e8 over a 20-stage "
                "horizon, so the PREDICTION itself is garbage and every "
                "line-search trial scores worse than the unstepped iterate, "
                "which freezes the controller at exactly zero steering/accel. "
                "4 decays cleanly over the whole 0.1-25 m/s envelope. "
                "CORRECTED 2026-09-15: an earlier version of this docstring "
                "claimed 3 substeps also fails at 2.50 m/s exactly -- a "
                "direct re-measurement (perturbation growth through this "
                "same _rollout, all 8 states individually perturbed, several "
                "control-sequence shapes) found 3 fully stable (<=1.6x "
                "growth) everywhere tested in and around that band; the "
                "original claim's basis is not reproduced. 3 is used above "
                "nmpc_rk_gate_speed via nmpc_rk_substeps_fast. This is the "
                "SAME stiffness mechanism as nmpc_jac_substeps below, but a "
                "distinct defect: that one corrupts the QP's step DIRECTION, "
                "this one corrupts the predicted TRAJECTORY the step is "
                "scored against, so raising jac_substeps alone does not fix "
                "it. See fsae_MPCTest/docs/logs/nmpc_low_speed_"
                "accel_stall_investigation.md",
        "controller": "nmpc_only",
    })
    nmpc_jac_substeps: int = field(default=4, metadata={
        "unit": "substeps",
        "desc": "RK4 substeps used when finite-differencing the QP's A_k/B_k "
                "sensitivities. Was 1 (coarser than nmpc_rk_substeps=2, on the "
                "assumption that a less-accurate Jacobian only costs a "
                "slightly worse SQP step direction) until the (v_y, r) "
                "sub-dynamics were found to go numerically UNSTABLE (not just "
                "less accurate) below ~6.5-7 m/s with only 1 substep -- RK4's "
                "stability limit is exceeded, the divergence compounds across "
                "the horizon's condensing loop, and the Hessian becomes so "
                "ill-conditioned that OSQP's only representable solution is "
                "exactly zero steering/accel, frozen once warm-started into "
                "that state. 4 is the validated fix: see "
                "fsae_MPCTest/docs/logs/nmpc_low_speed_accel_stall_"
                "investigation.md for the root-cause derivation.",
        "controller": "nmpc_only",
    })
    nmpc_jac_gate_speed: float = field(default=8.0, metadata={
        "unit": "m/s",
        "desc": "below this speed (checked against the SLOWEST predicted stage "
                "in the horizon, not the instantaneous speed, so the gate "
                "changes rarely), _jacobians uses the full nmpc_jac_substeps. "
                "At or above it, nmpc_jac_substeps_fast is used instead. The "
                "jac_substeps=4 fix's own instability envelope (measured "
                "max|A_k| vs the converged value) shows the divergence is "
                "confined to low speed: 2.41e2 at 2.5 m/s but only 4.06 at "
                "8 m/s and 4.01 at 14 m/s (js=1 vs converged js=4), so the "
                "cost of the fix does not have to apply everywhere it was "
                "needed nowhere. An analytic Jacobian would not lower this "
                "floor either -- the variational equation shares RK4's "
                "stability region with the nominal ODE (verified: both "
                "diverge at the same lambda*dt), so the floor is inherent to "
                "RK4 sensitivity propagation at low speed, not to finite "
                "differencing. Set equal to a speed below the operating range "
                "to disable the gate (nmpc_jac_substeps_fast is then never "
                "read).",
        "controller": "nmpc_only",
    })
    nmpc_jac_substeps_fast: int = field(default=2, metadata={
        "unit": "substeps",
        "desc": "nmpc_jac_substeps used at/above nmpc_jac_gate_speed. 2 tracks "
                "the converged (4-substep) sensitivity closely with no "
                "divergence in that speed range (measured max|A_k| 3.07 vs "
                "3.18 at 8 m/s, 4.90 vs 4.93 at 14 m/s). 1 is NOT safe here "
                "despite not diverging: it is inaccurate rather than unstable "
                "at speed (1.30 vs a converged 3.85 at 10 m/s), which is a "
                "worse SQP step direction, not a fix. Set equal to "
                "nmpc_jac_substeps to disable the gate exactly.",
        "controller": "nmpc_only",
    })
    nmpc_rk_gate_speed: float = field(default=4.0, metadata={
        "unit": "m/s",
        "desc": "same technique as nmpc_jac_gate_speed, applied to the "
                "ROLLOUT itself (checked per predicted STAGE, not the whole "
                "horizon at once, since _rollout builds X incrementally and "
                "a stage's speed can cross the gate mid-horizon on a hard "
                "launch/brake). The rollout's instability is confined to a "
                "NARROWER band than the Jacobian's (measured stable, <=1.6x "
                "perturbation growth, at and above ~3.75 m/s vs the "
                "Jacobian's ~8 m/s), so this gate opens earlier.",
        "controller": "nmpc_only",
    })
    nmpc_rk_substeps_fast: int = field(default=3, metadata={
        "unit": "substeps",
        "desc": "nmpc_rk_substeps used at/above nmpc_rk_gate_speed. NOT 2: "
                "2 is the one substep count confirmed unstable (up to ~260x "
                "perturbation growth) in the 2.25-3.75 m/s band. 3 is fully "
                "converged (<=1.6x growth) everywhere measured at and above "
                "the gate speed, across all 8 states individually perturbed "
                "and several control-sequence shapes. Set equal to "
                "nmpc_rk_substeps to disable the gate exactly.",
        "controller": "nmpc_only",
    })

    # ── Standstill steering damping ─────────────────────────────────────
    # At v_x = 0 steering has NO physical effect (the kinematic branch gives
    # r_kin = v_x*tan(d)/L = 0, and the dynamic branch is blended out), but
    # the SQP's cost is summed over the WHOLE horizon, and the rolled-out
    # v_x profile leaves zero by stage 1 ([0, 0.118, 0.294, 0.465, ...]).
    # Nothing in the cost distinguishes "this stage cannot act yet" from
    # "this stage will act soon", so the optimiser pre-commits U[0] toward
    # whatever helps the later, physically-active stages. Measured live and
    # reproduced offline: steering ramps to ~-6.8 deg over the ~1 s before
    # the car physically moves, so it launches already turned and curves
    # sideways until the correction catches up.
    nmpc_standstill_steer_damp_enabled: bool = field(default=True, metadata={
        "desc": "damp the SQP's stage-0 steering effort while the car is "
                "measurably stationary, so it does not pre-commit a steering "
                "angle it cannot act on and then launch already turned. Only "
                "stage 0 is damped: the predicted v_x leaves zero by stage 1, "
                "so every later stage can genuinely act and its planning is "
                "left untouched, and U[0] is the only element actually "
                "shipped to the car.",
        "controller": "nmpc_only",
    })
    nmpc_standstill_speed: float = field(default=0.5, metadata={
        "unit": "m/s",
        "desc": "below this MEASURED speed (x0's own v_x, not a predicted "
                "stage), stage 0's steering effort weight is scaled by "
                "nmpc_standstill_steer_r_scale. Keyed on the measurement so "
                "the damping disengages the moment the car actually moves.",
        "controller": "nmpc_only",
    })
    nmpc_standstill_fade_speed: float = field(default=3.0, metadata={
        "unit": "m/s",
        "desc": "speed at which the standstill damping has faded fully back "
                "to 1x. The multiplier is held at its full value below "
                "nmpc_standstill_speed and ramped linearly to 1.0 here, so "
                "the weight never changes in one step. A hard release put "
                "the whole change into a single tick right where the car is "
                "most sensitive: measured live, steering ran -1.8 to -12.9 "
                "deg over the six ticks straight after the release. Set at "
                "or below nmpc_standstill_speed to restore a hard cutoff.",
        "controller": "nmpc_only",
    })
    nmpc_standstill_steer_r_scale: float = field(default=200.0, metadata={
        "desc": "multiplier on r_delta for stage 0 only, tapering to 1x by "
                "nmpc_standstill_fade_speed. A weight sweep on the "
                "standstill reproduction gives a stage-0 pre-load of -6.71 "
                "deg at 1x, -2.41 at 20x, -1.18 at 50x, -0.63 at 100x and "
                "-0.33 at 200x, halving roughly per doubling with no effect "
                "on the accel command at any scale. 200x pins the pre-load "
                "to a few tenths of a degree while staying short of the very "
                "stiff end, where stage 0 becomes a de-facto hard constraint. "
                "Live-validated 2026-09-15 on comp_test_map_3: peak steering "
                "through the 0.5-3.0 m/s band 10.68 -> 6.79 deg, mean "
                "per-tick steering change 1.77 -> 1.00 deg/tick, launch max "
                "|e_y| 0.490 -> 0.387 m, whole-run reversals 5.78 -> 3.82 "
                "percent. It does NOT fix the launch yaw excursion (11.42 -> "
                "11.12 deg, unchanged within noise) -- that has a separate "
                "cause in the speed-target ramp. A tuning value, not a "
                "derived constant.",
        "controller": "nmpc_only",
    })

    # ── SQP step control ────────────────────────────────────────────────
    nmpc_trust_delta_rad: float = field(default=math.radians(9.0), metadata={
        "unit": "rad",
        "desc": "per-iteration trust region on each stage's steering deviation. "
                "9 deg = MPCController's own du_max steering slew per tick "
                "(180 deg/s * 0.05 s) — reused, not invented",
        "controller": "nmpc_only",
    })
    nmpc_trust_a: float = field(default=0.6, metadata={
        "unit": "m/s^2",
        "desc": "per-iteration trust region on each stage's accel deviation. "
                "0.6 = MPCController's du_max[1] — reused, not invented",
        "controller": "nmpc_only",
    })
    nmpc_backtrack_max: int = field(default=2, metadata={
        "unit": "halvings",
        "desc": "max step halvings if a full SQP step increases the true "
                "nonlinear cost (divergence guard). 0 disables backtracking",
        "controller": "nmpc_only",
    })

    # ── Soft track constraint (mirrors MPCController's own) ─────────────
    nmpc_track_halfwidth: float = field(default=3.5, metadata={
        "unit": "m",
        "desc": "soft |e_y| bound with slack, copied from _build_qp's existing "
                "+-3.5 m literal. <=0 removes the constraint (and its slack "
                "variables) entirely",
        "controller": "nmpc_only",
    })
    nmpc_slack_weight: float = field(default=10000.0, metadata={
        "unit": "1/m^2",
        "desc": "penalty on the track-bound slack, copied from _build_qp's "
                "existing W_slack = 10000.0",
        "controller": "nmpc_only",
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
        "controller": "nmpc_only",
    })
    nmpc_curvature_smooth_w: int = field(default=3, metadata={
        "unit": "samples",
        "desc": "moving-average width applied before differencing headings "
                "(= curvature_speed()'s w). 1 disables smoothing",
        "controller": "nmpc_only",
    })
    nmpc_kappa_clip: float = field(default=0.5, metadata={
        "unit": "1/m",
        "desc": "hard clamp on |kappa(s)|. GUARD, not a tuning knob: 0.5 = a "
                "2 m radius, the tightest corner curvature_speed()'s own "
                "docstring contemplates, so it is inert on any real track "
                "line and only catches a degenerate/spiking path",
        "controller": "nmpc_only",
    })

    # nmpc_kappa_rate_max: same idea as MPCParams' V_CURV_FALL_RATE (the
    # speed-side rate limiter), applied to the kappa(s) PROFILE instead of a
    # scalar speed target. planner_only_lap2_corner_spinout.md measured the
    # live planner's per-tick-rebuilt path making the NMPC's horizon-end
    # curvature move up to 0.49 1/m per TICK (40x a precomputed path's
    # 0.012 1/m/tick) during a hard-braking corner entry -- exactly the
    # window where a rejected SQP step holds a stale plan for 50 ms, and a
    # stale plan against a fast-moving reference is a much bigger error than
    # against a slow-moving one (see that doc's mechanism section). Unlike
    # V_CURV_FALL_RATE, this limits BOTH directions: curvature can swing
    # either sign as the corner's own shape gets re-resolved by the planner,
    # there's no "only delay bad news" asymmetry the way braking has.
    nmpc_kappa_rate_max: float = field(default=2.0, metadata={
        "unit": "1/m per s",
        "desc": "max tick-to-tick CHANGE of kappa(s) at matching arc-length "
                "samples, live-planner mode only (inert when a static/"
                "precomputed path is set via set_static_path). "
                "LIVE-VALIDATED at 2.0 2026-09-15 (planner_only_lap2_corner_"
                "spinout.md): survives the corner that used to stall "
                "permanently, rejected-tick rate roughly halves (1.7% -> "
                "1.0%), but the car still visibly stumbles through it "
                "(speed collapse + e_psi swinging to -103 deg before "
                "recovering). TIGHTENING TRIED AND REVERTED: 1.0 FAILED at "
                "the same corner (stalled to a stop this time) and its own "
                "telemetry showed MORE kappa_horizon_end sign-flipping "
                "through the corner than 2.0's run, not less -- consistent "
                "with 1.0 being tight enough to lag the corner's genuinely "
                "fast-resolving curvature estimate and then overshoot when "
                "it catches up, i.e. this specific corner needs more than "
                "1.0 1/m/s of legitimate correction rate. Do not re-try "
                "values below 2.0 without new evidence.",
        "controller": "nmpc_only",
    })

    nmpc_alat_ceiling_enabled: bool = field(default=True, metadata={
        "unit": "bool",
        "desc": "include FSDS's measured sustained lateral-acceleration ceiling "
                "(MPCParams.alat_ceiling_flat/_slope/_intercept, the same law "
                "mpc_core._alat_ceiling_at uses) as a smooth saturation of the "
                "prediction's tyre forces. True is correct for FSDS; set False "
                "for real-vehicle work, mirroring "
                "model/vehicle_physics.VehicleParams.alat_ceiling_enabled",
        "controller": "nmpc_only",
    })

    # ── Path reference construction ─────────────────────────────────────
    nmpc_spline_reference_enabled: bool = field(default=True, metadata={
        "unit": "bool",
        "desc": "true (default) -> PathReference builds kappa(s)/psi_ref(s) "
                "from an analytic CubicSpline fit to the raw waypoints "
                "(x(s), y(s) each independently splined over cumulative arc "
                "length) instead of the dense-resample + moving-average + "
                "finite-difference pipeline. A strict numerical-quality fix "
                "to the documented centreline-curvature-spikes defect with no "
                "new solver coupling, so it defaults on -- unlike every other "
                "flag in this file. False restores the old moving-average "
                "path exactly (kept, not deleted), for A/B comparison",
        "controller": "nmpc_only",
    })

    # ── Friction-circle hard constraint (EXPERIMENTAL, default off) ──────
    nmpc_friction_circle_enabled: bool = field(default=False, metadata={
        "unit": "bool",
        "desc": "true -> add a HARD |F_yf|/|F_yr| <= F_max bound to the "
                "condensed QP, ADDITIONAL to (not a replacement for) the "
                "existing SOFT alat-ceiling tanh saturation inside _f/"
                "_f_scalar -- see CLAUDE.md's warning against touching that "
                "mechanism, which this does not. F_max is derived from the "
                "SAME measured ceiling law "
                "(alat_ceiling_flat/_slope/_intercept) via "
                "F_max = m * ceiling(v_x) / 2 per axle. Changes the QP's "
                "fixed sparsity pattern, so it is read once at construction "
                "time, not per-tick. When False, _build_qp/_outputs/"
                "_output_jacobians/_solve_step produce IDENTICAL output "
                "(including array shapes) to before this feature existed. "
                "Default False: genuine experiment, not yet validated",
        "controller": "nmpc_only",
    })

    # ── Solve-time latency compensation (EXPERIMENTAL, default off) ─────
    # The existing delay_compensation_enabled/pose_age_s mechanism
    # (mpc_params.py) rolls x0 forward through the ACTUAL applied control
    # history to correct for how stale the MEASURED POSE already was when
    # the solve started. It does not compensate for the solve's own
    # wall-clock time: the linearisation and reference are still built
    # around x0 as of the START of the tick, but u[0] is not applied until
    # the solve finishes, up to nmpc_solve_budget_ms later. At corners where
    # the car is decelerating hard and the reference (curvature ahead,
    # target speed) is changing quickly tick to tick, that gap between
    # "where x0 was linearised" and "where the car is when u[0] actually
    # lands" is exactly the kind of staleness the backtracking line search
    # (nmpc_backtrack_max) rejects steps over -- see
    # planner_only_lap2_corner_spinout.md. This rolls x0 forward the SAME
    # way as delay compensation (nonlinear _step_scalar, actual applied
    # control), just forward past "now" using the last shipped command held
    # constant, instead of backward using the recorded command history.
    # Held constant, not extrapolated, because the true future command is
    # exactly what this tick's solve is trying to determine -- guessing it
    # would inject a new error source rather than remove one. Unlike
    # pose-age compensation (which corrects a MEASURED quantity), this rolls
    # forward by a budget estimate, not a measured latency.
    #
    # LIVE-TESTED 2026-09-15 at the lap-2 corner margin
    # (planner_only_lap2_corner_spinout.md): does not close it. Two valid
    # runs (n_latency=1 confirmed engaged both times, after fixing a
    # rounding bug that silently no-op'd the first attempt) show a modest
    # rejected-solve-rate improvement (7/1507 and 26/1507 vs a 24/1406
    # baseline) but the same stall mechanism recurs -- a dense rejection
    # cluster during hard braking at the same corner, car settles into a
    # converged solved state at e_psi~-100 deg, v~0. Consistent with the
    # root cause being live-planner REFERENCE volatility during braking
    # (measured 5-40x noisier than a precomputed path), which this feature
    # doesn't address -- it only compensates the fixed SOLVE-TIME gap.
    # Initially looked harmless alone (rejected-rate improvement, no
    # downside) and was tried combined with nmpc_kappa_rate_max=2.0. That
    # combined run went on to show a real, separate regression (jerky/
    # twitchy gentle-corner behaviour, never seen before that session,
    # worse jerk_rms and control_smooth_rms than the day's earlier
    # baseline). Unlike nmpc_kappa_rate_max (structurally inert on a
    # precomputed path, later confirmed NOT the cause), this feature runs
    # EVERY tick regardless of path source, so it remains a live,
    # un-isolated suspect for that regression alongside a too-fast
    # nmpc_v_des_filter_alpha (which WAS directly evidenced as harmful at
    # large steps). Defaulted back to False pending an isolated test that
    # actually singles this one out. Do not re-enable for a live run
    # without doing that isolation first.
    nmpc_latency_compensation_enabled: bool = field(default=False, metadata={
        "unit": "bool",
        "desc": "true -> roll x0 forward by nmpc_latency_compensation_ms "
                "(held at the last applied control, via the same nonlinear "
                "_step_scalar rollforward pose-age delay compensation uses) "
                "before linearising, to compensate for the solve's own "
                "wall-clock time rather than pose staleness. EXPERIMENTAL, "
                "not yet live-validated. Default false.",
        "controller": "nmpc_only",
    })
    nmpc_latency_compensation_ms: float = field(default=25.0, metadata={
        "unit": "ms",
        "desc": "assumed solve latency to roll x0 forward by when "
                "nmpc_latency_compensation_enabled is true. Defaults to "
                "nmpc_solve_budget_ms (a budget, not a per-tick "
                "measurement: the true solve time isn't known until AFTER "
                "the solve this would need to run before). Rounded to the "
                "nearest whole dt step, same as pose-age compensation's "
                "n_delay, and capped by max_delay_compensation_steps.",
        "controller": "nmpc_only",
    })

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
        "controller": "nmpc_only",
    })
    nmpc_osqp_eps: float = field(default=1e-4, metadata={
        "unit": "unitless",
        "desc": "OSQP eps_abs/eps_rel for the SQP subproblem. Looser than "
                "MPCController's 1e-5 on purpose: an SQP subproblem is a STEP "
                "direction that the next iteration corrects, not the final "
                "answer, so sub-1e-4 accuracy buys nothing and costs iterations",
        "controller": "nmpc_only",
    })

    # ── Speed-target smoothing ────────────────────────────────────────────
    # First-order low-pass on the incoming desired_speed before it reaches
    # the cost function (v_ref = v_ref + alpha*(desired_speed - v_ref) each
    # tick). Introduced 2026-06-29 (mirrors MPCController's own identical
    # filter in mpc_core.py) to smooth the live planner's ~1 Hz cone-map
    # target-speed jumps, so a single noisy planner update doesn't look like
    # a step-input speed error to the solver. Has NO offline analogue
    # (rollout_core.py/nmpc_optimiser.py never filter the speed target), so
    # this has always been live-only and untested against the offline
    # weight sweeps.
    #
    # LIVE-TUNING HISTORY 2026-09-15 (planner_only_lap2_corner_spinout.md):
    # started at the original default 0.08 (dt/alpha ~= 0.6 s time
    # constant). At one hard-braking hairpin (11 m -> 4.65 m radius,
    # collapsing target ~4.7 m/s in ~1 s) this lagged the raw target by up
    # to 1.7 m/s for over a second, braking stayed weak, and the car
    # carried too much speed into the tightest part of the corner (|e_y|
    # briefly ~2.6-2.8 m). Raising alpha to fix that made OVERALL
    # performance worse at every LARGE step tried: 0.25 (dt/alpha ~= 0.2 s)
    # caused oscillating accel/brake and steering hunting at several OTHER
    # corners plus one genuine off-track excursion; 0.13 (dt/alpha ~=
    # 0.38 s) still showed 4 excursion clusters, one WORSE than the 0.08
    # baseline's single corner; disabling the filter entirely (v_ref =
    # desired_speed, no smoothing) was worse still, 7 clusters, the worst
    # result of any config tested; 0.05 (slower than 0.08) also
    # underperformed the original in a controlled A/B (also confounded
    # with nmpc_kappa_rate_max/latency-compensation being active in that
    # run, since disentangled). Once nmpc_kappa_rate_max and latency
    # compensation were correctly isolated (both confirmed structurally
    # inert on this precomputed-path test, and disabled respectively), a
    # SMALL step above the original -- 0.09 -- gave the best full-run
    # result of the day: composite_score 0.410 (vs this session's best-
    # ever 0.369 at the unmodified 0.08 baseline that morning, and 0.476
    # for the clean all-off baseline that evening), ZERO rejected solves,
    # max|e_y| 0.84 m, jerk_rms/control_smooth_rms both in the healthy
    # range. Landed here: a small tightening of the smoothing, not the
    # large speedups that repeatedly proved counterproductive.
    nmpc_v_des_filter_alpha: float = field(default=0.09, metadata={
        "unit": "unitless",
        "desc": "first-order low-pass coefficient on the speed target "
                "before it reaches the NMPC's cost function. Smaller = "
                "slower/heavier smoothing (more lag, more noise "
                "rejection); larger = faster tracking (less lag, more "
                "sensitive to planner/perception jitter). See this "
                "field's own comment for the 2026-09-15 live-tuning "
                "history -- raising it has repeatedly made overall "
                "performance worse, not better.",
        "controller": "nmpc_only",
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
