# Retired Mechanisms

Mechanisms that were built and then removed, superseded or disabled. Each entry states what it was, why it went away (with numbers), and where the full record is.

This doc exists so a future session does not re-invent something already tried and measured. Mechanisms that are live today are in [control_mechanisms.md](control_mechanisms.md).

## Summary

| Mechanism | Status | Why it went away | Record |
|---|---|---|---|
| Lookahead gain-scheduling family (about 15 functions) | Removed | Reweights cost the prediction never sees. Offline replacement result: 4.8% to 0% saturation | [section](#the-lookahead-gain-scheduling-family-removed-structurally-unable-to-anticipate) |
| Exit-heading boost | Removed with the family | Decay clock keyed on the wrong curvature, then window too short | [section](#exit-heading-boost-removed-decay-timing-and-window) |
| Curvature-forcing term | Removed | Solver defers externally supplied data; wrong-direction steer at every useful gain | [section](#curvature-forcing-term-removed-the-solver-defers-external-data) |
| Low-speed steering-rate boost | Removed | Speed alone cannot tell turn-in from post-exit wobble | [section](#low-speed-steering-rate-boost-removed) |
| Adaptive `R_rate` curvature floor | Removed | Output always overwritten before reaching the QP | [section](#adaptive-r_rate-curvature-floor-removed-computed-then-always-overwritten) |
| Post-solve output smoothing | Removed | Added lag, never beat the rate cost | [section](#post-solve-output-smoothing-removed) |
| Single shared `r_a` weight | Superseded | One weight could not serve both accel and brake | [section](#single-shared-accel-effort-weight-superseded) |
| NMPC horizon speed profile (two variants) | Removed | Cost variant overspeeds corners, constraint variant never engages | [section](#nmpc-horizon-speed-profile-two-variants-removed) |
| NMPC friction-circle constraint | Disabled, code present | Hard rows without slack go infeasible in normal cornering | [section](#nmpc-friction-circle-constraint-disabled) |
| NMPC progress term | Disabled, code present | Does not reliably complete a lap at any weight | [section](#nmpc-progress-term-disabled) |
| Dynamic speed cap | Disabled by the shipped launch, code default on | Improved its target metric, worsened the ones that matter | [section](#dynamic-speed-cap-disabled-in-the-shipped-launch) |

None of the removed fields, functions or flags below exist on `MPCParams`, the live controller modules or `controller/model_utils.py`. Names appear in code comments only. Constants quoted for removed code come from git history (`bb8863d^`) and cannot be re-checked against a running tree.

## The lookahead gain-scheduling family (removed, structurally unable to anticipate)

**What it was.** About 15 interacting LTV-QP functions scanned forward along the path each tick for the sharpest upcoming curvature (`kappa_max_abs`) and reweighted `Q`, `R` and `R_rate` from it.

| Mechanism | What it scanned for | What it reweighted |
|---|---|---|
| Lookahead corner anticipation | Peak curvature in a speed-scaled window ahead | `Q[0,0]`, `Q[2,2]` up and `Q[3,3]` down approaching and exiting a corner |
| Demand normalisation | Same, scaled by how much grip the corner needs at the current speed | Made the boosts scale-free across corners and speeds |
| U-turn detector | Accumulated heading change in the window | Extra `Q[0,0]`, `Q[2,2]`, `Q[3,3]` boost for long gradual turns |
| Straight-line adjustments | Window clear of curvature | `Q[0,0]` down, `Q[2,2]`, `Q[3,3]`, `R[0,0]` up |
| Corner segmentation (`CornerMap`) | Same scan, precomputed once per static path | Replaced the live scan with an index lookup |
| Steer-effort relax | Corner demand | `R[0,0]` toward a floor on approach |
| Low-speed rate boost, exit boost | See their own sections | See their own sections |

**Why it was retired.** In plain terms: the controller looked ahead, saw the bend, made steering cheaper, and still did not turn until the car was already off the line, because the solver's own prediction did not contain the bend.

- The scan only ever reached the solver as a reweighting of cost (`Q[0,0]`, `Q[2,2]`, `R[0,0]`, ...). It changes how expensive an existing tracking error is. It never changes what the solver's own rollout predicts.
- The LTV-QP's model (`Ad`/`Bd`) has no path-curvature term. With the car on the line (`e_y` and `e_psi` near 0), the rollout predicts near-zero error for the whole horizon, bend or no bend, so there is nothing for a cheaper weight to act on.
- Measured directly in live telemetry: `kappa_max_abs` and the boosts moved correctly and over a full second early (`m_R_steer_relax` falling to about 0.55, `Q_ey_eff` climbing from 2.5 to 4.5 and above) while `steer_deg` stayed near 0 for the whole approach. Turn-in began only when current-position curvature started producing real error.
- Piecemeal tuning never produced a net win. Three separate "raise the boost" changes (curvature-forcing gain, `adaptive_q_lookahead_q_boost_max` 2.0 to 3.0, the `e_psi` approach boost 1.5 to 2.5) each measured worse live and were reverted. A widened extended-lookahead window (`adaptive_q_extended_lookahead_dist_max` 60.0) performed badly live and was reverted (root cause not established). Several members (`adaptive_q_scaling`, anti-hunt, straight `R[0,0]` boost) carried no before/after evidence at all.
- Replacement (commit `bb8863d`, offline `python -m tuner.validation.recorded_map_rollout` on `comp_test_map_3`): steering saturation 4.8% to 0% and `|e_psi|` mean/p90 from 6.9/18.5 to 2.45/5.66 degrees, no DNF. These are offline figures from the commit message, not live results.

**What replaced it.** The current-curvature [corner-factor scheduler](control_mechanisms.md#corner-factor-scheduler-ltv-qp-weights-follow-the-current-curvature), and, for actual anticipation, the [NMPC](control_mechanisms.md#nonlinear-mpc-use_nmpc-the-second-controller), whose model contains `e_psi_dot = r - kappa(s) * s_dot`.

**Where the record is.** [late_turn_in_investigation.md](../logs/late_turn_in_investigation.md), Parts 1 to 6 (mechanisms and the `CornerMap` design), Part 2 (the failed boost raises), Part 3f (evidence audit), Part 15b (extended lookahead); [error_states.md](error_states.md) for the model-side argument.

### What each member did, and the specific lessons

- **Lookahead approach and exit boosts.** Window `clip(v_x * 1.13 s, 3 m, 17 m)` (later raised to a 25 m ceiling). Approaching: `Q[0,0]` up to 2.0x, `Q[2,2]` up to 1.5x, `Q[3,3]` down to 0.5x. Exit: `Q[2,2]` boosted for a decaying 5 m window. It used a rising-edge-after-a-clear-peak detector, because a running maximum would not re-trigger on a second corner of equal or lesser curvature.
- **Demand normalisation.** Boosts were driven by `demand = kappa_max_abs / kappa_limit(v)` with `kappa_limit = a_lat_ceiling(v) / v²`, not raw curvature. Raw curvature left the whole driven range on the flat part of the curve: a 40 m radius sweeper (`kappa` 0.025) reached 17% of the boost and a 12 m corner (0.083) reached 40%. The ceiling law itself lives on (see the last section).
- **U-turn detector.** Extra boost past 60 degrees of accumulated heading change in the window, full at 120. 60 rather than 90 because 17 m of arc at a 12 m radius subtends only about 81 degrees. It only helped before the corner. On its motivating log the steering was already at full lock for over a second mid-corner, limited by the lateral-acceleration ceiling, and no `Q` boost adds steering that is saturated. On a two-lap log it fired on 14.6% of ticks and never exceeded severity 0.29 of 1.
- **Straight-line adjustments.** `Q[0,0]` floor 0.7x, `Q[2,2]` and `Q[3,3]` ceilings 1.1x and 1.5x, `R[0,0]` up to 1.5x, all fading out as a corner entered the window. The straight `Q[2,2]` ceiling stayed small on purpose: a strong heading weight on a straight amplifies the reaction to heading noise.
- **`CornerMap` (`use_precomputed_corner_map`).** Per-waypoint precomputed segmentation, added and removed within a day. Offline-validated (bit-identical when off), never live-tested.
- **Steer-effort relax.** Relaxed `R[0,0]` toward a floor on approach. Added because neither the speed-based scaling nor the straight boost ever pushed `R[0,0]` below baseline before a corner.

## Exit-heading boost (removed, decay timing and window)

**What it was.** `Q[2,2]` boosted for a decaying distance after a corner's peak curvature, to help the car straighten on exit.

**Why it was retired.** Removed with the family. Before that it needed two fixes:

- The decay clock reset on the lookahead-window peak, which appears 10 to 17 m before the car reaches the corner. In a live log the clock read more than 28 m by the physical apex, against a 5 m window, so the boost was already a no-op before the car reached the exit. Keying it on current-position curvature improved every offline metric slightly (score 0.520 to 0.517, `|e_psi|` mean 8.01 to 7.60 degrees), and was reported as performing well live, qualitatively.
- The 5 m window was still too short: in a later log the car was 11.7 to 20.6 m (mean about 15.7 m) past the peak when `|e_y|` peaked, with no boost applied at any of 10 excursions, because errors keep growing 1.5 to 2.7 s after the apex.

**Record.** [late_turn_in_investigation.md](../logs/late_turn_in_investigation.md), "Addendum: exit-heading boost was firing at the wrong time".

## Curvature-forcing term (removed, the solver defers external data)

**What it was.** A term added to the QP dynamics constraint to inject the path's curvature at each horizon step: `predicted_e_psi[k+1] += -v_x * kappa(s_k) * dt * gain`, with `gain` 1.0 being the physically exact value (`path_yaw_rate = v_x * kappa`).

**Why it was retired.** The forcing term is known to the solver before it chooses any input, so the solver is free to decide when across the horizon to respond to it. At every gain that produced a meaningful response, the cheapest total-cost trajectory steered away from the corner first.

Isolated QP test (no noise, no other adaptive mechanism, 13 m radius corner at 17 m/s starting about 15 m ahead):

| `gain` | Accumulated forcing | Commanded steer |
|---|---|---|
| 1.0 | -1.19 | -0.27 degrees (noise scale) |
| 3.0 | -3.56 | -0.96 degrees |
| 6.0 | -7.13 | -25.0 degrees, away from the corner |
| 9.0 to 15.0 | -10.7 to -17.8 | -25.0 degrees, away from the corner |
| 20.0 | -23.8 | +25.0 degrees, correct direction but saturated at 20x the physical value |

At `gain = 1.0` the QP's own `e_psi` decay (`Ad[2,2]` about 0.946 per step, as recorded in the log) bled the forcing off almost as fast as it accumulated. Live, the disabled term left the car turning in late but without wrong-direction flicks.

**Lesson.** Getting curvature into the dynamics was the right instinct. Injecting it as external data the solver can defer does not work. The NMPC makes `kappa` a function of a state the solver chooses (arc length `s`), which removes the separate slot. A shaped heading reference was tried as a redesign ([control_mechanisms.md](control_mechanisms.md#precomputed-shaped-heading-lead-profile-ltv-qp-only-off)), with the same trap warned against for per-step extensions. The NMPC comparison is worked through in [error_states.md](error_states.md).

**Record.** [late_turn_in_investigation.md](../logs/late_turn_in_investigation.md), Part 6b.

## Low-speed steering-rate boost (removed)

**What it was.** `R_rate[0,0] *= 1 + (boost_max - 1) / (1 + k * v_x)` with `boost_max = 2.5` and `k = 0.35` (about 1.73x at 3 m/s, about 1.0x at race speed). It was inverted from a Stanley-style `k / (v + eps)` shape, making fast steering changes costlier at low speed.

**Why it was retired.** It targeted a post-exit low-speed wobble (steering swinging +25 to -9 to 0 degrees over about 1.5 s at 3 to 4 m/s), which neither anti-hunt nor the corner blend touched. Live, it also suppressed turn-in, which is low-speed and needs a fast steering-rate change too, so the car struggled to turn early. Speed alone cannot separate the two.

**Note for a rework.** It needs a curvature gate so it fires only when the car is not approaching or inside a corner. The corner-factor scheduler's `_low_speed_corner_boost` is gated on `corner_factor` for that reason.

**Record.** [late_turn_in_investigation.md](../logs/late_turn_in_investigation.md), "Appendix: Low-speed steering-rate boost: full incident".

## Adaptive `R_rate` curvature floor (removed, computed then always overwritten)

**What it was.** `_adaptive_R_rate` (live) and `adaptive_R_rate` (offline) relaxed `R_rate[0,0]` in sharp corners using `scale = max(during_floor, 1 / (1 + 3 |kappa|))`, with `during_floor` 0.625, gated by `adaptive_r_rate_enable_in_corners`.

**Why it was retired.** It never reached the solver. `compute()` and `solve_ltv_tick()` computed it, stored it in `R_rate_scaled`, and logged `adapt["m_Rrate_corner"]`. A few lines later the corner-factor blend assigned `R_rate_scaled[0,0]` from scratch and did not multiply the adaptive term back in, so the logged value looked live while having no effect. Only tracing the assignment order shows it. Commit `8b666eb` removed the function, both fields, the `settings/` constants and the telemetry column on both sides.

**Lesson.** A new `R_rate[0,0]` multiplier must be threaded through the blend line, or composed into the blend's output the way anti-hunt and the reversal penalty are. A logged multiplier is not evidence that it took effect.

## Post-solve output smoothing (removed)

**What it was.** A low-pass filter on the solved steering command: `filtered += alpha * (raw - filtered)`, then `steering = (1 - w) * raw + w * filtered`. Unlike the cost reweights, it kept state from tick to tick.

**Why it was retired.** It added lag and never beat the QP's own steering-rate cost, which attacks jitter at the source. The levers that worked were `r_rate_delta` and the NMPC input-jerk cost. The shipped default was always off, and the node parameters, `peak_kappa_ahead()`, the offline mirror and the launch wiring were deleted from both sides (commit `10c4406`). The chatter investigation lists it as made redundant by `r_rate_delta` and `nmpc_rjerk_delta`.

**Record.** [steering_chatter_investigation.md](../logs/steering_chatter_investigation.md), "Resolution summary". No isolated before/after numbers for the filter are recorded.

## Single shared accel effort weight (superseded)

**What it was.** One scalar `R_diag[1]` (`MPCParams.r_a`) weighting `|a_cmd|` for both accelerating and braking.

**Why it was superseded.** Cutting `r_a` from 0.85 to 0.77 freed acceleration on clean straights (live lap time 69.99 s to 59.52 s), and cut braking authority by the same amount. A live corner-entry log then showed the car arriving hot, with `a_cmd` floored near -1.4 m/s² through a 2 s speed deficit. Separate `r_a_accel` and `r_a_brake` weights replaced it ([control_mechanisms.md](control_mechanisms.md#accelbrake-effort-weight-split)). Do not reintroduce one shared weight without accounting for this.

**Record.** [sim_to_real_investigation.md](../logs/sim_to_real_investigation.md), section 59; [late_turn_in_investigation.md](../logs/late_turn_in_investigation.md), "Part 0 (background)".

## NMPC horizon speed profile (two variants, removed)

**What it was.** Sampling the precomputed speed profile at each horizon stage's predicted arc length (`PathReference.v_ref_at(s)`) instead of holding one `v_ref` across the horizon.

**Why it was retired.** Both variants failed live.

| Variant | Live result |
|---|---|
| Cost term (`nmpc_horizon_speed_profile_enabled`) | `v_actual` climbed from about 5.7 to 16.7 m/s while `v_desired` dropped to 3.3 to 5 m/s, with `a_cmd` positive throughout and `e_y` reaching -3.6 m. A high `v_ref` at a later stage offset a low one at an earlier stage in the summed cost. |
| Hard per-stage constraint (`nmpc_speed_limit_enabled`), two runs | Off-track both times: `|e_y|` max 4.40 and 2.19 m against a 3.5 m half-width, `v_actual` max 16.5 and 16.3 m/s. The constraint diagnostic read exactly 0.0 throughout, because the solver's own predicted braking already satisfied it while the real car did not brake that way. |

The constraint fixed the cost variant's loophole and still failed, because it constrains a prediction that is wrong in this regime. Both flags and their plumbing (`v_ref_at`, per-stage slack rows) were removed. A third variant should address the model-prediction gap first ([simulator_fidelity.md](simulator_fidelity.md)), not retune margins.

**Record.** [late_turn_in_investigation.md](../logs/late_turn_in_investigation.md), Part 16.10; [nmpc_speed_limit_investigation.md](../logs/nmpc_speed_limit_investigation.md).

## NMPC friction-circle constraint (disabled)

**What it is.** `nmpc_friction_circle_enabled` (default false, code present). A hard `|F_yf|, |F_yr| <= F_max` bound in the SQP, additional to the soft `tanh` ceiling, with `F_max = m * ceiling(v_x) / 2` per axle.

**Why it is disabled.** Live on `comp_test_map_3`, the SQP subproblem failed (`nmpc_status = 0`) on 77.5% of 614 ticks, steering sat at full lock on 30.8% of ticks from t = 0.65 s, and the run ended in a stall 4.94 m off-track with heading error -52 degrees. The hard rows have no slack, so ordinary cornering geometry and the bound conflict under normal driving, the subproblem goes infeasible, and the controller stops updating steering. No offline A/B was run first.

**To retry.** Re-derive a looser `F_max` and add slack. The missing telemetry columns (`nmpc_fyf_max_abs`, `nmpc_fyr_max_abs`) are now declared in `fsae_control/telemetry/columns.py`. Run an offline A/B with `python -m tuner.validation.nmpc_offline_check` first.

**Record.** [late_turn_in_investigation.md](../logs/late_turn_in_investigation.md), Part 16.10.

## NMPC progress term (disabled)

**What it is.** `nmpc_progress_enabled` (default false, code present). A reward for arc-length progress plus a one-sided speed cap, replacing the `q_e_v` speed-tracking cost, so the solver picks its own speed.

**Why it is disabled.** It does not reliably complete a lap at any weight, and never beat the tracking controller (offline score 0.757, or 0.714 at the later deficit setting):

| Configuration | Score | Progress | Off-track |
|---|---|---|---|
| `q_progress` 3 | 13.000 | 0.000 | no (never launches) |
| `q_progress` 5 | 12.583 | 0.570 | yes |
| `q_progress` 7 | 15.417 | 0.097 | yes |
| `q_progress` 5 plus linear slack | 0.892 | 0.995 | no, but not repeatable |

- Below the band the reward cannot pay for the roughly 2.3 m/s² needed to break static friction (`F_stiction` 600 N over 255 kg), so the car never moves.
- Above it the car carries too much speed into corners. The Frenet progress rate `s_dot` rewards sitting on the inside of a bend, and a quadratic track penalty has zero gradient at zero violation.
- The one completed lap was a lucky draw. At `SPEED_TARGET_DEFICIT_MAX` 5.0, raising the slack weight 50x changed nothing at any `q_progress`. Enabling it by default was attempted on 2026-09-21 and reverted the same day.

**Side result kept.** `SPEED_TARGET_DEFICIT_MAX` 2.5 was the binding acceleration constraint for 36.8% of a normal lap. Raising it to 5.0 made the lap about 2.5 s faster with lower `|e_y|` and saturation. The dataclass default `speed_target_deficit_max` is 2.55, and the `launch_all.sh` line for 5.0 is commented out.

**Record.** [nmpc_progress_term_investigation.md](../logs/nmpc_progress_term_investigation.md).

## Dynamic speed cap (disabled in the shipped launch)

**What it is.** `min(precomputed profile, live curvature cap)` on the target speed ([control_mechanisms.md](control_mechanisms.md#dynamic-speed-cap-a-live-brake-early-floor-under-the-precomputed-profile)). Code default and YAML default are on. `ENABLE_DYNAMIC_SPEED_CAP=false` in `ros2/launch_all.sh` turns it off.

**Why it is disabled.** Offline with the planner in the loop (`--planner`, `comp_test_map_3`):

| Metric | Cap off | Cap on |
|---|---|---|
| Steering saturation | 4.37% | 5.54% |
| `|e_psi|` mean/p90 | 7.04 / 15.94 degrees | 8.01 / 16.82 degrees |
| a_lat max | 10.06 | 10.61 |
| a_lat above ceiling | 2.92% | 0.62% |
| Score (lower is better) | 0.503 | 0.520 |

The cap does what it targets (a_lat above the ceiling fell 4.7x), while saturation, heading error and the score got worse. The cause was not diagnosed. A candidate is interaction with the rise limiter and the tracking gate (braking early for one corner leaves a worse heading into the next), never confirmed. It also performed worse subjectively on the live car the same day.

**Do not re-enable** for a live run without diagnosing why saturation and heading error rose.

**Record.** [late_turn_in_investigation.md](../logs/late_turn_in_investigation.md), "Addendum: dynamic speed cap".

## What survives from the retired code

| Item | Where it lives now |
|---|---|
| The measured lateral-acceleration ceiling law (7.5 flat, 0.47 slope, 2.46 intercept) | `_Plant` defaults in `fsae_control/nmpc/dynamics.py`, `alat_ceiling_at()` in `model/vehicle_physics/params.py`. It is a measured simulator property, not a tuning knob ([simulator_fidelity.md](simulator_fidelity.md), [vehicle_physics.md](vehicle_physics.md)). |
| Current-curvature `_corner_factor`, `_blend`, `_low_speed_corner_boost` | [control_mechanisms.md](control_mechanisms.md) |
| `adaptive_Q_scaling`, `steer_rate_anti_hunt`, `reversal_penalty_boost` (current-state only) | [control_mechanisms.md](control_mechanisms.md) |
