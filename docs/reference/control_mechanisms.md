# Control Mechanisms Reference

Per-mechanism reference for the control stack: what each mechanism does, why it exists, how it works, and what to be careful of when changing it.

## Scope and vocabulary

A **mechanism** is a layer added on top of a controller's core solve loop. It is enabled and tuned independently and targets one known failure mode (steering chatter, late turn-in, sensor lag). Switching it off leaves a controller that still drives, without the fix.

This doc covers mechanisms that exist in the code today. Related docs:

- Which knob to turn and to what value: [tuning.md](../guides/tuning.md).
- How the controllers themselves are built: [lmpc.md](../controllers/lmpc.md), [nmpc.md](../controllers/nmpc.md), [stanley.md](../controllers/stanley.md).
- Mechanisms that were removed, rejected or disabled, with the evidence: [retired_mechanisms.md](retired_mechanisms.md).
- The reference path and speed profile: [reference_path_and_speed.md](reference_path_and_speed.md).
- Live/offline numeric parity of every weight: [offline_live_parity.md](offline_live_parity.md).

`use_nmpc` is not a mechanism. It swaps the whole solve loop (LTV-QP: one convex QP per tick; NMPC: one real-time-iteration SQP step per tick). Each mechanism below belongs to LTV-QP only, NMPC only, or both.

Three layers set a value at run time, in increasing priority: the dataclass default (`mpc_params.py`, `nmpc_params.py`), the YAML default (`fsae_params.yaml`), and a `launch_all.sh` argument. Offline, the matching uppercase constant lives in the `settings/` package. The shipped `launch_all.sh` differs from the dataclass default for several rows in the status table.

## Status: code default versus shipped launch

The shipped `ros2/launch_all.sh` (mirrored in `fsds_simulator/launch_all.sh`) sets `USE_NMPC=true`. A plain launch therefore runs the NMPC, and every LTV-QP-only mechanism is inactive on the car. Offline, `settings.USE_NMPC` defaults to `False`, so the offline rollout runs the LTV-QP unless the flag is set.

| Mechanism | Controller | Code default | Shipped `launch_all.sh` |
|---|---|---|---|
| Corner-factor scheduler | LTV-QP | always on, no flag | not applied (NMPC runs) |
| Speed-based `R[0,0]` scaling | LTV-QP | always on | not applied |
| Heading-error accel/brake asymmetry (`epsi_ra_*`) | LTV-QP | always on | not applied |
| Steer-rate anti-hunt | LTV-QP | on (`steer_rate_anti_hunt_enabled`) | not applied |
| Steer-rate anti-hunt | NMPC | off (`nmpc_steer_rate_anti_hunt_enabled`) | off |
| Adaptive Q-scaling | LTV-QP | on (`adaptive_q_scaling_enabled`) | `MPC_ADAPTIVE_Q_SCALING_ENABLED=false` |
| Reversal penalty | LTV-QP | off (`reversal_penalty_enabled`) | `REVERSAL_PENALTY_ENABLED=true` |
| Reversal penalty | NMPC | off (`nmpc_reversal_penalty_enabled`) | `NMPC_REVERSAL_PENALTY_ENABLED=false` |
| Slew-rate limit `du_max` | both | fixed, 180 deg/s and 0.6 per step | fixed |
| Accel/brake effort split | both | `r_a_accel` 0.9, `r_a_brake` 0.6 | not overridden |
| Precomputed heading-lead profile | LTV-QP | off (`use_precomputed_heading_profile`) | `USE_PRECOMPUTED_HEADING_PROFILE=false` |
| Dynamic speed cap | node, both controllers | on (`enable_dynamic_speed_cap`) | `ENABLE_DYNAMIC_SPEED_CAP=false` |
| Stanley speed-target smoothing | Stanley | always on (live-path branch) | n/a |
| NMPC corner rate blend | NMPC | off (`nmpc_corner_rrate_blend_enabled`) | `NMPC_CORNER_RRATE_BLEND_ENABLED=false` |
| NMPC three-zone rate schedule | NMPC | on (`nmpc_rrate_zone_enabled`), 2.0 / 0.80 / 0.15 | on, same values |
| NMPC input-jerk cost | NMPC | `nmpc_rjerk_delta` 150.0, `nmpc_rjerk_a` 0.0 | `NMPC_RJERK_DELTA=150.0` |
| NMPC rate stage ramp | NMPC | off (`nmpc_rrate_stage_ramp_enabled`) | commented out (off) |
| NMPC spline reference | NMPC | on (`nmpc_spline_reference_enabled`) | commented out (on) |
| NMPC lateral-acceleration ceiling in the prediction | NMPC | on (`nmpc_alat_ceiling_enabled`) | commented out (on) |
| NMPC friction-circle constraint | NMPC | off | off, see [retired_mechanisms.md](retired_mechanisms.md) |
| NMPC progress term | NMPC | off | off, see [retired_mechanisms.md](retired_mechanisms.md) |

Two offline defaults differ from the shipped launch: `settings.ADAPTIVE_Q_SCALING_ENABLED` is `True` and `settings.REVERSAL_PENALTY_ENABLED` is `False`, while the launch file sets the opposite for both. `settings.ENABLE_DYNAMIC_SPEED_CAP` is `True` offline. Compare like with like before reading an offline score against a live one.

## Corner-factor scheduler (LTV-QP): weights follow the current curvature

**What it does.** The controller weights its costs differently on a straight and in a corner. It blends smoothly between the two sets of weights using how sharply the road bends at the car right now. It does not forecast the corner ahead.

**Why it exists.** The LTV-QP needs some way to be gentle and quiet on straights and firm in corners. A single fixed weight set trades one for the other.

**Why this design.** A forward scan of the path was tried first and removed (see [retired_mechanisms.md](retired_mechanisms.md#the-lookahead-gain-scheduling-family-removed-structurally-unable-to-anticipate)). The LTV-QP's own prediction already covers the horizon, so reweighting today's near-zero cost from a forward scan cannot make the prediction see a bend. A current-curvature blend is the small, honest replacement. The real fix for anticipation is the NMPC.

**How it works.**

- `kappa` is the path curvature about 1 m ahead of the nearest waypoint, computed once per tick by `_curvature()` in `fsae_control/lmpc/adaptive_gains.py`. Offline, `curvature_estimate()` in `controller/model_utils.py` uses a different signal, `|yaw_rate / v_x|` from the plant state. The two agree only when the car follows the path.
- The corner fraction combines two saturating terms:

```
corner_factor   = 1 - 1 / (1 + k * |kappa|)                 # k = corner_factor_k (8.0)
low_speed_boost = corner_factor * max_extra * v_half / (v_half + v)
corner_frac     = clip(corner_factor + low_speed_boost, 0, 1)
```

- `low_speed_boost` is multiplied by `corner_factor`, so it is exactly zero on a straight at any speed. The removed low-speed rate boost fired on speed alone.
- Each weight is a linear blend `straight + (corner - straight) * corner_frac`:

| Weight | Straight endpoint | Corner endpoint |
|---|---|---|
| `Q[0,0]` (`e_y`) | `q_ey_straight` 4.5 | `q_ey_corner` 9.0 |
| `Q[2,2]` (`e_psi`) | `q_epsi_straight` 1.5 | `q_epsi_corner` 3.0 |
| `Q[3,3]` (yaw rate) | `q_r_straight` 1.0 | `q_r_corner` 0.5 |
| `R_rate[0,0]` (steer rate) | `rrate_steer_straight` 2.0 | `rrate_steer_corner` 1.25 |

- `R[0,0]` (steering effort) first gets the speed-based scale `1 + 1.5 v / (6 + v)` from `_adaptive_R_scaling`, then blends toward `r_steer_corner_mid` (1.35), a middle value, so it never reaches the extremes of the other four.
- An always-on heading-error asymmetry scales the accel and brake effort weights by `frac = |e_psi| / (|e_psi| + epsi_ra_half_rad)`:

```
r_a_accel_eff = r_a_accel * (1 + (epsi_ra_accel_boost_max - 1) * frac)      # max 2.0
r_a_brake_eff = r_a_brake * (1 - (1 - epsi_ra_brake_floor) * frac)          # floor 0.5
```

- Order inside `compute()` in `fsae_control/lmpc/controller.py`: `R_rate_scaled` starts as a copy of `R_rate`, the anti-hunt and reversal multipliers are applied and logged, then the blend overwrites `R_rate_scaled[0,0]` and multiplies both logged multipliers back in on the same line. `adaptive_Q_scaling` runs last on the blended `Q`.
- Offline mirror: `solve_ltv_tick()` in `sim/rollout/tick_solve.py`, using `controller/model_utils.py` and the `settings/lmpc.py` constants.

**Tuning and pitfalls.**

- An assignment to `R_rate_scaled[0,0]` that omits a multiplier silently discards it while still logging it. The removed adaptive `R_rate` floor died this way ([retired_mechanisms.md](retired_mechanisms.md#adaptive-r_rate-curvature-floor-removed-computed-then-always-overwritten)). Any new `R_rate[0,0]` multiplier must be threaded through the blend line.
- Corner and straight endpoints are parity constants. Change live and offline together ([offline_live_parity.md](offline_live_parity.md)).
- The NMPC does not use this scheduler. Its own rate-cost shaping is covered under the NMPC sections below.

## Steering-rate anti-hunt: costlier steering changes when nothing needs doing

**What it does.** Steering chatter (rapid small back-and-forth wheel motion) is most likely when the car is straight, centred and aligned, so it needs almost no steering. The mechanism makes fast steering changes more expensive in exactly that state and leaves the cost unchanged elsewhere.

**Why it exists.** Residual steering hunting on straights. It was added as an experiment and never validated against a live log, so treat it as unproven.

**Why this design.** A hard threshold on curvature caused solver-iteration spikes at the crossing, so the boost is a continuous product of three saturating terms instead.

**How it works.** `_steer_rate_anti_hunt` (live) and `steer_rate_anti_hunt` (offline) multiply `R_rate[0,0]`:

```
boost_kappa = 1 / (1 + 30.0 * |kappa|)
boost_ey    = 1 / (1 + 15.0 * |e_y|)
boost_epsi  = 1 / (1 + 11.5 * |e_psi|)
scale       = 1 + (boost_max - 1) * boost_kappa * boost_ey * boost_epsi
```

- Full strength needs all three inputs near zero. The boost fades continuously, never snaps.
- `e_psi` is included so a car that is centred but still misaligned after a corner exit does not get the full boost, which would make the yaw-back correction expensive.
- `boost_max` is `anti_hunt_boost_max` (6.0). `enabled=False` returns the matrix unchanged.
- The NMPC imports the same function. Its flag is `nmpc_steer_rate_anti_hunt_enabled` (off), with `nmpc_anti_hunt_boost_max` (`-1.0` inherits).
- On the NMPC, `kappa` comes from the spline at the projected arc length, not the 1 m preview.

**Tuning and pitfalls.**

- Offline `steer_rate_anti_hunt` hardcodes `boost_max = 6.0`. `settings/` has no `ANTI_HUNT_BOOST_MAX`, so `nmpc_anti_hunt_boost_max` has no offline effect. This is a standing live/offline gap ([offline_live_parity.md](offline_live_parity.md)).
- On the NMPC, anti-hunt and the corner rate blend are alternatives. The blend takes priority if both are enabled.

## Adaptive Q-scaling (LTV-QP): softer lateral pull near the centreline

**What it does.** When the car is already very close to the path, the normal quadratic lateral-error cost still pulls at full proportional strength, which can encourage darting across the line. This mechanism halves that pull when `|e_y|` is small.

**Why it exists.** A live log showed steering reversal rate rising as `|e_y|` shrank. The offline recorded-map rollout shows the opposite trend, so the effect could not be reproduced offline.

**Why this design.** A linear ramp with a floor is the simplest shape that leaves large errors untouched.

**How it works.** `_adaptive_Q_scaling` (live) and `adaptive_Q_scaling` (offline) multiply `Q[0,0]`:

```
scale = 0.5                                        |e_y| <= 0.05
scale = 0.5 + 0.5 * (|e_y| - 0.05) / (0.3 - 0.05)  0.05 < |e_y| < 0.3
scale = 1.0                                        |e_y| >= 0.3
```

- The thresholds (0.05, 0.3, floor 0.5) are hardcoded in the function bodies on both sides, not `MPCParams` fields.
- They are far below `tracking_error_speed_gate`'s 0.5 to 2.0 m ramp, which handles a different regime (badly off the line).
- It runs last in the `Q[0,0]` pipeline, after the corner blend, on the delay-compensated `e_y` (`x0[0]`).

**Tuning and pitfalls.**

- No before/after evidence supports it. The corner-approach interaction found once (softening `Q[0,0]` right before a good turn-in) is the reason the shipped launch turns it off.
- Rationale for the code default staying `True` is not recorded.

## Soft steering-reversal penalty: costlier steering changes when steering is near zero

**What it does.** A steering reversal (wheel flicks one way, then the other) always passes through zero steering. The penalty makes any steering change costlier when the previous command was near zero, which discourages the flip without penalising it directly.

**Why it exists.** Reduce sign-flip steering chatter.

**Why this design.** A reversal depends on this tick's own decision, so penalising it directly would make the cost non-convex. `u_prev_steer` is a known constant at solve time, so keying on it keeps the term an ordinary quadratic.

**How it works.** `_reversal_penalty_boost` (live), `reversal_penalty_boost` (offline):

```
boost_near_zero = 1 / (1 + k * |u_prev_steer|)          # k = reversal_penalty_k = 8.0 per rad
scale           = 1 + (boost_max - 1) * boost_near_zero  # boost_max = 4.0
R_rate[0,0]    *= scale
```

- `k = 8` gives half boost at about 7.2 degrees of previous steering.
- Separate flags and `-1.0`-inherit overrides exist per controller: `reversal_penalty_*` (LTV-QP) and `nmpc_reversal_penalty_*` (NMPC).
- On the NMPC the multiplier applies on top of whichever base `R_rate[0,0]` value ran first (corner blend or anti-hunt), tracked through `rrate_steer_current` in `fsae_control/nmpc/solver.py`.

**Tuning and pitfalls.**

- Offline A/B on `comp_test_map_3` showed about a 20% reversal-count reduction on the LTV-QP at negligible cost, with no DNF. On the NMPC, reversals fell only about 5.6% while the composite score worsened, so the shipped launch keeps `NMPC_REVERSAL_PENALTY_ENABLED=false`. Those two figures are quoted from the investigation notes and were not re-run.
- Not live-tested on either path.

## Slew-rate limit `du_max`: identical on both sides at 180 deg/s

**What it does.** The steering rack cannot turn arbitrarily fast, so the solver may not command a bigger step in steering, or in acceleration, than the actuator can follow in one tick.

**Why it exists.** Live telemetry (live log `mpc_standalone_control_1785976976`) showed the steering command pinned on an 80 deg/s limit for 41% of control steps, reversing at about 8 Hz. That is a limit cycle from the rate limit, not a weight problem. Inverting logged yaw rate through the kinematic bicycle (`delta = atan(L r / v)`) put the achieved roadwheel rate at p99 about 138 deg/s and maximum about 218 deg/s, so the actuator manages at least about 200 deg/s.

**Why this design.**

- 180 deg/s sits just under the measured floor. It is a lower-bound estimate, not a datasheet figure. The true FSDS rate is not recoverable from the repo (the vehicle setup lives in git-LFS assets).
- The bound is expressed as a rate times `dt` so its physical meaning survives a change of tick period.
- The offline sim did not show the original 80 deg/s limit as a problem: it hit the limit on 0.5% of steps against 41% live, and composite scores at 80 and 180 deg/s differed by under 0.002 on the synthetic paths. Offline uses a fixed `DELAY_STEPS = 1` and smooth paths, so it never enters the saturated regime. A flat offline score is not evidence that the constraint is harmless.

**How it works.**

- `du_max = [radians(180) * dt, 0.6]`: steering in rad per tick, acceleration in m/s² per tick (12 m/s³ at `dt = 0.05`).
- Live LTV-QP: hard constraints in `_build_qp` (`fsae_control/lmpc/controller.py`), including the step-0 constraint against `u_prev`.
- Live NMPC: `NMPCController.du_max` in `fsae_control/nmpc/solver.py`, enforced as QP rows and in the warm-start projection.
- Offline LTV-QP: `init_parameterized_mpc()` in `controller/lmpc/build.py` includes the step-0 constraint too. `du_max` is cached like `u_min`/`u_max`, so a changed value rebuilds the problem.
- Offline value: `vehicle_params.max_steer_rate * settings.DT` from `model/vehicle_physics/params.py`, so the plant and the controller share one number.

**Tuning and pitfalls.**

- Raising it previously regressed smoothness metrics. Re-measure with the steering system-ID harness ([debugging_tools.md](../guides/debugging_tools.md)) before changing it, and change live and offline together.

## Accel/brake effort weight split

**What it does.** Braking and accelerating carry separate effort costs, so making the controller keener to brake does not change how keen it is to accelerate.

**Why it exists.** A single shared `r_a` loose enough to accelerate well on straights was also too loose on braking, and the reverse. Diagnosis history is in [late_turn_in_investigation.md](../logs/late_turn_in_investigation.md) ("Part 0 (background): how the accel/brake effort split came about") and [sim_to_real_investigation.md](../logs/sim_to_real_investigation.md) (section 59, the preceding single-scalar cut).

**Why this design.** A slack-variable formulation was considered and rejected in favour of `cp.pos`/`cp.neg`. The two terms are individually convex, and `pos(x)² + neg(x)² = x²`, so equal weights reproduce the old single weight exactly.

**How it works.**

```
r_a_accel * sum(pos(a_cmd)^2) + r_a_brake * sum(neg(a_cmd)^2)
```

- Live LTV-QP: `r_a_accel_param`/`r_a_brake_param` in `_build_qp`. Offline: `solve_mpc()` in `controller/lmpc/solve.py`, fed by `settings.R_A_ACCEL`/`R_A_BRAKE`.
- Current defaults: `r_a_accel` 0.9, `r_a_brake` 0.6 in `mpc_params.py`, `fsae_params.yaml` and `settings/`. Recheck before quoting, since these are among the most-retuned weights.
- `R_diag[1]` remains a nominal reporting value. No adaptive gain touches index 1 of `R` or `R_rate`.
- On the LTV-QP the heading-error asymmetry above scales both per tick.

**Tuning and pitfalls.** Do not reintroduce a single shared weight without accounting for why it was split.

## Frozen target speed across the horizon

Not a mechanism, a property of the LTV-QP. The controller takes one target speed at the start of each solve and holds it across the whole horizon. It enters as `x0[4] = e_v = v - v_desired`, and nothing in `Ad`/`Bd` re-references it. The next tick refreshes it. See [lmpc.md](../controllers/lmpc.md) for the full explanation.

The NMPC's `q_e_v` cost also tracks one frozen `v_ref` per solve. The per-stage horizon speed profile that would have changed this was tried and removed ([retired_mechanisms.md](retired_mechanisms.md#nmpc-horizon-speed-profile-two-variants-removed)).

## Dynamic speed cap: a live brake-early floor under the precomputed profile

**What it does.** The precomputed speed profile assumes the car is where the plan expects. If the car runs faster than planned, the cap pulls the target speed down before the next corner, never up.

**Why it exists.** A live log showed a corner entered at about 9 m/s against a 4.6 m/s target, with steering pinned at 25 degrees for over a second. The static profile lookup has no notion of the car's actual speed against the remaining braking distance.

**Why this design.** The cap layers under the precomputed lookup with `min()` rather than replacing it, because the profile encodes the whole-lap optimisation. It uses tighter constants than the plain live branch (`a_lat_max` 3.2 and `safety` 0.9, against 4.75 and 1.0) so it engages before the oracle profile is violated.

**How it works.**

```
v_curv = min(precomputed_speed_at(...), dynamic_speed_cap(path_ahead, a_lat_max=3.2, safety=0.9))
```

- `dynamic_speed_cap()` in `fsae_control/control_utils.py` wraps `curvature_speed()`.
- It runs in the control node (`fsae_control/mpc/control_step.py`), upstream of either controller's solve, only when a precomputed speed profile is loaded.
- The gate (`tracking_error_speed_gate`), the fall limiter and `SPEED_TARGET_RISE_RATE` apply downstream unchanged.
- Parameters: `enable_dynamic_speed_cap`, `dynamic_cap_a_lat_max`, `dynamic_cap_safety`. Offline: `settings.ENABLE_DYNAMIC_SPEED_CAP` and the `DYNAMIC_CAP_*` constants.

**Tuning and pitfalls.**

- Status is disabled in the shipped launch ([retired_mechanisms.md](retired_mechanisms.md#dynamic-speed-cap-disabled-in-the-shipped-launch)). The YAML default is still `true`, so a launch path that bypasses `launch_all.sh` runs with the cap on.
- The offline rollout only reaches this code with the planner in the loop (`python -m tuner.validation.recorded_map_rollout --planner`).

## Stanley speed-target smoothing: the MPC limiters ported to Stanley

**What it does.** With no precomputed speed profile, the planner rebuilds the path every tick, and a few centimetres of lateral wiggle can make the curvature-derived target jump several m/s in one 50 ms tick. The smoothing rate-limits and gates the target so a noisy tick cannot yank the speed.

**Why it exists.** Stanley had no smoothing. A noisy tick could spike steering (cross-track error comes from the same noisy path) and drop the speed target together, with nothing pulling the target back up, which produced a live spin-out within the first seconds of a run. Now live-tested per [stanley.md](../controllers/stanley.md).

**Why this design.** It reuses the three safeguards `control_step.py` already applies around `curvature_speed()`, so both controllers see the same speed logic.

**How it works.** Live-path branch only (no `map_path`), in `fsae_control/stanley_controller.py`, applied in order with a measured `dt` (Stanley runs off pose arrival, not a fixed timer):

| Step | Constant | Value |
|---|---|---|
| Limit how fast `curvature_speed()` output may fall | `V_CURV_FALL_RATE` | 7.0 m/s² |
| Scale the target down when `|e_y|`/`|e_psi|` are large, rate-limited | `GATE_RATE_LIMIT` | 2.0 per s, either direction |
| Bound the rise of the composed target, seeded from actual speed on the first tick | `SPEED_TARGET_RISE_RATE` | 7.0 m/s² |

## Precomputed shaped heading-lead profile (LTV-QP only, off)

**What it does.** The car is told to start turning in slightly before the geometry says, by however much yaw it can physically achieve before the bend at the planned speed. It replaces the geometric heading reference (`atan2` of the path tangent) with a shaped one.

**Why it exists.** Late turn-in on sudden corners. The LTV-QP's prediction cannot see the bend, so a lead in the reference heading is a way to hand it early error to react to.

**Why this design.**

- It changes the reference `e_psi` is measured against at `k = 0`, before the QP runs. Curvature forcing and a cost-target shift were both rejected because the QP is free to satisfy an added future deviation however is cheapest ([retired_mechanisms.md](retired_mechanisms.md#curvature-forcing-term-removed-the-solver-defers-external-data)).
- The lead is scaled by achievable yaw rate at each station's planned speed. A fixed lookahead distance saturates steering to full lock at about 8 m of lead on a realistic corner-entry ramp.

**How it works.**

- Computed once, offline, per waypoint by `build_shaped_heading_profile()` in `tuner/tools/raceline_optimizer.py`, run in `export()` after path and speed have converged. It does not feed back into the path or speed optimisation.
- CSV columns: `x,y,psi,psi_target,v_target`. A four-column file loads with `psi_target = psi`.
- Live loader: `load_path_heading_profile_csv()` in `fsae_control/control_utils.py`. Consumer: `MPCController.set_heading_profile()`, used in `_error_state()` for `e_psi` only. `e_y` keeps the geometric tangent.
- `check_slip()` and `SLIP_LIMIT_RAD` (5 degrees) are a diagnostic. The limit is an unvalidated placeholder.
- Toggle: node parameter `use_precomputed_heading_profile` (not an `MPCParams` field), `USE_PRECOMPUTED_HEADING_PROFILE` in `launch_all.sh`.

**Tuning and pitfalls.**

- No effect when `use_nmpc` is true. The NMPC node accepts the profile, ignores it and logs one line.
- Do not extend it from a `k = 0` lead to a per-horizon-step reference. That reproduces curvature forcing's wrong-direction trap ([late_turn_in_investigation.md](../logs/late_turn_in_investigation.md), Part 15).
- The lead stays active through the whole corner on tracks with few true straights, fighting the corner geometry. The launch default is off. Live test history (an initial "worse" read, then a high-variance correction) is in Parts 7 to 13 of the same log.

## Nonlinear MPC (`use_nmpc`): the second controller

**What it does.** `NMPCController` solves a nonlinear version of the same tracking problem, so its prediction sees the road curve ahead instead of reacting only to error that has already appeared. It replaces `MPCController` wholesale when `use_nmpc` is true (node parameter, default false, shipped launch true).

**Why it exists.** `MPCController._discrete_model` is the bicycle model in error coordinates with the reference frame's rotation dropped, and it lacks the term `e_psi_dot = r - kappa(s) * s_dot`. With `e_y = e_psi = 0` the QP predicts zero error forever, and no weighting can produce turn-in before real error exists. Measured: the LTV-QP commands exactly 0.000 degrees at all 8 dead-on-line test states approaching a known bend.

**Why this design.**

- `kappa` is a function of the state `s` (driven by the car's own predicted motion), not of the horizon index, so the anticipation obligation cannot be scheduled early the way the curvature-forcing and heading-lead workarounds do.
- MPCC's progress-maximising formulation was not adopted wholesale, because it reintroduces an exogenous, schedulable future obligation. The formulation survey and falsification method are in [late_turn_in_investigation.md](../logs/late_turn_in_investigation.md), Part 16.
- Gauss-Newton SQP with a condensed dense QP solved by OSQP, one iteration per tick, warm-started. Horizon `nmpc_horizon` 20 and one iteration were chosen from a closed-loop sweep (Part 16.7): N=35 tracked worst, and extra iterations were slightly worse and about 2x slower. Mean solve at N=20, one iteration: 9.1 ms.

**How it works.**

- Live: `NMPCController` in `fsae_control/nmpc/solver.py`, split across `layout`, `reference`, `dynamics`, `outputs`, `weight_schedule`, `qp_model`, `sqp_step`. Offline port: `controller/nmpc/` with the same module names, not an import (the repos cannot import each other). Driven by `sim/rollout/core.py` when `settings.USE_NMPC` is true.
- States `[s, e_y, e_psi, v_x, v_y, r, delta_act, a_act]`, inputs `[delta_cmd, a_cmd]`. The full derivation is in [nmpc.md](../controllers/nmpc.md) and the error definitions in [error_states.md](error_states.md).
- Vehicle constants (`lf` 0.70, `lr` 0.85, `m` 255, `Iz` 150, `Cf`, `Cr`, `tau_delta` 0.08, `tau_a` 0.02) and the kinematic/dynamic blend band (1.0 to 2.5 m/s) match the LTV-QP. Live they are hardcoded in `_Plant` and `MPCController.__init__`. Offline they are read from `VehicleParams`.
- Cost weights come from the same `MPCParams` instance.
- Three deliberate differences from the LTV-QP:

1. `q_r` weights heading-error rate `r - kappa * s_dot`, not absolute yaw rate. Same slot, different regressor, so re-sweep it.
2. `e_y` and `e_psi` are measured against the smoothed spline reference, not the raw segment tangent.
3. FSDS's measured lateral-acceleration ceiling sits inside the prediction as a smooth `tanh` saturation of predicted tyre forces, with the law `max(7.5, 0.47 |v_x| + 2.46)` hardcoded as `_Plant` defaults. `nmpc_alat_ceiling_enabled=false` recovers the unconstrained plant.

- Tyre lateral force is multiplied by the kinematic/dynamic blend after the ceiling saturation. Without this the slip-angle floor at low `v_x` lets steering manufacture lateral force at zero speed (the standstill hard-steer bug, [late_turn_in_investigation.md](../logs/late_turn_in_investigation.md) sections 16.11 and 16.12).

**Tuning and pitfalls.**

- Inactive when `use_nmpc` is true: the corner-factor scheduler, speed-based `R[0,0]` scaling, LTV-QP anti-hunt, adaptive Q-scaling, `use_precomputed_heading_profile` and `ref_heading_rate_limit_enabled`. No `m_*` telemetry columns are written for these.
- Unchanged under the NMPC: `use_precomputed_path`, `use_precomputed_speed`, `enable_dynamic_speed_cap`, and delay compensation (`delay_compensation_enabled`, `pose_age_lp_alpha`, `n_delay_hysteresis`, `max_delay_compensation_steps`). The delay rollforward uses the nonlinear model instead of `predict_ahead()`.
- The horizon is short (1.0 s). Anticipation is limited to bends within `v * N * dt`.

### Three-zone rate schedule (`nmpc_rrate_zone_enabled`)

**What it does.** Steering-rate cost is boosted on a clear straight, eased while a corner is ahead (so turn-in is not taxed) and lowered mid-corner (so chatter is still damped).

**Why it exists.** A single flat rate weight cannot both kill chatter on straights and allow a fast turn-in. It is the smallest contributor of the four chatter levers found ([steering_chatter_investigation.md](../logs/steering_chatter_investigation.md), "Resolution summary").

**Why this design.** A continuous surface with no thresholds or hysteresis. It degrades gracefully on winding roads and on a lone kink.

**How it works.** Both inputs pass through the same saturating curve with `k = nmpc_corner_factor_k` (27.0):

```
now   = corner_factor(|kappa_now|)                  # spline kappa at the car's arc length
ahead = corner_factor(max |kappa| over predicted horizon)
lead  = max(0, ahead - now)
s     = boost_straight + (ease_approach - boost_straight) * lead
zone  = s + (floor_corner - s) * now
```

- Defaults: `boost_straight` 2.0, `ease_approach` 0.80, `floor_corner` 0.15.
- `_rrate_zone_scale()` in `fsae_control/nmpc/weight_schedule.py`. It multiplies the whole-horizon `R_rate[0,0]` array after the horizon rollout, on top of the corner blend, anti-hunt and reversal composition, before the SQP cost matrix is rebuilt.

**Tuning and pitfalls.**

- Endpoints are reached only as the corner factor approaches 1. At `k = 8` a track whose tightest corner has `|kappa|` near 0.2 tops out near 0.63 and the schedule degrades to a mild global boost. Check the `m_Rrate_zone` telemetry column against `floor_corner` before concluding the endpoints did anything. Raise `k`, not the endpoints.
- `ease_approach` 0.80 is the shipped value, not the intended 0.35, which DNFs offline.
- `nmpc_corner_factor_k` is shared with the corner rate blend. It is not exclusive to either.
- Against a live planner path, `kappa_ahead` inherits the open centreline curvature-spike defect ([simulator_fidelity.md](simulator_fidelity.md), section on centreline curvature spikes), with no downstream limiter to absorb a spike.

### NMPC corner rate blend (`nmpc_corner_rrate_blend_enabled`, off)

Blends `R_rate[0,0]` between `nmpc_rrate_steer_straight` and `nmpc_rrate_steer_corner` by current curvature, using the shared `_corner_factor`/`_blend`. It overwrites the base `R_rate[0,0]` (unlike the zone schedule, which multiplies), and takes priority over NMPC anti-hunt.

**Pitfall:** at the `-1.0` inherit value the endpoints resolve to the LTV-QP's 2.0 and 1.25 and the blend silently discards `r_rate_delta`. That gave 20.95% steering saturation, `|e_y|` 1.18 m and `|e_psi|` 12.1 degrees in the chatter investigation. Set both endpoints explicitly, never `-1`.

### Input-jerk cost (`nmpc_rjerk_delta`, `nmpc_rjerk_a`)

A second-difference penalty on the inputs, alongside the first-difference rate cost. A steady ramp into a corner scores near zero, an alternating wiggle scores high, so wobble is penalised without penalising turn-in. Defaults 150.0 and 0.0 (0.0 removes the term). Derivation, QP integration points and measurements: [tuning.md](../guides/tuning.md). The `_cost()` function must carry the same term as the QP, because the SQP line search scores candidates with `_cost()`.

### Rate stage ramp (`nmpc_rrate_stage_ramp_enabled`, off)

Scales the steering-rate cost by horizon stage, from `nmpc_rrate_stage_near` (0.15) at stage 0 up to 1.0 at the last stage. It was rejected offline as a fix for shallow-corner jerk because it moved slew-limited ticks the wrong way (8.4% to 12 to 15%). It stays because it is the only change found that clears the offline `nmpc_offline_check` DNF. `nmpc_rrate_stage_near` = 1.0 is an exact no-op.

### Spline path reference (`nmpc_spline_reference_enabled`, on)

`PathReference` fits `x(s)` and `y(s)` as independent `CubicSpline` objects over arc length and derives `kappa(s)` and `psi_ref(s)` analytically, replacing the dense-resample, moving-average and finite-difference pipeline (still selectable behind the flag). It is a numerical-quality change with no new solver coupling, aimed at the centreline curvature-spike defect. It was adopted from the MPCC comparison on its own, without MPCC's progress variable. `nmpc_kappa_clip` (0.5) and `nmpc_kappa_rate_max` (2.0) bound the curvature profile.

### Which settings affect which controller

Every `MPCParams` and `NMPCParams` field carries a `metadata["controller"]` tag (`both`, `ltv_qp_only`, `nmpc_only`). `settings/` mirrors the classification as a `[LTV-QP only]`, `[NMPC only]` or `[shared]` comment prefix. Counts as of this writing: `MPCParams` 69 fields, `NMPCParams` 35. Count directly rather than trusting these.

- **Shared base weights**: `q_e_y`, `q_e_yd`, `q_e_psi`, `q_r` (meaning differs), `q_e_v`, `r_delta`, `r_a_accel`, `r_a_brake`, `r_rate_delta`, `r_rate_a`, `terminal_q_scale`. The NMPC reads them through a `_pick(override, inherited)` helper; a launch with every `nmpc_q_*`/`nmpc_r_*` override at `-1.0` starts the NMPC from the LTV-QP's tuned set.
- **Shared delay and speed fields**: `delay_compensation_enabled`, `max_delay_compensation_steps`, `pose_age_lp_alpha`, `n_delay_hysteresis`, `speed_target_deficit_max`. `predict_epsi_clip` is LTV-QP only.
- **LTV-QP only**: `adaptive_q_scaling_enabled`, `steer_rate_anti_hunt_enabled`, `anti_hunt_boost_max`, `ref_heading_rate_limit_enabled`, `ref_heading_rise_rate_deg_s`, `corner_factor_k`, the `q_*`/`rrate_*` straight and corner endpoints, `r_steer_corner_mid`, `low_speed_corner_boost_*`, `epsi_ra_*`, `reversal_penalty_*`.
- **NMPC-only overrides in `MPCParams`**: `nmpc_q_*` and `nmpc_r_*` weight overrides, `nmpc_q_progress`, the anti-hunt, reversal, corner-blend, zone, stage-ramp and jerk fields above.
- **`NMPCParams`** (`nmpc_params.py`): all NMPC-only. It holds `use_nmpc`, horizon and solver settings, Jacobian and Runge-Kutta sub-step gating, standstill steering damping, SQP trust and backtracking, the soft track constraint, curvature-reference construction, the ceiling, spline, friction and progress flags, latency compensation, OSQP tolerances and `nmpc_v_des_filter_alpha`.
- **`settings/` only, no dataclass field**: `USE_PRECOMPUTED_SPEED_PROFILE`, `ENABLE_DYNAMIC_SPEED_CAP` with `DYNAMIC_CAP_*`, `DELAY_STEPS`, `DELAY_JITTER_*`, `ALAT_CEILING_*`.

See [offline_live_parity.md](offline_live_parity.md) for the field-by-field mapping and [tuning.md](../guides/tuning.md) for values.

### NMPC steering chatter: three settings, plus a speed cause

Chatter while cornering (magnitude hunting tick to tick, not a sign-flip reversal) is controlled by settings, with the full history in [steering_chatter_investigation.md](../logs/steering_chatter_investigation.md).

| Lever | Effect |
|---|---|
| `r_rate_delta` (steering-rate cost) | The dominant lever. Raised from 2.8 to 52.5 in the investigation, which moved live chatter 2.538 to 1.919 degrees per tick. The current default is 100.0 on both sides. |
| `nmpc_rjerk_delta` = 150 | Holds 0 saturated and 0 slew-limited ticks and 1 reversal over three laps on the centreline. |
| Reference line | `centerline.csv` rather than `raceline.csv` took steering reversals from 13 to 1 and the score from 0.752 to 0.488. The raceline demands grip the tyre model cannot supply at that speed, which no controller weight fixes. |

- Sudden steering jumps at tight corners are a speed problem, not a controller fault. `CURVATURE_SPEED_A_LAT_MAX` 5.5 to 4.75 took stutters from 33.3 to 9.8 per minute ([reference_path_and_speed.md](reference_path_and_speed.md)). Check lateral-acceleration demand and speed overshoot before touching a steering weight.
- A residual of about 9.8 stutters per minute (1.5 to 3.2 degrees) clusters at corner exits with `|e_psi|` 13 to 16 degrees and small `|e_y|`. Not chased at this amplitude. The next lever is `q_e_psi`/`q_r` at corner exit. Reproduce with `python -m tuner.investigations.steering_chatter_check`.

### Dependencies

`osqp` only. CasADi and acados are not used by the shipped code (not installable into the ROS interpreter on Ubuntu 24.04 without `--break-system-packages`), and the SQP and Jacobians are numpy-only. CasADi was used once from a private install to cross-check the optimum against IPOPT. Reproduce the offline closed-loop check with `python -m tuner.validation.nmpc_offline_check`, or run `ros2/src/fsae_planning/control/fsae_control/test/nmpc_offline_check.py` (no ROS or FSDS session needed).

## Gradual-corner accel oscillation is track geometry, not a bug

This is a diagnostic note, not a mechanism.

Through mild S-curves or chicanes, `v_desired` and `a_cmd` can rise to a local peak, dip and rise again. The precomputed speed profile (`speed_profile.csv`, built by `compute_speed_profile()` in `sim/speed_profile.py`) is following a real sign change in path curvature: a bend straightens briefly before curving the other way, so the profile speeds up through the short straight and slows again for the next corner.

Before treating it as a bug, compare the car's `v_desired` and `a_cmd` against the track's own `speed_profile.csv` and the raw path's curvature at the same position. A sign change in curvature, not just its magnitude, confirms correct tracking.

- Do not "fix" it through `R_rate_diag[1]` or similar damping. That slows the response to a real upcoming tightening and trades a correctly anticipated slowdown for a late, harder one.
- If a specific track needs it addressed, the levers are the profile's generation parameters (`a_lat_max`, scan window) or the raw path geometry (a spurious kink), never the live adaptive gains. Confirm the geometry is spurious first. The worked example is in [late_turn_in_investigation.md](../logs/late_turn_in_investigation.md), "Gradual-corner accel oscillation".
