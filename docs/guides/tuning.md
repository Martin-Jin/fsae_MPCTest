# MPC Tuning Guide

This is the reference for tuning the MPC: what each weight and flag does, what is shipped, what was tried and rejected, and how to change a value safely.

Current numbers live in code, not here. Offline: the `settings/` package (`settings/lmpc.py`, `settings/nmpc.py`, `settings/general.py`, `settings/scoring.py`). Live: `mpc_params.py` and `nmpc_params.py` under `ros2/src/fsae_planning/control/fsae_control/fsae_control/mpc/`, plus `fsae_params.yaml` and `ros2/launch_all.sh`. Values quoted below are shipped defaults at the time of writing. Re-read the source before relying on one.

## Two controllers, two tuning surfaces

- The LTV-QP (linear time-varying quadratic program, `use_nmpc=false`) uses the base weights plus an adaptive-gain layer that reshapes them every tick. See [lmpc.md](../controllers/lmpc.md).
- The NMPC (nonlinear MPC, `use_nmpc=true`) reads the same base weights as a starting point. It ignores the adaptive-gain layer and has its own override fields and rate-shaping terms. See [nmpc.md](../controllers/nmpc.md).
- `ros2/launch_all.sh` ships `USE_NMPC=true` and `CONTROLLER=mpc`. The offline default `settings.USE_NMPC` is `False`, so the offline tuner and rollouts run the LTV-QP unless `USE_NMPC` is set.
- The shared weights are not unit-normalised. Each raw weight multiplies its raw-unit error term (`q_e_y` on metres squared, `q_e_psi` on radians squared). Do not compare two weights by size alone.

## How to change a value

1. Find the parameter below and read its known constraints.
2. Change it on both sides: the offline constant in `settings/` and the live field in `mpc_params.py` or `nmpc_params.py`. A one-sided edit makes offline scores meaningless. Field mapping: [offline_live_parity.md](../reference/offline_live_parity.md).
3. Check for runtime overrides. A launched node reads `ros2/launch_all.sh` arguments first, `fsae_params.yaml` second and the dataclass default last. Editing only the dataclass leaves the car on the old value. See the status table below.
4. Change one value at a time, by 20 to 30 percent, then re-validate. Weights interact and a second change hides the first.
5. Validate with `python -m tuner.validation.recorded_map_rollout` (headless, about 2 minutes). For anything touching adaptive gains or delay handling also run the synthetic `VALIDATION_SUITE`. For NMPC solver changes run `python -m tuner.validation.nmpc_offline_check`.
6. Do not trust an offline score alone for a change that moves saturation or heading error. The offline sim does not fully predict the car. See [simulator_fidelity.md](../reference/simulator_fidelity.md).

## What is shipped and what runs on the car

Three layers can hold a value. The launch layer wins.

| Mechanism | Dataclass default | `fsae_params.yaml` | `ros2/launch_all.sh` | Offline `settings/` | Runs on the car |
|---|---|---|---|---|---|
| NMPC (`use_nmpc`) | off | off | `USE_NMPC=true` | `USE_NMPC=False` | on |
| Steering-rate three-zone schedule (`nmpc_rrate_zone_enabled`) | on | on | `true`, endpoints 2.0 / 0.80 / 0.15 | on | on (NMPC only) |
| Input-jerk cost (`nmpc_rjerk_delta`) | 150.0 | 150.0 | `NMPC_RJERK_DELTA=150.0` | 150.0 | on (NMPC only) |
| `nmpc_corner_factor_k` | 27.0 | 27.0 | `27.0` | 27.0 | 27.0 (NMPC only) |
| Adaptive Q scaling (`adaptive_q_scaling_enabled`) | on | on | `MPC_ADAPTIVE_Q_SCALING_ENABLED=false` | on | off (LTV-QP only, inert under NMPC) |
| Steer-rate anti-hunt (`steer_rate_anti_hunt_enabled`) | on | on | line commented out | on | on (LTV-QP only, inert under NMPC) |
| Reversal penalty (`reversal_penalty_enabled`) | off | off | `REVERSAL_PENALTY_ENABLED=true` | off | on (LTV-QP only, inert under NMPC) |
| NMPC reversal penalty | off | off | `false` | off | off |
| Dynamic speed cap (`enable_dynamic_speed_cap`) | on (node parameter) | on | `ENABLE_DYNAMIC_SPEED_CAP=false` | on | off |
| Reference-heading rate limit | off | off | not exposed | off | off |
| Precomputed heading profile | off (node parameter) | not present | `false` | not modelled | off |

The launch file disagrees with the offline default for adaptive Q scaling and the reversal penalty. Both only matter when the LTV-QP runs, which needs `USE_NMPC=false`. Offline LTV-QP results are then not a like-for-like preview of a launched LTV-QP.

## Core cost weights

`Q_diag`, `R_diag`, `R_rate_diag` (offline) and the matching `MPCParams` fields set the base driving personality: tracking accuracy against control effort against smoothness. Every adaptive gain multiplies these, none replaces them.

| Field | Penalises | Shipped | Purpose |
|---|---|---|---|
| `q_e_y` | lateral deviation from the path | 6.4 | primary tracking term |
| `q_e_yd` | rate of change of lateral deviation | 0.0 | damps lateral oscillation, unused |
| `q_e_psi` | heading error against the path tangent | 1.65 | keeps the car pointed along the path |
| `q_r` | yaw rate (LTV-QP) or heading-error rate (NMPC) | 1.0 | damping |
| `q_e_v` | speed error, car speed minus target | 1.5 | tracks the speed profile |
| `r_delta` | steering command size | 1.8 | discourages large steering angles |
| `r_a_accel` | acceleration command size, `a_cmd >= 0` | 0.9 | discourages large throttle |
| `r_a_brake` | acceleration command size, `a_cmd < 0` | 0.6 | discourages large braking |
| `r_rate_delta` | steering rate of change | 100.0 | discourages jerky steering |
| `r_rate_a` | acceleration rate of change | 2.0 | discourages jerky throttle and brake |
| `terminal_q_scale` | final predicted state in the horizon | 1.0 | extra weight on where the plan ends |

Known constraints:

- `terminal_q_scale`: 1.0 (no-op) is the only value validated against the current weights. Changing it shifts the effective tuning of everything else.
- `r_a_accel` and `r_a_brake`: an acceleration command is a rate. Its effort cost is paid immediately and its benefit (removed speed error) arrives slowly. A weight that looks reasonable can leave most of the car's braking authority unused. Lower the relevant weight first when braking looks weak, and sweep around the current value, because the relationship has a measured local optimum and is not "lower is better". `r_a_brake` alone is the more targeted lever for weak braking. History of the two-weight split: [control_mechanisms.md](../reference/control_mechanisms.md).
- `R_diag[1]` (offline) and the tuner's `a_cmd` effort dimension do not change the acceleration effort cost. Both controllers take `R_A_ACCEL` and `R_A_BRAKE` instead. Confirmed by reading `sim/rollout/tick_solve.py`, not by a sweep.
- `q_e_v`: matters more in the corner-approach phase than in whole-run averages, since most of a lap is straight with small speed error. Compare corner-approach metrics, not just whole-run RMSE.
- `r_rate_delta` moved from 52.5 to 100.0 in a Q/R retune commit on 2026-09-20. Rationale not recorded. The measurements in the steering-smoothness section below were taken at 52.5 and have not been repeated at 100.0.

## Steering smoothness: chatter and turn-in

Chatter (the wheel twitching instead of holding an angle) and late or jerky turn-in at tight corners are two different faults. Diagnose which one is present before tuning, because each ignores the other's controls. Full history: [steering_chatter_investigation.md](../logs/steering_chatter_investigation.md).

### Chatter is a cost-function problem

Order of measured impact:

| Lever | Setting | Effect |
|---|---|---|
| Steering-rate cost | `r_rate_delta` 2.8 to 52.5 (100.0 today) | mean steering step per tick 2.54 to 1.92 deg live, sign-flip rate off its 65 percent floor. Largest single lever. |
| Input-jerk cost | `nmpc_rjerk_delta` 150.0 | on the centreline reference: 0 saturated ticks, 0 slew-limited ticks and 1 steering reversal over three laps, at `r_rate_delta=52.5` |
| Three-zone rate schedule | `nmpc_rrate_zone_*` with `nmpc_corner_factor_k=27` | smallest contributor, and only after `k` was raised from 8 |
| Post-solve output smoothing | removed | made redundant by the first two |

The `r_rate_delta=0.0` control run measured 4.72 deg per tick against 1.92 deg at 52.5, which confirms the diagnosis.

### Input-jerk cost (`nmpc_rjerk_delta`, `nmpc_rjerk_a`), NMPC only

**What it does.** The plain rate cost charges the same for a steady ramp into a corner as for one leg of an oscillation, so raising it to stop a wobble also makes the car reluctant to turn. The jerk term prices the change in the steering rate instead. A steady turn scores near zero and a wobble scores high.

**Why this design.** Measured on a live run of 2910 ticks, classified by whether steering reversed or continued:

| Behaviour | Ticks | mean \|d1\| (rate) | mean \|d2\| (acceleration) |
|---|---|---|---|
| reversals (chatter) | 1739 | 2.375 deg | 4.669 deg |
| same direction (ramp) | 1171 | 1.247 deg | 1.083 deg |

The second difference separates the two by 4.31 times against about 1.9 times for the first. 96 percent of slew-limited ticks were reversals, so even the large catch-up events are mostly the tail of an oscillation.

**How it works.** `E` is the first-difference operator already built for the rate cost. The jerk term uses `E2 = E @ E`:

```
Hess += E2' diag(rj) E2          rj = tile([rjerk_delta, rjerk_a], N)
grad += E2' (rj * e_jerk)
```

- The term is anchored to the two previous applied inputs. A second difference spanning the tick boundary needs `u_prev` and `u_prev2`, so `e_jerk[:NU] -= (2 u_prev - u_prev2)` and `e_jerk[NU:2NU] += u_prev`. Without this the term misses a reversal that straddles the boundary. The controller carries `_u_prev2`, updated before `_u_prev`.
- The OSQP sparsity pattern is unchanged. `p_mask[:n_du,:n_du]` is already a dense upper triangle, so `E2' R E2` adds no nonzeros and the term can be switched between runs.
- `_cost()` carries a matching `jerk` term. The SQP line search scores steps with `_cost()`, so a term in the Hessian but not in `_cost()` makes the search optimise a different objective from the one solved. The rate cost once had exactly this bug.
- Setting both weights to 0.0 removes the term from the QP entirely.

**Tuning and pitfalls.**

- Shipped 150.0 for `nmpc_rjerk_delta` and 0.0 for `nmpc_rjerk_a`. `nmpc_rjerk_a` has never been run at a nonzero value.
- `rjerk_delta=0` fails the recorded-map rollout offline where 150 completes. The tuner searches it for that reason (see the tuner section).
- Reach for this term when raising `r_rate_delta` starts to make the car reluctant to turn.
- An earlier live run showed about 4.5 percent steering saturation at `rjerk=150`, first read as the jerk term trading smoothness for saturation. That was the raceline reference. The same weight on the centreline saturates 0.00 percent. See [reference_path_and_speed.md](../reference/reference_path_and_speed.md).
- A pairing of `r_rate_delta=5.0` with `nmpc_rjerk_delta=250.0` beat the flat 52.5 baseline on every offline metric in an earlier session. The source log for that number was not found, and it has not been run live or against the current weights. Not verified.

### Three-zone rate schedule (`nmpc_rrate_zone_*`) and why `k` gates it

**What it does.** The steering-rate price slides continuously between three levels: high on a straight so the car holds still, lower on the approach to a corner so it is willing to start turning, lowest mid-corner.

**How it works.** A multiplier on the NMPC steering-rate cost, driven by current curvature and the peak curvature the horizon predicts ahead:

| Zone | Condition | Multiplier (shipped) |
|---|---|---|
| straight | nothing now, nothing ahead | `nmpc_rrate_zone_boost_straight` 2.0 |
| approach | nothing now, corner ahead | `nmpc_rrate_zone_ease_approach` 0.80 |
| corner | turning now | `nmpc_rrate_zone_floor_corner` 0.15 |

- It multiplies `r_rate_delta`, so it composes with the tuned value. `nmpc_corner_rrate_blend_enabled` overwrites `R_rate[0,0]` and discards it.
- Both the "now" and "ahead" signals pass through `corner_factor(|kappa|, k) = 1 - 1/(1 + k |kappa|)`. The floor is only reached as that approaches 1, so the schedule is only as strong as `k` allows over the track's own curvature range.

| `\|kappa\|` (1/m) | `corner_factor` at k=8 | at k=27 |
|---|---|---|
| 0.00 | 0.000 | 0.000 |
| 0.06 | 0.390 | 0.618 |
| 0.209 (tightest corner on `comp_test_map_3`) | 0.626 | 0.850 |
| 1.125 | 0.900 | 0.968 |

- Size `k` from the track: `k ≈ target / ((1 - target) * kappa_max)`.
- At the inherited `k=8` the zone never left its boost band. Measured live: `m_Rrate_zone` ranged 0.829 to 1.962 with 0 percent of ticks in the ease or floor band, and the score was a wash against the zone-off baseline (0.522 against 0.488).
- Read the `m_Rrate_zone` telemetry column before believing any A/B of the endpoints. If it never nears `floor_corner`, the endpoints were not what was tested.
- `nmpc_corner_factor_k` is shared with `nmpc_corner_rrate_blend_enabled`. Raising it sharpens that blend too.
- `corner_frac` in telemetry and the node-level output smoothing use `params.corner_factor_k` (the LTV-QP field), not the NMPC override, so raising `nmpc_corner_factor_k` does not change smoothing.
- `k` past 27 does not help. `k=60` measured worse live.

**Open: the intended `ease_approach=0.35` fails offline.** With `k=27` and 0.35 the offline rollout leaves the track at the tightest corner (x about 41.4, y about 49.6) with full-lock steering and `|e_y|` growing to 2.53 m. Explanations tested and rejected:

- Too much release: raising `floor_corner` to 0.5 or 0.7 still fails.
- Not monotonic: `boost_straight=0.8`, which makes the multiplier at most 1 everywhere (weaker than no zone), also fails.
- Speed profile: matching the harness `v_max` to the centreline's 16.70 changes nothing.
- `k`: with the zone disabled, `k` of 15, 27 and 40 are identical to baseline.
- Weight compounding: `_Rr_flat` is rebuilt each tick, so scaling does not accumulate.

The failing runs carry high non-`solved` SQP rates (42 percent at `boost_straight=0.8` against 14 percent at baseline). The leading hypothesis is that rescaling `_Rr_flat` late in the tick interacts with the warm start or line search. Not confirmed. `ease_approach` ships at 0.80, where the offline rollout completes (score 0.796 against 0.822 baseline, `|e_y|` 0.476 against 0.496). The 0.35 turn-in release is untested live and offline. These numbers are from an earlier configuration and were not re-run for this rewrite.

The sim and the car disagree here. The same zone at `k=8` and 0.35 completed two clean laps live while failing offline. An offline failure is not automatically a live failure.

Check `python -m tuner.investigations.steering_chatter_check` before shipping a zone or `k` change. The `k=27` and 0.35 pair was chosen from saturation arithmetic alone and would have reached the car as an offline failure.

### Tight-corner jerks are a speed problem

If the car turns in late, runs wide and the steering slams over at the tightest corners, check the speed before touching a steering weight.

- `CURVATURE_SPEED_A_LAT_MAX` 5.5 to 4.75 cut stutters from 33.3 to 9.8 per minute and `|d_steer| > 5 deg` events from 20 to 1.
- Four steering-side weights (`r_rate_delta`, `NMPC_CORNER_FACTOR_K`, `NMPC_Q_E_Y`, `NMPC_SQP_ITERS`) were tried first and none moved it. The steering command was already arriving early at about 94 percent of the needed angle. The car reached the hardest curvature ramp needing about 15 m/s^2 against a plant ceiling of about 7.5.
- The constant lives in `sim/speed_profile.py` (4.75 shipped) and only affects the exported `speed_profile.csv`. See [reference_path_and_speed.md](../reference/reference_path_and_speed.md).

### Turn-in timing: the command leads, and six levers do not move it

When the car seems to turn in late, the steering command is not late. Measured live on `centerline.csv` over 3 runs (correlation 0.93): the command leads the geometrically required angle (`atan(L * kappa)` at the car's station) by 0.15 s and delivers 0.936 of it in corners. `delta_cmd` and applied `steer_deg` match to 0.0005 deg. A late-turn-in report is about what happens after the command.

Six candidate causes were tested and falsified (evidence in the chatter log):

| Candidate | Result |
|---|---|
| `r_rate_delta` | 52.5 is the best value tried, lower runs wider and fails |
| `nmpc_corner_factor_k` past 27 | `k=60` measurably worse live |
| `nmpc_q_e_y` | 7.5 does not change the drift rate |
| `NMPC_SQP_ITERS` | slew-limited fraction flat across 1, 2, 3 |
| speed-profile braking feasibility | 0.00 percent of stations exceed -7.0 m/s^2 |
| delay compensation | same lag in the high and low latency halves of one run |
| `alat_ceiling` model | conservative, not optimistic, in the 6 to 14 m/s band |

Invariant: across every configuration tried on this track (`r_rate` 5 to 52.5, `k` 8 to 60, `q_e_y` 6.35 to 7.5) the rate-normalised count of sustained lateral-error-growth episodes stays near 31.5 per minute. A cost-weight change moves episode size, not episode rate. Divide raw counts by run duration: the same configuration gave 29 episodes in 55 s and 48 in 90 s.

Two metric traps produced a plausible wrong conclusion before being caught:

- Lap-wide ratios are dominated by straights. "Plant yaw gain" (achieved yaw rate over `v/L * tan(delta)`) reads 0.42 at p10, which looks like understeer. Binned by speed it is an artefact of near-zero denominators, and it rises with `a_lat`, the opposite of a grip limit.
- The time derivative of the speed target along a rollout is not the profile's gradient. It read -26.9 m/s^2 where the exported file's spatial gradient is -5.03. Read feasibility off the exported file.

### Outcome of the turn-in option study

The `r_rate_delta` increase fixed chatter and introduced a new fault on shallow corners: the car holds a smooth line, refuses to turn, then jerks once predicted errors overpower the rate cost. Every jerk landed at exactly 9.00 deg per tick, the actuator slew limit (`du_max`, 180 deg/s times 0.05 s). The controller deferred until it had to catch up faster than the plant allows.

**Why a flat weight cannot fix it.** The rate cost scales with step size and the tracking cost with error squared, so their ratio swings with corner severity:

| Corner | `e_y` | `e_psi` | tracking cost | rate cost of a 1 deg step | ratio |
|---|---|---|---|---|---|
| shallow | 0.05 m | 1.0 deg | 0.328 | 0.016 | 20 times |
| moderate | 0.20 m | 4.0 deg | 5.241 | 0.016 | 328 times |
| sharp | 0.50 m | 12.0 deg | 33.198 | 0.016 | 2076 times |

On a sharp corner tracking overwhelms the rate cost at once. On a shallow one the two are the same order, so waiting is the optimal plan. The QP behaves correctly and the weighting is wrong. Live baseline at the time (`r_rate_delta=52.5`): slew-limited ticks 1.75 percent against 0.00 percent at 2.8, `max|e_psi|` 21.7 to 29.4 deg, and `mean|d_steer|` 0.833 deg per tick at `corner_frac < 0.1` against 2.482 deg at 0.25 to 0.50.

| Option | Result | Status |
|---|---|---|
| Per-stage rate ramp (`nmpc_rrate_stage_ramp_enabled`, `nmpc_rrate_stage_near`): cheap steering rate at near horizon stages, full price late | Offline (`comp_test_map_3`, `a_lat_max` 5.5): flat 8.43 percent slew-limited (and failed), ramp near=0.40 12.38 percent, near=0.30 15.42 percent. Live at near=0.30: slew 1.75 to 5.10 percent, `mean\|d_steer\|` 1.919 to 2.804 deg, `\|e_y\|` 0.288 to 0.434 m, `max\|e_psi\|` 29.4 to 42.3 deg, saturation 0.03 to 0.27 percent. | Rejected offline and live. Kept in code, default off, because it cleared the offline failure at the time. |
| Curvature-scheduled rate (`nmpc_corner_rrate_blend_enabled`, `k=20`, straight 52.5, corner 8.0) | Live, schedule delivered as designed (applied `Rrate_steer` min 16.5, p50 29.2, max 52.5): slew 1.75 to 2.54 percent, `\|e_y\|` 0.288 to 0.467 m, `max\|e_psi\|` 29.4 to 36.8 deg, saturation 0.03 to 1.52 percent. Worse on every metric. | Rejected. Do not retry. |
| Lookahead-curvature scheduling (same idea keyed on peak curvature ahead) | Not built. One second before each of 51 slew-limited jerks, median `corner_frac` was 0.360 and median `\|e_psi\|` 3.42 deg. 14 of 51 (27 percent) had `corner_frac < 0.15` and `\|e_psi\| < 3` deg, indistinguishable from a straight. `nmpc_kappa_horizon_end` one second before a jerk was 0.0908 against 0.0569 overall. | Deprioritised. The horizon holds the information, state-keyed signals do not. The three-zone schedule above is the later horizon-keyed variant. |
| Second-difference (jerk) penalty | Basis is the 4.31 times separation table above. | Chosen, shipped as `nmpc_rjerk_delta` 150.0. |
| Raise `du_max` | Treats the symptom. 180 deg/s is a measured lower-bound estimate of the real actuator, and a higher value commands motion the plant cannot deliver. | Rejected. Revisit only with a fresh system-ID. See [control_mechanisms.md](../reference/control_mechanisms.md). |
| Lower `r_rate_delta` and suppress chatter another way | Re-opens a solved problem. | Kept as a retreat, not taken. |

The two rejected schedules attacked the problem from opposite directions (soften by horizon position, soften by track position) and both traded chatter for compliance at about 1 to 1. Any weakening that lets turn-in happen early also lets oscillation happen. The fix changed what is penalised, not when.

Two caveats from the study. The offline closed-loop harness once failed on shipped defaults because of `a_lat_max=5.5`, and a later run showed the 4.75 speed-profile value clears it, so the "live A/B is authoritative" caveat from that period is dated. The per-stage ramp was the first result where offline and live agreed. Earlier offline mispredictions were all on the `a_lat_max` by `r_rate_delta` interaction.

## Delay compensation

| Field | Purpose | Shipped |
|---|---|---|
| `delay_compensation_enabled` | roll the controller's state estimate forward through pending commands before optimising, so the plan accounts for actuation lag | on |
| `max_delay_compensation_steps` | cap on the rollforward depth | 3 |
| `predict_epsi_clip` | small-angle bound on heading error inside `predict_ahead()` (LTV-QP only) | 0.5 rad |
| `pose_age_lp_alpha` | low-pass coefficient smoothing the estimated pose age per tick | 0.15 |
| `n_delay_hysteresis` | deadband either side of an `n_delay` bin boundary, so depth does not flip-flop | 0.25 steps |

Offline-only settings model the lag itself and how wrong the controller's belief about it may be:

- `DELAY_STEPS` (1), `DELAY_JITTER_STEPS` (0.2), `DELAY_JITTER_SEED` (12345) in `settings/general.py`. With zero jitter the tuner solves an easier problem than the car, whose loop period jitters.
- Set `DELAY_STEPS` from a realistic hardware lag and `DELAY_JITTER_STEPS` from a measured live loop-period standard deviation converted to steps. Leave jitter above 0 unless deliberately testing the ideal case.
- `n_delay_hysteresis` prevents oscillation at a bin boundary. Do not remove it without confirming the oscillation does not return.
- A stalled pose feed inflates `pose_age_s` and pins `n_delay` at its cap. See [periodic_pose_teleport_investigation.md](../logs/periodic_pose_teleport_investigation.md).

## LTV-QP adaptive gains

Each adaptive gain is an enable flag plus shape constants. Shape constants only matter while the flag is on and the LTV-QP is running. All of this is inert under NMPC. The lookahead family that used to precede these (approach and exit boosts, demand normalisation, U-turn detector, curvature forcing, the precomputed corner map) is gone. See [retired_mechanisms.md](../reference/retired_mechanisms.md) for why.

### Adaptive Q scaling (`adaptive_q_scaling_enabled`)

Relaxes `Q[0,0]` (lateral error) when the car is near the centreline, to reduce small-error hunting. Scale is 0.5 below `|e_y|` of 0.05 m, rising linearly to 1.0 at 0.3 m. Dataclass and offline default on. The launch file overrides it to off, so the car does not run it under the LTV-QP. The offline recorded-map rollout does not reproduce the live symptom it was built for (steering reversals rise with `|e_y|` offline, the opposite of live), so treat offline validation of this mechanism with caution. Mechanism: [control_mechanisms.md](../reference/control_mechanisms.md).

### Steering-rate anti-hunt (`steer_rate_anti_hunt_enabled`, `anti_hunt_boost_max`)

Extra penalty on `R_rate[0,0]` when the car is centred, not curving and well aligned. Three saturating factors multiply: `1/(1 + 30|kappa|)`, `1/(1 + 15|e_y|)`, `1/(1 + 11.5|e_psi|)`. The boost is `1 + (boost_max - 1)` times that product, with `boost_max=6.0`.

- Half-fade points: `|kappa|` 0.033 (a 30 m radius), `|e_y|` 6.7 cm, `|e_psi|` 5.0 deg. The constants were halved from 60 / 30 / 23 because the boost faded out too fast on gentle curves, leaving jitter under-damped.
- It reads only current curvature, `e_y` and `e_psi`, so it cannot anticipate a corner.
- Default on, and the launch file leaves it on. Not validated against `VALIDATION_SUITE`, the recorded map or any live log as a whole mechanism. Treat as experimental.

### Reversal penalty (`reversal_penalty_enabled`, `_boost_max`, `_k`)

Boosts `R_rate[0,0]` when last tick's steering was near zero, the state every sign flip must pass through. Defaults 4.0 and 8.0 (half-boost at about 7.2 deg of previous steering). Dataclass default off, launch file on. Experimental and not validated live. Keyed on `u_prev`, not the current decision, so the cost stays convex. On the NMPC path an offline A/B was a net regression (reversal count barely improved, composite worse), so `nmpc_reversal_penalty_enabled` stays off.

### Speed-dependent steering effort

`R[0,0]` is multiplied by `1 + 1.5 vx / (6 + vx)`, a saturating ramp reaching 2.5 at high speed. The acceleration effort multiplier is fixed at 1.0: a speed-dependent one made `R[1,1]` rise with speed exactly where corner-entry braking needs to be strongest. Not a tunable field.

### Corner-factor scheduler

Replaces the retired lookahead family with one continuous fraction, `corner_frac` (0 straight, 1 full corner), from current curvature only. It blends four weights between a straight and a corner endpoint, adds a low-speed boost and applies an always-on heading-error accel and brake asymmetry. Formulas: [control_mechanisms.md](../reference/control_mechanisms.md).

| Field | Purpose | Shipped |
|---|---|---|
| `corner_factor_k` | sharpness of `corner_factor` against current `\|kappa\|` | 8.0 |
| `q_ey_straight` / `q_ey_corner` | `Q[0,0]` endpoints | 4.5 / 9.0 |
| `q_epsi_straight` / `q_epsi_corner` | `Q[2,2]` endpoints | 1.5 / 3.0 |
| `q_r_straight` / `q_r_corner` | `Q[3,3]` endpoints, corner lower (relaxes in corner) | 1.0 / 0.5 |
| `rrate_steer_straight` / `rrate_steer_corner` | `R_rate[0,0]` endpoints, corner lower | 2.0 / 1.25 |
| `r_steer_corner_mid` | `R[0,0]` at full corner, a middle value so turn-in is not made cheapest where saturation risk peaks | 1.35 |
| `low_speed_corner_boost_v_half` | speed at which the low-speed boost has halved | 4.0 m/s |
| `low_speed_corner_boost_max_extra` | extra `corner_frac` at zero speed, gated on `corner_factor` so it is a no-op on a straight | 0.3 |
| `epsi_ra_half_rad` | `\|e_psi\|` at which the asymmetry is half strength | 0.1745 rad (10 deg) |
| `epsi_ra_accel_boost_max` | max multiplier on `r_a_accel` at large `\|e_psi\|` | 2.0 |
| `epsi_ra_brake_floor` | min multiplier on `r_a_brake` at large `\|e_psi\|` | 0.5 |

The `rrate_steer_*` endpoints (2.0 and 1.25) are on a much lower scale than `r_rate_delta` (100.0). Enabling a blend that overwrites `R_rate[0,0]` with them discards the tuned rate weight. The blend is not validated against `VALIDATION_SUITE`, the recorded map or a live log as a whole. The `epsi_ra_*` asymmetry is independent of `corner_frac`. Tune it separately.

## Reference-heading rate limit

`ref_heading_rate_limit_enabled` (default off, LTV-QP only) caps how fast the tracked reference heading may change per tick, at `ref_heading_rise_rate_deg_s` (90.0 deg/s). Offline names: `REF_HEADING_RATE_LIMIT_ENABLED`, `REF_HEADING_RISE_RATE`.

- Motivation: the planner's reference heading can swing faster than either car can yaw. See [sim_to_real_investigation.md](../logs/sim_to_real_investigation.md), sections 12.8 and 26.
- Live testing found it makes saturation and heading error worse. It stays in the code as a validated-off feature. Do not re-enable without a specific reason to re-test.
- A rise rate much below about 85 deg/s risks holding the reference back so hard that a fast slalom leaves the track.
- To re-test, run `python -m tuner.investigations.ref_heading_limiter_suite_check` first.

## Precomputed heading-lead profile (`use_precomputed_heading_profile`)

A live node parameter, not an `MPCParams` field. It uses the `psi_target` column of `raceline.csv` as the `e_psi` reference in place of the geometric path tangent, so `e_psi` carries a current error approaching a corner. Default false, and the launch file ships `USE_PRECOMPUTED_HEADING_PROFILE=false`. It only acts when `use_precomputed_path` is true and the CSV has the `psi_target` column. It is ignored under NMPC, whose curvature model already carries what the profile approximates.

- `HEADING_LEAD_AUTHORITY_FRAC` (0.5) in `tuner/tools/raceline_optimizer.py`: fraction of achievable yaw rate pre-spent as lead. Baked into the CSV at export, so re-export to change it.
- `SLIP_LIMIT_RAD` (5 deg) in the same file: diagnostic bound used to flag stations, an unvalidated placeholder.
- Status: offline no-op-when-off confirmed. Live, four runs at the default 0.5 ranged from the best single run recorded (zero saturation) to some of the worst, against a baseline that varied nearly as much. Inconclusive.
- `comp_test_map_3` has few true straights, so the lead is active almost everywhere, a plausible but unconfirmed explanation for the variance.
- Design and run data: [control_mechanisms.md](../reference/control_mechanisms.md) and [late_turn_in_investigation.md](../logs/late_turn_in_investigation.md), parts 7 to 13.

## NMPC tuning surface

The NMPC reads the base weights above, so the tuned set is the starting point. Everything in the LTV-QP adaptive-gain section is inactive under NMPC. Tuning it has no effect on an NMPC run.

- **`q_r` changes meaning.** In the LTV-QP it weights absolute yaw rate. In the NMPC it weights heading-error rate `r - kappa * s_dot`, zero for a car tracking a corner correctly. Same number, different regressor. Re-sweep it first.
- **Overrides.** The `nmpc_q_*` and `nmpc_r_*` fields live in `MPCParams` (`nmpc_q_epsi_dot` overrides `q_r`). A value of `-1.0` inherits the base weight. Offline names are `NMPC_Q_E_Y` and so on. `NMPCParams` holds only structural, solver and feature fields. All overrides ship at `-1.0`. A `NMPC_Q_E_Y=7.5` line in `launch_all.sh` is commented out. It measured score-neutral live (0.4538 against 0.4542) with lower peak lateral error (1.023 against 1.116 m), and is not the shipped value.
- **`nmpc_alat_ceiling_enabled` (true) is required on FSDS.** With it off the prediction believes it can hold any corner at any speed and the car spins offline. Set false only for real-vehicle work.
- **NMPC-only rate-shaping and experiments.** `nmpc_steer_rate_anti_hunt_enabled`, `nmpc_reversal_penalty_enabled`, `nmpc_corner_rrate_blend_enabled` and `nmpc_rrate_stage_ramp_enabled` all default off. Do not enable one for a live run without an offline A/B.
- **`nmpc_corner_rrate_blend_enabled` trap.** It overwrites `R_rate[steer]`. If its two endpoints stay at `-1` they inherit the LTV-QP's 2.0 and 1.25, a 30 times cut from 52.5 that produced 21 percent saturation. Always set both endpoints explicitly, scaled to the current `r_rate_delta`.
- **MPCC-inspired flags.** `nmpc_spline_reference_enabled` (default true) is a numerical-quality fix, set false only to compare against the old moving-average reference. `nmpc_friction_circle_enabled` (default false) was rejected live with no offline A/B first. `nmpc_progress_enabled` (default false) failed to complete a lap at every setting tried and stays off. A per-stage horizon speed profile flag existed, was rejected live and was removed. See [control_mechanisms.md](../reference/control_mechanisms.md) and [nmpc_progress_term_investigation.md](../logs/nmpc_progress_term_investigation.md).
- **`nmpc_track_halfwidth` (3.35) and `nmpc_slack_weight` (10000).** The soft track bound. Narrowing to 3.0 for the progress experiment also tightened ordinary tracking and measurably hurt it, and was reverted. `nmpc_slack_linear_weight` ships at 500.0 and is inert while the progress term is off.
- **`nmpc_curvature_dense_step` (0.5) and `nmpc_curvature_smooth_w` (3)** copy the denoising precedent of `curvature_speed()`. Not new constants to tune.

### Structural settings and where their values came from

Live source is `nmpc_params.py`, with the reasons in its field comments. Offline names are `NMPC_*` in `settings/nmpc.py`.

| Field | Shipped | Reason |
|---|---|---|
| `nmpc_horizon` | 20 (1.0 s) | measured better than 35: the prediction model is optimistic and the mismatch compounds. N=35 gave the fastest lap and the worst tracking. |
| `nmpc_sqp_iters` | 1 | measured better and about 2 times cheaper than 2. Real-time-iteration style: the shifted previous solution is the warm start. |
| `nmpc_solve_budget_ms` | 25 | hard stop, ships the best feasible iterate rather than overrunning the 50 ms tick |
| `nmpc_rk_substeps` | 4 | 2 was RK4-unstable across 2.25 to 3.5 m/s (about 6e8 disturbance growth), which freezes the output at exactly zero |
| `nmpc_rk_substeps_fast`, `nmpc_rk_gate_speed` | 3 above 4.0 m/s | 3 is fully stable at and above the gate. 2 is the one count confirmed unstable in the 2.25 to 3.75 m/s band. |
| `nmpc_jac_substeps` | 4 | 1 was sound about accuracy but divergent below about 6.5 to 7 m/s, because the same 1/`v_x` lateral stiffness compounds through condensing into a Hessian whose only solution is zero. Keep equal to `nmpc_rk_substeps`. |
| `nmpc_jac_substeps_fast`, `nmpc_jac_gate_speed` | 2 above 8.0 m/s | the instability is confined to low speed, so the cost of 4 substeps need not apply above it. 1 is not safe: inaccurate rather than unstable. |
| `nmpc_standstill_*` | damping on, `r_delta` scale 200 on stage 0 | at `v_x=0` steering cannot move the car, and the SQP pre-commits stage 0 toward what helps later stages, so the car launched already turned (about -6.8 deg). The 200 scale is a tuning value, not derived. |

## Speed-side settings

| Setting | Shipped | Notes |
|---|---|---|
| `speed_target_deficit_max` | 2.55 m/s | caps how far the ramped speed target may lead the car's speed, read by both controllers before the speed-error row sees the target |
| `CURVATURE_SPEED_A_LAT_MAX` | 4.75 | corner speed in the exported profile. See the tight-corner section above. |
| `enable_dynamic_speed_cap` | on in code, off at launch | see below |

`speed_target_deficit_max` was 2.5 and binding on acceleration for 36.8 percent of a lap. A raise to 5.0 improved lap time, `|e_y|` and saturation offline only. It ships at 2.55. Rationale for 2.55 not recorded. It has not been re-measured at that value.

**Dynamic speed cap.** `enable_dynamic_speed_cap`, `dynamic_cap_a_lat_max` (3.2) and `dynamic_cap_safety` (0.9) layer a curvature-lookahead speed cap under the precomputed profile. They are node parameters in `mpc_controller.py` and keys in `fsae_params.yaml`, not `MPCParams` fields. The offline constants are `ENABLE_DYNAMIC_SPEED_CAP` (True), `DYNAMIC_CAP_A_LAT_MAX` and `DYNAMIC_CAP_SAFETY` in `settings/general.py`. When built and live-tested it improved the lateral-acceleration-over-ceiling ratio but worsened steering saturation and heading error, and the score regressed. The cause was never diagnosed. `launch_all.sh` sets it false, so check that file before assuming the code default drives a run. Do not re-enable it without first finding out why it regressed. Writeup: the dynamic speed cap addendum in [late_turn_in_investigation.md](../logs/late_turn_in_investigation.md).

## What the offline tuner searches

`tuner/offline_tuner.py` runs CMA-ES (a derivative-free evolutionary optimiser) over 14 parameters. Usage: [offline_guide.md](offline_guide.md).

| Block | Count | Form | Fields |
|---|---|---|---|
| `Q_diag` | 5 | multiplier on the template | `e_y`, `e_y_dot`, `e_psi`, `e_psi_dot`, `e_v` |
| `R_diag` | 2 | multiplier | `delta_cmd`, `a_cmd` |
| `R_rate_diag` | 2 | multiplier | steering rate, acceleration rate |
| `TUNABLE_NMPC` | 5 | absolute value | `rjerk_delta`, `corner_factor_k`, `rrate_zone_boost_straight`, `rrate_zone_ease_approach`, `rrate_zone_floor_corner` |

- The `R_diag` `a_cmd` dimension does not move the cost (see core weights above), so the effective search is 13 parameters. Not confirmed by a run.
- The NMPC block is absolute because those fields have no template to scale and their useful ranges are not centred on 1.0 (`rjerk_delta` spans 1 to 400). Generation 0 starts from the shipped `settings/` values, so the first thing evaluated is the current car.
- The NMPC block only acts when `settings.USE_NMPC` is True, which defaults to False. Under the LTV-QP all five fields are ignored, every candidate scores the same in those dimensions and the population is wasted. Either set `USE_NMPC=True` before importing the tuner or empty `TUNABLE_NMPC`.
- These are searched rather than held fixed because `rjerk_delta=0` fails the recorded-map rollout where 150 completes. A weight set tuned with them pinned is only valid at those values.
- `MAX_EVALS` is 1500 and `USE_OPTUNA_PRESEARCH` is True in `settings/solver.py`.
- `run_core_rollout` takes an `nmpc_overrides` dict keyed by the controller's own argument names (`rjerk_delta`, `rrate_zone_ease_approach`, and so on). This lets one process evaluate many configurations. Keys are not validated, so a mistyped name is silently ignored. Confirm a swept field moves the score before trusting a null result.
- `sim/rollout/tick_solve.py` reads `settings.NMPC_*` at call time, so a runtime `setattr` on `settings` reaches the NMPC. Default arguments of `run_core_rollout` (`use_nmpc`, `n_horizon`, `use_planner`) bind at import, which is why `USE_NMPC` must be set before the import.

Smoke-test the wiring after editing `TUNABLE_NMPC`. A full run is not needed:

```python
# with USE_NMPC=True set before importing the tuner
score = run_headless_rollout(x0, "PATH_SUDDEN_TURN", 400, 0.0, 0.0)
# then move one field to its far bound and confirm the score changes
```

## Scoring: `METRIC_SCALES` and `SCORE_WEIGHTS`

`sim/scoring.py` is the single source of the composite score. The live copy at `telemetry/scoring.py` under `fsae_control` is a copy with the constants inlined. Change the offline file first, then recopy. See [offline_live_parity.md](../reference/offline_live_parity.md).

The score has three tiers:

- Runs that crash, leave the track or never finish land above `CONSTRAINT_FLOOR` (10.0). No amount of good driving lifts an infeasible run above a feasible one.
- Feasible runs score `TIME_OBJECTIVE_WEIGHT * time_cost + QUALITY_WEIGHT * quality` (1.0 and 0.35), where `time_cost = 1 - optimal_time / actual_time`.
- `quality` is the weighted sum below. It shapes a solution and cannot buy lap time.

Metric sets:

- **`METRIC_SCALES`**: a typical magnitude for each of the 13 metrics, dividing each onto a comparable scale before weighting. Without it a metric's influence is weight times typical magnitude, which collapsed the score to tracking RMSE and peak lateral error. Change an entry only when that metric's typical magnitude has shifted (after a plant or planner change), not to change its priority.
- **`SCORE_WEIGHTS`**: the priority of each normalised metric. Lower score is better. The 13 weights sum to 1.0, so `quality` is 1.0 for a run sitting at every reference scale. Change a weight by 20 to 30 percent of its value at a time and take the offsetting change from another to keep the sum.

The 13 metrics in order: `rmse`, `yaw_rms`, `smooth_rms`, `steer_rms`, `accel_rms`, `max_steering`, `steering_sat_ratio`, `jerk_rms`, `max_yaw_rate`, `steering_reversal_rms`, `peak_lateral_error`, `speed_rmse`, `accel_reversal_rms`. The two reversal metrics are magnitude-weighted so hunting is distinguished from a path that legitimately demands frequent small direction changes. Definitions: the `sim/scoring.py` module and [architecture.md](../reference/architecture.md).

## Simulator-only fidelity settings

These are not MPC weights, but they decide whether an offline result transfers to the car.

| Setting | Shipped | Guidance |
|---|---|---|
| `N_HORIZON` | 35 (1.75 s) | LTV-QP horizon. Must match the live `MPCController(dt, N=35)` or tuned weights behave differently. The NMPC has its own `NMPC_HORIZON` of 20. |
| `DELAY_STEPS` | 1 | fixed actuation lag in 0.05 s steps. 0 is the idealised case. |
| `DELAY_JITTER_STEPS` | 0.2 | spread of the controller's belief about the lag. Set from measured loop jitter. Zero makes the tuner solve an easier problem than reality. |
| `SLAM_NOISE_ENABLED`, `SLAM_POS_JITTER_STD`, `SLAM_YAW_JITTER_STD` | off, 0.02 m, 0.3 deg | simulates SLAM pose jitter instead of FSDS ground truth. Recalibrate against a current live log's reversal rate before turning on. Pose noise does not reproduce chatter caused by the steering slew limit or delay-estimate jitter. |

## Related documents

- [architecture.md](../reference/architecture.md): system structure and the tuner's internals.
- [offline_guide.md](offline_guide.md): running the tuner and the 2D GUI.
- [offline_live_parity.md](../reference/offline_live_parity.md): field-by-field mapping between `settings/` and `MPCParams`, and the resync procedure.
- [control_mechanisms.md](../reference/control_mechanisms.md): mechanism design for each adaptive gain and the NMPC.
- [retired_mechanisms.md](../reference/retired_mechanisms.md): removed mechanisms and why.
- [sim_to_real_investigation.md](../logs/sim_to_real_investigation.md): the history behind several constraints above. Read it for how a constraint was found. This guide states what the constraint is.
- [getting_started.md](getting_started.md): a standalone onboarding page that explains tuning from scratch.
