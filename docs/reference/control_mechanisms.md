# Control Mechanisms Reference

Per-mechanism reference for the control stack: what each one does, why it is shaped the way it is, and what to be careful of when changing it.

**Scope.** This covers mechanisms that exist in the code today. For:

- which knob to turn and to what value → `docs/tuning.md`
- how a subsystem is built → `docs/architecture.md`
- a mechanism that no longer exists → `docs/removed_mechanisms.md`
- the reference path and speed profile → `docs/reference/reference_path_and_speed.md`

**What "mechanism" means here.** A mechanism is an extra layer bolted on top of a controller's core solve loop, enabled or tuned independently, addressing a specific known failure mode (steering chatter, late turn-in, sensor/actuation lag) rather than being part of the base prediction-and-solve loop itself. A mechanism can be disabled entirely and the controller still drives, just without whatever failure mode it was added to fix.

This does **not** include the NMPC itself. `use_nmpc` does not add a feature to "the MPC", it swaps the entire core solve loop for a different one (LTV-QP's one convex QP per tick vs. NMPC's real-time-iteration SQP, see `architecture.md`); neither is optional on top of the other; it is which controller a given feature belongs to. The ["Nonlinear MPC (`use_nmpc`)"](#nonlinear-mpc-use_nmpc-a-second-controller) section below covers the two controllers themselves for that reason; every other section is a mechanism in the sense above, and can be LTV-QP-only, NMPC-only, or shared depending on which controller's blind spot it addresses.

## Corner-factor scheduler: what replaced the lookahead gain-scheduling family

**Plain version:** the controller drives differently depending on whether it is currently on a straight or currently in a corner, smoothly blending its weights between the two based on how sharply the road is curving right now, not based on a forecast of what's coming up ahead.

**The current mechanism** is `_corner_factor`/`_low_speed_corner_boost`/ `_blend`: a single continuous CURRENT-curvature-only fraction blending four `Q`/`R_rate` weights between a straight endpoint and a corner endpoint, plus an always-on, independent heading-error-driven accel/brake asymmetry (`epsi_ra_*`). See `architecture.md`'s "Corner-factor scheduler" section for the full formulas and mechanism, and `tuning.md` §4.3b for the tuning-surface reference, not repeated here to avoid duplicating either.

It exists on both sides: `fsae_MPCTest/controller/model_utils.py` (`_corner_factor`/`_low_speed_corner_boost`/`_blend` plus `settings.py`'s matching constants) and `fsds_simulator/`'s copies of `mpc_core.py`/ `mpc_params.py`/`fsae_params.yaml`, byte-identical to the live files (see `offline_live_parity.md`'s "MPC weight/gain parity" table).

### Detection

Current path curvature `kappa` only, no forward scan. `kappa` is the same ~1 m-preview curvature estimate the caller already computes once per tick (`curvature_estimate`/`_curvature`), reused by `steer_rate_anti_hunt` too.

`corner_frac` (the value actually driving every blend below) composes two saturating terms:

```
corner_factor = 1 - 1 / (1 + k * |kappa|)                      # k = MPCParams.corner_factor_k
low_speed_boost = _low_speed_corner_boost(v, corner_factor, v_half, max_extra)   # gated on corner_factor > 0
corner_frac = clip(corner_factor + low_speed_boost, 0.0, 1.0)
```

`low_speed_boost` is multiplicatively gated on `corner_factor > 0`, so it is an exact no-op on a straight regardless of speed (unlike the removed `low_speed_steer_rate_boost`, which fired on speed alone).

### Penalty/effect

Four weights are linearly blended (`_blend(straight_val, corner_val, corner_frac) = straight_val + (corner_val - straight_val) * corner_frac`) between a straight-line endpoint and a corner endpoint:

| Weight | Straight endpoint | Corner endpoint |
|---|---|---|
| `Q[0,0]` (e_y) | `q_ey_straight` | `q_ey_corner` |
| `Q[2,2]` (e_psi) | `q_epsi_straight` | `q_epsi_corner` |
| `Q[3,3]` (yaw rate) | `q_r_straight` | `q_r_corner` |
| `R_rate[0,0]` (steer rate) | `rrate_steer_straight` | `rrate_steer_corner` |

`R[0,0]` (steering effort) is blended toward a middle value instead of a floor/ceiling extreme: `R_scaled[0,0] = _blend(R_scaled[0,0], r_steer_corner_mid, corner_frac)`, so it never reaches as far in either direction as the other four.

Independently of `corner_frac`, an always-on heading-error asymmetry scales the accel/brake effort weights by current `|e_psi|` (`x0[2]`):

```
frac_epsi = |e_psi| / (|e_psi| + epsi_ra_half_rad)
r_a_accel_eff = r_a_accel * (1 + (epsi_ra_accel_boost_max - 1) * frac_epsi)
r_a_brake_eff = r_a_brake * (1 - (1 - epsi_ra_brake_floor) * frac_epsi)
```

This does not touch `_adaptive_R_scaling`'s speed-based `R[0,0]` scaling, which is a separate, unrelated multiplier applied earlier.

### Integration

Both controllers. `corner_frac` and the four `_blend` calls are shared code (`mpc_core._corner_factor`/`_blend`, imported verbatim by `nmpc_core.py`); the NMPC path additionally gates the `R_rate[0,0]` corner blend behind `nmpc_corner_rrate_blend_enabled` and, when both are configured, it takes priority over `nmpc_steer_rate_anti_hunt_enabled` rather than composing with it (use one or the other, not both, on the NMPC side).

Enters as a cost-term reweight (`Q`/`R`/`R_rate` diagonal entries handed to the QP/SQP each tick), not a hard constraint or a state-vector change. Order of composition in `mpc_core.py`'s `compute()`: `R_rate_scaled` starts as a plain copy of `self.R_rate`, `_steer_rate_anti_hunt` and `_reversal_penalty_boost` are applied to it and their resulting multipliers logged, then the corner-factor blend **overwrites** `R_rate_scaled[0,0]`'s base value and re-multiplies both logged multipliers back in on the same line (see the reversal-penalty section below for why this matters). `adaptive_Q_scaling` runs last, after the corner blend has already set `Q_base`, and is applied on top of the corner-blended `Q[0,0]`, not in place of it.

**What it replaced, and why re-adding it is a bad idea.** The scheduler above reacts to CURRENT curvature only. The design it replaced instead scanned FORWARD along the path every tick, trying to anticipate a corner before the car reached it. That earlier design does not work, for a structural reason explained below, and re-adding any part of it should be weighed against that reason, not just against how it tuned in isolation.

The previous design was ~15 interacting functions:

- `lookahead_curvature_profile` → a scalar `kappa_max_abs`, the peak curvature within a speed-scaled lookahead window.
- Reweighted `Q`/`R`/`R_rate` in anticipation of a corner not yet reached: approach/exit `Q[0,0]`/`Q[2,2]` boosts, `Q[3,3]`/`R[0,0]` relaxations, demand normalisation (`_corner_demand`/`_alat_ceiling_at`), a U-turn detector, straight-line `Q`/`R[0,0]` adjustments.
- A `CornerMap`/`_segment_corners` precomputed-path fast path for the same scan.

See `mpc_core.py`'s own "Lookahead gain-scheduling family: removed" comment for the exhaustive function-name list, and [`removed_mechanisms.md`](../removed_mechanisms.md) for the mechanism-level detail of each (`tuning.md` §4.4/§4.6/§4.8/§4.10 and `architecture.md`'s "Historical" subsection both link there rather than restating it).

The family was removed because this MPC formulation already predicts state error against the reference at each future horizon step. Reweighting TODAY's (usually near-zero) cost based on what a forward scan finds ahead does not change what the horizon predicts once the car gets there. The mechanism was reweighting a cost that mostly was not there yet, not manufacturing anticipation. Repeated piecemeal retuning of it never produced a clear net win. This is the same structural argument, one level up, that motivates the nonlinear MPC below: reweighting an existing error's cost is not the same as making the *prediction itself* see the road bend.

## Steering-rate anti-hunt (`steer_rate_anti_hunt_enabled` / `nmpc_steer_rate_anti_hunt_enabled`)

**Plain version:** steering chatter (rapid small back-and-forth wheel motion) is most likely exactly when the car does not need to be steering much at all: pointed straight, centred on the path, not currently in a corner. This mechanism makes rapid steering changes more expensive specifically in that "nothing to do" state, without touching cost anywhere else.

**Default off, experimental on both controllers.**

### Detection

Three current-state signals, each independently saturating toward 1.0 as its input shrinks toward zero; their product is the applied boost, so full strength requires all three to be small simultaneously:

```
boost_kappa = 1 / (1 + k_kappa * |kappa|)
boost_ey    = 1 / (1 + k_ey    * |e_y|)
boost_epsi  = 1 / (1 + k_epsi  * |e_psi|)
```

Constants (both sides, since 2026-08-19): `k_kappa=30.0`, `k_ey=15.0`, `k_epsi=11.5`. `e_psi` is included so a car that is centred (`e_y` small) but still misaligned after just exiting a corner (`e_psi` large) does not get the full straight-line boost, which would make exactly the yaw-back correction it needs artificially expensive.

### Penalty/effect

```
scale = 1 + (boost_max - 1) * boost_kappa * boost_ey * boost_epsi
R_rate[0, 0] *= scale
```

`boost_max` (`anti_hunt_boost_max`, default source of the ceiling) applies only when curvature, lateral error, and heading error are all near zero simultaneously; it fades continuously, never snaps, as any one of the three grows. `enabled=False` returns the input matrix unmodified.

### Integration

Both controllers, shared implementation (`nmpc_core.py` imports `mpc_core._steer_rate_anti_hunt` verbatim rather than keeping a separate copy). Enters as a cost-term multiplier on `R_rate[0,0]`, never a constraint or state change.

On the LTV-QP path this multiplier is computed and logged (`m_Rrate_antihunt`) before the corner-factor blend, then re-multiplied back into `R_rate_scaled[0,0]` on the blend's overwrite line (see the corner-factor scheduler section above), the same threading pattern the reversal penalty uses. On the NMPC path, `nmpc_corner_rrate_blend_enabled` and `nmpc_steer_rate_anti_hunt_enabled` are mutually exclusive alternatives for setting the base `R_rate[0,0]` value (the corner blend takes priority if both are set); the reversal penalty then applies on top of whichever one ran.

## Adaptive Q-scaling: lateral-error softening near the centreline (`adaptive_q_scaling_enabled`)

**Plain version:** when the car is already very close to the path centreline, a normal quadratic lateral-error cost still pulls proportionally hard toward zero error, which can encourage darting/overcorrecting right at the point the car should be settling rather than correcting. This mechanism softens that pull only when the lateral error is already small.

**Default off, not validated against live data**, implemented specifically to test against a live steering-reversal-rate log without changing any offline-tuned behaviour by default (the offline recorded-map rollout does not reproduce the live trend this was built to address; see `controller/model_utils.py`'s `adaptive_Q_scaling` docstring for the full caveat).

### Detection

Exact signal: `e_y`, current lateral deviation from the path centreline (m), sign ignored. Exact saturating formula (`model_utils.adaptive_Q_scaling`, mirrored by `mpc_core._adaptive_Q_scaling`), a linear ramp with a floor and a ceiling, not a smooth saturating curve like the two mechanisms above:

```
scale = floor                                              |e_y| <= ey_lo
scale = floor + (1-floor) * (|e_y|-ey_lo) / (ey_hi-ey_lo)   ey_lo < |e_y| < ey_hi
scale = 1.0                                                 |e_y| >= ey_hi
```

Exact thresholds, identical on both sides: `ey_lo = 0.05`, `ey_hi = 0.3`, `floor = 0.5`. These are deliberately far below `tracking_error_speed_gate`'s own `ey_lo=0.5`/`ey_hi=2.0`, a different mechanism gating a much larger-error "recovering from being badly off-line" regime, not small-error hunting.

### Penalty/effect

`Q[0,0] *= scale` (or `Q[0] *= scale` if `Q_base` is passed as a 1-D diagonal vector rather than a full matrix). `enabled=False` returns the input completely unmodified, not even copied.

### Integration

Both controllers can call it (same function on both sides), gated by `MPCParams.adaptive_q_scaling_enabled` live and `settings.py`'s matching flag offline. Enters as a cost-term multiplier on `Q[0,0]`, never a constraint or state change.

In `mpc_core.py`'s `compute()`, this runs **last** in the `Q[0,0]` pipeline: it is applied to `Q_base` after the corner-factor blend has already set `Q_base[0,0]` to its straight/corner-blended value, so the two compose (blend first, then soften near-centreline) rather than one overwriting the other:

```python
Q_scaled = _adaptive_Q_scaling(x0[0], Q_base, self.adaptive_q_scaling_enabled)
```

`x0[0]` is the delay-compensated `e_y` (rolled forward by `predict_ahead()` if `n_delay > 0`), not a bare pre-delay `e_y`; `compute()` has no separate bare-`e_y` variable in scope.

## Soft steering-reversal penalty (`reversal_penalty_*` / `nmpc_reversal_penalty_*`)

**Plain version:** a steering reversal (the wheel flicking one way then immediately the other) always passes through zero steering angle on the way. This mechanism makes it a little more expensive to change steering quickly right when the previous command was near zero, nudging the controller away from that flip-prone state without penalising the flip itself directly.

**Default off, experimental on both controllers.**

### Detection

Exact signal: `u_prev_steer`, LAST tick's already-committed steering command (rad), not the current solve's own decision variable. A reversal itself can't be detected/penalised directly inside one solve, since it depends on THIS tick's own decision (the thing being optimised), which would make the cost non-convex if used directly; `u_prev_steer` is a known constant by solve time, so keying on it instead keeps the term an ordinary quadratic.

Exact formula (`controller/model_utils.py`'s `reversal_penalty_boost`, mirrored by `mpc_core.py`'s `_reversal_penalty_boost`):

```
boost_near_zero = 1 / (1 + k * |u_prev_steer|)
scale = 1 + (boost_max - 1) * boost_near_zero
```

`k = 8.0 rad^-1` (`reversal_penalty_k`) sets half-boost at ~7.2 deg of previous steering; `boost_max = 4.0` (`reversal_penalty_boost_max`) is the ceiling, reached only when `u_prev_steer = 0` exactly.

### Penalty/effect

Multiplies `R_rate[0,0]` (steering rate-of-change cost) by `scale` above. `enabled=False` returns the input matrix completely unmodified (not even copied).

### Integration

Both controllers, each with its own enable flag and `-1.0`-inherit override convention (`mpc_params.py`'s `reversal_penalty_enabled`/`_boost_max`/`_k` for LTV-QP; `nmpc_reversal_penalty_enabled`/`_boost_max`/`_k` for NMPC, `-1.0` = inherit the LTV-QP value). Enters as a cost-term multiplier on `R_rate[0,0]`, never a hard constraint.

**This multiplier IS threaded into the final `R_rate_scaled[0,0]` value handed to the QP**, unlike the removed adaptive-R_rate mechanism below whose result was computed and then silently discarded. In `mpc_core.py`'s `compute()`, the corner-factor blend line is:

```python
R_rate_scaled[0, 0] = _blend(
    self.params.rrate_steer_straight, self.params.rrate_steer_corner,
    corner_frac,
) * adapt["m_Rrate_antihunt"] * adapt["m_Rrate_reversal"]
```

`adapt["m_Rrate_reversal"]` is the reversal-penalty multiplier computed earlier in the same tick; because it is explicitly re-multiplied back in on this line, it survives the overwrite and reaches the solver. On the NMPC path (`nmpc_core.py`), the same composition happens through a running `rrate_steer_current` value: the corner-blend/anti-hunt branch sets it first, then `_reversal_penalty_boost` is applied on top of whatever that branch produced, so the penalty compounds onto the corner blend rather than replacing it.

**Implemented in:**
- **Live**: `mpc_core.py`'s `_reversal_penalty_boost` (LTV-QP), mirrored in `nmpc_core.py` (NMPC path, tracked through `rrate_steer_current` as above).
- **Offline**: `controller/model_utils.py`'s `reversal_penalty_boost` (same function, both controllers call it), wired into `sim/rollout_core.py`'s LTV-QP and NMPC branches; `settings.py`'s `REVERSAL_PENALTY_*` / `NMPC_REVERSAL_PENALTY_*` constants (same `-1.0`-inherit convention).

**Validated on the LTV-QP path only.** Offline A/B on `comp_test_map_3` showed a genuine ~20% reversal-count reduction at negligible cost, no DNF. The NMPC path's own reversal penalty regressed the composite score offline (reversals dropped only ~5.6% while score worsened), so leave `nmpc_reversal_penalty_enabled` off unless a live A/B says otherwise; it is wired and functional, just not currently worth enabling.

## Slew-rate limit (`du_max`): on both sides, at 180 deg/s

**Plain version:** the steering rack can only turn so fast, so the controller is not allowed to command a bigger step change in steering than the real actuator (or, offline, the simulated one) could achieve in one control tick. This cap has to be identical on both sides, or weights tuned offline will ask more of the real car than it can deliver.

### Detection

Not signal-triggered: this is an always-present hard bound on the solver's own decision variable, not a reweight keyed off a measured state.

### Penalty/effect

Per-step change in `[delta_cmd, a_cmd]` is bounded by `du_max`, expressed as a *rate* (`max_steer_rate * DT`) rather than a fixed per-step angle, so its physical meaning survives a change of `DT`. Current value: **180 deg/s** (was 80 deg/s).

**Why 180 deg/s and not the original 80.** Live telemetry (`mpc_standalone_control_1785976976.csv`) showed the steering command pinned exactly on an 80 deg/s limit for **41% of all control steps**, reversing sign at ~8 Hz, a rate-limit-induced limit cycle, not a weight-tuning problem. Inverting the logged yaw rate through the kinematic bicycle (`delta = atan(L*r/v)`) put the *achieved* roadwheel rate at p99 ≈ 138 deg/s and max ≈ 218 deg/s, so the real actuator is at least ~200 deg/s. 180 deg/s sits just under that measured floor. The true FSDS steering rate is **not recoverable from this repo** (the PhysX vehicle setup lives in git-LFS `.uasset` binaries), so 180 deg/s is a measured lower-bound estimate, not a datasheet figure. Refine it via system-ID on the running sim and update both sides together.

**Why the offline sim didn't catch the original 80 deg/s limit as a problem.** With the same weights and the same 80 deg/s limit, an offline rollout on `PATH_MICRO_SLALOM` hits the limit on only **0.5%** of steps versus the live car's 41%, and composite scores at 80 vs 180 deg/s differ by <0.002 across `PATH_S_BEND`, `PATH_SUDDEN_TURN` and `PATH_SPIRAL`. The offline sim uses a fixed `DELAY_STEPS = 1` and a smooth synthetic path, so it never enters the saturated regime the live car lives in (jittery measured delay, replanned/noisy perception path). Do not read "the offline score barely moved" as "the change doesn't matter". The constraint is there to stop the live controller sitting on its limit.

### Integration

Both controllers, entering as a **hard constraint** on the QP/SQP's own decision variables, not a cost-term reweight and not a state-vector change. `init_parameterized_mpc()`/`solve_mpc()` in `controller/lmpc/solve.py` take an optional `du_max`, mirroring the live formulation; it is baked into the cached QP the same way `u_min`/`u_max` are, so it participates in the same cache-staleness check (passing a different `du_max` rebuilds the problem instead of silently reusing stale constraints). Only the step-0 constraint against `u_prev` is omitted offline, since step 0 is already anchored through `weighted_u_prev` in the cost, and `u_prev` isn't a constraint parameter in the cached formulation. Without this constraint in `controller/lmpc/solve.py`, the offline tuner optimises against a plant that can change steering arbitrarily fast while the real car is clamped, so weights tuned offline do not transfer faithfully, independent of any weight choice.

## MPC prediction horizon: frozen target speed

**Plain version:** the target speed the MPC predicts against does not change across the steps it looks ahead within one solve. The whole horizon plans against a single frozen speed target, refreshed only on the next tick.

This is an architectural characteristic of the LTV-QP formulation, not a triggerable mechanism, so it does not cleanly split into detection/penalty/integration: there is no signal that turns it on or off, and no separate cost term to describe, it is simply how `x0[4]` (`e_v`) is defined for every solve. `sim/rollout_core.py`'s (and `mpc_core.py`'s) MPC formulation bakes `desired_speed` into `x0[4]` as a single scalar frozen for the whole prediction horizon, refreshed only on the next tick's solve. See `README.md`'s state-vector section (search "e_v's target speed is frozen for the whole horizon") for the full explanation; not repeated here to avoid duplication.

Applies to the LTV-QP only; the NMPC's `q_e_v` cost is likewise evaluated against a single frozen `v_ref` per solve today (the per-stage horizon speed profile that would have changed this was tried and removed, see the Nonlinear MPC section's "Three MPCC-inspired additions" below).

## Accel/brake effort weight split

**Plain version:** braking and accelerating are charged different "effort" costs instead of one shared cost, so tuning how eager the controller is to brake doesn't automatically change how eager it is to accelerate.

### Detection

Exact signal: the sign of the commanded longitudinal accel decision variable `a_cmd` itself (a QP decision, not a measured state): `a_cmd >= 0` (accelerating) vs. `a_cmd < 0` (braking). This is a structural split in the cost, not a saturating/threshold detector on an external signal.

### Penalty/effect

Two independent weights replace one symmetric `R_diag[1]` scalar: `r_a_accel` penalises `a_cmd >= 0` and `r_a_brake` penalises `a_cmd < 0`, via

```
R_a_accel * sum(pos(a_cmd)^2) + R_a_brake * sum(neg(a_cmd)^2)
```

in the QP cost (`cp.pos(u[1,:])`/`cp.neg(u[1,:])`). `R_diag[1]`/`self.R[1,1]` remain nominal/reporting values only; no adaptive gain (`_adaptive_R_scaling`, the corner-factor blend, etc.) touches index 1 of `R`/`R_rate` anywhere in this codebase, so this split composes cleanly with the rest of the adaptive-gain machinery without double-scaling.

### Integration

Both controllers. Enters directly in the QP/SQP cost (a structural cost-term split, not a hard constraint or state-vector change).

**Implemented in:**
- **Live**: `mpc_core.py`'s `_build_qp`/`_solve_qp` (`r_a_accel_param`/ `r_a_brake_param` `cp.Parameter`s), `mpc_params.py`'s `r_a_accel`/ `r_a_brake` fields, `fsae_params.yaml`'s `controller.r_a_accel`/ `controller.r_a_brake`, `launch_all.sh`'s `MPC_R_A_ACCEL`/`MPC_R_A_BRAKE` shortlist entries.
- **Offline**: `controller/lmpc/solve.py`'s `init_parameterized_mpc`/ `solve_mpc` (same `cp.pos`/`cp.neg` split; `r_a_accel`/`r_a_brake` kwargs default to `R[1,1]` when omitted, for backward compatibility), `settings.py`'s `R_A_ACCEL`/`R_A_BRAKE` (read by `sim/rollout_core.py`'s `solve_mpc()` call).

**Re-check `mpc_params.py`'s `r_a_accel`/`r_a_brake` and `settings.py`'s `R_A_ACCEL`/`R_A_BRAKE` for the current numeric defaults before relying on any value quoted elsewhere**: these two remain the most frequently live-retuned weights in the whole `MPCParams` set. Keep `settings.py` synced to `mpc_params.py`'s live values per this document's parity rule whenever either changes.

For the full diagnosis history (the corner-entry-too-hot symptom that motivated the split, the slack-variable design considered and rejected in favor of the `cp.pos`/`cp.neg` rewrite, and the live-tuning value trajectory), see `docs/logs/late_turn_in_investigation.md`'s "Part 0 (background): how the accel/brake effort split (`r_a_accel`/`r_a_brake`) came about" and `docs/logs/sim_to_real_investigation.md` §59 for the preceding single-scalar `r_a` cut this split superseded.

## Post-solve output smoothing, removed

**Plain version:** a low-pass filter was once applied to the steering command after the solver had already produced it, to smooth out chatter. It has been removed because it just added lag, and the QP's own steering-rate cost fixes the same chatter without that penalty.

**A post-solve low-pass filter on the SOLVED steering command (`filtered += alpha*(raw - filtered)`, then `steering = (1-w)*raw + w*filtered`), distinct from every `Q`/`R`/`R_rate` gain-scheduling mechanism elsewhere in this codebase (which reshape the QP's COST before solving, fresh each tick with no memory), this one persisted `filtered` tick to tick, adding genuine lag.**

Removed: it never improved on the QP's own steering-rate cost (`r_rate_delta`), which attacks the same jitter at its source instead of filtering an already-chattery command after the fact (see `tuning.md`'s tuning-order table, `r_rate_delta=52.5` and `NMPC_RJERK_DELTA=150.0` are the levers that worked). Shipped default was always `false`; the mechanism (node params, `peak_kappa_ahead()`, the offline mirror in `sim/rollout_core.py`/`settings.py`, and the `launch_all.sh`/launch-file wiring) has been deleted from both the live and offline sides rather than left as unused dead code.

## Gradual-corner accel oscillation is genuine track geometry, not a bug

This section is a diagnostic note, not a mechanism: it does not have its own detection/penalty/integration split, since the behaviour it describes is a property of the precomputed speed profile's own curvature-following, not a separate adaptive-gain layer bolted on top of the solve.

Through mild/gradual turns (e.g. S-curves/chicanes), `v_desired` and `a_cmd` can legitimately oscillate, rising to a local peak, dipping, rising again, because the precomputed speed profile (`speed_profile.csv`) is tracking a genuine sign change in the path's own curvature (a left-hand bend straightening briefly before curving right into the next bend), not adaptive-gain misbehaviour or spurious jitter. The offline curvature scan that builds `speed_profile.csv` (`sim/speed_profile.py`'s `compute_speed_profile()`) correctly speeds the car up through the brief straight and slows it back down anticipating the next corner.

Before treating this pattern as a bug, cross-reference the car's actual `v_desired`/`a_cmd` against the track's own `speed_profile.csv` and the raw path's Menger curvature at the same position: a genuine varying-radius feature (sign change in curvature, not just magnitude) confirms the oscillation is correct tracking, not noise.

**Do not "fix" this via `R_rate_diag[1]`** (acceleration-rate-of-change cost) or similar damping, as that makes the MPC slower to respond to a real, upcoming tightening corner, trading a correctly-anticipated slowdown for a late, harder one. If oscillation on a specific track needs addressing, the correct levers are the speed profile's own generation parameters (`a_lat_max`, scan window in `sim/speed_profile.py`'s `compute_speed_profile()`) or the raw path geometry itself (smoothing a spurious kink), never the live adaptive gains. Confirm the geometry is spurious (not real track shape) before touching either lever; see `late_turn_in_investigation.md`'s "Gradual-corner accel oscillation" section for the full worked example of how to distinguish the two.

## Dynamic speed cap

**Plain version:** the precomputed speed profile assumes the car is where the plan expects it to be. If the car is actually running faster than planned (e.g. it exited the last corner quicker than expected), this mechanism pulls the target speed back down before the next corner, on top of whatever the precomputed profile already says.

**Code default on, but disabled by the shipped launch shortlist** (see Integration below); do not assume enabled from the code default alone.

### Detection

Runs `curvature_speed()`'s own scan/braking-distance mechanism (see that function's docstring for the scan mechanism itself, not repeated here) with tighter constants than the live-only branch uses: `DYNAMIC_CAP_A_LAT_MAX=3.2` and `DYNAMIC_CAP_SAFETY=0.9` (vs. `a_lat_max=4.75`/`safety=1.0` elsewhere), so it detects the need to slow down a little before the oracle profile itself would be violated. The trigger is structural (whenever a track is mapped) rather than threshold-crossing: the cap is always evaluated, and only its *value* changes what the effective target speed becomes.

### Penalty/effect

Not a cost reweight: replaces the target speed value itself.

```
controller_target_speed = min(precomputed_speed_at(...), dynamic_speed_cap(...))
```

`precomputed_speed_at()` (used when `USE_PRECOMPUTED_SPEED_PROFILE=True` / `map_path` is set) is a static, position-indexed lookup with no notion of the car's actual current speed relative to remaining braking distance; `dynamic_speed_cap()` (a thin wrapper over `curvature_speed()`) exists to catch the case where live tracking has drifted ahead of the oracle plan and pull the target speed down before a corner is reached, never up. Downstream, `tracking_error_speed_gate()` and `SPEED_TARGET_RISE_RATE` apply exactly as before; the cap only changes what `v_curv` feeds into that existing pipeline. It has no effect when no track is mapped, and is byte-identical to pre-cap behaviour when disabled.

### Integration

Live-only, `mpc_controller.py` (the node shared by both LTV-QP and NMPC), not `stanley_controller.py`. Not controller-specific in the LTV-QP-vs-NMPC sense, since it selects the target speed both solves consume, upstream of either one's own cost/constraint machinery. Enters as a substitution of the target-speed input the controller solves against, not a QP/SQP cost term and not a hard constraint inside the solve itself.

Controlled by `enable_dynamic_speed_cap` (ROS param) / `ENABLE_DYNAMIC_SPEED_CAP` (`settings.py`), code default `True`, but **`ros2/launch_all.sh`'s MPC tuning shortlist currently overrides this to `false`**, so a plain `launch_all.sh` run has the cap off; check that shortlist rather than assuming the code-level default is what actually drives. Do not re-enable for a live run without first understanding why it regressed steering saturation and heading error offline (see `docs/logs/late_turn_in_investigation.md`'s "Dynamic speed cap" addendum for the measured before/after and the live-test result). The a_lat-ceiling metric it targets improved, but the metrics that matter more got worse, for reasons never diagnosed.

## Stanley speed-target smoothing, ported from the MPC pipeline

**Plain version:** without a precomputed speed profile, the live planner rebuilds the path every tick, and that path carries a few centimetres of lateral wiggle frame-to-frame. Feeding that wiggle straight into a target speed makes the target randomly jump several m/s in a single 50 ms tick, even on a straight. On the LTV-QP/NMPC controller this was already smoothed out; Stanley had no such smoothing at all, so a noisy tick could simultaneously spike steering (Stanley's cross-track error comes from the same noisy path) and yank the speed target down, and nothing pulled the target back up once tracking degraded. This combination produced a live spin-out within the first few seconds of a run.

Not yet live-tested as of this port; validate on a live/sim run before trusting it the way `dynamic_speed_cap()` above has been.

### Detection

Three independent triggers, live-mode only (`map_path` unset; the precomputed-speed-profile branch, `precomputed_speed_at()`, is untouched):

- Tick-to-tick fall in `curvature_speed()`'s raw output (the function itself has no memory of its own last value, so any fall is a candidate for limiting).
- `|e_y|`/`|e_psi|` crossing `tracking_error_speed_gate()`'s own thresholds.
- Tick-to-tick rise in the final composed target.

### Penalty/effect

Not a cost reweight: each trigger rate-limits or gates the target-speed *value* itself, applied in sequence:

- **`V_CURV_FALL_RATE`** (7.0 m/s²) rate-limits how fast `curvature_speed()`'s output may *fall* tick-to-tick.
- **`tracking_error_speed_gate()`**, rate-limited by **`GATE_RATE_LIMIT`** (2.0 /s in either direction), scales the target down once `|e_y|`/`|e_psi|` grow past their thresholds, so a controller that is already tracking badly is not simultaneously told to go fast.
- **`SPEED_TARGET_RISE_RATE`** (7.0 m/s²) bounds the final composed target's rise, seeded from the car's actual speed on the first tick so a standing start does not jump straight to the full target.

Stanley has no fixed control-loop timer (`mpc_controller.py`'s `CONTROL_HZ` has no Stanley equivalent, it runs off `car_position` arrival), so all three limiters use a measured `dt` between ticks rather than a compile-time tick period.

### Integration

Stanley-only (`stanley_controller.py`), porting the same three safeguards `mpc_controller.py` already had around `curvature_speed()`'s raw output. Enters as a substitution/rate-limit on the target-speed value fed to Stanley's own law, not a cost term (Stanley has no cost function) and not a hard constraint.

## Precomputed shaped heading-lead profile

**Plain version:** the car is told to start turning in slightly before the path geometry itself says to, by however much yaw it can physically achieve before the bend at the speed it's planned to be going. This is a derived, offline-only third pass over an already-optimized raceline, replacing the geometric heading reference (`atan2` of the path tangent) with a SHAPED one.

Default off (`use_precomputed_heading_profile=false` on both nodes, both launch files, and in `launch_all.sh`).

### Detection

Not a runtime signal: the lead is computed once, offline, per waypoint, at export time, not triggered by any live state. At each waypoint, the lead angle is however much yaw the car can physically achieve between here and the upcoming bend, scaled by achievable yaw rate at that station's already-planned speed (not a fixed lookahead distance, which would saturate steering to full lock at ~8 m of lead on a realistic corner-entry ramp). This naturally decays the lead to ~0 once a corner's constant-curvature section begins.

### Penalty/effect

Not a cost reweight: substitutes the reference value `e_psi` is measured against. `mpc_core.py`'s `MPCController.set_heading_profile(psi_target)` substitutes the shaped value for `path_yaw` at `base_idx`, only for `e_psi`'s reference (`e_y`'s projection keeps using the geometric tangent). This changes `e_psi` at `k=0` itself, before the QP ever runs, rather than adding a future-deviation cost the QP is free to satisfy however is cheapest (the failure mode of both curvature-forcing and a cost-target shift, both rejected in favour of this approach).

**`check_slip`'s `SLIP_LIMIT_RAD` (5°) is an unvalidated placeholder**: diagnostic-only, does not fail the export or reshape anything. Do not treat it as authoritative until it has a real measurement behind it.

### Integration

LTV-QP only: `use_precomputed_heading_profile` has no effect when `use_nmpc` is active, as the NMPC model already carries curvature directly. Enters as a state-vector-adjacent substitution (the reference value `e_psi`'s error is computed against at `k=0`), not a cost term and not a hard constraint.

**Where it lives:**
- Offline: `fsae_MPCTest/tuner/tools/raceline_optimizer.py`: `max_yaw_rate`, `build_shaped_heading_profile`, `check_slip`, run in `export()` after `optimize_raceline()`'s path+speed have converged. Does not feed back into path/speed optimization (kept as a separate derived pass to avoid invalidating that loop's own tuned constants).
- CSV format: `x,y,psi,psi_target,v_target` (5 columns), backward-compatible with the old 4-column format; `control_utils._load_profile_csv` sets `psi_target = psi` for any 4-column file.
- Live: `mpc_core.py`'s `MPCController.set_heading_profile(psi_target)`. Loader: `control_utils.load_path_heading_profile_csv`.
- Toggle: `use_precomputed_heading_profile`, a node-level launch parameter (not an `MPCParams` field), exposed as `USE_PRECOMPUTED_HEADING_PROFILE` in `ros2/launch_all.sh`.

**Standing warning:** do not extend this mechanism from a k=0-only lead to a full per-horizon-step reference, as that reproduces curvature-forcing's same wrong-direction-dip trap (see `docs/logs/late_turn_in_investigation.md` Part 15).

For the full derivation, the rejected fixed-lookahead-distance and cost-target-shift alternatives, the `comp_test_map_3`-specific near-everywhere-lead caveat, and both rounds of live-test results (an initial "worse" read followed by a high-variance correction across more runs), see `docs/logs/late_turn_in_investigation.md` Parts 7-13.

## Nonlinear MPC (`use_nmpc`): a second controller

**Plain version:** the LTV-QP controller (`MPCController`) is one of two controllers available; the other, `NMPCController`, solves a nonlinear version of the same problem so its own prediction can see the road curve ahead, instead of only reacting to error that has already appeared.

`ros2/src/fsae_planning/control/fsae_control/fsae_control/mpc/nmpc_core.py`'s `NMPCController` is a Frenet-frame **nonlinear** MPC (Gauss-Newton SQP, a condensed dense QP subproblem solved by OSQP, real-time-iteration style: one SQP iteration per tick, warm-started from the previous tick). It has an independent offline port, `controller/nmpc/solver.py`'s `NMPCController` (same model, same SQP/OSQP scheme, not an import, as the two repos cannot import each other), wired into `sim/rollout_core.py`'s `run_core_rollout()` behind `settings.USE_NMPC` (default false). It is selected live by the node parameter `use_nmpc` (default **false**) and replaces `MPCController` wholesale when true; `mpc_core.py` is untouched either way.

### Why it exists

`MPCController._discrete_model` is the bicycle model in error coordinates with the reference frame's own rotation dropped, and it is missing `e_psi_dot = r - kappa(s) * s_dot` from its `Ad`/`Bd`, so with `e_y = e_psi = 0` the QP's rollout predicts staying at zero error forever and no weighting can produce turn-in before real error exists (`MPCController` measurably commands 0.000 deg at 8 dead-on-line states approaching a known bend).

The Frenet formulation fixes this by making `kappa` a function of the **state** `s` (driven by the car's own predicted motion) rather than a horizon-indexed exogenous schedule, so the anticipation obligation is not schedulable and the solver cannot pre-pay it early the way the curvature-forcing and heading-lead workarounds do. Full derivation, the formulation survey (including why MPCC's progress-maximising formulation was not adopted wholesale), and the falsification methodology are in `late_turn_in_investigation.md` Part 16.

### Model and what it reuses

States `[s, e_y, e_psi, v_x, v_y, r, delta_act, a_act]`, inputs `[delta_cmd, a_cmd]`. Every vehicle constant (`lf` 0.70, `lr` 0.85, `m` 255, `Iz` 150, `Cf`/`Cr`, `tau_delta` 0.08, `tau_a` 0.02, `MAX_STEER_RAD`, `MAX_ACCEL`, `MAX_BRAKE`, `du_max`) and the kinematic/dynamic blend band (1.0–2.5 m/s) come from `MPCController.__init__` unchanged, no new physical constant. Cost weights come from the same `MPCParams` instance the LTV-QP uses. Three deliberate differences from the LTV-QP:

1. **`q_r` weights heading-error RATE** (`r - kappa*s_dot`), not absolute yaw rate, same slot, different regressor from the LTV-QP's `q_r`.
2. **`e_y`/`e_psi` are measured against the smoothed reference**, not the raw segment tangent (bounded by the same 1.5 m smoothing window as `control_utils.curvature_speed()`'s `dense_step=0.5`/`w=3`).
3. **FSDS's measured `a_lat` ceiling is inside the prediction**, as a smooth `tanh` saturation of predicted tyre forces, reusing the same numeric law (flat/slope/intercept = 7.5/0.47/2.46) as `mpc_core._alat_ceiling_at` / `model/vehicle_physics.alat_ceiling_at`, hardcoded as class-level defaults on `nmpc_core.py`'s `_Plant` rather than read from `MPCParams`. `nmpc_alat_ceiling_enabled=false` recovers the unconstrained plant, mirroring `VehicleParams.alat_ceiling_enabled`.

Tyre lateral force (`F_yf`/`F_yr`, from `alpha_f = atan((v_y + lf*r)/v_safe) - delta_act`) is scaled by the same kinematic/dynamic `blend` factor used elsewhere in the model, applied immediately after the force is computed and after the `alat_ceiling` soft saturation. Without this scaling the slip-angle floor at low `v_x` lets steering alone manufacture lateral force with zero forward speed. This applies in all three copies: `nmpc_core.py` and `controller/nmpc/dynamics.py`'s `_f`, `_f_scalar`, and its `nmpc_friction_circle_enabled`-only `_tyre_forces()` helper.

### What is inactive when `use_nmpc=true`

- The entire adaptive gain schedule (lookahead approach/exit boosts, yaw-rate relax, straight boosts, centred softening, U-turn detector), and no `m_*` telemetry columns are written for these.
- `use_precomputed_heading_profile` has no effect (one startup log line says so), as the NMPC's curvature model already carries what that flag approximates.
- `curvature_forcing_enabled`, `ref_heading_rate_limit_enabled` are LTV-QP-only.
- `use_precomputed_corner_map` no longer exists on either `use_nmpc` setting (removed by the corner_factor rewrite; see `architecture.md`'s "Precomputed corner segmentation" note).

**Exception: `nmpc_steer_rate_anti_hunt_enabled`** (default `False`, `mpc_params.py`'s `nmpc_steer_rate_anti_hunt_enabled`/ `nmpc_anti_hunt_boost_max`) is an independent NMPC-only opt-in, not inherited from the LTV-QP's `steer_rate_anti_hunt_enabled`. Unlike the rest of the adaptive-gain family, it only makes steering-rate more expensive when the current state is already centred/aligned/uncurving, the opposite direction from anticipation. Reuses `mpc_core._steer_rate_anti_hunt` verbatim on the live side and `model_utils.steer_rate_anti_hunt` offline. Writes the existing shared `m_Rrate_antihunt` telemetry column when on. See `mpc_params.py`'s field comment and `nmpc_core.py`'s module docstring ("WHAT THIS CONTROLLER DELIBERATELY DOES NOT DO") for the reasoning.

`use_precomputed_path` / `use_precomputed_speed` / `enable_dynamic_speed_cap` / delay compensation (`delay_compensation_enabled`, `pose_age_lp_alpha`, `n_delay_hysteresis`, `max_delay_compensation_steps`) all work exactly as under the LTV-QP; the delay rollforward uses the nonlinear model instead of `predict_ahead()`'s linearisation.

### Three-zone rate schedule (`nmpc_rrate_zone_enabled`)

**Plain version:** steering-rate cost is relaxed while approaching a corner (so turn-in isn't taxed), tightened back down once actually cornering (so mid-corner chatter is still damped), and boosted on a clear straight, using both the car's current curvature and the peak curvature the solver's own predicted horizon sees ahead.

Default off. NMPC-only, no LTV-QP equivalent.

#### Detection

Two curvature inputs, both passed through the same saturating `_corner_factor` curve (shared with the corner-factor scheduler above, same `k = nmpc_corner_factor_k`):

```
now   = _corner_factor(|kappa_now|, k)     # current curvature
ahead = _corner_factor(max(|kappa_horizon|), k)   # peak |kappa| the SQP's own rolled-out horizon X predicts
```

`kappa_now` is available as soon as `x0` is known; `kappa_ahead` requires the horizon rollout `X`, so this schedule is applied after `_rollout()`, not in the earlier per-tick block the corner blend/anti-hunt/reversal composition runs in. Against a live planner path (not a static raceline) this inherits the open centreline curvature-spike defect (see "Before changing the planner" in `CLAUDE.md`), since there is no rate limiter downstream to absorb a spurious spike in `kappa_ahead` the way there is for a speed target.

#### Penalty/effect

Three-zone continuous multiplier, no thresholds or hysteresis:

```
straight  (now low, ahead low)   -> boost_straight   (>= 1, nmpc_rrate_zone_boost_straight)
approach  (now low, ahead high)  -> ease_approach     (nmpc_rrate_zone_ease_approach)
corner    (now high)             -> floor_corner      (<= 1, nmpc_rrate_zone_floor_corner)
```

Blend order matters: the approach ease is applied first against the straight boost, then the corner floor takes over as `now` rises, giving boost -> ease -> floor in sequence ("release the brake before you need to turn"). The endpoints are only reached as `_corner_factor` saturates toward 1, which depends on `k` relative to the track's own curvature range; check the `m_Rrate_zone` telemetry column against `floor_corner` before concluding the schedule reached its endpoints.

#### Integration

NMPC-only. **Multiplies** the whole-horizon `R_rate[0,0]` array (unlike the corner-factor blend, which overwrites the per-tick base value): `self._Rr_flat = self._Rr_flat * tile([m_rrate_zone, 1.0], N)`, applied on top of whatever the corner blend/anti-hunt/reversal composition already produced for `Rr_flat`, after the horizon rollout, before the SQP's cost matrix `_ErE` is rebuilt from it. Enters as a cost-term reweight across the whole SQP horizon, not a single-tick value and not a hard constraint.

### Three MPCC-inspired additions

Assessed against Alexander Liniger's Model Predictive Contouring Control (`https://github.com/alexliniger/MPCC`). MPCC's headline idea, progress `θ` as a free variable the solver maximises, was not adopted: it reintroduces the "exogenous, schedulable future obligation" failure mode this NMPC's `kappa(s)`-as-state design exists to avoid. Three narrower ideas were kept, all NMPC-only and implemented identically in `nmpc_core.py` (live) and `controller/nmpc/` (offline). None of the three fits a clean detection/penalty/integration split the way the weight-reweighting mechanisms above do: 1 is a reference-construction quality change with no separate trigger or penalty, 2 is fully removed (kept here only so it isn't re-tried), and 3 is a proposed, currently-disabled hard constraint rather than an active mechanism.

1. **Spline-based path reference**: `nmpc_spline_reference_enabled` / `NMPC_SPLINE_REFERENCE_ENABLED`, default **true**. `PathReference` fits `x(s)`/`y(s)` as independent `scipy.interpolate.CubicSpline` objects over arc length and derives `kappa(s)`/`psi_ref(s)` analytically (`kappa = (x'y'' - y'x'') / (x'^2+y'^2)^1.5`), replacing the old dense-resample + moving-average + finite-difference pipeline. A strict numerical-quality improvement with no new solver coupling: it changes how `kappa(s)`/`psi_ref(s)` are computed from the reference path, not what cost or constraint consumes them. The old moving-average path is kept behind the flag for A/B if needed. Directly targets the open "centreline curvature spikes" defect described below.
2. **Horizon speed profile: tried, removed, do not re-add without new evidence.** Two variants existed, both meant to sample a precomputed per-lap speed profile at each horizon stage's own predicted arc length (`PathReference.v_ref_at(s)`) instead of holding `v_ref` constant across the horizon, and both were live-tested and rejected. A cost-term version (`nmpc_horizon_speed_profile_enabled`) summed `v_x - v_ref(s_k)` across all stages, which let a high `v_ref` at a later stage offset a low `v_ref` at an earlier one within the same solve, defeating the non-schedulability property this feature was meant to inherit from `kappa(s)`; live-tested, produced a 16.7 m/s corner overspeed against a 3-5 m/s target. A follow-up hard-constraint version (`nmpc_speed_limit_enabled`) replaced the cost term with a per-stage inequality specifically to close that loophole, but failed live on two separate tests for a different reason: the constraint is keyed to the solver's own predicted trajectory, so a wrong prediction satisfies it on paper while the real car is still measurably over target, producing the same corner-overspeed/off-track outcome the inequality was supposed to prevent. Both flags and their shared plumbing (`v_ref_at`, `path_v_xy`/`path_v`, the per-stage slack rows) were removed from `nmpc_core.py`/`nmpc_params.py` (live and offline) rather than left as dead/experimental code. See `late_turn_in_investigation.md` Part 16 §16.9 and `docs/logs/nmpc_speed_limit_investigation.md` for the full evidence from both rejections before attempting a third variant.
3. **Friction-circle hard constraint**: `nmpc_friction_circle_enabled` / `NMPC_FRICTION_CIRCLE_ENABLED`, default **false**. Detection would be the same tyre-force computation the existing soft `tanh` saturation already uses (`F_yf`/`F_yr` from `alpha_f`/`alpha_r`); the effect would add a hard `|F_yf|, |F_yr| <= F_max` bound (additional to, not replacing, the existing soft saturation), integrated as a genuine hard constraint in the SQP rather than a cost term, with `F_max` derived from the same measured ceiling law. **Do not enable without first fixing `telemetry_logger.py`'s `NMPC_COLUMNS`** (missing `nmpc_fyf_max_abs`/ `nmpc_fyr_max_abs`) **and re-deriving a looser `F_max`**, as the hard bound has no slack variable, and ordinary cornering geometry on this track conflicts with `F_max = m * ceiling(v_x) / 2` per axle under completely normal driving, not just extreme conditions. See `late_turn_in_investigation.md` Part 16 §16.9 for the failure evidence.

Neither feature 2 nor 3 has offline A/B numbers; reproduce a comparison with `python -m tuner.nmpc_offline_check` once one exists.

### Dependencies

`osqp` only, already a documented requirement of `mpc_core` via cvxpy (`fsae_control/package.xml`). CasADi/acados are **not** used by the shipped code (not installable into the ROS interpreter on Ubuntu 24.04 without `--break-system-packages`); the SQP and its Jacobians are numpy-only. CasADi was used once, from a private `--target` install, purely to cross-check the optimum against IPOPT.

### Which settings affect which controller

Every `MPCParams`/`NMPCParams` field carries an explicit `metadata["controller"]` tag (`"both"`, `"ltv_qp_only"`, or `"nmpc_only"`); see `mpc_params.py`/`nmpc_params.py` for the authoritative per-field value. `settings.py` mirrors the same three-way classification as a `[LTV-QP only]`/`[NMPC only]`/`[shared]` comment prefix on each constant, and `ros2/launch_all.sh` tags its shortlists the same way.

**Shared base weights** (`mpc_params.py:47-67`, read by both controllers through `nmpc_core.py`'s `_pick(override, inherited)` at lines 859-877): `q_e_y`, `q_e_yd`, `q_e_psi`, `q_r` (meaning differs, see above), `q_e_v`, `r_delta`, `r_a_accel`, `r_a_brake`, `r_rate_delta`, `r_rate_a`, `terminal_q_scale`. A launch with every `nmpc_q_*`/`nmpc_r_*` override left at its `-1.0` sentinel starts the NMPC from the LTV-QP's own tuned set exactly.

**Shared delay-compensation fields** (`mpc_params.py:73, 80, 84-85`): `delay_compensation_enabled`, `max_delay_compensation_steps`, `pose_age_lp_alpha`, `n_delay_hysteresis` gate/shape both sides identically (mechanism differs: NMPC rolls `x0` through the nonlinear model instead of `predict_ahead()`'s linearisation). `predict_epsi_clip` (`mpc_params.py:81`) is LTV-QP only, specific to `predict_ahead()`'s linear rollforward.

**LTV-QP-only, the adaptive-gain-schedule fields** (`mpc_params.py:70-72, 74, 77, 81, 88, 91, 101-161`): `adaptive_q_scaling_enabled`, `steer_rate_anti_hunt_enabled`, `ref_heading_rate_limit_enabled`, `ref_heading_rise_rate_deg_s`, `anti_hunt_boost_max`, `corner_factor_k`, `q_ey_straight`/`q_ey_corner`, `q_epsi_straight`/`q_epsi_corner`, `q_r_straight`/`q_r_corner`, `rrate_steer_straight`/`rrate_steer_corner`, `r_steer_corner_mid`, `low_speed_corner_boost_v_half`/`_max_extra`, `epsi_ra_half_rad`/`_accel_boost_max`/`_brake_floor`, `reversal_penalty_enabled`/ `_boost_max`/`_k` (soft steering-reversal penalty, see its own section below). None have any read site in `nmpc_core.py`. (`adaptive_r_rate_enable_in_corners`/`adaptive_r_rate_during_floor` no longer exist, see `removed_mechanisms.md`'s "Adaptive R_rate current-curvature floor" entry.)

**NMPC-only overrides** (`mpc_params.py:189-205, 222-223, 238-240, 253-256`), each read solely by `nmpc_core.py`'s `_pick()` calls; `mpc_core.py` never references any of them:

- Base weight overrides: `nmpc_q_e_y`, `nmpc_q_e_yd`, `nmpc_q_e_psi`, `nmpc_q_epsi_dot` (overrides `q_r`, different regressor), `nmpc_q_e_v`, `nmpc_r_delta`, `nmpc_r_a_accel`, `nmpc_r_a_brake`, `nmpc_r_rate_delta`, `nmpc_r_rate_a`, `nmpc_terminal_scale`.
- `nmpc_steer_rate_anti_hunt_enabled`, `nmpc_anti_hunt_boost_max`.
- `nmpc_reversal_penalty_enabled`/`_boost_max`/`_k` (see the reversal-penalty section below).
- `nmpc_corner_rrate_blend_enabled`, `nmpc_corner_factor_k`, `nmpc_rrate_steer_straight`/`_corner` (blends `R_rate[0,0]` by current curvature; takes priority over `nmpc_steer_rate_anti_hunt_enabled` if both are set, use one or the other, not both).
- `nmpc_rrate_zone_enabled`/`_boost_straight`/`_ease_approach`/`_floor_corner` (the three-zone rate schedule, see "Three-zone rate schedule" below; note it also reads `nmpc_corner_factor_k`, so that field is NOT exclusive to the corner blend).
- `nmpc_rjerk_delta`/`_a` (second-difference input cost).
- `nmpc_rrate_stage_ramp_enabled`/`_near`.

**`NMPCParams`** (`nmpc_params.py`): all 20 fields NMPC-only by the file's own design (module docstring, lines 9-19); `mpc_core.py` never imports or reads this dataclass at all. Includes the master switch `use_nmpc` itself, horizon/solver settings (`nmpc_horizon`, `nmpc_sqp_iters`, `nmpc_solve_budget_ms`, `nmpc_rk_substeps`, `nmpc_jac_substeps`), SQP step control (`nmpc_trust_delta_rad`, `nmpc_trust_a`, `nmpc_backtrack_max`), the soft track constraint (`nmpc_track_halfwidth`, `nmpc_slack_weight`), curvature-reference construction (`nmpc_curvature_dense_step`, `nmpc_curvature_smooth_w`, `nmpc_kappa_clip`), `nmpc_alat_ceiling_enabled`, the three MPCC-inspired flags above, and solver tolerances (`nmpc_osqp_max_iter`, `nmpc_osqp_eps`).

**`settings.py`-only, no dataclass field** (rollout/plant configuration, not `MPCController`/`NMPCController` fields): `USE_PRECOMPUTED_SPEED_PROFILE`, `ENABLE_DYNAMIC_SPEED_CAP` and its `DYNAMIC_CAP_*` constants, `DELAY_STEPS`/`DELAY_JITTER_*`, `ALAT_CEILING_FLAT`/`_SLOPE`/`_INTERCEPT`.

Structural/solver constants (`NMPC_HORIZON`, `NMPC_SQP_ITERS`, etc.) live in `settings.py`'s "Nonlinear MPC (NMPC)" section, kept numerically identical to `NMPCParams` by hand. See `docs/tuning.md`'s NMPC section for the tuning surface, and `late_turn_in_investigation.md` Part 16 for the full research, implementation, and validation history, extended by §16.9-16.12 with the live-test results, the MPCC-feature live tests, and two DISTINCT standstill-steering bugs/fixes (§16.11: a manufactured tyre force at `v_x=0`; §16.12: the NMPC's own speed-tracking cost term leaking into steering, fixed by seeding the speed-target rise limiter's state from the car's actual speed on the first control tick instead of `None`).

Reproduce the offline closed-loop comparison with `python3 ros2/src/fsae_planning/control/fsae_control/test/nmpc_offline_check.py` (no ROS/FSDS session needed; the closed-loop section self-skips without an `fsae_MPCTest` sibling checkout) or `python -m tuner.nmpc_offline_check`.

**NMPC steering chatter while cornering** (magnitude hunting tick-to-tick, not a sign-flip reversal, distinct from both the reversal-penalty feature above and the two standstill bugs) is controlled by two settings. See `docs/logs/steering_chatter_investigation.md`'s "Resolution summary" for the full history, including everything ruled out along the way:

1. **`r_rate_delta=52.5`**: the steering-rate cost weight. At the much lower 2.8 the cost barely charges for rapid changes; 52.5 is what keeps `mean|d_steer|` and the sign-flip rate down.
2. **The tracked reference line matters independently of weights.** `centerline.csv`, not `raceline.csv`, holds steering reversals and both saturation and slew-limited ticks near zero: the raceline's own geometry demands grip the simulator's tyre model can't supply at that speed, which no controller-side weight can fix (see "Centreline beats raceline").

A small residual remains, clustered at corner **exits**: ~9.8 stutters/min, amplitude 1.5-3.2°; the signature is heading still unwinding (`|e_psi|` 13-16°) while lateral error is already small and shrinking. Not yet chased further at this amplitude; the next lever to try is `q_e_psi`/`q_r` at corner exit. Reproduce/extend with `python -m tuner.steering_chatter_check`.
