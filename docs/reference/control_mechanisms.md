# Control Mechanisms Reference

Per-mechanism reference for the control stack: what each one does, why it is
shaped the way it is, and what to be careful of when changing it.

**Scope.** This covers mechanisms that exist in the code today. For:

- which knob to turn and to what value → `docs/tuning.md`
- how a subsystem is built → `docs/architecture.md`
- a mechanism that no longer exists → `docs/reference/removed_mechanisms.md`
- the reference path and speed profile → `docs/reference/reference_path_and_speed.md`

## Corner-factor scheduler — what replaced the lookahead gain-scheduling family

**The current mechanism** is `_corner_factor`/`_low_speed_corner_boost`/
`_blend`: a single continuous CURRENT-curvature-only fraction blending four
`Q`/`R_rate` weights between a straight endpoint and a corner endpoint, plus
an always-on, independent heading-error-driven accel/brake asymmetry
(`epsi_ra_*`). See `architecture.md`'s "Corner-factor scheduler" section for
the full formulas and mechanism, and `tuning.md` §4.3b for the tuning-surface
reference — not repeated here to avoid duplicating either.

It exists on both sides: `fsae_MPCTest/controller/model_utils.py`
(`_corner_factor`/`_low_speed_corner_boost`/`_blend` plus `settings.py`'s
matching constants) and `fsds_simulator/`'s copies of `mpc_core.py`/
`mpc_params.py`/`fsae_params.yaml`, byte-identical to the live files (see the
parity table above).

**What it replaced, and why that matters if you are tempted to re-add it.**
The previous design was ~15 interacting functions that scanned FORWARD along
the path every tick (`lookahead_curvature_profile` → a scalar `kappa_max_abs`,
the peak curvature within a speed-scaled lookahead window) and reweighted
`Q`/`R`/`R_rate` in anticipation of a corner not yet reached: approach/exit
`Q[0,0]`/`Q[2,2]` boosts, `Q[3,3]`/`R[0,0]` relaxations, demand normalisation
(`_corner_demand`/`_alat_ceiling_at`), a U-turn detector, straight-line
`Q`/`R[0,0]` adjustments, and a `CornerMap`/`_segment_corners`
precomputed-path fast path for the same scan. See `mpc_core.py`'s own
"Lookahead gain-scheduling family: removed" comment for the exhaustive
function-name list, and [`removed_mechanisms.md`](removed_mechanisms.md) for
the mechanism-level detail of each (`tuning.md` §4.4/§4.6/§4.8/§4.10 and
`architecture.md`'s "Historical" subsection both link there rather than
restating it).

The family was removed because this MPC formulation already predicts state
error against the reference at each future horizon step. Reweighting TODAY's
(usually near-zero) cost based on what a forward scan finds ahead does not
change what the horizon predicts once the car actually gets there — the
mechanism was reweighting a cost that mostly was not there yet, not
manufacturing anticipation. Repeated piecemeal retuning of it never produced
a clear net win. This is the same structural argument, one level up, that
motivates the nonlinear MPC below: reweighting an existing error's cost is not
the same as making the *prediction itself* see the road bend.

## Soft steering-reversal penalty (`reversal_penalty_*` / `nmpc_reversal_penalty_*`)

**Default off, experimental on both controllers.** Boosts `R_rate[0,0]`
(steering rate-of-change cost) whenever LAST tick's steering command was
already close to zero — the one state a sign-flip reversal must pass
through. A reversal itself can't be penalised directly inside one solve
(it depends on THIS tick's own decision, the thing being optimised, which
would make the cost non-convex); this penalises the precondition instead,
via a saturating curve `1/(1 + k*|u_prev_steer|)` keyed on `u_prev` (a known
constant by solve time, not the solve's own variable). Same shape as
`_steer_rate_anti_hunt`/`_corner_factor`, and composes multiplicatively with
whichever of those is active rather than replacing it.

**Implemented in:**
- **Live**: `mpc_core.py`'s `_reversal_penalty_boost` (LTV-QP), mirrored in
  `nmpc_core.py` (NMPC path, tracked through a running `rrate_steer_current`
  value alongside the corner-blend/anti-hunt branches so the penalty
  compounds onto whichever of those ran rather than the pre-branch base).
  Fields: `mpc_params.py`'s `reversal_penalty_enabled`/`_boost_max`/`_k`
  (LTV-QP-only) and `nmpc_reversal_penalty_enabled`/`_boost_max`/`_k`
  (NMPC-only overrides, `-1.0` = inherit the LTV-QP value).
- **Offline**: `controller/model_utils.py`'s `reversal_penalty_boost` (same
  function, both controllers call it), wired into `sim/rollout_core.py`'s
  LTV-QP and NMPC branches; `settings.py`'s `REVERSAL_PENALTY_*` /
  `NMPC_REVERSAL_PENALTY_*` constants (same `-1.0`-inherit convention).

**Validated on the LTV-QP path only.** Offline A/B on `comp_test_map_3`
showed a genuine ~20% reversal-count reduction at negligible cost, no DNF.
The NMPC path's own reversal penalty regressed the composite score
offline (reversals dropped only ~5.6% while score worsened) — leave
`nmpc_reversal_penalty_enabled` off unless a live A/B says otherwise; it is
wired and functional, just not currently worth enabling.

## Slew-rate limit (`du_max`): on both sides, at 180 deg/s

The hard per-step slew-rate constraint on `[delta_cmd, a_cmd]` must exist on
both sides. It was once live-only, with `controller/optimiser.py` carrying no
such constraint at all, so the offline tuner was optimising against a plant
that could change steering arbitrarily fast while the real car was clamped —
weights tuned offline did not transfer faithfully, independent of any weight
choice. Keep both sides constrained.

How it is formulated:

1. `init_parameterized_mpc()` / `solve_mpc()` in `controller/optimiser.py`
   take an optional `du_max`, mirroring the live formulation. It is baked
   into the cached QP the same way `u_min`/`u_max` are, so it participates in
   the same cache-staleness check (passing a different `du_max` rebuilds the
   problem instead of silently reusing stale constraints). Only the step-0
   constraint against `u_prev` is omitted offline — step 0 is already anchored
   through `weighted_u_prev` in the cost, and `u_prev` isn't a constraint
   parameter in the cached formulation.
2. The limit is **180 deg/s**, expressed as a *rate* (`max_steer_rate * DT`)
   rather than a fixed per-step angle, so its physical meaning survives a
   change of `DT`.

**Why 180 deg/s and not the original 80.** Live telemetry
(`mpc_standalone_control_1785976976.csv`) showed the steering command pinned
exactly on an 80 deg/s limit for **41% of all control steps**, reversing sign
at ~8 Hz — a rate-limit-induced limit cycle, not a weight-tuning problem.
Inverting the logged yaw rate through the kinematic bicycle
(`delta = atan(L*r/v)`) put the *achieved* roadwheel rate at p99 ≈ 138 deg/s
and max ≈ 218 deg/s, so the real actuator is at least ~200 deg/s. 180 deg/s
sits just under that measured floor.

**Why the offline sim didn't catch it.** With the same weights and the same
80 deg/s limit, an offline rollout on `PATH_MICRO_SLALOM` hits the limit on
only **0.5%** of steps versus the live car's 41%, and composite scores at
80 vs 180 deg/s differ by <0.002 across `PATH_S_BEND`, `PATH_SUDDEN_TURN` and
`PATH_SPIRAL`. The offline sim uses a fixed `DELAY_STEPS = 1` and a smooth
synthetic path, so it simply never enters the saturated regime the live car
lives in (jittery measured delay, replanned/noisy perception path). Do not
read "the offline score barely moved" as "the change doesn't matter" — the
constraint is there to stop the live controller sitting on its limit.

The true FSDS steering rate is **not recoverable from this repo** (the PhysX
vehicle setup lives in git-LFS `.uasset` binaries), so 180 deg/s is a measured
lower-bound estimate, not a datasheet figure. Refine it via system-ID on the
running sim and update both sides together.

## MPC prediction horizon: frozen target speed

`sim/rollout_core.py`'s (and `mpc_core.py`'s) MPC formulation bakes
`desired_speed` into `x0[4]` (`e_v`) as a single scalar frozen for the whole
prediction horizon. This is an architectural characteristic of the
formulation, not a bug. See `README.md`'s state-vector section (search "e_v's
target speed is frozen for the whole horizon") for the full explanation; not
repeated here to avoid duplication.

## Accel/brake effort weight split

The MPC's acceleration-effort cost is split into two independent weights
rather than one symmetric `R_diag[1]` scalar: `r_a_accel` penalizes
`a_cmd >= 0` and `r_a_brake` penalizes `a_cmd < 0`, via
`R_a_accel * sum(pos(a_cmd)²) + R_a_brake * sum(neg(a_cmd)²)` in the QP cost
(`cp.pos(u[1,:])`/`cp.neg(u[1,:])`). This lets braking and acceleration
effort be tuned independently instead of forcing both to move together
whenever `R_diag[1]` is retuned.

**Implemented in:**
- **Live**: `mpc_core.py`'s `_build_qp`/`_solve_qp` (`r_a_accel_param`/
  `r_a_brake_param` `cp.Parameter`s), `mpc_params.py`'s `r_a_accel`/
  `r_a_brake` fields, `fsae_params.yaml`'s `controller.r_a_accel`/
  `controller.r_a_brake`, `launch_all.sh`'s `MPC_R_A_ACCEL`/`MPC_R_A_BRAKE`
  shortlist entries.
- **Offline**: `controller/optimiser.py`'s `init_parameterized_mpc`/
  `solve_mpc` (same `cp.pos`/`cp.neg` split; `r_a_accel`/`r_a_brake` kwargs
  default to `R[1,1]` when omitted, for backward compatibility),
  `settings.py`'s `R_A_ACCEL`/`R_A_BRAKE` (read by `sim/rollout_core.py`'s
  `solve_mpc()` call). `R_diag[1]`/`self.R[1,1]` remain nominal/reporting
  values only — no adaptive gain (`_adaptive_R_scaling`, `_adaptive_R_rate`,
  etc.) touches index 1 of `R`/`R_rate` anywhere in this codebase, so the
  split composes cleanly with the rest of the adaptive-gain machinery.

**Re-check `mpc_params.py`'s `r_a_accel`/`r_a_brake` and `settings.py`'s
`R_A_ACCEL`/`R_A_BRAKE` for the current numeric defaults before relying on
any value quoted elsewhere** — these two remain the most frequently
live-retuned weights in the whole `MPCParams` set. Keep `settings.py`
synced to `mpc_params.py`'s live values per this document's parity rule whenever
either changes.

For the full diagnosis history (the corner-entry-too-hot symptom that
motivated the split, the slack-variable design considered and rejected in
favor of the `cp.pos`/`cp.neg` rewrite, and the live-tuning value
trajectory), see `docs/logs/late_turn_in_investigation.md`'s "Part 0
(background) — how the accel/brake effort split (`r_a_accel`/`r_a_brake`)
came about" and `docs/logs/sim_to_real_investigation.md` §59 for the
preceding single-scalar `r_a` cut this split superseded.

## Lookahead corner-anticipation window (approach side)

`adaptive_q_lookahead_dist_max` is `25.0` on both sides (`fsae_params.yaml`,
`fsds_simulator` mirror), matching `adaptive_q_lookahead_exit_decay_dist_max`
(also `25.0`) — the approach-side and exit-side ceilings are symmetric.
`sim/rollout_core.py`'s hardcoded literal is `(1.13, 3.0, 25.0)`
(`clip(v * adaptive_q_lookahead_time_s, 3, 25)`), kept as a plain literal
rather than sourced from a new `settings.py` constant, since `fsae_MPCTest`
and `fsae_planning` must never share an import dependency in either
direction.

`adaptive_q_lookahead` (§4.4 in `tuning.md`) only reweights `Q[0,0]`/`Q[2,2]`
on an *existing* tracking error — with `e_y ≈ e_psi ≈ 0` on approach, a wider
window lets the boost detect the corner's curvature sooner, but multiplying a
still-near-zero error by a bigger number is still near-zero. This is the same
"reweighting cannot manufacture an error" ceiling documented in the
curvature-forcing postmortem elsewhere in this doc and in
`junior_project_mpc_docs.md`: the boosts (and downstream steering commitment)
can only trigger sooner and larger once real error/curvature already appears
inside the window, not before.

A genuine fix for before-any-error anticipation on a precomputed path would
need to change what the reference/error is measured against — e.g. compute
`e_psi` against a lookahead point on the path rather than the nearest point —
not reweight costs on the current-position error. No design or
implementation exists for this. See `docs/logs/late_turn_in_investigation.md`
Part 14 for why even the 25m ceiling is still short of some corner-approach
gaps on real tracks, and for the candidate fixes considered.

## Post-solve output smoothing — removed

**A post-solve low-pass filter on the SOLVED steering command
(`filtered += alpha*(raw - filtered)`, then
`steering = (1-w)*raw + w*filtered`), distinct from every `Q`/`R`/`R_rate`
gain-scheduling mechanism elsewhere in this codebase (which reshape the QP's
COST before solving, fresh each tick with no memory) — this one persisted
`filtered` tick to tick, adding genuine lag.**

Removed: it never improved on the QP's own steering-rate cost
(`r_rate_delta`), which attacks the same jitter at its source instead of
filtering an already-chattery command after the fact (see `tuning.md`'s
tuning-order table — `r_rate_delta=52.5` and `NMPC_RJERK_DELTA=150.0` are
the levers that actually worked). Shipped default was always `false`; the
mechanism (node params, `peak_kappa_ahead()`, the offline mirror in
`sim/rollout_core.py`/`settings.py`, and the `launch_all.sh`/launch-file
wiring) has been deleted from both the live and offline sides rather than
left as unused dead code.

## Straight-line lateral-error snap-back sharpness

`_lookahead_straight_lateral_reduce` softens `Q[0,0]` (lateral-error cost) to
`ey_floor=0.7` on a clear straight, then fades back to full weight as
curvature enters the lookahead window, at a rate set by
`adaptive_q_straight_ey_k` (currently `8.0`, matching `adaptive_q_straight_k`
— the shared fade sharpness for the `Q[2,2]`/`Q[3,3]` straight-line boosts).
The straight-line relaxation benefit itself (`ey_floor=0.7`) is independent
of this and unchanged by it — `adaptive_q_straight_ey_k` only controls how
quickly full lateral weight is restored once a corner is detected ahead.

Do not raise `ey_floor` as a way to address slow snap-back — that reduces
the straight-line-hunting benefit this mechanism exists for and changes
behaviour even far from any corner, which is the wrong lever. If
straight-line hunting reappears, `adaptive_q_straight_ey_k` and the
`ey_floor`/`adaptive_q_straight_ey_k` field comments in
`mpc_params.py`/`settings.py` are the first things to check.

## Gradual-corner accel oscillation is genuine track geometry, not a bug

Through mild/gradual turns (e.g. S-curves/chicanes), `v_desired` and `a_cmd`
can legitimately oscillate — rising to a local peak, dipping, rising again —
because the precomputed speed profile (`speed_profile.csv`) is tracking a
genuine sign change in the path's own curvature (a left-hand bend
straightening briefly before curving right into the next bend), not
adaptive-gain misbehaviour or spurious jitter. The `kappa_max_abs`-driven
lookahead correctly speeds the car up through the brief straight and slows
it back down anticipating the next corner.

Before treating this pattern as a bug, cross-reference the car's actual
`v_desired`/`a_cmd` against the track's own `speed_profile.csv` and the raw
path's Menger curvature at the same position — a genuine varying-radius
feature (sign change in curvature, not just magnitude) confirms the
oscillation is correct tracking, not noise.

**Do not "fix" this via `R_rate_diag[1]`** (acceleration-rate-of-change
cost) or similar damping — that makes the MPC slower to respond to a real,
upcoming tightening corner, trading a correctly-anticipated slowdown for a
late, harder one. If oscillation on a specific track needs addressing, the
correct levers are the speed profile's own generation parameters
(`a_lat_max`, scan window in `sim/speed_profile.py`'s
`compute_speed_profile()`) or the raw path geometry itself (smoothing an
actually-spurious kink) — never the live adaptive gains. Confirm the
geometry is genuinely spurious (not real track shape) before touching
either lever; see `late_turn_in_investigation.md`'s "Gradual-corner accel
oscillation" section for the full worked example of how to distinguish the
two.

## Dynamic speed cap

`control_utils.dynamic_speed_cap()` is a thin wrapper over
`curvature_speed()` (see that function's own docstring for the scan/
braking-distance mechanism) that runs *underneath* the precomputed oracle
speed profile: whenever a track is mapped, the controller target speed is
`min(precomputed_speed_at(...), dynamic_speed_cap(...))`. The oracle profile
(`precomputed_speed_at()`, used when `USE_PRECOMPUTED_SPEED_PROFILE=True` /
`map_path` is set) is a static, position-indexed lookup with no notion of
the car's actual current speed relative to remaining braking distance; the
dynamic cap exists to catch the case where live tracking has drifted ahead
of the oracle plan (e.g. exiting the previous corner faster than expected)
and pull the target speed down before a corner is reached, never up.

It uses its own, tighter constants than the live-`curvature_speed()`-only
branch's numeric-parity defaults: `DYNAMIC_CAP_A_LAT_MAX=3.2` and
`DYNAMIC_CAP_SAFETY=0.9` (vs. `a_lat_max=4.75`/`safety=1.0` elsewhere), so it
engages a little before the oracle profile would actually be violated.
Downstream, `tracking_error_speed_gate()` and `SPEED_TARGET_RISE_RATE` apply
exactly as before — the cap only changes what `v_curv` feeds into that
existing pipeline. It has no effect when no track is mapped, and is
byte-identical to pre-cap behaviour when disabled.

Controlled by `enable_dynamic_speed_cap` (ROS param) /
`ENABLE_DYNAMIC_SPEED_CAP` (`settings.py`) — code default `True`, but
**`ros2/launch_all.sh`'s MPC tuning shortlist currently overrides this to
`false`**, so a plain `launch_all.sh` run has the cap off; check that
shortlist rather than assuming the code-level default is what actually
drives. Do not re-enable for a live run without first understanding why it
regressed steering saturation and heading error offline (see
`docs/logs/late_turn_in_investigation.md`'s "Dynamic speed cap" addendum for
the measured before/after and the live-test result) — the a_lat-ceiling
metric it targets improved, but the metrics that matter more got worse, for
reasons never diagnosed.

## Precomputed shaped heading-lead profile

A derived, offline-only third pass over an already-optimized raceline that
replaces the geometric heading reference (`atan2` of the path tangent) with
a SHAPED one: at each waypoint, the heading target leads the geometric
tangent by however much yaw the car can physically achieve between here and
the upcoming bend at that station's already-planned speed. This changes
`e_psi` at `k=0` itself — before the QP ever runs — rather than adding a
future-deviation cost the QP is free to satisfy however is cheapest (the
failure mode of both curvature-forcing and a cost-target shift, both
rejected). It also does not use a fixed lookahead distance (which saturates
steering to full lock at ~8m of lead on a realistic corner-entry ramp);
instead the lead is scaled by achievable yaw rate at the station's planned
speed, which naturally decays it to ~0 once a corner's constant-curvature
section begins.

**Where it lives:**
- Offline: `fsae_MPCTest/tuner/tools/raceline_optimizer.py` — `max_yaw_rate`,
  `build_shaped_heading_profile`, `check_slip`, run in `export()` after
  `optimize_raceline()`'s path+speed have converged. Does not feed back into
  path/speed optimization (kept as a separate derived pass to avoid
  invalidating that loop's own tuned constants).
- CSV format: `x,y,psi,psi_target,v_target` (5 columns), backward-compatible
  with the old 4-column format — `control_utils._load_profile_csv` sets
  `psi_target = psi` for any 4-column file.
- Live: `mpc_core.py`'s `MPCController.set_heading_profile(psi_target)`
  substitutes the shaped value for `path_yaw` at `base_idx`, only for
  `e_psi`'s reference (`e_y`'s projection keeps using the geometric
  tangent). Loader: `control_utils.load_path_heading_profile_csv`.
- Toggle: `use_precomputed_heading_profile`, a node-level launch parameter
  (not an `MPCParams` field), exposed as `USE_PRECOMPUTED_HEADING_PROFILE`
  in `ros2/launch_all.sh`. Current default: `false` on both nodes, both
  launch files, and in `launch_all.sh`.
- `use_precomputed_heading_profile` has no effect when `use_nmpc` is active
  — the NMPC model already carries curvature directly.

**`check_slip`'s `SLIP_LIMIT_RAD` (5°) is an unvalidated placeholder** —
diagnostic-only, does not fail the export or reshape anything. Do not treat
it as authoritative until it has a real measurement behind it.

**Standing warning:** do not extend this mechanism from a k=0-only lead to
a full per-horizon-step reference — that reproduces curvature-forcing's
same wrong-direction-dip trap (see
`docs/logs/late_turn_in_investigation.md` Part 15).

For the full derivation, the rejected fixed-lookahead-distance and
cost-target-shift alternatives, the `comp_test_map_3`-specific
near-everywhere-lead caveat, and both rounds of live-test results
(an initial "worse" read followed by a high-variance correction across
more runs), see `docs/logs/late_turn_in_investigation.md` Parts 7-13.

## Nonlinear MPC (`use_nmpc`) — a second controller

`ros2/src/fsae_planning/control/fsae_control/fsae_control/nmpc_core.py`'s
`NMPCController` is a Frenet-frame **nonlinear** MPC (Gauss-Newton SQP, a
condensed dense QP subproblem solved by OSQP, real-time-iteration style — one
SQP iteration per tick, warm-started from the previous tick). It has an
independent offline port, `controller/nmpc_optimiser.py`'s `NMPCController`
(same model, same SQP/OSQP scheme, not an import — the two repos cannot
import each other), wired into `sim/rollout_core.py`'s `run_core_rollout()`
behind `settings.USE_NMPC` (default false). It is selected live by the node
parameter `use_nmpc` (default **false**) and replaces `MPCController`
wholesale when true; `mpc_core.py` is untouched either way.

### Why it exists

`MPCController._discrete_model` is the bicycle model in error coordinates
with the reference frame's own rotation dropped — it is missing
`e_psi_dot = r - kappa(s) * s_dot` from its `Ad`/`Bd`, so with `e_y = e_psi =
0` the QP's rollout predicts staying at zero error forever and no weighting
can produce turn-in before real error exists (`MPCController` measurably
commands 0.000 deg at 8 dead-on-line states approaching a known bend). The
Frenet formulation makes `kappa` a function of the **state** `s` (driven by
the car's own predicted motion) rather than a horizon-indexed exogenous
schedule, so the anticipation obligation is not schedulable and the solver
cannot pre-pay it early the way the curvature-forcing and heading-lead
workarounds do. Full derivation, the formulation survey (including why MPCC's
progress-maximising formulation was not adopted wholesale), and the falsification
methodology are in `late_turn_in_investigation.md` Part 16.

### Model and what it reuses

States `[s, e_y, e_psi, v_x, v_y, r, delta_act, a_act]`, inputs `[delta_cmd,
a_cmd]`. Every vehicle constant (`lf` 0.70, `lr` 0.85, `m` 255, `Iz` 150,
`Cf`/`Cr`, `tau_delta` 0.08, `tau_a` 0.02, `MAX_STEER_RAD`, `MAX_ACCEL`,
`MAX_BRAKE`, `du_max`) and the kinematic/dynamic blend band (1.0–2.5 m/s)
come from `MPCController.__init__` unchanged — no new physical constant.
Cost weights come from the same `MPCParams` instance the LTV-QP uses. Three
deliberate differences from the LTV-QP:

1. **`q_r` weights heading-error RATE** (`r - kappa*s_dot`), not absolute yaw
   rate — same slot, different regressor from the LTV-QP's `q_r`.
2. **`e_y`/`e_psi` are measured against the smoothed reference**, not the raw
   segment tangent (bounded by the same 1.5 m smoothing window as
   `control_utils.curvature_speed()`'s `dense_step=0.5`/`w=3`).
3. **FSDS's measured `a_lat` ceiling is inside the prediction**, as a smooth
   `tanh` saturation of predicted tyre forces, reusing the same numeric law
   (flat/slope/intercept = 7.5/0.47/2.46) as `mpc_core._alat_ceiling_at` /
   `model/vehicle_physics.alat_ceiling_at`, hardcoded as class-level defaults
   on `nmpc_core.py`'s `_Plant` rather than read from `MPCParams`.
   `nmpc_alat_ceiling_enabled=false` recovers the unconstrained plant,
   mirroring `VehicleParams.alat_ceiling_enabled`.

Tyre lateral force (`F_yf`/`F_yr`, from `alpha_f = atan((v_y + lf*r)/v_safe) -
delta_act`) is scaled by the same kinematic/dynamic `blend` factor used
elsewhere in the model, applied immediately after the force is computed and
after the `alat_ceiling` soft saturation — without this scaling the slip-angle
floor at low `v_x` lets steering alone manufacture lateral force with zero
forward speed. This applies in all three copies: `nmpc_core.py` and
`controller/nmpc_optimiser.py`'s `_f`, `_f_scalar`, and its
`nmpc_friction_circle_enabled`-only `_tyre_forces()` helper.

### What is inactive when `use_nmpc=true`

- The entire adaptive gain schedule (lookahead approach/exit boosts,
  yaw-rate relax, straight boosts, centred softening, U-turn detector) — no
  `m_*` telemetry columns are written for these.
- `use_precomputed_heading_profile` has no effect (one startup log line says
  so) — the NMPC's curvature model already carries what that flag
  approximates.
- `curvature_forcing_enabled`, `ref_heading_rate_limit_enabled` are
  LTV-QP-only.
- `use_precomputed_corner_map` no longer exists on either `use_nmpc` setting
  (removed by the corner_factor rewrite; see `architecture.md`'s
  "Precomputed corner segmentation" note).

**Exception: `nmpc_steer_rate_anti_hunt_enabled`** (default `False`,
`mpc_params.py`'s `nmpc_steer_rate_anti_hunt_enabled`/
`nmpc_anti_hunt_boost_max`) is an independent NMPC-only opt-in, not inherited
from the LTV-QP's `steer_rate_anti_hunt_enabled` — unlike the rest of the
adaptive-gain family, it only makes steering-rate more expensive when the
current state is already centred/aligned/uncurving, the opposite direction
from anticipation. Reuses `mpc_core._steer_rate_anti_hunt` verbatim on the
live side and `model_utils.steer_rate_anti_hunt` offline. Writes the existing
shared `m_Rrate_antihunt` telemetry column when on. See `mpc_params.py`'s
field comment and `nmpc_core.py`'s module docstring ("WHAT THIS CONTROLLER
DELIBERATELY DOES NOT DO") for the reasoning.

`use_precomputed_path` / `use_precomputed_speed` / `enable_dynamic_speed_cap`
/ delay compensation (`delay_compensation_enabled`, `pose_age_lp_alpha`,
`n_delay_hysteresis`, `max_delay_compensation_steps`) all work exactly as
under the LTV-QP; the delay rollforward uses the nonlinear model instead of
`predict_ahead()`'s linearisation.

### Three MPCC-inspired additions

Assessed against Alexander Liniger's Model Predictive Contouring Control
(`https://github.com/alexliniger/MPCC`). MPCC's headline idea — progress `θ`
as a free variable the solver maximises — was not adopted: it reintroduces
the "exogenous, schedulable future obligation" failure mode this NMPC's
`kappa(s)`-as-state design exists to avoid. Three narrower ideas were kept,
all NMPC-only and implemented identically in `nmpc_core.py` (live) and
`controller/nmpc_optimiser.py` (offline):

1. **Spline-based path reference** — `nmpc_spline_reference_enabled` /
   `NMPC_SPLINE_REFERENCE_ENABLED`, default **true**. `PathReference` fits
   `x(s)`/`y(s)` as independent `scipy.interpolate.CubicSpline` objects over
   arc length and derives `kappa(s)`/`psi_ref(s)` analytically
   (`kappa = (x'y'' - y'x'') / (x'^2+y'^2)^1.5`), replacing the old
   dense-resample + moving-average + finite-difference pipeline. A strict
   numerical-quality improvement with no new solver coupling — the old
   moving-average path is kept behind the flag for A/B if needed. Directly
   targets the open "centreline curvature spikes" defect described below.
2. **Horizon speed profile** — `nmpc_horizon_speed_profile_enabled` /
   `NMPC_HORIZON_SPEED_PROFILE_ENABLED`, default **false**. Would sample a
   precomputed per-lap speed profile at each horizon stage's own predicted
   arc length via `PathReference.v_ref_at(s)`, instead of holding `v_ref`
   constant across the horizon. **Do not enable without first bounding how
   far ahead along `s` the sampled `v_ref` may rise** (or an equivalent
   per-stage clamp) — summing `v_x - v_ref(s_k)` across all horizon stages
   lets a high `v_ref` at a later stage offset a low `v_ref` at an earlier
   one within the same solve, defeating the non-schedulability property this
   feature was meant to inherit from `kappa(s)`. See
   `late_turn_in_investigation.md` Part 16 §16.9 for the live-test evidence
   behind this warning.
3. **Friction-circle hard constraint** — `nmpc_friction_circle_enabled` /
   `NMPC_FRICTION_CIRCLE_ENABLED`, default **false**. Would add a hard
   `|F_yf|, |F_yr| <= F_max` bound (additional to, not replacing, the
   existing soft `tanh` saturation) with `F_max` derived from the same
   measured ceiling law. **Do not enable without first fixing
   `telemetry_logger.py`'s `NMPC_COLUMNS`** (missing `nmpc_fyf_max_abs`/
   `nmpc_fyr_max_abs`) **and re-deriving a looser `F_max`** — the hard bound
   has no slack variable, and ordinary cornering geometry on this track
   conflicts with `F_max = m * ceiling(v_x) / 2` per axle under completely
   normal driving, not just extreme conditions. See
   `late_turn_in_investigation.md` Part 16 §16.9 for the failure evidence.

Neither feature 2 nor 3 has offline A/B numbers; reproduce a comparison with
`python -m tuner.nmpc_offline_check` once one exists.

### Dependencies

`osqp` only — already a documented requirement of `mpc_core` via cvxpy
(`fsae_control/package.xml`). CasADi/acados are **not** used by the shipped
code (not installable into the ROS interpreter on Ubuntu 24.04 without
`--break-system-packages`); the SQP and its Jacobians are numpy-only. CasADi
was used once, from a private `--target` install, purely to cross-check the
optimum against IPOPT.

### Which settings affect which controller

Every `MPCParams`/`NMPCParams` field carries an explicit
`metadata["controller"]` tag (`"both"`, `"ltv_qp_only"`, or `"nmpc_only"`) —
see `mpc_params.py`/`nmpc_params.py` for the authoritative per-field value.
`settings.py` mirrors the same three-way classification as a
`[LTV-QP only]`/`[NMPC only]`/`[shared]` comment prefix on each constant, and
`ros2/launch_all.sh` tags its shortlists the same way.

**Shared base weights** (`mpc_params.py:47-67`, read by both controllers
through `nmpc_core.py`'s `_pick(override, inherited)` at lines 859-877): `q_e_y`,
`q_e_yd`, `q_e_psi`, `q_r` (meaning differs — see above), `q_e_v`, `r_delta`,
`r_a_accel`, `r_a_brake`, `r_rate_delta`, `r_rate_a`, `terminal_q_scale`. A
launch with every `nmpc_q_*`/`nmpc_r_*` override left at its `-1.0` sentinel
starts the NMPC from the LTV-QP's own tuned set exactly.

**Shared delay-compensation fields** (`mpc_params.py:73, 80, 84-85`):
`delay_compensation_enabled`, `max_delay_compensation_steps`,
`pose_age_lp_alpha`, `n_delay_hysteresis` gate/shape both sides identically
(mechanism differs: NMPC rolls `x0` through the nonlinear model instead of
`predict_ahead()`'s linearisation). `predict_epsi_clip`
(`mpc_params.py:81`) is LTV-QP only — specific to `predict_ahead()`'s linear
rollforward.

**LTV-QP-only, the adaptive-gain-schedule fields** (`mpc_params.py:70-72, 74,
77, 81, 88, 91, 101-161`): `adaptive_q_scaling_enabled`,
`steer_rate_anti_hunt_enabled`, `adaptive_r_rate_enable_in_corners`,
`ref_heading_rate_limit_enabled`, `ref_heading_rise_rate_deg_s`,
`adaptive_r_rate_during_floor`, `anti_hunt_boost_max`, `corner_factor_k`,
`q_ey_straight`/`q_ey_corner`, `q_epsi_straight`/`q_epsi_corner`,
`q_r_straight`/`q_r_corner`, `rrate_steer_straight`/`rrate_steer_corner`,
`r_steer_corner_mid`, `low_speed_corner_boost_v_half`/`_max_extra`,
`epsi_ra_half_rad`/`_accel_boost_max`/`_brake_floor`, `reversal_penalty_enabled`/
`_boost_max`/`_k` (soft steering-reversal penalty, see its own section below)
— none have any read site in `nmpc_core.py`.

**NMPC-only overrides** (`mpc_params.py:189-205, 222-223, 238-240, 253-256`):
`nmpc_q_e_y`, `nmpc_q_e_yd`, `nmpc_q_e_psi`, `nmpc_q_epsi_dot` (overrides
`q_r`, different regressor), `nmpc_q_e_v`, `nmpc_r_delta`, `nmpc_r_a_accel`,
`nmpc_r_a_brake`, `nmpc_r_rate_delta`, `nmpc_r_rate_a`, `nmpc_terminal_scale`,
`nmpc_steer_rate_anti_hunt_enabled`, `nmpc_anti_hunt_boost_max`,
`nmpc_reversal_penalty_enabled`/`_boost_max`/`_k` (see the reversal-penalty
section below), `nmpc_corner_rrate_blend_enabled`, `nmpc_corner_factor_k`,
`nmpc_rrate_steer_straight`/`_corner` (blends `R_rate[0,0]` by current
curvature; takes priority over `nmpc_steer_rate_anti_hunt_enabled` if both
are set — use one or the other, not both),
`nmpc_rrate_zone_enabled`/`_boost_straight`/`_ease_approach`/`_floor_corner`
(the three-zone rate schedule — see "Three-zone rate schedule" below; note it
also reads `nmpc_corner_factor_k`, so that field is NOT exclusive to the
corner blend),
`nmpc_rjerk_delta`/`_a` (second-difference input cost),
`nmpc_rrate_stage_ramp_enabled`/`_near` — each read solely by
`nmpc_core.py`'s `_pick()` calls; `mpc_core.py` never references any of them.

**`NMPCParams`** (`nmpc_params.py`) — all 20 fields NMPC-only by the file's
own design (module docstring, lines 9-19); `mpc_core.py` never imports or
reads this dataclass at all. Includes the master switch `use_nmpc` itself,
horizon/solver settings (`nmpc_horizon`, `nmpc_sqp_iters`,
`nmpc_solve_budget_ms`, `nmpc_rk_substeps`, `nmpc_jac_substeps`), SQP step
control (`nmpc_trust_delta_rad`, `nmpc_trust_a`, `nmpc_backtrack_max`), the
soft track constraint (`nmpc_track_halfwidth`, `nmpc_slack_weight`),
curvature-reference construction (`nmpc_curvature_dense_step`,
`nmpc_curvature_smooth_w`, `nmpc_kappa_clip`), `nmpc_alat_ceiling_enabled`,
the three MPCC-inspired flags above, and solver tolerances
(`nmpc_osqp_max_iter`, `nmpc_osqp_eps`).

**`settings.py`-only, no dataclass field** (rollout/plant configuration, not
`MPCController`/`NMPCController` fields): `USE_PRECOMPUTED_SPEED_PROFILE`,
`ENABLE_DYNAMIC_SPEED_CAP` and its `DYNAMIC_CAP_*` constants,
`DELAY_STEPS`/`DELAY_JITTER_*`, `ALAT_CEILING_FLAT`/`_SLOPE`/`_INTERCEPT`.

Structural/solver constants (`NMPC_HORIZON`, `NMPC_SQP_ITERS`, etc.) live in
`settings.py`'s "Nonlinear MPC (NMPC)" section, kept numerically identical to
`NMPCParams` by hand. See `docs/tuning.md`'s NMPC section for the tuning
surface, and `late_turn_in_investigation.md` Part 16 for the full research,
implementation, and validation history, extended by §16.9-16.12 with the
live-test results, the MPCC-feature live tests, and two DISTINCT
standstill-steering bugs/fixes (§16.11: a manufactured tyre force at
`v_x=0`; §16.12: the NMPC's own speed-tracking cost term leaking into
steering, fixed by seeding the speed-target rise limiter's state from the
car's actual speed on the first control tick instead of `None`).
Reproduce the offline closed-loop comparison with
`python3 ros2/src/fsae_planning/control/fsae_control/test/nmpc_offline_check.py`
(no ROS/FSDS session needed; the closed-loop section self-skips without an
`fsae_MPCTest` sibling checkout) or `python -m tuner.nmpc_offline_check`.

**Open, separate issue: NMPC steering chatter while cornering** (magnitude
hunting tick-to-tick, not a sign-flip reversal — distinct from both the
reversal-penalty feature above and the two standstill bugs). Root cause not
yet found; confirmed NMPC-specific (~7x noisier than the LTV-QP under
matched conditions) and reproducible offline with zero sensor noise, so it
is not purely a live sensor-noise artifact, though a live-only contributor
also appears to stack on top of the offline-reproducible baseline. See
`docs/logs/steering_chatter_investigation.md` for the full investigation —
what's been ruled out (single cost-weight retunes, SQP iteration count,
trust-region size in isolation, FD-Jacobian coarseness, reference-spline
noise, Frenet-projection instability, terminal cost scale) and what hasn't
been tried yet, before repeating any of that work. Reproduce/extend with
`python -m tuner.steering_chatter_check`.
