# MPC Tuning Reference

This is the single canonical reference for tuning the MPC: every weight,
adaptive-gain shape constant, and feature flag, what it does, how to adjust
it, and anything specific to keep in mind when changing it. Other docs
(`architecture.md`, `developer_guide.md`, `planning_control_sync.md`) link
here instead of repeating this material — check those docs only for things
tuning doesn't cover (system architecture, how to run the tuner, live/offline
resync procedure).

**Source of truth for exact numbers**: `settings.py` (offline) and
`ros2/src/fsae_planning/control/fsae_control/fsae_control/mpc_params.py`
(live). This doc explains *what each one does and how to tune it*; it
deliberately does not restate every current numeric value, since those drift
as tuning continues and a second copy of the numbers here would just be one
more place to fall out of sync. See CLAUDE.md's parity rule: any change here
must be applied to **both** `settings.py` and `mpc_params.py`.

**Two controllers, two tuning surfaces.** Sections 1-4.4/4.9 below tune the
default linear time-varying MPC (LTV-QP). The NMPC (`use_nmpc=true`, see
[§4.5d](#45d-nonlinear-mpc-use_nmpc))
shares the LTV-QP's base weights (section 1) as its starting point but is
entirely unaffected by all of section 4's adaptive machinery, and can be
retuned independently via its own `nmpc_q_*`/`nmpc_r_*` override fields
without touching the LTV-QP. See
[architecture.md](architecture.md#second-controller-nonlinear-mpc-use_nmpc)
for what the NMPC is and why it exists.

## How to use this doc

1. Find the parameter or feature you want to change below.
2. Read its purpose and the "how to tune it" guidance.
3. Check the "known constraints" column — some values have a hard floor/
   ceiling discovered by prior testing. Don't re-cross a boundary already
   shown to make things worse without a specific reason to re-test it.
4. Change the value in **both** `settings.py` and `mpc_params.py`.
5. Re-validate: `python -m tuner.recorded_map_rollout` (offline, ~2 min) at
   minimum; for anything touching adaptive gains or delay handling, also run
   `VALIDATION_SUITE`. Never trust an offline-only score for a change that
   affects saturation or heading error — see CLAUDE.md's "offline sim does
   not yet fully predict the car" section.

---

## 1. Core cost weights (`Q_diag` / `R_diag` / `R_rate_diag`)

These three vectors are the base "driving personality" — how much the
controller cares about tracking accuracy vs. control effort vs. smoothness.
Every adaptive gain below is a *multiplier* applied on top of these values,
not a replacement for them.

| Field | Penalises | Purpose |
|---|---|---|
| `q_e_y` | lateral deviation from centreline | primary tracking-accuracy term |
| `q_e_yd` | rate of change of lateral deviation | damps lateral oscillation |
| `q_e_psi` | heading error vs. path tangent | keeps the car pointed along the path |
| `q_r` | yaw rate | damps spin/yaw oscillation |
| `q_e_v` | speed error (car speed − target speed) | tracks the speed profile |
| `r_delta` | steering command effort | discourages large steering angles |
| `r_a_accel` | acceleration command effort, `a_cmd >= 0` | discourages large throttle commands |
| `r_a_brake` | acceleration command effort, `a_cmd < 0` | discourages large braking commands |
| `r_rate_delta` | steering rate of change | discourages jerky steering |
| `r_rate_a` | acceleration rate of change | discourages jerky throttle/brake |
| `terminal_q_scale` | final predicted state in the horizon | extra weight on where the plan ends up |

**How to tune**: change one value at a time by no more than 20–30%, then
re-run the tuner/validation suite. These interact — a change to one can be
masked or amplified by another, so isolate.

**Known constraints**:
- `terminal_q_scale` — 1.0 (no-op) is the only value ever validated against
  the current `Q_diag`/`R_diag`/`R_rate_diag` set. Changing it changes the
  effective tuning of everything else; re-validate fully rather than
  treating it as an independent knob.
- `r_a_accel` / `r_a_brake` — because `a_cmd` is a *rate* term, its effort
  cost is paid immediately per tick while its benefit (removed speed error)
  accrues slowly; a naively "reasonable-looking" weight can leave a large
  fraction of the car's real braking/accel authority unused. If
  braking/acceleration looks underused relative to what the car demonstrably
  sustains elsewhere in the same lap, the corresponding one of these is the
  first thing to lower — but sweep around the current value rather than
  assuming further cuts keep helping; a single shared weight is not a
  monotonic "lower is better" relationship, it has a measured local optimum
  (see `planning_control_sync.md`'s "Accel/brake effort weight split" for
  the history behind why this is two independent weights rather than one).
  Lowering ONLY `r_a_brake` (leaving `r_a_accel` fixed) is the more targeted
  lever if the specific symptom is weak braking without also making the car
  over-eager to accelerate. These two are actively live-tuned; check
  `mpc_params.py` for the current values rather than trusting a number
  quoted here.
- `q_e_v` — has more effect on the corner-approach phase specifically than
  on whole-run averages, since a large fraction of any lap is spent on
  straights where speed error is naturally small. When re-tuning this,
  compare corner-approach-phase metrics (ticks with high corner demand), not
  just whole-run RMSE — whole-run numbers can hide a real improvement or
  regression.

---

## 2. Delay compensation

| Field | Purpose |
|---|---|
| `delay_compensation_enabled` | roll the MPC's belief about its own state forward through pending commands (`predict_ahead()`) before optimizing, so the plan accounts for the actuation lag between deciding and acting |
| `max_delay_compensation_steps` | caps how far forward `predict_ahead()` is allowed to roll |
| `predict_epsi_clip` | small-angle bound used inside `predict_ahead()`'s heading-error prediction |
| `pose_age_lp_alpha` | low-pass filter coefficient smoothing the estimated pose age each tick |
| `n_delay_hysteresis` | deadband either side of an `n_delay` bin boundary, to stop the compensation depth flip-flopping tick-to-tick near a boundary |

`DELAY_STEPS`/`DELAY_JITTER_STEPS`/`DELAY_JITTER_SEED` (offline-only,
simulator-side) model how much actuation lag exists and how much the
controller's *belief* about that lag is allowed to be wrong, independent of
the true plant delay. This is what makes an offline score meaningful: with
zero jitter, the offline tuner is optimizing against an easier problem than
the real car (a real control loop's period jitters, so its lag estimate is
never perfect).

**How to tune**: `DELAY_STEPS` should reflect a realistic actuation lag for
the current hardware. `DELAY_JITTER_STEPS` should be set from a measured
live control-loop jitter (loop-period standard deviation, converted to
steps) — don't leave it at 0 unless deliberately testing the idealized case.

**Known constraints**: `n_delay_hysteresis` exists specifically to prevent
oscillation at a bin boundary — don't remove it without confirming that
oscillation doesn't reappear.

---

## 3. Reference-heading rate limit

| Field | Purpose |
|---|---|
| `ref_heading_rate_limit_enabled` | caps how fast the *tracked reference heading* itself is allowed to change per tick, independent of the car's own yaw dynamics |
| `ref_heading_rise_rate_deg_s` | the cap, in deg/s, when enabled |

This exists because the planner's reference heading can swing faster than
either the sim or the real car can ever physically yaw — see CLAUDE.md's
"reference-heading lead" discussion. The limiter only ever slows down how
fast the *target* is allowed to change; it never reverses a correction's
sign.

**Known constraints**: **do not re-enable without a specific reason to
re-test.** This was tried live and found to make saturation/heading-error
*worse*, not better, despite being a reasonable-sounding hypothesis — see
`docs/logs/sim_to_real_investigation.md` for the investigation. It remains
in the codebase as a validated-off feature, not a half-finished one.
Lowering `ref_heading_rise_rate_deg_s` much below ~85 deg/s risks holding
the reference back so hard that a fast, tight slalom goes off-track — if
re-testing this, re-run `tuner/checks/ref_heading_limiter_suite_check.py` first.

---

## 4. Adaptive-gain overview

Every adaptive gain below is an *enable flag* (whether the mechanism runs)
plus a set of *shape constants* (the floors/ceilings/ramp sharpness of the
curve it applies). The flags decide whether a mechanism is active; the shape
constants decide how strongly it acts once active. Change shape constants
only for mechanisms that are enabled — tuning a disabled mechanism's shape
has no effect until you also flip its flag.

**Two generations, same as `architecture.md`'s "Adaptive gain scheduling"
section.** §4.1-4.3 below describe mechanisms still active today.
§4.4-§4.8 describe the forward-scanning "lookahead" family that the
corner-factor rewrite deleted wholesale — kept collapsed for the
tuning history and the reasoning behind what replaced it, not because
there's anything left in that family to tune. §4.3b documents the
replacement: the corner-factor scheduler this whole family was replaced
with. §4.9-§4.10 describe two further mechanisms that were tried and
disabled/removed independently of the corner-factor rewrite, for their own
separate reasons.

### 4.1 Adaptive Q-scaling near centreline (`adaptive_q_scaling_enabled`)

**Purpose**: relax the lateral-error penalty (`Q[0,0]`) when the car is
already close to the centreline, to reduce small-error "hunting" — a
controller that penalises tiny lateral errors as heavily as large ones will
often make small, unnecessary corrections that look like steering chatter.

**Known constraints**: not currently reproduced by the offline recorded-map
rollout as tuned (steering-reversal rate rises *with* `|e_y|` offline, the
opposite of the live trend) — this may be a live-only symptom. Treat offline
validation of this specific mechanism with caution; the live behavior is the
one it was built to fix.

### 4.2 Steering-rate anti-hunt boost (`steer_rate_anti_hunt_enabled`)

**Purpose**: extra penalty on steering-rate-of-change (`R_rate[0,0]`), on
top of the corner-softening in §4.3, but only when the car is already
centred, not currently curving, *and* well-aligned — i.e. specifically
targets residual steering chatter on a genuine straight, not steering rate
needed for cornering.

| Field | Purpose |
|---|---|
| `anti_hunt_boost_max` | ceiling multiplier when the car is straight/centred/aligned |

**Known constraints**: detects "currently curving"/"centred"/"aligned"
only via current curvature/`e_y`/`e_psi` (reactive) — these three alone
cannot anticipate a corner before the car is already turning into it. Not
validated against `VALIDATION_SUITE`/recorded-map or any live log as a
whole mechanism; treat as experimental. (This section used to also carry
`anti_hunt_k_lookahead`, a lookahead-curvature fade gate — removed along
with the rest of the lookahead family in the corner-factor rewrite; see §4.4.)

### 4.3 Adaptive R-rate corner softening (`adaptive_r_rate_enable_in_corners`)

**Purpose**: continuously softens the steering-rate-of-change cost as
current curvature rises, so the controller isn't over-penalised for the
extra steering rate a corner genuinely demands.

| Field | Purpose |
|---|---|
| `adaptive_r_rate_during_floor` | `R_rate[0,0]` floor driven by the car's CURRENT curvature |

**Known constraints**: keep this enabled (True) for continuous softening —
disabling it does not remove softening symmetrically, it switches to a
discontinuous step at a fixed curvature threshold, which was tested and
caused severe lag specifically in corners (likely from the discontinuous
`R_rate[0,0]` jump spiking QP solver iterations / invalidating warm-starts
near the threshold). Lowering `adaptive_r_rate_during_floor` too far
reintroduces steering sign-reversal chatter mid-corner — this is the
mechanism that keeps that in check, not just a free knob. (This section
used to also carry a second, lookahead-driven floor —
`adaptive_r_rate_entering_floor`/`_k_entering` — removed along with the
rest of the lookahead family in the corner-factor rewrite; see §4.4.)

### 4.3b Corner-factor scheduler

**Purpose**: replaces the entire forward-scanning lookahead family in §4.4
below with one continuous, CURRENT-curvature-only fraction
(`corner_frac`, 0=straight → 1=full corner) that blends four weights
between a straight endpoint and a corner endpoint, plus an independent
low-speed boost and an always-on heading-error-driven accel/brake
asymmetry. See `architecture.md`'s "Corner-factor scheduler" section for
the formulas and the reasoning behind each piece; this section is the
tuning-surface reference.

| Field | Purpose |
|---|---|
| `corner_factor_k` | sharpness of the `corner_factor` curve vs. CURRENT `\|kappa\|` — higher means the straight/corner transition happens over a narrower curvature range |
| `q_ey_straight` / `q_ey_corner` | `Q[0,0]` (lateral error) blend endpoints |
| `q_epsi_straight` / `q_epsi_corner` | `Q[2,2]` (heading error) blend endpoints |
| `q_r_straight` / `q_r_corner` | `Q[3,3]` (yaw rate) blend endpoints — `_corner` is normally LOWER than `_straight` (relaxes in-corner) |
| `rrate_steer_straight` / `rrate_steer_corner` | `R_rate[0,0]` (steering rate) blend endpoints — `_corner` is normally LOWER than `_straight` (relaxes in-corner) |
| `r_steer_corner_mid` | `R[0,0]` (steering effort) blend target at full corner — a MIDDLE value between the straight weight and the corner-relaxed extreme used by the other weights above, so turn-in isn't made maximally cheap right when saturation risk is highest |
| `low_speed_corner_boost_v_half` | speed at which the low-speed corner boost has decayed to half its `max_extra` |
| `low_speed_corner_boost_max_extra` | max extra `corner_frac` added at `car_speed=0`, fully inside a corner — gated multiplicatively on `corner_factor`, so this is an exact no-op on a straight regardless of speed |
| `epsi_ra_half_rad` | `\|e_psi\|` at which the accel/brake asymmetry reaches half its max effect |
| `epsi_ra_accel_boost_max` | max multiplier on `r_a_accel` at large `\|e_psi\|` (more expensive, discourages accelerating through a heading error) |
| `epsi_ra_brake_floor` | min multiplier on `r_a_brake` at large `\|e_psi\|` (cheaper, frees up braking authority) |

**Known constraints**: not validated against `VALIDATION_SUITE`/recorded-map
or any live log as a whole mechanism; treat as experimental, same status
the lookahead family it replaced carried before its own removal. The
`epsi_ra_*` asymmetry is independent of `corner_frac` and always active —
tune it separately from the four corner-blend weights above.

### 4.4 Historical: lookahead corner anticipation (removed)

None of these fields exist on `MPCParams` anymore. Full description,
reasoning, and the field-by-field purpose table:
[`removed_mechanisms.md` §3](removed_mechanisms.md#3-lookahead-corner-anticipation)
(anticipation boosts) and
[§4](removed_mechanisms.md#4-demand-normalisation) (demand normalisation).
Kept for the tuning history and the elimination reasoning that's the direct
motivation for the nonlinear MPC (§5) — **this whole mechanism only
reweighted the COST of an existing tracking error, it could not manufacture
one.**

### 4.5 Exit-boost decay distance

Part of the same removed family —
[`removed_mechanisms.md` §3](removed_mechanisms.md#3-lookahead-corner-anticipation)
covers the exit-boost decay alongside the approach-side boosts it paired
with.

### 4.5b Precomputed corner segmentation (`use_precomputed_corner_map`) — REMOVED

**This mechanism no longer exists.** The corner_factor rewrite deleted
`use_precomputed_corner_map`, `CornerMap`, and `_segment_corners` along with
the rest of the lookahead adaptive-gain family. Full description:
[`removed_mechanisms.md` §7](removed_mechanisms.md#7-precomputed-corner-segmentation-cornermap).
§4.5d's "everything in section 4 is inactive under NMPC" is a different,
independent statement (about what the _current_ `MPCParams` mechanisms do)
and is unaffected by this removal.

### 4.5c Precomputed shaped heading-lead profile (`use_precomputed_heading_profile`) — LIVE-ONLY

| Field | Purpose |
|---|---|
| `use_precomputed_heading_profile` | node-level launch parameter (NOT an `MPCParams` field) — use `raceline.csv`'s shaped `psi_target` column as `e_psi`'s reference instead of the geometric path tangent. Default `false`. Only has an effect when `use_precomputed_path` is ALSO true, and requires a re-exported `raceline.csv` with the `psi_target` column (an older 4-column file degrades to the geometric tangent, a no-op). |
| `HEADING_LEAD_AUTHORITY_FRAC` | (offline constant, `tuner/tools/raceline_optimizer.py`, NOT an `MPCParams` field — set at export time, baked into the CSV) fraction of the car's achievable yaw rate to pre-spend as heading lead. Default `0.5`. Re-export the CSV to change it. |
| `SLIP_LIMIT_RAD` | (offline constant, `tuner/tools/raceline_optimizer.py`) diagnostic-only rear-slip-angle bound used to flag stations, unvalidated placeholder (5°). |

**Purpose**: precompute a heading reference that already leads the
geometric path tangent by however much yaw is achievable at the planned
speed, so `e_psi` carries a real, current error approaching a corner
instead of relying on Q/R reweighting of an error that doesn't exist yet.
See `planning_control_sync.md`'s "Precomputed shaped heading-lead profile"
section for the full design and why this avoids curvature-forcing's
wrong-direction-transient failure.

**Status: implemented, offline-validated (no-op-when-off confirmed, profile
shape sanity-checked), live-tested with a HIGH-VARIANCE, inconclusive
result** — four live runs at the default `authority_frac=0.5` ranged from
the single best run recorded all session (zero steering saturation) to
some of the worst, against a baseline that itself varied nearly as much
run-to-run; not enough runs to call it a net win or a net loss. Currently
shipped **OFF** (`USE_PRECOMPUTED_HEADING_PROFILE=false` in
`ros2/launch_all.sh`) pending more data, not because it's confirmed not to
help — check that file's shortlist before assuming either default.
`comp_test_map_3` has few true straights, so the lead is active
almost everywhere on that track at the default `authority_frac` — a
plausible explanation for the variance (the lead can't selectively target
the approach phase on this track) that further runs haven't yet confirmed
or ruled out. See `planning_control_sync.md`'s caveat and
`late_turn_in_investigation.md` Parts 12-13 for the full run-by-run data
before drawing conclusions from any single result.

### 4.6 Historical: U-turn detector and straight-line adjustments (removed)

None of these fields exist on `MPCParams` anymore — part of the same
lookahead family removed in §4.4. Full description and field-by-field
purpose tables:
[`removed_mechanisms.md` §5](removed_mechanisms.md#5-u-turn-detection)
(U-turn detector) and
[§6](removed_mechanisms.md#6-straight-line-adjustments) (straight-line
adjustments).

### 4.8 Historical: FSDS lateral-acceleration ceiling law as a lookahead input (removed)

The ceiling law's three fields (`alat_ceiling_flat`/`_slope`/`_intercept`)
no longer exist on `MPCParams` — **the law itself is not gone**, it moved
to `nmpc_core.py`'s `_Plant` (hardcoded there, since only the NMPC path
uses it now) and `model/vehicle_physics.py`'s `alat_ceiling_at()` — see §5.
This is a measured property of the simulator, not a free tuning knob; if
it's ever suspected wrong, re-measure with `ros2/run_steering_sysid.sh` /
`ros2/run_steering_step.sh`, don't guess. Full history:
[`removed_mechanisms.md` §10](removed_mechanisms.md#10-fsds-lateral-acceleration-ceiling-as-a-lookahead-input).

### 4.9 Historical: low-speed steering-rate boost (removed)

No fields remain on either side (`mpc_params.py`, `settings.py`) — this
mechanism, which made `R_rate[0,0]` more expensive at low speed to damp a
post-corner-exit wobble, was tried, live-tested, disabled, and has since
been removed entirely along with the rest of the lookahead gain-scheduling
family. It gated purely on speed with no curvature/lookahead signal, so it
could not distinguish "post-exit overcorrection at low speed" (the case it
was built for) from "turn-in at low speed" (also low speed, but wanted) —
it suppressed both identically. Full incident and the tuned values it used
(kept on record in case a future curvature-gated rework wants a starting
point): [`removed_mechanisms.md` §9](removed_mechanisms.md#9-low-speed-steering-rate-boost-removed).

### 4.10 Historical: curvature forcing term — removed, structurally unsound

`curvature_forcing_enabled`/`curvature_forcing_gain` and the code behind
them no longer exist. This was the one attempt in the whole removed family
that tried to fix the *actual* structural limit (injecting curvature into
the predicted dynamics, not just reweighting cost) — and the reason it
still failed (the solver can defer a forcing term added to its own
dynamics recursion, producing a wrong-direction transient at every gain
tried) is the direct motivation for the nonlinear MPC in §5. Full gain
sweep and mechanism:
[`removed_mechanisms.md` §8](removed_mechanisms.md#8-curvature-forcing-the-closest-attempt-and-why-it-still-failed).

---

### 4.5d Nonlinear MPC (`use_nmpc`)

`use_nmpc=true` swaps `mpc_core.MPCController` (linear time-varying QP) for
`nmpc_core.NMPCController` (Frenet-frame nonlinear MPC, Gauss-Newton SQP).
Default **false**. This repo now has its own offline port too
(`controller/nmpc_optimiser.py`, selected by `settings.USE_NMPC`, same
default false) — see `planning_control_sync.md`'s "Nonlinear MPC
(`use_nmpc`)" section for the full description, and
`tuner/nmpc_offline_check.py` for the reproducible validation suite
(`python -m tuner.nmpc_offline_check`, no ROS/FSDS session needed).

**Tuning implications, which is what this doc is for:**

- **Everything in section 4 above is INACTIVE** when `use_nmpc=true` —
  including §4.1-4.3/4.3b's still-current mechanisms, not just the
  historical §4.4/4.6/4.8/4.10 ones. Every adaptive multiplier, gate, floor,
  boost, and the corner-factor scheduler itself exist to synthesise corner
  anticipation a curvature-blind prediction cannot produce. The NMPC's model
  carries `kappa(s)` directly, so none of section 4 is applied at all.
  Retuning any of it has no effect on an NMPC run.
- **The tuning surface is therefore ~6 numbers, not ~56**: the base weights of
  section 1 (`q_e_y`, `q_e_yd`, `q_e_psi`, `q_r`, `q_e_v`, `r_delta`,
  `r_a_accel`/`r_a_brake`, `r_rate_delta`/`r_rate_a`, `terminal_q_scale`), which
  the NMPC reads from the SAME `MPCParams` the QP uses — so the current tuned
  set is the starting point, not a blank slate.
- **`q_r` is the one weight whose MEANING changes.** In the QP it weights
  absolute yaw rate `r`; in the NMPC it weights heading-error rate
  `r - kappa*s_dot`, which is zero for a car correctly tracking a corner rather
  than proportional to how hard the corner is. Penalising absolute `r` in a
  curvature-aware model would fight cornering. Same number, different
  regressor: **re-sweep this one first**.
- **To retune the NMPC without touching the LTV-QP's own weights**, use the
  `nmpc_q_*` / `nmpc_r_*` override fields, which live IN `MPCParams`
  itself (not a separate `NMPCParams`), alongside the
  base weights they inherit from at their `-1.0` sentinel — see
  `mpc_params.py`'s own "NMPC weight overrides" section). Both the base
  weights and these overrides carry the same `settings.py`
  (`NMPC_Q_E_Y`, ...) parity obligation as every other `MPCParams` field.
  `ros2/launch_all.sh` carries a commented-out shortlist of the likely ones.
- **Structural knobs and where their values came from** (all measured, see
  `late_turn_in_investigation.md` Part 16 §16.7): `nmpc_horizon=20` (1.0 s —
  measured BETTER than 35, because the prediction model is optimistic and the
  mismatch compounds; N=35 gave the fastest lap but the worst tracking),
  `nmpc_sqp_iters=1` (measured better AND ~2x cheaper than 2),
  `nmpc_solve_budget_ms=25` (hard stop; ships the best feasible iterate rather
  than overrunning the 50 ms tick), `nmpc_rk_substeps=2` (needed because
  `tau_a=0.02 s` is stiff against `dt=0.05 s`), `nmpc_jac_substeps=1` (only
  sets the SQP step direction, never the prediction).
- **`nmpc_alat_ceiling_enabled=true` is not optional on FSDS.** With it false
  the linear-tyre prediction believes it can hold any corner at any speed and
  the car spins mid-lap offline. Set false only for real-vehicle work, mirroring
  `VehicleParams.alat_ceiling_enabled`.
- `nmpc_track_halfwidth=3.5` / `nmpc_slack_weight=10000` are copies of
  `_build_qp`'s existing soft-track literals, and
  `nmpc_curvature_dense_step=0.5` / `nmpc_curvature_smooth_w=3` are
  `control_utils.curvature_speed()`'s existing denoise precedent — none of the
  four is a new constant to tune.
- **Three MPCC-inspired flags, all NMPC-only**:
  `nmpc_spline_reference_enabled` (default true — not really "tunable", a
  numerical-quality fix; set false only to A/B against the old moving-average
  path), `nmpc_horizon_speed_profile_enabled` and
  `nmpc_friction_circle_enabled` (both default false, genuine unvalidated
  experiments — do not enable for a live run without an offline A/B first).
  See `planning_control_sync.md`'s "Three MPCC-inspired additions" subsection
  for the mechanism and "Which settings affect which controller" for the
  complete field-by-field controller-scope map (which settings are LTV-QP-only,
  NMPC-only, or shared).

## 5. Dynamic speed cap — disabled, do not re-enable without re-diagnosis

`enable_dynamic_speed_cap` / `dynamic_cap_a_lat_max` / `dynamic_cap_safety`
layer a real-time curvature-lookahead speed cap under the precomputed speed
profile. **Built and live-tested; made overall driving worse despite
improving its own target metric** (lateral-acceleration-over-ceiling ratio
improved sharply, but steering saturation and heading error both got worse,
and the composite score regressed). The root cause of that regression was
never diagnosed. `ros2/launch_all.sh` disables this at runtime regardless of
the code-level default — see its MPC tuning shortlist. Do not re-enable
without first diagnosing why it regressed saturation/heading-error, not just
re-tuning `dynamic_cap_a_lat_max`/`dynamic_cap_safety` and hoping. Full
writeup: `docs/logs/sim_to_real_investigation.md`.

---

## 6. Scoring: `METRIC_SCALES` and `SCORE_WEIGHTS`

These two arrays define what the tuner is actually trying to optimize.
`sim/scoring.py` is the single source of truth for the composite-score
formula; the live copy at
`ros2/src/fsae_planning/control/fsae_control/fsae_control/scoring.py` must
stay a verbatim numeric copy (see CLAUDE.md's "Scoring parity" section).

- **`METRIC_SCALES`** — a typical/reference magnitude for each of the 13
  scored metrics, used to normalise them onto a comparable scale before
  weighting. Without this, a metric's real influence on the score is
  `weight × typical magnitude`, not `weight` — and the 12–13 metrics have
  wildly different natural magnitudes (e.g. `steering_reversal_rms` ~0.007
  vs. `accel_rms` ~1.3), which without normalisation collapses the score to
  being effectively single-objective (dominated almost entirely by tracking
  RMSE and peak lateral error). Change a `METRIC_SCALES` entry only if that
  metric's typical magnitude has genuinely shifted (e.g. after a plant or
  planner change) — not to change how much it matters, which is what
  `SCORE_WEIGHTS` is for.
- **`SCORE_WEIGHTS`** — how much each of the 13 normalised metrics
  contributes to the final composite score (lower is better). This is
  literally the tuner's definition of "good driving." Must sum to ~1.0 so a
  run scoring exactly at every metric's reference scale scores 1.0 before
  bonuses/penalties — if you change one weight, take the offsetting change
  from another to preserve the sum. Typical adjustment: change a weight by
  roughly 20–30% of its own value at a time, then re-tune and compare.

The 13 metrics, in order: `rmse`, `yaw_rms`, `smooth_rms`, `steer_rms`,
`accel_rms`, `max_steering`, `steering_sat_ratio`, `jerk_rms`,
`max_yaw_rate`, `steering_reversal_rms`, `peak_lateral_error`, `speed_rmse`,
`accel_reversal_rms`. See `sim/scoring.py`'s module docstring for exactly
what each one measures and why the two reversal-RMS metrics are
magnitude-weighted rather than flat reversal counts.

---

## 7. Simulator-only fidelity settings

Not MPC weights, but directly affect whether an offline tuning result will
transfer to the real car — get these wrong and a weight set can look
excellent offline while being fragile live.

| Setting | Purpose | Tuning guidance |
|---|---|---|
| `N_HORIZON` | planning horizon length, in 0.05s steps | must exactly match the live controller's horizon length or tuned weights won't behave the same on the car. Change by 5 steps at a time; longer horizon smooths anticipation but each step's compute cost roughly squares. |
| `DELAY_STEPS` | fixed actuation lag modelled, in steps | set to a realistic lag for the current hardware; 0 = idealized/no lag |
| `DELAY_JITTER_STEPS` | how much the controller's *belief* about lag is allowed to be wrong (std dev, in steps) | set from a measured live control-loop jitter; leaving at 0 makes the tuner optimize against an easier problem than reality — a classic way for a weight set to score well offline and still wobble live |
| `SLAM_NOISE_ENABLED` + `SLAM_POS_JITTER_STD`/`SLAM_YAW_JITTER_STD` | simulate real SLAM pose jitter instead of FSDS's perfect ground-truth pose | currently OFF — re-calibrate against a current live log's reversal rate before turning back on; do not use this to try to reproduce steering chatter caused by the steering slew limit or delay-estimation jitter, since pose noise is a different mechanism and won't reproduce that specific symptom |

---

## 8. Where NOT to look for tuning guidance

The following docs contain tuning-*adjacent* material but are not the
canonical tuning reference — they're kept focused on their own scope, and
link here for anything about what to change and why:

- `architecture.md` — system architecture and module reference; points here
  for weight/gain guidance.
- `developer_guide.md` — how to run the tuner and simulator; points here for
  what the tuner is actually optimizing.
- `planning_control_sync.md` — the live/offline field-mapping table (which
  `settings.py` constant matches which `MPCParams` field) and the
  upstream-resync procedure; not a tuning-values guide.
- `docs/logs/sim_to_real_investigation.md` — the full chronological
  investigation history behind several of the "known constraints" above.
  Read it for *how* a constraint was discovered; this doc is where you look
  up *what* the constraint is without reading the whole investigation.
- `junior_project_mpc_docs.md` — a standalone, self-contained onboarding
  wiki page; intentionally still explains tuning from scratch rather than
  linking here, since it's meant to be readable without any other doc open.
