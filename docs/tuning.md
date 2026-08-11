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
| `r_a` | acceleration command effort | discourages large throttle/brake commands |
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
- `r_a` — because `a_cmd` is a *rate* term, its effort cost is paid
  immediately per tick while its benefit (removed speed error) accrues
  slowly; a naively "reasonable-looking" `r_a` can leave a large fraction of
  the car's real braking/accel authority unused. If braking/acceleration
  looks underused relative to what the car demonstrably sustains elsewhere
  in the same lap, `r_a` is the first thing to lower — but sweep around the
  current value rather than assuming further cuts keep helping; it has a
  measured local optimum, not a monotonic "lower is better" relationship.
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
re-testing this, re-run `tuner/ref_heading_limiter_suite_check.py` first.

---

## 4. Adaptive-gain overview

Every adaptive gain below is an *enable flag* (whether the mechanism runs)
plus a set of *shape constants* (the floors/ceilings/ramp sharpness of the
curve it applies). The flags decide whether a mechanism is active; the shape
constants decide how strongly it acts once active. Change shape constants
only for mechanisms that are enabled — tuning a disabled mechanism's shape
has no effect until you also flip its flag.

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
centred *and* not currently curving — i.e. specifically targets residual
steering chatter on a straight, not steering rate needed for cornering.

**Known constraints**: detects "corner ahead" only via current curvature
(reactive), the same signal §4.3 uses — it cannot anticipate a corner before
the car is already turning into it. Not validated against
`VALIDATION_SUITE`/recorded-map or any live log; treat as experimental.

### 4.3 Adaptive R-rate corner softening (`adaptive_r_rate_enable_in_corners`)

**Purpose**: continuously softens the steering-rate-of-change cost as
curvature rises, so the controller isn't over-penalised for the extra
steering rate a corner genuinely demands.

| Field | Purpose |
|---|---|
| `adaptive_r_rate_during_floor` | `R_rate[0,0]` floor driven by the car's CURRENT curvature |
| `adaptive_r_rate_entering_floor` | `R_rate[0,0]` floor driven by LOOKAHEAD curvature (acts before the car reaches the corner; deliberately shallower than the during-floor) |
| `adaptive_r_rate_k_entering` | ramp sharpness of the entering floor vs. lookahead curvature |

**Known constraints**: keep this enabled (True) for continuous softening —
disabling it does not remove softening symmetrically, it switches to a
discontinuous step at a fixed curvature threshold, which was tested and
caused severe lag specifically in corners (likely from the discontinuous
`R_rate[0,0]` jump spiking QP solver iterations / invalidating warm-starts
near the threshold). Lowering `adaptive_r_rate_during_floor` too far
reintroduces steering sign-reversal chatter mid-corner — this is the
mechanism that keeps that in check, not just a free knob.

### 4.4 Lookahead corner anticipation (`adaptive_q_lookahead_enabled`)

**Purpose**: scans a speed-scaled window of path ahead of the car (not just
the current point) for the sharpest upcoming curvature, and uses it to:
- boost `Q[0,0]`/`Q[2,2]` (lateral/heading error) *approaching* a corner, so
  steering authority commits before the car drifts off-line, not after;
- relax `Q[3,3]` (yaw rate) approaching a corner, so a straight-line yaw-rate
  penalty doesn't itself make turn-in feel slow;
- keep boosting `Q[2,2]` for a short distance *after* the corner (the exit
  boost), scaled by how sharp that corner was;
- on a genuinely clear straight, soften `Q[0,0]` and boost
  `Q[2,2]`/`Q[3,3]`/`R[0,0]` slightly (see §4.6/§4.7/§4.8).

All boosts are scaled by corner **demand** (`kappa_max_abs / kappa_limit(v)`
— how much of the car's available grip *at the current speed* this corner
needs), not raw curvature, when `adaptive_q_demand_normalised` is True. A
gradual sweeper taken fast and a tight corner taken slow can demand the same
thing; scoring by raw curvature alone made the configured boost ceilings
almost unreachable on real corners. Set `adaptive_q_demand_normalised=False`
to restore the old raw-curvature curve for A/B comparison.

| Field | Purpose |
|---|---|
| `adaptive_q_lookahead_time_s` | speed → lookahead distance conversion |
| `adaptive_q_lookahead_dist_min` / `_dist_max` | clamp floor/ceiling on the lookahead distance |
| `adaptive_q_lookahead_q_boost_max` | max `Q[0,0]` multiplier approaching a corner |
| `adaptive_q_lookahead_k_approach` | legacy (non-demand-normalised) approach ramp sharpness |
| `adaptive_q_lookahead_epsi_boost_max` | max `Q[2,2]` multiplier exiting a corner |
| `adaptive_q_lookahead_epsi_approach_boost_max` | max `Q[2,2]` multiplier approaching a corner |
| `adaptive_q_lookahead_k_epsi_approach` | legacy ramp sharpness (epsi approach) |
| `adaptive_q_lookahead_r_floor` | min `Q[3,3]` multiplier at high lookahead curvature |
| `adaptive_q_lookahead_k_r_relax` | legacy ramp sharpness (yaw-rate relax) |
| `adaptive_q_demand_half` | corner demand at which a demand-normalised boost reaches half its max — lower means boosts saturate on easier corners |

**Known constraints**: not validated against `VALIDATION_SUITE`/recorded-map
or any live log as a whole mechanism; treat changes here as experimental
until re-validated.

### 4.5 Exit-boost decay distance

| Field | Purpose |
|---|---|
| `adaptive_q_lookahead_exit_decay_dist` | exit-boost taper-distance floor (low-speed corners) |
| `adaptive_q_lookahead_exit_decay_time_s` | speed → taper distance conversion, mirrors `adaptive_q_lookahead_time_s` |
| `adaptive_q_lookahead_exit_decay_dist_max` | taper-distance clamp ceiling |
| `adaptive_q_lookahead_k_exit_norm` | normalises the exit boost by corner sharpness |
| `adaptive_q_lookahead_peak_hysteresis` | "cleared" threshold that re-arms peak-curvature detection for the next corner |

**Purpose of the speed scaling**: a *fixed* taper distance undershoots at
speed. Live telemetry showed lateral/heading error peaking well past the
geometric apex — the car is still sliding wide/yawing back through the exit
for 1.5–2.7s of travel — so a fixed short window can fully decay before the
tracking error is actually at its worst, leaving the exit boost inert
exactly when it would matter most. Scaling the taper distance by current
speed keeps the boost active through that window at any speed.

**Known constraints**: offline-validated only as of this writing — not yet
confirmed on the live car. Don't assume it transfers without a live check.

### 4.6 U-turn detector

| Field | Purpose |
|---|---|
| `adaptive_q_uturn_heading_thresh_rad` | accumulated heading change at which the detector engages |
| `adaptive_q_uturn_heading_sat_rad` | accumulated heading change at which the detector is fully saturated |
| `adaptive_q_uturn_ey_boost_max` | extra `Q[0,0]` multiplier at full U-turn severity |
| `adaptive_q_uturn_epsi_boost_max` | extra `Q[2,2]` multiplier at full U-turn severity |
| `adaptive_q_uturn_r_relax_floor` | `Q[3,3]` multiplier at full U-turn severity (relaxes yaw-rate penalty so the car can rotate faster) |

**Purpose**: a peak-curvature signal alone under-boosts long, gradual
U-turns — a wide U-turn's peak curvature can look like a mild bend even
though it demands a huge total rotation. This detector instead measures
*accumulated* heading change over the lookahead window, so severity ramps
smoothly between the threshold and saturation bounds. Ordinary corners fall
under the threshold and score nothing, so this cannot disturb
already-working sudden-corner behavior.

### 4.7 Straight-line adjustments

Three independent mechanisms that only activate when the lookahead window is
genuinely clear of curvature (a straight), each fading back to baseline
sharply as a corner enters the window:

| Field | Purpose |
|---|---|
| `adaptive_q_straight_ey_floor` / `_k` | reduces `Q[0,0]` (lateral cost) on a clear straight — nothing to track hard against — with a sharp fade so full authority returns the moment a corner appears |
| `adaptive_q_straight_epsi_boost_max` | boosts `Q[2,2]` (heading error) on a straight to keep the car pointed straight |
| `adaptive_q_straight_r_boost_max` | boosts `Q[3,3]` (yaw rate) on a straight to damp yaw wander |
| `adaptive_q_straight_k` | shared fade-out sharpness for the epsi/yaw-rate boosts above |
| `steer_effort_straight_boost_max` / `_k` (flag: `steer_effort_straight_boost_enabled`) | makes steering effort `R[0,0]` (not its rate — see §4.2) expensive on a clear straight |
| `anti_hunt_boost_max` | ceiling multiplier for the steer-rate anti-hunt boost (§4.2) |

**Known constraint on `adaptive_q_straight_epsi_boost_max`**: keep this
deliberately small. A strong straight-line heading weight amplifies the
QP's reaction to ordinary heading noise, which can itself introduce
oscillation — the opposite of this mechanism's purpose.

**Known constraints (general)**: `steer_effort_straight_boost_enabled` and
`anti_hunt_boost_max` are not validated against `VALIDATION_SUITE` or any
live log; treat as experimental.

### 4.8 FSDS lateral-acceleration ceiling law

| Field | Purpose |
|---|---|
| `alat_ceiling_flat` | low-speed floor of FSDS's fitted sustained lateral-acceleration ceiling, `a_lat_max(v) = max(FLAT, SLOPE·|v| + INTERCEPT)` |
| `alat_ceiling_slope` | ceiling-law slope vs. speed |
| `alat_ceiling_intercept` | ceiling-law intercept |

**This is a measured property of the simulator, not a free tuning knob.**
See CLAUDE.md's "dynamically-enforced lateral-acceleration ceiling" section
for the measurement. It feeds `_corner_demand`, which is how every
lookahead boost above knows whether an upcoming corner is actually holdable
at the current speed — every demand-normalised adaptive gain depends on this
being accurate. Must stay in sync with `model/vehicle_physics.py`'s
`alat_ceiling_at()`. Don't hand-tune these three values for "feel" — if the
ceiling law is wrong, re-measure it with `ros2/run_steering_sysid.sh` /
`ros2/run_steering_step.sh`, don't guess.

---

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
