# Simulator Fidelity and Known Defects

Where the offline simulator and the live car diverge, and which divergences
are understood.

**Plain version:** the offline simulator is a good enough model to tune
against, but it is not the car. This document records every place the two are
known to disagree, how large each gap is, and which are explained versus still
open. Read it before trusting an offline-only result.

The headline caution: the live car saturates its steering far more often than
the offline rollout and carries roughly twice the heading error, on the same
map with the same gains. Full history in
`docs/logs/sim_to_real_investigation.md`.

## Simulator fidelity limits (what FSDS does NOT model)

Read this before trusting any offline or FSDS result as a prediction of
real-car behaviour. These are the known ways both simulators are *easier* than
reality.

| Aspect | FSDS / this rollout | Real car | Modelled? |
|---|---|---|---|
| **Localisation accuracy** | Perfect. `sim_perception` copies ground-truth `/fsds/testing_only/odom` verbatim onto `/fsae/slam/car_position`. No noise, no drift, no estimation lag. | ZED visual odometry + `cone_mapper` SLAM: jitters, drifts, lags. | Offline, via `SLAM_NOISE_ENABLED` (**default off** — adds jitter + slow drift to the estimated pose when enabled; see `settings.py`'s own comment for the current rationale). |
| **Cone map** | Latched *oracle* map of exact cone positions, cropped to a forward window + radius. Only **range** is limited. | Real detections: false positives/negatives, position error, colour confusion, range-dependent noise. | Partly, via `CONE_NOISE_ENABLED` (**default off**, position jitter only — false positives/negatives/range-dependence remain unmodelled). |
| **Pose rate** | 20 Hz (`pose_rate`), matching the controller. Was 10 Hz — see the section below. | Bounded by the perception pipeline's real throughput. | Live-only concern; the offline rollout always uses a fresh pose per step. |
| **Actuation delay** | Fixed `DELAY_STEPS`, compensated exactly by `predict_ahead()`. | Variable, estimated from a timestamp, never exactly known. | Partly — `DELAY_JITTER_STEPS` perturbs the controller's *belief* about the lag, and `POSE_HOLD_*`/`PoseFeedHold` (see [Measurement rate](#measurement-rate-pose-must-keep-up-with-the-controller) below) separately models a live fault where the pose feed stalls for a few ticks. |
| **Steering slew** | Hard `du_max`, now on both sides. | Real rack limit, measured ≥ ~200 deg/s. | Yes, at 180 deg/s (a measured lower bound, not a datasheet figure). |
| **Tyre/plant model** | Bicycle model with estimated `lf`/`lr`/`Iz`/`Cf`/`Cr` — the true values live in git-LFS `.uasset` binaries and are not readable from the repo. | Actual vehicle dynamics. | Approximated; refine via system-ID. |

The practical consequence: **a clean FSDS run does not certify the real car**,
most of all because both localisation and cone detection are oracles there.
The cone-map gap in particular has no model at all today — if perception
quality becomes the limiting factor, that's the next thing to build.

## Measurement rate: pose must keep up with the controller

`sim_perception` used to publish pose **and** cones on one 10 Hz timer while
the MPC ran at 20 Hz, so every second control step re-solved against a pose
that had not changed. Measured in `mpc_standalone_control_1785976976.csv`:
`car_x`/`car_y` were byte-identical to the previous row on **50.5%** of control
steps (effective pose rate 9.9 Hz), and the freeze runs were almost all exactly
one tick long (766 of 829) — the signature of a 2:1 rate mismatch rather than
random dropouts.

This caused real steering oscillation, and it is a *different* failure from
either the slew limit or delay jitter. On a frozen tick the car had actually
travelled a median 0.33 m (max 0.63 m) and rotated a median 0.84 deg, but `e_y`
did not move — so the controller read its own correction as having failed and
pushed harder, then over-corrected when the pose jumped two steps' worth at
once. 24 steps in that log show `|de_y|` exceeding `v*dt`: lateral error
changing faster than the car could physically move, i.e. catch-up jumps.
Reversal rates on fresh-pose (0.792) versus stale-pose (0.810) ticks are
near-identical, confirming `predict_ahead()` was *not* bridging the gap — it
compensates actuation lag, not a missing measurement.

Data availability was never the constraint: the FSDS bridge publishes odom at
250 Hz (`update_odom_every_n_sec: 0.004`). `sim_perception` was the bottleneck.

**Fixed** by splitting into two timers — `pose_rate` (default 20 Hz, must be
>= the controller's `CONTROL_HZ`) and `cone_rate` (default 10 Hz). Cones stay
slower deliberately: cropping the oracle map and building three messages is
that node's expensive path, and the planner gains nothing from running it at
the control rate. The node logs a warning if `pose_rate < 20`.

Note the offline rollout never modelled this at all — it calls
`perception.visible_cones()` and `planner.update()` every step with a fresh
pose, i.e. it always assumed the 20 Hz behaviour the fix now delivers. So this
was a live-only defect, and the sim needs no mirrored change. Reproducing a
slow-pose regime offline would require a **pose zero-order hold at a
configurable rate**, not more `DELAY_JITTER_STEPS` — jitter models a
*varying* delay, whereas this was a *systematically halved measurement
rate*. Those are different failure modes and the jitter knob will not
reproduce it.

**A sibling bug of the same root-cause class, also fixed** (see
`docs/logs/sim_to_real_investigation.md` §55): the fix above makes
`pose_rate` keep up with the controller, but it did not guarantee that a
given `car_position` sample and the `car_speed`/`car_yaw_rate` the
controller read *at the same tick* came from the same underlying odom
instant. `mpc_controller.py`/`mpc_controller_standalone.py` subscribed to
the raw 250 Hz `/fsds/testing_only/odom` directly for speed/yaw-rate, a
second, independent subscription racing `sim_perception.py`'s own separate
subscription to the same publisher (the one that produces `car_position`).

This is a **cross-topic snapshot mismatch**, not a rate mismatch — different
mechanism, same underlying cause (`sim_perception.py`'s publish timing not
actually delivering what a downstream consumer assumes). Fixed by adding
`/fsae/slam/car_odom` (`nav_msgs/Odometry`), published from the exact same
`_odom_cb`-updated state and the exact same 20 Hz timer tick as
`car_position`, and switching both MPC controllers to read speed/yaw-rate
from it instead of the raw topic. Same "no offline mirror needed" reasoning
applies: `sim/rollout_core.py` has one single, internally-consistent plant
state at every instant, so it never had an equivalent of two racing
subscriptions to begin with.

## Delay realism: why the tuner under-reproduces live chatter

The offline rollout applies a fixed `DELAY_STEPS` lag and `predict_ahead()`
compensates for it **exactly** — the simulated controller knows its own lag
perfectly. The live controller never does: it estimates the lag from a pose
timestamp divided by a jittering loop period.

Measured live, that loop period is:
- median 0.0498 s
- p99 0.0741 s
- max 0.1205 s
- jitter σ ≈ 0.0092 s ≈ 0.18 steps

So the live step count is regularly wrong by one — and each wrong value
changes how far `x0` is rolled forward, feeding a step disturbance into the
QP at the control rate.

`settings.DELAY_JITTER_STEPS` (default `0.2`, matching the measured σ above)
is what closes part of that gap — see [tuning.md](tuning.md#2-delay-compensation)
for what it does and how to tune it. The error is deliberately two-sided —
over-estimating re-rolls the oldest pending command, mirroring what a
too-large `pose_age_s` does live.

**How much this actually recovers — measured, not assumed.** With
`USE_PLANNER=True` (the real configuration), raising the slew limit from
80 → 180 deg/s drops the fraction of steps pinned on the limit from 1.6–4.3%
to ~0.5% across `PATH_MICRO_SLALOM`/`PATH_S_BEND`/`PATH_SUDDEN_TURN`, so the
constraint is now visibly active in the tuner rather than inert.

Delay jitter on its own moves composite scores by <0.002.

**What it still does not reproduce.** The offline rollout produces ~6–12
steering reversals per run; the live log has ~1441 (≈8 Hz). Delay jitter and
the slew limit together do not close that gap. Note also that `use_planner`
matters far more than either: with `use_planner=False` the peak commanded slew
is 88 deg/s, with `use_planner=True` it is 397 deg/s.

The dominant missing factor turned out to be the **10 Hz pose against a 20 Hz
controller** described in the section above — not modelled offline because the
rollout always used a fresh pose every step. That is a live-only defect and is
now fixed in `sim_perception` rather than modelled here.

**SLAM pose noise is not the remaining cause, despite being an intuitive
guess.** The log came from FSDS, where `sim_perception` republishes
ground-truth odom, so the pose was exact — just stale. Staleness is not noise.
`SLAM_NOISE_ENABLED` exists to model the real car's localisation error
specifically, and this conclusion about that FSDS log holds regardless of that
flag's default (see the "Simulator fidelity limits" table above).

**Treat a clean offline score as necessary but not sufficient** — re-measure
the live reversal count after the pose-rate fix before assuming the remaining
gap is still open, then confirm weights on the car regardless.

## The sim-to-real gap: a lateral-acceleration ceiling, partly closed

> **Full investigation history** — every hypothesis tried, why each looked
> right, and how it was eliminated — is in
> [`docs/logs/sim_to_real_investigation.md`](logs/sim_to_real_investigation.md).
> Read that before re-testing any candidate below; most already were.

On the recorded `comp test map 3`, same tuned gains both sides, the live car
saturates steering far more than the offline sim and carries a much larger
heading error:

| | offline sim | live car |
|---|---|---|
| steering saturation | 4.8% | **21.1%** |
| \|e_psi\| mean / p90 | 6.9° / 18.5° | **15.9° / 42.0°** |
| a_lat max | 11.24 | 12.34 |

When the car is at full lock it is pulling only ~4.1 m/s² lateral at ~5.7 m/s
— it is not cornering hard, it is rotating back from a large heading error.
Heading error arrives in sustained episodes (median ~0.5 s, up to 2.4 s, 96%
of energy below 1 Hz) — a stale/wrong reference, not high-frequency chatter.

**Candidates eliminated** (see the log for each measurement): plant grip too
generous, entering corners too fast, planner centreline quality, SLAM pose
noise, extra actuation delay, planner update rate, pose-feed hold, tyre
grip/understeer, `MAX_STEER_RAD` command scaling, actuator lag, yaw_rate/speed
telemetry error, tyre front/rear balance, FSDS's `SteeringCurve` UE4/PhysX
speed-dependent steering scaling (confirmed flat at 1.0, no scaling at all).

### Root cause: FSDS enforces a sustained lateral-acceleration ceiling

Below ~6 m/s the car delivers the commanded steering angle exactly; above it
the yaw response collapses, in a cliff rather than a gradual curve. This is
**not** tyre saturation — a grip limit doesn't depend on speed — and it engages
far below the ~12–14 m/s² lateral acceleration the same car/plant can reach
elsewhere. A step-input test shows the ceiling holds lateral acceleration
(not yaw rate) constant across speeds, and that it overshoots before settling
— so it is enforced by a term that takes time to build, not a hard clip.

Modelled in `model/vehicle_physics.py` (`alat_ceiling*`) as a restoring yaw
moment with a first-order lag, using a **leaky integral of the signed
excess** (not the proportional law first tried, which structurally cannot
pin the settled value at the ceiling for any gain):

| parameter | value | basis |
|---|---|---|
| `alat_ceiling` | 7.5 m/s² | measured settled lateral acceleration |
| `alat_ceiling_mode` | `'pi'` | leaky-integral law; a proportional law cannot hold a setpoint |
| `alat_ceiling_gain` | 450 N·m | fitted to measured peak; settled value falls out by structure |
| `alat_ceiling_tau` | 0.40 s | measured transient time constant |
| `alat_ceiling_enabled` | `True` | models **FSDS**, not the physical car — disable for real-vehicle work |

This closes part of the gap (the plant's lateral-acceleration distribution
now matches the car's) but **not the steering-saturation gap** — live still
saturates roughly 4× more often. The residual is narrowed to the *rate of
entry* into the high-heading-error state, not steady-state cornering
capability; the reference-heading lead is the next avenue and is testable
offline. See the log's §9-§13 for the full derivation, the two false starts
in the ceiling's control law, and the quantified ledger of what each tested
factor explains.

**Validated with:**

| tool | answers |
|---|---|
| `tuner/checks/steering_sysid_analysis.py`, `tuner/checks/steering_step_analysis.py` | what does FSDS do? |
| `tuner/checks/plant_openloop_validation.py` | does the plant model reproduce it? (`--ab`, `--robustness`) |
| `tuner/recorded_map_rollout.py` | the closed-loop table above, headless and reproducible |
| `tuner/checks/live_vs_sim_diagnostics.py` | conditional + reference-heading decomposition of live vs sim |

Reproduce the closed-loop table with
`python -m tuner.recorded_map_rollout [--mode p --gain 700 | --tau 0.25 | --no-ceiling]`.

### The open-loop system-ID experiment (reusable methodology)

Isolating the plant from the controller — command fixed steering angles at
fixed speeds on an empty map and record the achieved yaw rate — is what
found the ceiling above. Reuse this whenever a plant-vs-car discrepancy is
suspected; a closed-loop lap log cannot separate a plant defect from a
controller/reference one.

**Where the pieces live** (the node and harness are working-tree-only files
in the live ROS 2 workspace, no mirror in this repo):

| file | repo | role |
|---|---|---|
| `control/fsae_control/fsae_control/steering_sysid.py` | `fsae_planning` (live ROS 2 ws) | the node — drives FSDS directly |
| `ros2/run_steering_sysid.sh` | **FSDS repo root**, next to `launch_all.sh` | one-command harness |
| `tuner/checks/steering_sysid_analysis.py` | `fsae_MPCTest` | reads the log, names the mechanism |
| `fsae_control/steering_step.py` / `ros2/run_steering_step.sh` / `tuner/checks/steering_step_analysis.py` | same split | the step-input companion test (50 Hz, isolates the transient) |

**Run it with one command** (starts FSDS, waits for RPC, starts the bridge,
waits for odom, runs the sweep, analyses the log, tears everything down,
including on Ctrl+C):

    cd <FSDS repo>/ros2 && ./run_steering_sysid.sh

Flags: `--no-sim` (FSDS already running), `--quick` (fewer points), and any
`-p name:=value` passes through to the node. **Run it on an empty map** — it
circles at up to 14 m/s and does not brake for cones. The harness refuses to
start if `mpc_controller`, `fsds_bridge`, or `stanley` is already running,
since two publishers on `/fsds/control_command` would interleave and corrupt
the log.

**Geometry is bounded automatically.** The node reaches target speed while
already turning (so it orbits rather than travelling), checks a geofence
(`home_radius`/`max_radius`) from every phase, and predicts each point's orbit
size in advance (using a deliberately pessimistic `K_US_ESTIMATE = 0.05`) to
skip any (speed, steering) pair whose orbit won't fit in the geofence,
logging what it dropped. Default steering commands
(`[0.5, 0.65, 0.8, 1.0]`) are biased high since low-angle, high-speed points
are both the least informative and the least likely to fit.

**Reading the log.** It records the raw normalised `cmd.steering` alongside
the roadwheel angle it's assumed to map to — recording only the assumed
angle would beg the question the test exists to answer. A falling
`s = δ_ach/δ_cmd` is not by itself diagnostic (a speed-scaled rack, genuine
understeer, and grip saturation all produce one), so the analyser fits all
five candidate mechanisms to achieved yaw rate and reports the margin to the
runner-up:

| winning model | meaning |
|---|---|
| neutral (s≈1) | steering path is fine; look at the controller/reference |
| constant scale | `MAX_STEER_RAD` wrong — fix in all three copies |
| speed-scaled rack | FSDS reduces lock with speed; model it in the plant |
| understeer (v²) | real vehicle dynamics |
| grip saturation | yaw capped by lateral grip |

The default speed sweep is **3–14 m/s**, wide enough to separate speed-scaled
rack from understeer (near-degenerate over a narrow band). If the analyser
prints a margin warning, widen the speed range and re-run rather than
trusting the verdict; it also refuses a verdict when fewer than 3 windows
contain real motion (a car wedged against a wall otherwise reports a
confident, meaningless answer from all-zero data).

### Checked: the longitudinal path is not mis-scaled

Throttle authority, acceleration, and braking limits match FSDS closely (the
car if anything brakes harder than the plant model allows; 0% throttle
saturation). **But `fsds_bridge.py` discards the MPC's own `a_cmd` output**
and re-derives throttle from a separate P-controller on target speed —
offline, `a_cmd` drives the plant directly. This is a genuine, unmodelled
sim/live divergence in the longitudinal loop (mean speed error ~0.6 m/s), but
it cannot explain the yaw-saturation gap — a longitudinal path cannot stop
the car rotating. Modelling it offline, or feeding `a_cmd` through live, is
untested.

**Consequence:** offline scores are not yet fully predictive of live
behaviour. A tuning run that scores well offline can still produce a car
that saturates steering a fifth of the time. Always validate on the car
before trusting a tuned weight set.

**One specific failure mode of this P-loop: it can stall the car completely
at a low target speed.** `fsds_bridge.py`'s throttle is
`KP_THROTTLE * speed_error` with no floor — at `car_speed=0` that only
saturates to a usable value when the target speed is large (a 20 m/s target
gives throttle 1.0 from a stop; a 3 m/s target gives only 0.18).

Measured live: a 3 m/s low-speed test left `v_actual` at ~0 for a full 54 s
run, not merely "accelerates slowly."

Fixed with a stiction-breaking throttle floor
(`STICTION_KICK_SPEED`/`STICTION_KICK_THROTTLE`, both in `fsds_bridge.py`):
below 1.0 m/s car speed the throttle is floored at 0.35 while accelerating,
then the floor stops applying and the normal P-loop tapers it down as the
target is approached.

Inert at any target speed where the P-loop already saturates throttle to
something above the floor from a stop, so this does not change normal
(higher-speed) behaviour — it only fixes the low-target-speed stall.

## Known planner defect: centreline curvature spikes (OPEN — not fixed)

**Status: open.** The controller carries workarounds; the root cause is in
the planner and has not been addressed. Read this before changing
`centerline_planner.py`, `boundary.py`, `cone_sorting.py`, `path_utils.py`, or
the planner's smoothing parameters.

### What it is

The published centreline contains curvature spikes that do not correspond to
any real feature of the track: the same physical corner can be reported with
a radius several times smaller or larger than its true geometry from one
~1 s snapshot to the next, in the extreme down to sub-1 m implied radii that
are physically impossible for this car (min turn radius at 25° lock and a
1.55 m wheelbase is ≈ 3.7 m). The path is otherwise in the *right place* —
repeated passes over the same physical location agree well on where the
centreline sits; the problem is local kinks, not global drift. See
`docs/logs/sim_to_real_investigation.md` §18b for the original measurement
(per-lap severity figures, volatility figures).

### Why it matters to control

`v_target = √(a_lat_max / κ)`, so a spurious κ spike collapses the speed
target, and its absence next frame lets it jump back. This alone can
destabilise the longitudinal loop.

### Workarounds in the controller (defence in depth)

These treat the symptom. They are **not** a fix, and none of them should be
removed without re-measuring against a repaired planner:

1. **Curvature smoothing** — `curvature_speed()` (both
   `control_utils.py` and `sim/speed_profile.py`) takes the max of a 3-point
   running mean of the Menger-curvature series instead of the raw max, so one
   bad triple cannot set the speed for the whole scan window. Modest on its
   own because the whole path moves between frames, not just one point. A raw
   percentile (p75/p90) is deliberately not used instead — the scan window
   only yields ~7 triples, so a percentile is both noisy and biased upward,
   pushing `v_target` well above what the raw max would give. Wrong direction
   to err.
2. **Tracking-error speed gate** — `tracking_error_speed_gate()`. Scales the
   target down on `|e_y|`/`|e_psi|`. Inert in normal driving, but cuts the
   commanded speed sharply once `|e_y| > 1.5 m`. The gate's own output is
   additionally rate-limited (`GATE_RATE_LIMIT`, both node files and
   `sim/rollout_core.py`) so its tick-to-tick change is bounded in either
   direction — see `docs/logs/sim_to_real_investigation.md` §55/§56 for why:
   applying it unsmoothed let a fast-changing tracking error compound with an
   already-falling curvature-based target into a single-tick `v_desired`
   cliff.
3. **Speed-target rise limiter** — `SPEED_TARGET_RISE_RATE = 7.0` m/s², applied
   in both controller nodes and `sim/rollout_core.py`. Increases only;
   decreases pass through instantly so a genuine brake request is never
   delayed.

Combined, these bound tick-to-tick `v_desired` volatility and cap commanded
speed in the unrecoverable `|e_y| > 1.5 m` regime, at a small cost on
already-clean paths — the expected trade-off for a safety gate. See
`docs/logs/sim_to_real_investigation.md` §18b for the measured before/after
figures.

### Suggested fix (not attempted)

Root-cause work belongs in the planner: curvature-aware smoothing or a
spline-fit residual check in `centerline_planner.py`, and/or rejecting cone
pairings that imply a sub-3.7 m radius.

> **One mechanism has a known root cause, and it is NOT cone-map
> accumulation** (`docs/logs/sim_to_real_investigation.md` §19).
>
> The mechanism: `filter_cones_window`'s `min_ahead=0.5` cutoff drops the
> nearest surviving midpoint as soon as the car's own pose crosses it,
> forcing the car-anchored spline (`pin_start` in `smooth_centreline`) to
> reach for the next midpoint instead — sometimes several metres further
> away — producing a sharp, transient near-field curvature spike. This
> reproduces from a single static, already-fully-built cone map, with no
> lap-to-lap accumulation involved; whether a given lap hits it depends on
> *how often that lap's specific pose trace happens to straddle a midpoint
> boundary*, not on the map getting dirtier.
>
> `_gen_midpoints()` returns byte-identical midpoints across the jump, which
> rules out cone-map duplication, `_absorb()`, and exclusive-nearest-neighbour
> reassignment as the cause of *this* mechanism.
>
> **Its measured frequency is modest.** A near-field path *tangent
> direction* metric, cross-checked against `e_psi`/`steer_deg` to rule out
> genuine corners, finds large single-tick tangent jumps on a small fraction
> of ticks in the checked log (worst instance a ~17° tangent reversal, not a
> 5×+ radius jump). The mechanism is real but infrequent at this measured
> size; no fix has been shipped for it. See
> `docs/logs/sim_to_real_investigation.md` §19/§23 for the full measurement,
> including an earlier much larger frequency estimate that was retracted as
> a measurement artifact.

> **A separate reference-heading tail effect exists, and it is a DIFFERENT
> mechanism from the `min_ahead` seed jump above** — treat them as two open
> items, not one.
>
> The bulk of the planner's online reference-heading swing is genuine
> geometry, tracking a fixed geometry-only reference closely, but a small
> tail of ticks swings faster than the geometric rate and carries a much
> higher immediate steering-saturation rate. That tail is not the
> `min_ahead` seed-jump mechanism above (only a small minority of high-excess
> ticks coincide with one) — tracing the tail ticks directly instead shows a
> sustained turn-in lag at braking corner entries: the planner's online
> reference correctly anticipates a sharp corner earlier and more
> aggressively than the car has physically yawed yet, so the
> reference-minus-car gap grows continuously for over a second before
> closing. See `docs/logs/sim_to_real_investigation.md` §26/§27 for the
> measured ratios and rates.

> **The reference-heading rate limiter is a tried candidate fix that does NOT
> work. Default `False`; do not re-enable casually.**
>
> `settings.REF_HEADING_RATE_LIMIT_ENABLED`/`REF_HEADING_RISE_RATE` (in
> `sim/rollout_core.py::_rate_limit_ref_psi`) caps how fast the tracked
> reference heading may change per tick — same shape as the existing
> `SPEED_TARGET_RISE_RATE`. Offline it improves saturation on both the
> recorded map and every path in `VALIDATION_SUITE` with no DNF at a moderate
> rate limit, but tightening it further reaches 0% saturation on the
> recorded map while DNFing `PATH_MICRO_SLALOM` off-track in the suite — a
> failure the recorded map cannot show, having no fast-reversal slalom
> geometry.
>
> On the car it made saturation **worse**, not better, and produced a
> multi-second continuous saturation episode — the same failure mode found
> offline on `PATH_MICRO_SLALOM`, just short of a full DNF. Holding the
> reference back during turn-in leaves a larger heading deficit to claw back
> later, worse than not limiting at all. See
> `docs/logs/sim_to_real_investigation.md` §28/§29 for the measured rates.
>
> The underlying measurement (the reference genuinely outpaces the car's
> yaw) still stands; only this fix is known not to work. Do not re-enable
> without a new offline test against a synthetic path shaped like this
> failure (a long, smoothly-growing heading deficit through a decelerating
> corner) — the recorded map and `VALIDATION_SUITE` as they stand only
> partially warn about it.

### A cone-map duplication bug in `_absorb()` — fixed here, still open upstream

`planning/cone_map.py::ConeMap._absorb()` had a genuine bug: two detections
of one physical cone in the same frame, both farther than `MERGE_DIST`
(0.8 m) from anything already in the map — i.e. that cone's first sighting —
were both appended as separate, permanent entries. This was deterministic
and independent of `MERGE_DIST` tuning, since it only compared each candidate
against the existing map, never against other candidates in the same batch.

Fixed in both copies within this repo (offline `planning/cone_map.py` and the
`fsds_simulator` mirror's `cone_map.py`) by also checking candidates against
each other before appending. **Not ported upstream** — the live
`fsae_planning` repo's `cone_map.py::_absorb()` still carries the
same-frame-duplicate-prone version; porting it there is a resync TODO, not
something this repo can apply directly.

Sim output is byte-identical when the bug cannot fire (FSDS's cone perception
is a noise-free oracle by default, so it never produces the same-frame
duplicate detections needed to trigger it). This does **not** establish that
the bug explains any part of the curvature-spike defect or the saturation gap
— that needs either a measured real-detector noise figure or a live log
showing actual duplicate clustering, neither available. See
`docs/logs/sim_to_real_investigation.md` §15 for full detail, including the
`CONE_NOISE_ENABLED` offline testing capability this fix was verified
against.

### Related but eliminated: `blend_paths`' reset-bypass discontinuity

`path_utils.py::blend_paths()` (used by both `centerline_planner.py` and
`sim/sim_track.py`, `alpha=0.4`) exists to stop the from-scratch rebuild every
pose tick from producing a heading jump. It has a `reset_dist=2.0` m bypass
that skips the blend entirely when the rebuild has moved too far from the
previous publish — plausibly correlated with this section's curvature-spike
defect, since a spike event is exactly when the rebuild changes most. It is
real and can jump the reference sharply on other geometries, but does not
fire on the recorded map (its max trigger-distance there sits just under the
threshold) — so it cannot explain that map's saturation gap. Re-check if a
planner fix here changes rebuild volatility enough to push the recorded map
over the threshold. See `docs/logs/sim_to_real_investigation.md` §14 for the
measured figures.

## Cone geometry

Track width and cone spacing match FSDS exactly and are not a source of the
sim-to-real gap — full measurements (track width, spacing percentiles, FS
rule limits) are in `docs/logs/sim_to_real_investigation.md`'s
"## 11. Also verified: cone geometry is accurate".

> **Measure spacing along the path, not down the array.** Cones are stored in
> **recording order** (`source: fsae_sim_perception.cone_recorder`), not sorted
> around the track, so consecutive entries are not spatially adjacent. Naively
> differencing the array reports phantom gaps of tens of metres. Project each
> cone onto its nearest centreline index and sort by that first.
