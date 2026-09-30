# Simulator Fidelity and Known Defects

Where the offline simulator, FSDS and the live car diverge, and which divergences are understood. See [glossary.md](glossary.md) for what "offline simulator" and "FSDS" mean and how far each can be trusted. This doc covers the specific measured gaps between them.

**Plain version.** The offline simulator is a rough model, good enough to get weights into the right region, but it is not matched against reality and carries no accuracy guarantee. FSDS is a closer approximation, a real physics engine, but it is not confirmed against the real car either. This doc records every place the offline simulator, FSDS and the real car are known to disagree, how large each gap is, and which are explained. Read it before trusting an offline-only or FSDS-only result.

**Headline caution.** On the same map with the same gains, the live car saturates its steering far more often than the offline rollout and carries a much larger heading error. The history is in `docs/logs/sim_to_real_investigation.md`.

## Contents

1. [What FSDS and the offline rollout do not model](#what-fsds-and-the-offline-rollout-do-not-model)
2. [Pose must keep up with the controller](#pose-must-keep-up-with-the-controller)
3. [The tuner under-reproduces live chatter because delay is too clean](#the-tuner-under-reproduces-live-chatter-because-delay-is-too-clean)
4. [The offline sim still misses the live steering saturation](#the-offline-sim-still-misses-the-live-steering-saturation)
5. [FSDS enforces a speed-dependent lateral-acceleration ceiling](#fsds-enforces-a-speed-dependent-lateral-acceleration-ceiling)
6. [The longitudinal path](#the-longitudinal-path)
7. [Known planner defect: centreline curvature spikes](#known-planner-defect-centreline-curvature-spikes)
8. [Cone-map duplication in `_absorb()` is fixed in every copy](#cone-map-duplication-in-_absorb-is-fixed-in-every-copy)
9. [Cone geometry is accurate](#cone-geometry-is-accurate)

## What FSDS and the offline rollout do not model

These are the known ways both simulators are easier than reality.

| aspect | FSDS and this rollout | real car | modelled offline? |
|---|---|---|---|
| localisation | Perfect. `sim_perception` copies ground-truth `/fsds/testing_only/odom` verbatim onto `/fsae/slam/car_position`. No noise, drift or lag | ZED visual odometry plus `cone_mapper` SLAM: jitters, drifts, lags | via `SLAM_NOISE_ENABLED` (default off), which adds jitter and slow drift to the estimated pose |
| cone map | latched oracle map of exact positions, cropped to a forward window plus an omni radius. Only range is limited | real detections: false positives and negatives, position error, colour confusion, range-dependent noise | partly, via `CONE_NOISE_ENABLED` (default off), position jitter only |
| pose rate | 20 Hz (`pose_rate`), matching the controller. Was 10 Hz, see below | bounded by the perception pipeline's real throughput | live-only concern. The rollout always uses a fresh pose per step, and `PoseFeedHold` models the stalls |
| actuation delay | fixed `DELAY_STEPS`, compensated exactly by `predict_ahead()` | variable, estimated from a timestamp, never exactly known | partly: `DELAY_JITTER_STEPS` perturbs the controller's belief, and `POSE_HOLD_*` models pose-feed stalls |
| steering slew | hard `du_max`, on both sides | real rack limit. Achieved rates reached about 138 deg/s (p99) and 218 deg/s (max) on a live log | yes, at 180 deg/s, a measured lower bound and not a datasheet figure |
| tyre and plant model | bicycle model with estimated `lf`, `lr`, `Iz`, `Cf`, `Cr`. The true values sit in git-LFS `.uasset` binaries not readable from the repo | actual vehicle dynamics | approximated, refine via system-ID |
| perception coverage | live `sim_perception` also publishes every cone within an omni-directional radius (25 m) | | no. `SimPerception` uses the forward box only |

A clean FSDS run does not certify the real car, most of all because localisation and cone detection are oracles there. The cone-map gap has no model at all today. If perception quality becomes the limiting factor, that is the next thing to build.

## Pose must keep up with the controller

`sim_perception` publishes `pose_rate` and `cone_rate` on two independent timers. A single shared 10 Hz timer, with the MPC at 20 Hz, had left every second control step re-solving against a pose that had not changed.

Measured on one live log (the source CSV is not retained locally, see `docs/logs/sim_to_real_investigation.md` section 60): `car_x` and `car_y` were byte-identical to the previous row on 50.5% of control steps (effective pose rate 9.9 Hz), and the freeze runs were almost all exactly one tick long (766 of 829), the signature of a 2:1 rate mismatch and not random dropouts.

This caused real steering oscillation and is a different failure from the slew limit or delay jitter. On a frozen tick the car had travelled a median 0.33 m (max 0.63 m) and rotated a median 0.84 deg, but `e_y` did not move, so the controller read its own correction as failed and pushed harder, then over-corrected when the pose jumped two steps at once. 24 steps in that log show `|de_y|` above `v*dt`, lateral error changing faster than the car could physically move. Reversal rates on fresh-pose (0.792) and stale-pose (0.810) ticks are near-identical, so `predict_ahead()` was not bridging the gap. It compensates actuation lag, not a missing measurement.

Data availability was never the constraint. The FSDS bridge publishes odom at 250 Hz (`update_odom_every_n_sec: 0.004`). `sim_perception` was the bottleneck.

`pose_rate` (default 20 Hz, must be at least the controller's `CONTROL_HZ`) and `cone_rate` (default 10 Hz) run as separate timers. Cones stay slower on purpose: cropping the oracle map and building three messages is the node's expensive path and the planner gains nothing from the control rate. The node logs a warning if `pose_rate < 20`.

The offline rollout does not model a slow-pose regime. It calls `perception.visible_cones()` and `planner.update()` every step with a fresh pose. Reproducing it would need a pose zero-order hold at a configurable rate, not more `DELAY_JITTER_STEPS`, because jitter models a varying delay while a shared-timer mismatch is a systematically halved measurement rate.

**A sibling of the same root-cause class.** Keeping `pose_rate` at the controller rate does not by itself guarantee that a `car_position` sample and the `car_speed` and `car_yaw_rate` read at the same tick come from one odom instant. `mpc_controller.py` reads speed and yaw rate from `/fsae/slam/car_odom` (an Odometry message from nav_msgs), published from the same `_odom_cb` state on the same 20 Hz tick as `car_position`. Subscribing to the raw 250 Hz `/fsds/testing_only/odom` instead would be a second subscription racing `sim_perception`'s own, a cross-topic snapshot mismatch and not a rate mismatch. The rollout has one internally consistent plant state per instant, so it has no equivalent.

## The tuner under-reproduces live chatter because delay is too clean

The offline rollout applies a fixed `DELAY_STEPS` lag and `predict_ahead()` compensates for it exactly, so the simulated controller knows its own lag perfectly. The live controller estimates the lag from a pose timestamp divided by a jittering loop period. Measured live, that loop period has a median of 0.0498 s, p99 0.0741 s, max 0.1205 s and jitter sigma of about 0.0092 s (0.18 steps). The live step count is therefore regularly wrong by one, and each wrong value changes how far `x0` is rolled forward, feeding a step disturbance into the QP at the control rate.

`DELAY_JITTER_STEPS` (default `0.2`, matching the measured sigma) closes part of that gap. See [tuning.md](../guides/tuning.md#delay-compensation) for what it does. The error is two-sided: over-estimating re-rolls the oldest pending command, mirroring what a too-large `pose_age_s` does live.

**Measured effect.** With `USE_PLANNER=True`, raising the slew limit from 80 to 180 deg/s cut the fraction of steps pinned on the limit from 1.6 to 4.3% down to about 0.5% across the micro-slalom, S-bend and sudden-turn paths, so the constraint is now active in the tuner and not inert. Delay jitter alone moves composite scores by under 0.002.

**What it still does not reproduce.** The offline rollout produces about 6 to 12 steering reversals per run, and the live log had about 1441 (roughly 8 Hz). Delay jitter and the slew limit together do not close that. `use_planner` matters more than either: with `use_planner=False` the peak commanded slew is 88 deg/s, and with `True` it is 397 deg/s. The dominant missing factor is the 10 Hz pose against a 20 Hz controller described above, fixed in `sim_perception` and not modelled here. Source: `docs/logs/sim_to_real_investigation.md`, sections 58 and 60.

**SLAM pose noise is not the remaining cause.** That log came from FSDS, where `sim_perception` republishes ground-truth odom, so the pose was exact but stale. Staleness is not noise. `SLAM_NOISE_ENABLED` models the real car's localisation error, and this conclusion holds whatever its default.

A clean offline score is necessary and not sufficient. Re-measure the live reversal count after the pose-rate fix before assuming the remaining gap is still open, then confirm weights on the car regardless.

## The offline sim still misses the live steering saturation

Same map (`comp_test_map_3`), same tuned gains on both sides. The live column is the fixed reference set in `tuner/validation/recorded_map_rollout.py` (`LIVE`). The offline columns were measured by running that script on the current code.

| | offline, oracle path | offline, planner in loop | live car |
|---|---|---|---|
| steering saturation | 0.10% | 5.71% | 21.1% |
| `\|e_psi\|` mean / p90 (deg) | 2.16 / 5.18 | 7.30 / 16.85 | 15.9 / 42.0 |
| `a_lat` max (m/s^2) | 8.94 | 8.91 | 12.34 |
| `a_lat` above the ceiling | 5.48% | 1.21% | 9.8% |
| reversals per second | 0.23 | 1.03 | 1.62 |
| composite score | 0.428 | 0.530 | not comparable |

Commands: `python -m tuner.validation.recorded_map_rollout` (oracle path, the default, matching `USE_PLANNER=False`) and `python -m tuner.validation.recorded_map_rollout --planner --continue-after-dnf` (planner in loop). Both finished the lap with no DNF. The planner-in-loop column is the comparable one, since the live car runs its planner.

Earlier write-ups quoted a single offline column of 4.8% saturation, 6.9 / 18.5 deg heading error, 11.24 `a_lat` max and 10.9% above 7.5. The configuration behind that column was not recorded and it did not reproduce on the current code, so those numbers are not used. Use the table above, and re-run the script rather than quoting either set.

Live saturates roughly four times as often as the planner-in-loop offline run and carries about twice the heading error. When the car is at full lock it is pulling only about 4.1 m/s^2 lateral at about 5.7 m/s, so it is not cornering hard. It is rotating back from a large heading error. Heading error arrives in sustained episodes (median about 0.5 s, up to 2.4 s, 96% of energy below 1 Hz): a stale or wrong reference, not high-frequency chatter.

**Candidates eliminated** (each measured, see the log): plant grip too generous, entering corners too fast, planner centreline quality, SLAM pose noise, extra actuation delay, planner update rate, pose-feed hold, tyre grip and understeer, `MAX_STEER_RAD` command scaling, actuator lag, yaw-rate and speed telemetry error, tyre front and rear balance, and FSDS's `SteeringCurve` speed-dependent steering scaling (confirmed flat at 1.0). Two more eliminated on the ceiling side: lowering the ceiling (DNFs off track, the wrong failure mode) and `fsds_bridge` discarding `a_cmd` (inside saturation the sim already arrives hotter than live).

**What is still open.** The leading candidate is the planner's reference-heading lead: in both stacks the reference heading swings faster than either car can yaw and drives most heading-error growth. A rate limiter on it was tried and made the live car worse, see [the planner defect section](#known-planner-defect-centreline-curvature-spikes). The gap remains, and a tuned weight set that scores well offline can still saturate a fifth of the time on the car. Always validate on the car.

## FSDS enforces a speed-dependent lateral-acceleration ceiling

Open-loop system-ID (fixed steering at fixed speeds with the MPC bypassed, plus a step-input test) shows that above about 6 m/s the yaw response collapses, in a cliff and not a gradual curve. Below about 6 m/s the car delivers the commanded angle exactly.

| speed | commanded steer | achieved | ratio |
|---|---|---|---|
| 3 to 5 m/s | 25 deg | 25 deg | 1.00 |
| 8 m/s | 25 deg | 8.5 deg | 0.34 |
| 14 m/s | 25 deg | 4.1 deg | 0.17 |

This is not tyre saturation. A grip limit does not depend on speed, and this engages far below the 12 to 14 m/s^2 lateral acceleration the same car reaches elsewhere. A step-input test shows the ceiling holds lateral acceleration (not yaw rate) constant across speeds and overshoots before settling, so it is enforced by a term that takes time to build, not a hard clip.

**The ceiling rises with speed.** The sustained value measured by the sweep and by a 15 s step hold:

| speed | sweep (sustained orbit) | step test, 15 s hold | model `max(7.5, 2.46 + 0.47*v)` |
|---|---|---|---|
| 8 m/s | 6.45 | 8.17 | 7.50 |
| 11 m/s | 7.54 | 8.40 | 7.63 |
| 14 m/s | 9.26 | 9.67 | 9.04 |

The two tests agree on the shape and disagree on the level (the step test sits 0.4 to 1.7 m/s^2 above the sweep). The sweep's fit was chosen because a lap is sustained circular cornering, which is what the sweep measures. That was a judgement call. The model takes the larger of the flat 7.5 and the sweep line, so it never lowers the ceiling below the value validated earlier, and the line takes over above about 10.7 m/s. At 8 m/s the model sits 1.05 above the sweep measurement. Sources: `docs/logs/sim_to_real_investigation.md`, sections 35 and 37.

It is modelled in `model/vehicle_physics/params.py` (`alat_ceiling*`) as a restoring yaw moment with a first-order lag, using a leaky integral of the signed excess. A proportional law was the first attempt. It cannot pin the settled value at the ceiling for any gain, because a proportional term needs a finite error to produce output, so its equilibrium sits above the setpoint. Fitting the peak left sustained cornering 13% high, and fitting the settled value flattened the excursions and DNF'd. The integral form settles at the ceiling by structure.

| parameter | value | basis |
|---|---|---|
| `alat_ceiling` | 7.5 m/s^2 | low-speed floor, the measured settled lateral acceleration |
| `alat_ceiling_slope`, `alat_ceiling_intercept` | 0.47 m/s^2 per m/s, 2.46 m/s^2 | the sweep's fit, used where it exceeds the floor |
| `alat_ceiling_mode` | `'pi'` | leaky-integral law. The proportional law (`'p'`) is kept only so `plant_openloop_validation --ab` can reproduce why it was rejected |
| `alat_ceiling_gain` | 450 N*m | fitted to the measured peak. The settled value falls out by structure |
| `alat_ceiling_tau` | 0.40 s | measured transient time constant. It affects only the transient. Moving it did not close the saturation gap (it moved saturation the wrong way), so do not revisit it without new evidence |
| `alat_ceiling_enabled` | `True` | models FSDS, not the physical car. Disable for real-vehicle work |

This closes part of the gap (the plant's lateral-acceleration distribution now resembles the car's) but not the steering-saturation gap. The residual is narrowed to the rate of entry into the high-heading-error state, not steady-state cornering capability.

**A live validation of the speed-dependent ceiling has not been done.** The recorded-map ratios in the log are offline only.

**Validated with** the open-loop harness ([offline_live_parity.md](offline_live_parity.md), "Steering system-ID harness", whose ROS 2 nodes are currently missing), `python -m tuner.validation.plant_openloop_validation` (replays both open-loop experiments through the offline plant) and `python -m tuner.validation.recorded_map_rollout` (`--mode p --gain 700`, `--tau 0.25` and `--no-ceiling` reproduce the historical and unconstrained variants). Run the open-loop check after any change to `model/vehicle_physics/`. Its low-speed rows are a known rig confound, not a finding. See [debugging_tools.md](../guides/debugging_tools.md).

## The longitudinal path

Throttle authority, acceleration and braking limits match FSDS closely (the car if anything brakes harder than the plant allows, and throttle saturation is 0%). But `fsds_bridge.py` discards the MPC's own `a_cmd` and re-derives throttle from a separate P-controller on target speed, while offline `a_cmd` drives the plant directly. This is an unmodelled sim-to-live divergence in the longitudinal loop (mean speed error about 0.6 m/s). It cannot explain the yaw-saturation gap, since a longitudinal path cannot stop the car rotating. It applies only with `standalone_output=false`. In the default `standalone_output=true` mode the node publishes the MPC's throttle and brake directly and bypasses `fsds_bridge`.

**The P-loop can stall the car at a low target speed.** The throttle is `KP_THROTTLE * speed_error` (`KP_THROTTLE = 0.06`) with no floor, so at zero speed it saturates only when the target is large. A 20 m/s target gives throttle 1.0 from a stop, and a 3 m/s target gives 0.18. Measured live, a 3 m/s test left `v_actual` near 0 for a full 54 s run. `fsds_bridge.py` therefore floors the throttle at `STICTION_KICK_THROTTLE = 0.35` below `STICTION_KICK_SPEED = 1.0` m/s car speed while accelerating, then lets the normal P-loop taper it down. The floor is inert wherever the P-loop already exceeds it from a stop, so it changes only the low-target-speed stall.

## Known planner defect: centreline curvature spikes

**Status: open.** The controller carries workarounds. The root cause is in the planner and has not been addressed. Read this before changing `centerline_planner.py`, `boundary.py`, `cone_sorting.py`, `path_utils.py` or the planner's smoothing parameters.

### What it is

The published centreline contains curvature spikes that do not correspond to any real feature of the track. The same corner can be reported with a radius several times smaller or larger than its true geometry from one snapshot (about 1 s) to the next, in the extreme down to sub-1 m implied radii, which are impossible for this car (minimum turn radius at 25 degrees of lock with a 1.55 m wheelbase is about 3.3 m). The path is otherwise in the right place: repeated passes agree on where the centreline sits. The problem is local kinks, not global drift. See `docs/logs/sim_to_real_investigation.md` section 18b for the measurements.

### Why it matters to control

`v_target = sqrt(a_lat_max / kappa)`, so a spurious curvature spike collapses the speed target, and its absence next frame lets it jump back. That alone can destabilise the longitudinal loop.

### Workarounds in the controller (defence in depth)

These treat the symptom. None should be removed without re-measuring against a repaired planner.

1. **Curvature smoothing.** `curvature_speed()` (in `control_utils.py` and `sim/speed_profile.py`) takes the max of a 3-point running mean of the Menger-curvature series instead of the raw max, so one bad triple cannot set the speed for the whole scan window. A raw percentile (p75 or p90) is not used instead: the scan window yields only about 7 triples, so a percentile is noisy and biased upward, pushing `v_target` above what the raw max gives, the wrong direction to err.
2. **Tracking-error speed gate.** `tracking_error_speed_gate()` scales the target down on `|e_y|` and `|e_psi|`. It is inert in normal driving and cuts the commanded speed sharply once `|e_y|` exceeds 1.5 m. Its output is rate-limited (`GATE_RATE_LIMIT = 2.0` per second, in `mpc/node_constants.py`, `stanley_controller.py` and `sim/rollout/speed_target.py`), because applying it unsmoothed let a fast-changing tracking error compound with an already-falling curvature target into a single-tick `v_desired` cliff. See the log, sections 55 and 56.
3. **Speed-target rise limiter.** `SPEED_TARGET_RISE_RATE = 7.0` m/s^2 (in `mpc/node_constants.py`, `stanley_controller.py` and `sim/rollout/speed_target.py`). Increases only, and decreases pass through instantly so a real brake request is never delayed. The rate assumes the car can accelerate at 7.0 m/s^2, which is false from a standing start (the car does not break static friction for about 1 s). `speed_target_deficit_max` caps how far the ramp may run ahead of the car's measured speed before it holds and waits. It holds at `car_speed + deficit_max`, never below the previous target, so a real speed error always remains and the ramp resumes as the car closes the gap. A gate that held the target near 0 while stationary would deadlock, since no error means no throttle. It is a real parameter (`MPCParams.speed_target_deficit_max`, offline `SPEED_TARGET_DEFICIT_MAX`) and both sides hold **2.55 m/s**.

   **Why not a higher value.** At the earlier value of 2.5 the clamp was not acting as a launch and recovery guard. It was the binding constraint on acceleration for a third of a normal lap: 36.8% of ticks pinned at exactly the limit, holding `a_cmd` to 4.45 m/s^2 against a plant that delivers about 12. An offline run at 5.0 cut the pinned fraction to 2.3%, nearly doubled peak `a_cmd` to 8.32, made the lap about 2.5 s faster, and slightly lowered lateral error and steering saturation, with launch behaviour unchanged. Values above about 5 buy nothing further. That result is offline only and 5.0 has not been made the default or validated live. The launch script carries it as a commented suggestion. The default is 2.55, which sits at the value that bound acceleration. Rationale for keeping 2.55 rather than adopting 5.0 is not recorded. See `docs/logs/nmpc_progress_term_investigation.md`.

Combined, these bound tick-to-tick `v_desired` volatility and cap commanded speed in the unrecoverable `|e_y| > 1.5 m` regime, at a small cost on already-clean paths, the expected trade-off for a safety gate. See the log, section 18b, for the measured before and after.

### Suggested fix (not attempted)

Root-cause work belongs in the planner: curvature-aware smoothing or a spline-fit residual check in `centerline_planner.py`, and rejecting cone pairings that imply a radius below about 3.3 m.

### Two separate mechanisms, both open

**The `min_ahead` seed jump.** `filter_cones_window`'s `min_ahead=0.5` cutoff drops the nearest surviving midpoint as soon as the car's pose crosses it. That forces the car-anchored spline (`pin_start` in `smooth_centreline`) to reach for the next midpoint, sometimes several metres further, producing a sharp transient near-field curvature spike. It reproduces from a single static, fully built cone map, with no lap-to-lap accumulation, so it is not cone-map clutter. `_gen_midpoints()` returns byte-identical midpoints across the jump, which rules out cone-map duplication, `_absorb()` and nearest-neighbour reassignment as the cause. Its measured frequency is modest: large single-tick tangent jumps on a small fraction of ticks, worst about 17 degrees, not a 5-times radius jump. An earlier much larger frequency estimate was retracted as a measurement artefact. No fix is shipped. See the log, sections 19 and 23.

**The reference-heading tail effect.** A different mechanism. Most of the planner's online reference-heading swing is real geometry, tracking a fixed geometry-only reference closely. A small tail of ticks swings faster than the geometric rate and carries a much higher immediate steering-saturation rate. Tracing those ticks shows a sustained turn-in lag at braking corner entries: the planner's reference correctly anticipates a sharp corner earlier and more aggressively than the car has yawed, so the reference-minus-car gap grows for over a second before closing. See the log, sections 26 and 27.

### The reference-heading rate limiter does not work

`REF_HEADING_RATE_LIMIT_ENABLED` and `REF_HEADING_RISE_RATE` (in `settings/general.py`, applied by `_rate_limit_ref_psi` in `sim/rollout/reference.py`, live `ref_heading_rate_limit_enabled` and `ref_heading_rise_rate_deg_s`) cap how fast the tracked reference heading may change per tick, in the same shape as `SPEED_TARGET_RISE_RATE`. **Default `False`. Do not re-enable casually.**

- Offline it improves saturation on the recorded map and on every path in `VALIDATION_SUITE` with no DNF at a moderate rate.
- Tightening it further reaches 0% saturation on the recorded map while DNFing `PATH_MICRO_SLALOM` off track, a failure the recorded map cannot show because it has no fast-reversal slalom geometry.
- On the car it made saturation worse and produced a multi-second continuous saturation episode, the same failure found offline on the slalom path, just short of a DNF. Holding the reference back during turn-in leaves a larger heading deficit to claw back later.

The underlying measurement (the reference outpaces the car's yaw) still stands. Only this fix is known not to work. Do not re-enable it without a new offline test against a synthetic path shaped like this failure, a long smoothly growing heading deficit through a decelerating corner. See the log, sections 28 and 29.

### `blend_paths` and its reset bypass are eliminated

`path_utils.blend_paths()` (used by `centerline_planner.py` and `sim/planner.py`, `alpha=0.4`) stops the from-scratch rebuild each pose tick from producing a heading jump. It has a `reset_dist=2.0` m bypass that skips the blend when the rebuild has moved too far from the previous publish. That is plausibly correlated with the spike defect, since a spike event is when the rebuild changes most. It can jump the reference sharply on other geometries but does not fire on the recorded map (its maximum trigger distance there sits just under the threshold), so it cannot explain that map's saturation gap. Re-check if a planner fix changes rebuild volatility enough to push the recorded map over the threshold. See the log, section 14.

## Cone-map duplication in `_absorb()` is fixed in every copy

`_absorb()` in `cone_map.py` had a bug: two detections of one physical cone in the same frame, both farther than `MERGE_DIST` (0.8 m) from anything already in the map (that cone's first sighting), were both appended as separate permanent entries. It is deterministic and independent of `MERGE_DIST`, because each candidate was compared only against the existing map, never against other candidates in the same batch.

The fix checks candidates against each other before appending. It is present in all three copies: the offline `planning/cone_map.py`, the `fsds_simulator/` mirror and the live `fsae_planning` tree. The offline and live files differ only by a two-line comment.

Simulator output is unchanged when the bug cannot fire, and FSDS's cone perception is a noise-free oracle by default, so it never produced the same-frame duplicates needed to trigger it. This does not establish that the bug explains any part of the curvature-spike defect or the saturation gap. That needs either a measured real-detector noise figure or a live log showing duplicate clustering, and neither exists. See the log, section 15, including the `CONE_NOISE_ENABLED` capability the fix was verified against.

## Cone geometry is accurate

Track width and cone spacing match FSDS exactly and are not a source of the sim-to-real gap. The measurements (track width, spacing percentiles, FS rule limits) are in `docs/logs/sim_to_real_investigation.md`, section 11 ("Also verified: cone geometry is accurate").

**Measure spacing along the path, not down the array.** Cones are stored in recording order (`source: fsae_sim_perception.cone_recorder`), not sorted around the track, so consecutive entries are not spatially adjacent. Differencing the array reports phantom gaps of tens of metres. Project each cone onto its nearest centreline index and sort by that first.
