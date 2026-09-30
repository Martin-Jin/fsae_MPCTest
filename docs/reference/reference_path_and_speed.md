# Reference Path and Speed Profile

How the car decides where to drive and how fast, and how to change either.

Both come from files generated offline from a recorded cone map, then read by the controller at run time. Nothing here is computed live unless the precomputed toggles are off, in which case the car falls back to the live planner and a per-tick curvature speed estimate.

Related documents:

- [integration_guide.md](../fsds/integration_guide.md), "Recording, exporting and driving a track", for the step-by-step workflow that produces these files.
- [tuning.md](../guides/tuning.md), for the controller weights that track the reference.
- [offline_live_parity.md](offline_live_parity.md), for the parity rules the numbers here are subject to.

## Contents

1. [Three files per track, two describe the drive](#three-files-per-track-two-describe-the-drive)
2. [The launch script picks the track and the files](#the-launch-script-picks-the-track-and-the-files)
3. [The precomputed toggles are resolved in the launch file](#the-precomputed-toggles-are-resolved-in-the-launch-file)
4. [How the speed profile is calculated](#how-the-speed-profile-is-calculated)
5. [How the path geometry is calculated](#how-the-path-geometry-is-calculated)
6. [A flat or corner-only profile for low-speed testing](#a-flat-or-corner-only-profile-for-low-speed-testing)
7. [The centreline drives better than the raceline on this track](#the-centreline-drives-better-than-the-raceline-on-this-track)
8. [`CURVATURE_SPEED_A_LAT_MAX` sets corner aggressiveness](#curvature_speed_a_lat_max-sets-corner-aggressiveness)
9. [Both launch branches must honour `SPEED_CSV` and `PATH_CSV`](#both-launch-branches-must-honour-speed_csv-and-path_csv)

## Three files per track, two describe the drive

Each track lives in one directory, `ros2/src/fsae_planning/tracks/<track>/`, beside the `cone_map.json` the files were generated from. Both exporters write there directly.

| file | columns | produced by | supplies | launch argument |
|---|---|---|---|---|
| `speed_profile.csv` | `x,y,psi,v_target` | `python -m tuner.tools.export_speed_profile` | the speed target | `map_path`, from `$SPEED_CSV` |
| `raceline.csv` | `x,y,psi,psi_target,v_target` | `python -m tuner.tools.raceline_optimizer` | path geometry | `path_map_path`, from `$PATH_CSV` |
| `centerline.csv` | `x,y,psi,psi_target,v_target` | `python -m tuner.tools.raceline_optimizer --mode centerline` | path geometry | `path_map_path`, from `$PATH_CSV` |
| `tracks/comp_test_map_3/speed_profile_corner_test.csv` | `x,y,psi,v_target` | `python -m tuner.tools.export_speed_profile --corner-slowdown KAPPA` | an alternative speed target for testing | `map_path`, by editing `$SPEED_CSV` |

**The speed file and the geometry file deliberately describe different lines.** That looks wrong, but pointing both at the same file regressed tracking badly before (RMSE and steering saturation both blew up). See [CURVATURE_SPEED_A_LAT_MAX](#curvature_speed_a_lat_max-sets-corner-aggressiveness) and the comment in `ros2/launch_all.sh` before changing the pairing.

**The precomputed-speed branch applies no `v_max` clip.** Whatever `SPEED_CSV` contains is commanded directly, so its top speed is the car's top speed. Choosing the file is a speed-cap decision as much as a profile decision. Check both files' `v_target` ranges before swapping either. On `comp_test_map_3` the current files read 5.998 to 16.5 m/s (`speed_profile.csv`) and 5.536 to 16.7 m/s (`centerline.csv`). These ranges move whenever a file is re-exported.

## The launch script picks the track and the files

**Plain version.** By default the car drives the most recently recorded track and the newest geometry file for it, preferring the centreline over the raceline. Nothing has to be typed to pick these up. `TRACK=` pins a specific track instead.

`ros2/launch_all.sh` resolves the variables once, at the top, before the launch command runs.

- **`TRACK`** defaults to the newest track directory, chosen by the modification time of its `cone_map.json`, not by parsing a date from the directory name. An undated legacy track and a re-recorded track (which refreshes its directory's time without renaming it) both resolve as newest when they are. The logic is `_newest_track` in the script, written in bash because the script must run with no `fsae_MPCTest` checkout present. The Python equivalent is `tracks.newest_track()`.
- **The script currently pins `TRACK=comp_test_map_3`** as an active line. While that line is set, auto-discovery is bypassed. Comment it out to return to newest-track selection.
- **`PATH_CSV`** defaults to the newest export inside the track, preferring `centerline.csv` over `raceline.csv` (`_track_geometry_name`, matching `tracks.geometry_path()`). Set `PATH_CSV` explicitly, for example to `"$TRACK_DIR/raceline.csv"`, to force the raceline for a timed run.
- **`SPEED_CSV`** is not resolved this way. It always points at `speed_profile.csv`, because there is one speed exporter and one output name.
- **Editing the variables mid-run has no effect** on an already-launched node. Relaunch.

```bash
TRACK=comp_test_map_3
TRACK_DIR="$HOST_ROS2_DIR/src/fsae_planning/tracks/$TRACK"
SPEED_CSV="$TRACK_DIR/speed_profile.csv"
PATH_CSV="$TRACK_DIR/$_TRACK_GEOMETRY_NAME"   # centerline.csv, else raceline.csv
```

**Recording a new track.** A new `TRACK=` name is dated automatically as `<name>_<YYYYmmdd>` the moment the script sees the directory does not exist (matching `tracks.dated_track_name()`), so two recordings under one base name on different days do not overwrite each other. Re-recording an existing track refreshes its cone map in place and is not dated again. While recording, set both precomputed toggles to `false`, otherwise the launch replays the old track.

**Overwrite protection.** `export_speed_profile` and `raceline_optimizer` both accept `--no-overwrite`, which raises an error instead of replacing an existing file. Overwrite is the default, since re-exporting after retuning is the common case.

`ls ros2/src/fsae_planning/tracks` still lists every track. Discovery hides and deletes nothing. The script also fails early if `USE_PRECOMPUTED_PATH` or `USE_PRECOMPUTED_SPEED` is true and the file is missing, instead of silently falling back to the live planner.

## The precomputed toggles are resolved in the launch file

**Plain version.** Whether the precomputed files are used at all is decided once, in `control.launch.py`, before any controller starts. Every controller (Stanley, and the MPC in either `standalone_output` mode) is handed the same result. A Stanley run and an MPC run on the same track therefore share the same speed target and path if that is what the toggles say.

Mechanism: `control.launch.py` computes `effective_map_path` and `effective_path_map_path` with an `IfElseSubstitution`. With `USE_PRECOMPUTED_SPEED=true` it passes `map_path` through, and with `false` it passes an empty string whatever `map_path` was. The path toggle works the same way. That resolved value, not the raw `SPEED_CSV` or `PATH_CSV`, is what each node's `map_path` and `path_map_path` parameter receives.

Reading a controller's source alone makes it look as if the toggles are ignored. No node declares a `use_precomputed_speed` or `use_precomputed_path` parameter, because the toggles are applied one layer up. Each node sees only "load this file" (non-empty) or "no file" (empty).

## How the speed profile is calculated

**Plain version.** Before the car drives a track, one file says where to drive and another says how fast. The speed file is built in three sweeps.

1. For each point on the path, find the fastest speed the corner can be taken at without exceeding the grip budget.
2. Sweep forwards, cutting any speed the car could not have accelerated up to from the point behind.
3. Sweep backwards, cutting any speed the car could not brake down from in time for the corner ahead.

The result is a speed at every point that is safe for the corner and reachable by a real car. Without sweeps 2 and 3 the file demanded impossible braking, measured at about 273 m/s^2 before they were added.

`compute_speed_profile()` in `sim/speed_profile.py`:

| pass | what it enforces | formula |
|---|---|---|
| 0 | corner speed from curvature | `curvature_speed()` at every point |
| 1 | forward, acceleration limit | `v[i] <= sqrt(v[i-1]^2 + 2*a_accel_max*ds)` |
| 2 | backward, braking limit | `v[i] <= sqrt(v[i+1]^2 + 2*abs(a_brake_max)*ds)` |

- **Pass 0 is not a copy.** It calls the same `curvature_speed()` the live car uses, so the oracle profile and the live per-tick target cannot drift apart. `curvature_speed()` scans the next 24 m of path (`scan_end`), takes the peak curvature there, and returns `safety * sqrt(a_lat_max / kappa_peak)` clamped to `[v_min, v_max]`. `compute_speed_profile()` defaults `v_max` to `CURVATURE_SPEED_V_MAX = 16.5`, which is why the exported file tops out at 16.5 m/s.
- **`a_accel_max` and `a_brake_max` are planning values, not the plant's limits.** The defaults are 7.0 and -5.0 m/s^2 against the plant's 12.0 and -7.0. Planning at the true limits leaves no margin for combined slip or model error and makes passes 1 and 2 nearly non-binding.
- **Passes 1 and 2 have no live counterpart.** The car relies on `curvature_speed()`'s 24 m look-ahead to see a corner early enough to brake, and on `A_BRAKE_PLAN = 5.0` to propagate braking distance. The passes exist so `speed_rmse` does not punish the controller for failing to track an unreachable reference.
- **Closed-loop wrapping.** With `closed_loop=True` (default, correct for a recorded lap) the last point is adjacent to point 0 and passes 1 and 2 run twice in each direction, so a constraint that crosses the start and finish seam propagates all the way round. Without it the car met an artificial slowdown at the line each lap. Use `--open-loop` on the exporter only for an open path, such as an acceleration run that ends at a stop.

## How the path geometry is calculated

`tuner/tools/raceline_optimizer.py` starts both modes from a centreline rebuilt from the cone map and resampled to even arc-length spacing.

- **`--mode raceline`** (default) parameterises the line as a per-station lateral offset `alpha[i]` within the track's own width budget, then iteratively nudges each station toward lower curvature (a Kegel-style minimum-curvature search), re-profiling speed each round. Candidates are ranked by lap time plus a curvature penalty (`CURVATURE_SOFT_MAX = 0.22` per metre, 50 seconds per unit of excess), so a kinked line cannot win on lap time alone. A final smoothing pass removes residual noise.
- **`--mode centerline`** pins `alpha` to zero. There is no search and no smoothing. The exported path is the reconstructed centreline and only its speed profile is optimised. It writes `centerline.csv`, never `raceline.csv`, so exporting a diagnostic line cannot overwrite the raceline a timed run depends on.

Both modes run the same cone-clearance check before writing, so a line that passes too close to a cone fails the export instead of shipping. Both write the 5-column CSV, which the live controller consumes identically.

A greedy per-station nudge was chosen over a full minimum-curvature QP because it needs no extra solver dependency and converges to a good, not provably optimal, line. That is adequate for a reference the MPC tracks.

## A flat or corner-only profile for low-speed testing

**Plain version.** Testing how the car turns at low speed is slow if it crawls the whole lap to reach a corner. The corner-slowdown export drives at normal speed everywhere and slows only where the path curves, so a test run reaches the corner at full pace, then slows for it. With a threshold of zero it instead flattens the whole lap to one speed, for a constant-speed test.

`python -m tuner.tools.export_speed_profile <track> --corner-slowdown KAPPA` (`sim.speed_profile.compute_corner_slowdown_profile()`):

1. Computes the ordinary curvature-limited profile, the same content as `speed_profile.csv`.
2. Clamps every station where `|kappa| > KAPPA` to `min(existing, corner_speed)` (`--corner-speed`, default 3 m/s). It uses `min()`, not an overwrite, so a `corner_speed` above what the curvature limit already demands cannot raise the target.
3. Re-runs the forward and backward propagation passes over the clamped profile, so the step down to `corner_speed` becomes a reachable deceleration zone, not an instant drop.

It writes the corner-test file (named for corner testing, beside the other exports), never `speed_profile.csv`. To use it, point `SPEED_CSV` at it in `launch_all.sh` for the test, then back afterwards.

**Picking `KAPPA`.** Run `--corner-slowdown` with no value to print the track's curvature distribution and, per candidate threshold, how many contiguous corner zones it flags. Too low a threshold catches every gentle bend, which is the whole-lap-slow problem. Too high catches nothing. On `comp_test_map_3`:

| threshold | points flagged | corner zones |
|---|---|---|
| 0.06 | 340 of 1000 | 9 |
| 0.08 | 234 of 1000 | 9 |
| 0.10 | 123 of 1000 | 9 |
| 0.12 | 73 of 1000 | 6 |
| 0.15 | 23 of 1000 | 4 |

The curvature percentiles are p50 0.027, p75 0.079, p90 0.108, p99 0.163 and a maximum of 0.208 per metre. A threshold of 0.10 flags nine zones and skips the gentle bends.

The corner-test file currently on disk (`tracks/comp_test_map_3/speed_profile_corner_test.csv`) is the flat variant: every point is 3.0 m/s (its header reads "kappa>0.0 -> 3.0 m/s"). The comment in `ros2/launch_all.sh` describes this file as the flat 3 m/s profile and gives the regenerating command (`--corner-slowdown 0 --corner-speed <m/s>`).

## The centreline drives better than the raceline on this track

**Plain version.** A racing line cuts corners for speed, running wide on entry and clipping the apex. A centreline follows the middle of the track. The racing line is faster in principle but harder to follow. When something goes wrong, the logs cannot say whether a large error means the car missed the line or the line deliberately went near the edge. The centreline removes that ambiguity. On `comp_test_map_3` it is also faster in practice.

| mode | output | lateral offsets | use |
|---|---|---|---|
| `raceline` (default) | `raceline.csv` | curvature-minimising search | timed runs |
| `centerline` | `centerline.csv` | pinned to zero | diagnosis, and currently the better line on `comp_test_map_3` |

Centreline mode short-circuits the optimisation (`optimize_raceline`'s `lateral_offsets=False`). The speed profile, heading shaping and clearance check are the same code in both modes.

**Why a centreline is worth having.** On a raceline a large logged `|e_y|` is ambiguous, because the line sits near a boundary at an apex on purpose. On the centreline `|e_y|` is the distance from the middle of the track, which makes a "drove too close to the cones" report answerable from the log alone.

**Measured.** Same controller settings, same `speed_profile.csv`, 3 laps each, NMPC. The run used `r_rate_delta=52.5`, `nmpc_rjerk_delta=150.0`, `q_e_y=6.35`, smoothing off and the three-zone schedule off, which differs from current defaults. The source is `docs/logs/steering_chatter_investigation.md`.

| | raceline | centreline |
|---|---|---|
| composite score | 0.752 | 0.488 |
| lap time | 54.50 s | 51.34 s |
| RMSE | 0.506 m | 0.366 m |
| peak `\|e_y\|` | 1.804 m | 1.004 m |
| max `\|e_psi\|` | 37.6 deg | 22.0 deg |
| steering saturation | 0.18% | 0.00% |
| `\|e_y\| > 1.0 m` | 4.00% | 0.14% |
| steering reversals | 13 | 1 |

At the corner that motivated it (`nmpc_s0` 170 to 195, the tightest on the track) the raceline produced a 1.8 m excursion on two of three laps with the car slowing to 2.19 m/s. The centreline held a 4.96 m/s minimum and peaked at 1.004 m. These corner figures and the lap lengths below are carried from an earlier write-up and are not in `docs/logs/`, so they are not re-verified. The centreline was faster despite being 5.1 m longer (470.6 m against 465.5 m), because no lap is spent recovering from an excursion.

**The mechanism is an optimiser flaw.**

- The raceline's offset from the centreline is small: 0.13 m mean and 0.48 m maximum over the lap, and only 0.35 m through the failing corner.
- At that corner `|kappa|` is about 0.21, the maximum for the track and about 70% of the car's full-lock kinematic floor (1/3.32 m = 0.30, from `L / tan(max_steer)` with `L = 1.55 m`).
- There is no width left to cut with, so the search bought no lap time. The offset it did apply still perturbed the curvature of a corner already at the edge of what the plant delivers.
- `_candidate_score`'s `CURVATURE_SOFT_MAX` penalty does not catch this. It thresholds absolute curvature (0.22) and not curvature against the `alat_ceiling` that the planned speed at that station permits.

Consequences:

- **A small mean offset does not mean the choice cannot matter.** 0.13 m mean still cost 0.26 composite score. The offsets that matter are local to the one or two corners nearest the plant's limit.
- **Steering-quality metrics move with the reference, not only with the weights.** Reversals 13 to 1 and saturation to 0 came from changing the line with every weight held fixed. A chatter or turn-in result measured on a reference the car cannot track is not attributable to the weight under test. See `docs/logs/steering_chatter_investigation.md`.
- **The fix is not done.** It means constraining a candidate's `kappa * v^2` against `alat_ceiling_at(v)` per station, not `|kappa|` against a flat constant.

## `CURVATURE_SPEED_A_LAT_MAX` sets corner aggressiveness

Corner speed in the oracle profile comes from `v = sqrt(a_lat_max / kappa)`, so `CURVATURE_SPEED_A_LAT_MAX` in `sim/speed_profile.py` (4.75) is the single knob for how hard the car is willing to corner. `v_max` and `V_MAX` do not affect corner speed. They are a flat top-speed clip that binds only on the fastest straights, and on the precomputed-speed path they are not applied at all.

**It is not a launch argument.** The value is baked into `speed_profile.csv` at export time. Changing it means editing `sim/speed_profile.py`, re-running `python -m tuner.tools.export_speed_profile <track>`, and relaunching. It is also the default `a_lat_max` of the live `control_utils.curvature_speed()` (used when no precomputed profile is loaded), so both sides change together. See [offline_live_parity.md](offline_live_parity.md).

**Scale.** FSDS's measured sustained lateral-acceleration ceiling is about 7.5 m/s^2 at low speed, rising with speed (see [simulator_fidelity.md](simulator_fidelity.md)). The planning value sits below it to leave margin for combined slip, model-plant mismatch and actuation lag. Raising it makes every corner faster in proportion to `sqrt(a_lat_max)`. Because the precomputed branch applies no `v_max` clip, the exported values are commanded directly with nothing above to catch an over-aggressive profile. Step it up and measure.

**Only `speed_profile.csv` responds to this constant.** The two exporters plan corner speed from different limits.

| exported file | exporter | corner-speed limit |
|---|---|---|
| `speed_profile.csv` | `export_speed_profile` | `CURVATURE_SPEED_A_LAT_MAX` |
| `raceline.csv`, `centerline.csv` | `raceline_optimizer` | `alat_ceiling_at(v) * ALAT_MARGIN` (0.85), from `model/vehicle_physics/params.py` |

*Plain version.* The file that decides how fast to go and the file that decides where to drive come from two tools that work out safe corner speeds in two ways. Changing this constant and re-exporting updates the first and leaves the second alone. After changing it, re-run `export_speed_profile`. Re-running `raceline_optimizer` reports an unchanged `v_target` range, and that is correct, not a failed export.

With the default pairing the speed the car tracks comes from `speed_profile.csv`, so the constant is live-relevant even while the car drives `centerline.csv`.

### It is a steering-smoothness parameter, not only a lap-time one

*Plain version.* This number sets how fast the car plans to take corners. Set too high, the car arrives at the tightest corner faster than it can turn, the steering slams over, the car runs wide, and it feels like a sudden jerk. Lowering it slightly made the steering much smoother at a small cost in lap time.

At 5.5 the car arrives at the track's hardest curvature ramp (`s0` about 43 to 46, where the required angle climbs from 8.5 to 15.2 degrees in 2.7 m) carrying about 3 m/s more than its own target. That needs roughly 15 m/s^2 of lateral acceleration, twice the plant's ceiling. No steering policy can track it, so the command stalls near 9 degrees and `e_psi` runs away to -18 degrees.

Live effect of 5.5 to 4.75, same controller settings (source: `docs/logs/steering_chatter_investigation.md`):

| | 5.5 | 4.75 |
|---|---|---|
| stutters per minute (sign flip, both `\|d\| > 1.5 deg`) | 33.3 | 9.8 |
| `\|d_steer\| > 5 deg` events | 20 | 1 |
| max `\|d_steer\|` | 7.92 deg | 5.20 deg |
| mean `\|d2\|` (jerk) | 0.904 | 0.604 |
| peak `\|e_y\|` | 1.250 m | 1.008 m |
| `a_lat` above 7.5 | 4.69% | 2.67% |
| gain at the problem corner | 0.746 | 0.843 |

**Tuning order.** Four controller-side changes (`r_rate_delta`, `nmpc_corner_factor_k`, `nmpc_q_e_y`, `NMPC_SQP_ITERS`) were each tried against this symptom first and none moved it. The steering command already led the geometric requirement by 0.15 s at about 94% of the required magnitude. The binding problem was the speed the car brought to the corner. **For a "won't turn" or "jerks at tight corners" report, check `a_lat` demand and speed overshoot before touching any steering weight.**

The mechanism is not that the car brakes better. Speed overshoot relative to target barely changed (hot ticks 13.31% to 13.14%). The target itself is lower, so the absolute speed and the required angle at the curvature ramp are both smaller.

`a_brake_max` at -3.5 looked better on per-tick metrics (hot ticks 0.00%, saturation 0.65%) but DNF'd in every offline variant tried. Do not ship it without understanding that failure. The figures in this paragraph and the "hot ticks" figures above are not in `docs/logs/`, so they are not re-verified.

## Both launch branches must honour `SPEED_CSV` and `PATH_CSV`

**Plain version.** The launch script starts the software one of two ways depending on whether it runs inside a container. Both must respect the setting that chooses which path file to drive, or changing it appears to do nothing.

`launch_all.sh` ends in an `if [ "$USE_DOCKER" = true ]` split, and `USE_DOCKER` is auto-detected (native ROS 2 install present means false), not set by hand. The container mounts the repo at a different root, so host paths cannot pass through. The Docker branch derives the filenames from the variables and re-roots them at `$CONTAINER_TRACK_DIR`.

```bash
map_path:=$CONTAINER_TRACK_DIR/$(basename "$SPEED_CSV")
path_map_path:=$CONTAINER_TRACK_DIR/$(basename "$PATH_CSV")
```

If a Docker branch hard-coded a filename instead of deriving it, switching `PATH_CSV` would silently drive the wrong file on Docker runs, while the telemetry header still reports whichever file was loaded. When a config change appears to have no effect, check the launch header in the telemetry CSV: `launch.path_map_path` and `launch.map_path` record what the controller received.
