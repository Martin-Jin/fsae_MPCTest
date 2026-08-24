# Reference Path and Speed Profile

How the car decides **where** to drive and **how fast**, and how to change
either.

Both come from files generated offline from a recorded cone map, then read by
the controller at run time. Nothing here is computed live unless the
precomputed-path toggles are turned off, in which case the car falls back to
the live planner and a per-tick curvature speed estimate.

**Related documents**

- `docs/developer_guide.md`, "Recording, exporting and driving a track" — the
  step-by-step workflow for producing these files.
- `docs/tuning.md` — the controller weights that track the reference.
- `docs/reference/offline_live_parity.md` — the offline/live parity rules the numbers
  here are subject to.

## The two files, and which does what

| file | produced by | supplies | launch arg |
|---|---|---|---|
| `speed_profile.csv` | `tuner.tools.export_speed_profile` | the speed target | `map_path` / `$SPEED_CSV` |
| `raceline.csv` | `tuner.tools.raceline_optimizer` | path geometry | `path_map_path` / `$PATH_CSV` |
| `centerline.csv` | `tuner.tools.raceline_optimizer --mode centerline` | path geometry | `path_map_path` / `$PATH_CSV` |

All of them live in one directory per track,
`ros2/src/fsae_planning/tracks/<track>/`, alongside the `cone_map.json` they
were generated from. Both exporters write there directly.

Switching tracks or reference lines is a one-line edit in `ros2/launch_all.sh`:

```bash
TRACK=comp_test_map_3
SPEED_CSV="$TRACK_DIR/speed_profile.csv"
PATH_CSV="$TRACK_DIR/centerline.csv"      # or raceline.csv
```

**The speed file and the geometry file deliberately describe different lines.**
That looks wrong and has been "fixed" once by pointing both at the same file,
which regressed the car badly and was reverted. See "Speed-profile
aggressiveness" below and `launch_all.sh`'s own comment before changing the
pairing.

## Which file the car actually loads at launch — there is no auto-discovery

**Plain version:** the car does not look for "the most recent export" or scan
the tracks folder for anything. `launch_all.sh` names one exact folder and two
exact filenames as plain strings. Re-exporting a file overwrites the one
already there — there is no versioning, so "the newest one" and "the only one
that exists" are always the same file.

```bash
TRACK=comp_test_map_3
TRACK_DIR="$HOST_ROS2_DIR/src/fsae_planning/tracks/$TRACK"
SPEED_CSV="$TRACK_DIR/speed_profile.csv"
PATH_CSV="$TRACK_DIR/centerline.csv"
```

Consequences:

- **Switching tracks** means editing `TRACK=` in `launch_all.sh` to a
  different subfolder name under `ros2/src/fsae_planning/tracks/`. Nothing
  scans for "the most recently recorded track" — forgetting to change `TRACK=`
  drives whatever track that name still points at.
- **Switching reference line** (raceline vs centreline) means editing
  `PATH_CSV=` to the other filename in the same folder — see "The two files"
  above.
- **Re-running an exporter overwrites its output file in place.** There is no
  `_v2` suffix, no timestamp, no backup. Comparing an old export against a new
  one requires copying the old file aside first (or checking it into git — the
  `fsae_planning` tracks folder is a separate repo, see
  `docs/developer_guide.md`) before re-exporting.
- **The three variables are read once, at the top of `launch_all.sh`, before
  the launch command runs.** Editing them mid-run has no effect on an already-
  launched node; relaunch to pick up a change.

## `USE_PRECOMPUTED_SPEED`/`_PATH` are resolved in the launch file, identically for all three controllers

**Plain version:** whether the precomputed files get used at all is decided
once, in `control.launch.py`, before any controller node starts — not inside
each controller's own code. All three controllers (Stanley, MPC, MPC-
standalone) are handed the exact same result, so a Stanley run and an MPC run
on the same track are guaranteed to share the identical speed target and/or
path if that is what the toggles say.

Mechanism: `control.launch.py` computes `effective_map_path` and
`effective_path_map_path` with an `IfElseSubstitution` —
`USE_PRECOMPUTED_SPEED=true` passes `map_path` through as given,
`USE_PRECOMPUTED_SPEED=false` passes an empty string instead (regardless of
what `map_path` was set to), and the same for the path toggle. That
*resolved* value, not the raw `SPEED_CSV`/`PATH_CSV`, is what every
controller's `map_path`/`path_map_path` ROS parameter actually receives.

Consequence for reading the controller source: `stanley_controller.py` has no
`use_precomputed_speed`/`use_precomputed_path` parameter and never checks
either toggle — nor do the two MPC nodes. Reading only the node's own code
therefore looks like the toggles are ignored; they are not, they are applied
one layer up. All three nodes only ever see "load this file" (a non-empty
string) or "there is no file" (empty string), and treat those two cases
identically to each other.

## How the speed profile and the precomputed path are calculated

**Plain version.** Before the car drives a track, two files are produced from
the recorded cone map: one saying *where* to drive, one saying *how fast*.

The speed file is built in three sweeps:

1. Look at each point on the path and work out the fastest speed that corner
   can be taken at without exceeding the grip budget.
2. Sweep forwards, cutting any speed the car could not have accelerated up to
   from the point behind it.
3. Sweep backwards, cutting any speed the car could not brake down from in
   time for the corner ahead.

The result is a speed at every point that is both cornering-safe and
reachable by a real car. Without sweeps 2 and 3 the file can demand
impossible braking — measured at ~273 m/s² before they were added.

### The speed profile: `compute_speed_profile()` in `sim/speed_profile.py`

| pass | what it enforces | formula |
|---|---|---|
| 0 | corner speed from curvature | delegated to `curvature_speed()` at every point |
| 1 | forward, acceleration limit | `v[i] ≤ √(v[i−1]² + 2·a_accel_max·ds)` |
| 2 | backward, braking limit | `v[i] ≤ √(v[i+1]² + 2·|a_brake_max|·ds)` |

Points worth knowing:

- **Pass 0 is not a copy.** `compute_speed_profile()` calls the same
  `curvature_speed()` the live car uses, so the offline oracle and the live
  per-tick target cannot drift apart. `curvature_speed()` itself scans the
  next 24 m of path, takes the peak curvature in that window, and returns
  `safety·√(a_lat_max / κ_peak)` clamped to `[v_min, v_max]`.
- **`a_accel_max`/`a_brake_max` are planning values, not the plant's limits**
  (7.0 / −5.0 against the plant's 12.0 / −9.0). Planning at the true limits
  leaves the controller no margin for combined slip or model error and makes
  passes 1–2 nearly non-binding.
- **Passes 1–2 have no live counterpart.** The car relies on
  `curvature_speed()`'s 24 m look-ahead to see a corner early enough to brake.
  The passes exist offline so `speed_rmse` does not penalise the controller
  for failing to track an unreachable reference.
- **Closed-loop wrapping.** With `closed_loop=True` (the default, correct for
  a recorded lap) point *n−1* is treated as adjacent to point 0 and passes 1–2
  run twice in each direction, so a constraint crossing the start/finish seam
  propagates all the way round. Without it the car met an artificial slowdown
  at the line once per lap. Set `False` only for a genuinely open path, such
  as an acceleration event ending at a stop.

### The path geometry: `tuner/tools/raceline_optimizer.py`

Both modes start from a centreline reconstructed from the cone map, resampled
to even arc-length spacing.

- **`--mode raceline`** parameterises the line as a per-station lateral offset
  `alpha[i]` within the track's own width budget, then iteratively nudges each
  station toward lower curvature (a Kegel-style minimum-curvature search),
  re-profiling speed each round. Candidates are ranked by lap time *plus* a
  curvature penalty, so a kinked line cannot win on lap time alone. A final
  smoothing pass removes residual per-station noise.
- **`--mode centerline`** pins `alpha` to zero: no search, no smoothing. The
  exported path is the reconstructed centreline and only its speed profile is
  optimised.

Both modes then run the same cone-clearance check *before* writing anything,
so a line that passes too close to a cone fails the export rather than
silently shipping.

### Which file the car actually reads

`launch_all.sh` passes two separate paths, and they deliberately point at
different files:

| launch arg | variable | supplies |
|---|---|---|
| `map_path` | `SPEED_CSV` | the speed target |
| `path_map_path` | `PATH_CSV` | the path geometry |

Both are only consulted when `USE_PRECOMPUTED_SPEED` / `USE_PRECOMPUTED_PATH`
are true; otherwise the car falls back to the live planner and to
`curvature_speed()` computed per tick.

**The precomputed-speed branch applies no `v_max` clip**, so whatever
`SPEED_CSV` contains is commanded directly. That makes the choice of file a
speed-cap decision as much as a profile decision — check both files' ranges
before swapping either.

## Reference line: raceline vs centreline

**Plain version:** there are two ways to decide the line the car drives. A
*racing line* cuts corners for speed, running wide on entry and clipping the
apex. A *centreline* just follows the middle of the track. The racing line is
faster in principle, but it is harder to follow, and when something goes wrong
it is impossible to tell from the logs whether a large error means the car
missed the line or the line deliberately went near the edge. The centreline
removes that ambiguity — and on this track it is currently also faster in
practice.

`tuner/tools/raceline_optimizer.py` has two modes, selected by `--mode`, and
both write a 5-column `x,y,psi,psi_target,v_target` CSV that the live
controller consumes identically through `path_map_path`:

| mode | output | lateral offsets | use |
|---|---|---|---|
| `raceline` (default) | `raceline.csv` | curvature-minimising search | timed runs |
| `centerline` | `centerline.csv` | pinned to zero | diagnosis, and currently the better line on `comp_test_map_3` |

Centreline mode short-circuits the optimisation loop (`optimize_raceline`'s
`lateral_offsets=False`): no curvature-reduction search, no final smoothing
pass. The speed profile, heading shaping and cone-clearance check are the
same code in both modes. It writes a **different filename on purpose**, so
exporting a diagnostic line can never overwrite the raceline a timed run
depends on.

**Why a centreline is worth having at all.** On a raceline a large logged
`|e_y|` is ambiguous: the line deliberately sits near a boundary at an apex,
so "1.8 m from the path" is either a tracking failure or the line working as
designed, and the telemetry cannot distinguish them. On the centreline `|e_y|`
is unambiguously distance from the middle of the track, which is what makes a
"drove too close to the cones" report answerable from the log alone.

**On `comp_test_map_3` the centreline is not merely more legible — it is
faster.** Same controller settings, same `speed_profile.csv`, 3 laps each:

| | raceline | centreline |
|---|---|---|
| composite score | 0.752 | **0.488** |
| lap time | 54.50 s | **51.34 s** |
| RMSE | 0.506 | **0.366 m** |
| peak \|e_y\| | 1.804 | **1.004 m** |
| max \|e_psi\| | 37.6° | **22.0°** |
| steering saturation | 0.18% | **0.00%** |
| \|e_y\| > 1.0 m | 4.00% | **0.14%** |
| steering reversals | 13 | **1** |

At the corner that motivated this (`nmpc_s0` 170–195, the tightest on the
track) the raceline produced a 1.8 m excursion on two of three laps with the
car bogging to 2.19 m/s; the centreline holds 4.96 m/s minimum and peaks at
1.004 m. Faster overall **despite being 5.1 m longer** (470.6 vs 465.5 m),
because no lap is spent recovering from the excursion.

**The mechanism, and why it is a real optimiser bug.**

- The raceline's offset from the centreline is tiny: mean 0.13 m, max 0.48 m
  over the whole lap, and only 0.35 m through the failing corner.
- At that corner `|κ|` is 0.209 — the global maximum for this track, and about
  70% of the car's full-lock kinematic floor (1/3.32 m = 0.30).
- There is no width left to cut with, so the search bought no lap time. But
  the offset it did apply still perturbed the curvature of a corner already at
  the edge of what the plant can deliver: a marginally harder corner for no
  gain.
- `_candidate_score`'s `CURVATURE_SOFT_MAX` penalty does not catch this. It
  thresholds **absolute** curvature (0.22) rather than curvature against the
  `alat_ceiling` that the planned speed at that station actually permits.

Consequences:

- **Do not read the small mean offset as "the raceline is basically the
  centreline, so the choice cannot matter."** It was measured as 0.13 m mean
  and still cost 0.26 composite score. The offsets that matter are local to
  the one or two corners nearest the plant's limit.
- **The steering-quality metrics move with the reference, not only with the
  cost weights.** Reversals 13 → 1 and saturation → 0 came from changing the
  line, with every weight held fixed. A chatter or turn-in result measured on
  a reference the car cannot track is not attributable to the weight under
  test — see `docs/logs/steering_chatter_investigation.md`.
- Fixing the optimiser means constraining a candidate's `κ·v²` against
  `alat_ceiling_at(v)` per station, not `|κ|` against a flat constant. Not
  done.

## Speed-profile aggressiveness: `CURVATURE_SPEED_A_LAT_MAX`

Corner speed in the oracle profile comes from `v = √(a_lat_max / κ)`, so
`CURVATURE_SPEED_A_LAT_MAX` (`sim/speed_profile.py`) is the single knob that
sets how hard the car is willing to corner. `v_max`/`V_MAX` do NOT affect
corner speed at all — they are a flat top-speed clip that only ever binds on
the fastest straights, and on the precomputed-speed path they are not applied
at all (the CSV's own values are the target, see `launch_all.sh`'s
`SPEED_CSV` comment).

**It is not a launch arg.** The value is baked into
`tracks/<name>/speed_profile.csv` at export time. Changing it requires
editing `sim/speed_profile.py`, re-running
`python -m tuner.tools.export_speed_profile <map>`, and relaunching. It is
ALSO the live `control_utils.curvature_speed()` default (used when no
precomputed profile is loaded), so both sides must be changed together —
see the numeric-parity table above.

Scale reference: FSDS's measured sustained lateral-acceleration ceiling is
~7.5 m/s² (with bursts to ~12.3 observed on a lap), so this planning value
sits deliberately below the physical limit to leave margin for combined
slip, model-plant mismatch and actuation lag. Raising it makes every corner
faster proportionally to `√(a_lat_max)`; straights are unaffected once they
are already at the `v_max` ceiling.

**Caution when raising it:** the precomputed-speed branch applies no `v_max`
clip, so the exported CSV's values are commanded directly with nothing above
them to catch an over-aggressive profile. Step it up and measure rather than
jumping to the measured physical ceiling.

**Only `speed_profile.csv` responds to this constant. `raceline.csv` and
`centerline.csv` do not.** The two exporters plan corner speed from different
limits:

| exported file | exporter | corner-speed limit |
|---|---|---|
| `speed_profile.csv` | `tuner.tools.export_speed_profile` | `CURVATURE_SPEED_A_LAT_MAX` (this constant) |
| `raceline.csv`, `centerline.csv` | `tuner.tools.raceline_optimizer` | `alat_ceiling_at(v) × ALAT_MARGIN` (0.85) from `model/vehicle_physics.py` |

*Plain version:* the file that decides how fast to go and the file that decides
where to drive are produced by two different tools, and they work out safe
corner speeds in two different ways. Changing this constant and re-exporting
updates the first but silently leaves the second alone.

Consequence: after changing this constant, re-run **`export_speed_profile`**.
Re-running `raceline_optimizer --mode centerline` will report an unchanged
`v_target` range and that is correct, not a failed export — on
`comp_test_map_3` the current files read 6.00–18.00 (`speed_profile.csv`) and
5.54–16.70 (`centerline.csv`).

Which one actually reaches the car depends on `launch_all.sh`: `SPEED_CSV`
supplies the speed target and `PATH_CSV` supplies the geometry, and they
deliberately point at different files (see that script's own comment). With the
default pairing the speed the car tracks comes from `speed_profile.csv`, so
this constant is live-relevant even while the car drives `centerline.csv`.

**This value is a steering-smoothness parameter, not only a lap-time one.**

*Plain version:* this number decides how fast the car is allowed to plan to go
through corners. Set too high, the car arrives at the tightest corner faster
than it can physically turn — so the steering slams over, the car runs wide,
and it feels like a sudden jerk. Lowering it slightly made the steering much
smoother at a small cost in lap time.

Currently **4.75**, reduced from 5.5 after 5.5 was traced to a specific
sudden-steering-jump symptom. At 5.5 the car arrived at the track's hardest
curvature ramp (s0≈43→46, where the geometrically-required angle climbs
8.5°→15.2° in 2.7 m) carrying ~3 m/s more than its own target, which needs
roughly 15 m/s² of lateral acceleration — twice the plant's ~7.5 ceiling.
No steering policy can track that, so the command stalls near 9° and `e_psi`
runs away to −18°.

Live effect of 5.5 → 4.75, same controller settings:

| | 5.5 | 4.75 |
|---|---|---|
| stutters/min (sign flip, both \|d\|>1.5°) | 33.3 | **9.8** |
| \|d_steer\| > 5° events | 20 | **1** |
| max \|d_steer\| | 7.92° | **5.20°** |
| mean \|d2\| (jerk) | 0.904 | **0.604** |
| peak \|e_y\| | 1.250 | **1.008 m** |
| `a_lat` above 7.5 | 4.69% | **2.67%** |
| gain at the problem corner | 0.746 | **0.843** |

**Why this matters for tuning order:** four controller-side weight changes
(`r_rate_delta`, `nmpc_corner_factor_k`, `nmpc_q_e_y`, `NMPC_SQP_ITERS`) were
each tried against this symptom first and none of them moved it — see "Turn-in
timing" above. The steering command was already *early* (leading the geometric
requirement by 0.15 s) and at ~94% of the required magnitude; the binding
problem was the speed the car brought to the corner. **A "won't turn / jerks
at tight corners" report should be checked against `a_lat` demand and speed
overshoot before any steering weight is touched.**

Note the mechanism is not "the car brakes better" — speed overshoot relative
to target barely changed (hot ticks 13.31% → 13.14%). What changed is that the
target itself is lower, so the absolute speed and the required angle at the
curvature ramp are both smaller.

Also note `a_brake_max` (the `compute_speed_profile` braking pass) looked even
better on per-tick metrics at 3.5 — hot ticks to 0.00%, saturation 0.65% — but
**DNF'd in every offline variant tried**. Do not ship it without understanding
that failure.

## `launch_all.sh` has two launch branches; both must honour `SPEED_CSV`/`PATH_CSV`

**Plain version:** the launch script can start the software in two ways
depending on whether it is running inside a container. One of those two ways
used to ignore the setting that chooses which path file to drive, so changing
that setting appeared to do nothing.

`launch_all.sh` ends in an `if [ "$USE_DOCKER" = true ]` split, and
`USE_DOCKER` is **auto-detected**, not set by hand — so which branch runs is
not obvious from reading the config at the top of the file.

The Docker branch previously hard-coded the filenames:

```bash
map_path:=$CONTAINER_TRACK_DIR/speed_profile.csv
path_map_path:=$CONTAINER_TRACK_DIR/raceline.csv      # ignored PATH_CSV
```

It now derives them, matching the non-Docker branch:

```bash
map_path:=$CONTAINER_TRACK_DIR/$(basename "$SPEED_CSV")
path_map_path:=$CONTAINER_TRACK_DIR/$(basename "$PATH_CSV")
```

The container mounts the repo at a different root, so the host-side
`$SPEED_CSV`/`$PATH_CSV` paths cannot be passed through verbatim — only their
basenames, re-rooted at `$CONTAINER_TRACK_DIR`.

**Why this matters beyond the one-line fix:** with the old code, switching
`PATH_CSV` to `centerline.csv` silently drove the raceline on any Docker run,
and the telemetry header would have reported the raceline correctly while the
operator believed otherwise. **When a config change appears to have no effect,
check the launch header in the telemetry CSV** — `launch.path_map_path` and
`launch.map_path` record what the controller actually received.
