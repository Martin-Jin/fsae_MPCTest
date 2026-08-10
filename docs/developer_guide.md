# Developer Guide

Practical how-to content: running the simulator and offline tuner, FSDS/ROS 2
integration, manual drive mode, dependencies, and guidelines for extending the
project (new synthetic paths, debugging solver failures). For the deep
technical explanation of *why* the system is built this way, see
[Architecture](architecture.md).

## Table of Contents

1. [Running the Simulator](#running-the-simulator)
2. [Running the Offline Tuner](#running-the-offline-tuner)
3. [Simulator integration](#simulator-integration)
4. [Manual Drive Mode](#manual-drive-mode)
5. [Dependencies](#dependencies)
6. [Extending and Debugging](#extending-and-debugging)

---

## Running the Simulator

The simulator (`gui/simulation.py`) is an interactive matplotlib GUI for drawing
or loading a path, running one closed-loop MPC rollout against the nonlinear
vehicle plant, and reviewing the result frame by frame.

### 1. Install dependencies

```bash
pip install numpy scipy matplotlib cvxpy cma
pip install cvxpy[osqp] cvxpy[clarabel]
```

### 2. Launch

```bash
cd /path/to/project
python -m gui.simulation
```

### 3. Get a path onto the map

Either:

- **Draw one** — click and drag on the map (at least 6 points). On release the
  path is automatically splined, headings computed, and a speed profile
  generated.
- **Load a synthetic one** — click **Load Test Path** to cycle through the
  10 built-in FS-spec paths (`PATH_SUDDEN_TURN`, `PATH_S_BEND`, `PATH_SPIRAL`,
  `PATH_MICRO_SLALOM`, `PATH_OFFSET_CHICANE`, `PATH_ACCELERATION`,
  `PATH_HAIRPIN`, `PATH_CHICANE`, `PATH_FS_CORNER`, `PATH_MIXED`). Each click
  advances to the next path; the camera auto-frames around it with a 15 m
  margin.
- **Load a recorded track** — click **Load Recorded Track** to cycle
  (newest-first) through `tracks/*/cone_map.json` (and, for captures predating
  that layout, `fsds_simulator/cone_maps/*.json`), the cone maps written by
  `fsae_planning`'s `cone_recorder` ROS 2 node after a live FSDS lap (see
  [Recording, exporting and driving a track](#recording-exporting-and-driving-a-track)
  below). Unlike the synthetic paths, the blue/yellow cones rendered are the
  *actual recorded cones*, not `place_cones()` output — a real perception
  recording, resimulated exactly as `SimPerception`/`SimPlanner` would drive
  it live. The centreline drawn on load is only a reconstruction for the
  oracle-mode reference path and initial camera framing (see
  `sim/track_io.py`). With `USE_PLANNER = True`, the actual driving line
  during the rollout instead comes from `SimPlanner` rebuilding it
  cone-by-cone, exactly as for a synthetic path — but `USE_PLANNER = False`
  is now the default (2026-08-08), so by default the rollout tracks this
  reconstructed oracle path/speed profile directly, matching the live ROS
  side's `path_map_path` mode.

### 4. (Optional) set initial conditions

Once a path exists, two sliders appear:

- **Initial Lat Error** (±4 m) — starts the car offset sideways from the path.
- **Initial Yaw Error** (±30°) — starts the car pointing the wrong way.

Useful for stress-testing recovery behaviour rather than always starting
perfectly on-line.

### 5. Run it

Click **Start Sim**. The rollout runs synchronously (no live animation while
it solves — this can take a few seconds for a long path). When it finishes,
the title turns green and a **Time** scrub slider appears below the map.

### 6. Review the run

Drag the **Time** slider to replay the run frame by frame. The trail, the
cyan MPC horizon prediction, the car marker, and the telemetry panel (speed,
position, heading, tracking errors, steering/accel commands) all update
together.

### 7. Score it

Click **Show Metrics** to print a full 13-metric breakdown to the console
(see [Composite Score](architecture.md#the-composite-score) below) and show a one-line
summary in the plot title. Click **Benchmark All Paths** to run every
synthetic path 3× each with the currently loaded weights and print a
per-path score table — useful for checking a weight set generalises rather
than only working on the path you happened to test.

### 8. Reset

Click **Reset Environment** to clear everything and start over.

---

## Running the Offline Tuner

The offline tuner (`tuner/offline_tuner.py`) automatically searches for `Q`, `R`,
`R_rate` cost weights that minimise the [composite score](architecture.md#the-composite-score)
across a library of synthetic corner shapes, using CMA-ES (see
[How the Offline Tuner Works](architecture.md#how-the-offline-tuner-works) for the algorithm
itself). It has no GUI — it's a long-running batch job you leave to finish.

### 1. Install dependencies

Same as the simulator (see above) — `tuner/offline_tuner.py` uses the same
`cvxpy`/`osqp`/`clarabel`/`cma` stack, plus Python's built-in
`multiprocessing` to spread rollouts across CPU cores.

### 2. Check `settings.py` first

Before running, confirm:

- `VALIDATION_SUITE` lists the corner shapes you want the tuner to optimise
  for (see [Configuring the Project](architecture.md#configuring-the-project-settingspy)).
- `MAX_EVALS` is set to a budget you're happy to wait for (a good run is
  20 minutes to a few hours depending on core count and `MAX_EVALS`).
- `USE_PLANNER` reflects whether you want the tuner testing the full
  perception/planning pipeline (`True`) or driving on the perfect
  reference line (`False`, default as of 2026-08-08, also faster).
- `USE_OPTUNA_PRESEARCH` (default `True`) — set `False` to skip the short
  Optuna TPE search that runs before CMA-ES starts and seeds its starting
  point, falling back instead to the fixed geometric midpoint (see
  [Optional Optuna TPE pre-search](architecture.md#optional-optuna-tpe-pre-search)).
  Requires `optuna` to be installed (see
  [Dependencies](#dependencies)). Set `False` for the exact pre-2026-08-05
  behaviour.

### 3. Launch

```bash
cd /path/to/project
python -m tuner.offline_tuner
```

This uses all available CPU cores minus one (one is left free for the OS).
If `USE_OPTUNA_PRESEARCH` is enabled, the Optuna TPE pre-pass runs first and
prints one line per trial, then a short summary before CMA-ES begins:

```
[Optuna TPE] trial   12/ 375 | score 0.3120 | best 0.1896
[Offline Tuner] Optuna pre-pass done in 8.42 min | trials run: 375/375 | best score: 0.1896
  x0 (Optuna-seeded): [2.451, 0.873, 4.201, 1.05, 0.612, 3.31, 0.774, 2.9, 5.14]
```

CMA-ES then starts from that seeded point instead of the fixed midpoint.
Progress prints once per CMA-ES generation:

```
[lq-CMA-ES] gen    5 | true_evals    90 | gen_best 0.2341 | overall_best 0.1892 | sigma 6.123e-01
```

`gen_best` is this generation's best score; `overall_best` is the best score
seen so far across the whole run; `sigma` is CMA-ES's current search-radius
(shrinks as it converges). Lower scores are better throughout.

You can safely stop early with **Ctrl+C** — the tuner finishes its current
generation, then reports the best weights found so far rather than exiting
uncleanly.

### 4. Read the result

On completion (or early stop), the tuner prints the best weight arrays found:

```
Replace your gui/simulation.py weights with:
Q_diag      = [9.35, 22.2, 18.9, 49.8, 10.8, 0.0, 0.0, 0.0]
R_diag      = [49.3, 45.4]
R_rate_diag = [50.0, 49.6]
```

It also prints a list of "improvement milestones" — the point in the search
(by true-evaluation count) at which each meaningfully better score was found,
so you can see how much of the run's time was actually productive.

### 5. Apply the weights

Copy the three arrays into **both**:

- `settings.py` — `Q_diag`, `R_diag`, `R_rate_diag` (used by `gui/simulation.py`
  and, from there, everything that imports them)
- `mpc_core.py` — the same three arrays hardcoded inside
  `MPCController.__init__` (used by the live ROS 2 controller)

Both must stay in sync manually — the tuner was designed against the same
plant and horizon used by both, but there is currently no single shared
import between them (`mpc_core.py` is a standalone file so the live
ROS 2 node has no simulator dependencies).

### 6. Log the result

Every run appends its result to `tuning_history.txt` automatically
(timestamp, weight diagonals, duration, tuner score, git commit hash). Go
back and manually fill in the `Overall score` field once you've actually
tested the weights in FSDS or on the real car — the offline tuner score
alone doesn't perfectly predict real-world performance, so this file is
where the two get reconciled over time. See existing entries in
`tuning_history.txt` for the expected format.

### Key constants to adjust

All of these live in `settings.py`, not `tuner/offline_tuner.py` — see the next
section for what each one does and how much to change it by:

```python
MAX_EVALS         # Total true rollout budget (surrogate reduces actual count ~3-10x)
VALIDATION_SUITE  # Which synthetic corner shapes the tuner scores against
```

`sigma0` (CMA-ES's initial search radius) and `max_restarts` (BIPOP restart
budget) are algorithm-internal tuning knobs rather than project settings —
they're set near the bottom of `tuner/offline_tuner.py`'s `__main__` block if you
need to adjust them; see [How the Offline Tuner Works](architecture.md#how-the-offline-tuner-works)
for what they control.

---

## Simulator integration

`fsds_simulator/` in this repo is a full staging mirror of `fsae_planning`'s
own ROS 2 workspace — every package (`fsae_interfaces`, `fsae_bringup`,
`fsae_sim_perception`, `fsae_planning`, `fsae_control`), not just the
control-layer files, e.g. `fsds_simulator/control/fsae_control/
fsae_control/mpc_core.py` sits at the exact relative path it needs to land at
inside a `fsae_planning` checkout. There are two ways to use it:

- **If you already have a `fsae_planning` checkout**, copy everything under
  `fsds_simulator/control/`, `fsds_simulator/perception/`,
  `fsds_simulator/common/`, and `fsds_simulator/planning/` in this repo over
  the matching paths inside `fsae_planning` (same relative structure, so
  it's a straight directory copy, not a manual file-by-file paste).
- **If you don't**, `fsds_simulator/` alone (plus FSDS and the two message
  repos it depends on) is enough to build a working workspace from scratch —
  see [fsds_simulator/README.md](../fsds_simulator/README.md).

See [docs/planning_control_sync.md](planning_control_sync.md) for the full
file mapping and what's a deliberate non-mirror.
(If you already have the simulator set up with the `fsae_planning` repo. Scroll down for installing from scratch on windows.)

**Choosing the controller and planner:**

`fsae_bringup`'s `sim.launch.py` takes `controller` and `planner` as launch
arguments — it doesn't need editing to switch between them. It also launches
`cone_recorder` alongside the stack by default (see
[Recording, exporting and driving a track](#recording-exporting-and-driving-a-track) below), so
`launch_all.sh` / a bare `ros2 launch fsae_bringup sim.launch.py` gives you
MPC + cone recording in one command, no second terminal needed:

```bash
ros2 launch fsae_bringup sim.launch.py                              # mpc_standalone (default), cone_recorder on
ros2 launch fsae_bringup sim.launch.py controller:=stanley
ros2 launch fsae_bringup sim.launch.py controller:=mpc
ros2 launch fsae_bringup sim.launch.py planner:=skidpad_planner controller:=mpc
ros2 launch fsae_bringup sim.launch.py record_cones:=false          # skip cone_recorder
```

- `stanley` and `mpc` both publish the shared `cmd_vel` interface;
  `fsds_bridge` converts it to `fs_msgs/ControlCommand` and owns GO-gating +
  cone e-braking for either one.
- `mpc_standalone` (default) is this repo's `mpc_controller_standalone.py` —
  see above for its control-loop phases. It publishes `ControlCommand`
  directly and skips `fsds_bridge` (the launch file handles that
  automatically).

**Topic map for the control node:**

```
/fsds/testing_only/track       → sim_perception   → /fsae/slam/left_track
                                                   → /fsae/slam/right_track
                                                   → /fsae/perception/cone_detection
/fsds/testing_only/odom        → sim_perception   → /fsae/slam/car_odom
                                  centerline_planner (via car_position)

/fsae/slam/left_track,
/fsae/slam/right_track,
/fsae/slam/car_position        → centerline_planner → /fsae/planning/selected_trajectory

/fsae/planning/selected_trajectory  → mpc_controller_standalone   → /fsds/control_command
/fsae/slam/car_position             → mpc_controller_standalone
/fsae/slam/car_odom                 → mpc_controller_standalone  (SAME snapshot as car_position;
                                                                   see sim_perception.py's "Speed/
                                                                   yaw-rate synchronisation" note —
                                                                   do NOT use the raw
                                                                   /fsds/testing_only/odom directly)
/fsae/perception/cone_detection     → mpc_controller_standalone  (cone proximity brake)

/fsds/signal/go                → mpc_controller_standalone  (unlock)
```

Note: `mpc_controller_standalone` does not subscribe to a desired-speed
topic — it computes `desired_speed` itself every tick from the current path
via `control_utils.curvature_speed()` (see the `v_max`/`v_min` ROS
parameters it declares, which default to `V_MAX`/`V_MIN`). Also note
`mpc_controller_standalone` publishes `fs_msgs/ControlCommand` **directly** —
it does *not* go through `fsds_bridge.py` (upstream's shared GO-gating/
cone-brake/throttle-conversion layer that Stanley and upstream's default
`mpc_controller.py` both use). Don't launch `fsds_bridge` alongside
`mpc_controller_standalone` — they would both publish to
`/fsds/control_command`. (`control.launch.py`'s `controller:=mpc_standalone`
option already handles this — it skips `fsds_bridge` automatically.)

**Control loop phases** (see `mpc_controller_standalone.py`'s `_control_loop`):

1. **Hold at start line** — full brake until the `/fsds/signal/go` signal is
   received.
2. **Stale-path emergency brake** — full brake, and `MPCController.reset()`,
   if no fresh path has arrived within `TARGET_TIMEOUT` (0.5 s) or the path
   has fewer than 2 points. The reset discards the QP's warm start and
   actuator-lag memory so the controller doesn't resume from stale state
   once the path returns.
3. **Normal MPC solve** — `MPCController.compute()`.
4. **Cone-proximity brake override** — hard-overrides throttle/brake (not
   steering) if a fused cone is inside a dynamic corridor directly ahead.
   After `CONE_RESET_THRESHOLD` (0.3 s) of continuous braking the controller
   is reset once (edge-triggered, re-armed once the brake clears).
5. **Telemetry logging** (optional, `LOG_DIR`) — logs the *final*,
   post-override command, so the CSV reflects what was actually sent to the
   vehicle.
6. **Publish.**

### Recording, exporting and driving a track

This is the full pipeline from "no map of this track exists" to "the car
drives the precomputed line/speed on it" — recording, the two export tools,
the on-disk layout they share, and the one switch that puts a track on the
car. Each stage used to be documented (or not) in a different file; this
section is now the single place that chains them. If you only need the
concept, not the steps: **CLAUDE.md**'s planning/control-parity note and
`tracks/__init__.py`'s module docstring cover *why* the layout looks like
this; this section covers *how* to use it.

#### Where a track lives: `tracks/<name>/`

Everything for one track sits in one directory:

```
tracks/<name>/
    cone_map.json      the cone_recorder capture (the source of truth)
    speed_profile.csv  tuner.export_speed_profile output (centreline + oracle speed)
    raceline.csv       tuner.raceline_optimizer output (minimum-time line)
```

`comp_test_map_3` is the track every baseline number in this repo's docs
(`sim_to_real_investigation.md`, this guide, CLAUDE.md) is quoted against —
don't overwrite it; give a new recording its own name. List what exists with
`python3 -m tuner.export_speed_profile --list` (from `fsae_MPCTest/`), or
just `ls tracks/`.

#### 1. Record a lap

`fsae_planning`'s `cone_recorder` ROS 2 node (in the `fsae_sim_perception`
package) records one lap's worth of accumulated boundary cones from a live
FSDS run and writes them to a JSON file this repo can load — see
`sim/track_io.py` and the **Load Recorded Track** button in
[Get a path onto the map](#3-get-a-path-onto-the-map) above.

`sim.launch.py` launches `cone_recorder` automatically (`record_cones:=true`
is the default), so a normal `ros2 launch fsae_bringup sim.launch.py` is
already recording; no second terminal or separate launch command needed.
**But** if `use_precomputed_speed`/`use_precomputed_path` are on (the
default), the car is tracking the *existing* map's line, not driving off the
live planner — recording a genuinely new track needs those off, and
`ros2/launch_all.sh` is the easiest place to set that (see step 4 below,
which covers exactly this run). Driving through `sim.launch.py` directly
instead:

```bash
ros2 launch fsae_bringup sim.launch.py controller:=stanley \
    use_precomputed_speed:=false use_precomputed_path:=false \
    cone_out_path:=/path/to/fsae_MPCTest/tracks/<name>/cone_map.json
```

(`cone_recorder.launch.py` still exists standalone if you want to attach a
recorder to a stack that's already running — any planner/controller works,
since it only subscribes and doesn't affect the pipeline:

```bash
ros2 launch fsae_bringup cone_recorder.launch.py
ros2 launch fsae_bringup cone_recorder.launch.py out_path:=/path/to/cone_map.json
```
)

It starts recording on the first `/fsds/signal/go`, accumulates cones the
same way `centerline_planner.py`'s `ConeMap` does, and writes the file once
the car returns near its start pose after having driven at least
`min_lap_dist` (default 8 m) away from it — i.e. one closed lap. If the lap
never closes (e.g. a DNF) it writes anyway after `max_record_time` (default
300 s) and marks the file `"lap_closed": false`, so a partial/failed
recording is still usable but distinguishable from a clean lap.

Writing straight into `tracks/<name>/` (as above) means the export tools in
step 2 need no path argument — just the track name. If you instead record via
a bare `ros2 launch fsae_bringup sim.launch.py` (default output
`~/fsae_logs/cone_map_<timestamp>.json`), move or copy that file into
`tracks/<name>/cone_map.json` before exporting, or pass its full path to the
export tools directly (both accept an explicit path as well as a track name —
see `tracks/__init__.py`'s `resolve_map_arg`).

`ros2/launch_all.sh` writes directly to `tracks/$TRACK/cone_map.json` using
whatever `TRACK=` it's currently set to, so setting `TRACK=<name>` there
*before* recording is usually the least fiddly path (see step 4).

`fsae_MPCTest/fsds_simulator/launch_all.sh` (the separate mirror-repo copy)
still writes timestamped files to its own `fsds_simulator/cone_maps/`
instead — a deliberate, pre-existing difference from this repo's script, not
drift to fix. **Load Recorded Track** in the GUI reads both `tracks/*/` and
`fsds_simulator/cone_maps/`, so either script's output is still pickable up
there.

#### 2. Export the speed profile and raceline

Two independent offline tools turn a recorded `cone_map.json` into the CSVs
the live controller can read. Run from `fsae_MPCTest/`:

```bash
python3 -m tuner.export_speed_profile <name>   # -> tracks/<name>/speed_profile.csv
python3 -m tuner.raceline_optimizer   <name>    # -> tracks/<name>/raceline.csv
```

Omitting `<name>` targets the default track (`comp_test_map_3`). Both also
accept an explicit `cone_map.json` path in place of a name (for a capture
that isn't under `tracks/` yet), and `--list` prints what's available.

- `export_speed_profile.py` reconstructs the centreline the same way
  `sim/track_io.load_recorded_track()` does (scipy `CubicSpline` +
  `planning/boundary.build_path_walls()` marched around the lap) and writes
  its `x,y,psi,v_target` as a plain CSV — this is the "oracle path", tracking
  it directly at `e_y=0` matches the pre-`raceline_optimizer` behaviour.
- `raceline_optimizer.py` takes the same reconstruction and iteratively
  reshapes it within the track width for minimum lap time (widen-entry,
  clip-apex), respecting the physical model's `alat_ceiling` (see CLAUDE.md)
  rather than a flat friction limit. Same CSV format/columns, different
  geometry and generally higher `v_target`.

Both write a `# source_map=<path>` comment line into the CSV, so a stray
export can always be traced back to the map it came from. **Re-run either
tool whenever the recorded map changes** — nothing regenerates these
automatically.

The CSV format is deliberately trivial (4 columns, ~15-line reader, no scipy)
so the live ROS package (`control_utils.load_speed_profile_csv()` /
`load_path_profile_csv()`, in the separate `fsae_planning` repo) doesn't need
to port the reconstruction logic — see `export_speed_profile.py`'s module
docstring for the full reasoning.

#### 3. What the live controller reads

Two independent ROS launch args, both consumed by `mpc`/`mpc_standalone`
(not `stanley`):

| Launch arg | Default | Effect |
|------|---------|--------|
| `map_path` + `use_precomputed_speed` | `tracks/comp_test_map_3/speed_profile.csv` | Look up target speed from the CSV's oracle profile instead of live `curvature_speed()` per tick |
| `path_map_path` + `use_precomputed_path` | `tracks/comp_test_map_3/raceline.csv` | Track the CSV's geometry instead of subscribing to `centerline_planner.py`'s `/fsae/planning/selected_trajectory` — removes the live planner from the control loop entirely |

Both default `true`, so a bare `ros2 launch fsae_bringup sim.launch.py`
already drives the default track's precomputed line and speed with the
planner out of the loop. `map_path` and `path_map_path` can point at
different files (e.g. speed from the centreline, geometry from the raceline)
since the toggles are independent — but the common case is both pointing at
the same track, which is what step 4 gives you in one setting.

If a CSV path doesn't exist (e.g. before the first export), the node logs an
error at startup and falls back to live `curvature_speed()`/the live
planner — it does not crash, but it also silently isn't doing what you asked,
so check the log if a run looks unexpectedly like a live-planner run.

#### 4. Switching which track the car drives

**One variable.** In `ros2/launch_all.sh`:

```bash
TRACK=comp_test_map_3    # change this line to any name under fsae_MPCTest/tracks/
```

This expands to both `map_path` and `path_map_path` (and, for a *new*
recording, `cone_out_path`) automatically — no other line in that script
needs editing, and you never touch the hardcoded absolute defaults in
`sim.launch.py`/`control.launch.py` (those exist only as the fallback for a
bare `ros2 launch`, not as the thing to edit day-to-day). The script checks
the track's CSVs exist before launching and fails with a clear message
(naming the tracks that *do* exist) rather than silently falling back to
live planning.

To drive a track without going through `launch_all.sh`, pass the same three
args directly:

```bash
ros2 launch fsae_bringup sim.launch.py \
    map_path:=fsae_MPCTest/tracks/<name>/speed_profile.csv \
    path_map_path:=fsae_MPCTest/tracks/<name>/raceline.csv
```

Putting it all together, end to end:

```bash
# 1. Record (planner-in-loop, so the recording is a real live-driven lap)
#    -- set TRACK=<new-name>, USE_PRECOMPUTED_SPEED=false,
#    USE_PRECOMPUTED_PATH=false, CONTROLLER=mpc_standalone (or stanley) in
#    ros2/launch_all.sh, then:
./ros2/launch_all.sh

# 2. Export (from fsae_MPCTest/)
python3 -m tuner.export_speed_profile <new-name>
python3 -m tuner.raceline_optimizer   <new-name>

# 3. Drive it -- set TRACK=<new-name>, both toggles back to true
./ros2/launch_all.sh
```

### CSV telemetry logging

Every controller node (`stanley_controller.py`, `mpc_controller.py`,
`mpc_controller_standalone.py`) can optionally write two CSVs per run —
per-control-step telemetry and periodic path snapshots — via
`telemetry_logger.ControlLogger`. **Off by default**, same toggle pattern as
`cone_recorder` above: a ROS parameter, not a separate node or launch flag.

```bash
ros2 launch fsae_bringup sim.launch.py log_csv:=true                          # -> ~/fsae_logs
ros2 launch fsae_bringup sim.launch.py log_csv:=true log_dir:=/path/to/logs   # custom output dir
ros2 launch fsae_bringup sim.launch.py                                        # logging off (default)
```

Or directly on a `ros2 run`/node if you're not going through `sim.launch.py`:

```bash
ros2 run fsae_control mpc_controller_standalone --ros-args -p log_csv:=true -p log_dir:=/path/to/logs
```

Each run writes `<tag>_control_<timestamp>.csv` (one row per 20 Hz control
step: position, heading, speed, tracking error, commanded steering/accel,
solver health, and the latency-diagnostic columns) and
`<tag>_path_<timestamp>.csv` (path snapshots at ~1 Hz) into `log_dir`
(`~/fsae_logs` if unset). On shutdown, the control CSV is rewritten with a
`#`-commented header holding the run's composite score, computed by the exact
same maths as the offline tuner — see
[The Composite Score](architecture.md#the-composite-score) and
`fsae_control/telemetry_logger.py`'s module docstring for the full column
reference and units.

Logging and cone recording are independent toggles and can be combined freely
(`log_csv:=true record_cones:=true`) — a common pattern for a validation lap
you want to both replay through the CSV telemetry and reload into the GUI as
a recorded track.

### Launching nodes with FSDS on Windows (WSL + Docker)

This sets up the ROS 2 bridge and planning/control stack from scratch on a
Windows machine, using the precompiled Windows FSDS `.exe` alongside a
Dockerised ROS 2 Jazzy environment running inside WSL. Do the cloning step
in your WSL **home directory**, not inside an existing project folder.

**1. Clone the repo and start a ROS 2 Jazzy container**

```bash
# In WSL Ubuntu, from your home directory
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator.git --recurse-submodules

docker run -it \
  --name fsds_ros2_bridge \
  --net=host \
  --privileged \
  -v "$(pwd)":/root/Formula-Student-Driverless-Simulator \
  osrf/ros:jazzy-desktop \
  bash
```

`--net=host` is what makes the WSL-IP handshake in step 3 work — the
container shares WSL's network namespace rather than getting its own.

**2. Build the workspace inside the container**

Install the ROS 2 build tooling and message dependencies the bridge needs:

```bash
apt-get update && apt-get install -y \
  python3-colcon-common-extensions \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-tf2-geometry-msgs \
  libyaml-cpp-dev
```

FSDS's Windows `.exe` is built on AirSim, and the `/ros2` bridge package
in this repo depends on AirSim's client headers, so AirSim's own external
dependencies need fetching before the bridge will compile:

```bash
apt-get update && apt-get install -y eigen3-devel || apt-get install -y libeigen3-dev
apt-get update && apt-get install -y wget

cd /root/Formula-Student-Driverless-Simulator/AirSim
./setup.sh
```

Then build the ROS 2 workspace.

```bash
cd /root/Formula-Student-Driverless-Simulator/ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

**3. Point the bridge at the Windows-side simulator**

The bridge runs in the Linux/Docker side; the simulator `.exe` runs on
Windows. They talk over AirSim's RPC protocol (port `41451` by default),
so the bridge needs your WSL host's IP address to reach across that
boundary.

Get the IP (run this in a **WSL terminal**, not inside Docker):

```bash
ip route | grep default | awk '{print $3}'
```

Set that IP as the `host` launch argument default in
`fsds_ros2_bridge.launch.py`:

```python
launch.actions.DeclareLaunchArgument(
    'host',
    default_value='xxx.xx.xxx.x',  # your WSL_IP from above
    description='IP address of the Windows host running the simulator'
),
```

**Execution order matters:** always start the Windows `.exe` first (it
opens the RPC port), *then* launch the ROS 2 bridge — launching the bridge
before the simulator is up will fail to connect. (or use the launch file)

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py
```

Once connected, `ros2 topic list` (in a second container terminal) should
show live vehicle telemetry, image, and sensor topics streaming from the
simulator.

**4. Add the `fsae_planning` repo and this project's controller**

```bash
cd /root/Formula-Student-Driverless-Simulator/ros2/src
git clone https://github.com/UOA-FSAE/fsae_planning.git
```

Copy everything under `fsds_simulator/control/`, `fsds_simulator/perception/`,
`fsds_simulator/common/`, and `fsds_simulator/planning/` (in this repo) over
the matching paths inside the freshly-cloned `fsae_planning` checkout — the
hierarchy already matches, so this is a straight directory copy (see
[docs/planning_control_sync.md](planning_control_sync.md) for the exact file
mapping if you want to copy file-by-file instead), then resolve dependencies
and build:

```bash
cd /root/Formula-Student-Driverless-Simulator/ros2
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

**5. Run the closed loop**

With the Windows `.exe` and the bridge already running (steps 3), open a
third terminal into the same container and launch the planning stack:

```bash
docker exec -it fsds_ros2_bridge bash
source /opt/ros/jazzy/setup.bash
cd /root/Formula-Student-Driverless-Simulator/ros2
source install/local_setup.bash

ros2 launch fsae_bringup sim.launch.py

# Prevents core-dump files from being written on crashes:
ulimit -c 0
```

Alternatively, use the provided launch script to bring up the bridge and
planning nodes together: (note you need to change the paths in the launch file 
to where you installed your fsds simulator)

```bash
cd /home/Formula-Student-Driverless-Simulator/ros2/
chmod +x launch_all.sh
./launch_all.sh
```

**Installing solver dependencies (MPC controller) inside the container**

The base `osrf/ros:jazzy-desktop` image doesn't ship the QP solver stack
this controller needs (see [The solver](architecture.md#the-solver)). Install it manually
inside a running container:

```bash
apt update && apt install -y python3-pip
pip3 install cvxpy osqp --no-deps --break-system-packages
pip3 install qdldl scs clarabel highspy sparsediffpy jinja2 joblib markupsafe cffi pycparser --no-deps --break-system-packages
pip3 install cvxpy osqp --ignore-installed --break-system-packages
pip3 install "setuptools<80" --break-system-packages
pip3 install matplotlib kiwisolver --ignore-installed --break-system-packages
pip3 install "sparsediffpy<0.4.0" --break-system-packages
```

...or bake all of the above into a reusable custom image instead of
repeating it by hand every time the container is recreated:

```bash
cat << 'EOF' > fsds_ros2_custom.Dockerfile
FROM osrf/ros:jazzy-desktop
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    ros-jazzy-ackermann-msgs \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install cvxpy osqp --no-deps --break-system-packages
RUN pip3 install qdldl scs clarabel highspy sparsediffpy jinja2 joblib markupsafe cffi pycparser --no-deps --break-system-packages
RUN pip3 install cvxpy osqp --ignore-installed --break-system-packages
RUN pip3 install "setuptools<80" --break-system-packages
RUN pip3 install matplotlib kiwisolver --ignore-installed --break-system-packages
RUN pip3 install "sparsediffpy<0.4.0" --break-system-packages
EOF

docker build --no-cache -f fsds_ros2_custom.Dockerfile -t fsds_ros2_custom .
```

**Reopening after a reboot / rebuilding a single package:**

The container itself doesn't persist across a host reboot (only the
volume-mapped repo folder does), so it needs recreating from the custom
image:

```bash
cd /home/Formula-Student-Driverless-Simulator
docker rm -f fsds_ros2_bridge

docker run -it \
    --name fsds_ros2_bridge \
    --net=host \
    --privileged \
    -v "$(pwd)":/root/Formula-Student-Driverless-Simulator \
    fsds_ros2_custom \
    bash
```

To rebuild just the `fsae_planning` package after editing it (e.g. after
re-copying an updated `mpc_controller_standalone.py`/`mpc_core.py`/
`control_utils.py` from this repo's `fsds_simulator/` staging mirror):

```bash
cd /root/Formula-Student-Driverless-Simulator/ros2
rm -rf build/fsae_planning/ install/fsae_planning/
colcon build --packages-select fsae_planning --symlink-install
```

To edit the workspace files from Windows, open VS Code directly against
the WSL folder rather than editing inside the container:

```bash
cd /home/Formula-Student-Driverless-Simulator/ros2
code .
```

## Manual Drive Mode

`gui/manual_drive.py` is a small standalone app for driving the nonlinear plant
directly — useful for building intuition for the vehicle's handling limits,
eyeballing track/cone geometry, and generating a human reference trace to
compare against MPC runs on the same path. It shares the same 24-state
nonlinear plant and synthetic path library as the simulator, but is entirely
open-loop: no tracking error is computed, no MPC solve happens, and nothing
is scored.

**Run it:**

```bash
python -m gui.manual_drive
```

**Controls:** `W`/`S` throttle/brake, `A`/`D` steer left/right, `SPACE` full
brake (overrides throttle). Inputs are rate-limited toward the key-held
target so taps feel analog rather than an on/off step.

**Workflow:** **Load Test Path** to cycle through the synthetic path
library and place cones → **Start Driving** to spawn the plant at the
path's start pose → drive → **Reset** to stop and clear the trail.

---

## Dependencies

| Package | Version | Purpose |
|---|---|---|
| `numpy` | ≥1.24 | All numerical computation |
| `scipy` | ≥1.10 | ZOH discretisation (`expm`), spline fitting (`CubicSpline`) |
| `matplotlib` | ≥3.7 | Simulator/manual-drive GUI |
| `cvxpy` | ≥1.4 | MPC QP formulation |
| `osqp` | ≥0.6 | Primary QP solver (via CVXPY) |
| `clarabel` | ≥0.6 | Fallback QP solver (via CVXPY) |
| `cma` | ≥3.3 | CMA-ES optimiser (`fmin_lq_surr2`, BIPOP+surrogate) |
| `optuna` | ≥4.0 | Optional TPE pre-search that seeds CMA-ES's starting point (`tuner/offline_tuner.py`, only needed if `USE_OPTUNA_PRESEARCH = True` in `settings.py`) |
| `rclpy` | ROS 2 Humble+ | ROS 2 nodes only |
| `fs_msgs` | FSDS | `ControlCommand`, `GoSignal` message types |
| `fsae_interfaces` | `fsae_planning` | `ConeDetection` (cone-proximity brake input) |
| `nav_msgs` | ROS 2 | `Odometry` |
| `geometry_msgs` | ROS 2 | `Pose`, `PoseArray` |

```bash
pip install numpy scipy matplotlib cvxpy cma
pip install cvxpy[osqp] cvxpy[clarabel]
pip install optuna  # optional: only needed for USE_OPTUNA_PRESEARCH in settings.py
```

---

## Extending and Debugging

If you're extending the simulator or tuning the vehicle, follow these
guidelines to keep the MPC/plant architecture consistent.

### Modifying vehicle parameters

See [Configuring the Vehicle](architecture.md#configuring-the-vehicle-modelvehicle_physicspy)
above. The short version: `VehicleParams` in `model/vehicle_physics.py` is the
single source of truth; if you import new Pacejka tyre data, you must also
recompute `Cf`/`Cr` to match its initial slope, or the MPC's internal model
will silently diverge from the plant it's controlling.

### Adding a new synthetic path

1. In `tuner/offline_tuner.py`, open `build_synthetic_paths()`.
2. Define your segments — `_make_arc(cx, cy, radius, start_deg, end_deg, n)`
   for constant-radius corners, `np.linspace()` for straights.
3. Concatenate the segment arrays and pass them through `_resample_path(wx, wy)`.
4. Add the resulting tuple to the `paths` dictionary under a new key.
5. *(Optional)* Add that key to `VALIDATION_SUITE` in `settings.py` if you
   want the tuner to optimise against it — see
   [Configuring the Project](architecture.md#configuring-the-project-settingspy).

### Debugging solver failures

If the live simulator reports `consecutive_solver_failures` or the console
frequently shows `OPTIMAL_INACCURATE`:

- **Weight scaling** — OSQP is sensitive to poorly-conditioned matrices. If
  any entry of `Q`, `R`, or `R_rate` exceeds `1e4` or drops below `1e-4`,
  convergence can suffer. Check `adaptive_R_scaling()`'s output at your
  test speed isn't blowing up the steering cost unexpectedly.
- **Kinematic vs. dynamic gap** — if the car consistently fails at tight
  hairpins, `sim/speed_profile.py` may be commanding a speed that demands more
  lateral force than the Pacejka friction circle can supply at that
  curvature. Lower `mu` in `compute_speed_profile()` to force more
  conservative corner-entry speeds.
- **Model-plant mismatch at extremes** — remember the MPC's internal model
  is linear and only blends kinematic/dynamic behaviour between 1-2.5 m/s;
  well outside that (very low speed under load, or very high lateral
  acceleration near the tyre limit) is where the biggest prediction error
  will show up, and where `adaptive_R_scaling`/`adaptive_R_rate` matter most.
