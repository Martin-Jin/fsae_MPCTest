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
  (newest-first) through `*.json` files in `~/fsae_logs`, the cone maps
  written by `fsae_planning`'s `cone_recorder` ROS 2 node after a live FSDS
  lap (see [Recording a track from FSDS](#recording-a-track-from-fsds)
  below). Unlike the synthetic paths, the blue/yellow cones rendered are the
  *actual recorded cones*, not `place_cones()` output — a real perception
  recording, resimulated exactly as `SimPerception`/`SimPlanner` would drive
  it live. The centreline drawn on load is only a reconstruction for the
  oracle-mode reference path and initial camera framing (see
  `sim/track_io.py`); with `USE_PLANNER = True` (the default), the actual
  driving line during the rollout still comes from `SimPlanner` rebuilding
  it cone-by-cone, exactly as for a synthetic path.

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

Click **Show Metrics** to print a full 12-metric breakdown to the console
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
  perception/planning pipeline (`True`, default) or driving on the perfect
  reference line (`False`, faster).

### 3. Launch

```bash
cd /path/to/project
python -m tuner.offline_tuner
```

This uses all available CPU cores minus one (one is left free for the OS).
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

`fsds_simulator/` in this repo is a staging area that mirrors `fsae_planning`'s
own ROS 2 package hierarchy — e.g. `fsds_simulator/control/fsae_control/
fsae_control/mpc_core.py` sits at the exact relative path it needs to land at
inside a `fsae_planning` checkout. To run the controller against the FSDS
simulator in ros2, obtain the `fsae_planning` repo, then copy everything
under `fsds_simulator/control/`, `fsds_simulator/perception/`, and
`fsds_simulator/common/` in this repo over the matching paths inside
`fsae_planning` (same relative structure, so it's a straight directory copy,
not a manual file-by-file paste). See
[docs/planning_control_sync.md](planning_control_sync.md) for the full file
mapping and what's a direct mirror vs. deliberately not ported (e.g. this
repo's frozen Stanley reference implementation).
(If you already have the simulator set up with the `fsae_planning` repo. Scroll down for installing from scratch on windows.)

**Topic map for the control node:**

```
/fsds/testing_only/track       → sim_perception   → /fsae/slam/left_track
                                                   → /fsae/slam/right_track
                                                   → /fsae/perception/cone_detection
/fsds/testing_only/odom        → sim_perception
                                  centerline_planner (via car_position)
                                  mpc_controller_standalone

/fsae/slam/left_track,
/fsae/slam/right_track,
/fsae/slam/car_position        → centerline_planner → /fsae/planning/selected_trajectory

/fsae/planning/selected_trajectory  → mpc_controller_standalone   → /fsds/control_command
/fsae/slam/car_position             → mpc_controller_standalone
/fsds/testing_only/odom             → mpc_controller_standalone
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

### Recording a track from FSDS

`fsae_planning`'s `cone_recorder` ROS 2 node (in the `fsae_sim_perception`
package) records one lap's worth of accumulated boundary cones from a live
FSDS run and writes them to a JSON file this repo can load — see
`sim/track_io.py` and the **Load Recorded Track** button in
[Get a path onto the map](#3-get-a-path-onto-the-map) above.

Launch it alongside a normal run (any planner/controller — it only
subscribes, it doesn't affect the pipeline):

```bash
ros2 launch fsae_bringup cone_recorder.launch.py
# or, with an explicit output path:
ros2 launch fsae_bringup cone_recorder.launch.py out_path:=/path/to/cone_map.json
```

It starts recording on the first `/fsds/signal/go`, accumulates cones the
same way `centerline_planner.py`'s `ConeMap` does, and writes the file once
the car returns near its start pose after having driven at least
`min_lap_dist` (default 8 m) away from it — i.e. one closed lap. If the lap
never closes (e.g. a DNF) it writes anyway after `max_record_time` (default
300 s) and marks the file `"lap_closed": false`, so a partial/failed
recording is still usable but distinguishable from a clean lap. Default
output location is `~/fsae_logs/cone_map_<timestamp>.json` — the same
directory **Load Recorded Track** cycles through, so no extra copying is
needed between the two repos as long as both read/write the same
filesystem (e.g. the same WSL/Docker volume mount).

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
and `fsds_simulator/common/` (in this repo) over the matching paths inside
the freshly-cloned `fsae_planning` checkout — the hierarchy already matches,
so this is a straight directory copy (see
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
| `rclpy` | ROS 2 Humble+ | ROS 2 nodes only |
| `fs_msgs` | FSDS | `ControlCommand`, `GoSignal` message types |
| `fsae_interfaces` | `fsae_planning` | `ConeDetection` (cone-proximity brake input) |
| `nav_msgs` | ROS 2 | `Odometry` |
| `geometry_msgs` | ROS 2 | `Pose`, `PoseArray` |

```bash
pip install numpy scipy matplotlib cvxpy cma
pip install cvxpy[osqp] cvxpy[clarabel]
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
