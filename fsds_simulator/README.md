# `fsds_simulator/` — standalone ROS 2 workspace mirror

This folder is a **staging mirror** of the live
[`fsae_planning`](https://github.com/UOA-FSAE/fsae_planning) ROS 2 workspace — every
file lives at the exact relative path colcon expects, so this folder alone (plus
FSDS itself and the two message repos below) is enough to build and run the full
autonomous stack: `centerline_planner`, and either the `stanley` or `mpc`/`mpc_standalone`
controller. Every node/package file is byte-for-byte identical to its live counterpart;
`launch_all.sh` is the one exception — see "What's here but adapted" below. See the parent
repo's [docs/planning_control_sync.md](../docs/planning_control_sync.md)
for the exact file-by-file mapping, what is deliberately *not* mirrored, and the resync
procedure.

Nothing under `fsds_simulator/` is imported by this repo's own simulator/tuner
(`gui/`, `sim/`, `model/`, `controller/`, `tuner/`) — this folder exists purely to
hold a ready-to-build copy of the ROS 2 side.

## Layout

```
fsds_simulator/
├── launch_all.sh                # one-command launcher (adapted paths, see below)
├── requirements.txt              # Python deps for this stack (mpc_core.py's solver, etc.)
├── common/
│   ├── fsae_interfaces/        # vendored msgs (Track, ConeDetection, …)
│   └── fsae_bringup/           # fsae_params.yaml + launch composition (sim.launch.py)
├── perception/
│   └── fsae_sim_perception/    # sim_perception: FSDS oracle+odom → /fsae/* inputs
├── planning/
│   └── fsae_planning/          # centerline_planner, skidpad_planner + utils
└── control/
    └── fsae_control/           # stanley_controller / mpc_controller / mpc_controller_standalone
                                 # + fsds_bridge (cmd_vel → FSDS) + mpc_core (shared MPC QP)
                                 # + scoring.py (live/offline score parity, see planning_control_sync.md)
```

## Building it into a workspace

1. Create a ROS 2 (Jazzy) workspace and clone the two message dependencies this stack
   needs alongside it:
   ```bash
   mkdir -p ~/ros2_fsd/src && cd ~/ros2_fsd/src
   git clone https://github.com/FS-Driverless/fs_msgs.git -b ros2   # fs_msgs (FSDS's own messages)
   ```
   `ackermann_msgs` is a released ROS package, not a repo to clone:
   ```bash
   sudo apt install ros-jazzy-ackermann-msgs
   ```
2. Copy (or symlink) every package folder under this `fsds_simulator/` directory
   into `~/ros2_fsd/src/`, preserving their relative paths — i.e. `common/`,
   `perception/`, `planning/`, `control/` end up directly under `src/`, each
   containing its packages (`fsae_interfaces`, `fsae_bringup`, `fsae_sim_perception`,
   `fsae_planning`, `fsae_control`).
3. You also need the FSDS ↔ ROS 2 bridge itself
   (`fsds_ros2_bridge`, from the
   [FSDS simulator repo](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator)'s
   `ros2/src/fsds_ros2_bridge`) in the same workspace — this mirror does not include it,
   since it's part of FSDS itself, not the planning/control stack.
4. Install this stack's own Python dependencies (`requirements.txt` in this
   directory, copied from the live workspace):
   ```bash
   pip install -r requirements.txt
   ```
5. Build:
   ```bash
   cd ~/ros2_fsd && source /opt/ros/jazzy/setup.bash && colcon build
   ```

## Running

```bash
# Terminal 1 — FSDS itself
cd ~/fsds-v2.2.0-linux && ./FSDS.sh

# Terminal 2 — FSDS <-> ROS 2 bridge
cd ~/ros2_fsd && source install/setup.bash
ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py UDP_control:=false

# Terminal 3 — perception + planning + control
cd ~/ros2_fsd && source install/setup.bash
ros2 launch fsae_bringup sim.launch.py controller:=stanley           # Stanley controller
ros2 launch fsae_bringup sim.launch.py controller:=mpc               # MPC via fsds_bridge (steering only)
ros2 launch fsae_bringup sim.launch.py controller:=mpc_standalone     # MPC's own throttle/brake (default)
```

`controller:=mpc_standalone` is the only mode whose longitudinal behaviour matches
what this repo's offline tuner actually tunes — see the parent README's explanation
of why `mpc_controller.py` discards the MPC's own throttle/brake. `stanley` and `mpc`
both route through `fsds_bridge`'s simple speed-error P-loop instead.

### One-command launch (`launch_all.sh`)

If your workspace lives at a fixed path (rather than the generic `~/ros2_fsd` used
above), `launch_all.sh` in this directory automates all three terminals above —
starts FSDS, waits for its RPC server, starts the bridge, waits for odom, then
launches the autonomous stack, tearing everything down cleanly on exit or Ctrl+C.
It is **adapted to this repo's own machine** (hardcoded Windows username, screen
resolution, and workspace path) — read it and edit those three things for your own
setup before relying on it; it is not a drop-in script.

```bash
./launch_all.sh
```

## What's here but adapted (not byte-identical)

`launch_all.sh` hardcodes machine-specific paths (Windows FSDS install location,
this host's ROS 2 workspace path, log output directory) — copied from the live
script and then edited to this repo owner's own machine, the same way you'd need
to edit it again for yours. Every other file in this mirror is a byte-for-byte copy.

## What's deliberately not here

- `fsds_ros2_bridge` itself — part of FSDS, not this stack.
- Anything under `ros2/src/fsae_planning`'s `.git/`, `build/`, `install/`, `log/`,
  `__pycache__/` — build artifacts, not source.
- `steering_sysid.py` / `steering_step.py` and their harness scripts — standalone
  open-loop diagnostics, not part of the MPC controller's runtime dependencies.
- `launch_terminals.sh` — a simpler multi-terminal opener superseded by this
  mirror's own `launch_all.sh`, which additionally waits for FSDS's RPC
  server and odom before launching and tears everything down on exit.
- `CHANGES.md` / `.gitignore` — repo-management files specific to the live
  `fsae_planning` checkout, not needed to build or run this mirror.
