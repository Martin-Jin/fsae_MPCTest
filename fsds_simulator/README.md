# fsds_simulator: snapshot mirror of the ROS 2 workspace

This directory is a snapshot mirror of the `ros2/src/fsae_planning` ROS 2 workspace (the live simulation tree). It stages changes made there so they can be pull-requested into the separate `fsae_planning` repo later. Nothing is pushed to `fsae_planning` directly, so this mirror is where those changes are stored.

- **Same paths.** Files sit at the relative paths colcon expects. This directory, plus FSDS and the two message dependencies below, builds and runs the whole stack: `centerline_planner`, and the `stanley` or `mpc` controller.
- **Identical implementation files.** Every Python file under `control/`, `perception/`, `planning/` and `common/` is currently byte-identical to its live counterpart. `launch_all.sh` is identical too.
- **A snapshot, not a live mirror.** A change to a file that exists here is applied here as well. Files that were never here are not added. Unrelated drift is not fixed on the way past.
- **Not imported by the offline tools.** Nothing under `fsds_simulator/` is imported by `gui/`, `sim/`, `model/`, `controller/` or `tuner/` in this repo.

The file mapping, the deliberate non-mirrors and the resync procedure are in [offline_live_parity.md](../docs/reference/offline_live_parity.md).

## Layout

```
fsds_simulator/
├── launch_all.sh                 one-command launcher (same as ros2/launch_all.sh)
├── requirements.txt              Python deps for this stack
├── fsds_ros2_custom.Dockerfile   ROS 2 Jazzy image with the solver stack
├── common/
│   ├── fsae_interfaces/          vendored messages (Track, ConeDetection, ...)
│   └── fsae_bringup/             fsae_params.yaml + launch files (sim.launch.py, control.launch.py, ...)
├── perception/
│   └── fsae_sim_perception/      sim_perception, cone_recorder
├── planning/
│   └── fsae_planning/            centerline_planner, skidpad_planner, boundary/cone/path utilities
├── control/
│   └── fsae_control/fsae_control/
│       ├── stanley_controller.py     Stanley node (entry point: controller)
│       ├── fsds_bridge.py            cmd_vel to FSDS control command (entry point: fsds_bridge)
│       ├── control_utils.py          curvature speed, CSV loaders, speed gates
│       ├── brake_sysid.py            open-loop braking system-ID (entry point: brake_sysid)
│       ├── mpc/                      mpc_controller.py (node, entry point: mpc_controller),
│       │                             control_step.py, debug_publish.py, node_constants.py,
│       │                             mpc_params.py, nmpc_params.py
│       ├── lmpc/                     LTV-QP: constants, predict, adaptive_gains, controller
│       ├── nmpc/                     NMPC: layout, reference, dynamics, outputs, weight_schedule,
│       │                             qp_model, sqp_step, solver
│       ├── telemetry/                columns, config_lines, horizon_tracker, lap_progress,
│       │                             control_logger, scoring
│       └── live_viz/                 panels, node, app (entry point: live_viz)
├── tracks/                       recorded tracks (a copy that has drifted from the live tracks/)
├── cone_maps/                    an older recorded cone map
└── recorded_runs/                exported telemetry CSVs, read by tuner/tools/plot_playback.py
```

- `mpc/` holds the node and the parameter dataclasses (`MPCParams`, `NMPCParams`). `lmpc/` and `nmpc/` hold the solvers. `nmpc/` mirrors the offline `controller/nmpc/` file for file.
- `telemetry/scoring.py` is a copy of the offline `sim/scoring.py` with the scoring constants inlined. Keep the two numerically identical.
- The live and offline parity rule and the numeric-parity constants are in [offline_live_parity.md](../docs/reference/offline_live_parity.md).

## Building it into a workspace

1. Create a ROS 2 Jazzy workspace and clone the message package next to it:

   ```bash
   mkdir -p ~/ros2_fsd/src && cd ~/ros2_fsd/src
   git clone https://github.com/FS-Driverless/fs_msgs.git -b ros2
   sudo apt install ros-jazzy-ackermann-msgs
   ```

   `ackermann_msgs` is a released ROS package, not a repo to clone.
2. Copy or symlink `common/`, `perception/`, `planning/` and `control/` from this directory into `~/ros2_fsd/src/`, keeping their relative paths.
3. Add the bridge, `fsds_ros2_bridge`, from the FSDS repo's `ros2/src/fsds_ros2_bridge`, into the same workspace. This mirror does not carry it, because it belongs to FSDS.
4. Install the Python dependencies:

   ```bash
   pip install -r requirements.txt
   ```

5. Build:

   ```bash
   cd ~/ros2_fsd && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
   ```

   Use `--symlink-install` so later edits to `src/` take effect without a rebuild.

The external steps above (the `fs_msgs` branch, the apt package) were not re-run when this file was rewritten.

## Running

```bash
# Terminal 1: FSDS itself
cd ~/fsds-v2.2.0-linux && ./FSDS.sh

# Terminal 2: FSDS to ROS 2 bridge
cd ~/ros2_fsd && source install/setup.bash
ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py

# Terminal 3: perception, planning and control
cd ~/ros2_fsd && source install/setup.bash
ros2 launch fsae_bringup sim.launch.py controller:=stanley
ros2 launch fsae_bringup sim.launch.py controller:=mpc standalone_output:=false
ros2 launch fsae_bringup sim.launch.py controller:=mpc
```

- The bridge launch file reads the simulator address from `$FSDS_HOST_IP` (default `localhost`). Under WSL2 with a Windows-side `.exe`, set it first. See the [integration guide](../docs/fsds/integration_guide.md#3-point-the-bridge-at-the-windows-side-simulator).
- `controller:=mpc` defaults to `standalone_output:=true`. That is the mode whose throttle and brake come from the MPC itself, which is what the offline tuner tunes. With `standalone_output:=false`, `mpc` and `stanley` both route speed through `fsds_bridge`'s speed-error P-loop instead. See [choosing the controller and planner](../docs/fsds/integration_guide.md#choosing-the-controller-and-planner).
- By default the launch uses the precomputed speed and path of the newest or pinned track. See the [integration guide](../docs/fsds/integration_guide.md#recording-exporting-and-driving-a-track).

### One-command launch

`launch_all.sh` automates the three terminals: it starts the Windows FSDS `.exe`, waits for the RPC port, starts the bridge, then launches the stack, and tears everything down on exit or Ctrl+C. It hardcodes one machine's Windows install path (`WINDOWS_SIM_PATH` and a `cmd.exe` line) and expects the `src/fsae_planning/tracks/` layout beside it. Read and edit it before use. It is not a drop-in script.

```bash
./launch_all.sh
```

## Deliberately not mirrored

- `fsds_ros2_bridge`: part of FSDS.
- The live checkout's `.git/`, `build/`, `install/`, `log/` and `__pycache__/`.
- `launch_terminals.sh`: a simpler multi-terminal opener that `launch_all.sh` supersedes.
- `CHANGES.md` and `.gitignore`: repo management files of the live `fsae_planning` checkout.
- The live checkout's `README.md`. It differs from this file.
- Live-only tracks (`acceleration_20260916`, `comp_test_map_2_20260916`) and `speed_profile_corner_test.csv`.
