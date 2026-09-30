# FSDS ROS 2 workspace module reference

`fsds_simulator/` is a snapshot of the live ROS 2 workspace at `ros2/src/fsae_planning` in the outer FSDS repo. It holds the code that drives the simulated car: perception stand-in, planner, the LTV-QP and NMPC controllers, the FSDS command bridge, telemetry and the live debug viewer. Nothing in the offline simulator imports it. The offline counterpart is documented in [offline_sim.md](offline_sim.md). How the two sides are kept equal is in [Mirror workflow](#mirror-workflow).

Each file below has one entry: what it does, what to change it for (with the check to run afterwards), what to leave alone, and its main names. Paths are repo-relative as `git ls-files` prints them. Scripts that live in the outer FSDS repo, not in this repo, are given with outer-repo-relative paths.

Checks named in the entries, all run from the repo root:

| Check | Command | Covers |
|---|---|---|
| Offline scoring reproduction (~2 min) | `python -m tuner.validation.recorded_map_rollout` | weights, plant, scoring, speed target, planner changes |
| NMPC solver self-consistency | `python -m tuner.validation.nmpc_offline_check` | `nmpc/` changes |
| Plant open-loop replay | `python -m tuner.validation.plant_openloop_validation` | plant model only (offline side) |
| Doc conventions | `python -m tuner.tools.doc_lint` | docs and this reference |

No offline check exercises ROS 2 nodes, launch files or the FSDS bridge. Such changes need a full FSDS session. The offline result alone never confirms a planning or control change, see [offline_live_parity.md](../reference/offline_live_parity.md).

## Where to change what

| Task | Edit | Then |
|---|---|---|
| Retune an LTV-QP weight (`q_e_y`, `r_delta`, `r_rate_delta`, ...) | `fsds_simulator/control/fsae_control/fsae_control/mpc/mpc_params.py` default, the same key in `fsds_simulator/common/fsae_bringup/config/fsae_params.yaml`, then the matching constant in `settings/lmpc.py` | Check `launch_all.sh` for an uncommented override of the field. Run `python -m tuner.validation.recorded_map_rollout`. Live-test. |
| Retune an NMPC weight override (`nmpc_q_e_y`, `nmpc_rjerk_delta`, ...) | `mpc_params.py` (these override fields live in `MPCParams`), `fsae_params.yaml`, `settings/nmpc.py` | Same as above |
| Change an NMPC structural or solver field (`nmpc_horizon`, `nmpc_sqp_iters`, RK4 substeps) | `fsds_simulator/control/fsae_control/fsae_control/mpc/nmpc_params.py`, `fsae_params.yaml`, `settings/nmpc.py` | `python -m tuner.validation.nmpc_offline_check`, then the rollout check |
| Add a new tunable field | Field with metadata in `mpc_params.py` or `nmpc_params.py`, key in `fsae_params.yaml`, constant in `settings/`, read it in the controller | Launch args are generated from the field metadata, no launch edit needed. Add a `launch_all.sh` shortlist line only if it will be retuned often. |
| Push a live-validated param change to the mirror and `fsae_autonomous` | Edit the 3 param files in `ros2/src/fsae_planning` | `python -m tuner.tools.sync_mpc_params`, then `--apply` |
| Change LTV-QP maths (cost, constraints, delay compensation, adaptive gains) | `fsds_simulator/control/fsae_control/fsae_control/lmpc/` | Mirror by hand into offline `controller/lmpc/` and `controller/model_utils.py`, then the rollout check |
| Change NMPC maths (dynamics, cost rows, SQP step, rate-weight zones) | `fsds_simulator/control/fsae_control/fsae_control/nmpc/` (file names match offline `controller/nmpc/` one to one) | Apply the same edit to the offline file. Run `nmpc_offline_check` and the rollout check. |
| Change the speed target, gates or rate limits | `fsds_simulator/control/fsae_control/fsae_control/mpc/control_step.py`, `node_constants.py`, `fsds_simulator/control/fsae_control/fsae_control/control_utils.py` | Offline twins are `sim/rollout/speed_target.py` and `sim/speed_profile.py`. Rollout check. |
| Change the score formula | `sim/scoring.py` first, then copy into `fsds_simulator/control/fsae_control/fsae_control/telemetry/scoring.py` | Keep the inlined constants equal to `settings/scoring.py`. Rollout check. |
| Add or rename a telemetry CSV column | `telemetry/columns.py` (adaptive block) or `telemetry/control_logger.py` | Check the analysers in `tuner/` that read the CSV |
| Change the planner or cone handling | `fsds_simulator/planning/fsae_planning/fsae_planning/` | Read the known centreline-curvature-spike defect in [simulator_fidelity.md](../reference/simulator_fidelity.md) first. Mirror into offline `planning/`. Rollout check and an FSDS session. |
| Change what the car perceives (FOV, rates) | `sim_perception` block of `fsae_params.yaml`, or `fsds_simulator/perception/fsae_sim_perception/fsae_sim_perception/sim_perception.py` | `pose_rate` must stay at or above the controller rate (20 Hz) |
| Change which track, controller or feature flags a launch uses | Variables at the top of `ros2/launch_all.sh` (outer repo), then copy it to `fsds_simulator/launch_all.sh` | Never edit `sim.launch.py` or `control.launch.py` defaults for a one-off change |
| Add or change a launch argument | `fsds_simulator/common/fsae_bringup/launch/control.launch.py` and `sim.launch.py` (both, they forward to each other) | Launch in an FSDS session and read the printed warnings |
| Add a node or console script | Node file, then `setup.py` `entry_points` of its package | Rebuild with `colcon build --symlink-install` |
| Change a ROS message | `.msg` file and `CMakeLists.txt` `msg_files` in `fsds_simulator/common/fsae_interfaces/` | Rebuild the whole workspace, every consumer is affected |
| Measure braking or steering response | `ros2/run_brake_sysid.sh` with `fsds_simulator/control/fsae_control/fsae_control/brake_sysid.py` | Analyse with the `tuner/investigations/` scripts |
| Investigate the periodic pose stall | `ros2/clock_drift_check.py`, `ros2/launch_all.sh` diagnostic block, `check_fsds.sh` | Read `docs/logs/periodic_pose_teleport_investigation.md` first |
| Add a track | Data lives in `ros2/src/fsae_planning/tracks/<name>/`, produced by `tuner/tools/` exporters | See [reference_path_and_speed.md](../reference/reference_path_and_speed.md) |

## Workspace layout and runtime data flow

The workspace has five ROS 2 packages under four folders. `colcon` discovers them by `package.xml`, so the folder nesting has no effect on the build.

| Package | Folder | Role |
|---|---|---|
| `fsae_interfaces` | `fsds_simulator/common/fsae_interfaces/` | Message definitions (CMake package) |
| `fsae_bringup` | `fsds_simulator/common/fsae_bringup/` | Launch composition and the central parameter YAML |
| `fsae_sim_perception` | `fsds_simulator/perception/fsae_sim_perception/` | FSDS oracle map and odometry turned into the car's `/fsae/*` inputs |
| `fsae_planning` | `fsds_simulator/planning/fsae_planning/` | Centreline planner and skidpad planner |
| `fsae_control` | `fsds_simulator/control/fsae_control/` | Stanley, LTV-QP and NMPC controllers, FSDS bridge, telemetry, live viewer |

Data flow in the default configuration (`controller:=mpc`, `standalone_output=true`, precomputed path and speed):

1. FSDS publishes ground-truth odometry and a latched cone map through `fsds_ros2_bridge` (outer repo, not in this workspace).
2. `sim_perception` republishes pose, odometry snapshot and cropped cones on `/fsae/slam/*` and `/fsae/perception/cone_detection`.
3. The planner publishes `/fsae/planning/selected_trajectory`. With `use_precomputed_path=true` it is not launched and the controller reads the path CSV itself.
4. `mpc_controller` solves at 20 Hz and publishes the FSDS `ControlCommand` message on `/fsds/control_command`. With `standalone_output=false` it publishes steering only on `/fsae/control/cmd_vel` and `fsds_bridge` computes throttle and brake.
5. `live_viz` and the CSV logger read the same topics for debugging and scoring.

Packages are pure Python (`ament_python`) except `fsae_interfaces`. Each has a `resource/<package>` marker file (ament index) and a `setup.cfg` that points installed scripts at `lib/<package>`. Empty `__init__.py` files (`fsds_simulator/control/fsae_control/fsae_control/__init__.py`, `fsds_simulator/control/fsae_control/fsae_control/mpc/__init__.py`, `fsds_simulator/perception/fsae_sim_perception/fsae_sim_perception/__init__.py`, `fsds_simulator/planning/fsae_planning/fsae_planning/__init__.py`, `fsds_simulator/planning/fsae_planning/fsae_planning/special_utils/__init__.py`, `fsds_simulator/common/fsae_bringup/fsae_bringup/__init__.py`) only mark packages. The package `__init__.py` files of `lmpc/`, `nmpc/`, `telemetry/` and `live_viz/` carry the module docstrings and re-exports and are described in their package intros.

Run-time data files in the mirror are not documented per file: `fsds_simulator/tracks/comp_test_map_3/` (cone map, speed profile, raceline, centreline CSVs), `fsds_simulator/cone_maps/` and `fsds_simulator/recorded_runs/` (per-controller reference run CSVs). The canonical track data is in `ros2/src/fsae_planning/tracks/`, the mirror copy is a snapshot.

## fsae_control

Controllers, FSDS command bridge, telemetry and the debug viewer. Console scripts registered in `fsds_simulator/control/fsae_control/setup.py`: `controller` (Stanley), `mpc_controller`, `fsds_bridge`, `live_viz`, `brake_sysid`. Both MPC controllers run inside the one `mpc_controller` node, chosen by `use_nmpc`. The sub-packages:

| Sub-package | Content |
|---|---|
| `mpc/` | The ROS node, its per-tick step, debug publishing, node constants, and the two parameter dataclasses |
| `lmpc/` | LTV-QP controller (linear time-varying model predictive control, a convex QP each tick) |
| `nmpc/` | Frenet-frame nonlinear MPC (Gauss-Newton SQP, OSQP subproblem), file names match offline `controller/nmpc/` |
| `telemetry/` | CSV logging, lap progress, horizon accuracy, score header |
| `live_viz/` | Matplotlib debug window |

### `fsds_simulator/control/fsae_control/setup.py`

Does: builds the package, installs `resource/fsae_control` and `package.xml`, registers the five console scripts.
Change it to:
- Add a script, for example `fsae_control.some_tool:main`, then rebuild and check `ros2 run fsae_control <name>`.
Don't:
- Rename a module a console script points at without editing its entry here, because `launch_all.sh` and the launch files call scripts by entry-point name and the failure appears only at launch.
Key API: `entry_points['console_scripts']`

### `fsds_simulator/control/fsae_control/setup.cfg`

Does: sends installed scripts to `$base/lib/fsae_control`, where `ros2 run` looks.
Change it to:
- Nothing in normal work.
Don't:
- Delete it, because `ros2 run fsae_control ...` then cannot find the scripts.
Key API: `[develop] script_dir`, `[install] install_scripts`

### `fsds_simulator/control/fsae_control/package.xml`

Does: declares ROS dependencies (`ackermann_msgs`, `fs_msgs`, `fsae_interfaces`, `nav_msgs`, `python3-tk`) for `fsae_control`.
Change it to:
- Add an `exec_depend` when a node imports a new ROS package.
Don't:
- Expect it to install `cvxpy`, `osqp`, `clarabel` or `matplotlib`, because those are pip-only and are listed in a comment, not resolved by `rosdep`. The comment still names two modules that no longer exist, `mpc_core` and a single-file `live_viz` (see the report).
Key API: `exec_depend`, `test_depend`

### `fsds_simulator/control/fsae_control/test/nmpc_offline_check.py`

Does: runs six NMPC checks with no ROS or FSDS: scalar vs vectorised model parity, Jacobians, SQP convergence, turn-in sign, and a closed loop that is skipped without a sibling `fsae_MPCTest`.
Change it to:
- Add a check after an NMPC solver change, then run `python -m tuner.validation.nmpc_offline_check` (the offline twin) and this file.
Don't:
- Read a pass as live validation, because it uses the offline plant, not FSDS.
- Rely on its closed-loop step from inside the mirror, because it locates `fsae_MPCTest` by walking up from `ros2/src/fsae_planning`, a layout the mirror does not have.
Key API: `test_model_parity`, `test_jacobians`, `test_convergence`, `test_turn_in`, `test_closed_loop`, `main`

### `fsds_simulator/control/fsae_control/fsae_control/stanley_controller.py`

Does: Stanley path-tracking node (`controller` script) that publishes a curvature-limited speed and steering angle on `/fsae/control/cmd_vel`.
Change it to:
- Change how the speed target is chosen, for example a different `map_path` use. Check with a Stanley run in FSDS, the offline rollout check does not run this node.
Don't:
- Add MPC-only parameters here, because `control.launch.py` gives the non-MPC node a reduced parameter set.
Key API: `StanleyControllerNode`, `main`

### `fsds_simulator/control/fsae_control/fsae_control/control_utils.py`

Does: shared control helpers: the Stanley law, curvature-limited and dynamic speed caps, tracking-error speed gate, and CSV loaders for precomputed speed, path and heading profiles.
Change it to:
- Adjust `curvature_speed` or `dynamic_speed_cap` behaviour. Apply the same edit to offline `sim/speed_profile.py`, then run the rollout check.
- Add a CSV column to the loaders, then check `tuner/tools/export_speed_profile.py` and `tuner/tools/raceline_optimizer.py` still write it.
Don't:
- Change loader return shapes without editing every caller, because `mpc_controller.py` and `control_step.py` index the arrays directly.
Key API: `StanleyController`, `curvature_speed`, `dynamic_speed_cap`, `tracking_error_speed_gate`, `load_speed_profile_csv`, `load_path_profile_csv`, `load_path_heading_profile_csv`, `precomputed_speed_at`

### `fsds_simulator/control/fsae_control/fsae_control/fsds_bridge.py`

Does: converts `/fsae/control/cmd_vel` into the FSDS `ControlCommand` message (speed error to throttle and brake, steering angle to normalised steering), holds the brake until GO, and emergency-brakes on close cones.
Change it to:
- Retune the speed P-loop for Stanley or `standalone_output=false` runs. FSDS session only.
Don't:
- Launch it with `mpc_controller` in `standalone_output=true`, because both publish `/fsds/control_command` and race. `control.launch.py` already skips it in that mode.
Key API: `FsdsBridge`, `main`

### `fsds_simulator/control/fsae_control/fsae_control/brake_sysid.py`

Does: open-loop braking system-ID node (`brake_sysid` script): commands fixed brake levels at set speeds and logs the achieved deceleration.
Change it to:
- Add brake levels or speeds to the sweep, then run `ros2/run_brake_sysid.sh` and analyse with `tuner/investigations/brake_sysid_analysis.py`.
Don't:
- Run it beside the control stack, because two publishers on `/fsds/control_command` corrupt the measurement.
Key API: `BrakeSysID`, `main`

### `fsds_simulator/control/fsae_control/fsae_control/mpc/mpc_controller.py`

Does: the ROS node named `controller` for both MPC types. Declares parameters, builds `MPCController` and `NMPCController`, wires topics, loads precomputed path and speed CSVs, and inherits the per-tick and debug mixins.
Change it to:
- Add a node parameter or topic. Topology switches such as `standalone_output` stay plain node parameters, tuning weights go on the dataclasses.
- Change startup wiring. FSDS session only.
Don't:
- Subscribe to raw `/fsds/testing_only/odom`, because pose and speed then come from different instants. Use `/fsae/slam/car_odom`.
- Put per-tick logic here, because it belongs in `control_step.py`.
Key API: `MPCControllerNode`, `main`

### `fsds_simulator/control/fsae_control/fsae_control/mpc/control_step.py`

Does: `_ControlStepMixin`, the 20 Hz timer body: GO hold, stale path or pose brake and MPC reset, speed target with gates and rate limits, solve, cone-proximity brake, command publish and logging.
Change it to:
- Change speed-target shaping, for example the rise limit. Edit `node_constants.py` for the number, mirror offline `sim/rollout/speed_target.py`, run the rollout check.
Don't:
- Change gate or limiter order without the offline twin, because the target then differs between sim and car.
Key API: `_ControlStepMixin._control_step`, `_ControlStepMixin._check_cone_proximity`

### `fsds_simulator/control/fsae_control/fsae_control/mpc/debug_publish.py`

Does: `_DebugPublishMixin`: scores completed laps, matures horizon-accuracy predictions, publishes the weighted cost breakdown for `live_viz`.
Change it to:
- Add a debug term, then add the same term to `live_viz/panels.py`.
Don't:
- Make control decisions from these values, because they exist for display and logging only.
Key API: `_DebugPublishMixin._process_lap_and_horizon`, `_DebugPublishMixin._publish_debug_weights`

### `fsds_simulator/control/fsae_control/fsae_control/mpc/node_constants.py`

Does: node-level constants: `CONTROL_HZ`, cone-brake corridor, path timeout, speed-target rise and fall rates, gate rate limit.
Change it to:
- Retune `SPEED_TARGET_RISE_RATE`, `V_CURV_FALL_RATE` or `GATE_RATE_LIMIT`. Set the same value in `sim/rollout/speed_target.py`, then run the rollout check.
Don't:
- Change `CONTROL_HZ` alone, because it must equal `1 / dt` of both controllers and the `pose_rate` floor in `fsae_params.yaml`.
Key API: `CONTROL_HZ`, `CONE_BRAKE_DIST`, `PATH_TIMEOUT`, `SPEED_TARGET_RISE_RATE`, `V_CURV_FALL_RATE`, `GATE_RATE_LIMIT`

### `fsds_simulator/control/fsae_control/fsae_control/mpc/mpc_params.py`

Does: `MPCParams` dataclass, the live single source of truth for LTV-QP weights, adaptive-gain shape constants and flags, plus the NMPC weight overrides. 69 fields. Generates ROS parameter declarations and launch arguments from field metadata.
Change it to:
- Retune a default, for example `q_e_y`. Then update `fsae_params.yaml` (it overrides the default at declare time), `settings/lmpc.py`, and check `launch_all.sh` overrides. Run the rollout check.
Don't:
- Edit the default without `fsae_params.yaml`, because the YAML value wins and the old number keeps running silently.
- Treat values as unit-normalised, because each weight multiplies a raw-unit error term.
Key API: `MPCParams`, `DEFAULT_MPC_PARAMS`, `MPC_PARAM_FIELDS`, `declare_mpc_params`, `mpc_params_from_node`

### `fsds_simulator/control/fsae_control/fsae_control/mpc/nmpc_params.py`

Does: `NMPCParams` dataclass: NMPC structural and solver fields (horizon, SQP iterations, trust region, substeps, flags, `use_nmpc`). 35 fields.
Change it to:
- Change a solver field, then mirror into `settings/nmpc.py` and `fsae_params.yaml`, run `nmpc_offline_check` and the rollout check.
Don't:
- Add weight fields here, because NMPC weight overrides live in `mpc_params.py`.
Key API: `NMPCParams`, `DEFAULT_NMPC_PARAMS`, `NMPC_PARAM_FIELDS`, `declare_nmpc_params`, `nmpc_params_from_node`

### `fsds_simulator/control/fsae_control/fsae_control/lmpc/`

The LTV-QP controller. Package `__init__.py` (`fsds_simulator/control/fsae_control/fsae_control/lmpc/__init__.py`) documents the state and input vectors (8 states, 2 inputs) and re-exports the public and private names the node and the NMPC check import.

### `fsds_simulator/control/fsae_control/fsae_control/lmpc/constants.py`

Does: physical limits shared by both controllers: `MAX_STEER_RAD` (25 degrees), `MAX_ACCEL`, `MAX_BRAKE`.
Change it to:
- Nothing without the offline equivalent, `max_steer` in `model/vehicle_physics/params.py` (also 25 degrees). Run the rollout check after any change.
Don't:
- Re-sync `MAX_STEER_RAD` from offline unchanged, because the live value is deliberately 25 degrees to match `control_utils.py` and `fsds_bridge.py`.
Key API: `MAX_STEER_RAD`, `MAX_ACCEL`, `MAX_BRAKE`

### `fsds_simulator/control/fsae_control/fsae_control/lmpc/predict.py`

Does: `predict_ahead` rolls the measured error state forward through commands issued but not yet visible in the pose (delay compensation).
Change it to:
- Change the rollforward cap or clip. Mirror the offline `predict_ahead` in `sim/rollout/delay.py`, run the rollout check.
Don't:
- Deepen the rollforward, because it iterates the linear model with no ground-truth correction, so pose noise compounds into steering thrash and late turn-in.
Key API: `predict_ahead`

### `fsds_simulator/control/fsae_control/fsae_control/lmpc/adaptive_gains.py`

Does: current-state gain scheduling: R and Q scaling, steering-rate anti-hunt, reversal-penalty boost, corner factor, low-speed corner boost.
Change it to:
- Change a shaping function. Apply the same edit to offline `controller/model_utils.py`, run the rollout check.
Don't:
- Edit one side only, because these are numerically identical copies kept in step by hand.
Key API: `_adaptive_R_scaling`, `_steer_rate_anti_hunt`, `_reversal_penalty_boost`, `_adaptive_Q_scaling`, `_corner_factor`, `_blend`, `_low_speed_corner_boost`

### `fsds_simulator/control/fsae_control/fsae_control/lmpc/controller.py`

Does: `MPCController`: builds the persistent CVXPY problem, discretises the model, computes the delay-compensated error state, solves with OSQP then Clarabel, returns steering, throttle and brake.
Change it to:
- Change cost or constraint structure, for example a new slack term. Mirror offline `controller/lmpc/build.py` and `solve.py`, run the rollout check.
Don't:
- Change cost structure on one side only, because tuned weights then stop transferring.
Key API: `MPCController`, `compute`, `reset`, `set_heading_profile`

### `fsds_simulator/control/fsae_control/fsae_control/nmpc/`

The nonlinear controller. Package `__init__.py` (`fsds_simulator/control/fsae_control/fsae_control/nmpc/__init__.py`) states why it exists (the LTV-QP model has no term for the path bending, so it cannot turn in before error exists) and re-exports `NMPCController` and helpers. Each file below has a same-named offline twin in `controller/nmpc/`.

### `fsds_simulator/control/fsae_control/fsae_control/nmpc/layout.py`

Does: state, input and output indices and sizes (`IDX_*`, `NX`, `NU`, `NH_*`), finite-difference step sizes, Frenet denominator guard.
Change it to:
- Add a state or output row, then update every module that indexes through these names. Run `nmpc_offline_check`.
Don't:
- Retune `_FD_EPS_X` or `_FD_EPS_U` casually, because they balance truncation against round-off per variable and `nmpc_offline_check` compares them with central differences.
Key API: `IDX_VX`, `NX`, `NU`, `NH_TRACKING`, `NH_PROGRESS`, `_wrap`

### `fsds_simulator/control/fsae_control/fsae_control/nmpc/dynamics.py`

Does: Frenet-frame vehicle model: plant constants, tyre forces, continuous dynamics `_f` and `_f_scalar`, RK4 steps `_step` and `_step_scalar`.
Change it to:
- Change the model, then edit both the scalar and vectorised versions and the offline twin. `nmpc_offline_check` parity test catches a mismatch. Also run `python -m tuner.validation.plant_openloop_validation` if the plant changes.
Don't:
- Imitate the FSDS lateral-acceleration ceiling with tyre parameters, see [vehicle_physics.md](../reference/vehicle_physics.md).
Key API: `_Plant`, `_tyre_forces`, `_f`, `_f_scalar`, `_step`, `_step_scalar`

### `fsds_simulator/control/fsae_control/fsae_control/nmpc/reference.py`

Does: `PathReference`: arc-length and curvature reference from a path (spline curvature, front-axle projection, xy and heading lookup by arc length).
Change it to:
- Change curvature estimation. Run the rollout check, curvature spikes affect corner behaviour.
Don't:
- Remove the old moving-average branch without checking `nmpc_spline_reference_enabled` A/B use.
Key API: `PathReference`, `kappa_at`, `psi_ref_at`, `xy_at`, `project`

### `fsds_simulator/control/fsae_control/fsae_control/nmpc/outputs.py`

Does: `_outputs`: the residual rows the NMPC cost weighs, plus optional friction-circle rows.
Change it to:
- Add a cost residual. Extend `layout.py` sizes and the weight vector together, run `nmpc_offline_check`.
Don't:
- Weight the friction-circle rows into the cost, because they exist only to build QP constraints.
Key API: `_outputs`

### `fsds_simulator/control/fsae_control/fsae_control/nmpc/weight_schedule.py`

Does: steering-rate weight scheduling: curvature-zone scale `_rrate_zone_scale` and per-stage ramp `_rrate_stage_ramp`.
Change it to:
- Retune zone behaviour through the `nmpc_rrate_zone_*` fields first. Edit this file only for the shape, then run the rollout check.
Don't:
- Change the composition with `nmpc_rjerk_delta`, because both shipped fields act together.
Key API: `_rrate_zone_scale`, `_rrate_stage_ramp`

### `fsds_simulator/control/fsae_control/fsae_control/nmpc/qp_model.py`

Does: `_QPModelMixin`: builds the fixed-sparsity OSQP problem once, evaluates rollout, Jacobians and cost.
Change it to:
- Add a cost term, for example a new penalty. Keep the sparsity pattern fixed or rebuild it in `_build_qp`. Run `nmpc_offline_check` and the rollout check.
Don't:
- Change sparsity per tick, because the solver updates values in place in a fixed pattern.
Key API: `_QPModelMixin`, `_build_qp`, `_rollout`, `_jacobians`, `_cost`, `_cost_breakdown`, `_csc_pattern`

### `fsds_simulator/control/fsae_control/fsae_control/nmpc/sqp_step.py`

Does: `_SQPStepMixin`: one Gauss-Newton SQP step (linearise, update OSQP, return correction) and feasibility projection.
Change it to:
- Change the trust region or backtracking. Run `nmpc_offline_check` (monotonic convergence) and the rollout check.
Don't:
- Raise SQP iterations to chase steering spikes, because that was measured not to fix them and solve time can exceed the 50 ms tick.
Key API: `_SQPStepMixin`, `_solve_step`, `_project_feasible`

### `fsds_simulator/control/fsae_control/fsae_control/nmpc/solver.py`

Does: `NMPCController`: construction, reset, static path handling, the per-tick `compute`, path-reference helpers; inherits the two mixins.
Change it to:
- Change per-tick flow (delay handling, kappa rate limit, warm start). Apply the same edit offline, run both NMPC checks.
Don't:
- Edit warm-start or state anchoring without reading [nmpc.md](../controllers/nmpc.md), because a wrong change here passes offline checks yet fails on the car.
Key API: `NMPCController`, `compute`, `set_static_path`, `set_heading_profile`, `reset`

### `fsds_simulator/control/fsae_control/fsae_control/telemetry/`

CSV logging and scoring. Package `__init__.py` (`fsds_simulator/control/fsae_control/fsae_control/telemetry/__init__.py`) documents the CSV column contract and re-exports `ControlLogger`, `LapProgressTracker`, `HorizonAccuracyTracker`, `build_config_lines`, `ADAPTIVE_COLUMNS`. Retention: run CSVs go to `fsae_logs/` with no rotation.

### `fsds_simulator/control/fsae_control/fsae_control/telemetry/columns.py`

Does: `ADAPTIVE_COLUMNS`: ordered trailing CSV columns fed from controller `last_telemetry`.
Change it to:
- Add a column and fill it in both controllers, so the column set stays identical.
Don't:
- Reorder existing entries, because analysers read by position and header.
Key API: `ADAPTIVE_COLUMNS`

### `fsds_simulator/control/fsae_control/fsae_control/telemetry/config_lines.py`

Does: renders launch configuration as `#` comment lines at the top of the CSV so a file reproduces its run.
Change it to:
- Add a new launch flag to the header.
Don't:
- Drop a field, because the header is the only record of the launch configuration a CSV came from.
Key API: `build_config_lines`

### `fsds_simulator/control/fsae_control/fsae_control/telemetry/horizon_tracker.py`

Does: `HorizonAccuracyTracker`: compares NMPC predicted trajectory with where the car went, per lap.
Change it to:
- Change the accuracy formula. It is independent of the composite score.
Don't:
- Feed it into the score, because it is live-only.
Key API: `HorizonAccuracyTracker`, `add_pose`, `add_prediction`, `update`, `pop_lap_mean`

### `fsds_simulator/control/fsae_control/fsae_control/telemetry/lap_progress.py`

Does: `LapProgressTracker`: progress, completion and time bonus from car position against the precomputed path.
Change it to:
- Change lap start or finish detection, then check the score header of a live run.
Don't:
- Expect meaningful values without a loaded path, because the score is then marked partial.
Key API: `LapProgressTracker`, `update`, `result`

### `fsds_simulator/control/fsae_control/fsae_control/telemetry/scoring.py`

Does: live copy of `sim/scoring.py`: composite score from the same metrics and weights.
Change it to:
- Nothing directly. Edit `sim/scoring.py`, copy the function bodies here, keep the inlined constants equal to `settings/scoring.py`. Run the rollout check.
Don't:
- Edit the formula here first, because live and offline scores then stop being comparable.
Key API: `compute_composite_score`, `RolloutMetrics`, `SCORE_WEIGHTS`, `METRIC_SCALES`

### `fsds_simulator/control/fsae_control/fsae_control/telemetry/control_logger.py`

Does: `ControlLogger`: writes control and path CSVs, accumulates score metrics, writes the score and config header on close.
Change it to:
- Add a logged signal in `log_control`, then update the column contract in the package docstring.
Don't:
- Pass normalised steering, because `steer_deg` is computed from radians and inflates about 2.3 times.
Key API: `ControlLogger`, `log_control`, `log_path`, `finish_lap`, `close`

### `fsds_simulator/control/fsae_control/fsae_control/live_viz/`

Debug window, sim-only, not launched on the car. Package `__init__.py` (`fsds_simulator/control/fsae_control/fsae_control/live_viz/__init__.py`) lists subscribed topics and re-exports `main`, `LiveVizNode`, `get_car_triangle`. Launched by `ros2/launch_all.sh` on the host (not in the Docker path).

### `fsds_simulator/control/fsae_control/fsae_control/live_viz/panels.py`

Does: window size and refresh constants, ordered term lists behind each bar panel. Pure data.
Change it to:
- Add a bar term to match a new debug value.
Don't:
- Import ROS or matplotlib here, because the module is meant to load without them.
Key API: `VIEW_HALF_WIDTH`, `REDRAW_HZ`, `TRAIL_MAXLEN`, `DEBUG_BAR_GROUPS`, `DEBUG_HORIZON_TERMS`, `STANLEY_ERROR_TERMS`, `STANLEY_LAW_TERMS`

### `fsds_simulator/control/fsae_control/fsae_control/live_viz/node.py`

Does: `LiveVizNode` subscribes to the ROS topics and caches the latest message; `get_car_triangle` builds the car marker.
Change it to:
- Subscribe to a new topic.
Don't:
- Do drawing here, because it belongs in `app.py`.
Key API: `LiveVizNode`, `get_car_triangle`

### `fsds_simulator/control/fsae_control/fsae_control/live_viz/app.py`

Does: builds the matplotlib figure, redraws from the node on a timer, provides `main`.
Change it to:
- Add a panel or plot element.
Don't:
- Move the Tk backend selection below the pyplot import, because it must run first.
Key API: `main`

## fsae_planning

Planner package. Console scripts in `fsds_simulator/planning/fsae_planning/setup.py`: `centerline_planner`, `skidpad_planner`. Offline copies of the shared geometry live in `planning/`. Read the known curvature-spike defect in [simulator_fidelity.md](../reference/simulator_fidelity.md) before changing smoothing, the boundary walk or cone sorting: the controller carries workarounds for it.

### `fsds_simulator/planning/fsae_planning/setup.py`

Does: builds the package and registers the two planner scripts.
Change it to:
- Register a new planner node.
Don't:
- Rely on its launch-folder data glob, because the package has no launch folder (harmless, the launch file lives in `fsae_bringup`).
Key API: `entry_points['console_scripts']`

### `fsds_simulator/planning/fsae_planning/setup.cfg`

Does: installs scripts under the package's own lib folder.
Change it to:
- Nothing in normal work.
Don't:
- Delete it, because `ros2 run` then fails to find the scripts.
Key API: `[develop]`, `[install]`

### `fsds_simulator/planning/fsae_planning/package.xml`

Does: declares dependencies (`geometry_msgs`, `fsae_interfaces`, `std_msgs`, numpy, scipy, matplotlib).
Change it to:
- Add a dependency when the planner imports a new package.
Don't:
- Forget `fsae_interfaces`, because the planner subscribes to `Track`.
Key API: `exec_depend`, `depend`

### `fsds_simulator/planning/fsae_planning/test/test_copyright.py`

Does: stock ament copyright lint test.
Change it to:
- Nothing.
Don't:
- Treat a failure as a project gate, because no linter or licence policy is enforced for this project.
Key API: `test_copyright`

### `fsds_simulator/planning/fsae_planning/test/test_flake8.py`

Does: stock ament flake8 lint test.
Change it to:
- Nothing.
Don't:
- Treat it as a project gate, because no linter is configured for this project.
Key API: `test_flake8`

### `fsds_simulator/planning/fsae_planning/test/test_pep257.py`

Does: stock ament docstring lint test.
Change it to:
- Nothing.
Don't:
- Treat it as a project gate, same reason.
Key API: `test_pep257`

### `fsds_simulator/planning/fsae_planning/fsae_planning/centerline_planner.py`

Does: centreline planner node: builds a rolling-window centreline from the accumulated cone map each cycle and publishes `/fsae/planning/selected_trajectory`.
Change it to:
- Retune `smooth`, `plan_horizon`, `path_blend`, `look_radius` (defaults in the `centerline_planner` block of `fsae_params.yaml`). Rollout check plus FSDS.
Don't:
- Shorten `plan_horizon` below the controller speed-scan window, because a tight corner is then revealed too late to brake for.
Key API: `CenterlinePlanner`, `main`

### `fsds_simulator/planning/fsae_planning/fsae_planning/boundary.py`

Does: cone-wall mesh planner: joins same-colour cones into walls, generates midpoints, chains them with a wall-crossing penalty, clamps to a horizon and smooths.
Change it to:
- Change midpoint chaining or the wall horizon. Mirror offline `planning/boundary.py`, run the rollout check.
Don't:
- Change the wall horizon without keeping `plan_horizon` and `look_radius` in `fsae_params.yaml` consistent with it, because a shorter horizon reveals corners too late.
Key API: `build_path_walls`, `build_wall_segments`, `segment_crosses_walls`

### `fsds_simulator/planning/fsae_planning/fsae_planning/cone_map.py`

Does: persistent cone map that merges detections within a merge distance and never removes cones.
Change it to:
- Change merge distance or dedupe logic. `cone_recorder.py` carries a copy of the merge routine, change both.
Don't:
- Remove cones that leave the field of view, because historical walls are needed for planning.
Key API: `ConeMap`, `update`, `reset`, `blue`, `yellow`

### `fsds_simulator/planning/fsae_planning/fsae_planning/cone_sorting.py`

Does: cone ordering, pairing and forward or windowed filtering on (N, 2) arrays.
Change it to:
- Change a filter window. Mirror offline `planning/cone_sorting.py`.
Don't:
- Change the frame convention, because all planning code assumes ENU with x forward and y left.
Key API: `sort_cones_nn`, `pair_cones_nn`, `filter_cones_forward`, `filter_cones_window`

### `fsds_simulator/planning/fsae_planning/fsae_planning/path_utils.py`

Does: centreline computation, smoothing, path blending, lookahead waypoint and loop rolling helpers.
Change it to:
- Change smoothing. Mirror offline `planning/path_utils.py`, run the rollout check.
Don't:
- Change smoothing without re-measuring curvature spikes, because controller workarounds depend on the current behaviour.
Key API: `compute_centreline`, `smooth_centreline`, `build_local_path`, `get_lookahead_waypoint`, `blend_paths`, `roll_loop_to_car`

### `fsds_simulator/planning/fsae_planning/fsae_planning/special_utils/skidpad.py`

Does: skidpad geometry: clusters cones into two circles, fits them, stitches a closed figure-8 centreline, measures path deviation.
Change it to:
- Change the figure-8 construction. FSDS skidpad run only.
Don't:
- Use it for ordinary tracks, because it assumes two circles.
Key API: `Figure8Track`, `fit_circle`, `build_figure8`, `path_deviation`

### `fsds_simulator/planning/fsae_planning/fsae_planning/special_utils/skidpad_planner.py`

Does: sim-only skidpad characterisation node: laps the figure-8 at a rising speed until the car slides off, drives the car itself through `/fsae/control/cmd_vel` and logs the departure.
Change it to:
- Retune `v_start`, `ramp_accel`, `v_cap` in the `skidpad_planner` block of `fsae_params.yaml`.
Don't:
- Launch a controller beside it, because the planner publishes `cmd_vel` directly.
Key API: `SkidpadPlanner`, `main`

### `fsds_simulator/planning/fsae_planning/fsae_planning/special_utils/speed_input.py`

Does: manual target-speed entry (Tk box on a thread, terminal fallback) for skidpad runs.
Change it to:
- Change the input limits.
Don't:
- Block the ROS executor, because input runs on a daemon thread by design.
Key API: `SpeedInput`, `start`, `latest`, `stop`

## fsae_sim_perception

Stand-in for the real car's camera and SLAM front end. Console scripts in `fsds_simulator/perception/fsae_sim_perception/setup.py`: `sim_perception`, `cone_recorder`.

### `fsds_simulator/perception/fsae_sim_perception/setup.py`

Does: builds the package and registers `sim_perception` and `cone_recorder`.
Change it to:
- Register a new perception tool.
Don't:
- Add a Python import of `fsae_planning`, because this package deliberately depends only on `fsae_interfaces`.
Key API: `entry_points['console_scripts']`

### `fsds_simulator/perception/fsae_sim_perception/setup.cfg`

Does: installs scripts under the package's own lib folder.
Change it to:
- Nothing in normal work.
Don't:
- Delete it, same reason as the other `setup.cfg` files.
Key API: `[develop]`, `[install]`

### `fsds_simulator/perception/fsae_sim_perception/package.xml`

Does: declares perception dependencies.
Change it to:
- Add a dependency for a new import.
Don't:
- Leave out `fs_msgs`, because the node reads FSDS messages.
Key API: `exec_depend`

### `fsds_simulator/perception/fsae_sim_perception/fsae_sim_perception/sim_perception.py`

Does: crops the FSDS oracle cone map to a forward window and republishes pose, an atomic odometry snapshot, boundary tracks and cone detections on the topics the car stack expects.
Change it to:
- Retune FOV and rates in the `sim_perception` block of `fsae_params.yaml`. Rollout check does not cover it, use an FSDS session.
Don't:
- Set `pose_rate` below the controller rate, because the MPC then re-solves on unchanged poses and over-corrects.
- Treat the pose as SLAM output, because it is ground truth with no noise.
Key API: `SimPerception`, `main`

### `fsds_simulator/perception/fsae_sim_perception/fsae_sim_perception/cone_recorder.py`

Does: records one lap of boundary cones to a JSON file for later replay, then optionally exits.
Change it to:
- Change lap-close detection (`min_lap_dist`, `close_dist`, `max_record_time`).
Don't:
- Import `ConeMap` from `fsae_planning`, because the merge routine is copied on purpose, keep the two in step.
Key API: `ConeRecorder`, `main`

## fsae_bringup

Launch composition and the central parameter file. It has no console scripts (`fsds_simulator/common/fsae_bringup/setup.py` installs `launch/*.launch.py` and `config/*.yaml`).

### `fsds_simulator/common/fsae_bringup/setup.py`

Does: builds the package and installs launch and config files into the package's share folder.
Change it to:
- Add a new launch or config glob.
Don't:
- Add Python modules here expecting `ros2 run`, because `entry_points` is empty.
Key API: `data_files`

### `fsds_simulator/common/fsae_bringup/setup.cfg`

Does: standard script directory settings.
Change it to:
- Nothing.
Don't:
- Delete it.
Key API: `[develop]`, `[install]`

### `fsds_simulator/common/fsae_bringup/package.xml`

Does: declares dependence on the other three code packages.
Change it to:
- Add a package to the launch composition.
Don't:
- Drop an `exec_depend`, because launch then fails on a missing package only at run time.
Key API: `exec_depend`

### `fsds_simulator/common/fsae_bringup/config/fsae_params.yaml`

Does: central ROS parameters per node name: `sim_perception`, `centerline_planner`, `skidpad_planner`, `controller` (speed caps plus every `MPCParams` and `NMPCParams` field), `fsds_bridge`. Every dataclass field has a key with the same default (checked).
Change it to:
- Retune a controller value together with the dataclass default and `settings/`. Keys must equal the node name.
Don't:
- Edit only the dataclass, because the YAML value wins at declare time.
- Expect `v_max` here to be the run value, because `launch_all.sh` passes its own `V_MAX` (20.0 at the time of writing, YAML 15.0).
Key API: `sim_perception`, `centerline_planner`, `skidpad_planner`, `controller`, `fsds_bridge`

### `fsds_simulator/common/fsae_bringup/launch/sim.launch.py`

Does: top-level bring-up: perception, planner (skipped with `use_precomputed_path=true`), control and cone recorder. Declares every MPC and NMPC launch argument from field metadata and forwards them.
Change it to:
- Change a top-level default (`planner`, `controller`, `record_cones`). Prefer overriding in `ros2/launch_all.sh`.
Don't:
- Hand-write per-field arguments, because they are generated from `MPC_PARAM_FIELDS` and `NMPC_PARAM_FIELDS`.
- Trust the hard-coded absolute `map_path` defaults on another machine, because they point at this host's checkout.
Key API: `generate_launch_description`

### `fsds_simulator/common/fsae_bringup/launch/control.launch.py`

Does: launches the selected controller node and `fsds_bridge`, resolves precomputed path and speed toggles, and prints warnings for no-op flag combinations.
Change it to:
- Add a controller-level launch argument, then forward it from `sim.launch.py`.
Don't:
- Pass `use_nmpc` or the heading-profile flag to Stanley, because that node does not declare them.
Key API: `generate_launch_description`

### `fsds_simulator/common/fsae_bringup/launch/perception.launch.py`

Does: launches `sim_perception` with the YAML and a `full_track` argument.
Change it to:
- Change how perception is launched.
Don't:
- Rename the node, because its YAML block key equals the node name.
Key API: `generate_launch_description`

### `fsds_simulator/common/fsae_bringup/launch/planning.launch.py`

Does: launches one planner chosen by `planner` (`centerline_planner` or `skidpad_planner`).
Change it to:
- Add a planner option.
Don't:
- Give the node a name that differs from its executable, because YAML matching depends on it.
Key API: `generate_launch_description`

### `fsds_simulator/common/fsae_bringup/launch/cone_recorder.launch.py`

Does: launches `cone_recorder` with an `out_path` argument.
Change it to:
- Change default output handling.
Don't:
- Make it publish, because it only subscribes.
Key API: `generate_launch_description`

## fsae_interfaces

Message package (CMake). `fsds_simulator/common/fsae_interfaces/CMakeLists.txt` and `fsds_simulator/common/fsae_interfaces/package.xml` are its only build files.

### `fsds_simulator/common/fsae_interfaces/CMakeLists.txt`

Does: generates the nine message types with `rosidl_generate_interfaces`.
Change it to:
- Add a `.msg` file to `msg_files`, rebuild the workspace.
Don't:
- Add a message without a consumer, because unused messages were pruned on purpose.
Key API: `msg_files`

### `fsds_simulator/common/fsae_interfaces/package.xml`

Does: declares `ament_cmake`, `std_msgs`, `geometry_msgs`, rosidl generators.
Change it to:
- Add a dependency when a message embeds a new type.
Don't:
- Forget the matching `find_package` in `CMakeLists.txt`.
Key API: `depend`, `build_depend`

### `fsds_simulator/common/fsae_interfaces/msg/*.msg`

Does: the message set: `fsds_simulator/common/fsae_interfaces/msg/Track.msg` (boundary cone points), `fsds_simulator/common/fsae_interfaces/msg/ConeDetection.msg` (local detections plus car pose), `fsds_simulator/common/fsae_interfaces/msg/AllTrajectories.msg`, `fsds_simulator/common/fsae_interfaces/msg/CAN.msg`, `fsds_simulator/common/fsae_interfaces/msg/CANStamped.msg`, `fsds_simulator/common/fsae_interfaces/msg/HardwareStates.msg`, `fsds_simulator/common/fsae_interfaces/msg/HardwareStatesStamped.msg`, `fsds_simulator/common/fsae_interfaces/msg/MissionStates.msg`, `fsds_simulator/common/fsae_interfaces/msg/MissionStatesStamped.msg`. The sim stack uses `Track` and `ConeDetection`. The rest mirror the production car interface.
Change it to:
- Add a field, then update every publisher and subscriber and rebuild everything.
Don't:
- Change a field on one side of the sim and car, because message layouts must match `fsae_autonomous`.
Key API: `Track`, `ConeDetection`

## Workspace-level files

### `fsds_simulator/requirements.txt`

Does: pip and system dependency list for the workspace, with non-pip ROS and source-built packages in comments.
Change it to:
- Add a pip package when a node imports it.
Don't:
- Trust it as complete, because it omits the solver packages (`cvxpy`, `osqp`, `clarabel`) that the Docker file installs, and it names `viz_utils` and `launch_terminals.sh`, neither of which exists in the mirror.
Key API: pip section, apt comments

### `fsds_simulator/fsds_ros2_custom.Dockerfile`

Does: builds a ROS Jazzy image with `ackermann_msgs`, `cvxpy`, `osqp`, `matplotlib` and pinned `setuptools` for the containerised run path.
Change it to:
- Add a pip package the controller needs.
Don't:
- Reorder the `--no-deps` and `--ignore-installed` installs, because they work around version conflicts in the base image.
Key API: `FROM osrf/ros:jazzy-desktop`

### `fsds_simulator/launch_all.sh`

Does: mirror copy of `ros2/launch_all.sh`, byte-identical when last checked.
Change it to:
- Copy it from `ros2/launch_all.sh` after editing there, never diverge.
Don't:
- Run it from the mirror, because it derives paths from its own location and expects an `install` folder and a tracks folder under `src` beside it.
Key API: same as the outer script

## Outer-repo scripts that drive the workspace

These live in the outer FSDS repo, not in this repo. Paths are outer-repo-relative.

### `ros2/launch_all.sh`

Does: one-command launcher: starts FSDS on Windows, waits for its RPC port, starts `fsds_ros2_bridge`, optional diagnostics and `live_viz`, then runs `sim.launch.py` in the foreground and cleans up on exit.
Change it to:
- Pick a track (`TRACK`), controller (`CONTROLLER`, `USE_NMPC`, `STANDALONE_OUTPUT`), speed caps (`V_MAX`, `V_MIN`) or uncomment a shortlisted override such as `NMPC_Q_E_Y`. The `_append_mpc_arg` list turns each into a launch argument. After editing, copy to `fsds_simulator/launch_all.sh`.
Don't:
- Assume source edits are live, because the `colcon build --symlink-install` step is commented out and a stale build silently runs old code.
- Expect a shortlist value to match the YAML, because a launch argument overrides both YAML and dataclass.
Key API: `TRACK`, `CONTROLLER`, `USE_NMPC`, `MPC_LAUNCH_ARGS`, `_append_mpc_arg`, `cleanup`

### `ros2/run_brake_sysid.sh`

Does: one-shot harness for the braking system-ID: starts FSDS and the bridge, runs `brake_sysid`, analyses the log, tears down.
Change it to:
- Change sweep arguments via `-p name:=value`. Point its analysis step at `tuner.investigations.brake_sysid_analysis`, it still calls the old `tuner.checks` path (see the report).
Don't:
- Start the control stack alongside it.
Key API: `--quick`, `--no-sim`

### `ros2/clock_drift_check.py`

Does: subscribes to `/clock` and logs wall time against sim time to CSV, to test whether FSDS falls behind real time during the periodic pose stall.
Change it to:
- Nothing, it is a temporary diagnostic started by the diagnostic block of `ros2/launch_all.sh`.
Don't:
- Read a ratio near 1.0 as clearing the stall, because it only rules out the simulation clock.
Key API: `ClockDriftCheck`

### `check_fsds.sh`

Does: at the outer repo root, checks that exactly one FSDS process serves RPC on port 41451 before the bridge starts.
Change it to:
- Change the default host address.
Don't:
- Ignore a CloseWait count, because it means connections landed on a dead instance.
Key API: script argument `HOST`

## Mirror workflow

`fsds_simulator/` is a PR-staging snapshot of the live workspace. It exists so changes to the ROS 2 side have a git history, because nothing is committed or pushed to the `fsae_planning` repo itself. It is not imported by anything in this repo.

### Rules

- **Byte-identical, not just similar.** Every file under `fsds_simulator/` that also exists in `ros2/src/fsae_planning` must match it. Verified at the time of writing: only the repo-specific files differ (see the table).
- **Edit either side, then copy to the other.** Make the change in `ros2/src/fsae_planning` or in `fsds_simulator/`, then copy the file to the same relative path on the other side. Both must end byte-identical. The mirror commit is the change record. Never commit or push in `fsae_planning`.
- **Do not add files that were never there, and do not fix unrelated drift.** Propagate the specific change only.
- **Check on disk, not with `git status`.** An ignore pattern once hid a whole mirrored package from `git status`. Use `ls`, `find` or `diff`.
- **Params move the other way.** A retuned and live-validated param value goes from `ros2/src/fsae_planning` into the mirror and into the local `fsae_autonomous` tree with the sync tool below.
- **`fsae_autonomous` gets local edits only.** Pushing it is a separate, human-initiated step.

Files that differ by design:

| File | Live tree | Mirror |
|---|---|---|
| `fsds_simulator/README.md` | project README of `fsae_planning` | describes the mirror |
| `fsds_simulator/launch_all.sh` | none, the live script is `ros2/launch_all.sh` in the outer repo | copy of it |
| `fsds_simulator/fsds_ros2_custom.Dockerfile` | none | mirror only |
| `.gitignore`, `CHANGES.md`, `launch_terminals.sh` | present | absent |
| `tracks/`, `cone_maps/`, `recorded_runs/` | canonical track data in `tracks/` | snapshot data |

Check the mirror against the live tree from the repo root:

```bash
diff -rq --exclude=__pycache__ --exclude=build --exclude=install --exclude=log --exclude=tracks --exclude=cone_maps --exclude=recorded_runs --exclude='*.bak' --exclude=README.md fsds_simulator ../ros2/src/fsae_planning
```

Expected output lists only the table rows above (`.gitignore`, `CHANGES.md`, `launch_terminals.sh`, `launch_all.sh`, the Dockerfile).

### How live and offline parity works by hand

The offline stack cannot import the live one and the live one cannot import `settings/`, so the same numbers exist twice and are kept equal by hand. A divergence does not fail a build. It makes an offline-tuned weight set wrong on the car. Field-by-field tables are in [offline_live_parity.md](../reference/offline_live_parity.md).

| Live side | Offline side |
|---|---|
| `fsds_simulator/control/fsae_control/fsae_control/mpc/mpc_params.py` (`MPCParams`, 69 fields) | `settings/lmpc.py` and `settings/nmpc.py` (weight overrides), one uppercase constant per field |
| `fsds_simulator/control/fsae_control/fsae_control/mpc/nmpc_params.py` (`NMPCParams`, 35 fields) | `settings/nmpc.py` |
| `fsds_simulator/common/fsae_bringup/config/fsae_params.yaml` `controller:` block | same defaults as the two dataclasses (checked equal) |
| `fsds_simulator/control/fsae_control/fsae_control/mpc/node_constants.py` | `sim/rollout/speed_target.py` |
| `fsds_simulator/control/fsae_control/fsae_control/lmpc/`, `nmpc/` | `controller/lmpc/`, `controller/model_utils.py`, `controller/nmpc/` |
| `fsds_simulator/control/fsae_control/fsae_control/telemetry/scoring.py` | `sim/scoring.py` (source of truth) and `settings/scoring.py` |

Precedence at launch, highest first: `launch_all.sh` launch argument, `fsae_params.yaml`, dataclass default. A change made in a lower layer is invisible while a higher layer sets the same field.

Scoring: the function bodies of `compute_composite_score`, `RolloutMetrics.add_step` and `RolloutMetrics.finalize` are equal to `sim/scoring.py` apart from docstrings and the replacement of `settings.X` by inlined module constants. The constants (`SCORE_WEIGHTS`, `METRIC_SCALES`, penalties, `CONSTRAINT_FLOOR`, `COMPLETION_THRESHOLD`, `TIME_OBJECTIVE_WEIGHT`, `QUALITY_WEIGHT`) match `settings/scoring.py` numerically. Change `sim/scoring.py` first and re-copy. A score is meaningful only when the caller supplies real progress, which `LapProgressTracker` does when a precomputed path is loaded.

### Syncing params with `tuner.tools.sync_mpc_params`

Run from the repo root:

```bash
python -m tuner.tools.sync_mpc_params           # dry run, prints a diff per file per destination
python -m tuner.tools.sync_mpc_params --apply   # overwrite
```

- **Scope:** exactly three files, `mpc/mpc_params.py`, `mpc/nmpc_params.py` and `common/fsae_bringup/config/fsae_params.yaml`, at the same relative paths on each side.
- **Direction:** one way, from `ros2/src/fsae_planning` into the mirror and into the local `fsae_autonomous` checkout. It never reads the destinations as a source.
- **Not covered:** `settings/` (edit by hand or through the GUI Settings tab) and every other source file. Code changes still need the manual mirror step above.
- **Safety:** dry run is the default. `--apply` writes a one-time `.bak` beside each destination before overwriting. If no `fsae_autonomous` checkout is found among its candidate paths it warns and syncs only the mirror.
- **Output:** a `SYNC-TARGET: <name> = changed|unchanged|missing` line per destination, which the GUI reads for its confirmation prompt.
- **After applying:** commit the mirror change in this repo. Committing `fsae_autonomous` stays a human step.
