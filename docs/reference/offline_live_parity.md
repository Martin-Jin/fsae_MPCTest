# Offline/Live Parity

Two different obligations share the word "parity". They are maintained differently and fail differently.

**1. The `fsds_simulator/` mirror: byte-identical copies.** `fsds_simulator/` holds copies of the live `fsae_planning` ROS 2 workspace files. They are maintained with `cp` and checked with `diff`.

```bash
diff -rq --exclude=__pycache__ \
  ../ros2/src/fsae_planning/control fsds_simulator/control
```

Nothing here needs judgement. A difference is either an unpropagated change or an accident, and the fix is to copy the file. At the time of writing the `common`, `control`, `perception` and `planning` subtrees show no difference.

**2. Offline/live numeric parity: the same number in different code.** This is not a mirror and cannot be diffed. The `settings/` package and `sim/speed_profile.py` are structurally different from the live `mpc_params.py`, `nmpc_params.py` and `control_utils.py`. The live node cannot import `settings/`, and no `settings/` exists on the car. So a value like `a_lat_max = 4.75` is typed independently into `sim/speed_profile.py` and `control_utils.py`, and nothing mechanical keeps the two equal.

The tables below record those pairs. A silent divergence does not break a build. It makes an offline-tuned weight set invalid on the car while every check still passes.

## Contents

1. [Four standing rules](#four-standing-rules)
2. [The `fsds_simulator/` mirror](#the-fsds_simulator-mirror)
3. [Numeric parity between `settings/` and the live parameters](#numeric-parity-between-settings-and-the-live-parameters)
4. [Live/offline score parity](#liveoffline-score-parity)
5. [Lap timing and live-only diagnostics](#lap-timing-and-live-only-diagnostics)
6. [Steering system-ID harness](#steering-system-id-harness)
7. [Resync procedure](#resync-procedure)

## Four standing rules

**1. The stack exists in two places and both must change.** Planning and control logic lives in the live ROS 2 nodes (`ros2/src/fsae_planning/control/fsae_control/`) and in this repo's offline simulator (`sim/rollout/`, `settings/`, `controller/`). Weights tuned offline are valid on the car only if the live code matches numerically. A one-sided edit to either copy is an incomplete change. The field-by-field mapping is in [Numeric parity](#numeric-parity-between-settings-and-the-live-parameters).

**2. One scoring formula, copied verbatim.** `sim/scoring.py` is the single source of truth. The live `telemetry/scoring.py` is a copy so a score logged on the car is comparable to an offline one. Change the offline file first, then re-copy. The only intended difference is that the live copy inlines the weight constants, because no `settings/` exists on the car. Those must stay numerically identical. See [Live/offline score parity](#liveoffline-score-parity).

**3. The offline simulator does not fully predict the car.** On the same map and gains, the live car saturates its steering far more often and carries about twice the heading error. An offline score alone is not evidence. Validate on the car before accepting a tuning result. The measured gap and its causes are in [simulator_fidelity.md](simulator_fidelity.md) and `docs/logs/sim_to_real_investigation.md`.

**4. Do not imitate the lateral-acceleration ceiling with tyre parameters.** FSDS enforces a sustained lateral-acceleration ceiling (7.5 m/s^2 at low speed, rising with speed), so it is not a grip limit. Scaling `mu` or the cornering stiffnesses to reproduce it was tried and fails: it matches one measurement while wrecking the plant's real grip and failing the full-lock and closed-loop checks. The `alat_ceiling*` model in `model/vehicle_physics/params.py` is the mechanism to tune. Details are in [simulator_fidelity.md](simulator_fidelity.md).

## The `fsds_simulator/` mirror

**What it is.** A staging copy of the whole `fsae_planning` ROS 2 workspace hierarchy, every package (`common/fsae_interfaces`, `common/fsae_bringup`, `perception/fsae_sim_perception`, `planning/fsae_planning`, `control/fsae_control`) with scaffolding (`package.xml`, `setup.py`, `setup.cfg`, `resource/`). The tree can be copied into a workspace source directory at the same relative paths. See [fsds_simulator/README.md](../../fsds_simulator/README.md) for building and running it.

**What it is not.** Nothing under `fsds_simulator/` is imported by `gui/`, `tuner/`, `sim/`, `model/` or `controller/`. It exists so this repo can hold and hand off a ready-to-build copy of the ROS 2 side, including to someone who has only this repo and FSDS.

**Rules for changing it.** A change to a file that exists in both copies is applied to both. A file that was never in the mirror is not added, and unrelated drift found in passing is not fixed. Check existence with `ls` or `find`, not `git status`, because an ignore pattern can make a present file look missing.

Stray `.bak` files (`mpc_params.py.bak`, `nmpc_params.py.bak`, `fsae_params.yaml.bak`) exist in both the live tree and the mirror. They come from the GUI's and `sync_mpc_params` backup step and are not part of the workspace.

### Root `planning/` is an algorithm-only port

`planning/` (used by `sim/` and the tuner) is not a mirror. It carries the same algorithms as the live package with intra-package imports rewritten from `fsae_planning.xxx` to `planning.xxx`.

| this repo | live package (`planning/fsae_planning/fsae_planning/`) | note |
|---|---|---|
| `planning/boundary.py` | `boundary.py` | same algorithm, imports rewritten, comments differ |
| `planning/cone_map.py` | `cone_map.py` | same logic |
| `planning/cone_sorting.py` | `cone_sorting.py` | identical |
| `planning/path_utils.py` | `path_utils.py` | same algorithms, imports rewritten |
| `planning/geometry.py` | (inside `boundary.py`) | `segment_crosses_walls` and `_seg_intersect` split out here so `boundary.py` and `path_utils.py` can both import them without a cycle. Live keeps them in `boundary.py` |
| (no file) | `centerline_planner.py` | the planner node. Offline, `SimPlanner.update()` in `sim/planner.py` plays that role |

### Mirror file map

| `fsds_simulator/` path | notes |
|---|---|
| `common/fsae_interfaces/` | message package (`Track`, `ConeDetection`, `CAN`, and others) |
| `common/fsae_bringup/` | `fsae_params.yaml` under `config/` (central tunables) and five launch files: `sim`, `control`, `planning`, `perception`, `cone_recorder` |
| `perception/fsae_sim_perception/` | `sim_perception.py` (FSDS oracle and odom to `/fsae/*`) and `cone_recorder.py` |
| `planning/fsae_planning/` | `centerline_planner.py`, `boundary.py`, `cone_map.py`, `cone_sorting.py`, `path_utils.py`, and the `special_utils` package (skidpad) |
| `control/fsae_control/fsae_control/lmpc/` | LTV-QP controller: `constants.py`, `predict.py`, `adaptive_gains.py`, `controller.py` |
| `control/fsae_control/fsae_control/nmpc/` | NMPC: `layout.py`, `reference.py`, `dynamics.py`, `outputs.py`, `weight_schedule.py`, `qp_model.py`, `sqp_step.py`, `solver.py` |
| `control/fsae_control/fsae_control/mpc/` | the node and its parameters: `mpc_controller.py`, `control_step.py`, `debug_publish.py`, `node_constants.py`, `mpc_params.py`, `nmpc_params.py` |
| `control/fsae_control/fsae_control/telemetry/` | CSV logging and scoring: `columns.py`, `config_lines.py`, `horizon_tracker.py`, `lap_progress.py`, `control_logger.py`, `scoring.py` |
| `control/fsae_control/fsae_control/live_viz/` | live debug window: `panels.py`, `node.py`, `app.py` |
| `control/fsae_control/fsae_control/` | `control_utils.py` (with `curvature_speed()` and `StanleyController`), `stanley_controller.py`, `fsds_bridge.py`, `brake_sysid.py` |
| `control/fsae_control/setup.py` | registers five console scripts: `controller`, `mpc_controller`, `fsds_bridge`, `live_viz`, `brake_sysid` |

`zip_safe=False` is set in all four mirror `setup.py` files (`common/fsae_bringup`, `control/fsae_control`, `perception/fsae_sim_perception`, `planning/fsae_planning`) and in the four live ones. It is the fix for a stale-`colcon build` bug (recorded in `docs/logs/sim_to_real_investigation.md`). Keep it on any new package on either side.

### `telemetry/scoring.py` is a copy, not an independent file

`telemetry/scoring.py` is a verbatim copy of this repo's `sim/scoring.py`. Edit `sim/scoring.py` first, then re-copy. The `fsae_planning` git history (the remote main branch at the time of writing) also carries older single-file forms of `scoring.py`, `cone_recorder.py` and `cone_recorder.launch.py`, so these files are not new to that history. What is new is their current split layout.

### The offline NMPC has one module per live module

`controller/nmpc/` mirrors `nmpc/` in the live package file for file, in the same definition order, so a diff is per module.

| module | contents |
|---|---|
| `layout.py` | `IDX_*`, `NX`, `NU`, `NH_*`, finite-difference constants, `_wrap` |
| `reference.py` | `PathReference` |
| `dynamics.py` | `_Plant`, `_tyre_forces`, `_f`, `_f_scalar`, `_step_scalar`, `_step` |
| `outputs.py` | `_outputs` |
| `weight_schedule.py` | `_rrate_zone_scale`, `_rrate_stage_ramp` |
| `qp_model.py` | QP construction (`_QPModelMixin`) |
| `sqp_step.py` | SQP iteration (`_SQPStepMixin`) |
| `solver.py` | `NMPCController` |

`controller/nmpc/__init__.py` re-exports the names, so `from controller import nmpc as no; no._step(...)` works.

### The offline LMPC is two modules, the live LMPC is four

| offline (`controller/lmpc/`) | live (`lmpc/`) |
|---|---|
| `build.py`: `init_parameterized_mpc()`, the parameterised CVXPY QP | `controller.py`: `MPCController`, which holds the QP build and solve |
| `solve.py`: `solve_mpc()` and the compiled-problem cache | `predict.py`, `adaptive_gains.py`, `constants.py`: prediction rollforward, gain scheduling, shared constants |

The offline adaptive-gain functions live in `controller/model_utils.py` and the linear model in `model/bicycle_model.py`. The live package duplicates both locally so the node has no simulator dependencies. Diff by function name, not by file.

The offline rollout is split into `sim/rollout/core.py` (tick loop), `reference.py`, `speed_target.py`, `delay.py`, `tick_solve.py`, plus `sim/sensor_noise.py`. It has no live counterpart to diff against. The live equivalents are spread across `mpc/control_step.py` and the `lmpc/` package.

### The MPC controller node has two output modes

`mpc_controller.py`'s `standalone_output` parameter (default `true`, also the default of the `control.launch.py` argument and of `STANDALONE_OUTPUT` in `launch_all.sh`) selects the output mode in one node file.

- **`standalone_output=false`.** Publishes an `AckermannDriveStamped` message (ackermann_msgs) (steering and target speed) on the shared `cmd_vel` interface. `fsds_bridge.py`'s speed-error P-loop computes throttle and brake and owns GO-gating and cone braking, as for Stanley. The MPC's own throttle and brake output is discarded.
- **`standalone_output=true`.** Publishes a `ControlCommand` message (fs_msgs) directly from `MPCController.compute()`'s `(steering, throttle, brake)`, which preserves the offline-tuned longitudinal behaviour. The node re-implements GO-hold, stale-path braking and cone-proximity braking itself. `control.launch.py` skips `fsds_bridge` in this mode.

The two paths share the same QP core. The file's own module docstring lists which parts branch on the mode. Do not unify them without a reason.

### Deliberately not mirrored

- **A frozen Stanley reference implementation.** None exists in either repo. The real `StanleyController` (in `control_utils.py`) and the `stanley_controller.py` node are what is mirrored. An earlier frozen copy targeted an old `/fsds/planned_path` interface, was never kept in sync, and imported a helper defined nowhere.
- **`roll_loop_to_car`** in `path_utils.py`, a closed-loop reordering helper for skidpad planning. It is kept in the root `planning/path_utils.py` although this repo has no skidpad mode and nothing there calls it, because it is parity, not dead code. In the mirror it is called by `skidpad_planner.py`.
- **The ft-fsd trace-sort planner (`build_path_trace` and helpers).** Absent from every copy. Do not re-add it from an older local copy.
- **The oracle speed-profile array** (`compute_speed_profile()` and `smooth_profile()` in `sim/speed_profile.py`). Live uses the scalar `curvature_speed()` each tick. The offline oracle path is static across a rollout, so precomputing the array once costs nothing in accuracy. The live function's dense-resample-and-denoise step exists to combat replanning jitter, which a static path never has. `SimPlanner` correspondingly emits only `.centreline`.
- **The `steering_sysid` and `steering_step` node files.** They never appeared in the `fsae_planning` git history and are absent from the working tree. See [Steering system-ID harness](#steering-system-id-harness).

## Numeric parity between `settings/` and the live parameters

### Where a live value comes from

The live node reads a parameter from the first of these that sets it.

1. A launch argument. `ros2/launch_all.sh` appends `name:=value` for each uncommented shortlist variable (`_append_mpc_arg`). `control.launch.py` and `sim.launch.py` expose every `MPCParams` and `NMPCParams` field as a launch argument, generated from the field metadata.
2. The YAML default in `common/fsae_bringup/config/fsae_params.yaml`, `controller:` block. It overrides the dataclass default at parameter-declare time.
3. The dataclass default in `mpc_params.py` or `nmpc_params.py`.

Syncing only the `.py` files and not the YAML leaves the old value running with no visible sign. `python -m tuner.tools.sync_mpc_params` handles all three files together, see [Syncing params](#syncing-live-params-to-the-other-checkouts).

The offline side has one layer: a constant in the `settings/` package, edited in the submodule that holds it (find it with `grep`). Consumers use `import settings; settings.X`, so a runtime `setattr(settings, name, value)` reaches every consumer.

### Field counts

| set | count | source |
|---|---|---|
| `MPCParams` fields | 69 | `mpc/mpc_params.py` |
| `NMPCParams` fields | 35 | `mpc/nmpc_params.py` |
| total live-tunable fields | 104 | 97 have an offline constant, 7 are live-only or hard-coded offline |
| `settings/` constants re-exported by the package | 146 | `settings/__init__.py`, across `general` (20), `noise` (10), `planner` (4), `lmpc` (26), `nmpc` (66), `solver` (10), `scoring` (10) |

Counts drift. Recount from the code rather than trusting these numbers. Every one of the 104 fields has the same default in the dataclass and in `fsae_params.yaml`. Of the 97 with an offline constant, all 97 carry the same default value as the live dataclass. One (`NMPC_V_DES_FILTER_ALPHA`) has no consumer offline.

### The launch script overrides several defaults for a real run

A run started with `ros2/launch_all.sh` differs from both defaults on these fields. The mirror copy `fsds_simulator/launch_all.sh` carries the same active lines.

| field | offline default | live dataclass and YAML | `launch_all.sh` active value | effect |
|---|---|---|---|---|
| `use_nmpc` | `USE_NMPC = False` | `False` | `USE_NMPC=true` | a live run uses the NMPC |
| `adaptive_q_scaling_enabled` | `True` | `True` | `MPC_ADAPTIVE_Q_SCALING_ENABLED=false` | LTV-QP only, and the NMPC does not apply adaptive gains |
| `reversal_penalty_enabled` | `False` | `False` | `REVERSAL_PENALTY_ENABLED=true` | LTV-QP only, NMPC has its own flag (`nmpc_reversal_penalty_enabled`, `false`) |
| `enable_dynamic_speed_cap` | `True` | `True` (YAML) | `ENABLE_DYNAMIC_SPEED_CAP=false` | the cap is off on a launch_all.sh run. The offline default keeps it on |
| `nmpc_slack_linear_weight` | `500.0` | `500.0` | `500.0` (explicit) | same value |
| `nmpc_corner_factor_k` | `27.0` | `27.0` | `27.0` (explicit) | same value |
| `nmpc_rrate_zone_enabled` and the three zone values | `True`, `2.0`, `0.8`, `0.15` | same | same (explicit) | same value |
| `nmpc_rjerk_delta` | `150.0` | `150.0` | `150.0` (explicit) | same value |
| `v_max` | `PLANNER_V_MAX = 20.0` | node default `20.0`, YAML `15.0`, launch-argument default `15.0` | `V_MAX=20.0` | `launch_all.sh` sets 20.0. A bare `sim.launch.py` without the argument runs at 15.0 |
| `TRACK` | not applicable | not applicable | `TRACK=comp_test_map_3` (pinned) | the newest-track auto-discovery is bypassed while this line is set |

Everything else in the launch shortlist is commented out, so the YAML value applies. For example `NMPC_Q_E_Y`, `NMPC_R_RATE_DELTA` and `MPC_SPEED_TARGET_DEFICIT_MAX` are commented out. With `nmpc_q_e_y = -1.0` the NMPC inherits `q_e_y = 6.4`, and with `nmpc_r_rate_delta = -1.0` it inherits `r_rate_delta = 100.0`. `speed_target_deficit_max` is `2.55` on both sides. The launch script carries a commented suggestion of `5.0`, which was measured offline (see `docs/logs/nmpc_progress_term_investigation.md`) and has not been made a default or live-validated.

### MPC cost weights: `MPCParams` and `settings/lmpc.py`

Values are identical on both sides.

| `MPCParams` field | offline constant | value |
|---|---|---|
| `q_e_y`, `q_e_yd`, `q_e_psi`, `q_r`, `q_e_v` | `Q_diag[0]` to `Q_diag[4]` | 6.4, 0.0, 1.65, 1.0, 1.5 |
| `r_delta` | `R_diag[0]` | 1.8 |
| `r_a_accel`, `r_a_brake` | `R_A_ACCEL`, `R_A_BRAKE` | 0.9, 0.6 |
| `r_rate_delta`, `r_rate_a` | `R_rate_diag[0]`, `R_rate_diag[1]` | 100.0, 2.0 |
| `terminal_q_scale` | `TERMINAL_Q_SCALE` (`settings/general.py`) | 1.0 |

`R_diag[1] = 0.77` is a nominal value kept for the array's shape. The QP reads `R_A_ACCEL` and `R_A_BRAKE`, and `R_diag[1]` only when a caller omits them. These weights are raw and not unit-normalised: each multiplies its own error term in its own units, unlike `METRIC_SCALES` in the score.

### LTV-QP adaptive gains and flags

Each field has a constant of the uppercase field name in `settings/lmpc.py` unless noted. Values are identical on both sides.

| fields | value |
|---|---|
| `adaptive_q_scaling_enabled`, `steer_rate_anti_hunt_enabled` | `True`, `True` |
| `reversal_penalty_enabled`, `reversal_penalty_boost_max`, `reversal_penalty_k` | `False`, `4.0`, `8.0` |
| `ref_heading_rate_limit_enabled` (`settings/general.py`) | `False` |
| `ref_heading_rise_rate_deg_s` (offline `REF_HEADING_RISE_RATE`, `settings/general.py`) | `90.0` |
| `speed_target_deficit_max` | `2.55` |
| `corner_factor_k` | `8.0` |
| `q_ey_straight`, `q_ey_corner` | `4.5`, `9.0` |
| `q_epsi_straight`, `q_epsi_corner` | `1.5`, `3.0` |
| `q_r_straight`, `q_r_corner` | `1.0`, `0.5` |
| `rrate_steer_straight`, `rrate_steer_corner` | `2.0`, `1.25` |
| `r_steer_corner_mid` | `1.35` |
| `low_speed_corner_boost_v_half`, `low_speed_corner_boost_max_extra` | `4.0`, `0.3` |
| `epsi_ra_half_rad`, `epsi_ra_accel_boost_max`, `epsi_ra_brake_floor` | `radians(10.0)`, `2.0`, `0.5` |

The corner-factor scheduler replaced the older lookahead and demand-normalisation family, which no longer exists on either side. See [control_mechanisms.md](control_mechanisms.md).

### Fields with no matched offline constant

| field | live default | offline handling | status |
|---|---|---|---|
| `anti_hunt_boost_max` | `6.0` | literal `boost_max = 6.0` inside `steer_rate_anti_hunt()` in `controller/model_utils.py` | one-sided by construction. The value is equal today but nothing links them |
| `nmpc_anti_hunt_boost_max` | `-1.0` (inherit) | none. `settings/nmpc.py` notes there is no override path | live-only |
| `predict_epsi_clip` | `0.5` rad | literal `_PREDICT_EPSI_CLIP = 0.5` in `sim/rollout/delay.py` | equal today, hard-coded offline |
| `delay_compensation_enabled` | `True` | offline always compensates (`predict_ahead()`) | no toggle offline |
| `max_delay_compensation_steps` | `3` | offline caps a believed lag at `n_true + 2`, not at 3 | different rule |
| `pose_age_lp_alpha`, `n_delay_hysteresis` | `0.15`, `0.25` | no offline equivalent. Offline has no measured pose age | live-only |

To tune one of the live-only knobs offline, add a `settings/` constant and a call-site keyword in the rollout first.

### NMPC fields

Every `NMPCParams` field and every NMPC-shaped `MPCParams` field has an offline constant named `NMPC_<FIELD>` (or `USE_NMPC` for `use_nmpc`) in `settings/nmpc.py`, with the same default. The live YAML and dataclass hold `-1.0` as an "inherit the LTV-QP value" sentinel on the override fields, and the offline constants use the same sentinel.

| group | fields and values |
|---|---|
| override sentinels, all `-1.0` | `nmpc_q_e_y`, `nmpc_q_e_yd`, `nmpc_q_e_psi`, `nmpc_q_epsi_dot`, `nmpc_q_e_v`, `nmpc_r_delta`, `nmpc_r_a_accel`, `nmpc_r_a_brake`, `nmpc_r_rate_delta`, `nmpc_r_rate_a`, `nmpc_terminal_scale`, `nmpc_reversal_penalty_boost_max`, `nmpc_reversal_penalty_k`, `nmpc_rrate_steer_straight`, `nmpc_rrate_steer_corner`. `nmpc_anti_hunt_boost_max` is also `-1.0` but has no offline constant |
| rate shaping | `nmpc_rrate_zone_enabled True`, boost/ease/floor `2.0`/`0.8`/`0.15`, `nmpc_rjerk_delta 150.0`, `nmpc_rjerk_a 0.0`, `nmpc_corner_factor_k 27.0`, `nmpc_rrate_stage_ramp_enabled False`, `nmpc_rrate_stage_near 0.15`, `nmpc_corner_rrate_blend_enabled False` |
| experimental flags, off | `nmpc_steer_rate_anti_hunt_enabled`, `nmpc_reversal_penalty_enabled`, `nmpc_friction_circle_enabled`, `nmpc_progress_enabled` |
| progress term | `nmpc_q_progress 4.25`, `nmpc_progress_reach 3.0`, `nmpc_progress_v_min 3.0`. The launch script's commented hints show other values (`5.0`, `2.0`, `0.5`) from an experiment that was reverted |
| solver structure | `nmpc_horizon 20`, `nmpc_sqp_iters 1`, `nmpc_solve_budget_ms 25.0`, `nmpc_osqp_max_iter 500`, `nmpc_osqp_eps 1e-4` |
| speed-gated sub-steps | `nmpc_rk_substeps 4`, `nmpc_jac_substeps 4`, `nmpc_jac_gate_speed 8.0`, `nmpc_jac_substeps_fast 2`, `nmpc_rk_gate_speed 4.0`, `nmpc_rk_substeps_fast 3` |
| standstill damping | `nmpc_standstill_steer_damp_enabled True`, `nmpc_standstill_speed 0.5`, `nmpc_standstill_fade_speed 3.0`, `nmpc_standstill_steer_r_scale 200.0` |
| trust region and slack | `nmpc_trust_delta_rad` = 9 deg, `nmpc_trust_a 0.6`, `nmpc_backtrack_max 2`, `nmpc_track_halfwidth 3.35`, `nmpc_slack_weight 10000.0`, `nmpc_slack_linear_weight 500.0` |
| reference | `nmpc_curvature_dense_step 0.5`, `nmpc_curvature_smooth_w 3`, `nmpc_kappa_clip 0.5`, `nmpc_kappa_rate_max 2.0`, `nmpc_spline_reference_enabled True`, `nmpc_alat_ceiling_enabled True` |
| latency | `nmpc_latency_compensation_enabled False`, `nmpc_latency_compensation_ms 25.0` |
| speed target filter | `nmpc_v_des_filter_alpha 0.09` |

`NMPC_V_DES_FILTER_ALPHA` is a placeholder. No offline consumer reads it, so the offline NMPC feeds the desired speed to the solver unfiltered while the live node low-passes it. This is a long-standing live-only mechanism, not a fresh gap. The tuning sweep behind the value is in `docs/logs/planner_only_lap2_corner_spinout.md`.

`nmpc_kappa_rate_max` acts only in live-planner mode. With a precomputed path the cached static reference is returned before the limiter runs, so it is inert there on both sides. `nmpc_latency_compensation_*` was live-tested and did not close the lap-2 corner margin it was built for. `k` (`nmpc_corner_factor_k`) is load-bearing for the three-zone schedule: at the inherited `8.0` the ease and floor bands are unreachable on a track with maximum curvature near 0.2, so a mismatch silently disables the schedule on one side. See [tuning.md](../guides/tuning.md).

### The lateral-acceleration ceiling law exists in three copies

The measured law is `a_lat_max(v) = max(7.5, 2.46 + 0.47 * v)` in m/s^2. The ceiling is speed-dependent in the offline plant. The flat 7.5 applies below about 10.7 m/s and the line takes over above it.

| copy | where | values |
|---|---|---|
| offline plant | `VehicleParams` in `model/vehicle_physics/params.py` (`alat_ceiling`, `alat_ceiling_slope`, `alat_ceiling_intercept`, `alat_ceiling_at()`) | `7.5`, `0.47`, `2.46` |
| offline NMPC prediction | `ALAT_CEILING_FLAT`, `ALAT_CEILING_SLOPE`, `ALAT_CEILING_INTERCEPT` in `settings/nmpc.py` | `7.5`, `0.47`, `2.46` |
| live NMPC prediction | `_Plant` in `nmpc/dynamics.py` (`alat_ceiling_flat`, `_slope`, `_intercept`) | `7.5`, `0.47`, `2.46` |

Comments in the live NMPC files refer to `MPCParams.alat_ceiling_flat`. That field does not exist. The constants live on `_Plant`. Keep all three copies together.

### Constants outside the parameter dataclasses

These pairs must stay identical. Re-confirm after any resync.

| constant | offline | live | value |
|---|---|---|---|
| `curvature_speed()` `a_lat_max` | `CURVATURE_SPEED_A_LAT_MAX` in `sim/speed_profile.py` | function default in `control_utils.py` | `4.75` |
| `curvature_speed()` scan window, curvature reduction | `sim/speed_profile.py` (`scan_end` 24 m, max of a 3-point running mean) | `control_utils.py` | same |
| `A_BRAKE_PLAN` | `sim/speed_profile.py` | `control_utils.py` | `5.0` m/s^2 |
| planner speed clamp | `PLANNER_V_MAX`, `PLANNER_V_MIN` in `sim/rollout/speed_target.py` | `v_max`, `v_min` node parameters in `mpc_controller.py` | `20.0`, `1.5` |
| steering slew limit (`du_max[0]`) | `VehicleParams.max_steer_rate * DT` (`model/vehicle_physics/params.py`, used in `sim/rollout/core.py`) | `MAX_STEER_RATE_RAD_S * dt` in `lmpc/controller.py`, literal `radians(180.0) * dt` in `nmpc/solver.py` | 180 deg/s |
| accel slew limit (`du_max[1]`) | second element of `du_max` in `sim/rollout/core.py` | second element in `lmpc/controller.py` and `nmpc/solver.py` | `0.6` per step |
| `tracking_error_speed_gate()` | `sim/speed_profile.py` | `control_utils.py` | `ey_lo` and `ey_hi` 0.5 and 2.0 m, `epsi_lo` and `epsi_hi` 20 and 60 deg, `floor` 0.3 |
| gate rate limit | `GATE_RATE_LIMIT` in `sim/rollout/speed_target.py` | `GATE_RATE_LIMIT` in `mpc/node_constants.py` and `stanley_controller.py` | `2.0` per second |
| speed-target rise limit | `SPEED_TARGET_RISE_RATE` in `sim/rollout/speed_target.py` | `SPEED_TARGET_RISE_RATE` in `mpc/node_constants.py` | `7.0` m/s^2 |
| dynamic speed cap | `ENABLE_DYNAMIC_SPEED_CAP`, `DYNAMIC_CAP_A_LAT_MAX`, `DYNAMIC_CAP_SAFETY` | `enable_dynamic_speed_cap`, `dynamic_cap_a_lat_max`, `dynamic_cap_safety` node parameters (YAML) | `True`, `3.2`, `0.9`, but the launch script turns the cap off |
| planner constants | `PLANNER_SMOOTH_PER_PT`, `PLANNER_LOOK_RADIUS`, `PLANNER_PLAN_HORIZON`, `PLANNER_PATH_BLEND` in `settings/planner.py` | `centerline_planner` block of `fsae_params.yaml` | `0.015`, `25.0`, `25.0`, `0.4` |
| perception box | `LOOK_AHEAD`, `LOOK_WIDE`, `MIN_AHEAD` in `sim/perception.py` | `sim_perception` block of `fsae_params.yaml` | `25.0`, `10.0`, `0.5` (live also has an omni `look_radius` of 25 m with no offline counterpart) |
| score constants | `settings/scoring.py` (`SCORE_WEIGHTS`, `METRIC_SCALES`, `COMPLETION_BONUS_WEIGHT`, `TIME_BONUS_WEIGHT`, `DNF_PENALTY`, `DNF_OFFTRACK_PENALTY`, `CONSTRAINT_FLOOR`, `COMPLETION_THRESHOLD`, `TIME_OBJECTIVE_WEIGHT`, `QUALITY_WEIGHT`) | `telemetry/scoring.py` (inlined) | verified equal by import at the time of writing |
| launch-speed gate | `LAUNCH_SPEED_MPS` in `sim/rollout/core.py` | `LapProgressTracker.LAUNCH_SPEED_MPS` in `telemetry/lap_progress.py` | `0.5` m/s |

`curvature_speed()` is called with explicit `v_max` and `v_min` on both sides. The offline `use_planner=True` branch passes `PLANNER_V_MAX` and `PLANNER_V_MIN`, and the live node passes `v_max` and `v_min` from its parameters. The call-site arguments must match. The function defaults (`v_max=15.0`) are not reached on either path. `compute_speed_profile()` (the offline oracle) uses `CURVATURE_SPEED_V_MAX = 16.5`, which is why the exported `speed_profile.csv` tops out at 16.5 m/s.

`a_lat_max = 4.75` in `curvature_speed()` is deliberately different from `compute_speed_profile()`'s own `mu * g` convention for its plant-limit arguments. That function has no live counterpart to stay matched to.

Offline-only or live-only items with no pair:

| item | side | note |
|---|---|---|
| `PoseFeedHold`, `POSE_HOLD_*` | offline | models a live fault, see [architecture.md](architecture.md) |
| SLAM and cone noise | offline | both off by default |
| latency columns `pose_age_s`, `path_age_s`, `n_delay`, `solve_ms`, `cmd_latency_ms` | live | offline has no equivalent |
| lap and horizon columns `lap_idx`, `pred_err_m`, `pred_acc_pct`, `lap_score`, `lap_pred_acc_pct` | live | diagnostics, not fed back into control |

### Syncing live params to the other checkouts

`python -m tuner.tools.sync_mpc_params` copies the three live parameter files (`mpc/mpc_params.py`, `mpc/nmpc_params.py`, `common/fsae_bringup/config/fsae_params.yaml`) from `ros2/src/fsae_planning` into the `fsae_autonomous` working tree and into `fsds_simulator/`. It is one-way and never reads the destinations as a source.

```bash
cd fsae_MPCTest
python -m tuner.tools.sync_mpc_params            # dry run, prints a diff per file per destination
python -m tuner.tools.sync_mpc_params --apply     # overwrite, keeping a one-time .bak per destination
```

It never touches `settings/` (the offline side of a different boundary), the launch script, or non-parameter source such as the controller modules and `live_viz`. Those follow the ordinary manual propagation in [Resync procedure](#resync-procedure). It only writes into the local `fsae_autonomous` working tree and never commits. It is also reachable from the launcher GUI's Settings tab ("Overwrite All Params...", dry run first). See [debugging_tools.md](../guides/debugging_tools.md).

## Live/offline score parity

`telemetry/scoring.py` is a verbatim copy of `sim/scoring.py`. `compute_composite_score()`, `RolloutMetrics.add_step()` and `RolloutMetrics.finalize()` are identical, so a score from the car is directly comparable to one from `tuner/offline_tuner.py`. The one intended difference is the settings import. `sim/scoring.py` reads its constants from `settings`, which is not on the car's `PYTHONPATH`, so the live copy inlines them as module constants. The constants are listed in the table above and were verified equal by importing both files.

`METRIC_SCALES` divides each metric by a reference magnitude before weighting: `score = SCORE_WEIGHTS @ (metrics / METRIC_SCALES)`. It is inlined in the live `telemetry/scoring.py` and in the mirror copy, plus `settings/scoring.py`, so a change is a three-file edit, like `SCORE_WEIGHTS`. Why it exists is in [architecture.md](architecture.md).

### The live caller must supply real progress and completion

`progress`, `reached_end` and `time_bonus` have to be passed to `close()` explicitly. If a controller node called `telemetry.close()` with no arguments, `progress` would default to `0.0` and `reached_end` to `None`, `compute_composite_score()` would read that as "never finished", and every live run would score exactly `CONSTRAINT_FLOOR + DNF_PENALTY = 13.0` regardless of how the car drove. The 13 underlying metrics stay correct, and only the composite goes dead. See the "Live scorer reports 13.0" row in the findings table of `docs/logs/sim_to_real_investigation.md`.

`LapProgressTracker` (`telemetry/lap_progress.py`) supplies them.

- It tracks the car's forward-bounded nearest-index position against the precomputed track path (the CSV already loaded for the speed lookup) to get `progress` and `reached_end`.
- It integrates `ds / v_target` over that speed profile for an optimal-time bound. It does not call `speed_profile.optimal_lap_time()`, because that solver is in this repo and not on the node's path.
- `time_bonus = optimal_time * progress / actual_lap_time`, clipped to `[0, 1]`, the same convention as `sim/rollout/`.

This works only with a precomputed speed profile loaded (`map_path` set, the normal setup). A run against the live planner topic has no path end to measure against, so `progress` and `reached_end` fall back to their defaults. The CSV header then records `score_is_partial=1`, and the run's header also records `lap_time_s` and `optimal_time_s`.

`offtrack` has no live equivalent, because the offline rollout knows ground-truth track edges and the car does not. It defaults to `False`. The weighted-metric part of the score is comparable either way. Only the bonus and penalty terms differ.

## Lap timing and live-only diagnostics

### Lap timing starts at 0.5 m/s, not at the first tick

**Plain version.** The clock starts when the car is moving, not when the software starts. Timing from tick 0 would include the roughly one second the car sits still before launch, and offline and live numbers would measure different things.

`LAUNCH_SPEED_MPS = 0.5` is the threshold on both sides: `LapProgressTracker.LAUNCH_SPEED_MPS` live, and `LAUNCH_SPEED_MPS` with `launch_step` in `sim/rollout/core.py` offline.

- **Live.** `LapProgressTracker.update()` takes `car_speed` and defers the start of timing until `abs(car_speed) >= LAUNCH_SPEED_MPS`. Passing `None` falls back to timing from tick 0.
- **Offline.** `sim_time = (n_ran - launch_step) * DT`, where `launch_step` is the first step above the threshold.

A lap timed from tick 0 is about 0.95 s slower than the same drive timed from launch speed, so the two are not comparable. Excluding the standstill also stops it consuming step budget that would cause a spurious DNF in `nmpc_offline_check`. Both output modes must pass `car_speed`: `debug_publish.py` calls `self._lap_tracker.update(self._car_pos, t, self._car_speed)` for the MPC node in either mode, and `stanley_controller.py` does the same.

### Multi-lap scoring and prediction-horizon accuracy are live-only

**Plain version.** The live car can drive several laps in one run and gets a separate score for each, shown live in the `live_viz` lap panel and afterwards in `plot_playback.py`. A second number, prediction-horizon accuracy, says how well the controller's own look-ahead predicted where the car went. Both are diagnostics. Neither feeds control and neither exists offline.

**Multi-lap.** `LapProgressTracker` re-arms after each finish: the index search restarts, the lap timer restarts (a flying lap), and `update()` returns a completed-lap dict the moment a lap finishes. `ControlLogger.finish_lap()` (`telemetry/control_logger.py`) finalises that lap's own `RolloutMetrics` accumulator into a per-lap `composite_score`, then resets it. The whole-run score in the header still comes from the run's own accumulator, which is never reset. For a single-lap run the two are close but not identical.

The re-arm guard is real distance travelled since the last finish, not the search index. An index-based guard cannot tell a car still sitting on the finish line from one that has lapped again, because the index can advance with no car motion or coincide with a closed loop's own start point.

**Prediction-horizon accuracy** compares the NMPC's predicted horizon (front-axle points about 1 s ahead, published for `live_viz` and built with `PathReference.xy_at()`) with where the car was once the prediction had time to come true. Per prediction: `100% * (1 - mean_error / horizon_length)`, clipped to `[0, 100]`, where `mean_error` is the mean distance between each predicted point and the car's time-interpolated position. It is NMPC-only, because the LTV-QP never exposes a Cartesian horizon. LTV-QP runs report n/a. Implementation: `HorizonAccuracyTracker` in `telemetry/horizon_tracker.py`.

**No offline equivalent, by design.**

- `sim/rollout/tick_solve.py`'s `record_horizon_prediction()` builds a GUI-only, cosmetic line for the LTV-QP and records empty arrays for the NMPC, so there is nothing to compare for the one controller the metric targets.
- The offline tuner has no multi-lap concept. `recorded_map_rollout` and `offline_tuner` each score one traversal.

To validate the metric offline, the smallest correct change is to record an NMPC horizon in the rollout and port `HorizonAccuracyTracker` unchanged, not to reimplement the formula.

## Steering system-ID harness

**Status: the ROS 2 nodes are missing.** The harness scripts `ros2/run_steering_sysid.sh` and `ros2/run_steering_step.sh` run `ros2 run fsae_control steering_sysid` and `steering_step`. Neither node file exists in the working tree, in the mirror, in `setup.py`'s console scripts, or in any `fsae_planning` commit. Only the offline analysers survive. The scripts also call the analysers by an old module path (`tuner.steering_sysid_analysis`), which no longer exists. The current paths are `tuner.investigations.steering_sysid_analysis` and `tuner.investigations.steering_step_analysis`. Until the nodes are restored, the harness cannot run end to end. The design below is what the scripts and analysers were written for.

**What it does.** Commanding fixed steering angles at fixed speeds on an empty map, with the MPC bypassed, and recording the achieved yaw rate isolates the plant from the controller. This is how FSDS's lateral-acceleration ceiling was found (see [simulator_fidelity.md](simulator_fidelity.md)). A closed-loop lap log alone cannot separate a plant defect from a controller or reference defect, so this method is the one to reuse whenever a plant-versus-car discrepancy is suspected.

| piece | location | role |
|---|---|---|
| `ros2/run_steering_sysid.sh` | outer repo, beside `launch_all.sh` | one-command speed sweep |
| `ros2/run_steering_step.sh` | same | one-command step-input test (50 Hz, isolates the transient) |
| `tuner/investigations/steering_sysid_analysis.py` | this repo | reads the sweep log, names the mechanism |
| `tuner/investigations/steering_step_analysis.py` | this repo | reads the step log, names the mechanism |
| `steering_sysid` and `steering_step` nodes | live workspace, not present | drive FSDS directly and write the log |

The nodes, the scripts and their logs are not mirrored into `fsds_simulator/`. The analysers are offline-only.

**Running the sweep.** `cd <FSDS repo>/ros2 && ./run_steering_sysid.sh` starts FSDS, waits for the RPC port, starts the bridge, waits for `/fsds/testing_only/odom`, runs the sweep, analyses the newest log and tears everything down, including on Ctrl+C. Flags: `--no-sim` (FSDS already running), `--quick` (fewer points), and any `-p name:=value` passed to the node. Logs go to `$HOME/fsae_logs/steering_sysid_*.csv`. `ros2/run_steering_step.sh` takes the same flags.

- **Empty map only.** The car circles at up to 14 m/s and does not brake for cones.
- **Nothing else may publish control.** The script refuses to start if `mpc_controller`, `fsds_bridge` or `stanley` is running, because a second publisher on `/fsds/control_command` interleaves commands and corrupts the log. It is deliberately separate from `launch_all.sh`.
- **Measured property, not a tuning knob.** If the result is suspected stale, re-measure.

**Reading the sweep log.** The analyser reports, per (speed, steering) point, the achieved-to-commanded steering ratio `s = delta_achieved / delta_commanded`, with `delta_achieved = atan(L * yaw_rate / v)` and `L = 1.55 m`. A falling `s` is not diagnostic alone, since a speed-scaled steering rack, real understeer and grip saturation all produce one. The analyser fits five candidate models to the achieved yaw rate and ranks them by R^2.

| winning model | meaning |
|---|---|
| neutral (`s = 1`) | the steering path is fine, so look at the controller or reference |
| constant scale | assumed maximum steering is wrong. Fix it in `fsds_bridge`, the LTV-QP controller and `control_utils` together |
| speed-scaled rack | FSDS reduces lock with speed. Model it in the plant, do not bend tyre parameters |
| understeer (`v^2`) | vehicle dynamics. The offline plant cannot reach the measured understeer gradient with physical tyre parameters |
| grip saturation | yaw is capped at a lateral-acceleration ceiling. A fitted ceiling far below the car's demonstrated grip (12 m/s^2) means it is not tyre saturation |

The analyser warns when the runner-up is within 0.05 R^2, which means the log does not separate them. Widen the speed range and re-run rather than trusting the verdict. It also refuses a verdict when the car did not move in the recorded windows, because all-zero data otherwise gives a confident, meaningless answer. The sweep's default speed range is 3 to 14 m/s.

**Reading the step log.** The step analyser decides which of three mechanisms caps yaw rate from the transient shape. Overshoot above 10% (peak over final) is the signature of active damping and rules out the other two.

| mechanism | transient |
|---|---|
| hard yaw-rate limit | yaw rises and clips at a maximum, no overshoot, the approach flattens abruptly |
| speed-scaled authority | yaw rises smoothly to a lower plateau, first-order, no overshoot |
| active damping | yaw overshoots the settled value then decays |

The transient shape found FSDS's ceiling to be a lagged restoring term, which is why `alat_ceiling*` is modelled with a first-order lag. Other checks are peak against the sweep's fitted ceiling, rise shape, and settling.

The measured results from these tests, and the model fitted to them, are in [simulator_fidelity.md](simulator_fidelity.md). To confirm the offline plant still reproduces them, run `python -m tuner.validation.plant_openloop_validation`.

## Resync procedure

1. Update the sibling `fsae_planning` checkout used as the source (`ros2/src/fsae_planning/`).
2. For each file in the mapping tables, read both the old (already ported) version and the new version in full before porting. Do not diff-and-patch blind. The one mechanical difference for the root `planning/` folder is that live files import each other as `from fsae_planning.xxx import yyy`, and this repo's package is `planning`, so every intra-package import is rewritten. Everything under `fsds_simulator/` needs no rewrite because it is a literal copy at the matching path.
3. Port algorithm changes only. Keep the root `planning/` import style and the deliberate non-mirrors above (do not add a `.v_profile` to `SimPlanner`). `fsds_simulator/` mirrors both `standalone_output` modes and `stanley_controller.py`.
4. If the change touches `planning/` or the live controller modules, check whether `sim/rollout/` needs a matching change. The offline rollout and the live controller are two implementations of one control loop. State in the resync notes whether a mirrored change was or was not needed.
5. Re-check the [constants table](#constants-outside-the-parameter-dataclasses) and the parameter tables. If `curvature_speed()`'s `a_lat_max` or the planner speed clamps changed, update `sim/speed_profile.py`, `sim/rollout/speed_target.py`, `control_utils.py` and `mpc/mpc_controller.py` together. For parameter values use `sync_mpc_params`.
6. Run the smoke test from [offline_guide.md](../guides/offline_guide.md): confirm changed files import cleanly, then run `python -m gui.simulation` or a short `python -m tuner.offline_tuner` with `FAST_TEST_MODE = True` in `settings/solver.py`, against one synthetic path, and check the rollout still converges. The mirror's ROS 2 files cannot be tested against FSDS or the car from this repo. Reason through the change against `sim/rollout/` and flag it for live testing.
