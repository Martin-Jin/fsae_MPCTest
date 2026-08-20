# Planning/Control Upstream Sync

Reference for re-syncing this repo's `planning/` package and `fsds_simulator/`
staging files against a newer clone of the upstream
[`fsae_planning`](https://github.com/UOA-FSAE/fsae_planning) repo. Read this
before touching either directory during a resync — it records which files
mirror which upstream files, which pieces are deliberately *not* mirrored (and
why), and the numeric-parity constants that must stay matched across the
offline/live boundary.

**`fsds_simulator/` is a staging area, not a live module.** It mirrors
`fsae_planning`'s own ROS 2 workspace hierarchy exactly — every package
(`common/fsae_interfaces`, `common/fsae_bringup`, `perception/
fsae_sim_perception`, `planning/fsae_planning`, `control/fsae_control`), not
just the control-layer files — so the whole thing can be copied straight
across into a workspace `src/` at the same relative paths with no manual
re-pathing and no missing scaffolding (`package.xml`/`setup.py`/`setup.cfg`/
`resource/` included). See `fsds_simulator/README.md` for the build/run
instructions this enables. Nothing under `fsds_simulator/` is imported by
`gui/simulation.py`, `tuner/offline_tuner.py`, or anything else in this repo —
those all live under `planning/`, `sim/`, `model/`, `controller/` instead.
`fsds_simulator/` exists purely so this repo can hold, version, and hand off a
ready-to-build copy of the ROS 2 side — including to someone who has only
this repo and FSDS, with no separate `fsae_planning` checkout at all.

**Current mirror scope.** `fsds_simulator/` covers the full workspace, not
just the control layer: `common/fsae_interfaces` (message package),
`common/fsae_bringup` (`fsae_params.yaml`, `perception.launch.py`,
`planning.launch.py`, `sim.launch.py`, full package scaffolding), all of
`planning/fsae_planning` as a real package (the root-level `planning/` folder
separately holds an algorithm-only mirror — see the file mapping below),
`perception/fsae_sim_perception`, and `control/fsae_control`'s full node set
(`fsds_bridge.py`, `stanley_controller.py`, `mpc_controller.py`,
`mpc_controller_standalone.py`, `telemetry_logger.py`).

Notes on how the current state differs from what an older checkout may show:

- There is no frozen `fsds_simulator/stanley_controller/` reference
  implementation any more (`stanley_control.py` /
  `stanely_control_utils.py`). It targeted an old `/fsds/planned_path`+`Track`
  interface and imported a never-defined `separate_cones_by_color`; the real,
  current `stanley_controller.py` (which uses `control_utils.py`'s
  `StanleyController`) is mirrored instead.
- The root `planning/` mirror tracks the `fsae_planning` checkout in
  `ros2/src/fsae_planning/`. It carries no `build_path_trace` ft-fsd
  trace-sort planner (upstream removed it — see "Deliberately not mirrored"
  below), exposes `build_wall_segments`/`segment_crosses_walls` as public,
  and includes `roll_loop_to_car` (a skidpad-support helper — see its own
  note below).
- `fsds_simulator/` uses the mirrored package hierarchy described above, not
  a flat folder. It has no `control_node.py`; upstream's
  `mpc_controller_standalone.py` fills that role (see "What replaced
  `control_node.py`" below — a same-design, different-file swap, not a rename).
- `steering_sysid.py`/`steering_step.py` and their harness scripts are **not**
  mirrored, and have no upstream counterpart to track: they never existed in
  `fsae_planning`'s committed git history, and upstream discarded them from
  its own working tree before raising a PR (see
  `fsae_MPCTest/docs/fsae_planning_pending_pr.md`'s "Discarded, not part of
  this PR" note). `tuner/checks/steering_sysid_analysis.py` /
  `tuner/checks/steering_step_analysis.py` are `fsae_MPCTest`-only and have
  nothing to mirror either way.

## File mapping

| This repo | Upstream (`fsae_planning`) | Notes |
|---|---|---|
| `planning/boundary.py` | `planning/fsae_planning/fsae_planning/boundary.py` | Direct mirror |
| `planning/cone_map.py` | `planning/fsae_planning/fsae_planning/cone_map.py` | Direct mirror |
| `planning/cone_sorting.py` | `planning/fsae_planning/fsae_planning/cone_sorting.py` | Direct mirror |
| `planning/path_utils.py` | `planning/fsae_planning/fsae_planning/path_utils.py` | Direct mirror |
| — (no file counterpart) | `planning/fsae_planning/fsae_planning/centerline_planner.py` | The ROS 2 planner node itself. Its behaviour (temporal centreline blending via `blend_paths()`, called every planning tick) is reproduced inline by `sim/sim_track.py`'s `SimPlanner.update()` rather than as a ported file — the simulator has no separate planner *node*, `SimPlanner` plays that role directly. This is `planning/`'s own algorithm-only mirror; the full node **is** mirrored under `fsds_simulator/` (see below) since that folder mirrors the whole workspace, not just simulator-relevant algorithm code. |

`planning/` (this repo's root-level folder, used by `gui/simulation.py` /
`tuner/offline_tuner.py`) and `fsds_simulator/` (the ROS 2 staging mirror,
used by nobody in this repo — see above) both track upstream, but serve
different purposes and are mapped separately. `fsds_simulator/` mirrors
upstream's **entire** workspace — every package, including scaffolding
(`package.xml`, `setup.py`, `setup.cfg`, `resource/`) — not just the files an
offline tuner cares about:

| `fsds_simulator/` path | Upstream (`fsae_planning`) | Notes |
|---|---|---|
| `common/fsae_interfaces/` (whole package) | `common/fsae_interfaces/` | Direct mirror. Message definitions (`Track`, `ConeDetection`, `CAN`, …) every other package depends on. |
| `common/fsae_bringup/` (whole package) | `common/fsae_bringup/` | Direct mirror. `config/fsae_params.yaml` (central tunables), all 5 launch files (`sim.launch.py`, `control.launch.py`, `planning.launch.py`, `perception.launch.py`, `cone_recorder.launch.py`). |
| `perception/fsae_sim_perception/` (whole package) | `perception/fsae_sim_perception/` | `sim_perception.py` is a direct mirror (FSDS oracle+odom → `/fsae/*`). `cone_recorder.py` and `common/fsae_bringup/launch/cone_recorder.launch.py` have **never existed in upstream `fsae_planning`'s git history** — despite the "Direct mirror" label this table used to give the whole package, these two files are this repo's own addition, staged here to be upstreamed later, not copied from an existing upstream file. |
| `planning/fsae_planning/` (whole package) | `planning/fsae_planning/` | Direct mirror. `centerline_planner.py` (the actual ROS 2 node — see the row above for why the root `planning/` folder doesn't have this), `boundary.py`, `cone_map.py`, `cone_sorting.py`, `path_utils.py`, `special_utils/` (skidpad). |
| `control/fsae_control/fsae_control/mpc_core.py` | `control/fsae_control/fsae_control/mpc_core.py` | Direct mirror. The `MPCController` class — QP-based MPC, kept in byte-for-byte parity with `sim/rollout_core.py`/`controller/optimiser.py` per this repo's own numeric-parity rule (see `CLAUDE.md`). |
| `control/fsae_control/fsae_control/control_utils.py` | `control/fsae_control/fsae_control/control_utils.py` | Direct mirror — includes both `curvature_speed()` and `StanleyController`. |
| `control/fsae_control/fsae_control/stanley_controller.py` | `control/fsae_control/fsae_control/stanley_controller.py` | Direct mirror. The actual current `controller:=stanley` node (publishes `cmd_vel`, routes through `fsds_bridge`). Replaces the old frozen reference implementation — see "Current mirror scope" above. |
| `control/fsae_control/fsae_control/mpc_controller.py` | `control/fsae_control/fsae_control/mpc_controller.py` | Direct mirror. Upstream's default `controller:=mpc` node — steering only through `cmd_vel`/`fsds_bridge`, discards the MPC's own throttle/brake. See "Two MPC-controller nodes" below. |
| `control/fsae_control/fsae_control/mpc_controller_standalone.py` | *(no upstream counterpart — never existed in `fsae_planning`'s git history)* | **Not a direct mirror** despite this table's general framing — this file was authored in this repo (ported from the retired `fsds_simulator/control_node.py`) and is staged here for eventual upstreaming, the reverse direction from every other row. See "What replaced `control_node.py`" for the history. |
| `control/fsae_control/fsae_control/fsds_bridge.py` | `control/fsae_control/fsae_control/fsds_bridge.py` | Direct mirror. Needed by both `stanley_controller.py` and `mpc_controller.py` (not `mpc_controller_standalone.py`, which bypasses it). |
| `control/fsae_control/fsae_control/telemetry_logger.py` | `control/fsae_control/fsae_control/telemetry_logger.py` | Direct mirror. CSV telemetry shared by all three controller nodes. Also computes the run's composite score (via `scoring.py`) and prepends it to the control CSV as a `#`-commented header on `close()`. Includes `LapProgressTracker`, which computes real `progress`/`reached_end`/`time_bonus` from the precomputed track path; see "Live/offline score parity" below. |
| `control/fsae_control/fsae_control/scoring.py` | *(no upstream counterpart — never existed in `fsae_planning`'s git history)* | **Not a direct mirror.** Staged here for upstreaming, same direction as `mpc_controller_standalone.py` above. It **is** a verbatim copy of this repo's own `sim/scoring.py` — see "Live/offline score parity" below. Changes must be made in `sim/scoring.py` first, then re-copied here (and eventually upstreamed). |
| `control/fsae_control/setup.py` | `control/fsae_control/setup.py` | Direct mirror **except** the `mpc_controller_standalone`/`scoring.py` entry points/imports this repo's own two staged-for-upstream files above need — those exist here but not upstream. Registers four console-script entry points (`controller`, `mpc_controller`, `mpc_controller_standalone`, `fsds_bridge`). |

> **`zip_safe=False` is required, on both sides.** All four of this mirror's
> `setup.py` files (`common/fsae_bringup`, `control/fsae_control`, `perception/
> fsae_sim_perception`, `planning/fsae_planning`) set `zip_safe=False` — the
> root-cause fix for a stale-`colcon-build` bug (§49 in
> `docs/logs/sim_to_real_investigation.md`). The four live copies set it too,
> verified byte-identical on this setting. Keep it on any new package added to
> either side.

`planning/` (root) and the `MPCController` half of `mpc_core.py` are shared
algorithm code and should track upstream closely. `mpc_controller_standalone.py`
is this repo's own integration pattern, ported upstream and then kept in the
mirror going forward — it's expected to diverge in upstream-specific ways
(topic names, message types) while keeping the same behavioural design.

### Two MPC-controller nodes, plus Stanley

### What replaced `control_node.py`

This repo used to have its own `fsds_simulator/control_node.py`, targeting an
older ROS 2 topic/message interface (`/fsds/planned_path` as `nav_msgs/Path`,
`/FusionCones` as `fs_msgs/Track`) that no longer exists in current
`fsae_planning`. It has been retired and its design ported into upstream's
package as a new file, **`mpc_controller_standalone.py`**, updated for the
current topics (`/fsae/planning/selected_trajectory`, `/fsae/slam/
car_position`, `/fsae/perception/cone_detection`). The mirror in this repo
now holds that upstream file directly — there is no separate
`control_node.py` anymore, in either repo.

This matters because `fsae_planning` actually has **two** MPC-controller node
files, and they are not interchangeable:

- **`mpc_controller.py`** — upstream's original `controller:=mpc` node.
  Publishes `ackermann_msgs/AckermannDriveStamped` (steering + target speed)
  on the shared `cmd_vel` interface and lets `fsds_bridge.py`'s simple
  speed-error P-loop compute throttle/brake, and own GO-gating/cone-braking,
  identically to the Stanley controller. It **discards** the MPC's own
  throttle/brake output. Unchanged, still exists, still the default.
- **`mpc_controller_standalone.py`** — the new node this repo's old
  `control_node.py` was ported into. Publishes `fs_msgs/ControlCommand`
  directly, using `MPCController.compute()`'s `(steering, throttle, brake)`
  output unchanged (preserving the offline-tuned longitudinal behaviour this
  repo's tuner produces), and re-implements GO-hold/stale-path-brake/
  cone-proximity-brake itself instead of relying on `fsds_bridge.py`.
  Selected via `controller:=mpc_standalone` in `control.launch.py`, which
  skips `fsds_bridge` for that mode.

If you're resyncing this repo's mirror against a newer `fsae_planning`,
diff `mpc_controller_standalone.py` against `mpc_controller_standalone.py` —
**not** against `mpc_controller.py`. `mpc_controller.py`'s accel-discarding
design has no counterpart in this repo at all; don't try to reconcile the
two, they're deliberately different controllers reusing the same
`MPCController` QP core.

## Deliberately not mirrored

- **The old frozen Stanley reference** (previously at
  `fsds_simulator/stanley_controller/stanely_control_utils.py` +
  `stanley_control.py`) — **removed**. It targeted an old
  `/fsds/planned_path`+`Track` interface, was not kept in sync with upstream,
  and imported a `separate_cones_by_color` helper that isn't defined anywhere
  in either repo. The real, current `StanleyController` (in
  `control_utils.py`) and `stanley_controller.py` node are now mirrored
  instead, kept in sync like everything else under `fsds_simulator/`. If you
  find an older local copy of this repo with the frozen reference still
  present, that's the *previous* state — don't resurrect it.
- **`roll_loop_to_car`** (added to upstream's `path_utils.py`) — a
  closed-loop-reordering helper upstream's skidpad planner uses to follow a
  known figure-8. Ported here for parity (see "Last resynced" above). In the
  root-level `planning/path_utils.py` copy (used by `gui/simulation.py` /
  `tuner/offline_tuner.py`), **this repo has no skidpad mode**, so nothing
  calls it there yet — keep it on resync rather than stripping it as dead
  code, it's parity, not scope creep. The `fsds_simulator/` mirror copy is
  different: it **is** called, by
  `fsds_simulator/planning/fsae_planning/fsae_planning/special_utils/skidpad_planner.py`,
  since that mirror carries the whole upstream workspace including its
  skidpad planner — don't treat that copy as unreferenced.
- **The ft-fsd trace-sort planner (`build_path_trace` and its private
  helpers)** — upstream removed this entirely (see "Last resynced" above);
  it is gone from this repo's `boundary.py` too as of the same resync. If an
  older local copy of this repo still has it, that's the *previous* state,
  not something to re-add.
- **The offline oracle speed-profile array** (`sim/speed_profile.py`'s
  `compute_speed_profile()` / `smooth_profile()`) — used only for the
  synthetic/oracle path in `tuner/offline_tuner.py` and `gui/simulation.py`'s
  exact-path mode. Upstream replaced the equivalent live-path logic with a
  scalar `curvature_speed()` call, but this repo deliberately keeps the
  precomputed-array version for the oracle path: it's static across a whole
  rollout, so precomputing it once is a performance win with no accuracy
  cost. The live-path `curvature_speed()`'s dense-resample-and-denoise step
  exists specifically to combat frame-to-frame replanning jitter — a static
  oracle path never has that problem, so there is no equivalent upstream
  logic to port back for the oracle branch. `sim/sim_track.py`'s `SimPlanner`
  correspondingly emits only `.centreline` (no `.v_profile`), matching
  upstream's planner-emits-path-only design; the live-planner branch derives
  its speed on demand via `speed_profile.curvature_speed()` instead.

## Numeric-parity constants

These pairs must stay numerically identical across the offline/live
boundary, or offline-tuned weights will not transfer faithfully to the live
controller. Line numbers below were confirmed by grep at the time of
writing — re-confirm before relying on them, since a resync can move them.

| Constant | Offline copy | Live copy | Current value |
|---|---|---|---|
| `curvature_speed()`'s `a_lat_max` | `sim/speed_profile.py:499` (function default) | `fsds_simulator/control/fsae_control/fsae_control/control_utils.py:194` (function default) | `4.0` |
| Planner top/bottom speed clamp | `sim/rollout_core.py:67-68` (`PLANNER_V_MAX`, `PLANNER_V_MIN`) | `fsds_simulator/control/fsae_control/fsae_control/mpc_controller_standalone.py` (declared as the `v_max`/`v_min` ROS parameters, default `20.0`/`1.5`) | `20.0` / `1.5` |
| Steering slew-rate limit (`du_max[0]`) | `model/vehicle_physics.py` (`VehicleParams.max_steer_rate`), applied as `max_steer_rate * DT` in `sim/rollout_core.py` and passed to `controller/optimiser.py`'s `du_max` | `fsds_simulator/control/fsae_control/fsae_control/mpc_core.py` (`MAX_STEER_RATE_RAD_S`, applied as `* self.dt`) | `radians(180.0)` rad/s |
| Accel slew-rate limit (`du_max[1]`) | `sim/rollout_core.py` (`du_max` second element) | `fsds_simulator/control/fsae_control/fsae_control/mpc_core.py` (`self.du_max` second element) | `0.6` per step |
| `tracking_error_speed_gate()` thresholds | `sim/speed_profile.py` | `fsds_simulator/.../control_utils.py` | `ey_lo/hi` 0.5/2.0 m, `epsi_lo/hi` 20/60 deg, `floor` 0.3 |
| Speed-target rise limit | `sim/rollout_core.py` (`SPEED_TARGET_RISE_RATE`) | `fsds_simulator/.../mpc_controller.py` and `mpc_controller_standalone.py` (same name) | `2.0` m/s² |
| `curvature_speed()` κ reduction | `sim/speed_profile.py` | `fsds_simulator/.../control_utils.py` | max of 3-point running mean |
| Score weights / bonuses / penalties | `settings.py` (`SCORE_WEIGHTS`, `COMPLETION_BONUS_WEIGHT`, `TIME_BONUS_WEIGHT`, `DNF_PENALTY`, `DNF_OFFTRACK_PENALTY`) | `fsds_simulator/control/fsae_control/fsae_control/scoring.py` (inlined as module constants) | weights sum to `1.0`; `0.5` / `0.25` / `3.0` / `3.0` |
| Metric normalisation scales | `settings.py` (`METRIC_SCALES`) | `fsds_simulator/control/fsae_control/fsae_control/scoring.py` (inlined as module constant) | 13 entries, `[0.40, 0.45, 0.30, 0.18, 1.50, 0.40, 0.02, 0.30, 1.00, 0.015, 0.70, 2.30, 0.08]` |
| Constrained-scoring constants | `settings.py` (`CONSTRAINT_FLOOR`, `COMPLETION_THRESHOLD`, `TIME_OBJECTIVE_WEIGHT`, `QUALITY_WEIGHT`) | `fsds_simulator/.../scoring.py` (inlined as module constants) | `10.0` / `0.98` / `1.0` / `0.35` |
| `A_BRAKE_PLAN` (braking-distance propagation in `curvature_speed`) | `sim/speed_profile.py` | `fsds_simulator/.../control_utils.py` | `5.0` m/s², positive magnitude |
| Dynamic speed cap enable/gains | `settings.py` (`ENABLE_DYNAMIC_SPEED_CAP`, `DYNAMIC_CAP_A_LAT_MAX`, `DYNAMIC_CAP_SAFETY`) | `mpc_controller.py`/`mpc_controller_standalone.py` (`enable_dynamic_speed_cap`/`dynamic_cap_a_lat_max`/`dynamic_cap_safety` ROS params) | `True` / `3.2` m/s² / `0.9` — see "Dynamic speed cap" section below |
| Latency telemetry columns | — (offline has no equivalent) | `fsds_simulator/.../telemetry_logger.py` | `pose_age_s`, `path_age_s`, `n_delay`, `solve_ms`, `cmd_latency_ms` |
| Pose-feed hold model | `settings.py` (`POSE_HOLD_*`) + `sim/rollout_core.PoseFeedHold` | — (offline-only; models a live fault) | `PROB 0.05`, `MEAN_TICKS 2.1`, `MAX_TICKS 5` |
| Accel/brake effort split (`R[1,1]`) | `settings.py` (`R_A_ACCEL`, `R_A_BRAKE`), read by `controller/optimiser.py`'s `solve_mpc(r_a_accel=, r_a_brake=)` | `mpc_params.py` (`r_a_accel`, `r_a_brake`), read by `mpc_core.py`'s `_solve_qp` | actively being live-tuned — re-check both sides' current values before trusting this row; see "Accel/brake effort weight split" below |
| Corner-factor scheduler + heading-error accel/brake asymmetry | `settings.py` (`CORNER_FACTOR_K`, `Q_EY_*`/`Q_EPSI_*`/`Q_R_*`/`RRATE_STEER_*`/`R_STEER_CORNER_MID`, `LOW_SPEED_CORNER_BOOST_*`, `EPSI_RA_*`) | `mpc_params.py` (same names, lowercase) | see the "MPC weight/gain parity" table above for the full current field list. The lookahead gain-scheduling family this row replaces (exit-boost decay distance/peak-tracker, low-speed steering-rate boost, steering-effort relaxation, curvature forcing, anti-hunt lookahead gate, exit-boost `\|e_psi\|` hold threshold) no longer exists on either side — see "Corner-factor scheduler" below |

Notes on how these are actually used:

- `sim/speed_profile.py`'s `curvature_speed()` (the offline mirror of
  `control_utils.py`'s `curvature_speed()`) is called by `sim/rollout_core.py`'s
  `use_planner=True` branch at `sim/rollout_core.py:716`, which passes
  `v_max=PLANNER_V_MAX, v_min=PLANNER_V_MIN` explicitly — overriding that
  function's own `v_max=15.0, v_min=1.5` defaults. The live side calls the
  same function the same way: `mpc_controller_standalone.py`'s
  `_control_loop` passes `v_max=self._v_max, v_min=self._v_min` (also
  overriding `control_utils.py`'s function defaults) — sourced from the
  `v_max`/`v_min` ROS parameters, which default to the same `20.0`/`1.5`. So
  it's the **call-site arguments** (`PLANNER_V_MAX`/`PLANNER_V_MIN` vs. the
  `v_max`/`v_min` ROS params), not the functions' own default parameter
  values, that must be kept matched — the function defaults themselves are
  never hit in either the offline or live path.
- `a_lat_max=4.0` is the value actually used (as each function's default,
  not overridden at either call site) and must stay identical between
  `sim/speed_profile.py`'s and `control_utils.py`'s `curvature_speed()` — see
  the explicit callout in `sim/speed_profile.py`'s `curvature_speed()`
  docstring, which also notes this is deliberately *different* from
  `compute_speed_profile()`'s own `a_lat_max = mu * g` (≈5.886) convention,
  since that function has no live counterpart to stay matched to.

If a resync changes any of these values or call sites, update both sides in
the same change and re-grep this table's line numbers.

## MPC weight/gain parity: `MPCParams` ↔ `settings.py`

Every MPC cost weight, adaptive-gain shape constant, and feature flag is
centralized one place per side (see the outer repo's `CLAUDE.md`, "Single
source of truth for MPC tuning, per side"):

- **Live**: `ros2/src/fsae_planning/control/fsae_control/fsae_control/mpc_params.py`'s
  `MPCParams` dataclass (also exposed as ROS2 launch parameters — see that
  repo's README).
- **Offline**: `settings.py`, imported by `sim/rollout_core.py` and threaded
  into `controller/model_utils.py`'s adaptive-gain functions as explicit
  keyword arguments.

This table is the field-by-field mapping. Every field below is confirmed
present on both `MPCParams` and `settings.py`; re-confirm after any resync,
since a parity obligation stated only in prose comments is easy to miss.
See "Corner-factor scheduler" below for what replaced the ~35-field
lookahead/demand-normalisation/U-turn/straight-line family — those fields no
longer exist on either side.

| `MPCParams` field | `settings.py` constant | Current value |
|---|---|---|
| `q_e_y` | `Q_diag[0]` | `6.35` live / `6.0` offline — not yet re-synced |
| `q_e_yd` | `Q_diag[1]` | `0.5` live / `0.8` offline — not yet re-synced |
| `q_e_psi` | `Q_diag[2]` | `2.0` live / `1.6` offline — not yet re-synced |
| `q_r` | `Q_diag[3]` | `1.0` live / `0.70` offline — not yet re-synced |
| `q_e_v` | `Q_diag[4]` | `5.45` live / `5.55` offline — close, not yet re-synced |
| `r_delta` | `R_diag[0]` | `1.8` (matched) |
| `r_a_accel` | `R_diag`/`R_A_ACCEL` | `2.5` live / `1.8` offline (`R_diag[0]` doubles as the offline accel-effort slot) — not yet re-synced; see "Accel/brake effort weight split" below |
| `r_a_brake` | `R_A_BRAKE`/`R_diag[1]` | `0.5` live / `0.77` offline — not yet re-synced |
| `r_rate_delta` | `R_rate_diag[0]` | `2.0` live / `2.5` offline — not yet re-synced |
| `r_rate_a` | `R_rate_diag[1]` | `2.35` live / `2.4` offline — close, not yet re-synced |
| `terminal_q_scale` | `TERMINAL_Q_SCALE` | `1.0` (matched) |
| `adaptive_q_scaling_enabled` | `ADAPTIVE_Q_SCALING_ENABLED` | `True` (matched) |
| `steer_rate_anti_hunt_enabled` | `STEER_RATE_ANTI_HUNT_ENABLED` | `True` (matched) |
| `adaptive_r_rate_enable_in_corners` | `ADAPTIVE_R_RATE_ENABLE_IN_CORNERS` | `True` (matched) |
| `ref_heading_rate_limit_enabled` | `REF_HEADING_RATE_LIMIT_ENABLED` | `False` (matched) |
| `ref_heading_rise_rate_deg_s` | `REF_HEADING_RISE_RATE` | `90.0` (matched) |
| `adaptive_r_rate_during_floor` | `ADAPTIVE_R_RATE_DURING_FLOOR` | `0.625` (matched) |
| `anti_hunt_boost_max` | `ANTI_HUNT_BOOST_MAX` | `6.0` — see "remaining fields" note; confirm this offline constant still exists at re-sync time |
| `corner_factor_k` | `CORNER_FACTOR_K` | `8.0` (matched) |
| `q_ey_straight` / `q_ey_corner` | `Q_EY_STRAIGHT` / `Q_EY_CORNER` | `4.5` / `9.0` (matched) |
| `q_epsi_straight` / `q_epsi_corner` | `Q_EPSI_STRAIGHT` / `Q_EPSI_CORNER` | `1.5` / `3.0` (matched) |
| `q_r_straight` / `q_r_corner` | `Q_R_STRAIGHT` / `Q_R_CORNER` | `1.0` / `0.5` (matched) |
| `rrate_steer_straight` / `rrate_steer_corner` | `RRATE_STEER_STRAIGHT` / `RRATE_STEER_CORNER` | `2.0` / `1.25` (matched) |
| `r_steer_corner_mid` | `R_STEER_CORNER_MID` | `1.35` (matched) |
| `low_speed_corner_boost_v_half` | `LOW_SPEED_CORNER_BOOST_V_HALF` | `4.0` (matched) |
| `low_speed_corner_boost_max_extra` | `LOW_SPEED_CORNER_BOOST_MAX_EXTRA` | `0.3` (matched) |
| `epsi_ra_half_rad` | `EPSI_RA_HALF_RAD` | `radians(10.0)` (matched) |
| `epsi_ra_accel_boost_max` | `EPSI_RA_ACCEL_BOOST_MAX` | `2.0` (matched) |
| `epsi_ra_brake_floor` | `EPSI_RA_BRAKE_FLOOR` | `0.5` (matched) |
| `nmpc_q_e_y` … `nmpc_anti_hunt_boost_max` (13 override fields) | `NMPC_Q_E_Y` … `NMPC_STEER_RATE_ANTI_HUNT_ENABLED` | all `-1.0`/`False` (inherit sentinel, matched) — see "Nonlinear MPC" section below. `NMPC_ANTI_HUNT_BOOST_MAX` has no offline constant (see that section) |

The remaining `MPCParams` fields (`max_delay_compensation_steps`,
`predict_epsi_clip`, `pose_age_lp_alpha`, `n_delay_hysteresis`,
`delay_compensation_enabled`) are live-only tuning knobs with no offline
`settings.py` counterpart — the offline sim has no equivalent of live pose
latency to compensate for. Add a `settings.py` constant and a
`rollout_core.py` call-site keyword the same way as the rows above before
relying on tuning one of these offline, if that ever becomes relevant.

## Corner-factor scheduler — what replaced the lookahead gain-scheduling family

**The current mechanism** is `_corner_factor`/`_low_speed_corner_boost`/
`_blend`: a single continuous CURRENT-curvature-only fraction blending four
`Q`/`R_rate` weights between a straight endpoint and a corner endpoint, plus
an always-on, independent heading-error-driven accel/brake asymmetry
(`epsi_ra_*`). See `architecture.md`'s "Corner-factor scheduler" section for
the full formulas and mechanism, and `tuning.md` §4.3b for the tuning-surface
reference — not repeated here to avoid duplicating either.

It exists on both sides: `fsae_MPCTest/controller/model_utils.py`
(`_corner_factor`/`_low_speed_corner_boost`/`_blend` plus `settings.py`'s
matching constants) and `fsds_simulator/`'s copies of `mpc_core.py`/
`mpc_params.py`/`fsae_params.yaml`, byte-identical to the live files (see the
parity table above).

**What it replaced, and why that matters if you are tempted to re-add it.**
The previous design was ~15 interacting functions that scanned FORWARD along
the path every tick (`lookahead_curvature_profile` → a scalar `kappa_max_abs`,
the peak curvature within a speed-scaled lookahead window) and reweighted
`Q`/`R`/`R_rate` in anticipation of a corner not yet reached: approach/exit
`Q[0,0]`/`Q[2,2]` boosts, `Q[3,3]`/`R[0,0]` relaxations, demand normalisation
(`_corner_demand`/`_alat_ceiling_at`), a U-turn detector, straight-line
`Q`/`R[0,0]` adjustments, and a `CornerMap`/`_segment_corners`
precomputed-path fast path for the same scan. See `mpc_core.py`'s own
"Lookahead gain-scheduling family: removed" comment for the exhaustive
function-name list, and [`removed_mechanisms.md`](removed_mechanisms.md) for
the mechanism-level detail of each (`tuning.md` §4.4/§4.6/§4.8/§4.10 and
`architecture.md`'s "Historical" subsection both link there rather than
restating it).

The family was removed because this MPC formulation already predicts state
error against the reference at each future horizon step. Reweighting TODAY's
(usually near-zero) cost based on what a forward scan finds ahead does not
change what the horizon predicts once the car actually gets there — the
mechanism was reweighting a cost that mostly was not there yet, not
manufacturing anticipation. Repeated piecemeal retuning of it never produced
a clear net win. This is the same structural argument, one level up, that
motivates the nonlinear MPC below: reweighting an existing error's cost is not
the same as making the *prediction itself* see the road bend.

## Slew-rate limit (`du_max`): on both sides, at 180 deg/s

The hard per-step slew-rate constraint on `[delta_cmd, a_cmd]` must exist on
both sides. It was once live-only, with `controller/optimiser.py` carrying no
such constraint at all, so the offline tuner was optimising against a plant
that could change steering arbitrarily fast while the real car was clamped —
weights tuned offline did not transfer faithfully, independent of any weight
choice. Keep both sides constrained.

How it is formulated:

1. `init_parameterized_mpc()` / `solve_mpc()` in `controller/optimiser.py`
   take an optional `du_max`, mirroring the live formulation. It is baked
   into the cached QP the same way `u_min`/`u_max` are, so it participates in
   the same cache-staleness check (passing a different `du_max` rebuilds the
   problem instead of silently reusing stale constraints). Only the step-0
   constraint against `u_prev` is omitted offline — step 0 is already anchored
   through `weighted_u_prev` in the cost, and `u_prev` isn't a constraint
   parameter in the cached formulation.
2. The limit is **180 deg/s**, expressed as a *rate* (`max_steer_rate * DT`)
   rather than a fixed per-step angle, so its physical meaning survives a
   change of `DT`.

**Why 180 deg/s and not the original 80.** Live telemetry
(`mpc_standalone_control_1785976976.csv`) showed the steering command pinned
exactly on an 80 deg/s limit for **41% of all control steps**, reversing sign
at ~8 Hz — a rate-limit-induced limit cycle, not a weight-tuning problem.
Inverting the logged yaw rate through the kinematic bicycle
(`delta = atan(L*r/v)`) put the *achieved* roadwheel rate at p99 ≈ 138 deg/s
and max ≈ 218 deg/s, so the real actuator is at least ~200 deg/s. 180 deg/s
sits just under that measured floor.

**Why the offline sim didn't catch it.** With the same weights and the same
80 deg/s limit, an offline rollout on `PATH_MICRO_SLALOM` hits the limit on
only **0.5%** of steps versus the live car's 41%, and composite scores at
80 vs 180 deg/s differ by <0.002 across `PATH_S_BEND`, `PATH_SUDDEN_TURN` and
`PATH_SPIRAL`. The offline sim uses a fixed `DELAY_STEPS = 1` and a smooth
synthetic path, so it simply never enters the saturated regime the live car
lives in (jittery measured delay, replanned/noisy perception path). Do not
read "the offline score barely moved" as "the change doesn't matter" — the
constraint is there to stop the live controller sitting on its limit.

The true FSDS steering rate is **not recoverable from this repo** (the PhysX
vehicle setup lives in git-LFS `.uasset` binaries), so 180 deg/s is a measured
lower-bound estimate, not a datasheet figure. Refine it via system-ID on the
running sim and update both sides together.

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
3. **Speed-target rise limiter** — `SPEED_TARGET_RISE_RATE = 2.0` m/s², applied
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
was a live-only defect, and the sim needs no mirrored change. If you ever want
to reproduce a slow-pose regime offline, the correct model is a **pose
zero-order hold at a configurable rate**, not more `DELAY_JITTER_STEPS` —
jitter models a *varying* delay, whereas this was a *systematically halved
measurement rate*. Those are different failure modes and the jitter knob will
not reproduce it.

**A sibling bug of the same root-cause class, also fixed** (see
`docs/logs/sim_to_real_investigation.md` §55): the fix above makes `pose_rate` keep up
with the controller, but it did not guarantee that a given `car_position`
sample and the `car_speed`/`car_yaw_rate` the controller read *at the same
tick* came from the same underlying odom instant — `mpc_controller.py`/
`mpc_controller_standalone.py` subscribed to the raw 250 Hz
`/fsds/testing_only/odom` directly for speed/yaw-rate, a second, independent
subscription racing `sim_perception.py`'s own separate subscription to the
same publisher (the one that produces `car_position`). This is a
**cross-topic snapshot mismatch**, not a rate mismatch — different mechanism,
same underlying cause (`sim_perception.py`'s publish timing not actually
delivering what a downstream consumer assumes). Fixed by adding
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

## Live/offline score parity

`fsds_simulator/control/fsae_control/fsae_control/scoring.py` is a **verbatim
copy** of `sim/scoring.py` — `compute_composite_score()`, `RolloutMetrics.
add_step()` and `RolloutMetrics.finalize()` are identical, so a score produced
on the live car is directly comparable to one produced by
`tuner/offline_tuner.py`. This was verified by running both implementations
over 500 identical synthetic steps: all 18 returned fields matched to within
1e-12 (bit-identical composite score).

The one intentional difference is the settings import. `sim/scoring.py` pulls
`SCORE_WEIGHTS`, `METRIC_SCALES` and the bonus/penalty constants from
`settings.py`, which is not on the live car's `PYTHONPATH`; the live copy
inlines them as module constants. **These must be kept numerically identical**
— they're listed in the numeric-parity table above.

`METRIC_SCALES` divides each metric by a reference magnitude before weighting:
`score = SCORE_WEIGHTS @ (metrics / METRIC_SCALES)`. It exists because without
it a metric's real influence is `weight × typical magnitude`, which made the
composite effectively single-objective — all ten non-tracking metrics combined
moved the score by +0.0064 against a −0.2649 tracking term, so the smoothness
and oscillation terms could not bite regardless of their weights. Because it is
inlined in **three** places (`settings.py`, the live `fsae_control/scoring.py`,
and the `fsds_simulator/` mirror), a change to it is a three-file edit — the
same rule as `SCORE_WEIGHTS`.

**`progress`/`reached_end`/`time_bonus` are computed live too.** They have to
be passed explicitly: if both live controller nodes call `telemetry.close()`
with no arguments, `progress` defaults to `0.0` and `reached_end` to `None`,
`compute_composite_score()` reads that as "never finished," and every live run
scores exactly `CONSTRAINT_FLOOR + DNF_PENALTY = 13.0` regardless of how the
car actually drove (the 13 underlying quality metrics stay correct; only the
composite number goes dead). This was a real defect once — see the "Live
scorer reports `13.0`" row in
[`docs/logs/sim_to_real_investigation.md`](logs/sim_to_real_investigation.md)'s
findings table.

`LapProgressTracker` in `telemetry_logger.py` supplies them: it tracks the car's
forward-bounded nearest-index position against the precomputed track path
(the same CSV already loaded for the live speed lookup) to get real
`progress`/`reached_end`, and integrates `ds / v_target` over the
already-loaded speed profile for an `optimal_time` bound —
**not** a call into `speed_profile.optimal_lap_time()`, since that solver
lives in `fsae_MPCTest` and is not on the live node's `PYTHONPATH` (see the
settings-import caveat above). `time_bonus = optimal_time * progress /
actual_lap_time`, clipped to `[0, 1]`, same scaling convention as
`sim/rollout_core.py`. Both controller nodes feed the tracker's output into
`close()`, and the CSV header now also records `lap_time_s`/`optimal_time_s`.
This only works when a precomputed speed profile is loaded (`map_path` set,
the normal live-driving setup); a run against the live planner topic instead
still has no path end to measure progress against, so `progress`/`reached_end`
fall back to their old defaults in that mode only.

One input still has no live equivalent and defaults to `False`:

- `offtrack` — the offline rollout knows ground-truth track edges; the car
  does not.

The emitted CSV header records `score_is_partial=1` whenever `time_bonus` is
`0.0` and `offtrack` is `False`, so a reader can't mistake a partial live
score (no precomputed speed profile, or run never finished) for a full one.
The weighted-metric component (the 13 metrics × `SCORE_WEIGHTS` — see the
"Score weights / bonuses / penalties" row above) is directly comparable either
way; only the bonus/penalty terms differ, and only `offtrack` is
unconditionally unavailable.

## The sim-to-real gap: CAUSE FOUND, fix not yet applied

> **Full investigation history** — every hypothesis tried, why each looked
> right, and how it was eliminated — is in
> [`docs/logs/sim_to_real_investigation.md`](logs/sim_to_real_investigation.md). Read that
> before re-testing any candidate that looks unexplored; most already were.

Measured on the recorded `comp test map 3`, same tuned gains both sides:

| | offline sim | live car |
|---|---|---|
| steering saturation | 3.4% | **21.1%** |
| \|e_psi\| mean / p90 | 6.0° / 13.8° | **15.9° / 42.0°** |
| max \|e_y\| | 1.82 m | 1.20 m |

The car sits at full steering lock six times more often than the simulator, and
when it does it is pulling only 4.14 m/s² lateral at 5.74 m/s — it is not
cornering hard, it is rotating back from a large heading error.

Heading error arrives in sustained episodes (median 0.47 s, up to 2.44 s, 96% of
energy below 1 Hz), i.e. a stale/wrong reference rather than high-frequency
chatter.

**Candidates tested and eliminated:**

| candidate | verdict |
|---|---|
| plant grip too generous | No — a_lat mean 3.57 sim vs 3.76 live |
| entering corners too fast | No — car is *slower* when saturated (5.74 vs 8.03 m/s) |
| planner centreline quality | No — offline is *worse* in the tail (R=0.16 m vs 1.26 m) |
| SLAM pose noise | No — overshoots reversals (3.76/s vs 1.62) barely moves saturation |
| extra actuation delay | No — 2 steps moved saturation 3.4% → 2.3% |
| planner update rate (1 Hz vs 20 Hz) | No — throttling *improved* the sim |
| **pose-feed hold** | **No** — model added and verified firing; 3.4% → 4.4% only |
| **tyre grip / understeer** | **No** — fitted and validated, see below |
| **`MAX_STEER_RAD` command scaling** | **No** — refuted; deficit is speed-dependent, not constant |
| **actuator lag** | **No** — `s` does not degrade with command rate |
| **yaw_rate / speed telemetry error** | **No** — channels validated against pose derivatives |
| **tyre front/rear balance** | **No** — needs C_f at 10% of physical to reach live K_us |
| **`SteeringCurve` (UE4/PhysX speed-dependent steering scaling)** | **No** — read directly from `TechnionCarPawn`'s `WheeledVehicleMovementComponent4W` in the UE4 Editor: flat at 1.0 across all 3 keyframes, no speed-dependent scaling at all. See `docs/logs/sim_to_real_investigation.md` §18/§20/§24/§25 for the full mechanism search, the UE 4.27 build process needed to check it, and the read-out |

**ROOT CAUSE** (open-loop system-ID + step test):
**FSDS enforces a sustained LATERAL-ACCELERATION ceiling of ~7.5 m/s².**
Below ~6 m/s the car never reaches it and the commanded steering angle is
delivered exactly (`s` = 1.00–1.01 at 3 and 5 m/s); above it the yaw response
collapses (`s` = 0.34 at 8 m/s, 0.17 at 14 m/s). It is far below the 12.3 m/s²
the same car reaches on a lap, so it is **not** tyre saturation — a grip limit
does not depend on speed.

The sweep alone read this as a *yaw-rate* cap (~0.7 rad/s); the step test
showed **lateral acceleration** is what is held constant (1.07× spread across
speeds vs 1.56× for yaw rate). The two are indistinguishable at a single speed.

**Modelled** in `model/vehicle_physics.py` (`alat_ceiling*`). It moves every
metric toward the car but closes only part of the gap — live still saturates 3×
more often. See "MECHANISM: a dynamically-enforced lateral-acceleration
ceiling" below, and `docs/logs/sim_to_real_investigation.md` for the full history.

The pose-feed hold is real (see `PoseFeedHold`) and is modelled, but it
accounts for almost none of the gap.

### The understeer measurement, and why grip is NOT the answer

The most promising lead was that the offline plant simply corners better than
the car. It does — but fixing that does not close the gap, and the way it fails
is informative.

**Full-lock steady-state turn** (speed held by throttle, 300 steps to settle).
Kinematic limit at 25° lock with L=1.55 m is R=3.32 m:

| speed | offline R | understeer | live |
|---|---|---|---|
| 4 m/s | 3.76 m | ×1.13 | |
| 6 m/s | 3.81 m | ×1.15 | **R=6.9 m, ×2.03** |
| 8 m/s | 4.43 m | ×1.33 | |
| 10 m/s | 7.09 m | ×2.13 | |

At 6.2 m/s (the live mean *during saturation*) the sim turns essentially at the
kinematic limit while the car needs nearly double the radius.

**Understeer gradient** `K` from `delta = L/R + K·a_lat`, fitted over
quasi-steady points (|yaw accel| < 0.5, v > 3, |yaw rate| > 0.05 — 261 points,
16.5% of the live run):

| | K (rad per m/s²) | deg/g |
|---|---|---|
| live | 0.00869 | 4.89 |
| offline, μ=1.76 (current) | 0.00595 | 3.34 |
| offline, μ=1.455 (**fitted by bisection**) | 0.00869 | 4.89 |

**The fit succeeds on K and then fails validation.** μ=1.455 reproduces the
gradient exactly, but still gives ×1.16 understeer at 6 m/s against the live
×2.03, and in closed loop moves saturation only 4.4% → 4.9% (live: 21.1%).

Two independent measurements of "how much does this car understeer" give
incompatible answers. That is the signature of a model **structure** problem,
not a parameter needing a scale factor.

Pushing grip lower does not help either — it produces a *different* failure:

| μ | saturation | \|e_psi\| mean | outcome |
|---|---|---|---|
| 1.76 (current) | 4.4% | 6.3° | ok |
| 1.455 (fitted) | 4.9% | 6.7° | ok |
| 1.06 | 8.4% | 5.8° | **DNF** |
| 0.79 | 7.0% | 4.7° | **DNF** |
| **live** | **21.1%** | **15.9°** | ok |

Lower grip makes the sim car *slide* — heading error goes DOWN and it crashes.
The car does something else: it holds full lock for sustained periods (21
episodes, median 0.75 s, up to 2.5 s) at only 4.14 m/s² lateral, which is well
inside any plausible grip limit.

### MEASURED: the car's yaw response is ~3× weaker than commanded

Measured from `fsae_logs/mpc_standalone_control_1786007642.csv` and
`…_1786005274.csv` (two independent one-lap runs, same tuned weights) by
inverting the kinematic bicycle on logged `yaw_rate` and `delta_cmd`:

    delta_achieved = atan(L · yaw_rate / v)      L = 1.55 m

**Headline: at full lock the car delivers well under half the commanded angle.**

| | run 1786007642 | run 1786005274 |
|---|---|---|
| samples at full lock (\|δ_cmd\| ≥ 24.9°) | 281 | 313 |
| mean speed there | 6.82 m/s | 5.97 m/s |
| mean \|yaw rate\| | 0.739 rad/s | 0.734 rad/s |
| **implied achieved steer** | **9.54°** | **10.79°** |
| deficit vs commanded 25° | **×2.62** | **×2.32** |

This happens at ~5 m/s² lateral — nowhere near the ~12 m/s² the same car
demonstrably reaches — so it is *not* tyre saturation.

#### The deficit is speed-dependent, not a constant scale

Candidate models fitted to achieved yaw rate (common target, so R² is directly
comparable). `A` is what the offline plant does today:

| model | run 1 R² | run 2 R² |
|---|---|---|
| A neutral steer — *the offline plant* | **−1.81** | **−1.02** |
| B constant command scale (`MAX_STEER_RAD` error) | 0.655 | 0.606 |
| C understeer, `r = vδ/(L + K_us v²)` | **0.759** | **0.705** |
| D yaw-rate ceiling, `r_max·tanh(...)` | 0.751 | 0.638 |
| C+D combined | 0.774 | 0.706 |

Fitted `K_us ≈ 0.038–0.045 s²` → **characteristic speed 5.9–6.4 m/s**. The
offline plant measures `K_us ≈ 0.0006` → **52 m/s**, i.e. essentially neutral.
A *negative* R² for A means the offline plant predicts the car's yaw worse than
guessing the mean. This is the sim-to-real gap in one number.

#### Eliminated by this measurement

- **`MAX_STEER_RAD` scaling (the previous leading hypothesis) — REFUTED.**
  A constant scale error predicts a flat `s = δ_ach/δ_cmd` across speed. The
  measured `s` collapses with speed (0.85 → 0.42 → 0.31 → 0.28 across speed
  quartiles), and model B loses to C decisively.
- **Actuator lag — REFUTED.** Within a fixed 7–10 m/s band, `s` does *not*
  degrade as command rate rises (0.30/0.43/0.28 and 0.37/0.40/0.42 across
  \|δ̇\| terciles); lag requires the opposite. `K_us` is also stable as the
  quasi-steady filter tightens from 30°/s to 10°/s.
- **Telemetry error — REFUTED.** `yaw_rate` matches `d(car_yaw)/dt` at
  slope 0.95 / corr 0.95; logged speed matches position-derived speed
  (slope 1.08–1.18). The channels are sound.
- **Tyre front/rear balance — CANNOT REACH IT.** Sweeping front stiffness
  `B_f` from 16.5 down to 5.0 (an implausibly soft front) moves `K_us` only
  0.000 → 0.0047 — an order of magnitude short of 0.040. Analytically,
  `K_us = (m/L)(l_r/C_f − l_f/C_r) = 0.040` requires **C_f at 10% of current**
  (2 500 N/rad vs 24 390), i.e. 43 N per degree of front slip. Not a physical
  tyre.

#### What this leaves

Grip and yaw response are **decoupled**: the car reaches ~12 m/s² lateral
(sim 14.5, so grip is roughly right) yet rotates as if it had almost no front
axle. No tyre model does that, which is why scaling `mu` never worked — and
confirms the earlier μ=1.455 fit was fitting the wrong basis (`a_lat` rather
than `v²`; on a lap the two are confounded, and that fit returned a physically
backwards *negative* understeer gradient, the tell that it was mis-specified).

Remaining candidates, none yet tested:

1. **Steering geometry / rack ratio inside FSDS** — a nonlinear or
   speed-scaled rack (some driving sims reduce lock with speed for
   controllability) would produce exactly this signature. Most likely.
2. **Wheelbase mismatch** — `VehicleParams` uses L = 1.55 m unverified against
   the FSDS vehicle. Note this cannot be the whole story: L would have to be
   ~3.9 m to explain the full-lock number alone.
3. **A yaw-damping or stability-control term in FSDS** not modelled offline.

FSDS's vehicle configuration lives in git-LFS `.uasset` binaries and is not
readable from this repo, so these must be separated by **open-loop
experiment**: command fixed steering angles at fixed speeds on an empty map
and record the achieved yaw rate. That isolates the plant from the controller,
which no lap log can do.

#### The open-loop experiment

**Where the three pieces live.** The node and harness are working-tree-only
files in the live ROS 2 workspace with no mirror here and no upstream
counterpart (see "Current mirror scope" above); only the analysis script lives
in this repo.

| file | repo | role |
|---|---|---|
| `control/fsae_control/fsae_control/steering_sysid.py` | `fsae_planning` (live ROS 2 ws) | the node — drives FSDS directly |
| `ros2/run_steering_sysid.sh` | **FSDS repo root**, next to `launch_all.sh` | one-command harness |
| `tuner/checks/steering_sysid_analysis.py` | `fsae_MPCTest` | reads the log, names the mechanism |

**Run it with one command** (starts FSDS, waits for RPC, starts the bridge,
waits for odom, runs the sweep, analyses the log, tears everything down —
including on Ctrl+C):

    cd <FSDS repo>/ros2 && ./run_steering_sysid.sh

Flags: `--no-sim` (FSDS already running), `--quick` (fewer points), and any
`-p name:=value` passes through to the node.

The harness **refuses to start** if `mpc_controller`, `fsds_bridge` or
`stanley` is already running: two publishers on `/fsds/control_command`
interleave commands and silently corrupt the log.

**Run it on an empty map.** It circles at up to 14 m/s and does not brake for
cones.

##### Geometry constraints learned the hard way

The first real run drove into a map wall. Three separate design faults, all
now fixed — worth recording because each is easy to reintroduce:

1. **Straight-line settling does not fit in any bounded area.** The original
   sequence was settle-straight → turn → recover-straight, which carries the
   car ~120 m downrange per point at 14 m/s (measured: 103 m of drift before
   impact). The node now reaches target speed *while already turning*, so it
   orbits instead of travelling.
2. **A geofence is necessary but not sufficient.** `home_radius` (40 m,
   return) and `max_radius` (70 m, hard abort) are checked from every phase.
   On their own they fired ~126 times per sweep and starved it of time —
   16/20 points in 40 min.
3. **Some (speed, steering) pairs cannot be driven in a bounded area at all.**
   Radius grows as `(L + K_us·v²)/δ`, so high speed with small steering traces
   an enormous arc — at 14 m/s and 0.5 normalised steering the real orbit is
   **~86 m across**. Those points are also the least informative (small angle
   ⇒ small yaw signal). The node now predicts each orbit and skips the ones
   that will not fit, logging what it dropped.

   The prediction uses a **deliberately pessimistic** `K_US_ESTIMATE = 0.05`,
   above the ~0.038–0.045 measured on the car. Over-estimating costs one
   skipped point; under-estimating costs a geofence abort mid-measurement.
   An earlier near-neutral estimate (0.005) predicted 23 m for that same
   point and let it through — that is what caused fault 2's symptom.

   With all three fixed: **16/16 points, 2.6 min, zero geofence triggers,**
   max distance 63 m against the 70 m limit, full 3–14 m/s coverage retained.

Default steering commands are `[0.5, 0.65, 0.8, 1.0]` — biased high for the
same reason.

##### Reading the log and the verdict

The log records the **raw normalised `cmd.steering`** alongside the roadwheel
angle we assume it maps to. That assumption is the thing under test, so
recording only the assumed angle would beg the question.

A falling `s = δ_ach/δ_cmd` is **not** by itself diagnostic — a speed-scaled
rack, genuine understeer and tyre saturation all produce one. The analyser
therefore fits all five candidates to achieved yaw rate (common target ⇒
comparable R²) and reports the margin to the runner-up:

| winning model | meaning |
|---|---|
| neutral (s≈1) | steering path is fine; look at the controller/reference |
| constant scale | `MAX_STEER_RAD` wrong — fix in all three copies |
| speed-scaled rack | FSDS reduces lock with speed; model it in the plant |
| understeer (v²) | real vehicle dynamics — but see the C_f note above |
| grip saturation | yaw capped by lateral grip |

Validated against synthetic logs with each mechanism injected: all five are
identified correctly and their parameters recovered exactly (scale 0.500,
c 0.0600, K_us 0.0400, a_lat_max 12.0). The default speed set is **3–14 m/s**
because speed-scaled rack and understeer are near-degenerate over a narrow
band — they sat within 0.004 R² over 4–10 m/s, versus a separable 0.018–0.054
over 3–14 m/s. **If the analyser prints a margin warning, widen the speed
range and re-run rather than trusting the verdict.**

The analyser also refuses to return a verdict when fewer than 3 RECORD windows
contain real motion. A car wedged against a wall still emits RECORD rows, but
they are all zero speed / zero yaw; fitting those gives `R² = nan` for every
model, and the ranking then reports a confident, meaningless answer. (Observed:
the first crashed run produced "FSDS delivers the commanded angle" from pure
zeros.)

##### RESULT (2026-08-06): FSDS caps yaw rate above ~6 m/s

Full clean sweep, 16/16 points, no geofence triggers —
`fsae_logs/steering_sysid_1786014330.csv`.

**Below ~5 m/s the car delivers the commanded angle exactly. Above it, the
response collapses.**

| speed | commanded | achieved | `s` |
|---|---|---|---|
| 3 m/s | 25° | 25.3° | **1.01** |
| 5 m/s | 25° | 24.9° | **1.00** |
| 8 m/s | 25° | **8.5°** | **0.34** |
| 11 m/s | 25° | 5.8° | 0.23 |
| 14 m/s | 25° | 4.1° | 0.17 |

Not a gradual understeer curve — a cliff between 5 and 8 m/s. Model fit:

| model | R² |
|---|---|
| **grip saturation** (`a_lat` ceiling) | **0.987** |
| understeer (v²) | 0.898 |
| speed-scaled rack | 0.893 |
| constant scale | 0.617 |
| neutral | −1.226 |

First time the models have separated by a decisive margin (0.089).

**The cap is not tyres.** Fitted ceiling **7.0 m/s²**, against 12.3 m/s²
measured on the same car during a lap and 14.5 m/s² in the offline plant. A
tyre limit is speed-independent; this one engages above a threshold speed.
Lateral acceleration vs steering makes it plain — it rises normally at low
speed and goes flat once the cap engages:

| v | 0.50 | 0.65 | 0.80 | 1.00 |
|---|---|---|---|---|
| 3 m/s | 1.61 | 1.93 | 2.38 | 2.75 |
| 5 m/s | 4.37 | 5.28 | 6.30 | 7.48 |
| **8 m/s** | **6.27** | **5.95** | **5.72** | **6.09** |
| 11 m/s | — | 6.84 | 7.55 | 8.05 |

At 8 m/s more steering produces *less* cornering. The signature — a yaw-rate
ceiling (~0.7 rad/s) that engages above a threshold speed while commanded
angles are honoured exactly below it — points at a **stability-control or
yaw-damping term inside FSDS**, which the offline plant does not model at all.

This closes the loop on the closed-loop symptom: the MPC plans a corner, FSDS
clamps the yaw, heading error builds, the controller demands more steer and
hits the 25° stop → the 21% saturation seen on every lap.

> **Not yet acted on.** The fix is to model the cap in the offline plant, not
> to bend tyre parameters to imitate it (which would also wreck the plant's
> genuine 14.5 m/s² grip).

##### MECHANISM (2026-08-06): a dynamically-enforced *lateral-acceleration*

##### ceiling, ~7.5 m/s²

Step-input test, 12 trials —
`fsae_logs/steering_step_1786015706.csv`.

**Two signatures appear together, and neither alone explains it.**

1. **It is a cap.** At 8 m/s, 0.60 and 1.00 steering settle to the *same* yaw
   rate (0.93–0.96 vs 0.90–0.94 rad/s) despite a 67% larger command. A scaled
   authority would keep them proportional.
2. **It overshoots.** Mean **30%** above the settled value before decaying
   (peaks 10.9 m/s² lateral against a ~7.5 ceiling). A static clip cannot
   overshoot.

So the ceiling is enforced by a term that **takes time to build**, not by a
hard clip.

**What is held constant is lateral acceleration, not yaw rate.** This was the
discriminating measurement, and it needed two capped speeds to answer:

| v | settled yaw | settled a_lat |
|---|---|---|
| 8 m/s | 0.949 rad/s | **7.80 m/s²** |
| 12 m/s | 0.609 rad/s | **7.29 m/s²** |
| spread | **1.56×** | **1.07×** |

Yaw rate varies 1.56× across speeds while lateral acceleration is flat to
within 7%. At 4 m/s the car sits at 4.15 m/s², below the ceiling, so it is
unconstrained there — and yaw duly scales with steering (1.76× spread vs 1.10×
at 8 m/s). That is why the sweep saw `s ≈ 1.0` below ~6 m/s: **the cap has not
engaged, not that the steering behaves differently.**

**Modelled 2026-08-06** in `model/vehicle_physics.py` as a restoring yaw moment
with a first-order lag, *not* a clip (a clip reproduces steady state but
removes the turn-in transient the MPC reacts to):

| parameter | value | basis |
|---|---|---|
| `alat_ceiling` | 7.5 m/s² | measured settled a_lat (7.80 @ 8 m/s, 7.29 @ 12) |
| `alat_ceiling_mode` | `'pi'` | **corrected 2026-08-07** — see below |
| `alat_ceiling_gain` | 450 N·m | fitted to measured PEAK only; settled falls out by structure |
| `alat_ceiling_tau` | 0.40 s | **measured 2026-08-07**, 8 s hold (was 0.25, behavioural) — did NOT close the saturation gap |
| `alat_ceiling_enabled` | `True` | models **FSDS**, not the physical car — disable for real-vehicle work |

> **CORRECTED 2026-08-07: the law was proportional, and a proportional law
> cannot hold a setpoint.** With `alat_lim` lagging toward the excess, steady
> state gives `alat_lim = excess`, so the restoring moment is `excess × gain` —
> a P controller, whose equilibrium must sit *above* its setpoint. Sweeping the
> gain shows the trade-off is monotone and unavoidable: at 700 the settled value
> was **+0.97 high**; reaching the settled value (6000) collapsed the peak
> **−2.55 low**. The two "false starts" recorded above were the two ends of one
> structural flaw, not two bad guesses.
>
> The law is now a **leaky integral of the signed excess**, clamped at zero:
>
>     err      = |a_lat| - ceiling
>     alat_lim = max(0, alat_lim + err * h / tau)
>     M_z     -= sign(r) * alat_lim * gain
>
> An integral can only stop growing when the error is zero, so the settled value
> is pinned AT the ceiling **by structure for any gain**, leaving one free
> parameter for the transient. Both measured targets are now hit at once
> (settled 7.50 vs 7.68 measured; peak 10.37 vs 10.42), and on the **held-out
> sweep** the capped-point error improves 5× (mean +1.41 → +0.29, MAE
> 1.60 → 0.87). Before the fix the sim sustained 8.3–8.9 m/s² where FSDS
> sustains 6.1–8.1 — a 20–35% surplus centred on the live car's mean speed.
>
> Reproduce: `python -m tuner.checks.plant_openloop_validation --ab`
>
> **What this did NOT fix:** steering saturation moved only 6.32 → 6.74% against
> live's 21.1%. It corrects the plant's *lateral-acceleration distribution*
> (time-above-ceiling ×1.45 → ×0.91 of live), not its steering behaviour. See
> `docs/logs/sim_to_real_investigation.md` §12 for what was then eliminated and what the
> residual gap has been narrowed to.

> **Newly identified, not yet modelled: the ceiling is speed-dependent.**
> Measured sustained a_lat *rises* with speed — 6.45 @ 8 m/s, 7.54 @ 11,
> 9.26 @ 14 — while the model pins it flat at 7.5 (residuals +1.0 / ~0 / −1.76).
> Deliberately not fitted: 16 points from one run. Note also that the step test's
> 3 s settle (7.5 @ 8 m/s) and the sweep's long orbit (6.45) **disagree**, which
> a longer `step_s` would resolve at the same time as `tau`.

Verified: below ~6 m/s the plant is untouched (1.84 / 3.46 m/s² at 3 / 4 m/s,
identical with the ceiling on or off), matching the measurement that the cap
does not engage there. Peak a_lat on the recorded map falls 14.06 → 9.05 m/s².

> **`tau` was nearly set wrong, and the failure is instructive.** Fitting it to
> the measured ~30% *overshoot* gave `tau = 1.0 s`, which reproduced the step
> test well — and then DNF'd the car at 6.3 s on a real lap. Corners arrive in
> ~0.4 s, so a term that takes ~1 s to build does nothing during turn-in, lets
> the car overshoot into the corner, and engages only once it is already off
> line. **A ceiling that acts too late is worse than no ceiling.** `tau = 0.25`
> matches peak a_lat instead and acts in time.

##### The speed profile is NOT a discrepancy (investigated and closed)

An earlier revision of this section claimed the recorded track's speed profile
was ~50% faster than the car and blamed it for a DNF. **That was wrong on both
counts**, and the correction is worth recording because the mistake is easy to
repeat.

The error was comparing the **stored oracle profile** (`V`, mean 12.08 m/s)
against the live car's **achieved speed** (8.03 m/s). Those are different
quantities. `V` comes from `compute_speed_profile()` in `track_io`'s
`_resample_dense()` and is used only to size the step budget — **it is never
the runtime target.** Both stacks compute their target per tick instead.

Compared like with like, they agree closely:

| | sim | live |
|---|---|---|
| runtime `v_target` mean | 10.50 m/s | 12.53 m/s (first 6 s) |
| **achieved speed mean** | **8.20 m/s** | **8.03 m/s** |
| achieved max | 10.86 | 12.50 |

A 2% difference in achieved speed, not 50%. In the first 6 s the live car is
in fact *faster* (9.98 mean, 13.87 max vs the sim's 8.30 / 10.86), so "the sim
enters corners too fast" was backwards.

Parity was also verified in the code: the live node
(`mpc_controller_standalone.py`) computes `curvature_speed(v_max=20, v_min=1.5,
scan_end=24, a_lat_max=4.0)`, then applies `tracking_error_speed_gate` and a
rise-rate limit. `sim/rollout_core.py` applies **both** of those at runtime with
the same constants.

**The actual cause of that DNF was my ceiling being too stiff** — see the gain
note above. `alat_ceiling_gain = 3000` enforced 7.5 m/s² as a near-absolute
limit, but 7.5 is a *sustained* ceiling: the live car exceeds it on **9.8% of
ticks**, peaking at **12.34 m/s²**, in short excursions (median 0.05 s, max
0.85 s). Refitting the gain to the measured *peak* rather than the settled
value (700) reproduces that behaviour and completes the lap.

##### Effect of the ceiling, and what remains

Same map, same tuned gains:

| | before (no ceiling) | P law (700) | PI, tau=0.25 (450) | **PI, tau=0.40 (450)** | live |
|---|---|---|---|---|---|
| steering saturation | 4.4% | 6.3% | 6.7% | **4.8%** | **21.1%** |
| reversals/s | 0.84 | 0.82 | 0.93 | 0.80 | 1.62 |
| \|e_psi\| mean / p90 | 6.3 / 14.2 | 7.3 / 19.2 | 7.5 / 20.0 | 6.9 / 18.5 | **15.9 / 42.0** |
| a_lat max | 14.06 | 10.53 | 10.80 | 11.24 | 12.34 |
| a_lat > 7.5 | 14.2% | 14.2% | 8.9% | **10.9%** | 9.8% |
| composite score | — | 0.675 | 0.700 | 0.627 | — |

Reproduce any column with
`python -m tuner.recorded_map_rollout [--mode p --gain 700 | --tau 0.25 | --no-ceiling]`.
`tau=0.40` is the shipped, measured value (see below); saturation moving
*further* from live under it is expected — it's a plant-fidelity fit to a
direct FSDS measurement, not a saturation-tuning knob.

Every metric moves toward the car, and peak lateral acceleration is now
realistic. **But the gap is only partly closed** — live still saturates 4×
more often and carries 2× the heading error. The yaw cap was a real and
necessary fix; it is not the whole story.

The PI columns reproduce the car's **lateral-acceleration distribution** well
(time-above-ceiling within 11–9% of live, vs 45% too high before) while barely
moving, or moving the wrong way on, **steering saturation**. Those are now
known to be separate problems: `docs/logs/sim_to_real_investigation.md` §12 eliminates
cornering capability (§12.6), the `a_cmd` divergence (§12.7), AND
`alat_ceiling_tau` (§12.12 — measured, refit, still no saturation improvement)
as causes of the saturation gap, and narrows it to the *rate of entry* into the
high-heading-error state (2.6×, with matching in-state behaviour). The
reference-heading lead (§12.8) is next, and is testable **offline**.

**Validation tooling** (added 2026-08-07 — this loop was previously missing, and
its absence is how a 13% sustained-cornering surplus survived a refit):

| tool | answers |
|---|---|
| `tuner/checks/steering_sysid_analysis.py`, `tuner/checks/steering_step_analysis.py` | what does FSDS do? |
| **`tuner/checks/plant_openloop_validation.py`** | **does our plant reproduce it?** (`--ab`, `--robustness`) |
| **`tuner/recorded_map_rollout.py`** | the closed-loop table above, headless and reproducible |
| **`tuner/checks/live_vs_sim_diagnostics.py`** | conditional + reference-heading decomposition of live vs sim |

Caveat carried by `plant_openloop_validation.py`: its **low-speed** comparison is
a confound, not a finding. At 4 m/s full lock the plant cannot hold speed, so
a_lat swings 2.50→4.03 across the speed-hold gains; the capped regime (≥7 m/s) is
robust to ±3.5% over the same range and is exactly timestep-independent.

> **Do not treat the sim as validated yet.** Re-tuning against it now would be
> better than before but still optimistic. The remaining saturation gap needs
> its own investigation.

> **MEASURED 2026-08-07** with `step_s=8.0`, `repeats=2` (12 trials,
> `fsae_logs/steering_step_1786047535.csv`). The original 3 s hold gave a decay
> too scattered to fit (median 0.08 s over a 0.04–1.06 s range); at 8 s the fit
> is tight — median **0.35 s**, 11/12 trials within 0.28–0.46 s (one 0.88 s
> outlier at 5.1 m/s, right at the cap's speed threshold).
>
> Refit to this run's PEAK alone (settled is pinned by structure regardless of
> `tau` — confirmed flat at 7.50 across a 0.25–0.45 sweep): **`tau = 0.40`**,
> taking peak error from −0.45 to −0.04 m/s² against this measurement
> (10.82 measured, 10.37 old model, 10.78 new model). No DNF on the recorded
> map. Closed-loop saturation moved the *wrong* way (6.74% → 4.80%, away from
> live's 21.1%) — expected, since §12.9/§12.12 in
> `docs/logs/sim_to_real_investigation.md` had already localised the residual gap to the
> planner/reference, not this parameter. Reproduce with
> `python -m tuner.checks.plant_openloop_validation`.
>
> Also checked: does a_lat keep decaying past 3 s into the hold? No — flat
> within noise at the 3/5/8 s marks across all 12 trials. The step test's
> short-hold settle (~7.5–7.9) and the sweep's long-orbit sustained value
> (6.1–8.1, lower at every matched speed) are **not** reconciled by slow decay
> within a single hold; that disagreement stays open.
>
> Getting this measurement required fixing a second harness bug beyond the
> quoting fix above: `"${EXTRA_ARGS[@]}"` was expanding *outside* the quoted
> `bash -c "..."` string containing the actual command, so the array never
> reached the inner script — `ros2` saw a bare trailing `-p`. Fixed by passing
> the array as real positional parameters: `bash -c '...' _ "$@"`.

##### The test that produced this (reusable)

Three mechanisms fit the steady-state data about equally: a **hard yaw-rate
limit**, a **speed-scaled steering authority**, or **active damping**. They
differ in the transient after a sudden steering input:

| mechanism | transient signature | how to model it |
|---|---|---|
| A hard yaw limit | rises, then **clips**; different angles settle to the *same* yaw rate | clip yaw rate in the plant |
| B scaled authority | smooth rise to a **lower plateau**, angles stay proportional | scale steering by `f(v)` |
| C active damping | **overshoots**, then decays | add a yaw-damping torque |

Overshoot is the discriminator: neither A nor B can produce it.

- **Node:** `fsae_control/steering_step.py` — settles straight at zero
  steering, then steps and holds, sampling at **50 Hz** (20 Hz would smear a
  rise completing in a few hundred ms). 12 trials, ~2–3 min.
- **Harness:** `ros2/run_steering_step.sh` (same one-command pattern).
- **Analysis:** `tuner/checks/steering_step_analysis.py`.

Validated against synthetic logs with each mechanism injected: all three
identified correctly, A's limit recovered exactly (0.75 rad/s) and B's time
constant to within 0.01 s.

The sweep's `HOLD_STEER` windows had already hinted at this (15 of 16 showed
25–75% overshoot), but they did not start from zero steering, and one showed
243% overshoot from a single 4.533 rad/s telemetry glitch amid ~1.0 rad/s
neighbours. The dedicated test starts from a genuinely straight car and the
analyser median-filters before peak-finding.

##### Earlier partial data (superseded by the result above)

The wall-crash run still yielded **11 valid points at 3–5 m/s** before impact,
and they point somewhere unexpected:

| | measured open-loop | same speeds, from lap logs |
|---|---|---|
| `s = δ_ach/δ_cmd` at 3 m/s | **1.09–1.26** | — |
| `s` at 5 m/s | **1.04–1.21** | ~0.85 at 4 m/s |
| `s` at 8 m/s (1 point) | 1.05 | ~0.42 at 6 m/s |
| fitted `K_us` | **0.0000** | 0.038–0.045 |

Open-loop, the car delivers the commanded angle — slightly *more* than
commanded — with no measurable understeer over the range reached. That is the
opposite of the lap-log finding.

**Do not conclude from this yet.** The run reached 8 m/s exactly once, and the
entire gap lives above that speed; the analyser correctly flagged the
top-two margin as too small to separate. But if the full sweep holds `s ≈ 1`
out to 14 m/s, the plant is exonerated and the investigation moves to the
**controller and its reference**, not the vehicle model — a materially
different search from the one this section has pursued so far.

> **Not yet acted on.** No plant change is recommended from this measurement
> alone. Reproducing `K_us ≈ 0.04` by bending tyre parameters would match the
> symptom while keeping the physics wrong, and would corrupt every downstream
> grip-dependent result. Identify the mechanism first.

### Checked while investigating: the LONGITUDINAL path is not mis-scaled

Prompted by the question "could max throttle / max acceleration / max braking
differ between the offline model and FSDS?", measured on the same two logs:

| | value |
|---|---|
| `a_actual / a_cmd` slope | **1.14–1.18** (car slightly *over*-delivers) |
| peak achieved accel | 11.2–12.4 m/s² (model: 12.0) |
| peak achieved braking | −12.2 to −13.0 m/s² (model: −9.0) |
| throttle saturation | **0.00%** of the run |

So throttle authority, acceleration and braking limits are **not** the problem;
if anything the car brakes harder than the plant model allows.

**But there is a structural mismatch worth knowing about** (found in
`fsds_bridge.py`, not yet acted on): the MPC solves for an acceleration
`a_cmd`, and the bridge **discards it**. It consumes only the target speed and
re-derives throttle with its own P-controller (`KP_THROTTLE = 0.06`,
`KP_BRAKE = 0.40`). Offline, `a_cmd` drives the plant directly.

The live longitudinal loop therefore has an extra proportional lag stage the
offline sim does not model at all. Mean speed error is 0.59 m/s (p90 2.3–2.4),
consistent with a P-controller that never fully catches up. This cannot cause
the yaw deficit — a longitudinal path cannot stop the car rotating — but it is
a genuine sim/live divergence and a candidate explanation for speed-tracking
differences. Modelling it offline (or feeding `a_cmd` through) is untested.

**Consequence:** offline scores are not yet predictive of live behaviour. A
tuning run that scores well offline can still produce a car that saturates
steering a fifth of the time — this has already happened once. Validate on the
car before trusting any tuned weight set.

## MPC prediction horizon: frozen target speed

`sim/rollout_core.py`'s (and `mpc_core.py`'s) MPC formulation bakes
`desired_speed` into `x0[4]` (`e_v`) as a single scalar frozen for the whole
prediction horizon. This is an architectural characteristic of the
formulation, not a bug. See `README.md`'s state-vector section (search "e_v's
target speed is frozen for the whole horizon") for the full explanation; not
repeated here to avoid duplication.

## Exit-heading boost: superseded by the corner-factor scheduler

The old `_lookahead_exit_boost`/`_update_lookahead_peak`/`dist_since_peak`
mechanism (which boosted `Q[2,2]` for a decaying window after a corner's
peak curvature, to help the car straighten out on exit) no longer exists on
either side. It was replaced by the corner-factor scheduler — see
"Corner-factor scheduler — what replaced the lookahead gain-scheduling
family" above for the current mechanism, and
[`removed_mechanisms.md`](logs/removed_mechanisms.md) for what was removed.

For the history of the exit-boost mechanism itself — the timing bug where
its decay clock was keyed on lookahead-window peak curvature instead of the
car's own physical apex (causing it to decay to a no-op before the car
reached the corner exit), the follow-up fix making the decay window
speed-scaled instead of a fixed 5 m, and a rejected `R[1,1]`/`Q[4,4]`
heading-misalignment accel gate — see `docs/logs/late_turn_in_investigation.md`,
"Addendum (2026-08-11): exit-heading boost was firing at the wrong time".

## Accel effort weight (superseded by accel/brake split)

The MPC's acceleration-effort weight (`R_diag[1]` / `MPCParams.r_a`) is no longer a single scalar applied symmetrically to accel and brake — it was replaced by independent accel/brake weights (see "Accel/brake effort weight split" below). Do not reintroduce a single shared `r_a` scalar without accounting for why it was split: a shared weight that is loose enough to accelerate well on straights is also too loose on braking, and vice versa.

Historical tuning path and full measurements: `docs/logs/sim_to_real_investigation.md` § 59 ("MPC underaccelerating on clean straights: `r_a` swept 0.85 → 0.77").

## Accel/brake effort weight split

The MPC's acceleration-effort cost is split into two independent weights
rather than one symmetric `R_diag[1]` scalar: `r_a_accel` penalizes
`a_cmd >= 0` and `r_a_brake` penalizes `a_cmd < 0`, via
`R_a_accel * sum(pos(a_cmd)²) + R_a_brake * sum(neg(a_cmd)²)` in the QP cost
(`cp.pos(u[1,:])`/`cp.neg(u[1,:])`). This lets braking and acceleration
effort be tuned independently instead of forcing both to move together
whenever `R_diag[1]` is retuned.

**Implemented in:**
- **Live**: `mpc_core.py`'s `_build_qp`/`_solve_qp` (`r_a_accel_param`/
  `r_a_brake_param` `cp.Parameter`s), `mpc_params.py`'s `r_a_accel`/
  `r_a_brake` fields, `fsae_params.yaml`'s `controller.r_a_accel`/
  `controller.r_a_brake`, `launch_all.sh`'s `MPC_R_A_ACCEL`/`MPC_R_A_BRAKE`
  shortlist entries.
- **Offline**: `controller/optimiser.py`'s `init_parameterized_mpc`/
  `solve_mpc` (same `cp.pos`/`cp.neg` split; `r_a_accel`/`r_a_brake` kwargs
  default to `R[1,1]` when omitted, for backward compatibility),
  `settings.py`'s `R_A_ACCEL`/`R_A_BRAKE` (read by `sim/rollout_core.py`'s
  `solve_mpc()` call). `R_diag[1]`/`self.R[1,1]` remain nominal/reporting
  values only — no adaptive gain (`_adaptive_R_scaling`, `_adaptive_R_rate`,
  etc.) touches index 1 of `R`/`R_rate` anywhere in this codebase, so the
  split composes cleanly with the rest of the adaptive-gain machinery.

**Re-check `mpc_params.py`'s `r_a_accel`/`r_a_brake` and `settings.py`'s
`R_A_ACCEL`/`R_A_BRAKE` for the current numeric defaults before relying on
any value quoted elsewhere** — these two remain the most frequently
live-retuned weights in the whole `MPCParams` set. Keep `settings.py`
synced to `mpc_params.py`'s live values per CLAUDE.md's parity rule whenever
either changes.

For the full diagnosis history (the corner-entry-too-hot symptom that
motivated the split, the slack-variable design considered and rejected in
favor of the `cp.pos`/`cp.neg` rewrite, and the live-tuning value
trajectory), see `docs/logs/late_turn_in_investigation.md`'s "Part 0
(background) — how the accel/brake effort split (`r_a_accel`/`r_a_brake`)
came about" and `docs/logs/sim_to_real_investigation.md` §59 for the
preceding single-scalar `r_a` cut this split superseded.

## Low-speed steering-rate boost (removed)

A mechanism that scaled `R_rate[0,0]` up at low speed (`_low_speed_steer_rate_boost`, `boost_max=2.5, k=0.35`) was tried, live-tested, and disabled the same day for regressing turn-in; it no longer exists in either codebase at all, having been removed along with the rest of the lookahead gain-scheduling family when the corner-factor scheduler replaced it. See "Corner-factor scheduler" above for what replaced it and the current mechanism, and `docs/logs/late_turn_in_investigation.md`'s "Appendix — Low-speed steering-rate boost: full incident" for the full incident history.

## Steering-effort relaxation approaching a corner

`_adaptive_R_scaling`'s speed-dependent steering-effort penalty (`R[0,0] *=
1 + 1.5*vx/(6+vx)`, e.g. ~2.07x at 15 m/s) has no lookahead relief built
into it — it stays at full strength through an approaching corner
regardless of curvature. `_steer_effort_straight_boost`, the only other
mechanism touching `R[0,0]`, only ever raises it on a clear straight or
relaxes back to the unscaled baseline as a corner is detected; neither
pushes `R[0,0]` below baseline for an approaching corner. Without a third
mechanism, a car entering a corner hot pays the full speed-based
steering-effort penalty at exactly the moment it most needs to commit to
turn-in. (`_lookahead_yaw_rate_relax` is the equivalent relief for
`Q[3,3]`/yaw-rate — its docstring names "turns late/slowly" as the failure
mode it guards against.)

`_lookahead_steer_effort_relax(kappa_max_abs, car_speed, floor=0.5, ...)`
is the `R[0,0]` counterpart: it mirrors `_lookahead_yaw_rate_relax`'s shape
exactly (same demand-normalised corner-severity curve), falling from `1.0`
(no corner ahead) toward `floor=0.5` as corner demand rises, composing
multiplicatively with `_adaptive_R_scaling` and
`_steer_effort_straight_boost`'s existing `R[0,0]` scalings — the only
mechanism in the `R[0,0]` chain that pushes below baseline approaching a
corner. `floor=0.5` matches `_lookahead_yaw_rate_relax`'s own default floor
(same magnitude as the sibling mechanism it's modelled on).

Fields: `mpc_core.py`/`mpc_params.py`
(`lookahead_steer_effort_relax_enabled`,
`adaptive_q_lookahead_steer_relax_floor`); mirrored in `model_utils.py`
(`lookahead_steer_effort_relax`) / `sim/rollout_core.py` / `settings.py`
(`LOOKAHEAD_STEER_EFFORT_RELAX_ENABLED`,
`ADAPTIVE_Q_LOOKAHEAD_STEER_RELAX_FLOOR`), and the `fsds_simulator` mirror.
See `docs/logs/late_turn_in_investigation.md`'s "Part 0c" for the original
diagnosis and the full `R[0,0]`/`R_rate[0,0]` mechanism inventory it sits
within.

## Curvature-forcing term: a rejected approach to blind path-bending prediction

The QP's dynamics model (`Ad`/`Bd`) has no path-curvature term, so with
`e_y ≈ e_psi ≈ 0` on a straight approach its own predicted rollout stays
near zero regardless of how sharply the real path bends ahead — no
reweighting of an *existing* tracking error (`adaptive_Q_lookahead`,
`lookahead_steer_effort_relax`, etc.) can compensate, since there is no
predicted error yet for a cheaper weight to act on.

A forcing term (`curvature_forcing_enabled`/`curvature_forcing_gain`) was
built to inject predicted curvature directly into the dynamics constraint
(`w[2,k] = -v_x·κ(s_k)·dt·gain`) so the QP's own rollout would anticipate
the bend. It is **structurally unsound and disabled**: because the term
perturbs the same recursion the QP minimizes cost over, the solver is free
to choose *how* to spend the disturbance across the horizon, and at any
gain large enough to matter it commits to a transient steer *away* from the
corner before correcting — reproduced in a clean, noise-free synthetic QP
test across a full gain sweep, not a live-noise artifact.

**Do not re-enable `curvature_forcing_enabled` by flipping the flag alone.**
A future redesign should shift the *reference*/error definition (curve the
heading `e_psi` is measured against) rather than perturb the QP's own
dynamics recursion — this is the direction the later NMPC formulation takes,
where curvature enters as a function of a state the solver actively chooses
rather than external data it can defer absorbing.

See `docs/logs/late_turn_in_investigation.md`'s "Part 6b" for the full
derivation, the synthetic verification, the anti-hunt interaction
(`anti_hunt_k_lookahead`, settled at `15.0`), and the gain-sweep evidence
behind the structural-unsoundness finding.

## Lookahead corner-anticipation window (approach side)

`adaptive_q_lookahead_dist_max` is `25.0` on both sides (`fsae_params.yaml`,
`fsds_simulator` mirror), matching `adaptive_q_lookahead_exit_decay_dist_max`
(also `25.0`) — the approach-side and exit-side ceilings are symmetric.
`sim/rollout_core.py`'s hardcoded literal is `(1.13, 3.0, 25.0)`
(`clip(v * adaptive_q_lookahead_time_s, 3, 25)`), kept as a plain literal
rather than sourced from a new `settings.py` constant, since `fsae_MPCTest`
and `fsae_planning` must never share an import dependency in either
direction.

`adaptive_q_lookahead` (§4.4 in `tuning.md`) only reweights `Q[0,0]`/`Q[2,2]`
on an *existing* tracking error — with `e_y ≈ e_psi ≈ 0` on approach, a wider
window lets the boost detect the corner's curvature sooner, but multiplying a
still-near-zero error by a bigger number is still near-zero. This is the same
"reweighting cannot manufacture an error" ceiling documented in the
curvature-forcing postmortem elsewhere in this doc and in
`junior_project_mpc_docs.md`: the boosts (and downstream steering commitment)
can only trigger sooner and larger once real error/curvature already appears
inside the window, not before.

A genuine fix for before-any-error anticipation on a precomputed path would
need to change what the reference/error is measured against — e.g. compute
`e_psi` against a lookahead point on the path rather than the nearest point —
not reweight costs on the current-position error. No design or
implementation exists for this. See `docs/logs/late_turn_in_investigation.md`
Part 14 for why even the 25m ceiling is still short of some corner-approach
gaps on real tracks, and for the candidate fixes considered.

## Straight-line lateral-error snap-back sharpness

`_lookahead_straight_lateral_reduce` softens `Q[0,0]` (lateral-error cost) to
`ey_floor=0.7` on a clear straight, then fades back to full weight as
curvature enters the lookahead window, at a rate set by
`adaptive_q_straight_ey_k` (currently `8.0`, matching `adaptive_q_straight_k`
— the shared fade sharpness for the `Q[2,2]`/`Q[3,3]` straight-line boosts).
The straight-line relaxation benefit itself (`ey_floor=0.7`) is independent
of this and unchanged by it — `adaptive_q_straight_ey_k` only controls how
quickly full lateral weight is restored once a corner is detected ahead.

Do not raise `ey_floor` as a way to address slow snap-back — that reduces
the straight-line-hunting benefit this mechanism exists for and changes
behaviour even far from any corner, which is the wrong lever. If
straight-line hunting reappears, `adaptive_q_straight_ey_k` and the
`ey_floor`/`adaptive_q_straight_ey_k` field comments in
`mpc_params.py`/`settings.py` are the first things to check.

## Gradual-corner accel oscillation is genuine track geometry, not a bug

Through mild/gradual turns (e.g. S-curves/chicanes), `v_desired` and `a_cmd`
can legitimately oscillate — rising to a local peak, dipping, rising again —
because the precomputed speed profile (`speed_profile.csv`) is tracking a
genuine sign change in the path's own curvature (a left-hand bend
straightening briefly before curving right into the next bend), not
adaptive-gain misbehaviour or spurious jitter. The `kappa_max_abs`-driven
lookahead correctly speeds the car up through the brief straight and slows
it back down anticipating the next corner.

Before treating this pattern as a bug, cross-reference the car's actual
`v_desired`/`a_cmd` against the track's own `speed_profile.csv` and the raw
path's Menger curvature at the same position — a genuine varying-radius
feature (sign change in curvature, not just magnitude) confirms the
oscillation is correct tracking, not noise.

**Do not "fix" this via `R_rate_diag[1]`** (acceleration-rate-of-change
cost) or similar damping — that makes the MPC slower to respond to a real,
upcoming tightening corner, trading a correctly-anticipated slowdown for a
late, harder one. If oscillation on a specific track needs addressing, the
correct levers are the speed profile's own generation parameters
(`a_lat_max`, scan window in `sim/speed_profile.py`'s
`compute_speed_profile()`) or the raw path geometry itself (smoothing an
actually-spurious kink) — never the live adaptive gains. Confirm the
geometry is genuinely spurious (not real track shape) before touching
either lever; see `late_turn_in_investigation.md`'s "Gradual-corner accel
oscillation" section for the full worked example of how to distinguish the
two.

## Dynamic speed cap

`control_utils.dynamic_speed_cap()` is a thin wrapper over
`curvature_speed()` (see that function's own docstring for the scan/
braking-distance mechanism) that runs *underneath* the precomputed oracle
speed profile: whenever a track is mapped, the controller target speed is
`min(precomputed_speed_at(...), dynamic_speed_cap(...))`. The oracle profile
(`precomputed_speed_at()`, used when `USE_PRECOMPUTED_SPEED_PROFILE=True` /
`map_path` is set) is a static, position-indexed lookup with no notion of
the car's actual current speed relative to remaining braking distance; the
dynamic cap exists to catch the case where live tracking has drifted ahead
of the oracle plan (e.g. exiting the previous corner faster than expected)
and pull the target speed down before a corner is reached, never up.

It uses its own, tighter constants than the live-`curvature_speed()`-only
branch's numeric-parity defaults: `DYNAMIC_CAP_A_LAT_MAX=3.2` and
`DYNAMIC_CAP_SAFETY=0.9` (vs. `a_lat_max=4.0`/`safety=1.0` elsewhere), so it
engages a little before the oracle profile would actually be violated.
Downstream, `tracking_error_speed_gate()` and `SPEED_TARGET_RISE_RATE` apply
exactly as before — the cap only changes what `v_curv` feeds into that
existing pipeline. It has no effect when no track is mapped, and is
byte-identical to pre-cap behaviour when disabled.

Controlled by `enable_dynamic_speed_cap` (ROS param) /
`ENABLE_DYNAMIC_SPEED_CAP` (`settings.py`) — code default `True`, but
**`ros2/launch_all.sh`'s MPC tuning shortlist currently overrides this to
`false`**, so a plain `launch_all.sh` run has the cap off; check that
shortlist rather than assuming the code-level default is what actually
drives. Do not re-enable for a live run without first understanding why it
regressed steering saturation and heading error offline (see
`docs/logs/late_turn_in_investigation.md`'s "Dynamic speed cap" addendum for
the measured before/after and the live-test result) — the a_lat-ceiling
metric it targets improved, but the metrics that matter more got worse, for
reasons never diagnosed.

## Resync procedure

1. Clone/update the sibling `fsae_planning` repo checkout used as the sync
   source (gitignored in this repo, e.g. at `fsae_planning/` in the repo
   root).
2. For each file in the mapping table above, read **both** the old (already
   ported) version and the new upstream version in full before porting
   anything — don't diff-and-patch blind. The one mechanical, unavoidable
   difference: upstream's `planning/` files import each other as
   `from fsae_planning.xxx import yyy` (their ROS 2 package is named
   `fsae_planning`); this repo's package is named `planning`, so every ported
   file needs its intra-package imports rewritten from `fsae_planning.xxx` to
   `planning.xxx` — a one-line-per-import search/replace, not a content
   change. Everything under `fsds_simulator/` has no such rewrite to do: it's
   a byte-for-byte staging mirror of upstream's own package hierarchy (every
   row in the file mapping table above is a "Direct mirror"), so those files
   are already a literal copy-paste at the matching path.
3. Port algorithm changes only, preserving this repo's existing import style
   (for the root `planning/` folder) and the deliberate non-mirrors listed
   above (don't restore a `.v_profile` on `SimPlanner`). `fsds_simulator/`
   mirrors `mpc_controller.py`, `mpc_controller_standalone.py`, and
   `stanley_controller.py` — all three current nodes, not just one — see
   "Two MPC-controller nodes, plus Stanley" above.
4. If the change touches `planning/` or `mpc_core.py`, check per `CLAUDE.md`'s
   numeric-parity rule whether `sim/rollout_core.py` needs a mirrored change —
   `rollout_core.run_core_rollout()` and `mpc_core.MPCController` are two
   implementations of the same control loop kept in deliberate numeric
   parity. Call this out explicitly in the resync notes if a mirrored change
   is or isn't needed.
5. Re-check the numeric-parity constants table above — if upstream changed
   `curvature_speed()`'s `a_lat_max` or the planner speed clamp values, both
   the offline (`sim/speed_profile.py`, `sim/rollout_core.py`) and live
   (`control_utils.py`, `mpc_controller_standalone.py`) copies need the same
   update.
6. Run the smoke-test pattern from `CLAUDE.md`'s Testing section: confirm
   changed files import cleanly, then run `python -m gui.simulation` (or a
   short `python -m tuner.offline_tuner` run with `FAST_TEST_MODE = True` in
   `settings.py`) against one synthetic path and check the rollout still
   converges and tracks correctly. There is no way to test the
   `fsds_simulator/` mirror's ROS 2 files against the real/FSDS car from this
   repo directly — reason through the change against `sim/rollout_core.py`
   instead and flag it for live testing by a human once actually pasted into
   `fsae_planning`.

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

## Precomputed corner segmentation (removed)

*(Historical: a precomputed per-waypoint `CornerMap` once replaced the live corner-anticipation scan with an exact index lookup for static paths. It was part of the ~15-mechanism lookahead gain-scheduling family removed wholesale by the corner-factor rewrite — see [`Corner-factor scheduler`](#corner-factor-scheduler-what-replaced-the-lookahead-gain-scheduling-family) above for what replaced it, and [`removed_mechanisms.md`](removed_mechanisms.md)'s "7. Precomputed corner segmentation (`CornerMap`)" for the mechanism-level summary. Full implementation history in `docs/logs/late_turn_in_investigation.md` Parts 3-6.)*

## Precomputed shaped heading-lead profile

A derived, offline-only third pass over an already-optimized raceline that
replaces the geometric heading reference (`atan2` of the path tangent) with
a SHAPED one: at each waypoint, the heading target leads the geometric
tangent by however much yaw the car can physically achieve between here and
the upcoming bend at that station's already-planned speed. This changes
`e_psi` at `k=0` itself — before the QP ever runs — rather than adding a
future-deviation cost the QP is free to satisfy however is cheapest (the
failure mode of both curvature-forcing and a cost-target shift, both
rejected). It also does not use a fixed lookahead distance (which saturates
steering to full lock at ~8m of lead on a realistic corner-entry ramp);
instead the lead is scaled by achievable yaw rate at the station's planned
speed, which naturally decays it to ~0 once a corner's constant-curvature
section begins.

**Where it lives:**
- Offline: `fsae_MPCTest/tuner/tools/raceline_optimizer.py` — `max_yaw_rate`,
  `build_shaped_heading_profile`, `check_slip`, run in `export()` after
  `optimize_raceline()`'s path+speed have converged. Does not feed back into
  path/speed optimization (kept as a separate derived pass to avoid
  invalidating that loop's own tuned constants).
- CSV format: `x,y,psi,psi_target,v_target` (5 columns), backward-compatible
  with the old 4-column format — `control_utils._load_profile_csv` sets
  `psi_target = psi` for any 4-column file.
- Live: `mpc_core.py`'s `MPCController.set_heading_profile(psi_target)`
  substitutes the shaped value for `path_yaw` at `base_idx`, only for
  `e_psi`'s reference (`e_y`'s projection keeps using the geometric
  tangent). Loader: `control_utils.load_path_heading_profile_csv`.
- Toggle: `use_precomputed_heading_profile`, a node-level launch parameter
  (not an `MPCParams` field), exposed as `USE_PRECOMPUTED_HEADING_PROFILE`
  in `ros2/launch_all.sh`. Current default: `false` on both nodes, both
  launch files, and in `launch_all.sh`.
- `use_precomputed_heading_profile` has no effect when `use_nmpc` is active
  — the NMPC model already carries curvature directly.

**`check_slip`'s `SLIP_LIMIT_RAD` (5°) is an unvalidated placeholder** —
diagnostic-only, does not fail the export or reshape anything. Do not treat
it as authoritative until it has a real measurement behind it.

**Standing warning:** do not extend this mechanism from a k=0-only lead to
a full per-horizon-step reference — that reproduces curvature-forcing's
same wrong-direction-dip trap (see
`docs/logs/late_turn_in_investigation.md` Part 15).

For the full derivation, the rejected fixed-lookahead-distance and
cost-target-shift alternatives, the `comp_test_map_3`-specific
near-everywhere-lead caveat, and both rounds of live-test results
(an initial "worse" read followed by a high-variance correction across
more runs), see `docs/logs/late_turn_in_investigation.md` Parts 7-13.

## Nonlinear MPC (`use_nmpc`) — a second controller

`ros2/src/fsae_planning/control/fsae_control/fsae_control/nmpc_core.py`'s
`NMPCController` is a Frenet-frame **nonlinear** MPC (Gauss-Newton SQP, a
condensed dense QP subproblem solved by OSQP, real-time-iteration style — one
SQP iteration per tick, warm-started from the previous tick). It has an
independent offline port, `controller/nmpc_optimiser.py`'s `NMPCController`
(same model, same SQP/OSQP scheme, not an import — the two repos cannot
import each other), wired into `sim/rollout_core.py`'s `run_core_rollout()`
behind `settings.USE_NMPC` (default false). It is selected live by the node
parameter `use_nmpc` (default **false**) and replaces `MPCController`
wholesale when true; `mpc_core.py` is untouched either way.

### Why it exists

`MPCController._discrete_model` is the bicycle model in error coordinates
with the reference frame's own rotation dropped — it is missing
`e_psi_dot = r - kappa(s) * s_dot` from its `Ad`/`Bd`, so with `e_y = e_psi =
0` the QP's rollout predicts staying at zero error forever and no weighting
can produce turn-in before real error exists (`MPCController` measurably
commands 0.000 deg at 8 dead-on-line states approaching a known bend). The
Frenet formulation makes `kappa` a function of the **state** `s` (driven by
the car's own predicted motion) rather than a horizon-indexed exogenous
schedule, so the anticipation obligation is not schedulable and the solver
cannot pre-pay it early the way the curvature-forcing and heading-lead
workarounds do. Full derivation, the formulation survey (including why MPCC's
progress-maximising formulation was not adopted wholesale), and the falsification
methodology are in `late_turn_in_investigation.md` Part 16.

### Model and what it reuses

States `[s, e_y, e_psi, v_x, v_y, r, delta_act, a_act]`, inputs `[delta_cmd,
a_cmd]`. Every vehicle constant (`lf` 0.70, `lr` 0.85, `m` 255, `Iz` 150,
`Cf`/`Cr`, `tau_delta` 0.08, `tau_a` 0.02, `MAX_STEER_RAD`, `MAX_ACCEL`,
`MAX_BRAKE`, `du_max`) and the kinematic/dynamic blend band (1.0–2.5 m/s)
come from `MPCController.__init__` unchanged — no new physical constant.
Cost weights come from the same `MPCParams` instance the LTV-QP uses. Three
deliberate differences from the LTV-QP:

1. **`q_r` weights heading-error RATE** (`r - kappa*s_dot`), not absolute yaw
   rate — same slot, different regressor from the LTV-QP's `q_r`.
2. **`e_y`/`e_psi` are measured against the smoothed reference**, not the raw
   segment tangent (bounded by the same 1.5 m smoothing window as
   `control_utils.curvature_speed()`'s `dense_step=0.5`/`w=3`).
3. **FSDS's measured `a_lat` ceiling is inside the prediction**, as a smooth
   `tanh` saturation of predicted tyre forces, reusing the same numeric law
   (flat/slope/intercept = 7.5/0.47/2.46) as `mpc_core._alat_ceiling_at` /
   `model/vehicle_physics.alat_ceiling_at`, hardcoded as class-level defaults
   on `nmpc_core.py`'s `_Plant` rather than read from `MPCParams`.
   `nmpc_alat_ceiling_enabled=false` recovers the unconstrained plant,
   mirroring `VehicleParams.alat_ceiling_enabled`.

Tyre lateral force (`F_yf`/`F_yr`, from `alpha_f = atan((v_y + lf*r)/v_safe) -
delta_act`) is scaled by the same kinematic/dynamic `blend` factor used
elsewhere in the model, applied immediately after the force is computed and
after the `alat_ceiling` soft saturation — without this scaling the slip-angle
floor at low `v_x` lets steering alone manufacture lateral force with zero
forward speed. This applies in all three copies: `nmpc_core.py` and
`controller/nmpc_optimiser.py`'s `_f`, `_f_scalar`, and its
`nmpc_friction_circle_enabled`-only `_tyre_forces()` helper.

### What is inactive when `use_nmpc=true`

- The entire adaptive gain schedule (lookahead approach/exit boosts,
  yaw-rate relax, straight boosts, centred softening, U-turn detector) — no
  `m_*` telemetry columns are written for these.
- `use_precomputed_heading_profile` has no effect (one startup log line says
  so) — the NMPC's curvature model already carries what that flag
  approximates.
- `curvature_forcing_enabled`, `ref_heading_rate_limit_enabled` are
  LTV-QP-only.
- `use_precomputed_corner_map` no longer exists on either `use_nmpc` setting
  (removed by the corner_factor rewrite; see `architecture.md`'s
  "Precomputed corner segmentation" note).

**Exception: `nmpc_steer_rate_anti_hunt_enabled`** (default `False`,
`mpc_params.py`'s `nmpc_steer_rate_anti_hunt_enabled`/
`nmpc_anti_hunt_boost_max`) is an independent NMPC-only opt-in, not inherited
from the LTV-QP's `steer_rate_anti_hunt_enabled` — unlike the rest of the
adaptive-gain family, it only makes steering-rate more expensive when the
current state is already centred/aligned/uncurving, the opposite direction
from anticipation. Reuses `mpc_core._steer_rate_anti_hunt` verbatim on the
live side and `model_utils.steer_rate_anti_hunt` offline. Writes the existing
shared `m_Rrate_antihunt` telemetry column when on. See `mpc_params.py`'s
field comment and `nmpc_core.py`'s module docstring ("WHAT THIS CONTROLLER
DELIBERATELY DOES NOT DO") for the reasoning.

`use_precomputed_path` / `use_precomputed_speed` / `enable_dynamic_speed_cap`
/ delay compensation (`delay_compensation_enabled`, `pose_age_lp_alpha`,
`n_delay_hysteresis`, `max_delay_compensation_steps`) all work exactly as
under the LTV-QP; the delay rollforward uses the nonlinear model instead of
`predict_ahead()`'s linearisation.

### Three MPCC-inspired additions

Assessed against Alexander Liniger's Model Predictive Contouring Control
(`https://github.com/alexliniger/MPCC`). MPCC's headline idea — progress `θ`
as a free variable the solver maximises — was not adopted: it reintroduces
the "exogenous, schedulable future obligation" failure mode this NMPC's
`kappa(s)`-as-state design exists to avoid. Three narrower ideas were kept,
all NMPC-only and implemented identically in `nmpc_core.py` (live) and
`controller/nmpc_optimiser.py` (offline):

1. **Spline-based path reference** — `nmpc_spline_reference_enabled` /
   `NMPC_SPLINE_REFERENCE_ENABLED`, default **true**. `PathReference` fits
   `x(s)`/`y(s)` as independent `scipy.interpolate.CubicSpline` objects over
   arc length and derives `kappa(s)`/`psi_ref(s)` analytically
   (`kappa = (x'y'' - y'x'') / (x'^2+y'^2)^1.5`), replacing the old
   dense-resample + moving-average + finite-difference pipeline. A strict
   numerical-quality improvement with no new solver coupling — the old
   moving-average path is kept behind the flag for A/B if needed. Directly
   targets the open "centreline curvature spikes" defect noted in CLAUDE.md.
2. **Horizon speed profile** — `nmpc_horizon_speed_profile_enabled` /
   `NMPC_HORIZON_SPEED_PROFILE_ENABLED`, default **false**. Would sample a
   precomputed per-lap speed profile at each horizon stage's own predicted
   arc length via `PathReference.v_ref_at(s)`, instead of holding `v_ref`
   constant across the horizon. **Do not enable without first bounding how
   far ahead along `s` the sampled `v_ref` may rise** (or an equivalent
   per-stage clamp) — summing `v_x - v_ref(s_k)` across all horizon stages
   lets a high `v_ref` at a later stage offset a low `v_ref` at an earlier
   one within the same solve, defeating the non-schedulability property this
   feature was meant to inherit from `kappa(s)`. See
   `late_turn_in_investigation.md` Part 16 §16.9 for the live-test evidence
   behind this warning.
3. **Friction-circle hard constraint** — `nmpc_friction_circle_enabled` /
   `NMPC_FRICTION_CIRCLE_ENABLED`, default **false**. Would add a hard
   `|F_yf|, |F_yr| <= F_max` bound (additional to, not replacing, the
   existing soft `tanh` saturation) with `F_max` derived from the same
   measured ceiling law. **Do not enable without first fixing
   `telemetry_logger.py`'s `NMPC_COLUMNS`** (missing `nmpc_fyf_max_abs`/
   `nmpc_fyr_max_abs`) **and re-deriving a looser `F_max`** — the hard bound
   has no slack variable, and ordinary cornering geometry on this track
   conflicts with `F_max = m * ceiling(v_x) / 2` per axle under completely
   normal driving, not just extreme conditions. See
   `late_turn_in_investigation.md` Part 16 §16.9 for the failure evidence.

Neither feature 2 nor 3 has offline A/B numbers; reproduce a comparison with
`python -m tuner.nmpc_offline_check` once one exists.

### Dependencies

`osqp` only — already a documented requirement of `mpc_core` via cvxpy
(`fsae_control/package.xml`). CasADi/acados are **not** used by the shipped
code (not installable into the ROS interpreter on Ubuntu 24.04 without
`--break-system-packages`); the SQP and its Jacobians are numpy-only. CasADi
was used once, from a private `--target` install, purely to cross-check the
optimum against IPOPT.

### Which settings affect which controller

Every `MPCParams`/`NMPCParams` field carries an explicit
`metadata["controller"]` tag (`"both"`, `"ltv_qp_only"`, or `"nmpc_only"`) —
see `mpc_params.py`/`nmpc_params.py` for the authoritative per-field value.
`settings.py` mirrors the same three-way classification as a
`[LTV-QP only]`/`[NMPC only]`/`[shared]` comment prefix on each constant, and
`ros2/launch_all.sh` tags its shortlists the same way.

**Shared base weights** (`mpc_params.py:47-67`, read by both controllers
through `nmpc_core.py`'s `_pick(override, inherited)` at lines 859-877): `q_e_y`,
`q_e_yd`, `q_e_psi`, `q_r` (meaning differs — see above), `q_e_v`, `r_delta`,
`r_a_accel`, `r_a_brake`, `r_rate_delta`, `r_rate_a`, `terminal_q_scale`. A
launch with every `nmpc_q_*`/`nmpc_r_*` override left at its `-1.0` sentinel
starts the NMPC from the LTV-QP's own tuned set exactly.

**Shared delay-compensation fields** (`mpc_params.py:73, 80, 84-85`):
`delay_compensation_enabled`, `max_delay_compensation_steps`,
`pose_age_lp_alpha`, `n_delay_hysteresis` gate/shape both sides identically
(mechanism differs: NMPC rolls `x0` through the nonlinear model instead of
`predict_ahead()`'s linearisation). `predict_epsi_clip`
(`mpc_params.py:81`) is LTV-QP only — specific to `predict_ahead()`'s linear
rollforward.

**LTV-QP-only, the adaptive-gain-schedule fields** (`mpc_params.py:70-72, 74,
77, 81, 88, 91, 101-161`): `adaptive_q_scaling_enabled`,
`steer_rate_anti_hunt_enabled`, `adaptive_r_rate_enable_in_corners`,
`ref_heading_rate_limit_enabled`, `ref_heading_rise_rate_deg_s`,
`adaptive_r_rate_during_floor`, `anti_hunt_boost_max`, `corner_factor_k`,
`q_ey_straight`/`q_ey_corner`, `q_epsi_straight`/`q_epsi_corner`,
`q_r_straight`/`q_r_corner`, `rrate_steer_straight`/`rrate_steer_corner`,
`r_steer_corner_mid`, `low_speed_corner_boost_v_half`/`_max_extra`,
`epsi_ra_half_rad`/`_accel_boost_max`/`_brake_floor` — none have any read
site in `nmpc_core.py`.

**NMPC-only overrides** (`mpc_params.py:189-205, 222-223`): `nmpc_q_e_y`,
`nmpc_q_e_yd`, `nmpc_q_e_psi`, `nmpc_q_epsi_dot` (overrides `q_r`, different
regressor), `nmpc_q_e_v`, `nmpc_r_delta`, `nmpc_r_a_accel`, `nmpc_r_a_brake`,
`nmpc_r_rate_delta`, `nmpc_r_rate_a`, `nmpc_terminal_scale`,
`nmpc_steer_rate_anti_hunt_enabled`, `nmpc_anti_hunt_boost_max` — each read
solely by `nmpc_core.py`'s `_pick()` calls; `mpc_core.py` never references any
of them.

**`NMPCParams`** (`nmpc_params.py`) — all 20 fields NMPC-only by the file's
own design (module docstring, lines 9-19); `mpc_core.py` never imports or
reads this dataclass at all. Includes the master switch `use_nmpc` itself,
horizon/solver settings (`nmpc_horizon`, `nmpc_sqp_iters`,
`nmpc_solve_budget_ms`, `nmpc_rk_substeps`, `nmpc_jac_substeps`), SQP step
control (`nmpc_trust_delta_rad`, `nmpc_trust_a`, `nmpc_backtrack_max`), the
soft track constraint (`nmpc_track_halfwidth`, `nmpc_slack_weight`),
curvature-reference construction (`nmpc_curvature_dense_step`,
`nmpc_curvature_smooth_w`, `nmpc_kappa_clip`), `nmpc_alat_ceiling_enabled`,
the three MPCC-inspired flags above, and solver tolerances
(`nmpc_osqp_max_iter`, `nmpc_osqp_eps`).

**`settings.py`-only, no dataclass field** (rollout/plant configuration, not
`MPCController`/`NMPCController` fields): `USE_PRECOMPUTED_SPEED_PROFILE`,
`ENABLE_DYNAMIC_SPEED_CAP` and its `DYNAMIC_CAP_*` constants,
`DELAY_STEPS`/`DELAY_JITTER_*`, `ALAT_CEILING_FLAT`/`_SLOPE`/`_INTERCEPT`.

Structural/solver constants (`NMPC_HORIZON`, `NMPC_SQP_ITERS`, etc.) live in
`settings.py`'s "Nonlinear MPC (NMPC)" section, kept numerically identical to
`NMPCParams` by hand. See `docs/tuning.md`'s NMPC section for the tuning
surface, and `late_turn_in_investigation.md` Part 16 for the full research,
implementation, and validation history, extended by §16.9-16.11 with the
live-test results, the MPCC-feature live tests, and the standstill-steering
bug and fix. Reproduce the offline closed-loop comparison with
`python3 ros2/src/fsae_planning/control/fsae_control/test/nmpc_offline_check.py`
(no ROS/FSDS session needed; the closed-loop section self-skips without an
`fsae_MPCTest` sibling checkout) or `python -m tuner.nmpc_offline_check`.
