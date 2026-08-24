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
| `curvature_speed()`'s `a_lat_max` | `sim/speed_profile.py` (`CURVATURE_SPEED_A_LAT_MAX` + function default) | `fsds_simulator/control/fsae_control/fsae_control/control_utils.py:194` (function default) | `4.75` |
| Planner top/bottom speed clamp | `sim/rollout_core.py:67-68` (`PLANNER_V_MAX`, `PLANNER_V_MIN`) | `fsds_simulator/control/fsae_control/fsae_control/mpc_controller_standalone.py` (declared as the `v_max`/`v_min` ROS parameters, default `20.0`/`1.5`) | `20.0` / `1.5` |
| Steering slew-rate limit (`du_max[0]`) | `model/vehicle_physics.py` (`VehicleParams.max_steer_rate`), applied as `max_steer_rate * DT` in `sim/rollout_core.py` and passed to `controller/optimiser.py`'s `du_max` | `fsds_simulator/control/fsae_control/fsae_control/mpc_core.py` (`MAX_STEER_RATE_RAD_S`, applied as `* self.dt`) | `radians(180.0)` rad/s |
| Accel slew-rate limit (`du_max[1]`) | `sim/rollout_core.py` (`du_max` second element) | `fsds_simulator/control/fsae_control/fsae_control/mpc_core.py` (`self.du_max` second element) | `0.6` per step |
| `tracking_error_speed_gate()` thresholds | `sim/speed_profile.py` | `fsds_simulator/.../control_utils.py` | `ey_lo/hi` 0.5/2.0 m, `epsi_lo/hi` 20/60 deg, `floor` 0.3 |
| Speed-target rise limit | `sim/rollout_core.py` (`SPEED_TARGET_RISE_RATE`) | `fsds_simulator/.../mpc_controller.py` and `mpc_controller_standalone.py` (same name) | `7.0` m/s² |
| `curvature_speed()` κ reduction | `sim/speed_profile.py` | `fsds_simulator/.../control_utils.py` | max of 3-point running mean |
| Score weights / bonuses / penalties | `settings.py` (`SCORE_WEIGHTS`, `COMPLETION_BONUS_WEIGHT`, `TIME_BONUS_WEIGHT`, `DNF_PENALTY`, `DNF_OFFTRACK_PENALTY`) | `fsds_simulator/control/fsae_control/fsae_control/scoring.py` (inlined as module constants) | weights sum to `1.0`; `0.5` / `0.25` / `3.0` / `3.0` |
| Metric normalisation scales | `settings.py` (`METRIC_SCALES`) | `fsds_simulator/control/fsae_control/fsae_control/scoring.py` (inlined as module constant) | 13 entries, `[0.40, 0.45, 0.30, 0.18, 1.50, 0.40, 0.02, 0.30, 1.00, 0.015, 0.70, 2.30, 0.08]` |
| Constrained-scoring constants | `settings.py` (`CONSTRAINT_FLOOR`, `COMPLETION_THRESHOLD`, `TIME_OBJECTIVE_WEIGHT`, `QUALITY_WEIGHT`) | `fsds_simulator/.../scoring.py` (inlined as module constants) | `10.0` / `0.98` / `1.0` / `0.35` |
| `A_BRAKE_PLAN` (braking-distance propagation in `curvature_speed`) | `sim/speed_profile.py` | `fsds_simulator/.../control_utils.py` | `5.0` m/s², positive magnitude |
| Dynamic speed cap enable/gains | `settings.py` (`ENABLE_DYNAMIC_SPEED_CAP`, `DYNAMIC_CAP_A_LAT_MAX`, `DYNAMIC_CAP_SAFETY`) | `mpc_controller.py`/`mpc_controller_standalone.py` (`enable_dynamic_speed_cap`/`dynamic_cap_a_lat_max`/`dynamic_cap_safety` ROS params) | `True` / `3.2` m/s² / `0.9` — see "Dynamic speed cap" section below |
| Latency telemetry columns | — (offline has no equivalent) | `fsds_simulator/.../telemetry_logger.py` | `pose_age_s`, `path_age_s`, `n_delay`, `solve_ms`, `cmd_latency_ms` |
| Pose-feed hold model | `settings.py` (`POSE_HOLD_*`) + `sim/rollout_core.PoseFeedHold` | — (offline-only; models a live fault) | `PROB 0.05`, `MEAN_TICKS 2.1`, `MAX_TICKS 5` |
| Accel/brake effort split (`R[1,1]`) | `settings.py` (`R_A_ACCEL`, `R_A_BRAKE`), read by `controller/optimiser.py`'s `solve_mpc(r_a_accel=, r_a_brake=)` | `mpc_params.py` (`r_a_accel`, `r_a_brake`), read by `mpc_core.py`'s `_solve_qp` | actively being live-tuned — re-check both sides' current values before trusting this row; see "Accel/brake effort weight split" below |
| NMPC rate-shaping family (zone / jerk / stage ramp / `k`) | `settings.py` (`NMPC_RRATE_ZONE_*`, `NMPC_RJERK_DELTA`/`_A`, `NMPC_RRATE_STAGE_*`, `NMPC_CORNER_FACTOR_K`), threaded through `sim/rollout_core.py` into `controller/nmpc_optimiser.py` | `mpc_params.py` (same names, lowercase), read by `nmpc_core.py` | zone `True` @ `2.0`/`0.80`/`0.15` (the intended `0.35` ease DNFs offline), `rjerk_delta` `150.0`, `rjerk_a` `0.0`, stage ramp `False`, `nmpc_corner_factor_k` `27.0`, `nmpc_q_e_y` `7.5` — **`k` is load-bearing for the zone, not cosmetic**: at the inherited `8.0` the ease/floor bands are unreachable on a track with `κ_max`~0.2, so a divergence here silently disables the schedule offline while leaving it on live. See "Three-zone rate schedule" below |
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
- `a_lat_max=4.75` is the value actually used (as each function's default,
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
| `q_r` | `Q_diag[3]` | `1.20` both sides — synced |
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

## Soft steering-reversal penalty (`reversal_penalty_*` / `nmpc_reversal_penalty_*`)

**Default off, experimental on both controllers.** Boosts `R_rate[0,0]`
(steering rate-of-change cost) whenever LAST tick's steering command was
already close to zero — the one state a sign-flip reversal must pass
through. A reversal itself can't be penalised directly inside one solve
(it depends on THIS tick's own decision, the thing being optimised, which
would make the cost non-convex); this penalises the precondition instead,
via a saturating curve `1/(1 + k*|u_prev_steer|)` keyed on `u_prev` (a known
constant by solve time, not the solve's own variable). Same shape as
`_steer_rate_anti_hunt`/`_corner_factor`, and composes multiplicatively with
whichever of those is active rather than replacing it.

**Implemented in:**
- **Live**: `mpc_core.py`'s `_reversal_penalty_boost` (LTV-QP), mirrored in
  `nmpc_core.py` (NMPC path, tracked through a running `rrate_steer_current`
  value alongside the corner-blend/anti-hunt branches so the penalty
  compounds onto whichever of those ran rather than the pre-branch base).
  Fields: `mpc_params.py`'s `reversal_penalty_enabled`/`_boost_max`/`_k`
  (LTV-QP-only) and `nmpc_reversal_penalty_enabled`/`_boost_max`/`_k`
  (NMPC-only overrides, `-1.0` = inherit the LTV-QP value).
- **Offline**: `controller/model_utils.py`'s `reversal_penalty_boost` (same
  function, both controllers call it), wired into `sim/rollout_core.py`'s
  LTV-QP and NMPC branches; `settings.py`'s `REVERSAL_PENALTY_*` /
  `NMPC_REVERSAL_PENALTY_*` constants (same `-1.0`-inherit convention).

**Validated on the LTV-QP path only.** Offline A/B on `comp_test_map_3`
showed a genuine ~20% reversal-count reduction at negligible cost, no DNF.
The NMPC path's own reversal penalty regressed the composite score
offline (reversals dropped only ~5.6% while score worsened) — leave
`nmpc_reversal_penalty_enabled` off unless a live A/B says otherwise; it is
wired and functional, just not currently worth enabling.

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
3. **Speed-target rise limiter** — `SPEED_TARGET_RISE_RATE = 7.0` m/s², applied
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

## The sim-to-real gap: a lateral-acceleration ceiling, partly closed

> **Full investigation history** — every hypothesis tried, why each looked
> right, and how it was eliminated — is in
> [`docs/logs/sim_to_real_investigation.md`](logs/sim_to_real_investigation.md).
> Read that before re-testing any candidate below; most already were.

On the recorded `comp test map 3`, same tuned gains both sides, the live car
saturates steering far more than the offline sim and carries a much larger
heading error:

| | offline sim | live car |
|---|---|---|
| steering saturation | 4.8% | **21.1%** |
| \|e_psi\| mean / p90 | 6.9° / 18.5° | **15.9° / 42.0°** |
| a_lat max | 11.24 | 12.34 |

When the car is at full lock it is pulling only ~4.1 m/s² lateral at ~5.7 m/s
— it is not cornering hard, it is rotating back from a large heading error.
Heading error arrives in sustained episodes (median ~0.5 s, up to 2.4 s, 96%
of energy below 1 Hz) — a stale/wrong reference, not high-frequency chatter.

**Candidates eliminated** (see the log for each measurement): plant grip too
generous, entering corners too fast, planner centreline quality, SLAM pose
noise, extra actuation delay, planner update rate, pose-feed hold, tyre
grip/understeer, `MAX_STEER_RAD` command scaling, actuator lag, yaw_rate/speed
telemetry error, tyre front/rear balance, FSDS's `SteeringCurve` UE4/PhysX
speed-dependent steering scaling (confirmed flat at 1.0, no scaling at all).

### Root cause: FSDS enforces a sustained lateral-acceleration ceiling

Below ~6 m/s the car delivers the commanded steering angle exactly; above it
the yaw response collapses, in a cliff rather than a gradual curve. This is
**not** tyre saturation — a grip limit doesn't depend on speed — and it engages
far below the ~12–14 m/s² lateral acceleration the same car/plant can reach
elsewhere. A step-input test shows the ceiling holds lateral acceleration
(not yaw rate) constant across speeds, and that it overshoots before settling
— so it is enforced by a term that takes time to build, not a hard clip.

Modelled in `model/vehicle_physics.py` (`alat_ceiling*`) as a restoring yaw
moment with a first-order lag, using a **leaky integral of the signed
excess** (not the proportional law first tried, which structurally cannot
pin the settled value at the ceiling for any gain):

| parameter | value | basis |
|---|---|---|
| `alat_ceiling` | 7.5 m/s² | measured settled lateral acceleration |
| `alat_ceiling_mode` | `'pi'` | leaky-integral law; a proportional law cannot hold a setpoint |
| `alat_ceiling_gain` | 450 N·m | fitted to measured peak; settled value falls out by structure |
| `alat_ceiling_tau` | 0.40 s | measured transient time constant |
| `alat_ceiling_enabled` | `True` | models **FSDS**, not the physical car — disable for real-vehicle work |

This closes part of the gap (the plant's lateral-acceleration distribution
now matches the car's) but **not the steering-saturation gap** — live still
saturates roughly 4× more often. The residual is narrowed to the *rate of
entry* into the high-heading-error state, not steady-state cornering
capability; the reference-heading lead is the next avenue and is testable
offline. See the log's §9-§13 for the full derivation, the two false starts
in the ceiling's control law, and the quantified ledger of what each tested
factor explains.

**Validated with:**

| tool | answers |
|---|---|
| `tuner/checks/steering_sysid_analysis.py`, `tuner/checks/steering_step_analysis.py` | what does FSDS do? |
| `tuner/checks/plant_openloop_validation.py` | does our plant reproduce it? (`--ab`, `--robustness`) |
| `tuner/recorded_map_rollout.py` | the closed-loop table above, headless and reproducible |
| `tuner/checks/live_vs_sim_diagnostics.py` | conditional + reference-heading decomposition of live vs sim |

Reproduce the closed-loop table with
`python -m tuner.recorded_map_rollout [--mode p --gain 700 | --tau 0.25 | --no-ceiling]`.

### The open-loop system-ID experiment (reusable methodology)

Isolating the plant from the controller — command fixed steering angles at
fixed speeds on an empty map and record the achieved yaw rate — is what
found the ceiling above. Reuse this whenever a plant-vs-car discrepancy is
suspected; a closed-loop lap log cannot separate a plant defect from a
controller/reference one.

**Where the pieces live** (the node and harness are working-tree-only files
in the live ROS 2 workspace, no mirror in this repo):

| file | repo | role |
|---|---|---|
| `control/fsae_control/fsae_control/steering_sysid.py` | `fsae_planning` (live ROS 2 ws) | the node — drives FSDS directly |
| `ros2/run_steering_sysid.sh` | **FSDS repo root**, next to `launch_all.sh` | one-command harness |
| `tuner/checks/steering_sysid_analysis.py` | `fsae_MPCTest` | reads the log, names the mechanism |
| `fsae_control/steering_step.py` / `ros2/run_steering_step.sh` / `tuner/checks/steering_step_analysis.py` | same split | the step-input companion test (50 Hz, isolates the transient) |

**Run it with one command** (starts FSDS, waits for RPC, starts the bridge,
waits for odom, runs the sweep, analyses the log, tears everything down,
including on Ctrl+C):

    cd <FSDS repo>/ros2 && ./run_steering_sysid.sh

Flags: `--no-sim` (FSDS already running), `--quick` (fewer points), and any
`-p name:=value` passes through to the node. **Run it on an empty map** — it
circles at up to 14 m/s and does not brake for cones. The harness refuses to
start if `mpc_controller`, `fsds_bridge`, or `stanley` is already running,
since two publishers on `/fsds/control_command` would interleave and corrupt
the log.

**Geometry is bounded automatically.** The node reaches target speed while
already turning (so it orbits rather than travelling), checks a geofence
(`home_radius`/`max_radius`) from every phase, and predicts each point's orbit
size in advance (using a deliberately pessimistic `K_US_ESTIMATE = 0.05`) to
skip any (speed, steering) pair whose orbit won't fit in the geofence,
logging what it dropped. Default steering commands
(`[0.5, 0.65, 0.8, 1.0]`) are biased high since low-angle, high-speed points
are both the least informative and the least likely to fit.

**Reading the log.** It records the raw normalised `cmd.steering` alongside
the roadwheel angle it's assumed to map to — recording only the assumed
angle would beg the question the test exists to answer. A falling
`s = δ_ach/δ_cmd` is not by itself diagnostic (a speed-scaled rack, genuine
understeer, and grip saturation all produce one), so the analyser fits all
five candidate mechanisms to achieved yaw rate and reports the margin to the
runner-up:

| winning model | meaning |
|---|---|
| neutral (s≈1) | steering path is fine; look at the controller/reference |
| constant scale | `MAX_STEER_RAD` wrong — fix in all three copies |
| speed-scaled rack | FSDS reduces lock with speed; model it in the plant |
| understeer (v²) | real vehicle dynamics |
| grip saturation | yaw capped by lateral grip |

The default speed sweep is **3–14 m/s**, wide enough to separate speed-scaled
rack from understeer (near-degenerate over a narrow band). If the analyser
prints a margin warning, widen the speed range and re-run rather than
trusting the verdict; it also refuses a verdict when fewer than 3 windows
contain real motion (a car wedged against a wall otherwise reports a
confident, meaningless answer from all-zero data).

### Checked: the longitudinal path is not mis-scaled

Throttle authority, acceleration, and braking limits match FSDS closely (the
car if anything brakes harder than the plant model allows; 0% throttle
saturation). **But `fsds_bridge.py` discards the MPC's own `a_cmd` output**
and re-derives throttle from a separate P-controller on target speed —
offline, `a_cmd` drives the plant directly. This is a genuine, unmodelled
sim/live divergence in the longitudinal loop (mean speed error ~0.6 m/s), but
it cannot explain the yaw-saturation gap — a longitudinal path cannot stop
the car rotating. Modelling it offline, or feeding `a_cmd` through live, is
untested.

**Consequence:** offline scores are not yet fully predictive of live
behaviour. A tuning run that scores well offline can still produce a car
that saturates steering a fifth of the time. Always validate on the car
before trusting a tuned weight set.

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

## Post-solve output smoothing (`output_smoothing_*`)

**A genuinely different kind of mechanism from every other adaptive-gain
feature above.** Every `Q`/`R`/`R_rate` gain-scheduling mechanism
(`adaptive_R_rate`, corner-factor scheduler, reversal penalty, etc.)
reshapes the QP's COST before solving, fresh each tick with no memory
across ticks. Output smoothing instead low-pass-filters the SOLVED
steering command afterward: `filtered += alpha*(raw - filtered)`, then
blends `steering = (1-w)*raw + w*filtered`. Because `filtered` persists
tick to tick, this is a genuine temporal filter and adds lag — the
one mechanism in this family that does.

That lag is bounded by fading `w` toward (never fully to) a floor as the
car needs a fast, accurate response, via four independent, multiplicative
`1/(1+k*|x|)`-shaped factors (the same saturating-curve style used
throughout this codebase):

1. **Current curvature** — `w = max(corner_floor, 1 - corner_frac)`, using
   the existing `_corner_factor`. Full smoothing on a clean straight, fading
   toward `corner_floor` as the car actually turns.
2. **Current tracking error** (`k_ey`, `k_epsi`) — fades further as
   `|e_y|`/`|e_psi|` grow, independent of curvature, so a disturbance
   recovery on a straight (where curvature alone would keep smoothing at
   full strength) still gets a fast response.
3. **Lookahead curvature** (`lookahead_lead_s`) — fades smoothing down
   BEFORE a corner already visible in the path arrives, not only once the
   car's own current curvature has risen. Needed because a purely
   current-curvature fade only starts acting after a short straight has
   mostly already ended on tracks where straights are shorter than the
   filter's own settle time. Implemented via `peak_kappa_ahead()` (mirrors
   `curvature_speed()`'s dense-resample/denoise pipeline, returning peak
   curvature over a scan window instead of a speed target) scanning a
   speed-scaled distance (`scan_end = max(car_speed, 2.0) * lookahead_lead_s`
   — a TIME lead converted to a DISTANCE via current speed, so the warning
   lead time stays constant across speed rather than a fixed-metres window
   giving less warning exactly when a fast car needs more) ahead of the
   car's nearest point on the path; `corner_frac = max(current, lookahead)`.
4. **`corner_floor`** — the hard floor factors 1-3 can never fade below,
   so smoothing never fully disengages even at maximum corner severity.

**Implemented in:**
- **Live**: `mpc_controller.py`/`mpc_controller_standalone.py`'s
  post-solve block (search "Output smoothing"); `peak_kappa_ahead()` in
  `control_utils.py`. Params: `output_smoothing_enabled`, `_alpha`,
  `_corner_floor`, `_k_ey`, `_k_epsi`, `_lookahead_lead_s` — node-declared
  parameters, NOT `MPCParams`/`NMPCParams` fields, so (unlike most tunables
  in this codebase) they need EXPLICIT `DeclareLaunchArgument`/parameters-dict
  wiring in `control.launch.py`/`sim.launch.py` rather than picking up
  auto-generated launch args; `launch_all.sh`'s `OUTPUT_SMOOTHING_*`
  variables forward into that wiring via `_append_mpc_arg`.
- **Offline**: `sim/speed_profile.py`'s `peak_kappa_ahead()`, wired into
  `sim/rollout_core.py`'s `if OUTPUT_SMOOTHING_ENABLED:` block (a local
  `corner_frac_smooth` variable, not reassigning `corner_frac`, which is
  read elsewhere in that function); `settings.py`'s `OUTPUT_SMOOTHING_*`
  constants.

**Caution:** the underlying EMA filter's own settle time
(`~3/(alpha*CONTROL_HZ)` seconds to ~95%) sets the scale for tuning both
`corner_floor` and `lookahead_lead_s` — a `lookahead_lead_s` much shorter
than that settle time gives the fade too little time to complete before a
corner arrives. `alpha` itself needs a genuine disturbance-recovery test
to tune, not just an oracle-path rollout: a near-perfect reference path
never exercises the recovery speed a real disturbance demands, so an
offline sweep on that metric alone can select an `alpha` that looks better
but drifts badly live. Re-tuning any of these four factors without a live
test is not sufficient validation.

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

## Input-jerk cost (`nmpc_rjerk_delta` / `nmpc_rjerk_a`) — NMPC only

**Plain version:** the controller already pays a price for *moving* the
steering wheel. That price is the same whether it is turning steadily into a
corner or wobbling back and forth, so raising it to stop the wobble also makes
the car reluctant to turn. This term prices something different — how much the
*rate* of steering changes — which is small for a steady turn and large for a
wobble. It lets the wobble be penalised without penalising turning in.

Technically: a **second-difference** penalty on the control inputs, alongside
the existing first-difference rate cost. `nmpc_rjerk_delta` weights steering
*acceleration* (the change in the change); `nmpc_rjerk_a` does the same for
the longitudinal input. Both default to `0.0`, which removes the term from the
QP entirely (no Hessian contribution). Live/offline defaults today:
**150.0 / 0.0**.

**Why it exists.** The plain rate cost charges by `|du|`, which is identical
for a sustained ramp into a corner and for one leg of an oscillation — so it
cannot suppress hunting without also resisting turn-in. That is the trade the
whole steering-chatter investigation kept running into. Measured on live data,
direction **reversals** carry ~4.3× the `|d2|` of same-direction ramps versus
only ~1.9× the `|d1|`, so the second difference separates the two about twice
as sharply. A steady ramp scores near zero here and is nearly free; an
alternating wiggle is expensive.

**How it enters the QP.** `E` is the first-difference operator already built
for the rate cost; the jerk term reuses it as `E2 = E @ E`, so
`du_k − du_{k−1}` is the discrete input acceleration:

```
Hess += E2ᵀ · diag(rj) · E2          rj = tile([rjerk_delta, rjerk_a], N)
grad += E2ᵀ · (rj · e_jerk)
```

Two implementation points that matter before modifying this term:

- **It is anchored to the two previous applied inputs, not just one.** A
  second difference spanning the tick boundary needs `u_prev` *and*
  `u_prev2`, hence the corrections `e_jerk[:NU] -= (2·u_prev − u_prev2)` and
  `e_jerk[NU:2NU] += u_prev`. Without them the term is blind to a reversal
  that straddles the boundary — exactly the case it exists to catch. The
  controller therefore carries `_u_prev2` state, which must be updated
  before `_u_prev`.
- **No OSQP sparsity change.** `p_mask[:n_du,:n_du]` is already a dense upper
  triangle, so `E2ᵀ·R·E2` adds no new nonzeros and the solver's pattern is
  unchanged — the term can be enabled/disabled between runs without
  rebuilding the problem structure.

`_cost()` carries a matching `jerk` term. **This must stay in step with the
QP**: the SQP line search scores candidate steps with `_cost()`, so a term
present in the Hessian but absent from `_cost()` means the search is
optimising a different objective than the one being solved. That exact bug
existed for the rate cost (it used the flat `self.r_rate` while the QP used
`_Rr_flat`) and affected every rate-reshaping flag.

**Measured effect (live, `centerline.csv`).** At `rjerk_delta=150` with
`r_rate_delta=52.5`: 0 saturated ticks, 0 slew-limited ticks and 1 steering
reversal over three laps. Offline at the same pair, slew-limited ticks fall
7.80% → 2.77% and chatter 2.825 → 1.686 °/tick.

**Caution on attributing a saturation figure to this term.** An earlier live
run showed ~4.5% steering saturation with `rjerk=150` and it was initially
read as the jerk penalty trading smoothness for saturation. That was the
*reference line*, not this weight — the same weight on the centreline
saturates 0.00%. See "Reference line: raceline vs centreline" below.

**Untested:** the offline low-rate pairing `r_rate_delta=5.0` with
`rjerk_delta=250.0`, which beats the flat-52.5 baseline on every offline
metric. Set both together if trying it. `nmpc_rjerk_a` has never been
exercised at a nonzero value on either side.

## Three-zone rate schedule (`nmpc_rrate_zone_*`) — and why `k` gates it

**Plain version:** the price the controller pays for moving the steering wheel
should not be the same everywhere. On a straight it should be high, so the car
holds still instead of hunting. Approaching a corner it should drop, so the car
is willing to start turning. Through the corner it should be lowest. This
mechanism slides that price continuously between three levels based on how much
the road is bending now and how much it will bend just ahead.

Technically: a continuous multiplier on the NMPC's steering-rate cost, driven
by current curvature and the peak curvature the horizon predicts ahead:

| zone | condition | multiplier |
|---|---|---|
| straight | nothing now, nothing ahead | `boost_straight` (2.0) |
| approach | nothing now, corner ahead | `ease_approach` (0.35) |
| corner | turning now | `floor_corner` (0.15) |

It **multiplies** `r_rate_delta` rather than overwriting it, so it composes
with the tuned 52.5 — unlike `nmpc_corner_rrate_blend_enabled`, which
overwrites `R_rate[0,0]` and silently discards it.

**The endpoints are gated by `nmpc_corner_factor_k`, and this is easy to miss.**
Both `now` and `ahead` pass through
`_corner_factor(|κ|, k) = 1 − 1/(1 + k|κ|)`, and the corner floor is only
reached as that approaches 1. So the schedule is only as strong as `k` lets it
saturate **over the track's own curvature range**:

| `\|κ\|` | `_corner_factor` at k=8 | at k=27 |
|---|---|---|
| 0.00 | 0.000 | 0.000 |
| 0.06 | 0.390 | 0.618 |
| 0.209 (this track's tightest) | **0.626** | **0.850** |
| 1.125 | 0.900 | 0.968 |

At the LTV-QP's inherited `k=8.0`, reaching `_corner_factor`=0.9 needs
`|κ|`=1.125, but `comp_test_map_3`'s tightest corner is 0.209. Measured live
with the zone enabled at 2.0/0.35/0.15 and `k` inherited, `m_Rrate_zone`
ranged **0.829–1.962 with 0% of ticks in either the ease or floor band** — the
multiplier never left the boost band, so what actually ran was a mild global
rate *boost*, not a three-zone schedule. Scores were a wash against the
zone-off baseline (0.522 vs 0.488), which is the expected result of a
mechanism that never engaged.

`k=27.0` puts 0.209 at `_corner_factor`=0.85, giving a real swing rather than
the 2.4× the inherited `k=8` allowed. Derive it from the track, not by feel:

```
k ≈ target_corner_frac / ((1 − target_corner_frac) · κ_max)
```

**OPEN: the schedule DNFs offline at the intended endpoints, unexplained.**
With `k=27` and `ease_approach`=0.35 the offline rollout goes off-track at the
track's tightest corner (x≈41.4, y≈49.6 — the same corner as the live
raceline excursion), full-lock steering with `|e_y|` growing monotonically to
2.53 m. That is a late-turn-in failure, i.e. the *opposite* of what releasing
the rate weight on approach should do. The obvious readings are all
contradicted by measurement:

- Not "too much release": raising `floor_corner` to 0.5 or 0.7 still DNFs.
- **Not a monotonic tuning effect at all:** `boost_straight`=0.8 — which makes
  the multiplier ≤1 everywhere, i.e. uniformly *weaker* than no zone — also
  DNFs, at step 289. A configuration strictly gentler than the baseline
  cannot cause worse turn-in than the baseline through the rate weight alone.
- Not the speed profile: matching the harness's `v_max` to the driven
  centreline's 16.70 changes nothing.
- Not `k`: with the zone disabled, k=15/27/40 are all identical to baseline
  (the field is inert without the zone), and k=15/40 with the zone on
  complete.
- Not weight compounding: `_Rr_flat` is rebuilt fresh from `r_rate_tick` each
  tick, because `rrate_zone_enabled` is in the rebuild guard's condition —
  so the scaling does not accumulate across ticks.

The DNF runs also carry high non-`solved` SQP rates (42% at
`boost_straight`=0.8 vs 14% at baseline), so the leading hypothesis is that
rescaling `_Rr_flat` *after* the rollout — the zone is applied later than
every other rate-reshaping flag, because it needs horizon curvature —
interacts badly with the warm start or line search. **Not confirmed.**

`ease_approach` is therefore set to **0.80**, the value at which the offline
rollout completes (score 0.796 vs 0.822 baseline, `|e_y|` 0.476 vs 0.496,
p90 0.976 vs 1.044 — a modest genuine improvement). **The intended
0.35 turn-in release remains untested**, live and offline.

Note the sim and the car disagree here: the same zone at `k=8`/0.35 completed
two clean laps live (score 0.522) while DNFing offline at step 274. Given the
documented sim-to-real gap, that is not proof either side is right, but it
does mean an offline DNF here is not automatically a live DNF.

Consequences:

- **Read `m_Rrate_zone` before believing an A/B of the endpoints.** If it
  never approaches `floor_corner`, the endpoints are not what was tested and
  `k` is the thing to change. A null result here means "did not engage" at
  least as often as it means "does not help."
- **Run `tuner.steering_chatter_check` before shipping a zone/`k` change.**
  The `k=27`/0.35 combination was set from the saturation arithmetic alone
  and would have gone to the car as an offline DNF had the check not been
  run afterwards.
- **`nmpc_corner_factor_k` is shared** with `nmpc_corner_rrate_blend_enabled`.
  Raising it for the zone also sharpens that blend — harmless while the blend
  is off, but not independent.
- `corner_frac` is published in telemetry and read by the node-level output
  smoothing, but through `params.corner_factor_k` (the LTV-QP field), **not**
  the NMPC override — so raising `nmpc_corner_factor_k` does not perturb
  smoothing even when it is enabled.

## Turn-in timing: the command leads, and six levers do not move it

**Plain version:** when the car seems to turn in late, the steering command
itself is not late. Comparing what the wheel was told to do against what the
corner geometrically required shows the command arriving about 0.15 s *early*
and at roughly 94% of the needed angle. So a "turns in late" complaint is
about what happens after the command, not about the controller's timing.

Measured live on `centerline.csv` (3 runs, correlation 0.93): the commanded
steering **leads** the geometrically-required angle (`atan(L·κ)` at the car's
own station — the steering angle a simple bicycle model needs for that
curvature) by **0.15 s**, and delivers **0.936** of it in corners.
`delta_cmd` and the applied `steer_deg` are identical to 0.0005°.

So a "turns in late" report is not the controller deciding late. Whatever
produces the residual symptom, it is downstream of a command that is early
and close to the right magnitude.

**Six candidate causes were tested and falsified** — full evidence in
`docs/logs/steering_chatter_investigation.md` ("Session N+1"):
`r_rate_delta` (52.5 is the *best* value tried; lower is wider and DNFs),
`nmpc_corner_factor_k` past 27 (k=60 measurably worse live),
`nmpc_q_e_y` (kept at 7.5 but does not change the drift rate),
`NMPC_SQP_ITERS` (slew-limited tick fraction flat across 1/2/3),
speed-profile braking feasibility (exported profiles are feasible; 0.00% of
stations exceed −7.0 m/s²), delay compensation (same lag in high- and
low-latency halves of the same run), and the `alat_ceiling` model (it is
*conservative* in the 6–14 m/s band, not optimistic).

**A drift-rate invariant worth knowing before tuning this again:** across
every configuration tried on this track — `r_rate` 5→52.5, `k` 8→60,
`q_e_y` 6.35→7.5 — the rate-normalised count of sustained lateral-error-growth
episodes sits at **~31.5 per minute**. A cost-weight change redistributes
episode size, not episode rate. Treat a raw episode count as uninterpretable
unless divided by run duration: the same config gave 29 episodes in 55 s and
48 in 90 s, which was briefly mis-read as a regression.

**Two metric traps found here**, both of which produced a plausible wrong
conclusion before being caught:

- **Lap-wide ratios are dominated by straights.** "Plant yaw gain" (achieved
  yaw rate ÷ `v/L·tan(δ)`) reads 0.42 at p10, which looks like severe
  understeer. Binned by speed it is an artefact: the low-gain ticks are fast
  and nearly straight, where the denominator is near zero. The gain *rises*
  with `a_lat` (0.13 at 0–2 m/s² → 1.10 at 6–8), the opposite of a grip
  limit. The same applies to a steering-vs-yaw-rate cross-correlation over a
  full lap — it measures the phase of the straights.
- **`dv_target/dt` along a rollout is not the profile's gradient.** It reads
  −26.9 m/s² (apparently infeasible) where the exported CSV's own spatial
  gradient is −5.03; the car crossing stations faster inflates the time
  derivative. Read feasibility off the exported file, not off a rollout.

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

**The mechanism, and why it is a real optimiser bug.** The raceline's offset
from the centreline is tiny — mean 0.13 m, max 0.48 m over the whole lap, and
only 0.35 m through the failing corner. At that corner `|κ|` is 0.209, the
global maximum for this track and ~70% of the car's full-lock kinematic floor
(1/3.32 m = 0.30). There is no width left to cut with, so the search bought
no lap time; but the offset it did apply still perturbed the curvature of a
corner already at the edge of what the plant can deliver. Worst of both — a
marginally harder corner for no gain. `_candidate_score`'s
`CURVATURE_SOFT_MAX` penalty does not catch this, because it thresholds
**absolute** curvature (0.22) rather than curvature against the
`alat_ceiling` the planned speed at that station actually permits.

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
`DYNAMIC_CAP_SAFETY=0.9` (vs. `a_lat_max=4.75`/`safety=1.0` elsewhere), so it
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
`epsi_ra_half_rad`/`_accel_boost_max`/`_brake_floor`, `reversal_penalty_enabled`/
`_boost_max`/`_k` (soft steering-reversal penalty, see its own section below)
— none have any read site in `nmpc_core.py`.

**NMPC-only overrides** (`mpc_params.py:189-205, 222-223, 238-240, 253-256`):
`nmpc_q_e_y`, `nmpc_q_e_yd`, `nmpc_q_e_psi`, `nmpc_q_epsi_dot` (overrides
`q_r`, different regressor), `nmpc_q_e_v`, `nmpc_r_delta`, `nmpc_r_a_accel`,
`nmpc_r_a_brake`, `nmpc_r_rate_delta`, `nmpc_r_rate_a`, `nmpc_terminal_scale`,
`nmpc_steer_rate_anti_hunt_enabled`, `nmpc_anti_hunt_boost_max`,
`nmpc_reversal_penalty_enabled`/`_boost_max`/`_k` (see the reversal-penalty
section below), `nmpc_corner_rrate_blend_enabled`, `nmpc_corner_factor_k`,
`nmpc_rrate_steer_straight`/`_corner` (blends `R_rate[0,0]` by current
curvature; takes priority over `nmpc_steer_rate_anti_hunt_enabled` if both
are set — use one or the other, not both),
`nmpc_rrate_zone_enabled`/`_boost_straight`/`_ease_approach`/`_floor_corner`
(the three-zone rate schedule — see "Three-zone rate schedule" below; note it
also reads `nmpc_corner_factor_k`, so that field is NOT exclusive to the
corner blend),
`nmpc_rjerk_delta`/`_a` (second-difference input cost),
`nmpc_rrate_stage_ramp_enabled`/`_near` — each read solely by
`nmpc_core.py`'s `_pick()` calls; `mpc_core.py` never references any of them.

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
implementation, and validation history, extended by §16.9-16.12 with the
live-test results, the MPCC-feature live tests, and two DISTINCT
standstill-steering bugs/fixes (§16.11: a manufactured tyre force at
`v_x=0`; §16.12: the NMPC's own speed-tracking cost term leaking into
steering — see this doc's "Post-solve output smoothing" section's
neighbour, the speed-target rise limiter, for the fix's other half).
Reproduce the offline closed-loop comparison with
`python3 ros2/src/fsae_planning/control/fsae_control/test/nmpc_offline_check.py`
(no ROS/FSDS session needed; the closed-loop section self-skips without an
`fsae_MPCTest` sibling checkout) or `python -m tuner.nmpc_offline_check`.

**Open, separate issue: NMPC steering chatter while cornering** (magnitude
hunting tick-to-tick, not a sign-flip reversal — distinct from both the
reversal-penalty feature above and the two standstill bugs). Root cause not
yet found; confirmed NMPC-specific (~7x noisier than the LTV-QP under
matched conditions) and reproducible offline with zero sensor noise, so it
is not purely a live sensor-noise artifact, though a live-only contributor
also appears to stack on top of the offline-reproducible baseline. See
`docs/logs/steering_chatter_investigation.md` for the full investigation —
what's been ruled out (single cost-weight retunes, SQP iteration count,
trust-region size in isolation, FD-Jacobian coarseness, reference-spline
noise, Frenet-projection instability, terminal cost scale) and what hasn't
been tried yet, before repeating any of that work. Reproduce/extend with
`python -m tuner.steering_chatter_check`.
