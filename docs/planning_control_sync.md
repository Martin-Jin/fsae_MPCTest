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

**Last resynced:**
- Expanded from a control-layer-only mirror to the full workspace: added
  `common/fsae_interfaces` (message package), the rest of `common/fsae_bringup`
  (`fsae_params.yaml`, `perception.launch.py`, `planning.launch.py`,
  `sim.launch.py`, full package scaffolding), all of `planning/fsae_planning`
  as a real package (previously only `planning/` at this repo's root held the
  algorithm-only mirror — see the file mapping below, that mirror is
  unchanged), `perception/fsae_sim_perception`'s package scaffolding, and
  `control/fsae_control`'s remaining nodes: `fsds_bridge.py`,
  `stanley_controller.py`, `mpc_controller.py`, `telemetry_logger.py`, plus
  package scaffolding. **The frozen `fsds_simulator/stanley_controller/`
  reference implementation (`stanley_control.py` / `stanely_control_utils.py`,
  targeting an old `/fsds/planned_path`+`Track` interface and importing a
  never-defined `separate_cones_by_color`) was deleted** and replaced by the
  real, current `stanley_controller.py` (mirroring upstream's actual node,
  which uses `control_utils.py`'s `StanleyController` — see "Deliberately not
  mirrored" below, that exclusion no longer applies).
- `planning/` (`boundary.py`, `cone_sorting.py`, `path_utils.py`) brought to
  parity with the `fsae_planning` checkout in `ros2/src/fsae_planning/` in
  this repo's sibling directory. This dropped the `build_path_trace` ft-fsd
  trace-sort planner (upstream removed it — see "Deliberately not mirrored"
  below), renamed `_build_wall_segments` → `build_wall_segments` and added
  `segment_crosses_walls` (both now public), and added `roll_loop_to_car` (a
  skidpad-support helper — see its own note below). `cone_sorting.py` also
  lost some already-dead commented-out code (`separate_cones_by_color`, an
  unused `Cone` import) that upstream had already cleaned up.
- `fsds_simulator/` reorganised from a flat folder (`control_node.py`,
  `mpc_core.py`, `control_utils.py` all directly inside `fsds_simulator/`)
  into the mirrored hierarchy described above, and its old `control_node.py`
  was retired entirely in favour of upstream's `mpc_controller_standalone.py`
  (see "What replaced `control_node.py`" below — this is a same-design,
  different-file swap, not a like-for-like rename).
- **2026-08-09: `steering_sysid.py`/`steering_step.py` and their harness
  scripts genuinely mirrored, at explicit user request** — this reverses the
  "why none of it is mirrored" rationale recorded below for `steering_sysid`
  (and the equivalent absence for `steering_step`), which was correct at the
  time (both were diagnostics not part of the car's runtime stack) but is
  now superseded. Added: `control/fsae_control/fsae_control/steering_sysid.py`,
  `steering_step.py` (byte-identical copies of the live working-tree files —
  neither is in `fsae_planning`'s committed git history, same status as
  `mpc_controller_standalone.py`/`scoring.py`, see the file-mapping table),
  and `setup.py`'s two matching entry points (also now byte-identical — the
  `steering_sysid`-only divergence documented below no longer exists).
  `ros2/run_steering_step.sh`/`run_steering_sysid.sh` copied to
  `fsds_simulator/`'s own root (alongside `launch_all.sh`) with the same kind
  of machine-specific path adaptation `launch_all.sh` already carries
  (Windows username, and `LOG_DIR` pointed at `$HOST_REPO_ROOT/fsae_logs` to
  match `launch_all.sh`'s `log_dir:=` arg instead of the live scripts'
  `~/fsae_logs` default) — genuinely adapted copies, not byte-identical,
  same as `launch_all.sh`. `tuner/checks/steering_sysid_analysis.py` /
  `tuner/checks/steering_step_analysis.py` remain `fsae_MPCTest`-only; nothing to mirror
  there. Also folded in a same-day `launch_all.sh` resync from a parallel
  session: both copies now set `USE_PRECOMPUTED_SPEED`/
  `USE_PRECOMPUTED_PATH` shell variables and pass them through, and the
  mirror picked up the `--symlink-install` rebuild step (§49) and `log_dir`
  launch arg it was missing relative to the live copy.

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
| `control/fsae_control/fsae_control/stanley_controller.py` | `control/fsae_control/fsae_control/stanley_controller.py` | Direct mirror. The actual current `controller:=stanley` node (publishes `cmd_vel`, routes through `fsds_bridge`). Replaces the old frozen reference implementation — see "Last resynced" above. |
| `control/fsae_control/fsae_control/mpc_controller.py` | `control/fsae_control/fsae_control/mpc_controller.py` | Direct mirror. Upstream's default `controller:=mpc` node — steering only through `cmd_vel`/`fsds_bridge`, discards the MPC's own throttle/brake. See "Two MPC-controller nodes" below. |
| `control/fsae_control/fsae_control/mpc_controller_standalone.py` | *(no upstream counterpart — never existed in `fsae_planning`'s git history)* | **Not a direct mirror** despite this table's general framing — this file was authored in this repo (ported from the retired `fsds_simulator/control_node.py`) and is staged here for eventual upstreaming, the reverse direction from every other row. See "What replaced `control_node.py`" for the history. |
| `control/fsae_control/fsae_control/fsds_bridge.py` | `control/fsae_control/fsae_control/fsds_bridge.py` | Direct mirror. Needed by both `stanley_controller.py` and `mpc_controller.py` (not `mpc_controller_standalone.py`, which bypasses it). |
| `control/fsae_control/fsae_control/telemetry_logger.py` | `control/fsae_control/fsae_control/telemetry_logger.py` | Direct mirror. CSV telemetry shared by all three controller nodes. Also computes the run's composite score (via `scoring.py`) and prepends it to the control CSV as a `#`-commented header on `close()`. Includes `LapProgressTracker` (added 2026-08-11) — computes real `progress`/`reached_end`/`time_bonus` from the precomputed track path, fixing the live composite score being permanently pinned at `13.0`; see "Live/offline score parity" below. |
| `control/fsae_control/fsae_control/scoring.py` | *(no upstream counterpart — never existed in `fsae_planning`'s git history)* | **Not a direct mirror.** Staged here for upstreaming, same direction as `mpc_controller_standalone.py` above. It **is** a verbatim copy of this repo's own `sim/scoring.py` — see "Live/offline score parity" below. Changes must be made in `sim/scoring.py` first, then re-copied here (and eventually upstreamed). |
| `control/fsae_control/fsae_control/steering_sysid.py` | *(not in `fsae_planning`'s committed git history — mirrored from its working tree, same status as `mpc_controller_standalone.py`/`scoring.py` above)* | Direct mirror of the live working-tree file (added 2026-08-09 — see "Last resynced" above; previously deliberately absent, see "Why none of it is mirrored" further down for the historical rationale). Open-loop steering system-ID diagnostic node. |
| `control/fsae_control/fsae_control/steering_step.py` | *(same git-history status as `steering_sysid.py` above)* | Direct mirror of the live working-tree file (added 2026-08-09, same resync as `steering_sysid.py`). Step-input transient diagnostic node. |
| `control/fsae_control/setup.py` | `control/fsae_control/setup.py` | Direct mirror **except** the `mpc_controller_standalone`/`scoring.py` entry points/imports this repo's own two staged-for-upstream files above need — those exist here but not upstream. Registers six console-script entry points (`controller`, `mpc_controller`, `mpc_controller_standalone`, `fsds_bridge`, `steering_sysid`, `steering_step`) — the last two used to be an intentional one-line-each omission (see below) until both nodes were mirrored 2026-08-09. |

> **Former intentional divergence in `setup.py`, resolved 2026-08-09.** Until
> then, the live working tree had two entry points this mirror's `setup.py`
> didn't (`steering_sysid`, and later `steering_step`) — both open-loop
> diagnostic nodes, deliberately not mirrored per "do not add files that were
> never there" (see "Why none of it is mirrored" below for the full
> historical rationale). Copying an entry point without its module would
> have pointed `setup.py` at a non-existent target. Once both nodes were
> genuinely mirrored (see "Last resynced" above), both entry-point lines were
> added at the same time, per the standing rule this note originally set out.

> **`zip_safe=False` (2026-08-08, S49) — no longer a divergence as of
> 2026-08-09.** All four of this mirror's `setup.py` files
> (`common/fsae_bringup`, `control/fsae_control`, `perception/
> fsae_sim_perception`, `planning/fsae_planning`) set `zip_safe=False` — the
> root cause fix for a stale-`colcon-build` bug (§49 in
> `docs/logs/sim_to_real_investigation.md`). This was flagged here as a resync TODO
> because the live working tree hadn't picked it up yet; checked again
> 2026-08-09 during the `steering_sysid`/`steering_step` resync and all four
> live copies now also set `zip_safe=False` — verified byte-identical on
> this setting, nothing left to resync here.

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
| Metric normalisation scales | `settings.py` (`METRIC_SCALES`) | `fsds_simulator/control/fsae_control/fsae_control/scoring.py` (inlined as module constant) | 13 entries (added `accel_reversal_rms` 2026-08-08 — see that commit's message for the METRIC_SCALES[12]=0.08 measurement), `[0.40, 0.45, 0.30, 0.18, 1.50, 0.40, 0.02, 0.30, 1.00, 0.015, 0.70, 2.30, 0.08]` |
| Constrained-scoring constants | `settings.py` (`CONSTRAINT_FLOOR`, `COMPLETION_THRESHOLD`, `TIME_OBJECTIVE_WEIGHT`, `QUALITY_WEIGHT`) | `fsds_simulator/.../scoring.py` (inlined as module constants) | `10.0` / `0.98` / `1.0` / `0.35` |
| `A_BRAKE_PLAN` (braking-distance propagation in `curvature_speed`) | `sim/speed_profile.py` | `fsds_simulator/.../control_utils.py` | `5.0` m/s², positive magnitude |
| Dynamic speed cap enable/gains | `settings.py` (`ENABLE_DYNAMIC_SPEED_CAP`, `DYNAMIC_CAP_A_LAT_MAX`, `DYNAMIC_CAP_SAFETY`) | `mpc_controller.py`/`mpc_controller_standalone.py` (`enable_dynamic_speed_cap`/`dynamic_cap_a_lat_max`/`dynamic_cap_safety` ROS params) | `True` / `3.2` m/s² / `0.9` — see "Dynamic speed cap" section below |
| Latency telemetry columns | — (offline has no equivalent) | `fsds_simulator/.../telemetry_logger.py` | `pose_age_s`, `path_age_s`, `n_delay`, `solve_ms`, `cmd_latency_ms` |
| Pose-feed hold model | `settings.py` (`POSE_HOLD_*`) + `sim/rollout_core.PoseFeedHold` | — (offline-only; models a live fault) | `PROB 0.05`, `MEAN_TICKS 2.1`, `MAX_TICKS 5` |
| Accel/brake effort split (`R[1,1]`) | `settings.py` (`R_A_ACCEL`, `R_A_BRAKE`), read by `controller/optimiser.py`'s `solve_mpc(r_a_accel=, r_a_brake=)` | `mpc_params.py` (`r_a_accel`, `r_a_brake`), read by `mpc_core.py`'s `_solve_qp` | actively being live-tuned — re-check both sides' current values before trusting this row; see "Accel/brake effort weight split" below |
| Corner-factor scheduler + heading-error accel/brake asymmetry | `settings.py` (`CORNER_FACTOR_K`, `Q_EY_*`/`Q_EPSI_*`/`Q_R_*`/`RRATE_STEER_*`/`R_STEER_CORNER_MID`, `LOW_SPEED_CORNER_BOOST_*`, `EPSI_RA_*`) | `mpc_params.py` (same names, lowercase) | see the "MPC weight/gain parity" table above for the full current field list — **replaces every row this table used to carry for the deleted lookahead gain-scheduling family** (exit-boost decay distance/peak-tracker, low-speed steering-rate boost, steering-effort relaxation, curvature forcing, anti-hunt lookahead gate, exit-boost `\|e_psi\|` hold threshold — all removed 2026-08-13, see "Corner-factor scheduler rewrite" below) |

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

This table is the field-by-field mapping.

Previously (before this row existed) `Q_diag`/`R_diag`/`R_rate_diag` and
every adaptive-gain constant had a parity OBLIGATION stated only in prose
comments, with no row here — that gap is what this section closes.

**Table current as of the 2026-08-13 corner-factor rewrite** — every field
below is confirmed present on both `MPCParams` and `settings.py` as of that
date. See "Corner-factor scheduler rewrite" below for what replaced the
~35-field lookahead/demand-normalisation/U-turn/straight-line family this
table used to list; those fields no longer exist on either side.

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
| `nmpc_q_e_y` … `nmpc_terminal_scale` (11 override fields) | `NMPC_Q_E_Y` … `NMPC_TERMINAL_SCALE` | all `-1.0` (inherit sentinel, matched) — see "Nonlinear MPC" section below |

The remaining `MPCParams` fields (`max_delay_compensation_steps`,
`predict_epsi_clip`, `pose_age_lp_alpha`, `n_delay_hysteresis`,
`delay_compensation_enabled`) are live-only tuning knobs with no offline
`settings.py` counterpart — the offline sim has no equivalent of live pose
latency to compensate for. Add a `settings.py` constant and a
`rollout_core.py` call-site keyword the same way as the rows above before
relying on tuning one of these offline, if that ever becomes relevant.

## Corner-factor scheduler rewrite — replaces the lookahead gain-scheduling family (2026-08-13)

**What was deleted.** ~15 interacting functions that scanned FORWARD along
the path every tick (`lookahead_curvature_profile` → a scalar
`kappa_max_abs`, the peak curvature within a speed-scaled lookahead
window) and reweighted `Q`/`R`/`R_rate` in anticipation of a corner not yet
reached: the approach/exit `Q[0,0]`/`Q[2,2]` boosts, the `Q[3,3]`/`R[0,0]`
relaxations, demand normalisation (`_corner_demand`/`_alat_ceiling_at`), the
U-turn detector, the straight-line `Q`/`R[0,0]` adjustments, and the
`CornerMap`/`_segment_corners` precomputed-path fast path for the same
scan (itself only added the day before, see "Precomputed corner
segmentation" above — both were removed in the same rewrite). See
`mpc_core.py`'s own "Lookahead gain-scheduling family: removed" comment for
the exhaustive function-name list. `tuning.md` §4.4/§4.6-§4.8/§4.10 keep
the original descriptions collapsed for the tuning history; `architecture.md`'s
"Historical" subsection keeps the mechanism-level detail.

**Why.** This MPC formulation already predicts state error against the
reference at each future horizon step. Reweighting TODAY's (usually
near-zero) cost based on what a forward scan finds ahead doesn't change
what the horizon predicts once the car actually gets there — the mechanism
was reweighting a cost that mostly wasn't there yet, not manufacturing
anticipation. (This is the same structural argument, one level up, that
motivates the nonlinear MPC below: reweighting an existing error's cost is
not the same as making the *prediction itself* see the road bend.) The
family had also been raised/live-tested/reverted piecemeal for a long
time without a clear net win — see CHANGES.md and this doc's own dated
sections above for that history.

**What replaced it**: `_corner_factor`/`_low_speed_corner_boost`/`_blend` — a
single continuous CURRENT-curvature-only fraction blending four `Q`/`R_rate`
weights between a straight/corner endpoint, plus an always-on,
independent heading-error-driven accel/brake asymmetry (`epsi_ra_*`). See
`architecture.md`'s "Corner-factor scheduler" section for the full formulas
and mechanism, and `tuning.md` §4.3b for the tuning-surface reference — not
repeated here to avoid duplicating either.

**Mirrored the same day**: `fsae_MPCTest/controller/model_utils.py`
(`_corner_factor`/`_low_speed_corner_boost`/`_blend`, and `settings.py`'s
matching constants) and `fsds_simulator/`'s copies of `mpc_core.py`/
`mpc_params.py`/`fsae_params.yaml` — confirmed byte-identical to the live
files as of this rewrite (see the parity table above).

## Slew-rate limit (`du_max`): was live-only, now on both sides

Until 2026-08-06 the hard per-step slew-rate constraint on
`[delta_cmd, a_cmd]` existed **only** in the live `mpc_core.py`.
`controller/optimiser.py` had no such constraint at all, so the offline tuner
was optimising against a plant that could change steering arbitrarily fast
while the real car was clamped to `radians(4.0)`/step (= 80 deg/s at
`DT = 0.05`). Weights tuned offline therefore did not transfer faithfully —
independent of any weight choice.

Two changes fixed this:

1. `init_parameterized_mpc()` / `solve_mpc()` in `controller/optimiser.py`
   gained an optional `du_max`, mirroring the live formulation. It is baked
   into the cached QP the same way `u_min`/`u_max` are, so it participates in
   the same cache-staleness check (passing a different `du_max` rebuilds the
   problem instead of silently reusing stale constraints). Only the step-0
   constraint against `u_prev` is omitted offline — step 0 is already anchored
   through `weighted_u_prev` in the cost, and `u_prev` isn't a constraint
   parameter in the cached formulation.
2. The limit itself was raised from 80 deg/s to **180 deg/s**, and is now
   expressed as a *rate* (`max_steer_rate * DT`) rather than a fixed per-step
   angle, so its physical meaning survives a change of `DT`.

**Why it was raised.** Live telemetry
(`mpc_standalone_control_1785976976.csv`) showed the steering command pinned
exactly on the 80 deg/s limit for **41% of all control steps**, reversing sign
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

**Status: open.** The controller currently carries workarounds; the root cause
is in the planner and has not been addressed. Read this before changing
`centerline_planner.py`, `boundary.py`, `cone_sorting.py`, `path_utils.py`, or
the planner's smoothing parameters.

### What it is

The published centreline contains curvature spikes that do not correspond to
any real feature of the track. Measured on
`mpc_standalone_path_1785980686.csv`, peak curvature over consecutive ~1 s
snapshots **of the same physical corner**:

| t (s) | R_min | implied v (√(a_lat/κ)) |
|---|---|---|
| 86.5 | 5.3 m | 4.6 m/s |
| 88.5 | 32.3 m | 11.4 m/s |
| 89.5 | 4.2 m | 4.1 m/s |
| 90.6 | **1.0 m** | 2.0 m/s |

Same track, same 21–24 m path length, one second apart. Worst case on lap 2 was
**R = 0.14 m** — physically impossible for this car (min turn radius at 25° lock
and a 1.55 m wheelbase is ≈ 3.7 m).

It **degrades with laps**. Peak-κ statistics across snapshots:

| Phase | median κ | max κ |
|---|---|---|
| Lap 1 early | 0.189 | 0.793 |
| Lap 1 mid | 0.120 | 0.444 |
| **Lap 2** | **0.271** | **7.242** |

Consistent with cone-map clutter accumulating over a lap and corrupting the
centreline fit. The path is otherwise in the *right place* — lap-2 geometry
matched lap-1 geometry at the same physical locations to within 0.12–0.23 m
median separation. The problem is local kinks, not global drift.

### Why it matters to control

`v_target = √(a_lat_max / κ)`, so a spurious κ spike collapses the speed
target, and its absence next frame lets it jump back. Measured `v_desired`
volatility reached **250 m/s²** on lap 2. That alone destabilises the
longitudinal loop.

### Workarounds currently in the controller (defence in depth)

These treat the symptom. They are **not** a fix, and none of them should be
removed without re-measuring against a repaired planner:

1. **Curvature smoothing** — `curvature_speed()` (both
   `control_utils.py` and `sim/speed_profile.py`) takes the max of a 3-point
   running mean of the Menger-curvature series instead of the raw max, so one
   bad triple cannot set the speed for the whole scan window. Modest on its own
   (frame-to-frame volatility 2.75 → 2.61 m/s) because the whole path moves
   between frames, not just one point.
   *Note:* a raw percentile (p75/p90) was tried and rejected — the scan window
   only yields ~7 triples, so a percentile is both noisy and biased upward,
   pushing `v_target` to 20 m/s where the raw max said 5. Wrong direction to err.
2. **Tracking-error speed gate** — `tracking_error_speed_gate()`. Scales the
   target down on `|e_y|`/`|e_psi|`. Inert in normal driving (gate < 0.99 on
   1.6% of clean-section ticks) but cuts the commanded speed from a mean 7.4 m/s
   (max 15.0) to 2.8 m/s (max 4.65) when `|e_y| > 1.5 m`. The gate's own output
   is additionally rate-limited (`GATE_RATE_LIMIT`, both node files and
   `sim/rollout_core.py`) so its tick-to-tick change is bounded in either
   direction — see `docs/logs/sim_to_real_investigation.md` §55/§56 for why: applying
   it unsmoothed let a fast-changing tracking error compound with an
   already-falling curvature-based target into a single-tick `v_desired`
   cliff.
3. **Speed-target rise limiter** — `SPEED_TARGET_RISE_RATE = 2.0` m/s², applied
   in both controller nodes and `sim/rollout_core.py`. Increases only;
   decreases pass through instantly so a genuine brake request is never
   delayed.

Combined effect replayed over the failing log: tick-to-tick `|Δv_desired|`
mean 0.234 → 0.123, p99 2.33 → 0.91; and in the unrecoverable `|e_y| > 1.5 m`
regime, commanded speed mean 7.44 → 2.80. Offline, worst-case peak lateral
error across the 10 synthetic paths fell 1.76 m → 0.79 m (`PATH_SPIRAL`), for a
net composite improvement of −0.267 with small regressions on already-clean
paths — the expected cost of a safety gate.

### Suggested fix (not attempted)

Root-cause work belongs in the planner: curvature-aware smoothing or a
spline-fit residual check in `centerline_planner.py`, and/or rejecting cone
pairings that imply a sub-3.7 m radius. Investigate why lap 2 is worse than lap
1 first — that points at cone-map accumulation rather than the fitter itself.

> **Update — root cause found for one mechanism, and it is NOT cone-map
> accumulation (`docs/logs/sim_to_real_investigation.md` §19).**
>
> Replaying `build_path_walls()` directly against today's real cone map and
> live log found the same physical corner producing a smooth, tightening
> radius on two laps and a discontinuous 5×+ radius jump on the other two —
> reproduced from a single static, already-fully-built cone map, with no
> lap-to-lap accumulation involved.
>
> The actual mechanism: `filter_cones_window`'s `min_ahead=0.5` cutoff drops
> the nearest surviving midpoint as soon as the car's own pose crosses it,
> forcing the car-anchored spline (`pin_start` in `smooth_centreline`) to
> reach for the next midpoint instead — sometimes several metres further
> away — producing a sharp, transient near-field curvature spike.
>
> Confirmed by comparing the two builds directly: `_gen_midpoints()` returns
> byte-identical midpoints across the jump, which rules out cone-map
> duplication, `_absorb()`, and exclusive-nearest-neighbour reassignment as
> the cause of *this* mechanism (all three were reasonable suspects going
> in). "Why lap 2 is worse than lap 1" narrows to *how often a lap's specific
> pose trace happens to straddle a midpoint boundary*, not the map getting
> dirtier.
>
> **Correction, same day.** The original "22.4% of ticks show a near-field
> jump" figure was itself a measurement artifact (dominated by ordinary
> arc-length resampling on a curving path, not the defect) and has been
> retracted — two targeted fix attempts against it moved nothing, which is
> what exposed the flawed metric.
>
> A corrected metric (near-field path *tangent direction*, cross-checked
> against `e_psi`/`steer_deg` to rule out genuine corners) found the real
> effect is real but modest: **3 large single-tick tangent jumps out of 4160
> ticks (0.07%)** in the checked log, and the original t=74.83/74.98 instance
> re-measured at a ~17° tangent reversal, not the "5×+ radius jump" framing
> implied on its own. The mechanism itself (byte-identical midpoints, a
> genuine chain-anchor discontinuity) is still confirmed real — only its
> measured *frequency* was wrong. No fix has been shipped at this size; see
> `docs/logs/sim_to_real_investigation.md` §19's correction note and §23 for
> the full detail, including two new lessons on trusting a metric only after
> checking it against ground truth.
>
> **Update (2026-08-07, later the same day) — a real tail effect exists, but
> it is a DIFFERENT mechanism from the one above, not this one at higher
> stakes.**
>
> `docs/logs/sim_to_real_investigation.md` §26 compared the planner's online
> reference heading against a fixed, geometry-only reference computed from
> the same rollout and found the bulk of reference-heading swing is genuine
> geometry (ratio ≈1.2 mean/p90 vs. geometric, r=0.80) — but a small tail
> (5.8% of ticks, planner rate 1.87–3.51× the geometric rate) carries an 18×
> higher immediate steering-saturation rate (42.2% vs 2.3%).
>
> §27 cross-checked this tail directly against the `min_ahead` seed-jump
> mechanism above and found only 9% (6/64) of the high-excess ticks coincide
> with one — this is **not** the same defect at higher stakes. Tracing the
> tail ticks directly instead shows a sustained turn-in lag at braking corner
> entries: the planner's online reference correctly anticipates a sharp
> corner earlier and more aggressively than the car has physically yawed
> yet, so the reference-minus-car gap grows continuously for over a second
> before closing — confirmed on all 3 flagged episodes, including one
> hairpin. This is a distinct, previously undocumented mechanism, not a
> resizing of this section's `min_ahead` finding — treat them as two separate
> open items, not one.
>
> **Update (2026-08-07, later still) — a candidate fix exists, off by
> default, and it has its own suite-safety caveat.**
>
> `docs/logs/sim_to_real_investigation.md` §28 adds a symmetric
> reference-heading rate limiter
> (`settings.REF_HEADING_RATE_LIMIT_ENABLED`/`REF_HEADING_RISE_RATE`, in
> `sim/rollout_core.py::_rate_limit_ref_psi`) that caps how fast the tracked
> reference heading may change per tick — same shape as the existing
> `SPEED_TARGET_RISE_RATE`. At 90°/s it improves saturation on both the
> recorded map and every path in `VALIDATION_SUITE` with no DNF anywhere.
>
> **Do not tighten this toward the recorded map's more dramatic numbers**
> (65–70°/s reach 0% saturation there) — both DNF `PATH_MICRO_SLALOM`
> off-track in the suite, a failure the recorded map cannot show because it
> has no fast-reversal slalom geometry. Default OFF pending a live test.
>
> **Update (2026-08-07, live test run) — tried at 90°/s, made saturation
> WORSE (21.1%/26.4% baselines → 28.0%).**
>
> `docs/logs/sim_to_real_investigation.md` §29: confirmed active on the car
> (max reference-heading rate capped 1508°/s → 220°/s) but produced a 3.77 s
> continuous saturation episode — the same failure mode §28 found offline on
> `PATH_MICRO_SLALOM`, just short of a full DNF. Holding the reference back
> during turn-in left a larger heading deficit to claw back later, worse
> than not limiting at all.
>
> Reverted to `False` in the live `mpc_core.py`. The underlying §26/§27
> measurement (reference genuinely outpaces the car's yaw) is unaffected by
> this — only this specific fix is now known not to work. Do not re-enable
> this limiter without a new offline test against a synthetic path shaped
> like this failure (a long, smoothly-growing heading deficit through a
> decelerating corner) — the recorded map and `VALIDATION_SUITE` as they
> stand did not fully predict this outcome, only partially warn about its
> existence.

### Fixed: a real cone-map duplication bug in `_absorb()`

Investigating "why lap 2 is worse" (`docs/logs/sim_to_real_investigation.md` §15) found
and fixed a genuine bug in `planning/cone_map.py::ConeMap._absorb()`: two
detections of one physical cone in the same frame, both farther than
`MERGE_DIST` (0.8 m) from anything already in the map — i.e. that cone's
first sighting — were both appended as separate, permanent entries. This is
deterministic (confirmed for detections 1 cm apart) and independent of
`MERGE_DIST` tuning, since it only compared each candidate against the
existing map, never against other candidates in the same batch. Fixed in both
copies within this repo (offline `planning/cone_map.py` and the
`fsds_simulator` mirror's `cone_map.py`) by also checking candidates against
each other before appending. **Not yet ported upstream** — the live
`fsae_planning` repo's `cone_map.py::_absorb()` still has the unfixed,
same-frame-duplicate-prone version as of its `dfd1a08` HEAD; porting this fix
there is a resync TODO, not something this repo can apply directly. Verified byte-identical
sim output when the bug cannot fire (FSDS's cone perception is a noise-free
oracle by default, so it never produces the same-frame duplicate detections
needed to trigger this). Does **not** yet establish this explains any part of
the curvature-spike defect or the saturation gap — that needs either a
measured real-detector noise figure or a live log showing actual duplicate
clustering, neither available yet. See §15 for full detail, including the new
`CONE_NOISE_ENABLED` offline testing capability this fix was verified against.

### Related but eliminated: `blend_paths`' reset-bypass discontinuity

`path_utils.py::blend_paths()` (used by both `centerline_planner.py` and
`sim/sim_track.py`, `alpha=0.4`) exists to stop the from-scratch rebuild every
pose tick from producing a heading jump. It has a `reset_dist=2.0` m bypass
that skips the blend entirely when the rebuild has moved too far from the
previous publish — plausibly correlated with this section's curvature-spike
defect, since a spike event is exactly when the rebuild changes most.
Measured on the offline sim (see
`docs/logs/sim_to_real_investigation.md` §14): real, can jump the reference up to 166°
on other geometries, but **fires 0/1038 times on the recorded map** (max
trigger-distance 1.98 m, just under the threshold) — so it cannot explain
that map's saturation gap. Re-check if a planner fix here changes rebuild
volatility enough to push the recorded map over 2.0 m.

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

**A sibling bug, same root cause class, found and fixed 2026-08-08** (see
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

An earlier revision of this section guessed SLAM pose noise was the likely
remaining cause. **That was wrong** and is corrected here: the log came from
FSDS, where `sim_perception` republishes ground-truth odom, so the pose was
exact — just stale. Staleness is not noise. `SLAM_NOISE_ENABLED` exists to
model the real car's localisation error specifically — this conclusion about
that specific FSDS log is unaffected by the flag's current default (which
changed to on 2026-08-08, see the "Simulator fidelity limits" table above).

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

`METRIC_SCALES` (added 2026-08-06) divides each metric by a reference magnitude
before weighting: `score = SCORE_WEIGHTS @ (metrics / METRIC_SCALES)`. It exists
because without it a metric's real influence is `weight × typical magnitude`,
which had made the composite effectively single-objective — all ten non-tracking
metrics combined moved the score by +0.0064 against a −0.2649 tracking term, so
the smoothness and oscillation terms could not bite regardless of their weights.
Because it is inlined in **three** places (`settings.py`, the live
`fsae_control/scoring.py`, and the `fsds_simulator/` mirror), a change to it is
a three-file edit — the same rule as `SCORE_WEIGHTS`.

**`progress`/`reached_end`/`time_bonus` are now computed live too (2026-08-11
fix).** Previously both live controller nodes called `telemetry.close()` with
no arguments, so `progress` defaulted to `0.0` and `reached_end` to `None` —
`compute_composite_score()` reads that as "never finished" and every live run
scored exactly `CONSTRAINT_FLOOR + DNF_PENALTY = 13.0`, regardless of how the
car actually drove (the 13 underlying quality metrics were still computed
correctly; only the composite number was dead). Root cause and fix are logged
in [`docs/logs/sim_to_real_investigation.md`](logs/sim_to_real_investigation.md)'s findings
table (the "Live scorer reports `13.0`" row, now marked fixed).

The fix is `LapProgressTracker` in `telemetry_logger.py`: it tracks the car's
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
The weighted-metric component (the 13 metrics × `SCORE_WEIGHTS` — added
`accel_reversal_rms` 2026-08-08, see the "Score weights / bonuses / penalties"
row above) is directly comparable either way; only the bonus/penalty terms
differ, and now only `offtrack` is unconditionally unavailable.

## The sim-to-real gap: CAUSE FOUND, fix not yet applied

> **Full investigation history** — every hypothesis tried, why each looked
> right, and how it was eliminated — is in
> [`docs/logs/sim_to_real_investigation.md`](logs/sim_to_real_investigation.md). Read that
> before re-testing any candidate that looks unexplored; most already were.

Measured 2026-08-06 on the recorded `comp test map 3`, same tuned gains both
sides:

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
| **`SteeringCurve` (UE4/PhysX speed-dependent steering scaling)** | **No** — read directly from `TechnionCarPawn`'s `WheeledVehicleMovementComponent4W` in the UE4 Editor (2026-08-07): flat at 1.0 across all 3 keyframes, no speed-dependent scaling at all. See `docs/logs/sim_to_real_investigation.md` §18/§20/§24/§25 for the full mechanism search, the UE 4.27 build process needed to check it, and the read-out |

**ROOT CAUSE FOUND** (2026-08-06, open-loop system-ID + step test):
**FSDS enforces a sustained LATERAL-ACCELERATION ceiling of ~7.5 m/s².**
Below ~6 m/s the car never reaches it and the commanded steering angle is
delivered exactly (`s` = 1.00–1.01 at 3 and 5 m/s); above it the yaw response
collapses (`s` = 0.34 at 8 m/s, 0.17 at 14 m/s). It is far below the 12.3 m/s²
the same car reaches on a lap, so it is **not** tyre saturation — a grip limit
does not depend on speed.

The sweep alone read this as a *yaw-rate* cap (~0.7 rad/s); the step test
showed **lateral acceleration** is what is held constant (1.07× spread across
speeds vs 1.56× for yaw rate). The two are indistinguishable at a single speed.

**Now modelled** in `model/vehicle_physics.py` (`alat_ceiling*`). It moves every
metric toward the car but closes only part of the gap — live still saturates 3×
more often. See "MECHANISM: a dynamically-enforced lateral-acceleration
ceiling" below, and `docs/logs/sim_to_real_investigation.md` for the full history.

The pose-feed hold is real (see `PoseFeedHold`) and is now modelled, but it
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

Measured 2026-08-06 from `fsae_logs/mpc_standalone_control_1786007642.csv` and
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

#### The open-loop experiment (built, not yet run)

**Where the three pieces live** — the node and harness are now mirrored too
(2026-08-09, see "Last resynced" above); the analysis script never had
anything to mirror. See "Why none of it is mirrored" below for why the first
two were absent until this resync.

| file | repo | mirror location | role |
|---|---|---|---|
| `control/fsae_control/fsae_control/steering_sysid.py` | `fsae_planning` (live ROS 2 ws) | `fsae_MPCTest/fsds_simulator/control/fsae_control/fsae_control/steering_sysid.py` | the node — drives FSDS directly |
| `ros2/run_steering_sysid.sh` | **FSDS repo root**, next to `launch_all.sh` | `fsae_MPCTest/fsds_simulator/run_steering_sysid.sh` (adapted paths, same convention as `launch_all.sh`) | one-command harness |
| `tuner/checks/steering_sysid_analysis.py` | `fsae_MPCTest` | *(none — already lives here)* | reads the log, names the mechanism |

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

##### Why none of it was mirrored (historical — reversed 2026-08-09)

`fsds_simulator/` is a PR-staging snapshot, and its rule is "do not add files
that were never there". At the time this section was written, all three
files were new and none were mirrored, for the reasons below. **This is no
longer current** — `steering_sysid.py` and `run_steering_sysid.sh` (plus the
equivalent `steering_step.py`/`run_steering_step.sh` pair) were genuinely
mirrored 2026-08-09 at explicit user request; see "Last resynced" at the top
of this document and the file-mapping table above for where they landed.
Kept below for the record, since the *reasoning* (not the outcome) is still
useful context for judging future "should this diagnostic be mirrored?"
calls:

- `steering_sysid.py` — a diagnostic, not part of the car's runtime stack.
  That is still true; it was mirrored anyway because the user wants this
  folder to be a complete, cloneable copy of the working setup, diagnostics
  included, not strictly the runtime stack.
- `run_steering_sysid.sh` — the mirror *does* carry `launch_all.sh`, but that
  copy is **intentionally adapted** (different Windows username, its own
  `cone_maps` path, different resolution), not a sync target. Copying the
  sysid harness created a second file needing the same manual divergence —
  done anyway 2026-08-09, adapted the same way (username, `LOG_DIR` pointed
  at `$HOST_REPO_ROOT/fsae_logs` to match `launch_all.sh`'s `log_dir:=` arg).
- `steering_sysid_analysis.py` — lives in `fsae_MPCTest` only; nothing to
  mirror. Still true, unaffected by the 2026-08-09 resync.

The one knock-on **used to be**: `setup.py`'s `steering_sysid` entry point
was deliberately omitted from the mirror's copy, since the module wasn't
there. Both node modules are there now, so both entry points (`steering_sysid`
and `steering_step`) are registered — see the `setup.py` row in the
file-mapping table.

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
prediction horizon — this is a pre-existing architectural characteristic,
not something introduced by this sync, and not a bug. See `README.md`'s
state-vector section (search "e_v's target speed is frozen for the whole
horizon") for the full explanation; not repeated here to avoid duplication.

## Exit-heading boost was firing at the wrong time (fixed 2026-08-11, confirmed live)

**Fixed, small measured offline improvement, confirmed performing well on
the live car same day** (dynamic speed cap left OFF for this run — see
below — so this result is attributable to the exit-boost fix alone).
Reported symptom: after exiting a corner, the car is sometimes left pointed
slightly off the path tangent, and accelerating out of the corner in that
state produces drift/slip.

Root cause: `_lookahead_exit_boost()`'s 5 m decay window (default
`MPCParams.adaptive_q_lookahead_exit_decay_dist`) is meant to boost `Q[2,2]`
(heading-error cost) right after the car passes a corner's peak curvature, so
the MPC works harder to straighten out on exit. Its decay clock
(`_dist_since_peak`) was reset by `_update_lookahead_peak()` off
`kappa_max_abs` — the **lookahead-window** peak curvature, detected the
moment a corner enters the speed-scaled scan window (10–17 m ahead of the car
at speed, per `MPCParams.adaptive_q_lookahead_time_s`/`_dist_max`). That
means the decay clock started counting down 10–17 m before the car
physically reached the corner. Verified on live telemetry
(`fsae_logs/mpc_standalone_control_1786436674.csv`): at the corner starting
around t=3.28s, `dist_since_peak` reset to 2.56 at the moment the lookahead
first saw rising curvature, then counted up continuously through the car's
actual apex (t≈5.2s, where current-position `kappa` itself peaked at 0.175
and heading error was worst) — by then `dist_since_peak` was already past
28 m, decades beyond the 5 m decay window. The exit-heading boost had
already decayed to a no-op (`1.0`, i.e. contributing nothing) before the car
was anywhere near its physical exit. This is not merely "a bit early" — on
a continuous chain of corners (curvature never drops back to the
`adaptive_q_lookahead_peak_hysteresis` re-arm threshold between them), the
detector effectively never gets a chance to fire again mid-sequence either,
per the `dist_since_peak` trace above.

Fix: `_update_lookahead_peak()` (`mpc_core.py`, mirrored
`model_utils.update_lookahead_peak()`) is now keyed on **current-position**
`kappa` (the same near-instantaneous, ~1 m preview curvature
`_adaptive_R_rate`/`_steer_rate_anti_hunt` already use) instead of
`kappa_max_abs`. `kappa` peaks at the car's own physical apex by
construction, so the decay clock — and therefore the exit-heading boost —
now actually covers the physical exit instead of having already elapsed
before it. The re-arm/hysteresis check was moved onto the same signal for
internal consistency (armed once the car itself is on a straight, not just
once the far lookahead window is clear). `_lookahead_exit_boost()` itself,
and its `k_exit_norm`/`boost_max` parameters, are unchanged — only the
timing of when its decay countdown starts.

Measured (`tuner.recorded_map_rollout --planner`, the `USE_PLANNER=True`
branch where this mechanism is actually exercised — see the dynamic-speed-cap
section below for why the default oracle-only invocation never reaches this
code path at all):

| metric | before | after |
|---|---|---|
| steering sat % | 5.54 | **5.46** |
| `\|e_psi\|` mean / p90 (deg) | 8.01 / 16.82 | **7.60 / 15.74** |
| a_lat max | 10.61 | **10.55** |
| score (lower better) | 0.520 | **0.517** |

Small but consistently in the right direction on every metric offline
(unlike the dynamic speed cap and the abandoned heading-misalignment accel
gate below, both of which improved one metric while regressing others).
This is a timing bug fix to an existing, previously-inert mechanism, not a
new tunable — no new `MPCParams`/`settings.py` fields were added.

**Live test, 2026-08-11 (same day): confirmed performing well**, with
`ENABLE_DYNAMIC_SPEED_CAP=false` (see the dynamic-speed-cap section below —
that mechanism stayed disabled for this run), so the improvement is
attributable to this exit-boost timing fix specifically, not a combination.
No quantitative before/after live log pair was captured for this change
specifically (unlike the dynamic-speed-cap A/B, which had matched control
logs) — "performing well" is qualitative/subjective from this run. The
residual gap is still expected to be non-zero: this fixes one specific
mistiming, not the broader reference-heading-lead issue CLAUDE.md already
flags as open (§"Still open" in the sim-to-real section) — re-read that
section before assuming this closes the corner-exit-misalignment complaint
entirely.

**v2 fix, same day: the 5m decay window was still too short even after the
timing fix above.** After the `r_a` cut below made the car accelerate
harder out of corners, a live log
(`fsae_logs/mpc_standalone_control_1786443033.csv`) showed 10 stretches of
`|e_y| > 0.5` m with `m_Q_epsi_exit == 1.00` (i.e. NO boost applied) at
every single one — `dist_since_peak` at the moment `|e_y|` peaked was
11.7-20.6 m (mean ~15.7 m, one cluster of outliers at ~45 m attributable to
a different corner's peak already having re-armed), decades past the
now-correctly-timed 5 m window. Root cause: even keyed on the true apex,
`|e_y|`/`|e_psi|` don't peak instantaneously AT the apex — the car is still
sliding wide/yawing back through the exit for 1.5-2.7 s of travel
afterward, and at 5-8 m/s that is well over 5 m. The fixed decay window was
simply too short for how long a real exit disturbance actually takes to
play out, independent of the timing-origin bug already fixed.

Fix: `adaptive_q_lookahead_exit_decay_dist` is now a FLOOR, not the whole
story — the actual decay window used is
`car_speed * adaptive_q_lookahead_exit_decay_time_s` (default `2.5` s, fit
to the ~15.7 m mean from the cluster above), clamped to
`[adaptive_q_lookahead_exit_decay_dist, adaptive_q_lookahead_exit_decay_dist_max]`
(`5`–`25` m default) — same shape as the approach-side `lookahead_dist`
(`adaptive_q_lookahead_time_s`/`_dist_min`/`_dist_max`). Applied in
`mpc_core.py`'s `compute()` (computed at the `_lookahead_exit_boost` call
site, `_lookahead_exit_boost()` itself unchanged) and offline as three new
`adaptive_Q_lookahead()` kwargs (`exit_decay_dist_floor`/`_time_s`/`_max`)
threaded from `settings.py`'s new `ADAPTIVE_Q_LOOKAHEAD_EXIT_DECAY_DIST`/
`_TIME_S`/`_DIST_MAX` constants — mirrored to `fsds_simulator`.

While implementing this, found (but did NOT fix, out of scope) that offline
`adaptive_Q_lookahead()`'s call to `lookahead_exit_boost()` had never
threaded `k_exit_norm`/`boost_max` from `settings.py` either — it silently
used that function's own hardcoded defaults the whole time, unlike every
other lookahead call in the same function. Pre-existing drift, not
introduced by this change; flagging for whoever next touches this function.

Measured (`tuner.recorded_map_rollout --planner`, same track/config as the
v1 table above, now on top of the `r_a=0.77` cut below): score 0.499→0.497,
lap time 55.15s→54.95s, steering sat 4.35%→4.37% (flat). Small further
improvement, no regression on the oracle-path baseline (0.408→0.409).
**Not yet tested live.**

**Investigated and explicitly rejected as a further fix for the same
symptom (2026-08-11):** raising `R[1,1]` (acceleration effort) and/or
relaxing `Q[4,4]` (`e_v`, speed-tracking urgency) when CURRENT `|e_psi|` is
large, so the MPC is reluctant to accelerate hard while still visibly
misaligned. Implemented in both `mpc_core.py`/`model_utils.py` as
`_epsi_misalignment_accel_gate`/`_epsi_misalignment_speed_relax` (`R[1,1]`
boost) and their `Q[4,4]` counterpart, gated by a new
`epsi_accel_gate_enabled` flag defaulted off. At the first-tried gains
(`boost_max=3.0`, `speed_floor=0.4`, `k_epsi=15.0`, half-effect ~3.8° of
`e_psi`), offline testing showed a clear regression, not an improvement:
steering sat 5.54%→7.70%, `|e_psi|` mean 8.01°→9.50°, score 0.520→0.660.
Reverted entirely (no trace of `epsi_accel_gate_*` remains in any file) —
this was not committed as a disabled feature the way the dynamic speed cap
below was, because unlike that mechanism it showed no redeeming metric at
all, only a uniform regression. If revisiting a steering/accel exit penalty
in the future, do not re-derive this same current-|e_psi|-gated `R[1,1]`
approach without accounting for why it made things worse — a likely
candidate (never diagnosed) is that relaxing `Q[4,4]` let the car dawdle at
the wrong speed while the already-existing heading-correction machinery
(the exit-boost fix above, and the anti-hunt `boost_epsi` term) was doing
its own uncoordinated thing on the same `e_psi` signal.

## MPC underaccelerating on clean straights (r_a 0.85 → 0.77, 2026-08-11)

**Fixed offline, applied to live default, not yet tested live.** Reported
symptom: the MPC leaves lap time on the table in some places, not going as
fast as it could.

Root cause: the same structural effort/benefit mismatch already documented
above for `R_diag[1]`'s 2026-08-10 braking fix (`a_cmd` is a RATE — one 50ms
QP step at even |a_cmd|=6 only changes speed by 0.30 m/s, while the effort
cost `R[1,1]*a_cmd²` is paid immediately, so the QP structurally prefers
small, cheap accel steps over large ones unless the single-step benefit is
large) applies symmetrically to ACCELERATION, not just braking, and had
never been retuned for that side.

Confirmed on live telemetry (`fsae_logs/mpc_standalone_control_1786440962.csv`):
during t≈67.6-69.2s, the car is cleanly on-line (`e_y` -0.03 to 0.02 m,
`e_psi` 1-4°, steering ~1°, no competing lateral/heading demand) with a
3-9 m/s speed deficit (`v_desired` climbing toward 16+ m/s while `v_actual`
sits at 7-8), yet `a_cmd` peaks at only ~3.1 m/s² and then decays back down
even as the deficit stays large — well under the 12 m/s² ceiling the same
lap demonstrably used elsewhere (e.g. from a standing start at t<2s).

Measured acceleration (`dv/dt` from `v_actual`) in the same window actually
*exceeds* `a_cmd` (6-9 m/s² achieved vs ~2-3 commanded), ruling out
actuator/throttle capping as the cause — the QP itself is choosing the
conservative command, the car isn't failing to deliver on it.

Swept `R_diag[1]` (`r_a`) offline via `tuner.recorded_map_rollout --planner`
on `comp_test_map_3`, holding `R_rate_diag`/`Q_diag` fixed:

| `r_a` | steering sat % | `\|e_psi\|` mean (deg) | score (lower better) | lap time (n_steps×0.05s) |
|---|---|---|---|---|
| 0.85 (old default) | 5.46 | 7.60 | 0.517 | 55.85 s |
| **0.77 (new default)** | **4.35** | **6.74** | **0.499** | **55.15 s** |
| 0.71 | 5.71 | 7.09 | 0.516 | 55.15 s |
| 0.65 | 6.90 | 8.76 | 0.551 | 56.55 s |

0.77 is a local optimum on this run, not a point on a monotonic
"lower-is-better" curve — 0.71 and 0.65 both score worse. No regression on
the oracle-path (`USE_PLANNER=False`) baseline (0.408→0.409, within noise).
`R_rate_diag[1]` (`r_rate_a`) was also tried at 2.2 alongside `r_a=0.77` —
no additional benefit over `r_a` alone (score 0.501 vs 0.499) — left
unchanged at 2.6.

Applied to `mpc_params.py`'s `r_a` default (live), `settings.py`'s
`R_diag[1]` (offline), `fsae_params.yaml`'s `controller.r_a`, the
`fsds_simulator` mirror, and `ros2/launch_all.sh`'s MPC tuning shortlist
(`MPC_R_A`, commented out by default like the other shortlist entries).

**Confirmed live, 2026-08-11 (same day):** `lap_time_s` 69.99 s → 59.52 s
(15% faster) on a like-for-like comparison against the previous live log
(which already had the exit-boost v1 timing fix but not this `r_a` cut —
isolating this change's effect). Real trade-off, not a free win: tracking
got noticeably worse in the same comparison — RMSE 0.199 → 0.336 m, peak
`|e_y|` 0.84 → 1.26 m, `steering_sat_ratio` 1.4% → 3.0% — so `composite_score`
is roughly flat (0.638 → 0.641) despite the large time gain, because the
scoring formula weights both speed and tracking quality. The user confirmed
the tracking regression was concentrated specifically at corner EXITS (car
now accelerates harder while still not fully realigned) rather than diffuse
across the lap — this is what motivated the exit-boost v2 fix above (same
log). Only tested on one track (`comp_test_map_3`); if this drifts on a
different map, re-sweep rather than assume 0.77 transfers everywhere.

## Accel/brake effort weight split (2026-08-12)

**Implemented on both sides; the resulting weights are actively being
live-tuned — treat any specific value quoted here as a snapshot, not
current truth.** Reported symptom, after the `r_a=0.77` cut above was
confirmed live: the same shared weight that freed up acceleration also
weakened braking by the same amount, since `R_diag[1]` was applied
symmetrically to `|a_cmd|` regardless of sign. Live telemetry after the
`r_a` cut showed the resulting asymmetry directly — a corner-entry log
(`mpc_standalone_control_1786444690.csv`) had the car arriving faster into
corners (accel side now eager) while `a_cmd` still floored around -1.4 m/s²
during a sustained 2-second, 3-5 m/s speed deficit (braking side unchanged
and still weak) — too hot into corners, followed by steering saturation and
an unstable post-saturation recovery.

**Mechanism**: `R_diag[1]`'s single scalar weight was replaced with two
independent weights, `r_a_accel` (`a_cmd >= 0`) and `r_a_brake` (`a_cmd <
0`), applied via `cp.pos(u[1,:])`/`cp.neg(u[1,:])` in the QP cost —
`R_a_accel * sum(pos(a_cmd)²) + R_a_brake * sum(neg(a_cmd)²)` — rather than
new slack variables or constraints (the originally-considered
slack-variable design was replaced with this simpler `cp.pos`/`cp.neg`
rewrite once it was confirmed DCP-valid and numerically identical to the old
single-weight cost when `r_a_accel == r_a_brake`, since `pos(x)²+neg(x)²
== x²` for any real `x`). Implemented in:

- **Live**: `mpc_core.py`'s `_build_qp`/`_solve_qp` (new `r_a_accel_param`/
  `r_a_brake_param` `cp.Parameter`s), `mpc_params.py`'s `r_a_accel`/
  `r_a_brake` fields (replacing the single `r_a` field), `fsae_params.yaml`'s
  `controller.r_a_accel`/`r_a_brake` (replacing `controller.r_a`),
  `launch_all.sh`'s `MPC_R_A_ACCEL`/`MPC_R_A_BRAKE` shortlist entries
  (replacing `MPC_R_A`).
- **Offline**: `controller/optimiser.py`'s `init_parameterized_mpc`/
  `solve_mpc` (same `cp.pos`/`cp.neg` split, new `r_a_accel`/`r_a_brake`
  kwargs defaulting to `R[1,1]` when omitted, for backward compatibility
  with any caller that doesn't pass them), `settings.py`'s `R_A_ACCEL`/
  `R_A_BRAKE` (read by `sim/rollout_core.py`'s `solve_mpc()` call; `R_diag[1]`
  itself is now a nominal value only, read as the fallback default and by
  callers that don't know about the split).

`self.R[1,1]` / `R_diag[1]` are kept as nominal/reporting values only — no
adaptive gain (`_adaptive_R_scaling`, `_adaptive_R_rate`, etc.) touches index
1 of `R`/`R_rate` anywhere in this codebase, confirmed by direct grep before
implementing, so the split composes cleanly with every existing adaptive
mechanism without any interaction to account for.

**Re-check `mpc_params.py`'s `r_a_accel`/`r_a_brake` and `settings.py`'s
`R_A_ACCEL`/`R_A_BRAKE` for the current values before relying on this
section** — these are being adjusted directly during live testing (observed
moving 0.35/0.2 → 0.5/0.2 → 0.5/0.1 → 1.0/0.6 across one session), faster
than this doc can track. Sync `settings.py` to match `mpc_params.py`'s
current live values after each live-tuning session, per CLAUDE.md's parity
rule.

## Low-speed steering-rate boost (added and disabled same day, 2026-08-12)

**Added, live-tested, found to have an unwanted side effect, disabled by
default — code stays in place for a future rework.** Reported symptom: after
exiting a corner at low speed (3-4 m/s), steering swung through a large,
fast, under-damped correction while accelerating — confirmed on
`mpc_standalone_control_1786483673.csv`, t=6.9-7.7s: `steer_deg` swings
+25° → -9° → 0° over ~1.5s while `a_cmd` climbs 0 → 2.15 m/s², with
`Rrate_steer_eff` essentially flat (~1.8-2.2) throughout — neither
`_adaptive_R_rate` nor `_steer_rate_anti_hunt` (both gated on curvature/
tracking-error, not speed) meaningfully reacted, since the wobble's `kappa`
was already small (car past the apex) by the time it happened.

**Mechanism (as implemented)**: `_low_speed_steer_rate_boost(vx, ...)` —
INVERTED from Stanley's `k/(v+eps)` correction-gain shape (cheap correction
at low speed): this instead makes steering-RATE changes MORE expensive at
low speed (`R_rate[0,0] *= 1 + (boost_max-1)/(1+k*vx)`, `boost_max=2.5,
k=0.35` → ~1.73× at 3 m/s, ~1.0× by race speed), on the theory that a fast
swing matters more when the car has little momentum to resist it. A literal
Stanley-shaped (cheap-at-low-speed) mechanism was explicitly considered and
rejected before implementing this one — see the mechanism note in
`mpc_core.py`'s `_low_speed_steer_rate_boost` docstring for why.

**Live-tested same day, found to regress turn-in**: because this gates
purely on speed with no curvature/lookahead signal, it cannot distinguish
"post-exit overcorrection at low speed" (the case it was built for) from
"turn-in at low speed" (also low speed, also needs a fast steering-rate
change, but wanted) — live driving reported the car "struggling to turn
early in turns" after this was enabled, i.e. it suppressed the two cases
identically. **Disabled** (`low_speed_steer_rate_boost_enabled=False` in
both `mpc_params.py` and `settings.py`, plus the `fsds_simulator` mirror and
`fsae_params.yaml`) same day. The function, its `MPCParams`/`settings.py`
fields, and its telemetry column (`m_Rrate_lowspeed`) are all left in place
at their designed values (`boost_max=2.5, k=0.35`) rather than removed, so a
future lookahead-curvature-gated rework (fire only when NOT
approaching/inside a corner) doesn't need to re-derive the shape from
scratch.

## Steering-effort relaxation approaching a corner (2026-08-12)

**Implemented on both sides, not yet live-tested in isolation.** Reported
symptom (a follow-on diagnosis from the same live session): the car is slow
to commit to turn-in specifically at higher corner-entry speed.

**Root cause**: `_adaptive_R_scaling`'s speed-dependent steering-effort
penalty (`R[0,0] *= 1 + 1.5*vx/(6+vx)`, e.g. ~2.07× at 15 m/s) has no
lookahead relief at all — it stays at full strength right through an
approaching corner regardless of curvature. The only other mechanism
touching `R[0,0]`, `_steer_effort_straight_boost`, only ever RAISES it (on a
clear straight) or relaxes back to the unscaled baseline as a corner is
detected — neither one ever pushes `R[0,0]` BELOW baseline for an
approaching corner. A car entering a corner hot therefore paid the full
speed-based steering-effort penalty at exactly the moment it most needed to
commit to turn-in. (`_lookahead_yaw_rate_relax` already does the equivalent
relief for `Q[3,3]`/yaw-rate — its own docstring explicitly names "turns
late/slowly" as the failure mode it exists to prevent — but no `R[0,0]`
counterpart existed until now.)

**Mechanism**: `_lookahead_steer_effort_relax(kappa_max_abs, car_speed,
floor=0.5, ...)` — mirrors `_lookahead_yaw_rate_relax`'s shape exactly (same
demand-normalised corner-severity curve), falling from `1.0` (no corner
ahead) toward `floor=0.5` as corner demand rises, composing multiplicatively
with `_adaptive_R_scaling` and `_steer_effort_straight_boost`'s existing
`R[0,0]` scalings. `floor=0.5` matches `_lookahead_yaw_rate_relax`'s own
default floor (same magnitude as the sibling mechanism this is modelled on).
Implemented in `mpc_core.py`/`mpc_params.py`
(`lookahead_steer_effort_relax_enabled`,
`adaptive_q_lookahead_steer_relax_floor`), mirrored in `model_utils.py`
(`lookahead_steer_effort_relax`) / `sim/rollout_core.py` /
`settings.py` (`LOOKAHEAD_STEER_EFFORT_RELAX_ENABLED`,
`ADAPTIVE_Q_LOOKAHEAD_STEER_RELAX_FLOOR`), and the `fsds_simulator` mirror.

## Curvature-forcing term: the QP's own prediction was blind to the path bending ahead (2026-08-12)

**Implemented on both sides, offline smoke-tested (no crash), live symptom
not yet re-confirmed fixed.** Reported symptom, after the steering-effort
relax above and the `ey_k` fix below were both live-tested: the car still
turned in late on sudden/sharp corners.

**Root cause (deeper than either fix above)**: every existing lookahead
mechanism (`adaptive_Q_lookahead`, `lookahead_steer_effort_relax`, etc.)
only reweights an *existing* tracking error — it changes how expensive
being off-line already is, never the QP's own predicted trajectory. The
QP's internal dynamics model (`Ad`/`Bd`) has *no path-curvature term at
all*: with `e_y ≈ e_psi ≈ 0` (car dead on-line on the still-straight
approach, exactly the state before any real corner), the QP's own 35-step
rollout predicts staying at `≈0` for the whole horizon regardless of how
sharply the real path bends ahead. No amount of cheaper steering effort
can fix this, because there is no predicted error yet for a cheaper weight
to act on — this is why the steering-effort relax fix above, while
correctly implemented and firing, could not fully close the gap on its
own. Confirmed directly: `m_R_steer_relax` and `Q_ey_eff` moved correctly
and over a full second early in live telemetry, while `steer_deg` stayed
at ≈0° the whole time, because there was no `e_y`/`e_psi` for those
reweighted costs to react to.

**Mechanism**: `_curvature_horizon_profile`/`curvature_horizon_profile`
walks the reference path forward from the car's current position by the
PREDICTED arc-length at each of the QP's `N` steps (`v_x·k·dt`), returning
signed curvature at each step (distinct from `_lookahead_curvature_profile`,
which only returns the single peak value used for the Q/R reweighting
above). That per-step curvature feeds a new forcing term added directly to
the dynamics constraint — `x[:,1:] == Ad@x[:,:-1] + Bd@u + w`, where `w` is
zero everywhere except the `e_psi` row: `w[2,k] = -v_x·κ(s_k)·dt·gain`. The
sign follows directly from `e_psi = car_yaw - path_yaw` and
`path_yaw_rate = v_x·κ`: a path curving left (`κ>0`) drives `e_psi`
negative over the horizon even if the car holds a constant heading. `w=None`
(the default for any caller that doesn't pass it) sends an all-zero array,
making this an exact no-op — existing callers are unaffected.

**Verified with a synthetic constant-curvature path** before trusting the
sign: with `curvature_forcing_enabled=False`, commanded steering is
exactly `0.000°` 24 m before a 20 m-radius left bend with zero tracking
error (reproducing the diagnosed bug); with it `True`, steering correctly
leans toward the bend (`+1.5°` at 17 m/s, before any `e_y`/`e_psi` error
exists) — and mirrored for a right bend (`-1.5°`). A closed-loop
simulation (the car actually driving toward the bend using its own
commands, not a single frozen snapshot) confirms the controller commits to
real, sustained left steering (ramping through `+2°` → `+8.6°`) well
before reaching the curved section.

**A genuinely confusing artifact was found and ruled out during
verification**: at the exact instant `x0=0` with no history, the very
first commanded step can be a tiny (hundredths-of-a-degree), momentarily
WRONG-SIGN "flinch" before the plan settles into the correct direction —
e.g. `-0.01°` at the first tick, then correctly positive from the second
tick onward. This is a real, known linear-MPC phenomenon (the coupled
`e_y`/`e_psi`/`r` state dynamics briefly trade off a different combination
of costs at `x0` exactly zero) and is negligible in magnitude compared to
the real turn-in signal — confirmed via the closed-loop simulation above,
where it never shows up as a problem once the car is actually driving
(nonzero `x0` history from the previous tick's plan).

**A second problem was found DURING live testing of this fix**: even with
the forcing term firing correctly and early (`w_epsi_sum` already
`-1.0` to `-1.35` more than 2 s before a sharp corner, confirmed in
telemetry), the car still turned in late. Root cause: `_steer_rate_anti_hunt`
(see the anti-hunt section above) was tuned before this forcing term
existed — it reads "`e_y`/`e_psi`/current `kappa` all near zero" as
"nothing happening, dampen any steering-rate change," which is now exactly
the state the forcing term deliberately produces on approach. Live
telemetry showed steering oscillating ±10° with no net commitment while
`m_Rrate_antihunt` sat at `1.2×`–`3.3×` throughout the approach — anti-hunt
was actively cancelling the forcing term's whole effect, every tick, right
up until real curvature arrived and steering had to snap to the 25° stop
anyway. Fixed by adding a fourth, `kappa_max_abs`-gated factor to
`_steer_rate_anti_hunt`/`steer_rate_anti_hunt` (`anti_hunt_k_lookahead`,
default `60.0`, same `k` as the existing current-curvature term) that
relaxes the anti-hunt boost once a real corner is detected in the
lookahead window — not just once the car is already turning through it.

**Implemented in**: live `mpc_core.py`/`mpc_params.py`
(`curvature_forcing_enabled`, `curvature_forcing_gain`,
`anti_hunt_k_lookahead`), offline `controller/optimiser.py`'s
`init_parameterized_mpc`/`solve_mpc` (new `w` parameter) and
`controller/model_utils.py` (`curvature_horizon_profile`,
`steer_rate_anti_hunt`'s new `kappa_max_abs`/`k_lookahead` params),
`sim/rollout_core.py`, `settings.py` (`CURVATURE_FORCING_ENABLED`,
`CURVATURE_FORCING_GAIN`, `ANTI_HUNT_K_LOOKAHEAD`), `fsae_params.yaml`,
`launch_all.sh`'s shortlist, and the `fsds_simulator` mirror.

**Re-tested live (2026-08-12) — regression found and fixed: `anti_hunt_k_lookahead=60.0`
was itself too aggressive.** The combined fix made things *worse*: still no
earlier net turn-in, plus a new symptom — brief wrong-direction (rightward)
steering flicks right before some left corners, occasionally costing enough
line to miss the corner.

Log analysis (`mpc_standalone_control_1786509640.csv`) found the actual
mechanism has nothing to do with curvature-forcing's sign or magnitude —
`w_epsi_sum` was firing correctly and its tick-to-tick drift was negligible
(±0.01–0.02) throughout the approach. Instead, `steer_deg` was swinging
±9° almost every tick with an exactly-repeating pattern, the signature of a
**pre-existing, already-documented mechanism**: `predict_ahead()`'s
rollforward (see `mpc_core.py`'s "Delay compensation" comment) compounds
pose noise through `n_delay` steps with no ground-truth correction, and
that noise reaches the steering command directly at small `e_y`/`e_psi` —
"steer swinging +-5-10 deg per tick... causing late turn-in and running
wide," exactly as that comment already warned, unrelated to today's work.

What changed today is how much that pre-existing noise gets damped.
`boost_lookahead = 1/(1 + k_lookahead·|kappa_max_abs|)` with `k=60` already
cuts anti-hunt's damping in half at `kappa_max_abs=0.02` — a corner still
far outside the window curvature-forcing actually needs help fighting
through (`kappa_max_abs` there is usually 0.06+ by the time forcing's
correction matters). Comparing the same approach-window statistics before
vs. after the gate: mean anti-hunt multiplier fell at **every** curvature
level (e.g. `kappa_max_abs<0.02`: `2.19×→1.50×`), and mean `|Δsteer_deg|`
per tick rose correspondingly (`2.74→3.46`, `2.63→2.93`, `3.59→4.26`) —
confirming the gate was relaxing damping broadly, not selectively where
curvature-forcing's early correction needed room, letting the pre-existing
noise oscillate more everywhere. Reversal *rate* alone looked unchanged
(0.171 vs 0.174) precisely because this is amplified-noise, not new noise —
the amplitude grew, not the frequency, which is why it read as "still late"
(bigger, noisier swings, no more net commitment) rather than a raw increase
in direction changes.

**Fix**: lowered `anti_hunt_k_lookahead` from `60.0` to `15.0` on both
sides (`mpc_params.py`, `settings.py`, `fsae_params.yaml`,
`launch_all.sh`'s shortlist, `fsds_simulator` mirror). At `k=15`,
`boost_lookahead` stays close to `1.0` (little relaxation) until
`kappa_max_abs` is around `0.05`+, and only drops substantially past
`0.1`–`0.15` — inert during the very early, faint lookahead signal, active
once a corner is close/sharp enough for curvature-forcing's correction to
actually need the room. **Not yet re-tested live at `k=15`** — this is a
targeted correction of the over-triggering, not a re-validated tuning; the
same telemetry columns (`w_epsi_sum`, `kappa_horizon_end`,
`m_Rrate_antihunt`, and now especially `kappa_max_abs` alongside
`m_Rrate_antihunt`) are what to check on the next log.

**While investigating this, the `MPC_ANTI_HUNT_K_LOOKAHEAD` comment in
`launch_all.sh` was found to have the tuning direction backwards** ("Lower
= relaxes anti-hunt sooner/more" — actually the opposite: higher `k` means
more relaxation at a given `kappa_max_abs`, since `k` multiplies inside the
denominator). Fixed alongside this change.

### Re-tested live at `k=15` (2026-08-12) — curvature forcing itself found structurally unsound, disabled

`anti_hunt_k_lookahead=15.0` worked as intended: the same approach-window
statistics from the regression above returned close to their original
(pre-`k=60`) values (`mean_antihunt` at low `kappa_max_abs`: `2.19×` →
(regression) `1.50×` → (fixed) `2.01×`; `mean|Δsteer|`: `2.74` → `3.46` →
`2.90`). But the user still reported no real improvement in turn-in
timing, which meant the anti-hunt interaction was never the main story —
it was a second-order effect on top of a first-order problem in the
forcing term itself.

**Isolated QP testing (not a live log this time — a controlled synthetic
test, `x0=0`, no noise, no other adaptive mechanism, using the actual
`init_parameterized_mpc`/`solve_mpc`/`curvature_horizon_profile` code)
found the forcing term is unsound at any gain that matters:**

At `curvature_forcing_gain=1.0` (the "physically exact" default) with a
realistic corner (radius 13 m, 17 m/s, corner start ~15 m / ~0.9 s ahead —
comparable to the live log's approach windows), the QP's own predicted
`e_psi` trajectory only deviates a few tenths of a degree from zero across
the whole horizon, even though `w_epsi_sum` (the accumulated forcing)
reaches -1.2. The reason: `Ad`'s own `e_psi` decay (`Ad[2,2]≈0.946`/step)
bleeds off a small per-step forcing almost as fast as it's added, so it
never builds into a state deviation big enough to be worth paying real
steering-rate cost to counter. The resulting `delta_cmd` response is
sub-1° — noise-scale, exactly matching live telemetry's slightly-negative
mean steer (`-1.05°`) buried in `±4.4°` of pre-existing oscillation during
this same window. This is the mechanism behind "still doesn't turn early."

**Raising `curvature_forcing_gain` to compensate does not fix this — it
makes it worse in a new way.** Sweeping gain against the same synthetic
corner:

| gain | w_epsi_sum | steer_cmd |
|---|---|---|
| 1.0 | -1.19 | -0.27° |
| 3.0 | -3.56 | -0.96° |
| 6.0 | -7.13 | **-25.00°** (saturated AWAY from the corner) |
| 9.0–15.0 | -10.7 to -17.8 | **-25.00°** (still saturated away) |
| 20.0 | -23.8 | +25.00° (finally correct direction, but saturated, 20x the physical value) |

Inspecting the QP's full predicted trajectory at `gain=6.0` shows why:
`u[0,:]` commits immediately to `-25°` (steering AWAY from the corner) for
the first several steps, driving predicted `e_psi` to `-24°` and `e_y` to
`-2.3 m`, before reversing to `+25°` around step 6 onward. This is the
"drifts right before some left corners" symptom, reproduced exactly in a
clean, noise-free, single-mechanism test — not a live-noise artifact and
not an anti-hunt interaction.

**Root cause**: the forcing term is implemented as a disturbance on the
dynamics constraint itself (`x[:,1:] = A@x[:,:-1] + B@u + w`), which is
the same recursion the QP is minimizing total quadratic cost over. That
gives the solver freedom to choose *how* to spend/absorb the disturbance
across the whole horizon — it is not "tracking a path that bends," it is
finding the cheapest predicted trajectory subject to an artificial forcing
term, and nothing in that formulation prevents the cheapest trajectory
from being a transient swing away before correcting. This is a structural
property of forcing-via-dynamics-disturbance, not a magnitude/tuning
question — no gain between the two extremes tested is both large enough
to matter and free of the wrong-direction transient.

**Fix**: disabled `curvature_forcing_enabled` on both sides (`mpc_params.py`,
`settings.py`, `fsae_params.yaml`, `launch_all.sh`'s shortlist, `fsds_simulator`
mirror). This reverts to the pre-2026-08-12 baseline: no early anticipation,
but also no wrong-direction transient — the better-understood failure mode.
`anti_hunt_k_lookahead` was left at `15.0` (harmless no-op with forcing off,
and a real improvement over `60.0` if forcing is ever revisited).

**Code kept in place, not deleted**: `curvature_horizon_profile` (both
sides), the `w` parameter threaded through `_build_qp`/`_solve_qp`
(live) and `init_parameterized_mpc`/`solve_mpc` (offline), and the
`kappa_max_abs`-gated anti-hunt factor. A future redesign should likely
make curvature shift the *reference*/error definition (e.g. curving the
reference heading `e_psi` is measured against, so the QP's cost directly
penalises deviation from a bending reference) rather than perturbing the
same recursion the QP optimizes trajectories over. Do not re-enable
`curvature_forcing_enabled` by flipping the flag alone without re-deriving
the mechanism — the gain sweep above shows the current formulation has no
safe operating point.

**Late turn-in on sudden corners is therefore still an open problem** —
this session's attempted fix is reverted, not replaced. The
reference-heading-lead mechanism (§12.8 in `sim_to_real_investigation.md`,
also flagged in CLAUDE.md's "Still open" list) was investigated as the
next avenue — **but it applies to the live-planner branch specifically**
(the planner's per-tick, FOV-limited centreline rebuild), not to driving
against a precomputed `map_path`/`path_map_path` raceline, which is the
actual live driving configuration. On a precomputed path there is no
per-tick rebuild to swing unpredictably; the reference is fixed in
advance. Re-measured anyway out of thoroughness (see below) before this
scope mismatch was caught — the re-measurement itself is real and worth
keeping on file, but is not the fix for the precomputed-path late-turn-in
symptom.

**Re-measurement note (2026-08-12, for the live-planner branch only,
scope above notwithstanding)**: `tuner/reference_heading_geometry_check.py`
and `tuner/reference_excess_mechanism_check.py` (§26/§27 of
`sim_to_real_investigation.md`) were deleted in the same-day `tuner/`
reorg (`c182a05`) as "concluded one-off investigation scripts," but the
investigation's own "Open — mechanism confirmed real, first candidate fix
tried live and FAILED" status (its last recorded state) was never actually
closed, and — separately — an 2026-08-08 parity bug fix (§31,
`SimPlanner` never passing live-tuned smoothing/blend params) invalidated
every number §26/§27 measured, with no re-measurement ever done. Restored
both scripts from git history (`c182a05^`) and re-ran against the
corrected planner: the mechanism now measures **larger**, not smaller —
planner/geometric reference-rate ratio 1.66/1.76/6.91/12.84
(mean/p90/p99/max), correlation with true track geometry down to 0.336
(was 1.22/1.87/3.51 and 0.80 pre-fix), and 110 ticks (of 1045) show
>30°/s excess swing, only 9% explained by the known seed-jump artifact —
unchanged from before, so the remaining ~91% is still an unlocalized,
distinct planner mechanism. This is a real, larger-than-previously-known
open item **for the live-planner branch** — restored scripts are back in
`tuner/` for whoever next drives with `use_planner=True` instead of a
precomputed path.

## Lookahead corner-anticipation window widened, approach side (2026-08-12)

`adaptive_q_lookahead_dist_max` raised `17.0` → `25.0` (both sides,
`fsae_params.yaml`, `fsds_simulator` mirror) to match
`adaptive_q_lookahead_exit_decay_dist_max`, which was already `25.0` — the
approach-side ceiling was tighter than the exit-side one for no documented
reason (each field's own git history shows it was only ever set once, at
`MPCParams` centralization, ported over from whatever value predated that
with no dedicated tuning record). At typical corner-approach speed
(16-17 m/s, from `mpc_standalone_control_1786513486.csv`) the intended
lookahead (`car_speed * adaptive_q_lookahead_time_s` = 18.3-19.4 m) was
being silently clamped to 17.0 m — under 1 s of lead time regardless of
actual speed.

**This is NOT expected to fix late turn-in on its own.** `adaptive_
q_lookahead` (§4.4 in `tuning.md`) only reweights `Q[0,0]`/`Q[2,2]` on an
*existing* tracking error — with `e_y ≈ e_psi ≈ 0` on approach, a wider
window lets the boost detect the corner's curvature a little sooner, but
multiplying a still-near-zero error by a bigger number is still
near-zero. This is the same "reweighting cannot manufacture an error"
ceiling documented at length in the curvature-forcing postmortem above,
and in `junior_project_mpc_docs.md`. Expected effect, if any: the
boosts (and downstream steering commitment) trigger marginally sooner and
larger once real error/curvature *does* appear inside the window, not
before. `sim/rollout_core.py`'s hardcoded `(1.13, 3.0, 17.0)` literal was
updated to `(1.13, 3.0, 25.0)` to match — kept as a plain literal, not
sourced from a new `settings.py` constant, since `fsae_MPCTest` and
`fsae_planning` must never share an import dependency in either
direction (`fsae_planning`'s standing "no settings.py-on-the-car"
constraint applies symmetrically here). Not yet live-tested.

**A genuine fix for before-any-error anticipation on a precomputed path**
would need to change what the reference/error is measured against — e.g.
compute `e_psi` against a lookahead point on the path rather than the
nearest point — not reweight costs on the current-position error. This is
unexplored; no design or implementation exists yet.

## Straight-line lateral-error snap-back was too sharp (2026-08-12)

**Implemented on both sides (`adaptive_q_straight_ey_k` 20.0 → 8.0), not yet
live-tested in isolation.** Reported symptom, raised alongside the turn-in
diagnosis above: the car sometimes enters a corner at the wrong lateral
position relative to the planned path, as if it drifted off-line on the
approach.

**Root cause**: `_lookahead_straight_lateral_reduce` softens `Q[0,0]`
(lateral-error cost) to `ey_floor=0.7` on a clear straight, and previously
snapped back to full weight very sharply (`k=20`, deliberately much sharper
than the `k=8` shared by the `Q[2,2]`/`Q[3,3]` straight-line boosts) as soon
as any curvature entered the lookahead window. That snap-back could still
be incomplete by the time the car needed to be precisely positioned for
turn-in, so the car could still be mid-recovery from the straight-line
relaxation exactly when the corner arrived — entering already offset from
the intended line rather than from the intended centreline point.

**Fix**: lowered `adaptive_q_straight_ey_k` from `20.0` to `8.0`, matching
`adaptive_q_straight_k` (the `Q[2,2]`/`Q[3,3]` boosts' shared fade
sharpness) — the straight-line relaxation benefit itself is unchanged
(`ey_floor` still `0.7`), only the speed of the transition back to full
lateral weight as a corner approaches. Two alternative fixes were
considered and rejected in favour of this one: raising `ey_floor` (would
reduce the straight-line-hunting benefit this mechanism exists for, and
would change behaviour even far from any corner) and leaving `k` alone
while addressing this some other way. Untested live in isolation — if
straight-line hunting reappears after this change, that is the first thing
to re-check, per this section and the `adaptive_q_straight_ey_k` field
comment in `mpc_params.py`/`settings.py`.

## Gradual-corner accel oscillation is genuine track geometry, not a bug (investigated 2026-08-11)

**No fix applied — confirmed correct behaviour.** Reported symptom: through
mild/gradual turns, the car accelerates, slows, accelerates, slows
repeatedly rather than holding a smooth speed or steady acceleration.

Traced on the same live log used for the exit-boost v2 fix
(`mpc_standalone_control_1786443033.csv`), t=39.7–42.9s: `v_desired`
genuinely oscillates (12.62 → 9.79 → 12.94 → 8.91 m/s over ~3s) and `a_cmd`
faithfully tracks it (+3.0 to −3.4 m/s² swings) — but this is NOT the
adaptive-gain machinery misbehaving. Cross-referenced against
`speed_profile.csv` (the precomputed oracle) at the car's actual position
(`tracks/comp_test_map_3/speed_profile.csv`, idx 656–680): `v_target` rises
smoothly to a local peak of 13.18 m/s then dips back to 11.79 m/s, tracking
a real change in the raw path geometry — Menger curvature computed directly
from the CSV's own `(x, y)` points crosses through ~0 in SIGN (not just
magnitude) at idx≈662-664, confirmed by checking the signed cross product
of consecutive segments. This is a genuine S-curve/chicane: a left-hand
bend straightens briefly, then curves right for the next bend. The
`kappa_max_abs`-driven lookahead correctly speeds the car up through the
brief straight and slows it back down anticipating the next corner — the
oscillation is the plan working as intended on a real varying-radius
feature, not spurious jitter to be filtered out.

Checked `a_lat_max` (`4.0`, `sim/speed_profile.py`'s
`compute_speed_profile()` default, well under the car's measured ~6.45-7.5
m/s² ceiling per the sim-to-real investigation) as a possible source of
excess conservatism at this specific corner, but did not raise it: this
would be a track-wide, not corner-specific, change, and CLAUDE.md's own
sim-to-real section explicitly warns the measured ceiling is
speed-dependent and NOT to assume a tuning change closes that gap without
per-corner validation — raising `a_lat_max` broadly was flagged as a
follow-up to actually test, not applied here.

**Do not attempt to smooth this away via `R_rate_diag[1]`
(acceleration-rate-of-change cost) or similar** — that would make the MPC
slower to respond to a real, upcoming tightening corner, trading a
correctly-anticipated slowdown for a late, harder one. If the oscillation
"feels wrong" on a specific track, the right lever is the speed profile's
own generation parameters (`a_lat_max`, scan window) or the path geometry
itself (smoothing a genuinely spurious kink), never the live adaptive gains
— but confirm the geometry is actually spurious first, the way this
investigation did, rather than assuming it.

## Dynamic speed cap: closing the gap between the oracle profile and live tracking

**Added 2026-08-11.** `precomputed_speed_at()` (the oracle-profile lookup used
when a track is already mapped — `USE_PRECOMPUTED_SPEED_PROFILE=True` /
`map_path` set) is a static, position-indexed nearest-point lookup: it has no
notion of the car's *actual current speed* relative to how much runway is
left to brake for the upcoming corner.

Combined with the frozen-`e_v`-horizon characteristic in the section above
(the MPC has no internal mechanism to anticipate a corner and ease off
early — it only ever tracks whatever `desired_speed` scalar it's handed
*this* tick), a car that enters a fast straight even slightly ahead of the
oracle profile's own pace has no predictive braking margin: the target speed
only starts dropping once the car reaches the position the profile
associates with braking, by which point there may not be enough distance
left.

This showed up directly in live telemetry
(`fsae_logs/mpc_standalone_control_*.csv`): at a corner entry, `v_actual`
was measured at ~9 m/s against a `v_desired` already at ~4.6 m/s, with
steering saturated at the 25° stop for over a second while speed caught up
to the target — i.e. the corner was recognised too late to brake for
smoothly.

`control_utils.dynamic_speed_cap()` (a thin wrapper over the already-existing
`curvature_speed()` — see that function's own docstring for the full
mechanism: it scans ~24 m of live path ahead, converts curvature to a
lateral-accel-limited corner speed, then propagates a braking-distance
constraint back from each corner) is now layered *underneath* the oracle
lookup rather than replacing it: every tick, when a track is mapped, the
controller takes `min(precomputed_speed_at(...), dynamic_speed_cap(...))`.
The oracle profile remains the trusted primary target (it encodes the whole
lap's raceline optimisation); the dynamic cap only ever pulls the target
*down* from that, catching the case where live tracking has drifted from the
plan (e.g. exiting the previous corner faster than expected).

Deliberately **not** a call to `curvature_speed()` with its own default
`a_lat_max=4.0`/`safety=1.0` (the values already tied to numeric parity with
the offline `use_planner=True` branch — see the parity table above): the
dynamic cap uses separate, tighter defaults
(`DYNAMIC_CAP_A_LAT_MAX=3.2`, `DYNAMIC_CAP_SAFETY=0.9`) so it engages a
little before the oracle profile would actually be violated, rather than
exactly at the edge — since it is a safety net under an already-tuned
profile, not a second opinion on the racing line.

Tunable on/off (`enable_dynamic_speed_cap` ROS param /
`ENABLE_DYNAMIC_SPEED_CAP` in `settings.py`, both default `True`) so it can
be A/B'd against the oracle-profile-only baseline without code changes —
`ros2/launch_all.sh`'s MPC tuning shortlist has a commented-out
`ENABLE_DYNAMIC_SPEED_CAP=false` line for a one-off disable. With the flag
off, behaviour is byte-identical to before this change. Has no effect when
no track is mapped (the live-`curvature_speed()`-only branch already runs
predictive lookahead speed control with no separate oracle target to cap).

Downstream of this cap, `tracking_error_speed_gate()` and
`SPEED_TARGET_RISE_RATE` apply exactly as before — the dynamic cap only
changes what `v_curv` feeds into that existing pipeline, reusing it rather
than adding a second gate/rate-limiter.

**Measured offline, mixed result — not yet validated on the live car.**
`python -m tuner.recorded_map_rollout --planner` (the `USE_PLANNER=True`
branch this cap actually runs in; the default oracle-only
`recorded_map_rollout` invocation never reaches this code path at all, since
it has no live centreline to scan) on `comp_test_map_3`, cap default
(`a_lat_max=3.2`, `safety=0.9`) vs. `ENABLE_DYNAMIC_SPEED_CAP=False`:

| metric | cap off | cap on |
|---|---|---|
| steering sat % | 4.37 | **5.54** |
| `\|e_psi\|` mean / p90 (deg) | 7.04 / 15.94 | **8.01 / 16.82** |
| a_lat max | 10.06 | **10.61** |
| a_lat > ceiling % | 2.92 | **0.62** |
| score (lower better) | 0.503 | **0.520** |

The cap does what it's narrowly designed to do — `a_lat > ceiling %` drops
4.7×, confirming it's genuinely holding the car below the lateral-accel
ceiling more often. But steering saturation and heading error both got
**worse**, and the composite score regressed. This is the opposite of the
predicted effect on saturation and is not yet understood — a likely
candidate is interaction with `SPEED_TARGET_RISE_RATE`/the tracking-error
gate (braking earlier/harder for one corner may leave the car in a worse
heading position entering the next one), but this has not been diagnosed.
Kept enabled by default in code (`enable_dynamic_speed_cap`/
`ENABLE_DYNAMIC_SPEED_CAP` default `True`) because the underlying mechanism
(a real-time lookahead cap under a static oracle profile with no notion of
the car's actual current speed) is the one CLAUDE.md's investigation
identifies as structurally missing — but treat `DYNAMIC_CAP_A_LAT_MAX`/
`DYNAMIC_CAP_SAFETY` as unresolved tuning, not validated defaults.

**2026-08-11, tested live: disabled again.** Subjectively performed worse on
the live car, consistent with the offline score regression above (0.503 →
0.520). `ros2/launch_all.sh` now uncomments `ENABLE_DYNAMIC_SPEED_CAP=false`
so a plain `launch_all.sh` run has the cap off, overriding the code-level
`True` default — the mechanism and its launch-arg/YAML/settings.py plumbing
are left in place (not reverted) for whoever picks up the tuning next, but
do not assume it is on by default in this repo's actual driving
configuration; check `launch_all.sh`'s shortlist first. Do not re-enable for
a live run without first understanding why it made steering saturation and
heading error worse, not just the a_lat ceiling metric it was targeted at.

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

## Cone geometry: verified accurate to FSDS (2026-08-06)

Checked because a track-geometry mismatch would corrupt every planner
comparison. It is not a source of the sim-to-real gap.

| | recorded map (`comp test map 3`) | sim | FS rules |
|---|---|---|---|
| track width | **3.50 m** (zero variance) | `TRACK_HALF_WIDTH = 1.75` → **3.50 m** | ≥ 3.0 m |
| cone spacing (median) | 3.95 m blue / 4.07 m yellow | — | ≤ 5 m |
| spacing ≤ 5 m | **98% / 99%** | — | — |
| spacing max | 5.25 m | — | — |

Width matches *exactly*, and spacing sits within FS limits.

> **Measure spacing along the path, not down the array.** Cones are stored in
> **recording order** (`source: fsae_sim_perception.cone_recorder`), not sorted
> around the track, so consecutive entries are not spatially adjacent. Naively
> differencing the array reports gaps up to 43.7 m and 23–31% of spacings over
> 10 m — all artifacts. Project each cone onto its nearest centreline index and
> sort by that first.

## Precomputed corner segmentation — replaces live curvature heuristics with exact lookups (2026-08-12)

**Motivation.** Live "what corner am I in / how far through it am I" was
never known directly — it was re-approximated every tick from local
geometric proxies (`kappa`, `kappa_max_abs`) plus a small amount of
tick-to-tick state (`_dist_since_peak`, `_armed_for_next_peak`). This has a
confirmed, real bug: `_lookahead_exit_boost`'s decay tracker can't
distinguish "13 m past the apex, corner over" from "13 m past the apex,
still fighting a bad exit," because it only ever sees local curvature, never
the corner's actual identity or how that specific approach is going —
confirmed on a real log (`mpc_standalone_control_1786516297.csv`, lap 1's
hard corner): `m_Q_epsi_exit` was already back to `1.00` (fully decayed) by
`t=5.92` while `e_psi=-13.96°` and the worst of the heading correction was
still ahead (steering saturating repeatedly through `t=6.5-7.3`).

Since a fixed/precomputed path (`_static_path`, when `path_map_path` is
set) is a known array, corner identity/extent doesn't need to be inferred
at all — it can be computed ONCE, when the path loads, and looked up by
index every tick instead of re-derived.

**Scope decision**: this is LIVE-ONLY, unlike this file's usual two-sided
parity rule. Consistent with the rest of this session's investigation
(which was live-only throughout by explicit user instruction), the user
was asked directly whether this structural feature should also be
prototyped in `fsae_MPCTest` and chose live-only. **`fsae_MPCTest`/
`settings.py`/`model_utils.py`/`rollout_core.py` are NOT touched by this
change** — `CornerMap`/`_segment_corners` exist only in
`ros2/src/fsae_planning/control/fsae_control/fsae_control/mpc_core.py`.
This is a deliberate, one-off exception to this file's normal parity rule,
not an oversight — flag it if porting offline work later assumes symmetry
that doesn't exist here.

**Caveat — only applies to a static/precomputed path.** The live-planner
path (SLAM-built, growing tick to tick, no `path_map_path` set) keeps
today's live heuristics completely unchanged; precomputing corner metadata
only makes sense for a path that's already fully known.

### `CornerMap` (mpc_core.py)

```python
@dataclass
class CornerMap:
    corner_id:            np.ndarray  # (n,) int,   -1 = on a straight
    dist_into_corner:     np.ndarray  # (n,) float, m from this corner's start
    dist_to_apex:         np.ndarray  # (n,) float, m to this corner's peak |kappa|
    dist_since_apex:      np.ndarray  # (n,) float, m since this corner's peak
    corner_length:        np.ndarray  # (n,) float, total arc length of the corner
    peak_kappa_abs:       np.ndarray  # (n,) float, this corner's peak |kappa|
    total_heading_change: np.ndarray  # (n,) float, this corner's total |heading change|
```

One row per path waypoint, computed once by `_segment_corners(path)` — an
O(n) pass that thresholds `|kappa|` against
`MPCParams.adaptive_q_lookahead_peak_hysteresis` (0.01 1/m, reused rather
than introducing a second straight/corner cutoff) to split the path into
straight/corner runs, then finds each run's apex, arc length, and total
rotation. No wraparound (matches every other function in `mpc_core.py` —
none of them treat the path as a closed loop either); confirmed correct
for `comp_test_map_3`'s recorded lap (start/end are 5.77 m apart, not
closed).

**Offline-validated (2026-08-12)** against `tracks/comp_test_map_3/raceline.csv`
(1000 waypoints): found 9 corner runs (2 of them single-waypoint blips right
at the path's start/end, marginally over the straight threshold — expected,
not a bug). The 7 real corners' arc-length positions and mean `v_target`
line up exactly as expected physically (tighter corners get lower planned
speed: 8.2-10.9 m/s at the sharpest corners vs 15.8-16.7 m/s on the
straighter sections) — cross-checked against the speed profile independent
of the segmentation code itself.

### Migration table (what replaced what)

| Today (live heuristic) | Replacement | Why it's better |
|---|---|---|
| `_lookahead_curvature_profile`'s per-tick forward scan for `kappa_max_abs` | `_corner_map_lookahead()` — index lookup if already inside a corner run; a bounded forward walk over `corner_id` transitions (not per-step `_curvature()` calls) if approaching one from a straight | Removes the O(lookahead_dist / waypoint_spacing) `_curvature()` evaluation from every tick; exact instead of sampled. |
| `_update_lookahead_peak` / `_dist_since_peak` (local-peak rising-edge detector, stateful) | `corner_map.dist_since_apex[base_idx]` / `corner_map.peak_kappa_abs[base_idx]` — direct lookup, no rising-edge state | Eliminates an entire stateful mechanism (`_armed_for_next_peak`) that was only ever approximating "which corner, how far past its apex" — the corner map answers this exactly. |
| `_lookahead_exit_boost`'s fixed-distance decay (the confirmed bug above) | Same decay shape, but held (not allowed to advance toward 1.0) while `abs(e_psi)` remains above the new `MPCParams.adaptive_q_lookahead_epsi_hold_thresh_rad`, blended continuously via `clip(abs(e_psi)/thresh, 0, 1)` — only active when a corner map is present | Fixes the confirmed bug: the boost can no longer wear off before heading-error recovery is actually done. |
| U-turn accumulated heading change (scanned live over a speed-scaled window, can under-see a slow U-turn if the window is smaller than the turn) | `corner_map.total_heading_change` exists in the dataclass, but is NOT YET wired into `_uturn_severity`'s call site | Deferred — only the `kappa_max_abs`/heading-change lookup needed for the row above was wired this session. Revisit if a U-turn under-detection issue is reported. |
| `_corner_demand`/`_demand_frac` | UNCHANGED | Correctly live/speed-dependent — same corner, different speed, different demand. Not precomputable. |

`_adaptive_R_scaling`, `_adaptive_Q_scaling`, `_steer_rate_anti_hunt`'s
current-position terms, and `_error_state`'s single-point preview `kappa`
are unchanged — these are about the car's current tracking error/speed, not
corner geometry.

### New parameter: `use_precomputed_corner_map` (node-level launch arg, NOT an `MPCParams` field)

Unlike every other flag in the numeric-parity table above, this is a
launch-argument/node-parameter concern, the same tier as `path_map_path`/
`use_precomputed_path` — it does not belong in the `MPCParams` parity table.
Declared in `sim.launch.py`/`control.launch.py` (grouped with
`path_map_path`/`use_precomputed_path`) and forwarded straight through (no
`IfElseSubstitution`, since it's a pure behaviour switch with no string to
blank) into both `mpc_controller.py`/`mpc_controller_standalone.py`'s
`declare_parameters` block. Has no effect unless `use_precomputed_path` is
ALSO true. Default `false` on both the node parameter and every launch
layer — land off, prove live before flipping. `ros2/launch_all.sh` exposes
it as `USE_PRECOMPUTED_CORNER_MAP=false`, alongside
`USE_PRECOMPUTED_SPEED`/`USE_PRECOMPUTED_PATH`.

**Regression-checked (2026-08-12)**: with the flag off / `set_static_path()`
never called, `MPCController.compute()`'s output is bit-for-bit identical
run-to-run (verified via a 200-tick synthetic replay against
`comp_test_map_3`'s raceline) — the new code path is a true no-op until
opted into. With the corner map active, output measurably differs (max abs
diff 0.072 over the same replay), confirming the new path is actually taken
when enabled.

**Status: implemented, offline-validated, NOT YET LIVE-TESTED.** The
`adaptive_q_lookahead_epsi_hold_thresh_rad` fix defaults to `0.0`
(disabled — today's unmodified fixed-distance decay) even with the corner
map enabled, so it needs a second, explicit opt-in on top of
`use_precomputed_corner_map=True` before it changes behaviour. Do not
raise it above `0.0` without a live A/B, same discipline as every other
untested mechanism in this file.

## Precomputed shaped heading-lead profile (2026-08-12)

**Motivation.** `raceline.csv`'s `psi` column has always been purely
geometric (`atan2` of the path tangent) — `_error_state`'s `e_psi` is
measured against wherever the car's nearest path point happens to be
pointing right now, with no anticipation of an upcoming bend. User's idea:
precompute a SHAPED heading reference, per waypoint, ahead of time — one
that already leads the geometric tangent by however much yaw the car can
physically achieve between here and there at the ALREADY-PLANNED speed —
so the heading-error term itself provides early commitment, without
relying on any Q/R reweighting of an error that doesn't exist yet.

**Why this is structurally different from curvature-forcing (already
tried and disabled) or a cost-target shift (tried and rejected the same
day, see below):** both of those told the QP about a future deviation it
was free to satisfy however was cheapest inside the horizon — a wrong-
direction dip-then-correct could look cheaper than committing immediately,
and testing confirmed both produce exactly that. A precomputed lead
heading instead changes what's true AT `k=0`, before the QP ever runs:
`e_psi` becomes a real, already-existing error the instant `_error_state`
computes it. There is no "spend it later" freedom, because nothing is
scheduled for later — synthetic testing (single-step and full corner-ramp
scenarios, multiple speeds) confirmed direct, monotonic correction with no
wrong-direction transient, unlike both prior mechanisms.

**Also tested and rejected: a FIXED lookahead distance.** The first version
of this idea (evaluate `path_yaw` at `base_idx + fixed_distance` instead of
`base_idx`) hit a cliff: as little as 8m of lead saturates steering to full
lock IMMEDIATELY on a realistic corner-entry ramp, at every speed tested.
**Fix: scale the lead by achievable yaw rate at the station's own planned
speed, not a fixed distance** — see `build_shaped_heading_profile` below.
This naturally caps the lead at whatever the car can actually execute, and
decays it to ~0 once a corner's constant-curvature section begins (nothing
left to "pre-achieve" once the car is expected to already be mid-turn) —
verified on an isolated synthetic corner. See
`late_turn_in_investigation.md` Parts 7-9 for the full derivation and
synthetic-test evidence.

### Where it lives

- **Offline (`fsae_MPCTest/tuner/tools/raceline_optimizer.py`)**:
  `max_yaw_rate`, `build_shaped_heading_profile`, `check_slip` — a DERIVED
  third pass, run in `export()` AFTER `optimize_raceline()`'s path+speed
  have already converged. Does NOT feed back into the path or speed (see
  that file's own coupling-depth discussion in its new functions'
  docstrings) — a deliberate scope decision: the existing optimizer's
  constants (`ALAT_MARGIN`, `_smooth_step`'s taper, etc.) are each tied to
  a specific measured failure mode: folding a new objective into that loop
  risked invalidating that tuning. Confirmed via direct comparison: x, y,
  psi, v_target are BYTE-IDENTICAL before/after this change on
  `comp_test_map_3` — only a new `psi_target` column is added.
- **CSV format**: extended from `x,y,psi,v_target` (4 columns) to
  `x,y,psi,psi_target,v_target` (5). Backward-compatible:
  `control_utils._load_profile_csv` detects column count per row and sets
  `psi_target = psi` for any 4-column file (old exports, or
  `speed_profile.csv`, which never needs this column) — a genuine no-op.
- **Live (`mpc_core.py`)**: `MPCController.set_heading_profile(psi_target)`
  stores the array; `_error_state` substitutes it for `path_yaw` at
  `base_idx`, ONLY for `e_psi`'s reference — `e_y`'s projection keeps using
  the geometric tangent, unchanged (same "only e_psi changes" precedent as
  `ref_heading_rate_limit_enabled`). New loader
  `control_utils.load_path_heading_profile_csv` (kept separate from
  `load_path_profile_csv`, whose `(n,2)` return shape is unchanged so
  `CornerMap`/every other `_static_path` consumer is unaffected).
- **Toggle**: `use_precomputed_heading_profile`, node-level launch
  parameter (NOT an `MPCParams` field — same tier as `use_precomputed_
  corner_map`/`path_map_path`), default `false` on both nodes and both
  launch files. `ros2/launch_all.sh` exposes
  `USE_PRECOMPUTED_HEADING_PROFILE=false`.

### Slip check (diagnostic only, unvalidated limit)

`check_slip` flags stations where the path's own geometric turning rate
implies more rear-axle slip (`beta_r = lr*r/v`, the standard steady-state
linear-bicycle relationship, same `Cf`/`Cr`-model family
`alat_ceiling_at()` already trusts) than a placeholder `SLIP_LIMIT_RAD`
(5°, **unmeasured, do not treat as authoritative**). On `comp_test_map_3`:
110/1000 stations (11%) exceed it, all at the track's sharpest, slowest
corners (peak kappa 0.12-0.21, v 5.5-7.9 m/s) — not scattered, so the
formula is behaving sensibly; whether 5° is the right bound is still
unknown. Diagnostic print only, does not fail the export or reshape
anything (unlike `_assert_clearance`'s hard `RuntimeError`) — a real
slip-limit measurement is needed before this becomes load-bearing.

### Honest caveat found during offline validation

`comp_test_map_3` has almost no genuinely straight sections — the
geometric `psi` climbs 0.4°→16° over the first 40m of what looks like a
"straight" in the corner-segmentation sense (part of the gentle S-curve,
Part 3e). Consequence: at `HEADING_LEAD_AUTHORITY_FRAC=0.5`, the shaped
lead sits near its max (~3.5-3.75°) across almost the WHOLE lap — only 2 of
1000 exported points ever reach near-zero lead. This is the algorithm
correctly responding to this track's actual geometry, not a bug (Part 8's
isolated single-corner test showed proper decay-to-zero on an actual
straight-then-corner shape) — but it means a live test on this track
exercises something closer to a constant heading offset than a
corner-triggered anticipation signal. `HEADING_LEAD_AUTHORITY_FRAC` (in
`raceline_optimizer.py`) is the first thing to lower if live behaviour
looks like "the car cuts every bend a bit early everywhere" rather than
"the car commits earlier specifically approaching sharp corners."

### Status

Implemented (offline: `raceline_optimizer.py`'s three new functions +
export wiring; live: `mpc_core.py`/both nodes/both launch files/
`launch_all.sh`, all live-only per this session's standing pattern — see
"Never push to fsae_planning repo" in project memory). Regression-checked:
`use_precomputed_heading_profile` unset reproduces `compute()`'s output
bit-for-bit; set, it measurably changes it (max abs diff 0.266 over a
200-tick synthetic replay). `comp_test_map_3/raceline.csv` re-exported
with the new column (x/y/psi/v_target confirmed byte-identical to the
pre-existing file).

**LIVE-TESTED 2026-08-12 at `HEADING_LEAD_AUTHORITY_FRAC=0.5` (the shipped
default) — first read: WORSE, not better.** Two completed laps vs. three
same-day baseline laps: mean `|e_psi|` rose (9.5°/10.5° vs. baseline's
6.1-8.2°), peak `|e_psi|` on the corner this investigation has tracked
throughout rose from 19.4° to 27.0-31.4°, steering saturation on that
corner roughly doubled to tripled. Likely cause floated at the time:
`comp_test_map_3` has almost no true straights (Part 10's own caveat), so
the lead is active nearly everywhere rather than gated to a corner's
approach phase — through much of a real corner's interior the car carries
extra lead ON TOP of the geometric heading, which is already changing
correctly there, rather than helping it commit early to a bend it hasn't
reached yet.

**Corrected after two more runs: high run-to-run variance, not a clean
regression.** The two additional profile-on runs scored 0.79/0.93
(composite score), both better than every baseline run recorded so far,
and one hit the single best result of the session on the tracked corner
(peak `|e_psi|` 11.1°, zero saturation) — spanning the same range from
best-of-session to worst-of-session as the original two runs. The
same-day baseline itself varied 19.4-22.7° run to run, nearly as wide a
spread. Four profile-on runs and three baseline runs is too small a sample
to call this a regression OR a win. **Currently left ON**
(`USE_PRECOMPUTED_HEADING_PROFILE=true` in `ros2/launch_all.sh`) pending
more data, superseding the "reverted to OFF" action taken after the first
two runs. See `late_turn_in_investigation.md` Parts 12-13 for the full
run-by-run data and candidate next steps (lower `authority_frac`
substantially, or gate the lead to the approach phase only rather than
letting it propagate into a corner's own interior — still not ruled out
as the explanation for the variance, just not yet confirmed either).

## Nonlinear MPC (`use_nmpc`) — a SECOND controller (added 2026-08-13; offline port added 2026-08-13)

**Update (2026-08-13): this now has an offline counterpart.**
`controller/nmpc_optimiser.py`'s `NMPCController` is an independent PORT of
the live `nmpc_core.py` module (same model, same SQP/OSQP scheme — not an
import; the two repos still cannot import each other), wired into
`sim/rollout_core.py`'s `run_core_rollout()` behind `settings.USE_NMPC`
(default false) the same way every other controller-mode flag works.
Structural/solver constants (`NMPC_HORIZON`, `NMPC_SQP_ITERS`, ...) live in
`settings.py`'s "Nonlinear MPC (NMPC)" section, kept numerically identical
to `nmpc_params.NMPCParams` by hand. The cost-weight overrides moved
live-side from `nmpc_params.py` into `mpc_params.MPCParams` itself (see
that file's own section) — `settings.py`'s matching `NMPC_Q_E_Y` etc.
constants mirror that move. See `docs/tuning.md`'s NMPC section for the
tuning surface, and reproduce the validation with
`python -m tuner.nmpc_offline_check` (no ROS/FSDS session needed).

### What it is

`ros2/src/fsae_planning/control/fsae_control/fsae_control/nmpc_core.py`'s
`NMPCController`: a Frenet-frame **nonlinear** MPC (Gauss-Newton SQP, condensed
dense QP subproblem solved directly by OSQP, real-time-iteration style — one
iteration per tick warm-started from the previous tick). It is selected by a
single node parameter, `use_nmpc` (default **false**), and replaces
`MPCController` wholesale when true. `mpc_core.py` is byte-unchanged.

### Why it exists — the one thing every mechanism above works around

`MPCController._discrete_model` is the bicycle model in error coordinates with
the reference frame's own rotation dropped. The missing term is exactly:

```
e_psi_dot = r - kappa(s) * s_dot          <-- absent from Ad/Bd
```

so with `e_y = e_psi = 0` the QP's whole rollout predicts staying at zero
forever and no weighting can produce turn-in before real error exists. Measured
directly: `MPCController` commands **0.000 deg** at 8 separate dead-on-line
states approaching a known bend. The "Curvature-forcing term" section above,
and the shaped heading-lead section, are both attempts to inject that term as
exogenous horizon-indexed data; both produce a wrong-direction transient,
because a future obligation known at solve time lets the solver choose when to
pay it (`late_turn_in_investigation.md` Parts 2/7/15).

In the Frenet formulation `kappa` is not horizon-indexed — it is `kappa(s)`
with `s` a **state** driven by the car's own predicted motion, so the
obligation is not schedulable. Verified: the wrong-direction excursion is a
single-step -0.33 deg against a +4.9 deg correct-direction peak (and CasADi +
IPOPT reproduces -0.327 deg, so it is the true optimum, not a solver artifact),
versus ~7 consecutive steps at comparable-to-peak magnitude for the mechanisms
above.

### Model, and what it reuses

States `[s, e_y, e_psi, v_x, v_y, r, delta_act, a_act]`, inputs
`[delta_cmd, a_cmd]`. **Every** vehicle constant comes from
`MPCController.__init__` unchanged (`lf` 0.70, `lr` 0.85, `m` 255, `Iz` 150,
`Cf`/`Cr`, `tau_delta` 0.08, `tau_a` 0.02, `MAX_STEER_RAD`, `MAX_ACCEL`,
`MAX_BRAKE`, `du_max` = 180 deg/s x dt), including the same kinematic->dynamic
blend band (1.0-2.5 m/s). Cost weights come from the same `MPCParams` instance
the LTV-QP uses. **No new physical constant was introduced.**

Three deliberate differences, all of which matter when reading a log:

1. **`q_r` weights heading-error RATE** (`r - kappa*s_dot`), not absolute yaw
   rate. Penalising absolute `r` in a curvature-aware model penalises the yaw
   rate the car MUST hold to follow a corner. Same number, different regressor
   — expect to re-sweep it live.
2. **`e_y`/`e_psi` are measured against the SMOOTHED reference**, not the raw
   segment tangent. The raw tangent steps by `ds/R` (5.7 deg per 0.5 m waypoint
   at R=5 m); the NMPC reads each step as real state error, which produced a
   period-2 +-25 deg steering limit cycle until the reference heading was
   derived from the same smoothed samples as `kappa(s)`. Bounded by the 1.5 m
   smoothing window (itself `control_utils.curvature_speed()`'s existing
   `dense_step=0.5`/`w=3` precedent, not a new constant).
3. **FSDS's measured `a_lat` ceiling is inside the prediction**, as a smooth
   `tanh` saturation of the predicted tyre forces, reusing the same numeric law
   (flat/slope/intercept = 7.5/0.47/2.46) that `mpc_core._alat_ceiling_at` and
   `model/vehicle_physics.alat_ceiling_at` also use — hardcoded independently
   as class-level defaults on `nmpc_core.py`'s own `_Plant`, not read from
   `MPCParams` (which no longer carries these fields after the 2026-08-13
   corner_factor rewrite removed them). Without it the linear-tyre model
   believes it can hold any corner at any speed and the car **spins**; with it
   the same run completes the lap. `nmpc_alat_ceiling_enabled=false` recovers
   the unconstrained plant for real-vehicle work, mirroring
   `VehicleParams.alat_ceiling_enabled`.

### What is inactive when `use_nmpc=true`

- The **entire adaptive gain schedule** (every mechanism documented in the
  sections above: lookahead approach/exit boosts, yaw-rate relax, anti-hunt,
  straight boosts, centred softening, U-turn detector). They exist to
  synthesise anticipation this model does structurally. No `m_*` telemetry
  columns are written.
- **`use_precomputed_heading_profile`** — no effect (one startup log line says
  so). It approximates the curvature the NMPC models exactly.
- `curvature_forcing_enabled`, `ref_heading_rate_limit_enabled` — LTV-QP-only.

(`use_precomputed_corner_map` no longer exists at all — the corner_factor
rewrite deleted the corner-map/adaptive-lookahead mechanism it fed, on both
`use_nmpc` settings — see `architecture.md`'s "Precomputed corner
segmentation" note.)

`use_precomputed_path` / `use_precomputed_speed` / `enable_dynamic_speed_cap`
/ delay compensation (`delay_compensation_enabled`, `pose_age_lp_alpha`,
`n_delay_hysteresis`, `max_delay_compensation_steps`) all work exactly as
today; the delay rollforward uses the nonlinear model instead of
`predict_ahead()`'s linearisation.

### Measured, offline (identical weights, identical plant, single-variable A/B)

Closed loop, this repo's 25-state Pacejka plant (`step_nonlinear_plant`,
`alat_ceiling` on) along `comp_test_map_3/raceline.csv`:

| | \|e_y\| mean / p90 / max | \|e_psi\| mean / p90 | steer sat | lap | solve mean / p95 / max |
|---|---|---|---|---|---|
| LTV-QP (as shipped) | 0.400 / 1.451 / 2.323 m | 5.92 / 15.41 deg | 12.5% | 43.1 s | 7.4 / 9.9 / 40.5 ms |
| NMPC (defaults) | 0.277 / 0.686 / 1.150 m | 5.84 / 14.50 deg | **0.8%** | 42.0 s | 8.9 / 11.6 / 14.7 ms |

Turn-in (arc length where |steer| first reaches 25% of that corner's peak,
relative to the corner start): **earlier on 7/7 corners, median 25.6 m**, and
negative (before the corner starts) on 7/7 versus positive on 6/7 for the
LTV-QP. Full tables, the horizon/iteration sweep behind the `N=20`/1-iteration
defaults, the CasADi+IPOPT cross-check, and the four bugs this testing found
are in `late_turn_in_investigation.md` Part 16.

**Live-tested and performing well, consistent with the offline A/B above.**
Matched same-day live pair on `comp_test_map_3` (`mpc_standalone`, same
weights: `q_e_y=6.35, q_e_yd=0.5, q_e_psi=1.65, q_r=1.0, q_e_v=5.40,
r_delta=1.8, r_a_accel=2.25, r_a_brake=0.5, r_rate=[2.5, 2.25]`), logged in
`fsae_logs/Linear mpc/mpc_standalone_control_1786568958.csv` (`use_nmpc=0`)
and `fsae_logs/NMPC/mpc_standalone_control_1786571019.csv` (`use_nmpc=1`):

| | LTV-QP (live) | NMPC (live) |
|---|---|---|
| lap time | 54.72 s | 52.35 s |
| composite score | 0.695 | 0.532 |
| RMSE (lateral) | 0.455 m | 0.378 m |
| peak lateral error | 1.636 m | 1.179 m |
| \|e_psi\| mean / p90 / max | 7.85° / 15.53° / 28.16° | 5.06° / 11.71° / 21.22° |
| steering saturation | 6.45% | 0.58% |
| steering reversals | 299 | 226 |

Same direction and similar magnitude as the offline A/B (steering
saturation drops sharply, tracking error improves across the board), on a
single matched pair — not yet the same n=multiple-runs rigor as the
offline sweep. Reproduce the offline numbers above with
`python3 ros2/src/fsae_planning/control/fsae_control/test/nmpc_offline_check.py`
(no ROS, no FSDS; the closed-loop section self-skips without an `fsae_MPCTest`
sibling checkout).

### Dependencies

`osqp` only — already a documented requirement of `mpc_core` via cvxpy
(`fsae_control/package.xml`). **CasADi/acados are NOT used by the shipped code**
(CasADi is not installable into the ROS interpreter on Ubuntu 24.04 without
`--break-system-packages`); the SQP and its Jacobians are numpy, and CasADi was
used once, from a private `--target` install, purely to cross-check the optimum.

### Which settings affect which controller — the full map

Every `MPCParams`/`NMPCParams` field now carries an explicit
`metadata["controller"]` tag (`"both"`, `"ltv_qp_only"`, or `"nmpc_only"`) —
see `mpc_params.py`/`nmpc_params.py` themselves for the authoritative
per-field value; the tables below are a reader-facing summary of the same
classification, verified against actual usage in `mpc_core.py`/`nmpc_core.py`
(not inferred from field names). `settings.py` mirrors the same three tags as
a `[LTV-QP only]`/`[NMPC only]`/`[shared]` prefix on each constant's comment,
and `ros2/launch_all.sh` tags its two shortlists the same way. See "What is
inactive when `use_nmpc=true`" above for *why* the LTV-QP-only mechanisms
don't apply under NMPC — this section only maps *which* fields fall in each
bucket.

**`MPCParams` base weights — shared, both controllers read them
(`mpc_params.py:47-67`):**

| Field | Live line | NMPC read site | Notes |
|---|---|---|---|
| `q_e_y` | `mpc_params.py:47` | `nmpc_core.py:864` | identical meaning |
| `q_e_yd` | `mpc_params.py:48` | `nmpc_core.py:865` | identical meaning |
| `q_e_psi` | `mpc_params.py:49` | `nmpc_core.py:866` | identical meaning |
| `q_r` | `mpc_params.py:50` | `nmpc_core.py:867` | **meaning differs**: LTV-QP weights absolute yaw rate `r`; NMPC (via `nmpc_q_epsi_dot`'s inherited base) weights heading-error RATE `r - kappa*s_dot`. Same slot, different regressor — see field's own docstring |
| `q_e_v` | `mpc_params.py:51` | `nmpc_core.py:868` | identical meaning |
| `r_delta` | `mpc_params.py:53` | `nmpc_core.py:870` | identical meaning |
| `r_a_accel` | `mpc_params.py:60` | `nmpc_core.py:871` | identical meaning |
| `r_a_brake` | `mpc_params.py:61` | `nmpc_core.py:872` | identical meaning |
| `r_rate_delta` | `mpc_params.py:63` | `nmpc_core.py:874` | identical meaning |
| `r_rate_a` | `mpc_params.py:64` | `nmpc_core.py:875` | identical meaning |
| `terminal_q_scale` | `mpc_params.py:67` | `nmpc_core.py:877` | identical meaning |

All ten are read by `nmpc_core.py`'s `__init__` through the same `_pick(override,
inherited)` helper (`nmpc_core.py:859-860`) that resolves each `nmpc_q_*`/
`nmpc_r_*` override below — a plain launch with every override left at its
`-1.0` sentinel means the NMPC starts from the LTV-QP's own tuned set exactly.

**`MPCParams` delay-compensation / n_delay fields — shared, both controllers
read them (`mpc_params.py:73, 80, 84-85`):**

| Field | Live line | NMPC read site |
|---|---|---|
| `delay_compensation_enabled` | `mpc_params.py:73` | `nmpc_core.py:1433` |
| `max_delay_compensation_steps` | `mpc_params.py:80` | `nmpc_core.py:1370,1505-1506` |
| `pose_age_lp_alpha` | `mpc_params.py:84` | `nmpc_core.py:1376` |
| `n_delay_hysteresis` | `mpc_params.py:85` | `nmpc_core.py:1378` |

The mechanism itself differs (the NMPC rolls `x0` forward through the
nonlinear model instead of `predict_ahead()`'s linearisation — `nmpc_core.py`
comment at line 1430), but all four fields gate/shape it identically on both
sides. `predict_epsi_clip` (`mpc_params.py:81`) is the one exception in this
group: it is a small-angle bound specific to `predict_ahead()`'s LINEAR
rollforward (`mpc_core.py:1182-1184`) and has no NMPC read site — LTV-QP only.

**`MPCParams` adaptive-gain-schedule fields — LTV-QP only
(`mpc_params.py:70-72, 74, 77, 81, 88, 91, 101-161`):**

None of these have any read site in `nmpc_core.py` (verified by grep, not
just absence from its docstring) — the entire corner-factor scheduler and
everything built on it is inert under `use_nmpc=true`, per "What is inactive"
above.

| Field | Live line |
|---|---|
| `adaptive_q_scaling_enabled` | `mpc_params.py:70` |
| `steer_rate_anti_hunt_enabled` | `mpc_params.py:71` |
| `adaptive_r_rate_enable_in_corners` | `mpc_params.py:72` |
| `ref_heading_rate_limit_enabled` | `mpc_params.py:74` |
| `ref_heading_rise_rate_deg_s` | `mpc_params.py:77` |
| `predict_epsi_clip` | `mpc_params.py:81` |
| `adaptive_r_rate_during_floor` | `mpc_params.py:88` |
| `anti_hunt_boost_max` | `mpc_params.py:91` |
| `corner_factor_k` | `mpc_params.py:101` |
| `q_ey_straight` / `q_ey_corner` | `mpc_params.py:108-109` |
| `q_epsi_straight` / `q_epsi_corner` | `mpc_params.py:113-114` |
| `q_r_straight` / `q_r_corner` | `mpc_params.py:120-121` |
| `rrate_steer_straight` / `rrate_steer_corner` | `mpc_params.py:127-128` |
| `r_steer_corner_mid` | `mpc_params.py:141` |
| `low_speed_corner_boost_v_half` / `_max_extra` | `mpc_params.py:150-151` |
| `epsi_ra_half_rad` / `_accel_boost_max` / `_brake_floor` | `mpc_params.py:159-161` |

**`MPCParams` NMPC weight overrides — NMPC only, by construction
(`mpc_params.py:189-205`):** every `nmpc_q_*`/`nmpc_r_*`/`nmpc_terminal_scale`
field exists solely to be read by `nmpc_core.py`'s `_pick()` calls
(`nmpc_core.py:864-877`); `mpc_core.py` never references any of them.

| Field | Live line | Overrides |
|---|---|---|
| `nmpc_q_e_y` | `mpc_params.py:189` | `q_e_y` |
| `nmpc_q_e_yd` | `mpc_params.py:190` | `q_e_yd` |
| `nmpc_q_e_psi` | `mpc_params.py:191` | `q_e_psi` |
| `nmpc_q_epsi_dot` | `mpc_params.py:192` | `q_r` (different regressor — see above) |
| `nmpc_q_e_v` | `mpc_params.py:199` | `q_e_v` |
| `nmpc_r_delta` | `mpc_params.py:200` | `r_delta` |
| `nmpc_r_a_accel` | `mpc_params.py:201` | `r_a_accel` |
| `nmpc_r_a_brake` | `mpc_params.py:202` | `r_a_brake` |
| `nmpc_r_rate_delta` | `mpc_params.py:203` | `r_rate_delta` |
| `nmpc_r_rate_a` | `mpc_params.py:204` | `r_rate_a` |
| `nmpc_terminal_scale` | `mpc_params.py:205` | `terminal_q_scale` |

**`NMPCParams` — all 20 fields NMPC only, by the file's own design
(`nmpc_params.py`):** the module docstring states this file holds only
structural/solver fields with no LTV-QP analogue (`nmpc_params.py:9-19`);
`mpc_core.py` never imports or reads `NMPCParams` at all. This includes the
master switch `use_nmpc` itself (`nmpc_params.py:62-66`), horizon/solver
settings (`nmpc_horizon`, `nmpc_sqp_iters`, `nmpc_solve_budget_ms`,
`nmpc_rk_substeps`, `nmpc_jac_substeps` — `nmpc_params.py:78-114`), SQP step
control (`nmpc_trust_delta_rad`, `nmpc_trust_a`, `nmpc_backtrack_max` —
`nmpc_params.py:117-132`), the soft track constraint
(`nmpc_track_halfwidth`, `nmpc_slack_weight` — `nmpc_params.py:135-145`),
curvature-reference construction (`nmpc_curvature_dense_step`,
`nmpc_curvature_smooth_w`, `nmpc_kappa_clip` — `nmpc_params.py:153-169`),
`nmpc_alat_ceiling_enabled` (`nmpc_params.py:171-179`), the three
MPCC-inspired flags documented in the subsection immediately below
(`nmpc_spline_reference_enabled`, `nmpc_horizon_speed_profile_enabled`,
`nmpc_friction_circle_enabled` — `nmpc_params.py:182-227`), and solver
tolerances (`nmpc_osqp_max_iter`, `nmpc_osqp_eps` — `nmpc_params.py:230-246`).

**`settings.py`-only flags with no dataclass field (offline side only,
LTV-QP-adjacent but not part of `MPCParams`/`NMPCParams`'s own field list):**
`USE_PRECOMPUTED_SPEED_PROFILE`, `ENABLE_DYNAMIC_SPEED_CAP` and its
`DYNAMIC_CAP_*` shape constants, `DELAY_STEPS`/`DELAY_JITTER_*`,
`ALAT_CEILING_FLAT`/`_SLOPE`/`_INTERCEPT` (a plant-physics constant consumed
by both controllers' in-prediction ceiling models, not a `MPCParams` field)
are out of this table's scope — they configure the ROLLOUT/PLANT, not
`MPCController`/`NMPCController` directly. `settings.py`'s own
`[LTV-QP only]`/`[NMPC only]`/`[shared]` comment tags cover only the
constants that DO have a dataclass-field counterpart, consistent with this
table.

### Three MPCC-inspired additions to the NMPC (added 2026-08-13)

Prompted by a comparison against Alexander Liniger's Model Predictive
Contouring Control (MPCC — the "C" is Contouring; see
`https://github.com/alexliniger/MPCC`). MPCC's headline idea — treating
progress along the track `θ` as a free decision variable the solver
*maximises*, with a contouring-error/lag-error split against a parametric
spline — was assessed and **not adopted**: `θ̇`-maximisation is a more
aggressive version of exactly the "exogenous, schedulable future obligation"
failure mode this NMPC's own `kappa(s)`-as-state design exists to avoid (see
"Why it exists" above), and adopting it would need the same falsification
testing (Part 16 §16.6's dead-on-line synthetic corner approach) before being
trusted. Three narrower, self-contained ideas survived that filter. All three
are NMPC-only (`mpc_core.py`, the LTV-QP, is untouched by any of them) and
implemented identically in both `nmpc_core.py` (live) and
`controller/nmpc_optimiser.py` (offline port) — see each file's own
docstrings for the exact mechanics.

**1. Spline-based path reference — `nmpc_spline_reference_enabled` /
`NMPC_SPLINE_REFERENCE_ENABLED`, default `true`.** `PathReference` now fits
`x(s)` and `y(s)` as two independent `scipy.interpolate.CubicSpline` objects
over cumulative arc length, and derives `kappa(s)` / `psi_ref(s)` analytically
from the spline's own first/second derivatives
(`kappa = (x'y'' - y'x'') / (x'^2+y'^2)^1.5`), instead of the old
dense-resample + moving-average + finite-difference-headings pipeline. This
is MPCC's *reference-parametrisation* mechanism (a continuous spline in arc
length) adopted on its own, decoupled from the contouring/progress apparatus
built on top of it in the original paper. Unlike the two flags below, this
defaults **on**: it is a strict numerical-quality improvement with no new
coupling to solver dynamics, and it directly targets the open, unfixed
"centreline curvature spikes" defect (CLAUDE.md) — a proper spline fit was
one of that defect's two previously-*named-but-unattempted* remedies. The old
moving-average path is kept intact (not deleted) behind the flag, so it can
still be A/B'd if a regression turns up. `kappa_at`/`kappa_scalar`/
`psi_ref_at`/`project` needed no changes — only how `self.s_kappa`/
`self.kappa`/`self.s_psi`/`self.psi_ref` get populated changed.

**2. Horizon speed profile — `nmpc_horizon_speed_profile_enabled` /
`NMPC_HORIZON_SPEED_PROFILE_ENABLED`, default `false`, EXPERIMENTAL.** Today
`v_ref` (the cost's speed target, `H[:,4] = v_x - v_ref`) is a single scalar
held constant across the whole horizon — deliberately, per the module
docstring, so a live A/B of feature 1's lateral-model change alone wouldn't
be confounded by a simultaneous longitudinal change. This flag is that
deferred change: a new `PathReference.v_ref_at(s)` samples a precomputed
per-lap speed profile at each horizon stage's own **predicted** arc length
`s_k`, the same way `kappa_at(s)` is already looked up against the predicted
state rather than scheduled by horizon step. That choice is deliberate and
important: it is what lets this feature inherit `kappa(s)`'s
non-schedulability property (see "Why it exists" above) instead of
reproducing the three earlier curvature-scheduling failures in a new,
longitudinal form. It targets the still-open Part 11 braking-lag problem.
Only takes effect when a speed-profile array is actually supplied at
`PathReference` construction time; **fully wired** in
`sim/rollout_core.py`'s NMPC construction/`compute_step()` call (offline) and
`mpc_controller_standalone.py`'s `set_static_path()` call (live mirror);
**deliberately not wired** in `mpc_controller.py` (the LTV-QP-parity node) —
with no array supplied, or with the flag off, `v_ref` is the exact same
frozen scalar as before.

**LIVE-TESTED 2026-08-13 and REJECTED.** Enabled alone (spline reference
also on, friction circle off) on `comp_test_map_3`,
`mpc_standalone_control_1786585464.csv` shows the exact predicted failure
mode, worse than the curvature case it was modelled on: at t≈58–61s
approaching a corner, `v_actual` climbs from ~5.7 to **16.7 m/s** while
`v_desired` **drops** to 3.3–5 m/s and stays there for ~2s — `a_cmd` is
strongly positive (0.6 to 2.5+ m/s²) throughout, i.e. the controller is
actively accelerating, not merely failing to brake. `e_y` grows to **-3.6 m**
(car off-track; `nmpc_track_halfwidth` is 3.5 m) before the corner
geometrically opens back up and it recovers. Mechanism: unlike `kappa(s)`,
whose obligation only exists once the predicted trajectory's own `s` has
actually reached it, summing `v_x - v_ref(s_k)` across all 20 horizon stages
lets a high `v_ref` at a later stage (the straight after this corner) offset
the cost of a low `v_ref` at an earlier stage (the corner itself) *in the
same solve* — the QP's net gradient can favour accelerating now against a
target that's about to rise, even though the current target is low. This is
the same "solver pre-pays a future obligation" trap as the three
curvature-scheduling failures (Parts 2/7/15), but the failure surfaces as
real off-track excursions rather than a self-correcting steering wobble,
because a wrong-direction speed decision carries real kinetic energy into
the corner for the LATERAL controller to then fight. **Reverted to
`false`.** Do not re-enable without addressing the underlying mechanism
(e.g. bounding how far ahead along `s` the sampled `v_ref` may rise, or
some other per-stage clamp preventing a later high-speed stage from
outvoting an earlier low-speed one) and revalidating offline first — the
non-schedulability property this feature was designed to inherit from
`kappa(s)` does not actually transfer to a *summed* cost over the horizon;
`kappa(s)`'s safety came from the state coupling, not from being
state-indexed per se, and speed's cost structure (a plain sum of squared
errors across all stages) breaks that coupling in a way curvature's
structure did not.

**3. Friction-circle hard constraint — `nmpc_friction_circle_enabled` /
`NMPC_FRICTION_CIRCLE_ENABLED`, default `false`, EXPERIMENTAL.** Adds a hard
`|F_yf|, |F_yr| <= F_max` bound to the condensed QP, **additional to, not a
replacement for**, the existing soft `tanh` lateral-force saturation already
inside `_f`/`_f_scalar` (difference #3 under "Model, and what it reuses"
above) — that soft mechanism is untouched, per CLAUDE.md's standing caution
against re-litigating it without new measurement evidence. `F_max` is derived
from the same measured ceiling law (`alat_ceiling_flat/_slope/_intercept`)
via `F_max = m * ceiling(v_x) / 2` per axle. Mechanically: `_outputs()` grows
two extra, cost-UNWEIGHTED rows (tyre forces, computed post-soft-saturation)
that ride through the existing `_output_jacobians` finite-differencing for
free, so `dF/dU_flat` reuses the condensing step's own `S = dx/dU_flat`
rather than needing a second rollout; `_build_qp`/`_solve_step` add `2*N` new
hard rows only when the flag is on, changing the QP's fixed sparsity pattern
(read once at construction, like the existing soft-slack rows). When off,
`_build_qp`/`_outputs`/`_output_jacobians`/`_solve_step` are IDENTICAL —
same array shapes, same QP dimensions — to before this feature existed, not
merely "the extra rows are empty." Telemetry exposes
`nmpc_fyf_max_abs`/`nmpc_fyr_max_abs` only when enabled (but see the
telemetry gap noted below — these never actually reached a CSV column).
Loosely inspired by MPCC's friction-ellipse tyre constraints, adapted to
bound the same tyre-force quantity this NMPC's plant already computes rather
than introducing a new force model.

**LIVE-TESTED 2026-08-13 and REJECTED — much more severely broken than
feature 2.** Enabled alone (spline reference also on, horizon speed profile
off) on `comp_test_map_3`, `mpc_standalone_control_1786585910.csv`: the SQP
subproblem failed to solve (`nmpc_status=0`) on **77.5% of all 614 ticks**,
steering sat at the mechanical hard lock (±25°) on **30.8% of ticks**
starting as early as t=0.65s, and the run ended in a full stall — car
stopped (`v_actual≈0`), 4.94 m off-track, heading error -52°, every column
frozen tick-for-tick for the final ~1 s of the log. This was enabled with NO
prior offline A/B (unlike features 1/2), against the explicit caution given
before testing it live; the result confirms that caution was warranted.
**Reverted to `false`** in `launch_all.sh` immediately.

Root cause: unlike the soft `tanh` saturation it sits alongside (which only
engages once *beyond* the ceiling, and is a smooth penalty the solver can
trade off against), the new hard `|F_yf|,|F_yr| <= F_max` rows have no slack
variable — there is nothing for the QP to give up if ordinary cornering
geometry and the force bound conflict simultaneously, which they evidently
do under completely normal driving on this track, not just extreme
conditions. When that happens the subproblem goes infeasible, `_solve_step`
correctly refuses to act on the resulting garbage direction (per its own
documented safety logic), and the practical effect is the controller
stops updating its steering command tick after tick — exactly the "pretty
much doesn't work anymore" symptom observed, compounding into a spin/stall
as the geometry degrades further with no correction being applied. This
means `F_max = m * ceiling(v_x) / 2` per axle is measurably **tighter than
what normal cornering actually needs** on this plant/track, not merely a
conservative-but-workable bound.

**Bug found alongside this (separate from the rejection above, worth fixing
regardless of whether this feature is ever revisited):** `nmpc_fyf_max_abs`/
`nmpc_fyr_max_abs` never actually appeared in the CSV — `telemetry_logger.py`'s
`NMPC_COLUMNS` is a hand-maintained tuple, unlike `build_config_lines()`'s
`dataclasses.asdict()`-based config dump, and nobody added the two new field
names to it when the feature was implemented. This silently dropped exactly
the diagnostic that would have shown, tick-by-tick, how close to (or over)
`F_max` the solve was running — the post-mortem above had to rely on
`nmpc_status`/steering/stall behaviour alone because of this gap. Fix
`NMPC_COLUMNS` before ever re-attempting this feature.

None of the three has offline A/B numbers yet (feature 1 is a numerical
improvement to an existing mechanism and is default-on; features 2 and 3 are
default-off, both now live-tested-and-rejected in their current form) —
reproduce a comparison with `python -m tuner.nmpc_offline_check` once one
exists. Do not re-enable 2 or 3 without first fixing the identified
mechanism, adding an offline A/B, and (for feature 3) the missing telemetry
columns above.

### Fixed: NMPC steers hard-right at a standstill on every run (2026-08-13)

**Symptom:** every NMPC run from a standing start — independent of any of
the three features above, reproduces with only feature 1 (spline reference,
default-on) active — commanded a hard, transient right-steer excursion in
the first ~0.5-0.7s: steering reaches the full ±25° mechanical lock by
t≈0.59-0.65s, `nmpc_pred_ey_end` (the horizon's own predicted terminal
lateral error) swings to roughly -3.3 m, while `v_actual` is still
essentially zero. The car recovers once genuinely moving and does not
repeat the excursion on later laps through the same map location.

**Not the track/spawn geometry.** The car spawns at `(0,0)` facing `+x`;
the track's first recorded waypoint sits at `x≈1.76, y≈-0.18` with
`psi≈0.006-0.009` rad — a small, ordinary offset (`e_y≈0.16` m at t=0,
`nmpc_s0≈-1.06` m, i.e. the Frenet projection lands just behind the path's
recorded start). Ruled out as the cause by a same-day, same-spawn A/B: both
the LTV-QP (`mpc_core.py`, `Linear mpc/mpc_standalone_control_1786594804.csv`)
and Stanley (`stanley_control_1786594710.csv`) see the identical `e_y≈0.16-0.17`
at t=0 and show no equivalent hard-lock snap — Stanley's initial command is
-8.9°, converging smoothly; the LTV-QP's is a few degrees, also smooth. Since
both react to the same starting error without incident, the cause has to be
specific to the NMPC's own model, not the track/spawn setup — track padding
was considered and rejected as a fix for this reason (the same v_x=0 startup
condition would recur at any new start point).

**Root cause: `_f`/`_f_scalar`'s tyre-slip-angle formula manufactures a
lateral force from steering ALONE at `v_x≈0`.** `alpha_f = arctan((v_y +
lf*r)/v_safe) - d`, where `v_safe = max(|v_x|, v_blend_hi)` floors the
denominator to avoid a divide-by-zero as `v_x -> 0`. That floor is necessary,
but its side effect is that at `v_x=0` with `v_y, r` small, `alpha_f ≈ -d` —
the slip angle tracks the commanded steering angle directly, producing a
substantial `F_yf` (and `F_yr`) purely from steering, with zero forward
speed. A real tyre generates ~zero lateral force with no rolling contact
velocity, regardless of steering angle — this was backwards. The
`blend` factor already computed for the kinematic/dynamic mix (0 at
`v_x <= v_blend_lo`, 1 at `v_x >= v_blend_hi`) already excludes
`v_y_dot_dyn`/`r_dot_dyn` from the state derivative at low speed, but that
happens too late: the force itself already existed and was available to the
SQP's cost/Jacobians before that exclusion, and (for
`nmpc_friction_circle_enabled`, when it's ever re-enabled) would have been
visible to the friction-circle constraint too.

**Fix:** scale `F_yf`/`F_yr` by the same `blend` factor immediately after
they're computed (and after the existing `alat_ceiling` soft saturation),
in all three copies — `nmpc_core.py` (live and mirror) and
`controller/nmpc_optimiser.py`'s `_f`, `_f_scalar`, and its
`nmpc_friction_circle_enabled`-only `_tyre_forces()` helper (which computes
the same force independently for telemetry/constraint rows and must stay a
line-by-line mirror of `_f`'s own computation per its own docstring). At
`v_x=0` with any commanded steering, every lateral-dynamics derivative
(`e_y_dot`, `v_y_dot`, `r_dot`) is now confirmed exactly zero (previously
`v_y_dot`/`r_dot` were nonzero purely from `d`). Verified: `_f`/`_f_scalar`
parity holds at 1e-13/1e-14 (both live/mirror and offline, well under the
1e-12 bar `test_scalar_matches_vectorised` requires), and
`python -m tuner.nmpc_offline_check`'s full suite (model parity, SQP
convergence, turn-in/wrong-direction, closed-loop) passes with no regression
in the closed-loop `|e_y|`/`|e_psi|`/saturation numbers.

**LIVE-TESTED 2026-08-13 and CONFIRMED FIXED.** First live attempt after
copying the fix to the live checkout (`mpc_standalone_control_1786595389.csv`)
still showed the hard-lock snap — traced to the ROS 2 workspace not having
been rebuilt, **not** a flaw in the fix: this project's `--symlink-install`
setup symlinks `ros2/build/fsae_control/fsae_control` back to
`src/fsae_planning/control/fsae_control/fsae_control`, but the running
Python process had an older module already loaded/cached from before the
edit, so the source-level edit was invisible until the workspace was
rebuilt and the nodes restarted. This is exactly the class of issue
`ros2/launch_all.sh`'s own commented-out `--symlink-install` rebuild step
exists to catch (see that file: *"an edit to `src/` after the last build is
silently invisible to `ros2 launch` until rebuilt"* — previously bit twice
in one session per that comment, now a third time). After rebuilding,
`mpc_standalone_control_1786595530.csv` shows the fix working exactly as
predicted: steering never exceeds ~18° in the first two seconds (previously
pegged at the full ±25° lock for most of a second), and the full run (1107
ticks, a complete lap) posted the best numbers of the whole session —
`|e_y|` mean 0.195 m, max 0.968 m, steering saturation 0.09%.
