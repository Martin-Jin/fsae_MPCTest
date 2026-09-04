# Offline/Live Parity

**Two different obligations share the word "parity". They are maintained
differently and fail differently.**

**1. The `fsds_simulator/` mirror — byte-identical copies.**
`fsds_simulator/` holds copies of the live `fsae_planning` ROS 2 workspace
files. They are maintained with `cp` and checked with `diff`:

```bash
diff -rq --exclude=__pycache__ \
  ../ros2/src/fsae_planning/control fsds_simulator/control
```

Nothing here needs judgement. A difference is either an unpropagated change or
an accident, and the fix is to copy the file.

**2. Offline↔live numeric parity — the same *number* in different code.**
This is not a mirror and cannot be diffed. `settings.py` and
`sim/speed_profile.py` are structurally different from the live
`mpc_params.py` and `control_utils.py`, and **the live node cannot import
`settings.py`** — there is no `settings.py` on the car. So a value like
`a_lat_max = 4.75` is typed independently into `sim/speed_profile.py:409` and
`control_utils.py:194`, and nothing mechanical keeps the two equal.

That is what the tables below are for. A silent divergence here does not break
a build; it makes an offline-tuned weight set invalid on the car while every
test still passes.

## Project rules this document is the authority for

Four standing rules govern edits to the planning/control stack. They are stated
here because several other documents refer to them; this section is the
canonical wording.

**1. Parity rule — the stack exists in two places and both must change.**
Planning/control logic lives in the live ROS 2 nodes
(`ros2/src/fsae_planning/control/fsae_control/`) *and* in this repo's offline
simulator (`sim/rollout_core.py`, `settings.py`, `controller/`). Weights tuned
offline are only valid on the car if the live code matches numerically.
A one-sided edit to either copy is an incomplete change. The
authoritative field-by-field mapping is the "Numeric-parity constants" and
"MPC weight/gain parity" tables below.

**2. Scoring parity — one formula, copied verbatim.** `sim/scoring.py` is the
single source of truth for the composite score;
`control/fsae_control/fsae_control/scoring.py` is a verbatim copy so a score
logged on the car is directly comparable to an offline one. Change the offline
file first, then re-copy. The one intentional difference is that the live copy
inlines the weight constants (there is no `settings.py` on the car), and those
must be kept numerically identical. See "Live/offline score parity" below.

**3. The offline simulator does not fully predict the car.** Same map and same
gains, the live car saturates its steering far more often than the offline
rollout and carries roughly twice the heading error. **An offline score alone
is not evidence.** Always validate on the car before accepting a tuning
result. The measured cause and how much of the gap is closed are in
`docs/logs/sim_to_real_investigation.md`.

**4. Do not imitate the simulator's lateral-acceleration ceiling with tyre
parameters.** FSDS enforces a sustained lateral-acceleration ceiling of about
7.5 m/s² that is *speed-dependent*, so it is not a grip limit. Reproducing it
by scaling `mu` or the cornering stiffnesses was tried and fails — it matches
one measurement while wrecking the plant's genuine grip and failing the
full-lock and closed-loop checks. The `alat_ceiling*` model in
`model/vehicle_physics.py` is the correct mechanism; tune that, not the tyres.
Details in "The sim-to-real gap" below and
`docs/logs/sim_to_real_investigation.md`.

**`fsds_simulator/` is a staging area, not a live module.** It mirrors
`fsae_planning`'s own ROS 2 workspace hierarchy exactly — every package
(`common/fsae_interfaces`, `common/fsae_bringup`, `perception/
fsae_sim_perception`, `planning/fsae_planning`, `control/fsae_control`), not
just the control-layer files. That means the whole tree can be copied
straight across into a workspace `src/` at the same relative paths, with no
manual re-pathing and no missing scaffolding (`package.xml`/`setup.py`/
`setup.cfg`/`resource/` included). See `fsds_simulator/README.md` for the
build/run instructions this enables.

Nothing under `fsds_simulator/` is imported by `gui/simulation.py`,
`tuner/offline_tuner.py`, or anything else in this repo — those all live
under `planning/`, `sim/`, `model/`, `controller/` instead. `fsds_simulator/`
exists purely so this repo can hold, version, and hand off a ready-to-build
copy of the ROS 2 side — including to someone who has only this repo and
FSDS, with no separate `fsae_planning` checkout at all.

**Current mirror scope.** `fsds_simulator/` covers the full workspace, not
just the control layer: `common/fsae_interfaces` (message package),
`common/fsae_bringup` (`fsae_params.yaml`, `perception.launch.py`,
`planning.launch.py`, `sim.launch.py`, full package scaffolding), all of
`planning/fsae_planning` as a real package (the root-level `planning/` folder
separately holds an algorithm-only mirror — see the file mapping below),
`perception/fsae_sim_perception`, and `control/fsae_control`'s full node set
(`fsds_bridge.py`, `stanley_controller.py`, `mpc/mpc_controller.py`,
`telemetry_logger.py`).

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
  `mpc/mpc_controller.py` fills that role via its `standalone_output=true`
  mode (see "What replaced `control_node.py`" below — a same-design,
  different-file swap, not a rename). MPC-related files
  (`mpc_core.py`/`mpc_params.py`/`nmpc_core.py`/`nmpc_params.py`/
  `mpc_controller.py`) live in a `control/fsae_control/fsae_control/mpc/`
  subpackage, not the package's top level.
- `steering_sysid.py`/`steering_step.py` and their harness scripts are **not**
  mirrored, and have no upstream counterpart to track: they never existed in
  `fsae_planning`'s committed git history, and upstream discarded them from
  its own working tree before raising a PR (see
  `fsae_MPCTest/docs/fsae_planning_pending_pr.md`, which tracks what is and is
  not part of the pending PR). `tuner/checks/steering_sysid_analysis.py` /
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
| `control/fsae_control/fsae_control/mpc/mpc_core.py` | `control/fsae_control/fsae_control/mpc/mpc_core.py` | Direct mirror. The `MPCController` class — QP-based MPC, kept in byte-for-byte parity with `sim/rollout_core.py`/`controller/optimiser.py` per the parity rule and numeric-parity tables in this document. |
| `control/fsae_control/fsae_control/mpc/mpc_params.py` | `control/fsae_control/fsae_control/mpc/mpc_params.py` | Direct mirror. `MPCParams` dataclass — every MPC weight/gain/flag, ~56 fields, the single source of truth for MPC tuning on the live side. |
| `control/fsae_control/fsae_control/mpc/nmpc_core.py` | `control/fsae_control/fsae_control/mpc/nmpc_core.py` | Direct mirror. `NMPCController` — the Frenet-frame nonlinear MPC (`use_nmpc=true`), kept in numeric parity with `controller/nmpc_optimiser.py`. |
| `control/fsae_control/fsae_control/mpc/nmpc_params.py` | `control/fsae_control/fsae_control/mpc/nmpc_params.py` | Direct mirror. `NMPCParams` dataclass — the NMPC's own tunables, separate from `MPCParams` (see that file's own note on why). |
| `control/fsae_control/fsae_control/control_utils.py` | `control/fsae_control/fsae_control/control_utils.py` | Direct mirror — includes both `curvature_speed()` and `StanleyController`. |
| `control/fsae_control/fsae_control/stanley_controller.py` | `control/fsae_control/fsae_control/stanley_controller.py` | Direct mirror. The actual current `controller:=stanley` node (publishes `cmd_vel`, routes through `fsds_bridge`). Replaces the old frozen reference implementation — see "Current mirror scope" above. |
| `control/fsae_control/fsae_control/mpc/mpc_controller.py` | `control/fsae_control/fsae_control/mpc/mpc_controller.py` | Direct mirror. The `controller:=mpc` node — its `standalone_output` ROS2 parameter (default `true`) selects between the two output modes a `false`/`true` value used to be two separate files for; see "The MPC controller's two output modes" below. |
| `control/fsae_control/fsae_control/fsds_bridge.py` | `control/fsae_control/fsae_control/fsds_bridge.py` | Direct mirror. Needed by `stanley_controller.py` always, and by `mpc_controller.py` only when `standalone_output=false` (skipped when `true`, which bypasses it). |
| `control/fsae_control/fsae_control/telemetry_logger.py` | `control/fsae_control/fsae_control/telemetry_logger.py` | Direct mirror. CSV telemetry shared by all controller nodes. Also computes the run's composite score (via `scoring.py`) and prepends it to the control CSV as a `#`-commented header on `close()`. Includes `LapProgressTracker`, which computes real `progress`/`reached_end`/`time_bonus` from the precomputed track path; see "Live/offline score parity" below. |
| `control/fsae_control/fsae_control/scoring.py` | *(no upstream counterpart — never existed in `fsae_planning`'s git history)* | **Not a direct mirror.** Staged here for upstreaming. It **is** a verbatim copy of this repo's own `sim/scoring.py` — see "Live/offline score parity" below. Changes must be made in `sim/scoring.py` first, then re-copied here (and eventually upstreamed). |
| `control/fsae_control/setup.py` | `control/fsae_control/setup.py` | Direct mirror **except** the `scoring.py`-related entry points/imports this repo's own staged-for-upstream file above needs — that exists here but not upstream. Registers three console-script entry points (`controller`, `mpc_controller`, `fsds_bridge`). |

> **`zip_safe=False` is required, on both sides.** All four of this mirror's
> `setup.py` files (`common/fsae_bringup`, `control/fsae_control`, `perception/
> fsae_sim_perception`, `planning/fsae_planning`) set `zip_safe=False` — the
> root-cause fix for a stale-`colcon-build` bug (§49 in
> `docs/logs/sim_to_real_investigation.md`). The four live copies set it too,
> verified byte-identical on this setting. Keep it on any new package added to
> either side.

`planning/` (root) and `mpc/mpc_core.py` are shared algorithm code and
should track upstream closely. `mpc/mpc_controller.py`'s
`standalone_output=true` code path traces back to this repo's own
integration pattern (see "What replaced `control_node.py`" below) but is now
a direct mirror like every other file in the table above — it's expected to
diverge from the `standalone_output=false` path in upstream-specific ways
(topic names, message types) while keeping the same behavioural design.

### The MPC controller's two output modes, plus Stanley

### What replaced `control_node.py`

This repo used to have its own `fsds_simulator/control_node.py`, targeting an
older ROS 2 topic/message interface (`/fsds/planned_path` as `nav_msgs/Path`,
`/FusionCones` as `fs_msgs/Track`) that no longer exists in current
`fsae_planning`. It was retired and its design ported into upstream's
package as a new node, `mpc_controller_standalone.py`, updated for the
current topics (`/fsae/planning/selected_trajectory`, `/fsae/slam/
car_position`, `/fsae/perception/cone_detection`).

That node has since been **merged into `mpc_controller.py`** as its
`standalone_output=true` mode (alongside `mpc_controller.py`'s own
pre-existing `standalone_output=false` mode) — one node file selected by a
boolean parameter, instead of two launchable executables selected by a
controller-name string. There is no separate `control_node.py` or
`mpc_controller_standalone.py` anymore, in either repo.

`mpc_controller.py`'s `standalone_output` parameter (default `true`) picks
between the two output modes that used to be two separate files:

- **`standalone_output=false`** — the original `mpc_controller.py` design.
  Publishes `ackermann_msgs/AckermannDriveStamped` (steering + target speed)
  on the shared `cmd_vel` interface and lets `fsds_bridge.py`'s simple
  speed-error P-loop compute throttle/brake, and own GO-gating/cone-braking,
  identically to the Stanley controller. It **discards** the MPC's own
  throttle/brake output.
- **`standalone_output=true`** (default) — the design this repo's old
  `control_node.py` was ported into. Publishes `fs_msgs/ControlCommand`
  directly, using `MPCController.compute()`'s `(steering, throttle, brake)`
  output unchanged (preserving the offline-tuned longitudinal behaviour this
  repo's tuner produces), and re-implements GO-hold/stale-path-brake/
  cone-proximity-brake itself instead of relying on `fsds_bridge.py`.
  Selected via `standalone_output:=true` (the default) in `control.launch.py`,
  which skips `fsds_bridge` for that mode.

When resyncing this repo's mirror against a newer `fsae_planning`, diff
`mpc_controller.py` against `mpc_controller.py` — both modes live in the same
file now, so there's only one file to resync. The two modes' code paths
inside it are deliberately different (see the file's own module docstring
for exactly which parts branch on `standalone_output` and which are shared),
reusing the same `MPCController` QP core; don't unify them further without a
specific reason to.

## Deliberately not mirrored

- **The old frozen Stanley reference** (previously at
  `fsds_simulator/stanley_controller/stanely_control_utils.py` +
  `stanley_control.py`) — **removed**. It targeted an old
  `/fsds/planned_path`+`Track` interface, was not kept in sync with upstream,
  and imported a `separate_cones_by_color` helper that isn't defined anywhere
  in either repo. The real, current `StanleyController` (in
  `control_utils.py`) and `stanley_controller.py` node are now mirrored
  instead, kept in sync like everything else under `fsds_simulator/`. An
  older local copy of this repo with the frozen reference still present
  reflects the *previous* state — don't resurrect it.
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
| Planner top/bottom speed clamp | `sim/rollout_core.py:67-68` (`PLANNER_V_MAX`, `PLANNER_V_MIN`) | `fsds_simulator/control/fsae_control/fsae_control/mpc/mpc_controller.py` (declared as the `v_max`/`v_min` ROS parameters, default `20.0`/`1.5`) | `20.0` / `1.5` |
| Steering slew-rate limit (`du_max[0]`) | `model/vehicle_physics.py` (`VehicleParams.max_steer_rate`), applied as `max_steer_rate * DT` in `sim/rollout_core.py` and passed to `controller/optimiser.py`'s `du_max` | `fsds_simulator/control/fsae_control/fsae_control/mpc/mpc_core.py` (`MAX_STEER_RATE_RAD_S`, applied as `* self.dt`) | `radians(180.0)` rad/s |
| Accel slew-rate limit (`du_max[1]`) | `sim/rollout_core.py` (`du_max` second element) | `fsds_simulator/control/fsae_control/fsae_control/mpc/mpc_core.py` (`self.du_max` second element) | `0.6` per step |
| `tracking_error_speed_gate()` thresholds | `sim/speed_profile.py` | `fsds_simulator/.../control_utils.py` | `ey_lo/hi` 0.5/2.0 m, `epsi_lo/hi` 20/60 deg, `floor` 0.3 |
| Speed-target rise limit | `sim/rollout_core.py` (`SPEED_TARGET_RISE_RATE`) | `fsds_simulator/.../mpc/mpc_controller.py` (same name, shared by both `standalone_output` modes) | `7.0` m/s² |
| `curvature_speed()` κ reduction | `sim/speed_profile.py` | `fsds_simulator/.../control_utils.py` | max of 3-point running mean |
| Score weights / bonuses / penalties | `settings.py` (`SCORE_WEIGHTS`, `COMPLETION_BONUS_WEIGHT`, `TIME_BONUS_WEIGHT`, `DNF_PENALTY`, `DNF_OFFTRACK_PENALTY`) | `fsds_simulator/control/fsae_control/fsae_control/scoring.py` (inlined as module constants) | weights sum to `1.0`; `0.5` / `0.25` / `3.0` / `3.0` |
| Metric normalisation scales | `settings.py` (`METRIC_SCALES`) | `fsds_simulator/control/fsae_control/fsae_control/scoring.py` (inlined as module constant) | 13 entries, `[0.40, 0.45, 0.30, 0.18, 1.50, 0.40, 0.02, 0.30, 1.00, 0.015, 0.70, 2.30, 0.08]` |
| Constrained-scoring constants | `settings.py` (`CONSTRAINT_FLOOR`, `COMPLETION_THRESHOLD`, `TIME_OBJECTIVE_WEIGHT`, `QUALITY_WEIGHT`) | `fsds_simulator/.../scoring.py` (inlined as module constants) | `10.0` / `0.98` / `1.0` / `0.35` |
| `A_BRAKE_PLAN` (braking-distance propagation in `curvature_speed`) | `sim/speed_profile.py` | `fsds_simulator/.../control_utils.py` | `5.0` m/s², positive magnitude |
| Dynamic speed cap enable/gains | `settings.py` (`ENABLE_DYNAMIC_SPEED_CAP`, `DYNAMIC_CAP_A_LAT_MAX`, `DYNAMIC_CAP_SAFETY`) | `mpc/mpc_controller.py` (`enable_dynamic_speed_cap`/`dynamic_cap_a_lat_max`/`dynamic_cap_safety` ROS params) | `True` / `3.2` m/s² / `0.9` — see "Dynamic speed cap" section below |
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
  same function the same way: `mpc_controller.py`'s
  `_control_step` passes `v_max=self._v_max, v_min=self._v_min` (also
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
centralized one place per side (see "Single
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
| `q_e_y` | `Q_diag[0]` | `6.35` both sides — synced |
| `q_e_yd` | `Q_diag[1]` | `0.5` both sides — synced |
| `q_e_psi` | `Q_diag[2]` | `1.65` both sides — synced |
| `q_r` | `Q_diag[3]` | `1.20` both sides — synced |
| `q_e_v` | `Q_diag[4]` | `5.5` both sides — synced |
| `r_delta` | `R_diag[0]` | `1.35` both sides — synced |
| `r_a_accel` | `R_diag`/`R_A_ACCEL` | `2.25` both sides — synced; see "Accel/brake effort weight split" below |
| `r_a_brake` | `R_A_BRAKE`/`R_diag[1]` | `0.5` both sides — synced |
| `r_rate_delta` | `R_rate_diag[0]` | `52.5` both sides — synced (raised from ~2.8/2.0, see the steering-chatter fix below) |
| `r_rate_a` | `R_rate_diag[1]` | `5.0` both sides — synced |
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

`LapProgressTracker` in `telemetry_logger.py` supplies them: it tracks the
car's forward-bounded nearest-index position against the precomputed track
path (the same CSV already loaded for the live speed lookup) to get real
`progress`/`reached_end`, and integrates `ds / v_target` over the
already-loaded speed profile for an `optimal_time` bound — **not** a call
into `speed_profile.optimal_lap_time()`, since that solver lives in
`fsae_MPCTest` and is not on the live node's `PYTHONPATH` (see the
settings-import caveat above).

`time_bonus = optimal_time * progress / actual_lap_time`, clipped to
`[0, 1]`, same scaling convention as `sim/rollout_core.py`. Both controller
nodes feed the tracker's output into `close()`, and the CSV header now also
records `lap_time_s`/`optimal_time_s`.

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

## Lap timing starts at 0.5 m/s, not at the first tick

**Plain version:** a run's clock used to start the moment the software began,
which included about a second of the car sitting still before it moved. Every
lap time was that much too slow, and the offline and live numbers were not
measuring the same thing. The clock now starts when the car actually starts
moving.

`LAUNCH_SPEED_MPS = 0.5` is the threshold, defined on both sides:

| side | location |
|---|---|
| live | `telemetry_logger.py`'s `LapProgressTracker.LAUNCH_SPEED_MPS` |
| offline | `sim/rollout_core.py`'s `LAUNCH_SPEED_MPS` + `launch_step` |

- **Live**: `LapProgressTracker.update()` takes `car_speed` and defers
  `_start_wall` until `abs(car_speed) >= LAUNCH_SPEED_MPS`. Passing
  `car_speed=None` falls back to the old first-tick behaviour, so an older
  caller still works.
- **Offline**: `sim_time = (n_ran - launch_step) * DT`, where `launch_step` is
  the first step above the threshold.

**Consequence for comparing numbers:** a lap time recorded before this change
includes the standstill and is roughly 0.95 s slower than the same drive
measured after it. The two are not directly comparable. This also cleared a
spurious DNF in `nmpc_offline_check`, where the standstill was consuming step
budget.

**Both output modes must pass `car_speed`** — `mpc_controller.py`'s
`_control_step` calls
`self._lap_tracker.update(self._car_pos, t, self._car_speed)` regardless of
`standalone_output`. A caller that omits it silently reverts to timing from
tick 0.

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
   mirrors `mpc_controller.py` (both `standalone_output` modes) and
   `stanley_controller.py` — both current nodes, not just one — see
   "The MPC controller's two output modes, plus Stanley" above.
4. If the change touches `planning/` or `mpc_core.py`, check per this document's
   numeric-parity rule whether `sim/rollout_core.py` needs a mirrored change —
   `rollout_core.run_core_rollout()` and `mpc_core.MPCController` are two
   implementations of the same control loop kept in deliberate numeric
   parity. Call this out explicitly in the resync notes if a mirrored change
   is or isn't needed.
5. Re-check the numeric-parity constants table above — if upstream changed
   `curvature_speed()`'s `a_lat_max` or the planner speed clamp values, both
   the offline (`sim/speed_profile.py`, `sim/rollout_core.py`) and live
   (`control_utils.py`, `mpc/mpc_controller.py`) copies need the same
   update.
6. Run the smoke-test pattern from `docs/developer_guide.md`'s testing section: confirm
   changed files import cleanly, then run `python -m gui.simulation` (or a
   short `python -m tuner.offline_tuner` run with `FAST_TEST_MODE = True` in
   `settings.py`) against one synthetic path and check the rollout still
   converges and tracks correctly. There is no way to test the
   `fsds_simulator/` mirror's ROS 2 files against the real/FSDS car from this
   repo directly — reason through the change against `sim/rollout_core.py`
   instead and flag it for live testing by a human once actually pasted into
   `fsae_planning`.
