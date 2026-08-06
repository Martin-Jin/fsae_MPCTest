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
| `perception/fsae_sim_perception/` (whole package) | `perception/fsae_sim_perception/` | Direct mirror. `sim_perception.py` (FSDS oracle+odom → `/fsae/*`) + `cone_recorder.py`. |
| `planning/fsae_planning/` (whole package) | `planning/fsae_planning/` | Direct mirror. `centerline_planner.py` (the actual ROS 2 node — see the row above for why the root `planning/` folder doesn't have this), `boundary.py`, `cone_map.py`, `cone_sorting.py`, `path_utils.py`, `special_utils/` (skidpad). |
| `control/fsae_control/fsae_control/mpc_core.py` | `control/fsae_control/fsae_control/mpc_core.py` | Direct mirror. The `MPCController` class — QP-based MPC, kept in byte-for-byte parity with `sim/rollout_core.py`/`controller/optimiser.py` per this repo's own numeric-parity rule (see `CLAUDE.md`). |
| `control/fsae_control/fsae_control/control_utils.py` | `control/fsae_control/fsae_control/control_utils.py` | Direct mirror — includes both `curvature_speed()` and `StanleyController`. |
| `control/fsae_control/fsae_control/stanley_controller.py` | `control/fsae_control/fsae_control/stanley_controller.py` | Direct mirror. The actual current `controller:=stanley` node (publishes `cmd_vel`, routes through `fsds_bridge`). Replaces the old frozen reference implementation — see "Last resynced" above. |
| `control/fsae_control/fsae_control/mpc_controller.py` | `control/fsae_control/fsae_control/mpc_controller.py` | Direct mirror. Upstream's default `controller:=mpc` node — steering only through `cmd_vel`/`fsds_bridge`, discards the MPC's own throttle/brake. See "Two MPC-controller nodes" below. |
| `control/fsae_control/fsae_control/mpc_controller_standalone.py` | `control/fsae_control/fsae_control/mpc_controller_standalone.py` | Direct mirror. See "What replaced `control_node.py`" for the history. |
| `control/fsae_control/fsae_control/fsds_bridge.py` | `control/fsae_control/fsae_control/fsds_bridge.py` | Direct mirror. Needed by both `stanley_controller.py` and `mpc_controller.py` (not `mpc_controller_standalone.py`, which bypasses it). |
| `control/fsae_control/fsae_control/telemetry_logger.py` | `control/fsae_control/fsae_control/telemetry_logger.py` | Direct mirror. CSV telemetry shared by all three controller nodes. Also computes the run's composite score (via `scoring.py`) and prepends it to the control CSV as a `#`-commented header on `close()`. |
| `control/fsae_control/fsae_control/scoring.py` | `control/fsae_control/fsae_control/scoring.py` | Direct mirror **and** a verbatim copy of this repo's own `sim/scoring.py` — see "Live/offline score parity" below. Changes must be made in `sim/scoring.py` first, then re-copied. |
| `control/fsae_control/setup.py` | `control/fsae_control/setup.py` | Direct mirror. Registers all four console-script entry points (`controller`, `mpc_controller`, `mpc_controller_standalone`, `fsds_bridge`). |

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
  known figure-8. Ported here for parity (see "Last resynced" above) but
  **this repo has no skidpad mode**, so nothing calls it yet. Keep it if you
  resync again rather than stripping it as dead code — it's parity, not
  scope creep — but don't expect to exercise it until/unless a skidpad
  characterisation mode is added here too.
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
| `curvature_speed()`'s `a_lat_max` | `sim/speed_profile.py:328` (function default) | `fsds_simulator/control/fsae_control/fsae_control/control_utils.py:37` (function default) | `4.0` |
| Planner top/bottom speed clamp | `sim/rollout_core.py:55-56` (`PLANNER_V_MAX`, `PLANNER_V_MIN`) | `fsds_simulator/control/fsae_control/fsae_control/mpc_controller_standalone.py` (declared as the `v_max`/`v_min` ROS parameters, default `20.0`/`1.5`) | `20.0` / `1.5` |
| Steering slew-rate limit (`du_max[0]`) | `model/vehicle_physics.py` (`VehicleParams.max_steer_rate`), applied as `max_steer_rate * DT` in `sim/rollout_core.py` and passed to `controller/optimiser.py`'s `du_max` | `fsds_simulator/control/fsae_control/fsae_control/mpc_core.py` (`MAX_STEER_RATE_RAD_S`, applied as `* self.dt`) | `radians(180.0)` rad/s |
| Accel slew-rate limit (`du_max[1]`) | `sim/rollout_core.py` (`du_max` second element) | `fsds_simulator/control/fsae_control/fsae_control/mpc_core.py` (`self.du_max` second element) | `0.6` per step |
| `tracking_error_speed_gate()` thresholds | `sim/speed_profile.py` | `fsds_simulator/.../control_utils.py` | `ey_lo/hi` 0.5/2.0 m, `epsi_lo/hi` 20/60 deg, `floor` 0.3 |
| Speed-target rise limit | `sim/rollout_core.py` (`SPEED_TARGET_RISE_RATE`) | `fsds_simulator/.../mpc_controller.py` and `mpc_controller_standalone.py` (same name) | `2.0` m/s² |
| `curvature_speed()` κ reduction | `sim/speed_profile.py` | `fsds_simulator/.../control_utils.py` | max of 3-point running mean |
| Score weights / bonuses / penalties | `settings.py` (`SCORE_WEIGHTS`, `COMPLETION_BONUS_WEIGHT`, `TIME_BONUS_WEIGHT`, `DNF_PENALTY`, `DNF_OFFTRACK_PENALTY`) | `fsds_simulator/control/fsae_control/fsae_control/scoring.py` (inlined as module constants) | weights sum to `1.0`; `0.5` / `0.25` / `3.0` / `3.0` |
| Metric normalisation scales | `settings.py` (`METRIC_SCALES`) | `fsds_simulator/control/fsae_control/fsae_control/scoring.py` (inlined as module constant) | 12 entries, `[0.40, 0.45, 0.30, 0.18, 1.50, 0.40, 0.02, 0.30, 1.00, 0.015, 0.70, 2.30]` |
| Constrained-scoring constants | `settings.py` (`CONSTRAINT_FLOOR`, `COMPLETION_THRESHOLD`, `TIME_OBJECTIVE_WEIGHT`, `QUALITY_WEIGHT`) | `fsds_simulator/.../scoring.py` (inlined as module constants) | `10.0` / `0.98` / `1.0` / `0.35` |
| `A_BRAKE_PLAN` (braking-distance propagation in `curvature_speed`) | `sim/speed_profile.py` | `fsds_simulator/.../control_utils.py` | `5.0` m/s², positive magnitude |
| Latency telemetry columns | — (offline has no equivalent) | `fsds_simulator/.../telemetry_logger.py` | `pose_age_s`, `path_age_s`, `n_delay`, `solve_ms`, `cmd_latency_ms` |
| Pose-feed hold model | `settings.py` (`POSE_HOLD_*`) + `sim/rollout_core.PoseFeedHold` | — (offline-only; models a live fault) | `PROB 0.05`, `MEAN_TICKS 2.1`, `MAX_TICKS 5` |

Notes on how these are actually used:

- `sim/speed_profile.py`'s `curvature_speed()` (the offline mirror of
  `control_utils.py`'s `curvature_speed()`) is called by `sim/rollout_core.py`'s
  `use_planner=True` branch at `sim/rollout_core.py:232-234`, which passes
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
   (max 15.0) to 2.8 m/s (max 4.65) when `|e_y| > 1.5 m`.
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

## Simulator fidelity limits (what FSDS does NOT model)

Read this before trusting any offline or FSDS result as a prediction of
real-car behaviour. These are the known ways both simulators are *easier* than
reality.

| Aspect | FSDS / this rollout | Real car | Modelled? |
|---|---|---|---|
| **Localisation accuracy** | Perfect. `sim_perception` copies ground-truth `/fsds/testing_only/odom` verbatim onto `/fsae/slam/car_position`. No noise, no drift, no estimation lag. | ZED visual odometry + `cone_mapper` SLAM: jitters, drifts, lags. | Offline only, via `SLAM_NOISE_ENABLED` (**default off** — FSDS has no such error, so defaulting it on would make offline scores pessimistic against the very runs they're compared to). |
| **Cone map** | Latched *oracle* map of exact cone positions, cropped to a forward window + radius. Only **range** is limited. | Real detections: false positives/negatives, position error, colour confusion, range-dependent noise. | **No.** Not modelled anywhere. |
| **Pose rate** | 20 Hz (`pose_rate`), matching the controller. Was 10 Hz — see the section below. | Bounded by the perception pipeline's real throughput. | Live-only concern; the offline rollout always uses a fresh pose per step. |
| **Actuation delay** | Fixed `DELAY_STEPS`, compensated exactly by `predict_ahead()`. | Variable, estimated from a timestamp, never exactly known. | Partly — `DELAY_JITTER_STEPS` perturbs the controller's *belief* about the lag. |
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

## Delay realism: why the tuner under-reproduces live chatter

The offline rollout applies a fixed `DELAY_STEPS` lag and `predict_ahead()`
compensates for it **exactly** — the simulated controller knows its own lag
perfectly. The live controller never does: it estimates the lag from a pose
timestamp divided by a jittering loop period. Measured live, that loop period
is median 0.0498 s but p99 0.0741 s and max 0.1205 s (jitter σ ≈ 0.0092 s ≈
0.18 steps), so the live step count is regularly wrong by one — and each wrong
value changes how far `x0` is rolled forward, feeding a step disturbance into
the QP at the control rate.

`settings.DELAY_JITTER_STEPS` (default `0.2`, matching the measured σ) closes
part of that gap: it perturbs **only the controller's belief** about how many
commands are in flight, leaving the plant's true lag at `DELAY_STEPS`. The
draw is seeded (`DELAY_JITTER_SEED`) so rollouts stay reproducible and CMA-ES
still gets a stable score per candidate. The error is deliberately two-sided —
over-estimating re-rolls the oldest pending command, mirroring what a
too-large `pose_age_s` does live.

**How much this actually recovers — measured, not assumed.** With
`USE_PLANNER=True` (the real configuration), raising the slew limit from
80 → 180 deg/s drops the fraction of steps pinned on the limit from 1.6–4.3%
to ~0.5% across `PATH_MICRO_SLALOM`/`PATH_S_BEND`/`PATH_SUDDEN_TURN`, so the
constraint is now visibly active in the tuner rather than inert. Delay jitter
on its own moves composite scores by <0.002.

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
exact — just stale. Staleness is not noise. `SLAM_NOISE_ENABLED` exists for the
real car's localisation error and defaults to off for exactly this reason.

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

Two inputs have no faithful live equivalent and default to `0.0`/`False`:

- `time_bonus` — needs a known expected lap time / step budget.
- `offtrack` — the offline rollout knows ground-truth track edges.

The emitted CSV header records this as `score_is_partial=1` so a reader can't
mistake a partial live score for a full offline one. The weighted-metric
component (the 12 metrics × `SCORE_WEIGHTS`) is directly comparable either
way; only the bonus/penalty terms differ.

## OPEN: the sim-to-real gap is not yet explained

Measured 2026-08-06 on the recorded `comp test map 3`, same tuned gains both
sides:

| | offline sim | live car |
|---|---|---|
| steering saturation | 3.4% | **21.1%** |
| \|e_psi\| mean / p90 | 6.0° / 13.8° | **15.9° / 42.0°** |
| max \|e_y\| | 1.82 m | 1.20 m |

The car sits at full steering lock six times more often than the simulator, and
when it does it is pulling only 4.14 m/s² lateral at 5.74 m/s — it is not
cornering hard, it is rotating back from a large heading error. Heading error
arrives in sustained episodes (median 0.47 s, up to 2.44 s, 96% of energy below
1 Hz), i.e. a stale/wrong reference rather than high-frequency chatter.

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

**Root cause is now localised** (2026-08-06): the car's yaw response to a
steering command is ~3× weaker than commanded, in a *speed-dependent* way the
offline plant does not reproduce at all (characteristic speed 6 m/s live vs
52 m/s offline). Grip is roughly right; the rotation is not. The mechanism
inside FSDS is still unidentified — see "MEASURED: the car's yaw response is
~3× weaker than commanded" below.

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

> **Not yet acted on.** No plant change is recommended from this measurement
> alone. Reproducing `K_us ≈ 0.04` by bending tyre parameters would match the
> symptom while keeping the physics wrong, and would corrupt every downstream
> grip-dependent result. Identify the mechanism first.

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
