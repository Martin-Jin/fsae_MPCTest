# Changes pending PR into `fsae_planning`

Source: uncommitted working-tree changes in the live `fsae_planning` checkout
(`ros2/src/fsae_planning/CHANGES.md`), as of 2026-08-13. This is a summary for
tracking purposes — the live repo is out of scope for edits/commits from here;
see `fsae_planning/CHANGES.md` itself for full detail. Local `main` is
up to date with `origin/main` (no upstream drift) — everything below is
working-tree-only.

## Perception

- Split pose/cone publishing into independent timers (20Hz/10Hz) — the 20Hz
  MPC was solving against a stale pose shared with the slower cone-map timer.
- New `/fsae/slam/car_odom` topic, atomic pose+twist snapshot — pose and
  speed/yaw-rate previously came from separately-timed reads with no
  guarantee they matched.
- `car_position` → `PoseStamped` (adds timestamp) — needed for delay
  compensation.
- `look_radius` 18m → 25m — keep ahead of the planner's own extended
  lookahead.

## Planning

- `_WALL_PLAN_HORIZON`/`look_radius` 15/18m → 25m — match new
  braking-distance scan range.
- Path-seed rejection loosened (reject only clearly-behind points) — old
  cutoff dropped the nearest midpoint mid-corner, deleting a real corner from
  the path.
- `ConeMap._absorb()` now dedupes within a batch — same-frame duplicate
  detections were becoming permanent duplicate cones.

## Control — speed planning

- `curvature_speed()` rewritten to propagate a real braking-distance limit
  per corner — old version could demand more deceleration than the car can
  produce.
- Curvature scan denoised (resample + moving average) — planner refit noise
  was causing speed oscillation on straights.
- New `tracking_error_speed_gate()` — slow down when badly off-path, since
  curvature alone can't detect that.
- New CSV-backed speed/path lookup (`map_path`/`path_map_path`) — bypass live
  re-derivation for already-mapped tracks.

## Control — MPC core

- Pose-age delay compensation (`predict_ahead()`, `_update_n_delay()`) — plan
  against predicted current state, not a stale measurement.
- `e_yd` now includes body-frame lateral velocity — old formula silently
  dropped sideslip contribution.
- `e_y` now signed perpendicular projection, not nearest-point distance — old
  version only correct when the nearest point happened to be abeam.
- Steering-rate limit 80°/s → 180°/s (measured achievable rate ~200°/s), now
  expressed as deg/s×dt so it survives a change of `dt`.
- Fixed logged steering angle being computed from the wrong input scale
  (~2.3x inflated) — had been masking the rate-limit issue in prior test logs.
- `N` 25→35, `MAX_BRAKE` 9→7, retuned `Cf`/`Cr`/weights — kept numerically
  identical to `fsae_MPCTest`/mirror per the parity rule.
- Disabled-by-default adaptive-Q-scaling + ref-heading rate limiting,
  `terminal_scale` — parity scaffolding for future re-evaluation.
- **Removed the entire forward-scanning lookahead gain-scheduling family**
  (~15 mechanisms: approach/exit boosts, demand normalisation, U-turn
  detector, straight-line adjustments, curvature forcing, and the
  precomputed `CornerMap` fast path) — reweighting today's cost based on a
  forward scan doesn't change what the QP's horizon predicts once the car
  gets there. Replaced by `_corner_factor`/`_low_speed_corner_boost`: one
  continuous CURRENT-curvature fraction blending four weights between a
  straight/corner endpoint, plus an always-on heading-error-driven
  accel/brake asymmetry. Mirrored same-day into `fsae_MPCTest`.
- **New second controller, `nmpc_core.py`/`nmpc_params.py`**: a Frenet-frame
  nonlinear MPC (`use_nmpc`, default off; `mpc_core.py` byte-unchanged when
  off). Closes the LTV-QP's structural gap — its linear prediction has no
  path-curvature term, so a car dead on-line approaching a corner is
  predicted to stay on-line forever (measured: exactly 0.000° commanded
  across 8 synthetic states). The NMPC tracks arc length as a state and
  looks up curvature directly. Offline A/B: steering saturation 12.5% →
  0.8%, turns in earlier on 7/7 corners tested (median 25.6 m earlier).
  **Live-tested, matched same-day pair**: steering saturation 6.45% →
  0.58%, lap 54.72s → 52.35s, composite score 0.695 → 0.532, |e_psi| mean
  7.85° → 5.06°. Mirrored into `fsae_MPCTest/fsds_simulator/` with its own
  offline port, `controller/nmpc_optimiser.py` (`settings.USE_NMPC`).

## Control — centralized MPC tuning

- New `mpc_params.py`: pulls every `mpc_core.py` weight/gain/flag (Q/R/R_rate
  weights, adaptive-gain shape constants, feature-enable flags) out of
  hardcoded constants into one `MPCParams` dataclass, matching
  `fsae_MPCTest/settings.py` field-for-field. Pure relocation, no behaviour
  change at defaults. Field count has moved since (44 as of the
  corner-factor rewrite, down from an original ~56 — that rewrite deleted
  more fields than the NMPC overrides added).
- Both controller nodes now declare every `MPCParams` field as a ROS2
  parameter and build the live `MPCParams` from those values, so weights are
  retunable at launch time.
- `control.launch.py`/`sim.launch.py` generate their `MPCParams` launch args
  mechanically from the dataclass instead of by hand; `fsae_params.yaml`
  gained matching YAML defaults.
- See `planning_control_sync.md`'s "MPC weight/gain parity: `MPCParams` ↔
  `settings.py`" table for the current field-by-field mapping against the
  offline `settings.py` constants this must stay numerically identical to.

## Control — nodes

- New `mpc_controller_standalone.py` — sends MPC throttle/brake directly, so
  offline longitudinal tuning actually reaches the car (bypasses
  `fsds_bridge`'s separate P-loop).
- Tracking-error gate + speed-rise-rate limiter added to both controller
  nodes.
- Both nodes now slice tracked path from car's nearest point before
  curvature — fixes inconsistent curvature measurement between the two
  nodes.

## New files

- `scoring.py` — verbatim copy of offline composite score, so live runs are
  graded identically to tuner rollouts.
- `cone_recorder.py`/`.launch.py` — records a completed lap's cone map for
  offline reuse.
- `tracks/comp_test_map_3/` (cone map + both exported CSVs) — committed
  track data, so a checkout of FSDS + `fsae_planning` alone can drive this
  track with no `fsae_MPCTest` checkout needed. `fsae_MPCTest` remains where
  *new* tracks get produced; its exporters write here when both repos are
  checked out side by side.
- `control/fsae_control/test/nmpc_offline_check.py` — reproduces the NMPC
  offline A/B (steering saturation, turn-in distance) with no ROS/FSDS
  needed. The one intentional exception to this repo's standalone rule: it
  optionally imports `fsae_MPCTest` for an extra closed-loop cross-check if
  that repo happens to be checked out alongside, and degrades to
  synthetic-state checks only if it isn't.

## Build/package plumbing

- `setup.py` (all four packages): new `console_scripts` entry points —
  `mpc_controller_standalone` (control), `cone_recorder` (perception).
  Without these, `ros2 run` can't launch either new node even though the
  source files are present.
- `setup.py` (all four packages): `zip_safe=True` → `False` — works around a
  stale-install issue in `colcon build --symlink-install` where an edited
  source file wasn't picked up until a full clean rebuild.

## Telemetry

- New diagnostic CSV columns (delay/latency/solver-failure) + prepended score
  header.
- `LapProgressTracker`: fixes every live run's composite score being
  permanently pinned at the DNF floor (`13.0`) by deriving real
  `progress`/`reached_end`/`time_bonus` from the car's position against a
  precomputed track path. Adds `lap_time_s`/`optimal_time_s` to the header.
  Only applies when driving against a precomputed `map_path`; runs against
  the live/on-the-fly planner topic still have no known track end, so those
  still score `score_is_partial=1`.
- **Fixed: `ADAPTIVE_COLUMNS` still declared the deleted lookahead family's
  columns** (silently empty on every run since the corner-factor rewrite)
  **and never declared the corner-factor rewrite's own telemetry** (silently
  dropped from every CSV instead of logged). Now lists exactly what
  `compute()` currently writes.

## Discarded, not part of this PR

`steering_sysid.py`/`steering_step.py` — standalone diagnostics, not MPC
runtime dependencies.
