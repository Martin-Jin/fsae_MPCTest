# Changes pending PR into `fsae_planning`

Source: uncommitted working-tree changes in the live `fsae_planning` checkout
(`ros2/src/fsae_planning/CHANGES.md`), as of 2026-08-10. This is a summary for
tracking purposes — the live repo is out of scope for edits/commits from here;
see `fsae_planning/CHANGES.md` itself for full detail.

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
- Steering-rate limit now expressed as deg/s×dt — survives a change of `dt`.
- `N` 25→35, `MAX_BRAKE` 9→7, retuned `Cf`/`Cr`/weights — kept numerically
  identical to `fsae_MPCTest`/mirror per the parity rule.
- Disabled-by-default adaptive-Q-scaling + ref-heading rate limiting,
  `terminal_scale` — parity scaffolding for future re-evaluation.

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

## Telemetry

- New diagnostic CSV columns (delay/latency/solver-failure) + prepended score
  header.
- `LapProgressTracker`: fixes every live run's composite score being
  permanently pinned at the DNF floor (`13.0`) by deriving real
  `progress`/`reached_end`/`time_bonus` from the car's position against a
  precomputed track path. Adds `lap_time_s`/`optimal_time_s` to the header.

## Discarded, not part of this PR

`steering_sysid.py`/`steering_step.py` — standalone diagnostics, not MPC
runtime dependencies.
