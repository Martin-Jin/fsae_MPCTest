"""
Lightweight CSV telemetry logger for the control nodes (debug/tuning aid).

Both controllers (Stanley, MPC) can write two compact CSVs so planner problems
can be separated from controller problems offline. On close() the control CSV
is rewritten with a scored, commented header block (see "Score header" below).

Why CSV and not rosbag: the tracking errors e_y / e_psi are computed inside the
controller and never published to a topic, so rosbag can't see them, and they
are what distinguishes "path is wiggly" from "controller oscillates". CSV is
also far smaller (~100 KB/min) and directly plottable.

Coordinate frame: everything positional is in the simulator's global ENU frame
(x = east, y = north, yaw zero at +x/east), written with no conversion, so
control rows and path rows overlay directly on one plot. e_y/e_psi are
Frenet-style, measured at the front axle relative to the nearest planner path
segment:
  e_y   > 0  → front axle is to the LEFT of the path      (metres)
  e_psi > 0  → car heading is rotated CCW (left) of the path tangent

Time: `t` is run-relative seconds starting at 0.0 (first logged sample), not a
ROS epoch stamp; the epoch of that origin is preserved in the header as
`t0_epoch_s`. Both CSVs share one origin.

Control CSV columns: t, car_x, car_y, car_yaw, v_actual, v_desired, steer_deg,
e_y, e_psi_deg, yaw_rate, delta_cmd (rad), a_cmd (m/s^2), solver_failed,
inaccurate, plus the latency diagnostics pose_age_s/path_age_s/n_delay/
solve_ms/cmd_latency_ms. delta_cmd/a_cmd are logged in the MPC's own units
rather than normalised FSDS command units so the score can be recomputed from
the file without re-deriving the scaling.

Lap / horizon-accuracy columns (trailing the latency block): lap_idx (which
lap this tick belongs to, 0-indexed until the first completion),
pred_err_m/pred_acc_pct (this tick's own matured prediction-vs-actual
comparison from HorizonAccuracyTracker, empty on every tick for a
non-NMPC run), lap_score/lap_pred_acc_pct (filled ONLY on the row where a
lap completes -- see ControlLogger.finish_lap()). "Horizon accuracy" here
means how closely the NMPC's predicted path over the next ~1 s matched
where the car actually went; 100% is a perfect prediction, see
HorizonAccuracyTracker's docstring for the exact formula. This metric is
independent of composite_score: a run can drive well (low score) with a
model that predicts itself poorly, or vice versa.

Trailing the above is the adaptive-feature trace — curvature/demand context
plus one column per adaptive multiplier and the resulting absolute weights.
See ADAPTIVE_COLUMNS below for the full list and what each one means
(its tail carries the NMPC-only columns, empty on LTV-QP runs).

Path CSV columns: t, idx, x, y — waypoint snapshots at ~1 Hz.

Units contract: log_control()'s steer_rad/e_psi_rad are radians and converted
to degrees on write. Callers must not pass the normalised ControlCommand.steering
([-1, 1]) — scale it by MAX_STEER_RAD first, or steer_deg silently inflates by
~2.3x while still looking plausible.

Score header: close() prepends a `#`-commented block with the composite score
and every component metric, computed by fsae_control.scoring — a verbatim
copy of fsae_MPCTest/sim/scoring.py, so a live run is directly comparable to
an offline tuner rollout. `pandas.read_csv(path, comment='#')` parses the file
unchanged; numpy's genfromtxt needs `skip_header=<count of '#' lines>` since it
doesn't skip comments when locating the `names=True` row. The car can't
measure `time_bonus` or `offtrack`, so when the caller supplies neither those
terms are 0.0/False and the header records `score_is_partial`.

Config header: right below the score header (see `_write_score_header`),
`ControlLogger.set_config_lines()`/`build_config_lines()` write a
`#`-commented dump of the ENTIRE launch-time configuration this run used —
which controller (Stanley / LTV-QP MPC / NMPC), every `MPCParams`/
`NMPCParams` field (a plain `dataclasses.asdict()` dump, so it never goes
stale when a field is added/removed/retuned), the NMPC's own RESOLVED
weights (post `-1.0`-inherits-from-MPCParams), and the launch-time flags that
pick a path/speed source (`map_path`, `path_map_path`,
`use_precomputed_heading_profile`, `enable_dynamic_speed_cap`, `v_max`,
`v_min`). The goal is that a single CSV, on its own, is enough to reproduce
the exact run it came from — see `build_config_lines()`'s own docstring for
the one thing this does NOT capture (the adaptive-gain SCHEME itself, as
opposed to its numeric weights, since that's code, not a parameter).

MODULE MAP
----------
  columns.py          ADAPTIVE_COLUMNS
  config_lines.py     build_config_lines
  horizon_tracker.py  HorizonAccuracyTracker
  lap_progress.py     LapProgressTracker
  control_logger.py   ControlLogger
  scoring.py          same formulas as fsae_MPCTest/sim/scoring.py (constants inlined)
"""

from fsae_control.telemetry.columns import ADAPTIVE_COLUMNS  # noqa: F401
from fsae_control.telemetry.config_lines import build_config_lines  # noqa: F401
from fsae_control.telemetry.horizon_tracker import HorizonAccuracyTracker  # noqa: F401
from fsae_control.telemetry.lap_progress import LapProgressTracker  # noqa: F401
from fsae_control.telemetry.control_logger import ControlLogger  # noqa: F401
