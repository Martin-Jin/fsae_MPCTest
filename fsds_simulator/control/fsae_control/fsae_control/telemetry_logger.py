"""
Lightweight CSV telemetry logger for the control nodes (debug/tuning aid).

Both controllers (Stanley, MPC) can write two compact CSVs so we can separate
planner problems from controller problems offline.  On close() the control CSV
is rewritten with a scored, commented header block (see "SCORE HEADER" below).

Why CSV and not rosbag: the tracking errors e_y / e_psi are computed inside the
controller and never published to a topic, so rosbag can't see them — yet they
are exactly what distinguishes "path is wiggly" (small e_y, wiggly path) from
"controller oscillates" (steer + e_y swing on a smooth path).  CSV is also far
smaller (~100 KB/min) and directly plottable.


COORDINATE FRAME
================
Everything positional is in the simulator's **global ENU frame**: x = east,
y = north, z = up, x pointing forward at spawn, rotations right-handed, yaw
zero when facing +x/east (docs/coordinate-frames.md).  There is NO frame
conversion anywhere in this logger — car_x/car_y/car_yaw are written exactly
as the controller received them from the pose topic, and the path CSV's x/y
come from the same global-frame planner waypoints the MPC solves against.  So
control rows and path rows are directly overlayable on one plot with no
transform.  (The cone-proximity check in mpc_controller_standalone.py does work
in the car-local frame, but none of those values are logged here.)

The two error signals are Frenet-style, measured at the **front axle**
(car_pos advanced by lf along the heading), relative to the nearest planner
path segment — not relative to the world frame:
  e_y   > 0  → front axle is to the LEFT of the path      (metres)
  e_psi > 0  → car heading is rotated CCW (left) of the path tangent
Both signs follow from the ENU right-handed convention (+y is left of +x), and
are SHARED by both controllers so A/B plots overlay.


TIME
====
`t` is seconds since the logger was constructed — i.e. **run-relative time
starting at 0.0**, not a ROS epoch stamp.  Callers still pass their absolute
ROS clock reading; the logger subtracts the first sample it sees.  The raw
epoch of that t=0 origin is preserved in the header (`t0_epoch_s`) so a run can
still be lined up against a rosbag if needed.  Both CSVs share the same origin,
so control and path rows remain directly comparable.


COLUMNS — control CSV
=====================
  t             s        Run-relative time, 0.0 at first logged control step
  car_x         m        Global ENU east position of the car reference point
  car_y         m        Global ENU north position
  car_yaw       rad      Global ENU heading, right-handed, 0 = +x/east
  v_actual      m/s      Measured forward speed magnitude
  v_desired     m/s      Planner/curvature target speed (post-filtering)
  steer_deg     deg      Commanded ROADWHEEL angle, +ve = left turn
  e_y           m        Lateral path error at front axle, +ve = left of path
  e_psi_deg     deg      Heading error vs path tangent, +ve = CCW/left
  yaw_rate      rad/s    Measured yaw rate, +ve = CCW/left
  delta_cmd     rad      MPC's commanded steering (u[0]); steer_deg in radians
  a_cmd         m/s^2    MPC's commanded longitudinal accel (u[1]), -ve = brake
  solver_failed 0/1      1 if the MPC solve failed this step
  inaccurate    0/1      1 if the solver returned OPTIMAL_INACCURATE

delta_cmd/a_cmd are logged in the MPC's own units (rad, m/s^2) rather than
normalised FSDS command units precisely so the score below can be computed
from the file without re-deriving the scaling.

COLUMNS — path CSV
==================
  t             s        Run-relative time of this path snapshot (~1 Hz)
  idx           -        Waypoint index within that snapshot
  x, y          m        Global ENU waypoint position (same frame as car_x/y)


UNITS CONTRACT
==============
log_control()'s steer_rad and e_psi_rad are RADIANS and are converted to
degrees on write.  Callers driving FSDS must NOT pass the normalised
ControlCommand.steering ([-1, 1]); scale it back by MAX_STEER_RAD first.  Doing
otherwise silently produces a steer_deg column inflated by 180/(pi*MAX_STEER_RAD)
(~2.3x at a 25 deg limit) that still looks plausible — this bug hid live
slew-rate saturation for an entire tuning cycle.


SCORE HEADER
============
close() prepends a `#`-commented block holding the composite score and all
component metrics, computed by fsae_control.scoring — a verbatim copy of
fsae_MPCTest/sim/scoring.py.  A live run and an offline tuner rollout are
therefore graded by identical maths and are directly comparable.

Every header line starts with `#`, so `pandas.read_csv(path, comment='#')`
parses the file unchanged.  numpy's genfromtxt does NOT skip comment lines
when locating the `names=True` row, so it needs the count explicitly:

    n = sum(1 for l in open(path) if l.startswith('#'))
    np.genfromtxt(path, delimiter=',', names=True, skip_header=n)

The metrics are also returned directly from close(), so tooling generally
does not need to parse them back out of the header at all.

Caveat recorded in the header as `score_is_partial`: the live car cannot
measure `time_bonus` (needs a known step budget) or `offtrack` (needs
ground-truth track edges).  When the caller supplies neither, those terms are
0.0/False and the score is comparable to an offline score only in its
weighted-metric component.
"""
import csv
import math
import os
import time

from fsae_control.scoring import RolloutMetrics


class ControlLogger:
    def __init__(self, tag: str, log_dir: str = '', path_period: float = 1.0,
                 max_steer_rad: float = math.radians(25.0)):
        log_dir = os.path.expanduser(log_dir) if log_dir else os.path.expanduser('~/fsae_logs')
        os.makedirs(log_dir, exist_ok=True)
        stamp = int(time.time())
        self._tag = tag
        self._ctrl_path = os.path.join(log_dir, f'{tag}_control_{stamp}.csv')
        self._path_path = os.path.join(log_dir, f'{tag}_path_{stamp}.csv')

        self._ctrl_f = open(self._ctrl_path, 'w', newline='')
        self._path_f = open(self._path_path, 'w', newline='')
        self._ctrl_w = csv.writer(self._ctrl_f)
        self._path_w = csv.writer(self._path_f)
        self._ctrl_w.writerow(
            ['t', 'car_x', 'car_y', 'car_yaw', 'v_actual', 'v_desired',
             'steer_deg', 'e_y', 'e_psi_deg', 'yaw_rate',
             'delta_cmd', 'a_cmd', 'solver_failed', 'inaccurate',
             # ── Latency diagnostics ──────────────────────────────────────
             # Added to answer a specific question: the offline simulator
             # assumes DELAY_STEPS=1 (50 ms) of actuation lag and a perfectly
             # uniform 20 Hz loop, and does NOT reproduce the sustained
             # heading error seen on the car (live |e_psi| mean 15.9 deg /
             # p90 42.0 vs sim 6.0 / 13.8 on the same map with the same
             # gains). These five columns measure the real latency chain so
             # that assumption can be checked rather than trusted.
             'pose_age_s',      # age of the pose the solve used, seconds
             'path_age_s',      # age of the planner path the solve used
             'n_delay',         # rollforward depth the controller chose
             'solve_ms',        # QP solve wall time
             'cmd_latency_ms',  # loop start -> command published
             ])
        self._path_w.writerow(['t', 'idx', 'x', 'y'])

        self._path_period = path_period
        self._last_path_t: float | None = None
        self._n = 0

        # Run-relative time origin — set from the first sample of either
        # stream so both CSVs share one t=0. See "TIME" in the module docstring.
        self._t0: float | None = None
        self._t0_epoch: float = float(stamp)

        # Live scoring accumulator (fsae_control.scoring == offline scoring).
        self._metrics = RolloutMetrics()
        self._max_steer_rad = float(max_steer_rad)
        self._closed = False

    @property
    def paths(self) -> tuple[str, str]:
        return self._ctrl_path, self._path_path

    def _rel(self, t: float) -> float:
        """Absolute clock reading -> run-relative seconds (first sample = 0.0)."""
        if self._t0 is None:
            self._t0 = float(t)
            # Prefer the true epoch of the first sample over construction time.
            self._t0_epoch = float(t)
        return float(t) - self._t0

    def log_control(self, t, car_x, car_y, car_yaw, v_actual, v_desired,
                    steer_rad, e_y, e_psi_rad, yaw_rate,
                    delta_cmd=None, a_cmd=None,
                    solver_failed=False, inaccurate=False,
                    pose_age_s=None, path_age_s=None, n_delay=None,
                    solve_ms=None, cmd_latency_ms=None) -> None:
        """
        Record one control step.  See the module docstring for the units and
        frame of every argument.

        delta_cmd (rad) / a_cmd (m/s^2) are the MPC's raw command pair; when
        omitted, delta_cmd falls back to steer_rad and a_cmd to 0.0 so the
        Stanley controller (which has no longitudinal command) still logs and
        scores its lateral behaviour.

        The five latency arguments are optional and written as empty cells when
        not supplied, so callers that don't have them (Stanley) still log.
          pose_age_s      age of the pose fed to this solve (s)
          path_age_s      age of the planner path fed to this solve (s)
          n_delay         integer rollforward depth the controller chose
          solve_ms        QP solve wall time (ms)
          cmd_latency_ms  loop entry -> command publish (ms)
        """
        def _f(x, fmt='.4f'):
            return '' if x is None else format(float(x), fmt)
        if delta_cmd is None:
            delta_cmd = steer_rad
        if a_cmd is None:
            a_cmd = 0.0

        t_rel = self._rel(t)
        self._ctrl_w.writerow([
            f'{t_rel:.4f}', f'{car_x:.4f}', f'{car_y:.4f}', f'{car_yaw:.5f}',
            f'{v_actual:.3f}', f'{v_desired:.3f}', f'{math.degrees(steer_rad):.3f}',
            f'{e_y:.4f}', f'{math.degrees(e_psi_rad):.3f}', f'{yaw_rate:.4f}',
            f'{delta_cmd:.6f}', f'{a_cmd:.4f}',
            int(bool(solver_failed)), int(bool(inaccurate)),
            _f(pose_age_s), _f(path_age_s),
            '' if n_delay is None else int(n_delay),
            _f(solve_ms, '.3f'), _f(cmd_latency_ms, '.3f'),
        ])

        # Same accumulation the offline tuner runs, step for step.
        self._metrics.add_step(
            e_y=e_y, e_psi=e_psi_rad, r=yaw_rate,
            u_opt=(delta_cmd, a_cmd),
            v_target=v_desired, v_actual=v_actual,
            u_max_steer=self._max_steer_rad,
            solver_failed=bool(solver_failed), inaccurate=bool(inaccurate),
        )

        self._n += 1
        if self._n % 20 == 0:          # flush ~1 s so a Ctrl-C leaves valid data
            self._ctrl_f.flush()

    def log_path(self, t, path) -> None:
        t_rel = self._rel(t)
        if self._last_path_t is not None and (t_rel - self._last_path_t) < self._path_period:
            return
        self._last_path_t = t_rel
        for i, pt in enumerate(path):
            self._path_w.writerow([f'{t_rel:.4f}', i, f'{float(pt[0]):.4f}', f'{float(pt[1]):.4f}'])
        self._path_f.flush()

    def score(self, progress: float = 0.0, time_bonus: float = 0.0,
              dnf: bool = False, offtrack: bool = False) -> dict:
        """
        Finalise the accumulated metrics into the same dict the offline
        tuner's RolloutMetrics.finalize() returns.  Safe to call more than
        once; does not consume the accumulator.
        """
        return self._metrics.finalize(
            progress=progress, time_bonus=time_bonus, dnf=dnf, offtrack=offtrack,
        )

    def _write_score_header(self, result: dict, partial: bool) -> None:
        """
        Rewrite the control CSV with a `#`-commented score block on top.

        A header can't be prepended in place, so the body is read back and
        re-written.  Done once, at close, on a file of ~100 KB/min — cheap
        enough, and it keeps the score physically attached to the data it
        describes instead of in a sidecar that gets separated from it.
        """
        try:
            with open(self._ctrl_path, 'r', newline='') as f:
                body = f.read()
        except OSError:
            return

        if body.startswith('#'):        # already headed; don't double-prepend
            return

        lines = [
            f'# fsae control log — tag={self._tag}',
            f'# t0_epoch_s={self._t0_epoch:.4f}  (t column is seconds since this instant)',
            '# frame=global ENU (x east, y north, yaw right-handed, 0=+x); '
            'e_y/e_psi are front-axle Frenet errors vs the path, +ve = left/CCW',
            '# score: fsae_control.scoring, verbatim copy of '
            'fsae_MPCTest/sim/scoring.py — lower is better',
            f'# score_is_partial={int(partial)}'
            '  (1 = time_bonus/offtrack unavailable live; weighted-metric '
            'component is still directly comparable to an offline score)',
        ]
        for key in (
            'composite_score', 'n_steps', 'rmse', 'peak_lateral_error_m',
            'speed_rmse_mps', 'yaw_rms_radps', 'max_yaw_rate_radps',
            'control_smooth_rms', 'jerk_rms', 'steer_rms', 'accel_rms_mps2',
            'max_steering_rad', 'max_accel_mps2', 'steering_sat_ratio',
            'steering_reversal_rms', 'steering_reversal_rate',
            'steering_reversals', 'inaccurate_count',
        ):
            if key in result:
                val = result[key]
                val_s = f'{val:.6f}' if isinstance(val, float) else str(val)
                lines.append(f'# {key}={val_s}')

        tmp = self._ctrl_path + '.tmp'
        try:
            with open(tmp, 'w', newline='') as f:
                f.write('\n'.join(lines) + '\n')
                f.write(body)
            os.replace(tmp, self._ctrl_path)
        except OSError:
            # Never let a logging failure take down the control node; the
            # un-headed CSV is still perfectly usable.
            try:
                os.remove(tmp)
            except OSError:
                pass

    def close(self, progress: float = 0.0, time_bonus: float = 0.0,
              dnf: bool = False, offtrack: bool = False) -> dict | None:
        """
        Flush and close both CSVs, then prepend the score header to the
        control CSV.  Returns the finalised metrics dict (None if nothing was
        logged).  Idempotent.
        """
        if self._closed:
            return None
        self._closed = True

        for f in (self._ctrl_f, self._path_f):
            try:
                f.close()
            except Exception:
                pass

        if self._n == 0:
            return None

        result = self.score(progress=progress, time_bonus=time_bonus,
                            dnf=dnf, offtrack=offtrack)
        partial = (time_bonus == 0.0 and not offtrack)
        self._write_score_header(result, partial)
        return result
