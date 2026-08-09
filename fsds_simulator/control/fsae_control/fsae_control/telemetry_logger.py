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
             # These five columns measure the real latency chain (perception
             # + planning + control + actuation) so it can be checked against
             # the offline simulator's fixed-delay assumption rather than
             # trusted blindly.
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
