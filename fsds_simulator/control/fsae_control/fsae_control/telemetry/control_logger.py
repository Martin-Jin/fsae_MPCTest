"""
fsae_control/telemetry/control_logger.py — ControlLogger

Writes the two per-run CSVs (control and path), the score header and the
config header; accumulates the per-step score metrics.
"""

import csv
import math
import os
import time

import numpy as np

from fsae_control.telemetry.scoring import RolloutMetrics
from fsae_control.telemetry.columns import ADAPTIVE_COLUMNS


class ControlLogger:
    def __init__(self, tag: str, log_dir: str = '', path_period: float = 1.0,
                 max_steer_rad: float = math.radians(25.0),
                 config_lines: list[str] | None = None):
        log_dir = os.path.expanduser(log_dir) if log_dir else os.path.expanduser('~/fsae_logs')
        os.makedirs(log_dir, exist_ok=True)
        # Local wall-clock stamp, YYYYmmdd-HHMMSS. Chosen over the epoch
        # seconds this used to write because a log's filename is the only
        # thing that identifies it in a directory listing, and an operator
        # comparing "the run before lunch" against "the one after" cannot do
        # that from 1787532892.
        #
        # Format is deliberately lexicographically sortable, so a plain `ls`
        # or a filename sort puts runs in chronological order — which the
        # epoch form also gave, and which readers (see
        # tuner/tools/plot_playback.py's _stamp) still rely on. Local time,
        # not UTC: it is read by a person standing next to the car.
        # Two separate values, deliberately: `stamp` names the FILE and is for
        # a human reading a directory listing; `now_epoch` is the numeric time
        # origin written into the CSV header and must stay a float. They were
        # briefly the same variable, which crashed the node at startup once the
        # filename form stopped being numeric.
        now_epoch = time.time()
        stamp = time.strftime('%Y%m%d-%H%M%S', time.localtime(now_epoch))
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
             # ── Lap / horizon-accuracy diagnostics ───────────────────────
             # lap_idx: which lap this tick belongs to (0 during the first
             # lap, matching LapProgressTracker.lap_idx before its first
             # completion). pred_err_m/pred_acc_pct: this tick's OWN matured
             # prediction-vs-actual comparison (empty when none matured this
             # tick, or always empty on a non-NMPC run — see
             # HorizonAccuracyTracker). lap_score/lap_pred_acc_pct: filled
             # ONLY on the row where a lap just completed — see
             # ControlLogger.finish_lap().
             'lap_idx', 'pred_err_m', 'pred_acc_pct',
             'lap_score', 'lap_pred_acc_pct',
             ] + list(ADAPTIVE_COLUMNS))
        self._path_w.writerow(['t', 'idx', 'x', 'y'])

        self._path_period = path_period
        self._last_path_t: float | None = None
        self._n = 0

        # Run-relative time origin — set from the first sample of either
        # stream so both CSVs share one t=0. See "TIME" in the module docstring.
        self._t0: float | None = None
        self._t0_epoch: float = now_epoch

        # Live scoring accumulator (fsae_control.scoring == offline scoring).
        # Whole-run accumulator, used by close()'s header score exactly as
        # before multi-lap existed.
        self._metrics = RolloutMetrics()
        # Per-lap accumulator: reset every time finish_lap() is called, so
        # each lap's composite_score reflects only that lap's ticks rather
        # than the whole run's running sums (several of RolloutMetrics'
        # fields are max/count, not plain sums, so this has to be a
        # SEPARATE accumulator rather than a snapshot-and-difference of
        # the whole-run one).
        self._lap_metrics = RolloutMetrics()
        self._lap_summaries: list[dict] = []
        self._max_steer_rad = float(max_steer_rad)
        self._closed = False

        # Full-run-configuration dump (see module docstring's "Config
        # header" section): plain strings, ALREADY `# `-prefixed by the
        # caller (mpc_controller.py / stanley_controller.py -- see each
        # node's own `_build_config_lines`-style helper). Stored here rather
        # than written immediately because
        # it's folded into the SAME single-rewrite-at-close() mechanism the
        # score header already uses (see _write_score_header) -- one header
        # write, not two.
        self._config_lines: list[str] = list(config_lines) if config_lines else []

    @property
    def paths(self) -> tuple[str, str]:
        return self._ctrl_path, self._path_path

    def set_config_lines(self, lines: list[str]) -> None:
        """
        Set (replacing any previous value) the run-configuration lines
        written into the score header at close(). Exists as a separate
        method rather than a constructor-only argument because the
        controller object (whose params/effective weights make up most of
        this dump — see build_config_lines()) is often constructed AFTER
        ControlLogger in a node's __init__ (e.g. mpc_controller.py builds
        `self._telemetry` before `self._mpc`); calling this any time before
        close() is fine, the lines are only read at that point.
        """
        self._config_lines = list(lines)

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
                    solve_ms=None, cmd_latency_ms=None, adaptive=None,
                    lap_idx=None, pred_err_m=None, pred_acc_pct=None,
                    lap_summary=None) -> None:
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

        `adaptive` is the controller's last_telemetry dict (or any mapping);
        the ADAPTIVE_COLUMNS keys are pulled out of it and everything else is
        ignored, so mpc_core can add telemetry keys without touching this
        file. Omit it (Stanley) and those cells are written empty.

        lap_idx/pred_err_m/pred_acc_pct come from the caller's
        LapProgressTracker/HorizonAccuracyTracker (see mpc_controller.py);
        omitted entirely (Stanley, or an NMPC run before its first matured
        prediction) they write empty cells, same convention as the rest of
        this method's optional columns.

        lap_summary is the dict finish_lap() returns, passed on the SAME
        tick a lap completes so its composite_score/pred_acc_pct land on
        the row that completed it (a CSV writer can't go back and fill an
        earlier row, so this has to happen before that row is written, not
        after). Every other tick omits it and those two cells are empty.
        """
        def _f(x, fmt='.4f'):
            return '' if x is None else format(float(x), fmt)
        if delta_cmd is None:
            delta_cmd = steer_rad
        if a_cmd is None:
            a_cmd = 0.0

        t_rel = self._rel(t)
        lap_score = lap_summary.get('composite_score') if lap_summary else None
        lap_pred_acc = lap_summary.get('pred_acc_pct') if lap_summary else None
        self._ctrl_w.writerow([
            f'{t_rel:.4f}', f'{car_x:.4f}', f'{car_y:.4f}', f'{car_yaw:.5f}',
            f'{v_actual:.3f}', f'{v_desired:.3f}', f'{math.degrees(steer_rad):.3f}',
            f'{e_y:.4f}', f'{math.degrees(e_psi_rad):.3f}', f'{yaw_rate:.4f}',
            f'{delta_cmd:.6f}', f'{a_cmd:.4f}',
            int(bool(solver_failed)), int(bool(inaccurate)),
            _f(pose_age_s), _f(path_age_s),
            '' if n_delay is None else int(n_delay),
            _f(solve_ms, '.3f'), _f(cmd_latency_ms, '.3f'),
            '' if lap_idx is None else int(lap_idx),
            _f(pred_err_m, '.4f'), _f(pred_acc_pct, '.2f'),
            _f(lap_score, '.6f'), _f(lap_pred_acc, '.2f'),
            # A key absent from `adaptive` writes an empty cell rather than a
            # default, so "this feature was disabled/not reported" stays
            # distinguishable from "this feature reported exactly 1.0".
            *[_f((adaptive or {}).get(k), '.5f') for k in ADAPTIVE_COLUMNS],
        ])

        # Same accumulation the offline tuner runs, step for step. Fed into
        # BOTH accumulators: _metrics for the whole-run header score (as
        # before multi-lap existed), _lap_metrics for the CURRENT lap only
        # (reset by finish_lap() each time a lap completes).
        step_kwargs = dict(
            e_y=e_y, e_psi=e_psi_rad, r=yaw_rate,
            u_opt=(delta_cmd, a_cmd),
            v_target=v_desired, v_actual=v_actual,
            u_max_steer=self._max_steer_rad,
            solver_failed=bool(solver_failed), inaccurate=bool(inaccurate),
        )
        self._metrics.add_step(**step_kwargs)
        self._lap_metrics.add_step(**step_kwargs)

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
              dnf: bool = False, offtrack: bool = False,
              reached_end: bool | None = None) -> dict:
        """
        Finalise the accumulated metrics into the same dict the offline
        tuner's RolloutMetrics.finalize() returns.  Safe to call more than
        once; does not consume the accumulator.

        reached_end should come from a LapProgressTracker.result() when one
        is in use — see compute_composite_score's docstring for why
        progress alone (a bounded nearest-index search that stops short of
        the final path point) can't reliably stand in for it.
        """
        return self._metrics.finalize(
            progress=progress, time_bonus=time_bonus, dnf=dnf, offtrack=offtrack,
            reached_end=reached_end,
        )

    def finish_lap(self, lap: dict, pred_acc_pct: float | None = None,
                   pred_err_m: float | None = None) -> dict:
        """
        Finalise the JUST-COMPLETED lap's own accumulator into a score, then
        reset it so the next lap starts from zero. Called by the node the
        instant LapProgressTracker.update() returns a completed-lap dict
        (see that method's docstring), on the SAME tick — its return value
        is meant to be passed straight into log_control()'s `lap_summary`
        kwarg so the score lands on the row that completed the lap.

        `lap` is that completed-lap dict: {'lap_idx', 'progress',
        'reached_end', 'time_bonus', 'lap_time_s', 'optimal_time_s'}.
        pred_acc_pct/pred_err_m are this lap's mean HorizonAccuracyTracker
        result (None for a non-NMPC run, or an NMPC run where nothing
        matured during the lap — e.g. a very short lap).

        Returns a dict: everything RolloutMetrics.finalize() returns (same
        shape as score()'s return value) plus 'lap_idx', 'lap_time_s',
        'pred_acc_pct', 'pred_err_m'. Stored in self._lap_summaries for the
        header at close().
        """
        result = self._lap_metrics.finalize(
            progress=lap.get('progress', 1.0),
            time_bonus=lap.get('time_bonus', 0.0),
            reached_end=lap.get('reached_end', True),
        )
        result['lap_idx'] = lap.get('lap_idx')
        result['lap_time_s'] = lap.get('lap_time_s')
        result['pred_acc_pct'] = pred_acc_pct
        result['pred_err_m'] = pred_err_m
        self._lap_summaries.append(result)
        self._lap_metrics = RolloutMetrics()
        return result

    def _write_score_header(self, result: dict, partial: bool,
                             lap_time_s: float | None = None,
                             optimal_time_s: float | None = None) -> None:
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
        if self._config_lines:
            lines.append('# ── run configuration (enough to reproduce this exact run) ──')
            lines.extend(self._config_lines)
        if lap_time_s is not None:
            lines.append(f'# lap_time_s={lap_time_s:.4f}')
        if optimal_time_s is not None:
            lines.append(f'# optimal_time_s={optimal_time_s:.4f}'
                         '  (ds/v_target integral over the precomputed speed'
                         ' profile, scaled by progress -- see LapProgressTracker)')

        # Per-lap lines, one lap's worth of score + horizon accuracy per
        # completed lap (see finish_lap()). Named lap_N_* (1-indexed) so a
        # human/script can grep a specific lap without parsing the whole
        # header. This is IN ADDITION to the whole-run composite_score
        # below (the current, in-progress-or-final lap at close() time),
        # not a replacement for it -- a single-lap run still gets the same
        # composite_score= line it always has, for backward compatibility
        # with anything that already parses that key.
        if self._lap_summaries:
            lines.append('# ── per-lap scores ──')
            for lap in self._lap_summaries:
                idx = lap.get('lap_idx')
                lap_time = lap.get('lap_time_s')
                acc = lap.get('pred_acc_pct')
                lines.append(f'# lap_{idx}_score={lap["composite_score"]:.6f}')
                if lap_time is not None:
                    lines.append(f'# lap_{idx}_time_s={lap_time:.4f}')
                lines.append(
                    f'# lap_{idx}_pred_acc_pct='
                    + (f'{acc:.2f}' if acc is not None else 'n/a'))

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

        # Whole-run horizon accuracy: mean of each completed lap's own mean
        # accuracy (NOT re-derived from the raw per-tick values -- those
        # aren't retained past finish_lap()'s reset). NMPC-only; absent
        # entirely on a Stanley/LTV-QP run or a run with zero completed
        # laps, same "column/line just doesn't exist" convention as the
        # rest of this header.
        lap_accs = [lap['pred_acc_pct'] for lap in self._lap_summaries
                    if lap.get('pred_acc_pct') is not None]
        if lap_accs:
            lines.append(f'# pred_acc_pct={float(np.mean(lap_accs)):.2f}'
                         '  (mean of each completed lap\'s own mean horizon accuracy)')

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
              dnf: bool = False, offtrack: bool = False,
              reached_end: bool | None = None,
              lap_time_s: float | None = None,
              optimal_time_s: float | None = None) -> dict | None:
        """
        Flush and close both CSVs, then prepend the score header to the
        control CSV.  Returns the finalised metrics dict (None if nothing was
        logged).  Idempotent.

        Callers should pass progress/reached_end/time_bonus from a
        LapProgressTracker rather than leaving them at their defaults: the
        defaults (progress=0.0, reached_end=None) make compute_composite_score
        treat every run as never having left the start line, permanently
        pinning composite_score at CONSTRAINT_FLOOR + DNF_PENALTY regardless
        of how the car drove.
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
                            dnf=dnf, offtrack=offtrack, reached_end=reached_end)
        partial = (time_bonus == 0.0 and not offtrack)
        self._write_score_header(result, partial, lap_time_s=lap_time_s,
                                  optimal_time_s=optimal_time_s)
        return result
