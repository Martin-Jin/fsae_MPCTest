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
"""
import csv
import dataclasses
import math
import os
import time

import numpy as np

from fsae_control.scoring import RolloutMetrics


# ── Adaptive-feature trace columns ───────────────────────────────────────
# Written from mpc_core's last_telemetry (see its "Adaptive-feature trace"
# comment). Each m_* column is the multiplier that ONE feature applied to ONE
# weight on that tick, so 1.0 means "this feature did nothing here" and the
# product of a weight's m_* columns times its base value is its *_eff column.
# That decomposition is the point: it tells you which feature moved a weight,
# not merely that the weight moved.
#
# Order here defines CSV column order; log_control writes one cell per
# entry, in this order, by looking each key up in the `adaptive` mapping.
# Controllers with no adaptive features (Stanley) pass nothing and every cell
# is written empty, so the column set stays identical across controllers.
ADAPTIVE_COLUMNS = (
    # Corner-factor scheduler (mpc_core._corner_factor/_low_speed_corner_boost):
    # a single CURRENT-curvature-driven fraction, and the low-speed boost
    # that adds to it, both gated multiplicatively so the boost cannot fire
    # on low speed alone.
    'corner_factor',           # 0 (straight) -> 1 (full corner), from current-position kappa only
    'low_speed_corner_boost',  # extra push toward "full corner", active only when corner_factor > 0 AND speed is low
    'corner_frac',             # corner_factor + low_speed_corner_boost, clipped to [0,1] -- the shared blend driver below
    # Q[0,0]/Q[2,2]/Q[3,3] after the straight/corner blend, before adaptive_Q_scaling's centred-softening.
    'Q_ey_base', 'Q_epsi_base', 'Q_r_base',
    # R[0,0]/R_rate[0,0] after their own straight/corner blend.
    'R_steer_corner_blend', 'Rrate_steer_corner_blend',
    # Per-feature multipliers still in use.
    'm_Q_ey_soften',      # adaptive_Q_scaling's centred-softening multiplier
    'm_R_speed',          # adaptive_R_scaling's speed-based multiplier
    'm_Rrate_corner',     # adaptive_R_rate's current-curvature floor multiplier
    'm_Rrate_antihunt',   # steer_rate_anti_hunt's straight/centred/aligned boost multiplier
    # Absolute weights handed to the QP after all of the above.
    'Q_ey_eff', 'Q_epsi_eff', 'Q_r_eff', 'R_steer_eff', 'Rrate_steer_eff',
    'R_a_accel_eff', 'R_a_brake_eff',  # a_cmd effort weight, split by sign; R_a_*_eff already includes the heading-error-driven asymmetry (epsi_ra_*)
    # ── NMPC-only columns (nmpc_core.NMPCController; empty for every LTV-QP
    # run, exactly as the m_* columns are empty for Stanley). Appended at the
    # END so the existing column order — and every offline script that parses
    # these CSVs by name — is unaffected.
    #
    # These are the NMPC's equivalent of the m_* decomposition: they say what
    # the solver did (how many Gauss-Newton iterations, whether the QP
    # subproblem actually solved, the achieved cost) and what its own
    # prediction expected (terminal e_y/e_psi, peak predicted |e_y|,
    # curvature at the far end of the horizon). A run where nmpc_status
    # spends time at 0, or where nmpc_pred_ey_end disagrees badly with the
    # e_y actually reached ~1 s later, is a model/solver problem rather than
    # a weighting problem — which is the distinction the LTV-QP's telemetry
    # could never make.
    'nmpc_iters',              # Gauss-Newton SQP iterations actually taken this tick
    'nmpc_status',             # 1.0 = QP subproblem solved, 0.0 = not (rejected/failed/budget)
    'nmpc_cost',               # achieved nonlinear cost of the shipped trajectory
    'nmpc_s0',                 # arc length of the car's Frenet projection (m)
    'nmpc_kappa_horizon_end',  # kappa(s) at the far end of the PREDICTED horizon (1/m)
    'nmpc_pred_ey_end',        # predicted e_y at the end of the horizon (m)
    'nmpc_pred_epsi_end',      # predicted e_psi at the end of the horizon (rad)
    'nmpc_pred_ey_max_abs',    # peak predicted |e_y| anywhere in the horizon (m)
)


def build_config_lines(
    controller: str,
    launch_flags: dict | None = None,
    mpc_params=None,
    nmpc_params=None,
    nmpc_effective: dict | None = None,
) -> list[str]:
    """
    Build the `#`-commented run-configuration dump ControlLogger.
    set_config_lines() stores for `_write_score_header()` to write. Every
    line is returned ALREADY prefixed with `# ` (ControlLogger.
    set_config_lines() does not add its own prefix).

    Parameters
    ----------
    controller : str
        'stanley' | 'mpc' | 'mpc_standalone' -- whichever node built this.
    launch_flags : dict, optional
        Arbitrary key -> value pairs the CALLER already has from its own
        get_parameter() calls (map_path, path_map_path,
        use_precomputed_heading_profile, enable_dynamic_speed_cap,
        dynamic_cap_a_lat_max, dynamic_cap_safety, v_max, v_min, ...) --
        this function doesn't know or care what a node declares, it just
        formats whatever dict it's handed. Empty/None values are logged as
        such (not skipped), since "map_path was empty" (-> live planner
        mode) is exactly the kind of thing worth being able to read back.
    mpc_params : mpc_params.MPCParams, optional
        Dumped via dataclasses.asdict() -- every field, whatever they
        currently are. This is DELIBERATE: hand-listing field names here
        would silently go stale every time a weight is added, renamed or
        removed (as already happened once this session, see the
        corner_factor rewrite) -- asdict() cannot go stale, it reflects
        whatever MPCParams actually is at import time.
    nmpc_params : nmpc_params.NMPCParams, optional
        Same treatment, only meaningful when controller used_nmpc=True.
    nmpc_effective : dict, optional
        The NMPC's RESOLVED weights after its own `-1.0`-inherits-from-
        MPCParams logic (e.g. {'w_out': ctrl.w_out.tolist(), 'r_delta':
        ctrl.r_delta, ...} read straight off the constructed
        NMPCController) -- logged separately from nmpc_params's raw
        (possibly -1.0) override fields so a reader doesn't have to
        mentally re-run the inheritance to know what was ACTUALLY used.

    What this does NOT capture (read this before assuming the dump is
    complete): the adaptive-gain SCHEME itself -- e.g. today's
    `_corner_factor`/`_blend` continuous blend in mpc_core.py -- is CODE,
    not a parameter, and isn't reproducible from a config dump the way a
    numeric weight is. If the scheme changes (as it already has once this
    session), a config header from an OLDER run describes weights for a
    scheme that no longer exists. `mpc_core.py`'s own module/function
    docstrings are the authoritative description of the CURRENT scheme;
    this dump only ever tells you the NUMBERS that scheme was using.
    """
    lines: list[str] = [f'# controller={controller}']
    lines.append(f'# use_nmpc={int(bool(nmpc_effective))}')

    if launch_flags:
        for key, val in launch_flags.items():
            lines.append(f'# launch.{key}={val!r}')

    if mpc_params is not None:
        for key, val in dataclasses.asdict(mpc_params).items():
            lines.append(f'# mpc_params.{key}={val!r}')

    if nmpc_params is not None:
        for key, val in dataclasses.asdict(nmpc_params).items():
            lines.append(f'# nmpc_params.{key}={val!r}')

    if nmpc_effective:
        for key, val in nmpc_effective.items():
            lines.append(f'# nmpc_effective.{key}={val!r}')

    return lines


class LapProgressTracker:
    """
    Turns a precomputed (path_X, path_Y, path_V) speed profile plus a stream
    of car positions into the progress/reached_end/time_bonus terms
    compute_composite_score() needs, so a live run stops being permanently
    scored as "never finished" (see close()'s previous progress=0.0 default,
    which pinned every live composite_score at CONSTRAINT_FLOOR + DNF_PENALTY
    regardless of how the car actually drove).

    Mirrors fsae_MPCTest/sim/rollout_core.py's own progress/reached_end/
    time_bonus derivation as closely as the live node's available data
    allows: same nearest-index-forward-bounded-search shape for progress,
    same "near the last point" reached_end check. The one deliberate
    difference is optimal_time: rollout_core.py calls speed_profile.py's
    quasi-steady-state optimal_lap_time() solver, which lives in
    fsae_MPCTest and is not on the live node's PYTHONPATH (see
    CLAUDE.md's scoring-parity note — the car has no fsae_MPCTest checkout).
    Since the live node already loads the SAME profile's v_target curve
    (load_speed_profile_csv, precomputed offline by that same solver) to
    drive the car, integrating ds / v_target over it directly is a
    zero-new-dependency stand-in for calling the solver again.
    """

    def __init__(self, path_X, path_Y, path_V):
        self._path_X = np.asarray(path_X, dtype=float)
        self._path_Y = np.asarray(path_Y, dtype=float)
        seg_dx = np.diff(self._path_X)
        seg_dy = np.diff(self._path_Y)
        self._seg_len = np.hypot(seg_dx, seg_dy)
        self._cum_len = np.concatenate(([0.0], np.cumsum(self._seg_len)))
        self._path_length = float(self._cum_len[-1]) if len(self._cum_len) else 0.0

        # Optimal time = integral of ds / v_target over the precomputed
        # profile, using each segment's leading-point speed (matches how
        # path_V is sampled — one v_target per waypoint).
        v_seg = np.asarray(path_V, dtype=float)[:-1]
        v_seg = np.maximum(v_seg, 1e-3)   # guard a stray zero in the profile
        self._optimal_time = float(np.sum(self._seg_len / v_seg))

        self._idx = 0            # forward-bounded nearest-index, like rollout_core.py
        self._start_wall: float | None = None
        self._end_wall: float | None = None
        self._reached_end = False

    def update(self, car_pos, now: float) -> None:
        """Advance the forward-bounded nearest-index search by one sample."""
        if self._reached_end or len(self._path_X) < 2:
            return
        if self._start_wall is None:
            self._start_wall = now

        # Forward-bounded: only search from the current index onward, same
        # rationale as rollout_core.py's find_closest_reference_bounded — it
        # can't jump backward onto a spatially-close-but-lapped-already point.
        window = self._path_X[self._idx:]
        d2 = (window - car_pos[0]) ** 2 + (self._path_Y[self._idx:] - car_pos[1]) ** 2
        self._idx += int(np.argmin(d2))

        near_end = self._idx >= len(self._path_X) - max(1, int(0.1 * len(self._path_X))) - 1
        dist_to_finish = math.hypot(
            car_pos[0] - self._path_X[-1], car_pos[1] - self._path_Y[-1]
        )
        if self._idx >= len(self._path_X) - 1 or (near_end and dist_to_finish <= 3.0):
            self._reached_end = True
            self._end_wall = now

    def result(self, now: float) -> dict:
        """
        progress/reached_end/time_bonus for ControlLogger.close(). Safe to
        call at any time (e.g. mid-run on an early shutdown) — reached_end
        stays False and time_bonus stays 0.0 until update() has actually
        seen the car cross the finish check.
        """
        progress = float(np.clip(
            self._cum_len[min(self._idx, len(self._cum_len) - 1)] / self._path_length, 0.0, 1.0
        )) if self._path_length > 0 else 0.0

        time_bonus = 0.0
        lap_time_s = None
        if self._reached_end and self._start_wall is not None:
            lap_time_s = float((self._end_wall if self._end_wall is not None else now) - self._start_wall)
            if self._optimal_time > 0.0 and lap_time_s > 0.0:
                ref_time = self._optimal_time * max(progress, 1e-6)
                time_bonus = float(np.clip(ref_time / lap_time_s, 0.0, 1.0))

        return {
            'progress': progress,
            'reached_end': self._reached_end,
            'time_bonus': time_bonus,
            'lap_time_s': lap_time_s,
            'optimal_time_s': self._optimal_time,
        }


class ControlLogger:
    def __init__(self, tag: str, log_dir: str = '', path_period: float = 1.0,
                 max_steer_rad: float = math.radians(25.0),
                 config_lines: list[str] | None = None):
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
             ] + list(ADAPTIVE_COLUMNS))
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

        # Full-run-configuration dump (see module docstring's "Config
        # header" section): plain strings, ALREADY `# `-prefixed by the
        # caller (mpc_controller.py / mpc_controller_standalone.py /
        # stanley_controller.py -- see each node's own `_build_config_lines`-
        # style helper). Stored here rather than written immediately because
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
        ControlLogger in a node's __init__ (e.g.
        mpc_controller_standalone.py builds `self._telemetry` before
        `self._mpc`); calling this any time before close() is fine, the
        lines are only read at that point.
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
                    solve_ms=None, cmd_latency_ms=None, adaptive=None) -> None:
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
            # A key absent from `adaptive` writes an empty cell rather than a
            # default, so "this feature was disabled/not reported" stays
            # distinguishable from "this feature reported exactly 1.0".
            *[_f((adaptive or {}).get(k), '.5f') for k in ADAPTIVE_COLUMNS],
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
