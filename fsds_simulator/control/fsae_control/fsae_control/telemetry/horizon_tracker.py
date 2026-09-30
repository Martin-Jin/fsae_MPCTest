"""
fsae_control/telemetry/horizon_tracker.py — HorizonAccuracyTracker

Compares the controller's predicted trajectory against where the car actually
went, per lap.
"""

import math

import numpy as np


class HorizonAccuracyTracker:
    """
    Compares the NMPC's predicted horizon (published each tick as Cartesian
    front-axle points, see nmpc_core.py's xy_at()) against where the car
    actually was once enough time has passed for the prediction to "come
    true". NMPC-only: the LTV-QP path never exposes a Cartesian horizon (see
    mpc_core.py), so a caller with no predictions to feed just never calls
    add_prediction() and update() always returns None.

    A prediction made at tick k covers stages j=0..N at times
    t_stage0 + j*dt (t_stage0 already accounts for pose age and any
    rollforward delay — see the caller). It "matures" once the pose history
    covers its last stage's time, at which point the mean Euclidean distance
    between each predicted point and the car's actual (interpolated)
    position at that same time is computed. That mean error, divided by the
    horizon's own arc length, is what turns into the accuracy percentage
    (see the caller / docs/debugging_tools.md for the exact formula) — this
    class only produces the raw mean error and lets the caller apply that
    formula, so the percentage definition lives in one place.
    """

    # Below this speed a "prediction" is really just "stay where you are"
    # and would score as near-perfect for the wrong reason (no motion to get
    # wrong). Same gate as LapProgressTracker.LAUNCH_SPEED_MPS, so a run's
    # lap accuracy and its lap timer agree on what counts as "actually
    # driving".
    LAUNCH_SPEED_MPS = 0.5

    # How much pose history to retain for interpolating "where was the car
    # at time t". Must exceed the horizon length (dt*N, currently 1.0 s)
    # with margin for pose_age_s/n_delay/n_latency pushing t_stage0 earlier
    # than "now".
    _POSE_HISTORY_S = 3.0

    def __init__(self):
        self._pose_t: list[float] = []
        self._pose_xy: list[tuple[float, float]] = []
        # Pending predictions not yet matured: each is (t_stage0, dt, xs, ys).
        self._pending: list[tuple[float, float, np.ndarray, np.ndarray]] = []
        self.last_err_m: float | None = None
        self.last_acc_pct: float | None = None
        # Running sum for the CURRENT lap's mean accuracy (what
        # pop_lap_mean() reports at finish_lap() time) — kept separately
        # from last_err_m/last_acc_pct (this tick's own value) because the
        # per-lap figure the plan calls for is the mean over every
        # prediction that matured DURING the lap, not just the last one.
        self._lap_err_sum = 0.0
        self._lap_acc_sum = 0.0
        self._lap_n = 0

    def add_pose(self, t: float, x_front: float, y_front: float) -> None:
        self._pose_t.append(float(t))
        self._pose_xy.append((float(x_front), float(y_front)))
        cutoff = t - self._POSE_HISTORY_S
        while len(self._pose_t) > 2 and self._pose_t[1] < cutoff:
            self._pose_t.pop(0)
            self._pose_xy.pop(0)

    def add_prediction(self, t_stage0: float, dt: float, xs, ys, v0: float) -> None:
        if v0 < self.LAUNCH_SPEED_MPS:
            return
        xs = np.asarray(xs, dtype=float)
        ys = np.asarray(ys, dtype=float)
        if xs.size < 2:
            return
        self._pending.append((float(t_stage0), float(dt), xs, ys))

    def _actual_at(self, t: float) -> tuple[float, float] | None:
        ts = self._pose_t
        if len(ts) < 2 or t < ts[0] or t > ts[-1]:
            return None
        idx = int(np.searchsorted(ts, t))
        idx = min(max(idx, 1), len(ts) - 1)
        t0, t1 = ts[idx - 1], ts[idx]
        x0, y0 = self._pose_xy[idx - 1]
        x1, y1 = self._pose_xy[idx]
        if t1 <= t0:
            return x0, y0
        frac = (t - t0) / (t1 - t0)
        return x0 + frac * (x1 - x0), y0 + frac * (y1 - y0)

    def update(self, now: float) -> tuple[float, float] | None:
        """
        Evaluate any predictions that have fully matured by `now`. Returns
        the most recently matured (err_m, acc_pct), or None if nothing
        matured this call (the pending prediction still needs more pose
        history, or there was nothing to evaluate). Also accumulates into
        a running per-lap sum — see pop_lap_mean(), which the caller uses
        at lap-end instead of this per-tick return value, since a single
        tick's result would only reflect the last prediction to mature,
        not the lap as a whole.
        """
        result = None
        still_pending = []
        for t_stage0, dt, xs, ys in self._pending:
            t_last = t_stage0 + dt * (len(xs) - 1)
            if t_last > now or not self._pose_t or t_last > self._pose_t[-1]:
                still_pending.append((t_stage0, dt, xs, ys))
                continue
            errs = []
            path_len = 0.0
            prev = None
            for j in range(len(xs)):
                actual = self._actual_at(t_stage0 + dt * j)
                if actual is not None:
                    errs.append(math.hypot(xs[j] - actual[0], ys[j] - actual[1]))
                if prev is not None:
                    path_len += math.hypot(xs[j] - prev[0], ys[j] - prev[1])
                prev = (xs[j], ys[j])
            if not errs:
                continue
            err_m = float(np.mean(errs))
            acc_pct = float(np.clip(1.0 - err_m / max(path_len, 1.0), 0.0, 1.0)) * 100.0
            self.last_err_m, self.last_acc_pct = err_m, acc_pct
            self._lap_err_sum += err_m
            self._lap_acc_sum += acc_pct
            self._lap_n += 1
            result = (err_m, acc_pct)
        self._pending = still_pending
        return result

    def pop_lap_mean(self) -> tuple[float, float] | None:
        """
        Mean (err_m, acc_pct) over every prediction that matured since the
        last call (i.e. during the just-completed lap), then resets the
        running sums for the next lap. Returns None if nothing matured
        during the lap (e.g. a very short lap, or a run that never reached
        launch speed) — the caller (finish_lap()) then logs pred_acc_pct as
        n/a for that lap rather than a misleading 0%/100%.
        """
        if self._lap_n == 0:
            return None
        mean = (self._lap_err_sum / self._lap_n, self._lap_acc_sum / self._lap_n)
        self._lap_err_sum = self._lap_acc_sum = 0.0
        self._lap_n = 0
        return mean
