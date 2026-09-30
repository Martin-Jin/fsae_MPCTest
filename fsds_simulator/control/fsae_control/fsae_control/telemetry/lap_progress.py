"""
fsae_control/telemetry/lap_progress.py — LapProgressTracker

Derives real progress/completion/time-bonus from the car position against a
precomputed track path, so a live score is meaningful.
"""

import math

import numpy as np


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

        # Multi-lap: once a lap finishes, _idx/_reached_end/_start_wall are
        # re-armed for the next lap (see update()) rather than staying
        # latched forever. lap_idx counts COMPLETED laps (0 during the
        # first lap). _armed guards the re-finish check: right after a
        # reset the car is still sitting in the finish zone that just
        # triggered, so the finish check must not fire again until the car
        # has actually travelled a real distance since (see update()'s
        # _dist_since_reset -- NOT index progress, which can advance with
        # zero real motion or jump straight back to the finish on a
        # closed-loop path; both failure modes were caught by synthetic
        # tests before this landed on distance).
        self.lap_idx = 0
        self._armed = True
        self._dist_since_reset = 0.0
        self._reset_car_pos: tuple[float, float] | None = None

    # Speed (m/s) above which the car counts as having launched, for the
    # lap-timer start. The clock MUST NOT start on the first control tick:
    # the node logs from the moment it comes up, which is before the GO
    # signal and before the car physically moves, so a first-tick start
    # silently folds the whole standstill into lap_time_s. Measured ~0.95 s
    # of dead time on a normal run -- enough to make lap times
    # non-comparable between runs (a longer hold looks like slower driving)
    # and to deflate time_bonus, and hence the composite score, by ~2%.
    # 0.5 m/s is well clear of pose noise at a standstill while still
    # triggering within one or two ticks of a real launch.
    LAUNCH_SPEED_MPS = 0.5

    def update(self, car_pos, now: float, car_speed: float | None = None) -> dict | None:
        """
        Advance the forward-bounded nearest-index search by one sample.

        `car_speed` (m/s) gates the lap-timer start: the clock begins on the
        first sample where the car is actually moving (see
        LAUNCH_SPEED_MPS), not on the first tick. Omitted (None) falls back
        to starting on the first call, preserving the old behaviour for any
        caller that has no speed to hand.

        Returns a completed-lap dict (see result()'s return shape, plus
        'lap_idx') the instant a lap finishes, so the caller can score and
        log it immediately rather than waiting for close(). Returns None on
        every other tick. The tracker then re-arms for the next lap: index
        search restarts from the beginning, the timer restarts from this
        finish instant (a flying lap), and lap_idx increments.
        """
        if len(self._path_X) < 2:
            return None

        if self._start_wall is None:
            if car_speed is None or abs(car_speed) >= self.LAUNCH_SPEED_MPS:
                self._start_wall = now

        # Track real distance travelled since the last reset FIRST (using
        # the car's raw position, independent of the index search below) --
        # see the _armed block for why this, not the index, is what gates
        # re-arming.
        if self._reset_car_pos is not None:
            self._dist_since_reset += math.hypot(
                car_pos[0] - self._reset_car_pos[0], car_pos[1] - self._reset_car_pos[1])
        self._reset_car_pos = (float(car_pos[0]), float(car_pos[1]))

        # Forward-bounded: only search from the current index onward, same
        # rationale as rollout_core.py's find_closest_reference_bounded — it
        # can't jump backward onto a spatially-close-but-lapped-already point.
        #
        # While not yet _armed, the search is ADDITIONALLY capped to how far
        # the car has actually travelled since the reset (plus margin) --
        # not the whole remaining array. On a CLOSED-LOOP path (start/finish
        # coincide), resetting _idx to 0 does not relocate the CAR: it's
        # still spatially closest to the path's LAST few points, not its
        # first, so an UNCAPPED search relocks straight back onto index
        # ~N-1 within a couple of ticks regardless of real progress,
        # multi-second before the car has gone anywhere -- confirmed by a
        # synthetic closed-loop test that kept re-finishing every ~2.6 s
        # (one tenth of a real lap) instead of once per real lap. Capping
        # the window to (distance travelled + a fixed margin) means the
        # index literally cannot outrun the car's own odometry, so it can
        # only reach the finish zone once the car actually has.
        if self._armed:
            window_end = len(self._path_X)
        else:
            reachable = self._dist_since_reset + 5.0   # margin: path curvature vs. straight-line odometry
            idx_cap = self._idx + int(np.searchsorted(
                self._cum_len[self._idx:] - self._cum_len[self._idx], reachable))
            window_end = min(len(self._path_X), max(self._idx + 1, idx_cap))
        window = self._path_X[self._idx:window_end]
        d2 = (window - car_pos[0]) ** 2 + (self._path_Y[self._idx:window_end] - car_pos[1]) ** 2
        self._idx += int(np.argmin(d2))

        # 10% window / 3 m radius: mirrors fsae_MPCTest/sim/rollout_core.py's
        # identical check — see that file's comment for why these values
        # (not independently measured, but wide/narrow enough to avoid a
        # false trigger on a lap's own start/finish straight).
        near_end = self._idx >= len(self._path_X) - max(1, int(0.1 * len(self._path_X))) - 1
        dist_to_finish = math.hypot(
            car_pos[0] - self._path_X[-1], car_pos[1] - self._path_Y[-1]
        )
        in_finish_zone = self._idx >= len(self._path_X) - 1 or (near_end and dist_to_finish > 0.0 and dist_to_finish <= 3.0)

        # _armed prevents an immediate re-finish right after a reset. Gated
        # on actual distance travelled (see above), which -- combined with
        # the capped search window above -- means the car has to have
        # covered a meaningful fraction of the track (not merely "the
        # search index says so") before the next finish can be recognised.
        # A car sitting still at a point-to-point path's end (no capped
        # window needed there, distance never advances) also never
        # re-arms, so it stays finished rather than re-triggering on tiny
        # index jitter -- caught by a synthetic point-to-point test.
        if not self._armed:
            if self._dist_since_reset >= 0.2 * self._path_length:
                self._armed = True
            return None

        if in_finish_zone:
            lap_time_s = None
            if self._start_wall is not None:
                lap_time_s = float(now - self._start_wall)
            time_bonus = 0.0
            if self._optimal_time > 0.0 and lap_time_s and lap_time_s > 0.0:
                time_bonus = float(np.clip(self._optimal_time / lap_time_s, 0.0, 1.0))

            self.lap_idx += 1
            completed = {
                'lap_idx': self.lap_idx,
                'progress': 1.0,
                'reached_end': True,
                'time_bonus': time_bonus,
                'lap_time_s': lap_time_s,
                'optimal_time_s': self._optimal_time,
            }

            # Re-arm: flying-lap timer (next lap starts timing at this
            # finish instant), index search restarts from the top, distance
            # accumulator restarts from zero (see the _armed gate above).
            self._idx = 0
            self._start_wall = now
            self._end_wall = None
            self._reached_end = False
            self._armed = False
            self._dist_since_reset = 0.0
            return completed

        return None

    def result(self, now: float) -> dict:
        """
        progress/reached_end/time_bonus for ControlLogger.close(). Safe to
        call at any time (e.g. mid-run on an early shutdown) — reached_end
        stays False and time_bonus stays 0.0 until update() has actually
        seen the car cross the finish check.

        Reflects whatever lap is CURRENTLY in progress, not the whole run:
        once a lap completes, update() reports it directly (see its
        docstring) and re-arms this tracker for the next lap, so a call to
        result() after N completed laps describes lap N+1 (partial if the
        run ends mid-lap). Multi-lap runs should use update()'s returned
        per-lap dicts for completed laps and this method only for the
        trailing partial lap at shutdown.
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
