"""
sim/sensor_noise.py — models of the live perception/localisation feed's
imperfections, applied by run_core_rollout() to what the controller and
planner SEE. The plant and the score always use the true state.

  SlamNoise     pose jitter + slow drift (FSDS has no real SLAM)
  ConeNoise     per-cone detection jitter
  PoseFeedHold  pose feed repeating its last sample (measured live)
"""

import numpy as np

from sim.rollout_phases import _normalize_angle


class SlamNoise:
    """
    Corrupts the pose the controller/planner SEE, leaving the true plant
    state untouched.

    Why this exists
    ---------------
    FSDS has no real SLAM. Its `sim_perception` node republishes ground-truth
    `/fsds/testing_only/odom` straight onto `/fsae/slam/car_position`, and the
    cone map is a latched oracle map cropped to a forward window — the only
    modelled limitation is sensor RANGE, not accuracy. This rollout mirrored
    that by feeding exact plant state back into the planner and the
    tracking-error maths, which makes the simulator systematically easier than
    the real car, whose pose comes from ZED visual odometry + cone_mapper.

    Localisation error matters here specifically because it lands directly in
    e_y/e_psi — the signals the MPC steers on. A pose that jitters makes the
    measured error jitter, and an under-damped controller chases it.

    NOT the cause of the observed FSDS chatter: FSDS's pose is already exact,
    so pose noise cannot explain steering reversal chatter seen there. That
    was instead caused by (a) the steering slew limit binding on a large
    fraction of steps and (b) sim_perception publishing the pose at a lower
    rate than the control loop, so many MPC solves re-used an unchanged
    pose. Both are fixed elsewhere; this class is for the REAL car's
    localisation error, and defaults to off.

    Model
    -----
    Two additive components, matching how real SLAM misbehaves:

      jitter — zero-mean white noise, redrawn every step. Causes chatter.
      drift  — a first-order (Ornstein-Uhlenbeck) process pulled back toward
               zero with time constant SLAM_DRIFT_TAU. Wanders over seconds
               and self-corrects, like a SLAM estimate between loop closures,
               instead of random-walking away over a long rollout.

    The OU update uses the exact discrete-time form
    ``d <- a*d + sqrt(1-a^2)*sigma*w`` with ``a = exp(-dt/tau)``, whose
    stationary standard deviation is exactly ``sigma`` regardless of dt — so
    SLAM_POS_DRIFT_STD means what it says and does not change meaning if DT
    changes.

    IMPORTANT: this is deliberately NOT applied to the state handed to the
    plant or to scoring. The car is judged on where it actually ended up, not
    where it believed it was — the same asymmetry real localisation error has.
    """

    def __init__(self, dt, seed, pos_jitter_std, yaw_jitter_std,
                 pos_drift_std, yaw_drift_std, drift_tau):
        self._rng = np.random.default_rng(seed)
        self._pos_jitter_std = float(pos_jitter_std)
        self._yaw_jitter_std = float(yaw_jitter_std)
        self._pos_drift_std = float(pos_drift_std)
        self._yaw_drift_std = float(yaw_drift_std)

        tau = max(float(drift_tau), 1e-6)
        self._a = float(np.exp(-float(dt) / tau))
        # Scale that makes the OU process's stationary std equal *_drift_std.
        self._q = float(np.sqrt(max(0.0, 1.0 - self._a * self._a)))

        # Start the drift at a stationary sample rather than 0, so step 0 isn't
        # artificially better-localised than the rest of the run.
        self._drift = self._rng.normal(0.0, 1.0, size=3)

    def corrupt(self, x, y, yaw):
        """Return (x_est, y_est, yaw_est) as the controller would measure them."""
        self._drift = self._a * self._drift + self._q * self._rng.normal(0.0, 1.0, size=3)
        jitter = self._rng.normal(0.0, 1.0, size=3)

        x_est = x + self._drift[0] * self._pos_drift_std + jitter[0] * self._pos_jitter_std
        y_est = y + self._drift[1] * self._pos_drift_std + jitter[1] * self._pos_jitter_std
        yaw_est = yaw + self._drift[2] * self._yaw_drift_std + jitter[2] * self._yaw_jitter_std
        return float(x_est), float(y_est), float(_normalize_angle(yaw_est))


class ConeNoise:
    """
    Corrupts cone positions AFTER SimPerception's FOV filter, modelling real
    per-detection vision noise on top of FSDS's noise-free oracle cone map.

    Why this exists
    ---------------
    sim_track.SimPerception.visible_cones() returns exact ground-truth cone
    coordinates, only cropped by range/FOV — see `docs/reference/simulator_fidelity.md`,
    "Simulator fidelity limits": the cone map was, until this class existed,
    the one aspect of the sim/real gap with literally no model at all. This
    adds the minimum needed to make perception-side hypotheses testable
    offline: independent per-cone, per-frame position jitter. It does NOT
    model false positives/negatives or range-dependent noise growth — both
    are real, both are still unmodelled, this only closes the position-jitter
    gap. See settings.CONE_NOISE_ENABLED for the full rationale.

    Unlike SlamNoise, there is no drift/bias component: a vision cone detector
    re-estimates each cone's position independently every frame rather than
    tracking one belief over time, so there is nothing that should carry over
    between frames the way SLAM's OU drift does. If a future measurement shows
    real cone detections DO carry frame-to-frame correlated error (e.g. from a
    slowly-drifting camera calibration), add a drift term the same way
    SlamNoise does rather than repurposing jitter for it.

    Deliberately applied to EACH CONE INDEPENDENTLY, not once per frame as a
    shared offset — SlamNoise corrupts a single pose shared by the whole cone
    set, which is correct for localisation error, but detection error is
    per-object (each cone has its own range/angle/occlusion to the sensor).
    """

    def __init__(self, seed, pos_jitter_std):
        self._rng = np.random.default_rng(seed)
        self._pos_jitter_std = float(pos_jitter_std)

    def corrupt(self, cones):
        """Return a copy of `cones` (n, 2) with independent per-point jitter."""
        if len(cones) == 0 or self._pos_jitter_std <= 0.0:
            return cones
        return cones + self._rng.normal(0.0, self._pos_jitter_std, size=cones.shape)


class PoseFeedHold:
    """
    Models the live pose feed REPEATING the previous measurement instead of
    delivering a fresh one — the dominant sim-to-real gap.

    WHY THIS EXISTS
    ---------------
    The offline rollout gave the controller a brand-new, exact pose every
    single tick. DELAY_STEPS models a fixed 50 ms lag, but the controller
    still learns something new every step, so its heading error can never
    accumulate. The real car does not work that way: `/fsae/slam/car_position`
    intermittently stops publishing, and the controller re-uses the last pose
    it received while the car keeps moving.

    Measured from live telemetry across runs on the same track and same
    tuned gains, differing only in how badly the feed stalled: a healthy run
    has a low repeated-tick rate and short holds, while a degraded run can
    see the majority of ticks repeating with holds approaching a second. In
    one such degraded run the pose froze for roughly a second at speed —
    enough distance travelled with no positional update that, when the feed
    resumed, the heading error was unrecoverable and the car spun. Both runs
    used identical weights on identical track; the only difference was the
    feed.

    This is NOT the same thing as DELAY_STEPS or DELAY_JITTER_STEPS:
      - DELAY_STEPS delays a pose that is still FRESH each tick.
      - DELAY_JITTER_STEPS perturbs only the controller's BELIEF about the lag.
      - This repeats the DATA, so pose_age genuinely ramps and the controller
        is flying blind. Nothing in the previous model produced that.

    MODEL
    -----
    Two-state Markov chain over "fresh" and "held":
      - each tick, with probability `p_hold`, a hold begins
      - hold length is drawn geometrically, mean `mean_hold_ticks`, capped at
        `max_hold_ticks`
    A geometric hold length reproduces the measured histogram shape well: many
    short 2-tick holds, a thin tail of long ones. Fitting anything more
    elaborate to two runs would be overfitting.

    The whole ESTIMATED state is frozen, not just x/y — the live log shows
    v_actual and yaw repeating alongside position, because they come from the
    same odometry message. Freezing position while letting speed update would
    model a failure mode that does not exist.

    Seeded, so rollouts stay reproducible and CMA-ES still gets a stable score
    per candidate (see settings.POSE_HOLD_SEED).
    """

    def __init__(self, p_hold, mean_hold_ticks, max_hold_ticks, seed):
        self._rng = np.random.default_rng(seed)
        self._p_hold = float(np.clip(p_hold, 0.0, 1.0))
        # Geometric success prob giving the requested mean hold length.
        # Repeats per hold average mean_hold - 1 (see apply()); guard the
        # degenerate mean_hold <= 1 case, which means "never actually hold".
        self._mean_hold = max(float(mean_hold_ticks), 1.0)
        self._q_repeat = 1.0 / max(self._mean_hold - 1.0, 1e-6)
        self._q_repeat = float(np.clip(self._q_repeat, 1e-6, 1.0))
        self._max_hold = int(max(1, max_hold_ticks))
        self._remaining = 0          # ticks still to hold
        self._held = None            # the frozen (state, X, Y, psi) tuple

    def apply(self, state_est, X_est, Y_est, psi_est):
        """
        Return the pose the controller actually sees this tick, plus how many
        ticks old it is.

        Returns (state_est, X, Y, psi, age_ticks). age_ticks is 0 on a fresh
        sample and increments through a hold, mirroring the live pose_age_s.
        """
        if self._remaining > 0:
            self._remaining -= 1
            s, x, y, psi, age = self._held
            self._held = (s, x, y, psi, age + 1)
            return s, x, y, psi, age + 1

        # Fresh sample this tick; decide whether the NEXT ticks are held.
        # A "hold of length L" spans L ticks TOTAL (this fresh one plus L-1
        # repeats), matching how the live histogram was counted, so the number
        # of repeat ticks to schedule is L-1. np.random.geometric returns >= 1,
        # hence the -1: without it every hold ran one tick long and the mean
        # came out at 2.94 against a measured 2.08.
        if self._rng.random() < self._p_hold:
            # A hold spans `mean_hold_ticks` ticks TOTAL (this fresh one plus
            # the repeats), matching how the live histogram was counted, so the
            # number of REPEATS to schedule averages mean_hold - 1. A geometric
            # draw has mean 1/q, hence q = 1/(mean_hold - 1).
            self._remaining = int(np.clip(
                self._rng.geometric(self._q_repeat), 1, self._max_hold - 1))

        self._held = (state_est.copy(), float(X_est), float(Y_est), float(psi_est), 0)
        return state_est, X_est, Y_est, psi_est, 0
