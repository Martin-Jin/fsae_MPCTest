"""
speed_profile.py — Curvature-Based Speed Profiler

PURPOSE
-------
Computes a physically achievable per-point target speed along a path, replacing
the simulator's previous fixed v_ref constant. The profiler uses the same
fundamental approach as racing-line / lap-time simulation tools: limit corner
speed by centripetal acceleration demand, then propagate acceleration and braking
limits forward and backward along the path.

The result is a smooth speed profile that:
  - Goes fast on straights (up to v_max)
  - Slows appropriately before corners (proportional to curvature)
  - Respects the vehicle's longitudinal acceleration and braking limits
  - Provides the MPC's speed reference via simulation.py and offline_tuner.py

HOW THE PROFILING WORKS
-----------------------
The algorithm has three passes:

  Pass 0 — Corner speed limit at each point:
      The centripetal acceleration needed to follow a path of curvature κ at
      speed v is: a_c = v² * κ. Setting a_c = mu*g gives the grip-limited
      corner speed: v_corner = sqrt(mu*g / κ). This is the speed at which the
      lateral tyre force demand exactly equals the available lateral grip,
      using the same friction-circle concept as vehicle_physics.py's tyre model
      (but applied at the path planning level rather than per-tyre).

  Pass 1 — Forward pass (acceleration limit):
      Starting from the first point, each subsequent point's speed is capped by
      how fast the vehicle can accelerate from the previous point:
          v_next² ≤ v_prev² + 2 * a_accel_max * ds
      Prevents the profile from asking for unreachable speed gains between points.

  Pass 2 — Backward pass (braking limit):
      Starting from the last point, each previous point's speed is capped by
      how early braking must begin to reach the next point's speed limit:
          v_prev² ≤ v_next² + 2 * |a_brake_max| * ds
      This enforces "you must start braking before the corner, not at it",
      which is the critical insight for generating realistic speed profiles.

USED BY
-------
  simulation.py    — on_release() and load_test_path() call compute_speed_profile()
                     + smooth_profile() to build path_v_profile for the simulator.
  offline_tuner.py — _resample_path() calls both functions to build path_v for
                     every synthetic test path; also used in scoring time bonus.

DOES NOT USE
------------
  vehicle_physics.py, model.py, optimiser.py, sim_track.py, performance_stats.py
"""

import numpy as np


def compute_path_curvature(path_X, path_Y):
    """
    Compute the signed curvature κ(s) at each point along the path using
    finite differences of path coordinates.

    Physics / geometry:
    For a curve parameterised by arc length s with coordinates (x(s), y(s)):
        κ = (x' y'' − y' x'') / (x'² + y'²)^(3/2)
    where primes denote derivatives with respect to the parameter (here arc length).

    Positive κ = path curves left (counterclockwise); negative = curves right.
    The magnitude |κ| = 1/R where R is the radius of curvature at that point.

    numpy.gradient uses central differences on interior points and one-sided
    differences at endpoints, which gives O(h²) accuracy and handles
    non-uniformly-sampled paths (though paths are typically densely resampled
    before this is called).

    Parameters
    ----------
    path_X : array-like, shape (n,)
        X coordinates of the path (m).
    path_Y : array-like, shape (n,)
        Y coordinates of the path (m).

    Returns
    -------
    kappa : np.ndarray, shape (n,)
        Signed curvature at each path point (rad/m = 1/m).

    Called by: compute_speed_profile()
    """
    path_X = np.asarray(path_X)
    path_Y = np.asarray(path_Y)
    dx  = np.gradient(path_X)     # First derivative: dx/dt
    dy  = np.gradient(path_Y)     # First derivative: dy/dt
    ddx = np.gradient(dx)         # Second derivative: d²x/dt²
    ddy = np.gradient(dy)         # Second derivative: d²y/dt²

    # Denominator: (x'² + y'²)^(3/2) — the speed cubed in the parameter
    denom = (dx**2 + dy**2) ** 1.5
    # Floor to avoid division by near-zero on duplicate or very close path points
    denom = np.where(denom < 1e-6, 1e-6, denom)

    kappa = (dx * ddy - dy * ddx) / denom
    return kappa


def compute_speed_profile(
    path_X, path_Y,
    v_max=18.0,
    mu=0.6,
    g=9.81,
    a_accel_max=4.0,
    a_brake_max=-5.0,
    v_min=2.5,
    safety=1.0,
    scan_end=14.0,
):
    """
    Compute a physically achievable per-point target speed profile along the path.

    The three-pass algorithm (corner limit → forward → backward) produces the
    fastest achievable speed profile that respects both cornering grip limits
    and longitudinal acceleration/braking limits simultaneously.

    Parameter design rationale
    --------------------------
    v_max=18.0:
        Soft cap on straight-line speed. The project's previous fixed v_ref was
        7.0 m/s; 18.0 m/s allows meaningfully faster straights without asking the
        8-state linear MPC — which was tuned near 7-10 m/s — to operate far
        outside its linearisation range.

    mu=0.6:
        Planning-level friction coefficient, deliberately lower than the tyre
        model's peak mu=1.6 (about 37%). This provides:
          1. Headroom for the MPC to handle combined-slip (simultaneous Fx+Fy)
          2. Margin for model-plant mismatch and disturbances
          3. Avoidance of the nonlinear regime where the linear MPC model
             becomes inaccurate
        At mu=0.6 and R=9 m: v_corner = sqrt(0.6*9.81/0.111) ≈ 7.3 m/s.

    a_accel_max=5.0 / a_brake_max=−5.0:
        Match the vehicle's actuator bounds in optimiser.py (u_max[1]=5.0,
        u_min[1]=−5.0) so the profiler doesn't request speeds that the
        controller's own limits prevent it from achieving.

    safety=1.0:
        Multiplicative factor on corner speed. Set below 1.0 (e.g. 0.85−0.95)
        if spline smoothing underestimates true path curvature at tight corners.
        Left at 1.0 here; the mu margin (above) already provides implicit safety.

    scan_end=14.0:
        When the visible path is shorter than scan_end metres, v_max is scaled
        down proportionally. This mirrors the ROS2 planner's behaviour: less
        visible look-ahead → less confidence → slower speed cap. Prevents the
        profiler from commanding high speeds on short path fragments.

    Parameters
    ----------
    path_X : array-like, shape (n,)
        X coordinates of the path (m).
    path_Y : array-like, shape (n,)
        Y coordinates of the path (m).
    v_max : float
        Absolute speed cap (m/s). Applied after curvature and path length checks.
    mu : float
        Friction coefficient for planning-level corner speed limit. Intentionally
        conservative relative to vehicle_physics.py's peak mu=1.6.
    g : float
        Gravitational acceleration (m/s²).
    a_accel_max : float
        Maximum longitudinal acceleration for forward pass (m/s²). Must be positive.
    a_brake_max : float
        Maximum longitudinal deceleration for backward pass (m/s²). Must be negative.
    v_min : float
        Minimum speed floor (m/s). Prevents near-zero targets on noisy curvature spikes.
    safety : float
        Multiplicative factor applied to curvature-derived corner speed. Default 1.0.
    scan_end : float
        Reference path length (m) for v_max scaling. Shorter visible paths get
        a proportionally reduced speed cap.

    Returns
    -------
    v_profile : np.ndarray, shape (n,)
        Target speed at each path point (m/s), in range [v_min, v_max_eff].

    Called by: simulation.py (on_release, load_test_path),
               offline_tuner.py (_resample_path)
    """
    path_X = np.asarray(path_X)
    path_Y = np.asarray(path_Y)
    n = len(path_X)
    if n < 3:
        # Not enough points to compute curvature; return flat profile
        return np.full(n, v_max)

    # Arc length between consecutive path points (m)
    # ds[i] = distance from point i to point i+1; ds[-1] is a repeat of ds[-2]
    ds = np.hypot(np.diff(path_X), np.diff(path_Y))
    ds = np.append(ds, ds[-1] if len(ds) > 0 else 1.0)   # Pad to length n

    # Short-path v_max scaling: if total path < scan_end, we lack look-ahead
    # confidence to plan at full v_max. Scale linearly down to zero at zero length.
    total_arc = float(ds[:-1].sum())
    v_max_eff = max(v_min, v_max * min(1.0, total_arc / scan_end))

    # ── Pass 0: Corner speed limit at every point ─────────────────────────────
    # v_corner = safety * sqrt(mu * g / |κ|)
    # This is the speed at which required centripetal acceleration v²*κ equals
    # the available lateral grip mu*g (friction circle approximation).
    kappa     = compute_path_curvature(path_X, path_Y)
    kappa_abs = np.maximum(np.abs(kappa), 1e-6)   # Floor to avoid infinite speed
    # Apply safety multiplier before forward/backward passes so it propagates correctly
    v_corner  = safety * np.sqrt(mu * g / kappa_abs)
    v_profile = np.clip(v_corner, v_min, v_max_eff)

    # ── Pass 1: Forward pass (acceleration limit) ─────────────────────────────
    # Cannot speed up faster than a_accel_max allows between points.
    # Kinematic relation: v_next² = v_prev² + 2 * a * ds
    # → v_next ≤ sqrt(v_prev² + 2 * a_accel_max * ds[i])
    for i in range(1, n):
        v_allowed = np.sqrt(v_profile[i - 1] ** 2 + 2 * a_accel_max * ds[i - 1])
        v_profile[i] = min(v_profile[i], min(v_allowed, v_max_eff))

    # ── Pass 2: Backward pass (braking limit) ─────────────────────────────────
    # Must already be slow enough at each point to brake down to the next
    # point's speed limit. a_brake_max is negative.
    # Kinematic relation: v_prev² = v_next² + 2 * a_brake * ds (a_brake < 0)
    # → v_prev ≤ sqrt(v_next² + 2 * a_brake * ds[i])   [radicand may → 0]
    for i in range(n - 2, -1, -1):
        radicand  = v_profile[i + 1] ** 2 + 2 * a_brake_max * ds[i]
        v_profile[i] = min(v_profile[i], np.sqrt(np.maximum(radicand, 0.0)))

    return np.clip(v_profile, v_min, v_max_eff)


def smooth_profile(v_profile, window=9):
    """
    Apply a light moving-average smoothing pass to the speed profile.

    The forward/backward passes in compute_speed_profile() already produce a
    continuous, non-jumpy profile by construction (the kinematic constraints
    prevent discontinuities). However, raw per-point curvature estimated from
    finite differences on a hand-drawn or spline-fit path can still be noisy,
    creating small speed oscillations that would cause unnecessary throttle
    cycling by the MPC.

    The moving average smooths these oscillations without altering the overall
    acceleration/braking shape, because the window (9 points ≈ 0.45 m at
    0.05 m spacing) is much shorter than the deceleration/acceleration ramps.

    Edge padding ('edge' mode) replicates the first and last values to prevent
    the convolution from reducing speed near path endpoints.

    Parameters
    ----------
    v_profile : np.ndarray, shape (n,)
        Raw speed profile from compute_speed_profile().
    window : int
        Moving average window width (number of path points). Default 9.
        Larger values = smoother but may flatten sharp braking zones.

    Returns
    -------
    smoothed : np.ndarray, shape (n,)
        Smoothed speed profile, same shape as input.

    Called by: simulation.py (on_release, load_test_path),
               offline_tuner.py (_resample_path)
    """
    if window < 2 or len(v_profile) < window:
        return v_profile   # Too short to smooth; return unchanged
    kernel = np.ones(window) / window   # Uniform (box) average kernel
    # Pad to preserve array length: pad by window//2 on each side with edge values
    padded   = np.pad(v_profile, (window // 2, window // 2), mode="edge")
    smoothed = np.convolve(padded, kernel, mode="valid")
    return smoothed[:len(v_profile)]   # Trim to original length (handles odd/even window)