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
The algorithm iterates over every point in the path and applies a look-ahead
heuristic to find the maximum upcoming curvature. 

  1. Look-ahead Window:
     At each path point, it samples upcoming points between a `scan_start` 
     and `scan_end` distance.
  2. 3-Point Curvature Estimation:
     It uses a 3-point cross-product method across the sampled window to 
     find the maximum geometric curvature (κ_max) ahead of the vehicle.
  3. Speed Target Generation:
     Using the friction circle approximation (a_c = v² * κ), it sets the 
     target speed to v_target = sqrt(a_lat_max / κ_max), keeping it bounded 
     between the global v_min and an effective v_max (which scales down near 
     the end of the path).

This method directly mirrors the ROS2 planner's speed profiling behavior, 
prioritizing upcoming severe corners over strict point-to-point kinematic 
acceleration limits.

USED BY
-------
  simulation.py    — on_release() and load_test_path() call compute_speed_profile()
                     + smooth_profile() to build path_v_profile for the simulator.
  offline_tuner.py — _resample_path() calls both functions to build path_v for
                     every synthetic test path; also used in scoring time bonus.

DOES NOT USE
------------
  vehicle_physics.py, bicycle_model.py, optimiser.py, sim_track.py, performance_stats.py
"""

import numpy as np
import math

# Note: not called by the active compute_speed_profile; retained for previous speed profile function
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
    denom = (dx**2 + dy**2) ** 1.2
    # Floor to avoid division by near-zero on duplicate or very close path points
    denom = np.where(denom < 1e-6, 1e-6, denom)

    kappa = (dx * ddy - dy * ddx) / denom
    return kappa


# def compute_speed_profile(
#     path_X, path_Y,
#     v_max=20.0,
#     mu=0.6,
#     g=9.81,
#     a_accel_max=4.0,
#     a_brake_max=-5.0,
#     v_min=1.5,
#     safety=1.0,
#     scan_end=14.0,
# ):
#     """
#     Compute a physically achievable per-point target speed profile along the path.

#     The three-pass algorithm (corner limit → forward → backward) produces the
#     fastest achievable speed profile that respects both cornering grip limits
#     and longitudinal acceleration/braking limits simultaneously.

#     Parameter design rationale
#     --------------------------
#     v_max=20.0:
#         Soft cap on straight-line speed. The project's previous fixed v_ref was
#         7.0 m/s; 20.0 m/s allows meaningfully faster straights without asking the
#         8-state linear MPC — which was tuned near 7-10 m/s — to operate far
#         outside its linearisation range.

#     mu=0.6:
#         Planning-level friction coefficient, deliberately lower than the tyre
#         model's peak mu=1.6 (about 37%). This provides:
#           1. Headroom for the MPC to handle combined-slip (simultaneous Fx+Fy)
#           2. Margin for model-plant mismatch and disturbances
#           3. Avoidance of the nonlinear regime where the linear MPC model
#              becomes inaccurate
#         At mu=0.6 and R=9 m: v_corner = sqrt(0.6*9.81/0.111) ≈ 7.3 m/s.

#     a_accel_max=4.0 / a_brake_max=−5.0:
#         Match the vehicle's actuator bounds in optimiser.py (u_max[1]=5.0,
#         u_min[1]=−5.0) so the profiler doesn't request speeds that the
#         controller's own limits prevent it from achieving.

#     safety=1.0:
#         Multiplicative factor on corner speed. Set below 1.0 (e.g. 0.85−0.95)
#         if spline smoothing underestimates true path curvature at tight corners.
#         Left at 1.0 here; the mu margin (above) already provides implicit safety.

#     scan_end=14.0:
#         When the visible path is shorter than scan_end metres, v_max is scaled
#         down proportionally. This mirrors the ROS2 planner's behaviour: less
#         visible look-ahead → less confidence → slower speed cap. Prevents the
#         profiler from commanding high speeds on short path fragments.

#     Parameters
#     ----------
#     path_X : array-like, shape (n,)
#         X coordinates of the path (m).
#     path_Y : array-like, shape (n,)
#         Y coordinates of the path (m).
#     v_max : float
#         Absolute speed cap (m/s). Applied after curvature and path length checks.
#     mu : float
#         Friction coefficient for planning-level corner speed limit. Intentionally
#         conservative relative to vehicle_physics.py's peak mu=1.6.
#     g : float
#         Gravitational acceleration (m/s²).
#     a_accel_max : float
#         Maximum longitudinal acceleration for forward pass (m/s²). Must be positive.
#     a_brake_max : float
#         Maximum longitudinal deceleration for backward pass (m/s²). Must be negative.
#     v_min : float
#         Minimum speed floor (m/s). Prevents near-zero targets on noisy curvature spikes.
#     safety : float
#         Multiplicative factor applied to curvature-derived corner speed. Default 1.0.
#     scan_end : float
#         Reference path length (m) for v_max scaling. Shorter visible paths get
#         a proportionally reduced speed cap.

#     Returns
#     -------
#     v_profile : np.ndarray, shape (n,)
#         Target speed at each path point (m/s), in range [v_min, v_max_eff].

#     Called by: simulation.py (on_release, load_test_path),
#                offline_tuner.py (_resample_path)
#     """
#     path_X = np.asarray(path_X)
#     path_Y = np.asarray(path_Y)
#     n = len(path_X)
#     if n < 3:
#         # Not enough points to compute curvature; return flat profile
#         return np.full(n, v_max)

#     # Arc length between consecutive path points (m)
#     # ds[i] = distance from point i to point i+1; ds[-1] is a repeat of ds[-2]
#     ds = np.hypot(np.diff(path_X), np.diff(path_Y))
#     ds = np.append(ds, ds[-1] if len(ds) > 0 else 1.0)   # Pad to length n

#     # Short-path v_max scaling: if total path < scan_end, we lack look-ahead
#     # confidence to plan at full v_max. Scale linearly down to zero at zero length.
#     total_arc = float(ds[:-1].sum())
#     v_max_eff = max(v_min, v_max * min(1.0, total_arc / scan_end))

#     # ── Pass 0: Corner speed limit at every point ─────────────────────────────
#     # v_corner = safety * sqrt(mu * g / |κ|)
#     # This is the speed at which required centripetal acceleration v²*κ equals
#     # the available lateral grip mu*g (friction circle approximation).
#     kappa     = compute_path_curvature(path_X, path_Y)
#     kappa_abs = np.maximum(np.abs(kappa), 1e-6)   # Floor to avoid infinite speed
#     # Apply safety multiplier before forward/backward passes so it propagates correctly
#     v_corner  = safety * np.sqrt(mu * g / kappa_abs)
#     v_profile = np.clip(v_corner, v_min, v_max_eff)

#     # ── Pass 1: Forward pass (acceleration limit) ─────────────────────────────
#     # Cannot speed up faster than a_accel_max allows between points.
#     # Kinematic relation: v_next² = v_prev² + 2 * a * ds
#     # → v_next ≤ sqrt(v_prev² + 2 * a_accel_max * ds[i])
#     for i in range(1, n):
#         v_allowed = np.sqrt(v_profile[i - 1] ** 2 + 2 * a_accel_max * ds[i - 1])
#         v_profile[i] = min(v_profile[i], min(v_allowed, v_max_eff))

#     # ── Pass 2: Backward pass (braking limit) ─────────────────────────────────
#     # Must already be slow enough at each point to brake down to the next
#     # point's speed limit. a_brake_max is negative.
#     # Kinematic relation: v_prev² = v_next² + 2 * a_brake * ds (a_brake < 0)
#     # → v_prev ≤ sqrt(v_next² + 2 * a_brake * ds[i])   [radicand may → 0]
#     for i in range(n - 2, -1, -1):
#         radicand  = v_profile[i + 1] ** 2 + 2 * a_brake_max * ds[i]
#         v_profile[i] = min(v_profile[i], np.sqrt(np.maximum(radicand, 0.0)))

#     return np.clip(v_profile, v_min, v_max_eff)

# Speed profiler that matches the fsae planning node speed profiler
def compute_speed_profile(
    path_X, path_Y,
    v_max=20.0,
    mu=0.6,
    g=9.81,
    a_accel_max=4.0,  # Maintained for signature compatibility, unused by path_utils logic
    a_brake_max=-5.0, # Maintained for signature compatibility, unused by path_utils logic
    v_min=1.5,
    safety=1.0,
    scan_end=14.0,
):
    """
    Compute a per-point target speed profile using a forward look-ahead heuristic.

    At each path point, samples upcoming points in the window [scan_start, scan_end]
    metres ahead and finds the maximum curvature using the 3-point cross-product
    method. Target speed is derived from the friction circle limit:
        v_target = safety * sqrt(a_lat_max / kappa_max)

    The a_accel_max and a_brake_max parameters are kept for signature compatibility
    with the old three-pass implementation but are not used.
    """
    path_X = np.asarray(path_X)
    path_Y = np.asarray(path_Y)
    n = len(path_X)
    v_profile = np.full(n, float(v_max))

    if n < 3:
        return v_profile

    a_lat_max = mu * g  # Map speed_profile params to path_utils a_lat_max
    scan_start = 1.5
    step = 2.0

    # Pre-compute cumulative arc length for the entire path
    pts = np.column_stack([path_X, path_Y])
    segs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    arc = np.concatenate([[0.0], np.cumsum(segs)])
    total_length = arc[-1]

    for i in range(n):
        remaining_arc = total_length - arc[i]
        
        # Scale down v_max if we are near the end of the known path
        v_max_eff = max(v_min, v_max * min(1.0, remaining_arc / scan_end))

        if remaining_arc < scan_start + step:
            v_profile[i] = float(v_max_eff)
            continue

        # Sample points ahead of the current position [i]
        sample_arcs = np.arange(arc[i] + scan_start, min(arc[i] + scan_end, total_length), step)
        if len(sample_arcs) < 3:
            v_profile[i] = float(v_max_eff)
            continue

        sx = np.interp(sample_arcs, arc, path_X)
        sy = np.interp(sample_arcs, arc, path_Y)
        sampled_pts = np.column_stack([sx, sy])

        # Find maximum curvature in the look-ahead window (3-point method)
        max_kappa = 0.0
        for j in range(1, len(sampled_pts) - 1):
            p1, p2, p3 = sampled_pts[j - 1], sampled_pts[j], sampled_pts[j + 1]
            d12 = float(np.linalg.norm(p2 - p1))
            d23 = float(np.linalg.norm(p3 - p2))
            d31 = float(np.linalg.norm(p1 - p3))
            
            denom = d12 * d23 * d31
            if denom < 1e-9:
                continue
                
            v1 = p2 - p1
            v2 = p3 - p1
            cross = abs(float(v1[0] * v2[1] - v1[1] * v2[0]))
            kappa = 2.0 * cross / denom
            
            if kappa > max_kappa:
                max_kappa = kappa

        # Calculate target speed for this specific point
        if max_kappa < 1e-4:
            v_profile[i] = float(v_max_eff)
        else:
            v_target = safety * math.sqrt(a_lat_max / max_kappa)
            v_profile[i] = float(max(v_min, min(v_max_eff, v_target)))

    return v_profile


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