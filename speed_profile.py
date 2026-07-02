"""
Curvature-Based Speed Profiler
File Name: speed_profile.py

Replaces the simulator's fixed v_ref with a per-point target speed along
the drawn path, computed from path curvature -- the same basic approach
used in racing-line / lap-time-simulation tools:

  1. Compute curvature kappa(s) at every point of the path.
  2. Corner speed limit: v_corner = sqrt(mu * g / |kappa|), capped at
     v_max. This is the speed at which the required centripetal
     acceleration (v^2 * kappa) exactly equals the available lateral
     grip (mu * g) -- the same friction-circle idea already used in the
     nonlinear tire model in vehicle_physics.py, just applied to a single
     combined friction limit here rather than per-axle Pacejka curves
     (good enough for path-level speed planning; the MPC/tire model still
     handles the real per-axle force distribution moment to moment).
  3. Forward pass: speed can only increase along the path as fast as the
     vehicle's longitudinal acceleration limit allows (v_next^2 <= v^2 +
     2*a_max*ds).
  4. Backward pass: speed must already be low enough to decelerate down
     to each corner's limit in time (v_prev^2 <= v_next^2 + 2*a_brake*ds),
     i.e. you have to start braking before the corner, not at it.

The result is a smooth, physically-achievable target speed at every point
along the path, which the simulator looks up (via the same closest-point
search already used for lateral error) instead of using a constant v_ref.
"""
import numpy as np


def compute_path_curvature(path_X, path_Y):
    """
    Curvature kappa(s) = (x'y'' - y'x'') / (x'^2+y'^2)^1.5, via finite
    differences on the (already densely resampled) path arrays.
    Returns kappa, same length as path_X/path_Y.
    """
    path_X = np.asarray(path_X)
    path_Y = np.asarray(path_Y)
    dx = np.gradient(path_X)
    dy = np.gradient(path_Y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)

    denom = (dx**2 + dy**2) ** 1.5
    denom = np.where(denom < 1e-6, 1e-6, denom)  # avoid div-by-zero on duplicate points
    kappa = (dx * ddy - dy * ddx) / denom

    return kappa


def compute_speed_profile(
    path_X, path_Y,
    v_max=16.0,
    mu=1.1,
    g=9.81,
    a_accel_max=3.5,
    a_brake_max=4.5,
    v_min=2.5,
):
    """
    Computes a per-point target speed profile along the path.

    v_max:        absolute top speed cap (straight-line limited, e.g. by
                   powertrain), m/s. Set well below the plant's theoretical
                   max so the MPC always has headroom to correct -- the
                   project's previous fixed v_ref was 7.0 m/s, so 14 m/s
                   gives meaningfully faster straights without asking the
                   8-state linear MPC model to operate far outside the
                   speed range it was effectively tuned/tested at.
    mu:            friction coefficient used for the *planning* speed limit.
                   vehicle_physics.VehicleParams.mu (the tire model's peak
                   friction) is 1.6 -- this is deliberately lower (~70% of
                   peak) so the profiler doesn't plan speeds that demand
                   the absolute friction limit, leaving margin for the
                   nonlinear tire model's actual combined-slip behavior
                   and any disturbance the MPC needs to correct for.
    g:             gravity, m/s^2.
    a_accel_max:   max longitudinal acceleration the forward pass assumes
                   available for speeding up, m/s^2. Kept at/below the
                   controller's actual a_cmd actuator bound (+5 m/s^2 in
                   simulation.py's u_bounds_max) so the profile the MPC is
                   asked to follow is achievable given its own limits.
    a_brake_max:   max longitudinal deceleration the backward pass assumes
                   available for slowing down before a corner, m/s^2.
                   Larger than a_accel_max since braking grip typically
                   exceeds drive grip, but still within the |a_cmd|<=5
                   actuator bound.
    v_min:         floor speed, so tight/noisy curvature spikes (e.g. from
                   path-drawing jitter) can't drive the target to a stop.

    Returns: v_profile, np.array same length as path_X (m/s at each point).
    """
    path_X = np.asarray(path_X)
    path_Y = np.asarray(path_Y)
    n = len(path_X)
    if n < 3:
        return np.full(n, v_max)

    # --- Arc length between consecutive points ---
    ds = np.hypot(np.diff(path_X), np.diff(path_Y))
    ds = np.append(ds, ds[-1] if len(ds) > 0 else 1.0)  # pad to length n

    # --- Step 1: curvature-limited corner speed at every point ---
    kappa = compute_path_curvature(path_X, path_Y)
    kappa_abs = np.maximum(np.abs(kappa), 1e-6)
    v_corner = np.sqrt(mu * g / kappa_abs)
    v_profile = np.clip(v_corner, v_min, v_max)

    # --- Step 2: forward pass (acceleration limit) ---
    # Can't speed up faster along the path than a_accel_max allows, even if
    # the corner limit ahead would otherwise permit it.
    for i in range(1, n):
        v_allowed = np.sqrt(v_profile[i - 1] ** 2 + 2 * a_accel_max * ds[i - 1])
        v_profile[i] = min(v_profile[i], v_allowed)

    # --- Step 3: backward pass (braking limit) ---
    # Must already be slow enough, looking backward from each point, to
    # decelerate down to it in time -- i.e. braking has to start before
    # the corner, not at the corner entry.
    for i in range(n - 2, -1, -1):
        v_allowed = np.sqrt(v_profile[i + 1] ** 2 + 2 * a_brake_max * ds[i])
        v_profile[i] = min(v_profile[i], v_allowed)

    return np.clip(v_profile, v_min, v_max)


def smooth_profile(v_profile, window=9):
    """
    Light moving-average smoothing on the final profile. The forward/
    backward passes already produce a continuous (non-jumpy) profile by
    construction, but raw per-point curvature can still be noisy on a
    hand-drawn / spline-fit path, so this takes the edge off without
    undoing the acceleration/braking limit shape.
    """
    if window < 2 or len(v_profile) < window:
        return v_profile
    kernel = np.ones(window) / window
    # 'same' mode + edge padding so the array length/endpoints are preserved
    padded = np.pad(v_profile, (window // 2, window // 2), mode="edge")
    smoothed = np.convolve(padded, kernel, mode="valid")
    return smoothed[: len(v_profile)]