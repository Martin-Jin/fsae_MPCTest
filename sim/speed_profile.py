"""
sim/speed_profile.py — Curvature-Based Speed Profiler

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
  - Provides the MPC's speed reference via gui/simulation.py and tuner/offline_tuner.py

HOW THE PROFILING WORKS
-----------------------
compute_speed_profile() runs three passes over the path:

  Pass 0 — Look-ahead curvature limit:
     1. Look-ahead Window: at each path point, sample upcoming points between
        a `scan_start` and `scan_end` distance.
     2. 3-Point Curvature Estimation: use a 3-point cross-product method
        across the sampled window to find the maximum geometric curvature
        (κ_max) ahead of the vehicle.
     3. Speed Target Generation: using the friction circle approximation
        (a_c = v² * κ), set the target to v = sqrt(a_lat_max / κ_max),
        bounded between v_min and an effective v_max (which scales down near
        the end of the path).
  Pass 1 — Forward propagation: cap how fast the target may RISE between
     points, per a_accel_max.
  Pass 2 — Backward propagation: cap how fast it may FALL, i.e. require each
     point be slow enough to brake to the next point's limit, per
     a_brake_max.

Pass 0 alone mirrors the ROS2 planner's heuristic, but it only bounds speed
point-wise — it never checks consecutive targets are joined by an achievable
longitudinal acceleration, so it can demand a deceleration the car physically
cannot produce (measured at ~273 m/s² on a 60 m straight into a 5 m hairpin).
`speed_rmse` then penalises the controller for failing to do the impossible,
and the tuner contorts the gains chasing it. Passes 1-2 exist to make the
profile a trajectory the vehicle can actually follow.

PARITY
------
curvature_speed() in this module is a numeric-parity port of the live
control_utils.curvature_speed() (ros2/src/fsae_planning/control/fsae_control/).
It is the ONLY curvature heuristic here: compute_speed_profile()'s pass 0
calls it per path point rather than re-implementing it, so the oracle
reference profile and the live per-tick target cannot drift apart. Shared
defaults live in the CURVATURE_SPEED_* constants below and must be kept
equal to the live function's defaults.

This delegation fixed a real gap. The oracle profile previously used its own
copy of the heuristic with scan_end=14 m, a_lat_max=mu*g=5.886 and v_max=20,
against the car's scan_end=24 m, a_lat_max=4.0 and v_max=15, and skipped the
live function's dense-resample denoising. On a 60 m straight into a 5 m
hairpin it commanded 2.45 m/s faster on average and 5 m/s faster on
straights — so weights tuned offline were tuned for a car faster than the
one that exists.

Passes 1-2 (forward/backward propagation) have no live counterpart; the car
relies on the 24 m look-ahead to see corners early enough. They only ever
LOWER a target below what pass 0 returned, so the profile stays bounded by
what the live heuristic would command.

USED BY
-------
  gui/simulation.py    — on_release() and load_test_path() call compute_speed_profile()
                     + smooth_profile() to build path_v_profile for the simulator
                     (oracle/offline reference path only).
  tuner/offline_tuner.py — _resample_path() calls both functions to build path_v for
                     every synthetic test path; also used in scoring time bonus.
  sim/rollout_core.py  — run_core_rollout()'s use_planner=True branch calls
                     curvature_speed() each step on the live planner's centreline
                     (no oracle profile exists for a planner-built path).

DOES NOT USE
------------
  model/vehicle_physics.py, model/bicycle_model.py, controller/optimiser.py, sim/sim_track.py, tuner/performance_stats.py
"""

import numpy as np
import math

# ── Live-node parity constants ────────────────────────────────────────────
# These MUST match the defaults of curvature_speed() in
# ros2/src/fsae_planning/control/fsae_control/fsae_control/control_utils.py
# (mirrored in fsds_simulator/.../control_utils.py). They are named here so
# both curvature_speed() below and compute_speed_profile() draw from one
# place instead of repeating literals — the drift that let the oracle profile
# run 2.45 m/s faster than the car.
#
# scan_end=24.0 in particular is load-bearing: a tight hairpin (~2 m radius,
# v_target ~2.7 m/s) approached at v_max needs ~24 m to brake for at a
# realistic achieved deceleration. A shorter scan sees the corner too late,
# saturating steering and spinning out at corner entry (observed live).
CURVATURE_SPEED_V_MAX = 15.0
CURVATURE_SPEED_V_MIN = 1.5
CURVATURE_SPEED_A_LAT_MAX = 4.0
CURVATURE_SPEED_SCAN_START = 1.5
CURVATURE_SPEED_SCAN_END = 24.0
CURVATURE_SPEED_STEP = 2.0

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
    denom = (dx**2 + dy**2) ** 1.5
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
#         Match the vehicle's actuator bounds in controller/optimiser.py (u_max[1]=5.0,
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
#         conservative relative to model/vehicle_physics.py's peak mu=1.6.
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

#     Called by: gui/simulation.py (on_release, load_test_path),
#                tuner/offline_tuner.py (_resample_path)
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

# Speed profiler for the ORACLE reference path. Look-ahead curvature limit
# (matching the live planning node's curvature_speed heuristic) followed by
# forward/backward longitudinal propagation.
#
# WHY THE PROPAGATION PASSES ARE BACK
# -----------------------------------
# The look-ahead heuristic alone sets each point's speed from the maximum
# curvature in the window ahead, but never checks that consecutive targets are
# connected by an achievable longitudinal acceleration. It can therefore demand
# a speed drop the car physically cannot brake for, and `speed_rmse` then
# penalises the controller for failing to do something impossible — which the
# tuner responds to by contorting the gains. The forward/backward passes make
# the profile a trajectory the vehicle can actually follow.
#
# PARITY: pass 0 below DELEGATES to curvature_speed() — the numeric-parity
# port of the live control_utils.curvature_speed() — evaluated at every path
# point, rather than re-implementing the same heuristic with different
# defaults. Before this, the oracle profile used scan_end=14 m,
# a_lat_max=mu*g=5.886 and v_max=20, while the car runs scan_end=24 m,
# a_lat_max=4.0 and v_max=15, and skipped the live function's dense-resample
# denoising entirely. Measured on a 60 m straight into a 5 m hairpin, the
# oracle profile commanded 2.45 m/s faster on average and 5 m/s faster on
# straights than the car will ever drive — so weights tuned against it were
# tuned for a faster car than exists. Delegating makes that class of drift
# structurally impossible: there is now exactly one curvature heuristic.
def compute_speed_profile(
    path_X, path_Y,
    v_max=CURVATURE_SPEED_V_MAX,
    mu=0.6,
    g=9.81,
    a_accel_max=4.0,
    a_brake_max=-5.0,
    v_min=CURVATURE_SPEED_V_MIN,
    safety=1.0,
    scan_end=CURVATURE_SPEED_SCAN_END,
    a_lat_max=CURVATURE_SPEED_A_LAT_MAX,
):
    """
    Compute a physically achievable per-point target speed profile.

    Pass 0 — look-ahead curvature limit, delegated to curvature_speed() at
    every path point so the oracle profile and the live per-tick target come
    from the same code with the same defaults (see PARITY note above).

    Pass 1 — forward propagation. Caps how fast speed may RISE between points:
    v[i] <= sqrt(v[i-1]^2 + 2*a_accel_max*ds).

    Pass 2 — backward propagation. Caps how fast speed may FALL, i.e. requires
    each point be slow enough to brake to the next point's limit:
    v[i] <= sqrt(v[i+1]^2 + 2*|a_brake_max|*ds).

    a_accel_max / a_brake_max are deliberately conservative relative to the
    plant's true bounds (VehicleParams.max_accel=12.0,
    max_accel_brake=-9.0). Planning at the true limits would leave the
    controller no margin for combined slip or model-plant mismatch, and would
    make the passes nearly non-binding — the same rationale as planning at
    a_lat_max=4.0 against a plant whose peak mu is 1.76.

    `mu` and `g` are retained for signature compatibility with older callers
    but no longer set the lateral limit — a_lat_max does, matching the live
    node. Passing mu/g has no effect.

    NOTE: passes 1-2 have no live counterpart; the car relies on
    curvature_speed()'s 24 m look-ahead to see corners early enough to brake.
    They exist here because a per-point profile is otherwise not guaranteed to
    be connected by achievable longitudinal acceleration (measured at ~273
    m/s^2 of demanded braking before they were restored), which made
    speed_rmse penalise the controller for the impossible. They only ever
    LOWER a target, so the profile stays bounded by what the live heuristic
    would command.
    """
    path_X = np.asarray(path_X)
    path_Y = np.asarray(path_Y)
    n = len(path_X)
    v_profile = np.full(n, float(v_max))

    if n < 3:
        return v_profile

    # Pre-compute segment lengths for the propagation passes below.
    pts = np.column_stack([path_X, path_Y])
    segs = np.linalg.norm(np.diff(pts, axis=0), axis=1)

    # ── Pass 0: look-ahead curvature limit (delegated) ─────────────────────
    # Evaluate the LIVE heuristic at every path point, scanning forward from
    # that point exactly as the car does from its current position each tick.
    # pts[i:] is "the path ahead of point i", which is the same argument shape
    # rollout_core passes when it calls curvature_speed() on the planner's
    # centreline — so oracle and planner branches now agree by construction.
    for i in range(n):
        v_profile[i] = curvature_speed(
            pts[i:],
            v_max=v_max, v_min=v_min, a_lat_max=a_lat_max,
            scan_start=CURVATURE_SPEED_SCAN_START,
            scan_end=scan_end, step=CURVATURE_SPEED_STEP,
            safety=safety,
        )

    # ── Pass 1: forward propagation (acceleration limit) ──────────────────
    # v_next^2 = v_prev^2 + 2*a*ds  ->  cap how fast the target may rise.
    # `segs[i-1]` is the arc length from point i-1 to point i.
    for i in range(1, n):
        v_allowed = math.sqrt(v_profile[i - 1] ** 2 + 2.0 * a_accel_max * segs[i - 1])
        if v_allowed < v_profile[i]:
            v_profile[i] = v_allowed

    # ── Pass 2: backward propagation (braking limit) ──────────────────────
    # Each point must already be slow enough to brake down to the next point's
    # limit. a_brake_max is negative, so the radicand can go below zero when
    # the demanded drop exceeds what ds allows — clamp at 0 and let v_min
    # restore the floor afterwards.
    a_brake_mag = abs(a_brake_max)
    for i in range(n - 2, -1, -1):
        radicand = v_profile[i + 1] ** 2 + 2.0 * a_brake_mag * segs[i]
        v_allowed = math.sqrt(max(radicand, 0.0))
        if v_allowed < v_profile[i]:
            v_profile[i] = v_allowed

    # The propagation passes can push points below v_min (e.g. approaching a
    # hairpin whose corner speed is already at the floor); restore the floor
    # so the profile never demands a stop the curvature limit didn't ask for.
    np.clip(v_profile, v_min, None, out=v_profile)

    return v_profile


def tracking_error_speed_gate(e_y, e_psi,
                              ey_lo=0.5, ey_hi=2.0,
                              epsi_lo=math.radians(20.0), epsi_hi=math.radians(60.0),
                              floor=0.3):
    """
    Multiplier in [floor, 1] that scales the speed target down when the car is
    failing to track the path.

    OFFLINE MIRROR of fsae_control.control_utils.tracking_error_speed_gate —
    keep the two numerically identical (same defaults, same shape), or the
    tuner optimises against a different longitudinal policy than the car runs.

    WHY THIS EXISTS
    ---------------
    curvature_speed() looks only at the SHAPE of the path ahead.  It has no
    idea where the car actually is relative to it, so it will happily command
    full speed down a straight while the car is 3 m off-line and pointing 60
    deg the wrong way.  That is exactly how the lap-2 failure in
    mpc_standalone_control_1785980686.csv became unrecoverable: with |e_y| >
    1.5 m the mean commanded speed was still 7.3 m/s (peaking at 15.0), and at
    t=85.5-86.6 s the car held full 25 deg lock while being told to ACCELERATE
    from 4.8 to 9.3 m/s.  Steering was saturated 49.9% of that lap — no
    steering command can recover a corner the car is being driven faster into.

    Slowing down is the only remaining control authority once steering
    saturates, so the speed target must respond to tracking error, not just to
    path geometry.

    SHAPE
    -----
    Linear ramp on each of |e_y| and |e_psi|, taking the WORST (minimum) of the
    two — being badly misaligned is dangerous even when laterally close, and
    vice versa.  Below *_lo the gate is exactly 1.0, so normal driving is
    completely unaffected (measured on the clean middle of that same log: gate
    < 0.99 on only 1.6% of ticks, mean 0.998).  Above *_hi it saturates at
    `floor` rather than 0 — the car still needs enough speed to steer and to
    make progress back to the path; cutting to a standstill mid-track is its
    own failure mode.
    """
    if ey_hi <= ey_lo or epsi_hi <= epsi_lo:
        return 1.0
    gy = 1.0 - (abs(float(e_y)) - ey_lo) / (ey_hi - ey_lo)
    gp = 1.0 - (abs(float(e_psi)) - epsi_lo) / (epsi_hi - epsi_lo)
    return float(np.clip(min(gy, gp), floor, 1.0))


def curvature_speed(waypoints, v_max=15.0, v_min=1.5, a_lat_max=4.0,
                    scan_start=1.5, scan_end=24.0, step=2.0, safety=1.0):
    """
    Curvature-limited target speed over the next scan_end metres of the path.

    On-demand single-scalar counterpart to compute_speed_profile(), used by the
    live planner branch (a real cone-vision planner has no oracle path to
    pre-profile, so the target speed is recomputed each tick from whatever
    centreline is currently available). This is a numeric-parity port of
    fsds_simulator/control/fsae_control/fsae_control/control_utils.py's
    curvature_speed() (the live ROS node's own re-implementation, used so
    that node has no cross-package import) — so these defaults must match
    that function's bit-for-bit for tuned weights to transfer.

    This is now the single curvature heuristic in this module:
    compute_speed_profile() calls it per path point for its pass 0 rather
    than keeping a second copy. Previously the two diverged (this one at
    a_lat_max=4.0 / scan_end=24, the profile at mu*g=5.886 / scan_end=14),
    which made the oracle reference systematically faster than anything the
    car would drive. Defaults here are mirrored in the module-level
    CURVATURE_SPEED_* constants, which compute_speed_profile() uses; change
    both together, and only alongside the matching live-node change.

    scan_end=24 m (was 14 m) matches the live boundary._WALL_PLAN_HORIZON: a
    tight hairpin (~2 m radius, v_target ~2.7 m/s) approached at v_max needs
    ~24 m to brake for at a realistic achieved deceleration, not the raw
    a_max_brake limit — a shorter scan sees the corner too late, saturating
    steering and spinning out at corner entry (observed on the live stack).

    v_target = safety*sqrt(a_lat_max / kappa_peak), with a short-path cap that
    scales v_max down when the visible path is shorter than scan_end.

    waypoints[0] is assumed to be the car's current position.
    """
    n = len(waypoints)
    if n < 3:
        return float(v_max)

    segs  = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    arc   = np.concatenate([[0.0], np.cumsum(segs)])
    total = arc[-1]

    v_max_eff = max(v_min, v_max * min(1.0, total / scan_end))
    if total < scan_start + step:
        return float(v_max_eff)

    hi = min(scan_end, total)
    # Densely resample + moving-average denoise before measuring curvature, so
    # per-point replanning jitter on straights doesn't read as spurious curvature.
    pts = None
    dense = np.arange(scan_start, hi, 1.0)
    if len(dense) >= 7:
        dx = np.interp(dense, arc, waypoints[:, 0])
        dy = np.interp(dense, arc, waypoints[:, 1])
        w  = min(5, len(dense) - 4)
        ker = np.ones(w) / w
        sx = np.convolve(dx, ker, mode='valid')
        sy = np.convolve(dy, ker, mode='valid')
        pts = np.column_stack([sx, sy])[::2]
    if pts is None or len(pts) < 3:
        sample_arcs = np.arange(scan_start, hi, step)
        if len(sample_arcs) < 3:
            return float(v_max_eff)
        sx  = np.interp(sample_arcs, arc, waypoints[:, 0])
        sy  = np.interp(sample_arcs, arc, waypoints[:, 1])
        pts = np.column_stack([sx, sy])

    # Collect every triple's Menger curvature, then reduce.  Taking the raw MAX
    # (the original behaviour) lets a SINGLE bad triple set the speed for the
    # whole scan window, and the planner does emit those: measured on
    # mpc_standalone_path_1785980686.csv, peak kappa across consecutive
    # ~1 s snapshots of the same physical corner swung R=5.3 m -> 32.3 m ->
    # 4.2 m -> 1.0 m, with a lap-2 worst case of R=0.14 m — not a real corner,
    # a centreline-fit artifact.  Since v = sqrt(a_lat/kappa), that made
    # v_target swing 4.7 -> 16.7 -> 6.0 m/s frame to frame.
    kappas = []
    for i in range(1, len(pts) - 1):
        p1, p2, p3 = pts[i - 1], pts[i], pts[i + 1]
        d12 = float(np.linalg.norm(p2 - p1))
        d23 = float(np.linalg.norm(p3 - p2))
        d31 = float(np.linalg.norm(p1 - p3))
        denom = d12 * d23 * d31
        if denom < 1e-9:
            continue
        v1    = p2 - p1
        v2    = p3 - p1
        cross = abs(float(v1[0] * v2[1] - v1[1] * v2[0]))
        kappas.append(2.0 * cross / denom)

    if not kappas:
        return float(v_max_eff)

    k = np.asarray(kappas, dtype=float)
    # A genuine corner spans several consecutive triples, so it survives a
    # short running mean; an isolated fit artifact is averaged down.  Take the
    # max of the smoothed series rather than a percentile of the raw one:
    # the scan window only yields ~7 triples, so a p75/p90 is both noisy AND
    # biased upward (measured: p75 pushed v_target to 20 m/s on paths where
    # the raw max said 5 — dangerously fast, the wrong direction to err).
    # This keeps a sustained bend fully authoritative while refusing to let one
    # point dominate.  Still a MAX over the smoothed series, so it stays
    # conservative.
    if len(k) >= 3:
        k = np.convolve(k, np.ones(3) / 3.0, mode='valid')
    max_kappa = float(k.max())

    if max_kappa < 1e-4:
        return float(v_max_eff)

    v_target = safety * math.sqrt(a_lat_max / max_kappa)
    return float(max(v_min, min(v_max_eff, v_target)))


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

    Called by: gui/simulation.py (on_release, load_test_path),
               tuner/offline_tuner.py (_resample_path)
    """
    if window < 2 or len(v_profile) < window:
        return v_profile   # Too short to smooth; return unchanged
    kernel = np.ones(window) / window   # Uniform (box) average kernel
    # Pad to preserve array length: pad by window//2 on each side with edge values
    padded   = np.pad(v_profile, (window // 2, window // 2), mode="edge")
    smoothed = np.convolve(padded, kernel, mode="valid")
    return smoothed[:len(v_profile)]   # Trim to original length (handles odd/even window)