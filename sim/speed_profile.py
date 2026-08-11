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
#
# scan_start=0.0: scan_start plus the moving-average's own centring offset
# ((w-1)/2 * dense_step) together set how far ahead of the car curvature
# actually starts being measured. A nonzero scan_start combined with the
# centring offset can push that effective start several metres ahead of the
# car -- a short, tight corner whose curvature is only sustained over a few
# metres of arc can then sit entirely inside that dead zone, with the window
# striding clean over the apex at every query, so curvature_speed() never
# sees it and the target speed climbs monotonically through the whole corner
# instead of dipping. Starting the scan at the car's own position (0.0)
# removes that dead zone.
CURVATURE_SPEED_V_MAX = 18.0
CURVATURE_SPEED_V_MIN = 1.5
CURVATURE_SPEED_A_LAT_MAX = 4.0
CURVATURE_SPEED_SCAN_START = 0.0
CURVATURE_SPEED_SCAN_END = 24.0
CURVATURE_SPEED_STEP = 2.0

# Planning-level braking capability (m/s^2, positive magnitude) used by
# curvature_speed()'s braking-distance propagation. Deliberately well below the
# plant's true limit (VehicleParams.max_accel_brake = -9.0): the target must be
# achievable with the tyres ALSO generating the lateral force for the corner
# being braked into (friction circle), and with margin for model error and
# actuation lag. Planning at the true limit would make the propagation
# non-binding exactly when it matters most.
# MUST match control_utils.A_BRAKE_PLAN on the live car.
A_BRAKE_PLAN = 5.0

# ── optimal_lap_time() limits — PHYSICAL, not planning ────────────────────
# These are deliberately NOT the CURVATURE_SPEED_* planning values above.
# Those are conservative on purpose (margin for combined slip, model error and
# actuation lag), so using them for a "fastest physically possible" reference
# produces a bound the car routinely BEATS — measured, 7 of 10 paths saturated
# the time term at exactly 1.0, destroying all discrimination in the primary
# objective. A lower bound must use what the vehicle can actually do.
#
# a_lat: the plant's peak grip is mu=1.76 (vehicle_physics.py: 1.6 * GRIP_SCALE
# 1.1), i.e. ~17 m/s^2. Using the full figure would assume the tyres generate
# peak lateral force with zero longitudinal demand, everywhere at once, which
# even an ideal driver cannot sustain through corner entry/exit. 12.0 (~0.7g of
# the available 1.76g) is a defensible steady-state cornering figure that stays
# genuinely unreachable in practice while not being absurdly optimistic.
# a_long: VehicleParams max_accel 12.0 / max_accel_brake -9.0.
OPTIMAL_LAP_V_MAX = 20.0        # PLANNER_V_MAX — the fastest the stack commands
OPTIMAL_LAP_A_LAT_MAX = 12.0
OPTIMAL_LAP_A_ACCEL = 12.0
OPTIMAL_LAP_A_BRAKE = -9.0

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
    a_accel_max=7.0,
    a_brake_max=-5.0,
    v_min=CURVATURE_SPEED_V_MIN,
    safety=1.0,
    scan_end=CURVATURE_SPEED_SCAN_END,
    a_lat_max=CURVATURE_SPEED_A_LAT_MAX,
    closed_loop=True,
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

    closed_loop : if True (default), the path is treated as a lap — point
    n-1 is adjacent to point 0, so passes 1-2 wrap with a closing segment and
    run twice each direction to let a constraint that crosses the seam
    propagate all the way around (same structure as
    tuner/raceline_optimizer._lap_time_and_speed). Without this, point 0 got
    no braking obligation from the fast point that precedes it at the end of
    the array, and point n-1 got no acceleration credit from the point that
    follows it at the start — a discontinuity the car met once per lap at
    the start/finish line, forcing an unnecessary slowdown there. Set False
    for a genuinely open, point-to-point path (e.g. an acceleration event
    that ends at a stop, not a lap).

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
    if closed_loop:
        # n entries: segs[-1] is the closing segment from point n-1 back to
        # point 0 (mirrors raceline_optimizer._lap_time_and_speed's segs).
        segs = np.hypot(np.diff(pts[:, 0], append=pts[0, 0]),
                        np.diff(pts[:, 1], append=pts[0, 1]))
    else:
        segs = np.linalg.norm(np.diff(pts, axis=0), axis=1)

    # ── Pass 0: look-ahead curvature limit (delegated) ─────────────────────
    # Evaluate the LIVE heuristic at every path point, scanning forward from
    # that point exactly as the car does from its current position each tick.
    # pts[i:] is "the path ahead of point i", which is the same argument shape
    # rollout_core passes when it calls curvature_speed() on the planner's
    # centreline — so oracle and planner branches now agree by construction.
    # On a closed loop, wrap the scan window past the end back to the start
    # so a point near the finish line still sees the corner just after the
    # start line, instead of curvature_speed() truncating its look-ahead.
    for i in range(n):
        ahead = pts[i:]
        if closed_loop and len(ahead) < n:
            wrap_needed = math.ceil(scan_end / max(np.min(segs), 1e-6)) + 1
            ahead = np.concatenate([ahead, pts[:min(n, wrap_needed)]], axis=0)
        v_profile[i] = curvature_speed(
            ahead,
            v_max=v_max, v_min=v_min, a_lat_max=a_lat_max,
            scan_start=CURVATURE_SPEED_SCAN_START,
            scan_end=scan_end, step=CURVATURE_SPEED_STEP,
            safety=safety,
        )

    a_brake_mag = abs(a_brake_max)

    if closed_loop:
        # Two wrapped laps per direction: the first propagates a constraint
        # from its binding station all the way around; the second carries
        # whatever crossed the start/finish seam back around again. A third
        # changes nothing (monotone min-update over fixed corner limits).
        for _lap in range(2):
            # ── Pass 1: forward propagation (acceleration limit) ──────────
            for i in range(n):
                j = (i - 1) % n
                v_allowed = math.sqrt(v_profile[j] ** 2 + 2.0 * a_accel_max * segs[j])
                if v_allowed < v_profile[i]:
                    v_profile[i] = v_allowed
            # ── Pass 2: backward propagation (braking limit) ──────────────
            for i in range(n - 1, -1, -1):
                j = (i + 1) % n
                radicand = v_profile[j] ** 2 + 2.0 * a_brake_mag * segs[i]
                v_allowed = math.sqrt(max(radicand, 0.0))
                if v_allowed < v_profile[i]:
                    v_profile[i] = v_allowed
    else:
        # ── Pass 1: forward propagation (acceleration limit) ──────────────
        # v_next^2 = v_prev^2 + 2*a*ds  ->  cap how fast the target may rise.
        # `segs[i-1]` is the arc length from point i-1 to point i.
        for i in range(1, n):
            v_allowed = math.sqrt(v_profile[i - 1] ** 2 + 2.0 * a_accel_max * segs[i - 1])
            if v_allowed < v_profile[i]:
                v_profile[i] = v_allowed

        # ── Pass 2: backward propagation (braking limit) ───────────────────
        # Each point must already be slow enough to brake down to the next
        # point's limit. a_brake_max is negative, so the radicand can go
        # below zero when the demanded drop exceeds what ds allows — clamp at
        # 0 and let v_min restore the floor afterwards.
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
    full speed down a straight while the car is badly off-line and pointing
    the wrong way — and once steering saturates trying to correct that, no
    steering command can recover a corner the car is being driven faster
    into.

    Slowing down is the only remaining control authority once steering
    saturates, so the speed target must respond to tracking error, not just to
    path geometry.

    SHAPE
    -----
    Linear ramp on each of |e_y| and |e_psi|, taking the WORST (minimum) of the
    two — being badly misaligned is dangerous even when laterally close, and
    vice versa.  Below *_lo the gate is exactly 1.0, so normal driving is
    completely unaffected.  Above *_hi it saturates at `floor` rather than 0 —
    the car still needs enough speed to steer and to make progress back to
    the path; cutting to a standstill mid-track is its own failure mode.
    """
    if ey_hi <= ey_lo or epsi_hi <= epsi_lo:
        return 1.0
    gy = 1.0 - (abs(float(e_y)) - ey_lo) / (ey_hi - ey_lo)
    gp = 1.0 - (abs(float(e_psi)) - epsi_lo) / (epsi_hi - epsi_lo)
    return float(np.clip(min(gy, gp), floor, 1.0))


def curvature_speed(waypoints, v_max=15.0, v_min=1.5, a_lat_max=4.0,
                    scan_start=0.0, scan_end=24.0, step=2.0, safety=1.0):
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

    v_target = safety*sqrt(a_lat_max / kappa_peak), propagated for braking
    distance. A short-path cap (scales v_max down when the visible path is
    shorter than scan_end) applies ONLY when there isn't enough path to
    measure curvature at all -- once curvature has been measured, it is not
    reapplied on top of v_target (see the comment above the final return).

    scan_start=0.0 (see the APEX BLIND SPOT note below): curvature
    measurement starts at the car's own position rather than some distance
    ahead of it, closing a dead zone that let short, tight corners go
    entirely unmeasured.

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
    # pts_s0 is the arc distance AHEAD OF THE CAR of pts[0].  It is not simply
    # scan_start in the dense branch: a width-w 'valid' moving average places its
    # first output at the CENTRE of the first window, i.e. (w-1)/2 * dense_step
    # further along.  Carrying it explicitly is what keeps the braking propagation
    # below honest — deriving distances from pts alone silently loses this offset.
    #
    # APEX BLIND SPOT: a nonzero scan_start combined with a coarse dense_step
    # and wide smoothing window pushes pts_s0 (the effective start of
    # curvature measurement) several metres ahead of the car. A short, tight
    # corner whose curvature is only sustained over a few metres of arc can
    # then be entirely inside that dead zone on every single query as the car
    # approaches — the window strides clean over the apex and
    # curvature_speed() reports a monotonically RISING target through the
    # whole corner instead of dipping near the true minimum. Coarse
    # decimation of the sample points makes this worse by reducing the number
    # of samples that could land inside a short apex zone.
    #
    # Fix: scan from the car's own position (scan_start=0.0), sample finely
    # (dense_step=0.5) so a short apex zone still gets several samples, use a
    # narrower smoothing window (w=3) so the apex isn't averaged away against
    # its shallower neighbours, and use the smoothed points directly as the
    # triples rather than decimating them, so no samples are thrown away in
    # exactly the region that needed more of them. This closes the apex gap
    # without introducing spurious low-speed dips on straight sections.
    pts = None
    pts_s0 = scan_start
    dense_step = 0.5
    dense = np.arange(scan_start, hi, dense_step)
    if len(dense) >= 7:                       # room to smooth and still leave >=3 triples
        dx = np.interp(dense, arc, waypoints[:, 0])
        dy = np.interp(dense, arc, waypoints[:, 1])
        w  = min(3, len(dense) - 4)           # 'valid' conv keeps len-w+1 >= 3 points
        ker = np.ones(w) / w
        sx = np.convolve(dx, ker, mode='valid')
        sy = np.convolve(dy, ker, mode='valid')
        pts = np.column_stack([sx, sy])       # no decimation -- keep every smoothed sample
        pts_s0 = scan_start + (w - 1) / 2.0 * dense_step
    if pts is None or len(pts) < 3:
        # Short scan window: no headroom to denoise — fall back to coarse sampling.
        sample_arcs = np.arange(scan_start, hi, step)
        if len(sample_arcs) < 3:
            return float(v_max_eff)
        sx  = np.interp(sample_arcs, arc, waypoints[:, 0])
        sy  = np.interp(sample_arcs, arc, waypoints[:, 1])
        pts = np.column_stack([sx, sy])
        pts_s0 = scan_start

    # Collect every triple's Menger curvature, then reduce.  Taking the raw MAX
    # (the original behaviour) lets a SINGLE bad triple set the speed for the
    # whole scan window, and the planner does emit those: measured on
    # mpc_standalone_path_1785980686.csv, peak kappa across consecutive
    # ~1 s snapshots of the same physical corner swung R=5.3 m -> 32.3 m ->
    # 4.2 m -> 1.0 m, with a lap-2 worst case of R=0.14 m — not a real corner,
    # a centreline-fit artifact.  Since v = sqrt(a_lat/kappa), that made
    # v_target swing 4.7 -> 16.7 -> 6.0 m/s frame to frame.
    # kappa_at[j] records which pts index kappas[j] is centred on.  A degenerate
    # triple is skipped, so the two lists would otherwise drift out of step and
    # every later curvature would be attributed to the wrong distance.
    kappas = []
    kappa_at = []
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
        kappa_at.append(i)

    if not kappas:
        return float(v_max_eff)

    k = np.asarray(kappas, dtype=float)
    k_at = np.asarray(kappa_at, dtype=int)
    # A genuine corner spans several consecutive triples, so it survives a
    # short running mean; an isolated fit artifact is averaged down.  Reduce
    # with a max over the SMOOTHED series rather than a percentile of the raw
    # one: the scan window only yields ~7 triples, so a p75/p90 is both noisy
    # AND biased upward (measured: p75 pushed v_target to 20 m/s on paths where
    # the raw max said 5 — dangerously fast, the wrong direction to err).
    # This keeps a sustained bend fully authoritative while refusing to let one
    # point dominate.
    # The 'valid' 3-point mean drops one entry at each end, so the surviving
    # centres are k_at[1:-1] — track them alongside k rather than assuming a
    # fixed offset, which breaks whenever this branch does not run.
    if len(k) >= 3:
        k = np.convolve(k, np.ones(3) / 3.0, mode='valid')
        k_at = k_at[1:len(k) + 1]

    # ── Braking-distance propagation ──────────────────────────────────────
    # Taking a single max over the window and returning sqrt(a_lat/kappa)
    # ignores WHERE the corner is: a hairpin 24 m ahead and the same hairpin
    # 2 m ahead produce an identical target, so the profile can demand a
    # deceleration the car cannot physically produce.  Measured offline on a
    # 60 m straight into a 5 m hairpin, the equivalent single-max profile
    # implied ~273 m/s^2 (~28 g) of braking at corner entry.  The car can't do
    # that, so the speed error is charged to the controller for failing at
    # something impossible, and it arrives at the corner too fast regardless.
    # scan_end=24 m was sized so a hairpin is *seen* in time; this makes the
    # target actually *achievable* from here.
    #
    # For each sampled corner at distance d ahead with its own corner-speed
    # limit v_corner, the fastest we may travel NOW and still brake to it is
    #     v_allowed = sqrt(v_corner^2 + 2 * a_brake * d)
    # (from v_f^2 = v_i^2 - 2*a*d).  Take the most restrictive over the window.
    # A corner far enough away imposes no limit, which falls out naturally
    # because v_allowed then exceeds v_max_eff.
    #
    # Distances: each surviving entry k[j] is centred on pts[k_at[j]], whose
    # distance ahead of the car is pts_s0 (where pts[0] actually sits, INCLUDING
    # the moving-average centre shift) plus the arc length along pts to there.
    # A previous version assumed a fixed "+2" index offset instead, which was
    # wrong in both directions depending on which sampling branch ran: -2 m in
    # the common smoothed case and +2 m in the coarse short-path case.
    #
    # CORNER_ENTRY_MARGIN: a curvature sample describes the middle of a bend, but
    # the car must already be AT corner speed by the bend's ENTRY, which is
    # earlier.  Braking to the sample's own distance is therefore too late.  The
    # old "-2 m" index error happened to supply this margin by accident; making
    # it explicit keeps the distance honest and the conservatism deliberate.
    # Sized at one triple half-width (the arc a single curvature sample spans),
    # so it scales with the sampling density instead of being a magic number.
    k_safe = np.maximum(k, 1e-9)
    v_corner = safety * np.sqrt(a_lat_max / k_safe)
    if len(pts) > 1:
        pts_arc = np.concatenate(
            [[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
        )
        entry_margin = float(np.median(np.diff(pts_arc))) if len(pts_arc) > 1 else 0.0
        d_ahead = (pts_s0
                   + pts_arc[np.clip(k_at, 0, len(pts_arc) - 1)]
                   - entry_margin)
        d_ahead = np.maximum(d_ahead, 0.0)
    else:
        d_ahead = np.full(len(k), pts_s0)

    v_allowed = np.sqrt(v_corner ** 2 + 2.0 * A_BRAKE_PLAN * d_ahead)
    v_target = float(np.min(v_allowed))

    # v_max_eff (the SHORT-PATH-scaled ceiling) is NOT reapplied here -- it
    # exists purely for the two early-return "not enough path to measure
    # curvature at all" cases above, and reapplying it here would only ever
    # make a validly-measured tight corner's target MORE restrictive using a
    # cruder signal than v_target already used.
    #
    # v_max ITSELF is re-clamped here, which is a distinct question from
    # v_max_eff. On a straight approach, before a corner enters the scan
    # window, kappa is measured near zero, so v_corner =
    # safety*sqrt(a_lat_max/kappa) and therefore v_target can be arbitrarily
    # large -- there is nothing else in this function that bounds the upper
    # end once curvature has been measured at all. A target above the car's
    # own configured top speed eats into the braking-distance margin
    # curvature_speed()'s whole design (A_BRAKE_PLAN, scan_end) assumes is
    # available -- the car arrives at the point curvature is finally
    # measured already faster than intended, not just later than intended.
    return float(np.clip(v_target, v_min, v_max))


def optimal_lap_time(path_X, path_Y, v_max=None, a_lat_max=None,
                     a_accel_max=None, a_brake_max=None, v_min=None,
                     v_start=0.0):
    """
    Quasi-steady-state minimum traversal time for a path (seconds).

    WHAT THIS IS FOR
    ----------------
    A physically-grounded reference lap time, so `time_bonus` can measure
    "how close to the fastest this car could physically go" instead of being
    anchored to a placeholder constant. Previously the time baseline was
    `arc_length / 2.5 m/s * 1.5` (see sim_track.calculate_dynamic_max_steps),
    an arbitrary figure with no physical meaning — which made
    TIME_BONUS_WEIGHT (0.25, the second-largest score term) a reward measured
    against nothing in particular.

    METHOD
    ------
    The standard quasi-steady-state lap simulation used by racing-line tools:

      1. Corner limit at every point from the friction circle,
         v = sqrt(a_lat_max / |kappa|)
      2. Forward pass  — cap how fast speed may RISE  (a_accel_max)
      3. Backward pass — cap how fast speed may FALL  (a_brake_max)
      4. Integrate t = sum(ds / v_avg) over the resulting profile

    This is the same three-pass algorithm compute_speed_profile() uses, but
    applied to the TRUE per-point curvature rather than a forward-looking
    scan, and without the look-ahead/denoising machinery that exists to cope
    with a noisy live centreline. On a known offline path there is no noise to
    reject, so using true curvature gives a genuine physical bound.

    "Quasi-steady-state" means it assumes the car is always exactly at some
    limit (cornering, accelerating or braking) and ignores transient dynamics
    — load transfer, tyre relaxation, yaw inertia. Real achievable time is
    therefore somewhat SLOWER than this. That is the right direction for a
    reference: it is a lower bound the controller approaches but should not
    beat, so `time_bonus` stays in [0, 1] under normal operation.

    NOT a racing line. This is the fastest traversal of the GIVEN path
    (the centreline), not the fastest line through the track. A real racing
    line cuts corners and would be faster still. Computing that needs a proper
    minimum-time trajectory optimiser (e.g. TUM's
    global_racetrajectory_optimization) and the track boundaries, not just the
    centreline.

    Parameters
    ----------
    path_X, path_Y : array-like, shape (n,)
        Path coordinates (m).
    v_max : float, optional
        Speed cap (m/s). Defaults to OPTIMAL_LAP_V_MAX.
    a_lat_max : float, optional
        Lateral grip limit (m/s^2). Defaults to OPTIMAL_LAP_A_LAT_MAX — the
        PHYSICAL grip, deliberately not the conservative planning value. Using
        the planning limit here produced a "bound" the car routinely beat (7 of
        10 paths saturated the time term at exactly 1.0), which is not a bound
        at all. See the OPTIMAL_LAP_* constants for the reasoning.
    a_accel_max : float
        Longitudinal acceleration limit (m/s^2, positive).
    a_brake_max : float
        Longitudinal braking limit (m/s^2, negative).
    v_min : float, optional
        Speed floor (m/s), applied after the passes.
    v_start : float
        Speed at the first point (m/s). Default 0.0 — rollouts start from rest,
        so the reference must too, otherwise it is unreachable by construction.

    Returns
    -------
    float : traversal time in seconds. `inf`-safe; returns 0.0 for a
            degenerate path.
    """
    if v_max is None:
        v_max = OPTIMAL_LAP_V_MAX
    if a_lat_max is None:
        a_lat_max = OPTIMAL_LAP_A_LAT_MAX
    if v_min is None:
        v_min = 0.0
    if a_accel_max is None:
        a_accel_max = OPTIMAL_LAP_A_ACCEL
    if a_brake_max is None:
        a_brake_max = OPTIMAL_LAP_A_BRAKE

    X = np.asarray(path_X, dtype=float)
    Y = np.asarray(path_Y, dtype=float)
    n = len(X)
    if n < 2:
        return 0.0

    segs = np.hypot(np.diff(X), np.diff(Y))
    total = float(segs.sum())
    if total <= 0.0:
        return 0.0
    if n < 3:
        return total / max(v_max, 1e-9)

    # ── Pass 0: corner-speed limit from true curvature ────────────────────
    kappa = np.abs(compute_path_curvature(X, Y))
    kappa = np.maximum(kappa, 1e-9)
    v = np.minimum(np.sqrt(a_lat_max / kappa), v_max)

    # ── Pass 1: forward (acceleration-limited), anchored at v_start ───────
    v[0] = min(v[0], max(float(v_start), 0.0))
    for i in range(1, n):
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2.0 * a_accel_max * segs[i - 1]))

    # ── Pass 2: backward (braking-limited) ────────────────────────────────
    a_brake_mag = abs(a_brake_max)
    for i in range(n - 2, -1, -1):
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * a_brake_mag * segs[i]))

    # The floor must not apply to the standing start — the car genuinely is at
    # 0 m/s there, and clamping it up would fabricate free speed.
    v[1:] = np.maximum(v[1:], v_min)
    v[0] = max(v[0], 0.0)

    # ── Integrate: t = sum(ds / v_avg) over each segment ──────────────────
    v_avg = 0.5 * (v[:-1] + v[1:])
    # Guard the standing-start segment, where v_avg can be ~0.
    v_avg = np.maximum(v_avg, 1e-3)
    return float(np.sum(segs / v_avg))


def smooth_profile(v_profile, window=9, closed_loop=True):
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

    closed_loop controls the padding used at the array boundary before
    convolving: 'wrap' (default) pulls padding from the opposite end of the
    array, matching compute_speed_profile(closed_loop=True)'s treatment of
    point n-1 as adjacent to point 0 — 'edge' padding would instead replicate
    the first/last values and flatten exactly the region compute_speed_profile
    worked to connect across the start/finish seam. Set False to fall back to
    'edge' padding for a genuinely open, point-to-point path.

    Parameters
    ----------
    v_profile : np.ndarray, shape (n,)
        Raw speed profile from compute_speed_profile().
    window : int
        Moving average window width (number of path points). Default 9.
        Larger values = smoother but may flatten sharp braking zones.
    closed_loop : bool
        Pad with 'wrap' (True, default) or 'edge' (False). Should match the
        closed_loop value passed to compute_speed_profile().

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
    # Pad to preserve array length: pad by window//2 on each side.
    pad_mode = "wrap" if closed_loop else "edge"
    padded   = np.pad(v_profile, (window // 2, window // 2), mode=pad_mode)
    smoothed = np.convolve(padded, kernel, mode="valid")
    return smoothed[:len(v_profile)]   # Trim to original length (handles odd/even window)