import math

import numpy as np

# FSDS: max steering angle 25 degrees per ros-bridge.md
MAX_STEER_RAD = math.radians(25.0)

# Planning-level braking capability (m/s^2, positive magnitude) used by
# curvature_speed()'s braking-distance propagation.  Deliberately well below
# the vehicle's true limit (~9 m/s^2): the target must be achievable with the
# tyres ALSO generating the lateral force for the corner being braked into
# (friction circle), and with margin for model error and the ~0.05 s actuation
# lag.  Planning at the true limit would make the propagation non-binding
# exactly when it matters most.  Mirrors the a_brake_max used by
# fsae_MPCTest/sim/speed_profile.py's compute_speed_profile().
A_BRAKE_PLAN = 5.0


def _heading_error(car_pos, car_yaw, target_global) -> float:
    """Return heading error in radians: positive when target is left of car."""
    dx = target_global[0] - car_pos[0]
    dy = target_global[1] - car_pos[1]
    cos_y, sin_y = math.cos(car_yaw), math.sin(car_yaw)
    x_car =  dx * cos_y + dy * sin_y
    y_car = -dx * sin_y + dy * cos_y
    return math.atan2(y_car, x_car)


def compute_steering(car_pos, car_yaw, target_global) -> float:
    """Pure-proportional steering (legacy helper, kept for reference)."""
    return float(np.clip(-_heading_error(car_pos, car_yaw, target_global)
                         / MAX_STEER_RAD, -1.0, 1.0))


class StanleyController:
    """
    Stanley path-tracking steering controller (Thrun et al., DARPA 2005).

    δ = θ_e + arctan(k_cte · e / (v + k_soft))

    θ_e — heading error: path tangent angle minus car yaw (rad).
          Positive when path turns left relative to the car.
    e   — cross-track error: signed lateral distance from the front axle
          to the nearest path point (m), positive when the axle is to
          the RIGHT of the path.
    v   — car speed (m/s); k_soft prevents division by zero at standstill.

    Sign convention (FSDS ENU: x forward, y left):
      output > 0  → steer right
      output < 0  → steer left
      output ∈ [-1, 1]

    Tuning:
      k_cte     — cross-track gain.  Higher values correct lateral error faster
                  but cause oscillation on a high-speed straight.
      k_soft    — speed softening (m/s).  Set to ~walking speed so the CTE
                  term doesn't saturate the steering at low speeds.
      k_d       — yaw-rate damper gain.  Subtracts k_d·ω from the Stanley
                  angle before normalising, opposing rapid heading changes.
                  This is the primary fix for left-right sway: the CTE term
                  alone has no memory of how fast the heading is already
                  changing, so it overshoots; k_d·ω counters each swing.
      wheelbase — distance from rear to front axle (m).  Used to project
                  the control point to the front axle, which is where Stanley
                  measures cross-track error.
    """

    def __init__(
        self,
        k_cte: float = 1.0,
        k_soft: float = 1.0,
        k_d: float = 0.1,
        wheelbase: float = 1.5,
    ):
        self.k_cte     = k_cte
        self.k_soft    = k_soft
        self.k_d       = k_d
        self.wheelbase = wheelbase

    def compute(
        self,
        path: np.ndarray,
        car_pos: np.ndarray,
        car_yaw: float,
        car_speed: float,
        car_yaw_rate: float = 0.0,
    ) -> float:
        """
        Return a steering command in [-1, 1].

        path         — (N, 2) array of waypoints in map frame (must have N ≥ 2)
        car_pos      — (2,) car position in map frame
        car_yaw      — car heading in radians
        car_speed    — car speed in m/s
        car_yaw_rate — yaw rate in rad/s (positive = left / CCW); used by the
                       damper term to oppose rapid heading changes
        """
        if len(path) < 2:
            return 0.0

        # Project control point to front axle
        fa = car_pos + self.wheelbase * np.array([math.cos(car_yaw), math.sin(car_yaw)])

        # Nearest waypoint index to front axle
        idx = int(np.argmin(np.linalg.norm(path - fa, axis=1)))

        # Unit tangent in direction of travel at that waypoint
        if idx < len(path) - 1:
            seg = path[idx + 1] - path[idx]
        else:
            seg = path[idx] - path[idx - 1]
        seg_len = float(np.linalg.norm(seg))
        if seg_len < 1e-6:
            return 0.0
        t = seg / seg_len

        # Heading error: path_yaw - car_yaw, normalised to (-π, π)
        path_yaw = math.atan2(t[1], t[0])
        theta_e = math.atan2(
            math.sin(path_yaw - car_yaw),
            math.cos(path_yaw - car_yaw),
        )

        # Cross-track error: right-normal of path, positive = axle right of path
        right_n = np.array([t[1], -t[0]])   # 90° CW rotation of tangent
        e = float(np.dot(fa - path[idx], right_n))

        # Stanley angle — positive = left turn (standard convention).  Damper
        # subtracts k_d·ω: when the car is already swinging left (ω > 0), this
        # reduces δ so the next tick steers less left, preventing overshoot.
        delta = (theta_e
                 + math.atan2(self.k_cte * e, car_speed + self.k_soft)
                 - self.k_d * car_yaw_rate)

        # Return the steering ANGLE in radians (positive = left), clamped to the
        # physical limit.  FSDS normalisation (+1 = right) is done by fsds_bridge.
        return float(np.clip(delta, -MAX_STEER_RAD, MAX_STEER_RAD))


def tracking_error_speed_gate(e_y, e_psi,
                              ey_lo=0.5, ey_hi=2.0,
                              epsi_lo=math.radians(20.0), epsi_hi=math.radians(60.0),
                              floor=0.3):
    """
    Multiplier in [floor, 1] that scales the speed target down when the car is
    failing to track the path.

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

    Parameters
    ----------
    e_y : float     Lateral tracking error (m).   Sign ignored.
    e_psi : float   Heading tracking error (rad). Sign ignored.
    ey_lo/ey_hi, epsi_lo/epsi_hi : float
        Ramp start/end. At/below _lo -> 1.0; at/above _hi -> floor.
    floor : float   Minimum multiplier.

    Returns
    -------
    float in [floor, 1.0]
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

    Ported from the planner's speed logic so the controller can set cmd_vel.speed
    without a cross-package import.  v_target = safety·√(a_lat_max / κ_peak),
    propagated for braking distance.  A short-path cap (scales v_max down when
    the visible path is shorter than scan_end) applies ONLY when there isn't
    enough path to measure curvature at all -- once curvature has been
    measured, it is not reapplied on top of v_target (see the comment above
    the final return).  waypoints[0] is assumed to be the car's current
    position.

    scan_end=24 m (was 14 m): a tight hairpin (~2 m radius, v_target ~2.7 m/s)
    approached at v_max needs ~24 m to brake for at a realistic achieved
    deceleration, not the raw a_max_brake limit. A shorter scan sees the
    corner too late for the car to shed enough speed, saturating steering and
    spinning out at corner entry (observed on the live fsae_planning stack;
    kept in sync with the planner's own lookahead there).

    The returned target also accounts for BRAKING DISTANCE (see the propagation
    block below): each corner in the scan window is converted to the fastest
    speed from which that corner is still reachable at A_BRAKE_PLAN, and the
    most restrictive wins. Without that, a corner 24 m ahead and the same corner
    2 m ahead give the same answer, so the profile can request a deceleration
    the car cannot produce. scan_end makes a corner VISIBLE in time; this makes
    the resulting target ACHIEVABLE.

    scan_start=0.0 (was 1.5, fixed 2026-08-08 -- see fsae_MPCTest's
    sim_to_real_investigation.md §50 and the APEX BLIND SPOT note in the dense
    resample block below): curvature measurement now starts at the car's own
    position rather than 1.5 m ahead of it, closing a dead zone that let
    short, tight corners go entirely unmeasured.
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
    # The planner re-fits the centreline every frame, so on a straight the
    # published path carries a few cm of per-point lateral wiggle.  Computing the
    # MAX Menger curvature over raw ~2 m triples turns that noise into a large
    # spurious kappa (true kappa is ~0 on a straight, so the max is pure noise),
    # collapsing v_target and making it oscillate frame-to-frame — the "rapid
    # accel/decel" on straights.  Fix at the source: densely resample the scan
    # window and moving-average denoise it before measuring curvature.  A real
    # corner is a sustained bend that survives the smoothing; only the cm-scale
    # wiggle is removed (this also stops noise from over-slowing corners).
    #
    # pts_s0 is the arc distance AHEAD OF THE CAR of pts[0].  It is not simply
    # scan_start in the dense branch: a width-w 'valid' moving average places its
    # first output at the CENTRE of the first window, i.e. (w-1)/2 * dense_step
    # further along.  Carrying it explicitly is what keeps the braking propagation
    # below honest — deriving distances from pts alone silently loses this offset.
    #
    # APEX BLIND SPOT (fixed 2026-08-08, see fsae_MPCTest's
    # sim_to_real_investigation.md §50): scan_start=1.5 combined with the OLD
    # dense_step=1.0/w=5 pushed pts_s0 (the effective start of curvature
    # measurement) to scan_start + (w-1)/2*dense_step = 1.5 + 2.0 = 3.5 m ahead
    # of the car. A short, tight corner whose curvature is only sustained over
    # ~2-3 m of arc (measured on a recorded map: true tightest R≈5.7 m over a
    # ~2-3 m apex zone) can be entirely inside that 3.5 m dead zone on every
    # single query as the car approaches — the window strides clean over the
    # apex and curvature_speed() reports a monotonically RISING target through
    # the whole corner instead of dipping near the true minimum. The old
    # [::2] decimation back to ~2 m triple spacing made this worse by halving
    # the number of samples that could land inside a short apex zone.
    #
    # Fix: scan_start 1.5->0.0 (measure from the car's own position), dense_step
    # 1.0->0.5 (finer sampling so a 2-3 m apex zone still gets several samples),
    # w 5->3 (a narrower smoothing window so the apex isn't averaged away against
    # its shallower neighbours), and the [::2] decimation REMOVED (the smoothed
    # points are used directly as the triples, so no samples are thrown away in
    # exactly the region that needed more of them). Full-track validation on
    # the offline mirror (fsae_MPCTest/sim/speed_profile.py, numeric-parity
    # counterpart of this function) found this closes the apex gap at every
    # corner on a recorded map with zero regressions on straights — see
    # sim_to_real_investigation.md §50.
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

    # v_max_eff (the SHORT-PATH-scaled ceiling) is NOT reapplied here -- that
    # part of the S33/S34 reasoning still holds: it exists purely for the two
    # early-return "not enough path to measure curvature at all" cases above,
    # and reapplying it here would only ever make a validly-measured tight
    # corner's target MORE restrictive using a cruder signal than v_target
    # already used.
    #
    # v_max ITSELF is re-clamped here (added 2026-08-08, S45) -- this was
    # missed by the S33/S34 fix and is a distinct question from v_max_eff.
    # On a straight approach, before a corner enters the scan window, kappa
    # is measured near zero, so v_corner = safety*sqrt(a_lat_max/kappa) and
    # therefore v_target can be arbitrarily large -- there is nothing else in
    # this function that bounds the upper end once curvature has been
    # measured at all. Confirmed live: mpc_standalone_control_1786140619.csv
    # shows v_desired (the FILTERED target the MPC actually reacts to) reach
    # 24.7 m/s against v_max=15.0, in 8 distinct episodes up to 3.4s long,
    # including exactly the corner-entry window of a "brakes too late" event
    # the user reported. A target above the car's own configured top speed
    # eats into the braking-distance margin curvature_speed()'s whole design
    # (A_BRAKE_PLAN, scan_end=24m) assumes is available -- the car arrives at
    # the point curvature is finally measured already faster than intended,
    # not just later than intended.
    return float(np.clip(v_target, v_min, v_max))


def _load_profile_csv(csv_path: str):
    """
    Shared reader for fsae_MPCTest's tuner/export_speed_profile.py CSVs
    (header "x,y,psi,v_target"; comment lines starting with '#' skipped).

    Deliberately does NOT depend on scipy or fsae_MPCTest's centreline
    reconstruction (sim/track_io.py, which needs CubicSpline and a
    planning/boundary.py march over the whole lap) -- that reconstruction
    runs once, offline, when the CSV is exported; this is a plain reader so
    the car's control package picks up no new dependency for it.

    Returns
    -------
    (path_X, path_Y, path_Psi, path_V) : tuple of np.ndarray, shape (n,)

    Raises
    ------
    FileNotFoundError, ValueError : bad csv_path or malformed contents.
    """
    xs, ys, psis, vs = [], [], [], []
    with open(csv_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or line.startswith('x,y'):
                continue
            x_s, y_s, psi_s, v_s = line.split(',')
            xs.append(float(x_s))
            ys.append(float(y_s))
            psis.append(float(psi_s))
            vs.append(float(v_s))
    if len(xs) < 2:
        raise ValueError(f"{csv_path}: fewer than 2 valid rows")
    return np.asarray(xs), np.asarray(ys), np.asarray(psis), np.asarray(vs)


def load_speed_profile_csv(csv_path: str):
    """
    Load a pre-computed (x, y, v_target) speed profile exported by
    fsae_MPCTest's tuner/export_speed_profile.py.

    For a track that's already been mapped, this replaces curvature_speed()'s
    per-tick re-derivation with a lookup against the oracle profile computed
    once, offline, from the WHOLE recorded map -- see
    fsae_MPCTest/docs/sim_to_real_investigation.md S48 for why: the live-built
    centreline is measurably shorter than curvature_speed()'s own assumed
    scan_end=24m on effectively every tick (perception FOV clips laterally on
    a corner before its forward window does), so the live planner is
    permanently short of the lookahead its own braking-distance design
    assumes. This bypasses that shortfall entirely for a known track, since
    it needs no live cone visibility to know the target speed.

    Parameters
    ----------
    csv_path : str   Path to a CSV with header "x,y,psi,v_target" (comment
                      lines starting with '#' are skipped).

    Returns
    -------
    (path_X, path_Y, path_V) : tuple of np.ndarray, shape (n,)

    Raises
    ------
    FileNotFoundError, ValueError : bad csv_path or malformed contents.
    """
    path_X, path_Y, _path_Psi, path_V = _load_profile_csv(csv_path)
    return path_X, path_Y, path_V


def load_path_profile_csv(csv_path: str):
    """
    Load a pre-computed (x, y) path exported by fsae_MPCTest's
    tuner/export_speed_profile.py, for use as a drop-in replacement of the
    live planner's /fsae/planning/selected_trajectory centreline.

    See the "Offline parity note for the live ROS side's path_map_path param"
    comment in fsae_MPCTest/settings.py (USE_PLANNER=False is the offline
    equivalent -- no separate flag exists there): this removes the
    live planner (centerline_planner.py / boundary.py / cone_map.py) from the
    control loop entirely for a track that's already been mapped, isolating
    controller+plant tracking error from planner-induced path error (e.g. the
    known centreline curvature-spike defect -- see
    fsae_MPCTest/docs/planning_control_sync.md).

    psi is exported but not returned here: MPCController._error_state()
    already derives path heading from consecutive waypoints (atan2 of the
    segment direction, the same convention centerline_planner.py's published
    PoseArray uses), so the (n,2) array below is a direct substitute for the
    live topic's path with no interface change to mpc_core.py.

    Parameters
    ----------
    csv_path : str   Path to a CSV with header "x,y,psi,v_target" (comment
                      lines starting with '#' are skipped).

    Returns
    -------
    path : np.ndarray, shape (n, 2)   [x, y] waypoints, global frame.

    Raises
    ------
    FileNotFoundError, ValueError : bad csv_path or malformed contents.
    """
    path_X, path_Y, _path_Psi, _path_V = _load_profile_csv(csv_path)
    return np.column_stack([path_X, path_Y])


def precomputed_speed_at(car_pos, path_X, path_Y, path_V) -> float:
    """
    Nearest-point lookup into a pre-computed speed profile (see
    load_speed_profile_csv()).

    Deliberately a plain nearest-point search, not a Frenet/arc-length
    projection: the profile is dense (see export script, default 1000 pts
    over a lap), so nearest-point error is small and this avoids needing the
    heading/tangent bookkeeping a proper Frenet projection would add for a
    speed lookup that only needs to be "close enough," unlike e_y/e_psi
    tracking error, which does need that precision.

    Parameters
    ----------
    car_pos : array-like, shape (2,)   Car's current [x, y] (global frame).
    path_X, path_Y, path_V : np.ndarray, shape (n,)   From load_speed_profile_csv().

    Returns
    -------
    float : v_target at the nearest profile point to car_pos.
    """
    d2 = (path_X - car_pos[0]) ** 2 + (path_Y - car_pos[1]) ** 2
    return float(path_V[int(np.argmin(d2))])
