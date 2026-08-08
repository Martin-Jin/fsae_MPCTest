"""
tuner/raceline_optimizer.py — Minimum-time racing line, not just a centreline
speed profile.

WHAT THIS IS
------------
sim/speed_profile.optimal_lap_time()'s own docstring says plainly: "NOT a
racing line. This is the fastest traversal of the GIVEN path (the
centreline)... Computing that needs a proper minimum-time trajectory
optimiser... and the track boundaries, not just the centreline." This module
is that optimiser.

It reshapes the path LATERALLY within the recorded track's boundaries (it
does not have to hug the centreline — cutting the inside of a corner and
running wide on entry/exit is exactly the point) to minimise lap time, then
re-profiles speed on the optimised path. Both new path and new speed are
exported through the existing tuner/export_speed_profile.py CSV mechanism,
so the live car needs no changes: it already just tracks whatever
(x, y, psi, v_target) rows that CSV contains.

METHOD — iterative curvature-minimisation raceline (Kegel-style)
------------------------------------------------------------------
This is the standard, simple racing-line algorithm used by e.g. TUM's
global_racetrajectory_optimization when a full nonlinear-program minimum-
curvature solve is overkill: parameterise the line as a per-station lateral
offset alpha[i] from the centreline, alpha[i] in [-w_right[i], w_left[i]]
(the recorded track's own width budget), and iteratively nudge each station
toward locally lower curvature (curvature is a convex-ish local function of
neighbouring offsets under quadratic smoothing), re-fitting speed each
round via the same three-pass method sim/speed_profile.py already uses
(corner-speed limit -> forward accel cap -> backward brake cap). Convergence
is judged on lap time, not curvature directly, since curvature reduction that
does not shorten the lap is not the objective.

WHY NOT A FULL NONLINEAR MINIMUM-CURVATURE / MINIMUM-TIME QP
--------------------------------------------------------------
TUM's tool (and most competition raceline optimisers) solves a convex QP over
all stations at once for the true minimum-curvature line, then a separate
convex QP for minimum-time given fixed geometry. That is more optimal than
the greedy per-station nudge here, but needs a QP solver dependency this repo
does not otherwise have (cvxpy/osqp) and meaningfully more code. The
iterative nudge is the same family of method (used historically before the
QP formulations existed, e.g. Kegel/Casanova) and converges to a good, not
necessarily globally optimal, racing line -- adequate for an offline
reference the live MPC tracks; it does not need to be provably optimal to be
faster than the centreline.

ALAT_CEILING — the offline optimiser must respect the SIM's cap, not the
physical grip limit
--------------------------------------------------------------------------
CLAUDE.md documents FSDS enforcing a measured sustained lateral-acceleration
ceiling (model/vehicle_physics.py's alat_ceiling_at(), ~7.5 m/s^2 flat below
~10.7 m/s, rising per the sweep fit above it) well below the plant's true
grip (mu*g ~17 m/s^2, OPTIMAL_LAP_A_LAT_MAX=12.0 in sim/speed_profile.py).
optimal_lap_time() uses OPTIMAL_LAP_A_LAT_MAX deliberately AS a physical
lower bound the car should approach but need not achieve in practice (see
that function's own docstring on why the conservative planning limit made a
"bound" the car routinely beat). A racing line is different: it is the path
the live/sim car is actually meant to DRIVE, so if it is optimised against a
lateral limit the sim will not deliver, the exported path/speed pair asks
the car to do something it structurally cannot -- exactly the gap CLAUDE.md
tracks as still open. This module therefore speed-profiles (and re-shapes)
against alat_ceiling_at(v), not OPTIMAL_LAP_A_LAT_MAX/mu.

USAGE
-----
    python3 -m tuner.raceline_optimizer                       # cone_map.json -> raceline_export.csv
    python3 -m tuner.raceline_optimizer /path/to/cone_map.json out.csv
    python3 -m tuner.raceline_optimizer --iters 60 --margin 0.3

Re-run whenever the recorded map changes, same as export_speed_profile.py.
"""
import argparse
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from model.vehicle_physics import VehicleParams  # noqa: E402
from sim.track_io import load_cone_map, _reconstruct_centreline  # noqa: E402
from sim.track_io import _resample_dense  # noqa: E402
import sim.speed_profile as speed_profile  # noqa: E402

DEFAULT_MAP = os.path.join(
    os.path.dirname(os.path.dirname(_HERE)), "cone_map.json")
DEFAULT_OUT = os.path.join(_HERE, "raceline_export.csv")

# Margin kept clear of each boundary cone (m) -- a physical car has nonzero
# width and the recorded cone position itself carries some noise even under
# FSDS's ground-truth perception stand-in (see CLAUDE.md: the cone_recorder's
# own merge distance is 0.8 m). Kept well under half a typical FS lane width
# (~3 m) so this rarely binds except at the very tightest apexes.
DEFAULT_MARGIN = 0.35

# Smoothing weight in the per-iteration curvature-reduction step: how strongly
# each station is pulled toward the average of its neighbours' offsets
# (measured, not planned, curvature reduction) versus kept at its current
# value. 0 disables smoothing (no progress); too high overshoots and can push
# a station straight to its track-width limit every round, oscillating rather
# than converging. Chosen empirically -- see _smooth_step().
SMOOTH_GAIN = 0.35

DEFAULT_ITERS = 80

# Final pure-smoothing pass applied to the best alpha found by the main loop
# (see optimize_raceline) -- cleans up residual per-station noise the main
# loop's fixed step size leaves even at its own lap-time optimum.
FINAL_SMOOTH_ITERS = 15
FINAL_SMOOTH_GAIN = 0.3
# Fractional lap-time regression tolerated in exchange for the smoothing
# pass's curvature-noise reduction (see the accept check below).
FINAL_SMOOTH_TIME_TOLERANCE = 0.002


def _station_frame(path_xy):
    """
    Per-station unit tangent and left-normal for a closed polyline, via
    central differences (matches sim/speed_profile.compute_path_curvature's
    own finite-difference convention so curvature sign/orientation agree).

    Returns (tangent, left_normal), each shape (n, 2), unit vectors.
    """
    d = np.gradient(path_xy, axis=0)
    norm = np.linalg.norm(d, axis=1, keepdims=True)
    norm = np.where(norm < 1e-9, 1.0, norm)
    tangent = d / norm
    left_normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
    return tangent, left_normal


def _track_width_bounds(centre_xy, tangent, left_normal, blue, yellow,
                         max_search=6.0, along_track_window=2.5):
    """
    Per-station (w_left, w_right) track-width budget: how far the centreline
    may move toward blue (left, +alpha) or yellow (right, -alpha) before
    reaching that station's boundary cones.

    Approximated as the distance from the centreline point to the nearest
    cone of each colour ABEAM of it (within along_track_window of zero
    longitudinal offset), projected onto the local left-normal -- consistent
    with the left=blue/right=yellow convention sim/track_io._seed_pose() and
    planning/boundary.py both use.

    The along-track window is essential, not cosmetic: on a tight corner
    (small radius), a cone from the OTHER side of the bend -- ahead or
    behind along the track's own path, not truly abeam -- can sit at a small
    RAW distance from the station while contributing almost nothing to
    lateral clearance, or worse, can lie geometrically "to the left" in pure
    lateral-projection terms while actually describing the track's far side.
    Measured on this recorded map: filtering only by sign of the lateral
    component (no along-track gate) found a blue cone 0.098 m "left" of the
    tightest apex station, versus 1.07 m once cones more than
    along_track_window along the tangent are excluded -- the near-zero
    figure was a same-corner cone almost directly ahead, not a real width
    constraint at that station.

    A station with NO cone found within along_track_window (a gap in the
    recording, or a corner tight enough that the along-track gate excludes
    every cone that would otherwise look abeam) is NOT given max_search
    directly -- max_search (6.0 m) is a generous ceiling, not a conservative
    one, relative to this track's typical ~1.2-1.7 m station widths, so
    handing a gap station the ceiling value tells the optimiser "the track
    is unusually wide here" at exactly the wrong moment. Measured on this
    recorded map: one such gap station (idx 406, no blue cone within 2.5 m
    along-track of it at all -- the nearest lateral candidates found were
    ~38 m away, i.e. a different part of the track entirely, not a real
    boundary) produced w_left=6.0 against ~1.3-1.6 m at every neighbouring
    station, and the optimiser used that spurious slack to introduce a
    sharp, localised curvature kink there (peak |dkappa/di| 3.5x every other
    station's). Gap stations are instead interpolated from the nearest
    valid (non-gap) neighbours on each side, closed-loop.
    """
    n = len(centre_xy)
    w_left = np.full(n, max_search)
    w_right = np.full(n, max_search)
    left_found = np.zeros(n, dtype=bool)
    right_found = np.zeros(n, dtype=bool)

    for i in range(n):
        p = centre_xy[i]
        tn = tangent[i]
        ln = left_normal[i]

        # A candidate must ALSO be within max_search of the station in the
        # lateral component itself (not just pass the along-track gate), or
        # it is rejected as a candidate entirely rather than clipped to
        # max_search -- clipping a bogus far-away "abeam" match (see
        # _fill_gap_stations' docstring: idx 406 on this map matched a cone
        # ~38 m away purely because its along-track offset happened to be
        # small) still marks the station as `found`, so the gap-filling
        # pass below never gets a chance to run for it.
        db = blue - p
        along_b = db @ tn
        lat_b = db @ ln
        abeam_b = np.abs(along_b) <= along_track_window
        pos_b = lat_b[abeam_b & (lat_b > 0) & (lat_b <= max_search)]
        if len(pos_b) > 0:
            w_left[i] = float(np.min(pos_b))
            left_found[i] = True

        dy = yellow - p
        along_y = dy @ tn
        lat_y = -(dy @ ln)   # yellow is on the right -> negate to get a positive distance
        abeam_y = np.abs(along_y) <= along_track_window
        pos_y = lat_y[abeam_y & (lat_y > 0) & (lat_y <= max_search)]
        if len(pos_y) > 0:
            w_right[i] = float(np.min(pos_y))
            right_found[i] = True

    w_left = _fill_gap_stations(w_left, left_found)
    w_right = _fill_gap_stations(w_right, right_found)
    return w_left, w_right


def _fill_gap_stations(w, found):
    """
    Replace stations where `found` is False (see _track_width_bounds) with a
    linear interpolation, in ARRAY-INDEX space, between the nearest valid
    (found=True) neighbour on each side -- treating the array as a closed
    loop, matching a recorded lap. If every station is a gap (found is
    entirely False), returns `w` unchanged rather than dividing by zero.
    """
    n = len(w)
    if found.all() or not found.any():
        return w

    idx = np.arange(n)
    valid_idx = idx[found]
    w_out = w.copy()
    gap_idx = idx[~found]

    # Circular distance from each gap station to every valid station, so the
    # nearest neighbour search wraps correctly across the array's own ends
    # (index 0 and n-1 are adjacent on a closed loop).
    for i in gap_idx:
        d = np.abs(valid_idx - i)
        d = np.minimum(d, n - d)
        nearest_two = valid_idx[np.argsort(d)[:2]]
        if len(nearest_two) == 1:
            w_out[i] = w[nearest_two[0]]
        else:
            a, b = nearest_two
            wa, wb = w[a], w[b]
            da = min(abs(a - i), n - abs(a - i))
            db_ = min(abs(b - i), n - abs(b - i))
            total = da + db_
            w_out[i] = wb if total == 0 else (wa * db_ + wb * da) / total

    return w_out


def _apply_offset(centre_xy, left_normal, alpha):
    return centre_xy + left_normal * alpha[:, None]


def _curvature(path_xy):
    """
    Signed curvature via arc-length-parameterised finite differences.

    NOT sim.speed_profile.compute_path_curvature(): that function's
    np.gradient() differentiates with respect to INDEX, implicitly assuming
    uniform point spacing. A racing line (unlike the evenly-arc-length-
    resampled centreline it was written for) has non-uniform spacing after a
    lateral offset is applied -- offset stations trace a longer or shorter
    path locally depending on which way they were pushed -- and differentiating
    by index against that distorts the curvature estimate badly (measured:
    a single "curvature-reducing" step on this module's first attempt nearly
    quadrupled peak curvature, 0.208 -> 0.74, purely from this artifact).
    Differentiating with respect to actual arc length removes the distortion.
    """
    x = path_xy[:, 0]
    y = path_xy[:, 1]
    ds = np.hypot(np.gradient(x), np.gradient(y))
    ds = np.where(ds < 1e-9, 1e-9, ds)

    dx = np.gradient(x) / ds
    dy = np.gradient(y) / ds
    ddx = np.gradient(dx) / ds
    ddy = np.gradient(dy) / ds

    denom = (dx ** 2 + dy ** 2) ** 1.5
    denom = np.where(denom < 1e-9, 1e-9, denom)
    return (dx * ddy - dy * ddx) / denom


def _smooth_step(alpha, kappa, w_left, w_right, gain=SMOOTH_GAIN):
    """
    One curvature-reduction iteration, in two parts:

    1. PUSH each station directly away from the centre of its local bend —
       moving toward the OUTSIDE of a turn increases the effective corner
       radius there (the classic "widen entry, clip apex, widen exit" racing
       line). left_normal (see _station_frame) points from the path TOWARD
       the bend's own centre of curvature for a positive-kappa (left-hand)
       turn -- verified on a synthetic 90 deg arc: dot(left_normal,
       direction-to-turn-centre) = +1.0 exactly at the apex. Moving toward
       the OUTSIDE is therefore the negative of that: alpha must INCREASE
       (away from left_normal's own direction) for positive kappa, hence the
       PLUS sign below (and the reverse for a right-hand/negative-kappa
       bend, where alpha must decrease) -- i.e. alpha moves with sign(kappa),
       not against it. Scaled by curvature magnitude so straight sections
       (kappa ~ 0) do not move at all, only genuine corners do. Starting
       alpha at all-zero needs this outward push: an update that only
       averages neighbouring offsets (no direction term) cannot break
       symmetry from a flat initial line, since neighbour_avg is 0 too.

    2. SMOOTH by pulling each station part-way toward the average of its two
       neighbours, so the push in (1) does not produce a jagged, station-by-
       station zig-zag — curvature is driven by the SECOND difference of
       position, so this directly damps high-frequency curvature the push
       alone would introduce.

    Both steps are clipped to the track-width budget (w_left/w_right) so the
    line never leaves the recorded track. Closed-loop neighbours (index 0
    and -1 wrap around), matching a recorded lap.
    """
    n = len(alpha)
    kappa_abs = np.abs(kappa)
    kmax = kappa_abs.max()
    weight = kappa_abs / kmax if kmax > 1e-9 else np.zeros(n)

    # Fixed per-iteration step size (m), NOT scaled by the local track-width
    # budget: scaling by w_left/w_right let a single iteration push a station
    # most of the way to its full track-width limit in one shot (measured:
    # alpha reached 87% of max track width after 5 iterations, well before
    # the neighbour-smoothing pass could keep the line coherent), causing the
    # curvature-reduction step to overshoot into instability instead of
    # gradually converging. A small constant step means many iterations are
    # needed, but each one stays local enough for the smoothing pass to
    # actually damp the high-frequency curvature the push introduces.
    PUSH_STEP_M = 0.08
    pushed = alpha + gain * weight * np.sign(kappa) * PUSH_STEP_M

    left_nb = np.roll(pushed, 1)
    right_nb = np.roll(pushed, -1)
    neighbour_avg = 0.5 * (left_nb + right_nb)
    smoothed = pushed + gain * (neighbour_avg - pushed)

    return np.clip(smoothed, -w_right, w_left)


def optimize_raceline(
    blue, yellow,
    n_points=None,
    iters=DEFAULT_ITERS,
    margin=DEFAULT_MARGIN,
    params=None,
):
    """
    Compute a minimum-time racing line + speed profile from a recorded cone
    map's boundaries.

    Parameters
    ----------
    blue, yellow : (N, 2) arrays  Recorded left/right boundary cones.
    iters : int    Number of curvature-reduction rounds (see module docstring).
    margin : float  Clearance (m) kept from each boundary, see DEFAULT_MARGIN.
    params : VehicleParams, optional  Source of alat_ceiling_at()/max_accel/
             max_accel_brake. A fresh VehicleParams() if not given.

    Returns
    -------
    (path_X, path_Y, path_Psi, path_v, blue, yellow, lap_time_s) — the first
    four have the same shape/meaning as sim.track_io.load_recorded_track()'s
    return, so callers (export, GUI) can treat a raceline exactly like any
    other precomputed path.
    """
    if params is None:
        params = VehicleParams()

    from sim.track_io import PATH_N_POINTS
    n_points = PATH_N_POINTS if n_points is None else n_points

    centreline = _reconstruct_centreline(blue, yellow)
    # Dense, evenly arc-length-spaced resample BEFORE optimising: the raw
    # marched centreline has uneven point spacing (denser on tight corners,
    # per build_path_walls' own step size), which would make the neighbour-
    # averaging step in _smooth_step operate over wildly different arc-length
    # gaps station to station. _resample_dense's chord-parameterised spline
    # fit removes that so each iteration's neighbour pull is a comparable
    # arc-length scale everywhere on the lap.
    path_X, path_Y, _psi0, _v0 = _resample_dense(
        centreline[:, 0], centreline[:, 1], n_points=n_points
    )
    centre_xy = np.column_stack([path_X, path_Y])

    tangent, left_normal = _station_frame(centre_xy)
    w_left, w_right = _track_width_bounds(centre_xy, tangent, left_normal, blue, yellow)
    # Keep clear of the boundary itself.
    w_left = np.maximum(w_left - margin, 0.0)
    w_right = np.maximum(w_right - margin, 0.0)

    alpha = np.zeros(len(centre_xy))
    best_alpha = alpha.copy()
    best_time = np.inf
    best_v = None

    for it in range(iters):
        path_xy = _apply_offset(centre_xy, left_normal, alpha)
        kappa = _curvature(path_xy)

        lap_time, v_profile = _lap_time_and_speed(path_xy, kappa, params)
        if lap_time < best_time:
            best_time = lap_time
            best_alpha = alpha.copy()
            best_v = v_profile

        alpha = _smooth_step(alpha, kappa, w_left, w_right)

    # Final pure-smoothing pass (no outward push, see _smooth_step) on the
    # best-lap-time alpha found above. The main loop's fixed step size
    # leaves small residual per-station noise even at its own optimum —
    # measured on this map's tightest corner: a non-monotonic curvature
    # wobble (kappa dipping/spiking/dipping across 5 adjacent stations)
    # that a clean "widen entry, clip apex, widen exit" line would not have.
    # Neighbour-averaging only (no push) strictly reduces high-frequency
    # noise without reintroducing the symmetry-breaking problem _smooth_step
    # solves during the main loop (alpha is already non-zero here), so this
    # is applied unconditionally and re-checked against lap time; if the
    # smoothed line is not at least as fast, the pre-smooth best is kept.
    smoothed_alpha = best_alpha.copy()
    for _ in range(FINAL_SMOOTH_ITERS):
        left_nb = np.roll(smoothed_alpha, 1)
        right_nb = np.roll(smoothed_alpha, -1)
        neighbour_avg = 0.5 * (left_nb + right_nb)
        smoothed_alpha = smoothed_alpha + FINAL_SMOOTH_GAIN * (neighbour_avg - smoothed_alpha)
        smoothed_alpha = np.clip(smoothed_alpha, -w_right, w_left)

    smoothed_path = _apply_offset(centre_xy, left_normal, smoothed_alpha)
    smoothed_kappa = _curvature(smoothed_path)
    smoothed_time, smoothed_v = _lap_time_and_speed(smoothed_path, smoothed_kappa, params)
    # Accept a small lap-time cost (tolerance below) in exchange for removing
    # residual curvature noise -- measured on this map's tightest corner, the
    # smoothing pass cut a non-monotonic 5-station curvature wobble at a cost
    # of +0.005% lap time (36.9646 -> 36.9663 s). A path that is a few
    # milliseconds "faster" but noticeably jagged at one corner is not
    # actually the better reference for the MPC to track.
    if smoothed_time <= best_time * (1.0 + FINAL_SMOOTH_TIME_TOLERANCE):
        best_alpha, best_time, best_v = smoothed_alpha, smoothed_time, smoothed_v

    path_xy = _apply_offset(centre_xy, left_normal, best_alpha)
    dx = np.gradient(path_xy[:, 0])
    dy = np.gradient(path_xy[:, 1])
    path_Psi = np.arctan2(dy, dx)

    return (path_xy[:, 0], path_xy[:, 1], path_Psi, best_v,
            blue, yellow, best_time)


def _lap_time_and_speed(path_xy, kappa, params):
    """
    Speed-profile + integrate lap time for a candidate path, using the SIM's
    alat_ceiling_at(v) (speed-dependent) rather than a flat physical limit —
    see module docstring's ALAT_CEILING section for why. Requires a small
    fixed-point pass since the corner-speed limit at each station depends on
    the ceiling AT that station's own speed.
    """
    n = len(path_xy)
    segs = np.hypot(np.diff(path_xy[:, 0]), np.diff(path_xy[:, 1]))
    kappa_safe = np.maximum(np.abs(kappa), 1e-9)

    # Fixed point: start from the flat low-speed ceiling, tighten once speed
    # is known, repeat a few rounds -- alat_ceiling_at is monotonically
    # non-decreasing in v and the corner speed from a HIGHER ceiling can only
    # rise, so this converges quickly (in practice 2-3 rounds).
    v = np.full(n, params.alat_ceiling)
    for _ in range(4):
        ceiling = np.array([params.alat_ceiling_at(vi) for vi in v])
        v_corner = np.sqrt(ceiling / kappa_safe)
        v_corner = np.minimum(v_corner, params.max_v)

        v_new = v_corner.copy()
        for i in range(1, n):
            v_allowed = np.sqrt(max(v_new[i - 1], 0.0) ** 2 + 2.0 * params.max_accel * segs[i - 1])
            v_new[i] = min(v_new[i], v_allowed)
        a_brake_mag = abs(params.max_accel_brake)
        for i in range(n - 2, -1, -1):
            v_allowed = np.sqrt(max(v_new[i + 1], 0.0) ** 2 + 2.0 * a_brake_mag * segs[i])
            v_new[i] = min(v_new[i], v_allowed)
        v = v_new

    v_avg = 0.5 * (v[:-1] + v[1:])
    v_avg = np.maximum(v_avg, 1e-3)
    lap_time = float(np.sum(segs / v_avg))
    return lap_time, v


def export(map_path: str, out_path: str, iters=DEFAULT_ITERS, margin=DEFAULT_MARGIN):
    blue, yellow = load_cone_map(map_path)
    params = VehicleParams()

    centreline = _reconstruct_centreline(blue, yellow)
    from sim.track_io import PATH_N_POINTS
    cx, cy, _psi, centre_v = _resample_dense(centreline[:, 0], centreline[:, 1], n_points=PATH_N_POINTS)
    centre_kappa = speed_profile.compute_path_curvature(cx, cy)
    centre_time, _ = _lap_time_and_speed(np.column_stack([cx, cy]), centre_kappa, params)

    path_X, path_Y, path_Psi, path_v, _blue, _yellow, race_time = optimize_raceline(
        blue, yellow, iters=iters, margin=margin, params=params
    )

    with open(out_path, "w") as f:
        f.write("# x,y,psi,v_target -- minimum-time racing line exported by "
                "tuner.raceline_optimizer from a cone_recorder map.\n")
        f.write(f"# source_map={os.path.abspath(map_path)}\n")
        f.write(f"# centreline_lap_time_s={centre_time:.3f} raceline_lap_time_s={race_time:.3f} "
                f"improvement_pct={100.0 * (centre_time - race_time) / centre_time:.2f}\n")
        f.write("x,y,psi,v_target\n")
        for x, y, psi, v in zip(path_X, path_Y, path_Psi, path_v):
            f.write(f"{x:.4f},{y:.4f},{psi:.5f},{v:.4f}\n")

    print(f"Wrote {len(path_X)} points -> {out_path}")
    print(f"centreline lap time (alat_ceiling-limited): {centre_time:.3f} s")
    print(f"raceline   lap time (alat_ceiling-limited): {race_time:.3f} s "
          f"({100.0 * (centre_time - race_time) / centre_time:.2f}% faster)")
    print(f"v_target range: {path_v.min():.2f} - {path_v.max():.2f} m/s")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("map_path", nargs="?", default=DEFAULT_MAP)
    ap.add_argument("out_path", nargs="?", default=DEFAULT_OUT)
    ap.add_argument("--iters", type=int, default=DEFAULT_ITERS,
                     help="curvature-reduction iterations (default: %(default)s)")
    ap.add_argument("--margin", type=float, default=DEFAULT_MARGIN,
                     help="clearance kept from each boundary cone, metres (default: %(default)s)")
    args = ap.parse_args()
    export(args.map_path, args.out_path, iters=args.iters, margin=args.margin)


if __name__ == "__main__":
    main()
