"""
tuner/tools/raceline_optimizer.py — Minimum-time racing line, not just a centreline
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
exported through the existing tuner/tools/export_speed_profile.py CSV mechanism,
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

Respecting the ceiling is necessary but NOT sufficient: planning at exactly
100% of it leaves the controller no lateral budget to correct with, since
closing any tracking error needs curvature beyond the reference's own. Left
unmargined, every corner runs at corner_demand > 1 and steering saturates on
a large fraction of corner ticks. ALAT_MARGIN / BRAKE_MARGIN reserve that
headroom -- see their comments.

THIS FILE'S v_target IS THE ONE THE CAR DRIVES
----------------------------------------------
launch_all.sh points BOTH map_path (speed) and path_map_path (geometry) at
raceline.csv. Speed and geometry must describe the same line: pairing this
path with tuner/tools/export_speed_profile.py's centreline profile hands the car a
speed computed for a curvature it is not driving, and the two differ most
exactly at the apexes (raceline p99 |kappa| 0.174 vs centreline 0.162 -- a
racing line trades a straighter entry for a TIGHTER apex). The v_target
written here is therefore load-bearing, not a diagnostic byproduct.

USAGE
-----
    python3 -m tuner.tools.raceline_optimizer                       # the default track
    python3 -m tuner.tools.raceline_optimizer comp_test_map_3       # a track by name
    python3 -m tuner.tools.raceline_optimizer --list                # what tracks exist
    python3 -m tuner.tools.raceline_optimizer /path/to/cone_map.json out.csv
    python3 -m tuner.tools.raceline_optimizer --iters 60 --margin 0.3

With a track name (or no argument), the output goes to
`tracks/<name>/raceline.csv`, which is where `launch_all.sh`'s `TRACK=`
variable points the car. See `tracks/__init__.py` for the layout.

Re-run whenever the recorded map changes, same as export_speed_profile.py.
"""
import argparse
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(_HERE)))

from model.vehicle_physics import VehicleParams  # noqa: E402
from sim.track_io import load_cone_map, _reconstruct_centreline  # noqa: E402
from sim.track_io import _resample_dense  # noqa: E402
import sim.speed_profile as speed_profile  # noqa: E402
from tracks import (  # noqa: E402
    DEFAULT_TRACK, RACELINE_NAME, default_out_for, list_tracks, resolve_map_arg,
)

# Margin kept clear of each boundary cone (m), measured from the cone centre
# to the PATH -- and the path is the car's centreline, so this must cover the
# car's own half-width or a "contained" line still puts a wheel through a cone.
#
# Budget: half of VehicleParams.tf (front track 1.25 m) = 0.625, plus tyre
# width and the recorded cone's own position noise (the cone_recorder merges
# detections within 0.8 m, so a recorded centre can sit some way off the true
# one). 0.90 m covers the axle half-width with ~0.28 m left over.
#
# Must cover at least half the front track, or this margin does not actually
# model the car regardless of what its comment claims -- a tighter speed
# profile plus a tight optimiser can then push the line to within a few
# centimetres of a cone centre, i.e. inside the car's own width, without any
# clearance check catching it. _assert_clearance() below fails the export
# rather than shipping such a line silently.
DEFAULT_MARGIN = 0.90

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

# Fraction of alat_ceiling_at(v) the exported speed profile is allowed to use.
#
# The ceiling is what FSDS delivers to a car already tracking its line
# perfectly. A profile planned AT 100% of it therefore leaves the MPC nothing
# to correct with: any lateral error needs curvature beyond the reference's
# own, which is exactly the acceleration the sim refuses to supply, so the
# controller saturates its steering and the error grows instead of closing.
# Without this margin, corners run with corner_demand > 1 and steering
# saturates on a large fraction of corner ticks, since the exported profile's
# own implied a_lat can still exceed the ceiling at some stations.
#
# 0.85 keeps ~15% of the lateral budget as correction headroom. Chosen to
# leave margin comparable to the tracking errors actually observed rather
# than fitted -- if the car stops saturating and corner_demand drops near 1,
# this can be raised again for lap time.
ALAT_MARGIN = 0.85

# Ceiling on the exported profile's braking, as a fraction of
# |params.max_accel_brake|.
#
# max_accel_brake (-7.0) is documented in model/vehicle_physics.py as a
# BACKSTOP matching mpc_core's MAX_BRAKE, explicitly "not an FSDS-measured
# value". Planning the reference right at it asks the car to brake at its
# own absolute limit for the whole braking zone, leaving no authority for the
# MPC to brake harder when it arrives hot. The live logs bear this out: the
# car sustains about -6.3 m/s^2 (0.25 s window, 1st percentile) but the MPC
# never COMMANDS below -2.2 m/s^2, so the reference -- not the brakes -- is
# what sets corner entry speed.
BRAKE_MARGIN = 0.85

# Curvature the exported line should not exceed (1/m). Above this a candidate
# is penalised in the selection score below, so a kinked path cannot win on
# lap time alone.
#
# 0.22 is just above the max curvature of every line this optimiser produced
# before the kink appeared (old raceline 0.208, margin-0.35 raceline 0.220,
# centreline 0.212) -- i.e. it is the empirical "a clean line on this track
# never needs more than this", not a vehicle limit. For reference the car's
# own kinematic floor is 1/3.32 m = 0.30, so anything above that is not
# merely tight but impossible at full lock.
CURVATURE_SOFT_MAX = 0.22

# Seconds of lap time one unit of excess curvature is worth in the selection
# score. Excess is measured as max(0, max|kappa| - CURVATURE_SOFT_MAX), so at
# this weight the 0.379 spike scores as +7.9 s -- decisively worse than any
# real lap-time gain, while a line at 0.23 pays only 0.16 s and can still win
# if it is genuinely quicker. The penalty is deliberately steep: a kink is a
# correctness problem (the live car saturates and leaves the track), not a
# comfort preference to be traded off gently.
CURVATURE_PENALTY_S_PER_INVM = 50.0


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


def _candidate_score(lap_time, kappa):
    """
    Selection score (lower is better): lap time plus a curvature penalty.

    Lap time alone is the wrong objective for a reference the MPC must TRACK.
    A path with a curvature kink can be marginally quicker in the point-mass
    speed model used here -- that model only sees kappa through the corner
    speed sqrt(a_lat/kappa) and is perfectly happy to take a 2.6 m radius --
    while being undrivable for a car whose steering stops at a 3.32 m radius:
    a curvature spike above the car's steering limit can win on lap time in
    this model while saturating the live car's steering for a large fraction
    of ticks in that band.

    Penalising only the EXCESS above CURVATURE_SOFT_MAX (rather than, say,
    mean curvature) keeps this inert for every well-formed line -- a clean
    path scores exactly its lap time and the optimiser behaves as before.
    """
    excess = max(0.0, float(np.abs(kappa).max()) - CURVATURE_SOFT_MAX)
    return lap_time + CURVATURE_PENALTY_S_PER_INVM * excess


def _curvature(path_xy):
    """
    Signed curvature via arc-length-parameterised finite differences.

    NOT sim.speed_profile.compute_path_curvature(): that function's
    np.gradient() differentiates with respect to INDEX, implicitly assuming
    uniform point spacing. A racing line (unlike the evenly-arc-length-
    resampled centreline it was written for) has non-uniform spacing after a
    lateral offset is applied -- offset stations trace a longer or shorter
    path locally depending on which way they were pushed -- and differentiating
    by index against that distorts the curvature estimate badly: a
    "curvature-reducing" step evaluated this way can spuriously balloon peak
    curvature purely from this artifact, worse than the curvature it started
    with. Differentiating with respect to actual arc length removes the
    distortion.
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

    # Headroom taper: shrink the step as a station approaches the bound it is
    # being pushed TOWARD, so it converges onto its limit instead of slamming
    # into the clip below.
    #
    # NOT the same thing as the rejected "scale by w_left/w_right" above. That
    # scaled the step UP where the corridor was wide (one iteration crossing
    # most of the track). This only ever scales DOWN, and by REMAINING headroom
    # rather than total width -- a station with room takes the full 0.08 m, a
    # station near its bound takes proportionally less, and none takes more.
    #
    # Why it matters: raising DEFAULT_MARGIN to 0.90 halved the corridor
    # (median 2.80 -> 1.70 m, min 0.90) and the fixed step became large
    # relative to it. Clip activations went 398 -> 1431 over the run, with 15
    # stations pinned against a bound for more than half the iterations. A
    # pinned station cannot move while its neighbours still can, so the
    # neighbour-average in (2) bends the line AROUND the frozen points and
    # kinks it -- measured as a curvature spike to 0.379 (R = 2.6 m, inside
    # the car's own 3.32 m kinematic minimum) that the final smoothing pass
    # could not fully undo, and which saturated the live car's steering on
    # 26.8% of ticks in that band.
    direction = np.sign(kappa)
    headroom = np.where(direction >= 0.0, w_left - alpha, alpha + w_right)
    headroom = np.maximum(headroom, 0.0)
    # Full step once at least TAPER_SPAN_M of room remains; linear below that.
    TAPER_SPAN_M = 4.0 * PUSH_STEP_M
    taper = np.minimum(1.0, headroom / TAPER_SPAN_M)
    pushed = alpha + gain * weight * direction * PUSH_STEP_M * taper

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
    # Candidates are ranked by _candidate_score (lap time + curvature penalty),
    # NOT lap time alone -- see that function. best_time tracks the true lap
    # time of the chosen candidate for reporting; best_score ranks them.
    best_score = np.inf
    best_time = np.inf
    best_v = None

    for it in range(iters):
        path_xy = _apply_offset(centre_xy, left_normal, alpha)
        kappa = _curvature(path_xy)

        lap_time, v_profile = _lap_time_and_speed(path_xy, kappa, params)
        score = _candidate_score(lap_time, kappa)
        if score < best_score:
            best_score = score
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
    # Compared on _candidate_score, not raw lap time, so the smoothed line is
    # credited for the curvature it removes. Under lap time alone a smoothing
    # pass that cut a kink but cost more than the tolerance was REJECTED --
    # the pass would do its job and then be thrown away.
    smoothed_score = _candidate_score(smoothed_time, smoothed_kappa)
    if smoothed_score <= best_score * (1.0 + FINAL_SMOOTH_TIME_TOLERANCE):
        best_alpha, best_time, best_v = smoothed_alpha, smoothed_time, smoothed_v
        best_score = smoothed_score

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
    # Segment i is the gap from station i to i+1, with the LAST entry closing
    # the loop from n-1 back to 0. The geometry everywhere else in this module
    # already treats the path as closed (np.roll in _smooth_step /
    # _fill_gap_stations); before this, the accel/brake passes below were the
    # one place that did not, so the profile was solved as an open segment.
    # That let station 0 start at whatever speed the corner limit allowed with
    # no braking distance leading into it, and left the final stations with no
    # obligation to slow for turn 1 -- a discontinuity the car meets once per
    # lap, exactly at the start/finish seam.
    segs = np.hypot(np.diff(path_xy[:, 0], append=path_xy[0, 0]),
                    np.diff(path_xy[:, 1], append=path_xy[0, 1]))
    kappa_safe = np.maximum(np.abs(kappa), 1e-9)

    # Fixed point: start from the flat low-speed ceiling, tighten once speed
    # is known, repeat a few rounds -- alat_ceiling_at is monotonically
    # non-decreasing in v and the corner speed from a HIGHER ceiling can only
    # rise, so this converges quickly (in practice 2-3 rounds).
    v = np.full(n, params.alat_ceiling)
    for _ in range(4):
        # ALAT_MARGIN reserves lateral headroom for the MPC to correct with;
        # see its comment. Without it the reference consumes the entire
        # ceiling and any tracking error becomes unrecoverable by construction.
        ceiling = np.array([params.alat_ceiling_at(vi) for vi in v]) * ALAT_MARGIN
        v_corner = np.sqrt(ceiling / kappa_safe)
        v_corner = np.minimum(v_corner, params.max_v)

        v_new = v_corner.copy()
        # Two wrapped laps per direction: one lap propagates a constraint from
        # any station to every later one, and the second carries whatever
        # crossed the seam on the first back around. A third changes nothing --
        # each pass is a monotone min-update over the same fixed corner limits,
        # so this reaches its fixed point after the constraint has travelled
        # the full loop once from its binding station.
        a_brake_mag = abs(params.max_accel_brake) * BRAKE_MARGIN
        for _lap in range(2):
            for i in range(n):                       # forward: traction limit
                j = (i - 1) % n
                v_allowed = np.sqrt(max(v_new[j], 0.0) ** 2
                                    + 2.0 * params.max_accel * segs[j])
                v_new[i] = min(v_new[i], v_allowed)
            for i in range(n - 1, -1, -1):           # backward: braking limit
                j = (i + 1) % n
                v_allowed = np.sqrt(max(v_new[j], 0.0) ** 2
                                    + 2.0 * a_brake_mag * segs[i])
                v_new[i] = min(v_new[i], v_allowed)
        v = v_new

    # segs now has n entries (the last closes the loop), so pair each station
    # with its successor mod n rather than truncating -- otherwise the closing
    # segment's time is dropped from the lap total.
    v_avg = 0.5 * (v + np.roll(v, -1))
    v_avg = np.maximum(v_avg, 1e-3)
    lap_time = float(np.sum(segs / v_avg))
    return lap_time, v


# Fraction of the car's max achievable yaw rate (at delta_max, per-station
# v_target -- see max_yaw_rate()) to "pre-spend" as heading lead when
# shaping psi_target below. 1.0 = as aggressive as physically achievable
# given the ALREADY-CONVERGED speed profile; lower values are a gentler
# nudge. See late_turn_in_investigation.md Part 8/9 for the synthetic
# testing behind this: a FIXED lookahead distance (tried first, rejected)
# saturates steering immediately on a realistic corner entry, whereas
# scaling by achievable yaw rate at the station's own planned speed does
# not, because the lead can never exceed what the speed profile already
# allows the car to execute.
#
# Measured on comp_test_map_3 at this value (Part 10): mean lead 3.58 deg,
# max 3.76 deg -- no full-lock cliff, but ALSO active across nearly the
# WHOLE lap (998/1000 stations), not just approaching corners, because
# this track has almost no genuinely straight sections (geometric psi
# climbs 0.4->16 deg over the first 40m alone). Lower this first if a live
# test looks like "cuts every bend early everywhere" rather than "commits
# earlier specifically into sharp corners" -- that symptom is this
# constant, not a broken mechanism. NOT YET LIVE-VALIDATED at any value.
HEADING_LEAD_AUTHORITY_FRAC = 0.5

# Placeholder rear-axle slip-angle bound (rad) used only to FLAG stations
# where the shaped heading profile implies more yaw rate than the linear
# bicycle model's tyres can deliver without leaving their linear region --
# NOT a validated limit (typical small-FS-car linear-region edges are
# reported anywhere from 3-8 deg depending on tyre/loading; 5 deg is a
# round, unmeasured guess). Diagnostic-only for now (see check_slip's
# docstring) -- do not treat slip_flags as authoritative or feed it into a
# hard export failure until this is measured against this car's actual
# Cf/Cr linear region.
#
# Measured on comp_test_map_3 at this value (Part 10): 110/1000 stations
# (11%) exceed it, all at the track's sharpest/slowest corners (peak
# |kappa| 0.12-0.21, v 5.5-7.9 m/s) -- clustered at real corners, not
# scattered noise, so the FORMULA is behaving sensibly; whether 5 deg
# itself is the right bound for this car is still unknown.
SLIP_LIMIT_RAD = math.radians(5.0)


def max_yaw_rate(v, params, delta_max_rad=math.radians(25.0)):
    """
    Max steady-state yaw rate the kinematic bicycle model can deliver at
    speed v with full steering lock -- same relationship mpc_core.py's
    _discrete_model uses for A_kin[2,6] (r = v/(lf+lr) * delta), evaluated
    at the live controller's actual steering limit (see mpc_core.py's own
    module-note on MAX_STEER_RAD 35->25 deg for why 25, not 35).
    """
    return v / (params.lf + params.lr) * delta_max_rad


def build_shaped_heading_profile(s, geometric_psi, v_target, params,
                                  authority_frac=HEADING_LEAD_AUTHORITY_FRAC):
    """
    Shape a heading-error REFERENCE profile that leads the path's own
    geometric tangent by an amount bounded by how much yaw the car can
    actually achieve between each station and the next, AT THAT STATION'S
    OWN v_target (the already-converged speed profile from
    _lap_time_and_speed, which itself already respects
    params.max_accel/max_accel_brake -- see this module's docstring
    section "THIS FILE'S v_target IS THE ONE THE CAR DRIVES"). A corner
    taken slower gets proportionally more lead per metre of arc length,
    since more time is available per metre at lower speed -- the
    speed-awareness requested explicitly (see
    late_turn_in_investigation.md Part 8).

    Does NOT feed back into path_xy or v_target -- this is a DERIVED pass,
    computed once after optimize_raceline() has already converged both (see
    Part 9's coupling-depth decision: derived pass first, full coupling
    only if this proves insufficient).

    Walks the arc BACKWARD from the end so each station's lead is capped by
    the achievable heading change to the NEXT station's already-computed
    target, compounding correctly across a multi-station ramp into a corner
    (verified in Part 8's synthetic testing: this naturally decays the lead
    to ~0 once a corner's constant-curvature section begins, since nothing
    is left to "pre-achieve" once the car is expected to already be
    mid-turn).

    Parameters
    ----------
    s : (n,) cumulative arc length (m), same convention as _lap_time_and_speed's segs.
    geometric_psi : (n,) the path's own atan2(dy,dx) tangent heading (rad) --
        i.e. what path_Psi already is today; NOT overwritten by this function.
    v_target : (n,) the ALREADY-CONVERGED speed profile from _lap_time_and_speed.
    params : VehicleParams
    authority_frac : float in (0, 1] -- see HEADING_LEAD_AUTHORITY_FRAC.

    Returns
    -------
    psi_target : (n,) shaped heading profile, same indexing as geometric_psi.
    """
    n = len(s)
    psi_target = geometric_psi.copy()
    for i in range(n - 2, -1, -1):
        ds = s[i + 1] - s[i]
        dt_seg = ds / max(v_target[i], 0.5)
        max_dpsi = max_yaw_rate(v_target[i], params) * authority_frac * dt_seg
        diff = psi_target[i + 1] - geometric_psi[i]
        step = float(np.clip(diff, -max_dpsi, max_dpsi))
        psi_target[i] = geometric_psi[i] + step
    return psi_target


def check_slip(kappa, v_target, params, slip_limit_rad=SLIP_LIMIT_RAD):
    """
    Flag stations where the path's own geometric turning rate (kappa*v,
    the yaw rate a car with zero slip would need to hold this line at this
    speed) implies more REAR-axle slip than slip_limit_rad, via the
    standard steady-state linear-bicycle relationship beta_r = lr*r/v --
    same Cf/Cr-model family alat_ceiling_at() already trusts for its own
    tyre-force reasoning (see VehicleParams), not a new tyre model.

    Diagnostic only (see SLIP_LIMIT_RAD's own comment) -- does not modify
    path_xy, v_target, or psi_target. A high slip_flags count on a real
    track is the evidence that would justify Part 9's "full coupling"
    alternative (letting slip reshape the path/speed themselves) instead of
    this derived-pass-only approach; a clean/near-empty result confirms the
    derived pass is sufficient and coupling is not yet needed.

    Returns
    -------
    slip_flags : (n,) bool
    beta_r : (n,) float (rad) -- the implied slip angle itself, for
        reporting magnitude, not just count.
    """
    r_implied = kappa * v_target
    beta_r = params.lr * r_implied / np.maximum(v_target, 0.5)
    slip_flags = np.abs(beta_r) > slip_limit_rad
    return slip_flags, beta_r


# Hard floor on cone clearance for an exported line (m). Below half the front
# track (VehicleParams.tf/2 = 0.625) the car's wheel is through the cone, so a
# line that gets this close is not merely tight, it is undrivable. Checked
# against the FINAL exported points -- the optimiser's own width bounds are
# computed per-station from a nearest-cone search that can miss a cone the
# smoothing pass later steers toward, so verifying the bounds is not the same
# as verifying the result.
MIN_CONE_CLEARANCE = 0.625


def _assert_clearance(path_xy, blue, yellow, margin):
    """
    Fail the export if the finished line passes closer to any cone than
    MIN_CONE_CLEARANCE. Raises RuntimeError; the caller does not catch it.

    Silent containment failure is the specific thing this prevents: an
    unchecked raceline that passes too close to a cone can be driven as
    exported, producing large peak tracking error and off-track excursions
    that get misattributed to controller tuning instead of the line itself.
    """
    cones = np.vstack([np.asarray(blue)[:, :2], np.asarray(yellow)[:, :2]])
    # Pairwise nearest distance; the point counts here (~1e3 x ~1e2) make the
    # dense computation cheaper than building a spatial index.
    d = np.hypot(path_xy[:, 0][:, None] - cones[None, :, 0],
                 path_xy[:, 1][:, None] - cones[None, :, 1]).min(axis=1)
    worst = float(d.min())
    n_bad = int((d < MIN_CONE_CLEARANCE).sum())
    if n_bad:
        raise RuntimeError(
            f"raceline passes within {worst:.3f} m of a cone at {n_bad} of "
            f"{len(path_xy)} stations (floor {MIN_CONE_CLEARANCE:.3f} m = half "
            f"the front track). The line is not drivable as exported.\n"
            f"  margin used: {margin:.2f} m -- raise --margin and re-run.\n"
            f"  NOTHING was written; the previous export is untouched."
        )
    return worst


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

    # Verify BEFORE opening the output file: a failed check must leave the
    # previous, known-good export in place rather than truncating it.
    worst_clearance = _assert_clearance(
        np.column_stack([path_X, path_Y]), blue, yellow, margin
    )

    # Derived third pass (see Part 9's coupling-depth decision): computed
    # AFTER path_X/path_Y/path_v have already converged above, does not
    # feed back into them. path_Psi (geometric tangent) is kept in the CSV
    # unchanged for anything that wants the true tangent (e.g. the live
    # controller's CornerMap curvature segmentation); psi_target is an
    # ADDITIONAL column, only consumed by the heading-error reference when
    # use_precomputed_heading_profile is enabled.
    segs = np.hypot(np.diff(path_X, append=path_X[0]), np.diff(path_Y, append=path_Y[0]))
    s = np.concatenate([[0.0], np.cumsum(segs)])[:-1]
    kappa_final = _curvature(np.column_stack([path_X, path_Y]))
    psi_target = build_shaped_heading_profile(s, path_Psi, path_v, params)
    slip_flags, beta_r = check_slip(kappa_final, path_v, params)

    with open(out_path, "w") as f:
        f.write("# x,y,psi,psi_target,v_target -- minimum-time racing line exported by "
                "tuner.tools.raceline_optimizer from a cone_recorder map.\n")
        f.write(f"# source_map={os.path.abspath(map_path)}\n")
        f.write(f"# centreline_lap_time_s={centre_time:.3f} raceline_lap_time_s={race_time:.3f} "
                f"improvement_pct={100.0 * (centre_time - race_time) / centre_time:.2f}\n")
        f.write("# psi_target: shaped heading-lead reference (see "
                "late_turn_in_investigation.md Part 8/9), NOT the geometric "
                "path tangent (psi). Backward-compatible: a loader reading "
                "only 4 columns (x,y,psi,v_target) still parses this file.\n")
        f.write("x,y,psi,psi_target,v_target\n")
        for x, y, psi, psit, v in zip(path_X, path_Y, path_Psi, psi_target, path_v):
            f.write(f"{x:.4f},{y:.4f},{psi:.5f},{psit:.5f},{v:.4f}\n")

    print(f"Wrote {len(path_X)} points -> {out_path}")
    print(f"centreline lap time (alat_ceiling-limited): {centre_time:.3f} s")
    print(f"raceline   lap time (alat_ceiling-limited): {race_time:.3f} s "
          f"({100.0 * (centre_time - race_time) / centre_time:.2f}% faster)")
    print(f"v_target range: {path_v.min():.2f} - {path_v.max():.2f} m/s")
    print(f"min cone clearance: {worst_clearance:.3f} m "
          f"(floor {MIN_CONE_CLEARANCE:.3f}, margin {margin:.2f})")
    lead_deg = np.degrees(np.abs(psi_target - path_Psi))
    print(f"heading lead: max {lead_deg.max():.2f} deg, mean {lead_deg.mean():.2f} deg "
          f"(authority_frac={HEADING_LEAD_AUTHORITY_FRAC})")
    n_slip = int(slip_flags.sum())
    if n_slip:
        print(f"SLIP CHECK (diagnostic, unvalidated limit {math.degrees(SLIP_LIMIT_RAD):.1f} deg): "
              f"{n_slip}/{len(slip_flags)} stations exceed it, "
              f"max |beta_r| {np.degrees(np.abs(beta_r)).max():.2f} deg")
    else:
        print(f"SLIP CHECK: 0/{len(slip_flags)} stations exceed the "
              f"{math.degrees(SLIP_LIMIT_RAD):.1f} deg diagnostic limit")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument(
        "map", nargs="?", default=None,
        help=f"Track name under tracks/ (default: {DEFAULT_TRACK}), or an "
             "explicit path to a cone_map.json.")
    ap.add_argument(
        "out_path", nargs="?", default=None,
        help="Output CSV (default: raceline.csv beside the source map).")
    ap.add_argument("--list", action="store_true",
                    help="List available tracks and exit.")
    ap.add_argument("--iters", type=int, default=DEFAULT_ITERS,
                     help="curvature-reduction iterations (default: %(default)s)")
    ap.add_argument("--margin", type=float, default=DEFAULT_MARGIN,
                     help="clearance kept from each boundary cone, metres (default: %(default)s)")
    args = ap.parse_args()

    if args.list:
        for name in list_tracks():
            print(name)
        return

    try:
        map_path = resolve_map_arg(args.map)
    except ValueError as e:
        ap.error(str(e))
    out_path = args.out_path or default_out_for(map_path, RACELINE_NAME)
    export(map_path, out_path, iters=args.iters, margin=args.margin)


if __name__ == "__main__":
    main()
