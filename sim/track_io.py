"""
sim/track_io.py — Load a recorded cone map into this repo's path/track format.

PURPOSE
-------
fsae_planning's fsae_sim_perception.cone_recorder node records one lap's worth
of accumulated boundary cones from a live FSDS run and writes them to a JSON
file (see that node's module docstring for the exact format). This module
turns such a file into the same (path_X, path_Y, path_Psi, path_v, blue,
yellow) tuple that tuner/offline_tuner.SYNTHETIC_PATHS stores per synthetic
path, so a recorded real track can be loaded into gui/simulation.py alongside
the synthetic library and re-simulated exactly like any other path.

RECONSTRUCTION APPROACH
------------------------
A recorded cone map has no path — only cones. To get a reference centreline
(needed for path_v_profile, camera framing, and the oracle/use_planner=False
tracking mode), this module marches a virtual car around the whole lap,
calling planning/boundary.build_path_walls() at each step exactly as the live
SimPlanner does per tick, and stitches the near-field chunk each call returns
into one continuous loop.

A single global nearest-neighbour sort+pair over all cones at once (the
approach this used to take) breaks down wherever the lap crosses near itself
(a figure-eight/pinch): cones from two different, spatially-close-but-
topologically-distant legs get sorted/paired together, producing a centreline
that cuts across the track. build_path_walls() avoids this because it only
ever looks at a local window around a car pose and penalises steps that cross
the existing cone-wall mesh — the same reason it's safe to use live. Walking
that local planner around the recorded lap reuses that safety for the static
reconstruction too, keeping this path consistent with what SimPlanner actually
drives (see CLAUDE.md: sim and live planning must stay numerically identical).

This reconstructed centreline is a fallback/reference path only. When the GUI
loads a recorded track with use_planner=True (the normal mode — see
settings.USE_PLANNER), the actual driving line still comes from SimPlanner
rebuilding it cone-by-cone during the rollout, exactly as for a synthetic
path; this module's reconstruction is only what oracle-mode tracking and the
initial camera framing use.

Does not require tuner/offline_tuner.py (which imports the optional `cma`
dependency) — the resampling here is a standalone copy of that module's
_resample_path(), using only planning/ and sim/speed_profile.py.
"""
import json
import math

import numpy as np
from scipy.interpolate import CubicSpline

from planning.boundary import build_path_walls
from planning.path_utils import smooth_centreline
import sim.speed_profile as speed_profile

PATH_N_POINTS = 1000   # dense resample count, matches offline_tuner.PATH_N_POINTS default
# Arc-length advanced per build_path_walls() call while marching the virtual
# car around the recorded lap.  Small enough that the accepted chunk of each
# call's centreline (see _reconstruct_centreline) stays within its near-field,
# well before the horizon where the windowed planner's chain becomes unreliable.
_MARCH_STEP = 3.0
# Hard cap on march steps — a safety backstop against an unexpected non-closing
# loop (e.g. a malformed recording) spinning forever; a real lap closes in a
# few hundred steps at most given _MARCH_STEP.
_MARCH_MAX_STEPS = 2000
# Distance to the seed position below which the march is considered to have
# completed the loop and closed back on its start.
_MARCH_CLOSE_DIST = 3.0
# Minimum arc-length marched before loop-closure is checked, so the march
# doesn't immediately "close" one step after starting.
_MARCH_MIN_ARC = 20.0
# Fraction of each colour's cones that must have been passed (see _MARCH_VISIT_DIST)
# before proximity to the seed is allowed to close the loop. A track that
# crosses itself (a figure-eight) passes near the start's neighbourhood again
# mid-lap, before the far lobe has been driven — closing on proximity alone
# there would cut the lap short. Requiring near-full coverage first means the
# march only stops once it has actually been all the way around, at the cost
# of not tolerating an unrecorded gap larger than (1 - this) of the cones.
_MARCH_MIN_COVERAGE = 0.9
# Distance within which a cone counts as "passed" by an accepted march chunk.
_MARCH_VISIT_DIST = 4.0
# Minimum straight-line gap to leave between the reconstructed path's first
# and last point (see the tail-trim in _reconstruct_centreline). A recorded
# lap is a closed loop, so without any trim the march's last point sits right
# on top of its first — rollout_core.find_closest_reference_bounded() (a
# forward-bounded nearest-index search) would then immediately snap idx to
# the array's tail on step one, since the closest point to a near-duplicate
# start IS the end. This just needs to be enough to give idx somewhere
# unambiguous to start; it does not need to (and in general cannot, on a lap
# that runs close to itself elsewhere) guarantee the tail is clear of every
# other point on the lap — see rollout_core.run_core_rollout's near_end-gated
# finish check for how that is actually handled.
_MARCH_TAIL_GAP = 5.0


def load_cone_map(json_path: str) -> tuple[np.ndarray, np.ndarray]:
    """
    Read a cone_recorder JSON file and return (blue, yellow) as (N, 2) arrays.

    Raises FileNotFoundError / json.JSONDecodeError on a missing/malformed
    file — deliberately not caught here, since a silent empty-map fallback
    would let a bad path load an empty track without any error.
    """
    with open(json_path, 'r') as f:
        payload = json.load(f)
    blue   = np.array(payload.get('blue', []),   dtype=np.float64).reshape(-1, 2)
    yellow = np.array(payload.get('yellow', []), dtype=np.float64).reshape(-1, 2)
    return blue, yellow


def _seed_pose(blue: np.ndarray, yellow: np.ndarray) -> tuple[np.ndarray, float]:
    """
    Estimate a starting (car_pos, car_yaw) for the march, from the map origin
    (the recorded map's frame — cone_recorder writes cones in the same global
    frame car_position used during recording, so the origin is wherever the
    car started).

    car_pos is the midpoint of the nearest blue and nearest yellow cone to the
    origin (approximates the start line). The two cones flanking a start/
    finish line are typically almost equidistant in BOTH directions along the
    track, so picking car_yaw from "whichever neighbour is marginally closer"
    is a near coin-flip that can point the march backwards around the loop —
    silently reversing the whole reconstructed path's direction of travel
    (and the car's starting orientation on the static preview, and the pose
    handed to the MPC rollout). Instead, try both opposite headings and keep
    whichever one build_path_walls() actually extends forward from — the same
    planner the march itself uses, so "is this direction drivable" is judged
    by the real cone-wall/topology check rather than a raw-distance guess.
    """
    origin = np.zeros(2)
    b0 = blue[int(np.argmin(np.linalg.norm(blue - origin, axis=1)))]
    y0 = yellow[int(np.argmin(np.linalg.norm(yellow - origin, axis=1)))]
    start = (b0 + y0) * 0.5

    b_dists = np.linalg.norm(blue - b0, axis=1)
    y_dists = np.linalg.norm(yellow - y0, axis=1)
    b_dists[np.argmin(b_dists)] = np.inf
    y_dists[np.argmin(y_dists)] = np.inf
    b1 = blue[int(np.argmin(b_dists))]
    y1 = yellow[int(np.argmin(y_dists))]
    ahead = (b1 + y1) * 0.5

    direction = ahead - start
    if np.linalg.norm(direction) < 1e-6:
        return start, 0.0
    yaw = math.atan2(direction[1], direction[0])

    def _forward_reach(candidate_yaw):
        cl, _, _, _ = build_path_walls(blue, yellow, start, candidate_yaw)
        if cl is None or len(cl) < 2:
            return 0.0
        return float(np.sum(np.linalg.norm(np.diff(cl, axis=0), axis=1)))

    if _forward_reach(yaw + math.pi) > _forward_reach(yaw):
        yaw += math.pi

    return start, yaw


def _reconstruct_centreline(blue: np.ndarray, yellow: np.ndarray) -> np.ndarray:
    """
    Build a single reference loop from a full recorded cone map by marching a
    virtual car around the whole lap, exactly as sim.sim_track.SimPlanner does
    per tick while actually driving. See module docstring for why this — not a
    single global nearest-neighbour sort+pair — is needed to survive a lap that
    crosses near itself.

    Each step calls planning.boundary.build_path_walls() at the current virtual
    pose (using the *entire* recorded map, matching SimPlanner's fully-
    accumulated ConeMap), keeps only the near-field chunk of the returned
    centreline up to _MARCH_STEP metres of arc-length, appends it to the loop,
    and advances the virtual pose to the chunk's end (position + heading of the
    last segment).

    Stops once the march is back near its own start AND has covered at least
    _MARCH_MIN_COVERAGE of both colours' cones (see _MARCH_MIN_COVERAGE) — a
    track that crosses itself passes near the start's neighbourhood again
    mid-lap, before the far side of the crossing has been driven, so proximity
    to the seed alone is not sufficient to detect a genuine lap completion.
    """
    if len(blue) < 2 or len(yellow) < 2:
        raise ValueError(
            f'Not enough cones to reconstruct a centreline (blue={len(blue)}, yellow={len(yellow)}); '
            'need at least 2 of each colour.'
        )

    seed_pos, car_yaw = _seed_pose(blue, yellow)
    car_pos = seed_pos

    loop = [car_pos]
    total_arc = 0.0
    blue_visited   = np.zeros(len(blue), dtype=bool)
    yellow_visited = np.zeros(len(yellow), dtype=bool)

    def _mark_visited(pt):
        blue_visited[np.linalg.norm(blue - pt, axis=1) < _MARCH_VISIT_DIST]     = True
        yellow_visited[np.linalg.norm(yellow - pt, axis=1) < _MARCH_VISIT_DIST] = True

    _mark_visited(car_pos)

    for _ in range(_MARCH_MAX_STEPS):
        cl, _, _, _ = build_path_walls(blue, yellow, car_pos, car_yaw)
        if cl is None or len(cl) < 2:
            break

        seg = np.linalg.norm(np.diff(cl, axis=0), axis=1)
        arc = np.concatenate([[0.0], np.cumsum(seg)])
        cut = int(np.searchsorted(arc, _MARCH_STEP)) + 1
        cut = min(cut, len(cl) - 1)
        if cut < 1:
            break

        chunk = cl[1:cut + 1]   # drop cl[0], which duplicates the car anchor
        if len(chunk) == 0:
            break

        loop.extend(chunk)
        total_arc += float(arc[cut])
        for pt in chunk:
            _mark_visited(pt)

        step_dir = chunk[-1] - car_pos
        step_len = float(np.linalg.norm(step_dir))
        if step_len < 1e-6:
            break
        car_yaw = math.atan2(step_dir[1], step_dir[0])
        car_pos = chunk[-1]

        coverage = min(blue_visited.mean(), yellow_visited.mean())
        if (total_arc > _MARCH_MIN_ARC
                and coverage >= _MARCH_MIN_COVERAGE
                and float(np.linalg.norm(car_pos - seed_pos)) < _MARCH_CLOSE_DIST):
            break

    raw = np.array(loop, dtype=np.float64)
    if len(raw) < 2:
        raise ValueError(
            'Could not march a centreline around this recording — '
            'build_path_walls produced no usable path from the seed pose.'
        )

    # A recorded lap is a closed loop, so the march above stops once it has
    # come back within _MARCH_CLOSE_DIST of its own start — meaning raw[-1]
    # sits right next to raw[0]. Trim the tail back to the last point that is
    # still _MARCH_TAIL_GAP clear of the start, so the path's own two ends are
    # unambiguous (see _MARCH_TAIL_GAP for why, and run_core_rollout's
    # near_end-gated finish check for how a lap that runs close to itself
    # elsewhere is handled — this trim does not need to solve that).
    dist_to_seed = np.linalg.norm(raw - seed_pos, axis=1)
    clear_of_start = np.where(dist_to_seed > _MARCH_TAIL_GAP)[0]
    if len(clear_of_start) > 1:
        raw = raw[:clear_of_start[-1] + 1]

    return smooth_centreline(raw, n_out=max(20, len(raw) * 5), pin_start=False)


def _resample_dense(waypoints_x, waypoints_y, n_points=PATH_N_POINTS):
    """
    Fit a clamped cubic spline through sparse waypoints and resample to
    n_points, computing heading and speed profile.

    Standalone copy of tuner/offline_tuner._resample_path()'s spline-fit
    logic — kept here rather than imported so loading a recorded track does
    not require offline_tuner's optional `cma` dependency. Keep this in sync
    with _resample_path() if that function's spline/profile logic changes.

    Returns (path_X, path_Y, path_Psi, path_v) — see _resample_path()'s own
    docstring for the shape/meaning of each.
    """
    wx = np.asarray(waypoints_x, dtype=float)
    wy = np.asarray(waypoints_y, dtype=float)
    t = np.linspace(0.0, 1.0, len(wx))

    d0 = np.array([wx[1] - wx[0], wy[1] - wy[0]]) / (t[1] - t[0])
    dN = np.array([wx[-1] - wx[-2], wy[-1] - wy[-2]]) / (t[-1] - t[-2])

    cs_x = CubicSpline(t, wx, bc_type=((1, d0[0]), (1, dN[0])))
    cs_y = CubicSpline(t, wy, bc_type=((1, d0[1]), (1, dN[1])))

    t_fine = np.linspace(0.0, 1.0, n_points)
    path_X = cs_x(t_fine)
    path_Y = cs_y(t_fine)
    dx = cs_x.derivative()(t_fine)
    dy = cs_y.derivative()(t_fine)
    path_Psi = np.arctan2(dy, dx)

    raw_v = speed_profile.compute_speed_profile(path_X, path_Y)
    path_v = speed_profile.smooth_profile(raw_v, window=9)

    return path_X, path_Y, path_Psi, path_v


def load_recorded_track(json_path: str, n_points: int = PATH_N_POINTS):
    """
    Load a cone_recorder JSON file into the SYNTHETIC_PATHS-shaped tuple.

    Returns
    -------
    (path_X, path_Y, path_Psi, path_v, blue, yellow) : tuple
        Same shape as tuner/offline_tuner.SYNTHETIC_PATHS[name] and
        sim/sim_track.place_cones()'s output, so callers can use a recorded
        track everywhere a synthetic path is currently accepted — e.g.
        gui/simulation.py's load_test_path()/on_release() assign these same
        six values.  blue/yellow are the RECORDED cones (not re-placed via
        place_cones()) — they are real perception data, not synthetic
        geometry, so they're passed through unchanged.

    Raises
    ------
    FileNotFoundError, json.JSONDecodeError : bad json_path.
    ValueError : too few cones/pairs to reconstruct a centreline
                 (see _reconstruct_centreline).
    """
    blue, yellow = load_cone_map(json_path)
    centreline = _reconstruct_centreline(blue, yellow)
    path_X, path_Y, path_Psi, path_v = _resample_dense(
        centreline[:, 0], centreline[:, 1], n_points=n_points
    )
    return path_X, path_Y, path_Psi, path_v, blue, yellow
