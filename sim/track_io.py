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
tracking mode), this module globally NN-sorts each boundary and NN-pairs them
into a single loop, the same primitives planning/path_utils.build_local_path
uses for its live per-tick window, but WITHOUT that function's forward-window
restriction — a static recorded map is the whole lap at once, not a live
partial view, so there is no "forward of the car" to restrict to.

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

import numpy as np
from scipy.interpolate import CubicSpline

from planning.cone_sorting import pair_cones_nn, sort_cones_nn
from planning.path_utils import compute_centreline, smooth_centreline
import sim.speed_profile as speed_profile

PATH_N_POINTS = 1000   # dense resample count, matches offline_tuner.PATH_N_POINTS default


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


def _reconstruct_centreline(blue: np.ndarray, yellow: np.ndarray) -> np.ndarray:
    """
    Build a single reference loop from a full recorded cone map.

    Globally NN-sorts each boundary from the origin (the recorded map's
    frame — cone_recorder writes cones in the same global frame car_position
    used during recording, so the origin is wherever the car started) then
    NN-pairs the two sorted boundaries into cone-pair midpoints, and smooths
    them into a dense centreline. See module docstring for why this doesn't
    reuse build_local_path (its forward-window filtering assumes a live
    partial view, not a complete static map).
    """
    if len(blue) < 2 or len(yellow) < 2:
        raise ValueError(
            f'Not enough cones to reconstruct a centreline (blue={len(blue)}, yellow={len(yellow)}); '
            'need at least 2 of each colour.'
        )

    blue_sorted   = sort_cones_nn(blue)
    yellow_sorted = sort_cones_nn(yellow)
    pairs = pair_cones_nn(blue_sorted, yellow_sorted)

    if len(pairs) < 2:
        raise ValueError(
            f'Only {len(pairs)} blue/yellow cone pairs matched within range — '
            'cannot reconstruct a centreline from this recording.'
        )

    raw = compute_centreline(pairs)
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
