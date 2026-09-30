"""
model/vehicle_physics/tracking.py — reference-path lookup and tracking error

PURPOSE
-------
Geometry helpers that relate a plant state to the reference path: bounded
nearest-point search, interpolated reference point, and the tracking-error
tuple (e_y, e_psi, ...) the controller consumes.

USED BY
-------
  gui/simulation.py, tuner/offline_tuner.py and the offline rollout.
"""
import numpy as np

from model.vehicle_physics.state import (
    IDX_X,
    IDX_Y,
    IDX_PSI,
    IDX_VX,
    IDX_VY,
    IDX_R,
    IDX_DELTA,
    IDX_A_ACT,
)


# ─────────────────────────────────────────────────────────────────────────────
# TRACKING ERROR HELPER
# ─────────────────────────────────────────────────────────────────────────────
def find_closest_reference_bounded(path_X, path_Y, path_Psi, x_g, y_g, last_idx, window=40):
    """
    Find the closest point on the reference path (path_X, path_Y) to the
    given global position, searching within a bounded window around last_idx.

    The windowed search prevents the tracker from jumping backward on paths
    that double back on themselves (e.g. after a hairpin). At the start of a
    simulation (last_idx ≤ 5), a wider initial window prevents the tracker
    from locking onto index 0 if the vehicle has already moved forward.

    This function reads the module-level path_X, path_Y, path_Psi arrays.

    Parameters
    ----------
    x_g, y_g : float   Vehicle global position (m).
    last_idx : int      Previously found closest index (search anchor).
    window : int        Forward search range in path indices. Default 40.

    Returns
    -------
    (global_idx, ref_x, ref_y, ref_psi) : (int, float, float, float)
        Index and coordinates of the nearest path point, plus path heading there.

    Called by: simulate_closed_loop() — fallback path when SimPlanner has no centreline,
               and for path-end detection (idx ≥ len(path_X) - 2)
    """
    if last_idx <= 5:
        start_search = 0
        end_search   = min(len(path_X), 100)   # Wide initial window
    else:
        start_search = max(0, last_idx - 5)
        end_search   = min(len(path_X), last_idx + window)

    distances  = np.hypot(
        path_X[start_search:end_search] - x_g,
        path_Y[start_search:end_search] - y_g,
    )
    local_idx  = np.argmin(distances)
    global_idx = start_search + local_idx

    return global_idx, path_X[global_idx], path_Y[global_idx], path_Psi[global_idx]


def get_interpolated_ref_point(x, y, path_x, path_y, path_psi):
    """
    Computes a smooth, continuous reference point on the path via linear interpolation.

    To eliminate discontinuous "ballooning" errors caused by snapping to discrete path
    nodes, this function projects the vehicle's position onto the line segment between
    the two closest path points. This generates a "virtual" reference point (rx, ry, rpsi)
    that allows for continuous, smooth error estimation, significantly reducing noise
    in the tracking error signal.

    Parameters
    ----------
    x, y : float
        Current vehicle global coordinates (m).
    path_x, path_y : np.ndarray
        Reference path coordinates.
    path_psi : np.ndarray
        Reference path heading at each point (rad).

    Returns
    -------
    ref_x, ref_y : float
        The interpolated global coordinates on the path closest to the vehicle.
    ref_psi : float
        The interpolated heading angle at the projected point (rad).
    """
    # Find squared distance to all points
    dist_sq = (path_x - x)**2 + (path_y - y)**2
    idx = np.argmin(dist_sq)

    # If at the very start or end, return the closest point
    if idx <= 0 or idx >= len(path_x) - 1:
        return path_x[idx], path_y[idx], path_psi[idx]

    # Use the closest point and the one ahead/behind it (whichever is closer)
    # We define vectors along the path
    p_prev = np.array([path_x[idx-1], path_y[idx-1]])
    p_curr = np.array([path_x[idx], path_y[idx]])
    p_next = np.array([path_x[idx+1], path_y[idx+1]])

    # Choose the segment [p_prev, p_curr] or [p_curr, p_next]
    # We pick the one that the vehicle is closer to
    dist_to_prev = np.hypot(x - p_prev[0], y - p_prev[1])
    dist_to_next = np.hypot(x - p_next[0], y - p_next[1])
    
    if dist_to_prev < dist_to_next:
        p1, p2 = p_prev, p_curr
        psi1, psi2 = path_psi[idx-1], path_psi[idx]
    else:
        p1, p2 = p_curr, p_next
        psi1, psi2 = path_psi[idx], path_psi[idx+1]

    # Project vehicle onto line segment p1->p2 to find interpolation factor 't'
    v = p2 - p1
    w = np.array([x, y]) - p1
    t = np.clip(np.dot(w, v) / np.dot(v, v), 0, 1)

    # Linear Interpolate
    ref_x = p1[0] + t * v[0]
    ref_y = p1[1] + t * v[1]
    
    # Angular Interpolation (normalise angle difference)
    d_psi = (psi2 - psi1 + np.pi) % (2 * np.pi) - np.pi
    ref_psi = psi1 + t * d_psi

    return ref_x, ref_y, ref_psi


def plant_to_tracking_error(state, ref_x=None, ref_y=None, ref_psi=None, 
                            path_x=None, path_y=None, path_psi=None):
    """
    Computes tracking error. If path_x/y/psi are provided, it interpolates 
    the reference point for smoother cornering.
    """
    # 1. Resolve Reference Point (Interpolated or Provided)
    if path_x is not None:
        ref_x, ref_y, ref_psi = get_interpolated_ref_point(
            state[IDX_X], state[IDX_Y], path_x, path_y, path_psi
        )
    
    # 2. Extract State
    X, Y, psi  = state[IDX_X], state[IDX_Y], state[IDX_PSI]
    vx, vy     = state[IDX_VX], state[IDX_VY]
    r          = state[IDX_R]
    delta_act  = state[IDX_DELTA]
    a_act      = state[IDX_A_ACT]

    # 3. Calculate Errors
    dx = X - ref_x
    dy = Y - ref_y
    
    # Lateral error (signed distance)
    e_y_proj = dy * np.cos(ref_psi) - dx * np.sin(ref_psi)
    e_y = e_y_proj  # Keep sign intact directly
    
    # Heading error
    e_psi = np.arctan2(np.sin(psi - ref_psi), np.cos(psi - ref_psi))

    # Velocities
    e_y_dot = vx * np.sin(e_psi) + vy * np.cos(e_psi)
    e_psi_dot = r 

    return e_y, e_y_dot, e_psi, e_psi_dot, delta_act, a_act, vx
