"""
Cone colour separation, ordering, and filtering utilities.

All functions operate on (N, 2) float64 numpy arrays of cone positions in the
global ENU frame (x = forward, y = left).
"""
import math

import numpy as np

# Typical FS track width upper bound used to reject implausible pairs
MAX_PAIR_DIST = 7.0  # metres


def sort_cones_nn(cones, start=None):
    """
    Order a (N, 2) cone array into track sequence using a greedy nearest-neighbour
    walk starting from the cone closest to `start` (default: map origin = car start).
    Returns a reordered (N, 2) array.
    """
    if len(cones) == 0:
        return cones.copy()

    if start is None:
        start = np.zeros(2)

    remaining = list(range(len(cones)))
    seed = int(np.argmin(np.linalg.norm(cones - start, axis=1)))
    remaining.remove(seed)
    ordered = [seed]

    while remaining:
        last = cones[ordered[-1]]
        dists = np.linalg.norm(cones[remaining] - last, axis=1)
        nearest = remaining[int(np.argmin(dists))]
        ordered.append(nearest)
        remaining.remove(nearest)

    return cones[ordered]


def pair_cones_nn(left_cones, right_cones, max_dist=MAX_PAIR_DIST):
    """
    Match each left cone (sorted track order) to its nearest unpaired right cone
    within max_dist metres.
    Returns a list of (left_pt, right_pt) pairs as 1-D float64 arrays.
    """
    if len(left_cones) == 0 or len(right_cones) == 0:
        return []

    right_remaining = list(range(len(right_cones)))
    pairs = []

    for lc in left_cones:
        if not right_remaining:
            break
        candidates = right_cones[right_remaining]
        dists = np.linalg.norm(candidates - lc, axis=1)
        best_local = int(np.argmin(dists))
        if dists[best_local] <= max_dist:
            pairs.append((lc, right_cones[right_remaining[best_local]]))
            right_remaining.pop(best_local)

    return pairs


def filter_cones_forward(cones, car_pos, car_yaw,
                          min_ahead=0.5, max_ahead=25.0, max_lateral=6.0):
    """Return cones within the car's forward window."""
    if len(cones) == 0:
        return cones.copy()
    cos_y = math.cos(car_yaw)
    sin_y = math.sin(car_yaw)
    rel = cones - car_pos
    x_car =  rel[:, 0] * cos_y + rel[:, 1] * sin_y
    y_car = -rel[:, 0] * sin_y + rel[:, 1] * cos_y
    mask = (x_car > min_ahead) & (x_car < max_ahead) & (np.abs(y_car) < max_lateral)
    return cones[mask]


def filter_cones_window(cones, car_pos, car_yaw, radius=18.0,
                        min_ahead=0.5, max_ahead=25.0, max_lateral=8.0):
    """
    Keep cones within `radius` of the car (omni-directional) OR inside the
    forward preview box.

    A heading-aligned forward box alone truncates the path at corners: the track
    curves out of the box, so the cones around the bend are dropped even though
    the map still holds them.  The radius keeps those corner cones (they stay
    close to the car), while the box preserves long-range preview straight ahead
    on straights.  The union of the two is corner-robust without losing reach.
    """
    if len(cones) == 0:
        return cones.copy()
    rel  = cones - car_pos
    dist = np.hypot(rel[:, 0], rel[:, 1])
    cos_y = math.cos(car_yaw)
    sin_y = math.sin(car_yaw)
    x_car =  rel[:, 0] * cos_y + rel[:, 1] * sin_y
    y_car = -rel[:, 0] * sin_y + rel[:, 1] * cos_y
    box  = (x_car > min_ahead) & (x_car < max_ahead) & (np.abs(y_car) < max_lateral)
    return cones[(dist < radius) | box]
