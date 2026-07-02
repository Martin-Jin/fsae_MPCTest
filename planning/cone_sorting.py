"""
Cone colour separation, ordering, and filtering utilities.

All functions operate on (N, 2) float64 numpy arrays of cone positions in the
global ENU frame (x = forward, y = left).
"""
import math

import numpy as np

# from fs_msgs.msg import Cone

# Typical FS track width upper bound used to reject implausible pairs
MAX_PAIR_DIST = 7.0  # metres


# def separate_cones_by_color(track_msg):
    # """
    # Split a Track message into blue (left boundary) and yellow (right boundary)
    # cone arrays, discarding orange and unknown cones.
    # Returns (blue_cones, yellow_cones) as float64 arrays of shape (N, 2).
    # """
    # blue, yellow = [], []
    # for cone in track_msg.track:
    #     pt = [cone.location.x, cone.location.y]
    #     if cone.color == Cone.BLUE:
    #         blue.append(pt)
    #     elif cone.color == Cone.YELLOW:
    #         yellow.append(pt)
    # return np.array(blue, dtype=np.float64), np.array(yellow, dtype=np.float64)


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
