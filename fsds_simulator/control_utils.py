# Title: control_utils.py

"""
control_utils.py — Live Speed-Profile Helper for FSDS

PURPOSE
-------
Provides curvature_speed(), a self-contained (numpy/math only, no
cross-package imports) curvature-limited target-speed calculator. Used by
control_node.py to compute desired_speed locally from the current path each
control tick, instead of subscribing to a separately-published planner
speed topic.

HOW IT WORKS
------------
Given the upcoming path waypoints, curvature_speed() measures the peak path
curvature over a look-ahead window and returns
v_target = safety * sqrt(a_lat_max / kappa_peak), clipped to [v_min,
v_max_eff] — where v_max_eff itself scales v_max down if the visible path
is shorter than the look-ahead window (not enough road to justify full
speed). Deliberately mirrors sim_track.compute_speed_profile()'s corner
logic so the live desired_speed behaves like the offline
simulator/tuner's, but is a standalone re-implementation (see USED BY).

USED BY
-------
  control_node.py — ControlNode._control_loop calls curvature_speed(path,
                    v_max=V_MAX, v_min=V_MIN) every 20 Hz tick to source
                    MPCController.compute()'s desired_speed argument.
"""

import math

import numpy as np


def curvature_speed(waypoints, v_max=15.0, v_min=1.5, a_lat_max=4.0,
                     scan_start=1.5, scan_end=14.0, step=2.0, safety=1.0):
    """
    Curvature-limited target speed over the next scan_end metres of the path.

    Ported from the planner's speed logic so the controller can set its own
    desired_speed without a cross-package import. v_target = safety *
    sqrt(a_lat_max / kappa_peak), with a short-path cap that scales v_max
    down when the visible path is shorter than scan_end. waypoints[0] is
    assumed to be the car's current position.
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
    # published path carries a few cm of per-point lateral wiggle. Computing
    # the MAX Menger curvature over raw ~2 m triples turns that noise into a
    # large spurious kappa (true kappa is ~0 on a straight, so the max is
    # pure noise), collapsing v_target and making it oscillate frame-to-frame
    # — the "rapid accel/decel" on straights. Fix at the source: densely
    # resample the scan window and moving-average denoise it before
    # measuring curvature. A real corner is a sustained bend that survives
    # the ~5 m smoothing; only the cm-scale wiggle is removed (this also
    # stops noise from over-slowing corners).
    pts = None
    dense = np.arange(scan_start, hi, 1.0)
    if len(dense) >= 7:                       # room to smooth and still leave >=3 triples
        dx = np.interp(dense, arc, waypoints[:, 0])
        dy = np.interp(dense, arc, waypoints[:, 1])
        w  = min(5, len(dense) - 4)           # 'valid' conv keeps len-w+1 >= 3 points
        ker = np.ones(w) / w
        sx = np.convolve(dx, ker, mode='valid')
        sy = np.convolve(dy, ker, mode='valid')
        pts = np.column_stack([sx, sy])[::2]  # back to ~2 m spacing for the triples
    if pts is None or len(pts) < 3:
        # Short scan window: no headroom to denoise — fall back to coarse sampling.
        sample_arcs = np.arange(scan_start, hi, step)
        if len(sample_arcs) < 3:
            return float(v_max_eff)
        sx  = np.interp(sample_arcs, arc, waypoints[:, 0])
        sy  = np.interp(sample_arcs, arc, waypoints[:, 1])
        pts = np.column_stack([sx, sy])

    max_kappa = 0.0
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
        kappa = 2.0 * cross / denom
        if kappa > max_kappa:
            max_kappa = kappa

    if max_kappa < 1e-4:
        return float(v_max_eff)

    v_target = safety * math.sqrt(a_lat_max / max_kappa)
    return float(max(v_min, min(v_max_eff, v_target)))
