"""
planning/geometry.py — segment-intersection helpers shared by boundary.py
and path_utils.py.

Split out so both modules can import it at module scope with no cycle:
boundary.py imports from path_utils.py already, so path_utils.py needing
segment_crosses_walls (used only inside build_local_path's tangent-entry
reachability check) used to require a function-local import to avoid the
cycle. Pure geometry with no dependency on either module, so it moves
cleanly.
"""
import numpy as np


def segment_crosses_walls(
    p1: np.ndarray,
    p2: np.ndarray,
    wall_segs: list[tuple[np.ndarray, np.ndarray]],
) -> bool:
    """True if the segment p1→p2 crosses any cone-wall segment."""
    return any(_seg_intersect(p1, p2, w1, w2) for (w1, w2) in wall_segs)


def _seg_intersect(
    a1: np.ndarray, a2: np.ndarray,
    b1: np.ndarray, b2: np.ndarray,
) -> bool:
    """True if segment a1→a2 properly intersects segment b1→b2 (endpoints excluded)."""
    d1 = a2 - a1
    d2 = b2 - b1
    denom = float(d1[0] * d2[1] - d1[1] * d2[0])
    if abs(denom) < 1e-10:
        return False
    diff = b1 - a1
    t = float(diff[0] * d2[1] - diff[1] * d2[0]) / denom
    u = float(diff[0] * d1[1] - diff[1] * d1[0]) / denom
    return 0.0 < t < 1.0 and 0.0 < u < 1.0
