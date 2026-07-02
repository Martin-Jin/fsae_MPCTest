"""
Persistent cone map — accumulates cone observations across the full session.

FSDS publishes cone positions in the global ENU map frame (same frame as
odometry), so no coordinate transform is required.  Each call to update()
merges new detections into the running map: an observation within MERGE_DIST
metres of an existing entry updates that entry's position (running average)
rather than adding a duplicate.  Cones that leave the sensor FOV are never
removed, so the map grows monotonically and historical walls are preserved.
"""
import numpy as np

MERGE_DIST = 0.8   # metres — two detections closer than this → same cone


class ConeMap:
    """
    Accumulates blue and yellow cone observations over the full session.

    Usage
    -----
    map = ConeMap()
    map.update(blue_obs, yellow_obs)   # called on each /FusionCones message
    plan_with(map.blue, map.yellow)    # always returns the full accumulated set
    """

    def __init__(self) -> None:
        self._blue:   np.ndarray = np.empty((0, 2), dtype=np.float64)
        self._yellow: np.ndarray = np.empty((0, 2), dtype=np.float64)

    def update(self, blue_obs: np.ndarray, yellow_obs: np.ndarray) -> None:
        """Merge new observations into the accumulated map."""
        self._blue   = _absorb(self._blue,   blue_obs)
        self._yellow = _absorb(self._yellow, yellow_obs)

    def reset(self) -> None:
        self._blue   = np.empty((0, 2), dtype=np.float64)
        self._yellow = np.empty((0, 2), dtype=np.float64)

    @property
    def blue(self) -> np.ndarray:
        return self._blue

    @property
    def yellow(self) -> np.ndarray:
        return self._yellow


def _absorb(store: np.ndarray, obs: np.ndarray) -> np.ndarray:
    """
    Merge obs array into store:
    - points within MERGE_DIST of an existing entry → update that entry
      (running average to correct noisy repeated detections)
    - points beyond MERGE_DIST of all existing entries → appended as new
    """
    if len(obs) == 0:
        return store
    if len(store) == 0:
        return obs.copy()

    store = store.copy()
    new_pts: list[np.ndarray] = []

    for pt in obs:
        dists = np.linalg.norm(store - pt, axis=1)
        best  = int(np.argmin(dists))
        if dists[best] < MERGE_DIST:
            store[best] = (store[best] + pt) * 0.5   # running mean
        else:
            new_pts.append(pt)

    if new_pts:
        store = np.vstack([store, np.array(new_pts, dtype=np.float64)])

    return store
