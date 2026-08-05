"""
One-lap cone map recorder (logging / offline-resimulation aid).

Accumulates blue/yellow boundary-cone observations for the duration of one
lap, then writes the accumulated map to a JSON file and (optionally) shuts
itself down. This is NOT part of the planning/control pipeline: it is a
standalone logging node you launch alongside a normal run to capture what the
car actually saw, for later replay outside FSDS (see
fsae_MPCTest/sim/track_io.py, which reads the file this node writes).

Deliberately self-contained: the merge-and-deduplicate logic below is a
straight copy of fsae_planning's own planning/cone_map.py (ConeMap/_absorb),
not an import of it — this package (fsae_sim_perception) has no dependency on
the fsae_planning package/interfaces beyond fsae_interfaces, and adding a
cross-ament_python-package Python import is more coupling than a ~20-line
merge routine is worth. If planning/cone_map.py's merge behaviour changes,
update this copy too.

    in   /fsae/slam/left_track    fsae_interfaces/Track     blue (left) boundary, global frame
    in   /fsae/slam/right_track   fsae_interfaces/Track     yellow (right) boundary, global frame
    in   /fsae/slam/car_position  geometry_msgs/PoseStamped x,y in position; yaw in orientation.w
    in   /fsds/signal/go          fs_msgs/GoSignal          race start -> begins recording

Subscribing directly to the left/right boundary-track topics (rather than
depending on centerline_planner.py's own accumulated map) keeps this node
independent of which planner — or none — is running; it works the same way
during a manual/Stanley characterisation lap as it does with the centreline
planner active.

LAP DETECTION
-------------
Recording starts on the first GO signal. It stops (and the map is dumped) the
first time the car returns within `close_dist` of its start pose AFTER having
first travelled at least `min_lap_dist` away from it — the min-distance gate
stops the check from firing immediately at the start line before the car has
gone anywhere. If `max_record_time` elapses first, the map is dumped anyway
(so a run that never returns to the start pose, e.g. a DNF, still produces a
file) and the node logs that the stop was time-based, not a clean lap.

OUTPUT FORMAT
-------------
JSON, written to `out_path` (default: ~/fsae_logs/cone_map_<timestamp>.json):
    {
      "blue":   [[x, y], ...],
      "yellow": [[x, y], ...],
      "source": "fsae_sim_perception.cone_recorder",
      "lap_closed": true | false
    }
"""
import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node

from fs_msgs.msg import GoSignal
from fsae_interfaces.msg import Track
from geometry_msgs.msg import PoseStamped

MERGE_DIST = 0.8   # metres — two detections closer than this → same cone (see planning/cone_map.py)


def _cones_to_array(cones) -> np.ndarray:
    """geometry_msgs/Point[] → (N, 2) float64 array of x,y."""
    if not cones:
        return np.empty((0, 2))
    return np.array([[p.x, p.y] for p in cones], dtype=np.float64)


def _absorb(store: np.ndarray, obs: np.ndarray) -> np.ndarray:
    """
    Merge obs into store: points within MERGE_DIST of an existing entry update
    that entry (running average); points beyond MERGE_DIST are appended as new.
    Copy of planning/cone_map.py's _absorb() — see module docstring.
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
            store[best] = (store[best] + pt) * 0.5
        else:
            new_pts.append(pt)

    if new_pts:
        store = np.vstack([store, np.array(new_pts, dtype=np.float64)])

    return store


class ConeRecorder(Node):
    def __init__(self):
        super().__init__('cone_recorder')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('out_path', ''),           # '' -> ~/fsae_logs/cone_map_<timestamp>.json
                ('min_lap_dist', 8.0),       # m — must travel this far from start before lap-close checks arm
                ('close_dist', 3.0),         # m — distance to start pose that counts as "returned"
                ('max_record_time', 300.0),  # s — safety cap so a run that never closes the lap still dumps
                ('shutdown_on_save', True),  # exit the node once the map is written
            ],
        )
        self._min_lap_dist    = self.get_parameter('min_lap_dist').get_parameter_value().double_value
        self._close_dist      = self.get_parameter('close_dist').get_parameter_value().double_value
        self._max_record_time = self.get_parameter('max_record_time').get_parameter_value().double_value
        self._shutdown_on_save = self.get_parameter('shutdown_on_save').get_parameter_value().bool_value

        out_path = self.get_parameter('out_path').get_parameter_value().string_value
        if not out_path:
            log_dir = os.path.expanduser('~/fsae_logs')
            os.makedirs(log_dir, exist_ok=True)
            out_path = os.path.join(log_dir, f'cone_map_{int(time.time())}.json')
        self._out_path = out_path

        self.create_subscription(Track, '/fsae/slam/left_track',   self._left_cb,  10)
        self.create_subscription(Track, '/fsae/slam/right_track',  self._right_cb, 10)
        self.create_subscription(PoseStamped, '/fsae/slam/car_position', self._pose_cb,  10)
        self.create_subscription(GoSignal, '/fsds/signal/go',      self._go_cb,    10)

        self._blue_map:   np.ndarray = np.empty((0, 2), dtype=np.float64)
        self._yellow_map: np.ndarray = np.empty((0, 2), dtype=np.float64)
        self._blue_frame:   np.ndarray = np.empty((0, 2))
        self._yellow_frame: np.ndarray = np.empty((0, 2))

        self._recording   = False
        self._done         = False
        self._start_pos: np.ndarray | None = None
        self._max_dist_from_start = 0.0
        self._record_start_time: float | None = None

        self.get_logger().info(
            f'cone_recorder ready — waiting for GO. Will write to {self._out_path}'
        )

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _left_cb(self, msg: Track) -> None:
        self._blue_frame = _cones_to_array(msg.cones)

    def _right_cb(self, msg: Track) -> None:
        self._yellow_frame = _cones_to_array(msg.cones)

    def _go_cb(self, msg: GoSignal) -> None:
        if not self._recording and not self._done:
            self._recording = True
            self._record_start_time = time.time()
            self.get_logger().info('GO received — recording cone map for one lap.')

    def _pose_cb(self, msg: PoseStamped) -> None:
        if self._done:
            return

        car_pos = np.array([msg.pose.position.x, msg.pose.position.y])

        if not self._recording:
            return

        # Accumulate this frame's boundary cones into the persistent map.
        self._blue_map   = _absorb(self._blue_map,   self._blue_frame)
        self._yellow_map = _absorb(self._yellow_map, self._yellow_frame)

        if self._start_pos is None:
            self._start_pos = car_pos.copy()

        dist_from_start = float(np.linalg.norm(car_pos - self._start_pos))
        self._max_dist_from_start = max(self._max_dist_from_start, dist_from_start)

        lap_closed = (
            self._max_dist_from_start >= self._min_lap_dist
            and dist_from_start <= self._close_dist
        )
        timed_out = (
            self._record_start_time is not None
            and (time.time() - self._record_start_time) >= self._max_record_time
        )

        if lap_closed or timed_out:
            self._finish(lap_closed=lap_closed)

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------

    def _finish(self, lap_closed: bool) -> None:
        self._recording = True   # keep ignoring further pose callbacks below
        self._done = True

        payload = {
            'blue':   self._blue_map.tolist(),
            'yellow': self._yellow_map.tolist(),
            'source': 'fsae_sim_perception.cone_recorder',
            'lap_closed': bool(lap_closed),
        }
        with open(self._out_path, 'w') as f:
            json.dump(payload, f, indent=2)

        n_blue, n_yellow = len(self._blue_map), len(self._yellow_map)
        if lap_closed:
            self.get_logger().info(
                f'Lap closed — wrote {n_blue} blue + {n_yellow} yellow cones to {self._out_path}'
            )
        else:
            self.get_logger().warn(
                f'max_record_time elapsed before the lap closed — wrote {n_blue} blue + '
                f'{n_yellow} yellow cones to {self._out_path} anyway (lap_closed=false).'
            )

        if self._shutdown_on_save:
            self.get_logger().info('shutdown_on_save=true — shutting down.')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ConeRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
