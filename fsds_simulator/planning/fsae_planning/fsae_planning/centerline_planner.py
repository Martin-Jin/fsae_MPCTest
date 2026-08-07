"""
Barebone centreline planner node.

Plans a centreline a few midpoints at a time using the cone-wall barrier planner
(see boundary.build_path_walls): same-colour cones are connected into a wall mesh
and midpoints are chained with a greedy walk that penalises steps crossing the
mesh.  There is NO localisation — the car plans purely from the boundary cones it
currently has, lap after lap.  This is a pure first-lap centreline follower; the
same rolling-window plan is republished every lap.

Interface (matches the fsae_autonomous car stack):

    in   /fsae/slam/left_track    fsae_interfaces/Track     blue (left) boundary, global frame
    in   /fsae/slam/right_track   fsae_interfaces/Track     yellow (right) boundary, global frame
    in   /fsae/slam/car_position  geometry_msgs/PoseStamped x,y in position; yaw in orientation.w
    out  /fsae/planning/selected_trajectory  geometry_msgs/PoseArray   centreline waypoints

The plan loop is triggered by each car_position update (upstream convention).
"""
import numpy as np
import rclpy
from rclpy.node import Node

from fsae_interfaces.msg import Track
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs.msg import Empty

from fsae_planning.boundary import build_path_walls
from fsae_planning.cone_map import ConeMap
from fsae_planning.path_utils import (
    blend_paths,
    build_local_path,
    DEFAULT_SMOOTH_PER_PT,
)


def cones_to_array(cones) -> np.ndarray:
    """geometry_msgs/Point[] → (N, 2) float64 array of x,y."""
    if not cones:
        return np.empty((0, 2))
    return np.array([[p.x, p.y] for p in cones], dtype=np.float64)


class CenterlinePlanner(Node):
    def __init__(self, node_name: str = 'centerline_planner'):
        super().__init__(node_name)

        # Per-point spline smoothing budget (splprep s = smooth * n_points).
        # 0.0 → interpolating spline (reproduces every cone-pairing kink);
        # a small positive value approximates the midpoints for a clean line.
        self.declare_parameter('smooth', DEFAULT_SMOOTH_PER_PT)
        self._smooth_per_pt = self.get_parameter('smooth').get_parameter_value().double_value

        # Arc-length horizon (m) the published centreline is clamped to.  Keeps
        # the near path in front of the car invariant to how far the lookahead
        # reaches (extra far midpoints no longer reshape it) and stops distant
        # apex points dragging the corner line inward.  See build_path_walls.
        # 25.0 (was 15.0) matches boundary._WALL_PLAN_HORIZON and
        # control_utils.curvature_speed's scan_end — the car needs ~24 m to
        # brake from v_max down to a tight hairpin's speed at a realistic
        # achieved deceleration; a shorter horizon revealed such corners too
        # late, saturating steering and spinning out at corner entry.
        self.declare_parameter('plan_horizon', 25.0)
        self._plan_horizon = self.get_parameter('plan_horizon').get_parameter_value().double_value

        # Temporal path blend weight toward each freshly-planned path (EMA in the
        # map frame).  The planner rebuilds the path from scratch every tick, so
        # without blending successive paths jump and the controller jerks.
        # 1.0 disables blending (pure new path); smaller = smoother/laggier.
        self.declare_parameter('path_blend', 0.4)
        self._path_blend = self.get_parameter('path_blend').get_parameter_value().double_value

        # Cone visibility radius for planning (m).  The planner crops the
        # accumulated cone map to this radius (omni) OR the forward box, so the
        # path spans corners instead of truncating when the track curves out of
        # a heading-aligned box.  See boundary.build_path_walls / filter_cones_window.
        # 25.0 (was 18.0) — kept >= plan_horizon so the wall/midpoint mesh
        # actually extends as far as the path is allowed to; see plan_horizon.
        self.declare_parameter('look_radius', 25.0)
        self._look_radius = self.get_parameter('look_radius').get_parameter_value().double_value

        self.create_subscription(Track, '/fsae/slam/left_track',  self._left_cb,  10)
        self.create_subscription(Track, '/fsae/slam/right_track', self._right_cb, 10)
        self.create_subscription(PoseStamped, '/fsae/slam/car_position', self._pose_cb, 10)

        # Debug hook: the accumulated ConeMap never forgets a cone (see cone_map.py),
        # so an external tool that edits the track has no way to retract one.  An
        # Empty here drops the map; the next boundary frame rebuilds it from scratch.
        self.create_subscription(Empty, '/fsae/planning/reset_map', self._reset_cb, 10)

        self.pub_traj = self.create_publisher(PoseArray, '/fsae/planning/selected_trajectory', 10)

        # Debug: expose the wall mesh and candidate midpoints that build_path_walls
        # computes but otherwise keeps to itself.  Off by default — this is pure
        # instrumentation for the external path analyser.
        self.declare_parameter('debug_viz', False)
        self._debug_viz = self.get_parameter('debug_viz').get_parameter_value().bool_value
        self._dbg_pubs: dict = {}
        if self._debug_viz:
            self._dbg_pubs = {
                'midpoints':    self.create_publisher(PoseArray, '/fsae/planning/debug/midpoints', 10),
                'blue_walls':   self.create_publisher(PoseArray, '/fsae/planning/debug/blue_walls', 10),
                'yellow_walls': self.create_publisher(PoseArray, '/fsae/planning/debug/yellow_walls', 10),
            }
            self.get_logger().info('debug_viz on — publishing /fsae/planning/debug/*')

        self._cone_map    = ConeMap()          # accumulated historical cone map
        self._blue_cones:   np.ndarray = np.empty((0, 2))   # latest boundary frame
        self._yellow_cones: np.ndarray = np.empty((0, 2))
        self._car_pos   = np.zeros(2)
        self._car_yaw   = 0.0
        self._have_pose = False
        self._centreline: np.ndarray | None = None
        self._prev_centreline: np.ndarray | None = None   # last published, for blending
        self._blue_segs:   list = []
        self._yellow_segs: list = []
        self._midpoints:   np.ndarray = np.empty((0, 2))

        self.get_logger().info(f'{node_name} ready — waiting for car_position.')

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _left_cb(self, msg: Track) -> None:
        self._blue_cones = cones_to_array(msg.cones)

    def _right_cb(self, msg: Track) -> None:
        self._yellow_cones = cones_to_array(msg.cones)

    def _pose_cb(self, msg: PoseStamped) -> None:
        # x,y in position; yaw (rad) is stuffed into orientation.w (upstream convention).
        self._car_pos   = np.array([msg.pose.position.x, msg.pose.position.y])
        self._car_yaw   = float(msg.pose.orientation.w)
        self._have_pose = True
        self._planning_loop()

    def _reset_cb(self, _msg: Empty) -> None:
        """Drop the accumulated cone map (debug/testing only)."""
        self._cone_map.reset()
        self._blue_cones   = np.empty((0, 2))
        self._yellow_cones = np.empty((0, 2))
        self._centreline   = None
        self._prev_centreline = None
        self._blue_segs    = []
        self._yellow_segs  = []
        self._midpoints    = np.empty((0, 2))
        self.get_logger().info('cone map reset by external request')

    # ------------------------------------------------------------------
    # Planning loop (template — subclasses override the hooks)
    # ------------------------------------------------------------------

    def _planning_loop(self) -> None:
        if not self._have_pose:
            return

        # Accumulate the current boundary frame into the persistent map.
        self._cone_map.update(self._blue_cones, self._yellow_cones)

        self._compute_path()

        # Temporally blend the fresh path with the last one so the published
        # trajectory eases between frames instead of jumping (which the
        # controller would track as a steering jerk).  Recursive EMA in the map
        # frame; resets itself when the path genuinely diverges (see blend_paths).
        if self._centreline is not None and len(self._centreline) >= 2:
            self._centreline = blend_paths(
                self._prev_centreline, self._centreline, self._car_pos,
                alpha=self._path_blend, horizon=self._plan_horizon,
            )
            self._prev_centreline = self._centreline
        else:
            self._prev_centreline = None

        self._publish_trajectory()
        self._publish_debug()

        if self._centreline is None:
            self.get_logger().warn(
                'No forward cones visible — no trajectory published',
                throttle_duration_sec=2.0,
            )
        else:
            self.get_logger().info(
                f'trajectory published: {len(self._centreline)} pts  '
                f'car=({self._car_pos[0]:.1f},{self._car_pos[1]:.1f})',
                throttle_duration_sec=1.0,
            )

    # ------------------------------------------------------------------
    # Override hooks
    # ------------------------------------------------------------------

    def _compute_path(self) -> None:
        """Cone-wall mesh planner over the accumulated boundary cones."""
        try:
            self._centreline, self._blue_segs, self._yellow_segs, self._midpoints = \
                build_path_walls(
                    self._cone_map.blue, self._cone_map.yellow,
                    self._car_pos, self._car_yaw,
                    smooth_per_pt=self._smooth_per_pt,
                    look_radius=self._look_radius,
                    plan_horizon=self._plan_horizon,
                )
        except Exception as exc:
            self.get_logger().warn(
                f'Wall-barrier planner failed ({exc!r}), falling back to simple pairing',
                throttle_duration_sec=5.0,
            )
            self._centreline = build_local_path(
                self._cone_map.blue, self._cone_map.yellow,
                self._car_pos, self._car_yaw,
            )
            self._blue_segs   = []
            self._yellow_segs = []
            self._midpoints   = np.empty((0, 2))

    # ------------------------------------------------------------------
    # Publishing / visualisation
    # ------------------------------------------------------------------

    def _publish_trajectory(self) -> None:
        if self._centreline is None or len(self._centreline) == 0:
            return
        msg = PoseArray()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for wp in self._centreline:
            pose = Pose()
            pose.position.x = float(wp[0])
            pose.position.y = float(wp[1])
            msg.poses.append(pose)
        self.pub_traj.publish(msg)

    def _pose_array(self, points) -> PoseArray:
        """(N, 2) points -> PoseArray in the map frame (positions only)."""
        msg = PoseArray()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for pt in points:
            pose = Pose()
            pose.position.x = float(pt[0])
            pose.position.y = float(pt[1])
            msg.poses.append(pose)
        return msg

    def _publish_debug(self) -> None:
        """
        Publish the wall mesh and candidate midpoints.

        Wall segments go out as a flat PoseArray of consecutive endpoint PAIRS
        (poses 0-1 are one segment, 2-3 the next, ...).  Reusing PoseArray keeps this
        patch free of new interface definitions and rebuilds of fsae_interfaces.
        """
        if not self._dbg_pubs:
            return
        self._dbg_pubs['midpoints'].publish(self._pose_array(self._midpoints))
        for key, segs in (('blue_walls', self._blue_segs), ('yellow_walls', self._yellow_segs)):
            flat = [pt for seg in segs for pt in seg]
            self._dbg_pubs[key].publish(self._pose_array(flat))


def main(args=None):
    rclpy.init(args=args)
    node = CenterlinePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
