import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from fs_msgs.msg import GoSignal, Track
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32

from fsae_planning.boundary import build_path_walls
from fsae_planning.cone_map import ConeMap
from fsae_planning.cone_sorting import separate_cones_by_color
from fsae_planning.path_utils import (
    build_local_path,
    check_direction,
    compute_desired_speed,
    get_lookahead_waypoint,
)
from fsae_planning.viz_utils import Visualizer

LOOKAHEAD_DIST = 4.0  # metres ahead for pure-pursuit target
V_MAX          = 20.0  # m/s — top speed on straights
V_MIN          = 1.5  # m/s — minimum speed through tight corners


class PlannerNode(Node):
    def __init__(self):
        super().__init__('centreline_planner')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Track, '/FusionCones', self._track_cb, 10)
        self.create_subscription(Odometry, '/fsds/testing_only/odom', self._odom_cb, sensor_qos)
        self.create_subscription(GoSignal, '/fsds/signal/go', self._go_cb, 10)

        self.pub_path   = self.create_publisher(Path,         '/fsds/planned_path',     10)
        self.pub_target = self.create_publisher(PointStamped, '/fsds/lookahead_target', 10)
        self.pub_speed  = self.create_publisher(Float32,      '/fsds/desired_speed',    10)

        self._go_received = False
        self._cone_map    = ConeMap()          # accumulated historical cone map
        self._blue_cones:   np.ndarray = np.empty((0, 2))   # latest sensor frame (viz)
        self._yellow_cones: np.ndarray = np.empty((0, 2))
        self._car_pos = np.zeros(2)
        self._car_yaw = 0.0
        self._centreline: np.ndarray | None = None
        self._blue_segs:   list = []
        self._yellow_segs: list = []
        self._midpoints:   np.ndarray = np.empty((0, 2))

        self.create_timer(0.05, self._planning_loop)

        self._viz = Visualizer()
        self.create_timer(1 / 3.0, self._viz_loop)

        self.get_logger().info('Planner node ready — waiting for GO signal.')

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _go_cb(self, msg: GoSignal) -> None:
        if not self._go_received:
            self.get_logger().info(
                f'GO received: mission={msg.mission!r}, track={msg.track!r}'
            )
            self._go_received = True

    def _track_cb(self, msg: Track) -> None:
        blue, yellow = separate_cones_by_color(msg)
        self._blue_cones   = blue
        self._yellow_cones = yellow
        self._cone_map.update(blue, yellow)
        self.get_logger().info(
            f'Track update: {len(blue)} blue + {len(yellow)} yellow this frame  '
            f'| map: {len(self._cone_map.blue)}b + {len(self._cone_map.yellow)}y total',
            throttle_duration_sec=2.0,
        )

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._car_pos = np.array([p.x, p.y])

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._car_yaw = math.atan2(siny_cosp, cosy_cosp)

    # ------------------------------------------------------------------
    # Planning loop
    # ------------------------------------------------------------------

    def _planning_loop(self) -> None:
        if not self._go_received:
            self.get_logger().info('Waiting for GO signal...', throttle_duration_sec=2.0)
            return

        try:
            self._centreline, self._blue_segs, self._yellow_segs, self._midpoints = \
                build_path_walls(
                    self._cone_map.blue, self._cone_map.yellow,
                    self._car_pos, self._car_yaw,
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
        self._publish_path()

        if self._centreline is None:
            self.get_logger().warn(
                'No forward cones visible — no target published',
                throttle_duration_sec=2.0,
            )
            return

        if not check_direction(
            self._car_pos, self._car_yaw,
            self._blue_cones, self._yellow_cones,   # use live frame for direction check
        ):
            self.get_logger().warn(
                'Direction check failed: blue not left / yellow not right',
                throttle_duration_sec=1.0,
            )
            return

        target = get_lookahead_waypoint(
            self._centreline, self._car_pos, self._car_yaw, LOOKAHEAD_DIST
        )

        if target is None:
            return

        msg = PointStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'fsds/map'
        msg.point.x = float(target[0])
        msg.point.y = float(target[1])
        msg.point.z = 0.0
        self.pub_target.publish(msg)

        v_target = compute_desired_speed(self._centreline, v_max=V_MAX, v_min=V_MIN)
        speed_msg = Float32()
        speed_msg.data = v_target
        self.pub_speed.publish(speed_msg)

        self.get_logger().info(
            f'Target: ({float(target[0]):.1f}, {float(target[1]):.1f})  '
            f'v_target={v_target:.2f} m/s  '
            f'car=({self._car_pos[0]:.1f},{self._car_pos[1]:.1f})',
            throttle_duration_sec=1.0,
        )

    # ------------------------------------------------------------------
    # Visualisation
    # ------------------------------------------------------------------

    def _viz_loop(self) -> None:
        # Show the full accumulated cone map so historical cones remain visible.
        self._viz.update(
            self._car_pos,
            self._car_yaw,
            self._cone_map.blue,
            self._cone_map.yellow,
            self._centreline,
            blue_segs=self._blue_segs,
            yellow_segs=self._yellow_segs,
            midpoints=self._midpoints,
        )

    def _publish_path(self) -> None:
        if self._centreline is None or len(self._centreline) == 0:
            return
        path = Path()
        path.header.frame_id = 'fsds/map'
        path.header.stamp    = self.get_clock().now().to_msg()
        for wp in self._centreline:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(wp[0])
            ps.pose.position.y = float(wp[1])
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_path.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
