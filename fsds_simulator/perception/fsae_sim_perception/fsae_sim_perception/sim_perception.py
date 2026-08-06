"""
Simulator perception stand-in (FSDS → fsae_autonomous interface bridge).

On the real car the perception front-end is a ZED camera node (cone detection +
visual-odometry pose) followed by a SLAM cone_mapper that produces the boundary
tracks.  The FSDS simulator instead broadcasts a latched oracle cone map plus
ground-truth odometry.  This node stands in for the whole camera+SLAM front-end:
it crops the oracle map to a forward FOV window (to mimic a real sensor) and
republishes it on the exact topics/messages the car stack expects.

    in   /fsds/testing_only/track  fs_msgs/Track       latched oracle cone map
    in   /fsds/testing_only/odom   nav_msgs/Odometry   ground-truth pose

    out  /fsae/slam/car_position   geometry_msgs/PoseStamped     x,y in position; yaw in orientation.w;
                                                                  header.stamp = odom's own measurement time
    out  /fsae/slam/left_track     fsae_interfaces/Track         blue (left) boundary, global frame
    out  /fsae/slam/right_track    fsae_interfaces/Track         yellow (right) boundary, global frame
    out  /fsae/perception/cone_detection  fsae_interfaces/ConeDetection  local-frame detections + car_pose

Set the `full_track` parameter true to publish the entire map every frame (used by
the skidpad planner, which reconstructs the whole figure-8 up front); the default
(false) publishes only the forward window, matching normal driving.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from fs_msgs.msg import Cone, Track as FsTrack
from fsae_interfaces.msg import ConeDetection, Track
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry


class SimPerception(Node):
    def __init__(self):
        super().__init__('sim_perception')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('look_ahead', 25.0),   # m ahead to include in the forward box
                ('look_wide', 10.0),    # m lateral half-width of the forward box
                ('min_ahead', 0.5),     # m: box ignores cones behind / at the car
                # 25.0 (was 18.0) — kept >= centerline_planner's look_radius/
                # plan_horizon so cones for a tight corner reach the planner
                # far enough out to detect and brake for it in time.
                ('look_radius', 25.0),  # m: omni-directional visibility radius
                ('full_track', False),  # publish the whole map instead of the window
            ],
        )
        self._look_ahead  = self.get_parameter('look_ahead').get_parameter_value().double_value
        self._look_wide   = self.get_parameter('look_wide').get_parameter_value().double_value
        self._min_ahead   = self.get_parameter('min_ahead').get_parameter_value().double_value
        self._look_radius = self.get_parameter('look_radius').get_parameter_value().double_value
        self._full_track  = self.get_parameter('full_track').get_parameter_value().bool_value

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(FsTrack, '/fsds/testing_only/track', self._track_cb, latched_qos)
        self.create_subscription(Odometry, '/fsds/testing_only/odom', self._odom_cb, sensor_qos)

        self.pub_pose  = self.create_publisher(PoseStamped,   '/fsae/slam/car_position',        10)
        self.pub_left  = self.create_publisher(Track,         '/fsae/slam/left_track',          10)
        self.pub_right = self.create_publisher(Track,         '/fsae/slam/right_track',         10)
        self.pub_det   = self.create_publisher(ConeDetection, '/fsae/perception/cone_detection', 10)

        # Oracle map, split by colour into global (N, 2) arrays once received.
        self._blue_all:   np.ndarray = np.empty((0, 2))
        self._yellow_all: np.ndarray = np.empty((0, 2))
        self._have_map = False
        self._have_odom = False
        self._car_x = 0.0
        self._car_y = 0.0
        self._car_yaw = 0.0
        self._odom_stamp = None

        self.create_timer(0.1, self._publish)  # 10 Hz

        mode = ('full map' if self._full_track else
                f'radius {self._look_radius} m ∪ box {self._look_ahead}×{2 * self._look_wide} m')
        self.get_logger().info(f'sim_perception ready ({mode}).')

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _track_cb(self, msg: FsTrack) -> None:
        blue, yellow = [], []
        for cone in msg.track:
            pt = [cone.location.x, cone.location.y]
            if cone.color == Cone.BLUE:
                blue.append(pt)
            elif cone.color == Cone.YELLOW:
                yellow.append(pt)
        self._blue_all   = np.array(blue,   dtype=np.float64) if blue   else np.empty((0, 2))
        self._yellow_all = np.array(yellow, dtype=np.float64) if yellow else np.empty((0, 2))
        self._have_map = True
        self.get_logger().info(
            f'Oracle map received: {len(blue)} blue + {len(yellow)} yellow.', once=True
        )

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._car_x = p.x
        self._car_y = p.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._car_yaw = math.atan2(siny_cosp, cosy_cosp)
        self._odom_stamp = msg.header.stamp
        self._have_odom = True

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _visible(self, cones: np.ndarray) -> np.ndarray:
        """
        Crop cones to the sensor FOV (car frame), or pass all through.

        FOV = within look_radius of the car (omni — mimics a 360° lidar and keeps
        the cones around a bend that a forward-only box loses when the track
        curves away) OR inside the forward box (longer-range preview straight
        ahead).  Cropping still happens so the planner's cone map fills in track
        order over a lap rather than being handed the whole oracle map at once.
        """
        if self._full_track or len(cones) == 0:
            return cones
        cos_y, sin_y = math.cos(self._car_yaw), math.sin(self._car_yaw)
        rel = cones - np.array([self._car_x, self._car_y])
        dist = np.hypot(rel[:, 0], rel[:, 1])
        x_car =  rel[:, 0] * cos_y + rel[:, 1] * sin_y
        y_car = -rel[:, 0] * sin_y + rel[:, 1] * cos_y
        box = (x_car > self._min_ahead) & (x_car < self._look_ahead) & \
              (np.abs(y_car) < self._look_wide)
        return cones[(dist < self._look_radius) | box]

    def _car_pose_msg(self) -> PoseStamped:
        msg = PoseStamped()
        # Odom's own timestamp, not now() — this is the time the pose was
        # actually measured, which is what delay/age estimation downstream
        # (MPCController's pose_age_s) needs to be measured against.
        msg.header.stamp = self._odom_stamp
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(self._car_x)
        msg.pose.position.y = float(self._car_y)
        # Upstream convention: yaw (rad) is repurposed into orientation.w.
        msg.pose.orientation.w = float(self._car_yaw)
        return msg

    @staticmethod
    def _global_track(cones: np.ndarray) -> Track:
        msg = Track()
        for c in cones:
            msg.cones.append(Point(x=float(c[0]), y=float(c[1]), z=0.0))
        return msg

    def _local_points(self, cones: np.ndarray) -> list:
        """Transform global cones into the car frame for cone_detection."""
        pts = []
        cos_y, sin_y = math.cos(self._car_yaw), math.sin(self._car_yaw)
        for c in cones:
            dx = c[0] - self._car_x
            dy = c[1] - self._car_y
            x_local =  dx * cos_y + dy * sin_y
            y_local = -dx * sin_y + dy * cos_y
            pts.append(Point(x=float(x_local), y=float(y_local), z=0.0))
        return pts

    def _publish(self) -> None:
        # Publish the car pose as soon as odometry is available — it does not
        # depend on the cone map, and the planners' loop is triggered by it.
        if not self._have_odom:
            return
        pose = self._car_pose_msg()
        self.pub_pose.publish(pose)

        if not self._have_map:
            return

        blue_vis   = self._visible(self._blue_all)
        yellow_vis = self._visible(self._yellow_all)
        self.pub_left.publish(self._global_track(blue_vis))
        self.pub_right.publish(self._global_track(yellow_vis))

        det = ConeDetection()
        det.header.stamp    = self.get_clock().now().to_msg()
        det.header.frame_id = 'base_link'
        det.car_pose = pose.pose   # ConeDetection.car_pose is geometry_msgs/Pose, not Stamped
        det.blue   = self._local_points(blue_vis)
        det.yellow = self._local_points(yellow_vis)
        self.pub_det.publish(det)


def main(args=None):
    rclpy.init(args=args)
    node = SimPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
