import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from fs_msgs.msg import ControlCommand, GoSignal, Track
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32

from control_utils import StanleyController
from cone_sorting import separate_cones_by_color

KP_THROTTLE      = 0.06  # throttle P-gain  (throttle per m/s of under-speed)
KP_BRAKE         = 0.40  # brake P-gain     (brake    per m/s of over-speed)
V_FALLBACK       = 2.0   # m/s — desired speed used until planner publishes one
CONE_BRAKE_DIST  = 2.5   # metres forward — hard-brake if cone enters this zone
CONE_BRAKE_WIDTH = 0.6   # metres lateral half-width of the braking corridor
TARGET_TIMEOUT   = 0.5   # seconds — brake if no fresh target received


class ControlNode(Node):
    def __init__(self):
        super().__init__('controller')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Path,      '/fsds/planned_path',      self._path_cb,   10)
        self.create_subscription(Float32,   '/fsds/desired_speed',     self._speed_cb,  10)
        self.create_subscription(Odometry,  '/fsds/testing_only/odom', self._odom_cb,   sensor_qos)
        self.create_subscription(Track,     '/FusionCones',            self._track_cb,  10)
        self.create_subscription(GoSignal,  '/fsds/signal/go',         self._go_cb,     10)

        self.pub_cmd = self.create_publisher(ControlCommand, '/fsds/control_command', 10)

        self._go_received    = False
        self._path_pts:  np.ndarray = np.empty((0, 2))
        self._path_stamp = None          # rclpy.time.Time
        self._desired_speed: float = V_FALLBACK
        self._car_pos        = np.zeros(2)
        self._car_yaw        = 0.0
        self._car_speed      = 0.0
        self._car_yaw_rate   = 0.0
        self._blue_cones:   np.ndarray = np.empty((0, 2))
        self._yellow_cones: np.ndarray = np.empty((0, 2))

        self._stanley = StanleyController()

        self.create_timer(0.05, self._control_loop)

        self.get_logger().info('Control node ready.')

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _go_cb(self, msg: GoSignal) -> None:
        if not self._go_received:
            self._go_received = True
            self.get_logger().info('GO received.')

    def _path_cb(self, msg: Path) -> None:
        self._path_pts  = np.array(
            [[ps.pose.position.x, ps.pose.position.y] for ps in msg.poses],
            dtype=np.float64,
        ) if msg.poses else np.empty((0, 2))
        self._path_stamp = self.get_clock().now()

    def _speed_cb(self, msg: Float32) -> None:
        self._desired_speed = float(msg.data)

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._car_pos = np.array([p.x, p.y])
        v = msg.twist.twist.linear
        self._car_speed    = math.hypot(v.x, v.y)
        self._car_yaw_rate = msg.twist.twist.angular.z

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._car_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _track_cb(self, msg: Track) -> None:
        self._blue_cones, self._yellow_cones = separate_cones_by_color(msg)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        cmd = ControlCommand()

        if not self._go_received:
            cmd.throttle = 0.0
            cmd.steering = 0.0
            cmd.brake    = 0.0
            self.pub_cmd.publish(cmd)
            self.get_logger().info('Waiting for GO signal...', throttle_duration_sec=2.0)
            return

        # Brake if path is stale or missing
        path_stale = (
            self._path_stamp is None
            or (self.get_clock().now() - self._path_stamp).nanoseconds * 1e-9 > TARGET_TIMEOUT
            or len(self._path_pts) < 2
        )
        if path_stale:
            cmd.throttle = 0.0
            cmd.steering = 0.0
            cmd.brake    = 1.0
            self.pub_cmd.publish(cmd)
            self.get_logger().warn('No fresh path — braking', throttle_duration_sec=1.0)
            return

        cmd.steering = self._stanley.compute(
            self._path_pts, self._car_pos, self._car_yaw,
            self._car_speed, self._car_yaw_rate,
        )

        # Cone proximity brake
        cone_parts = [c for c in (self._blue_cones, self._yellow_cones) if len(c) > 0]
        too_close = False
        if cone_parts:
            all_cones = np.vstack(cone_parts)
            cos_y = math.cos(self._car_yaw)
            sin_y = math.sin(self._car_yaw)
            rel   = all_cones - self._car_pos
            x_car =  rel[:, 0] * cos_y + rel[:, 1] * sin_y
            y_car = -rel[:, 0] * sin_y + rel[:, 1] * cos_y
            too_close = bool(np.any(
                (x_car > 0.3) &
                (x_car < CONE_BRAKE_DIST) &
                (np.abs(y_car) < CONE_BRAKE_WIDTH)
            ))

        if too_close:
            cmd.throttle = 0.0
            cmd.brake    = 1.0
        else:
            speed_error = self._desired_speed - self._car_speed
            if speed_error >= 0.0:
                cmd.throttle = min(1.0, KP_THROTTLE * speed_error)
                cmd.brake    = 0.0
            else:
                cmd.throttle = 0.0
                cmd.brake    = min(1.0, KP_BRAKE * (-speed_error))

        cmd.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(
            f'CMD thr={cmd.throttle:.3f} brk={cmd.brake:.1f} steer={cmd.steering:.3f}  '
            f'v={self._car_speed:.2f}/{self._desired_speed:.2f} m/s  '
            f'car=({self._car_pos[0]:.1f},{self._car_pos[1]:.1f})',
            throttle_duration_sec=1.0,
        )
        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
