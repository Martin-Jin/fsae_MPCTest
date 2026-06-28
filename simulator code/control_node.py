# control_node.py — ROS 2 MPC Control Loop for FSDS

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from fs_msgs.msg import ControlCommand, GoSignal, Track
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32

from .control_utils import MPCController
from fsae_planning.cone_sorting import separate_cones_by_color

# ── Tuneable constants ─────────────────────────────────────────────────────────

V_FALLBACK       = 2.0   # m/s  — desired speed used until planner publishes one
CONE_BRAKE_DIST  = 1.5   # m    — forward corridor depth for cone proximity brake
CONE_BRAKE_WIDTH = 0.18  # m    — lateral half-width of braking corridor (36 cm total)
TARGET_TIMEOUT   = 0.5   # s    — brake if no fresh path received within this window

# Minimum duration (s) the cone proximity brake must be active continuously
# before reset() is called on the MPC.  Single-frame cone hits do not warrant
# discarding the warm-start; only sustained stops do.
CONE_RESET_THRESHOLD = 0.3   # s  (~6 consecutive 50 ms ticks)


class ControlNode(Node):
    def __init__(self):
        super().__init__('controller')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Subscriptions ──────────────────────────────────────────────
        self.create_subscription(Path,     '/fsds/planned_path',      self._path_cb,  10)
        self.create_subscription(Float32,  '/fsds/desired_speed',     self._speed_cb, 10)
        self.create_subscription(Odometry, '/fsds/testing_only/odom', self._odom_cb,  sensor_qos)
        self.create_subscription(Track,    '/FusionCones',            self._track_cb, 10)
        self.create_subscription(GoSignal, '/fsds/signal/go',         self._go_cb,    10)

        # ── Publisher ──────────────────────────────────────────────────
        self.pub_cmd = self.create_publisher(ControlCommand, '/fsds/control_command', 10)

        # ── Internal state ─────────────────────────────────────────────
        self._go_received    = False
        self._path_pts:  np.ndarray = np.empty((0, 2))
        self._path_stamp = None           # rclpy.time.Time of last path message
        self._desired_speed: float = V_FALLBACK
        self._car_pos        = np.zeros(2)
        self._car_yaw        = 0.0
        self._car_speed      = 0.0
        self._car_yaw_rate   = 0.0
        self._blue_cones:   np.ndarray = np.empty((0, 2))
        self._yellow_cones: np.ndarray = np.empty((0, 2))

        # Cone-brake continuity tracking — avoids spurious MPC resets on
        # momentary single-frame detections inside the braking corridor.
        self._cone_brake_duration: float = 0.0   # seconds currently braking

        # ── MPC controller ─────────────────────────────────────────────
        # N=20 gives 1 s of preview at 50 ms per step.
        # The parameterised QP is compiled on the first compute() call so
        # there is no startup latency on this constructor.
        self._mpc = MPCController(dt=0.05, N=20)

        # ── 20 Hz control timer ────────────────────────────────────────
        self.create_timer(0.05, self._control_loop)
        self.get_logger().info('MPC Control node initialised and ready.')

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _go_cb(self, msg: GoSignal) -> None:
        if not self._go_received:
            self._go_received = True
            self.get_logger().info('GO signal received. Launching control loop.')

    def _path_cb(self, msg: Path) -> None:
        self._path_pts = np.array(
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

        # Quaternion -> yaw (Z-axis rotation)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._car_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _track_cb(self, msg: Track) -> None:
        self._blue_cones, self._yellow_cones = separate_cones_by_color(msg)

    # ------------------------------------------------------------------
    # Core MPC control loop (50 ms / 20 Hz)
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        cmd = ControlCommand()

        # ── Phase 1: Hold at start line until GO signal ────────────────
        if not self._go_received:
            cmd.throttle = 0.0
            cmd.steering = 0.0
            cmd.brake    = 1.0
            self.pub_cmd.publish(cmd)
            self.get_logger().info(
                'Waiting for GO signal...', throttle_duration_sec=2.0
            )
            return

        # ── Phase 2: Emergency brake on stale / missing path ──────────
        path_stale = (
            self._path_stamp is None
            or (self.get_clock().now() - self._path_stamp).nanoseconds * 1e-9
               > TARGET_TIMEOUT
            or len(self._path_pts) < 2
        )
        if path_stale:
            cmd.throttle = 0.0
            cmd.steering = 0.0
            cmd.brake    = 1.0
            self._mpc.reset()   # path loss is a large discontinuity; discard warm-start
            self.pub_cmd.publish(cmd)
            self.get_logger().warn(
                'Trajectory path lost or stale — emergency braking.',
                throttle_duration_sec=1.0,
            )
            return

        # ── Phase 3: MPC optimal control ──────────────────────────────
        steer_out, throttle_out, brake_out = self._mpc.compute(
            path=self._path_pts,
            car_pos=self._car_pos,
            car_yaw=self._car_yaw,
            car_speed=self._car_speed,
            desired_speed=self._desired_speed,
            car_yaw_rate=self._car_yaw_rate,
        )

        cmd.steering = steer_out
        cmd.throttle = throttle_out
        cmd.brake    = brake_out

        # ── Phase 4: Cone proximity brake override ────────────────────
        # Transform all visible cones into the car-relative frame and check
        # whether any fall inside the braking corridor ahead of the car.
        cone_parts = [c for c in (self._blue_cones, self._yellow_cones) if len(c) > 0]
        too_close  = False

        if cone_parts:
            all_cones = np.vstack(cone_parts)
            cos_y = math.cos(self._car_yaw)
            sin_y = math.sin(self._car_yaw)
            rel   = all_cones - self._car_pos

            x_car =  rel[:, 0] * cos_y + rel[:, 1] * sin_y   # forward (+)
            y_car = -rel[:, 0] * sin_y + rel[:, 1] * cos_y   # left    (+)

            # Scale braking corridor depth with speed so faster approaches
            # get more stopping distance; clamped to physical limits.
            dynamic_brake_dist = float(np.clip(
                self._car_speed * 0.25, 0.6, CONE_BRAKE_DIST
            ))

            too_close = bool(np.any(
                (x_car > 0.2) &
                (x_car < dynamic_brake_dist) &
                (np.abs(y_car) < CONE_BRAKE_WIDTH)
            ))

        if too_close:
            cmd.throttle = 0.0
            cmd.brake    = 1.0
            self._cone_brake_duration += self.dt if hasattr(self, 'dt') else 0.05

            # Only reset warm-start after a sustained stop — brief single-frame
            # detections should not throw away OSQP continuity.
            if self._cone_brake_duration >= CONE_RESET_THRESHOLD:
                self._mpc.reset()

            self.get_logger().warn(
                f'Cone proximity brake active '
                f'({self._cone_brake_duration:.2f} s).',
                throttle_duration_sec=0.5,
            )
        else:
            # Clear cone-brake timer when corridor is free
            self._cone_brake_duration = 0.0

        # ── Phase 5: Timestamp and publish ────────────────────────────
        cmd.header.stamp = self.get_clock().now().to_msg()

        # Per-tick telemetry at DEBUG level — does not flood the INFO log
        # buffer at 20 Hz, but is visible when --log-level debug is set.
        self.get_logger().debug(
            f'MPC thr={cmd.throttle:.2f} brk={cmd.brake:.2f} '
            f'steer={cmd.steering:.3f} | '
            f'v={self._car_speed:.1f}/{self._desired_speed:.1f} m/s'
        )
        # Throttled INFO summary — one line per second for operator awareness
        self.get_logger().info(
            f'MPC_CMD thr={cmd.throttle:.2f} brk={cmd.brake:.2f} '
            f'steer={cmd.steering:.3f} | '
            f'v={self._car_speed:.1f}/{self._desired_speed:.1f} m/s',
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