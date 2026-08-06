"""
FSDS command bridge (fsae control interface → simulator).

The car stack's controllers emit an Ackermann drive command on
/fsae/control/cmd_vel (target speed + steering angle); on the real car a CAN
bridge turns that into bus frames the actuators understand.  This is the
simulator analog: it converts cmd_vel into an fs_msgs/ControlCommand for FSDS —
speed → throttle/brake (P-control against the simulator's current speed),
steering angle → normalised steering — and gates everything on the FSDS GO signal
so the car holds the brake until the run starts.

    in   /fsae/control/cmd_vel            ackermann_msgs/AckermannDriveStamped
    in   /fsds/testing_only/odom          nav_msgs/Odometry       current speed feedback
    in   /fsds/signal/go                  fs_msgs/GoSignal        race start
    in   /fsae/perception/cone_detection  fsae_interfaces/ConeDetection  proximity emergency brake
    out  /fsds/control_command            fs_msgs/ControlCommand
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from fs_msgs.msg import ControlCommand, GoSignal
from fsae_interfaces.msg import ConeDetection
from nav_msgs.msg import Odometry

MAX_STEER_RAD    = math.radians(25.0)  # FSDS max steering angle
KP_THROTTLE      = 0.06  # throttle P-gain (throttle per m/s of under-speed)
KP_BRAKE         = 0.40  # brake P-gain    (brake    per m/s of over-speed)
CONE_BRAKE_DIST  = 2.5   # metres forward — hard-brake if a cone enters this zone
CONE_BRAKE_WIDTH = 0.6   # metres lateral half-width of the braking corridor
CMD_TIMEOUT      = 0.5   # seconds — brake if no fresh cmd_vel received


class FsdsBridge(Node):
    def __init__(self):
        super().__init__('fsds_bridge')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(AckermannDriveStamped, '/fsae/control/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(Odometry, '/fsds/testing_only/odom', self._odom_cb, sensor_qos)
        self.create_subscription(GoSignal, '/fsds/signal/go', self._go_cb, 10)
        self.create_subscription(ConeDetection, '/fsae/perception/cone_detection', self._det_cb, 10)

        self.pub_cmd = self.create_publisher(ControlCommand, '/fsds/control_command', 10)

        self._go_received = False
        self._cmd_speed = 0.0
        self._cmd_steer = 0.0            # steering angle (rad, +ve = left)
        self._cmd_stamp = None           # rclpy.time.Time of last cmd_vel
        self._car_speed = 0.0
        self._cones_local: np.ndarray = np.empty((0, 2))

        self.create_timer(0.05, self._loop)  # 20 Hz

        self.get_logger().info('fsds_bridge ready — waiting for GO signal.')

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _go_cb(self, msg: GoSignal) -> None:
        if not self._go_received:
            self._go_received = True
            self.get_logger().info('GO received.')

    def _cmd_cb(self, msg: AckermannDriveStamped) -> None:
        self._cmd_speed = float(msg.drive.speed)
        self._cmd_steer = float(msg.drive.steering_angle)
        self._cmd_stamp = self.get_clock().now()

    def _odom_cb(self, msg: Odometry) -> None:
        v = msg.twist.twist.linear
        self._car_speed = float(math.hypot(v.x, v.y))

    def _det_cb(self, msg: ConeDetection) -> None:
        pts = [[p.x, p.y] for p in msg.blue] + [[p.x, p.y] for p in msg.yellow]
        self._cones_local = np.array(pts, dtype=np.float64) if pts else np.empty((0, 2))

    # ------------------------------------------------------------------
    # Command loop
    # ------------------------------------------------------------------

    def _loop(self) -> None:
        cmd = ControlCommand()

        if not self._go_received:
            self.pub_cmd.publish(cmd)   # all-zero: idle before GO
            self.get_logger().info('Waiting for GO signal...', throttle_duration_sec=2.0)
            return

        cmd_stale = (
            self._cmd_stamp is None
            or (self.get_clock().now() - self._cmd_stamp).nanoseconds * 1e-9 > CMD_TIMEOUT
        )
        if cmd_stale:
            cmd.throttle = 0.0
            cmd.steering = 0.0
            cmd.brake    = 1.0
            self.pub_cmd.publish(cmd)
            self.get_logger().warn('No fresh cmd_vel — braking', throttle_duration_sec=1.0)
            return

        # Steering: rad (+ve = left) → FSDS normalised (+1 = right).
        cmd.steering = float(np.clip(-self._cmd_steer / MAX_STEER_RAD, -1.0, 1.0))

        if self._cone_too_close():
            cmd.throttle = 0.0
            cmd.brake    = 1.0
        else:
            speed_error = self._cmd_speed - self._car_speed
            if speed_error >= 0.0:
                cmd.throttle = min(1.0, KP_THROTTLE * speed_error)
                cmd.brake    = 0.0
            else:
                cmd.throttle = 0.0
                cmd.brake    = min(1.0, KP_BRAKE * (-speed_error))

        cmd.header.stamp = self.get_clock().now().to_msg()
        self.pub_cmd.publish(cmd)
        self.get_logger().info(
            f'CMD thr={cmd.throttle:.3f} brk={cmd.brake:.1f} steer={cmd.steering:.3f}  '
            f'v={self._car_speed:.2f}/{self._cmd_speed:.2f} m/s',
            throttle_duration_sec=1.0,
        )

    def _cone_too_close(self) -> bool:
        """Emergency brake if a cone sits in the forward corridor (car frame)."""
        if len(self._cones_local) == 0:
            return False
        x = self._cones_local[:, 0]
        y = self._cones_local[:, 1]
        return bool(np.any((x > 0.3) & (x < CONE_BRAKE_DIST) & (np.abs(y) < CONE_BRAKE_WIDTH)))


def main(args=None):
    rclpy.init(args=args)
    node = FsdsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
