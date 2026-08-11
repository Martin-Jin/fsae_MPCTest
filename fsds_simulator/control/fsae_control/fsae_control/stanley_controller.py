"""
Stanley path-tracking controller.

Follows the planned centreline and emits a drive command on the car stack's
control interface: a target speed (curvature-limited) plus a steering angle.

    in   /fsae/planning/selected_trajectory  geometry_msgs/PoseArray   path to follow
    in   /fsae/slam/car_position             geometry_msgs/PoseStamped x,y in position; yaw in orientation.w
    in   /fsds/testing_only/odom             nav_msgs/Odometry         speed + yaw-rate feedback (sim reality)
    out  /fsae/control/cmd_vel               ackermann_msgs/AckermannDriveStamped  speed + steering_angle

The FSDS-specific conversion (speed→throttle/brake, steering_angle→normalised
steering, GO gating) lives downstream in fsds_bridge, mirroring how the real car
turns cmd_vel into CAN frames.  Speed feedback comes from the simulator odometry
(the real car reads it from CAN); the steering pose comes from car_position.
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry

from fsae_control.control_utils import (
    StanleyController, curvature_speed, load_speed_profile_csv, precomputed_speed_at,
)
from fsae_control.telemetry_logger import ControlLogger, LapProgressTracker


class StanleyControllerNode(Node):
    def __init__(self):
        super().__init__('controller')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('v_max', 15.0),      # m/s — top speed on straights
                ('v_min', 1.5),       # m/s — minimum speed through tight corners
                ('stanley_gain', 1.0),  # cross-track gain (k_cte)
                ('log_csv', False),   # write CSV telemetry to log_dir
                ('log_dir', ''),      # '' -> ~/fsae_logs
                ('map_path', ''),     # '' -> live curvature_speed() (default);
                                       # else a fsae_MPCTest tuner/export_speed_profile.py
                                       # CSV to use instead — see mpc_controller.py's
                                       # identical param. Lets a Stanley run be pointed
                                       # at the same precomputed speed profile as an MPC
                                       # run on the same track, for a directly comparable
                                       # telemetry CSV.
            ],
        )
        self._v_max = self.get_parameter('v_max').get_parameter_value().double_value
        self._v_min = self.get_parameter('v_min').get_parameter_value().double_value
        k_cte = self.get_parameter('stanley_gain').get_parameter_value().double_value

        self._speed_profile = None  # (path_X, path_Y, path_V) or None
        map_path = self.get_parameter('map_path').get_parameter_value().string_value
        if map_path:
            try:
                self._speed_profile = load_speed_profile_csv(map_path)
                self.get_logger().info(
                    f'Loaded precomputed speed profile ({len(self._speed_profile[0])} pts) '
                    f'from {map_path} — using it instead of live curvature_speed().'
                )
            except (OSError, ValueError) as exc:
                self.get_logger().error(
                    f'Failed to load map_path={map_path}: {exc}. '
                    'Falling back to live curvature_speed().'
                )

        self._telemetry = None
        if self.get_parameter('log_csv').get_parameter_value().bool_value:
            log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
            self._telemetry = ControlLogger('stanley', log_dir=log_dir)
            self.get_logger().info(f'CSV telemetry -> {self._telemetry.paths[0]}')

        # See mpc_controller.py's identical field — drives close()'s
        # progress/reached_end/time_bonus.
        self._lap_tracker: LapProgressTracker | None = None
        if self._telemetry is not None and self._speed_profile is not None:
            self._lap_tracker = LapProgressTracker(*self._speed_profile)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(PoseArray, '/fsae/planning/selected_trajectory', self._traj_cb, 10)
        self.create_subscription(PoseStamped, '/fsae/slam/car_position', self._pose_cb, 10)
        self.create_subscription(Odometry, '/fsds/testing_only/odom', self._odom_cb, sensor_qos)

        self.pub_cmd = self.create_publisher(AckermannDriveStamped, '/fsae/control/cmd_vel', 10)

        self._path: np.ndarray = np.empty((0, 2))
        self._path_stamp = None
        self._car_pos = np.zeros(2)
        self._car_yaw = 0.0
        self._car_speed = 0.0
        self._car_yaw_rate = 0.0

        self._stanley = StanleyController(k_cte=k_cte)

        self.get_logger().info('controller ready — waiting for a trajectory + car_position.')

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _traj_cb(self, msg: PoseArray) -> None:
        self._path = np.array(
            [[p.position.x, p.position.y] for p in msg.poses], dtype=np.float64
        ) if msg.poses else np.empty((0, 2))
        self._path_stamp = self.get_clock().now()

    def _odom_cb(self, msg: Odometry) -> None:
        v = msg.twist.twist.linear
        self._car_speed = float(np.hypot(v.x, v.y))
        self._car_yaw_rate = float(msg.twist.twist.angular.z)

    def _pose_cb(self, msg: PoseStamped) -> None:
        self._car_pos = np.array([msg.pose.position.x, msg.pose.position.y])
        self._car_yaw = float(msg.pose.orientation.w)
        self._control_step()

    # ------------------------------------------------------------------
    # Control step (triggered by car_position)
    # ------------------------------------------------------------------

    def _control_step(self) -> None:
        _t_loop0 = time.perf_counter()
        if len(self._path) < 2:
            return

        steering = self._stanley.compute(
            self._path, self._car_pos, self._car_yaw,
            self._car_speed, self._car_yaw_rate,
        )
        if self._speed_profile is not None:
            # Same oracle-speed bypass as mpc_controller.py's map_path, so a
            # Stanley run and an MPC run on the same track use the identical
            # speed target and differ only in steering behaviour.
            path_X, path_Y, path_V = self._speed_profile
            speed = precomputed_speed_at(self._car_pos, path_X, path_Y, path_V)
        else:
            speed = curvature_speed(self._path, v_max=self._v_max, v_min=self._v_min)

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        self.pub_cmd.publish(msg)

        if self._telemetry is not None:
            t = self.get_clock().now().nanoseconds * 1e-9
            path_age_s = (
                (self.get_clock().now() - self._path_stamp).nanoseconds * 1e-9
                if self._path_stamp is not None else None
            )
            self._telemetry.log_control(
                t, self._car_pos[0], self._car_pos[1], self._car_yaw,
                self._car_speed, speed, steering,
                self._stanley.last_e_y, self._stanley.last_e_psi, self._car_yaw_rate,
                path_age_s=path_age_s,
                cmd_latency_ms=(time.perf_counter() - _t_loop0) * 1e3,
            )
            self._telemetry.log_path(t, self._path)
            if self._lap_tracker is not None:
                self._lap_tracker.update(self._car_pos, t)

        self.get_logger().info(
            f'cmd_vel: speed={speed:.2f} m/s  steer={steering:.3f} rad  '
            f'v_actual={self._car_speed:.2f} m/s',
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = StanleyControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._telemetry is not None:
            if node._lap_tracker is not None:
                lap = node._lap_tracker.result(node.get_clock().now().nanoseconds * 1e-9)
                node._telemetry.close(
                    progress=lap['progress'], time_bonus=lap['time_bonus'],
                    reached_end=lap['reached_end'], lap_time_s=lap['lap_time_s'],
                    optimal_time_s=lap['optimal_time_s'],
                )
            else:
                node._telemetry.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
