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
    StanleyController, curvature_speed, load_path_profile_csv,
    load_speed_profile_csv, precomputed_speed_at, tracking_error_speed_gate,
)
from fsae_control.telemetry_logger import ControlLogger, LapProgressTracker, build_config_lines

# Same three safeguards mpc_controller.py wraps around curvature_speed()'s
# raw output, ported here after live testing showed Stanley stuttering (a
# single noisy live-refit path tick can swing v_curv 3-10 m/s and spike
# steering error simultaneously, with nothing here to absorb either) --
# see mpc_controller.py's own V_CURV_FALL_RATE/SPEED_TARGET_RISE_RATE/
# GATE_RATE_LIMIT comments for the full rationale. Stanley has no fixed
# control-loop timer (it runs off car_position arrival, not CONTROL_HZ), so
# the rate limits below are applied per measured dt, not a compile-time
# tick period.
V_CURV_FALL_RATE = 7.0     # m/s^2 -- max rate curvature_speed()'s output may FALL
SPEED_TARGET_RISE_RATE = 7.0   # m/s^2 -- max rate the composed target may RISE
GATE_RATE_LIMIT = 2.0      # 1/s -- max rate tracking_error_speed_gate()'s output may change, either direction


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
                ('path_map_path', ''),  # '' -> live /fsae/planning/selected_trajectory
                                       # (default); else the SAME kind of CSV as map_path
                                       # (e.g. a raceline.csv), used for the tracked PATH
                                       # instead of just speed — see mpc_controller.py's
                                       # identical param.
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

        # Static precomputed path — see mpc_controller.py's identical field
        # for the full rationale/safety discussion.
        self._static_path: np.ndarray | None = None
        path_map_path = self.get_parameter('path_map_path').get_parameter_value().string_value
        if path_map_path:
            try:
                self._static_path = load_path_profile_csv(path_map_path)
                self.get_logger().info(
                    f'Loaded precomputed path ({len(self._static_path)} pts) from '
                    f'{path_map_path} — planner output on /fsae/planning/selected_trajectory '
                    'will be ignored.'
                )
            except (OSError, ValueError) as exc:
                self.get_logger().error(
                    f'Failed to load path_map_path={path_map_path}: {exc}. '
                    'Falling back to the live planner topic.'
                )

        self._telemetry = None
        if self.get_parameter('log_csv').get_parameter_value().bool_value:
            log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
            self._telemetry = ControlLogger('stanley', log_dir=log_dir)
            self.get_logger().info(f'CSV telemetry -> {self._telemetry.paths[0]}')
            # Stanley has no MPCParams/NMPCParams (no adaptive gain schedule,
            # no NMPC) -- just the controller name, gain, and path source, so
            # a run can still be told apart from an MPC/NMPC one at a glance.
            self._telemetry.set_config_lines(build_config_lines(
                controller='stanley',
                launch_flags={
                    'stanley_gain': k_cte,
                    'map_path': map_path, 'path_map_path': path_map_path,
                },
            ))

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

        self._path: np.ndarray = (
            self._static_path if self._static_path is not None else np.empty((0, 2))
        )
        # Static path never goes stale (no topic to lose) — see
        # mpc_controller.py's identical field/comment.
        self._path_stamp = None
        self._car_pos = np.zeros(2)
        self._car_yaw = 0.0
        self._car_speed = 0.0
        self._car_yaw_rate = 0.0

        self._stanley = StanleyController(k_cte=k_cte)

        # State for the three rate limiters below -- see their module-level
        # constants' comments. All reset to None on a stale/lost-path event
        # (mirroring mpc_controller.py) so a fresh start doesn't inherit a
        # rate limit computed against a stale previous tick.
        self._v_curv_prev: float | None = None
        self._gate_prev: float | None = None
        self._v_des_prev: float | None = None
        self._last_tick_time: float | None = None

        self.get_logger().info('controller ready — waiting for a trajectory + car_position.')

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _traj_cb(self, msg: PoseArray) -> None:
        if self._static_path is not None:
            # A precomputed path is active — ignore the live planner's output.
            # See mpc_controller.py's identical guard in its own _traj_cb.
            return
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
            # No path to track -- drop rate-limiter state so a fresh start
            # once a path arrives doesn't inherit a limit computed against a
            # now-meaningless previous tick (mirrors mpc_controller.py's
            # equivalent reset on its own path_stale/no-pose branch).
            self._v_curv_prev = None
            self._gate_prev = None
            self._v_des_prev = None
            self._last_tick_time = None
            return

        steering = self._stanley.compute(
            self._path, self._car_pos, self._car_yaw,
            self._car_speed, self._car_yaw_rate,
        )

        now = time.perf_counter()
        dt = (now - self._last_tick_time) if self._last_tick_time is not None else None
        self._last_tick_time = now

        if self._speed_profile is not None:
            # Same oracle-speed bypass as mpc_controller.py's map_path, so a
            # Stanley run and an MPC run on the same track use the identical
            # speed target and differ only in steering behaviour. The oracle
            # lookup is not re-derived from a noisy live path, so none of the
            # rate limiters below apply to this branch either (matches
            # mpc_controller.py).
            path_X, path_Y, path_V = self._speed_profile
            speed = precomputed_speed_at(self._car_pos, path_X, path_Y, path_V)
        else:
            v_curv = curvature_speed(self._path, v_max=self._v_max, v_min=self._v_min)

            # curvature_speed() has no memory of its own last output and the
            # live path is re-fit every tick -- see V_CURV_FALL_RATE's own
            # comment above and mpc_controller.py's identical mechanism.
            if self._v_curv_prev is not None and dt is not None:
                v_curv = max(v_curv, self._v_curv_prev - V_CURV_FALL_RATE * dt)
            self._v_curv_prev = v_curv

            # Scale down when tracking is already bad, so a controller that's
            # off-line doesn't keep getting told to go fast -- see
            # tracking_error_speed_gate()'s own docstring. Gate's own output
            # is rate-limited (GATE_RATE_LIMIT) same as mpc_controller.py.
            raw_gate = tracking_error_speed_gate(self._stanley.last_e_y, self._stanley.last_e_psi)
            if self._gate_prev is not None and dt is not None:
                max_step = GATE_RATE_LIMIT * dt
                raw_gate = float(np.clip(raw_gate, self._gate_prev - max_step, self._gate_prev + max_step))
            self._gate_prev = raw_gate
            # Never gate below v_min: the car still needs authority to steer back.
            speed = max(self._v_min, v_curv * raw_gate)

            # Bound the composed target's RISE the same way mpc_controller.py
            # does -- SPEED_TARGET_RISE_RATE. Seed from the car's actual speed
            # on the first tick so a standing start doesn't jump straight to
            # the full target.
            if self._v_des_prev is None:
                self._v_des_prev = self._car_speed
            if dt is not None:
                speed = min(speed, self._v_des_prev + SPEED_TARGET_RISE_RATE * dt)
            self._v_des_prev = speed

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
                self._lap_tracker.update(self._car_pos, t, self._car_speed)

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
