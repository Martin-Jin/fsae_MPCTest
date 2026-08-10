"""
MPC path-tracking controller.

A drop-in alternative to the Stanley controller: it follows the planned
centreline and emits the same drive command on the car stack's control
interface (target speed + steering angle), so the downstream FSDS conversion in
fsds_bridge (speed->throttle/brake, GO gating, cone e-brake) is unchanged.

    in   /fsae/planning/selected_trajectory  geometry_msgs/PoseArray   path to follow
    in   /fsae/slam/car_position             geometry_msgs/PoseStamped x,y in position; yaw in orientation.w
    in   /fsae/slam/car_odom                 nav_msgs/Odometry         speed + yaw-rate feedback; SAME
                                                                        snapshot as car_position above (see
                                                                        sim_perception.py's "Speed/yaw-rate
                                                                        synchronisation" docstring note — do
                                                                        NOT subscribe to the raw
                                                                        /fsds/testing_only/odom directly)
    out  /fsae/control/cmd_vel               ackermann_msgs/AckermannDriveStamped  speed + steering_angle

The optimiser (mpc_core.MPCController) is a linear time-varying MPC ported from
the fsae_MPCTest repo.  Unlike Stanley (which reacts to the instantaneous
cross-track/heading error), the MPC plans a 1.25 s horizon, which is what damps
the high-speed left-right sway.  It natively outputs throttle/brake too, but to
preserve the cmd_vel -> fsds_bridge abstraction (and keep fsds_bridge the single
owner of GO-gating and cone-braking) we forward only its steering angle (rad)
and let the curvature-limited target speed drive the bridge's speed controller.

The control step runs on a FIXED 20 Hz timer (not the pose callback like
Stanley), because the MPC's discretisation assumes a constant dt = 0.05 s
between solves.
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.time import Time

from fsae_control.control_utils import (
    curvature_speed, load_path_profile_csv, load_speed_profile_csv,
    precomputed_speed_at, tracking_error_speed_gate,
)
from fsae_control.mpc_core import MPCController
from fsae_control.telemetry_logger import ControlLogger

CONTROL_HZ    = 20.0   # must match MPCController(dt=0.05); dt = 1 / CONTROL_HZ

# Max rate (m/s^2) at which the speed TARGET may rise. Mirrors
# mpc_controller_standalone.SPEED_TARGET_RISE_RATE and
# sim/rollout_core.SPEED_TARGET_RISE_RATE — keep all three in sync. Decreases
# are never rate-limited; delaying a genuine brake request is the failure this
# is meant to prevent.
SPEED_TARGET_RISE_RATE = 2.0
# Max rate (gate-units/s) at which tracking_error_speed_gate()'s output may
# change per tick, in either direction. Mirrors
# mpc_controller_standalone.GATE_RATE_LIMIT — keep both in sync, see that
# constant's own comment for the rationale.
GATE_RATE_LIMIT = 2.0
PATH_TIMEOUT  = 0.5    # s — reset the MPC if no fresh trajectory within this window


class MPCControllerNode(Node):
    def __init__(self):
        super().__init__('controller')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('v_max', 15.0),      # m/s — top speed on straights
                ('v_min', 1.5),       # m/s — minimum speed through tight corners
                ('steer_lp', 0.3),    # output steering low-pass (EMA); 1.0 disables
                ('log_csv', False),   # write CSV telemetry to log_dir
                ('log_dir', ''),      # '' -> ~/fsae_logs
                ('map_path', ''),     # '' -> live curvature_speed() (default);
                                       # else a fsae_MPCTest tuner/export_speed_profile.py
                                       # CSV to use instead — see mpc_controller_standalone.py's
                                       # map_path param.
                ('path_map_path', ''),  # '' -> live /fsae/planning/selected_trajectory
                                       # (default); else the SAME kind of CSV as map_path,
                                       # used for the tracked PATH instead of just speed —
                                       # see mpc_controller_standalone.py's path_map_path param
                                       # and USE_PLANNER=False in fsae_MPCTest/settings.py (the
                                       # offline equivalent -- no separate flag exists there).
            ],
        )
        self._v_max = self.get_parameter('v_max').get_parameter_value().double_value
        self._v_min = self.get_parameter('v_min').get_parameter_value().double_value
        self._steer_lp = self.get_parameter('steer_lp').get_parameter_value().double_value

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

        # Static precomputed path — see mpc_controller_standalone.py's
        # identical field for the full rationale/safety discussion.
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
        self._delta_filt: float | None = None   # filtered steering state

        self._telemetry = None
        if self.get_parameter('log_csv').get_parameter_value().bool_value:
            log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
            self._telemetry = ControlLogger('mpc', log_dir=log_dir)
            self.get_logger().info(f'CSV telemetry -> {self._telemetry.paths[0]}')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(PoseArray, '/fsae/planning/selected_trajectory', self._traj_cb, 10)
        self.create_subscription(PoseStamped, '/fsae/slam/car_position', self._pose_cb, 10)
        # /fsae/slam/car_odom, NOT the raw /fsds/testing_only/odom -- see this
        # file's own docstring and sim_perception.py's "Speed/yaw-rate
        # synchronisation" note. The raw topic raced sim_perception's own
        # separate subscription to the same 250 Hz publisher, so car_speed/
        # car_yaw_rate could reflect a different odom instant than
        # car_pos/car_yaw on any given tick.
        self.create_subscription(Odometry, '/fsae/slam/car_odom', self._odom_cb, sensor_qos)

        self.pub_cmd = self.create_publisher(AckermannDriveStamped, '/fsae/control/cmd_vel', 10)

        self._path: np.ndarray = (
            self._static_path if self._static_path is not None else np.empty((0, 2))
        )
        # Static path never goes stale (no topic to lose) — see the
        # standalone node's identical field/comment.
        self._path_stamp = None
        self._car_pos = np.zeros(2)
        # Previous tick's speed target, for the rise-rate limiter. None = no
        # history (start, or after a fail-safe reset) -> first target passes
        # through unlimited instead of ramping up from zero.
        self._v_des_prev: float | None = None
        # Previous tick's tracking-error speed gate, for GATE_RATE_LIMIT below.
        self._gate_prev: float | None = None
        self._car_yaw = 0.0
        self._car_speed = 0.0
        self._car_vy = 0.0
        self._car_yaw_rate = 0.0
        self._pose_stamp = None
        self._have_pose = False

        dt = 1.0 / CONTROL_HZ
        self._mpc = MPCController(dt=dt, N=35)

        self.create_timer(dt, self._control_step)

        self.get_logger().info('MPC controller ready — waiting for a trajectory + car_position.')

    # ------------------------------------------------------------------
    # Subscribers (cache latest state; the timer does the work)
    # ------------------------------------------------------------------

    def _traj_cb(self, msg: PoseArray) -> None:
        if self._static_path is not None:
            # A precomputed path is active — ignore the live planner's output.
            # See mpc_controller_standalone.py's _path_cb for the same guard.
            return
        self._path = np.array(
            [[p.position.x, p.position.y] for p in msg.poses], dtype=np.float64
        ) if msg.poses else np.empty((0, 2))
        self._path_stamp = self.get_clock().now()

    def _odom_cb(self, msg: Odometry) -> None:
        # v.x/v.y are body-frame (sim_perception.py relays them unrotated
        # from the bridge's already-body-frame odom) -- keep both instead of
        # collapsing to hypot(), which silently drops the vy*cos(e_psi) term
        # _error_state needs (see mpc_core.py's e_yd comment).
        v = msg.twist.twist.linear
        self._car_speed = float(v.x)
        self._car_vy = float(v.y)
        self._car_yaw_rate = float(msg.twist.twist.angular.z)

    def _pose_cb(self, msg: PoseStamped) -> None:
        # x,y in position; yaw (rad) is stuffed into orientation.w (upstream convention).
        self._car_pos = np.array([msg.pose.position.x, msg.pose.position.y])
        self._car_yaw = float(msg.pose.orientation.w)
        self._pose_stamp = msg.header.stamp
        self._have_pose = True

    # ------------------------------------------------------------------
    # Control step (fixed 20 Hz)
    # ------------------------------------------------------------------

    def _control_step(self) -> None:
        # Loop-entry timestamp for the cmd_latency_ms telemetry column — see
        # telemetry_logger's latency-diagnostics note.
        _t_loop0 = time.perf_counter()
        # No pose or no path yet, or the path has gone stale: publish nothing and
        # reset the MPC so it doesn't warm-start from a stale trajectory.  The
        # downstream fsds_bridge brakes on its own cmd_vel timeout.
        if self._static_path is not None:
            # No topic backing this path, so "staleness" doesn't apply — see
            # mpc_controller_standalone.py's Phase 2 for the same reasoning.
            path_stale = False
        else:
            path_stale = (
                self._path_stamp is None
                or (self.get_clock().now() - self._path_stamp).nanoseconds * 1e-9 > PATH_TIMEOUT
            )
        if not self._have_pose or len(self._path) < 2 or path_stale:
            self._mpc.reset()
            self._delta_filt = None   # drop filter state with the MPC warm-start
            self._v_des_prev = None   # don't ramp from a pre-fail-safe target
            self._gate_prev = None    # ditto for the tracking-error speed gate
            return

        # Slice from the car's nearest point, gate on tracking error, and
        # rate-limit rises — identical to mpc_controller_standalone.py's
        # Phase 3; see that file and control_utils.tracking_error_speed_gate
        # for the rationale behind each. This node forwards the result
        # through cmd_vel to fsds_bridge's speed loop rather than using the
        # MPC's own accel, but the speed TARGET must be derived the same way
        # in both nodes or they diverge in exactly the regime that matters.
        if self._speed_profile is not None:
            # Track is already fully mapped (map_path param set) — look up
            # the oracle speed target instead of re-deriving it from the
            # live-built centreline. See load_speed_profile_csv()'s docstring.
            path_X, path_Y, path_V = self._speed_profile
            v_curv = precomputed_speed_at(self._car_pos, path_X, path_Y, path_V)
        else:
            path_ahead = self._path
            if len(path_ahead) > 2:
                i_near = int(np.argmin(np.linalg.norm(path_ahead - self._car_pos, axis=1)))
                if i_near < len(path_ahead) - 2:
                    path_ahead = path_ahead[i_near:]

            v_curv = curvature_speed(path_ahead, v_max=self._v_max, v_min=self._v_min)

        # Gate's own output is rate-limited (GATE_RATE_LIMIT) so its
        # tick-to-tick change is bounded — see that constant's comment and
        # mpc_controller_standalone.py's identical logic.
        tel = self._mpc.last_telemetry
        raw_gate = tracking_error_speed_gate(tel.get('e_y', 0.0), tel.get('e_psi', 0.0))
        if self._gate_prev is not None:
            max_step = GATE_RATE_LIMIT / CONTROL_HZ
            raw_gate = float(np.clip(raw_gate, self._gate_prev - max_step, self._gate_prev + max_step))
        self._gate_prev = raw_gate
        gate = raw_gate
        desired_speed = max(self._v_min, v_curv * gate)

        if self._v_des_prev is not None:
            desired_speed = min(desired_speed,
                                self._v_des_prev + SPEED_TARGET_RISE_RATE / CONTROL_HZ)
        self._v_des_prev = desired_speed

        # Age of the pose the MPC is about to solve against — how long ago it
        # was actually measured, not how long ago the callback fired. Lets
        # MPCController compensate for the real, unknown/time-varying delay
        # instead of assuming the state is fresh (see mpc_core.py compute()).
        pose_age_s = (self.get_clock().now() - Time.from_msg(self._pose_stamp)).nanoseconds * 1e-9

        # The MPC returns FSDS-normalised (steering, throttle, brake); we take its
        # pre-normalisation steering angle in radians (last_telemetry['delta_cmd'],
        # +ve = left, already clamped to the 25deg limit) and forward the
        # curvature-limited speed, keeping the cmd_vel abstraction intact.
        self._mpc.compute(
            self._path, self._car_pos, self._car_yaw,
            self._car_speed, desired_speed, self._car_yaw_rate,
            pose_age_s=pose_age_s, car_vy=self._car_vy,
        )
        steering = float(self._mpc.last_telemetry.get('delta_cmd', 0.0))

        # Low-pass the steering command across ticks (matches the Stanley node) so
        # rapid left-right jitter never reaches the servo.  1.0 disables.
        if self._delta_filt is None or self._steer_lp >= 1.0:
            self._delta_filt = steering
        else:
            self._delta_filt += self._steer_lp * (steering - self._delta_filt)
        steering = self._delta_filt

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(desired_speed)
        msg.drive.steering_angle = steering
        self.pub_cmd.publish(msg)

        if self._telemetry is not None:
            tel = self._mpc.last_telemetry
            t = self.get_clock().now().nanoseconds * 1e-9
            # steering is already the roadwheel angle in radians here (this
            # node publishes an Ackermann steering_angle, not a normalised
            # FSDS command), so it is both the logged steer and delta_cmd.
            # See mpc_controller_standalone.py's identical logic for why a
            # static path logs 0.0 rather than None.
            if self._static_path is not None:
                path_age_s = 0.0
            elif self._path_stamp is not None:
                path_age_s = (self.get_clock().now() - self._path_stamp).nanoseconds * 1e-9
            else:
                path_age_s = None
            self._telemetry.log_control(
                t, self._car_pos[0], self._car_pos[1], self._car_yaw,
                self._car_speed, desired_speed, steering,
                tel.get('e_y', 0.0), tel.get('e_psi', 0.0), self._car_yaw_rate,
                delta_cmd=steering, a_cmd=tel.get('a_cmd', 0.0),
                pose_age_s=tel.get('pose_age_s'),
                path_age_s=path_age_s,
                n_delay=tel.get('n_delay'),
                solve_ms=tel.get('solve_ms'),
                cmd_latency_ms=(time.perf_counter() - _t_loop0) * 1e3,
                adaptive=tel)
            self._telemetry.log_path(t, self._path)

        self.get_logger().info(
            f'cmd_vel: speed={desired_speed:.2f} m/s  steer={steering:.3f} rad  '
            f'v_actual={self._car_speed:.2f} m/s  '
            f'e_y={self._mpc.last_telemetry.get("e_y", 0.0):.2f}',
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._telemetry is not None:
            node._telemetry.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
