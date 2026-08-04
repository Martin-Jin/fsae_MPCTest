"""
MPC path-tracking controller — standalone variant (bypasses fsds_bridge).

mpc_controller.py (this package's default `mpc` controller) forwards only the
MPC's steering command through the shared cmd_vel abstraction; fsds_bridge.py
computes throttle/brake from a simple speed-error P-loop against a
curvature-limited target, the same way it does for the Stanley controller.

This node is a DIFFERENT integration of the same MPCController
(mpc_core.MPCController): it publishes fs_msgs/ControlCommand directly, using
the MPC's own (steering, throttle, brake) output unchanged. That preserves the
offline-tuned longitudinal behaviour from the fsae_MPCTest repo's
tuner/offline_tuner.py and gui/simulation.py, which both drive the vehicle
plant with the MPC's own commanded acceleration (see that repo's
sim/rollout_core.py) — mpc_controller.py's accel-discarding design does not.

Ported from fsae_MPCTest's control_node.py (mirrored at
fsds_simulator/control/fsae_control/fsae_control/mpc_controller_standalone.py
in that repo, alongside its own copies of mpc_core.py/control_utils.py — that
directory is a staging area mirroring this package's own hierarchy, kept for
porting purposes only), which implements the same design against an older
ROS 2 topic/message interface; this is that same node updated for the
current fsae_planning topics/messages.

    in   /fsae/planning/selected_trajectory  geometry_msgs/PoseArray        planner centreline
    in   /fsae/slam/car_position             geometry_msgs/Pose             x,y in position; yaw in orientation.w
    in   /fsds/testing_only/odom             nav_msgs/Odometry              speed + yaw-rate feedback
    in   /fsds/signal/go                     fs_msgs/GoSignal               race start
    in   /fsae/perception/cone_detection     fsae_interfaces/ConeDetection  proximity e-brake (car-local frame)
    out  /fsds/control_command                fs_msgs/ControlCommand

Because this node owns GO-gating, stale-command braking, and cone-proximity
braking itself (mirroring fsds_bridge.py's own logic against the same inputs),
do NOT launch fsds_bridge.py alongside this node — its output would be
published but never used, and it would race this node for
/fsds/control_command. Select this controller with `controller:=mpc_standalone`
in control.launch.py, which skips fsds_bridge for that mode.

CONTROL LOOP PHASES (see _control_loop)
----------------------------------------------------------------------------
  Phase 1 — Hold at start line until GO signal received.
  Phase 2 — Emergency brake if the planner path is missing/stale (>TARGET_TIMEOUT
            old) or has fewer than 2 points, or the SLAM pose hasn't arrived yet;
            also resets the MPC so it doesn't warm-start from a stale trajectory
            once the path returns.
  Phase 3 — Normal MPC solve via MPCController.compute().
  Phase 4 — Cone-proximity brake override: hard-overrides the MPC's
            throttle/brake (not steering) if a cone is inside the dynamic
            braking corridor. After CONE_RESET_THRESHOLD seconds of
            continuous braking, the MPC is reset exactly once (edge-triggered
            on the rising duration threshold, re-armed once the brake clears).
  Phase 4a — Telemetry logging of the *final* (post-override) command.
  Phase 5 — Publish.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from fs_msgs.msg import ControlCommand, GoSignal
from fsae_interfaces.msg import ConeDetection
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry

from fsae_control.control_utils import curvature_speed
from fsae_control.mpc_core import MPCController
from fsae_control.telemetry_logger import ControlLogger

CONTROL_HZ = 20.0   # must match MPCController(dt=0.05); dt = 1 / CONTROL_HZ

# CONE_BRAKE_DIST is also the ceiling on the dynamic corridor computed in
# _check_cone_proximity() (car_speed * 0.25, clipped to [0.6, CONE_BRAKE_DIST]).
CONE_BRAKE_DIST      = 2.0    # m — forward corridor depth for cone proximity brake
CONE_BRAKE_WIDTH     = 0.18   # m — lateral half-width of braking corridor (36 cm total)
CONE_RESET_THRESHOLD = 0.3    # s — continuous cone-brake duration before one MPC reset
PATH_TIMEOUT         = 0.5    # s — reset the MPC if no fresh trajectory within this window


class MPCControllerStandaloneNode(Node):
    def __init__(self):
        super().__init__('controller')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('v_max', 20.0),      # m/s — top speed on straights
                ('v_min', 1.5),       # m/s — minimum speed through tight corners
                ('log_csv', False),   # write CSV telemetry to log_dir
                ('log_dir', ''),      # '' -> ~/fsae_logs
            ],
        )
        self._v_max = self.get_parameter('v_max').get_parameter_value().double_value
        self._v_min = self.get_parameter('v_min').get_parameter_value().double_value

        self._telemetry = None
        if self.get_parameter('log_csv').get_parameter_value().bool_value:
            log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
            self._telemetry = ControlLogger('mpc_standalone', log_dir=log_dir)
            self.get_logger().info(f'CSV telemetry -> {self._telemetry.paths[0]}')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            PoseArray, '/fsae/planning/selected_trajectory', self._path_cb, 10)
        self.create_subscription(
            Pose, '/fsae/slam/car_position', self._pose_cb, 10)
        self.create_subscription(
            Odometry, '/fsds/testing_only/odom', self._odom_cb, sensor_qos)
        self.create_subscription(
            GoSignal, '/fsds/signal/go', self._go_cb, 10)
        self.create_subscription(
            ConeDetection, '/fsae/perception/cone_detection', self._cone_cb, 10)

        self.pub_cmd = self.create_publisher(ControlCommand, '/fsds/control_command', 10)

        self._go_received = False
        self._path_pts: np.ndarray = np.empty((0, 2))
        self._path_stamp = None
        self._have_pose = False
        self._car_pos = np.zeros(2)
        self._car_yaw = 0.0
        self._car_speed = 0.0
        self._car_yaw_rate = 0.0
        self._desired_speed = 0.0

        # Cones arrive already in the car-LOCAL frame (see ConeDetection.msg /
        # fsae_sim_perception's sim_perception.py), so no car_pos/car_yaw
        # transform is needed for the proximity check below.
        self._cones_local: np.ndarray = np.empty((0, 2))
        self._cone_brake_duration = 0.0
        self._cone_reset_done = False

        dt = 1.0 / CONTROL_HZ
        self._mpc = MPCController(dt=dt, N=25)

        self.create_timer(dt, self._control_loop)
        self.get_logger().info(
            'MPC standalone controller ready — publishing ControlCommand directly '
            '(fsds_bridge is NOT used in this mode).'
        )

    # ------------------------------------------------------------------
    # Subscribers (cache latest state; the timer does the work)
    # ------------------------------------------------------------------

    def _go_cb(self, msg: GoSignal) -> None:
        if not self._go_received:
            self._go_received = True
            self.get_logger().info('GO signal received.')

    def _path_cb(self, msg: PoseArray) -> None:
        self._path_pts = np.array(
            [[p.position.x, p.position.y] for p in msg.poses], dtype=np.float64
        ) if msg.poses else np.empty((0, 2))
        self._path_stamp = self.get_clock().now()

    def _pose_cb(self, msg: Pose) -> None:
        # x,y in position; yaw (rad) is stuffed into orientation.w (upstream convention).
        self._car_pos = np.array([msg.position.x, msg.position.y])
        self._car_yaw = float(msg.orientation.w)
        self._have_pose = True

    def _odom_cb(self, msg: Odometry) -> None:
        v = msg.twist.twist.linear
        self._car_speed = float(np.hypot(v.x, v.y))
        self._car_yaw_rate = float(msg.twist.twist.angular.z)

    def _cone_cb(self, msg: ConeDetection) -> None:
        pts = [[p.x, p.y] for p in msg.blue] + [[p.x, p.y] for p in msg.yellow]
        self._cones_local = np.array(pts, dtype=np.float64) if pts else np.empty((0, 2))

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _check_cone_proximity(self) -> bool:
        """True if a cone sits inside the dynamic forward braking corridor."""
        if len(self._cones_local) == 0:
            return False
        x_car = self._cones_local[:, 0]   # forward (+)
        y_car = self._cones_local[:, 1]   # left    (+)
        dynamic_brake_dist = float(np.clip(self._car_speed * 0.25, 0.6, CONE_BRAKE_DIST))
        return bool(np.any(
            (x_car > 0.2) & (x_car < dynamic_brake_dist) & (np.abs(y_car) < CONE_BRAKE_WIDTH)
        ))

    def _publish(self, cmd: ControlCommand) -> None:
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.pub_cmd.publish(cmd)

    # ------------------------------------------------------------------
    # Control step (fixed 20 Hz)
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        cmd = ControlCommand()

        # ── Phase 1: hold until GO ──────────────────────────────────────
        if not self._go_received:
            cmd.throttle, cmd.steering, cmd.brake = 0.0, 0.0, 1.0
            self._publish(cmd)
            self.get_logger().info('Waiting for GO signal...', throttle_duration_sec=2.0)
            return

        # ── Phase 2: emergency brake on stale/missing path or pose ──────
        path_stale = (
            self._path_stamp is None
            or (self.get_clock().now() - self._path_stamp).nanoseconds * 1e-9 > PATH_TIMEOUT
        )
        if not self._have_pose or path_stale or len(self._path_pts) < 2:
            cmd.throttle, cmd.steering, cmd.brake = 0.0, 0.0, 1.0
            self._mpc.reset()
            self._publish(cmd)
            self.get_logger().warn(
                'Trajectory path lost or stale — emergency braking.', throttle_duration_sec=1.0)
            return

        # ── Phase 3: MPC solve ───────────────────────────────────────────
        self._desired_speed = curvature_speed(self._path_pts, v_max=self._v_max, v_min=self._v_min)

        steering, throttle, brake = self._mpc.compute(
            path=self._path_pts,
            car_pos=self._car_pos,
            car_yaw=self._car_yaw,
            car_speed=self._car_speed,
            desired_speed=self._desired_speed,
            car_yaw_rate=self._car_yaw_rate,
        )
        cmd.steering, cmd.throttle, cmd.brake = steering, throttle, brake

        # ── Phase 4: cone-proximity brake override ──────────────────────
        if self._check_cone_proximity():
            cmd.throttle = 0.0
            cmd.brake = 1.0
            self._cone_brake_duration += 1.0 / CONTROL_HZ
            if self._cone_brake_duration >= CONE_RESET_THRESHOLD and not self._cone_reset_done:
                self._mpc.reset()
                self._cone_reset_done = True
            self.get_logger().warn(
                f'Cone proximity brake active ({self._cone_brake_duration:.2f} s).',
                throttle_duration_sec=0.5,
            )
        else:
            self._cone_brake_duration = 0.0
            self._cone_reset_done = False

        # ── Phase 4a: telemetry (post-override, reflects the final cmd) ─
        if self._telemetry is not None:
            tel = self._mpc.last_telemetry
            t = self.get_clock().now().nanoseconds * 1e-9
            self._telemetry.log_control(
                t, self._car_pos[0], self._car_pos[1], self._car_yaw,
                self._car_speed, self._desired_speed, cmd.steering,
                tel.get('e_y', 0.0), tel.get('e_psi', 0.0), self._car_yaw_rate)
            self._telemetry.log_path(t, self._path_pts)

        # ── Phase 5: publish ─────────────────────────────────────────────
        self._publish(cmd)

        self.get_logger().info(
            f'MPC thr={cmd.throttle:.2f} brk={cmd.brake:.2f} steer={cmd.steering:.3f} | '
            f'v={self._car_speed:.1f}/{self._desired_speed:.1f} m/s',
            throttle_duration_sec=1.0,
        )

    def destroy_node(self) -> None:
        if self._telemetry is not None:
            self._telemetry.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerStandaloneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
