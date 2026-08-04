"""
control_node.py — ROS 2 Real-Time Control Node for FSDS

PURPOSE
-------
The live, in-sim ROS 2 node that wraps MPCController
(mpc_core.py) in a 20 Hz control loop. It is the only place in the
codebase where the MPC is driven by real sensor topics rather than the
offline simulator (simulation.py) or the tuner (offline_tuner.py).

INTERFACE (matches the current fsae_planning car stack)
----------------------------------------------------------------------------
    in   /fsae/planning/selected_trajectory  geometry_msgs/PoseArray        planner centreline
    in   /fsae/slam/car_position             geometry_msgs/Pose             x,y in position; yaw in orientation.w
    in   /fsds/testing_only/odom             nav_msgs/Odometry              speed + yaw-rate feedback
    in   /fsds/signal/go                     fs_msgs/GoSignal               race start
    in   /fsae/perception/cone_detection     fsae_interfaces/ConeDetection  proximity e-brake (car-local frame)
    out  /fsds/control_command                fs_msgs/ControlCommand

DELIBERATE DEVIATION FROM fsae_planning's mpc_controller.py
----------------------------------------------------------------------------
fsae_planning's mpc_controller.py forwards only the MPC's steering command and
lets fsds_bridge.py's simple speed-error P-loop compute throttle/brake against
a curvature-limited target speed. This node instead publishes ControlCommand
directly and uses MPCController.compute()'s own (steering, throttle, brake)
output unchanged, preserving the offline-tuned longitudinal behaviour from
tuner/offline_tuner.py (which also drives the plant with the MPC's own a_cmd —
see sim/rollout_core.py). Consequently this node does NOT go through
fsds_bridge.py: GO-gating, stale-command braking, and cone-proximity braking
are re-implemented here instead of relying on the bridge, matching this
repo's previous control_node.py design. Don't launch fsds_bridge.py alongside
this node — its output would be published but never used, and it would fight
this node for the /fsds/control_command topic.

Responsibilities:
  1. Subscribe to the planner path, SLAM car pose, odometry, fused cone
     detections, and the race "GO" signal. Desired speed is not subscribed
     from a topic — it is computed locally every tick from the current path
     via control_utils.curvature_speed(), mirroring sim_track.SimPlanner's
     offline speed profile.
  2. Maintain the latest vehicle state (position, yaw, speed, yaw rate)
     and the latest planned path.
  3. Run a fixed 20 Hz timer that calls MPCController.compute() and
     publishes a ControlCommand.
  4. Apply two independent safety overrides on top of the MPC output:
       a. GO-wait hold (Phase 1) — full brake until the race start signal.
       b. Stale-path emergency brake (Phase 2) — full brake + MPC reset if
          no fresh path has been received within TARGET_TIMEOUT.
       c. Cone-proximity brake (Phase 4) — full brake if any fused cone is
          inside a dynamic corridor directly ahead of the car.
  5. Optionally log per-step telemetry (state, error terms, and the final
     published command) to CSV for offline analysis, mirroring the
     LOG_FIELDS also produced by scoring.py / performance_stats.py.

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

USED BY
-------
  Launched as the `controller` ROS 2 node entry point (see main()). Depends
  on MPCController from mpc_core.py for all optimal-control computation;
  the node itself contains no MPC/optimisation logic.
"""

import csv
import math
import time
from pathlib import Path as FsPath

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from fs_msgs.msg import ControlCommand, GoSignal
from fsae_interfaces.msg import ConeDetection
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry

from .control_utils import curvature_speed
from .mpc_core import MPCController

# ── Tuneable constants ─────────────────────────────────────────────────────────

# v_max/v_min mirror sim_track.SimPlanner's defaults (the offline
# simulator/tuner's speed-profile source) rather than curvature_speed()'s own
# generic defaults (15.0/1.5) — keeping these matched to the offline pipeline
# is what makes offline-tuned weights transfer to the live desired_speed.
V_MAX            = 20.0  # m/s  — curvature_speed() top speed on straights
V_MIN            = 1.5   # m/s  — curvature_speed() minimum speed through corners
CONE_BRAKE_DIST  = 2.0   # m    — forward corridor depth for cone proximity brake
                         # NOTE: this is also the *ceiling* on the dynamic
                         # corridor computed in _check_cone_proximity() (car_speed
                         # * 0.25, clipped to [0.6, CONE_BRAKE_DIST]). At and above
                         # 8 m/s the corridor is capped at 2.0 m of forward look-ahead
                         # regardless of speed, i.e. well under 1 stopping second —
                         # worth reviewing against the car's real braking distance
                         # at top speed.
CONE_BRAKE_WIDTH = 0.18  # m    — lateral half-width of braking corridor (36 cm total)
TARGET_TIMEOUT   = 0.5   # s    — brake if no fresh path received within this window

# Minimum duration (s) the cone proximity brake must be active continuously
# before reset() is called on the MPC.
CONE_RESET_THRESHOLD = 0.3   # s  (~6 consecutive 50 ms ticks)

# ── Logging ─────────────────────────────────────────────────────────────────
LOG_DIR = None #FsPath('/tmp/mpc_logs')
LOG_FIELDS = [
    'ros_time', 'car_speed', 'desired_speed',
    'e_y', 'e_psi', 'e_v', 'kappa',
    'steering', 'throttle', 'brake',
    'delta_cmd', 'a_cmd', 'delta_act', 'a_act',
    'car_x', 'car_y', 'car_yaw',
]

class ControlNode(Node):
    """
    ROS 2 Node responsible for handling sensor data, maintaining vehicle state,
    and running the MPC control loop at a fixed frequency (20 Hz).
    """

    def __init__(self):
        super().__init__('controller')

        # Best effort QoS ensures we don't build up a latency-inducing backlog
        # of stale odometry frames if CPU loads momentarily spike.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Subscriptions ──────────────────────────────────────────────
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

        # ── Publisher ──────────────────────────────────────────────────
        self.pub_cmd = self.create_publisher(ControlCommand, '/fsds/control_command', 10)

        # ── Internal state ─────────────────────────────────────────────
        self._go_received    = False
        self._path_pts:  np.ndarray = np.empty((0, 2))
        self._path_stamp = None
        self._desired_speed: float = 0.0  # recomputed via curvature_speed() every Phase 3 tick
        self._car_pos        = np.zeros(2)
        self._car_yaw        = 0.0
        self._car_speed      = 0.0
        self._car_yaw_rate   = 0.0
        self._have_pose       = False
        # Cones arrive already in the car-LOCAL frame (see ConeDetection.msg /
        # fsae_sim_perception/sim_perception.py), so the proximity check below
        # needs no car_pos/car_yaw transform.
        self._cones_local: np.ndarray = np.empty((0, 2))
        self._cone_brake_duration: float = 0.0
        self._cone_reset_done: bool = False

        # ── MPC controller ─────────────────────────────────────────────
        self._mpc = MPCController(dt=0.05, N=25)

        # ── CSV telemetry logger ────────────────────────────────────────
        self._log_file = None
        self._log_writer = None
        if LOG_DIR is not None:
            try:
                LOG_DIR.mkdir(parents=True, exist_ok=True)
                log_path = LOG_DIR / f'mpc_run_{int(time.time())}.csv'
                self._log_file = open(log_path, 'w', newline='')
                self._log_writer = csv.DictWriter(self._log_file, fieldnames=LOG_FIELDS)
                self._log_writer.writeheader()
                self.get_logger().info(f'Logging MPC telemetry to {log_path}')
            except OSError as exc:
                self.get_logger().warn(f'Could not open log file ({exc!r}) — logging disabled.')
                self._log_file = None

        # ── 20 Hz control timer ────────────────────────────────────────
        self.create_timer(0.05, self._control_loop)
        self.get_logger().info('MPC Control node initialised and ready.')

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _go_cb(self, msg: GoSignal) -> None:
        """
        Latch the race "GO" signal.

        Once True, self._go_received is never reset, so the vehicle cannot
        be re-locked mid-run by a repeated/duplicate GoSignal message. Only
        the first GO message has any effect (subsequent ones are ignored).
        """
        if not self._go_received:
            self._go_received = True
            self.get_logger().info('GO signal received. Launching control loop.')

    def _path_cb(self, msg: PoseArray) -> None:
        """
        Convert the incoming planner PoseArray into a 2D numpy array [x, y]
        and record the wall-clock arrival time.

        self._path_stamp is used by _control_loop's staleness check
        (TARGET_TIMEOUT) — note it is stamped with node-clock "now" at
        message arrival, not the PoseArray message's own header stamp, so
        this measures ROS-callback latency/dropout rather than sensor age.
        An empty poses list degrades gracefully to an empty (0, 2) array,
        which subsequently trips the `len(self._path_pts) < 2` staleness
        condition in _control_loop.
        """
        self._path_pts = np.array(
            [[p.position.x, p.position.y] for p in msg.poses],
            dtype=np.float64,
        ) if msg.poses else np.empty((0, 2))
        self._path_stamp = self.get_clock().now()

    def _pose_cb(self, msg: Pose) -> None:
        """
        Car pose from SLAM. Upstream convention: x,y in position; yaw (rad)
        is stuffed into orientation.w rather than a proper quaternion — see
        fsae_sim_perception/sim_perception.py's _car_pose_msg().
        """
        self._car_pos   = np.array([msg.position.x, msg.position.y])
        self._car_yaw   = float(msg.orientation.w)
        self._have_pose = True

    def _odom_cb(self, msg: Odometry) -> None:
        """
        Extract forward speed and yaw rate from Odometry. Position/yaw come
        from _pose_cb (the SLAM car_position topic) instead, matching the
        fsae_planning stack's pose/twist split.

        car_speed is computed as hypot(vx, vy) of the *linear* twist, i.e.
        vehicle-frame planar speed magnitude, not projected onto the car's
        forward axis — this only equals true forward speed if lateral twist
        (side-slip) is negligible, which is the standard MPC small-slip
        assumption but can under/overstate speed during a slide.
        """
        v = msg.twist.twist.linear
        self._car_speed    = math.hypot(v.x, v.y)
        self._car_yaw_rate = msg.twist.twist.angular.z

    def _cone_cb(self, msg: ConeDetection) -> None:
        """
        Cache the fused blue+yellow cones (already car-local, see
        ConeDetection.msg) for the Phase 4 cone-proximity brake. Used only by
        that safety override, not by the MPC path tracking itself (path
        already comes pre-planned from the planner).
        """
        pts = [[p.x, p.y] for p in msg.blue] + [[p.x, p.y] for p in msg.yellow]
        self._cones_local = np.array(pts, dtype=np.float64) if pts else np.empty((0, 2))

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _check_cone_proximity(self) -> bool:
        """
        Checks the already car-local cone cache for any cone obstructing the
        dynamic braking corridor directly ahead of the vehicle.

        Returns True if a cone collision is imminent.
        """
        if len(self._cones_local) == 0:
            return False

        x_car = self._cones_local[:, 0]   # forward (+)
        y_car = self._cones_local[:, 1]   # left    (+)

        dynamic_brake_dist = float(np.clip(
            self._car_speed * 0.25, 0.6, CONE_BRAKE_DIST
        ))

        too_close = bool(np.any(
            (x_car > 0.2) &
            (x_car < dynamic_brake_dist) &
            (np.abs(y_car) < CONE_BRAKE_WIDTH)
        ))

        return too_close

    def _publish(self, cmd: ControlCommand) -> None:
        """
        Stamp and publish a ControlCommand.

        Centralised so every exit path of _control_loop (GO-wait, stale-path
        estop, and normal operation) publishes with a valid, current stamp —
        previously only the normal-operation path at the end of the loop
        set cmd.header.stamp, leaving early-return messages unstamped.
        """
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.pub_cmd.publish(cmd)

    # ------------------------------------------------------------------
    # Core MPC control loop (50 ms / 20 Hz)
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        """
        Main execution loop running at 20 Hz.
        Handles startup wait, trajectory loss fail-safes, MPC optimization,
        and the safety cone-brake override.
        """
        cmd = ControlCommand()

        # ── Phase 1: Hold at start line until GO signal ────────────────
        if not self._go_received:
            cmd.throttle, cmd.steering, cmd.brake = 0.0, 0.0, 1.0
            self._publish(cmd)
            self.get_logger().info('Waiting for GO signal...', throttle_duration_sec=2.0)
            return

        # ── Phase 2: Emergency brake on stale / missing path or pose ───
        path_stale = (
            self._path_stamp is None
            or (self.get_clock().now() - self._path_stamp).nanoseconds * 1e-9 > TARGET_TIMEOUT
            or len(self._path_pts) < 2
        )
        if not self._have_pose or path_stale:
            cmd.throttle, cmd.steering, cmd.brake = 0.0, 0.0, 1.0
            self._mpc.reset()
            self._publish(cmd)
            self.get_logger().warn('Trajectory path lost or stale — emergency braking.', throttle_duration_sec=1.0)
            return

        # ── Phase 3: MPC optimal control ──────────────────────────────
        # Desired speed is computed fresh each tick from the current path
        # (no external speed topic) — see module docstring.
        self._desired_speed = curvature_speed(self._path_pts, v_max=V_MAX, v_min=V_MIN)

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
        # This must run BEFORE telemetry logging (Phase 4a below) so the
        # logged throttle/brake reflect what is actually published, not
        # the pre-override MPC output.
        if self._check_cone_proximity():
            cmd.throttle = 0.0
            cmd.brake    = 1.0
            self._cone_brake_duration += 0.05

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

        # ── Phase 4a: Telemetry log (post-override, reflects final cmd) ─
        if self._log_writer is not None:
            tel = self._mpc.last_telemetry
            try:
                self._log_writer.writerow({
                    'ros_time': self.get_clock().now().nanoseconds * 1e-9,
                    'car_speed': self._car_speed,
                    'desired_speed': self._desired_speed,
                    'e_y': tel.get('e_y', ''),
                    'e_psi': tel.get('e_psi', ''),
                    'e_v': tel.get('e_v', ''),
                    'kappa': tel.get('kappa', ''),
                    'steering': cmd.steering,
                    'throttle': cmd.throttle,
                    'brake': cmd.brake,
                    'delta_cmd': tel.get('delta_cmd', ''),
                    'a_cmd': tel.get('a_cmd', ''),
                    'delta_act': tel.get('delta_act', ''),
                    'a_act': tel.get('a_act', ''),
                    'car_x': self._car_pos[0],
                    'car_y': self._car_pos[1],
                    'car_yaw': self._car_yaw,
                })
                self._log_file.flush()
            except Exception as exc:
                self.get_logger().warn(f'Telemetry log write failed: {exc!r}', throttle_duration_sec=5.0)

        # ── Phase 5: Publish ───────────────────────────────────────────
        self._publish(cmd)

        self.get_logger().debug(
            f'MPC thr={cmd.throttle:.2f} brk={cmd.brake:.2f} steer={cmd.steering:.3f} | '
            f'v={self._car_speed:.1f}/{self._desired_speed:.1f} m/s'
        )

    def destroy_node(self) -> None:
        """Cleanup hook to cleanly close the telemetry logging file."""
        if self._log_file is not None:
            try:
                self._log_file.flush()
                self._log_file.close()
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    """
    ROS 2 entry point: initialise rclpy, spin ControlNode until Ctrl-C, then
    cleanly destroy the node (flushing/closing the telemetry CSV via
    ControlNode.destroy_node()) and shut down rclpy.
    """
    rclpy.init(args=args)
    node = ControlNode()
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