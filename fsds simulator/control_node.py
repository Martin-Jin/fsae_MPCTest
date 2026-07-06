# Title: control_node.py

# control_node.py — ROS 2 MPC Control Loop for FSDS

import csv
import math
import time
from pathlib import Path as FsPath

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
CONE_BRAKE_DIST  = 2.0   # m    — forward corridor depth for cone proximity brake
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
        self._path_stamp = None           
        self._desired_speed: float = V_FALLBACK
        self._car_pos        = np.zeros(2)
        self._car_yaw        = 0.0
        self._car_speed      = 0.0
        self._car_yaw_rate   = 0.0
        self._blue_cones:   np.ndarray = np.empty((0, 2))
        self._yellow_cones: np.ndarray = np.empty((0, 2))
        self._cone_brake_duration: float = 0.0   

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
        """Unlocks the vehicle to begin following the path."""
        if not self._go_received:
            self._go_received = True
            self.get_logger().info('GO signal received. Launching control loop.')

    def _path_cb(self, msg: Path) -> None:
        """Converts the incoming Path poses into a 2D numpy array [x, y]."""
        self._path_pts = np.array(
            [[ps.pose.position.x, ps.pose.position.y] for ps in msg.poses],
            dtype=np.float64,
        ) if msg.poses else np.empty((0, 2))
        self._path_stamp = self.get_clock().now()

    def _speed_cb(self, msg: Float32) -> None:
        """Updates the desired target speed published by the planner."""
        self._desired_speed = float(msg.data)

    def _odom_cb(self, msg: Odometry) -> None:
        """Extracts position, speed, and yaw rate from Odometry."""
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
        """Separates the incoming cone map into blue and yellow arrays."""
        self._blue_cones, self._yellow_cones = separate_cones_by_color(msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _check_cone_proximity(self) -> bool:
        """
        Transforms visible cones into the car-relative frame to check if any
        obstruct the dynamic braking corridor ahead of the vehicle.
        
        Returns True if a cone collision is imminent.
        """
        cone_parts = [c for c in (self._blue_cones, self._yellow_cones) if len(c) > 0]
        if not cone_parts:
            return False

        all_cones = np.vstack(cone_parts)
        cos_y = math.cos(self._car_yaw)
        sin_y = math.sin(self._car_yaw)
        rel   = all_cones - self._car_pos

        x_car =  rel[:, 0] * cos_y + rel[:, 1] * sin_y   # forward (+)
        y_car = -rel[:, 0] * sin_y + rel[:, 1] * cos_y   # left    (+)

        dynamic_brake_dist = float(np.clip(
            self._car_speed * 0.25, 0.6, CONE_BRAKE_DIST
        ))

        too_close = bool(np.any(
            (x_car > 0.2) &
            (x_car < dynamic_brake_dist) &
            (np.abs(y_car) < CONE_BRAKE_WIDTH)
        ))
        
        return too_close

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
            self.pub_cmd.publish(cmd)
            self.get_logger().info('Waiting for GO signal...', throttle_duration_sec=2.0)
            return

        # ── Phase 2: Emergency brake on stale / missing path ──────────
        path_stale = (
            self._path_stamp is None
            or (self.get_clock().now() - self._path_stamp).nanoseconds * 1e-9 > TARGET_TIMEOUT
            or len(self._path_pts) < 2
        )
        if path_stale:
            cmd.throttle, cmd.steering, cmd.brake = 0.0, 0.0, 1.0
            self._mpc.reset()   
            self.pub_cmd.publish(cmd)
            self.get_logger().warn('Trajectory path lost or stale — emergency braking.', throttle_duration_sec=1.0)
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

        # ── Telemetry log (pre-override) ──────────────────────────────
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
                    'steering': steer_out,
                    'throttle': throttle_out,
                    'brake': brake_out,
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

        # ── Phase 4: Cone proximity brake override ────────────────────
        if self._check_cone_proximity():
            cmd.throttle = 0.0
            cmd.brake    = 1.0
            self._cone_brake_duration += 0.05

            if self._cone_brake_duration >= CONE_RESET_THRESHOLD:
                self._mpc.reset()

            self.get_logger().warn(
                f'Cone proximity brake active ({self._cone_brake_duration:.2f} s).',
                throttle_duration_sec=0.5,
            )
        else:
            self._cone_brake_duration = 0.0

        # ── Phase 5: Timestamp and publish ────────────────────────────
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.pub_cmd.publish(cmd)
        
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