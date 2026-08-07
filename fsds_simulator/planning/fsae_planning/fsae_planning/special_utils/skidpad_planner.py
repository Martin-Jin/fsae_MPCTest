"""
Skidpad (figure-8) characterisation planner node.

A special (sim-only) planner: instead of mapping/racing an arbitrary track it
reconstructs the figure-8 once from the full cone map and laps it at a steadily
rising speed until the car slides off the lane, recording the exact (time, speed)
of departure.  This is the data used to compare the simulator against the real
vehicle.  There is no upstream (car-stack) equivalent.

Because the characterisation needs a *commanded speed ramp* — which the standard
planning→control interface (a positions-only PoseArray) cannot carry — this node
drives the car itself: it publishes /fsae/control/cmd_vel directly with the ramped
speed and a pure-pursuit steering angle, so the Stanley controller is NOT launched
in skidpad mode.  It still publishes /fsae/planning/selected_trajectory for viz.

Interface:
    in   /fsae/slam/left_track    fsae_interfaces/Track   blue cones  (full map; sim_perception full_track:=true)
    in   /fsae/slam/right_track   fsae_interfaces/Track   yellow cones (full map)
    in   /fsae/slam/car_position  geometry_msgs/PoseStamped x,y in position; yaw in orientation.w
    in   /fsds/testing_only/odom  nav_msgs/Odometry       ground-truth speed (characterisation only)
    out  /fsae/planning/selected_trajectory  geometry_msgs/PoseArray
    out  /fsae/control/cmd_vel    ackermann_msgs/AckermannDriveStamped  ramped speed + pursuit steering
"""
import csv
import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from fsae_interfaces.msg import Track
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry

from fsae_planning.boundary import build_wall_segments
from fsae_planning.centerline_planner import cones_to_array
from fsae_planning.path_utils import get_lookahead_waypoint, roll_loop_to_car
from fsae_planning.special_utils.skidpad import build_figure8, path_deviation
from fsae_planning.special_utils.speed_input import SpeedInput

LOOKAHEAD_DIST = 4.0   # metres ahead for the pursuit target
WHEELBASE      = 1.5   # m — for the pure-pursuit steering-angle law
V_START_MOVE   = 0.3   # m/s — car considered "released"; starts the ramp clock

SKID_AHEAD           = 18.0  # m — forward planning window along the figure-8
SKID_OFFTRACK_MARGIN = 0.5   # m — slop beyond the lane half-width before "off track"
SKID_MIN_CENTRE_SEP  = 8.0   # m — reject fits whose two circles are too close
SKID_MIN_CONES       = 30    # need at least this many cones before attempting a fit
SKID_WALL_MAX_DIST   = 4.5   # m — just above the max outer-ring cone spacing so both
                             #     rings stay fully connected as walls
SKID_ENTER_RADIUS    = 1.5   # m — within this of the crossing the tangent join is done
SKID_LOG_PATH = os.path.expanduser('~/skidpad_speed_log.csv')


class SkidpadPlanner(Node):
    def __init__(self, node_name: str = 'skidpad_planner'):
        super().__init__(node_name)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('v_start', 3.0),      # m/s — speed at the start of the ramp
                ('ramp_accel', 0.25),  # m/s per second — how fast the target rises
                ('v_cap', 25.0),       # m/s — hard ceiling on the ramp
            ],
        )
        self._v_start    = self.get_parameter('v_start').get_parameter_value().double_value
        self._ramp_accel = self.get_parameter('ramp_accel').get_parameter_value().double_value
        self._v_cap      = self.get_parameter('v_cap').get_parameter_value().double_value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Track, '/fsae/slam/left_track',  self._left_cb,  10)
        self.create_subscription(Track, '/fsae/slam/right_track', self._right_cb, 10)
        self.create_subscription(PoseStamped, '/fsae/slam/car_position', self._pose_cb, 10)
        # Ground-truth speed for characterisation logging (sim-only input).
        self.create_subscription(Odometry, '/fsds/testing_only/odom', self._odom_cb, sensor_qos)

        self.pub_traj = self.create_publisher(PoseArray, '/fsae/planning/selected_trajectory', 10)
        self.pub_cmd  = self.create_publisher(AckermannDriveStamped, '/fsae/control/cmd_vel', 10)

        self._blue_cones:   np.ndarray = np.empty((0, 2))
        self._yellow_cones: np.ndarray = np.empty((0, 2))
        self._car_pos   = np.zeros(2)
        self._car_yaw   = 0.0
        self._car_speed = 0.0
        self._centreline: np.ndarray | None = None

        self._skid_track      = None    # cached Figure8Track once reconstructed
        self._skid_blue_segs  = []      # cone-wall mesh (blue) for entry gating + viz
        self._skid_yellow_segs = []
        self._skid_walls      = []
        self._skid_entered    = False   # car reached the crossing (tangent-join done)
        self._skid_t0         = None    # rclpy.time.Time — ramp start
        self._skid_v_target   = 0.0
        self._skid_dev        = 0.0
        self._skid_on_lane    = False
        self._skid_off_track  = False
        self._skid_csv_ready  = False

        # Manual speed entry (GUI text box / terminal): overrides the ramp.
        self._speed_input = SpeedInput(
            minimum=0.0, maximum=self._v_cap,
            logger=self.get_logger().info,
        ).start()

        self.create_timer(1.0, self._skid_log_loop)   # speed log @ 1 Hz

        self.get_logger().info('skidpad_planner ready (characterisation mode) — waiting for cones + car_position.')

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _left_cb(self, msg: Track) -> None:
        self._blue_cones = cones_to_array(msg.cones)
        self._try_build_figure8()

    def _right_cb(self, msg: Track) -> None:
        self._yellow_cones = cones_to_array(msg.cones)
        self._try_build_figure8()

    def _odom_cb(self, msg: Odometry) -> None:
        v = msg.twist.twist.linear
        self._car_speed = float(math.hypot(v.x, v.y))

    def _pose_cb(self, msg: PoseStamped) -> None:
        self._car_pos = np.array([msg.pose.position.x, msg.pose.position.y])
        self._car_yaw = float(msg.pose.orientation.w)
        self._planning_loop()

    # ------------------------------------------------------------------
    # Figure-8 reconstruction (once, from the full cone map)
    # ------------------------------------------------------------------

    def _try_build_figure8(self) -> None:
        if self._skid_track is not None:
            return
        blue, yellow = self._blue_cones, self._yellow_cones
        if len(blue) + len(yellow) < SKID_MIN_CONES:
            return

        track = build_figure8(blue, yellow)
        if track is None:
            self.get_logger().warn(
                f'Skidpad: could not fit a figure-8 '
                f'({len(blue)} blue + {len(yellow)} yellow cones).',
                throttle_duration_sec=3.0,
            )
            return
        sep = float(np.linalg.norm(track.centres[0] - track.centres[1]))
        if sep < SKID_MIN_CENTRE_SEP:
            self.get_logger().warn(
                f'Skidpad: rejected fit — circles only {sep:.1f} m apart.',
                throttle_duration_sec=3.0,
            )
            return

        self._skid_track = track
        self._skid_blue_segs   = build_wall_segments(blue, SKID_WALL_MAX_DIST)
        self._skid_yellow_segs = build_wall_segments(yellow, SKID_WALL_MAX_DIST)
        self._skid_walls       = self._skid_blue_segs + self._skid_yellow_segs
        self.get_logger().info(
            f'Skidpad figure-8 precomputed from {len(blue) + len(yellow)} cones: '
            f'circles {sep:.1f} m apart, lane radius {track.lane_radius:.2f} m, '
            f'half-width {track.half_width:.2f} m.'
        )

    # ------------------------------------------------------------------
    # Planning + command loop (triggered by car_position)
    # ------------------------------------------------------------------

    def _planning_loop(self) -> None:
        if self._skid_track is None:
            self.get_logger().info(
                'Skidpad: waiting for enough cones to precompute the figure-8...',
                throttle_duration_sec=2.0,
            )
            return

        # Start the ramp clock the first time the car is actually moving.
        if self._skid_t0 is None and self._car_speed > V_START_MOVE:
            self._skid_t0 = self.get_clock().now()
            self._init_skid_csv()
            self.get_logger().info(
                f'Skidpad run starting — ramping from {self._v_start:.1f} m/s '
                f'at {self._ramp_accel:.2f} m/s².'
            )

        loop = self._skid_track.loop

        # Forward planning window along the figure-8.  The cone-wall mesh gates the
        # entry (the car must reach the figure-8 through the opening); tangent_entry
        # merges the car smoothly onto the circle until it reaches the crossing.
        crossing = self._skid_track.centres.mean(axis=0)
        if not self._skid_entered and \
                float(np.linalg.norm(self._car_pos - crossing)) <= SKID_ENTER_RADIUS:
            self._skid_entered = True
        self._centreline = roll_loop_to_car(
            loop, self._car_pos, self._car_yaw, ahead=SKID_AHEAD,
            wall_segs=self._skid_walls, tangent_entry=not self._skid_entered,
        )
        self._publish_trajectory()

        # Track deviation → arm and watch for the spin-off.
        self._skid_dev = path_deviation(loop, self._car_pos)
        off_limit = self._skid_track.half_width + SKID_OFFTRACK_MARGIN
        if not self._skid_on_lane and self._skid_dev <= self._skid_track.half_width:
            self._skid_on_lane = True
        if (self._skid_on_lane and not self._skid_off_track
                and self._skid_dev > off_limit):
            self._record_spinoff()

        # Target speed: a manually entered speed overrides the ramp; a recorded
        # spin-off forces zero (terminal safety state).
        manual = self._speed_input.latest() if self._speed_input else None
        if self._skid_off_track:
            self._skid_v_target = 0.0
        elif manual is not None:
            self._skid_v_target = float(manual)
        else:
            self._skid_v_target = min(
                self._v_cap, self._v_start + self._ramp_accel * self._skid_elapsed()
            )

        self._publish_cmd()

    def _publish_cmd(self) -> None:
        """Publish cmd_vel: ramped speed + pure-pursuit steering along the loop."""
        steering = 0.0
        target = get_lookahead_waypoint(
            self._centreline, self._car_pos, self._car_yaw, LOOKAHEAD_DIST
        )
        if target is not None:
            to_t = np.asarray(target) - self._car_pos
            ld = float(np.linalg.norm(to_t))
            if ld > 1e-3:
                h = np.array([math.cos(self._car_yaw), math.sin(self._car_yaw)])
                cross = h[0] * to_t[1] - h[1] * to_t[0]   # +ve = target to the left
                dot   = float(h @ to_t)
                alpha = math.atan2(cross, dot)
                steering = math.atan2(2.0 * WHEELBASE * math.sin(alpha), ld)

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(self._skid_v_target)
        msg.drive.steering_angle = float(steering)
        self.pub_cmd.publish(msg)

    def _publish_trajectory(self) -> None:
        if self._centreline is None or len(self._centreline) == 0:
            return
        msg = PoseArray()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for wp in self._centreline:
            pose = Pose()
            pose.position.x = float(wp[0])
            pose.position.y = float(wp[1])
            msg.poses.append(pose)
        self.pub_traj.publish(msg)

    # ------------------------------------------------------------------
    # Ramp timing + logging
    # ------------------------------------------------------------------

    def _skid_elapsed(self) -> float:
        if self._skid_t0 is None:
            return 0.0
        return (self.get_clock().now() - self._skid_t0).nanoseconds * 1e-9

    def _record_spinoff(self) -> None:
        """Latch the exact time and speed at which the car left the lane."""
        self._skid_off_track = True
        t = self._skid_elapsed()
        v = self._car_speed
        self.get_logger().error(
            f'*** SPIN-OFF *** car left the lane after {t:.2f} s at '
            f'{v:.2f} m/s (deviation {self._skid_dev:.2f} m '
            f'> {self._skid_track.half_width + SKID_OFFTRACK_MARGIN:.2f} m). '
            f'Logged to {SKID_LOG_PATH}'
        )
        self._append_skid_csv(t, self._skid_v_target, v, self._skid_dev, event='SPIN_OFF')

    def _skid_log_loop(self) -> None:
        """Log the speed once per second while characterising."""
        if self._skid_track is None or self._skid_t0 is None or self._skid_off_track:
            return
        t = self._skid_elapsed()
        self.get_logger().info(
            f'Skidpad t={t:6.1f} s  v_target={self._skid_v_target:5.2f} m/s  '
            f'v_actual={self._car_speed:5.2f} m/s  dev={self._skid_dev:.2f} m'
        )
        self._append_skid_csv(t, self._skid_v_target, self._car_speed,
                              self._skid_dev, event='run')

    def _init_skid_csv(self) -> None:
        try:
            with open(SKID_LOG_PATH, 'w', newline='') as f:
                csv.writer(f).writerow(
                    ['time_s', 'v_target_mps', 'v_actual_mps', 'deviation_m', 'event']
                )
            self._skid_csv_ready = True
        except OSError as exc:
            self.get_logger().warn(f'Could not open skidpad log {SKID_LOG_PATH}: {exc!r}')
            self._skid_csv_ready = False

    def _append_skid_csv(self, t: float, v_target: float, v_actual: float,
                         dev: float, event: str) -> None:
        if not self._skid_csv_ready:
            return
        try:
            with open(SKID_LOG_PATH, 'a', newline='') as f:
                csv.writer(f).writerow(
                    [f'{t:.3f}', f'{v_target:.3f}', f'{v_actual:.3f}', f'{dev:.3f}', event]
                )
        except OSError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SkidpadPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._speed_input is not None:
            node._speed_input.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
