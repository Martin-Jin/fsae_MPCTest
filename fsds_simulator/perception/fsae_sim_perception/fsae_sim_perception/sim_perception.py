"""
Simulator perception stand-in (FSDS → fsae_autonomous interface bridge).

On the real car the perception front-end is a ZED camera node (cone detection +
visual-odometry pose) followed by a SLAM cone_mapper that produces the boundary
tracks.  The FSDS simulator instead broadcasts a latched oracle cone map plus
ground-truth odometry.  This node stands in for the whole camera+SLAM front-end:
it crops the oracle map to a forward FOV window (to mimic a real sensor) and
republishes it on the exact topics/messages the car stack expects.

    in   /fsds/testing_only/track  fs_msgs/Track       latched oracle cone map
    in   /fsds/testing_only/odom   nav_msgs/Odometry   ground-truth pose

    out  /fsae/slam/car_position   geometry_msgs/PoseStamped     x,y in position; yaw in orientation.w;
                                                                  header.stamp = odom's own measurement time
    out  /fsae/slam/car_odom       nav_msgs/Odometry             SAME snapshot as car_position (position/
                                                                  yaw in pose, speed/yaw-rate in twist) --
                                                                  see "Speed/yaw-rate synchronisation" below
    out  /fsae/slam/left_track     fsae_interfaces/Track         blue (left) boundary, global frame
    out  /fsae/slam/right_track    fsae_interfaces/Track         yellow (right) boundary, global frame
    out  /fsae/perception/cone_detection  fsae_interfaces/ConeDetection  local-frame detections + car_pose

Set the `full_track` parameter true to publish the entire map every frame (used by
the skidpad planner, which reconstructs the whole figure-8 up front); the default
(false) publishes only the forward window, matching normal driving.

Two publish rates (`pose_rate` 20 Hz, `cone_rate` 10 Hz): pose and cones are
published on separate timers. `pose_rate` must be >= the controller's rate
(`CONTROL_HZ = 20` in mpc_controller_standalone.py) — if the MPC re-solves
against an unchanged pose on some ticks, the pose jumps multiple steps' worth
at once on the next tick and the controller over-corrects (catch-up jumps
rather than smooth motion). The FSDS bridge itself publishes odom at 250 Hz,
so this node's own publish rate is the only possible bottleneck. Cones stay
at 10 Hz — cropping the oracle map and building three messages is the
expensive part of this node, and the planner gains nothing from running it
at the control rate.

Known limitation: this node is not a SLAM stand-in for accuracy, only for
range. The pose it publishes is FSDS ground truth, copied verbatim: no noise,
no drift, no estimation lag. The offline tuner models this gap explicitly via
`SLAM_NOISE_ENABLED` in fsae_MPCTest/settings.py (default off, since FSDS has
no such error).

Speed/yaw-rate synchronisation: mpc_controller.py / mpc_controller_standalone.py
get car_pos/car_yaw AND car_speed/car_yaw_rate from /fsae/slam/car_odom (this
node's 20 Hz relay), not from a separate direct subscription to the raw 250 Hz
/fsds/testing_only/odom topic. Two independent subscriptions racing the same
250 Hz publisher have no guarantee the "latest" sample each holds at a given
20 Hz control tick came from the same underlying odom instant, so the MPC's
x0 could be built from a position/heading snapshot and a speed/yaw-rate
snapshot up to ~1/pose_rate apart, producing small accel/brake oscillation
concentrated in curves. /fsae/slam/car_odom is published from this node's
_odom_cb-updated state at the same 20 Hz timer tick as car_position, so pose
and twist the controller reads for one solve always originate from one atomic
snapshot. car_position (PoseStamped) keeps publishing unchanged for
centerline_planner.py / stanley_controller.py / cone_recorder.py /
skidpad_planner.py, none of which need speed.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from fs_msgs.msg import Cone, Track as FsTrack
from fsae_interfaces.msg import ConeDetection, Track
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry


class SimPerception(Node):
    def __init__(self):
        super().__init__('sim_perception')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('look_ahead', 25.0),   # m ahead to include in the forward box
                ('look_wide', 10.0),    # m lateral half-width of the forward box
                ('min_ahead', 0.5),     # m: box ignores cones behind / at the car
                # 25.0 (was 18.0) — kept >= centerline_planner's look_radius/
                # plan_horizon so cones for a tight corner reach the planner
                # far enough out to detect and brake for it in time.
                ('look_radius', 25.0),  # m: omni-directional visibility radius
                ('full_track', False),  # publish the whole map instead of the window
                # Publish rates (Hz). Split deliberately — see the module
                # docstring. pose_rate MUST be >= the controller's rate
                # (CONTROL_HZ = 20) or the MPC re-solves against stale poses.
                ('pose_rate', 20.0),
                ('cone_rate', 10.0),
            ],
        )
        self._look_ahead  = self.get_parameter('look_ahead').get_parameter_value().double_value
        self._look_wide   = self.get_parameter('look_wide').get_parameter_value().double_value
        self._min_ahead   = self.get_parameter('min_ahead').get_parameter_value().double_value
        self._look_radius = self.get_parameter('look_radius').get_parameter_value().double_value
        self._full_track  = self.get_parameter('full_track').get_parameter_value().bool_value
        self._pose_rate   = self.get_parameter('pose_rate').get_parameter_value().double_value
        self._cone_rate   = self.get_parameter('cone_rate').get_parameter_value().double_value
        # Guard against a zero/negative rate turning into a divide-by-zero or
        # a timer that never fires.
        self._pose_rate = self._pose_rate if self._pose_rate > 0.0 else 20.0
        self._cone_rate = self._cone_rate if self._cone_rate > 0.0 else 10.0

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(FsTrack, '/fsds/testing_only/track', self._track_cb, latched_qos)
        self.create_subscription(Odometry, '/fsds/testing_only/odom', self._odom_cb, sensor_qos)

        self.pub_pose  = self.create_publisher(PoseStamped,   '/fsae/slam/car_position',        10)
        self.pub_odom  = self.create_publisher(Odometry,      '/fsae/slam/car_odom',            10)
        self.pub_left  = self.create_publisher(Track,         '/fsae/slam/left_track',          10)
        self.pub_right = self.create_publisher(Track,         '/fsae/slam/right_track',         10)
        self.pub_det   = self.create_publisher(ConeDetection, '/fsae/perception/cone_detection', 10)

        # Oracle map, split by colour into global (N, 2) arrays once received.
        self._blue_all:   np.ndarray = np.empty((0, 2))
        self._yellow_all: np.ndarray = np.empty((0, 2))
        self._have_map = False
        self._have_odom = False
        self._car_x = 0.0
        self._car_y = 0.0
        self._car_yaw = 0.0
        # Speed/yaw-rate, set alongside car_x/y/yaw in the SAME _odom_cb call
        # so car_odom's twist and pose always originate from one atomic
        # snapshot -- see the module docstring's "Speed/yaw-rate
        # synchronisation" note for why this matters.
        self._car_vx = 0.0
        self._car_vy = 0.0
        self._car_yaw_rate = 0.0
        self._odom_stamp = None

        # Two timers, deliberately at different rates — see the module
        # docstring's "Two publish rates" note. Pose must keep up with the
        # controller; the cone map does not need to.
        self.create_timer(1.0 / self._pose_rate, self._publish_pose)
        self.create_timer(1.0 / self._cone_rate, self._publish_cones)

        mode = ('full map' if self._full_track else
                f'radius {self._look_radius} m ∪ box {self._look_ahead}×{2 * self._look_wide} m')
        self.get_logger().info(
            f'sim_perception ready ({mode}); '
            f'pose @ {self._pose_rate:g} Hz, cones @ {self._cone_rate:g} Hz.'
        )
        if self._pose_rate < 20.0:
            self.get_logger().warn(
                f'pose_rate is {self._pose_rate:g} Hz but the MPC controller runs at 20 Hz — '
                'some control steps will re-solve against an unchanged pose, which can '
                'induce steering oscillation. See this node\'s docstring.'
            )

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _track_cb(self, msg: FsTrack) -> None:
        blue, yellow = [], []
        for cone in msg.track:
            pt = [cone.location.x, cone.location.y]
            if cone.color == Cone.BLUE:
                blue.append(pt)
            elif cone.color == Cone.YELLOW:
                yellow.append(pt)
        self._blue_all   = np.array(blue,   dtype=np.float64) if blue   else np.empty((0, 2))
        self._yellow_all = np.array(yellow, dtype=np.float64) if yellow else np.empty((0, 2))
        self._have_map = True
        self.get_logger().info(
            f'Oracle map received: {len(blue)} blue + {len(yellow)} yellow.', once=True
        )

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._car_x = p.x
        self._car_y = p.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._car_yaw = math.atan2(siny_cosp, cosy_cosp)
        # Set in the SAME callback invocation as position/yaw above, so
        # car_odom's pose and twist always come from one atomic odom sample
        # — see the module docstring's "Speed/yaw-rate synchronisation" note.
        v = msg.twist.twist.linear
        self._car_vx = v.x
        self._car_vy = v.y
        self._car_yaw_rate = msg.twist.twist.angular.z
        self._odom_stamp = msg.header.stamp
        self._have_odom = True

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _visible(self, cones: np.ndarray) -> np.ndarray:
        """
        Crop cones to the sensor FOV (car frame), or pass all through.

        FOV = within look_radius of the car (omni — mimics a 360° lidar and keeps
        the cones around a bend that a forward-only box loses when the track
        curves away) OR inside the forward box (longer-range preview straight
        ahead).  Cropping still happens so the planner's cone map fills in track
        order over a lap rather than being handed the whole oracle map at once.
        """
        if self._full_track or len(cones) == 0:
            return cones
        cos_y, sin_y = math.cos(self._car_yaw), math.sin(self._car_yaw)
        rel = cones - np.array([self._car_x, self._car_y])
        dist = np.hypot(rel[:, 0], rel[:, 1])
        x_car =  rel[:, 0] * cos_y + rel[:, 1] * sin_y
        y_car = -rel[:, 0] * sin_y + rel[:, 1] * cos_y
        box = (x_car > self._min_ahead) & (x_car < self._look_ahead) & \
              (np.abs(y_car) < self._look_wide)
        return cones[(dist < self._look_radius) | box]

    def _car_pose_msg(self) -> PoseStamped:
        msg = PoseStamped()
        # Odom's own timestamp, not now() — this is the time the pose was
        # actually measured, which is what delay/age estimation downstream
        # (MPCController's pose_age_s) needs to be measured against.
        msg.header.stamp = self._odom_stamp
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(self._car_x)
        msg.pose.position.y = float(self._car_y)
        # Upstream convention: yaw (rad) is repurposed into orientation.w.
        msg.pose.orientation.w = float(self._car_yaw)
        return msg

    def _car_odom_msg(self) -> Odometry:
        """
        Same snapshot as _car_pose_msg() (both read _car_x/_car_y/_car_yaw/
        _car_vx/_car_vy/_car_yaw_rate, all set together in _odom_cb) --
        see the module docstring's "Speed/yaw-rate synchronisation" note.
        Unlike car_position's repurposed orientation.w convention, this uses
        a real quaternion (nav_msgs/Odometry consumers expect one).
        """
        msg = Odometry()
        msg.header.stamp = self._odom_stamp
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(self._car_x)
        msg.pose.pose.position.y = float(self._car_y)
        half_yaw = 0.5 * self._car_yaw
        msg.pose.pose.orientation.z = float(math.sin(half_yaw))
        msg.pose.pose.orientation.w = float(math.cos(half_yaw))
        msg.twist.twist.linear.x = float(self._car_vx)
        msg.twist.twist.linear.y = float(self._car_vy)
        msg.twist.twist.angular.z = float(self._car_yaw_rate)
        return msg

    @staticmethod
    def _global_track(cones: np.ndarray) -> Track:
        msg = Track()
        for c in cones:
            msg.cones.append(Point(x=float(c[0]), y=float(c[1]), z=0.0))
        return msg

    def _local_points(self, cones: np.ndarray) -> list:
        """Transform global cones into the car frame for cone_detection."""
        pts = []
        cos_y, sin_y = math.cos(self._car_yaw), math.sin(self._car_yaw)
        for c in cones:
            dx = c[0] - self._car_x
            dy = c[1] - self._car_y
            x_local =  dx * cos_y + dy * sin_y
            y_local = -dx * sin_y + dy * cos_y
            pts.append(Point(x=float(x_local), y=float(y_local), z=0.0))
        return pts

    def _publish_pose(self) -> None:
        """
        Pose-only tick, run at pose_rate (default 20 Hz to match the
        controller).  Deliberately does NOT touch the cone map — see the
        class docstring's "Two publish rates" note for why these are split.

        Publishes car_position and car_odom back-to-back from the SAME
        _car_x/_car_y/_car_yaw/_car_vx/_car_vy/_car_yaw_rate snapshot -- see
        the module docstring's "Speed/yaw-rate synchronisation" note. Do not
        call _odom_cb or otherwise refresh state between these two publishes.
        """
        if not self._have_odom:
            return
        self.pub_pose.publish(self._car_pose_msg())
        self.pub_odom.publish(self._car_odom_msg())

    def _publish_cones(self) -> None:
        """
        Cone/track tick, run at cone_rate (default 10 Hz).  Cropping the
        oracle map and building three messages is the expensive part of this
        node, and the planner gains nothing from it running at the control
        rate.
        """
        if not self._have_odom or not self._have_map:
            return

        # ConeDetection embeds the pose the detections were taken from, so
        # this tick builds its own (current) pose rather than reusing one
        # from the faster pose timer — the two must be consistent with the
        # cone transforms computed immediately below.
        pose = self._car_pose_msg()

        blue_vis   = self._visible(self._blue_all)
        yellow_vis = self._visible(self._yellow_all)
        self.pub_left.publish(self._global_track(blue_vis))
        self.pub_right.publish(self._global_track(yellow_vis))

        det = ConeDetection()
        det.header.stamp    = self.get_clock().now().to_msg()
        det.header.frame_id = 'base_link'
        det.car_pose = pose.pose   # ConeDetection.car_pose is geometry_msgs/Pose, not Stamped
        det.blue   = self._local_points(blue_vis)
        det.yellow = self._local_points(yellow_vis)
        self.pub_det.publish(det)


def main(args=None):
    rclpy.init(args=args)
    node = SimPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
