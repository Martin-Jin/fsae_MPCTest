"""
fsae_control/live_viz/node.py — LiveVizNode and the car glyph

`LiveVizNode` subscribes to the live ROS2 topics and caches the latest
message of each for the plot to read; `get_car_triangle` builds the car marker.
"""

import json
import time
from collections import deque

import numpy as np

from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)

from ackermann_msgs.msg import AckermannDriveStamped
from fs_msgs.msg import ControlCommand
from fsae_interfaces.msg import Track
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from fsae_control.live_viz.panels import TRAIL_MAXLEN


def get_car_triangle(x, y, heading, size=1.6):
    """
    (x, y) vertices of a triangle marking the car's position/heading, apex
    forward. Same construction as fsae_MPCTest/gui/simulation.py's
    get_car_triangle() (offline tool), duplicated here rather than imported
    since this module has no dependency on that offline-only package.
    """
    corners = np.array([
        [size,        0.0],
        [-size / 1.5,  size / 1.5],
        [-size / 1.5, -size / 1.5],
        [size,        0.0],
    ])
    rot = np.array([
        [np.cos(heading), -np.sin(heading)],
        [np.sin(heading),  np.cos(heading)],
    ])
    rotated = (rot @ corners.T).T
    return rotated[:, 0] + x, rotated[:, 1] + y


class LiveVizNode(Node):
    def __init__(self):
        super().__init__('live_viz')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Matches mpc_controller.py's static_path_qos exactly -- ROS2 requires
        # a TRANSIENT_LOCAL subscriber to receive a TRANSIENT_LOCAL
        # publisher's last message regardless of connection order, which is
        # the whole point here (this node starts before mpc_controller.py
        # even exists, see launch_all.sh). A plain (VOLATILE) subscription
        # would silently never see the one-shot publish.
        static_path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = 0.0
        self.car_speed = 0.0
        self.have_pose = False

        self.left_cones = np.empty((0, 2))
        self.right_cones = np.empty((0, 2))
        self.ref_path = np.empty((0, 2))
        self.static_ref_path = np.empty((0, 2))
        self.nmpc_pred_path = np.empty((0, 2))
        self.trail = deque(maxlen=TRAIL_MAXLEN)

        self.steering = 0.0
        self.throttle = 0.0
        self.brake = 0.0
        self.cmd_speed_target = None   # cmd_vel mode only
        self.control_topic = None      # which of the two actually fired, for the stats panel
        self.debug_weights = None      # parsed JSON dict from /fsae/control/debug_weights, or None
        self.debug_stanley = None      # parsed JSON dict from /fsae/control/debug_stanley, or None
        # Completed-lap summaries (see ControlLogger.finish_lap()), newest
        # last -- the lap panel (see _redraw's stats box) shows the whole
        # list, most recent highlighted. Bounded so a very long multi-lap
        # session doesn't grow this without limit; the panel only has room
        # to show the last handful anyway.
        self.lap_summaries: deque = deque(maxlen=20)
        # Which debug topic fired most recently -- the only positive signal
        # this node has for "which controller is actually active" (MPC and
        # Stanley share one ROS node name and, in cmd_vel mode, one output
        # topic). Compared by wall-clock arrival, not message content, so a
        # stale topic from a controller that's no longer running (e.g. left
        # over from an earlier launch this session) stops winning once the
        # other one starts publishing.
        self._debug_weights_at = 0.0
        self._debug_stanley_at = 0.0

        # Bumped by every callback below (see _counted), so redraw() can
        # detect "nothing new arrived" and stop draining without needing a
        # per-callback counter to remember to update.
        self._callback_count = 0

        self._subscribe(Track, '/fsae/slam/left_track', self._left_track_cb, 10)
        self._subscribe(Track, '/fsae/slam/right_track', self._right_track_cb, 10)
        self._subscribe(PoseStamped, '/fsae/slam/car_position', self._pose_cb, 10)
        self._subscribe(Odometry, '/fsae/slam/car_odom', self._odom_cb, sensor_qos)
        self._subscribe(
            PoseArray, '/fsae/planning/selected_trajectory', self._ref_path_cb, 10)
        self._subscribe(
            PoseArray, '/fsae/control/static_reference_path',
            self._static_ref_path_cb, static_path_qos)
        self._subscribe(
            PoseArray, '/fsae/control/nmpc_predicted_path', self._nmpc_pred_cb, 10)
        self._subscribe(
            ControlCommand, '/fsds/control_command', self._control_command_cb, 10)
        self._subscribe(
            AckermannDriveStamped, '/fsae/control/cmd_vel', self._cmd_vel_cb, 10)
        self._subscribe(
            String, '/fsae/control/debug_weights', self._debug_weights_cb, 10)
        self._subscribe(
            String, '/fsae/control/debug_stanley', self._debug_stanley_cb, 10)
        # RELIABLE + KEEP_LAST(10), matching the publisher's QoS (see
        # mpc_controller.py's pub_lap_summary comment) -- a lap completion
        # is a rare, one-shot event and must not be silently dropped the
        # way a BEST_EFFORT subscription could under momentary congestion.
        self._subscribe(
            String, '/fsae/control/lap_summary', self._lap_summary_cb,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )

    def _subscribe(self, msg_type, topic, callback, qos):
        """
        create_subscription wrapper that bumps _callback_count around every
        callback, so redraw() can tell "nothing new arrived" without each
        callback remembering to update a counter itself.
        """
        def counted(msg, _cb=callback):
            _cb(msg)
            self._callback_count += 1
        return self.create_subscription(msg_type, topic, counted, qos)

    @staticmethod
    def _pose_array_to_xy(msg: PoseArray) -> np.ndarray:
        if not msg.poses:
            return np.empty((0, 2))
        return np.array([[p.position.x, p.position.y] for p in msg.poses])

    @staticmethod
    def _points_to_xy(points) -> np.ndarray:
        if not points:
            return np.empty((0, 2))
        return np.array([[p.x, p.y] for p in points])

    def _left_track_cb(self, msg: Track) -> None:
        self.left_cones = self._points_to_xy(msg.cones)

    def _right_track_cb(self, msg: Track) -> None:
        self.right_cones = self._points_to_xy(msg.cones)

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.car_x = msg.pose.position.x
        self.car_y = msg.pose.position.y
        # NOT a real quaternion: sim_perception.py's _car_pose_msg()
        # repurposes orientation.w to carry raw yaw (radians) directly,
        # x=y=z=0 always (see that file's own "Upstream convention"
        # comment). Every real consumer of this topic (mpc_controller.py,
        # stanley_controller.py, centerline_planner.py, skidpad_planner.py)
        # already reads it this way -- a standard atan2 quaternion decode
        # here was wrong (always evaluated to 0 against x=y=z=0) and is why
        # the debug triangle was stuck pointing at yaw=0 regardless of the
        # car's actual heading.
        self.car_yaw = float(msg.pose.orientation.w)
        self.have_pose = True
        self.trail.append((self.car_x, self.car_y))

    def _odom_cb(self, msg: Odometry) -> None:
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.car_speed = float(np.hypot(vx, vy))

    def _ref_path_cb(self, msg: PoseArray) -> None:
        self.ref_path = self._pose_array_to_xy(msg)

    def _static_ref_path_cb(self, msg: PoseArray) -> None:
        self.static_ref_path = self._pose_array_to_xy(msg)

    def _nmpc_pred_cb(self, msg: PoseArray) -> None:
        self.nmpc_pred_path = self._pose_array_to_xy(msg)

    def _control_command_cb(self, msg: ControlCommand) -> None:
        self.steering, self.throttle, self.brake = msg.steering, msg.throttle, msg.brake
        self.control_topic = '/fsds/control_command'

    def _cmd_vel_cb(self, msg: AckermannDriveStamped) -> None:
        self.steering = msg.drive.steering_angle
        self.cmd_speed_target = msg.drive.speed
        self.control_topic = '/fsae/control/cmd_vel'

    def _debug_weights_cb(self, msg: String) -> None:
        try:
            self.debug_weights = json.loads(msg.data)
            self._debug_weights_at = time.monotonic()
        except (json.JSONDecodeError, TypeError):
            self.debug_weights = None

    def _debug_stanley_cb(self, msg: String) -> None:
        try:
            self.debug_stanley = json.loads(msg.data)
            self._debug_stanley_at = time.monotonic()
        except (json.JSONDecodeError, TypeError):
            self.debug_stanley = None

    def _lap_summary_cb(self, msg: String) -> None:
        try:
            self.lap_summaries.append(json.loads(msg.data))
        except (json.JSONDecodeError, TypeError):
            pass

    def active_controller(self) -> str:
        """'mpc' or 'stanley', whichever debug topic fired most recently,
        or 'unknown' if neither has fired yet this session. Wall-clock
        arrival, not message content -- see the two _at fields' own
        comment for why."""
        if self._debug_weights_at == 0.0 and self._debug_stanley_at == 0.0:
            return 'unknown'
        return 'mpc' if self._debug_weights_at >= self._debug_stanley_at else 'stanley'
