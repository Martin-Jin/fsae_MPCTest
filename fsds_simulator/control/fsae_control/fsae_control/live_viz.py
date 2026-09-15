"""
Live, close-up debug visualiser for the simulator: car (as a triangle), cone
map, planner/reference path, NMPC's predicted horizon (NMPC only), the car's
driven trail, and the controller's current output/stats, all redrawn from
live ROS2 topics on a timer.

Sim-only debug tool, not part of the car's autonomy stack: never launched by
`fsae_autonomous`, only by `ros2/launch_all.sh` alongside the simulator, the
same way the periodic-teleport diagnostics are (see that script's "TEMPORARY"
block). Standalone matplotlib window, no rqt/rviz dependency, run:

    ros2 run fsae_control live_viz

Topics subscribed (see this repo's fsae_planning per-file docstrings for the
authoritative topic table):
    /fsae/slam/left_track            fsae_interfaces/Track          blue boundary, global frame
    /fsae/slam/right_track           fsae_interfaces/Track          yellow boundary, global frame
    /fsae/slam/car_position           geometry_msgs/PoseStamped      car pose, global frame
    /fsae/slam/car_odom               nav_msgs/Odometry              car speed/yaw rate
    /fsae/planning/selected_trajectory  geometry_msgs/PoseArray      live planner's centreline
                                                                     (empty in precomputed-path
                                                                     mode -- the planner does not
                                                                     even run then, see
                                                                     sim.launch.py)
    /fsae/control/static_reference_path geometry_msgs/PoseArray      one-shot, TRANSIENT_LOCAL:
                                                                     the precomputed path
                                                                     (path_map_path), when set --
                                                                     see mpc_controller.py. Drawn
                                                                     INSTEAD OF the topic above
                                                                     when populated, never both
    /fsae/control/nmpc_predicted_path geometry_msgs/PoseArray        NMPC's predicted horizon,
                                                                     only published when
                                                                     use_nmpc=true (see
                                                                     nmpc_core.py's xy_at())
    /fsds/control_command             fs_msgs/ControlCommand         steering/throttle/brake
                                                                     (standalone_output=true)
    /fsae/control/cmd_vel             ackermann_msgs/AckermannDriveStamped  (standalone_output=false)

Both control-output topics are subscribed; whichever one is actually being
published (depends on the `standalone_output` launch arg) is the one that
updates the stats panel, the other simply never fires.
"""

from collections import deque

import matplotlib
import numpy as np

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt  # noqa: E402 (backend must be selected first)
from matplotlib.animation import FuncAnimation  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)

from ackermann_msgs.msg import AckermannDriveStamped  # noqa: E402
from fs_msgs.msg import ControlCommand  # noqa: E402
from fsae_interfaces.msg import Track  # noqa: E402
from geometry_msgs.msg import PoseArray, PoseStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402

# Close-up window: how far ahead/behind/either side of the car the axes span
# (metres). Small enough to actually see steering/tracking detail up close,
# per the "close up" ask -- this is a debug tool, not a full-track overview.
VIEW_HALF_WIDTH = 15.0
VIEW_AHEAD = 25.0
VIEW_BEHIND = 8.0

TRAIL_MAXLEN = 2000        # ~40s at 50 Hz control rate, plenty for a debug view
REDRAW_HZ = 25.0           # window refresh rate; independent of the 50 Hz control loop.
# Not pushed higher than this: each frame does a full ax.clear() + re-plot
# (scatter/lines/legend/text), not a blit-based partial update, so redraw
# cost scales with cone/path point counts: past ~25-30 Hz the redraw itself
# starts taking longer than the interval on a typical track-sized cone map,
# and frames just queue up behind rclpy.spin_once() instead of arriving
# sooner. Move to blitting (redrawing only changed artists) if a higher rate
# is ever needed.


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
        q = msg.pose.orientation
        # Yaw from quaternion (planar, matches sim_perception.py's own
        # convention: FSDS/ENU, z-axis rotation only).
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.car_yaw = float(np.arctan2(siny_cosp, cosy_cosp))
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


def main():
    rclpy.init()
    node = LiveVizNode()

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal')

    def redraw(_frame):
        # Process every callback queued since the last frame, not just one:
        # at REDRAW_HZ < 50 Hz (the control loop's own rate), a single
        # spin_once per frame falls behind and each redraw would show a
        # stale, queued-up state rather than the latest tick. rclpy has no
        # built-in "drain everything ready right now" call, so spin_once
        # (non-blocking, timeout_sec=0) is called in a bounded loop instead;
        # each of this node's callbacks is a cheap attribute write, so a
        # 50 Hz backlog empties in well under a millisecond, and the loop
        # exits itself (via the callback-count check) once nothing is left,
        # rather than always running to the cap.
        for _ in range(20):
            before = node._callback_count
            rclpy.spin_once(node, timeout_sec=0.0)
            if node._callback_count == before:
                break
        ax.clear()
        ax.set_aspect('equal')

        if node.left_cones.size:
            ax.scatter(node.left_cones[:, 0], node.left_cones[:, 1],
                       c='tab:blue', marker='^', s=25, label='left (blue)')
        if node.right_cones.size:
            ax.scatter(node.right_cones[:, 0], node.right_cones[:, 1],
                       c='gold', marker='^', s=25, label='right (yellow)')

        # Precomputed mode: the planner never runs (sim.launch.py gates it
        # off), so ref_path stays empty and static_ref_path carries the real
        # reference instead -- draw whichever one actually has data, not
        # both (they're never populated at the same time in practice).
        if node.static_ref_path.size:
            ax.plot(node.static_ref_path[:, 0], node.static_ref_path[:, 1],
                    c='tab:gray', lw=1.5, ls='--', label='precomputed reference path')
        elif node.ref_path.size:
            ax.plot(node.ref_path[:, 0], node.ref_path[:, 1],
                    c='tab:gray', lw=1.5, ls='--', label='reference/planner path')

        if node.nmpc_pred_path.size:
            ax.plot(node.nmpc_pred_path[:, 0], node.nmpc_pred_path[:, 1],
                    c='tab:red', lw=2.0, marker='o', ms=3, label='NMPC predicted horizon')

        if len(node.trail) >= 2:
            trail = np.array(node.trail)
            ax.plot(trail[:, 0], trail[:, 1], c='tab:green', lw=1.2, alpha=0.7,
                    label='driven trail')

        if node.have_pose:
            tx, ty = get_car_triangle(node.car_x, node.car_y, node.car_yaw)
            ax.fill(tx, ty, c='black', label='car')

            fwd = np.array([np.cos(node.car_yaw), np.sin(node.car_yaw)])
            right = np.array([np.sin(node.car_yaw), -np.cos(node.car_yaw)])
            center = np.array([node.car_x, node.car_y])
            forward_span = center + fwd * VIEW_AHEAD
            backward_span = center - fwd * VIEW_BEHIND
            ax.set_xlim(min(forward_span[0], backward_span[0]) - VIEW_HALF_WIDTH,
                        max(forward_span[0], backward_span[0]) + VIEW_HALF_WIDTH)
            ax.set_ylim(min(forward_span[1], backward_span[1]) - VIEW_HALF_WIDTH,
                        max(forward_span[1], backward_span[1]) + VIEW_HALF_WIDTH)
            del right  # reserved for a future car-relative (rotated) view

        stats = (
            f"v = {node.car_speed:.2f} m/s\n"
            f"steer = {node.steering:+.3f}\n"
            f"throttle = {node.throttle:.2f}  brake = {node.brake:.2f}\n"
        )
        if node.cmd_speed_target is not None:
            stats += f"cmd v_target = {node.cmd_speed_target:.2f} m/s\n"
        stats += f"control topic: {node.control_topic or '(none yet)'}\n"
        stats += f"NMPC horizon: {'yes' if node.nmpc_pred_path.size else 'no'}"
        ax.text(0.02, 0.98, stats, transform=ax.transAxes, va='top', ha='left',
                fontsize=9, family='monospace',
                bbox=dict(boxstyle='round', fc='white', alpha=0.85))

        ax.legend(loc='lower right', fontsize=8)
        ax.set_title('Live MPC debug view')

    ani = FuncAnimation(fig, redraw, interval=1000.0 / REDRAW_HZ, cache_frame_data=False)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
