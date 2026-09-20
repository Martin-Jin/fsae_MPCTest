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
    /fsae/slam/car_position           geometry_msgs/PoseStamped      car pose, global frame.
                                                                     orientation is NOT a real
                                                                     quaternion: .w carries raw
                                                                     yaw (radians) directly, see
                                                                     sim_perception.py's
                                                                     _car_pose_msg()
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
    /fsae/control/debug_weights       std_msgs/String                 JSON: per-tick weighted
                                                                       tracking-error breakdown
                                                                       (e_y/e_psi/e_v: error,
                                                                       weight, cost, pct of total)
                                                                       + solve_ms, debug-only, see
                                                                       mpc_controller.py's
                                                                       _publish_debug_weights()
    /fsae/control/debug_stanley       std_msgs/String                 JSON: Stanley's own three
                                                                       control-law terms (heading
                                                                       error, atan2 cross-track,
                                                                       yaw-rate damping) + e_y/
                                                                       e_psi/steering, debug-only.
                                                                       Also this node's only
                                                                       positive "Stanley is
                                                                       active" signal, see
                                                                       stanley_controller.py's
                                                                       _publish_debug_stanley()

Both control-output topics are subscribed; whichever one is actually being
published (depends on the `standalone_output` launch arg) is the one that
updates the stats panel, the other simply never fires.
"""

import json
import os
import signal
import time
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
from std_msgs.msg import String  # noqa: E402

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
        self.debug_weights = None      # parsed JSON dict from /fsae/control/debug_weights, or None
        self.debug_stanley = None      # parsed JSON dict from /fsae/control/debug_stanley, or None
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

    def active_controller(self) -> str:
        """'mpc' or 'stanley', whichever debug topic fired most recently,
        or 'unknown' if neither has fired yet this session. Wall-clock
        arrival, not message content -- see the two _at fields' own
        comment for why."""
        if self._debug_weights_at == 0.0 and self._debug_stanley_at == 0.0:
            return 'unknown'
        return 'mpc' if self._debug_weights_at >= self._debug_stanley_at else 'stanley'


# Which terms (mpc_controller.py's _publish_debug_weights() dict keys)
# belong on which bar-graph panel, and each panel's own axis label. Grouped
# by unit family, NOT all in one chart -- a steering-rate term of a few
# milliradians/tick and a 0.3 m lateral error are both real costs but
# squared-and-weighted they sit at wildly different absolute magnitudes, so
# one shared 0-100% scale makes the smaller group always look like ~0% even
# when it is the dominant term within its own family. Matches the 'group'
# field _publish_debug_weights() tags each term with; kept as an explicit
# order/label table here (not derived from the message) so panel order is
# stable regardless of dict iteration order.
DEBUG_BAR_GROUPS = (
    # 'progress' and 'v_cap_hinge' are NMPC-only and only present when
    # nmpc_progress_enabled; absent terms are skipped, so listing them here
    # is inert in every other configuration. 'v_cap_hinge' REPLACES 'e_v'
    # in that mode rather than reusing its name (see mpc_controller.py's
    # _publish_debug_weights), so a bar labelled e_v is always a real
    # two-sided speed error and never a cap hinge sitting at zero.
    ('tracking', 'Tracking error cost (% of tracking total)',
     ('e_y', 'e_yd', 'e_psi', 'yaw_rate', 'e_v', 'v_cap_hinge', 'progress',
      'steering', 'accel')),
    ('effort', 'Input effort cost (% of effort total)',
     ('steer_effort', 'accel_effort')),
    ('rate', 'Input rate-of-change cost (% of rate total)',
     ('delta_u_steer', 'delta_u_accel')),
)

# Order the horizon-summed panel draws its bars in. Unlike DEBUG_BAR_GROUPS
# above (step-0-only, one 100% scale per unit-family group), every one of
# these IS on one shared scale together: each is a share of total_cost, the
# solver's true full-horizon objective, so they are genuinely comparable
# (see mpc_controller.py's _publish_debug_weights()'s horizon_terms).
DEBUG_HORIZON_TERMS = (
    'e_y', 'e_yd', 'e_psi', 'yaw_rate', 'e_v', 'v_cap_hinge', 'progress',
    'steer_effort', 'accel_effort', 'delta_u_steer', 'delta_u_accel',
)

# Stanley's debug_stanley terms shown on its own error panel (NOT its
# control-law-term panel below): just its two tracking errors, one shared
# 100% scale -- heading error and cross-track (lateral) error are both
# radians/metres-squared-weighted-free raw values here (Stanley has no QP
# cost weights), so "percentage" is share of |e_y|+|e_psi|, a simple
# at-a-glance "which error dominates" signal, analogous to MPC's tracking
# panel but with only the two terms Stanley actually has.
STANLEY_ERROR_TERMS = ('e_y', 'e_psi')

# Stanley's three additive control-law terms (see
# stanley_controller.py's _publish_debug_stanley()) -- percentage of the
# sum of their absolute values, i.e. "how much of this tick's total
# steering effort came from which term", matching the user's own framing
# ("atan term vs heading error term... as a percentage of total control
# input to steering").
STANLEY_LAW_TERMS = ('heading_error', 'atan_cross_track', 'yaw_rate_damping')


def main():
    rclpy.init()
    node = LiveVizNode()

    # Tk's mainloop (entered below via plt.show()) is a blocking C event
    # loop: CPython only runs a signal handler between bytecode
    # instructions, so with no handler registered here, SIGTERM/SIGINT
    # delivery while blocked in Tk was left to chance -- it only got through
    # incidentally, whenever FuncAnimation's own timer happened to pump the
    # loop back into Python. launch_all.sh's cleanup() sends SIGTERM (and
    # Ctrl+C sends SIGINT) expecting a prompt exit; os._exit(0) rather than
    # sys.exit()/plt.close() so the process dies immediately instead of
    # waiting for Tk to unwind its own C loop cleanly (which is exactly the
    # step that was hanging).
    def _handle_shutdown_signal(_signum, _frame):
        os._exit(0)
    signal.signal(signal.SIGTERM, _handle_shutdown_signal)
    signal.signal(signal.SIGINT, _handle_shutdown_signal)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal')

    # Separate window (per the "too much for one window" call): the weighted
    # cost breakdown has nothing spatial about it and doesn't need to share
    # a canvas with the map view. Left column: the 3 step-0 unit-family
    # panels (own 100% scale each). Right column: one tall panel with every
    # term's horizon-summed share of total_cost, on one shared scale, since
    # those percentages are genuinely comparable to each other (slack is
    # deliberately excluded here, unused -- see DEBUG_HORIZON_TERMS, so
    # these bars fall a little short of summing to total_cost).
    fig_dbg = plt.figure(figsize=(13, 9))
    gs = fig_dbg.add_gridspec(len(DEBUG_BAR_GROUPS), 2, width_ratios=[1.0, 1.0])
    ax_bars = [fig_dbg.add_subplot(gs[i, 0]) for i in range(len(DEBUG_BAR_GROUPS))]
    ax_horizon = fig_dbg.add_subplot(gs[:, 1])
    mpc_axes = ax_bars + [ax_horizon]

    # Stanley's own debug figure -- separate from fig_dbg (built once,
    # same as fig_dbg, then shown/hidden as a whole depending on which
    # controller is actually active, see redraw_debug()). Two panels: the
    # user's own two asks, one shared window each.
    fig_stanley = plt.figure(figsize=(8, 6))
    gs_stanley = fig_stanley.add_gridspec(2, 1)
    ax_stanley_error = fig_stanley.add_subplot(gs_stanley[0, 0])
    ax_stanley_law = fig_stanley.add_subplot(gs_stanley[1, 0])
    stanley_axes = [ax_stanley_error, ax_stanley_law]

    fig_dbg.suptitle('MPC weighted-cost breakdown (debug)')
    fig_stanley.suptitle('Stanley control-law breakdown (debug)')

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

        controller = node.active_controller()
        controller_label = {'mpc': 'MPC', 'stanley': 'Stanley', 'unknown': '(unknown)'}[controller]

        stats = (
            f"controller = {controller_label}\n"
            f"v = {node.car_speed:.2f} m/s\n"
            f"steer = {node.steering:+.3f}\n"
            f"throttle = {node.throttle:.2f}  brake = {node.brake:.2f}\n"
        )
        if node.cmd_speed_target is not None:
            stats += f"cmd v_target = {node.cmd_speed_target:.2f} m/s\n"
        stats += f"control topic: {node.control_topic or '(none yet)'}\n"
        # NMPC horizon line is MPC-specific (Stanley never predicts a
        # horizon at all) -- only shown when MPC is the one actually active,
        # rather than printing a permanently-"no" line for a Stanley run.
        if controller != 'stanley':
            stats += f"NMPC horizon: {'yes' if node.nmpc_pred_path.size else 'no'}"
        # Progress-term diagnostics (nmpc_progress_enabled only; the key is
        # absent on every other run, so nothing is printed then). cap_over
        # separates "the cap is holding the car back" from "the car chose to
        # go slower than it was allowed", which is the question the progress
        # term exists to change the answer to. s_gap GROWING tick-over-tick
        # means the solve is stuck or regressing, not converging.
        prog = (node.debug_weights or {}).get('progress')
        if prog:
            v_cap = prog.get('v_cap')
            over = prog.get('speed_cap_over')
            gap = prog.get('s_target_gap_end')
            stats += "\nprogress term:"
            if v_cap is not None:
                stats += f"\n  v_cap = {v_cap:.2f} m/s"
            if over is not None:
                state = 'CAP BINDING' if over > 0.05 else 'under cap'
                stats += f"\n  cap_over = {over:+.2f} m/s ({state})"
            if gap is not None:
                stats += f"\n  s_gap_end = {gap:.2f} m"
        ax.text(0.02, 0.98, stats.rstrip('\n'), transform=ax.transAxes, va='top', ha='left',
                fontsize=9, family='monospace',
                bbox=dict(boxstyle='round', fc='white', alpha=0.85))

        ax.legend(loc='lower right', fontsize=8)
        ax.set_title(f'Live {controller_label} debug view')

    def _draw_pct_bars(ax, all_names, present, pcts, details, red_threshold, xlabel):
        """Shared bar-graph renderer for every debug panel below: a
        horizontal 0-100% bar per name in ALL_NAMES, in that FIXED order,
        every single frame -- red past red_threshold else blue, with
        details[i] appended to each present bar's label.

        Takes the full fixed name list, not just the ones with data this
        tick, and always draws one row per name in ALL_NAMES: a name
        missing from `present` (this frame's actual mode has no data for
        it, e.g. 'progress' when nmpc_progress_enabled is off) still gets
        an empty grey row at its own fixed position instead of being
        omitted. Omitting it used to collapse every row below it upward by
        one slot the instant that term's presence changed tick to tick,
        which is what made the whole panel appear to jump around even
        though no single term's own value did anything unusual -- ax.clear()
        every frame plus barh() placing bars in LIST order (not a fixed
        category axis) means a shorter list is a visually different
        layout, not just fewer bars. Centralised so every panel wraps its
        label text the same way (see the wrap step below, added because
        long detail strings were being clipped past the figure's right
        edge)."""
        ax.clear()
        present_set = set(present)
        pct_by_name = dict(zip(present, pcts))
        detail_by_name = dict(zip(present, details))
        pcts_fixed = [pct_by_name.get(n, 0.0) for n in all_names]
        colors = ['tab:red' if p >= red_threshold else 'tab:blue'
                  if n in present_set else 'lightgrey'
                  for n, p in zip(all_names, pcts_fixed)]
        # ALL_NAMES passed straight through, not reordered: barh's own
        # bottom-to-top placement of a fixed list is exactly what the group
        # panels already rendered before this fix (first declared name at
        # the bottom), so this keeps their look unchanged and gives the
        # horizon panel that same fixed, stable order instead of its old
        # per-frame value sort.
        bars = ax.barh(all_names, pcts_fixed, color=colors)
        for name, bar in zip(all_names, bars):
            if name not in present_set:
                continue
            detail = detail_by_name[name]
            # Bar labels are drawn in DATA coordinates (x in [0, 100],
            # not axes-fraction), so a wide label on a near-100% bar
            # can extend past the axes' right edge and get clipped by
            # the figure boundary -- clip_on=False lets it draw into
            # the figure margin instead (tight_layout/subplots_adjust
            # below reserves that margin), and a fixed-width right
            # margin is reserved on every panel for exactly this.
            ax.text(bar.get_width() + 1.5, bar.get_y() + bar.get_height() / 2,
                    detail, va='center', ha='left', fontsize=7,
                    family='monospace', clip_on=False)
        ax.set_xlim(0, 100)
        ax.set_xlabel(xlabel, fontsize=8)

    def _set_window_visible(fig, visible: bool) -> None:
        """Figure.set_visible() only controls whether the figure's ARTISTS
        draw onto its own canvas -- it does not touch the OS-level window
        the TkAgg backend opened for it, so the inactive controller's debug
        figure (fig_stanley under MPC/NMPC, or vice versa) was left on
        screen showing nothing, a blank numbered "Figure" window with no
        way to tell why it was empty. Tk's own window object, reached via
        the canvas manager, is what actually needs withdraw()/deiconify()."""
        window = fig.canvas.manager.window
        if visible:
            window.deiconify()
        else:
            window.withdraw()

    _debug_visibility_state = {'show_stanley': None}

    def _sync_debug_visibility():
        """Which of the two debug figures is actually meaningful right now
        -- called from both figures' own animations (each figure needs its
        own FuncAnimation to redraw its own canvas; a single animation
        tied to one figure does not repaint a different figure's canvas)."""
        show_stanley = node.active_controller() == 'stanley'
        fig_dbg.set_visible(not show_stanley)
        fig_stanley.set_visible(show_stanley)
        for ax_ in mpc_axes:
            ax_.set_visible(not show_stanley)
        for ax_ in stanley_axes:
            ax_.set_visible(show_stanley)
        # withdraw()/deiconify() are window-manager calls, not cheap artist
        # toggles -- only issue them on an actual transition (this function
        # runs at REDRAW_HZ from TWO animations, ~50 calls/sec combined),
        # or a manually moved/resized debug window gets fought back into
        # place every frame even while the controller never changes.
        if _debug_visibility_state['show_stanley'] != show_stanley:
            _debug_visibility_state['show_stanley'] = show_stanley
            _set_window_visible(fig_dbg, not show_stanley)
            _set_window_visible(fig_stanley, show_stanley)
        return show_stanley

    def redraw_debug(_frame):
        if not _sync_debug_visibility():
            _redraw_mpc_debug()

    def redraw_debug_stanley(_frame):
        if _sync_debug_visibility():
            _redraw_stanley_debug()

    def _redraw_mpc_debug():
        # Weighted-cost breakdown, one bar-graph panel per unit-family group
        # (see DEBUG_BAR_GROUPS) -- which term is costing the solver the
        # most right now, as a share of ITS OWN GROUP's sum. See
        # mpc_controller.py's _publish_debug_weights() for what "weighted
        # cost" means here (error^2 * effective weight, step-0 only) and why
        # no group's bars sum to total_cost (shown in the figure suptitle):
        # total_cost is the true full-horizon solved objective, including
        # terms (full-horizon summation, jerk, slack) this bar graph does
        # not attempt to decompose per-tick.
        dw = node.debug_weights
        terms = dw.get('terms', {}) if dw is not None else {}
        for ax_bar, (_group, label, names) in zip(ax_bars, DEBUG_BAR_GROUPS):
            present = [n for n in names if n in terms]
            pcts = [terms[n]['pct'] for n in present]
            details = []
            for name in present:
                t = terms[name]
                # 'steering'/'accel' are synthetic combined bars (effort
                # + rate folded together, see _publish_debug_weights())
                # with no single error/weight of their own to show.
                if t['error'] is None:
                    detail = f"{t['pct']:.1f}%  (effort + rate combined)"
                else:
                    detail = f"{t['pct']:.1f}%  (v={t['error']:+.4f}, w={t['weight']:.2f})"
                details.append(detail)
            _draw_pct_bars(ax_bar, names, present, pcts, details, red_threshold=50.0, xlabel=label)

        # Right-side panel: every term's horizon-summed cost as a share of
        # total_cost, one shared scale (see DEBUG_HORIZON_TERMS' comment).
        # Fixed row order (DEBUG_HORIZON_TERMS' own declared order), NOT
        # sorted by current value -- sorting by value every frame was
        # exactly what made a term's row visibly jump as its cost share
        # crossed another term's, even though each term's OWN value was
        # moving smoothly. A reader tracking "is e_y still climbing"
        # should not have to re-find e_y's row after every redraw.
        horizon_terms = dw.get('horizon_terms', {}) if dw is not None else {}
        present_h = [n for n in DEBUG_HORIZON_TERMS if n in horizon_terms]
        pcts_h = [horizon_terms[n]['pct'] for n in present_h]
        details_h = [f"{horizon_terms[n]['pct']:.1f}%  (cost={horizon_terms[n]['cost']:.3f})"
                     for n in present_h]
        _draw_pct_bars(ax_horizon, DEBUG_HORIZON_TERMS, present_h, pcts_h, details_h,
                       red_threshold=30.0,
                       xlabel='Horizon-summed cost (% of true total solver cost)')
        ax_horizon.set_title('Every term, full predicted horizon', fontsize=9)

        header = []
        if dw is not None:
            total_cost = dw.get('total_cost')
            if total_cost is not None:
                header.append(f"true total solver cost = {total_cost:.4f}")
            solve_ms = dw.get('solve_ms')
            if solve_ms is not None:
                header.append(f"solve time = {solve_ms:.2f} ms")
        fig_dbg.suptitle('MPC weighted-cost breakdown (debug)'
                          + ('\n' + '   |   '.join(header) if header else ''))

    def _redraw_stanley_debug():
        # Panel 1: Stanley's own two tracking errors (heading, cross-track/
        # lateral), one shared 0-100% scale -- see STANLEY_ERROR_TERMS'
        # comment for why "percentage" here is share of |e_y|+|e_psi|.
        ds = node.debug_stanley
        e_y = ds.get('e_y') if ds is not None else None
        e_psi = ds.get('e_psi') if ds is not None else None
        error_values = {'e_y': e_y, 'e_psi': e_psi}
        total_abs = sum(abs(v) for v in error_values.values() if v is not None)
        present_e = [n for n in STANLEY_ERROR_TERMS if error_values.get(n) is not None]
        pcts_e = [(100.0 * abs(error_values[n]) / total_abs) if total_abs > 0.0 else 0.0
                  for n in present_e]
        labels_e = {'e_y': 'lateral error (e_y)', 'e_psi': 'heading error (e_psi)'}
        details_e = [f"{p:.1f}%  (v={error_values[n]:+.4f} rad or m)"
                     for n, p in zip(present_e, pcts_e)]
        # Fixed row per STANLEY_ERROR_TERMS entry (translated to its own
        # label), not just the ones with data this tick -- same fix as the
        # MPC panels above, so a term temporarily missing (e.g. e_psi not
        # yet published) gets an empty row at its own position instead of
        # collapsing the other row up to fill the gap.
        _draw_pct_bars(ax_stanley_error, [labels_e[n] for n in STANLEY_ERROR_TERMS],
                       [labels_e[n] for n in present_e], pcts_e, details_e,
                       red_threshold=60.0, xlabel='Tracking error (% of |e_y| + |e_psi|)')
        ax_stanley_error.set_title('Heading vs. lateral error', fontsize=9)

        # Panel 2: Stanley's own three additive control-law terms, as a
        # share of the total steering magnitude they combine to produce --
        # see stanley_controller.py's _publish_debug_stanley() and
        # STANLEY_LAW_TERMS' own comment.
        law_terms = ds.get('terms', {}) if ds is not None else {}
        present_l = [n for n in STANLEY_LAW_TERMS if n in law_terms]
        pcts_l = [law_terms[n]['pct'] for n in present_l]
        labels_l = {
            'heading_error': 'heading error term',
            'atan_cross_track': 'atan2 cross-track term',
            'yaw_rate_damping': 'yaw-rate damping term',
        }
        details_l = [f"{law_terms[n]['pct']:.1f}%  (v={law_terms[n]['value']:+.4f} rad)"
                     for n in present_l]
        _draw_pct_bars(ax_stanley_law, [labels_l[n] for n in STANLEY_LAW_TERMS],
                       [labels_l[n] for n in present_l], pcts_l, details_l,
                       red_threshold=60.0,
                       xlabel='Share of total steering magnitude (|heading| + |atan2| + |damping|)')
        ax_stanley_law.set_title('Control-law term breakdown', fontsize=9)

        header = []
        if ds is not None and ds.get('steering') is not None:
            header.append(f"steering command = {ds['steering']:+.4f} rad")
        fig_stanley.suptitle('Stanley control-law breakdown (debug)'
                              + ('\n' + '   |   '.join(header) if header else ''))

    fig.tight_layout()
    fig_dbg.tight_layout(rect=(0, 0, 0.92, 0.94))  # leave room for suptitle + right-margin labels
    fig_stanley.tight_layout(rect=(0, 0, 0.85, 0.92))  # narrower figure, wider label margin needed
    ani = FuncAnimation(fig, redraw, interval=1000.0 / REDRAW_HZ, cache_frame_data=False)
    ani_dbg = FuncAnimation(fig_dbg, redraw_debug, interval=1000.0 / REDRAW_HZ,
                             cache_frame_data=False)
    ani_stanley = FuncAnimation(fig_stanley, redraw_debug_stanley, interval=1000.0 / REDRAW_HZ,
                                 cache_frame_data=False)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
