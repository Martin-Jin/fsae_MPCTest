"""
MPC path-tracking controller.

A drop-in alternative to the Stanley controller: it follows the planned
centreline using one of two optimisers, constructed unconditionally in
__init__ (mpc_core.MPCController, a linear time-varying MPC, by default; or
nmpc_core.NMPCController when use_nmpc=true). Unlike Stanley (which reacts
to the instantaneous cross-track/heading error), the MPC plans a 1.25 s
horizon, which is what damps the high-speed left-right sway.

This node has TWO output modes, selected by the `standalone_output` ROS2
parameter (default true):

  standalone_output=false — forwards only the MPC's steering command through
    the car stack's shared cmd_vel abstraction (speed + steering_angle on
    /fsae/control/cmd_vel); fsds_bridge.py then computes throttle/brake from
    a simple speed-error P-loop against the curvature-limited target, the
    same way it does for the Stanley controller. Run fsds_bridge.py alongside
    this node in that mode.

  standalone_output=true — publishes fs_msgs/ControlCommand directly, using
    the MPC's own (steering, throttle, brake) output unchanged. That
    preserves the offline-tuned longitudinal behaviour from the fsae_MPCTest
    repo's tuner/offline_tuner.py and gui/simulation.py, which both drive the
    vehicle plant with the MPC's own commanded acceleration (see that repo's
    sim/rollout_core.py) — the false mode's accel-discarding design does not.
    This node also owns GO-gating, stale-command braking, and cone-proximity
    braking itself in this mode (mirroring fsds_bridge.py's own logic against
    the same inputs) — do NOT launch fsds_bridge.py alongside this node when
    standalone_output=true, its output would be published but never used,
    and it would race this node for /fsds/control_command.
    control.launch.py's `standalone_output` launch arg already handles this
    — it skips fsds_bridge automatically for this mode.

The standalone_output=true design was originally ported from an
fsae_MPCTest prototype implementing the same direct-ControlCommand approach
against an older ROS 2 topic/message interface, then merged into this single
node (with the false mode) so the two integrations share one file instead of
being selected by two separate launchable executables.

    in   /fsae/planning/selected_trajectory  geometry_msgs/PoseArray        planner centreline
    in   /fsae/slam/car_position             geometry_msgs/PoseStamped      x,y in position; yaw in orientation.w
    in   /fsae/slam/car_odom                 nav_msgs/Odometry              speed + yaw-rate feedback; SAME
                                                                             snapshot as car_position above
                                                                             (both from sim_perception.py's
                                                                             one _odom_cb per tick — see that
                                                                             node's "Speed/yaw-rate
                                                                             synchronisation" docstring note.
                                                                             Do NOT subscribe to the raw
                                                                             /fsds/testing_only/odom directly.)
    in   /fsds/signal/go                     fs_msgs/GoSignal               race start (standalone_output=true only)
    in   /fsae/perception/cone_detection     fsae_interfaces/ConeDetection  proximity e-brake, car-local frame
                                                                             (standalone_output=true only)
    out  /fsae/control/cmd_vel               ackermann_msgs/AckermannDriveStamped  (standalone_output=false)
    out  /fsds/control_command                fs_msgs/ControlCommand               (standalone_output=true)
    out  /fsae/control/static_reference_path  geometry_msgs/PoseArray        one-shot, TRANSIENT_LOCAL (path_map_path
                                                                             set only) — self._static_path, for
                                                                             live_viz.py's debug display only, not
                                                                             read by anything in the control loop

CONTROL LOOP PHASES (see _control_step)
----------------------------------------------------------------------------
  Phase 1 (standalone_output=true only) — Hold at start line until GO signal
            received.
  Phase 2 — Emergency brake/reset if the planner path is missing/stale
            (>PATH_TIMEOUT old) or has fewer than 2 points, or the SLAM pose
            hasn't arrived yet; also resets the MPC so it doesn't warm-start
            from a stale trajectory once the path returns. When
            path_map_path is set, the path can never be "stale" (see that
            param's declaration) — only the pose check still applies. In
            standalone_output=true mode this publishes an explicit brake
            command; in false mode it publishes nothing and relies on
            fsds_bridge's own cmd_vel timeout to brake.
  Phase 3 — Normal MPC solve via MPCController.compute().
  Phase 4 (standalone_output=true only) — Cone-proximity brake override:
            hard-overrides the MPC's throttle/brake (not steering) if a cone
            is inside the dynamic braking corridor. After
            CONE_RESET_THRESHOLD seconds of continuous braking, the MPC is
            reset exactly once (edge-triggered on the rising duration
            threshold, re-armed once the brake clears).
  Phase 4a — Telemetry logging of the *final* (post-override) command.
  Phase 5 — Publish.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from ackermann_msgs.msg import AckermannDriveStamped
from fs_msgs.msg import ControlCommand, GoSignal
from fsae_interfaces.msg import ConeDetection
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from fsae_control.control_utils import (
    load_path_profile_csv,
    load_path_heading_profile_csv,
    load_speed_profile_csv,
)
from fsae_control.lmpc import MPCController
from fsae_control.nmpc import NMPCController
from fsae_control.mpc.mpc_params import (
    declare_mpc_params,
    mpc_params_from_node,
)
from fsae_control.mpc.nmpc_params import (
    declare_nmpc_params,
    nmpc_params_from_node,
)
from fsae_control.telemetry import (
    ControlLogger,
    LapProgressTracker,
    HorizonAccuracyTracker,
    build_config_lines,
)
from fsae_control.mpc.control_step import _ControlStepMixin
from fsae_control.mpc.debug_publish import _DebugPublishMixin
from fsae_control.mpc.node_constants import CONTROL_HZ


class MPCControllerNode(_ControlStepMixin, _DebugPublishMixin, Node):
    def __init__(self):
        super().__init__('controller')

        self.declare_parameters(
            namespace='',
            parameters=[
                # Selects the output mode documented in this module's
                # docstring — false = steering only via cmd_vel/fsds_bridge,
                # true = the MPC's own throttle/brake published directly.
                # This is a topology switch (which topics/logic own the
                # output), not a QP tuning weight, so it is a plain node
                # parameter rather than a field on MPCParams.
                ('standalone_output', True),
                ('v_max', 20.0),      # m/s — top speed on straights
                ('v_min', 1.5),       # m/s — minimum speed through tight corners
                # Output steering low-pass (EMA); 1.0 disables. Only applied
                # when standalone_output=false.
                ('steer_lp', 0.3),
                # Real-time curvature-lookahead speed cap layered under the
                # precomputed speed profile (map_path) — see
                # control_utils.dynamic_speed_cap()'s docstring. No effect
                # when map_path is unset. Mirrors fsae_MPCTest/settings.py's
                # ENABLE_DYNAMIC_SPEED_CAP / DYNAMIC_CAP_A_LAT_MAX /
                # DYNAMIC_CAP_SAFETY.
                ('enable_dynamic_speed_cap', True),
                ('dynamic_cap_a_lat_max', 3.2),   # m/s^2
                ('dynamic_cap_safety', 0.9),
                ('log_csv', False),   # write CSV telemetry to log_dir
                ('log_dir', ''),      # '' -> ~/fsae_logs
                ('map_path', ''),     # '' -> live curvature_speed() (default);
                                       # else a fsae_MPCTest tuner/export_speed_profile.py
                                       # CSV to use instead — see
                                       # USE_PRECOMPUTED_SPEED_PROFILE in
                                       # fsae_MPCTest/settings.py.
                ('path_map_path', ''),  # '' -> live /fsae/planning/selected_trajectory
                                       # (default); else the SAME kind of CSV as map_path,
                                       # used for the tracked PATH instead of just speed —
                                       # see USE_PLANNER=False in fsae_MPCTest/settings.py
                                       # (the offline equivalent -- no separate flag exists
                                       # there). Removes centerline_planner.py from the
                                       # control loop entirely, to isolate controller/plant
                                       # tracking error from planner-induced path error.
                ('use_precomputed_heading_profile', False),  # only has an effect
                                       # when path_map_path is ALSO set -- see
                                       # mpc_core.py's set_heading_profile() and
                                       # late_turn_in_investigation.md Part 8/9. Uses
                                       # raceline_optimizer.py's shaped psi_target
                                       # column (heading-lead reference) in place of
                                       # the geometric path tangent for e_psi's
                                       # reference ONLY (e_y is unaffected). Default
                                       # False: land off, prove live before flipping.
            ],
        )
        self._standalone_output = self.get_parameter(
            'standalone_output').get_parameter_value().bool_value

        # All MPCController tuning (Q/R/R_rate weights, adaptive-gain shape
        # constants, feature flags) — see mpc_params.py's MPCParams for the
        # full field list and fsae_params.yaml's controller block for the
        # launch-time defaults/overrides.
        declare_mpc_params(self)
        mpc_params = mpc_params_from_node(self)
        # NMPCParams: the nonlinear-MPC controller's own tunables plus its
        # master switch (use_nmpc, default False). Declared unconditionally so
        # control.launch.py can always pass them; nothing below changes unless
        # use_nmpc is true. See nmpc_params.py for why these are a separate
        # dataclass from MPCParams (settings.py parity) and nmpc_core.py for
        # the formulation.
        declare_nmpc_params(self)
        nmpc_params = nmpc_params_from_node(self)

        self._v_max = self.get_parameter('v_max').get_parameter_value().double_value
        self._v_min = self.get_parameter('v_min').get_parameter_value().double_value
        self._steer_lp = self.get_parameter('steer_lp').get_parameter_value().double_value
        self._enable_dynamic_speed_cap = self.get_parameter(
            'enable_dynamic_speed_cap').get_parameter_value().bool_value
        self._dynamic_cap_a_lat_max = self.get_parameter(
            'dynamic_cap_a_lat_max').get_parameter_value().double_value
        self._dynamic_cap_safety = self.get_parameter(
            'dynamic_cap_safety').get_parameter_value().double_value

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

        # Static precomputed path (see path_map_path above). Loaded once at
        # startup; self._path is populated from this immediately and never
        # overwritten by _path_cb while it is set, so Phase 2/3 below don't
        # need to know which source is active. None = normal live-topic mode.
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

        self._heading_profile: np.ndarray | None = None
        use_precomputed_heading_profile = self.get_parameter(
            'use_precomputed_heading_profile').get_parameter_value().bool_value
        if use_precomputed_heading_profile:
            if path_map_path:
                try:
                    self._heading_profile = load_path_heading_profile_csv(path_map_path)
                except (OSError, ValueError) as exc:
                    self.get_logger().error(
                        f'Failed to load heading profile from path_map_path='
                        f'{path_map_path}: {exc}. Falling back to geometric heading.'
                    )
            else:
                self.get_logger().info(
                    'use_precomputed_heading_profile=True but path_map_path is '
                    'unset — nothing to load, ignoring.'
                )
        self._delta_filt: float | None = None   # filtered steering state (standalone_output=false only)

        self._telemetry = None
        if self.get_parameter('log_csv').get_parameter_value().bool_value:
            log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
            tag = 'mpc_standalone' if self._standalone_output else 'mpc'
            self._telemetry = ControlLogger(tag, log_dir=log_dir)
            self.get_logger().info(f'CSV telemetry -> {self._telemetry.paths[0]}')

        # Drives close()'s progress/reached_end/time_bonus so composite_score
        # reflects how far/fast the car actually got instead of being pinned
        # at the DNF floor (see LapProgressTracker's docstring). Needs the
        # precomputed speed profile for its ds/v_target optimal-time
        # integral, so it's only available in that mode — a live-planner run
        # still logs and scores everything except the time-based terms.
        self._lap_tracker: LapProgressTracker | None = None
        if self._telemetry is not None and self._speed_profile is not None:
            self._lap_tracker = LapProgressTracker(*self._speed_profile)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(PoseArray, '/fsae/planning/selected_trajectory', self._path_cb, 10)
        self.create_subscription(PoseStamped, '/fsae/slam/car_position', self._pose_cb, 10)
        # /fsae/slam/car_odom, NOT the raw /fsds/testing_only/odom -- see this
        # file's own docstring and sim_perception.py's "Speed/yaw-rate
        # synchronisation" note. The raw topic raced sim_perception's own
        # separate subscription to the same 250 Hz publisher, so car_speed/
        # car_yaw_rate could reflect a different odom instant than
        # car_pos/car_yaw on any given tick.
        self.create_subscription(Odometry, '/fsae/slam/car_odom', self._odom_cb, sensor_qos)

        # GO-gating and cone-proximity braking are this node's own
        # responsibility only in standalone_output=true mode (fsds_bridge.py
        # owns them otherwise) -- see this module's docstring.
        self._go_received = False
        self._cones_local: np.ndarray = np.empty((0, 2))
        self._cone_brake_duration = 0.0
        self._cone_reset_done = False
        if self._standalone_output:
            self.create_subscription(GoSignal, '/fsds/signal/go', self._go_cb, 10)
            self.create_subscription(
                ConeDetection, '/fsae/perception/cone_detection', self._cone_cb, 10)
            self.pub_cmd = self.create_publisher(ControlCommand, '/fsds/control_command', 10)
        else:
            self.pub_cmd = self.create_publisher(AckermannDriveStamped, '/fsae/control/cmd_vel', 10)

        # NMPC's predicted horizon (Cartesian, from last_telemetry['nmpc_pred_xy'],
        # see nmpc_core.py's xy_at()), for live_viz.py only -- not read by
        # anything else in this stack, empty/absent whenever the LTV-QP path
        # is in use (last_telemetry never has this key in that case).
        self.pub_nmpc_pred_path = self.create_publisher(
            PoseArray, '/fsae/control/nmpc_predicted_path', 10)

        # Per-tick weighted-error breakdown + solve time, for live_viz.py's
        # debug panel only. JSON over a plain String rather than a new
        # fsae_interfaces .msg: this is a debug-only, best-effort field set
        # with no other consumer, not a stable interface worth a schema.
        self.pub_debug_weights = self.create_publisher(
            String, '/fsae/control/debug_weights', 10)

        # Per-lap score + horizon-accuracy summary, published the instant a
        # lap completes (see LapProgressTracker.update()'s return value) --
        # live_viz.py's lap panel. JSON over a plain String, same rationale
        # as pub_debug_weights above (debug-only, best-effort, no schema
        # worth a dedicated .msg). RELIABLE + KEEP_LAST(10): a lap
        # completion is a one-shot, low-rate event (once per lap, not once
        # per tick like debug_weights), so it must not be silently dropped
        # the way a BEST_EFFORT publish could under momentary congestion.
        self.pub_lap_summary = self.create_publisher(
            String, '/fsae/control/lap_summary',
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )

        self._path: np.ndarray = (
            self._static_path if self._static_path is not None else np.empty((0, 2))
        )

        # live_viz.py had no way to show the ACTUAL reference being driven
        # against in precomputed-path mode. THREE earlier attempts got this
        # wrong before landing here. One and two tried publishing the static
        # path onto /fsae/planning/selected_trajectory (the live planner's
        # own topic) to fix live_viz.py's subscription to it: that topic's
        # other publisher, centerline_planner.py, had no use_precomputed_path
        # awareness and used to keep running/publishing regardless (no
        # gating existed in sim.launch.py, unlike e.g. cone_recorder's
        # IfCondition), so a one-shot publish only won a race against it for
        # an instant, and publishing an ~1000-point PoseArray every 50 ms
        # tick to try to keep winning measurably inflated solve_ms and
        # caused a genuine live stall. Fixed the live-planner side of that
        # at the source (sim.launch.py now gates planning.launch.py's
        # inclusion on use_precomputed_path, so the planner never runs in
        # this mode) -- but attempt three's one-shot-after-a-fixed-delay
        # publish, still onto the SAME shared topic, STILL showed nothing
        # live: launch_all.sh starts live_viz.py well before this node even
        # exists (see its own "topics simply have no data yet" comment), so
        # a plain VOLATILE publish is a genuine race against ROS2 discovery
        # completing on live_viz.py's side with no guaranteed margin, timer
        # delay or not -- confirmed live (a standalone repro showed the
        # message correctly logged as sent by this node, but never observed
        # by a subscriber that started earlier).
        #
        # Fixed properly with its OWN topic + TRANSIENT_LOCAL durability,
        # not a bigger delay: /fsae/planning/selected_trajectory still also
        # carries the LIVE planner's own (VOLATILE) output in non-
        # precomputed mode, and a TRANSIENT_LOCAL subscriber cannot match a
        # VOLATILE publisher under ROS2's QoS compatibility rules -- putting
        # the static path there under TRANSIENT_LOCAL would have broken
        # live-planner-mode viewing instead. A separate topic sidesteps that
        # entirely. TRANSIENT_LOCAL removes the discovery-timing race
        # itself: ROS2 guarantees a late-joining subscriber (also
        # TRANSIENT_LOCAL, see live_viz.py's matching subscription)
        # receives the publisher's last message regardless of when it
        # connects. See planner_only_lap2_corner_spinout.md.
        self._static_path_pub = None
        if self._static_path is not None:
            static_path_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self._static_path_pub = self.create_publisher(
                PoseArray, '/fsae/control/static_reference_path', static_path_qos)
            self._publish_static_path_once()
        # Static path never goes stale (no topic to lose) — treated as
        # "always fresh" by never being touched by the staleness check below,
        # rather than by faking a stamp that keeps advancing on its own.
        self._path_stamp = None
        self._have_pose = False
        self._car_pos = np.zeros(2)
        self._car_yaw = 0.0
        self._car_speed = 0.0
        self._car_vy = 0.0
        self._car_yaw_rate = 0.0
        self._pose_stamp = None
        # Previous tick's speed target, for the rise-rate limiter. None = no
        # history yet (first tick after start or after a reset), so the first
        # target passes through unlimited rather than ramping up from zero.
        self._v_des_prev: float | None = None
        # Previous tick's tracking-error speed gate, for GATE_RATE_LIMIT below.
        # None = no history yet, so the first gate value passes through
        # unlimited (nothing to ramp from).
        self._gate_prev: float | None = None
        # Previous tick's LIVE curvature_speed() output, for V_CURV_FALL_RATE
        # below. Only used in that branch (never the precomputed-track oracle
        # lookup); None = no history yet, so the first value passes through
        # unlimited.
        self._v_curv_prev: float | None = None

        dt = 1.0 / CONTROL_HZ
        # Controller selection. use_nmpc=False (default) constructs exactly
        # what this node has always constructed; the NMPC is a separate class
        # with the same compute()/reset()/set_heading_profile()/
        # last_telemetry surface, so nothing downstream branches on which
        # one is running. NMPCController ALSO has a set_static_path(), but
        # it means something entirely different there (see below) --
        # MPCController no longer has one at all.
        if nmpc_params.use_nmpc:
            self._mpc = NMPCController(
                dt=dt, params=mpc_params, nmpc=nmpc_params,
                logger=self.get_logger(),
            )
            self.get_logger().warn(
                f'use_nmpc=True: running the NONLINEAR MPC '
                f'(nmpc_core.NMPCController, N={self._mpc.N}, '
                f'sqp_iters={nmpc_params.nmpc_sqp_iters}) instead of the '
                'LTV-QP MPCController. Its adaptive gain schedule and '
                'use_precomputed_heading_profile do NOT apply -- see '
                'nmpc_core.py.'
            )
            # NMPCController.set_static_path() precomputes the arc-length /
            # curvature / reference-heading profile its prediction needs,
            # which is not optional and has nothing to do with the deleted
            # CornerMap -- without this call it would rebuild that on the
            # first tick instead (correct, just not free).
            if self._static_path is not None:
                self._mpc.set_static_path(self._static_path)
        else:
            self._mpc = MPCController(dt=dt, N=35, params=mpc_params)

        # Prediction-horizon accuracy: NMPC-only (see HorizonAccuracyTracker's
        # docstring for why -- the LTV-QP path never exposes a Cartesian
        # horizon). Constructed unconditionally (cheap, no ROS deps) but
        # only ever fed when nmpc_params.use_nmpc, so a non-NMPC run's
        # tracker just sits empty and every pred_err_m/pred_acc_pct column
        # stays blank.
        self._horizon_acc: HorizonAccuracyTracker | None = None
        if nmpc_params.use_nmpc:
            self._horizon_acc = HorizonAccuracyTracker()

        if self._heading_profile is not None:
            self._mpc.set_heading_profile(self._heading_profile)
            self.get_logger().info(
                f'Shaped heading profile loaded from {path_map_path} '
                '(use_precomputed_heading_profile=True).'
            )

        # Full run-configuration dump into the CSV's score header -- see
        # build_config_lines()'s docstring; done here (after self._mpc
        # exists) since ControlLogger itself was constructed earlier (before
        # map_path/self._mpc were known) and this needs the fully-resolved
        # NMPC weights off the constructed controller object, not just the
        # raw (possibly -1.0) NMPCParams override fields.
        if self._telemetry is not None:
            nmpc_effective = None
            if nmpc_params.use_nmpc:
                nmpc_effective = {
                    'w_out': self._mpc.w_out.tolist(),
                    'r_delta': self._mpc.r_delta,
                    'r_a_accel': self._mpc.r_a_accel,
                    'r_a_brake': self._mpc.r_a_brake,
                    'r_rate': self._mpc.r_rate.tolist(),
                    'terminal_scale': self._mpc.terminal_scale,
                }
            self._telemetry.set_config_lines(build_config_lines(
                controller=('mpc_standalone' if self._standalone_output else 'mpc'),
                launch_flags={
                    'map_path': map_path,
                    'path_map_path': path_map_path,
                    'use_precomputed_heading_profile':
                        self.get_parameter('use_precomputed_heading_profile')
                            .get_parameter_value().bool_value,
                    'enable_dynamic_speed_cap': self._enable_dynamic_speed_cap,
                    'dynamic_cap_a_lat_max': self._dynamic_cap_a_lat_max,
                    'dynamic_cap_safety': self._dynamic_cap_safety,
                    'v_max': self._v_max, 'v_min': self._v_min,
                },
                mpc_params=mpc_params,
                nmpc_params=(nmpc_params if nmpc_params.use_nmpc else None),
                nmpc_effective=nmpc_effective,
            ))

        self.create_timer(dt, self._control_step)

        mode = 'standalone (ControlCommand direct)' if self._standalone_output else 'cmd_vel (fsds_bridge)'
        self.get_logger().info(f'MPC controller ready [{mode}] — waiting for a trajectory + car_position.')

    # ------------------------------------------------------------------
    # Subscribers (cache latest state; the timer does the work)
    # ------------------------------------------------------------------

    def _go_cb(self, msg: GoSignal) -> None:
        if not self._go_received:
            self._go_received = True
            self.get_logger().info('GO signal received.')

    def _path_cb(self, msg: PoseArray) -> None:
        if self._static_path is not None:
            # A precomputed path is active — ignore the live planner's output
            # entirely (still subscribed so the topic doesn't dangle, but
            # never written to self._path). See path_map_path above.
            return
        self._path = np.array(
            [[p.position.x, p.position.y] for p in msg.poses], dtype=np.float64
        ) if msg.poses else np.empty((0, 2))
        self._path_stamp = self.get_clock().now()

    def _publish_static_path_once(self) -> None:
        """
        One-shot: see the comment on _static_path_pub's construction for why
        TRANSIENT_LOCAL durability (not timing) is what makes "once" safe
        for a subscriber that connects later.
        """
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'map'
        for x, y in self._static_path:
            pose = Pose()
            pose.position.x, pose.position.y = float(x), float(y)
            pose_array.poses.append(pose)
        self._static_path_pub.publish(pose_array)

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

    def _cone_cb(self, msg: ConeDetection) -> None:
        pts = [[p.x, p.y] for p in msg.blue] + [[p.x, p.y] for p in msg.yellow]
        self._cones_local = np.array(pts, dtype=np.float64) if pts else np.empty((0, 2))

    def destroy_node(self) -> None:
        if self._telemetry is not None:
            if self._lap_tracker is not None:
                lap = self._lap_tracker.result(self.get_clock().now().nanoseconds * 1e-9)
                self._telemetry.close(
                    progress=lap['progress'], time_bonus=lap['time_bonus'],
                    reached_end=lap['reached_end'], lap_time_s=lap['lap_time_s'],
                    optimal_time_s=lap['optimal_time_s'],
                )
            else:
                self._telemetry.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerNode()
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
