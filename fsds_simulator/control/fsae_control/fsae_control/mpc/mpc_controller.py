"""
MPC path-tracking controller.

A drop-in alternative to the Stanley controller: it follows the planned
centreline and drives on the same optimiser (mpc_core.MPCController, a linear
time-varying MPC, or nmpc_core.NMPCController when use_nmpc=true) either
node built here always runs. Unlike Stanley (which reacts to the
instantaneous cross-track/heading error), the MPC plans a 1.25 s horizon,
which is what damps the high-speed left-right sway.

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
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from fs_msgs.msg import ControlCommand, GoSignal
from fsae_interfaces.msg import ConeDetection
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.time import Time

from fsae_control.control_utils import (
    curvature_speed, dynamic_speed_cap, load_path_profile_csv,
    load_path_heading_profile_csv,
    load_speed_profile_csv, precomputed_speed_at, tracking_error_speed_gate,
)
from fsae_control.mpc.mpc_core import MAX_STEER_RAD, MPCController
from fsae_control.mpc.nmpc_core import NMPCController
from fsae_control.mpc.mpc_params import declare_mpc_params, mpc_params_from_node
from fsae_control.mpc.nmpc_params import declare_nmpc_params, nmpc_params_from_node
from fsae_control.telemetry_logger import ControlLogger, LapProgressTracker, build_config_lines

CONTROL_HZ = 20.0   # must match MPCController(dt=0.05); dt = 1 / CONTROL_HZ

# CONE_BRAKE_DIST is also the ceiling on the dynamic corridor computed in
# _check_cone_proximity() (car_speed * 0.25, clipped to [0.6, CONE_BRAKE_DIST]).
# Only used when standalone_output=true (cone braking is fsds_bridge's job
# otherwise).
CONE_BRAKE_DIST      = 2.0    # m — forward corridor depth for cone proximity brake
CONE_BRAKE_WIDTH     = 0.18   # m — lateral half-width of braking corridor (36 cm total)
CONE_RESET_THRESHOLD = 0.3    # s — continuous cone-brake duration before one MPC reset
PATH_TIMEOUT         = 0.5    # s — reset the MPC if no fresh trajectory within this window

# Max rate (m/s^2) at which the speed TARGET may rise. Mirrors
# sim/rollout_core.SPEED_TARGET_RISE_RATE — keep both in sync. Decreases are
# never rate-limited; delaying a genuine brake request is the failure this is
# meant to prevent.
SPEED_TARGET_RISE_RATE = 7.0
# Max rate (gate-units/s, gate in [floor, 1.0]) at which
# tracking_error_speed_gate()'s output may change per tick, in EITHER
# direction. Without this, a fast-growing e_y sweeping through the gate's
# active band can compound with a simultaneously falling curvature-based
# speed target into a sharp single-tick v_desired drop that bypasses
# SPEED_TARGET_RISE_RATE (that limiter only bounds RISES), producing erratic
# a_cmd right after. Rate-limiting the gate itself spreads the same total
# slowdown over several ticks instead of one, keeping the safety response
# (the car DOES still slow down when tracking badly) while removing the
# single-tick cliff. Sized to the same order of magnitude as
# SPEED_TARGET_RISE_RATE by design choice, not measurement.
GATE_RATE_LIMIT = 2.0


class MPCControllerNode(Node):
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

        self._path: np.ndarray = (
            self._static_path if self._static_path is not None else np.empty((0, 2))
        )
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
                # self._speed_profile (path_X, path_Y, path_V), when loaded,
                # is a SEPARATE array from self._static_path -- a different
                # CSV load entirely (see load_speed_profile_csv vs
                # load_path_profile_csv above) -- so it is passed through
                # explicitly rather than assumed identical. No-op unless
                # nmpc_horizon_speed_profile_enabled is also set.
                if self._speed_profile is not None:
                    sp_x, sp_y, sp_v = self._speed_profile
                    self._mpc.set_static_path(
                        self._static_path,
                        path_v_xy=np.column_stack([sp_x, sp_y]), path_v=sp_v,
                    )
                else:
                    self._mpc.set_static_path(self._static_path)
        else:
            self._mpc = MPCController(dt=dt, N=35, params=mpc_params)

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

    # ------------------------------------------------------------------
    # Helpers (standalone_output=true only)
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

    # ------------------------------------------------------------------
    # Control step (fixed 20 Hz)
    # ------------------------------------------------------------------

    def _control_step(self) -> None:
        # Loop-entry timestamp for the cmd_latency_ms telemetry column — how
        # long this tick took from entering the callback to publishing a
        # command. Distinguishes "our compute is slow" from "our inputs were
        # already stale when we got them" (pose_age_s / path_age_s).
        _t_loop0 = time.perf_counter()

        # ── Phase 1 (standalone_output=true only): hold until GO ────────
        if self._standalone_output and not self._go_received:
            cmd = ControlCommand()
            cmd.throttle, cmd.steering, cmd.brake = 0.0, 0.0, 1.0
            cmd.header.stamp = self.get_clock().now().to_msg()
            self.pub_cmd.publish(cmd)
            self.get_logger().info('Waiting for GO signal...', throttle_duration_sec=2.0)
            return

        # ── Phase 2: emergency brake/reset on stale/missing path or pose ──
        # No topic backing a static path, so "staleness" doesn't apply to it
        # — the only thing that can fail here is the live pose (still
        # checked below via self._have_pose), exactly the safety net
        # path_map_path's docstring promises: a car running with a
        # precomputed path still brakes correctly if its live localisation
        # fails, same as before.
        if self._static_path is not None:
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
            if self._standalone_output:
                # Explicit brake command — this node owns braking, unlike
                # false mode below, which publishes nothing and relies on
                # fsds_bridge's own cmd_vel timeout to brake.
                cmd = ControlCommand()
                cmd.throttle, cmd.steering, cmd.brake = 0.0, 0.0, 1.0
                cmd.header.stamp = self.get_clock().now().to_msg()
                self.pub_cmd.publish(cmd)
                self.get_logger().warn(
                    'Trajectory path lost or stale — emergency braking.', throttle_duration_sec=1.0)
            return

        # ── Phase 3: MPC solve ───────────────────────────────────────────
        # Slice from the car's nearest point, gate on tracking error, and
        # rate-limit rises — the speed TARGET must be derived the same way
        # regardless of output mode, or they diverge in exactly the regime
        # that matters. See control_utils.tracking_error_speed_gate for the
        # rationale behind each step.
        if self._speed_profile is not None:
            # Track is already fully mapped (map_path param set) — look up
            # the oracle speed target instead of re-deriving it from the
            # live-built centreline. See load_speed_profile_csv()'s docstring.
            path_X, path_Y, path_V = self._speed_profile
            v_curv = precomputed_speed_at(self._car_pos, path_X, path_Y, path_V)

            # The oracle lookup above has no notion of the car's actual
            # current speed relative to how much runway is left to brake for
            # the upcoming corner — see control_utils.dynamic_speed_cap()'s
            # docstring. Layer a live curvature-lookahead cap under it
            # (min, never above the oracle target) so a corner reached
            # faster than planned still gets braked for in time.
            if self._enable_dynamic_speed_cap:
                path_ahead = self._path
                if len(path_ahead) > 2:
                    i_near = int(np.argmin(np.linalg.norm(path_ahead - self._car_pos, axis=1)))
                    if i_near < len(path_ahead) - 2:
                        path_ahead = path_ahead[i_near:]
                v_cap = dynamic_speed_cap(
                    path_ahead, v_max=self._v_max, v_min=self._v_min,
                    a_lat_max=self._dynamic_cap_a_lat_max,
                    safety=self._dynamic_cap_safety,
                )
                v_curv = min(v_curv, v_cap)
        else:
            path_ahead = self._path
            if len(path_ahead) > 2:
                i_near = int(np.argmin(np.linalg.norm(path_ahead - self._car_pos, axis=1)))
                if i_near < len(path_ahead) - 2:
                    path_ahead = path_ahead[i_near:]

            v_curv = curvature_speed(path_ahead, v_max=self._v_max, v_min=self._v_min)

        # Gate's own output is rate-limited (GATE_RATE_LIMIT) so its
        # tick-to-tick change is bounded — see that constant's own comment.
        tel = self._mpc.last_telemetry
        raw_gate = tracking_error_speed_gate(tel.get('e_y', 0.0), tel.get('e_psi', 0.0))
        if self._gate_prev is not None:
            max_step = GATE_RATE_LIMIT / CONTROL_HZ
            raw_gate = float(np.clip(raw_gate, self._gate_prev - max_step, self._gate_prev + max_step))
        self._gate_prev = raw_gate
        gate = raw_gate
        # Never gate below v_min: the car still needs authority to steer back.
        desired_speed = max(self._v_min, v_curv * gate)

        # Seed the ramp from the car's ACTUAL speed on the first tick after
        # startup/a fail-safe reset, not from an unlimited jump straight to
        # desired_speed -- see mpc_params.py / CLAUDE.md's standstill
        # steering-saturation note. Without this, a standing-start run asks
        # the controller (NMPC especially, via its e_v cost term) to track
        # the full-speed target from tick 0, which is the actual root cause
        # of the "steers hard at startup" symptom, not a plant/tyre-force bug.
        if self._v_des_prev is None:
            self._v_des_prev = self._car_speed
        desired_speed = min(desired_speed,
                            self._v_des_prev + SPEED_TARGET_RISE_RATE / CONTROL_HZ)
        self._v_des_prev = desired_speed

        # Age of the pose the MPC is about to solve against — how long ago it
        # was actually measured, not how long ago the callback fired. Lets
        # MPCController compensate for the real, unknown/time-varying delay
        # instead of assuming the state is fresh (see mpc_core.py compute()).
        pose_age_s = (self.get_clock().now() - Time.from_msg(self._pose_stamp)).nanoseconds * 1e-9

        # MPCController/NMPCController.compute() always returns the
        # FSDS-normalised (steering, throttle, brake) tuple regardless of
        # caller; standalone_output=true uses it directly, false mode
        # instead reads last_telemetry['delta_cmd'] (pre-normalisation
        # radians, +ve = left) below and forwards only that + the speed
        # target, keeping the cmd_vel abstraction intact.
        mpc_steering, mpc_throttle, mpc_brake = self._mpc.compute(
            path=self._path, car_pos=self._car_pos, car_yaw=self._car_yaw,
            car_speed=self._car_speed, desired_speed=desired_speed,
            car_yaw_rate=self._car_yaw_rate, pose_age_s=pose_age_s, car_vy=self._car_vy,
        )
        if self._standalone_output:
            steering, throttle, brake = mpc_steering, mpc_throttle, mpc_brake
        else:
            steering = float(self._mpc.last_telemetry.get('delta_cmd', 0.0))

            # Low-pass the steering command across ticks (matches the
            # Stanley node) so rapid left-right jitter never reaches the
            # servo. 1.0 disables. Only applied in this mode.
            if self._delta_filt is None or self._steer_lp >= 1.0:
                self._delta_filt = steering
            else:
                self._delta_filt += self._steer_lp * (steering - self._delta_filt)
            steering = self._delta_filt

        if self._standalone_output:
            cmd = ControlCommand()
            cmd.steering, cmd.throttle, cmd.brake = steering, throttle, brake

            # ── Phase 4: cone-proximity brake override ───────────────────
            if self._check_cone_proximity():
                cmd.throttle = 0.0
                cmd.brake = 1.0
                self._cone_brake_duration += 1.0 / CONTROL_HZ
                if self._cone_brake_duration >= CONE_RESET_THRESHOLD and not self._cone_reset_done:
                    self._mpc.reset()
                    self._v_des_prev = None   # see the stale-path reset above
                    self._gate_prev = None
                    self._cone_reset_done = True
                self.get_logger().warn(
                    f'Cone proximity brake active ({self._cone_brake_duration:.2f} s).',
                    throttle_duration_sec=0.5,
                )
            else:
                self._cone_brake_duration = 0.0
                self._cone_reset_done = False

            # ── Phase 4a: telemetry (post-override, reflects the final cmd) ─
            # log_control's steer argument is RADIANS of roadwheel angle.
            # cmd.steering is the normalised FSDS [-1, 1] command, so it must
            # be scaled back by MAX_STEER_RAD (and un-negated — mpc_core
            # flips sign for the FSDS convention) before logging.
            if self._telemetry is not None:
                tel = self._mpc.last_telemetry
                t = self.get_clock().now().nanoseconds * 1e-9
                steer_rad = -float(cmd.steering) * MAX_STEER_RAD
                # delta_cmd/a_cmd come from the MPC's own telemetry so the
                # logged score is computed on the solver's real [rad, m/s^2]
                # command pair. They fall back to the published command when
                # a fail-safe (cone brake / no solve) overrode the MPC, so
                # the score reflects what the car actually did.
                a_cmd = tel.get('a_cmd', 0.0)
                if cmd.brake > 0.0 and cmd.throttle == 0.0:
                    a_cmd = min(a_cmd, -float(cmd.brake) * self._mpc.a_max_brake)
                # Age of the planner path this solve consumed. A static
                # precomputed path (self._static_path set) is logged as
                # exactly 0.0 rather than None — it is never stale by
                # construction, and 0.0 keeps this column numeric for any
                # downstream analysis that assumes path_age_s is always a
                # float (see fsae_MPCTest's telemetry_logger.py mirror,
                # which must match this convention).
                if self._static_path is not None:
                    path_age_s = 0.0
                elif self._path_stamp is not None:
                    path_age_s = (self.get_clock().now() - self._path_stamp).nanoseconds * 1e-9
                else:
                    path_age_s = None
                self._telemetry.log_control(
                    t, self._car_pos[0], self._car_pos[1], self._car_yaw,
                    self._car_speed, desired_speed, steer_rad,
                    tel.get('e_y', 0.0), tel.get('e_psi', 0.0), self._car_yaw_rate,
                    delta_cmd=steer_rad, a_cmd=a_cmd,
                    pose_age_s=tel.get('pose_age_s'),
                    path_age_s=path_age_s,
                    n_delay=tel.get('n_delay'),
                    solve_ms=tel.get('solve_ms'),
                    cmd_latency_ms=(time.perf_counter() - _t_loop0) * 1e3,
                    adaptive=tel)
                self._telemetry.log_path(t, self._path)
                if self._lap_tracker is not None:
                    self._lap_tracker.update(self._car_pos, t, self._car_speed)

            # ── Phase 5: publish ──────────────────────────────────────────
            cmd.header.stamp = self.get_clock().now().to_msg()
            self.pub_cmd.publish(cmd)

            self.get_logger().info(
                f'MPC thr={cmd.throttle:.2f} brk={cmd.brake:.2f} steer={cmd.steering:.3f} | '
                f'v={self._car_speed:.1f}/{desired_speed:.1f} m/s',
                throttle_duration_sec=1.0,
            )
        else:
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.drive.speed = float(desired_speed)
            msg.drive.steering_angle = steering
            self.pub_cmd.publish(msg)

            if self._telemetry is not None:
                tel = self._mpc.last_telemetry
                t = self.get_clock().now().nanoseconds * 1e-9
                # steering is already the roadwheel angle in radians here
                # (this mode publishes an Ackermann steering_angle, not a
                # normalised FSDS command), so it is both the logged steer
                # and delta_cmd.
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
                if self._lap_tracker is not None:
                    self._lap_tracker.update(self._car_pos, t, self._car_speed)

            self.get_logger().info(
                f'cmd_vel: speed={desired_speed:.2f} m/s  steer={steering:.3f} rad  '
                f'v_actual={self._car_speed:.2f} m/s  '
                f'e_y={self._mpc.last_telemetry.get("e_y", 0.0):.2f}',
                throttle_duration_sec=1.0,
            )

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
