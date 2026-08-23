"""
MPC path-tracking controller.

A drop-in alternative to the Stanley controller: it follows the planned
centreline and emits the same drive command on the car stack's control
interface (target speed + steering angle), so the downstream FSDS conversion in
fsds_bridge (speed->throttle/brake, GO gating, cone e-brake) is unchanged.

    in   /fsae/planning/selected_trajectory  geometry_msgs/PoseArray   path to follow
    in   /fsae/slam/car_position             geometry_msgs/PoseStamped x,y in position; yaw in orientation.w
    in   /fsae/slam/car_odom                 nav_msgs/Odometry         speed + yaw-rate feedback; SAME
                                                                        snapshot as car_position above (see
                                                                        sim_perception.py's "Speed/yaw-rate
                                                                        synchronisation" docstring note — do
                                                                        NOT subscribe to the raw
                                                                        /fsds/testing_only/odom directly)
    out  /fsae/control/cmd_vel               ackermann_msgs/AckermannDriveStamped  speed + steering_angle

The optimiser (mpc_core.MPCController) is a linear time-varying MPC ported from
the fsae_MPCTest repo.  Unlike Stanley (which reacts to the instantaneous
cross-track/heading error), the MPC plans a 1.25 s horizon, which is what damps
the high-speed left-right sway.  It natively outputs throttle/brake too, but to
preserve the cmd_vel -> fsds_bridge abstraction (and keep fsds_bridge the single
owner of GO-gating and cone-braking) we forward only its steering angle (rad)
and let the curvature-limited target speed drive the bridge's speed controller.

The control step runs on a FIXED 20 Hz timer (not the pose callback like
Stanley), because the MPC's discretisation assumes a constant dt = 0.05 s
between solves.
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.time import Time

from fsae_control.control_utils import (
    curvature_speed, dynamic_speed_cap, load_path_profile_csv,
    load_path_heading_profile_csv,
    load_speed_profile_csv, precomputed_speed_at, tracking_error_speed_gate,
    peak_kappa_ahead,
)
from fsae_control.mpc_core import MPCController, _corner_factor
from fsae_control.nmpc_core import NMPCController
from fsae_control.mpc_params import declare_mpc_params, mpc_params_from_node
from fsae_control.nmpc_params import declare_nmpc_params, nmpc_params_from_node
from fsae_control.telemetry_logger import ControlLogger, LapProgressTracker, build_config_lines

CONTROL_HZ    = 20.0   # must match MPCController(dt=0.05); dt = 1 / CONTROL_HZ

# Max rate (m/s^2) at which the speed TARGET may rise. Mirrors
# mpc_controller_standalone.SPEED_TARGET_RISE_RATE and
# sim/rollout_core.SPEED_TARGET_RISE_RATE — keep all three in sync. Decreases
# are never rate-limited; delaying a genuine brake request is the failure this
# is meant to prevent.
SPEED_TARGET_RISE_RATE = 7.0
# Max rate (gate-units/s) at which tracking_error_speed_gate()'s output may
# change per tick, in either direction. Mirrors
# mpc_controller_standalone.GATE_RATE_LIMIT — keep both in sync, see that
# constant's own comment for the rationale.
GATE_RATE_LIMIT = 2.0
PATH_TIMEOUT  = 0.5    # s — reset the MPC if no fresh trajectory within this window


class MPCControllerNode(Node):
    def __init__(self):
        super().__init__('controller')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('v_max', 18.0),      # m/s — top speed on straights
                ('v_min', 1.5),       # m/s — minimum speed through tight corners
                ('steer_lp', 0.3),    # output steering low-pass (EMA); 1.0 disables
                # Real-time curvature-lookahead speed cap layered under the
                # precomputed speed profile (map_path) — see
                # control_utils.dynamic_speed_cap()'s docstring. No effect
                # when map_path is unset (live curvature_speed() is already
                # the speed target in that mode). Mirrors
                # fsae_MPCTest/settings.py's ENABLE_DYNAMIC_SPEED_CAP /
                # DYNAMIC_CAP_A_LAT_MAX / DYNAMIC_CAP_SAFETY.
                ('enable_dynamic_speed_cap', True),
                ('dynamic_cap_a_lat_max', 3.2),   # m/s^2
                ('dynamic_cap_safety', 0.9),
                ('log_csv', False),   # write CSV telemetry to log_dir
                ('log_dir', ''),      # '' -> ~/fsae_logs
                ('map_path', ''),     # '' -> live curvature_speed() (default);
                                       # else a fsae_MPCTest tuner/export_speed_profile.py
                                       # CSV to use instead — see mpc_controller_standalone.py's
                                       # map_path param.
                ('path_map_path', ''),  # '' -> live /fsae/planning/selected_trajectory
                                       # (default); else the SAME kind of CSV as map_path,
                                       # used for the tracked PATH instead of just speed —
                                       # see mpc_controller_standalone.py's path_map_path param
                                       # and USE_PLANNER=False in fsae_MPCTest/settings.py (the
                                       # offline equivalent -- no separate flag exists there).
                ('use_precomputed_heading_profile', False),  # see
                                       # mpc_controller_standalone.py's identical param
                                       # for the full rationale — only has an effect
                                       # when path_map_path is ALSO set.
                # EXPERIMENTAL -- see mpc_controller_standalone.py's
                # identical params for the full mechanism (post-solve
                # moving-average filter on the final steering command, NOT a
                # QP weight change), and planning_control_sync.md's
                # "Post-solve output smoothing" section for the tuning
                # surface.
                ('output_smoothing_enabled', False),
                ('output_smoothing_alpha', 0.425),
                ('output_smoothing_corner_floor', 0.1),
                # Fades smoothing down, never off, as CURRENT tracking error
                # grows, on top of the curvature-based fade -- see
                # mpc_controller_standalone.py's identical params.
                ('output_smoothing_k_ey', 0.8),
                ('output_smoothing_k_epsi', 1.115),
                # Fades smoothing down BEFORE the car reaches a corner
                # already visible in the path -- see
                # mpc_controller_standalone.py's identical param.
                ('output_smoothing_lookahead_lead_s', 0.5),
            ],
        )
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

        self._output_smoothing_enabled = self.get_parameter(
            'output_smoothing_enabled').get_parameter_value().bool_value
        self._output_smoothing_alpha = self.get_parameter(
            'output_smoothing_alpha').get_parameter_value().double_value
        self._output_smoothing_corner_floor = self.get_parameter(
            'output_smoothing_corner_floor').get_parameter_value().double_value
        self._output_smoothing_k_ey = self.get_parameter(
            'output_smoothing_k_ey').get_parameter_value().double_value
        self._output_smoothing_k_epsi = self.get_parameter(
            'output_smoothing_k_epsi').get_parameter_value().double_value
        self._output_smoothing_lookahead_lead_s = self.get_parameter(
            'output_smoothing_lookahead_lead_s').get_parameter_value().double_value
        self._steer_filtered: float | None = None

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

        # Static precomputed path — see mpc_controller_standalone.py's
        # identical field for the full rationale/safety discussion.
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
        self._delta_filt: float | None = None   # filtered steering state

        self._telemetry = None
        if self.get_parameter('log_csv').get_parameter_value().bool_value:
            log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
            self._telemetry = ControlLogger('mpc', log_dir=log_dir)
            self.get_logger().info(f'CSV telemetry -> {self._telemetry.paths[0]}')

        # See mpc_controller_standalone.py's identical field for the full
        # rationale — drives close()'s progress/reached_end/time_bonus.
        self._lap_tracker: LapProgressTracker | None = None
        if self._telemetry is not None and self._speed_profile is not None:
            self._lap_tracker = LapProgressTracker(*self._speed_profile)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(PoseArray, '/fsae/planning/selected_trajectory', self._traj_cb, 10)
        self.create_subscription(PoseStamped, '/fsae/slam/car_position', self._pose_cb, 10)
        # /fsae/slam/car_odom, NOT the raw /fsds/testing_only/odom -- see this
        # file's own docstring and sim_perception.py's "Speed/yaw-rate
        # synchronisation" note. The raw topic raced sim_perception's own
        # separate subscription to the same 250 Hz publisher, so car_speed/
        # car_yaw_rate could reflect a different odom instant than
        # car_pos/car_yaw on any given tick.
        self.create_subscription(Odometry, '/fsae/slam/car_odom', self._odom_cb, sensor_qos)

        self.pub_cmd = self.create_publisher(AckermannDriveStamped, '/fsae/control/cmd_vel', 10)

        self._path: np.ndarray = (
            self._static_path if self._static_path is not None else np.empty((0, 2))
        )
        # Static path never goes stale (no topic to lose) — see the
        # standalone node's identical field/comment.
        self._path_stamp = None
        self._car_pos = np.zeros(2)
        # Previous tick's speed target, for the rise-rate limiter. None = no
        # history (start, or after a fail-safe reset) -> first target passes
        # through unlimited instead of ramping up from zero.
        self._v_des_prev: float | None = None
        # Previous tick's tracking-error speed gate, for GATE_RATE_LIMIT below.
        self._gate_prev: float | None = None
        self._car_yaw = 0.0
        self._car_speed = 0.0
        self._car_vy = 0.0
        self._car_yaw_rate = 0.0
        self._pose_stamp = None
        self._have_pose = False

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
        # Full run-configuration dump into the CSV's score header -- see
        # build_config_lines()'s docstring, and mpc_controller_standalone.py's
        # identical block for why this happens here (after self._mpc exists)
        # rather than at ControlLogger construction time.
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
                controller='mpc',
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
                    'output_smoothing_enabled': self._output_smoothing_enabled,
                    'output_smoothing_alpha': self._output_smoothing_alpha,
                    'output_smoothing_corner_floor': self._output_smoothing_corner_floor,
                    'output_smoothing_k_ey': self._output_smoothing_k_ey,
                    'output_smoothing_k_epsi': self._output_smoothing_k_epsi,
                    'output_smoothing_lookahead_lead_s': self._output_smoothing_lookahead_lead_s,
                },
                mpc_params=mpc_params,
                nmpc_params=(nmpc_params if nmpc_params.use_nmpc else None),
                nmpc_effective=nmpc_effective,
            ))

        if self._heading_profile is not None:
            self._mpc.set_heading_profile(self._heading_profile)
            self.get_logger().info(
                f'Shaped heading profile loaded from {path_map_path} '
                '(use_precomputed_heading_profile=True).'
            )

        self.create_timer(dt, self._control_step)

        self.get_logger().info('MPC controller ready — waiting for a trajectory + car_position.')

    # ------------------------------------------------------------------
    # Subscribers (cache latest state; the timer does the work)
    # ------------------------------------------------------------------

    def _traj_cb(self, msg: PoseArray) -> None:
        if self._static_path is not None:
            # A precomputed path is active — ignore the live planner's output.
            # See mpc_controller_standalone.py's _path_cb for the same guard.
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

    # ------------------------------------------------------------------
    # Control step (fixed 20 Hz)
    # ------------------------------------------------------------------

    def _control_step(self) -> None:
        # Loop-entry timestamp for the cmd_latency_ms telemetry column — see
        # telemetry_logger's latency-diagnostics note.
        _t_loop0 = time.perf_counter()
        # No pose or no path yet, or the path has gone stale: publish nothing and
        # reset the MPC so it doesn't warm-start from a stale trajectory.  The
        # downstream fsds_bridge brakes on its own cmd_vel timeout.
        if self._static_path is not None:
            # No topic backing this path, so "staleness" doesn't apply — see
            # mpc_controller_standalone.py's Phase 2 for the same reasoning.
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
            return

        # Slice from the car's nearest point, gate on tracking error, and
        # rate-limit rises — identical to mpc_controller_standalone.py's
        # Phase 3; see that file and control_utils.tracking_error_speed_gate
        # for the rationale behind each. This node forwards the result
        # through cmd_vel to fsds_bridge's speed loop rather than using the
        # MPC's own accel, but the speed TARGET must be derived the same way
        # in both nodes or they diverge in exactly the regime that matters.
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
        # tick-to-tick change is bounded — see that constant's comment and
        # mpc_controller_standalone.py's identical logic.
        tel = self._mpc.last_telemetry
        raw_gate = tracking_error_speed_gate(tel.get('e_y', 0.0), tel.get('e_psi', 0.0))
        if self._gate_prev is not None:
            max_step = GATE_RATE_LIMIT / CONTROL_HZ
            raw_gate = float(np.clip(raw_gate, self._gate_prev - max_step, self._gate_prev + max_step))
        self._gate_prev = raw_gate
        gate = raw_gate
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

        # The MPC returns FSDS-normalised (steering, throttle, brake); we take its
        # pre-normalisation steering angle in radians (last_telemetry['delta_cmd'],
        # +ve = left, already clamped to the 25deg limit) and forward the
        # curvature-limited speed, keeping the cmd_vel abstraction intact.
        self._mpc.compute(
            self._path, self._car_pos, self._car_yaw,
            self._car_speed, desired_speed, self._car_yaw_rate,
            pose_age_s=pose_age_s, car_vy=self._car_vy,
        )
        steering = float(self._mpc.last_telemetry.get('delta_cmd', 0.0))

        # Low-pass the steering command across ticks (matches the Stanley node) so
        # rapid left-right jitter never reaches the servo.  1.0 disables.
        if self._delta_filt is None or self._steer_lp >= 1.0:
            self._delta_filt = steering
        else:
            self._delta_filt += self._steer_lp * (steering - self._delta_filt)
        steering = self._delta_filt

        # ── Output smoothing (EXPERIMENTAL, default off) ──
        # See mpc_controller_standalone.py's identical block for the full
        # mechanism/reasoning. Applied ON TOP of steer_lp above (which this
        # node already had and mpc_controller_standalone.py does not) --
        # when off (default), steer_lp's existing behaviour is completely
        # unchanged. Stacking both if both are non-trivial compounds lag;
        # this node's steer_lp already provides flat smoothing, so leave
        # output_smoothing_enabled off here unless specifically testing the
        # curvature-aware floor behaviour on this node too.
        if self._output_smoothing_enabled:
            if self._steer_filtered is None:
                self._steer_filtered = steering
            self._steer_filtered += self._output_smoothing_alpha * (steering - self._steer_filtered)
            tel = self._mpc.last_telemetry
            corner_frac = tel.get('corner_frac', 0.0)
            if self._output_smoothing_lookahead_lead_s > 0.0:
                # See mpc_controller_standalone.py's identical block for the
                # full mechanism/reasoning.
                scan_end = max(self._car_speed, 2.0) * self._output_smoothing_lookahead_lead_s
                path_ahead = self._path
                if len(path_ahead) > 2:
                    i_near = int(np.argmin(np.linalg.norm(path_ahead - self._car_pos, axis=1)))
                    if i_near < len(path_ahead) - 2:
                        path_ahead = path_ahead[i_near:]
                kappa_ahead = peak_kappa_ahead(path_ahead, scan_end=scan_end)
                corner_frac_ahead = _corner_factor(kappa_ahead, self._mpc.params.corner_factor_k)
                corner_frac = max(corner_frac, corner_frac_ahead)
                self._mpc.last_telemetry['corner_frac_ahead'] = corner_frac_ahead
            w_smoothed = max(self._output_smoothing_corner_floor, 1.0 - corner_frac)
            fade_ey = 1.0 / (1.0 + self._output_smoothing_k_ey * abs(tel.get('e_y', 0.0)))
            fade_epsi = 1.0 / (1.0 + self._output_smoothing_k_epsi * abs(tel.get('e_psi', 0.0)))
            w_smoothed = max(self._output_smoothing_corner_floor, w_smoothed * fade_ey * fade_epsi)
            steering = (1.0 - w_smoothed) * steering + w_smoothed * self._steer_filtered

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(desired_speed)
        msg.drive.steering_angle = steering
        self.pub_cmd.publish(msg)

        if self._telemetry is not None:
            tel = self._mpc.last_telemetry
            t = self.get_clock().now().nanoseconds * 1e-9
            # steering is already the roadwheel angle in radians here (this
            # node publishes an Ackermann steering_angle, not a normalised
            # FSDS command), so it is both the logged steer and delta_cmd.
            # See mpc_controller_standalone.py's identical logic for why a
            # static path logs 0.0 rather than None.
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


def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._telemetry is not None:
            if node._lap_tracker is not None:
                lap = node._lap_tracker.result(node.get_clock().now().nanoseconds * 1e-9)
                node._telemetry.close(
                    progress=lap['progress'], time_bonus=lap['time_bonus'],
                    reached_end=lap['reached_end'], lap_time_s=lap['lap_time_s'],
                    optimal_time_s=lap['optimal_time_s'],
                )
            else:
                node._telemetry.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
