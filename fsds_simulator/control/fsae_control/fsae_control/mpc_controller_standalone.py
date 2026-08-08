"""
MPC path-tracking controller — standalone variant (bypasses fsds_bridge).

mpc_controller.py (this package's default `mpc` controller) forwards only the
MPC's steering command through the shared cmd_vel abstraction; fsds_bridge.py
computes throttle/brake from a simple speed-error P-loop against a
curvature-limited target, the same way it does for the Stanley controller.

This node is a DIFFERENT integration of the same MPCController
(mpc_core.MPCController): it publishes fs_msgs/ControlCommand directly, using
the MPC's own (steering, throttle, brake) output unchanged. That preserves the
offline-tuned longitudinal behaviour from the fsae_MPCTest repo's
tuner/offline_tuner.py and gui/simulation.py, which both drive the vehicle
plant with the MPC's own commanded acceleration (see that repo's
sim/rollout_core.py) — mpc_controller.py's accel-discarding design does not.

Ported from fsae_MPCTest's control_node.py (mirrored at
fsds_simulator/control/fsae_control/fsae_control/mpc_controller_standalone.py
in that repo, alongside its own copies of mpc_core.py/control_utils.py — that
directory is a staging area mirroring this package's own hierarchy, kept for
porting purposes only), which implements the same design against an older
ROS 2 topic/message interface; this is that same node updated for the
current fsae_planning topics/messages.

    in   /fsae/planning/selected_trajectory  geometry_msgs/PoseArray        planner centreline
    in   /fsae/slam/car_position             geometry_msgs/PoseStamped      x,y in position; yaw in orientation.w
    in   /fsae/slam/car_odom                 nav_msgs/Odometry              speed + yaw-rate feedback; SAME
                                                                             snapshot as car_position above
                                                                             (both from sim_perception.py's
                                                                             one _odom_cb per tick — see that
                                                                             node's "Speed/yaw-rate
                                                                             synchronisation" docstring note.
                                                                             Do NOT subscribe to the raw
                                                                             /fsds/testing_only/odom directly:
                                                                             that was the bug this fixed.)
    in   /fsds/signal/go                     fs_msgs/GoSignal               race start
    in   /fsae/perception/cone_detection     fsae_interfaces/ConeDetection  proximity e-brake (car-local frame)
    out  /fsds/control_command                fs_msgs/ControlCommand

Because this node owns GO-gating, stale-command braking, and cone-proximity
braking itself (mirroring fsds_bridge.py's own logic against the same inputs),
do NOT launch fsds_bridge.py alongside this node — its output would be
published but never used, and it would race this node for
/fsds/control_command. Select this controller with `controller:=mpc_standalone`
in control.launch.py, which skips fsds_bridge for that mode.

CONTROL LOOP PHASES (see _control_loop)
----------------------------------------------------------------------------
  Phase 1 — Hold at start line until GO signal received.
  Phase 2 — Emergency brake if the planner path is missing/stale (>TARGET_TIMEOUT
            old) or has fewer than 2 points, or the SLAM pose hasn't arrived yet;
            also resets the MPC so it doesn't warm-start from a stale trajectory
            once the path returns. When path_map_path is set, the path can
            never be "stale" (see that param's declaration) — only the pose
            check still applies.
  Phase 3 — Normal MPC solve via MPCController.compute().
  Phase 4 — Cone-proximity brake override: hard-overrides the MPC's
            throttle/brake (not steering) if a cone is inside the dynamic
            braking corridor. After CONE_RESET_THRESHOLD seconds of
            continuous braking, the MPC is reset exactly once (edge-triggered
            on the rising duration threshold, re-armed once the brake clears).
  Phase 4a — Telemetry logging of the *final* (post-override) command.
  Phase 5 — Publish.
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from fs_msgs.msg import ControlCommand, GoSignal
from fsae_interfaces.msg import ConeDetection
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.time import Time

from fsae_control.control_utils import (
    curvature_speed, load_path_profile_csv, load_speed_profile_csv,
    precomputed_speed_at, tracking_error_speed_gate,
)
from fsae_control.mpc_core import MAX_STEER_RAD, MPCController
from fsae_control.telemetry_logger import ControlLogger

CONTROL_HZ = 20.0   # must match MPCController(dt=0.05); dt = 1 / CONTROL_HZ

# CONE_BRAKE_DIST is also the ceiling on the dynamic corridor computed in
# _check_cone_proximity() (car_speed * 0.25, clipped to [0.6, CONE_BRAKE_DIST]).
CONE_BRAKE_DIST      = 2.0    # m — forward corridor depth for cone proximity brake
CONE_BRAKE_WIDTH     = 0.18   # m — lateral half-width of braking corridor (36 cm total)
CONE_RESET_THRESHOLD = 0.3    # s — continuous cone-brake duration before one MPC reset
PATH_TIMEOUT         = 0.5    # s — reset the MPC if no fresh trajectory within this window

# Max rate (m/s^2) at which the SPEED TARGET may rise. Decreases are never
# rate-limited — slowing down is always safe, and delaying a genuine brake
# request is exactly the failure this is meant to prevent. Sized just under the
# car's real acceleration capability so it never becomes the binding limit on a
# true straight; it only suppresses planner-induced target jitter.
SPEED_TARGET_RISE_RATE = 2.0

# Max rate (gate-units/s, gate in [floor, 1.0]) at which
# tracking_error_speed_gate()'s output may change per tick, in EITHER
# direction. Without this, a fast-growing e_y sweeping through the gate's
# active band (ey_lo=0.5 to ey_hi=2.0) can compound with a simultaneously
# falling curvature-based speed target into a sharp single-tick v_desired
# drop that bypasses SPEED_TARGET_RISE_RATE (that limiter only bounds RISES)
# and produces erratic a_cmd right after (measured: mpc_standalone_control_
# 1786178292.csv, t=53.28-53.71s, v_desired 8.20->6.53->4.09 m/s over 3
# ticks, a_cmd swinging -1.06/-0.46/+0.14 then slamming to -7.0). That
# measurement is why the gate was disabled outright for precomputed-speed
# runs (2026-08-08) rather than smoothed — this rate limit is the smoothed
# alternative: it spreads the same total slowdown over ~0.35s (a full
# 1.0->0.3 span at this rate) instead of one tick, keeping the safety
# response (the car DOES still slow down when tracking badly) while
# removing the single-tick cliff that caused the erratic a_cmd. Sized to
# the same order of magnitude as SPEED_TARGET_RISE_RATE by design choice,
# not measurement — re-tune against a live/offline A-B run if it turns out
# to still cost real safety margin (too slow) or still chatter (too fast).
GATE_RATE_LIMIT = 2.0


class MPCControllerStandaloneNode(Node):
    def __init__(self):
        super().__init__('controller')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('v_max', 20.0),      # m/s — top speed on straights
                ('v_min', 1.5),       # m/s — minimum speed through tight corners
                ('log_csv', False),   # write CSV telemetry to log_dir
                ('log_dir', ''),      # '' -> ~/fsae_logs
                ('map_path', ''),     # '' -> live curvature_speed() (default);
                                       # else a fsae_MPCTest tuner/export_speed_profile.py
                                       # CSV to use instead — see USE_PRECOMPUTED_SPEED_PROFILE
                                       # in fsae_MPCTest/settings.py and S48 in
                                       # sim_to_real_investigation.md for why.
                ('path_map_path', ''),  # '' -> live /fsae/planning/selected_trajectory
                                       # (default); else the SAME kind of CSV as map_path,
                                       # used for the tracked PATH instead of just speed —
                                       # see USE_PLANNER=False in fsae_MPCTest/settings.py (the
                                       # offline equivalent -- no separate flag exists there).
                                       # Removes centerline_planner.py from the control loop
                                       # entirely, to isolate controller/plant tracking error
                                       # from planner-induced path error.
            ],
        )
        self._v_max = self.get_parameter('v_max').get_parameter_value().double_value
        self._v_min = self.get_parameter('v_min').get_parameter_value().double_value

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
        # startup; self._path_pts is populated from this immediately and never
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

        self._telemetry = None
        if self.get_parameter('log_csv').get_parameter_value().bool_value:
            log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
            self._telemetry = ControlLogger('mpc_standalone', log_dir=log_dir)
            self.get_logger().info(f'CSV telemetry -> {self._telemetry.paths[0]}')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            PoseArray, '/fsae/planning/selected_trajectory', self._path_cb, 10)
        self.create_subscription(
            PoseStamped, '/fsae/slam/car_position', self._pose_cb, 10)
        # /fsae/slam/car_odom, NOT the raw /fsds/testing_only/odom -- see this
        # file's own docstring "Speed/yaw-rate synchronisation" note and
        # sim_perception.py's identical one. Subscribing to the raw topic
        # directly raced sim_perception's own separate subscription to the
        # same 250 Hz publisher, so car_speed/car_yaw_rate could reflect a
        # different odom instant than car_pos/car_yaw on any given tick.
        self.create_subscription(
            Odometry, '/fsae/slam/car_odom', self._odom_cb, sensor_qos)
        self.create_subscription(
            GoSignal, '/fsds/signal/go', self._go_cb, 10)
        self.create_subscription(
            ConeDetection, '/fsae/perception/cone_detection', self._cone_cb, 10)

        self.pub_cmd = self.create_publisher(ControlCommand, '/fsds/control_command', 10)

        self._go_received = False
        self._path_pts: np.ndarray = (
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
        self._desired_speed = 0.0
        # Previous tick's speed target, for the rise-rate limiter. None = no
        # history yet (first tick after start or after a reset), so the first
        # target passes through unlimited rather than ramping up from zero.
        self._v_des_prev: float | None = None
        # Previous tick's tracking-error speed gate, for GATE_RATE_LIMIT below.
        # None = no history yet, so the first gate value passes through
        # unlimited (nothing to ramp from).
        self._gate_prev: float | None = None

        # Cones arrive already in the car-LOCAL frame (see ConeDetection.msg /
        # fsae_sim_perception's sim_perception.py), so no car_pos/car_yaw
        # transform is needed for the proximity check below.
        self._cones_local: np.ndarray = np.empty((0, 2))
        self._cone_brake_duration = 0.0
        self._cone_reset_done = False

        dt = 1.0 / CONTROL_HZ
        self._mpc = MPCController(dt=dt, N=35)

        self.create_timer(dt, self._control_loop)
        self.get_logger().info(
            'MPC standalone controller ready — publishing ControlCommand directly '
            '(fsds_bridge is NOT used in this mode).'
        )

    # ------------------------------------------------------------------
    # Subscribers (cache latest state; the timer does the work)
    # ------------------------------------------------------------------

    def _go_cb(self, msg: GoSignal) -> None:
        if not self._go_received:
            self._go_received = True
            self.get_logger().info('GO signal received.')

    def _path_cb(self, msg: PoseArray) -> None:
        if self._static_path is not None:
            # A precomputed path is active — centerline_planner.py's output is
            # ignored entirely (still subscribed so the topic doesn't dangle,
            # but never written to self._path_pts). See path_map_path above.
            return
        self._path_pts = np.array(
            [[p.position.x, p.position.y] for p in msg.poses], dtype=np.float64
        ) if msg.poses else np.empty((0, 2))
        self._path_stamp = self.get_clock().now()

    def _pose_cb(self, msg: PoseStamped) -> None:
        # x,y in position; yaw (rad) is stuffed into orientation.w (upstream convention).
        self._car_pos = np.array([msg.pose.position.x, msg.pose.position.y])
        self._car_yaw = float(msg.pose.orientation.w)
        self._pose_stamp = msg.header.stamp
        self._have_pose = True

    def _odom_cb(self, msg: Odometry) -> None:
        # msg is /fsae/slam/car_odom -- see this class's subscription comment.
        # v.x/v.y are body-frame (sim_perception.py relays them unrotated
        # from the bridge's already-body-frame odom) -- keep both instead of
        # collapsing to hypot(), which silently drops the vy*cos(e_psi) term
        # _error_state needs (see mpc_core.py's e_yd comment).
        v = msg.twist.twist.linear
        self._car_speed = float(v.x)
        self._car_vy = float(v.y)
        self._car_yaw_rate = float(msg.twist.twist.angular.z)

    def _cone_cb(self, msg: ConeDetection) -> None:
        pts = [[p.x, p.y] for p in msg.blue] + [[p.x, p.y] for p in msg.yellow]
        self._cones_local = np.array(pts, dtype=np.float64) if pts else np.empty((0, 2))

    # ------------------------------------------------------------------
    # Helpers
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

    def _publish(self, cmd: ControlCommand) -> None:
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.pub_cmd.publish(cmd)

    # ------------------------------------------------------------------
    # Control step (fixed 20 Hz)
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        # Loop-entry timestamp for the cmd_latency_ms telemetry column: how
        # long this tick took from entering the callback to publishing a
        # command. Distinguishes "our compute is slow" from "our inputs were
        # already stale when we got them" (pose_age_s / path_age_s).
        _t_loop0 = time.perf_counter()
        cmd = ControlCommand()

        # ── Phase 1: hold until GO ──────────────────────────────────────
        if not self._go_received:
            cmd.throttle, cmd.steering, cmd.brake = 0.0, 0.0, 1.0
            self._publish(cmd)
            self.get_logger().info('Waiting for GO signal...', throttle_duration_sec=2.0)
            return

        # ── Phase 2: emergency brake on stale/missing path or pose ──────
        # A static precomputed path has no topic to go stale — its
        # "freshness" is meaningless, so the only thing that can fail here is
        # the live pose (still checked below via self._have_pose), exactly
        # the safety net path_map_path's docstring promises: a car running
        # with a precomputed path still brakes correctly if its live
        # localisation fails, same as before.
        if self._static_path is not None:
            path_stale = False
        else:
            path_stale = (
                self._path_stamp is None
                or (self.get_clock().now() - self._path_stamp).nanoseconds * 1e-9 > PATH_TIMEOUT
            )
        if not self._have_pose or path_stale or len(self._path_pts) < 2:
            cmd.throttle, cmd.steering, cmd.brake = 0.0, 0.0, 1.0
            self._mpc.reset()
            self._v_des_prev = None   # don't ramp from a pre-fail-safe target
            self._gate_prev = None    # ditto for the tracking-error speed gate
            self._publish(cmd)
            self.get_logger().warn(
                'Trajectory path lost or stale — emergency braking.', throttle_duration_sec=1.0)
            return

        # ── Phase 3: MPC solve ───────────────────────────────────────────
        # Slice the path from the car's nearest point before measuring
        # curvature. curvature_speed() documents "waypoints[0] is assumed to be
        # the car's current position", and its short-path cap
        # (v_max * total/scan_end) is measured from waypoints[0] too — so
        # passing the whole path silently mis-measures both whenever the car is
        # past the path's first point. sim/rollout_core.py already sliced
        # (cl[cl_idx:]); this brings the live node into parity with it.
        if self._speed_profile is not None:
            # Track is already fully mapped (map_path param set) — look up
            # the oracle speed target instead of re-deriving it from the
            # live-built centreline. See load_speed_profile_csv()'s docstring
            # and sim_to_real_investigation.md S48 for why.
            path_X, path_Y, path_V = self._speed_profile
            v_curv = precomputed_speed_at(self._car_pos, path_X, path_Y, path_V)
        else:
            path_ahead = self._path_pts
            if len(path_ahead) > 2:
                i_near = int(np.argmin(np.linalg.norm(path_ahead - self._car_pos, axis=1)))
                if i_near < len(path_ahead) - 2:
                    path_ahead = path_ahead[i_near:]

            v_curv = curvature_speed(path_ahead, v_max=self._v_max, v_min=self._v_min)

        # Scale the target down when we're failing to track. Uses the PREVIOUS
        # tick's errors (this tick's aren't known until the MPC solves), which
        # is one 50 ms step of lag — negligible next to the error timescales
        # this responds to. See control_utils.tracking_error_speed_gate.
        #
        # Re-enabled for precomputed-speed runs (2026-08-09) via GATE_RATE_LIMIT
        # instead of left disabled: disabling it entirely (2026-08-08) traded
        # away its whole purpose -- with no gate, a precomputed-path run that
        # drifts badly off-line keeps commanding full track speed regardless,
        # observed driving fast while significantly off-track. The original
        # problem was never the gate existing, it was that it applied
        # UNSMOOTHED (unlike the rise-rate limiter below, which only limits
        # increases): a fast-growing e_y sweeping through the gate's active
        # band (ey_lo=0.5 to ey_hi=2.0) compounded with a simultaneously-
        # falling v_curv into a sharp v_desired step that bypassed the rise
        # limiter (decreases pass through instantly) and hit the MPC as a
        # sudden target-speed cliff, producing erratic a_cmd right after
        # (mpc_standalone_control_1786178292.csv, t=53.28-53.71s: e_y
        # -0.69->-1.53m over 5 ticks, v_desired 8.20->6.53->4.09, a_cmd
        # swinging -1.06/-0.46/+0.14 then slamming to -7.0 three ticks later).
        # Rate-limiting the gate itself spreads the same total slowdown over
        # several ticks instead of one, keeping the safety response while
        # removing the cliff — see GATE_RATE_LIMIT's own comment.
        tel = self._mpc.last_telemetry
        raw_gate = tracking_error_speed_gate(tel.get('e_y', 0.0), tel.get('e_psi', 0.0))
        if self._gate_prev is not None:
            max_step = GATE_RATE_LIMIT / CONTROL_HZ
            raw_gate = float(np.clip(raw_gate, self._gate_prev - max_step, self._gate_prev + max_step))
        self._gate_prev = raw_gate
        gate = raw_gate
        # Never gate below v_min: the car still needs authority to steer back.
        self._desired_speed = max(self._v_min, v_curv * gate)

        # Rate-limit INCREASES only. Decreases pass through instantly because
        # slowing down is always the safe direction. The planner's frame-to-
        # frame curvature jitter otherwise swings the target by a measured mean
        # 2.75 m/s (p95 10.7) between snapshots; limiting the rise cuts that to
        # 1.72 (p95 5.4) without capping achievable speed on a real straight.
        if self._v_des_prev is not None:
            max_rise = SPEED_TARGET_RISE_RATE / CONTROL_HZ
            self._desired_speed = min(self._desired_speed, self._v_des_prev + max_rise)
        self._v_des_prev = self._desired_speed

        pose_age_s = (self.get_clock().now() - Time.from_msg(self._pose_stamp)).nanoseconds * 1e-9

        steering, throttle, brake = self._mpc.compute(
            path=self._path_pts,
            car_pos=self._car_pos,
            car_yaw=self._car_yaw,
            car_speed=self._car_speed,
            desired_speed=self._desired_speed,
            car_yaw_rate=self._car_yaw_rate,
            pose_age_s=pose_age_s,
            car_vy=self._car_vy,
        )
        cmd.steering, cmd.throttle, cmd.brake = steering, throttle, brake

        # ── Phase 4: cone-proximity brake override ──────────────────────
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
        # log_control's steer argument is RADIANS of roadwheel angle. cmd.steering
        # is the normalised FSDS [-1, 1] command, so it must be scaled back by
        # MAX_STEER_RAD (and un-negated — mpc_core flips sign for the FSDS
        # convention) before logging. Passing cmd.steering directly made the
        # logger's math.degrees() emit a meaningless number ~2.3x the real angle,
        # which masked the fact that the controller was sitting on its slew limit.
        if self._telemetry is not None:
            tel = self._mpc.last_telemetry
            t = self.get_clock().now().nanoseconds * 1e-9
            steer_rad = -float(cmd.steering) * MAX_STEER_RAD
            # delta_cmd/a_cmd come from the MPC's own telemetry so the logged
            # score is computed on the solver's real [rad, m/s^2] command pair.
            # They fall back to the published command when a fail-safe (cone
            # brake / no solve) overrode the MPC, so the score reflects what
            # the car actually did, not what the MPC wanted.
            a_cmd = tel.get('a_cmd', 0.0)
            if cmd.brake > 0.0 and cmd.throttle == 0.0:
                a_cmd = min(a_cmd, -float(cmd.brake) * self._mpc.a_max_brake)
            # Age of the planner path this solve consumed. The path arrives at
            # ~1 Hz while this loop runs at 20 Hz, so it is routinely ~20 ticks
            # old; logging it makes that concrete instead of inferred. A static
            # precomputed path (self._static_path set) is logged as exactly
            # 0.0 rather than None — it is never stale by construction, and
            # 0.0 keeps this column numeric for any downstream analysis that
            # assumes path_age_s is always a float (see fsae_MPCTest's
            # telemetry_logger.py mirror, which must match this convention).
            if self._static_path is not None:
                path_age_s = 0.0
            elif self._path_stamp is not None:
                path_age_s = (self.get_clock().now() - self._path_stamp).nanoseconds * 1e-9
            else:
                path_age_s = None
            self._telemetry.log_control(
                t, self._car_pos[0], self._car_pos[1], self._car_yaw,
                self._car_speed, self._desired_speed, steer_rad,
                tel.get('e_y', 0.0), tel.get('e_psi', 0.0), self._car_yaw_rate,
                delta_cmd=steer_rad, a_cmd=a_cmd,
                pose_age_s=tel.get('pose_age_s'),
                path_age_s=path_age_s,
                n_delay=tel.get('n_delay'),
                solve_ms=tel.get('solve_ms'),
                cmd_latency_ms=(time.perf_counter() - _t_loop0) * 1e3)
            self._telemetry.log_path(t, self._path_pts)

        # ── Phase 5: publish ─────────────────────────────────────────────
        self._publish(cmd)

        self.get_logger().info(
            f'MPC thr={cmd.throttle:.2f} brk={cmd.brake:.2f} steer={cmd.steering:.3f} | '
            f'v={self._car_speed:.1f}/{self._desired_speed:.1f} m/s',
            throttle_duration_sec=1.0,
        )

    def destroy_node(self) -> None:
        if self._telemetry is not None:
            self._telemetry.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerStandaloneNode()
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
