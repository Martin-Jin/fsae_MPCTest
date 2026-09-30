"""
fsae_control/mpc/control_step.py — `_ControlStepMixin`: the per-tick control step

`_control_step` is the 20 Hz timer body: pose staleness, speed target and gates,
MPC/NMPC solve, command publication and logging. `_check_cone_proximity` is the
standalone-output cone brake. Split from mpc_controller.py only to keep files
readable; `MPCControllerNode` inherits this mixin so `self` state is unchanged.
"""

import time

import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from fs_msgs.msg import ControlCommand
from geometry_msgs.msg import Pose, PoseArray
from rclpy.time import Time

from fsae_control.control_utils import (
    curvature_speed,
    dynamic_speed_cap,
    precomputed_speed_at,
    tracking_error_speed_gate,
)
from fsae_control.lmpc import MAX_STEER_RAD
from fsae_control.mpc.node_constants import (
    CONTROL_HZ,
    CONE_BRAKE_DIST,
    CONE_BRAKE_WIDTH,
    CONE_RESET_THRESHOLD,
    PATH_TIMEOUT,
    SPEED_TARGET_RISE_RATE,
    V_CURV_FALL_RATE,
    GATE_RATE_LIMIT,
)


class _ControlStepMixin:
    """Per-tick control step for MPCControllerNode."""

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
            self._v_curv_prev = None  # ditto for the live curvature_speed() fall limiter
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

            # curvature_speed() has no memory of its own last output and the
            # live path is re-fit every tick, so a single noisy sample can
            # swing v_curv down (never up, in this direction rises are what
            # the corner needs) far faster than any real corner's own
            # braking-distance curve would ask for -- see V_CURV_FALL_RATE's
            # own comment. The precomputed-track oracle branch above does not
            # need this: it is not re-derived from a noisy live path.
            if self._v_curv_prev is not None:
                max_fall = V_CURV_FALL_RATE / CONTROL_HZ
                v_curv = max(v_curv, self._v_curv_prev - max_fall)
            self._v_curv_prev = v_curv

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
        # Stop ramping once the target has run this far ahead of the car; see
        # params.speed_target_deficit_max (mpc_params.py). Never DROPS the
        # target (max against the previous value), so a car that is merely
        # slow does not get the target dragged down to meet it, and a genuine
        # brake request still passes through the min() above untouched.
        deficit_max = self._mpc.params.speed_target_deficit_max
        if desired_speed - self._car_speed > deficit_max:
            desired_speed = min(desired_speed,
                                max(self._v_des_prev,
                                    self._car_speed + deficit_max))
        self._v_des_prev = desired_speed

        # Age of the pose the MPC is about to solve against — how long ago it
        # was actually measured, not how long ago the callback fired. Lets
        # MPCController compensate for the real, unknown/time-varying delay
        # instead of assuming the state is fresh (see lmpc/controller.py compute()).
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
        pred_xy = self._mpc.last_telemetry.get('nmpc_pred_xy')
        if pred_xy is not None:
            pred_x, pred_y = pred_xy
            pose_array = PoseArray()
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.header.frame_id = 'map'
            for x, y in zip(pred_x, pred_y):
                pose = Pose()
                pose.position.x, pose.position.y = float(x), float(y)
                pose_array.poses.append(pose)
            self.pub_nmpc_pred_path.publish(pose_array)

        self._publish_debug_weights(self._mpc.last_telemetry)

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
                    self._v_curv_prev = None
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
            # be scaled back by MAX_STEER_RAD (and un-negated — lmpc.controller
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
                # float (see fsae_MPCTest's telemetry/control_logger.py mirror,
                # which must match this convention).
                if self._static_path is not None:
                    path_age_s = 0.0
                elif self._path_stamp is not None:
                    path_age_s = (self.get_clock().now() - self._path_stamp).nanoseconds * 1e-9
                else:
                    path_age_s = None
                pred_err_m, pred_acc_pct, lap_summary = self._process_lap_and_horizon(t, tel)
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
                    adaptive=tel,
                    lap_idx=self._lap_tracker.lap_idx if self._lap_tracker is not None else None,
                    pred_err_m=pred_err_m, pred_acc_pct=pred_acc_pct,
                    lap_summary=lap_summary)
                self._telemetry.log_path(t, self._path)

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
                pred_err_m, pred_acc_pct, lap_summary = self._process_lap_and_horizon(t, tel)
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
                    adaptive=tel,
                    lap_idx=self._lap_tracker.lap_idx if self._lap_tracker is not None else None,
                    pred_err_m=pred_err_m, pred_acc_pct=pred_acc_pct,
                    lap_summary=lap_summary)
                self._telemetry.log_path(t, self._path)

            self.get_logger().info(
                f'cmd_vel: speed={desired_speed:.2f} m/s  steer={steering:.3f} rad  '
                f'v_actual={self._car_speed:.2f} m/s  '
                f'e_y={self._mpc.last_telemetry.get("e_y", 0.0):.2f}',
                throttle_duration_sec=1.0,
            )
