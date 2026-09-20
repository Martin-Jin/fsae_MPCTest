"""
Open-loop braking system-ID: what deceleration does FSDS actually deliver for
a given brake command, at a given speed?

Companion to `steering_sysid.py`'s lateral experiment (see
`docs/reference/` -> "MEASURED: the car's yaw response is ~3x weaker than
commanded"), but for the LONGITUDINAL braking side, which has never had an
equivalent measurement (see `docs/logs/brake_sysid_investigation.md` once a
sweep has been run and analysed). This exists because two NMPC speed-profile
attempts were rejected for reasons downstream of an unverified braking
assumption (`docs/logs/nmpc_speed_limit_investigation.md`,
`late_turn_in_investigation.md` Part 16 SS16.7) -- a scan of ~90 real NMPC
control logs found the OPPOSITE of the lateral case (achieved deceleration
usually MEETS OR EXCEEDS commanded a_cmd, only ~11% of sustained braking
events show any shortfall), so before any braking ceiling or model change is
attempted, this node measures the real relationship directly, the same way
`alat_ceiling` was derived from `steering_sysid`/`steering_step` rather than
assumed.

Drives FSDS directly with fixed throttle/brake, bypassing the MPC/Stanley
controller entirely (same reason `steering_sysid` does: two publishers on
/fds/control_command would interleave and corrupt the measurement).

For each (approach_speed, brake_cmd) pair:
  1. ACCELERATE: full throttle until car_speed reaches approach_speed
     (or a timeout, logged as a skipped point -- FSDS's own top speed or
     track geometry may not allow every approach speed).
  2. COAST: brief zero-command settle so the throttle transient decays
     before the brake step (mirrors steering_step's own settle phase).
  3. RECORD: apply brake_cmd at t=0 of this phase and hold it fixed
     (throttle=0, steering=0) for `record_s` seconds or until car_speed
     reaches ~0, whichever comes first.
  4. Between points: brief pause with everything zeroed.

Topics (identical to steering_sysid's, see that file's own docstring):
    in   /fsds/testing_only/odom     nav_msgs/Odometry        speed feedback
    in   /fsds/signal/go             fs_msgs/GoSignal         race start
    out  /fsds/control_command       fs_msgs/ControlCommand

Logs to <repo_root>/fsae_logs/brake_sysid_<epoch>.csv, columns:
    t, phase, approach_speed, brake_cmd, v_actual, a_cmd_equiv

`a_cmd_equiv` converts the raw ControlCommand.brake (0..1, FSDS units) to the
MPC's own m/s^2 units via `nmpc_core.MAX_BRAKE`/`mpc_core.MAX_BRAKE` so the
analysis script can plot directly against `a_cmd`-labelled axes from the
closed-loop logs this investigation started from -- see that constant's own
import for why it must stay in sync rather than a hardcoded local copy.

Usage:
    ros2 run fsae_control brake_sysid
    ros2 run fsae_control brake_sysid --ros-args -p 'approach_speeds:=[4.0, 8.0, 12.0, 16.0]' -p 'brake_cmds:=[0.3, 0.6, 1.0]'
"""
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from fs_msgs.msg import ControlCommand, GoSignal
from nav_msgs.msg import Odometry

from .mpc.mpc_core import MAX_BRAKE

LOOP_HZ = 20.0
ACCEL_TIMEOUT_S = 15.0   # give up reaching approach_speed after this long
SETTLE_S = 1.0           # zero-command coast before the brake step
RECORD_S = 6.0           # max recording window per point
PAUSE_S = 2.0            # zero-command gap between points
STOP_SPEED = 0.3         # m/s -- treat as stopped, end RECORD early


class BrakeSysID(Node):
    def __init__(self):
        super().__init__('brake_sysid')

        self.declare_parameter('approach_speeds', [4.0, 8.0, 11.0, 14.0, 16.5])
        self.declare_parameter('brake_cmds', [0.3, 0.5, 0.7, 1.0])
        # Caller-supplied repo root (run_brake_sysid.sh already computes this
        # correctly from its own script path) -- NOT derived from __file__
        # here, since colcon copies this module into install/ at a different
        # relative depth than its source location, making any fixed count of
        # os.path.dirname() calls silently wrong depending on which copy is
        # actually running. Empty default falls back to '~/fsae_logs' only
        # for a bare `ros2 run` with no launcher, same as telemetry_logger.py.
        self.declare_parameter('repo_root', '')

        self.approach_speeds = list(self.get_parameter('approach_speeds').value)
        self.brake_cmds = list(self.get_parameter('brake_cmds').value)
        repo_root = self.get_parameter('repo_root').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Odometry, '/fsds/testing_only/odom', self._odom_cb, sensor_qos)
        self.create_subscription(GoSignal, '/fsds/signal/go', self._go_cb, 10)
        self.pub_cmd = self.create_publisher(ControlCommand, '/fsds/control_command', 10)

        self._go_received = False
        self._car_speed = 0.0

        log_dir = (os.path.join(repo_root, 'fsae_logs') if repo_root
                   else os.path.expanduser('~/fsae_logs'))
        os.makedirs(log_dir, exist_ok=True)
        self._log_path = os.path.join(log_dir, f'brake_sysid_{int(time.time())}.csv')
        self._log_file = open(self._log_path, 'w', newline='')
        self._writer = csv.writer(self._log_file)
        self._writer.writerow(
            ['t', 'phase', 'approach_speed', 'brake_cmd', 'v_actual', 'a_cmd_equiv'])
        self.get_logger().info(f'Logging to {self._log_path}')

        self._t0 = None
        self._points = [(v, b) for v in self.approach_speeds for b in self.brake_cmds]
        self._point_idx = 0
        self._phase = 'wait_go'
        self._phase_t0 = None
        self._cur_v, self._cur_b = None, None

        self.create_timer(1.0 / LOOP_HZ, self._tick)

    def _odom_cb(self, msg: Odometry) -> None:
        v = msg.twist.twist.linear
        self._car_speed = float(math.hypot(v.x, v.y))

    def _go_cb(self, msg: GoSignal) -> None:
        if not self._go_received:
            self._go_received = True
            self.get_logger().info('GO received, starting sweep.')

    def _publish(self, throttle=0.0, steering=0.0, brake=0.0):
        cmd = ControlCommand()
        cmd.throttle, cmd.steering, cmd.brake = throttle, steering, brake
        self.pub_cmd.publish(cmd)

    def _log(self, phase, v_target, b_cmd):
        t = time.perf_counter() - self._t0
        a_equiv = -b_cmd * MAX_BRAKE
        self._writer.writerow([f'{t:.3f}', phase, v_target, b_cmd,
                               f'{self._car_speed:.3f}', f'{a_equiv:.3f}'])

    def _tick(self) -> None:
        if self._t0 is None:
            self._t0 = time.perf_counter()

        if self._phase == 'wait_go':
            self._publish()
            if self._go_received:
                self._advance_point()
            return

        if self._phase == 'done':
            self._publish()
            return

        v_target, b_cmd = self._cur_v, self._cur_b
        now = time.perf_counter()
        elapsed = now - self._phase_t0

        if self._phase == 'accel':
            self._publish(throttle=1.0)
            self._log('accel', v_target, b_cmd)
            if self._car_speed >= v_target:
                self._enter_phase('settle')
            elif elapsed > ACCEL_TIMEOUT_S:
                self.get_logger().warn(
                    f'Timed out reaching {v_target} m/s (stuck at '
                    f'{self._car_speed:.1f}); skipping this point.')
                self._log('skipped', v_target, b_cmd)
                self._advance_point()
            return

        if self._phase == 'settle':
            self._publish()
            self._log('settle', v_target, b_cmd)
            if elapsed > SETTLE_S:
                self._enter_phase('record')
            return

        if self._phase == 'record':
            self._publish(brake=b_cmd)
            self._log('record', v_target, b_cmd)
            if elapsed > RECORD_S or self._car_speed < STOP_SPEED:
                self._enter_phase('pause')
            return

        if self._phase == 'pause':
            self._publish()
            self._log('pause', v_target, b_cmd)
            if elapsed > PAUSE_S:
                self._advance_point()
            return

    def _enter_phase(self, phase):
        self._phase = phase
        self._phase_t0 = time.perf_counter()

    def _advance_point(self):
        if self._point_idx >= len(self._points):
            self._phase = 'done'
            self.get_logger().info('Sweep complete.')
            self._log_file.close()
            return
        self._cur_v, self._cur_b = self._points[self._point_idx]
        self._point_idx += 1
        self.get_logger().info(
            f'[{self._point_idx}/{len(self._points)}] approach_speed='
            f'{self._cur_v} brake_cmd={self._cur_b}')
        self._enter_phase('accel')

    def destroy_node(self):
        if not self._log_file.closed:
            self._log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BrakeSysID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
