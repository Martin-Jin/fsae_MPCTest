"""Step-input test: identify WHICH mechanism caps FSDS's yaw rate.

The open-loop sweep (`steering_sysid.py`) established THAT yaw is capped above
~6 m/s at ~7 m/s2 equivalent lateral acceleration, well below the 12.3 m/s2 the
same car reaches on a lap.  See
`fsae_MPCTest/docs/planning_control_sync.md` -> "RESULT: FSDS caps yaw rate
above ~6 m/s".

Three mechanisms fit that steady-state data about equally well:

  A. HARD YAW-RATE LIMIT      yaw is clipped at r_max, full stop.
  B. SPEED-SCALED AUTHORITY   effective steering angle is scaled by f(v).
  C. ACTIVE DAMPING           a restoring torque opposes yaw once it builds.

They are indistinguishable in steady state but differ sharply in the TRANSIENT
after a sudden steering input:

  A -> yaw rises and CLIPS.  It never exceeds r_max, and the approach is a
       hard corner, not a curve.  No overshoot, ever.
  B -> yaw rises smoothly to a lower plateau, exactly like a smaller steering
       angle.  First-order shape, no overshoot.
  C -> yaw OVERSHOOTS the final value, then decays back as the damping term
       builds.  Overshoot is the fingerprint; A and B cannot produce it.

Protocol: settle straight at the target speed with zero steering, then apply a
step and hold, sampling at full rate through the transient.  Steering starts
from EXACTLY zero (the sweep did not guarantee this -- the previous point's
angle was often still applied), so the transient is clean.

Preliminary evidence from the sweep's HOLD_STEER windows already shows 25-75%
overshoot, which points at C.  But those were not clean steps from zero, so
this test exists to confirm it properly.

SAFETY: same as steering_sysid -- empty map, nothing else publishing to
/fsds/control_command.  Geofenced.

    ros2 run fsae_control steering_step
    ros2 run fsae_control steering_step --ros-args -p speeds:="[8.0, 12.0]"

Writes ~/fsae_logs/steering_step_<ts>.csv; analyse with
`fsae_MPCTest/tuner/steering_step_analysis.py`.
"""
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from fs_msgs.msg import ControlCommand, GoSignal
from nav_msgs.msg import Odometry

MAX_STEER_RAD = math.radians(25.0)

KP_BRAKE = 0.40
KP_THROTTLE_SYSID = 0.25
KI_THROTTLE = 0.30
I_THROTTLE_MAX = 0.85
LAUNCH_SPEED = 1.0
LAUNCH_THROTTLE = 0.55

LOG_DIR = os.path.expanduser('~/fsae_logs')


class Phase:
    SETTLE = 'settle'     # straight line, zero steering, reach target speed
    STEP = 'step'         # steering applied, sampling the transient
    RELAX = 'relax'       # steering released, let the car straighten


class SteeringStep(Node):
    def __init__(self):
        super().__init__('steering_step')

        # Speeds bracket the ~6 m/s threshold so the transient can be compared
        # below it (where the sweep showed s ~ 1.0) and above it (s ~ 0.3).
        self.declare_parameter('speeds', [4.0, 8.0, 12.0])
        self.declare_parameter('steer_cmds', [0.6, 1.0])
        self.declare_parameter('settle_s', 4.0)
        self.declare_parameter('step_s', 3.0)      # must span the transient
        self.declare_parameter('relax_s', 3.0)
        self.declare_parameter('repeats', 2)       # average out sim noise
        self.declare_parameter('require_go', True)
        self.declare_parameter('max_radius', 70.0)

        self.speeds = list(self.get_parameter('speeds').value)
        self.steer_cmds = list(self.get_parameter('steer_cmds').value)
        self.settle_s = float(self.get_parameter('settle_s').value)
        self.step_s = float(self.get_parameter('step_s').value)
        self.relax_s = float(self.get_parameter('relax_s').value)
        self.repeats = int(self.get_parameter('repeats').value)
        self._go_received = not bool(self.get_parameter('require_go').value)
        self.max_radius = float(self.get_parameter('max_radius').value)

        # Alternate direction between repeats: exposes any left/right bias and
        # keeps the car from spiralling one way across the whole run.
        self.plan = []
        for v in self.speeds:
            for sc in self.steer_cmds:
                for k in range(self.repeats):
                    self.plan.append((float(v), float(sc) * (1.0 if k % 2 == 0 else -1.0)))

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, '/fsds/testing_only/odom',
                                 self._odom_cb, sensor_qos)
        self.create_subscription(GoSignal, '/fsds/signal/go', self._go_cb, 10)
        self.pub_cmd = self.create_publisher(ControlCommand,
                                             '/fsds/control_command', 10)

        self._speed = self._yaw_rate = 0.0
        self._x = self._y = self._yaw = 0.0
        self._have_odom = False
        self._i_term = 0.0
        self._home = None
        self._idx = 0
        self._phase = Phase.SETTLE
        self._t0 = time.time()
        self._phase_t0 = self._t0
        self._step_t0 = None

        os.makedirs(LOG_DIR, exist_ok=True)
        self._path = os.path.join(LOG_DIR, f'steering_step_{int(self._t0)}.csv')
        self._fh = open(self._path, 'w')
        self._fh.write(
            '# fsae open-loop STEP-INPUT test\n'
            f'# t0_epoch_s={self._t0:.4f}\n'
            '# identifies hard-limit vs speed-scaled-authority vs active damping\n'
            '# t_step is seconds since the step was applied (blank outside the step)\n'
        )
        self._fh.write('t,t_step,phase,trial,target_v,steer_norm,delta_cmd_rad,'
                       'throttle,brake,v_actual,yaw_rate,car_x,car_y,car_yaw\n')
        self._fh.flush()

        # 50 Hz: the transient is the measurement here, and 20 Hz would smear
        # a rise that completes in a few hundred milliseconds.
        self.create_timer(0.02, self._loop)
        self.get_logger().info(
            f'steering_step: {len(self.plan)} trials -> {self._path}')

    # ------------------------------------------------------------------

    def _go_cb(self, msg):
        if not self._go_received:
            self._go_received = True
            self.get_logger().info('GO received.')

    def _odom_cb(self, msg):
        lv = msg.twist.twist.linear
        self._speed = float(math.hypot(lv.x, lv.y))
        self._yaw_rate = float(msg.twist.twist.angular.z)
        p = msg.pose.pose.position
        self._x, self._y = float(p.x), float(p.y)
        q = msg.pose.pose.orientation
        self._yaw = float(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        if self._home is None:
            self._home = (self._x, self._y)
        self._have_odom = True

    def _dist_home(self):
        if self._home is None:
            return 0.0
        return float(math.hypot(self._x - self._home[0],
                                self._y - self._home[1]))

    def _steer_home(self):
        if self._home is None:
            return 0.0
        bearing = math.atan2(self._home[1] - self._y, self._home[0] - self._x)
        err = math.atan2(math.sin(bearing - self._yaw),
                         math.cos(bearing - self._yaw))
        return float(np.clip(-2.0 * err / MAX_STEER_RAD, -1.0, 1.0))

    def _throttle_brake(self, target_v, dt=0.02):
        err = target_v - self._speed
        if err < -0.5:
            self._i_term *= 0.9
            return 0.0, min(1.0, KP_BRAKE * (-err))
        self._i_term = float(np.clip(self._i_term + KI_THROTTLE * err * dt,
                                     0.0, I_THROTTLE_MAX))
        thr = KP_THROTTLE_SYSID * err + self._i_term
        if self._speed < LAUNCH_SPEED and err > 0.0:
            thr = max(thr, LAUNCH_THROTTLE)
        return float(np.clip(thr, 0.0, 1.0)), 0.0

    def _advance(self, phase):
        self._phase = phase
        self._phase_t0 = time.time()

    def _loop(self):
        if not self._go_received or not self._have_odom:
            self.pub_cmd.publish(ControlCommand())
            return

        if self._idx >= len(self.plan):
            cmd = ControlCommand()
            cmd.throttle, cmd.brake, cmd.steering = 0.0, 1.0, 0.0
            self.pub_cmd.publish(cmd)
            return

        target_v, steer_norm = self.plan[self._idx]
        elapsed = time.time() - self._phase_t0
        t_step = ''

        if self._phase == Phase.SETTLE:
            # Zero steering: the step must start from a genuinely straight car,
            # otherwise the "transient" is contaminated by the previous trial.
            applied = 0.0
            if self._dist_home() > self.max_radius:
                applied = self._steer_home()
            straight = abs(self._yaw_rate) < 0.05
            if (abs(self._speed - target_v) < 0.3 and straight
                    and elapsed > 1.5) or elapsed > self.settle_s + 10.0:
                self._step_t0 = time.time()
                self._advance(Phase.STEP)
                self.get_logger().info(
                    f'trial {self._idx + 1}/{len(self.plan)}: STEP '
                    f'v={self._speed:.1f} steer={steer_norm:+.2f}')
        elif self._phase == Phase.STEP:
            applied = steer_norm
            t_step = f'{time.time() - self._step_t0:.4f}'
            if elapsed > self.step_s:
                self._advance(Phase.RELAX)
        else:  # RELAX
            applied = self._steer_home() if self._dist_home() > 20.0 else 0.0
            target_v = min(target_v, 6.0)
            if elapsed > self.relax_s:
                self._idx += 1
                self._i_term = 0.0
                self._advance(Phase.SETTLE)
                if self._idx >= len(self.plan):
                    self.get_logger().info('All trials complete — braking.')

        throttle, brake = self._throttle_brake(target_v)
        cmd = ControlCommand()
        cmd.throttle, cmd.brake = float(throttle), float(brake)
        cmd.steering = float(np.clip(applied, -1.0, 1.0))
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.pub_cmd.publish(cmd)

        self._fh.write(
            f'{time.time() - self._t0:.4f},{t_step},{self._phase},{self._idx},'
            f'{target_v:.3f},{cmd.steering:.5f},'
            f'{-cmd.steering * MAX_STEER_RAD:.6f},{cmd.throttle:.4f},'
            f'{cmd.brake:.4f},{self._speed:.4f},{self._yaw_rate:.5f},'
            f'{self._x:.4f},{self._y:.4f},{self._yaw:.5f}\n')
        self._fh.flush()

    def destroy_node(self):
        try:
            self._fh.close()
            self.get_logger().info(f'Log written: {self._path}')
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SteeringStep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
