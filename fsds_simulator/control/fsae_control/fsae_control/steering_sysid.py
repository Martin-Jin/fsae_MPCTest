"""Open-loop steering system-ID: measure FSDS's true steering->yaw response.

WHY THIS EXISTS
---------------
Live lap logs show the car's yaw response is ~3x weaker than commanded: at
full lock (25 deg) at ~6 m/s it achieves only 9.5-10.8 deg of effective
roadwheel angle, while pulling just ~5 m/s2 lateral -- far below the ~12 m/s2
the same car demonstrably reaches.  So it is not tyre saturation.  The offline
plant (`fsae_MPCTest`) is near neutral-steer and does not reproduce this at
all.  See `fsae_MPCTest/docs/planning_control_sync.md` ->
"MEASURED: the car's yaw response is ~3x weaker than commanded".

Lap logs cannot separate the plant from the controller -- the MPC is always
reacting, and speed/steer/lateral-load are all confounded along a lap.  This
node removes the controller entirely: it publishes fixed steering commands at
fixed speeds and records what the car actually does.

WHAT IT DISTINGUISHES
---------------------
Let s = delta_achieved / delta_commanded, where the achieved roadwheel angle is
inverted from the kinematic bicycle, delta_achieved = atan(L * yaw_rate / v):

  * s flat across BOTH speed and steering angle
        -> constant rack-scale error (MAX_STEER_RAD simply wrong)
  * s falls as SPEED rises, at fixed steering angle
        -> nonlinear / speed-scaled steering map inside FSDS  [leading candidate]
  * s falls as STEERING ANGLE rises, at fixed speed
        -> nonlinear rack geometry, or genuine tyre saturation; separate the
           two by checking whether |a_lat| is near the ~12 m/s2 grip ceiling
  * yaw rate decays while the steering command is held constant
        -> unmodelled yaw damping / stability control in FSDS

IMPORTANT: this logs the RAW NORMALISED command actually sent to FSDS
(`cmd.steering`, in [-1, 1]) alongside the roadwheel angle we believe it maps
to.  The open question is precisely whether that mapping is what we assume, so
recording only the assumed angle would beg the question.

SAFETY / USAGE
--------------
Run on an EMPTY map with clear space -- this drives in circles at up to the
configured top speed and does NOT brake for cones.  It ignores the perception
proximity brake that `fsds_bridge` implements.

Do not run this at the same time as `fsds_bridge` or any controller: both
publish to /fsds/control_command and the commands would interleave.

    ros2 run fsae_control steering_sysid
    ros2 run fsae_control steering_sysid --ros-args \
        -p speeds:="[4.0, 8.0]" -p steer_cmds:="[0.4, 1.0]"

Writes a CSV to ~/fsae_logs/steering_sysid_<unix_ts>.csv.  Analyse with
`fsae_MPCTest/tuner/steering_sysid_analysis.py`.
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

# Must match fsds_bridge.MAX_STEER_RAD -- this is the assumed mapping under
# test, not a verified fact.
MAX_STEER_RAD = math.radians(25.0)

# Speed-hold gains.  KP_BRAKE matches fsds_bridge; the throttle side does NOT
# (see _throttle_brake for why -- fsds_bridge's pure-P law cannot launch the
# car from rest and always leaves a steady-state speed offset).
KP_BRAKE = 0.40
KP_THROTTLE_SYSID = 0.25   # throttle per m/s of under-speed
KI_THROTTLE = 0.30         # integral gain — supplies holding throttle at v=target
I_THROTTLE_MAX = 0.85      # integrator clamp (anti-windup)
LAUNCH_SPEED = 1.0         # m/s below which the launch floor applies
LAUNCH_THROTTLE = 0.55     # enough to break static friction from a standstill
RETURN_SPEED = 6.0         # m/s while driving back to the start point

# Understeer coefficient used ONLY to predict how big each test point's circle
# will be, so points too large for the geofence can be skipped up front.
# Deliberately pessimistic: 0.05 is above the ~0.038-0.045 measured on the car,
# because over-estimating the radius costs a skipped point while
# under-estimating it costs a geofence abort mid-measurement.
K_US_ESTIMATE = 0.05

LOG_DIR = os.path.expanduser('~/fsae_logs')


class Phase:
    """Test sequence phases."""
    SETTLE_SPEED = 'settle_speed'   # straight-line, reach target speed
    HOLD_STEER = 'hold_steer'       # steering applied, yaw settling
    RECORD = 'record'               # steering held, sampling
    RECOVER = 'recover'             # straight, slow down between points
    RETURN = 'return'               # drive back toward the start point


class SteeringSysId(Node):
    def __init__(self):
        super().__init__('steering_sysid')

        # Wide speed range on purpose.  A speed-scaled rack and genuine
        # understeer are nearly degenerate over a narrow band (they sat within
        # 0.004 R^2 of each other over 4-10 m/s in synthetic validation);
        # spanning 3-14 m/s pushes them apart to a separable margin.
        self.declare_parameter('speeds', [3.0, 5.0, 8.0, 11.0, 14.0])
        # Biased toward larger angles: small commands at high speed trace
        # circles too big for any bounded area (see the plan-building filter
        # below), and give the weakest yaw signal. 0.5-1.0 keeps every circle
        # inside a ~70 m fence while spanning enough angle to expose any
        # nonlinearity in the rack.
        self.declare_parameter('steer_cmds', [0.5, 0.65, 0.8, 1.0])
        self.declare_parameter('settle_s', 4.0)
        self.declare_parameter('hold_s', 2.5)
        self.declare_parameter('record_s', 2.0)
        self.declare_parameter('recover_s', 2.0)
        self.declare_parameter('alternate_sign', True)
        self.declare_parameter('require_go', True)
        # Geofence.  home_radius: come back before the next point if further
        # than this.  max_radius: hard limit — abandon the current point
        # immediately.  Sized for FSDS's open maps; shrink for a small area.
        self.declare_parameter('home_radius', 40.0)
        self.declare_parameter('max_radius', 70.0)

        self.speeds = list(self.get_parameter('speeds').value)
        self.steer_cmds = list(self.get_parameter('steer_cmds').value)
        self.settle_s = float(self.get_parameter('settle_s').value)
        self.hold_s = float(self.get_parameter('hold_s').value)
        self.record_s = float(self.get_parameter('record_s').value)
        self.recover_s = float(self.get_parameter('recover_s').value)
        self.alternate_sign = bool(self.get_parameter('alternate_sign').value)
        self._go_received = not bool(self.get_parameter('require_go').value)
        self.home_radius = float(self.get_parameter('home_radius').value)
        self.max_radius = float(self.get_parameter('max_radius').value)
        self._home = None   # set from the first odom sample

        # Build the plan, dropping (speed, steer) pairs whose steady-state
        # circle will not fit inside the geofence.
        #
        # The circle radius is roughly R = v/yaw_rate = (L + K v^2)/delta, so
        # high speed with small steering traces an enormous arc: 14 m/s at 0.2
        # normalised steering is a ~108 m radius (215 m across). Those points
        # cannot be driven in a bounded area at all, and they are also the
        # least informative -- a small angle produces a small yaw signal.
        # Estimated with a deliberately pessimistic (near-neutral) K, since the
        # whole question is what K actually is; underestimating R would let
        # unfittable points back in.
        self.plan = []
        skipped = []
        for i, v in enumerate(self.speeds):
            for j, sc in enumerate(self.steer_cmds):
                delta = abs(float(sc)) * MAX_STEER_RAD
                if delta < 1e-6:
                    continue
                # The orbit is not centred on home: the car enters the circle
                # from wherever the previous point left it, so its distance
                # from the start reaches roughly TWICE the turn radius. Budget
                # for the diameter, not the radius.
                #
                # Use a PESSIMISTIC (high) understeer coefficient here. The
                # live car measures K_us ~ 0.04, which inflates the radius
                # sharply with speed -- at 14 m/s and 0.5 steering the real
                # orbit is ~86 m across, not the ~23 m a near-neutral estimate
                # predicts. Underestimating K lets unfittable points into the
                # plan, where they trip the geofence on every lap and starve
                # the sweep of time (measured: 126 fence triggers, 16/20
                # points done). Assuming the worst simply skips them.
                r_est = (1.55 + K_US_ESTIMATE * float(v) ** 2) / delta
                if 2.0 * r_est > self.max_radius * 0.9:
                    skipped.append((float(v), float(sc), r_est))
                    continue
                sign = -1.0 if (self.alternate_sign and (i + j) % 2) else 1.0
                self.plan.append((float(v), float(sc) * sign))

        if skipped:
            self.get_logger().warn(
                f'Skipping {len(skipped)} test point(s) whose turning circle '
                f'exceeds the {self.max_radius:.0f} m geofence:')
            for v, sc, r in skipped:
                self.get_logger().warn(
                    f'    v={v:.1f} steer={sc:.2f} -> R~{r:.0f} m')
            self.get_logger().warn(
                'Raise max_radius on a larger map to include them.')
        if not self.plan:
            raise RuntimeError(
                'No test points fit inside the geofence. Raise max_radius or '
                'use larger steering commands.')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Odometry, '/fsds/testing_only/odom',
                                 self._odom_cb, sensor_qos)
        self.create_subscription(GoSignal, '/fsds/signal/go', self._go_cb, 10)
        self.pub_cmd = self.create_publisher(ControlCommand,
                                             '/fsds/control_command', 10)

        self._speed = 0.0
        self._yaw_rate = 0.0
        self._x = self._y = self._yaw = 0.0
        self._have_odom = False
        self._i_term = 0.0          # throttle integrator (see _throttle_brake)
        self._stuck_warned = False

        self._idx = 0
        self._phase = Phase.SETTLE_SPEED
        self._phase_t0 = time.time()
        self._t0 = time.time()

        os.makedirs(LOG_DIR, exist_ok=True)
        self._path = os.path.join(LOG_DIR, f'steering_sysid_{int(self._t0)}.csv')
        self._fh = open(self._path, 'w')
        self._write_header()

        self.create_timer(0.05, self._loop)  # 20 Hz, matching fsds_bridge
        self.get_logger().info(
            f'steering_sysid: {len(self.plan)} test points, logging to {self._path}')
        if not self._go_received:
            self.get_logger().info('Waiting for GO signal...')

    # ------------------------------------------------------------------

    def _write_header(self):
        self._fh.write(
            '# fsae open-loop steering system-ID\n'
            f'# t0_epoch_s={self._t0:.4f}\n'
            f'# MAX_STEER_RAD_assumed={MAX_STEER_RAD:.6f} '
            f'({math.degrees(MAX_STEER_RAD):.1f} deg)\n'
            '# steer_norm is the RAW value sent to FSDS in [-1,1]; '
            'delta_cmd_rad is the angle we ASSUME it maps to\n'
            '# delta_achieved = atan(L*yaw_rate/v) must be computed at '
            'analysis time (wheelbase is a fit parameter)\n'
        )
        self._fh.write(
            't,phase,target_v,steer_norm,delta_cmd_rad,throttle,brake,'
            'v_actual,yaw_rate,car_x,car_y,car_yaw\n'
        )
        self._fh.flush()

    def _go_cb(self, msg: GoSignal) -> None:
        if not self._go_received:
            self._go_received = True
            self.get_logger().info('GO received — starting sweep.')

    def _odom_cb(self, msg: Odometry) -> None:
        v = msg.twist.twist.linear
        self._speed = float(math.hypot(v.x, v.y))
        self._yaw_rate = float(msg.twist.twist.angular.z)
        p = msg.pose.pose.position
        self._x, self._y = float(p.x), float(p.y)
        q = msg.pose.pose.orientation
        self._yaw = float(math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        ))
        if self._home is None:
            self._home = (self._x, self._y)
        self._have_odom = True

    # ------------------------------------------------------------------

    def _throttle_brake(self, target_v, dt=0.05):
        """Speed-hold PI control with a launch floor.

        NOT the same as fsds_bridge's pure-P law, deliberately.  Two reasons:

        1. Static friction.  fsds_bridge's KP_THROTTLE=0.06 yields only
           0.06 * 3.0 = 0.18 throttle at the lowest target speed, which is not
           enough to move the car from rest -- the sim just sits at 0 m/s.
           The bridge never hits this because the car is already rolling by the
           time the MPC takes over; this node starts from a standstill.
        2. Steady-state error.  A pure P law needs a non-zero error to produce
           the throttle that balances drag, so it always undershoots.  Here the
           speed IS the independent variable of the experiment, so a persistent
           offset would bias every measurement.

        The integral term supplies the holding throttle at zero error, and the
        launch floor guarantees enough torque to break static friction while
        the car is still nearly stopped.
        """
        err = target_v - self._speed

        if err < -0.5:
            # Braking: bleed the integrator so it cannot wind up against the
            # brake and then surge when the brake releases.
            self._i_term *= 0.9
            return 0.0, min(1.0, KP_BRAKE * (-err))

        self._i_term = float(np.clip(self._i_term + KI_THROTTLE * err * dt,
                                     0.0, I_THROTTLE_MAX))
        throttle = KP_THROTTLE_SYSID * err + self._i_term

        # Launch floor: below LAUNCH_SPEED the car may be stuck in static
        # friction, where a small throttle produces no motion and no error
        # reduction.  Force enough torque to actually get it rolling.
        if self._speed < LAUNCH_SPEED and err > 0.0:
            throttle = max(throttle, LAUNCH_THROTTLE)

        return float(np.clip(throttle, 0.0, 1.0)), 0.0

    def _advance(self, phase):
        self._phase = phase
        self._phase_t0 = time.time()

    # ------------------------------------------------------------------
    # Geofence: keep the car circling near where it started
    # ------------------------------------------------------------------

    def _dist_home(self):
        if self._home is None:
            return 0.0
        return float(math.hypot(self._x - self._home[0], self._y - self._home[1]))

    def _steer_toward_home(self):
        """Proportional heading control back to the start point."""
        if self._home is None:
            return 0.0
        bearing = math.atan2(self._home[1] - self._y, self._home[0] - self._x)
        err = math.atan2(math.sin(bearing - self._yaw),
                         math.cos(bearing - self._yaw))
        # Sign matches the FSDS convention used everywhere else here:
        # cmd.steering = -delta / MAX_STEER_RAD, so a +ve heading error
        # (turn left) needs a -ve normalised command.
        return float(np.clip(-2.0 * err / MAX_STEER_RAD, -1.0, 1.0))

    def _abort(self, reason):
        self.get_logger().error(f'ABORTING SWEEP: {reason}')
        self.get_logger().error(
            f'Completed {self._idx}/{len(self.plan)} points. Partial log is '
            f'still valid up to this point: {self._path}')
        cmd = ControlCommand()
        cmd.throttle, cmd.brake, cmd.steering = 0.0, 1.0, 0.0
        self.pub_cmd.publish(cmd)
        self._idx = len(self.plan)   # stops the sweep; loop() idles braking

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

        # HARD GEOFENCE — checked from every phase, before anything else can
        # command steering.  The alternating turn direction alone is not
        # enough: each point ends with the car pointing somewhere new, so the
        # offsets compound.  Interrupt whatever we are doing and come home.
        if self._phase != Phase.RETURN and self._dist_home() > self.max_radius:
            self.get_logger().warn(
                f'GEOFENCE: {self._dist_home():.0f} m from start (limit '
                f'{self.max_radius:.0f} m) — abandoning point '
                f'{self._idx + 1} and returning.')
            self._advance(Phase.RETURN)
            return

        # Reach the target speed while ALREADY TURNING, not in a straight line.
        #
        # Straight-line settling was the original design and it does not fit in
        # any usable area: at 14 m/s, 4 s of settling plus 2.5 s of hold plus
        # 2 s of recording carries the car ~120 m downrange, so every point
        # ended outside the fence and the sweep spent its time returning
        # instead of measuring (measured: 10-15 of 25 points in 25 min).
        #
        # Turning throughout keeps the car on a closed circle whose radius is
        # set by the test point itself, so it never travels away from home.
        # The yaw transient still settles -- HOLD_STEER exists precisely to let
        # it -- and RECORD only samples once it has.
        if self._phase == Phase.SETTLE_SPEED:
            applied_steer = steer_norm
            # A stuck car would otherwise burn through all 25 points in silence
            # and leave a log full of zero-speed rows.  Say so loudly instead.
            if elapsed > 5.0 and self._speed < 0.5 and not self._stuck_warned:
                self._stuck_warned = True
                self.get_logger().error(
                    'Car is NOT MOVING after 5 s of full throttle — most '
                    'likely wedged against a wall. Aborting: further points '
                    'would only log zeros.')
            # Wedged against something: no point continuing the sweep.
            if elapsed > 9.0 and self._speed < 0.5:
                self._abort('car stalled against an obstacle')
                return
            # Advance on reaching speed, or on timeout if it cannot get there.
            if (abs(self._speed - target_v) < 0.3 and elapsed > 1.0) \
                    or elapsed > self.settle_s + 8.0:
                self._advance(Phase.HOLD_STEER)
        elif self._phase == Phase.HOLD_STEER:
            applied_steer = steer_norm
            if elapsed > self.hold_s:
                self._advance(Phase.RECORD)
        elif self._phase == Phase.RECORD:
            applied_steer = steer_norm
            if elapsed > self.record_s:
                self._advance(Phase.RECOVER)
        elif self._phase == Phase.RETURN:
            # Steer back toward the start point.  Each test point leaves the
            # car pointing somewhere new, so without this the offsets compound
            # and the car walks off across the map (measured: 103 m of drift
            # before hitting a wall on the first real run).
            applied_steer = self._steer_toward_home()
            target_v = RETURN_SPEED
            if self._dist_home() < self.home_radius * 0.5 or elapsed > 25.0:
                self._advance(Phase.SETTLE_SPEED)
                self._i_term = 0.0
        else:  # RECOVER
            # Keep circling rather than straightening: a straight recovery
            # phase is itself a departure from home, and the next point has to
            # re-establish a turn anyway.
            applied_steer = steer_norm
            if elapsed > self.recover_s:
                self._idx += 1
                self._stuck_warned = False
                # Carry the integrator across points only when the next target
                # is the same speed; otherwise it is holding the wrong value.
                if self._idx < len(self.plan) \
                        and abs(self.plan[self._idx][0] - target_v) > 0.1:
                    self._i_term = 0.0
                # Come back before starting the next point if we have wandered.
                if self._dist_home() > self.home_radius:
                    self.get_logger().info(
                        f'{self._dist_home():.0f} m from start — returning '
                        f'before the next point.')
                    self._advance(Phase.RETURN)
                else:
                    self._advance(Phase.SETTLE_SPEED)
                if self._idx < len(self.plan):
                    nv, ns = self.plan[self._idx]
                    self.get_logger().info(
                        f'point {self._idx + 1}/{len(self.plan)}: '
                        f'v={nv:.1f} m/s steer_norm={ns:+.2f}')
                else:
                    self.get_logger().info('Sweep complete — braking.')

        throttle, brake = self._throttle_brake(target_v)

        cmd = ControlCommand()
        cmd.throttle = float(throttle)
        cmd.brake = float(brake)
        cmd.steering = float(np.clip(applied_steer, -1.0, 1.0))
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.pub_cmd.publish(cmd)

        # delta_cmd_rad records the mapping under test, so a wrong assumption
        # stays visible and correctable at analysis time.
        delta_cmd = -cmd.steering * MAX_STEER_RAD
        self._fh.write(
            f'{time.time() - self._t0:.4f},{self._phase},{target_v:.3f},'
            f'{cmd.steering:.5f},{delta_cmd:.6f},{cmd.throttle:.4f},'
            f'{cmd.brake:.4f},{self._speed:.4f},{self._yaw_rate:.5f},'
            f'{self._x:.4f},{self._y:.4f},{self._yaw:.5f}\n'
        )
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
    node = SteeringSysId()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
