#!/usr/bin/env python3
"""Measure what the control surfaces actually DO in the simulator.

The effectiveness coefficients in airframe.py (AIL_MX_PER_Q, ELE_MY_PER_Q,
RUD_MZ_PER_Q) get measured here rather than derived, because an analytical
reading of the LiftDrag blocks is not enough to be sure of the SIGN, and a wrong
sign closes the pitch or roll loop positive and flies the aircraft into the
ground.

Method: steady-state force balance
----------------------------------
Three open-loop step-response attempts failed first, and it is worth recording
why, because the failures are instructive:

  * Two runs at fixed +D and -D are not comparable. An unstabilised aircraft has
    diverged to a different attitude and airspeed by the time the window opens;
    that version reported ratios of 11 and 24 for the same quantity.
  * A square wave with a 0.35 s window under-reports control power four- to
    five-fold, because this airframe is stiff: the V-tail gives dMy/dalpha of
    roughly 140 N*m/rad, so a 0.25 rad elevator step is cancelled by a 0.2 rad
    change in angle of attack, and the short period is only 0.4 s long.
  * Shrinking the window to 70 ms to get inside the short period drops the
    signal-to-noise ratio to about one, because differentiating the gyro over
    70 ms of a tumbling aircraft is mostly noise.

So this measures a steady state instead of a transient, and lets the flight
controller do the stabilising:

  1. The sim runs with publish_surfaces:=false, so the controller flies the
     aircraft as a pure multirotor - nacelles at zero tilt, every moment on the
     rotors - and never touches the four aerodynamic surfaces.
  2. It is flown to a steady forward-flight trim with real airspeed.
  3. This tool holds one surface at 0, then +D, then -D, and averages the ROTOR
     commands the controller settles on.

In steady state the total moment is zero, so whatever moment the surface makes,
the rotors are cancelling exactly. The rotor moment follows in closed form from
the commanded speeds and the geometry in airframe.py, and differencing +D
against -D removes everything that does not depend on the surface: the
airframe's own aerodynamic moment, gravity, and the rotor reaction torques.

Usage:

    ros2 launch vtol_control vtol_sim.launch.py gui:=false publish_surfaces:=false
    python3 tools/measure_surfaces.py
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from actuator_msgs.msg import Actuators
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64, String

sys.path.insert(0, __file__.rsplit('/tools/', 1)[0] + '/src/vtol_control')
from vtol_control import airframe as af  # noqa: E402

K_MOMENT = 0.016             # <momentConstant> in the plugin layer
DEFL = 0.30                  # rad
DWELL = 5.0                  # s per deflection
SAMPLE = 2.5                 # s averaged at the end of each dwell
CRUISE_PITCH_STICK = -0.55   # nose down, to build airspeed at zero tilt


class Probe(Node):

    def __init__(self):
        super().__init__('measure_surfaces')
        qos = QoSPresetProfiles.SENSOR_DATA.value
        self.pub_input = self.create_publisher(Twist, '/vtol/manual_input', 10)
        self.pub_mode = self.create_publisher(String, '/vtol/mode_request', 10)
        self.pub_arm = self.create_publisher(Bool, '/vtol/arm_request', 10)
        self.pub_surf = {n: self.create_publisher(Float64, f'/vtol/{n}/cmd', 10)
                         for n in ('aileron_left', 'aileron_right',
                                   'ruddervator_left', 'ruddervator_right')}

        self.create_subscription(Imu, '/vtol/imu', self.on_imu, qos)
        self.create_subscription(Odometry, '/vtol/odom', self.on_odom, qos)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(Actuators, '/vtol/command/motor_speed',
                                 self.on_motors, 10)

        self.rpy = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.alt = 0.0
        self.thrust = [0.0, 0.0, 0.0]
        self.joints = {}
        self._last = None

    def on_imu(self, m):
        q = m.orientation
        self.rpy = [
            math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x**2 + q.y**2)),
            math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))),
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))]
        self.gyro = [m.angular_velocity.x, m.angular_velocity.y,
                     m.angular_velocity.z]

    def on_odom(self, m):
        p = m.pose.pose.position
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        cur = (t, p.x, p.y, p.z)
        if self._last and 1e-4 < t - self._last[0] < 0.2:
            dt = t - self._last[0]
            self.vel = [(cur[i + 1] - self._last[i + 1]) / dt for i in range(3)]
        self._last = cur
        self.alt = p.z - 0.24

    def on_joints(self, m):
        self.joints = dict(zip(m.name, m.position))

    def on_motors(self, m):
        self.thrust = [af.K_THRUST * w * w for w in m.velocity[:3]]

    @property
    def airspeed(self):
        return math.sqrt(sum(v * v for v in self.vel))

    # ------------------------------------------------------------------------
    def rotor_moments(self):
        """Moment the rotors are applying, N*m, in base_link axes.

        Tilt is zero throughout, so both front thrust vectors point along body
        +z and only the rear tilt angle mu vectors any thrust sideways.
        """
        f_l, f_r, f_rear = self.thrust
        mu = self.joints.get('rear_tilt_joint', 0.0)
        mx = af.Y_FRONT * (f_l - f_r) + af.Z_REAR * f_rear * math.sin(mu)
        my = -af.X_FRONT * (f_l + f_r) - af.X_REAR * f_rear * math.cos(mu)
        mz = (-af.X_REAR * f_rear * math.sin(mu)
              - K_MOMENT * f_l + K_MOMENT * f_r - K_MOMENT * f_rear)
        return mx, my, mz

    def run(self, seconds, sticks, surfaces):
        end = time.time() + seconds
        acc = []
        while time.time() < end and rclpy.ok():
            t = Twist()
            t.linear.y, t.linear.x, t.angular.z, t.linear.z = (
                float(v) for v in sticks)
            self.pub_input.publish(t)
            for name, pub in self.pub_surf.items():
                pub.publish(Float64(data=float(surfaces.get(name, 0.0))))
            rclpy.spin_once(self, timeout_sec=0.01)
            acc.append((self.rotor_moments(), self.airspeed, list(self.rpy),
                        list(self.gyro)))
        return acc

    @staticmethod
    def average(samples):
        n = max(len(samples), 1)
        mom = [sum(s[0][i] for s in samples) / n for i in range(3)]
        v = sum(s[1] for s in samples) / n
        rpy = [sum(s[2][i] for s in samples) / n for i in range(3)]
        rate = [sum(abs(s[3][i]) for s in samples) / n for i in range(3)]
        return mom, v, rpy, rate


def main():
    rclpy.init()
    p = Probe()
    p.run(2.0, (0, 0, 0, 0), {})

    print('\nArming and climbing. The controller is flying on rotors only '
          '(publish_surfaces:=false).')
    p.pub_arm.publish(Bool(data=True))
    p.run(1.0, (0, 0, 0, 0), {})
    p.pub_mode.publish(String(data='takeoff'))
    for _ in range(60):
        p.run(1.0, (0, 0, 0, 0), {})
        if p.alt > 38.0:
            break
    print(f'  at {p.alt:.0f} m. Accelerating to a steady forward-flight trim.')
    p.run(16.0, (0, CRUISE_PITCH_STICK, 0, 0), {})
    print(f'  trimmed: V {p.airspeed:.1f} m/s, pitch '
          f'{math.degrees(-p.rpy[1]):+.1f} deg, alt {p.alt:.0f} m\n')

    sticks = (0, CRUISE_PITCH_STICK, 0, 0)
    cases = [
        ('ELEVATOR', 1, 'ELE_MY_PER_Q', af.ELE_MY_PER_Q,
         lambda d: {'ruddervator_left': d, 'ruddervator_right': d}),
        ('AILERON', 0, 'AIL_MX_PER_Q', af.AIL_MX_PER_Q,
         lambda d: {'aileron_left': -d, 'aileron_right': d}),
        ('RUDDER', 2, 'RUD_MZ_PER_Q', af.RUD_MZ_PER_Q,
         lambda d: {'ruddervator_left': d, 'ruddervator_right': -d}),
    ]
    axis_name = 'xyz'
    measured = {}

    for label, ax, const_name, const_val, mk in cases:
        rows = {}
        for d in (0.0, +DEFL, -DEFL):
            samples = p.run(DWELL, sticks, mk(d))
            rows[d] = p.average(samples[int(len(samples) * (1.0 - SAMPLE / DWELL)):])
        print(f'{label}   (rotor moment about {axis_name[ax]}, N*m)')
        for d in (0.0, +DEFL, -DEFL):
            mom, v, rpy, rate = rows[d]
            print(f'   defl {d:+5.2f}  M_rotor {mom[ax]:+8.3f}  V {v:5.1f}  '
                  f'roll {math.degrees(rpy[0]):+6.1f}  pitch '
                  f'{math.degrees(-rpy[1]):+6.1f}  |rate| {rate[ax]:.3f}')
        m_per_defl = -(rows[+DEFL][0][ax] - rows[-DEFL][0][ax]) / (2.0 * DEFL)
        v = (rows[+DEFL][1] + rows[-DEFL][1]) / 2.0
        q_dyn = af.dynamic_pressure(v)
        coeff = m_per_defl / q_dyn
        steady = max(rows[+DEFL][3][ax], rows[-DEFL][3][ax]) < 0.30
        v_spread = abs(rows[+DEFL][1] - rows[-DEFL][1]) / max(v, 1e-6)
        ok = steady and v_spread < 0.12
        print(f'   -> surface moment {m_per_defl:+.2f} N*m/rad at q={q_dyn:.0f} Pa')
        print(f'   -> measured {const_name} = {coeff:+.4f}   '
              f'file has {const_val:+.4f}   ratio {coeff / const_val:+.2f}')
        print(f'   -> {"steady" if steady else "RATES NOT SETTLED"}, airspeed '
              f'spread {v_spread * 100:.1f}%  [{"TRUST" if ok else "SUSPECT"}]\n')
        measured[const_name] = (coeff, ok)

    p.pub_mode.publish(String(data='land'))
    p.run(2.0, (0, 0, 0, 0), {})

    print('Summary (the sign is the part that matters):')
    for k, (v, ok) in measured.items():
        rel = 'SAME sign as airframe.py' if v * getattr(af, k) > 0 \
            else 'OPPOSITE sign to airframe.py'
        print(f'   {k} = {v:+.5f}   {rel}{"" if ok else "   (suspect)"}')
    p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
