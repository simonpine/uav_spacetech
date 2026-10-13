#!/usr/bin/env python3
"""Stabilised flight controller for the tri-motor tiltrotor VTOL.

The description package's README is explicit that it ships no controller: "no
stabilisation, no mixer, no transition sequencing". This node is that missing
layer. It is a cascaded attitude controller plus an effector allocator that
blends continuously between multirotor and fixed-wing authority, so one set of
loops covers hover, transition and cruise.

    pilot sticks -> attitude/rate loops -> desired (Fz, Mx, My, Mz)
                                              |
                          +-------------------+-------------------+
                          |                                       |
                   rotor allocation                      surface allocation
             (front pair + rear rotor + rear          (ailerons, ruddervators,
              tilt, weighted by cos(tilt))             weighted by (V/V_aero)^2)

Why it is built this way
------------------------
The obvious design - two controllers with a switch at 45 degrees of tilt - fails
on this airframe, because at the switch point neither effector group has full
authority: the rotors' pitch arm has collapsed and the wing has not yet got the
dynamic pressure. So instead every axis's demand is split between the two groups
in proportion to the authority each actually has right now, and the two shares
always sum to the full demand. There is no switch and no discontinuity.

Two things in here are less obvious and were both found by flying it:

1. The rear rotor is solved for PITCH, sequentially, after the front pair
   thrust has been decided - not jointly with the vertical force in a 2x2. The
   2x2 is exact in hover but goes singular at 84 degrees of tilt, and blending
   its output with a throttle-driven cruise mix injects an uncompensated
   nose-up moment: the front rotors are ahead of the CoM, so any front thrust
   the 2x2 did not ask for pitches the aircraft up. Solving pitch last makes
   the pitch axis exact at every tilt angle and leaves the vertical axis - which
   has a climb-rate integrator behind it - to absorb the slack.

2. The tilt angle is scheduled on AIRSPEED, not on a timer. That makes the front
   transition self-regulating: the nacelles only reach 90 degrees once the
   aircraft is genuinely fast enough for the wing to carry it, and if
   acceleration stalls the tilt stops advancing instead of dropping the
   aircraft.

Sign conventions are the raw URDF FLU ones (+My = nose down, +Mz = nose left).
See airframe.py for the table. Pilot inputs use the intuitive aerospace signs
and are converted at the boundary, in compute_setpoints().
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from actuator_msgs.msg import Actuators
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64, String

from . import airframe as af

# Fraction of the weight the front pair carries in hover, set by the pitch
# balance: 2 * 16.043 / 53.897.
FRONT_LIFT_SHARE = 0.5954

# det of the hover 2x2 allocation, used as the rotor vertical/pitch authority
# measure. 0.900 at zero tilt, zero at 84.3 degrees.
DET_HOVER = 0.900
DET_FLOOR = 0.150


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Pid:
    """Rate-loop PID with conditional integration.

    The output is an angular acceleration, so the caller multiplies by the axis
    inertia to get a moment. Keeping the gains in 1/s lets you read them as loop
    bandwidths instead of guessing at N*m units.
    """

    def __init__(self, kp, ki, kd, i_limit, out_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit, self.out_limit = i_limit, out_limit
        self.integral = 0.0
        self.prev_meas = 0.0
        self.d_filt = 0.0

    def reset(self):
        self.integral = 0.0
        self.d_filt = 0.0

    def update(self, setpoint, meas, dt):
        err = setpoint - meas
        # Derivative on the measurement, low-passed: differentiating the
        # setpoint would kick on every key press.
        if dt > 0.0:
            raw_d = (meas - self.prev_meas) / dt
            self.d_filt += clamp(dt / (dt + 0.02), 0.0, 1.0) * (raw_d - self.d_filt)
        self.prev_meas = meas

        out = self.kp * err - self.kd * self.d_filt
        cand = clamp(self.integral + self.ki * err * dt, -self.i_limit, self.i_limit)
        # Only integrate while the output still has room, so a saturated
        # allocator cannot wind the integrator up.
        if abs(out + cand) < self.out_limit:
            self.integral = cand
        return clamp(out + self.integral, -self.out_limit, self.out_limit)


class FlightController(Node):

    MC = 'DRONE'
    T_FW = 'TRANS>PLANE'
    FW = 'AIRPLANE'
    T_MC = 'TRANS>DRONE'

    def __init__(self):
        super().__init__('vtol_flight_controller')

        declared = self.declare_parameters('', [
            ('att_p_roll', 7.0), ('att_p_pitch', 6.0),
            ('rate_roll_p', 14.0), ('rate_roll_i', 9.0), ('rate_roll_d', 0.55),
            ('rate_pitch_p', 13.0), ('rate_pitch_i', 8.0), ('rate_pitch_d', 0.55),
            ('rate_yaw_p', 7.0), ('rate_yaw_i', 3.0), ('rate_yaw_d', 0.15),
            ('max_rate_roll', 3.5), ('max_rate_pitch', 2.5), ('max_rate_yaw', 2.0),
            ('moment_limit_x', 8.0), ('moment_limit_y', 10.0), ('moment_limit_z', 5.5),
            ('max_bank_mc', 0.40), ('max_pitch_mc', 0.35),
            ('max_bank_fw', 0.80), ('max_pitch_fw', 0.45),
            ('max_yaw_rate_mc', 1.6), ('max_yaw_rate_fw', 1.0),
            ('max_climb_rate', 3.5), ('throttle_slew', 0.55),
            ('alt_p', 1.1), ('vz_p', 3.2), ('vz_i', 1.8), ('vz_d', 0.10),
            ('accel_limit_up', 9.0), ('accel_limit_down', 6.0),
            # Wing-borne altitude hold is deliberately much gentler than the
            # pilot's own pitch authority: a 26 deg dive to recover 4 m turns
            # into a 38 m/s overspeed on an airframe with this little drag.
            ('fw_alt_p', 0.25), ('fw_alt_pitch_p', 0.085),
            ('fw_max_climb', 2.5), ('fw_pitch_limit', 0.20),
            # AIRPLANE throttle is an airspeed hold, not a raw power lever.
            ('v_target_default', 20.0), ('v_target_min', 14.0),
            ('v_target_max', 30.0), ('v_target_slew', 4.0),
            ('speed_p', 0.055), ('speed_i', 0.075),
            ('v_trans_start', 2.0), ('v_trans_full', 19.0),
            ('tilt_trans_min', 0.30),
            ('tilt_rate_fwd', 0.50), ('tilt_rate_back', 0.45),
            ('v_min_fixedwing', 15.0), ('v_cruise', 21.0), ('v_aero_full', 16.0),
            ('trans_min_altitude', 12.0), ('fw_min_altitude', 6.0),
            ('trans_throttle_p', 0.065), ('trans_throttle_ff', 0.50),
            ('takeoff_altitude', 15.0), ('takeoff_climb_rate', 2.5),
            ('land_descent_rate', 3.0), ('land_descent_min', 0.45),
            ('land_descent_slope', 0.35), ('ground_clearance', 0.24),
            ('input_timeout', 1.5), ('status_rate', 10.0),
            # Set false to fly the airframe as a pure multirotor with the
            # aerodynamic surfaces left alone. tools/measure_surfaces.py needs
            # this so it can own the surfaces while the rotors hold attitude.
            ('publish_surfaces', True),
        ])
        self.P = {d.name: d.value for d in declared}

        # ------------------------------------------------------------- state --
        self.state = self.MC
        self.armed = False
        self.auto = None                      # None | 'TAKEOFF' | 'LAND'
        self.message = 'disarmed - press ENTER to arm'

        self.tilt_cmd = 0.0
        self.throttle = 0.0                   # front-pair power, 0..1
        self.v_target = 0.0                   # AIRPLANE airspeed hold, m/s
        self.thr_i = 0.0                      # airspeed-hold integrator
        self.alt_sp = None
        self.land_timer = 0.0

        self.roll = self.pitch = self.yaw = 0.0
        self.p_body = self.q_body = self.r_body = 0.0
        self.pos = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.airspeed = 0.0
        self.alpha = 0.0
        self.have_odom = self.have_imu = False
        self._last_odom = None
        self._dbg = (0.0,) * 6

        self.sticks = [0.0, 0.0, 0.0, 0.0]    # roll, pitch, yaw, power
        self.sticks_stamp = -1e9
        self.t_imu = None

        self.pid_roll = Pid(self.P['rate_roll_p'], self.P['rate_roll_i'],
                            self.P['rate_roll_d'], 6.0, 30.0)
        self.pid_pitch = Pid(self.P['rate_pitch_p'], self.P['rate_pitch_i'],
                             self.P['rate_pitch_d'], 6.0, 25.0)
        self.pid_yaw = Pid(self.P['rate_yaw_p'], self.P['rate_yaw_i'],
                           self.P['rate_yaw_d'], 4.0, 12.0)
        self.pid_vz = Pid(self.P['vz_p'], self.P['vz_i'], self.P['vz_d'], 8.0,
                          self.P['accel_limit_up'])

        # -------------------------------------------------------------- ros ---
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self.pub_motors = self.create_publisher(Actuators, '/vtol/command/motor_speed', 10)
        self.pub_servo = {name: self.create_publisher(Float64, f'/vtol/{name}/cmd', 10)
                          for name in af.SERVO_TOPICS}
        self.pub_status = self.create_publisher(String, '/vtol/status', 10)

        self.create_subscription(Imu, '/vtol/imu', self.on_imu, sensor_qos)
        self.create_subscription(Odometry, '/vtol/odom', self.on_odom, sensor_qos)
        self.create_subscription(Twist, '/vtol/manual_input', self.on_sticks, 10)
        self.create_subscription(String, '/vtol/mode_request', self.on_mode, 10)
        self.create_subscription(Bool, '/vtol/arm_request', self.on_arm, 10)

        self._servo_decim = self._motor_decim = 0
        self.create_timer(1.0 / self.P['status_rate'], self.publish_status)

        self.get_logger().info(
            'VTOL flight controller up, waiting for /vtol/imu and /vtol/odom. '
            'Run "ros2 run vtol_control keyboard_teleop" in another terminal to fly.')

    # ================================================================ input ==
    def on_sticks(self, msg: Twist):
        self.sticks = [clamp(msg.linear.y, -1.0, 1.0),     # roll,  + right
                       clamp(msg.linear.x, -1.0, 1.0),     # pitch, + nose up
                       clamp(msg.angular.z, -1.0, 1.0),    # yaw,   + right
                       clamp(msg.linear.z, -1.0, 1.0)]     # power, + up
        self.sticks_stamp = self.now

    def on_arm(self, msg: Bool):
        self.arm() if msg.data else self.disarm('disarmed by pilot')

    def on_mode(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd in ('toggle', ''):
            self.toggle_mode()
        elif cmd in ('mc', 'drone', 'hover'):
            self.request_mc()
        elif cmd in ('fw', 'plane', 'airplane', 'cruise'):
            self.request_fw()
        elif cmd == 'takeoff':
            self.start_takeoff()
        elif cmd == 'land':
            self.start_land()
        elif cmd == 'hold':
            self.auto = None
            self.alt_sp = self.pos[2]
            self.message = 'holding altitude'
        elif cmd == 'kill':
            self.disarm('KILLED')

    def on_imu(self, msg: Imu):
        q = msg.orientation
        # ZYX extraction. In this FLU frame +pitch is nose DOWN and +yaw is nose
        # LEFT; the pilot-side conversion is in compute_setpoints().
        self.roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                               1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        self.pitch = math.asin(clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0))
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.p_body = msg.angular_velocity.x
        self.q_body = msg.angular_velocity.y
        self.r_body = msg.angular_velocity.z

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.have_imu = True
        if self.t_imu is None or not 0.0 < t - self.t_imu < 0.25:
            self.t_imu = t
            return
        dt, self.t_imu = t - self.t_imu, t
        self.update(dt)

    def on_odom(self, msg: Odometry):
        pos = msg.pose.pose.position
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        new = (t, pos.x, pos.y, pos.z)
        if self._last_odom is not None and 1e-4 < new[0] - self._last_odom[0] < 0.25:
            dt = new[0] - self._last_odom[0]
            # Sim pose is exact, so a plain difference is a clean velocity; the
            # light filter only removes the one-sample staircase.
            a = clamp(dt / (dt + 0.015), 0.0, 1.0)
            for i in range(3):
                self.vel[i] += a * ((new[i + 1] - self._last_odom[i + 1]) / dt
                                    - self.vel[i])
            self._last_odom = new
        elif self._last_odom is None:
            self._last_odom = new
        self.pos = [pos.x, pos.y, pos.z]
        self.have_odom = True

        vx, vy, vz = self.vel
        self.airspeed = math.sqrt(vx * vx + vy * vy + vz * vz)
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        # body = R^T * world, with R = Rz(yaw) Ry(pitch) Rx(roll)
        u = cp * cy * vx + cp * sy * vy - sp * vz
        w = ((cy * sp * cr + sy * sr) * vx + (sy * sp * cr - cy * sr) * vy
             + cp * cr * vz)
        self.alpha = -math.atan2(w, u) if u > 1.0 else 0.0

    @property
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @property
    def agl(self):
        return self.pos[2] - self.P['ground_clearance']

    def beta(self):
        """Rotor vertical/pitch authority, 1 in hover and 0 past ~76 deg tilt."""
        c, s = math.cos(self.tilt_cmd), math.sin(self.tilt_cmd)
        det = -af.X_REAR * c - (af.Z_FRONT * s - af.X_FRONT * c)
        return clamp((det - DET_FLOOR) / (DET_HOVER - DET_FLOOR), 0.0, 1.0)

    # ============================================================= commands ==
    def arm(self):
        if self.armed:
            return
        if abs(self.roll) > 0.35 or abs(self.pitch) > 0.35:
            self.message = 'refusing to arm: not level'
            return
        self.armed = True
        self.state = self.MC
        self.tilt_cmd = self.throttle = self.thr_i = 0.0
        self.v_target = self.P['v_target_default']
        self.auto = None
        self.alt_sp = self.pos[2]
        self.reset_loops()
        self.message = 'ARMED in DRONE mode - R to climb, or T for auto take-off'

    def disarm(self, why):
        self.armed = False
        self.auto = None
        self.throttle = self.tilt_cmd = self.thr_i = 0.0
        self.state = self.MC
        self.reset_loops()
        self.message = why

    def reset_loops(self):
        for pid in (self.pid_roll, self.pid_pitch, self.pid_yaw, self.pid_vz):
            pid.reset()

    def start_takeoff(self):
        if not self.armed:
            self.arm()
        if not self.armed:
            return
        if self.state != self.MC:
            self.message = 'take-off only from DRONE mode'
            return
        self.auto = 'TAKEOFF'
        self.alt_sp = self.pos[2] + self.P['takeoff_altitude']
        self.message = f'auto take-off to {self.P["takeoff_altitude"]:.0f} m'

    def start_land(self):
        if self.state in (self.FW, self.T_FW):
            self.request_mc()
        self.auto = 'LAND'
        self.land_timer = 0.0
        self.message = 'auto land'

    def toggle_mode(self):
        self.request_fw() if self.state in (self.MC, self.T_MC) else self.request_mc()

    def request_fw(self):
        if not self.armed:
            self.message = 'arm first (ENTER)'
            return
        if self.agl < self.P['trans_min_altitude']:
            self.message = (f'need {self.P["trans_min_altitude"]:.0f} m AGL to '
                            f'transition (at {self.agl:.1f} m) - climb with R, or T')
            return
        self.auto = None
        self.state = self.T_FW
        self.alt_sp = self.pos[2]
        self.message = 'transitioning to AIRPLANE - accelerating'

    def request_mc(self):
        if not self.armed:
            return
        self.state = self.T_MC
        self.alt_sp = self.pos[2]
        self.message = 'transitioning to DRONE - decelerating'

    # ============================================================ main loop ==
    def update(self, dt):
        if not (self.have_imu and self.have_odom):
            return
        if self.alt_sp is None:
            self.alt_sp = self.pos[2]
        if self.now - self.sticks_stamp > self.P['input_timeout']:
            self.sticks = [0.0, 0.0, 0.0, 0.0]

        if not self.armed:
            self.send_motors(0.0, 0.0, 0.0)
            self.send_servos(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            return

        self.run_automation(dt)
        self.run_state_machine(dt)
        roll_sp, pitch_sp, yaw_rate_sp = self.compute_setpoints(dt)
        mx, my, mz = self.attitude_loop(roll_sp, pitch_sp, yaw_rate_sp, dt)
        fz = self.vertical_loop(dt)
        self.allocate(fz, mx, my, mz)

    def run_automation(self, dt):
        if self.auto == 'TAKEOFF':
            if self.pos[2] >= self.alt_sp - 0.7:
                self.auto = None
                self.message = 'take-off complete - SPACE for AIRPLANE mode'
        elif self.auto == 'LAND':
            touching = self.agl < 0.25 and abs(self.vel[2]) < 0.25
            self.land_timer = self.land_timer + dt if touching else 0.0
            if self.land_timer > 0.8:
                self.disarm('landed and disarmed')

    def run_state_machine(self, dt):
        tmax, v = af.TILT_MAX, self.airspeed

        if self.state == self.MC:
            self.tilt_cmd -= clamp(self.tilt_cmd, 0.0, self.P['tilt_rate_back'] * dt)
            self.throttle = 0.0

        elif self.state == self.T_FW:
            frac = ((v - self.P['v_trans_start'])
                    / max(self.P['v_trans_full'] - self.P['v_trans_start'], 1e-3))
            target = max(self.P['tilt_trans_min'], clamp(frac, 0.0, 1.0) * tmax)
            rate = self.P['tilt_rate_fwd'] * dt
            self.tilt_cmd += clamp(target - self.tilt_cmd, -rate, rate)
            # Auto-throttle towards cruise speed; becomes the pilot's lever
            # setting the moment AIRPLANE is declared.
            self.throttle = clamp(
                self.P['trans_throttle_ff']
                + self.P['trans_throttle_p'] * (self.P['v_cruise'] - v), 0.15, 1.0)
            if self.tilt_cmd > tmax - 0.02 and v >= self.P['v_min_fixedwing']:
                self.state = self.FW
                # Hand over smoothly: hold the altitude and speed it arrives
                # with, and seed the throttle integrator with the auto-throttle
                # value so the airspeed hold does not start from zero.
                self.alt_sp = self.pos[2]
                self.v_target = clamp(v, self.P['v_target_min'],
                                      self.P['v_target_max'])
                self.thr_i = self.throttle
                self.message = (f'AIRPLANE - holding {self.v_target:.0f} m/s, '
                                f'R/F trims speed, SPACE to come back')

        elif self.state == self.FW:
            self.tilt_cmd = tmax
            # Hand back to the rotors rather than let a keyboard pilot mush it
            # into the ground.
            if v < af.V_STALL * 0.95:
                self.request_mc()
                self.message = f'STALL PROTECTION ({v:.1f} m/s) - back to DRONE'
            elif self.agl < self.P['fw_min_altitude']:
                self.request_mc()
                self.message = f'TOO LOW ({self.agl:.1f} m AGL) - back to DRONE'

        elif self.state == self.T_MC:
            self.tilt_cmd -= self.P['tilt_rate_back'] * dt
            self.throttle = clamp(0.30 * self.tilt_cmd / tmax, 0.0, 0.30)
            if self.tilt_cmd <= 0.02:
                self.tilt_cmd, self.throttle = 0.0, 0.0
                self.state = self.MC
                self.message = 'DRONE mode'

        self.tilt_cmd = clamp(self.tilt_cmd, 0.0, tmax)

    # ----------------------------------------------------------- setpoints ---
    def compute_setpoints(self, dt):
        """Pilot sticks (aerospace signs) -> URDF-frame attitude setpoints."""
        s_roll, s_pitch, s_yaw, s_power = self.sticks
        beta = self.beta()
        wing_borne = 1.0 - beta          # 0 in hover, 1 once the wing has it

        max_bank = (1.0 - wing_borne) * self.P['max_bank_mc'] + \
            wing_borne * self.P['max_bank_fw']
        max_pitch = (1.0 - wing_borne) * self.P['max_pitch_mc'] + \
            wing_borne * self.P['max_pitch_fw']
        max_yaw = (1.0 - wing_borne) * self.P['max_yaw_rate_mc'] + \
            wing_borne * self.P['max_yaw_rate_fw']

        roll_sp = s_roll * max_bank                # +roll right, same sign
        yaw_rate_sp = -s_yaw * max_yaw             # +stick = yaw right = -r
        pitch_stick = -s_pitch * max_pitch         # +stick = nose up = -pitch

        # Once the wing carries the aircraft, altitude is held with PITCH, not
        # with thrust, so a centred stick means "hold this altitude" and a
        # deflected stick is a direct attitude command.
        if abs(s_pitch) > 0.05:
            pitch_wing = pitch_stick
            self.alt_sp = self.pos[2]
        else:
            vz_sp = clamp(self.P['fw_alt_p'] * (self.alt_sp - self.pos[2]),
                          -self.P['fw_max_climb'], self.P['fw_max_climb'])
            pitch_wing = -clamp(self.P['fw_alt_pitch_p'] * (vz_sp - self.vel[2]),
                                -self.P['fw_pitch_limit'], self.P['fw_pitch_limit'])
        pitch_sp = (1.0 - wing_borne) * pitch_stick + wing_borne * pitch_wing

        # Stall guard: bleed off commanded nose-up as the margin disappears.
        if pitch_sp < 0.0 and self.tilt_cmd > 0.7:
            pitch_sp *= clamp((self.airspeed - af.V_STALL * 1.05) / 4.0, 0.0, 1.0)

        if self.state == self.FW:
            self.v_target = clamp(
                self.v_target + s_power * self.P['v_target_slew'] * dt,
                self.P['v_target_min'], self.P['v_target_max'])
            self.airspeed_hold(dt)
        return roll_sp, pitch_sp, yaw_rate_sp

    def airspeed_hold(self, dt):
        """PI airspeed hold on the front pair.

        The aero model has no parasite drag - cd is cda*alpha and nothing else -
        so drag FALLS as the aircraft speeds up and a fixed throttle has no
        stable trim speed. Holding airspeed instead of power is what makes
        AIRPLANE mode flyable, and it also protects both ends of the envelope:
        the integrator backs the power off in a dive and pushes it up near
        stall, without the pilot having to manage a lever.
        """
        err = self.v_target - self.airspeed
        cand = clamp(self.thr_i + self.P['speed_i'] * err * dt, 0.0, 1.0)
        raw = self.P['speed_p'] * err + cand
        if 0.0 < raw < 1.0:
            self.thr_i = cand
        self.throttle = clamp(self.P['speed_p'] * err + self.thr_i, 0.0, 1.0)

    # ------------------------------------------------------------ attitude ---
    def attitude_loop(self, roll_sp, pitch_sp, yaw_rate_sp, dt):
        p_sp = clamp(self.P['att_p_roll'] * wrap_pi(roll_sp - self.roll),
                     -self.P['max_rate_roll'], self.P['max_rate_roll'])
        q_sp = clamp(self.P['att_p_pitch'] * wrap_pi(pitch_sp - self.pitch),
                     -self.P['max_rate_pitch'], self.P['max_rate_pitch'])
        r_sp = clamp(yaw_rate_sp, -self.P['max_rate_yaw'], self.P['max_rate_yaw'])

        # In cruise a bank should turn the aircraft, so feed the coordinated turn
        # rate in rather than hold heading against the bank. +Mz is nose left, so
        # a right bank needs a negative yaw rate.
        if self.tilt_cmd > 0.7 and self.airspeed > af.V_STALL:
            turn = (af.GRAVITY * math.tan(clamp(self.roll, -1.2, 1.2))
                    / max(self.airspeed, 1.0))
            r_sp = clamp(r_sp - turn, -self.P['max_rate_yaw'], self.P['max_rate_yaw'])

        mx = af.IXX * self.pid_roll.update(p_sp, self.p_body, dt)
        my = af.IYY * self.pid_pitch.update(q_sp, self.q_body, dt)
        mz = af.IZZ * self.pid_yaw.update(r_sp, self.r_body, dt)
        return (clamp(mx, -self.P['moment_limit_x'], self.P['moment_limit_x']),
                clamp(my, -self.P['moment_limit_y'], self.P['moment_limit_y']),
                clamp(mz, -self.P['moment_limit_z'], self.P['moment_limit_z']))

    # ------------------------------------------------------------ vertical ---
    def vertical_loop(self, dt):
        """Body-z force wanted from the propulsion, in newtons."""
        if self.auto == 'TAKEOFF':
            vz_sp = clamp(self.P['alt_p'] * (self.alt_sp - self.pos[2]),
                          -1.0, self.P['takeoff_climb_rate'])
        elif self.auto == 'LAND':
            # Descend fast while there is height to lose, slowly near the
            # ground: a fixed rate makes a 56 m approach take almost a minute.
            vz_sp = -clamp(self.P['land_descent_slope'] * self.agl,
                           self.P['land_descent_min'],
                           self.P['land_descent_rate'])
        elif abs(self.sticks[3]) > 0.03 and self.state != self.FW:
            vz_sp = self.sticks[3] * self.P['max_climb_rate']
            self.alt_sp = self.pos[2]
        else:
            vz_sp = clamp(self.P['alt_p'] * (self.alt_sp - self.pos[2]),
                          -self.P['max_climb_rate'], self.P['max_climb_rate'])

        az = clamp(self.pid_vz.update(vz_sp, self.vel[2], dt),
                   -self.P['accel_limit_down'], self.P['accel_limit_up'])

        # Feed the wing's contribution forward. Without this the integrator has
        # to unwind ~50 N of lift during the transition and the aircraft balloons.
        lift = af.lift_estimate(self.airspeed, self.alpha)
        lift_vertical = lift * math.cos(self.roll) * math.cos(self.pitch)
        needed = af.WEIGHT + af.MASS * az - lift_vertical
        attitude_loss = max(math.cos(self.roll) * math.cos(self.pitch), 0.45)
        return clamp(needed / attitude_loss, 0.0, 3.0 * af.F_MAX * 0.98)

    # ---------------------------------------------------------- allocation ---
    def allocate(self, fz_dem, mx_dem, my_dem, mz_dem):
        tilt = self.tilt_cmd
        c, s = math.cos(tilt), math.sin(tilt)
        beta = self.beta()

        # Split every moment demand between rotors and surfaces in proportion to
        # the authority each has right now. The two shares always sum to the
        # full demand, so there is no dead band in the middle of a transition.
        k_aero = (clamp((self.airspeed / self.P['v_aero_full']) ** 2, 0.0, 1.0)
                  if self.P['publish_surfaces'] else 0.0)

        def split(dem, k_rot):
            n = k_rot + k_aero + 1e-6
            return dem * k_rot / n, dem * k_aero / n

        mx_rot, mx_aero = split(mx_dem, c * c)     # differential front thrust
        my_rot, my_aero = split(my_dem, beta)      # rear rotor thrust
        mz_rot, mz_aero = split(mz_dem, c * c)     # rear rotor tilt

        # --- front pair: lift in hover, thrust in cruise, blended on beta ----
        f_lift = fz_dem * FRONT_LIFT_SHARE / max(c, 0.35)
        f_thrust = self.throttle * 2.0 * af.F_MAX
        f_pair = clamp(beta * f_lift + (1.0 - beta) * f_thrust, 0.0, 2.0 * af.F_MAX)

        # --- rear rotor: solve PITCH exactly for this f_pair -----------------
        # my = f_pair*(Z_FRONT*s - X_FRONT*c) + f_rear*(-X_REAR).  The f_rear
        # coefficient is the constant 0.5358, so this never goes singular -
        # unlike solving pitch and heave together, which does at 84 deg.
        pitch_per_front = af.Z_FRONT * s - af.X_FRONT * c
        f_rear = clamp((my_rot - f_pair * pitch_per_front) / (-af.X_REAR),
                       0.0, af.F_MAX)

        # --- differential front thrust for roll ------------------------------
        delta = clamp(mx_rot / (af.Y_FRONT * max(c, 0.15)),
                      -2.0 * af.F_MAX, 2.0 * af.F_MAX)
        f_left = clamp(0.5 * (f_pair + delta), 0.0, af.F_MAX)
        f_right = clamp(0.5 * (f_pair - delta), 0.0, af.F_MAX)

        # --- rear rotor tilt: the tricopter yaw actuator ---------------------
        if f_rear > 1.0:
            rear_tilt = math.asin(clamp(mz_rot / (-af.X_REAR * f_rear), -0.999, 0.999))
        else:
            rear_tilt = 0.0
        rear_tilt = clamp(rear_tilt, -af.REAR_TILT_MAX, af.REAR_TILT_MAX)

        # --- aerodynamic surfaces --------------------------------------------
        q_dyn = af.dynamic_pressure(self.airspeed)
        if q_dyn > 1.0:
            ail = clamp(mx_aero / (af.AIL_MX_PER_Q * q_dyn), -af.AIL_LIMIT, af.AIL_LIMIT)
            ele = clamp(my_aero / (af.ELE_MY_PER_Q * q_dyn), -af.RV_LIMIT, af.RV_LIMIT)
            rud = clamp(mz_aero / (af.RUD_MZ_PER_Q * q_dyn), -af.RV_LIMIT, af.RV_LIMIT)
        else:
            ail = ele = rud = 0.0

        # V-tail mixing, elevator first: pitch matters more than the rudder.
        head = af.RV_LIMIT - abs(ele)
        rud = clamp(rud, -head, head)

        self.send_motors(f_left, f_right, f_rear)
        self.send_servos(tilt, rear_tilt, -ail, ail,
                         clamp(ele + rud, -af.RV_LIMIT, af.RV_LIMIT),
                         clamp(ele - rud, -af.RV_LIMIT, af.RV_LIMIT))
        self._dbg = (f_left, f_right, f_rear, beta, ele, ail)

    # -------------------------------------------------------------- output ---
    def send_motors(self, f_left, f_right, f_rear):
        self._motor_decim = (self._motor_decim + 1) % 2
        if self._motor_decim:
            return
        msg = Actuators()
        msg.header.stamp = self.get_clock().now().to_msg()
        idle = af.W_IDLE if self.armed else 0.0
        msg.velocity = [max(af.thrust_to_speed(f_left), idle),
                        max(af.thrust_to_speed(f_right), idle),
                        max(af.thrust_to_speed(f_rear), idle)]
        self.pub_motors.publish(msg)

    def send_servos(self, tilt, rear_tilt, ail_l, ail_r, rv_l, rv_r):
        self._servo_decim = (self._servo_decim + 1) % 4
        if self._servo_decim:
            return
        outputs = [('tilt_left', tilt), ('tilt_right', tilt),
                   ('rear_tilt', rear_tilt)]
        if self.P['publish_surfaces']:
            outputs += [('aileron_left', ail_l), ('aileron_right', ail_r),
                        ('ruddervator_left', rv_l), ('ruddervator_right', rv_r)]
        for name, value in outputs:
            self.pub_servo[name].publish(Float64(data=float(value)))

    def publish_status(self):
        if not (self.have_imu and self.have_odom):
            self.pub_status.publish(String(data='waiting for simulator...'))
            return
        d = self._dbg
        self.pub_status.publish(String(data=(
            f"{'ARMED ' if self.armed else 'SAFE  '}"
            f"| {self.state:<11s}"
            f"| tilt {math.degrees(self.tilt_cmd):5.1f}d"
            f"| alt {self.agl:6.1f}m"
            f"| V {self.airspeed:5.1f}m/s"
            f"| vz {self.vel[2]:+5.1f}"
            f"| rol {math.degrees(self.roll):+5.1f} pit {math.degrees(-self.pitch):+5.1f}"
            f"| aoa {math.degrees(self.alpha):+5.1f}"
            f"| thr {self.throttle * 100:3.0f}%"
            f"{f' -> {self.v_target:4.1f}m/s' if self.state == self.FW else ''}"
            f"| rotors {d[0]:4.1f}/{d[1]:4.1f}/{d[2]:4.1f}N"
            f"| ele {math.degrees(d[4]):+5.1f}"
            f"\n{self.message}")))


def main():
    rclpy.init()
    node = FlightController()
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
