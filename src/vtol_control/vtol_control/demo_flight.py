#!/usr/bin/env python3
"""Scripted pilot: flies the full mission with no keyboard.

    ros2 run vtol_control demo_flight

Sends exactly the same messages keyboard_teleop sends, so it exercises the real
control path rather than a back door. It is the regression test for the
controller: arm, auto take-off, hover, transition to AIRPLANE, a couple of
turns, back-transition, auto land, disarm. Anything that breaks the transition
schedule shows up here as a failed stage rather than as a bad feeling in the
cockpit.

    --csv FILE    write telemetry for plotting
    --quiet       stage transitions only
"""

import argparse
import sys

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String

from . import airframe as af


class Stage:
    """One mission step.

    `invariant` is the part that matters: without it a stage whose `done`
    condition is "mode == DRONE" passes trivially after the aircraft has crashed
    and the safety logic has dropped it back to DRONE. Each stage asserts what
    must stay true while it runs, so a crash is a failure and not a pass.
    """

    def __init__(self, name, action=None, done=None, timeout=60.0, sticks=None,
                 invariant=None, settle=0.0, repeat=False):
        self.name = name
        self.action = action
        self.done = done
        self.timeout = timeout
        self.sticks = sticks or (0.0, 0.0, 0.0, 0.0)
        self.invariant = invariant
        self.settle = settle
        # Re-send the action every tick. Needed for the arm request, which is
        # otherwise dropped: the publisher is only milliseconds old on the first
        # tick and DDS discovery has not matched the controller yet. Only safe
        # for idempotent requests - 'takeoff' would keep resetting the target.
        self.repeat = repeat


class DemoPilot(Node):

    def __init__(self, csv_path=None, quiet=False):
        super().__init__('vtol_demo_flight')
        self.pub_input = self.create_publisher(Twist, '/vtol/manual_input', 10)
        self.pub_mode = self.create_publisher(String, '/vtol/mode_request', 10)
        self.pub_arm = self.create_publisher(Bool, '/vtol/arm_request', 10)
        self.create_subscription(Odometry, '/vtol/odom', self.on_odom, 10)
        self.create_subscription(Imu, '/vtol/imu', self.on_imu, 10)
        self.create_subscription(String, '/vtol/status', self.on_status, 10)

        self.alt = 0.0
        self.speed = 0.0
        self.vz = 0.0
        self.roll = self.pitch = 0.0
        self.status = ''
        self.mode = ''
        self.ready = False
        self.quiet = quiet
        self.failures = []

        self.csv = open(csv_path, 'w') if csv_path else None
        if self.csv:
            self.csv.write('t,stage,mode,alt,speed,vz,roll_deg,pitch_deg\n')

        self.stages = self.build_mission()
        self.index = 0
        self.t0 = None
        self.stage_t0 = None
        self.entered = False
        self.mark_alt = 0.0

        self.create_timer(0.05, self.tick)

    # --------------------------------------------------------------- mission --
    def build_mission(self):
        upright = lambda s: abs(s.roll) < 75.0 and abs(s.pitch) < 60.0
        airborne = lambda s: s.alt > 8.0
        flying = lambda s: upright(s) and airborne(s)
        cruising = lambda s: s.mode == 'AIRPLANE' and flying(s)

        return [
            Stage('wait for sim',
                  done=lambda s: s.ready and s.status != '', timeout=60.0),
            Stage('arm', action=lambda s: s.pub_arm.publish(Bool(data=True)),
                  done=lambda s: 'ARMED' in s.status, timeout=15.0, repeat=True),
            Stage('auto take-off to 40 m',
                  action=lambda s: s.pub_mode.publish(String(data='takeoff')),
                  done=lambda s: s.alt > 39.0, timeout=45.0,
                  invariant=upright, settle=1.0),
            Stage('hold hover 6 s', done=lambda s: s.stage_elapsed > 6.0,
                  timeout=10.0, invariant=flying),
            Stage('hover: hold altitude within 1 m',
                  done=lambda s: s.stage_elapsed > 4.0, timeout=8.0,
                  invariant=lambda s: flying(s) and abs(s.alt - s.mark_alt) < 1.0),
            Stage('yaw right in hover', sticks=(0.0, 0.0, 0.6, 0.0),
                  done=lambda s: s.stage_elapsed > 4.0, timeout=8.0,
                  invariant=flying),
            Stage('translate forward in hover', sticks=(0.0, -0.7, 0.0, 0.0),
                  done=lambda s: s.stage_elapsed > 4.0, timeout=8.0,
                  invariant=flying),
            Stage('transition to AIRPLANE',
                  action=lambda s: s.pub_mode.publish(String(data='fw')),
                  done=lambda s: s.mode == 'AIRPLANE', timeout=60.0,
                  invariant=flying, settle=1.0),
            Stage('cruise straight 10 s', done=lambda s: s.stage_elapsed > 10.0,
                  timeout=14.0, invariant=cruising),
            Stage('bank right 7 s', sticks=(0.7, 0.0, 0.0, 0.0),
                  done=lambda s: s.stage_elapsed > 7.0, timeout=11.0,
                  invariant=cruising),
            Stage('bank left 7 s', sticks=(-0.7, 0.0, 0.0, 0.0),
                  done=lambda s: s.stage_elapsed > 7.0, timeout=11.0,
                  invariant=cruising),
            Stage('climb in cruise', sticks=(0.0, 0.5, 0.0, 0.0),
                  done=lambda s: s.stage_elapsed > 6.0, timeout=10.0,
                  invariant=cruising),
            Stage('back-transition to DRONE',
                  action=lambda s: s.pub_mode.publish(String(data='mc')),
                  done=lambda s: s.mode == 'DRONE', timeout=30.0,
                  invariant=flying, settle=1.0),
            Stage('settle in hover 6 s', done=lambda s: s.stage_elapsed > 6.0,
                  timeout=10.0, invariant=flying),
            Stage('auto land',
                  action=lambda s: s.pub_mode.publish(String(data='land')),
                  done=lambda s: 'SAFE' in s.status, timeout=90.0,
                  invariant=upright),
        ]

    # ----------------------------------------------------------------- state --
    def on_odom(self, msg):
        self.alt = msg.pose.pose.position.z - 0.24
        self.ready = True

    def on_imu(self, msg):
        pass

    def on_status(self, msg):
        self.status = msg.data.split('\n')[0]
        fields = [f.strip() for f in self.status.split('|')]
        for f in fields:
            if f in ('DRONE', 'AIRPLANE') or f.startswith('TRANS'):
                self.mode = f
            elif f.startswith('V '):
                try:
                    self.speed = float(f[2:].replace('m/s', ''))
                except ValueError:
                    pass
            elif f.startswith('vz '):
                try:
                    self.vz = float(f[3:])
                except ValueError:
                    pass
            elif f.startswith('rol '):
                parts = f.split()
                try:
                    self.roll = float(parts[1])
                    self.pitch = float(parts[3])
                except (IndexError, ValueError):
                    pass

    @property
    def stage_elapsed(self):
        return 0.0 if self.stage_t0 is None else self.now - self.stage_t0

    @property
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ loop --
    def tick(self):
        if self.t0 is None:
            self.t0 = self.now
        if self.index >= len(self.stages):
            self.finish()
            return

        stage = self.stages[self.index]
        first = not self.entered
        if first:
            self.stage_t0 = self.now
            self.entered = True
            self.mark_alt = self.alt
            self.log(f'--> {stage.name}')
        if stage.action and (first or stage.repeat):
            stage.action(self)

        msg = Twist()
        msg.linear.y, msg.linear.x, msg.angular.z, msg.linear.z = stage.sticks
        self.pub_input.publish(msg)

        if self.csv:
            self.csv.write(f'{self.now - self.t0:.2f},{stage.name},{self.mode},'
                           f'{self.alt:.2f},{self.speed:.2f},{self.vz:.2f},'
                           f'{self.roll:.1f},{self.pitch:.1f}\n')
        if not self.quiet and int(self.stage_elapsed * 2) != getattr(self, '_last_p', -1):
            self._last_p = int(self.stage_elapsed * 2)
            print(f'  [{self.now - self.t0:6.1f}s] {self.mode:<11s} '
                  f'alt {self.alt:6.1f} m  V {self.speed:5.1f} m/s  '
                  f'vz {self.vz:+5.1f}  roll {self.roll:+6.1f}  pitch {self.pitch:+6.1f}')

        # Invariants are checked after a settling window, so a stage that starts
        # mid-manoeuvre is not failed by the manoeuvre it was entered from.
        if (stage.invariant and self.stage_elapsed > stage.settle
                and not stage.invariant(self)):
            self.failures.append(
                f'{stage.name}: invariant violated at t={self.now - self.t0:.1f}s '
                f'(mode={self.mode} alt={self.alt:.1f} V={self.speed:.1f} '
                f'roll={self.roll:.0f} pitch={self.pitch:.0f})')
            self.log('    FAILED (invariant)')
            self.advance()
        elif stage.done and stage.done(self):
            self.log(f'    ok  ({self.stage_elapsed:.1f} s)')
            self.advance()
        elif self.stage_elapsed > stage.timeout:
            self.failures.append(f'{stage.name}: timed out after {stage.timeout:.0f} s '
                                 f'(mode={self.mode} alt={self.alt:.1f} V={self.speed:.1f})')
            self.log(f'    FAILED (timeout)')
            self.advance()

    def advance(self):
        self.index += 1
        self.entered = False
        self._last_p = -1

    def log(self, text):
        print(text, flush=True)

    def finish(self):
        if self.csv:
            self.csv.close()
        print()
        if self.failures:
            print(f'MISSION INCOMPLETE - {len(self.failures)} stage(s) failed:')
            for f in self.failures:
                print('  * ' + f)
        else:
            print(f'MISSION COMPLETE - all {len(self.stages)} stages passed. '
                  f'Stall speed was {af.V_STALL:.1f} m/s.')
        raise SystemExit(1 if self.failures else 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv')
    ap.add_argument('--quiet', action='store_true')
    args, rest = ap.parse_known_args()

    rclpy.init(args=rest)
    node = DemoPilot(args.csv, args.quiet)
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        code = exc.code or 0
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
