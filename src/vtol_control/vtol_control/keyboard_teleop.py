#!/usr/bin/env python3
"""Keyboard pilot for the tri-motor tiltrotor VTOL.

Run this in its own terminal - it needs a tty, so it cannot be started from a
launch file that swallows stdin:

    ros2 run vtol_control keyboard_teleop

Key handling
------------
A terminal gives you key *events*, not key state: one event on press, then a
burst of auto-repeats after the repeat delay, and nothing at all on release.
So each event nudges its axis toward full deflection and an axis with no event
for 0.22 s decays back to centre.  Tapping gives a small correction, holding
builds to full travel and releasing self-centres in about 0.4 s.  That reads as
a spring-centred stick rather than the on/off bang-bang you get from naive
keyboard teleop.

The throttle axis behaves the same way, but the flight controller integrates it
into a throttle lever in AIRPLANE mode, so a held key ramps power and a release
leaves it where it was.
"""

import atexit
import math
import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

HELP = """
\033[1m======================= VTOL TILTROTOR - KEYBOARD PILOT =======================\033[0m

  \033[1mSPACE\033[0m   toggle  \033[36mDRONE\033[0m <-> \033[33mAIRPLANE\033[0m   (needs 12 m altitude to go out)
  \033[1mENTER\033[0m   arm / disarm            \033[1mX\033[0m  kill motors immediately

  \033[1mW / S\033[0m   pitch    nose down / nose up
            DRONE: W flies forward, S flies backward
            AIRPLANE: it is a stick - W dives, S climbs
  \033[1mA / D\033[0m   roll     left / right
  \033[1mQ / E\033[0m   yaw      left / right   (DRONE: tail rotor tilts, AIRPLANE: rudder)
  \033[1mR / F\033[0m   power    DRONE: climb / descend      AIRPLANE: throttle up / down
            arrow Up/Down do the same as R/F

  \033[1mT\033[0m  auto take-off to 15 m      \033[1mL\033[0m  auto land
  \033[1mH\033[0m  level off and hold altitude \033[1mZ\033[0m  centre all sticks
  \033[1m?\033[0m  reprint this help          \033[1mCtrl-C\033[0m  quit

  Typical flight: ENTER, T, wait for the climb, SPACE, fly, SPACE, L.
\033[1m===============================================================================\033[0m
"""

# axis index -> (negative key, positive key)
AXES = {
    0: ('a', 'd'),        # roll   : + right
    1: ('w', 's'),        # pitch  : + nose up
    2: ('q', 'e'),        # yaw    : + right
    3: ('f', 'r'),        # power  : + up
}

STEP = 0.34          # deflection added per key event
DECAY = 2.6          # per second, once the key stops repeating
IDLE = 0.22          # s without an event before an axis starts centring


class KeyboardPilot(Node):

    def __init__(self):
        super().__init__('vtol_keyboard_teleop')
        self.pub_input = self.create_publisher(Twist, '/vtol/manual_input', 10)
        self.pub_mode = self.create_publisher(String, '/vtol/mode_request', 10)
        self.pub_arm = self.create_publisher(Bool, '/vtol/arm_request', 10)
        self.create_subscription(String, '/vtol/status', self.on_status, 10)

        self.axis = [0.0, 0.0, 0.0, 0.0]
        self.last_evt = [0.0, 0.0, 0.0, 0.0]
        self.armed_request = False
        self.hud = 'waiting for /vtol/status ...'
        self.message = ''
        self.shown_message = None

        self.key_to_axis = {}
        for idx, (neg, pos) in AXES.items():
            self.key_to_axis[neg] = (idx, -1.0)
            self.key_to_axis[pos] = (idx, +1.0)

        self.fd = sys.stdin.fileno()
        self.tty_ok = os.isatty(self.fd)
        self.saved = None
        if self.tty_ok:
            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)      # cbreak keeps ISIG, so Ctrl-C still works
            atexit.register(self.restore_tty)
        else:
            self.get_logger().error(
                'stdin is not a terminal. Start this node directly in a terminal '
                '("ros2 run vtol_control keyboard_teleop"), not from a launch file.')

        sys.stdout.write(HELP)
        sys.stdout.flush()
        self.dt = 0.02
        self.create_timer(self.dt, self.tick)

    def restore_tty(self):
        if self.saved is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
            self.saved = None
            sys.stdout.write('\n')
            sys.stdout.flush()

    # ------------------------------------------------------------------ io --
    def read_keys(self):
        keys = []
        while self.tty_ok and select.select([sys.stdin], [], [], 0)[0]:
            ch = os.read(self.fd, 1).decode('utf-8', 'ignore')
            if not ch:
                break
            if ch == '\x1b':
                # Arrow keys arrive as ESC [ A/B/C/D.
                seq = ''
                while select.select([sys.stdin], [], [], 0.002)[0] and len(seq) < 2:
                    seq += os.read(self.fd, 1).decode('utf-8', 'ignore')
                keys.append({'[A': 'r', '[B': 'f', '[C': 'd', '[D': 'a'}.get(seq, ''))
            else:
                keys.append(ch)
        return [k for k in keys if k]

    def on_status(self, msg: String):
        parts = msg.data.split('\n', 1)
        self.hud = parts[0]
        if len(parts) > 1:
            self.message = parts[1]

    # ---------------------------------------------------------------- tick --
    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        for key in self.read_keys():
            self.handle_key(key, now)

        for i in range(4):
            if now - self.last_evt[i] > IDLE and self.axis[i] != 0.0:
                mag = max(0.0, abs(self.axis[i]) - DECAY * self.dt)
                self.axis[i] = math.copysign(mag, self.axis[i]) if mag else 0.0

        msg = Twist()
        msg.linear.y = self.axis[0]      # roll  +right
        msg.linear.x = self.axis[1]      # pitch +nose up
        msg.angular.z = self.axis[2]     # yaw   +right
        msg.linear.z = self.axis[3]      # power +up
        self.pub_input.publish(msg)
        self.draw()

    def handle_key(self, key, now):
        low = key.lower()
        if key == ' ':
            self.pub_mode.publish(String(data='toggle'))
            return
        if key in ('\r', '\n'):
            self.armed_request = not self.armed_request
            self.pub_arm.publish(Bool(data=self.armed_request))
            return
        if low in self.key_to_axis:
            idx, sign = self.key_to_axis[low]
            if self.axis[idx] * sign < 0.0:
                self.axis[idx] = 0.0          # reversing centres first
            self.axis[idx] = max(-1.0, min(1.0, self.axis[idx] + sign * STEP))
            self.last_evt[idx] = now
            return
        if low == 'x':
            self.armed_request = False
            self.pub_mode.publish(String(data='kill'))
        elif low == 't':
            self.armed_request = True
            self.pub_mode.publish(String(data='takeoff'))
        elif low == 'l':
            self.pub_mode.publish(String(data='land'))
        elif low == 'h':
            self.axis = [0.0, 0.0, 0.0, 0.0]
            self.pub_mode.publish(String(data='hold'))
        elif low == 'z':
            self.axis = [0.0, 0.0, 0.0, 0.0]
        elif key == '?':
            sys.stdout.write('\r\033[2K' + HELP)

    # ---------------------------------------------------------------- view --
    def draw(self):
        if self.message != self.shown_message:
            sys.stdout.write(f'\r\033[2K\033[36m>> {self.message}\033[0m\n')
            self.shown_message = self.message
        sticks = ' '.join(
            f'{name}{self.bar(self.axis[i])}'
            for i, name in enumerate(('rol', 'pit', 'yaw', 'pwr')))
        sys.stdout.write(f'\r\033[2K{self.hud} | {sticks}')
        sys.stdout.flush()

    @staticmethod
    def bar(value):
        n = int(round(abs(value) * 4))
        if n == 0:
            return '·'
        return ('<' if value < 0 else '>') * n


def main():
    rclpy.init()
    node = KeyboardPilot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_tty()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
