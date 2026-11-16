#!/usr/bin/env python3
"""Keyboard teleoperation with a short command timeout for manual mapping."""
import argparse
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/cmd_vel')
    parser.add_argument('--linear', type=float, default=0.15)
    parser.add_argument('--angular', type=float, default=0.7)
    parser.add_argument('--timeout', type=float, default=0.3,
                        help='stop after this many seconds without a motion key')
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error('run in an interactive terminal')
    if not (0 < args.linear <= 0.5 and 0 < args.angular <= 2.0
            and 0.1 <= args.timeout <= 1.0):
        parser.error('linear: (0, 0.5], angular: (0, 2.0], timeout: [0.1, 1.0]')

    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = Node('mini_slam_keyboard_teleop')
    publisher = node.create_publisher(Twist, args.topic, 10)
    linear, angular = args.linear, args.angular
    motion = {'w': (1, 0), 's': (-1, 0), 'a': (0, 1), 'd': (0, -1)}
    active = (0, 0)
    expires = 0.0
    print(f'Publishing Twist on {args.topic}')
    print('Hold W/S: forward/back; A/D: turn; space: stop; Q: quit')
    print('R/F: linear speed +/-; T/G: angular speed +/-')
    print('Motion stops when key repeats stop. Keep navigation stopped.')
    print(f'linear={linear:.2f} m/s angular={angular:.2f} rad/s')
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1).lower()
                if key in motion:
                    active = motion[key]
                    expires = time.monotonic() + args.timeout
                elif key in ('q', '\x03'):
                    break
                elif key == ' ':
                    active = (0, 0)
                elif key in 'rftg':
                    if key == 'r':
                        linear = min(0.5, linear + 0.05)
                    elif key == 'f':
                        linear = max(0.05, linear - 0.05)
                    elif key == 't':
                        angular = min(2.0, angular + 0.1)
                    else:
                        angular = max(0.1, angular - 0.1)
                    print(f'linear={linear:.2f} m/s angular={angular:.2f} rad/s')
            if time.monotonic() >= expires:
                active = (0, 0)
            msg = Twist()
            msg.linear.x = active[0] * linear
            msg.angular.z = active[1] * angular
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if rclpy.ok():
            publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.1)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()