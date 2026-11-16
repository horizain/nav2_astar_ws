#!/usr/bin/env python3
"""驱动 TurtleBot3 走一个方形环路（用于录制 SLAM 对拍用的 bag）。

用法（仿真已在跑，且 use_sim_time 生效）:
    ros2 run mini_slam drive_square.py            # 或
    python3 drive_square.py --side-time 12 --speed 0.2
"""
import argparse

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class DriveSquare(Node):
    def __init__(self, side_time: float, speed: float, turn_time: float):
        super().__init__("drive_square")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.side_time = side_time
        self.speed = speed
        self.turn_time = turn_time
        self.start = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f"开始走方形: 边 {side_time}s @ {speed}m/s, 转弯 {turn_time}s")

    def tick(self):
        elapsed = (self.get_clock().now() - self.start).nanoseconds / 1e9
        cycle = self.side_time + self.turn_time
        phase = elapsed % cycle
        cmd = Twist()
        if phase < self.side_time:
            cmd.linear.x = self.speed
        else:
            cmd.angular.z = 1.2  # ~1.3s 转 90°
        self.pub.publish(cmd)
        total_cycles = int(elapsed // cycle) + 1
        if total_cycles > 6:  # 6 条边 ≈ 1.5 圈，保证重访
            self.get_logger().info("走完收工")
            self.pub.publish(Twist())
            raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side-time", type=float, default=10.0)
    ap.add_argument("--turn-time", type=float, default=2.0)
    ap.add_argument("--speed", type=float, default=0.2)
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = DriveSquare(args.side_time, args.speed, args.turn_time)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
