#!/usr/bin/env python3
"""带激光安全停止的方形环路驾驶员（用于录制 SLAM 对拍 bag）。

策略：直行 → 原地转 90° → 直行 …… 转若干圈制造重访。
安全：正前方 ±40° 扇区最小距离 < 安全阈值时停住前进，改为原地转向
（在柱子密布的 tb3_sandbox 里不至于卡死）。
"""
import argparse
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SafeSquare(Node):
    def __init__(self, side_m: float, speed: float, laps: int):
        super().__init__("drive_safe_square")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan = None
        self.sub = self.create_subscription(
            LaserScan, "scan", self.on_scan, 10)
        self.side_m = side_m
        self.speed = speed
        self.laps = laps
        self.state = "forward"
        self.state_start = self.now()
        self.side_count = 0
        self.stuck_since = None
        self.done = False
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f"方形环路: 边长 {side_m}m @ {speed}m/s, 共 {laps} 圈")

    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def on_scan(self, msg: LaserScan):
        self.scan = msg

    def front_clear(self) -> bool:
        """正前方 ±40° 扇区最小距离是否安全（雷达 0 起始角 = 正前方）。

        没有扫描 / 扇区内没有任何有效距离时必须返回 False（保守停车）：
        默认"安全"会让机器人在传感器掉线时盲目前进。
        """
        if self.scan is None:
            return False
        best = float("inf")
        # scan 从 0 到 2π，取 0±40° 与 2π-40°..2π 两段
        n = len(self.scan.ranges)
        sector = int(40.0 / 360.0 * n)
        for i in list(range(0, sector + 1)) + list(range(n - sector, n)):
            r = self.scan.ranges[i]
            if math.isfinite(r) and r > 0.05:
                best = min(best, r)
        if not math.isfinite(best):
            return False   # 没有任何有效回波：不知道前方是否安全
        return best > 0.18

    def tick(self):
        if self.done:
            return
        cmd = Twist()
        t = self.now() - self.state_start

        if self.state == "forward":
            cmd.linear.x = self.speed
            if not self.front_clear():
                # 前方太近：停住并原地转向避开
                if self.stuck_since is None:
                    self.stuck_since = self.now()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.8
                if self.now() - self.stuck_since > 6.0:
                    self.get_logger().warn("卡住超时，提前收工")
                    self.finish()
                    return
            else:
                self.stuck_since = None
            if t * self.speed >= self.side_m and self.front_clear():
                self.enter("turn")
        elif self.state == "turn":
            cmd.angular.z = 0.8
            # 90° 需要 ~1.96s（0.8 rad/s），留点余量到 2.1s
            if t >= 2.1:
                self.side_count += 1
                self.get_logger().info(f"边 {self.side_count}/{self.laps * 4}")
                if self.side_count >= self.laps * 4:
                    self.finish()
                    return
                self.enter("forward")
        self.pub.publish(cmd)

    def enter(self, state: str):
        self.state = state
        self.state_start = self.now()

    def stop(self):
        self.pub.publish(Twist())  # 停车

    def finish(self):
        # 不在 timer 回调里 raise SystemExit：那会跳过 spin 的正常收尾。
        # 置 done 让 tick 立即返回，由 main 的循环退出后统一停车/销毁。
        self.stop()
        self.done = True
        self.get_logger().info("走完收工")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", type=float, default=0.8)
    ap.add_argument("--speed", type=float, default=0.12)
    ap.add_argument("--laps", type=int, default=4)
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = SafeSquare(args.side, args.speed, args.laps)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
