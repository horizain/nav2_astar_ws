#!/usr/bin/env python3
"""自动探索建图：接触触发转向的随机覆盖行走（docs/20 SLAM 管线配套）

状态机（扫地机器人式）：
  DRIVE  前进；前方 ±35° 锥内最近障碍 < 0.35m → 转入 TURN
         转向方向自动避开更近的一侧（远离墙）
  TURN   原地旋转 ±90~180°；期间免疫卡住检测；转完回 DRIVE（3s 免疫期）
  卡住检测 仅 DRIVE 态且免疫期外生效：4 秒位移 < 5cm → 大角度脱困旋转
  周期环扫 连续 10s 无接触前进 → 原地慢转 360° 帮激光补全环境

直接发布 /cmd_vel（绕过导航链——SLAM 会话里 nav 栈空闲，无冲突；
本脚本自带避障逻辑，不依赖 CBF/碰撞监视器）。

用法（配合 slam:=True 的建图会话）：
  python3 docs/tools/auto_explorer.py --duration 150
"""
import argparse
import math
import random
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

DRIVE, TURN, SWEEP = 'DRIVE', 'TURN', 'SWEEP'


class AutoExplorer(Node):

    def __init__(self, duration, speed):
        super().__init__('auto_explorer')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])
        self.duration = duration
        self.speed = speed
        self.state = DRIVE
        self.scan = None            # 前方 ±35° 锥内最近距离
        self.scan_side = 0.0        # 左右最近距离之差（>0 左侧更空）
        self.odom = None
        self.last_pos = None
        self.total_dist = 0.0
        self.leg_dist = 0.0         # 当前 DRIVE 段的里程
        self.turn_remaining = 0.0
        self.immunity_until = 0.0   # 卡住检测免疫期（转完/脱困后给 3s）
        self.last_progress_t = time.time()
        self.sweep_deadline = time.time() + 10.0
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)
        self.create_subscription(Odometry, 'odom', self._odom_cb, 10)

    def _scan_cb(self, m):
        front, left, right = float('inf'), float('inf'), float('inf')
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or r <= m.range_min:
                continue
            a = math.degrees(m.angle_min + i * m.angle_increment)
            if abs(a) < 35:
                front = min(front, r)
            elif 35 <= a < 90:
                left = min(left, r)
            elif -90 < a <= -35:
                right = min(right, r)
        self.scan = front
        self.scan_side = (1e9 if left == float('inf') else left) - \
                         (1e9 if right == float('inf') else right)

    def _odom_cb(self, m):
        p = m.pose.pose.position
        if self.last_pos is not None:
            d = math.hypot(p.x - self.last_pos[0], p.y - self.last_pos[1])
            self.total_dist += d
            if self.state == DRIVE:
                self.leg_dist += d
            if d > 0.05:
                self.last_progress_t = time.time()
        self.last_pos = (p.x, p.y)
        self.odom = (p.x, p.y)

    def run(self):
        self.get_logger().info(f'探索开始：{self.duration}s，速度 {self.speed} m/s')
        t0 = time.time()
        last_log = 0.0
        while rclpy.ok() and time.time() - t0 < self.duration:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.time()
            wall_t = now - t0
            cmd = Twist()

            if self.state == DRIVE:
                # 卡住检测（免疫期外）
                if now > self.immunity_until and \
                        now - self.last_progress_t > 4.0:
                    self.turn_remaining = random.choice([-1, 1]) * math.radians(180)
                    self.state = TURN
                    self.immunity_until = now + 3.0
                    self.get_logger().warn(f'{wall_t:.0f}s 卡住 → 脱困旋转')
                elif self.scan is not None and self.scan < 0.35:
                    # 转向方向：远离更近的一侧
                    direction = -1 if self.scan_side > 0 else 1
                    angle = random.uniform(math.radians(90), math.radians(170))
                    self.turn_remaining = direction * angle
                    self.state = TURN
                    self.get_logger().info(
                        f'{wall_t:.0f}s 前方 {self.scan:.2f}m → 转 '
                        f'{math.degrees(self.turn_remaining):+.0f}°（避近侧）')
                elif self.leg_dist > 2.5 and now > self.immunity_until:
                    # 一段直行后主动环扫（补全激光视野）
                    self.state = SWEEP
                    self.leg_dist = 0.0
                else:
                    cmd.linear.x = self.speed

            if self.state == TURN:
                step = 0.9 * 0.05
                turn = math.copysign(min(abs(self.turn_remaining), step),
                                     self.turn_remaining)
                cmd.angular.z = turn
                self.turn_remaining -= turn
                if abs(self.turn_remaining) < 0.06:
                    self.state = DRIVE
                    self.leg_dist = 0.0
                    self.immunity_until = time.time() + 3.0
                    self.last_progress_t = time.time()

            if self.state == SWEEP:
                cmd.angular.z = 0.7
                self.turn_remaining -= 0.7 * 0.05
                if self.turn_remaining <= 0.0:
                    self.turn_remaining = random.uniform(
                        math.radians(90), math.radians(170))
                    self.state = TURN
                if self.scan is not None and self.scan < 0.6:
                    self.state = DRIVE  # 环扫中 proximity 障碍 → 立即回驱动

            if self.state == DRIVE and now < self.sweep_deadline:
                pass
            elif self.state == DRIVE:
                self.sweep_deadline = now + 10.0

            self.pub.publish(cmd)
            if wall_t - last_log > 2.0:
                last_log = wall_t
                print(f't={wall_t:5.0f}s  pos=({self.odom[0]:+.2f},{self.odom[1]:+.2f})  '
                      f'前方={self.scan if self.scan else 9:.2f}m  状态={self.state}  '
                      f'总里程={self.total_dist:.1f}m', flush=True)
            time.sleep(0.05)

        stop = Twist()
        self.pub.publish(stop)
        rclpy.spin_once(self, timeout_sec=0.2)
        print(f'探索结束：总里程 {self.total_dist:.1f}m')


def main():
    ap = argparse.ArgumentParser(description='自动探索建图（docs/20）')
    ap.add_argument('--duration', type=float, default=150.0, help='探索时长（秒）')
    ap.add_argument('--speed', type=float, default=0.22, help='前进速度（m/s）')
    args = ap.parse_args()

    rclpy.init()
    node = AutoExplorer(args.duration, args.speed)
    try:
        node.run()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
