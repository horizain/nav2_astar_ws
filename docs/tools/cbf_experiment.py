#!/usr/bin/env python3
"""CBF 安全实验：同一标称指令"裸奔 vs 过滤"的行为对比（docs/17 §7）

场景：机器人面向 ~0.4m 外的圆柱，以恒定 0.2 m/s 直冲。

两种模式：
  --mode baseline  指令直接发到 /cmd_vel（绕过整条链）→ 预期：撞上
  --mode cbf       指令发到 /cmd_vel_smoothed（经 CBF 过滤器）→ 预期：平滑停住

记录：t、机器人位置、最近障碍 h = range - d_safe、过滤器实际输出。
日志打印到 stdout，重定向存档即可。

前置：仿真与 nav2 栈在运行、导航已完成（cmd_vel_smoothed 无其他发布者）、
      机器人前方有障碍。撞圆柱在仿真里无害。
"""
import argparse
import math
import os
import time

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class CbfExperiment(Node):

    def __init__(self, mode, speed, duration):
        super().__init__('cbf_experiment')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])
        self.mode = mode
        self.speed = speed
        self.duration = duration
        self.scan = None
        self.odom = None
        self.filtered = None
        self.records = []
        # baseline 直捣底盘；cbf 走标称入口
        out_topic = '/cmd_vel' if mode == 'baseline' else '/cmd_vel_smoothed'
        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)
        self.create_subscription(Odometry, 'odom', self._odom_cb, 10)
        if mode == 'cbf':
            self.create_subscription(Twist, 'cmd_vel_cbf', self._filtered_cb, 10)

    def _scan_cb(self, m):
        rs = [(r, m.angle_min + i * m.angle_increment)
              for i, r in enumerate(m.ranges)
              if math.isfinite(r) and r > m.range_min and r < m.range_max]
        self.scan = min(rs)[0] if rs else None  # 最近障碍距离（含各方向）

    def _odom_cb(self, m):
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y)

    def _filtered_cb(self, m):
        self.filtered = m.linear.x

    def run(self):
        self.get_logger().info(f'模式={self.mode} 速度={self.speed} 时长={self.duration}s')
        t0 = time.time()
        next_log = 0.0
        while time.time() - t0 < self.duration and rclpy.ok():
            cmd = Twist()
            cmd.linear.x = self.speed
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.time() - t0
            if now >= next_log and self.scan is not None and self.odom:
                next_log = now + 0.5
                line = (f't={now:5.1f}  pos=({self.odom[0]:+.2f},{self.odom[1]:+.2f})  '
                        f'nearest_range={self.scan:.3f}  h={self.scan - 0.22:+.3f}')
                if self.mode == 'cbf' and self.filtered is not None:
                    line += f'  cbf_out={self.filtered:+.2f}'
                print(line, flush=True)
                self.records.append(line)
        stop = Twist()
        self.pub.publish(stop)
        rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info('实验结束（已停车）')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=['baseline', 'cbf'], default='cbf')
    ap.add_argument('--speed', type=float, default=0.2)
    ap.add_argument('--duration', type=float, default=12.0)
    ap.add_argument('--viz', action='store_true',
                    help='拉起实时可视化面板（地图/h(t)/速度对比）')
    args = ap.parse_args()

    rclpy.init()
    node = CbfExperiment(args.mode, args.speed, args.duration)
    viz_proc = None
    if args.viz:
        import subprocess
        viz = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           'live_monitor.py')
        viz_proc = subprocess.Popen(
            [sys.executable, viz, '--d-safe', '0.22'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print('[VIZ] 实时面板已启动')
    try:
        node.run()
    finally:
        rclpy.shutdown()
        if viz_proc is not None:
            viz_proc.terminate()


if __name__ == '__main__':
    main()
