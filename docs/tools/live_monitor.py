#!/usr/bin/env python3
"""实验实时可视化面板（独立进程，与 RViz 并存，headless 会话也能看）

三个视图：
  左  地图    ：静态地图 + 机器人轨迹 + A* 路径 + 目标点 + 激光点
  右上 h(t)   ：最近障碍屏障值曲线（h = 最近距离 - d_safe），红线 = 屏障 0
  右下 速度   ：标称指令（蓝）vs CBF 过滤输出（红）——CBF 拦截一目了然

用法（任选其一，都不影响被观察的脚本）：
  python3 docs/tools/live_monitor.py                     # 默认地图
  python3 docs/tools/live_monitor.py --d-safe 0.22       # 对齐 CBF 部署参数
  python3 docs/tools/live_monitor.py --save-shot /tmp/x.png --shot-after 8
                                                          # 8 秒后存快照（用于无头验证）
被观察脚本侧：e2e_nav_test.py --viz 会自动拉起本面板；也可手动另开终端跑。
"""
import argparse
import math
import os
import threading
import time

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def load_map(yaml_path):
    """读取 OccupancyGrid 栅格图（PGM + yaml），返回 (img, extent)"""
    import yaml
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)
    img_path = os.path.join(os.path.dirname(yaml_path), cfg['image'])
    with open(img_path, 'rb') as f:
        f.readline()
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        data = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
    # PGM 255=自由 0=占据 → 反相成"黑墙白地"，未知 205 → 灰
    res, ox, oy = cfg['resolution'], cfg['origin'][0], cfg['origin'][1]
    extent = [ox, ox + w * res, oy, oy + h * res]
    return data, extent


class LiveMonitor(Node):

    def __init__(self, args):
        super().__init__('live_monitor')
        self.d_safe = args.d_safe
        self.trail = ([], [])                    # 机器人轨迹（map 系）
        self.path = ([], [])                     # A* 全局路径
        self.goal = None                         # 最近一次导航目标
        self.scan_pts = ([], [])                 # 激光点（map 系）
        self.pose = None                         # (x, y, yaw)
        self.h_hist = ([], [])                   # h(t)
        self.v_hist = {'t': [], 'nom': [], 'cbf': []}
        self.t0 = time.time()

        qos_sensor = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                                reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose',
                                 self._amcl_cb, 10)
        self.create_subscription(Odometry, 'odom', self._odom_cb, qos_sensor)
        self.create_subscription(Path, 'plan', self._plan_cb, 10)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, qos_sensor)
        self.create_subscription(PoseStamped, 'goal_pose', self._goal_cb, 10)
        self.create_subscription(Twist, 'cmd_vel_smoothed', self._nom_cb, 10)
        self.create_subscription(Twist, 'cmd_vel_cbf', self._cbf_cb, 10)

    # ---------- 回调：只更新状态，画图放主线程 ----------
    def _amcl_cb(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        self.trail[0].append(p.x)
        self.trail[1].append(p.y)
        if len(self.trail[0]) > 4000:
            self.trail[0] = self.trail[0][::2]
            self.trail[1] = self.trail[1][::2]

    def _odom_cb(self, m):
        if self.pose is None:  # 无 AMCL 时退化为 odom 位姿
            p = m.pose.pose.position
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            self.pose = (p.x, p.y, yaw)

    def _plan_cb(self, m):
        self.path = ([p.pose.position.x for p in m.poses],
                     [p.pose.position.y for p in m.poses])

    def _scan_cb(self, m):
        self._latest_scan = m
        if self.pose is not None and m.ranges:
            rs = [(r, m.angle_min + i * m.angle_increment)
                  for i, r in enumerate(m.ranges)
                  if math.isfinite(r) and r > m.range_min and r < m.range_max]
            if rs:
                nearest = min(rs)[0]
                t = time.time() - self.t0
                self.h_hist[0].append(t)
                self.h_hist[1].append(nearest - self.d_safe)
                if len(self.h_hist[0]) > 3000:
                    self.h_hist[0] = self.h_hist[0][::2]
                    self.h_hist[1] = self.h_hist[1][::2]

    def _goal_cb(self, m):
        self.goal = (m.pose.position.x, m.pose.position.y)

    def _nom_cb(self, m):
        self.v_hist['t'].append(time.time() - self.t0)
        self.v_hist['nom'].append(m.linear.x)
        self.v_hist['cbf'].append(
            self.v_hist['cbf'][-1] if self.v_hist['cbf'] else 0.0)
        if len(self.v_hist['t']) > 3000:
            for k in self.v_hist:
                self.v_hist[k] = self.v_hist[k][::2]

    def _cbf_cb(self, m):
        if self.v_hist['cbf']:
            self.v_hist['cbf'][-1] = m.linear.x

    # ---------- 激光点投影到地图（用最新位姿） ----------
    def scan_points_map(self):
        m = getattr(self, '_latest_scan', None)
        if m is None or self.pose is None:
            return ([], [])
        x, y, yaw = self.pose
        xs, ys = [], []
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or r > 8.0 or r <= m.range_min:
                continue
            a = yaw + m.angle_min + i * m.angle_increment
            xs.append(x + r * math.cos(a))
            ys.append(y + r * math.sin(a))
        return (xs, ys)


def main():
    ap = argparse.ArgumentParser(description='实验实时可视化面板')
    ap.add_argument('--map', default=os.path.expanduser(
        '~/ros2_jazzy/install/nav2_bringup/share/nav2_bringup/maps/tb3_sandbox.yaml'))
    ap.add_argument('--d-safe', type=float, default=0.22,
                    help='屏障线位置（与 CBF 部署参数一致）')
    ap.add_argument('--save-shot', default=None, help='N 秒后保存快照 PNG')
    ap.add_argument('--shot-after', type=float, default=10.0)
    args = ap.parse_args()

    rclpy.init()
    node = LiveMonitor(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    img, extent = load_map(args.map)
    plt.ion()
    fig = plt.figure('nav2 实验实时面板', figsize=(13, 5.5))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.6, 1])

    ax_map = fig.add_subplot(gs[:, 0])
    ax_map.imshow(img, cmap='gray', extent=extent, origin='upper')
    ax_map.set_title('Map + Trail + A* Path + Goal')
    ax_map.set_xlabel('x [m]'), ax_map.set_ylabel('y [m]')
    trail_l, = ax_map.plot([], [], 'b.-', ms=2, label='trail')
    path_l, = ax_map.plot([], [], 'g-', lw=2, label='A* path')
    scan_l, = ax_map.plot([], [], 'o', color='orange', ms=2, label='laser')
    robot_l, = ax_map.plot([], [], 'r^', ms=12, label='robot')
    goal_l, = ax_map.plot([], [], 'r*', ms=16, label='goal')
    ax_map.legend(loc='upper right', fontsize=8)

    ax_h = fig.add_subplot(gs[0, 1])
    ax_h.axhline(0, color='r', ls='--', label='barrier h=0')
    h_l, = ax_h.plot([], [], 'b-', label='h(t) clearance')
    ax_h.set_ylabel('h [m]'), ax_h.legend(fontsize=8, loc='upper right')
    ax_h.set_title(f'h(t)  (d_safe={args.d_safe})')

    ax_v = fig.add_subplot(gs[1, 1])
    nom_l, = ax_v.plot([], [], 'b-', label='nominal v (smoothed)')
    cbf_l, = ax_v.plot([], [], 'r-', label='CBF output v')
    ax_v.set_xlabel('t [s]'), ax_v.set_ylabel('v [m/s]')
    ax_v.set_ylim(-0.6, 0.6), ax_v.legend(fontsize=8, loc='upper right')
    ax_v.set_title('nominal vs CBF output (split = intercept)')

    shot_at = time.time() + args.shot_after if args.save_shot else None
    try:
        while plt.fignum_exists(fig.number):
            # 地图视图
            trail_l.set_data(node.trail[0], node.trail[1])
            path_l.set_data(node.path[0], node.path[1])
            sx, sy = node.scan_points_map()
            scan_l.set_data(sx, sy)
            if node.pose:
                robot_l.set_data([node.pose[0]], [node.pose[1]])
            if node.goal:
                goal_l.set_data([node.goal[0]], [node.goal[1]])
            # h(t)
            h_l.set_data(node.h_hist[0], node.h_hist[1])
            if node.h_hist[0]:
                ax_h.relim(), ax_h.autoscale_view()
            # 速度
            nom_l.set_data(node.v_hist['t'], node.v_hist['nom'])
            cbf_l.set_data(node.v_hist['t'], node.v_hist['cbf'])
            if node.v_hist['t']:
                ax_v.set_xlim(max(0, node.v_hist['t'][-1] - 60),
                              max(10, node.v_hist['t'][-1] + 1))
            fig.canvas.draw_idle()
            plt.pause(0.1)
            if shot_at and time.time() >= shot_at:
                fig.savefig(args.save_shot, dpi=110)
                print(f'快照已保存: {args.save_shot}')
                shot_at = None
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
