# -*- coding: utf-8 -*-
"""
robot.py —— 差速（独轮车）机器人模型：运动学 + 里程计噪声 + 激光雷达

一台机器人 = 三件事：
  1. 运动学：给 (v, ω) 指令，它按独轮车模型移动        —— 这是"真值"
  2. 里程计：它 *以为* 自己走了多少（带噪声，会漂移）  —— 这是 SLAM 的输入
  3. 激光雷达：旋转一周测距（带高斯噪声）              —— 这也是 SLAM 的输入

SLAM 的任务：只用 2 和 3，把机器人的真实轨迹（1）和地图恢复出来。
"""
import numpy as np

from world import World


class Lidar:
    """2D 激光雷达参数。360° 视场、120 条光束、量程 9 m。"""

    def __init__(self, n_beams=120, fov=2 * np.pi, max_range=9.0, sigma=0.02):
        self.n_beams = n_beams
        self.fov = fov
        self.max_range = max_range
        self.sigma = sigma                     # 测距噪声标准差（米）
        # 光束角度均匀分布在机体坐标系（从 0 到 2π，不含重合的收尾角）
        self.beam_angles = np.linspace(0.0, fov, n_beams, endpoint=False)

    def scan(self, world: World, pose):
        """在真实位姿 pose 处打一帧激光，加噪声后返回 (B,) 距离数组。"""
        raw = world.raycast(pose, self.beam_angles, self.max_range)
        noisy = raw + np.random.normal(0.0, self.sigma, size=raw.shape)
        # 超量程/无回波的点保持 inf（不做噪声处理）；压成 (B,) 一维
        noisy[~np.isfinite(raw)] = np.inf
        return noisy.reshape(-1)


class Robot:
    """
    差速机器人。

    状态：true_pose = (x, y, θ)     —— 上帝视角的真值（SLAM 拿不到）
          odom_pose = (x, y, θ)     —— 里程计积分出来的"自认为"位姿
    """

    def __init__(self, world: World, pose0=(2.0, 2.0, 0.0), dt=0.05,
                 lidar: Lidar = None):
        self.world = world
        self.dt = dt
        self.lidar = lidar or Lidar()
        self.true_pose = np.array(pose0, dtype=float)
        self.odom_pose = np.array(pose0, dtype=float)
        # 里程计（编码器/IMU）噪声：每一步独立的测量噪声。
        # 就是这几行噪声，让第 4 章的"漂移惨案"发生。
        self.odom_std_v = 0.045          # 线速度测量噪声 (m/s)
        self.odom_std_w = 0.055          # 角速度测量噪声 (rad/s)
        self.act_std_v = 0.008           # 执行器（轮子打滑等）噪声
        self.act_std_w = 0.010

    # ---------- 运动学 ----------
    @staticmethod
    def _integrate(pose, v, w, dt):
        """独轮车运动学一步（一阶欧拉积分，dt=0.05s 时足够精细）。"""
        p = pose.copy()
        p[0] += v * np.cos(p[2]) * dt
        p[1] += v * np.sin(p[2]) * dt
        p[2] += w * dt
        p[2] = (p[2] + np.pi) % (2 * np.pi) - np.pi      # 归一化到 [-π, π)
        return p

    def command(self, v, w):
        """
        执行一次控制指令：
          · 真值：v,w 加微小执行噪声后积分；
          · 里程计：v,w 加*更大*的测量噪声后积分。
        里程计误差随时间累积 —— 这正是 SLAM 存在的理由。
        """
        v_true = v + np.random.normal(0.0, self.act_std_v)
        w_true = w + np.random.normal(0.0, self.act_std_w)
        self.true_pose = self._integrate(self.true_pose, v_true, w_true, self.dt)

        v_meas = v + np.random.normal(0.0, self.odom_std_v)
        w_meas = w + np.random.normal(0.0, self.odom_std_w)
        self.odom_pose = self._integrate(self.odom_pose, v_meas, w_meas, self.dt)
        return v_meas, w_meas

    def scan(self):
        """以真值位姿打一帧带噪激光（真实机器人只能拿到这份数据）。"""
        return self.lidar.scan(self.world, self.true_pose)

    def polar_to_points(self, ranges, pose):
        """把一帧极坐标激光转成世界坐标系点云 (B, 2)（inf 的点丢弃）。"""
        ang = self.lidar.beam_angles + pose[2]
        hit = np.isfinite(ranges)
        pts = np.stack([pose[0] + ranges * np.cos(ang),
                        pose[1] + ranges * np.sin(ang)], axis=-1)
        return pts[hit]


# ---------- 航点跟踪器（机器人的"自动驾驶"，第 3 章实现） ----------
def follow_waypoints(robot: Robot, waypoints, speed=0.5, tol=0.30,
                     k_w=2.2, max_w=1.5):
    """
    简易航点跟随器：一次调用走一步，返回 (v, w, 是否到达全部航点)。

    原理（纯追踪的最简版）：
      · 把目标点转到机体坐标系，得到横向偏差与航向偏差；
      · ω = k_w · 航向偏差（P 控制），v 固定巡航速度（偏差大时减速）。
    """
    wps = np.asarray(waypoints, dtype=float)
    state = {"idx": 0}

    def step():
        p = robot.true_pose          # 每次读取最新位姿（不要在闭包外缓存引用！）
        i = state["idx"]
        if i >= len(wps):
            return 0.0, 0.0, True
        target = wps[i]
        dx, dy = target[0] - p[0], target[1] - p[1]
        dist = np.hypot(dx, dy)
        if dist < tol:                       # 到达当前航点 → 切换下一个
            state["idx"] += 1
            return step()
        heading_err = np.arctan2(dy, dx) - p[2]
        heading_err = np.arctan2(np.sin(heading_err), np.cos(heading_err))
        v = speed if abs(heading_err) < 0.7 else 0.35 * speed
        w = np.clip(k_w * heading_err, -max_w, max_w)
        return v, w, False
    return step
