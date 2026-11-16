# -*- coding: utf-8 -*-
"""
world.py —— 2D 世界：墙体线段 + 激光光线投射（ray casting）

mini-SLAM 的第一块积木。整个"物理世界"只有两样东西：
  1. 一堆线段（外墙 + 障碍物轮廓）
  2. 一个函数 raycast()：从某个位姿沿某方向发射激光，返回撞到哪条线段、距离多少

为什么要用"线段 + 解析求交"而不是图像？
  - 解析法零误差、速度快，新手也能逐行看懂每个公式；
  - 真实雷达测的就是"光束打到哪"，线段模型和它一一对应。
"""
import numpy as np


def cross2(p, q):
    """2D 叉积 p×q（标量）。支持批量：p/q 形状 (... , 2)，广播相乘。"""
    return p[..., 0] * q[..., 1] - p[..., 1] * q[..., 0]


class World:
    """由线段构成的 2D 世界。size=(宽, 高) 单位：米。"""

    def __init__(self, size=(20.0, 12.0)):
        self.size = size
        self.segments = []          # 每条线段存 (x1, y1, x2, y2)
        self.solid_rects = []       # 实心障碍矩形 (cx, cy, w, h)（供规划器填充内部）
        self._build_default_map()
        self.segments = np.asarray(self.segments, dtype=float)  # (M, 4)

    # ---------- 建图：一栋"小房间 + 柱子 + 桌子"的室内场景 ----------
    def _add_rect(self, cx, cy, w, h, solid=False):
        """以 (cx,cy) 为中心、宽 w 高 h 的矩形边框（4 条线段）。
        solid=True 时同时记录为实心障碍（规划时内部视为占据）。"""
        x0, y0 = cx - w / 2.0, cy - h / 2.0
        x1, y1 = cx + w / 2.0, cy + h / 2.0
        corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        for i in range(4):
            a, b = corners[i], corners[(i + 1) % 4]
            self.segments.append((*a, *b))
        if solid:
            self.solid_rects.append((cx, cy, w, h))

    def _build_default_map(self):
        W, H = self.size
        self._add_rect(W / 2, H / 2, W, H)      # 四面外墙（空壳，非实心）
        self._add_rect(5.0, 8.0, 1.2, 1.2, solid=True)    # 方形立柱
        self._add_rect(10.0, 3.6, 2.6, 0.8, solid=True)   # 长桌
        self._add_rect(14.0, 8.5, 1.5, 1.5, solid=True)   # 大柱子
        self._add_rect(7.5, 5.5, 0.8, 0.8, solid=True)    # 小柱子
        self._add_rect(16.0, 4.2, 1.0, 1.5, solid=True)   # 靠墙柜子（避开默认航线）

    # ---------- 核心：光线投射 ----------
    def raycast(self, poses, angles, max_range):
        """
        批量光线投射。

        参数
        ----
        poses    : (N, 2) 或 (N, 3)，激光原点（或带朝向的位姿）
        angles   : (B,)，机体坐标系下的光束角度
        max_range: 最大量程（米）

        返回
        ----
        dist : (N, B)，每条光束的距离；没有回波时为 inf。

        原理（解析求交）
        ---------------
        光束:  p(t) = o + t·d            （d 为单位方向，t≥0）
        线段:  q(u) = a + u·(b - a)      （u ∈ [0,1]）

        联立 o + t·d = a + u·(b-a)，二维叉积解出（标准结果）：

            t = (a-o) × (b-a) / (d × (b-a))
            u = (a-o) × d     / (d × (b-a))

        分母为 0 表示平行（不相交）。t 就是激光飞行"距离"，取所有线段中
        最小的合法 t 即为该方向回波距离。
        """
        poses = np.atleast_2d(np.asarray(poses, dtype=float))
        o = poses[:, :2]                                   # (N, 2)
        if poses.shape[1] >= 3:
            yaw = poses[:, 2]
        else:
            yaw = np.zeros(len(poses))
        angles = np.asarray(angles, dtype=float)           # (B,)

        seg = self.segments
        a = seg[:, 0:2]                                    # (M, 2)
        e = seg[:, 2:4] - a                                # (M, 2) 线段方向

        # 全局光束方向 (N, B, 2)
        g = angles[None, :] + yaw[:, None]
        d = np.stack([np.cos(g), np.sin(g)], axis=-1)
        N, B = g.shape
        M = e.shape[0]

        dist = np.full((N, B), np.inf)
        chunk = 64                    # 分块计算，防止大 batch 撑爆内存
        for i in range(0, N, chunk):
            n = min(chunk, N - i)
            oc = o[i:i + n][:, None, :]                # (n, 1, 2)
            dc = d[i:i + n][:, :, None, :]             # (n, B, 1, 2)
            ao = (a - oc)[:, None, :, :]               # (n, 1, M, 2) = a - o
            ee = e[None, None, :, :]                   # (1, 1, M, 2)

            denom = cross2(dc, ee)                     # d × e      (n, B, M)
            with np.errstate(divide="ignore", invalid="ignore"):
                t = cross2(ao, ee) / denom             # 光束参数
                u = cross2(ao, dc) / denom             # 线段参数
            valid = (np.abs(denom) > 1e-12) & (t > 0.02) & (t <= max_range) \
                    & (u >= 0.0) & (u <= 1.0)
            dist[i:i + n] = np.where(valid, t, np.inf).min(axis=2)
        return dist

    def plot(self, ax, color="#334155", lw=1.2):
        """把世界画到 matplotlib 坐标轴上（供各脚本复用）。"""
        for x1, y1, x2, y2 in self.segments:
            ax.plot([x1, x2], [y1, y2], color=color, lw=lw,
                    solid_capstyle="round", zorder=3)
