# -*- coding: utf-8 -*-
"""
grid_map.py —— 占据栅格地图（Occupancy Grid）与 log-odds 更新

核心思想：每个格子是"占据/空闲"的二值问题，但我们不存 0/1，
而存它的对数几率（log-odds）：

    l = log( p(占据) / (1 - p(占据)) )

好处：贝叶斯更新在 log 域里就是"加法"！

    l_new = l_old + l_meas          （l_meas: 本次测量带来的证据）

  · 光束打中的格子      → l_meas = +l_occ  （证据：这里可能有墙）
  · 光束穿过的沿途格子  → l_meas = -l_free（证据：这一路是空的）
多次测量自然按证据强弱叠加，最后用 sigmoid 映射回概率显示。
"""
import numpy as np


class OccupancyGrid:
    """分辨率 resolution 的 2D 占据栅格，原点在地图左下角。"""

    def __init__(self, size=(20.0, 12.0), resolution=0.06,
                 l_occ=0.85, l_free=-0.40, l_max=4.0):
        self.size = size
        self.res = resolution
        self.w = int(size[0] / resolution)           # 格子列数
        self.h = int(size[1] / resolution)           # 格子行数
        self.l_occ = l_occ                           # 命中格子的 log-odds
        self.l_free = l_free                         # 穿过格子的 log-odds
        self.l_max = l_max                           # 饱和截断（防止一格刷爆）
        self.logodds = np.zeros((self.h, self.w))    # 全 0 = 完全未知 50%

    def _to_cell(self, x, y):
        """世界坐标 → 格子索引（裁剪到地图范围内）。"""
        c = np.clip((x / self.res).astype(int), 0, self.w - 1)
        r = np.clip((y / self.res).astype(int), 0, self.h - 1)
        return r, c

    def update(self, pose, ranges, beam_angles, max_range):
        """
        用一帧激光更新地图。

        实现：沿每条光束按 resolution/2 步长"走"一遍，
              走过的格子 -l_free，命中点所在格子 +l_occ。
        （教学版用向量化光线步进；工业实现常用 Bresenham 直线算法，
         见第 6 章练习。）
        """
        ang = beam_angles + pose[2]
        # 命中距离：直接用测量值（命中格 = 距离所在格子）。
        # 无回波（inf）的光束：只标记沿途 free，绝不标记 occupied。
        r_hit = np.where(np.isfinite(ranges), ranges, max_range)

        steps = int(max_range / (self.res / 2.0))
        t = np.linspace(0.0, 1.0, steps)                       # 归一化进度
        # (B, steps, 2) 所有点：原点 + t·方向·距离
        pts = (np.array([pose[0], pose[1]])
               + t[None, :, None] * r_hit[:, None, None]
               * np.stack([np.cos(ang), np.sin(ang)], axis=-1)[:, None, :])

        B, S, _ = pts.shape
        rr, cc = self._to_cell(pts[..., 0].ravel(), pts[..., 1].ravel())
        rr = rr.reshape(B, S)
        cc = cc.reshape(B, S)

        free_mask = np.ones((B, S), dtype=bool)
        occ_mask = np.zeros((B, S), dtype=bool)
        has_return = np.isfinite(ranges)                        # 该光束是否真有回波
        # 有回波的光束：命中点 +l_occ（只此一个格子）
        hit_idx = np.nonzero(has_return)[0]
        if len(hit_idx):
            k_hit = np.searchsorted(t, 1.0) - 1                 # 最后一个采样点
            occ_mask[hit_idx, k_hit] = True
            free_mask[hit_idx, k_hit] = False
        # 所有光束的"命中距离之前"沿途都是 free
        prog = t[None, :] * r_hit[:, None]
        free_mask &= prog < r_hit[:, None] - 0.02
        free_mask &= ~occ_mask

        # np.add.at 处理重复索引的累加（scatter-add）
        np.add.at(self.logodds, (rr[free_mask], cc[free_mask]), self.l_free)
        np.add.at(self.logodds, (rr[occ_mask], cc[occ_mask]), self.l_occ)
        np.clip(self.logodds, -self.l_max, self.l_max, out=self.logodds)

        # np.add.at 处理重复索引的累加（scatter-add）
        np.add.at(self.logodds, (rr[free_mask], cc[free_mask]), self.l_free)
        np.add.at(self.logodds, (rr[~free_mask], cc[~free_mask]), self.l_occ)
        np.clip(self.logodds, -self.l_max, self.l_max, out=self.logodds)

    def rebuild(self, poses, scans, beam_angles, max_range):
        """清空后按一组 (位姿, 扫描) 重建整张地图（位姿图优化后用）。"""
        self.logodds = np.zeros_like(self.logodds)
        for pose, rng in zip(poses, scans):
            self.update(pose, rng, beam_angles, max_range)
        return self

    def probability(self):
        """log-odds → 占据概率（用于显示）。"""
        return 1.0 - 1.0 / (1.0 + np.exp(self.logodds))
