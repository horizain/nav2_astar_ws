# -*- coding: utf-8 -*-
"""
run_astar.py —— A* 全局路径规划（第 10 章导航栈的第一层）

把 SLAM 建好的占据栅格地图做成"可搜索"的图：
  · 障碍物按机器人半径膨胀（inflation）—— 离墙太近的格子不可走
  · A*：f(n) = g(n) + h(n)，h 用对角欧氏距离（可采纳启发式）
  · 找到路径后再"视线拉直"（string pulling）减少锯齿

运行：python run_astar.py
"""
import os
import json
import heapq
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from world import World

C_OPEN = "#bfdbfe"      # A* 探索过的格子
C_PATH = "#f59e0b"      # 最终路径
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results")
IMG = os.path.abspath(os.path.join(HERE, "..", "..", "assets", "img"))

RES = 0.2               # 栅格分辨率 (m)
ROBOT_R = 0.35          # 机器人半径 (m) → 膨胀半径


def rasterize(world: World, res=RES):
    """把线段世界栅格化 → 占据布尔图 (H, W)。

    线段只覆盖"边框"；实心障碍矩形（柱子/桌子）的内部
    用 matplotlib.path 显式填充，避免"空心环"。
    """
    W = int(world.size[0] / res)
    H = int(world.size[1] / res)
    occ = np.zeros((H, W), dtype=bool)
    for x1, y1, x2, y2 in world.segments:
        L = max(int(np.hypot(x2 - x1, y2 - y1) / (res / 2)), 1)
        ts = np.linspace(0, 1, L)
        xs = x1 + (x2 - x1) * ts
        ys = y1 + (y2 - y1) * ts
        occ[np.clip((ys / res).astype(int), 0, H - 1),
            np.clip((xs / res).astype(int), 0, W - 1)] = True

    # 实心障碍内部填充
    from matplotlib.path import Path
    cc, rr = np.meshgrid(np.arange(W) * res + res / 2,
                         np.arange(H) * res + res / 2)
    pts = np.stack([cc.ravel(), rr.ravel()], axis=-1)
    for cx, cy, w, h in world.solid_rects:
        poly = Path([(cx - w/2, cy - h/2), (cx + w/2, cy - h/2),
                     (cx + w/2, cy + h/2), (cx - w/2, cy + h/2)])
        occ |= poly.contains_points(pts).reshape(H, W)
    return occ


def inflate(occ, radius, res=RES):
    """按机器人半径膨胀障碍（用距离变换，快且准）。"""
    from scipy.ndimage import distance_transform_edt
    dist = distance_transform_edt(~occ) * res      # 到最近障碍的距离
    return dist < radius


def astar(occ, start, goal):
    """
    8 连通 A*。
    start/goal: (row, col)。返回 (路径, 访问过的格子集合)。
    代价：直行 1，斜行 √2；启发式：对角欧氏距离（可采纳 → 最优性）。
    """
    H, W = occ.shape
    free = ~occ
    if not free[start] or not free[goal]:
        raise ValueError("起点/终点落在障碍物里")
    SQ2 = np.sqrt(2.0)

    def h(r, c):
        dr, dc = abs(r - goal[0]), abs(c - goal[1])
        return (dr + dc) + (SQ2 - 2) * min(dr, dc)   # 对角欧氏

    g = {start: 0.0}
    parent = {}
    closed = set()
    pq = [(h(*start), 0.0, start)]                   # (f, g, 节点)
    neigh = [(-1,0,1.0),(1,0,1.0),(0,-1,1.0),(0,1,1.0),
             (-1,-1,SQ2),(-1,1,SQ2),(1,-1,SQ2),(1,1,SQ2)]
    while pq:
        f, gcur, cur = heapq.heappop(pq)
        if cur in closed:
            continue
        closed.add(cur)
        if cur == goal:
            path = [cur]
            while path[-1] in parent:
                path.append(parent[path[-1]])
            return path[::-1], closed
        r, c = cur
        for dr, dc, w in neigh:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < H and 0 <= nc < W) or not free[nr, nc]:
                continue
            if dr != 0 and dc != 0:                  # 防止斜穿墙角
                if not free[r + dr, c] or not free[r, c + dc]:
                    continue
            ng = gcur + w
            if ng < g.get((nr, nc), np.inf):
                g[(nr, nc)] = ng
                parent[(nr, nc)] = cur
                heapq.heappush(pq, (ng + h(nr, nc), ng, (nr, nc)))
    return None, closed


def string_pull(path, occ):
    """视线拉直：只要两点间视线无障碍，就删掉中间点。"""
    def los(a, b):
        n = int(max(abs(b[0]-a[0]), abs(b[1]-a[1]))) * 2 + 1
        for t in np.linspace(0, 1, n):
            r = a[0] + (b[0]-a[0]) * t
            c = a[1] + (b[1]-a[1]) * t
            if occ[int(round(r)), int(round(c))]:
                return False
        return True
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not los(path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def main():
    os.makedirs(OUT, exist_ok=True)
    os.makedirs(IMG, exist_ok=True)
    world = World()
    occ = rasterize(world)
    occ_inf = inflate(occ, ROBOT_R)
    res = RES
    start = (int(2.5 / res), int(2.5 / res))     # (row=y, col=x)
    goal = (int(10.5 / res), int(16.0 / res))

    print("=== A* 路径规划 ===")
    import time
    t0 = time.time()
    path, closed = astar(occ_inf, start, goal)
    dt_ms = (time.time() - t0) * 1000
    pulled = string_pull(path, occ_inf)
    plen = sum(np.hypot(path[i+1][0]-path[i][0], path[i+1][1]-path[i][1])
               for i in range(len(path)-1)) * res
    print(f"  探索格子 {len(closed)}，路径 {len(path)} 格，"
          f"拉直后 {len(pulled)} 点，长度 {plen:.1f} m，耗时 {dt_ms:.1f} ms")

    # ---- 出图 ----
    fig, ax = plt.subplots(figsize=(10, 6.4), dpi=150)
    disp = np.ones(occ_inf.shape + (3,))
    disp[occ_inf] = [0.71, 0.85, 0.95]               # 膨胀区：浅蓝
    disp[occ] = [0.15, 0.23, 0.42]                   # 障碍：深蓝
    visited = np.zeros(occ.shape, dtype=bool)
    for (r, c) in closed:
        visited[r, c] = True
    disp[visited & ~occ_inf] = [0.75, 0.86, 0.99]    # A* 探索区
    ax.imshow(disp, origin="lower", extent=[0, world.size[0], 0, world.size[1]],
              alpha=0.9)
    world.plot(ax, color="#334155", lw=1.0)
    pr = np.array([p[0] for p in path]) * res + res / 2
    pc = np.array([p[1] for p in path]) * res + res / 2
    ax.plot(pc, pr, color="#60a5fa", lw=2.5,
            label=f"A* grid path ({len(path)} cells)")
    ur = np.array([p[0] for p in pulled]) * res + res / 2
    uc = np.array([p[1] for p in pulled]) * res + res / 2
    ax.plot(uc, ur, color=C_PATH, lw=2.5, ls="--",
            label=f"after string-pulling ({len(pulled)} waypoints)")
    ax.plot([start[1]*res+res/2], [start[0]*res+res/2], marker="o", ms=11,
            color="#10b981", ls="", label="start")
    ax.plot([goal[1]*res+res/2], [goal[0]*res+res/2], marker="*", ms=16,
            color="#ef4444", ls="", label="goal")
    ax.legend(loc="lower right")
    ax.set_title(f"A* on inflated occupancy grid "
                 f"({len(closed)} cells expanded, {dt_ms:.0f} ms)")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "fig_astar.png"))
    plt.close(fig)

    metrics = {"expanded": len(closed), "path_cells": len(path),
               "waypoints": len(pulled), "path_len_m": float(plen),
               "time_ms": float(dt_ms), "res": res, "robot_radius": ROBOT_R}
    with open(os.path.join(OUT, "metrics_astar.json"), "w") as f:
        json.dump(metrics, f, indent=2)
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
