# -*- coding: utf-8 -*-
"""
icp.py —— 迭代最近点（Iterative Closest Point），2D 点到点版本

ICP 回答的问题是：两帧激光点云 A、B 之间差了一个刚体变换 (R, t)，
求这个变换，让 B 变换后与 A 重合。

算法是一个漂亮的"鸡生蛋"循环：
    1. 猜一个初值（比如里程计给出的相对位姿）
    2. 固定变换 → 每个点找 A 中最近点，得到对应关系（correspondence）
    3. 固定对应关系 → 用 SVD 解析求最优刚体变换（一个闭式最小二乘！）
    4. 回到 2，直到收敛

每一步都在让误差下降，所以必然收敛（可能收敛到局部极小 —— 真实坑点）。
"""
import numpy as np
from scipy.spatial import cKDTree


def best_fit_transform(A, B):
    """
    闭式解：求 T 使 Σ||T·a_i - b_i||² 最小（已知一一对应）。
    经典 SVD 解法（Umeyama / K.S. Arun 1987）：
      1. 去质心
      2. H = Σ ā bᵀ；SVD: H = UΣVᵀ
      3. R = V·Uᵀ（若 det<0 翻转最后一列防镜像）
      4. t = b̄ - R ā
    """
    centroid_A = A.mean(axis=0)
    centroid_B = B.mean(axis=0)
    H = (A - centroid_A).T @ (B - centroid_B)          # 2×2 互协方差
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:                           # 防止解出"镜像"
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = centroid_B - R @ centroid_A
    T = np.eye(3)
    T[:2, :2] = R
    T[:2, 2] = t
    return T


def se2_to_matrix(pose):
    """(x, y, θ) → 3×3 齐次变换矩阵。"""
    x, y, th = pose
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


# 统一命名（与 pose_graph.se2_mat 同义）
se2_matrix = se2_to_matrix


def matrix_to_se2(T):
    """3×3 齐次矩阵 → (x, y, θ)。"""
    return np.array([T[0, 2], T[1, 2], np.arctan2(T[1, 0], T[0, 0])])


def icp(ref_pts, cur_pts, init_pose=(0.0, 0.0, 0.0), max_iter=30,
        tol=1e-5, trim_ratio=0.8):
    """
    把 cur_pts 对齐到 ref_pts。

    参数
    ----
    ref_pts, cur_pts : (N,2) 点云（同一传感器、同一路径上相邻两帧）
    init_pose        : (x,y,θ) 初值 —— 来自里程计的相对位姿（很重要！
                       没有好初值，ICP 会掉进局部极小）
    trim_ratio       : 修剪匹配最差的 20% 点对（抗外点，如动态障碍）

    返回
    ----
    T      : 3×3 变换，cur 系 → ref 系
    rmse   : 收敛后内点的均方根误差
    iters  : 实际迭代次数
    """
    ref_tree = cKDTree(ref_pts)
    T = se2_to_matrix(init_pose)
    cur = np.asarray(cur_pts, dtype=float)
    prev_err = None

    for it in range(max_iter):
        # ---- 步骤 1：用当前变换把 cur 投到 ref 系，找最近点 ----
        transformed = cur @ T[:2, :2].T + T[:2, 2]
        dist, idx = ref_tree.query(transformed, k=1)

        # ---- 修剪最差外点 ----
        order = np.argsort(dist)
        keep = order[: int(len(order) * trim_ratio)]
        rmse = float(np.sqrt(np.mean(dist[keep] ** 2)))

        # ---- 步骤 2：SVD 闭式求*增量*变换并左乘累积 ----
        # A 取"已变换后的点"（处于当前估计位置），B 是对应 ref 点；
        # T_delta 把 A 精确拉到 B → 总变换 T ← T_delta · T
        A = transformed[keep]
        B = ref_pts[idx[keep]]
        T_delta = best_fit_transform(A, B)
        T = T_delta @ T

        # ---- 收敛判断 ----
        if prev_err is not None and abs(prev_err - rmse) < tol:
            break
        prev_err = rmse
    return T, rmse, it + 1
