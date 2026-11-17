# -*- coding: utf-8 -*-
"""
pose_graph.py —— SE(2) 位姿图优化（Pose Graph Optimization）+ Gauss-Newton

位姿图把 SLAM 变成一个"拼图"问题：
    节点 node = 机器人位姿 T_i ∈ SE(2)（3×3 齐次矩阵）
    边   edge = 位姿间的*相对*测量 z_ij（ICP 配准结果）+ 信息矩阵 Ω_ij
    回环 edge = 一条"拉直所有漂移"的额外约束

优化目标（最小二乘）：
    min_{T_i}  Σ e_ijᵀ Ω_ij e_ij ,   e_ij = Log( Z_ij⁻¹ · T_i⁻¹ · T_j )

Gauss-Newton 迭代：
    线性化  e ≈ e₀ + J·δ  →  正规方程 (JᵀΩJ) δ = -JᵀΩe  →  更新 T_i ← T_i·Exp(δ_i)

教学版用*数值雅可比*（有限差分），清晰不易错；节点数 ≤ 数百时稠密求解
毫无压力。工业级实现（g2o/Ceres/GTSAM）用解析雅可比 + 稀疏求解，
"思考题"里我们讨论为什么 H 是稀疏的。
"""
import numpy as np


# ---------------- SE(2) 指数/对数映射（精确公式） ----------------
def se2_exp(d):
    """指数映射：增量 d=(dx,dy,dθ) → 3×3 变换矩阵。"""
    dx, dy, th = d
    c, s = np.cos(th), np.sin(th)
    if abs(th) < 1e-9:
        vx, vy = dx, dy
    else:
        # V(θ) = (1/θ)·[[sinθ, -(1-cosθ)], [1-cosθ, sinθ]]，v = V⁻¹·t
        A = np.array([[s, -(1 - c)], [1 - c, s]]) / th
        vx, vy = np.linalg.solve(A, np.array([dx, dy]))
    return np.array([[c, -s, vx], [s, c, vy], [0.0, 0.0, 1.0]])


def se2_log(T):
    """对数映射：3×3 变换矩阵 → (x, y, θ)。"""
    c, s = T[0, 0], T[1, 0]
    th = np.arctan2(s, c)
    if abs(th) < 1e-9:
        v = T[:2, 2].copy()
    else:
        A = np.array([[s, -(1 - c)], [1 - c, s]]) / th
        v = np.linalg.solve(A, T[:2, 2])
    return np.array([v[0], v[1], th])


def se2_mat(pose):
    """(x, y, θ) → 3×3 矩阵。"""
    return se2_exp(np.asarray(pose, dtype=float))


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class PoseGraph:
    """节点 + 边 + Gauss-Newton 优化器。"""

    def __init__(self):
        self.nodes = []          # list[ndarray(3,3)] 位姿初值（里程计链）
        self.edges = []          # list[(i, j, z(3,), info(3,3))]

    def add_node(self, T_init):
        self.nodes.append(np.asarray(T_init, dtype=float))

    def add_edge(self, i, j, z, info):
        """z: 测得的 i→j 相对位姿 (x,y,θ)；info: 3×3 信息矩阵。"""
        self.edges.append((i, j, np.asarray(z, float), np.asarray(info, float)))

    # ---------- 残差 ----------
    def residual(self, i, j, z):
        """e_ij = Log( Z⁻¹ · T_i⁻¹ · T_j )，角度分量归一化到 [-π, π)。"""
        T = (np.linalg.inv(se2_mat(z))
             @ np.linalg.inv(self.nodes[i]) @ self.nodes[j])
        e = se2_log(T)
        e[2] = wrap_angle(e[2])
        return e

    def _jac(self, e_fn, node, eps=1e-6):
        """数值雅可比：对节点 node 右乘扰动 Exp(δ)，差分出 3×3 雅可比。"""
        J = np.zeros((3, 3))
        T_backup = self.nodes[node].copy()
        for d in range(3):
            delta = np.zeros(3)
            delta[d] = eps
            self.nodes[node] = T_backup @ se2_exp(delta)
            J[:, d] = (e_fn() - self._e0) / eps
            self.nodes[node] = T_backup
        return J

    # ---------- Gauss-Newton ----------
    def optimize(self, max_iter=20, tol=1e-5, verbose=True):
        """固定节点 0 作为全局坐标系"锚"，迭代直到收敛。返回代价历史。"""
        n = len(self.nodes)
        history = []
        for it in range(max_iter):
            H = np.zeros((3 * n, 3 * n))
            b = np.zeros(3 * n)
            cost = 0.0

            for (i, j, z, info) in self.edges:
                self._e0 = self.residual(i, j, z)
                cost += float(self._e0 @ info @ self._e0)

                e_fn = lambda: self.residual(i, j, z)   # noqa: E731
                Ji = self._jac(e_fn, i)                 # (3,3)
                Jj = self._jac(e_fn, j)

                # H 的块结构：每条边只把节点 i、j 连起来 → H 天生稀疏！
                H[3*i:3*i+3, 3*i:3*i+3] += Ji.T @ info @ Ji
                H[3*j:3*j+3, 3*j:3*j+3] += Jj.T @ info @ Jj
                H[3*i:3*i+3, 3*j:3*j+3] += Ji.T @ info @ Jj
                H[3*j:3*j+3, 3*i:3*i+3] += Jj.T @ info @ Ji
                b[3*i:3*i+3] += Ji.T @ info @ self._e0
                b[3*j:3*j+3] += Jj.T @ info @ self._e0

            # 固定第一个节点（SLAM 需要一个全局坐标系"锚"，否则整体漂移无定义）
            H[0:3, 0:3] += 1e6 * np.eye(3)

            delta = np.linalg.solve(H, -b)
            history.append(cost)
            if verbose:
                print(f"  [PGO] iter {it+1:2d}: cost = {cost:12.4f}")

            for k in range(n):
                self.nodes[k] = self.nodes[k] @ se2_exp(delta[3*k:3*k+3])
            if np.max(np.abs(delta)) < tol:
                break
        return history
