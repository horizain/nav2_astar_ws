# -*- coding: utf-8 -*-
"""
run_mpc.py —— MPC 轨迹跟踪控制器（第 9 章的主角）

模型预测控制一句话：**每一步都求解"未来 N 步的最优控制序列"，
但只执行第一步，然后滚动到下一步重新求解**（Receding Horizon）。

本文件的完整配方（工业界最常用的 kinematic MPC）：
  1. 参考轨迹：航点 → 三次样条平滑 → (x_r, y_r, θ_r, v_r, ω_r)
  2. 误差模型：在参考点处线性化差速机器人运动学 → 线性时变系统
     e_{k+1} = A_k e_k + B_k Δu_k + c_k
  3. QP：min Σ ||e_k||²_Q + ||Δu_k||²_R  s.t. 速度/角速度/角加速度约束
  4. OSQP 求解 → 取第一个控制量作用到机器人 → 滚动
  5. 对照组：纯追踪（Pure Pursuit）——感受 MPC 好在哪里

运行：python run_mpc.py
"""
import os
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import osqp
from scipy import sparse

from world import World
from robot import Robot

C_REF = "#64748b"; C_MPC = "#2563eb"; C_PP = "#f59e0b"; C_GHOST = "#93c5fd"
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results")
IMG = os.path.abspath(os.path.join(HERE, "..", "..", "assets", "img"))

DT = 0.05                    # 控制周期 20 Hz
N = 25                       # 预测时域（1.25 s）
V_REF = 0.55                 # 参考速度
V_MAX, V_MIN = 1.0, 0.0
W_MAX = 1.6                  # 角速度限幅
DW_MAX = 3.0                 # 角加速度限幅 (rad/s²)
Q = np.diag([3.0, 3.0, 0.8])       # 状态误差权重（横向/纵向/航向）
R = np.diag([0.2, 0.2])            # 控制量权重
QF = 8.0 * Q                       # 终端权重（稳定性关键）


# ======================================================================
# 参考轨迹：航点 → 样条平滑
# ======================================================================
def make_reference(laps=2):
    """
    绕房间一圈的闭环参考线。
    关键：按*弧长*均匀重采样（ds=1cm），并按曲率限速 —— 让参考轨迹
    自身满足角速度约束（v·κ ≤ 0.9·ω_max）。
    返回 (x, y, θ, v, ω, ds)。路径闭合：第 0 点 == 末点（去掉重复）。
    """
    wp = np.array([(17.5, 2.0), (17.5, 9.8), (8.0, 9.8), (4.0, 9.5),
                   (2.0, 6.0), (2.0, 2.5)])
    wp_closed = np.vstack([wp, wp[:1]])                 # 闭环样条
    t = np.arange(len(wp_closed))
    csx = CubicSpline(t, wp_closed[:, 0], bc_type="periodic")
    csy = CubicSpline(t, wp_closed[:, 1], bc_type="periodic")

    ss = np.linspace(0.0, laps * len(wp), laps * 4000)  # 参数域稠密采样（1 圈 = 7 段）
    xs, ys = csx(ss), csy(ss)
    dx, dy = csx(ss, 1), csy(ss, 1)
    ddx, ddy = csx(ss, 2), csy(ss, 2)
    th = np.unwrap(np.arctan2(dy, dx))
    kappa = (dx * ddy - dy * ddx) / (dx**2 + dy**2) ** 1.5

    # 曲率限速：|ω| = |κ|·v ≤ 0.9 W_MAX
    vr = np.minimum(V_REF, 0.9 * W_MAX / np.maximum(np.abs(kappa), 1e-6))
    wr = kappa * vr

    # 按弧长均匀重采样
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(xs), np.diff(ys)))])
    total = s[-1]
    n_per_lap = int(total / laps / 0.01)                # 每圈 1cm 网格
    s_grid = np.linspace(0.0, total, laps * n_per_lap + 1)
    xs = np.interp(s_grid, s, xs)
    ys = np.interp(s_grid, s, ys)
    th = np.interp(s_grid, s, th)
    vr = np.interp(s_grid, s, vr)
    wr = np.interp(s_grid, s, wr)
    th = wrap(th)                                       # 回到 [-π, π)
    ds = s_grid[1] - s_grid[0]
    return xs[:-1], ys[:-1], th[:-1], vr[:-1], wr[:-1], ds


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ======================================================================
# 核心：一次 MPC 求解
# ======================================================================
class KinematicMPC:
    """误差状态 kinematic MPC（OSQP 稠密/ condensed 形式）。"""

    def __init__(self, N=N, dt=DT):
        self.N = N
        self.dt = dt
        self.nz = 2 * N

    def _dynamics(self, ref):
        """
        在参考轨迹处线性化：e_{k+1} = A e + B Δu + c。
        c_k = 参考轨迹自身一步推演与下一参考点的差（动力学不一致补偿）。
        """
        N, dt = self.N, self.dt
        As, Bs, cs = [], [], []
        for k in range(N):
            x, y, th, v, w = ref[k]
            xn, yn, thn = ref[k + 1][:3]
            c, s = np.cos(th), np.sin(th)
            A = np.eye(3)
            A[0, 2] = -v * s * dt
            A[1, 2] = v * c * dt
            B = np.array([[c * dt, 0.0], [s * dt, 0.0], [0.0, dt]])
            cvec = np.array([x + v * c * dt - xn,
                             y + v * s * dt - yn,
                             wrap(th + w * dt - thn)])
            As.append(A); Bs.append(B); cs.append(cvec)
        return As, Bs, cs

    def solve(self, state, ref, w_prev):
        """
        state: 当前真值状态 (x, y, θ)
        ref:   未来 N+1 步参考 (x, y, θ, v, ω) —— 数组 (N+1, 5)
        w_prev: 上一步实际角速度（用于角加速度约束）
        返回 (v, w) 最优第一控制量。
        """
        N, dt = self.N, self.dt
        xr, yr, thr, vr, wr = ref[:, 0], ref[:, 1], ref[:, 2], ref[:, 3], ref[:, 4]

        # ---- 误差状态初始值 ----
        e0 = np.array([state[0] - xr[0], state[1] - yr[0],
                       wrap(state[2] - thr[0])])

        As, Bs, cs = self._dynamics(ref)

        # ---- 凝聚预测：E_k = Φ_k e0 + Su_k z + C_k（逐步累积） ----
        Phi = np.eye(3)                       # 状态转移累积
        Su = np.zeros((3 * N, 2 * N))         # 控制敏感度矩阵
        E = np.zeros(3 * N)                   # 自由响应（e0 + 补偿项）
        Qbar = np.zeros((3 * N, 3 * N))
        acc = np.zeros((3, 2 * N))            # 当前步对所有未来控制的敏感度
        Cacc = np.zeros(3)                    # 补偿项累积
        for k in range(N):
            Phi = As[k] @ Phi
            Cacc = As[k] @ Cacc + cs[k]
            acc = As[k] @ acc
            acc[:, 2 * k:2 * k + 2] = Bs[k]
            Su[3 * k:3 * k + 3] = acc         # e_{k+1} 占第 3k~3k+2 行
            E[3 * k:3 * k + 3] = Phi @ e0 + Cacc
            Qk = QF if k == N - 1 else Q
            Qbar[3 * k:3 * k + 3, 3 * k:3 * k + 3] = Qk

        # ---- QP：min 0.5 zᵀH z + qᵀ z ----
        # 代价 = ||E||²_Qbar + ||Δu||²_R  （Δu 本身就是决策变量）
        H = Su.T @ Qbar @ Su + sparse.block_diag([R] * N).toarray()
        q = Su.T @ Qbar @ E
        P = sparse.csc_matrix(H)

        # ---- 约束：|v|、|ω| 限幅 + 角加速度限幅 ----
        A_rows, l_rows, u_rows = [], [], []
        for k in range(N):
            # v_k = v_r + Δv_k ∈ [V_MIN, V_MAX]
            row = np.zeros(2 * N); row[2 * k] = 1.0
            A_rows.append(row)
            l_rows.append(V_MIN - vr[k]); u_rows.append(V_MAX - vr[k])
            # w_k = w_r + Δw_k ∈ [-W_MAX, W_MAX]
            row = np.zeros(2 * N); row[2 * k + 1] = 1.0
            A_rows.append(row)
            l_rows.append(-W_MAX - wr[k]); u_rows.append(W_MAX - wr[k])
        for k in range(N):
            # |w_k - w_{k-1}| ≤ DW_MAX·dt （w_{-1} = w_prev 已知）
            row = np.zeros(2 * N); row[2 * k + 1] = 1.0
            if k > 0:
                row[2 * (k - 1) + 1] = -1.0
                base = wr[k] - wr[k - 1]
            else:
                base = wr[k] - w_prev
            A_rows.append(row)
            l_rows.append(-DW_MAX * dt - base)
            u_rows.append(DW_MAX * dt - base)

        A = sparse.csc_matrix(np.array(A_rows))
        prob = osqp.OSQP()
        prob.setup(P=P, q=q, A=A, l=np.array(l_rows), u=np.array(u_rows),
                   verbose=False, eps_abs=1e-3, eps_rel=1e-3,
                   max_iter=3000, warm_starting=True)
        res = prob.solve()
        if res.info.status_val not in (1, 2):    # 1=solved 2=solved inaccurate
            return vr[0], wr[0]                  # 求解失败 → 退化：按参考走
        z = res.x
        return vr[0] + z[0], wr[0] + z[1]


# ======================================================================
# 对照组：纯追踪 Pure Pursuit
# ======================================================================
class PurePursuit:
    def __init__(self, xs, ys, lookahead=0.8):
        self.xs, self.ys = xs, ys
        self.L = lookahead

    def control(self, state):
        # 找到路径上离机器人 lookahead 距离的目标点
        d = np.hypot(self.xs - state[0], self.ys - state[1])
        i_near = np.argmin(d)
        i = i_near
        while i < len(self.xs) - 1 and \
                np.hypot(self.xs[i] - state[0], self.ys[i] - state[1]) < self.L:
            i += 1
        tx, ty = self.xs[i], self.ys[i]
        heading_err = np.arctan2(ty - state[1], tx - state[0]) - state[2]
        heading_err = wrap(heading_err)
        w = np.clip(2.2 * heading_err, -W_MAX, W_MAX)
        v = V_REF if abs(heading_err) < 0.6 else 0.4 * V_REF
        return v, w


# ======================================================================
# 闭环仿真
# ======================================================================
def simulate(controller, ref_pack, seed=3):
    xs, ys, thr, vr, wr, ds = ref_pack
    world = World()
    robot = Robot(world, pose0=(xs[0], ys[0], thr[0]), dt=DT)
    np.random.seed(seed)
    log = {"t": [], "x": [], "y": [], "th": [], "v": [], "w": [],
           "ghosts": []}
    w_prev = 0.0
    s_pos = 0.0                                    # 当前参考弧长位置（格）
    total_steps = int(len(xs) * ds / (V_REF * DT)) + 50
    for k in range(total_steps):
        state = robot.true_pose
        if isinstance(controller, KinematicMPC):
            i0 = int(s_pos) % len(xs)
            idx = (i0 + np.arange(N + 1)) % len(xs)
            ref = np.stack([xs[idx], ys[idx], thr[idx], vr[idx], wr[idx]], axis=-1)
            v, w = controller.solve(state, ref, w_prev)
            # 保存预测滚动（用参考速度 + 求得的 w 非线性前向，画图用）
            if k % max(1, total_steps // 100) == 0:
                ghosts = [state[:2].copy()]
                gs = state.copy()
                for j in range(N):
                    gs = Robot._integrate(gs, ref[j, 3], w, DT)
                    ghosts.append(gs[:2].copy())
                log["ghosts"].append(np.array(ghosts))
            s_pos += ref[0, 3] * DT / ds
        else:
            v, w = controller.control(state)
            s_pos += V_REF * DT / ds
        w_prev = w
        robot.command(v, w)
        log["t"].append(k * DT)
        log["x"].append(robot.true_pose[0])
        log["y"].append(robot.true_pose[1])
        log["th"].append(robot.true_pose[2])
        log["v"].append(v)
        log["w"].append(w)
    for key in log:
        if key != "ghosts":
            log[key] = np.array(log[key])
    return log


def cross_track_rmse(log, xs, ys):
    d = np.min(np.hypot(xs[None, :] - log["x"][:, None],
                        ys[None, :] - log["y"][:, None]), axis=1)
    return float(np.sqrt(np.mean(d ** 2))), d


def main():
    os.makedirs(OUT, exist_ok=True)
    os.makedirs(IMG, exist_ok=True)
    ref_pack = make_reference()
    xs, ys = ref_pack[0], ref_pack[1]
    print(f"[参考] 参考轨迹 {len(xs)} 点，路径长 "
          f"{len(xs) * ref_pack[5]:.1f} m")

    print("=== MPC 闭环仿真 ===")
    mpc = KinematicMPC()
    log_mpc = simulate(mpc, ref_pack, seed=3)
    rmse_mpc, err_mpc = cross_track_rmse(log_mpc, xs, ys)
    print(f"[MPC]  横向误差 RMSE = {rmse_mpc:.3f} m")

    print("=== 纯追踪对照 ===")
    pp = PurePursuit(xs, ys)
    log_pp = simulate(pp, ref_pack, seed=3)
    rmse_pp, err_pp = cross_track_rmse(log_pp, xs, ys)
    print(f"[PP]   横向误差 RMSE = {rmse_pp:.3f} m")

    # ---- 图 1：轨迹对比 ----
    fig, ax = plt.subplots(figsize=(10, 6.4), dpi=150)
    ax.plot(xs, ys, color=C_REF, lw=2.4, ls="--", label="Reference path")
    ax.plot(log_pp["x"], log_pp["y"], color=C_PP, lw=1.4,
            label=f"Pure pursuit (RMSE {rmse_pp:.3f} m)")
    ax.plot(log_mpc["x"], log_mpc["y"], color=C_MPC, lw=2.0,
            label=f"MPC (RMSE {rmse_mpc:.3f} m)")
    ax.set_title("Trajectory tracking: MPC vs pure pursuit")
    ax.legend(loc="upper left")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "fig_mpc_track.png"))
    plt.close(fig)

    # ---- 图 2：横向误差随时间 ----
    fig, ax = plt.subplots(figsize=(9, 4), dpi=150)
    ax.plot(log_pp["t"], err_pp, color=C_PP, lw=1.2, label="Pure pursuit")
    ax.plot(log_mpc["t"], err_mpc, color=C_MPC, lw=1.5, label="MPC")
    ax.set_xlabel("time (s)"); ax.set_ylabel("cross-track error (m)")
    ax.set_title("Cross-track error over time")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "fig_mpc_err.png"))
    plt.close(fig)

    # ---- 图 3：控制量 ----
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), dpi=150, sharex=True)
    axes[0].plot(log_mpc["t"], log_mpc["v"], color=C_MPC, lw=1.2, label="MPC v")
    axes[0].axhline(V_MAX, color="#ef4444", lw=1, ls=":")
    axes[0].axhline(V_MIN, color="#ef4444", lw=1, ls=":")
    axes[0].set_ylabel("v (m/s)")
    axes[0].set_title("Control inputs stay inside constraints (blue lines)")
    axes[0].legend(loc="upper right")
    axes[1].plot(log_mpc["t"], log_mpc["w"], color=C_MPC, lw=1.2, label="MPC ω")
    axes[1].axhline(W_MAX, color="#ef4444", lw=1, ls=":")
    axes[1].axhline(-W_MAX, color="#ef4444", lw=1, ls=":")
    axes[1].set_ylabel("ω (rad/s)")
    axes[1].set_xlabel("time (s)")
    axes[1].legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "fig_mpc_inputs.png"))
    plt.close(fig)

    # ---- 图 4：滚动时域预测（3 个时刻的"水晶球"） ----
    fig, ax = plt.subplots(figsize=(10, 6.4), dpi=150)
    ax.plot(xs, ys, color=C_REF, lw=2.4, ls="--", label="Reference path")
    picks = [int(len(log_mpc["ghosts"]) * f) for f in (0.15, 0.45, 0.75)]
    colors = ["#3b82f6", "#10b981", "#f59e0b"]
    for p, c in zip(picks, colors):
        g = log_mpc["ghosts"][p]
        ax.plot(g[:, 0], g[:, 1], color=c, lw=1.6, marker="o", ms=2.5,
                label=f"predicted horizon @ t={log_mpc['t'][p]:.1f}s")
        ax.plot([log_mpc["x"][p]], [log_mpc["y"][p]], color=c, marker="s", ms=8)
    ax.plot(log_mpc["x"], log_mpc["y"], color="#94a3b8", lw=1.0, alpha=0.6,
            label="executed trajectory")
    ax.set_title("Receding horizon: predicted futures at three moments")
    ax.legend(loc="upper left")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "fig_mpc_horizon.png"))
    plt.close(fig)

    metrics = {
        "mpc_cross_track_rmse": rmse_mpc,
        "pp_cross_track_rmse": rmse_pp,
        "mpc_max_w": float(np.max(np.abs(log_mpc["w"]))),
        "mpc_max_v": float(np.max(np.abs(log_mpc["v"]))),
        "duration_s": float(log_mpc["t"][-1]),
        "n": N, "dt": DT,
    }
    with open(os.path.join(OUT, "metrics_mpc.json"), "w") as f:
        json.dump(metrics, f, indent=2)
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
