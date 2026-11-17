# -*- coding: utf-8 -*-
"""
run_slam.py —— mini-SLAM 主程序：把所有积木拼成完整的 2D 激光 SLAM

流水线（第 8 章逐行拆解）：

    1. 仿真机器人沿航点路线跑两圈（保证有回环！）
    2. 每一步记录：里程计位姿、真值位姿（仅评估用）、激光帧
    3. 关键帧：机器人走够 0.8 m 才收一帧 → 后端不用处理每一帧
    4. 相邻关键帧 ICP 配准 → 位姿图的"运动边"
    5. 回环检测：回到老地方（估计距离近、关键帧号隔得远）→ ICP → "回环边"
    6. 位姿图优化（Gauss-Newton）→ 漂移被拉直
    7. 用优化后的位姿重建占据栅格地图
    8. 评估 ATE，输出全部结果图

运行：python run_slam.py
"""
import os
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from world import World
from robot import Robot, follow_waypoints
from icp import icp, se2_matrix, matrix_to_se2
from grid_map import OccupancyGrid
from pose_graph import PoseGraph, se2_exp

C_GT = "#dc2626"; C_ODOM = "#f59e0b"; C_SLAM = "#2563eb"; C_LOOP = "#ef4444"
C_MAP = "#10b981"
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results")
IMG = os.path.abspath(os.path.join(HERE, "..", "..", "assets", "img"))


# ---------------- 小工具 ----------------
def scan_to_local(ranges, lidar):
    """一帧激光 → 机体系点云 (N,2)。激光的"原生"表达。"""
    hit = np.isfinite(ranges)
    ang = lidar.beam_angles[hit]
    return np.stack([ranges[hit] * np.cos(ang), ranges[hit] * np.sin(ang)], axis=-1)


def se2_apply(T, pts):
    """3×3 变换作用于点集 (N,2)。"""
    return pts @ T[:2, :2].T + T[:2, 2]


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def umeyama_align(src, dst):
    """相似变换（旋转+平移，不含缩放）把 src 刚性对齐到 dst。"""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    H = (src - mu_s).T @ (dst - mu_d)
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    return R, mu_d - R @ mu_s


def ate(est, gt):
    """ATE：对齐后逐点平移误差（米）。"""
    R, t = umeyama_align(est, gt)
    return np.linalg.norm(est @ R.T + t - gt, axis=1)


# ======================================================================
# 第 1 步：数据采集
# ======================================================================
def collect_data():
    world = World()
    robot = Robot(world, pose0=(2.0, 2.0, 0.0))
    # 航点路线：房间绕一大圈 × 2 —— 第二圈重走老路 → 天然回环
    # （路线特意避开室内所有障碍物，机器人不会"穿墙"）
    route = [(17.5, 2.0), (17.5, 9.8), (8.0, 9.8), (4.0, 9.5),
             (2.0, 6.0), (2.0, 2.5)] * 2
    step_fn = follow_waypoints(robot, route, speed=0.55)

    true_traj, odom_traj, scans = [], [], []
    for k in range(int(200 / robot.dt)):
        v, w, done = step_fn()
        robot.command(v, w)
        scans.append(robot.scan())
        true_traj.append(robot.true_pose.copy())
        odom_traj.append(robot.odom_pose.copy())
        if done and k > 100:
            break
    return world, robot.lidar, np.array(true_traj), np.array(odom_traj), scans


# ======================================================================
# 第 2 步：前端 —— 关键帧 + ICP 运动边 + 回环检测
# ======================================================================
def keyframe_scan_matching(lidar, odom_traj, scans):
    kf_step = 0.8                                   # 关键帧间隔（米）
    info_odom = np.diag([400.0, 400.0, 1.0e4])      # 运动边信息矩阵
    info_loop = np.diag([300.0, 300.0, 5.0e3])      # 回环边略"少信一点"
    info_bad = np.diag([40.0, 40.0, 1.0e3])         # 配准质量差的边：重降权

    # ---- 关键帧筛选（按里程计位移） ----
    kf_idx = [0]
    for k in range(1, len(odom_traj)):
        d = odom_traj[k, :2] - odom_traj[kf_idx[-1], :2]
        turn = abs(wrap(odom_traj[k, 2] - odom_traj[kf_idx[-1], 2]))
        if np.hypot(*d) >= kf_step or turn > np.deg2rad(45):
            kf_idx.append(k)
    n_kf = len(kf_idx)
    n_empty = sum(1 for i in kf_idx if np.sum(np.isfinite(scans[i])) < 30)
    print(f"[前端] 总帧数 {len(scans)}，关键帧 {n_kf}（退化帧 {n_empty}）")

    pg = PoseGraph()
    for k in kf_idx:
        pg.add_node(se2_matrix(odom_traj[k]))

    # ---- 运动边：相邻关键帧 ICP（在参考帧机体系内求相对位姿） ----
    T_odom = [se2_matrix(odom_traj[k]) for k in kf_idx]
    n_bad = 0
    chain = [pg.nodes[0].copy()]            # ICP 链式位姿估计（比纯里程计准）
    for a in range(1, n_kf):
        T_init = np.linalg.inv(T_odom[a - 1]) @ T_odom[a]   # 里程计初值
        ref = scan_to_local(scans[kf_idx[a - 1]], lidar)     # 上一帧机体系
        cur = scan_to_local(scans[kf_idx[a]], lidar)         # 当前帧机体系
        if len(ref) < 30 or len(cur) < 30:                  # 退化帧保护
            pg.add_edge(a - 1, a, np.zeros(3), info_bad)
            chain.append(chain[-1])
            continue
        # ICP 内部先用里程计初值再精化；返回的 T 就是 i→j 完整相对位姿
        T, rmse, _ = icp(ref, cur, init_pose=matrix_to_se2(T_init))
        if not np.isfinite(rmse):
            pg.add_edge(a - 1, a, np.zeros(3), info_bad)
            chain.append(chain[-1])
            continue
        info = info_bad if rmse > 0.25 else info_odom
        n_bad += rmse > 0.25
        pg.add_edge(a - 1, a, matrix_to_se2(T), info)
        chain.append(chain[-1] @ se2_exp(matrix_to_se2(T)))
    print(f"[前端] 运动边 {n_kf-1} 条（其中低质量 {n_bad} 条已降权）")
    for a in range(1, n_kf):
        pg.nodes[a] = chain[a]              # 用链式估计作为节点初值

    # ---- 回环检测：用当前链式估计找"物理靠近 + 时间隔远"的节点对 ----
    est = [matrix_to_se2(T) for T in chain]
    n_loop = 0
    last_loop_a = 10 ** 9          # 哨兵：表示"还没有回环"
    for a in range(n_kf - 1, 20, -1):
        if last_loop_a - a < 6:                              # 控制回环边密度
            continue
        for b in range(0, a - 20):
            if np.hypot(est[a][0] - est[b][0], est[a][1] - est[b][1]) > 1.5:
                continue
            T_ab_init = np.linalg.inv(se2_matrix(est[b])) @ se2_matrix(est[a])
            ref = scan_to_local(scans[kf_idx[b]], lidar)
            cur = scan_to_local(scans[kf_idx[a]], lidar)
            T, rmse, _ = icp(ref, cur, init_pose=matrix_to_se2(T_ab_init))
            if rmse < 0.12:                                  # 匹配优秀 → 确认回环
                pg.add_edge(b, a, matrix_to_se2(T), info_loop)
                n_loop += 1
                last_loop_a = a
                break
    print(f"[前端] 回环边 {n_loop} 条")
    return kf_idx, pg


# ======================================================================
# 主流程
# ======================================================================
def main():
    os.makedirs(OUT, exist_ok=True)
    os.makedirs(IMG, exist_ok=True)
    np.random.seed(7)

    print("=== 第 1 步：采集数据（机器人跑两圈） ===")
    world, lidar, true_traj, odom_traj, scans = collect_data()
    seg = np.hypot(np.diff(true_traj[:, 0]), np.diff(true_traj[:, 1]))
    path_len = float(np.sum(seg))
    print(f"  轨迹 {len(true_traj)} 帧（{len(true_traj)*0.05:.1f} s，"
          f"路径长 {path_len:.1f} m）")

    # ---- 图：里程计漂移（第 4 章的"惨案现场"） ----
    err_odom_full = ate(odom_traj[:, :2], true_traj[:, :2])
    fig, ax = plt.subplots(figsize=(10, 6), dpi=150)
    world.plot(ax, color="#334155", lw=1.0)
    ax.plot(true_traj[:, 0], true_traj[:, 1], color="#10b981", lw=2.0, ls="--",
            label="Ground truth")
    ax.plot(odom_traj[:, 0], odom_traj[:, 1], color=C_ODOM, lw=1.6,
            label="Odometry only (dead reckoning)")
    ax.scatter([odom_traj[-1, 0]], [odom_traj[-1, 1]], color=C_ODOM, s=60, zorder=5)
    ax.scatter([true_traj[-1, 0]], [true_traj[-1, 1]], color="#10b981", s=60, zorder=5)
    ax.annotate("odometry endpoint", (odom_traj[-1, 0], odom_traj[-1, 1]),
                xytext=(8, -14), textcoords="offset points", color=C_ODOM, fontsize=9)
    ax.set_title(f"Dead reckoning drifts: final error "
                 f"{err_odom_full[-1]:.2f} m after {len(true_traj)*0.05:.0f} s")
    ax.legend(loc="upper left")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "fig_odom_drift.png"))
    plt.close(fig)

    print("=== 第 2 步：ICP 前端 + 回环检测 ===")
    kf_idx, pg = keyframe_scan_matching(lidar, odom_traj, scans)

    # ---- 图：ICP 配准前后（第 5 章） ----
    a = len(kf_idx) // 3                       # 取中段一对相邻关键帧
    T_odom_a = se2_matrix(odom_traj[kf_idx[a - 1]])
    T_odom_b = se2_matrix(odom_traj[kf_idx[a]])
    T_init = np.linalg.inv(T_odom_a) @ T_odom_b
    ref = scan_to_local(scans[kf_idx[a - 1]], lidar)
    cur_local = scan_to_local(scans[kf_idx[a]], lidar)
    T_fit, rmse_fit, iters = icp(ref, cur_local, init_pose=matrix_to_se2(T_init))
    cur_init = se2_apply(T_init, cur_local)     # 仅用于画"配准前"
    cur_fit = se2_apply(T_fit, cur_local)       # 仅用于画"配准后"
    fig, axes = plt.subplots(1, 2, figsize=(11, 5.2), dpi=150, sharex=True, sharey=True)
    axes[0].scatter(ref[:, 0], ref[:, 1], s=6, color="#64748b", label="scan A (ref)")
    axes[0].scatter(cur_init[:, 0], cur_init[:, 1], s=6, color=C_ODOM,
                    label="scan B (odometry guess)")
    axes[0].set_title(f"Before ICP (odometry initialization)")
    axes[1].scatter(ref[:, 0], ref[:, 1], s=6, color="#64748b", label="scan A (ref)")
    axes[1].scatter(cur_fit[:, 0], cur_fit[:, 1], s=6, color=C_SLAM,
                    label="scan B (after ICP)")
    axes[1].set_title(f"After ICP: {iters} iters, RMSE = {rmse_fit:.3f} m")
    for ax in axes:
        ax.legend(loc="lower right"); ax.set_aspect("equal")
        ax.set_xlabel("x in scan-A frame (m)")
    axes[0].set_ylabel("y (m)")
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "fig_icp.png"))
    plt.close(fig)
    print(f"[图] ICP 演示用关键帧对 #{a}: RMSE={rmse_fit:.3f}, 迭代 {iters} 次")

    print("=== 第 3 步：位姿图优化（Gauss-Newton） ===")
    before = np.array([T[:2, 2] for T in pg.nodes])   # 3×3 矩阵的平移列
    hist = pg.optimize(verbose=True)
    after = np.array([T[:2, 2] for T in pg.nodes])

    gt_kf = true_traj[kf_idx, :2]
    rmse_before = float(np.sqrt(np.mean(ate(before, gt_kf) ** 2)))
    rmse_after = float(np.sqrt(np.mean(ate(after, gt_kf) ** 2)))
    print(f"[评估] 关键帧 ATE RMSE  优化前: {rmse_before:.3f} m → 优化后: {rmse_after:.3f} m")

    print("=== 第 4 步：重建占据栅格地图 ===")
    grid = OccupancyGrid(world.size, resolution=0.06)
    poses_slam = [matrix_to_se2(T) for T in pg.nodes]
    n_map = len(poses_slam)

    # ---- 图：建图过程（第 6 章） ----
    fig, axes = plt.subplots(1, 4, figsize=(16, 4.0), dpi=150)
    fracs = [0.25, 0.5, 0.75, 1.0]
    for ax, frac in zip(axes, fracs):
        g = OccupancyGrid(world.size, resolution=0.06)
        upto = max(2, int(n_map * frac))
        for pose, i in zip(poses_slam[:upto], kf_idx[:upto]):
            g.update(pose, scans[i], lidar.beam_angles, lidar.max_range)
        ax.imshow(g.probability(), cmap="gray_r", origin="lower",
                  extent=[0, world.size[0], 0, world.size[1]], vmin=0, vmax=1)
        ax.plot(true_traj[kf_idx[:upto], 0], true_traj[kf_idx[:upto], 1],
                color=C_MAP, lw=1.4)
        ax.set_title(f"{int(frac*100)}% of keyframes")
        ax.set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")
    fig.suptitle("Occupancy grid grows as the robot explores")
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "fig_map_progress.png"))
    plt.close(fig)

    # 完整地图（fig_slam_result 用）
    grid.rebuild(poses_slam, [scans[i] for i in kf_idx],
                 lidar.beam_angles, lidar.max_range)

    # ---- 图：里程计建图 vs SLAM 建图（"为什么要 SLAM"的最好答案） ----
    grid_odom = OccupancyGrid(world.size, resolution=0.06).rebuild(
        odom_traj[kf_idx], [scans[i] for i in kf_idx],
        lidar.beam_angles, lidar.max_range)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.8), dpi=150, sharey=True)
    for ax, g, title in (
            (axes[0], grid_odom, "Map from odometry poses only"),
            (axes[1], grid, "Map from SLAM poses")):
        ax.imshow(g.probability(), cmap="gray_r", origin="lower",
                  extent=[0, world.size[0], 0, world.size[1]], vmin=0, vmax=1)
        world.plot(ax, color="#dc2626", lw=0.9)
        ax.set_title(title)
        ax.set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "fig_map_compare.png"))
    plt.close(fig)

    # ---- 图 A：整体结果 ----
    fig, ax = plt.subplots(figsize=(10, 6), dpi=150)
    ax.imshow(grid.probability(), cmap="gray_r", origin="lower",
              extent=[0, world.size[0], 0, world.size[1]], vmin=0, vmax=1)
    world.plot(ax, color="#dc2626", lw=1.0)
    ax.plot(true_traj[:, 0], true_traj[:, 1], color="#10b981", lw=1.8, ls="--",
            label="Ground truth")
    ax.plot(odom_traj[:, 0], odom_traj[:, 1], color=C_ODOM, lw=1.1, alpha=0.85,
            label="Odometry only")
    ax.plot(after[:, 0], after[:, 1], color=C_SLAM, lw=2.2, label="mini-SLAM")
    ax.legend(loc="upper left", framealpha=0.95)
    ax.set_title(f"mini-SLAM: occupancy map + trajectories "
                 f"(ATE RMSE {rmse_after:.3f} m)")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "fig_slam_result.png"))
    plt.close(fig)

    # ---- 图 B：位姿图优化前后 ----
    odom_kf = odom_traj[kf_idx, :2]          # 里程计链（未做任何 SLAM）
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6), dpi=150, sharey=True)
    for ax, nodes, title, err in (
            (axes[0], odom_kf, "Before optimization (odometry chain)", ate(odom_kf, gt_kf)),
            (axes[1], after, "After pose-graph optimization", ate(after, gt_kf))):
        world.plot(ax, color="#94a3b8", lw=1.0)
        ax.plot(gt_kf[:, 0], gt_kf[:, 1], color="#10b981", lw=1.5, ls="--",
                label="Ground truth")
        ax.plot(nodes[:, 0], nodes[:, 1], color=C_SLAM, lw=1.8, marker="o",
                ms=2.5, label="Keyframe poses")
        for (i, j, z, info) in pg.edges:
            if abs(j - i) > 1:
                ax.plot([nodes[i, 0], nodes[j, 0]],
                        [nodes[i, 1], nodes[j, 1]],
                        color=C_LOOP, lw=1.3, ls=":", alpha=0.9)
        ax.set_title(f"{title}\nATE RMSE = {np.sqrt(np.mean(err**2)):.3f} m")
        ax.set_xlabel("x (m)")
    axes[0].set_ylabel("y (m)")
    axes[1].legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "fig_pose_graph.png"))
    plt.close(fig)

    # ---- 图 C：ATE 曲线 ----
    fig, ax = plt.subplots(figsize=(9, 4), dpi=150)
    ax.plot(np.arange(len(err_odom_full)) * 0.05, err_odom_full,
            color=C_ODOM, lw=1.2, label="Odometry only")
    ax.plot(np.array(kf_idx) * 0.05, ate(after, gt_kf),
            color=C_SLAM, lw=1.8, label="mini-SLAM")
    ax.set_xlabel("time (s)"); ax.set_ylabel("ATE (m)")
    ax.set_title("Absolute trajectory error over time")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "fig_ate.png"))
    plt.close(fig)

    metrics = {
        "n_frames": int(len(true_traj)),
        "n_keyframes": int(n_map),
        "n_loop_edges": int(sum(1 for e in pg.edges if abs(e[1] - e[0]) > 1)),
        "path_len_m": path_len,
        "ate_odom_full_rmse": float(np.sqrt(np.mean(err_odom_full**2))),
        "ate_odom_final": float(err_odom_full[-1]),
        "ate_odom_keyframe_rmse": rmse_before,
        "ate_slam_keyframe_rmse": rmse_after,
        "icp_demo_iters": int(iters),
        "icp_demo_rmse": float(rmse_fit),
        "pgo_cost_first": float(hist[0]),
        "pgo_cost_last": float(hist[-1]),
        "duration_s": float(len(true_traj) * 0.05),
    }
    with open(os.path.join(OUT, "metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2)
    print(json.dumps(metrics, indent=2))
    print("结果图已输出到:", IMG)


if __name__ == "__main__":
    main()
