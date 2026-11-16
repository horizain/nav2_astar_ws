#!/usr/bin/env python3
"""DWB-lite 评分器离线复算器 —— doc 16《评分函数设计》配套演示工具

做什么：
  在纯 Python 里逐行复刻 nav2_controller_lab 的两个核心函数——
  rolloutConstantControl（差速模型 rollout）和 evaluateTrajectory（评分），
  然后在一个"解析重建的 tb3_sandbox 代价场"上，把 DWB 的 7×11 采样网格
  完整打分，打印成表格。

  用 --old-weights 复现"冻结"（doc 15 的 DWB 案例），
  用默认新权重复现"解冻"（doc 16 的修复结果）。

为什么值得手算：
  你会亲眼看到 v=0 原地不动的总分如何"合法地"赢过所有前进轨迹——
  不是 bug，是权重比例的结构性后果。换一组权重，同一张表立刻反转。

运行（不需要 ROS / Gazebo）：
  python3 docs/tools/dwb_scoring_demo.py                  # 新权重（解冻）
  python3 docs/tools/dwb_scoring_demo.py --old-weights    # 旧权重（冻结）
  python3 docs/tools/dwb_scoring_demo.py --goal-w 24 --obstacle-w 2
"""
import argparse
import math

# ---------------- 场景几何（tb3_sandbox，与仿真一致） ----------------
CYLINDER_RADIUS = 0.15
CYLINDERS = [(x, y) for x in (-1.1, 0.0, 1.1) for y in (-1.1, 0.0, 1.1)]  # 3×3 阵
# 六边形墙近似（本演示只用到出生点附近：西墙 x=-3.0）
WALL_SEGMENTS = [(( -3.0, -2.5), (-3.0, 2.5))]  # 西墙（线段近似）

# 膨胀层参数（与 nav2_params_controller_lab.yaml 一致）
INFLATION_RADIUS = 0.70
INSCRIBED = 0.22          # robot_radius
COST_SCALING = 3.0
MAX_COST = 252.0


def point_to_segment_dist(px, py, ax, ay, bx, by):
    """点到线段距离"""
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    ab2 = abx * abx + aby * aby
    t = 0.0 if ab2 == 0 else max(0.0, min(1.0, (apx * abx + apy * aby) / ab2))
    dx, dy = apx - t * abx, apy - t * aby
    return math.hypot(dx, dy)


def obstacle_surface_distance(x, y):
    """到最近障碍物表面的距离（圆柱表面 / 墙面）"""
    d = float('inf')
    for cx, cy in CYLINDERS:
        d = min(d, math.hypot(x - cx, y - cy) - CYLINDER_RADIUS)
    for (ax, ay), (bx, by) in WALL_SEGMENTS:
        d = min(d, point_to_segment_dist(x, y, ax, ay, bx, by))
    return d


def cost_at(x, y):
    """解析膨胀场：cost(d) = 252·exp(-λ·(d-r_inscribed)/(R-r_inscribed))

    d < r_inscribed(0.22)  → 253（内切区，机器人中心进入必碰撞）
    d ≥ R(0.70)            → 0（膨胀场外）
    """
    d = obstacle_surface_distance(x, y)
    if d < INSCRIBED:
        return 253
    if d >= INFLATION_RADIUS:
        return 0
    return int(round(MAX_COST * math.exp(
        -COST_SCALING * (d - INSCRIBED) / (INFLATION_RADIUS - INSCRIBED))))


# ---------------- 逐行复刻 C++（trajectory_utils / trajectory_cost） ----------------
def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def rollout_constant_control(robot, v, w, steps, dt):
    """复刻 trajectory_utils.hpp::rolloutConstantControl（差速模型）"""
    x, y, yaw = robot
    traj = [(x, y, yaw)]
    for _ in range(steps):
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt
        traj.append((x, y, yaw))
    return traj


def project_onto_path(point, path):
    """复刻 path_utils::projectOntoPath 的距离部分（逐段投影取最近）"""
    px, py, _ = point
    best = float('inf')
    for i in range(len(path) - 1):
        ax, ay, _ = path[i]
        bx, by, _ = path[i + 1]
        best = min(best, point_to_segment_dist(px, py, ax, ay, bx, by))
    return min(best, math.hypot(px - path[-1][0], py - path[-1][1]))


def remaining_distance(robot, path):
    """复刻 remainingDistance：投影点到终点的弧长 + 到投影点的直线距离"""
    px, py, _ = robot
    # 找最近投影
    best_i, best_d = 0, float('inf')
    for i in range(len(path) - 1):
        ax, ay, _ = path[i]
        bx, by, _ = path[i + 1]
        d = point_to_segment_dist(px, py, ax, ay, bx, by)
        if d < best_d:
            best_d, best_i = d, i
    remaining = math.hypot(px - path[best_i][0], py - path[best_i][1])
    for i in range(best_i, len(path) - 1):
        remaining += math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
    return remaining


def evaluate_trajectory(traj, path, weights, command_v, command_w):
    """复刻 trajectory_cost.cpp::evaluateTrajectory（含 /252 归一化）"""
    path_cost = obstacle_cost = 0.0
    for point in traj:
        path_cost += project_onto_path(point, path)
        cell = cost_at(point[0], point[1])
        if cell >= weights['collision_threshold']:
            return weights['collision_cost']  # 一票否决
        obstacle_cost += cell / 252.0
    n = 1.0 / len(traj)
    path_cost *= n
    obstacle_cost *= n
    goal_cost = math.hypot(traj[-1][0] - path[-1][0], traj[-1][1] - path[-1][1])
    return (weights['path_weight'] * path_cost +
            weights['goal_weight'] * goal_cost +
            weights['obstacle_weight'] * obstacle_cost +
            weights['velocity_weight'] * abs(command_v - weights['desired_v']) +
            weights['control_weight'] * abs(command_w))


# ---------------- DWB 采样网格复刻 ----------------
def dwb_compute(robot, path, weights, v_samples=7, w_samples=11,
                max_v=0.5, max_w=1.9, horizon=1.5, dt=0.1):
    """复刻 dwb_core.cpp::compute 的 7×11 采样，返回 (best, 评分表)"""
    steps = max(1, round(horizon / dt))
    best_cost, best = float('inf'), None
    table = []
    for vi in range(v_samples):
        v = weights['min_v'] + (max_v - weights['min_v']) * vi / (v_samples - 1)
        row = []
        for wi in range(w_samples):
            w = -max_w + 2 * max_w * wi / (w_samples - 1)
            traj = rollout_constant_control(robot, v, w, steps, dt)
            c = evaluate_trajectory(traj, path, weights, v, w)
            row.append(c)
            if c < best_cost:
                best_cost, best = c, (v, w)
        table.append(row)
    return best, table


def main():
    ap = argparse.ArgumentParser(description='DWB-lite 评分器离线复算（doc 16 配套）')
    ap.add_argument('--old-weights', action='store_true',
                    help='用 doc 15 的旧权重复现冻结（obstacle=100/goal=2/阈值254）')
    ap.add_argument('--goal-w', type=float, default=None)
    ap.add_argument('--obstacle-w', type=float, default=None)
    ap.add_argument('--path-w', type=float, default=None)
    args = ap.parse_args()

    if args.old_weights:
        weights = dict(path_weight=1.0, goal_weight=2.0, obstacle_weight=100.0,
                       velocity_weight=0.2, control_weight=0.05,
                       collision_threshold=254, collision_cost=1e6, desired_v=0.5,
                       min_v=0.0)
    else:
        weights = dict(path_weight=8.0, goal_weight=24.0, obstacle_weight=2.0,
                       velocity_weight=0.2, control_weight=0.05,
                       collision_threshold=253, collision_cost=1e6, desired_v=0.5,
                       min_v=0.0)
    if args.goal_w is not None:
        weights['goal_weight'] = args.goal_w
    if args.obstacle_w is not None:
        weights['obstacle_weight'] = args.obstacle_w
    if args.path_w is not None:
        weights['path_weight'] = args.path_w

    # 场景：机器人静止在出生点 (-2.0, -0.5)，朝向正东（yaw=0）
    # A* 路径：直线沿缝隙中线（y=-0.5）向东，直到接近目标区
    robot = (-2.0, -0.5, 0.0)
    path = [(-2.0 + 0.1 * i, -0.5, 0.0) for i in range(40)]  # 4m 直线

    print(f'== 评分权重 ==')
    print(f"  path={weights['path_weight']}  goal={weights['goal_weight']}  "
          f"obstacle={weights['obstacle_weight']}  threshold={weights['collision_threshold']}")
    print(f'== 场景 ==  机器人在出生点 (-2.0,-0.5) yaw=0，路径沿缝隙中线向东 4m')
    print(f'  脚下代价 cost_at(-2.0,-0.5) = {cost_at(*robot[:2])}'
          f'（到最近障碍表面 {obstacle_surface_distance(*robot[:2]):.2f}m）')
    print()

    best, table = dwb_compute(robot, path, weights)
    vs = [weights['min_v'] + (0.5 - weights['min_v']) * i / 6 for i in range(7)]
    ws = [-1.9 + 3.8 * i / 10 for i in range(11)]

    print('== 完整 7×11 采样评分表（行=v，列=w；★=本周期被选中；×=一票否决）==')
    print('      ' + ' '.join(f'w={w:+.2f}'.rjust(9) for w in ws))
    for vi, v in enumerate(vs):
        cells = []
        for wi, w in enumerate(ws):
            c = table[vi][wi]
            mark = '★' if best == (v, w) else (' ' if c < weights['collision_cost'] else '×')
            cells.append(f'{c:8.1f}{mark}')
        print(f'v={v:.2f} ' + ' '.join(cells))

    print()
    print(f'== 被选中的控制: v={best[0]:.3f}, w={best[1]:+.2f}'
          f'  (总分 {min(min(r) for r in table):.1f}) ==')
    v0 = min(min(table[0]) for _ in [0])  # v=0 行的最小值
    print(f'  对照：v=0 原地不动的最好总分 = {v0:.1f}')
    fwd = min(table[-1])
    print(f'  对照：v=0.5 全速前进的最好总分 = {fwd:.1f}')
    if best[0] < 0.05:
        print('  结论：采样器选择原地不动 → 冻结（v=0 赢了）')
    else:
        print(f'  结论：选择前进 → 正常运动'
              f'（{v0:.1f} vs {fwd:.1f}，前进赢 {v0 - fwd:.1f} 分）')


if __name__ == '__main__':
    main()
