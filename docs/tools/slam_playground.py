#!/usr/bin/env python3
"""SLAM 原理 playground —— doc 21《SLAM 建图原理零基础课》配套工具

六个演示，每个都能跑出完整的中间数值表（不止结果，连过程都打出来）：

  --demo 1   占据栅格：log-odds 逐次更新（看 p 如何随证据变化）
  --demo 2   贝叶斯滤波：3 格走廊定位（预测→观测→归一化 全表）
  --demo 3   EKF-SLAM：1D 机器人 + 1 路标（预测/增益/协方差收缩 全过程）
  --demo 4   粒子滤波：100 个粒子从均匀分布收敛到真值（直方图动画）
  --demo 5   相关性扫描匹配：评分表（含里程计先验破平局）
  --demo 6   图优化：三节点回环（正规方程求解 + 精度加权互验）

用法：
  python3 docs/tools/slam_playground.py --demo 1     # 单个演示
  python3 docs/tools/slam_playground.py --all        # 全部跑一遍
"""
import argparse
import math

import numpy as np

P_OCC = 0.7      # 束端：占据
P_FREE = 0.3     # 束中：自由
P_PRIOR = 0.5    # 先验
L_OCC = math.log(P_OCC / (1 - P_OCC))     # +0.847
L_FREE = math.log(P_FREE / (1 - P_FREE))  # -0.847
L_PRIOR = 0.0


def logodds_to_p(l):
    return 1.0 / (1.0 + math.exp(-l))


# ================================================================
# 演示 1：占据栅格 log-odds 逐次更新
# ================================================================
def demo1():
    print("=" * 72)
    print("演示 1：占据栅格 log-odds —— 一面墙是怎么被'画'出来的")
    print("=" * 72)
    print("""
场景：12 格 1D 世界。机器人站在格子 2，朝右发激光束（长度 6 格）。
束模型（doc 21 §3.2）：束中格子 → 自由证据 l=-0.847；束端格 → 占据证据 l=+0.847。
先验 l=0（p=0.5）。对同一面墙连续测 3 次，再让一束"穿透"（噪声）。
""")
    n = 12
    robot = 2
    beam_len = 6
    l = [L_PRIOR] * n

    beams = [
        ("第 1 束：右方 6 格处有墙（正常回波）", robot + beam_len),
        ("第 2 束：同上（再测一次）", robot + beam_len),
        ("第 3 束：同上（第三次）", robot + beam_len),
        ("第 4 束：噪声！束长 3（假回波，把格子 5 标成了占据）", robot + 3),
    ]

    for name, end in beams:
        print(f"\n--- {name} ---")
        print(f"{'格':>4} {'更新前l':>8} {'证据':>10} {'更新后l':>8} {'更新后p':>8}")
        for c in range(n):
            if c == end:
                meas, tag = L_OCC, "占+0.847"
            elif robot < c < end:
                meas, tag = L_FREE, "自-0.847"
            else:
                continue  # 未扫到，不更新
            new_l = l[c] + meas - L_PRIOR
            print(f"{c:>4} {l[c]:>8.3f} {tag:>10} {new_l:>8.3f} "
                  f"{logodds_to_p(new_l):>8.3f}")
            l[c] = new_l
        bar = "".join('#' if logodds_to_p(v) > 0.65 else
                      ('+' if logodds_to_p(v) > 0.35 else '.') for v in l)
        print(f"  栅格快照  [{bar}]   (#=占据 >65%, +=未知, .=自由 <35%)")

    print("""
观察：
  • 格子 8（墙）：三次占据证据 → l=0+3×0.847=2.54 → p=0.927
  • 格子 3-7（束中）：先 -0.847（自由）后 +0.847（假回波）→ 回到 0（对冲！）
  • 第 4 束的噪声被'证据对冲'机制消化——动态物体/噪声的数学解（doc 22）""")


# ================================================================
# 演示 2：贝叶斯滤波 3 格走廊
# ================================================================
def demo2():
    print("=" * 72)
    print("演示 2：贝叶斯滤波 —— 3 格走廊定位（预测→观测 全表）")
    print("=" * 72)
    print("""
场景：3 格走廊，格子 1、3 有门。传感器报"门/非门"：
  P(报门|真有门)=0.8   P(报门|没门)=0.2（误报率）
运动：命令右移一格 → 80% 成功、20% 打滑原地；格子 3 右墙挡住（=原地）。
先验：完全不知道 → [1/3, 1/3, 1/3]
序列：测到"门" → 右移一格 → 再测到"门"
""")
    prior = [1 / 3, 1 / 3, 1 / 3]
    doors = [True, False, True]   # 格子 1,2,3 是否有门
    P_door_if_door, P_door_if_nodoor = 0.8, 0.2
    P_move, P_stay = 0.8, 0.2

    def show(tag, bel):
        print(f"  {tag:<24} [" + ", ".join(f"{b:.4f}" for b in bel) + "]")

    show("先验", prior)

    # 观测 1：报门
    unnorm = [prior[i] * (P_door_if_door if doors[i] else P_door_if_nodoor)
              for i in range(3)]
    eta = sum(unnorm)
    prior = [u / eta for u in unnorm]
    print(f"  （观测1 未归一化 [" + ", ".join(f"{u:.4f}" for u in unnorm)
          + f"]，η={eta:.4f}）")
    show("观测1(报门)后", prior)

    # 运动：右移一格
    pred = [0.0] * 3
    for i, b in enumerate(prior):
        dest = min(i + 1, 2)          # 格子 3 的右墙挡住
        pred[dest] += b * P_move
        pred[i] += b * P_stay
    print("  （预测：80%右移 / 20%原地；格子3右墙=原地）")
    show("预测(右移)后", prior if False else pred)
    prior = pred

    # 观测 2：又报门
    unnorm = [prior[i] * (P_door_if_door if doors[i] else P_door_if_nodoor)
              for i in range(3)]
    eta = sum(unnorm)
    prior = [u / eta for u in unnorm]
    print(f"  （观测2 未归一化 [" + ", ".join(f"{u:.4f}" for u in unnorm)
          + f"]，η={eta:.4f}）")
    show("观测2(报门)后", prior)

    print("""
观察：
  • 第一步只测到"门"→ 1 和 3 都可能（多峰！0.444 vs 0.444 平分）
  • 右移一格后，两个峰摊开成 0.378/0.533
  • 第二次测到"门"→ 74% 锁定格子 3 —— 一次运动+一次观测打破了对称
  这就是 AMCL 粒子滤波定位的原理（把分布换成 N 个粒子）。""")


# ================================================================
# 演示 3：EKF-SLAM 1D 机器人 + 1 路标（全过程数值）
# ================================================================
def demo3():
    print("=" * 72)
    print("演示 3：EKF-SLAM —— 1D 机器人 + 1 路标（每一步数值）")
    print("=" * 72)
    print("""
状态 x = [机器人位置, 路标位置]，先验（高斯）：
  x̄ = [1.0, 3.0]，Σ = [[1.5, 0], [0, 0.5]]
运动：u=1，运动噪声 σ²=0.5（只有机器人会动）
观测：z = 路标 − 机器人 = 1.2（观测噪声方差 R=0.25）
""")
    x = np.array([1.0, 3.0])
    S = np.array([[1.5, 0.0], [0.0, 0.5]])

    print(f"先验          x = {x}, Σ =\n{S}")
    # 预测：机器人前进 u=1（路标不动），运动噪声只作用在机器人
    Q = np.array([[0.5, 0], [0, 0]])
    x = x + np.array([1.0, 0.0])
    S = S + Q
    print(f"\n预测后        x = {x}, Σ =\n{S}")
    print("  （机器人方差 1.5→2.0 膨胀；路标不变）")
    # 更新
    z = 1.2
    H = np.array([[-1.0, 1.0]])               # z = m − r
    R = np.array([[0.25]])
    innov = z - (H @ x)[0]
    Sh = (H @ S @ H.T + R)[0, 0]
    K = (S @ H.T / Sh).flatten()
    x = x + K * innov
    S = (np.eye(2) - np.outer(K, H)) @ S
    print(f"\n观测 z=1.2    预测值 H·x = 3.0−2.0 = 1.0"
          f"（用预测前 x=2.0,3.0 → 3.0−2.0=1.0）")
    print(f"残差 = 1.2 − 1.0 = 0.2")
    print(f"S = H·Σ·Hᵀ+R = 2.0+0.5+0.25 = 2.75")
    print(f"K = Σ·Hᵀ/S = [−2.0, 0.5]/2.75 = [{K[0]:+.3f}, {K[1]:+.3f}]")
    print(f"\n更新后        x = {x}")
    print(f"Σ =\n{S}")
    print("""
观察：
  • 机器人方差 2.0 → {:.3f}（收缩 ✓）
  • 交叉协方差 {:.3f} ≠ 0：机器人和路标从此关联——
    '知道机器人在哪就能推路标，反之亦然'
  • 路标估计 3.0 → {:.3f}：机器人的位置不确定性'传染'给了路标
  • n 个路标时这个关联链让 Σ 变成 O(n²)——EKF-SLAM 被淘汰的原因""".format(
        S[0, 0], S[0, 1], x[1]))


# ================================================================
# 演示 4：粒子滤波（100 粒子收敛）
# ================================================================
def demo4():
    print("=" * 72)
    print("演示 4：粒子滤波 —— 100 粒子从'完全迷失'到'唯一确定'")
    print("=" * 72)
    print("""
场景：1D 世界 [0, 7m]，门在 1.0m 和 3.3m（非对称间距，故意打破对称）。
传感器：报"到最近门的距离"（σ=0.05）。运动：前进 0.7m（σ=0.03）。
机器人真实位置：从 2.15m 出发（粒子不知道）。
粒子：100 个，均匀撒满 [0,7]。
""")
    rng = np.random.default_rng(42)
    doors = np.array([1.0, 3.3])
    true_x = 2.15
    particles = rng.uniform(0, 7, 200)

    def nearest_door(xs):
        return np.min(np.abs(np.asarray(xs)[:, None] - doors[None, :]), axis=1)

    def show_hist(xs, tag):
        bins = np.linspace(0, 7, 36)
        hist, _ = np.histogram(xs, bins=bins)
        scale = 40 / max(1, hist.max())
        bar = "".join('#' if c > 0 else '.' for c in hist)
        print(f"  {tag} [{bar}]  n={len(xs)}")

    def measure_weight(xs, z):
        pred = nearest_door(xs)
        return np.exp(-((pred - z) ** 2) / (2 * 0.05 ** 2))

    def resample(xs, w):
        w = w / w.sum()
        idx = rng.choice(len(xs), size=len(xs), p=w)
        return xs[idx]

    show_hist(particles, "初始均匀    ")

    # t0：测距
    z = abs(true_x - doors[np.argmin(np.abs(doors - true_x))]) + rng.normal(0, 0.05)
    w = measure_weight(particles, z)
    particles = resample(particles, w)
    show_hist(particles, f"测距 z={z:.2f}  ")
    modes = [round(v, 2) for v in sorted(set(np.round(particles, 1)))][:4]
    print(f"      → 存活簇（多峰）：{modes}")

    # 移动 + 测距
    true_x += 0.7 + rng.normal(0, 0.03)
    particles += 0.7 + rng.normal(0, 0.03, len(particles))
    z = abs(true_x - doors[np.argmin(np.abs(doors - true_x))]) + rng.normal(0, 0.05)
    w = measure_weight(particles, z)
    particles = resample(particles, w)
    show_hist(particles, "移动+测距   ")
    print(f"      → 真值 {true_x:.2f}，粒子均值 {particles.mean():.2f}")

    # 再移动 + 测距
    true_x += 0.7 + rng.normal(0, 0.03)
    particles += 0.7 + rng.normal(0, 0.03, len(particles))
    z = abs(true_x - doors[np.argmin(np.abs(doors - true_x))]) + rng.normal(0, 0.05)
    w = measure_weight(particles, z)
    particles = resample(particles, w)
    show_hist(particles, "再移+测距   ")
    print(f"      → 真值 {true_x:.2f}，粒子均值 {particles.mean():.2f}，"
          f"标准差 {particles.std():.3f}")
    print("""
观察：
  • 第一次测距后是多峰（'门两侧都像'）——粒子滤波天生表示多峰
  • 走 0.7m（门间距不均匀）+ 再测 → 假设们死光，只剩真值附近
  • 100 个粒子 × 每个粒子一张地图 = FastSLAM/Gmapping 的核心结构
  • 粒子标准差 = 定位置信度——收敛后不再变化即为「锁定」""")


# ================================================================
# 演示 5：相关性扫描匹配（评分表）
# ================================================================
def demo5():
    print("=" * 72)
    print("演示 5：相关性扫描匹配 —— 评分表（含里程计先验破平局）")
    print("=" * 72)
    print("""
地图 5×5（代价：墙=10，自由=0），墙在 x=3 列：格子 (3,1)(3,2)(3,3)。
机器人激光点：S = {(2,1), (2,2), (2,3)}（墙前一排）。
候选偏移 (dx,dy)：在窗口内枚举，把 S 投影到地图求代价和。
总分 = 匹配代价 + λ·‖偏移 − 里程计预测‖²  （λ=2，里程计预测 (+0.1, 0)）
""")
    wall = {(3, 1), (3, 2), (3, 3)}
    S = [(2, 1), (2, 2), (2, 3)]
    lam, odom = 2.0, (0.1, 0.0)
    cands = [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1), (1, 1)]

    print(f"{'候选(dx,dy)':>12} {'投影后代价':>12} {'里程计罚':>10} {'总分':>8}")
    best = None
    for dx, dy in cands:
        match = 0.0
        detail = []
        for (sx, sy) in S:
            px, py = sx + dx, sy + dy
            c = 10.0 if (px, py) in wall else 0.0
            match += c
            detail.append(f"({px},{py})={c:.0f}")
        pen = lam * ((dx - odom[0]) ** 2 + (dy - odom[1]) ** 2)
        total = match + pen
        mark = ""
        if best is None or total < best[0]:
            best = (total, dx, dy)
            mark = " ← 最优"
        print(f"({dx:+d},{dy:+d}){'':>5} {match:>12.0f} {pen:>10.2f} "
              f"{total:>8.2f}{mark}   [{' '.join(detail)}]")
    print(f"\n最优偏移：({best[1]:+d},{best[2]:+d})")
    print("""
观察：
  • (0,0) 和 (−1,0) 匹配代价打平（都是 0）——纯匹配分有歧义
  • 里程计先验（罚项）把最优解拉向'里程计预测附近' → 破平局
  • (+1,0) 撞墙代价 30 → 直接否决（doc 16 的'碰撞一票否决'同款）
  • slam_toolbox 的 correlation_search_space_* 参数就是这个
    窗口的大小/步长；smear_deviation 是把阶梯代价场做高斯模糊，
    让'差 2cm'和'完美'得分连续（可逼近）""")


# ================================================================
# 演示 6：图优化 三节点回环
# ================================================================
def demo6():
    print("=" * 72)
    print("演示 6：图优化 —— 三节点 + 一次回环（正规方程求解）")
    print("=" * 72)
    print("""
1D 机器人三个位姿，x1 锚定 0：
  里程计边：x2−x1 = 1.0 (σ=0.1)    x3−x2 = 1.0 (σ=0.1)
  回环边：  x3−x1 = 1.8 (σ=0.2)    ← 里程计积分说 2.0，漂了 0.2
最小化 J = Σ (预测−观测)²/σ²
""")
    # 信息矩阵（权重 = 1/σ²）
    # 边：x2−x1=1.0 (w=100), x3−x2=1.0 (w=100), x3−x1=1.8 (w=25)
    # 线性化 A·x = b（x1 固定 0）：
    #   x2 − x1 = 1.0  →  x2 = 1.0      w=100
    #   x3 − x2 = 1.0  →  −x2 + x3 = 1.0  w=100
    #   x3 − x1 = 1.8  →  x3 = 1.8      w=25
    A = np.array([
        [1.0, 0.0],      # x2 = 1.0
        [-1.0, 1.0],     # −x2 + x3 = 1.0
        [0.0, 1.0],      # x3 = 1.8
    ])
    b = np.array([1.0, 1.0, 1.8])
    W = np.diag([100.0, 100.0, 25.0])
    Hn = A.T @ W @ A
    gn = A.T @ W @ b
    xs = np.linalg.solve(Hn, gn)
    xs = np.concatenate([[0.0], xs])
    print(f"正规方程 H = AᵀWA =\n{Hn}")
    print(f"g = AᵀWb = {gn}")
    print(f"解：x1={xs[0]:.4f}, x2={xs[1]:.4f}, x3={xs[2]:.4f}")
    # 精度加权互验
    p_odo = 1 / (0.1 ** 2 + 0.1 ** 2)
    p_loop = 1 / 0.2 ** 2
    x3_check = (2.0 * p_odo + 1.8 * p_loop) / (p_odo + p_loop)
    print(f"\n互验（精度加权）：里程计链 x3=2.0 精度 {p_odo:.0f}，"
          f"回环 x3=1.8 精度 {p_loop:.0f}")
    print(f"  x3 = (2.0×{p_odo:.0f} + 1.8×{p_loop:.0f})/"
          f"({p_odo + p_loop:.0f}) = {x3_check:.4f}  ✓ 与求解一致")
    print("""
观察：
  • 回环把 x3 从漂移值 2.0 拉回 {:.3f}
  • x2 也被拉到 {:.3f}（没有回环时它是 1.0）——一个回环约束改善整条链
  • 权重 = 1/σ²：σ 是'这条边多可信'——调参调的就是信任分配
  • 真实 slam_toolbox 的求解器（Ceres/SPA）做的就是这个，只是
    节点上千、维度是 (x,y,θ)、边来自扫描匹配""".format(
        xs[2], xs[1]))


def main():
    ap = argparse.ArgumentParser(description='SLAM 原理 playground（doc 21 配套）')
    ap.add_argument('--demo', type=int, choices=[1, 2, 3, 4, 5, 6])
    ap.add_argument('--all', action='store_true')
    args = ap.parse_args()
    demos = {1: demo1, 2: demo2, 3: demo3, 4: demo4, 5: demo5, 6: demo6}
    if args.all:
        for i, d in demos.items():
            d()
            print()
    elif args.demo:
        demos[args.demo]()
    else:
        ap.print_help()


if __name__ == '__main__':
    main()
