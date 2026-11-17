# mini_slam —— 课程配套代码

一个约 1000 行、专为教学设计的 2D 激光 SLAM + MPC 导航项目。
网站每一章的代码块都来自这里，每一张结果图都由这里真实运行生成。

## 文件导览

| 文件 | 内容 | 对应章节 |
|------|------|----------|
| `world.py` | 2D 世界（墙体线段）+ 光线投射 | 第 3 章 |
| `robot.py` | 差速机器人：运动学 / 里程计噪声 / 激光雷达 / 航点跟踪 | 第 3、4 章 |
| `icp.py` | 点到点 ICP（SVD 闭式解 + 修剪稳健化） | 第 5 章 |
| `grid_map.py` | 占据栅格地图 + log-odds 更新 | 第 6 章 |
| `pose_graph.py` | SE(2) 位姿图 + Gauss-Newton 优化 | 第 7 章 |
| `run_slam.py` | 完整 mini-SLAM 流水线 + ATE 评估 | 第 8 章 |
| `run_mpc.py` | 误差模型 MPC（OSQP）+ 纯追踪对照 | 第 9 章 |
| `run_astar.py` | 栅格 A* + 障碍膨胀 + 视线拉直 | 第 10 章 |
| `generate_figures.py` | 一键生成网站全部结果图 | —— |

## 快速开始

```bash
# 1. 安装依赖（任选其一）
pip install -r requirements.txt
# Ubuntu 24.04+ 若提示 externally-managed：
python3 -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt

# 2. 跑通完整 SLAM（约 10 秒）
python run_slam.py

# 3. MPC 轨迹跟踪对比实验
python run_mpc.py

# 4. A* 全局规划
python run_astar.py

# 或一键生成所有结果图
python generate_figures.py
```

## 预期输出（随机种子固定为 7）

- 纯里程计：ATE RMSE ≈ 0.78 m，终点漂移 ≈ 1.6 m（跑 2 圈 / 87 m）
- mini-SLAM：ATE RMSE ≈ 0.22 m，回环边 ≈ 10 条
- MPC 跟踪：横向误差 RMSE ≈ 0.01–0.02 m（纯追踪 ≈ 0.03 m）
- 结果图输出到 `../../assets/img/`，指标 JSON 输出到 `results/`

## 练习建议

1. 把 `robot.py` 的里程计噪声调大 10 倍，观察 ICP 链何时开始失效；
2. 把 `run_slam.py` 的回环检测关掉（注释掉回环边），看地图鬼影；
3. 把 ICP 的 `trim_ratio` 设为 1.0（不修剪），制造一帧动态外点看翻车；
4. 把 MPC 的 `W_MAX` 降到 0.6，观察约束激活时控制器如何"抄近路"。
