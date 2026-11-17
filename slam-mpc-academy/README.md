# SLAM·MPC 导航课堂 —— 从零实现 2D 激光 SLAM 与机器人导航

一个面向**零基础新手**的教学网站 + 完整配套代码项目。

- **11 章图文教程**：从"什么是 SLAM"到组装完整导航栈（A* + MPC）
- **7 个可交互仿真**：激光扫描、漂移实验、ICP 逐步对齐、实时 mini-SLAM、
  MPC 调参驾驶舱、A* 工作台、导航全栈——全部原生 Canvas，无需构建工具
- **~1000 行可运行 Python**：`code/mini_slam/` 下每行都有中文注释
- **所有结果图均由真实运行生成**：`assets/img/fig_*.png` 由 `generate_figures.py` 输出

## 快速开始

```bash
# 看网站（任何静态服务器都行）
./serve.sh                # 打开 http://localhost:8080

# 跑代码
cd code/mini_slam
pip install -r requirements.txt    # 建议在 venv 里
python run_slam.py                 # 完整 mini-SLAM（约 10 秒）
python run_mpc.py                  # MPC vs 纯追踪对比实验
python run_astar.py                # A* 全局规划
python generate_figures.py         # 一键重新生成网站全部结果图
```

## 目录结构

```
slam-mpc-academy/
├── index.html            课程首页（含导航全栈压轴演示）
├── chapters/ch01..11     11 章教学内容
├── js/sim/               7 个交互仿真的源码（core.js 是共享引擎）
├── assets/img/           全部结果图（代码真实运行生成）
├── assets/vendor/        本地化的 KaTeX 与 highlight.js（离线可用）
├── code/mini_slam/       教学代码仓库（Python）
└── tests/                仿真引擎逻辑自检（node tests/node_core_test.js）
```

## 质量验证

- Python 流水线：`generate_figures.py` 一键复现全部结果图（固定随机种子，数字可复现）
- 仿真引擎逻辑：`node tests/node_core_test.js` —— 光线投射 / SE(2) 指数映射 / ICP 收敛 /
  A* 路径安全间距 / MPC 输出 / 参考轨迹连续性，共 12 项自检
- 全站浏览器回归：12 个页面 × 无头 Chromium —— 零 JS 错误、零失败请求，
  全栈演示端到端跑通（建图 110 关键帧 → 8 回环 → A* → MPC 抵达终点，ATE 0.03 m）

## 课程地图

| 部分 | 章节 | 内容 |
|------|------|------|
| 准备 | 1-2 | 总览与环境 · 数学最小集 |
| mini-SLAM | 3-8 | 仿真器 · 里程计 · ICP · 占据栅格 · 位姿图 · 组装 |
| 导航与控制 | 9-11 | MPC · A* 全栈 · 真车与 ROS 2 进阶 |
