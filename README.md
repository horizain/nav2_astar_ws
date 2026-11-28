# 自己建图、定位与导航：ROS 2 教学项目

当前完整课程从 [23 mini_slam完整导航实践](docs/23_mini_slam完整导航实践.md) 开始：mini_slam建图与定位 → 自研A* → 自研控制器 → 保持启用的安全链路。旧的AMCL/slam_toolbox实验作为对照课程阅读。建设状态见[教学建设计划](docs/教学建设计划.md)。

一个用于学习 **SLAM、A*、路径跟踪及Nav2插件工程** 的教学项目：

- `mini_slam`：自己的扫描匹配、位姿图建图和定位，支持成套存图与重载；
- `nav2_astar_planner`：把纯 C++ A* 栅格规划器封装成 `nav2_core::GlobalPlanner` 插件；
- `nav2_controller_lab`：在同一套仿真里实现并比较 PID / Pure Pursuit / Stanley /
  DWB-lite / MPPI-lite / MPC-lite 六种 `nav2_core::Controller` 插件。
- `scan_localizer`：纯 C++20 + Eigen 的已知地图 2D 激光定位核心，配套 ROS 2 Jazzy
  Lifecycle 节点、仿真与 E1–E10 基准；[交互式工程课](docs/30_scan_localizer教程导览.md)
  覆盖从 SE(2)/EDT/Field-GN/4D EKF 到生产节点与公平实验的完整重建路径。

> 代码不是重点，**“怎么从算法走到工程插件”**才是。
> 如果你没有算法基础，或没写过 ROS 2 / Nav2 插件，从
> [教学文档总目录](docs/README.md) 开始。

---

## 快速开始

```bash
cd ~/ros2_ws/src/nav2_astar_ws

# 1. 编译
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
colcon build --packages-select \
  mini_slam nav2_astar_planner nav2_controller_lab nav2_controller_lab_rviz \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 2. 跑纯算法单元测试（不需要 Gazebo）
colcon test --packages-select \
  nav2_astar_planner nav2_controller_lab nav2_controller_lab_rviz
colcon test-result --verbose

# 3. 启动Gazebo、mini_slam、Nav2、RViz和键盘终端
./run_mini_slam_mapping.sh

# 4. 在操作终端低速建图，按3保存，按2切换定位/导航
# 需要时设置2D Pose Estimate，再用Nav2 Goal发目标
```

> 如果 `ros2 launch` 报 `package not found`，但 `install/<pkg>/` 明明存在，
> 先跑 `./check_workspace.sh` 做环境自检；确认是 install 前缀损坏后，
> 用 `./check_workspace.sh --fix` 备份并重建（不要只删 `build/`）。
> 原因见 [18 工程优化 §5](docs/18_工程优化与实战技巧.md)。

只想跑原来的 A* + MPPI 官方控制器：

```bash
./run_astar_gui.sh
```

环境配置、初始位姿、常见报错见
[01 从零复现教程](docs/01_教程_从零复现A星规划器.md)。

---

## 目录结构

```text
nav2_astar_ws/
├── src/nav2_astar_planner/
│   ├── include/nav2_astar_planner/
│   │   ├── astar_algorithm.hpp   # 纯算法（无 ROS 依赖）
│   │   └── astar_planner.hpp     # GlobalPlanner 适配层
│   ├── src/
│   │   ├── astar_algorithm.cpp
│   │   └── astar_planner.cpp
│   ├── test/
│   │   └── test_astar_algorithm.cpp
│   ├── params/nav2_params_astar.yaml
│   ├── launch/astar_simulation_launch.py
│   ├── global_planner_plugin.xml
│   ├── package.xml
│   └── CMakeLists.txt
├── src/nav2_controller_lab/      # 控制器实验室
│   ├── include/nav2_controller_lab/
│   │   ├── controller_base.hpp   # ROS 适配层基类
│   │   ├── *_core.hpp            # 纯算法：PID/PP/Stanley/DWB/MPPI/MPC
│   │   └── *_controller.hpp      # 6 个 Controller 插件
│   ├── src/
│   ├── test/                     # 纯算法与插件加载回归
│   ├── params/nav2_params_controller_lab.yaml
│   ├── launch/controller_lab_launch.py
│   ├── controller_plugins.xml
│   ├── package.xml
│   └── CMakeLists.txt
├── src/nav2_controller_lab_rviz/ # RViz 面板 + 调试标记
│   ├── src/controller_lab_panel.cpp
│   ├── src/controller_lab_visualizer.cpp
│   ├── rviz_plugins.xml
│   ├── package.xml
│   └── CMakeLists.txt
├── docs/                         # 教学文档
│   ├── README.md                 # 学习路线与文档索引
│   ├── 01_教程_从零复现A星规划器.md
│   ├── 02_原理_A星算法与Nav2插件机制.md
│   ├── 03_教学进度记录.md
│   ├── 04_A星算法零基础课.md
│   ├── 05_ROS2与Nav2接口入门.md
│   ├── 06_从纯算法到Nav2插件实战.md
│   ├── 07_源码逐行导读.md
│   ├── 08_控制器算法学习路线.md
│   ├── 09_控制器实验手册.md
│   ├── 10_控制器算法零基础课.md
│   ├── 11_Controller接口与插件实战.md
│   ├── 12_控制器源码逐行导读.md
│   ├── 13_控制器实验调参与排错.md
│   ├── 14_控制器开发进度记录.md
│   └── tools/                    # 不依赖 ROS 的算法练习工具
├── run_astar_gui.sh              # A* + 官方 MPPI 控制器
├── run_controller_lab.sh         # A* + 本项目控制器
├── run_controller_lab_rviz.sh    # 单独启动控制器实验室 RViz
├── run_rviz.sh
└── activate_nav.sh
```

---

## 当前完整课程

23～28构成同一条可复现主线：完整运行 → 状态诊断 → 定量评价 → MPC故障复盘 → 独立真值精度 → 固定实验与作业。入口见[教学总目录](docs/README.md)。

```bash
# 固定场景（先完成三级source）
python3 docs/tools/run_navigation_lesson.py --scenario straight --controller MPC
# 离线工具单测，无需ROS
python3 -m unittest discover -s docs/tools/tests -v
```

run_controller_lab.sh保留为AMCL/官方地图对照课程入口，与mini_slam主线区分。

---

## 文档怎么读

| 你的情况 | 从这里开始 |
|---|---|
| 完全没学过算法 | [04 A* 算法零基础课](docs/04_A星算法零基础课.md) |
| 不懂 ROS2 / Nav2 接口 | [05 ROS2 与 Nav2 接口入门](docs/05_ROS2与Nav2接口入门.md) |
| 想把算法写成插件 | [06 从纯算法到 Nav2 插件实战](docs/06_从纯算法到Nav2插件实战.md) |
| 想逐行读懂源码 | [07 源码逐行导读](docs/07_源码逐行导读.md) |
| 想学控制器 / MPPI | [08 控制器算法学习路线](docs/08_控制器算法学习路线.md) |
| 想补控制器算法 | [10 控制器算法零基础课](docs/10_控制器算法零基础课.md) |
| 想写 Controller 插件 | [11 Controller 接口与插件实战](docs/11_Controller接口与插件实战.md) |
| 想读控制器源码 | [12 控制器源码逐行导读](docs/12_控制器源码逐行导读.md) |
| 想做控制器对比实验 | [09 控制器实验手册](docs/09_控制器实验手册.md) / [13 调参排错](docs/13_控制器实验调参与排错.md) |
| 想看开发踩坑 | [14 控制器开发进度记录](docs/14_控制器开发进度记录.md) |
| 只想先跑起来 | [01 从零复现教程](docs/01_教程_从零复现A星规划器.md) |

完整学习路线和时间预估见 [docs/README.md](docs/README.md)。

---

## 不装 ROS 也能练算法

```bash
# 5x5 小地图，4 邻域，打印每次扩展
python3 docs/tools/astar_playground.py --no-diagonal --trace

# 对比 A* / 加权 A* / Dijkstra / BFS
python3 docs/tools/astar_playground.py --demo-map --compare

# 地形代价：数字越大越贵
python3 docs/tools/astar_playground.py \
  --map docs/tools/maps/terrain_demo.txt --terrain-weight 3
```

---

## 许可

Apache-2.0。教学实现，生产使用前请补充 footprint 碰撞检查、
并发保护、动态参数和更完整的测试。

---

## `scan_localizer` 真实公开数据集基准与离线排障

在 3 套公开真实 2D 激光数据集上的端到端评测报告（含可视化图表与结构化 `summary.json`）：

| 数据集与场景 | 核心评测维度 | 关键实测结果 | 详细报告 |
| :--- | :--- | :--- | :--- |
| 浙大玉泉 GP-SLAM-2D（曹光彪 46×46m / 永谦 27×28m） | 5 种匹配器对比 + 异步全局重定位 + 3×绑架恢复 + 对称圆弧消歧 | 曹光彪默认参数 94.43% 接受率 / 0.101 m ATE、绑架 **3/3**；永谦对称镜像误锁治理后 ATE **4.729 m → 0.725 m (-84.7%)** | [GP-SLAM-2D 基准报告](docs/results/gp_slam_2d_benchmark/README.md) |
| Google Cartographer MiR100（24×15m，360 s / 124.5 m） | 4D EKF 在线陀螺零偏估计 + 50% 旋转打滑 + 无里程计退化 | 12 组工况 **0 Degraded / 0 Lost**；`+0.25 rad/s` 零偏 17.7 s 收敛（残差 `<0.03°/s`）；50% 打滑 `kMahal` 降 **91.0%**；纯无里程计+陀螺 **99.98%** | [MiR100 4D EKF 报告](docs/results/mir100_imu_4d_ekf_benchmark/README.md) |
| Google Cartographer 德意志博物馆双包（53×44m，相隔 22 天） | 跨 22 天环境变化 + 零轮式里程计全盲冷启动压测 | 32.3% 动态/展品变化下 **14.8 s** 全盲冷启动锁定，并验证纯 2D 定位对 3D 背包晃动的物理边界 | [德意志博物馆压测报告](docs/results/deutsches_museum_crossday_benchmark/README.md) |

离线排障与推送前自检工具（均不依赖 ROS 2）：

```bash
# 离线复盘 BlackboxRecorder 录制的 .slocbb 事故黑盒（可选 --map / --csv / --plot）
python3 src/scan_localizer/tools/inspect_blackbox.py path/to/fault.slocbb --map map.yaml --plot diag.png

# 推送 PR 前一键本地预检（参数一致性 / 代码清单 / 文档链接 / flake8 / 行宽）
./scripts/lint_scan_localizer.sh
```
