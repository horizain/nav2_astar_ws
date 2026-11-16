# nav2_astar_planner + nav2_controller_lab

一个用于学习 **A\* 算法、Nav2 插件开发、控制器算法** 的教学项目：

- `nav2_astar_planner`：把纯 C++ A* 栅格规划器封装成 `nav2_core::GlobalPlanner` 插件；
- `nav2_controller_lab`：在同一套仿真里实现并比较 PID / Pure Pursuit / Stanley /
  DWB-lite / MPPI-lite / MPC-lite 六种 `nav2_core::Controller` 插件。

> 代码不是重点，**“怎么从算法走到工程插件”**才是。
> 如果你没有算法基础，或没写过 ROS 2 / Nav2 插件，从
> [教学文档总目录](docs/README.md) 开始。

---

## 快速开始

```bash
cd ~/ros2_jazzy/nav2_astar_ws

# 1. 编译
source /opt/ros/jazzy/setup.bash
source ~/ros2_jazzy/install/setup.bash
colcon build --packages-select \
  nav2_astar_planner nav2_controller_lab nav2_controller_lab_rviz \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# 2. 跑纯算法单元测试（不需要 Gazebo）
colcon test --packages-select \
  nav2_astar_planner nav2_controller_lab nav2_controller_lab_rviz
colcon test-result --verbose

# 3. 启动 Gazebo + Nav2 + RViz（A* 全局规划器 + Pure Pursuit 控制器）
./run_controller_lab.sh

# 4. 在 RViz 里设置初始位姿、点 Nav2 Goal
#    如果 bringup 超时，另开终端执行：
./activate_nav.sh
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
│   ├── test/                     # 24 个算法测试 + 插件加载测试
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
