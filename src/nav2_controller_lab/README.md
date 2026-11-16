# nav2_controller_lab

Nav2 控制器算法教学实验室：在同一个 Nav2 仿真里，用同一张地图、同一个
A* 全局规划器，横向比较 **PID / Pure Pursuit / Stanley / DWB-lite / MPPI-lite / MPC-lite**
六种控制器。

> 全局规划器复用同工作空间的 `nav2_astar_planner::AStarPlanner`。
> 本包只负责 `nav2_core::Controller` 插件，不重写全局规划。

**配套文档**：

- [08 控制器算法学习路线](../../docs/08_控制器算法学习路线.md)
- [10 控制器算法零基础课](../../docs/10_控制器算法零基础课.md)
- [11 Controller 接口与插件实战](../../docs/11_Controller接口与插件实战.md)
- [12 控制器源码逐行导读](../../docs/12_控制器源码逐行导读.md)
- [09 控制器实验手册](../../docs/09_控制器实验手册.md)
- [13 控制器实验、调参与排错](../../docs/13_控制器实验调参与排错.md)
- [14 控制器开发进度记录](../../docs/14_控制器开发进度记录.md)
- [15 端到端链路实战](../../docs/15_端到端链路实战.md)（headless 一条命令跑通导航：`python3 docs/tools/e2e_nav_test.py --controller MPPI`）
- [16 局部轨迹评分函数设计](../../docs/16_局部轨迹评分函数设计.md)（DWB 冻结解剖与解冻：`python3 docs/tools/dwb_scoring_demo.py`）
- [17 CBF 安全过滤器](../../docs/17_CBF安全过滤器.md)（独立安全层，插在速度链路：`python3 docs/tools/cbf_experiment.py` 做 h(t) 对比）
- [18 工程优化与实战技巧](../../docs/18_工程优化与实战技巧.md)（实测调优：控制环 5.5→19.6Hz；symlink-install 免编译改参）
- [19 端到端策略与 VLA 路线图](../../docs/19_端到端策略与VLA路线图.md)（**规划篇⏳ 未实施**：BC/DAgger/VLA 微调的四阶段施工图）
- [20 定位与建图 SLAM](../../docs/20_定位与建图SLAM.md)（slam_toolbox 建图→存图→自建图导航闭环：`auto_explorer.py` + `save_slam_map.py`）
- [21 SLAM 建图原理零基础课](../../docs/21_SLAM建图原理零基础课.md)（栅格/EKF/粒子滤波/图优化，每个算法手算一遍且配 `slam_playground.py` 可跑演示 + [交互式网站](../../docs/website/index.html)）
- [22 SLAM 工程实战](../../docs/22_SLAM工程实战.md)（参数逐条/雷达与里程计标定/抗干扰手册/故障速查）

---

## 1. 这个包做了什么

| 控制器 | 类型 | 难度 | 参考的 Nav2 控制器 | 特点 |
|---|---|---|---|---|
| `PidController` | 反馈控制基线 | ★ | 无完整对应 | 展示“PID 单独跟踪路径”的局限 |
| `PurePursuitController` | 几何跟踪 | ★ | `RegulatedPurePursuitController` | 最推荐入门；一个 lookahead 参数 |
| `StanleyController` | 几何 + 横向误差 | ★★ | 无完全对应 | 更贴线，速度高时需调参 |
| `DwbController` | 采样 + critics | ★★★ | `DWBLocalPlanner` | 显式评估未来轨迹和局部障碍 |
| `MppiController` | 采样 + 软最大 | ★★★★ | `MPPIController` | 控制序列采样，平滑但计算量大 |
| `MpcController` | 梯度优化 | ★★★★ | 无（核心外） | 控制序列优化、约束投影、线搜索 |

所有控制器共享同一个 `ControllerBase` 适配层，负责：

- Nav2 生命周期：`configure / activate / deactivate / cleanup`
- 参数实例名前缀：`FollowPath.xxx`
- 全局路径 `map` → 局部代价地图 `odom` 的 TF 变换
- `nav_msgs::Path` → 纯 C++ `Pose2D` 路径
- 速度限制、`TwistStamped` 封装、异常处理

算法核心全部放在 `*_core.{hpp,cpp}`，**不依赖 ROS**，可以单独用 gtest 测试。

---

## 2. 目录结构

```text
src/nav2_controller_lab/
├── CMakeLists.txt
├── package.xml
├── controller_plugins.xml
├── README.md
├── include/nav2_controller_lab/
│   ├── controller_types.hpp          # Pose2D / Control / 角度归一化
│   ├── path_utils.hpp                # 投影、lookahead、剩余距离
│   ├── trajectory_utils.hpp          # 差速模型 rollout
│   ├── trajectory_cost.hpp           # DWB / MPPI / MPC 共用 critics
│   ├── pid_core.hpp
│   ├── pure_pursuit_core.hpp
│   ├── stanley_core.hpp
│   ├── dwb_core.hpp
│   ├── mppi_core.hpp
│   ├── mpc_core.hpp
│   ├── controller_base.hpp           # ROS 适配层基类
│   ├── pid_controller.hpp
│   ├── pure_pursuit_controller.hpp
│   ├── stanley_controller.hpp
│   ├── dwb_controller.hpp
│   ├── mppi_controller.hpp
│   └── mpc_controller.hpp
├── src/                              # 与 include 一一对应
├── params/
│   └── nav2_params_controller_lab.yaml
├── launch/
│   └── controller_lab_launch.py
├── rviz/
│   └── controller_lab_view.rviz
└── test/
    ├── test_controller_core.cpp      # 24 个纯算法测试
    └── test_plugin_load.cpp          # pluginlib 加载测试
```

---

## 3. 编译与测试

```bash
cd ~/ros2_jazzy/nav2_astar_ws

source /opt/ros/jazzy/setup.bash
source ~/ros2_jazzy/install/setup.bash

# 同时编译全局规划器、控制器实验室和 RViz 面板
colcon build --packages-select \
  nav2_astar_planner nav2_controller_lab nav2_controller_lab_rviz \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash

# 纯算法 + 插件加载测试
colcon test --packages-select nav2_controller_lab
colcon test-result --verbose
```

期望：

```text
24 tests from 9 test suites ... PASSED
1 test from ControllerPlugins ... PASSED
```

测试内容：

- 路径投影、lookahead、剩余距离、rollout
- PID / Pure Pursuit / Stanley 的直行、偏移、到点
- DWB 的前进选择、碰撞拒绝
- MPPI 的前进选择、碰撞拒绝
- MPC 的前进、偏移、到点、碰撞拒绝
- pluginlib 能否创建全部 6 个控制器

---

## 4. 启动仿真

```bash
cd ~/ros2_jazzy/nav2_astar_ws
./run_controller_lab.sh
```

然后：

1. RViz 里点 **2D Pose Estimate**，在 `(-2.0, -0.5)` 拖出朝向；
2. 如果终端报 `Aborting bringup`，执行 `./activate_nav.sh`；
3. 点 **Nav2 Goal** 选目标。

默认控制器是 **Pure Pursuit**。

启动日志应出现：

```text
Created controller plugin FollowPath of type nav2_controller_lab::PurePursuitController
ControllerLab(FollowPath) 已配置: frame=odom -> base_link, freq=20.0 Hz, goal_tol=0.25 m
PurePursuitController(FollowPath): L=0.60 [0.30, 1.20] vmax=0.50 wmax=1.90
```

---

## 5. 切换控制器

编辑：

```text
src/nav2_controller_lab/params/nav2_params_controller_lab.yaml
```

找到 `controller_server.ros__parameters.FollowPath`，只改 `plugin` 和参数：

```yaml
FollowPath:
  plugin: "nav2_controller_lab::PurePursuitController"
  ...
```

### 5.1 PID 基线

```yaml
FollowPath:
  plugin: "nav2_controller_lab::PidController"
  transform_tolerance: 0.1
  goal_tolerance: 0.25

  kp_heading: 0.8
  ki_heading: 0.0
  kd_heading: 0.05
  kp_cross: 0.6
  kp_speed: 1.0
  ki_speed: 0.0
  kd_speed: 0.0
  max_linear_velocity: 0.5
  max_angular_velocity: 1.9
  slow_radius: 0.6
  heading_slowdown: 0.8
  integral_limit: 1.0
```

### 5.2 Pure Pursuit（默认）

```yaml
FollowPath:
  plugin: "nav2_controller_lab::PurePursuitController"
  transform_tolerance: 0.1
  goal_tolerance: 0.25

  lookahead_distance: 0.6
  min_lookahead_distance: 0.3
  max_lookahead_distance: 1.2
  use_velocity_scaled_lookahead: true
  lookahead_time: 1.0
  max_linear_velocity: 0.5
  min_linear_velocity: 0.05
  max_angular_velocity: 1.9
  slowdown_radius: 0.6
  curvature_slowdown: 0.7
```

### 5.3 Stanley

```yaml
FollowPath:
  plugin: "nav2_controller_lab::StanleyController"
  transform_tolerance: 0.1
  goal_tolerance: 0.25

  k_cross: 1.0
  k_heading: 1.0
  wheelbase: 0.5
  front_axle_offset: 0.0
  max_linear_velocity: 0.5
  min_linear_velocity: 0.05
  max_angular_velocity: 1.9
  max_steering_angle: 0.7
  slowdown_radius: 0.6
  heading_slowdown: 0.8
```

### 5.4 DWB-lite

```yaml
FollowPath:
  plugin: "nav2_controller_lab::DwbController"
  transform_tolerance: 0.1
  goal_tolerance: 0.25

  min_linear_velocity: 0.0
  max_linear_velocity: 0.5
  max_angular_velocity: 1.9
  linear_samples: 7
  angular_samples: 11
  horizon: 1.5
  model_dt: 0.1

  path_weight: 1.0
  goal_weight: 2.0
  obstacle_weight: 100.0
  heading_weight: 0.5
  velocity_weight: 0.2
  control_weight: 0.05
  collision_cost: 1000000.0
  collision_threshold: 254
  desired_linear_velocity: 0.5
```

### 5.5 MPPI-lite

```yaml
FollowPath:
  plugin: "nav2_controller_lab::MppiController"
  transform_tolerance: 0.1
  goal_tolerance: 0.25

  min_linear_velocity: 0.0
  max_linear_velocity: 0.5
  max_angular_velocity: 1.9
  time_steps: 20
  batch_size: 100
  model_dt: 0.1
  vx_std: 0.15
  wz_std: 0.35
  temperature: 0.3
  seed: 42

  path_weight: 1.0
  goal_weight: 2.0
  obstacle_weight: 100.0
  heading_weight: 0.5
  velocity_weight: 0.2
  control_weight: 0.05
  collision_cost: 1000000.0
  collision_threshold: 254
  desired_linear_velocity: 0.5
```

> 修改 `plugin` 类型或参数后必须重新启动 launch（参数在 configure 阶段读取）。
> 当前参数文件已经加载了 6 个实例，也可以用下面的运行时切换。

### 5.5b MPC-lite

```yaml
FollowPath:
  plugin: "nav2_controller_lab::MpcController"
  transform_tolerance: 0.1
  goal_tolerance: 0.34

  min_linear_velocity: 0.0
  max_linear_velocity: 0.5
  max_angular_velocity: 1.9
  horizon_steps: 15
  iterations: 8
  model_dt: 0.1
  max_linear_acceleration: 1.0
  max_angular_acceleration: 3.2
  terminal_weight: 2.0
  smoothness_weight: 0.05

  path_weight: 5.0
  goal_weight: 2.0
  obstacle_weight: 8.0
  heading_weight: 0.5
  velocity_weight: 0.2
  control_weight: 0.05
  collision_cost: 1000000.0
  collision_threshold: 253
  desired_linear_velocity: 0.5
```

采用多初值和确定性控制块搜索，所有候选按测量速度约束加速度并检查预测碰撞。普通重规划保留热启动；无可行解时清空旧解并报错。旧的 gradient_step、gradient_epsilon、line_search_steps 参数仅保留兼容，不参与新求解。

预测评分跟踪近期路径参考点，避免绕行时被全局终点距离困住；大转角先执行经过碰撞检查的路径对齐。末端按直线距离进入朝向调整，停止距离须小于 Nav2 的位置验收容差。

### 5.6 运行时切换（推荐）

参数文件现在已经一次性加载 6 个控制器实例：

```yaml
controller_plugins: ["FollowPath", "PID", "Stanley", "DWB", "MPPI", "MPC"]
```

- `FollowPath` = Pure Pursuit（默认）
- `PID` = PidController
- `Stanley` = StanleyController
- `DWB` = DwbController
- `MPPI` = MppiController
- `MPC` = MpcController

运行时不需要重启，向 `/controller_selector` 发布实例名即可：

```bash
ros2 topic pub --once /controller_selector std_msgs/msg/String \
  "{data: MPPI}" \
  --qos-durability transient_local \
  --qos-reliability reliable
```

切回 Pure Pursuit：

```bash
ros2 topic pub --once /controller_selector std_msgs/msg/String \
  "{data: FollowPath}" \
  --qos-durability transient_local \
  --qos-reliability reliable
```

> `/controller_selector` 的订阅端使用 `transient_local`，所以命令必须加
> `--qos-durability transient_local`，否则可能匹配不上。
>
> 该切换只影响下一次 `FollowPath` action 使用哪个控制器实例，
> 不会改变正在执行的旧 action。

### 5.7 RViz 面板

同工作空间的 `nav2_controller_lab_rviz` 提供：

- `Controller Lab` 面板：6 个按钮一键切换控制器；
- 显示当前控制器和 `/cmd_vel`；
- “显示调试标记”开关；
- `/controller_lab/markers`：
  - 绿色：全局路径
  - 黄色：前视点 + 机器人到前视点连线
  - 蓝色：按当前 `cmd_vel` 预测的 2 秒轨迹
  - 白色文字：控制器名、v/w

启动：

```bash
./run_controller_lab.sh
```

RViz 配置已经包含 `Controller Lab` 面板和 `Controller Lab Markers` 显示。
如果单独打开 RViz，使用：

```bash
rviz2 -d install/nav2_controller_lab/share/nav2_controller_lab/rviz/controller_lab_view.rviz
```

也可以手动添加 Panel：

```text
Panels → Add New Panel → nav2_controller_lab_rviz/Controller Lab
```

以及 MarkerArray 显示：

```text
Add → By topic → /controller_lab/markers → MarkerArray
```

> 修改源 `params/*.yaml` 后，launch 读取的是 `install/` 下的副本。
> 记得重新 `colcon build --packages-select nav2_controller_lab`，
> 或者使用 `colcon build --symlink-install` 开发。

---

## 6. 参数参考

### 6.1 公共参数

| 参数 | 默认 | 作用 |
|---|---:|---|
| `transform_tolerance` | `0.1` | TF 查询超时（秒） |
| `goal_tolerance` | `0.25` | 控制器内部的到点停止距离（米） |
| `controller_frequency` | 节点参数 `20.0` | 控制频率，决定 `dt` |

### 6.2 Pure Pursuit

| 参数 | 默认 | 调大效果 | 调小效果 |
|---|---:|---|---|
| `lookahead_distance` | `0.6` | 更稳、易切内弯 | 更贴线、易摆动 |
| `min/max_lookahead_distance` | `0.3/1.2` | 限制自适应范围 | 同上 |
| `use_velocity_scaled_lookahead` | `true` | 速度越快前视越远 | 固定前视 |
| `lookahead_time` | `1.0` | 高速时前视更远 | 更激进 |
| `max_linear_velocity` | `0.5` | 更快 | 更慢 |
| `max_angular_velocity` | `1.9` | 转弯更猛 | 转弯更保守 |
| `curvature_slowdown` | `0.7` | 弯道减速更明显 | 弯道更快 |

### 6.3 PID

| 参数 | 作用 |
|---|---|
| `kp_heading` / `ki_heading` / `kd_heading` | 航向 PID |
| `kp_cross` | 横向误差比例项 |
| `kp_speed` / `ki_speed` / `kd_speed` | 速度环 PID |
| `slow_radius` | 到点前减速半径 |
| `heading_slowdown` | 航向误差超过该值时减速 |
| `integral_limit` | 积分限幅，防 windup |

### 6.4 Stanley

| 参数 | 作用 |
|---|---|
| `k_cross` | 横向误差增益；越大越贴线，太大振荡 |
| `k_heading` | 航向误差增益 |
| `wheelbase` | 轴距；用于 `omega = v tan(delta) / L` |
| `front_axle_offset` | 前轴相对机器人中心的前向偏移 |
| `max_steering_angle` | 最大等效舵角 |

### 6.5 DWB / MPPI / MPC 的 critics 权重

| 参数 | 对应 critic | 调大效果 |
|---|---|---|
| `path_weight` | PathAlign / PathFollow | 更贴全局路径 |
| `goal_weight` | GoalCritic | 更积极冲向目标 |
| `obstacle_weight` | CostCritic | 更远离障碍 |
| `heading_weight` | PathAngleCritic | 更对齐路径切线 |
| `velocity_weight` | 速度偏好 | 更接近期望速度 |
| `control_weight` | 平滑/能耗 | 更偏向小角速度 |
| `collision_threshold` | 碰撞判定 | 253 更保守，254 更宽松 |

---

## 7. 算法说明（简版）

### 7.1 Pure Pursuit

```text
前视点 P（路径上，距离 L）
机器人坐标系下 P = (x, y)
曲率 kappa = 2y / (x² + y²)
omega = v * kappa
```

- `L` 太小 → 左右摆动；
- `L` 太大 → 切内弯；
- 推荐先调 `lookahead_distance`，再调速度。

### 7.2 Stanley

```text
delta = k_heading * heading_error
      - atan(k_cross * cross_track / (v + eps))
omega = v * tan(delta) / wheelbase
```

横向误差定义：机器人位于路径左侧为正，因此修正项取负号。

### 7.3 DWB-lite

```text
for v in [vmin, vmax]:
  for w in [-wmax, wmax]:
    traj = rollout(robot, v, w, horizon, dt)
    cost = path + goal + obstacle + heading + velocity + control
pick min cost
```

### 7.4 MPPI-lite

```text
维护标称控制序列 U
for k in 1..K:
  U_k = U + GaussianNoise
  traj_k = rollout(U_k)
  cost_k = critics(traj_k)
w_k = softmax(-cost_k / temperature)
U = Σ w_k U_k
执行 U[0]，滚动时域
```

---

## 8. 为什么有 PID，但不推荐只用 PID

PID 是反馈律，不是完整的路径跟踪器：

- 需要额外决定参考点（最近点？前视点？）；
- 直接对 `(x, y)` 做 PID 会试图让差速机器人横移；
- 不处理障碍、曲率、速度/加速度约束；
- 弯道有滞后，高速易振荡。

PID 更适合做内环（速度环、舵角环），或作为 Stanley 的横向误差项。
本包保留 `PidController` 是为了做对照实验，让你亲眼看到差异。

---

## 9. 坐标系与线程

- 全局路径：`map`；
- 局部代价地图 / 机器人位姿：`odom`；
- 适配层在每个控制周期用 TF 把路径逐点变换到 `odom`；
- DWB/MPPI/MPC 读局部代价地图时，由基类持有 `Costmap2D::mutex_t`；
- 控制器核心函数本身不加锁，便于单测。

---

## 10. 验证清单

```bash
# 1. ament 索引
cat install/nav2_controller_lab/share/ament_index/resource_index/\
nav2_core__pluginlib__plugin/nav2_controller_lab
# 期望：share/nav2_controller_lab/controller_plugins.xml

# 2. .so 符号
nm -D --defined-only install/nav2_controller_lab/lib/libnav2_controller_lab.so \
  | grep -E 'PurePursuitController|MppiController'

# 3. 运行时参数
ros2 param get /controller_server FollowPath.plugin
# 期望：nav2_controller_lab::PurePursuitController

ros2 param list /controller_server | grep FollowPath

# 4. 生命周期
ros2 lifecycle get /controller_server

# 5. 速度指令
ros2 topic echo /cmd_vel
```

---

## 11. 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| `Unable to find class` | ament 索引缺失 | 重新编译并 source；检查 package.xml/CMake |
| `Failed to load library` | XML/库名/宏不匹配 | 检查 `library path`、`PLUGINLIB_EXPORT_CLASS` |
| 参数全是默认值 | 少了 `FollowPath.` 前缀 | 看 `ros2 param list` |
| 控制器不转 | 路径 frame 与 pose frame 不一致 | `ros2 run tf2_ros tf2_echo map odom` |
| 到点不停 | `goal_tolerance` 太大/太小 | 调 `goal_tolerance` 和 goal_checker |
| Pure Pursuit 摆动 | `lookahead` 太小 | 增大 `min_lookahead_distance` |
| Pure Pursuit 切内弯 | `lookahead` 太大 | 减小 `max_lookahead_distance` |
| DWB 不动 | 所有采样轨迹都碰撞 | 调 `collision_threshold`、降低速度 |
| MPPI 很慢/抖动 | `batch_size` 太大/温度太低 | 降 `batch_size`，升 `temperature` |
| MPC 抖动/很慢 | 步长/迭代/ horizon 太大 | 降 `gradient_step` / `iterations` / `horizon_steps` |
| Stanley 高速振荡 | `k_cross` 太大 | 降 `k_cross`，加 `heading_slowdown` |

---

## 12. 练习

1. 固定地图和起点/终点，比较 6 个控制器的到达时间、横向误差、最大角速度。
2. 只改 `lookahead_distance`，画出“摆动 → 稳定 → 切内弯”的曲线。
3. 把 `PidController` 的 `ki_heading` 调大，观察积分 windup。
4. 让 DWB 的 `obstacle_weight` 从 0 调到 500，观察路径。
5. 把 MPPI 的 `temperature` 从 0.05 调到 2.0，观察平滑性和跟踪误差。
6. 给 DWB/MPPI/MPC 加一个 `prefer_forward` 项，惩罚倒车。
7. 用 `ros2 bag record /plan /cmd_vel /tf` 录一次，离线比较。

---

## 13. 代码阅读顺序

建议按这个顺序读：

```text
1. controller_types.hpp / path_utils.hpp
     位姿、路径投影、lookahead、剩余距离
2. pure_pursuit_core.hpp / .cpp
     最短的完整控制器，先理解“几何跟踪”
3. controller_base.hpp / .cpp
     看 ROS 数据怎么变成 Pose2D，路径怎么从 map 变到 odom
4. pure_pursuit_controller.hpp / .cpp
     看插件如何只做“读参数 + 调核心”
5. stanley_core、pid_core
     对比不同几何/反馈方法
6. trajectory_utils / trajectory_cost
     DWB/MPPI/MPC 的 rollout 和 critics
7. dwb_core、mppi_core
     从“单步采样”到“控制序列采样”
```

对应理论见 [docs/10_控制器算法零基础课.md](../../docs/10_控制器算法零基础课.md)，
源码逐行见 [docs/12_控制器源码逐行导读.md](../../docs/12_控制器源码逐行导读.md)，
实验见 [docs/09_控制器实验手册.md](../../docs/09_控制器实验手册.md)，
调参排错见 [docs/13_控制器实验调参与排错.md](../../docs/13_控制器实验调参与排错.md)。

---

## 14. 许可

Apache-2.0。教学实现；生产使用前请补充：

- 更严格的 footprint 碰撞检查；
- 动态参数回调；
- 控制器输出限幅/滤波；
- 更完整的集成测试和性能剖析。
