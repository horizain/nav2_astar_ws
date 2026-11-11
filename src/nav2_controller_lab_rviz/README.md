# nav2_controller_lab_rviz

`nav2_controller_lab` 的 RViz 面板与调试可视化，支持 6 种控制器切换。

## 1. 面板

`Controller Lab` 面板提供：

- 6 个按钮：Pure Pursuit / PID / Stanley / DWB-lite / MPPI-lite / MPC-lite；
- 点击后向 `/controller_selector` 发布实例名，运行时切换控制器；
- 显示当前控制器和 `/cmd_vel`；
- “显示调试标记”开关，控制 `/controller_lab/debug_enable`。

## 2. 可视化节点

`controller_lab_visualizer` 发布 `/controller_lab/markers`（MarkerArray）：

| 颜色 | 内容 |
|---|---|
| 绿色 | `/plan` 全局路径 |
| 黄色 | 前视点 + 机器人到前视点连线 |
| 蓝色 | 按当前 `/cmd_vel` 预测的 2 秒轨迹 |
| 白色文字 | 当前控制器名、v/w |

参数：

| 参数 | 默认 | 作用 |
|---|---:|---|
| `map_frame` | `map` | 路径为空 frame 时使用 |
| `robot_frame` | `base_link` | 机器人 frame |
| `lookahead_distance` | `0.6` | 前视距离（米） |
| `prediction_horizon` | `2.0` | 预测时长（秒） |
| `prediction_dt` | `0.1` | 预测步长 |
| `publish_rate` | `10.0` | 标记发布频率（Hz） |

## 3. 启动

通常由主 launch 自动启动：

```bash
cd ~/ros2_jazzy/nav2_astar_ws
./run_controller_lab.sh
```

单独启动可视化节点：

```bash
ros2 run nav2_controller_lab_rviz controller_lab_visualizer
```

RViz 中：

```text
Panels → Add New Panel → nav2_controller_lab_rviz/Controller Lab
Add → By topic → /controller_lab/markers → MarkerArray
```

## 4. 编译

```bash
colcon build --packages-select nav2_controller_lab nav2_controller_lab_rviz \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 5. 说明

- 面板需要 `rviz_common` 和 Qt5；
- 可视化节点复用 `nav2_controller_lab` 的路径/rollout 工具；
- 修改控制器参数后，如果 launch 读取的是 install 副本，记得重新编译主包，
  或使用 `colcon build --symlink-install`。
