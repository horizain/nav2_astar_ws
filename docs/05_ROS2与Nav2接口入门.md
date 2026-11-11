# ROS 2 与 Nav2 接口入门

> **前置要求**：会 C++ 基本语法即可。**不需要**先学完一整套 ROS 2 教程。
>
> **学完你应该能**：
> 1. 用大白话解释节点、话题、服务、动作、参数、TF、生命周期；
> 2. 画清楚一次导航从“点目标”到“轮子转”的完整数据流；
> 3. 读懂 `nav2_core::GlobalPlanner` 这个接口的每个参数和约定；
> 4. 说清 pluginlib 是如何在运行时找到并加载你的 `.so` 的；
> 5. 知道 Nav2 里其他插件（Controller / Smoother / Behavior）的接口长什么样；
> 6. 遇到“插件没加载 / 参数没读到 / TF 报错”时，知道用哪条命令查。

---

## 0. 先建立一张地图

ROS 2 不是“一个程序”，而是**一堆进程通过约定好的消息互相通信**。
Nav2 是跑在 ROS 2 上的一套导航系统，由很多节点组成。

你可以把整个系统想成一家公司：

```text
┌─────────────────────────────────────────────────────────────┐
│                          ROS 2                              │
│  节点 = 员工        话题 = 广播        服务 = 打电话         │
│  动作 = 派一个可跟踪、可取消的长任务    参数 = 员工手册       │
│  TF = 公司平面图（谁在哪个坐标系）                           │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                          Nav2                               │
│  bt_navigator   = 项目经理（行为树编排）                     │
│  planner_server = 规划组（算全局路径）                       │
│  controller_server = 控制组（沿着路径走）                    │
│  costmap        = 地图/障碍信息                              │
│  你的插件       = 规划组里一个可替换的“算法专家”             │
└─────────────────────────────────────────────────────────────┘
```

**最重要的一句话**：

> 你写的 `AStarPlanner` **不是一个节点**。它是一个被 `planner_server`
> 这个生命周期节点在运行时创建、配置、激活、调用的**类**。
> 它通过 `pluginlib` 被“插”进 `planner_server`。

---

## 1. ROS 2 最小概念

### 1.1 节点（Node）

节点是 ROS 2 里的基本执行单元。一个进程里可以有一个或多个节点。

```cpp
auto node = std::make_shared<rclcpp::Node>("my_node");
rclcpp::spin(node);   // 让节点开始处理回调
```

- **名字**：`/planner_server`、`/controller_server`、`/amcl`。
- **命名空间**：多机器人时用来隔离，例如 `/robot1/planner_server`。
- **执行器（executor）**：决定回调在哪个线程跑。
- **spin**：让节点“转起来”，处理订阅、服务、定时器。

> 写插件时你通常**不创建节点**，而是拿到父节点的 `WeakPtr`，
> 用它读参数、拿 logger、拿 clock。父节点就是 `planner_server`。

### 1.2 话题（Topic）：广播，单向，异步

发布者（publisher）往一个话题发消息，订阅者（subscriber）收。
发布者不关心谁在听，也不等待回复。

```text
publisher ──► /plan ──► subscriber
              (nav_msgs/msg/Path)
```

Nav2 里常见话题：

| 话题 | 类型 | 作用 |
|---|---|---|
| `/plan` | `nav_msgs/msg/Path` | 全局规划器输出的路径 |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 全局代价地图（可视化用） |
| `/global_costmap/costmap_raw` | `nav2_msgs/msg/Costmap` | 全局代价地图原始数据 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 局部代价地图 |
| `/tf`、`/tf_static` | `tf2_msgs/msg/TFMessage` | 坐标变换 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 速度指令 |

查看：

```bash
ros2 topic list -t
ros2 topic info /plan
ros2 topic echo /plan --once
```

**QoS（服务质量）**：
话题还有“可靠性、历史深度、是否保留最后一条”等策略。
例如 `/tf_static` 用 `transient_local`，晚加入的订阅者也能收到之前的静态变换。
初学阶段记住：**收不到话题时，先怀疑 QoS，再怀疑网络**。

### 1.3 服务（Service）：请求/响应，一问一答

服务像函数调用：客户端发请求，服务端算完返回响应。

```text
client ──► request ──► server
client ◄── response ◄── server
```

Nav2 里常见服务：

| 服务 | 类型 | 作用 |
|---|---|---|
| `/planner_server/get_state` | `lifecycle_msgs/srv/GetState` | 查生命周期状态 |
| `/planner_server/change_state` | `lifecycle_msgs/srv/ChangeState` | 切生命周期状态 |
| `/planner_server/is_path_valid` | `nav2_msgs/srv/IsPathValid` | 检查路径是否还有效 |
| `/global_costmap/clear_entirely_global_costmap` | `nav2_msgs/srv/ClearEntireCostmap` | 清代价地图 |

查看：

```bash
ros2 service list -t
ros2 service type /planner_server/get_state
ros2 service call /planner_server/get_state lifecycle_msgs/srv/GetState "{}"
```

### 1.4 动作（Action）：可跟踪、可取消的长任务

动作 = 目标（goal） + 反馈（feedback） + 结果（result） + 可取消（cancel）。

```text
client ──► goal ──────────────► server
client ◄── feedback（过程）──── server
client ◄── result（最终）────── server
client ──► cancel（随时）─────► server
```

为什么规划用动作而不是服务？

- 规划可能耗时几百毫秒到几秒；
- 用户可能中途改目标或取消；
- 上层（行为树）需要知道“进行到哪了”。

Nav2 里最关键的动作：

| 动作 | 类型 | 作用 |
|---|---|---|
| `/compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | 算一条到目标位姿的路径 |
| `/compute_path_through_poses` | `nav2_msgs/action/ComputePathThroughPoses` | 经过多个途经点 |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 完整导航（规划+控制） |
| `/follow_path` | `nav2_msgs/action/FollowPath` | 跟踪一条路径 |

看接口：

```bash
ros2 interface show nav2_msgs/action/ComputePathToPose
```

输出（本机 Jazzy）：

```text
#goal definition
geometry_msgs/PoseStamped goal
geometry_msgs/PoseStamped start
string planner_id
bool use_start   # false 用机器人当前位姿，true 用上面的 start

#result definition
uint16 NONE=0
uint16 UNKNOWN=200
uint16 INVALID_PLANNER=201
uint16 TF_ERROR=202
uint16 START_OUTSIDE_MAP=203
uint16 GOAL_OUTSIDE_MAP=204
uint16 START_OCCUPIED=205
uint16 GOAL_OCCUPIED=206
uint16 TIMEOUT=207
uint16 NO_VALID_PATH=208

nav_msgs/Path path
builtin_interfaces/Duration planning_time
uint16 error_code
string error_msg

#feedback definition
```

> 注意 `planner_id`：它对应 YAML 里 `planner_plugins: ["GridBased"]` 的实例名。
> 行为树可以指定用哪个规划器；为空且只有一个插件时，用唯一那个。

### 1.5 参数（Parameter）：节点的配置

参数是键值对，通常从 YAML 加载：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_astar_planner::AStarPlanner"
      allow_diagonal: true
```

代码里要先声明，再读取：

```cpp
node->declare_parameter("my_param", 1.0);
double value = node->get_parameter("my_param").as_double();
```

查看/修改：

```bash
ros2 param list /planner_server
ros2 param get /planner_server GridBased.allow_diagonal
ros2 param set /planner_server GridBased.allow_diagonal false
```

> 参数名写错**不会报错**，只会一直用默认值。这是 Nav2 插件最常见的坑之一。

### 1.6 TF：坐标变换

机器人系统里有多个坐标系：

```text
map（地图全局坐标）
  └── odom（里程计连续坐标，会漂移）
        └── base_link（机器人本体）
              ├── laser（雷达）
              ├── camera（相机）
              └── ...
```

- `map → odom`：由 AMCL/定位发布，修正里程计漂移；
- `odom → base_link`：由轮式里程计发布；
- `base_link → laser`：由 `robot_state_publisher` 根据 URDF 发布。

**为什么重要？**

- 规划器收到的起点/终点必须在**同一个全局坐标系**（通常是 `map`）；
- 如果 `map → base_link` 查不到，`planner_server` 激活会失败；
- 如果 AMCL 初始位姿设错，机器人会被“传送”到墙里，规划报“无路可走”。

查看：

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames     # 生成 frames.pdf
ros2 topic echo /tf_static --once
```

### 1.7 Launch：一次启动多个节点

Launch 文件用 Python 写，负责：

- 启动多个节点；
- 传参数文件；
- 设置环境变量；
- 条件启动（比如 `headless` 时不开 Gazebo GUI）。

本项目的 `launch/astar_simulation_launch.py` 没有重写整套 Nav2，
而是 `IncludeLaunchDescription` 复用官方 `tb3_simulation_launch.py`，
只覆盖 `params_file`。这是最稳的做法。

### 1.8 概念速查表

| 概念 | 一句话 | 方向 | 有返回值 | Nav2 例子 | 查看命令 |
|---|---|---|---|---|---|
| 节点 | 一个可执行单元 | - | - | `/planner_server` | `ros2 node list` |
| 话题 | 广播 | 单向 | 否 | `/plan` | `ros2 topic echo` |
| 服务 | 一问一答 | 双向 | 是 | `/get_state` | `ros2 service call` |
| 动作 | 可跟踪长任务 | 双向 | 是 | `/compute_path_to_pose` | `ros2 action send_goal` |
| 参数 | 配置键值 | - | - | `GridBased.allow_diagonal` | `ros2 param get` |
| TF | 坐标变换 | - | 查询 | `map → base_link` | `tf2_echo` |
| Launch | 启动编排 | - | - | `astar_simulation_launch.py` | `ros2 launch ...` |

---

## 2. 你必须认识的几条消息

### 2.1 `geometry_msgs/msg/PoseStamped`

表示“带时间戳和坐标系的位姿”。

```text
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id
geometry_msgs/Pose pose
  geometry_msgs/Point position      # x, y, z
  geometry_msgs/Quaternion orientation  # x, y, z, w
```

- `frame_id` 通常是 `map`；
- `position` 是米；
- `orientation` 是**四元数**，不是欧拉角。

**yaw → 四元数**：

```cpp
double yaw = std::atan2(dy, dx);
q.x = 0.0;
q.y = 0.0;
q.z = std::sin(yaw / 2.0);
q.w = std::cos(yaw / 2.0);
```

**四元数 → yaw**：

```cpp
double yaw = std::atan2(
  2.0 * (q.w * q.z + q.x * q.y),
  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
```

> 平面导航通常只关心 yaw，但消息里必须存完整四元数。

### 2.2 `nav_msgs/msg/Path`

```text
std_msgs/Header header
geometry_msgs/PoseStamped[] poses
```

这就是你的插件最终要返回的东西：

- `header.frame_id` 必须和起点/终点一致（通常是 `map`）；
- `poses` 必须非空；
- 第一个点应接近起点，最后一个点应接近终点；
- 每个点的 `orientation` 应尽量沿路径切线，方便控制器跟踪。

### 2.3 `nav2_msgs/msg/Costmap`

`/global_costmap/costmap_raw` 的类型，包含：

```text
std_msgs/Header header
nav2_msgs/CostmapMetaData metadata
  builtin_interfaces/Time map_load_time
  builtin_interfaces/Time update_time
  string layer
  float32 resolution
  uint32 size_x
  uint32 size_y
  geometry_msgs/Pose origin
uint8[] data
```

`data` 就是每个格子的 0~255 代价，按行优先排列（`index = y * size_x + x`）。

### 2.4 `nav2_msgs/action/ComputePathToPose`

已经在 1.4 看过。记住：

- 输入有 `start`、`goal`、`planner_id`、`use_start`；
- 输出有 `path`、`planning_time`、`error_code`、`error_msg`；
- `error_code` 和插件抛出的异常类型一一对应。

---

## 3. Nav2 的整体架构

### 3.1 一张图看懂

```text
                        ┌─────────────────────┐
                        │   用户 / RViz        │
                        │  “Nav2 Goal”        │
                        └──────────┬──────────┘
                                   │ NavigateToPose action
                                   ▼
                        ┌─────────────────────┐
                        │    bt_navigator     │
                        │   （行为树编排）      │
                        └──────────┬──────────┘
                                   │ ComputePathToPose action
                                   ▼
                        ┌─────────────────────┐
                        │   planner_server    │
                        │  （生命周期节点）     │
                        │                     │
                        │  planner_plugins:   │
                        │    ["GridBased"]    │
                        │         │           │
                        │         ▼           │
                        │  ┌───────────────┐  │
                        │  │ 你的 A* 插件   │  │
                        │  └───────┬───────┘  │
                        └──────────┼──────────┘
                                   │ /plan (nav_msgs/Path)
                                   ▼
                        ┌─────────────────────┐
                        │  controller_server  │
                        │  （局部控制）        │
                        └──────────┬──────────┘
                                   │ /cmd_vel
                                   ▼
                        ┌─────────────────────┐
                        │   机器人底盘         │
                        └─────────────────────┘
```

### 3.2 各节点职责

| 节点 | 职责 | 关键输入 | 关键输出 |
|---|---|---|---|
| `map_server` | 发布静态地图 | 地图 YAML/图片 | `/map` |
| `amcl` | 定位 | 激光、地图、初始位姿 | `map→odom` TF |
| `global_costmap` | 全局代价地图 | 地图、障碍、膨胀层 | 代价数据 |
| `local_costmap` | 局部代价地图 | 传感器、机器人足迹 | 代价数据 |
| `planner_server` | 全局规划 | 起点、终点、全局代价地图 | `/plan` |
| `controller_server` | 局部控制 | `/plan`、局部代价地图、当前速度 | `/cmd_vel` |
| `smoother_server` | 路径平滑 | 原始路径 | 平滑后的路径 |
| `behavior_server` | 恢复行为 | 行为树调用 | 旋转、后退、等待等 |
| `bt_navigator` | 行为树编排 | 导航目标 | 调用各服务器 |
| `lifecycle_manager` | 生命周期管理 | 节点列表 | configure/activate 等 |

### 3.3 一次规划的完整调用链

```text
1. 用户在 RViz 点目标
2. bt_navigator 收到 NavigateToPose
3. 行为树执行 ComputePathToPose 节点
4. 该节点向 /compute_path_to_pose 发 action goal
5. planner_server 的 action 回调被触发
6. planner_server：
     - 检查自己是否 active
     - 等代价地图就绪
     - 把 start/goal 转换到全局坐标系
     - 根据 planner_id 找到插件实例
7. 调用你的 createPlan(start, goal, cancel_checker)
8. 你的插件：
     - 加锁，取 costmap
     - worldToMap 转格子
     - 调 A* 算法
     - 格子路径插值成 nav_msgs::Path
     - 返回
9. planner_server 校验路径非空，发布 /plan，返回 action result
10. controller_server 订阅 /plan，开始跟踪
11. controller_server 输出 /cmd_vel
```

> 注意：**规划器只负责“算路径”，不负责“让机器人动”**。
> 让机器人动的是 `controller_server`。这是初学者常混的地方。

### 3.4 全局规划 vs 局部规划

| | 全局规划 | 局部规划 |
|---|---|---|
| 节点 | `planner_server` | `controller_server` |
| 地图 | 全局代价地图 | 局部代价地图 |
| 目标 | 从当前位置到最终目标 | 跟踪全局路径的一小段 |
| 输出 | `nav_msgs/Path` | `geometry_msgs/TwistStamped` |
| 频率 | 较低（按需/重规划） | 较高（通常 10~20 Hz） |
| 本项目 | A* 插件 | DWB（官方） |

### 3.5 为什么需要行为树

导航不是“算一条路就完事”。真实场景会遇到：

- 路径被新障碍挡住；
- 机器人卡住；
- 目标点在障碍上；
- 定位丢失。

行为树负责决定“先规划、再跟踪；失败就清代价地图、重试；还不行就旋转/后退”。
你的插件抛出的异常类型，会影响行为树选择哪种恢复行为。

---

## 4. 生命周期节点（Lifecycle Node）

### 4.1 为什么普通节点不够

Nav2 的节点不能“一启动就干活”。它需要：

1. 先加载参数、创建插件；
2. 等 TF、地图、传感器就绪；
3. 再开始对外提供服务；
4. 出问题时能安全地停掉、重配。

所以 Nav2 用**生命周期节点**，有明确的状态机：

```text
                 configure                activate
unconfigured ──────────────► inactive ──────────────► active
     ▲                          │  ▲                    │
     │       cleanup            │  │    deactivate      │
     └──────────────────────────┘  └────────────────────┘
```

实际状态：

| 状态 | 含义 |
|---|---|
| `unconfigured` | 刚创建，什么都没准备 |
| `inactive` | 已配置，但不对外服务 |
| `active` | 正常工作 |
| `finalized` | 已销毁 |

### 4.2 插件方法什么时候被调用

`planner_server` 在自己的生命周期回调里调用插件：

| `planner_server` 状态转换 | 插件方法 | 你该做什么 |
|---|---|---|
| `on_configure` | `configure(parent, name, tf, costmap_ros)` | 读参数、存指针、创建算法对象 |
| `on_activate` | `activate()` | 启动线程/定时器（本项目不需要） |
| `on_deactivate` | `deactivate()` | 停线程，保证不再被调用 |
| `on_cleanup` | `cleanup()`，随后插件对象被销毁 | 释放算法对象、线程、资源；析构函数要安全 |
| 节点析构 | 插件析构函数 | 确保不泄漏、不留后台线程 |

> `configure` 只会调用一次（每次 configure 周期），**不要**把每次规划都要做的
> 事情放进去；`createPlan` 才是每次请求都调用的。

### 4.3 为什么激活会失败

常见原因：

- TF 查不到 `map → base_link`；
- 代价地图还没收到第一帧；
- 插件 `configure` 抛异常；
- 参数类型不对。

本项目就遇到过：AMCL 没收到初始位姿 → 没有 `map→odom` →
`global_costmap` 激活失败 → `planner_server` 激活失败 → 整个 bringup abort。

### 4.4 常用命令

```bash
# 查所有生命周期节点状态
ros2 lifecycle get --no-daemon --spin-time 5

# 查单个节点
ros2 lifecycle get /planner_server

# 手动激活
ros2 lifecycle set /planner_server activate

# 手动去激活
ros2 lifecycle set /planner_server deactivate
```

> 本机 ros2cli daemon 有 DDS 发现缓存问题，查不到节点时加 `--no-daemon`；
> 发现太慢时加 `--spin-time 10`。

---

## 5. pluginlib：插件是怎么被“发现”的

### 5.1 问题：服务器怎么认识你的类？

`planner_server` 在编译时**不可能**知道未来会有哪些规划器。
它只知道一个抽象基类 `nav2_core::GlobalPlanner`。

你的 `AStarPlanner` 在另一个 `.so` 里，编译时和 `planner_server` 没有链接关系。
运行时怎么把它“插”进去？

答案：**pluginlib + 约定**。

### 5.2 完整链路

```text
① 编译期：导出插件描述
   CMakeLists.txt:
     pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)
   package.xml:
     <nav2_core plugin="${prefix}/global_planner_plugin.xml"/>
        │
        ▼
   install/.../share/ament_index/resource_index/
       nav2_core__pluginlib__plugin/nav2_astar_planner
       内容：share/nav2_astar_planner/global_planner_plugin.xml

② 运行期：pluginlib 扫描
   ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core", "nav2_core::GlobalPlanner");
   遍历 ament index，找到所有 base_class_type 匹配的 XML
        │
        ▼
   读到：
     <library path="nav2_astar_planner">
       <class type="nav2_astar_planner::AStarPlanner"
              base_class_type="nav2_core::GlobalPlanner">

③ 创建实例
   loader.createUniqueInstance("nav2_astar_planner::AStarPlanner")
        │
        ▼
   dlopen("libnav2_astar_planner.so")
        │
        ▼
   查找 PLUGINLIB_EXPORT_CLASS 注册的工厂符号
        │
        ▼
   构造 AStarPlanner 对象，返回 GlobalPlanner 指针
```

### 5.3 四件套，缺一不可

| 文件/代码 | 作用 | 缺失后果 |
|---|---|---|
| `package.xml` 里的 `<nav2_core plugin="..."/>` | 告诉 ament：这个包导出了 nav2_core 插件 | ament 索引没有记录，报 `Unable to find class` |
| `CMakeLists.txt` 里的 `pluginlib_export_plugin_description_file(...)` | 把 XML 登记到 ament index | 同上 |
| `global_planner_plugin.xml` | 描述“哪个库、哪个类、继承哪个基类” | pluginlib 不知道去哪找 |
| `.cpp` 末尾的 `PLUGINLIB_EXPORT_CLASS(...)` | 在 `.so` 里生成工厂符号 | 能 dlopen，但 `Failed to load library` |

### 5.4 四个名字不要搞混

以本项目为例：

```xml
<library path="nav2_astar_planner">
  <class type="nav2_astar_planner::AStarPlanner"
         base_class_type="nav2_core::GlobalPlanner">
    <description>基于 A* 算法的全局路径规划器（教学实现）</description>
  </class>
</library>
```

| 名字 | 值 | 含义 |
|---|---|---|
| `library path` | `nav2_astar_planner` | `.so` 文件名去掉 `lib` 前缀和 `.so` 后缀 |
| `type` | `nav2_astar_planner::AStarPlanner` | C++ 类的完整名字（命名空间 + 类名） |
| `base_class_type` | `nav2_core::GlobalPlanner` | 抽象基类的完整名字 |
| YAML 里的 `plugin` | `nav2_astar_planner::AStarPlanner` | 和 `type` 一致 |

编译出的库是：

```text
install/nav2_astar_planner/lib/libnav2_astar_planner.so
                                └─────────────┘
                                library path 就是去掉 lib 和 .so
```

### 5.5 验证插件登记成功

```bash
source install/setup.bash

# 1. ament 索引是否存在
cat install/nav2_astar_planner/share/ament_index/resource_index/\
nav2_core__pluginlib__plugin/nav2_astar_planner
# 期望：share/nav2_astar_planner/global_planner_plugin.xml

# 2. .so 是否导出了类符号
nm -D --defined-only install/nav2_astar_planner/lib/libnav2_astar_planner.so \
  | grep AStarPlanner

# 3. 运行时是否加载（启动后看日志）
# 期望：Created global planner plugin GridBased of type nav2_astar_planner::AStarPlanner
```

### 5.6 常见失败现象对照

| 现象 | 最可能的原因 |
|---|---|
| `Unable to find class ...` | ament 索引没有，或没 source 工作空间 |
| `Failed to load library ...` | `.so` 没找到、`library path` 写错、缺 `PLUGINLIB_EXPORT_CLASS` |
| `class_loader` 警告 + 退出崩溃 | 关闭时的已知现象，通常不影响导航 |
| 插件加载了但参数全是默认值 | 参数名没加实例前缀 |
| `plugin is not a valid planner` | `planner_id` 和 `planner_plugins` 里的实例名不一致 |

---

## 6. 参数：Nav2 插件的第二个大坑

### 6.1 YAML 的层级

```yaml
planner_server:                 # 节点名
  ros__parameters:              # 固定写法
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]   # 插件实例名列表
    costmap_update_timeout: 1.0

    GridBased:                   # 实例名
      plugin: "nav2_astar_planner::AStarPlanner"
      allow_diagonal: true
      allow_unknown: true
      heuristic_weight: 1.0
```

实际参数名是：

```text
/planner_server:
  expected_planner_frequency
  planner_plugins
  GridBased.plugin
  GridBased.allow_diagonal
  GridBased.allow_unknown
  GridBased.heuristic_weight
```

### 6.2 为什么参数必须带实例名前缀

Nav2 允许**同时加载多个规划器**：

```yaml
planner_plugins: ["GridBased", "AnotherPlanner"]
GridBased:
  plugin: "nav2_astar_planner::AStarPlanner"
  allow_diagonal: true
AnotherPlanner:
  plugin: "nav2_navfn_planner::NavfnPlanner"
  tolerance: 0.5
```

两个插件可能都有 `allow_diagonal` 或 `tolerance`，不加前缀就冲突了。
所以每个插件只读自己前缀下的参数。

`configure` 收到的 `name` 就是实例名：

```cpp
void configure(..., std::string name, ...) {
    name_ = name;   // name == "GridBased"
    nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".allow_diagonal", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + ".allow_diagonal", allow_diagonal_);
}
```

**如果写成 `"allow_diagonal"`（没有 `name_ + "."`）**：

- 代码不报错；
- 参数永远读不到；
- 一直用默认值；
- 你会以为“调参没用”。

> 这是 Nav2 插件最难察觉的 bug 之一。验证方法：
> `ros2 param get /planner_server GridBased.allow_diagonal`。

### 6.3 参数类型要匹配

```cpp
// 默认值是 int
declare_parameter_if_not_declared(node, name_ + ".max_expansions",
                                  rclcpp::ParameterValue(1000000));
int max_expansions;
node->get_parameter(name_ + ".max_expansions", max_expansions);

// 默认值是 double
declare_parameter_if_not_declared(node, name_ + ".heuristic_weight",
                                  rclcpp::ParameterValue(1.0));
double heuristic_weight;
node->get_parameter(name_ + ".heuristic_weight", heuristic_weight);
```

如果 YAML 写 `heuristic_weight: 1`（整数），而代码按 double 读，
可能报类型不匹配。**YAML 里该写 `1.0` 就写 `1.0`**。

### 6.4 动态参数（选做）

如果你希望运行中改参数立即生效，可以在 `activate()` 里注册回调：

```cpp
dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&MyPlanner::dynamicParametersCallback, this, _1));
```

在 `deactivate()` 里移除。初学阶段可以先不做，重启节点生效即可。

---

## 7. 代价地图（Costmap）接口

### 7.1 `Costmap2DROS` 和 `Costmap2D` 的区别

| 类型 | 是什么 | 你用它做什么 |
|---|---|---|
| `nav2_costmap_2d::Costmap2DROS` | 一个生命周期节点，管理图层、发布话题、处理 TF | `getCostmap()` 拿原始数据 |
| `nav2_costmap_2d::Costmap2D` | 纯数据：尺寸、分辨率、原点、代价数组 | 读格子、坐标转换 |

`configure` 会收到 `std::shared_ptr<Costmap2DROS> costmap_ros`。
`createPlan` 里：

```cpp
costmap_ = costmap_ros_->getCostmap();
```

### 7.2 常用 API

| 方法 | 作用 |
|---|---|
| `getSizeInCellsX()` / `getSizeInCellsY()` | 地图宽高（格子数） |
| `getResolution()` | 米/格 |
| `getOriginX()` / `getOriginY()` | 地图左下角世界坐标 |
| `getCharMap()` | 指向 `unsigned char[]` 代价数组的指针 |
| `getCost(mx, my)` | 取某格代价 |
| `worldToMap(wx, wy, mx, my)` | 世界坐标 → 格子；越界返回 false |
| `mapToWorld(mx, my, wx, wy)` | 格子 → 世界坐标 |
| `getGlobalFrameID()` | 全局坐标系名字（通常是 `map`） |
| `getMutex()` | 代价地图锁 |

### 7.3 必须加锁

代价地图有一个**更新线程**在后台不断改数据。`createPlan` 读地图时必须持锁：

```cpp
std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
    *(costmap_->getMutex()));
```

否则可能读到“更新到一半”的地图：

- 一半是旧障碍，一半是新障碍；
- 规划结果飘忽；
- 偶尔穿墙。

> 锁的粒度要尽量小：读数据、拷贝到本地 `vector`，然后可以早点释放锁再跑算法。
> 本项目为了简单，整个 `createPlan` 都持锁；教学够用，生产环境可以优化。

### 7.4 为什么要拷贝一份代价数组

```cpp
const unsigned char * char_map = costmap_->getCharMap();
std::vector<uint8_t> costs(char_map, char_map + size_x * size_y);
astar_->setCostmap(costs, size_x, size_y);
```

原因：

1. 算法层不想依赖 `nav2_costmap_2d`；
2. 拷贝后即使锁释放，算法也能安全读；
3. 算法可以脱离 ROS 单测。

384×384 的拷贝约 147 KB，几毫秒内完成，可以接受。

### 7.5 坐标系约定

`planner_server` 在调用你的插件之前，已经把 `start`/`goal`
转换到全局坐标系（`costmap_ros_->getGlobalFrameID()`，通常是 `map`）。

所以你的插件可以假设：

```text
start.pose.position 和 goal.pose.position 已经在 global_frame_ 下
```

但**不要**假设它们在 `odom` 或 `base_link` 下。
返回的 `Path.header.frame_id` 也必须是 `global_frame_`。

---

## 8. `GlobalPlanner` 接口契约

### 8.1 接口原文

位置：`/opt/ros/jazzy/include/nav2_core/global_planner.hpp`

```cpp
class GlobalPlanner
{
public:
  using Ptr = std::shared_ptr<GlobalPlanner>;
  virtual ~GlobalPlanner() {}

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  virtual void cleanup() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;

  virtual nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) = 0;
};
```

### 8.2 每个参数的含义

| 参数 | 类型 | 你该怎么用 |
|---|---|---|
| `parent` | `LifecycleNode::WeakPtr` | `lock()` 后读参数、拿 logger/clock；**不要**长期持有 `shared_ptr` 造成循环引用 |
| `name` | `std::string` | 插件实例名，参数前缀 |
| `tf` | `shared_ptr<tf2_ros::Buffer>` | 需要查变换时用；规划器通常不需要，因为 start/goal 已转换 |
| `costmap_ros` | `shared_ptr<Costmap2DROS>` | `getCostmap()` 拿地图；`getGlobalFrameID()` 拿坐标系 |
| `start` | `PoseStamped` | 起点，已在全局坐标系 |
| `goal` | `PoseStamped` | 终点，已在全局坐标系 |
| `cancel_checker` | `std::function<bool()>` | 返回 true 表示用户取消；长循环里要定期检查 |

### 8.3 输入/输出契约

**输入**：

- `start`、`goal` 的 `header.frame_id` 是全局坐标系；
- `start`/`goal` 的 `position` 单位是米；
- 代价地图已配置好，可以读。

**输出**：

- 返回 `nav_msgs::msg::Path`；
- `header.frame_id` 必须是全局坐标系；
- `header.stamp` 用当前时间；
- `poses` 非空；
- 第一个点接近 `start`，最后一个点接近 `goal`；
- 每个点的朝向尽量沿路径切线（除非你想让终点朝向目标朝向）。

**失败**：

- **抛异常**，不要返回空 `Path`；
- 异常类型决定了 `ComputePathToPose` 的 `error_code`，也决定行为树怎么恢复。

### 8.4 异常类型对照表

| 情况 | 抛什么 | `error_code` |
|---|---|---|
| 起点越界 | `nav2_core::StartOutsideMapBounds` | `START_OUTSIDE_MAP` |
| 终点越界 | `nav2_core::GoalOutsideMapBounds` | `GOAL_OUTSIDE_MAP` |
| 起点在障碍上 | `nav2_core::StartOccupied` | `START_OCCUPIED` |
| 终点在障碍上 | `nav2_core::GoalOccupied` | `GOAL_OCCUPIED` |
| 无可行路径 | `nav2_core::NoValidPathCouldBeFound` | `NO_VALID_PATH` |
| 超时 | `nav2_core::PlannerTimedOut` | `TIMEOUT` |
| TF 错误 | `nav2_core::PlannerTFError` | `TF_ERROR` |
| 被取消 | `nav2_core::PlannerCancelled` | 无（terminate） |
| 其他 | `nav2_core::PlannerException` 或 `std::exception` | `UNKNOWN` |

> 这些异常定义在 `nav2_core/planner_exceptions.hpp`。
> `planner_server` 会 `catch` 它们并填 `error_code`。**不要自己吞掉异常**，
> 否则行为树会以为规划成功了。

### 8.5 调用频率与线程

- `createPlan` 由 `planner_server` 的 action 回调调用；
- 不是每 20 Hz 都调用；`expected_planner_frequency` 只是期望/告警阈值；
- 行为树可能在路径失效、目标更新时重规划；
- `createPlan` 可能被并发调用（多个 action），你的实现要线程安全；
- 代价地图更新线程会并发改地图，读地图要加锁。

### 8.6 不要做的事

| 不要 | 为什么 |
|---|---|
| 在插件里创建并 spin 一个新节点 | 生命周期和 executor 会乱；用父节点 |
| 在 `createPlan` 里阻塞等 TF 很久 | 会卡住 action；用超时/异常 |
| 返回空 `Path` 表示失败 | 上层可能当成功；要抛异常 |
| 在 `configure` 里读每次规划都要变的参数 | 参数更新不会自动生效 |
| 在 `createPlan` 里改 `costmap` | 你只是读者，不是所有者 |
| 把格子坐标直接当世界坐标 | 差 `resolution`、`origin`、`+0.5` |
| 在日志里每个节点都打印 | 大地图会刷屏；用 DEBUG 或汇总 |

---

## 9. 其他 Nav2 插件长什么样

你学会 `GlobalPlanner` 后，其他插件是同一套路，只是“核心方法”不同。

### 9.1 插件类型对照

| 插件类型 | 基类 | 头文件 | 核心方法 | 由谁加载 | YAML 列表 |
|---|---|---|---|---|---|
| 全局规划器 | `nav2_core::GlobalPlanner` | `nav2_core/global_planner.hpp` | `createPlan` | `planner_server` | `planner_plugins` |
| 控制器 | `nav2_core::Controller` | `nav2_core/controller.hpp` | `computeVelocityCommands` | `controller_server` | `controller_plugins` |
| 平滑器 | `nav2_core::Smoother` | `nav2_core/smoother.hpp` | `smooth` | `smoother_server` | `smoother_plugins` |
| 恢复行为 | `nav2_core::Behavior` | `nav2_core/behavior.hpp` | 由 `TimedBehavior` 提供 | `behavior_server` | `behavior_plugins` |
| 目标检查 | `nav2_core::GoalChecker` | `nav2_core/goal_checker.hpp` | `isGoalReached` | `controller_server` | `goal_checker_plugins` |
| 进度检查 | `nav2_core::ProgressChecker` | `nav2_core/progress_checker.hpp` | `check` | `controller_server` | `progress_checker_plugins` |

### 9.2 控制器插件的最小骨架

```cpp
class MyController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
};
```

核心方法要返回 `TwistStamped`（线速度 + 角速度），并在 `setPlan` 里保存全局路径。

### 9.3 平滑器插件的最小骨架

```cpp
class MySmoother : public nav2_core::Smoother
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  bool smooth(
    nav_msgs::msg::Path & path,
    const rclcpp::Duration & max_time) override;
};
```

### 9.4 恢复行为插件

`nav2_core::Behavior` 的 `configure` 会收到两个碰撞检查器，
另外还要实现 `getResourceInfo()`。真正执行动作时，Nav2 通常用
`nav2_behaviors::TimedBehavior<ActionT>` 模板来封装一个 action server。
这比规划器复杂，建议先看懂 `nav2_behaviors` 里的 `Spin`、`BackUp` 再动手。

### 9.5 通用八步法

不管哪种插件，流程都是：

```text
1. 找到基类头文件，读纯虚函数
2. 创建 ament_cmake 包
3. 写一个类继承基类，实现所有纯虚函数
4. 在 .cpp 末尾 PLUGINLIB_EXPORT_CLASS
5. 写插件 XML（library / type / base_class_type）
6. package.xml 加导出标签
7. CMakeLists 加 pluginlib_export_plugin_description_file + install
8. YAML 里配置实例名和 plugin 类型
```

> 你把这八步走通一次，换任何插件类型都只是“换基类、换核心方法”。

---

## 10. 怎么自己查接口

不要背。需要时用这些命令：

```bash
# 看消息/服务/动作定义
ros2 interface show nav2_msgs/action/ComputePathToPose

# 找头文件
ros2 pkg prefix nav2_core
find $(ros2 pkg prefix nav2_core)/include -name '*.hpp'

# 看某个基类的纯虚函数
grep -n "virtual" /opt/ros/jazzy/include/nav2_core/global_planner.hpp

# 看运行中的节点
ros2 node list
ros2 node info /planner_server

# 看参数
ros2 param list /planner_server
ros2 param get /planner_server GridBased.allow_diagonal

# 看生命周期
ros2 lifecycle get /planner_server

# 看动作/话题/服务
ros2 action list -t
ros2 topic list -t
ros2 service list -t

# 看 TF
ros2 run tf2_ros tf2_echo map base_link

# 看代价地图
ros2 topic echo /global_costmap/costmap_raw --once
```

> 本机如果 `ros2 node list` 为空，加 `--no-daemon`。

---

## 11. 练习

1. **读接口**：运行 `ros2 interface show nav2_msgs/action/ComputePathToPose`，
   解释 `use_start` 的作用。如果它为 `false`，起点从哪来？
2. **找基类**：用 `grep` 找出 `nav2_core::GlobalPlanner` 的所有纯虚函数，
   数一数必须实现几个。
3. **参数坑**：如果把 `node->get_parameter(name_ + ".allow_diagonal", ...)`
   改成 `node->get_parameter("allow_diagonal", ...)`，会发生什么？怎么验证？
4. **插件不是节点**：为什么在插件里 `rclcpp::spin()` 是错的？
5. **异常映射**：如果终点在 `254` 上，应该抛哪个异常？行为树收到哪个 `error_code`？
6. **坐标约定**：`createPlan` 收到的 `start` 在哪个坐标系？返回的 `Path.header.frame_id`
   应该是什么？
7. **对比插件**：找到 `nav2_navfn_planner` 的插件 XML，和本项目的 XML 对比，
   哪些字段不同？哪些必须相同？
8. **生命周期**：`planner_server` 从 `inactive` 到 `active` 时，你的插件的哪个方法被调用？
   从 `active` 到 `inactive` 呢？

### 参考答案

1. `use_start=false` 时用机器人当前位姿作为起点；`true` 时用请求里的 `start`。
2. 5 个：`configure`、`cleanup`、`activate`、`deactivate`、`createPlan`。
3. 参数永远读不到，一直用默认值，且不报错。验证：
   `ros2 param get /planner_server GridBased.allow_diagonal` 看实际值，
   或在 `configure` 里打印读到的值。
4. 插件不是独立节点，没有自己的 executor；spin 会阻塞 `planner_server`
   的回调线程，导致整个规划服务器卡死。
5. 抛 `nav2_core::GoalOccupied`；`error_code = GOAL_OCCUPIED (206)`。
6. 起点/终点在全局坐标系（通常是 `map`）；返回的 `Path.header.frame_id` 也必须是它。
7. `library path`、`type`、`description` 不同；`base_class_type` 必须相同，
   都是 `nav2_core::GlobalPlanner`。
8. 激活时调用 `activate()`；去激活时调用 `deactivate()`。

---

## 12. 一页总结

```text
ROS2：节点 + 话题 + 服务 + 动作 + 参数 + TF + launch
Nav2：bt_navigator 编排 → planner_server 规划 → controller_server 控制

你的插件：
  不是节点，是被 planner_server 托管的类
  通过 pluginlib 在运行时 dlopen
  四件套：package.xml / CMake / XML / EXPORT_CLASS
  生命周期：configure → activate → createPlan → deactivate → cleanup
  参数：必须带实例名前缀 GridBased.xxx
  地图：Costmap2DROS.getCostmap()，读之前加锁
  坐标：start/goal 已在 global frame；返回 Path 也用 global frame
  失败：抛 nav2_core 的异常，不要返回空 Path
```

### 下一步

- 想知道怎么从零写出来：看 [06 从纯算法到 Nav2 插件实战](06_从纯算法到Nav2插件实战.md)。
- 想逐行读本项目：看 [07 源码逐行导读](07_源码逐行导读.md)。
- 想补算法：看 [04 A* 算法零基础课](04_A星算法零基础课.md)。
