# Controller 接口与插件实战

> **前置**：读完 [10 控制器算法零基础课](10_控制器算法零基础课.md)。
> 本文回答：一个算法核心，怎么变成 Nav2 能加载、能切换、能上车的 Controller 插件？
>
> **配套代码**：`src/nav2_controller_lab/`
> **逐行代码**：[12 控制器源码逐行导读](12_控制器源码逐行导读.md)

---

## 1. 先看接口：`nav2_core::Controller`

位置：`/opt/ros/jazzy/include/nav2_core/controller.hpp`

```cpp
class Controller
{
public:
  virtual ~Controller() {}

  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  virtual void cleanup() = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;

  virtual void setPlan(const nav_msgs::msg::Path & path) = 0;

  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) = 0;

  virtual bool cancel() {return true;}

  virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) = 0;

  virtual void reset() {}
};
```

必须实现 7 个纯虚函数：

| 方法 | 调用时机 | 你该做什么 |
|---|---|---|
| `configure` | `controller_server` configure | 读参数、存 costmap/TF、创建算法 |
| `activate` | activate | 启动线程（本项目不需要） |
| `deactivate` | deactivate | 停线程 |
| `cleanup` | cleanup | 释放算法/资源 |
| `setPlan` | 收到新全局路径 | 保存路径，清内部状态 |
| `computeVelocityCommands` | 每个控制周期（20 Hz） | 核心：算 v/ω |
| `setSpeedLimit` | 速度限制回调 | 保存并应用限速 |

`cancel()` / `reset()` 有默认实现，可选。

---

## 2. 生命周期与调用顺序

```text
controller_server configure
  └─ createUniqueInstance(plugin_type)
       └─ controller->configure(node, instance_name, tf, costmap_ros)

controller_server activate
  └─ controller->activate()

收到路径
  └─ controller->setPlan(path)

每个控制周期
  └─ controller->computeVelocityCommands(pose, velocity, goal_checker)

controller_server deactivate
  └─ controller->deactivate()

controller_server cleanup
  └─ controller->cleanup()
```

**重点**：

- `configure` 只做一次初始化；参数在这里读；
- `computeVelocityCommands` 每 50 ms 调一次（20 Hz）；
- `setPlan` 可能被频繁调用（重规划）；
- 不要在 `computeVelocityCommands` 里做耗时初始化。

---

## 3. 输入输出契约

### 3.1 输入

| 输入 | 帧 | 说明 |
|---|---|---|
| `pose` | 局部代价地图全局帧（通常是 `odom`） | 机器人当前位姿 |
| `velocity` | 机器人本体 | 当前速度 |
| `goal_checker` | - | 官方目标检查器，可直接用 |
| `global_plan_` | 通常是 `map` | 通过 `setPlan` 保存 |
| `costmap_ros` | 局部代价地图 | `local_costmap` |
| `tf` | - | `map ↔ odom` 变换 |

### 3.2 输出

```cpp
geometry_msgs::msg::TwistStamped
  header.frame_id = base_link
  twist.linear.x  = v
  twist.angular.z = ω
```

- 只控制 `v` 和 `ω`；
- 不要填 `linear.y`（差速车不能用）；
- 时间戳用当前 ROS 时间。

### 3.3 坐标系坑

全局路径在 `map`，机器人位姿和局部代价地图在 `odom`。
控制器必须：

```text
map 路径 --TF--> odom 路径
```

否则路径会偏移、控制器不转、到点不停。

本项目在 `ControllerBase::transformPlanToLocalFrame()` 里逐点变换：

```cpp
tf_->transform(pose_in, pose_out, global_frame_, timeout);
```

并且用**当前时间**，不是路径规划时刻的旧时间戳。

---

## 4. 局部代价地图

Controller 拿到的是 **local costmap**，不是 global costmap。

| | global costmap | local costmap |
|---|---|---|
| 帧 | `map` | `odom` |
| 范围 | 整张地图 | 机器人周围滚动窗口 |
| 用途 | 全局规划 | 局部避障 |
| 更新 | 约 1 Hz | 约 5 Hz |

DWB/MPPI/MPC 需要读：

```cpp
unsigned int mx, my;
if (costmap_->worldToMap(wx, wy, mx, my)) {
  uint8_t cost = costmap_->getCost(mx, my);
}
```

读之前必须加锁：

```cpp
std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
  *(costmap_->getMutex()));
```

本项目由 `ControllerBase` 在 `needsCostmap() == true` 时统一加锁。

---

## 5. 异常约定

控制器抛出的异常会被 `controller_server` 捕获，并转成 `FollowPath` action 的 `error_code`。

| 情况 | 抛什么 |
|---|---|
| TF 失败 | `nav2_core::ControllerTFError` |
| 路径为空 | `nav2_core::InvalidPath` |
| 没有可行控制 | `nav2_core::NoValidControl` |
| 超时 | `nav2_core::ControllerTimedOut` |
| 其他 | `nav2_core::ControllerException` |

> 不要静默返回 0 速度；行为树需要知道“是暂时没控制还是彻底失败”。

---

## 6. 参数：实例名前缀

Nav2 允许同时加载多个 controller：

```yaml
controller_plugins: ["FollowPath", "PID", "Stanley", "DWB", "MPPI", "MPC"]

FollowPath:
  plugin: "nav2_controller_lab::PurePursuitController"
  lookahead_distance: 0.6
PID:
  plugin: "nav2_controller_lab::PidController"
  kp_heading: 0.8
```

`configure` 收到的 `name` 就是实例名：

```cpp
name_ = name;  // "FollowPath"
declareAndGet(node, "lookahead_distance", 0.6, lookahead_distance_);
// 实际参数名：FollowPath.lookahead_distance
```

**写成 `lookahead_distance` 而不是 `name_ + "." + ...`，参数读不到且不报错。**
这是 Nav2 插件最常见的坑。

---

## 7. pluginlib 四件套

### 7.1 `controller_plugins.xml`

```xml
<library path="nav2_controller_lab">
  <class type="nav2_controller_lab::PurePursuitController"
         base_class_type="nav2_core::Controller">
    <description>Pure Pursuit 控制器</description>
  </class>
</library>
```

### 7.2 `package.xml`

```xml
<export>
  <build_type>ament_cmake</build_type>
  <nav2_core plugin="${prefix}/controller_plugins.xml"/>
</export>
```

### 7.3 `CMakeLists.txt`

```cmake
pluginlib_export_plugin_description_file(nav2_core controller_plugins.xml)
install(FILES controller_plugins.xml DESTINATION share/${PROJECT_NAME})
```

### 7.4 `.cpp`

```cpp
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  nav2_controller_lab::PurePursuitController, nav2_core::Controller)
```

### 7.5 四个名字

| 位置 | 值 |
|---|---|
| XML `library path` | `.so` 去掉 `lib` 和 `.so` |
| XML `type` | C++ 类全名 |
| XML `base_class_type` | `nav2_core::Controller` |
| YAML `plugin` | 与 XML `type` 一致 |

---

## 8. 从零写一个 Controller 插件

### 8.1 目录

```text
src/my_controller/
├── include/my_controller/my_controller.hpp
├── src/my_controller.cpp
├── controller_plugins.xml
├── package.xml
└── CMakeLists.txt
```

### 8.2 头文件骨架

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

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string name_;
  std::string global_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("MyController")};
  nav_msgs::msg::Path global_plan_;
  double max_linear_{0.5};
  double max_angular_{1.9};
};
```

### 8.3 `configure`

```cpp
void MyController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::ControllerException("父节点失效");
  }

  parent_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_linear_velocity", rclcpp::ParameterValue(0.5));
  node->get_parameter(name_ + ".max_linear_velocity", max_linear_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_angular_velocity", rclcpp::ParameterValue(1.9));
  node->get_parameter(name_ + ".max_angular_velocity", max_angular_);
}
```

### 8.4 `setPlan`

```cpp
void MyController::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    throw nav2_core::InvalidPath("空路径");
  }
  global_plan_ = path;
}
```

### 8.5 `computeVelocityCommands`

```cpp
geometry_msgs::msg::TwistStamped MyController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  // 1. 路径 map -> odom（逐点 TF）
  // 2. 机器人位姿转 Pose2D
  // 3. 调用算法核心 -> (v, ω)
  // 4. 限速、限角速度
  // 5. 填 TwistStamped
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd.header.stamp = node_->now();
  cmd.twist.linear.x = v;
  cmd.twist.angular.z = w;
  return cmd;
}
```

> 完整可运行实现见 `src/nav2_controller_lab/src/controller_base.cpp`。

### 8.6 四件套

见第 7 节。

---

## 9. 运行时切换控制器

Nav2 的机制：

1. `controller_plugins` 一次性加载多个实例；
2. 行为树的 `ControllerSelector` 订阅 `/controller_selector`；
3. 你发布实例名，BT 把 `selected_controller` 传给 `FollowPath`。

```yaml
controller_plugins: ["FollowPath", "PID", "Stanley", "DWB", "MPPI", "MPC"]
```

```bash
ros2 topic pub --once /controller_selector std_msgs/msg/String \
  "{data: MPPI}" \
  --qos-durability transient_local \
  --qos-reliability reliable
```

> 切换影响下一次 `FollowPath` action；正在执行的旧 action 不会中途换算法。

---

## 10. RViz 面板与调试可视化

本项目的 `nav2_controller_lab_rviz`：

- `Controller Lab` 面板：6 个按钮切换控制器；
- `/controller_lab/markers`：
  - 绿色：全局路径
  - 黄色：前视点
  - 蓝色：当前速度预测轨迹
  - 白色文字：控制器名、v/w

这能把“算法在干什么”直接画出来，比只看 `/cmd_vel` 直观。

---

## 11. 验证七层

| 层 | 检查 | 命令/现象 |
|---|---|---|
| L0 | 编译 | `colcon build` |
| L1 | 插件索引 | `cat .../nav2_core__pluginlib__plugin/...` |
| L2 | 符号 | `nm -D ... | grep PurePursuit` |
| L3 | 插件加载 | 日志 `Created controller : FollowPath ...` |
| L4 | 参数 | `ros2 param get /controller_server FollowPath.plugin` |
| L5 | 生命周期 | `ros2 lifecycle get /controller_server` |
| L6 | 控制输出 | `ros2 topic echo /cmd_vel` |
| L7 | 导航 | `Goal succeeded` |

---

## 12. 常见坑

| 现象 | 原因 | 处理 |
|---|---|---|
| `Unable to find class` | ament 索引没有 | 检查 package.xml/CMake，重新编译 |
| `Failed to load library` | XML/库名/宏不匹配 | 检查 `library path`、`EXPORT_CLASS` |
| 参数全是默认值 | 少 `name_.` 前缀 | 看 `ros2 param list` |
| 控制器不转 | `map→odom` TF 缺失 | `tf2_echo map base_link` |
| 到点不停 | goal_checker/容差 | 看 `goal_tolerance`、goal checker |
| 原地转圈 | 路径/位姿坐标系错 | 检查 TF、frame_id |
| DWB 不动 | 所有轨迹碰撞 | 调 `collision_threshold`、降速 |
| MPPI 很慢 | `batch_size` 大 | 降采样 |
| MPC 抖动 | 步长/迭代过大 | 降 `gradient_step` / `iterations` |
| 改参数没生效 | launch 读 install 副本 | 重新 `colcon build` 或用 `--symlink-install` |
| 切换控制器没反应 | QoS 不对 | 加 `--qos-durability transient_local` |

---

## 13. 检查清单

- [ ] 实现全部纯虚函数；
- [ ] 参数带实例名前缀；
- [ ] `configure` 里保存 `costmap_ros`、TF、logger；
- [ ] `setPlan` 校验空路径；
- [ ] `computeVelocityCommands` 处理 `map→odom`；
- [ ] 读局部 costmap 时加锁；
- [ ] 输出只填 `linear.x` / `angular.z`；
- [ ] 速度/角速度有上限；
- [ ] 异常类型正确；
- [ ] 四件套齐全；
- [ ] 有单元测试；
- [ ] 有 RViz/日志可观察。

---

## 14. 一页总结

```text
Controller 插件 = 算法核心 + ROS 适配层 + pluginlib 注册

接口：configure / activate / deactivate / cleanup
      setPlan / computeVelocityCommands / setSpeedLimit

输入：odom 位姿、map 路径、局部 costmap、TF
输出：TwistStamped(v, ω)

关键工程点：
  参数实例名前缀
  map→odom TF
  costmap 锁
  异常类型
  四件套
  运行时 /controller_selector
```

### 下一步

- 逐行读代码：[12 控制器源码逐行导读](12_控制器源码逐行导读.md)。
- 做实验：[09 控制器实验手册](09_控制器实验手册.md)。
- 排错调参：[13 控制器实验、调参与排错](13_控制器实验调参与排错.md)。
