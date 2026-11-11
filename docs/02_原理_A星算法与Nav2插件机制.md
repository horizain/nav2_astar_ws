# A* 算法与 Nav2 插件机制 —— 原理教学

> **读者导航**：本篇是“原理速览”，适合已经能跑通、想快速回顾的读者。
> 完全没接触过 A*，先读 [04 算法零基础课](04_A星算法零基础课.md)；
> 不知道 ROS2/Nav2 接口，先读 [05 接口入门](05_ROS2与Nav2接口入门.md)；
> 想从零写插件，看 [06 实战手册](06_从纯算法到Nav2插件实战.md)。
> 全部文档索引见 [README](README.md)。

本文档讲清两件事：**A\* 算法为什么能找到最短路**，以及 **Nav2 如何把这样一个算法插进它的导航流程**。

---

## 第一部分：A* 算法原理

### 1.1 先看问题：栅格地图上的寻路

代价地图（costmap）本质上是一张二维格子表，每格有一个 `uint8_t` 代价：

| 值 | 含义 | 能否通行 |
|---|---|---|
| `0` | `FREE_SPACE` 空闲 | ✓ |
| `1~252` | 膨胀代价，越大越靠近障碍 | ✓（代价高） |
| `253` | `INSCRIBED_INFLATED_OBSTACLE` 内切膨胀 | 策略相关（已进入碰撞风险区） |
| `254` | `LETHAL_OBSTACLE` 真实障碍 | ✗ |
| `255` | `NO_INFORMATION` 未知 | 由 `allow_unknown` 决定 |

寻路目标：从起点格子走到终点格子，路径总代价最小。

### 1.2 三个关键量

A* 围绕三个数展开：

- **g(n)**：从**起点**到节点 n 的**实际**代价（已知）
- **h(n)**：从节点 n 到**终点**的**估计**代价（启发式，猜的）
- **f(n) = g(n) + h(n)**：经过 n 的总代价估计

A* 每次从待考察集合（open_list）里取 **f 最小**的节点来扩展。

```
       起点 S                终点 G
         ●─────────────────────●
          \                   /
           \   g(n): 已走过的路程
            ● n ──────────────
                 h(n): 到终点的直线距离估计
           
           f(n) = g(n) + h(n)   ← 用这个排序
```

### 1.3 为什么 A* 能保证最短路？

关键在 **h 的可采纳性（admissible）**：

> 若 h(n) 永不高估真实剩余代价（h(n) ≤ 真实距离），则 A* 第一次弹出终点时，
> 它一定是最优解。

直觉：因为 h 不高估，f 就不高估。如果某条路能更短，它的 f 会更小，
A* 会先探索它，而不是抢先收工。

### 1.4 h 怎么选：曼哈顿 vs 八方向（Octile）

本实现支持 4 邻域和 8 邻域两种，h 必须与允许的移动方式**匹配**：

**4 邻域（只能上下左右）→ 曼哈顿距离**
```
h = |dx| + |dy|
```

**8 邻域（可对角）→ Octile 距离**
```
h = (dx + dy) + (√2 − 2) · min(dx, dy)
```
> 为什么不是欧氏距离？在 8 邻域里，`√(dx²+dy²)` 会**低估**太多，
> 导致 A* 退化成 Dijkstra，扩展大量无用节点。
> 而欧氏距离在纯 4 邻域下又**高估**（可采纳性被破坏）。
> Octile 是 8 邻域下的**精确**最小步数，恰好可采纳且最紧。

代码位置：`astar_algorithm.cpp` 的 `heuristic()`

```cpp
if (options_.allow_diagonal) {
  h = (dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy);  // Octile
} else {
  h = dx + dy;                                                // 曼哈顿
}
return h * options_.heuristic_weight;
```

### 1.5 `heuristic_weight`：最优性 vs 速度的旋钮

```
h' = h × w
```

- `w = 1.0`：标准 A*，**保证最短路**
- `w > 1.0`：贪婪 A*（Weighted A*），高估 h —— 更快，但**不保证最优**，
  路径可能明显更"贴墙"

这是实践中很常用的调优手段：`w` 取到 1.1~1.5 通常能显著加速，
路径长度只差几个百分点。

### 1.6 算法主循环（本实现的骨架）

```cpp
g_score[start] = 0;
open.push({f = h(start, goal), g = 0, index = start});

while (!open.empty()) {
    current = open.pop_min_f();        // 取 f 最小
    if (closed[current]) continue;     // 旧副本，跳过
    closed[current] = 1;

    if (current == goal) break;        // 到达，回溯路径

    for (neighbor in 8邻域) {
        if (closed[neighbor] || !可通行(neighbor)) continue;

        tentative_g = g_score[current] + 移动代价;

        if (tentative_g < g_score[neighbor]) {   // 找到更优路径
            g_score[neighbor] = tentative_g;
            parent[neighbor] = current;          // 记下父亲，用于回溯
            open.push({tentative_g + h(neighbor, goal), tentative_g, neighbor});
        }
    }
}
```

### 1.7 三个实现细节（都踩过坑）

#### ① `closed` 判重为什么必要

`priority_queue` 不支持"修改已有元素的优先级"，所以同一节点可能被**多次 push**。
弹出时必须检查是否已扩展过：

```cpp
if (closed[current.index]) continue;
```

#### ② 对角移动的"切角"检查

如果只检查目标格是否能走，A* 会从两个障碍的**缝隙**中斜穿过去：

```
  ███ 表示障碍
  ● →↗ 目标格
  ███
  这样斜穿会让机器人足迹撞到障碍角
```

所以对角移动时，必须**同时**检查两个相邻直移格：

```cpp
if (diagonal) {
    if (!可通行(cx + dx, cy) || !可通行(cx, cy + dy)) continue;
}
```

#### ③ 起点代价的坑（本项目实际踩过的 bug）

最初我把可通行阈值设成 `< 253`（`INSCRIBED_INFLATED_OBSTACLE`），
结果报"无可行路径"。

调试发现：**机器人贴墙停车时，起点格的代价正是 253**！

于是 `253 < 253 == false`，起点周围的格子全被判为不可通行，A* 一个邻居都扩展不出去，
搜索很快结束，直接返回 NO_PATH。

**本项目的取舍**：只有 `254`（真实障碍）才不可通行，`253` 视为高代价可通行。
这样贴墙停车时起点不会直接判死。

> 注意：这不是说 `253` 绝对安全。`253` 表示机器人中心已进入“内切膨胀”区，
> 严格按 footprint 检查可能已经碰撞。官方 Navfn 在 `createPlan()` 层只对终点
> 检查 `254`，但它内部的代价转换会把 `253` 当障碍处理；不同版本/实现策略不同。
> 本项目为了让教学链路跑通，选择更宽松的策略。生产环境应结合 footprint
> 碰撞检查或增大 `inflation_radius`。

```cpp
// src/astar_planner.cpp
astar_->setTraversabilityChecker(
  [allow_unknown](unsigned int, uint8_t cost) -> bool {
    if (cost == nav2_costmap_2d::NO_INFORMATION) return allow_unknown;
    return cost < nav2_costmap_2d::LETHAL_OBSTACLE;   // < 254
  });
```

> **这是本项目最有价值的一个教训**：一定要在真实代价地图上打印起点附近
> 的代价分布，而不是靠猜。当时用 Python 复现同样逻辑、打印 11×11 邻域，
> 一眼就看出机器人被 254 围住（或起点本身是 253）。

### 1.8 复杂度

| 项 | 复杂度 |
|---|---|
| 时间 | O(E log V)，E≈8V，实际取决于地图 |
| 空间 | O(V)，V = 宽×高 |
| 最坏情况 | 整张地图扩展一遍（h 很松时退化为 Dijkstra） |

本实现用 `std::priority_queue` + 一维索引数组，384×384 地图通常几毫秒完成。

---

## 第二部分：Nav2 插件机制

### 2.1 整体数据流

```
用户点 Nav2 Goal
      │
      ▼
bt_navigator (行为树)
      │  调用 ComputePathToPose action
      ▼
planner_server (生命周期节点)
      │  查 planner_plugins 参数，从列表里取实例名
      │  pluginlib 按名字创建插件对象
      ▼
┌──────────────────────────────────┐
│  nav2_core::GlobalPlanner (接口)  │  ← 抽象基类，只有虚函数
└──────────────┬───────────────────┘
               │ 实现
               ▼
    nav2_astar_planner::AStarPlanner  ← 我们的插件
               │  读取 global_costmap
               ▼
          AStarAlgorithm (纯算法)
               │  返回格子路径
               ▼
    interpolatePath() → nav_msgs::Path
               │
               ▼
    controller_server 跟踪路径 → 输出 cmd_vel → Gazebo
```

### 2.2 `GlobalPlanner` 接口：必须实现的 5 个方法

位置：`nav2_core/include/nav2_core/global_planner.hpp`

```cpp
class GlobalPlanner {
public:
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

| 方法 | 什么时候调用 | 该做什么 |
|---|---|---|
| `configure` | 节点 `on_configure` | 读参数、存 `costmap_ros`、初始化算法 |
| `activate` | 节点 `on_activate` | 启动线程/定时器（本项目无需） |
| `deactivate` | 节点 `on_deactivate` | 停线程 |
| `cleanup` | 节点 `on_cleanup` | 释放算法对象 |
| `createPlan` | **每次规划请求** | 核心：算路径 |

> **为什么是"生命周期节点"**：`planner_server` 不是普通的 Node，而是
> `LifecycleNode`。它有 `unconfigured → inactive → active → inactive → ...`
> 的状态机。只有 `active` 状态才会响应规划请求。
> 这样设计是为了让导航栈能安全地"热身"（先 configure 加载插件、
> 等 TF 和 costmap 就绪，再 activate 开始服务）。

### 2.3 pluginlib：插件是怎么被"发现"的

这是初学者最容易卡住的地方。pluginlib 的完整链路：

```
① 编译期
   pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)
   ↓ 在 install 下生成 ament 索引：
   share/ament_index/resource_index/nav2_core__pluginlib__plugin/nav2_astar_planner
   内容：share/nav2_astar_planner/global_planner_plugin.xml

② 运行期（pluginlib 扫描）
   ClassLoader<GlobalPlanner> loader("nav2_core", "nav2_core::GlobalPlanner");
   ↓ 遍历 ament 索引，找到所有 base_class_type 匹配的 xml
   ↓ 读到 <class type="nav2_astar_planner::AStarPlanner">

③ 创建实例
   loader.createInstance("nav2_astar_planner::AStarPlanner")
   ↓ dlopen("libnav2_astar_planner.so")
   ↓ 查找 PLUGINLIB_EXPORT_CLASS 注册的工厂符号
   ↓ 构造对象，返回 GlobalPlanner 指针
```

**四个断点，任一缺失都会失败**：

| 断点 | 失败现象 |
|---|---|
| `CMakeLists` 缺 `pluginlib_export_plugin_description_file` | ament 索引里没有记录，报 `Unable to find class` |
| `package.xml` 缺 `<nav2_core plugin=...>` | 同上 |
| `.cpp` 缺 `PLUGINLIB_EXPORT_CLASS` | 能 dlopen 但找不到工厂，报 `Failed to load library` |
| `library path` 与实际 `.so` 名不符 | 同上 |

### 2.4 参数命名规则（第二个大坑）

Nav2 支持**同时加载多个规划器**（`planner_plugins` 是数组）。
为了区分，每个插件的参数都必须带上**实例名**前缀：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased", "AnotherPlanner"]   # 两个实例
    GridBased:
      plugin: "nav2_astar_planner::AStarPlanner"
      allow_diagonal: true        # → 实际参数名 GridBased.allow_diagonal
    AnotherPlanner:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5              # → AnotherPlanner.tolerance
```

插件里通过 `name` 参数拿到自己的前缀：

```cpp
void configure(..., std::string name, ...) {
  name_ = name;                              // name == "GridBased"
  nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".allow_diagonal", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".allow_diagonal", allow_diagonal_);
}
```

> 写成 `"allow_diagonal"` 而不是 `name_ + ".allow_diagonal"`，
> 参数会读不到（一直用默认值），且**不报错** —— 极难察觉。

### 2.5 代价地图的访问：必须加锁

`costmap_` 会被 costmap 的更新线程并发修改，`createPlan` 里必须持锁：

```cpp
std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
```

否则会出现读到"半更新"的地图，规划结果飘忽。

### 2.6 为什么需要路径插值

A* 输出的是**格子序列**，相邻格子间距 = 分辨率（本机 0.05 m）。
直接把它当 `Path` 发出去，在转弯处会有突兀的折角，控制器跟踪质量差。

`interpolatePath()` 在相邻格子之间线性插值，保证点间距 ≤
`interpolation_resolution`，并按路径切线方向设置每个点的朝向
（yaw），让控制器知道该往哪转。

### 2.7 坐标系与异常约定

`createPlan` 收到的 `start`/`goal` 已在 `global_frame`（通常是 `map`）下。
必须用 `worldToMap` 转成格子坐标，越界要抛**正确的异常类型**，
BT 才能识别并重试：

| 情况 | 抛出的异常 |
|---|---|
| 起点越界 | `nav2_core::StartOutsideMapBounds` |
| 终点越界 | `nav2_core::GoalOutsideMapBounds` |
| 终点在障碍上 | `nav2_core::GoalOccupied` |
| 无可行路径 | `nav2_core::NoValidPathCouldBeFound` |
| 被取消 | `nav2_core::PlannerCancelled` |

抛错而不是返回空 Path，BT 才会正确触发恢复行为（清 costmap、重试等）。

---

## 第三部分：本实现的参数速查

| 参数 | 默认 | 作用 | 调优建议 |
|---|---|---|---|
| `allow_diagonal` | `true` | 8 邻域 / 4 邻域 | 保持 true，路径更自然 |
| `allow_unknown` | `true` | 是否穿越未知区 | 探索任务开；已知地图可关 |
| `cost_weight` | `0.0` | 地形代价权重 | 0=最短；调到 2~5 会明显远离障碍 |
| `heuristic_weight` | `1.0` | 启发式倍数 | 大图可试 1.2~1.5 提速 |
| `interpolation_resolution` | `0.05` | 插值间距(米) | 与地图分辨率一致即可 |
| `max_expansions` | `1000000` | 扩展上限 | 防病态地图卡死 |
| `use_final_approach_orientation` | `false` | 末端朝向取路径切线 | 想避免终点原地转向时设 true |

---

## 第四部分：与官方规划器的对比

| 规划器 | 算法 | 特点 |
|---|---|---|
| `NavfnPlanner` | Dijkstra / A*（可选） | nav2 默认，代价场传播，稳定 |
| `SmacPlanner2D` | A*（8 邻域） | 平滑、支持多分辨率 |
| `SmacPlannerHybrid` | Hybrid-A* | 考虑车辆运动学（非完整约束） |
| `ThetaStarPlanner` | Theta* | 任意角度，路径更直 |
| **本项目 AStarPlanner** | **A*（8 邻域）** | **教学实现，结构最简，便于改造** |

> 本实现相比 `SmacPlanner2D` 缺的是：路径平滑（smoother 后处理在
> `smoother_server` 里，本栈已启用）、多分辨率搜索、更精细的代价模型。
> 作为教学起点，它把 A* 的骨架完整暴露出来了 —— 这正是学习目的。

---

## 附：推荐阅读顺序

1. 先读 `astar_algorithm.cpp` 的 `plan()` —— 50 行核心循环，理解 A* 骨架
2. 再读 `astar_planner.cpp` 的 `createPlan()` —— 看 ROS 数据怎么进算法
3. 然后读 `configure()` —— 看参数怎么读
4. 最后看 `pluginlib` 四件套 —— 理解插件如何被装载
5. 动手改：把 `heuristic_weight` 设为 3.0，观察扩展节点数的变化
