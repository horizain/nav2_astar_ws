# 教学文档总目录

> **这套文档写给谁？**
> 会一点 C++、能用 Linux 终端，但**没有算法基础**，也**没写过 ROS 2 / Nav2 插件**，想往机器人规划方向发展的读者。
>
> **最终目标不是“把这个项目跑起来”**，而是让你掌握一条可复用的路径：
>
> **一个纯算法 → 一个能翻译 ROS 数据的适配层 → 一个能被 Nav2 加载的工程插件。**

---

## 0. 先选一条学习路线

不要从第一篇开始顺着读到尾。按你的目标选路线：

| 路线 | 适合谁 | 阅读顺序 | 预计时间 |
|---|---|---|---|
| **A. 只想先懂算法** | 完全没接触过 A*，看到 open/closed 就头大 | 本篇 → [04 算法零基础课](04_A星算法零基础课.md) → 玩 `tools/astar_playground.py` | 6~10 小时 |
| **B. 只想懂 ROS2/Nav2 接口** | 算法大概懂，但不知道插件怎么被加载 | 本篇 → [05 ROS2 与 Nav2 接口入门](05_ROS2与Nav2接口入门.md) | 4~6 小时 |
| **C. 想完整走通“算法→插件”**（推荐） | 想真正具备工程能力 | 04 → 05 → [06 从纯算法到插件实战](06_从纯算法到Nav2插件实战.md) → [07 源码逐行导读](07_源码逐行导读.md) | 15~25 小时 |
| **D. 只想复现本项目** | 环境已就绪，只想把它跑起来 | [01 从零复现教程](01_教程_从零复现A星规划器.md) → [03 开发日志](03_教学进度记录.md) | 2~4 小时 |
| **E. 想学控制器 / MPPI / MPC**（推荐） | 已经懂 A* 和插件机制 | [08 路线图](08_控制器算法学习路线.md) → [10 算法课](10_控制器算法零基础课.md) → [11 接口实战](11_Controller接口与插件实战.md) → [12 源码导读](12_控制器源码逐行导读.md) → [09 实验](09_控制器实验手册.md) / [13 调参排错](13_控制器实验调参与排错.md) | 15~25 小时 |

> 路线 D 是“操作手册”。如果你不理解背后的东西，跑通后很快会忘。
> 建议至少把 04 和 05 的“核心心智模型”部分读完。

---

## 1. 文档地图

| 文档 | 它回答的问题 | 读完你应该能做什么 |
|---|---|---|
| [01 从零复现教程](01_教程_从零复现A星规划器.md) | 这个项目具体怎么编译、启动、验证？ | 在本机跑通 Gazebo + Nav2 + A* |
| [02 原理：A* 与插件机制](02_原理_A星算法与Nav2插件机制.md) | A* 为什么最优？插件链路是什么？ | 说清 `createPlan` 的数据流 |
| [03 教学进度记录](03_教学进度记录.md) | 开发过程中踩过哪些坑？ | 遇到类似报错时快速定位 |
| **[04 A* 算法零基础课](04_A星算法零基础课.md)** | 没有算法基础，怎么从 BFS 一步步理解 A*？ | 手算 A*、解释 h/g/f、判断启发式是否可采纳 |
| **[05 ROS2 与 Nav2 接口入门](05_ROS2与Nav2接口入门.md)** | ROS2 的节点/话题/服务/动作/参数/TF 是什么？Nav2 插件契约长什么样？ | 读懂 `nav2_core::GlobalPlanner`，知道插件如何被 pluginlib 加载 |
| **[06 从纯算法到 Nav2 插件实战](06_从纯算法到Nav2插件实战.md)** | 怎么把“一段算路径的代码”变成工程可用的插件？ | 从零建包、写适配层、注册插件、编译、分层验证 |
| **[07 源码逐行导读](07_源码逐行导读.md)** | 本项目每个文件、每个函数到底在干什么？ | 独立修改算法或接口，不破坏契约 |
| **[08 控制器算法学习路线](08_控制器算法学习路线.md)** | Nav2 控制器最简单的算法是什么？PID 为什么不够？怎么从 Pure Pursuit 学到 MPPI / MPC？ | 设计 `nav2_controller_lab`，按阶梯实现控制器 |
| **[09 控制器实验手册](09_控制器实验手册.md)** | 六种控制器怎么跑、怎么切换、怎么对比？ | 用同一张地图做 A/B 实验，记录指标 |
| **[10 控制器算法零基础课](10_控制器算法零基础课.md)** | PID / Pure Pursuit / Stanley / DWB / MPPI / MPC 的数学和直觉 | 手算、解释参数、判断失效模式 |
| **[11 Controller 接口与插件实战](11_Controller接口与插件实战.md)** | Controller 接口、TF、costmap、pluginlib、运行时切换 | 从零写一个可加载的 Controller 插件 |
| **[12 控制器源码逐行导读](12_控制器源码逐行导读.md)** | `nav2_controller_lab` 每个函数在干什么？ | 独立修改算法/适配层，不破坏契约 |
| **[13 控制器实验、调参与排错](13_控制器实验调参与排错.md)** | 怎么做对比实验、怎么调参、怎么排错？ | 用数据选择控制器，定位常见故障 |
| **[14 控制器开发进度记录](14_控制器开发进度记录.md)** | 开发过程中踩过哪些坑？ | 避免重复犯错 |
| **[算法练习工具](tools/astar_playground.py)** | 不装 ROS 也能练 A* 吗？ | 用 ASCII 地图观察搜索过程、对比算法 |

---

## 2. 前置知识：到底需要会多少？

### 2.1 必须会（不会就先补）

| 类别 | 具体内容 | 在本项目中的用途 |
|---|---|---|
| C++ 基础 | 变量、`if/for`、函数、`struct`/`class` | 看懂算法 |
| C++ 面向对象 | 继承、虚函数、`override`、抽象基类 | 实现 `GlobalPlanner` 接口 |
| C++ 常用容器 | `std::vector`、`std::priority_queue` | 存代价地图、open list |
| C++ 现代特性 | 引用 `&`、`const`、`std::function`、lambda、`std::unique_ptr` | 回调、资源管理 |
| Linux | `cd/ls/cat/grep/find`、环境变量、编译命令 | 编译和排错 |
| CMake | `add_library`、`target_link_libraries`、`install` | 构建插件 |

> **不需要**先学完整个 ROS2 教程。05 会把写插件真正用到的部分讲透。
> **不需要**高深数学。A* 只用到加减法、绝对值、平方根。

### 2.2 C++ 语法急救表

读源码时遇到这些别慌：

| 写法 | 怎么读 | 例子 |
|---|---|---|
| `std::vector<uint8_t>` | 一串字节，像数组 | 一维代价地图 |
| `std::priority_queue<T, ..., std::greater<T>>` | 最小堆，每次弹出“最小”的元素 | open list 按 f 排序 |
| `std::function<bool()>` | 可以存一个函数/ lambda 的变量 | `cancel_checker` |
| `[&](int x) { ... }` | lambda，现场写的小函数 | 可通行判定 |
| `std::unique_ptr<T>` | 独占所有权智能指针，不能复制 | 持有算法对象 |
| `std::shared_ptr<T>` | 共享所有权智能指针 | 持有 costmap |
| `override` | “我确实在重写基类虚函数” | `createPlan(...) override` |
| `const Foo &` | 只读引用，避免拷贝 | 传入起点/终点 |
| `throw SomeException("...")` | 抛异常，交给上层处理 | 终点在障碍上 |
| `namespace a::b` | 防止名字冲突的命名空间 | `nav2_astar_planner` |

### 2.3 环境自检

本项目所在机器已经配置过 `.bashrc`。新开终端先确认：

```bash
echo $ROS_DISTRO                 # 期望：jazzy
ros2 pkg prefix nav2_core        # 期望输出一个路径，而不是报错
ros2 pkg prefix nav2_astar_planner
```

如果 `ros2` 命令不存在，按 [01 教程第 0 节](01_教程_从零复现A星规划器.md) 的三级 source 顺序配置：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_jazzy/install/setup.bash
source ~/ros2_jazzy/nav2_astar_ws/install/setup.bash
```

> 顺序不能反：`/opt` → 外层 Nav2 源码工作空间 → 本项目工作空间。

---

## 3. 先建立 5 个心智模型

在读细节前，先记住这 5 句话。后面所有内容都是它们的展开。

1. **地图是格子，路径是格子序列。**
   工程里的“世界坐标”只是格子的另一套编号，两者用分辨率和原点互相换算。

2. **A* = Dijkstra + 一个“往终点方向”的估计。**
   Dijkstra 按 `g`（已走代价）排序；A* 按 `f = g + h`（已走 + 估计剩余）排序。

3. **Nav2 插件不是“一个节点”，而是“被服务器托管的类”。**
   `planner_server` 是节点；你的 `AStarPlanner` 是它 `new` 出来的对象，生命周期由它调用。

4. **算法层不碰 ROS，适配层不碰算法细节。**
   算法只接收 `vector<uint8_t>` + 起终点格子；适配层负责坐标、锁、参数、异常、消息。

5. **插件能跑通，靠的是“四件套”同时正确。**
   `package.xml` 导出 + CMake 导出 XML + 插件 XML 描述 + `.cpp` 里的 `PLUGINLIB_EXPORT_CLASS`，缺一不可。

---

## 4. 推荐的学习方法

### 4.1 不要“只读不跑”

每篇文档都配了可执行命令或练习。最低要求：

- 04：用 `astar_playground.py` 跑 3 组参数，手算一次 5×5 小地图。
- 05：用 `ros2 interface show` / `ros2 param list` 真的看一眼接口。
- 06：从零建一个**自己的**包（不要直接复制本项目），哪怕只输出一条日志。
- 07：改一行代码，预测结果，再编译验证。

### 4.2 运行算法练习工具（不需要 ROS）

```bash
cd ~/ros2_jazzy/nav2_astar_ws

# 默认 5x5 小地图，4 邻域，打印每次扩展
python3 docs/tools/astar_playground.py --no-diagonal --trace

# 对比 A* / 加权 A* / Dijkstra / BFS
python3 docs/tools/astar_playground.py --demo-map --compare

# 观察启发式权重 w 的影响
python3 docs/tools/astar_playground.py --demo-map --weight 2.0

# 观察地形代价：9 表示很贵的格子
python3 docs/tools/astar_playground.py --map docs/tools/maps/terrain_demo.txt \
  --terrain-weight 3.0
```

地图格式与参数说明见 [04 文档第 11 节](04_A星算法零基础课.md) 和
`docs/tools/astar_playground.py` 文件头。

### 4.3 跑项目自带的算法单测

项目里有 10 个纯算法单元测试，不需要启动 ROS 节点，也不需要 Gazebo：

```bash
cd ~/ros2_jazzy/nav2_astar_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_jazzy/install/setup.bash
colcon build --packages-select nav2_astar_planner
colcon test --packages-select nav2_astar_planner
colcon test-result --verbose
```

这套测试覆盖：空旷地图、整列墙、起点 253、终点 254、起点=终点、未知区开关、
对角切角、取消、扩展上限。看懂它，你就知道“算法层”应该怎么验证。

### 4.4 遇到报错时的顺序

1. **先确认数据**：起点/终点坐标、代价、坐标系。
2. **再确认接口**：插件是否被加载、参数是否读到、生命周期是否 active。
3. **最后怀疑算法**：用固定小地图做单元测试，脱离 ROS 复现。
4. **查日志要带上下文**：不要只搜最后一行 `ERROR`，往前找第一个异常。

> 这条顺序来自本项目最贵的一次教训：报“无可行路径”时，真实原因是
> **AMCL 初始位姿设错**，机器人被“传送”到墙里，算法本身完全正确。
> 详见 [03 阶段 6.4](03_教学进度记录.md)。

---

## 5. 30 分钟速览（如果你现在没时间）

```text
现实世界
  └─ 传感器/地图 → 栅格代价地图（每格 0~255）
       └─ 你的算法：输入 起点格子 + 终点格子 + 代价数组
            └─ 输出 格子路径 [ (x0,y0), (x1,y1), ... ]
                 └─ 适配层：格子 → 世界坐标，线性插值，设置朝向
                      └─ nav_msgs/Path
                           └─ planner_server 通过 action 返回给 BT
                                └─ controller_server 跟踪路径 → cmd_vel
```

你要写的工程代码，本质上就是中间那三层：

```text
纯算法（可单测）  →  ROS 适配层（坐标/锁/参数/异常）  →  插件注册（pluginlib 四件套）
```

---

## 6. 术语表（先混个脸熟）

| 术语 | 一句话解释 |
|---|---|
| **costmap** | 代价地图。每个格子有一个 0~255 的值，越大越难走。 |
| **global planner** | 全局规划器。从当前位置到目标，算一条全局路径。 |
| **local planner / controller** | 局部控制器。沿着全局路径走，输出速度指令。 |
| **BT / behavior tree** | 行为树。Nav2 用它编排“规划→跟踪→恢复”的流程。 |
| **lifecycle node** | 生命周期节点。有 configure/activate/deactivate/cleanup 状态机。 |
| **pluginlib** | ROS2 的插件加载库。按 XML 描述在运行时 `dlopen` 动态库。 |
| **ament index** | ROS2 的资源索引。pluginlib 靠它找到插件 XML。 |
| **TF** | 坐标变换。`map → odom → base_link` 描述机器人在地图里的位置。 |
| **action** | 带反馈、可取消的“长任务”通信方式。规划就是一个 action。 |
| **QoS** | 通信质量策略。可靠性、历史深度、是否保留最后一条等。 |
| **admissible** | 可采纳。启发式 `h` 从不高估真实剩余代价。 |
| **octile** | 8 邻域下的精确距离，`(dx+dy)+(√2−2)·min(dx,dy)`。 |

---

## 7. 文档维护约定

- 代码行号会随修改变化；**以函数名和代码片段为准**，行号只作定位参考。
- 命令默认在 `~/ros2_jazzy/nav2_astar_ws` 下执行。
- 标注 **【必做】** 的是理解所必需；**【选做】** 的是进阶。
- 发现文档与代码不一致时，以代码为准，并回头修正文档。

---

## 8. 下一步

- 完全零基础：从 [04 A* 算法零基础课](04_A星算法零基础课.md) 开始。
- 已经懂 A*：直接看 [05 ROS2 与 Nav2 接口入门](05_ROS2与Nav2接口入门.md)。
- 想动手写插件：读 [06 实战手册](06_从纯算法到Nav2插件实战.md)。
- 想彻底吃透本仓库：最后读 [07 源码逐行导读](07_源码逐行导读.md)。
- 想学控制器：从 [08 路线图](08_控制器算法学习路线.md) 开始，
  然后读 [10 算法课](10_控制器算法零基础课.md) 和
  [11 接口实战](11_Controller接口与插件实战.md)。
- 想读控制器源码：[12 源码逐行导读](12_控制器源码逐行导读.md)。
- 想调参排错：[13 实验、调参与排错](13_控制器实验调参与排错.md)。
