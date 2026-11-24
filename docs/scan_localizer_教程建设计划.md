# `scan_localizer` 交互式工程教程建设计划

> **状态**：**已完成（M1 → M4 全链路落地）**。教程入口见 [30 scan_localizer 交互式工程教程导览](30_scan_localizer教程导览.md) 与 [教程首页](scan_localizer_tutorial/index.html)。
>
> **基线提交**：`98fcd28`（`src/scan_localizer` 共 33 个文件 / ~11,900 行代码；`colcon test` 192 项测试 0 失败；Intel Research Lab 450 组真值三方评测 GT Trans RPE RMSE **3.06 cm**）。

---

## 一、总体形态与架构设计

采用**「交互式静态教程站 + Markdown 课程入口」双形态**（零前端构建工具、纯静态可离线打开，复用仓库已有的 KaTeX 与 Highlight.js `.min` 资产）：

- **主站目录（规划）**：`docs/scan_localizer_tutorial/`
  - 资产复用策略：`vendor/` 从 [docs/website](website/index.html) 的 `vendor/katex/`（与 `slam-mpc-academy/assets/vendor/katex/` 一致）及 `slam-mpc-academy/assets/vendor/hljs/`（`atom-one-dark.min.css`、`highlight.min.js`）引入最小集合，只保留 `.min` 版并仅引用 `contrib/auto-render.min.js`。
- **Markdown 课程入口（规划）**：`docs/30_scan_localizer教程导览.md`，后续接入 [教学文档总目录](README.md)，并通过 [链接检查脚本](tools/check_tutorial_links.py) 检验。
- **一键本地启动（规划）**：`docs/scan_localizer_tutorial/serve.sh`
  - 使用轻量 Python HTTP 服务启动教程站，同时以只读方式挂载 `src/scan_localizer/`，支持**网页内点击跳转徽章滑出实时源码抽屉**。
- **每章统一 6 段式教学结构**：
  1. **直觉与动画先行**：先拖动滑块 / 观察 Canvas 动画，建立几何与动态直觉；
  2. **公式推导**：从零基础微积分/线性代数逐步推到工程矩阵形式（KaTeX 渲染）；
  3. **核心源码逐行剖析**：展示真实 C++20 / ROS 2 代码，附精确 `文件:行号` 跳转徽章；
  4. **语法解码（Syntax Spotlight）**：现场拆解本章出现的 C++20、Eigen 与 ROS 2 写法；
  5. **工程技巧与踩坑复盘**：结合 Phase 0–5 重构与 Intel Lab 实测踩过的真实缺陷复盘；
  6. **思考题与自检**：带折叠答案的交互自检卡，避免“看懂 ≠ 学会”。

---

## 二、课程路线规划（10 章 + 1 个全景代码索引页）

从入门概念逐层递进到 `scan_localizer` 生产级多阶段融合定位架构：

| 阶段 | 章节 | 核心主题、入门铺垫与数学推导 | 对应仓库源码 | 专属交互动画 / 可视化 |
|---|---|---|---|---|
| **第一阶段：入门与底座** | **Ch01 定位是什么 & $SE(2)$ 与 TF 树** | 已知地图定位 vs SLAM；激光极坐标 $(r_i,\phi_i) \to (x_i,y_i)$；$SE(2)$ 群复合 $T_1 T_2$ 与逆 $T^{-1}$；为何不能直接发 `map→base_link` 而必须发布 `map→odom = T_{m\leftarrow b} T_{o\leftarrow b}^{-1}`；PGM 图像原点在左上角与 ROS 左下角坐标的 Y 轴翻转、格心 `0.5` 偏移 | [se2.hpp](../src/scan_localizer/include/scan_localizer/se2.hpp), [map_io.hpp](../src/scan_localizer/include/scan_localizer/map_io.hpp), [map_io.cpp](../src/scan_localizer/src/map_io.cpp) | **动画 1**：2D 机器人 + 雷达扫描 + TF 链实时演示（可拖动轮式漂移、切换倒装雷达 `roll = \pi`） |
| | **Ch02 一张地图喂饱所有算法：MapField** | 从暴力最近墙查找 $O(HW)$ 到 Felzenszwalb–Huttenlocher 1D/2D 精确 EDT $O(HW)$ 抛物线下包络推导（含负数 `floor_div` 边界坑）；SDF 双线性插值与解析梯度 $\nabla \text{SDF}$；高斯似然场 $\exp(-d^2 / 2\sigma^2)$；Cartographer 滑窗最大值栈 `swmax_`（$2^k \times 2^k$ 锚点卷积） | [map_field.hpp](../src/scan_localizer/include/scan_localizer/map_field.hpp), [map_field.cpp](../src/scan_localizer/src/map_field.cpp) | **动画 2**：可手绘画墙的 **4 层地图场切换器**（占据栅格 / SDF+梯度箭头 / 似然场 / 金字塔 `k=0..4`）+ **1D EDT 抛物线包络步进动画** |
| **第二阶段：扫描匹配演进** | **Ch03 从暴力搜索到经典匹配器：Correlative / ICP / NDT** | Olson 窗口穷举相关性匹配（旋转点云复用与两级粗/精搜索）；点到点 ICP 的旋转力臂雅可比与“掉进局部极小”问题；2D NDT 体素高斯拟合 $(\boldsymbol{\mu}, \Sigma)$ 与 D2P 指数得分最大化 | [matchers.hpp](../src/scan_localizer/include/scan_localizer/matchers.hpp), [matchers.cpp](../src/scan_localizer/src/matchers.cpp) | **动画 3**：**Correlative 得分热力图 vs ICP-P2P 逐步拉扯对比**（拖大初值误差观察 ICP 掉坑）+ NDT 体素椭圆场 |
| | **Ch04 王牌精修算法：SDF 连续场高斯-牛顿（Field-GN）** | 为何在 SDF 场上无需数据关联即可实现点-线等价；链式法则雅可比 $\mathbf{J}_i = \nabla\text{SDF}(\mathbf{p}_w) \cdot \frac{\partial \mathbf{p}_w}{\partial \boldsymbol{\xi}}$；Huber IRLS 权重与 `trim_dist` 残差截断抑制行人/货箱遮挡；LM 阻尼、非单调拒绝回滚；`ComposedMatcher`（`Correlative → FieldGn`）的两级互补原理 | [matchers.cpp](../src/scan_localizer/src/matchers.cpp) (`FieldGnMatcher`, `ComposedMatcher`) | **动画 4**：**Field-GN 逐迭代可视化**（每束射线法向力矢量、残差直方图与收敛曲线）+ 一键投放「动态行人」对比 L2 与 Huber |
| | **Ch05 长走廊退化与各向异性协方差** | 特征退化（光秃长走廊）的线性代数本质：$H = \mathbf{J}^\top \mathbf{W} \mathbf{J}$ 出现接近零的特征值；为什么匹配协方差 $\Sigma = \sigma^2 (H)^{-1}$ **绝不能带入 LM 阻尼项 $\lambda I$**；如何用特征分解检验不可观测方向（长走廊 120,000× 各向异性方差） | [matchers.cpp](../src/scan_localizer/src/matchers.cpp) (`eval_stats`), [localizer_core.cpp](../src/scan_localizer/src/localizer_core.cpp) | **动画 5**：**长走廊 Hessian 协方差椭圆实验室**（直观对比标量滤波 / 带阻尼假协方差 vs 各向异性 EKF 锁横向、放沿轴） |
| **第三阶段：全局定位与滤波状态机** | **Ch06 迷路了怎么办：B&B 全局定位 vs 粒子滤波（AMCL-lite）** | 冷启动与机器人绑架问题；基于 `swmax_` 的金字塔 Branch-and-Bound 上界剪枝、优先队列展开与 Top-K NMS 去重；对比 `ParticleFilter`（Thrun 反向运动分解、log-sum-exp 混合似然防浮点下溢、ESS 自适应重采样） | [matchers.cpp](../src/scan_localizer/src/matchers.cpp) (`global_correlative`), [particle_filter.hpp](../src/scan_localizer/include/scan_localizer/particle_filter.hpp), [particle_filter.cpp](../src/scan_localizer/src/particle_filter.cpp) | **动画 6**：**B&B 优先队列剪枝树动画 vs 500 粒子滤波收敛竞赛**（支持“一键绑架”观察多假设晋升） |
| | **Ch07 4D 各向异性 EKF、四道门控与状态机** | 4 维状态 $[x,y,\theta,b_g]^\top$ 在线估计 IMU 陀螺零偏；SE(2)+Bias $4\times 4$ 雅可比预测与 Joseph 形式协方差更新；四道门控（`min_score` / `min_inlier_ratio` / $\chi^2(3)$ 马氏门限 / 最小特征值）；五态机 `UNINIT/TRACKING/DEGRADED/LOST/GLOBAL`；多假设队列与真实故障复盘（静止马氏锁死 & 对称走廊误跳变 `98fcd28`） | [localizer_core.hpp](../src/scan_localizer/include/scan_localizer/localizer_core.hpp), [localizer_core.cpp](../src/scan_localizer/src/localizer_core.cpp) | **动画 7**：**4D EKF 零偏收敛曲线 + 五态机门控模拟器**（手动注入陀螺零偏 / 动态遮挡 / 绑架 / 对称走廊，实时观察状态流转） |
| **第四阶段：工程落地与验证** | **Ch08 ROS 2 生产级节点架构：LocalizerNode** | 算法核零 ROS 依赖分层；`LifecycleNode` 五回调与 `autostart: false`；`state_mtx_` 锁外构场与 `applyMap()` 原子交换 + 锁外发布 `OutputSnapshot`；TF2 3D 外参与扫瞄时间戳插值；6×6 协方差 `[35]` 偏航角下标坑；动态调参回调与话题/服务契约 | [localizer_node.hpp](../src/scan_localizer/include/scan_localizer/localizer_node.hpp), [localizer_node.cpp](../src/scan_localizer/src/localizer_node.cpp), [localizer.yaml](../src/scan_localizer/config/localizer.yaml), [demo.launch.py](../src/scan_localizer/launch/demo.launch.py) | **动画 8**：**全链路迷你仓库定位驾驶舱**（端到端闭环运行预测→匹配→4D EKF→状态机，支持切换匹配器与绑架测试）+ Lifecycle 状态流转图 |
| | **Ch09 仿真测试、E1–E10 消融实验与 Intel Lab 真值三方评测** | 受控实验设计方法论（如何避免“泄露真值”、“随机种子死循环”、“重分箱角度偏置”等 E1–E10 评测陷阱）；ASan/UBSan 与 9 组 GTest 单测/集成/回归测试；Intel Lab 450 组手工精校真值三方实测分析（`scan_localizer` vs `nav2_amcl` vs `slam_toolbox`） | [sim_world.hpp](../src/scan_localizer/include/scan_localizer/sim_world.hpp), [sim_world.cpp](../src/scan_localizer/src/sim_world.cpp), [benchmark.cpp](../src/scan_localizer/test/benchmark.cpp), [benchmark_results_x86.txt](../src/scan_localizer/docs/benchmark_results_x86.txt) | **交互图表**：E1–E10 误差/耗时对比交互图 + 嵌入 Intel Lab 真值轨迹/精度/系统开销实测图表 |
| **第五阶段：速查与索引** | **Ch10 C++20 × Eigen × ROS 2 语法与工程技巧实战手册** | 横跨全项目的现代 C++20、Eigen 数值代数、零分配缓冲优化、并发锁设计与 ROS 2 Lifecycle/TF2/QoS 陷阱速查卡片库 | 横跨 `src/scan_localizer` 全仓库典型片段 | **交互筛选器**：按 `C++20` / `Eigen` / `内存与并发` / `ROS 2` / `数值稳定` 标签实时过滤语法与工程卡片 |
| **独立索引页** | **`code-index.html` 项目代码全景跳转索引** | 覆盖 [src/scan_localizer](../src/scan_localizer/README.md) 全部 33 个文件（头/源/测/配/Launch/CMake），提供 **3 条阅读路线**（3h 速通 / 算法核精读 / 全栈工程） | `src/scan_localizer` 全部 33 个文件 | **可点击架构依赖图（SVG）+ 符号搜索框 + 实时源码抽屉（精确行号高亮 + VS Code 一键跳转）** |

---

## 三、项目代码跳转索引的 3 层体验设计

为了让读者像看“带地图的源码百科”一样读懂约 11,900 行代码，`code-index.html` 与各章节联动提供 3 层跳转机制：

1. **全景交互式架构图（SVG）**
   - 首页与 `code-index.html` 均放置分层依赖图（`SE2 / MapIO / MapField` → `Matchers / ParticleFilter` → `LocalizerCore` → `LocalizerNode / SimWorld`）；
   - 点击任意模块卡片，即刻展开职责说明、关键函数清单、调用链与对应章节入口。
2. **双模式精确行号跳转**
   - 教程正文每处公式与算法模块旁均附可点击源码徽章（例如 `src/localizer_core.cpp:L310-L378 · 4D EKF 更新`）；
   - 通过 `serve.sh` 打开教程站时，点击徽章直接滑出**右侧实时源码抽屉**（直接读取仓库真实文件，Highlight.js 高亮并自动滚动定位目标行区间）；
   - 同时提供一键 **VS Code 跳转**（`vscode://file/...`）与**复制路径:行号**按钮。
3. **函数与文件级反向索引表**
   - 将全部 33 个文件按「基础几何与地图 / 匹配器与全局搜索 / 滤波与状态机 / ROS 2 节点与配置 / 仿真器与测试套件」分组；
   - 提供符号/关键词实时检索框，命中后展示「职责摘要 + 文件:行号 + 对应教程章节」。

---

## 四、重点语法讲解与真实工程踩坑专题清单

### 4.1 C++20 / Eigen / ROS 2 高频语法专题（对应 Ch10 与各章 Syntax Spotlight）

- **C++20 特性**：`std::numbers::pi`、`std::random_access_iterator` 概念约束、`if constexpr` 编译期分支（[se2.hpp](../src/scan_localizer/include/scan_localizer/se2.hpp)）；`std::filesystem::path` 路径拼接；`std::atomic<bool>` 状态门控；`std::scoped_lock` / `std::unique_lock` 手动解锁配合快照发布。
- **Eigen 技巧**：`Eigen::Matrix3d` / `Matrix4d` / `Vector2d` 固定尺寸栈分配；`LDLT` 与 `SelfAdjointEigenSolver` 求解小规模对称正定 Hessian 及协方差；`topLeftCorner<3, 3>()` 块视图零拷贝切片；Joseph 形式 `(I - K H) P (I - K H)^T + K R K^T` 保证数值对称正定性。
- **零分配热点优化**：`transform_scan(..., std::vector<Vec2> & out)` 传入复用缓冲，避免雷达回调热路径频繁 `malloc/free`；`resample_buf_`、`log_w_buf_` 成员缓冲复用。
- **ROS 2 Jazzy 工程特性**：`rclcpp_lifecycle::LifecycleNode` 状态机与 `LifecyclePublisher`；`CallbackGroup` 传感器/工作回调组分离与 `MultiThreadedExecutor`；`SensorDataQoS` vs `TransientLocal` QoS 匹配；`tf2_ros::Buffer` + `TransformListener` 查表与容差外推；`add_on_set_parameters_callback` 热参数校验；`ament_cmake_gtest` + `scan_localizer_sim_lib` 剥离仿真依赖。

### 4.2 来自实战重构的十大工程陷阱复盘（穿插 Ch01–Ch09）

1. **PGM 与栅格原点错位**（Phase 1）：PGM 第 0 行是图像顶部，漏掉 `row_pgm = height - 1 - iy` 导致 Nav2 地图上下颠倒。
2. **2D EDT 整数除法向零截断 Bug**（Phase 1）：`s` 计算未用精确向下取整 `floor_div`，导致距离场局部凹陷错位。
3. **裸指针悬空风险与不可观测区 SDF 符号**（Phase 1）：`MapField` 改用 `std::shared_ptr<const OccupancyGrid>` 消除局部栅格析构后的 UAF。
4. **ICP 旋转雅可比零力臂与 Correlative 重复偏移**（Phase 2）：ICP 未以当前扫描旋转点作力臂导致角度收敛停滞。
5. **B&B 剪枝失效与错误淘汰序列**（Phase 2）：100% 边界约束与 `worst_kept` 更新缺陷导致全局定位卡死或漏掉真解。
6. **阻尼协方差掩盖长走廊退化**（Phase 3）：带 LM 阻尼求逆给出虚假高置信度，破坏各向异性 EKF。
7. **Covariance 6×6 数组把偏航角方差写入 z 轴 `[14]` 而非 `[35]`**（Phase 4）：行优先 `(5,5)` 索引误写成 `(2,2)`。
8. **`autostart` 绕过 Lifecycle 状态机 & `on_cleanup()` 状态残留**（Phase 4）：IMU 初值锚点未清零导致重配置后首帧航向跳变。
9. **基准测试 E1–E10 自欺陷阱**（Phase 5）：`rng()` 取模误用导致注入 0 个动态障碍、`scan` 重分箱产生 `-0.25°` 系统误差。
10. **静止马氏锁死 & 对称走廊 `GLOBAL` 误跳变**（提交 `98fcd28`）：高得分高内点但被 `mahal_gate` 拒收时连续触发 `GLOBAL` 绑架，且外部候选未与内部替代假设隔离引发 20 m 错跳。

---

## 五、实施里程碑（M1–M4）与自动验收标准

后续正式实施时，按 4 个里程碑推进，并以 **4 项硬性自动检查**收口：

- **M1 · 站内框架与全景代码索引**：搭建 `docs/scan_localizer_tutorial/` 共享样式、侧栏目录、KaTeX + Highlight.js、实时源码抽屉、`serve.sh`、`index.html` 与 `code-index.html`（含 33 个文件精确行号表）。
- **M2 · 基础与扫描匹配篇（Ch01–Ch05 + 动画 1–5）**：完成 `sim-core.js` 公共几何/射线/EDT 引擎及前 5 章动画交互演示与推导。
- **M3 · 全局定位、4D EKF、ROS 2 工程与语法手册（Ch06–Ch10 + 动画 6–8）**：完成 B&B vs PF 动画、4D EKF 五态机模拟器、全链路驾驶舱、Intel Lab 实测图表融合与 Ch10 语法筛选手册。
- **M4 · 文档接入与全站自动验收**：编写 `docs/30_scan_localizer教程导览.md` 并更新 [docs/README.md](README.md)，执行以下 4 项验收：
  1. **Node.js 仿真数学核自检**：对 `sim-core.js` 的 $SE(2)$、光线投射、1D/2D EDT、Field-GN 梯度与 4D EKF 数值做断言测试，确保浏览器动画与 C++ 核心算法数学一致；
  2. **源码行号一致性脚本检查**：校验教程与 `code-index.html` 中出现的每一处 `文件:行号` 均存在且行数未越界；
  3. **无头浏览器全站巡检**：逐页打开全部页面，验证 0 JS 报错、0 404 资源请求、KaTeX 0 渲染错误、Canvas 正常绘制；
  4. **仓库文档链接检查**：运行 `python3 docs/tools/check_tutorial_links.py`，要求 **0 缺失链接**。

---

## 六、正式开工前待确认的 3 个小决策

1. **目录位置**：放在 `docs/scan_localizer_tutorial/`（接入 [docs/README.md](README.md)，保持仓库根目录整洁），还是放在仓库根目录 `scan-localizer-academy/`？
2. **视觉风格**：偏好类似 `slam-mpc-academy/` 的**明亮教科书风**、类似 [docs/website](website/index.html) 的**暗黑霓虹风**，还是提供右上角**一键切换深/浅色**？
3. **推进节奏**：一次性按 `M1 → M4` 全部开发完成后统一验收，还是每完成一个里程碑暂停预览？
