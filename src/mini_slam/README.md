# mini_slam —— 从零手写 slam_toolbox

一份可读、可跑、可对拍的 2D 图优化 SLAM 复现，对应 Steve Macenski 的
[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)（血统：SRI KartoSLAM / libkarto）。

**算法三大件**：

| 部件 | 论文/出处 | 本仓库实现 |
|---|---|---|
| correlative scan matching | Olson, *Real-Time Correlative Scan Matching*, ICRA 2009 | `core/src/correlative_matcher.cpp` |
| 位姿图 + SPA | Konolige 等, *Efficient Sparse Pose Adjustment for 2D Mapping*, IROS 2010 | `core/src/spa_solver.cpp`（Ceres autodiff + 稀疏 LM） |
| 建图策略/闭环 | KartoSLAM（libkarto） | `core/src/mapper.cpp` |

## 算法流水线（对齐 slam_toolbox 的 MapperProcessScan）

```
/scan + /odom → 位移阈值(0.5m/0.5rad)判定
  → running scans 顺序匹配（Olson 两级穷举，细调窗口 ±0.15m）
  → 节点入图：里程计边（纯增量）+ 参照链匹配边（response² 加权）
  → 闭环候选（空间半径 3m 内、时间间隔 ≥20 帧、≥10 帧候选链）
      宽窗 ±3.5m 粗搜 → 方差门控(≤10m²) → 细搜 → 阈值 0.4/0.5（总点数归一）
  → 闭环边入图 → Ceres SPA（SPARSE_NORMAL_CHOLESKY，首节点固定）
  → log-odds 射线渲染 → OccupancyGrid
```

几个容易踩的坑（本仓库都以注释形式钉在代码里）：

1. **得分场必须二值标记**：近处墙点密、远处点稀，累加会让峰值被近处密度
   垄断，远处墙的响应被系统性压低（karto 的 CorrelationGrid 即二值）。
2. **归一化模式要分场景**：顺序匹配用"场内点归一"（参考场只是局部）；
   闭环验证必须"总点数归一"——出界点计 0 是防假闭环的护栏。
3. **running scans 与闭环候选要解耦**：前者是顺序匹配的参照集（可以慷慨，
   老节点还能帮锚定），后者靠时间间隔排除自匹配。混用会让绕圈场景的
   闭环永远凑不满候选链。
4. **窗口枚举别写成 2W**（本项目真实踩过：±W 写成了 ±2W，假闭环元凶）。
5. **刚柔配比**：里程计边太硬 → 闭环掰不动轨迹；太软 → 图没有刚度。
   σ 按"每步里程计漂移"给。

## 构建

纯算法核（无 ROS 依赖）：

```bash
cd src/mini_slam
cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMINI_SLAM_WITH_ROS=OFF
cmake --build build -j
ctest --test-dir build          # 纯算法核 17 个测试（含 ROS 时共 21 个）
```

含 ROS 包装（colcon，需在 ROS 环境中）：

```bash
colcon build --packages-select mini_slam
colcon test --packages-select mini_slam
colcon test-result --verbose    # 当前 76 个测试断言，0 failure
```

> 跨进程参数测试（`test_param_cli`）在本机需要
> `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` + `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`
> （CMake 已把这两个变量写进测试环境）；默认 Fast DDS 在本机跨进程发现不通，
> 细节与配置表见 `docs/07_参数与服务接口.md`。

## 跑数据集（Intel Lab，CARMEN 格式）

```bash
# 数据放独立 data/ 目录（见 datasets.php 与镜像，intel.clf）
./build/mini_slam_dataset --input data/intel.clf --out out_intel
# 产物：out_intel/map.pgm + map.png（含里程计/优化轨迹叠加）+ map.meta + trajectory.svg
```

## 跑 ROS（在线 / bag 回放）

生产入口是 `MultiThreadedExecutor(4)`；`/tf` 由 `TransformListener` 的专用
线程消费（`spin_thread=true`），worker 是第三条独立线程。`scan_callback`
里 20ms 的 TF lookup 等待不再挡住 TF 入库——详见
`docs/07_参数与服务接口.md` 的线程模型一节。

```bash
ros2 launch mini_slam mini_slam_online.launch.py \
    use_sim_time:=true map_output_prefix:=out_tb3/mini_map
# 参数名对齐 slam_toolbox：resolution / minimum_travel_distance / loop_match_* 等
# graph_path 非空时节点启动即自动加载该图；运行期修改 graph_path 只改变
# 后续 save_graph/load_graph 的默认路径，不会立即重新加载（见 docs/07）。
```

## 交互式建图（RViz 拖拽位姿图节点）

```bash
ros2 launch mini_slam mini_slam_online.launch.py enable_interactive_markers:=true
# RViz: Add → InteractiveMarkers → Update Topic 改 /mini_slam/update
# 拖动节点后松手 → 固定该节点 + node0 重跑 SPA → 地图/marker 刷新
```

标准 `interactive_markers`（不写自定义 RViz 插件），默认关闭；参数与抽样规则
见 `docs/08_交互式建图.md`。

## 一键闭环测试（建图 → 存图 → 定位 → A* 导航 → 到达验证）

```bash
cd ~/ros2_jazzy/nav2_astar_ws/src/mini_slam
./scripts/closed_loop_test.sh                   # 完整闭环：建图 + AMCL 定位 + A* 导航
./scripts/closed_loop_test.sh --skip-mapping    # 跳过建图，直接用上次的 mini_map.yaml
./scripts/closed_loop_test.sh --rviz            # 同时开 RViz 观看
```

脚本内置本机环境全部踩坑的自动化处理（详见脚本头注释 P1–P6）：
启动前清场防僵尸进程（P1）、数据流健康门禁后才激活 nav2（P2）、
初始位姿用仿真时间戳（P3）、激活与定位上线后各清一次代价地图防幽灵障碍（P4）、
从实时代价图选代价 ≤60 的开阔目标点（P5）、新目标自动抢占旧目标（P6）。

已验证结果（TurtleBot3 沙盒）：mini_slam 在线建图（3 圈方形、12 次闭环）→
AMCL 在 mini_slam 地图上定位收敛 → nav2_astar_planner A* 全局规划
（45 格路径 / 0.06ms）→ PurePursuit 控制到达，误差 0.22m（容差 0.25m 内，SUCCEEDED）。
注意：CBF 安全过滤器在窄柱道会与 PurePursuit 速度死锁，测试时用
`ros2 param set /cbf_safety_filter enabled false` 旁路（collision_monitor 仍兜底）。

## 与 slam_toolbox 对拍

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
ros2 run nav2_map_server map_saver_cli -f slam_map
./build/mini_slam_compare mini_map.pgm slam_map.pgm overlay.png
```

## C++23 备忘（写这套代码时实际用到的）

- `std::expected` + monadic `and_then`：解析/匹配的失败路径（错误即类型）。
- `std::format` 编译期格式串检查 + 特性探测宏（`__cpp_lib_print`）兼容
  GCC 13（无 `<print>`）与 GCC 14+（有 `std::println`）。
- `std::span` 传点云视图，零拷贝。
- `constexpr` SE(2) 数学；`std::ranges::sort` + projection 写闭环候选排序。
- GCC 14 才有的 `deducing this` / `std::print` / `ranges::to`：探针
  （`feature_probe.cpp`）在编译期检测并汇报，代码自动走回退路径。

## 已知边界（诚实记录）

- **Intel Lab PowerBot 里程计**（终点漂 60m+）超出单轨迹图 SLAM 的适用
  范围——slam_toolbox 文档同样要求 reasonably good odometry。我们在此
  数据集上完成了：真闭环零扭曲（方差门控后）、局部走廊成图锐利、图整体
  尺寸正确；但全局仍受"顺序匹配对漂移图的反馈"拖累。RBPF 类方法
  （gmapping）靠粒子多假设处理这类输入，是架构层面的差异。
- 合成世界端到端（含闭环）：0.55m 漂移优化后残差 0.08m。
