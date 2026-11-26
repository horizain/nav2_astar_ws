# scan_localizer —— 生产级 2D 激光已知地图定位

一套**实验驱动设计**、可直接产品化的室内移动机器人 2D 激光定位方案。面向 2D 激光雷达 + 嵌入式主控（RK3399/RK3588）场景，兼顾确定性高精度、退化安全性与可控算力预算。

- **算法核心零 ROS 依赖**（纯 C++20 + Eigen），可独立单测、独立交叉编译
- **五种匹配器 + 粒子滤波**全部实现并在统一基准下对比（**E1–E10**，见下）
- **4 状态各向异性 EKF** `[x, y, θ, b_g]^T`：在线估计陀螺零偏 `b_g`，支持有/无里程计运行与多假设验证恢复
- **ROS 2 Jazzy 生命周期组件节点**（默认 `autostart: false` 对齐 Nav2 `lifecycle_manager`），支持 TF2 3D 外参、时间戳对齐的里程计查询/插值、动态参数热更新与 YAML 地图热切换
- **端到端仿真与完整测试集**：冷启动全局定位 → 104m 仓库环路跟踪（含人群遮挡段），9 组 gtest 单测/集成/回归测试 + 全套 ament lint

```
┌────────────────────────── LocalizerNode (lifecycle 组件) ──────────────────────┐
│  /map 或 map_yaml ─► MapField 预计算（EDT→SDF→似然场→滑窗最大值栈）             │
│  /odom + /imu     ─► 4D EKF 预测 [x, y, θ, b_g]（TF2/环形缓冲插值 + 滑移模型）  │
│  /scan (TF2 3D外参)► ┌──────────── 两级匹配 ────────────┐  四道门控：得分/内点/ │
│                      │ correlative 捕获 → field-GN 精修 │  马氏/最小特征值      │
│                      └────────────────────────────────┘  拒绝→纯预测外推        │
│  状态机 UNINIT→TRACKING ⇄ DEGRADED → LOST / GLOBAL（B&B 滑窗栈 + top-K 多假设） │
│  输出 TF map→odom（扫描时刻对齐 + 保活）/ localizer/pose / status / hypotheses  │
│  服务 localizer/set_fine_mode / global_relocalize / nomotion_update            │
└────────────────────────────────────────────────────────────────────────────────┘
```

---

## 一、实验结论：哪个组件用哪种算法

所有数据来自 [docs/benchmark_results_x86.txt](docs/benchmark_results_x86.txt)（`ros2 run scan_localizer scan_localizer_benchmark` 可完整复现）。仓库仿真 40×30m（800×600 cells，5cm 分辨率，外墙/隔间/货架阵列/柱阵），720 束 12m 雷达，MapField 一次性构建耗时 **27.7 ms**。**所有匹配器共用同一张 SDF 似然场地图（σ=0.20m）并在配对试验上评测**。

### E1 跟踪精度（先验误差 ~2cm/0.5°，无噪扫描，100 帧配对）

| 匹配器 | 位置中位误差 | p90 | 航向中位误差 | 耗时 (ms) | 成功率 (<5cm & <1°) |
|---|---|---|---|---|---|
| correlative | 20 mm | 30 mm | 0.092° | 6.36 | 100.0% |
| **field_gn (SDF 场 GN)** | **20 mm** | **29 mm** | **0.047°** | **0.054** | **100.0%** |
| icp_p2p | 20 mm | 31 mm | 0.047° | 0.297 | 100.0% |
| ndt | 35 mm | 35 mm | 0.001° | 0.132 | 99.0% |
| **composed** | **20 mm** | **29 mm** | **0.047°** | 6.39 | **100.0%** |

> 注：在 5cm 栅格地图上，激光射线命中占据格前沿面与占据格中心（SDF=0）存在半胞元（25mm）几何量化差，因此干净扫描的中位位置误差收敛在 ~20mm 物理下界。

**结论 1：精修级用 SDF 连续场高斯-牛顿（点-线 ICP 的无数据关联等价）。**
残差 = SDF 双线性插值，雅可比 = SDF 解析梯度，单帧仅 **0.054 ms**，比 ICP-P2P（0.297 ms）快 5.5 倍且在大初值扰动下捕获范围远优于 ICP-P2P（见 E2）。NDT 受 0.5m 体素平滑限制，位置误差稳定在 ~35mm，但姿态角精度高、抗差稳健。

### E2 捕获范围（跨匹配器配对试验，大初始误差下的成功率）

| 匹配器 | 0.05m / ±2° | 0.15m / ±5° | 0.40m / ±10° | 0.80m / ±18° | 1.50m / ±30° | 大误差耗时 (ms) |
|---|---|---|---|---|---|---|
| correlative (±0.15m) | 100.0% | 100.0% | 0.0% | 0.0% | 0.0% | 6.0 |
| correlative_wide (±1.6m) | 100.0% | 100.0% | 100.0% | 100.0% | 100.0% | 208.8 |
| field_gn 单独 | 100.0% | 100.0% | 98.0% | 48.0% | 6.0% | 0.2 |
| icp_p2p 单独 | 100.0% | 100.0% | 28.0% | 10.0% | 4.0% | 1.5 |
| ndt 单独 | 100.0% | 68.0% | 0.0% | 0.0% | 0.0% | 0.2 |
| **composed (±0.15m)** | **100.0%** | **100.0%** | **100.0%** | **86.0%** | 16.0% | **6.2** |
| **composed_wide (±1.6m)** | **100.0%** | **100.0%** | **100.0%** | **100.0%** | **100.0%** | 208.8 |

**结论 2：捕获级用 correlative（Olson 窗口穷举）+ field-GN 精修组合。**
单独 `field_gn` 在 0.80m 以上容易落入局部极小（48%），`icp_p2p` 在 0.40m 即降至 28%；`composed` 默认 ±0.15m 窗口可在 0.40m 内保持 100%、0.80m 达 86%（耗时 6.2 ms）；宽窗 `composed_wide` 在 1.50m / ±30° 下仍 100% 捕获（~209 ms，适合异常恢复突发调用）。

### E3/E4 传感器噪声与动态障碍鲁棒性（配对试验）

| 场景 | correlative | field_gn | field_gn_tight | ndt | composed | composed_tight |
|---|---|---|---|---|---|---|
| E3 测距噪声 σ=0/1/3/5cm（中位误差 mm） | 19/19/15/30 | **20/20/14/14** | — | 35/35/30/26 | **20/20/14/14** | — |
| E4 动态障碍（每帧实注 2 个 0.7m 近距箱，共 200 箱） | 21mm / 99% | **22mm / 99%** | **22mm / 99%** | 35mm / 99% | **22mm / 99%** | **22mm / 99%** |

**结论 3：Huber 鲁棒核 + 残差截断（`trim_dist = 0.50m`）足以应对常规动态遮挡。**
在每帧确认注入 2 个近距动态箱（1.2–2.8m）的条件下，全部匹配器保持 99% 成功率，位置中位误差仅增加 ~2mm。`tight` 配置（`trim_dist=0.20m, huber_delta=0.05m`）作为重度人流/对接模式参数预留。

### E5 长走廊严格退化（30×3m，严格采样 `x ∈ [13, 17]`，两端墙均超 12m 量程）

| 匹配器 | 沿轴误差 (mm) | 横向误差 (mm) | 沿轴方差 `cov_along` (m²) | 横向方差 `cov_across` (m²) |
|---|---|---|---|---|
| correlative | 179.2 | 2.6 | 0.009 | 0.00019 |
| **field_gn** | **13.6** | **0.3** | **12.482** | **0.00010** |
| icp_p2p | 13.2 | 0.3 | 16.613 | 0.00010 |
| ndt | 16.1 | 25.0 | 0.068 | 0.00073 |
| composed | 179.2 | 0.3 | **12.482** | **0.00010** |

**各向异性 EKF 走廊行走 4m（2% 轮式滑移，16 帧全采纳 16/16）**：
- 横向最大误差：**0.0 mm**（激光锁死侧墙，`cov_across = 0.0001 m²`）
- 沿轴最大误差：**79.8 mm**（精确等于 4m × 2% 轮式滑移，沿不可观测方向自动信任里程计，`cov_along = 0.0017 m²`）

**结论 4：协方差必须来自未阻尼 Hessian，融合必须各向异性。**
`field_gn` 沿走廊方向协方差高达 **12.48 m²**，横向仅 **0.0001 m²**（>120,000× 各向异性）；各向异性 EKF 自动在横向采用激光观测、在沿轴方向保留里程计预测，既不整帧误拒，也不引入虚假修正。

### E6/E7 扫描密度与单帧 CPU 耗时（720 束，x86 Release）

| 匹配器 | 720 束 | 360 束 (`subsample=2`) | 180 束 | 90 束 | E7 720 束 p50 / p90 / mean (ms) |
|---|---|---|---|---|---|
| correlative | 19.6mm / 6.42ms | 19.3mm / 3.28ms | 19.2mm / 1.68ms | 19.3mm / 0.88ms | 6.36 / 8.01 / 6.44 |
| field_gn | 19.7mm / 0.06ms | 19.8mm / 0.03ms | 20.5mm / 0.02ms | 20.3mm / 0.01ms | **0.054 / 0.075 / 0.057** |
| icp_p2p | — | — | — | — | 0.297 / 0.448 / 0.307 |
| ndt | — | — | — | — | 0.132 / 0.348 / 0.177 |
| **composed** | **19.9mm / 6.46ms** | **19.8mm / 3.31ms** | 20.5mm / 1.70ms | 20.3mm / 0.87ms | **6.39 / 8.16 / 6.50** |

**结论 5：`scan_subsample: 2`（360 束）可将 `composed` 耗时直接减半（6.46ms → 3.31ms），精度损失 <1mm。**

### E8 全局重定位（冷启动，30 次试验，射线生成在计时外）

| 方案 | 成功率 (<0.3m & <3°) | 中位耗时 | 备注 |
|---|---|---|---|
| **B&B correlative（滑窗最大值栈）** | **76.7%** | **20.38 ms** | 单帧确定性搜索，输出 top-5 假设 |
| 粒子滤波（20000 粒子 × 10 步无碰撞运动） | 6.7% | 445.89 ms | 中位误差 15.60 m，需运动且覆盖不足 |

**结论 6：全局定位用滑窗最大值栈 B&B correlative + top-K 多假设队列，不用粒子滤波。**
在 1200 m² 仓库中，B&B 单帧耗时 **~20 ms**，速度比 2 万粒子 PF 快 **22 倍**、召回率高 **11 倍**；对称货架区的歧义由 `LocalizerCore` 的多假设队列在后续帧随运动确认消歧。

### E9 轨迹级架构对比（104m 环路 × 2% 轮滑 × 10 帧人群遮挡，同输入同 MapField σ=0.20m）

| 架构 | 平均误差 | 最大误差 | 采纳率 | CPU 耗时 |
|---|---|---|---|---|
| **A: LocalizerCore + composed** (`window_lin=0.25`) | **21.4 mm** | **35.6 mm** | **100.0%** | 11.26 ms/scan（总 1.17 s） |
| B: 粒子滤波（2000 粒子） | 33.2 mm | 215.8 mm | — | 3.96 ms/scan（总 0.41 s） |

**结论 7：`LocalizerCore + composed` 最大误差仅为粒子滤波的 1/6（35.6mm vs 215.8mm）且全程 100% 确定性收敛。**
E9 中架构 A 放宽了搜索窗（`window_lin=0.25`）且未降采样（720 束）；生产配置 `window_lin=0.15` + `scan_subsample=2` 下 `composed` 耗时约 **3.3 ms/scan**，在算力持平的同时保持显著的精度与最坏情况上界优势。

### E10 IMU 陀螺航向融合（104m 环路 × 20 帧重度人群遮挡，陀螺零偏 0.02 rad/s + 随机游走）

| 工况 | 平均误差 | 最大误差 | 采纳率（拒绝帧） | 在线零偏估计 `b_g`（真值） |
|---|---|---|---|---|
| 3% 常规轮式旋转打滑，仅轮式 | 20.7 mm | 33.8 mm | 100.0% (0) | — |
| 3% 常规轮式旋转打滑，+IMU (4D EKF) | 20.7 mm | 33.8 mm | 100.0% (0) | **0.0202** (0.0199 rad/s) |
| 10% 地毯重打滑，仅轮式 | 6.98 m（发散） | 12.87 m | 10.6% (93) | — |
| **10% 地毯重打滑，+IMU (4D EKF)** | **23.3 mm** | **121.6 mm** | **97.1% (3)** | **0.0200** (0.0198 rad/s) |

**结论 8：4D EKF 陀螺融合将可承受的轮式旋转打滑余量从 ~3% 提升至 >10%，并能在线精准收敛陀螺零偏 `b_g`（误差 < 0.0003 rad/s）。**

### 总结论（决策表）

| 组件 | 推荐方案 | 实验依据 |
|---|---|---|
| 精修匹配 | **SDF 连续场 GN**（点-线等价） | E1/E3/E5/E7 全维最优（0.054 ms） |
| 捕获匹配 | **correlative → field_gn (`composed`)** | E2 窗口内 100% 无局部极小 |
| 跟踪架构 | **4D 各向异性 EKF + 四门限状态机** | E5 退化有界、E9 最大误差降为 PF 的 1/6 |
| 全局重定位 | **滑窗最大值栈 B&B + top-K 多假设队列** | E8：20 ms、76.7% 召回，22× 快于 PF |
| 动态抑制 | Huber + 3σ `trim_dist` | E4 实注 200 动态箱保持 99% 成功率 |
| 航向鲁棒性 | 4D EKF 陀螺融合（`use_imu: true`） | E10：10% 重打滑仍 97.1% 采纳且在线估计 `b_g` |
| 对接精修 | `localizer/set_fine_mode` 服务热切换紧配置 | 缩小搜窗/截断并提高内点率门限至 0.75 |

---

## 二、构建、测试与使用

### 构建与全量测试

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select scan_localizer --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 运行全部 9 组 gtest（含单测、节点集成测试与基准回归测试）+ ament lint
colcon test --packages-select scan_localizer --ctest-args --output-on-failure
colcon test-result --all --test-result-base build/scan_localizer --verbose
```

### 端到端演示（自带仿真，无需硬件）

```bash
ros2 launch scan_localizer demo.launch.py
# 独立进程模式（关闭组件容器）：
ros2 launch scan_localizer demo.launch.py use_composition:=false

# 另一终端观察：
ros2 topic echo /localizer/status          # TRACKING / score / inlier_ratio / fine_mode
ros2 topic echo /localizer/pose            # PoseWithCovarianceStamped (map 系)
ros2 run tf2_ros tf2_echo map odom         # 定位输出 TF
```

仿真源 `scan_localizer_sim` 在 40×30m 仓库中运行 104m 矩形环路（2 圈 × 52m，带 2% 轮滑与 10 帧人群遮挡），同步发布 `/map`、`/odom`、`odom→base_link` TF、`/imu`、`/scan`，并订阅 `/localizer/pose` 在运行结束后打印定位误差统计。

### 接入真实机器人

节点默认 `autostart: false`（适配 Nav2 `lifecycle_manager`），单机调试可传入 `-p autostart:=true`：

| 输入 / 服务 | 类型 | 说明 |
|---|---|---|
| `map`（或参数 `map_yaml`） | `nav_msgs/OccupancyGrid` | transient_local 地图话题，或配置 `map_yaml` 免 `map_server` 直载 |
| `scan` | `sensor_msgs/LaserScan` | SensorDataQoS；优先通过 TF2 查询 `base_frame → scan.frame_id` 完整 3D 外参并投影到 2D，无 TF 时回退 `laser_x/y/theta` |
| `odom`（可选） | `nav_msgs/Odometry` | 优先查 TF2 `odom_frame → base_frame`，回退时间戳环形缓冲线性/角度插值；无 `/odom` 时自动以无里程计模式运行 |
| `imu`（可选） | `sensor_msgs/Imu` | SensorDataQoS；`use_imu: true` 时驱动 4D EKF 在线估计陀螺零偏 `b_g` |
| `initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz `2D Pose Estimate` |
| `localizer/map_path` | `std_msgs/String` | YAML 地图热切换 |
| `localizer/global_hypotheses` | `geometry_msgs/PoseArray` | 外部视觉/二维码/UWB 候选位姿云注入 |
| `localizer/set_fine_mode` | `std_srvs/SetBool` | 切换紧配置（`window_lin=0.10, window_ang=0.05, huber_delta=0.05, trim_dist=0.20, max_iterations=30, min_inlier_ratio=0.75`） |
| `localizer/global_relocalize` | `std_srvs/Trigger` | 手动触发 B&B 全局重定位 |
| `localizer/nomotion_update` | `std_srvs/Empty` | 静止状态下强制执行一帧激光观测更新 |

```bash
# 真机启动（单机自启示例）：
ros2 run scan_localizer scan_localizer_node --ros-args \
  --params-file $(ros2 pkg prefix scan_localizer)/share/scan_localizer/config/localizer.yaml \
  -p autostart:=true

# 切换精修模式：
ros2 service call /localizer/set_fine_mode std_srvs/srv/SetBool "{data: true}"
# 手动触发全局重定位：
ros2 service call /localizer/global_relocalize std_srvs/srv/Trigger
# 静止强制更新：
ros2 service call /localizer/nomotion_update std_srvs/srv/Empty
# 热切换 YAML 地图：
ros2 topic pub --once /localizer/map_path std_msgs/msg/String "{data: '/maps/floor2.yaml'}"
```

### 运行基准测试与地图导出

```bash
# 完整 E1–E10 基准（与 docs/benchmark_results_x86.txt 对应）
ros2 run scan_localizer scan_localizer_benchmark
# 快速冒烟（样本量 1/5）
ros2 run scan_localizer scan_localizer_benchmark --quick
# 导出仿真地图（含 warehouse 与 corridor 全字段逐格 roundtrip 校验）
ros2 run scan_localizer sim_map_export /tmp/maps
```

---

## 三、工程结构与参数速查

### 目录结构

```
scan_localizer/
├── include/scan_localizer/
│   ├── se2.hpp               SE(2) 位姿代数、O(1) wrap_angle 与扫描数据结构
│   ├── map_field.hpp         占据栅格（左下角原点）→ EDT/SDF/似然场/滑窗最大值栈
│   ├── map_io.hpp            零外部依赖 YAML + P5/P2 PGM 地图读写
│   ├── matchers.hpp          correlative / field_gn / icp_p2p / ndt / composed + B&B 全局
│   ├── particle_filter.hpp   轻量粒子滤波（对照组）
│   ├── localizer_core.hpp    4D 各向异性 EKF + 四门限状态机 + 多假设队列 + 绑架检测
│   ├── sim_world.hpp         仿真世界（仓库/走廊地图、射线仿真、公共 104m 轨迹）
│   └── localizer_node.hpp    ROS 2 生命周期组件节点
├── src/
│   ├── map_field.cpp         map_io.cpp / matchers.cpp / particle_filter.cpp / localizer_core.cpp
│   ├── localizer_node.cpp    ROS 2 生命周期组件封装（TF2 外参、odom 插值、动态参数）
│   ├── sim_world.cpp         仿真世界实现（独立静态库，不混入核心库）
│   ├── localizer_sim.cpp     端到端仿真源节点（发布传感器/TF 并统计定位误差）
│   └── sim_map_export.cpp    仿真地图导出与逐格 roundtrip 校验工具
├── test/
│   ├── benchmark.cpp                 E1–E10 全量基准框架
│   ├── test_se2.cpp                  SE(2) 代数与 wrap_angle 单测
│   ├── test_map_io.cpp               YAML/PGM 读写与容错单测
│   ├── test_map_field.cpp            EDT/SDF/似然场/滑窗最大值栈单测
│   ├── test_matchers.cpp             五种匹配器与 B&B 全局搜索单测
│   ├── test_particle_filter.cpp      粒子滤波单测
│   ├── test_localizer_core.cpp       4D EKF、状态机、多假设、绑架检测与无 odom 单测
│   ├── test_localizer_node.cpp       生命周期节点、TF2 外参、热切图与服务集成测试
│   ├── test_sim_world.cpp            仿真射线、DynBox 遮挡与走廊防死循环单测
│   └── test_benchmark_regression.cpp E1/E4/E5/E8 轻量确定性回归断言
├── config/localizer.yaml     全量生产参数（与 LocalizerNode 声明 100% 一一对应）
├── launch/demo.launch.py     组合容器 / 独立进程端到端演示启动文件
└── docs/
    ├── benchmark_results_x86.txt        E1–E10 实测基准输出
    ├── corridor_relocalization_notes.md 长走廊退化、射线可见性约束与对称重定位工程说明
    └── generalization_audit_and_roadmap.md 泛化架构独立审核意见与四阶段演进路线
```

### 核心参数与调参速查（详见 `config/localizer.yaml`）

| 参数名 | 默认值 | 说明 |
|---|---|---|
| `autostart` | `false` | 生产由 Nav2 `lifecycle_manager` 托管；单机演示可设 `true` |
| `matcher` | `"composed"` | 可选 `composed` / `correlative` / `field_gn` / `icp_p2p` / `ndt` |
| `likelihood_sigma` | `0.20` | 似然场平滑尺度 (m) |
| `scan_subsample` | `2` | 每 N 束取 1（360 束 CPU 减半，精度损失 <1mm） |
| `min_scan_points` | `10` | 有效激光点数下限，不足则丢帧并节流告警 |
| `laser_x` / `laser_y` / `laser_theta` | `0.0` | 无 TF2 3D 外参时的 2D 回退安装偏移 |
| `match_window_lin` / `match_window_ang` | `0.15` / `0.10` | 跟踪粗搜窗 (±m / ±rad)；地毯重打滑可放宽至 `0.25` |
| `huber_delta` / `trim_dist` | `0.10` / `0.50` | 鲁棒核转折与动态障碍残差截断 (m)；重度人流可收紧 `trim_dist` 至 `0.20` |
| `min_inlier_ratio` / `min_score` / `mahal_gate` / `max_cov_eig` | `0.60` / `0.35` / `9.0` / `1.0` | 四道安全门限，任一不过即纯预测外推 |
| `degraded_after` / `lost_after` | `5` / `60` | 连续拒绝帧数进 `DEGRADED` / `LOST` |
| `use_imu` / `imu_rot_var_per_rad` / `max_imu_dt` | `false` / `2.0e-5` / `0.5` | 启用 4D EKF 陀螺航向融合与在线零偏估计 |
| `odom_trans_var_per_m` / `odom_rot_var_per_rad` / `odom_rot_var_per_m` | `4.0e-4` / `1.0e-3` / `2.0e-3` | 轮式里程计过程噪声模型 |
| `odom_timeout` | `1.0` | `/odom` 超时阈值 (s)，超时后自动降级为无里程计预测 |
| `tf_broadcast` / `transform_tolerance` / `tf_publish_rate` / `tf_lookup_timeout` | `true` / `0.1` / `10.0` / `0.05` | TF 广播开关、未来时间戳裕度、保活频率与查询超时 |
| `map_yaml` / `map_keep_pose` | `""` / `true` | 启动直载 YAML 地图与切图后是否保留当前位姿 |

### 安全性设计与部署预算

1. **拒绝永远比接受坏匹配便宜**：四道门（得分、内点率、马氏距离 χ²(3)、最小特征值）任一不过即拒绝观测；连续拒绝 → `DEGRADED` → `LOST` 自动触发全局重定位，并配备滑动窗口主动绑架检测。
2. **协方差诚实**：1cm/0.2° 物理下限；退化方向由未阻尼 Hessian 如实膨胀（走廊沿轴 12.48 m² vs 横向 0.0001 m²），Joseph 形式协方差更新保证半正定对称。
3. **时间戳纪律**：`map→odom` TF 使用扫描时间戳 + `transform_tolerance` 展期并支持 `tf_publish_rate` 保活；odom 优先按扫描时间戳查 TF2，回退带时间戳环形缓冲插值。
4. **RK3399/3588 预算（由 x86 Release 数据折算）**：`scan_subsample: 2` 时 `composed` 在 x86 上约 **3.3 ms/帧**；在 RK3399 (A72) 上约 ~7–10 ms/帧（@10Hz 占单核 ~7–10%），在 RK3588 (A76) 上占单核 <4%；800×600 地图全套场内存占用约 35 MB。

---

## 四、已知边界

- 2D 激光在 5cm 栅格地图上的物理精度受限于地图分辨率与半胞元量化（中位 ~2cm）；`set_fine_mode` 通过收紧搜窗、鲁棒核截断并提高内点率门限至 0.75 来抑制外点干扰，不包含额外视觉/反光板传感器融合。
- 在完全重复对称环境（如无限长同规格货架阵列）中，单帧全局搜索可能出现多个同分假设——多假设队列会在机器人运动后自动验证并晋升唯一正确假设，极端对称场景建议配合 `/localizer/global_hypotheses` 外部先验。
- 无 `/odom` 模式下使用常值位置预测（配合可选 IMU 航向），要求机器人在扫描周期内的位移落在 `match_window_lin` 范围内。
- 面向长走廊退化、半建图边界反朝向翻转与实车遮挡场景的算法改进与阈值权衡分析，详见 [docs/corridor_relocalization_notes.md](docs/corridor_relocalization_notes.md)。关于泛化架构的独立交叉审核意见与四阶段演进路线，详见 [docs/generalization_audit_and_roadmap.md](docs/generalization_audit_and_roadmap.md)。

## 许可

Apache-2.0（与 `package.xml` 及全仓源码头一致）。算法参考并致谢：Olson 2009（correlative）、Cartographer（B&B / PrecomputationGrid 思路）、LaMa（连续似然场定位）、AMCL（粒子滤波基线）。
