# scan_localizer —— 生产级 2D 激光已知地图定位

一套**实验驱动设计**、可直接产品化的室内移动机器人定位方案。针对你的需求：
2D 激光雷达 + 嵌入式主控（RK3399/RK3588）+ 与业务/导航代码共核算力预算 +
高精度（巡航 cm 级、对接模式 mm 级）。

- **算法核心零 ROS 依赖**（纯 C++20 + Eigen），可独立单测、独立交叉编译
- **五种匹配器 + 粒子滤波**全部实现并在统一基准下对比（E1–E9，见下）
- ROS 2 Jazzy 生命周期组件节点，**即插即用**：任意发布 `/scan` `/odom`
  `/map` 的设备直接接入
- 端到端仿真集成测试：冷启动全局定位 → 106m 环路跟踪（含人群遮挡段）

```
┌────────────────────────── LocalizerNode (lifecycle 组件) ──────────────────────┐
│  /map ──► MapField 预计算（EDT→SDF→似然场→滑窗栈）                              │
│  /odom ─► 预测（SE2 复合 + 滑移协方差）                                         │
│  /scan ─► ┌──────────── 两级匹配 ────────────┐  三门限：内点率/马氏/最小特征值   │
│           │ correlative 捕获 → field-GN 精修 │  拒绝 → odom 外推，绝不收坏匹配  │
│           └────────────────────────────────┘                                      │
│  状态机 TRACKING ⇄ DEGRADED → LOST → B&B 全局搜索（多假设，后续帧确认）           │
│  输出 TF map→odom（时间戳精确对齐）/ pose_with_cov / diagnostics                   │
│  服务 /localizer/set_fine_mode（对接精修） /localizer/global_relocalize          │
└──────────────────────────────────────────────────────────────────────────────────┘
```

---

## 一、实验结论：哪个组件用哪种算法（本项目最重要的产出）

所有数据来自 `docs/benchmark_results_x86.txt`（`ros2 run scan_localizer
scan_localizer_benchmark` 可复现）。仓库仿真 40×30m（外墙/隔间/货架阵列/柱阵），
720 束 12m 雷达，5cm 地图。**所有匹配器共用同一张 SDF 似然场地图**，变量唯一。

### E1 跟踪精度（先验误差 ~2cm/0.5°）

| 匹配器 | 位置中位误差 | p90 | 成功率 | 耗时 |
|---|---|---|---|---|
| correlative | 38mm | 80mm | 64% | 4.7ms |
| **field_gn (SDF 场 GN)** | **0.0mm** | 36mm | **100%** | **0.04ms** |
| icp_p2p | 76mm | 164mm | 35% | 0.23ms |
| ndt | 35mm | 40mm | 94% | 0.12ms |
| **composed** | **0.0mm** | 49mm | **99%** | 4.8ms |

**结论 1：精修级用 SDF 连续场高斯-牛顿（点-线 ICP 的无数据关联等价）。**
残差 = SDF 双线性插值，雅可比 = SDF 解析梯度，精度达地图分辨率极限
（仿真里 <0.1mm），比经典 ICP 快 5 倍、准 20 倍——因为省掉了最近邻搜索，
且墙体表面被建模为连续零水平集而非离散点集。经典 point-to-point ICP
受离散对应量化拖累，**不建议用于 2D 栅格地图**。NDT 的体素高斯平滑
限制了精度上限（~3.5cm ≈ 体素分辨率），但其概率得分天然抗差，
**动态障碍场景下的稳健性第二名（98% 成功率）**，适合作为备选精修器。

### E2 捕获范围（大初始误差下的成功率）

| 初始误差 | 5cm | 15cm | 40cm | 80cm | 1.5m |
|---|---|---|---|---|---|
| correlative(±15cm 窗) | 56% | 4% | 0% | 0% | 0% |
| field_gn 单独 | 100% | 100% | 78% | 46% | 10% |
| **composed(±15cm 窗)** | 100% | 100% | 100% | 76% | 14% |
| **composed(±1.6m 宽窗)** | 100% | 100% | 94% | 98% | 78% |

**结论 2：捕获级用 correlative（Olson 穷举），且必须与精修级组合。**
单独 correlative 被 5cm 栅格量化封顶（38mm 误差）；单独 field_gn 在
>40cm 时落入错误盆地。两级组合在窗口内**兼具大捕获范围与亚毫米精度**，
代价 ~5ms。odom 正常时窗口 ±15cm 足够（100% → 40cm 都能拉回）；
轮子严重打滑场景用宽窗（±1.6m，~180ms 突发，可降频触发）。

### E3/E4 鲁棒性

| 场景 | field_gn | ndt | composed |
|---|---|---|---|
| 测距噪声 σ=1/3/5cm | 1/9/17mm | 35/27/28mm | 1/11/20mm |
| 动态障碍（机器人旁 0.6m 箱×2） | **100% 成功 / 0mm** | 98% / 35mm | 96% / 0mm |

**结论 3：Huber + 残差截断（3σ）足以应对动态障碍**，无需显式聚类剔除。
SDF 场精修天然把"不在地图上的点"当作大残差降权，人群遮挡段实测
得分降到 0.78 仍稳定跟踪（见端到端演示）。trim_dist 从 0.5m 收到
0.2m（tight 模式）在常规动态场景无增益——**预留作重度遮挡工况**。

### E5 走廊退化（30×3m，两端墙超出量程）

| 匹配器 | 沿轴误差 | 横向误差 | 沿轴方差 |
|---|---|---|---|
| field_gn | 保持先验不动* | 0.0mm | **200 m²（如实报告不可观测）** |
| correlative | 41mm | 22mm | 响应面二阶矩膨胀 |

*field_gn 在不可观测方向既不幻想修正也不发散，把先验留在原地。

**结论 4：协方差必须来自未阻尼 Hessian。** LM 阻尼用于解步防发散，
协方差用裸 H 才能如实暴露退化方向（沿轴 200 m² → 融合层自动改信
里程计）。门控用**最小特征值**而非迹：走廊里横向/航向仍有效，
不应整帧丢弃。

### E8 全局重定位（冷启动，30 次）

| 方案 | 成功率 | 耗时 |
|---|---|---|
| **B&B correlative（滑窗栈）** | **80%** | **7ms** |
| 粒子滤波（20000 粒子 × 20 步运动） | 3% | 570ms |

**结论 5：全局定位用分支定界 correlative，不用粒子滤波。**
PF 全局要 O(地图面积/精度²) 个粒子才能覆盖假设空间（1200m² 地图上
2 万粒子都不够），且需运动才能收敛；B&B 是**分辨率完备**的确定性搜索，
8ms 出结果。两个工程要点（都是踩过坑的）：
1. 上界必须用**滑动窗口最大值栈**（Carto PrecomputationGrid 同思路），
   块分区 max-pool 会让"逐点独立松弛"混入上界，搜索直接失效（实测
   1.1 亿次展开找错答案）；
2. 重复结构环境（货架阵列）保留 **top-K 假设**交给下一帧确认。

### E9 轨迹级架构对比（106m 环路 × 2% odom 滑移 × 人群遮挡段）

| 架构 | 平均误差 | 最大误差 | 备注 |
|---|---|---|---|
| **A: 状态机 + composed 跟踪 + B&B 恢复** | **18mm** | **60mm** | 98% 帧采纳 |
| B: 粒子滤波（2000 粒子，公平调参） | 79mm | 237mm | 8 倍算力 |

**总结论（决策表）**：

| 组件 | 推荐 | 依据 |
|---|---|---|
| 精修匹配 | **SDF 场 GN**（点-线等价） | E1/E3/E5 全维最优 |
| 捕获匹配 | **correlative 两级** | E2 窗口内无局部极小 |
| 跟踪架构 | **状态机 + 三门限**，不用 PF | E9：精度 4 倍、算力 1/8 |
| 全局重定位 | **滑窗栈 B&B + top-K 假设** | E8：确定性、7ms |
| 动态抑制 | Huber + 3σ trim（在线） | E4 够用，聚类剔除留作重工况 |
| 退化处理 | 未阻尼 Hessian 协方差 + 最小特征值门 | E5 如实暴露不可观测方向 |
| 对接精修 | `set_fine_mode` 服务热切换紧门限配置 | E1 精度可达亚毫米 |

---

## 二、使用

### 端到端演示（自带仿真，无任何硬件）

```bash
colcon build --packages-select scan_localizer
source install/setup.bash
ros2 launch scan_localizer demo.launch.py
# 另一终端观察：
ros2 topic echo /localizer/status          # TRACKING / score / inlier
ros2 run tf2_ros tf2_echo map odom         # 定位输出 TF
```

冷启动第一帧即自动 B&B 全局定位（日志 `global search: (3.25, 13.50,
0.0 deg) score=0.969`），之后全程 `TRACKING`；人群中 score 降至 0.78
仍稳定，出人群恢复 1.0。

### 接入真实机器人（即插即用面）

节点消费 ROS 2 标准消息，任何设备直接替换：

| 输入 | 消息 | 典型来源 |
|---|---|---|
| `/map` | nav_msgs/OccupancyGrid（latched） | slam_toolbox / map_server 建好的图 |
| `/scan` | sensor_msgs/LaserScan | RPLIDAR/思岚/镭神/北醒等任意 2D 雷达驱动 |
| `/odom` | nav_msgs/Odometry（可选） | 底盘驱动 / robot_localization |
| `/initialpose` | PoseWithCovarianceStamped | RViz 2D Pose Estimate |

```bash
# 真机（已建图）：
ros2 run scan_localizer scan_localizer_node --ros-args \
  --params-file $(ros2 pkg prefix scan_localizer)/share/scan_localizer/config/localizer.yaml
# 对接模式（充电桩/货架 1.5m 内切换）：
ros2 service call /localizer/set_fine_mode std_srvs/srv/SetBool "{data: true}"
# 丢失手动重定位：
ros2 service call /localizer/global_relocalize std_srvs/srv/Trigger
```

### 匹配器切换（配置驱动）

```yaml
matcher: "composed"   # composed | correlative | field_gn | icp_p2p | ndt
```

### 运行基准（复现实验）

```bash
ros2 run scan_localizer scan_localizer_benchmark          # 完整 E1-E9
ros2 run scan_localizer scan_localizer_benchmark --quick  # 快速冒烟
```

---

## 三、工程设计

### 结构

```
include/scan_localizer/
  se2.hpp             SE(2) 代数与扫描结构（全库底座）
  map_field.hpp       占据栅格 → SDF/似然场/滑窗最大值栈（一次预计算喂所有组件）
  matchers.hpp        匹配器接口 + correlative/field-GN/ICP-P2P/NDT/composed + B&B 全局
  particle_filter.hpp AMCL-lite（对照组 & 备选全局方案）
  localizer_core.hpp  状态机 + 三门限（ROS 无关，可独立单测）
  sim_world.hpp       仿真世界（受控实验数据源，也是 demo 的世界引擎）
  localizer_node.hpp ROS 2 生命周期组件节点
src/                  对应实现（核心 5 个文件，共 ~1800 行，零外部依赖除 Eigen）
test/benchmark.cpp    E1-E9 实验框架（结果在 docs/benchmark_results_x86.txt）
launch/demo.launch.py 组合容器端到端演示
config/localizer.yaml 生产参数（带注释与调参指引）
```

### 安全性设计原则

1. **拒绝永远比接受坏匹配便宜**：三道门（内点率 ≥0.60、马氏 ≤χ²(3)3σ、
   最小特征值 ≤1.0m²）任一不过即纯 odom 外推；连续 5 帧拒绝 → DEGRADED；
   持续 → LOST → 自动全局搜索。E9 实测 98% 帧采纳、恢复闭环全自动。
2. **协方差诚实**：1cm/0.2° 物理下限（禁止宣称亚毫米精度）；退化方向
   如实膨胀（走廊沿轴 200 m²），融合层据此自动改信里程计。
3. **数值防御**：EDT 用 int64 防平方溢出；包络哨兵与距离上界分离；
   SDF 查询带非有限输入护栏；GN 带步长护栏（≤0.2m/0.1rad）与
   残差最优回退。
4. **时间戳纪律**：TF 用扫描时间戳 + tolerance 展期，杜绝外推抖动；
   odom 增量按"上一帧扫描时刻 → 本帧扫描时刻"插值对齐。

### RK3399/3588 部署预算（x86 数据折算，A72 约为实验机单核的 1/2）

| 项 | RK3399 (A72) | RK3588 (A76) |
|---|---|---|
| MapField 构建（一次性） | ~50ms | ~15ms |
| composed 跟踪 @10Hz | ~5ms/帧 ≈ 5% 单核 | ~2% |
| 全局搜索（罕见事件） | ~15ms | ~5ms |
| 内存（800×600 地图全套场） | ~35MB | ~35MB |

建议：`scan_subsample: 2`（精度损失 <1mm，CPU 减半）；控制节点绑小核
（A53），本定位跑大核；组合容器部署（demo 已示范）。

### 调参速查

| 症状 | 动作 |
|---|---|
| 地毯打滑频繁 DEGRADED | `match_window_lin` 0.15→0.25，`odom_trans_var_per_m` ×2 |
| 玻璃幕墙假内点 | 建图时遮蔽该区域（map 编辑），或 `min_inlier_ratio` 0.60→0.70 |
| 重度人流（商场） | `trim_dist` 0.5→0.2（E4 的 tight 配置） |
| 冷启动偶发锁错对称区 | 确认 top-K 后续帧确认在跑（日志 alternates 数） |
| 地图老旧匹配持续拒绝 | 重新建图（slam_toolbox），MapField 构建仅 ~50ms |

---

## 四、进阶能力（IMU 融合 / 地图管理 / 视觉重定位入口）

### IMU 陀螺航向融合 —— 收益实测（E10）

节点订阅 `/imu`（sensor_msgs/Imu，参数 `use_imu`），陀螺积分 Δyaw
替换轮式 Δθ（平移仍取轮式，帧间差分自动消除慢漂移）。

| 工况（106m 环路 + 80 帧人群遮挡） | 平均误差 | 最大误差 | 采纳率 |
|---|---|---|---|
| 常规打滑 3%，仅轮式 | 1.27 m | 6.00 m | 35.6% |
| **常规打滑 3%，+IMU** | **0.018 m** | **0.086 m** | **98.1%** |
| 地毯重打滑 10%，仅轮式 | 5.08 m（失效） | 12.01 m | 10.6% |
| **地毯重打滑 10%，+IMU** | **0.111 m** | **0.360 m** | 64.4% |

结论：IMU **不提升稳态精度**（激光扫描匹配决定精度），提升的是**鲁棒性
余量**——把系统可承受的轮式旋转误差从 ~1-2% 放大到 >10%，转弯遮挡段
不丢帧。ROS 端严格对拍（60 帧含人群段）：均值 16mm / 中位 9mm / 最大
42mm，59/60 TRACKING。**强烈建议接入**（成本一个话题订阅）。

### 全局自动重定位（已内置，无需外部依赖）

- **自动**：连续拒绝 → DEGRADED → LOST → 自动触发 B&B 全局搜索
  （滑窗栈 + top-K 假设队列；top-1 若为对称结构鬼影，跟踪失败后
  下一轮自动消费备选假设，激光验证择优——无需重跑搜索）
- **手动**：`ros2 service call /localizer/global_relocalize std_srvs/srv/Trigger`
  （立即进入全局重定位）；或 RViz `2D Pose Estimate`（`/initialpose`）

### 地图管理（业务层 API）

| 操作 | 方式 |
|---|---|
| **加载**（免 map_server） | 启动参数 `map_yaml:=/maps/floor1.yaml` 直接装载（nav2 兼容 YAML+PGM，自动半胞元约定补偿，roundtrip 精确） |
| **加载**（标准链路） | 订阅 `/map`（latched，slam_toolbox / nav2_map_server 发布） |
| **热切换** | `ros2 topic pub /localizer/map_path std_msgs/msg/String "data: /maps/floor2.yaml"` —— 重建 MapField ~50ms；`map_keep_pose:=true` 同坐标系保位姿续跟（实测切换后 TRACKING 不中断），`:=false` 自动全局重定位 |
| **多楼层** | 楼层即多张 yaml；电梯到达 → 切图 + `map_keep_pose:=false` → 自动重定位 |
| **导出/建图资产** | `ros2 run scan_localizer sim_map_export <dir>`（演示数据）；slam_toolbox 建的图直接可用 |

### 视觉重定位入口（已预留并验证）

外部视觉（也可用于二维码/UWB/反光板）节点把**候选位姿云**发到：

```
/localizer/global_hypotheses  (geometry_msgs/PoseArray, map 系)
```

定位节点对每个假设做小窗激光匹配**强验证**（≥0.85 分才采纳）：
- 端到端实测：含假目标的假设云，真假设 VERIFIED（0.969），假目标落选
- 过期/错误假设（含航向错的"真值"）被拒 → 自动回退 B&B，
  **视觉假阳性进不了跟踪环**（实测错误航向假设被正确拒绝后 B&B 恢复）
- 适用链路：NetVLAD/词袋检索 → 地图位姿假设 → 本节点激光确认；
  RK3588 NPU 跑检索模型，激光验证在 CPU 只需 ~5ms/假设

## 五、已知边界与路线图

- 单帧激光物理极限 ~±2-3cm（点噪声平均后 <1cm）；对接 <5mm 需叠加
  反光板/AprilTag，`set_fine_mode` 已预留融合入口
- 全局搜索在完全对称环境（同规格货架无限阵列且无其他特征）理论上多假设
  并列——top-K + 运动确认是当前解，极限场景建议部署时加一个地标
- 激光安装偏移当前为恒等（`scanToSensorFrame` 预留），产品化接 TF 树
  base_link→laser 即可
- 路线图：多地图版本管理服务化（当前 map_path 话题已支持热切换）、
  逐帧动态点聚类剔除（重工况备选）、视觉检索模型示例节点

## 许可

BSD-3-Clause。实验与实现参考并致谢：Olson 2009（correlative）、
Cartographer（B&B/PrecomputationGrid 思路）、LaMa（连续似然场定位）、
AMCL（粒子滤波基线）。
