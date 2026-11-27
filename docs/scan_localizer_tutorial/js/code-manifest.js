// Complete manifest of all 36 files in src/scan_localizer (synced after Step 1-4)
// Consumed by code-index.html, app.js (Cmd+K global symbol search), and source-reference verification.
(function (global) {
  "use strict";

  const CHAPTERS = [
    { id: "ch01", num: "01", title: "从零实现 SE(2) 与 TF 树", sub: "位姿复合/求逆、数值演算、源码步进与练习", file: "chapters/ch01.html" },
    { id: "ch02", num: "02", title: "从零写出 MapField 与精确 EDT", sub: "最小 edt1d、栈状态手算、源码步进与练习", file: "chapters/ch02.html" },
    { id: "ch03", num: "03", title: "从零实现 Correlative 与经典匹配器", sub: "窗口评分、旋转复用、局部极小与源码步进", file: "chapters/ch03.html" },
    { id: "ch04", num: "04", title: "从零写出 Field-GN 与两级组合", sub: "最小实现、单步手算、源码步进与填空验收", file: "chapters/ch04.html" },
    { id: "ch05", num: "05", title: "从零实现退化安全的协方差", sub: "特征值夹紧、未阻尼 Hessian 与实时各向异性", file: "chapters/ch05.html" },
    { id: "ch06", num: "06", title: "从零实现 B&B 全局重定位", sub: "上界剪枝、Top-K NMS、粒子滤波对照与练习", file: "chapters/ch06.html" },
    { id: "ch07", num: "07", title: "从零实现 4D EKF 与五态状态机", sub: "预测/门控/Joseph 步进、实时 P4 与练习", file: "chapters/ch07.html" },
    { id: "ch08", num: "08", title: "从零实现 ROS 2 定位节点", sub: "Lifecycle、锁外构场、时间对齐与快照发布", file: "chapters/ch08.html" },
    { id: "ch09", num: "09", title: "从零设计不作弊的基准实验", sub: "配对试验、固定种子、指标复算与 E1–E10", file: "chapters/ch09.html" },
    { id: "ch10", num: "10", title: "C++20 × Eigen × ROS 2 代码重建", sub: "语法卡片、关键行填空、源码步进与验收", file: "chapters/ch10.html" },
  ];

  // Exact file & symbol map across all 36 files of src/scan_localizer
  const FILES = [
    // Group 1: Geometry & Map Representation
    {
      path: "include/scan_localizer/se2.hpp",
      group: "geometry",
      lines: 154,
      chapter: "ch01",
      role: "公共几何底座：wrap_angle、SE(2) 群位姿 Pose2、点云与缓冲复用 transform_scan",
      symbols: [
        { name: "wrap_angle(double a)", range: "34-46", desc: "O(1) 安全角度归一化 (-π, π]，处理 NaN/±inf" },
        { name: "struct Pose2", range: "49-69", desc: "SE(2) 位姿复合 operator* 与群逆 inverse()" },
        { name: "transform_scan(pose, scan, out)", range: "80-114", desc: "C++20 concept + 缓冲复用点云坐标变换" },
      ],
    },
    {
      path: "include/scan_localizer/map_io.hpp",
      group: "geometry",
      lines: 46,
      chapter: "ch01",
      role: "Nav2 / map_server 兼容的 YAML + PGM 地图读写声明",
      symbols: [
        { name: "load_map_yaml / save_map_yaml", range: "37-44", desc: "统一 ROS 左下角原点与 PGM 第 0 行翻转约定" },
      ],
    },
    {
      path: "src/map_io.cpp",
      group: "geometry",
      lines: 484,
      chapter: "ch01",
      role: "PGM P5/P2 严格解析、扁平 YAML 解析与双向 Y 轴翻转实现",
      symbols: [
        { name: "read_pgm", range: "186-267", desc: "P2/P5 魔数、注释跳过与截断校验" },
        { name: "load_map_yaml", range: "271-398", desc: "相对路径解析、trinary/scale/raw 与 row_pgm = h - 1 - iy 翻转" },
        { name: "save_map_yaml", range: "400-482", desc: "OccupancyGrid 导出为 map_server 可用 PGM + YAML" },
      ],
    },
    {
      path: "include/scan_localizer/map_field.hpp",
      group: "geometry",
      lines: 228,
      chapter: "ch02",
      role: "OccupancyGrid 坐标换算与 MapField（EDT→SDF→似然场→swmax 金字塔）接口",
      symbols: [
        { name: "struct OccupancyGrid", range: "42-81", desc: "worldToGridIndex vs worldToGridF (格心 0.5 偏移)" },
        { name: "class MapField", range: "84-170", desc: "sdf()、likelihood()、descendToObstacle() 与 swmax_ 接口" },
      ],
    },
    {
      path: "src/map_field.cpp",
      group: "geometry",
      lines: 764,
      chapter: "ch02",
      role: "双向 Meijster/FH EDT、SDF 双线性梯度、似然场与 2^k 滑窗最大值栈构建",
      symbols: [
        { name: "floor_div / edt2d", range: "35-89", desc: "int64 防溢出精确 2D 平方欧氏距离变换" },
        { name: "edt1d", range: "93-138", desc: "O(N) 抛物线下包络算法实现" },
        { name: "MapField::build", range: "149-279", desc: "双向 EDT→SDF→高斯似然→swmax_ 滑窗金字塔" },
        { name: "MapField::sdf", range: "281-315", desc: "SDF 双线性插值与解析梯度 ∇SDF" },
        { name: "MapField::descendToObstacle", range: "317-417", desc: "沿 SDF 下降/上升查找最近障碍表面格" },
      ],
    },

    // Group 2: Matchers & Global Search
    {
      path: "include/scan_localizer/matchers.hpp",
      group: "matchers",
      lines: 195,
      chapter: "ch03",
      role: "五种局部匹配器接口、MatchOptions 参数与 global_correlative 全局搜索声明",
      symbols: [
        { name: "MatchResult / MatchOptions", range: "37-63", desc: "匹配结果位姿、3×3 协方差、score、inlier_ratio 与参数" },
        { name: "class Matcher 族", range: "65-136", desc: "Correlative / FieldGn / IcpP2p / Ndt / Composed 多态族" },
        { name: "global_correlative / make_matcher", range: "139-157", desc: "B&B 全局定位输出与配置驱动工厂" },
      ],
    },
    {
      path: "src/matchers.cpp",
      group: "matchers",
      lines: 1748,
      chapter: "ch04",
      role: "五种局部匹配器、未阻尼 Hessian 协方差、B&B 全局重定位与工厂实现",
      symbols: [
        { name: "eval_stats / cov_from_hessian", range: "46-98", desc: "平均似然统计与安全特征分解求逆协方差" },
        { name: "CorrelativeMatcher::match", range: "105-269", desc: "Olson 粗细两级滑窗搜索与响应面二阶矩协方差" },
        { name: "FieldGnMatcher::match", range: "274-390", desc: "SDF 连续场高斯-牛顿、Huber IRLS 与历史最优回滚" },
        { name: "IcpP2pMatcher::match", range: "395-499", desc: "点到点 ICP、旋转力臂雅可比与法向信息矩阵协方差" },
        { name: "NdtMatcher::build / match", range: "504-686", desc: "2D 体素高斯拟合与 3×3 邻域 D2P 优化" },
        { name: "ComposedMatcher::match", range: "691-705", desc: "Correlative 粗捕获 → FieldGn 精修与退化保护" },
        { name: "global_correlative", range: "716-1014", desc: "基于 swmax_ 的优先队列 B&B + Top-K NMS + 末端精修" },
        { name: "make_matcher", range: "1017-1039", desc: "按配置名创建匹配器实例" },
      ],
    },
    {
      path: "include/scan_localizer/particle_filter.hpp",
      group: "matchers",
      lines: 87,
      chapter: "ch06",
      role: "AMCL-lite 粒子滤波（作为架构对照组与全局定位候选对比）声明",
      symbols: [
        { name: "PfOptions / ParticleFilter", range: "29-85", desc: "Thrun 运动噪声、似然观测与 ESS 重采样参数" },
      ],
    },
    {
      path: "src/particle_filter.cpp",
      group: "matchers",
      lines: 350,
      chapter: "ch06",
      role: "Thrun 倒车安全运动模型、log-sum-exp 混合似然更新与系统重采样",
      symbols: [
        { name: "sample_motion / predict", range: "100-148", desc: "支持倒车反向 rot1 分解，防 ±π 噪声爆炸" },
        { name: "ParticleFilter::update", range: "150-254", desc: "log-sum-exp 防下溢、有效光束缩放与 ESS 重采样" },
        { name: "ParticleFilter::estimate / resample", range: "256-348", desc: "sin/cos 圆均值位姿与系统低方差重采样" },
      ],
    },

    // Group 3: Filter & State Machine
    {
      path: "include/scan_localizer/localizer_core.hpp",
      group: "core",
      lines: 240,
      chapter: "ch07",
      role: "ROS 无关核心状态机：4 状态各向异性 EKF [x,y,θ,b_g]^T、门控与多假设队列",
      symbols: [
        { name: "enum class LocState / CoreConfig", range: "43-88", desc: "五态机、四门控、4D EKF 零偏及远跳保护配置" },
        { name: "class LocalizerCore", range: "100-185", desc: "on_odom / on_imu_* / on_scan / request_global 接口" },
      ],
    },
    {
      path: "src/localizer_core.cpp",
      group: "core",
      lines: 1483,
      chapter: "ch07",
      role: "4D SE(2)+Bias EKF、Joseph 协方差、四道门控、静止锁死松弛与跳变保护实现",
      symbols: [
        { name: "is_valid_spd_cov3 / compute_inlier_ratio", range: "36-72", desc: "协方差正定校验与扫描内点率计算" },
        { name: "set_initial_pose / request_global", range: "105-160", desc: "人工初始化重置与显式全局重定位标志" },
        { name: "on_imu_yaw / on_imu_gyro / map_to_odom", range: "216-275", desc: "IMU 首帧双锚定与扫描时刻锚定 TF" },
        { name: "compute_motion_delta / try_promote_hypotheses", range: "277-374", desc: "减零偏后的 IMU Δyaw 与多假设晋升验证" },
        { name: "LocalizerCore::on_scan", range: "398-758", desc: "完整主循环：4D F4/Q4 预测、4 门控、Joseph 更新与异常恢复" },
      ],
    },

    // Group 4: ROS 2 Lifecycle Node & Config
    {
      path: "include/scan_localizer/localizer_node.hpp",
      group: "ros",
      lines: 260,
      chapter: "ch08",
      role: "ROS 2 Jazzy Lifecycle 组件节点接口、回调组、快照与话题服务声明",
      symbols: [
        { name: "struct OutputSnapshot", range: "87-100", desc: "锁外发布所需的状态快照（TF/位姿/诊断/假设）" },
        { name: "class LocalizerNode", range: "69-250", desc: "5 个生命周期回调、参数校验与 TF2 成员" },
      ],
    },
    {
      path: "src/localizer_node.cpp",
      group: "ros",
      lines: 1805,
      chapter: "ch08",
      role: "生命周期治理、锁外构场热切图、TF2 3D 外参、Odom 插值、热调参与 6×6 协方差发布",
      symbols: [
        { name: "LocalizerNode 构造 / readAndValidateParameters", range: "62-365", desc: "52 项参数集中声明、校验与回调组划分" },
        { name: "onSetParameters", range: "367-642", desc: "运行时动态调参校验与原子提交" },
        { name: "on_configure / activate / deactivate / cleanup", range: "695-888", desc: "完整 Lifecycle 状态切换与 resetResources 幂等清理" },
        { name: "applyMap", range: "890-963", desc: "锁外构建 MapField+Matcher，加锁瞬间原子交换" },
        { name: "lookupOdomAt", range: "1113-1187", desc: "优先 TF2 扫描时刻插值，回退环形缓冲线性插值" },
        { name: "onScan", range: "1189-1336", desc: "TF2 3D 外参转 base_frame 点云、核心步进与防竞争快照" },
        { name: "publishOutputs", range: "1555-1667", desc: "map→odom TF、{0,1,5} 6×6 协方差与诊断状态广播" },
      ],
    },
    {
      path: "config/localizer.yaml",
      group: "ros",
      lines: 80,
      chapter: "ch08",
      role: "全部 52 项节点生产参数默认值（含 autostart: false 对齐 Nav2）",
      symbols: [
        { name: "autostart / matcher / gates / TF params", range: "1-80", desc: "与 LocalizerNode::readAndValidateParameters 一一对应" },
      ],
    },
    {
      path: "launch/demo.launch.py",
      group: "ros",
      lines: 103,
      chapter: "ch08",
      role: "支持 use_composition True/False 分支与仿真闭环启动的 Launch 脚本",
      symbols: [
        { name: "generate_launch_description", range: "32-103", desc: "ComposableNodeContainer intra-process vs standalone" },
      ],
    },
    {
      path: "CMakeLists.txt",
      group: "ros",
      lines: 223,
      chapter: "ch08",
      role: "core / sim_lib / node_component / executable / 9 组 gtest 与严格编译告警配置",
      symbols: [
        { name: "scan_localizer_core / scan_localizer_sim_lib", range: "1-120", desc: "生产核与仿真工具解耦与 -Wall -Wextra -Wpedantic" },
        { name: "BUILD_TESTING gtest + ament_lint", range: "121-208", desc: "9 个 gtest 套件与 lint 目标注册" },
      ],
    },
    {
      path: "package.xml",
      group: "ros",
      lines: 40,
      chapter: "ch08",
      role: "ROS 2 Jazzy 包依赖与 Nav2/TF2/Lifecycle 声明",
      symbols: [
        { name: "package dependencies", range: "1-40", desc: "rclcpp_lifecycle, tf2_ros, eigen, ament_cmake_gtest" },
      ],
    },
    {
      path: "README.md",
      group: "ros",
      lines: 300,
      chapter: "ch09",
      role: "scan_localizer 实验驱动设计总结与 E1–E10 选型结论手册",
      symbols: [
        { name: "E1–E10 实验与架构总结", range: "1-297", desc: "基准复现命令、匹配器决策依据与 52 项参数一览" },
      ],
    },
    {
      path: "docs/benchmark_results_x86.txt",
      group: "ros",
      lines: 89,
      chapter: "ch09",
      role: "Phase 5 重建后的 x86 Release 基准完整原始实测报告",
      symbols: [
        { name: "E1–E10 Release 实测表", range: "1-89", desc: "Field-GN 0.054ms、走廊 12.482m² 各向异性与 E10 10% 滑移" },
      ],
    },

    // Group 5: Simulator & Test Suites (14 files)
    {
      path: "include/scan_localizer/sim_world.hpp",
      group: "sim_test",
      lines: 96,
      chapter: "ch09",
      role: "受控仿真环境：仓库/长走廊生成、射线求交与精确角度激光模拟声明",
      symbols: [
        { name: "make_warehouse / make_corridor", range: "30-34", desc: "40×30m 仓库与 30×3m 严格退化走廊生成器" },
        { name: "raycast_range / make_scan / make_laser_ranges", range: "49-67", desc: "动态箱遮挡射线与无重分箱偏差测距生成" },
      ],
    },
    {
      path: "src/sim_world.cpp",
      group: "sim_test",
      lines: 525,
      chapter: "ch09",
      role: "DDA/步进光线求交、动态障碍注入、自由空间随机采样与 104m 轨迹生成",
      symbols: [
        { name: "make_warehouse / mark_exterior_unknown", range: "40-109", desc: "仓库布局构建与墙外未知区标记" },
        { name: "raycast_range / make_laser_ranges", range: "129-221", desc: "噪声 σ=0 零标准差防护与精确光束角模拟" },
        { name: "random_free_poses / make_warehouse_trajectory", range: "223-311", desc: "考虑 origin 的安全采样与 104m 双圈轨迹" },
      ],
    },
    {
      path: "src/localizer_sim.cpp",
      group: "sim_test",
      lines: 305,
      chapter: "ch08",
      role: "ROS 2 仿真数据源节点：发布 /map /scan /odom /imu 与真值滑移对比日志",
      symbols: [
        { name: "class LocalizerSim", range: "41-297", desc: "闭环轨迹驱动、人群遮挡注入与 odom 滑移模拟" },
      ],
    },
    {
      path: "src/sim_map_export.cpp",
      group: "sim_test",
      lines: 115,
      chapter: "ch01",
      role: "仓库仿真地图导出为 map_server PGM/YAML 并执行往返校验的 CLI 工具",
      symbols: [
        { name: "verify_roundtrip / main", range: "31-113", desc: "save + reload 栅格逐胞元无损验证" },
      ],
    },
    {
      path: "test/benchmark.cpp",
      group: "sim_test",
      lines: 975,
      chapter: "ch09",
      role: "E1–E10 全自动实验框架（精度/捕获/噪声/动态/走廊/密度/耗时/全局/轨线/IMU）",
      symbols: [
        { name: "run_e9_trajectory / run_e10_imu", range: "193-469", desc: "104m 闭环轨迹与 10% 轮式滑移 IMU 零偏消融" },
        { name: "E1–E8 main benchmark", range: "470-975", desc: "配对随机试验与受控消融评测逻辑" },
      ],
    },
    {
      path: "test/test_se2.cpp",
      group: "sim_test",
      lines: 228,
      chapter: "ch01",
      role: "SE(2) 群运算、wrap_angle NaN/inf 防护与 transform_scan 单元测试",
      symbols: [
        { name: "SE2 Test Cases", range: "1-228", desc: "群逆恒等式、O(1) 边界角与模板迭代器验证" },
      ],
    },
    {
      path: "test/test_map_io.cpp",
      group: "sim_test",
      lines: 409,
      chapter: "ch01",
      role: "PGM P2/P5、Y 轴翻转、CRLF、格式错误拒绝与 YAML 解析单测 (6 cases)",
      symbols: [
        { name: "MapIO Test Cases", range: "80-409", desc: "往返一致、截断报文与非法 origin 安全拒绝" },
      ],
    },
    {
      path: "test/test_map_field.cpp",
      group: "sim_test",
      lines: 348,
      chapter: "ch02",
      role: "edt1d 负截断边界、未知区 SDF、梯度精度与 swmax 金字塔单测 (10 cases)",
      symbols: [
        { name: "MapField Test Cases", range: "1-348", desc: "{9,100,4}->8、双向 EDT 与退化图安全防护" },
      ],
    },
    {
      path: "test/test_matchers.cpp",
      group: "sim_test",
      lines: 672,
      chapter: "ch04",
      role: "五种匹配器收敛、退化走廊协方差、B&B 100%越界与失败回调单测 (8 cases)",
      symbols: [
        { name: "Matcher Test Cases", range: "60-482", desc: "各向异性特征值比与多假设 NMS 回归验证" },
      ],
    },
    {
      path: "test/test_particle_filter.cpp",
      group: "sim_test",
      lines: 203,
      chapter: "ch06",
      role: "ParticleFilter 倒车分解、log-sum-exp 与 ESS 重采样单测 (5 cases)",
      symbols: [
        { name: "ParticleFilter Test Cases", range: "1-203", desc: "±π 航向稳定与极端权重防 NaN 验证" },
      ],
    },
    {
      path: "test/test_localizer_core.cpp",
      group: "sim_test",
      lines: 1294,
      chapter: "ch07",
      role: "4D EKF 零偏收敛、门控、走廊滑移、静止松弛与对称走廊防误跳单测 (14 cases)",
      symbols: [
        { name: "4D EKF / Degeneracy / Jump Guard Tests", range: "120-1073", desc: "包含 98fcd28 修复的两项回归单测" },
      ],
    },
    {
      path: "test/test_localizer_node.cpp",
      group: "sim_test",
      lines: 866,
      chapter: "ch08",
      role: "ROS 2 Lifecycle 状态迁移、TF2 倒装雷达外参、6×6 协方差与热调参集成测试",
      symbols: [
        { name: "LocalizerNode Integration Tests", range: "181-866", desc: "6 组真实 DDS 节点端到端测试" },
      ],
    },
    {
      path: "test/test_sim_world.cpp",
      group: "sim_test",
      lines: 163,
      chapter: "ch09",
      role: "仿真地图、动态遮挡射线与无死循环随机采样单测 (5 cases)",
      symbols: [
        { name: "SimWorld Test Cases", range: "1-163", desc: "noise_std=0 安全与全墙图有限退出测试" },
      ],
    },
    {
      path: "test/test_benchmark_regression.cpp",
      group: "sim_test",
      lines: 202,
      chapter: "ch09",
      role: "核心指标自动化回归门禁（精修误差、走廊协方差、104m 环路与 IMU 消融）",
      symbols: [
        { name: "Benchmark Regression Gates", range: "40-202", desc: "4 项核心指标 CI 守护测试" },
      ],
    },
    {
      path: "test/test_generalization_matrix.cpp",
      group: "sim_test",
      lines: 649,
      chapter: "ch07",
      role: "S1–S12 跨场景泛化回归矩阵：同一套默认参数覆盖走廊、对称、动态人群、倾斜走廊与地图先验",
      symbols: [
        { name: "Generalization Matrix S1-S12", range: "1-649", desc: "不针对单一场景调参的端到端回归" },
      ],
    },
    {
      path: "docs/generalization_audit_and_roadmap.md",
      group: "ros",
      lines: 59,
      chapter: "ch07",
      role: "泛化架构独立审核意见与 Step1–4 演进路线",
      symbols: [
        { name: "Audit & Roadmap", range: "1-59", desc: "审核结论、四阶段路线与完成状态" },
      ],
    },
    {
      path: "docs/corridor_relocalization_notes.md",
      group: "ros",
      lines: 58,
      chapter: "ch07",
      role: "走廊重定位与退化处理笔记",
      symbols: [
        { name: "Corridor Notes", range: "1-58", desc: "走廊重定位问题与对策" },
      ],
    },
    {
      path: "include/scan_localizer/blackbox_recorder.hpp",
      group: "core",
      lines: 166,
      chapter: "ch07",
      role: "低内存环形缓冲黑匣子接口：固定尺寸扫描/里程计快照、紧凑二进制读写与触发状态机",
      symbols: [
        { name: "struct PackedScanRecord / BlackboxBundle", range: "60-90", desc: "固定 512 点零堆分配扫描快照与回放包结构" },
        { name: "class BlackboxRecorder", range: "97-146", desc: "前后窗口环形录制、异步写盘与磁盘配额轮转" },
      ],
    },
    {
      path: "src/blackbox_recorder.cpp",
      group: "core",
      lines: 559,
      chapter: "ch07",
      role: "黑匣子环形缓冲、SLOCBB01 二进制读写、JSON 摘要与后台配额轮转实现",
      symbols: [
        { name: "save_blackbox_bundle / load_blackbox_bundle", range: "99-236", desc: "带魔数与边界校验的紧凑二进制事故包序列化" },
        { name: "BlackboxRecorder::record_scan", range: "281-323", desc: "O(1) 覆盖写入并监测 GLOBAL/LOST/kWallPhys 自动触发" },
      ],
    },
    {
      path: "test/test_blackbox_recorder.cpp",
      group: "tests",
      lines: 291,
      chapter: "ch09",
      role: "黑匣子模块单元测试：二进制往返、前后窗切片、大点云降采样、冷却防风暴与磁盘配额",
      symbols: [
        { name: "BlackboxRecorderTest.*", range: "57-237", desc: "覆盖序列化完整性、自动触发窗口与环形配额淘汰" },
      ],
    },

    {
      path: "include/scan_localizer/async_global_worker.hpp",
      group: "core",
      lines: 111,
      chapter: "ch08",
      role: "异步全局重定位隔离工作线程接口：Latest-Only 单槽覆盖、Linux 绑核与 nice 降权",
      symbols: [
        { name: "struct GlobalWorkerConfig / GlobalWorkerResult", range: "34-52", desc: "绑核列表、降权 nice 与墙钟预算配置及结果快照" },
        { name: "class AsyncGlobalWorker", range: "54-99", desc: "常驻单线程异步重定位与非阻塞结果轮询" },
      ],
    },
    {
      path: "src/async_global_worker.cpp",
      group: "core",
      lines: 185,
      chapter: "ch08",
      role: "AsyncGlobalWorker 实现：pthread_setaffinity_np 绑核、setpriority 降权与自适应角步长",
      symbols: [
        { name: "AsyncGlobalWorker::apply_thread_isolation", range: "102-133", desc: "Linux 绑核与线程 nice 降权（静默降级安全）" },
        { name: "AsyncGlobalWorker::worker_loop", range: "135-183", desc: "单槽最新帧消费并透传 max_time_ms 墙钟预算" },
      ],
    },
    {
      path: "test/test_async_global_worker.cpp",
      group: "tests",
      lines: 99,
      chapter: "ch09",
      role: "AsyncGlobalWorker 单元测试：绑核降权重定位精度与高频突发提交单槽覆盖验证",
      symbols: [
        { name: "AsyncGlobalWorkerTest.*", range: "27-92", desc: "覆盖绑核执行、非阻塞轮询与旧请求自动丢弃" },
      ],
    },

  ];

  const ROUTES = {
    fast: {
      name: "3 小时快速理解主线",
      desc: "跳过底层实现细节，直击核心架构、Field-GN、4D EKF 与节点部署。",
      steps: [
        { ch: "ch01", title: "Ch01 · SE(2) 与 TF 树", note: "先懂 map→odom 为什么不能直接发 map→base_link" },
        { ch: "ch04", title: "Ch04 · Field-GN 精修", note: "看懂为什么 0.054ms 比 ICP 快 5.5 倍" },
        { ch: "ch07", title: "Ch07 · 4D EKF 与状态机", note: "理解四门控与线上防跳变机制" },
        { ch: "ch08", title: "Ch08 · LocalizerNode", note: "看懂 ROS 2 生命周期节点与参数契约" },
        { ch: "ch09", title: "Ch09 · E1–E10 与真值对比", note: "从实测数据反推工程选型依据" },
      ],
    },
    algo: {
      name: "纯算法内核精读路线",
      desc: "不依赖任何 ROS 知识，彻底啃透地图场、匹配数学与贝叶斯滤波。",
      steps: [
        { file: "include/scan_localizer/se2.hpp", title: "se2.hpp (119L)", note: "SE(2) 群逆与点云变换" },
        { file: "src/map_field.cpp", title: "map_field.cpp (439L)", note: "精确 EDT、SDF 解析梯度与 swmax_ 金字塔" },
        { file: "src/matchers.cpp", title: "matchers.cpp (1041L)", note: "5 种匹配器、cov_from_hessian 与 B&B" },
        { file: "src/particle_filter.cpp", title: "particle_filter.cpp (350L)", note: "Thrun 运动学与 log-sum-exp 重采样" },
        { file: "src/localizer_core.cpp", title: "localizer_core.cpp (760L)", note: "4D EKF、Joseph 更新与异常恢复" },
      ],
    },
    eng: {
      name: "全栈 ROS 2 生产工程路线",
      desc: "面向机器人架构师：并发锁模式、生命周期、外参 TF 与 CI 回归体系。",
      steps: [
        { file: "CMakeLists.txt", title: "CMakeLists.txt (202L)", note: "algorithm-core 与 ROS/Sim 彻底剥离" },
        { file: "src/map_io.cpp", title: "map_io.cpp (484L)", note: "健壮文件 IO 与 Y 轴翻转约定" },
        { file: "src/localizer_node.cpp", title: "localizer_node.cpp (1672L)", note: "锁外构场、TF2 3D 外参与 6×6 协方差" },
        { file: "config/localizer.yaml", title: "localizer.yaml (80L)", note: "52 项参数预算与 Nav2 对齐" },
        { file: "test/test_localizer_node.cpp", title: "test_localizer_node.cpp (866L)", note: "真实 DDS 生命周期集成测试" },
      ],
    },
  };

  global.SLOC_MANIFEST = { CHAPTERS, FILES, ROUTES };
})(window);
