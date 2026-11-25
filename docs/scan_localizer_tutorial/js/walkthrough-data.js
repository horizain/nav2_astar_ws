// Structured architectural walkthrough data for scan_localizer:
// - CLASSES: 10 core classes / structs with ownership, members & design intent
// - FLOWS: 3 step-by-step execution pipelines (Scan frame, Map load/hot-swap, Global recovery)
// - CALL_CHAINS: 3 interactive collapsible call trees linking directly to function breakdowns
// - FUNCTIONS: 14 "write-from-scratch" deep guides (contract, thinking steps, skeleton, tradeoffs, checklists)
// - BUILD_STEPS: 6-step bottom-up coding roadmap with GTest acceptance commands
(function (global) {
  "use strict";

  const CLASSES = {
    Pose2: {
      name: "Pose2 & Scan",
      file: "include/scan_localizer/se2.hpp",
      lines: "49-114",
      chapter: "ch01",
      layer: "L0 · 几何底座",
      ownership: "值语义 (POD + std::vector<Eigen::Vector2d>)",
      intent: "把 2D 刚体位姿复合 T1*T2、群逆 T⁻¹ 与零拷贝点云坐标变换收口在单一零依赖头文件中，避免各模块重复手写 cos/sin 导致符号与 wrap_angle 不一致。",
      members: [
        "double x, y, theta — 平移(m)与 [-π, π] 归一化航向角(rad)",
        "Pose2 operator*(const Pose2&) — 刚体复合，内部强制 wrap_angle",
        "Pose2 inverse() — 刚体求逆，供 map_to_odom = T_mb * T_ob.inverse() 使用",
        "transform_scan(pose, scan, out) — C++20 concept 预留内存，零分配热路径",
      ],
      fnKey: "se2_core",
    },
    MapIO: {
      name: "load_map_yaml / save_map_yaml",
      file: "src/map_io.cpp",
      lines: "271-482",
      chapter: "ch01",
      layer: "L0 · 地图 IO",
      ownership: "纯函数工具，永不抛异常（bool + std::string& err）",
      intent: "无需启动 nav2_map_server 即可直接加载/导出标准 YAML+PGM，内置图像左上角到栅格左下角的双向 Y 轴翻转、trinary/scale/raw 三模式与严格格式校验。",
      members: [
        "parse_flat_yaml() — 无第三方 yaml-cpp 依赖的扁平键值解析",
        "read_pgm() — P2/P5 校验、注释跳过与截断检测",
        "row_pgm = height - 1 - iy — ROS 标准左下角原点对齐",
      ],
      fnKey: "load_map_yaml",
    },
    OccupancyGrid: {
      name: "OccupancyGrid",
      file: "include/scan_localizer/map_field.hpp",
      lines: "42-81",
      chapter: "ch02",
      layer: "L1 · 离散地图",
      ownership: "std::shared_ptr<const OccupancyGrid> 共享只读所有权",
      intent: "严格区分整数格索引 worldToGridIndex（floor）与连续格心坐标 worldToGridF（减 0.5 偏移），确保栅格矩形边界与双线性插值顶点完全自洽。",
      members: [
        "int width, height; double resolution, origin_x, origin_y",
        "std::vector<int8_t> cells — -1 unknown, 0 free, 100 occupied",
        "worldToGridF / gridToWorld — 格心对齐 ±0.5 cell",
      ],
      fnKey: "map_field_build",
    },
    MapField: {
      name: "MapField",
      file: "include/scan_localizer/map_field.hpp",
      lines: "84-170",
      chapter: "ch02",
      layer: "L1 · 连续几何场",
      ownership: "内部持有 shared_ptr<const OccupancyGrid>（杜绝 UAF 悬空指针）",
      intent: "建图时花一次 27.7ms 预计算，喂饱 5 种局部匹配器 + B&B 全局搜索 + 粒子滤波：同时产出精确 SDF、双线性解析梯度、高斯似然场和 2^k 滑窗最大值栈 swmax_。",
      members: [
        "std::vector<float> sdf_cells_ — 双向 EDT 构造（自由>0, 表面=0, 墙内/未知<0）",
        "std::vector<float> like_ — 高斯似然 exp(-sdf² / 2σ²)",
        "std::vector<SWGrid> swmax_ — 第 k 层存 2^k × 2^k 滑窗最大似然",
      ],
      fnKey: "map_field_build",
    },
    MatcherBase: {
      name: "Matcher (抽象基类) & CorrelativeMatcher",
      file: "include/scan_localizer/matchers.hpp",
      lines: "65-81",
      chapter: "ch03",
      layer: "L2 · 匹配器族",
      ownership: "std::shared_ptr<Matcher> 多态配置驱动",
      intent: "通过统一纯虚接口 match(field, prior, scan, opt) -> MatchResult 解耦 LocalizerCore 与具体匹配算法，使得单测试验 E1–E10 与运行时热切换完全统一。",
      members: [
        "CorrelativeMatcher — 两轮 Olson 滑窗穷举 + 响应面 Softmax 二阶矩协方差",
        "IcpP2pMatcher — SDF 梯度下降对应 + 旋转力臂雅可比 + 法向 Hessian 协方差",
        "NdtMatcher — 0.5m 体素高斯建模 + 3×3 邻域 D2P 分数最大化",
      ],
      fnKey: "correlative_match",
    },
    FieldGnMatcher: {
      name: "FieldGnMatcher",
      file: "src/matchers.cpp",
      lines: "274-390",
      chapter: "ch04",
      layer: "L2 · 连续场精修",
      ownership: "无状态轻量匹配器，可多线程只读并发调用",
      intent: "以 SDF 双线性插值为标量残差、解析梯度为表面法向雅可比，配合 Huber IRLS 与残差截断，0.054ms 实现点-线 ICP 等价精修并产出无阻尼真实各向异性协方差。",
      members: [
        "eval_pose lambda — 计算 (H, b, ravg) 与 Huber 加权",
        "best_pose 回滚机制 — LM 步长发散时绝不丢失历史更优解",
        "cov_from_hessian(best_H, best_var) — 绝不混入阻尼 λI",
      ],
      fnKey: "field_gn_match",
    },
    ComposedMatcher: {
      name: "ComposedMatcher",
      file: "src/matchers.cpp",
      lines: "691-705",
      chapter: "ch04",
      layer: "L2 · 两级流水线",
      ownership: "组合持有 shared_ptr<Matcher> coarse_ & fine_",
      intent: "把“大窗口不陷局部极小（Correlative）”和“亚胞元连续精修+真实走廊协方差（FieldGn）”串联起来，并在精修退化时自动回退粗匹配解。",
      members: [
        "coarse_ -> match() — ±0.15m / ±0.10rad 捕获全局峰",
        "fine_ -> match() — 从 coarse.pose 起步收敛到 ~20mm",
        "score + 0.05 < coarse.score 防跑飞回退门限",
      ],
      fnKey: "composed_match",
    },
    GlobalAndPF: {
      name: "global_correlative & ParticleFilter",
      file: "src/matchers.cpp",
      lines: "716-1014",
      chapter: "ch06",
      layer: "L2 · 全局/对照组",
      ownership: "自由函数（传入 MapField swmax_）与成员缓冲复用类",
      intent: "用优先队列分支定界（B&B）+ Top-K NMS 实现确定性冷启动与多假设输出；同时以 AMCL-lite ParticleFilter 提供透明对照组。",
      members: [
        "std::priority_queue<Node> pq — 按可采纳上界 bound 降序展开",
        "same_mode NMS (0.50m, 0.35rad) — 避免单一峰霸占 Top-K",
        "ParticleFilter::update — log-sum-exp 防下溢与 ESS 重采样",
      ],
      fnKey: "global_bnb",
    },
    LocalizerCore: {
      name: "LocalizerCore",
      file: "include/scan_localizer/localizer_core.hpp",
      lines: "100-185",
      chapter: "ch07",
      layer: "L3 · ROS无关状态机",
      ownership: "std::unique_ptr<LocalizerCore> 被节点独占，纯 C++20",
      intent: "定位系统的“大脑”：4D [x,y,θ,b_g] 各向异性 EKF 预测+Joseph 更新、4 道安全门控、5 态状态机、多假设验证与对称走廊跳变保护。",
      members: [
        "Cov4 P4_ & double gyro_bias_ — 在线估计陀螺零偏与全 4×4 协方差",
        "GlobalFn global_fn_ — std::function 回调注入，保持算法零节点依赖",
        "std::vector<Hypothesis> hypotheses_ — 对称走廊多假设晋升队列",
      ],
      fnKey: "core_on_scan",
    },
    LocalizerNode: {
      name: "LocalizerNode (LifecycleNode)",
      file: "src/localizer_node.cpp",
      lines: "62-174",
      chapter: "ch08",
      layer: "L4 · ROS 2 适配层",
      ownership: "rclcpp_lifecycle::LifecycleNode 组件节点",
      intent: "严格隔离算法核与 ROS 2 DDS：锁外重计算（applyMap）、双回调组隔离、TF2 3D 外参（支持 roll=π 倒装）与扫描时刻 Odom 插值、锁外快照发布。",
      members: [
        "std::mutex state_mtx_ — 仅保护状态读写与瞬间指针交换",
        "sensor_cb_group_ / work_cb_group_ — Reentrant/MutuallyExclusive 分流",
        "OutputSnapshot — 拷贝输出快照后在锁外发送 TF / Pose / Diagnostics",
      ],
      fnKey: "node_on_scan",
    },
  };

  const FLOWS = {
    scan_loop: {
      name: "链路 A · 一帧激光的完整旅程（40Hz 热路径）",
      desc: "从 /scan 消息到达 LocalizerNode 开始，经历 3D 外参投影、时空对齐、4D EKF 预测、两级匹配、四道门控，直到在锁外广播 map→odom。",
      steps: [
        { id: "s1", title: "1. 读参数快照", sub: "active_ / state_mtx_", fnKey: "node_on_scan", desc: "先用 atomic active_ 快速放行；再短锁读取外参、sub_sample 等局部副本后立即解锁。" },
        { id: "s2", title: "2. TF2 3D 外参转点云", sub: "onScan() L1217-1293", fnKey: "node_on_scan", desc: "优先查 TF2 base→laser（兼容倒装 roll=π 雷达），投影到 base_frame 并过滤 NaN/超界点。" },
        { id: "s3", title: "3. 扫描时刻 Odom 对齐", sub: "lookupOdomAt()", fnKey: "node_on_scan", desc: "用激光真实时间戳 scan_stamp 查 TF2 odom→base，若超时回退环形缓冲线性插值。" },
        { id: "s4", title: "4. 4D EKF 预测", sub: "F4 / Q4 + bias", fnKey: "core_on_scan", desc: "扣除 gyro_bias_ 算运动增量 Δpose，构建含 -Δt 的 4×4 雅可比 F4 与体/地图系过程噪声 Q4。" },
        { id: "s5", title: "5. 两级扫描匹配", sub: "Correlative → Field-GN", fnKey: "composed_match", desc: "先用 Correlative 大窗抓初值防局部极小，再用 Field-GN 0.054ms 连续精修并算无阻尼协方差。" },
        { id: "s6", title: "6. 四道门控审验", sub: "Score/Inlier/Cov/Mahal", fnKey: "core_on_scan", desc: "连续验证得分≥0.35、内点率≥0.60、min_eig∈(0,1.0)、马氏距离<9.0，一票否决坏匹配。" },
        { id: "s7", title: "7. Joseph 更新或拒绝", sub: "状态更新 / 静止解锁", fnKey: "core_on_scan", desc: "全通过→4D Joseph 更新 pose 与 b_g；若高分仅马氏拒→适度松弛 P 防静止锁死，连拒→DEGRADED/GLOBAL。" },
        { id: "s8", title: "8. 构造快照与释放锁", sub: "buildSnapshotLocked()", fnKey: "node_publish", desc: "用 last_pose_ * odom_at_last_scan_.inverse() 算出 map→odom，装进 OutputSnapshot 后彻底解锁。" },
        { id: "s9", title: "9. 锁外发布输出", sub: "publishOutputs()", fnKey: "node_publish", desc: "锁外发 TF(带 transform_tolerance)、6×6 协方差 [0,1,5] 位姿与 DiagnosticArray 状态。" },
      ],
    },
    map_load: {
      name: "链路 B · 地图加载与热切图流水线",
      desc: "无论是启动读取 map_yaml，还是运行时动态订阅 /map，所有耗时预计算全在互斥锁外完成，切换瞬间零卡顿。",
      steps: [
        { id: "m1", title: "1. 解析 YAML + PGM", sub: "load_map_yaml()", fnKey: "load_map_yaml", desc: "扁平解析 YAML 字段，用 row_pgm = h-1-iy 翻转图像行序对齐 ROS 左下角坐标。" },
        { id: "m2", title: "2. 锁外读取构建参数", sub: "applyMap() L909-918", fnKey: "node_apply_map", desc: "短锁拷贝 sigma、matcher_name、ndt_voxel，随后立即释放 state_mtx_。" },
        { id: "m3", title: "3. 双向 1D/2D EDT", sub: "edt1d / edt2d", fnKey: "edt_1d_2d", desc: "以占据格与自由格分别做种子运行 O(HW) 抛物线下包络（int64 + floor_div 防负数截断错）。" },
        { id: "m4", title: "4. 构造 SDF 与似然场", sub: "MapField::build()", fnKey: "map_field_build", desc: "合成双向带符号距离场（未知区给有界负值）并一次算完高斯似然表 like_。" },
        { id: "m5", title: "5. 滑窗金字塔 swmax_", sub: "2^k × 2^k 最大值栈", fnKey: "map_field_build", desc: "基于前一层 4 邻域取 max，在 O(HW·log W) 内建成可供 B&B 直接查表的上界金字塔。" },
        { id: "m6", title: "6. 锁外预建 Matcher", sub: "make_matcher()", fnKey: "node_apply_map", desc: "若是 NDT 在此处预构建体素高斯桶；失败则直接返回 false，丝毫不污染线上旧地图。" },
        { id: "m7", title: "7. 加锁原子指针交换", sub: "std::move(field_)", fnKey: "node_apply_map", desc: "短锁内交换 grid_/field_/matcher_ 并通知 core_，旧图由 shared_ptr 安全延迟释放。" },
      ],
    },
    global_recovery: {
      name: "链路 C · 冷启动与绑架全局恢复",
      desc: "当机器人处于 UNINIT / LOST / GLOBAL 时如何 20ms 找回全局位置，并防止在对称走廊里被误触发 20m 错跳。",
      steps: [
        { id: "g1", title: "1. 进入恢复态", sub: "冷启动 / 绑架 / 服务", fnKey: "core_on_scan", desc: "首次 UNINIT、连续异常达到 anomaly_bad_threshold，或调用 /global_relocalize 服务进入 GLOBAL。" },
        { id: "g2", title: "2. 自由框与角度离散", sub: "global_correlative", fnKey: "global_bnb", desc: "遍历自由空间得搜索包围盒，按 ang_step 预旋转降采样激光点至整数偏移数组。" },
        { id: "g3", title: "3. 顶层块打分入队", sub: "Level K, 步长 2^K", fnKey: "global_bnb", desc: "查 swmax_[K] 得可采纳得分上界（整块越界点记 0 分），超 0.05 的顶层节点压入大顶堆 pq。" },
        { id: "g4", title: "4. B&B 展开与剪枝", sub: "bound <= worst_kept 退出", fnKey: "global_bnb", desc: "每弹出非叶节点分裂为 4 个子块；一旦收集满 topk 峰且队列堆顶上界不如第 K 优，直接剪枝。" },
        { id: "g5", title: "5. 精确重评 + NMS + 精修", sub: "全点集 ±2 格重搜", fnKey: "global_bnb", desc: "叶子节点在原图精确重评分，按 0.50m/0.35rad NMS 去重后做小窗口全点集精修，产出主解与 alternates。" },
        { id: "g6", title: "6. 远距离跳变安全门", sub: "防对称走廊误跳 (98fcd28)", fnKey: "core_on_scan", desc: "非手动触发且跳变>1.0m 时，先测局部 pred_local：若全局得分未能领先局部 +0.15，则拒跳并留待观察。" },
        { id: "g7", title: "7. 多假设队列晋升", sub: "try_promote_hypotheses", fnKey: "core_on_scan", desc: "若真绑架导致局部失稳，DEGRADED 阶段随里程计同步传播 alternates，连续 2 帧高分验证即晋升转正。" },
      ],
    },
  };

  const CALL_CHAINS = {
    runtime: {
      name: "高频定位主链 (/scan 回调)",
      root: {
        fn: "LocalizerNode::onScan(LaserScan)",
        note: "ROS 线程入口，3D 外参+时钟对齐",
        fnKey: "node_on_scan",
        children: [
          { fn: "tf_buffer_->lookupTransform(base, laser)", note: "查 3D 静态外参，支持 roll=π 倒装", fnKey: "node_on_scan" },
          { fn: "LocalizerNode::lookupOdomAt(stamp)", note: "TF2 查表，超时退回环形缓冲线性插值", fnKey: "node_on_scan" },
          {
            fn: "LocalizerCore::on_odom() & on_scan()",
            note: "进入纯 C++ 核心状态机",
            fnKey: "core_on_scan",
            children: [
              { fn: "compute_motion_delta()", note: "合成平移 + 减零偏后 IMU Δθ", fnKey: "core_on_scan" },
              {
                fn: "ComposedMatcher::match()",
                note: "粗捕获 + 精修两级流水线",
                fnKey: "composed_match",
                children: [
                  { fn: "CorrelativeMatcher::match()", note: "Olson 粗搜 + 细搜 + Softmax 协方差", fnKey: "correlative_match" },
                  {
                    fn: "FieldGnMatcher::match()",
                    note: "SDF 连续场高斯-牛顿精修",
                    fnKey: "field_gn_match",
                    children: [
                      { fn: "MapField::sdf(gx, gy, &grad)", note: "双线性插值残差与解析梯度", fnKey: "map_field_sdf" },
                      { fn: "cov_from_hessian(best_H)", note: "未阻尼特征分解求逆保退化椭圆", fnKey: "cov_from_hessian" },
                      { fn: "eval_stats()", note: "计算 score 与 inlier_ratio", fnKey: "field_gn_match" },
                    ],
                  },
                ],
              },
              { fn: "四门控检查 + Joseph 4×4 更新", note: "Score/Inlier/MinEig/Mahal，或拒绝松弛", fnKey: "core_on_scan" },
            ],
          },
          {
            fn: "buildSnapshotLocked() → publishOutputs()",
            note: "构造快照并解锁后发 DDS 输出",
            fnKey: "node_publish",
            children: [
              { fn: "LocalizerCore::map_to_odom()", note: "last_pose_ * odom_at_last_scan_.inverse()", fnKey: "se2_core" },
            ],
          },
        ],
      },
    },
    startup: {
      name: "建图与热切图调用链",
      root: {
        fn: "LocalizerNode::on_configure() / onMap()",
        note: "配置或动态 /map 话题回调",
        fnKey: "node_apply_map",
        children: [
          { fn: "loadMapFromYaml() → load_map_yaml()", note: "读取 YAML+PGM，row_pgm = h - 1 - iy 翻转", fnKey: "load_map_yaml" },
          {
            fn: "LocalizerNode::applyMap(grid, keep_pose)",
            note: "锁外构场 + 锁内原子指针交换",
            fnKey: "node_apply_map",
            children: [
              {
                fn: "MapField::build(grid, sigma)",
                note: "一次性构建 4 层场",
                fnKey: "map_field_build",
                children: [
                  { fn: "edt2d() → edt1d()", note: "行列分离 + int64 抛物线下包络", fnKey: "edt_1d_2d" },
                  { fn: "构建 sdf_cells_ / like_ / swmax_", note: "双向 SDF 与 2^k 滑窗栈", fnKey: "map_field_build" },
                ],
              },
              { fn: "make_matcher(name, grid)", note: "按配置构建 Composed / NDT 等实例", fnKey: "node_apply_map" },
              { fn: "加锁 swap + core_->set_map/matcher()", note: "无阻塞瞬间切图", fnKey: "node_apply_map" },
            ],
          },
        ],
      },
    },
    global: {
      name: "冷启动 / 绑架全局重定位链",
      root: {
        fn: "LocalizerCore::on_scan() [UNINIT/LOST/GLOBAL]",
        note: "状态机顶部分支，短路常规局部流程",
        fnKey: "core_on_scan",
        children: [
          { fn: "propagate_hypotheses() & try_promote_hypotheses()", note: "先验多假设随里程计同步验证晋升", fnKey: "core_on_scan" },
          {
            fn: "global_fn_() → runGlobalSearchLocked()",
            note: "外层节点注入的全局搜索回调",
            fnKey: "global_bnb",
            children: [
              {
                fn: "global_correlative()",
                note: "基于 swmax_ 的优先队列分支定界",
                fnKey: "global_bnb",
                children: [
                  { fn: "eval_block_bound(level, bx, by, A)", note: "查 swmax_[level] 算可采纳块上界", fnKey: "global_bnb" },
                  { fn: "NMS 去重 + ±2 格全点集精修", note: "产出主位姿与 alternates 备选假设", fnKey: "global_bnb" },
                ],
              },
              { fn: "core_->add_hypotheses(global_alternates_)", note: "分离外部假设与内部候选", fnKey: "core_on_scan" },
            ],
          },
          { fn: "远跳安全保护门 + 复位 P4_ / gyro_bias_", note: "防对称走廊误瞬移；跳变成功重置滤波", fnKey: "core_on_scan" },
        ],
      },
    },
  };

  const FUNCTIONS = {
    se2_core: {
      title: "Pose2::operator* / inverse / map_to_odom",
      file: "include/scan_localizer/se2.hpp",
      lines: "49-114",
      chapter: "ch01",
      contract: "输入：SE(2) 位姿 T1, T2 或点云 Scan。输出：复合位姿 T1*T2、群逆 T⁻¹、世界系点云或 map→odom TF。",
      thoughtSteps: [
        "先把几何约定写成单一事实：Pose2(x, y, theta) 代表“先绕原点转 theta，再平移 (x, y)”。",
        "写 operator*：根据齐次矩阵乘法 R1(R2*p + t2) + t1 = (R1*R2)*p + (R1*t2 + t1)，直接展开 4 次乘法并在构造结果时强制 wrap_angle。",
        "写 inverse()：对 T^-1 = [-R^T t, -θ] 展开即得 -(c*x + s*y), -(-s*x + c*y)。",
        "写 transform_scan 重载：提供传入复用引用 std::vector<Vec2>& out 的版本，用 C++20 concept 在编译期判断是否一次 reserve，消除热路径 malloc。",
        "在 LocalizerCore::map_to_odom() 中用 last_pose_ * odom_at_last_scan_.inverse() 求得平滑 TF。",
      ],
      skeleton: `Pose2 T_res = {
  x + c*o.x - s*o.y,
  y + s*o.x + c*o.y,
  wrap_angle(theta + o.theta)
};
// 关键：必须用锚定到 scan 时刻的 odom 求逆！
Pose2 map_to_odom = last_pose_ * odom_at_last_scan_.inverse();`,
      tradeoffs: [
        "为何手写 2D Pose2 而不全程用 Eigen::Isometry2d？因为 2D 定位大量操作只用 (x,y,θ) 标量展开（如旋转步长、角度归一化），轻量 POD 结构体可直接在寄存器传值且无额外 3×3 构造开销。",
        "wrap_angle 必须用 O(1) std::remainder 并屏蔽 NaN/inf，绝不能写 while 循环。",
      ],
      checklist: [
        "inverse() 后与原位姿相乘误差 < 1e-12？",
        "map_to_odom() 是否误用了异步到达的最新 last_odom_（会引起锯齿抖动）？",
      ],
    },
    load_map_yaml: {
      title: "load_map_yaml & read_pgm (地图读写)",
      file: "src/map_io.cpp",
      lines: "271-398",
      chapter: "ch01",
      contract: "输入：yaml_path。输出：填充好的 ROS 标准左下角原点 OccupancyGrid，失败返回 false 与清晰 err。",
      thoughtSteps: [
        "确定“永不抛异常”原则：用布尔返回值 + std::string& err 报告格式或 IO 错，方便嵌入式和单测拦截。",
        "用 parse_flat_yaml 读取 image/resolution/origin/negate/thresholds（不引入 yaml-cpp 外部库风险）。",
        "用 std::filesystem::path 处理相对图片路径：仅当 yaml_fs.parent_path() 非空时拼接，修复裸文件名 \"floor.yaml\" Bug。",
        "解析 PGM 魔数（P2 ASCII / P5 Binary），严格跳过 '#' 注释并校验实际字节数 gcount() 防截断。",
        "遍历 iy = 0..h-1，取 row_pgm = h - 1 - iy 做垂直翻转，按 trinary / scale / raw 写入 cells。",
      ],
      skeleton: `// 关键：PGM 第 0 行在左上角，ROS 栅格 iy=0 在左下角
for (int iy = 0; iy < h; ++iy) {
  size_t row_pgm = h - 1 - iy;
  for (int ix = 0; ix < w; ++ix) {
    uint8_t v = gray[row_pgm * w + ix];
    double p = negate ? (v / inv) : ((maxval - v) / inv);
    grid.cells[iy * w + ix] = (p > occ_th) ? 100 : (p < free_th ? 0 : -1);
  }
}`,
      tradeoffs: [
        "为什么不用第三方库直接读？为了保持 scan_localizer_core 零第三方解析库耦合、编译零额外依赖。",
      ],
      checklist: [
        "是否处理了 Windows CRLF 换行与 PGM 头注释行？",
        "save_map_yaml → load_map_yaml 往返测试每个像素是否 100% 一致？",
      ],
    },
    edt_1d_2d: {
      title: "edt1d & edt2d (精确欧氏距离下包络)",
      file: "src/map_field.cpp",
      lines: "35-138",
      chapter: "ch02",
      contract: "输入：宽度 W、高度 H、0/1 种子数组 seeds。输出：每个栅格到最近种子点的精确平方欧氏距离 dist2_out。",
      thoughtSteps: [
        "明确为什么要分解：2D 平方距离 min_{x',y'} ((x-x')^2 + (y-y')^2) 可拆成先逐列算 1D 距离 g(x, y')，再逐行算下包络 min_{x'} ((x-x')^2 + g(x', y)^2)。",
        "确定数值安全类型：对于宽地图 W+H 平方会接近或超 int32 风险，因此全程用 std::int64_t 并且 INF = W + H + 2。",
        "写 edt1d 维护两条栈：v[] 存仍参与下包络的抛物线顶点坐标，z[] 存相邻两条抛物线的分割交点。",
        "核心不变量：计算新抛物线 q 与当前栈顶 p 的交点横坐标 s 时，必须调用精确向下取整 floor_div(num, den)！",
        "若 s <= z[k]，说明栈顶 p 被新抛物线 q 完全抬高遮蔽，弹栈 (--k) 并重算 s；最后顺序双指针扫 q = 0..n-1 读值。",
      ],
      skeleton: `for (size_t q = 1; q < n; ++q) {
  int64_t s = floor_div((f[q] + q*q) - (f[p] + p*p), 2*(q - p));
  while (s <= z[k]) { --k; p = v[k]; s = floor_div(...); }
  v[++k] = q; z[k] = s; z[k+1] = kZPos;
}`,
      tradeoffs: [
        "BFS/Dijkstra 只能算 4/8 邻域棋盘近似距离，且大地图速度慢；抛物线下包络是严格 O(W·H) 且精确欧氏距离（各向同性），SDF 梯度才不会歪。",
      ],
      checklist: [
        "是否用 test_map_field 测过 edt1d({9, 100, 4}) == {8, 5, 4}？（排查 C++ '/' 负数截断陷阱）",
      ],
    },
    map_field_build: {
      title: "MapField::build (四层地图场一次成型)",
      file: "src/map_field.cpp",
      lines: "149-279",
      chapter: "ch02",
      contract: "输入：shared_ptr<const OccupancyGrid>, sigma_m。输出：sdf_cells_, like_, swmax_ 金字塔。",
      thoughtSteps: [
        "先做所有输入参数防御（空指针、非正分辨率/sigma、尺寸与 cells 大小不匹配）并清空旧状态。",
        "分离两份种子：occ_seeds (> occ_thresh) 和 free_seeds (0..free_thresh)，分别跑一次 edt2d 得 d_occ 与 d_free。",
        "合成双向 SDF：自由格 sdf = +sqrt(d_occ)*res；墙体格 sdf = -(sqrt(d_free)-1)*res（贴墙表面那圈恰好=0）；未知格给有界负值且似然设为 0.10。",
        "把第 0 层似然场拷贝为 swmax_[0]，然后自底向上逐层对 (x, y), (x+h, y), (x, y+h), (x+h, y+h) 取 max 构造 2^k 滑窗最大值栈。",
      ],
      skeleton: `// 1. 双向 EDT
edt2d(w, h, occ_seeds, d_occ);
edt2d(w, h, free_seeds, d_free);
// 2. 分区赋值 SDF 与似然
// free: sdf = +sqrt(d_occ)*res; occ: -(sqrt(d_free)-1)*res; unknown: <0 & low like
// 3. 构造滑窗金字塔 (每次跨步 h = 2^(k-1))
dst[y,x] = max(src[y,x], src[y,xh], src[yh,x], src[yh,xh]);`,
      tradeoffs: [
        "为何未知区不能把 SDF 设成 0？若设为 0，梯度和优化会把所有激光束往未建图空白区吸！",
        "为什么保存 shared_ptr<const OccupancyGrid> 而非裸指针？彻底消灭局部变量析构导致 UAF。",
      ],
      checklist: [
        "全自由图（无障碍）和全墙图（无自由空间）两个退化分支是否安全不除零/越界？",
      ],
    },
    map_field_sdf: {
      title: "MapField::sdf (连续双线性插值 + 解析梯度)",
      file: "src/map_field.cpp",
      lines: "281-315",
      chapter: "ch02",
      contract: "输入：连续格心坐标 (gx, gy)。输出：平滑 SDF(m) 及可选物理单位解析梯度 grad (m/m)。",
      thoughtSteps: [
        "检查 valid_ 与 std::isfinite(gx, gy)；非法时梯度清零并返回大数，避免给外部优化器 NaN。",
        "把 (gx, gy) 夹紧在 [0, w-1] × [0, h-1]，求左下格心整数索引 x0, y0 及小数权重 fx = gx - x0, fy = gy - y0。",
        "取出 2×2 邻域格值 v00, v10, v01, v11，套用双线性插值公式算出残差。",
        "直接对插值公式求 gx/gy 偏导并除以 resolution，即得物理单位 (1/m * m = m/m) 解析梯度！",
      ],
      skeleton: `fx = gx - x0; fy = gy - y0;
grad->x() = ((v10 - v00)*(1 - fy) + (v11 - v01)*fy) / res;
grad->y() = ((v01 - v00)*(1 - fx) + (v11 - v10)*fx) / res;
return v00*(1-fx)*(1-fy) + v10*fx*(1-fy) + v01*(1-fx)*fy + v11*fx*fy;`,
      tradeoffs: [
        "解析求导 vs 中心有限差分：解析梯度一次取 4 个 float 就算完，速度比再调 4 次 sdf() 快 4 倍且没有步长 h 挑选烦恼。",
      ],
      checklist: [
        "对 width == 1 或 height == 1 的退化极窄栅格是否做了 x0=0, x1=0, fx=0 保护？",
      ],
    },
    correlative_match: {
      title: "CorrelativeMatcher::match (Olson 两级穷举)",
      file: "src/matchers.cpp",
      lines: "105-269",
      chapter: "ch03",
      contract: "输入：MapField, prior, scan, MatchOptions。输出：窗口最优位姿 + Softmax 响应面二阶矩协方差。",
      thoughtSteps: [
        "先过滤 NaN/Inf 点到 valid_pts，少于 5 点直接返回 invalid。",
        "外层分两轮（round=0 粗搜大窗口，round=1 在粗最优 ±1 个粗步长内以 1/3 步长细搜，且 round=1 用连续 SDF 消除平顶量化）。",
        "经典加速技巧：先固定外层旋转角 θ 把所有点转好存进 cells，内层 (yi, xi) 循环只做 dx/res、dy/res 标量加法查表！",
        "记录第 0 轮全部网格分数 coarse_poses，用自适应温度 softmax w = exp((s - smax)*beta) 统计离散响应面二阶矩 m2 - mean*mean^T 作为协方差。",
        "做特征值分解下界夹紧保证 SPD 协方差，最后调用 eval_stats 算出规范化 score 与 inlier_ratio。",
      ],
      skeleton: `for (round = 0..1) {
  for (ai in angles) {
    rotate_points_once(prior, th -> cells); // 提取到内层平移循环之外
    for (yi) for (xi) {
      offx = (cand_x - prior.x) / res; offy = (cand_y - prior.y) / res;
      score = avg(round==0 ? likelihood(round(cell+off)) : exp(-sdf²/2σ²));
    }
  }
}`,
      tradeoffs: [
        "为什么不用单一极细网格直接暴力穷举？粗+细两级把计算量缩小 9~25 倍（默认仅 ~6.4ms，降采样到 360 束只要 3.3ms）。",
      ],
      checklist: [
        "round=1 重新找最高分时，是否重置 best_s = -1（避免细搜分数受连续化略低而被粗解平顶卡死）？",
        "offx 是否严格相对旋转中心 prior.x 计算，而不是相对 cand_x 重复累加？",
      ],
    },
    field_gn_match: {
      title: "FieldGnMatcher::match (SDF 连续场高斯-牛顿)",
      file: "src/matchers.cpp",
      lines: "274-390",
      chapter: "ch04",
      contract: "输入：MapField, prior, scan, opt。输出：亚胞元精修位姿 + 未阻尼 Hessian 协方差（真实反映退化）。",
      thoughtSteps: [
        "把“评估当前 pose”写成局部 lambda eval_pose：转点云 → 查 sdf(g.x, g.y, &grad) → 踢掉 |r| > trim_dist 遮挡外点 → 算 Huber IRLS 权重 w。",
        "用链式法则合成 1×3 雅可比 J = [gx, gy, gx*(-(wy-py)) + gy*(wx-px)]，累加加权正规方程 H += w*J*J^T 和 b += w*J*r。",
        "每轮求解前，先用当前 pose 与刚算出的 ravg 更新“历史最优 best_pose / best_H”（保证阻尼走偏可回滚）。",
        "仅向副本矩阵 Hd 加微小对角阻尼求解 dp = -Hd.ldlt().solve(b)，并给单步加护栏（≤0.2m / 0.1rad），满足收敛阈值提前退出。",
        "退出循环后再评估一次终态 pose 是否比 best_pose 更好，最终只拿**未阻尼 best_H** 传入 cov_from_hessian！",
      ],
      skeleton: `double r = field.sdf(g.x(), g.y(), &grad);
if (abs(r) > trim_dist) continue; // 动态遮挡截断
double w = abs(r) <= huber_delta ? 1.0 : huber_delta / abs(r);
Vector3d J(grad.x(), grad.y(), grad.x()*(-(wy-py)) + grad.y()*(wx-px));
H += w * J * J.transpose(); b += w * J * r;
Hd = H + 1e-6 * diag; dp = Hd.ldlt().solve(-b);`,
      tradeoffs: [
        "为什么单步 0.054ms 比 ICP-P2P (0.297ms) 快 5.5 倍？省去了每个点沿梯度网格下降寻找障碍单元的多步循环。",
        "绝不把 Hd 送进 cov_from_hessian：LM 阻尼会把长走廊本来=0 的特征值人为抬高，假装沿走廊很准。",
      ],
      checklist: [
        "早期收敛 break 时是否评估并采纳了更新后的最终位姿？",
        "trim_dist (0.50m) 与 huber_delta (0.10m) 是否合理抑制近距行人箱子？",
      ],
    },
    cov_from_hessian: {
      title: "cov_from_hessian (退化安全信息矩阵求逆)",
      file: "src/matchers.cpp",
      lines: "71-98",
      chapter: "ch05",
      contract: "输入：未阻尼 3×3 Hessian H、残差方差 res_var。输出：经特征值夹紧、对称化与物理下限保护后的 3×3 位姿协方差 Cov3。",
      thoughtSteps: [
        "先显式对称化 Hsym = 0.5*(H + H.transpose())，消除浮点加法带来的微弱不对称。",
        "用 Eigen::SelfAdjointEigenSolver 计算特征值与正交特征向量 Q（Hsym 是实对称矩阵）。",
        "核心安全机制：将小于 kMinEig (1e-4) 的特征值夹紧到 1e-4 再求倒数！这样不可观测方向的方差膨胀至 10,000×尺度，既不除零爆炸又能如实告诉 EKF“这轴不要信激光”。",
        "算回矩阵 H_inv = Q * inv_eigs * Q^T，乘以物理误差尺度 scale * inflation 并钳位对角物理下限 (1cm², 0.2°²)。",
      ],
      skeleton: `SelfAdjointEigenSolver<Matrix3d> es(0.5 * (H + H.transpose()));
Vector3d eigs = es.eigenvalues().cwiseMax(1e-4);
Matrix3d H_inv = es.eigenvectors() * eigs.cwiseInverse().asDiagonal() * es.eigenvectors().transpose();
Cov3 cov = std::max(res_var, 1e-4) * inflation * H_inv;`,
      tradeoffs: [
        "直接调 H.inverse() 在纯平行走廊上几乎奇异，要么抛出 NaN 要么产生负数方差，特征值夹紧彻底解决了数值稳定和物理表征。",
      ],
      checklist: [
        "Eigen 对称化时写 .eval() 了吗？（防 aliasing）",
        "走廊实验 E5 是否实测 cov_along ≈ 12.48 m², cov_across ≈ 0.0001 m²？",
      ],
    },
    composed_match: {
      title: "ComposedMatcher::match (粗捕获+精修保护)",
      file: "src/matchers.cpp",
      lines: "691-705",
      chapter: "ch04",
      contract: "输入：MapField, prior, scan, opt。输出：大捕获范围且亚胞元精度的 MatchResult。",
      thoughtSteps: [
        "校验 coarse_ 与 fine_ 指针（缺失一端时优雅退化为另一端，绝不空指针崩溃）。",
        "先用 coarse_->match(prior) 把初始 ±0.4m/10° 误差抓回到正确盆地附近。",
        "以 coarse.pose 作为 fine_->match 的先验继续迭代到 20mm。",
        "关键护栏：如果精修后意外失败或者似然分反而比粗解掉超过 0.05（如局部强噪干扰），果断丢弃精修退回 coarse！",
      ],
      skeleton: `MatchResult coarse = coarse_->match(field, prior, scan, opt);
if (!coarse.valid || !fine_) return coarse;
MatchResult fine = fine_->match(field, coarse.pose, scan, opt);
if (!fine.valid || fine.score + 0.05 < coarse.score) return coarse;
return fine;`,
      tradeoffs: [
        "为什么默认不用单独 FieldGnMatcher？在 E2 实验 0.80m 初始偏差下，单独 field_gn 成功率只有 48%，而 composed 可达 86%（宽窗 100%）。",
      ],
      checklist: [
        "粗搜窗口 ±0.15m 是否与上游 40Hz odom 一周期打滑预算匹配？",
      ],
    },
    global_bnb: {
      title: "global_correlative (金字塔分支定界冷启动)",
      file: "src/matchers.cpp",
      lines: "716-1014",
      chapter: "ch06",
      contract: "输入：MapField (含 swmax_), scan, beams_subsample, ang_step, n_hypotheses。输出：主假设与多峰 alternates。",
      thoughtSteps: [
        "收集自由空间边界框 [free_xmin..ymax]，允许部分越界射线按 0 分处理（修复边缘全被判死 Bug）。",
        "按 ang_step 预旋降采样激光束存入整数格数组 AngData；定义 eval_block_bound 查 swmax_[level]（对整块彻底出图的光束跳过，避免边界夹紧虚高）。",
        "顶层块 (步长 2^K) 全部评估并入 max-heap 优先队列；只要栈顶上界小于当前第 topk 优得分 worst_kept，剪掉剩余全部队列！",
        "弹出 level=0 叶子节点用原始似然精确重算，用 same_mode (0.50m, 0.35rad) NMS 管理前 K 个互斥峰值并实时更新 worst_kept。",
        "最后把 K 个粗糙峰值放在全量点集上进行 ±2格 / ±0.5角步 细修，输出最稳 Top-1 与 alternates。",
      ],
      skeleton: `while (!pq.empty()) {
  Node nd = pq.top(); pq.pop();
  if (hyps.size() >= topk && nd.bound <= worst_kept) break; // 剪枝!
  if (nd.level == 0) { exact_eval_and_nms_topk(...); continue; }
  for (4 children) if (bound > worst_kept) pq.push(child);
}`,
      tradeoffs: [
        "为何抛弃全局粒子滤波改用 B&B？E8 实测 20,000 粒子随机散落 40×30m 仓库仅 6.7% 召回且耗时 446ms；B&B 只需 20.38ms 稳定抓回。",
      ],
      checklist: [
        "worst_kept 必须在 hyps 达满 topk 时才生效，未满 topk 时保持 0.0。",
        "跨界光束若整个 block 在图外必须跳过（不能 clamp 到边界格偷分）。",
      ],
    },
    core_on_scan: {
      title: "LocalizerCore::on_scan (4D EKF + 状态机大脑)",
      file: "src/localizer_core.cpp",
      lines: "398-758",
      chapter: "ch07",
      contract: "输入：车体系点云 Scan（外层已转外参）。输出：CoreOutput(位姿/3×3协方差/状态/是否采纳)。",
      thoughtSteps: [
        "先过滤 NaN/Inf 点（全有限时走 const Scan* 零拷贝快路径）。",
        "【分支 1】若处于 UNINIT / LOST / GLOBAL：优先沿用 Δpose 传播并尝试晋升多假设队列；否则调 global_fn_ 并过「远距离跳变安全保护门」(>1.0m 且得分不够领先绝不乱跳，只加假设)。",
        "【分支 2】正常追踪先预测：扣除 gyro_bias_ 算 Δpose → 构造 4×4 F4/Q4 → P4_pred = F4*P4*F4^T + Q4。",
        "调 matcher_->match() 后过四道门控：gate_inlier / gate_score / gate_cov(min_eig!) / gate_mahal(LDLT S^-1)。",
        "全部通过：按 K4 = (S^-1 H P4_pred)^T 求 4D 修正 dx 更新 pose 与 gyro_bias_，执行 4×4 Joseph 公式保正定，重置拒收计数。",
        "任一门控拒绝：纯预测外推！如果高分高内点却仅因马氏距离过紧被拒（静止锁死），1.5 倍松弛 P4_ 解锁；否则累计 reject_streak_ / anomaly_window_ 触发降级或 GLOBAL。",
      ],
      skeleton: `// 四门控决定采纳或纯预测外推
if (gate_inlier && gate_score && gate_cov && gate_mahal) {
  K4 = ldlt.solve(P4_pred.topRows<3>()).transpose();
  update_pose_and_bias(K4 * innov);
  P4_ = (I - K4*H)*P4_pred*(I - K4*H).transpose() + K4*R*K4.transpose();
} else {
  if (gate_inlier && gate_score && gate_cov && !gate_mahal) relax_cov(); // 解锁
  update_degraded_lost_or_anomaly_global();
}`,
      tradeoffs: [
        "4D 状态 vs 固定外推：消费级 MEMS 陀螺存在 ~0.02~0.04 rad/s 零偏，不在线估算的话遇纯滑移地毯 104m 必定偏飞；4D 实测误差仅 2.33 cm。",
        "注意：高分高内点仅马氏不过绝不能记为 is_anomaly（98fcd28 核心修补）。",
      ],
      checklist: [
        "首次收到 IMU 时是否做双锚定避免跳变？（on_imu_yaw L226-234）",
        "全局复原成功后是否重置了 gyro_bias_ 和 anomaly_window_？",
      ],
    },
    node_apply_map: {
      title: "LocalizerNode::applyMap (锁外构场+原子切图)",
      file: "src/localizer_node.cpp",
      lines: "890-963",
      chapter: "ch08",
      contract: "输入：OccupancyGrid shared_ptr 与 keep_pose。输出：无卡顿替换在线地图与匹配器。",
      thoughtSteps: [
        "先验图长宽、resolution 和 cells 字节数匹配（不对立即打 ERROR 返回，绝不崩溃）。",
        "短锁取出 likelihood_sigma / matcher_name / ndt_voxel 局部拷贝，随即立即释放锁！",
        "在完全无锁状态下构建庞大的 MapField::build（28ms+）与 make_matcher（NDT 构桶）。",
        "如果构场或构匹配器失败，提前 return false，线上正在运行的旧图完全不受影响。",
        "再获取 state_mtx_，执行微秒级指针交换 grid_ / field_ / matcher_ 并更新 core_；若非首次且 !keep_pose 触发 request_global()。",
      ],
      skeleton: `{ lock_guard lk(state_mtx_); copy_build_params(); } // 立刻释放锁
auto field = make_shared<MapField>(); field->build(grid, sigma); // 锁外耗时构建
auto matcher = make_matcher(name, grid.get(), voxel);
{ lock_guard lk(state_mtx_); swap(grid_, field_, matcher_); core_->set_map(field_); }`,
      tradeoffs: [
        "若全程持锁构建 MapField，高频 100Hz IMU 与 40Hz Scan 回调全部堵塞丢帧甚至触发 Odom 超时报错。",
      ],
      checklist: [
        "on_cleanup() 时是否统一调用 resetResources() 清空核心与订阅定时器，防止二次 activate() 鬼影？",
      ],
    },
    node_on_scan: {
      title: "LocalizerNode::onScan (3D 外参与时间对齐入口)",
      file: "src/localizer_node.cpp",
      lines: "1189-1336",
      chapter: "ch08",
      contract: "输入：LaserScan::SharedPtr。输出：驱动 Core 推进一帧并锁外发布输出。",
      thoughtSteps: [
        "先查 std::atomic<bool> active_，非 ACTIVE 期零开销丢弃。",
        "短锁拷贝所需参数快照后立刻解锁。",
        "查 TF2 3D base_frame ← scan.frame_id 旋转四元数与平移（失败才退回 2D laser_x/y/theta）。",
        "遍历 rays（隔 scan_subsample 抽样），过滤 <=0/NaN/超界光线并用 R_bl*p + t_bl 投回车体系 2D。",
        "按 scan_stamp 调用 lookupOdomAt 精确同步当时里程计位姿。",
        "短锁内喂入 core_->on_odom 与 core_->on_scan，生成纯值 OutputSnapshot snap 后立刻释放锁，交给 publishOutputs()！",
      ],
      skeleton: `lookupTransform(base_frame, m->header.frame_id, scan_stamp);
for (i = 0; i < n; i += step) {
  p_base = R_bl * Vector3d(r*cos(a), r*sin(a), 0) + t_bl;
  scan.points.emplace_back(p_base.x(), p_base.y());
}
lookupOdomAt(scan_stamp, odom_at_scan, stale);
{ lock_guard lk(state_mtx_); core_->on_odom(odom_at_scan); out = core_->on_scan(scan); snap = buildSnapshotLocked(out, stale); }
publishOutputs(snap, scan_stamp);`,
      tradeoffs: [
        "为什么先用 TF2 再投影 2D 而不是直接调 tf2_geometry_msgs？手写 Eigen 3D 变换避免额外消息依赖且在 720 束循环里极快。",
      ],
      checklist: [
        "倒装雷达 roll=π 时 p_base.y() 是否正确取反？",
        "有效点数 < min_scan_points 时是否节流告警并直接丢帧，防止遮挡盲走乱算？",
      ],
    },
    node_publish: {
      title: "LocalizerNode::publishOutputs (锁外快照发布)",
      file: "src/localizer_node.cpp",
      lines: "1555-1667",
      chapter: "ch08",
      contract: "输入：值语义 OutputSnapshot 与时间戳 stamp。输出：广播 map→odom TF、6×6 位姿协方差、多假设与诊断状态。",
      thoughtSteps: [
        "首行检查 active_.load()，一旦刚被 deactivate 就不再发幽灵消息。",
        "若 publish_tf，将 stamp 加上 transform_tolerance（默认 +0.1s）作为 TF 头时间戳，消除高频 Nav2 规划器的外推报错。",
        "填充 PoseWithCovarianceStamped：用 constexpr int kIdx[3] = {0, 1, 5} 映射 6×6 协方差阵，把 yaw 方差精确填进下标 35。",
        "填充 PoseArray 供 RViz 显示所有竞争中的候选位置；打包 DiagnosticArray（OK/WARN/ERROR + KeyValue）发往系统监控。",
      ],
      skeleton: `constexpr int kIdx[3] = {0, 1, 5}; // x=0, y=1, yaw=5 -> [35]
for (int i = 0; i < 3; ++i)
  for (int j = 0; j < 3; ++j)
    pm.pose.covariance[6 * kIdx[i] + kIdx[j]] = snap.out.covariance(i, j);
pose_pub_->publish(pm);`,
      tradeoffs: [
        "为何要多拷贝一个 struct OutputSnapshot 到栈上？保证在调用 rclcpp publish()（内部含中间件锁/回调）时绝不持有用户锁 state_mtx_，杜绝 AB-BA 死锁。",
      ],
      checklist: [
        "2D 航向角方差确认为下标 [35]，z 轴 [14] 严格为 0.0？（test_localizer_node 专项断言）",
      ],
    },
  };

  const BUILD_STEPS = [
    {
      step: "01",
      title: "先立几何与 IO 地基（零 ROS、纯 C++）",
      files: ["include/scan_localizer/se2.hpp", "include/scan_localizer/map_io.hpp", "src/map_io.cpp"],
      why: "先把角度归一化、SE(2) 乘法/逆、Y 轴翻转确定为全仓库唯一契约。",
      test: "ctest -R 'test_se2|test_map_io'",
    },
    {
      step: "02",
      title: "写 MapField 把黑白栅格变连续场",
      files: ["include/scan_localizer/map_field.hpp", "src/map_field.cpp"],
      why: "一旦 EDT/SDF/似然场/swmax_ 通过单测，后续全部匹配器都能共享一份经过验证的几何查询。",
      test: "ctest -R test_map_field",
    },
    {
      step: "03",
      title: "实现匹配器多态族与 B&B 全局搜",
      files: ["include/scan_localizer/matchers.hpp", "src/matchers.cpp", "src/particle_filter.cpp"],
      why: "在统一 Matcher 基类下先把精修 Field-GN、捕获 Correlative、退化协方差与 B&B 跑通，并拿 ParticleFilter 当参照组。",
      test: "ctest -R 'test_matchers|test_particle_filter'",
    },
    {
      step: "04",
      title: "组装 4D EKF + 状态机大脑 LocalizerCore",
      files: ["include/scan_localizer/localizer_core.hpp", "src/localizer_core.cpp"],
      why: "不依赖任何 ROS 头文件即可在内存里测试 100m 走廊、陀螺零偏收敛、门控拦截与绑架跳变保护。",
      test: "ctest -R test_localizer_core",
    },
    {
      step: "05",
      title: "用仿真世界与 E1–E10 基准驱动工程选型",
      files: ["include/scan_localizer/sim_world.hpp", "src/sim_world.cpp", "test/benchmark.cpp"],
      why: "用量化消融实验验证参数预算（例如 scan_subsample=2 耗时减半精度几乎无损），把回归指标写进 CI 门禁。",
      test: "ctest -R 'test_sim_world|test_benchmark_regression'",
    },
    {
      step: "06",
      title: "最后套上 ROS 2 LifecycleNode 外壳",
      files: ["include/scan_localizer/localizer_node.hpp", "src/localizer_node.cpp", "launch/demo.launch.py"],
      why: "算法全绿后再写锁外构场、TF2 外参、Lifecycle 与参数热更新，排查 ROS 问题时不用怀疑底层数学。",
      test: "ctest -R test_localizer_node",
    },
  ];

  global.SLOC_WALKTHROUGH = { CLASSES, FLOWS, CALL_CHAINS, FUNCTIONS, BUILD_STEPS };
})(window);
