// Ch10 hands-on layer: reconstruct key production lines, then verify three exercises.
(function () {
  "use strict";

  const TRACKS = [
    { id: "se2", label: "SE2 / C++20" },
    { id: "eigen", label: "Eigen 特征分解" },
    { id: "concurrency", label: "锁与快照" },
    { id: "lifecycle", label: "Lifecycle" },
  ];

  const STAGES = [
    {
      id: "se2-finite",
      track: "se2",
      title: "1 · 先截断非有限角度",
      file: "include/scan_localizer/se2.hpp",
      lines: "34-39",
      blankLines: [37],
      answer: "return 0.0;",
      tokens: ["return", "0.0"],
      task: "填补 L37：当 std::isfinite(a) 为 false 时，立即返回哪个安全值？只填 return 语句。",
      contract: [
        ["接口", "inline double wrap_angle(double a)"],
        ["输入", "任意 double，包括 NaN / ±inf"],
        ["返回契约", "非有限输入返回 0.0；有限输入最终进入 (-kPi, kPi]"],
        ["不变量", "L36 的提前返回必须先于 std::remainder，不能改成循环"],
      ],
      hint: "NaN 与无穷角没有可靠几何方向。源码选择 0.0，并且立刻返回，不再参与后续运算。",
      success: "正确：非有限角先回落 0.0，避免 remainder 接收 NaN，也避免 while 循环卡死。",
      excerpt: [
        [34, "inline double wrap_angle(double a)"],
        [35, "{"],
        [36, "  if (!std::isfinite(a)) {"],
        [37, "    return 0.0;"],
        [38, "  }"],
        [39, "  double r = std::remainder(a, kTwoPi);"],
      ],
    },
    {
      id: "se2-remainder",
      track: "se2",
      title: "2 · 用一次 remainder 归一化",
      file: "include/scan_localizer/se2.hpp",
      lines: "39-46",
      blankLines: [39],
      answer: "double r = std::remainder(a, kTwoPi);",
      tokens: ["double r", "std::remainder", "kTwoPi"],
      task: "填补 L39：用标准库一次算出 a 对 2π 的余数，结果写入局部变量 r。",
      contract: [
        ["接口", "double wrap_angle(double a)"],
        ["已建立", "kTwoPi = 2.0 * std::numbers::pi（L30-31）"],
        ["性能约束", "有限角必须 O(1)，不能按角度大小逐圈减 2π"],
        ["边界修正", "remainder 后还要处理 -π 与 +π，使区间成为 (-π, π]"],
      ],
      hint: "使用 <cmath> 的 std::remainder；第二参数是已经定义的 kTwoPi，不要重新写 M_PI。",
      success: "正确：std::remainder(a, kTwoPi) 与后续边界分支共同实现 O(1) 归一化。",
      excerpt: [
        [39, "  double r = std::remainder(a, kTwoPi);"],
        [40, "  if (r <= -kPi) {"],
        [41, "    r += kTwoPi;"],
        [42, "  } else if (r > kPi) {"],
        [43, "    r -= kTwoPi;"],
        [44, "  }"],
        [45, "  return r;"],
        [46, "}"],
      ],
    },
    {
      id: "se2-compose",
      track: "se2",
      title: "3 · 重建 SE(2) 位姿复合",
      file: "include/scan_localizer/se2.hpp",
      lines: "57-62",
      blankLines: [60, 61],
      answer: "return {x + c * o.x - s * o.y, y + s * o.x + c * o.y,\n      wrap_angle(theta + o.theta)};",
      tokens: ["x + c * o.x - s * o.y", "y + s * o.x + c * o.y", "wrap_angle(theta + o.theta)"],
      task: "填补 L60-61：this 表示父位姿，o 表示子系位姿。先旋转 o 的平移，再叠加 this 的平移，最后复合航向。",
      contract: [
        ["接口", "Pose2 operator*(const Pose2 & o) const"],
        ["已建立", "c = cos(theta)，s = sin(theta)（L59）"],
        ["返回类型", "Pose2{x, y, theta}"],
        ["约束", "不修改 this 或 o；航向必须再次调用 wrap_angle"],
      ],
      hint: "x/y 都是“父平移 + 父旋转 × 子平移”；第三项是父航向加子航向，而不是直接相加后返回。",
      success: "正确：平移复合与角度归一化都保留了 SE(2) 的群语义。",
      excerpt: [
        [57, "  Pose2 operator*(const Pose2 & o) const     // 本系位姿 o 复合到本位姿"],
        [58, "  {"],
        [59, "    const double c = std::cos(theta), s = std::sin(theta);"],
        [60, "    return {x + c * o.x - s * o.y, y + s * o.x + c * o.y,"],
        [61, "      wrap_angle(theta + o.theta)};"],
        [62, "  }"],
      ],
    },
    {
      id: "se2-concept",
      track: "se2",
      title: "4 · 用 Concept 裁掉无谓预分配",
      file: "include/scan_localizer/se2.hpp",
      lines: "80-93",
      blankLines: [85],
      answer: "if constexpr (std::random_access_iterator<It>) {",
      tokens: ["if constexpr", "std::random_access_iterator<It>"],
      task: "填补 L85：只允许随机访问迭代器进入 reserve 分支，其他迭代器在编译期裁掉该分支。",
      contract: [
        ["接口", "template<typename It> void transform_scan(const Pose2 &, It begin, It end, std::vector<Vec2> & out)"],
        ["语言契约", "CMakeLists.txt:L4-8 固定 C++20；se2.hpp:L22 已包含 <iterator>"],
        ["输入约束", "begin/end 同为 It；out 是可复用缓冲"],
        ["性能约束", "std::list 等非随机访问迭代器不能为 reserve 白走一次 distance"],
      ],
      hint: "这是编译期分支，关键字是 if constexpr；概念名称是 std::random_access_iterator。",
      success: "正确：随机访问迭代器才承担 O(1) distance，前向迭代器仍可正常工作。",
      excerpt: [
        [80, "template<typename It>"],
        [81, "inline void transform_scan("],
        [82, "  const Pose2 & pose, It begin, It end, std::vector<Vec2> & out)"],
        [83, "{"],
        [84, "  out.clear();"],
        [85, "  if constexpr (std::random_access_iterator<It>) {"],
        [86, "    out.reserve(static_cast<std::size_t>(std::distance(begin, end)));"],
        [87, "  }"],
        [88, "  const double c = std::cos(pose.theta), s = std::sin(pose.theta);"],
        [89, "  for (It it = begin; it != end; ++it) {"],
        [90, "    const double px = it->x(), py = it->y();"],
        [91, "    out.emplace_back(pose.x + c * px - s * py, pose.y + s * px + c * py);"],
        [92, "  }"],
        [93, "}"],
      ],
    },
    {
      id: "eigen-symmetrize",
      track: "eigen",
      title: "1 · 先恢复 Hessian 的对称性",
      file: "src/matchers.cpp",
      lines: "70-78",
      blankLines: [77],
      answer: "const Eigen::Matrix3d Hsym = 0.5 * (H + H.transpose());",
      tokens: ["Eigen::Matrix3d Hsym", "H + H.transpose()", "0.5"],
      task: "填补 L77：从 const 引用 H 构造对称矩阵 Hsym，不修改调用方输入。",
      contract: [
        ["接口", "Cov3 cov_from_hessian(const Eigen::Matrix3d & H, double res_var, ...)"],
        ["类型", "Cov3 是 Eigen::Matrix3d 的别名（se2.hpp:L116-117）"],
        ["原因", "自伴特征求解只应对称矩阵；H 可能因数值计算出现微小不对称"],
        ["约束", "结果必须显式求值到新的 Matrix3d，不能原地改写 H"],
      ],
      hint: "对称化的标准写法是 0.5 * (H + H.transpose())，并赋给独立的 Hsym。",
      success: "正确：先把近似 Hessian 对称化，才交给 SelfAdjointEigenSolver。",
      excerpt: [
        [71, "Cov3 cov_from_hessian("],
        [72, "  const Eigen::Matrix3d & H, double res_var,"],
        [73, "  double min_var_xy = 1e-4,                       // (1cm)²"],
        [74, "  double min_var_th = 1.2e-5,                     // (0.2°)²"],
        [75, "  double inflation = 2.0)"],
        [76, "{"],
        [77, "  const Eigen::Matrix3d Hsym = 0.5 * (H + H.transpose());"],
        [78, "  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(Hsym);"],
      ],
    },
    {
      id: "eigen-solver",
      track: "eigen",
      title: "2 · 只分解对称矩阵",
      file: "src/matchers.cpp",
      lines: "77-85",
      blankLines: [78],
      answer: "Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(Hsym);",
      tokens: ["SelfAdjointEigenSolver<Eigen::Matrix3d>", "es(Hsym)"],
      task: "填补 L78：创建求解器并立即对 Hsym 做自伴特征分解。",
      contract: [
        ["输入", "上一步得到的 const Eigen::Matrix3d Hsym"],
        ["API", "Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>"],
        ["失败契约", "下一步必须检查 es.info() != Eigen::Success"],
        ["回退", "失败时 L80-84 返回单位对角协方差，不继续读取特征值"],
      ],
      hint: "矩阵已对称，应使用 SelfAdjointEigenSolver，而不是为一般非对称矩阵设计的 EigenSolver。",
      success: "正确：求解器类型与 Hsym 的实对称维度一致。",
      excerpt: [
        [77, "  const Eigen::Matrix3d Hsym = 0.5 * (H + H.transpose());"],
        [78, "  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(Hsym);"],
        [79, "  if (es.info() != Eigen::Success) {"],
        [80, "    Cov3 fallback = Cov3::Zero();"],
        [81, "    fallback(0, 0) = 1.0;"],
        [82, "    fallback(1, 1) = 1.0;"],
        [83, "    fallback(2, 2) = 1.0;"],
        [84, "    return fallback;"],
        [85, "  }"],
      ],
    },
    {
      id: "eigen-clamp",
      track: "eigen",
      title: "3 · 先夹紧，再求逆",
      file: "src/matchers.cpp",
      lines: "85-93",
      blankLines: [87],
      answer: "const Eigen::Vector3d eigs = es.eigenvalues().cwiseMax(kMinEig);",
      tokens: ["Eigen::Vector3d eigs", "es.eigenvalues()", "cwiseMax(kMinEig)"],
      task: "填补 L87：把每个特征值下限夹到 kMinEig，得到可安全求逆的 eigs。",
      contract: [
        ["已定义", "constexpr double kMinEig = 1e-4（L86）"],
        ["顺序", "先 cwiseMax 夹紧，后 cwiseInverse"],
        ["几何意义", "退化方向保留小信息量，对应更大方差，而不是丢弃该方向"],
        ["类型", "eigs 为 Eigen::Vector3d"],
      ],
      hint: "不要先做 cwiseInverse；零特征值会先爆炸。使用 cwiseMax(kMinEig) 生成夹紧后的向量。",
      success: "正确：先把特征值抬到正下界，避免退化方向除零。",
      excerpt: [
        [86, "  constexpr double kMinEig = 1e-4;"],
        [87, "  const Eigen::Vector3d eigs = es.eigenvalues().cwiseMax(kMinEig);"],
        [88, "  const Eigen::Matrix3d H_inv ="],
        [89, "    es.eigenvectors() * eigs.cwiseInverse().asDiagonal() *"],
        [90, "    es.eigenvectors().transpose();"],
        [91, "  const double scale = std::max(res_var, min_var_xy) * inflation;"],
        [92, "  Cov3 cov = scale * H_inv;"],
        [93, "  cov = 0.5 * (cov + cov.transpose()).eval();"],
      ],
    },
    {
      id: "eigen-reconstruct",
      track: "eigen",
      title: "4 · 用 Q diag(1/λ) Qᵀ 重建逆",
      file: "src/matchers.cpp",
      lines: "88-97",
      blankLines: [88, 89, 90],
      answer: "const Eigen::Matrix3d H_inv =\n    es.eigenvectors() * eigs.cwiseInverse().asDiagonal() *\n    es.eigenvectors().transpose();",
      tokens: ["es.eigenvectors()", "eigs.cwiseInverse().asDiagonal()", "es.eigenvectors().transpose()"],
      task: "填补 L88-90：用特征向量矩阵 Q、夹紧后特征值的逆对角阵，重建 H_inv。",
      contract: [
        ["公式", "H_inv = Q · diag(1 / λ_clamped) · Qᵀ"],
        ["API", "eigs.cwiseInverse().asDiagonal() 生成对角矩阵表达式"],
        ["禁止项", "不调用 H.inverse()；不对原始未夹紧特征值求逆"],
        ["后续", "L91-96 再缩放、对称化并施加 x/y/θ 物理方差下限"],
      ],
      hint: "左边和右边都是 es.eigenvectors()；右边必须转置，中间是 eigs.cwiseInverse().asDiagonal()。",
      success: "正确：特征分解逆保持对称结构，并与前面的特征值夹紧配套。",
      excerpt: [
        [88, "  const Eigen::Matrix3d H_inv ="],
        [89, "    es.eigenvectors() * eigs.cwiseInverse().asDiagonal() *"],
        [90, "    es.eigenvectors().transpose();"],
        [91, "  const double scale = std::max(res_var, min_var_xy) * inflation;"],
        [92, "  Cov3 cov = scale * H_inv;"],
        [93, "  cov = 0.5 * (cov + cov.transpose()).eval();"],
        [94, "  cov(0, 0) = std::max(cov(0, 0), min_var_xy);"],
        [95, "  cov(1, 1) = std::max(cov(1, 1), min_var_xy);"],
        [96, "  cov(2, 2) = std::max(cov(2, 2), min_var_th);"],
        [97, "  return cov;"],
      ],
    },
    {
      id: "lock-config-copy",
      track: "concurrency",
      title: "1 · 锁内只复制构建参数",
      file: "src/localizer_node.cpp",
      lines: "909-918",
      blankLines: [914],
      answer: "std::lock_guard<std::mutex> lk(state_mtx_);",
      tokens: ["std::lock_guard<std::mutex>", "state_mtx_"],
      task: "填补 L914：进入短临界区，复制 sigma、matcher_name、ndt_voxel；锁必须在 L918 前释放。",
      contract: [
        ["接口", "bool LocalizerNode::applyMap(std::shared_ptr<OccupancyGrid> grid, bool keep_pose)"],
        ["保护对象", "成员参数 likelihood_sigma_ / matcher_name_ / ndt_voxel_"],
        ["锁边界", "只复制三个值；MapField::build 与 make_matcher 必须在锁外"],
        ["原因", "建场可能耗时数十毫秒，不能饿死 scan / odom / imu 回调"],
      ],
      hint: "成员 state_mtx_ 是 std::mutex；源码使用显式模板参数的 std::lock_guard<std::mutex>。",
      success: "正确：先把配置复制成线程私有值，随后即可释放锁。",
      excerpt: [
        [909, "  // 读取构建参数副本，在锁外构建 MapField 与 Matcher，避免阻塞高频传感器回调"],
        [910, "  double sigma = 0.20;"],
        [911, "  std::string matcher_name;"],
        [912, "  double ndt_voxel = 0.5;"],
        [913, "  {"],
        [914, "    std::lock_guard<std::mutex> lk(state_mtx_);"],
        [915, "    sigma = likelihood_sigma_;"],
        [916, "    matcher_name = matcher_name_;"],
        [917, "    ndt_voxel = ndt_voxel_;"],
        [918, "  }"],
      ],
    },
    {
      id: "lock-build-outside",
      track: "concurrency",
      title: "2 · 锁外构建新地图资源",
      file: "src/localizer_node.cpp",
      lines: "920-931",
      blankLines: [920],
      answer: "auto field = std::make_shared<MapField>();",
      tokens: ["auto field", "std::make_shared<MapField>"],
      task: "填补 L920：创建新的 MapField 所有者；随后 L921 在锁外 build，L925 在锁外创建 matcher。",
      contract: [
        ["输入", "grid 由调用方持有；sigma / matcher_name / ndt_voxel 是上一步的副本"],
        ["返回契约", "field->build(grid, sigma) 为 false 时直接失败，不发布半成品"],
        ["匹配器接口", "make_matcher(matcher_name, grid.get(), ndt_voxel)"],
        ["并发约束", "整个 L920-931 不得持有 state_mtx_"],
      ],
      hint: "字段成员是 std::shared_ptr<MapField>；源码用 std::make_shared<MapField>() 创建独立新对象。",
      success: "正确：新资源在锁外完整构建，失败时不会污染当前已发布资源。",
      excerpt: [
        [920, "  auto field = std::make_shared<MapField>();"],
        [921, "  if (!field->build(grid, sigma)) {"],
        [922, "    RCLCPP_ERROR(get_logger(), \"applyMap: MapField::build failed\");"],
        [923, "    return false;"],
        [924, "  }"],
        [925, "  auto matcher = make_matcher(matcher_name, grid.get(), ndt_voxel);"],
        [926, "  if (!matcher) {"],
        [927, "    RCLCPP_ERROR("],
        [928, "      get_logger(), \"applyMap: failed to build matcher '%s'\","],
        [929, "      matcher_name.c_str());"],
        [930, "    return false;"],
        [931, "  }"],
      ],
    },
    {
      id: "lock-atomic-swap",
      track: "concurrency",
      title: "3 · 短锁内原子交换一代资源",
      file: "src/localizer_node.cpp",
      lines: "933-953",
      blankLines: [941],
      answer: "field_ = std::move(field);",
      tokens: ["field_", "std::move(field)"],
      task: "填补 L941：把锁外构建好的 field 移动进成员字段，与 grid_ / matcher_ 作为同一代资源发布。",
      contract: [
        ["锁边界", "L934-953 的临界区只做指针交换与少量状态更新"],
        ["配套写入", "L940-942 同时替换 grid_ / field_ / matcher_"],
        ["核心绑定", "L946-947 在同一锁内调用 core_->set_map / set_matcher"],
        ["所有权", "std::move 转移 shared_ptr，不复制地图数据"],
      ],
      hint: "等号左边是成员 field_，右边是局部 shared_ptr；std::move 表示转移所有权。",
      success: "正确：读者要么看到旧一代资源，要么看到完整的新一代资源。",
      excerpt: [
        [933, "  bool first = false;"],
        [934, "  {"],
        [935, "    std::lock_guard<std::mutex> lk(state_mtx_);"],
        [936, "    if (!core_) {"],
        [937, "      return false;"],
        [938, "    }"],
        [939, "    first = !map_received_;"],
        [940, "    grid_ = std::move(grid);"],
        [941, "    field_ = std::move(field);"],
        [942, "    matcher_ = std::move(matcher);"],
        [943, "    pending_hypotheses_.clear();"],
        [944, "    global_alternates_.clear();"],
        [945, "    stable_tracking_count_ = 0;"],
        [946, "    core_->set_map(field_);"],
        [947, "    core_->set_matcher(matcher_);"],
        [948, "    map_received_ = true;"],
      ],
    },
    {
      id: "snapshot-before-publish",
      track: "concurrency",
      title: "4 · 锁内构快照，锁外发布",
      file: "src/localizer_node.cpp",
      lines: "1308-1335",
      blankLines: [1332],
      answer: "snap = buildSnapshotLocked(out, odom_stale);",
      tokens: ["snap", "buildSnapshotLocked(out, odom_stale)"],
      task: "填补 L1332：在 core_->on_scan 之后、离开 state_mtx_ 之前，把输出复制成独立 OutputSnapshot。",
      contract: [
        ["调用前提", "L1310 已持有 state_mtx_；core_ 与 map_received_ 已复核"],
        ["快照内容", "CoreOutput、map_to_odom、帧名、诊断标志与 hypotheses 的副本"],
        ["锁边界", "L1333 离开锁；L1335 才调用 publishOutputs"],
        ["原因", "DDS / TF 发布可能阻塞，不能与算法状态锁串行绑定"],
      ],
      hint: "函数名明确要求调用者持锁：buildSnapshotLocked；两个实参依次是 out 与 odom_stale。",
      success: "正确：发布使用值语义快照，不再读取可能正在变化的成员状态。",
      excerpt: [
        [1308, "  OutputSnapshot snap;"],
        [1309, "  {"],
        [1310, "    std::lock_guard<std::mutex> lk(state_mtx_);"],
        [1311, "    if (!active_.load() || !map_received_ || !core_) {return;}"],
        [1318, "    const CoreOutput out = core_->on_scan(scan);"],
        [1332, "    snap = buildSnapshotLocked(out, odom_stale);"],
        [1333, "  }"],
        [1335, "  publishOutputs(snap, scan_stamp.nanoseconds() == 0 ? now() : scan_stamp);"],
      ],
    },
    {
      id: "lifecycle-autostart",
      track: "lifecycle",
      title: "1 · 把启动权交还 Lifecycle",
      file: "src/localizer_node.cpp",
      lines: "142-148",
      blankLines: [143],
      answer: "declare_parameter<bool>(\"autostart\", false);",
      tokens: ["declare_parameter<bool>", "\"autostart\"", "false"],
      task: "填补 L143：声明 autostart 参数，默认关闭，让 Nav2 lifecycle_manager 决定 configure / activate 时机。",
      contract: [
        ["上下文", "LocalizerNode 构造函数；此时不是 on_configure"],
        ["API", "declare_parameter<bool>(名称, 默认值)"],
        ["默认契约", "生产默认 false；演示 launch 可显式覆写 true"],
        ["状态约束", "默认路径不能绕过 UNCONFIGURED → INACTIVE → ACTIVE"],
      ],
      hint: "参数类型是 bool，名称是 autostart，生产默认值必须是 false。",
      success: "正确：默认构造只声明参数，不偷偷激活节点。",
      excerpt: [
        [142, "  // autostart：默认 false 以适配 Nav2 lifecycle_manager；演示/测试可在 launch 覆写 true"],
        [143, "  declare_parameter<bool>(\"autostart\", false);"],
        [144, ""],
        [145, "  sensor_cb_group_ = create_callback_group("],
        [146, "    rclcpp::CallbackGroupType::MutuallyExclusive);"],
        [147, "  work_cb_group_ = create_callback_group("],
        [148, "    rclcpp::CallbackGroupType::MutuallyExclusive);"],
      ],
    },
    {
      id: "lifecycle-configure",
      track: "lifecycle",
      title: "2 · configure 才创建算法核心",
      file: "src/localizer_node.cpp",
      lines: "701-719",
      blankLines: [714],
      answer: "core_ = std::make_unique<LocalizerCore>(core_cfg_);",
      tokens: ["core_", "std::make_unique<LocalizerCore>", "core_cfg_"],
      task: "填补 L714：在参数校验通过后创建 LocalizerCore；后续回调组、发布者与订阅都在 configure 建立。",
      contract: [
        ["接口", "CallbackReturn on_configure(const rclcpp_lifecycle::State &) override"],
        ["失败路径", "readAndValidateParameters() 失败时 L705 返回 CallbackReturn::FAILURE"],
        ["所有权", "core_ 是 std::unique_ptr<LocalizerCore>"],
        ["状态约束", "创建资源不等于激活；LifecyclePublisher 仍保持 inactive"],
      ],
      hint: "成员 core_ 持有 unique_ptr；构造参数是已经校验的 core_cfg_。",
      success: "正确：configure 建立可复用资源，但不提前打开数据流。",
      excerpt: [
        [701, "LocalizerNode::CallbackReturn LocalizerNode::on_configure("],
        [702, "  const rclcpp_lifecycle::State &)"],
        [703, "{"],
        [704, "  if (!readAndValidateParameters()) {"],
        [705, "    return CallbackReturn::FAILURE;"],
        [706, "  }"],
        [712, "  {"],
        [713, "    std::lock_guard<std::mutex> lk(state_mtx_);"],
        [714, "    core_ = std::make_unique<LocalizerCore>(core_cfg_);"],
        [715, "    core_->set_global_fn("],
        [719, "    fine_on_ = false;"],
      ],
    },
    {
      id: "lifecycle-activate",
      track: "lifecycle",
      title: "3 · activate 先开发布者",
      file: "src/localizer_node.cpp",
      lines: "852-861",
      blankLines: [855],
      answer: "if (pose_pub_) {pose_pub_->on_activate();}",
      tokens: ["pose_pub_", "on_activate()"],
      task: "填补 L855：只有 pose_pub_ 非空时才激活它；L856-857 对另外两个 LifecyclePublisher 做同样处理。",
      contract: [
        ["接口", "CallbackReturn on_activate(const rclcpp_lifecycle::State &) override"],
        ["顺序", "先激活三个 LifecyclePublisher，再 L858 将 active_ 置 true"],
        ["返回值", "成功返回 CallbackReturn::SUCCESS"],
        ["对称性", "deactivate 在 L866-869 先关 active_，再停用三个发布者"],
      ],
      hint: "先判空 pose_pub_，再调用它自己的 on_activate()；不要调用节点的 on_activate 递归自己。",
      success: "正确：LifecyclePublisher 的激活与节点 active_ 标志按生产顺序完成。",
      excerpt: [
        [852, "LocalizerNode::CallbackReturn LocalizerNode::on_activate("],
        [853, "  const rclcpp_lifecycle::State &)"],
        [854, "{"],
        [855, "  if (pose_pub_) {pose_pub_->on_activate();}"],
        [856, "  if (status_pub_) {status_pub_->on_activate();}"],
        [857, "  if (hypotheses_pub_) {hypotheses_pub_->on_activate();}"],
        [858, "  active_.store(true);"],
        [859, "  RCLCPP_INFO(get_logger(), \"activated\");"],
        [860, "  return CallbackReturn::SUCCESS;"],
        [861, "}"],
      ],
    },
    {
      id: "lifecycle-cleanup",
      track: "lifecycle",
      title: "4 · cleanup 必须完整回收",
      file: "src/localizer_node.cpp",
      lines: "874-880",
      blankLines: [877],
      answer: "resetResources();",
      tokens: ["resetResources()"],
      task: "填补 L877：调用统一回收入口，保证再次 configure 时不会继承旧地图、旧扫描或旧 IMU 锚点。",
      contract: [
        ["接口", "CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override"],
        ["统一入口", "resetResources()（实现 L644-699）"],
        ["回收范围", "active_、定时器、参数回调、订阅、服务、发布者、TF 与算法状态"],
        ["复用契约", "on_shutdown 也调用同一入口；指针与 shared_ptr 允许重复 reset"],
      ],
      hint: "只写类内已经存在的私有方法调用；它不是 rclcpp::Node::reset，而是本节点的 resetResources()。",
      success: "正确：cleanup 走完整、幂等的资源回收，而不是只翻一个布尔标志。",
      excerpt: [
        [874, "LocalizerNode::CallbackReturn LocalizerNode::on_cleanup("],
        [875, "  const rclcpp_lifecycle::State &)"],
        [876, "{"],
        [877, "  resetResources();"],
        [878, "  RCLCPP_INFO(get_logger(), \"cleaned up\");"],
        [879, "  return CallbackReturn::SUCCESS;"],
        [880, "}"],
      ],
    },
  ];

  const SOURCE_PREFIX = "../../../src/scan_localizer/";

  function normalizeCode(value) {
    return String(value || "")
      .trim()
      .replace(/[“”]/g, "\"")
      .replace(/[‘’]/g, "'")
      .replace(/\s+/g, " ")
      .replace(/\s*([(){}\[\],;.<>])\s*/g, "$1")
      .replace(/\s*(::|->|\+=|-=|=|\+|\-|\*|\/)\s*/g, "$1")
      .replace(/;+$/, "");
  }

  function normalizeExercise(value) {
    return String(value || "")
      .trim()
      .toLowerCase()
      .replace(/\s+/g, "")
      .replace(/^\(/, "")
      .replace(/\)$/, "");
  }

  function fillVars(host, rows) {
    if (!host) return;
    host.textContent = "";
    const fragment = document.createDocumentFragment();
    rows.forEach(([key, value]) => {
      const cell = document.createElement("div");
      cell.className = "hands-on-var";
      const dt = document.createElement("dt");
      const dd = document.createElement("dd");
      dt.textContent = key;
      dd.textContent = value;
      cell.appendChild(dt);
      cell.appendChild(dd);
      fragment.appendChild(cell);
    });
    host.appendChild(fragment);
  }

  function init() {
    if (window.PAGE_ID !== "ch10") return;

    const tabList = document.getElementById("ch10-rebuild-tracks");
    const panel = document.getElementById("ch10-rebuild-panel");
    const title = document.getElementById("ch10-rebuild-title");
    const meta = document.getElementById("ch10-rebuild-meta");
    const explain = document.getElementById("ch10-rebuild-explain");
    const contract = document.getElementById("ch10-rebuild-contract");
    const stageCode = document.getElementById("ch10-rebuild-code");
    const sourceLabel = document.getElementById("ch10-rebuild-source-label");
    const sourceBtn = document.getElementById("ch10-rebuild-source");
    const sourceNote = document.getElementById("ch10-rebuild-source-note");
    const stageIndex = document.getElementById("ch10-rebuild-index");
    const stagePrev = document.getElementById("ch10-rebuild-prev");
    const stageNext = document.getElementById("ch10-rebuild-next");
    const copyBtn = document.getElementById("ch10-rebuild-copy");
    const hintBtn = document.getElementById("ch10-rebuild-hint");
    const resetBtn = document.getElementById("ch10-rebuild-reset");
    const form = document.getElementById("ch10-rebuild-form");
    const input = document.getElementById("ch10-rebuild-input");
    const feedback = document.getElementById("ch10-rebuild-feedback");
    const progress = document.getElementById("ch10-rebuild-progress");
    const acceptanceVars = document.getElementById("ch10-acceptance-vars");
    const acceptanceStatus = document.getElementById("ch10-acceptance-status");

    if (!tabList || !panel || !title || !meta || !explain || !contract ||
        !stageCode || !sourceLabel || !sourceBtn || !sourceNote || !stageIndex ||
        !stagePrev || !stageNext || !copyBtn || !hintBtn || !resetBtn ||
        !form || !input || !feedback || !progress || !acceptanceVars ||
        !acceptanceStatus) {
      return;
    }

    let currentTrack = TRACKS[0].id;
    let currentStage = 0;
    const drafts = new Map();
    const solvedStages = new Set();
    const solvedExercises = new Set();
    const sources = new Map();
    const sourceState = new Map();
    const exerciseForms = Array.from(document.querySelectorAll(".hands-on-exercise[data-exercise]"));

    function trackStages(trackId) {
      return STAGES.filter((stage) => stage.track === trackId);
    }

    function current() {
      return trackStages(currentTrack)[currentStage];
    }

    function sourceRows(stage) {
      const start = Number(String(stage.lines).split("-")[0]);
      const end = Number(String(stage.lines).split("-")[1] || start);
      const fileLines = sources.get(stage.file);
      if (sourceState.get(stage.file) === "live" && fileLines && fileLines.length >= end) {
        const rows = [];
        for (let line = start; line <= end; line += 1) {
          rows.push([line, fileLines[line - 1]]);
        }
        return rows;
      }
      return stage.excerpt;
    }

    function codeText(stage) {
      const solved = solvedStages.has(stage.id);
      const blankSet = new Set(stage.blankLines);
      let blankWritten = false;
      const rows = [];
      sourceRows(stage).forEach(([line, sourceText]) => {
        if (blankSet.has(line)) {
          if (blankWritten) return;
          blankWritten = true;
          const replacement = solved ? stage.answer : "/* 重建这一行 */";
          const parts = replacement.split("\n");
          rows.push(`L${String(line).padStart(4, "0")} | ${parts[0]}`);
          for (let i = 1; i < parts.length; i += 1) {
            rows.push(`        | ${parts[i]}`);
          }
          return;
        }
        rows.push(`L${String(line).padStart(4, "0")} | ${sourceText}`);
      });
      return rows.join("\n");
    }

    function highlightStageCode() {
      stageCode.className = "language-cpp";
      delete stageCode.dataset.highlighted;
      if (window.hljs) {
        try { window.hljs.highlightElement(stageCode); } catch (_) {}
      }
    }

    function renderCode() {
      stageCode.textContent = codeText(current());
      highlightStageCode();
    }

    function updateSourceNote() {
      const state = sourceState.get(current().file);
      if (state === "live") {
        sourceNote.textContent = "片段按当前仓库文件实时载入；左侧数字就是 1-based 源码行号。";
      } else if (state === "loading") {
        sourceNote.textContent = "正在读取真实源码；读取完成前显示随页保存的已核对片段。";
      } else {
        sourceNote.textContent = "当前使用随页保存的真实行片段；行号不连续处仅省略上下文。点“真实源码”可查看完整区间。";
      }
    }

    function updateProgress() {
      let completedTracks = 0;
      TRACKS.forEach((track) => {
        const stages = trackStages(track.id);
        const count = stages.filter((stage) => solvedStages.has(stage.id)).length;
        if (count === stages.length) completedTracks += 1;
        const button = document.getElementById(`ch10-track-${track.id}`);
        if (button) {
          button.textContent = `${count === stages.length ? "✓ " : ""}${track.label} · ${count}/${stages.length}`;
          button.setAttribute("aria-label", `${track.label}，已完成 ${count}/${stages.length} 个片段`);
        }
      });

      const exerciseTotal = exerciseForms.length;
      const exerciseCount = solvedExercises.size;
      progress.textContent = `关键行 ${solvedStages.size}/${STAGES.length} · 完整方向 ${completedTracks}/${TRACKS.length} · 练习 ${exerciseCount}/${exerciseTotal}`;
      fillVars(acceptanceVars, [
        ["关键行", `${solvedStages.size} / ${STAGES.length}`],
        ["完整方向", `${completedTracks} / ${TRACKS.length}`],
        ["练习", `${exerciseCount} / ${exerciseTotal}`],
      ]);

      const passed = solvedStages.size === STAGES.length && exerciseCount === exerciseTotal;
      acceptanceStatus.className = `exercise-feedback${passed ? " ok" : ""}`;
      acceptanceStatus.textContent = passed
        ? "自动验收通过：16 个关键行与 3 道练习均完成。再口头复述下方四条边界，即可完成本章。"
        : `尚未通过：还需完成 ${STAGES.length - solvedStages.size} 个关键行、${Math.max(0, exerciseTotal - exerciseCount)} 道练习。`;
    }

    function evaluateStage(stage, raw, live) {
      const compact = normalizeCode(raw);
      if (!compact) {
        solvedStages.delete(stage.id);
        feedback.className = "exercise-feedback";
        feedback.textContent = live ? "填写关键行后自动校验；也可按 Enter 提交。" : "先填写这一行，再检查。";
        input.setAttribute("aria-invalid", "false");
        renderCode();
        updateProgress();
        return false;
      }

      const accepted = (stage.accepts || [stage.answer]).some((answer) =>
        normalizeCode(answer) === compact);
      if (accepted) {
        solvedStages.add(stage.id);
        drafts.set(stage.id, stage.answer);
        feedback.className = "exercise-feedback ok";
        feedback.textContent = stage.success;
        input.setAttribute("aria-invalid", "false");
        renderCode();
        updateProgress();
        return true;
      }

      solvedStages.delete(stage.id);
      const missing = stage.tokens.filter((token) => !compact.includes(normalizeCode(token)));
      const detail = missing.length
        ? `还缺关键结构：${missing.slice(0, 3).join("、")}。`
        : "关键token已出现，但顺序、类型或返回值还未与生产行完全一致。";
      feedback.className = live ? "exercise-feedback" : "exercise-feedback bad";
      feedback.textContent = live ? `继续补全。${detail}` : `未通过。${detail}${stage.hint}`;
      input.setAttribute("aria-invalid", live ? "false" : "true");
      renderCode();
      updateProgress();
      return false;
    }

    function render() {
      const stage = current();
      title.textContent = stage.title;
      meta.textContent = `${stage.file}:L${stage.lines} · ${currentTrack} ${currentStage + 1}/${trackStages(currentTrack).length}`;
      explain.textContent = stage.task;
      fillVars(contract, stage.contract);
      sourceLabel.textContent = `${stage.file} · L${stage.lines}`;
      sourceBtn.setAttribute("data-file", stage.file);
      sourceBtn.setAttribute("data-lines", stage.lines);
      sourceBtn.setAttribute("data-label", `Ch10 重建：${stage.title}`);
      stageIndex.textContent = `${currentStage + 1} / ${trackStages(currentTrack).length}`;
      stagePrev.disabled = currentStage === 0;
      stageNext.disabled = currentStage === trackStages(currentTrack).length - 1;
      panel.setAttribute("aria-labelledby", `ch10-track-${currentTrack}`);
      form.dataset.stage = stage.id;

      const draft = drafts.get(stage.id) || "";
      input.value = draft;
      input.setAttribute("aria-invalid", "false");
      if (solvedStages.has(stage.id)) {
        feedback.className = "exercise-feedback ok";
        feedback.textContent = stage.success;
      } else if (draft) {
        evaluateStage(stage, draft, true);
      } else {
        feedback.className = "exercise-feedback";
        feedback.textContent = "填写关键行后自动校验；也可按 Enter 提交。";
      }
      renderCode();
      updateSourceNote();
      updateProgress();
    }

    function selectTrack(trackId) {
      if (!TRACKS.some((track) => track.id === trackId)) return;
      drafts.set(current().id, input.value);
      currentTrack = trackId;
      currentStage = 0;
      document.querySelectorAll("#ch10-rebuild-tracks button").forEach((button) => {
        const active = button.dataset.track === trackId;
        button.classList.toggle("active", active);
        button.setAttribute("aria-selected", active ? "true" : "false");
        button.tabIndex = active ? 0 : -1;
      });
      render();
    }

    tabList.addEventListener("click", (event) => {
      const button = event.target.closest("button[data-track]");
      if (button) selectTrack(button.dataset.track);
    });

    tabList.addEventListener("keydown", (event) => {
      if (event.key !== "ArrowLeft" && event.key !== "ArrowRight") return;
      event.preventDefault();
      const index = TRACKS.findIndex((track) => track.id === currentTrack);
      const delta = event.key === "ArrowRight" ? 1 : -1;
      const next = TRACKS[(index + delta + TRACKS.length) % TRACKS.length];
      selectTrack(next.id);
      document.getElementById(`ch10-track-${next.id}`).focus();
    });

    stagePrev.addEventListener("click", () => {
      if (currentStage === 0) return;
      drafts.set(current().id, input.value);
      currentStage -= 1;
      render();
    });

    stageNext.addEventListener("click", () => {
      if (currentStage >= trackStages(currentTrack).length - 1) return;
      drafts.set(current().id, input.value);
      currentStage += 1;
      render();
    });

    input.addEventListener("input", () => {
      drafts.set(current().id, input.value);
      evaluateStage(current(), input.value, true);
    });

    form.addEventListener("submit", (event) => {
      event.preventDefault();
      drafts.set(current().id, input.value);
      evaluateStage(current(), input.value, false);
    });

    hintBtn.addEventListener("click", () => {
      feedback.className = "exercise-feedback";
      feedback.textContent = current().hint;
    });

    copyBtn.addEventListener("click", () => {
      if (navigator.clipboard) {
        navigator.clipboard.writeText(stageCode.textContent).catch(() => {});
      }
      const oldText = copyBtn.textContent;
      copyBtn.textContent = "已复制";
      setTimeout(() => { copyBtn.textContent = oldText; }, 1200);
    });

    exerciseForms.forEach((exerciseForm) => {
      const exerciseInput = exerciseForm.querySelector(".exercise-input");
      const exerciseFeedback = exerciseForm.querySelector(".exercise-feedback");
      if (!exerciseInput || !exerciseFeedback) return;
      const exerciseId = exerciseForm.dataset.exercise;

      exerciseInput.addEventListener("input", () => {
        if (solvedExercises.delete(exerciseId)) updateProgress();
        exerciseFeedback.className = "exercise-feedback";
        exerciseFeedback.textContent = "已修改答案，请重新检查。";
        exerciseInput.setAttribute("aria-invalid", "false");
      });

      exerciseForm.addEventListener("submit", (event) => {
        event.preventDefault();
        const raw = normalizeExercise(exerciseInput.value);
        const acceptedAnswers = [exerciseForm.dataset.answer || ""];
        if (exerciseForm.dataset.accepts) {
          acceptedAnswers.push(...exerciseForm.dataset.accepts.split("|"));
        }
        const ok = raw !== "" && acceptedAnswers.some((answer) =>
          normalizeExercise(answer) === raw);
        if (ok) solvedExercises.add(exerciseId);
        else solvedExercises.delete(exerciseId);
        exerciseFeedback.className = `exercise-feedback${ok ? " ok" : " bad"}`;
        exerciseFeedback.textContent = ok
          ? (exerciseForm.dataset.success || "正确。")
          : (exerciseForm.dataset.hint || "还不对，回到接口与约束再检查。");
        exerciseInput.setAttribute("aria-invalid", ok ? "false" : "true");
        updateProgress();
      });
    });

    resetBtn.addEventListener("click", () => {
      drafts.clear();
      solvedStages.clear();
      solvedExercises.clear();
      exerciseForms.forEach((exerciseForm) => {
        const exerciseInput = exerciseForm.querySelector(".exercise-input");
        const exerciseFeedback = exerciseForm.querySelector(".exercise-feedback");
        if (exerciseInput) {
          exerciseInput.value = "";
          exerciseInput.setAttribute("aria-invalid", "false");
        }
        if (exerciseFeedback) {
          exerciseFeedback.className = "exercise-feedback";
          exerciseFeedback.textContent = "";
        }
      });
      input.value = "";
      selectTrack(TRACKS[0].id);
    });

    function loadSources() {
      const files = Array.from(new Set(STAGES.map((stage) => stage.file)));
      files.forEach((file) => {
        sourceState.set(file, "loading");
        fetch(SOURCE_PREFIX + file, { cache: "no-store" })
          .then((response) => {
            if (!response.ok) throw new Error(`HTTP ${response.status}`);
            return response.text();
          })
          .then((text) => {
            sources.set(file, text.split(/\r?\n/));
            sourceState.set(file, "live");
          })
          .catch(() => {
            sourceState.set(file, "fallback");
          })
          .then(() => {
            renderCode();
            updateSourceNote();
          });
      });
    }

    render();
    loadSources();
  }

  window.addEventListener("sloc:ready", init);
})();
