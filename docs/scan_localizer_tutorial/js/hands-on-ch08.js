// Ch08 hands-on layer: lifecycle ownership, concurrency timeline, source stepper and exercises.
(function () {
  "use strict";

  const STAGES = [
    {
      title: "1 · configure：先校验，再创建一致的核心状态",
      lines: "701-732",
      explain: "参数校验失败时还没有创建新资源，直接返回 FAILURE。通过后才在短临界区里创建 LocalizerCore，并清零 IMU 锚、odom 缓冲、上一帧扫描与候选队列；这些状态不能从上一轮 configure 残留。",
      code: `CallbackReturn LocalizerNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!readAndValidateParameters()) {
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(
    get_logger(), "configuring scan_localizer (matcher=%s)",
    matcher_name_.c_str());

  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    core_ = std::make_unique<LocalizerCore>(core_cfg_);
    core_->set_global_fn(
      [this](const Scan & s, Pose2 & out, double & sc) {
        return use_global_search_ && runGlobalSearchLocked(s, out, sc);
      });
    fine_on_ = false;
    map_received_ = false;
    stable_tracking_count_ = 0;
    imu_yaw_ = 0.0;
    imu_started_ = false;
    last_imu_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    odom_buffer_.clear();
    has_odom_sample_ = false;
    last_odom_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_scan_.points.clear();
    has_last_scan_ = false;
    pending_hypotheses_.clear();
    global_alternates_.clear();
  }

  // ... L734-832：创建发布器、TF、订阅、服务与参数回调 ...`,
      facts: [
        ["state_mtx_", "只保护核心与算法状态初始化"],
        ["失败路径", "参数非法 → FAILURE，保持 UNCONFIGURED"],
        ["为什么", "资源所有权必须跟 Lifecycle 状态一致"],
        ["回归", "LifecycleTransitionsAndCleanupSafety"],
      ],
    },
    {
      title: "2 · 地图加载失败要回滚；activate 才打开发布器",
      lines: "834-872",
      explain: "configure 已经创建资源后，map_yaml 仍是可能失败的外部输入。失败必须调用 resetResources()，不能返回 INACTIVE 留一堆半初始化资源。activate 只打开发布器并置 active_，不重新建核心。",
      code: `// on_configure() 尾部
if (!map_yaml_.empty()) {
  if (!loadMapFromYaml(map_yaml_)) {
    RCLCPP_ERROR(
      get_logger(), "map_yaml load FAILED: %s", map_yaml_.c_str());
    resetResources();
    return CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(get_logger(), "map loaded from yaml: %s", map_yaml_.c_str());
}
return CallbackReturn::SUCCESS;

CallbackReturn LocalizerNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (pose_pub_) {pose_pub_->on_activate();}
  if (status_pub_) {status_pub_->on_activate();}
  if (hypotheses_pub_) {hypotheses_pub_->on_activate();}
  active_.store(true);
  RCLCPP_INFO(get_logger(), "activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LocalizerNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  if (pose_pub_) {pose_pub_->on_deactivate();}
  if (status_pub_) {status_pub_->on_deactivate();}
  if (hypotheses_pub_) {hypotheses_pub_->on_deactivate();}
  RCLCPP_INFO(get_logger(), "deactivated");
  return CallbackReturn::SUCCESS;
}`,
      facts: [
        ["active_", "activate 后 true；deactivate 先置 false"],
        ["发布器", "LifecyclePublisher 必须显式 on_activate/on_deactivate"],
        ["失败路径", "地图加载失败 → resetResources() → FAILURE"],
        ["回归", "LifecycleTransitionsAndCleanupSafety"],
      ],
    },
    {
      title: "3 · cleanup 与 shutdown 共用同一条清理路径",
      lines: "863-888",
      explain: "deactivate 保留核心与地图，方便再次 activate；cleanup 才回到 UNCONFIGURED 并销毁资源。两者语义不同，不能把 deactivate 写成 resetResources()。shutdown 是最终退出，同样复用清理函数。",
      code: `CallbackReturn LocalizerNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  resetResources();
  RCLCPP_INFO(get_logger(), "cleaned up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LocalizerNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  resetResources();
  RCLCPP_INFO(get_logger(), "shut down");
  return CallbackReturn::SUCCESS;
}`,
      facts: [
        ["deactivate", "停止输出，保留核心/地图/TF 缓冲"],
        ["cleanup", "释放资源，状态机回到 UNCONFIGURED"],
        ["shutdown", "最终释放，进入 FINALIZED"],
        ["设计", "清理逻辑单点复用，避免状态鬼影"],
      ],
    },
    {
      title: "4 · resetResources：先关闸门，再销毁状态",
      lines: "644-699",
      explain: "active_ 先置 false，随后取消定时器、移除参数回调并重置 ROS 实体，最后在 state_mtx_ 内销毁 core/field/grid/matcher。imu_started_、odom_buffer_、last_scan_ 与候选队列必须归零，否则 cleanup 后重新 configure 会继承旧锚。",
      code: `void LocalizerNode::resetResources()
{
  active_.store(false);

  if (autostart_timer_) {
    autostart_timer_->cancel();
    autostart_timer_.reset();
  }
  if (tf_keepalive_timer_) {
    tf_keepalive_timer_->cancel();
    tf_keepalive_timer_.reset();
  }
  if (param_cb_handle_) {
    remove_on_set_parameters_callback(param_cb_handle_.get());
    param_cb_handle_.reset();
  }

  map_sub_.reset();
  scan_sub_.reset();
  odom_sub_.reset();
  imu_sub_.reset();
  // ... 其余订阅、服务、发布器与 TF ...

  std::lock_guard<std::mutex> lk(state_mtx_);
  core_.reset();
  field_.reset();
  grid_.reset();
  matcher_.reset();
  map_received_ = false;
  fine_on_ = false;
  stable_tracking_count_ = 0;
  imu_yaw_ = 0.0;
  imu_started_ = false;
  last_imu_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  odom_buffer_.clear();
  has_odom_sample_ = false;
  last_odom_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  last_scan_.points.clear();
  has_last_scan_ = false;
  pending_hypotheses_.clear();
  global_alternates_.clear();
}`,
      facts: [
        ["闸门", "active_.store(false) 先于资源销毁"],
        ["锁内", "core/field/grid/matcher 与算法缓冲"],
        ["典型残留", "imu_started_ 未清零 → 二次激活航向跳变"],
        ["回归", "ImuSensorDataQosNegativeDtAndCleanupReset"],
      ],
    },
    {
      title: "5 · applyMap：坏地图在构建前就被拒绝",
      lines: "890-907",
      explain: "空指针、非正尺寸、非正分辨率和胞元数不匹配都在锁外检查。这样既不占用状态锁，也不会让一个明显坏的地图进入昂贵的 MapField 构建。",
      code: `bool LocalizerNode::applyMap(
  std::shared_ptr<OccupancyGrid> grid, bool keep_pose)
{
  if (!grid || grid->width <= 0 || grid->height <= 0 ||
    !isFinitePositive(grid->resolution))
  {
    RCLCPP_ERROR(get_logger(), "applyMap: invalid grid dimensions/resolution");
    return false;
  }
  const std::size_t expected_cells =
    static_cast<std::size_t>(grid->width) *
    static_cast<std::size_t>(grid->height);
  if (grid->cells.size() != expected_cells) {
    RCLCPP_ERROR(
      get_logger(), "applyMap: cells size %zu != %zu",
      grid->cells.size(), expected_cells);
    return false;
  }

  // ... L909：复制构建参数 ...`,
      facts: [
        ["state_mtx_", "未持有"],
        ["输入不变量", "width/height/resolution/cells.size 一致"],
        ["失败结果", "返回 false，旧地图不受影响"],
        ["为什么", "廉价校验先于昂贵分配与计算"],
      ],
    },
    {
      title: "6 · applyMap：参数副本 + 锁外构建",
      lines: "909-931",
      explain: "第一次短锁只复制 likelihood_sigma、matcher_name 和 ndt_voxel。MapField::build 与 make_matcher 都在锁外；任一步失败，局部 shared_ptr 销毁，旧 grid_/field_/matcher_ 从未被修改。",
      code: `// 读取构建参数副本，在锁外构建 MapField 与 Matcher
double sigma = 0.20;
std::string matcher_name;
double ndt_voxel = 0.5;
{
  std::lock_guard<std::mutex> lk(state_mtx_);
  sigma = likelihood_sigma_;
  matcher_name = matcher_name_;
  ndt_voxel = ndt_voxel_;
}

auto field = std::make_shared<MapField>();
if (!field->build(grid, sigma)) {
  RCLCPP_ERROR(get_logger(), "applyMap: MapField::build failed");
  return false;
}
auto matcher = make_matcher(matcher_name, grid.get(), ndt_voxel);
if (!matcher) {
  RCLCPP_ERROR(
    get_logger(),
    "applyMap: failed to build matcher '%s'",
    matcher_name.c_str());
  return false;
}`,
      facts: [
        ["锁内工作", "复制 3 个构建参数"],
        ["锁外工作", "MapField::build + make_matcher"],
        ["直接受益者", "sensor_cb_group_ 的 odom/IMU 短回调"],
        ["失败不变量", "旧地图继续服务"],
      ],
    },
    {
      title: "7 · applyMap：同一临界区提交整套地图",
      lines: "933-953",
      explain: "构建完成后才再次加锁。grid、field、matcher、核心指针、候选队列和 map_received_ 在同一临界区提交；读取者不会看到新场配旧匹配器。非首图且 keep_pose=false 时才请求全局重定位。",
      code: `bool first = false;
{
  std::lock_guard<std::mutex> lk(state_mtx_);
  if (!core_) {
    return false;
  }
  first = !map_received_;
  grid_ = std::move(grid);
  field_ = std::move(field);
  matcher_ = std::move(matcher);
  pending_hypotheses_.clear();
  global_alternates_.clear();
  stable_tracking_count_ = 0;
  core_->set_map(field_);
  core_->set_matcher(matcher_);
  map_received_ = true;

  if (!first && !keep_pose) {
    core_->request_global();
  }
}`,
      facts: [
        ["state_mtx_", "只覆盖指针/状态交换"],
        ["逻辑原子", "同一 mutex 保护整套地图视图"],
        ["首图", "first=true，不因切图直接请求 GLOBAL"],
        ["热切", "!first && !keep_pose → request_global()"],
      ],
    },
    {
      title: "8 · onScan：入口闸门、配置副本与扫描时间戳",
      lines: "1189-1215",
      explain: "active_ 是廉价原子预检；没有地图或核心则在锁内返回。配置被复制为局部变量后，scan_stamp 固定为消息头时间戳。后续外参、odom 与输出都必须围绕这个时间锚，而不是回调运行到哪一步。",
      code: `void LocalizerNode::onScan(
  const sensor_msgs::msg::LaserScan::SharedPtr m)
{
  if (!active_.load() || !m) {return;}

  std::string base_frame;
  int subsample = 1;
  int min_pts = 10;
  double range_max_override = 0.0;
  double tf_timeout = 0.05;
  double lx = 0.0;
  double ly = 0.0;
  double lth = 0.0;
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (!map_received_ || !core_) {return;}
    base_frame = base_frame_;
    subsample = scan_subsample_;
    min_pts = min_scan_points_;
    range_max_override = range_max_;
    tf_timeout = tf_lookup_timeout_;
    lx = laser_x_;
    ly = laser_y_;
    lth = laser_theta_;
  }

  const rclcpp::Time scan_stamp(m->header.stamp);`,
      facts: [
        ["原子预检", "active_.load() 不替代 state_mtx_"],
        ["配置副本", "避免 TF/预处理期间反复读取可变参数"],
        ["时间锚", "scan_stamp = m->header.stamp"],
        ["失败路径", "未配置/无地图直接返回"],
      ],
    },
    {
      title: "9 · 外参 TF：按扫描时刻保留完整 3D 变换",
      lines: "1217-1258",
      explain: "base→laser 使用 scan_stamp 查询，倒装雷达的 roll=π 因此能正确翻转点云。只有 ExtrapolationException 才回退 latest；其他 TransformException 退回 2D 外参参数。四元数近零时生产代码把旋转退化为单位阵，但仍保留平移并使用 3D 分支。",
      code: `bool use_3d_tf = false;
Eigen::Matrix3d R_bl = Eigen::Matrix3d::Identity();
Eigen::Vector3d t_bl = Eigen::Vector3d::Zero();
if (tf_buffer_ && !m->header.frame_id.empty() &&
  m->header.frame_id != base_frame)
{
  try {
    const tf2::TimePoint query =
      scan_stamp.nanoseconds() == 0 ?
      tf2::TimePointZero :
      tf2::TimePoint(std::chrono::nanoseconds(scan_stamp.nanoseconds()));
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        base_frame, m->header.frame_id, query,
        tf2::durationFromSec(tf_timeout));
    } catch (const tf2::ExtrapolationException &) {
      tf = tf_buffer_->lookupTransform(
        base_frame, m->header.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout));
    }
    Eigen::Quaterniond q(
      tf.transform.rotation.w,
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z);
    if (q.norm() > 1e-9) {
      q.normalize();
    } else {
      q = Eigen::Quaterniond::Identity();
    }
    R_bl = q.toRotationMatrix();
    t_bl = Eigen::Vector3d(
      tf.transform.translation.x,
      tf.transform.translation.y,
      tf.transform.translation.z);
    use_3d_tf = true;
  } catch (const tf2::TransformException &) {
    use_3d_tf = false;
  }
}`,
      facts: [
        ["精确时刻", "lookupTransform(..., scan_stamp, ...)"],
        ["特殊回退", "仅 ExtrapolationException → latest"],
        ["一般回退", "2D laser_x/laser_y/laser_theta"],
        ["回归", "InvertedLidarTfRollPi"],
      ],
    },
    {
      title: "10 · lookupOdomAt：TF 优先，缓冲兜底并标记 stale",
      lines: "1113-1168",
      explain: "odom→base 也查 scan_stamp。TF 异常后才进入 /odom 环形缓冲。缓冲为空返回 false；扫描早于前端或晚于后端时使用端点样本。只有扫描明显晚于末端并超过 odom_timeout 时才置 stale，生产代码仍返回估计而不是直接丢帧。",
      code: `bool LocalizerNode::lookupOdomAt(
  const rclcpp::Time & stamp, Pose2 & out, bool & stale)
{
  stale = false;
  std::string odom_frame;
  std::string base_frame;
  double timeout = 0.05;
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    odom_frame = odom_frame_;
    base_frame = base_frame_;
    timeout = tf_lookup_timeout_;
  }

  if (tf_buffer_) {
    try {
      const tf2::TimePoint query =
        stamp.nanoseconds() == 0 ?
        tf2::TimePointZero :
        tf2::TimePoint(std::chrono::nanoseconds(stamp.nanoseconds()));
      const auto tf = tf_buffer_->lookupTransform(
        odom_frame, base_frame, query, tf2::durationFromSec(timeout));
      out = Pose2(
        tf.transform.translation.x,
        tf.transform.translation.y,
        quatToYaw(tf.transform.rotation));
      return true;
    } catch (const tf2::TransformException &) {
      // 回退至 /odom 环形缓冲插值
    }
  }

  std::lock_guard<std::mutex> lk(state_mtx_);
  if (odom_buffer_.empty()) {
    return false;
  }
  if (stamp.nanoseconds() > 0 &&
    (stamp - odom_buffer_.back().stamp).seconds() > odom_timeout_)
  {
    stale = true;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "odom is stale by %.3f s (timeout=%.2f s)",
      (stamp - odom_buffer_.back().stamp).seconds(), odom_timeout_);
  }

  if (odom_buffer_.size() == 1 || stamp <= odom_buffer_.front().stamp) {
    out = odom_buffer_.front().pose;
    return true;
  }
  if (stamp >= odom_buffer_.back().stamp) {
    out = odom_buffer_.back().pose;
    return true;
  }

  // ... L1170-1183：区间内线性插值 ...`,
      facts: [
        ["TF 路径", "扫描时刻成功即直接返回，stale 保持 false"],
        ["空缓冲", "have_odom=false，本帧不喂新 odom"],
        ["超时", "stale=true，但仍返回末端样本"],
        ["诊断", "TRACKING + stale → WARN"],
      ],
    },
    {
      title: "11 · 环形缓冲插值：角度必须先 wrap",
      lines: "1170-1186",
      explain: "在相邻时间戳之间求 alpha，x/y 线性插值；theta 先对样本差 wrap_angle，再插值并 wrap。否则 +179° 到 -179° 会错误地长距离旋转。找不到区间时生产代码返回末端样本。",
      code: `for (std::size_t i = 0; i + 1 < odom_buffer_.size(); ++i) {
  const auto & a = odom_buffer_[i];
  const auto & b = odom_buffer_[i + 1];
  if (a.stamp <= stamp && stamp <= b.stamp) {
    const double dt = (b.stamp - a.stamp).seconds();
    const double alpha =
      dt > 1e-9 ? std::clamp((stamp - a.stamp).seconds() / dt, 0.0, 1.0) :
      0.0;
    out.x = a.pose.x + alpha * (b.pose.x - a.pose.x);
    out.y = a.pose.y + alpha * (b.pose.y - a.pose.y);
    out.theta = wrap_angle(
      a.pose.theta + alpha * wrap_angle(b.pose.theta - a.pose.theta));
    return true;
  }
}
out = odom_buffer_.back().pose;
return true;`,
      facts: [
        ["alpha", "(stamp - a.stamp) / (b.stamp - a.stamp)"],
        ["x / y", "普通线性插值"],
        ["theta", "wrap(b-a) → 插值 → wrap"],
        ["边界", "端点钳制，不无限外推"],
      ],
    },
    {
      title: "12 · onScan：锁内更新，锁外发布",
      lines: "1303-1336",
      explain: "预处理与 TF 查询都在锁外。锁内再次检查生命周期，喂入扫描时刻的 odom、运行核心、维护稳定跟踪计数并构造快照。离开花括号后绝不带状态锁调用 publishOutputs；零时间戳输出改用 now()。",
      code: `Pose2 odom_at_scan{};
bool odom_stale = false;
const bool have_odom = lookupOdomAt(scan_stamp, odom_at_scan, odom_stale);

OutputSnapshot snap;
{
  std::lock_guard<std::mutex> lk(state_mtx_);
  if (!active_.load() || !map_received_ || !core_) {return;}
  if (have_odom) {
    core_->on_odom(odom_at_scan);
  }
  last_scan_ = scan;
  has_last_scan_ = true;

  const CoreOutput out = core_->on_scan(scan);
  if (out.state == LocState::TRACKING && out.accepted &&
    out.reject_streak == 0)
  {
    ++stable_tracking_count_;
    if (stable_tracking_count_ >= kStableTrackingClearFrames &&
      (!pending_hypotheses_.empty() || !global_alternates_.empty()))
    {
      pending_hypotheses_.clear();
      global_alternates_.clear();
    }
  } else {
    stable_tracking_count_ = 0;
  }
  snap = buildSnapshotLocked(out, odom_stale);
}

publishOutputs(snap, scan_stamp.nanoseconds() == 0 ? now() : scan_stamp);`,
      facts: [
        ["锁内", "odom 锚 + core.on_scan + OutputSnapshot"],
        ["锁外", "publishOutputs / DDS / TF 发送"],
        ["二次检查", "覆盖预处理期间发生的失活/清理"],
        ["输出时间", "有扫描时间戳就保留它；为 0 才用 now()"],
      ],
    },
    {
      title: "13 · buildSnapshotLocked：复制同一个线性化视图",
      lines: "1520-1553",
      explain: "快照不是指向核心的引用，而是值复制。位姿、map→odom、坐标系、诊断标志与全部候选假设在同一把锁内取出，发布阶段不再读取可变节点状态。",
      code: `LocalizerNode::OutputSnapshot LocalizerNode::buildSnapshotLocked(
  const CoreOutput & out, bool odom_stale) const
{
  OutputSnapshot snap;
  snap.out = out;
  snap.publish_tf = tf_broadcast_ && core_ &&
    core_->state() != LocState::UNINIT;
  if (snap.publish_tf) {
    snap.map_to_odom = core_->map_to_odom();
  }
  snap.fine_on = fine_on_;
  snap.odom_stale = odom_stale;
  snap.tf_broadcast = tf_broadcast_;
  snap.transform_tolerance = transform_tolerance_;
  snap.map_frame = map_frame_;
  snap.odom_frame = odom_frame_;
  snap.base_frame = base_frame_;

  if (out.state != LocState::UNINIT) {
    snap.hypotheses.push_back(out.pose);
  }
  for (const auto & h : pending_hypotheses_) {
    snap.hypotheses.push_back(h);
  }
  for (const auto & h : global_alternates_) {
    snap.hypotheses.push_back(h);
  }
  if (core_) {
    const auto core_hyps = core_->hypothesis_poses();
    snap.hypotheses.insert(
      snap.hypotheses.end(), core_hyps.begin(), core_hyps.end());
  }
  return snap;
}`,
      facts: [
        ["调用约束", "调用者必须已持有 state_mtx_"],
        ["所有权", "CoreOutput、string、vector 全部值复制"],
        ["一致性", "TF/位姿/诊断/候选来自同一临界区"],
        ["不保证", "订阅者收到时仍是最新状态"],
      ],
    },
    {
      title: "14 · publishOutputs：无状态锁发布与 6×6 槽位",
      lines: "1555-1592",
      explain: "发布函数只读 snap。TF 使用扫描时刻加 transform_tolerance；位姿消息保留测量时间戳。协方差先清零，再把 3×3 写到 {0,1,5}，yaw 方差落在 [35]，z 槽位 [14] 保持 0。诊断与 hypotheses 在 L1594-1667 继续只读 snap。",
      code: `void LocalizerNode::publishOutputs(
  const OutputSnapshot & snap, const rclcpp::Time & stamp)
{
  if (!active_.load()) {return;}

  if (snap.publish_tf && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp + rclcpp::Duration::from_seconds(
      snap.transform_tolerance);
    tf.header.frame_id = snap.map_frame;
    tf.child_frame_id = snap.odom_frame;
    tf.transform.translation.x = snap.map_to_odom.x;
    tf.transform.translation.y = snap.map_to_odom.y;
    tf.transform.rotation.z = std::sin(0.5 * snap.map_to_odom.theta);
    tf.transform.rotation.w = std::cos(0.5 * snap.map_to_odom.theta);
    tf_broadcaster_->sendTransform(tf);
  }

  if (pose_pub_ && pose_pub_->is_activated()) {
    geometry_msgs::msg::PoseWithCovarianceStamped pm;
    pm.header.stamp = stamp;
    pm.header.frame_id = snap.map_frame;
    pm.pose.pose.position.x = snap.out.pose.x;
    pm.pose.pose.position.y = snap.out.pose.y;
    pm.pose.pose.orientation.z = std::sin(0.5 * snap.out.pose.theta);
    pm.pose.pose.orientation.w = std::cos(0.5 * snap.out.pose.theta);
    pm.pose.covariance.fill(0.0);
    constexpr int kIdx[3] = {0, 1, 5};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        pm.pose.covariance[static_cast<std::size_t>(6 * kIdx[i] + kIdx[j])] =
          snap.out.covariance(i, j);
      }
    }
    pose_pub_->publish(pm);
  }

  // ... L1594-1667：hypotheses 与 DiagnosticArray ...`,
      facts: [
        ["state_mtx_", "未持有"],
        ["Lifecycle", "active_ + publisher.is_activated() 闸门"],
        ["协方差", "{0,1,5}；yaw=[35]；z=[14]=0"],
        ["回归", "CovarianceSlotsNanFilteringAndNoMotionUpdate"],
      ],
    },
  ];

  const TIMELINES = {
    "map-success": {
      title: "正确路径：锁外构建、原子交换、锁外发布",
      invariant: "新地图未全部就绪前，旧地图保持完整可用",
      events: [
        { t: "T0", work: "applyMap：验证 grid", sensor: "odom/IMU 写缓冲", lock: "FREE", decision: "坏输入在构建前返回，不占状态锁。", tone: "ok" },
        { t: "T1", work: "复制 sigma / matcher / voxel", sensor: "odom/IMU 可并行", lock: "LOCK（短暂）", decision: "只复制构建参数，随后立即解锁。", tone: "ok" },
        { t: "T2", work: "MapField::build（锁外）", sensor: "odom/IMU 继续推进", lock: "FREE", decision: "昂贵计算不阻塞传感器状态写入。", tone: "ok" },
        { t: "T3", work: "make_matcher（锁外）", sensor: "IMU 继续推进", lock: "FREE", decision: "field 与 matcher 都就绪后才考虑提交。", tone: "ok" },
        { t: "T4", work: "交换 grid/field/matcher/core 视图", sensor: "odom/IMU 等这一短暂临界区", lock: "LOCK（短暂）", decision: "同一把锁让读取者只见交换前或交换后。", tone: "ok" },
        { t: "T5", work: "地图回调结束；scan 才能进入 work 组", sensor: "odom/IMU 不受影响", lock: "FREE", decision: "真实 /scan 也在 work 组，mutually exclusive。", tone: "ok" },
        { t: "T6", work: "onScan：更新核心 + 构造快照", sensor: "odom/IMU 等这一短暂临界区", lock: "LOCK（短暂）", decision: "快照复制位姿、TF、诊断与候选。", tone: "ok" },
        { t: "T7", work: "离开临界区，开始 publish", sensor: "odom/IMU 继续写缓冲", lock: "FREE", decision: "DDS 与 TF 发送不持状态锁。", tone: "ok" },
        { t: "T8", work: "DDS publish 因慢订阅者阻塞", sensor: "odom/IMU 仍可推进", lock: "FREE", decision: "慢发布不拖住核心状态读取。", tone: "ok" },
      ],
    },
    "map-fail": {
      title: "失败路径：MapField::build 返回 false",
      invariant: "旧 grid_/field_/matcher_ 在提交前不被修改",
      events: [
        { t: "T0", work: "applyMap：地图 B 输入有效", sensor: "使用地图 A 正常定位", lock: "FREE", decision: "输入有效不代表构建一定成功。", tone: "warn" },
        { t: "T1", work: "复制构建参数", sensor: "odom/IMU 可并行", lock: "LOCK（短暂）", decision: "参数副本完成，锁释放。", tone: "ok" },
        { t: "T2", work: "MapField::build(B) 返回 false", sensor: "odom/IMU 继续推进", lock: "FREE", decision: "尚未进入交换临界区。", tone: "bad" },
        { t: "T3", work: "局部 field 销毁；applyMap 返回 false", sensor: "odom/IMU 不受影响", lock: "FREE", decision: "不清空旧 field，不提交半成品。", tone: "ok" },
        { t: "T4", work: "后续 scan 仍读取地图 A", sensor: "odom/IMU 正常", lock: "LOCK（正常步进）", decision: "定位服务保持连续。", tone: "ok" },
        { t: "T5", work: "调用方检查 bool 并上报失败", sensor: "继续运行", lock: "FREE", decision: "忽略返回值会把切换失败误报为成功。", tone: "warn" },
      ],
    },
    "tf-fallback": {
      title: "回退路径：扫描时刻 TF 缺失",
      invariant: "外参与 odom 都尽量回到同一个 scan_stamp",
      events: [
        { t: "10.000s", work: "onScan 固定 scan_stamp", sensor: "odom 缓冲已有 9.990/10.010", lock: "FREE", decision: "不把回调时刻或最新 odom 当作测量时刻。", tone: "ok" },
        { t: "T1", work: "base→laser 精确查询发生 ExtrapolationException", sensor: "IMU 可并行", lock: "FREE", decision: "仅这种异常回退 latest 静态外参。", tone: "warn" },
        { t: "T2", work: "过滤扫描并变换到 base_frame", sensor: "odom 缓冲可更新", lock: "FREE", decision: "有效点不足会在核心更新前丢帧。", tone: "ok" },
        { t: "T3", work: "odom→base TF 查询失败", sensor: "缓冲仍可用", lock: "FREE", decision: "进入 /odom 环形缓冲回退。", tone: "warn" },
        { t: "T4", work: "在 9.990/10.010 间插值", sensor: "odom 写入等缓冲锁", lock: "LOCK（缓冲读取）", decision: "x/y 线性插值，theta 对样本差先 wrap。", tone: "ok" },
        { t: "T5", work: "锁内喂入 odom_at_scan 并步进核心", sensor: "odom/IMU 等状态锁", lock: "LOCK（核心更新）", decision: "运动锚与扫描属于同一时刻。", tone: "ok" },
        { t: "T6", work: "锁外发布", sensor: "odom/IMU 继续", lock: "FREE", decision: "消息 header.stamp 保留 10.000s。", tone: "ok" },
      ],
    },
    "odom-stale": {
      title: "告警路径：扫描晚于 odom 缓冲末端",
      invariant: "stale 是显式质量信号，不静默伪装成新鲜数据",
      events: [
        { t: "10.000s", work: "onScan 固定扫描时刻", sensor: "odom 最新样本仍为 8.000s", lock: "FREE", decision: "扫描与 odom 已明显分离。", tone: "warn" },
        { t: "T1", work: "odom TF 不可用", sensor: "缓冲非空", lock: "FREE", decision: "进入环形缓冲端点处理。", tone: "warn" },
        { t: "T2", work: "发现 2.0s > odom_timeout", sensor: "odom/IMU 可并行", lock: "LOCK（缓冲读取）", decision: "stale=true，并限频告警。", tone: "warn" },
        { t: "T3", work: "仍取末端 8.000s 样本", sensor: "继续运行", lock: "FREE", decision: "生产代码标记 stale，不直接拒绝该帧。", tone: "ok" },
        { t: "T4", work: "核心更新并构造快照", sensor: "等状态锁", lock: "LOCK（核心更新）", decision: "odom_stale 被复制进 OutputSnapshot。", tone: "ok" },
        { t: "T5", work: "锁外发布诊断", sensor: "继续运行", lock: "FREE", decision: "TRACKING + stale 时诊断级别为 WARN。", tone: "warn" },
      ],
    },
    "deactivate-race": {
      title: "生命周期竞态：预处理期间被 deactivate",
      invariant: "入口 active_ 检查不足以覆盖整个异步回调",
      events: [
        { t: "T0", work: "onScan 入口看到 active_=true", sensor: "odom/IMU 正常", lock: "FREE", decision: "开始读取配置与预处理。", tone: "ok" },
        { t: "T1", work: "TF 查询 / 扫描变换（锁外）", sensor: "odom/IMU 正常", lock: "FREE", decision: "此时还没有提交核心更新。", tone: "ok" },
        { t: "T2", work: "lifecycle：active_=false，发布器失活", sensor: "新回调被入口闸门拒绝", lock: "FREE", decision: "失活先于发布器关闭。", tone: "warn" },
        { t: "T3", work: "scan 返回预处理，尝试提交", sensor: "odom/IMU 可运行", lock: "LOCK（二次检查）", decision: "active_ 已为 false，立即 return。", tone: "ok" },
        { t: "T4", work: "不调用 core.on_scan，不发布", sensor: "继续运行", lock: "FREE", decision: "没有过期输出，也没有修改失活后的核心。", tone: "ok" },
      ],
    },
    "held-lock": {
      title: "反例：锁内构场、锁内发布",
      invariant: "反例只用于识别故障，不要照抄",
      events: [
        { t: "T0", work: "错误：持 state_mtx_ 开始构建地图", sensor: "odom/IMU 等待状态锁", lock: "LOCK（错误）", decision: "昂贵构建占用了所有状态读取。", tone: "bad" },
        { t: "T1", work: "MapField::build 持续数十毫秒（示意）", sensor: "odom/IMU 被阻塞", lock: "LOCK（错误）", decision: "传感器缓冲延迟增长。", tone: "bad" },
        { t: "T2", work: "构建完成，继续切图", sensor: "scan 还受 work 组串行约束", lock: "LOCK（错误）", decision: "即使切图成功，阻塞已经发生。", tone: "bad" },
        { t: "T3", work: "错误：仍持锁调用 publish()", sensor: "所有状态读写继续等待", lock: "LOCK（错误）", decision: "慢订阅者把局部阻塞扩散到全节点。", tone: "bad" },
        { t: "T4", work: "DDS 背压迟迟不返回", sensor: "odom/IMU 超时风险", lock: "LOCK（错误）", decision: "修法：构建与 publish 都移出状态锁。", tone: "bad" },
      ],
    },
  };

  function byId(id) {
    return document.getElementById(id);
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

  function initLifecycle() {
    const vars = byId("ch08-life-vars");
    const log = byId("ch08-life-log");
    const fail = byId("ch08-life-fail");
    if (!vars || !log) return;

    let state = "UNCONFIGURED";
    let configureFailed = false;
    const stateInfo = {
      UNCONFIGURED: {
        active: "false",
        publishers: "未创建 / 未激活",
        resources: configureFailed ? "configure 失败后已回滚" : "尚未创建",
        allowed: ["configure", "shutdown"],
      },
      INACTIVE: {
        active: "false",
        publishers: "已创建 / 未激活",
        resources: "core、地图、TF、订阅已创建",
        allowed: ["activate", "cleanup", "shutdown"],
      },
      ACTIVE: {
        active: "true",
        publishers: "已激活",
        resources: "core、地图、TF、订阅正在服务",
        allowed: ["deactivate", "shutdown"],
      },
      FINALIZED: {
        active: "false",
        publishers: "已销毁 / 未激活",
        resources: "已全部清理",
        allowed: [],
      },
    };

    function render() {
      const info = stateInfo[state];
      ["UNCONFIGURED", "INACTIVE", "ACTIVE", "FINALIZED"].forEach((name) => {
        const pill = byId(`ch08-life-${name.toLowerCase()}`);
        if (!pill) return;
        const current = name === state;
        pill.setAttribute("aria-current", current ? "step" : "false");
        pill.style.borderColor = current ? "var(--accent)" : "";
        pill.style.background = current ? "var(--accent-soft)" : "";
        pill.style.color = current ? "var(--accent)" : "";
      });
      document.querySelectorAll("[data-life-action]").forEach((button) => {
        button.disabled = !info.allowed.includes(button.dataset.lifeAction);
      });
      if (fail) fail.disabled = state !== "UNCONFIGURED";
      fillVars(vars, [
        ["Lifecycle state", state],
        ["active_", info.active],
        ["发布器", info.publishers],
        ["核心 / 地图 / TF", info.resources],
        ["下一步合法迁移", info.allowed.length ? info.allowed.join(" / ") : "无；点击“重置演示”重新开始"],
      ]);
    }

    document.querySelectorAll("[data-life-action]").forEach((button) => {
      button.addEventListener("click", () => {
        const action = button.dataset.lifeAction;
        if (!stateInfo[state].allowed.includes(action)) return;
        if (action === "configure") {
          if (fail && fail.checked) {
            configureFailed = true;
            stateInfo.UNCONFIGURED.resources = "configure 失败后已回滚";
            log.textContent = "configure → 参数合法，但 map_yaml 加载失败 → resetResources() → FAILURE；状态保持 UNCONFIGURED。";
          } else {
            configureFailed = false;
            stateInfo.UNCONFIGURED.resources = "尚未创建";
            state = "INACTIVE";
            log.textContent = "configure → 校验通过、创建核心/TF/订阅/发布器 → SUCCESS；状态进入 INACTIVE，active_ 仍为 false。";
          }
        } else if (action === "activate") {
          state = "ACTIVE";
          log.textContent = "activate → 发布器 on_activate() → active_=true → ACTIVE；现在 scan 才可能产生输出。";
        } else if (action === "deactivate") {
          state = "INACTIVE";
          log.textContent = "deactivate → active_=false → 发布器 on_deactivate()；核心与地图保留，状态回到 INACTIVE。";
        } else if (action === "cleanup") {
          state = "UNCONFIGURED";
          configureFailed = false;
          stateInfo.UNCONFIGURED.resources = "尚未创建";
          log.textContent = "cleanup → resetResources()；核心、地图、TF 与缓冲销毁，imu_started_ 清零 → UNCONFIGURED。";
        } else if (action === "shutdown") {
          state = "FINALIZED";
          log.textContent = "shutdown → resetResources() → FINALIZED；不能再 configure 或 activate。";
        }
        render();
      });
    });

    byId("ch08-life-reset")?.addEventListener("click", () => {
      state = "UNCONFIGURED";
      configureFailed = false;
      stateInfo.UNCONFIGURED.resources = "尚未创建";
      if (fail) fail.checked = false;
      log.textContent = "已重置：UNCONFIGURED；不创建核心、TF 或发布器。";
      render();
    });

    render();
  }

  function initTimeline() {
    const select = byId("ch08-timeline-scenario");
    const rowsHost = byId("ch08-timeline-rows");
    const title = byId("ch08-timeline-title");
    const meta = byId("ch08-timeline-meta");
    const vars = byId("ch08-timeline-vars");
    const decision = byId("ch08-timeline-decision");
    const prev = byId("ch08-timeline-prev");
    const next = byId("ch08-timeline-next");
    const play = byId("ch08-timeline-play");
    if (!select || !rowsHost || !title || !meta || !decision || !prev || !next || !play) return;

    let current = -1;
    let playTimer = null;

    function scenario() {
      return TIMELINES[select.value] || TIMELINES["map-success"];
    }

    function stopPlay() {
      if (playTimer !== null) {
        window.clearInterval(playTimer);
        playTimer = null;
      }
      play.textContent = "播放";
    }

    function render() {
      const data = scenario();
      const events = data.events;
      title.textContent = data.title;
      meta.textContent = `相对时间线（示意，非基准） · ${current + 1} / ${events.length}`;
      rowsHost.textContent = "";
      const fragment = document.createDocumentFragment();
      events.forEach((event, index) => {
        const tr = document.createElement("tr");
        const active = index === current;
        const past = index < current;
        tr.style.opacity = active || past ? "1" : "0.45";
        tr.style.background = active ? "var(--accent-soft)" : "";
        [event.t, event.work, event.sensor, event.lock, event.decision].forEach((text) => {
          const td = document.createElement("td");
          td.textContent = text;
          tr.appendChild(td);
        });
        if (active) {
          tr.style.boxShadow = "inset 3px 0 0 var(--accent)";
          const lockCell = tr.children[3];
          lockCell.style.color = event.tone === "bad" ? "var(--red)" :
            event.tone === "warn" ? "var(--amber, #b7791f)" : "var(--green)";
        }
        fragment.appendChild(tr);
      });
      rowsHost.appendChild(fragment);
      prev.disabled = current <= 0;
      next.disabled = current >= events.length - 1;
      if (current < 0) {
        decision.textContent = "尚未推进。先观察哪一列持锁，再点击“下一步”。";
        decision.style.color = "";
      } else {
        decision.textContent = events[current].decision;
        decision.style.color = events[current].tone === "bad" ? "var(--red)" :
          events[current].tone === "warn" ? "var(--amber, #b7791f)" : "var(--green)";
      }
      const event = current >= 0 ? events[current] : null;
      fillVars(vars, [
        ["当前场景", data.title],
        ["state_mtx_", event ? event.lock : "尚未推进"],
        ["work 组", "map 与 scan mutually exclusive；组内不并发"],
        ["sensor 组", "odom/IMU 可与 work 组并行，前提是多线程 executor"],
        ["失败不变量", data.invariant],
      ]);
    }

    function reset() {
      stopPlay();
      current = -1;
      render();
    }

    prev.addEventListener("click", () => {
      stopPlay();
      if (current > 0) current--;
      render();
    });
    next.addEventListener("click", () => {
      stopPlay();
      if (current < scenario().events.length - 1) current++;
      render();
    });
    byId("ch08-timeline-reset")?.addEventListener("click", reset);
    select.addEventListener("change", reset);
    play.addEventListener("click", () => {
      if (playTimer !== null) {
        stopPlay();
        return;
      }
      if (current >= scenario().events.length - 1) current = -1;
      const advance = () => {
        if (current < scenario().events.length - 1) {
          current++;
          render();
        }
        if (current >= scenario().events.length - 1) stopPlay();
      };
      advance();
      if (current < scenario().events.length - 1) {
        play.textContent = "暂停";
        playTimer = window.setInterval(advance, 900);
      }
    });

    render();
  }

  function initStepper() {
    const title = byId("ch08-stage-title");
    const meta = byId("ch08-stage-meta");
    const explain = byId("ch08-stage-explain");
    const code = byId("ch08-stage-code");
    const index = byId("ch08-stage-index");
    const facts = byId("ch08-stage-facts");
    const prev = byId("ch08-stage-prev");
    const next = byId("ch08-stage-next");
    const source = byId("ch08-stage-source");
    const copy = byId("ch08-stage-copy");
    if (!title || !meta || !explain || !code || !index || !facts || !prev || !next || !source) return;

    let stage = 0;

    function render() {
      const cur = STAGES[stage];
      title.textContent = cur.title;
      meta.textContent = `src/localizer_node.cpp:L${cur.lines} · 生产源码节选`;
      index.textContent = `${stage + 1} / ${STAGES.length}`;
      explain.textContent = cur.explain;
      code.textContent = cur.code;
      code.className = "language-cpp";
      delete code.dataset.highlighted;
      if (window.hljs) {
        try { window.hljs.highlightElement(code); } catch (_) {}
      }
      source.setAttribute("data-file", "src/localizer_node.cpp");
      source.setAttribute("data-lines", cur.lines);
      source.setAttribute("data-label", `LocalizerNode L${cur.lines}`);
      fillVars(facts, cur.facts);
      prev.disabled = stage === 0;
      next.disabled = stage === STAGES.length - 1;
    }

    prev.addEventListener("click", () => {
      if (stage > 0) { stage--; render(); }
    });
    next.addEventListener("click", () => {
      if (stage < STAGES.length - 1) { stage++; render(); }
    });
    copy?.addEventListener("click", () => {
      if (navigator.clipboard) {
        navigator.clipboard.writeText(code.textContent).then(() => {
          copy.textContent = "已复制";
        }).catch(() => {
          copy.textContent = "请手动复制";
        });
      } else {
        copy.textContent = "请手动复制";
      }
      window.setTimeout(() => { copy.textContent = "复制"; }, 1200);
    });

    render();
  }

  function initExercises() {
    document.querySelectorAll(".hands-on-exercise").forEach((form) => {
      form.addEventListener("submit", (event) => {
        event.preventDefault();
        const input = form.querySelector(".exercise-input");
        const feedback = form.querySelector(".exercise-feedback");
        if (!input || !feedback) return;
        const raw = input.value.trim().replace(/\s+/g, "");
        const kind = form.dataset.kind || "number";
        let ok = false;
        if (kind === "choice") {
          ok = raw.toLowerCase() === String(form.dataset.answer || "").toLowerCase();
        } else {
          const value = Number(raw);
          const tolerance = Number(form.dataset.tolerance || 1e-6);
          ok = raw !== "" && Number.isFinite(value) &&
            Math.abs(value - Number(form.dataset.answer)) <= tolerance;
        }
        feedback.className = `exercise-feedback ${ok ? "ok" : "bad"}`;
        feedback.textContent = ok
          ? (form.dataset.success || "正确。继续下一题。")
          : (form.dataset.hint || "还不对：回到时间线、失败路径与源码边界再检查一次。");
      });
    });
  }

  function init() {
    if (window.PAGE_ID !== "ch08") return;
    initLifecycle();
    initTimeline();
    initStepper();
    initExercises();
  }

  window.addEventListener("sloc:ready", init);
})();
