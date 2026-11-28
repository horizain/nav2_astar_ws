// Copyright 2026 ruijianding
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// ============================================================================
// LocalizerNode 实现
// ============================================================================
#include "scan_localizer/localizer_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>

#include "Eigen/Geometry"
#include "scan_localizer/map_io.hpp"

namespace sloc
{

namespace
{

constexpr double kExternalHypMinScore = 0.85;
constexpr double kExternalHypWindowLin = 0.30;
constexpr double kExternalHypWindowAng = 0.20;
constexpr double kAlternateMinSepM = 0.30;
constexpr double kDefaultFallbackRangeMax = 12.0;
constexpr int kStableTrackingClearFrames = 3;
constexpr double kMaxOriginYawRad = 1e-3;
constexpr double kDegPerRad = 180.0 / kPi;

bool isFinitePositive(double v)
{
  return std::isfinite(v) && v > 0.0;
}

}  // namespace

LocalizerNode::LocalizerNode(const rclcpp::NodeOptions & opts)
: rclcpp_lifecycle::LifecycleNode("scan_localizer", opts)
{
  declareParameters();
  sensor_cb_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  work_cb_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  if (get_parameter("autostart").as_bool()) {
    autostart_timer_ = create_wall_timer(
      std::chrono::milliseconds(0),
      [this]() {
        if (autostart_timer_) {
          autostart_timer_->cancel();
        }
        if (get_current_state().id() !=
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
        {
          return;
        }
        this->configure();
        if (get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        {
          this->activate();
        } else {
          RCLCPP_ERROR(
            get_logger(),
            "autostart configure() did not reach INACTIVE; skipping activate()");
        }
      },
      work_cb_group_);
  }
}

double LocalizerNode::quatToYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool LocalizerNode::isValidMatcherName(const std::string & name)
{
  return name == "composed" || name == "correlative" ||
         name == "field_gn" || name == "icp_p2p" || name == "ndt";
}

CoreConfig LocalizerNode::buildFineConfig(const CoreConfig & base)
{
  CoreConfig fine = base;
  fine.match_opt.window_lin = 0.10;
  fine.match_opt.window_ang = 0.05;
  fine.match_opt.huber_delta = 0.05;
  fine.match_opt.trim_dist = 0.20;
  fine.match_opt.max_iterations = 30;
  fine.min_inlier_ratio = 0.75;
  return fine;
}

Pose2 LocalizerNode::odomToPose(const nav_msgs::msg::Odometry & m) const
{
  const auto & p = m.pose.pose.position;
  return {p.x, p.y, quatToYaw(m.pose.pose.orientation)};
}


void LocalizerNode::resetResources()
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
  map_path_sub_.reset();
  hyp_sub_.reset();
  initpose_sub_.reset();

  global_srv_.reset();
  fine_srv_.reset();
  nomotion_srv_.reset();

  pose_pub_.reset();
  status_pub_.reset();
  hypotheses_pub_.reset();

  tf_broadcaster_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  std::unique_ptr<AsyncGlobalWorker> old_worker;
  std::unique_ptr<BlackboxRecorder> old_blackbox;
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    old_worker = std::move(global_worker_);
    old_blackbox = std::move(blackbox_);
    core_.reset();
    field_.reset();
    grid_.reset();
    matcher_.reset();
    map_received_ = false;
    fine_on_ = false;
    stable_tracking_count_ = 0;
    imu_yaw_ = 0.0;
    last_imu_wz_ = 0.0;
    imu_started_ = false;
    last_imu_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    odom_sync_.clear();
    global_retry_gate_.reset();
    has_odom_sample_ = false;
    first_odom_fed_ = false;
    last_odom_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_scan_.points.clear();
    has_last_scan_ = false;
    pending_hypotheses_.clear();
    global_alternates_.clear();
  }
}

LocalizerNode::CallbackReturn LocalizerNode::on_configure(
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
    core_->set_global_multi_fn(
      [this](const Scan & s) {
        return use_global_search_ ? runGlobalMultiSearchLocked(s) : GlobalResult{};
      });
    if (use_global_search_ && global_async_) {
      GlobalWorkerConfig wcfg;
      wcfg.beams_subsample = 3;
      wcfg.ang_step = 0.0;
      wcfg.n_hypotheses = 8;
      wcfg.max_time_ms = global_max_time_ms_;
      wcfg.nice_value = global_worker_nice_;
      global_worker_ = std::make_unique<AsyncGlobalWorker>(wcfg);
    } else {
      global_worker_.reset();
    }
    if (blackbox_enabled_) {
      BlackboxConfig bbcfg;
      bbcfg.enabled = true;
      bbcfg.output_dir = blackbox_output_dir_;
      blackbox_ = std::make_unique<BlackboxRecorder>(bbcfg);
    } else {
      blackbox_.reset();
    }
    fine_on_ = false;
    map_received_ = false;
    stable_tracking_count_ = 0;
    imu_yaw_ = 0.0;
    last_imu_wz_ = 0.0;
    imu_started_ = false;
    last_imu_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    odom_sync_.clear();
    global_retry_gate_.reset();
    has_odom_sample_ = false;
    first_odom_fed_ = false;
    last_odom_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_scan_.points.clear();
    has_last_scan_ = false;
    pending_hypotheses_.clear();
    global_alternates_.clear();
  }

  pose_pub_ = create_publisher<
    geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic_, 10);
  status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    status_topic_, 10);
  hypotheses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
    hypotheses_topic_, 10);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::SubscriptionOptions sensor_sub_opts;
  sensor_sub_opts.callback_group = sensor_cb_group_;
  rclcpp::SubscriptionOptions work_sub_opts;
  work_sub_opts.callback_group = work_cb_group_;

  // map：transient_local QoS，支持 map_server 动态重载
  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.transient_local();
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, map_qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr m) {onMap(m);},
    work_sub_opts);

  // scan：SensorDataQoS
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr m) {onScan(m);},
    work_sub_opts);

  // odom / imu：独立高频传感器回调组
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 50,
    [this](const nav_msgs::msg::Odometry::SharedPtr m) {onOdom(m);},
    sensor_sub_opts);
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu::SharedPtr m) {onImu(m);},
    sensor_sub_opts);

  // 地图热切换、外部全局假设与初始位姿
  map_path_sub_ = create_subscription<std_msgs::msg::String>(
    map_path_topic_, 10,
    [this](const std_msgs::msg::String::SharedPtr m) {onMapPath(m);},
    work_sub_opts);
  hyp_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
    global_hypotheses_topic_, 10,
    [this](const geometry_msgs::msg::PoseArray::SharedPtr m) {
      onHypotheses(m);
    },
    work_sub_opts);
  initpose_sub_ =
    create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    initialpose_topic_, 10,
    [this](
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
      onInitialPose(m);
    },
    work_sub_opts);

  // 服务
  global_srv_ = create_service<std_srvs::srv::Trigger>(
    global_relocalize_service_,
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr req,
      std_srvs::srv::Trigger::Response::SharedPtr res) {
      onGlobalRelocalize(req, res);
    },
    rclcpp::ServicesQoS(), work_cb_group_);
  fine_srv_ = create_service<std_srvs::srv::SetBool>(
    set_fine_mode_service_,
    [this](
      const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
      onSetFineMode(req, res);
    },
    rclcpp::ServicesQoS(), work_cb_group_);
  nomotion_srv_ = create_service<std_srvs::srv::Empty>(
    nomotion_update_service_,
    [this](
      const std_srvs::srv::Empty::Request::SharedPtr req,
      std_srvs::srv::Empty::Response::SharedPtr res) {
      onNoMotionUpdate(req, res);
    },
    rclcpp::ServicesQoS(), work_cb_group_);

  // TF 保活定时器
  if (tf_publish_rate_ > 0.0) {
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / tf_publish_rate_));
    tf_keepalive_timer_ = create_wall_timer(
      period, [this]() {onTfKeepaliveTimer();}, work_cb_group_);
  }

  // 动态参数校验与热更新回调
  param_cb_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      return onSetParameters(params);
    });

  // 若配置了 map_yaml，直接加载；失败则返回 FAILURE 并回滚资源
  if (!map_yaml_.empty()) {
    if (!loadMapFromYaml(map_yaml_)) {
      RCLCPP_ERROR(
        get_logger(), "map_yaml load FAILED: %s", map_yaml_.c_str());
      resetResources();
      return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "map loaded from yaml: %s", map_yaml_.c_str());
  }

  RCLCPP_INFO(
    get_logger(),
    "configured. waiting for map on '%s' and scans on '%s'",
    map_topic_.c_str(), scan_topic_.c_str());
  return CallbackReturn::SUCCESS;
}

LocalizerNode::CallbackReturn LocalizerNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (pose_pub_) {pose_pub_->on_activate();}
  if (status_pub_) {status_pub_->on_activate();}
  if (hypotheses_pub_) {hypotheses_pub_->on_activate();}
  active_.store(true);
  RCLCPP_INFO(get_logger(), "activated");
  return CallbackReturn::SUCCESS;
}

LocalizerNode::CallbackReturn LocalizerNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  if (pose_pub_) {pose_pub_->on_deactivate();}
  if (status_pub_) {status_pub_->on_deactivate();}
  if (hypotheses_pub_) {hypotheses_pub_->on_deactivate();}
  RCLCPP_INFO(get_logger(), "deactivated");
  return CallbackReturn::SUCCESS;
}

LocalizerNode::CallbackReturn LocalizerNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  resetResources();
  RCLCPP_INFO(get_logger(), "cleaned up");
  return CallbackReturn::SUCCESS;
}

LocalizerNode::CallbackReturn LocalizerNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  resetResources();
  RCLCPP_INFO(get_logger(), "shut down");
  return CallbackReturn::SUCCESS;
}

bool LocalizerNode::applyMap(
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

  // 读取构建参数副本，在锁外构建 MapField 与 Matcher，避免阻塞高频传感器回调
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
      get_logger(), "applyMap: failed to build matcher '%s'",
      matcher_name.c_str());
    return false;
  }

  const int map_w = grid->width;
  const int map_h = grid->height;
  const double map_res = grid->resolution;
  const double map_ox = grid->origin_x;
  const double map_oy = grid->origin_y;
  bool first = false;
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
    if (global_worker_) {
      global_worker_->invalidate();
    }
    global_retry_gate_.reset();
    core_->set_map(field_);
    core_->set_matcher(matcher_);
    map_received_ = true;

    if (!first && !keep_pose) {
      core_->request_global();
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "map applied: %dx%d @ %.3fm origin(%.2f,%.2f)%s",
    map_w, map_h, map_res, map_ox, map_oy,
    first ? "" : (keep_pose ? " [switched, keep_pose]" :
    " [switched, global_relocalize]"));
  return true;
}

bool LocalizerNode::loadMapFromYaml(const std::string & yaml_path)
{
  auto grid = std::make_shared<OccupancyGrid>();
  std::string err;
  if (!load_map_yaml(yaml_path, *grid, err)) {
    RCLCPP_ERROR(
      get_logger(), "load_map_yaml(%s): %s", yaml_path.c_str(), err.c_str());
    return false;
  }
  bool keep_pose = true;
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    keep_pose = map_keep_pose_;
  }
  return applyMap(std::move(grid), keep_pose);
}

void LocalizerNode::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr m)
{
  if (!m) {return;}
  if (m->info.width == 0 || m->info.height == 0 ||
    !isFinitePositive(m->info.resolution))
  {
    RCLCPP_ERROR(
      get_logger(), "invalid /map metadata: %ux%u res=%.4f",
      m->info.width, m->info.height, m->info.resolution);
    return;
  }
  const std::size_t expected =
    static_cast<std::size_t>(m->info.width) *
    static_cast<std::size_t>(m->info.height);
  if (m->data.size() != expected) {
    RCLCPP_ERROR(
      get_logger(), "invalid /map data size: %zu != %zu",
      m->data.size(), expected);
    return;
  }
  const double origin_yaw = quatToYaw(m->info.origin.orientation);
  if (std::abs(origin_yaw) > kMaxOriginYawRad) {
    RCLCPP_WARN(
      get_logger(),
      "rejecting /map with non-zero origin orientation yaw=%.4f rad",
      origin_yaw);
    return;
  }

  auto grid = std::make_shared<OccupancyGrid>();
  grid->width = static_cast<int>(m->info.width);
  grid->height = static_cast<int>(m->info.height);
  grid->resolution = m->info.resolution;
  // 标准 ROS 左下角原点约定（与 T1.1/T1.2 一致）
  grid->origin_x = m->info.origin.position.x;
  grid->origin_y = m->info.origin.position.y;
  grid->cells = m->data;

  bool keep_pose = true;
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    keep_pose = map_keep_pose_;
  }
  applyMap(std::move(grid), keep_pose);
}

void LocalizerNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr m)
{
  if (!m) {return;}
  const Pose2 pose = odomToPose(*m);
  const double vx = m->twist.twist.linear.x;
  const double vy = m->twist.twist.linear.y;
  const double wz = m->twist.twist.angular.z;
  rclcpp::Time stamp(m->header.stamp);
  if (stamp.nanoseconds() == 0) {
    stamp = now();
  }

  std::lock_guard<std::mutex> lk(state_mtx_);
  if (!core_) {return;}

  odom_sync_.push(stamp.seconds(), pose, vx, vy, wz);
  last_odom_stamp_ = stamp;
  has_odom_sample_ = true;
  if (!first_odom_fed_) {
    core_->on_odom(pose);
    first_odom_fed_ = true;
  }
  if (blackbox_) {
    blackbox_->record_odom(stamp.seconds(), pose, imu_yaw_, imu_started_);
  }
}

void LocalizerNode::onImu(const sensor_msgs::msg::Imu::SharedPtr m)
{
  if (!m) {return;}
  const double wz = m->angular_velocity.z;
  if (!std::isfinite(wz)) {return;}
  rclcpp::Time stamp(m->header.stamp);
  if (stamp.nanoseconds() == 0) {
    stamp = now();
  }

  std::lock_guard<std::mutex> lk(state_mtx_);
  if (!core_) {return;}
  if (!imu_started_) {
    imu_started_ = true;
    last_imu_stamp_ = stamp;
    last_imu_wz_ = wz;
    return;
  }
  const double dt = (stamp - last_imu_stamp_).seconds();
  if (dt <= 0.0 || dt > max_imu_dt_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "dropping IMU sample with invalid dt=%.4f s (max=%.3f s)",
      dt, max_imu_dt_);
    last_imu_stamp_ = stamp;
    last_imu_wz_ = wz;
    return;
  }
  const double prev_wz = last_imu_wz_;
  last_imu_stamp_ = stamp;
  last_imu_wz_ = wz;
  imu_yaw_ += 0.5 * (prev_wz + wz) * dt;
  core_->on_imu_yaw(imu_yaw_, dt);
}

void LocalizerNode::onMapPath(const std_msgs::msg::String::SharedPtr m)
{
  if (!m || m->data.empty()) {return;}
  RCLCPP_INFO(get_logger(), "map switch requested: %s", m->data.c_str());
  loadMapFromYaml(m->data);
}

void LocalizerNode::onHypotheses(
  const geometry_msgs::msg::PoseArray::SharedPtr m)
{
  if (!m) {return;}
  std::lock_guard<std::mutex> lk(state_mtx_);
  if (!core_) {return;}
  if (!m->header.frame_id.empty() && m->header.frame_id != map_frame_) {
    RCLCPP_WARN(
      get_logger(),
      "ignoring global_hypotheses in frame '%s' (expected '%s')",
      m->header.frame_id.c_str(), map_frame_.c_str());
    return;
  }
  std::vector<Pose2> ext_poses;
  ext_poses.reserve(m->poses.size());
  for (const auto & p : m->poses) {
    const double yaw = quatToYaw(p.orientation);
    if (std::isfinite(p.position.x) && std::isfinite(p.position.y) && std::isfinite(yaw)) {
      ext_poses.emplace_back(p.position.x, p.position.y, yaw);
    }
  }
  if (ext_poses.empty()) {
    return;
  }
  stable_tracking_count_ = 0;
  // 若处于跟踪或降级态，直接注入 Core 多假设队列参与实时异常恢复；若处于未初始化/全局态，
  // 交由全局多峰搜索入口做宽窗验证与对称消歧（绝不静默丢弃）。
  if (core_->state() == LocState::TRACKING || core_->state() == LocState::DEGRADED) {
    core_->add_hypotheses(ext_poses);
    pending_hypotheses_.clear();
  } else {
    pending_hypotheses_ = ext_poses;
    if (core_->state() == LocState::UNINIT && map_received_) {
      core_->request_global(true);
    }
  }
  RCLCPP_INFO(
    get_logger(), "external hypotheses received: %zu",
    ext_poses.size());
}

bool LocalizerNode::lookupOdomAt(
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

  // 1) 优先查 TF2 odom_frame -> base_frame
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

  // 2) 回退至带时间戳的 /odom 环形缓冲插值（委托 OdomSyncBuffer）
  std::lock_guard<std::mutex> lk(state_mtx_);
  const double query_s = stamp.nanoseconds() > 0 ? stamp.seconds() : 0.0;
  double stale_age_s = 0.0;
  if (!odom_sync_.lookup(query_s, out, odom_timeout_, stale, stale_age_s)) {
    return false;
  }
  if (stale) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "odom is stale by %.3f s (timeout=%.2f s)",
      stale_age_s, odom_timeout_);
  }
  return true;
}

void LocalizerNode::estimateTwistAtLocked(
  const rclcpp::Time & stamp, double & vx, double & vy, double & wz) const
{
  const double query_s = stamp.nanoseconds() > 0 ? stamp.seconds() : odom_sync_.latest_stamp();
  odom_sync_.estimate_twist(query_s, vx, vy, wz);

  if (core_cfg_.use_imu_yaw && imu_started_ &&
    (stamp - last_imu_stamp_).seconds() < max_imu_dt_)
  {
    wz = last_imu_wz_ - (core_ ? core_->gyro_bias() : 0.0);
  }
}

void LocalizerNode::onScan(const sensor_msgs::msg::LaserScan::SharedPtr m)
{
  if (!active_.load() || !m) {return;}

  // 读取扫描参数快照
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

  const rclcpp::Time scan_stamp(m->header.stamp);

  // 1) 获取 base_frame -> laser_frame 3D 外参（支持倒装 roll=π 雷达）
  bool use_3d_tf = false;
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
  }

  // 2) LaserScan -> base_frame 2D 点云（过滤 NaN/Inf 及超界点）
  Scan scan;
  const double rmax =
    range_max_override > 0.0 ?
    range_max_override :
    (std::isfinite(m->range_max) && m->range_max > 0.0 ?
    static_cast<double>(m->range_max) : kDefaultFallbackRangeMax);
  const double rmin =
    (std::isfinite(m->range_min) && m->range_min > 0.0) ?
    static_cast<double>(m->range_min) : 0.0;
  const int n = static_cast<int>(m->ranges.size());
  const int step = std::max(1, subsample);
  scan.points.reserve(static_cast<std::size_t>(n / step + 1));

  double beam_dt = 0.0;
  const bool valid_inc = std::isfinite(m->time_increment) &&
    m->time_increment > 0.0f &&
    static_cast<double>(n) * static_cast<double>(m->time_increment) <= 0.5;
  const bool valid_scan_t = std::isfinite(m->scan_time) &&
    m->scan_time > 0.0f && m->scan_time <= 0.5 && n > 1;
  if (valid_inc) {
    beam_dt = static_cast<double>(m->time_increment);
  } else if (valid_scan_t) {
    beam_dt = static_cast<double>(m->scan_time) / static_cast<double>(n);
  }

  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  if (beam_dt > 0.0) {
    std::lock_guard<std::mutex> lk(state_mtx_);
    estimateTwistAtLocked(scan_stamp, vx, vy, wz);
  }

  const double lc = std::cos(lth);
  const double ls = std::sin(lth);
  for (int i = 0; i < n; i += step) {
    const double r = static_cast<double>(m->ranges[static_cast<std::size_t>(i)]);
    if (!std::isfinite(r) || r <= 0.0 || r < rmin || r >= rmax) {continue;}
    const double a =
      static_cast<double>(m->angle_min) +
      static_cast<double>(m->angle_increment) * static_cast<double>(i);
    if (!std::isfinite(a)) {continue;}
    const double px = r * std::cos(a);
    const double py = r * std::sin(a);
    double bx = 0.0;
    double by = 0.0;
    if (use_3d_tf) {
      const Eigen::Vector3d p_base = R_bl * Eigen::Vector3d(px, py, 0.0) + t_bl;
      bx = p_base.x();
      by = p_base.y();
    } else {
      bx = lx + lc * px - ls * py;
      by = ly + ls * px + lc * py;
    }
    if (beam_dt > 0.0) {
      const Vec2 deskewed = deskew_point(
        Vec2(bx, by), static_cast<double>(i) * beam_dt, vx, vy, wz);
      scan.points.emplace_back(deskewed.x(), deskewed.y());
    } else {
      scan.points.emplace_back(bx, by);
    }
  }

  if (static_cast<int>(scan.points.size()) < min_pts) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "dropping scan with %zu valid points (< min_scan_points=%d)",
      scan.points.size(), min_pts);
    return;
  }

  // 3) 按激光时间戳查询 odom -> base_link（优先 TF2，回退 /odom 插值）
  Pose2 odom_at_scan{};
  bool odom_stale = false;
  const bool have_odom = lookupOdomAt(scan_stamp, odom_at_scan, odom_stale);

  OutputSnapshot snap;
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (!active_.load() || !map_received_ || !core_) {return;}
    if (have_odom) {
      core_->on_odom(odom_at_scan);
      first_odom_fed_ = true;
    }
    const rclcpp::Time eff_scan_stamp =
      scan_stamp.nanoseconds() == 0 ? now() : scan_stamp;
    active_scan_stamp_s_ = eff_scan_stamp.seconds();
    last_scan_ = scan;
    has_last_scan_ = true;

    const CoreOutput out = core_->on_scan(scan);
    if (out.state == LocState::TRACKING && out.accepted) {
      global_retry_gate_.reset();
    }
    if (blackbox_) {
      const rclcpp::Time eff_stamp =
        scan_stamp.nanoseconds() == 0 ? now() : scan_stamp;
      blackbox_->record_scan(eff_stamp.seconds(), scan, out);
    }
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

  publishOutputs(snap, scan_stamp.nanoseconds() == 0 ? now() : scan_stamp);
}

void LocalizerNode::onInitialPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m)
{
  if (!m) {return;}
  std::lock_guard<std::mutex> lk(state_mtx_);
  if (!core_) {return;}
  if (!m->header.frame_id.empty() && m->header.frame_id != map_frame_) {
    RCLCPP_WARN(
      get_logger(),
      "ignoring initialpose in frame '%s' (expected '%s')",
      m->header.frame_id.c_str(), map_frame_.c_str());
    return;
  }

  const auto & p = m->pose.pose.position;
  const Pose2 pose(p.x, p.y, quatToYaw(m->pose.pose.orientation));

  constexpr int kIdx[3] = {0, 1, 5};
  Cov3 cov = Cov3::Zero();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      cov(i, j) =
        m->pose.covariance[static_cast<std::size_t>(6 * kIdx[i] + kIdx[j])];
    }
  }

  core_->set_initial_pose(pose, cov);
  if (global_worker_) {
    global_worker_->invalidate();
  }
  global_retry_gate_.reset();
  pending_hypotheses_.clear();
  global_alternates_.clear();
  stable_tracking_count_ = 0;
  RCLCPP_INFO(
    get_logger(), "initial pose set: (%.2f, %.2f, %.1f deg)",
    pose.x, pose.y, pose.theta * kDegPerRad);
}

GlobalResult LocalizerNode::runGlobalMultiSearchLocked(const Scan & scan)
{
  GlobalResult out;
  if (!field_ || !matcher_) {return out;}

  // 1) 外部假设优先：保留全部通过门槛的外部候选，单峰直接秒锁、对称多峰自动进入 Core 消歧
  if (!pending_hypotheses_.empty()) {
    MatchOptions vo = core_cfg_.match_opt;
    vo.window_lin = kExternalHypWindowLin;
    vo.window_ang = kExternalHypWindowAng;
    struct ExtCand { double score; Pose2 pose; };
    std::vector<ExtCand> verified;
    double best_s = -1.0;
    for (const auto & h : pending_hypotheses_) {
      const auto r = matcher_->match(*field_, h, scan, vo);
      if (r.valid && std::isfinite(r.score)) {
        const double vis = (core_cfg_.use_visibility_check && scan.points.size() >= 16) ?
          beam_visibility_consistency(*field_, r.pose, scan, true) : 1.0;
        const double eff_s = r.score * vis;
        best_s = std::max(best_s, eff_s);
        if (eff_s >= core_cfg_.hyp_min_score) {
          verified.push_back({eff_s, r.pose});
        }
      }
    }
    pending_hypotheses_.clear();
    std::sort(verified.begin(), verified.end(),
      [](const ExtCand & a, const ExtCand & b) {return a.score > b.score;});
    if (!verified.empty() && verified[0].score >= kExternalHypMinScore) {
      out.valid = true;
      out.pose = verified[0].pose;
      out.score = verified[0].score;
      global_alternates_.clear();
      for (std::size_t i = 1; i < verified.size(); ++i) {
        out.alternates.push_back(verified[i].pose);
        if (std::hypot(
            verified[i].pose.x - out.pose.x,
            verified[i].pose.y - out.pose.y) > kAlternateMinSepM)
        {
          global_alternates_.push_back(verified[i].pose);
        }
      }
      RCLCPP_INFO(
        get_logger(),
        "external hypothesis VERIFIED: (%.2f, %.2f, %.1f deg) score=%.3f (+%zu alternates)",
        out.pose.x, out.pose.y, out.pose.theta * kDegPerRad, out.score,
        out.alternates.size());
      return out;
    }
    RCLCPP_WARN(
      get_logger(),
      "external hypotheses rejected (best score %.3f < %.2f), fallback to B&B",
      best_s, kExternalHypMinScore);
  }

  // 2) 优先走非阻塞异步全局重定位线程（避免持 state_mtx_ 阻塞主回调）
  GlobalResult g;
  if (global_async_ && global_worker_) {
    const double cur_stamp_s =
      active_scan_stamp_s_ > 0.0 ? active_scan_stamp_s_ : now().seconds();
    GlobalWorkerResult wres;
    if (global_worker_->poll_result(wres) && wres.result.valid) {
      g = std::move(wres.result);
      // 补偿异步搜索耗时期间的机器人运动增量，避免运动中返回陈旧位姿
      Pose2 odom_delta{};
      if (odom_sync_.compute_delta(wres.stamp_s, cur_stamp_s, odom_delta)) {
        OdomSyncBuffer::extrapolate_global_result(g, odom_delta);
      }
    } else {
      Pose2 cur_odom{};
      const bool has_cur = odom_sync_.latest_pose(cur_odom);
      if (global_retry_gate_.should_submit(
          cur_stamp_s, has_cur, cur_odom, global_worker_->is_busy(),
          global_retry_cooldown_s_, global_retry_move_m_))
      {
        global_worker_->submit(cur_stamp_s, field_, scan);
        global_retry_gate_.mark_submitted(cur_stamp_s, has_cur, cur_odom);
      }
      return out;
    }
  } else {
    // 同步回退：透传墙钟预算 max_time_ms 锁死长尾耗时
    const double ang_step = compute_adaptive_ang_step(*field_, scan);
    g = global_correlative(
      *field_, scan, 3, ang_step, 8, global_max_time_ms_);
  }
  if (!g.valid) {return out;}

  global_alternates_.clear();
  for (const auto & a : g.alternates) {
    const double d = std::hypot(a.x - g.pose.x, a.y - g.pose.y);
    if (d > kAlternateMinSepM) {
      global_alternates_.push_back(a);
    }
  }
  RCLCPP_INFO(
    get_logger(),
    "global search: (%.2f, %.2f, %.1f deg) score=%.3f (%zu alternates forwarded to core)",
    g.pose.x, g.pose.y, g.pose.theta * kDegPerRad, g.score,
    g.alternates.size());
  return g;
}

void LocalizerNode::onGlobalRelocalize(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lk(state_mtx_);
  if (!core_) {
    res->success = false;
    res->message = "not configured";
    return;
  }
  const bool ok = core_->request_global();
  if (ok) {
    stable_tracking_count_ = 0;
    if (global_worker_) {
      global_worker_->invalidate();
    }
    global_retry_gate_.reset();
    if (blackbox_) {
      const double trigger_s =
        active_scan_stamp_s_ > 0.0 ? active_scan_stamp_s_ : now().seconds();
      blackbox_->trigger(trigger_s, BlackboxTrigger::kExternalRequest);
    }
  }
  res->success = ok;
  res->message = ok ?
    "global relocalization requested" :
    "global relocalization rejected (not ready or already active)";
}

void LocalizerNode::onSetFineMode(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  std::lock_guard<std::mutex> lk(state_mtx_);
  if (!core_) {
    res->success = false;
    res->message = "not configured";
    return;
  }
  core_->set_options(req->data ? fine_cfg_ : core_cfg_);
  fine_on_ = req->data;
  res->success = true;
  res->message = req->data ?
    "fine mode (docking) enabled" : "fine mode disabled";
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

void LocalizerNode::onNoMotionUpdate(
  const std_srvs::srv::Empty::Request::SharedPtr,
  std_srvs::srv::Empty::Response::SharedPtr)
{
  if (!active_.load()) {return;}

  const rclcpp::Time update_stamp = now();
  OutputSnapshot snap;
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (!active_.load() || !map_received_ || !core_ || !has_last_scan_) {
      return;
    }
    active_scan_stamp_s_ = update_stamp.seconds();
    const CoreOutput out = core_->on_scan(last_scan_);
    if (blackbox_) {
      blackbox_->record_scan(active_scan_stamp_s_, last_scan_, out);
    }
    snap = buildSnapshotLocked(out, false);
  }
  publishOutputs(snap, update_stamp);
}

void LocalizerNode::onTfKeepaliveTimer()
{
  if (!active_.load()) {return;}

  Pose2 mo{};
  double tol = 0.1;
  std::string map_frame;
  std::string odom_frame;
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (!active_.load() || !tf_broadcast_ || !map_received_ || !core_ ||
      core_->state() == LocState::UNINIT)
    {
      return;
    }
    mo = core_->map_to_odom();
    tol = transform_tolerance_;
    map_frame = map_frame_;
    odom_frame = odom_frame_;
  }

  if (!tf_broadcaster_) {return;}
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = now() + rclcpp::Duration::from_seconds(tol);
  tf.header.frame_id = map_frame;
  tf.child_frame_id = odom_frame;
  tf.transform.translation.x = mo.x;
  tf.transform.translation.y = mo.y;
  tf.transform.rotation.z = std::sin(0.5 * mo.theta);
  tf.transform.rotation.w = std::cos(0.5 * mo.theta);
  tf_broadcaster_->sendTransform(tf);
}

LocalizerNode::OutputSnapshot LocalizerNode::buildSnapshotLocked(
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
}

void LocalizerNode::publishOutputs(
  const OutputSnapshot & snap, const rclcpp::Time & stamp)
{
  if (!active_.load()) {return;}

  // 1) TF: map -> odom
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

  // 2) PoseWithCovarianceStamped（标准 6×6 槽位：{0, 1, 5} -> (x, y, yaw)）
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

  // 3) Hypotheses PoseArray（用于 RViz 可视化）
  if (hypotheses_pub_ && hypotheses_pub_->is_activated()) {
    geometry_msgs::msg::PoseArray pa;
    pa.header.stamp = stamp;
    pa.header.frame_id = snap.map_frame;
    pa.poses.reserve(snap.hypotheses.size());
    for (const auto & h : snap.hypotheses) {
      geometry_msgs::msg::Pose pose_msg;
      pose_msg.position.x = h.x;
      pose_msg.position.y = h.y;
      pose_msg.orientation.z = std::sin(0.5 * h.theta);
      pose_msg.orientation.w = std::cos(0.5 * h.theta);
      pa.poses.push_back(pose_msg);
    }
    hypotheses_pub_->publish(pa);
  }

  // 4) DiagnosticArray（使用标准 DiagnosticStatus 常量）
  if (status_pub_ && status_pub_->is_activated()) {
    using diagnostic_msgs::msg::DiagnosticStatus;
    diagnostic_msgs::msg::DiagnosticArray da;
    da.header.stamp = stamp;
    DiagnosticStatus st;
    st.name = "scan_localizer";
    st.hardware_id = snap.base_frame;
    switch (snap.out.state) {
      case LocState::TRACKING:
        st.level = snap.odom_stale ? DiagnosticStatus::WARN :
          DiagnosticStatus::OK;
        st.message = snap.odom_stale ? "TRACKING (odom stale)" : "TRACKING";
        break;
      case LocState::DEGRADED:
        st.level = DiagnosticStatus::WARN;
        st.message = "DEGRADED";
        break;
      case LocState::LOST:
        st.level = DiagnosticStatus::ERROR;
        st.message = "LOST";
        break;
      case LocState::GLOBAL:
        st.level = DiagnosticStatus::WARN;
        st.message = "GLOBAL";
        break;
      default:
        st.level = DiagnosticStatus::STALE;
        st.message = "UNINIT";
        break;
    }
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "score";
    kv.value = std::to_string(snap.out.score);
    st.values.push_back(kv);
    kv.key = "inlier";
    kv.value = std::to_string(snap.out.inlier_ratio);
    st.values.push_back(kv);
    kv.key = "accepted";
    kv.value = snap.out.accepted ? "1" : "0";
    st.values.push_back(kv);
    kv.key = "reject_streak";
    kv.value = std::to_string(snap.out.reject_streak);
    st.values.push_back(kv);
    kv.key = "reject_flags";
    kv.value = std::to_string(snap.out.diag.reject_flags);
    st.values.push_back(kv);
    kv.key = "raw_score";
    kv.value = std::to_string(snap.out.diag.raw_score);
    st.values.push_back(kv);
    kv.key = "wall_visibility";
    kv.value = std::to_string(snap.out.diag.wall_visibility);
    st.values.push_back(kv);
    kv.key = "frontier_visibility";
    kv.value = std::to_string(snap.out.diag.frontier_visibility);
    st.values.push_back(kv);
    kv.key = "short_ratio";
    kv.value = std::to_string(snap.out.diag.short_ratio);
    st.values.push_back(kv);
    kv.key = "mahal_dist";
    kv.value = std::to_string(snap.out.diag.mahal_dist);
    st.values.push_back(kv);
    kv.key = "fine_mode";
    kv.value = snap.fine_on ? "1" : "0";
    st.values.push_back(kv);
    kv.key = "odom_stale";
    kv.value = snap.odom_stale ? "1" : "0";
    st.values.push_back(kv);
    kv.key = "tf_broadcast";
    kv.value = snap.tf_broadcast ? "1" : "0";
    st.values.push_back(kv);
    da.status.push_back(st);
    status_pub_->publish(da);
  }
}

}  // namespace sloc

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sloc::LocalizerNode)
