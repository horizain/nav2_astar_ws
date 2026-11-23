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

constexpr std::size_t kMaxOdomBufferSize = 200;
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

bool isFiniteNonNegative(double v)
{
  return std::isfinite(v) && v >= 0.0;
}

}  // namespace

LocalizerNode::LocalizerNode(const rclcpp::NodeOptions & opts)
: rclcpp_lifecycle::LifecycleNode("scan_localizer", opts)
{
  // 话题与服务（默认相对名，支持命名空间隔离）
  declare_parameter<std::string>("map_topic", map_topic_);
  declare_parameter<std::string>("scan_topic", scan_topic_);
  declare_parameter<std::string>("odom_topic", odom_topic_);
  declare_parameter<std::string>("imu_topic", imu_topic_);
  declare_parameter<std::string>("initialpose_topic", initialpose_topic_);
  declare_parameter<std::string>("pose_topic", pose_topic_);
  declare_parameter<std::string>("status_topic", status_topic_);
  declare_parameter<std::string>("hypotheses_topic", hypotheses_topic_);
  declare_parameter<std::string>(
    "global_hypotheses_topic", global_hypotheses_topic_);
  declare_parameter<std::string>("map_path_topic", map_path_topic_);
  declare_parameter<std::string>(
    "global_relocalize_service", global_relocalize_service_);
  declare_parameter<std::string>(
    "set_fine_mode_service", set_fine_mode_service_);
  declare_parameter<std::string>(
    "nomotion_update_service", nomotion_update_service_);

  // 坐标系
  declare_parameter<std::string>("base_frame", base_frame_);
  declare_parameter<std::string>("odom_frame", odom_frame_);
  declare_parameter<std::string>("map_frame", map_frame_);

  // 匹配器与地图场
  declare_parameter<std::string>("matcher", matcher_name_);
  declare_parameter<double>("likelihood_sigma", likelihood_sigma_);
  declare_parameter<double>("ndt_voxel", ndt_voxel_);
  declare_parameter<bool>("use_global_search", use_global_search_);

  // TF 与扫描预处理
  declare_parameter<double>("transform_tolerance", transform_tolerance_);
  declare_parameter<bool>("tf_broadcast", tf_broadcast_);
  declare_parameter<double>("tf_publish_rate", tf_publish_rate_);
  declare_parameter<double>("tf_lookup_timeout", tf_lookup_timeout_);
  declare_parameter<int>("scan_subsample", scan_subsample_);
  declare_parameter<int>("min_scan_points", min_scan_points_);
  declare_parameter<double>("range_max_override", range_max_);
  declare_parameter<double>("max_imu_dt", max_imu_dt_);
  declare_parameter<double>("odom_timeout", odom_timeout_);

  // 传感器外参回退（无 TF 时使用）
  declare_parameter<double>("laser_x", laser_x_);
  declare_parameter<double>("laser_y", laser_y_);
  declare_parameter<double>("laser_theta", laser_theta_);

  // 地图管理
  declare_parameter<std::string>("map_yaml", map_yaml_);
  declare_parameter<bool>("map_keep_pose", map_keep_pose_);

  // 门限与状态机
  declare_parameter<double>("min_inlier_ratio", core_cfg_.min_inlier_ratio);
  declare_parameter<double>("min_score", core_cfg_.min_score);
  declare_parameter<double>("mahal_gate", core_cfg_.mahal_gate);
  declare_parameter<double>("max_cov_eig", core_cfg_.max_cov_eig);
  declare_parameter<int>("degraded_after", core_cfg_.degraded_after);
  declare_parameter<int>("lost_after", core_cfg_.lost_after);

  // 预测噪声与 IMU 融合
  declare_parameter<bool>("use_imu", core_cfg_.use_imu_yaw);
  declare_parameter<double>(
    "odom_trans_var_per_m", core_cfg_.odom_trans_var_per_m);
  declare_parameter<double>(
    "odom_rot_var_per_rad", core_cfg_.odom_rot_var_per_rad);
  declare_parameter<double>(
    "odom_rot_var_per_m", core_cfg_.odom_rot_var_per_m);
  declare_parameter<double>(
    "imu_rot_var_per_rad", core_cfg_.imu_rot_var_per_rad);

  // 匹配器选项
  declare_parameter<double>("match_window_lin", core_cfg_.match_opt.window_lin);
  declare_parameter<double>("match_window_ang", core_cfg_.match_opt.window_ang);
  declare_parameter<double>("match_step_lin", core_cfg_.match_opt.step_lin);
  declare_parameter<double>("match_step_ang", core_cfg_.match_opt.step_ang);
  declare_parameter<double>("huber_delta", core_cfg_.match_opt.huber_delta);
  declare_parameter<double>("trim_dist", core_cfg_.match_opt.trim_dist);

  // autostart：默认 false 以适配 Nav2 lifecycle_manager；演示/测试可在 launch 覆写 true
  declare_parameter<bool>("autostart", false);

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

bool LocalizerNode::readAndValidateParameters()
{
  const std::string map_topic = get_parameter("map_topic").as_string();
  const std::string scan_topic = get_parameter("scan_topic").as_string();
  const std::string odom_topic = get_parameter("odom_topic").as_string();
  const std::string imu_topic = get_parameter("imu_topic").as_string();
  const std::string initialpose_topic =
    get_parameter("initialpose_topic").as_string();
  const std::string pose_topic = get_parameter("pose_topic").as_string();
  const std::string status_topic = get_parameter("status_topic").as_string();
  const std::string hypotheses_topic =
    get_parameter("hypotheses_topic").as_string();
  const std::string global_hypotheses_topic =
    get_parameter("global_hypotheses_topic").as_string();
  const std::string map_path_topic =
    get_parameter("map_path_topic").as_string();
  const std::string global_srv =
    get_parameter("global_relocalize_service").as_string();
  const std::string fine_srv =
    get_parameter("set_fine_mode_service").as_string();
  const std::string nomotion_srv =
    get_parameter("nomotion_update_service").as_string();

  const std::string base_frame = get_parameter("base_frame").as_string();
  const std::string odom_frame = get_parameter("odom_frame").as_string();
  const std::string map_frame = get_parameter("map_frame").as_string();
  const std::string matcher_name = get_parameter("matcher").as_string();

  const double likelihood_sigma = get_parameter("likelihood_sigma").as_double();
  const double ndt_voxel = get_parameter("ndt_voxel").as_double();
  const bool use_global_search = get_parameter("use_global_search").as_bool();
  const double transform_tolerance =
    get_parameter("transform_tolerance").as_double();
  const bool tf_broadcast = get_parameter("tf_broadcast").as_bool();
  const double tf_publish_rate = get_parameter("tf_publish_rate").as_double();
  const double tf_lookup_timeout =
    get_parameter("tf_lookup_timeout").as_double();
  const int scan_subsample =
    static_cast<int>(get_parameter("scan_subsample").as_int());
  const int min_scan_points =
    static_cast<int>(get_parameter("min_scan_points").as_int());
  const double range_max = get_parameter("range_max_override").as_double();
  const double max_imu_dt = get_parameter("max_imu_dt").as_double();
  const double odom_timeout = get_parameter("odom_timeout").as_double();
  const double laser_x = get_parameter("laser_x").as_double();
  const double laser_y = get_parameter("laser_y").as_double();
  const double laser_theta = get_parameter("laser_theta").as_double();
  const std::string map_yaml = get_parameter("map_yaml").as_string();
  const bool map_keep_pose = get_parameter("map_keep_pose").as_bool();

  CoreConfig cfg{};
  cfg.min_inlier_ratio = get_parameter("min_inlier_ratio").as_double();
  cfg.min_score = get_parameter("min_score").as_double();
  cfg.mahal_gate = get_parameter("mahal_gate").as_double();
  cfg.max_cov_eig = get_parameter("max_cov_eig").as_double();
  cfg.degraded_after =
    static_cast<int>(get_parameter("degraded_after").as_int());
  cfg.lost_after = static_cast<int>(get_parameter("lost_after").as_int());
  cfg.use_imu_yaw = get_parameter("use_imu").as_bool();
  cfg.odom_trans_var_per_m =
    get_parameter("odom_trans_var_per_m").as_double();
  cfg.odom_rot_var_per_rad =
    get_parameter("odom_rot_var_per_rad").as_double();
  cfg.odom_rot_var_per_m = get_parameter("odom_rot_var_per_m").as_double();
  cfg.imu_rot_var_per_rad = get_parameter("imu_rot_var_per_rad").as_double();
  cfg.match_opt.window_lin = get_parameter("match_window_lin").as_double();
  cfg.match_opt.window_ang = get_parameter("match_window_ang").as_double();
  cfg.match_opt.step_lin = get_parameter("match_step_lin").as_double();
  cfg.match_opt.step_ang = get_parameter("match_step_ang").as_double();
  cfg.match_opt.huber_delta = get_parameter("huber_delta").as_double();
  cfg.match_opt.trim_dist = get_parameter("trim_dist").as_double();

  if (!isValidMatcherName(matcher_name)) {
    RCLCPP_ERROR(
      get_logger(), "invalid matcher '%s'", matcher_name.c_str());
    return false;
  }
  if (base_frame.empty() || odom_frame.empty() || map_frame.empty()) {
    RCLCPP_ERROR(get_logger(), "frame names must be non-empty");
    return false;
  }
  if (!isFinitePositive(likelihood_sigma) || !isFinitePositive(ndt_voxel) ||
    !isFiniteNonNegative(transform_tolerance) ||
    !isFiniteNonNegative(tf_publish_rate) ||
    !isFiniteNonNegative(tf_lookup_timeout) ||
    scan_subsample < 1 || min_scan_points < 3 ||
    !isFiniteNonNegative(range_max) ||
    !isFinitePositive(max_imu_dt) || !isFinitePositive(odom_timeout) ||
    !std::isfinite(laser_x) || !std::isfinite(laser_y) ||
    !std::isfinite(laser_theta))
  {
    RCLCPP_ERROR(
      get_logger(), "invalid sensor/TF numeric parameter out of range");
    return false;
  }
  if (!std::isfinite(cfg.min_inlier_ratio) || cfg.min_inlier_ratio <= 0.0 ||
    cfg.min_inlier_ratio > 1.0 ||
    !std::isfinite(cfg.min_score) || cfg.min_score < 0.0 ||
    cfg.min_score > 1.0 ||
    !isFinitePositive(cfg.mahal_gate) ||
    !isFinitePositive(cfg.max_cov_eig) ||
    cfg.degraded_after < 1 || cfg.lost_after < cfg.degraded_after ||
    !isFinitePositive(cfg.odom_trans_var_per_m) ||
    !isFinitePositive(cfg.odom_rot_var_per_rad) ||
    !isFinitePositive(cfg.odom_rot_var_per_m) ||
    !isFinitePositive(cfg.imu_rot_var_per_rad) ||
    !isFinitePositive(cfg.match_opt.window_lin) ||
    !isFinitePositive(cfg.match_opt.window_ang) ||
    !isFinitePositive(cfg.match_opt.step_lin) ||
    !isFinitePositive(cfg.match_opt.step_ang) ||
    !isFinitePositive(cfg.match_opt.huber_delta) ||
    !isFinitePositive(cfg.match_opt.trim_dist))
  {
    RCLCPP_ERROR(
      get_logger(), "invalid core/matcher numeric parameter out of range");
    return false;
  }

  std::lock_guard<std::mutex> lk(state_mtx_);
  map_topic_ = map_topic;
  scan_topic_ = scan_topic;
  odom_topic_ = odom_topic;
  imu_topic_ = imu_topic;
  initialpose_topic_ = initialpose_topic;
  pose_topic_ = pose_topic;
  status_topic_ = status_topic;
  hypotheses_topic_ = hypotheses_topic;
  global_hypotheses_topic_ = global_hypotheses_topic;
  map_path_topic_ = map_path_topic;
  global_relocalize_service_ = global_srv;
  set_fine_mode_service_ = fine_srv;
  nomotion_update_service_ = nomotion_srv;

  base_frame_ = base_frame;
  odom_frame_ = odom_frame;
  map_frame_ = map_frame;
  matcher_name_ = matcher_name;
  likelihood_sigma_ = likelihood_sigma;
  ndt_voxel_ = ndt_voxel;
  use_global_search_ = use_global_search;
  transform_tolerance_ = transform_tolerance;
  tf_broadcast_ = tf_broadcast;
  tf_publish_rate_ = tf_publish_rate;
  tf_lookup_timeout_ = tf_lookup_timeout;
  scan_subsample_ = scan_subsample;
  min_scan_points_ = min_scan_points;
  range_max_ = range_max;
  max_imu_dt_ = max_imu_dt;
  odom_timeout_ = odom_timeout;
  laser_x_ = laser_x;
  laser_y_ = laser_y;
  laser_theta_ = laser_theta;
  map_yaml_ = map_yaml;
  map_keep_pose_ = map_keep_pose;
  core_cfg_ = cfg;
  fine_cfg_ = buildFineConfig(core_cfg_);
  return true;
}

rcl_interfaces::msg::SetParametersResult LocalizerNode::onSetParameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;

  std::lock_guard<std::mutex> lk(state_mtx_);
  CoreConfig next_cfg = core_cfg_;
  bool next_use_global = use_global_search_;
  double next_tf_tol = transform_tolerance_;
  bool next_tf_broadcast = tf_broadcast_;
  double next_tf_lookup_timeout = tf_lookup_timeout_;
  int next_scan_subsample = scan_subsample_;
  int next_min_scan_points = min_scan_points_;
  double next_range_max = range_max_;
  double next_max_imu_dt = max_imu_dt_;
  double next_odom_timeout = odom_timeout_;
  double next_laser_x = laser_x_;
  double next_laser_y = laser_y_;
  double next_laser_theta = laser_theta_;
  bool next_map_keep_pose = map_keep_pose_;
  std::string next_matcher = matcher_name_;
  bool matcher_changed = false;
  bool core_changed = false;

  auto reject = [&res](const std::string & msg) {
      res.successful = false;
      res.reason = msg;
      return res;
    };

  for (const auto & p : params) {
    const std::string & name = p.get_name();
    if (name == "matcher") {
      const std::string val = p.as_string();
      if (!isValidMatcherName(val)) {
        return reject("invalid matcher name: " + val);
      }
      if (val != next_matcher) {
        next_matcher = val;
        matcher_changed = true;
      }
    } else if (name == "likelihood_sigma") {
      if (!isFinitePositive(p.as_double())) {
        return reject("likelihood_sigma must be > 0");
      }
    } else if (name == "ndt_voxel") {
      if (!isFinitePositive(p.as_double())) {
        return reject("ndt_voxel must be > 0");
      }
    } else if (name == "min_inlier_ratio") {
      const double v = p.as_double();
      if (!std::isfinite(v) || v <= 0.0 || v > 1.0) {
        return reject("min_inlier_ratio must be in (0, 1]");
      }
      next_cfg.min_inlier_ratio = v;
      core_changed = true;
    } else if (name == "min_score") {
      const double v = p.as_double();
      if (!std::isfinite(v) || v < 0.0 || v > 1.0) {
        return reject("min_score must be in [0, 1]");
      }
      next_cfg.min_score = v;
      core_changed = true;
    } else if (name == "mahal_gate") {
      const double v = p.as_double();
      if (!isFinitePositive(v)) {
        return reject("mahal_gate must be > 0");
      }
      next_cfg.mahal_gate = v;
      core_changed = true;
    } else if (name == "max_cov_eig") {
      const double v = p.as_double();
      if (!isFinitePositive(v)) {
        return reject("max_cov_eig must be > 0");
      }
      next_cfg.max_cov_eig = v;
      core_changed = true;
    } else if (name == "degraded_after") {
      const int v = static_cast<int>(p.as_int());
      if (v < 1) {
        return reject("degraded_after must be >= 1");
      }
      next_cfg.degraded_after = v;
      core_changed = true;
    } else if (name == "lost_after") {
      const int v = static_cast<int>(p.as_int());
      if (v < 1) {
        return reject("lost_after must be >= 1");
      }
      next_cfg.lost_after = v;
      core_changed = true;
    } else if (name == "odom_trans_var_per_m") {
      const double v = p.as_double();
      if (!isFinitePositive(v)) {
        return reject("odom_trans_var_per_m must be > 0");
      }
      next_cfg.odom_trans_var_per_m = v;
      core_changed = true;
    } else if (name == "odom_rot_var_per_rad") {
      const double v = p.as_double();
      if (!isFinitePositive(v)) {
        return reject("odom_rot_var_per_rad must be > 0");
      }
      next_cfg.odom_rot_var_per_rad = v;
      core_changed = true;
    } else if (name == "odom_rot_var_per_m") {
      const double v = p.as_double();
      if (!isFinitePositive(v)) {
        return reject("odom_rot_var_per_m must be > 0");
      }
      next_cfg.odom_rot_var_per_m = v;
      core_changed = true;
    } else if (name == "imu_rot_var_per_rad") {
      const double v = p.as_double();
      if (!isFinitePositive(v)) {
        return reject("imu_rot_var_per_rad must be > 0");
      }
      next_cfg.imu_rot_var_per_rad = v;
      core_changed = true;
    } else if (name == "use_imu") {
      next_cfg.use_imu_yaw = p.as_bool();
      core_changed = true;
    } else if (name == "match_window_lin") {
      const double v = p.as_double();
      if (!isFinitePositive(v)) {
        return reject("match_window_lin must be > 0");
      }
      next_cfg.match_opt.window_lin = v;
      core_changed = true;
    } else if (name == "match_window_ang") {
      const double v = p.as_double();
      if (!isFinitePositive(v)) {
        return reject("match_window_ang must be > 0");
      }
      next_cfg.match_opt.window_ang = v;
      core_changed = true;
    } else if (name == "match_step_lin") {
      const double v = p.as_double();
      if (!isFinitePositive(v)) {
        return reject("match_step_lin must be > 0");
      }
      next_cfg.match_opt.step_lin = v;
      core_changed = true;
    } else if (name == "match_step_ang") {
      const double v = p.as_double();
      if (!isFinitePositive(v)) {
        return reject("match_step_ang must be > 0");
      }
      next_cfg.match_opt.step_ang = v;
      core_changed = true;
    } else if (name == "huber_delta") {
      const double v = p.as_double();
      if (!isFinitePositive(v)) {
        return reject("huber_delta must be > 0");
      }
      next_cfg.match_opt.huber_delta = v;
      core_changed = true;
    } else if (name == "trim_dist") {
      const double v = p.as_double();
      if (!isFinitePositive(v)) {
        return reject("trim_dist must be > 0");
      }
      next_cfg.match_opt.trim_dist = v;
      core_changed = true;
    } else if (name == "transform_tolerance") {
      const double v = p.as_double();
      if (!isFiniteNonNegative(v)) {
        return reject("transform_tolerance must be >= 0");
      }
      next_tf_tol = v;
    } else if (name == "tf_broadcast") {
      next_tf_broadcast = p.as_bool();
    } else if (name == "tf_lookup_timeout") {
      const double v = p.as_double();
      if (!isFiniteNonNegative(v)) {
        return reject("tf_lookup_timeout must be >= 0");
      }
      next_tf_lookup_timeout = v;
    } else if (name == "tf_publish_rate") {
      if (!isFiniteNonNegative(p.as_double())) {
        return reject("tf_publish_rate must be >= 0");
      }
    } else if (name == "use_global_search") {
      next_use_global = p.as_bool();
    } else if (name == "scan_subsample") {
      const int v = static_cast<int>(p.as_int());
      if (v < 1) {
        return reject("scan_subsample must be >= 1");
      }
      next_scan_subsample = v;
    } else if (name == "min_scan_points") {
      const int v = static_cast<int>(p.as_int());
      if (v < 3) {
        return reject("min_scan_points must be >= 3");
      }
      next_min_scan_points = v;
    } else if (name == "range_max_override") {
      const double v = p.as_double();
      if (!isFiniteNonNegative(v)) {
        return reject("range_max_override must be >= 0");
      }
      next_range_max = v;
    } else if (name == "max_imu_dt") {
      const double v = p.as_double();
      if (!isFinitePositive(v)) {
        return reject("max_imu_dt must be > 0");
      }
      next_max_imu_dt = v;
    } else if (name == "odom_timeout") {
      const double v = p.as_double();
      if (!isFinitePositive(v)) {
        return reject("odom_timeout must be > 0");
      }
      next_odom_timeout = v;
    } else if (name == "laser_x") {
      const double v = p.as_double();
      if (!std::isfinite(v)) {
        return reject("laser_x must be finite");
      }
      next_laser_x = v;
    } else if (name == "laser_y") {
      const double v = p.as_double();
      if (!std::isfinite(v)) {
        return reject("laser_y must be finite");
      }
      next_laser_y = v;
    } else if (name == "laser_theta") {
      const double v = p.as_double();
      if (!std::isfinite(v)) {
        return reject("laser_theta must be finite");
      }
      next_laser_theta = v;
    } else if (name == "map_keep_pose") {
      next_map_keep_pose = p.as_bool();
    }
  }

  if (next_cfg.lost_after < next_cfg.degraded_after) {
    return reject("lost_after must be >= degraded_after");
  }

  if (matcher_changed && grid_) {
    auto m = make_matcher(next_matcher, grid_.get(), ndt_voxel_);
    if (!m) {
      return reject("failed to construct matcher: " + next_matcher);
    }
    matcher_ = std::move(m);
    if (core_) {
      core_->set_matcher(matcher_);
    }
  }

  matcher_name_ = next_matcher;
  core_cfg_ = next_cfg;
  fine_cfg_ = buildFineConfig(core_cfg_);
  use_global_search_ = next_use_global;
  transform_tolerance_ = next_tf_tol;
  tf_broadcast_ = next_tf_broadcast;
  tf_lookup_timeout_ = next_tf_lookup_timeout;
  scan_subsample_ = next_scan_subsample;
  min_scan_points_ = next_min_scan_points;
  range_max_ = next_range_max;
  max_imu_dt_ = next_max_imu_dt;
  odom_timeout_ = next_odom_timeout;
  laser_x_ = next_laser_x;
  laser_y_ = next_laser_y;
  laser_theta_ = next_laser_theta;
  map_keep_pose_ = next_map_keep_pose;

  if (core_changed && core_) {
    core_->set_options(fine_on_ ? fine_cfg_ : core_cfg_);
  }

  return res;
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
    grid_->width, grid_->height, grid_->resolution,
    grid_->origin_x, grid_->origin_y,
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
  rclcpp::Time stamp(m->header.stamp);
  if (stamp.nanoseconds() == 0) {
    stamp = now();
  }

  std::lock_guard<std::mutex> lk(state_mtx_);
  if (!core_) {return;}

  if (!odom_buffer_.empty() && stamp < odom_buffer_.back().stamp) {
    odom_buffer_.clear();
  }
  odom_buffer_.push_back({stamp, pose});
  while (odom_buffer_.size() > kMaxOdomBufferSize) {
    odom_buffer_.pop_front();
  }
  last_odom_stamp_ = stamp;
  has_odom_sample_ = true;
  core_->on_odom(pose);
}

void LocalizerNode::onImu(const sensor_msgs::msg::Imu::SharedPtr m)
{
  if (!m) {return;}
  const double wz = m->angular_velocity.z;
  if (!std::isfinite(wz)) {return;}
  const rclcpp::Time stamp(m->header.stamp);

  std::lock_guard<std::mutex> lk(state_mtx_);
  if (!core_) {return;}
  if (!imu_started_) {
    imu_started_ = true;
    last_imu_stamp_ = stamp;
    return;
  }
  const double dt = (stamp - last_imu_stamp_).seconds();
  if (dt <= 0.0 || dt > max_imu_dt_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "dropping IMU sample with invalid dt=%.4f s (max=%.3f s)",
      dt, max_imu_dt_);
    last_imu_stamp_ = stamp;
    return;
  }
  last_imu_stamp_ = stamp;
  imu_yaw_ += wz * dt;
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
  pending_hypotheses_.clear();
  for (const auto & p : m->poses) {
    pending_hypotheses_.emplace_back(
      p.position.x, p.position.y, quatToYaw(p.orientation));
  }
  stable_tracking_count_ = 0;
  if (!pending_hypotheses_.empty()) {
    RCLCPP_INFO(
      get_logger(), "external hypotheses received: %zu",
      pending_hypotheses_.size());
  }
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

  // 2) 回退至带时间戳的 /odom 环形缓冲插值
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

  for (std::size_t i = 0; i + 1 < odom_buffer_.size(); ++i) {
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
  return true;
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
    if (use_3d_tf) {
      const Eigen::Vector3d p_base = R_bl * Eigen::Vector3d(px, py, 0.0) + t_bl;
      scan.points.emplace_back(p_base.x(), p_base.y());
    } else {
      scan.points.emplace_back(
        lx + lc * px - ls * py,
        ly + ls * px + lc * py);
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
  pending_hypotheses_.clear();
  global_alternates_.clear();
  stable_tracking_count_ = 0;
  RCLCPP_INFO(
    get_logger(), "initial pose set: (%.2f, %.2f, %.1f deg)",
    pose.x, pose.y, pose.theta * kDegPerRad);
}

bool LocalizerNode::runGlobalSearchLocked(
  const Scan & scan, Pose2 & out, double & score)
{
  if (!field_ || !matcher_) {return false;}

  // 1) 外部假设优先（激光强验证，仅采纳 valid 且得分过门槛的假设）
  if (!pending_hypotheses_.empty()) {
    MatchOptions vo = core_cfg_.match_opt;
    vo.window_lin = kExternalHypWindowLin;
    vo.window_ang = kExternalHypWindowAng;
    double best_s = -1.0;
    Pose2 best_p{};
    bool found_valid = false;
    for (const auto & h : pending_hypotheses_) {
      const auto r = matcher_->match(*field_, h, scan, vo);
      if (r.valid && r.score > best_s) {
        best_s = r.score;
        best_p = r.pose;
        found_valid = true;
      }
    }
    pending_hypotheses_.clear();
    if (found_valid && best_s >= kExternalHypMinScore) {
      out = best_p;
      score = best_s;
      RCLCPP_INFO(
        get_logger(),
        "external hypothesis VERIFIED: (%.2f, %.2f, %.1f deg) score=%.3f",
        best_p.x, best_p.y, best_p.theta * kDegPerRad, best_s);
      return true;
    }
    RCLCPP_WARN(
      get_logger(),
      "external hypotheses rejected (best score %.3f < %.2f), fallback to B&B",
      best_s, kExternalHypMinScore);
  }

  // 2) 回退：B&B 激光全局搜索
  const auto g = global_correlative(*field_, scan, 3, 2.0 * kPi / 180.0, 5);
  if (!g.valid) {return false;}
  out = g.pose;
  score = g.score;

  global_alternates_.clear();
  for (const auto & a : g.alternates) {
    const double d = std::hypot(a.x - g.pose.x, a.y - g.pose.y);
    if (d > kAlternateMinSepM) {
      global_alternates_.push_back(a);
    }
  }
  RCLCPP_INFO(
    get_logger(),
    "global search: (%.2f, %.2f, %.1f deg) score=%.3f (%zu alternates recorded)",
    g.pose.x, g.pose.y, g.pose.theta * kDegPerRad, g.score,
    global_alternates_.size());
  return true;
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

  OutputSnapshot snap;
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    if (!active_.load() || !map_received_ || !core_ || !has_last_scan_) {
      return;
    }
    const CoreOutput out = core_->on_scan(last_scan_);
    snap = buildSnapshotLocked(out, false);
  }
  publishOutputs(snap, now());
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
