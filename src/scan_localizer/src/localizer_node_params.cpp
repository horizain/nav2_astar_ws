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
// LocalizerNode 参数声明、读取校验与动态热更新（从 localizer_node.cpp 拆分）
// ============================================================================
#include "scan_localizer/localizer_node.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace sloc
{
namespace
{

bool isFinitePositive(double v)
{
  return std::isfinite(v) && v > 0.0;
}

bool isFiniteNonNegative(double v)
{
  return std::isfinite(v) && v >= 0.0;
}

}  // namespace

void LocalizerNode::declareParameters()
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
  declare_parameter<bool>("global_async", global_async_);
  declare_parameter<double>("global_max_time_ms", global_max_time_ms_);
  declare_parameter<int>("global_worker_nice", global_worker_nice_);
  declare_parameter<double>("global_retry_cooldown_s", global_retry_cooldown_s_);
  declare_parameter<double>("global_retry_move_m", global_retry_move_m_);
  // P0 协方差下限、高置信度 kMahal 软门控与里程计跳变防护
  declare_parameter<double>("min_pred_trans_var", core_cfg_.min_pred_trans_var);
  declare_parameter<double>("min_pred_rot_var", core_cfg_.min_pred_rot_var);
  declare_parameter<double>("mahal_soft_min_score", core_cfg_.mahal_soft_min_score);
  declare_parameter<double>("mahal_soft_min_inlier", core_cfg_.mahal_soft_min_inlier);
  declare_parameter<double>("mahal_soft_max_cond", core_cfg_.mahal_soft_max_cond);
  declare_parameter<double>("odom_jump_max_trans", core_cfg_.odom_jump_max_trans);
  declare_parameter<double>("odom_jump_max_rot", core_cfg_.odom_jump_max_rot);
  declare_parameter<bool>("blackbox_enabled", blackbox_enabled_);
  declare_parameter<std::string>("blackbox_output_dir", blackbox_output_dir_);

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
  const bool global_async = get_parameter("global_async").as_bool();
  const double global_max_time_ms =
    get_parameter("global_max_time_ms").as_double();
  const int global_worker_nice =
    static_cast<int>(get_parameter("global_worker_nice").as_int());
  const double global_retry_cooldown_s =
    get_parameter("global_retry_cooldown_s").as_double();
  const double global_retry_move_m =
    get_parameter("global_retry_move_m").as_double();
  const bool blackbox_enabled = get_parameter("blackbox_enabled").as_bool();
  const std::string blackbox_output_dir =
    get_parameter("blackbox_output_dir").as_string();
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
    !isFiniteNonNegative(global_max_time_ms) ||
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
  global_async_ = global_async;
  global_max_time_ms_ = global_max_time_ms;
  global_worker_nice_ = global_worker_nice;
  global_retry_cooldown_s_ = std::max(0.0, global_retry_cooldown_s);
  global_retry_move_m_ = std::max(0.0, global_retry_move_m);
  blackbox_enabled_ = blackbox_enabled;
  blackbox_output_dir_ = blackbox_output_dir;
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

}  // namespace sloc
