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

#include <cmath>
#include <lifecycle_msgs/msg/state.hpp>

#include "scan_localizer/map_io.hpp"

namespace sloc
{

LocalizerNode::LocalizerNode(const rclcpp::NodeOptions & opts)
: rclcpp_lifecycle::LifecycleNode("scan_localizer", opts)
{
  declare_parameter<std::string>("map_topic", map_topic_);
  declare_parameter<std::string>("scan_topic", scan_topic_);
  declare_parameter<std::string>("odom_topic", odom_topic_);
  declare_parameter<std::string>("base_frame", base_frame_);
  declare_parameter<std::string>("odom_frame", odom_frame_);
  declare_parameter<std::string>("map_frame", map_frame_);
  declare_parameter<std::string>("matcher", matcher_name_);
  declare_parameter<double>("likelihood_sigma", likelihood_sigma_);
  declare_parameter<double>("ndt_voxel", ndt_voxel_);
  declare_parameter<bool>("use_global_search", use_global_search_);
  declare_parameter<double>("transform_tolerance", transform_tolerance_);
  declare_parameter<int>("scan_subsample", scan_subsample_);
  declare_parameter<double>("range_max_override", range_max_);
    // 传感器校准（激光在 base 系安装位姿；无 TF 静态发布时用参数）
  declare_parameter<double>("laser_x", laser_x_);
  declare_parameter<double>("laser_y", laser_y_);
  declare_parameter<double>("laser_theta", laser_theta_);
    // IMU
  declare_parameter<bool>("use_imu", core_cfg_.use_imu_yaw);
    // 地图管理
  declare_parameter<std::string>("map_yaml", map_yaml_);
  declare_parameter<bool>("map_keep_pose", map_keep_pose_);
  declare_parameter<double>("min_inlier_ratio", core_cfg_.min_inlier_ratio);
  declare_parameter<double>("min_score", core_cfg_.min_score);
  declare_parameter<double>("mahal_gate", core_cfg_.mahal_gate);
  declare_parameter<double>("max_cov_eig", core_cfg_.max_cov_eig);
  declare_parameter<int>("degraded_after", core_cfg_.degraded_after);
  declare_parameter<int>("lost_after", core_cfg_.lost_after);
  declare_parameter<double>("match_window_lin",
                              core_cfg_.match_opt.window_lin);
  declare_parameter<double>("match_window_ang",
                              core_cfg_.match_opt.window_ang);
  declare_parameter<double>("match_step_lin", core_cfg_.match_opt.step_lin);
  declare_parameter<double>("match_step_ang", core_cfg_.match_opt.step_ang);
  declare_parameter<double>("huber_delta", core_cfg_.match_opt.huber_delta);
  declare_parameter<double>("trim_dist", core_cfg_.match_opt.trim_dist);

  map_topic_ = get_parameter("map_topic").as_string();
  scan_topic_ = get_parameter("scan_topic").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  odom_frame_ = get_parameter("odom_frame").as_string();
  map_frame_ = get_parameter("map_frame").as_string();
  matcher_name_ = get_parameter("matcher").as_string();
  likelihood_sigma_ = get_parameter("likelihood_sigma").as_double();
  ndt_voxel_ = get_parameter("ndt_voxel").as_double();
  use_global_search_ = get_parameter("use_global_search").as_bool();
  transform_tolerance_ = get_parameter("transform_tolerance").as_double();
  scan_subsample_ = get_parameter("scan_subsample").as_int();
  range_max_ = get_parameter("range_max_override").as_double();
  core_cfg_.min_inlier_ratio =
    get_parameter("min_inlier_ratio").as_double();
  core_cfg_.min_score = get_parameter("min_score").as_double();
  core_cfg_.mahal_gate = get_parameter("mahal_gate").as_double();
  core_cfg_.max_cov_eig = get_parameter("max_cov_eig").as_double();
  core_cfg_.degraded_after = get_parameter("degraded_after").as_int();
  core_cfg_.lost_after = get_parameter("lost_after").as_int();
  core_cfg_.match_opt.window_lin =
    get_parameter("match_window_lin").as_double();
  core_cfg_.match_opt.window_ang =
    get_parameter("match_window_ang").as_double();
  core_cfg_.match_opt.step_lin =
    get_parameter("match_step_lin").as_double();
  core_cfg_.match_opt.step_ang =
    get_parameter("match_step_ang").as_double();
  core_cfg_.match_opt.huber_delta =
    get_parameter("huber_delta").as_double();
  core_cfg_.match_opt.trim_dist = get_parameter("trim_dist").as_double();
  laser_x_ = get_parameter("laser_x").as_double();
  laser_y_ = get_parameter("laser_y").as_double();
  laser_theta_ = get_parameter("laser_theta").as_double();
  core_cfg_.use_imu_yaw = get_parameter("use_imu").as_bool();
  map_yaml_ = get_parameter("map_yaml").as_string();
  map_keep_pose_ = get_parameter("map_keep_pose").as_bool();

    // 精修模式配置（对接/充电桩：窗口收紧 + 截断更狠 + 迭代加深）
  fine_cfg_ = core_cfg_;
  fine_cfg_.match_opt.window_lin = 0.10;
  fine_cfg_.match_opt.window_ang = 0.05;
  fine_cfg_.match_opt.huber_delta = 0.05;
  fine_cfg_.match_opt.trim_dist = 0.20;
  fine_cfg_.match_opt.max_iterations = 30;
  fine_cfg_.min_inlier_ratio = 0.75;

    // autostart：构造后立即 configure+activate（演示/测试用；
    // 生产部署置 false，由 Nav2 lifecycle manager 驱动状态迁移）
  declare_parameter<bool>("autostart", true);
  if (get_parameter("autostart").as_bool()) {
    autostart_timer_ = create_wall_timer(
            std::chrono::milliseconds(0), [this]() {
        autostart_timer_->cancel();
        if (get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
        {
          on_configure(get_current_state());
          on_activate(get_current_state());
        }
            });
  }
}

Pose2 LocalizerNode::odomToPose(const nav_msgs::msg::Odometry & m) const
{
  const auto & p = m.pose.pose.position;
  const auto & q = m.pose.pose.orientation;
  return {p.x, p.y, std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
}

LocalizerNode::CallbackReturn LocalizerNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "configuring scan_localizer (matcher=%s)",
                matcher_name_.c_str());

  core_ = std::make_unique<LocalizerCore>(core_cfg_);
  core_->set_global_fn([this](const Scan & s, Pose2 & out, double & sc) {
      return use_global_search_ && runGlobalSearch(s, out, sc);
    });

  pose_pub_ = create_publisher<
    geometry_msgs::msg::PoseWithCovarianceStamped>("localizer/pose", 10);
  status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "localizer/status", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // map：latched QoS
  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.transient_local();
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, map_qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr m) {
      onMap(m);
        });
    // scan：传感器数据 best-effort
  rclcpp::QoS scan_qos(rclcpp::KeepLast(5));
  scan_qos.best_effort();
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, scan_qos,
    [this](const sensor_msgs::msg::LaserScan::SharedPtr m) {
      onScan(m);
        });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 50,
    [this](const nav_msgs::msg::Odometry::SharedPtr m) {onOdom(m);});
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 100,
    [this](const sensor_msgs::msg::Imu::SharedPtr m) {onImu(m);});
    // 地图热切换（业务层）：传 yaml 路径，重建后按策略保位姿或重定位
  map_path_sub_ = create_subscription<std_msgs::msg::String>(
        "localizer/map_path", 10,
    [this](const std_msgs::msg::String::SharedPtr m) {
      onMapPath(m);
        });
    // 外部全局假设（视觉/二维码/UWB 入口）：激光验证后采纳
  hyp_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
        "localizer/global_hypotheses", 10,
    [this](const geometry_msgs::msg::PoseArray::SharedPtr m) {
      onHypotheses(m);
        });
  initpose_sub_ =
    create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 1,
    [this](
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
      m) {onInitialPose(m);});
  global_srv_ = create_service<std_srvs::srv::Trigger>(
        "localizer/global_relocalize",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res) {
      onGlobalRelocalize(req, res);
        });
  fine_srv_ = create_service<std_srvs::srv::SetBool>(
        "localizer/set_fine_mode",
    [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res) {
      onSetFineMode(req, res);
        });

  RCLCPP_INFO(get_logger(),
                "configured. waiting for map on '%s' (latched) and scans on "
                "'%s'",
                map_topic_.c_str(), scan_topic_.c_str());

    // 业务直载：map_yaml 参数非空 → 免 map_server 直接装图
  if (!map_yaml_.empty()) {
    if (loadMapFromYaml(map_yaml_)) {
      RCLCPP_INFO(get_logger(), "map loaded from yaml: %s",
                        map_yaml_.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "map_yaml load FAILED: %s",
                         map_yaml_.c_str());
    }
  }
  return CallbackReturn::SUCCESS;
}

LocalizerNode::CallbackReturn LocalizerNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  pose_pub_->on_activate();
  status_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "activated");
  return CallbackReturn::SUCCESS;
}
LocalizerNode::CallbackReturn LocalizerNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  pose_pub_->on_deactivate();
  status_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}
LocalizerNode::CallbackReturn LocalizerNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  map_sub_.reset();
  scan_sub_.reset();
  odom_sub_.reset();
  initpose_sub_.reset();
  global_srv_.reset();
  fine_srv_.reset();
  tf_broadcaster_.reset();
  core_.reset();
  field_.reset();
  grid_.reset();
  matcher_.reset();
  map_received_ = false;
  return CallbackReturn::SUCCESS;
}
LocalizerNode::CallbackReturn LocalizerNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

void LocalizerNode::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr m)
{
  if (map_received_) {return;}      // 静态地图：只取第一张
  grid_ = std::make_shared<OccupancyGrid>();
  grid_->width = static_cast<int>(m->info.width);
  grid_->height = static_cast<int>(m->info.height);
  grid_->resolution = m->info.resolution;
  grid_->origin_x = m->info.origin.position.x;
  grid_->origin_y = m->info.origin.position.y;
  grid_->cells = m->data;

  field_ = std::make_shared<MapField>();
  if (!field_->build(grid_, likelihood_sigma_)) {
    RCLCPP_ERROR(get_logger(), "MapField::build failed for /map");
    return;
  }
  matcher_ = make_matcher(matcher_name_, grid_.get(), ndt_voxel_);
  if (!matcher_) {
    RCLCPP_ERROR(get_logger(), "unknown matcher '%s'",
                     matcher_name_.c_str());
    return;
  }
  core_->set_map(field_);
  core_->set_matcher(matcher_);
  map_received_ = true;
  RCLCPP_INFO(get_logger(),
                "map received: %dx%d @ %.3fm, MapField built (sigma=%.2f)",
                grid_->width, grid_->height, grid_->resolution,
                likelihood_sigma_);
}

void LocalizerNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr m)
{
  if (!core_) {return;}
  core_->on_odom(odomToPose(*m));
}

void LocalizerNode::onImu(const sensor_msgs::msg::Imu::SharedPtr m)
{
  if (!core_) {return;}
  const rclcpp::Time stamp(m->header.stamp);
  if (!imu_started_) {
    imu_started_ = true;
    last_imu_stamp_ = stamp;
    return;      // 首帧只建时间基准
  }
  const double dt = std::min(
        std::max((stamp - last_imu_stamp_).seconds(), 1e-4), 0.5);
  last_imu_stamp_ = stamp;
    // 陀螺积分（任意固定系；core 用帧间差分，慢漂移自动消除）
  imu_yaw_ += m->angular_velocity.z * dt;
  core_->on_imu_yaw(imu_yaw_, dt);
}

bool LocalizerNode::loadMapFromYaml(const std::string & yaml_path)
{
  auto grid = std::make_shared<OccupancyGrid>();
  std::string err;
  if (!load_map_yaml(yaml_path, *grid, err)) {
    RCLCPP_ERROR(get_logger(), "load_map_yaml(%s): %s", yaml_path.c_str(),
                     err.c_str());
    return false;
  }
  grid_ = grid;
  field_ = std::make_shared<MapField>();
  if (!field_->build(grid_, likelihood_sigma_)) {
    RCLCPP_ERROR(get_logger(), "MapField::build failed for yaml: %s",
                     yaml_path.c_str());
    return false;
  }
  matcher_ = make_matcher(matcher_name_, grid_.get(), ndt_voxel_);
  if (!matcher_) {
    RCLCPP_ERROR(get_logger(), "unknown matcher '%s'",
                     matcher_name_.c_str());
    return false;
  }
  const bool first = !map_received_;
  core_->set_map(field_);
  core_->set_matcher(matcher_);
  map_received_ = true;
  RCLCPP_INFO(get_logger(),
                "map loaded: %dx%d @ %.3fm origin(%.2f,%.2f)%s", grid_->width,
                grid_->height, grid_->resolution, grid_->origin_x,
                grid_->origin_y, first ? "" : " [switched]");
    // 切图策略：同系坐标系 → 保位姿续跟；跨层/坐标系重建 → 全局重定位
  if (!first) {
    if (map_keep_pose_) {
      RCLCPP_INFO(get_logger(), "map switched, pose kept (TRACKING)");
    } else {
      core_->request_global();
      RCLCPP_INFO(get_logger(), "map switched, global relocalize armed");
    }
  }
  return true;
}

void LocalizerNode::onMapPath(const std_msgs::msg::String::SharedPtr m)
{
  RCLCPP_INFO(get_logger(), "map switch requested: %s", m->data.c_str());
  loadMapFromYaml(m->data);
}

void LocalizerNode::onHypotheses(
  const geometry_msgs::msg::PoseArray::SharedPtr m)
{
  pending_hypotheses_.clear();
  for (const auto & p : m->poses) {
    const double yaw = std::atan2(2.0 * (p.orientation.w * p.orientation.z +
        p.orientation.x * p.orientation.y),
                                      1.0 - 2.0 * (p.orientation.y *
        p.orientation.y +
        p.orientation.z *
        p.orientation.z));
    pending_hypotheses_.push_back({p.position.x, p.position.y, yaw});
  }
  if (!pending_hypotheses_.empty()) {
    RCLCPP_INFO(get_logger(), "external hypotheses: %zu",
                    pending_hypotheses_.size());
  }
}

Pose2 LocalizerNode::scanToSensorFrame(
  const sensor_msgs::msg::LaserScan & m) const
{
    // 预留：激光雷达安装位姿应从 TF(base_link→laser) 获取。
    // 产品部署时由参数 laser_x/laser_y/laser_theta 提供，简化为恒等。
  (void)m;
  return {0.0, 0.0, 0.0};
}

void LocalizerNode::onScan(const sensor_msgs::msg::LaserScan::SharedPtr m)
{
  if (!map_received_ || !core_) {return;}

    // LaserScan → 点云（激光系 → base 系，含安装位姿补偿）
  Scan scan;
  const double rmax =
    range_max_ > 0.0 ?
    range_max_ :
    (m->range_max > 0.0 ? m->range_max : 12.0);
  const int n = static_cast<int>(m->ranges.size());
  const int step = std::max(1, scan_subsample_);
  const double lc = std::cos(laser_theta_), ls = std::sin(laser_theta_);
  for (int i = 0; i < n; i += step) {
    const double r = m->ranges[static_cast<std::size_t>(i)];
    if (!std::isfinite(r) || r < m->range_min || r >= rmax) {continue;}
    const double a =
      m->angle_min + m->angle_increment * static_cast<double>(i);
    const double px = r * std::cos(a), py = r * std::sin(a);
    scan.points.emplace_back(laser_x_ + lc * px - ls * py,
                                 laser_y_ + ls * px + lc * py);
  }
  if (scan.points.size() < 10) {return;}

  const auto out = core_->on_scan(scan);
  publishOutputs(out, m->header.stamp);
}

void LocalizerNode::onInitialPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m)
{
  if (!core_) {return;}
  const auto & p = m->pose.pose.position;
  const auto & q = m->pose.pose.orientation;
  const Pose2 pose(p.x, p.y,
    std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
  core_->set_initial_pose(pose);
  RCLCPP_INFO(get_logger(), "initial pose set: (%.2f, %.2f, %.1f deg)",
                pose.x, pose.y, pose.theta * 180.0 / M_PI);
}

bool LocalizerNode::runGlobalSearch(
  const Scan & scan, Pose2 & out,
  double & score)
{
  if (!field_ || !matcher_) {return false;}
    // 1) 外部假设优先（视觉/二维码/UWB）：激光强验证（假设本应粗准，
    //    过不了高分门槛视为过期/错误 → 回退 B&B，绝不误采纳）
  if (!pending_hypotheses_.empty() && field_ && matcher_) {
    MatchOptions vo = core_cfg_.match_opt;
    vo.window_lin = 0.30;
    vo.window_ang = 0.20;
    double best_s = -1.0;
    Pose2 best_p{};
    for (const auto & h : pending_hypotheses_) {
      const auto r = matcher_->match(*field_, h, scan, vo);
      if (r.score > best_s) {
        best_s = r.score;
        best_p = r.pose;
      }
    }
    pending_hypotheses_.clear();
        // 强门槛：真假设（粗准）应得高分；过期/错误假设直接落选
    if (best_s >= 0.85) {
      out = best_p;
      score = best_s;
      RCLCPP_INFO(get_logger(),
                        "external hypothesis VERIFIED: (%.2f, %.2f, %.1f "
                        "deg) score=%.3f",
                        best_p.x, best_p.y, best_p.theta * 180.0 / M_PI,
                        best_s);
      return true;
    }
    RCLCPP_WARN(get_logger(),
                    "external hypotheses rejected (best score %.3f < 0.85), "
                    "fallback to B&B",
                    best_s);
  }
    // 2) 回退：B&B 激光全局搜索
  const auto g = global_correlative(*field_, scan, 3,
                                      2.0 * M_PI / 180.0, 5);
  if (!g.valid) {return false;}
  out = g.pose;
  score = g.score;
    // top-K 假设挂起：若 top-1 是对称结构鬼影，跟踪失败 → LOST →
    // 下一轮全局先消费备选（激光验证），无需重跑 B&B
  for (const auto & a : g.alternates) {
    const double d = std::hypot(a.x - g.pose.x, a.y - g.pose.y);
    if (d > 0.3) {      // 只保留显著不同的备选（同簇的重复解丢弃）
      pending_hypotheses_.push_back(a);
    }
  }
  RCLCPP_INFO(get_logger(),
                "global search: (%.2f, %.2f, %.1f deg) score=%.3f "
                "(%zu alternates queued)",
                g.pose.x, g.pose.y, g.pose.theta * 180.0 / M_PI, g.score,
                pending_hypotheses_.size());
  return true;
}

void LocalizerNode::onGlobalRelocalize(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  if (!core_) {
    res->success = false;
    res->message = "not configured";
    return;
  }
  const bool ok = core_->request_global();
  res->success = ok;
  res->message = ok ? "global relocalization requested" :
    "global relocalization rejected (not ready or already active)";
}

void LocalizerNode::onSetFineMode(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  if (!core_) {
    res->success = false;
    res->message = "not configured";
    return;
  }
  core_->set_options(req->data ? fine_cfg_ : core_cfg_);
  fine_on_ = req->data;
  res->success = true;
  res->message = req->data ? "fine mode (docking) enabled" :
    "fine mode disabled";
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

void LocalizerNode::publishOutputs(
  const CoreOutput & out,
  const rclcpp::Time & stamp)
{
    // TF: map → odom（有效期加 tolerance，杜绝"TF extrapolation"抖动）
  if (core_->state() != LocState::UNINIT) {
    const Pose2 mo = core_->map_to_odom();
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp + rclcpp::Duration::from_seconds(
                                      transform_tolerance_);
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    tf.transform.translation.x = mo.x;
    tf.transform.translation.y = mo.y;
    const double th = mo.theta;
    tf.transform.rotation.z = std::sin(0.5 * th);
    tf.transform.rotation.w = std::cos(0.5 * th);
    tf_broadcaster_->sendTransform(tf);
  }

    // pose with covariance
  geometry_msgs::msg::PoseWithCovarianceStamped pm;
  pm.header.stamp = stamp;
  pm.header.frame_id = map_frame_;
  pm.pose.pose.position.x = out.pose.x;
  pm.pose.pose.position.y = out.pose.y;
  pm.pose.pose.orientation.z = std::sin(0.5 * out.pose.theta);
  pm.pose.pose.orientation.w = std::cos(0.5 * out.pose.theta);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      pm.pose.covariance[static_cast<std::size_t>(6 * i + j)] =
        out.covariance(i, j);
    }
  }
  pose_pub_->publish(pm);

    // 状态诊断
  diagnostic_msgs::msg::DiagnosticArray da;
  da.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "scan_localizer";
  st.hardware_id = base_frame_;
  switch (out.state) {
    case LocState::TRACKING: st.level = 0; st.message = "TRACKING"; break;
    case LocState::DEGRADED: st.level = 1; st.message = "DEGRADED"; break;
    case LocState::LOST:     st.level = 2; st.message = "LOST"; break;
    case LocState::GLOBAL:   st.level = 1; st.message = "GLOBAL"; break;
    default:                 st.level = 3; st.message = "UNINIT"; break;
  }
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "score";    kv.value = std::to_string(out.score);      st.values.push_back(kv);
  kv.key = "inlier";    kv.value = std::to_string(out.inlier_ratio); st.values.push_back(kv);
  kv.key = "accepted";  kv.value = out.accepted ? "1" : "0";      st.values.push_back(kv);
  kv.key = "reject_streak"; kv.value = std::to_string(out.reject_streak); st.values.push_back(kv);
  kv.key = "fine_mode"; kv.value = fine_on_ ? "1" : "0";        st.values.push_back(kv);
  da.status.push_back(st);
  status_pub_->publish(da);
}

}  // namespace sloc

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sloc::LocalizerNode)
