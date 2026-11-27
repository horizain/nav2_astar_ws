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

#pragma once
// ============================================================================
// LocalizerNode —— ROS 2 生命周期节点（可组合，Nav2 对齐）
//
// 输入（均为可配相对话题，支持命名空间隔离）：
//   map                          nav_msgs/OccupancyGrid（transient_local，支持动态重载）
//   scan                         sensor_msgs/LaserScan（SensorDataQoS，支持 TF2 3D 外参）
//   odom                         nav_msgs/Odometry（优先查 TF2 odom->base，回退插值缓冲）
//   imu                          sensor_msgs/Imu（SensorDataQoS，陀螺航向融合）
//   initialpose                  geometry_msgs/PoseWithCovarianceStamped
//   localizer/map_path           std_msgs/String（YAML 地图热切换）
//   localizer/global_hypotheses  geometry_msgs/PoseArray（外部全局假设注入）
// 输出：
//   TF: map → odom（扫描时刻对齐 + 可配置保活定时器 + tf_broadcast 开关）
//   localizer/pose               geometry_msgs/PoseWithCovarianceStamped（标准 6×6 协方差）
//   localizer/status             diagnostic_msgs/DiagnosticArray
//   localizer/hypotheses         geometry_msgs/PoseArray（当前位姿与候选假设可视化）
// 服务：
//   localizer/global_relocalize  std_srvs/Trigger（手动触发全局重定位）
//   localizer/set_fine_mode      std_srvs/SetBool（精修/对接模式切换）
//   localizer/nomotion_update    std_srvs/Empty（静止强制更新）
// ============================================================================
#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "scan_localizer/async_global_worker.hpp"
#include "scan_localizer/blackbox_recorder.hpp"
#include "scan_localizer/localizer_core.hpp"
#include "scan_localizer/matchers.hpp"

namespace sloc
{

class LocalizerNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit LocalizerNode(const rclcpp::NodeOptions & opts);

  // 生命周期回调
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  struct TimedPose
  {
    rclcpp::Time stamp;
    Pose2 pose;
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
  };

  struct OutputSnapshot
  {
    CoreOutput out;
    Pose2 map_to_odom;
    bool publish_tf{false};
    bool fine_on{false};
    bool odom_stale{false};
    bool tf_broadcast{true};
    double transform_tolerance{0.1};
    std::string map_frame;
    std::string odom_frame;
    std::string base_frame;
    std::vector<Pose2> hypotheses;
  };

  // 工具方法
  static double quatToYaw(const geometry_msgs::msg::Quaternion & q);
  static bool isValidMatcherName(const std::string & name);
  static CoreConfig buildFineConfig(const CoreConfig & base);
  Pose2 odomToPose(const nav_msgs::msg::Odometry & m) const;

  // 参数读取、校验与动态更新
  bool readAndValidateParameters();
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & params);

  // 资源统一清理（cleanup / shutdown / configure 失败回滚共用）
  void resetResources();

  // 统一建图与切图入口（锁外构场与匹配器，加锁原子交换）
  bool applyMap(std::shared_ptr<OccupancyGrid> grid, bool keep_pose);
  bool loadMapFromYaml(const std::string & yaml_path);

  // Odom 查询：优先 TF2，回退带时间戳环形缓冲插值
  bool lookupOdomAt(const rclcpp::Time & stamp, Pose2 & out, bool & stale);

  // 回调
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void onMapPath(const std_msgs::msg::String::SharedPtr msg);
  void onHypotheses(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void onInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void onGlobalRelocalize(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);
  void onSetFineMode(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);
  void onNoMotionUpdate(
    const std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res);
  void onTfKeepaliveTimer();

  // 执行一次多峰全局搜索（在持有 state_mtx_ 的上下文中调用，候选交由 Core 统一裁决）
  GlobalResult runGlobalMultiSearchLocked(const Scan & scan);

  // 构造快照并在锁外发布 TF / 位姿 / 诊断 / 假设
  OutputSnapshot buildSnapshotLocked(
    const CoreOutput & out, bool odom_stale) const;
  void publishOutputs(
    const OutputSnapshot & snap, const rclcpp::Time & stamp);

  // ---------- 生命周期与并发 ----------
  std::atomic<bool> active_{false};
  mutable std::mutex state_mtx_;
  rclcpp::CallbackGroup::SharedPtr sensor_cb_group_;
  rclcpp::CallbackGroup::SharedPtr work_cb_group_;

  // ---------- 参数 ----------
  std::string map_topic_{"map"};
  std::string scan_topic_{"scan"};
  std::string odom_topic_{"odom"};
  std::string imu_topic_{"imu"};
  std::string initialpose_topic_{"initialpose"};
  std::string pose_topic_{"localizer/pose"};
  std::string status_topic_{"localizer/status"};
  std::string hypotheses_topic_{"localizer/hypotheses"};
  std::string global_hypotheses_topic_{"localizer/global_hypotheses"};
  std::string map_path_topic_{"localizer/map_path"};
  std::string global_relocalize_service_{"localizer/global_relocalize"};
  std::string set_fine_mode_service_{"localizer/set_fine_mode"};
  std::string nomotion_update_service_{"localizer/nomotion_update"};

  std::string base_frame_{"base_link"};
  std::string odom_frame_{"odom"};
  std::string map_frame_{"map"};
  std::string matcher_name_{"composed"};
  double likelihood_sigma_{0.20};
  double ndt_voxel_{0.5};
  bool use_global_search_{true};
  bool global_async_{true};
  double global_max_time_ms_{250.0};
  int global_worker_nice_{10};
  bool blackbox_enabled_{false};
  std::string blackbox_output_dir_{"/tmp/scan_localizer_blackbox"};
  double transform_tolerance_{0.1};
  bool tf_broadcast_{true};
  double tf_publish_rate_{10.0};
  double tf_lookup_timeout_{0.05};
  int scan_subsample_{1};
  int min_scan_points_{10};
  double range_max_{0.0};    // 0 = 用消息自带 range_max
  double max_imu_dt_{0.5};
  double odom_timeout_{1.0};
  double laser_x_{0.0};
  double laser_y_{0.0};
  double laser_theta_{0.0};
  bool map_keep_pose_{true};
  std::string map_yaml_{};

  // ---------- 算法与内部状态（受 state_mtx_ 保护） ----------
  CoreConfig core_cfg_{};
  CoreConfig fine_cfg_{};
  bool fine_on_{false};
  bool map_received_{false};
  int stable_tracking_count_{0};

  // 辅助函数与状态
  void estimateTwistAtLocked(
    const rclcpp::Time & stamp, double & vx, double & vy, double & wz) const;

  double imu_yaw_{0.0};
  double last_imu_wz_{0.0};
  bool imu_started_{false};
  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};

  std::deque<TimedPose> odom_buffer_;
  bool has_odom_sample_{false};
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};

  Scan last_scan_{};
  bool has_last_scan_{false};

  std::vector<Pose2> pending_hypotheses_;
  std::vector<Pose2> global_alternates_;

  std::shared_ptr<MapField> field_;
  std::shared_ptr<OccupancyGrid> grid_;
  std::shared_ptr<Matcher> matcher_;
  std::unique_ptr<LocalizerCore> core_;
  std::unique_ptr<AsyncGlobalWorker> global_worker_;
  std::unique_ptr<BlackboxRecorder> blackbox_;

  // ---------- ROS 实体 ----------
  rclcpp::TimerBase::SharedPtr autostart_timer_;
  rclcpp::TimerBase::SharedPtr tf_keepalive_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    param_cb_handle_;

  rclcpp_lifecycle::LifecyclePublisher<
    geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    geometry_msgs::msg::PoseArray>::SharedPtr hypotheses_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr hyp_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initpose_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr global_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr fine_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr nomotion_srv_;
};

}  // namespace sloc
