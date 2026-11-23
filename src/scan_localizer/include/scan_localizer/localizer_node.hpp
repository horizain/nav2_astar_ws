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
// LocalizerNode —— ROS 2 生命周期节点（可组合，产品化外壳）
//
// 输入（即插即用的传感器面）：
//   /map          nav_msgs/OccupancyGrid      （map_server / slam_toolbox 发布，latched）
//   /scan         sensor_msgs/LaserScan       （任意 2D 激光雷达驱动）
//   /odom         nav_msgs/Odometry           （可选；无 odom 也可跑，预测退化为静止）
//   /initialpose  geometry_msgs/PoseWithCovarianceStamped （RViz 2D Pose Estimate）
// 输出：
//   TF: map → odom（扫描时间戳精确对齐）
//   /localizer/pose     geometry_msgs/PoseWithCovarianceStamped
//   /localizer/status   diagnostic_msgs/DiagnosticArray（状态/得分/内点率/拒绝计数）
// 服务：
//   /localizer/global_relocalize  std_srvs/Trigger（手动触发全局搜索）
//   /localizer/set_fine_mode      std_srvs/SetBool（对接/精修模式：收紧门限）
// ============================================================================
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

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
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void onMapPath(const std_msgs::msg::String::SharedPtr msg);
  void onHypotheses(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void onInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void onGlobalRelocalize(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res);
  void onSetFineMode(
    const std_srvs::srv::SetBool::Request::SharedPtr,
    std_srvs::srv::SetBool::Response::SharedPtr res);

    // 执行一次全局搜索并把结果交回 core（多假设由后续帧确认）
  bool runGlobalSearch(const Scan & scan, Pose2 & out, double & score);

    // 发布 TF（map→odom）与位姿
  void publishOutputs(const CoreOutput & out, const rclcpp::Time & stamp);

    // ---------- 参数 ----------
  std::string map_topic_{"/map"};
  std::string scan_topic_{"/scan"};
  std::string odom_topic_{"/odom"};
  std::string base_frame_{"base_link"};
  std::string odom_frame_{"odom"};
  std::string map_frame_{"map"};
  std::string matcher_name_{"composed"};
  double likelihood_sigma_{0.20};
  double ndt_voxel_{0.5};
  bool use_global_search_{true};
  double transform_tolerance_{0.1};
  int scan_subsample_{1};
  double range_max_{0.0};    // 0 = 用消息自带 range_max

    // 精修模式（对接/充电桩）：收紧门限的完整配置，服务切换时整体热替换
  CoreConfig fine_cfg_{};
  bool fine_on_{false};
    // 自动配置+激活（演示/测试用；生产由 Nav2 lifecycle manager 驱动）
  rclcpp::TimerBase::SharedPtr autostart_timer_;

    // 地图管理（业务层）
  bool loadMapFromYaml(const std::string & yaml_path);   // 重建场/匹配器，可选保位姿
  bool map_keep_pose_{true};     // 切图后保留当前位姿（false → 全局重定位）
  std::string map_yaml_{};       // configure 时直接加载（免 map_server）

    // IMU 陀螺积分（任意固定系；帧间差分消漂移）
  double imu_yaw_{0.0};
  bool imu_started_{false};
  rclcpp::Time last_imu_stamp_{};

    // 外部全局假设（视觉/二维码/UWB 重定位入口），激光验证后采纳
  std::vector<Pose2> pending_hypotheses_;

    // ---------- 状态 ----------
  std::shared_ptr<MapField> field_;
  std::shared_ptr<OccupancyGrid> grid_;
  std::shared_ptr<Matcher> matcher_;
  std::unique_ptr<LocalizerCore> core_;
  CoreConfig core_cfg_;
  bool map_received_{false};

    // ---------- ROS 面 ----------
  rclcpp_lifecycle::LifecyclePublisher<
    geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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
    // 传感器校准：激光在 base 系下的安装位姿（视觉/外设同理走 TF 常规路径）
  double laser_x_{0.0}, laser_y_{0.0}, laser_theta_{0.0};

    // /odom 位姿抽取
  Pose2 odomToPose(const nav_msgs::msg::Odometry & m) const;
  Pose2 scanToSensorFrame(const sensor_msgs::msg::LaserScan & m) const;
};

}  // namespace sloc
