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
// LocalizerSim —— 端到端仿真源（集成测试 / 演示）
// 发布 /map（latched）、/odom（带 2% 滑移 + odom->base_link TF）、/imu、
// /scan（仓库 104m 环路 + 人群段遮挡），并订阅 localizer/pose 统计定位误差。
// ============================================================================
#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "scan_localizer/sim_world.hpp"

namespace sloc
{

class LocalizerSim : public rclcpp::Node {
public:
  LocalizerSim()
  : Node("scan_localizer_sim")
  {
    publish_laser_tf_ = declare_parameter<bool>("publish_laser_tf", false);
    laser_frame_ = declare_parameter<std::string>("laser_frame", "base_link");

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local());
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS());
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS());
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localizer/pose", 20,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
        onEstimatedPose(m);
      });
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    grid_ = std::make_shared<OccupancyGrid>(sim::make_warehouse(0.05));
    field_pub_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {publishMapOnce();});

    // 与基准 E9/E10 共用 104m 矩形环路（2 圈 × 52m，步长 0.25m）
    traj_ = sim::make_warehouse_trajectory(2, 0.25);
    odom_ = traj_.front();

    sim_timer_ = create_wall_timer(
      std::chrono::milliseconds(250),  // 0.25s/步
      [this]() {step();});
    const double total_dist =
      traj_.empty() ? 0.0 : 0.25 * static_cast<double>(traj_.size() - 1);
    RCLCPP_INFO(
      get_logger(),
      "sim started: %zu steps (%.0f m), publishing /map /odom /imu /scan + TF",
      traj_.size() - 1, total_dist);
  }

private:
  static double quatToYaw(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny, cosy);
  }

  void publishMapOnce()
  {
    if (map_sent_) {return;}
    nav_msgs::msg::OccupancyGrid m;
    m.header.stamp = now();
    m.header.frame_id = "map";
    m.info.resolution = static_cast<float>(grid_->resolution);
    m.info.width = static_cast<std::uint32_t>(grid_->width);
    m.info.height = static_cast<std::uint32_t>(grid_->height);
    // 左下角原点约定（与 OccupancyGrid T1.2 一致）
    m.info.origin.position.x = grid_->origin_x;
    m.info.origin.position.y = grid_->origin_y;
    m.info.origin.orientation.w = 1.0;
    m.data = grid_->cells;
    map_pub_->publish(m);
    map_sent_ = true;
    if (field_pub_timer_) {
      field_pub_timer_->cancel();
    }
  }

  void onEstimatedPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (traj_.empty()) {return;}
    const Pose2 & gt = traj_[std::min(k_, traj_.size() - 1)];
    const double ex = msg->pose.pose.position.x - gt.x;
    const double ey = msg->pose.pose.position.y - gt.y;
    const double pos_err = std::hypot(ex, ey);
    const double yaw_err = std::abs(
      wrap_angle(quatToYaw(msg->pose.pose.orientation) - gt.theta));
    pos_errs_.push_back(pos_err);
    yaw_errs_.push_back(yaw_err);
  }

  void printSummary()
  {
    if (pos_errs_.empty()) {
      RCLCPP_INFO(
        get_logger(), "sim done (%zu steps, no /localizer/pose received)", k_);
      return;
    }
    auto sorted_pos = pos_errs_;
    std::sort(sorted_pos.begin(), sorted_pos.end());
    double pos_sum = 0.0;
    double yaw_sum = 0.0;
    double yaw_max = 0.0;
    for (double e : pos_errs_) {pos_sum += e;}
    for (double a : yaw_errs_) {
      yaw_sum += a;
      yaw_max = std::max(yaw_max, a);
    }
    const double pos_mean = pos_sum / static_cast<double>(pos_errs_.size());
    const double pos_med = sorted_pos[sorted_pos.size() / 2];
    const double pos_max = sorted_pos.back();
    const double yaw_mean_deg =
      (yaw_sum / static_cast<double>(yaw_errs_.size())) * 180.0 / kPi;
    const double yaw_max_deg = yaw_max * 180.0 / kPi;
    RCLCPP_INFO(
      get_logger(),
      "sim done (%zu steps, %zu poses): pos mean=%.4fm med=%.4fm max=%.4fm | "
      "yaw mean=%.2fdeg max=%.2fdeg",
      k_, pos_errs_.size(), pos_mean, pos_med, pos_max,
      yaw_mean_deg, yaw_max_deg);
  }

  void step()
  {
    if (k_ + 1 >= traj_.size()) {
      printSummary();
      sim_timer_->cancel();
      return;
    }
    ++k_;
    constexpr double kDt = 0.25;
    const rclcpp::Time stamp = now();
    const Pose2 prev = traj_[k_ - 1];
    const Pose2 gt = traj_[k_];

    // 本体系真实增量（使用 wrap_angle 避免拐角 -3π/2 跳变）
    const double c = std::cos(prev.theta);
    const double s = std::sin(prev.theta);
    const double mx = gt.x - prev.x;
    const double my = gt.y - prev.y;
    const Pose2 d_local(
      c * mx + s * my, -s * mx + c * my, wrap_angle(gt.theta - prev.theta));

    // 真实尺度轮式滑移噪声（2% 平移尺度 + 5mm 随机噪声，1% 旋转尺度 + 2mrad 噪声）
    std::normal_distribution<double> gn_xy(0.0, 0.005);
    std::normal_distribution<double> gn_th(0.0, 0.002);
    const Pose2 d_odom(
      d_local.x * 1.02 + gn_xy(rng_),
      d_local.y * 1.02 + gn_xy(rng_),
      d_local.theta * 1.01 + gn_th(rng_));
    odom_ = odom_ * d_odom;

    // 广播 odom -> base_link TF（与 /odom 共用同一时间戳）
    geometry_msgs::msg::TransformStamped tf_odom;
    tf_odom.header.stamp = stamp;
    tf_odom.header.frame_id = "odom";
    tf_odom.child_frame_id = "base_link";
    tf_odom.transform.translation.x = odom_.x;
    tf_odom.transform.translation.y = odom_.y;
    tf_odom.transform.rotation.z = std::sin(0.5 * odom_.theta);
    tf_odom.transform.rotation.w = std::cos(0.5 * odom_.theta);
    tf_broadcaster_->sendTransform(tf_odom);

    if (publish_laser_tf_ && laser_frame_ != "base_link") {
      geometry_msgs::msg::TransformStamped tf_laser;
      tf_laser.header.stamp = stamp;
      tf_laser.header.frame_id = "base_link";
      tf_laser.child_frame_id = laser_frame_;
      tf_laser.transform.rotation.w = 1.0;
      tf_broadcaster_->sendTransform(tf_laser);
    }

    // /odom（含速度与非零协方差）
    nav_msgs::msg::Odometry om;
    om.header.stamp = stamp;
    om.header.frame_id = "odom";
    om.child_frame_id = "base_link";
    om.pose.pose.position.x = odom_.x;
    om.pose.pose.position.y = odom_.y;
    om.pose.pose.orientation.z = std::sin(0.5 * odom_.theta);
    om.pose.pose.orientation.w = std::cos(0.5 * odom_.theta);
    om.pose.covariance[0] = 1.0e-3;
    om.pose.covariance[7] = 1.0e-3;
    om.pose.covariance[35] = 1.0e-3;
    om.twist.twist.linear.x = d_odom.x / kDt;
    om.twist.twist.linear.y = d_odom.y / kDt;
    om.twist.twist.angular.z = d_odom.theta / kDt;
    om.twist.covariance[0] = 4.0e-4;
    om.twist.covariance[7] = 4.0e-4;
    om.twist.covariance[35] = 1.0e-4;
    odom_pub_->publish(om);

    // 真值日志（每 4 步 = 1s，供外部对拍）
    if (k_ % 4 == 0) {
      RCLCPP_INFO(
        get_logger(), "SIMGT %.3f %.3f %.3f", gt.x, gt.y, gt.theta);
    }

    // IMU：真实角速度 + MEMS 白噪声（非零协方差，orientation/accel 标记 -1 不可用）
    sensor_msgs::msg::Imu im;
    im.header.stamp = stamp;
    im.header.frame_id = "base_link";
    std::normal_distribution<double> mn(0.0, 0.002);
    im.angular_velocity.z = d_local.theta / kDt + mn(rng_);
    im.orientation_covariance[0] = -1.0;
    im.linear_acceleration_covariance[0] = -1.0;
    im.angular_velocity_covariance[8] = 4.0e-6;
    imu_pub_->publish(im);

    // 人群段遮挡（每 4 步一帧扫描 → 步 150-190 对应 10 帧遮挡）
    std::vector<sim::DynBox> dyn;
    if (k_ > 150 && k_ < 190) {
      dyn.push_back({gt.x + 1.5 * std::cos(gt.theta),
          gt.y + 1.5 * std::sin(gt.theta), 0.35, 0.35});
      dyn.push_back({gt.x + 2.8 * std::cos(gt.theta + 0.4),
          gt.y + 2.8 * std::sin(gt.theta + 0.4), 0.35, 0.35});
    }
    if (k_ % 4 == 0) {
      constexpr int kBeams = 720;
      const double angle_min = -kPi;
      const double angle_inc = kTwoPi / static_cast<double>(kBeams);
      sensor_msgs::msg::LaserScan ls;
      ls.header.stamp = stamp;
      ls.header.frame_id = laser_frame_;
      ls.angle_min = static_cast<float>(angle_min);
      ls.angle_increment = static_cast<float>(angle_inc);
      ls.angle_max = static_cast<float>(
        angle_min + static_cast<double>(kBeams - 1) * angle_inc);
      ls.scan_time = 0.1f;
      ls.time_increment = ls.scan_time / static_cast<float>(kBeams);
      ls.range_min = 0.10f;
      ls.range_max = 12.0f;
      // 直接按 LaserScan 光束角发射射线，消除重分箱造成的 -0.25° 角度偏移
      ls.ranges = sim::make_laser_ranges(
        *grid_, gt, angle_min, angle_inc, kBeams,
        0.10, 12.0, 0.01, rng_, dyn);
      scan_pub_->publish(ls);
    }
  }

  bool publish_laser_tf_{false};
  std::string laser_frame_{"base_link"};

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr field_pub_timer_, sim_timer_;

  std::shared_ptr<OccupancyGrid> grid_;
  std::vector<Pose2> traj_;
  Pose2 odom_{};
  std::size_t k_{0};
  bool map_sent_{false};
  std::mt19937 rng_{7};
  std::vector<double> pos_errs_;
  std::vector<double> yaw_errs_;
};

}  // namespace sloc

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sloc::LocalizerSim>());
  rclcpp::shutdown();
  return 0;
}
