// Copyright (c) 2026
// CBF 安全过滤器 ROS 节点
//
// 定位：独立的安全层节点，插在速度链路里
//   velocity_smoother → cmd_vel_smoothed → [本节点] → cmd_vel_cbf → collision_monitor → cmd_vel
// 任何标称源（DWB/MPPI/遥控/学习策略）的指令都要过这道闸。
//
// 与 nav2 自带 collision_monitor 的分工（docs/17 §5）：
//   collision_monitor = 最后防线（碰撞在即 → 停/慢，行为是离散的）
//   CBF 过滤器        = 平滑约束（连续地把指令投影到安全域，不惊扰上层）
//
// 失效安全（fail-safe）设计：
//   - enabled=false       → 纯透传（用于 before/after 对比实验）
//   - scan 超时           → 停车（安全层"失明"时绝不放行）
//   - 前后夹击无解        → 停车（cbf_core 的 infeasible 分支）

#include "nav2_controller_lab/cbf_core.hpp"

#include <chrono>
#include <mutex>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace nav2_controller_lab
{

class CbfSafetyFilter : public rclcpp::Node
{
public:
  explicit CbfSafetyFilter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("cbf_safety_filter", options)
  {
    declare_parameter<bool>("enabled", true);
    declare_parameter<double>("d_safe", 0.22);
    declare_parameter<double>("alpha", 2.0);
    declare_parameter<double>("max_linear_velocity", 0.5);
    declare_parameter<double>("min_linear_velocity", -0.5);
    declare_parameter<double>("scan_influence_range", 1.2);
    declare_parameter<double>("lidar_offset_x", -0.064);
    declare_parameter<double>("lidar_offset_y", 0.0);
    declare_parameter<double>("scan_timeout", 0.5);
    declare_parameter<std::string>("in_topic", "cmd_vel_smoothed");
    declare_parameter<std::string>("out_topic", "cmd_vel_cbf");
    declare_parameter<std::string>("scan_topic", "scan");

    get_parameter("enabled", enabled_);

    CbfParams params;
    get_parameter("d_safe", params.d_safe);
    get_parameter("alpha", params.alpha);
    get_parameter("max_linear_velocity", params.max_linear_velocity);
    get_parameter("min_linear_velocity", params.min_linear_velocity);
    get_parameter("scan_influence_range", params.scan_influence_range);
    get_parameter("lidar_offset_x", params.lidar_offset_x);
    get_parameter("lidar_offset_y", params.lidar_offset_y);
    filter_ = std::make_unique<CbfFilter>(params);

    std::string in_topic, out_topic, scan_topic;
    get_parameter("in_topic", in_topic);
    get_parameter("out_topic", out_topic);
    get_parameter("scan_topic", scan_topic);
    get_parameter("scan_timeout", scan_timeout_);

    // /scan 是传感器话题：桥接端是 best_effort，订阅端必须匹配
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      std::bind(&CbfSafetyFilter::onScan, this, std::placeholders::_1));
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      in_topic, 10,
      std::bind(&CbfSafetyFilter::onCmdVel, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(out_topic, 10);

    RCLCPP_INFO(
      get_logger(),
      "CBFSafetyFilter: %s, in=%s out=%s scan=%s d_safe=%.2f alpha=%.1f",
      enabled_ ? "ENABLED" : "BYPASS", in_topic.c_str(), out_topic.c_str(),
      scan_topic.c_str(), params.d_safe, params.alpha);
  }

private:
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan_points_.clear();
    scan_stamp_ = msg->header.stamp;
    have_scan_ = true;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r <= msg->range_min || r >= msg->range_max) {
        continue;
      }
      const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      scan_points_.emplace_back(angle, static_cast<double>(r));
    }
  }

  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    bool enabled;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      enabled = enabled_;
    }
    if (!enabled) {
      // 旁路模式：纯透传（before/after 对比实验用）
      cmd_pub_->publish(*msg);
      return;
    }

    // 失明保护：一段时间没收到 scan → 停车。安全层不能在"看不见"时放行。
    const bool scan_fresh = have_scan_ &&
      (now() - rclcpp::Time(scan_stamp_)) < rclcpp::Duration::from_seconds(scan_timeout_);
    if (!scan_fresh) {
      geometry_msgs::msg::Twist stop;
      cmd_pub_->publish(stop);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "CBFSafetyFilter: scan 数据超时（>%.1fs），fail-safe 停车", scan_timeout_);
      return;
    }

    std::vector<std::pair<double, double>> scan;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      scan = scan_points_;
    }

    const auto result = filter_->filter(msg->linear.x, msg->angular.z, scan);
    if (result.infeasible) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "CBFSafetyFilter: 前后夹击无可行速度，停车 (min_h=%.2f)", result.min_h);
    } else if (result.active) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "CBFSafetyFilter: 拦截 v %.2f -> %.2f (min_h=%.2f, 束缚点 #%d)",
        msg->linear.x, result.linear, result.min_h, result.binding_index);
    }

    geometry_msgs::msg::Twist out = *msg;
    out.linear.x = result.linear;
    out.angular.z = result.angular;
    cmd_pub_->publish(out);
  }

  std::unique_ptr<CbfFilter> filter_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  std::mutex scan_mutex_;
  std::vector<std::pair<double, double>> scan_points_;
  builtin_interfaces::msg::Time scan_stamp_;
  bool have_scan_{false};
  bool enabled_{true};
  double scan_timeout_{0.5};
};

}  // namespace nav2_controller_lab

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_controller_lab::CbfSafetyFilter>());
  rclcpp::shutdown();
  return 0;
}
