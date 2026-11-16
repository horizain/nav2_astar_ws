// Copyright (c) 2026
// Controller Lab 可视化节点
//
// 订阅：
//   /plan                         全局路径
//   /cmd_vel                      当前速度
//   /controller_selector          当前控制器实例名
//   /controller_lab/debug_enable  是否发布标记
//
// 发布：
//   /controller_lab/markers       visualization_msgs/MarkerArray
//
// 标记内容：
//   - 全局路径（绿色）
//   - 前视点 + 机器人到前视点的连线（黄色）
//   - 当前 cmd_vel 的预测轨迹（蓝色）
//   - 控制器名和 v/w 文字（白色）
//
// Licensed under the Apache License, Version 2.0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav2_controller_lab/path_utils.hpp"
#include "nav2_controller_lab/trajectory_utils.hpp"

namespace nav2_controller_lab_rviz
{

namespace
{

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::vector<nav2_controller_lab::Pose2D> pathToPose2D(const nav_msgs::msg::Path & path)
{
  std::vector<nav2_controller_lab::Pose2D> result;
  result.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    nav2_controller_lab::Pose2D p;
    p.x = pose.pose.position.x;
    p.y = pose.pose.position.y;
    p.yaw = yawFromQuaternion(pose.pose.orientation);
    result.push_back(p);
  }
  return result;
}

}  // namespace

class ControllerLabVisualizer : public rclcpp::Node
{
public:
  ControllerLabVisualizer()
  : Node("controller_lab_visualizer"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
    lookahead_distance_ = declare_parameter<double>("lookahead_distance", 0.6);
    prediction_horizon_ = declare_parameter<double>("prediction_horizon", 2.0);
    prediction_dt_ = declare_parameter<double>("prediction_dt", 0.1);
    const double publish_rate = declare_parameter<double>("publish_rate", 10.0);

    rclcpp::QoS selector_qos(rclcpp::KeepLast(1));
    selector_qos.transient_local().reliable();

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "controller_lab/markers", 10);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "plan", 10,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        path_ = *msg;
        path_frame_ = msg->header.frame_id;
        if (path_frame_.empty()) {
          path_frame_ = map_frame_;
        }
      });

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        linear_velocity_ = msg->linear.x;
        angular_velocity_ = msg->angular.z;
      });

    prediction_sub_ = create_subscription<nav_msgs::msg::Path>(
      "controller_lab/predicted_trajectory", 10,
      [this](nav_msgs::msg::Path::SharedPtr message) {
        solver_prediction_ = *message;
        prediction_received_ = std::chrono::steady_clock::now();
      });

    selector_sub_ = create_subscription<std_msgs::msg::String>(
      "controller_selector", selector_qos,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        selected_controller_ = msg->data;
      });

    debug_sub_ = create_subscription<std_msgs::msg::Bool>(
      "controller_lab/debug_enable", selector_qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        debug_enabled_ = msg->data;
      });

    const double rate = std::max(1.0, publish_rate);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&ControllerLabVisualizer::publishMarkers, this));

    RCLCPP_INFO(
      get_logger(),
      "ControllerLabVisualizer: lookahead=%.2f prediction=%.1fs @ %.1fHz",
      lookahead_distance_, prediction_horizon_, rate);
  }

private:
  void publishDeleteAll()
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(marker);
    marker_pub_->publish(array);
  }

  void publishMarkers()
  {
    if (!debug_enabled_) {
      if (published_markers_) {
        publishDeleteAll();
        published_markers_ = false;
      }
      return;
    }

    if (path_.poses.size() < 2) {
      return;
    }

    const std::string frame = path_frame_.empty() ? map_frame_ : path_frame_;

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        frame, robot_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "等待 TF %s -> %s: %s", frame.c_str(), robot_frame_.c_str(), ex.what());
      return;
    }

    nav2_controller_lab::Pose2D robot;
    robot.x = transform.transform.translation.x;
    robot.y = transform.transform.translation.y;
    robot.yaw = yawFromQuaternion(transform.transform.rotation);

    auto path = pathToPose2D(path_);
    nav2_controller_lab::recomputePathYaw(path);

    visualization_msgs::msg::MarkerArray array;
    const auto stamp = this->now();

    // 1. 全局路径
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame;
      marker.header.stamp = stamp;
      marker.ns = "global_path";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.025;
      marker.color.r = 0.1F;
      marker.color.g = 0.9F;
      marker.color.b = 0.2F;
      marker.color.a = 0.9F;
      marker.pose.orientation.w = 1.0;
      for (const auto & pose : path) {
        geometry_msgs::msg::Point p;
        p.x = pose.x;
        p.y = pose.y;
        p.z = 0.02;
        marker.points.push_back(p);
      }
      array.markers.push_back(marker);
    }

    // 2. 前视点
    nav2_controller_lab::Pose2D lookahead;
    const bool has_lookahead = nav2_controller_lab::findLookaheadPoint(
      robot, path, std::max(0.0, lookahead_distance_), lookahead);

    if (has_lookahead) {
      visualization_msgs::msg::Marker sphere;
      sphere.header.frame_id = frame;
      sphere.header.stamp = stamp;
      sphere.ns = "lookahead";
      sphere.id = 1;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.scale.x = 0.18;
      sphere.scale.y = 0.18;
      sphere.scale.z = 0.18;
      sphere.color.r = 1.0F;
      sphere.color.g = 0.9F;
      sphere.color.b = 0.1F;
      sphere.color.a = 1.0F;
      sphere.pose.position.x = lookahead.x;
      sphere.pose.position.y = lookahead.y;
      sphere.pose.position.z = 0.08;
      sphere.pose.orientation.w = 1.0;
      array.markers.push_back(sphere);

      visualization_msgs::msg::Marker line;
      line.header.frame_id = frame;
      line.header.stamp = stamp;
      line.ns = "lookahead_line";
      line.id = 2;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.02;
      line.color.r = 1.0F;
      line.color.g = 0.9F;
      line.color.b = 0.1F;
      line.color.a = 0.8F;
      line.pose.orientation.w = 1.0;

      geometry_msgs::msg::Point robot_point;
      robot_point.x = robot.x;
      robot_point.y = robot.y;
      robot_point.z = 0.05;
      geometry_msgs::msg::Point lookahead_point;
      lookahead_point.x = lookahead.x;
      lookahead_point.y = lookahead.y;
      lookahead_point.z = 0.05;
      line.points = {robot_point, lookahead_point};
      array.markers.push_back(line);
    }

    // 3. 当前速度的预测轨迹
    {
      const int steps = std::max(
        1, static_cast<int>(std::round(prediction_horizon_ / prediction_dt_)));
      const auto future = nav2_controller_lab::rolloutConstantControl(
        robot, linear_velocity_, angular_velocity_, steps, prediction_dt_);

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame;
      marker.header.stamp = stamp;
      marker.ns = "predicted_trajectory";
      marker.id = 3;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.04;
      marker.color.r = 0.1F;
      marker.color.g = 0.5F;
      marker.color.b = 1.0F;
      marker.color.a = 0.9F;
      marker.pose.orientation.w = 1.0;
      for (const auto & pose : future) {
        geometry_msgs::msg::Point p;
        p.x = pose.x;
        p.y = pose.y;
        p.z = 0.04;
        marker.points.push_back(p);
      }
      array.markers.push_back(marker);
    }

    // 4. 文字标签
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame;
      marker.header.stamp = stamp;
      marker.ns = "controller_info";
      marker.id = 4;
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.z = 0.25;
      marker.color.r = 1.0F;
      marker.color.g = 1.0F;
      marker.color.b = 1.0F;
      marker.color.a = 1.0F;
      marker.pose.position.x = robot.x;
      marker.pose.position.y = robot.y;
      marker.pose.position.z = 0.9;
      marker.pose.orientation.w = 1.0;
      marker.text = selected_controller_ +
        "\nv=" + std::to_string(linear_velocity_).substr(0, 5) +
        " w=" + std::to_string(angular_velocity_).substr(0, 5);
      array.markers.push_back(marker);
    }

    // 5. Actual selected MPC sequence. Blue remains constant-command extrapolation.
    visualization_msgs::msg::Marker prediction;
    prediction.ns = "selected_solver_prediction";
    prediction.id = 5;
    prediction.type = visualization_msgs::msg::Marker::LINE_STRIP;
    prediction.pose.orientation.w = 1.0;
    prediction.scale.x = 0.045;
    prediction.color.g = prediction.color.b = prediction.color.a = 1.0F;

    const bool current = prediction_received_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - prediction_received_).count() < 1.0;
    prediction.header.frame_id = frame;
    prediction.header.stamp = stamp;
    prediction.action = visualization_msgs::msg::Marker::DELETE;
    if (current && solver_prediction_.poses.size() > 1) {
      try {
        const auto transform = tf_buffer_.lookupTransform(
          frame, solver_prediction_.header.frame_id, tf2::TimePointZero);
        const auto & origin = transform.transform.translation;
        const double yaw = yawFromQuaternion(transform.transform.rotation);
        prediction.action = visualization_msgs::msg::Marker::ADD;
        for (const auto & pose : solver_prediction_.poses) {
          geometry_msgs::msg::Point point;
          point.x = origin.x + std::cos(yaw) * pose.pose.position.x -
            std::sin(yaw) * pose.pose.position.y;
          point.y = origin.y + std::sin(yaw) * pose.pose.position.x +
            std::cos(yaw) * pose.pose.position.y;
          point.z = 0.12;
          prediction.points.push_back(point);
        }
      } catch (const tf2::TransformException &) {
        // Unknown transform is represented by deletion, never a mislabeled path.
      }
    }
    array.markers.push_back(prediction);
    marker_pub_->publish(array);
    published_markers_ = true;
  }

  std::string map_frame_{"map"};
  std::string robot_frame_{"base_link"};
  double lookahead_distance_{0.6};
  double prediction_horizon_{2.0};
  double prediction_dt_{0.1};

  bool debug_enabled_{true};
  bool published_markers_{false};

  nav_msgs::msg::Path path_;
  nav_msgs::msg::Path solver_prediction_;
  std::chrono::steady_clock::time_point prediction_received_{};
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr prediction_sub_;
  std::string path_frame_;
  std::string selected_controller_{"FollowPath"};
  double linear_velocity_{0.0};
  double angular_velocity_{0.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr selector_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr debug_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nav2_controller_lab_rviz

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_controller_lab_rviz::ControllerLabVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
