// Copyright (c) 2026
// Nav2 Controller 适配层公共基类实现
//
// Licensed under the Apache License, Version 2.0

#include "nav2_controller_lab/controller_base.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/controller_exceptions.hpp"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_controller_lab/path_utils.hpp"

namespace nav2_controller_lab
{

namespace
{

double quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

void ControllerBase::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_node_ = parent;
  auto node = parent_node_.lock();
  if (!node) {
    throw nav2_core::ControllerException("父节点已失效，无法 configure 控制器");
  }

  plugin_name_ = name;
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  if (!costmap_ros_) {
    throw nav2_core::ControllerException("局部代价地图为空");
  }
  costmap_ = costmap_ros_->getCostmap();
  if (!costmap_) {
    throw nav2_core::ControllerException("无法获取局部代价地图数据");
  }

  global_frame_ = costmap_ros_->getGlobalFrameID();
  base_frame_ = costmap_ros_->getBaseFrameID();

  declareAndGet(node, "transform_tolerance", 0.1, transform_tolerance_);
  declareAndGet(node, "goal_tolerance", 0.25, goal_tolerance_);

  nav2_util::declare_parameter_if_not_declared(
    node, "controller_frequency", rclcpp::ParameterValue(20.0));
  node->get_parameter("controller_frequency", control_frequency_);
  control_frequency_ = std::max(1.0, control_frequency_);

  configureParams(node);

  RCLCPP_INFO(
    logger_,
    "ControllerLab(%s) 已配置: frame=%s -> %s, freq=%.1f Hz, goal_tol=%.2f m",
    plugin_name_.c_str(), global_frame_.c_str(), base_frame_.c_str(),
    control_frequency_, goal_tolerance_);
}

void ControllerBase::cleanup()
{
  RCLCPP_INFO(logger_, "ControllerLab(%s) 清理资源", plugin_name_.c_str());
  resetController();
  global_plan_.poses.clear();
  has_plan_ = false;
  costmap_ = nullptr;
  costmap_ros_.reset();
  tf_.reset();
}

void ControllerBase::activate()
{
  RCLCPP_INFO(logger_, "ControllerLab(%s) 已激活", plugin_name_.c_str());
}

void ControllerBase::deactivate()
{
  RCLCPP_INFO(logger_, "ControllerLab(%s) 已去激活", plugin_name_.c_str());
}

void ControllerBase::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    throw nav2_core::InvalidPath("收到空路径");
  }

  global_plan_ = path;
  if (global_plan_.header.frame_id.empty()) {
    global_plan_.header.frame_id = global_frame_;
  }
  has_plan_ = true;
  resetController();

  RCLCPP_DEBUG(
    logger_, "ControllerLab(%s) 收到路径: %zu 个点, frame=%s",
    plugin_name_.c_str(), global_plan_.poses.size(),
    global_plan_.header.frame_id.c_str());
}

std::vector<Pose2D> ControllerBase::transformPlanToLocalFrame() const
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::InvalidPath("全局路径为空");
  }

  const std::string source_frame = global_plan_.header.frame_id.empty() ?
    global_frame_ : global_plan_.header.frame_id;

  std::vector<Pose2D> path;
  path.reserve(global_plan_.poses.size());

  for (const auto & input : global_plan_.poses) {
    geometry_msgs::msg::PoseStamped transformed = input;

    if (input.header.frame_id.empty()) {
      transformed.header.frame_id = source_frame;
    }

    if (transformed.header.frame_id != global_frame_) {
      if (!tf_) {
        throw nav2_core::ControllerTFError("没有 TF buffer，无法变换全局路径");
      }
      // 用当前时间变换，避免使用规划时刻的旧 TF 把路径“拉偏”
      if (clock_) {
        transformed.header.stamp = clock_->now();
      }
      try {
        geometry_msgs::msg::PoseStamped output;
        tf_->transform(
          transformed, output, global_frame_,
          tf2::durationFromSec(transform_tolerance_));
        transformed = output;
      } catch (const tf2::TransformException & ex) {
        throw nav2_core::ControllerTFError(
                std::string("路径变换到局部坐标系失败: ") + ex.what());
      }
    }

    Pose2D pose;
    pose.x = transformed.pose.position.x;
    pose.y = transformed.pose.position.y;
    pose.yaw = quaternionToYaw(transformed.pose.orientation);
    path.push_back(pose);
  }

  // 统一用相邻点切线，避免上游朝向缺失或不一致
  recomputePathYaw(path);
  return path;
}

geometry_msgs::msg::TwistStamped ControllerBase::zeroTwist() const
{
  geometry_msgs::msg::TwistStamped command;
  command.header.frame_id = base_frame_;
  if (clock_) {
    command.header.stamp = clock_->now();
  }
  return command;
}

geometry_msgs::msg::TwistStamped ControllerBase::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  if (!has_plan_ || global_plan_.poses.empty()) {
    throw nav2_core::InvalidPath("还没有收到全局路径");
  }
  if (!costmap_) {
    throw nav2_core::ControllerException("控制器尚未 configure 或代价地图已释放");
  }

  const std::vector<Pose2D> path = transformPlanToLocalFrame();

  Pose2D robot;
  robot.x = pose.pose.position.x;
  robot.y = pose.pose.position.y;
  robot.yaw = quaternionToYaw(pose.pose.orientation);

  const double dt = 1.0 / std::max(control_frequency_, 1.0);

  Control command;
  if (needsCostmap()) {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *(costmap_->getMutex()));
    command = computeCommand(robot, path, velocity.linear.x, dt);
  } else {
    command = computeCommand(robot, path, velocity.linear.x, dt);
  }

  if (!std::isfinite(command.linear) || !std::isfinite(command.angular)) {
    throw nav2_core::NoValidControl("控制器输出了非有限值");
  }

  if (speed_limit_active_) {
    double limit = speed_limit_;
    if (speed_limit_is_percentage_) {
      const double ratio = speed_limit_ > 1.0 ? speed_limit_ / 100.0 : speed_limit_;
      limit = maxLinearVelocity() * std::max(0.0, ratio);
    }
    command.linear = clamp(command.linear, -std::abs(limit), std::abs(limit));
  }

  geometry_msgs::msg::TwistStamped command_msg;
  command_msg.header.frame_id = base_frame_;
  if (clock_) {
    command_msg.header.stamp = clock_->now();
  }
  command_msg.twist.linear.x = command.linear;
  command_msg.twist.angular.z = command.angular;

  RCLCPP_DEBUG(
    logger_, "ControllerLab(%s): v=%.3f m/s, w=%.3f rad/s, path=%zu",
    plugin_name_.c_str(), command.linear, command.angular, path.size());

  return command_msg;
}

void ControllerBase::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
  speed_limit_active_ = true;

  RCLCPP_INFO(
    logger_, "ControllerLab(%s) 速度限制: %.3f (%s)",
    plugin_name_.c_str(), speed_limit, percentage ? "百分比" : "绝对 m/s");
}

}  // namespace nav2_controller_lab
