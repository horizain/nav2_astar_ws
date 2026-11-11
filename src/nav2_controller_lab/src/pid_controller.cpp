// Copyright (c) 2026
// PID 路径跟踪控制器插件实现
//
// Licensed under the Apache License, Version 2.0

#include "nav2_controller_lab/pid_controller.hpp"

#include <memory>

#include "pluginlib/class_list_macros.hpp"

namespace nav2_controller_lab
{

void PidController::configureParams(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
  declareAndGet(node, "kp_heading", params_.kp_heading, params_.kp_heading);
  declareAndGet(node, "ki_heading", params_.ki_heading, params_.ki_heading);
  declareAndGet(node, "kd_heading", params_.kd_heading, params_.kd_heading);
  declareAndGet(node, "kp_cross", params_.kp_cross, params_.kp_cross);

  declareAndGet(node, "kp_speed", params_.kp_speed, params_.kp_speed);
  declareAndGet(node, "ki_speed", params_.ki_speed, params_.ki_speed);
  declareAndGet(node, "kd_speed", params_.kd_speed, params_.kd_speed);

  declareAndGet(
    node, "max_linear_velocity",
    params_.max_linear_velocity, params_.max_linear_velocity);
  declareAndGet(
    node, "max_angular_velocity",
    params_.max_angular_velocity, params_.max_angular_velocity);
  declareAndGet(node, "slow_radius", params_.slow_radius, params_.slow_radius);
  declareAndGet(
    node, "heading_slowdown", params_.heading_slowdown, params_.heading_slowdown);
  declareAndGet(
    node, "integral_limit", params_.integral_limit, params_.integral_limit);

  params_.goal_tolerance = goal_tolerance_;
  core_.params() = params_;
  core_.reset();

  RCLCPP_INFO(
    logger_,
    "PidController(%s): kp_heading=%.2f ki_heading=%.2f kd_heading=%.2f "
    "kp_cross=%.2f vmax=%.2f wmax=%.2f",
    plugin_name_.c_str(), params_.kp_heading, params_.ki_heading,
    params_.kd_heading, params_.kp_cross, params_.max_linear_velocity,
    params_.max_angular_velocity);
}

Control PidController::computeCommand(
  const Pose2D & robot,
  const std::vector<Pose2D> & path,
  double current_speed,
  double dt)
{
  return core_.compute(robot, path, current_speed, dt);
}

}  // namespace nav2_controller_lab

PLUGINLIB_EXPORT_CLASS(nav2_controller_lab::PidController, nav2_core::Controller)
