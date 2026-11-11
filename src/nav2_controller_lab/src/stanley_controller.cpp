// Copyright (c) 2026
// Stanley 控制器插件实现
//
// Licensed under the Apache License, Version 2.0

#include "nav2_controller_lab/stanley_controller.hpp"

#include <memory>

#include "pluginlib/class_list_macros.hpp"

namespace nav2_controller_lab
{

void StanleyController::configureParams(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
  declareAndGet(node, "k_cross", params_.k_cross, params_.k_cross);
  declareAndGet(node, "k_heading", params_.k_heading, params_.k_heading);
  declareAndGet(node, "wheelbase", params_.wheelbase, params_.wheelbase);
  declareAndGet(
    node, "front_axle_offset",
    params_.front_axle_offset, params_.front_axle_offset);

  declareAndGet(
    node, "max_linear_velocity",
    params_.max_linear_velocity, params_.max_linear_velocity);
  declareAndGet(
    node, "min_linear_velocity",
    params_.min_linear_velocity, params_.min_linear_velocity);
  declareAndGet(
    node, "max_angular_velocity",
    params_.max_angular_velocity, params_.max_angular_velocity);
  declareAndGet(
    node, "max_steering_angle",
    params_.max_steering_angle, params_.max_steering_angle);
  declareAndGet(
    node, "slowdown_radius", params_.slowdown_radius, params_.slowdown_radius);
  declareAndGet(
    node, "heading_slowdown", params_.heading_slowdown, params_.heading_slowdown);

  params_.goal_tolerance = goal_tolerance_;
  core_.params() = params_;
  core_.reset();

  RCLCPP_INFO(
    logger_,
    "StanleyController(%s): k_cross=%.2f k_heading=%.2f wheelbase=%.2f "
    "vmax=%.2f wmax=%.2f",
    plugin_name_.c_str(), params_.k_cross, params_.k_heading,
    params_.wheelbase, params_.max_linear_velocity, params_.max_angular_velocity);
}

Control StanleyController::computeCommand(
  const Pose2D & robot,
  const std::vector<Pose2D> & path,
  double current_speed,
  double /*dt*/)
{
  return core_.compute(robot, path, current_speed);
}

}  // namespace nav2_controller_lab

PLUGINLIB_EXPORT_CLASS(nav2_controller_lab::StanleyController, nav2_core::Controller)
