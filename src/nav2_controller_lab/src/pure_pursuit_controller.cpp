// Copyright (c) 2026
// Pure Pursuit 控制器插件实现
//
// Licensed under the Apache License, Version 2.0

#include "nav2_controller_lab/pure_pursuit_controller.hpp"

#include <memory>

#include "pluginlib/class_list_macros.hpp"

namespace nav2_controller_lab
{

void PurePursuitController::configureParams(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
  declareAndGet(
    node, "lookahead_distance",
    params_.lookahead_distance, params_.lookahead_distance);
  declareAndGet(
    node, "min_lookahead_distance",
    params_.min_lookahead_distance, params_.min_lookahead_distance);
  declareAndGet(
    node, "max_lookahead_distance",
    params_.max_lookahead_distance, params_.max_lookahead_distance);
  declareAndGet(
    node, "use_velocity_scaled_lookahead",
    params_.use_velocity_scaled_lookahead, params_.use_velocity_scaled_lookahead);
  declareAndGet(
    node, "lookahead_time", params_.lookahead_time, params_.lookahead_time);

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
    node, "slowdown_radius", params_.slowdown_radius, params_.slowdown_radius);
  declareAndGet(
    node, "curvature_slowdown",
    params_.curvature_slowdown, params_.curvature_slowdown);

  params_.goal_tolerance = goal_tolerance_;
  core_.params() = params_;
  core_.reset();

  RCLCPP_INFO(
    logger_,
    "PurePursuitController(%s): L=%.2f [%.2f, %.2f] vmax=%.2f wmax=%.2f",
    plugin_name_.c_str(), params_.lookahead_distance,
    params_.min_lookahead_distance, params_.max_lookahead_distance,
    params_.max_linear_velocity, params_.max_angular_velocity);
}

Control PurePursuitController::computeCommand(
  const Pose2D & robot,
  const std::vector<Pose2D> & path,
  double current_speed,
  double /*dt*/)
{
  return core_.compute(robot, path, current_speed);
}

}  // namespace nav2_controller_lab

PLUGINLIB_EXPORT_CLASS(nav2_controller_lab::PurePursuitController, nav2_core::Controller)
