// Copyright (c) 2026
// DWB-lite 控制器插件实现
//
// Licensed under the Apache License, Version 2.0

#include "nav2_controller_lab/dwb_controller.hpp"

#include <algorithm>
#include <memory>

#include "pluginlib/class_list_macros.hpp"

#include "nav2_core/controller_exceptions.hpp"

namespace nav2_controller_lab
{

void DwbController::configureParams(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
  declareAndGet(
    node, "min_linear_velocity",
    params_.min_linear_velocity, params_.min_linear_velocity);
  declareAndGet(
    node, "max_linear_velocity",
    params_.max_linear_velocity, params_.max_linear_velocity);
  declareAndGet(
    node, "max_angular_velocity",
    params_.max_angular_velocity, params_.max_angular_velocity);

  declareAndGet(node, "linear_samples", params_.linear_samples, params_.linear_samples);
  declareAndGet(node, "angular_samples", params_.angular_samples, params_.angular_samples);
  declareAndGet(node, "horizon", params_.horizon, params_.horizon);
  declareAndGet(node, "model_dt", params_.dt, params_.dt);

  declareAndGet(
    node, "path_weight", params_.cost.path_weight, params_.cost.path_weight);
  declareAndGet(
    node, "goal_weight", params_.cost.goal_weight, params_.cost.goal_weight);
  declareAndGet(
    node, "obstacle_weight",
    params_.cost.obstacle_weight, params_.cost.obstacle_weight);
  declareAndGet(
    node, "heading_weight", params_.cost.heading_weight, params_.cost.heading_weight);
  declareAndGet(
    node, "velocity_weight",
    params_.cost.velocity_weight, params_.cost.velocity_weight);
  declareAndGet(
    node, "control_weight", params_.cost.control_weight, params_.cost.control_weight);
  declareAndGet(
    node, "collision_cost", params_.cost.collision_cost, params_.cost.collision_cost);
  declareAndGet(
    node, "collision_threshold",
    params_.cost.collision_threshold, params_.cost.collision_threshold);
  declareAndGet(
    node, "desired_linear_velocity",
    params_.max_linear_velocity, params_.cost.desired_linear_velocity);

  params_.goal_tolerance = goal_tolerance_;
  params_.dt = std::max(1e-3, params_.dt);
  core_.params() = params_;
  core_.reset();

  RCLCPP_INFO(
    logger_,
    "DwbController(%s): v=[%.2f, %.2f] wmax=%.2f samples=%dx%d horizon=%.2f",
    plugin_name_.c_str(), params_.min_linear_velocity,
    params_.max_linear_velocity, params_.max_angular_velocity,
    params_.linear_samples, params_.angular_samples, params_.horizon);
}

Control DwbController::computeCommand(
  const Pose2D & robot,
  const std::vector<Pose2D> & path,
  double current_speed,
  double /*dt*/)
{
  auto cost_at = [this](double wx, double wy) -> uint8_t {
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap_->worldToMap(wx, wy, mx, my)) {
        return 255;
      }
      return costmap_->getCost(mx, my);
    };

  const auto result = core_.compute(robot, path, cost_at, current_speed);
  if (!result.valid) {
    throw nav2_core::NoValidControl("DWB-lite 没有找到可行轨迹");
  }
  return result.control;
}

}  // namespace nav2_controller_lab

PLUGINLIB_EXPORT_CLASS(nav2_controller_lab::DwbController, nav2_core::Controller)
