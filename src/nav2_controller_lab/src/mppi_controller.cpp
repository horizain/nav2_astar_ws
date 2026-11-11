// Copyright (c) 2026
// MPPI-lite 控制器插件实现
//
// Licensed under the Apache License, Version 2.0

#include "nav2_controller_lab/mppi_controller.hpp"

#include <algorithm>
#include <memory>

#include "pluginlib/class_list_macros.hpp"

#include "nav2_core/controller_exceptions.hpp"

namespace nav2_controller_lab
{

void MppiController::configureParams(
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

  declareAndGet(node, "time_steps", params_.time_steps, params_.time_steps);
  declareAndGet(node, "batch_size", params_.batch_size, params_.batch_size);
  declareAndGet(node, "model_dt", params_.dt, params_.dt);
  declareAndGet(node, "vx_std", params_.vx_std, params_.vx_std);
  declareAndGet(node, "wz_std", params_.wz_std, params_.wz_std);
  declareAndGet(node, "temperature", params_.temperature, params_.temperature);

  int seed = static_cast<int>(params_.seed);
  declareAndGet(node, "seed", seed, seed);
  params_.seed = static_cast<unsigned int>(std::max(0, seed));

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
  params_.time_steps = std::max(1, params_.time_steps);
  params_.batch_size = std::max(1, params_.batch_size);
  params_.temperature = std::max(1e-3, params_.temperature);

  core_.params() = params_;
  core_.reset();

  RCLCPP_INFO(
    logger_,
    "MppiController(%s): K=%d steps=%d dt=%.2f temp=%.2f vmax=%.2f wmax=%.2f",
    plugin_name_.c_str(), params_.batch_size, params_.time_steps,
    params_.dt, params_.temperature, params_.max_linear_velocity,
    params_.max_angular_velocity);
}

Control MppiController::computeCommand(
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
    throw nav2_core::NoValidControl("MPPI-lite 没有找到可行轨迹");
  }
  return result.control;
}

}  // namespace nav2_controller_lab

PLUGINLIB_EXPORT_CLASS(nav2_controller_lab::MppiController, nav2_core::Controller)
