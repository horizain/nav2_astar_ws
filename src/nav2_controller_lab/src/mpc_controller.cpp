// Copyright (c) 2026
// MPC-lite 控制器插件实现
//
// Licensed under the Apache License, Version 2.0

#include "nav2_controller_lab/mpc_controller.hpp"

#include <algorithm>
#include <memory>

#include "nav2_core/controller_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_controller_lab
{

void MpcController::configureParams(
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

  declareAndGet(node, "horizon_steps", params_.horizon_steps, params_.horizon_steps);
  declareAndGet(node, "iterations", params_.iterations, params_.iterations);
  declareAndGet(
    node, "line_search_steps",
    params_.line_search_steps, params_.line_search_steps);
  declareAndGet(node, "model_dt", params_.dt, params_.dt);
  declareAndGet(
    node, "gradient_step", params_.gradient_step, params_.gradient_step);
  declareAndGet(
    node, "gradient_epsilon",
    params_.gradient_epsilon, params_.gradient_epsilon);
  declareAndGet(
    node, "terminal_weight",
    params_.terminal_weight, params_.terminal_weight);
  declareAndGet(
    node, "smoothness_weight",
    params_.smoothness_weight, params_.smoothness_weight);

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

  declareAndGet(node, "max_linear_acceleration", 1.0, params_.max_linear_acceleration);
  declareAndGet(node, "max_angular_acceleration", 3.2, params_.max_angular_acceleration);
  prediction_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "/controller_lab/predicted_trajectory", 10);
  params_.goal_tolerance = goal_tolerance_;
  params_.dt = std::max(1e-3, params_.dt);
  params_.horizon_steps = std::max(1, params_.horizon_steps);
  params_.iterations = std::max(1, params_.iterations);

  core_.params() = params_;
  core_.reset();

  RCLCPP_INFO(
    logger_,
    "MpcController(%s): steps=%d iter=%d dt=%.2f vmax=%.2f wmax=%.2f",
    plugin_name_.c_str(), params_.horizon_steps, params_.iterations,
    params_.dt, params_.max_linear_velocity, params_.max_angular_velocity);
}

void MpcController::activate()
{
  ControllerBase::activate();
  if (prediction_pub_) {prediction_pub_->on_activate();}
}

void MpcController::deactivate()
{
  core_.reset();
  publishPrediction();
  if (prediction_pub_) {prediction_pub_->on_deactivate();}
  ControllerBase::deactivate();
}

void MpcController::publishPrediction()
{
  if (!prediction_pub_ || !prediction_pub_->is_activated()) {return;}
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_->now();
  for (const auto & predicted : core_.predictedTrajectory()) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = predicted.x;
    pose.pose.position.y = predicted.y;
    pose.pose.orientation.z = std::sin(predicted.yaw / 2);
    pose.pose.orientation.w = std::cos(predicted.yaw / 2);
    path.poses.push_back(pose);
  }
  prediction_pub_->publish(path);
}

Control MpcController::computeCommand(
  const Pose2D & robot,
  const std::vector<Pose2D> & path,
  double current_speed,
  double dt)
{
  auto cost_at = [this](double wx, double wy) -> uint8_t {
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap_->worldToMap(wx, wy, mx, my)) {
        return 255;
      }
      return costmap_->getCost(mx, my);
    };

  const auto result = core_.compute(robot, path, cost_at, current_speed, dt, current_angular_);
  publishPrediction();
  if (!result.valid) {
    throw nav2_core::NoValidControl("MPC-lite 没有找到可行控制序列");
  }
  return result.control;
}

}  // namespace nav2_controller_lab

PLUGINLIB_EXPORT_CLASS(nav2_controller_lab::MpcController, nav2_core::Controller)
