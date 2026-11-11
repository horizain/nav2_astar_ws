// Copyright (c) 2026
// PID 路径跟踪基线实现
//
// Licensed under the Apache License, Version 2.0

#include "nav2_controller_lab/pid_core.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_controller_lab/path_utils.hpp"

namespace nav2_controller_lab
{

PidCore::PidCore(const PidParams & params)
: params_(params)
{
}

void PidCore::reset()
{
  heading_integral_ = 0.0;
  previous_heading_error_ = 0.0;
  heading_initialized_ = false;
  speed_integral_ = 0.0;
  previous_speed_error_ = 0.0;
  speed_initialized_ = false;
}

Control PidCore::compute(
  const Pose2D & robot,
  const std::vector<Pose2D> & path,
  double current_speed,
  double dt)
{
  Control command;
  if (path.empty()) {
    reset();
    return command;
  }

  const double remaining = remainingDistance(robot, path);
  if (remaining <= params_.goal_tolerance) {
    reset();
    return command;
  }

  const double safe_dt = std::max(dt, 1e-3);
  const PathProjection projection = projectOntoPath(robot, path);

  const double heading_error = normalizeAngle(projection.path_yaw - robot.yaw);
  heading_integral_ = clamp(
    heading_integral_ + heading_error * safe_dt,
    -params_.integral_limit, params_.integral_limit);

  double heading_derivative = 0.0;
  if (heading_initialized_) {
    heading_derivative = (heading_error - previous_heading_error_) / safe_dt;
  }
  previous_heading_error_ = heading_error;
  heading_initialized_ = true;

  command.angular =
    params_.kp_heading * heading_error +
    params_.ki_heading * heading_integral_ +
    params_.kd_heading * heading_derivative -
    params_.kp_cross * projection.signed_lateral;
  command.angular = clamp(
    command.angular, -params_.max_angular_velocity, params_.max_angular_velocity);

  double slowdown = 1.0;
  if (params_.slow_radius > kEpsilon) {
    slowdown = std::min(1.0, remaining / params_.slow_radius);
  }
  const double heading_scale = 1.0 - 0.7 * std::min(
    1.0, std::abs(heading_error) / std::max(params_.heading_slowdown, kEpsilon));
  const double desired_speed =
    params_.max_linear_velocity * std::max(0.1, slowdown * heading_scale);

  const double speed_error = desired_speed - current_speed;
  speed_integral_ = clamp(
    speed_integral_ + speed_error * safe_dt,
    -params_.integral_limit, params_.integral_limit);

  double speed_derivative = 0.0;
  if (speed_initialized_) {
    speed_derivative = (speed_error - previous_speed_error_) / safe_dt;
  }
  previous_speed_error_ = speed_error;
  speed_initialized_ = true;

  command.linear =
    desired_speed +
    params_.kp_speed * speed_error +
    params_.ki_speed * speed_integral_ +
    params_.kd_speed * speed_derivative;
  command.linear = clamp(command.linear, 0.0, params_.max_linear_velocity);

  return command;
}

}  // namespace nav2_controller_lab
