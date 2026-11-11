// Copyright (c) 2026
// Stanley 控制器核心实现
//
// Licensed under the Apache License, Version 2.0

#include "nav2_controller_lab/stanley_core.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_controller_lab/path_utils.hpp"

namespace nav2_controller_lab
{

StanleyCore::StanleyCore(const StanleyParams & params)
: params_(params)
{
}

Control StanleyCore::compute(
  const Pose2D & robot,
  const std::vector<Pose2D> & path,
  double current_speed)
{
  Control command;
  if (path.empty()) {
    return command;
  }

  const double remaining = remainingDistance(robot, path);
  if (remaining <= params_.goal_tolerance) {
    return command;
  }

  Pose2D front = robot;
  front.x += params_.front_axle_offset * std::cos(robot.yaw);
  front.y += params_.front_axle_offset * std::sin(robot.yaw);

  const PathProjection projection = projectOntoPath(front, path);
  const double heading_error = normalizeAngle(projection.path_yaw - robot.yaw);

  double speed = params_.max_linear_velocity;
  if (params_.slowdown_radius > kEpsilon) {
    speed *= std::min(1.0, remaining / params_.slowdown_radius);
  }
  speed *= 1.0 - 0.6 * std::min(
    1.0, std::abs(heading_error) / std::max(params_.heading_slowdown, kEpsilon));
  speed = clamp(speed, params_.min_linear_velocity, params_.max_linear_velocity);

  const double cross_term = std::atan2(
    params_.k_cross * projection.signed_lateral, std::max(speed, 0.1));
  double steering = params_.k_heading * heading_error - cross_term;
  steering = clamp(
    steering, -params_.max_steering_angle, params_.max_steering_angle);

  command.linear = speed;
  if (params_.wheelbase > kEpsilon) {
    command.angular = speed * std::tan(steering) / params_.wheelbase;
  } else {
    // 退化形式：直接把 steering 当角速度
    command.angular = params_.k_heading * heading_error -
      params_.k_cross * projection.signed_lateral / std::max(speed, 0.1);
  }

  return clampControl(command, params_.max_linear_velocity, params_.max_angular_velocity);
}

}  // namespace nav2_controller_lab
