// Copyright (c) 2026
// Pure Pursuit 控制器核心实现
//
// Licensed under the Apache License, Version 2.0

#include "nav2_controller_lab/pure_pursuit_core.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_controller_lab/path_utils.hpp"

namespace nav2_controller_lab
{

PurePursuitCore::PurePursuitCore(const PurePursuitParams & params)
: params_(params)
{
}

Control PurePursuitCore::compute(
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

  double lookahead = params_.lookahead_distance;
  if (params_.use_velocity_scaled_lookahead) {
    lookahead = std::abs(current_speed) * params_.lookahead_time;
  }
  lookahead = clamp(
    lookahead, params_.min_lookahead_distance, params_.max_lookahead_distance);

  Pose2D target;
  if (!findLookaheadPoint(robot, path, lookahead, target)) {
    return command;
  }

  const Pose2D local = toRobotFrame(robot, target);
  const double l2 = local.x * local.x + local.y * local.y;
  if (l2 < kEpsilon) {
    return command;
  }

  const double curvature = 2.0 * local.y / l2;

  double speed = params_.max_linear_velocity;
  if (params_.slowdown_radius > kEpsilon) {
    speed *= std::min(1.0, remaining / params_.slowdown_radius);
  }
  speed *= 1.0 - params_.curvature_slowdown *
    std::min(1.0, std::abs(curvature) * params_.max_linear_velocity /
    std::max(params_.max_angular_velocity, kEpsilon));
  speed = std::max(speed, params_.min_linear_velocity);

  command.linear = speed;
  command.angular = speed * curvature;

  return clampControl(command, params_.max_linear_velocity, params_.max_angular_velocity);
}

}  // namespace nav2_controller_lab
