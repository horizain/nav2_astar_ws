// Copyright (c) 2026
// DWB-lite 实现
//
// Licensed under the Apache License, Version 2.0

#include "nav2_controller_lab/dwb_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_controller_lab/path_utils.hpp"
#include "nav2_controller_lab/trajectory_utils.hpp"

namespace nav2_controller_lab
{

DwbCore::DwbCore(const DwbParams & params)
: params_(params)
{
  params_.cost.desired_linear_velocity = params_.max_linear_velocity;
}

ControlResult DwbCore::compute(
  const Pose2D & robot,
  const std::vector<Pose2D> & path,
  const std::function<uint8_t(double, double)> & cost_at,
  double /*current_speed*/)
{
  ControlResult result;
  if (path.empty()) {
    result.valid = false;
    return result;
  }

  if (remainingDistance(robot, path) <= params_.goal_tolerance) {
    return result;
  }

  const int v_samples = std::max(2, params_.linear_samples);
  const int w_samples = std::max(3, params_.angular_samples);
  const int steps = std::max(1, static_cast<int>(std::round(params_.horizon / params_.dt)));

  double best_cost = std::numeric_limits<double>::infinity();
  Control best{};
  bool found = false;

  for (int vi = 0; vi < v_samples; ++vi) {
    const double v = params_.min_linear_velocity +
      (params_.max_linear_velocity - params_.min_linear_velocity) * vi /
      static_cast<double>(v_samples - 1);

    for (int wi = 0; wi < w_samples; ++wi) {
      const double w = -params_.max_angular_velocity +
        2.0 * params_.max_angular_velocity * wi /
        static_cast<double>(w_samples - 1);

      const auto trajectory = rolloutConstantControl(robot, v, w, steps, params_.dt);
      const double cost = evaluateTrajectory(
        trajectory, path, cost_at, params_.cost, v, w);

      if (cost < best_cost) {
        best_cost = cost;
        best = Control{v, w};
        found = true;
      }
    }
  }

  result.control = best;
  result.cost = best_cost;
  result.valid = found && best_cost < params_.cost.collision_cost;
  if (!result.valid) {
    result.control = Control{};
  }
  return result;
}

}  // namespace nav2_controller_lab
