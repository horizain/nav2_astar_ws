// Copyright (c) 2026
// MPC-lite 实现
//
// Licensed under the Apache License, Version 2.0

#include "nav2_controller_lab/mpc_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "nav2_controller_lab/path_utils.hpp"
#include "nav2_controller_lab/trajectory_utils.hpp"

namespace nav2_controller_lab
{

MpcCore::MpcCore(const MpcParams & params)
: params_(params)
{
  params_.cost.desired_linear_velocity = params_.max_linear_velocity;
}

void MpcCore::reset()
{
  nominal_.clear();
  prediction_.clear();
  initialized_ = false;
}

Control MpcCore::clampControl(const Control & control) const
{
  Control out;
  out.linear = clamp(
    control.linear, params_.min_linear_velocity, params_.max_linear_velocity);
  out.angular = clamp(
    control.angular, -params_.max_angular_velocity, params_.max_angular_velocity);
  return out;
}

double MpcCore::trajectoryCost(
  const Pose2D & robot,
  const std::vector<Pose2D> & path,
  const std::function<uint8_t(double, double)> & cost_at,
  const std::vector<Control> & controls) const
{
  if (controls.empty() || path.empty()) {
    return params_.cost.collision_cost;
  }

  const auto trajectory = rolloutControlSequence(robot, controls, params_.dt);

  double mean_v = 0.0;
  double mean_w = 0.0;
  for (const auto & control : controls) {
    mean_v += control.linear;
    mean_w += control.angular;
  }
  mean_v /= static_cast<double>(controls.size());
  mean_w /= static_cast<double>(controls.size());

  auto weights = params_.cost;
  weights.goal_weight = 0.0;  // Follow the local route, including detours away from the global goal.
  double cost = evaluateTrajectory(
    trajectory, path, cost_at, weights, mean_v, mean_w);
  if (cost >= params_.cost.collision_cost) {return cost;}
  Pose2D reference;
  double tracking = 0.0;
  const double reference_speed = std::max(0.1, params_.cost.desired_linear_velocity);
  for (size_t i = 1; i < trajectory.size(); ++i) {
    findLookaheadPoint(robot, path, reference_speed * params_.dt * i, reference);
    tracking += squaredDistance(trajectory[i], reference);
  }
  cost += (params_.cost.path_weight + params_.cost.goal_weight) *
    tracking / static_cast<double>(controls.size());
  cost += params_.terminal_weight * distance(trajectory.back(), reference);

  for (size_t i = 1; i < controls.size(); ++i) {
    const double dv = controls[i].linear - controls[i - 1].linear;
    const double dw = controls[i].angular - controls[i - 1].angular;
    cost += params_.smoothness_weight * (dv * dv + dw * dw);
  }

  return cost;
}

ControlResult MpcCore::compute(
  const Pose2D & robot,
  const std::vector<Pose2D> & path,
  const std::function<uint8_t(double, double)> & cost_at,
  double current_speed, double control_dt, double current_angular)
{
  ControlResult result;
  if (path.empty() || !std::isfinite(current_speed) || !std::isfinite(current_angular)) {
    result.valid = false;
    reset();
    return result;
  }
  if (remainingDistance(robot, path) <= params_.goal_tolerance) {
    reset();
    const double dv = params_.max_linear_acceleration * std::max(1e-3, control_dt);
    const double dw = params_.max_angular_acceleration * std::max(1e-3, control_dt);
    result.control = {clamp(0.0, current_speed - dv, current_speed + dv),
      clamp(0.0, current_angular - dw, current_angular + dw)};
    return result;
  }
  const int steps = std::max(1, params_.horizon_steps);
  params_.dt = std::max(1e-3, params_.dt);
  control_dt = std::max(1e-3, control_dt);
  // Project the entire candidate, including its first command, from measured velocity.
  auto project = [&](std::vector<Control> & sequence) {
      Control previous{current_speed, current_angular};
      for (size_t i = 0; i < sequence.size(); ++i) {
        const double dt = i == 0 ? control_dt : params_.dt;
        auto c = clampControl(sequence[i]);
        const double dv = std::max(1e-3, params_.max_linear_acceleration) * dt;
        const double dw = std::max(1e-3, params_.max_angular_acceleration) * dt;
        c.linear = clamp(c.linear, previous.linear - dv, previous.linear + dv);
        c.angular = clamp(c.angular, previous.angular - dw, previous.angular + dw);
        sequence[i] = c;
        previous = c;
      }
    };
  Pose2D target;
  findLookaheadPoint(robot, path, 0.5, target);
  const auto local_target = toRobotFrame(robot, target);
  const double steering_error = std::atan2(local_target.y, local_target.x);
  // A differential drive cannot track a sideways reference before aligning.
  // Use a checked braking/rotation maneuver rather than a stationary local optimum.
  if (std::abs(steering_error) > 0.65) {
    std::vector<Control> align(steps, Control{0.0,
      clamp(2.0 * steering_error, -params_.max_angular_velocity, params_.max_angular_velocity)});
    project(align);
    const double cost = trajectoryCost(robot, path, cost_at, align);
    if (std::isfinite(cost) && cost < params_.cost.collision_cost) {
      reset();
      prediction_ = rolloutControlSequence(robot, align, params_.dt);
      result.control = align.front();
      result.cost = cost;
      return result;
    }
  }
  std::vector<Control> best;
  double best_cost = params_.cost.collision_cost;
  auto consider = [&](std::vector<Control> candidate) {
      project(candidate);
      double cost = trajectoryCost(robot, path, cost_at, candidate);
      // Couple the first command to measured motion, not just adjacent prediction steps.
      cost += params_.smoothness_weight * (
        std::pow(candidate.front().linear - current_speed, 2) +
        std::pow(candidate.front().angular - current_angular, 2));
      cost += params_.cost.heading_weight * std::abs(
        normalizeAngle(steering_error - candidate.front().angular * 0.3));
      if (std::isfinite(cost) && cost < best_cost) {
        best_cost = cost;
        best = std::move(candidate);
        return true;
      }
      return false;
    };
  if (initialized_ && static_cast<int>(nominal_.size()) == steps) {
    consider(nominal_);
  }
  // Feasible braking and path-guided seeds escape a colliding warm start.
  consider(std::vector<Control>(steps, Control{}));
  for (double speed_ratio : {0.3, 0.6, 1.0}) {
    for (double lookahead : {0.35, 0.7}) {
      Pose2D pose = robot;
      std::vector<Control> seed;
      for (int i = 0; i < steps; ++i) {
        Pose2D target;
        findLookaheadPoint(pose, path, lookahead, target);
        const auto local = toRobotFrame(pose, target);
        const double angle = std::atan2(local.y, local.x);
        Control c{params_.max_linear_velocity * speed_ratio *
          std::max(0.0, std::cos(angle)), 2.0 * angle};
        c = clampControl(c);
        seed.push_back(c);
        pose = rolloutConstantControl(pose, c.linear, c.angular, 1, params_.dt).back();
      }
      consider(seed);
    }
  }
  // Include both turning directions even when the nearest path seed is blocked.
  for (double turn : {-1.0, -0.5, 0.5, 1.0}) {
    for (double speed : {0.0, 0.15, 0.3}) {
      consider(std::vector<Control>(steps,
        Control{speed, turn * params_.max_angular_velocity}));
    }
  }
  if (best.empty()) {
    reset();
    result.valid = false;
    return result;
  }
  // Grid costs are discontinuous: pattern search does not rely on a false zero gradient.
  double dv = 0.2 * params_.max_linear_velocity;
  double dw = 0.25 * params_.max_angular_velocity;
  for (int iteration = 0; iteration < std::max(1, params_.iterations); ++iteration) {
    bool improved = false;
    for (int start = 0; start < steps; start += 3) {
      for (int dimension = 0; dimension < 2; ++dimension) {
        for (double sign : {-1.0, 1.0}) {
          auto candidate = best;
          for (int i = start; i < std::min(start + 3, steps); ++i) {
            if (dimension == 0) {candidate[i].linear += sign * dv;}
            else {candidate[i].angular += sign * dw;}
          }
          improved = consider(std::move(candidate)) || improved;
        }
      }
    }
    if (!improved) {dv *= 0.5; dw *= 0.5;}
  }
  result.valid = true;
  result.cost = best_cost;
  result.control = best.front();
  prediction_ = rolloutControlSequence(robot, best, params_.dt);
  nominal_ = best;
  nominal_.erase(nominal_.begin());
  nominal_.push_back(best.back());
  initialized_ = true;
  return result;
}

}  // namespace nav2_controller_lab
