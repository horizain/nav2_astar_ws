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

  double cost = evaluateTrajectory(
    trajectory, path, cost_at, params_.cost, mean_v, mean_w);

  cost += params_.terminal_weight * distance(trajectory.back(), path.back());

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

  const int steps = std::max(1, params_.horizon_steps);
  params_.dt = std::max(1e-3, params_.dt);

  if (!initialized_ || static_cast<int>(nominal_.size()) != steps) {
    const Control initial{
      std::max(params_.min_linear_velocity, 0.5 * params_.max_linear_velocity),
      0.0};
    nominal_.assign(static_cast<size_t>(steps), initial);
    initialized_ = true;
  }

  std::vector<Control> current = nominal_;
  double current_cost = trajectoryCost(robot, path, cost_at, current);

  std::vector<Control> best = current;
  double best_cost = current_cost;

  const double epsilon = std::max(1e-4, params_.gradient_epsilon);
  const int iterations = std::max(1, params_.iterations);

  for (int iter = 0; iter < iterations; ++iter) {
    std::vector<double> grad_v(static_cast<size_t>(steps), 0.0);
    std::vector<double> grad_w(static_cast<size_t>(steps), 0.0);

    for (int i = 0; i < steps; ++i) {
      for (int dim = 0; dim < 2; ++dim) {
        std::vector<Control> plus = current;
        std::vector<Control> minus = current;
        if (dim == 0) {
          plus[i].linear += epsilon;
          minus[i].linear -= epsilon;
        } else {
          plus[i].angular += epsilon;
          minus[i].angular -= epsilon;
        }
        plus[i] = clampControl(plus[i]);
        minus[i] = clampControl(minus[i]);

        const double cost_plus = trajectoryCost(robot, path, cost_at, plus);
        const double cost_minus = trajectoryCost(robot, path, cost_at, minus);
        const double gradient = (cost_plus - cost_minus) / (2.0 * epsilon);
        if (dim == 0) {
          grad_v[i] = gradient;
        } else {
          grad_w[i] = gradient;
        }
      }
    }

    double step = std::max(1e-4, params_.gradient_step);
    bool accepted = false;
    for (int attempt = 0; attempt <= std::max(0, params_.line_search_steps); ++attempt) {
      std::vector<Control> candidate = current;
      for (int i = 0; i < steps; ++i) {
        candidate[i].linear -= step * grad_v[i];
        candidate[i].angular -= step * grad_w[i];
        candidate[i] = clampControl(candidate[i]);
      }

      const double candidate_cost = trajectoryCost(robot, path, cost_at, candidate);
      if (candidate_cost < current_cost) {
        current = std::move(candidate);
        current_cost = candidate_cost;
        accepted = true;
        break;
      }
      step *= 0.5;
    }

    if (!accepted) {
      break;
    }

    if (current_cost < best_cost) {
      best_cost = current_cost;
      best = current;
    }

    if (best_cost >= params_.cost.collision_cost) {
      break;
    }
  }

  result.cost = best_cost;
  result.valid = best_cost < params_.cost.collision_cost;
  result.control = result.valid ? best.front() : Control{};

  nominal_ = best;
  if (!nominal_.empty()) {
    const Control last = nominal_.back();
    nominal_.erase(nominal_.begin());
    nominal_.push_back(last);
  }

  return result;
}

}  // namespace nav2_controller_lab
