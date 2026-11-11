// Copyright (c) 2026
// MPPI-lite 实现
//
// Licensed under the Apache License, Version 2.0

#include "nav2_controller_lab/mppi_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_controller_lab/path_utils.hpp"
#include "nav2_controller_lab/trajectory_utils.hpp"

namespace nav2_controller_lab
{

MppiCore::MppiCore(const MppiParams & params)
: params_(params), rng_(params.seed)
{
  params_.cost.desired_linear_velocity = params_.max_linear_velocity;
}

void MppiCore::reset()
{
  nominal_.clear();
  initialized_ = false;
  rng_.seed(params_.seed);
}

ControlResult MppiCore::compute(
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

  const int steps = std::max(1, params_.time_steps);
  const int batch = std::max(1, params_.batch_size);
  const double dt = std::max(1e-3, params_.dt);
  const double temperature = std::max(1e-3, params_.temperature);

  if (!initialized_ || static_cast<int>(nominal_.size()) != steps) {
    const Control initial{
      std::max(params_.min_linear_velocity, 0.5 * params_.max_linear_velocity), 0.0};
    nominal_.assign(static_cast<size_t>(steps), initial);
    initialized_ = true;
  }

  std::normal_distribution<double> linear_noise(0.0, std::max(0.0, params_.vx_std));
  std::normal_distribution<double> angular_noise(0.0, std::max(0.0, params_.wz_std));

  std::vector<std::vector<Control>> samples;
  std::vector<double> costs;
  samples.reserve(static_cast<size_t>(batch));
  costs.reserve(static_cast<size_t>(batch));

  double best_cost = std::numeric_limits<double>::infinity();
  double min_cost = std::numeric_limits<double>::infinity();

  for (int k = 0; k < batch; ++k) {
    std::vector<Control> sample = nominal_;
    for (auto & control : sample) {
      control.linear += linear_noise(rng_);
      control.angular += angular_noise(rng_);
      control.linear = clamp(
        control.linear, params_.min_linear_velocity, params_.max_linear_velocity);
      control.angular = clamp(
        control.angular, -params_.max_angular_velocity, params_.max_angular_velocity);
    }

    const auto trajectory = rolloutControlSequence(robot, sample, dt);
    double mean_v = 0.0;
    double mean_w = 0.0;
    for (const auto & control : sample) {
      mean_v += control.linear;
      mean_w += control.angular;
    }
    mean_v /= static_cast<double>(sample.size());
    mean_w /= static_cast<double>(sample.size());

    const double cost = evaluateTrajectory(
      trajectory, path, cost_at, params_.cost, mean_v, mean_w);
    costs.push_back(cost);
    samples.push_back(std::move(sample));

    best_cost = std::min(best_cost, cost);
    min_cost = std::min(min_cost, cost);
  }

  if (!std::isfinite(min_cost) || min_cost >= params_.cost.collision_cost) {
    result.valid = false;
    result.cost = min_cost;
    return result;
  }

  double weight_sum = 0.0;
  std::vector<double> weights(static_cast<size_t>(batch), 0.0);
  for (int k = 0; k < batch; ++k) {
    const double exponent = std::max(-50.0, -(costs[k] - min_cost) / temperature);
    weights[k] = std::exp(exponent);
    weight_sum += weights[k];
  }
  if (weight_sum <= kEpsilon) {
    result.valid = false;
    return result;
  }

  std::vector<Control> updated(static_cast<size_t>(steps), Control{});
  for (int k = 0; k < batch; ++k) {
    const double w = weights[k] / weight_sum;
    for (int i = 0; i < steps; ++i) {
      updated[i].linear += w * samples[k][i].linear;
      updated[i].angular += w * samples[k][i].angular;
    }
  }

  for (auto & control : updated) {
    control.linear = clamp(
      control.linear, params_.min_linear_velocity, params_.max_linear_velocity);
    control.angular = clamp(
      control.angular, -params_.max_angular_velocity, params_.max_angular_velocity);
  }

  nominal_ = updated;
  result.control = nominal_.front();
  result.cost = best_cost;
  result.valid = true;

  // 滚动时域：丢掉第一步，末尾复制最后一个控制量
  nominal_.erase(nominal_.begin());
  nominal_.push_back(updated.back());

  return result;
}

}  // namespace nav2_controller_lab
