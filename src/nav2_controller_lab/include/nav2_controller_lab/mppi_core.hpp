// Copyright (c) 2026
// MPPI-lite：采样控制序列 + 软最大加权 —— 纯 C++
//
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_CONTROLLER_LAB__MPPI_CORE_HPP_
#define NAV2_CONTROLLER_LAB__MPPI_CORE_HPP_

#include <cstdint>
#include <functional>
#include <random>
#include <vector>

#include "nav2_controller_lab/controller_types.hpp"
#include "nav2_controller_lab/trajectory_cost.hpp"

namespace nav2_controller_lab
{

struct MppiParams
{
  double min_linear_velocity{0.0};
  double max_linear_velocity{0.5};
  double max_angular_velocity{1.9};

  int time_steps{20};
  double dt{0.1};
  int batch_size{100};
  double vx_std{0.15};
  double wz_std{0.35};
  double temperature{0.3};

  double goal_tolerance{0.25};
  unsigned int seed{42};

  TrajectoryCostParams cost;
};

/**
 * @brief 简化版 MPPI
 *
 * 维护一条标称控制序列 U。每个周期在 U 附近采样 K 条带噪声的序列，
 * rollout 后按 exp(-cost/temperature) 加权平均，滚动时域执行第一步。
 */
class MppiCore
{
public:
  explicit MppiCore(const MppiParams & params = MppiParams());

  ControlResult compute(
    const Pose2D & robot,
    const std::vector<Pose2D> & path,
    const std::function<uint8_t(double, double)> & cost_at,
    double current_speed);

  void reset();

  const MppiParams & params() const {return params_;}
  MppiParams & params() {return params_;}

private:
  MppiParams params_;
  std::vector<Control> nominal_;
  std::mt19937 rng_;
  bool initialized_{false};
};

}  // namespace nav2_controller_lab

#endif  // NAV2_CONTROLLER_LAB__MPPI_CORE_HPP_
