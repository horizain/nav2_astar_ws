// Copyright (c) 2026
// DWB-lite：采样 (v, w) + 轨迹评分 —— 纯 C++
//
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_CONTROLLER_LAB__DWB_CORE_HPP_
#define NAV2_CONTROLLER_LAB__DWB_CORE_HPP_

#include <cstdint>
#include <functional>
#include <vector>

#include "nav2_controller_lab/controller_types.hpp"
#include "nav2_controller_lab/trajectory_cost.hpp"

namespace nav2_controller_lab
{

struct DwbParams
{
  double min_linear_velocity{0.0};
  double max_linear_velocity{0.5};
  double max_angular_velocity{1.9};

  int linear_samples{7};
  int angular_samples{11};
  double horizon{1.5};
  double dt{0.1};
  double goal_tolerance{0.25};

  TrajectoryCostParams cost;
};

/**
 * @brief 简化版 DWB
 *
 * 每个控制周期采样 (v, w) 网格，rollout 未来轨迹，用 critics 评分，
 * 选代价最低的一组，执行第一个控制量。
 */
class DwbCore
{
public:
  explicit DwbCore(const DwbParams & params = DwbParams());

  ControlResult compute(
    const Pose2D & robot,
    const std::vector<Pose2D> & path,
    const std::function<uint8_t(double, double)> & cost_at,
    double current_speed);

  void reset() {}

  const DwbParams & params() const {return params_;}
  DwbParams & params() {return params_;}

private:
  DwbParams params_;
};

}  // namespace nav2_controller_lab

#endif  // NAV2_CONTROLLER_LAB__DWB_CORE_HPP_
