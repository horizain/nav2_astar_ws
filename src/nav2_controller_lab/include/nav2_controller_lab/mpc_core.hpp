// Copyright (c) 2026
// MPC：多初值、带加速度约束的确定性滚动搜索
//
// 和 MPPI 的区别：
//   MPPI：随机采样控制序列，软最大加权
//   MPC ：对多个可行初值做控制块模式搜索，投影到速度与加速度约束内
//
// 这是一个教学用简化 MPC，不是工业级 QP/NLP 求解器。
//
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_CONTROLLER_LAB__MPC_CORE_HPP_
#define NAV2_CONTROLLER_LAB__MPC_CORE_HPP_

#include <cstdint>
#include <functional>
#include <vector>

#include "nav2_controller_lab/controller_types.hpp"
#include "nav2_controller_lab/trajectory_cost.hpp"

namespace nav2_controller_lab
{

struct MpcParams
{
  double min_linear_velocity{0.0};
  double max_linear_velocity{0.5};
  double max_angular_velocity{1.9};

  int horizon_steps{15};
  double dt{0.1};
  int iterations{8};
  int line_search_steps{3};

  double gradient_step{0.05};
  double gradient_epsilon{0.01};

  double max_linear_acceleration{1.0};
  double max_angular_acceleration{3.2};
  double goal_tolerance{0.25};
  double terminal_weight{2.0};
  double smoothness_weight{0.05};

  TrajectoryCostParams cost = [] {
      TrajectoryCostParams value;
      value.path_weight = 5.0;
      value.obstacle_weight = 8.0;
      value.collision_threshold = 253;
      return value;
    }();
};

/**
 * @brief 简化 MPC
 *
 * 代价：
 *   path/goal/obstacle/heading/velocity/control（复用 TrajectoryCost）
 *   + terminal_weight * 终点距离
 *   + smoothness_weight * 相邻控制量差
 *
 * 约束：
 *   v ∈ [min_v, max_v]
 *   ω ∈ [-max_w, max_w]
 *
 * 求解：
 *   结合上一周期解和路径引导初值，对控制块做确定性模式搜索。
 *   每个候选先投影到速度与加速度约束，再完整检查预测轨迹。
 */
class MpcCore
{
public:
  explicit MpcCore(const MpcParams & params = MpcParams());

  ControlResult compute(
    const Pose2D & robot,
    const std::vector<Pose2D> & path,
    const std::function<uint8_t(double, double)> & cost_at,
    double current_speed, double control_dt = 0.05, double current_angular = 0.0);

  void reset();

  const MpcParams & params() const {return params_;}
  MpcParams & params() {return params_;}

private:
  double trajectoryCost(
    const Pose2D & robot,
    const std::vector<Pose2D> & path,
    const std::function<uint8_t(double, double)> & cost_at,
    const std::vector<Control> & controls) const;

  Control clampControl(const Control & control) const;

  MpcParams params_;
  std::vector<Control> nominal_;
  bool initialized_{false};
};

}  // namespace nav2_controller_lab

#endif  // NAV2_CONTROLLER_LAB__MPC_CORE_HPP_
