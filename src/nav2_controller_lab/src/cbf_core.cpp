// Copyright (c) 2026
// CBF 安全过滤器核心实现
//
// Licensed under the Apache License, Version 2.0

#include "nav2_controller_lab/cbf_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nav2_controller_lab
{

namespace
{

inline double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

}  // namespace

CbfFilter::CbfFilter(const CbfParams & params)
: params_(params)
{
}

CbfResult CbfFilter::filter(
  double nominal_v, double nominal_w,
  const std::vector<std::pair<double, double>> & scan)
{
  CbfResult result;
  result.linear = nominal_v;
  result.angular = nominal_w;

  // v 的可行区间，初始为底盘物理限幅
  double v_lb = params_.min_linear_velocity;
  double v_ub = params_.max_linear_velocity;
  double min_h = std::numeric_limits<double>::infinity();
  int binding_index = -1;

  for (size_t i = 0; i < scan.size(); ++i) {
    const double angle = scan[i].first;
    const double range = scan[i].second;
    if (range > params_.scan_influence_range) {
      continue;
    }

    // ---- 外参补偿：把扫描点从雷达系变换到机器人中心系 ----
    // 点在中心系 = (range·cosψ + lx, range·sinψ + ly)
    // TB3 waffle 的雷达在中心后方 6.4cm，不补偿时侧向束的
    // range ≠ 中心距离，h 会被错误判负（docs/17 §6 实车案例）。
    const double px = range * std::cos(angle) + params_.lidar_offset_x;
    const double py = range * std::sin(angle) + params_.lidar_offset_y;
    const double d = std::hypot(px, py);            // 中心到障碍点距离
    const double h = d - params_.d_safe;
    if (h < min_h) {
      min_h = h;
    }
    if (d > params_.scan_influence_range) {
      continue;
    }

    // 障碍点相对中心的方位角 θ；ḣ = -v·cos(θ)
    const double c = px / d;
    if (std::fabs(c) < params_.cos_epsilon) {
      // 一阶下切向运动不改变距离：该点不约束 v（见 docs/17 §3）
      continue;
    }

    // 约束 v·cos(θ) ≤ α·h（由 ḣ ≥ -α·h 移项而来）
    const double bound = params_.alpha * h / c;
    if (c > 0.0) {
      // 前方点 → v 的上界。记住"最紧的那个上界"来自谁（诊断）
      if (bound < v_ub) {
        v_ub = bound;
        binding_index = static_cast<int>(i);
      }
    } else {
      // 后方点 → v 的下界（倒车撞后方障碍时起作用）
      if (bound > v_lb) {
        v_lb = bound;
        binding_index = static_cast<int>(i);
      }
    }
  }

  result.min_h = (min_h == std::numeric_limits<double>::infinity()) ? 0.0 : min_h;

  if (v_lb > v_ub) {
    // 前后夹击（前有障碍逼倒车、后有障碍逼前进）→ 无可行速度
    // 最安全的裁决是停车。角速度不受影响（原地转是安全的）。
    result.infeasible = true;
    result.active = true;
    result.linear = 0.0;
    result.binding_index = binding_index;
    return result;
  }

  const double filtered = clamp(nominal_v, v_lb, v_ub);
  result.active = std::fabs(filtered - nominal_v) > 1e-9;
  result.linear = filtered;
  // 没修改指令时，binding 信息没有意义
  result.binding_index = result.active ? binding_index : -1;
  return result;
}

}  // namespace nav2_controller_lab
