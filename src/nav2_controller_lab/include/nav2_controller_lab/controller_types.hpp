// Copyright (c) 2026
// 纯 C++ 控制算法公共类型 —— 不依赖 ROS
//
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_CONTROLLER_LAB__CONTROLLER_TYPES_HPP_
#define NAV2_CONTROLLER_LAB__CONTROLLER_TYPES_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace nav2_controller_lab
{

constexpr double kEpsilon = 1e-9;

/**
 * @brief 平面位姿（米、弧度）
 */
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/**
 * @brief 速度指令
 */
struct Control
{
  double linear{0.0};    ///< m/s
  double angular{0.0};   ///< rad/s
};

/**
 * @brief 控制器核心的计算结果
 */
struct ControlResult
{
  Control control;
  bool valid{true};
  double cost{0.0};
};

inline double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

/**
 * @brief 把角度归一化到 [-pi, pi]
 */
inline double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline double distance(const Pose2D & a, const Pose2D & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

inline double squaredDistance(const Pose2D & a, const Pose2D & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

inline double yawFromVector(double dx, double dy)
{
  return std::atan2(dy, dx);
}

/**
 * @brief 按线速度/角速度上限裁剪控制量
 */
inline Control clampControl(const Control & command, double max_linear, double max_angular)
{
  Control out;
  out.linear = clamp(command.linear, -max_linear, max_linear);
  out.angular = clamp(command.angular, -max_angular, max_angular);
  return out;
}

/**
 * @brief 把路径点的朝向按“相邻点切线”补齐
 *
 * 全局规划器通常会填好朝向，但有些路径只有位置。控制器只关心切线方向，
 * 所以这里统一用相邻点重算，避免依赖上游是否填了四元数。
 */
inline void recomputePathYaw(std::vector<Pose2D> & path)
{
  if (path.size() < 2) {
    return;
  }
  for (size_t i = 0; i < path.size(); ++i) {
    const size_t prev = (i == 0) ? 0 : i - 1;
    const size_t next = std::min(i + 1, path.size() - 1);
    const double dx = path[next].x - path[prev].x;
    const double dy = path[next].y - path[prev].y;
    if (std::hypot(dx, dy) > kEpsilon) {
      path[i].yaw = std::atan2(dy, dx);
    }
  }
}

}  // namespace nav2_controller_lab

#endif  // NAV2_CONTROLLER_LAB__CONTROLLER_TYPES_HPP_
