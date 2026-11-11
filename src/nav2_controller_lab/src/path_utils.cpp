// Copyright (c) 2026
// 路径几何工具实现
//
// Licensed under the Apache License, Version 2.0

#include "nav2_controller_lab/path_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nav2_controller_lab
{

double pathLength(const std::vector<Pose2D> & path)
{
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    length += distance(path[i - 1], path[i]);
  }
  return length;
}

PathProjection projectOntoPath(const Pose2D & robot, const std::vector<Pose2D> & path)
{
  PathProjection best;
  best.distance = std::numeric_limits<double>::infinity();
  best.path_yaw = robot.yaw;

  if (path.empty()) {
    return best;
  }

  if (path.size() == 1) {
    best.index = 0;
    best.point = path.front();
    best.distance = distance(robot, best.point);
    best.path_yaw = path.front().yaw;
    const double rx = robot.x - best.point.x;
    const double ry = robot.y - best.point.y;
    best.signed_lateral =
      std::cos(best.path_yaw) * ry - std::sin(best.path_yaw) * rx;
    return best;
  }

  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const Pose2D & a = path[i];
    const Pose2D & b = path[i + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    if (len2 <= kEpsilon) {
      continue;
    }

    const double t = clamp(
      ((robot.x - a.x) * dx + (robot.y - a.y) * dy) / len2, 0.0, 1.0);

    Pose2D point;
    point.x = a.x + t * dx;
    point.y = a.y + t * dy;
    const double yaw = std::atan2(dy, dx);
    point.yaw = yaw;

    const double d2 = squaredDistance(robot, point);
    if (d2 < best.distance * best.distance) {
      best.index = i;
      best.point = point;
      best.distance = std::sqrt(d2);
      best.path_yaw = yaw;

      const double rx = robot.x - point.x;
      const double ry = robot.y - point.y;
      // cross(tangent, robot - point)：左侧为正
      best.signed_lateral = std::cos(yaw) * ry - std::sin(yaw) * rx;
    }
  }

  if (!std::isfinite(best.distance)) {
    // 所有段都退化成一个点
    best.point = path.front();
    best.distance = distance(robot, best.point);
    best.path_yaw = path.front().yaw;
  }
  return best;
}

size_t findClosestIndex(const Pose2D & robot, const std::vector<Pose2D> & path)
{
  if (path.empty()) {
    return 0;
  }
  if (path.size() == 1) {
    return 0;
  }
  return projectOntoPath(robot, path).index;
}

bool findLookaheadPoint(
  const Pose2D & robot,
  const std::vector<Pose2D> & path,
  double lookahead,
  Pose2D & out)
{
  if (path.empty()) {
    return false;
  }
  if (path.size() == 1) {
    out = path.front();
    return true;
  }

  const PathProjection projection = projectOntoPath(robot, path);
  const double target = std::max(0.0, lookahead);

  if (projection.distance >= target) {
    out = projection.point;
    return true;
  }

  double previous_distance = projection.distance;
  Pose2D previous = projection.point;

  for (size_t i = projection.index; i + 1 < path.size(); ++i) {
    const Pose2D & next = path[i + 1];
    const double next_distance = distance(robot, next);

    if (next_distance >= target) {
      if (next_distance - previous_distance <= kEpsilon) {
        out = next;
        out.yaw = std::atan2(
          path[i + 1].y - previous.y, path[i + 1].x - previous.x);
        return true;
      }

      const double t = clamp(
        (target - previous_distance) / (next_distance - previous_distance),
        0.0, 1.0);
      out.x = previous.x + t * (next.x - previous.x);
      out.y = previous.y + t * (next.y - previous.y);
      out.yaw = std::atan2(next.y - previous.y, next.x - previous.x);
      return true;
    }

    previous = next;
    previous_distance = next_distance;
  }

  out = path.back();
  return true;
}

double distanceToPath(const Pose2D & robot, const std::vector<Pose2D> & path)
{
  return projectOntoPath(robot, path).distance;
}

double remainingDistance(const Pose2D & robot, const std::vector<Pose2D> & path)
{
  if (path.empty()) {
    return 0.0;
  }
  const PathProjection projection = projectOntoPath(robot, path);
  if (path.size() == 1) {
    return distance(robot, path.front());
  }

  double remaining = distance(projection.point, path[projection.index + 1]);
  for (size_t i = projection.index + 1; i + 1 < path.size(); ++i) {
    remaining += distance(path[i], path[i + 1]);
  }
  return remaining;
}

Pose2D toRobotFrame(const Pose2D & robot, const Pose2D & point)
{
  const double dx = point.x - robot.x;
  const double dy = point.y - robot.y;
  const double c = std::cos(robot.yaw);
  const double s = std::sin(robot.yaw);

  Pose2D out;
  out.x = c * dx + s * dy;
  out.y = -s * dx + c * dy;
  out.yaw = normalizeAngle(point.yaw - robot.yaw);
  return out;
}

}  // namespace nav2_controller_lab
