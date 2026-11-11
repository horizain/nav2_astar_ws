// Copyright (c) 2026
// 路径几何工具 —— 纯 C++，不依赖 ROS
//
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_CONTROLLER_LAB__PATH_UTILS_HPP_
#define NAV2_CONTROLLER_LAB__PATH_UTILS_HPP_

#include <cstddef>
#include <vector>

#include "nav2_controller_lab/controller_types.hpp"

namespace nav2_controller_lab
{

/**
 * @brief 机器人在路径上的投影结果
 */
struct PathProjection
{
  size_t index{0};           ///< 投影所在段的起点索引
  Pose2D point;              ///< 路径上的投影点
  double distance{0.0};      ///< 机器人到投影点的距离
  double signed_lateral{0.0};///< 机器人在路径左侧为正
  double path_yaw{0.0};      ///< 投影处的路径切线方向
};

/**
 * @brief 计算路径总长度
 */
double pathLength(const std::vector<Pose2D> & path);

/**
 * @brief 把机器人投影到路径折线上（逐段投影，不是只找最近点）
 */
PathProjection projectOntoPath(const Pose2D & robot, const std::vector<Pose2D> & path);

/**
 * @brief 找最近路径点的索引（简单版本，用于调试/测试）
 */
size_t findClosestIndex(const Pose2D & robot, const std::vector<Pose2D> & path);

/**
 * @brief 找前视点
 *
 * 从机器人在路径上的投影开始，沿路径向前走，找到距离机器人约
 * lookahead 的点；如果路径不够长，返回最后一个点。
 *
 * @param robot 机器人位姿
 * @param path 路径（至少 1 个点）
 * @param lookahead 前视距离（米）
 * @param out 输出前视点
 * @return 找到前视点返回 true；路径为空返回 false
 */
bool findLookaheadPoint(
  const Pose2D & robot,
  const std::vector<Pose2D> & path,
  double lookahead,
  Pose2D & out);

/**
 * @brief 机器人到路径的最短距离
 */
double distanceToPath(const Pose2D & robot, const std::vector<Pose2D> & path);

/**
 * @brief 从机器人投影点到路径终点的剩余弧长
 */
double remainingDistance(const Pose2D & robot, const std::vector<Pose2D> & path);

/**
 * @brief 把点变换到机器人坐标系
 */
Pose2D toRobotFrame(const Pose2D & robot, const Pose2D & point);

}  // namespace nav2_controller_lab

#endif  // NAV2_CONTROLLER_LAB__PATH_UTILS_HPP_
