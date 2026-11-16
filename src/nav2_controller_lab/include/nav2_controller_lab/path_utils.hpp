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
 * @brief 终点旋转指令：原地旋转到目标朝向（纯 P 控制）
 *
 * 六种控制器核心都只负责"跟路径"，路径走完就输出零速，没有
 * "原地转到目标朝向"的逻辑。而 nav2 的 SimpleGoalChecker 同时检查
 * 位置和朝向，缺了这一步就会死锁：机器人停在目标旁边不动，
 * 导航永远不返回成功。官方控制器也有对应阶段（RPP 的
 * rotateToGoalHeading、MPPI 的 GoalAngleCritic）。
 *
 * @param robot 机器人位姿
 * @param goal_yaw 目标朝向（弧度；A* 路径末位姿已写入目标朝向）
 * @param max_angular_velocity 旋转角速度上限（rad/s）
 * @return linear=0，angular 为限幅后的 P 控制旋转指令
 */
Control rotateInPlace(const Pose2D & robot, double goal_yaw, double max_angular_velocity);

/**
 * @brief 把点变换到机器人坐标系
 */
Pose2D toRobotFrame(const Pose2D & robot, const Pose2D & point);

}  // namespace nav2_controller_lab

#endif  // NAV2_CONTROLLER_LAB__PATH_UTILS_HPP_
