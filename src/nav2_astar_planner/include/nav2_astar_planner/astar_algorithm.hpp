// Copyright (c) 2026
// 基于 A* 算法的全局路径规划器 —— 教学实现
//
// 本文件实现纯算法部分，不依赖任何 ROS 类型，便于单元测试和理解。
// 与 ROS 的粘合逻辑在 astar_planner.cpp 中。
//
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_ASTAR_PLANNER__ASTAR_ALGORITHM_HPP_
#define NAV2_ASTAR_PLANNER__ASTAR_ALGORITHM_HPP_

#include <vector>
#include <queue>
#include <limits>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <functional>
#include <utility>

namespace nav2_astar_planner
{

/**
 * @brief 栅格坐标（单位：格子）
 */
struct GridCoord
{
  int x;
  int y;
};

/**
 * @brief A* 算法核心实现
 *
 * 设计要点（教学重点）：
 * 1. 代价地图被抽象为「宽度 x 高度」的一维代价数组，避免二维 vector 的额外开销
 * 2. 使用「一维索引」访问，index = y * width + x
 * 3. open_list 使用优先队列（最小堆），按 f = g + h 排序
 * 4. closed_list 记录已扩展节点，避免重复处理
 * 5. 通过 can_traverse 回调判断格子是否可通过，从而与 costmap 解耦
 */
class AStarAlgorithm
{
public:
  using CostMap = std::vector<uint8_t>;

  /**
   * @brief 规划结果状态
   */
  enum class Result
  {
    SUCCESS,        ///< 成功找到路径
    NO_PATH,        ///< 搜索空间耗尽，无可行路径
    START_INVALID,  ///< 起点非法（越界或不可通行）
    GOAL_INVALID,   ///< 终点非法（越界或不可通行）
    CANCELLED       ///< 被外部取消
  };

  /**
   * @brief 规划参数
   */
  struct Options
  {
    /// 是否允许 8 邻域扩展（false 则只用 4 邻域）
    bool allow_diagonal = true;
    /// 是否允许穿越未知区域（NO_INFORMATION）
    bool allow_unknown = true;
    /// 代价缩放：把 costmap 的代价（0-254）转换为额外通行代价的系数
    double cost_weight = 0.0;
    /// 启发式函数权重，1.0 为标准 A*（可采纳），>1.0 为贪婪（更快但非最优）
    double heuristic_weight = 1.0;
    /// 节点扩展上限，防止病态地图上无限搜索
    size_t max_expansions = 1000000;
    /// 障碍物膨胀视角：是否把 "可能碰撞的代价" 视为不可通行
    bool treat_unknown_as_lethal = false;
  };

  /**
   * @brief 规划结果统计（用于教学观察算法行为）
   */
  struct Stats
  {
    size_t nodes_expanded = 0;   ///< 实际扩展的节点数
    size_t nodes_opened = 0;     ///< 曾加入 open_list 的节点数
    size_t path_length = 0;      ///< 路径节点数
    double path_cost = 0.0;      ///< 路径总代价
    double planning_time_ms = 0.0;  ///< 规划耗时（毫秒）
  };

  AStarAlgorithm() = default;

  /**
   * @brief 设置代价地图
   * @param costs 一维代价数组，长度必须等于 width * height
   * @param width 地图宽度（格子数）
   * @param height 地图高度（格子数）
   */
  void setCostmap(const CostMap & costs, unsigned int width, unsigned int height);

  /**
   * @brief 判断某格子是否可通行
   *
   * 该方法由外部注入，使得算法本身不依赖 nav2_costmap_2d。
   * 默认实现：代价 < 253（即非致命障碍）即可通行。
   */
  void setTraversabilityChecker(std::function<bool(unsigned int index, uint8_t cost)> checker);

  /**
   * @brief 覆盖算法选项
   *
   * 供 ROS 封装层在每次规划前把参数同步进来。
   */
  void setOptionOverrides(
    bool allow_diagonal, bool allow_unknown,
    double cost_weight, double heuristic_weight,
    size_t max_expansions)
  {
    options_.allow_diagonal = allow_diagonal;
    options_.allow_unknown = allow_unknown;
    options_.cost_weight = cost_weight;
    options_.heuristic_weight = heuristic_weight;
    options_.max_expansions = max_expansions;
  }

  /**
   * @brief 执行 A* 搜索
   * @param start 起点（格子坐标）
   * @param goal 终点（格子坐标）
   * @param cancel_checker 取消检查回调，返回 true 表示应中止
   * @return 规划结果状态
   *
   * 成功后，可用 getPath() 取回路径（从起点到终点的格子序列）。
   */
  Result plan(
    const GridCoord & start, const GridCoord & goal,
    std::function<bool()> cancel_checker = nullptr);

  /**
   * @brief 取回上一次规划得到的路径
   */
  const std::vector<GridCoord> & getPath() const {return path_;}

  /**
   * @brief 取回统计信息
   */
  const Stats & getStats() const {return stats_;}

  /**
   * @brief 把格子坐标转换为世界坐标
   */
  static void gridToWorld(
    int gx, int gy, double origin_x, double origin_y,
    double resolution, double & wx, double & wy);

private:
  /**
   * @brief open_list 中的节点
   */
  struct Node
  {
    double f = 0.0;      ///< f = g + h
    double g = 0.0;      ///< 从起点到该节点的实际代价
    unsigned int index = 0;
    unsigned int parent = 0;

    // 优先队列是最大堆，这里反转比较实现最小堆
    bool operator>(const Node & other) const {return f > other.f;}
  };

  /**
   * @brief 计算启发式代价 h
   *
   * 采用「八方向距离」（octile distance）：
   *   h = (dx + dy) + (sqrt(2) - 2) * min(dx, dy)
   * 这在 8 邻域且有对角移动时是可采纳（admissible）的。
   * 若只允许 4 邻域，则退化为曼哈顿距离。
   */
  double heuristic(unsigned int from, unsigned int to) const;

  /**
   * @brief 计算两点间的移动代价
   */
  double moveCost(unsigned int from, unsigned int to, bool diagonal) const;

  /**
   * @brief 检查坐标是否在地图内
   */
  bool inBounds(int x, int y) const
  {
    return x >= 0 && y >= 0 && x < static_cast<int>(width_) && y < static_cast<int>(height_);
  }

  /**
   * @brief 一维索引
   */
  unsigned int toIndex(int x, int y) const
  {
    return static_cast<unsigned int>(y) * width_ + static_cast<unsigned int>(x);
  }

  /**
   * @brief 判断索引处是否可通行（含未知区域策略）
   */
  bool isTraversable(unsigned int index) const;

  CostMap costs_;
  unsigned int width_ = 0;
  unsigned int height_ = 0;

  Options options_;
  std::vector<GridCoord> path_;
  Stats stats_;

  std::function<bool(unsigned int, uint8_t)> traversability_checker_;
};

}  // namespace nav2_astar_planner

#endif  // NAV2_ASTAR_PLANNER__ASTAR_ALGORITHM_HPP_
