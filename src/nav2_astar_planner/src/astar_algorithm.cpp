// Copyright (c) 2026
// A* 算法核心实现 —— 教学实现
//
// Licensed under the Apache License, Version 2.0

#include "nav2_astar_planner/astar_algorithm.hpp"

#include <chrono>
#include <functional>
#include <queue>
#include <vector>

namespace nav2_astar_planner
{

namespace
{
// nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE = 253
// nav2_costmap_2d::LETHAL_OBSTACLE = 254
// nav2_costmap_2d::NO_INFORMATION = 255
// 只有 254 (LETHAL_OBSTACLE) 是真正的物理障碍；253 是内切膨胀，可以走
constexpr uint8_t LETHAL_THRESHOLD = 254;
constexpr uint8_t NO_INFORMATION = 255;
}  // namespace

void AStarAlgorithm::setCostmap(
  const CostMap & costs, unsigned int width, unsigned int height)
{
  costs_ = costs;
  width_ = width;
  height_ = height;

  // 默认的可通行判定：非致命障碍（含内切膨胀障碍）即视为可通行
  if (!traversability_checker_) {
    traversability_checker_ = [](unsigned int, uint8_t cost) {
        return cost < LETHAL_THRESHOLD;
      };
  }
}

void AStarAlgorithm::setTraversabilityChecker(
  std::function<bool(unsigned int, uint8_t)> checker)
{
  traversability_checker_ = std::move(checker);
}

bool AStarAlgorithm::isTraversable(unsigned int index) const
{
  if (index >= costs_.size()) {
    return false;
  }

  const uint8_t cost = costs_[index];

  // 未知区域处理
  if (cost == NO_INFORMATION) {
    if (!options_.allow_unknown || options_.treat_unknown_as_lethal) {
      return false;
    }
    return true;
  }

  // 交给外部注入的判定器
  if (traversability_checker_) {
    return traversability_checker_(index, cost);
  }
  return cost < LETHAL_THRESHOLD;
}

double AStarAlgorithm::heuristic(unsigned int from, unsigned int to) const
{
  const int fx = static_cast<int>(from % width_);
  const int fy = static_cast<int>(from / width_);
  const int tx = static_cast<int>(to % width_);
  const int ty = static_cast<int>(to / width_);

  const double dx = std::abs(tx - fx);
  const double dy = std::abs(ty - fy);

  double h;
  if (options_.allow_diagonal) {
    // Octile 距离：8 邻域下的精确最小步数
    h = (dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy);
  } else {
    // 曼哈顿距离：4 邻域下的精确最小步数
    h = dx + dy;
  }

  return h * options_.heuristic_weight;
}

double AStarAlgorithm::moveCost(unsigned int from, unsigned int to, bool diagonal) const
{
  // 基础移动代价：直移为 1，对角为 sqrt(2)
  double cost = diagonal ? std::sqrt(2.0) : 1.0;

  // 叠加地形代价（让规划器倾向走低代价区域，远离障碍）
  if (options_.cost_weight > 0.0) {
    const uint8_t cell_cost = costs_[to];
    if (cell_cost != NO_INFORMATION) {
      // nav2 代价 0-252 映射为 0.0-1.0 的额外系数
      const double normalized = static_cast<double>(cell_cost) / 252.0;
      cost += options_.cost_weight * normalized;
    }
  }

  return cost;
}

AStarAlgorithm::Result AStarAlgorithm::plan(
  const GridCoord & start, const GridCoord & goal,
  std::function<bool()> cancel_checker)
{
  const auto t_start = std::chrono::steady_clock::now();

  path_.clear();
  stats_ = Stats();

  // ---------- 参数与边界校验 ----------
  if (!inBounds(start.x, start.y)) {
    return Result::START_INVALID;
  }
  if (!inBounds(goal.x, goal.y)) {
    return Result::GOAL_INVALID;
  }

  const unsigned int start_index = toIndex(start.x, start.y);
  const unsigned int goal_index = toIndex(goal.x, goal.y);

  // 起点判定：起点通常位于机器人当前位置，即使代价稍高也应允许出发，
  // 但如果是硬障碍则无意义。这里对起点放宽为「非致命」。
  if (costs_[start_index] >= LETHAL_THRESHOLD && costs_[start_index] != NO_INFORMATION) {
    // 起点在致命障碍内，仍允许尝试（机器人可能刚被推入），但记录
  }
  if (!isTraversable(goal_index)) {
    return Result::GOAL_INVALID;
  }

  // ---------- 起点即终点 ----------
  if (start_index == goal_index) {
    path_.push_back(start);
    stats_.path_length = 1;
    const auto t_end = std::chrono::steady_clock::now();
    stats_.planning_time_ms =
      std::chrono::duration<double, std::milli>(t_end - t_start).count();
    return Result::SUCCESS;
  }

  const size_t map_size = static_cast<size_t>(width_) * height_;
  const double INF = std::numeric_limits<double>::infinity();

  // g_score：起点到各节点的已知最小代价
  std::vector<double> g_score(map_size, INF);
  // parent：路径回溯用
  std::vector<unsigned int> parent(map_size, std::numeric_limits<unsigned int>::max());
  // closed：是否已扩展
  std::vector<uint8_t> closed(map_size, 0);
  // in_open：是否已在 open_list 中，避免重复插入
  std::vector<uint8_t> in_open(map_size, 0);

  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;

  g_score[start_index] = 0.0;
  {
    Node n;
    n.index = start_index;
    n.g = 0.0;
    n.f = heuristic(start_index, goal_index);
    n.parent = start_index;
    open_list.push(n);
    in_open[start_index] = 1;
    stats_.nodes_opened++;
  }

  // 8 邻域方向：先直后斜，顺序影响搜索偏好但不影响最优性
  static const int DIRS[8][2] = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},      // 4 直移
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}     // 4 对角
  };

  bool found = false;
  size_t expansions = 0;

  while (!open_list.empty()) {
    // 定期检查取消请求，避免长搜索无法中断
    if (cancel_checker && (expansions % 256 == 0) && cancel_checker()) {
      return Result::CANCELLED;
    }

    // 弹出 f 最小的节点
    Node current = open_list.top();
    open_list.pop();
    in_open[current.index] = 0;

    // 已扩展过则跳过（可能有多个旧副本留在堆中）
    if (closed[current.index]) {
      continue;
    }
    closed[current.index] = 1;
    stats_.nodes_expanded++;

    // 到达终点
    if (current.index == goal_index) {
      found = true;
      break;
    }

    if (++expansions > options_.max_expansions) {
      break;
    }

    const int cx = static_cast<int>(current.index % width_);
    const int cy = static_cast<int>(current.index / width_);

    const int dir_count = options_.allow_diagonal ? 8 : 4;

    for (int d = 0; d < dir_count; ++d) {
      const int nx = cx + DIRS[d][0];
      const int ny = cy + DIRS[d][1];

      if (!inBounds(nx, ny)) {
        continue;
      }

      const unsigned int n_index = toIndex(nx, ny);

      if (closed[n_index]) {
        continue;
      }
      if (!isTraversable(n_index)) {
        continue;
      }

      const bool diagonal = (DIRS[d][0] != 0 && DIRS[d][1] != 0);

      // 对角移动的「切角」检查：若两个相邻直移格子都是障碍，
      // 则禁止从缝隙中斜穿过去，否则机器人的足迹会撞到障碍角。
      if (diagonal) {
        const unsigned int adj1 = toIndex(cx + DIRS[d][0], cy);
        const unsigned int adj2 = toIndex(cx, cy + DIRS[d][1]);
        if (!isTraversable(adj1) || !isTraversable(adj2)) {
          continue;
        }
      }

      const double tentative_g = g_score[current.index] + moveCost(current.index, n_index, diagonal);

      if (tentative_g < g_score[n_index]) {
        g_score[n_index] = tentative_g;
        parent[n_index] = current.index;

        Node neighbor;
        neighbor.index = n_index;
        neighbor.g = tentative_g;
        neighbor.f = tentative_g + heuristic(n_index, goal_index);
        neighbor.parent = current.index;
        open_list.push(neighbor);

        if (!in_open[n_index]) {
          in_open[n_index] = 1;
          stats_.nodes_opened++;
        }
      }
    }
  }

  if (!found) {
    const auto t_end = std::chrono::steady_clock::now();
    stats_.planning_time_ms =
      std::chrono::duration<double, std::milli>(t_end - t_start).count();
    return Result::NO_PATH;
  }

  // ---------- 回溯路径 ----------
  std::vector<GridCoord> reversed;
  unsigned int idx = goal_index;
  while (idx != start_index) {
    reversed.push_back({static_cast<int>(idx % width_), static_cast<int>(idx / width_)});
    idx = parent[idx];
    // 防御：parent 链断裂
    if (reversed.size() > map_size) {
      break;
    }
  }
  reversed.push_back(start);

  // 反转为 起点 -> 终点 顺序
  path_.assign(reversed.rbegin(), reversed.rend());

  stats_.path_length = path_.size();
  stats_.path_cost = g_score[goal_index];

  const auto t_end = std::chrono::steady_clock::now();
  stats_.planning_time_ms =
    std::chrono::duration<double, std::milli>(t_end - t_start).count();

  return Result::SUCCESS;
}

void AStarAlgorithm::gridToWorld(
  int gx, int gy, double origin_x, double origin_y,
  double resolution, double & wx, double & wy)
{
  // 取格子中心，避免落在格线上
  wx = origin_x + (static_cast<double>(gx) + 0.5) * resolution;
  wy = origin_y + (static_cast<double>(gy) + 0.5) * resolution;
}

}  // namespace nav2_astar_planner
