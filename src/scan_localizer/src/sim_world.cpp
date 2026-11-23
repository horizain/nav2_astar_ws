// Copyright 2026 ruijianding
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// ============================================================================
// 仿真世界实现
// ============================================================================
#include "scan_localizer/sim_world.hpp"

#include <cmath>

namespace sloc::sim
{

namespace
{
void box(OccupancyGrid & g, double x0, double y0, double x1, double y1)
{
  rasterize_segment(g, x0, y0, x1, y0);
  rasterize_segment(g, x1, y0, x1, y1);
  rasterize_segment(g, x1, y1, x0, y1);
  rasterize_segment(g, x0, y1, x0, y0);
}
// 外墙以外的区域置为未知（模拟真实建图：内部已知自由）
void mark_exterior_unknown(
  OccupancyGrid & g, double x0, double y0, double x1,
  double y1)
{
  const auto a = g.worldToGridF(x0, y0);
  const auto b = g.worldToGridF(x1, y1);
  const int ax = static_cast<int>(std::floor(a.x())), ay =
    static_cast<int>(std::floor(a.y()));
  const int bx = static_cast<int>(std::ceil(b.x())), by =
    static_cast<int>(std::ceil(b.y()));
  for (int y = 0; y < g.height; ++y) {
    for (int x = 0; x < g.width; ++x) {
      if (x < ax || x > bx || y < ay || y > by) {
        g.cells[static_cast<std::size_t>(y) * g.width + x] = -1;
      }
    }
  }
}
}  // namespace

OccupancyGrid make_warehouse(double resolution)
{
  OccupancyGrid g;
  g.resolution = resolution;
  g.width = static_cast<int>(40.0 / resolution);
  g.height = static_cast<int>(30.0 / resolution);
  g.origin_x = 0.0;
  g.origin_y = 0.0;
  g.cells.assign(static_cast<std::size_t>(g.width) * g.height, 0);
    // 内部已知自由，外墙与结构 → 占据
  box(g, 0.05, 0.05, 39.95, 29.95);
  mark_exterior_unknown(g, 0.05, 0.05, 39.95, 29.95);
    // 隔间墙
  rasterize_segment(g, 15, 0.05, 15, 12);
  rasterize_segment(g, 15, 12, 25, 12);
  rasterize_segment(g, 25, 12, 25, 29.95);
  rasterize_segment(g, 30, 0.05, 30, 18);
  rasterize_segment(g, 30, 18, 39.95, 18);
    // 货架（细长矩形组，右区）
  for (int i = 0; i < 5; ++i) {
    box(g, 32, 3 + i * 3.0, 38, 4.6 + i * 3.0);
  }
    // 柱子阵列（左区）
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 2; ++j) {
      const double cx = 4.0 + i * 4.0, cy = 6.0 + j * 5.0;
      box(g, cx - 0.3, cy - 0.3, cx + 0.3, cy + 0.3);
    }
  }
    // 内部凸台
  box(g, 8, 20, 12, 22);
  return g;
}

OccupancyGrid make_corridor(double resolution)
{
  OccupancyGrid g;
  g.resolution = resolution;
  g.width = static_cast<int>(30.0 / resolution);
  g.height = static_cast<int>(3.0 / resolution);
  g.origin_x = 0.0;
  g.origin_y = 0.0;
  g.cells.assign(static_cast<std::size_t>(g.width) * g.height, 0);
  box(g, 0.05, 0.05, 29.95, 2.95);
  mark_exterior_unknown(g, 0.05, 0.05, 29.95, 2.95);
  return g;
}

namespace
{
bool in_dyn(const std::vector<DynBox> & dyn, double x, double y)
{
  for (const auto & b : dyn) {
    if (std::abs(x - b.cx) <= b.hw && std::abs(y - b.cy) <= b.hh) {
      return true;
    }
  }
  return false;
}
}  // namespace

Scan make_scan(
  const OccupancyGrid & grid, const Pose2 & pose,
  const ScanSimConfig & cfg, std::mt19937 & rng,
  const std::vector<DynBox> & dyn)
{
  Scan out;
  std::normal_distribution<double> noise(0.0, cfg.noise_std);
  const double dt = grid.resolution / 4.0;    // 采样步长（1/4 cell）
  const double px = pose.x, py = pose.y;
  auto occ_at = [&](double x, double y) {
      if (in_dyn(dyn, x, y)) {return true;}
      const auto g = grid.worldToGridF(x, y);
      const int ix = static_cast<int>(std::lround(g.x()));
      const int iy = static_cast<int>(std::lround(g.y()));
      if (!grid.inside(ix, iy)) {return false;}
      return grid.occupied(ix, iy);
    };
  for (int i = 0; i < cfg.beams; i += std::max(1, cfg.subsample)) {
    const double a = pose.theta - 0.5 * cfg.fov +
      cfg.fov * (i + 0.5) / cfg.beams;
    const double dx = std::cos(a), dy = std::sin(a);
    double t = cfg.min_range;
    double hit = -1.0;
    while (t < cfg.range_max) {
      const double sx = px + dx * t, sy = py + dy * t;
      const auto g = grid.worldToGridF(sx, sy);
      const int ix = static_cast<int>(std::lround(g.x()));
      const int iy = static_cast<int>(std::lround(g.y()));
      if (!grid.inside(ix, iy)) {
        break;          // 出图
      }
      if (occ_at(sx, sy)) {
                // 二分细化到亚毫米（消除步长量化偏差）
        double lo = std::max(cfg.min_range, t - dt), hi = t;
        for (int bi = 0; bi < 8; ++bi) {
          const double mid = 0.5 * (lo + hi);
          if (occ_at(px + dx * mid, py + dy * mid)) {
            hi = mid;
          } else {
            lo = mid;
          }
        }
        hit = hi;
        break;
      }
      t += dt;
    }
    if (hit < 0.0) {continue;}
    const double r = hit + noise(rng);
    if (r < cfg.min_range) {continue;}
        // 传感器系点
    const double ca = std::cos(a - pose.theta), sa = std::sin(a - pose.theta);
    out.points.emplace_back(r * ca, r * sa);
  }
  return out;
}

std::vector<Pose2> random_free_poses(
  const OccupancyGrid & grid, int n,
  std::mt19937 & rng)
{
  std::uniform_real_distribution<double> ux(
    1.5, grid.width * grid.resolution - 1.5);
  std::uniform_real_distribution<double> uy(
    1.5, grid.height * grid.resolution - 1.5);
  std::uniform_real_distribution<double> ut(-M_PI, M_PI);
  std::vector<Pose2> out;
    // 合法位姿：1.5m 半径内完全自由（保证扫描信息量充足、无退化帧）
  const int r_cells = static_cast<int>(1.5 / grid.resolution);
  while (static_cast<int>(out.size()) < n) {
    const double x = ux(rng), y = uy(rng);
    const auto g = grid.worldToGridF(x, y);
    const int gx = static_cast<int>(std::lround(g.x()));
    const int gy = static_cast<int>(std::lround(g.y()));
    bool ok = true;
    for (int dy = -r_cells; dy <= r_cells && ok; ++dy) {
      for (int dx = -r_cells; dx <= r_cells; ++dx) {
        if (dx * dx + dy * dy > r_cells * r_cells) {continue;}
        const int ix = gx + dx, iy = gy + dy;
        if (!grid.inside(ix, iy) ||
          grid.cells[static_cast<std::size_t>(iy) * grid.width +
          ix] != 0)
        {
          ok = false;            // 占据或未知都不算自由
          break;
        }
      }
    }
    if (ok) {out.push_back({x, y, ut(rng)});}
  }
  return out;
}

}  // namespace sloc::sim
