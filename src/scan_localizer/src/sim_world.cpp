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

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

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

// 外墙以外的区域置为未知（四边对称 1-cell 未知边框，与 rasterize_segment 格心对齐）
void mark_exterior_unknown(
  OccupancyGrid & g, double x0, double y0, double x1, double y1)
{
  const auto a = g.worldToGridF(x0, y0);
  const auto b = g.worldToGridF(x1, y1);
  const int ax = static_cast<int>(std::lround(a.x()));
  const int ay = static_cast<int>(std::lround(a.y()));
  const int bx = static_cast<int>(std::lround(b.x()));
  const int by = static_cast<int>(std::lround(b.y()));
  for (int y = 0; y < g.height; ++y) {
    for (int x = 0; x < g.width; ++x) {
      if (x < ax || x > bx || y < ay || y > by) {
        g.cells[static_cast<std::size_t>(y) *
          static_cast<std::size_t>(g.width) +
          static_cast<std::size_t>(x)] = -1;
      }
    }
  }
}

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

OccupancyGrid make_warehouse(double resolution)
{
  OccupancyGrid g;
  g.resolution = resolution;
  g.width = static_cast<int>(std::lround(40.0 / resolution));
  g.height = static_cast<int>(std::lround(30.0 / resolution));
  g.origin_x = 0.0;
  g.origin_y = 0.0;
  g.cells.assign(
    static_cast<std::size_t>(g.width) * static_cast<std::size_t>(g.height), 0);
  // 外墙放在第 1 格与第 (N-2) 格中心（距边缘 1.5*res），四边各留 1 格对称未知边框
  const double margin = 1.5 * resolution;
  const double max_x = 40.0 - margin;
  const double max_y = 30.0 - margin;
  box(g, margin, margin, max_x, max_y);
  mark_exterior_unknown(g, margin, margin, max_x, max_y);
  // 隔间墙
  rasterize_segment(g, 15.0, margin, 15.0, 12.0);
  rasterize_segment(g, 15.0, 12.0, 25.0, 12.0);
  rasterize_segment(g, 25.0, 12.0, 25.0, max_y);
  rasterize_segment(g, 30.0, margin, 30.0, 18.0);
  rasterize_segment(g, 30.0, 18.0, max_x, 18.0);
  // 货架（细长矩形组，右区）
  for (int i = 0; i < 5; ++i) {
    box(g, 32.0, 3.0 + i * 3.0, 38.0, 4.6 + i * 3.0);
  }
  // 柱子阵列（左区）
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 2; ++j) {
      const double cx = 4.0 + i * 4.0;
      const double cy = 6.0 + j * 5.0;
      box(g, cx - 0.3, cy - 0.3, cx + 0.3, cy + 0.3);
    }
  }
  // 内部凸台
  box(g, 8.0, 20.0, 12.0, 22.0);
  return g;
}

OccupancyGrid make_corridor(double resolution)
{
  OccupancyGrid g;
  g.resolution = resolution;
  g.width = static_cast<int>(std::lround(30.0 / resolution));
  g.height = static_cast<int>(std::lround(3.0 / resolution));
  g.origin_x = 0.0;
  g.origin_y = 0.0;
  g.cells.assign(
    static_cast<std::size_t>(g.width) * static_cast<std::size_t>(g.height), 0);
  const double margin = 1.5 * resolution;
  const double max_x = 30.0 - margin;
  const double max_y = 3.0 - margin;
  box(g, margin, margin, max_x, max_y);
  mark_exterior_unknown(g, margin, margin, max_x, max_y);
  return g;
}

OccupancyGrid make_twin_rooms(double resolution)
{
  OccupancyGrid g;
  g.resolution = resolution;
  g.width = static_cast<int>(std::lround(24.0 / resolution));
  g.height = static_cast<int>(std::lround(10.0 / resolution));
  g.origin_x = 0.0;
  g.origin_y = 0.0;
  g.cells.assign(
    static_cast<std::size_t>(g.width) * static_cast<std::size_t>(g.height), 0);

  const double margin = 1.5 * resolution;
  const double max_x = 24.0 - margin;
  const double max_y = 10.0 - margin;
  box(g, margin, margin, max_x, max_y);
  mark_exterior_unknown(g, margin, margin, max_x, max_y);

  // Solid middle block x in [10.0, 14.0], y in [margin, 6.5]
  rasterize_segment(g, 10.0, margin, 10.0, 6.5);
  rasterize_segment(g, 14.0, margin, 14.0, 6.5);
  rasterize_segment(g, 10.0, 6.5, 14.0, 6.5);

  // Mark interior cells strictly inside x in (10.05, 13.95), y in (margin, 6.45) as -1 unknown
  for (int y = 0; y < g.height; ++y) {
    for (int x = 0; x < g.width; ++x) {
      const auto w = g.gridToWorld(x, y);
      const double wx = w.x();
      const double wy = w.y();
      if (wx > 10.05 && wx < 13.95 && wy > margin && wy < 6.45) {
        g.cells[static_cast<std::size_t>(y) * static_cast<std::size_t>(g.width) +
          static_cast<std::size_t>(x)] = -1;
      }
    }
  }

  // Left room (x in [margin, 10.0], y in [margin, 6.5]): doorway gap x in [4.2, 5.8]
  rasterize_segment(g, margin, 6.5, 4.2, 6.5);
  rasterize_segment(g, 5.8, 6.5, 10.0, 6.5);
  // L-pillar in Left room
  rasterize_segment(g, 2.5, 2.5, 4.0, 2.5);
  rasterize_segment(g, 2.5, 2.5, 2.5, 4.0);

  // Right room (x in [14.0, max_x], y in [margin, 6.5]): doorway gap x in [18.2, 19.8]
  rasterize_segment(g, 14.0, 6.5, 18.2, 6.5);
  rasterize_segment(g, 19.8, 6.5, max_x, 6.5);
  // Identical L-pillar in Right room (+14.0m x translation)
  rasterize_segment(g, 16.5, 2.5, 18.0, 2.5);
  rasterize_segment(g, 16.5, 2.5, 16.5, 4.0);

  // Symmetry-breaking features in top corridor y in [6.5, max_y]
  box(g, 8.2, 8.0, 9.8, 9.5);
  rasterize_segment(g, 15.2, 7.8, 15.2, max_y);

  return g;
}

OccupancyGrid make_open_corridor(double resolution)
{
  OccupancyGrid g;
  g.resolution = resolution;
  g.width = static_cast<int>(std::lround(28.0 / resolution));
  g.height = static_cast<int>(std::lround(5.0 / resolution));
  g.origin_x = -20.0;
  g.origin_y = -2.5;
  g.cells.assign(
    static_cast<std::size_t>(g.width) * static_cast<std::size_t>(g.height), -1);

  // Carve free corridor (0) for world coordinates wx in [-18.0, +2.0], wy in [-1.1, +1.1]
  // and asymmetric North alcove (0) for wx in [-1.2, +0.2], wy in [+1.1, +2.0]
  for (int y = 0; y < g.height; ++y) {
    for (int x = 0; x < g.width; ++x) {
      const auto w = g.gridToWorld(x, y);
      const double wx = w.x();
      const double wy = w.y();
      const bool main_corridor = (wx >= -18.0 && wx <= 2.0 && wy >= -1.1 && wy <= 1.1);
      const bool north_alcove = (wx >= -1.2 && wx <= 0.2 && wy > 1.1 && wy <= 2.0);
      if (main_corridor || north_alcove) {
        g.cells[static_cast<std::size_t>(y) * static_cast<std::size_t>(g.width) +
          static_cast<std::size_t>(x)] = 0;
      }
    }
  }

  // Rasterize solid walls (100)
  rasterize_segment(g, -18.0, -1.15, 2.05, -1.15);   // South wall
  rasterize_segment(g, -18.0, 1.15, -1.2, 1.15);     // North wall west part
  rasterize_segment(g, 0.2, 1.15, 2.05, 1.15);       // North wall east part
  rasterize_segment(g, -1.2, 1.15, -1.2, 2.05);      // Alcove west wall
  rasterize_segment(g, -1.2, 2.05, 0.2, 2.05);       // Alcove north wall
  rasterize_segment(g, 0.2, 2.05, 0.2, 1.15);        // Alcove east wall
  rasterize_segment(g, 2.05, -1.15, 2.05, 1.15);     // East end-wall
  rasterize_segment(g, -18.05, -1.15, -18.05, 1.15);  // West back wall

  return g;
}

OccupancyGrid make_cluttered_corridor(double resolution, bool include_static_ghosts)
{
  OccupancyGrid g;
  g.resolution = resolution;
  g.width = static_cast<int>(std::lround(30.0 / resolution));
  g.height = static_cast<int>(std::lround(4.0 / resolution));
  g.origin_x = 0.0;
  g.origin_y = 0.0;
  g.cells.assign(
    static_cast<std::size_t>(g.width) * static_cast<std::size_t>(g.height), -1);

  // Free main corridor (0) for wx in [0.2, 29.8], wy in [0.8, 3.2]
  // plus 3 asymmetric recess alcoves along North/South corridor walls
  for (int y = 0; y < g.height; ++y) {
    for (int x = 0; x < g.width; ++x) {
      const auto w = g.gridToWorld(x, y);
      const double wx = w.x();
      const double wy = w.y();
      const bool main_c = (wx >= 0.2 && wx <= 29.8 && wy >= 0.8 && wy <= 3.2);
      const bool alc1 = (wx >= 6.0 && wx <= 7.2 && wy > 3.2 && wy <= 3.8);
      const bool alc2 = (wx >= 14.5 && wx <= 15.8 && wy >= 0.2 && wy < 0.8);
      const bool alc3 = (wx >= 22.0 && wx <= 23.5 && wy > 3.2 && wy <= 3.8);
      if (main_c || alc1 || alc2 || alc3) {
        g.cells[static_cast<std::size_t>(y) * static_cast<std::size_t>(g.width) +
          static_cast<std::size_t>(x)] = 0;
      }
    }
  }

  // Rasterize continuous boundary walls around corridor and all 3 alcoves
  rasterize_segment(g, 0.2, 0.8, 14.5, 0.8);
  rasterize_segment(g, 15.8, 0.8, 29.8, 0.8);  // South wall sections (gap only for alcove 2)

  rasterize_segment(g, 0.2, 3.2, 6.0, 3.2);
  rasterize_segment(g, 7.2, 3.2, 22.0, 3.2);
  rasterize_segment(g, 23.5, 3.2, 29.8, 3.2);  // North wall sections (gaps for alcoves 1 & 3)

  // Alcove 1 walls
  rasterize_segment(g, 6.0, 3.2, 6.0, 3.8);
  rasterize_segment(g, 6.0, 3.8, 7.2, 3.8);
  rasterize_segment(g, 7.2, 3.8, 7.2, 3.2);

  // Alcove 2 walls
  rasterize_segment(g, 14.5, 0.8, 14.5, 0.2);
  rasterize_segment(g, 14.5, 0.2, 15.8, 0.2);
  rasterize_segment(g, 15.8, 0.2, 15.8, 0.8);

  // Alcove 3 walls
  rasterize_segment(g, 22.0, 3.2, 22.0, 3.8);
  rasterize_segment(g, 22.0, 3.8, 23.5, 3.8);
  rasterize_segment(g, 23.5, 3.8, 23.5, 3.2);

  // End walls
  rasterize_segment(g, 0.2, 0.8, 0.2, 3.2);
  rasterize_segment(g, 29.8, 0.8, 29.8, 3.2);

  if (include_static_ghosts) {
    const double centers[][2] = {
      {5.0, 1.5}, {11.0, 2.5}, {18.0, 1.5}, {24.5, 2.4}
    };
    for (const auto & c : centers) {
      const double cx = c[0];
      const double cy = c[1];
      box(g, cx - 0.175, cy - 0.175, cx + 0.175, cy + 0.175);
      // Fill box interior
      for (int y = 0; y < g.height; ++y) {
        for (int x = 0; x < g.width; ++x) {
          const auto w = g.gridToWorld(x, y);
          const double wx = w.x();
          const double wy = w.y();
          if (wx >= cx - 0.175 && wx <= cx + 0.175 && wy >= cy - 0.175 && wy <= cy + 0.175) {
            g.cells[static_cast<std::size_t>(y) * static_cast<std::size_t>(g.width) +
              static_cast<std::size_t>(x)] = 100;
          }
        }
      }
    }
  }

  return g;
}

OccupancyGrid make_open_hall(double resolution)
{
  OccupancyGrid g;
  g.resolution = resolution;
  g.width = static_cast<int>(std::lround(36.0 / resolution));
  g.height = static_cast<int>(std::lround(28.0 / resolution));
  g.origin_x = 0.0;
  g.origin_y = 0.0;
  g.cells.assign(
    static_cast<std::size_t>(g.width) * static_cast<std::size_t>(g.height), 0);

  const double margin = 1.5 * resolution;
  const double max_x = 36.0 - margin;
  const double max_y = 28.0 - margin;
  box(g, margin, margin, max_x, max_y);
  mark_exterior_unknown(g, margin, margin, max_x, max_y);

  // Sparse interior pillars (0.6m x 0.6m boxes)
  const double pillar_centers[][2] = {
    {9.0, 8.0}, {18.0, 7.5}, {27.0, 9.0}, {13.5, 19.0}, {23.5, 20.0}
  };
  for (const auto & pc : pillar_centers) {
    const double cx = pc[0];
    const double cy = pc[1];
    box(g, cx - 0.3, cy - 0.3, cx + 0.3, cy + 0.3);
  }

  // L-shaped reception counter
  rasterize_segment(g, 5.0, 14.0, 7.5, 14.0);
  rasterize_segment(g, 7.5, 14.0, 7.5, 16.5);

  return g;
}

double raycast_range(
  const OccupancyGrid & grid, double px, double py, double world_angle,
  double min_range, double range_max,
  const std::vector<DynBox> & dyn)
{
  if (grid.width <= 0 || grid.height <= 0 || grid.resolution <= 0.0 ||
    range_max <= min_range)
  {
    return -1.0;
  }
  const double dt = grid.resolution / 4.0;
  const double dx = std::cos(world_angle);
  const double dy = std::sin(world_angle);
  auto occ_at = [&](double x, double y) {
      if (in_dyn(dyn, x, y)) {return true;}
      const auto idx = grid.worldToGridIndex(x, y);
      if (!grid.inside(idx.x(), idx.y())) {return false;}
      return grid.occupied(idx.x(), idx.y());
    };
  double t = min_range;
  while (t < range_max) {
    const double sx = px + dx * t;
    const double sy = py + dy * t;
    const auto idx = grid.worldToGridIndex(sx, sy);
    if (!grid.inside(idx.x(), idx.y())) {
      break;
    }
    if (occ_at(sx, sy)) {
      // 二分细化到亚毫米（消除步长量化偏差）
      double lo = std::max(min_range, t - dt);
      double hi = t;
      for (int bi = 0; bi < 8; ++bi) {
        const double mid = 0.5 * (lo + hi);
        if (occ_at(px + dx * mid, py + dy * mid)) {
          hi = mid;
        } else {
          lo = mid;
        }
      }
      return hi;
    }
    t += dt;
  }
  return -1.0;
}

Scan make_scan(
  const OccupancyGrid & grid, const Pose2 & pose,
  const ScanSimConfig & cfg, std::mt19937 & rng,
  const std::vector<DynBox> & dyn)
{
  Scan out;
  if (cfg.beams <= 0) {return out;}
  const bool add_noise = cfg.noise_std > 0.0;
  std::normal_distribution<double> noise(0.0, add_noise ? cfg.noise_std : 1.0);
  const int step = std::max(1, cfg.subsample);
  for (int i = 0; i < cfg.beams; i += step) {
    const double local_a = -0.5 * cfg.fov + cfg.fov * (i + 0.5) / cfg.beams;
    const double hit = raycast_range(
      grid, pose.x, pose.y, pose.theta + local_a,
      cfg.min_range, cfg.range_max, dyn);
    if (hit < 0.0) {continue;}
    const double r = add_noise ? (hit + noise(rng)) : hit;
    if (r < cfg.min_range || r > cfg.range_max) {continue;}
    out.points.emplace_back(r * std::cos(local_a), r * std::sin(local_a));
  }
  return out;
}

std::vector<float> make_laser_ranges(
  const OccupancyGrid & grid, const Pose2 & pose,
  double angle_min, double angle_increment, int beams,
  double min_range, double range_max, double noise_std,
  std::mt19937 & rng,
  const std::vector<DynBox> & dyn)
{
  std::vector<float> ranges;
  if (beams <= 0) {return ranges;}
  ranges.assign(
    static_cast<std::size_t>(beams), std::numeric_limits<float>::infinity());
  const bool add_noise = noise_std > 0.0;
  std::normal_distribution<double> noise(0.0, add_noise ? noise_std : 1.0);
  for (int i = 0; i < beams; ++i) {
    const double local_a = angle_min + static_cast<double>(i) * angle_increment;
    const double hit = raycast_range(
      grid, pose.x, pose.y, pose.theta + local_a, min_range, range_max, dyn);
    if (hit < 0.0) {continue;}
    const double r = add_noise ? (hit + noise(rng)) : hit;
    if (r < min_range || r > range_max) {continue;}
    ranges[static_cast<std::size_t>(i)] = static_cast<float>(r);
  }
  return ranges;
}

std::vector<Pose2> random_free_poses(
  const OccupancyGrid & grid, int n,
  std::mt19937 & rng,
  double clearance_m,
  int max_attempts)
{
  std::vector<Pose2> out;
  if (n <= 0 || max_attempts <= 0 || clearance_m < 0.0 ||
    grid.width <= 0 || grid.height <= 0 || grid.resolution <= 0.0)
  {
    return out;
  }
  const double min_x = grid.origin_x + clearance_m;
  const double max_x =
    grid.origin_x + static_cast<double>(grid.width) * grid.resolution -
    clearance_m;
  const double min_y = grid.origin_y + clearance_m;
  const double max_y =
    grid.origin_y + static_cast<double>(grid.height) * grid.resolution -
    clearance_m;
  if (min_x >= max_x || min_y >= max_y) {
    return out;
  }

  std::uniform_real_distribution<double> ux(min_x, max_x);
  std::uniform_real_distribution<double> uy(min_y, max_y);
  std::uniform_real_distribution<double> ut(-kPi, kPi);
  const int r_cells = static_cast<int>(std::ceil(clearance_m / grid.resolution));
  out.reserve(static_cast<std::size_t>(n));

  for (int attempt = 0;
    attempt < max_attempts && static_cast<int>(out.size()) < n;
    ++attempt)
  {
    const double x = ux(rng);
    const double y = uy(rng);
    const auto g = grid.worldToGridIndex(x, y);
    const int gx = g.x();
    const int gy = g.y();
    bool ok = true;
    for (int dy = -r_cells; dy <= r_cells && ok; ++dy) {
      for (int dx = -r_cells; dx <= r_cells; ++dx) {
        if (dx * dx + dy * dy > r_cells * r_cells) {continue;}
        const int ix = gx + dx;
        const int iy = gy + dy;
        if (!grid.inside(ix, iy) ||
          grid.cells[static_cast<std::size_t>(iy) *
          static_cast<std::size_t>(grid.width) +
          static_cast<std::size_t>(ix)] != 0)
        {
          ok = false;  // 占据或未知都不算自由
          break;
        }
      }
    }
    if (ok) {
      out.emplace_back(x, y, ut(rng));
    }
  }
  return out;
}

std::vector<Pose2> make_warehouse_trajectory(int laps, double step_m)
{
  std::vector<Pose2> traj;
  if (laps <= 0 || step_m <= 0.0) {return traj;}
  struct Waypoint { double x, y; };
  const Waypoint loop[] = {
    {2.0, 13.5}, {13.5, 13.5}, {13.5, 28.0}, {2.0, 28.0}, {2.0, 13.5}};
  Pose2 cur(loop[0].x, loop[0].y, 0.0);
  traj.push_back(cur);
  for (int lap = 0; lap < laps; ++lap) {
    for (std::size_t seg = 0; seg < 4; ++seg) {
      const double dx = loop[seg + 1].x - loop[seg].x;
      const double dy = loop[seg + 1].y - loop[seg].y;
      const double len = std::hypot(dx, dy);
      const int n = std::max(1, static_cast<int>(std::lround(len / step_m)));
      const double heading = std::atan2(dy, dx);
      for (int i = 1; i <= n; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(n);
        cur.x = loop[seg].x + dx * t;
        cur.y = loop[seg].y + dy * t;
        cur.theta = heading;
        traj.push_back(cur);
      }
    }
  }
  return traj;
}

}  // namespace sloc::sim
