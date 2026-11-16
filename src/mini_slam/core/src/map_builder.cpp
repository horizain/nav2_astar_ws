#include "mini_slam/map_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "mini_slam/occupancy_grid.hpp"

namespace mini_slam {
namespace {

constexpr float kMiss = -0.7f;  // 一次 free 观测的 log-odds
constexpr float kHit = 0.9f;    // 一次 hit 观测的 log-odds
constexpr float kClamp = 4.0f;

// 顶点区间 [begin,end) 的包围盒（传感器位置 + 端点）。
// 只统计有 scan 的节点；跳过 norm(p) > range_max 的点。与历史实现逐位一致。
struct Bounds {
  bool any{false};
  double min_x{std::numeric_limits<double>::max()};
  double min_y{std::numeric_limits<double>::max()};
  double max_x{std::numeric_limits<double>::lowest()};
  double max_y{std::numeric_limits<double>::lowest()};
};

Bounds vertex_range_bounds(const PoseGraph& graph, size_t begin, size_t end,
                           double range_max) {
  Bounds b;
  const auto& verts = graph.vertices();
  end = std::min(end, verts.size());
  begin = std::min(begin, end);
  for (size_t i = begin; i < end; ++i) {
    const Vertex& v = verts[i];
    if (!v.scan) continue;
    const Vec2 sensor{v.pose.x, v.pose.y};
    for (const Vec2& p : v.scan->points) {
      const Vec2 w = transform(v.pose, p);
      if (norm(p) > range_max) continue;
      b.any = true;
      b.min_x = std::min({b.min_x, w.x, sensor.x});
      b.min_y = std::min({b.min_y, w.y, sensor.y});
      b.max_x = std::max({b.max_x, w.x, sensor.x});
      b.max_y = std::max({b.max_y, w.y, sensor.y});
    }
  }
  return b;
}

struct MapGeometry {
  Vec2 origin;
  int width{0};
  int height{0};
};

// 由包围盒推出地图几何。返回 false 表示没有任何有效点（空图）。
bool compute_geometry(const Bounds& b, double resolution, double margin,
                      MapGeometry& g) {
  if (!b.any) return false;
  const int pad = std::max(1, static_cast<int>(std::ceil(margin / resolution)));
  g.width = std::max(
      1, static_cast<int>(std::ceil((b.max_x - b.min_x) / resolution)) +
             2 * pad + 1);
  g.height = std::max(
      1, static_cast<int>(std::ceil((b.max_y - b.min_y) / resolution)) +
             2 * pad + 1);
  g.origin = Vec2{b.min_x - margin, b.min_y - margin};
  return true;
}

// 对顶点区间 [begin,end) 做射线更新：沿途 free（miss），端点 hit。
void ray_update(const PoseGraph& graph, size_t begin, size_t end,
                double range_max, Grid& grid) {
  const auto& verts = graph.vertices();
  end = std::min(end, verts.size());
  begin = std::min(begin, end);
  const double resolution = grid.resolution();
  for (size_t vi = begin; vi < end; ++vi) {
    const Vertex& v = verts[vi];
    if (!v.scan) continue;
    const Vec2 sensor{v.pose.x, v.pose.y};
    for (const Vec2& p : v.scan->points) {
      if (norm(p) > range_max) continue;
      const Vec2 endp = transform(v.pose, p);
      const double dist = norm(Vec2{endp.x - sensor.x, endp.y - sensor.y});
      if (dist < 1e-6) continue;
      const int steps =
          std::max(1, static_cast<int>(std::ceil(dist / resolution)));
      // 沿途 free（不含端点格）
      for (int i = 0; i < steps - 1; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const Vec2 mid = sensor + t * (endp - sensor);  // 线性插值
        int cx = 0, cy = 0;
        if (grid.world_to_cell(mid, cx, cy)) {
          grid.add(cx, cy, kMiss);
        }
      }
      // 端点 hit
      int cx = 0, cy = 0;
      if (grid.world_to_cell(endp, cx, cy)) {
        grid.add(cx, cy, kHit);
      }
    }
  }
}

// log-odds → -1/0/100（保持历史 kClamp 与阈值规则）
void refresh_map_data(const Grid& grid, OccupancyMap& out) {
  out.resolution = grid.resolution();
  out.origin = grid.origin();
  out.width = grid.width();
  out.height = grid.height();
  out.data.assign(static_cast<size_t>(out.width) * out.height,
                  std::int8_t{-1});
  const auto& cells = grid.cells();
  for (size_t i = 0; i < cells.size(); ++i) {
    const float lo = std::clamp(cells[i], -kClamp, kClamp);
    if (lo > 0.5f) {
      out.data[i] = 100;
    } else if (lo < -0.5f) {
      out.data[i] = 0;
    }
  }
}

}  // namespace

OccupancyMap MapBuilder::render(const PoseGraph& graph, double resolution,
                                double range_max, double margin) {
  OccupancyMap out;
  out.resolution = resolution;
  // 防御性校验：非法分辨率会让 ceil(margin/res) 除零/产生无效尺寸。
  if (!std::isfinite(resolution) || resolution <= 0.0 ||
      !std::isfinite(margin) || margin < 0.0 || range_max <= 0.0) {
    return out;
  }

  // ---------- 1) 包围盒（传感器位置 + 端点）----------
  const Bounds bounds =
      vertex_range_bounds(graph, 0, graph.vertices().size(), range_max);
  MapGeometry geom;
  if (!compute_geometry(bounds, resolution, margin, geom)) {
    return out;  // 空图
  }

  Grid logodds(resolution, geom.origin, geom.width, geom.height);

  // ---------- 2) 射线更新 ----------
  ray_update(graph, 0, graph.vertices().size(), range_max, logodds);

  // ---------- 3) log-odds → -1/0/100 ----------
  refresh_map_data(logodds, out);
  return out;
}

const OccupancyMap& IncrementalMapBuilder::update(const PoseGraph& graph,
                                                  double resolution,
                                                  double range_max,
                                                  double margin,
                                                  bool force_full) {
  // 参数非法：清空状态，返回空图。
  if (!std::isfinite(resolution) || resolution <= 0.0 ||
      !std::isfinite(range_max) || range_max <= 0.0 ||
      !std::isfinite(margin) || margin < 0.0) {
    reset();
    return map_;
  }

  const size_t graph_size = static_cast<size_t>(graph.size());
  const bool params_changed = resolution != resolution_ ||
                              range_max != range_max_ || margin != margin_;
  bool full = force_full || !valid_ || params_changed ||
              graph.pose_revision() != pose_revision_ ||
              graph_size < applied_vertices_;

  if (!full) {
    if (graph_size == applied_vertices_) {
      return map_;  // 图没变
    }

    // 新增顶点没有有效扫描点：几何与内容都不变，只推进游标。
    const Bounds added =
        vertex_range_bounds(graph, applied_vertices_, graph_size, range_max);
    if (!added.any) {
      applied_vertices_ = graph_size;
      pose_revision_ = graph.pose_revision();
      return map_;
    }

    // 用全图包围盒复算几何。只有 origin/width/height 完全不变时才允许
    // 增量射线更新——任何边界扩张都退回全量，避免格子搬移的重采样误差。
    const Bounds all = vertex_range_bounds(graph, 0, graph_size, range_max);
    MapGeometry geom;
    const bool have_geom = compute_geometry(all, resolution, margin, geom);
    const bool same_geometry =
        have_geom && geom.origin.x == grid_.origin().x &&
        geom.origin.y == grid_.origin().y && geom.width == grid_.width() &&
        geom.height == grid_.height();

    if (same_geometry) {
      ray_update(graph, applied_vertices_, graph_size, range_max, grid_);
      // 只重算阈值转换（O(地图格数)），历史射线结果直接复用。
      refresh_map_data(grid_, map_);
      applied_vertices_ = graph_size;
      pose_revision_ = graph.pose_revision();
      resolution_ = resolution;
      range_max_ = range_max;
      margin_ = margin;
      return map_;
    }
    full = true;
  }

  // ---------- 全量重建 ----------
  const Bounds all = vertex_range_bounds(graph, 0, graph_size, range_max);
  MapGeometry geom;
  if (!compute_geometry(all, resolution, margin, geom)) {
    grid_ = Grid{};
    map_ = OccupancyMap{};
    map_.resolution = resolution;
  } else {
    grid_ = Grid(resolution, geom.origin, geom.width, geom.height);
    ray_update(graph, 0, graph_size, range_max, grid_);
    refresh_map_data(grid_, map_);
  }
  applied_vertices_ = graph_size;
  pose_revision_ = graph.pose_revision();
  resolution_ = resolution;
  range_max_ = range_max;
  margin_ = margin;
  valid_ = true;
  return map_;
}

void IncrementalMapBuilder::reset() {
  map_ = OccupancyMap{};
  grid_ = Grid{};
  applied_vertices_ = 0;
  pose_revision_ = 0;
  resolution_ = 0.0;
  range_max_ = 0.0;
  margin_ = 0.0;
  valid_ = false;
}

}  // namespace mini_slam
