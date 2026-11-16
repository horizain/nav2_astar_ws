#pragma once
// ============================================================================
// 地图渲染：位姿图 → log-odds 栅格 → nav 风格 OccupancyMap (-1/0/100)。
// 每条射线：沿途格记 free（miss），端点格记 occupied（hit）——
// 与 gmapping/slam_toolbox 的地图栅格更新同思路。
// ============================================================================
#include <cstddef>
#include <cstdint>
#include <vector>

#include "mini_slam/occupancy_grid.hpp"
#include "mini_slam/pose_graph.hpp"

namespace mini_slam {

struct OccupancyMap {
  double resolution{0.05};
  Vec2 origin;
  int width{0}, height{0};
  std::vector<std::int8_t> data;  // -1 未知, 0 空闲, 100 占据
};

class MapBuilder {
 public:
  // 全量渲染图中所有扫描（用各自当前优化位姿）
  static OccupancyMap render(const PoseGraph& graph, double resolution,
                             double range_max, double margin);
};

// 增量地图渲染器（Phase 4B）：
//   顺序建图时只对新增顶点做射线更新；地图几何（origin/width/height）一旦
//   需要变化就退回全量重建，避免任何"格子搬移"带来的重采样误差。
//   对外保证：对同一张图，update(...) 与 MapBuilder::render(...) 逐位一致。
class IncrementalMapBuilder {
 public:
  // 返回内部地图引用。force_full 或任何失效条件触发全量重建。
  const OccupancyMap& update(const PoseGraph& graph, double resolution,
                             double range_max, double margin,
                             bool force_full = false);
  void reset();

 private:
  OccupancyMap map_;
  Grid grid_;
  size_t applied_vertices_{0};
  std::uint64_t pose_revision_{0};
  double resolution_{0.0}, range_max_{0.0}, margin_{0.0};
  bool valid_{false};
};

}  // namespace mini_slam
