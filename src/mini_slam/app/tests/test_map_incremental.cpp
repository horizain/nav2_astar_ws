// Phase 4B 增量地图渲染回归测试：
//   逐帧建图 / 边界扩张 / 位姿变化 / 稀疏化 / 参数变化 / reset
// 每个场景都要求 IncrementalMapBuilder::update 与 MapBuilder::render 逐位相等。
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include "mini_slam/map_builder.hpp"
#include "mini_slam/pose_graph.hpp"

#include "test_util.hpp"

using namespace mini_slam;

namespace {

bool maps_equal(const OccupancyMap& a, const OccupancyMap& b) {
  return a.resolution == b.resolution && a.origin.x == b.origin.x &&
         a.origin.y == b.origin.y && a.width == b.width &&
         a.height == b.height && a.data == b.data;
}

// 合成一圈扫描点（传感器系），半径可变。
LaserScan ring(double radius, int n = 72) {
  LaserScan scan;
  scan.points.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    const double ang = 2.0 * kPi * static_cast<double>(i) /
                       static_cast<double>(n);
    scan.points.push_back(Vec2{radius * std::cos(ang), radius * std::sin(ang)});
  }
  return scan;
}

void add_frame(PoseGraph& graph, Pose2 pose, double radius) {
  auto scan = std::make_shared<const LaserScan>(ring(radius));
  graph.add_vertex(pose, pose, scan);
}

bool has_free_and_occupied(const OccupancyMap& m) {
  bool free_cell = false, occ_cell = false;
  for (const std::int8_t v : m.data) {
    if (v == 0) free_cell = true;
    if (v == 100) occ_cell = true;
  }
  return free_cell && occ_cell;
}

// 1) 顺序建图：后续帧都落在首帧包围盒内，走增量射线更新路径。
void test_sequential() {
  PoseGraph graph;
  IncrementalMapBuilder inc;
  const double res = 0.05, range_max = 10.0, margin = 1.0;

  add_frame(graph, Pose2{0.0, 0.0, 0.0}, 3.0);
  CHECK(maps_equal(inc.update(graph, res, range_max, margin),
                   MapBuilder::render(graph, res, range_max, margin)));

  for (int i = 1; i <= 6; ++i) {
    const Pose2 p{0.18 * i, 0.09 * i, 0.05 * i};
    add_frame(graph, p, 1.5);
    const OccupancyMap& got = inc.update(graph, res, range_max, margin);
    CHECK(maps_equal(got, MapBuilder::render(graph, res, range_max, margin)));
  }
  CHECK(has_free_and_occupied(inc.update(graph, res, range_max, margin)));
  std::puts("  [ok] 顺序建图增量路径与全量一致");
}

// 2) 边界扩张：新节点远离当前地图 → 内部必须全量重建。
void test_boundary_expansion() {
  PoseGraph graph;
  IncrementalMapBuilder inc;
  const double res = 0.05, range_max = 10.0, margin = 1.0;

  add_frame(graph, Pose2{0.0, 0.0, 0.0}, 2.0);
  add_frame(graph, Pose2{0.5, 0.2, 0.0}, 1.0);
  CHECK(maps_equal(inc.update(graph, res, range_max, margin),
                   MapBuilder::render(graph, res, range_max, margin)));

  add_frame(graph, Pose2{20.0, 0.0, 0.0}, 1.0);
  const OccupancyMap& got = inc.update(graph, res, range_max, margin);
  CHECK(maps_equal(got, MapBuilder::render(graph, res, range_max, margin)));
  CHECK(got.width > 0 && got.height > 0);
  std::puts("  [ok] 边界扩张触发全量重建且与全量一致");
}

// 3) 历史位姿被修改：调用方 bump_pose_revision → 全量重建。
void test_pose_revision() {
  PoseGraph graph;
  IncrementalMapBuilder inc;
  const double res = 0.05, range_max = 10.0, margin = 1.0;

  add_frame(graph, Pose2{0.0, 0.0, 0.0}, 2.0);
  add_frame(graph, Pose2{0.3, 0.1, 0.1}, 1.2);
  inc.update(graph, res, range_max, margin);

  graph.pose(1) = Pose2{1.4, -0.8, 0.6};
  graph.bump_pose_revision();
  const OccupancyMap& got = inc.update(graph, res, range_max, margin);
  CHECK(maps_equal(got, MapBuilder::render(graph, res, range_max, margin)));
  std::puts("  [ok] 位姿版本变化触发全量重建且与全量一致");
}

// 4) 稀疏化：对旧节点 set_scan(空扫描)，set_scan 自动 bump 版本。
void test_sparsify() {
  PoseGraph graph;
  IncrementalMapBuilder inc;
  const double res = 0.05, range_max = 10.0, margin = 1.0;

  add_frame(graph, Pose2{0.0, 0.0, 0.0}, 2.0);
  add_frame(graph, Pose2{0.4, 0.0, 0.0}, 2.0);
  add_frame(graph, Pose2{0.8, 0.0, 0.0}, 2.0);
  inc.update(graph, res, range_max, margin);

  graph.set_scan(1, std::make_shared<const LaserScan>(LaserScan{}));
  const OccupancyMap& got = inc.update(graph, res, range_max, margin);
  CHECK(maps_equal(got, MapBuilder::render(graph, res, range_max, margin)));
  std::puts("  [ok] 稀疏化（set_scan 自动 bump）后与全量一致");
}

// 5) 渲染参数变化：resolution / margin 改变 → 全量重建。
void test_parameter_change() {
  PoseGraph graph;
  IncrementalMapBuilder inc;
  const double range_max = 10.0;

  add_frame(graph, Pose2{0.0, 0.0, 0.0}, 2.0);
  add_frame(graph, Pose2{0.5, 0.3, 0.0}, 1.2);
  inc.update(graph, 0.05, range_max, 1.0);

  const OccupancyMap& coarse = inc.update(graph, 0.10, range_max, 1.0);
  CHECK(maps_equal(coarse, MapBuilder::render(graph, 0.10, range_max, 1.0)));

  const OccupancyMap& big_margin = inc.update(graph, 0.10, range_max, 2.0);
  CHECK(maps_equal(big_margin,
                   MapBuilder::render(graph, 0.10, range_max, 2.0)));
  std::puts("  [ok] 参数变化触发全量重建且与全量一致");
}

// 6) reset 后重新 update：全量重建。
void test_reset() {
  PoseGraph graph;
  IncrementalMapBuilder inc;
  const double res = 0.05, range_max = 10.0, margin = 1.0;

  add_frame(graph, Pose2{0.0, 0.0, 0.0}, 2.0);
  add_frame(graph, Pose2{0.3, 0.2, 0.0}, 1.0);
  inc.update(graph, res, range_max, margin);
  inc.reset();
  const OccupancyMap& got = inc.update(graph, res, range_max, margin);
  CHECK(maps_equal(got, MapBuilder::render(graph, res, range_max, margin)));
  std::puts("  [ok] reset 后重建成与全量一致");
}

}  // namespace

int main() {
  test_sequential();
  test_boundary_expansion();
  test_pose_revision();
  test_sparsify();
  test_parameter_change();
  test_reset();
  std::puts("[PASS] test_map_incremental");
  return 0;
}
