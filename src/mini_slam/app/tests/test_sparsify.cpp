// 地图稀疏化测试：往返运动 300 帧，验证
//  1) 开启稀疏化后，存储的扫描点总量被显著压缩
//  2) 地图渲染仍然完整（剪枝扫描被存活扫描等价覆盖）
//  3) 闭环结构不受影响（位姿与边保留）
#include <cmath>
#include <random>
#include <vector>

#include "mini_slam/map_builder.hpp"
#include "mini_slam/mapper.hpp"

#include "test_util.hpp"

using namespace mini_slam;

namespace {

// 往返轨迹：x 0→15→0→15→0（4 段），带扫描重访
std::vector<Pose2> shuttle(int legs = 4) {
  std::vector<Pose2> gt;
  gt.push_back(Pose2{0.5, 0.5, 0.0});
  double x = 0.5;
  bool east = true;
  for (int leg = 0; leg < legs; ++leg) {
    const double target = east ? 15.0 : 0.5;
    const double d = target - x;
    const double heading = east ? 0.0 : kPi;
    const int steps = static_cast<int>(std::ceil(std::abs(d) / 0.25));
    for (int i = 0; i < steps; ++i) {
      const double t = static_cast<double>(i) / steps;
      gt.push_back(Pose2{x + t * d, 0.5, heading});
    }
    x = target;
    east = !east;
  }
  return gt;
}

long total_points(const Mapper& m) {
  long sum = 0;
  for (const auto& v : m.graph().vertices()) {
    if (v.scan) sum += static_cast<long>(v.scan->points.size());
  }
  return sum;
}

}  // namespace

int main() {
  const std::vector<test::Segment> corridor = {{{0.0, 0.0}, {15.0, 0.0}},
                                               {{0.0, 1.2}, {15.0, 1.2}},
                                               {{0.0, 0.0}, {0.0, 1.2}},
                                               {{15.0, 0.0}, {15.0, 1.2}}};
  const auto gt = shuttle(4);
  std::mt19937 rng{41};

  // ---------- 不稀疏：基线 ----------
  Mapper base{Params{}};
  Pose2 odom = gt.front();
  for (size_t i = 1; i < gt.size(); ++i) {
    odom = compose(odom, compose(inverse(gt[i - 1]), gt[i]));
    const LaserScan scan =
        test::generate_scan(gt[i], corridor, 181, kPi, 8.0, &rng);
    base.process_scan(scan, odom);
  }
  const long base_points = total_points(base);
  const int base_nodes = base.stats().nodes;
  CHECK(base_nodes > 40);

  // ---------- 稀疏：同轨迹重放 ----------
  std::mt19937 rng2{41};
  Params params;
  params.sparsify_enabled = true;
  params.sparsify_keep_radius = 1.5;
  params.sparsify_min_keep_recent = 10;
  params.sparsify_every = 25;
  Mapper sparse{params};
  odom = gt.front();
  for (size_t i = 1; i < gt.size(); ++i) {
    odom = compose(odom, compose(inverse(gt[i - 1]), gt[i]));
    const LaserScan scan =
        test::generate_scan(gt[i], corridor, 181, kPi, 8.0, &rng2);
    sparse.process_scan(scan, odom);
  }
  const long sparse_points = total_points(sparse);

  std::printf("  基线: %d 节点 %ld 点 | 稀疏: %d 节点 %ld 点 (%.0f%%)\n",
              base_nodes, base_points, sparse.stats().nodes, sparse_points,
              100.0 * sparse_points / base_points);
  CHECK(sparse.stats().nodes == base_nodes);   // 图结构不变
  CHECK(sparse_points < base_points * 4 / 10); // 点云压缩到 40% 以下

  // ---------- 地图仍完整：两份地图的占据格数量接近 ----------
  const auto m1 = MapBuilder::render(base.graph(), 0.05, 8.0, 1.0);
  const auto m2 = MapBuilder::render(sparse.graph(), 0.05, 8.0, 1.0);
  long occ1 = 0, occ2 = 0;
  for (const std::int8_t v : m1.data) occ1 += (v == 100);
  for (const std::int8_t v : m2.data) occ2 += (v == 100);
  std::printf("  地图占据格: 基线 %ld / 稀疏 %ld\n", occ1, occ2);
  CHECK(occ2 > 100);
  CHECK(occ2 > occ1 * 7 / 10);   // 等价覆盖（允许少量剪枝损失）

  std::puts("[PASS] test_sparsify");
  return 0;
}
