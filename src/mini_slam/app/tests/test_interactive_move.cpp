// 交互式编辑（Phase 4C）：固定被拖节点做 SPA + Mapper::move_vertex。
//   1) SpaSolver::solve(g, fixed_id)：被拖节点位姿必须保持（< 1e-9），
//      其它节点按边观测让步；不传 fixed_id 时行为与 Phase 1-4B 一致。
//   2) Mapper::move_vertex：位姿写入、pose_revision/solves 递增、
//      空间索引按新位姿重建；非法 id / 非有限位姿返回 false 且不改图。
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "mini_slam/mapper.hpp"
#include "mini_slam/pose_graph.hpp"
#include "mini_slam/spa_solver.hpp"

#include "test_util.hpp"

using namespace mini_slam;

namespace {

// 手工三节点链：0 → 1 → 2，观测取自一条"真值"轨迹。
// 节点 1 的初值故意放在用户拖到的位置（与观测不一致），用来验证 fixed_id
// 真的把它钉在原地。
void test_solver_fixed_id() {
  const Pose2 p0{0.0, 0.0, 0.0};
  const Pose2 p1{1.0, 0.10, 0.20};
  const Pose2 p2{2.0, 0.50, 0.40};
  const Pose2 z01 = delta_pose(p0, p1);
  const Pose2 z12 = delta_pose(p1, p2);
  const Pose2 dragged{1.30, 0.25, 0.35};  // 用户放下的位置

  PoseGraph g;
  g.add_vertex(p0, p0, nullptr);
  g.add_vertex(p1, dragged, nullptr);
  g.add_vertex(p2, Pose2{2.0, 0.0, 0.0}, nullptr);
  g.add_edge(0, 1, z01, 100.0, 100.0, 100.0, false);
  g.add_edge(1, 2, z12, 100.0, 100.0, 100.0, false);

  const SpaSolver solver;
  const SpaSummary s = solver.solve(g, 1);

  // 被拖节点保持不动（Ceres 的 constant block 不会被写回）
  CHECK(distance_xy(g.vertices()[1].pose, dragged) < 1e-9);
  CHECK(std::abs(shortest_angle_diff(g.vertices()[1].pose.theta,
                                     dragged.theta)) < 1e-9);
  // 节点 0 仍然固定（规范自由度）
  CHECK(distance_xy(g.vertices()[0].pose, p0) < 1e-9);
  // 节点 2 由"拖动后的节点1 ⊕ 相对观测"唯一决定
  const Pose2 expect2 = compose(dragged, z12);
  const double err2 = distance_xy(g.vertices()[2].pose, expect2);
  const double err2th =
      std::abs(shortest_angle_diff(g.vertices()[2].pose.theta, expect2.theta));
  std::printf("  fixed_id=1: err(node1)=%.3e err(node2)=%.3e ang=%.3e "
              "cost=%.4f→%.4f iters=%d\n",
              distance_xy(g.vertices()[1].pose, dragged), err2, err2th,
              s.initial_cost, s.final_cost, s.iterations);
  CHECK(err2 < 1e-6);
  CHECK(err2th < 1e-6);
  CHECK(std::isfinite(s.final_cost));
  CHECK(s.final_cost <= s.initial_cost + 1e-12);
  // 0→1 边的观测被拖动破坏，残差压力留在代价里（这正是手动纠偏的语义：
  // 用户说"这里就是对的"，邻居节点负责让步）
  CHECK(s.final_cost > 1e-6);

  // 对照：不传 fixed_id（Phase 1-4B 的调用方式）→ 只有节点 0 固定，
  // 节点 1 会被拉回观测一致的位置。默认行为不变。
  {
    PoseGraph g2;
    g2.add_vertex(p0, p0, nullptr);
    g2.add_vertex(p1, dragged, nullptr);
    g2.add_vertex(p2, Pose2{2.0, 0.0, 0.0}, nullptr);
    g2.add_edge(0, 1, z01, 100.0, 100.0, 100.0, false);
    g2.add_edge(1, 2, z12, 100.0, 100.0, 100.0, false);
    solver.solve(g2);
    const double err = distance_xy(g2.vertices()[1].pose, p1);
    std::printf("  默认 solve()（不固定节点1）: err(node1→观测位置)=%.3e\n",
                err);
    CHECK(err < 1e-6);
  }
}

// 用 Mapper 处理合成扫描生成 5 节点小图，再走 move_vertex。
void test_mapper_move_vertex() {
  const auto walls = test::box_world();
  const std::vector<Pose2> gt = {
      Pose2{1.5, 2.0, 0.0}, Pose2{2.1, 2.0, 0.0}, Pose2{2.7, 2.0, 0.0},
      Pose2{3.3, 2.0, 0.0}, Pose2{3.9, 2.0, 0.0}};

  Mapper mapper;
  for (const Pose2& p : gt) {
    mapper.process_scan(test::generate_scan(p, walls, 120, 2.0 * kPi, 8.0), p);
  }
  CHECK(mapper.graph().size() == 5);
  CHECK(mapper.mode() == Mapper::Mode::Mapping);

  const std::uint64_t rev0 = mapper.graph().pose_revision();
  const int solves0 = mapper.stats().solves;
  const Pose2 before = mapper.graph().vertices()[1].pose;

  const Pose2 target{1.2, -0.8, 0.7};
  CHECK(mapper.move_vertex(1, target));

  // 1) 位姿写入（theta 归一化到 (-π, π]，0.7 原样）
  CHECK(distance_xy(mapper.graph().vertices()[1].pose, target) < 1e-12);
  CHECK(std::abs(shortest_angle_diff(mapper.graph().vertices()[1].pose.theta,
                                     target.theta)) < 1e-12);
  // 2) 版本号与统计：地图缓存/增量渲染都靠这两个信号失效
  CHECK(mapper.graph().pose_revision() > rev0);
  CHECK(mapper.stats().solves == solves0 + 1);
  // 3) 空间索引已按新位姿重建
  const std::vector<int> hits =
      mapper.graph().vertices_within(Vec2{target.x, target.y}, 0.3);
  CHECK(std::find(hits.begin(), hits.end(), 1) != hits.end());
  const std::vector<int> old_hits =
      mapper.graph().vertices_within(Vec2{before.x, before.y}, 0.3);
  CHECK(std::find(old_hits.begin(), old_hits.end(), 1) == old_hits.end());
  std::printf("  move_vertex: 节点1 → (%.3f, %.3f, %.3f)，revision %llu→%llu，"
              "SPA %d→%d\n",
              target.x, target.y, target.theta,
              static_cast<unsigned long long>(rev0),
              static_cast<unsigned long long>(mapper.graph().pose_revision()),
              solves0, mapper.stats().solves);

  // 4) 非法输入：返回 false 且不污染图/统计
  const std::uint64_t rev1 = mapper.graph().pose_revision();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  CHECK(!mapper.move_vertex(-1, Pose2{0.0, 0.0, 0.0}));
  CHECK(!mapper.move_vertex(mapper.graph().size(), Pose2{0.0, 0.0, 0.0}));
  CHECK(!mapper.move_vertex(1, Pose2{nan, 0.0, 0.0}));
  CHECK(!mapper.move_vertex(1, Pose2{0.0, inf, 0.0}));
  CHECK(!mapper.move_vertex(1, Pose2{0.0, 0.0, nan}));
  CHECK(mapper.graph().pose_revision() == rev1);
  CHECK(mapper.stats().solves == solves0 + 1);
  CHECK(distance_xy(mapper.graph().vertices()[1].pose, target) < 1e-12);
}

}  // namespace

int main() {
  test_solver_fixed_id();
  test_mapper_move_vertex();
  std::puts("[PASS] test_interactive_move");
  return 0;
}
