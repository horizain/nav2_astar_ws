// 角度跨 ±π 回归：
//  1) 统一最短角差辅助（double）
//  2) mapping 位移判定：3.10 → -3.10 只转了 +0.083，不应触发入图
//  3) localization 里程计增量先验：跨 ±π 后仍贴近当前里程计
//  4) SPA 角残差：初始已满足的 ±π 等价边不应被优化拉走
#include <cmath>
#include <vector>

#include "mini_slam/mapper.hpp"
#include "mini_slam/pose_graph.hpp"
#include "mini_slam/spa_solver.hpp"

#include "test_util.hpp"

using namespace mini_slam;

namespace {

LaserScan simple_scan() {
  LaserScan scan;
  for (int i = 0; i < 20; ++i) {
    scan.points.push_back(Vec2{2.0, -0.5 + 0.05 * i});
  }
  return scan;
}

}  // namespace

int main() {
  // ---------- 1) 最短角差 ----------
  constexpr double a = 3.10;
  constexpr double b = -3.10;
  const double d_ab = shortest_angle_diff(b, a);  // -3.10 - 3.10
  CHECK(std::abs(d_ab - 0.083185307179586) < 1e-9);
  const double d_ba = shortest_angle_diff(a, b);
  CHECK(std::abs(d_ba + 0.083185307179586) < 1e-9);

  const Pose2 p0{0.0, 0.0, a};
  const Pose2 p1{0.0, 0.0, b};
  CHECK(std::abs(delta_pose(p0, p1).theta - 0.083185307179586) < 1e-9);

  // ---------- 2) mapping 位移判定 ----------
  {
    Params params;
    params.min_travel_distance = 1.0;  // 这两帧平移只有 1cm
    params.min_travel_heading = 0.2;   // 真实转角 0.083 < 0.2
    Mapper mapper{params};
    const LaserScan scan = simple_scan();
    mapper.process_scan(scan, Pose2{0.0, 0.0, a});
    CHECK(mapper.stats().nodes == 1);

    // 旧实现会把 delta 角算成 -6.2，误判为"转够了"而插入第二个节点
    mapper.process_scan(scan, Pose2{0.01, 0.0, b});
    CHECK(mapper.stats().nodes == 1);

    // 真走够了仍然要入图（确认阈值判定没有被 wrap 破坏）
    mapper.process_scan(scan, Pose2{2.0, 0.0, b});
    CHECK(mapper.stats().nodes == 2);
  }

  // ---------- 3) localization 增量先验 ----------
  {
    Params params;
    params.min_travel_distance = 0.5;
    Mapper mapper{params};
    const LaserScan scan = simple_scan();
    mapper.process_scan(scan, Pose2{0.0, 0.0, a});
    mapper.set_mode(Mapper::Mode::Localization);

    // 初始化跟踪状态（首帧直接信任里程计）
    mapper.localize(scan, Pose2{0.0, 0.0, a});
    // 航向跨过 ±π：正确的增量是 +0.083，先验应贴近当前 odom
    const auto res = mapper.localize(scan, Pose2{0.01, 0.0, b});
    CHECK(distance_xy(res.pose, Pose2{0.01, 0.0, b}) < 0.35);
    CHECK(std::abs(shortest_angle_diff(res.pose.theta, b)) < 0.35);
  }

  // ---------- 4) SPA 角残差 ----------
  {
    PoseGraph g;
    const Pose2 n0{0.0, 0.0, a};
    const Pose2 n1{1.0, 0.0, b};  // 与 n0 的真实相对角 = +0.083
    g.add_vertex(n0, n0, nullptr);
    g.add_vertex(n1, n1, nullptr);
    g.add_edge(0, 1, delta_pose(n0, n1), 100.0, 100.0, 100.0, false);

    const SpaSolver solver;
    const SpaSummary s = solver.solve(g);
    // 初始残差已是 2π 等价于 0：优化不应把节点转走
    CHECK(s.final_cost < 1e-6);
    CHECK(distance_xy(g.vertices()[1].pose, n1) < 1e-3);
    CHECK(std::abs(shortest_angle_diff(g.vertices()[1].pose.theta, b)) < 1e-3);
  }

  std::puts("[PASS] test_angle_wrap");
  return 0;
}
