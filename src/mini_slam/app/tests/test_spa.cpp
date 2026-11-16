// Ceres SPA：带漂移初值 + 完美相对观测（含闭环边）→ 优化收敛到真值
#include <cmath>
#include <vector>

#include "mini_slam/pose_graph.hpp"
#include "mini_slam/spa_solver.hpp"

#include "test_util.hpp"

using namespace mini_slam;

int main() {
  // 8 节点绕圈，半径 5 m，朝向沿切线
  const int n = 8;
  const double radius = 5.0;
  std::vector<Pose2> gt(n);
  for (int i = 0; i < n; ++i) {
    const double th = 2.0 * kPi * i / n;
    gt[i] = Pose2{radius * std::cos(th), radius * std::sin(th), th + kPi / 2.0};
  }

  PoseGraph g;
  for (int i = 0; i < n; ++i) {
    // 初值：真值 + 随节点号线性增长的漂移（节点 0 精确，作为规范固定点）
    const Pose2 est{gt[i].x + 0.05 * i, gt[i].y + 0.02 * i,
                    gt[i].theta + 0.01 * i};
    g.add_vertex(gt[i] /*odom*/, est, nullptr);
  }

  // 边：相邻节点的完美相对观测 + 最后一条闭环边（7 → 0）
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    const Pose2 z = compose(inverse(gt[i]), gt[j]);
    const bool loop = (j == 0);
    g.add_edge(i, j, z, 100.0, 100.0, 100.0, loop);
  }

  const SpaSolver solver;
  const SpaSummary s = solver.solve(g);

  CHECK(s.final_cost < s.initial_cost);
  double max_err = 0.0;
  for (int i = 0; i < n; ++i) {
    max_err = std::max(max_err, distance_xy(g.vertices()[i].pose, gt[i]));
  }
  CHECK(max_err < 1e-3);  // 完美数据 + 完美闭环 → 应收敛到真值附近

  // ---------- 各向异性信息：世界 x 硬、世界 y 软 ----------
  {
    PoseGraph g;
    g.add_vertex(Pose2{0, 0, 0}, Pose2{0, 0, 0}, nullptr);
    g.add_vertex(Pose2{0, 0, 0}, Pose2{0.5, 0.5, 0}, nullptr);
    // 两条观测分别把世界 x / 世界 y 拉向不同目标：
    //   x 方向信息 100，y 方向信息 1 → 应优先满足 (1,1) 的 x=1；
    //   反之 y 方向硬 → 应优先满足 y=0。
    g.add_edge(0, 1, Pose2{1, 1, 0}, 100.0, 1.0, 1.0, false);
    g.add_edge(0, 1, Pose2{0, 0, 0}, 1.0, 100.0, 1.0, false);
    const SpaSolver s2;
    s2.solve(g);
    std::printf("  各向异性结果: x=%.3f y=%.3f\n",
                g.vertices()[1].pose.x, g.vertices()[1].pose.y);
    CHECK(g.vertices()[1].pose.x > 0.9);
    CHECK(std::abs(g.vertices()[1].pose.y) < 0.1);
  }

  // ---------- 回归：非零 z.theta + 各向异性 info（残差坐标系）----------
  // 场景：节点0 固定在 (0,0,π/2)，节点1 自由、初值 (0,0,0)；两条 0→1 边都
  // 带 z_theta = π/2，且 info 各向异性（世界 x 硬 / 世界 y 硬各一条）：
  //   边A z=(1,0,π/2) info=(100,1,100)
  //   边B z=(0,1,π/2) info=(1,100,100)
  // 正确残差 e_xy = R(a.θ)·(Δt_ab - z_xy) 给出的代价为
  //   100x² + (y-1)² + (x+1)² + 100y²
  // 解析解 x = -1/101 ≈ -0.0099，y = +1/101 ≈ +0.0099，θ = π。
  // 若把 z⁻¹ 旋转后的残差再按 a[2] 旋转（等价于按 a.θ + z.θ 旋转），
  // 加权方向被多转 90°，解会变成约 (-0.99, +0.99)——本用例专捕该错误。
  {
    PoseGraph g;
    g.add_vertex(Pose2{0, 0, kPi / 2.0}, Pose2{0, 0, kPi / 2.0}, nullptr);
    g.add_vertex(Pose2{0, 0, 0}, Pose2{0, 0, 0}, nullptr);
    g.add_edge(0, 1, Pose2{1, 0, kPi / 2.0}, 100.0, 1.0, 100.0, false);
    g.add_edge(0, 1, Pose2{0, 1, kPi / 2.0}, 1.0, 100.0, 100.0, false);
    const SpaSolver s3;
    s3.solve(g);
    const Pose2& p1 = g.vertices()[1].pose;
    const double dth = std::abs(shortest_angle_diff(p1.theta, kPi));
    std::printf("  非零 z.theta 回归: x=%.9f y=%.9f theta=%.9f |dθ-π|=%.3e\n",
                p1.x, p1.y, p1.theta, dth);
    CHECK(std::abs(p1.x - (-1.0 / 101.0)) < 1e-3);
    CHECK(std::abs(p1.y - (1.0 / 101.0)) < 1e-3);
    CHECK(dth < 1e-3);
  }

  std::puts("[PASS] test_spa");
  return 0;
}
