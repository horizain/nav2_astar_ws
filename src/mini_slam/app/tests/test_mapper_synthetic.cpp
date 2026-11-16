// 端到端：合成世界 + 漂移里程计走一圈 → 闭环触发、图优化消除漂移、地图合理
#include <cmath>
#include <random>
#include <vector>

#include "mini_slam/map_builder.hpp"
#include "mini_slam/mapper.hpp"

#include "test_util.hpp"

using namespace mini_slam;

int main() {
  const auto walls = test::box_world();

  // GT 轨迹：矩形环路 A→B→C→D→A 后，再沿 A→B 方向重走 7m（制造真实的
  // "长走廊重访"场景——矩形回环只在起点附近有少量老节点，凑不满闭环候选链）
  const std::vector<Vec2> corners = {
      Vec2{1, 1}, Vec2{12, 1}, Vec2{12, 7.5}, Vec2{1, 7.5}, Vec2{1, 1},
      Vec2{8, 1}};
  std::vector<Pose2> gt;
  for (size_t c = 0; c + 1 < corners.size(); ++c) {
    const Vec2 d = corners[c + 1] - corners[c];
    const double len = norm(d);
    const double heading = std::atan2(d.y, d.x);
    const int steps = std::max(1, static_cast<int>(std::ceil(len / 0.4)));
    for (int i = 0; i < steps; ++i) {
      const double t = static_cast<double>(i) / steps;
      gt.push_back(Pose2{corners[c].x + t * d.x, corners[c].y + t * d.y,
                         heading});
    }
  }
  gt.push_back(Pose2{corners.back().x, corners.back().y, 0.0});

  // 里程计：增量缩放 + 噪声 → 系统性漂移（约 1.5% 平移 / 1% 转角）
  std::mt19937 rng{7};
  std::normal_distribution<double> n_t(0.0, 0.008);
  std::normal_distribution<double> n_r(0.0, 0.004);

  Params params;
  params.min_travel_distance = 0.4;   // 轨迹步长 0.4m：每帧都入图，保证闭环候选链够大
  params.loop_min_chain_size = 6;
  params.loop_search_distance = 4.0;
  params.verbose = true;
  Mapper mapper{params};

  Pose2 odom = gt.front();
  for (size_t i = 1; i < gt.size(); ++i) {
    const Pose2 delta_true = compose(inverse(gt[i - 1]), gt[i]);
    Pose2 delta_odom;
    delta_odom.x = delta_true.x * 0.985 + n_t(rng);
    delta_odom.y = delta_true.y * 0.985 + n_t(rng);
    delta_odom.theta = delta_true.theta * 0.99 + n_r(rng);
    odom = compose(odom, delta_odom);

    const LaserScan scan =
        test::generate_scan(gt[i], walls, 361, 2.0 * kPi, 10.0, &rng);
    mapper.process_scan(scan, odom);
  }

  // ---------- 断言 ----------
  const auto& stats = mapper.stats();
  CHECK(stats.nodes > 30);
  CHECK(stats.loop_closures >= 1);
  CHECK(stats.solves >= 1);

  // 闭环前里程计确实漂了（对照项）
  const auto& last = mapper.graph().vertices().back();
  const double odom_err = distance_xy(last.odom_pose, gt.back());
  CHECK(odom_err > 0.15);

  // 图优化把漂移消掉
  const double map_err = distance_xy(last.pose, gt.back());
  CHECK(map_err < 0.5);
  CHECK(map_err < odom_err);  // 至少要修掉一部分

  // 地图渲染：墙有占据、内部有大量空闲格
  const auto map = MapBuilder::render(mapper.graph(), 0.05, 10.0, 1.0);
  CHECK(map.width > 0);
  long occ = 0, free_cells = 0;
  for (const std::int8_t v : map.data) {
    if (v == 100) ++occ;
    if (v == 0) ++free_cells;
  }
  CHECK(occ > 200);
  CHECK(free_cells > 2000);

  std::printf("[PASS] test_mapper_synthetic  (nodes=%d loops=%d odom_err=%.2fm map_err=%.2fm occ=%ld free=%ld)\n",
              stats.nodes, stats.loop_closures, odom_err, map_err, occ,
              free_cells);
  return 0;
}
