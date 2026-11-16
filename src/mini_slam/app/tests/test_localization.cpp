// 定位模式测试：
//  1) 建图阶段沿环路建图 → 切定位模式 → 重走同一环路，验证跟踪精度
//  2) 图冻结断言：定位阶段节点数不变
//  3) 传送门断言：里程计先验远离建图区域时 valid=false（退回先验）
#include <cmath>
#include <filesystem>
#include <string>
#include <unistd.h>
#include <random>
#include <vector>

#include "mini_slam/mapper.hpp"

#include "test_util.hpp"

using namespace mini_slam;

namespace {

// 矩形环路轨迹（与 test_mapper_synthetic 同款）：返回 GT 位姿序列
std::vector<Pose2> make_loop(double side = 1.5, double step = 0.25) {
  const std::vector<Vec2> corners = {Vec2{0, 0}, Vec2{side, 0},
                                     Vec2{side, side}, Vec2{0, side},
                                     Vec2{0, 0}};
  std::vector<Pose2> gt;
  for (size_t c = 0; c + 1 < corners.size(); ++c) {
    const Vec2 d = corners[c + 1] - corners[c];
    const double len = norm(d);
    const double heading = std::atan2(d.y, d.x);
    const int steps = std::max(1, static_cast<int>(std::ceil(len / step)));
    for (int i = 0; i < steps; ++i) {
      const double t = static_cast<double>(i) / steps;
      gt.push_back(Pose2{corners[c].x + t * d.x, corners[c].y + t * d.y,
                         heading});
    }
  }
  gt.push_back(Pose2{corners.back().x, corners.back().y, 0.0});
  return gt;
}

}  // namespace

int main() {
  const auto walls = test::box_world();
  std::mt19937 rng{11};

  // ---------- 建图阶段：走一圈，构建图 ----------
  Mapper mapper;
  mapper.set_mode(Mapper::Mode::Mapping);
  const auto gt = make_loop();
  Pose2 odom = gt.front();
  for (size_t i = 1; i < gt.size(); ++i) {
    const Pose2 delta = compose(inverse(gt[i - 1]), gt[i]);
    odom = compose(odom, delta);   // 合成仿真里里程计近乎完美
    const LaserScan scan =
        test::generate_scan(gt[i], walls, 361, 2.0 * kPi, 10.0, &rng);
    mapper.process_scan(scan, odom);
  }
  const int nodes_after_mapping = mapper.stats().nodes;
  CHECK(nodes_after_mapping > 10);

  // ---------- 切换定位模式：重走同一环路 ----------
  mapper.set_mode(Mapper::Mode::Localization);

  std::mt19937 rng2{77};
  std::normal_distribution<double> jitter(0.0, 0.02);   // 里程计重放噪声
  double max_err = 0.0;
  int valid_cnt = 0, frames = 0;
  Pose2 loc_odom = gt.front();
  Pose2 last_localized_pose;
  for (size_t i = 1; i < gt.size(); ++i) {
    const Pose2 delta = compose(inverse(gt[i - 1]), gt[i]);
    Pose2 noisy_delta = delta;
    noisy_delta.x += jitter(rng2);
    noisy_delta.y += jitter(rng2);
    noisy_delta.theta += 0.002 * jitter(rng2);
    loc_odom = compose(loc_odom, noisy_delta);

    const LaserScan scan =
        test::generate_scan(gt[i], walls, 361, 2.0 * kPi, 10.0, &rng2);
    const auto res = mapper.localize(scan, loc_odom);
    last_localized_pose = res.pose;
    ++frames;
    if (res.valid) ++valid_cnt;
    max_err = std::max(max_err, distance_xy(res.pose, gt[i]));
  }
  CHECK(frames > 10);
  CHECK(valid_cnt > frames * 9 / 10);          // 绝大多数帧匹配可信
  CHECK(max_err < 0.15);                        // 跟踪误差 < 15cm
  CHECK(mapper.stats().nodes == nodes_after_mapping);  // 图冻结 ✓

  // 同图切定位时，即使图优化后 map→odom 非零，首帧也应延续建图位姿。
  // 空扫描不会产生可信匹配，因而可以单独验证先验而不依赖匹配得分。
  {
    Mapper switching;
    LaserScan empty;
    switching.process_scan(empty, Pose2{0.0, 0.0, 0.0});
    switching.process_scan(empty, Pose2{0.6, 0.0, 0.0});
    CHECK(switching.graph().size() == 2);
    CHECK(switching.move_vertex(1, Pose2{1.6, 0.3, 0.1}));
    const Pose2 latest_odom{0.7, 0.0, 0.0};
    switching.process_scan(empty, latest_odom);  // 未达到入图阈值
    const Pose2 next_odom{0.8, 0.0, 0.0};
    const Pose2 expected = compose(switching.map_odom_tf(), next_odom);
    CHECK(distance_xy(expected, next_odom) > 0.5);
    switching.set_mode(Mapper::Mode::Localization);
    const auto first = switching.localize(empty, next_odom);
    CHECK(!first.valid);
    CHECK(distance_xy(first.pose, expected) < 1e-8);
    CHECK(std::abs(wrap_angle(first.pose.theta - expected.theta)) < 1e-8);

    // 换图后绝不能把旧图的 map→odom 当成新图先验。
    const auto file = std::filesystem::temp_directory_path() /
        ("mini_slam_mode_reload_" + std::to_string(static_cast<long>(::getpid())) + ".mmpg");
    std::string err;
    CHECK(switching.save_graph(file.string(), &err));
    switching.set_mode(Mapper::Mode::Mapping);
    CHECK(switching.load_graph(file.string(), &err));
    switching.set_mode(Mapper::Mode::Localization);
    const auto reloaded = switching.localize(empty, next_odom);
    CHECK(!reloaded.valid);
    CHECK(distance_xy(reloaded.pose, next_odom) < 1e-8);
  }

  // 旧图的里程计与新会话完全不同，定位后继续建图时不能连接旧尾节点
  // 的原始里程计坐标，否则 map→odom 会跳变数十米。
  {
    Mapper resumed;
    LaserScan empty;
    resumed.process_scan(empty, Pose2{0.0, 0.0, 0.0});
    resumed.process_scan(empty, Pose2{0.6, 0.0, 0.0});
    const Pose2 localized{0.1, 0.1, 0.0};
    const Pose2 new_odom{40.0, -20.0, 0.0};
    resumed.set_mode(Mapper::Mode::Localization);
    resumed.initialize_localization(localized, new_odom);
    resumed.set_mode(Mapper::Mode::Mapping);
    const Pose2 continued_odom{40.1, -20.0, 0.0};
    const Pose2 predicted = compose(localized, delta_pose(new_odom, continued_odom));
    const auto added = resumed.process_scan(empty, continued_odom);
    CHECK(added.added);
    CHECK(distance_xy(resumed.graph().vertices().back().pose, predicted) < 1e-8);
    CHECK(distance_xy(compose(resumed.map_odom_tf(), continued_odom), predicted) < 1e-8);
    CHECK(resumed.graph().edges().back().from == 0);
    const Pose2 later_odom{40.8, -20.0, 0.0};
    const Pose2 later_expected = compose(localized, delta_pose(new_odom, later_odom));
    CHECK(resumed.process_scan(empty, later_odom).added);
    CHECK(distance_xy(resumed.graph().vertices().back().pose, later_expected) < 1e-8);
    CHECK(resumed.graph().edges().back().from == 2);
  }

  // 还未收到建图扫描就切回定位，仍应保留定位锚点。
  {
    Mapper rapid;
    LaserScan empty;
    rapid.process_scan(empty, Pose2{0.0, 0.0, 0.0});
    const Pose2 map_pose{1.0, 2.0, 0.3};
    const Pose2 odom_pose{30.0, -10.0, -0.2};
    rapid.set_mode(Mapper::Mode::Localization);
    rapid.initialize_localization(map_pose, odom_pose);
    rapid.set_mode(Mapper::Mode::Mapping);
    CHECK(distance_xy(compose(rapid.map_odom_tf(), odom_pose), map_pose) < 1e-8);
    rapid.set_mode(Mapper::Mode::Localization);
    const auto result = rapid.localize(empty, odom_pose);
    CHECK(distance_xy(result.pose, map_pose) < 1e-8);
  }

  // ---------- 传送门断言：先验远离建图区域 → 匹配不可信 ----------
  const Pose2 far_pose{50.0, 50.0, 0.5};
  const LaserScan scan_far =
      test::generate_scan(gt.front(), walls, 361, 2.0 * kPi, 10.0);
  const auto res_far = mapper.localize(scan_far, far_pose);
  CHECK(!res_far.valid);                        // 场外定位：退回先验
  const Pose2 expected_far = compose(last_localized_pose, delta_pose(loc_odom, far_pose));
  CHECK(distance_xy(res_far.pose, expected_far) < 1e-8);

  std::printf("[PASS] test_localization  (nodes=%d frames=%d valid=%d max_err=%.3fm)\n",
              nodes_after_mapping, frames, valid_cnt, max_err);
  return 0;
}
