// 定位模式测试：
//  1) 建图阶段沿环路建图 → 切定位模式 → 重走同一环路，验证跟踪精度
//  2) 图冻结断言：定位阶段节点数不变
//  3) 传送门断言：里程计先验远离建图区域时 valid=false（退回先验）
#include <cmath>
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
    ++frames;
    if (res.valid) ++valid_cnt;
    max_err = std::max(max_err, distance_xy(res.pose, gt[i]));
  }
  CHECK(frames > 10);
  CHECK(valid_cnt > frames * 9 / 10);          // 绝大多数帧匹配可信
  CHECK(max_err < 0.15);                        // 跟踪误差 < 15cm
  CHECK(mapper.stats().nodes == nodes_after_mapping);  // 图冻结 ✓

  // ---------- 传送门断言：先验远离建图区域 → 匹配不可信 ----------
  const Pose2 far_pose{50.0, 50.0, 0.5};
  const LaserScan scan_far =
      test::generate_scan(gt.front(), walls, 361, 2.0 * kPi, 10.0);
  const auto res_far = mapper.localize(scan_far, far_pose);
  CHECK(!res_far.valid);                        // 场外定位：退回先验
  CHECK(distance_xy(res_far.pose, far_pose) < 0.5);  // 退回的先验 ≈ 远点本身

  std::printf("[PASS] test_localization  (nodes=%d frames=%d valid=%d max_err=%.3fm)\n",
              nodes_after_mapping, frames, valid_cnt, max_err);
  return 0;
}
