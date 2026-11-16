// correlative matcher：给已知偏移的先验，能恢复真位姿；脱靶处得分趋零
#include "mini_slam/correlative_matcher.hpp"

#include "test_util.hpp"

using namespace mini_slam;

int main() {
  const auto walls = test::box_world();
  const Pose2 truth{2.0, 2.0, 0.3};

  // 参考扫描（无噪声）建场；查询扫描带测距噪声
  const LaserScan ref_scan = test::generate_scan(truth, walls, 361, 2.0 * kPi, 12.0);
  std::mt19937 rng{42};
  const LaserScan noisy =
      test::generate_scan(truth, walls, 361, 2.0 * kPi, 12.0, &rng);

  const FramedScan ref{truth, ref_scan.points};
  const auto field = LikelihoodField::build(std::span{&ref, 1}, 0.05, 2.0, 4);
  CHECK(field.has_value());

  // 先验偏移 (0.15 m, -0.10 m, 0.08 rad) —— 在窗口内
  const MatchWindow w{0.4, 0.5, 0.10, 0.05};
  const Pose2 prior{truth.x + 0.15, truth.y - 0.10, truth.theta + 0.08};
  const MatchResult m = CorrelativeMatcher::match(*field, prior, noisy.points, w);
  CHECK(m.valid);
  std::printf("  [info] response=%.3f coarse=%.3f\n", m.response, m.coarse_response);
  // 对准位姿本身的得分（匹配理论上限的参考）
  const double s_at_truth = field->score(truth, noisy.points);
  std::printf("  [info] score@truth=%.3f  score@prior=%.3f\n", s_at_truth,
              field->score(prior, noisy.points));
  CHECK(m.response > 0.4);
  CHECK(std::abs(m.pose.x - truth.x) < 0.06);
  CHECK(std::abs(m.pose.y - truth.y) < 0.06);
  CHECK(std::abs(wrap_angle(m.pose.theta - truth.theta)) < 0.03);

  // 出界位姿查表全 0（得分场的边界行为）
  const double far = field->score(Pose2{50, 50, 0}, noisy.points);
  CHECK(far < 1e-6);

  std::puts("[PASS] test_matcher");
  return 0;
}
