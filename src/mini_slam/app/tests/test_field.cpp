// 得分场基本性质：对准高分、脱靶趋零、1 格偏移被模糊场兜住
#include "mini_slam/likelihood_field.hpp"

#include "test_util.hpp"

using namespace mini_slam;

int main() {
  // 一帧"竖直墙"扫描：传感器系下沿 +y 的 40 个点，间隔 0.05 m
  LaserScan wall;
  for (int i = 0; i < 40; ++i) {
    wall.points.push_back(Vec2{0.0, i * 0.05});
  }

  const FramedScan ref{Pose2{0, 0, 0}, wall.points};
  const auto field =
      LikelihoodField::build(std::span{&ref, 1}, 0.05, 1.0, 4);
  CHECK(field.has_value());

  // 位姿没动：点投影回墙上，得分高
  const double s_exact = field->score(Pose2{0, 0, 0}, wall.points);
  CHECK(s_exact > 0.5);

  // 平移 2 m：完全脱靶，得分 ≈ 0
  const double s_far = field->score(Pose2{2.0, 0, 0}, wall.points);
  CHECK(s_far < 0.05);

  // 偏移恰好 1 格（0.05 m）：模糊场兜住，得分下降但不归零
  const double s_one = field->score(Pose2{0.05, 0, 0}, wall.points);
  CHECK(s_one > 0.1);
  CHECK(s_one < s_exact);

  std::puts("[PASS] test_field");
  return 0;
}
