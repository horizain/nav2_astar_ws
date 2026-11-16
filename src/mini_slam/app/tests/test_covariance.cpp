// 扫描匹配协方差测试：
//  1) 辨识度高的场景（房间+柱阵）→ 协方差小（定位自信）
//  2) 退化场景（单一长墙）→ 沿墙方向滑移歧义 → 沿墙方差远大于垂直方向
//  3) 建图集成：边的信息矩阵随场景变化（不再是常数）
#include <cmath>
#include <set>
#include <random>

#include "mini_slam/mapper.hpp"

#include "test_util.hpp"

using namespace mini_slam;

int main() {
  // ---------- 1) 高辨识度场景 ----------
  {
    const auto walls = test::box_world();
    const Pose2 truth{2.0, 2.0, 0.3};
    const LaserScan ref_scan =
        test::generate_scan(truth, walls, 361, 2.0 * kPi, 12.0);
    const FramedScan ref{truth, ref_scan.points};
    const auto field =
        LikelihoodField::build(std::span{&ref, 1}, 0.05, 2.0, 4);
    CHECK(field.has_value());
    std::mt19937 rng{5};
    const LaserScan noisy =
        test::generate_scan(truth, walls, 361, 2.0 * kPi, 12.0, &rng);
    const MatchResult m = CorrelativeMatcher::match(
        *field, Pose2{truth.x + 0.1, truth.y - 0.1, truth.theta + 0.05},
        noisy.points, MatchWindow{0.4, 0.5, 0.10, 0.05});
    CHECK(m.valid);
    const double sxx = std::sqrt(std::max(m.covariance[0], 1e-9));
    const double syy = std::sqrt(std::max(m.covariance[4], 1e-9));
    std::printf("  [1] 房间场景 σx=%.3f σy=%.3f\n", sxx, syy);
    // 阈值注意：粗网格 (0.10m) 的量化使 σ 有 ~0.2 的下限——绝对值不代表
    // 好坏，退化场景与辨识场景的"对比"才是协方差的意义（见 [2]）
    CHECK(sxx < 0.35 && syy < 0.35);
  }

  // ---------- 2) 退化场景：单面长墙，沿墙滑移歧义 ----------
  {
    // 一面 40m 长墙（x 轴方向），机器人 1m 外正对它：沿墙 (x) 不可观测
    const std::vector<test::Segment> wall = {{{-20, 0}, {20, 0}}};
    const Pose2 facing{5.0, 1.0, -kPi / 2.0};  // 朝向 -y 正对墙
    const LaserScan ref_scan =
        test::generate_scan(facing, wall, 361, 2.0 * kPi, 12.0);
    const FramedScan ref{facing, ref_scan.points};
    const auto field =
        LikelihoodField::build(std::span{&ref, 1}, 0.05, 2.0, 4);
    CHECK(field.has_value());
    std::mt19937 rng{7};
    const LaserScan noisy =
        test::generate_scan(facing, wall, 361, 2.0 * kPi, 12.0, &rng);
    const MatchResult m = CorrelativeMatcher::match(
        *field, facing, noisy.points, MatchWindow{0.4, 0.5, 0.10, 0.05});
    CHECK(m.valid);
    const double cov_xx = m.covariance[0];   // 沿墙（x）方向
    const double cov_yy = m.covariance[4];   // 垂直墙（y）方向
    std::printf("  [2] 长墙场景 cov_xx=%.3f cov_yy=%.3f\n", cov_xx, cov_yy);
    CHECK(cov_xx > 1.5 * cov_yy);            // 沿墙各向异性被如实反映
    // 滑移歧义的直接证据：沿墙平移 0.4m 后得分几乎不变（响应平坦）
    const MatchResult m2 = CorrelativeMatcher::match(
        *field, Pose2{facing.x + 0.4, facing.y, facing.theta}, noisy.points,
        MatchWindow{0.05, 0.05, 0.02, 0.02});
    CHECK(m2.valid && std::abs(m2.response - m.response) < 0.1);
  }

  // ---------- 3) 建图集成：各向异性信息矩阵真正写进边 ----------
  {
    // 单面 40m 长墙 + 机器人沿墙平移：沿墙 (世界 x) 退化、垂直墙 (世界 y)
    // 精确。旧实现用 max(σx,σy) 压成各向同性，这里必须能看出 info_x != info_y。
    const std::vector<test::Segment> wall = {{{-20, 0}, {20, 0}}};
    Params params;
    Mapper mapper{params};
    std::mt19937 rng{31};
    Pose2 pose{0.5, 1.0, -kPi / 2.0};  // 面向墙
    for (int i = 0; i < 15; ++i) {
      const LaserScan scan =
          test::generate_scan(pose, wall, 361, 2.0 * kPi, 10.0, &rng);
      mapper.process_scan(scan, pose);
      pose.x += 0.5;   // 沿墙推进
    }
    std::set<std::pair<double, double>> infos;
    bool anisotropic = false;
    for (const auto& e : mapper.graph().edges()) {
      if (e.loop) continue;
      infos.insert({e.info_x, e.info_y});
      if (std::abs(e.info_x - e.info_y) > 1e-6) anisotropic = true;
    }
    std::printf("  [3] 非闭环边 (info_x,info_y) 取值数: %zu，存在各向异性=%d\n",
                infos.size(), anisotropic ? 1 : 0);
    CHECK(infos.size() >= 3);   // 信息随匹配质量变化，不再是常数
    CHECK(anisotropic);         // mapper 不再把 x/y 压成同一个标量
  }

  std::puts("[PASS] test_covariance");
  return 0;
}
