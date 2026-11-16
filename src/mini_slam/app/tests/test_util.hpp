#pragma once
// ============================================================================
// 测试共用工具：CHECK 断言宏 + 线段世界 + 射线投射 + 合成扫描生成。
// 有意不引 gtest：算法核保持零依赖，测试也零依赖。
// ============================================================================
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <span>
#include <vector>

#include "mini_slam/types.hpp"

#define CHECK(cond)                                                          \
  do {                                                                       \
    if (!(cond)) {                                                           \
      std::printf("[FAIL] %s:%d  %s\n", __FILE__, __LINE__, #cond);          \
      std::exit(1);                                                          \
    }                                                                        \
  } while (0)

namespace mini_slam::test {

struct Segment {
  Vec2 a, b;
};

// 合成世界：外框 + 两段内部隔断（让场景有辨识度，闭环匹配不歧义）
inline std::vector<Segment> box_world(double w = 14.0, double h = 9.0) {
  return {
      {{0, 0}, {w, 0}},   {{w, 0}, {w, h}},   {{w, h}, {0, h}},
      {{0, h}, {0, 0}},   {{5, 2}, {5, 5}},   {{9, 6}, {12, 6}},
  };
}

// 射线 o + t·dir 与线段集的最近交点距离；无交点返回 max_range
inline double cast_ray(Vec2 o, Vec2 dir, std::span<const Segment> walls,
                       double max_range) {
  double best = max_range;
  for (const auto& w : walls) {
    const Vec2 s = w.b - w.a;
    const Vec2 ao = w.a - o;
    const double det = cross(dir, s);
    if (std::abs(det) < 1e-12) continue;
    const double t = cross(ao, s) / det;
    const double u = cross(ao, dir) / det;
    if (t > 0 && u >= 0 && u <= 1 && t < best) best = t;
  }
  return best;
}

// 在位姿处生成一帧合成扫描。返回的点是**传感器系**（与 LaserScan 约定一致）。
inline LaserScan generate_scan(const Pose2& pose,
                               std::span<const Segment> walls, int n_rays,
                               double fov, double range_max,
                               std::mt19937* rng = nullptr,
                               double noise_sigma = 0.02) {
  LaserScan scan;
  std::normal_distribution<double> noise(0.0, noise_sigma);
  for (int i = 0; i < n_rays; ++i) {
    const double ang = pose.theta - fov / 2.0 +
                       fov * static_cast<double>(i) /
                           static_cast<double>(n_rays - 1);
    const double r = cast_ray(Vec2{pose.x, pose.y},
                              Vec2{std::cos(ang), std::sin(ang)}, walls,
                              range_max);
    if (r >= range_max - 1e-9) continue;  // 无回波 → 丢弃（真实雷达同样如此）
    const double rr = rng ? r + noise(*rng) : r;
    const double rel_ang = ang - pose.theta;  // 传感器系内的方向角
    scan.points.push_back(
        Vec2{rr * std::cos(rel_ang), rr * std::sin(rel_ang)});
  }
  return scan;
}

}  // namespace mini_slam::test
