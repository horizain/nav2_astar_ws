// Copyright 2026 ruijianding
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
// ============================================================================
// SE(2) 位姿代数与扫描数据结构 —— 全库的公共底座
// ============================================================================
#include <numbers>

#include <cmath>
#include <iterator>
#include <vector>

#include "Eigen/Dense"

namespace sloc
{

inline constexpr double kPi = std::numbers::pi;
inline constexpr double kTwoPi = 2.0 * kPi;

// O(1) 安全角度归一化至 (-kPi, kPi]；非有限输入（±inf / NaN）返回 0.0
inline double wrap_angle(double a)
{
  if (!std::isfinite(a)) {
    return 0.0;
  }
  double r = std::remainder(a, kTwoPi);
  if (r <= -kPi) {
    r += kTwoPi;
  } else if (r > kPi) {
    r -= kTwoPi;
  }
  return r;
}

// ---------- SE(2) ----------
struct Pose2
{
  double x{0.0}, y{0.0}, theta{0.0};

  Pose2() = default;
  Pose2(double x_, double y_, double th_)
  : x(x_), y(y_), theta(th_) {}

  Pose2 operator*(const Pose2 & o) const     // 本系位姿 o 复合到本位姿
  {
    const double c = std::cos(theta), s = std::sin(theta);
    return {x + c * o.x - s * o.y, y + s * o.x + c * o.y,
      wrap_angle(theta + o.theta)};
  }
  Pose2 inverse() const
  {
    const double c = std::cos(theta), s = std::sin(theta);
    return {-(c * x + s * y), -(-s * x + c * y), wrap_angle(-theta)};
  }
  Eigen::Vector2d trans() const {return {x, y};}
};

using Vec2 = Eigen::Vector2d;

// ---------- 扫描 ----------
struct Scan
{
  std::vector<Vec2> points;    // 传感器系直角坐标点
};

// 传感器系点云 → 世界系（缓冲复用重载，避免热循环重复分配）
template<typename It>
inline void transform_scan(
  const Pose2 & pose, It begin, It end, std::vector<Vec2> & out)
{
  out.clear();
  if constexpr (std::random_access_iterator<It>) {
    out.reserve(static_cast<std::size_t>(std::distance(begin, end)));
  }
  const double c = std::cos(pose.theta), s = std::sin(pose.theta);
  for (It it = begin; it != end; ++it) {
    const double px = it->x(), py = it->y();
    out.emplace_back(pose.x + c * px - s * py, pose.y + s * px + c * py);
  }
}

template<typename It>
inline std::vector<Vec2> transform_scan(const Pose2 & pose, It begin, It end)
{
  std::vector<Vec2> out;
  transform_scan(pose, begin, end, out);
  return out;
}

inline void transform_scan(
  const Pose2 & pose, const Scan & scan, std::vector<Vec2> & out)
{
  transform_scan(pose, scan.points.begin(), scan.points.end(), out);
}

inline std::vector<Vec2> transform_scan(const Pose2 & pose, const Scan & scan)
{
  std::vector<Vec2> out;
  transform_scan(pose, scan, out);
  return out;
}

// 3x3 位姿协方差（x, y, θ），Eigen 默认列主序
using Cov3 = Eigen::Matrix3d;

}  // namespace sloc
