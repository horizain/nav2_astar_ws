#pragma once
// ============================================================================
// SE(2) 位姿代数与扫描数据结构 —— 全库的公共底座
// ============================================================================
#include <cmath>
#include <cstdint>
#include <vector>

#include <Eigen/Dense>

namespace sloc {

// ---------- SE(2) ----------
struct Pose2 {
    double x{0.0}, y{0.0}, theta{0.0};

    Pose2() = default;
    Pose2(double x_, double y_, double th_) : x(x_), y(y_), theta(th_) {}

    Pose2 operator*(const Pose2& o) const {  // 本系位姿 o 复合到本位姿
        const double c = std::cos(theta), s = std::sin(theta);
        return {x + c * o.x - s * o.y, y + s * o.x + c * o.y,
                theta + o.theta};
    }
    Pose2 inverse() const {
        const double c = std::cos(theta), s = std::sin(theta);
        return {-(c * x + s * y), -(-s * x + c * y), -theta};
    }
    Eigen::Vector2d trans() const { return {x, y}; }
};

using Vec2 = Eigen::Vector2d;

// 传感器系点云 → 世界系
template <typename It>
inline std::vector<Vec2> transform_scan(const Pose2& pose, It begin, It end) {
    const double c = std::cos(pose.theta), s = std::sin(pose.theta);
    std::vector<Vec2> out;
    out.reserve(static_cast<std::size_t>(end - begin));
    for (It it = begin; it != end; ++it) {
        const double px = it->x(), py = it->y();
        out.emplace_back(pose.x + c * px - s * py, pose.y + s * px + c * py);
    }
    return out;
}

// ---------- 扫描 ----------
struct Scan {
    std::vector<Vec2> points;  // 传感器系直角坐标点
};

// 3x3 位姿协方差（x, y, θ），行主序
using Cov3 = Eigen::Matrix3d;

}  // namespace sloc
