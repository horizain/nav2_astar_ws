#pragma once
// ============================================================================
// scan_sim.hpp —— 2D 激光扫描仿真（bench 与调试工具共用）
// 射线求交生成一帧扫描（C++23 std::generator 协程），带高斯点噪声。
// ============================================================================
#include <cmath>
#include <generator>
#include <numbers>
#include <random>
#include <span>
#include <vector>

#include "fast_icp.hpp"
#include "icp.hpp"

namespace sim {

// 12×8 房间 + 柱子 + 内凸台（打破对称与沿墙滑移歧义）
struct Segment {
    double x0, y0, x1, y1;
};

inline auto make_room() -> std::vector<Segment>
{
    return {
        {0, 0, 12, 0},    {12, 0, 12, 8},   {12, 8, 0, 8},    {0, 8, 0, 0},
        // 柱子 0.5×0.5 @ (4, 3)
        {3.75, 2.75, 4.25, 2.75}, {4.25, 2.75, 4.25, 3.25},
        {4.25, 3.25, 3.75, 3.25}, {3.75, 3.25, 3.75, 2.75},
        // 内凸台（贴上墙）
        {8, 5, 10, 5},    {10, 5, 10, 8},
    };
}

// 惰性生成一帧扫描：每束射线与所有线段求交，co_yield 传感器系命中点
inline auto scan_beams(std::span<const Segment> segs, const icp::Pose2& sensor,
                       int beams, double fov, double range_max)
    -> std::generator<icp::Point2>
{
    const double sc = std::cos(sensor.theta);
    const double sd = std::sin(sensor.theta);
    for (int i = 0; i < beams; ++i) {
        const double a = sensor.theta - 0.5 * fov + fov * (i + 0.5) / beams;
        const double dx = std::cos(a);
        const double dy = std::sin(a);
        double t_best = range_max;
        bool hit = false;
        for (const auto& s : segs) {
            // o + t·d = p0 + u·(p1 - p0)
            const double ex = s.x1 - s.x0;
            const double ey = s.y1 - s.y0;
            const double den = dx * ey - dy * ex;
            if (std::abs(den) < 1e-12) {
                continue;
            }
            const double ox = s.x0 - sensor.x;
            const double oy = s.y0 - sensor.y;
            const double t = (ox * ey - oy * ex) / den;
            const double u = (ox * dy - oy * dx) / den;
            if (t > 0.0 && t < t_best && u >= 0.0 && u <= 1.0) {
                t_best = t;
                hit = true;
            }
        }
        if (hit) {
            // 命中点转回传感器系：R(-θ)·(t·d)
            co_yield {t_best * (sc * dx + sd * dy), t_best * (-sd * dx + sc * dy)};
        }
    }
}

struct ScanPair {
    icp::PointCloud2 pc;   // AoS double（参考版 ICP 用）
    ficp::Cloud fc;        // SoA float（FastICP 用）
};

// 传感器系点云 → 世界系
inline auto to_world(ScanPair& sp, const icp::Pose2& pose) -> void
{
    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    for (auto& p : sp.pc) {
        const double x = pose.x + c * p.x - s * p.y;
        const double y = pose.y + s * p.x + c * p.y;
        p.x = x;
        p.y = y;
    }
    for (std::size_t i = 0; i < sp.fc.size(); ++i) {
        const double x = pose.x + c * sp.fc.x[i] - s * sp.fc.y[i];
        const double y = pose.y + s * sp.fc.x[i] + c * sp.fc.y[i];
        sp.fc.x[i] = x;
        sp.fc.y[i] = y;
    }
}

inline auto make_scan(std::span<const Segment> segs, const icp::Pose2& sensor, int beams,
                      double sigma, std::mt19937& rng) -> ScanPair
{
    ScanPair out;
    out.fc.reserve(static_cast<std::size_t>(beams));
    std::normal_distribution<double> noise{0.0, sigma};
    for (const auto& p :
         scan_beams(segs, sensor, beams, 270.0 * std::numbers::pi / 180.0, 12.0)) {
        out.pc.push_back({p.x + noise(rng), p.y + noise(rng)});
        out.fc.push_back(out.pc.back().x, out.pc.back().y);
    }
    return out;
}

} // namespace sim
