// demo：用参考实现（src/icp.cpp）做一次合成扫描配准
#include <cmath>
#include <print>
#include <random>
#include <vector>

#include "icp.hpp"

namespace {

// 10×6 房间轮廓 + 柱子（打破沿墙滑移歧义）
auto make_room(double step = 0.05) -> icp::PointCloud2
{
    icp::PointCloud2 pts;
    const auto wall = [&](double x0, double y0, double x1, double y1) {
        const int n = static_cast<int>(std::hypot(x1 - x0, y1 - y0) / step);
        for (int i = 0; i <= n; ++i) {
            const double t = static_cast<double>(i) / n;
            pts.push_back({x0 + t * (x1 - x0), y0 + t * (y1 - y0)});
        }
    };
    wall(0, 0, 10, 0);
    wall(10, 0, 10, 6);
    wall(10, 6, 0, 6);
    wall(0, 6, 0, 0);
    wall(4, 3, 4.5, 3);
    wall(4.5, 3, 4.5, 3.5);
    wall(4.5, 3.5, 4, 3.5);
    wall(4, 3.5, 4, 3);
    return pts;
}

// source = T⁻¹ · target + 噪声（T 为真值位姿）
auto make_source(const icp::PointCloud2& target, const icp::Pose2& pose,
                 std::mt19937& rng, double sigma) -> icp::PointCloud2
{
    const double c = std::cos(pose.theta);
    const double s = std::sin(pose.theta);
    std::normal_distribution<double> noise{0.0, sigma};
    icp::PointCloud2 out;
    out.reserve(target.size());
    for (const auto& p : target) {
        const double dx = p.x - pose.x;
        const double dy = p.y - pose.y;
        out.push_back({c * dx + s * dy + noise(rng), -s * dx + c * dy + noise(rng)});
    }
    return out;
}

} // namespace

auto main() -> int
{
    std::println("ICP Point-to-Line 2D 配准 demo（C++23, 纯标准库）");

    std::mt19937 rng{42};
    constexpr icp::Pose2 gt{.x = 0.5, .y = -0.3, .theta = 0.08};

    const auto target = make_room();
    const auto source = make_source(target, gt, rng, 0.01);

    const auto result = icp::align(source, target);
    if (!result) {
        std::println("ICP 失败: {}", icp::to_string(result.error()));
        return 1;
    }

    std::println("真值位姿: x={:.4f} y={:.4f} θ={:.4f}", gt.x, gt.y, gt.theta);
    std::println("ICP 位姿: x={:.4f} y={:.4f} θ={:.4f}", result->pose.x,
                 result->pose.y, result->pose.theta);
    std::println("误差:     Δ={:.2f}mm / {:.2f}mrad | 迭代 {} 次 | RMS {:.3f}mm",
                 std::hypot(result->pose.x - gt.x, result->pose.y - gt.y) * 1000.0,
                 std::abs(std::remainder(result->pose.theta - gt.theta, 6.283185307179586)) * 1000.0,
                 result->iterations, result->rms_error * 1000.0);
    return 0;
}
