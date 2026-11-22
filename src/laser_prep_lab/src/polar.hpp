#pragma once
// ============================================================================
// PolarScan：有序极坐标扫描——本包的中心数据结构。
//
// 为什么不是 PointCloud2：清洗类算法（椒盐剔除、鬼点、掠射阴影）
// 全部依赖"相邻束"这个拓扑关系，deskew 依赖"每束的时刻"。
// 这两样信息在无序点云里都已经丢了。所以链路约定：
//   原始极坐标 → 逐束清洗（NaN 掉坏束）→ 最后一步才转点云。
//
// 无效束的表示：range = NaN。所有算子遇到 NaN 直接跳过，
// 绝不"猜一个值填上"——数据清洗的第一原则是不制造数据。
// ============================================================================
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "icp.hpp"

namespace lpl {

inline constexpr double kPi = 3.14159265358979323846;
// 无效束的统一表示：所有模块都用这一个
inline constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

struct PolarScan {
    double stamp{};      // 扫描起始时刻 (s)——deskew 的时间基准
    double beam_dt{};    // 相邻束时间间隔 (s)；0 = 未知（禁用 deskew）
    double angle_min{};  // 首束角度 (rad)
    double angle_inc{};  // 相邻束角间隔 (rad)
    std::vector<double> range;  // m；NaN = 无效/无回波

    [[nodiscard]] int size() const
    {
        return static_cast<int>(range.size());
    }
    [[nodiscard]] double angle(int i) const
    {
        return angle_min + angle_inc * static_cast<double>(i);
    }
    // 第 i 束的采样时刻。束间均匀分布是简化假设——真实雷达的
    // 束序时间未必均匀（如电机变速），需要时可用非线性表校正。
    [[nodiscard]] double time(int i) const
    {
        return stamp + beam_dt * static_cast<double>(i);
    }
    [[nodiscard]] bool valid(int i) const
    {
        return std::isfinite(range[static_cast<std::size_t>(i)]);
    }

    // 洗完之后才允许转点云（传感器系）。deskew 另有专门入口。
    [[nodiscard]] icp::PointCloud2 endpoints() const
    {
        icp::PointCloud2 pts;
        pts.reserve(range.size());
        for (int i = 0; i < size(); ++i) {
            if (!valid(i)) continue;
            const double a = angle(i);
            pts.push_back({range[static_cast<std::size_t>(i)] * std::cos(a),
                           range[static_cast<std::size_t>(i)] * std::sin(a)});
        }
        return pts;
    }
};

// SE(2) 工具（与 icp::Pose2 配套的小函数，仅本包内部使用）
[[nodiscard]] constexpr icp::Point2 transform(const icp::Pose2& p,
                                              const icp::Point2& q)
{
    const double c = std::cos(p.theta), s = std::sin(p.theta);
    return {p.x + c * q.x - s * q.y, p.y + s * q.x + c * q.y};
}

[[nodiscard]] constexpr icp::Pose2 compose(const icp::Pose2& a,
                                           const icp::Pose2& b)
{
    const double c = std::cos(a.theta), s = std::sin(a.theta);
    return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y,
            a.theta + b.theta};
}

[[nodiscard]] constexpr icp::Pose2 inverse(const icp::Pose2& p)
{
    const double c = std::cos(p.theta), s = std::sin(p.theta);
    return {-(c * p.x + s * p.y), s * p.x - c * p.y, -p.theta};
}

// 相对位姿（增量观测）：theta 取最短角差
[[nodiscard]] inline icp::Pose2 delta(const icp::Pose2& from,
                                      const icp::Pose2& to)
{
    icp::Pose2 d = compose(inverse(from), to);
    d.theta = std::atan2(std::sin(d.theta), std::cos(d.theta));
    return d;
}

// 里程计缓冲：按时间戳插值取位姿。deskew 的"真值来源"。
class OdomBuffer {
public:
    void push(double t, icp::Pose2 p)
    {
        samples_.push_back({t, p});
    }
    [[nodiscard]] bool empty() const { return samples_.empty(); }
    [[nodiscard]] std::size_t size() const { return samples_.size(); }

    // t 时刻的里程计位姿。缓冲为空 / t 越界（钳位到端点）都有明确语义，
    // 返回 nullopt 的只有"根本没有里程计"——调用方决定怎么办。
    [[nodiscard]] std::optional<icp::Pose2> at(double t) const
    {
        if (samples_.empty()) return std::nullopt;
        if (t <= samples_.front().t) return samples_.front().pose;
        if (t >= samples_.back().t) return samples_.back().pose;
        // lower_bound：首个 t_i >= t
        const auto it = std::lower_bound(
            samples_.begin(), samples_.end(), t,
            [](const Sample& s, double v) { return s.t < v; });
        const auto& hi = *it;
        const auto& lo = *std::prev(it);
        const double span = hi.t - lo.t;
        const double alpha = span > 0 ? (t - lo.t) / span : 0.0;
        // 角度走最短路：大转向时两帧间差可能接近 ±π，直接 lerp 会绕远
        double dth = hi.pose.theta - lo.pose.theta;
        dth = std::atan2(std::sin(dth), std::cos(dth));
        return icp::Pose2{lo.pose.x + alpha * (hi.pose.x - lo.pose.x),
                          lo.pose.y + alpha * (hi.pose.y - lo.pose.y),
                          lo.pose.theta + alpha * dth};
    }

private:
    struct Sample {
        double t{};
        icp::Pose2 pose{};
    };
    std::vector<Sample> samples_;
};

}  // namespace lpl
