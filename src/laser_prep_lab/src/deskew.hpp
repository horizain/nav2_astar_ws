#pragma once
// ============================================================================
// 运动畸变校正（deskew / 去运动畸变）——2D 激光最经典的工程难题。
//
// 病灶：一帧扫描不是瞬时拍的。SICK 一圈 ~25ms、国产雷达 ~50-100ms，
// 机器人 1m/s 时一圈内已经挪了 2.5-10cm。第 i 束激光的回波点
// 属于"雷达在 p(t_i) 时刻的位置"，却被默认当成都在 p(t_0)——
// 于是直墙变斜墙、拐角拖尾巴、门口变宽。地图上表现为墙"发糊"。
//
// 药方：每束按自己的时刻查里程计位姿，把点变换到统一基准帧。
// 这里输出基准 = 扫描起始帧（p(t_0)）——下游 ICP 把整帧当刚体处理，
// 基准必须是"某一时刻"而不是"平均时刻"（那样每帧的基准都不同，
// 位姿链没法递推）。
//
// 诚实边界：deskew 的精度上限 = 里程计的短时精度。轮式里程计在
// 打滑/加减速时本身就有误差，deskew 只能继承它，不能修复它。
// 没有 odom 时唯一诚实的做法是不做校正（返回未修正点），
// 而不是假装机器人没动。
// ============================================================================
#include "polar.hpp"

namespace lpl {

struct DeskewResult {
    icp::PointCloud2 points;   // 全部在扫描起始帧 p(t_0)
    bool corrected{false};     // false = 无 odom/beam_dt，原样转点
    // 供诊断：这一帧的时间跨度内机器人挪了多远（不校正的畸变量级）
    double raw_skew_m{0.0};
};

inline DeskewResult deskew(const PolarScan& s, const OdomBuffer& odom)
{
    DeskewResult out;
    const int n = s.size();
    if (n == 0) return out;

    // 前置条件不满足：诚实回退（不猜、不装没动——调用方看 corrected）
    // beam_dt 允许负值：时间戳是"扫描结束"时，束时间应该向后跨
    // （CARMEN FLASER 的约定之争——评估工具扫出正向假设有害后补的）
    const bool can_correct = s.beam_dt != 0.0 && !odom.empty();
    if (!can_correct) {
        out.points = s.endpoints();
        return out;
    }

    // 基准位姿：扫描起始时刻的里程计
    const icp::Pose2 p0 = *odom.at(s.stamp);
    const icp::Pose2 p_last = *odom.at(s.time(n - 1));
    out.raw_skew_m = std::hypot(p_last.x - p0.x, p_last.y - p0.y);
    out.corrected = true;

    const icp::Pose2 p0_inv = inverse(p0);
    out.points.reserve(s.range.size());
    for (int i = 0; i < n; ++i) {
        if (!s.valid(i)) continue;
        const double a = s.angle(i);
        const icp::Point2 beam{s.range[static_cast<std::size_t>(i)] * std::cos(a),
                               s.range[static_cast<std::size_t>(i)] * std::sin(a)};
        // 点的世界位置 = 该束时刻的里程计位姿作用在束方向上
        const icp::Pose2 pi = *odom.at(s.time(i));
        const icp::Point2 world = transform(pi, beam);
        // 拉回扫描起始帧
        out.points.push_back(transform(p0_inv, world));
    }
    return out;
}

}  // namespace lpl
