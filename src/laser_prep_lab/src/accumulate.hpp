#pragma once
// ============================================================================
// 多帧融合（运动补偿累积）。
//
// 动机：单帧 2D 扫描太稀——8m 墙上 0.5° 束距的点间隙 ~7cm，
// 一根柱子正面只有 2-3 个点。匹配器（尤其 point-to-line）要的是
// 结构密度。把最近 K 帧用各自位姿拼到一起，密度翻 K 倍，
// 且因为每帧都做过 deskew + 清洗 + 用了配准位姿，融合是
// "运动补偿过的"——不是把畸变云胡乱堆一起。
//
// 职责边界：只做融合，不做配准。位姿链由调用方维护
//（数据集入口用 icp::align；在线节点将用 odom+定位）。
// ============================================================================
#include <vector>

#include "polar.hpp"

namespace lpl {

struct AccumulatorParams {
    int max_scans{16};        // 融合窗口：16 帧 ≈ 2-4s，短到漂移可忽略
    double range_window{15.0};  // 距窗口内最新位姿超过此距离的点丢弃
};

// 环形缓冲：push(scan, pose_world) 进来的是"扫描起始帧里的点 +
// 该帧在世界系里的位姿"。cloud() 输出世界系融合云。
class ScanAccumulator {
public:
    explicit ScanAccumulator(const AccumulatorParams& p = {})
        : params_(p)
    {}

    void push(const icp::PointCloud2& scan, const icp::Pose2& pose_world)
    {
        entries_.push_back({scan, pose_world});
        if (static_cast<int>(entries_.size()) > params_.max_scans) {
            entries_.erase(entries_.begin());
        }
    }

    // 融合云（世界系）。裁掉离最新位姿太远的旧点——绕圈场景里
    // 8s 前在走廊另一头的点对"当前局部地图"只添乱。
    [[nodiscard]] icp::PointCloud2 cloud() const
    {
        icp::PointCloud2 out;
        if (entries_.empty()) return out;
        const icp::Pose2& latest = entries_.back().pose;
        for (const auto& e : entries_) {
            for (const auto& pt : e.scan) {
                const icp::Point2 w = transform(e.pose, pt);
                if (std::hypot(w.x - latest.x, w.y - latest.y) >
                    params_.range_window) {
                    continue;
                }
                out.push_back(w);
            }
        }
        return out;
    }

    [[nodiscard]] std::size_t size() const
    {
        return entries_.size();
    }
    void clear() { entries_.clear(); }

private:
    struct Entry {
        icp::PointCloud2 scan;
        icp::Pose2 pose;
    };
    AccumulatorParams params_;
    std::vector<Entry> entries_;
};

}  // namespace lpl
