#pragma once
// ============================================================================
// 动态障碍剔除——"这簇点是人还是墙"。
//
// 判据用的是最朴素的物理直觉：如果一簇点现在挡在某个位置，
// 而之前的扫描曾经**透视**过这个位置（射线穿过、没打到东西），
// 那它一定是后来出现的 → 动态。墙永远不会被透视：射线打到墙就停，
// 墙所在的格子只积累"占据"或"未知"，不会变"自由"。
//
// 实现就是一个只记"自由证据"的 log-odds 栅格（FreeSpaceGrid）：
//   每束激光的沿途格 += kFree（hit 格不记占据——本模块不需要它，
//   需要"这里曾经空过"，占据证据由墙自己"从未被透视"来表达）。
//   新一帧的簇若有足量格子处于"强自由"状态 → 判动态。
//
// 诚实边界：
//   - 判据是"曾经空过"，所以"从扫描开始就站在那的人"会被当成静态。
//     修复需要"多帧位置不一致"的轨迹级判据（跟踪模块的职责）。
//   - 位姿链漂移会让"透视记忆"错位：漂过一格，墙边会被误判。
//     所以窗口必须短（默认 8s），且位姿来自短期一致的 ICP 链。
// ============================================================================
#include <cmath>
#include <vector>

#include "cluster.hpp"
#include "polar.hpp"

namespace lpl {

// 简单定长世界栅格（不需要在线生长：动态检测只看机器人附近，
// 调用方给一个罩住活动范围的窗口即可，省掉生长逻辑这整块复杂度）。
class FreeSpaceGrid {
public:
    FreeSpaceGrid(double resolution, double half_size)
        : res_(resolution > 0 ? resolution : 0.05),
          half_(half_size > 0 ? half_size : 20.0),
          n_(static_cast<int>(2.0 * half_ / res_)),
          cells_(static_cast<std::size_t>(n_) * n_, 0.0f)
    {}

    // 一条射线：沿途格积累"自由"证据（不记 hit——见文件头说明）。
    // sensor/end 都是世界坐标，原点在窗口中心。
    // 关键细节：端点格周围 1 格（Chebyshev）不许记 free——掠射几何
    // 下 ±2cm 的测距噪声会让射线"穿透"墙格，把墙自己的格子记成
    // 自由证据，然后墙被自己的透视记忆误判成动态（真实踩坑：
    // Intel 数据上同一面远墙逐帧被标记，方向固定复现）。
    void ray_free(const icp::Point2& sensor, const icp::Point2& end)
    {
        const double dist = std::hypot(end.x - sensor.x, end.y - sensor.y);
        int ecx = 0, ecy = 0;
        // 出界也没关系：ecx/ecy 只用于跳过判定（出界时全程都不会匹配）
        (void)world_to_cell(end, ecx, ecy);
        const int steps = std::max(1, static_cast<int>(dist / res_));
        for (int i = 0; i < steps; ++i) {
            const double t = static_cast<double>(i) / steps;
            const icp::Point2 m{sensor.x + t * (end.x - sensor.x),
                                sensor.y + t * (end.y - sensor.y)};
            int cx = 0, cy = 0;
            if (!world_to_cell(m, cx, cy)) continue;
            if (std::abs(cx - ecx) <= 1 && std::abs(cy - ecy) <= 1) continue;
            cell(cx, cy) += kFree;
        }
    }

    // 一簇点里"此前被透视过"的比例
    [[nodiscard]] double seen_through_ratio(const icp::PointCloud2& pts) const
    {
        if (pts.empty()) return 0.0;
        int free_cells = 0;
        for (const auto& pt : pts) {
            int cx = 0, cy = 0;
            if (world_to_cell(pt, cx, cy) && cell(cx, cy) <= kFreeThreshold) {
                ++free_cells;
            }
        }
        return static_cast<double>(free_cells) / static_cast<double>(pts.size());
    }

    // 手动把一块区域标记为"已知自由"（测试与初始化用）
    void mark_free(const icp::Point2& p)
    {
        int cx = 0, cy = 0;
        if (world_to_cell(p, cx, cy)) cell(cx, cy) = -10.0f;
    }

    // 记忆衰减：所有自由证据 × factor（向 0 = 遗忘）。
    // 透视记忆必须有时效——位姿链的短时漂移会让旧证据错位，
    // 而且环境本身会变（停过的车开走了）。不衰减的版本在 Intel
    // 全程跑出了 5.4 个"动态簇"/帧的虚警（真实踩坑）。
    void decay(float factor)
    {
        for (auto& c : cells_) c *= factor;
    }

private:
    static constexpr float kFree = -0.4f;          // 一条射线的自由证据
    static constexpr float kFreeThreshold = -1.0f; // ≤ 此值 = "被透视过"

    [[nodiscard]] bool world_to_cell(const icp::Point2& p, int& cx,
                                     int& cy) const
    {
        const double fx = (p.x + half_) / res_;
        const double fy = (p.y + half_) / res_;
        cx = static_cast<int>(std::floor(fx));
        cy = static_cast<int>(std::floor(fy));
        return cx >= 0 && cy >= 0 && cx < n_ && cy < n_;
    }
    [[nodiscard]] float& cell(int cx, int cy)
    {
        return cells_[static_cast<std::size_t>(cy) * n_ + cx];
    }
    [[nodiscard]] float cell(int cx, int cy) const
    {
        return cells_[static_cast<std::size_t>(cy) * n_ + cx];
    }

    double res_;
    double half_;
    int n_;
    std::vector<float> cells_;
};

struct DynamicParams {
    double seen_through{0.6};  // 簇内 ≥60% 的格子此前被透视 → 动态
    int min_cluster_points{3};
    // 自由证据栅格的窗口半径 (m)：罩住短期活动范围即可
    double grid_half{20.0};
    double grid_res{0.15};
    // 记忆衰减系数（每帧 ×decay）：0.99 ≈ 半衰期 69 帧 ≈ 26s@Intel 帧率。
    // 透视记忆的时效上限 = 位姿链的短时一致性，给长了反而虚警。
    float decay{0.98f};
};

struct DynamicVerdict {
    bool is_dynamic{false};
    double seen_through{0.0};  // 诊断：透视比例
};

// 前置：本帧各簇已经在世界系（points 由 pose 变换得到）。
inline std::vector<DynamicVerdict> judge_clusters(
    const std::vector<Cluster>& world_clusters, const FreeSpaceGrid& grid,
    const DynamicParams& p)
{
    std::vector<DynamicVerdict> out;
    out.reserve(world_clusters.size());
    for (const auto& c : world_clusters) {
        DynamicVerdict v;
        v.seen_through = grid.seen_through_ratio(c.points);
        v.is_dynamic = c.size() >= p.min_cluster_points &&
                       v.seen_through >= p.seen_through;
        out.push_back(v);
    }
    return out;
}

// 本帧观察完，把它的自由空间证据记进栅格（先判后记：
// 当前帧自己不能给当前帧作证）。
inline void record_frame(const icp::Pose2& pose, const icp::PointCloud2& scan,
                         FreeSpaceGrid& grid)
{
    const icp::Point2 sensor{pose.x, pose.y};
    for (const auto& pt : scan) {
        grid.ray_free(sensor, transform(pose, pt));
    }
}

}  // namespace lpl
