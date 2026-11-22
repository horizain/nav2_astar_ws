#pragma once

#include <algorithm>
#include <cstdint>
#include <expected>
#include <limits>
#include <span>
#include <string_view>
#include <vector>

namespace icp {

// ── 基础类型 ──────────────────────────────────────────────

struct Point2 {
    double x{};
    double y{};
};

using PointCloud2 = std::vector<Point2>;

struct Pose2 {
    double x{};
    double y{};
    double theta{};
};

// ── KD-tree 最近邻搜索 ───────────────────────────────────
// O(n log n) 构建，平均 O(log n) 查询。
// 一次查询同时返回最近点和次近点——次近点用于 Point-to-Line
// 的局部直线估计，无需额外开销。

class KDTree {
public:
    explicit KDTree(const PointCloud2& points)
    {
        nodes_.reserve(points.size());
        PointCloud2 pts{points};
        root_ = build(std::span{pts});
    }

    struct Neighbors2 {
        const Point2* nearest{};
        const Point2* second{};
    };

    [[nodiscard]] auto find2(const Point2& q) const -> Neighbors2
    {
        Best2 best{};
        search(root_, q, best);
        return {.nearest = best.p1, .second = best.p2};
    }

private:
    struct Node {
        Point2 pt;
        int32_t left{-1};
        int32_t right{-1};
        uint8_t axis{};
    };

    struct Best2 {
        double d1{std::numeric_limits<double>::infinity()};
        double d2{std::numeric_limits<double>::infinity()};
        const Point2* p1{};
        const Point2* p2{};
    };

    // 递归建树：按伸展量更大的轴取中位数分割，保证树平衡
    auto build(std::span<Point2> pts) -> int32_t
    {
        if (pts.empty()) {
            return -1;
        }

        double minx{pts.front().x}, maxx{minx};
        double miny{pts.front().y}, maxy{miny};
        for (const auto& p : pts) {
            minx = std::min(minx, p.x);
            maxx = std::max(maxx, p.x);
            miny = std::min(miny, p.y);
            maxy = std::max(maxy, p.y);
        }
        const uint8_t axis = (maxx - minx) >= (maxy - miny) ? 0 : 1;

        const auto mid = static_cast<std::ptrdiff_t>(pts.size() / 2);
        std::ranges::nth_element(
            pts, pts.begin() + mid,
            [axis](const Point2& a, const Point2& b) {
                return axis == 0 ? a.x < b.x : a.y < b.y;
            });

        const int32_t idx = static_cast<int32_t>(nodes_.size());
        nodes_.push_back({.pt = pts[static_cast<std::size_t>(mid)], .axis = axis});

        // 递归可能触发 vector 扩容，因此只按索引回填，不持有引用
        const int32_t l = build(pts.first(mid));
        const int32_t r = build(pts.subspan(mid + 1));
        nodes_[static_cast<std::size_t>(idx)].left = l;
        nodes_[static_cast<std::size_t>(idx)].right = r;
        return idx;
    }

    // 带剪枝的 DFS：只有分割面距离 < 当前次近距离时才访问另一侧子树
    void search(int32_t idx, const Point2& q, Best2& best) const
    {
        if (idx < 0) {
            return;
        }
        const Node& nd = nodes_[static_cast<std::size_t>(idx)];

        const double dx = q.x - nd.pt.x;
        const double dy = q.y - nd.pt.y;
        const double d2 = dx * dx + dy * dy;

        if (d2 < best.d1) {
            best.d2 = best.d1;
            best.p2 = best.p1;
            best.d1 = d2;
            best.p1 = &nd.pt;
        } else if (d2 < best.d2) {
            best.d2 = d2;
            best.p2 = &nd.pt;
        }

        const double diff = nd.axis == 0 ? dx : dy;
        const auto near = diff < 0.0 ? nd.left : nd.right;
        const auto far = diff < 0.0 ? nd.right : nd.left;

        search(near, q, best);
        if (diff * diff < best.d2) {
            search(far, q, best);
        }
    }

    std::vector<Node> nodes_;
    int32_t root_{-1};
};

// ── ICP 配准 ─────────────────────────────────────────────

struct AlignParams {
    int max_iterations{50};              // 最大迭代次数
    double tolerance{1e-6};              // 位姿增量范数收敛阈值
    double max_correspondence_dist{2.0}; // 对应距离门限上限 (m)——首轮/远离时用
    double min_correspondence_dist{0.25}; // 门限下限 (m)——收敛后剔除视野盲区孤儿点
                                         // 与细结构（如 0.5m 柱子）的跨面伪匹配
    double max_line_len{0.25};           // 次近点距最近点超过此值不做 point-to-line
                                         // （防止墙角处两点跨墙连线出错误法向量）
    double huber_delta{0.2};             // Huber 鲁棒核阈值 (m)
    int stall_iters{2};                  // RMS 停滞判据：连续 N 轮视为收敛（0=禁用）
};

struct AlignStats {
    Pose2 pose{};          // 配准结果：T(source) ≈ target
    int iterations{};      // 实际迭代次数
    int correspondences{}; // 最后一轮有效对应点数
    double rms_error{};    // 最后一轮加权 RMS 残差 (m)
    bool converged{};      // 是否在最大迭代次数内收敛
};

enum class AlignError {
    empty_input,       // 输入点云为空
    no_correspondence, // 一轮迭代中没有任何有效对应点（初始位姿太远？）
    degenerate_system, // 正规方程奇异（几何退化，如所有点共线）
};

[[nodiscard]] constexpr auto to_string(AlignError error) -> std::string_view
{
    switch (error) {
    case AlignError::empty_input:       return "输入点云为空";
    case AlignError::no_correspondence: return "无有效对应点";
    case AlignError::degenerate_system: return "正规方程奇异（几何退化）";
    }
    return "未知错误";
}

// Point-to-Line ICP：求位姿 T 使 T(source) 与 target 对齐。
// source —— 传感器坐标系下的当前扫描；
// target —— 世界坐标系下的参考扫描（在其上构建 KD-tree）。
[[nodiscard]] auto align(const PointCloud2& source,
                         const PointCloud2& target,
                         const Pose2& initial = {},
                         const AlignParams& params = {})
    -> std::expected<AlignStats, AlignError>;

} // namespace icp
