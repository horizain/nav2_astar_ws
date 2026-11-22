#pragma once
// ============================================================================
// 自适应断裂聚类 + split-and-merge 线段提取。
//
// 聚类是后面两块（动态障碍判别、多帧融合）的基础设施：
// "哪些点属于同一个物理物体"是它们共同的第一个问题。
//
// 断裂准则（为什么不用固定阈值）：相邻束打到同一面平墙时，
// 点间距 ≈ r·Δθ——8m 远的墙比 1m 近的墙间距大 8 倍。固定阈值
// 要么把远墙剪碎、要么把近处两个不同物体粘成一团。自适应
// 准则 = 基础间隙 + 距离相关的伸展量（掠射墙面点间距更大，
// stretch 系数放宽容忍）。
// ============================================================================
#include <cmath>
#include <vector>

#include "polar.hpp"

namespace lpl {

struct ClusterParams {
    double base_break{0.06};  // 基础断裂距离 (m)：近场物体的最小分辨
    double stretch{3.0};      // 伸展系数：允许同表面束间距的几倍（掠射墙）
    int min_points{3};         // 小于此数的簇不输出（多为残余噪声）
};

struct Cluster {
    std::vector<int> beams;   // 束下标（保留与 PolarScan 的对应，动态
                               // 判别要用它回查"这簇在原扫描的哪里"）
    icp::PointCloud2 points;    // 对应点（传感器系或统一帧，与输入一致）
    icp::Point2 centroid{};
    [[nodiscard]] int size() const
    {
        return static_cast<int>(points.size());
    }
};

// points：按束序排好的点列（deskew 输出天然满足）；
// beams：与 points 一一对应的束下标（调用方从 PolarScan 过滤 NaN 时记录）。
// ang_step：相邻束角间隔 (rad)——断裂阈值 = base + stretch·r·ang_step，
// 没有它就无法区分"8m 远墙的正常束间距"和"近场两个物体的间隙"。
inline std::vector<Cluster> cluster_by_break(const icp::PointCloud2& points,
                                            const std::vector<int>& beams,
                                            double ang_step,
                                            const ClusterParams& p)
{
    std::vector<Cluster> out;
    if (points.empty() || points.size() != beams.size()) return out;
    Cluster cur;
    cur.points.push_back(points.front());
    cur.beams.push_back(beams.front());

    for (std::size_t k = 1; k < points.size(); ++k) {
        const icp::Point2& a = cur.points.back();
        const icp::Point2& b = points[k];
        const double d = std::hypot(b.x - a.x, b.y - a.y);
        // 自适应阈值：以近侧点的距离估"同表面束间距"量级
        const double r_near = std::min(std::hypot(a.x, a.y), std::hypot(b.x, b.y));
        const double allow = p.base_break + p.stretch * r_near * ang_step;
        if (d > allow) {
            if (cur.size() >= p.min_points) out.push_back(std::move(cur));
            cur = Cluster{};
        }
        cur.points.push_back(b);
        cur.beams.push_back(beams[k]);
    }
    if (cur.size() >= p.min_points) out.push_back(std::move(cur));

    // 质心
    for (auto& c : out) {
        double sx = 0, sy = 0;
        for (const auto& pt : c.points) {
            sx += pt.x;
            sy += pt.y;
        }
        c.centroid = {sx / c.points.size(), sy / c.points.size()};
    }
    return out;
}

// ── split-and-merge 线段提取 ──────────────────────────────
struct LineSegment {
    icp::Point2 a{}, b{};
    double residual{0.0};  // 点到线最大残差（拟合质量）
};

struct LineParams {
    double split_tol{0.05};  // 分裂阈值 (m)：点离弦超过就劈
    int min_points{4};       // 线段最小点数
    int max_depth{8};        // 递归深度上限（防病态输入栈溢出）
};

namespace detail {

// 对 [lo, hi] 递归：首尾连线做弦，最远点超差 → 从那里劈开
inline void split_lines(const std::vector<icp::Point2>& pts, int lo, int hi,
                        const LineParams& p, int depth,
                        std::vector<LineSegment>& out)
{
    if (hi - lo + 1 < p.min_points || depth > p.max_depth) return;
    const icp::Point2& a = pts[static_cast<std::size_t>(lo)];
    const icp::Point2& b = pts[static_cast<std::size_t>(hi)];
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double len = std::hypot(dx, dy);
    if (len < 1e-9) return;

    int far_i = -1;
    double far_d = 0.0;
    for (int i = lo + 1; i < hi; ++i) {
        // 点到弦的距离（叉积 / 弦长）
        const double d = std::abs(dx * (pts[static_cast<std::size_t>(i)].y - a.y) -
                                  dy * (pts[static_cast<std::size_t>(i)].x - a.x)) / len;
        if (d > far_d) {
            far_d = d;
            far_i = i;
        }
    }
    if (far_d <= p.split_tol) {
        // 全段贴弦 → 收线（residual 记录最大残差，供质量过滤）
        out.push_back({a, b, far_d});
        return;
    }
    split_lines(pts, lo, far_i, p, depth + 1, out);
    split_lines(pts, far_i, hi, p, depth + 1, out);
}

}  // namespace detail

// 一个簇 → 若干线段。聚类保证输入点列连通，
// 所以首尾连线做初始弦是安全的。
inline std::vector<LineSegment> extract_lines(const Cluster& c,
                                               const LineParams& p)
{
    std::vector<LineSegment> out;
    detail::split_lines(c.points, 0, static_cast<int>(c.points.size()) - 1, p,
                        0, out);
    return out;
}

}  // namespace lpl
