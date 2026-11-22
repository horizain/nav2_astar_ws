#include "icp.hpp"

#include <cmath>
#include <cstdlib>
#include <numbers>
#include <print>

namespace icp {
namespace {

// ICP_TRACE=1 时输出每轮迭代详情（调试用）
const bool g_trace = std::getenv("ICP_TRACE") != nullptr;

// Cramer 法则解 3×3 线性方程组 A·x = b，行列式接近奇异时返回 false
auto solve3x3(const double A[3][3], const double b[3], double x[3]) -> bool
{
    const double det =
        A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
        A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
        A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

    double scale{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            scale = std::max(scale, std::abs(A[i][j]));
        }
    }
    if (std::abs(det) <= 1e-14 * scale * scale * scale) {
        return false;
    }

    x[0] = (b[0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
            A[0][1] * (b[1] * A[2][2] - A[1][2] * b[2]) +
            A[0][2] * (b[1] * A[2][1] - A[1][1] * b[2])) / det;

    x[1] = (A[0][0] * (b[1] * A[2][2] - A[1][2] * b[2]) -
            b[0] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
            A[0][2] * (A[1][0] * b[2] - b[1] * A[2][0])) / det;

    x[2] = (A[0][0] * (A[1][1] * b[2] - b[1] * A[2][1]) -
            A[0][1] * (A[1][0] * b[2] - b[1] * A[2][0]) +
            b[0] * (A[1][0] * A[2][1] - A[1][1] * A[2][0])) / det;

    return std::isfinite(x[0]) && std::isfinite(x[1]) && std::isfinite(x[2]);
}

} // namespace

auto align(const PointCloud2& source,
           const PointCloud2& target,
           const Pose2& initial,
           const AlignParams& params) -> std::expected<AlignStats, AlignError>
{
    if (source.empty() || target.empty()) {
        return std::unexpected(AlignError::empty_input);
    }

    const KDTree tree{target};

    Pose2 pose = initial;
    AlignStats stats{};
    double prev_rms = std::numeric_limits<double>::infinity();
    int stall = 0;

    for (int iter = 0; iter < params.max_iterations; ++iter) {
        stats.iterations = iter + 1;
        // 自适应对应距离门限：随残差收窄。大初始误差时保持宽松，
        // 收敛后收紧到下限，把"视野盲区孤儿点"（源可见而参考无覆盖的区域，
        // 其最近邻落在别的墙上）挡在门外。
        const double gate = std::clamp(3.0 * prev_rms, params.min_correspondence_dist,
                                       params.max_correspondence_dist);
        const double max_d2 = gate * gate;
        const double c = std::cos(pose.theta);
        const double s = std::sin(pose.theta);

        // Gauss-Newton 正规方程：A·δ = b，未知量 δ = (dx, dy, dθ)
        double A[3][3]{{}};
        double b[3]{};
        double weight_sum{};
        double weighted_cost{};
        int valid{};

        for (const auto& p : source) {
            // 当前位姿下把源点变换到目标（世界）坐标系
            const Point2 pw{pose.x + c * p.x - s * p.y,
                            pose.y + s * p.x + c * p.y};

            const auto [q1, q2] = tree.find2(pw);
            if (q1 == nullptr) {
                continue;
            }

            const double dx = pw.x - q1->x;
            const double dy = pw.y - q1->y;
            const double d2 = dx * dx + dy * dy;
            if (d2 > max_d2) {
                continue; // 外点剔除
            }

            // 由 (最近点, 次近点) 估计局部直线，法向量 n 垂直于该直线
            Point2 n{};
            double r{};
            bool have_line{false};

            if (q2 != nullptr) {
                const double vx = q2->x - q1->x;
                const double vy = q2->y - q1->y;
                const double norm2 = vx * vx + vy * vy;
                const double norm = std::sqrt(norm2);
                // 次近点太远 → 大概率跨墙连线（墙角/稀疏区），不做 point-to-line
                if (norm > 1e-12 && norm <= params.max_line_len) {
                    // 投影参数 t ∈ [0,1] 表示 p 投影落在 [q1,q2] 线段内。
                    // 线段外（如柱子右侧面点投影到底面线段的延长线上）→
                    // 该直线不代表 p 处的局部表面 → 不采用
                    const double t = (dx * vx + dy * vy) / norm2;
                    if (t >= -0.25 && t <= 1.25) {
                        n = {.x = -vy / norm, .y = vx / norm};
                        r = n.x * dx + n.y * dy;
                        have_line = true;
                    }
                }
            }
            if (!have_line) {
                // 线段检验失败的匹配（如柱子跨面）直接丢弃——plICP 同款做法。
                // 合法覆盖边缘匹配的点到点残差本来就 ≈0，不提供约束；
                // 仅当目标只有单点（退化目标）时保留 Point-to-Point。
                if (q2 != nullptr) {
                    continue;
                }
                const double dist = std::sqrt(d2);
                if (dist < 1e-12) {
                    continue;
                }
                n = {.x = dx / dist, .y = dy / dist};
                r = dist;
            }

            // Huber 鲁棒核：小残差全额保留，大残差线性降权
            const double ar = std::abs(r);
            const double w = ar <= params.huber_delta
                               ? 1.0
                               : params.huber_delta / ar;

            // 残差 r 对 (dx, dy, dθ) 的雅可比：
            //   ∂pw/∂dx = (1,0)   ∂pw/∂dy = (0,1)
            //   ∂pw/∂θ  = (-s·px - c·py, c·px - s·py)
            const double j[3] = {n.x,
                                 n.y,
                                 n.x * (-s * p.x - c * p.y) +
                                     n.y * (c * p.x - s * p.y)};

            for (int i = 0; i < 3; ++i) {
                b[i] -= w * j[i] * r;
                for (int k = i; k < 3; ++k) {
                    A[i][k] += w * j[i] * j[k];
                }
            }
            A[1][0] = A[0][1];
            A[2][0] = A[0][2];
            A[2][1] = A[1][2];

            weight_sum += w;
            weighted_cost += w * r * r;
            ++valid;
        }

        if (valid == 0 || weight_sum == 0.0) {
            if (g_trace) {
                std::println(stderr, "[ref ] it{:>3} FAIL 无有效对应点", iter + 1);
            }
            return std::unexpected(AlignError::no_correspondence);
        }

        double delta[3]{};
        if (!solve3x3(A, b, delta)) {
            if (g_trace) {
                std::println(stderr, "[ref ] it{:>3} FAIL 正规方程奇异", iter + 1);
            }
            return std::unexpected(AlignError::degenerate_system);
        }

        pose.x += delta[0];
        pose.y += delta[1];
        pose.theta = std::remainder(pose.theta + delta[2],
                                    2.0 * std::numbers::pi);

        stats.pose = pose;
        stats.correspondences = valid;
        const double rms = std::sqrt(weighted_cost / weight_sum);
        // RMS 停滞计数：噪声底部振荡时的次级收敛判据（相对变化 < 5%）
        stall = (std::abs(rms - prev_rms) <= 0.05 * rms) ? stall + 1 : 0;
        prev_rms = rms;
        stats.rms_error = rms;

        if (g_trace) {
            std::println(stderr,
                         "[ref ] it{:>3} corr={:>4} rms={:.4f} pose=({:.4f},{:.4f},{:.5f})",
                         iter + 1, valid, stats.rms_error, pose.x, pose.y, pose.theta);
        }

        // 位姿增量足够小、或 RMS 连续停滞（噪声底部振荡）即认为收敛
        if (std::hypot(delta[0], delta[1], delta[2]) < params.tolerance ||
            (params.stall_iters > 0 && stall >= params.stall_iters)) {
            stats.converged = true;
            break;
        }
    }

    return stats;
}

} // namespace icp
