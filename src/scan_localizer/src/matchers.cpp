// ============================================================================
// 匹配器实现
// ============================================================================
#include "scan_localizer/matchers.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <queue>
#include <string>

namespace sloc {

// ---------- 公共工具 ----------
namespace {

// 位姿 → 世界点集
inline std::vector<Vec2> to_world(const Pose2& p, const Scan& scan) {
    const double c = std::cos(p.theta), s = std::sin(p.theta);
    std::vector<Vec2> w;
    w.reserve(scan.points.size());
    for (const auto& pt : scan.points) {
        w.emplace_back(p.x + c * pt.x() - s * pt.y(),
                       p.y + s * pt.x() + c * pt.y());
    }
    return w;
}

// 平均似然 + 内点率
inline void eval_stats(const MapField& f, const Pose2& p, const Scan& scan,
                       double inlier_gate, double& score,
                       double& inlier_ratio) {
    const auto w = to_world(p, scan);
    double s = 0.0, inl = 0.0;
    const int n = static_cast<int>(w.size());
    for (const auto& q : w) {
        const auto g = f.grid_->worldToGridF(q.x(), q.y());
        s += f.likelihoodAtF(g.x(), g.y());
        inl += (std::abs(f.sdf(g.x(), g.y())) < inlier_gate) ? 1.0 : 0.0;
    }
    score = n > 0 ? s / n : 0.0;
    inlier_ratio = n > 0 ? inl / n : 0.0;
}

// H⁻¹ 保守膨胀后的协方差
// H⁻¹ 保守膨胀后的协方差（含物理下限：扫描匹配不得宣称亚毫米精度）
Cov3 cov_from_hessian(const Eigen::Matrix3d& H, double res_var,
                       double min_var_xy = 1e-4,  // (1cm)²
                       double min_var_th = 1.2e-5,  // (0.2°)²
                       double inflation = 2.0) {
    Eigen::Matrix3d Hs = H;
    // 对角加载防奇异（走廊退化方向自动膨胀）
    Hs.diagonal().array() += 1e-6;
    Cov3 cov = std::max(res_var, min_var_xy) * Hs.inverse();
    cov = 0.5 * (cov + cov.transpose()).eval();
    cov = cov * inflation;
    cov(0, 0) = std::max(cov(0, 0), min_var_xy);
    cov(1, 1) = std::max(cov(1, 1), min_var_xy);
    cov(2, 2) = std::max(cov(2, 2), min_var_th);
    return cov;
}

}  // namespace

// ============================================================================
// CorrelativeMatcher —— Olson 窗口穷举（粗+细两级），协方差取响应面二阶矩
// ============================================================================
MatchResult CorrelativeMatcher::match(const MapField& field, const Pose2& prior,
                                      const Scan& scan,
                                      const MatchOptions& opt) const {
    MatchResult res;
    const auto* grid = field.grid_;
    const int n = static_cast<int>(scan.points.size());
    if (n == 0) { return res; }

    // 粗搜 + 细搜两轮（细轮在粗最优 ± 一个粗步长内以 1/3 步长重扫）
    double best_s = -1.0;
    double best_x = prior.x, best_y = prior.y, best_th = prior.theta;
    std::vector<double> coarse_scores;   // 响应面（协方差用）
    std::vector<std::array<double, 4>> coarse_poses;  // (s,x,y,θ)

    for (int round = 0; round < 2; ++round) {
        const double lin_step =
            round == 0 ? opt.step_lin : opt.step_lin / 3.0;
        const double ang_step =
            round == 0 ? opt.step_ang : opt.step_ang / 3.0;
        const double wlin = round == 0 ? opt.window_lin
                                       : opt.step_lin;  // 细搜只搜粗最优附近
        const double wang = round == 0 ? opt.window_ang : opt.step_ang;
        const int n_x = static_cast<int>(
            std::max(1.0, std::ceil(2.0 * wlin / lin_step)) + 1);
        const int n_y = n_x;
        const int n_a = static_cast<int>(
            std::max(1.0, std::ceil(2.0 * wang / ang_step)) + 1);
        const double cx = round == 0 ? prior.x : best_x;
        const double cy = round == 0 ? prior.y : best_y;
        const double cth = round == 0 ? prior.theta : best_th;
        coarse_poses.clear();
        coarse_scores.clear();

        for (int ai = 0; ai < n_a; ++ai) {
            const double th =
                cth - 0.5 * (n_a - 1) * ang_step + ai * ang_step;
            const double c = std::cos(th), s = std::sin(th);
            // 找到该角度的预旋转索引（粗轮直接重算，细轮用最近邻）
            std::vector<Eigen::Vector2d> cells;
            cells.reserve(scan.points.size());
            for (const auto& p : scan.points) {
                const double wx = prior.x + c * p.x() - s * p.y();
                const double wy = prior.y + s * p.x() + c * p.y();
                cells.emplace_back((wx - grid->origin_x) / grid->resolution,
                                   (wy - grid->origin_y) / grid->resolution);
            }
            for (int yi = 0; yi < n_y; ++yi) {
                for (int xi = 0; xi < n_x; ++xi) {
                    const double dx =
                        (cx - prior.x) + (xi - 0.5 * (n_x - 1)) * lin_step;
                    const double dy =
                        (cy - prior.y) + (yi - 0.5 * (n_y - 1)) * lin_step;
                    const double offx = dx / grid->resolution;
                    const double offy = dy / grid->resolution;
                    double sc = 0.0;
                    for (const auto& cel : cells) {
                        sc += field.likelihood(
                            static_cast<int>(std::lround(cel.x() + offx)),
                            static_cast<int>(std::lround(cel.y() + offy)));
                    }
                    sc /= n;
                    if (round == 0) {
                        coarse_poses.push_back({sc, cx + dx, cy + dy, th});
                    }
                    if (sc > best_s) {
                        best_s = sc;
                        best_x = cx + dx;
                        best_y = cy + dy;
                        best_th = th;
                    }
                }
            }
        }
    }

    res.pose = {best_x, best_y, best_th};
    // 响应面二阶矩协方差（karto 思路：软最大加权，以粗轮最优为参考）
    if (!coarse_poses.empty()) {
        // 粗轮最优响应（细轮可能更高，用它做权重会整体塌缩）
        double smax_c = -1.0;
        for (const auto& q : coarse_poses) { smax_c = std::max(smax_c, q[0]); }
        double wsum = 0.0;
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        Eigen::Matrix3d m2 = Eigen::Matrix3d::Zero();
        for (const auto& q : coarse_poses) {
            const double w = std::exp((q[0] - smax_c) * 25.0);
            if (w < 1e-6) { continue; }
            const Eigen::Vector3d v(q[1] - best_x, q[2] - best_y,
                                    q[3] - best_th);
            wsum += w;
            mean += w * v;
            m2 += w * v * v.transpose();
        }
        if (wsum > 1e-9) {
            mean /= wsum;
            m2 /= wsum;
            const Eigen::Matrix3d cov = m2 - mean * mean.transpose();
            // 3σ 下限，防病态过紧
            res.covariance(0, 0) = std::max(cov(0, 0), 9e-6);
            res.covariance(1, 1) = std::max(cov(1, 1), 9e-6);
            res.covariance(2, 2) = std::max(cov(2, 2), 9e-6);
            res.covariance(0, 1) = cov(0, 1);
            res.covariance(1, 0) = cov(0, 1);
        }
    }
    eval_stats(field, res.pose, scan, opt.inlier_gate, res.score,
               res.inlier_ratio);
    res.valid = best_s > 0.0;
    return res;
}

// ============================================================================
// FieldGnMatcher —— SDF 连续场 GN（点-线 ICP 等价，零数据关联）
// ============================================================================
MatchResult FieldGnMatcher::match(const MapField& field, const Pose2& prior,
                                  const Scan& scan,
                                  const MatchOptions& opt) const {
    MatchResult res;
    const auto* grid = field.grid_;
    const int n = static_cast<int>(scan.points.size());
    if (n < 5) { return res; }

    Pose2 p = prior;
    double best_score = -1.0;
    Pose2 best_pose = prior;
    Eigen::Matrix3d best_H = Eigen::Matrix3d::Identity() * 1e-6;
    double best_var = 1e-6;
    for (int it = 0; it < opt.max_iterations; ++it) {
        const double c = std::cos(p.theta), s = std::sin(p.theta);
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();
        double rsum = 0.0, rw = 0.0;
        int used = 0;
        for (const auto& pt : scan.points) {
            const double lx = pt.x(), ly = pt.y();
            const double wx = p.x + c * lx - s * ly;
            const double wy = p.y + s * lx + c * ly;
            const auto g = grid->worldToGridF(wx, wy);
            Eigen::Vector2d grad;
            const double r = field.sdf(g.x(), g.y(), &grad);
            if (std::abs(r) > opt.trim_dist) { continue; }
            // Huber
            const double ar = std::abs(r);
            const double w = ar <= opt.huber_delta
                                 ? 1.0
                                 : opt.huber_delta / std::max(ar, 1e-9);
            // ∂w/∂p = [gx, gy, gx·(-(wy-py)) + gy·(wx-px)]
            const double jth = grad.x() * (-(wy - p.y)) +
                               grad.y() * (wx - p.x);
            Eigen::Vector3d J(grad.x(), grad.y(), jth);
            H += w * J * J.transpose();
            b += w * J * r;
            rsum += w * r * r;
            rw += w;
            ++used;
        }
        if (used < 5) { break; }
        // LM 阻尼只用于解步；协方差用未阻尼 Hessian（退化方向如实膨胀）
        Eigen::Matrix3d Hd = H;
        Hd.diagonal().array() += 1e-3 * H.diagonal().maxCoeff() + 1e-9;
        const Eigen::Vector3d dp = Hd.ldlt().solve(-b);
        if (!dp.allFinite()) { break; }
        // 步长护栏：单次迭代 ≤0.2m / 0.1rad
        const double nl = dp.head<2>().norm();
        const double s_clip =
            std::min(1.0, std::min(0.2 / std::max(nl, 1e-9),
                                   0.1 / std::max(std::abs(dp(2)), 1e-9)));
        p.x += s_clip * dp(0);
        p.y += s_clip * dp(1);
        p.theta += s_clip * dp(2);
        // 记录残差最小点（防发散：残差单调性破坏时回退）
        const double ravg = rsum / std::max(rw, 1.0);
        if (ravg < best_score || best_score < 0.0) {
            best_score = ravg;
            best_pose = p;
            best_H = H;              // 协方差取残差最优点处的 Hessian
            best_var = std::max(ravg, 1e-6);
        }
        if (nl * s_clip < opt.converg_t &&
            std::abs(dp(2)) * s_clip < opt.converg_r) {
            break;
        }
    }
    res.pose = best_pose;
    res.covariance = cov_from_hessian(best_H, best_var);
    eval_stats(field, p, scan, opt.inlier_gate, res.score, res.inlier_ratio);
    res.valid = true;
    return res;
}

// ============================================================================
// IcpP2pMatcher —— 经典点到点 ICP（SDF 下降找对应）
// ============================================================================
MatchResult IcpP2pMatcher::match(const MapField& field, const Pose2& prior,
                                  const Scan& scan,
                                  const MatchOptions& opt) const {
    MatchResult res;
    const auto* grid = field.grid_;
    const int n = static_cast<int>(scan.points.size());
    if (n < 5) { return res; }

    Pose2 p = prior;
    for (int it = 0; it < opt.max_iterations; ++it) {
        const double c = std::cos(p.theta), s = std::sin(p.theta);
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();
        int used = 0;
        double rsum = 0.0;
        for (const auto& pt : scan.points) {
            const double wx = p.x + c * pt.x() - s * pt.y();
            const double wy = p.y + s * pt.x() + c * pt.y();
            const auto g = grid->worldToGridF(wx, wy);
            const auto ob = field.descendToObstacle(
                static_cast<int>(std::lround(g.x())),
                static_cast<int>(std::lround(g.y())));
            const auto qw = grid->gridToWorld(ob.x(), ob.y());
            const double ex = wx - qw.x(), ey = wy - qw.y();
            const double e2 = ex * ex + ey * ey;
            if (e2 > opt.trim_dist * opt.trim_dist) { continue; }
            const double e = std::sqrt(e2);
            const double w = e <= opt.huber_delta
                                 ? 1.0
                                 : opt.huber_delta / std::max(e, 1e-9);
            Eigen::Matrix<double, 2, 3> J;
            J << 1.0, 0.0, -(wy - qw.y()), 0.0, 1.0, (wx - qw.x());
            H += w * J.transpose() * J;
            b += w * J.transpose() * Eigen::Vector2d(ex, ey);
            rsum += w * e2;
            ++used;
        }
        if (used < 5) { break; }
        H.diagonal().array() += 1e-9;
        const Eigen::Vector3d dp = H.ldlt().solve(-b);
        p.x += dp(0);
        p.y += dp(1);
        p.theta += dp(2);
        if (dp.head<2>().norm() < opt.converg_t &&
            std::abs(dp(2)) < opt.converg_r) {
            break;
        }
        if (it == opt.max_iterations - 1) {
            res.covariance =
                cov_from_hessian(H, std::max(rsum / used, 1e-6));
        }
    }
    res.pose = p;
    eval_stats(field, p, scan, opt.inlier_gate, res.score, res.inlier_ratio);
    res.valid = true;
    return res;
}

// ============================================================================
// NdtMatcher —— 2D NDT（体素高斯，D2P 分数最大化）
// ============================================================================
void NdtMatcher::build(const OccupancyGrid& grid, double voxel_m) {
    voxel_m_ = voxel_m;
    voxel_grid_ = grid;  // 复用分辨率字段存体素划分
    voxel_grid_.resolution = voxel_m;
    const int vw = static_cast<int>(
        std::ceil(grid.width * grid.resolution / voxel_m));
    const int vh = static_cast<int>(
        std::ceil(grid.height * grid.resolution / voxel_m));
    voxel_grid_.width = vw;
    voxel_grid_.height = vh;

    std::vector<std::vector<Eigen::Vector2d>> buckets(
        static_cast<std::size_t>(vw) * vh);
    for (int y = 0; y < grid.height; ++y) {
        for (int x = 0; x < grid.width; ++x) {
            if (!grid.occupied(x, y)) { continue; }
            const auto w = grid.gridToWorld(x, y);
            const int vx = std::min(
                vw - 1,
                static_cast<int>((w.x() - grid.origin_x) / voxel_m));
            const int vy = std::min(
                vh - 1,
                static_cast<int>((w.y() - grid.origin_y) / voxel_m));
            buckets[static_cast<std::size_t>(vy) * vw + vx].push_back(w);
        }
    }
    gauss_.clear();
    for (auto& b : buckets) {
        if (b.size() < 3) { continue; }
        Gauss g;
        g.mu = Eigen::Vector2d::Zero();
        for (const auto& q : b) { g.mu += q; }
        g.mu /= double(b.size());
        Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
        for (const auto& q : b) {
            const Eigen::Vector2d d = q - g.mu;
            cov += d * d.transpose();
        }
        cov /= double(b.size() - 1);
        // 正则化：保证正定（NDT 惯例 λ=0.001·voxel²）
        cov.diagonal().array() += 0.001 * voxel_m * voxel_m;
        g.cov = cov;
        g.cov_inv = cov.inverse();
        gauss_.push_back(g);
        // 登记体素索引（挂在 bucket 位置上）
    }
    // 体素 → 高斯映射表
    voxel_gauss_idx_.assign(buckets.size(), -1);
    {
        std::size_t gi = 0;
        for (std::size_t bi = 0; bi < buckets.size(); ++bi) {
            if (buckets[bi].size() >= 3) {
                voxel_gauss_idx_[bi] = static_cast<int>(gi++);
            }
        }
    }
}

MatchResult NdtMatcher::match(const MapField& field, const Pose2& prior,
                              const Scan& scan,
                              const MatchOptions& opt) const {
    MatchResult res;
    const auto* grid = field.grid_;
    if (gauss_.empty()) { return res; }
    const double cell_r = 2.0 * voxel_m_;  // 对应搜索半径

    Pose2 p = prior;
    for (int it = 0; it < opt.max_iterations; ++it) {
        const double c = std::cos(p.theta), s = std::sin(p.theta);
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        Eigen::Vector3d g3 = Eigen::Vector3d::Zero();
        double score = 0.0;
        int used = 0;
        for (const auto& pt : scan.points) {
            const double wx = p.x + c * pt.x() - s * pt.y();
            const double wy = p.y + s * pt.x() + c * pt.y();
            const int vx = std::min(
                voxel_grid_.width - 1,
                std::max(0, static_cast<int>(
                                (wx - grid->origin_x) / voxel_m_)));
            const int vy = std::min(
                voxel_grid_.height - 1,
                std::max(0, static_cast<int>(
                                (wy - grid->origin_y) / voxel_m_)));
            // 找最近的非空体素（3x3 邻域）
            int best_g = -1;
            double best_d2 = cell_r * cell_r;
            for (int dy = -1; dy <= 1 && best_g < 0; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const int nx = vx + dx, ny = vy + dy;
                    if (nx < 0 || ny < 0 || nx >= voxel_grid_.width ||
                        ny >= voxel_grid_.height) {
                        continue;
                    }
                    const int gi = voxel_gauss_idx_[static_cast<std::size_t>(ny) *
                                                   voxel_grid_.width + nx];
                    if (gi < 0) { continue; }
                    const Eigen::Vector2d d(wx - gauss_[gi].mu.x(),
                                            wy - gauss_[gi].mu.y());
                    const double d2 = d.squaredNorm();
                    if (d2 < best_d2) {
                        best_d2 = d2;
                        best_g = gi;
                    }
                }
            }
            if (best_g < 0) { continue; }
            const Gauss& G = gauss_[static_cast<std::size_t>(best_g)];
            Eigen::Vector2d d(wx - G.mu.x(), wy - G.mu.y());
            const Eigen::Vector2d cd = G.cov_inv * d;
            const double m2 = d.dot(cd);
            if (m2 > 9.0) { continue; }  // 3σ 截断
            const double sc = std::exp(-0.5 * m2);
            score += sc;
            // ∂s/∂p = s · Jᵀ C⁻¹ d   （J = ∂d/∂p, 2x3）
            Eigen::Matrix<double, 2, 3> J;
            J << 1.0, 0.0, -(wy - p.y), 0.0, 1.0, (wx - p.x);
            const Eigen::Vector3d grad = sc * J.transpose() * cd;
            // GN 近似 Hessian（Magnusson：丢弃二阶项）
            H += sc * J.transpose() * G.cov_inv * J;
            g3 += grad;
            ++used;
        }
        if (used < 5) { break; }
        // 最大化 F=Σs 的近似牛顿步：∇F = -Σ s JᵀC⁻¹d，
        // -∇²F ≈ Σ s JᵀC⁻¹J = H（PSD，丢弃二阶项，Magnusson eq.17-19）
        Eigen::Matrix3d Hs = H;
        Hs.diagonal().array() += 1e-6;
        Eigen::Vector3d dp = -Hs.ldlt().solve(g3);  // dp = H⁻¹∇F = -H⁻¹g3
        if (!dp.allFinite()) { break; }
        // 步长护栏：单步不超过 0.5m / 0.2rad（防远场高斯误导时发散）
        const double nl = dp.head<2>().norm();
        const double s_clip =
            std::min(1.0, std::min(0.5 / std::max(nl, 1e-9),
                                   0.2 / std::max(std::abs(dp(2)), 1e-9)));
        p.x += s_clip * dp(0);
        p.y += s_clip * dp(1);
        p.theta += s_clip * dp(2);
        if (nl * s_clip < opt.converg_t &&
            std::abs(dp(2)) * s_clip < opt.converg_r) {
            break;
        }
    }
    res.pose = p;
    eval_stats(field, p, scan, opt.inlier_gate, res.score, res.inlier_ratio);
    res.valid = true;
    return res;
}

// ============================================================================
// ComposedMatcher —— 粗捕获 → 精修
// ============================================================================
MatchResult ComposedMatcher::match(const MapField& field, const Pose2& prior,
                                   const Scan& scan,
                                   const MatchOptions& opt) const {
    MatchResult coarse = coarse_->match(field, prior, scan, opt);
    if (!coarse.valid) { return coarse; }
    MatchResult fine = fine_->match(field, coarse.pose, scan, opt);
    // 门限：精修后似然不应显著低于粗匹配（防精修跑飞）
    if (fine.score + 0.05 < coarse.score) { return coarse; }
    return fine;
}

// ============================================================================
// ============================================================================
// ============================================================================
// ============================================================================
// GlobalCorrelative —— 分支定界全局搜索（Cartographer MatchFullSubmap 思路）
//
// · 金字塔层存 max-pool，跨越式 4 胞元取最大 → 对胞元内任意离散位姿
//   都是可采纳上界（B&B 正确性的关键）；
// · 优先队列按上界出队，弹出 level-0 节点即精确得分；
// · 重复结构环境（货架/柱阵）保留 top-K 假设，交给后续帧确认；
// · 末端小窗口细修（±10cm / ±0.5°），产出交给跟踪匹配器。
// ============================================================================
// 统一整偏移 B&B：候选位姿 = (aidx, 整数偏移 d)。
// 点 i 落在胞元 c_i + d（整偏移可交换于 lround）。
// 节点 = 偏移块 [b, b+2^lv)，bound = Σ swmax_lv[c_i+b]/n —— 由滑窗栈
// 性质知这是块内任意候选的紧致可采纳上界（无逐点松弛）。
// level-0 弹出即用 likelihood() 精确重评分（出界点记 0）。
GlobalResult global_correlative(const MapField& field, const Scan& scan,
                                int beams_subsample, double ang_step,
                                int n_hypotheses) {
    GlobalResult out;
    const auto* grid = field.grid_;
    if (scan.points.empty()) { return out; }
    const int topk = std::max(1, n_hypotheses);

    // 降采样（B&B 用粗点集，末端重评分用全点集）
    Scan sub;
    for (std::size_t i = 0; i < scan.points.size();
         i += static_cast<std::size_t>(std::max(1, beams_subsample))) {
        sub.points.push_back(scan.points[i]);
    }
    const int n = static_cast<int>(sub.points.size());
    if (n < 5) { return out; }
    const int K = field.poolLevels() - 1;
    if (K < 0) { return out; }
    const int n_ang =
        static_cast<int>(std::ceil(2.0 * M_PI / ang_step));

    // 每角度：整数胞元 c_i + 合法偏移范围
    struct AngData {
        std::vector<Eigen::Vector2i> cells;
        int dx_min{0}, dx_max{0}, dy_min{0}, dy_max{0};
    };
    std::vector<AngData> ang(static_cast<std::size_t>(n_ang));
    const int W = grid->width, H = grid->height;
    for (int a = 0; a < n_ang; ++a) {
        const double th = a * ang_step;
        const double cc = std::cos(th), ss = std::sin(th);
        auto& A = ang[static_cast<std::size_t>(a)];
        A.cells.reserve(sub.points.size());
        int minx = W, maxx = -W, miny = H, maxy = -H;
        for (const auto& pt : sub.points) {
            const int cx = static_cast<int>(std::lround(
                (cc * pt.x() - ss * pt.y()) / grid->resolution));
            const int cy = static_cast<int>(std::lround(
                (ss * pt.x() + cc * pt.y()) / grid->resolution));
            A.cells.push_back({cx, cy});
            minx = std::min(minx, cx);
            maxx = std::max(maxx, cx);
            miny = std::min(miny, cy);
            maxy = std::max(maxy, cy);
        }
        A.dx_min = -minx;
        A.dx_max = W - 1 - maxx;
        A.dy_min = -miny;
        A.dy_max = H - 1 - maxy;
    }

    struct Node {
        double bound;
        int level, bx, by, aidx;
        bool operator<(const Node& o) const { return bound < o.bound; }
    };
    std::priority_queue<Node> pq;

    // 播种：顶层块（步长 2^K）覆盖合法偏移范围
    {
        const int G = 1 << K;
        for (int a = 0; a < n_ang; ++a) {
            const auto& A = ang[static_cast<std::size_t>(a)];
            const int x0 = static_cast<int>(std::floor(A.dx_min / double(G))) * G;
            const int x1 = A.dx_max;
            const int y0 = static_cast<int>(std::floor(A.dy_min / double(G))) * G;
            const int y1 = A.dy_max;
            for (int by = y0; by <= y1; by += G) {
                for (int bx = x0; bx <= x1; bx += G) {
                    double sc = 0.0;
                    for (const auto& c : A.cells) {
                        sc += field.poolScore(K, c.x() + bx, c.y() + by);
                    }
                    sc /= n;
                    if (sc > 0.05) { pq.push({sc, K, bx, by, a}); }
                }
            }
        }
    }
    if (pq.empty()) { return out; }

    // B&B 主循环
    struct Hyp { double score; int dx, dy, aidx; };
    std::vector<Hyp> hyps;
    double worst_kept = 0.0;
    long long expanded = 0;
    while (!pq.empty()) {
        const Node nd = pq.top();
        pq.pop();
        if (static_cast<int>(hyps.size()) >= topk && nd.bound <= worst_kept) {
            break;  // 剪枝
        }
        if (nd.level == 0) {
            // 精确重评分（出界点 0 分，无滑窗膨胀）
            double sc = 0.0;
            const auto& A = ang[static_cast<std::size_t>(nd.aidx)];
            for (const auto& c : A.cells) {
                sc += field.likelihood(c.x() + nd.bx, c.y() + nd.by);
            }
            sc /= n;
            if (sc > 0.10) {
                hyps.push_back({sc, nd.bx, nd.by, nd.aidx});
                worst_kept = hyps.size() == 1
                                 ? hyps[0].score
                                 : std::min(worst_kept, hyps.back().score);
            }
            continue;
        }
        const auto& A = ang[static_cast<std::size_t>(nd.aidx)];
        const int h = 1 << (nd.level - 1);
        for (int dy = 0; dy < 2; ++dy) {
            for (int dx = 0; dx < 2; ++dx) {
                const int bx = nd.bx + dx * h;
                const int by = nd.by + dy * h;
                if (bx > A.dx_max || by > A.dy_max) { continue; }
                double sc = 0.0;
                for (const auto& c : A.cells) {
                    sc += field.poolScore(nd.level - 1, c.x() + bx, c.y() + by);
                }
                sc /= n;
                if (sc > 0.05 &&
                    (static_cast<int>(hyps.size()) < topk || sc > worst_kept)) {
                    pq.push({sc, nd.level - 1, bx, by, nd.aidx});
                }
                ++expanded;
            }
        }
        if (expanded > 30000000LL) { break; }  // 防御性上限
    }
    if (hyps.empty()) { return out; }

    // 每个假设：连续小窗口细修 + 全点集重评分
    struct Refined { double score; Pose2 pose; };
    std::vector<Refined> refined;
    for (const auto& hyp : hyps) {
        const double th0 = hyp.aidx * ang_step;
        const Eigen::Vector2d w0 = grid->gridToWorld(hyp.dx, hyp.dy);
        double best_sc = -1.0;
        double bx = w0.x(), by = w0.y(), bth = th0;
        for (int dth = -4; dth <= 4; ++dth) {
            const double th = th0 + dth * ang_step / 8.0;
            const double cc = std::cos(th), ss = std::sin(th);
            for (int dy = -2; dy <= 2; ++dy) {
                for (int dx = -2; dx <= 2; ++dx) {
                    const double px = w0.x() + dx * grid->resolution;
                    const double py = w0.y() + dy * grid->resolution;
                    double sc = 0.0;
                    for (const auto& pt : scan.points) {  // 全点集
                        const double wx = px + cc * pt.x() - ss * pt.y();
                        const double wy = py + ss * pt.x() + cc * pt.y();
                        const auto g = grid->worldToGridF(wx, wy);
                        sc += field.likelihoodAtF(g.x(), g.y());
                    }
                    sc /= static_cast<double>(scan.points.size());
                    if (sc > best_sc) {
                        best_sc = sc;
                        bx = px;
                        by = py;
                        bth = th;
                    }
                }
            }
        }
        refined.push_back({best_sc, {bx, by, bth}});
    }
    std::sort(refined.begin(), refined.end(),
              [](const Refined& a, const Refined& b) {
                  return a.score > b.score;
              });

    out.pose = refined[0].pose;
    out.score = refined[0].score;
    for (std::size_t i = 1; i < refined.size(); ++i) {
        out.alternates.push_back(refined[i].pose);
    }
    out.valid = out.score > 0.10;
    return out;
}

// 工厂
std::shared_ptr<Matcher> make_matcher(const std::string& name,
                                      const OccupancyGrid* grid,
                                      double ndt_voxel) {
    if (name == "correlative") {
        return std::make_shared<CorrelativeMatcher>();
    }
    if (name == "field_gn") { return std::make_shared<FieldGnMatcher>(); }
    if (name == "icp_p2p") { return std::make_shared<IcpP2pMatcher>(); }
    if (name == "ndt") {
        if (grid == nullptr) { return nullptr; }
        auto m = std::make_shared<NdtMatcher>();
        m->build(*grid, ndt_voxel);
        return m;
    }
    if (name == "composed") {
        return std::make_shared<ComposedMatcher>(
            std::make_shared<CorrelativeMatcher>(),
            std::make_shared<FieldGnMatcher>());
    }
    return nullptr;
}

}  // namespace sloc
