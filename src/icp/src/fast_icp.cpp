#include "fast_icp.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <numbers>
#include <print>
#include <set>
#include <span>
#include <string>
#include <thread>

#include "icp.hpp"
#include "thread_pool.hpp"

namespace ficp {
namespace {

// ICP_TRACE=1 时输出每轮迭代详情（调试用）
const bool g_trace = std::getenv("ICP_TRACE") != nullptr;

// Linux: 统计物理核数（SMT 兄弟去重）。检测失败回退硬件并发数。
// 基准实测：延迟敏感的 fork-join 池用物理核数比逻辑并发数均值快 ~2×
// （SMT 兄弟自旋会争抢物理核流水线）。
auto detect_physical_cores() -> unsigned
{
    const unsigned hw = std::thread::hardware_concurrency();
#if defined(__linux__)
    std::ifstream cpuinfo{"/proc/cpuinfo"};
    if (cpuinfo) {
        std::set<std::pair<int, int>> cores;
        std::string line, phys, core;
        while (std::getline(cpuinfo, line)) {
            if (line.empty()) {
                if (!phys.empty() && !core.empty()) {
                    cores.emplace(std::stoi(phys), std::stoi(core));
                }
                phys.clear();
                core.clear();
            } else if (line.starts_with("physical id")) {
                phys = line.substr(line.find(':') + 1);
            } else if (line.starts_with("core id")) {
                core = line.substr(line.find(':') + 1);
            }
        }
        if (!cores.empty()) {
            return static_cast<unsigned>(cores.size());
        }
    }
#endif
    return hw;
}

// ── 扁平中位数 KD-tree（SoA 思想的每点一节点结构）─────────
// 经验教训：对 ~1k 点的激光扫描，叶桶（bucket leaf）结构每次叶访问都要
// 扫满整桶点，总点距计算量是每点一节点结构的 ~2.5×，且桶内选择循环
// 分支误预测严重。本结构把点评估直接融进下降路径：
//   • 每节点一个点（float，24B 节点，flat 数组，DFS 连续存放）
//   • nth_element 中位数建树 → 完美平衡，深 log₂(n)
//   • 迭代下降 + 显式栈（无递归调用开销）
//   • 最近/次近的更新用三目（编译为 cmov，无分支误预测）
//   • 剪枝：分割面距离 ≥ 当前次近距离才跳过整枝
class KDTreeF {
public:
    static constexpr std::uint32_t kNone = std::numeric_limits<std::uint32_t>::max();

    void build(const Cloud& cloud)
    {
        const std::size_t n = cloud.size();
        nodes_.clear();
        nodes_.reserve(n);
        std::vector<std::uint32_t> idx(n);
        for (std::size_t i = 0; i < n; ++i) {
            idx[i] = static_cast<std::uint32_t>(i);
        }
        build_node(idx, cloud);
    }

    // 一次查询同时返回最近点与次近点（次近点用于局部直线估计）
    struct NN2 {
        std::uint32_t i1{kNone}, i2{kNone};
        double d1{std::numeric_limits<double>::max()};
        double d2{std::numeric_limits<double>::max()};
    };

    [[nodiscard]] auto nn2(double qx, double qy) const noexcept -> NN2
    {
        NN2 out;
        struct Frame {
            std::uint32_t node;
            double dsq;  // 入栈时的分割面距离²——弹出时用最新 d2 复检（延迟剪枝）
        };
        Frame stack[64];  // 平衡树深 ≤ log₂(n)，64 绰绰有余
        int sp = 0;
        std::uint32_t node = 0;
        for (;;) {
            for (;;) {  // 下钻：近侧优先，节点即点，cmov 更新最近/次近
                const Node& nd = nodes_[node];
                const double dx = qx - nd.x;
                const double dy = qy - nd.y;
                const double d2 = dx * dx + dy * dy;

                const bool is_best = d2 < out.d1;
                const bool is_sec = !is_best && d2 < out.d2;
                out.d2 = is_best ? out.d1 : (is_sec ? d2 : out.d2);
                out.i2 = is_best ? out.i1 : (is_sec ? node : out.i2);
                out.d1 = is_best ? d2 : out.d1;
                out.i1 = is_best ? node : out.i1;

                const double diff = (nd.axis == 0 ? qx : qy) - (nd.axis == 0 ? nd.x : nd.y);
                const auto [near, far] =
                    diff < 0.0 ? std::pair{nd.l, nd.r} : std::pair{nd.r, nd.l};
                // 远侧先无条件入栈；近侧访问完后（d2 已收紧）弹出时再剪枝，
                // 与递归版"search(near) 后再判断 far"的剪枝时机一致
                if (far != kNone) {
                    stack[sp++] = {far, diff * diff};
                }
                if (near == kNone) {
                    break;
                }
                node = near;
            }
            // 弹栈：用当前（更紧的）次近距离复检剪枝
            bool resumed = false;
            while (sp > 0) {
                const Frame fr = stack[--sp];
                if (fr.dsq < out.d2) {
                    node = fr.node;
                    resumed = true;
                    break;
                }
            }
            if (!resumed) {
                break;
            }
        }
        return out;
    }

    // 节点（即点）坐标：double 供 GN 残差计算
    [[nodiscard]] auto x(std::uint32_t i) const noexcept -> double
    {
        return nodes_[i].x;
    }
    [[nodiscard]] auto y(std::uint32_t i) const noexcept -> double
    {
        return nodes_[i].y;
    }
    [[nodiscard]] auto size() const noexcept -> std::size_t { return nodes_.size(); }

private:
    struct Node {
        double x, y;     // 该节点所代表的点
        std::uint32_t l{kNone}, r{kNone};
        std::uint8_t axis{0};
    };  // 32B，DFS 序连续存放

    std::uint32_t build_node(std::span<std::uint32_t> idx, const Cloud& cloud)
    {
        if (idx.empty()) {
            return kNone;  // 奇偶中位数切分会产生空右子区间，必须显式处理
        }
        // 分割轴取伸展量更大的一维
        double minx{cloud.x[idx[0]]}, maxx{minx};
        double miny{cloud.y[idx[0]]}, maxy{miny};
        for (const auto i : idx) {
            minx = std::min(minx, cloud.x[i]);
            maxx = std::max(maxx, cloud.x[i]);
            miny = std::min(miny, cloud.y[i]);
            maxy = std::max(maxy, cloud.y[i]);
        }
        const std::uint8_t axis = (maxx - minx) >= (maxy - miny) ? 0 : 1;

        const auto mid = static_cast<std::ptrdiff_t>(idx.size() / 2);
        std::ranges::nth_element(idx, idx.begin() + mid, [axis, &cloud](std::uint32_t a,
                                                                      std::uint32_t b) {
            return axis == 0 ? cloud.x[a] < cloud.x[b] : cloud.y[a] < cloud.y[b];
        });

        const auto id = static_cast<std::uint32_t>(nodes_.size());
        const auto m = static_cast<std::size_t>(mid);
        nodes_.push_back({.x = cloud.x[idx[m]],
                          .y = cloud.y[idx[m]],
                          .l = kNone,
                          .r = kNone,
                          .axis = axis});
        // 递归可能扩容 vector：先占位，回来后按下标回填孩子
        if (idx.size() > 1) {
            const std::uint32_t l = build_node(idx.first(mid), cloud);
            const std::uint32_t r = build_node(idx.subspan(mid + 1), cloud);
            nodes_[id].l = l;
            nodes_[id].r = r;
        }
        return id;
    }

    std::vector<Node> nodes_;  // flat 节点数组（连续内存，DFS 序）
};

// Cramer 法则解 3×3（与 src/icp.cpp 中一致，独立编译单元各持一份）
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

// ── Engine ───────────────────────────────────────────────

struct Engine::Impl {
    Params params;

    KDTreeF tree;

    // threads 语义：总参与者数（含主线程）。0 → 硬件并发数。
    std::unique_ptr<icp::ForkJoinPool> pool;

    // per-chunk Gauss-Newton 累加器，128B 对齐 → 各占独立缓存行对，
    // 不同线程的 chunk 之间零伪共享
    struct alignas(128) Accum {
        double a[6]{};  // 正规方程上三角: xx xy xt yy yt tt
        double b[3]{};
        double wsum{};
        double cost{};
        std::uint64_t valid{};
    };
    std::vector<Accum> accums;

    // 对应点缓存：迭代间位姿只动毫米级，大部分点的最近邻不变。
    // 惰性重验证用"年龄"判据：自建立缓存以来位姿累计位移 ≤ 当时最近点
    // 距离 ⟹ 查询点还没走出原邻域。p2l 的法向残差对"取墙上哪个点"不
    // 敏感，直线段检验与距离门限仍在使用点把关——故无需保证最近点身份
    // 严格不变（严格判据在稠密地图上永不命中，实测反而变慢）
    std::vector<std::uint32_t> cache_q1, cache_q2;
    std::vector<double> cache_d1;   // 建立缓存时的最近点距离
    std::vector<double> cache_cum;  // 建立缓存时的累计位姿位移
};

Engine::Engine(Params params) : impl_(std::make_unique<Impl>())
{
    // threads 语义：总参与者数（含主线程）。0 → 自动检测物理核数。
    unsigned total = params.threads > 0
                         ? static_cast<unsigned>(params.threads)
                         : std::max(1u, detect_physical_cores());
    total = std::max(1u, total);
    impl_->params = params;
    // 池里是"除主线程外"的 worker；单线程模式 → 空池（纯内联）
    impl_->pool =
        std::make_unique<icp::ForkJoinPool>(total - 1, params.pin_base_core);
    impl_->accums.resize(4 * static_cast<std::size_t>(total) + 1);
}

Engine::~Engine() = default;

void Engine::set_reference(const Cloud& ref)
{
    impl_->tree.build(ref);
}

auto Engine::reference_size() const -> std::size_t
{
    return impl_->tree.size();
}

auto Engine::align(const Cloud& source, double init_x, double init_y,
                   double init_theta) -> Result
{
    auto& im = *impl_;
    const std::size_t n = source.size();
    if (n == 0 || im.tree.size() == 0) {
        return {};
    }

    const double huber = im.params.huber_delta;

    double px = init_x, py = init_y, pth = init_theta;
    Result out;
    double prev_rms = std::numeric_limits<double>::infinity();
    int stall = 0;

    // 对应点缓存准备（每帧源点云不同，缓存必须重置）
    const bool use_cache = im.params.cache_correspondences;
    auto& cq1 = im.cache_q1;
    auto& cq2 = im.cache_q2;
    auto& cd1 = im.cache_d1;
    auto& ccum = im.cache_cum;
    if (use_cache) {
        if (cq1.size() != n) {
            cq1.assign(n, KDTreeF::kNone);
            cq2.assign(n, KDTreeF::kNone);
            cd1.assign(n, 0.0);
            ccum.assign(n, 0.0);
        } else {
            std::fill(cq1.begin(), cq1.end(), KDTreeF::kNone);
        }
    }

    // 源点最大半径：位姿转动增量的位移上界 = |δt| + |δθ|·max_r
    double max_r = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        max_r = std::max(max_r, std::hypot(source.x[i], source.y[i]));
    }
    double cum_move = 0.0;  // 累计位姿位移：缓存条目的"年龄"

    for (int iter = 0; iter < im.params.max_iterations; ++iter) {
        out.iterations = iter + 1;
        // 自适应对应距离门限：随残差收窄（同参考实现 src/icp.cpp）
        const double gate = std::clamp(3.0 * prev_rms, im.params.min_corr_dist,
                                       im.params.max_corr_dist);
        const double max_d2 = gate * gate;
        const double c = std::cos(pth);
        const double s = std::sin(pth);
        const KDTreeF& tree = im.tree;

        // 并行：对每个源点做最近邻查询 + Gauss-Newton 累加
        const std::size_t nchunks = im.pool->parallel_chunks(
            n, [&](std::size_t ci, std::size_t b, std::size_t e) {
                Impl::Accum& acc = im.accums[ci];
                acc = Impl::Accum{};  // 本 chunk 独占，直接清零

                for (std::size_t i = b; i < e; ++i) {
                    // 当前位姿下变换到世界系
                    const double qx = px + c * source.x[i] - s * source.y[i];
                    const double qy = py + s * source.x[i] + c * source.y[i];

                    std::uint32_t q1, q2;
                    bool need_query = true;
                    if (use_cache) {
                        q1 = cq1[i];
                        // 惰性重验证（年龄判据）：自建立缓存以来的累计位姿位移
                        // ≤ 当时最近点距离 ⟹ 查询点没走出原邻域，复用 q1/q2
                        if (q1 != KDTreeF::kNone && cum_move - ccum[i] <= cd1[i]) {
                            q2 = cq2[i];
                            need_query = false;
                        }
                    }
                    if (need_query) {
                        const auto nn = tree.nn2(qx, qy);
                        if (nn.i1 == KDTreeF::kNone) {
                            continue;
                        }
                        q1 = nn.i1;
                        q2 = nn.i2;
                        if (use_cache) {
                            cq1[i] = q1;
                            cq2[i] = q2;
                            cd1[i] = std::sqrt(nn.d1);  // 当时最近点距离
                            ccum[i] = cum_move;
                        }
                    }

                    const double dx1 = qx - tree.x(q1);
                    const double dy1 = qy - tree.y(q1);
                    const double d2 = dx1 * dx1 + dy1 * dy1;
                    if (d2 > max_d2) {
                        continue;  // 外点剔除
                    }

                    // (最近点, 次近点) → 局部直线 → 法向量
                    double nx = 0.0, ny = 0.0, r = 0.0;
                    bool have_line = false;
                    if (q2 != KDTreeF::kNone) {
                        const double vx = tree.x(q2) - tree.x(q1);
                        const double vy = tree.y(q2) - tree.y(q1);
                        const double norm2 = vx * vx + vy * vy;
                        const double norm = std::sqrt(norm2);
                        if (norm > 1e-12 && norm <= im.params.max_line_len) {
                            // 投影 t ∈ [0,1]：p 须落在 [q1,q2] 线段内（含少量外延），
                            // 否则该直线不代表 p 处局部表面（如柱子跨面匹配）
                            const double t = (dx1 * vx + dy1 * vy) / norm2;
                            if (t >= -0.25 && t <= 1.25) {
                                nx = -vy / norm;
                                ny = vx / norm;
                                r = nx * dx1 + ny * dy1;
                                have_line = true;
                            }
                        }
                    }
                    if (!have_line) {
                        // 线段检验失败的匹配（如柱子跨面）直接丢弃——plICP 同款
                        // 做法；仅当目标只有单点（退化目标）时保留 Point-to-Point
                        if (q2 != KDTreeF::kNone) {
                            continue;
                        }
                        const double dist = std::sqrt(d2);
                        if (dist < 1e-12) {
                            continue;
                        }
                        nx = dx1 / dist;
                        ny = dy1 / dist;
                        r = dist;
                    }

                    // Huber 权
                    const double ar = std::abs(r);
                    const double w = ar <= huber ? 1.0 : huber / ar;

                    // 雅可比第 3 元 = n·∂pw/∂θ（源点在传感器系）
                    const double sx = source.x[i];
                    const double sy = source.y[i];
                    const double j0 = nx;
                    const double j1 = ny;
                    const double j2 = nx * (-s * sx - c * sy) +
                                      ny * (c * sx - s * sy);

                    acc.a[0] += w * j0 * j0;
                    acc.a[1] += w * j0 * j1;
                    acc.a[2] += w * j0 * j2;
                    acc.a[3] += w * j1 * j1;
                    acc.a[4] += w * j1 * j2;
                    acc.a[5] += w * j2 * j2;
                    acc.b[0] -= w * j0 * r;
                    acc.b[1] -= w * j1 * r;
                    acc.b[2] -= w * j2 * r;
                    acc.wsum += w;
                    acc.cost += w * r * r;
                    ++acc.valid;
                }
            });

        // 归并各 chunk 累加器
        Impl::Accum total{};
        for (std::size_t ci = 0; ci < nchunks; ++ci) {
            const auto& a = im.accums[ci];
            for (int k = 0; k < 6; ++k) {
                total.a[k] += a.a[k];
            }
            for (int k = 0; k < 3; ++k) {
                total.b[k] += a.b[k];
            }
            total.wsum += a.wsum;
            total.cost += a.cost;
            total.valid += a.valid;
        }

        if (total.valid == 0 || total.wsum == 0.0) {
            if (g_trace) {
                std::println(stderr, "[fast] it{:>3} FAIL 无有效对应点", iter + 1);
            }
            return out;  // 无有效对应点
        }

        const double A[3][3]{{total.a[0], total.a[1], total.a[2]},
                             {total.a[1], total.a[3], total.a[4]},
                             {total.a[2], total.a[4], total.a[5]}};
        double delta[3]{};
        if (!solve3x3(A, total.b, delta)) {
            if (g_trace) {
                std::println(stderr, "[fast] it{:>3} FAIL 正规方程奇异", iter + 1);
            }
            return out;  // 奇异
        }

        px += delta[0];
        py += delta[1];
        pth = std::remainder(pth + delta[2], 2.0 * std::numbers::pi);
        // 累计位姿位移（缓存年龄）
        cum_move += std::hypot(delta[0], delta[1]) + std::abs(delta[2]) * max_r;

        out.x = px;
        out.y = py;
        out.theta = pth;
        out.correspondences = static_cast<int>(total.valid);
        const double rms = std::sqrt(total.cost / total.wsum);
        // RMS 停滞计数：噪声底部振荡时的次级收敛判据（相对变化 < 5%）
        stall = (std::abs(rms - prev_rms) <= 0.05 * rms) ? stall + 1 : 0;
        prev_rms = rms;
        out.rms = rms;

        if (g_trace) {
            std::println(stderr,
                         "[fast] it{:>3} corr={:>4} rms={:.4f} pose=({:.4f},{:.4f},{:.5f})",
                         iter + 1, total.valid, out.rms, px, py, pth);
        }

        if (std::hypot(delta[0], delta[1], delta[2]) < im.params.tolerance ||
            (im.params.stall_iters > 0 && stall >= im.params.stall_iters)) {
            out.converged = true;
            break;
        }
    }

    return out;
}

// ── 性能诊断钩子（dbg 用）────────────────────────────────
namespace detail {

struct TreeBench {
    double seconds;
    double sink;
};

// 建树 + 重复查询全部 query 点，返回耗时与防优化 sink
auto bench_tree(const Cloud& ref, const Cloud& queries, int iters) -> TreeBench
{
    KDTreeF tree;
    tree.build(ref);
    double sink = 0.0;
    const auto t0 = std::chrono::steady_clock::now();
    for (int it = 0; it < iters; ++it) {
        for (std::size_t i = 0; i < queries.size(); ++i) {
            const auto nn = tree.nn2(queries.x[i], queries.y[i]);
            sink += static_cast<double>(nn.i1);
        }
    }
    const double sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return {.seconds = sec, .sink = sink};
}

// 一致性检查：与参考实现 icp::KDTree 对同一批查询逐点对比最近/次近
auto check_tree(const Cloud& ref, const Cloud& queries) -> int
{
    KDTreeF ft;
    ft.build(ref);
    icp::PointCloud2 rp;
    rp.reserve(ref.size());
    for (std::size_t i = 0; i < ref.size(); ++i) {
        rp.push_back({ref.x[i], ref.y[i]});
    }
    const icp::KDTree rt{rp};

    int mismatches = 0;
    for (std::size_t i = 0; i < queries.size(); ++i) {
        const icp::Point2 q{queries.x[i], queries.y[i]};
        const auto a = ft.nn2(q.x, q.y);
        const auto b = rt.find2(q);

        const double a1x = ft.x(a.i1), a1y = ft.y(a.i1);
        const double b1x = b.nearest ? b.nearest->x : -1e9;
        const double b1y = b.nearest ? b.nearest->y : -1e9;
        bool bad = std::abs(a1x - b1x) > 1e-9 || std::abs(a1y - b1y) > 1e-9;

        const double a2x = a.i2 != KDTreeF::kNone ? ft.x(a.i2) : -1e9;
        const double a2y = a.i2 != KDTreeF::kNone ? ft.y(a.i2) : -1e9;
        const double b2x = b.second ? b.second->x : -1e9;
        const double b2y = b.second ? b.second->y : -1e9;
        bad = bad || std::abs(a2x - b2x) > 1e-9 || std::abs(a2y - b2y) > 1e-9;
        if (bad) {
            ++mismatches;
        }
    }
    return mismatches;
}

} // namespace detail

} // namespace ficp
