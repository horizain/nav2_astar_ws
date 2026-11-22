#pragma once
// ============================================================================
// FastICP —— 面向吞吐的 Point-to-Line ICP（2D 激光扫描匹配）
//
// 算法与 src/icp.cpp（参考实现）完全一致：KD-tree 最近邻 + point-to-line
// 残差 + Gauss-Newton + Huber 鲁棒核。工程层面的优化：
//   • SoA float32 存储        —— 内存带宽减半，KD-tree 搜索可自动向量化
//   • 叶桶 KD-tree（flat 数组）—— 树高 log₂(n/16)，叶内 16 点线性扫描吃 SIMD
//   • fork-join 线程池          —— 对应点搜索按 chunk 并行，per-chunk 累加器
//                                  128B 对齐隔离伪共享
//   • 稳态零内存分配            —— 所有缓冲区在 set_reference / 首次对齐时
//                                  预分配，之后 align() 不触碰堆
// ============================================================================
#include <cstddef>
#include <memory>
#include <vector>

namespace ficp {

struct Params {
    int max_iterations{50};
    double tolerance{1e-6};      // 位姿增量范数收敛阈值
    double max_corr_dist{2.0};   // 对应距离门限上限 (m)——首轮/远离时用
    double min_corr_dist{0.25};   // 门限下限 (m)——收敛后剔除视野盲区孤儿点
                                   // 与细结构（如 0.5m 柱子）的跨面伪匹配
    double max_line_len{0.25};   // 次近点距最近点超过此值不做 point-to-line
    double huber_delta{0.2};     // Huber 鲁棒核阈值 (m)
    int stall_iters{2};          // RMS 停滞判据：连续 N 轮视为收敛（0=禁用）
    bool cache_correspondences{false}; // 迭代间对应点缓存（年龄判据惰性重验证）。
                                       // 单线程 3.4×、大地图 1.26×，但强噪声下
                                       // 精度下降（缓存对应关系冻结了噪声偏差），
                                       // 追求与参考实现逐位一致时保持关闭
    int threads{0};              // 0 = 自动（物理核数），1 = 单线程，N = N 线程
    int pin_base_core{-1};       // ≥0：线程池 worker 从该 CPU 号开始轮转绑定
                                 //（低频实时部署建议绑定固定物理核，避开 cpu0/1）
};

struct Result {
    double x{}, y{}, theta{};    // T(source) ≈ reference
    int iterations{};
    int correspondences{};       // 最后一轮有效对应点数
    double rms{};                // 最后一轮加权 RMS 残差 (m)
    bool converged{};
};

// SoA 点云：坐标分列连续存放（nth_element 建树时顺序访问友好）
struct Cloud {
    std::vector<double> x, y;

    Cloud() = default;
    void reserve(std::size_t n) { x.reserve(n); y.reserve(n); }
    void push_back(double px, double py) { x.push_back(px); y.push_back(py); }
    [[nodiscard]] std::size_t size() const { return x.size(); }
};

class Engine {
public:
    explicit Engine(Params params = {});
    ~Engine();

    Engine(const Engine&) = delete;
    auto operator=(const Engine&) -> Engine& = delete;

    // 参考点云 → KD-tree（构建一次，可反复 align）
    void set_reference(const Cloud& ref);

    // ICP 配准；source/reference 为空时返回 converged=false 的空结果
    [[nodiscard]] auto align(const Cloud& source, double init_x, double init_y,
                             double init_theta) -> Result;

    [[nodiscard]] std::size_t reference_size() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace ficp
