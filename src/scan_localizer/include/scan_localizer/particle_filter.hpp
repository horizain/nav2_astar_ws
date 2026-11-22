#pragma once
// ============================================================================
// 轻量粒子滤波定位（AMCL-lite）：里程计运动模型 + 似然场观测模型
// 作为"PF 架构 vs 扫描匹配架构"的对照组，也充当全局重定位的候选方案
// ============================================================================
#include <random>
#include <vector>

#include "scan_localizer/map_field.hpp"
#include "scan_localizer/se2.hpp"

namespace sloc {

struct PfOptions {
    int n_particles{500};
    // 里程计噪声（Thrun sample model）
    double alpha_rot_rot{0.08};
    double alpha_rot_trans{0.05};
    double alpha_trans_trans{0.05};
    double alpha_trans_rot{0.05};
    // 观测
    int obs_subsample{3};
    // 重采样
    double ess_frac{0.5};
};

class ParticleFilter {
public:
    explicit ParticleFilter(PfOptions opt = {});

    void init_global(const OccupancyGrid& grid, std::mt19937& rng);
    void init_gaussian(const Pose2& mean, double std_xy, double std_th,
                       std::mt19937& rng);

    // 里程计增量（odom �系，base 位姿差）
    void predict(const Pose2& odom_delta, std::mt19937& rng);

    // 似然场观测更新 + ESS 重采样；返回有效粒子占比
    double update(const MapField& field, const Scan& scan, std::mt19937& rng);

    // 加权均值位姿 + 协方差（角度用 sin/cos 均值）
    void estimate(Pose2& mean, Cov3& cov) const;

    double mean_score() const { return mean_score_; }
    bool initialized() const { return !particles_.empty(); }
    std::size_t size() const { return particles_.size(); }

private:
    struct Particle { double x, y, th, w; };
    std::vector<Particle> particles_;
    PfOptions opt_;
    double mean_score_{0.0};
    void resample(std::mt19937& rng);
};

}  // namespace sloc
