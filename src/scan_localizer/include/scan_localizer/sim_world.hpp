#pragma once
// ============================================================================
// 仿真世界：占据栅格构建 + 射线扫描模拟（基准测试的受控数据源）
// 支持传感器噪声、动态障碍物注入（在扫描中出现但地图中不存在）
// ============================================================================
#include <random>
#include <vector>

#include "scan_localizer/map_field.hpp"
#include "scan_localizer/se2.hpp"

namespace sloc::sim {

// 场景一：仓库（40×30m，外墙+隔间+货架+柱子，特征丰富）
OccupancyGrid make_warehouse(double resolution = 0.05);

// 场景二：长走廊（30×3m，沿轴退化场景）
OccupancyGrid make_corridor(double resolution = 0.05);

// 动态障碍：矩形（世界系）
struct DynBox { double cx, cy, hw, hh; };

struct ScanSimConfig {
    int beams{720};
    double fov{2.0 * M_PI};
    double range_max{12.0};
    double noise_std{0.0};     // 测距高斯噪声 (m)
    int subsample{1};          // 每 subsample 取一束
    double min_range{0.10};
};

// 从 world 中的位姿 pose（base 系=传感器系，忽略安装偏移）发射一帧扫描
Scan make_scan(const OccupancyGrid& grid, const Pose2& pose,
               const ScanSimConfig& cfg, std::mt19937& rng,
               const std::vector<DynBox>& dyn = {});

// 随机生成 n 个合法测试位姿（自由空间中，且周围 3m 内有足够特征）
std::vector<Pose2> random_free_poses(const OccupancyGrid& grid, int n,
                                     std::mt19937& rng);

}  // namespace sloc::sim
