#pragma once
// ============================================================================
// Correlative Scan Matcher（Olson 2009《Real-Time Correlative Scan Matching》）
//
// 不做梯度优化，而是在先验位姿附近窗口内穷举离散位姿：
//   1) 粗搜索：全窗口按 (lin_step, ang_step) 网格扫一遍
//   2) 细搜索：粗最优 ± 一个粗步长，步长缩小 4 倍再扫一遍
// 窗口内全局最优，不依赖初值质量、不陷入局部极值；
// 代价是搜索量 O((W/step)² × (A/astep))，两级分辨率正是为了压住这个量。
// 对应 slam_toolbox（karto_sdk）的 CorrelativeScanMatcher。
// ============================================================================
#include <array>
#include <span>

#include "mini_slam/likelihood_field.hpp"

namespace mini_slam {

struct MatchWindow {
  double lin{0.4};       // ± 平移窗口 (m)，世界系
  double ang{0.5};       // ± 角度窗口 (rad)
  double lin_step{0.1};  // 粗搜索平移步长 (m)
  double ang_step{0.05}; // 粗搜索角度步长 (rad)

  // 得分归一化模式：
  //   InBounds：只数落在场内的点 —— 顺序匹配用（参考场只是局部区域）。
  //   AllPoints：总点数归一，出界点计 0 —— 闭环验证用。这是防假闭环的
  //     护栏：一帧大部分视野都不在候选区记录范围内的扫描，本来就不该
  //     强烈"确认"闭环（Olson 原文的归一化方式，地图覆盖全环境时两者等价）。
  bool in_bounds_only{true};
};

struct MatchResult {
  Pose2 pose;                  // 最优位姿
  double response{0.0};        // 细搜索得分 ∈ [0,1]
  double coarse_response{0.0}; // 粗搜索得分（闭环两级阈值用）
  double variance{0.0};        // 粗搜索响应面的位置方差 (m²)：
                               // 单峰尖锐 → 小；平摊/多峰 → 大。
                               // karto 的 LoopMatchMaximumVarianceCoarse 门控
  // 响应面 3×3 协方差（行主序 [xx xy xt; xy yy yt; xt yt tt]，单位 m²/m·rad/rad²）
  // —— 从粗搜索候选的加权二阶矩估计：峰尖锐 → 小；滑移歧义 → 大。
  // 这是比 response² 更有物理依据的边信息矩阵来源（karto 同思路）。
  std::array<double, 9> covariance{};
  bool valid{false};
};

class CorrelativeMatcher {
 public:
  static MatchResult match(const LikelihoodField& field, const Pose2& prior,
                           std::span<const Vec2> points, const MatchWindow& w);
};

}  // namespace mini_slam
