// Copyright 2026 ruijianding
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
// ============================================================================
// 扫描匹配器组件族（scan-to-map，已知地图定位）
//
//   CorrelativeMatcher      Olson 窗口穷举（两级分辨率），捕获范围大
//   FieldGnMatcher          SDF 连续场 GN（点-线等价），残差=SDF插值，无数据关联
//   IcpP2pMatcher           经典 ICP：SDF 下降找最近障碍单元 + 解析 SE(2) GN
//   NdtMatcher              2D NDT：占据单元体素高斯，D2P 分数最大化
//   ComposedMatcher         Correlative(捕获) → FieldGn(精修)：两级流水线
//   GlobalCorrelative       金字塔 max-pool 全局搜索（冷启动重定位）
// ============================================================================
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "scan_localizer/map_field.hpp"
#include "scan_localizer/se2.hpp"

namespace sloc
{

struct MatchResult
{
  Pose2 pose;
  double score{0.0};           // 归一化平均似然 [0,1]
  Cov3 covariance{Cov3::Identity()};
  double inlier_ratio{0.0};    // SDF<inlier_gate 的点占比
  bool valid{false};
};

struct MatchOptions
{
    // correlative
  double window_lin{0.15};      // ± m（跟踪窗口，按 odom 一周期误差给）
  double window_ang{0.10};      // ± rad
  double step_lin{0.03};        // 粗搜步长
  double step_ang{0.02};        // 粗搜角步长
  bool correlative_two_stage{true};  // 仅需要粗捕获再接 GN 精修时可设为 false 跳过细轮 SDF+exp
    // 精修通用
  int max_iterations{20};
  double converg_t{1e-4};       // m
  double converg_r{1e-5};       // rad
  double huber_delta{0.10};     // m
  double trim_dist{0.50};       // m：残差截断（动态物体抑制）
    // NDT
  double ndt_voxel{0.5};        // m
    // 门限输出
  double inlier_gate{0.10};     // m
};

class Matcher {
public:
  virtual ~Matcher() = default;
  virtual MatchResult match(
    const MapField & field, const Pose2 & prior,
    const Scan & scan,
    const MatchOptions & opt) const = 0;
};

// ---------- Olson 相关性匹配（窗口穷举，两级） ----------
class CorrelativeMatcher : public Matcher {
public:
  MatchResult match(
    const MapField & field, const Pose2 & prior,
    const Scan & scan,
    const MatchOptions & opt) const override;
};

// ---------- SDF 连续场高斯-牛顿（点-线等价） ----------
class FieldGnMatcher : public Matcher {
public:
  MatchResult match(
    const MapField & field, const Pose2 & prior,
    const Scan & scan,
    const MatchOptions & opt) const override;
};

// ---------- 经典 ICP（点到点，最近单元对应） ----------
class IcpP2pMatcher : public Matcher {
public:
  MatchResult match(
    const MapField & field, const Pose2 & prior,
    const Scan & scan,
    const MatchOptions & opt) const override;
};

// ---------- 2D NDT（D2P 分数最大化，解析 GN） ----------
class NdtMatcher : public Matcher {
public:
    // 从占据单元预构建体素高斯；无效地图或非正体素尺寸返回 false
  bool build(const OccupancyGrid & grid, double voxel_m);
  MatchResult match(
    const MapField & field, const Pose2 & prior,
    const Scan & scan,
    const MatchOptions & opt) const override;

private:
  struct Gauss { Eigen::Vector2d mu; Eigen::Matrix2d cov, cov_inv; };
  std::vector<Gauss> gauss_;
  std::vector<int> voxel_gauss_idx_;    // 体素 → gauss_ 索引（-1 空）
  double voxel_m_{0.5};
  int voxel_w_{0};
  int voxel_h_{0};
  double origin_x_{0.0};
  double origin_y_{0.0};
};

// ---------- 组合：粗捕获 + 精修 ----------
class ComposedMatcher : public Matcher {
public:
  explicit ComposedMatcher(
    std::shared_ptr<Matcher> coarse,
    std::shared_ptr<Matcher> fine)
  : coarse_(std::move(coarse)), fine_(std::move(fine)) {}
  MatchResult match(
    const MapField & field, const Pose2 & prior,
    const Scan & scan,
    const MatchOptions & opt) const override;

private:
  std::shared_ptr<Matcher> coarse_, fine_;
};

// ---------- 全局搜索（金字塔 B&B，冷启动）----------
struct GlobalResult
{
  Pose2 pose;
  double score{0.0};
  bool valid{false};
    // 重复结构环境（货架阵列等）会给出多个高分假设：交给后续帧确认
  std::vector<Pose2> alternates;
};
GlobalResult global_correlative(
  const MapField & field, const Scan & scan,
  int beams_subsample, double ang_step,
  int n_hypotheses);

// 光束内部自由空间 / 穿墙 / 未知区一致性因子 ∈ [0.05, 1.0]：
// 用于识别“仅靠侧墙端点得分虚高、但长光束穿墙或射出地图外未知灰区”的反向/错位假峰。
double beam_visibility_consistency(
  const MapField & field, const Pose2 & pose, const Scan & scan,
  bool check_occupied = true);

// 工厂：名字 → 匹配器（配置驱动，供 ROS 节点参数选择）
// grid 供 NDT 等需要预构建的匹配器使用；可为 nullptr（此时 "ndt" 返回空）
std::shared_ptr<Matcher> make_matcher(
  const std::string & name,
  const OccupancyGrid * grid = nullptr,
  double ndt_voxel = 0.5);

}  // namespace sloc
