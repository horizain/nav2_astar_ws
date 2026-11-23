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
// 地图场（MapField）：占据栅格 → 带符号距离场(SDF) → 似然场 → 金字塔
//
// 一次预计算，喂饱全部匹配组件：
//   · correlative 粗匹配：似然场查表打分 + max-pool 金字塔加速全局搜索
//   · field-GN(点-线)精修：SDF 双线性插值即残差，梯度即雅可比（无需数据关联）
//   · ICP-P2P：SDF 梯度下降找最近障碍单元（无需 kd-tree）
//   · PF：似然场即观测模型
// ============================================================================
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "scan_localizer/se2.hpp"

namespace sloc
{

// ---------- 占据栅格 ----------
struct OccupancyGrid
{
  int width{0}, height{0};
  double resolution{0.05};         // m/cell
  double origin_x{0.0}, origin_y{0.0};    // 左下角世界坐标
  std::vector<int8_t> cells;      // -1 unknown, 0 free, 100 occupied

  bool occupied(int ix, int iy) const
  {
    return cells[static_cast<std::size_t>(iy) * width + ix] > 65;
  }
  bool inside(int ix, int iy) const
  {
    return ix >= 0 && iy >= 0 && ix < width && iy < height;
  }
    // 世界 → 栅格（浮点，未 clamp）
  Eigen::Vector2d worldToGridF(double wx, double wy) const
  {
    return {(wx - origin_x) / resolution, (wy - origin_y) / resolution};
  }
  Eigen::Vector2d gridToWorld(double gx, double gy) const
  {
    return {origin_x + gx * resolution, origin_y + gy * resolution};
  }
};

// ---------- 带符号距离场 + 似然场 + 金字塔 ----------
class MapField {
public:
    // sigma_m: 似然场平滑尺度(m)；likelihood 分辨率与栅格一致
  void build(const OccupancyGrid & grid, double sigma_m = 0.20);

    // SDF(m)：>0 自由空间距墙距离，<0 墙体内部深度，双线性插值
    // grad: 插值解析梯度（指向 SDF 增大方向）
  double sdf(double gx, double gy, Eigen::Vector2d * grad = nullptr) const;

    // 似然打分：点(栅格系) → [0,1]
  inline double likelihood(int ix, int iy) const
  {
    if (!grid_->inside(ix, iy)) {return 0.0;}
    const int8_t c = grid_->cells[static_cast<std::size_t>(iy) *
        grid_->width + ix];
    if (c < 0) {return unknown_score_;}                 // 未知：轻微惩罚
    return like_[static_cast<std::size_t>(iy) * grid_->width + ix];
  }
  double likelihoodAtF(double gx, double gy) const
  {
    return likelihood(static_cast<int>(std::lround(gx)),
                         static_cast<int>(std::lround(gy)));
  }

    // 梯度下降到最近障碍单元（起点：栅格系整数坐标）；返回障碍单元索引
  Eigen::Vector2i descendToObstacle(int ix, int iy) const;

    // 滑动窗口最大值栈（Carto PrecomputationGrid 思路）：
    // swmax_[k][cell] = 以 cell 为锚点、2^k×2^k 窗口内 like_ 的最大值。
    // 关键性质：候选位姿是统一整偏移 d，点 i 的可达胞元恰为窗口
    // [c_i+d, c_i+d+2^k)，故该栈给出 B&B 的**紧致可采纳上界**。
    // （块分区 max-pool 会让逐点独立松弛混入上界，导致搜索失效。）
  int poolLevels() const {return static_cast<int>(swmax_.size());}
  int poolWidth(int) const {return grid_ ? grid_->width : 0;}
  int poolHeight(int) const {return grid_ ? grid_->height : 0;}
  int poolFactor(int level) const {return 1 << level;}
  double poolScore(int level, int px, int py) const
  {
    const int w = grid_->width, h = grid_->height;
    px = std::min(std::max(px, 0), w - 1);
    py = std::min(std::max(py, 0), h - 1);
    return swmax_[static_cast<std::size_t>(level)]
           .data[static_cast<std::size_t>(py) * w + px];
  }

  const OccupancyGrid * grid_{nullptr};
  double sigma_m_{0.20};
  double unknown_score_{0.10};

private:
  std::vector<float> sdf_cells_;     // m，=0 于障碍表面
  std::vector<float> like_;         // [0,1]
  struct SWGrid { int width{0}, height{0}; std::vector<float> data; };
  std::vector<SWGrid> swmax_;       // [0]=like，[k]=2^k 滑窗最大值

  void buildEdt(
    const OccupancyGrid & grid, bool from_occupied,
    std::vector<float> & dist_out) const;
};

// 构建辅助：从世界系线段栅格化出占据单元（仿真/工具用）
void rasterize_segment(
  OccupancyGrid & grid, double x0, double y0,
  double x1, double y1);

}  // namespace sloc
