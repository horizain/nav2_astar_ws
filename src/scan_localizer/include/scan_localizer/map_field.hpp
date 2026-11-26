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
#include <memory>
#include <vector>

#include "scan_localizer/se2.hpp"

namespace sloc
{

// 1D 精确平方欧氏距离下包络（供 2D EDT 与单元测试调用）
void edt1d(
  const std::vector<std::int64_t> & f,
  std::vector<std::int64_t> & out);

// ---------- 占据栅格（统一 ROS 标准左下角原点约定） ----------
struct OccupancyGrid
{
  int width{0}, height{0};
  double resolution{0.05};              // m/cell
  double origin_x{0.0}, origin_y{0.0};  // cell (0,0) 左下角世界坐标
  std::vector<int8_t> cells;            // -1 unknown, 0 free, 100 occupied

  bool occupied(int ix, int iy, int occ_thresh = 65) const
  {
    return cells[static_cast<std::size_t>(iy) * width + ix] > occ_thresh;
  }
  bool inside(int ix, int iy) const
  {
    return ix >= 0 && iy >= 0 && ix < width && iy < height;
  }
  // 世界 → 栅格整数索引（向下取整，对齐 ROS 胞元矩形 [origin+i·res, origin+(i+1)·res)）
  Eigen::Vector2i worldToGridIndex(double wx, double wy) const
  {
    return {
      static_cast<int>(std::floor((wx - origin_x) / resolution)),
      static_cast<int>(std::floor((wy - origin_y) / resolution))
    };
  }
  // 世界 → 连续格心坐标（用于格心上的双线性插值；格 (i,j) 中心映射到 (i,j)）
  Eigen::Vector2d worldToGridF(double wx, double wy) const
  {
    return {
      (wx - origin_x) / resolution - 0.5,
      (wy - origin_y) / resolution - 0.5
    };
  }
  // 连续格心坐标 → 世界坐标（格 (i,j) 中心 = origin + (i + 0.5)·res）
  Eigen::Vector2d gridToWorld(double gx, double gy) const
  {
    return {
      origin_x + (gx + 0.5) * resolution,
      origin_y + (gy + 0.5) * resolution
    };
  }
};

// ---------- 带符号距离场 + 似然场 + 金字塔 ----------
class MapField {
public:
  // sigma_m: 似然场平滑尺度(m)；支持自定义占据/自由阈值、最大金字塔层数与可选跳过逆向 EDT
  bool build(
    std::shared_ptr<const OccupancyGrid> grid,
    double sigma_m = 0.20,
    int occupied_thresh = 65,
    int free_thresh = 25,
    int max_pool_levels = 16,
    bool compute_free_edt = true);

  // 便捷重载：复制一份共享所有权，避免调用方局部变量析构导致悬空指针
  bool build(
    const OccupancyGrid & grid,
    double sigma_m = 0.20,
    int occupied_thresh = 65,
    int free_thresh = 25,
    int max_pool_levels = 16,
    bool compute_free_edt = true);

  bool valid() const {return valid_ && grid_ != nullptr;}
  const OccupancyGrid & grid() const {return *grid_;}
  std::shared_ptr<const OccupancyGrid> gridPtr() const {return grid_;}

  // SDF(m)：>0 自由空间距墙距离，=0 障碍表面，<0 墙体内部深度/未知区，双线性插值
  // grad: 插值解析梯度（指向 SDF 增大方向，单位 m/m）
  double sdf(double gx, double gy, Eigen::Vector2d * grad = nullptr) const;

  // 整数格 O(1) 精确 SDF 读取（供射线球面跳跃追踪快速步进使用）
  inline float cellSdf(int ix, int iy) const
  {
    if (!valid_ || !grid_->inside(ix, iy)) {return 0.0f;}
    const std::size_t idx =
      static_cast<std::size_t>(iy) * static_cast<std::size_t>(grid_->width) +
      static_cast<std::size_t>(ix);
    return sdf_cells_[idx];
  }

  // 似然打分：点(栅格系) → [0,1]（build 时未知/灰区已写入 unknown_score_，单次查表）
  inline double likelihood(int ix, int iy) const
  {
    if (!valid_ || !grid_->inside(ix, iy)) {return 0.0;}
    const std::size_t idx =
      static_cast<std::size_t>(iy) * static_cast<std::size_t>(grid_->width) +
      static_cast<std::size_t>(ix);
    return like_[idx];
  }
  double likelihoodAtF(double gx, double gy) const
  {
    return likelihood(
      static_cast<int>(std::lround(gx)),
      static_cast<int>(std::lround(gy)));
  }

  // 梯度下降（自由区）/ 梯度上升（障碍内部）到最近障碍表面单元
  Eigen::Vector2i descendToObstacle(int ix, int iy) const;

  // 滑动窗口最大值栈（Carto PrecomputationGrid 思路）：
  // swmax_[k][cell] = 以 cell 为锚点、2^k×2^k 窗口内 like_ 的最大值。
  int poolLevels() const {return static_cast<int>(swmax_.size());}
  int poolWidth(int) const {return grid_ ? grid_->width : 0;}
  int poolHeight(int) const {return grid_ ? grid_->height : 0;}
  int poolFactor(int level) const
  {
    if (level < 0 || level >= 31) {return 0;}
    return static_cast<int>(std::size_t{1} << static_cast<std::size_t>(level));
  }
  double poolScore(int level, int px, int py) const
  {
    if (!valid_ || level < 0 ||
      level >= static_cast<int>(swmax_.size()))
    {
      return 0.0;
    }
    const int w = grid_->width, h = grid_->height;
    px = std::min(std::max(px, 0), w - 1);
    py = std::min(std::max(py, 0), h - 1);
    return swmax_[static_cast<std::size_t>(level)]
           .data[static_cast<std::size_t>(py) * static_cast<std::size_t>(w) +
             static_cast<std::size_t>(px)];
  }

  double sigma_m_{0.20};
  double unknown_score_{0.10};

private:
  std::shared_ptr<const OccupancyGrid> grid_;
  bool valid_{false};
  int occupied_thresh_{65};
  int free_thresh_{25};
  std::vector<float> sdf_cells_;     // m，=0 于障碍表面
  std::vector<float> like_;          // [0,1]
  struct SWGrid { int width{0}, height{0}; std::vector<float> data; };
  std::vector<SWGrid> swmax_;        // [0]=like，[k]=2^k 滑窗最大值
};

// 构建辅助：从世界系线段栅格化出占据单元（仿真/工具用）
void rasterize_segment(
  OccupancyGrid & grid, double x0, double y0,
  double x1, double y1);

}  // namespace sloc
