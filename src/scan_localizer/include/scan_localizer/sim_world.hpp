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
// 仿真世界：占据栅格构建 + 射线扫描模拟（基准测试与仿真节点的受控数据源）
// 支持传感器噪声、动态障碍物注入（在扫描中出现但地图中不存在）
// ============================================================================
#include <random>
#include <vector>

#include "scan_localizer/map_field.hpp"
#include "scan_localizer/se2.hpp"

namespace sloc::sim
{

// 场景一：仓库（40×30m，外墙+隔间+货架+柱子，特征丰富）
OccupancyGrid make_warehouse(double resolution = 0.05);

// 场景二：长走廊（30×3m，沿轴退化场景）
OccupancyGrid make_corridor(double resolution = 0.05);

// 动态障碍：矩形（世界系）
struct DynBox { double cx, cy, hw, hh; };

struct ScanSimConfig
{
  int beams{720};
  double fov{kTwoPi};
  double range_max{12.0};
  double noise_std{0.0};       // 测距高斯噪声 (m)；<=0 时无噪声
  int subsample{1};            // 每 subsample 取一束
  double min_range{0.10};
};

// 单束世界系射线求交（命中返回距离 [min_range, range_max]，未命中返回 -1.0）
double raycast_range(
  const OccupancyGrid & grid, double px, double py, double world_angle,
  double min_range, double range_max,
  const std::vector<DynBox> & dyn = {});

// 从 world 中的位姿 pose（base 系=传感器系）发射一帧点云扫描
Scan make_scan(
  const OccupancyGrid & grid, const Pose2 & pose,
  const ScanSimConfig & cfg, std::mt19937 & rng,
  const std::vector<DynBox> & dyn = {});

// 按 LaserScan 光束角 angle_min + i * angle_increment 直接生成距离数组
// 未命中或超出 [min_range, range_max] 的光束填 +inf，无重分箱角度偏差
std::vector<float> make_laser_ranges(
  const OccupancyGrid & grid, const Pose2 & pose,
  double angle_min, double angle_increment, int beams,
  double min_range, double range_max, double noise_std,
  std::mt19937 & rng,
  const std::vector<DynBox> & dyn = {});

// 随机生成至多 n 个合法测试位姿（计入 grid.origin_x/y，净空半径 clearance_m，
// 最多尝试 max_attempts 次，确保在窄地图/全墙地图上永不死循环）
std::vector<Pose2> random_free_poses(
  const OccupancyGrid & grid, int n,
  std::mt19937 & rng,
  double clearance_m = 1.5,
  int max_attempts = 100000);

// 仓库自由区矩形环路轨迹（供 localizer_sim 与 benchmark E9/E10 共用）
// 每圈 (2,13.5)->(13.5,13.5)->(13.5,28)->(2,28)->(2,13.5) = 52m；
// 默认 2 圈共 104m（416 步 @ 0.25m）
std::vector<Pose2> make_warehouse_trajectory(
  int laps = 2, double step_m = 0.25);

}  // namespace sloc::sim
