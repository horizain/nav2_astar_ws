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
// 地图 I/O（业务层地图管理）：nav2/map_server 兼容的 YAML+PGM 加载与保存
//
// 加载：floor.yaml → image PGM → OccupancyGrid
// 保存：OccupancyGrid → floor.pgm + floor.yaml（可直接给 map_server/RViz 用）
//
// 统一采用 ROS 标准约定：
//   · origin_x/y 为栅格 (0, 0) 的左下角世界坐标；
//   · 格 (ix, iy) 中心 = origin + (i + 0.5)·res；
//   · PGM 图像第 0 行（左上）对应 OccupancyGrid 的 iy = height - 1（垂直翻转）。
// ============================================================================
#include <string>

#include "scan_localizer/map_field.hpp"

namespace sloc
{

// 加载 nav2 map_server 风格的地图（yaml + pgm）
// 成功返回 true 且 err 为空；失败返回 false 并写入 err（永不抛异常）。
// 支持字段：image/resolution/origin/negate/occupied_thresh/free_thresh/mode
bool load_map_yaml(
  const std::string & yaml_path, OccupancyGrid & out,
  std::string & err);

// 保存为 map_server 兼容格式（base_path 不带扩展名，生成 base_path.pgm + base_path.yaml）
bool save_map_yaml(
  const OccupancyGrid & grid, const std::string & base_path,
  std::string & err);

}  // namespace sloc
