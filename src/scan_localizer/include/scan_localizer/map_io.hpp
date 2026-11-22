#pragma once
// ============================================================================
// 地图 I/O（业务层地图管理）：nav2/map_server 兼容的 YAML+PGM 加载与保存
//
// 加载：floor.yaml → image PGM → OccupancyGrid
// 保存：OccupancyGrid → floor.pgm + floor.yaml（可直接给 map_server/RViz 用）
//
// 约定换算：本库栅格约定"cell i 中心 = origin + i·res"，
// ROS map_server 约定"cell i 矩形 = [origin + i·res, origin+(i+1)·res)"，
// 两者相差半个胞元 → 装载/保存时 origin 平移 ±res/2 补偿。
// ============================================================================
#include <string>

#include "scan_localizer/map_field.hpp"

namespace sloc {

// 加载 nav2 map_server 风格的地图（yaml + pgm）
// err 为空表示成功。支持字段：image/resolution/origin/negate/
// occupied_thresh/free_thresh（与 map_server 同名同义）
bool load_map_yaml(const std::string& yaml_path, OccupancyGrid& out,
                   std::string& err);

// 保存为 map_server 兼容格式（base 不带扩展名，生成 base.pgm + base.yaml）
bool save_map_yaml(const OccupancyGrid& grid, const std::string& base_path,
                   std::string& err);

}  // namespace sloc
