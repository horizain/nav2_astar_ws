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

// ============================================================================
// sim_map_export —— 演示/集成测试辅助：导出仿真地图为 map_server 兼容格式
//   ros2 run scan_localizer sim_map_export /tmp/maps
//   生成 warehouse.yaml/.pgm 与 corridor.yaml/.pgm，并执行全字段逐格 roundtrip 校验
// ============================================================================
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

#include "scan_localizer/map_io.hpp"
#include "scan_localizer/sim_world.hpp"

namespace
{

bool verify_roundtrip(
  const std::string & name,
  const std::string & yaml_path,
  const sloc::OccupancyGrid & orig)
{
  sloc::OccupancyGrid back;
  std::string err;
  if (!sloc::load_map_yaml(yaml_path, back, err)) {
    std::fprintf(stderr, "[%s] roundtrip load failed: %s\n",
      name.c_str(), err.c_str());
    return false;
  }
  if (back.width != orig.width || back.height != orig.height) {
    std::fprintf(stderr, "[%s] size mismatch: %dx%d vs %dx%d\n",
      name.c_str(), back.width, back.height, orig.width, orig.height);
    return false;
  }
  if (std::abs(back.resolution - orig.resolution) > 1e-6) {
    std::fprintf(stderr, "[%s] resolution mismatch: %.9f vs %.9f\n",
      name.c_str(), back.resolution, orig.resolution);
    return false;
  }
  if (std::abs(back.origin_x - orig.origin_x) > 1e-6 ||
    std::abs(back.origin_y - orig.origin_y) > 1e-6)
  {
    std::fprintf(stderr, "[%s] origin mismatch: (%.6f, %.6f) vs (%.6f, %.6f)\n",
      name.c_str(), back.origin_x, back.origin_y,
      orig.origin_x, orig.origin_y);
    return false;
  }
  if (back.cells.size() != orig.cells.size()) {
    std::fprintf(stderr, "[%s] cell count mismatch: %zu vs %zu\n",
      name.c_str(), back.cells.size(), orig.cells.size());
    return false;
  }
  for (std::size_t i = 0; i < orig.cells.size(); ++i) {
    if (back.cells[i] != orig.cells[i]) {
      std::fprintf(stderr, "[%s] cell[%zu] mismatch: %d vs %d\n",
        name.c_str(), i, static_cast<int>(back.cells[i]),
        static_cast<int>(orig.cells[i]));
      return false;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string dir = argc > 1 ? argv[1] : "/tmp/maps";
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {
    std::fprintf(stderr, "create_directories(%s) failed: %s\n",
      dir.c_str(), ec.message().c_str());
    return 1;
  }
  std::string base = dir;
  if (!base.empty() && base.back() != '/') {base += "/";}

  const auto warehouse = sloc::sim::make_warehouse(0.05);
  const auto corridor = sloc::sim::make_corridor(0.05);
  std::string err;
  if (!sloc::save_map_yaml(warehouse, base + "warehouse", err)) {
    std::fprintf(stderr, "save warehouse: %s\n", err.c_str());
    return 1;
  }
  if (!sloc::save_map_yaml(corridor, base + "corridor", err)) {
    std::fprintf(stderr, "save corridor: %s\n", err.c_str());
    return 1;
  }
  if (!verify_roundtrip("warehouse", base + "warehouse.yaml", warehouse)) {
    return 1;
  }
  if (!verify_roundtrip("corridor", base + "corridor.yaml", corridor)) {
    return 1;
  }
  std::printf(
    "exported: %swarehouse.{yaml,pgm} (%dx%d roundtrip 100%% exact), "
    "%scorridor.{yaml,pgm} (%dx%d roundtrip 100%% exact)\n",
    base.c_str(), warehouse.width, warehouse.height,
    base.c_str(), corridor.width, corridor.height);
  return 0;
}
