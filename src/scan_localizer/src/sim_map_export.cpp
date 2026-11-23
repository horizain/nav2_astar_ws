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
//   生成 warehouse.yaml/.pgm 与 corridor.yaml/.pgm
// ============================================================================
#include <cstdio>
#include <filesystem>
#include <string>

#include "scan_localizer/map_io.hpp"
#include "scan_localizer/sim_world.hpp"

int main(int argc, char ** argv)
{
  const std::string dir = argc > 1 ? argv[1] : "/tmp/maps";
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);    // 业务侧免预建目录
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
    // 回读校验（round-trip）
  sloc::OccupancyGrid back;
  if (!sloc::load_map_yaml(base + "warehouse.yaml", back, err)) {
    std::fprintf(stderr, "roundtrip load: %s\n", err.c_str());
    return 1;
  }
  std::printf("exported: %swarehouse.{yaml,pgm} (%dx%d roundtrip ok), "
                "%scorridor.{yaml,pgm}\n",
                base.c_str(), back.width, back.height, base.c_str());
  return 0;
}
