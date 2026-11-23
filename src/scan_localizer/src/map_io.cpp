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
// 地图 I/O 实现：手写 YAML 键值解析 + P5 PGM 读写（零外部依赖）
// ============================================================================
#include "scan_localizer/map_io.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>

namespace sloc
{

namespace
{

// 极简 YAML 顶层键值解析（map_server 的 map yaml 是扁平结构）
std::map<std::string, std::string> parse_flat_yaml(
  const std::string & path, std::string & err)
{
  std::map<std::string, std::string> kv;
  std::ifstream f(path);
  if (!f.is_open()) {
    err = "cannot open yaml: " + path;
    return kv;
  }
  std::string line;
  while (std::getline(f, line)) {
    const auto pos = line.find(':');
    if (pos == std::string::npos) {continue;}
    std::string key = line.substr(0, pos);
    std::string val = line.substr(pos + 1);
        // 去注释
    const auto hash = val.find('#');
    if (hash != std::string::npos) {val = val.substr(0, hash);}
        // trim
    auto trim = [](std::string & s) {
        const auto a = s.find_first_not_of(" \t\r\n");
        const auto b = s.find_last_not_of(" \t\r\n");
        s = (a == std::string::npos) ? "" : s.substr(a, b - a + 1);
      };
    trim(key);
    trim(val);
    if (!key.empty() && !val.empty()) {kv[key] = val;}
  }
  return kv;
}

bool read_pgm(
  const std::string & path, int & w, int & h,
  std::vector<uint8_t> & gray, std::string & err)
{
  std::ifstream f(path, std::ios::binary);
  if (!f.is_open()) {
    err = "cannot open pgm: " + path;
    return false;
  }
  std::string magic;
  f >> magic;
  if (magic != "P5" && magic != "P2") {
    err = "unsupported pgm magic '" + magic + "' (need P5/P2)";
    return false;
  }
  int maxval = 255;
  auto skip_comments = [&f]() {
      while (true) {
        f >> std::ws;
        if (f.peek() == '#') {
          std::string dummy;
          std::getline(f, dummy);
        } else {
          break;
        }
      }
    };
  skip_comments();
  f >> w >> h;
  skip_comments();
  f >> maxval;
  f.ignore(1);    // 单空白分隔符
  if (w <= 0 || h <= 0 || maxval <= 0 || maxval > 255) {
    err = "invalid pgm header";
    return false;
  }
  gray.assign(static_cast<std::size_t>(w) * h, 0);
  if (magic == "P5") {
    f.read(reinterpret_cast<char *>(gray.data()),
               static_cast<std::streamsize>(gray.size()));
  } else {
    for (auto & g : gray) {
      int v = 0;
      f >> v;
      g = static_cast<uint8_t>(v);
    }
  }
  if (!f.good() && !f.eof()) {
    err = "pgm data truncated";
    return false;
  }
  return true;
}

}  // namespace

bool load_map_yaml(
  const std::string & yaml_path, OccupancyGrid & out,
  std::string & err)
{
  const auto kv = parse_flat_yaml(yaml_path, err);
  if (!err.empty()) {return false;}
  if (!kv.count("image") || !kv.count("resolution")) {
    err = "yaml missing 'image' or 'resolution'";
    return false;
  }
  const std::string dir_base = yaml_path.substr(0, yaml_path.find_last_of('/'));
  std::string image = kv.at("image");
  if (!image.empty() && image[0] != '/') {
    image = dir_base + "/" + image;      // 相对路径：相对 yaml 所在目录
  }

  out.resolution = std::stod(kv.at("resolution"));
  double ox = 0.0, oy = 0.0, oth = 0.0;
  if (kv.count("origin")) {
        // origin: [x, y, yaw] —— 逗号分隔，流提取会卡死，先归一化分隔符
    std::string v = kv.at("origin");
    for (char & ch : v) {
      if (ch == ',' || ch == '[' || ch == ']') {ch = ' ';}
    }
    std::istringstream iss(v);
    iss >> ox >> oy >> oth;
  }
  const double negate = kv.count("negate") ? std::stod(kv.at("negate")) : 0.0;
  const double occ_th =
    kv.count("occupied_thresh") ? std::stod(kv.at("occupied_thresh")) : 0.65;
  const double free_th =
    kv.count("free_thresh") ? std::stod(kv.at("free_thresh")) : 0.25;
  (void)oth;

  int w = 0, h = 0;
  std::vector<uint8_t> gray;
  if (!read_pgm(image, w, h, gray, err)) {return false;}
  out.width = w;
  out.height = h;
    // ROS 约定补偿：cell i 矩形锚在 origin+i·res → 本库 cell 中心锚同点
  out.origin_x = ox + 0.5 * out.resolution;
  out.origin_y = oy + 0.5 * out.resolution;
  out.cells.assign(static_cast<std::size_t>(w) * h, -1);
  for (std::size_t i = 0; i < gray.size(); ++i) {
        // map_server 公式：p = negate ? v/255 : (255-v)/255
    const double p =
      (negate != 0.0) ? gray[i] / 255.0 : (255.0 - gray[i]) / 255.0;
    if (p > occ_th) {
      out.cells[i] = 100;
    } else if (p < free_th) {
      out.cells[i] = 0;
    } else {
      out.cells[i] = -1;
    }
  }
  return true;
}

bool save_map_yaml(
  const OccupancyGrid & grid, const std::string & base_path,
  std::string & err)
{
    // PGM：占据=黑(0)，自由=白(254)，未知=灰(205) —— map_server 同款
  const std::string pgm_path = base_path + ".pgm";
  {
    std::ofstream f(pgm_path, std::ios::binary);
    if (!f.is_open()) {
      err = "cannot write pgm: " + pgm_path;
      return false;
    }
    f     << "P5\n# scan_localizer map\n" << grid.width << " " << grid.height
          << "\n255\n";
    std::vector<uint8_t> buf(static_cast<std::size_t>(grid.width) *
      grid.height);
    for (std::size_t i = 0; i < buf.size(); ++i) {
      const int8_t c = grid.cells[i];
      buf[i] = c > 65 ? 0 : (c < 0 ? 205 : 254);
    }
    f.write(reinterpret_cast<const char *>(buf.data()),
                static_cast<std::streamsize>(buf.size()));
  }
    // YAML（origin 反向补偿回 ROS 约定）
  const std::string yaml_path = base_path + ".yaml";
  {
    std::ofstream f(yaml_path);
    if (!f.is_open()) {
      err = "cannot write yaml: " + yaml_path;
      return false;
    }
    const std::string pgm_name =
      pgm_path.substr(pgm_path.find_last_of('/') + 1);
    f << "image: " << pgm_name << "\n";
    f << "resolution: " << grid.resolution << "\n";
    f     << "origin: [" << (grid.origin_x - 0.5 * grid.resolution) << ", "
          << (grid.origin_y - 0.5 * grid.resolution) << ", 0.0]\n";
    f << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n";
  }
  return true;
}

}  // namespace sloc
