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
// 地图 I/O 实现：手写 YAML 键值解析 + P5/P2 PGM 读写（零外部依赖）
// ============================================================================
#include "scan_localizer/map_io.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace sloc
{

namespace
{

std::string trim_copy(const std::string & s)
{
  const auto a = s.find_first_not_of(" \t\r\n");
  if (a == std::string::npos) {return "";}
  const auto b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}

std::string strip_yaml_comment(const std::string & line)
{
  bool in_single = false;
  bool in_double = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '\'' && !in_double) {
      in_single = !in_single;
    } else if (c == '"' && !in_single) {
      in_double = !in_double;
    } else if (c == '#' && !in_single && !in_double) {
      return line.substr(0, i);
    }
  }
  return line;
}

std::string unquote_copy(const std::string & s)
{
  if (s.size() >= 2) {
    if ((s.front() == '"' && s.back() == '"') ||
      (s.front() == '\'' && s.back() == '\''))
    {
      return s.substr(1, s.size() - 2);
    }
  }
  return s;
}

std::string to_lower_copy(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char ch) {return static_cast<char>(std::tolower(ch));});
  return s;
}

bool parse_double_strict(const std::string & raw, double & out)
{
  const std::string s = trim_copy(unquote_copy(trim_copy(raw)));
  if (s.empty()) {return false;}
  try {
    std::size_t pos = 0;
    const double v = std::stod(s, &pos);
    if (pos != s.size() || !std::isfinite(v)) {return false;}
    out = v;
    return true;
  } catch (...) {
    return false;
  }
}

bool parse_negate(const std::string & raw, bool & out)
{
  const std::string s = to_lower_copy(trim_copy(unquote_copy(trim_copy(raw))));
  if (s == "0" || s == "false") {
    out = false;
    return true;
  }
  if (s == "1" || s == "true") {
    out = true;
    return true;
  }
  return false;
}

bool parse_origin(
  const std::string & raw, double & ox, double & oy, double & oth)
{
  std::string v = raw;
  for (char & ch : v) {
    if (ch == ',' || ch == '[' || ch == ']') {ch = ' ';}
  }
  std::istringstream iss(v);
  std::vector<std::string> tokens;
  std::string tok;
  while (iss >> tok) {
    tokens.push_back(tok);
  }
  if (tokens.size() != 3) {return false;}
  return parse_double_strict(tokens[0], ox) &&
         parse_double_strict(tokens[1], oy) &&
         parse_double_strict(tokens[2], oth);
}

// 扁平 YAML 键值解析（支持行内注释、引号及多行 origin 列表）
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
  std::string pending_list_key;
  while (std::getline(f, line)) {
    const std::string clean = strip_yaml_comment(line);
    const std::string trimmed = trim_copy(clean);
    if (trimmed.empty()) {continue;}

    if (!pending_list_key.empty() && trimmed.front() == '-') {
      const std::string item = trim_copy(trimmed.substr(1));
      if (!kv[pending_list_key].empty()) {
        kv[pending_list_key] += " ";
      }
      kv[pending_list_key] += item;
      continue;
    }
    pending_list_key.clear();

    const auto pos = clean.find(':');
    if (pos == std::string::npos) {continue;}
    const std::string key = trim_copy(clean.substr(0, pos));
    const std::string val = trim_copy(clean.substr(pos + 1));
    if (key.empty()) {continue;}
    if (val.empty()) {
      pending_list_key = key;
      kv[key] = "";
    } else {
      kv[key] = val;
    }
  }
  return kv;
}

void skip_pgm_comments(std::ifstream & f)
{
  while (true) {
    f >> std::ws;
    if (f.peek() == '#') {
      std::string dummy;
      std::getline(f, dummy);
    } else {
      break;
    }
  }
}

bool read_pgm(
  const std::string & path, int & w, int & h, int & maxval,
  std::vector<uint8_t> & gray, std::string & err)
{
  std::ifstream f(path, std::ios::binary);
  if (!f.is_open()) {
    err = "cannot open pgm: " + path;
    return false;
  }
  std::string magic;
  if (!(f >> magic)) {
    err = "empty or unreadable pgm header: " + path;
    return false;
  }
  if (magic != "P5" && magic != "P2") {
    err = "unsupported pgm magic '" + magic + "' (need P5/P2)";
    return false;
  }
  skip_pgm_comments(f);
  if (!(f >> w)) {
    err = "invalid pgm width";
    return false;
  }
  skip_pgm_comments(f);
  if (!(f >> h)) {
    err = "invalid pgm height";
    return false;
  }
  skip_pgm_comments(f);
  if (!(f >> maxval)) {
    err = "invalid pgm maxval";
    return false;
  }
  if (w <= 0 || h <= 0) {
    err = "invalid pgm dimensions";
    return false;
  }
  if (maxval <= 0 || maxval > 255) {
    err = "unsupported pgm maxval (only 8-bit 1..255 supported)";
    return false;
  }

  const std::size_t expected =
    static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  gray.assign(expected, 0);

  if (magic == "P5") {
    // 跳过 maxval 后的单个空白分隔符（支持 \r\n）
    const int sep = f.get();
    if (sep == std::char_traits<char>::eof() ||
      !std::isspace(static_cast<unsigned char>(sep)))
    {
      err = "malformed pgm P5 header separator";
      return false;
    }
    if (sep == '\r' && f.peek() == '\n') {
      f.get();
    }
    f.read(
      reinterpret_cast<char *>(gray.data()),
      static_cast<std::streamsize>(expected));
    if (f.gcount() != static_cast<std::streamsize>(expected)) {
      err = "pgm P5 data truncated";
      return false;
    }
  } else {
    for (std::size_t i = 0; i < expected; ++i) {
      skip_pgm_comments(f);
      int v = 0;
      if (!(f >> v)) {
        err = "pgm P2 data truncated or malformed";
        return false;
      }
      if (v < 0 || v > maxval) {
        err = "pgm P2 pixel value out of range";
        return false;
      }
      gray[i] = static_cast<uint8_t>(v);
    }
  }
  return true;
}

}  // namespace

bool load_map_yaml(
  const std::string & yaml_path, OccupancyGrid & out,
  std::string & err)
{
  err.clear();
  const auto kv = parse_flat_yaml(yaml_path, err);
  if (!err.empty()) {return false;}
  if (!kv.count("image") || !kv.count("resolution")) {
    err = "yaml missing 'image' or 'resolution'";
    return false;
  }

  const std::string image_str = unquote_copy(trim_copy(kv.at("image")));
  if (image_str.empty()) {
    err = "yaml 'image' is empty";
    return false;
  }
  const std::filesystem::path yaml_fs(yaml_path);
  std::filesystem::path img_fs(image_str);
  if (img_fs.is_relative()) {
    const std::filesystem::path parent = yaml_fs.parent_path();
    if (!parent.empty()) {
      img_fs = parent / img_fs;
    }
  }

  double resolution = 0.0;
  if (!parse_double_strict(kv.at("resolution"), resolution) ||
    resolution <= 0.0)
  {
    err = "invalid yaml 'resolution'";
    return false;
  }

  double ox = 0.0, oy = 0.0, oth = 0.0;
  if (kv.count("origin")) {
    if (!parse_origin(kv.at("origin"), ox, oy, oth)) {
      err = "invalid yaml 'origin'";
      return false;
    }
    if (std::abs(oth) > 1e-6) {
      err = "non-zero origin yaw is not supported";
      return false;
    }
  }

  bool negate = false;
  if (kv.count("negate")) {
    if (!parse_negate(kv.at("negate"), negate)) {
      err = "invalid yaml 'negate'";
      return false;
    }
  }

  double occ_th = 0.65;
  if (kv.count("occupied_thresh")) {
    if (!parse_double_strict(kv.at("occupied_thresh"), occ_th) ||
      occ_th < 0.0 || occ_th > 1.0)
    {
      err = "invalid yaml 'occupied_thresh'";
      return false;
    }
  }

  double free_th = 0.196;
  if (kv.count("free_thresh")) {
    if (!parse_double_strict(kv.at("free_thresh"), free_th) ||
      free_th < 0.0 || free_th > 1.0)
    {
      err = "invalid yaml 'free_thresh'";
      return false;
    }
  }

  std::string mode = "trinary";
  if (kv.count("mode")) {
    mode = to_lower_copy(trim_copy(unquote_copy(trim_copy(kv.at("mode")))));
    if (mode != "trinary" && mode != "scale" && mode != "raw") {
      err = "unsupported yaml 'mode': " + mode;
      return false;
    }
  }

  int w = 0, h = 0, maxval = 255;
  std::vector<uint8_t> gray;
  if (!read_pgm(img_fs.string(), w, h, maxval, gray, err)) {return false;}

  out.width = w;
  out.height = h;
  out.resolution = resolution;
  out.origin_x = ox;
  out.origin_y = oy;
  out.cells.assign(
    static_cast<std::size_t>(w) * static_cast<std::size_t>(h), -1);

  const double inv_max = 1.0 / static_cast<double>(maxval);
  for (int iy = 0; iy < h; ++iy) {
    // PGM 左上角（row=0）对应 ROS OccupancyGrid 的 iy = h - 1
    const std::size_t row_pgm = static_cast<std::size_t>(h - 1 - iy);
    const std::size_t row_grid = static_cast<std::size_t>(iy);
    for (int ix = 0; ix < w; ++ix) {
      const std::size_t src_idx =
        row_pgm * static_cast<std::size_t>(w) + static_cast<std::size_t>(ix);
      const std::size_t dst_idx =
        row_grid * static_cast<std::size_t>(w) + static_cast<std::size_t>(ix);
      const uint8_t v = gray[src_idx];
      if (mode == "raw") {
        out.cells[dst_idx] = (v <= 100) ? static_cast<int8_t>(v) : int8_t{-1};
        continue;
      }
      const double p = negate ?
        (static_cast<double>(v) * inv_max) :
        (static_cast<double>(maxval - v) * inv_max);
      if (p > occ_th) {
        out.cells[dst_idx] = 100;
      } else if (p < free_th) {
        out.cells[dst_idx] = 0;
      } else if (mode == "scale" && occ_th > free_th) {
        const double ratio = (p - free_th) / (occ_th - free_th);
        const int scaled = static_cast<int>(std::lround(1.0 + 98.0 * ratio));
        out.cells[dst_idx] = static_cast<int8_t>(std::clamp(scaled, 1, 99));
      } else {
        out.cells[dst_idx] = -1;
      }
    }
  }
  return true;
}

bool save_map_yaml(
  const OccupancyGrid & grid, const std::string & base_path,
  std::string & err)
{
  err.clear();
  if (grid.width <= 0 || grid.height <= 0) {
    err = "invalid grid dimensions";
    return false;
  }
  if (!std::isfinite(grid.resolution) || grid.resolution <= 0.0) {
    err = "invalid grid resolution";
    return false;
  }
  if (!std::isfinite(grid.origin_x) || !std::isfinite(grid.origin_y)) {
    err = "invalid grid origin";
    return false;
  }
  const std::size_t expected =
    static_cast<std::size_t>(grid.width) *
    static_cast<std::size_t>(grid.height);
  if (grid.cells.size() != expected) {
    err = "grid cells size mismatch";
    return false;
  }

  // PGM：占据=黑(0)，自由=白(254)，未知=灰(205) —— map_server 同款
  const std::filesystem::path pgm_fs(base_path + ".pgm");
  {
    std::ofstream f(pgm_fs, std::ios::binary);
    if (!f.is_open()) {
      err = "cannot write pgm: " + pgm_fs.string();
      return false;
    }
    f << "P5\n# scan_localizer map\n" << grid.width << " " << grid.height <<
      "\n255\n";
    std::vector<uint8_t> buf(expected);
    for (int iy = 0; iy < grid.height; ++iy) {
      const std::size_t row_pgm =
        static_cast<std::size_t>(grid.height - 1 - iy);
      const std::size_t row_grid = static_cast<std::size_t>(iy);
      for (int ix = 0; ix < grid.width; ++ix) {
        const std::size_t src_idx =
          row_grid * static_cast<std::size_t>(grid.width) +
          static_cast<std::size_t>(ix);
        const std::size_t dst_idx =
          row_pgm * static_cast<std::size_t>(grid.width) +
          static_cast<std::size_t>(ix);
        const int8_t c = grid.cells[src_idx];
        buf[dst_idx] = c > 65 ? 0 : (c < 0 ? 205 : 254);
      }
    }
    f.write(
      reinterpret_cast<const char *>(buf.data()),
      static_cast<std::streamsize>(buf.size()));
    f.flush();
    if (!f.good()) {
      err = "failed writing pgm: " + pgm_fs.string();
      return false;
    }
  }

  // YAML（ROS 标准左下角 origin 约定）
  const std::filesystem::path yaml_fs(base_path + ".yaml");
  {
    std::ofstream f(yaml_fs);
    if (!f.is_open()) {
      err = "cannot write yaml: " + yaml_fs.string();
      return false;
    }
    const std::string pgm_name = pgm_fs.filename().string();
    f << std::setprecision(10);
    f << "image: " << pgm_name << "\n";
    f << "resolution: " << grid.resolution << "\n";
    f << "origin: [" << grid.origin_x << ", " << grid.origin_y << ", 0.0]\n";
    f << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
    f.flush();
    if (!f.good()) {
      err = "failed writing yaml: " + yaml_fs.string();
      return false;
    }
  }
  return true;
}

}  // namespace sloc
