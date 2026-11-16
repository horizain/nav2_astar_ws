// ============================================================================
// mini_slam_compare：把两份地图按世界坐标对齐叠加成彩色 PNG。
//   mini_slam_compare <A.pgm> <B.pgm> <out.png>
// 来源：A（我们）红色，B（slam_toolbox）蓝色，两者重合→深灰；
// 空闲→白，未知→浅灰。几何信息来自 .meta 伴随文件（我们的输出）
// 或 .yaml（nav2 map_saver 输出）。
// ============================================================================
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "mini_slam/png_writer.hpp"

namespace {

struct GeoMap {
  int w{0}, h{0};
  double res{0.05};
  double ox{0.0}, oy{0.0};  // origin：栅格 (0,0) 的世界坐标
  std::vector<std::uint8_t> pix;  // 0=占据, 254=空闲, 205=未知
};

bool load_pgm(const std::string& path, GeoMap& m) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::string magic;
  f >> magic;
  if (magic != "P5") return false;
  // 跳过注释与空白，读 w h maxval
  auto read_int = [&f]() -> int {
    int c = f.get();
    while (c == ' ' || c == '\t' || c == '\n' || c == '\r') c = f.get();
    if (c == '#') {
      while (c != '\n' && c != EOF) c = f.get();
      while (c == ' ' || c == '\t' || c == '\n' || c == '\r') c = f.get();
    }
    int v = 0;
    while (c >= '0' && c <= '9') {
      v = v * 10 + (c - '0');
      c = f.get();
    }
    return v;
  };
  m.w = read_int();
  m.h = read_int();
  read_int();  // maxval
  if (m.w <= 0 || m.h <= 0) return false;
  // 注意：read_int 的数字循环已经消费了数字后的单个空白分隔符，
  // 这里不能再 get() 一次——会吃掉第一个像素字节（真实踩过的坑）
  m.pix.resize(static_cast<size_t>(m.w) * m.h);
  f.read(reinterpret_cast<char*>(m.pix.data()),
         static_cast<std::streamsize>(m.pix.size()));
  return static_cast<bool>(f);
}

// 伴随文件：优先 .meta（我们写的），其次 .yaml（map_saver 的）。
// 注意 sidecar 名字是把 pgm 路径的 ".pgm" 后缀换掉：
//   mini_map.pgm → mini_map.meta / mini_map.yaml（不是 mini_map.pgm.meta！）
bool load_geo(const std::string& pgm_path, GeoMap& m) {
  std::string base = pgm_path;
  if (base.size() > 4 && base.substr(base.size() - 4) == ".pgm") {
    base = base.substr(0, base.size() - 4);
  }
  if (std::ifstream meta(base + ".meta"); meta) {
    meta >> m.res >> m.ox >> m.oy >> m.w >> m.h;
    return m.res > 0 && m.w > 0;
  }
  if (std::ifstream yaml(base + ".yaml"); yaml) {
    std::string line;
    while (std::getline(yaml, line)) {
      if (line.find("resolution:") != std::string::npos) {
        try { m.res = std::stod(line.substr(line.find(':') + 1)); } catch (...) {}
      }
      if (line.find("origin:") != std::string::npos) {
        // 形如 origin: [-2.87, -3.9, 0.0]
        const std::string nums = line.substr(line.find('[') + 1);
        try {
          m.ox = std::stod(nums);
          const size_t comma = nums.find(',');
          if (comma != std::string::npos) {
            m.oy = std::stod(nums.substr(comma + 1));
          }
        } catch (...) {}
      }
    }
    return m.res > 0;
  }
  return false;
}

// 世界坐标 → 栅格值。
// PGM 第 0 行是世界坐标最大 y（map_output / run_dataset 写文件时都做了
// y 翻转），而 cy 是从地图底部（min y）数起的行号，所以索引时要翻回来：
//   row = h - 1 - cy
// 之前直接用 cy 索引，导致叠加图上下颠倒。
std::uint8_t sample(const GeoMap& m, double wx, double wy) {
  const int cx = static_cast<int>(std::floor((wx - m.ox) / m.res));
  const int cy = static_cast<int>(std::floor((wy - m.oy) / m.res));
  if (cx < 0 || cy < 0 || cx >= m.w || cy >= m.h) return 205;
  const int row = m.h - 1 - cy;
  return m.pix[static_cast<size_t>(row) * m.w + cx];
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::printf("用法: mini_slam_compare <A.pgm> <B.pgm> <out.png>\n");
    return 1;
  }

  GeoMap a, b;
  if (!load_pgm(argv[1], a) || !load_geo(argv[1], a)) {
    std::printf("A (%s) 加载失败（pgm 或 meta）\n", argv[1]);
    return 1;
  }
  if (!load_pgm(argv[2], b) || !load_geo(argv[2], b)) {
    std::printf("B (%s) 加载失败（pgm 或 yaml/meta）\n", argv[2]);
    return 1;
  }
  std::printf("A: %dx%d @ %.3f origin(%.2f, %.2f)\n", a.w, a.h, a.res, a.ox, a.oy);
  std::printf("B: %dx%d @ %.3f origin(%.2f, %.2f)\n", b.w, b.h, b.res, b.ox, b.oy);

  // 世界坐标包围盒（并集）
  const double min_x = std::min(a.ox, b.ox);
  const double min_y = std::min(a.oy, b.oy);
  const double max_x = std::max(a.ox + a.w * a.res, b.ox + b.w * b.res);
  const double max_y = std::max(a.oy + a.h * a.res, b.oy + b.h * b.res);
  const double res = std::min(a.res, b.res);
  const int w = static_cast<int>(std::ceil((max_x - min_x) / res));
  const int h = static_cast<int>(std::ceil((max_y - min_y) / res));

  // 输出 PNG 行序 = 世界 y 向上翻转
  std::vector<std::uint8_t> rgb(static_cast<size_t>(w) * h * 3, 255);
  for (int row = 0; row < h; ++row) {
    const double wy = min_y + (h - 1 - row) * res;
    for (int col = 0; col < w; ++col) {
      const double wx = min_x + col * res;
      const std::uint8_t va = sample(a, wx, wy);
      const std::uint8_t vb = sample(b, wx, wy);
      const bool a_occ = va == 0;
      const bool b_occ = vb == 0;
      const bool free = va == 254 || vb == 254;
      std::uint8_t r = 255, g = 255, bl = 255;
      if (a_occ && b_occ) {
        r = 40; g = 40; bl = 40;  // 双方重合
      } else if (a_occ) {
        r = 220; g = 40; bl = 40;  // A 独有：红
      } else if (b_occ) {
        r = 40; g = 60; bl = 220;  // B 独有：蓝
      } else if (!free) {
        r = g = bl = 230;  // 未知
      }
      const size_t i = (static_cast<size_t>(row) * w + col) * 3;
      rgb[i] = r; rgb[i + 1] = g; rgb[i + 2] = bl;
    }
  }

  if (!mini_slam::png::write_rgb(argv[3], w, h, rgb)) {
    std::printf("写出失败: %s\n", argv[3]);
    return 1;
  }
  std::printf("已写出 %s (%dx%d)\n", argv[3], w, h);
  return 0;
}
