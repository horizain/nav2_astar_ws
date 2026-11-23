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

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "scan_localizer/map_io.hpp"

namespace
{

namespace fs = std::filesystem;

class TempDirGuard
{
public:
  explicit TempDirGuard(const std::string & name)
  : dir_(fs::temp_directory_path() / ("scan_localizer_test_map_io_" + name))
  {
    std::error_code ec;
    fs::remove_all(dir_, ec);
    fs::create_directories(dir_);
  }
  ~TempDirGuard()
  {
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }
  const fs::path & path() const {return dir_;}

private:
  fs::path dir_;
};

class CwdGuard
{
public:
  explicit CwdGuard(const fs::path & target)
  : prev_(fs::current_path())
  {
    fs::current_path(target);
  }
  ~CwdGuard()
  {
    std::error_code ec;
    fs::current_path(prev_, ec);
  }

private:
  fs::path prev_;
};

void write_text_file(const fs::path & p, const std::string & content)
{
  std::ofstream f(p, std::ios::binary);
  ASSERT_TRUE(f.is_open());
  f.write(content.data(), static_cast<std::streamsize>(content.size()));
  ASSERT_TRUE(f.good());
}

void write_binary_file(
  const fs::path & p, const std::string & header,
  const std::vector<uint8_t> & pixels)
{
  std::ofstream f(p, std::ios::binary);
  ASSERT_TRUE(f.is_open());
  f.write(header.data(), static_cast<std::streamsize>(header.size()));
  if (!pixels.empty()) {
    f.write(
      reinterpret_cast<const char *>(pixels.data()),
      static_cast<std::streamsize>(pixels.size()));
  }
  ASSERT_TRUE(f.good());
}

TEST(MapIoTest, AsymmetricYFlipTopLeftMapsToRosTopRow)
{
  TempDirGuard tmp("yflip");
  // 3x2 P5 PGM:
  // row 0 (top in PGM):    [0 (occ), 254 (free), 205 (unknown)]
  // row 1 (bottom in PGM): [254 (free), 205 (unknown), 0 (occ)]
  write_binary_file(
    tmp.path() / "asym.pgm", "P5\n3 2\n255\n",
    {0, 254, 205, 254, 205, 0});
  write_text_file(
    tmp.path() / "asym.yaml",
    "image: asym.pgm\n"
    "resolution: 0.05\n"
    "origin: [-2.0, 3.5, 0.0]\n"
    "negate: 0\n"
    "occupied_thresh: 0.65\n"
    "free_thresh: 0.196\n");

  sloc::OccupancyGrid g;
  std::string err = "stale error should be cleared";
  ASSERT_TRUE(sloc::load_map_yaml((tmp.path() / "asym.yaml").string(), g, err))
    << err;
  EXPECT_TRUE(err.empty());
  EXPECT_EQ(g.width, 3);
  EXPECT_EQ(g.height, 2);
  EXPECT_DOUBLE_EQ(g.resolution, 0.05);
  EXPECT_DOUBLE_EQ(g.origin_x, -2.0);
  EXPECT_DOUBLE_EQ(g.origin_y, 3.5);
  ASSERT_EQ(g.cells.size(), 6U);

  // ROS iy = 0 (bottom row) comes from PGM row 1: [free(0), unknown(-1), occ(100)]
  EXPECT_EQ(g.cells[0], 0);
  EXPECT_EQ(g.cells[1], -1);
  EXPECT_EQ(g.cells[2], 100);
  // ROS iy = 1 (top row = height - 1) comes from PGM row 0: [occ(100), free(0), unknown(-1)]
  EXPECT_EQ(g.cells[3], 100);
  EXPECT_EQ(g.cells[4], 0);
  EXPECT_EQ(g.cells[5], -1);
}

TEST(MapIoTest, BareRelativeAndAbsolutePaths)
{
  TempDirGuard tmp("paths");
  const fs::path sub = tmp.path() / "sub";
  fs::create_directories(sub);

  write_binary_file(sub / "m.pgm", "P5\n2 2\n255\n", {0, 254, 254, 0});
  write_text_file(
    sub / "bare.yaml",
    "image: m.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n");
  write_text_file(
    tmp.path() / "abs.yaml",
    "image: \"" + (sub / "m.pgm").string() + "\"\n"
    "resolution: 0.1\norigin: [1.0, 2.0, 0.0]\n");

  // 1) Bare filename in current directory
  {
    CwdGuard cwd(sub);
    sloc::OccupancyGrid g;
    std::string err;
    ASSERT_TRUE(sloc::load_map_yaml("bare.yaml", g, err)) << err;
    EXPECT_EQ(g.width, 2);
    EXPECT_EQ(g.height, 2);
  }

  // 2) Relative path (sub/bare.yaml)
  {
    CwdGuard cwd(tmp.path());
    sloc::OccupancyGrid g;
    std::string err;
    ASSERT_TRUE(sloc::load_map_yaml("sub/bare.yaml", g, err)) << err;
    EXPECT_EQ(g.width, 2);
    EXPECT_EQ(g.height, 2);
  }

  // 3) Absolute image path inside yaml
  {
    sloc::OccupancyGrid g;
    std::string err;
    ASSERT_TRUE(sloc::load_map_yaml((tmp.path() / "abs.yaml").string(), g, err))
      << err;
    EXPECT_DOUBLE_EQ(g.origin_x, 1.0);
    EXPECT_DOUBLE_EQ(g.origin_y, 2.0);
  }
}

TEST(MapIoTest, RejectsTruncatedP5AndP2And16Bit)
{
  TempDirGuard tmp("trunc");

  // Truncated P5 (expects 4 bytes, only 3 bytes provided)
  write_binary_file(tmp.path() / "p5_short.pgm", "P5\n2 2\n255\n", {0, 254, 0});
  write_text_file(
    tmp.path() / "p5_short.yaml",
    "image: p5_short.pgm\nresolution: 0.05\n");
  {
    sloc::OccupancyGrid g;
    std::string err;
    EXPECT_FALSE(
      sloc::load_map_yaml((tmp.path() / "p5_short.yaml").string(), g, err));
    EXPECT_FALSE(err.empty());
  }

  // Truncated P2 (expects 4 pixels, only 2 provided)
  write_text_file(tmp.path() / "p2_short.pgm", "P2\n2 2\n255\n0 254\n");
  write_text_file(
    tmp.path() / "p2_short.yaml",
    "image: p2_short.pgm\nresolution: 0.05\n");
  {
    sloc::OccupancyGrid g;
    std::string err;
    EXPECT_FALSE(
      sloc::load_map_yaml((tmp.path() / "p2_short.yaml").string(), g, err));
    EXPECT_FALSE(err.empty());
  }

  // 16-bit PGM (maxval > 255)
  write_binary_file(
    tmp.path() / "p5_16bit.pgm", "P5\n1 1\n65535\n", {0, 0});
  write_text_file(
    tmp.path() / "p5_16bit.yaml",
    "image: p5_16bit.pgm\nresolution: 0.05\n");
  {
    sloc::OccupancyGrid g;
    std::string err;
    EXPECT_FALSE(
      sloc::load_map_yaml((tmp.path() / "p5_16bit.yaml").string(), g, err));
    EXPECT_FALSE(err.empty());
  }
}

TEST(MapIoTest, P2CrlfCommentsMaxvalNegateQuotesAndModes)
{
  TempDirGuard tmp("formats");

  // P2 with comments between magic/w/h/maxval, non-255 maxval=100,
  // negate: true, quoted filename, inline comment with ':'
  write_text_file(
    tmp.path() / "p2.pgm",
    "P2\n# comment 1\n2 # comment between w and h\n"
    "1\n# comment before maxval\n100\n"
    "90 10\n");
  write_text_file(
    tmp.path() / "p2.yaml",
    "# leading comment: should not split\n"
    "image: \"p2.pgm\"  # file: quoted\n"
    "resolution: 0.25   # unit: m/px\n"
    "origin:\n"
    "  - -1.5\n"
    "  - 2.5\n"
    "  - 0.0\n"
    "negate: true\n"
    "occupied_thresh: 0.65\n"
    "free_thresh: 0.25\n");

  {
    sloc::OccupancyGrid g;
    std::string err;
    ASSERT_TRUE(sloc::load_map_yaml((tmp.path() / "p2.yaml").string(), g, err))
      << err;
    EXPECT_EQ(g.width, 2);
    EXPECT_EQ(g.height, 1);
    EXPECT_DOUBLE_EQ(g.resolution, 0.25);
    EXPECT_DOUBLE_EQ(g.origin_x, -1.5);
    EXPECT_DOUBLE_EQ(g.origin_y, 2.5);
    // With negate=true and maxval=100: v=90 -> p=0.90 > 0.65 -> 100; v=10 -> p=0.10 < 0.25 -> 0
    ASSERT_EQ(g.cells.size(), 2U);
    EXPECT_EQ(g.cells[0], 100);
    EXPECT_EQ(g.cells[1], 0);
  }

  // P5 with CRLF (\r\n) header separator and negate: false
  write_binary_file(
    tmp.path() / "crlf.pgm", "P5\r\n2 1\r\n200\r\n", {0, 200});
  write_text_file(
    tmp.path() / "crlf.yaml",
    "image: 'crlf.pgm'\nresolution: 0.05\nnegate: false\n");
  {
    sloc::OccupancyGrid g;
    std::string err;
    ASSERT_TRUE(
      sloc::load_map_yaml((tmp.path() / "crlf.yaml").string(), g, err))
      << err;
    ASSERT_EQ(g.cells.size(), 2U);
    // negate=false, maxval=200: v=0 -> p=1.0 -> 100; v=200 -> p=0.0 -> 0
    EXPECT_EQ(g.cells[0], 100);
    EXPECT_EQ(g.cells[1], 0);
  }

  // negate: 1
  write_text_file(
    tmp.path() / "neg1.yaml",
    "image: crlf.pgm\nresolution: 0.05\nnegate: 1\n");
  {
    sloc::OccupancyGrid g;
    std::string err;
    ASSERT_TRUE(
      sloc::load_map_yaml((tmp.path() / "neg1.yaml").string(), g, err))
      << err;
    ASSERT_EQ(g.cells.size(), 2U);
    EXPECT_EQ(g.cells[0], 0);
    EXPECT_EQ(g.cells[1], 100);
  }

  // mode: scale and mode: raw
  write_binary_file(
    tmp.path() / "modes.pgm", "P5\n3 1\n255\n", {0, 127, 255});
  write_text_file(
    tmp.path() / "scale.yaml",
    "image: modes.pgm\nresolution: 0.05\nmode: scale\n"
    "occupied_thresh: 0.8\nfree_thresh: 0.2\n");
  {
    sloc::OccupancyGrid g;
    std::string err;
    ASSERT_TRUE(
      sloc::load_map_yaml((tmp.path() / "scale.yaml").string(), g, err))
      << err;
    ASSERT_EQ(g.cells.size(), 3U);
    EXPECT_EQ(g.cells[0], 100);
    EXPECT_GT(g.cells[1], 0);
    EXPECT_LT(g.cells[1], 100);
    EXPECT_EQ(g.cells[2], 0);
  }

  write_binary_file(
    tmp.path() / "raw.pgm", "P5\n3 1\n255\n", {0, 50, 205});
  write_text_file(
    tmp.path() / "raw.yaml",
    "image: raw.pgm\nresolution: 0.05\nmode: raw\n");
  {
    sloc::OccupancyGrid g;
    std::string err;
    ASSERT_TRUE(
      sloc::load_map_yaml((tmp.path() / "raw.yaml").string(), g, err))
      << err;
    ASSERT_EQ(g.cells.size(), 3U);
    EXPECT_EQ(g.cells[0], 0);
    EXPECT_EQ(g.cells[1], 50);
    EXPECT_EQ(g.cells[2], -1);
  }
}

TEST(MapIoTest, MalformedYamlDoesNotThrowAndRejectsInvalidFields)
{
  TempDirGuard tmp("bad_yaml");
  write_binary_file(tmp.path() / "ok.pgm", "P5\n1 1\n255\n", {0});

  const std::vector<std::string> bad_yamls = {
    "image: ok.pgm\nresolution: not_a_number\n",
    "image: ok.pgm\nresolution: -0.05\n",
    "image: ok.pgm\nresolution: 0.0\n",
    "image: ok.pgm\nresolution: 0.05\nnegate: maybe\n",
    "image: ok.pgm\nresolution: 0.05\norigin: [1.0, 2.0, 0.5]\n",
    "image: ok.pgm\nresolution: 0.05\norigin: [1.0, bad, 0.0]\n",
    "image: ok.pgm\nresolution: 0.05\nmode: unknown_mode\n",
    "resolution: 0.05\n",
  };

  for (std::size_t i = 0; i < bad_yamls.size(); ++i) {
    const fs::path yp = tmp.path() / ("bad_" + std::to_string(i) + ".yaml");
    write_text_file(yp, bad_yamls[i]);
    sloc::OccupancyGrid g;
    std::string err;
    EXPECT_NO_THROW({
        EXPECT_FALSE(sloc::load_map_yaml(yp.string(), g, err)) << "case " << i;
    });
    EXPECT_FALSE(err.empty()) << "case " << i;
  }
}

TEST(MapIoTest, SaveAndLoadExactRoundtripWithNonZeroOrigin)
{
  TempDirGuard tmp("roundtrip");

  sloc::OccupancyGrid orig;
  orig.width = 5;
  orig.height = 4;
  orig.resolution = 0.05;
  orig.origin_x = -12.345;
  orig.origin_y = 67.89;
  orig.cells.assign(20, 0);
  // Asymmetric pattern across corners and interior
  orig.cells[0 * 5 + 0] = 100;   // bottom-left
  orig.cells[0 * 5 + 4] = -1;    // bottom-right
  orig.cells[3 * 5 + 0] = -1;    // top-left
  orig.cells[3 * 5 + 4] = 100;   // top-right
  orig.cells[2 * 5 + 1] = 100;
  orig.cells[1 * 5 + 3] = -1;

  const fs::path base = tmp.path() / "saved_map";
  std::string err = "stale";
  ASSERT_TRUE(sloc::save_map_yaml(orig, base.string(), err)) << err;
  EXPECT_TRUE(err.empty());

  sloc::OccupancyGrid loaded;
  ASSERT_TRUE(sloc::load_map_yaml(base.string() + ".yaml", loaded, err))
    << err;
  EXPECT_TRUE(err.empty());
  EXPECT_EQ(loaded.width, orig.width);
  EXPECT_EQ(loaded.height, orig.height);
  EXPECT_NEAR(loaded.resolution, orig.resolution, 1e-9);
  EXPECT_NEAR(loaded.origin_x, orig.origin_x, 1e-7);
  EXPECT_NEAR(loaded.origin_y, orig.origin_y, 1e-7);
  EXPECT_EQ(loaded.cells, orig.cells);

  // save_map_yaml validation failures
  sloc::OccupancyGrid bad = orig;
  bad.cells.pop_back();
  EXPECT_FALSE(sloc::save_map_yaml(bad, (tmp.path() / "bad").string(), err));
  EXPECT_FALSE(err.empty());

  bad = orig;
  bad.resolution = 0.0;
  EXPECT_FALSE(sloc::save_map_yaml(bad, (tmp.path() / "bad2").string(), err));
  EXPECT_FALSE(err.empty());
}

}  // namespace
