#pragma once
// ============================================================================
// 极简 PNG 写出器（8-bit 灰度，零第三方依赖）。
// 手写 PNG 分块（chunk）+ zlib 流（用 deflate 的"存储块"模式，不压缩但合法），
// CRC32 表在编译期 constexpr 生成——顺带见识 constexpr 循环能做什么。
// ============================================================================
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace mini_slam::png {
namespace detail {

constexpr std::array<std::uint32_t, 256> make_crc_table() {
  std::array<std::uint32_t, 256> t{};
  for (std::uint32_t n = 0; n < 256; ++n) {
    std::uint32_t c = n;
    for (int k = 0; k < 8; ++k) {
      c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    }
    t[n] = c;
  }
  return t;
}

inline std::uint32_t crc32(const std::uint8_t* data, std::size_t n) {
  static const std::array<std::uint32_t, 256> table = make_crc_table();
  std::uint32_t c = 0xFFFFFFFFu;
  for (std::size_t i = 0; i < n; ++i) {
    c = table[(c ^ data[i]) & 0xFFu] ^ (c >> 8);
  }
  return c ^ 0xFFFFFFFFu;
}

inline std::uint32_t adler32(const std::uint8_t* data, std::size_t n) {
  std::uint32_t a = 1, b = 0;
  for (std::size_t i = 0; i < n; ++i) {
    a = (a + data[i]) % 65521u;
    b = (b + a) % 65521u;
  }
  return (b << 16) | a;
}

inline void push_u32(std::vector<std::uint8_t>& v, std::uint32_t x) {
  v.push_back(static_cast<std::uint8_t>(x >> 24));
  v.push_back(static_cast<std::uint8_t>(x >> 16));
  v.push_back(static_cast<std::uint8_t>(x >> 8));
  v.push_back(static_cast<std::uint8_t>(x));
}

inline void push_le16(std::vector<std::uint8_t>& v, std::uint16_t x) {
  v.push_back(static_cast<std::uint8_t>(x & 0xFFu));
  v.push_back(static_cast<std::uint8_t>(x >> 8));
}

inline void push_chunk(std::vector<std::uint8_t>& png, const char* type,
                       const std::vector<std::uint8_t>& payload) {
  push_u32(png, static_cast<std::uint32_t>(payload.size()));
  const auto ty = reinterpret_cast<const std::uint8_t*>(type);
  png.insert(png.end(), ty, ty + 4);
  png.insert(png.end(), payload.begin(), payload.end());
  std::vector<std::uint8_t> crc_input(ty, ty + 4);
  crc_input.insert(crc_input.end(), payload.begin(), payload.end());
  push_u32(png, crc32(crc_input.data(), crc_input.size()));
}

}  // namespace detail

// pixels：行优先灰度图，长度必须 = width * height
inline bool write_grayscale(const std::string& path, int width, int height,
                            const std::vector<std::uint8_t>& pixels) {
  if (width <= 0 || height <= 0 ||
      pixels.size() != static_cast<std::size_t>(width) * height) {
    return false;
  }

  std::vector<std::uint8_t> png;
  png.reserve(static_cast<std::size_t>(width + 1) * height + 128);

  // 文件签名
  const std::uint8_t sig[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
  png.insert(png.end(), sig, sig + 8);

  // IHDR：宽、高、位深 8、颜色类型 0（灰度）
  std::vector<std::uint8_t> ihdr;
  ihdr.reserve(13);
  detail::push_u32(ihdr, static_cast<std::uint32_t>(width));
  detail::push_u32(ihdr, static_cast<std::uint32_t>(height));
  ihdr.push_back(8);  // bit depth
  ihdr.push_back(0);  // color type: grayscale
  ihdr.push_back(0);  // compression
  ihdr.push_back(0);  // filter
  ihdr.push_back(0);  // interlace
  detail::push_chunk(png, "IHDR", ihdr);

  // 原始扫描线：每行前一个 filter 字节（0 = None）
  std::vector<std::uint8_t> raw;
  raw.reserve(static_cast<std::size_t>(width + 1) * height);
  for (int y = 0; y < height; ++y) {
    raw.push_back(0);
    const auto* row = &pixels[static_cast<std::size_t>(y) * width];
    raw.insert(raw.end(), row, row + width);
  }

  // IDAT：zlib 流 = 2 字节头 + 存储型 deflate 块序列 + adler32
  std::vector<std::uint8_t> idat;
  idat.push_back(0x78);
  idat.push_back(0x01);
  std::size_t off = 0;
  do {
    const std::size_t blk = std::min<std::size_t>(65535, raw.size() - off);
    const bool last = (off + blk == raw.size());
    idat.push_back(last ? 0x01 : 0x00);
    detail::push_le16(idat, static_cast<std::uint16_t>(blk));
    detail::push_le16(idat, static_cast<std::uint16_t>(~blk & 0xFFFFu));
    idat.insert(idat.end(), raw.begin() + static_cast<long>(off),
                raw.begin() + static_cast<long>(off + blk));
    off += blk;
  } while (off < raw.size());
  detail::push_u32(idat, detail::adler32(raw.data(), raw.size()));
  detail::push_chunk(png, "IDAT", idat);

  detail::push_chunk(png, "IEND", {});

  std::ofstream f(path, std::ios::binary);
  if (!f) return false;
  f.write(reinterpret_cast<const char*>(png.data()),
          static_cast<std::streamsize>(png.size()));
  return static_cast<bool>(f);
}

// pixels：行优先 RGB，长度必须 = width * height * 3
inline bool write_rgb(const std::string& path, int width, int height,
                      const std::vector<std::uint8_t>& pixels) {
  if (width <= 0 || height <= 0 ||
      pixels.size() != static_cast<std::size_t>(width) * height * 3) {
    return false;
  }

  std::vector<std::uint8_t> png;
  const std::uint8_t sig[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
  png.insert(png.end(), sig, sig + 8);

  std::vector<std::uint8_t> ihdr;
  detail::push_u32(ihdr, static_cast<std::uint32_t>(width));
  detail::push_u32(ihdr, static_cast<std::uint32_t>(height));
  ihdr.push_back(8);  // bit depth
  ihdr.push_back(2);  // color type: truecolor RGB
  ihdr.push_back(0);
  ihdr.push_back(0);
  ihdr.push_back(0);
  detail::push_chunk(png, "IHDR", ihdr);

  std::vector<std::uint8_t> raw;
  raw.reserve(static_cast<std::size_t>(width * 3 + 1) * height);
  for (int y = 0; y < height; ++y) {
    raw.push_back(0);
    const auto* row = &pixels[static_cast<std::size_t>(y) * width * 3];
    raw.insert(raw.end(), row, row + static_cast<std::size_t>(width) * 3);
  }

  std::vector<std::uint8_t> idat;
  idat.push_back(0x78);
  idat.push_back(0x01);
  std::size_t off = 0;
  do {
    const std::size_t blk = std::min<std::size_t>(65535, raw.size() - off);
    const bool last = (off + blk == raw.size());
    idat.push_back(last ? 0x01 : 0x00);
    detail::push_le16(idat, static_cast<std::uint16_t>(blk));
    detail::push_le16(idat, static_cast<std::uint16_t>(~blk & 0xFFFFu));
    idat.insert(idat.end(), raw.begin() + static_cast<long>(off),
                raw.begin() + static_cast<long>(off + blk));
    off += blk;
  } while (off < raw.size());
  detail::push_u32(idat, detail::adler32(raw.data(), raw.size()));
  detail::push_chunk(png, "IDAT", idat);

  detail::push_chunk(png, "IEND", {});

  std::ofstream f(path, std::ios::binary);
  if (!f) return false;
  f.write(reinterpret_cast<const char*>(png.data()),
          static_cast<std::streamsize>(png.size()));
  return static_cast<bool>(f);
}

}  // namespace mini_slam::png
