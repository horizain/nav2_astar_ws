#pragma once
// ============================================================================
// 极简 SVG 点云绘图：几层点集各配一色，自动取包围盒；
// 可选观察窗（cx, cy, half）做局部放大——鬼点特写用。
// 选 SVG 不选 PNG：文本可读、零编码器依赖、浏览器直接打开。
// ============================================================================
#include <cmath>
#include <fstream>
#include <format>
#include <string>
#include <vector>

#include "polar.hpp"

namespace lpl::viz {

struct Layer {
    const icp::PointCloud2* pts{};
    const char* color{"#333333"};
    double radius{1.5};
};

// 观察窗（可选）：只画 [cx−half, cx+half]×[cy−half, cy+half] 内的点，
// 并放大到 1600px。window 为空指针时自动取全部层的包围盒。
inline void write_svg(const std::string& path, std::vector<Layer> layers,
                       const double* window = nullptr)
{
    double min_x = 1e9, min_y = 1e9, max_x = -1e9, max_y = -1e9;
    bool any = false;
    for (const auto& l : layers) {
        if (!l.pts) continue;
        for (const auto& p : *l.pts) {
            // 观察窗模式下只需找到窗内有没有点
            if (window) {
                if (std::abs(p.x - window[0]) <= window[2] &&
                    std::abs(p.y - window[1]) <= window[2]) {
                    any = true;
                }
                continue;
            }
            any = true;
            min_x = std::min(min_x, p.x);
            min_y = std::min(min_y, p.y);
            max_x = std::max(max_x, p.x);
            max_y = std::max(max_y, p.y);
        }
    }
    std::ofstream f(path);
    if (!f || !any) {
        if (f) {
            f << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1\" "
                 "height=\"1\" style=\"background:white\"></svg>\n";
        }
        return;
    }

    if (window) {
        min_x = window[0] - window[2];
        min_y = window[1] - window[2];
        max_x = window[0] + window[2];
        max_y = window[1] + window[2];
    } else {
        const double pad = 1.0;
        min_x -= pad; min_y -= pad; max_x += pad; max_y += pad;
    }
    const double w = max_x - min_x, h = max_y - min_y;
    const int img_w = 1600;
    const double s = static_cast<double>(img_w) / std::max(w, 1e-9);
    const int img_h = std::max(1, static_cast<int>(h * s));
    auto sx = [&](double x) { return (x - min_x) * s; };
    auto sy = [&](double y) { return static_cast<double>(img_h) - (y - min_y) * s; };

    f << std::format(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{}\" "
        "height=\"{}\" style=\"background:white\">\n",
        img_w, img_h);
    for (const auto& l : layers) {
        if (!l.pts) continue;
        std::string body;
        body.reserve(l.pts->size() * 48);
        for (const auto& p : *l.pts) {
            if (window && (std::abs(p.x - window[0]) > window[2] ||
                           std::abs(p.y - window[1]) > window[2])) {
                continue;
            }
            body += std::format(
                "<circle cx=\"{:.1f}\" cy=\"{:.1f}\" r=\"{:.2f}\" "
                "fill=\"{}\"/>",
                sx(p.x), sy(p.y), l.radius, l.color);
        }
        f << "<g fill-opacity=\"0.85\">\n" << body << "\n</g>\n";
    }
    f << "</svg>\n";
}

}  // namespace lpl::viz
