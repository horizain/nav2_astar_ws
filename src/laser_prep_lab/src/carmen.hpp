#pragma once
// ============================================================================
// CARMEN 日志读取器——laser_prep 版。
// 与 mini_slam 那份的关键区别：保留原始极坐标（ranges + 角度表），
// 无效回波写成 NaN 而不是丢弃——清洗链的输入必须是"原始束列"，
// 而且统计要能数出"原始数据里有多少坏束"。
// 只支持 FLASER 行（Intel Lab 用的就是它；ROBOT LASER 变体 v1 不做，
// 需要时按 mini_slam 的解析逻辑补）。
// ============================================================================
#include <expected>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "polar.hpp"

namespace lpl::carmen {

struct CarmenData {
    std::vector<PolarScan> scans;      // stamp = FLASER 行内嵌 ipc 时间戳
    std::vector<icp::Pose2> scan_odom; // FLASER 内嵌里程计位姿（每扫描时刻）
    OdomBuffer odom;                   // ODOM 行流（deskew 插值用）
    long skipped_lines{0};
};

inline std::optional<double> to_double(const std::string& s)
{
    try {
        return std::stod(s);
    } catch (...) {
        return std::nullopt;
    }
}

inline std::expected<CarmenData, std::string> read(const std::string& path)
{
    std::ifstream f(path);
    if (!f) return std::unexpected("打不开日志文件: " + path);

    CarmenData data;
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream iss(line);
        std::vector<std::string> t;
        std::string tok;
        while (iss >> tok) t.push_back(tok);
        if (t.empty()) continue;

        if (t[0] == "ODOM" && t.size() >= 8) {
            // ODOM x y th tv rv accel ipc_ts host logger_ts
            const auto x = to_double(t[1]), y = to_double(t[2]),
                       th = to_double(t[3]);
            auto ts = to_double(t[7]);
            if (!ts) ts = to_double(t.back());
            if (!x || !y || !th || !ts) {
                ++data.skipped_lines;
                continue;
            }
            data.odom.push(*ts, icp::Pose2{*x, *y, *th});
        } else if (t[0] == "FLASER" && t.size() > 2) {
            // FLASER n r1..rn x y th tv rv accel ipc_ts host logger_ts
            const auto n_d = to_double(t[1]);
            if (!n_d || *n_d < 1 || *n_d > 5000) {
                ++data.skipped_lines;
                continue;
            }
            const int n = static_cast<int>(*n_d);
            const std::size_t base = 2 + static_cast<std::size_t>(n);
            if (t.size() < base + 8) {
                ++data.skipped_lines;
                continue;
            }
            PolarScan s;
            s.range.resize(static_cast<std::size_t>(n), kNaN);
            s.angle_min = -kPi / 2.0;  // SICK 180° FOV，首束 −90°
            s.angle_inc = n > 1 ? kPi / static_cast<double>(n - 1) : 0.0;
            for (int i = 0; i < n; ++i) {
                const auto r = to_double(t[2 + static_cast<std::size_t>(i)]);
                if (!r) continue;
                // CARMEN 无效占位：<=1cm（无回波）或 >=79m（81.83 占位）
                s.range[static_cast<std::size_t>(i)] =
                    (*r <= 0.01 || *r >= 79.0) ? kNaN : *r;
            }
            auto ts = to_double(t[base + 6]);
            if (!ts) ts = to_double(t.back());
            s.stamp = ts.value_or(0.0);
            // beam_dt 交给调用方填——日志里没有扫描时长信息（见 main）
            const auto x = to_double(t[base]), y = to_double(t[base + 1]),
                       th = to_double(t[base + 2]);
            if (!x || !y || !th) {
                ++data.skipped_lines;
                continue;
            }
            data.scans.push_back(std::move(s));
            data.scan_odom.push_back(icp::Pose2{*x, *y, *th});
        } else if (t[0] == "ROBOT") {
            ++data.skipped_lines;  // v1 不支持 ROBOT LASER（诚实计数）
        }
    }
    if (data.scans.empty()) {
        return std::unexpected("日志里没有 FLASER 扫描: " + path);
    }
    return data;
}

}  // namespace lpl::carmen
