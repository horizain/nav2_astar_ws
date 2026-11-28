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
// 全局相关性搜索（金字塔 Branch-and-Bound + uint8 LUT + 自适应角步长）
// ============================================================================
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

#include "scan_localizer/matchers.hpp"

namespace sloc
{

namespace
{

inline bool pose_is_finite(const Pose2 & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.theta);
}

inline bool pt_is_finite(const Vec2 & pt)
{
  return std::isfinite(pt.x()) && std::isfinite(pt.y());
}

}  // namespace

// ============================================================================
// GlobalCorrelative —— 分支定界全局搜索（Cartographer MatchFullSubmap 思路）
//
// · 金字塔层存滑窗最大值 swmax_ → 对偏移块 [b, b+2^lv) 内任意离散位姿
//   提供紧致可采纳上界（越界光束按 0 分处理而非剔除整个位姿）；
// · 优先队列按上界出队，弹出 level-0 节点即精确得分；
// · 固定容量 top-K + 空间/角度 NMS 维护剪枝下界 worst_kept；
// · 末端小窗口细修（±2 格 / ±半角步长），产出交给跟踪匹配器。
// ============================================================================
// 内部全命中无分支 uint8 似然累加 + 部分和上界早停（统一 4 路标量展开：
// ARM64 Cortex-A72 实测 NEON 缺乏硬件 gather，手写 8 路打包反而慢 ~1.26%）
static inline std::uint32_t eval_u8_inbounds_early_exit(
  const std::uint8_t * base, const std::int32_t * offs,
  int n, std::uint32_t min_sum_u8)
{
  std::uint32_t sum = 0u;
  int i = 0;
  for (; i + 4 <= n; i += 4) {
    sum += static_cast<std::uint32_t>(base[offs[i]]) +
      static_cast<std::uint32_t>(base[offs[i + 1]]) +
      static_cast<std::uint32_t>(base[offs[i + 2]]) +
      static_cast<std::uint32_t>(base[offs[i + 3]]);
    if (((i + 4) & 15) == 0 && min_sum_u8 > 0u) {
      if (sum + static_cast<std::uint32_t>(n - (i + 4)) * 255u <= min_sum_u8) {
        return 0u;
      }
    }
  }
  for (; i < n; ++i) {
    sum += static_cast<std::uint32_t>(base[offs[i]]);
  }
  return sum;
}

GlobalResult global_correlative(
  const MapField & field, const Scan & scan,
  int beams_subsample, double ang_step,
  int n_hypotheses, double max_time_ms)
{
  const auto t_start = std::chrono::steady_clock::now();
  const auto elapsed_ms = [&]() {
      return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_start).count();
    };
  const bool use_time_budget = std::isfinite(max_time_ms) && max_time_ms > 0.0;

  GlobalResult out;
  if (!field.valid() || scan.points.empty() ||
    !std::isfinite(ang_step) || ang_step <= 0.0)
  {
    return out;
  }
  const auto * grid = &field.grid();
  const int topk = std::max(1, n_hypotheses);
  const int W = grid->width, H = grid->height;

    // 收集有限点并降采样（B&B 用粗点集，末端重评分用全点集）
  std::vector<Vec2> valid_pts;
  valid_pts.reserve(scan.points.size());
  for (const auto & pt : scan.points) {
    if (pt_is_finite(pt)) {valid_pts.push_back(pt);}
  }
  if (valid_pts.size() < 5) {return out;}

  std::vector<Vec2> sub_pts;
  const std::size_t step =
    static_cast<std::size_t>(std::max(1, beams_subsample));
  for (std::size_t i = 0; i < valid_pts.size(); i += step) {
    sub_pts.push_back(valid_pts[i]);
  }
  const int n = static_cast<int>(sub_pts.size());
  if (n < 5) {return out;}
  const int K = field.poolLevels() - 1;
  if (K < 0) {return out;}

    // 计算地图自由空间边界框与可通行面积作为机器人位姿搜索域（允许部分光束越界记 0 分）
  int free_xmin = W, free_xmax = -1, free_ymin = H, free_ymax = -1;
  std::size_t free_cells = 0;
  for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
      const int8_t c = grid->cells[static_cast<std::size_t>(y) *
          static_cast<std::size_t>(W) + static_cast<std::size_t>(x)];
      if (c >= 0 && c <= 65) {
        ++free_cells;
        free_xmin = std::min(free_xmin, x);
        free_xmax = std::max(free_xmax, x);
        free_ymin = std::min(free_ymin, y);
        free_ymax = std::max(free_ymax, y);
      }
    }
  }
  if (free_xmin > free_xmax || free_ymin > free_ymax) {
    return out;
  }

  const int n_ang = static_cast<int>(std::ceil(kTwoPi / ang_step));
  if (n_ang <= 0) {return out;}

    // 每角度：整数胞元 c_i + 1D 扁平偏移 + 范围包围盒（供内部全命中无分支快路径使用）
  struct AngData
  {
    std::vector<Eigen::Vector2i> cells;
    std::vector<std::int32_t> flat_offs;
    int min_cx{0}, max_cx{0}, min_cy{0}, max_cy{0};
    int dx_min{0}, dx_max{0}, dy_min{0}, dy_max{0};
  };
  std::vector<AngData> ang(static_cast<std::size_t>(n_ang));
  for (int a = 0; a < n_ang; ++a) {
    const double th = a * ang_step;
    const double cc = std::cos(th), ss = std::sin(th);
    auto & A = ang[static_cast<std::size_t>(a)];
    A.cells.reserve(sub_pts.size());
    A.flat_offs.reserve(sub_pts.size());
    bool first_pt = true;
    for (const auto & pt : sub_pts) {
      const int cx = static_cast<int>(std::lround(
          (cc * pt.x() - ss * pt.y()) / grid->resolution));
      const int cy = static_cast<int>(std::lround(
          (ss * pt.x() + cc * pt.y()) / grid->resolution));
      A.cells.push_back({cx, cy});
      A.flat_offs.push_back(static_cast<std::int32_t>(cy * W + cx));
      if (first_pt) {
        A.min_cx = A.max_cx = cx;
        A.min_cy = A.max_cy = cy;
        first_pt = false;
      } else {
        A.min_cx = std::min(A.min_cx, cx);
        A.max_cx = std::max(A.max_cx, cx);
        A.min_cy = std::min(A.min_cy, cy);
        A.max_cy = std::max(A.max_cy, cy);
      }
    }
    A.dx_min = free_xmin;
    A.dx_max = free_xmax;
    A.dy_min = free_ymin;
    A.dy_max = free_ymax;
  }

  const double inv_n255 = 1.0 / (255.0 * static_cast<double>(n));

    // 块上界打分：优先走 uint8 量化金字塔 + 内部无分支快路径 + 部分和早停
  auto eval_block_bound = [&](int level, int bx, int by, const AngData & A, double min_bound)
    {
      const std::uint8_t * plane = field.poolDataU8(level);
      const std::uint32_t min_sum_u8 = (min_bound > 0.0) ?
        static_cast<std::uint32_t>(std::floor(min_bound * 255.0 * static_cast<double>(n))) : 0u;
      if (plane != nullptr) {
        if (bx + A.min_cx >= 0 && bx + A.max_cx < W &&
          by + A.min_cy >= 0 && by + A.max_cy < H)
        {
          const std::uint8_t * base =
            plane + static_cast<std::size_t>(by) * static_cast<std::size_t>(W) +
            static_cast<std::size_t>(bx);
          const std::uint32_t sum = eval_u8_inbounds_early_exit(
            base, A.flat_offs.data(), n, min_sum_u8);
          return static_cast<double>(sum) * inv_n255;
        }
        const int span = 1 << level;
        std::uint32_t sum = 0u;
        for (int i = 0; i < n; ++i) {
          const auto & c = A.cells[static_cast<std::size_t>(i)];
          const int px = c.x() + bx;
          const int py = c.y() + by;
          if (px >= W || px + span <= 0 || py >= H || py + span <= 0) {
            if (min_sum_u8 > 0u &&
              sum + static_cast<std::uint32_t>(n - 1 - i) * 255u <= min_sum_u8)
            {
              return 0.0;
            }
            continue;
          }
          const int cx = std::max(px, 0);
          const int cy = std::max(py, 0);
          sum += static_cast<std::uint32_t>(
            plane[static_cast<std::size_t>(cy) * static_cast<std::size_t>(W) +
            static_cast<std::size_t>(cx)]);
        }
        return static_cast<double>(sum) * inv_n255;
      }

      const int span = 1 << level;
      double sc = 0.0;
      for (const auto & c : A.cells) {
        const int px = c.x() + bx;
        const int py = c.y() + by;
        if (px >= W || px + span <= 0 || py >= H || py + span <= 0) {
          continue;
        }
        sc += field.poolScore(level, std::max(px, 0), std::max(py, 0));
      }
      return sc / n;
    };

  struct Node
  {
    double bound;
    int level, bx, by, aidx;
    bool operator<(const Node & o) const {return bound < o.bound;}
  };
  std::priority_queue<Node> pq;

    // 播种：顶层块（步长 2^K）覆盖自由空间边界框
  {
    const int G = 1 << K;
    for (int a = 0; a < n_ang; ++a) {
      const auto & A = ang[static_cast<std::size_t>(a)];
      const int x0 = static_cast<int>(
        std::floor(A.dx_min / static_cast<double>(G))) * G;
      const int y0 = static_cast<int>(
        std::floor(A.dy_min / static_cast<double>(G))) * G;
      for (int by = y0; by <= A.dy_max; by += G) {
        for (int bx = x0; bx <= A.dx_max; bx += G) {
          const double sc = eval_block_bound(K, bx, by, A, 0.05);
          if (sc > 0.05) {pq.push({sc, K, bx, by, a});}
        }
      }
    }
  }
  if (pq.empty()) {return out;}

    // NMS 判定阈值（用于区分多峰假设，防止同一峰相邻像素占满 top-K）
  constexpr double kNmsLinM = 0.50;
  constexpr double kNmsAngRad = 0.35;
    // 空间+航向分桶配额：单桶硬封顶 2，桶尺度随可通行面积、地图跨度与射程 p90 自适应放大，
    // 防止 200m 长走廊/大厂房下局部高分区域吞掉 top-K 全局配额并过早抬高剪枝下界。
  double r_p90 = 1.5;
  {
    std::vector<double> ranges;
    ranges.reserve(valid_pts.size());
    for (const auto & pt : valid_pts) {
      const double r = pt.norm();
      if (r > 1e-3) {ranges.push_back(r);}
    }
    if (!ranges.empty()) {
      const std::size_t idx90 = (ranges.size() * 9) / 10;
      std::nth_element(
        ranges.begin(),
        ranges.begin() + static_cast<std::ptrdiff_t>(idx90),
        ranges.end());
      r_p90 = std::max(1.5, ranges[idx90]);
    }
  }
  const double free_area_m2 =
    static_cast<double>(free_cells) * grid->resolution * grid->resolution;
  const double free_diag_m = std::hypot(
    static_cast<double>(free_xmax - free_xmin + 1) * grid->resolution,
    static_cast<double>(free_ymax - free_ymin + 1) * grid->resolution);
  const double denom_slots = static_cast<double>(std::max(4, topk));
  const double area_bucket = 0.6 * std::sqrt(std::max(0.0, free_area_m2) / denom_slots);
  const double span_bucket = 0.75 * free_diag_m / denom_slots;
  const double range_bucket = 0.35 * r_p90;
  const double bucket_size_m = std::clamp(
    std::max({2.50, area_bucket, span_bucket, range_bucket}), 2.50, 20.0);
  constexpr int kAngBuckets = 4;
  const int max_per_bucket = std::min(topk, 2);

  struct Hyp { double score; int dx, dy, aidx; };
  std::vector<Hyp> hyps;
  hyps.reserve(static_cast<std::size_t>(topk));
  double worst_kept = 0.0;

  auto same_mode = [&](const Hyp & a, const Hyp & b)
    {
      const double dist_m = std::hypot(
        (a.dx - b.dx) * grid->resolution,
        (a.dy - b.dy) * grid->resolution);
      const double dang = std::abs(wrap_angle((a.aidx - b.aidx) * ang_step));
      return dist_m < kNmsLinM && dang < kNmsAngRad;
    };

  auto same_bucket = [&](const Hyp & a, const Hyp & b)
    {
      const int axb = static_cast<int>(std::floor(a.dx * grid->resolution / bucket_size_m));
      const int ayb = static_cast<int>(std::floor(a.dy * grid->resolution / bucket_size_m));
      const int bxb = static_cast<int>(std::floor(b.dx * grid->resolution / bucket_size_m));
      const int byb = static_cast<int>(std::floor(b.dy * grid->resolution / bucket_size_m));
      if (axb != bxb || ayb != byb) {return false;}
      const double tha = wrap_angle(a.aidx * ang_step);
      const double thb = wrap_angle(b.aidx * ang_step);
      const int aab = std::clamp(
        static_cast<int>(std::floor((tha + kPi) / (kTwoPi / kAngBuckets))),
        0, kAngBuckets - 1);
      const int bab = std::clamp(
        static_cast<int>(std::floor((thb + kPi) / (kTwoPi / kAngBuckets))),
        0, kAngBuckets - 1);
      return aab == bab;
    };

  auto update_bounds = [&]()
    {
      double w = hyps.empty() ? 0.0 : hyps[0].score;
      double b = 0.0;
      for (const auto & h : hyps) {
        w = std::min(w, h.score);
        b = std::max(b, h.score);
      }
      const double rel_floor = (b > 0.35) ? (b - 0.25) : 0.0;
      if (static_cast<int>(hyps.size()) < topk) {
        worst_kept = rel_floor;
      } else {
        worst_kept = std::max(w, rel_floor);
      }
    };

    // B&B 主循环
  std::int64_t expanded = 0;
  std::int64_t next_time_check = 2048;
  while (!pq.empty()) {
    const Node nd = pq.top();
    pq.pop();
    if (nd.bound <= worst_kept) {
      break;        // 剪枝：所有剩余节点上界均不超过当前第 K 优下界或最优峰-裕度阈值
    }
    const auto & A = ang[static_cast<std::size_t>(nd.aidx)];
    if (nd.level == 0) {
      if (nd.bx < A.dx_min || nd.bx > A.dx_max ||
        nd.by < A.dy_min || nd.by > A.dy_max)
      {
        continue;
      }
      const int8_t origin_cell = grid->cells[
        static_cast<std::size_t>(nd.by) * static_cast<std::size_t>(W) +
        static_cast<std::size_t>(nd.bx)];
      if (origin_cell < 0 || origin_cell > 65) {
        continue;
      }
        // 精确重评分（出界点 0 分，优先走 uint8 内部快路径 + 早停）
      double sc = 0.0;
      const double min_need = std::max(0.10, worst_kept);
      const std::uint8_t * like_u8 = field.likeDataU8();
      if (like_u8 != nullptr &&
        nd.bx + A.min_cx >= 0 && nd.bx + A.max_cx < W &&
        nd.by + A.min_cy >= 0 && nd.by + A.max_cy < H)
      {
        const std::uint32_t min_sum_u8 = static_cast<std::uint32_t>(
          std::floor(min_need * 255.0 * static_cast<double>(n)));
        const std::uint8_t * base =
          like_u8 + static_cast<std::size_t>(nd.by) * static_cast<std::size_t>(W) +
          static_cast<std::size_t>(nd.bx);
        const std::uint32_t sum = eval_u8_inbounds_early_exit(
          base, A.flat_offs.data(), n, min_sum_u8);
        sc = static_cast<double>(sum) * inv_n255;
      } else {
        for (const auto & c : A.cells) {
          sc += field.likelihood(c.x() + nd.bx, c.y() + nd.by);
        }
        sc /= n;
      }
      if (sc > 0.10 && sc > worst_kept) {
        const Hyp cand{sc, nd.bx, nd.by, nd.aidx};
        bool merged = false;
        for (auto & h : hyps) {
          if (same_mode(h, cand)) {
            if (cand.score > h.score) {h = cand;}
            merged = true;
            break;
          }
        }
        if (!merged) {
          int bucket_cnt = 0;
          int worst_in_bucket = -1;
          for (std::size_t hi = 0; hi < hyps.size(); ++hi) {
            if (same_bucket(hyps[hi], cand)) {
              ++bucket_cnt;
              if (worst_in_bucket < 0 ||
                hyps[hi].score < hyps[static_cast<std::size_t>(worst_in_bucket)].score)
              {
                worst_in_bucket = static_cast<int>(hi);
              }
            }
          }
          if (bucket_cnt >= max_per_bucket && worst_in_bucket >= 0) {
            auto & wb = hyps[static_cast<std::size_t>(worst_in_bucket)];
            if (cand.score > wb.score) {
              wb = cand;
            }
          } else if (static_cast<int>(hyps.size()) < topk) {
            hyps.push_back(cand);
          } else {
            auto worst_it = std::min_element(
              hyps.begin(), hyps.end(),
              [](const Hyp & a, const Hyp & b) {return a.score < b.score;});
            if (cand.score > worst_it->score) {
              *worst_it = cand;
            }
          }
        }
        update_bounds();
      }
      continue;
    }

    const int h = 1 << (nd.level - 1);
    for (int dy = 0; dy < 2; ++dy) {
      for (int dx = 0; dx < 2; ++dx) {
        const int bx = nd.bx + dx * h;
        const int by = nd.by + dy * h;
        if (bx > A.dx_max || by > A.dy_max ||
          bx + h - 1 < A.dx_min || by + h - 1 < A.dy_min)
        {
          continue;
        }
        const double sc = eval_block_bound(
          nd.level - 1, bx, by, A, std::max(0.05, worst_kept));
        if (sc > 0.05 && sc > worst_kept) {
          pq.push({sc, nd.level - 1, bx, by, nd.aidx});
        }
        ++expanded;
      }
    }
    if (use_time_budget && expanded >= next_time_check) {
      next_time_check = expanded + 2048;
      const double el = elapsed_ms();
      if ((!hyps.empty() && el >= 0.75 * max_time_ms) ||
        el >= max_time_ms)
      {
        break;  // 墙钟熔断：有候选时保留 25% 给精修；无候选时到总预算硬止损
      }
    }
    if (expanded > 30000000LL) {break;}        // 防御性上限
  }
  if (hyps.empty()) {return out;}

    // 每个假设：连续小窗口细修 + GN 亚像素抛光 + 全点集重评分
  struct Refined { double score; Pose2 pose; };
  std::vector<Refined> refined;
  refined.reserve(hyps.size());
  std::vector<Eigen::Vector2d> rot_pts;
  rot_pts.reserve(valid_pts.size());
  // 按粗分降序排列，确保时间熔断触发时优先精修最高分候选
  std::sort(hyps.begin(), hyps.end(),
    [](const Hyp & a, const Hyp & b) {return a.score > b.score;});
  const std::size_t min_polish = std::min<std::size_t>(hyps.size(), 6u);
  FieldGnMatcher gn_refiner;
  MatchOptions gn_opt;
  gn_opt.max_iterations = 10;
  for (std::size_t hi = 0; hi < hyps.size(); ++hi) {
    if (use_time_budget && hi >= 1u && elapsed_ms() >= max_time_ms) {
      break;
    }
    if (use_time_budget && hi >= min_polish && elapsed_ms() >= 0.90 * max_time_ms) {
      break;
    }
    const auto & hyp = hyps[hi];
    const double th0 = hyp.aidx * ang_step;
    const Eigen::Vector2d w0 = grid->gridToWorld(hyp.dx, hyp.dy);
    double best_sc = -1.0;
    double bx = w0.x(), by = w0.y(), bth = wrap_angle(th0);
    for (int dth = -4; dth <= 4; ++dth) {
      const double th = wrap_angle(th0 + dth * ang_step / 8.0);
      const double cc = std::cos(th), ss = std::sin(th);
      rot_pts.clear();
      for (const auto & pt : valid_pts) {
        rot_pts.emplace_back(
          cc * pt.x() - ss * pt.y(), ss * pt.x() + cc * pt.y());
      }
      for (int dy = -2; dy <= 2; ++dy) {
        const double py = w0.y() + dy * grid->resolution;
        for (int dx = -2; dx <= 2; ++dx) {
          const double px = w0.x() + dx * grid->resolution;
          double sc = 0.0;
          for (const auto & rpt : rot_pts) {
            const auto g = grid->worldToGridF(px + rpt.x(), py + rpt.y());
            sc += field.likelihoodAtF(g.x(), g.y());
          }
          sc /= static_cast<double>(rot_pts.size());
          if (sc > best_sc) {
            best_sc = sc;
            bx = px;
            by = py;
            bth = th;
          }
        }
      }
    }
    const MatchResult gn_res = gn_refiner.match(field, {bx, by, bth}, scan, gn_opt);
    if (gn_res.valid && pose_is_finite(gn_res.pose) &&
      std::isfinite(gn_res.score) && gn_res.score + 0.01 >= best_sc)
    {
      bx = gn_res.pose.x;
      by = gn_res.pose.y;
      bth = wrap_angle(gn_res.pose.theta);
      best_sc = std::max(best_sc, gn_res.score);
    }
    refined.push_back({best_sc, {bx, by, bth}});
  }
  std::sort(refined.begin(), refined.end(),
    [](const Refined & a, const Refined & b) {
      return a.score > b.score;
    });

    // 精修后最终 NMS 去重，保证 alternates 满足最小间距且总数 <= topk
  std::vector<Refined> dedup;
  dedup.reserve(static_cast<std::size_t>(topk));
  for (const auto & r : refined) {
    bool dup = false;
    for (const auto & kept : dedup) {
      const double dist_m = std::hypot(
        r.pose.x - kept.pose.x, r.pose.y - kept.pose.y);
      const double dang = std::abs(wrap_angle(r.pose.theta - kept.pose.theta));
      if (dist_m < kNmsLinM && dang < kNmsAngRad) {
        dup = true;
        break;
      }
    }
    if (!dup) {
      dedup.push_back(r);
      if (static_cast<int>(dedup.size()) >= topk) {break;}
    }
  }
  if (dedup.empty()) {return out;}

  out.pose = dedup[0].pose;
  out.score = dedup[0].score;
  for (std::size_t i = 1; i < dedup.size(); ++i) {
    out.alternates.push_back(dedup[i].pose);
  }
  out.valid = out.score > 0.10;
  return out;
}

double compute_adaptive_ang_step(const MapField & field, const Scan & scan)
{
  constexpr double kMinStep = 0.75 * kPi / 180.0;
  constexpr double kMaxStep = 3.0 * kPi / 180.0;
  constexpr double kFallback = 2.0 * kPi / 180.0;
  if (!field.valid() || scan.points.empty()) {
    return kFallback;
  }
  std::vector<double> ranges;
  ranges.reserve(scan.points.size());
  for (const auto & pt : scan.points) {
    if (pt_is_finite(pt)) {
      const double r = pt.norm();
      if (r > 1e-3) {ranges.push_back(r);}
    }
  }
  if (ranges.empty()) {return kFallback;}
  const std::size_t idx90 = (ranges.size() * 9) / 10;
  std::nth_element(
    ranges.begin(),
    ranges.begin() + static_cast<std::ptrdiff_t>(idx90),
    ranges.end());
  const double r_p90 = std::max(1.5, ranges[idx90]);
  const double sigma = std::max(field.grid().resolution, field.sigma_m_);
  return std::clamp(0.85 * sigma / r_p90, kMinStep, kMaxStep);
}

}  // namespace sloc
