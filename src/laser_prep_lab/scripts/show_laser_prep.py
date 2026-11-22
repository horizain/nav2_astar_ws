#!/usr/bin/env python3
# ============================================================================
# laser_prep_lab 可视化前端（无浏览器环境用）。
#
# 背景：本机没有任何浏览器/图片查看器（xdg-open 全军覆没），但有
# matplotlib（TkAgg/Qt5Agg 可弹窗）。本包的 SVG 是 viz.hpp 手写的
# 纯文本格式（每层一个 <g>，circle 的 cx/cy 就是世界坐标），直接
# 解析即可，不需要 SVG 渲染器。
#
# 用法：
#   python3 scripts/show_laser_prep.py out_intel          # 弹窗看全部
#   python3 scripts/show_laser_prep.py out_intel --png-only  # 只存 PNG
#   python3 scripts/show_laser_prep.py out_intel --panel world  # 只看一张
# ============================================================================
import argparse
import pathlib
import re
import sys

import matplotlib

# 本机有 Noto Sans CJK SC（Ubuntu 24.04 默认装了 noto-cjk），但
# matplotlib 的字体缓存默认不扫 .ttc 集合——必须显式 addfont 注册。
# 字体缺失的机器上自动回落 DejaVu Sans：图不受影响，中文标题变方框。
import matplotlib.font_manager as _fm
import os
for _cjk in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
             "/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc"):
    if os.path.exists(_cjk):
        try:
            _fm.fontManager.addfont(_cjk)
        except Exception:
            pass
matplotlib.rcParams["font.sans-serif"] = [
    "Noto Sans CJK SC", "Noto Serif CJK SC", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

# ----------------------------------------------------------------------------
# 数据解析：SVG(viz.hpp 手写格式) → 分层点集
# ----------------------------------------------------------------------------

CIRCLE_RE = re.compile(
    r'<circle cx="([-\d.]+)" cy="([-\d.]+)" r="([-\d.]+)" fill="(#[0-9a-f]{6})"/>'
)


def parse_svg(path):
    """返回 [(color, radius, [(x, y), ...]), ...]——每个 <g> 一层。"""
    txt = pathlib.Path(path).read_text()
    layers = []
    for group in txt.split("<g")[1:]:
        circles = CIRCLE_RE.findall(group)
        if not circles:
            continue
        pts = [(float(cx), float(cy)) for cx, cy, r, c in circles]
        layers.append((circles[0][3], float(circles[0][2]), pts))
    return layers


# 颜色 → (中文图例, scatter 大小)。尺寸按"角色"给，不按 SVG 半径
# ——特写图的点半径更大，但 scatter 里固定尺寸更好看。
ROLE = {
    "#c8c8c8": ("原始束/未校正", 3),
    "#1a56db": ("保留/校正后", 4),
    "#6e7781": ("门控删除", 22),
    "#f0883e": ("椒盐孤点", 22),
    "#e02424": ("鬼点/动态", 22),
    "#a256e8": ("掠射阴影", 22),
    "#2da44e": ("轨迹", 3),
}
ORDER = ["#c8c8c8", "#1a56db", "#6e7781", "#f0883e", "#e02424", "#a256e8",
         "#2da44e"]


def parse_captions(out_dir):
    """从 index.html 抠出动态标题（帧号/畸变量）。"""
    caps = {}
    html = (out_dir / "index.html").read_text()
    m = re.search(r"删除最多的一帧（第 (\d+) 帧，删 (\d+) 束）", html)
    if m:
        caps["frame_best"] = f"① 清洗对比 — 第 {m.group(1)} 帧（删 {m.group(2)} 束）"
    m = re.search(r"第 (\d+) 帧，扫描期间平移 ([\d.]+)m、旋转 ([\d.-]+)rad", html)
    if m:
        caps["deskew_best"] = (
            f"③ 运动畸变(真实) — 第 {m.group(1)} 帧：平移 {m.group(2)}m / "
            f"旋转 {m.group(3)}rad（远墙上放大成 0.4m 错位）"
        )
    return caps


# ----------------------------------------------------------------------------
# 绘制
# ----------------------------------------------------------------------------

def draw_panel(ax, svg_path, title, small=False):
    layers = parse_svg(svg_path)
    if not layers:
        ax.set_title(f"{title}（缺文件）")
        ax.set_axis_off()
        return
    handles = {}
    for color, _r, pts in layers:
        label, size = ROLE.get(color, (color, 4))
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        h = ax.scatter(xs, ys, s=size * (0.6 if small else 1.0),
                       c=color, label=label, linewidths=0, zorder=2)
        handles[color] = h
    # SVG 的 y 向下（屏幕坐标），翻回来就是世界坐标的方向
    ax.invert_yaxis()
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=11)
    ax.grid(True, ls=":", alpha=0.4)
    # 图例按固定顺序，只列这个面板里出现过的层
    present = [c for c, _r, _ in layers]
    labels = [ROLE[c][0] for c in ORDER if c in present]
    if labels:
        ax.legend(labels, fontsize=8, loc="upper right", framealpha=0.9)


PANELS = [
    # (key, 文件名, 默认标题, 在汇总图里的位置)
    ("frame_best", "frame_best.svg", "① 清洗对比（删除最多的一帧）", (0, 0)),
    ("ghost_zoom", "ghost_zoom.svg", "② 鬼点特写", (0, 1)),
    ("deskew_demo", "deskew_demo.svg", "③b 运动畸变 — 合成演示（1m/s，0.2s 扫描）", (1, 0)),
    ("deskew_best", "deskew_best.svg", "③ 运动畸变 — 真实数据（高倍微窗）", (1, 1)),
    ("world", "world_view.svg", "④ 全程总览（蓝=静态 绿=轨迹 红=动态候选）", (2, slice(0, 2))),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir", type=pathlib.Path, help="laser_prep_lab 的 --out 目录")
    ap.add_argument("--png-only", action="store_true", help="不弹窗，只存 PNG")
    ap.add_argument("--panel", choices=[p[0] for p in PANELS],
                    help="只看一张（默认全部）")
    args = ap.parse_args()

    if args.png_only:
        matplotlib.use("Agg")
    else:
        try:
            matplotlib.use("TkAgg")
        except Exception:
            pass  # 回落到默认后端
    import matplotlib.pyplot as plt

    out = args.out_dir
    if not (out / "index.html").exists():
        sys.exit(f"{out} 不是 laser_prep_lab 的输出目录（缺 index.html）")
    caps = parse_captions(out)

    png_dir = out / "png"
    png_dir.mkdir(exist_ok=True)

    if args.panel:
        panels = [p for p in PANELS if p[0] == args.panel]
        key, fname, title, _pos = panels[0]
        title = caps.get(key, title)
        fig, ax = plt.subplots(figsize=(13, 10))
        draw_panel(ax, out / fname, title, small=(key == "world"))
        fig.savefig(png_dir / f"{key}.png", dpi=150, bbox_inches="tight")
        if not args.png_only:
            plt.show()
        print(f"已存 {png_dir / (key + '.png')}")
        return

    # 汇总仪表盘：3 行 2 列，world_view 跨两列
    fig = plt.figure(figsize=(17, 16))
    fig.suptitle("laser_prep_lab — 2D 激光清洗链可视化（Intel Lab）",
                 fontsize=14)
    gs = fig.add_gridspec(3, 2)
    ax_map = {
        "frame_best": fig.add_subplot(gs[0, 0]),
        "ghost_zoom": fig.add_subplot(gs[0, 1]),
        "deskew_demo": fig.add_subplot(gs[1, 0]),
        "deskew_best": fig.add_subplot(gs[1, 1]),
        "world": fig.add_subplot(gs[2, :]),
    }
    for key, fname, title, _pos in PANELS:
        draw_panel(ax_map[key], out / fname, caps.get(key, title),
                   small=(key == "world"))
    fig.tight_layout(rect=(0, 0, 1, 0.97))

    # 每张面板单独存一份 PNG（细节放大看）
    for key, fname, title, _pos in PANELS:
        fig2, ax2 = plt.subplots(figsize=(12, 9))
        draw_panel(ax2, out / fname, caps.get(key, title),
                   small=(key == "world"))
        fig2.savefig(png_dir / f"{key}.png", dpi=150, bbox_inches="tight")
        plt.close(fig2)
        print(f"已存 {png_dir / (key + '.png')}")

    fig.savefig(png_dir / "dashboard.png", dpi=120, bbox_inches="tight")
    print(f"已存 {png_dir / 'dashboard.png'}")
    if not args.png_only:
        plt.show()


if __name__ == "__main__":
    main()
