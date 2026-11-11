#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""零依赖的栅格寻路练习工具（教学用）。

它故意不依赖 ROS，只做一件事：把 A* / Dijkstra / BFS 在 ASCII 地图上的
行为完整打印出来。你可以用它验证文档里的手算过程，也可以改参数观察
「扩展节点数、路径长度、路径代价」如何变化。

地图格式（纯文本，每行等长）：
    #   障碍
    .   空地
    S   起点
    G   终点
    1-9 带地形代价的格子（9 最贵，仅 A*/Dijkstra 使用）

示例：
    python3 astar_playground.py
    python3 astar_playground.py --trace
    python3 astar_playground.py --algo dijkstra --no-diagonal
    python3 astar_playground.py --compare
    python3 astar_playground.py --map my_map.txt --weight 2.0

注意：BFS 只保证「步数最少」，不保证「距离最短」，也不使用地形代价。
"""

from __future__ import annotations

import argparse
import heapq
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

# 内置的 5x5 小地图，文档里的手算示例就是它。
# 行号从上往下增大，列号从左往右增大。
DEFAULT_MAP = """#####
#S..#
#.#.#
#..G#
#####"""

# 稍大一点的演示地图，适合看搜索范围的差异。
DEMO_MAP = """##########
#S.......#
#.##.###.#
#....#...#
#.##.#.#.#
#....#...#
#.#####..#
#.......G#
##########"""

DIRS_4 = [(1, 0), (-1, 0), (0, 1), (0, -1)]
DIRS_8 = DIRS_4 + [(1, 1), (1, -1), (-1, 1), (-1, -1)]


@dataclass
class Cell:
    x: int
    y: int
    ch: str

    @property
    def terrain(self) -> int:
        """把 '1'-'9' 转成 1..9 的地形代价，其它字符为 0。"""
        return int(self.ch) if self.ch.isdigit() else 0


@dataclass
class Grid:
    cells: list[list[Cell]]
    width: int
    height: int
    start: tuple[int, int]
    goal: tuple[int, int]

    def inside(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height

    def cell(self, x: int, y: int) -> Cell:
        return self.cells[y][x]

    def passable(self, x: int, y: int) -> bool:
        return self.inside(x, y) and self.cell(x, y).ch != "#"


def parse_map(text: str) -> Grid:
    lines = [line.rstrip("\n") for line in text.strip("\n").splitlines()]
    if not lines:
        raise ValueError("地图为空")
    width = len(lines[0])
    if any(len(line) != width for line in lines):
        raise ValueError("地图每一行的长度必须相同")

    cells: list[list[Cell]] = []
    start = goal = None
    for y, line in enumerate(lines):
        row: list[Cell] = []
        for x, ch in enumerate(line):
            if ch not in "#.SG123456789":
                raise ValueError(f"未知地图字符 {ch!r}，位置 ({x},{y})")
            if ch == "S":
                if start is not None:
                    raise ValueError("地图只能有一个 S")
                start = (x, y)
            elif ch == "G":
                if goal is not None:
                    raise ValueError("地图只能有一个 G")
                goal = (x, y)
            row.append(Cell(x, y, ch))
        cells.append(row)

    if start is None or goal is None:
        raise ValueError("地图必须包含 S 和 G")
    return Grid(cells, width, len(lines), start, goal)


@dataclass
class Result:
    algo: str
    found: bool
    path: list[tuple[int, int]] = field(default_factory=list)
    expanded: int = 0
    opened: int = 0
    cost: float = 0.0
    elapsed_ms: float = 0.0
    trace: list[str] = field(default_factory=list)


def heuristic(a: tuple[int, int], b: tuple[int, int], diagonal: bool) -> float:
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    if diagonal:
        # Octile：8 邻域下的精确最小步数
        return (dx + dy) + (math.sqrt(2.0) - 2.0) * min(dx, dy)
    # Manhattan：4 邻域下的精确最小步数
    return dx + dy


def search(
    grid: Grid,
    algo: str,
    diagonal: bool,
    weight: float,
    terrain_weight: float,
    trace: bool,
    trace_limit: int = 200,
) -> Result:
    """在栅格上搜索。algo: astar / dijkstra / bfs。"""
    if algo == "dijkstra":
        weight = 0.0  # Dijkstra 的 h 恒为 0
    elif algo == "bfs":
        diagonal = False  # BFS 只按「步数」比较，固定 4 邻域更直观
        weight = 0.0
        terrain_weight = 0.0

    dirs = DIRS_8 if diagonal else DIRS_4
    start, goal = grid.start, grid.goal

    if not grid.passable(*goal):
        return Result(algo, found=False)

    # g_score：起点到某格子的已知最小代价
    g_score: dict[tuple[int, int], float] = {start: 0.0}
    # parent：用于回溯路径
    parent: dict[tuple[int, int], tuple[int, int]] = {}
    # closed：已经正式扩展过的节点
    closed: set[tuple[int, int]] = set()
    # 堆元素：(f, g, 序号, 坐标)。序号保证 f 相同时按插入顺序出队，
    # 让演示结果可复现。
    counter = 0
    open_heap: list[tuple[float, float, int, tuple[int, int]]] = []
    h0 = 0.0 if algo in ("dijkstra", "bfs") else heuristic(start, goal, diagonal) * weight
    heapq.heappush(open_heap, (h0, 0.0, counter, start))
    counter += 1

    opened = 1
    expanded = 0
    trace_lines: list[str] = []
    found = False
    t0 = time.perf_counter()

    while open_heap:
        f, g, _, cur = heapq.heappop(open_heap)

        # 懒删除：堆里可能同时存在同一节点的多个旧副本
        if cur in closed:
            continue
        closed.add(cur)
        expanded += 1

        if trace and len(trace_lines) < trace_limit:
            trace_lines.append(
                f"第 {expanded:>3} 次扩展: 弹出 ({cur[0]},{cur[1]}) "
                f"g={g:.2f} h={f - g:.2f} f={f:.2f}"
            )

        if cur == goal:
            found = True
            break

        cx, cy = cur
        for dx, dy in dirs:
            nx, ny = cx + dx, cy + dy
            if not grid.passable(nx, ny):
                continue
            nxt = (nx, ny)
            if nxt in closed:
                continue

            move = math.hypot(dx, dy)

            # 对角「切角」检查：两个相邻直移格都必须能走，
            # 否则会从两个障碍的缝隙里斜穿过去。
            if dx != 0 and dy != 0:
                if not grid.passable(cx + dx, cy) or not grid.passable(cx, cy + dy):
                    continue

            # 地形代价：把 1..9 归一化到 0..1，再乘系数
            extra = 0.0
            if terrain_weight > 0.0:
                extra = terrain_weight * (grid.cell(nx, ny).terrain / 9.0)

            tentative_g = g + move + extra
            if tentative_g < g_score.get(nxt, math.inf):
                g_score[nxt] = tentative_g
                parent[nxt] = cur
                if algo in ("dijkstra", "bfs"):
                    h = 0.0
                else:
                    h = heuristic(nxt, goal, diagonal) * weight
                heapq.heappush(open_heap, (tentative_g + h, tentative_g, counter, nxt))
                counter += 1
                opened += 1

    elapsed = (time.perf_counter() - t0) * 1000.0

    path: list[tuple[int, int]] = []
    if found:
        node = goal
        while True:
            path.append(node)
            if node == start:
                break
            node = parent[node]
        path.reverse()

    return Result(
        algo=algo,
        found=found,
        path=path,
        expanded=expanded,
        opened=opened,
        cost=g_score.get(goal, math.inf) if found else math.inf,
        elapsed_ms=elapsed,
        trace=trace_lines,
    )


def render(grid: Grid, path: list[tuple[int, int]], color: bool) -> str:
    on_path = set(path)
    out: list[str] = []
    for y in range(grid.height):
        line: list[str] = []
        for x in range(grid.width):
            ch = grid.cell(x, y).ch
            if (x, y) == grid.start:
                ch = "S"
            elif (x, y) == grid.goal:
                ch = "G"
            elif (x, y) in on_path and ch == ".":
                ch = "*"
            if color and ch == "*":
                line.append("\033[1;32m*\033[0m")
            elif color and ch == "S":
                line.append("\033[1;36mS\033[0m")
            elif color and ch == "G":
                line.append("\033[1;35mG\033[0m")
            elif color and ch == "#":
                line.append("\033[90m#\033[0m")
            else:
                line.append(ch)
        out.append("".join(line))
    return "\n".join(out)


def result_line(r: Result, label: str | None = None) -> str:
    name = label if label is not None else r.algo
    if not r.found:
        return (
            f"{name:<12} 未找到路径      "
            f"扩展={r.expanded:<5} 打开={r.opened:<5} 耗时={r.elapsed_ms:.3f} ms"
        )
    return (
        f"{name:<12} 路径格数={len(r.path):<4} 代价={r.cost:<7.2f} "
        f"扩展={r.expanded:<5} 打开={r.opened:<5} 耗时={r.elapsed_ms:.3f} ms"
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="A*/Dijkstra/BFS 栅格寻路练习工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--map", type=Path, help="地图文件路径；缺省用内置 5x5 小地图")
    parser.add_argument(
        "--algo",
        choices=["astar", "dijkstra", "bfs"],
        default="astar",
        help="算法（默认 astar）",
    )
    parser.add_argument(
        "--diagonal",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="是否允许 8 邻域（默认允许；--no-diagonal 为 4 邻域）",
    )
    parser.add_argument(
        "--weight",
        type=float,
        default=1.0,
        help="A* 启发式权重 w：1.0 最优，>1 更快但可能不是最短路",
    )
    parser.add_argument(
        "--terrain-weight",
        type=float,
        default=0.0,
        help="地形代价系数：0 表示只看距离",
    )
    parser.add_argument("--trace", action="store_true", help="打印每次扩展的节点")
    parser.add_argument("--trace-limit", type=int, default=200, help="最多打印多少条扩展记录")
    parser.add_argument("--compare", action="store_true", help="对比四种配置")
    parser.add_argument("--demo-map", action="store_true", help="使用内置的较大演示地图")
    parser.add_argument("--no-color", action="store_true", help="关闭 ANSI 颜色")
    args = parser.parse_args()

    if args.map is not None:
        text = args.map.read_text(encoding="utf-8")
    elif args.demo_map:
        text = DEMO_MAP
    else:
        text = DEFAULT_MAP

    try:
        grid = parse_map(text)
    except ValueError as exc:
        print(f"地图解析失败: {exc}", file=sys.stderr)
        return 2

    color = sys.stdout.isatty() and not args.no_color

    if args.compare:
        configs = [
            ("astar", args.diagonal, 1.0, args.terrain_weight),
            ("astar(w=2)", args.diagonal, 2.0, args.terrain_weight),
            ("dijkstra", args.diagonal, 1.0, args.terrain_weight),
            ("bfs", False, 1.0, 0.0),
        ]
        print(f"地图: {grid.width}x{grid.height}  起点={grid.start} 终点={grid.goal}")
        print(f"邻域: {'8' if args.diagonal else '4'}  地形权重: {args.terrain_weight}")
        print("-" * 78)
        for name, diag, w, tw in configs:
            algo = "astar" if name.startswith("astar") else name
            r = search(grid, algo, diag, w, tw, trace=False)
            print(result_line(r, name))
        print("-" * 78)
        print("提示: BFS 的『代价/最短』是按步数算的，不是按欧氏距离。")
        return 0

    r = search(
        grid,
        args.algo,
        args.diagonal,
        args.weight,
        args.terrain_weight,
        trace=args.trace,
        trace_limit=args.trace_limit,
    )

    print(f"地图: {grid.width}x{grid.height}  起点={grid.start} 终点={grid.goal}")
    print(
        f"算法={args.algo} 邻域={'8' if args.diagonal else '4'} "
        f"w={args.weight} 地形权重={args.terrain_weight}"
    )
    print("-" * 78)
    print(render(grid, r.path, color))
    print("-" * 78)
    print(result_line(r))
    if args.trace:
        print("\n扩展顺序（f 最小的先弹出）:")
        for line in r.trace:
            print("  " + line)
        if r.expanded > len(r.trace):
            print(f"  ...（共 {r.expanded} 次扩展，只显示前 {len(r.trace)} 条）")
        print(
            "\n注意: f 相同是常见现象。A* 只要 h 可采纳，"
            "第一次弹出终点时得到的就是最优代价。"
        )
    return 0 if r.found else 1


if __name__ == "__main__":
    raise SystemExit(main())
