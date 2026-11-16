// Grid 坐标约定：cell cx 覆盖 [origin+cx·res, origin+(cx+1)·res)，
// world_to_cell 与 cell_center 必须一致；并覆盖负坐标/出界。
#include <cmath>

#include "mini_slam/occupancy_grid.hpp"

#include "test_util.hpp"

using namespace mini_slam;

int main() {
  const double res = 0.05;
  const Grid g(res, Vec2{0.0, 0.0}, 100, 100);

  int cx = -999, cy = -999;
  CHECK(g.world_to_cell(Vec2{0.01, 0.01}, cx, cy));
  CHECK(cx == 0 && cy == 0);
  CHECK(g.world_to_cell(Vec2{0.04, 0.04}, cx, cy));
  CHECK(cx == 0 && cy == 0);
  CHECK(g.world_to_cell(Vec2{0.06, 0.06}, cx, cy));
  CHECK(cx == 1 && cy == 1);

  // 下边界属于 cell 0，cell_center(0) 落在 cell 0 内
  CHECK(g.world_to_cell(Vec2{0.0, 0.0}, cx, cy));
  CHECK(cx == 0 && cy == 0);
  CHECK(g.world_to_cell(g.cell_center(0, 0), cx, cy));
  CHECK(cx == 0 && cy == 0);
  CHECK(g.world_to_cell(g.cell_center(99, 99), cx, cy));
  CHECK(cx == 99 && cy == 99);

  // 负坐标：floor 语义（-0.01 → -1），并报告出界
  CHECK(!g.world_to_cell(Vec2{-0.01, 0.0}, cx, cy));
  CHECK(cx == -1);
  CHECK(!g.world_to_cell(Vec2{0.0, -0.05}, cx, cy));
  CHECK(cy == -1);

  // 上边界/出界
  CHECK(g.world_to_cell(Vec2{4.99, 4.99}, cx, cy));
  CHECK(cx == 99 && cy == 99);
  CHECK(!g.world_to_cell(Vec2{5.0, 0.0}, cx, cy));
  CHECK(cx == 100);

  std::puts("[PASS] test_grid_boundary");
  return 0;
}
