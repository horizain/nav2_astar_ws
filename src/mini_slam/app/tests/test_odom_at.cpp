// OdomBuffer 回退方向与角度插值：
//   早于最早 → 最早；晚于最新 → 最新（不是最早）；区间内 → 插值。
#include <cmath>

#include "mini_slam/odom_buffer.hpp"

#include "test_util.hpp"

using namespace mini_slam;

int main() {
  OdomBuffer buf(400);
  CHECK(!buf.at(0.0).has_value());  // 空缓冲

  buf.push(1.0, Pose2{0.0, 0.0, 0.0});
  buf.push(2.0, Pose2{1.0, 0.0, 0.5});
  buf.push(3.0, Pose2{2.0, 0.0, 1.0});

  // 早于最早
  auto p = buf.at(0.5);
  CHECK(p.has_value());
  CHECK(p->x == 0.0 && p->y == 0.0 && p->theta == 0.0);

  // 晚于最新：必须是 back()，旧实现错误地返回 front()
  p = buf.at(10.0);
  CHECK(p.has_value());
  CHECK(std::abs(p->x - 2.0) < 1e-12);
  CHECK(std::abs(p->theta - 1.0) < 1e-12);

  // 恰好端点
  p = buf.at(1.0);
  CHECK(std::abs(p->x) < 1e-12);
  p = buf.at(3.0);
  CHECK(std::abs(p->x - 2.0) < 1e-12);

  // 区间内线性插值
  p = buf.at(1.5);
  CHECK(p.has_value());
  CHECK(std::abs(p->x - 0.5) < 1e-12);
  CHECK(std::abs(p->theta - 0.25) < 1e-12);

  // 角度跨 ±π：3.10 → -3.10 的最短路径经过 ±π，而不是绕回 0
  OdomBuffer wrap_buf;
  wrap_buf.push(0.0, Pose2{0.0, 0.0, 3.10});
  wrap_buf.push(1.0, Pose2{0.0, 0.0, -3.10});
  p = wrap_buf.at(0.5);
  CHECK(p.has_value());
  CHECK(std::abs(std::abs(p->theta) - kPi) < 0.02);  // 中点应落在 ±π 附近

  p = wrap_buf.at(0.1);
  CHECK(p.has_value());
  CHECK(std::abs(p->theta - 3.1083185307) < 0.02);

  // 端点 theta 也统一到 [-π,π]
  p = wrap_buf.at(-1.0);
  CHECK(p.has_value());
  CHECK(p->theta <= kPi && p->theta >= -kPi);

  std::puts("[PASS] test_odom_at");
  return 0;
}
