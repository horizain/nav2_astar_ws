// 启动期参数集中校验：非法值必须被拒绝且错误信息带参数名。
// 另验证 core 的防御性护栏：即使绕过节点校验，也不会除零。
#include <memory>
#include <string>

#include "mini_slam/correlative_matcher.hpp"
#include "mini_slam/likelihood_field.hpp"
#include "mini_slam/map_builder.hpp"
#include "mini_slam/pose_graph.hpp"
#include "mini_slam/validation.hpp"

#include "test_util.hpp"

using namespace mini_slam;

int main() {
  const Params defaults;
  const NodeParams node_defaults;
  CHECK(validate_parameters(defaults, node_defaults).empty());  // 默认值合法

  {
    NodeParams n = node_defaults;
    n.processing_queue_size = 0;
    const auto err = validate_parameters(defaults, n);
    CHECK(!err.empty());
    CHECK(err.find("processing_queue_size") != std::string::npos);
  }
  {
    Params p = defaults;
    p.sparsify_every = 0;
    const auto err = validate_parameters(p, node_defaults);
    CHECK(!err.empty());
    CHECK(err.find("sparsify_every") != std::string::npos);
  }
  {
    Params p = defaults;
    p.loop_attempt_interval = 0;
    const auto err = validate_parameters(p, node_defaults);
    CHECK(!err.empty());
    CHECK(err.find("loop_attempt_interval") != std::string::npos);
  }
  {
    Params p = defaults;
    p.resolution = 0.0;
    const auto err = validate_parameters(p, node_defaults);
    CHECK(!err.empty());
    CHECK(err.find("resolution") != std::string::npos);
  }
  {
    Params p = defaults;
    p.coarse_lin_step = 0.0;
    const auto err = validate_parameters(p, node_defaults);
    CHECK(!err.empty());
    CHECK(err.find("coarse_lin_step") != std::string::npos);
  }
  {
    NodeParams n = node_defaults;
    n.map_update_interval = 0.0;
    CHECK(!validate_parameters(defaults, n).empty());
  }
  {
    NodeParams n = node_defaults;
    n.base_frame.clear();
    const auto err = validate_parameters(defaults, n);
    CHECK(!err.empty());
    CHECK(err.find("base_frame") != std::string::npos);
  }
  {
    Params p = defaults;
    p.loop_min_response_coarse = -0.1;
    const auto err = validate_parameters(p, node_defaults);
    CHECK(!err.empty());
    CHECK(err.find("loop_match_minimum_response_coarse") != std::string::npos);
  }
  {
    Params p = defaults;
    p.loop_min_response_fine = 1.1;
    const auto err = validate_parameters(p, node_defaults);
    CHECK(!err.empty());
    CHECK(err.find("loop_match_minimum_response_fine") != std::string::npos);
  }
  {
    Params p = defaults;
    p.match_min_response = -0.1;
    const auto err = validate_parameters(p, node_defaults);
    CHECK(!err.empty());
    CHECK(err.find("match_min_response") != std::string::npos);
  }
  {
    Params p = defaults;
    p.match_min_response = 1.1;
    const auto err = validate_parameters(p, node_defaults);
    CHECK(!err.empty());
    CHECK(err.find("match_min_response") != std::string::npos);
  }
  {
    Params p = defaults;
    p.loop_max_variance = -1.0;
    const auto err = validate_parameters(p, node_defaults);
    CHECK(!err.empty());
    CHECK(err.find("loop_max_variance") != std::string::npos);
  }
  {
    Params p = defaults;
    p.running_scan_distance = -1.0;
    const auto err = validate_parameters(p, node_defaults);
    CHECK(!err.empty());
    CHECK(err.find("running_scan_distance") != std::string::npos);
  }

  // ---- core 防御性护栏：绕过节点校验也不能除零 ----
  LaserScan scan;
  for (int i = 0; i < 10; ++i) {
    scan.points.push_back(Vec2{1.0 + 0.05 * i, 0.0});
  }
  const FramedScan ref{Pose2{}, scan.points};
  const auto field =
      LikelihoodField::build(std::span{&ref, 1}, 0.05, 1.0, 2);
  CHECK(field.has_value());

  MatchWindow zero_step;
  zero_step.lin_step = 0.0;
  zero_step.ang_step = 0.0;
  const auto m = CorrelativeMatcher::match(*field, Pose2{}, scan.points,
                                           zero_step);
  CHECK(!m.valid);

  PoseGraph g;
  g.add_vertex(Pose2{}, Pose2{},
               std::make_shared<const LaserScan>(scan));
  const auto bad_map = MapBuilder::render(g, 0.0, 10.0, 1.0);
  CHECK(bad_map.width == 0 && bad_map.height == 0);

  std::puts("[PASS] test_validation");
  return 0;
}
