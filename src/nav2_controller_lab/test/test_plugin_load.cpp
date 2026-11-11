// Copyright (c) 2026
// pluginlib 加载测试：验证 5 个控制器都能被 ClassLoader 创建
//
// 这个测试需要包已经 install（colcon test 会设置好环境）。

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "nav2_core/controller.hpp"

TEST(ControllerPlugins, LoadsAllRegisteredControllers)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader(
    "nav2_core", "nav2_core::Controller");

  const std::vector<std::string> classes = {
    "nav2_controller_lab::PidController",
    "nav2_controller_lab::PurePursuitController",
    "nav2_controller_lab::StanleyController",
    "nav2_controller_lab::DwbController",
    "nav2_controller_lab::MppiController",
    "nav2_controller_lab::MpcController",
  };

  for (const auto & name : classes) {
    EXPECT_TRUE(loader.isClassAvailable(name)) << name;
    auto instance = loader.createUniqueInstance(name);
    EXPECT_NE(instance, nullptr) << name;
  }
}
