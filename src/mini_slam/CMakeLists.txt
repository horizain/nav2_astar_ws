cmake_minimum_required(VERSION 3.22)
project(mini_slam VERSION 0.1.0 LANGUAGES CXX)

# ---- 全局编译设置：C++23 ----
# 本机 GCC 13.3：format / expected / span / ranges 可用；
# deducing this / std::print / ranges::to 是 GCC 14 的，代码用探测宏兜底。
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

# ---------------- 算法核：纯 C++，零 ROS 依赖 ----------------
  add_library(mini_slam_core STATIC
  core/src/likelihood_field.cpp
  core/src/correlative_matcher.cpp
  core/src/pose_graph.cpp
  core/src/spa_solver.cpp
  core/src/mapper.cpp
  core/src/map_builder.cpp
  core/src/map_output.cpp
  core/src/serialization.cpp
)
target_include_directories(mini_slam_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/include>")
target_compile_features(mini_slam_core PUBLIC cxx_std_23)
target_link_libraries(mini_slam_core PUBLIC Ceres::ceres Eigen3::Eigen)

# ---------------- 测试与特性探针 ----------------
enable_testing()
foreach(t IN ITEMS feature_probe test_field test_matcher test_spa
                test_mapper_synthetic test_localization test_serialization
                test_covariance test_connected_chain test_sparsify
                test_angle_wrap test_grid_boundary test_odom_at
                test_validation test_map_output test_map_incremental
                test_interactive_move test_snapshot)
  add_executable(${t} app/tests/${t}.cpp)
  target_link_libraries(${t} PRIVATE mini_slam_core)
  add_test(NAME ${t} COMMAND ${t})
endforeach()

# ---------------- 数据集入口与对拍工具（app/，同样零 ROS 依赖）----------------
add_executable(mini_slam_dataset
  app/run_dataset.cpp
  app/carmen_reader.cpp
)
target_link_libraries(mini_slam_dataset PRIVATE mini_slam_core)

add_executable(mini_slam_compare app/compare_maps.cpp)
target_link_libraries(mini_slam_compare PRIVATE mini_slam_core)

# ---------------- ROS 包装：有 ament 环境才编（-DMINI_SLAM_WITH_ROS=OFF 可关）----------------
# 注意：find_package(ament_cmake) 在缺 ament_package Python 模块的环境里会硬报错，
# 所以必须把探测本身也放进开关里。
option(MINI_SLAM_WITH_ROS "在 ament 环境下编译 ROS 包装层" ON)
if(MINI_SLAM_WITH_ROS)
  find_package(ament_cmake QUIET)
endif()
if(ament_cmake_FOUND)
  find_package(rclcpp QUIET)
  if(rclcpp_FOUND)
    find_package(sensor_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(std_srvs REQUIRED)
    find_package(tf2_ros REQUIRED)
    find_package(interactive_markers REQUIRED)
    find_package(visualization_msgs REQUIRED)
    add_executable(mini_slam_node ros/mini_slam_node.cpp)
    # Jazzy 的包都导出了命名空间目标，用 keyword 签名统一链接
    # （ament_target_dependencies 是老式 plain 签名，和 PRIVATE 混用会报错）
    target_link_libraries(mini_slam_node PRIVATE
      mini_slam_core
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs
      nav_msgs::nav_msgs
      geometry_msgs::geometry_msgs
      std_srvs::std_srvs
      tf2_ros::tf2_ros
      interactive_markers::interactive_markers
      visualization_msgs::visualization_msgs)
    install(TARGETS mini_slam_node DESTINATION lib/${PROJECT_NAME})
    install(DIRECTORY ros/launch DESTINATION share/${PROJECT_NAME})

    # 控制终端：载图结果未知时禁止手动接管，明确失败时恢复原状态。
    add_test(NAME test_console_failures
             COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/ros/tests/test_console_failures.py)
    set_tests_properties(test_console_failures PROPERTIES TIMEOUT 30)

    # ROS 级并发回归：worker 灌帧与 save/load/stats/timer 同时运行。
    add_executable(test_node_concurrency ros/tests/test_node_concurrency.cpp)
    target_link_libraries(test_node_concurrency PRIVATE
      mini_slam_core
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs
      nav_msgs::nav_msgs
      geometry_msgs::geometry_msgs
      std_srvs::std_srvs
      tf2_ros::tf2_ros
      interactive_markers::interactive_markers
      visualization_msgs::visualization_msgs)
    add_test(NAME test_node_concurrency COMMAND test_node_concurrency)
    set_tests_properties(test_node_concurrency PROPERTIES
      ENVIRONMENT "ROS_DOMAIN_ID=87;ROS_LOCALHOST_ONLY=1"
      TIMEOUT 120)

    # 定位初始化：自动加载图 + /initialpose → map→odom TF
    add_executable(test_localization_init ros/tests/test_localization_init.cpp)
    target_link_libraries(test_localization_init PRIVATE
      mini_slam_core
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs
      nav_msgs::nav_msgs
      geometry_msgs::geometry_msgs
      std_srvs::std_srvs
      tf2_ros::tf2_ros
      interactive_markers::interactive_markers
      visualization_msgs::visualization_msgs)
    add_test(NAME test_localization_init COMMAND test_localization_init)
    set_tests_properties(test_localization_init PROPERTIES
      ENVIRONMENT "ROS_DOMAIN_ID=86;ROS_LOCALHOST_ONLY=1"
      TIMEOUT 120)

    # 交互式 marker（Phase 4C）：/mini_slam/update 出 marker +
    # /mini_slam/feedback 的 MOUSE_UP → move_vertex → SPA 链路。
    add_executable(test_interactive_markers ros/tests/test_interactive_markers.cpp)
    target_link_libraries(test_interactive_markers PRIVATE
      mini_slam_core
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs
      nav_msgs::nav_msgs
      geometry_msgs::geometry_msgs
      std_srvs::std_srvs
      tf2_ros::tf2_ros
      interactive_markers::interactive_markers
      visualization_msgs::visualization_msgs)
    add_test(NAME test_interactive_markers COMMAND test_interactive_markers)
    set_tests_properties(test_interactive_markers PROPERTIES
      ENVIRONMENT "ROS_DOMAIN_ID=84;ROS_LOCALHOST_ONLY=1"
      TIMEOUT 120)

    # 跨进程 `ros2 param set`：脚本自己起节点进程，再用 ros2 CLI 走真实
    # DDS 发现 + 参数服务链路（进程内 set_parameter 覆盖不到这一段）。
    # 发现配置见脚本头注释与 docs/07：本机必须 cyclone + SUBNET。
    add_test(NAME test_param_cli
             COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/ros/tests/test_param_cli.sh
                     $<TARGET_FILE:mini_slam_node>)
    set_tests_properties(test_param_cli PROPERTIES
      ENVIRONMENT
        "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;ROS_DOMAIN_ID=95;ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET"
      TIMEOUT 120)
    message(STATUS "mini_slam: ROS wrapper enabled")
  else()
    message(STATUS "mini_slam: rclcpp not found -> 只编 core + app")
  endif()

  install(TARGETS mini_slam_core ARCHIVE DESTINATION lib)
  install(TARGETS mini_slam_dataset mini_slam_compare
          DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY core/include/ DESTINATION include)
  ament_package()
endif()
