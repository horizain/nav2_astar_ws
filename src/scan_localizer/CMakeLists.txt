cmake_minimum_required(VERSION 3.16)
project(scan_localizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

set(SCAN_LOCALIZER_WARN_FLAGS -Wall -Wextra -Wpedantic)

# ============================================================================
# 算法核心库（无 ROS 依赖，无仿真代码，可独立复用/单测/交叉编译）
# ============================================================================
add_library(scan_localizer_core STATIC
  src/map_field.cpp
  src/map_io.cpp
  src/matchers.cpp
  src/particle_filter.cpp
  src/localizer_core.cpp
  src/blackbox_recorder.cpp
  src/async_global_worker.cpp
)
target_include_directories(scan_localizer_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(scan_localizer_core PUBLIC Eigen3::Eigen Threads::Threads)
set_property(TARGET scan_localizer_core PROPERTY POSITION_INDEPENDENT_CODE ON)
target_compile_options(scan_localizer_core PRIVATE ${SCAN_LOCALIZER_WARN_FLAGS})

# ============================================================================
# 仿真世界辅助库（仅仿真节点、地图导出工具、基准测试与单测链接）
# ============================================================================
add_library(scan_localizer_sim_lib STATIC
  src/sim_world.cpp
)
target_link_libraries(scan_localizer_sim_lib PUBLIC scan_localizer_core)
set_property(TARGET scan_localizer_sim_lib PROPERTY POSITION_INDEPENDENT_CODE ON)
target_compile_options(scan_localizer_sim_lib PRIVATE ${SCAN_LOCALIZER_WARN_FLAGS})

# ============================================================================
# 基准测试（组件级对比实验：E1-E10）
# ============================================================================
add_executable(scan_localizer_benchmark test/benchmark.cpp)
target_link_libraries(scan_localizer_benchmark PRIVATE
  scan_localizer_core
  scan_localizer_sim_lib
)
target_compile_options(scan_localizer_benchmark PRIVATE ${SCAN_LOCALIZER_WARN_FLAGS})

# ============================================================================
# ROS 组件节点 + 仿真源
# ============================================================================
add_library(localizer_node_component SHARED
  src/localizer_node.cpp
)
ament_target_dependencies(localizer_node_component PUBLIC
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  sensor_msgs
  nav_msgs
  geometry_msgs
  diagnostic_msgs
  lifecycle_msgs
  std_msgs
  std_srvs
  tf2_ros
)
target_link_libraries(localizer_node_component PUBLIC scan_localizer_core)
target_compile_options(localizer_node_component PRIVATE ${SCAN_LOCALIZER_WARN_FLAGS})
rclcpp_components_register_node(localizer_node_component
  PLUGIN "sloc::LocalizerNode"
  EXECUTABLE scan_localizer_node
)

add_executable(scan_localizer_sim src/localizer_sim.cpp)
ament_target_dependencies(scan_localizer_sim PUBLIC
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
)
target_link_libraries(scan_localizer_sim PRIVATE
  scan_localizer_core
  scan_localizer_sim_lib
)
target_compile_options(scan_localizer_sim PRIVATE ${SCAN_LOCALIZER_WARN_FLAGS})

add_executable(sim_map_export src/sim_map_export.cpp)
target_link_libraries(sim_map_export PRIVATE
  scan_localizer_core
  scan_localizer_sim_lib
)
target_compile_options(sim_map_export PRIVATE ${SCAN_LOCALIZER_WARN_FLAGS})

# ============================================================================
# 安装
# ============================================================================
install(TARGETS scan_localizer_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS localizer_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS scan_localizer_benchmark scan_localizer_sim sim_map_export
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_se2 test/test_se2.cpp)
  if(TARGET test_se2)
    # ament_add_gtest internally uses plain target_link_libraries
    target_link_libraries(test_se2 scan_localizer_core)
    target_compile_options(test_se2 PRIVATE ${SCAN_LOCALIZER_WARN_FLAGS})
  endif()

  ament_add_gtest(test_map_io test/test_map_io.cpp)
  if(TARGET test_map_io)
    target_link_libraries(test_map_io scan_localizer_core)
    target_compile_options(test_map_io PRIVATE ${SCAN_LOCALIZER_WARN_FLAGS})
  endif()

  ament_add_gtest(test_map_field test/test_map_field.cpp)
  if(TARGET test_map_field)
    target_link_libraries(test_map_field scan_localizer_core)
    target_compile_options(test_map_field PRIVATE ${SCAN_LOCALIZER_WARN_FLAGS})
  endif()

  ament_add_gtest(test_matchers test/test_matchers.cpp)
  if(TARGET test_matchers)
    target_link_libraries(test_matchers scan_localizer_core scan_localizer_sim_lib)
    target_compile_options(test_matchers PRIVATE ${SCAN_LOCALIZER_WARN_FLAGS})
  endif()

  ament_add_gtest(test_particle_filter test/test_particle_filter.cpp)
  if(TARGET test_particle_filter)
    target_link_libraries(test_particle_filter scan_localizer_core scan_localizer_sim_lib)
    target_compile_options(test_particle_filter PRIVATE ${SCAN_LOCALIZER_WARN_FLAGS})
  endif()

  ament_add_gtest(test_localizer_core test/test_localizer_core.cpp)
  if(TARGET test_localizer_core)
    target_link_libraries(test_localizer_core scan_localizer_core scan_localizer_sim_lib)
    target_compile_options(test_localizer_core PRIVATE ${SCAN_LOCALIZER_WARN_FLAGS})
  endif()

  ament_add_gtest(test_localizer_node test/test_localizer_node.cpp)
  if(TARGET test_localizer_node)
    target_link_libraries(test_localizer_node localizer_node_component scan_localizer_sim_lib)
    target_compile_options(test_localizer_node PRIVATE ${SCAN_LOCALIZER_WARN_FLAGS})
  endif()

  ament_add_gtest(test_sim_world test/test_sim_world.cpp)
  if(TARGET test_sim_world)
    target_link_libraries(test_sim_world scan_localizer_core scan_localizer_sim_lib)
    target_compile_options(test_sim_world PRIVATE ${SCAN_LOCALIZER_WARN_FLAGS})
  endif()

  ament_add_gtest(test_benchmark_regression test/test_benchmark_regression.cpp)
  if(TARGET test_benchmark_regression)
    target_link_libraries(test_benchmark_regression scan_localizer_core scan_localizer_sim_lib)
    target_compile_options(test_benchmark_regression PRIVATE ${SCAN_LOCALIZER_WARN_FLAGS})
  endif()

  ament_add_gtest(test_generalization_matrix test/test_generalization_matrix.cpp)
  if(TARGET test_generalization_matrix)
    target_link_libraries(test_generalization_matrix scan_localizer_core scan_localizer_sim_lib)
    target_compile_options(test_generalization_matrix PRIVATE ${SCAN_LOCALIZER_WARN_FLAGS})
  endif()

  ament_add_gtest(test_blackbox_recorder test/test_blackbox_recorder.cpp)
  if(TARGET test_blackbox_recorder)
    target_link_libraries(test_blackbox_recorder scan_localizer_core)
    target_compile_options(test_blackbox_recorder PRIVATE ${SCAN_LOCALIZER_WARN_FLAGS})
  endif()

  ament_add_gtest(test_async_global_worker test/test_async_global_worker.cpp)
  if(TARGET test_async_global_worker)
    target_link_libraries(test_async_global_worker scan_localizer_core scan_localizer_sim_lib)
    target_compile_options(test_async_global_worker PRIVATE ${SCAN_LOCALIZER_WARN_FLAGS})
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
