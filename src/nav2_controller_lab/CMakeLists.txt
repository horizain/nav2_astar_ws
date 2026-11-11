cmake_minimum_required(VERSION 3.8)
project(nav2_controller_lab)

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

set(library_name ${PROJECT_NAME})

set(dependencies
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

add_library(${library_name} SHARED
  src/controller_base.cpp
  src/path_utils.cpp
  src/trajectory_utils.cpp
  src/trajectory_cost.cpp
  src/pid_core.cpp
  src/pure_pursuit_core.cpp
  src/stanley_core.cpp
  src/dwb_core.cpp
  src/mppi_core.cpp
  src/mpc_core.cpp
  src/pid_controller.cpp
  src/pure_pursuit_controller.cpp
  src/stanley_controller.cpp
  src/dwb_controller.cpp
  src/mppi_controller.cpp
  src/mpc_controller.cpp
)

ament_target_dependencies(${library_name} ${dependencies})

# pluginlib 导出符号可见性
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB_DISABLE_BOOST_FUNCTIONS")

# 告诉 nav2_core 的 pluginlib：本包含控制器插件
pluginlib_export_plugin_description_file(nav2_core controller_plugins.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # 纯算法层测试：不链接 ROS，只测数学和状态机
  ament_add_gtest(test_controller_core
    test/test_controller_core.cpp
    src/path_utils.cpp
    src/trajectory_utils.cpp
    src/trajectory_cost.cpp
    src/pid_core.cpp
    src/pure_pursuit_core.cpp
    src/stanley_core.cpp
    src/dwb_core.cpp
    src/mppi_core.cpp
    src/mpc_core.cpp
  )
  target_include_directories(test_controller_core PRIVATE include)

  ament_add_gtest(test_plugin_load test/test_plugin_load.cpp)
  target_include_directories(test_plugin_load PRIVATE include)
  ament_target_dependencies(test_plugin_load
    nav2_core
    pluginlib
    rclcpp
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
