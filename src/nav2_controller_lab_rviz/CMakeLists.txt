cmake_minimum_required(VERSION 3.8)
project(nav2_controller_lab_rviz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_controller_lab REQUIRED)

include_directories(include)

# ---- RViz Panel ----
add_library(controller_lab_panel SHARED
  src/controller_lab_panel.cpp
  include/nav2_controller_lab_rviz/controller_lab_panel.hpp
)

target_include_directories(controller_lab_panel PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
)

ament_target_dependencies(controller_lab_panel
  rclcpp
  rviz_common
  std_msgs
  geometry_msgs
  Qt5
)

target_link_libraries(controller_lab_panel
  rviz_common::rviz_common
  Qt5::Widgets
)

pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)

# ---- 调试可视化节点 ----
add_executable(controller_lab_visualizer
  src/controller_lab_visualizer.cpp
)

ament_target_dependencies(controller_lab_visualizer
  rclcpp
  nav2_controller_lab
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
)

install(TARGETS
  controller_lab_panel
  controller_lab_visualizer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES rviz_plugins.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(controller_lab_panel)
ament_export_dependencies(
  rclcpp
  rviz_common
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
)
ament_package()
