cmake_minimum_required(VERSION 3.8)
project(nav2_astar_planner)

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

set(library_name ${PROJECT_NAME})

set(dependencies
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros
  builtin_interfaces
)

add_library(${library_name} SHARED
  src/astar_planner.cpp
  src/astar_algorithm.cpp
)

ament_target_dependencies(${library_name} ${dependencies})

# pluginlib 需要导出符号可见性
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB_DISABLE_BOOST_FUNCTIONS")

# 让 nav2_core 的 pluginlib 能通过 plugin xml 找到本插件
pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_planner_contract test/test_planner_contract.cpp)
  target_link_libraries(test_planner_contract ${library_name})
  ament_target_dependencies(test_planner_contract ${dependencies})
  set_tests_properties(test_planner_contract PROPERTIES TIMEOUT 60
    ENVIRONMENT "ROS_DOMAIN_ID=96")

  ament_add_gtest(test_astar_algorithm
    test/test_astar_algorithm.cpp
    src/astar_algorithm.cpp
  )
  target_include_directories(test_astar_algorithm PRIVATE include)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
