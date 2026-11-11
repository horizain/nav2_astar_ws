#!/bin/bash
# 备用：单独启动 RViz（当一体化启动仍有问题时使用）
# 使用已移除 Docking 面板的配置
source /opt/ros/jazzy/setup.bash
source /home/horizain/ros2_jazzy/install/setup.bash
source /home/horizain/ros2_jazzy/nav2_astar_ws/install/setup.bash
export DISPLAY=:0
exec rviz2 -d /home/horizain/ros2_jazzy/nav2_astar_ws/install/nav2_astar_planner/share/nav2_astar_planner/rviz/nav2_astar_view.rviz
