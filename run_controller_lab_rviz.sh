#!/bin/bash
# 备用：单独启动 RViz（当一体化启动仍有问题时使用）
# 使用 nav2_controller_lab 自带的 RViz 配置（已移除 Docking 面板）
source /opt/ros/jazzy/setup.bash
source /home/horizain/ros2_jazzy/install/setup.bash
source /home/horizain/ros2_jazzy/nav2_astar_ws/install/setup.bash
export DISPLAY="${DISPLAY:-:0}"
exec rviz2 -d /home/horizain/ros2_jazzy/nav2_astar_ws/install/nav2_controller_lab/share/nav2_controller_lab/rviz/controller_lab_view.rviz
