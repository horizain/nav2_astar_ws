#!/bin/bash
# ============================================================
#  设好 "2D Pose Estimate" 之后，激活导航栈
#
#  WHY: navigation 生命周期管理器启动时要求 global_costmap 能拿到
#       map -> base_link 变换，这依赖 AMCL 先收到初始位姿。
#       位姿未设则 bringup 会 abort（nav2 既定流程，非 bug）。
#
#  用 Python 单进程批量调用服务，约 3 秒完成；逐个 ros2 lifecycle
#  CLI 需 60+ 秒（每次都要重做 DDS 发现）。
# ============================================================
# 无条件 source 三级环境；不能只判断 ROS_DISTRO。
source /opt/ros/jazzy/setup.bash

if [ -f "$HOME/ros2_jazzy/install/setup.bash" ]; then
    source "$HOME/ros2_jazzy/install/setup.bash"
fi

if [ -f "$HOME/ros2_jazzy/nav2_astar_ws/install/setup.bash" ]; then
    source "$HOME/ros2_jazzy/nav2_astar_ws/install/setup.bash"
fi

exec python3 "$HOME/ros2_jazzy/nav2_astar_ws/src/nav2_astar_planner/activate_nodes.py" --activate
