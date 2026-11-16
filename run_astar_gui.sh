#!/bin/bash
# ============================================================
#  A* 规划器可视化仿真（Gazebo + 导航栈 + RViz 一体化启动）
#
#  前置：脚本会无条件 source 三级环境（/opt → 外层工作空间 → 本工作空间），
#        并检查 nav2_astar_planner 是否已编译。
#
#  RViz 使用自定义配置 nav2_astar_view.rviz（移除了会段错误的 Docking 面板），
#  故可与导航栈在同一 launch 中稳定运行。
#
#  启动后操作：
#   1. RViz 里点 "2D Pose Estimate"，在 (-2.0, -0.5) 拖出朝向
#   2. 若终端报 Aborting bringup，运行 ./activate_nav.sh
#   3. 点 "Nav2 Goal" 选目标点
# ============================================================
# 无条件 source 三级环境；不能只判断 ROS_DISTRO。
source /opt/ros/jazzy/setup.bash

if [ -f "$HOME/ros2_jazzy/install/setup.bash" ]; then
    source "$HOME/ros2_jazzy/install/setup.bash"
fi

if [ -f "$HOME/ros2_jazzy/nav2_astar_ws/install/setup.bash" ]; then
    source "$HOME/ros2_jazzy/nav2_astar_ws/install/setup.bash"
fi

if ! ros2 pkg prefix nav2_astar_planner >/dev/null 2>&1; then
    echo "[ERROR] 找不到 nav2_astar_planner，请先编译："
    echo "  cd ~/ros2_jazzy/nav2_astar_ws"
    echo "  colcon build --packages-select nav2_astar_planner \\"
    echo "    --cmake-args -DCMAKE_BUILD_TYPE=Release"
    echo "  source install/setup.bash"
    echo
    echo "若 install/ 前缀损坏（曾删过 build/），用："
    echo "  cd ~/ros2_jazzy/nav2_astar_ws && ./check_workspace.sh --fix"
    exit 1
fi

export GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share/nav2_minimal_tb3_sim/models:/opt/ros/jazzy/share${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export DISPLAY="${DISPLAY:-:0}"

exec ros2 launch nav2_astar_planner astar_simulation_launch.py \
  headless:=False use_rviz:=True use_composition:=False
