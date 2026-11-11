#!/bin/bash
# ============================================================
#  控制器实验室可视化仿真（Gazebo + Nav2 + RViz 一体化启动）
#
#  全局规划器：nav2_astar_planner::AStarPlanner
#  默认控制器：nav2_controller_lab::PurePursuitController
#
#  启动后：
#   1. RViz 里点 "2D Pose Estimate"，在 (-2.0, -0.5) 拖出朝向
#   2. 若终端报 Aborting bringup，运行 ./activate_nav.sh
#   3. 点 "Nav2 Goal" 选目标点
#
#  运行时切换：RViz 左侧 "Controller Lab" 面板的 5 个按钮，
#  或发布 /controller_selector（std_msgs/String，transient_local）。
#
#  调试可视化：面板勾选“显示调试标记”，查看 /controller_lab/markers。
# ============================================================
# 无条件 source 三级环境。
# 不能只在 ROS_DISTRO 为空时 source：很多终端的 .bashrc 已经设置了
# ROS_DISTRO=jazzy，但并没有 source 本工作空间，结果就是找不到新包。
source /opt/ros/jazzy/setup.bash

if [ -f "$HOME/ros2_jazzy/install/setup.bash" ]; then
    source "$HOME/ros2_jazzy/install/setup.bash"
fi

if [ -f "$HOME/ros2_jazzy/nav2_astar_ws/install/setup.bash" ]; then
    source "$HOME/ros2_jazzy/nav2_astar_ws/install/setup.bash"
fi

# 启动前确认两个包都已编译，避免 ros2 launch 报难以理解的 "package not found"。
for pkg in nav2_astar_planner nav2_controller_lab nav2_controller_lab_rviz; do
    if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
        echo "[ERROR] 找不到包：$pkg"
        echo
        echo "请先编译："
        echo "  cd ~/ros2_jazzy/nav2_astar_ws"
        echo "  source /opt/ros/jazzy/setup.bash"
        echo "  source ~/ros2_jazzy/install/setup.bash"
        echo "  colcon build --packages-select nav2_astar_planner nav2_controller_lab nav2_controller_lab_rviz \\"
        echo "    --cmake-args -DCMAKE_BUILD_TYPE=Release"
        echo "  source install/setup.bash"
        exit 1
    fi
done

export GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share/nav2_minimal_tb3_sim/models:/opt/ros/jazzy/share${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export DISPLAY="${DISPLAY:-:0}"

exec ros2 launch nav2_controller_lab controller_lab_launch.py \
  headless:=False use_rviz:=True use_composition:=False
