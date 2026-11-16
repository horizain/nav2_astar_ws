#!/bin/bash
# ============================================================
#  可视化端到端实验（Gazebo GUI + RViz 一体，非无头）
#
#  用法：
#    ./run_e2e_gui.sh                    # 只启动可视化栈（导航在另一终端跑）
#    ./run_e2e_gui.sh --auto             # 启动后自动跑一次端到端导航（默认 DWB）
#    ./run_e2e_gui.sh --auto MPPI        # 指定控制器（DWB/MPPI/PID/Stanley/MPC/FollowPath）
#    ./run_e2e_gui.sh --auto DWB --nocbf # 关掉 CBF 过滤器对比（对照实验）
#
#  三个窗口：Gazebo（物理世界）+ RViz（导航世界观）+ 实时面板（轨迹/h(t)/速度）
#  配套：docs/15 §7（复现步骤）、docs/17（CBF 实验）、docs/18（调优）
# ============================================================
# 无条件三级 source（doc 18 的环境规范）
source /opt/ros/jazzy/setup.bash
if [ -f "$HOME/ros2_jazzy/install/setup.bash" ]; then
    source "$HOME/ros2_jazzy/install/setup.bash"
fi
if [ -f "$HOME/ros2_jazzy/nav2_astar_ws/install/setup.bash" ]; then
    source "$HOME/ros2_jazzy/nav2_astar_ws/install/setup.bash"
fi
unset FASTRTPS_DEFAULT_PROFILES_FILE
# DDS 发现域限制本机（docs/18 技巧 4）——多机协作时删掉这一行
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share/nav2_minimal_tb3_sim/models:/opt/ros/jazzy/share"
export DISPLAY="${DISPLAY:-:0}"

# ---- 参数解析 ----
AUTO_CTRL=""
USE_CBF="True"
while [ $# -gt 0 ]; do
    case "$1" in
        --auto) AUTO_CTRL="${2:-DWB}"; shift 2 ;;
        --nocbf) USE_CBF="False"; shift ;;
        *) echo "未知参数: $1"; exit 1 ;;
    esac
done

# ---- 启动前检查（包没编译时给出明确指引）----
for pkg in nav2_astar_planner nav2_controller_lab nav2_controller_lab_rviz; do
    if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
        echo "[ERROR] 找不到包：$pkg —— 先编译："
        echo "  cd ~/ros2_jazzy/nav2_astar_ws && colcon build --packages-select \\"
        echo "    nav2_astar_planner nav2_controller_lab nav2_controller_lab_rviz \\"
        echo "    --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash"
        echo "若 install/ 前缀损坏（曾删过 build/），用："
        echo "  cd ~/ros2_jazzy/nav2_astar_ws && ./check_workspace.sh --fix"
        exit 1
    fi
done

# ---- 启动前清理（残留进程是万恶之源，docs/03 §6.3）----
for pat in "[n]av2_controller" "[n]av2_bt_navigator" "[n]av2_planner" "[n]av2_amcl" \
           "[n]av2_map_server" "[n]av2_smoother" "[n]av2_behaviors" \
           "[n]av2_velocity_smoother" "[n]av2_collision_monitor" \
           "[g]z sim" "[g]z-sim" "[p]arameter_bridge" "[r]obot_state_publisher" \
           "[r]os2launch" "[r]viz2" "[c]bf_safety_filter"; do
    pkill -9 -f "$pat" 2>/dev/null
done
sleep 1
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*

# ---- 观看指南 ----
cat <<'GUIDE'

============================================================
  可视化模式启动中（约 60-90 秒后窗口弹出）
============================================================
  Gazebo 窗口：看物理世界的机器人本体
  RViz 窗口  ：看导航系统的"世界观"，重点盯这几层：
    ├─ 黑色栅格 Map        = AMCL 在用的静态地图
    ├─ 彩色费用图          = 局部/全局代价地图（膨胀场）
    ├─ 绿色/彩色 Path      = A* 全局路径（每秒重规划一次）
    ├─ 橙色点云            = 激光实际打到的障碍
    ├─ 多边形 Polygon      = 机器人足迹与膨胀边界
    └─ Controller Lab 面板 = 六种控制器运行时切换按钮

  RViz 里手动玩导航：工具栏 "2D Pose Estimate" 设初始位姿
  （拖到出生点附近），再用 "Nav2 Goal" 点任意目标。

============================================================
GUIDE

# ---- 启动可视化栈 ----
if [ -n "$AUTO_CTRL" ]; then
    # --auto：launch 后台跑，e2e 前台跑，结果一目了然
    ros2 launch nav2_controller_lab controller_lab_launch.py \
      headless:=False use_rviz:=True use_visualizer:=True \
      use_composition:=False use_cbf:="$USE_CBF" &
    LAUNCH_PID=$!
    # 实时面板（第三个窗口）：地图轨迹 + h(t) + 标称vs CBF
    python3 "$HOME/ros2_jazzy/nav2_astar_ws/docs/tools/live_monitor.py"       --d-safe 0.22 &
    VIZ_PID=$!
    echo "[等待栈就绪 65s...]（gzclient/rviz2/实时面板 三窗口应已弹出）"
    sleep 65
    echo "[自动导航开始：控制器=$AUTO_CTRL CBF=$USE_CBF]"
    python3 "$HOME/ros2_jazzy/nav2_astar_ws/docs/tools/e2e_nav_test.py" \
      --controller "$AUTO_CTRL" --timeout 300
    RC=$?
    if [ $RC -eq 0 ]; then
        echo "[✅ 到达目标] 窗口保持打开，可直接用 RViz 的 Nav2 Goal 再点新目标玩"
    else
        echo "[❌ 导航失败 rc=$RC] 查 launch 日志，或 grep CBFSafetyFilter 看拦截"
    fi
    echo "[按 Ctrl-C 关闭全部窗口]"
    trap 'kill $LAUNCH_PID $VIZ_PID 2>/dev/null' INT TERM EXIT
    wait $LAUNCH_PID
else
    # 默认：launch 前台跑，Ctrl-C 一键全关；导航在另一终端：
    #   python3 docs/tools/e2e_nav_test.py --controller DWB
    exec ros2 launch nav2_controller_lab controller_lab_launch.py \
      headless:=False use_rviz:=True use_visualizer:=True \
      use_composition:=False use_cbf:="$USE_CBF"
fi
