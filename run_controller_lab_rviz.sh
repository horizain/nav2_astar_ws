#!/bin/bash
# 备用：单独启动 RViz（当一体化启动仍有问题时使用）
# 使用 nav2_controller_lab 自带的 RViz 配置（已移除 Docking 面板）
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
# 上层工作空间 underlay（存在才加载，路径与仓库位置无关）
if [ -f "$SCRIPT_DIR/../../install/setup.bash" ]; then
  source "$SCRIPT_DIR/../../install/setup.bash"
fi
source "$SCRIPT_DIR/install/setup.bash"
export DISPLAY="${DISPLAY:-:0}"
exec rviz2 -d "$SCRIPT_DIR/install/nav2_controller_lab/share/nav2_controller_lab/rviz/controller_lab_view.rviz"
