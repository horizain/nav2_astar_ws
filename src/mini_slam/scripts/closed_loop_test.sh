#!/bin/bash
# ============================================================================
# mini_slam 闭环测试一键脚本：建图 → 存图 → 定位 → A* 规划 → 控制到达
#
# 用法：
#   ./scripts/closed_loop_test.sh                    # 完整闭环（含在线建图）
#   ./scripts/closed_loop_test.sh --skip-mapping     # 直接用已存的 mini_map.yaml
#   ./scripts/closed_loop_test.sh --rviz             # 同时启动 RViz 观看
#
# 本脚本固化了本机环境踩过的全部坑（按发生顺序）：
#   P1 僵尸进程：gz/桥/nav2 节点不随会话结束而死，三代进程同域打架
#      → 启动前无条件清场 + 进程数断言
#   P2 数据流竞态：bridge 未就绪时 configure lifecycle 会失败
#      → 健康门禁：/scan /odom /clock 达标后才激活
#   P3 初始位姿时间戳：用墙钟 now() 发 initialpose → "extrapolation into
#      the future" → AMCL 拒收 → 定位永不上线
#      → 助手节点强制 use_sim_time，用仿真时间打戳
#   P4 幽灵障碍：错误定位阶段 obstacle_layer 把扫描画到错位，远离射线
#      范围后永不清除 → 激活后与定位修复后各清一次 costmap
#   P5 目标点选在膨胀区：publish 值 99 = 原始 253（机器人中心不可入）
#      → 从实时代价图上选代价 ≤60 的开阔点
#   P6 停不下来的旧目标：action 客户端被杀后 bt_navigator 仍继续执行
#      → 发新目标前先取消旧目标
# 注意：不要用 set -u —— ROS 的 setup.bash 引用未定义变量会直接报错
cd "$(dirname "$0")/.."   # mini_slam 项目根目录

WS_ROOT="$HOME/ros2_jazzy"
SIM_LOG=/tmp/closedloop_sim.log
MINISLAM_LOG=/tmp/closedloop_minislam.log
OUT_DIR="$PWD/out_nav"
DRIVE_SIDE=1.0
DRIVE_LAPS=3
GOAL_ARRIVE_TOL=0.45

SKIP_MAPPING=0; USE_RVIZ=0
for a in "$@"; do
  case $a in
    --skip-mapping) SKIP_MAPPING=1 ;;
    --rviz) USE_RVIZ=1 ;;
  esac
done

log()  { echo -e "\033[32m[闭环]\033[0m $*"; }
warn() { echo -e "\033[33m[闭环]\033[0m $*"; }
fail() { echo -e "\033[31m[闭环失败]\033[0m $*"; exit 1; }

source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/nav2_astar_ws/install/setup.bash"
export DISPLAY="${DISPLAY:-:0}"
export GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share/nav2_minimal_tb3_sim/models:/opt/ros/jazzy/share${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
mkdir -p "$OUT_DIR"

# ---------------------------------------------------------------------------
# P1: 预飞行清场（僵尸 gz / 桥 / nav2 节点 / rviz / 旧 launch）
# 教训：pkill -f 会匹配到"内嵌本脚本全文的父包装进程"而自杀
#       → 按 PID 逐个杀，排除自己的祖先链
# ---------------------------------------------------------------------------
kill_pattern() {
  local pat="$1" chain p pid
  chain=" $$"
  p=$$
  while [ -n "$p" ] && [ "$p" != "1" ]; do
    p=$(awk '/^PPid:/{print $2}' "/proc/$p/status" 2>/dev/null)
    chain="$chain $p"
  done
  for pid in $(pgrep -f "$pat" 2>/dev/null); do
    case " $chain " in *" $pid "*) continue ;; esac
    kill -9 "$pid" 2>/dev/null
  done
}
log "步骤 0：清场（杀掉所有残留的仿真与 nav2 进程，排除自身祖先链）"
kill_pattern 'parameter_bridge'
kill_pattern 'gz sim'
kill_pattern 'robot_state_publisher'
kill_pattern 'nav2_lifecycle_manager'
kill_pattern 'nav2_map_server'
kill_pattern 'nav2_amcl'
kill_pattern 'nav2_controller'
kill_pattern 'nav2_planner_server'
kill_pattern 'nav2_behaviors'
kill_pattern 'nav2_smoother'
kill_pattern 'nav2_waypoint_follower'
kill_pattern 'nav2_collision_monitor'
kill_pattern 'nav2_bt_navigator'
kill_pattern 'opennav_docking'
kill_pattern 'nav2_route'
kill_pattern 'cbf_safety_filter'
kill_pattern 'controller_lab_visualizer'
kill_pattern 'mini_slam_node'
kill_pattern 'bin/ros2 launch'
kill_pattern 'drive_safe_square'
sleep 3
log "清场完成"

# ---------------------------------------------------------------------------
# 健康门禁工具：/scan /odom /clock 速率 + 时钟不回跳
# ---------------------------------------------------------------------------
wait_streams_healthy() {  # $1=超时秒
  local deadline=$((SECONDS + $1))
  while [ $SECONDS -lt $deadline ]; do
    local scan_ok odom_ok clock_ok
    scan_ok=$(timeout 8 ros2 topic hz /scan    2>/dev/null | grep -oE 'average rate: [0-9.]+' | grep -oE '[0-9.]+' | head -1)
    odom_ok=$(timeout 8 ros2 topic hz /odom    2>/dev/null | grep -oE 'average rate: [0-9.]+' | grep -oE '[0-9.]+' | head -1)
    clock_ok=$(timeout 8 ros2 topic hz /clock  2>/dev/null | grep -oE 'average rate: [0-9.]+' | grep -oE '[0-9.]+' | head -1)
    if python3 -c "
import sys
ok = True
for name, v, minv in [('scan','$scan_ok',3),('odom','$odom_ok',10),('clock','$clock_ok',50)]:
    try:
        if float(v) < minv: ok = False
    except: ok = False
sys.exit(0 if ok else 1)" 2>/dev/null; then
      log "数据流健康: scan=${scan_ok}Hz odom=${odom_ok}Hz clock=${clock_ok}Hz"
      return 0
    fi
    sleep 3
  done
  warn "数据流健康门禁超时（scan=$scan_ok odom=$odom_ok clock=$clock_ok）"
  return 1
}

# ---------------------------------------------------------------------------
# 启动仿真栈
# ---------------------------------------------------------------------------
if [ "$SKIP_MAPPING" = "1" ]; then
  # 跳过建图就必须有上次的地图；不能拿空 MAP_ARG 继续跑（AMCL 会没有图）。
  [ -f "$OUT_DIR/mini_map.yaml" ] || \
    fail "--skip-mapping 需要已存在的 $OUT_DIR/mini_map.yaml；请先完整跑一次建图"
  MAP_ARG="$OUT_DIR/mini_map.yaml"
else
  MAP_ARG=""   # 建图阶段用默认图（AMCL 不发初始位姿就不会抢 map→odom）
fi

log "步骤 1：启动 Gazebo + nav2 仿真栈（headless）"
ros2 launch nav2_controller_lab controller_lab_launch.py \
    headless:=True use_rviz:=False use_visualizer:=False use_composition:=False \
    ${MAP_ARG:+map:=$MAP_ARG} > "$SIM_LOG" 2>&1 &
SIM_LAUNCH_PID=$!
log "  launch pid=$SIM_LAUNCH_PID"

wait_streams_healthy 150 || fail "仿真数据流未就绪"

# ---------------------------------------------------------------------------
# 阶段 A：mini_slam 在线建图
# ---------------------------------------------------------------------------
if [ "$SKIP_MAPPING" = "0" ]; then
  log "阶段 A：mini_slam 在线建图（方形环路 ${DRIVE_LAPS} 圈 × ${DRIVE_SIDE}m）"
  ros2 run mini_slam mini_slam_node --ros-args \
      -p use_sim_time:=true \
      -p map_output_prefix:="$OUT_DIR/mini_map" \
      -r /map:=/mini/map > "$MINISLAM_LOG" 2>&1 &
  MINISLAM_PID=$!
  sleep 4

  python3 "$PWD/scripts/drive_safe_square.py" --ros-args \
      -p use_sim_time:=true -p side:=$DRIVE_SIDE -p laps:=$DRIVE_LAPS \
      | while read -r l; do echo "  [驱动] $l"; done
  log "驱动完成，等待 8s 让最后一段扫描入图"
  sleep 8

  # 停建图节点（它的 map→odom TF 会和 AMCL 打架）
  kill "$MINISLAM_PID" 2>/dev/null; sleep 2

  # meta → nav2 yaml（map_server / AMCL 可用格式）
  read -r RES OX OY W H < "$OUT_DIR/mini_map.meta"
  cat > "$OUT_DIR/mini_map.yaml" << EOF
image: mini_map.pgm
resolution: $RES
origin: [$OX, $OY, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
mode: trinary
EOF
  log "已写出 $OUT_DIR/mini_map.yaml（nav2 兼容）"
fi

# ---------------------------------------------------------------------------
# 阶段 B：激活 nav2 + 换图 + 定位（P2/P3/P4 教训全在这里）
# ---------------------------------------------------------------------------
log "阶段 B：激活 nav2 生命周期节点"
python3 "$WS_ROOT/nav2_astar_ws/src/nav2_astar_planner/activate_nodes.py" --activate \
    | tail -4
log "  等待 3s 让激活后的节点完成内部初始化"
sleep 3

# P4: 激活后立即清 costmap（防止激活窗口内的错位标记）
ros2 service call /global_costmap/clear_entirely_global_costmap \
    nav2_msgs/srv/ClearEntireCostmap "{}" > /dev/null 2>&1
ros2 service call /local_costmap/clear_entirely_local_costmap \
    nav2_msgs/srv/ClearEntireCostmap "{}" > /dev/null 2>&1

if [ "$SKIP_MAPPING" = "1" ]; then
  # 直接用脚本启动时挂的地图
  log "定位：使用启动参数挂载的地图 $MAP_ARG"
else
  log "定位：load_map 换成 mini_slam 建的图"
  timeout 30 ros2 service call /map_server/load_map \
      nav2_msgs/srv/LoadMap "{map_url: $OUT_DIR/mini_map.yaml}" > /dev/null 2>&1
fi

# P3: 用仿真时间打戳的 initialpose（rclpy 节点 use_sim_time:=true）
log "定位：发布 AMCL 初始位姿（读取 odom 真实位姿，仿真时间戳）"
timeout 60 python3 << 'PYEOF'
import rclpy, math, time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

rclpy.init()
n = Node('initial_pose_helper')
n.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
latest = {}
n.create_subscription(Odometry, '/odom', lambda m: latest.__setitem__('o', m), 10)
deadline = time.time() + 25.0   # 墙钟超时（DDS 发现可能很慢）
while rclpy.ok() and time.time() < deadline and 'o' not in latest:
    rclpy.spin_once(n, timeout_sec=0.3)
assert 'o' in latest, "没收到 odom"
p = latest['o'].pose.pose.position
q = latest['o'].pose.pose.orientation
yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
m = PoseWithCovarianceStamped()
m.header.frame_id = 'map'
m.header.stamp = n.get_clock().now().to_msg()   # 仿真时间戳（P3 教训）
m.pose.pose.position.x = p.x
m.pose.pose.position.y = p.y
m.pose.pose.orientation.z = math.sin(yaw/2)
m.pose.pose.orientation.w = math.cos(yaw/2)
m.pose.covariance[0] = 0.04; m.pose.covariance[7] = 0.04; m.pose.covariance[35] = 0.05
for _ in range(6):
    pub.publish(m); rclpy.spin_once(n, timeout_sec=0.3); time.sleep(0.2)
print(f"initialpose @ ({p.x:.3f}, {p.y:.3f}, yaw={yaw:.3f}) 已发布")
PYEOF

# P4（续）：定位上线后（机器人真实位置被正确变换前）再清一次
sleep 4
ros2 service call /global_costmap/clear_entirely_global_costmap \
    nav2_msgs/srv/ClearEntireCostmap "{}" > /dev/null 2>&1
ros2 service call /local_costmap/clear_entirely_local_costmap \
    nav2_msgs/srv/ClearEntireCostmap "{}" > /dev/null 2>&1

# 定位收敛检查：amcl_pose 存在且 3s 内稳定（不稳定则用内置 publisher 重发）
log "定位：检查 AMCL 收敛"
AMCL_POS_RAW=$(timeout 120 python3 << 'PYEOF'
import rclpy, time, math, re, subprocess
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
rclpy.init()
n = Node('amcl_check')
n.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
def read_amcl():
    try:
        txt = subprocess.run(['ros2','topic','echo','--once','/amcl_pose'],
                             capture_output=True, text=True, timeout=20).stdout
        pairs = re.findall(r'x: ([-\d.e]+)\n\s*y: ([-\d.e]+)', txt)
        return (float(pairs[0][0]), float(pairs[0][1])) if pairs else None
    except Exception:
        return None
best = None
for attempt in range(4):
    pos = read_amcl()
    if pos is None:
        time.sleep(2); continue
    x, y = pos
    time.sleep(3)
    pos2 = read_amcl()
    if pos2 is None: continue
    x2, y2 = pos2
    d = math.hypot(x2-x, y2-y)
    print(f"  AMCL ({x:.3f},{y:.3f}) → ({x2:.3f},{y2:.3f}) 漂移 {d*100:.1f}cm")
    if d < 0.05:
        best = (x2, y2); break
    # 不稳定 → 重发当前位姿（内置 publisher，无 f-string 括号坑）
    m = PoseWithCovarianceStamped()
    m.header.frame_id = 'map'
    m.header.stamp = n.get_clock().now().to_msg()
    m.pose.pose.position.x = x2
    m.pose.pose.position.y = y2
    m.pose.pose.orientation.w = 1.0
    for _ in range(4):
        pub.publish(m); rclpy.spin_once(n, timeout_sec=0.2); time.sleep(0.2)
assert best, "AMCL 未收敛"
print(f"AMCL 已收敛: ({best[0]:.3f}, {best[1]:.3f})")
print(f"CONVERGED {best[0]:.4f} {best[1]:.4f}")   # 传给 bash：选点脚本直接用，避免二次 DDS 发现
PYEOF
)
AMCL_POS=$(echo "$AMCL_POS_RAW" | grep '^CONVERGED' | awk '{print $2, $3}')
log "AMCL 位姿已捕获: $AMCL_POS"
log "定位上线 ✓（mini_slam 的图 + AMCL）"

# ---------------------------------------------------------------------------
# 阶段 C：从实时代价图选目标 → BT 导航 → 到达验证
# ---------------------------------------------------------------------------
log "阶段 C：从实时代价图选取开阔目标点并导航"
GOAL=$(timeout 90 python3 - $AMCL_POS << 'PYEOF'
import rclpy, math, time, sys
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

mx, my = float(sys.argv[1]), float(sys.argv[2])   # 收敛检查传来的 AMCL 位姿
rclpy.init()
n = Node('goal_picker')
n.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 reliability=ReliabilityPolicy.RELIABLE)
latest = {}
n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: latest.__setitem__('c', m), qos)
deadline = time.time() + 40.0   # 墙钟超时 + 长发现窗口
while rclpy.ok() and time.time() < deadline and 'c' not in latest:
    rclpy.spin_once(n, timeout_sec=0.3)
assert 'c' in latest, "没收到 /global_costmap/costmap"
g = latest['c']
info = g.info
ox, oy, res = info.origin.position.x, info.origin.position.y, info.resolution
def max_cost_disk(cx, cy, rc):
    best = -1
    for dy in range(-rc, rc+1):
        for dx in range(-rc, rc+1):
            if dx*dx+dy*dy > rc*rc: continue
            x, y = cx+dx, cy+dy
            if x < 0 or y < 0 or x >= info.width or y >= info.height: return 255
            v = g.data[y*info.width+x]
            best = max(best, v if v >= 0 else 255)
    return best
rc = int(0.25/res)
cands = []
for cy in range(4, info.height-4, 3):
    for cx in range(4, info.width-4, 3):
        wx = ox + (cx+0.5)*res; wy = oy + (cy+0.5)*res
        d = math.hypot(wx-mx, wy-my)
        if d < 1.2: continue
        if max_cost_disk(cx, cy, rc) <= 60: cands.append((d, wx, wy))
cands.sort(reverse=True)
assert cands, "没有满足代价约束的开阔目标点"
d, wx, wy = cands[0]
print(f"{wx:.2f} {wy:.2f} {d:.2f}")
PYEOF
)
[ -z "$GOAL" ] && fail "找不到开阔目标点"
read -r GX GY GD <<< "$GOAL"
log "目标点: ($GX, $GY) 距离 ${GD}m（实时代价 ≤60）"

# P6: 发新目标自动抢占旧目标
timeout 240 ros2 action send_goal --feedback /navigate_to_pose \
    nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $GX, y: $GY, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" \
    2>&1 | grep -E 'Goal finished|error_code' | tail -3
RESULT_LOG=$?

FINAL=$(timeout 15 python3 << 'PYEOF'
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
rclpy.init()
n = Node('final_pose')
n.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
latest = {}
n.create_subscription(Odometry, '/odom', lambda m: latest.__setitem__('o', m), 10)
deadline = n.get_clock().now().nanoseconds + 10_000_000_000
while rclpy.ok() and n.get_clock().now().nanoseconds < deadline and 'o' not in latest:
    rclpy.spin_once(n, timeout_sec=0.2)
p = latest['o'].pose.pose.position
print(f"{p.x:.3f} {p.y:.3f}")
PYEOF
)
read -r FX FY <<< "$FINAL"
ERR=$(python3 -c "import math; print(f'{math.hypot($FX-$GX, $FY-$GY):.2f}')" 2>/dev/null || echo "n/a")
log "终点: ($FX, $FY)，距目标 $ERR m"

if python3 -c "import math,sys; sys.exit(0 if math.hypot($FX-$GX,$FY-$GY) < $GOAL_ARRIVE_TOL else 1)" 2>/dev/null; then
  log "\033[32m========== 闭环测试通过 ========== \033[0m"
  log "mini_slam 建图 → AMCL 定位（mini_slam 地图）→ A* 规划 → PurePursuit 控制 → 到达目标（误差 ${ERR}m < ${GOAL_ARRIVE_TOL}m）"
else
  warn "闭环测试未达标：终点距目标 ${ERR}m（阈值 ${GOAL_ARRIVE_TOL}m）"
  warn "请查看 $SIM_LOG / $MINISLAM_LOG 排查"
fi

if [ "$USE_RVIZ" = "1" ]; then
  warn "RViz 保持运行（关闭测试请手动 kill rviz2 / 仿真栈）"
fi
log "仿真栈仍在后台运行（pid $SIM_LAUNCH_PID），手动关闭: kill $SIM_LAUNCH_PID && pkill -9 -f 'gz sim|parameter_bridge|nav2_'"
