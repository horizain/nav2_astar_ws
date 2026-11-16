#!/usr/bin/env bash
# ============================================================================
# 跨进程 `ros2 param set` 回归测试（Phase 4A）
#
# 目的：验证"另一个进程里的 ros2 CLI 能发现节点、拿到参数服务、并触发
#       on_set_parameters 的接受/拒绝逻辑"。进程内 set_parameter 测试只覆盖
#       回调逻辑，覆盖不到 DDS 发现 + ros2 param 这条真实链路。
#
# 用法：
#   ./test_param_cli.sh <mini_slam_node 可执行文件> [节点额外参数...]
#   CMake 里用 $<TARGET_FILE:mini_slam_node> 传入。
#
# 本机（WSL2/Jazzy）实测的发现配置 —— 见 docs/07 的"跨进程 CLI 验证"：
#   - 默认 RMW = rmw_fastrtps_cpp：本机跨进程发现全部失败（node list 空，
#     param 报 "Node not found"），domain/SUBNET/LOCALHOST/静态 peer 都试过；
#   - rmw_cyclonedds_cpp + ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET：稳定可用；
#   - 同一个 Cyclone + LOCALHOST：node list 能看到节点，但参数服务发现超时
#     （"Wait for service timed out waiting for parameter services"）。
# 所以这里显式钉住 cyclone + SUBNET + 独立 domain，不依赖调用者 shell。
#
# 注意：`ros2 param set` 在"参数被回调拒绝"时返回码仍是 0（ros2cli 行为），
#       因此断言必须看输出文本，不能只看 $?。
# ============================================================================
set -u

NODE_BIN="${1:-}"
if [ -n "${1:-}" ]; then
  shift || true
fi

if [ -z "$NODE_BIN" ] || [ ! -x "$NODE_BIN" ]; then
  echo "[FAIL] 用法: $0 <mini_slam_node 可执行文件> [节点额外参数...]" >&2
  echo "       收到的路径: '${NODE_BIN}'" >&2
  exit 2
fi

# ---- 发现环境 ----
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-95}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET   # Cyclone 下 LOCALHOST 参数服务不可达
unset ROS_LOCALHOST_ONLY                      # 已弃用，不用它做局域限制

CLI_TIMEOUT=20
WORK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/mini_slam_param_cli.XXXXXX")"
NODE_LOG="$WORK_DIR/node.log"
NODE_PID=""

cleanup() {
  if [ -n "$NODE_PID" ] && kill -0 "$NODE_PID" 2>/dev/null; then
    kill -TERM "$NODE_PID" 2>/dev/null
    for _ in $(seq 1 50); do
      kill -0 "$NODE_PID" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL "$NODE_PID" 2>/dev/null
    wait "$NODE_PID" 2>/dev/null
  fi
}
trap cleanup EXIT INT TERM

FAILURES=0
fail() {
  echo "[FAIL] $*" >&2
  FAILURES=$((FAILURES + 1))
}

# ros2 CLI 包装：统一超时 + 合并 stderr（ros2 param 的报错在 stderr）。
cli() { timeout "$CLI_TIMEOUT" ros2 "$@" 2>&1; }

echo "== 环境: RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_AUTOMATIC_DISCOVERY_RANGE=$ROS_AUTOMATIC_DISCOVERY_RANGE"
echo "== 节点: $NODE_BIN ${*:-}"

"$NODE_BIN" "$@" >"$NODE_LOG" 2>&1 &
NODE_PID=$!

# ---- 1) 最多等 20s：ros2 node list 看到 /mini_slam_node ----
NODE_VISIBLE=0
for _ in $(seq 1 20); do
  if ! kill -0 "$NODE_PID" 2>/dev/null; then
    break
  fi
  if cli node list | grep -qx '/mini_slam_node'; then
    NODE_VISIBLE=1
    break
  fi
  sleep 1
done

if [ "$NODE_VISIBLE" -ne 1 ]; then
  fail "20s 内 ros2 node list 没看到 /mini_slam_node（DDS 发现失败或节点已退出）"
else
  echo "[ OK ] ros2 node list 看到 /mini_slam_node"
fi

# ---- 2) 参数服务可见 ----
PARAM_LIST=""
if [ "$FAILURES" -eq 0 ]; then
  PARAM_LIST="$(cli param list /mini_slam_node)"
  if printf '%s\n' "$PARAM_LIST" | grep -q 'minimum_travel_distance'; then
    echo "[ OK ] ros2 param list 列出 minimum_travel_distance"
  else
    fail "ros2 param list 未列出 minimum_travel_distance（参数服务不可达）"
  fi
fi

# ---- 3) 合法参数：set 0.7 成功 + get 读回 0.7 ----
if [ "$FAILURES" -eq 0 ]; then
  OUT="$(cli param set /mini_slam_node minimum_travel_distance 0.7)"
  if printf '%s' "$OUT" | grep -q 'Set parameter successful'; then
    echo "[ OK ] param set minimum_travel_distance=0.7 成功"
  else
    fail "param set minimum_travel_distance=0.7 未成功: $OUT"
  fi
  OUT="$(cli param get /mini_slam_node minimum_travel_distance)"
  if printf '%s' "$OUT" | grep -Eq 'value is: *0\.7([0]*)?$|: *0\.7$'; then
    echo "[ OK ] param get minimum_travel_distance == 0.7"
  else
    fail "param get minimum_travel_distance 不是 0.7: $OUT"
  fi
fi

# ---- 4) 非法值：越界必须被回调拒绝，且旧值不被写入 ----
if [ "$FAILURES" -eq 0 ]; then
  OUT="$(cli param set /mini_slam_node loop_match_minimum_response_coarse -1.0)"
  if printf '%s' "$OUT" | grep -q 'Setting parameter failed' &&
     ! printf '%s' "$OUT" | grep -q 'Set parameter successful'; then
    echo "[ OK ] 非法值 loop_match_minimum_response_coarse=-1.0 被拒绝"
    printf '        输出: %s\n' "$OUT"
  else
    fail "非法值 loop_match_minimum_response_coarse=-1.0 未被拒绝: $OUT"
  fi
  OUT="$(cli param get /mini_slam_node loop_match_minimum_response_coarse)"
  if printf '%s' "$OUT" | grep -q '0\.4'; then
    echo "[ OK ] 拒绝后参数仍是原值 0.4"
  else
    fail "拒绝后参数被意外改写: $OUT"
  fi
fi

# ---- 5) 运行时不支持修改项：必须带明确原因 ----
if [ "$FAILURES" -eq 0 ]; then
  OUT="$(cli param set /mini_slam_node scan_topic scan2)"
  if printf '%s' "$OUT" | grep -q '运行时不支持修改'; then
    echo "[ OK ] scan_topic 运行期修改被拒绝并给出原因"
    printf '        输出: %s\n' "$OUT"
  else
    fail "scan_topic 运行期修改未返回'运行时不支持修改': $OUT"
  fi
  OUT="$(cli param get /mini_slam_node scan_topic)"
  if printf '%s' "$OUT" | grep -q 'value is: scan$'; then
    echo "[ OK ] 拒绝后 scan_topic 仍是 'scan'"
  else
    fail "拒绝后 scan_topic 被意外改写: $OUT"
  fi
fi

# ---- 诊断（成功也打印一遍，便于对比基线）----
if [ "$FAILURES" -ne 0 ]; then
  echo "---- 诊断: ros2 node list ----" >&2
  cli node list >&2
  echo "---- 诊断: ros2 param list ----" >&2
  cli param list /mini_slam_node >&2
  echo "---- 诊断: 节点日志 ----" >&2
  tail -n 40 "$NODE_LOG" >&2
fi

if [ "$FAILURES" -ne 0 ]; then
  echo "[FAIL] test_param_cli: $FAILURES 项失败" >&2
  exit 1
fi

echo "[PASS] test_param_cli（跨进程 node list / param list / set / get / 拒绝路径）"
exit 0
