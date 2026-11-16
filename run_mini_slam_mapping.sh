#!/usr/bin/env bash
# Start the Gazebo simulation, mini_slam, RViz, and keyboard teleoperation.
set -o pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT" || exit 1

for setup in /opt/ros/jazzy/setup.bash "$HOME/ros2_ws/install/setup.bash" "$ROOT/install/setup.bash"; do
  if [[ ! -f "$setup" ]]; then
    echo "[ERROR] Missing ROS setup: $setup" >&2
    exit 1
  fi
  # shellcheck disable=SC1090
  source "$setup"
done

if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "[ERROR] No graphical display. Start this script in a WSLg graphical session." >&2
  exit 1
fi
if ! command -v x-terminal-emulator >/dev/null 2>&1; then
  echo "[ERROR] x-terminal-emulator is required to open the keyboard window." >&2
  exit 1
fi
for package in mini_slam nav2_controller_lab nav2_minimal_tb3_sim; do
  if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
    echo "[ERROR] ROS package not found: $package. Build the workspace first." >&2
    exit 1
  fi
done

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share/nav2_minimal_tb3_sim/models:/opt/ros/jazzy/share${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

simulation_pid=""
slam_pid=""
terminal_pid=""
cleanup() {
  trap - INT TERM EXIT
  echo "[mini_slam] Stopping processes started by this script..."
  [[ -n "$terminal_pid" ]] && kill -TERM "$terminal_pid" 2>/dev/null || true
  [[ -n "$slam_pid" ]] && kill -TERM -- "-$slam_pid" 2>/dev/null || true
  [[ -n "$simulation_pid" ]] && kill -TERM -- "-$simulation_pid" 2>/dev/null || true
  [[ -n "$terminal_pid" ]] && wait "$terminal_pid" 2>/dev/null || true
  [[ -n "$slam_pid" ]] && wait "$slam_pid" 2>/dev/null || true
  [[ -n "$simulation_pid" ]] && wait "$simulation_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "[mini_slam] Starting Gazebo and RViz..."
setsid ros2 launch "$ROOT/src/mini_slam/ros/launch/mini_slam_simulation.launch.py"   headless:=False use_rviz:=True use_visualizer:=True use_composition:=False   "rviz_config_file:=$ROOT/src/mini_slam/ros/mini_slam_mapping.rviz" "params_file:=$ROOT/src/mini_slam/ros/mini_slam_nav2_params.yaml" &
simulation_pid=$!

echo "[mini_slam] Starting online mapping..."
setsid ros2 run mini_slam mini_slam_node --ros-args   -p use_sim_time:=true -r /map:=/mini/map &
slam_pid=$!

echo "[mini_slam] Opening keyboard control terminal..."
terminal_program="$(readlink -f "$(command -v x-terminal-emulator)")"
if [[ "$terminal_program" == */zutty ]]; then
  x-terminal-emulator -fontpath /usr/share/fonts/truetype/dejavu -font DejaVuSansMono \
    -e python3 "$ROOT/src/mini_slam/scripts/mapping_console.py" &
else
  x-terminal-emulator -e python3 "$ROOT/src/mini_slam/scripts/mapping_console.py" &
fi
terminal_pid=$!

echo "[mini_slam] W/S move, A/D turn, space stops, Q exits."
echo "[mini_slam] Close the keyboard window or press Ctrl-C here to stop the session."
wait -n "$simulation_pid" "$slam_pid" "$terminal_pid"
status=$?
if [[ $status -ne 0 ]]; then
  echo "[mini_slam] A session process exited with status $status." >&2
fi
exit "$status"
