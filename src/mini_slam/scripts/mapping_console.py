#!/usr/bin/env python3
"""Interactive keyboard mapping, saved-map localization and Nav2 handoff."""
import math
from pathlib import Path
import select
import sys
import termios
import time
import tty

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

MAP_DIR = Path.home() / "mini_slam_maps"


class ServiceOutcomeUnknown(RuntimeError):
    # 请求已发出，但没有收到最终结果。
    pass


class Console(Node):
    def __init__(self):
        super().__init__("mini_slam_mapping_console")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.linear, self.angular = 0.15, 0.7
        self.motion = (0, 0)
        self.expiry = 0.0
        self.manual = True
        self.state_uncertain = False
        self.saved = None
        self.action_status = {"navigate_to_pose": {}, "follow_path": {}}
        self.odom_velocity = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, "/mini/display_map", qos)
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/mini/map", self.on_mapping_map, qos)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.status_subs = [
            self.create_subscription(
                GoalStatusArray, f"/{name}/_action/status",
                lambda msg, key=name: self.on_action_status(key, msg), qos)
            for name in self.action_status
        ]
        self.settings = termios.tcgetattr(sys.stdin)

    def on_mapping_map(self, msg):
        if self.manual:
            self.map_pub.publish(msg)

    def on_action_status(self, name, msg):
        self.action_status[name] = {
            bytes(item.goal_info.goal_id.uuid): item.status for item in msg.status_list
        }

    def on_odom(self, msg):
        velocity = msg.twist.twist
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.odom_velocity = (time.monotonic(), stamp, velocity.linear.x,
                              velocity.linear.y, velocity.angular.z)

    def stop(self):
        self.motion = (0, 0)
        self.pub.publish(Twist())
        rclpy.spin_once(self, timeout_sec=0.05)

    def call(self, typ, name, request, timeout=15):
        client = self.create_client(typ, name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError("Service unavailable: " + name)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            if not future.done() or future.exception():
                raise ServiceOutcomeUnknown("Service result unknown: " + name)
            return future.result()
        finally:
            self.destroy_client(client)

    def set_parameter(self, name, value):
        request = SetParameters.Request()
        request.parameters = [Parameter(name, value=value).to_parameter_msg()]
        result = self.call(SetParameters, "/mini_slam_node/set_parameters", request)
        if not result.results or not result.results[0].successful:
            raise RuntimeError("Parameter rejected: " + name + " " +
                               (result.results[0].reason if result.results else ""))

    def graph(self, op):
        result = self.call(Trigger, "/" + op + "_graph", Trigger.Request(), 30)
        if not result.success:
            raise RuntimeError(result.message)
        print(result.message, flush=True)

    def cancel_navigation(self):
        pending = {}
        for name in ("navigate_to_pose", "follow_path"):
            response = self.call(CancelGoal, f"/{name}/_action/cancel_goal",
                                 CancelGoal.Request(), 5)
            if response.return_code != CancelGoal.Response.ERROR_NONE:
                raise RuntimeError(f"{name} cancellation failed; keep teleop disabled")
            pending[name] = {bytes(info.goal_id.uuid) for info in response.goals_canceling}

        # 取消响应只表示请求被接受，必须等目标进入终态才允许手动接管。
        terminal = {GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                    GoalStatus.STATUS_ABORTED}
        deadline = time.monotonic() + 10.0
        while any(pending.values()):
            rclpy.spin_once(self, timeout_sec=0.1)
            for name, ids in pending.items():
                statuses = self.action_status[name]
                ids.difference_update(
                    {goal_id for goal_id in ids if statuses.get(goal_id) in terminal})
            if time.monotonic() >= deadline:
                raise ServiceOutcomeUnknown("Action cancellation did not reach a terminal state")

        # 控制动作已终止后，速度平滑链仍可能输出一段非零速度。
        # 持续发零并等待新鲜里程计连续静止，才能开放手动接管。
        deadline = time.monotonic() + 10.0
        stopped_since = None
        stopped_stamp = None
        while time.monotonic() < deadline:
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            sample = self.odom_velocity
            stopped = (sample is not None and now - sample[0] < 0.5 and
                       all(math.isfinite(v) for v in sample[2:]) and
                       math.hypot(sample[2], sample[3]) < 0.03 and
                       abs(sample[4]) < 0.08)
            if stopped:
                if stopped_since is None:
                    stopped_since = now
                    stopped_stamp = sample[1]
                elif now - stopped_since >= 0.5 and sample[1] > stopped_stamp:
                    return
            else:
                stopped_since = None
                stopped_stamp = None
        raise ServiceOutcomeUnknown("Odometry did not confirm a stable stop")

    def prompt_prefix(self, prompt):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        try:
            value = input(prompt).strip()
        finally:
            tty.setcbreak(sys.stdin.fileno())
        if not value:
            raise ValueError("No map name entered")
        path = Path(value).expanduser()
        if not path.is_absolute():
            path = MAP_DIR / path
        if path.suffix in (".yaml", ".pgm", ".mmpg"):
            path = path.with_suffix("")
        return path.resolve()

    def save(self):
        if not self.manual:
            raise RuntimeError("Switch to mapping mode before saving")
        prefix = self.prompt_prefix("Map name (relative to ~/mini_slam_maps): ")
        if any(Path(str(prefix) + ext).exists()
               for ext in (".mmpg", ".mmpg.tmp", ".yaml", ".yaml.tmp",
                           ".pgm", ".pgm.tmp", ".png", ".png.tmp",
                           ".meta", ".meta.tmp")):
            raise RuntimeError("A map or temporary file already exists; choose a new name")
        prefix.parent.mkdir(parents=True, exist_ok=True)
        self.set_parameter("graph_path", str(prefix) + ".mmpg")
        result = self.call(Trigger, "/save_snapshot", Trigger.Request(), 60)
        if not result.success:
            raise RuntimeError("Snapshot save failed: " + result.message)
        if not all(Path(str(prefix) + ext).is_file()
                   for ext in (".yaml", ".pgm", ".mmpg")):
            raise RuntimeError("Map files incomplete; inspect " + str(prefix))
        self.saved = prefix
        print("Saved graph and map:", prefix, flush=True)

    def load_static_map(self, prefix):
        request = LoadMap.Request()
        request.map_url = str(prefix) + ".yaml"
        response = self.call(LoadMap, "/map_server/load_map", request, 30)
        if response.result != LoadMap.Response.RESULT_SUCCESS:
            raise RuntimeError("Nav2 map load failed, result=" + str(response.result))
        return response.map

    def localize(self, prefix, load_graph):
        if not all(Path(str(prefix) + ext).is_file() for ext in (".yaml", ".pgm", ".mmpg")):
            raise RuntimeError("Need matching .yaml, .pgm and .mmpg files")
        was_manual = self.manual
        # 服务响应不确定时不得继续手动输出；地图加载也可能已在服务端执行。
        self.manual = False
        try:
            grid = self.load_static_map(prefix)
            # 先冻结建图再加载位姿图，避免异步扫描混入上一会话里程计。
            self.set_parameter("mode", "localization")
            if load_graph:
                self.set_parameter("graph_path", str(prefix) + ".mmpg")
                self.graph("load")
        except ServiceOutcomeUnknown:
            # 请求可能仍在服务端执行，不能与之竞争回滚或恢复手动控制。
            raise
        except (RuntimeError, ValueError, OSError):
            # 恢复上一次导航地图和图路径，避免失败后地图服务器与位姿图不一致。
            if self.saved is not None:
                try:
                    self.load_static_map(self.saved)
                    self.set_parameter("graph_path", str(self.saved) + ".mmpg")
                except ServiceOutcomeUnknown as restore_error:
                    raise ServiceOutcomeUnknown(
                        "Previous map restoration result unknown") from restore_error
                except (RuntimeError, ValueError, OSError) as restore_error:
                    print("ERROR: failed to restore previous map:", restore_error,
                          flush=True)
            if was_manual:
                try:
                    self.set_parameter("mode", "mapping")
                except ServiceOutcomeUnknown as restore_error:
                    raise ServiceOutcomeUnknown(
                        "Mapping mode restoration result unknown") from restore_error
                except (RuntimeError, ValueError, OSError) as restore_error:
                    print("ERROR: failed to restore mapping mode:", restore_error,
                          flush=True)
                else:
                    self.manual = True
            raise
        self.map_pub.publish(grid)
        self.saved = prefix
        print("Localization/navigation mode. Teleop stopped; use RViz 2D Pose Estimate",
              "if needed, then Nav2 Goal. A* + project Pure Pursuit are configured.",
              flush=True)

    def mapping(self):
        if not self.manual:
            self.cancel_navigation()

        self.set_parameter("mode", "mapping")
        self.manual = True
        print("Mapping/manual mode. W/S move, A/D turn, space stops.", flush=True)

    def operation(self, key):
        if self.state_uncertain:
            print("ERROR: state uncertain; restart the session before operating", flush=True)
            return
        self.stop()
        try:
            if key == "1":
                self.mapping()
            elif key == "2":
                if self.saved is None:
                    raise RuntimeError("Save a map with 3 or load one with 4 first")
                if not self.manual:
                    self.cancel_navigation()
                self.localize(self.saved, False)
            elif key == "3":
                self.save()
            elif key == "4":
                if not self.manual:
                    self.cancel_navigation()
                self.localize(self.prompt_prefix("Existing map name or full prefix: "), True)
        except ServiceOutcomeUnknown as exc:
            self.state_uncertain = True
            self.manual = False
            print("ERROR:", exc, "- state uncertain; restart the session", flush=True)
        except (RuntimeError, ValueError, OSError) as exc:
            print("ERROR:", exc, flush=True)

    def run(self):
        tty.setcbreak(sys.stdin.fileno())
        try:
            print("W/S forward/back, A/D turn, SPACE stop; R/F linear speed; T/G turn speed")
            print("1 mapping/manual | 2 localization/navigation | 3 save | 4 load | Q quit")
            print("In navigation mode choose Nav2 Goal in RViz; key 1 cancels navigation first.")
            # Printed only after cbreak is active: no observer may send input on an earlier banner.
            print("Console ready", flush=True)
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    key = sys.stdin.read(1).lower()
                    if key == "q":
                        break
                    if key in "1234":
                        self.operation(key)
                    elif self.manual and key in "wsad":
                        self.motion = {"w": (1, 0), "s": (-1, 0),
                                       "a": (0, 1), "d": (0, -1)}[key]
                        self.expiry = time.monotonic() + 0.3
                    elif self.manual and key == " ":
                        self.motion = (0, 0)
                    elif key in "rftg":
                        if key == "r": self.linear = min(0.5, self.linear + 0.05)
                        if key == "f": self.linear = max(0.05, self.linear - 0.05)
                        if key == "t": self.angular = min(2.0, self.angular + 0.1)
                        if key == "g": self.angular = max(0.1, self.angular - 0.1)
                        print("linear=%.2f angular=%.2f" % (self.linear, self.angular), flush=True)
                if self.manual:
                    if time.monotonic() >= self.expiry:
                        self.motion = (0, 0)
                    msg = Twist()
                    msg.linear.x = self.motion[0] * self.linear
                    msg.angular.z = self.motion[1] * self.angular
                    self.pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            if rclpy.ok() and self.manual:
                self.stop()


def main():
    if not sys.stdin.isatty():
        raise SystemExit("Run this console in an interactive terminal")
    rclpy.init()
    node = Console()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
