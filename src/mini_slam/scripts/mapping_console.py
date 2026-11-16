#!/usr/bin/env python3
"""Interactive keyboard mapping, saved-map localization and Nav2 handoff."""
import os
from pathlib import Path
import select
import subprocess
import sys
import termios
import time
import tty

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger

MAP_DIR = Path.home() / "mini_slam_maps"


class Console(Node):
    def __init__(self):
        super().__init__("mini_slam_mapping_console")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.linear, self.angular = 0.15, 0.7
        self.motion = (0, 0)
        self.expiry = 0.0
        self.manual = True
        self.saved = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, "/mini/display_map", qos)
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/mini/map", self.on_mapping_map, qos)
        self.settings = termios.tcgetattr(sys.stdin)

    def on_mapping_map(self, msg):
        if self.manual:
            self.map_pub.publish(msg)

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
                raise RuntimeError("Service failed or timed out: " + name)
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
        response = self.call(CancelGoal, "/navigate_to_pose/_action/cancel_goal",
                             CancelGoal.Request(), 5)
        if response.return_code != CancelGoal.Response.ERROR_NONE:
            raise RuntimeError("Navigation cancellation failed; keep teleop disabled")

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
        if any(Path(str(prefix) + ext).exists() for ext in (".mmpg", ".yaml", ".pgm")):
            raise RuntimeError("A map file already exists; choose a new name")
        prefix.parent.mkdir(parents=True, exist_ok=True)
        self.set_parameter("graph_path", str(prefix) + ".mmpg")
        self.graph("save")
        try:
            result = subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli",
                 "-t", "/mini/map", "-f", str(prefix)],
                capture_output=True, text=True, timeout=45, check=False)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError("Map saver timed out; graph may already exist") from exc
        if result.returncode != 0:
            raise RuntimeError("Map save failed; graph may already exist: " +
                               result.stdout + result.stderr)
        if not all(Path(str(prefix) + ext).is_file() for ext in (".yaml", ".pgm", ".mmpg")):
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
        grid = self.load_static_map(prefix)
        if load_graph:
            self.set_parameter("graph_path", str(prefix) + ".mmpg")
            self.graph("load")
        self.set_parameter("mode", "localization")
        self.manual = False
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
        self.stop()
        try:
            if key == "1":
                self.mapping()
            elif key == "2":
                if self.saved is None:
                    raise RuntimeError("Save a map with 3 or load one with 4 first")
                self.localize(self.saved, False)
            elif key == "3":
                self.save()
            elif key == "4":
                if not self.manual:
                    self.cancel_navigation()
                self.localize(self.prompt_prefix("Existing map name or full prefix: "), True)
        except (RuntimeError, ValueError, OSError) as exc:
            print("ERROR:", exc, flush=True)

    def run(self):
        print("W/S forward/back, A/D turn, SPACE stop; R/F linear speed; T/G turn speed")
        print("1 mapping/manual | 2 localization/navigation | 3 save | 4 load | Q quit")
        print("In navigation mode choose Nav2 Goal in RViz; key 1 cancels navigation first.")
        tty.setcbreak(sys.stdin.fileno())
        try:
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
