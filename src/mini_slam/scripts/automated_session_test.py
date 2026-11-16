#!/usr/bin/env python3
"""Isolated Gazebo acceptance test for mapping, localization and navigation.

Run after sourcing the ROS underlay and this workspace. This starts only its own
process groups, publishes motion only in the selected ROS domain / Gazebo
partition, and leaves its uniquely named map and logs in --output-dir.
"""
import argparse
import math
import os
from pathlib import Path
import pty
import re
import select
import signal
import subprocess
import time
import uuid

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


class Acceptance(Node):
    def __init__(self):
        super().__init__("mini_slam_acceptance")
        self.set_parameters([Parameter("use_sim_time", value=True)])
        transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
        sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan = self.odom = self.mapping = self.static = self.display = None
        self.map_seq = 0
        self.static_events = []
        self.subs = [
            self.create_subscription(LaserScan, "/scan", self.scan_cb, sensor),
            self.create_subscription(Odometry, "/odom", self.odom_cb, 10),
            self.create_subscription(OccupancyGrid, "/mini/map", self.map_cb, transient),
            self.create_subscription(
                OccupancyGrid, "/map", self.static_cb,
                QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)),
            self.create_subscription(OccupancyGrid, "/mini/display_map", self.display_cb, transient),
        ]
        self.velocity = self.create_publisher(Twist, "/cmd_vel", 10)
        self.initial = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.planner = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.navigator = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def scan_cb(self, msg):
        self.scan = msg

    def odom_cb(self, msg):
        self.odom = msg

    def map_cb(self, msg):
        self.mapping = msg
        self.map_seq += 1

    def static_cb(self, msg):
        self.static = msg
        self.static_events.append((msg.info.width, msg.info.height))

    def display_cb(self, msg):
        self.display = msg

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=max(0.0, min(0.1, end - time.monotonic())))

    def wait_for(self, predicate, timeout, description, processes=()):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            for proc in processes:
                if proc.poll() is not None:
                    raise RuntimeError(f"{description}: child exited {proc.returncode}")
            if predicate():
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(f"Timed out waiting for {description}")

    def call(self, service_type, name, request, timeout=20):
        client = self.create_client(service_type, name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f"Service unavailable: {name}")
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            if not future.done() or future.exception():
                raise RuntimeError(f"Service failed or timed out: {name}")
            return future.result()
        finally:
            self.destroy_client(client)

    def parameter(self, name, value):
        request = SetParameters.Request()
        request.parameters = [Parameter(name, value=value).to_parameter_msg()]
        result = self.call(SetParameters, "/mini_slam_node/set_parameters", request)
        if not result.results or not result.results[0].successful:
            raise RuntimeError(f"Parameter rejected: {name}: {result.results}")

    def map_pose(self):
        try:
            transform = self.tf.lookup_transform("map", "base_link", rclpy.time.Time())
            p, q = transform.transform.translation, transform.transform.rotation
            return p.x, p.y, math.atan2(2 * (q.w*q.z + q.x*q.y),
                                        1 - 2 * (q.y*q.y + q.z*q.z))
        except Exception:
            return None

    def map_odom(self):
        try:
            transform = self.tf.lookup_transform("map", "odom", rclpy.time.Time())
            p, q = transform.transform.translation, transform.transform.rotation
            return p.x, p.y, math.atan2(2 * (q.w*q.z + q.x*q.y),
                                        1 - 2 * (q.y*q.y + q.z*q.z))
        except Exception:
            return None

    def stop(self):
        for _ in range(5):
            self.velocity.publish(Twist())
            self.spin_for(0.08)

    def move_forward(self, duration):
        start = self.odom.pose.pose.position
        x, y = start.x, start.y
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if self.scan is None:
                raise RuntimeError("No scan while driving")
            front = []
            for i, distance in enumerate(self.scan.ranges):
                angle = self.scan.angle_min + i * self.scan.angle_increment
                if abs(math.atan2(math.sin(angle), math.cos(angle))) < 0.35:
                    if math.isfinite(distance) and distance > self.scan.range_min:
                        front.append(distance)
            if not front or min(front) < 0.50:
                raise RuntimeError("Front clearance unavailable or below 0.50 m")
            cmd = Twist()
            cmd.linear.x = 0.12
            self.velocity.publish(cmd)
            self.spin_for(0.08)
        self.stop()
        end_pos = self.odom.pose.pose.position
        return math.hypot(end_pos.x - x, end_pos.y - y)

    def action_result(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.exception():
            raise RuntimeError("Action request timed out")
        return future.result()

    def cancel(self, name):
        client = self.create_client(CancelGoal, name)
        try:
            if client.wait_for_service(timeout_sec=1):
                future = client.call_async(CancelGoal.Request())
                rclpy.spin_until_future_complete(self, future, timeout_sec=3)
        finally:
            self.destroy_client(client)

    def plan_goal(self, minimum=0.70, maximum=0.95, turn=0.0):
        grid = self.static
        x, y, yaw = self.map_pose()
        origin, res = grid.info.origin.position, grid.info.resolution
        candidates = []
        radius = max(1, int(0.22 / res))
        cells = int(maximum / res) + 2
        for dx in range(-cells, cells + 1, 2):
            for dy in range(-cells, cells + 1, 2):
                distance = math.hypot(dx * res, dy * res)
                if not minimum <= distance <= maximum:
                    continue
                px, py = x + dx * res, y + dy * res
                ix, iy = int((px - origin.x) / res), int((py - origin.y) / res)
                if (ix-radius < 0 or iy-radius < 0 or ix+radius >= grid.info.width
                        or iy+radius >= grid.info.height):
                    continue
                if all(grid.data[(iy+yy)*grid.info.width+ix+xx] == 0
                       for yy in range(-radius, radius+1)
                       for xx in range(-radius, radius+1)):
                    angle = math.atan2(dy, dx) - yaw - turn
                    candidates.append((abs(math.atan2(math.sin(angle), math.cos(angle))), px, py))
        candidates.sort()
        for _, px, py in candidates[:35]:
            req = ComputePathToPose.Goal()
            req.goal.header.frame_id = "map"
            req.goal.header.stamp = self.get_clock().now().to_msg()
            req.goal.pose.position.x, req.goal.pose.position.y = px, py
            req.goal.pose.orientation.z = math.sin(yaw / 2)
            req.goal.pose.orientation.w = math.cos(yaw / 2)
            req.planner_id = "GridBased"
            handle = self.action_result(self.planner.send_goal_async(req), 12)
            if not handle.accepted:
                continue
            result = self.action_result(handle.get_result_async(), 20)
            if result.status == 4 and result.result.path.poses:
                return px, py, yaw, len(result.result.path.poses)
        raise RuntimeError(f"No path among {len(candidates)} free candidates")


class ConsoleSession:
    def __init__(self, env, root):
        self.master, slave = pty.openpty()
        try:
            self.process = subprocess.Popen(
                ["python3", str(root / "src/mini_slam/scripts/mapping_console.py")],
                stdin=slave, stdout=slave, stderr=slave, env=env,
                start_new_session=True)
        finally:
            os.close(slave)
        self.output = ""

    def send(self, value):
        os.write(self.master, value.encode())

    def wait(self, node, text, timeout=45):
        start = len(self.output)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.process.poll() is not None:
                raise RuntimeError(f"Console exited {self.process.returncode}: {self.output}")
            if select.select([self.master], [], [], 0.1)[0]:
                self.output += os.read(self.master, 8192).decode(errors="replace")
                recent = self.output[start:]
                if text in recent:
                    return
                if "ERROR:" in recent or "Traceback" in recent:
                    raise RuntimeError("Console failed: " + recent)
            rclpy.spin_once(node, timeout_sec=0)
        raise RuntimeError(f"Console did not print {text}: {self.output[start:]}")

    def close(self):
        if self.process.poll() is None:
            try:
                self.send("q")
                self.process.wait(timeout=8)
            except (OSError, subprocess.TimeoutExpired):
                os.killpg(self.process.pid, signal.SIGTERM)
                self.process.wait(timeout=8)
        os.close(self.master)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain", type=int, default=191)
    parser.add_argument("--output-dir", default="/tmp")
    parser.add_argument("--stationary-seconds", type=float, default=60)
    parser.add_argument("--controller", default="FollowPath",
                        choices=["FollowPath", "PID", "Stanley", "DWB", "MPPI", "MPC"])
    parser.add_argument("--extended-navigation", action="store_true")
    args = parser.parse_args()
    if not 0 <= args.domain <= 232:
        parser.error("domain must be in [0, 232]")
    root = Path(__file__).resolve().parents[3]
    token = uuid.uuid4().hex[:10]
    out = Path(args.output_dir).expanduser().resolve() / ("mini_slam_accept_" + token)
    out.mkdir(parents=True, exist_ok=False)
    prefix = out / "map"
    env = os.environ.copy()
    env.update({
        "ROS_DOMAIN_ID": str(args.domain),
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "SUBNET",
        "GZ_PARTITION": "mini_slam_accept_" + token,
        "GZ_SIM_RESOURCE_PATH":
            "/opt/ros/jazzy/share/nav2_minimal_tb3_sim/models:/opt/ros/jazzy/share",
    })
    os.environ.update(env)
    launch = root / "src/mini_slam/ros/launch/mini_slam_simulation.launch.py"
    params = root / "src/mini_slam/ros/mini_slam_nav2_params.yaml"
    commands = [
        ["ros2", "launch", str(launch), "headless:=False", "use_rviz:=False",
         "use_visualizer:=False", "use_composition:=False", f"params_file:={params}"],
        ["ros2", "run", "mini_slam", "mini_slam_node", "--ros-args",
         "-p", "use_sim_time:=true", "-r", "/map:=/mini/map"],
    ]
    processes = []
    node = None
    console = None
    log = (out / "session.log").open("w")
    print(f"Artifacts: {out}", flush=True)
    try:
        for command in commands:
            processes.append(subprocess.Popen(command, env=env, stdout=log,
                                              stderr=subprocess.STDOUT,
                                              start_new_session=True))
        rclpy.init()
        node = Acceptance()
        selector = node.create_publisher(
            String, "/controller_selector",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        selector.publish(String(data=args.controller))
        print(f"Selected controller: {args.controller}", flush=True)
        node.wait_for(lambda: node.scan is not None and node.odom is not None
                      and node.mapping is not None and node.map_pose() is not None,
                      100, "mapping scan, odom, map and TF", processes)
        print("Mapping data ready", flush=True)
        moved = node.move_forward(3.0)
        print(f"Manual mapping motion: {moved:.3f} m", flush=True)
        if moved < 0.1:
            raise RuntimeError("Robot did not move during mapping")
        previous = node.map_seq
        node.wait_for(lambda: node.map_seq > previous, 20, "fresh map")
        before = node.map_pose()
        console = ConsoleSession(env, root)
        console.wait(node, "Q quit", 20)
        console.send("3")
        console.wait(node, "Map name")
        console.send(str(prefix) + chr(10))
        console.wait(node, "Saved graph and map:", 70)
        for ext in (".mmpg", ".yaml", ".pgm"):
            if not Path(str(prefix) + ext).is_file():
                raise RuntimeError(f"Missing snapshot file: {prefix}{ext}")
        # 已有成套文件时必须拒绝再次保存，避免静默覆盖用户地图。
        again = node.call(Trigger, "/save_snapshot", Trigger.Request(), 60)
        if again.success:
            raise RuntimeError("Snapshot unexpectedly overwrote existing files")

        # 栅格可加载但位姿图损坏时，终端必须恢复建图模式而非继续写坏图。
        bad = out / "invalid"
        Path(str(bad) + ".mmpg").write_bytes(b"not a pose graph")
        Path(str(bad) + ".pgm").write_bytes(
            b"P5" + bytes([10]) + b"2 2" + bytes([10]) + b"255" + bytes([10])
            + bytes([0, 254, 205, 254]))
        Path(str(bad) + ".yaml").write_text(
            "image: invalid.pgm" + chr(10) +
            "resolution: 0.05" + chr(10) +
            "origin: [0.0, 0.0, 0.0]" + chr(10) +
            "negate: 0" + chr(10) +
            "occupied_thresh: 0.65" + chr(10) +
            "free_thresh: 0.196" + chr(10))
        event_start = len(node.static_events)
        console.send("4")
        console.wait(node, "Existing map name")
        console.send(str(bad) + chr(10))
        console.wait(node, "ERROR:", 45)
        request = GetParameters.Request()
        request.names = ["mode"]
        status = node.call(GetParameters, "/mini_slam_node/get_parameters", request)
        if not status.values or status.values[0].string_value != "mapping":
            raise RuntimeError("Failed graph load did not restore mapping mode")
        node.wait_for(lambda: (2, 2) in node.static_events[event_start:]
                      and node.static is not None
                      and (node.static.info.width, node.static.info.height) != (2, 2),
                      15, "previous static map after failed mapping load")
        print("Invalid graph rejected; mapping mode and previous map restored", flush=True)
        console.send("2")
        console.wait(node, "Localization/navigation mode", 45)
        node.wait_for(lambda: node.static is not None and node.display is not None
                      and node.static.data == node.display.data, 20,
                      "matching Nav2 and displayed maps")
        node.spin_for(args.stationary_seconds)
        after = node.map_pose()
        drift = math.hypot(after[0] - before[0], after[1] - before[1])
        yaw_drift = abs(math.atan2(math.sin(after[2] - before[2]),
                                  math.cos(after[2] - before[2])))
        print(f"Stationary localization drift: {drift:.3f} m, {yaw_drift:.3f} rad",
              flush=True)
        if drift > 0.30 or yaw_drift > 0.35:
            raise RuntimeError("Localization jumped after mapping-to-localization")
        if not node.planner.wait_for_server(timeout_sec=30):
            raise RuntimeError("Planner action unavailable")
        if not node.navigator.wait_for_server(timeout_sec=30):
            raise RuntimeError("Navigation action unavailable")
        node.wait_for(lambda: node.static is not None, 15, "loaded static map")
        px, py, yaw, count = node.plan_goal()
        print(f"A* path: {count} poses; goal ({px:.2f}, {py:.2f})", flush=True)
        req = NavigateToPose.Goal()
        req.pose.header.frame_id = "map"
        req.pose.header.stamp = node.get_clock().now().to_msg()
        req.pose.pose.position.x, req.pose.pose.position.y = px, py
        req.pose.pose.orientation.z = math.sin(yaw / 2)
        req.pose.pose.orientation.w = math.cos(yaw / 2)
        start = node.odom.pose.pose.position
        sx, sy = start.x, start.y
        handle = node.action_result(node.navigator.send_goal_async(req), 15)
        if not handle.accepted:
            raise RuntimeError("Navigation goal rejected")
        result = node.action_result(handle.get_result_async(), 100)
        node.spin_for(1)
        end = node.odom.pose.pose.position
        distance = math.hypot(end.x - sx, end.y - sy)
        print(f"Navigation status: {result.status}; motion: {distance:.3f} m", flush=True)
        if result.status != 4 or distance < 0.15:
            raise RuntimeError("Navigation did not complete with sufficient motion")

        if args.extended_navigation:
            for turn in (1.0, -1.0):
                px, py, yaw, count = node.plan_goal(1.2, 1.6, turn)
                req = NavigateToPose.Goal()
                req.pose.header.frame_id = "map"
                req.pose.header.stamp = node.get_clock().now().to_msg()
                req.pose.pose.position.x, req.pose.pose.position.y = px, py
                req.pose.pose.orientation.z = math.sin(yaw / 2)
                req.pose.pose.orientation.w = math.cos(yaw / 2)
                begin = node.map_pose()
                handle = node.action_result(node.navigator.send_goal_async(req), 15)
                if not handle.accepted:
                    raise RuntimeError("Extended navigation rejected")
                result = node.action_result(handle.get_result_async(), 100)
                end = node.map_pose()
                travel = math.hypot(end[0] - begin[0], end[1] - begin[1])
                print(f"Extended turn {turn:+.1f}: {count} path poses; "
                      f"status {result.status}; displacement {travel:.3f} m", flush=True)
                if result.status != 4 or travel < 0.8:
                    raise RuntimeError("Extended turning navigation failed")

        # 定位模式加载损坏图失败时，仍须恢复上一张导航地图和图路径。
        expected_grid = node.static
        event_start = len(node.static_events)
        console.send("4")
        console.wait(node, "Existing map name")
        console.send(str(bad) + chr(10))
        console.wait(node, "ERROR:", 45)
        node.wait_for(lambda: (2, 2) in node.static_events[event_start:]
                      and node.static is not None and node.static.data == expected_grid.data,
                      15, "previous static map after failed localization load")
        request = GetParameters.Request()
        request.names = ["mode", "graph_path"]
        status = node.call(GetParameters, "/mini_slam_node/get_parameters", request)
        if (len(status.values) != 2 or
                status.values[0].string_value != "localization" or
                status.values[1].string_value != str(prefix) + ".mmpg"):
            raise RuntimeError("Failed graph load did not restore localization state")
        print("Invalid graph rejected; previous localization restored", flush=True)

        # 加载已保存的位姿图会清空旧 TF 与跟踪状态。以当前已知位姿
        # 重新初始化，验证重载后定位保持稳定且不会复用旧图的状态。
        known_pose = node.map_pose()
        console.send("4")
        console.wait(node, "Existing map name")
        console.send(str(prefix) + chr(10))
        console.wait(node, "Localization/navigation mode", 45)
        node.spin_for(1)
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = "map"
        initial.header.stamp = node.get_clock().now().to_msg()
        initial.pose.pose.position.x, initial.pose.pose.position.y = known_pose[:2]
        initial.pose.pose.orientation.z = math.sin(known_pose[2] / 2)
        initial.pose.pose.orientation.w = math.cos(known_pose[2] / 2)
        for _ in range(3):
            node.initial.publish(initial)
            node.spin_for(0.2)
        node.spin_for(10)
        reloaded_pose = node.map_pose()
        reload_drift = math.hypot(reloaded_pose[0] - known_pose[0],
                                  reloaded_pose[1] - known_pose[1])
        print(f"Graph reload localization drift: {reload_drift:.3f} m", flush=True)
        if reload_drift > 0.30:
            raise RuntimeError("Localization drifted after graph reload")
        px, py, yaw, count = node.plan_goal()
        print(f"Reloaded graph A* path: {count} poses; goal ({px:.2f}, {py:.2f})",
              flush=True)
        req = NavigateToPose.Goal()
        req.pose.header.frame_id = "map"
        req.pose.header.stamp = node.get_clock().now().to_msg()
        req.pose.pose.position.x, req.pose.pose.position.y = px, py
        req.pose.pose.orientation.z = math.sin(yaw / 2)
        req.pose.pose.orientation.w = math.cos(yaw / 2)
        start = node.odom.pose.pose.position
        sx, sy = start.x, start.y
        handle = node.action_result(node.navigator.send_goal_async(req), 15)
        if not handle.accepted:
            raise RuntimeError("Navigation goal after reload was rejected")
        result = node.action_result(handle.get_result_async(), 100)
        node.spin_for(1)
        end = node.odom.pose.pose.position
        distance = math.hypot(end.x - sx, end.y - sy)
        print(f"Reloaded graph navigation status: {result.status}; "
              f"motion: {distance:.3f} m", flush=True)
        if result.status != 4 or distance < 0.15:
            raise RuntimeError("Navigation after reload did not complete with motion")
        # 行驶中切回手动建图：必须先取消 NavigateToPose 和 FollowPath，
        # 并等目标进入终态，才能允许终端发送手动速度。
        node.stop()
        node.spin_for(1)
        px, py, yaw, count = node.plan_goal()
        req = NavigateToPose.Goal()
        req.pose.header.frame_id = "map"
        req.pose.header.stamp = node.get_clock().now().to_msg()
        req.pose.pose.position.x, req.pose.pose.position.y = px, py
        req.pose.pose.orientation.z = math.sin(yaw / 2)
        req.pose.pose.orientation.w = math.cos(yaw / 2)
        start = node.odom.pose.pose.position
        sx, sy = start.x, start.y
        moving_goal = node.action_result(node.navigator.send_goal_async(req), 15)
        if not moving_goal.accepted:
            raise RuntimeError("Navigation goal for cancellation was rejected")
        node.wait_for(
            lambda: math.hypot(node.odom.pose.pose.position.x - sx,
                               node.odom.pose.pose.position.y - sy) > 0.05,
            25, "navigation motion before manual takeover")
        before_mapping = node.map_odom()
        previous = node.map_seq
        console.send("1")
        console.wait(node, "Mapping/manual mode", 45)
        canceled = node.action_result(moving_goal.get_result_async(), 15)
        if canceled.status != 5:
            raise RuntimeError(f"Navigation was not canceled: {canceled.status}")
        stopped = node.odom.pose.pose.position
        stop_x, stop_y = stopped.x, stopped.y
        node.spin_for(1.5)
        stopped = node.odom.pose.pose.position
        residual = math.hypot(stopped.x - stop_x, stopped.y - stop_y)
        print(f"Manual takeover: canceled; residual motion {residual:.3f} m", flush=True)
        if residual > 0.08:
            raise RuntimeError("Robot kept moving after manual takeover")
        node.wait_for(lambda: node.map_seq > previous, 20, "mapping after localization")
        node.spin_for(2)
        resumed = node.map_odom()
        resume_jump = math.hypot(resumed[0] - before_mapping[0],
                                 resumed[1] - before_mapping[1])
        print(f"Localization-to-mapping map->odom change: {resume_jump:.3f} m", flush=True)
        if resume_jump > 0.30:
            raise RuntimeError("Pose jumped while returning to mapping")

        def graph_nodes():
            stats = node.call(Trigger, "/get_stats", Trigger.Request())
            match = re.search(r"节点=([0-9]+)", stats.message)
            if not stats.success or not match:
                raise RuntimeError("Cannot read graph node count")
            return int(match.group(1))

        nodes_before = graph_nodes()
        end_time = time.monotonic() + 2.5
        while time.monotonic() < end_time:
            console.send("a")
            node.spin_for(0.08)
        console.send(" ")
        node.spin_for(3)
        nodes_after = graph_nodes()
        print(f"Resumed mapping nodes: {nodes_before} -> {nodes_after}", flush=True)
        if nodes_after <= nodes_before:
            raise RuntimeError("Resumed mapping did not add a scan node")
        continued = out / "continued"
        console.send("3")
        console.wait(node, "Map name")
        console.send(str(continued) + chr(10))
        console.wait(node, "Saved graph and map:", 70)
        for ext in (".mmpg", ".yaml", ".pgm"):
            if not Path(str(continued) + ext).is_file():
                raise RuntimeError("Resumed mapping snapshot is incomplete")
        print("PASS mapping -> localization -> navigation -> graph reload -> "
              "navigation -> resumed mapping and snapshot", flush=True)
    finally:
        if console is not None:
            try:
                console.close()
            finally:
                (out / "console.log").write_text(console.output)
        if node is not None:
            for name in ("/navigate_to_pose/_action/cancel_goal",
                         "/follow_path/_action/cancel_goal"):
                try:
                    node.cancel(name)
                except Exception as exc:
                    print(f"Cancel {name}: {exc}", flush=True)
            node.stop()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        for proc in processes:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGTERM)
        for proc in processes:
            try:
                proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                print(f"Child {proc.pid} did not exit; inspect manually", flush=True)
        log.close()


if __name__ == "__main__":
    main()
