#!/usr/bin/env python3
"""Run one fixed teaching scenario using mini_slam, our A*, and our controller.

Each repetition owns a ROS domain/Gazebo partition and a unique output directory.
No global process cleanup, map overwrites, automatic target substitution, or
safety-filter disabling. Failures remain in the data and exit status.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/mini_slam/scripts"))
from automated_session_test import Acceptance, ConsoleSession
from navigation_metrics import write_report
from owned_gui import close_gazebo_gui
from teaching_scenarios import SCENARIOS, truth_robot, world_sdf

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
import yaml

RUNNER_SOURCE_SHA256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def record_truth(node, message):
    p, q = message.pose.pose.position, message.pose.pose.orientation
    node.events.append(dict(
        kind="truth", t=message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
        frame_id=message.header.frame_id, source="gazebo_model_world_pose",
        x=p.x, y=p.y,
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))


def yaw_of_odom(node):
    q = node.odom.pose.pose.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def map_rotation(node, processes):
    previous = yaw_of_odom(node)
    accumulated = 0.0
    deadline = time.monotonic() + 60
    while accumulated < 2 * math.pi:
        if time.monotonic() > deadline:
            raise RuntimeError("Mapping rotation did not complete")
        for process in processes:
            if process.poll() is not None:
                raise RuntimeError("Simulation exited during mapping")
        command = Twist()
        command.angular.z = 0.45
        node.velocity.publish(command)
        node.spin_for(0.1)
        current = yaw_of_odom(node)
        accumulated += math.atan2(math.sin(current - previous), math.cos(current - previous))
        previous = current
    node.stop()
    node.spin_for(2)
    print(f"Mapping rotation complete: {accumulated:.3f} rad", flush=True)


def goal_request(node, target):
    request = NavigateToPose.Goal()
    request.pose.header.frame_id = "map"
    request.pose.header.stamp = node.get_clock().now().to_msg()
    request.pose.pose.position.x, request.pose.pose.position.y = target[:2]
    request.pose.pose.orientation.z = math.sin(target[2] / 2)
    request.pose.pose.orientation.w = math.cos(target[2] / 2)
    return request


def run_episode(args, repetition):
    token = uuid.uuid4().hex[:10]
    output = args.output / f"{args.scenario}_{args.controller}_{token}"
    output.mkdir(parents=True, exist_ok=False)
    definition = SCENARIOS[args.scenario]
    print(f"Artifacts: {output}", flush=True)
    environment = os.environ.copy()
    environment.update(ROS_DOMAIN_ID=str(args.domain), RMW_IMPLEMENTATION="rmw_cyclonedds_cpp",
                       ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET", GZ_PARTITION="teaching_" + token,
                       GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share/nav2_minimal_tb3_sim/models:/opt/ros/jazzy/share")
    os.environ.update(environment)
    world = output / "world.sdf"
    world.write_text(world_sdf(args.scenario))
    robot = output / "robot.sdf.xacro"
    sim_dir = Path(get_package_share_directory("nav2_minimal_tb3_sim"))
    robot.write_text(truth_robot((sim_dir / "urdf/gz_waffle.sdf.xacro").read_text()))
    config = yaml.safe_load((ROOT / "src/mini_slam/ros/mini_slam_nav2_params.yaml").read_text())
    controller = config["controller_server"]["ros__parameters"]
    # A fixed experiment policy, recorded in the saved params. Never plan through unknown.
    config["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"] = False
    if args.xy_tolerance is not None:
        controller["general_goal_checker"]["xy_goal_tolerance"] = args.xy_tolerance
        controller[args.controller]["goal_tolerance"] = 0.8 * args.xy_tolerance
    if args.yaw_tolerance is not None:
        controller["general_goal_checker"]["yaw_goal_tolerance"] = args.yaw_tolerance
    params = output / "params.yaml"
    params.write_text(yaml.safe_dump(config, allow_unicode=True))
    launch = ROOT / "src/mini_slam/ros/launch/mini_slam_simulation.launch.py"
    commands = [
        ["ros2", "launch", str(launch), f"headless:={not args.gui}",
         f"use_rviz:={args.gui}", "use_visualizer:=True", "use_composition:=False",
         f"rviz_config_file:={ROOT / 'src/mini_slam/ros/mini_slam_mapping.rviz'}",
         f"params_file:={params}", f"world:={world}", f"robot_sdf:={robot}",
         "x_pose:=0.0", "y_pose:=0.0", "yaw:=0.0"],
        [str(Path(get_package_prefix("mini_slam")) / "lib/mini_slam/mini_slam_node"), "--ros-args",
         "-p", "use_sim_time:=true", "-r", "/map:=/mini/map"],
        [str(Path(get_package_prefix("ros_gz_bridge")) / "lib/ros_gz_bridge/parameter_bridge"),
         "/teaching/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry",
         "--ros-args", "-p", "use_sim_time:=true"],
    ]
    binary_hashes = {}
    for package in ("nav2_astar_planner", "nav2_controller_lab"):
        library = Path(get_package_prefix(package)) / "lib" / f"lib{package}.so"
        binary_hashes[package] = hashlib.sha256(library.read_bytes()).hexdigest()
    processes, node, console = [], None, None
    log = (output / "session.log").open("w")
    passed, error, cleanup_errors = False, "", []
    try:
        for index, command in enumerate(commands):
            destination = log if index == 0 else (output / ("slam.log" if index == 1 else "truth_bridge.log")).open("w")
            try:
                processes.append(subprocess.Popen(command, env=environment, stdout=destination,
                                                  stderr=subprocess.STDOUT, start_new_session=True))
            finally:
                if destination is not log:
                    destination.close()
        rclpy.init()
        node = Acceptance()
        node.requested_controller = args.controller
        node.subs.append(node.create_subscription(
            Odometry, "/teaching/ground_truth", lambda message: record_truth(node, message), 100))
        selector = node.create_publisher(
            String, "/controller_selector",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        selector.publish(String(data=args.controller))
        node.wait_for(lambda: node.scan is not None and node.odom is not None
                      and node.mapping is not None and node.map_pose() is not None
                      and any(e["kind"] == "truth" for e in node.events),
                      100, "scan/odom/map/TF/independent world pose", processes)
        map_rotation(node, processes)
        prefix = output / "map"
        console = ConsoleSession(environment, ROOT)
        console.wait(node, "Console ready", 20)
        console.send("3")
        console.wait(node, "Map name")
        console.send(str(prefix) + chr(10))
        console.wait(node, "Saved graph and map:", 70)
        console.send("2")
        console.wait(node, "Localization/navigation mode", 45)
        node.spin_for(2)
        if not node.navigator.wait_for_server(timeout_sec=30):
            raise RuntimeError("Navigator unavailable")
        results = []
        for index, target in enumerate(definition["goals"]):
            cancel_test = args.scenario == "cancel_replan" and index == 0
            purpose = "cancel_test" if cancel_test else "reach_goal"
            request = goal_request(node, target)
            begin = node.odom.pose.pose.position
            start_x, start_y = begin.x, begin.y
            handle = node.action_result(node.start_navigation(request, purpose=purpose), 15)
            if not handle.accepted:
                status = "rejected"
                node.events.append(dict(kind="result", t=node.sim_time(),
                                        goal_id=node.current_goal_id, status=status))
            else:
                if cancel_test:
                    node.wait_for(lambda: math.hypot(
                        node.odom.pose.pose.position.x-start_x,
                        node.odom.pose.pose.position.y-start_y) > 0.15,
                        args.timeout, "motion before cancellation", processes)
                    console.send("1")
                    console.wait(node, "Mapping/manual mode", 45)
                    result = node.action_result(handle.get_result_async(), 15)
                    console.send("2")
                    console.wait(node, "Localization/navigation mode", 45)
                else:
                    result = node.action_result(handle.get_result_async(), args.timeout)
                status = {4: "succeeded", 5: "canceled", 6: "aborted"}.get(result.status, "unknown")
            if status == "succeeded":
                matching_paths = [e for e in node.events if e["kind"] == "path"
                                  and e["frame_id"] == "map" and e.get("points")]
                if not matching_paths:
                    raise RuntimeError("No actual path observed for successful goal")
                endpoint = matching_paths[-1]["points"][-1]
                if math.hypot(endpoint[0] - target[0], endpoint[1] - target[1]) > 1e-6:
                    raise RuntimeError("Planner replaced the continuous requested goal")
            results.append(status)
            print(f"Fixed target {index+1} {target}: {status}", flush=True)
            node.spin_for(0.5)
            if status not in ("succeeded", "canceled"):
                break
        expected = definition["expected"]
        if expected == "aborted_or_rejected":
            passed = bool(results) and results[0] in ("aborted", "rejected")
        elif expected == "canceled_then_succeeded":
            passed = results == ["canceled", "succeeded"]
        else:
            passed = results == ["succeeded"] * len(definition["goals"])
        if passed and expected != "aborted_or_rejected":
            actual = {e.get("controller") for e in node.control_events}
            if args.controller not in actual:
                raise RuntimeError("Requested controller was not observed computing")
            if args.controller == "MPC" and node.prediction_samples == 0:
                raise RuntimeError("Selected MPC prediction was not observed")
        if not passed:
            error = f"Expected {expected}; got {results}"
        if args.hold_seconds and passed:
            print(f"Holding visualization for {args.hold_seconds} seconds", flush=True)
            node.spin_for(args.hold_seconds)
    except Exception as exc:
        error = str(exc)
    finally:
        def cleanup(label, action):
            try:
                action()
            except Exception as exc:
                cleanup_errors.append(f"{label}: {exc}")
        if console is not None:
            cleanup("console", console.close)
            cleanup("console log", lambda: (output / "console.log").write_text(console.output))
        if node is not None:
            for name in ("/navigate_to_pose/_action/cancel_goal", "/follow_path/_action/cancel_goal"):
                cleanup(name, lambda service=name: node.cancel(service))
            cleanup("stop", node.stop)
            cleanup("events", lambda: (output / "events.jsonl").write_text(
                "".join(json.dumps(e, ensure_ascii=False, allow_nan=False) + "\n" for e in node.events)))
            revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT,
                                      capture_output=True, text=True).stdout.strip()
            dirty = bool(subprocess.run(["git", "status", "--porcelain"], cwd=ROOT,
                                        capture_output=True, text=True).stdout)
            metadata = dict(
                scenario=args.scenario, repetition=repetition, fixed_targets=definition["goals"],
                expected=definition["expected"], controller_requested=args.controller,
                revision=revision, worktree_dirty=dirty, binary_sha256=binary_hashes,
                workload_note="development desktop; external CPU/GPU workloads not controlled",
                world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
                params_sha256=hashlib.sha256(params.read_bytes()).hexdigest(),
                source_runner_sha256=RUNNER_SOURCE_SHA256,
                spawn_world=[0, 0, 0], map_world_alignment=[0, 0, 0],
                alignment_source="known_spawn_at_world_origin_and_mini_slam_initial_map_origin",
                truth_source="gazebo_model_world_pose", independent_truth_required=True,
                safety_layers="CBF and collision_monitor enabled",
                noise_seed=None, hold_seconds=args.hold_seconds,
                note="fixed geometry and targets; sensor/physics variation remains",
                functional_expectation_passed=passed, error=error,
                xy_tolerance=controller["general_goal_checker"]["xy_goal_tolerance"],
                yaw_tolerance=controller["general_goal_checker"]["yaw_goal_tolerance"])
            cleanup("metrics", lambda: write_report(node.events, output / "metrics",
                                                     metadata=metadata, alignment=(0, 0, 0)))
            cleanup("node", node.destroy_node)
        if rclpy.ok():
            cleanup("rclpy", rclpy.shutdown)
        if args.gui:
            cleanup("owned Gazebo GUI close", lambda: close_gazebo_gui(environment["GZ_PARTITION"]))
        for child in processes:
            if child.poll() is None:
                cleanup(f"interrupt {child.pid}", lambda process=child:
                        process.send_signal(signal.SIGINT))
        for child in processes:
            try:
                child.wait(timeout=25)
            except subprocess.TimeoutExpired:
                cleanup_errors.append(f"forced cleanup required for {child.pid}")
                cleanup(f"terminate owned {child.pid}", lambda process=child:
                        os.killpg(process.pid, signal.SIGTERM))
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    cleanup(f"kill owned {child.pid}", lambda process=child:
                            os.killpg(process.pid, signal.SIGKILL))
                    cleanup(f"reap {child.pid}", lambda process=child: process.wait(timeout=5))
            if child.returncode not in (0, -signal.SIGINT, -signal.SIGTERM, 130, 143):
                cleanup_errors.append(f"abnormal process exit {child.pid}: {child.returncode}")
        cleanup("process exits", lambda: (output / "process_exits.json").write_text(
            json.dumps([dict(command=command, returncode=child.returncode)
                        for command, child in zip(commands, processes)], indent=2)))
        log.close()
        (output / "lesson_result.json").write_text(json.dumps(
            dict(passed=passed and not cleanup_errors, error=error, cleanup_errors=cleanup_errors),
            ensure_ascii=False, indent=2))
    print(("PASS" if passed and not cleanup_errors else "FAIL") + f" {args.scenario}: {error}", flush=True)
    for message in cleanup_errors:
        print(f"Cleanup: {message}", flush=True)
    return passed and not cleanup_errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=SCENARIOS, default="straight")
    parser.add_argument("--controller", choices=["FollowPath", "PID", "Stanley", "DWB", "MPPI", "MPC"],
                        default="FollowPath")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--domain", type=int, default=203)
    parser.add_argument("--output", type=Path, default=Path("/tmp/navigation_lessons"))
    parser.add_argument("--timeout", type=float, default=90)
    parser.add_argument("--xy-tolerance", type=float)
    parser.add_argument("--yaw-tolerance", type=float)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--hold-seconds", type=float, default=0)
    args = parser.parse_args()
    if not 0 <= args.domain <= 232 or args.repeat < 1 or args.timeout <= 0:
        parser.error("Invalid domain, repetition count or timeout")
    for value in (args.xy_tolerance, args.yaw_tolerance):
        if value is not None and (not math.isfinite(value) or value <= 0):
            parser.error("Tolerances must be finite and positive")
    results = [run_episode(args, i + 1) for i in range(args.repeat)]
    print(f"Functional expectations met: {sum(results)}/{len(results)}", flush=True)
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
