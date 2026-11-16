"""mini_slam 在线节点。

用法（数据源自行准备，例如 TB3 仿真或 bag 回放）：
    ros2 launch mini_slam mini_slam_online.launch.py
参数可在命令行覆盖：
    ros2 launch mini_slam mini_slam_online.launch.py resolution:=0.05
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # (name, default, python type) —— 显式类型，避免 launch 把 "false" 当字符串。
    specs = [
        ("use_sim_time", "false", bool),
        ("mode", "mapping", str),
        ("graph_path", "", str),
        ("resolution", "0.05", float),
        ("minimum_travel_distance", "0.5", float),
        ("minimum_travel_heading", "0.5", float),
        ("map_update_interval", "5.0", float),
        ("transform_publish_period", "0.05", float),
        ("scan_topic", "scan", str),
        ("odom_topic", "odom", str),
        ("map_frame", "map", str),
        ("odom_frame", "odom", str),
        ("base_frame", "base_footprint", str),
        ("scan_range_max", "12.0", float),
        ("loop_search_maximum_distance", "3.0", float),
        ("loop_match_minimum_chain_size", "10", int),
        ("loop_match_minimum_response_coarse", "0.40", float),
        ("loop_match_minimum_response_fine", "0.50", float),
        ("sparsify_enabled", "false", bool),
        ("sparsify_every", "50", int),
        ("sparsify_keep_radius", "0.8", float),
        ("sparsify_min_keep_recent", "50", int),
        ("localization_window", "10.0", float),
        ("processing_queue_size", "10", int),
        ("map_output_prefix", "", str),
        ("enable_interactive_markers", "false", bool),
        ("interactive_marker_max_vertices", "200", int),
    ]

    args = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default, _ in specs
    ]
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=typ)
        for name, _, typ in specs
    }

    node = Node(
        package="mini_slam",
        executable="mini_slam_node",
        name="mini_slam_node",
        output="screen",
        parameters=[parameters],
    )
    return LaunchDescription(args + [node])
