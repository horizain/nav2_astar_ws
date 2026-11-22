# 端到端演示/集成测试：仿真源 + 定位节点（composed 匹配 + 全局冷启动）
#
#   ros2 launch scan_localizer demo.launch.py
#   验证：ros2 topic echo /localizer/pose --field pose.pose.position
#         ros2 topic echo /localizer/status
#   （仿真机器人从 (2,13.5) 起步跑 ~106m 矩形环路，含人群遮挡段；
#     前几帧即自动完成 B&B 全局冷启动定位）
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    use_composition = LaunchConfiguration("use_composition")
    use_imu = LaunchConfiguration("use_imu")

    declare_args = [
        DeclareLaunchArgument(
            "use_composition", default_value="true",
            description="组合容器（嵌入式必开，省 DDS 进程间开销）"),
        DeclareLaunchArgument(
            "use_imu", default_value="true",
            description="陀螺航向融合（仿真源发布真实 IMU）"),
    ]

    sim = Node(
        package="scan_localizer",
        executable="scan_localizer_sim",
        name="scan_localizer_sim",
        output="screen",
    )

    localizer = ComposableNode(
        package="scan_localizer",
        plugin="sloc::LocalizerNode",
        name="scan_localizer",
        namespace="",
        remappings=[],
        parameters=[
            # 演示用：仿真 /scan 为 720 束满频；生产嵌入式中用 subsample
            {"scan_subsample": 1},
            {"matcher": "composed"},
            # 仿真源发布 /imu（真实角速度+噪声）
            {"use_imu": use_imu},
        ],
    )

    container = ComposableNodeContainer(
        name="localization_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[localizer],
        output="screen",
    )

    return launch.LaunchDescription(
        declare_args
        + [
            sim,
            # 演示默认组合容器；--use-composition:=false 时退化为独立进程
            container,
        ]
    )
