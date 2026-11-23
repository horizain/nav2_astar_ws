# Copyright 2026 ruijianding
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# 端到端演示/集成测试：仿真源 + 定位节点（composed 匹配 + 全局冷启动）
#
#   ros2 launch scan_localizer demo.launch.py
#   ros2 launch scan_localizer demo.launch.py use_composition:=false
#   验证：ros2 topic echo /localizer/pose --field pose.pose.position
#         ros2 topic echo /localizer/status
#   （仿真机器人从 (2,13.5) 起步跑 ~106m 矩形环路，含人群遮挡段；
#     前几帧即自动完成 B&B 全局冷启动定位）
import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_composition = LaunchConfiguration('use_composition')
    use_imu = LaunchConfiguration('use_imu')
    params_file = LaunchConfiguration('params_file')

    default_params = PathJoinSubstitution([
        FindPackageShare('scan_localizer'), 'config', 'localizer.yaml'
    ])

    declare_args = [
        DeclareLaunchArgument(
            'use_composition', default_value='true',
            description='组合容器（嵌入式必开，省 DDS 进程间开销；传 use_composition:=false 切独立进程）'),
        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='陀螺航向融合（仿真源发布真实 IMU）'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='定位节点 YAML 参数文件路径'),
    ]

    demo_overrides = {
        'autostart': True,
        'scan_subsample': 1,
        'matcher': 'composed',
        'use_imu': use_imu,
    }

    sim = Node(
        package='scan_localizer',
        executable='scan_localizer_sim',
        name='scan_localizer_sim',
        output='screen',
    )

    composable_localizer = ComposableNode(
        package='scan_localizer',
        plugin='sloc::LocalizerNode',
        name='scan_localizer',
        namespace='',
        parameters=[params_file, demo_overrides],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    container = ComposableNodeContainer(
        name='localization_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[composable_localizer],
        output='screen',
        condition=IfCondition(use_composition),
    )

    standalone_localizer = Node(
        package='scan_localizer',
        executable='scan_localizer_node',
        name='scan_localizer',
        namespace='',
        parameters=[params_file, demo_overrides],
        output='screen',
        condition=UnlessCondition(use_composition),
    )

    return launch.LaunchDescription(
        declare_args
        + [
            sim,
            container,
            standalone_localizer,
        ]
    )
